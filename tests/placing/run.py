#!/usr/bin/env python3
"""CTest synthesis and placement driver; artifacts stay in the binary tree."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import time


def yosys_quote(path):
    return '"' + str(path).replace('\\', '/').replace('"', '\\"') + '"'


def constraint_tcl(spec):
    lines = ['# Apply after loading this design, before Estimate/placement.']
    for name, period in spec['clocks'].items():
        lines.append(f'create_clock -name {name} -period {period:g} [get_ports {name}]')
    for limit in spec['max_delays']:
        lines.append(f'set_max_delay -datapath_only -from [get_clocks {limit["source"]}] '
                     f'-to [get_clocks {limit["capture"]}] {limit["ns"]:g}')
    return '\n'.join(lines)+'\n'


def synthesize(yosys, source, work):
    work.mkdir(parents=True, exist_ok=True)
    script = work / 'synth.ys'
    netlist = work / 'netlist.json'
    script.write_text(f'read_verilog -sv {yosys_quote(source.resolve())}\n'
                      'synth -top top -flatten -noabc\n'
                      'abc -lut 4\ndelete t:$scopeinfo\nclean\ncheck -assert\n'
                      f'write_json {yosys_quote(netlist.resolve())}\nstat\n')
    with (work / 'synthesis.log').open('w') as log:
        subprocess.run([yosys, '-T', '-s', str(script.resolve())], cwd=work,
                       stdout=log, stderr=subprocess.STDOUT, check=True, timeout=600)
    module = json.loads(netlist.read_text())['modules']['top']
    types = Counter(c['type'] for c in module['cells'].values())
    unsupported = [t for t in types if t != '$lut' and not t.startswith(('$_DFF_', '$_DFFE_', '$_SDFF_', '$_SDFFE_', '$_SDFFCE_'))]
    if unsupported:
        raise RuntimeError(f'Unsupported generic primitive(s): {unsupported}')
    return dict(cells=sum(types.values()), types=dict(sorted(types.items())),
                yosys=subprocess.check_output([yosys, '-V'], text=True).strip())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--yosys', required=True)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--work', type=Path, required=True)
    p.add_argument('--placer', required=True)
    args = p.parse_args()
    args.work = args.work.resolve()
    args.work.mkdir(parents=True, exist_ok=True)
    (args.work/'metrics.json').unlink(missing_ok=True)
    spec = json.loads((args.source.parent/'manifest.json').read_text())[args.source.stem]
    tier = args.source.parent.parent if args.source.parent.name == '2clk' else args.source.parent
    target = int(tier.name.removesuffix('k'))*1000
    if spec['target'] != target:
        raise RuntimeError('Manifest target does not match the size directory')
    if args.source.parent.name == '2clk':
        clocks = spec['constraints']['clocks']
        if len(clocks) != 2 or len(set(clocks.values())) != 2:
            raise RuntimeError('Two-clock tests require two declared, distinct periods')
    started = time.monotonic()
    metadata = synthesize(args.yosys, args.source, args.work)
    count = metadata['cells']
    if not .90*spec['target'] <= count <= 1.10*spec['target']:
        raise RuntimeError(f'Synthesized {count} cells; expected {spec["target"]} +/-10%')
    if not metadata['types'].get('$lut') or len(metadata['types']) < 2:
        raise RuntimeError('Expected a mix of combinational and sequential cells')
    metadata['synthesis_seconds'] = time.monotonic()-started
    metadata['source_sha256'] = hashlib.sha256(args.source.read_bytes()).hexdigest()
    metadata['placer_sha256'] = hashlib.sha256(Path(args.placer).read_bytes()).hexdigest()
    constraint_arg = str(spec.get('period_ns', ''))
    if 'constraints' in spec:
        constraint_path = args.source.with_suffix('.json')
        if json.loads(constraint_path.read_text()) != spec['constraints']:
            raise RuntimeError('Constraint file differs from manifest')
        metadata['constraints_sha256'] = hashlib.sha256(constraint_path.read_bytes()).hexdigest()
        tcl_path = args.source.with_suffix('.tcl')
        if tcl_path.read_text() != constraint_tcl(spec['constraints']):
            raise RuntimeError('Tcl constraints differ from checked JSON constraints')
        metadata['constraints_tcl_sha256'] = hashlib.sha256(tcl_path.read_bytes()).hexdigest()
        constraint_arg = str(constraint_path.resolve())
    (args.work/'synthesis.json').write_text(json.dumps(metadata, indent=2)+'\n')
    with (args.work/'placement.log').open('w') as log:
        result = subprocess.run([args.placer, str(args.work/'netlist.json'),
                                 constraint_arg, str(args.work/'metrics.json')],
                                cwd=args.work, stdout=log, stderr=subprocess.STDOUT,
                                timeout=1800)
    if result.returncode:
        print((args.work/'placement.log').read_text()[-14000:])
        raise RuntimeError(f'Placement failed ({result.returncode}); artifacts: {args.work}')
    metrics = json.loads((args.work/'metrics.json').read_text())
    diagnostics = {}
    lines = (args.work/'placement.log').read_text().splitlines()
    polish_count = sum(s['stage'] == 'post_swap_sorting' for s in metrics['stages'])
    for label in ('OUTLINE_SUMMARY', 'PLACE_PRE_SMEAR', 'PLACE_SMEAR_SUMMARY',
                  'PLACE_SORTING_SUMMARY', 'PLACE_SWAPPING_SUMMARY', 'PLACE_POST_SWAP_SORTING'):
        events = [dict(item.split('=', 1) for item in line.split()[1:] if '=' in item)
                  for line in lines if line.startswith(label+' ')]
        expected = 1 if label == 'OUTLINE_SUMMARY' else metrics['runs']
        if label == 'PLACE_SORTING_SUMMARY':
            expected += polish_count
        if label == 'PLACE_POST_SWAP_SORTING':
            expected = polish_count
        if len(events) != expected:
            raise RuntimeError(f'{label}: expected {expected} summaries, got {len(events)}')
        if label == 'OUTLINE_SUMMARY' and int(events[0]['bunch_capacity_iterations']) <= 0:
            raise RuntimeError('Outline skipped coarse capacity spreading')
        diagnostics[label] = events
    metrics['diagnostics'] = diagnostics
    (args.work/'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
    print(f'{args.source.parent.name}/{args.source.stem}: {count} cells, '
          f'{metrics["runs"]} runs, WNS {metrics["final_wns_ns"]:.3f} ns; {args.work}')

if __name__ == '__main__':
    main()
