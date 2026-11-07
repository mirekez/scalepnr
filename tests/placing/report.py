#!/usr/bin/env python3
"""Summarize successfully measured tiers, refusing missing or failed evidence."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--build', type=Path, required=True)
p.add_argument('--sizes', nargs='+', default=['1k','5k','10k','30k','50k','80k','100k'])
p.add_argument('--output', type=Path, required=True)
p.add_argument('--markdown', type=Path)
a=p.parse_args()
root=Path(__file__).resolve().parent
rows=[]
placer=a.build/'tests/placing/placing_synth_test'
if not placer.exists():
    placer=placer.with_suffix('.exe')
placer_hash=hashlib.sha256(placer.read_bytes()).hexdigest()
for size in a.sizes:
    specs=json.loads((root/size/'manifest.json').read_text())
    for family,spec in sorted(specs.items()):
        path=a.build/'tests'/'placing'/size/family
        m=json.loads((path/'metrics.json').read_text())
        s=json.loads((path/'synthesis.json').read_text())
        if not m.get('passed') or m['cells']!=s['cells']:
            raise RuntimeError(f'Missing passing evidence: {size}/{family}')
        if s.get('source_sha256') != hashlib.sha256((root/size/(family+'.sv')).read_bytes()).hexdigest():
            raise RuntimeError(f'Stale synthesis evidence: {size}/{family}')
        if s.get('placer_sha256') != placer_hash or m['period_ns'] != spec['period_ns']:
            raise RuntimeError(f'Stale placement evidence: {size}/{family}')
        times=defaultdict(float)
        for stage in m['stages']:
            times[stage['stage']]+=stage['seconds']
        rows.append(dict(size=size,family=family,cells=s['cells'],target=spec['target'],
                         types=s['types'],period_ns=m['period_ns'],runs=m['runs'],
                         endpoints=m['expected_endpoints'],wns_ns=m['final_wns_ns'],
                         tns_ns=m['final_tns_ns'],violations=m['final_violations'],
                         stage_seconds=dict(times),synthesis_seconds=s['synthesis_seconds'],
                         source_sha256=s['source_sha256'],
                         placer_sha256=placer_hash,
                         outline_peak_luts_per_tile=m['stages'][0]['outline_peak_luts_per_tile'],
                         outline_excess_luts=m['stages'][0]['outline_excess_luts'],
                         presmear_moved_bunches=sum(int(x['moved']) for x in m['diagnostics']['PLACE_PRE_SMEAR']),
                         sorting_moves=sum(int(x['accepted']) for x in m['diagnostics']['PLACE_SORTING_SUMMARY']),
                         swapping_moves=sum(int(x['accepted']) for x in m['diagnostics']['PLACE_SWAPPING_SUMMARY']),
                         post_swap_sorting_runs=len(m['diagnostics'].get('PLACE_POST_SWAP_SORTING', [])),
                         device_side=m['device_side'],yosys=s['yosys']))
a.output.write_text(json.dumps(rows,indent=2)+'\n')
if a.markdown:
    lines=['# Placement validation results', '',
           'All listed designs passed fresh Yosys synthesis, whole-design Outline,',
           'chunked packing/smearing, Sorting and Swapping. Clock periods are fixed',
           'by family in [README.md](README.md); final setup acceptance is WNS ≥ −0.10 ns.',
           'These are generic placement estimates, not routed silicon timing.', '',
           '| Size | Designs | Mapped cells (min–max) | Worst count error | Minimum WNS (ns) | Runs | Sorting moves | Swapping moves |',
           '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
    for size in a.sizes:
        subset=[r for r in rows if r['size']==size]
        lines.append(f"| {size} | {len(subset)} | {min(r['cells'] for r in subset):,}–{max(r['cells'] for r in subset):,} | "
                     f"{max(abs(r['cells']/r['target']-1) for r in subset):.2%} | "
                     f"{min(r['wns_ns'] for r in subset):.3f} | {sum(r['runs'] for r in subset):,} | "
                     f"{sum(r['sorting_moves'] for r in subset):,} | {sum(r['swapping_moves'] for r in subset):,} |")
    lines += ['', 'A stage may make zero moves when its clock already closes or no legal',
              'improving candidate is accepted; the final combined result must pass.',
              'Accepted swaps leaving an actionable deficit receive a final Sorting pass.',
              'Every test still verifies that each stage executed and that all admitted cells',
              'are legal, fixed cells stayed fixed, and final timing has no unplaced edges.', '',
              '[validation.json](validation.json) contains per-design measurements, stage',
              'intervals, primitive mixes, Yosys version and source/binary hashes.',
              'Intervals include validation and orchestration and may come from concurrent',
              'test runs; they are not isolated performance benchmarks.', '',
              f'Tested placer SHA-256: `{placer_hash}`.', '']
    a.markdown.write_text('\n'.join(lines))
print(f'Recorded {len(rows)} passing designs in {a.output}')
