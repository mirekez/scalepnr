#!/usr/bin/env python3
"""Read opt-in PREEMPT_AUDIT logs without trusting the router's cycle guard.

No routing decisions are changed. A successful one-for-one exchange is neutral,
not automatically beneficial; an uncommitted proposal is never a completion.
"""
import argparse
import collections
import json
import re


def fields(line):
    return {match[1]: match[2] if match[2] is not None else match[3]
            for match in re.finditer(r'(\w+)=(?:"([^"]*)"|([^\s]+))', line)}


def ancestor_path(parents, start, target):
    queue = collections.deque([(start, [])])
    seen = {start}
    while queue:
        node, path = queue.popleft()
        if node == target:
            return path
        for parent, event_id in parents.get(node, {}).items():
            if parent not in seen:
                seen.add(parent)
                queue.append((parent, path + [event_id]))
    return None


def read_events(lines):
    events = {}
    groups = {}
    parents = {}
    last_blocker = {}
    pending = []
    pending_tasks = collections.defaultdict(list)
    task_index = 0
    pass_number = None
    last_cut = None
    expected = []
    for line in lines:
        if 'PREEMPT_AUDIT_CUT ' in line:
            event = fields(line)
            event_id = int(event['id'])
            current, victim = event['current'], event['victim']
            key = (event_id, victim)
            group = groups.setdefault(event_id, [])
            if key in events:
                raise ValueError(f'duplicate victim in event {event_id}: {victim}')
            if group and events[group[0]]['current'] != current:
                raise ValueError(f'inconsistent requester in event {event_id}')
            event['cycle_path'] = ancestor_path(parents, current, victim)
            event['immediate_reverse_of'] = (
                last_blocker[current][1] if current in last_blocker
                and last_blocker[current][0] == victim else None)
            parents.setdefault(victim, {})[current] = event_id
            last_blocker[victim] = (current, event_id)
            event['releases'] = []
            events[key] = event
            group.append(key)
            last_cut = key
            pending.append(key)
            pending_tasks[current].append(key)
        elif 'PREEMPT_AUDIT_RELEASE ' in line:
            entry = fields(line)
            for key in groups[int(entry['id'])]:
                events[key]['releases'].append(entry)
        elif 'PREEMPT_AUDIT_OUTCOME ' in line:
            outcome = fields(line)
            for key in groups[int(outcome['id'])]:
                event = events[key]
                if 'outcome' in event:
                    raise ValueError(f'duplicate outcome {outcome["id"]}')
                event['outcome'] = outcome
        elif 'PREEMPT_AUDIT_INVALID_PROPOSAL ' in line:
            proposal = fields(line)
            for key in groups[int(proposal['id'])]:
                events[key]['invalid_proposal'] = proposal
        elif 'commit rollback:' in line and last_cut is not None:
            group = groups[last_cut[0]]
            for key in group:
                events[key]['commit_failure'] = line.split('commit rollback:', 1)[1].split(', cb(', 1)[0].strip()
            terminal = re.search(r'tile=\((\d+),(\d+)\), dst=(\d+), pin=(\d+), joint=(-?\d+)', line)
            masks = re.search(r'cb\(src=([0-9a-f]+),dst=([0-9a-f]+),joint=([0-9a-f]+),local=([0-9a-f]+)', line)
            if terminal and masks:
                dst, pin, joint = map(int, terminal.groups()[2:])
                busy_bits = {
                    'dst': bool(int(masks[2], 16) & (1 << dst)),
                    'local': bool(int(masks[4], 16) & (1 << pin)),
                    'joint': joint >= 0 and bool(int(masks[3], 16) & (1 << joint))}
                for key in group:
                    events[key]['terminal_busy_bits'] = busy_bits
        elif 'PREEMPT_AUDIT_TASK ' in line:
            task = fields(line)
            task_index += 1
            task['index'] = task_index
            for event_id in pending_tasks.pop(task['current'], []):
                events[event_id]['task'] = task
        elif 'routeDesign pass:' in line and 'stage_pass=' in line:
            pass_number = int(re.search(r'stage_pass=(\d+)', line)[1])
            for event_id in pending:
                events[event_id]['pass'] = pass_number
            pending.clear()
        elif 'routeDesign pass state:' in line:
            match = re.search(r'stage=(.*?),.*by_kind\(takeoff=(\d+)/(\d+),'
                              r'bridge=(\d+)/(\d+),grounding=(\d+)/(\d+)\)', line)
            if match:
                stage = ('Moving' if match[1].startswith('Moving ')
                         else match[1].removesuffix(' routing'))
                expected.append({'stage': stage,
                                 'pass': pass_number,
                                 'bridge': int(match[4]) + int(match[5]),
                                 'other': sum(int(match[i]) for i in (2, 3, 6, 7))})
    return list(events.values()), expected


def summarize(events, expected):
    counts = collections.Counter()
    for name in ('missing_outcomes', 'cuts_without_task_outcome', 'missing_release_records',
                 'bad_releases', 'multiple_owner_cuts', 'repeated_victim_same_pass',
                 'task_complete_binding_mismatches'):
        counts[name] = 0
    tasks = {}
    seen_victims = set()
    completed_per_group = collections.Counter()
    for event in events:
        completed_per_group[event['id']] += event['victim_complete'] == 'true'
    counted_outcomes = set()
    for event in events:
        counts['cuts'] += 1
        counts['joined' if event['joined'] == 'true' else 'unjoined'] += 1
        counts['complete_victims' if event['victim_complete'] == 'true' else 'partial_victims'] += 1
        counts['ignored_blocker_updates'] += event['blocker_update_ignored'] == 'true'
        if 'pass' in event:
            victim_key = (event['stage'], event['pass'], event['victim'])
            counts['repeated_victim_same_pass'] += victim_key in seen_victims
            seen_victims.add(victim_key)
        counts['accepted_historical_cycles'] += event['cycle_path'] is not None
        counts['accepted_immediate_reversals'] += event['immediate_reverse_of'] is not None
        counts['bad_releases'] += sum(r['leased'] != 'false' or r['owners'] != '0'
                                      for r in event['releases'])
        counts['missing_release_records'] += len(event['releases']) != int(event['cuts'])
        if 'task' in event:
            tasks[event['task']['index']] = event['task']
        else:
            counts['cuts_without_task_outcome'] += 1
        counts['multiple_owner_cuts'] += event['owners'] != '1'
        removed = event['fragments'].split('->')
        counts['removed_fragments'] += int(removed[0]) - int(removed[1])
        if 'outcome' not in event:
            counts['missing_outcomes'] += 1
            continue
        # Several binding cuts share one committed replacement. Count all
        # victims above, but credit that requester completion only once.
        if event['id'] in counted_outcomes:
            continue
        counted_outcomes.add(event['id'])
        result = event['outcome']
        if 'materialized' in result:
            counts['invalid_proposals'] += result['materialized'] != 'true'
            counts['post_materialization_commit_failures'] += (
                result['materialized'] == 'true' and result['committed'] != 'true')
        if 'invalid_proposal' in event:
            counts['invalid_proposal_' + event['invalid_proposal']['reason']] += 1
        if 'commit_failure' in event:
            kind = ('terminal' if event['commit_failure'].startswith('docking terminal lease failed')
                    else 'other')
            counts['commit_failure_' + kind] += 1
        complete = result['committed'] == 'true' and result['complete'] == 'true'
        delta = completed_per_group[event['id']] - int(complete)
        if delta != int(result['unfinished_delta']):
            raise ValueError(f'incorrect task accounting in event {event["id"]}')
        counts['requester_committed'] += result['committed'] == 'true'
        counts['requester_completed'] += complete
        counts['worsened' if delta > 0 else 'improved' if delta < 0 else 'neutral'] += 1
        counts['unfinished_delta_sum'] += delta
        if result['committed'] != 'true':
            counts['no_requester_commit'] += 1
    for task in tasks.values():
        complete = task['task_complete'] == 'true' and task['bound_complete'] == 'true'
        delta = int(task['complete_victims']) - int(complete)
        counts['tasks_with_preemption'] += 1
        counts['task_worsened' if delta > 0 else 'task_improved' if delta < 0 else 'task_neutral'] += 1
        counts['task_unfinished_delta_sum'] += delta
        counts['task_complete_binding_mismatches'] += task['task_complete'] != task['bound_complete']
        counts['task_completed'] += complete
        counts['task_preemptions'] += int(task['preemptions'])
    counts['expected_bridge_cuts'] = sum(p['bridge'] for p in expected)
    counts['bridge_coverage_difference'] = counts['cuts'] - counts['expected_bridge_cuts']
    counts['other_kind_cuts_not_in_bridge_audit'] = sum(p['other'] for p in expected)
    return dict(sorted(counts.items()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log')
    parser.add_argument('--stage', help='e.g. Fanouts; omit to audit every recorded bridge cut')
    parser.add_argument('--through-pass', type=int,
                        help='include only closed stage passes through this number')
    parser.add_argument('--json', action='store_true', help='include every event and its cycle witnesses')
    args = parser.parse_args()
    with open(args.log) as stream:
        events, expected = read_events(stream)
    if args.stage:
        events = [e for e in events if e['stage'] == args.stage]
        expected = [p for p in expected if p['stage'] == args.stage]
    if args.through_pass is not None:
        events = [e for e in events if e.get('pass') is not None and e['pass'] <= args.through_pass]
        expected = [p for p in expected if p['pass'] is not None and p['pass'] <= args.through_pass]
    report = {'summary': summarize(events, expected)}
    if args.json:
        report['events'] = events
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
