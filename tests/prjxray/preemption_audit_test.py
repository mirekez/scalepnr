import unittest

from preemption_audit import fields, read_events, summarize


def cut(event_id, current, victim, *, joined=True, complete=True, committed=True):
    flag = lambda value: str(value).lower()
    return [f'PREEMPT_AUDIT_CUT id={event_id} stage=Fanouts current="{current}" '
            f'victim="{victim}" victim_complete={flag(complete)} joined={flag(joined)} '
            'cuts=1 owners=1 fragments=10->3 first_blocker="" blocker_update_ignored=false',
            f'PREEMPT_AUDIT_RELEASE id={event_id} leased=false owners=0',
            f'PREEMPT_AUDIT_OUTCOME id={event_id} committed={flag(committed)} '
            f'complete={flag(committed)} unfinished_delta={int(complete)-int(committed)}']


class AuditTest(unittest.TestCase):
    def test_shared_cut_counts_all_victims_and_one_completion(self):
        for second_complete, delta in ((True, 1), (False, 0)):
            first = cut(1, 'A', 'B')
            second = cut(1, 'A', 'C', complete=second_complete)
            lines = [first[0].replace('owners=1', 'owners=2'),
                     second[0].replace('owners=1', 'owners=2'), first[1],
                     first[2].replace('unfinished_delta=0', f'unfinished_delta={delta}')]
            events, expected = read_events(lines)
            result = summarize(events, expected)
            self.assertEqual([e['victim'] for e in events], ['B', 'C'])
            self.assertEqual(result['cuts'], 2)
            self.assertEqual(result['requester_completed'], 1)
            self.assertEqual(result['unfinished_delta_sum'], delta)
            self.assertEqual(result['removed_fragments'], 14)
            self.assertEqual(result['missing_release_records'], 0)
            self.assertEqual(result['bad_releases'], 0)

    def test_duplicate_binding_is_not_a_shared_cut(self):
        lines = cut(1, 'A', 'B')
        with self.assertRaises(ValueError):
            read_events([lines[0], lines[0]])

    def test_fields(self):
        self.assertEqual(fields('id=7 current="net with spaces" first_blocker=""'),
                         {'id': '7', 'current': 'net with spaces', 'first_blocker': ''})

    def test_counts_only_committed_results(self):
        lines = (cut(1, 'A', 'B', joined=False, committed=False)
                 + cut(2, 'A', 'C') + cut(3, 'D', 'E', complete=False))
        events, expected = read_events(lines)
        result = summarize(events, expected)
        self.assertEqual((result['worsened'], result['neutral'], result['improved']), (1, 1, 1))
        self.assertEqual(result['no_requester_commit'], 1)
        self.assertEqual(result['bad_releases'], 0)

    def test_later_blocker_reverse_is_not_lost(self):
        # A cuts B; C cuts B later. Keeping only B's first blocker A loses
        # C, so a production first-parent guard can wrongly allow B to cut C.
        events, _ = read_events(cut(1, 'A', 'B') + cut(2, 'C', 'B') + cut(3, 'B', 'C'))
        self.assertEqual(events[2]['cycle_path'], [2])
        self.assertEqual(events[2]['immediate_reverse_of'], 2)

    def test_transitive_cycle(self):
        events, _ = read_events(cut(1, 'A', 'B') + cut(2, 'B', 'C') + cut(3, 'C', 'A'))
        self.assertEqual(events[2]['cycle_path'], [2, 1])
        self.assertIsNone(events[2]['immediate_reverse_of'])

    def test_whole_task_accounting_and_pass(self):
        # One task fails its first cut, then completes using a partial victim.
        # Count its final completion once, not once per speculative attempt.
        events, expected = read_events(
            cut(1, 'A', 'B', joined=False, committed=False)
            + cut(2, 'A', 'C', complete=False)
            + ['PREEMPT_AUDIT_TASK current="A" task_complete=true bound_complete=true '
               'complete_victims=1 preemptions=2',
               'routeDesign pass: 8, stage_pass=3, stage=Fanouts routing',
               'routeDesign pass state: stage=Fanouts routing, '
               'by_kind(takeoff=0/0,bridge=1/1,grounding=0/0)'])
        result = summarize(events, expected)
        self.assertEqual(result['worsened'], 1)
        self.assertEqual(result['task_neutral'], 1)
        self.assertEqual(result['task_unfinished_delta_sum'], 0)
        self.assertEqual(result['expected_bridge_cuts'], 2)
        self.assertEqual([e['pass'] for e in events], [3, 3])

    def test_missing_outcome_and_bad_release(self):
        events, expected = read_events(cut(1, 'A', 'B')[:1]
                                      + ['PREEMPT_AUDIT_RELEASE id=1 leased=true owners=1'])
        result = summarize(events, expected)
        self.assertEqual(result['missing_outcomes'], 1)
        self.assertEqual(result['bad_releases'], 1)

    def test_rejected_proposal_after_cut(self):
        lines = cut(1, 'A', 'B', committed=False)
        lines[-1] += ' materialized=false'
        lines.append('PREEMPT_AUDIT_INVALID_PROPOSAL id=1 reason=tile_reentry '
                     'forward="(1,0)->(2,0)" backward="(2,0)->(1,0)"')
        events, expected = read_events(lines)
        result = summarize(events, expected)
        self.assertEqual(result['invalid_proposal_tile_reentry'], 1)
        self.assertEqual(result['invalid_proposals'], 1)
        self.assertEqual(result['worsened'], 1)

    def test_terminal_failure_masks(self):
        lines = cut(1, 'A', 'B', committed=False)
        lines.insert(2, 'route debug: commit rollback: docking terminal lease failed '
                     'fragment=1, tile=(2,3), dst=4, pin=5, joint=6, pin_leased=false, '
                     'cb(src=0,dst=10,joint=40,local=0)')
        events, expected = read_events(lines)
        self.assertEqual(events[0]['terminal_busy_bits'],
                         {'dst': True, 'local': False, 'joint': True})
        self.assertEqual(summarize(events, expected)['commit_failure_terminal'], 1)

    def test_moving_stage_counter_names(self):
        lines = [line.replace('stage=Fanouts', 'stage=Moving') for line in cut(1, 'A', 'B')]
        lines += ['routeDesign pass: 70, stage_pass=1, stage=Moving destinations',
                  'routeDesign pass state: stage=Moving destinations, '
                  'by_kind(takeoff=0/2,bridge=1/0,grounding=0/0)']
        events, expected = read_events(lines)
        self.assertEqual(expected[0]['stage'], 'Moving')
        result = summarize(events, expected)
        self.assertEqual(result['bridge_coverage_difference'], 0)
        self.assertEqual(result['other_kind_cuts_not_in_bridge_audit'], 2)


if __name__ == '__main__':
    unittest.main()
