"""Offline counter aggregation, including mixed stdout/stderr log records."""

import io
import unittest

from summarize_repair_profile import summarize


class RepairProfileTest(unittest.TestCase):
    def test_verbose_cycles_are_not_charged_as_clean_search(self):
        record = ('REPAIR_CALL id=1 parent=0 kind=gap net="signal" route="load" '
                  'binding=0 route_id=3 revision=9 end_revision=9 result=failed '
                  'wall_s=1 exact_gap_search_s=1\n')
        log = io.StringIO('ROUT routeDesign stage: Clock routing\n'
                         'REPAIR_TOTAL context="before Fanouts routing"\n' + record +
                         'ROUT routeDesign stagnation detected: stage=Fanouts routing\n' + record +
                         'REPAIR_TOTAL context="before Moving destinations"\n' + record)
        reports = summarize(log)
        self.assertEqual(len(reports), 3)
        self.assertEqual(reports['chunk_1:Fanouts routing']['calls'], 1)
        self.assertEqual(reports['chunk_1:Fanouts routing [diagnostics active at return]']['calls'], 1)
        self.assertEqual(reports['chunk_1:Moving destinations']['calls'], 1)

    def test_nested_time_and_prefixed_records(self):
        log = io.StringIO(
            'ROUT routeDesign stage: Clock routing\n'
            'unfinished console messageREPAIR_TOTAL context="before Moving destinations"\n'
            'another messageREPAIR_CALL id=2 parent=1 kind=source net="signal a" '
            'route="load b" binding=0 route_id=3 revision=9 end_revision=9 '
            'result=failed wall_s=2 source_free_search_s=2 source_expanded=5\n'
            'REPAIR_CALL id=1 parent=0 kind=gap net="signal a" route="load b" '
            'binding=0 route_id=3 revision=9 end_revision=9 result=failed '
            'wall_s=3 exact_gap_search_s=1 gap_expanded=2\n'
        )
        report = summarize(log)["chunk_1:Moving destinations"]
        self.assertEqual(report["exclusive_total_seconds"], 3)
        self.assertEqual(report["root_calls"], 1)
        self.assertEqual(report["calls"], 2)
        self.assertEqual(report["counters"]["source_expanded"], 5)

    def test_binding_retries_and_chunk_boundaries(self):
        lines = ['ROUT routeDesign stage: Clock routing\n']
        for chunk in range(2):
            for call, revision in enumerate((9, 9, 10), 1):
                lines.append(
                    f'REPAIR_CALL id={call} parent=0 kind=gap net="signal" '
                    f'route="load" binding=0 route_id=3 revision={revision} '
                    f'end_revision={revision} result=failed wall_s=1 exact_gap_search_s=1\n'
                )
            lines.append('ROUT routeDesign stage: Clock routing\n')
        reports = summarize(io.StringIO(''.join(lines)))
        self.assertEqual(len(reports), 2)
        for report in reports.values():
            self.assertEqual(report["distinct_root_bindings"], 1)
            self.assertEqual(report["repeated_root_calls"], 2)
            self.assertEqual(report["same_start_revision_repeated_root_calls"], 1)


if __name__ == '__main__':
    unittest.main()
