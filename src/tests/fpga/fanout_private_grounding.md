# fanout_private_grounding

Purpose: replace retained Grounding from a newly reached DST.
The source trace must be able to use its own reserved sink LOCAL.
Implementation: `fanout_private_grounding_regression` in `fanout_gap_replay.cpp`.
Run: `ctest --test-dir build -R '^fpga.fanout_private_grounding$' --output-on-failure`.

The retained route connects (22,32)/LOCAL156 to (12,37)/LOCAL111.
Two GAPs interrupt its captured numeric path.
A complete sibling independently owns the first four physical hops.
A free six-hop alternative approaches the sink through DST263.
Intermediate LOCAL128 and LOCAL184 are routing bridges, not cell pins.
Variants keep the old two-JOINT Grounding or provide a direct sink arc.
Shared and alias variants retain dependent copies of the interrupted input.
The foreign-owner variant protects an unrelated connection to the sink.

Successful repair must use the six-hop physical path and retain the sink pin.
Dependent same-source terminal routes must be updated together.
Foreign terminal ownership must prevent replacement.
The complete sibling must retain its original four-hop physical path.
A missing final arc must prevent routing even with relaxed input rules.
That disconnected control uses an independent fixture and keeps learned deadends.
Immediate cancellation and a five-hop limit must prevent completion.
An intermediate packed LOCAL reservation must also prevent completion.
Ownership audits check leases, registrations and the deliberate foreign conflict.

History: the original failure rejected a reached binding's own private sink.
The regression also checks shared, alias and two-JOINT terminal variants.
The disconnected control formerly restored topology after learning deadends.
Independent fixtures now preserve immediate deadend marking without rollback.
