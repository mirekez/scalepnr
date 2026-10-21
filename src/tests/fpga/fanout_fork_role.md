# Shared hop lost because FORK and TRANSIT compare differently

The broader run-5 congestion audit found `(62,34)/SRC545` unleased despite a
surviving route claim. A numeric node watch traced its first loss to source
replacement of binding 75 of canonical net 162888, while binding 54 still
used the same hop. History 6285 attempts ownership promotion; by history 6287
the surviving fragment has `live_src=0`.

The departing owner stores `DST545 -> SRC545 -> (63,35)/DST545` as a FORK
(`pos=2`); its shared replica stores the identical physical hop as TRANSIT
(`pos=1`). `samePhysicalFragment()` incorrectly treats that ownership role
as distinct connectivity. Prefix promotion stops before the hop and cleanup
finds no surviving owner, although a shared reference remains. A subsequent
cut promotes that reference, exposing an owned-but-unleased SRC in the audit.

`fpga.fanout_fork_role` imports the recorded 19/21-fragment paths and the
12-hop replacement. A separate scaffold trunk supplies the common-prefix
owners omitted from the capture. Node names are synthetic. Before the fix it
fails with `FORK/TRANSIT metadata mismatch released a surviving shared SRC`.
It also checks cancellation, complete replacement connectivity, survivor
ownership, and safe replacement teardown.

The comparison now treats FORK and TRANSIT roles as the same physical DST
input when all other physical fields agree. It still distinguishes source
LOCAL Takeoff from DST input and checks every node, joint, and coordinate.
