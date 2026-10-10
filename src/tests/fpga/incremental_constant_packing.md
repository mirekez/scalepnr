# Incremental packing of a constant-fed enable behind existing Grounding

`incremental_constant_packing.cpp` reproduces example 1's run-4 constant
routing failure. All routing node names are abstract; none are imported
device wire names. This is a reduced terminal/packing reproduction, not a
replay of the whole design.

The original source, destination CrossBar and attached resource Tile are
represented at (0,0), (1,0) and (2,0). Resource positions are reduced to one
LUT slot and one register slot. The failing terminal IDs, sixteen incoming
alternatives and fourteen-occupied/two-free split are preserved.

## Existing route and new input

An earlier run has packed a fixed, completed LUT and installed a complete,
registered Grounding route on a preserved ordinary net:

```
arrival_244 -> junction_63 -> junction_48 -> input_105
```

The LUT also has a direct topology alternative from `arrival_90`, so it has
no mandatory reservation on `junction_63`. Its *live route* nevertheless
owns that JOINT. Thirteen additional complete registered routes occupy the
other busy arrivals. The production congestion audit checks every Tile;
there are no anonymous busy bits, orphan leases or stale registrations.

The next run attempts to pack a register with CE tied to logical constant one:

```
supply_one_64 -> junction_63 -> enable_40 -> register CE
```

The RTL deliberately uses a global constant driver with designator -2 and no
ordinary signal net, matching the placement-time representation. Replacing
it with an ordinary signal would incorrectly test the previous fix instead.

`enable_40` is free, but its only LOCAL/JOINT predecessor is occupied
`junction_63`. The test checks that topology explicitly. All sixteen ordinary
DST paths also require the same JOINT; the two free arrivals cannot bypass it.
The old owner is verified as Grounding, not preemptible transit.

## Tests and controls

- `fpga.incremental_constant_snapshot` validates the fixture, resolved jumps,
  production Takeoff/Grounding topology, occupancy split, ownership, and audits.
- A free-path control routes the old LUT through its alternate arrival. With
  the contested JOINT free, both strict and relaxed previews and exact packing
  must accept the same register. This control has three free original arrivals
  because the old LUT no longer occupies arrival 244.
- `fpga.incremental_constant_packing` requires rejection from strict/relaxed
  preview and strict/relaxed exact packing. Each operation uses a fresh fixture.
  The earlier fixed LUT and completed route must remain intact; every fixture
  is audited again after the operation. Previews/rejections must not place the
  new register.

Run:

```
cmake --build build --target packing_test -j2
ctest --test-dir build -R '^fpga.incremental_constant_' --output-on-failure
```

## Reproduction result before a fix

The snapshot and free-path controls pass. All four packing operations wrongly
return position 0 instead of rejecting the blocked constant path, so
`fpga.incremental_constant_packing` fails with:

```
incremental packing accepted constant-fed CE behind a foreign Grounding JOINT
```

The test is neither disabled nor marked `WILL_FAIL`.

## Fix and additional controls

Packing now searches the declared constant LOCAL/JOINT graph against live
occupancy before accepting a constant input, in both previews and exact
packing. It stops on the first available endpoint and never changes leases.
Foreign occupied nodes (including transit) are unavailable, matching the
Const stage's non-preempting search. A routed distributed net of the same
polarity may share its nodes even though its generated physical source is a
different object from the placement-time logical global driver.

The regression now passes. Additional audited controls cover same-polarity
sharing, opposite-polarity rejection, a longer free bypass with a cycle, and
a missing requested constant root. A deliberately unowned JOINT lease must
also reject placement; that corruption is removed before the final audit.
Abstract packing-only models with neither constant polarity declared retain
their existing behavior. Models declaring distribution must supply the
requested polarity through an available path.

This checks current path availability; it does not add path leases during
previews or reserve optional constant branches. Existing endpoint reservation
behavior is unchanged. The existing general packing and incremental
ordinary-Grounding regressions still pass.
