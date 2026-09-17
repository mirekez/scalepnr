# Multiple primary clocks

Clock constraints must be declared after loading the RTL and before Estimate,
Outline, packing, Sorting, Swapping or routing. Units are nanoseconds.

```tcl
create_clock -name core -period 1.0 [get_ports core_clk]
create_clock -period 1.5 -name peripheral [get_ports peripheral_clk]
get_clocks {core peripheral}

# Use this when the sources have no synchronous relationship.
set_clock_groups -asynchronous \
    -group [get_clocks core] -group [get_clocks peripheral]
```

`create_clock` accepts options in either order, one source port (including a
one-element Tcl list), and an optional name defaulting to the source name.
Periods must be finite and in [0.000001, 1e9] ns. Invalid options, duplicate
names/sources, unknown sources, empty/multiple sources and ambiguous capture
domains return Tcl errors without replacing the previous timing forest.
`get_clocks` accepts a list of Tcl glob patterns; no argument lists all clocks.
An empty pattern list matches nothing. `set_clock_groups` currently supports
only `-asynchronous` with at least two explicit, disjoint, nonempty groups.
Repeated group declarations accumulate exclusions between groups, not inside a
group. Clock storage keeps addresses stable when further clocks are declared.

## Timing semantics

Primary clocks have rising edges at time zero and repeat at their declared
periods. Unless explicitly asynchronous, a register-to-register setup check
uses the smallest positive launch-to-capture edge separation. Periods are
rounded to a 1 fs grid when computing this separation: it is the greatest
common divisor of the two periods on that grid. For example, 1.0 ns and 1.5 ns
clocks have a 0.5 ns setup budget in either direction, not the receiving
clock's entire period. This follows the closest preceding launch-edge rule
([clock-analysis reference](https://www.intel.com/content/www/us/en/support/programmable/support-resources/design-examples/quartus/tq-clock.html)).

Each capture domain has its own timing forest. A shared combinational output
is reused within that domain, never across domains with different launch
edges or exclusions. The launch offset is included by RTL timing, full placed
timing, prepared trial evaluation and direct local updates. Asynchronous
launch branches are excluded individually; valid same-domain branches of a
mixed cone remain constrained. A pure asynchronous endpoint is excluded from
setup violation/profit maps. Its RTL connections are **not** deleted: routing
must still connect the crossing.

Clock-pin lookup is scoped to the primitive type. A clock net connected to a
data input does not make that input a clock pin. Declared clock identity is
traced through single-input buffers described in `Tech::buffers_ports`,
including cascaded and branched clock distribution. Unknown launch domains
are not assigned an invented launch edge.

## Placement and routing

- Estimate resolves a register's clock before processing data inputs, regardless
  of port order. Zero-length aggregation does not merge different clock domains.
- Outline and exact placement obtain clock-dependent data-net pressure from
  `PlaceTiming::preparePlacementGuide`. Sorting and Swapping consume the same
  per-domain setup checks; their local evaluations retain launch-edge offsets.
- `Element::clock_group` optionally declares a shared physical clock input
  within one Tile. Nonnegative equal groups require the same driving RTL net;
  different groups permit independent clocks. `clock_port` names the cell's
  logical clock input (default `C`). Negative groups impose no restriction.
  Site-derived register elements with a modeled `CLK` pin share one group per
  site. Exact packing, pre-packing reservations and trial moves use the same
  legality check. Different buffer outputs remain different physical nets even
  if they originate from the same declared clock.
- Dedicated routing visits every declared clock and every clock-buffer branch.
  Nets are grouped by their actual source connection, not a single chosen
  buffer instance. Different clocks cannot lease the same physical routing
  node. Generic Basic/Fanouts/Moving routing still handles data connections,
  including asynchronous crossings; clock/data pin classification does not
  leak between primitive types.

## Supported boundary

This is a primary-clock **setup** model, not complete static timing analysis.
Generated clocks, arbitrary waveforms/phase shifts, falling-edge relationships,
clock mux alternatives, jitter/uncertainty, skew, clock-to-Q/setup library arcs,
hold checks and dual-clock primitive port-to-clock arcs are not implemented.
Unknown Tcl options are rejected. If two declared clocks reach one data
endpoint, the timing builder rejects the ambiguous primitive rather than
pretending each data port belongs to both clocks. Asynchronous exclusions are
timing constraints, not proof of CDC synchronizer safety.

## Regressions

`src/tests/placing/multi_clock.cpp` exercises real Tcl commands and synthetic
two-clock RTL. Run:

```sh
cmake --build build --target multi_clock_test
ctest --test-dir build -R '^multi_clock\.' --output-on-failure
```

Cases cover Tcl validation and >16 clocks, repeated forest rebuilds,
multirate edge relationships, shared combinational cones and asynchronous
pruning, Estimate clock ownership, full/local/prepared placed-timing agreement,
the complete placement sequence, actual Sorting/Swapping improvements,
shared-clock packing/previews, clock-buffer fanout routing, physical conflicts,
and rejection of ambiguous multi-clock primitives.
