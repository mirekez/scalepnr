# Routing Visualization

## Purpose

`fpga::Visualizer` produces a detailed PNG snapshot of routing state around one
grid coordinate. In debugging discussions this image is called a
**VISUALIZATION**. It is intended to answer two questions together:

1. Which numeric crossbar resources are leased in each nearby tile?
2. Which routed nets account for those leases?

The renderer is architecture-neutral. It reads numeric crossbar masks, resolved
jump deltas, tile lease state, and `Wire` fragments. It does not interpret
vendor, tile, wire, cell, or pin names.

## Image Geometry

The image is always 2560x2560 pixels and represents a fixed 5x5 tile window.
Each tile owns a 512x512 pixel slot. `drawCB(x, y)` treats `(x, y)` as the center
and draws coordinates from `(x-2, y-2)` through `(x+2, y+2)`. Slots outside the
device remain blank, so coordinates do not shift when the center is near an
edge.

Every valid tile retains its rectangle and coordinates, even when no crossbar
is assigned. Such tiles display `No crossbar assigned`. A resource tile sharing
another tile's crossbar displays `Crossbar at x,y` instead of duplicate nodes
with an unrelated empty lease state. Crossbar nodes and guides are drawn only
at the owning routing tile, using `Device::routeTile()` ownership. Endpoint
lookups and highlighting use that same owner. An owner outside the window is
identified in the label, not copied into the resource tile's slot.

Each rectangle is inset by 10 pixels. Numeric nodes
are 2x2 pixel dots placed at 9 pixel intervals:

- Local nodes are divided evenly between three internal vertical guides. The
  first-to-second spacing is 90 pixels and the second-to-third spacing is 110
  pixels; their offsets are 130, 220, and 330 pixels. Constant-zero and
  constant-one locals follow ordinary locals in the ordered set. Stored names
  are drawn in black to the left of each guide.
- Joint nodes occupy the right internal vertical guide, from top to bottom.
  Its offset is 350 pixels.
  Stored joint-node names are drawn in black to the right of the guide.
- DST nodes occupy the outer edge facing their physical source tile. On the
  top, right, bottom, and left edges they are counted respectively from the
  left, top, right, and bottom corners.
- SRC nodes occupy the edge facing their resolved target tile. Their order is
  opposite to DST order, so the two classes grow toward one another from
  opposite corners.

The local and joint guides start 60 pixels below the Crossbar's top edge. DST
and SRC sequences start 50 pixels away from their respective corners.
Their stored names are drawn inside the Crossbar, horizontally on left/right
edges and rotated inward on top/bottom edges. Node names are never truncated or
replaced with an ellipsis; long names continue inward past the nominal label
reserve. Labels use a conventional fixed 5x7 bitmap alphabet with a 7-pixel
character advance and are repainted after route lines so the overlay cannot
erase them.
If two node titles resolve to the same text anchor, only the first title is
drawn and the collision is reported in the visualization statistics. This
prevents different labels from being painted over one another.

When one node class exceeds the capacity of a single edge or guide, allocation
continues on additional 9-pixel-spaced lanes toward the rectangle interior.
DST and SRC lanes retain separate edge halves, so opposite ordering remains
unambiguous and dense crossbars never make visualization fail.

The edge is selected by the dominant direction relative to the two diagonals:
directions above the north-west/north-east diagonals use the top edge;
directions left of the north-west/south-west diagonals use the left edge;
directions right of the north-east/south-east diagonals use the right edge; all
remaining directions use the bottom edge. Exact physical directions come from
numeric `dst_by_src` resolution. Encoded numeric deltas are used only to place
nodes that have no valid resolved target.

Free nodes are blue. Nodes leased in the tile's local, joint, SRC, DST, or
tile-pin state are green. Deadend marks are not leases and therefore do not
change node color.

Each Crossbar also has its grid coordinate centered in a heavy RGB
`(105,105,105)` font whose dimensions and advance are five times the node font.
A dedicated single-pass filled rasterizer draws each coordinate exactly once;
it does not reuse the connected-stroke bolding used by smaller labels. It is
the first Crossbar content painted, before borders, guides, nodes, routes, and
labels, so it remains a background watermark.

`CBType::node_names` supplies annotations only. A node is drawn only when it
occurs in numeric connectivity, an exact resolved jump mapping, or live state.
This prevents inherited base-type names from appearing as nonexistent outputs
of a coordinate-specific routing subtype.

Nodes referenced by registered routes are included before positions are
allocated. Filtering stale annotations therefore cannot suppress a valid route
line whose endpoint is present in live route data.

## Route Overlay

Call `drawRoutes()` after `drawCB()`. The visualizer collects route bindings
registered on any tile in the 5x5 window. If any fragment of a binding touches
the window, all of that binding's fragments whose numeric endpoints are visible
are rendered.

Crossbar fragments are drawn in their stored order:

`LOCAL/DST -> JOINT2 -> JOINT -> SRC -> destination DST`

Explicit `WIRE_ROUTE_EDGE` fragments connect their numeric endpoint types
directly. Adjacent tile-pin and crossbar fragments supply the final local
connection. Each physical route binding receives a deterministic line color.
After all lines are rendered, node dots are repainted so green and blue state
remains visible and can be checked against the route overlay. Passing a net or
physical route name to `drawRoutes(name)` highlights every matching binding in
red; this is used for automatic failed-route diagnostics. Highlighted routes
use a three-pixel line, and the selected name is drawn once in a larger bold red
font beside the first visible route point. A small cleared background keeps the
title readable over dense node labels and route lines.

An unresolved task may have no trunk or partial route. In that case the image
does not invent a route line: it draws a three-pixel red cross and the task name
at the exact destination local node.

## Automatic Failure Images

The routing scheduler retains a failure coordinate on every unfinished task.
Forward routing records its last concrete frontier. Route-first source moving
records the last node expanded by backward routing and then the selected source
placement while that placement is being validated.

Unsuccessful reverse source-placement searches retain a bounded set of recent,
unleased diagnostic paths from their deepest explored frontiers to their
destinations. At failure, the newest path whose task is still unfinished is
selected and the image is centered on its frontier. These paths are never
committed to routing state. The terminal image draws the selected real attempt
in red at three pixels instead of showing only the failed-net title.
Moving may replace endpoint objects while preserving a route identity, so this
selection matches the stable net and route name rather than transient endpoint
pointers. If no retained object remains in a queue, the newest real failed
attempt is used as the diagnostic fallback.

If congestion prevents creation of any backward frontier, the first numeric
destination-entry attempt is retained. The image therefore still identifies
the blocked destination node and local pin instead of producing an empty red
overlay.

Moving-source eligibility can fail before reverse search begins. For that
case, the task records its intended numeric destination entry before checking
the movable source cluster. A later, deeper failed reverse attempt replaces
this initial two-fragment diagnostic.

Some stage timeouts can occur before a stage-specific search function is
entered. As the final fallback, `failRouting()` constructs the same numeric
destination-entry pair from the task's target mapping and centers the image on
it. This is a real legal endpoint path, not a geometric source-to-target line.

Before a terminal stage timeout, pass-limit failure, or non-progress assertion,
`RouteDesign` writes one image for the first unfinished task. Its physical route
is red and all other routes retain their deterministic colors. The image is
centered on the retained failure coordinate, with the destination and source
tiles used only as fallbacks when no search frontier was reached.

The output is written in the process working directory as:

```text
VISUALIZATION_unrouted_<escaped-route-name>_<stable-hash>.png
```

Characters outside ASCII letters, digits, `.`, `_`, and `-` are replaced. The
component is bounded and includes a hash of the complete unescaped name, so
long generated names remain portable and distinct. Only the first terminal
failure image is generated during one `routeDesign()` invocation.

All terminal routing-stage failures call `RouteDesign::failRouting()`, including
clock routing and clock repair, constant routing, stage deadlines, and exhausted
ordinary routing. It renders one live failure-centered 5x5 window with `drawCB()`
and `drawRoutes()` before exiting with status 1. There is no full-device scan for
a second congestion image. The selected route is drawn last in red at 3px width
with a bold title. When its fragments have already been released, the capture
marks an available endpoint identity and always prints the failed net's title;
it does not invent a route through free resources.

## API

```cpp
fpga::Visualizer view;
view.drawCB(x, y);
view.drawRoutes("optional_net_or_route_name");
view.writePNG("routing_region.png");
```

The equivalent convenience call is:

```cpp
view.visualize(x, y, "routing_region.png");
```

Calling `drawRoutes(x, y)` prepares a new crossbar window automatically when
the requested center differs from the currently prepared center.

`stats()` reports rendered tiles, nodes, occupied nodes, route bindings,
visible route segments, highlighted labels, and unresolved endpoint markers.
Failure captures also draw the focused task's committed partial route directly
because it may not yet be in a completed-route index.
