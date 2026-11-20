# 3D Parallel finishing

Create **Machining → 3D Parallel**, choose a ball nose end mill, then select solid bodies or individual faces. The independent CAD machining projection drives the path; display tessellation does not affect it. All bodies included in the setup remain collision obstacles.

On **Passes**, set horizontal **Stepover**, **Pass angle** (degrees counterclockwise from setup X), and **Tolerance**. **Both ways** alternates the travel direction on successive rows; disabling it makes every row run in the positive pass direction. Rows follow the model in Z. Cutting and plunge feeds are on **Tool**.

**Geometry** offers Inside/Center/Outside containment against the selected geometry's bounding rectangle in the pass coordinate system, plus an additional offset. Contact-only trimming splits rows outside selected face footprints. Positive Inside offsets shrink the rectangle; positive Center/Outside offsets expand it. Holes and gaps may split a row into multiple passes. Radial and axial stock allowances are supported; negative allowances are rejected.

With **Tool center on boundary**, the scan rows reach the projected face
boundary, including around holes and concave edges. The additional offset
adjusts the bounding rectangle used to generate rows. Smooth turns can extend
beyond the row ends. Repositioning moves can cross openings when the cutter
clears the model and stock.

With **Both ways** enabled, neighboring rows join with sideways cutting moves
where the cutter can follow the selected surface. The tool follows the model
in Z through each turn. Where an opening splits the rows, the planner builds
separate cutting chains along its sides so it can finish each side before
repositioning. With **Both ways** disabled, every row retains its selected
cutting direction.

On **Linking → Leads and transitions**, **Transition type** controls those
connections:

- **No contact** retracts between passes. Even when the retraction policy is
  Shortest path, the cutter lifts before traversing.
- **Straight line** joins the original ends of the passes with linear cutting
  moves. Where a chord would cross an opening, it follows the concave boundary
  in short segments if the cutter clears it. This is the default.
- **Shortest path** prefers the shortest checked cutting connection, using a
  straight 3D move when it clears the model. It looks for nearby machining
  chains when repositioning.
- **Smooth** begins and ends the rounded turn at the original row endpoints
  and bulges outward. It uses true tangent arcs on eligible level turns and
  short linear moves on slopes. If an outward turn cannot clear the full
  cutter envelope, the planner uses a checked linear turn or repositions.

On **Lead-in and lead-out moves → Linking moves**, **Retraction policy**
controls repositioning between chains. Parallel defaults to **Shortest path**.

- **Full retraction** raises the tool to the operation's Retract height before
  traversing. It uses Clearance height when Retract cannot clear the model
  and stock.
- **Minimum retraction** raises the tool just enough for the cutter to clear the
  model along the traverse, plus **Safe distance**.
- **Shortest path** tries a checked straight feed move between the endpoints
  and uses Minimum retraction when the straight route is blocked by the model.

The first approach and final exit use Clearance height. Low repositioning
moves use feed so their path works with dogleg rapid controllers. Stock
verification replaces blocked non-cutting links with clearance moves and
checks the revised program. The operation is for pre-roughed stock.

Clearance must exceed the complete model, allowances and stock. Swept checks
cover the ball and the cylindrical cutter envelope, including between contact
samples. Wider shafts, holder collision checks, flat and bull-nose cutters,
silhouette and custom boundaries, slope filters, multiple depths, and
uphill-only or downhill-only passes are outside this operation's current scope.

To generate sampled inward offsets instead of rows, choose **3D Parallel / Scallop**
and enable **Scallop (surface offsets)** on Passes. Stepover is the target distance
along the sampled cutter-center surface. The distance field uses an eight-neighbor
grid with at least four samples per stepover (and at most 0.5 mm spacing), so the
spacing is approximate and depends on grid direction. Contours begin half a stepover
inside the sampled boundary. The selected face footprint limits contact; the containment rectangle and stock allowances use
the same rules as Parallel. The program reports a warning because vertical walls,
small islands, gaps and narrow features can be missed by this height-field method.
Check material removal in simulation. It does not implement Fusion's rest machining,
slope filters, holder collision detection, climb/conventional direction, or its other advanced
Scallop settings. A ball nose cutter and pre-roughed stock are required.

When the last nominal offset grazes an enclosed boundary, it can break into many
tiny fragments. Where the sampled remaining area can still be covered within one
stepover, Scallop moves that final offset slightly away from the boundary to reduce
fragmentation. If that coverage bound cannot be met, the original offsets
are retained. This uses the same sampled approximation as the rest of Scallop.

On **Passes**, **Inside/Outside direction** orders the surface-offset passes:
**Outside → in** advances from the exterior boundary of each selected region;
enclosed holes do not create a competing inward front. **Inside → out** visits
those offsets in reverse. A separate selected island inside a hole has its own
outer boundary. **Shortest machining distance** keeps the outside-in stepover
sequence and uses a bounded greedy search to reduce estimated XY repositioning
among passes at the same level. It does not guarantee a globally shortest route.
When no lead-in position is specified, it may move a closed pass's start or
reverse an open pass to reduce travel. A selected lead-in position takes
precedence over that adjustment. Ordering does not relax the existing cutter,
selection, or stock-link checks, and it does not calculate chip load or rest stock.

On **Lead-in and lead-out moves**, pick a **Lead-in position** on the model. Each
closed contour starts at the sampled point nearest that setup-XY position; an open
pass starts at its nearer endpoint. This is a preference, not a forced plunge at
the selected point. Clear it to restore automatic placement.

**Retraction policy** controls links between Scallop passes. **Full retraction**
uses clearance for every pass. **Minimum retraction** uses a conservative raised
feed link above nearby protected CAD geometry, with the specified **Safe distance**.
**Shortest path** first tries a direct checked feed link; if it cannot clear the
model it tries the raised route. Either option falls back to full clearance if the
route cannot be certified or exceeds the local query limit. The first approach and
final exit always use clearance. These are feed moves, so they avoid diagonal G0
doglegs. Remaining stock is checked during job verification; blocked non-cutting
links are replaced by clearance moves and verified again.

On **Passes**, enable **Smoothing** and set its separate **Smoothing tolerance**
to reduce the generated motion count. The filter combines nearly straight 3D
segments and fits XY arcs on planar portions when selection coverage and swept
cutter checks certify them. Other portions remain linear. Filtering is bounded
against the original path by the smoothing tolerance; add it to the machining
tolerance when assessing total deviation. With smoothing off, all certified sampled
points are emitted. Small segments seen in the display therefore also appear in
the output toolpath; the display is not generating extra machining moves.

Earlier projects load the saved Scallop retraction policy where present. Parallel automatically connects eligible rows when its toolpath regenerates.
The operation is also available through MCP as kind `Parallel`; the `scallop`
parameter selects sampled surface offsets. Its `scallop_order` parameter accepts
`outside_in`, `inside_out`, or `shortest`.

The supplied tray regression can be run locally:

```sh
cargo test -p cam-app --test parallel_project -- --ignored --nocapture
```

It opens `testfiles/tray.ocam`, uses a 6 mm ball nose, generates 1 mm parallel finishing rows, validates the motions, and saves a separate `testfiles/tray-parallel.ocam` review project.
