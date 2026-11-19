# 3D Adaptive Clearing and Stock to Leave

Import STEP/IGES solids, add the relevant bodies to a setup and choose
**Machining → 3D Adaptive Clearing**. In Geometry, select bodies, faces or both;
choose an end mill and review stock and heights before configuring Passes.
Selecting a body already includes its faces.

| Control | Meaning |
| --- | --- |
| Tolerance | Permitted geometry approximation error, in mm. Smaller values preserve more detail. |
| Optimal Load | Requested radial engagement, in mm. |
| Stepdown | Maximum separation of primary roughing levels. |
| Fine Stepdown | Intermediate levels that reduce the staircase on changing surfaces. |
| Direction | Climb or conventional milling. |
| Stock to Leave | Separate radial wall and axial floor finishing allowances. |

Fine passes work upward after a deep roughing level. They focus on newly exposed
terraces, with enough overlap into the previously cleared area for the cutter
to reach the new stock. Unchanged vertical walls retain the primary stepdown.
Optimal Load can be exceeded in a full-width channel. Generation reports when it
uses concentric clearing.

In long, narrow pockets, Adaptive can open the full width near the entry, then
clear one end at a time. With a flat-bottomed tool, the passes can shorten
overlap at both ends and use low links where the swept cutter clears the floor.
If an innermost offset leaves a small fragment, a nearby concentric pass can
include it and save a separate entry.

When a frontier fragments into repeated retract/re-entry cycles, generation
compares it with concentric clearing and uses the fallback only if it passes the
model safety check and needs fewer clearance lifts. Otherwise it retains the
certified frontier. The fallback warning also applies here: concentric links and
corners can exceed Optimal Load. Complex regions can exhaust the stay-down
search budget. Their original frontier and any fallback still undergo the model
safety checks.

During calculation, the viewport shows each region as its moves pass the model
safety check. The backplot is provisional until the complete job has been
verified.

Stock to Leave is also available in Profile and Pocket, and as an axial allowance
in Face. Apply the edit before generating. Entry and exit preferences are described
in [entry and exit positions](lead-positions.md).
Use [Lift height](lift-height.md) in **Lead-in and lead-out moves → Linking moves**
to raise verified local transitions above the cutting level. Under **Leads and
transitions**, **Lead-in arc radius** and **Lead-out arc radius** set the arc sizes
for those links.

## Machining boundaries

Selected faces limit machining to projected XY areas. Bodies define local areas
from their setup-XY bounds. Choose **Inside**, **Center** or **Outside** containment.
A positive additional offset moves the boundary inward for Inside and outward for
Center or Outside. Center and Outside can allow runout beyond stock edges.
Selecting the only body in a one-body setup retains full-stock clearing.

All included setup bodies remain protected obstacles, including unselected or
hidden bodies. Visibility does not remove machining protection. Entry helices
must fit inside the permitted area; where they cannot fit, entry uses straight
plunge feed. Unsafe moves or clearance below a setup body reject the operation.

Outside runout can remove stock beside a slope below the slope's height. Rotate
the view when stock appears to overlap the model in a side view.

## Limits

This is conservative three-axis roughing. It leaves material under overhangs and
does not provide freeform finishing or rest machining. Toolholder, fixture and
machine clearance are outside the current verification scope; the heightfield
simulation cannot represent undercuts.

Selections require supported projected machining areas. Complete cylindrical
faces can be accepted when their circular boundary is qualified; partial arcs,
unsupported vertical planes and ambiguous/open trims are rejected. Height
overrides cannot extend below the supported cutting interval.

Older imported models may need reimporting to supply machining geometry.
Tolerance must cover the import approximation, arc fitting and planar rounding;
settings below that budget are rejected. Display mesh quality does not determine
machining accuracy.

Adaptive operations cannot be repeated through a pattern. Select multiple
bodies/faces or create a separate operation at each location.
