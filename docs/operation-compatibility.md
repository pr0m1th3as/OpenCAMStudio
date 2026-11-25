# Choosing an operation

Choose an operation for the feature you intend to machine. Compatible geometry
alone does not establish a suitable cutter, cutting depth or safe approach.

| Operation | Use it for | Main limits |
| --- | --- | --- |
| Profile | Cutting along drawing contours, planar CAD chains, face contours or body silhouettes. | Auto/Closed CAD chains need supported vertical walls. Explicit Open mode uses operator-defined depth and does not finish adjoining blends. Linked CAD excludes stock-boundary clearing, controller compensation and taper. |
| Pocket / 2D adaptive | Clearing a closed region with islands, or inside a closed planar CAD chain. | Choose one or more floor faces at the same height to infer each pocket and its islands, or Add Chain to select closed boundary loops or disconnected closed edge chains. All selected pockets share the operation cutting heights and settings; the tool retracts between regions. For a chain, set the cutting depth in Heights; the selected chain defines the top plane and does not infer islands. |
| Drill | Drawing circles or cylindrical CAD bores aligned with setup Z; pecking and dwell. | Grouped holes need equal diameters and heights. Cutter diameter must match. Conical tips, counterbore stacks and breakthrough are not inferred. |
| Face | Facing a drawing boundary, upward planar CAD target or body silhouette. | CAD target must be the highest accessible plane; raised protected material blocks lower facing. Stock to Leave is axial only. |
| Chamfer | Planar paths or modeled inclined planar bevel faces. | Modeled face bands support contour roughing; a bevel/cutter angle mismatch warns. The strategy needs a conical cutter; rounded fillets, general conical/freeform bevels and nonplanar edge following are unsupported. |
| Bore | Helically milling an accessible circular hole or boss with an end mill. | It finishes walls rather than clearing solid holes. CAD supports certified cylinders; drawing taper is limited to reachable slopes. Mixed sizes/depths need separate operations. |
| Thread | Milling a cylindrical internal or external 60° thread. | Requires a suitable thread mill, explicit major diameter/pitch and adequate prepared-bore clearance. Full-profile cutter pitch must match. No taper or other thread forms. |
| Engrave | Following open or closed planar paths with the tool axis. | Groove width follows tool shape and depth; cutter suitability and reach warn. No text generation or nonplanar surface following. |
| Carve | Variable-depth V-carving within a closed region, optionally with a separate clearing cutter. | Depth is a maximum; narrow areas may remain shallower. No arbitrary sculpted-surface or multi-plane carving. |
| 3D Adaptive Clearing | Roughing selected CAD bodies or projected face areas at changing Z levels. | Conservative three-axis roughing; undercuts stay protected. No general freeform finishing, rest machining or pattern repetition. |

## Geometry, tools and heights

Planar operations work in setup XY; cylindrical features align with setup Z.
Create another setup for tilted features. Bore, Drill and Thread can accept
qualified coaxial rounded entrances, but machine only the straight cylinder
beneath the entrance. Covered, narrowing, incomplete or obstructed entrances
remain unsupported.

Use a drill for drilling, an end mill for Bore and a thread mill for Thread.
A selected bore diameter is not a nominal thread specification. Review the
prepared hole, size, pitch and cutter reach using [thread presets](thread-presets.md).
Blind-hole clearance and thread fit/class are not inferred from the CAD model.

Linked top/bottom heights must remain within the supported feature. Editing a
height does not make an unsupported wall accessible. Stock to Leave is available
for Profile, Pocket, Face (axial only) and Adaptive; Bore has its own allowances.
Replacing or losing an included CAD selection blocks generation/export until
repaired. Excluded operations do not block the included job.

## Verification limits

Only completed jobs are exportable. Cancelled or failed fragments are inspection
data; regenerate successfully before export. Simulation checks rapid moves and
non-cutting feed links against remaining stock, and checks non-cutting tool tips
during descent. A thread mill needs its full cutter diameter clear for a straight
plunge; a small pilot hole that only fits its neck is insufficient. Bore wall
finishing can leave a solid core around that pilot hole. Prepare the hole with
drilling or pocket clearing before threading.

During playback, stock contact on these moves pauses at first contact, colors
the cutter red and shows the reason and coordinates. Press Play to acknowledge
that contact and continue inspecting; rewinding allows it to be detected again.
These collisions also block NC export by default. **Preferences → General** can
treat them as export warnings.

Simulation combines a heightfield with bounded tooth cuts to display thread
grooves and their machined flanks. It does not certify thread fit/class or support
general undercut machining, holders, fixtures or whole-machine verification.
Controller compensation also depends on the selected post; its backplot follows
programmed coordinates. Patterned paths need their own destination clearance
review; copying a path does not certify a new model feature.
