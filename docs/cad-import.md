# CAD import and selection

Use **Home → Import** for DXF/DWG drawings or STEP/STP and IGES/IGS models.
Release packages include STEP/IGES support. For source builds, see
[build instructions](../RUNNING.md). Import runs in the background; warnings and
errors appear in the application. A failed import leaves the project intact.
Resolve active editing drafts before importing.

STEP/IGES exporters sometimes encode circular holes as rational splines instead
of analytic cylinders. Import recognizes spline lines, circular arcs and straight
cylindrical spline extrusions from their control-point data, with recognition
checks at 0.000000001 mm. Degree-elevated extrusions are included.
The check covers the complete curves and extrusion, including arc traversal;
circular rims alone do not establish a cylindrical wall. Other splines retain
their exact curves and existing surface classification. Recognition does not
change body topology, placements or the display mesh. It applies to new imports;
replace or reimport older models and review their linked selections to use it.

Elliptical, parabolic and hyperbolic edges retain exact rational spline curves
for boundary selections. Cones and spheres retain analytic metadata; each
operation still needs to support the selected surface. Both STEP and IGES can
recover closed surface shells as solids, using imported edge tolerances when
sewing. Open or ambiguously nested surface shells remain import errors.

An invalid transferred solid receives a bounded repair attempt on a copy. A
repair is accepted only when solid validation succeeds, face/wire/edge/vertex
counts stay unchanged, and bounds stay within source/import tolerances. Import
diagnostics identify successful repairs, solids that remain invalid, and faces
or edges without supported feature representations. See the
[import robustness implementation notes](cad-import-robustness.md).

The Models tree groups each imported model and its bodies. Add the required bodies
to a [setup](multi-setup.md) before creating model-linked operations. Visibility,
hide and isolate affect the view; they do not change which bodies belong to a setup
or which bodies protect material during adaptive clearing.

Select geometry and create an operation, or create an operation and choose its
geometry in the **Geometry** tab. Review the extracted heights, boundaries and
islands before choosing a cutter. Shift-click toggles multiple features;
Command-click also toggles on macOS. Ctrl-click selects a body's XY silhouette
for Profile or Face. Unsupported selections show an incompatibility reason;
see [operation compatibility](operation-compatibility.md).

Geometry lists provide add, remove and settings controls. Settings use OK/Cancel
to stage or discard selections. **Edit / repair geometry** replaces a selection
while preserving machining parameters and height overrides. Blank linked height
inputs follow the selected geometry; explicit values override it within the
supported cutting interval.

Chamfer can use modeled planar bevel faces, boundary loops or individual edges.
For modeled faces, a cutter-angle mismatch warns; Stepdown limits each
finishing band. For edges and loops, set Chamfer width explicitly. Engrave follows
open or closed planar paths; Carve uses a closed region and retains its islands.
Thread selects cylindrical bores or bosses; review its nominal size and pitch
using the [thread presets guide](thread-presets.md).

To reposition geometry, right-click a model or body in the Models tree and choose
**Move model…** or **Move body…**. The inspector provides relative X/Y/Z distances,
angles and a world-space pivot, initially at the selected geometry's center. Drag
an arrow to translate along the displayed axis, or drag a colored arc's dot to
rotate around that axis. The axes follow the part's orientation; for a model with
multiple bodies, the first body defines the gizmo orientation. Inspector distances
and angles remain relative to the world axes. Dragged translations snap to 0.5, 1, 5 or 10 mm according to
zoom; rotations snap to 5 degrees. The small viewport field and the inspector
accept exact values. The viewport previews the move until **Move** is pressed;
**Cancel** discards it. Rotations are applied X, then Y, then Z around the pivot,
followed by translation. A model move affects all its bodies and can be undone in
one step. Linked toolpaths must be regenerated after a move.

Projects embed imported geometry, so reopening does not require the original
source file. Model replacement is available from the model context menu.
Replacement requires reviewing and reselecting linked geometry before generating
again. Missing or unresolved geometry in an included operation blocks generation
and export. Excluded operations do not block the included job. Older imported
models may need reimporting to enable newer spline or adaptive selections.

The **Output** pane identifies selection and generation errors by setup, operation
and geometry. Drag to select text and use Cmd/Ctrl+C, or choose **Copy output**.
Updates pause while text is selected; clear the selection to resume updates.
