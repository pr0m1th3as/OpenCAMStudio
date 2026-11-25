# Setups and work coordinates

New projects start with no setups. Import a STEP model once, select a model or
body in the project browser or 3D view, and create a setup. A selected model adds
all its bodies; selected bodies add just those instances. With no model or body
selected, the setup starts with no model targets.

The left browser lists physical setups, with operations
directly beneath them in execution order. Add a setup for another machining side,
or duplicate an existing setup to copy its operations and geometry links. Each
setup has independent model targets, work axes, datum, stock, material, safety
heights and NC program name. Selecting an operation activates its parent setup.

In the Setup tab, click Add to include more bodies,
then click each desired body in the shaded view. Done or Escape finishes picking;
Remove can clear the last body. The model list and viewport highlights preview
this selection until OK. Duplicating a setup retains its selected bodies.
The copy is inserted immediately after its source and automatically uses **From
previous setup** stock. Choose a box in its Stock tab to start with a fresh blank.
The Models tree groups body instances beneath their imported source model.

The right column holds Setup, Stock and Postprocess tabs. Edits across the tabs
share one draft. OK validates and commits the entire draft as one undoable change;
Cancel restores the committed values in every tab. Tab changes never commit.
Resolve a draft before navigating to another setup or operation.

Double-click a setup (or choose **Edit setup** from its context menu), then open
the **Setup** tab's **Tool change** section. **Use machine maximum Z** is enabled
by default; disable it to enter a **Tool-change height** in millimetres relative
to the base WCS origin. Click **Apply setup** to save. This height is used before
tool changes and manual re-fixturing and should be at or above every operation's
clearance plane. The planner raises lower values to clearance, with a warning
for differences greater than 0.000001 mm; smaller CAD padding is clamped silently.

The automatic value comes from **Maximum Z from base WCS** in the machine settings,
not from the Z travel span. For example, 100 mm of travel from −50 to +50 gives a
maximum of +50 mm. Set the maximum to the highest Z actually reachable relative to
the base work origin for the setup; changing it preserves the configured Z travel.
The machine settings do not know the physical placement of the work origin, so
verify this value for each setup or enter an explicit tool-change height.

The Setup tab combines orientation and datum in **Workpiece coordinate system
(WCS)**. The Orientation menu offers model orientation, Z/X, Z/Y and X/Y axis
pairs, plus individual selected axes. Choose straight model or stock edges for
axis references; each selected direction can be flipped, cleared or replaced.
Two perpendicular directions determine the third axis. Parallel, skewed, curved,
degenerate, missing and left-handed reference combinations are rejected.

The Origin menu offers model origin, a selected point, a stock-box point, a
model-box point, two points or numeric coordinates. For either box option, click
Point on box and choose a corner, edge midpoint, face center or box center in the
view. Box points remain selectable with drawing snapping disabled. Box and picked
points capture an explicit datum; they do not follow later stock/model resizing.
Changing orientation preserves an explicit datum's world position. Model origin
uses the selected model instance's origin. All WCS edits preview in the same setup
draft and commit with OK or revert with Cancel. In a setup view, the viewport grid
uses the active work origin and the setup axes; it updates with the WCS preview.
The model browser view uses the world grid.

Stock supports a fixed-size box, positioned against the model along each axis,
or a relative-size box with independent offsets on all six sides. Negative offsets
move a stock face inward. The resulting box must have positive dimensions.
Material is separate from stock shape.
After the first setup, **From previous setup** uses the remaining
material after the immediately preceding setup's included operations. Generating
or simulating recalculates the required earlier setups automatically, including
chains of inherited stock. Reordering setups changes the source. Upstream edits,
suppression and tool changes invalidate the inherited result. Separate box stock
starts a new stock chain. Work axes can change between setups; transferred stock
retains cavities when the workpiece is turned over. Resolve any upstream generation
errors or simulation collisions before using its stock. The first setup needs box
stock and cannot inherit stock.

Model dimensions and containment feedback are in the right column.
The center viewport contains the model and operation previews, including semantic
selection highlights; setup controls and stock/WCS overlays stay out of it.

Select Body, Face, Edge, Loop or Vertex geometry and create a supported operation,
or start an operation and select its geometry. STEP Pocket, Drill, Profile and
Face retain [their existing qualification limits](operation-compatibility.md).
Eligibility and cutting heights are evaluated in the active setup's axes.

Generate and export the active setup as its own NC program. Switching setups
discards the previous generated result; generate the newly active setup before
exporting. Copy/move commands re-resolve linked operations in their destination
setup and remap conflicting tool numbers before committing. Explicit planar
operations require the same axes when transferred between setups.

## Limits

Initial stock is a box. Cylindrical stock and stock from another solid are not
available. Material-removal simulation uses a heightfield with bounded material
intervals for transferred cavities. Stock transfer is sampled at the simulation
accuracy and stock bounds/heights use the enclosing original blank. Simulation checks setup fixture envelopes, but does not verify holders or an
entire machine assembly.

## Browser visibility

Imported model rows expand/collapse their body instances and provide a whole-model
visibility checkbox and context action. Parent visibility preserves individual
body choices. Picking geometry in the 3D view highlights its body in the browser
and expands the parent model. The browser title is the project filename without
`.ocam`, or Untitled before the first save. Selecting Models shows the collection; selecting a model or body
shows that item. Selecting a setup or one of its operations scopes CAD display,
picking, snapping and framing to the setup's model and fixture instances (including
draft membership). Empty model and fixture lists show no CAD bodies. Add-model picking temporarily
exposes other visible models so bodies can be added to the setup.

Tools (in use) starts collapsed. Operations expand to show their own tool details,
including both tools for a clearing/carving operation in cutting order. Expansion
and visibility are view state and do not change machining inclusion or the project.

The Setup tab also has a **Fixture** section. Click **Add**, select fixture bodies
in the viewport, and click **Done**. Selecting a fixture removes that body from
machining models; selecting it as a model removes it from fixtures. Set separate
**Radial clearance** (setup XY) and **Axial clearance** (setup Z), in millimetres;
both default to 5 mm and accept zero. Apply and Cancel include these settings in
the same setup transaction. Duplicated setups retain their fixture configuration.

Fixtures never become stock or machining targets. Their clearance envelopes use
conservative transformed solid body bounding boxes, protecting holes and recesses
as well. The planner raises clearance/retract and tool-change planes above the
fixture envelopes, then checks the complete path, including drilling cycles,
arcs/helices, patterns, cutting moves and transitions, against the cutter and
shank's conservative cylindrical envelope. Computed-compensation profiles with
straight plunge and no holding tabs are split around fixture keep-out regions.
Surviving sections reconnect above the fixtures and re-enter at plunge feed;
exact arcs and pattern ownership are preserved. A warning identifies omitted
sections and the protected material that needs another setup. A fully blocked
profile instance fails. Other operations and unsupported profile configurations
retain collision rejection. Every generated escape and re-entry is verified,
including overhead-fixture conflicts. Playback checks the same envelopes
and stops at contact. Fixture collisions block NC export by default. **Preferences →
General** can treat them as export warnings. Holder and machine
geometry are not included. Fixtures remain visible when simulation hides the model.
Changing fixtures, clearances, source revisions or placements invalidates generated
results. Missing fixture geometry must be removed or reselected before generation.

Fixture settings use project schema 37. Earlier projects migrate with no fixtures;
older builds refuse the newer schema rather than discarding fixture protection.
