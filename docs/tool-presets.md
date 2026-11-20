# Tool presets and stock materials

The **Tools** page shows the cutter library above a **Cutting parameters** table
for the selected cutter. Preset rows show the linked material name and stable ID.
The inspector edits cutter geometry, description, manufacturer, product ID, and
product link. The **Materials** button in the Tools ribbon opens the shared material
manager in a modal. Materials contain a stable ID, a name, and optional effective
specific cutting force **kc (N/mm²)** for [operation load estimates](cutting-forces.md).
Rename a material without changing its ID; presets and stock use the ID to match.
Materials are saved with the cross-project tool library and included in library
imports and exports.

1. Open **Tools → Materials** to add, rename, or delete materials, then click **Done**.
2. Select a cutter and use **Add preset** below the cutter table. With a
   preset selected this copies its values. Give the new preset a name and select
   its material, or leave it at **Any material**. The material picker stores the ID
   and displays its name. Select a preset row and use **Edit** or **Remove** to manage it.
3. Enter spindle RPM (or **Cutting speed**), cutting feed (or **Feed per tooth**),
   and plunge feed. Choose **Cooling → MMS** to
   run minimum-quantity lubrication, or **None** to cut without cooling. Optionally enter ramp,
   lead-in, and lead-out feeds, as well as stepdown and overlap. Click **Done** to
   return to the table, then **Apply changes** in the inspector to save the cutter
   and presets together. **Cancel** discards pending cutter and preset changes.
   Switching cutters keeps each cutter's pending edits; an asterisk marks edited
   cutters. Return to each marked cutter to **Apply changes** or **Cancel**.
4. In **Setup → Stock**, choose the stock material from the managed list.
5. Create an operation and choose a cutter. Its first preset matching the stock
   material is applied automatically. If none matches, its first **Any material**
   preset is used. If neither exists, the existing nominal/job defaults apply.

The operation's **Tool** tab lists every preset for that cutter, including presets
for other materials. Choose another to apply its values, or edit the operation's
cutting parameters directly. Stepdown is available on the **Passes** tab where
the operation supports it. Applying an overlap preset to an existing concentric
Pocket converts the overlap into **Maximum Stepover** using the cutter diameter.
Presets affect cutting data only, and tool geometry comes from the cutter.

Stepdown maps to axial depth per pass for milling/engraving and peck depth for
drilling. Drilling uses the plunge feed as its axial feed. Chamfer width increments,
thread milling, and the primary V-carve pass do not use the axial stepdown preset;
overlap applies to pocketing and facing. Other strategy settings remain editable
in their existing tabs.

Changing presets replaces the supported cutting values; subsequent manual edits
belong to that operation and do not update the library. Selecting **Manual cutting
data** removes the preset association while retaining the current values. Cancel
and undo work with preset selection and parameter changes together.

Projects embed their tools, available presets, selected preset baseline, material
ID/name, and concrete operation values. Opening a saved project does not reapply
library presets. To use updated library data, explicitly select the cutter again.
Changing stock material does not rewrite existing operations. Older files without
presets or managed material IDs retain their existing cutting data and material name.

A material used by a library preset or stock in the current project must be
unlinked before it can be deleted. Saved projects remain self-contained for
toolpath generation when a material is unavailable in the local library.
Cutting-force analysis requires that material's coefficient in the shared library.

## Selecting an operation cutter

On an operation's **Tool** tab, the cutter button shows its number, diameter, type,
and description, or **select...** when no cutter is selected. Click it to open the
selector. The top table lists cutters already used across the project's setups.
The library table below starts with recommended cutter types for that operation.
Use its search and column filters to narrow the list, or clear the filters to see
other allowed choices. Unsupported cutters cannot be selected.

Click a cutter row to use it. Choosing a project cutter uses its embedded values;
choosing a library cutter uses its saved library values. The change remains part
of the operation draft until Apply. **Cancel**, Escape, or clicking outside the
selector closes it without changing the cutter. The Tools view's search and
filters are restored when the selector closes.

## Cutter dimension drawings

In the Tools editor, click a geometry parameter or focus it with Tab to show its
measurement on the cutter profile. Dimension lines, arrows, and labels follow the
current working values as you edit. Selecting the flute count highlights the flutes.

On an operation's **Tool** tab, click **i** beside the cutter selector to open the
full dimension drawing for that operation's selected cutter, including a pending
cutter change. Close it with **Close**, Escape, or a click outside the drawing.

## Roughing end mills

Choose **Roughing End Mill** as the cutter type for a flat-bottomed rougher.
It has its own library identity and can be selected for milling and clearing.
Toolpaths and simulation use its nominal flat-ended cylindrical envelope; the
serrated cutting edges and the resulting surface finish are not modeled. Enter
cutting parameters appropriate for the actual cutter and material.

## Import Fusion tool databases

In **Tools → Import**, choose a Fusion `.tools` archive. Select individual tools
in the preview, then click **Import**. The selected tools are added to the current
library and receive free tool numbers; existing tools are retained. The preview
explains why an entry cannot be imported and notes any skipped presets. Flat,
ball-nose and bull-nose end mills are supported. Catalog information, dimensions,
flute count and valid named feed presets are imported; inch dimensions and feeds
are converted to millimeters. Presets are imported without a material link.

Cut direction is inferred from recognizable wording in the description; otherwise
it defaults to down-cut. Check the imported tool geometry, direction and cutting
data before use. Fusion coolant and preset step settings are not imported.
Importing an OpenCAMStudio `.ocam` library adds a separate library named after the file. Select a library in the left-hand list in Tools, or use the operation cutter picker. Right-click a library or use the menu button above the list to create, import, rename, export, or delete libraries. Deleting the last library restores a fresh default library. A new installation starts with only the default library; existing installations retain their previous library alongside the starter tools.

## Cooling

Tool defaults and cutting presets include **None** and **MMS**. Choosing a cutter
or applying a preset copies its cooling choice into the operation. The operation's
**Tool** tab can override it independently; Apply, Cancel, and undo include cooling.
Older tools, presets, and operations without this field default to **None**.

MMS switches on before an operation's cutting fragment and off after the fragment,
before tool changes, and at program end. This applies to clearing and carving
within a compound V-carve operation too. None never activates a coolant output.
For Estlcam, the machine's **Coolant / MMS outputs** settings select which output
is driven: output 1 uses M8/M9, output 2 uses M10/M11. Other posts emit their mist
commands (M7/M9). See [Estlcam export](estlcam.md).

## Cutting speed and feed per tooth

The operation's **Tool** tab and preset editor link **Cutting speed** (m/min) to
spindle RPM and **Feed per tooth** (mm/tooth) to cutting feed (mm/min):

- Cutting speed = π × cutter diameter (mm) × RPM / 1000.
- Feed per tooth = cutting feed / (RPM × flute count).

Editing either member of a pair updates the other immediately. Changing RPM also
recalculates feed per tooth, retaining the current cutting feed. Calculations use
the selected cutter's nominal diameter and flute count, and resolve RPM `0` to the
job's spindle speed. Cutters without a flute count cannot calculate feed per tooth.
The preset table shows both calculated values. Projects and presets save the linked
RPM and feed, so these values are restored without introducing independent,
potentially conflicting cutting data.

## Independent entry and exit feeds

The operation's **Tool** tab provides cutting and plunge feeds, plus independent
**Ramp feed**, **Lead-in feed**, and **Lead-out feed** where the strategy supports
them. Rates are in mm/min. New operations default ramp feed to `=plungeFeed` and
lead-in and lead-out feeds to `=feed`. For these three optional overrides, `0` inherits the
cutting feed. In a library preset, leave the corresponding field blank to leave
that operation setting unchanged when applying the preset.

Ramp feed controls inclined, helical and zigzag entries in profiles, pockets and
V-carving, including ramped lead-ins. Straight Z entries continue to use plunge
feed. Lead-in and lead-out feeds control generated entry and exit leads in closed
profiles, pockets, chamfers and thread milling. For a two-tool V-carve operation,
these overrides also apply to the clearing pass, using its cutting feed when unset.
A thread's helical cutting motion uses cutting feed; it is not a ramp entry.
Engraving, facing and drilling keep their existing cutting/plunge or axial feeds.

These settings change the feeds written into generated toolpaths and posted G-code;
changes are saved with the operation and can be undone.
