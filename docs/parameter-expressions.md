# Numeric parameter formulas

Numeric machining parameters support Excel-style formulas across Profile, Pocket,
Face, Drill, Bore, Thread, Chamfer, Engrave, Carve, 3D Adaptive and 3D Parallel/Scallop.
This includes cutting data, depths and stepdowns, stepovers and overlaps, allowances,
finishing and roughing passes, tolerances, angles, leads, ramping and entry positions.
Pass-count formulas must evaluate to whole numbers within the same limits as literal
counts. Geometry references, checkboxes, tool identifiers and selectors keep their
existing controls. Stock and reference-height settings use their dedicated editors.

Start a formula with `=`, for example:

- Stepdown: `=tool_diameter / 3`
- Feed: `=tool_diameter * 123.456789`
- Finishing spacing: `=Math.min(tool_diameter * 0.1; maximumStepover)`
- Roughing pass count: `=Math.ceil(tool_diameter / 4)`
- Cutting speed: `=1000 / 7`
- Feed per tooth: `=1 / 73`

The source is visible while editing. Enter or moving focus finishes the field and
displays its calculated value. Focus it again to restore the original formula
or unrounded numeric value. Enter does not apply the whole operation. Replace a formula with a
plain number to override it. Apply, Cancel, save/reopen and Undo/Redo retain the
bindings and overrides.

Right-click a parameter or open its **⋮** menu to see its current value.
The menu is available on individual numeric fields, checkboxes within sections,
and reusable dropdown choices. **Save as User Default** stores the formula,
number, or choice for that parameter and operation type. New operations of that
type use the saved default. **Reset to User Default** becomes available after
you save one.
**Reset to System Default** restores the programmed setting. The saved user
default stays available after a system reset. Use **Apply** to commit either
reset to an existing operation.
**Copy Parameter Name** copies the parameter's name. Numeric parameter names
can be used in formulas.

Typing letters in a formula shows the available matching variable names. Suggestions
are filtered at the caret, including inside function arguments. Click a suggestion,
or select it with ↑/↓ and insert it with Tab. Tab otherwise advances focus normally.

Tool variables are `tool_diameter`, `tool_cornerRadius`, `tool_length`,
`tool_fluteLength` and `tool_flutes`. Other variables are the named numeric parameters
of the current operation; autocomplete lists them. Formula values use the same units
as the field label: millimetres, degrees, percentages, RPM or mm/min. Do not enter
unit suffixes. Cutting speed is m/min; feed per tooth is mm/tooth.

Cutting-speed formulas drive spindle RPM. Feed-per-tooth formulas drive cutting feed
using flute count and spindle RPM. Entering a number or formula in the corresponding
RPM/feed field replaces that derived binding. A feed-per-tooth formula needs a cutter
with a flute count. When RPM was set to inherit the job default, entering such a
formula records that default as explicit RPM so its feed calculation is reproducible.
Other formulas referencing `cuttingSpeed` or `feedPerTooth` require explicit RPM (or a
cutting-speed/RPM formula).

**Preferences → number decimals affects display only.** Calculation, dependencies,
validation, toolpaths and saved values retain full floating-point precision. Changing
display accuracy never changes a numeric value or its formula. Integer counts display
as integers. Focus any decimal field to see its original unrounded source.

Supported arithmetic includes `+`, `-`, `*`, `/`, `%`, unary signs, parentheses,
decimal/scientific notation and `min`, `max`, `abs`, `sqrt`, `pow`, `floor`, `ceil`,
`round`. Functions accept an optional `Math.` prefix and comma or semicolon argument
separators. The evaluator executes only this arithmetic grammar, with bounded source
length, nesting and dependency depth. Unknown names, circular dependencies, division
by zero and results outside a parameter's existing limits block Apply and machining.
