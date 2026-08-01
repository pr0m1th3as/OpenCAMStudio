# Building & running OpenCAMStudio

OpenCAMStudio is a Cargo workspace. Everything except the desktop GUI is
headless and needs no system libraries; the GUI (iced + wgpu) needs a graphics
stack and is behind an opt-in feature.

## Prerequisites

- A recent stable Rust toolchain (`rustup`; the workspace pins the `stable`
  channel). `rustc --version` should be ≥ 1.88.
- For the **GUI only**: a working display and GPU/GL/Vulkan drivers, plus a few
  system libraries (see below).

## Headless (no GUI) — build, test, run

This is what CI runs, and it works anywhere with no extra packages:

```bash
cargo build --workspace
cargo test  --workspace          # the full test suite
cargo run   -p cam-app           # prints how to launch the GUI
```

### A note on the debug profile

The workspace `Cargo.toml` sets `[profile.dev.package."*"] debug = false` — debug
info is **off for dependencies**, on for our own crates. This is not a micro-
optimisation: with it on, a `--features gui` debug binary is 435 MiB of which
335 MiB is DWARF, each link costs the linker ~1.7 GiB of RSS, and
`cargo build --workspace --all-targets` links a dozen of those at once. That
reliably exhausted a 31 GiB machine. With it off the binary is 168 MiB and a cold
`--all-targets` build peaks around 7 GiB.

You keep breakpoints, variables and line numbers in `cam-*` code, and dependency
frames still carry symbol names in backtraces. If you genuinely need to step *into*
`wgpu`/`iced`/`naga`, comment those two lines out for that session.

## The desktop GUI

The interactive app lives behind the `gui` feature (which also enables
`cam-render`'s `gpu` feature):

```bash
cargo run -p cam-app --features gui
```

### System libraries (Linux)

On Debian/Ubuntu, install these once. On other distros use the equivalents; on
macOS and Windows no extra packages are needed.

```bash
sudo apt-get install -y \
  pkg-config libx11-dev libxkbcommon-dev libwayland-dev \
  libxcursor-dev libxrandr-dev libxi-dev \
  libgl1-mesa-dev libegl1-mesa-dev libfontconfig1-dev
```

If the window fails to create a GPU surface (headless box, VM, or a driver
issue), force a software renderer:

```bash
LIBGL_ALWAYS_SOFTWARE=1 WGPU_BACKEND=gl cargo run -p cam-app --features gui
```

### The window

Across the top is a tabbed **icon ribbon**; below it are three docked panes —
**Project** (left), **Viewport** (centre), **Inspector** (right) — over an
**Output** console (bottom).

- **Layout is stable under resize.** Project, Inspector, and Output hold their
  size when you resize the window; only the Viewport grows or shrinks. Drag a
  divider to set a side pane's size — that size is remembered across later window
  resizes. Panes are framed with visible separators.
- The **Viewport is always visible.** The **View → Panes** group has a checkbox per
  pane to show/hide **Project / Tools / Inspector / Output** (not the Viewport);
  hiding one hands its room to the Viewport.

### The ribbon

- **Home** — *Project* (New · Open · Save · Save As), *Data* (Import · Export ·
  Sample), *Edit* (Undo · Redo · Run). New/Open/Save/Import/Export use native
  file dialogs; **Sample** loads a built-in rectangle-with-a-hole demo (no dialog).
- **Operations** — *Create*: Face · Profile · Pocket · Drill · Thread · Chamfer ·
  Engrave · **Carve** — roughly the order a part is made. Clicking a kind starts
  the operation-creation wizard.
- **Tooling** — the cross-project **tool library** manager (New · Delete ·
  Renumber · Import Library · Export Library). While this tab is active the
  **Tool Library pane** replaces the Project pane, the Inspector becomes the tool
  editor, and the Viewport shows a **2D cross-section** of the selected tool.
- **View** — Show stock · Reset view · Cube on/off · Origin · Tips, then the
  orientation-cube **size slider**, then **Panes** — the pane show/hide checkboxes
  (the Tool Library is listed as *Tools* to keep the band narrow).

Tabs read left to right in workflow order: **Home → Edit → Operations → Tooling →
View**.

The ribbon collapses responsively as the window narrows (groups degrade
right-to-left; a collapsed group opens as a popup under its button).

### The panes

- **Project** (left) — the project tree: **Setup**, **Stock**, **Tools (in use)**,
  and **Operations**. Click a row to select it (the selected row is highlighted).
  - **Tools (in use)** is *read-only* and lists only the tools referenced by an
    operation — tools are chosen from the library during op setup, not here.
  - Each **operation** row has an include **checkbox** (untick to exclude it from
    the toolpath and simulation — it stays in the tree, marked *(excluded)*),
    inline **↑ / ↓** reorder arrows, and a **right-click menu** with **Duplicate**,
    **Delete** and **Reinitialize**.
  - On a job with more than one **workpiece origin**, that same right-click menu also
    lists **Move to Origin *n*** — one row per origin the operation is not already in,
    named the way the selected post will write it (`Origin 2 · G55`, or `· H2` on
    Okuma). This moves the operation into that origin's group in the tree, so it posts
    under that work offset. The ↑ / ↓ arrows reorder *within* a group; this is how an
    operation crosses between them.
  - **Reinitialize** re-runs the creation wizard for that operation and replaces it
    **in place**, keeping its id and its position in the job. This is the *only* way
    to change an existing operation's tool: the tool is bound at creation, alongside
    the geometry.
  - An operation whose last run **failed** is marked with a red **⚠**; hover it for
    the reason. It stays in the tree so you can fix the offending field — but export
    is blocked while any error stands, so a marked row can never reach the machine.
- **Viewport** (centre) — a native 3D `wgpu` view of the backplot and simulated
  stock. **Left-drag** orbits (a turntable, unclamped, so you can tilt to the
  underside), **right-drag** pans, the **wheel** zooms; it opens top-down. A
  rotating **orientation cube** sits top-right — click a face to snap the view
  square onto that side; **View → Reset view** returns to top; **View → Cube**
  toggles it. While an op pick is pending, a **pickbox** square + crosshair
  follows the cursor.
- **Inspector** (right) — editable fields for the *selected* node (Setup exposes
  Clearance / Retract / Top of stock; an Operation exposes Depth / Stepdown /
  (Stepover) / Feed / Plunge feed plus **Side / Lead-in / Lead-out / Plunge**
  pickers). Press **Enter** or **Apply** to commit (one undo step, recomputes the
  toolpath). While the Tooling tab is active the Inspector is instead the **tool
  editor** (kind-aware fields + a Type picker; edits preview live in the Viewport
  and **Apply** commits, greyed until you actually change something), and while a
  pick is pending it is the **operation wizard**.
- **Output** (bottom) — the status line and run / collision diagnostics.

### Creating an operation

1. Load geometry — **Home → Sample**, or **Home → Import** a `.dxf`/`.dwg`. A real
   import comes in as **geometry only**: no operations and no tools yet.
2. **Operations** tab → pick a kind. The Inspector becomes the wizard.
3. Choose a **Tool family**, then a **Tool** within it. The families offered are
   bounded by the operation — a pocket lists only end mills, engraving only V-bits —
   so a library of hundreds stays usable. If a family is empty the wizard says so and
   points you at the Tooling tab.
4. **Click the geometry** in the Viewport. Picking is line-based — click the outer
   contour, an inner hole, or an imported **open stroke** (engraving only); the click
   snaps to the nearest loop edge, and the picked vertex sets the toolpath start.
5. **Tool and geometry may be chosen in either order**, and either may be changed
   until you commit: re-picking simply moves the boundary, and changing family clears
   the tool. Nothing is created until **Confirm**, which stays disabled until both are
   settled.
6. **Pocket and Carve** enter **island mode** once a boundary is picked: click
   enclosed loops to toggle them as excluded islands (highlighted gold), then
   **Confirm**. A Carve *starts* with the picked region's own holes already excluded
   — the counters of letters, which are never wanted — so for most drawings there is
   nothing to click.
7. The operation appears in the tree, and its tool under **Tools (in use)**. Choosing
   a tool embeds a copy into the project, so `.ocam` files stay self-contained.
8. Select the operation and edit its fields / Side / lead / plunge in the Inspector;
   **Apply** to recompute. The **tool is not editable here** — use Reinitialize.

### Adaptive stepover — and why it is not a hard maximum

A Pocket, and an outside Profile's roughing, can clear the bulk **adaptively**: instead
of concentric rings at a fixed spacing, the tool advances a front that keeps its radial
width of cut roughly constant. Set **Adaptive stepover** above `0` to turn it on (`0` is
plain concentric clearing); it is **climb-only**, and requesting a lead-in/out sends the
whole clear back to concentric.

The number you type is the **straight-wall stepover** — the width the tool takes where
the path runs straight. It is *not* a ceiling the toolpath will never exceed, and the
reason is geometry rather than a shortcoming of the generator:

- Where the path curves tightly, the swept disc overlaps the uncut region by more than
  it does on a straight run. The floor is
  `a_e(ρ) = e·(ρ + r)/ρ − e²/(2ρ)` for a path of radius `ρ` with tool radius `r` and
  requested stepover `e` — which exceeds `e` at **every** finite radius, and reaches
  about **1.4·e** on the tight loops near a pocket's centre.
- No spiral clearer can beat that bound; it is a property of the shape, not of the
  strategy. Asking for a smaller stepover lowers the floor proportionally but never
  removes it.

What adaptive clearing actually buys you is the elimination of the **full-diameter
slot** — the pass that engages the tool on both flanks at once, which is the condition
that breaks cutters. Those are gone. The residual rise on tight loops is a **feed-rate**
matter: if the tool complains in the corners, slow the feed rather than chasing the
stepover down. (Automatic per-loop feed compensation is not implemented; it is the
natural next step and is logged.)

### Engraving

Engraving cuts a V-section groove with the tool centred **on** the path — no side, no
radius compensation. It is a different operation from chamfering, and the difference is
physical: a **chamfer mill's tip is a flat that does not cut**, so it would rub; a
**V-bit's tip is rounded and does cut**. Engraving therefore requires a **V-bit**, and
selecting a chamfer mill is refused with that reason.

- The groove's **width follows from the depth and the bit**, not from a separate
  setting. The program comment states it, e.g. a 60° V-bit with a 0.1 mm tip radius
  cutting 0.4 mm deep gives a 0.577 mm groove.
- **Depth** is limited by where the cone flares out to the tool's full cutting
  diameter; past that the shank would rub, and the operation is refused.
- **Stepdown** `0` cuts the full depth in one pass (normal for a surface mark);
  otherwise the depth is stepped, landing exactly on the target.
- **Open strokes** (imported lettering or decorative paths) engrave open — the tool
  lifts at the far end rather than closing back to the start. Closed loops close.
  Note the importer reads **LINE / ARC / CIRCLE / LWPOLYLINE** only: text must be
  exploded to curves in your CAD package, and splines are skipped.

### Carving

Carving is to engraving what a pocket is to a profile: the boundary outlines an
**area**, not a path. The tool never touches that boundary — it stands back and lets
its **flanks** land on it — and the depth is not a setting you dial in but a
consequence of how wide the shape is at each point. It needs a **V-bit**, for the same
physical reason engraving does, and it needs a **closed** boundary: there is nothing to
carve without an interior.

- **Depth is a cap, not a command.** The shape reaches it only where it is wide
  enough. The Inspector says what the shape itself allows, live, before you run:
  *"Full depth for this shape is 3.80 mm; this cap leaves 3 flat areas."* That one
  line tells you whether to deepen, accept, or add the second tool.
- **Islands come from the drawing.** Pick a region and the counters of its letters
  are excluded automatically, from the geometry's own nesting. Click one to toggle
  it back.
- **Ring step** is a **roughing** control, despite appearances. The finished wall is
  cut by the *deepest* ring alone — its flank spans from the boundary down to its
  tip — so the shallower rings limit how much one pass takes and reach into corners.
  Coarser costs tool load, not surface quality.
- **Floor scallop** is the real finish control, and applies where a cone genuinely
  cannot do better: a **flat floor**. Adjacent passes leave a ridge between them, so
  you ask for the ridge height and the spacing follows — which lets a rounded tip
  step much wider than a sharp one for the same finish.
- **Stay down** links the rings without lifting wherever that is safe. Each link is
  checked, and any that would gouge — or that would drag the tip across floor the
  clearing tool has already finished — lifts instead.

#### Clearing the flat areas (the second tool)

Where the depth cap leaves a flat floor, a cone cannot flatten it: adjacent passes
leave ridges. Tick **Clear flat areas** and an end mill takes that floor **first**,
before the V-bit runs — bulk removal with the strong tool, which also spares a fine
carving tip from full-depth work. The Inspector then shows a second block of controls
below the clearing tool, which are a pocket's: stepdown, overlap, allowance,
engagement, feeds, plunge style, climb, leads.

- The clearing tool must be **flat-bottomed** (end mill or bull nose). A ball nose is
  refused — it cuts, but leaves the same scalloped floor the V-bit already would, so
  it buys a tool change and nothing else.
- **Each stepdown level clears to its own depth's width**, not the bottom's, so the
  end mill roughs the taper as a staircase instead of leaving it all to the V-bit.
- The **clear allowance** is how far the end mill stays off the carved surface,
  leaving that skin for the V-bit — which finishes it better, with the flank of its
  cone rather than the corner of a cylinder. Nothing is abandoned: the V-bit's own
  passes are computed from what the clearing tool *actually swept*, so a larger
  allowance simply hands it more to do.
- A round cutter cannot enter a sharp corner, so it leaves a lens of stock at every
  concave corner of the flat area. The V-bit cleans those afterwards, and only those.
- Both tools appear on the operation's row in the Project pane, in cutting order
  (`3: Carve  T4 + T7`), and both count as in use.

### Tools must be able to cut

Every operation checks that the surface doing the cutting *is* a cutting surface,
reading the tool's own profile rather than its type:

- **Errors** (no G-code produced) when a **non-cutting** surface would do the work —
  engraving or drilling with a chamfer mill, profiling with a V-bit (its cone has no
  cylindrical flank, so it cannot cut a vertical wall at any depth), threading with
  anything but a thread mill — or when the cut runs **past the flute length** onto the
  shank.
- **Warns** but proceeds when the tool cuts, just not well — facing with a ball-nose
  leaves a scalloped floor; plunging an end mill as a drill has no point geometry to
  centre the hole.

The reason appears in the **Output** pane and against the operation in the tree.

### The tool library

Cross-project and persistent, stored in the platform config directory
(`~/.config/OpenCAMStudio/` on Linux; `%APPDATA%` on Windows; `~/Library/
Application Support` on macOS). Seeded with a few default end mills on first run.
A project **embeds copies** of the tools it uses, so `.ocam` files stay
self-contained; the library is the template you pick from.

A saved project records **the machine it was built for and the post it exports
through** — but only as *provenance*. **Opening a project never changes your machine or
your post.**

That is deliberate and it is a safety rule. A machine is local to your shop; a project
file travels. The machine's envelope is exactly what an export is checked against, so a
file that could set your machine could disarm that check: a job authored on a 1000 mm
router, opened by someone with a 300 mm mill, would otherwise be verified against the
*sender's* travel and pass. Which control you cut on is local for the same reason.

What the file recorded is still worth knowing, so opening a job built for something else
says so:

> ⚠ This job was built for "Big Router" (1000×600×200 mm); yours is "Small Mill"
> (300×200×100 mm), and built for the Okuma post; yours is grbl. Your own machine and
> post are unchanged — check it fits before cutting.

Nothing is said when they match. One consequence worth expecting rather than reporting:
the tool-change height falls back to the machine's maximum Z, so opening a shared job on
a different machine legitimately changes the height of its lifts.

- **Tool Library pane** (replaces Project while the Tooling tab is active): every
  library tool, in two views — **Ordered** (by number, `T1: ⌀6 …`) or **Grouped**
  (by family, then size). Right-click a row → **Set number…** (swaps if the number
  is taken). Select a tool to edit it.
- **Ribbon actions:** **New** (adds a tool, seeding its *type* from the current
  selection and taking the lowest free number), **Delete**, **Renumber** (guarded
  bulk 1…N in the pane's current order), **Import / Export Library** (a `.ocam`
  tagged as a library, distinct from a project `.ocam`).
- **Editing is live.** Pick a tool → the Inspector shows a **Type** picker plus the
  fields that kind actually has, and the Viewport draws the tool's **2D cross-
  section** (mirrored silhouette; **solid = cutting** surface, **dashed = non-
  cutting** shank/neck/tip — never colour alone). Every field or picker change
  updates the preview immediately; **Apply** commits it to the library on disk and
  is greyed until there is an unsaved change.
- **Tool kinds**, each with its own fields, validation, and silhouette:
  - **End mills** — Square, Ball-nose, Rounded-edge (bull-nose; adds a corner
    radius). Flute ⌀ / flute length / shank ⌀ / shank length / overall / flutes,
    plus a **cutting direction** (Down / Up, and Straight for square) shown as the
    flute-helix lean.
  - **Drill bit** — adds a point angle (bounded 90–135°); the helix reads as a
    right-hand twist.
  - **V-bit** — a single shaft ⌀ + point angle + **rounded** tip radius (the cone
    flares exactly to the shaft). The rounded tip *cuts*, which is what lets a V-bit
    engrave.
  - **Chamfer mill** — a cone with a flat, **non-cutting** tip (only the angled
    flank cuts). This is exactly what separates it from a V-bit, and why it cannot
    engrave. Both tips have a physical **minimum size** — no cutter is ground to a
    true zero point — so a V-bit's tip radius must be ≥ 0.05 mm and a chamfer mill's
    tip ⌀ ≥ 0.10 mm; below that the field is flagged and **Apply** stays disabled.
  - **Face mill** — a shell mill: cutting ⌀, body height, arbor ⌀, overall, and an
    insert count drawn as a row of 90° inserts on the body.
  - **Thread mill** — a **Single-point / Full-form** toggle. Single-point (single
    profile) has a min cutting ⌀, a reduced neck (which sets the max thread depth),
    and a length of cut, drawn as one 60° tooth on a long reduced neck. Full-form
    has a cutting ⌀, thread length, and pitch, drawn as a stack of 60° threads.

### Preferences

**Preferences** sits beside **About**, right of the ribbon tabs. It holds the settings
that have no other control:

- **Pickbox size** — the aperture of the square that follows the cursor during a pick.
  Its half-size is also the vertex-snap tolerance. The object-snap catch distance is
  shown beneath it but is **not separately settable**: it stays a fixed 1.5× the
  pickbox, because two independent numbers would let you set a catch distance smaller
  than the box feeding it.
- **Snap marker size** — how large the engaged snap glyph draws. Unlike the catch
  distance this *is* its own control: it is a visual size, not a tolerance, so a large
  marker with a tight aperture is a reasonable thing to want.
- **Origin marker size** — the workpiece-datum cross-and-ring. It is sized from the
  scene (6% of its extent), so it already stays legible on a 20 mm part and a 500 mm
  one; this scales that, for a louder or quieter datum against a busy backplot.
- **Smallest a pane may be** — the five per-pane minimums. **Project, Tools, Viewport
  and Inspector are widths; Output is a height** — they dock to the sides and the
  bottom respectively, so the same number bounds a different axis. The labels say
  which. These are **logical** pixels, so a high-DPI screen with display scaling is
  already handled; raise them on a large screen, lower them on a small or unscaled one
  where the shipped values can leave the viewport too narrow to work in.
The **post** is deliberately *not* here. It has its own control in the Machine ribbon
group, and it is remembered between runs as where you left off — not nominated as a
default. Neither creating a project nor opening someone else's changes it: which control
you cut on is a property of your shop.

Changes apply as you drag, and are written when you let go. **Restore defaults** puts
everything back — including the pane sizes and minimums, since it is the way out of a
layout that has left no room for the viewport.

Everything else that is remembered has its control where you already use it: the View
toggles, the cube-size slider, the armed object snaps and the pane sizes you drag to
are all **remembered between runs** without appearing here. Duplicating them in a panel
would create two places to change one thing.

All of it lives in `<config-dir>/OpenCAMStudio/settings.json`, beside `tools.json`.

The file is versioned, and a file it cannot read is **never overwritten**: an
unparseable one, or one written by a newer build, leaves you on the defaults with your
file intact. Delete it to return to the shipped defaults.

The same protection now covers the **tool library**. `tools.json` previously fell back
to the starter 36 tools *and saved them* if it failed to parse — silently replacing a
hand-built library. It now leaves the file alone, keeps a `tools.json.bak` copy beside
it, and says so in the status bar at startup.

### Plunge styles — how the tool gets down

Set per operation in the Inspector. All four reach exactly the requested depth; they
differ in how the tool gets there, and only **Straight** puts the tool's tip into
solid material with nowhere to go.

- **Straight** — a vertical drop. Needs a centre-cutting tool or a pre-drilled hole.
- **Ramp along path** — descends **along the toolpath itself**, one way, arriving on
  the contour at full depth. It enters the loop *before* the pass's start point, so
  the stretch it leaves sloped is the loop's own final stretch, which the pass then
  re-machines at full depth: no extra motion, and nothing left standing. The angle is
  from horizontal — shallower is gentler and travels further. A very shallow angle on
  a small loop wraps around it repeatedly; past 32 laps the ramp steepens to fit
  rather than growing without bound.
- **Zig-zag** — oscillates back and forth in place, for a slot too narrow to ramp
  along. This is the one case an oscillating entry is actually for.
- **Helix** — spirals down on its own radius, clear of the wall.

A clearing pass is the exception to "one way": its path is **open** — it never returns
to where it entered — so a one-way ramp there would strand the wedge it leaves. The
ramp runs forward along the path and retraces it at depth instead. A closed contour
(profile, carve wall rings) needs no such return.

### Run / export

- **Home → Run** recomputes for the current document. The backplot is coloured by
  move kind: green = cutting, yellow = rapid/link, red = plunge, **blue dashed =
  tool-change traverse** (the lift to tool-change height, the cross, and the descent
  back to clearance); the part outline is light grey. The dash marks the one move the
  *operator* never asked for — the planner inserted it — so colour says which kind of
  move it is and the dash says who put it there. Rapids are deliberately **not**
  dashed: a rapid is still a move the operation implies. The dash is a world-space
  pattern sized from the scene, so zooming far in stretches it back into a solid blue
  line. **Output** shows toolpath diagnostics (e.g. a tool too large)
  *and* material-removal **collisions** from the simulation (e.g. a rapid plowing
  through remaining stock, which a green backplot would hide).
- **View → Show stock** overlays the *simulated* stock surface under the backplot.
  The simulation grid **refines to the narrowest cut in the program**, so a
  sub-millimetre engraved groove is actually visible rather than falling between
  cells; it is capped so refining cannot turn a preview into a hang.
- **Home → Export** posts the toolpath to the selected controller dialect. It is
  blocked if the run had errors **or** the simulation found a rapid through stock;
  the status line says why. Otherwise it reports the line count. Program comments are
  reduced to printable ASCII (`°` becomes `deg`, `⌀` becomes `dia`) — grbl tolerates
  UTF-8, Fanuc/Haas controls and 7-bit DNC links often do not.
- **Undo / Redo** step through document edits.

### What to check when testing the GUI

Because the GUI is the one part that cannot be verified by automated tests, the
things worth eyeballing:

- **The Preferences panel:** open it (beside About). Drag **Pickbox size** and watch
  the pick square change *while you drag* — then check the catch-distance line beneath
  it tracks the pickbox rather than standing still. Drag a **pane minimum** up and
  confirm that divider now refuses to go past it. Then the escape hatch: push every
  pane minimum to maximum until the viewport is unusable, and confirm **Restore
  defaults** gets you back in one click without a confirmation step you might not be
  able to reach. Check the pane rows name their axis — **Output (height)**, the rest
  **(width)**. Drag **Origin marker size** and watch the datum cross grow.
- **Machine and control stay yours.** Change the post in the Machine ribbon, restart, and
  confirm it came back. Then open a `.ocam` saved with a *different* machine and post —
  yours must be untouched, and the status line must say what the job was built for.
- **Preferences that stick:** change the View-tab toggles (Stock / Cube / Origin /
  Tips), drag the cube-size slider, arm or disarm a couple of object snaps, and drag
  the pane dividers. **Restart.** All of it should come back as you left it — they are
  written to `<config-dir>/OpenCAMStudio/settings.json`. Then the case that matters:
  drag the dividers wide, restart with the **window made much narrower**, and confirm
  the layout degrades to fit rather than squeezing the Viewport to nothing. Deleting
  `settings.json` must bring back the shipped defaults; a *corrupt* one must leave you
  with defaults and the file untouched (nothing should overwrite it).
- **Dashed tool-change traverse:** build a job with **two tools** (or two origins),
  Run, and look at the blue moves — the lift to tool-change height, the cross, the
  descent. They must be **dashed**; everything else stays solid, rapids included.
  Then check the pattern is legible at the scale you work at: on a small part and on
  a large one the dashes should look about the same size, because the period comes
  from the scene, not from a fixed millimetre count. **Zoom right in** — the dashes
  stretch and the line goes solid blue. That is expected (the pattern is world-space,
  not screen-space), not a bug. What would be a bug: the viewport freezing or memory
  climbing while a backplot with traverses is on screen — that is the failure mode
  the walk used to have, so it is worth a glance at the memory figure.
- **Layout:** the window opens with the ribbon + Project / Viewport / Inspector /
  Output. Resizing the window changes **only** the Viewport; the side panes and
  Output keep their size. Drag a side divider, then resize the window — the pane
  keeps its dragged size. Separators are visible. **View → Panes** hides/shows
  Project / Tools / Inspector / Output but never the Viewport.
- **Project tree:** rows read as a tree (plain text, selected row highlighted —
  not blue buttons). **Right-click an operation** → a Delete / Duplicate /
  Reinitialize menu at the cursor; clicking off dismisses it. The include checkbox
  and ↑ / ↓ work. Reinitialize should replace the op **in place**, keeping its
  position in the list.
- **Tools (in use):** after creating an op, its tool appears here read-only; with
  no ops the section shows "(none yet …)".
- **Operation wizard:** Operations → Profile, choose a family then a tool, click
  the outer rectangle → it profiles the rectangle; click the inner circle → the
  circle. Then check the order does not matter: start again, click the geometry
  **first**, and confirm **Confirm** stays disabled until a tool is chosen. Re-pick a
  different loop before confirming and check the selection moves. A Pocket enters
  island mode (click islands gold, Confirm). A **Carve** should arrive with the
  region's holes *already* excluded.
- **Carve:** create one on a region with an island. The Inspector should state what
  the shape allows before any run ("full depth for this shape is …"), and update as
  you edit **Max depth**. Tick **Clear flat areas**: a rule and a *Clearing pass
  (end mill)* heading appear, with the tool picker and then a pocket's worth of
  fields **below** it — nothing clearing-related mixed in with the V-bit's own
  numbers above. Deepen the cap until the line says no flat areas remain and the
  whole block should vanish. The Project row should read `Carve  T… + T…` in cutting
  order. In the backplot the V-bit should link its rings without lifting, but **lift**
  rather than skim across floor the end mill has already finished.
- **Chamfer:** the Inspector's first field is **Top edge Z**, seeded from the top of
  stock. Lower it and the whole bevel should move down with it in the backplot —
  that is the case of chamfering a pocket rim or a step, which had no way to be
  stated before. A negative value must be accepted, not clamped.
- **Thread:** with **Passes** above 1, the **Gradual** checkbox should visibly change the
  infeed radii in the backplot — the early passes step further, the last one least. Both
  settings must still finish at exactly full depth.
- **Guards:** point an operation at a tool that cannot do the job — face with a
  chamfer mill, profile with a V-bit, thread with an end mill — and confirm the
  Output pane explains why and the operation is marked ⚠ in the tree, while merely
  poor choices (facing with a ball-nose) only warn.
- **Tool library:** Tooling tab → the Tool Library pane replaces Project and the
  Viewport shows the selected tool's 2D cross-section. New / Delete; switch the
  **Type** and watch the field set + silhouette change live; edit a field and
  confirm the preview updates *before* Apply, that **Apply** is greyed until you
  change something (and greys again if you revert), and that cutting surfaces draw
  solid vs dashed shanks. Try Renumber and right-click **Set number…**. **Export
  Library** then **Import Library** and confirm the round-trip. **Restart the app**
  and confirm edits persisted.
- **Files:** Import a real `.dwg` → geometry only (no ops). Save an `.ocam`,
  reopen it → geometry and embedded tools survive.
- **Viewport:** left-drag orbits (turntable, all the way to the underside), right-
  drag pans, wheel zooms; the orientation cube tracks the view and its faces snap.
  With **Show stock** on, the solid occludes itself while the toolpath stays on top.

## Notes

- The `gui` feature is **not** enabled by default so that `cargo test
  --workspace` and the headless controller stay GPU-free and fully testable.
- `cam-app`'s logic lives in the headless `AppController` (unit-tested); the iced
  layer is a thin view over it. The tool library
  (`cam-app/src/tool_library.rs`) is the one piece of GUI-side persistent state.
