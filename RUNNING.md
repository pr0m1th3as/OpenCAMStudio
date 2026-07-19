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
- **Operations** — *Create*: Profile · Pocket · Drill · Thread · Chamfer ·
  **Engrave** · Face. Clicking a kind starts the operation-creation wizard.
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
6. **Pocket** enters **island mode** once a boundary is picked: click enclosed loops
   to toggle them as excluded islands (highlighted gold), then **Confirm**.
7. The operation appears in the tree, and its tool under **Tools (in use)**. Choosing
   a tool embeds a copy into the project, so `.ocam` files stay self-contained.
8. Select the operation and edit its fields / Side / lead / plunge in the Inspector;
   **Apply** to recompute. The **tool is not editable here** — use Reinitialize.

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

### Run / export

- **Home → Run** recomputes for the current document. The backplot is coloured by
  move kind: green = cutting, yellow = rapid/link, red = plunge; the part outline
  is light grey. **Output** shows toolpath diagnostics (e.g. a tool too large)
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
  island mode (click islands gold, Confirm).
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
