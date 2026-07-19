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
- The **Viewport is always visible.** The **Windows** ribbon tab has a checkbox
  per pane to show/hide **Project / Inspector / Output** (not the Viewport);
  hiding one hands its room to the Viewport.

### The ribbon

- **Home** — *Project* (New · Open · Save · Save As), *Data* (Import · Export ·
  Sample), *Edit* (Undo · Redo · Run). New/Open/Save/Import/Export use native
  file dialogs; **Sample** loads a built-in rectangle-with-a-hole demo (no dialog).
- **Operations** — *Create*: Profile · Pocket · Drill · **Thread** · Face.
  Clicking a kind starts the operation-creation wizard. (Thread is a placeholder
  for now — it reports "not yet implemented".)
- **Tooling** — the cross-project **tool library** manager (New · Delete ·
  Renumber · Import Library · Export Library). While this tab is active the
  **Tool Library pane** replaces the Project pane, the Inspector becomes the tool
  editor, and the Viewport shows a **2D cross-section** of the selected tool.
- **View** — Show stock · Reset view · Cube on/off.
- **Windows** — the pane show/hide checkboxes.

The ribbon collapses responsively as the window narrows (groups degrade
right-to-left; a collapsed group opens as a popup under its button).

### The panes

- **Project** (left) — the project tree: **Setup**, **Stock**, **Tools (in use)**,
  and **Operations**. Click a row to select it (the selected row is highlighted).
  - **Tools (in use)** is *read-only* and lists only the tools referenced by an
    operation — tools are chosen from the library during op setup, not here.
  - Each **operation** row has an include **checkbox** (untick to exclude it from
    the toolpath and simulation — it stays in the tree, marked *(excluded)*),
    inline **↑ / ↓** reorder arrows, and a **right-click menu** with **Duplicate**
    and **Delete**.
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
2. **Operations** tab → pick a kind. The Inspector becomes the wizard: choose a
   **tool from the library** dropdown (or **＋ New** to add one), then **click a
   boundary line** in the Viewport. Picking is line-based — click the outer
   contour *or* an inner hole; the click snaps to the nearest loop edge, and the
   picked vertex sets the toolpath start. Choosing the tool embeds a copy into the
   project.
3. **Pocket** enters **island mode** after the boundary pick: click enclosed loops
   to toggle them as excluded islands (highlighted gold), then **Confirm**.
4. The operation appears in the tree, and its tool under **Tools (in use)**.
5. Select the operation and edit its fields / Side / lead / plunge in the
   Inspector; **Apply** to recompute.

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
  - **V-bit** — a single shaft ⌀ + point angle + tip radius (the cone flares
    exactly to the shaft).
  - **Chamfer mill** — a V-bit with a flat, **non-cutting** tip (only the angled
    flank cuts).
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
- **Home → Export** posts the toolpath to grbl G-code. It is blocked if the run
  had errors **or** the simulation found a rapid through stock; the status line
  says why. Otherwise it reports the line count.
- **Undo / Redo** step through document edits.

### What to check when testing the GUI

Because the GUI is the one part that cannot be verified by automated tests, the
things worth eyeballing:

- **Layout:** the window opens with the ribbon + Project / Viewport / Inspector /
  Output. Resizing the window changes **only** the Viewport; the side panes and
  Output keep their size. Drag a side divider, then resize the window — the pane
  keeps its dragged size. Separators are visible. The **Windows** tab hides/shows
  Project / Inspector / Output but never the Viewport.
- **Project tree:** rows read as a tree (plain text, selected row highlighted —
  not blue buttons). **Right-click an operation** → a Duplicate / Delete menu at
  the cursor; clicking off dismisses it. The include checkbox and ↑ / ↓ work.
- **Tools (in use):** after creating an op, its tool appears here read-only; with
  no ops the section shows "(none yet …)".
- **Operation wizard:** Operations → Profile, pick a tool from the library, click
  the outer rectangle → it profiles the rectangle; click the inner circle → the
  circle. A Pocket enters island mode (click islands gold, Confirm).
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
