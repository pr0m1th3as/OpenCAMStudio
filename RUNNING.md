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
- **Tooling** — the cross-project **tool library** manager (New · Delete). While
  this tab is active the Inspector becomes the library editor.
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
  toolpath). The Inspector is instead the **library editor** while the Tooling tab
  is active, and the **operation wizard** while a pick is pending.
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

- Cross-project and persistent, stored in the platform config directory
  (`~/.config/OpenCAMStudio/tools.json` on Linux; `%APPDATA%` on Windows;
  `~/Library/Application Support` on macOS). Seeded with a few default end mills
  on first run.
- **Tooling** tab: **New** adds a tool, **Delete** removes the selected one; select
  a library tool to edit its **diameter / length / flutes / kind** in the Inspector
  (saved to disk immediately).
- A project **embeds copies** of the tools it uses, so `.ocam` files stay
  self-contained; the library is the template you pick from.

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
- **Tool library:** Tooling tab → New / Delete; edit a tool's fields; **restart
  the app** and confirm the edits persisted (`tools.json`).
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
