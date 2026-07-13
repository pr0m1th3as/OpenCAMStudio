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

### Using the app

The window has a **toolbar** across the top and four **resizable, dockable
panes** below (drag a pane's title bar to rearrange; drag a border to resize):

- **Project** (left) — the document tree: Setup, Stock, Tools, and Operations.
  Click a node to select it; the selected node is marked with `▸`.
- **Viewport** (centre) — a native 3D `wgpu` view of the backplot and simulated
  stock. **Left-drag** orbits (a turntable: horizontal spins about the vertical
  axis, vertical tilts) with **no clamp**, so you can tilt all the way over to
  the underside while the horizon stays level; **right-drag** pans, the **scroll
  wheel** zooms; it opens top-down. A rotating **orientation cube** sits in the
  top-right corner — chamfered edges, colour-coded faces each labelled with the
  full word (**TOP / BOTTOM / FRONT / BACK / LEFT / RIGHT**) — turning with the
  view so you always know which side you are looking at. **Click a cube face** to
  snap the view square onto that side. The toolbar's **Reset view** returns to
  the top view; **Cube: on/off** toggles the cube.
- **Inspector** (right) — editable fields for the *selected* node.
- **Output** (bottom) — the status line and run diagnostics.

Workflow:

1. **Open sample** — loads a built-in rectangle-with-a-hole (no file dialog yet;
   file loading is the next increment). The tree fills with two profile
   operations and the viewport frames the geometry.
2. Select a node in **Project**, then edit its fields in **Inspector**:
   - a **Setup** exposes Clearance / Retract / Top of stock;
   - a **Tool** exposes its diameter;
   - an **Operation** exposes Depth / Stepdown / (Stepover) / Feed / Plunge feed.
   Press **Enter** or **Apply** to commit — each Apply is one undo step and
   recomputes the toolpath.
3. **Run** — recomputes for the current document. The backplot is colored by
   move kind: green = cutting, yellow = rapid/link, red = plunge; the part
   outline is light grey. Problems appear in **Output** (e.g. a tool too large).
4. **Undo / Redo** — step through document edits.
5. **Show stock / Hide stock** — overlays the *simulated* stock surface (the
   material left after the toolpath cuts) under the backplot, shaded so the
   pocket walls and stepdowns read. Available after a **Run**.
6. **Export .nc** — posts the toolpath to grbl G-code (blocked if the run had
   errors). The status line reports the line count.

### What to check when testing the GUI

Because the GUI is the one part that cannot be verified by automated tests, the
things worth eyeballing:

- The window opens with all four panes; the sample part frames in the viewport
  and the tree shows Setup / Stock / Tools / two Operations.
- Selecting **Operation 0** shows its Depth/Stepdown/Feed in the Inspector;
  changing **Depth** to `-8` and pressing **Apply** updates the backplot.
- Selecting the **Tool** and setting ⌀ to `12`, then **Apply**, produces a
  "tool too large" diagnostic in **Output** and blocks export (the 6 mm-radius
  tool can't open the 10 mm hole).
- **Undo** restores the previous value and the backplot updates.
- **Show stock** overlays a shaded grey surface with the pockets/holes carved
  out; the colored backplot stays visible on top. **Hide stock** removes it.
- **Left-drag** orbits as a turntable — horizontal spins about the vertical axis,
  vertical tilts; keep dragging up and the part tilts all the way over so you can
  inspect its **underside** (no clamp), yet the horizon stays level so the control
  feels predictable. With **Show stock** on, the solid should occlude itself
  correctly (near faces hide far ones) while the toolpath lines stay on top.
  **Right-drag** pans; the **wheel** zooms.
- The **orientation cube** (top-right) rotates together with the part as you
  orbit — the word on the face toward you (TOP/BOTTOM/FRONT/BACK/LEFT/RIGHT)
  names the current side; the words are rendered from a real TrueType font
  (Quicksand) so they should be crisp, and the cube edges softened (chamfered).
- **Clicking a cube face** should snap the view square onto that side, the right
  way up (Top/Bottom/Front/Back/Left/Right all upright). **Reset view** returns
  to the top view; **Cube: off** hides the cube (and clicks no longer pick),
  **Cube: on** restores it.
- Dragging a pane's title bar re-docks it; dragging a border resizes.

## Notes

- The `gui` feature is **not** enabled by default so that `cargo test
  --workspace` and the headless controller stay GPU-free and fully testable.
- `cam-app`'s logic lives in the headless `AppController` (unit-tested); the iced
  layer is a thin view over it.
