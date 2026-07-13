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

The window has a control panel on the left and a `wgpu` viewport on the right.

1. **Open sample part** — loads a built-in rectangle-with-a-hole (no file dialog
   yet; file loading is the next increment). The viewport frames the geometry.
2. Adjust **Tool ⌀**, **Depth**, **Stepdown**. Each edit is undoable.
3. **Run** — computes the toolpath. The viewport shows the backplot colored by
   move kind: green = cutting, yellow = rapid/link, red = plunge; the part
   outline is drawn in light grey. Any problems appear under **Diagnostics**
   (e.g. a tool too large to open the hole).
4. **Undo / Redo** — step through parameter changes.
5. **Show stock / Hide stock** — overlays the *simulated* stock surface (the
   material left after the toolpath cuts) under the backplot, shaded so the
   pocket walls and stepdowns read. Available after a **Run**.
6. **Export .nc** — posts the toolpath to grbl G-code (blocked if the run had
   errors). The status line reports the line count.

### What to check when testing the GUI

Because the GUI is the one part that cannot be verified by automated tests, the
things worth eyeballing:

- The window opens and the sample part frames correctly in the viewport.
- After **Run**, the backplot draws — an outer rounded-rectangle tool path, an
  inner loop around the hole, colored as above.
- Setting **Tool ⌀** to `12` and pressing **Run** produces a "tool too large"
  diagnostic and blocks export (the 6 mm-radius tool can't open the 10 mm hole).
- **Undo** restores the previous parameter and the backplot updates on the next
  **Run**.
- **Show stock** overlays a shaded grey surface with the pockets/holes carved
  out; the colored backplot stays visible on top. **Hide stock** removes it.

## Notes

- The `gui` feature is **not** enabled by default so that `cargo test
  --workspace` and the headless controller stay GPU-free and fully testable.
- `cam-app`'s logic lives in the headless `AppController` (unit-tested); the iced
  layer is a thin view over it.
