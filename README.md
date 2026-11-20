# Open CAM Studio

A CAM application for CNC toolpath generation, built in Rust.

> **Status: 2.5-D milling and initial 3D machining work end-to-end** — geometry
> import, toolpath generation, material-removal simulation and G-code export.
> 3D Adaptive Clearing provides CAD-derived roughing levels; 3D Parallel finishing
> follows freeform surfaces with a ball nose cutter on pre-roughed stock.
> Sampled 3D Scallop and Spiral finishing are also available; additional 3D
> finishing strategies, turning and plugins remain roadmap work.

![3D Parallel finishing toolpath generated for the tray project](docs/tray-parallel-toolpath.png)

*3D Parallel finishing on `tray.ocam`: surface-following passes with a 6 mm ball nose cutter and 1 mm stepover.*

![Simulation of the tray Parallel finishing operation](docs/tray-parallel-simulation.png)

*Completed material-removal simulation of the tray’s Parallel finishing operation.*

## What works

- **Operations** — profile, pocket, drill, bore, face, chamfer, **thread milling**,
  **V-carve engraving** and **V-carving**; leads and ramp/helix plunges;
  engagement-capped area clearing (adaptive front-advance where it certifies,
  concentric otherwise). Pocket operations offer optional finishing passes with
  separate radial stepover and feed, leads, a repeat pass and closure overlap;
  cutter compensation is computed in the toolpath. Profile and chamfer operations
  also support contour roughing passes; milling operations offer multiple-depth
  controls for staged cutting. See [Profile milling](docs/profile-operation.md)
  and [operation compatibility](docs/operation-compatibility.md).
- **Pocket stock contours** — setup-stock or selected closed footprints for
  open-pocket and 2D adaptive clearing, with cutter runout and protected islands.
  See [selection and behavior](docs/pocket-stock-contours.md).
- **3D Adaptive Clearing** — roughing levels with upward fine-step refinement,
  optimal load and tolerance controls, using the independent CAD machining
  tessellation. Select the bodies or faces to machine while retaining other setup
  bodies as protected obstacles. See the [workflow and geometry
  limits](docs/3d-adaptive-clearing.md).
- **3D Parallel finishing** — ball nose surface passes with scan angle, horizontal
  stepover, one-way or zigzag cutting, CAD body/face selection, bounding-box
  containment, automatic cutting turns and a retraction policy between cutting chains. See [controls and limits](docs/parallel.md).
- **3D Spiral finishing** — projected spiral or concentric-circle surface passes
  with selectable center, machining boundary, stepover, direction and depth controls.
- **Stock to Leave** — shared radial and axial finishing allowances for Profile,
  Pocket, 3D Adaptive Clearing and 3D Parallel; axial allowance for Face. Disabled settings
  preserve existing toolpaths.
- **Fourth-axis machining** — wrap Profile, Pocket and 2D adaptive toolpaths
  around a cylinder, or use indexed 3+1 machining with ordinary operations.
  Wrap supports CAD cylinder/boundary selection, continuous A/B motion and
  inverse-time LinuxCNC/Fanuc/Haas output. Rotary and mixed jobs support stock
  removal, collision checks and playback; see the [workflow and limits](docs/rotary-wrap.md).
- **V-carving** — the boundary outlines an *area*, not a path: the tool never
  touches it, its flanks land on it, and the depth follows from the shape's own
  width. Built on **inward offset rings, not a medial axis** — the ring at inward
  distance `w` *is* the locus of points at distance `w`, so the medial axis is
  simply where the offsets vanish. Optionally **two tools in one operation**: an
  end mill clears the flat land the depth cap leaves — each level to its own
  depth's width, so it roughs the taper as a staircase — and then hands over to
  the V-bit, which finishes the wall and cleans the corners a round cutter
  cannot reach.
- **Tool suitability warnings** — cutter shape, cutting surfaces and reach are
  advisory. Deliberately different cutters can generate and simulate; stock
  removal follows the selected cutter's actual profile. Numeric validity,
  computable paths and simulation collision checks remain enforced.
- **Geometry in** — DXF/DWG import with contour chaining and hole nesting,
  including **open paths** (lettering and decorative strokes, for engraving);
  a carved region's islands (the counters of letters) come from the drawing's own
  nesting rather than being clicked one by one;
  AutoCAD-style object snaps (end / mid / quadrant / nearest) when picking.
- **STEP/IGES feature selection (optional CAD build)** — background import and unit/
  dimension preview, shaded bodies with face/edge/loop picking, and linked Pocket,
  Drill, Profile and Face operations in the qualified fixed-axis subset. Geometry
  links, height overrides and model replacement/reselection survive `.ocam` saves
  and undo. Included in release packages. Source builds require OCCT 7.9.3 and
  `--features gui,occt`; see the
  [CAD import guide](docs/cad-import.md).
- **Tooling** — named cross-project tool libraries with every cutter kind fully
  characterised: square / ball-nose / rounded-edge end mills, drill, V-bit,
  chamfer mill, face mill (shell mill), and single-profile / full-form thread
  mills. Each carries real flute/shank/neck geometry and a per-kind revolve
  profile shown in a **live 2D cross-section preview** as you edit; import/export
  `.ocam` libraries; gap-filling / swap / bulk tool numbering. Operations pick a
  tool by **family, then tool**, with deliberate cutter substitutions available
  for general path operations. [Material-linked cutting presets](docs/tool-presets.md)
  automatically seed operations from a shared stock-material list; each operation
  can select another preset or override its cutting values.
- **Setup and patterns** — multiple setups with independent work coordinates,
  stock, fixtures and NC programs; inherit remaining stock between machining
  sides. Repeat compatible operations in linear or circular patterns. See
  [setups](docs/multi-setup.md) and [operation patterns](docs/operation-patterns.md).
- **Parameter formulas** — enter expressions using tool dimensions and operation
  parameters in numeric machining fields, with autocomplete and retained bindings.
  Display precision does not change calculated values. See [numeric parameter
  formulas](docs/parameter-expressions.md).
- **Verify** — heightfield material-removal simulation with gouge,
  rapid-through-stock and non-cutting feed/tip collision detection. Playback pauses
  at stock contact with the cutter highlighted red. Checks are **profile-aware**:
  a pointed tool is modelled
  as wide as its cone has actually opened at that depth, and the grid refines to
  resolve the narrowest cut (a sub-millimetre engraved groove is visible).
  The [simulation menu](docs/simulation.md) configures tool and path visibility,
  stock coloring, accuracy, transparency, rewind behavior and collision stopping.
- **G-code out** — eight posts (grbl / FluidNC / grblHAL, LinuxCNC, Fanuc / Haas,
  Okuma OSP, Estlcam) from one controller-neutral IR, with canned cycles where
  supported and expanded where not. Estlcam includes configurable coolant/MMS
  outputs; see [Estlcam setup](docs/estlcam.md).
- **Shell** — a `wgpu` viewport (backplot, solid stock, orientation cube) in an
  `iced` desktop GUI, with measurements shown in the 3D view. Hover over a setup or operation status indicator for its
  current state; click an indicator with warnings or errors to read the details.
  Toolpath diagnostics do not open dialogs automatically.
  Move imported models or individual bodies with viewport controls or exact values;
  a tabbed Preferences window includes the local desktop MCP settings.
- **CPU acceleration** — multicore Parallel/Scallop surface calculation, stock
  mesh tiles and stock-to-CAD comparison; portable double-precision SIMD uses
  Apple Silicon NEON or x86-64 SSE2. See [performance controls and benchmarks](docs/performance.md).
- **Planning and automation** — estimate [processing time](docs/processing-time.md)
  and [cutting forces](docs/cutting-forces.md), or control the application through
  [MCP commands](docs/mcp.md) in a headless or visible session.

See the [user guides](docs/README.md) for workflows and limits,
[ARCHITECTURE.md](ARCHITECTURE.md) for a brief overview and [ROADMAP.md](ROADMAP.md)
for future direction.

## Install

Downloads for each release are on the
[Releases page](https://github.com/mkomuc/OpenCAMStudio/releases).

Native packages are built for each release using GitHub Actions. See
[RELEASING.md](RELEASING.md) for publishing and manual build instructions.

| Platform | Download | Notes |
|---|---|---|
| **Linux** x86‑64 | `OpenCAMStudio-vX.Y.Z-linux-x86_64.AppImage` | `chmod +x` it and run. Needs glibc 2.35 or newer — Ubuntu 22.04, Debian 12 and anything later. |
| **Windows** x86‑64 | `…-windows-x86_64-installer.msi` | Installs to Program Files with a Start Menu entry. |
| | `…-windows-x86_64-portable.zip` | Extract the whole ZIP, then run `opencamstudio.exe`; keep the bundled DLLs beside it. |
| **macOS** Apple silicon | `…-macos-arm64.dmg` | Drag to Applications. Apple silicon only; there is no Intel build. |

Every download has a `.sha256` beside it:

```bash
sha256sum -c OpenCAMStudio-vX.Y.Z-linux-x86_64.AppImage.sha256
```

All packages include OCCT 7.9.3 for STEP/IGES import, its shared libraries and
license notices. No separate OCCT installation is needed. The corresponding OCCT
source is attached to each release as `OpenCAMStudio-OCCT-7.9.3-source.tar.gz`.

### The binaries are not code-signed

They are built in the open by
[GitHub Actions](https://github.com/mkomuc/OpenCAMStudio/actions) from the
tagged commit, but they carry no Authenticode or Apple Developer signature —
those require paid certificates. **Both operating systems will warn you, and the
warning is expected rather than a sign that something is wrong:**

- **macOS** reports:

  > **"Open CAM Studio.app" is damaged and can't be opened. You should move it to
  > the Trash.**

  **It is not damaged, and you should not move it to the Trash.** That is the
  message macOS gives an unsigned application, and it is indistinguishable from a
  genuinely corrupt download — if you want to be sure, the `.sha256` above proves
  the file is byte-for-byte what CI built.

  The usual right-click → **Open** → confirm route **does not clear it** for an
  unsigned bundle. Open the `.dmg`, **drag the app into Applications first**, then
  run this once:

  ```bash
  xattr -rd com.apple.quarantine "/Applications/Open CAM Studio.app"
  ```

  It will not launch until that command has been run; afterwards it opens normally
  from the Dock or Launchpad, and the step never needs repeating. The `-r` matters:
  the quarantine flag is set on files *inside* the bundle, not just on the bundle
  itself.
- **Windows** SmartScreen shows *"Windows protected your PC"*. Choose **More
  info → Run anyway**.

If you would rather not take our word for any of it, the checksums above let you
confirm you received exactly what CI produced, and the whole thing builds from
source in one command — see below.

## Build & run

```bash
cargo build --workspace                # headless: geometry, toolpaths, posts, sim
cargo test  --workspace                # the full test suite
cargo run -p cam-app --features gui,occt    # the desktop GUI - occt for 3d STEP/IGES support
cargo run -p cam-app --features gui -- --setup "testfiles/test_project.ocam" # open setup directly
OCCT_ROOT="$PWD/.local/occt" cargo build --release -p cam-app --features gui,occt
```

Stable Rust, pinned in `rust-toolchain.toml`. GUI prerequisites and platform
notes: [RUNNING.md](RUNNING.md).

CI checks crate dependency boundaries with `python3 scripts/check_architecture.py`,
alongside the Rust tests and Clippy.

Load a saved setup/project at startup (useful for automated GUI tests):

```bash
cargo run -p cam-app --features gui -- --setup "testfiles/test_project.ocam"
```

Packaged builds accept the same `--setup <path.ocam>` option.

Project saves use compressed `.ocam` archives with editable JSON and lossless stock
state; display meshes rebuild when opened. Legacy JSON projects still open and
convert on save. Tool libraries remain JSON. See the [project cache policy](docs/project-cache-policy.md)
for format details and measurements. New project archives require an updated build.

Run the GUI with a local OCCT installation on macOS:
```bash
OCCT_ROOT="$PWD/.local/occt" \
DYLD_LIBRARY_PATH="$PWD/.local/occt/lib" \
  cargo run -p cam-app --features gui,occt
```

## Acknowledgements

OpenCAMStudio owes its design outlook to
**[OpenCADStudio](https://github.com/HakanSeven12/OpenCADStudio)**, created and
developed by **[HakanSeven12](https://github.com/HakanSeven12)** and its
contributors — a Rust CAD application built on `wgpu` and `iced` with a native
plugin ABI and cross-platform CI. That combination is what this project set out to
follow: the same language and graphics stack, the same small-core-plus-plugins
shape, and a deliberately familiar ribbon, so that designing in CAD and then
machining in CAM feels like one workflow rather than two programs.

The two are **independent projects** with no runtime dependency between them. Both
are GPL-3.0, which is also what makes the borrowing more than skin-deep: several of
the ribbon icons here are OpenCADStudio's own, reused unmodified under that licence
and credited individually in
[`crates/cam-app/assets/icons/CREDITS.md`](crates/cam-app/assets/icons/CREDITS.md).

With thanks for the example, and for licensing it so that others can build on it.

## License

[GPL-3.0-only](LICENSE). All dependencies must be GPLv3-compatible (enforced in CI
via `cargo-deny`).
