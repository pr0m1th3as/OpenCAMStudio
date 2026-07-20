# Open CAM Studio

A CAM application for CNC toolpath generation, built in Rust.

> **Status: the 2.5-D milling slice works end-to-end** — DXF/DWG in, toolpaths
> and material simulation in a live viewport, real G-code out. Everything beyond
> 2.5-D (3-axis surfacing, turning, plugins) is roadmap.

![The OpenCAMStudio desktop application: the built-in sample part with two
operations — an outside profile and a pocket — showing the project tree on the
left, the 3-D toolpath backplot with orientation cube in the centre, the operation
inspector on the right, and run diagnostics along the bottom.](docs/screenshot.png)

*The built-in sample (**Home → Sample**) with a profile and a pocket, run and ready
to export.*

## What works

- **Operations** — profile, pocket, drill, face, chamfer, **thread milling**,
  **V-carve engraving** and **V-carving**; leads and ramp/helix plunges;
  engagement-capped area clearing (adaptive front-advance where it certifies,
  concentric otherwise).
- **V-carving** — the boundary outlines an *area*, not a path: the tool never
  touches it, its flanks land on it, and the depth follows from the shape's own
  width. Built on **inward offset rings, not a medial axis** — the ring at inward
  distance `w` *is* the locus of points at distance `w`, so the medial axis is
  simply where the offsets vanish. Optionally **two tools in one operation**: an
  end mill clears the flat land the depth cap leaves — each level to its own
  depth's width, so it roughs the taper as a staircase — and then hands over to
  the V-bit, which finishes the wall and cleans the corners a round cutter
  cannot reach.
- **Tool suitability guards** — every operation checks that the surface doing the
  cutting *is* a cutting surface, reading each tool's own profile rather than
  matching on its type. Using a non-cutting surface (engraving with a chamfer
  mill, whose tip is a flat that does not cut) or cutting past the flute length
  onto the shank is refused; merely *poor* choices warn and proceed.
- **Geometry in** — DXF/DWG import with contour chaining and hole nesting,
  including **open paths** (lettering and decorative strokes, for engraving);
  a carved region's islands (the counters of letters) come from the drawing's own
  nesting rather than being clicked one by one;
  AutoCAD-style object snaps (end / mid / quadrant / nearest) when picking.
- **Tooling** — a cross-project tool library with every cutter kind fully
  characterised: square / ball-nose / rounded-edge end mills, drill, V-bit,
  chamfer mill, face mill (shell mill), and single-profile / full-form thread
  mills. Each carries real flute/shank/neck geometry and a per-kind revolve
  profile shown in a **live 2D cross-section preview** as you edit; import/export
  `.ocam` libraries; gap-filling / swap / bulk tool numbering. Operations pick a
  tool by **family, then tool**, with the families bounded by what the operation
  can actually cut.
- **Setup** — workpiece origin (datum) + program start point; part-relative
  stock; first-class clearance/retract heights.
- **Verify** — heightfield material-removal simulation with gouge and
  rapid-through-stock detection, both **profile-aware**: a pointed tool is modelled
  as wide as its cone has actually opened at that depth, and the grid refines to
  resolve the narrowest cut (a sub-millimetre engraved groove is visible).
- **G-code out** — six posts across three families (grbl / FluidNC / grblHAL,
  LinuxCNC, Fanuc / Haas) from one controller-neutral IR, with canned cycles where
  supported and expanded where not.
- **Shell** — a `wgpu` viewport (backplot, solid stock, orientation cube) in an
  `iced` desktop GUI.

See [ARCHITECTURE.md](ARCHITECTURE.md) for the design and [ROADMAP.md](ROADMAP.md)
for the phased plan toward an EdgeCAM-class north star.

## Install

Downloads for each release are on the
[Releases page](https://github.com/pr0m1th3as/OpenCAMStudio/releases).

| Platform | Download | Notes |
|---|---|---|
| **Linux** x86‑64 | `OpenCAMStudio-vX.Y.Z-linux-x86_64.AppImage` | `chmod +x` it and run. Needs glibc 2.35 or newer — Ubuntu 22.04, Debian 12 and anything later. |
| **Windows** x86‑64 | `…-windows-x86_64-installer.msi` | Installs to Program Files with a Start Menu entry. |
| | `…-windows-x86_64-portable.exe` | A single self-contained executable — no install, run it from anywhere. |
| **macOS** Apple silicon | `…-macos-arm64.dmg` | Drag to Applications. Apple silicon only; there is no Intel build. |

Every download has a `.sha256` beside it:

```bash
sha256sum -c OpenCAMStudio-vX.Y.Z-linux-x86_64.AppImage.sha256
```

### The binaries are not code-signed

They are built in the open by
[GitHub Actions](https://github.com/pr0m1th3as/OpenCAMStudio/actions) from the
tagged commit, but they carry no Authenticode or Apple Developer signature —
those require paid certificates. **Both operating systems will warn you, and the
warning is expected rather than a sign that something is wrong:**

- **macOS** refuses to open the app on first launch. Right-click it in Finder and
  choose **Open**, then confirm — after that it launches normally. (Equivalently:
  `xattr -d com.apple.quarantine "/Applications/Open CAM Studio.app"`.)
- **Windows** SmartScreen shows *"Windows protected your PC"*. Choose **More
  info → Run anyway**.

If you would rather not take our word for any of it, the checksums above let you
confirm you received exactly what CI produced, and the whole thing builds from
source in one command — see below.

## Build & run

```bash
cargo build --workspace                # headless: geometry, toolpaths, posts, sim
cargo test  --workspace                # the full test suite
cargo run -p cam-app --features gui    # the desktop GUI
```

Stable Rust, pinned in `rust-toolchain.toml`. GUI prerequisites and platform
notes: [RUNNING.md](RUNNING.md).

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
