# OpenCAMStudio

A CAM application for CNC toolpath generation, built in Rust.

> **Status: the 2.5-D milling slice works end-to-end** — DXF/DWG in, toolpaths
> and material simulation in a live viewport, real G-code out. Everything beyond
> 2.5-D (3-axis surfacing, turning, plugins) is roadmap.

## What works

- **Operations** — profile, pocket, drill, face, chamfer, **thread milling** and
  **V-carve engraving**; leads and ramp/helix plunges; engagement-capped area
  clearing (adaptive front-advance where it certifies, concentric otherwise).
- **Tool suitability guards** — every operation checks that the surface doing the
  cutting *is* a cutting surface, reading each tool's own profile rather than
  matching on its type. Using a non-cutting surface (engraving with a chamfer
  mill, whose tip is a flat that does not cut) or cutting past the flute length
  onto the shank is refused; merely *poor* choices warn and proceed.
- **Geometry in** — DXF/DWG import with contour chaining and hole nesting,
  including **open paths** (lettering and decorative strokes, for engraving);
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

## Build & run

```bash
cargo build --workspace                # headless: geometry, toolpaths, posts, sim
cargo test  --workspace                # the full test suite
cargo run -p cam-app --features gui    # the desktop GUI
```

Stable Rust, pinned in `rust-toolchain.toml`. GUI prerequisites and platform
notes: [RUNNING.md](RUNNING.md).

## License

[GPL-3.0-only](LICENSE). All dependencies must be GPLv3-compatible (enforced in CI
via `cargo-deny`).
