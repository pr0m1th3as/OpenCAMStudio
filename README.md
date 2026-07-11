# OpenCAMStudio

A CAM application for CNC toolpath generation, built in Rust.

> **Status: early development (P0 skeleton).** Not yet usable. The first milestone
> is a 2.5-D milling slice — geometry → toolpath → simulation → G-code for
> grbl/FluidNC and Fanuc/Haas controllers.

## Vision

Industrial-grade CAM (an EdgeCAM-class north star), reached incrementally on a
small, stable core. Cutting **strategies** and **post-processors** are the primary
extension points; machines and tools are data. See
[ARCHITECTURE.md](ARCHITECTURE.md) for the design and [ROADMAP.md](ROADMAP.md) for
the phased plan.

## Build

```bash
cargo build --workspace
cargo run --bin opencamstudio
```

A stable Rust toolchain is used (pinned in `rust-toolchain.toml`).

## License

[GPL-3.0-only](LICENSE). All dependencies must be GPLv3-compatible (enforced in CI
via `cargo-deny`).
