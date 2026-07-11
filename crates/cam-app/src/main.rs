//! OpenCAMStudio application entry point.
//!
//! P0 skeleton: prints a banner so the workspace has a runnable binary and the
//! CI/release pipelines have something to build. The real `iced` shell arrives
//! at P5 (see `ROADMAP.md`).

fn main() {
    println!(
        "OpenCAMStudio {} — CAM for CNC toolpath generation (P0 skeleton)",
        env!("CARGO_PKG_VERSION")
    );
}
