//! OpenCAMStudio application entry point.
//!
//! The default build is headless (the tested [`cam_app::AppController`]); the
//! interactive desktop app is behind the `gui` feature:
//!
//! ```text
//! cargo run -p cam-app --features gui
//! ```

// Without this, Windows opens a console window behind the application and leaves it
// there for the whole session, because the default subsystem for a Rust binary is
// `console`. Three conditions, each load-bearing:
//
// - `windows`     — the attribute is meaningless elsewhere.
// - `feature = "gui"` — the headless build's whole output is a `println!`, so
//   detaching it from the console would send that into nothing. It must keep one.
// - `not(debug_assertions)` — a debug run keeps its console, so `cargo run` still
//   shows panics and logging on Windows. Only the shipped build is detached.
#![cfg_attr(
    all(windows, not(debug_assertions), feature = "gui"),
    windows_subsystem = "windows"
)]

#[cfg(feature = "gui")]
fn main() -> iced::Result {
    cam_app::gui::run()
}

#[cfg(not(feature = "gui"))]
fn main() {
    println!(
        "OpenCAMStudio {} — headless build. Launch the desktop app with:\n    \
         cargo run -p cam-app --features gui",
        env!("CARGO_PKG_VERSION")
    );
}
