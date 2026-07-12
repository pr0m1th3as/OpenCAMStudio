//! OpenCAMStudio application entry point.
//!
//! The default build is headless (the tested [`cam_app::AppController`]); the
//! interactive desktop app is behind the `gui` feature:
//!
//! ```text
//! cargo run -p cam-app --features gui
//! ```

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
