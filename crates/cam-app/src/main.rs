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

mod cli;

fn main() -> std::process::ExitCode {
    let args = match cli::parse(std::env::args_os().skip(1)) {
        Ok(args) => args,
        Err(error) => {
            eprintln!("{error}\n\n{}", cli::USAGE);
            return std::process::ExitCode::from(2);
        }
    };
    if args.help {
        println!("{}", cli::USAGE);
        return std::process::ExitCode::SUCCESS;
    }
    let result = if let Some(cli::ConnectDesktop { port }) = args.mcp_connect {
        cam_app::mcp::connect_desktop(port)
    } else if args.mcp {
        serve_mcp(args.setup)
    } else if args.mcp_gui {
        launch_mcp_gui(args.setup)
    } else {
        launch(args.setup)
    };
    match result {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("Open CAM Studio: {error}");
            std::process::ExitCode::FAILURE
        }
    }
}

fn serve_mcp(setup: Option<std::path::PathBuf>) -> Result<(), Box<dyn std::error::Error>> {
    let mut session = cam_app::mcp::Session::from_user_settings();
    if let Some(path) = setup {
        session.controller.open_project(path)?;
    }
    cam_app::mcp::serve(
        std::io::stdin().lock(),
        std::io::stdout().lock(),
        cam_app::mcp::tools(),
        |name, args| session.call(name, args),
    )?;
    Ok(())
}

#[cfg(feature = "gui")]
fn launch_mcp_gui(setup: Option<std::path::PathBuf>) -> Result<(), Box<dyn std::error::Error>> {
    cam_app::gui::run_with_mcp(setup.as_deref())
}

#[cfg(not(feature = "gui"))]
fn launch_mcp_gui(_setup: Option<std::path::PathBuf>) -> Result<(), Box<dyn std::error::Error>> {
    Err("--mcp-gui requires a build with --features gui; use --mcp for headless operation".into())
}

#[cfg(feature = "gui")]
fn launch(setup: Option<std::path::PathBuf>) -> Result<(), Box<dyn std::error::Error>> {
    match setup {
        Some(path) => cam_app::gui::run_with_setup(&path),
        None => cam_app::gui::run().map_err(Into::into),
    }
}

#[cfg(not(feature = "gui"))]
fn launch(setup: Option<std::path::PathBuf>) -> Result<(), Box<dyn std::error::Error>> {
    if setup.is_some() {
        return Err("--setup requires the desktop app; build with --features gui".into());
    }
    println!(
        "Open CAM Studio {} — headless build. Launch the desktop app with:\n    \
         cargo run -p cam-app --features gui",
        cam_app::version_string()
    );
    Ok(())
}
