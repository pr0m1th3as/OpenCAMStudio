//! Console entry point for MCP clients, including Windows GUI release builds.
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
        println!(
            "{}\n\nThis executable defaults to --mcp.",
            cli::USAGE.replace("opencamstudio ", "opencamstudio-mcp ")
        );
        return std::process::ExitCode::SUCCESS;
    }
    let result = run(args);
    match result {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("OpenCAMStudio MCP: {error}");
            std::process::ExitCode::FAILURE
        }
    }
}

fn run(args: cli::Args) -> Result<(), Box<dyn std::error::Error>> {
    if let Some(cli::ConnectDesktop { port }) = args.mcp_connect {
        return cam_app::mcp::connect_desktop(port);
    }
    if args.mcp_gui {
        #[cfg(feature = "gui")]
        return cam_app::gui::run_with_mcp(args.setup.as_deref());
        #[cfg(not(feature = "gui"))]
        return Err("--mcp-gui requires a build with --features gui".into());
    }
    let mut session = cam_app::mcp::Session::from_user_settings();
    if let Some(path) = args.setup {
        session.controller.open_project(path)?;
    }
    cam_app::mcp::serve(
        std::io::stdin().lock(),
        std::io::stdout().lock(),
        cam_app::mcp::tools(),
        |name, arguments| session.call(name, arguments),
    )?;
    Ok(())
}
