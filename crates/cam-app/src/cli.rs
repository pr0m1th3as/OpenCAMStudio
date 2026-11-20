//! Command-line arguments for the desktop executable.

use std::{ffi::OsString, path::PathBuf};

pub const USAGE: &str = "Usage: opencamstudio [--setup <path.ocam>] [--mcp | --mcp-gui] [--help]
       opencamstudio --mcp-connect [port]

  --setup <path.ocam>  Open a saved setup/project on startup without a file dialog
  --mcp                Serve application commands over MCP stdio without a window
  --mcp-gui            Serve MCP stdio controlling the desktop window (requires gui)
  --mcp-connect [port] Bridge MCP stdio to a running desktop. An explicit port uses OPENCAMSTUDIO_MCP_TOKEN.
  -h, --help           Show this help

Relative paths are resolved from the current working directory.";

#[derive(Debug, Default, PartialEq, Eq)]
pub struct Args {
    pub setup: Option<PathBuf>,
    pub help: bool,
    pub mcp: bool,
    pub mcp_gui: bool,
    pub mcp_connect: Option<ConnectDesktop>,
}

#[derive(Debug, PartialEq, Eq)]
pub struct ConnectDesktop {
    pub port: Option<u16>,
}

pub fn parse(args: impl IntoIterator<Item = OsString>) -> Result<Args, String> {
    let mut parsed = Args::default();
    let mut args = args.into_iter().peekable();
    while let Some(arg) = args.next() {
        if arg == "--setup" {
            if parsed.setup.is_some() {
                return Err("--setup may only be specified once".into());
            }
            let path = args
                .next()
                .ok_or("--setup requires a path to a .ocam project")?;
            if path.is_empty() || path.as_encoded_bytes().starts_with(b"-") {
                return Err("--setup requires a path to a .ocam project".into());
            }
            parsed.setup = Some(path.into());
        } else if arg == "--mcp" || arg == "--mcp-gui" {
            if parsed.mcp || parsed.mcp_gui {
                return Err("specify only one of --mcp or --mcp-gui".into());
            }
            parsed.mcp = arg == "--mcp";
            parsed.mcp_gui = arg == "--mcp-gui";
        } else if arg == "--mcp-connect" {
            if parsed.mcp_connect.is_some() {
                return Err("--mcp-connect may only be specified once".into());
            }
            let port = args
                .next_if(|arg| !arg.as_encoded_bytes().starts_with(b"-"))
                .map(|arg| {
                    arg.to_str()
                        .and_then(|port| port.parse::<u16>().ok())
                        .filter(|&port| port != 0)
                        .ok_or("--mcp-connect accepts a TCP port from 1 to 65535")
                })
                .transpose()?;
            parsed.mcp_connect = Some(ConnectDesktop { port });
        } else if arg == "--help" || arg == "-h" {
            parsed.help = true;
        } else {
            return Err(format!("Unknown argument: {}", arg.to_string_lossy()));
        }
    }
    if parsed.mcp_connect.is_some() && (parsed.mcp || parsed.mcp_gui || parsed.setup.is_some()) {
        return Err("--mcp-connect cannot be combined with --mcp, --mcp-gui or --setup".into());
    }
    Ok(parsed)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse_strs(args: &[&str]) -> Result<Args, String> {
        parse(args.iter().map(OsString::from))
    }

    #[test]
    fn default_startup_and_help() {
        assert_eq!(parse_strs(&[]).unwrap(), Args::default());
        for flag in ["--help", "-h"] {
            assert!(parse_strs(&[flag]).unwrap().help);
        }
    }

    #[test]
    fn setup_paths_preserve_spaces_and_relative_components() {
        for path in ["testfiles/test_project.ocam", "../My setups/test part.ocam"] {
            assert_eq!(
                parse_strs(&["--setup", path]).unwrap().setup,
                Some(path.into())
            );
        }
    }

    #[test]
    fn mcp_modes_accept_startup_projects_and_are_mutually_exclusive() {
        let headless = parse_strs(&["--mcp", "--setup", "part.ocam"]).unwrap();
        assert!(headless.mcp && !headless.mcp_gui);
        let desktop = parse_strs(&["--setup", "part.ocam", "--mcp-gui"]).unwrap();
        assert!(desktop.mcp_gui && !desktop.mcp);
        for args in [
            ["--mcp", "--mcp-gui"],
            ["--mcp", "--mcp"],
            ["--mcp-gui", "--mcp-gui"],
        ] {
            assert!(parse_strs(&args).is_err());
        }
    }

    #[test]
    fn rejects_missing_paths_duplicates_and_unknown_arguments() {
        for args in [
            vec!["--setup"],
            vec!["--setup", ""],
            vec!["--setup", "--help"],
            vec!["--setup", "a.ocam", "--setup", "b.ocam"],
            vec!["--unknown"],
            vec!["unexpected.ocam"],
        ] {
            assert!(parse_strs(&args).is_err(), "accepted {args:?}");
        }
    }

    #[cfg(unix)]
    #[test]
    fn setup_path_need_not_be_utf8() {
        use std::os::unix::ffi::OsStringExt;
        let path = OsString::from_vec(b"setup-\xff.ocam".to_vec());
        assert_eq!(
            parse([OsString::from("--setup"), path.clone()])
                .unwrap()
                .setup,
            Some(path.into())
        );
    }
}
