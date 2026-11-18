//! Connect a stdio MCP client to the opted-in desktop over IPv4 loopback.
use serde::{Deserialize, Serialize};
use std::fs;
use std::io::{self, Read, Write};
use std::net::{Ipv4Addr, Shutdown, TcpStream};
use std::path::Path;
use std::time::Duration;

const TOKEN_ENV: &str = "OPENCAMSTUDIO_MCP_TOKEN";
const TOKEN_LEN: usize = 64;
const AUTH_TIMEOUT: Duration = Duration::from_secs(5);
const ACCEPTED: &[u8; 3] = b"OK\n";
const ENDPOINT_PREFIX: &str = "mcp-endpoint-";
const ENDPOINT_SUFFIX: &str = ".json";

#[derive(Serialize, Deserialize)]
pub(crate) struct Endpoint {
    pub port: u16,
    pub token: String,
}

impl Endpoint {
    // Each listener owns its file. Closing one window cannot erase another's
    // endpoint, and an abrupt exit does not prevent publishing a new one.
    #[cfg(feature = "gui")]
    pub(crate) fn publish(&self, directory: &Path) -> io::Result<tempfile::NamedTempFile> {
        fs::create_dir_all(directory)?;
        let mut file = tempfile::Builder::new()
            .prefix(ENDPOINT_PREFIX)
            .suffix(ENDPOINT_SUFFIX)
            .tempfile_in(directory)?;
        file.write_all(&serde_json::to_vec(self)?)?;
        Ok(file)
    }

    pub(crate) fn discover(directory: &Path) -> io::Result<Self> {
        let path = fs::read_dir(directory)?
            .filter_map(Result::ok)
            .filter_map(|entry| {
                let name = entry.file_name();
                let name = name.to_str()?;
                if !name.starts_with(ENDPOINT_PREFIX)
                    || !name.ends_with(ENDPOINT_SUFFIX)
                    || !entry.file_type().ok()?.is_file()
                {
                    return None;
                }
                Some((entry.metadata().ok()?.modified().ok()?, entry.path()))
            })
            .max()
            .map(|(_, path)| path)
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "no published MCP endpoint"))?;
        Ok(serde_json::from_reader(fs::File::open(path)?.take(1024))?)
    }
}

#[cfg(feature = "gui")]
pub(crate) fn new_token() -> io::Result<String> {
    let mut bytes = [0u8; TOKEN_LEN / 2];
    getrandom::fill(&mut bytes).map_err(io::Error::other)?;
    Ok(bytes.iter().map(|byte| format!("{byte:02x}")).collect())
}

#[cfg(feature = "gui")]
pub(crate) fn authenticate(mut stream: &TcpStream, token: &str) -> io::Result<()> {
    stream.set_read_timeout(Some(AUTH_TIMEOUT))?;
    stream.set_write_timeout(Some(AUTH_TIMEOUT))?;
    let mut received = [0; TOKEN_LEN + 1];
    stream.read_exact(&mut received)?;
    let difference = received[..TOKEN_LEN]
        .iter()
        .zip(token.as_bytes())
        .fold(received[TOKEN_LEN] ^ b'\n', |diff, (a, b)| diff | (a ^ b));
    if difference != 0 {
        return Err(io::Error::new(
            io::ErrorKind::PermissionDenied,
            "invalid desktop token",
        ));
    }
    stream.write_all(ACCEPTED)?;
    stream.set_read_timeout(None)
}

/// Forward the client's stdio to a desktop that enabled MCP in Preferences.
pub fn connect_desktop(port: Option<u16>) -> Result<(), Box<dyn std::error::Error>> {
    let Endpoint { port, token } = match port {
        Some(port) => Endpoint {
            port,
            token: std::env::var(TOKEN_ENV).unwrap_or_default(),
        },
        None => {
            let settings = crate::settings_path().ok_or("Settings directory unavailable.")?;
            let directory = settings.parent().ok_or("Settings directory unavailable.")?;
            Endpoint::discover(directory).map_err(|err| {
                format!("Could not find the desktop MCP connection: {err}. Open Open CAM Studio and enable MCP in Preferences.")
            })?
        }
    };
    if token.len() != TOKEN_LEN || !token.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err(format!(
            "Set {TOKEN_ENV} using the client configuration from Preferences → MCP."
        )
        .into());
    }

    let mut stream = TcpStream::connect((Ipv4Addr::LOCALHOST, port)).map_err(|err| {
        format!("Could not connect to Open CAM Studio on 127.0.0.1:{port}: {err}. Check that the app is open and MCP is enabled.")
    })?;
    stream.set_read_timeout(Some(AUTH_TIMEOUT))?;
    stream.set_write_timeout(Some(AUTH_TIMEOUT))?;
    writeln!(stream, "{token}")?;
    let mut reply = [0; ACCEPTED.len()];
    if stream.read_exact(&mut reply).is_err() || &reply != ACCEPTED {
        return Err("Desktop did not accept the connection. Check the client configuration and close any other MCP client connected to this window.".into());
    }
    stream.set_read_timeout(None)?;
    stream.set_write_timeout(None)?;

    let mut input = stream.try_clone()?;
    // stdin may stay open after the desktop disconnects. Keep stdout on the
    // main thread so the bridge exits as soon as the desktop closes its socket.
    std::thread::Builder::new()
        .name("mcp-stdin".into())
        .spawn(move || {
            if let Err(error) = io::copy(&mut io::stdin().lock(), &mut input) {
                eprintln!("OpenCAMStudio MCP: {error}");
            }
            let _ = input.shutdown(Shutdown::Write);
        })?;
    let mut stdout = io::stdout().lock();
    io::copy(&mut stream, &mut stdout)?;
    stdout.flush()?;
    Ok(())
}
