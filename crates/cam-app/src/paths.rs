//! Where the application keeps per-user files.
//!
//! One resolver, shared. This lived privately in [`tool_library`](crate::tool_library)
//! until the settings file needed the same answer; two copies of a platform
//! convention is exactly the kind of duplicate that drifts silently, because
//! nothing ever compares them.

use std::path::PathBuf;

/// The application's own directory inside the platform config dir.
///
/// **NOT "Open CAM Studio".** This is a filesystem path, not a display string:
/// renaming it would orphan every existing user's library and settings, silently,
/// and the app would seed fresh defaults as though they had never had any. The
/// display name is spaced; identifiers and paths stay concatenated.
const APP_DIR: &str = "OpenCAMStudio";

/// `<config-dir>/OpenCAMStudio/<name>`, or `None` if no config dir is known.
///
/// `None` is not an error — a machine with neither `XDG_CONFIG_HOME` nor `HOME`
/// simply cannot persist, and every caller treats that as "changes do not survive
/// this run" rather than as a failure to report.
pub(crate) fn config_file(name: &str) -> Option<PathBuf> {
    config_dir().map(|d| d.join(APP_DIR).join(name))
}

/// The platform's per-user configuration directory, by each platform's convention.
/// A small no-dependency resolver — this is the whole reason the crate needs no
/// `dirs`-style dependency.
#[cfg(target_os = "windows")]
fn config_dir() -> Option<PathBuf> {
    std::env::var_os("APPDATA").map(PathBuf::from)
}

#[cfg(target_os = "macos")]
fn config_dir() -> Option<PathBuf> {
    std::env::var_os("HOME").map(|h| PathBuf::from(h).join("Library/Application Support"))
}

#[cfg(not(any(target_os = "windows", target_os = "macos")))]
fn config_dir() -> Option<PathBuf> {
    std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Both per-user files must land in the *same* directory. They were resolved by
    /// separate copies of this logic before, which is precisely how they could have
    /// come to disagree.
    #[test]
    fn every_config_file_shares_one_directory() {
        let (Some(tools), Some(settings)) = (config_file("tools.json"), config_file("settings.json"))
        else {
            return; // no config dir on this machine; nothing to compare
        };
        assert_eq!(tools.parent(), settings.parent());
        assert!(tools.parent().unwrap().ends_with(APP_DIR));
        assert_eq!(tools.file_name().unwrap(), "tools.json");
    }

    /// The directory name is load-bearing: changing it orphans existing users.
    #[test]
    fn the_app_directory_is_the_concatenated_name() {
        assert_eq!(APP_DIR, "OpenCAMStudio");
    }
}
