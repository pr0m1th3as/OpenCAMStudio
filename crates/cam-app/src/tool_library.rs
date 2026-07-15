//! The cross-project **tool library**: a persistent list of tool definitions the
//! user picks from during operation setup. It lives in the platform config
//! directory (not in any one project); a project embeds copies of the tools it
//! actually uses, so `.ocam` files stay self-contained.
//!
//! GUI-only: the library is session/app state, loaded at startup by the shell.

use std::path::PathBuf;

use cam_model::{Tool, ToolKind};

/// A reusable set of tool definitions, persisted to the config directory.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ToolLibrary {
    pub tools: Vec<Tool>,
}

impl ToolLibrary {
    /// The starter library seeded on first run: a few common end mills.
    pub fn defaults() -> Self {
        let em = |number, diameter| Tool {
            number,
            diameter,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
        };
        Self {
            tools: vec![em(1, 3.0), em(2, 6.0), em(3, 10.0)],
        }
    }

    /// Load the library from disk, falling back to (and persisting) the defaults if
    /// the file is missing or unreadable.
    pub fn load() -> Self {
        if let Some(path) = library_path() {
            if let Ok(text) = std::fs::read_to_string(&path) {
                if let Ok(lib) = serde_json::from_str::<ToolLibrary>(&text) {
                    return lib;
                }
            }
        }
        let lib = Self::defaults();
        lib.save();
        lib
    }

    /// Persist the library to the config directory (best-effort; errors are ignored
    /// — a read-only config dir simply means changes don't persist across runs).
    pub fn save(&self) {
        let Some(path) = library_path() else {
            return;
        };
        if let Some(parent) = path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        if let Ok(text) = serde_json::to_string_pretty(self) {
            let _ = std::fs::write(&path, text);
        }
    }

    /// Append a fresh default tool (numbered one past the highest) and return its
    /// index. The caller typically selects it and edits its fields.
    pub fn add_default(&mut self) -> usize {
        let number = self
            .tools
            .iter()
            .map(|t| t.number)
            .max()
            .map_or(1, |m| m + 1);
        self.tools.push(Tool {
            number,
            diameter: 6.0,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
        });
        self.tools.len() - 1
    }
}

/// `<config-dir>/OpenCAMStudio/tools.json`, or `None` if no config dir is known.
/// A small no-dependency resolver following each platform's convention.
fn library_path() -> Option<PathBuf> {
    config_dir().map(|d| d.join("OpenCAMStudio").join("tools.json"))
}

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

    #[test]
    fn defaults_are_nonempty_and_uniquely_numbered() {
        let lib = ToolLibrary::defaults();
        assert!(!lib.tools.is_empty());
        let mut numbers: Vec<u32> = lib.tools.iter().map(|t| t.number).collect();
        numbers.sort_unstable();
        numbers.dedup();
        assert_eq!(
            numbers.len(),
            lib.tools.len(),
            "tool numbers must be unique"
        );
    }

    #[test]
    fn json_round_trips() {
        let lib = ToolLibrary::defaults();
        let json = serde_json::to_string(&lib).unwrap();
        let back: ToolLibrary = serde_json::from_str(&json).unwrap();
        assert_eq!(lib, back);
    }

    #[test]
    fn add_default_appends_with_next_number() {
        let mut lib = ToolLibrary::defaults();
        let top = lib.tools.iter().map(|t| t.number).max().unwrap();
        let i = lib.add_default();
        assert_eq!(i, lib.tools.len() - 1);
        assert_eq!(lib.tools[i].number, top + 1);
    }
}
