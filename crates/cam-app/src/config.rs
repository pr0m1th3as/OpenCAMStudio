//! The per-user config-file contract, in one place.
//!
//! Three files now live in the config directory — the tool library, the settings, and
//! the machine library — and each wants the same behaviour. Writing it a third time is
//! how the two shutdown sequences came to disagree (see
//! `.claude/memory/duplicated-sequences-drift.md`): copies written days apart drift, and
//! nothing catches it, because no test ever contains both.
//!
//! # The contract
//!
//! | case | behaviour |
//! |---|---|
//! | missing | defaults; **seeded and written** only if the type says so (a library needs contents to pick from; settings do not) |
//! | unreadable / unparseable | defaults, the file **left exactly as it was**, and a `.bak` copy put beside it |
//! | version newer than this build | the same — refused, not read leniently |
//! | version older | migrated through [`ConfigFile::migrate`], then normalised |
//! | unknown keys | ignored on read, preserved on write (the type's own `#[serde(flatten)]`) |
//!
//! **The load never rewrites a file it could not read.** That is the whole point, and it
//! is the opposite of what `ToolLibrary::load` used to do — fall back to the stock 36
//! tools *and save them*, so a parse failure silently destroyed a hand-built library.

use std::path::{Path, PathBuf};

use serde::{de::DeserializeOwned, Serialize};

/// A file this application keeps for the user in the platform config directory.
pub(crate) trait ConfigFile: Serialize + DeserializeOwned + Default {
    /// File name inside `<config-dir>/OpenCAMStudio/`.
    const FILE_NAME: &'static str;
    /// The format version this build writes.
    const VERSION: u32;
    /// Whether a *missing* file is seeded from [`seed`](ConfigFile::seed) and written.
    ///
    /// True for a library — an empty tool list is useless, so first run gets contents.
    /// False for settings — nothing should appear on disk until the user changes
    /// something.
    const SEED_ON_MISSING: bool;
    /// What this file is, for the message on a rejected load.
    const WHAT: &'static str;

    /// The version the loaded value states.
    fn stated_version(&self) -> u32;
    /// Bring a tree of version `from` up to [`VERSION`](ConfigFile::VERSION), stamping
    /// the new version. Additive changes need no step.
    fn migrate(&mut self, from: u32);
    /// Force values into range after loading. A config file is hand-editable text; a
    /// nonsense number must not be able to produce an unusable app.
    fn normalise(&mut self) {}
    /// **The value used whenever the file provides none** — missing, unreadable, or
    /// written by a newer build.
    ///
    /// Not the same as `Default`, and the difference is load-bearing: an empty
    /// `ToolLibrary` is a *valid* value but a useless app, so a library whose file could
    /// not be read must fall back to its starter set rather than to nothing. Settings
    /// are the other way round — their `Default` is exactly right.
    ///
    /// [`SEED_ON_MISSING`](ConfigFile::SEED_ON_MISSING) decides whether this value is
    /// also *written* on a first run; this decides what it is.
    fn seed() -> Self {
        Self::default()
    }
}

/// What happened when a config file was read.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum ConfigLoad {
    /// No file, and none was written — defaults are in force.
    Fresh,
    /// No file: seeded from the defaults and written.
    Seeded,
    /// Read, migrated if needed, normalised.
    Loaded,
    /// A file exists but could not be used. Defaults are in force, **the file was left
    /// exactly as it was**, and a `.bak` copy sits beside it. The string says why.
    Rejected(String),
}


/// Read `T` from its place in the config directory.
pub(crate) fn load<T: ConfigFile>() -> (T, ConfigLoad) {
    match crate::paths::config_file(T::FILE_NAME) {
        Some(p) => load_from(&p),
        // Nowhere to persist on this machine: defaults, and nothing to write.
        None => (T::seed(), ConfigLoad::Fresh),
    }
}

/// Read `T` from `path`. See the module docs — the failure behaviour is the point.
pub(crate) fn load_from<T: ConfigFile>(path: &Path) -> (T, ConfigLoad) {
    let reject = |why: String| {
        // Preserve what we could not read, *before* anything can overwrite it. A later
        // deliberate save will still write over the original — no load contract can
        // prevent that — which is exactly why the copy has to exist first.
        let _ = std::fs::copy(path, backup_path(path));
        (T::seed(), ConfigLoad::Rejected(why))
    };
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            if T::SEED_ON_MISSING {
                let seeded = T::seed();
                let _ = save_to(&seeded, path);
                return (seeded, ConfigLoad::Seeded);
            }
            return (T::seed(), ConfigLoad::Fresh);
        }
        Err(e) => return reject(format!("could not be read ({e})")),
    };
    let mut value: T = match serde_json::from_str(&text) {
        Ok(v) => v,
        Err(e) => return reject(format!("is not valid {} ({e})", T::WHAT)),
    };
    if value.stated_version() > T::VERSION {
        return reject(format!(
            "was written by a newer version (format {}, this build understands {})",
            value.stated_version(),
            T::VERSION
        ));
    }
    value.migrate(value.stated_version());
    value.normalise();
    (value, ConfigLoad::Loaded)
}

/// `<path>.bak` — the copy left beside a file that could not be read.
fn backup_path(path: &Path) -> PathBuf {
    let mut name = path.file_name().unwrap_or_default().to_os_string();
    name.push(".bak");
    path.with_file_name(name)
}

/// Write `T` to its place in the config directory. Best-effort: a read-only config dir
/// simply means changes do not survive the run, which is not worth interrupting anyone
/// over.
pub(crate) fn save<T: ConfigFile>(value: &T) {
    if let Some(path) = crate::paths::config_file(T::FILE_NAME) {
        let _ = save_to(value, &path);
    }
}

/// Write `T` to `path`, creating the directory if needed.
pub(crate) fn save_to<T: ConfigFile>(value: &T, path: &Path) -> std::io::Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let text = serde_json::to_string_pretty(value)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
    std::fs::write(path, text)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};

    /// A scratch directory that removes itself. Shared by every config-file test.
    pub(crate) struct Scratch(pub PathBuf);

    impl Scratch {
        pub(crate) fn new(tag: &str) -> Self {
            static N: AtomicU32 = AtomicU32::new(0);
            let d = std::env::temp_dir().join(format!(
                "ocam-{tag}-{}-{}",
                std::process::id(),
                N.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir_all(&d).expect("scratch dir");
            Self(d)
        }
        pub(crate) fn file(&self, name: &str) -> PathBuf {
            self.0.join(name)
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// A minimal type exercising the contract itself, independent of any real file —
    /// so the guarantees are tested once rather than once per config file.
    #[derive(Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
    #[serde(default)]
    struct Probe {
        version: u32,
        value: i32,
        #[serde(flatten)]
        extra: serde_json::Map<String, serde_json::Value>,
    }

    impl ConfigFile for Probe {
        const FILE_NAME: &'static str = "probe.json";
        const VERSION: u32 = 3;
        const SEED_ON_MISSING: bool = false;
        const WHAT: &'static str = "a probe";
        fn stated_version(&self) -> u32 {
            self.version
        }
        fn migrate(&mut self, from: u32) {
            // Each step doubles, so a v1 file arrives at v3 having been through both.
            for _ in from..Self::VERSION {
                self.value *= 2;
            }
            self.version = Self::VERSION;
        }
        fn normalise(&mut self) {
            self.value = self.value.clamp(-100, 100);
        }
    }

    #[derive(Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
    #[serde(default)]
    struct Seeded {
        version: u32,
        items: Vec<i32>,
    }

    impl ConfigFile for Seeded {
        const FILE_NAME: &'static str = "seeded.json";
        const VERSION: u32 = 1;
        const SEED_ON_MISSING: bool = true;
        const WHAT: &'static str = "a seeded list";
        fn stated_version(&self) -> u32 {
            self.version
        }
        fn migrate(&mut self, _from: u32) {
            self.version = Self::VERSION;
        }
        fn seed() -> Self {
            Self {
                version: 1,
                items: vec![7, 8, 9],
            }
        }
    }

    #[test]
    fn a_missing_file_writes_nothing_unless_the_type_seeds() {
        let s = Scratch::new("cfg");
        let (p, outcome) = load_from::<Probe>(&s.file("probe.json"));
        assert_eq!(outcome, ConfigLoad::Fresh);
        assert_eq!(p, Probe::default());
        assert!(!s.file("probe.json").exists(), "settings-like files stay absent");

        let (v, outcome) = load_from::<Seeded>(&s.file("seeded.json"));
        assert_eq!(outcome, ConfigLoad::Seeded);
        assert_eq!(v.items, vec![7, 8, 9]);
        assert!(s.file("seeded.json").exists(), "a library seeds itself on first run");
    }

    /// **The contract that matters**, tested once for every file that uses it.
    #[test]
    fn an_unreadable_file_is_never_rewritten_and_is_backed_up() {
        let s = Scratch::new("cfg");
        let precious = "{ not json, but it is THEIRS";
        std::fs::write(s.file("probe.json"), precious).unwrap();

        let (p, outcome) = load_from::<Probe>(&s.file("probe.json"));
        assert!(matches!(outcome, ConfigLoad::Rejected(_)), "{outcome:?}");
        assert_eq!(p, Probe::default());
        assert_eq!(std::fs::read_to_string(s.file("probe.json")).unwrap(), precious);
        assert_eq!(std::fs::read_to_string(s.file("probe.json.bak")).unwrap(), precious);
    }

    /// A rejected load falls back to [`ConfigFile::seed`], **not** `Default`. Caught by
    /// the tool-library tests when this contract was first shared: `Default` for a
    /// library is the *empty* library, so a file that would not parse left the user with
    /// no tools at all rather than the starter set.
    #[test]
    fn a_rejected_load_falls_back_to_the_seed_not_to_default() {
        let s = Scratch::new("cfg");
        std::fs::write(s.file("seeded.json"), "{ not json").unwrap();
        let (v, outcome) = load_from::<Seeded>(&s.file("seeded.json"));
        assert!(matches!(outcome, ConfigLoad::Rejected(_)));
        assert_eq!(v.items, vec![7, 8, 9], "a usable app, not an empty one");
        assert_ne!(v, Seeded::default());
    }

    #[test]
    fn a_newer_version_is_refused_not_read_leniently() {
        let s = Scratch::new("cfg");
        let newer = r#"{"version": 99, "value": 5}"#;
        std::fs::write(s.file("probe.json"), newer).unwrap();
        let (p, outcome) = load_from::<Probe>(&s.file("probe.json"));
        let ConfigLoad::Rejected(why) = &outcome else {
            panic!("expected a refusal, got {outcome:?}")
        };
        assert!(why.contains("newer version"), "{why}");
        assert_eq!(p, Probe::default());
        assert_eq!(std::fs::read_to_string(s.file("probe.json")).unwrap(), newer);
    }

    #[test]
    fn an_older_file_is_migrated_then_normalised() {
        let s = Scratch::new("cfg");
        // v1 → v3 doubles twice: 10 → 40. Then the clamp applies to the *migrated*
        // value, which is the order that matters — normalising first would let a
        // migration push a value back out of range.
        std::fs::write(s.file("probe.json"), r#"{"version": 1, "value": 10}"#).unwrap();
        let (p, outcome) = load_from::<Probe>(&s.file("probe.json"));
        assert_eq!(outcome, ConfigLoad::Loaded);
        assert_eq!(p.value, 40);
        assert_eq!(p.version, 3);

        std::fs::write(s.file("probe.json"), r#"{"version": 1, "value": 90}"#).unwrap();
        let (p, _) = load_from::<Probe>(&s.file("probe.json"));
        assert_eq!(p.value, 100, "90 doubles twice to 360 and is then clamped");
    }

    #[test]
    fn unknown_keys_survive_a_round_trip() {
        let s = Scratch::new("cfg");
        std::fs::write(
            s.file("probe.json"),
            r#"{"version": 3, "value": 1, "from_the_future": true}"#,
        )
        .unwrap();
        let (p, _) = load_from::<Probe>(&s.file("probe.json"));
        save_to(&p, &s.file("probe.json")).unwrap();
        let text = std::fs::read_to_string(s.file("probe.json")).unwrap();
        assert!(text.contains("from_the_future"), "{text}");
    }
}
