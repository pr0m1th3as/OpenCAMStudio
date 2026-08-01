//! The local machine library — `MACHINE_PLAN.md` step 2.
//!
//! # Why it is local
//!
//! **A machine is shop-local; a project file is not.** The envelope is what *gates* an
//! export (`check_travel` refuses a program that does not fit), so a file that could set
//! your machine could disarm that gate — a job authored on a 1000 mm router, emailed to
//! someone with a 300 mm mill, would be verified against the sender's travel and pass.
//! A `.ocam` therefore records the machine it was built for as **provenance only**
//! (Andreas, 2026-08-01: *"machine + control local, `.ocam` file as provenance"*).
//!
//! This module is the other half: a user has more than one machine, so there must be a
//! set to pick from, and it lives here rather than in any project.
//!
//! # Active, not preferred
//!
//! Which machine is selected is **session state that persists** — last-used wins — not a
//! default nominated in a panel. Two settings answering "which machine do I start on" is
//! one too many; when they disagree someone authors against the wrong travel. The safety
//! comes from the active machine being *visible*, not from how it was chosen. The
//! selection itself lives in `settings.json`; this file holds only the set.

use cam_model::Machine;
use serde::{Deserialize, Serialize};

use crate::config::ConfigFile;

/// The machine the app has always started with, and what a first run seeds the library
/// with — so introducing the library changes nothing until a user adds a second.
///
/// Lives here rather than in the GUI because the library needs it and the library is not
/// GUI-gated: `cam-app` builds and tests without the `gui` feature.
pub fn default_machine() -> Machine {
    Machine {
        name: "desktop".into(),
        rapid_rate: 2000.0,
        max_spindle_rpm: 10_000.0,
        max_feed: 800.0,
        envelope: cam_model::Envelope::new(
            cam_cldata::Point3::new(0.0, 0.0, -50.0),
            cam_cldata::Point3::new(300.0, 180.0, 50.0),
        ),
        safe_z: 5.0,
        tool_change_pos: None,
    }
}

/// The machine library file's format version.
pub const MACHINES_VERSION: u32 = 1;

/// One machine, and the control it is wired to.
///
/// **The post lives here, not on [`Machine`]**, for two reasons. Layering: `cam-model`
/// knows nothing of `cam-post`, and should not learn. And meaning: which control a
/// machine has is a fact about *this shop's* installation of it, which is exactly what
/// this library is for.
///
/// The machine's own fields are flattened, so a v1 `machines.json` — written before the
/// post existed — loads unchanged with the post defaulting. Additive, no version step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MachineEntry {
    /// Travel, feeds, spindle ceiling, safe Z.
    #[serde(flatten)]
    pub machine: Machine,
    /// The control this machine has, and therefore the post its programs are written
    /// for.
    ///
    /// **Selecting a machine selects this.** The library exists because a shop has more
    /// than one machine, and more than one machine almost always means more than one
    /// control — so "pick the small mill, get Okuma G-code because that is what the last
    /// job used" is a mistake the library itself creates. Tying the two removes it.
    #[serde(default)]
    pub post: cam_post::PostKind,
}

impl MachineEntry {
    /// A machine with the default control.
    pub fn new(machine: Machine) -> Self {
        Self {
            machine,
            post: cam_post::PostKind::default(),
        }
    }

    /// This machine's name — the handle the selection is remembered by.
    pub fn name(&self) -> &str {
        &self.machine.name
    }
}

/// The machines this installation knows about.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MachineLibrary {
    /// Format version — see [`MACHINES_VERSION`].
    pub machines_version: u32,
    /// Every machine, in the order they are shown.
    pub machines: Vec<MachineEntry>,
    /// Keys this build does not know about, carried through a save untouched.
    #[serde(flatten)]
    pub extra: serde_json::Map<String, serde_json::Value>,
}

impl Default for MachineLibrary {
    /// An **empty** library at the current version — not the starter machine. Use
    /// [`MachineLibrary::seeded`] for that; conflating the two is how a test ends up
    /// asserting against whatever the shipped defaults happen to be.
    fn default() -> Self {
        Self {
            machines_version: MACHINES_VERSION,
            machines: Vec::new(),
            extra: Default::default(),
        }
    }
}

impl MachineLibrary {
    /// The library a first run gets: exactly the single machine the app used before this
    /// file existed, so introducing it changes nothing until the user adds a second.
    pub fn seeded() -> Self {
        Self {
            machines_version: MACHINES_VERSION,
            machines: vec![MachineEntry::new(default_machine())],
            extra: Default::default(),
        }
    }

    /// The entry with this name.
    pub fn by_name(&self, name: &str) -> Option<&MachineEntry> {
        self.machines.iter().find(|m| m.name() == name)
    }

    /// The entry with this name, mutably.
    pub fn by_name_mut(&mut self, name: &str) -> Option<&mut MachineEntry> {
        self.machines.iter_mut().find(|m| m.name() == name)
    }

    /// Every machine's name, in order — what the picker shows.
    pub fn names(&self) -> Vec<String> {
        self.machines.iter().map(|m| m.name().to_string()).collect()
    }

    /// Resolve a remembered selection to an actual machine.
    ///
    /// A name that no longer resolves — the machine was deleted, or the settings file
    /// was copied to another installation — falls back to the **first** in the library
    /// rather than to nothing. The caller is expected to say so: silently landing on a
    /// different machine than the one you left on is precisely the failure this whole
    /// area exists to prevent.
    pub fn resolve(&self, name: Option<&str>) -> Option<&MachineEntry> {
        name.and_then(|n| self.by_name(n)).or_else(|| self.machines.first())
    }

    /// Whether `name` names a machine that is actually present.
    pub fn resolves(&self, name: Option<&str>) -> bool {
        name.is_some_and(|n| self.by_name(n).is_some())
    }

    /// Add `machine`, giving it a name no existing entry uses.
    ///
    /// Names are the handle everywhere else — the selection is remembered by name, and
    /// the provenance note quotes it — so a duplicate would make "which machine"
    /// ambiguous in the one place it must not be.
    pub fn add(&mut self, mut entry: MachineEntry) -> String {
        if self.by_name(entry.name()).is_some() {
            let base = entry.name().to_string();
            let mut n = 2;
            while self.by_name(&format!("{base} ({n})")).is_some() {
                n += 1;
            }
            entry.machine.name = format!("{base} ({n})");
        }
        let name = entry.name().to_string();
        self.machines.push(entry);
        name
    }

    /// Replace the entry called `was` — used when the active machine is edited, since
    /// editing may change the very name it is keyed by. Returns the (possibly
    /// disambiguated) new name.
    pub fn replace(&mut self, was: &str, entry: MachineEntry) -> String {
        let renamed = entry.name() != was;
        let clashes = renamed && self.by_name(entry.name()).is_some();
        let Some(slot) = self.machines.iter().position(|m| m.name() == was) else {
            return self.add(entry);
        };
        let mut entry = entry;
        if clashes {
            // Same rule as `add`: names are the handle, so they stay unique.
            let base = entry.name().to_string();
            let mut n = 2;
            while self.by_name(&format!("{base} ({n})")).is_some() {
                n += 1;
            }
            entry.machine.name = format!("{base} ({n})");
        }
        let name = entry.name().to_string();
        self.machines[slot] = entry;
        name
    }

    /// Remove the machine with this name, returning whether it was there.
    ///
    /// The **last** machine cannot be removed: an empty library would leave nothing to
    /// gate an export against, and `Machine` has no meaningful null.
    pub fn remove(&mut self, name: &str) -> bool {
        if self.machines.len() <= 1 {
            return false;
        }
        let before = self.machines.len();
        self.machines.retain(|m| m.name() != name);
        self.machines.len() != before
    }
}

impl ConfigFile for MachineLibrary {
    const FILE_NAME: &'static str = "machines.json";
    const VERSION: u32 = MACHINES_VERSION;
    /// A library seeds itself: with no machines there is nothing to check an export
    /// against, so a first run gets the one the app already used.
    const SEED_ON_MISSING: bool = true;
    const WHAT: &'static str = "a machine library";

    fn stated_version(&self) -> u32 {
        self.machines_version
    }

    fn migrate(&mut self, _from: u32) {
        // v1 is the first version. The loop shape lives in `crate::config`; when a step
        // is needed it goes here.
        self.machines_version = MACHINES_VERSION;
    }

    fn normalise(&mut self) {
        // A library with no machines cannot gate an export against anything, and a
        // hand-edited file can easily arrive that way.
        if self.machines.is_empty() {
            self.machines = Self::seeded().machines;
        }
    }

    fn seed() -> Self {
        Self::seeded()
    }
}

/// Read the machine library from the config directory.
pub fn load() -> (MachineLibrary, MachineLoad) {
    let (lib, outcome) = crate::config::load::<MachineLibrary>();
    (lib, outcome.into())
}

/// Read it from `path`. See [`crate::config`] for the failure contract.
pub fn load_from(path: &std::path::Path) -> (MachineLibrary, MachineLoad) {
    let (lib, outcome) = crate::config::load_from::<MachineLibrary>(path);
    (lib, outcome.into())
}

impl MachineLibrary {
    /// Persist to the config directory (best-effort).
    pub fn save(&self) {
        crate::config::save(self)
    }

    /// Persist to `path`.
    pub fn save_to(&self, path: &std::path::Path) -> std::io::Result<()> {
        crate::config::save_to(self, path)
    }
}

/// What happened when the machine library was read.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MachineLoad {
    /// No file: seeded with the app's existing single machine, and written.
    Seeded,
    /// Read and adopted.
    Loaded,
    /// A file exists but could not be used. The seeded library is in force, **the file
    /// was left exactly as it was**, and a `.bak` copy sits beside it.
    Rejected(String),
}

impl From<crate::config::ConfigLoad> for MachineLoad {
    fn from(c: crate::config::ConfigLoad) -> Self {
        match c {
            crate::config::ConfigLoad::Fresh | crate::config::ConfigLoad::Seeded => {
                MachineLoad::Seeded
            }
            crate::config::ConfigLoad::Loaded => MachineLoad::Loaded,
            crate::config::ConfigLoad::Rejected(w) => MachineLoad::Rejected(w),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::tests::Scratch;

    fn named(name: &str) -> MachineEntry {
        MachineEntry::new(Machine {
            name: name.into(),
            ..default_machine()
        })
    }

    /// **The inertness test.** Introducing the library must change nothing until a user
    /// adds a second machine: a first run gets exactly the machine the app used before
    /// this file existed.
    #[test]
    fn a_first_run_seeds_the_machine_the_app_already_used() {
        let s = Scratch::new("machines");
        let (lib, outcome) = load_from(&s.file("machines.json"));
        assert_eq!(outcome, MachineLoad::Seeded);
        assert_eq!(lib.machines, vec![MachineEntry::new(default_machine())]);
        assert!(s.file("machines.json").exists());

        let (again, outcome) = load_from(&s.file("machines.json"));
        assert_eq!(outcome, MachineLoad::Loaded);
        assert_eq!(again, lib, "and it reloads as itself");
    }

    /// The contract is shared, so this checks the part that is *this* file's: a library
    /// that will not parse must not cost the user their machines, and must not leave
    /// them with none.
    #[test]
    fn an_unreadable_library_keeps_the_file_and_still_yields_a_usable_machine() {
        let s = Scratch::new("machines");
        let precious = r#"{"machines": [ corrupt but THEIRS"#;
        std::fs::write(s.file("machines.json"), precious).unwrap();

        let (lib, outcome) = load_from(&s.file("machines.json"));
        assert!(matches!(outcome, MachineLoad::Rejected(_)), "{outcome:?}");
        assert_eq!(
            std::fs::read_to_string(s.file("machines.json")).unwrap(),
            precious
        );
        assert_eq!(
            std::fs::read_to_string(s.file("machines.json.bak")).unwrap(),
            precious
        );
        assert!(!lib.machines.is_empty(), "never leave the app with no machine");
    }

    /// A hand-edited file can arrive with an empty list, which would leave nothing to
    /// gate an export against.
    #[test]
    fn an_empty_library_is_refilled_on_load() {
        let s = Scratch::new("machines");
        std::fs::write(
            s.file("machines.json"),
            r#"{"machines_version": 1, "machines": []}"#,
        )
        .unwrap();
        let (lib, outcome) = load_from(&s.file("machines.json"));
        assert_eq!(outcome, MachineLoad::Loaded);
        assert!(!lib.machines.is_empty());
    }

    /// The selection is remembered **by name**, so names must be unique — otherwise
    /// "which machine" is ambiguous in the one place it must not be.
    #[test]
    fn adding_a_duplicate_name_disambiguates_it() {
        let mut lib = MachineLibrary::default();
        assert_eq!(lib.add(named("Router")), "Router");
        assert_eq!(lib.add(named("Router")), "Router (2)");
        assert_eq!(lib.add(named("Router")), "Router (3)");
        assert_eq!(lib.machines.len(), 3);
    }

    /// A name that no longer resolves falls back to the first machine rather than to
    /// nothing — the caller is expected to *say so*, because silently landing on a
    /// different machine is the failure this area exists to prevent.
    #[test]
    fn a_stale_selection_falls_back_to_the_first_machine() {
        let mut lib = MachineLibrary::default();
        lib.add(named("Router"));
        lib.add(named("Mill"));

        assert_eq!(lib.resolve(Some("Mill")).unwrap().name(), "Mill");
        assert!(lib.resolves(Some("Mill")));

        assert_eq!(
            lib.resolve(Some("Deleted")).unwrap().name(),
            "Router",
            "a stale name lands on the first machine, not on nothing"
        );
        assert!(!lib.resolves(Some("Deleted")), "…and the caller can tell");
        assert_eq!(lib.resolve(None).unwrap().name(), "Router");
    }

    /// The last machine cannot be removed: an empty library has nothing to gate an
    /// export against.
    #[test]
    fn the_last_machine_cannot_be_removed() {
        let mut lib = MachineLibrary::default();
        lib.add(named("Router"));
        lib.add(named("Mill"));
        assert!(lib.remove("Mill"));
        assert!(!lib.remove("Router"), "the last one stays");
        assert_eq!(lib.machines.len(), 1);
        assert!(!lib.remove("Never existed"));
    }

    /// A `machines.json` written before machines had a control loads unchanged, with the
    /// post defaulting — additive, so no version step. The machine's own fields are
    /// flattened precisely so this holds.
    #[test]
    fn a_library_written_before_the_post_existed_still_loads() {
        let s = Scratch::new("machines");
        std::fs::write(
            s.file("machines.json"),
            r#"{"machines_version": 1, "machines": [
                 {"name": "Router", "rapid_rate": 2000.0, "max_spindle_rpm": 10000.0,
                  "max_feed": 800.0, "safe_z": 5.0, "tool_change_pos": null,
                  "envelope": {"min": [0.0,0.0,-50.0], "max": [300.0,180.0,50.0]}}]}"#,
        )
        .unwrap();
        let (lib, outcome) = load_from(&s.file("machines.json"));
        assert_eq!(outcome, MachineLoad::Loaded);
        assert_eq!(lib.machines.len(), 1);
        assert_eq!(lib.machines[0].name(), "Router");
        assert_eq!(lib.machines[0].post, cam_post::PostKind::default());
        assert_eq!(lib.machines[0].machine.max_feed, 800.0);
    }

    /// Editing the active machine can rename it — and the name is the handle the
    /// selection is remembered by, so the replacement has to keep names unique too.
    #[test]
    fn replacing_an_entry_follows_a_rename_and_still_disambiguates() {
        let mut lib = MachineLibrary::default();
        lib.add(named("Router"));
        lib.add(named("Mill"));

        let renamed = lib.replace("Router", named("Big Router"));
        assert_eq!(renamed, "Big Router");
        assert!(lib.by_name("Router").is_none());
        assert_eq!(lib.machines.len(), 2);

        // Renaming onto a name already taken must not create a second "Mill".
        let clashed = lib.replace("Big Router", named("Mill"));
        assert_eq!(clashed, "Mill (2)");
        assert_eq!(lib.machines.len(), 2);
    }

    /// A rename onto a name already taken comes back disambiguated, and the caller is
    /// expected to adopt what it returns — otherwise the inspector shows one name while
    /// the library holds another, and the selection is remembered by the library's.
    #[test]
    fn replace_reports_the_name_it_actually_used() {
        let mut lib = MachineLibrary::default();
        lib.add(named("Router"));
        lib.add(named("Mill"));
        let settled = lib.replace("Router", named("Mill"));
        assert_eq!(settled, "Mill (2)", "the caller must be told, not left guessing");
        assert!(lib.by_name("Mill (2)").is_some());
        assert!(lib.by_name("Router").is_none());
        assert_eq!(lib.machines.len(), 2);
    }

    #[test]
    fn default_is_empty_and_seeded_is_the_starter_machine() {
        assert!(MachineLibrary::default().machines.is_empty());
        assert_eq!(MachineLibrary::seeded().machines.len(), 1);
        assert_eq!(MachineLibrary::default().machines_version, MACHINES_VERSION);
    }
}
