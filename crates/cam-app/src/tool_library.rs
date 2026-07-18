//! The cross-project **tool library**: a persistent list of tool definitions the
//! user picks from during operation setup. It lives in the platform config
//! directory (not in any one project); a project embeds copies of the tools it
//! actually uses, so `.ocam` files stay self-contained.
//!
//! GUI-only: the library is session/app state, loaded at startup by the shell.

use std::path::PathBuf;

use cam_model::{Tool, ToolKind};

use crate::controller::OpKind;

/// A fresh tool of `kind` with sensible default dimensions and shop `number`. The
/// end-mill family (and everything else) uses `flute = 2·⌀`, `shank = 2.5·flute`
/// (overall = flute + shank); a **face mill** seeds as a real shell mill — a wide,
/// squat cutting body on a short, narrower arbor. Used both by "New" and by a Type
/// change that crosses tool families (so, e.g., a face mill is never left at ⌀6).
pub fn default_tool(number: u32, kind: ToolKind) -> Tool {
    if matches!(kind, ToolKind::FaceMill) {
        return Tool {
            number,
            diameter: 50.0,       // cutting ⌀ (the disc)
            flute_length: 30.0,   // body height (the wide body)
            shank_diameter: 22.0, // arbor ⌀ (the stub above the body)
            length: 42.0,         // overall (arbor sticks up 12 mm)
            flutes: 5,            // inserts
            kind,
            ..Default::default()
        };
    }
    let diameter = 6.0;
    let flute = 2.0 * diameter;
    let shank = 2.5 * flute;
    Tool {
        number,
        diameter,
        flute_length: flute,
        shank_diameter: diameter, // explicit shank ⌀ (= flute ⌀ by default)
        length: flute + shank,
        flutes: 2,
        kind,
        ..Default::default()
    }
}

/// A reusable set of tool definitions, persisted to the config directory.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ToolLibrary {
    pub tools: Vec<Tool>,
}

impl ToolLibrary {
    /// The starter library seeded on first run: a few common end mills.
    pub fn defaults() -> Self {
        // flute = 2·⌀, shank = 2.5·flute, overall = flute + shank (the end-mill convention).
        let em = |number, diameter: f64| {
            let flute = 2.0 * diameter;
            Tool {
                number,
                diameter,
                flute_length: flute,
                shank_diameter: diameter,
                length: flute + 2.5 * flute,
                flutes: 2,
                kind: ToolKind::EndMill,
                ..Default::default()
            }
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

    /// The library tool to seed a newly-created operation of `kind` with — a
    /// sensible starting default the user can still override in the wizard picker.
    ///
    /// Every op has a **natural tool kind** and defaults to the first tool of that
    /// kind: Profile/Pocket → `EndMill`, Drill → `Drill`, Thread → `ThreadMill`,
    /// Chamfer → `ChamferMill`. **Face** is the one that also sorts by size: it
    /// wants a flat-bottomed tool (a chamfer or ball tool would leave a scalloped
    /// floor), and the *largest* such tool means the fewest passes, so facing
    /// prefers the biggest `EndMill`/`FaceMill`.
    ///
    /// In every case this is only a **default** — we never *reject* a tool the user
    /// picks; and if no tool of the preferred kind exists we fall back to the first
    /// library tool (order = the user's own numbering). `None` only when the
    /// library is empty.
    pub fn default_tool_for(&self, kind: OpKind) -> Option<Tool> {
        let first = |pred: fn(&ToolKind) -> bool| self.tools.iter().find(|t| pred(&t.kind));
        let preferred = match kind {
            // Facing wants the *largest* flat tool (fewest passes) — scallop-safe.
            OpKind::Face => self
                .tools
                .iter()
                .filter(|t| matches!(t.kind, ToolKind::EndMill | ToolKind::FaceMill))
                .max_by(|a, b| a.diameter.total_cmp(&b.diameter)),
            // The rest take the first tool of their kind (no size preference).
            OpKind::Profile | OpKind::Pocket => first(|k| matches!(k, ToolKind::EndMill)),
            OpKind::Drill => first(|k| matches!(k, ToolKind::Drill { .. })),
            OpKind::Thread => first(|k| matches!(k, ToolKind::ThreadMill { .. })),
            OpKind::Chamfer => first(|k| matches!(k, ToolKind::ChamferMill { .. })),
        };
        preferred.or_else(|| self.tools.first()).copied()
    }

    /// Append a fresh default tool (numbered one past the highest) and return its
    /// index. The caller typically selects it and edits its fields.
    pub fn add_default(&mut self) -> usize {
        self.add_of_kind(ToolKind::EndMill)
    }

    /// Append a fresh default tool of a **given kind** (its kind-specific parameters
    /// carried through) and return its index — so "New" can seed the type from whatever
    /// tool is currently selected (a chamfer mill begets a chamfer mill, etc.).
    ///
    /// Seeded with a sensible flute/shank split: `flute = 2·⌀`, `shank = 2.5·flute`
    /// (so `overall = flute + shank`), the end-mill defaults Andreas specified. A
    /// **face mill** instead seeds as a real shell mill — a wide, squat cutting body
    /// (⌀ = cutting ⌀, height = flute length) on a short, narrower arbor (shank ⌀).
    pub fn add_of_kind(&mut self, kind: ToolKind) -> usize {
        let number = self.next_number();
        self.tools.push(default_tool(number, kind));
        self.tools.len() - 1
    }

    /// The **lowest free** shop number (≥ 1) — fills gaps left by deleted tools rather
    /// than always taking `max + 1`.
    fn next_number(&self) -> u32 {
        let used: std::collections::BTreeSet<u32> = self.tools.iter().map(|t| t.number).collect();
        (1..).find(|n| !used.contains(n)).expect("u32 space is never exhausted")
    }

    /// Set the number of the tool at `index` to `new`, **swapping** with whatever tool
    /// currently holds `new` so numbers stay unique. No-op if the index is out of range,
    /// `new` is 0, or the number is unchanged.
    pub fn set_number(&mut self, index: usize, new: u32) {
        if new == 0 {
            return;
        }
        let Some(old) = self.tools.get(index).map(|t| t.number) else {
            return;
        };
        if old == new {
            return;
        }
        if let Some(other) = self.tools.iter().position(|t| t.number == new) {
            self.tools[other].number = old; // swap
        }
        self.tools[index].number = new;
    }

    /// Bulk-renumber the tools **sequentially 1..N** in the given order (`order[k]` is
    /// the library index that becomes tool number `k+1`). Used by the guarded "Renumber"
    /// action to normalise the numbering (e.g. by family).
    pub fn set_numbers_in_order(&mut self, order: &[usize]) {
        for (k, &idx) in order.iter().enumerate() {
            if let Some(t) = self.tools.get_mut(idx) {
                t.number = (k + 1) as u32;
            }
        }
    }

    /// Promote `tool` into the shop library (the "Add to library" action, §6.3),
    /// **idempotent by geometry**: if a tool of the same [`identity`](Tool::identity)
    /// already exists, its number is returned and nothing is inserted; otherwise the
    /// tool is added with the next free shop number, which is returned. The caller
    /// then reconciles the project so the promoted tool adopts this number.
    pub fn add_tool(&mut self, mut tool: Tool) -> u32 {
        if let Some(existing) = self.tools.iter().find(|t| t.identity() == tool.identity()) {
            return existing.number;
        }
        let number = self.next_number();
        tool.number = number;
        self.tools.push(tool);
        number
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

    /// A bare tool of a given number/diameter/kind (length/flutes irrelevant here).
    fn mk(number: u32, diameter: f64, kind: ToolKind) -> Tool {
        Tool {
            number,
            diameter,
            length: 30.0,
            flutes: 2,
            kind,
            ..Default::default()
        }
    }

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
    fn face_defaults_to_the_largest_flat_tool() {
        // Starter library is three end mills (⌀3/6/10) — Face should pick ⌀10.
        let lib = ToolLibrary::defaults();
        let t = lib.default_tool_for(OpKind::Face).unwrap();
        assert_eq!(t.diameter, 10.0);
        assert!(matches!(t.kind, ToolKind::EndMill));
        // A non-Face op keeps the first library tool ( diameter order is the
        // user's own numbering, not a size preference).
        assert_eq!(
            lib.default_tool_for(OpKind::Profile).unwrap().number,
            lib.tools[0].number
        );
    }

    #[test]
    fn face_skips_non_flat_tools_but_falls_back_when_none_flat() {
        // A big chamfer mill must not win facing over a smaller flat end mill.
        let lib = ToolLibrary {
            tools: vec![
                mk(1, 20.0, ToolKind::ChamferMill {
                    included_angle_deg: 90.0,
                    tip_diameter: 0.5,
                }),
                mk(2, 8.0, ToolKind::EndMill),
                mk(3, 16.0, ToolKind::FaceMill),
            ],
        };
        let t = lib.default_tool_for(OpKind::Face).unwrap();
        assert_eq!(t.number, 3, "the ⌀16 face mill is the largest flat tool");

        // With only non-flat tools, fall back to the first (never leave Face
        // without a tool — the machinist can still change it).
        let only_chamfer = ToolLibrary {
            tools: vec![mk(1, 20.0, ToolKind::ChamferMill {
                included_angle_deg: 90.0,
                tip_diameter: 0.5,
            })],
        };
        assert_eq!(only_chamfer.default_tool_for(OpKind::Face).unwrap().number, 1);
    }

    #[test]
    fn kinded_ops_default_to_the_first_matching_tool_by_kind() {
        // Order deliberately shuffled: the *first match by kind* must win, not the
        // first tool overall, and size must not matter (only Face sorts by size).
        let lib = ToolLibrary {
            tools: vec![
                mk(1, 10.0, ToolKind::EndMill),
                mk(2, 3.2, ToolKind::Drill { point_angle_deg: 118.0 }),
                mk(3, 6.0, ToolKind::Drill { point_angle_deg: 135.0 }),
                mk(4, 8.0, ToolKind::ChamferMill {
                    included_angle_deg: 90.0,
                    tip_diameter: 0.0,
                }),
                mk(5, 12.0, ToolKind::ThreadMill { pitch: None }),
            ],
        };
        assert_eq!(lib.default_tool_for(OpKind::Drill).unwrap().number, 2, "first Drill");
        assert_eq!(lib.default_tool_for(OpKind::Chamfer).unwrap().number, 4, "first ChamferMill");
        assert_eq!(lib.default_tool_for(OpKind::Thread).unwrap().number, 5, "first ThreadMill");
        // Profile/Pocket want the first end mill (tool 1 here).
        assert_eq!(lib.default_tool_for(OpKind::Profile).unwrap().number, 1);
        assert_eq!(lib.default_tool_for(OpKind::Pocket).unwrap().number, 1);
    }

    #[test]
    fn profile_and_pocket_pick_the_first_end_mill_past_a_leading_drill() {
        // A drill numbered first must not become the profile/pocket default.
        let lib = ToolLibrary {
            tools: vec![
                mk(1, 6.0, ToolKind::Drill { point_angle_deg: 118.0 }),
                mk(2, 8.0, ToolKind::EndMill),
                mk(3, 4.0, ToolKind::EndMill),
            ],
        };
        assert_eq!(lib.default_tool_for(OpKind::Profile).unwrap().number, 2);
        assert_eq!(lib.default_tool_for(OpKind::Pocket).unwrap().number, 2);
    }

    #[test]
    fn kinded_ops_fall_back_to_first_when_no_matching_kind() {
        // A library of only end mills: every kinded op still gets a (valid) tool.
        let lib = ToolLibrary::defaults();
        for kind in [OpKind::Drill, OpKind::Thread, OpKind::Chamfer] {
            assert_eq!(
                lib.default_tool_for(kind).unwrap().number,
                lib.tools[0].number,
                "{kind:?} falls back to the first library tool"
            );
        }
    }

    #[test]
    fn add_default_appends_with_next_number() {
        let mut lib = ToolLibrary::defaults();
        let top = lib.tools.iter().map(|t| t.number).max().unwrap();
        let i = lib.add_default();
        assert_eq!(i, lib.tools.len() - 1);
        assert_eq!(lib.tools[i].number, top + 1);
    }

    #[test]
    fn new_tool_fills_the_lowest_free_number() {
        // Starter is 1/2/3; delete #2 so there is a gap.
        let mut lib = ToolLibrary::defaults();
        lib.tools.retain(|t| t.number != 2);
        let idx = lib.add_default();
        assert_eq!(lib.tools[idx].number, 2, "the freed #2 is reused, not #4");
        // Now 1/2/3 again → next is 4.
        lib.add_default();
        assert_eq!(lib.tools.last().unwrap().number, 4);
    }

    #[test]
    fn set_number_swaps_on_collision() {
        let mut lib = ToolLibrary::defaults(); // #1, #2, #3
        // Move #3 (index 2) onto #1 → the two swap.
        lib.set_number(2, 1);
        assert_eq!(lib.tools[2].number, 1, "target adopted");
        assert_eq!(lib.tools[0].number, 3, "the tool that held #1 took #3");
        // Numbers stay a unique set {1,2,3}.
        let nums: std::collections::BTreeSet<u32> = lib.tools.iter().map(|t| t.number).collect();
        assert_eq!(nums.len(), lib.tools.len());
        // Moving to a free number just takes it, no swap.
        lib.set_number(0, 9); // index 0 currently #3
        assert_eq!(lib.tools[0].number, 9);
    }

    #[test]
    fn add_tool_is_idempotent_by_geometry() {
        // Starter library is ⌀3/6/10 end mills.
        let mut lib = ToolLibrary::defaults();
        let before = lib.tools.len();

        // A genuinely new tool gets the next free number and is inserted.
        let new = mk(99, 8.0, ToolKind::Drill { point_angle_deg: 118.0 });
        let n = lib.add_tool(new);
        assert_eq!(n, 4, "next free shop number");
        assert_eq!(lib.tools.len(), before + 1);

        // Re-adding the *same geometry* (any number) is a no-op returning its number.
        let dup = mk(123, 8.0, ToolKind::Drill { point_angle_deg: 118.0 });
        let n2 = lib.add_tool(dup);
        assert_eq!(n2, 4, "identical geometry returns the existing number");
        assert_eq!(lib.tools.len(), before + 1, "no duplicate inserted");

        // Re-adding an *existing* library tool (its exact geometry) is a no-op that
        // returns its number, regardless of the number on the copy.
        let mut existing = lib.tools[1]; // the ⌀6 starter end mill (#2)
        existing.number = 77;
        assert_eq!(lib.add_tool(existing), 2);
        assert_eq!(lib.tools.len(), before + 1);
    }
}
