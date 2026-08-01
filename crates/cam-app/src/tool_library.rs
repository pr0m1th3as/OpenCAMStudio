//! The cross-project **tool library**: a persistent list of tool definitions the
//! user picks from during operation setup. It lives in the platform config
//! directory (not in any one project); a project embeds copies of the tools it
//! actually uses, so `.ocam` files stay self-contained.
//!
//! Loaded at startup by the shell, but **not GUI-gated**: the library, the tool
//! defaults and the per-operation tool *families* are plain data, and only their
//! presentation is GUI. Keeping them here is what lets the headless tests assert
//! that a fresh install can actually start all eight operations.

use std::path::{Path, PathBuf};

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
    if matches!(kind, ToolKind::ThreadMill { .. }) {
        // Single-profile (single-point) thread mill with typical proportions: one 60°
        // tooth at the tip (min cutting ⌀), a reduced neck over the length of cut (its
        // reach), then a standard shank. Full-form uses the same body (its pitch drives
        // the stacked-teeth silhouette instead).
        return Tool {
            number,
            diameter: 4.8,        // min cutting ⌀ (smallest hole it threads)
            neck_diameter: 3.5,   // reduced neck (sets max thread depth = (4.8−3.5)/2)
            flute_length: 15.0,   // length of cut (reach)
            shank_diameter: 6.0,  // shank ⌀
            length: 55.0,         // overall
            flutes: 3,
            kind,                 // carries the pitch (None = single-point by default)
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

/// The tool library file's format version.
///
/// Its own counter, like the settings file's — `cam_model::SCHEMA_VERSION` describes
/// the *document*, and a library is not a document. v1 is the shape every library
/// ever written already has, which is why an unversioned file reads as v1 rather
/// than being refused.
pub const LIBRARY_VERSION: u32 = 1;

/// A library file with no `library_version` predates the field. Every such file is
/// the v1 shape — the field was added *because* the format needed a version, not
/// because the format changed — so reading it as v1 is the truth, not a guess.
fn unversioned_is_v1() -> u32 {
    1
}

/// What happened when the library file was read. Returned rather than swallowed so
/// the app can *say* the library did not load, instead of the user meeting an
/// unexpected set of stock tools and wondering where theirs went.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LibraryLoad {
    /// No file yet: the starter library is in force, and was written if there was
    /// anywhere to write it.
    Seeded,
    /// Read and adopted.
    Loaded,
    /// A file exists but could not be used. The starter library is in force, **the
    /// file was left exactly as it was**, and a copy was put beside it as `.bak`.
    Rejected(String),
}

/// A reusable set of tool definitions, persisted to the config directory.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ToolLibrary {
    /// Format version — see [`LIBRARY_VERSION`].
    #[serde(default = "unversioned_is_v1")]
    pub library_version: u32,
    pub tools: Vec<Tool>,
}

impl Default for ToolLibrary {
    /// An **empty** library at the current version — not the starter set. Use
    /// [`ToolLibrary::defaults`] for that; the two are different things and conflating
    /// them is how a test ends up asserting against the shipped catalogue by accident.
    fn default() -> Self {
        Self {
            library_version: LIBRARY_VERSION,
            tools: Vec::new(),
        }
    }
}

impl ToolLibrary {
    /// The starter library seeded on first run.
    ///
    /// **Catalogue-grounded, not invented.** A fresh install previously held three end
    /// mills, which left five of the eight operations — drill, thread, chamfer, engrave,
    /// carve — impossible to start without hand-building a tool first, since the creation
    /// wizard bounds its families by what an operation can actually cut. These are
    /// starter tools that an operator is expected to edit; the point of grounding them in
    /// real catalogue geometry is that the *proportions* are right, so the guards, the
    /// cross-section preview and the depth limits all behave as they would on real steel.
    ///
    /// Sources (2026-07-20):
    /// - **Drills** — DIN 338 jobber, 118° point, parallel shank. Flute/overall lengths
    ///   cross-checked between the Würth DIN 338 datasheet and the Dormer/Farnell twist
    ///   drill catalogue, which agree exactly on every overlapping row.
    /// - **End / ball / bull nose** — DIN 6527 bodies (Hepyc 3172 via Coussement); the
    ///   ⌀6 length of cut is corroborated by DATRON. Ball and bull nose reuse the same
    ///   bodies, which is how the catalogues supply them.
    /// - **V-bits** — Amana Tool router-bit catalogue. Note these are **two families**,
    ///   not one: 30°/45° are fine *engraving* bits whose body equals the ¼″ shank, while
    ///   60°/90° are *V-groove* bits with a ½″ body on a ¼″ shank. That distinction is
    ///   load-bearing — `diameter` is what the cone flares to, so it sets
    ///   `vtip_max_depth`. Each tool's depth limit here reproduces the catalogue's own
    ///   cutting-height column to within the 0.1 mm tip radius.
    /// - **Thread mills** — full-profile M5/M6/M8 (Harvey Tool metric); the single-point
    ///   is dimensioned to Andreas's spec of 2 mm maximum thread depth, which the model
    ///   reads as `(diameter − neck) / 2`.
    /// - **Face mill** — a ⌀50 indexable, copied from Andreas's own library.
    ///
    /// Chamfer cone lengths are derived from the angle and tip flat rather than quoted.
    pub fn defaults() -> Self {
        let mut n = 0;
        let mut number = || {
            n += 1;
            n
        };
        // Milling bodies: (cutting ⌀, length of cut, overall, shank ⌀).
        let body = |d: f64| -> (f64, f64, f64) {
            match d as u32 {
                3 => (8.0, 57.0, 6.0),
                4 => (11.0, 57.0, 6.0),
                5 => (13.0, 57.0, 6.0),
                6 => (13.0, 57.0, 6.0),
                8 => (19.0, 63.0, 8.0),
                10 => (22.0, 72.0, 10.0),
                _ => (26.0, 83.0, 12.0),
            }
        };
        let mill = |number: u32, d: f64, flutes: u32, kind: ToolKind| {
            let (flute_length, length, shank_diameter) = body(d);
            Tool {
                number,
                diameter: d,
                flute_length,
                length,
                shank_diameter,
                flutes,
                kind,
                ..Default::default()
            }
        };
        // A pointed tool's cone flares to its own diameter (`Tool::profile` ignores
        // `flute_length` and the shank for these), so `diameter` is the cutting ⌀ and it
        // alone bounds how deep the tool may go.
        let pointed = |number: u32, d: f64, length: f64, flutes: u32, kind: ToolKind| Tool {
            number,
            diameter: d,
            length,
            flutes,
            kind,
            ..Default::default()
        };

        let mut tools = Vec::new();
        for d in [3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0] {
            tools.push(mill(number(), d, 3, ToolKind::EndMill));
        }
        for d in [5.0, 6.0, 8.0, 10.0] {
            tools.push(mill(number(), d, 2, ToolKind::BallMill));
        }
        for (d, corner_radius) in [(6.0, 0.5), (8.0, 1.0), (10.0, 1.5), (12.0, 2.0)] {
            tools.push(mill(number(), d, 4, ToolKind::BullNose { corner_radius }));
        }
        // A ⌀50 indexable face mill, copied from Andreas's own library — the shape of a
        // real one, so `Face` seeds something that actually faces rather than reaching
        // for the widest end mill. Its "flutes" are inserts and its "shank" the arbor.
        tools.push(Tool {
            number: number(),
            diameter: 50.0,
            flute_length: 25.0,
            length: 60.0,
            shank_diameter: 16.0,
            flutes: 4,
            kind: ToolKind::FaceMill,
            ..Default::default()
        });
        // 90° chamfer mills, ¼″ and ½″.
        for (d, length, tip_diameter) in [(6.35, 50.0, 0.2), (12.7, 63.0, 0.5)] {
            tools.push(pointed(
                number(),
                d,
                length,
                4,
                ToolKind::ChamferMill {
                    included_angle_deg: 90.0,
                    tip_diameter,
                },
            ));
        }
        // DIN 338 jobber drills, ⌀1–10. Shank = ⌀ (parallel shank), so it is left
        // unspecified and resolves to the diameter.
        for (d, flute_length, length) in [
            (1.0, 12.0, 34.0),
            (2.0, 24.0, 49.0),
            (3.0, 33.0, 61.0),
            (4.0, 43.0, 75.0),
            (5.0, 52.0, 86.0),
            (6.0, 57.0, 93.0),
            (7.0, 69.0, 109.0),
            (8.0, 75.0, 117.0),
            (9.0, 81.0, 125.0),
            (10.0, 87.0, 133.0),
        ] {
            tools.push(Tool {
                number: number(),
                diameter: d,
                flute_length,
                length,
                flutes: 2,
                kind: ToolKind::Drill {
                    point_angle_deg: 118.0,
                },
                ..Default::default()
            });
        }
        // V-bits. 30°/45° engraving (body = ¼″ shank); 60°/90° V-groove (½″ body).
        for (included_angle_deg, d, length, flutes) in [
            (30.0, 6.35, 50.8, 1),
            (45.0, 6.35, 57.15, 2),
            (60.0, 12.7, 44.45, 2),
            (90.0, 12.7, 41.27, 2),
        ] {
            tools.push(pointed(
                number(),
                d,
                length,
                flutes,
                ToolKind::VBit {
                    included_angle_deg,
                    tip_radius: 0.1,
                },
            ));
        }
        // Full-profile thread mills, then a single-point one good for 2 mm of thread
        // depth — `(diameter − neck) / 2`, which is what the reach gate reads.
        for (d, flute_length, length, shank_diameter, pitch) in [
            (3.5, 10.4, 45.0, 4.0, 0.8),
            (3.9, 12.0, 45.0, 4.0, 1.0),
            (5.8, 16.25, 57.0, 6.0, 1.25),
        ] {
            tools.push(Tool {
                number: number(),
                diameter: d,
                flute_length,
                length,
                shank_diameter,
                flutes: 3,
                kind: ToolKind::ThreadMill { pitch: Some(pitch) },
                ..Default::default()
            });
        }
        tools.push(Tool {
            number: number(),
            diameter: 10.0,
            flute_length: 20.0,
            length: 60.0,
            shank_diameter: 10.0,
            neck_diameter: 6.0,
            neck_length: 20.0,
            flutes: 3,
            kind: ToolKind::ThreadMill { pitch: None },
            ..Default::default()
        });
        Self {
            library_version: LIBRARY_VERSION,
            tools,
        }
    }

    /// Load the library from the config directory.
    ///
    /// See [`load_from`](Self::load_from) — the failure contract is the point.
    pub fn load() -> (Self, LibraryLoad) {
        match library_path() {
            Some(path) => Self::load_from(&path),
            // Nowhere to persist on this machine: the starter set, and no file.
            None => (Self::defaults(), LibraryLoad::Seeded),
        }
    }

    /// Load the library from `path`.
    ///
    /// **A file we could not read is never overwritten.** This function used to fall
    /// back to the starter set *and immediately save it*, so a parse failure — a
    /// future format change, a truncated write, a bad hand-edit — silently replaced a
    /// real tool library with the stock 36, with no error and no way back. A tool
    /// library is hand-built over time; it is some of the most expensive data the app
    /// holds.
    ///
    /// So now: missing seeds and saves (that is first-run, and correct), but
    /// unreadable, unparseable or newer-than-us leaves the file **exactly** as it is
    /// and puts a copy beside it as `.bak`. The user meets the starter set with an
    /// explanation, and their own file is still on disk.
    ///
    /// The residual case, which no load contract can prevent: once the user
    /// *deliberately* edits a tool, saving writes over that file. Hence the `.bak`
    /// copy and the message — the recovery has to exist before they touch anything.
    pub fn load_from(path: &Path) -> (Self, LibraryLoad) {
        let reject = |why: String| {
            // Preserve what we could not read, before anything can overwrite it.
            let bak = path.with_extension("json.bak");
            let _ = std::fs::copy(path, &bak);
            (Self::defaults(), LibraryLoad::Rejected(why))
        };
        let text = match std::fs::read_to_string(path) {
            Ok(t) => t,
            // Missing is first run, not a failure: seed and persist.
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                let lib = Self::defaults();
                let _ = lib.save_to(path);
                return (lib, LibraryLoad::Seeded);
            }
            Err(e) => return reject(format!("could not be read ({e})")),
        };
        let lib: ToolLibrary = match serde_json::from_str(&text) {
            Ok(l) => l,
            Err(e) => return reject(format!("is not a valid tool library ({e})")),
        };
        if lib.library_version > LIBRARY_VERSION {
            return reject(format!(
                "was written by a newer version (format {}, this build understands \
                 {LIBRARY_VERSION})",
                lib.library_version
            ));
        }
        (lib, LibraryLoad::Loaded)
    }

    /// Persist the library to the config directory (best-effort; errors are ignored
    /// — a read-only config dir simply means changes don't persist across runs).
    pub fn save(&self) {
        if let Some(path) = library_path() {
            let _ = self.save_to(&path);
        }
    }

    /// Persist to `path`, creating the directory if needed.
    pub fn save_to(&self, path: &Path) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let text = serde_json::to_string_pretty(self)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        std::fs::write(path, text)
    }

    /// The library tool to seed a newly-created operation of `kind` with — a
    /// sensible starting default the user can still override in the wizard picker.
    ///
    /// Every op has a **natural tool kind** and defaults to the median tool of that
    /// kind by diameter: Profile/Pocket → `EndMill`, Drill → `Drill`, Thread →
    /// `ThreadMill`, Chamfer → `ChamferMill`. **Face** is the one that instead takes
    /// the largest: it wants a flat-bottomed tool (a chamfer or ball tool would leave
    /// a scalloped floor), and the *largest* such tool means the fewest passes.
    /// **Engrave/Carve** are the ones that do not sort by size at all — a V-bit is
    /// chosen by its included angle (see below).
    ///
    /// In every case this is only a **default** — we never *reject* a tool the user
    /// picks; and if no tool of the preferred kind exists we fall back to the first
    /// library tool (order = the user's own numbering). `None` only when the
    /// library is empty.
    pub fn default_tool_for(&self, kind: OpKind) -> Option<Tool> {
        // The middle tool of its kind, by diameter — not the first.
        //
        // "First" meant the *smallest*, because a library is naturally kept in size
        // order: a new user's first click on Drill seeded the ⌀1, the most fragile drill
        // in the set, and Profile seeded the ⌀3. A median is only a seed, but a seed
        // should be the tool most jobs would actually start from.
        //
        // Median rather than a target size (say "6 mm"), because this runs against the
        // *operator's* library, which may hold three tools or three hundred, in any
        // sizes. A median degrades gracefully; a magic number does not. The lower middle
        // is taken, so an even count leans small — the cheaper mistake, since an
        // oversized cutter may not fit the feature at all.
        let middle = |pred: fn(&ToolKind) -> bool| -> Option<&Tool> {
            let mut of_kind: Vec<&Tool> = self.tools.iter().filter(|t| pred(&t.kind)).collect();
            if of_kind.is_empty() {
                return None;
            }
            of_kind.sort_by(|a, b| a.diameter.total_cmp(&b.diameter));
            Some(of_kind[(of_kind.len() - 1) / 2])
        };
        // V-bits are chosen by **included angle**, never by diameter. A V-bit's body
        // diameter says where its cone flares out — a reach limit — while the angle is
        // what shapes every cut it makes, so sorting by size ranks them on the wrong
        // property. On the starter set (30/45/60/90) the median-by-diameter landed on
        // the 45°, an engraving bit; the Amana catalogue calls the **60°** "the general
        // go-to bit", and it is the one both engraving and carving want to start from.
        //
        // A target angle here, deliberately unlike the median used for diameters above,
        // because the two quantities behave differently. Diameters span whatever the
        // shop happens to own, so a median degrades gracefully and a magic number does
        // not. Angles do the opposite: they cluster on a small standard set with a
        // conventional default, so "nearest 60°" holds against a library of three bits
        // or three hundred, while a median would just track whichever half of the set
        // is better stocked.
        //
        // Ties break to the **sharper** bit — the cheaper mistake, matching the lower
        // middle taken above. A narrow groove can always be cut deeper to widen it; a
        // blunt bit can never be made to cut a narrow one.
        const GO_TO_VBIT_ANGLE_DEG: f64 = 60.0;
        let nearest_vbit = || -> Option<&Tool> {
            self.tools
                .iter()
                .filter_map(|t| match t.kind {
                    ToolKind::VBit {
                        included_angle_deg, ..
                    } => Some((t, included_angle_deg)),
                    _ => None,
                })
                .min_by(|(_, a), (_, b)| {
                    let da = (a - GO_TO_VBIT_ANGLE_DEG).abs();
                    let db = (b - GO_TO_VBIT_ANGLE_DEG).abs();
                    da.total_cmp(&db).then(a.total_cmp(b))
                })
                .map(|(t, _)| t)
        };
        let preferred = match kind {
            // Facing is the exception, and deliberately so: it wants the *largest* flat
            // tool (fewest passes), which is scallop-safe and what a facing pass is for.
            OpKind::Face => self
                .tools
                .iter()
                .filter(|t| matches!(t.kind, ToolKind::EndMill | ToolKind::FaceMill))
                .max_by(|a, b| a.diameter.total_cmp(&b.diameter)),
            OpKind::Profile | OpKind::Pocket => middle(|k| matches!(k, ToolKind::EndMill)),
            OpKind::Drill => middle(|k| matches!(k, ToolKind::Drill { .. })),
            OpKind::Thread => middle(|k| matches!(k, ToolKind::ThreadMill { .. })),
            OpKind::Chamfer => middle(|k| matches!(k, ToolKind::ChamferMill { .. })),
            // Engraving *requires* a V-bit (a chamfer mill's tip does not cut), so
            // this is the one default that matches the strategy's hard gate.
            // Carving *requires* a V-bit too — it is the same gate, for the same
            // reason: the tool's own cone is what shapes the cut. And for the same
            // reason it is chosen by angle, not by size.
            OpKind::Engrave | OpKind::Carve => nearest_vbit(),
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
/// The platform convention lives in [`crate::paths`], shared with the settings file.
fn library_path() -> Option<PathBuf> {
    crate::paths::config_file("tools.json")
}

/// The tool-geometry class as a plain discriminant, for the inspector picker
/// (a friendlier face on the data-carrying [`ToolKind`], mirroring `PlungeKind`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToolKindPick {
    EndMill,
    BallMill,
    BullNose,
    ChamferMill,
    Drill,
    FaceMill,
    ThreadMill,
    VBit,
}

impl ToolKindPick {
    pub const ALL: [ToolKindPick; 8] = [
        ToolKindPick::EndMill,
        ToolKindPick::BallMill,
        ToolKindPick::BullNose,
        ToolKindPick::ChamferMill,
        ToolKindPick::Drill,
        ToolKindPick::FaceMill,
        ToolKindPick::ThreadMill,
        ToolKindPick::VBit,
    ];

    pub fn of(kind: ToolKind) -> Self {
        match kind {
            ToolKind::EndMill => ToolKindPick::EndMill,
            ToolKind::BallMill => ToolKindPick::BallMill,
            ToolKind::BullNose { .. } => ToolKindPick::BullNose,
            ToolKind::ChamferMill { .. } => ToolKindPick::ChamferMill,
            ToolKind::Drill { .. } => ToolKindPick::Drill,
            ToolKind::FaceMill => ToolKindPick::FaceMill,
            ToolKind::ThreadMill { .. } => ToolKindPick::ThreadMill,
            ToolKind::VBit { .. } => ToolKindPick::VBit,
        }
    }

    /// A `ToolKind` of this class with sensible default parameters.
    pub fn to_kind(self) -> ToolKind {
        match self {
            ToolKindPick::EndMill => ToolKind::EndMill,
            ToolKindPick::BallMill => ToolKind::BallMill,
            ToolKindPick::BullNose => ToolKind::BullNose { corner_radius: 1.0 },
            // A chamfer mill is always ground with a flat tip; 0 would make it a
            // V-bit. 0.2 mm is a typical small flat.
            ToolKindPick::ChamferMill => ToolKind::ChamferMill {
                included_angle_deg: 90.0,
                tip_diameter: 0.2,
            },
            ToolKindPick::Drill => ToolKind::Drill {
                point_angle_deg: 118.0,
            },
            ToolKindPick::FaceMill => ToolKind::FaceMill,
            ToolKindPick::ThreadMill => ToolKind::ThreadMill { pitch: None },
            // A V-bit's point is always ground to a radius; 0 is unmakeable. 0.1 mm
            // is a typical carving tip.
            ToolKindPick::VBit => ToolKind::VBit {
                included_angle_deg: 60.0,
                tip_radius: 0.1,
            },
        }
    }
}

/// The tool families offered for an operation, in the order shown.
///
/// A *family* is exactly a group in the Tool Library (the eight [`ToolKind`] classes),
/// so the wizard and the library speak the same vocabulary. Bounding the list by
/// operation is what keeps a library of hundreds usable: a pocket has no business
/// listing drills or chamfer mills.
///
/// The strategy guards are the hard floor — anything here that could still not cut
/// would be refused at Run time anyway — but this list is deliberately **narrower**
/// than "whatever would not error", agreed with Andreas: no ball-nose for facing (it
/// would leave a scalloped floor), no face mill for profiling or pocketing, and no end
/// mill for drilling. Those remain *possible* if a tool is set another way; they are
/// simply not offered.
pub fn families_for(kind: OpKind) -> &'static [ToolKindPick] {
    use ToolKindPick as F;
    match kind {
        // Side-milling a vertical wall: the end-mill family only.
        OpKind::Profile | OpKind::Pocket => &[F::EndMill, F::BallMill, F::BullNose],
        // A flat floor: flat-bottomed tools (a bull-nose floor is still flat).
        OpKind::Face => &[F::EndMill, F::BullNose, F::FaceMill],
        OpKind::Drill => &[F::Drill],
        OpKind::Thread => &[F::ThreadMill],
        // A chamfer is cut by the flank, which both of these have.
        OpKind::Chamfer => &[F::ChamferMill, F::VBit],
        // Engraving cuts with the tip; a chamfer mill's flat does not cut.
        // Carving cuts with the tip too, and its geometry comes from the cone.
        OpKind::Engrave | OpKind::Carve => &[F::VBit],
    }
}

impl std::fmt::Display for ToolKindPick {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.to_kind().to_string().as_str())
    }
}

#[cfg(test)]
mod library_file_tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};

    struct Scratch(PathBuf);

    impl Scratch {
        fn new() -> Self {
            static N: AtomicU32 = AtomicU32::new(0);
            let d = std::env::temp_dir().join(format!(
                "ocam-library-{}-{}",
                std::process::id(),
                N.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir_all(&d).expect("scratch dir");
            Self(d)
        }
        fn file(&self) -> PathBuf {
            self.0.join("tools.json")
        }
        fn bak(&self) -> PathBuf {
            self.0.join("tools.json.bak")
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// **The hazard this fix exists for.** The old `load()` fell back to the starter
    /// set *and immediately saved it*, so a library that failed to parse was replaced
    /// on disk — silently, with no error and no copy. A hand-built tool library is
    /// among the most expensive data the app holds.
    #[test]
    fn an_unreadable_library_is_never_overwritten_and_is_backed_up() {
        let s = Scratch::new();
        let precious = r#"{"tools": [ this is corrupt but it is THEIRS"#;
        std::fs::write(s.file(), precious).unwrap();

        let (lib, outcome) = ToolLibrary::load_from(&s.file());
        assert!(matches!(outcome, LibraryLoad::Rejected(_)), "{outcome:?}");
        assert_eq!(lib, ToolLibrary::defaults(), "the starter set is in force");
        assert_eq!(
            std::fs::read_to_string(s.file()).unwrap(),
            precious,
            "the user's library must survive a failed load"
        );
        assert_eq!(
            std::fs::read_to_string(s.bak()).unwrap(),
            precious,
            "and a copy must exist before anything can overwrite it"
        );
    }

    #[test]
    fn a_library_from_a_newer_build_is_refused_not_read_leniently() {
        let s = Scratch::new();
        let newer = format!(r#"{{"library_version": {}, "tools": []}}"#, LIBRARY_VERSION + 3);
        std::fs::write(s.file(), &newer).unwrap();

        let (lib, outcome) = ToolLibrary::load_from(&s.file());
        assert!(matches!(outcome, LibraryLoad::Rejected(_)), "{outcome:?}");
        assert_eq!(lib, ToolLibrary::defaults());
        assert_eq!(std::fs::read_to_string(s.file()).unwrap(), newer);
        // An empty `tools` list read leniently would have looked like "you have no
        // tools" — indistinguishable from a real empty library.
    }

    #[test]
    fn a_missing_library_seeds_the_starter_set_and_persists_it() {
        let s = Scratch::new();
        let (lib, outcome) = ToolLibrary::load_from(&s.file());
        assert_eq!(outcome, LibraryLoad::Seeded);
        assert_eq!(lib, ToolLibrary::defaults());
        assert!(s.file().exists(), "first run must seed the library");
        assert!(!s.bak().exists(), "a first run has nothing to back up");

        // And it reloads as itself.
        let (again, outcome) = ToolLibrary::load_from(&s.file());
        assert_eq!(outcome, LibraryLoad::Loaded);
        assert_eq!(again, lib);
    }

    /// Every library ever written predates the version field, and every one of them is
    /// the v1 shape. Reading them as v1 is the truth; refusing them would be a
    /// self-inflicted data loss on upgrade.
    #[test]
    fn a_library_written_before_the_version_field_loads_as_v1() {
        let s = Scratch::new();
        let old = ToolLibrary::defaults();
        let mut value = serde_json::to_value(&old).unwrap();
        value.as_object_mut().unwrap().remove("library_version");
        assert!(value.get("library_version").is_none());
        std::fs::write(s.file(), serde_json::to_string_pretty(&value).unwrap()).unwrap();

        let (lib, outcome) = ToolLibrary::load_from(&s.file());
        assert_eq!(outcome, LibraryLoad::Loaded);
        assert_eq!(lib.library_version, 1);
        assert_eq!(lib.tools, old.tools, "the tools must survive untouched");
        assert!(!s.bak().exists(), "an unversioned file is valid, not rejected");
    }

    #[test]
    fn a_saved_library_round_trips_with_its_version() {
        let s = Scratch::new();
        let lib = ToolLibrary::defaults();
        lib.save_to(&s.file()).unwrap();
        let text = std::fs::read_to_string(s.file()).unwrap();
        assert!(text.contains("library_version"), "the version must be written");
        let (back, outcome) = ToolLibrary::load_from(&s.file());
        assert_eq!(outcome, LibraryLoad::Loaded);
        assert_eq!(back, lib);
    }

    /// `Default` is an *empty* library, not the starter set. They are different things
    /// and the distinction is load-bearing: a test that took `Default` for the starter
    /// set would assert against whatever the catalogue happens to contain.
    #[test]
    fn default_is_empty_and_defaults_is_the_catalogue() {
        assert!(ToolLibrary::default().tools.is_empty());
        assert!(!ToolLibrary::defaults().tools.is_empty());
        assert_eq!(ToolLibrary::default().library_version, LIBRARY_VERSION);
    }
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
    fn every_operation_is_usable_out_of_the_box() {
        // The reason the starter library grew. A fresh install used to hold three end
        // mills, so drill, thread, chamfer, engrave and carve all opened the creation
        // wizard onto an empty family picker -- five of the eight operations unusable
        // until the operator hand-built a tool, including the two the README advertises
        // most. `families_for` bounds the wizard by what an operation can actually cut,
        // so "usable" means the library holds a tool of one of those families.
        let lib = ToolLibrary::defaults();
        for op in [
            OpKind::Face,
            OpKind::Profile,
            OpKind::Pocket,
            OpKind::Drill,
            OpKind::Thread,
            OpKind::Chamfer,
            OpKind::Engrave,
            OpKind::Carve,
        ] {
            let tool = lib.default_tool_for(op).expect("the library is not empty");
            let families = families_for(op);
            assert!(
                families
                    .iter()
                    .any(|f| std::mem::discriminant(&f.to_kind()) == std::mem::discriminant(&tool.kind)),
                "{op:?} seeds {} ({}), which is not one of the families it can cut: {families:?}",
                tool.number,
                tool.kind
            );
        }
    }

    #[test]
    fn the_starter_v_bits_reach_their_catalogue_cutting_heights() {
        // A V-bit's `diameter` is what its cone flares to, so it alone bounds how deep
        // Engrave and Carve will go. These four are Amana router bits, and each must
        // still reach the depth its catalogue quotes as the cutting height -- within the
        // 0.1 mm tip radius, which the catalogue's sharp-point figure does not allow for.
        let lib = ToolLibrary::defaults();
        for (angle, catalogue_ch) in [(30.0, 11.67), (45.0, 7.52), (60.0, 11.11), (90.0, 6.35)] {
            let t = lib
                .tools
                .iter()
                .find(|t| matches!(t.kind, ToolKind::VBit { included_angle_deg, .. }
                                   if (included_angle_deg - angle).abs() < 1e-9))
                .unwrap_or_else(|| panic!("no {angle}° V-bit in the starter library"));
            let ToolKind::VBit { tip_radius, .. } = t.kind else {
                unreachable!()
            };
            let depth = cam_geo::vtip_max_depth(
                (angle * 0.5_f64).to_radians(),
                tip_radius,
                t.radius(),
            );
            assert!(
                (depth - catalogue_ch).abs() < 0.25,
                "{angle}°: reaches {depth:.2} mm, catalogue cutting height {catalogue_ch:.2} mm"
            );
        }
    }

    #[test]
    fn the_single_point_thread_mill_reaches_two_millimetres_of_thread() {
        let lib = ToolLibrary::defaults();
        let t = lib
            .tools
            .iter()
            .find(|t| matches!(t.kind, ToolKind::ThreadMill { pitch: None }))
            .expect("a single-point thread mill");
        // The reach gate reads max thread depth as (cutting ⌀ − neck ⌀) / 2.
        assert!(((t.diameter - t.neck_dia()) / 2.0 - 2.0).abs() < 1e-9);
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
        // Facing wants the widest flat tool (fewest passes). Built here rather than
        // taken from `defaults()`: what the starter library happens to contain is not
        // what this rule is about, and coupling the two made the test fail the moment
        // the library grew.
        let lib = ToolLibrary {
            tools: vec![
                mk(1, 6.0, ToolKind::EndMill),
                mk(2, 12.0, ToolKind::EndMill),
                mk(3, 8.0, ToolKind::EndMill),
            ],
                      ..Default::default()
                  };
        let t = lib.default_tool_for(OpKind::Face).unwrap();
        assert_eq!(t.diameter, 12.0);
        assert!(matches!(t.kind, ToolKind::EndMill));
        // Face is the exception. Every other kinded op takes the *middle* tool by
        // diameter, so here ⌀8 rather than the widest.
        assert_eq!(lib.default_tool_for(OpKind::Profile).unwrap().diameter, 8.0);
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
                      ..Default::default()
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
                               ..Default::default()
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
                    tip_diameter: 0.2,
                }),
                mk(5, 12.0, ToolKind::ThreadMill { pitch: None }),
            ],
                      ..Default::default()
                  };
        assert_eq!(lib.default_tool_for(OpKind::Drill).unwrap().number, 2, "first Drill");
        assert_eq!(lib.default_tool_for(OpKind::Chamfer).unwrap().number, 4, "first ChamferMill");
        assert_eq!(lib.default_tool_for(OpKind::Thread).unwrap().number, 5, "first ThreadMill");
        // Profile/Pocket want the first end mill (tool 1 here).
        assert_eq!(lib.default_tool_for(OpKind::Profile).unwrap().number, 1);
        assert_eq!(lib.default_tool_for(OpKind::Pocket).unwrap().number, 1);
    }

    #[test]
    fn profile_and_pocket_ignore_a_leading_drill() {
        // A drill numbered first must not become the profile/pocket default, whatever
        // the size rule is. Of the two end mills, the lower middle of {4, 8} is ⌀4.
        let lib = ToolLibrary {
            tools: vec![
                mk(1, 6.0, ToolKind::Drill { point_angle_deg: 118.0 }),
                mk(2, 8.0, ToolKind::EndMill),
                mk(3, 4.0, ToolKind::EndMill),
            ],
                      ..Default::default()
                  };
        for op in [OpKind::Profile, OpKind::Pocket] {
            let t = lib.default_tool_for(op).unwrap();
            assert!(matches!(t.kind, ToolKind::EndMill), "{op:?} took the drill");
            assert_eq!(t.diameter, 4.0);
        }
    }

    #[test]
    fn kinded_ops_seed_the_middle_size_not_the_smallest() {
        // The change this rule exists for: a library is naturally kept in size order, so
        // "first of its kind" meant *smallest* -- a new user's first click on Drill
        // seeded the ⌀1, the most fragile drill in the set.
        let lib = ToolLibrary::defaults();
        let dia = |op| lib.default_tool_for(op).unwrap().diameter;
        assert_eq!(dia(OpKind::Profile), 6.0, "end mills 3..12 -> ⌀6");
        assert_eq!(dia(OpKind::Pocket), 6.0);
        assert_eq!(dia(OpKind::Drill), 5.0, "drills 1..10 -> ⌀5");
        assert_eq!(dia(OpKind::Chamfer), 6.35, "of 6.35/12.7 the lower middle");
        // Facing keeps its own rule -- widest flat tool, which is the face mill.
        assert_eq!(dia(OpKind::Face), 50.0);
        // V-bits are the exception to the size rule entirely -- see
        // `v_bits_are_seeded_by_angle_not_by_size`.
    }

    #[test]
    fn v_bits_are_seeded_by_angle_not_by_size() {
        // A V-bit's diameter is where its cone flares out -- a reach limit -- while the
        // angle is what shapes the cut, so ranking them by size ranks the wrong
        // property. On the starter set the median-by-diameter landed on the 45°
        // engraving bit; the catalogue go-to is the 60°.
        let lib = ToolLibrary::defaults();
        for op in [OpKind::Engrave, OpKind::Carve] {
            let v = lib.default_tool_for(op).unwrap();
            let ToolKind::VBit { included_angle_deg, .. } = v.kind else {
                panic!("{op:?} must seed a V-bit")
            };
            assert_eq!(included_angle_deg, 60.0, "{op:?} seeded the wrong V-bit");
            // And it is emphatically not the median by diameter, which is the point:
            // of {6.35, 6.35, 12.7, 12.7} the lower middle is 6.35, the 45° bit.
            assert_eq!(v.diameter, 12.7);
        }
    }

    #[test]
    fn the_nearest_v_bit_angle_wins_and_ties_go_to_the_sharper() {
        // No 60° in the library: the nearest angle wins from either side.
        let bit = |n, d, a| {
            mk(n, d, ToolKind::VBit {
                included_angle_deg: a,
                tip_radius: 0.1,
            })
        };
        let below = ToolLibrary { tools: vec![bit(1, 6.0, 30.0), bit(2, 6.0, 50.0)], ..Default::default() };
        assert_eq!(below.default_tool_for(OpKind::Engrave).unwrap().number, 2, "50° is nearer 60");
        let above = ToolLibrary { tools: vec![bit(1, 6.0, 90.0), bit(2, 6.0, 70.0)], ..Default::default() };
        assert_eq!(above.default_tool_for(OpKind::Engrave).unwrap().number, 2, "70° is nearer 60");

        // Equidistant (45 and 75): the sharper bit wins. A narrow groove can be cut
        // deeper to widen it; a blunt bit can never cut a narrow one.
        let tie = ToolLibrary { tools: vec![bit(1, 6.0, 75.0), bit(2, 6.0, 45.0)], ..Default::default() };
        let t = tie.default_tool_for(OpKind::Engrave).unwrap();
        assert_eq!(t.number, 2, "a tie must break to the sharper bit");

        // A library with no V-bit at all still yields something rather than nothing --
        // the operator can change it, an empty wizard cannot be worked around.
        let none = ToolLibrary { tools: vec![mk(1, 6.0, ToolKind::EndMill)], ..Default::default() };
        assert_eq!(none.default_tool_for(OpKind::Engrave).unwrap().number, 1);
    }

    #[test]
    fn a_single_tool_of_a_kind_is_still_chosen() {
        // The median must not misbehave at the edges: one tool, or two.
        let one = ToolLibrary { tools: vec![mk(1, 6.0, ToolKind::EndMill)], ..Default::default() };
        assert_eq!(one.default_tool_for(OpKind::Profile).unwrap().diameter, 6.0);
        let two = ToolLibrary {
            tools: vec![mk(1, 10.0, ToolKind::EndMill), mk(2, 4.0, ToolKind::EndMill)],
                      ..Default::default()
                  };
        assert_eq!(
            two.default_tool_for(OpKind::Profile).unwrap().diameter,
            4.0,
            "an even count leans small -- an oversized cutter may not fit at all"
        );
    }

    #[test]
    fn kinded_ops_fall_back_to_first_when_no_matching_kind() {
        // A library of only end mills: every kinded op still gets a (valid) tool. The
        // shipped library now *has* drills, V-bits and thread mills, so this rule has to
        // be exercised on a library that deliberately lacks them.
        let lib = ToolLibrary {
            tools: vec![mk(1, 6.0, ToolKind::EndMill), mk(2, 10.0, ToolKind::EndMill)],
                      ..Default::default()
                  };
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
        // 1/2/3 with #2 deleted, so there is a gap. Built locally: the rule is about
        // gaps, not about how many tools ship.
        let mut lib = ToolLibrary {
            tools: vec![
                mk(1, 3.0, ToolKind::EndMill),
                mk(2, 6.0, ToolKind::EndMill),
                mk(3, 10.0, ToolKind::EndMill),
            ],
                          ..Default::default()
                      };
        lib.tools.retain(|t| t.number != 2);
        let idx = lib.add_default();
        assert_eq!(lib.tools[idx].number, 2, "the freed #2 is reused, not #4");
        // Now 1/2/3 again → next is 4.
        lib.add_default();
        assert_eq!(lib.tools.last().unwrap().number, 4);
    }

    #[test]
    fn set_number_swaps_on_collision() {
        let mut lib = ToolLibrary {
            tools: vec![
                mk(1, 3.0, ToolKind::EndMill),
                mk(2, 6.0, ToolKind::EndMill),
                mk(3, 10.0, ToolKind::EndMill),
            ],
                          ..Default::default()
                      };
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
        let mut lib = ToolLibrary {
            tools: vec![
                mk(1, 3.0, ToolKind::EndMill),
                mk(2, 6.0, ToolKind::EndMill),
                mk(3, 10.0, ToolKind::EndMill),
            ],
                          ..Default::default()
                      };
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
    #[test]
    fn engrave_defaults_to_a_v_bit_not_a_chamfer_mill() {
        // The strategy hard-rejects a chamfer mill, so the default must not hand the
        // user an operation that cannot possibly run.
        let lib = ToolLibrary {
            tools: vec![
                mk(1, 6.0, ToolKind::ChamferMill {
                    included_angle_deg: 90.0,
                    tip_diameter: 0.2,
                }),
                mk(2, 6.0, ToolKind::VBit {
                    included_angle_deg: 60.0,
                    tip_radius: 0.2,
                }),
            ],
                      ..Default::default()
                  };
        assert_eq!(
            lib.default_tool_for(OpKind::Engrave).unwrap().number,
            2,
            "must pick the V-bit over the chamfer mill"
        );
        // Chamfering still prefers the chamfer mill.
        assert_eq!(lib.default_tool_for(OpKind::Chamfer).unwrap().number, 1);
    }

}
