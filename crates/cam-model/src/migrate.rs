//! Version-driven migration of a saved [`Document`](crate::Document).
//!
//! ## Why this exists
//!
//! Every schema bump from v2 to v9 was **additive**: a new field with
//! `#[serde(default)]`, which an older file simply lacks. Serde handled all of them, so
//! `schema_version` was written into every save-file and *never read by anything*. It
//! was a record, not a mechanism.
//!
//! v10 is the first bump that changes the **shape** of data already on disk:
//! [`PocketOp`](crate::PocketOp)'s flat clearing fields moved into a nested
//! [`ClearParams`](crate::ClearParams), so `{"stepdown": 2.0}` has to become
//! `{"clear": {"stepdown": 2.0}}`. No serde attribute expresses that. The version field
//! becomes load-bearing here for the first time.
//!
//! ## Why a JSON pass rather than a lenient `Deserialize`
//!
//! A hand-written `Deserialize` for `PocketOp` accepting both layouts would also work,
//! and would need no version at all. It was rejected for two reasons. It does not
//! compose — every future reshaping adds another bespoke deserializer, each one
//! independently able to be wrong. And it cannot express a **semantic** change at all:
//! a field that keeps its name and type but changes meaning (a unit, a sign convention,
//! a reference plane) is invisible to serde, because both versions parse. This project
//! has already made one such change to depth, and will make more.
//!
//! A migration keyed on the version can express both, and — the real payoff — each step
//! is a small pure function over a JSON tree that can be tested against a captured file
//! from the version it claims to read.
//!
//! ## Shape
//!
//! [`document`] runs the steps in order, `from` → `from+1` → … → [`SCHEMA_VERSION`], and
//! stamps the result. Steps are **not** allowed to look at [`SCHEMA_VERSION`] or at any
//! Rust struct: a step reads and writes the JSON of *its own two versions*, which are
//! frozen history. A step that deserializes into today's structs would silently change
//! meaning the next time those structs change, which is the failure mode this module
//! exists to prevent.

use serde_json::{Map, Value};

use crate::SCHEMA_VERSION;

/// The oldest schema version this build can still open.
///
/// **v3**, because v2→v3 replaced `Stock::Box { min, max }` — an absolute pair of corners
/// — with the part-relative `Stock::BoundingBox { x_offset, y_offset, top, thickness }`,
/// and that change shipped with the decision *"no migration — early stage"* on the
/// record. No released build has ever opened a v1 or v2 file; the schema was already
/// past v3 when v0.1.0 shipped.
///
/// This constant briefly said `1`, which was simply false: a v1 document reaching serde
/// failed on `unknown variant 'Box', expected 'BoundingBox'` — a parse error blaming the
/// file's contents for what is really an unsupported version. Found by opening a real
/// v1 project, not by the fixtures, which is the lesson: `every_supported_version_has_a_step`
/// only proves a step *exists*, never that an identity step is *honest*.
///
/// Converting a v2 stock faithfully is possible but not from here — the offsets are
/// measured from the part's XY bounds, which live in the project's `regions`, outside the
/// document this module migrates. It would need the whole project, and is worth doing
/// only if a v1/v2 file ever turns out to matter.
///
/// Kept as a named constant rather than a literal so that retiring old versions is a
/// deliberate, greppable edit with a note about which files stop opening, not a quiet
/// change to a loop bound.
pub const OLDEST_SUPPORTED: u32 = 3;

/// Why a saved document could not be brought up to the current schema.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MigrationError {
    /// Written by a newer build than this one. There is no forward migration: the file
    /// may contain operations and fields this build has never heard of, and opening it
    /// would silently discard them on the next save.
    FromTheFuture {
        /// The version stamped in the file.
        file: u32,
        /// The version this build writes.
        current: u32,
    },
    /// Older than [`OLDEST_SUPPORTED`] — the steps that would read it have been retired.
    TooOld {
        /// The version stamped in the file.
        file: u32,
        /// The oldest version this build still migrates.
        oldest: u32,
    },
    /// The file is stamped with a version this build knows, but its contents are not
    /// what that version's format allows.
    Malformed {
        /// The version being migrated *from* when the problem was found.
        step: u32,
        /// What was wrong, in terms a user could act on.
        detail: String,
    },
    /// No migration step is defined for a version inside the supported range — a bug in
    /// this module (`SCHEMA_VERSION` bumped without adding a step), not a bad file.
    /// Pinned by `every_supported_version_has_a_step`.
    MissingStep {
        /// The version with no step out of it.
        from: u32,
    },
}

impl std::fmt::Display for MigrationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MigrationError::FromTheFuture { file, current } => write!(
                f,
                "this project was saved by a newer version of OpenCAMStudio \
                 (file schema v{file}, this build writes v{current}) — upgrade to open it"
            ),
            MigrationError::TooOld { file, oldest } => write!(
                f,
                "this project uses schema v{file}, which this build no longer opens \
                 (oldest supported is v{oldest})"
            ),
            MigrationError::Malformed { step, detail } => {
                write!(f, "damaged project file (at schema v{step}): {detail}")
            }
            MigrationError::MissingStep { from } => write!(
                f,
                "internal error: no migration step from schema v{from} \
                 (please report this)"
            ),
        }
    }
}

impl std::error::Error for MigrationError {}

/// Bring a serialized [`Document`](crate::Document) up to [`SCHEMA_VERSION`], in place.
///
/// `value` is the document's own JSON object — not the enclosing save-file — and `from`
/// is the version it was written at. On success `value` deserializes into today's
/// `Document` and carries the current version stamp.
///
/// A document already at the current version is not a special case: the loop simply
/// runs zero steps. That keeps the common path on the same code as the migrating one,
/// so a step that corrupts a document cannot hide behind an "already current" shortcut.
pub fn document(value: &mut Value, from: u32) -> Result<(), MigrationError> {
    if from > SCHEMA_VERSION {
        return Err(MigrationError::FromTheFuture {
            file: from,
            current: SCHEMA_VERSION,
        });
    }
    if from < OLDEST_SUPPORTED {
        return Err(MigrationError::TooOld {
            file: from,
            oldest: OLDEST_SUPPORTED,
        });
    }

    for version in from..SCHEMA_VERSION {
        step(version, value)?;
    }

    // Stamp last, and only on success: a document that failed part-way through keeps
    // the version it actually matches, so a partial migration cannot be mistaken for a
    // complete one by anything that re-reads the tree.
    if let Some(obj) = value.as_object_mut() {
        obj.insert(
            "schema_version".to_string(),
            Value::from(u64::from(SCHEMA_VERSION)),
        );
    }
    Ok(())
}

/// Apply the single step that takes a document from `from` to `from + 1`.
fn step(from: u32, doc: &mut Value) -> Result<(), MigrationError> {
    match from {
        // v3→v9 were all additive: a new field carrying `#[serde(default)]`, which an
        // older file lacks and serde fills in. There is nothing to rewrite — the old
        // JSON *is* valid new JSON. One is worth naming because it looks like it should
        // need work and does not: **v9** *removed* the per-origin program start point,
        // and serde ignores unknown fields, so the stale value is dropped on read, which
        // is exactly the intent.
        //
        // The range starts at 3 rather than 1 because v2→v3 was **not** additive — it
        // replaced `Stock::Box` with `Stock::BoundingBox` and shipped without a
        // migration. See [`OLDEST_SUPPORTED`]; those versions are refused before any
        // step runs.
        //
        // Listed as an explicit range rather than a catch-all so that adding v12 without
        // a step is a `MissingStep` error, not a silent no-op.
        3..=8 => Ok(()),
        9 => v9_to_v10(doc),
        // v10→v11 added the machine and the post to the *project wrapper*, not to the
        // document — additive, `Option`, absent in every earlier file. Nothing in the
        // document moved, so there is nothing here to move. The version still bumped
        // because it numbers the save-file as a whole, and a format change that leaves
        // the version alone is the one that bites later.
        10 => Ok(()),
        _ => Err(MigrationError::MissingStep { from }),
    }
}

/// The clearing parameters that moved off `PocketOp` and into its nested `clear` object
/// at v10 — the field names as they appear in a **v9** file, frozen.
///
/// Deliberately a literal list rather than anything derived from `ClearParams`: this
/// step describes a historical layout, and must not start moving different fields
/// because a struct changed years later.
const V9_POCKET_CLEARING_FIELDS: [&str; 10] = [
    "stepdown",
    "overlap",
    "offset",
    "feed",
    "plunge_feed",
    "plunge",
    "lead_in",
    "lead_out",
    "lead_overlap",
    "clearing",
];

/// The subset of those that a v9 `PocketOp` had **no** serde default for, so every
/// genuine v9 file carries them.
///
/// Their absence is treated as a damaged file rather than migrated to a default,
/// because the defaults are not safe: `ClearParams` defaults each field individually
/// (`#[serde(default)]` → `f64::default()`), so a missing `overlap` would silently
/// become `0.0` — ring spacing of a full tool diameter, which leaves uncut ridges
/// between passes. Refusing to open is the honest outcome; quietly clearing a pocket
/// wrong is not.
const V9_POCKET_REQUIRED_FIELDS: [&str; 5] =
    ["stepdown", "overlap", "feed", "plunge_feed", "plunge"];

/// v9 → v10: `PocketOp`'s flat clearing fields move into a nested `clear` object, so
/// pocket and carve state one area-clearing parameter set between them.
fn v9_to_v10(doc: &mut Value) -> Result<(), MigrationError> {
    for op in operations_mut(doc, 9)? {
        // Operations are an externally tagged enum: `{"Pocket": { … }}`.
        let Some(pocket) = op.get_mut("Pocket").and_then(Value::as_object_mut) else {
            continue;
        };

        for field in V9_POCKET_REQUIRED_FIELDS {
            if !pocket.contains_key(field) {
                return Err(MigrationError::Malformed {
                    step: 9,
                    detail: format!(
                        "a pocket operation has no `{field}`, which every v9 pocket \
                         carries; refusing to substitute a default for a clearing \
                         parameter"
                    ),
                });
            }
        }

        let mut clear = Map::new();
        for field in V9_POCKET_CLEARING_FIELDS {
            if let Some(v) = pocket.remove(field) {
                clear.insert(field.to_string(), v);
            }
        }
        pocket.insert("clear".to_string(), Value::Object(clear));
    }
    Ok(())
}

/// The setup's operation list, for a step to walk. Shared because every future step
/// that touches operations needs it, and because the "damaged file" wording should be
/// written once.
fn operations_mut(doc: &mut Value, step: u32) -> Result<&mut Vec<Value>, MigrationError> {
    let malformed = |detail: &str| MigrationError::Malformed {
        step,
        detail: detail.to_string(),
    };
    doc.get_mut("setup")
        .ok_or_else(|| malformed("the document has no `setup`"))?
        .get_mut("operations")
        .ok_or_else(|| malformed("the setup has no `operations` list"))?
        .as_array_mut()
        .ok_or_else(|| malformed("`operations` is not a list"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Document;

    /// A v9 document with one pocket, written the way v9 actually wrote it — flat
    /// clearing fields, `Contour` as a point list. Hand-authored rather than generated
    /// from today's structs on purpose: a fixture built by serializing the *current*
    /// model would silently follow it forward and stop being a v9 file at all, which
    /// is precisely the mistake this module guards against.
    fn v9_document_with_a_pocket() -> Value {
        serde_json::json!({
            "schema_version": 9,
            "setup": {
                "name": "part",
                "heights": { "clearance": 50.0, "retract": 5.0, "top_of_stock": 0.0 },
                "stock": { "BoundingBox": {
                    "x_offset": 0.0, "y_offset": 0.0, "top": 0.0, "thickness": 10.0
                }},
                "origin": [0.0, 0.0, 0.0],
                "origin_index": 1,
                "extra_origins": [],
                "tools": [],
                "operations": [
                    { "Pocket": {
                        "id": 1,
                        "tool": 1,
                        "boundary": { "points": [[0.0,0.0],[40.0,0.0],[40.0,40.0],[0.0,40.0]] },
                        "islands": [],
                        "depth": 3.0,
                        "stepdown": 1.5,
                        "overlap": 0.4,
                        "offset": 0.2,
                        "spindle_rpm": 9000.0,
                        "work_offset": 2,
                        "feed": 600.0,
                        "plunge_feed": 200.0,
                        "plunge": "Straight",
                        "start": [5.0, 5.0],
                        "lead_overlap": 1.0,
                        "lead_in": "None",
                        "lead_out": "None",
                        "clearing": { "engagement": 2.0, "climb": true }
                    }}
                ]
            }
        })
    }

    fn pocket_of(doc: &Value) -> &Map<String, Value> {
        doc["setup"]["operations"][0]["Pocket"]
            .as_object()
            .expect("the fixture has one pocket")
    }

    #[test]
    fn a_v9_pocket_moves_its_clearing_fields_under_clear() {
        let mut doc = v9_document_with_a_pocket();
        document(&mut doc, 9).expect("migrates");

        let pocket = pocket_of(&doc);
        let clear = pocket["clear"].as_object().expect("nested clear object");

        // Every moved field arrives with its value intact...
        assert_eq!(clear["stepdown"], 1.5);
        assert_eq!(clear["overlap"], 0.4);
        assert_eq!(clear["offset"], 0.2);
        assert_eq!(clear["feed"], 600.0);
        assert_eq!(clear["plunge_feed"], 200.0);
        assert_eq!(clear["plunge"], "Straight");
        assert_eq!(clear["lead_overlap"], 1.0);
        assert_eq!(clear["clearing"]["engagement"], 2.0);

        // ...and leaves no copy behind. A field present in both places is the sort of
        // thing that reads fine until the two disagree.
        for field in V9_POCKET_CLEARING_FIELDS {
            assert!(
                !pocket.contains_key(field),
                "`{field}` was copied, not moved — the flat value is still there"
            );
        }
    }

    #[test]
    fn migration_leaves_the_pockets_own_fields_alone() {
        // The identity half of the step, and the one a careless field list breaks:
        // `offset` moves but `work_offset` must not, and both contain "offset".
        let mut doc = v9_document_with_a_pocket();
        document(&mut doc, 9).expect("migrates");

        let pocket = pocket_of(&doc);
        assert_eq!(pocket["id"], 1);
        assert_eq!(pocket["tool"], 1);
        assert_eq!(pocket["depth"], 3.0);
        assert_eq!(pocket["spindle_rpm"], 9000.0);
        assert_eq!(pocket["work_offset"], 2, "work_offset is not a clearing field");
        assert_eq!(pocket["start"], serde_json::json!([5.0, 5.0]));
        assert!(pocket.contains_key("boundary"));
    }

    #[test]
    fn a_migrated_v9_document_deserializes_and_keeps_what_it_said() {
        // The end-to-end claim: an old file opens, and the values a machinist entered
        // are the values the model holds. A migration that parses but shifts a number
        // is worse than one that fails.
        let mut doc = v9_document_with_a_pocket();
        document(&mut doc, 9).expect("migrates");

        let parsed: Document = serde_json::from_value(doc).expect("v10 document parses");
        assert_eq!(parsed.schema_version, SCHEMA_VERSION);
        let crate::Operation::Pocket(p) = &parsed.setup.operations[0] else {
            panic!("expected a pocket");
        };
        assert_eq!(p.depth, 3.0);
        assert_eq!(p.work_offset, 2);
        assert_eq!(p.clear.stepdown, 1.5);
        assert_eq!(p.clear.overlap, 0.4);
        assert_eq!(p.clear.offset, 0.2);
        assert_eq!(p.clear.feed, 600.0);
        assert_eq!(p.clear.plunge_feed, 200.0);
        assert_eq!(p.clear.lead_overlap, 1.0);
        assert_eq!(p.clear.clearing.engagement, 2.0);
        assert!(p.clear.clearing.climb);
        assert_eq!(p.start, Some([5.0, 5.0]));
    }

    #[test]
    fn a_pocket_missing_a_required_clearing_field_is_refused() {
        // Not pedantry: `ClearParams` defaults each field individually, so a dropped
        // `overlap` becomes 0.0 — full-diameter ring spacing, which leaves uncut ridges
        // between passes. Silently clearing a pocket wrong is the outcome being refused.
        for field in V9_POCKET_REQUIRED_FIELDS {
            let mut doc = v9_document_with_a_pocket();
            doc["setup"]["operations"][0]["Pocket"]
                .as_object_mut()
                .unwrap()
                .remove(field);
            let err = document(&mut doc, 9).expect_err("a pocket without `{field}` is damaged");
            assert!(
                matches!(err, MigrationError::Malformed { step: 9, ref detail } if detail.contains(field)),
                "expected a malformed-file error naming `{field}`, got {err:?}"
            );
        }
    }

    #[test]
    fn a_failed_migration_does_not_stamp_the_new_version() {
        // If a half-migrated tree kept the current stamp, a later reader would take it
        // for a finished v10 document and the damage would be invisible.
        let mut doc = v9_document_with_a_pocket();
        doc["setup"]["operations"][0]["Pocket"]
            .as_object_mut()
            .unwrap()
            .remove("stepdown");
        assert!(document(&mut doc, 9).is_err());
        assert_eq!(doc["schema_version"], 9, "the stamp still matches the content");
    }

    #[test]
    fn only_pockets_are_touched() {
        // The step walks every operation; anything that is not a pocket must come out
        // byte-identical, including a carve, which already has a `clear` of its own
        // shape (`{tool, params}`) and would be quietly wrecked by a blind rewrite.
        let mut doc = v9_document_with_a_pocket();
        let carve = serde_json::json!({ "Carve": {
            "id": 2, "tool": 3, "depth": 1.0,
            "clear": { "tool": 4, "params": { "stepdown": 0.5, "overlap": 0.5 } }
        }});
        doc["setup"]["operations"]
            .as_array_mut()
            .unwrap()
            .push(carve.clone());

        document(&mut doc, 9).expect("migrates");
        assert_eq!(
            doc["setup"]["operations"][1], carve,
            "a non-pocket operation was modified"
        );
    }

    #[test]
    fn a_current_document_is_unchanged() {
        // Zero steps, and specifically *not* a special case in `document` — a v10 file
        // must survive the same code path that migrates.
        let mut doc = v9_document_with_a_pocket();
        document(&mut doc, 9).expect("to v10");
        let once = doc.clone();
        document(&mut doc, SCHEMA_VERSION).expect("already current");
        assert_eq!(doc, once, "migrating a current document changed it");
    }

    #[test]
    fn a_file_from_the_future_is_refused_rather_than_opened() {
        // Opening it would drop every field this build does not know on the next save,
        // silently deleting the newer version's work.
        let mut doc = v9_document_with_a_pocket();
        assert_eq!(
            document(&mut doc, SCHEMA_VERSION + 1),
            Err(MigrationError::FromTheFuture {
                file: SCHEMA_VERSION + 1,
                current: SCHEMA_VERSION,
            })
        );
    }

    #[test]
    fn a_file_older_than_the_oldest_supported_is_refused() {
        let mut doc = v9_document_with_a_pocket();
        assert_eq!(
            document(&mut doc, OLDEST_SUPPORTED - 1),
            Err(MigrationError::TooOld {
                file: OLDEST_SUPPORTED - 1,
                oldest: OLDEST_SUPPORTED,
            })
        );
    }

    /// A **real** v1 document, taken from a project file written before `Stock` was
    /// part-relative: `Stock::Box { min, max }`, a bare `"EndMill"` tool kind, no origin.
    fn v1_document_with_an_absolute_stock_box() -> Value {
        serde_json::json!({
            "schema_version": 1,
            "setup": {
                "name": "part",
                "heights": { "clearance": 5.0, "retract": 2.0, "top_of_stock": 0.0 },
                "stock": { "Box": {
                    "min": [-26.775_638_580_322_266, -12.197_252_273_559_57, -4.0],
                    "max": [11.419_734_954_833_984, 6.171_515_941_619_873, 0.0]
                }},
                "tools": [{
                    "number": 1, "diameter": 3.0, "length": 20.0,
                    "flutes": 2, "kind": "EndMill"
                }],
                "operations": []
            }
        })
    }

    #[test]
    fn a_v1_stock_box_is_refused_by_version_rather_than_failing_to_parse() {
        // The regression this constant exists for. `OLDEST_SUPPORTED` was `1` when this
        // module shipped, which claimed a v1 file would open; it does not, because v2→v3
        // replaced `Stock::Box` with `Stock::BoundingBox` and shipped with "no migration
        // — early stage" on the record.
        //
        // The distinction is the whole point. Migrating and letting serde fail gives the
        // user `unknown variant 'Box', expected 'BoundingBox'` — a message that blames
        // the file's contents for what is really an unsupported version, and that no
        // amount of editing the file could act on. Refusing by version says the true
        // thing.
        let mut doc = v1_document_with_an_absolute_stock_box();
        assert_eq!(
            document(&mut doc, 1),
            Err(MigrationError::TooOld {
                file: 1,
                oldest: OLDEST_SUPPORTED
            })
        );

        // And prove the premise rather than asserting it: run the steps this version
        // *would* have taken and confirm the result really cannot be deserialized. If
        // some later change made `Stock::Box` loadable again, this fails and the
        // refusal above should be revisited rather than left as folklore.
        let mut forced = v1_document_with_an_absolute_stock_box();
        for v in 1..SCHEMA_VERSION {
            let _ = step(v, &mut forced);
        }
        let err = serde_json::from_value::<Document>(forced)
            .expect_err("a v1 stock must not parse as the current Stock");
        assert!(
            err.to_string().contains("Box"),
            "expected the stock variant to be what fails, got: {err}"
        );
    }

    #[test]
    fn every_supported_version_has_a_step() {
        // The guard on the module's one foreseeable maintenance slip: bumping
        // SCHEMA_VERSION without adding a step. Without this the omission surfaces as a
        // `MissingStep` error on a user's file, not here.
        for from in OLDEST_SUPPORTED..SCHEMA_VERSION {
            // A step may legitimately *reject* this particular fixture (it is a v9
            // document, not a v3 one); what is being asserted is only that a step exists
            // at all, so any outcome but `MissingStep` passes.
            let mut doc = v9_document_with_a_pocket();
            if let Err(MigrationError::MissingStep { .. }) = step(from, &mut doc) {
                panic!(
                    "no migration step from v{from} — SCHEMA_VERSION was bumped \
                     without adding one to `step`"
                );
            }
        }
    }

    #[test]
    fn a_document_without_operations_is_reported_as_damaged() {
        let mut doc = serde_json::json!({ "schema_version": 9, "setup": {} });
        assert!(matches!(
            document(&mut doc, 9),
            Err(MigrationError::Malformed { step: 9, .. })
        ));
    }
}
