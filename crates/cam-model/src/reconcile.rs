//! **Shop-number reconciliation** (`TOOLING_PLAN.md` §6, Phase 2).
//!
//! A shop keeps its own canonical tool numbering. When a foreign project is opened,
//! each tool it carries is matched **by geometry** against the shop library and, on a
//! match, **renumbered** to the shop's number — rewriting every operation reference so
//! the numbering the CNC operator loads by stays the shop's own.
//!
//! This is pure `Document`-level surgery: [`Tool::identity`] is the geometric
//! fingerprint (number-independent), [`reconcile_tool_numbers`] is the remap. No GUI,
//! headless-testable — correctness here is priority #1 (a bad remap silently points an
//! operation at the wrong tool).
//!
//! **Phase-2 fingerprint is *scalar*** — over `kind` + its parameters + the cutter
//! dimensions + `flutes`. Adequate while only built-in tools exist; it upgrades to the
//! resolved-generatrix basis at Phase 4/6 (§6.1) so a custom import can match a
//! built-in of the same shape.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use crate::document::Setup;
use crate::{CutDir, Tool, ToolKind};

/// Quantise a millimetre value to whole **microns** (0.001 mm) — tools are nominal, so
/// this both kills float noise and gives an `Eq`/`Hash`-able key with an implicit
/// ~1 µm epsilon.
fn q(mm: f64) -> i64 {
    (mm * 1_000.0).round() as i64
}

/// Quantise an angle in degrees to whole **milli-degrees**.
fn qa(deg: f64) -> i64 {
    (deg * 1_000.0).round() as i64
}

/// A `ToolKind` reduced to an `Eq`/`Hash`-able discriminant with quantised parameters.
#[derive(Clone, PartialEq, Eq, Hash, Debug)]
enum ToolKindId {
    EndMill,
    BallMill,
    FaceMill,
    BullNose { corner_um: i64 },
    ChamferMill { angle_mdeg: i64, tip_um: i64 },
    Drill { angle_mdeg: i64 },
    /// `pitch_um == 0` ⇒ single-form (`None`).
    ThreadMill { pitch_um: i64 },
    VBit { angle_mdeg: i64, tip_um: i64 },
}

fn kind_id(k: &ToolKind) -> ToolKindId {
    match *k {
        ToolKind::EndMill => ToolKindId::EndMill,
        ToolKind::BallMill => ToolKindId::BallMill,
        ToolKind::FaceMill => ToolKindId::FaceMill,
        ToolKind::BullNose { corner_radius } => ToolKindId::BullNose {
            corner_um: q(corner_radius),
        },
        ToolKind::ChamferMill {
            included_angle_deg,
            tip_diameter,
        } => ToolKindId::ChamferMill {
            angle_mdeg: qa(included_angle_deg),
            tip_um: q(tip_diameter),
        },
        ToolKind::Drill { point_angle_deg } => ToolKindId::Drill {
            angle_mdeg: qa(point_angle_deg),
        },
        ToolKind::ThreadMill { pitch } => ToolKindId::ThreadMill {
            pitch_um: pitch.map_or(0, q),
        },
        ToolKind::VBit {
            included_angle_deg,
            tip_radius,
        } => ToolKindId::VBit {
            angle_mdeg: qa(included_angle_deg),
            tip_um: q(tip_radius),
        },
    }
}

/// The geometric fingerprint of a tool — everything that makes two tools *the same
/// tool*, and nothing that doesn't. **Excludes** `number` (and, by construction,
/// names and any future holder position). Uses the **effective** cutter dimensions
/// ([`Tool::flute_len`] etc.) so an old tool (`0.0` sentinels) fingerprints identically
/// to one that spelled the same values out.
#[derive(Clone, PartialEq, Eq, Hash, Debug)]
pub struct ToolIdentity {
    kind: ToolKindId,
    diameter_um: i64,
    length_um: i64,
    flute_len_um: i64,
    shank_dia_um: i64,
    neck_len_um: i64,
    neck_dia_um: i64,
    flutes: u32,
    cutting_direction: CutDir,
}

impl Tool {
    /// This tool's geometric identity (see [`ToolIdentity`]).
    pub fn identity(&self) -> ToolIdentity {
        ToolIdentity {
            kind: kind_id(&self.kind),
            diameter_um: q(self.diameter),
            length_um: q(self.length),
            flute_len_um: q(self.flute_len()),
            shank_dia_um: q(self.shank_dia()),
            neck_len_um: q(self.neck_length),
            neck_dia_um: q(self.neck_dia()),
            flutes: self.flutes,
            cutting_direction: self.cutting_direction,
        }
    }
}

/// What a reconciliation pass did, for the Output-console summary (§6.3).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ReconcileReport {
    /// Matched a shop tool and **changed** number: `(old, new)`.
    pub matched_remapped: Vec<(u32, u32)>,
    /// Matched a shop tool whose number was already correct (no-op).
    pub matched_same: usize,
    /// Unmatched, kept its (project-local) number.
    pub local_kept: Vec<u32>,
    /// Unmatched, but its number clashed with a claimed shop number, so it moved:
    /// `(old, new)`.
    pub local_renumbered: Vec<(u32, u32)>,
}

impl ReconcileReport {
    /// Whether any number actually changed.
    pub fn changed(&self) -> bool {
        !self.matched_remapped.is_empty() || !self.local_renumbered.is_empty()
    }

    /// A one-line operator-facing summary, or `None` when nothing changed.
    pub fn summary(&self) -> Option<String> {
        if !self.changed() {
            return None;
        }
        let remapped = self.matched_remapped.len();
        let local = self.local_kept.len() + self.local_renumbered.len();
        let mut s = format!(
            "{remapped} tool{} remapped to shop numbers, {local} kept project-local",
            if remapped == 1 { "" } else { "s" }
        );
        if !self.local_renumbered.is_empty() {
            s.push_str(&format!(
                " ({} renumbered to avoid a clash)",
                self.local_renumbered.len()
            ));
        }
        Some(s)
    }
}

/// Compute the `old → new` number map for `tools` reconciled against `shop`, plus the
/// report. Pure — no mutation — so it is exhaustively unit-testable on tool lists
/// alone. Assumes `tools` carry **unique** numbers (a setup invariant).
fn plan_numbers(tools: &[Tool], shop: &[Tool]) -> (BTreeMap<u32, u32>, ReconcileReport) {
    // Shop identity → number; the *first* shop tool wins a duplicated identity.
    let mut shop_by_id: HashMap<ToolIdentity, u32> = HashMap::new();
    for t in shop {
        shop_by_id.entry(t.identity()).or_insert(t.number);
    }

    let mut mapping: BTreeMap<u32, u32> = BTreeMap::new();
    let mut used: BTreeSet<u32> = BTreeSet::new();
    let mut report = ReconcileReport::default();

    // Pass 1 — matched tools claim their shop number (first-come if two incoming tools
    // are identical and would claim the same one; the loser falls to pass 2).
    for t in tools {
        if let Some(&n) = shop_by_id.get(&t.identity()) {
            if !used.contains(&n) {
                used.insert(n);
                mapping.insert(t.number, n);
                if n == t.number {
                    report.matched_same += 1;
                } else {
                    report.matched_remapped.push((t.number, n));
                }
            }
        }
    }

    // Pass 2 — everything not yet mapped keeps its own number if free, else the next
    // free number. This covers unmatched tools *and* any matched-but-collided duplicate.
    for t in tools {
        if mapping.contains_key(&t.number) {
            continue;
        }
        let mut n = t.number.max(1);
        while used.contains(&n) {
            n = n.checked_add(1).expect("tool-number space exhausted");
        }
        used.insert(n);
        mapping.insert(t.number, n);
        if n == t.number {
            report.local_kept.push(n);
        } else {
            report.local_renumbered.push((t.number, n));
        }
    }

    (mapping, report)
}

/// Reconcile a setup's tool numbering against the shop `shop` library, **in place**:
/// each tool is matched by [`identity`](Tool::identity) and, on a match, adopts the
/// shop number; unmatched tools stay project-local (renumbered only to avoid a clash).
/// Every `Operation.tool` reference is rewritten so the setup stays a clean bijection.
/// Returns a [`ReconcileReport`] (its [`summary`](ReconcileReport::summary) is what the
/// GUI prints).
pub fn reconcile_tool_numbers(setup: &mut Setup, shop: &[Tool]) -> ReconcileReport {
    let (mapping, report) = plan_numbers(&setup.tools, shop);
    for t in &mut setup.tools {
        if let Some(&n) = mapping.get(&t.number) {
            t.number = n;
        }
    }
    for op in &mut setup.operations {
        // Every reference, not just the defining tool: a multi-tool operation's
        // secondary tool must be renumbered too or it dangles.
        op.map_tools(|old| mapping.get(&old).copied().unwrap_or(old));
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::document::{DrillOp, Heights, Operation, Stock};

    fn heights() -> Heights {
        Heights {
            clearance: 5.0,
            retract: 2.0,
            top_of_stock: 0.0,
        }
    }

    fn t(number: u32, diameter: f64, kind: ToolKind) -> Tool {
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
    fn identity_ignores_number_but_not_geometry() {
        let a = t(1, 12.0, ToolKind::EndMill);
        let b = t(7, 12.0, ToolKind::EndMill);
        assert_eq!(a.identity(), b.identity(), "same tool, different number ⇒ same id");

        // Differing flutes, length, diameter, or kind ⇒ different identity.
        let mut c = a;
        c.flutes = 3;
        assert_ne!(a.identity(), c.identity(), "flutes count");
        let mut d = a;
        d.length = 45.0;
        assert_ne!(a.identity(), d.identity(), "overall length (reach)");
        assert_ne!(a.identity(), t(1, 10.0, ToolKind::EndMill).identity(), "diameter");
        assert_ne!(a.identity(), t(1, 12.0, ToolKind::BallMill).identity(), "kind");

        // Cutting direction is a physical property, so it distinguishes tools too.
        let mut up = a;
        up.cutting_direction = CutDir::Up;
        assert_ne!(a.identity(), up.identity(), "up-cut vs down-cut");
    }

    #[test]
    fn effective_flute_length_makes_sentinel_and_explicit_equal() {
        let sentinel = t(1, 12.0, ToolKind::EndMill); // flute_length 0 ⇒ eff 30
        let mut explicit = t(2, 12.0, ToolKind::EndMill);
        explicit.flute_length = 30.0;
        assert_eq!(
            sentinel.identity(),
            explicit.identity(),
            "0-sentinel (fully fluted) fingerprints like an explicit full flute length"
        );
    }

    #[test]
    fn match_adopts_the_shop_number() {
        let shop = [t(1, 12.0, ToolKind::EndMill), t(2, 6.0, ToolKind::EndMill)];
        let incoming = [t(7, 12.0, ToolKind::EndMill)];
        let (map, rep) = plan_numbers(&incoming, &shop);
        assert_eq!(map[&7], 1, "⌀12 end mill adopts shop #1");
        assert_eq!(rep.matched_remapped, vec![(7, 1)]);
        assert!(rep.changed());
        assert_eq!(rep.summary().as_deref(), Some("1 tool remapped to shop numbers, 0 kept project-local"));
    }

    #[test]
    fn already_correct_number_is_a_noop() {
        let shop = [t(1, 12.0, ToolKind::EndMill)];
        let incoming = [t(1, 12.0, ToolKind::EndMill)];
        let (map, rep) = plan_numbers(&incoming, &shop);
        assert_eq!(map[&1], 1);
        assert_eq!(rep.matched_same, 1);
        assert!(!rep.changed(), "no number moved");
        assert_eq!(rep.summary(), None);
    }

    #[test]
    fn unmatched_tool_stays_project_local() {
        let shop = [t(1, 12.0, ToolKind::EndMill)];
        let incoming = [t(5, 3.0, ToolKind::Drill { point_angle_deg: 118.0 })];
        let (map, rep) = plan_numbers(&incoming, &shop);
        assert_eq!(map[&5], 5, "no match ⇒ keeps its own number");
        assert_eq!(rep.local_kept, vec![5]);
        assert!(rep.matched_remapped.is_empty());
    }

    #[test]
    fn near_miss_length_is_not_a_match() {
        // A ⌀12 end mill 45 mm long must NOT adopt the shop's ⌀12/30 mm number.
        let shop = [t(1, 12.0, ToolKind::EndMill)]; // length 30
        let mut long = t(1, 12.0, ToolKind::EndMill);
        long.length = 45.0;
        long.number = 9;
        let (map, rep) = plan_numbers(&[long], &shop);
        assert_eq!(map[&9], 9, "different reach ⇒ different tool, kept local");
        assert!(rep.matched_remapped.is_empty());
    }

    #[test]
    fn collision_batch_stays_a_clean_bijection() {
        // Shop #4 is the ⌀12 end mill. Incoming has that tool as #7 (→ must become #4),
        // and a *different* tool (⌀8) already sitting on #4 (→ must move off 4).
        let shop = [t(4, 12.0, ToolKind::EndMill)];
        let incoming = [
            t(7, 12.0, ToolKind::EndMill), // matches shop #4
            t(4, 8.0, ToolKind::EndMill),  // unmatched, currently on #4
        ];
        let (map, rep) = plan_numbers(&incoming, &shop);
        assert_eq!(map[&7], 4, "the ⌀12 adopts shop #4");
        assert_ne!(map[&4], 4, "the ⌀8 must move off #4");
        // Bijection: no two old numbers map to the same new number.
        let news: BTreeSet<u32> = map.values().copied().collect();
        assert_eq!(news.len(), map.len(), "mapping is injective");
        assert_eq!(rep.matched_remapped, vec![(7, 4)]);
        assert_eq!(rep.local_renumbered.len(), 1);
    }

    #[test]
    fn duplicate_identical_incoming_tools_do_not_both_claim_the_shop_number() {
        let shop = [t(2, 12.0, ToolKind::EndMill)];
        let incoming = [
            t(5, 12.0, ToolKind::EndMill), // → shop #2
            t(6, 12.0, ToolKind::EndMill), // identical; #2 already taken → stays local
        ];
        let (map, _) = plan_numbers(&incoming, &shop);
        assert_eq!(map[&5], 2);
        assert_ne!(map[&6], 2, "the second identical tool cannot also be #2");
        let news: BTreeSet<u32> = map.values().copied().collect();
        assert_eq!(news.len(), 2, "still a bijection");
    }

    #[test]
    fn operation_tool_references_are_rewritten() {
        let mk_drill = |id: u32, tool: u32| {
            Operation::Drill(DrillOp {
                spindle_rpm: 0.0,
                work_offset: 1,
                id,
                tool,
                points: vec![[0.0, 0.0]],
                depth: 4.0,
                start_offset: 0.0,
                peck: None,
                dwell: None,
                feed: 100.0,
            })
        };
        let mut setup = Setup {
            name: "s".into(),
            heights: heights(),
            stock: Stock::BoundingBox {
                x_offset: 0.0,
                y_offset: 0.0,
                top: 0.0,
                thickness: 10.0,
            },
            tools: vec![t(7, 12.0, ToolKind::EndMill), t(4, 8.0, ToolKind::EndMill)],
            operations: vec![mk_drill(1, 7), mk_drill(2, 4)],
            origin: [0.0, 0.0, 0.0],
            start_offset: None,
            work_offsets: vec![crate::Datum::base()],
            replication: None,
        };
        let shop = [t(4, 12.0, ToolKind::EndMill)]; // ⌀12 is shop #4
        let rep = reconcile_tool_numbers(&mut setup, &shop);

        // Tool #7 (⌀12) became #4; the ⌀8 that was #4 moved off it.
        let dia_of = |num: u32| setup.tools.iter().find(|t| t.number == num).map(|t| t.diameter);
        assert_eq!(dia_of(4), Some(12.0), "shop #4 is now the ⌀12 tool");
        assert!(setup.tools.iter().all(|t| t.number != 7), "old #7 is gone");
        // Op that cut with old #7 now references #4; op that cut with old #4 follows it.
        let op_tool = |id: u32| setup.operations.iter().find(|o| o.id() == id).unwrap().tool();
        assert_eq!(op_tool(1), 4, "op on the ⌀12 tool points at shop #4");
        let moved = rep.local_renumbered[0].1;
        assert_eq!(op_tool(2), moved, "op on the ⌀8 follows it to its new number");
        // Every op still points at a tool that exists.
        for op in &setup.operations {
            assert!(setup.tools.iter().any(|t| t.number == op.tool()), "no dangling ref");
        }
    }
}
