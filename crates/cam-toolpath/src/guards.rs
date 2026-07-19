//! **Tool-suitability guards** — shared checks that an operation is cutting with a
//! surface that actually cuts.
//!
//! ## The policy
//!
//! Two earlier decisions meet here and must not be confused:
//!
//! - *"Never reject a tool the user picks — the machinist stays free"* (2026-07-17).
//!   That is about **preference**: facing with a ball nose leaves scallops, but it
//!   does cut, and a machinist may have a reason.
//! - *Tools must be used with respect to their defined cutting surfaces.* That is
//!   about **possibility**: a chamfer mill's tip flat is tagged non-cutting, so
//!   plunging one does not cut at all — it rubs.
//!
//! So the rule applied throughout is:
//!
//! > **Error** when a *non-cutting* surface would be doing the cutting, or when the
//! > cut runs past the cutting edge onto neck/shank. **Warning** when the tool cuts,
//! > but leaves a worse result than the operation implies.
//!
//! Refusing to emit G-code that physically cannot cut is not overriding the
//! machinist's judgement; it is declining to produce a path that would rub, burn the
//! tool, and leave the feature uncut.
//!
//! ## Derived, not hand-written
//!
//! Wherever possible a guard asks the tool's own generatrix
//! ([`cam_geo::Profile2D`]) rather than matching on [`ToolKind`]: the cutting/
//! non-cutting tags are the authority, so a future imported custom tool is guarded
//! on the same terms as a built-in. A few rules genuinely are not in the geometry —
//! a twist drill's flutes *are* tagged cutting on the side, yet a drill must never
//! side-mill — and those are stated as explicit kind rules with a reason.

use cam_model::{Tool, ToolKind};

use crate::Diagnostic;

/// Guard a **side-milling** operation (profile walls, pocket walls): the tool must
/// have a cylindrical cutting flank, and the cut must stay on it.
///
/// `depth` is the axial depth of engagement in mm (a positive magnitude). Returns
/// `false` when the operation must not proceed.
pub(crate) fn check_side_milling(
    op_id: u32,
    op: &str,
    tool: &Tool,
    depth: f64,
    diagnostics: &mut Vec<Diagnostic>,
) -> bool {
    // Kind rules the generatrix cannot express.
    match tool.kind {
        // A twist drill's flutes are ground for axial cutting and chip evacuation;
        // the side lands are guides, not milling edges. Side-loading one deflects or
        // snaps it. The generatrix tags the flute side "cutting", so this must be
        // stated explicitly.
        ToolKind::Drill { .. } => {
            diagnostics.push(Diagnostic::error(format!(
                "operation {op_id} ({op}): tool {} is a drill bit. Its flutes cut on \
                 the point, not the side — side-milling with it deflects or breaks it. \
                 Use an end mill.",
                tool.number
            )));
            return false;
        }
        // A thread mill's flank is a thread form, not a plain cylinder: it would cut
        // a thread profile into the wall instead of a flat face.
        ToolKind::ThreadMill { .. } => {
            diagnostics.push(Diagnostic::error(format!(
                "operation {op_id} ({op}): tool {} is a thread mill. Its flank is a \
                 thread form, so it would cut a threaded wall, not a flat one.",
                tool.number
            )));
            return false;
        }
        _ => {}
    }

    let profile = tool.profile();
    let flank = profile.cutting_flank_height();
    if flank <= 1e-9 {
        diagnostics.push(Diagnostic::error(format!(
            "operation {op_id} ({op}): tool {} ({}) has no cylindrical cutting flank — \
             its cone flares straight into the shank, so it cannot cut a vertical wall \
             at any depth. Use an end mill.",
            tool.number, tool.kind
        )));
        return false;
    }
    if depth > flank + 1e-9 {
        diagnostics.push(Diagnostic::error(format!(
            "operation {op_id} ({op}): depth {depth:.3} mm exceeds tool {}'s {flank:.3} mm \
             length of cut — below that the non-cutting shank would rub against the wall. \
             Reduce the depth or use a longer-fluted tool.",
            tool.number
        )));
        return false;
    }
    true
}

/// Guard an operation that leaves a **flat floor** (facing, a pocket floor).
///
/// Never fatal on its own unless the tool cannot cut at its centre: a ball nose
/// leaves scallops but cuts, whereas a chamfer mill's non-cutting flat leaves an
/// uncut ridge under the axis.
pub(crate) fn check_flat_floor(
    op_id: u32,
    op: &str,
    tool: &Tool,
    diagnostics: &mut Vec<Diagnostic>,
) -> bool {
    let profile = tool.profile();
    if profile.cuts_flat_bottom() {
        return true;
    }
    if !profile.has_cutting_tip() {
        diagnostics.push(Diagnostic::error(format!(
            "operation {op_id} ({op}): tool {} ({}) has a non-cutting tip, so it would \
             leave an uncut ridge under its axis instead of a floor.",
            tool.number, tool.kind
        )));
        return false;
    }
    diagnostics.push(Diagnostic::warning(format!(
        "operation {op_id} ({op}): tool {} ({}) is not flat-bottomed — the floor will \
         follow the tool's shape (scallops/grooves), not come out flat.",
        tool.number, tool.kind
    )));
    true
}

/// Guard a **plunge** into solid material (drilling, a straight plunge entry): the
/// surface on the tool's axis must cut.
pub(crate) fn check_plunge(
    op_id: u32,
    op: &str,
    tool: &Tool,
    diagnostics: &mut Vec<Diagnostic>,
) -> bool {
    if tool.profile().has_cutting_tip() {
        return true;
    }
    diagnostics.push(Diagnostic::error(format!(
        "operation {op_id} ({op}): tool {} ({}) has a non-cutting tip and cannot plunge \
         into solid material — it would rub, not cut.",
        tool.number, tool.kind
    )));
    false
}

/// Guard the **axial reach** of a cut: the depth must stay on the cutting edge.
///
/// Used where the cut is not a vertical wall (drilling, engraving depth), so
/// [`check_side_milling`]'s flank rule does not apply, but burying the shank is still
/// wrong.
pub(crate) fn check_axial_reach(
    op_id: u32,
    op: &str,
    tool: &Tool,
    depth: f64,
    diagnostics: &mut Vec<Diagnostic>,
) -> bool {
    let profile = tool.profile();
    // The deepest the tool can be buried with only cutting surface in contact: the
    // flank where there is one, otherwise the height of the cutting end itself.
    let flank = profile.cutting_flank_height();
    let reach = if flank > 1e-9 {
        flank
    } else {
        cutting_end_height(&profile)
    };
    if depth > reach + 1e-9 {
        diagnostics.push(Diagnostic::error(format!(
            "operation {op_id} ({op}): depth {depth:.3} mm exceeds tool {}'s {reach:.3} mm \
             of cutting edge — deeper, the non-cutting shank would be in the hole.",
            tool.number
        )));
        return false;
    }
    true
}

/// The height of the tool's contiguous cutting end above the tip — how deep it can be
/// buried before a non-cutting surface reaches the work.
fn cutting_end_height(profile: &cam_geo::Profile2D) -> f64 {
    let mut z = 0.0_f64;
    let mut prev = profile.start;
    let mut started = false;
    for s in &profile.segs {
        if !s.cutting {
            // Skip a *leading* non-cutting surface (a chamfer mill's flat tip); stop
            // only once the cutting edge has begun and then ended.
            if started {
                break;
            }
            prev = s.end;
            continue;
        }
        started = true;
        z = z.max(prev.y.max(s.end.y));
        prev = s.end;
    }
    z
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Severity;

    fn tool(kind: ToolKind) -> Tool {
        Tool {
            number: 4,
            diameter: 6.0,
            flute_length: 12.0,
            length: 40.0,
            flutes: 2,
            kind,
            ..Default::default()
        }
    }

    fn errs(d: &[Diagnostic]) -> Vec<String> {
        d.iter()
            .filter(|x| x.severity == Severity::Error)
            .map(|x| x.message.clone())
            .collect()
    }

    fn warns(d: &[Diagnostic]) -> Vec<String> {
        d.iter()
            .filter(|x| x.severity == Severity::Warning)
            .map(|x| x.message.clone())
            .collect()
    }

    #[test]
    fn side_milling_accepts_an_end_mill_within_its_flute_length() {
        let mut d = Vec::new();
        assert!(check_side_milling(0, "profile", &tool(ToolKind::EndMill), 10.0, &mut d));
        assert!(d.is_empty());
    }

    #[test]
    fn side_milling_rejects_a_cut_past_the_flute_length() {
        // The guard that stops the shank being dragged along the wall.
        let mut d = Vec::new();
        assert!(!check_side_milling(0, "profile", &tool(ToolKind::EndMill), 15.0, &mut d));
        assert_eq!(errs(&d).len(), 1);
        assert!(errs(&d)[0].contains("length of cut"));
    }

    #[test]
    fn side_milling_rejects_pointed_tools_outright() {
        for kind in [
            ToolKind::VBit {
                included_angle_deg: 60.0,
                tip_radius: 0.0,
            },
            ToolKind::ChamferMill {
                included_angle_deg: 90.0,
                tip_diameter: 0.0,
            },
        ] {
            let mut d = Vec::new();
            assert!(!check_side_milling(0, "profile", &tool(kind), 1.0, &mut d));
            assert!(errs(&d)[0].contains("no cylindrical cutting flank"), "{:?}", errs(&d));
        }
    }

    #[test]
    fn side_milling_rejects_a_drill_even_though_its_flutes_are_cutting() {
        // The generatrix tags a drill's side "cutting", so only an explicit kind rule
        // catches this — hence the test.
        let t = tool(ToolKind::Drill {
            point_angle_deg: 118.0,
        });
        assert!(t.profile().cutting_flank_height() > 0.0, "geometry alone would allow it");
        let mut d = Vec::new();
        assert!(!check_side_milling(0, "profile", &t, 1.0, &mut d));
        assert!(errs(&d)[0].contains("drill bit"));
    }

    #[test]
    fn side_milling_rejects_a_thread_mill() {
        let mut d = Vec::new();
        let t = tool(ToolKind::ThreadMill { pitch: None });
        assert!(!check_side_milling(0, "profile", &t, 1.0, &mut d));
        assert!(errs(&d)[0].contains("thread mill"));
    }

    #[test]
    fn a_flat_floor_errors_only_for_a_non_cutting_tip() {
        // Chamfer mill: fatal — the flat under the axis does not cut.
        let mut d = Vec::new();
        let cham = tool(ToolKind::ChamferMill {
            included_angle_deg: 90.0,
            tip_diameter: 1.0,
        });
        assert!(!check_flat_floor(0, "face", &cham, &mut d));
        assert!(errs(&d)[0].contains("uncut ridge"));

        // Ball nose: cuts, but scallops — a warning, honouring "never reject a tool
        // the machinist picked" for what is only a quality matter.
        let mut d = Vec::new();
        assert!(check_flat_floor(0, "face", &tool(ToolKind::BallMill), &mut d));
        assert!(errs(&d).is_empty());
        assert_eq!(warns(&d).len(), 1);

        // Square end mill: silent.
        let mut d = Vec::new();
        assert!(check_flat_floor(0, "face", &tool(ToolKind::EndMill), &mut d));
        assert!(d.is_empty());
    }

    #[test]
    fn plunging_is_refused_for_a_non_cutting_tip_only() {
        let mut d = Vec::new();
        let cham = tool(ToolKind::ChamferMill {
            included_angle_deg: 90.0,
            tip_diameter: 1.0,
        });
        assert!(!check_plunge(0, "drill", &cham, &mut d));
        assert!(errs(&d)[0].contains("cannot plunge"));

        for kind in [
            ToolKind::EndMill,
            ToolKind::BallMill,
            ToolKind::Drill {
                point_angle_deg: 118.0,
            },
            ToolKind::VBit {
                included_angle_deg: 60.0,
                tip_radius: 0.0,
            },
        ] {
            let mut d = Vec::new();
            assert!(check_plunge(0, "drill", &tool(kind), &mut d), "{kind:?}");
            assert!(d.is_empty());
        }
    }

    #[test]
    fn axial_reach_is_bounded_by_the_cutting_edge() {
        let t = tool(ToolKind::Drill {
            point_angle_deg: 118.0,
        });
        let mut d = Vec::new();
        assert!(check_axial_reach(0, "drill", &t, 10.0, &mut d));
        assert!(d.is_empty());
        let mut d = Vec::new();
        assert!(!check_axial_reach(0, "drill", &t, 20.0, &mut d));
        assert!(errs(&d)[0].contains("cutting edge"));
    }

    #[test]
    fn reach_skips_a_chamfer_mills_leading_non_cutting_flat() {
        // Regression: the first walk stopped at the *first* non-cutting segment, which
        // for a chamfer mill is its tip flat — reporting zero cutting height and
        // rejecting every chamfer. The cone above the flat is what cuts.
        let t = tool(ToolKind::ChamferMill {
            included_angle_deg: 90.0,
            tip_diameter: 1.0,
        });
        // ⌀6 (r=3), 90° (α=45°), flat r=0.5 → the cone reaches full radius at 2.5.
        let mut d = Vec::new();
        assert!(check_axial_reach(0, "chamfer", &t, 2.4, &mut d), "{:?}", errs(&d));
        let mut d = Vec::new();
        assert!(!check_axial_reach(0, "chamfer", &t, 2.6, &mut d));
    }

    #[test]
    fn axial_reach_for_a_pointed_tool_uses_its_cone_height() {
        // A V-bit has no flank, so the bound is the height of the cone itself.
        let t = tool(ToolKind::VBit {
            included_angle_deg: 60.0,
            tip_radius: 0.0,
        });
        let cone_h = 3.0 / (30.0_f64).to_radians().tan(); // r / tan α
        let mut d = Vec::new();
        assert!(check_axial_reach(0, "engrave", &t, cone_h - 0.01, &mut d));
        let mut d = Vec::new();
        assert!(!check_axial_reach(0, "engrave", &t, cone_h + 0.01, &mut d));
    }
}
