//! Advisory cutter-suitability checks. The operator may deliberately choose a
//! cutter that leaves a different shape from the modeled feature. These findings
//! never suppress motions or simulation; numeric/path validity is checked by the
//! strategies, and simulation uses the selected tool's actual cutting profile.
use cam_model::{Tool, ToolKind};

use crate::Diagnostic;

// CAD bounding boxes carry small tolerance padding (typically 1e-7 mm).
// Allow one nanometre of numerical noise at an otherwise exact reach limit.
const REACH_TOLERANCE: f64 = 1e-6;

/// Report cutter suitability as warnings, without suppressing the operation.
pub(crate) fn warn_side_milling(
    op_id: u32,
    op: &str,
    tool: &Tool,
    depth: f64,
    diagnostics: &mut Vec<Diagnostic>,
) {
    // Kind rules the generatrix cannot express.
    match tool.kind {
        // A twist drill's flutes are ground for axial cutting and chip evacuation;
        // the side lands are guides, not milling edges. Side-loading one deflects or
        // snaps it. The generatrix tags the flute side "cutting", so this must be
        // stated explicitly.
        ToolKind::Drill { .. } => {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {op_id} ({op}): tool {} is a drill bit. Its flutes cut on \
                 the point, not the side — side-milling with it deflects or breaks it. \
                 Consider an end mill.",
                tool.number
            )));
            return;
        }
        // A thread mill's flank is a thread form, not a plain cylinder: it would cut
        // a thread profile into the wall instead of a flat face.
        ToolKind::ThreadMill { .. } => {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {op_id} ({op}): tool {} is a thread mill. Its flank is a \
                 thread form, so it would cut a threaded wall, not a flat one.",
                tool.number
            )));
            return;
        }
        _ => {}
    }

    let profile = tool.profile();
    let flank = profile.cutting_flank_height();
    if flank <= 1e-9 {
        diagnostics.push(Diagnostic::warning(format!(
            "operation {op_id} ({op}): tool {} ({}) has no cylindrical cutting flank — \
             the generated wall will follow the selected cutter's shape.",
            tool.number, tool.kind
        )));
        return;
    }
    if depth > flank + REACH_TOLERANCE {
        diagnostics.push(Diagnostic::warning(format!(
            "operation {op_id} ({op}): depth {depth:.3} mm exceeds tool {}'s {flank:.3} mm \
             length of cut — below that the non-cutting shank would rub against the wall. \
             Reduce the depth or use a longer-fluted tool.",
            tool.number
        )));
    }
}

/// Report cutter suitability as warnings, without suppressing the operation.
pub(crate) fn warn_flat_floor(
    op_id: u32,
    op: &str,
    tool: &Tool,
    diagnostics: &mut Vec<Diagnostic>,
) {
    let profile = tool.profile();
    if profile.cuts_flat_bottom() {
        return;
    }
    if !profile.has_cutting_tip() {
        diagnostics.push(Diagnostic::warning(format!(
            "operation {op_id} ({op}): tool {} ({}) has a non-cutting tip, so it would \
             leave an uncut ridge under its axis instead of a floor.",
            tool.number, tool.kind
        )));
        return;
    }
    diagnostics.push(Diagnostic::warning(format!(
        "operation {op_id} ({op}): tool {} ({}) is not flat-bottomed — the floor will \
         follow the tool's shape (scallops/grooves), not come out flat.",
        tool.number, tool.kind
    )));
}

/// Report cutter suitability as warnings, without suppressing the operation.
pub(crate) fn warn_plunge(op_id: u32, op: &str, tool: &Tool, diagnostics: &mut Vec<Diagnostic>) {
    if tool.profile().has_cutting_tip() {
        return;
    }
    diagnostics.push(Diagnostic::warning(format!(
        "operation {op_id} ({op}): tool {} ({}) has a non-cutting tip and cannot plunge \
         into solid material — it would rub, not cut.",
        tool.number, tool.kind
    )));
}

/// Report cutter suitability as warnings, without suppressing the operation.
pub(crate) fn warn_axial_reach(
    op_id: u32,
    op: &str,
    tool: &Tool,
    depth: f64,
    diagnostics: &mut Vec<Diagnostic>,
) {
    let profile = tool.profile();
    // The deepest the tool can be buried with only cutting surface in contact: the
    // flank where there is one, otherwise the height of the cutting end itself.
    let flank = profile.cutting_flank_height();
    let reach = if flank > 1e-9 {
        flank
    } else {
        cutting_end_height(&profile)
    };
    if depth > reach + REACH_TOLERANCE {
        diagnostics.push(Diagnostic::warning(format!(
            "operation {op_id} ({op}): depth {depth:.3} mm exceeds tool {}'s {reach:.3} mm \
             of cutting edge — deeper, the non-cutting shank would be in the hole.",
            tool.number
        )));
    }
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
            diameter: 6.,
            flute_length: 12.,
            length: 40.,
            flutes: 2,
            kind,
            ..Default::default()
        }
    }

    #[test]
    fn suitability_findings_are_warnings() {
        for kind in [
            ToolKind::VBit {
                included_angle_deg: 60.,
                tip_radius: 0.,
            },
            ToolKind::ChamferMill {
                included_angle_deg: 90.,
                tip_diameter: 1.,
            },
            ToolKind::Drill {
                point_angle_deg: 118.,
            },
            ToolKind::ThreadMill { pitch: None },
        ] {
            let mut diagnostics = Vec::new();
            let tool = tool(kind);
            warn_side_milling(7, "bore", &tool, 20., &mut diagnostics);
            warn_flat_floor(7, "bore", &tool, &mut diagnostics);
            warn_plunge(7, "bore", &tool, &mut diagnostics);
            warn_axial_reach(7, "bore", &tool, 50., &mut diagnostics);
            assert!(!diagnostics.is_empty(), "{kind:?}");
            assert!(diagnostics.iter().all(|d| d.severity == Severity::Warning));
        }
    }

    #[test]
    fn cutting_reach_allows_cad_padding_and_warns_for_real_overreach() {
        let tool = tool(ToolKind::EndMill);
        for extra in [0., 1e-7, 2e-7, 1e-4] {
            let mut diagnostics = Vec::new();
            warn_side_milling(0, "profile", &tool, 12. + extra, &mut diagnostics);
            warn_axial_reach(0, "bore", &tool, 12. + extra, &mut diagnostics);
            assert_eq!(
                diagnostics.len(),
                if extra < REACH_TOLERANCE { 0 } else { 2 }
            );
        }
    }

    #[test]
    fn a_small_end_mill_with_a_larger_shank_is_suitable_within_its_flutes() {
        let mut tool = tool(ToolKind::EndMill);
        tool.diameter = 3.;
        tool.shank_diameter = 6.;
        tool.flute_length = 8.;
        let mut diagnostics = Vec::new();
        warn_side_milling(0, "bore", &tool, 6., &mut diagnostics);
        warn_flat_floor(0, "bore", &tool, &mut diagnostics);
        assert!(diagnostics.is_empty(), "{diagnostics:?}");
    }

    #[test]
    fn reach_includes_a_chamfer_mills_cutting_cone_above_its_flat() {
        let mut tool = tool(ToolKind::ChamferMill {
            included_angle_deg: 90.,
            tip_diameter: 1.,
        });
        // End the cutting edge at the top of the cone; a longer flute also
        // has a cutting cylindrical flank above it.
        tool.flute_length = 2.5;
        let mut diagnostics = Vec::new();
        warn_axial_reach(0, "chamfer", &tool, 2.4, &mut diagnostics);
        assert!(diagnostics.is_empty());
        warn_axial_reach(0, "chamfer", &tool, 2.6, &mut diagnostics);
        assert_eq!(diagnostics.len(), 1);
        tool.flute_length = 12.;
        diagnostics.clear();
        warn_axial_reach(0, "chamfer", &tool, 2.6, &mut diagnostics);
        assert!(diagnostics.is_empty());
    }
}
