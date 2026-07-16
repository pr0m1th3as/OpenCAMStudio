//! Concentric-ring area clearing, shared by the pocket strategy and profile
//! roughing (stepover). Rings are offset loops of a region marching inward from
//! its wall; each is a plain closed cutting loop (approach, plunge per the plunge
//! strategy, cut with any closure overlap, retract).

use cam_cldata::{MoveKind, Point3, Program, Step, Tag};
use cam_geo::{offset, JoinStyle, Point, Polygon};
use cam_model::{Heights, Plunge};

use crate::CancelToken;

/// Safety cap on the number of concentric rings (guards the offset loop).
const MAX_RINGS: usize = 100_000;

/// Why ring generation stopped without producing rings.
pub(crate) enum RingsError {
    /// The job was cancelled mid-generation.
    Cancelled,
    /// A geometry offset failed.
    Offset(String),
}

/// Concentric offset rings clearing a region: offset it inward from the wall by
/// `first` (usually the tool radius), then by `stepover` repeatedly until it
/// closes off. Returns every resulting loop (outer boundaries and island/hole
/// loops), ordered by increasing offset (wall-most first). An empty result means
/// the tool cannot even enter.
pub(crate) fn concentric_rings(
    region: &Polygon,
    first: f64,
    stepover: f64,
    cancel: &CancelToken,
) -> Result<Vec<Vec<Point>>, RingsError> {
    let mut rings: Vec<Vec<Point>> = Vec::new();
    let mut d = first;
    loop {
        if cancel.is_cancelled() {
            return Err(RingsError::Cancelled);
        }
        let offsets = offset(std::slice::from_ref(region), -d, JoinStyle::Round)
            .map_err(|e| RingsError::Offset(e.to_string()))?;
        if offsets.is_empty() {
            break;
        }
        for poly in &offsets {
            rings.push(poly.outer().points().to_vec());
            for hole in poly.holes() {
                rings.push(hole.points().to_vec());
            }
        }
        d += stepover;
        // A non-positive stepover would never advance; take the wall ring and stop.
        if stepover <= 0.0 || rings.len() > MAX_RINGS {
            break;
        }
    }
    Ok(rings)
}

/// Emit approach, plunge, one closed cutting loop (plus any closure overlap), and
/// retract for a ring at height `z`. The entry uses the given plunge strategy: a
/// helix/ramp is placed on the *inward* side of the ring so it stays within the
/// cleared area, not the wall.
#[allow(clippy::too_many_arguments)]
pub(crate) fn emit_ring(
    prog: &mut Program,
    pts: &[Point],
    id: u32,
    feed: f64,
    plunge_feed: f64,
    plunge: Plunge,
    lead_overlap: f64,
    h: &Heights,
    z: f64,
) {
    if pts.len() < 3 {
        return;
    }
    let start = pts[0];
    let link = Tag::new(id, MoveKind::Link);
    let plunge_tag = Tag::new(id, MoveKind::Plunge);
    let cut = Tag::new(id, MoveKind::Cutting);
    let retract = Tag::new(id, MoveKind::Retract);

    prog.push(Step::Rapid {
        to: Point3::new(start.x, start.y, h.clearance),
        tag: link,
    });
    prog.push(Step::Rapid {
        to: Point3::new(start.x, start.y, h.top_of_stock),
        tag: link,
    });
    let tan = crate::profile::start_tangent(pts);
    let out = crate::profile::outward_normal(pts);
    crate::profile::emit_plunge(
        prog,
        start,
        tan,
        (-out.0, -out.1),
        h.top_of_stock,
        z,
        plunge,
        plunge_feed,
        feed,
        plunge_tag,
    );
    let (loop_pts, exit_pt, _tan) = crate::emit::loop_with_overlap(pts, lead_overlap);
    crate::emit::cut_polyline(prog, &loop_pts, feed, cut, z);
    prog.push(Step::Rapid {
        to: Point3::new(exit_pt.x, exit_pt.y, h.clearance),
        tag: retract,
    });
}
