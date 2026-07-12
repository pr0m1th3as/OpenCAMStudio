//! Shared move emission: cut a closed loop, refitting flattened corners to arcs.

use cam_cldata::{ArcDir, Point3, Program, Step, Tag};
use cam_geo::{fit_arcs, PathSeg, Point};

/// Tolerance for recognising arcs in flattened offset loops (mm).
const ARCFIT_TOL: f64 = 0.01;

/// Emit a closed cutting loop at height `z`, converting runs of flattened
/// segments back into `G2`/`G3` arcs where they fit. Assumes the tool is already
/// positioned at `pts[0]` (a plunge precedes this call).
pub(crate) fn cut_loop(prog: &mut Program, pts: &[Point], feed: f64, tag: Tag, z: f64) {
    if pts.len() < 2 {
        return;
    }
    let mut loop_pts = pts.to_vec();
    loop_pts.push(pts[0]); // close the loop

    for seg in fit_arcs(&loop_pts, ARCFIT_TOL) {
        match seg {
            PathSeg::Line { end } => prog.push(Step::Linear {
                to: Point3::new(end.x, end.y, z),
                feed,
                tag,
            }),
            PathSeg::Arc { end, center, ccw } => prog.push(Step::Arc {
                end: Point3::new(end.x, end.y, z),
                center: Point3::new(center.x, center.y, z),
                dir: if ccw { ArcDir::Ccw } else { ArcDir::Cw },
                feed,
                tag,
            }),
        }
    }
}
