//! Shared move emission: cut a closed loop, refitting flattened corners to arcs,
//! and the one rule every strategy's descent obeys ([`descend_to`]).

use cam_cldata::{MoveKind, Point3, Program, Step, Tag};
use cam_cldata::ArcDir;
use cam_geo::{fit_arcs, PathSeg, Point};
use cam_model::Heights;

/// Tolerance for recognising arcs in flattened offset loops (mm).
const ARCFIT_TOL: f64 = 0.01;

/// How far above a cut floor a rapid must stop, mm.
///
/// A descent that ends a **rapid** exactly on the surface cut by the previous pass has
/// no margin at all: an uncut cusp left by the last pass, stock that sprang back, a
/// pass that ran fractionally shallow, or a small Z-zero error, and the rapid ends in
/// metal. The geometry model cannot see any of those — to it, the air below really is
/// empty, which is why `cam-sim` rightly calls such a path clean.
///
/// It is the same reasoning already applied one level up (approach stops at the retract
/// plane rather than on the stock surface) and the same number as `PECK_CLEARANCE` in
/// `cam-post`, which stops a peck short of the previous peck's bottom for this reason.
pub(crate) const FLOOR_CLEARANCE: f64 = 0.5;

/// The lowest Z a **rapid** may end at when the cut floor beneath it is `floor_z`.
///
/// Two bounds, whichever is higher up: never below the **stock top**, since above the
/// surface there is definitionally nothing to hit, and never within
/// [`FLOOR_CLEARANCE`] of the **floor** the tool is returning to.
///
/// Note this may return a Z *above* `floor_z` — that is the point — and also one below
/// it when `floor_z` sits above the stock top (the first level starts at the retract
/// plane, and descending by rapid to the surface is free). Callers that stop at
/// `floor_z` rather than continuing past it must therefore clamp with `.max(floor_z)`;
/// [`descend_to`] does.
pub(crate) fn rapid_floor(floor_z: f64, h: &Heights) -> f64 {
    h.top_of_stock.min(floor_z + FLOOR_CLEARANCE)
}

/// Descend over `p` to `from_z` — the height an entry strategy starts from — rapiding
/// only as far as [`rapid_floor`] permits and **feeding** the remainder at cutting
/// feed.
///
/// Cutting feed, not plunge feed: what is being crossed is air the tool has already
/// cut, and plunge feed is for cutting downward into material (the measured cost of
/// getting that backwards was 4.7 minutes on a real export — see `clearing::clear`).
/// The feed is at most [`FLOOR_CLEARANCE`] long, so the time is negligible; what it
/// buys is that the move which *might* meet material is one the control can cut with
/// rather than crash on.
///
/// Leaves the tool at `p`/`from_z`, ready for the plunge strategy.
pub(crate) fn descend_to(
    prog: &mut Program,
    p: Point,
    from_z: f64,
    h: &Heights,
    feed: f64,
    id: u32,
) {
    // `.max`: this descent ends *at* `from_z`, so the free-air allowance above the
    // stock top must not drag the rapid below where it is going. On the first level
    // `from_z` is the retract plane and the whole descent stays a rapid, as before.
    let floor = rapid_floor(from_z, h).max(from_z);
    prog.push(Step::Rapid {
        to: Point3::new(p.x, p.y, floor),
        tag: Tag::new(id, MoveKind::Link),
    });
    if floor > from_z {
        // `Link`, not `Plunge`: `Plunge` means a vertical entry *into material*, and
        // both the chamfer and clearing suites read it that way — one plunge per pass,
        // and "with a helix selected no entry may be a bare straight drop". This move
        // is neither. It is a reposition that happens to be fed rather than rapided,
        // which is what `Link` already documents ("usually a rapid").
        prog.push(Step::Linear {
            to: Point3::new(p.x, p.y, from_z),
            feed,
            tag: Tag::new(id, MoveKind::Link),
        });
    }
}

/// Emit an open cutting polyline at height `z` exactly as given, converting runs
/// of flattened segments back into `G2`/`G3` arcs where they fit. Assumes the tool
/// is already positioned at `pts[0]` (a plunge precedes this call). Unlike
/// [`cut_loop`] this does not auto-close — the caller supplies the full point list
/// (e.g. a pre-closed loop, or a loop-plus-overlap from [`loop_with_overlap`]).
pub(crate) fn cut_polyline(prog: &mut Program, pts: &[Point], feed: f64, tag: Tag, z: f64) {
    if pts.len() < 2 {
        return;
    }
    for seg in fit_arcs(pts, ARCFIT_TOL) {
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

/// Emit a closed cutting loop at height `z`, converting runs of flattened
/// segments back into `G2`/`G3` arcs where they fit. Assumes the tool is already
/// positioned at `pts[0]` (a plunge precedes this call).
pub(crate) fn cut_loop(prog: &mut Program, pts: &[Point], feed: f64, tag: Tag, z: f64) {
    if pts.len() < 2 {
        return;
    }
    let mut loop_pts = pts.to_vec();
    loop_pts.push(pts[0]); // close the loop
    cut_polyline(prog, &loop_pts, feed, tag, z);
}

/// Build the cut polyline for a closed loop plus a *closure overlap*: the loop
/// `pts[0..n]` closed back to `pts[0]`, then continued along the contour
/// (`pts[0]→pts[1]→…`, wrapping) for `overlap` mm to an overlap point `P`. Returns
/// the polyline to feed [`cut_polyline`], the point `P` reached, and the unit
/// travel tangent at `P` (the direction of the segment `P` lies on).
///
/// For `overlap <= 0` (or a degenerate loop) it returns the plain closed loop,
/// `P = pts[0]`, and the arrival tangent at the start — so an overlap of zero is
/// byte-for-byte identical to [`cut_loop`] plus a lead-off/retract at the start.
/// The walk is capped at one full perimeter, so an overlap larger than the loop
/// simply stops after a lap rather than running away.
pub(crate) fn loop_with_overlap(pts: &[Point], overlap: f64) -> (Vec<Point>, Point, (f64, f64)) {
    let n = pts.len();
    let mut out = pts.to_vec();
    out.push(pts[0]); // close the loop
    if n < 2 {
        return (out, pts[0], (1.0, 0.0));
    }
    // Arrival tangent at the start (pts[last] → pts[0]) — the default lead-off/
    // retract direction when there is no overlap.
    let arrival = crate::profile::unit(pts[0].x - pts[n - 1].x, pts[0].y - pts[n - 1].y);
    if overlap <= 0.0 {
        return (out, pts[0], arrival);
    }
    let mut remaining = overlap;
    let mut from = pts[0];
    // Walk pts[1], pts[2], … (wrapping back through pts[0]), capped at one perimeter.
    for step in 1..=n {
        let to = pts[step % n];
        let seg = (to.x - from.x, to.y - from.y);
        let len = (seg.0 * seg.0 + seg.1 * seg.1).sqrt();
        if len <= 1e-12 {
            continue; // coincident vertices contribute no length
        }
        if len >= remaining {
            let t = remaining / len;
            let p = Point::new(from.x + seg.0 * t, from.y + seg.1 * t);
            out.push(p);
            return (out, p, crate::profile::unit(seg.0, seg.1));
        }
        remaining -= len;
        out.push(to);
        from = to;
    }
    // Overlap meets or exceeds the whole perimeter: stop at the last vertex (pts[0]).
    (out, from, arrival)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 10×10 CCW square starting at the origin; first edge runs +X.
    fn square() -> Vec<Point> {
        vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 10.0),
            Point::new(0.0, 10.0),
        ]
    }

    fn close(a: Point, b: Point) -> bool {
        (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9
    }

    #[test]
    fn zero_overlap_is_the_plain_closed_loop() {
        let pts = square();
        let (poly, p, tan) = loop_with_overlap(&pts, 0.0);
        // Closed loop: the four vertices plus the start again, nothing more.
        assert_eq!(poly.len(), 5);
        assert!(close(poly[4], pts[0]));
        assert!(close(p, pts[0]), "exit stays at the start");
        // Arrival tangent at the start: pts[3]→pts[0] is (0,-1).
        assert!((tan.0 - 0.0).abs() < 1e-9 && (tan.1 + 1.0).abs() < 1e-9);
    }

    #[test]
    fn overlap_within_the_first_edge_lands_partway_along_it() {
        let (poly, p, tan) = loop_with_overlap(&square(), 3.0);
        // Loop (5 pts) then one overlap point 3 mm along +X.
        assert_eq!(poly.len(), 6);
        assert!(close(p, Point::new(3.0, 0.0)));
        assert!(close(*poly.last().unwrap(), p));
        assert!((tan.0 - 1.0).abs() < 1e-9 && tan.1.abs() < 1e-9);
    }

    #[test]
    fn overlap_past_the_first_edge_turns_the_corner() {
        // 12 mm overlap: 10 along +X to the corner, then 2 up +Y.
        let (_poly, p, tan) = loop_with_overlap(&square(), 12.0);
        assert!(close(p, Point::new(10.0, 2.0)));
        assert!(tan.0.abs() < 1e-9 && (tan.1 - 1.0).abs() < 1e-9);
    }

    #[test]
    fn overlap_beyond_the_perimeter_is_capped_at_one_lap() {
        // The walk stops after a full lap rather than running away.
        let (poly, p, _tan) = loop_with_overlap(&square(), 1000.0);
        assert!(close(p, Point::new(0.0, 0.0)));
        assert!(poly.len() <= 2 * square().len() + 2);
    }
}
