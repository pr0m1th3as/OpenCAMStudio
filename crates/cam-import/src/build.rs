//! Turn raw DXF entities into `cam-geo` regions: expand arcs and bulges to
//! points, chain open fragments into closed contours, and nest holes.

use cam_geo::{Arc, Contour, Point, Polygon, Polyline};

use crate::dxf::Entity;

/// A tolerance beyond `0.0` used to guard divisions.
const EPS: f64 = 1e-12;

/// A piece of geometry from one entity: either an already-closed loop or an open
/// fragment awaiting chaining.
enum Piece {
    Closed(Vec<Point>),
    Open(Vec<Point>),
}

/// Convert every entity to a piece, flattening arcs and bulges to `chord_tol`.
fn entity_to_piece(entity: &Entity, chord_tol: f64) -> Piece {
    match entity {
        Entity::Line { a, b } => Piece::Open(vec![Point::new(a.0, a.1), Point::new(b.0, b.1)]),
        Entity::Circle { center, radius } => {
            let pts = Arc::circle(Point::new(center.0, center.1), *radius).flatten(chord_tol);
            Piece::Closed(pts)
        }
        Entity::Arc {
            center,
            radius,
            start_deg,
            end_deg,
        } => {
            let arc = Arc::new(
                Point::new(center.0, center.1),
                *radius,
                start_deg.to_radians(),
                end_deg.to_radians(),
                true, // DXF arcs sweep counter-clockwise
            );
            Piece::Open(arc.flatten(chord_tol))
        }
        Entity::LwPolyline { closed, verts } => {
            let pts = lwpolyline_points(verts, *closed, chord_tol);
            if *closed {
                Piece::Closed(pts)
            } else {
                Piece::Open(pts)
            }
        }
    }
}

/// Expand an `LWPOLYLINE`'s vertices to points, turning each non-zero bulge into
/// a flattened arc.
fn lwpolyline_points(verts: &[(f64, f64, f64)], closed: bool, chord_tol: f64) -> Vec<Point> {
    if verts.is_empty() {
        return Vec::new();
    }
    let n = verts.len();
    let mut pts = vec![Point::new(verts[0].0, verts[0].1)];
    let segments = if closed { n } else { n - 1 };
    for i in 0..segments {
        let (x0, y0, bulge) = verts[i];
        let (x1, y1, _) = verts[(i + 1) % n];
        let p0 = Point::new(x0, y0);
        let p1 = Point::new(x1, y1);
        if bulge.abs() < 1e-9 {
            pts.push(p1);
        } else {
            pts.extend(bulge_arc(p0, p1, bulge, chord_tol));
        }
    }
    pts
}

/// Flatten a bulge arc from `p0` to `p1` (excluding `p0`, including `p1`).
/// `bulge = tan(θ/4)` where θ is the arc's signed included angle (positive is
/// counter-clockwise).
fn bulge_arc(p0: Point, p1: Point, bulge: f64, chord_tol: f64) -> Vec<Point> {
    let theta = 4.0 * bulge.atan();
    let (dx, dy) = (p1.x - p0.x, p1.y - p0.y);
    let len = (dx * dx + dy * dy).sqrt();
    if len < EPS {
        return vec![p1];
    }
    let half = len * 0.5;
    let mid = Point::new((p0.x + p1.x) * 0.5, (p0.y + p1.y) * 0.5);
    // Centre lies on the chord's left normal, at half / tan(θ/2) from the mid.
    let t = (theta * 0.5).tan();
    let dist = if t.abs() < EPS { 0.0 } else { half / t };
    let (nx, ny) = (-dy / len, dx / len);
    let center = Point::new(mid.x + nx * dist, mid.y + ny * dist);
    let r = (half / (theta * 0.5).sin()).abs();
    let start = (p0.y - center.y).atan2(p0.x - center.x);
    let end = (p1.y - center.y).atan2(p1.x - center.x);
    let mut pts = Arc::new(center, r, start, end, bulge > 0.0).flatten(chord_tol);
    if !pts.is_empty() {
        pts.remove(0); // drop p0; it is already in the chain
    }
    pts
}

/// Greedily stitch open fragments into closed contours, matching endpoints
/// within `weld_tol`. Fragments that cannot be closed are returned as leftovers.
fn chain(fragments: Vec<Vec<Point>>, weld_tol: f64) -> (Vec<Contour>, Vec<Vec<Point>>) {
    let mut used = vec![false; fragments.len()];
    let mut closed = Vec::new();
    let mut leftover = Vec::new();

    for i in 0..fragments.len() {
        if used[i] {
            continue;
        }
        used[i] = true;
        let mut chain = fragments[i].clone();

        loop {
            let end = *chain.last().unwrap();
            if chain.len() >= 4 && end.distance(chain[0]) <= weld_tol {
                break; // closed the loop
            }
            let mut extended = false;
            for (j, frag) in fragments.iter().enumerate() {
                if used[j] {
                    continue;
                }
                let (fs, fe) = (frag[0], *frag.last().unwrap());
                if end.distance(fs) <= weld_tol {
                    chain.extend_from_slice(&frag[1..]);
                } else if end.distance(fe) <= weld_tol {
                    chain.extend(frag[..frag.len() - 1].iter().rev().copied());
                } else {
                    continue;
                }
                used[j] = true;
                extended = true;
                break;
            }
            if !extended {
                break;
            }
        }

        if chain.len() >= 4 && chain.last().unwrap().distance(chain[0]) <= weld_tol {
            chain.pop(); // remove the near-duplicate closing point
            closed.push(Contour::new(chain));
        } else {
            leftover.push(chain);
        }
    }

    (closed, leftover)
}

/// Nest closed contours into filled regions by containment: a contour enclosed
/// by an odd number of others is a hole; each hole attaches to the innermost
/// contour that contains it.
fn nest(contours: Vec<Contour>) -> (Vec<Polygon>, Vec<String>) {
    let n = contours.len();
    let mut warnings = Vec::new();

    // A single-contour Polygon per input, for containment tests.
    let probes: Vec<Option<Polygon>> = contours
        .iter()
        .map(|c| Polygon::new(c.clone()).ok())
        .collect();

    // container[i] = indices of contours that strictly contain contour i.
    let mut container: Vec<Vec<usize>> = vec![Vec::new(); n];
    for i in 0..n {
        let probe = contours[i].points()[0];
        for (j, pj) in probes.iter().enumerate() {
            if i == j {
                continue;
            }
            if let Some(pj) = pj {
                if pj.contains(probe) {
                    container[i].push(j);
                }
            }
        }
    }
    let depth = |i: usize| container[i].len();

    // Innermost parent of i = the container with the greatest depth.
    let parent = |i: usize| -> Option<usize> {
        container[i]
            .iter()
            .copied()
            .max_by_key(|&j| container[j].len())
    };

    let mut regions = Vec::new();
    for i in 0..n {
        if depth(i) % 2 != 0 {
            continue; // a hole, emitted with its parent
        }
        let holes: Vec<Contour> = (0..n)
            .filter(|&h| depth(h) % 2 == 1 && parent(h) == Some(i))
            .map(|h| contours[h].clone())
            .collect();
        match Polygon::with_holes(contours[i].clone(), holes) {
            Ok(p) => regions.push(p),
            Err(e) => warnings.push(format!("skipped a region: {e}")),
        }
    }

    (regions, warnings)
}

/// Assemble raw entities into regions plus any open chains that could not close.
pub(crate) fn assemble(
    entities: &[Entity],
    weld_tol: f64,
    chord_tol: f64,
) -> (Vec<Polygon>, Vec<Polyline>, Vec<String>) {
    let mut closed_contours = Vec::new();
    let mut open_fragments = Vec::new();
    for entity in entities {
        match entity_to_piece(entity, chord_tol) {
            Piece::Closed(pts) => {
                let c = Contour::new(pts);
                if c.is_valid() {
                    closed_contours.push(c);
                }
            }
            Piece::Open(pts) => open_fragments.push(pts),
        }
    }

    let (chained, leftover) = chain(open_fragments, weld_tol);
    closed_contours.extend(chained);

    let (regions, mut warnings) = nest(closed_contours);
    if !leftover.is_empty() {
        warnings.push(format!(
            "{} open chain(s) could not be closed and were dropped from regions",
            leftover.len()
        ));
    }
    let open = leftover.into_iter().map(Polyline::new).collect();

    (regions, open, warnings)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bulge_of_one_is_a_semicircle() {
        // bulge = 1 ⇒ θ = π: a CCW semicircle from (0,0) to (10,0), centre
        // (5,0), radius 5, dipping to (5,-5) at the midpoint.
        let pts = bulge_arc(Point::new(0.0, 0.0), Point::new(10.0, 0.0), 1.0, 1e-3);
        let last = *pts.last().unwrap();
        assert!(last.distance(Point::new(10.0, 0.0)) < 1e-6, "ends at p1");
        // The semicircle dips to y = -5 at the bottom.
        let miny = pts.iter().map(|p| p.y).fold(f64::MAX, f64::min);
        assert!((miny + 5.0).abs() < 0.01, "reaches depth -5, got {miny}");
        // Every point is radius 5 from the centre.
        for p in &pts {
            assert!((p.distance(Point::new(5.0, 0.0)) - 5.0).abs() < 0.05);
        }
    }

    #[test]
    fn chains_four_scrambled_edges_into_one_loop() {
        // A 10×10 square given as four lines, some reversed, out of order.
        let frags = vec![
            vec![Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            vec![Point::new(10.0, 10.0), Point::new(10.0, 0.0)], // reversed
            vec![Point::new(0.0, 10.0), Point::new(0.0, 0.0)],
            vec![Point::new(10.0, 10.0), Point::new(0.0, 10.0)],
        ];
        let (closed, leftover) = chain(frags, 1e-6);
        assert_eq!(closed.len(), 1, "one closed contour");
        assert!(leftover.is_empty());
        assert!((closed[0].area() - 100.0).abs() < 1e-6);
    }

    #[test]
    fn open_fragment_is_not_closed() {
        let frags = vec![vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 10.0),
        ]];
        let (closed, leftover) = chain(frags, 1e-6);
        assert!(closed.is_empty());
        assert_eq!(leftover.len(), 1);
    }

    #[test]
    fn nests_a_circle_inside_a_square_as_a_hole() {
        let square = Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(20.0, 0.0),
            Point::new(20.0, 20.0),
            Point::new(0.0, 20.0),
        ]);
        let circle = Arc::circle(Point::new(10.0, 10.0), 3.0).flatten(1e-2);
        let (regions, warnings) = nest(vec![square, Contour::new(circle)]);
        assert!(warnings.is_empty());
        assert_eq!(regions.len(), 1, "one solid region");
        assert_eq!(regions[0].holes().len(), 1, "the circle is its hole");
    }
}
