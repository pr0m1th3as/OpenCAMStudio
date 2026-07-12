//! P1 acceptance tests for `cam-geo`: the roadmap matrix — square, square+hole,
//! sharp corners, self-intersection — plus offset/boolean/contains behaviour.
//!
//! Areas are checked against exact Minkowski-sum formulas, not eyeballed.

use cam_geo::{
    clip_path, difference, intersection, offset, stroke_path, union, CapStyle, Containment,
    Contour, JoinStyle, Point, Polygon, Polyline,
};

use std::f64::consts::PI;

/// Axis-aligned square of `side`, lower-left corner at (`x`, `y`), CCW.
fn square(x: f64, y: f64, side: f64) -> Polygon {
    Polygon::new(Contour::new(vec![
        Point::new(x, y),
        Point::new(x + side, y),
        Point::new(x + side, y + side),
        Point::new(x, y + side),
    ]))
    .unwrap()
}

/// Assert `got ≈ want` within a relative tolerance (plus a small absolute floor
/// for values near zero).
fn approx(got: f64, want: f64, rel: f64) {
    let tol = (want.abs() * rel).max(1e-3);
    assert!(
        (got - want).abs() <= tol,
        "expected {want}, got {got} (tol {tol})"
    );
}

fn total_area(polys: &[Polygon]) -> f64 {
    polys.iter().map(Polygon::area).sum()
}

// ---------------------------------------------------------------------------
// Square
// ---------------------------------------------------------------------------

#[test]
fn square_area_and_orientation() {
    let s = square(0.0, 0.0, 10.0);
    approx(s.area(), 100.0, 1e-9);
    assert!(s.outer().is_ccw(), "outer boundary must be normalised CCW");
}

#[test]
fn offset_square_grow_is_minkowski_sum() {
    // Growing a convex polygon outward by d with round joins is its Minkowski
    // sum with a disk of radius d: area = A + perimeter·d + π·d².
    let s = square(0.0, 0.0, 10.0);
    let d = 2.0;
    let grown = offset(&[s], d, JoinStyle::Round).unwrap();
    assert_eq!(grown.len(), 1);
    let expected = 100.0 + 40.0 * d + PI * d * d;
    approx(total_area(&grown), expected, 1e-3);
}

#[test]
fn offset_square_shrink_keeps_sharp_inner_corners() {
    // Shrinking a convex square inward by d yields a smaller square of side
    // (s − 2d); convex corners recede to sharp points, so no arcs are added.
    let s = square(0.0, 0.0, 10.0);
    let shrunk = offset(&[s], -2.0, JoinStyle::Round).unwrap();
    assert_eq!(shrunk.len(), 1);
    approx(total_area(&shrunk), 36.0, 1e-3);
}

#[test]
fn offset_is_translation_invariant() {
    // The fixed grid must give the same result wherever the shape sits.
    let d = 1.7;
    let a = total_area(&offset(&[square(0.0, 0.0, 10.0)], d, JoinStyle::Round).unwrap());
    let b = total_area(&offset(&[square(137.4, -88.1, 10.0)], d, JoinStyle::Round).unwrap());
    approx(a, b, 1e-9);
}

#[test]
fn offset_past_medial_axis_vanishes() {
    // Shrink a 10 mm square by 6 mm (> half-width): it disappears entirely.
    let s = square(0.0, 0.0, 10.0);
    let gone = offset(&[s], -6.0, JoinStyle::Round).unwrap();
    assert!(gone.is_empty(), "over-shrunk region should vanish");
}

// ---------------------------------------------------------------------------
// Square + hole
// ---------------------------------------------------------------------------

/// 20 mm square with a centred 4 mm square hole.
fn square_with_hole() -> Polygon {
    let outer = Contour::new(vec![
        Point::new(0.0, 0.0),
        Point::new(20.0, 0.0),
        Point::new(20.0, 20.0),
        Point::new(0.0, 20.0),
    ]);
    let hole = Contour::new(vec![
        Point::new(8.0, 8.0),
        Point::new(12.0, 8.0),
        Point::new(12.0, 12.0),
        Point::new(8.0, 12.0),
    ]);
    Polygon::with_holes(outer, vec![hole]).unwrap()
}

#[test]
fn square_with_hole_area_and_containment() {
    let p = square_with_hole();
    approx(p.area(), 400.0 - 16.0, 1e-9);
    assert!(p.contains(Point::new(2.0, 2.0)), "point in the solid ring");
    assert!(
        !p.contains(Point::new(10.0, 10.0)),
        "point inside the hole is not contained"
    );
    assert!(
        !p.contains(Point::new(-1.0, -1.0)),
        "point outside the outer boundary is not contained"
    );
    assert!(
        p.holes().len() == 1 && !p.holes()[0].is_ccw(),
        "the hole must be normalised clockwise"
    );
}

#[test]
fn difference_carves_a_hole() {
    // Big square minus a fully-interior small square ⇒ one region with one hole.
    let big = square(0.0, 0.0, 20.0);
    let small = square(8.0, 8.0, 4.0);
    let result = difference(&[big], &[small]).unwrap();
    assert_eq!(result.len(), 1);
    assert_eq!(
        result[0].holes().len(),
        1,
        "the removed square becomes a hole"
    );
    approx(result[0].area(), 400.0 - 16.0, 1e-3);
}

// ---------------------------------------------------------------------------
// Boolean basics
// ---------------------------------------------------------------------------

#[test]
fn union_merges_overlap_and_keeps_disjoint_separate() {
    let a = square(0.0, 0.0, 10.0);
    let b = square(5.0, 0.0, 10.0); // overlaps a in x ∈ [5,10]
    let merged = union(&[a], &[b]).unwrap();
    assert_eq!(merged.len(), 1);
    approx(total_area(&merged), 100.0 + 100.0 - 50.0, 1e-3);

    let c = square(0.0, 0.0, 10.0);
    let d = square(100.0, 100.0, 10.0); // far away
    let disjoint = union(&[c], &[d]).unwrap();
    assert_eq!(disjoint.len(), 2);
}

#[test]
fn intersection_of_overlapping_squares() {
    let a = square(0.0, 0.0, 10.0);
    let b = square(6.0, 6.0, 10.0); // overlap is a 4×4 square
    let hit = intersection(&[a], &[b]).unwrap();
    assert_eq!(hit.len(), 1);
    approx(total_area(&hit), 16.0, 1e-3);
}

// ---------------------------------------------------------------------------
// Sharp corners
// ---------------------------------------------------------------------------

/// A 60° wedge triangle — a genuinely sharp convex corner at the apex.
fn wedge() -> Polygon {
    Polygon::new(Contour::new(vec![
        Point::new(0.0, 0.0),
        Point::new(20.0, 0.0),
        Point::new(10.0, 3.0),
    ]))
    .unwrap()
}

#[test]
fn round_join_rounds_sharp_corners() {
    // A round join replaces each convex corner with an arc, so the offset
    // outline has many more vertices than the miter version, and its area is
    // strictly the Minkowski sum (A + perimeter·d + π·d²) regardless of corner
    // sharpness.
    let tri = wedge();
    let peri = {
        let p = tri.outer().points();
        let mut s = 0.0;
        for i in 0..p.len() {
            s += p[i].distance(p[(i + 1) % p.len()]);
        }
        s
    };
    let area0 = tri.area();
    let d = 1.5;

    let round = offset(std::slice::from_ref(&tri), d, JoinStyle::Round).unwrap();
    approx(total_area(&round), area0 + peri * d + PI * d * d, 5e-3);

    let miter = offset(&[tri], d, JoinStyle::Miter).unwrap();
    let round_v: usize = round.iter().map(|p| p.outer().len()).sum();
    let miter_v: usize = miter.iter().map(|p| p.outer().len()).sum();
    assert!(
        round_v > miter_v,
        "round join should add arc vertices (round={round_v}, miter={miter_v})"
    );
}

// ---------------------------------------------------------------------------
// Self-intersection robustness
// ---------------------------------------------------------------------------

#[test]
fn self_intersecting_contour_is_cleaned() {
    // A bow-tie: edges (0,0)-(4,4) and (4,0)-(0,4) cross at (2,2). Feeding this
    // through a union must not panic and must yield simple, valid rings.
    let bowtie = Polygon::new(Contour::new(vec![
        Point::new(0.0, 0.0),
        Point::new(4.0, 4.0),
        Point::new(4.0, 0.0),
        Point::new(0.0, 4.0),
    ]))
    .unwrap();

    let cleaned = union(&[bowtie], &[]).unwrap();
    assert!(
        !cleaned.is_empty(),
        "self-intersection should resolve to area"
    );
    for poly in &cleaned {
        assert!(poly.outer().is_valid());
    }
    // The crossing splits the shape into two triangular lobes meeting at (2,2),
    // each with vertices {(2,2),(4,4),(4,0)} / {(2,2),(0,4),(0,0)} — area 4 apiece.
    approx(total_area(&cleaned), 8.0, 1e-2);
}

// ---------------------------------------------------------------------------
// Open-path stroking (open-path offset = tool-sweep footprint)
// ---------------------------------------------------------------------------

/// A straight horizontal segment of the given length from the origin.
fn segment(len: f64) -> Polyline {
    Polyline::new(vec![Point::new(0.0, 0.0), Point::new(len, 0.0)])
}

#[test]
fn stroke_butt_cap_is_a_plain_rectangle() {
    // Stroking a length-10 segment with tool radius 1 and butt caps sweeps a
    // 10 × 2 rectangle: area 20.
    let s = stroke_path(&segment(10.0), 1.0, CapStyle::Butt, JoinStyle::Round).unwrap();
    assert_eq!(s.len(), 1);
    approx(total_area(&s), 20.0, 1e-3);
}

#[test]
fn stroke_round_cap_adds_a_full_circle() {
    // Round caps add two semicircles of radius 1 — a full unit circle of area π.
    let s = stroke_path(&segment(10.0), 1.0, CapStyle::Round, JoinStyle::Round).unwrap();
    approx(total_area(&s), 20.0 + PI, 5e-3);
}

#[test]
fn stroke_square_cap_extends_by_radius() {
    // Square caps extend the ribbon by the radius at each end: (10 + 2) × 2 = 24.
    let s = stroke_path(&segment(10.0), 1.0, CapStyle::Square, JoinStyle::Round).unwrap();
    approx(total_area(&s), 24.0, 1e-3);
}

#[test]
fn stroke_width_is_twice_the_radius() {
    // Radius is the half-width: radius 2 over length 10, butt caps ⇒ 10 × 4 = 40.
    let s = stroke_path(&segment(10.0), 2.0, CapStyle::Butt, JoinStyle::Round).unwrap();
    approx(total_area(&s), 40.0, 1e-3);
}

#[test]
fn stroke_of_invalid_path_or_zero_radius_is_empty() {
    let dot = Polyline::new(vec![Point::new(0.0, 0.0)]);
    assert!(stroke_path(&dot, 1.0, CapStyle::Round, JoinStyle::Round)
        .unwrap()
        .is_empty());
    assert!(
        stroke_path(&segment(10.0), 0.0, CapStyle::Round, JoinStyle::Round)
            .unwrap()
            .is_empty()
    );
}

// ---------------------------------------------------------------------------
// Path clipping (2-D sectioning) — also pins the keep-inside/outside direction
// ---------------------------------------------------------------------------

#[test]
fn clip_keeps_the_inside_portion() {
    // A line at y = 5 running x ∈ [-5, 15] across a 10 mm square: the inside
    // portion is x ∈ [0, 10] — one piece of length 10.
    let line = Polyline::new(vec![Point::new(-5.0, 5.0), Point::new(15.0, 5.0)]);
    let inside = clip_path(&line, &[square(0.0, 0.0, 10.0)], true).unwrap();
    assert_eq!(inside.len(), 1, "one contiguous inside piece");
    approx(inside[0].length(), 10.0, 1e-3);
}

#[test]
fn clip_keeps_the_outside_portions() {
    // The same line kept outside yields two pieces, x ∈ [-5,0] and x ∈ [10,15],
    // length 5 each.
    let line = Polyline::new(vec![Point::new(-5.0, 5.0), Point::new(15.0, 5.0)]);
    let outside = clip_path(&line, &[square(0.0, 0.0, 10.0)], false).unwrap();
    assert_eq!(outside.len(), 2, "two outside pieces");
    let total: f64 = outside.iter().map(Polyline::length).sum();
    approx(total, 10.0, 1e-3);
}

// ---------------------------------------------------------------------------
// On-boundary containment policy
// ---------------------------------------------------------------------------

#[test]
fn locate_classifies_inside_outside_and_boundary() {
    let s = square(0.0, 0.0, 10.0);
    assert_eq!(s.locate(Point::new(5.0, 5.0)), Containment::Inside);
    assert_eq!(s.locate(Point::new(-1.0, 5.0)), Containment::Outside);
    assert_eq!(s.locate(Point::new(5.0, 0.0)), Containment::OnBoundary); // on an edge
    assert_eq!(s.locate(Point::new(0.0, 0.0)), Containment::OnBoundary); // a vertex
}

#[test]
fn contains_treats_boundary_as_inside() {
    let s = square(0.0, 0.0, 10.0);
    assert!(
        s.contains(Point::new(5.0, 0.0)),
        "edge point counts as inside"
    );
    assert!(
        s.contains(Point::new(10.0, 10.0)),
        "corner counts as inside"
    );
    assert!(
        !s.contains(Point::new(10.001, 5.0)),
        "just outside is outside"
    );
}

#[test]
fn locate_detects_hole_boundary() {
    let p = square_with_hole(); // 20 mm square, 4 mm hole at [8,12]²
    assert_eq!(p.locate(Point::new(8.0, 10.0)), Containment::OnBoundary); // hole edge
    assert_eq!(p.locate(Point::new(10.0, 10.0)), Containment::Outside); // inside hole ⇒ outside region
    assert_eq!(p.locate(Point::new(2.0, 2.0)), Containment::Inside); // solid ring
}

#[test]
fn boolean_empty_inputs_do_not_panic() {
    let empty: Vec<Polygon> = Vec::new();
    let _ = union(&empty, &empty).unwrap();
    let _ = intersection(&empty, &empty).unwrap();
    let _ = difference(&empty, &empty).unwrap();
    let s = square(0.0, 0.0, 10.0);
    assert_eq!(
        difference(std::slice::from_ref(&s), &empty).unwrap().len(),
        1
    );
    assert!(intersection(std::slice::from_ref(&s), &empty)
        .unwrap()
        .is_empty());
}
