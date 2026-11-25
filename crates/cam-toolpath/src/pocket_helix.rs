//! Helical entry for 2D pockets, using the entire allowed cutter-centre region.

use cam_cldata::{MoveKind, Point3, Program, Step, Tag};
use cam_geo::{offset, JoinStyle, Point, Polygon, GRID_MM};
use cam_model::{Heights, Plunge, MAX_PROFILE_MOVES};

use crate::profile::{loop_perimeter, rotate_to_start, unit, walk_loop};
use crate::rings::RingsError;

const MAX_RADIUS_DECIMALS: u8 = 8;
const RADIUS_UNITS_PER_MM: u64 = 10u64.pow(MAX_RADIUS_DECIMALS as u32);

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub(crate) struct PocketHelixFit {
    radius: f64,
    target: Point,
    fixed: bool,
    maximum_radius_units: u64,
}

impl PocketHelixFit {
    pub(crate) fn message(
        &self,
        decimals: Option<u8>,
        format_number: impl Fn(f64) -> String,
    ) -> String {
        let location = if self.fixed {
            "at the selected pocket entry"
        } else {
            "in the pocket area needed for this entry"
        };
        // Round down the maximum so the displayed setting can actually fit.
        let requested = if decimals.is_some() {
            format_number(self.radius)
        } else {
            self.radius.to_string()
        };
        let decimals = decimals.unwrap_or(3).min(MAX_RADIUS_DECIMALS);
        let divisor = 10u64.pow(u32::from(MAX_RADIUS_DECIMALS - decimals));
        let maximum =
            (self.maximum_radius_units / divisor) as f64 / 10u64.pow(u32::from(decimals)) as f64;
        format!(
            "Helix with radius {requested} mm does not fit {location} (X {}, Y {}). The maximum fitting helix radius is {} mm.",
            format_number(self.target.x),
            format_number(self.target.y),
            format_number(maximum),
        )
    }
}

impl std::fmt::Display for PocketHelixFit {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message(None, |value| format!("{value:.3}")))
    }
}

pub(crate) struct PocketHelix<'a> {
    guard: &'a [Polygon],
    centres: Vec<Vec<Polygon>>,
    radius: f64,
    pitch: f64,
}

pub(crate) struct Placement {
    start: Point,
    normal: (f64, f64),
    transfer: Vec<Point>,
}

impl<'a> PocketHelix<'a> {
    pub(crate) fn new(guard: &'a [Polygon], plunge: Plunge) -> Result<Option<Self>, RingsError> {
        let Plunge::Helix { radius, pitch } = plunge else {
            return Ok(None);
        };
        if !radius.is_finite() || radius <= 0.0 || !pitch.is_finite() || pitch <= 0.0 {
            return Err(RingsError::Limit(
                "Helix radius and pitch must be finite and positive".into(),
            ));
        }
        Self::with_radius(guard, radius, pitch).map(Some)
    }

    fn with_radius(guard: &'a [Polygon], radius: f64, pitch: f64) -> Result<Self, RingsError> {
        // Keep a thin centre region when the requested circle is tangent to
        // opposite walls. Every candidate is checked against the original guard.
        let inset = (radius - GRID_MM).max(0.0);
        let centres = guard
            .iter()
            .map(|region| offset(std::slice::from_ref(region), -inset, JoinStyle::Round))
            .collect::<Result<_, _>>()
            .map_err(|error| RingsError::Offset(error.to_string()))?;
        Ok(Self {
            guard,
            centres,
            radius,
            pitch,
        })
    }

    pub(crate) fn place(
        &self,
        target: Point,
        normal: (f64, f64),
        fixed: bool,
    ) -> Result<Placement, RingsError> {
        if let Some(placement) = self.try_place(target, normal, fixed) {
            return Ok(placement);
        }
        Err(RingsError::PocketHelixFit(PocketHelixFit {
            radius: self.radius,
            target,
            fixed,
            maximum_radius_units: self.maximum_fitting_radius_units(target, normal, fixed),
        }))
    }

    fn maximum_fitting_radius_units(&self, target: Point, normal: (f64, f64), fixed: bool) -> u64 {
        // Store the fit at the finest display precision. Smaller circles can
        // nest inside larger ones, even with a fixed point on the circumference.
        let scale = RADIUS_UNITS_PER_MM as f64;
        let mut lower = 0;
        let mut upper = (self.radius * scale).floor() as u64;
        while lower < upper {
            let middle = lower + (upper - lower).div_ceil(2);
            let radius = middle as f64 / scale;
            if Self::with_radius(self.guard, radius, self.pitch)
                .is_ok_and(|helix| helix.try_place(target, normal, fixed).is_some())
            {
                lower = middle;
            } else {
                upper = middle - 1;
            }
        }
        lower
    }

    fn try_place(&self, target: Point, normal: (f64, f64), fixed: bool) -> Option<Placement> {
        let radius = self.radius;
        let fits = |region: &Polygon, centre: Point| {
            region.contains(centre)
                && crate::frontadvance::boundary_dist(region, centre) + 1e-9 >= radius
        };
        let original = Point::new(target.x + normal.0 * radius, target.y + normal.1 * radius);
        for (region, centres) in self.guard.iter().zip(&self.centres) {
            if !region.contains(target) {
                continue;
            }
            if normal.0.hypot(normal.1) > 0.0 && fits(region, original) {
                return Some(Placement {
                    start: target,
                    normal,
                    transfer: Vec::new(),
                });
            }
            if let Some(normal) = fixed_normal(region, target, radius, |p| fits(region, p)) {
                return Some(Placement {
                    start: target,
                    normal,
                    transfer: Vec::new(),
                });
            }
            if fixed {
                continue;
            }
            let centre = centres
                .iter()
                .filter_map(|domain| {
                    // The nearest locus limits the transfer. Try the inset's interior
                    // when a point on its border only fits the relaxed offset.
                    let nearest = if domain.contains(target) {
                        target
                    } else {
                        boundaries(domain)
                            .map(|boundary| {
                                rotate_to_start(boundary, Some([target.x, target.y]))[0]
                            })
                            .min_by(|a, b| {
                                a.distance_sq(target).total_cmp(&b.distance_sq(target))
                            })?
                    };
                    if fits(region, nearest) {
                        return Some(nearest);
                    }
                    let mut inside = crate::frontadvance::entry_point(domain)
                        .filter(|&p| fits(region, p))
                        .or_else(|| {
                            boundaries(domain).flat_map(edges).find_map(|(a, b)| {
                                let p = midpoint(a, b);
                                fits(region, p).then_some(p)
                            })
                        })?;
                    let mut outside = nearest;
                    while inside.distance(outside) > GRID_MM {
                        let p = midpoint(outside, inside);
                        if fits(region, p) {
                            inside = p;
                        } else {
                            outside = p;
                        }
                    }
                    Some(inside)
                })
                .min_by(|a, b| a.distance_sq(target).total_cmp(&b.distance_sq(target)));
            if let Some(centre) = centre {
                let Some(route) = route_inside(region, centre, target) else {
                    continue;
                };
                let first = route[0];
                let (dx, dy) = unit(first.x - centre.x, first.y - centre.y);
                let (dx, dy) = if (dx, dy) == (0.0, 0.0) {
                    (1.0, 0.0)
                } else {
                    (dx, dy)
                };
                let start = Point::new(centre.x + dx * radius, centre.y + dy * radius);
                return Some(Placement {
                    start,
                    normal: (-dx, -dy),
                    transfer: route,
                });
            }
        }
        None
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn emit(
        &self,
        program: &mut Program,
        placement: &Placement,
        from_z: f64,
        z: f64,
        heights: &Heights,
        cutting_feed: f64,
        ramp_feed: f64,
        id: u32,
        local_from: Option<Point>,
    ) -> Result<(), RingsError> {
        let Placement {
            start,
            normal,
            transfer,
        } = placement;
        // Begin the helix at the stock surface or the previous cut floor.
        let top = if local_from.is_some() {
            from_z
        } else {
            from_z.min(heights.top_of_stock)
        };
        if top <= z {
            return Err(RingsError::Limit(
                "Helix must descend to the pocket floor".into(),
            ));
        }
        let arcs = 2.0 * ((top - z) / self.pitch).ceil().max(1.0);
        if arcs
            > MAX_PROFILE_MOVES.saturating_sub(program.len().saturating_add(transfer.len() + 3))
                as f64
        {
            return Err(RingsError::Limit(
                "Helix motion budget exceeded; increase helix pitch".into(),
            ));
        }
        let link = Tag::new(id, MoveKind::Link);
        if let Some(from) = local_from {
            program.push(Step::Linear {
                to: Point3::new(from.x, from.y, top),
                feed: cutting_feed,
                tag: link,
            });
            program.push(Step::Linear {
                to: Point3::new(start.x, start.y, top),
                feed: cutting_feed,
                tag: Tag::new(id, MoveKind::Cutting),
            });
        } else {
            program.push(Step::Rapid {
                to: Point3::new(start.x, start.y, heights.clearance),
                tag: link,
            });
            crate::emit::descend_to(program, *start, top, heights, cutting_feed, id);
        }
        crate::profile::emit_plunge(
            program,
            *start,
            (0.0, 0.0),
            *normal,
            top,
            z,
            Plunge::Helix {
                radius: self.radius,
                pitch: self.pitch,
            },
            ramp_feed,
            ramp_feed,
            Tag::new(id, MoveKind::Plunge),
        );
        let mut previous = *start;
        for &point in transfer {
            if previous.distance_sq(point) > 1e-18 {
                program.push(Step::Linear {
                    to: Point3::new(point.x, point.y, z),
                    feed: cutting_feed,
                    tag: Tag::new(id, MoveKind::Cutting),
                });
            }
            previous = point;
        }
        Ok(())
    }
}

fn midpoint(a: Point, b: Point) -> Point {
    Point::new(a.x + (b.x - a.x) * 0.5, a.y + (b.y - a.y) * 0.5)
}

fn boundaries(region: &Polygon) -> impl Iterator<Item = &[Point]> {
    std::iter::once(region.outer().points()).chain(region.holes().iter().map(|ring| ring.points()))
}

fn edges(points: &[Point]) -> impl Iterator<Item = (Point, Point)> + '_ {
    points
        .iter()
        .copied()
        .zip(points.iter().copied().cycle().skip(1))
}

/// Rotate the helix around its given start without moving that start. Clearance
/// can change only where a centre crosses a wall's radius-wide exclusion
/// zone: two offset edges and the round caps at the vertices.
fn fixed_normal(
    region: &Polygon,
    start: Point,
    radius: f64,
    fits: impl Fn(Point) -> bool,
) -> Option<(f64, f64)> {
    use std::f64::consts::TAU;
    let mut angles = vec![0.0, TAU];
    for (a, b) in boundaries(region).flat_map(edges) {
        if a.x.min(b.x) > start.x + 2.0 * radius
            || a.x.max(b.x) < start.x - 2.0 * radius
            || a.y.min(b.y) > start.y + 2.0 * radius
            || a.y.max(b.y) < start.y - 2.0 * radius
        {
            continue;
        }
        let (dx, dy) = unit(b.x - a.x, b.y - a.y);
        for side in [-radius, radius] {
            let shift = |p: Point| Point::new(p.x - dy * side, p.y + dx * side);
            angles.extend(circle_crossings(start, radius, shift(a), shift(b)));
        }
        let (x, y) = (a.x - start.x, a.y - start.y);
        let distance = x.hypot(y);
        if distance > 0.0 && distance <= 2.0 * radius {
            let angle = y.atan2(x);
            let delta = (distance / (2.0 * radius)).acos();
            angles.extend([
                (angle - delta).rem_euclid(TAU),
                (angle + delta).rem_euclid(TAU),
            ]);
        }
    }
    angles.sort_by(f64::total_cmp);
    angles.windows(2).find_map(|pair| {
        [pair[0], (pair[0] + pair[1]) * 0.5]
            .into_iter()
            .find_map(|angle| {
                let (sin, cos) = angle.sin_cos();
                fits(Point::new(start.x + radius * cos, start.y + radius * sin))
                    .then_some((cos, sin))
            })
    })
}

/// Directions from the start to intersections of its radius circle with a line segment.
fn circle_crossings(origin: Point, radius: f64, a: Point, b: Point) -> impl Iterator<Item = f64> {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let length = dx.hypot(dy);
    let (ux, uy) = unit(dx, dy);
    let (x, y) = (origin.x - a.x, origin.y - a.y);
    let along = x * ux + y * uy;
    let height = x * uy - y * ux;
    let span = (radius * radius - height * height).sqrt();
    [along - span, along + span]
        .into_iter()
        .filter(move |d| (0.0..=length).contains(d))
        .map(move |d| {
            (d * uy - y)
                .atan2(d * ux - x)
                .rem_euclid(std::f64::consts::TAU)
        })
}

/// Feed at depth inside one connected cutter-centre polygon. Any portion of
/// the straight segment outside the polygon is replaced by a walk around the
/// boundary it crossed (the pocket outline or the same island on both ends).
fn route_inside(region: &Polygon, from: Point, to: Point) -> Option<Vec<Point>> {
    let contained = |a, b| crate::linking::contained(std::slice::from_ref(region), a, b);
    if contained(from, to) {
        return Some(vec![to]);
    }
    let (dx, dy) = (to.x - from.x, to.y - from.y);
    let at = |t: f64| Point::new(from.x + t * dx, from.y + t * dy);
    let rings: Vec<_> = boundaries(region).collect();
    let mut crossings = Vec::new();
    for (ring, points) in rings.iter().enumerate() {
        let mut along = 0.0;
        for (a, b) in edges(points) {
            let (ex, ey) = (b.x - a.x, b.y - a.y);
            let length = ex.hypot(ey);
            let cross = dx * ey - dy * ex;
            if cross.abs() > 1e-12 {
                let (x, y) = (a.x - from.x, a.y - from.y);
                let t = (x * ey - y * ex) / cross;
                let u = (x * dy - y * dx) / cross;
                if (-1e-9..=1.0 + 1e-9).contains(&t) && (-1e-9..=1.0 + 1e-9).contains(&u) {
                    crossings.push((t.clamp(0.0, 1.0), ring, along + length * u.clamp(0.0, 1.0)));
                }
            }
            along += length;
        }
    }
    crossings.sort_by(|a, b| a.0.total_cmp(&b.0));
    crossings.dedup_by(|a, b| (a.0 - b.0).abs() <= 1e-9);
    let mut route = Vec::new();
    for gap in crossings.windows(2) {
        let [(a, ring_a, s), (b, ring_b, e)] = [gap[0], gap[1]];
        if region.contains(at((a + b) * 0.5)) {
            continue;
        }
        if ring_a != ring_b {
            return None;
        }
        let points = rings[ring_a];
        let perimeter = loop_perimeter(points);
        let forward = (e - s).rem_euclid(perimeter);
        if forward <= perimeter * 0.5 {
            route.extend(walk_loop(points, s, forward));
        } else {
            route.extend(walk_loop(points, e, perimeter - forward).into_iter().rev());
        }
    }
    route.push(to);
    std::iter::once(from)
        .chain(route.iter().copied())
        .zip(route.iter().copied())
        .all(|(a, b)| contained(a, b))
        .then_some(route)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_geo::Contour;

    fn contour(points: &[(f64, f64)]) -> Contour {
        Contour::new(points.iter().map(|&(x, y)| Point::new(x, y)).collect())
    }

    #[test]
    fn pocket_helix_moves_an_automatic_entry_out_of_a_narrow_extension() {
        let region = Polygon::new(contour(&[
            (0.0, 0.0),
            (20.0, 0.0),
            (20.0, 9.0),
            (40.0, 9.0),
            (40.0, 11.0),
            (20.0, 11.0),
            (20.0, 20.0),
            (0.0, 20.0),
        ]))
        .unwrap();
        let guard = [region];
        let helix = PocketHelix::new(
            &guard,
            Plunge::Helix {
                radius: 4.0,
                pitch: 2.0,
            },
        )
        .unwrap()
        .unwrap();
        let target = Point::new(37.0, 10.0);
        let automatic = helix.place(target, (-1.0, 0.0), false).unwrap();
        assert!(automatic.start.x < 21.0);
        assert_eq!(automatic.transfer.last(), Some(&target));
        let Err(RingsError::PocketHelixFit(fit)) = helix.place(target, (-1.0, 0.0), true) else {
            panic!("expected an error at the selected entry");
        };
        let message = fit.to_string();
        assert!(message.contains("at the selected pocket entry"));
        assert!(message.contains("The maximum fitting helix radius is 1.000 mm."));
    }

    #[test]
    fn pocket_helix_preserves_explicit_start_when_rotating_the_orbit_makes_it_fit() {
        let guard =
            [Polygon::new(contour(&[(0.0, 0.0), (5.0, 0.0), (5.0, 30.0), (0.0, 30.0)])).unwrap()];
        let helix = PocketHelix::new(
            &guard,
            Plunge::Helix {
                radius: 2.5,
                pitch: 1.0,
            },
        )
        .unwrap()
        .unwrap();
        let target = Point::new(2.0, 12.0);
        let placement = helix.place(target, (1.0, 0.0), true).unwrap();
        assert_eq!(placement.start, target);
        assert!(placement.transfer.is_empty());
        assert!((target.x + placement.normal.0 * 2.5 - 2.5).abs() < 1e-9);
    }

    #[test]
    fn pocket_helix_reports_a_radius_that_fits_at_display_precision() {
        let guard = [Polygon::new(contour(&[
            (0.0, 0.0),
            (5.0012, 0.0),
            (5.0012, 30.0),
            (0.0, 30.0),
        ]))
        .unwrap()];
        let target = Point::new(0.0, 12.34567);
        let helix = PocketHelix::with_radius(&guard, 3.0046, 1.0).unwrap();
        let Err(RingsError::PocketHelixFit(fit)) = helix.place(target, (1.0, 0.0), false) else {
            panic!("expected an error for the oversized helix");
        };
        let diagnostic = crate::Diagnostic::pocket_helix_fit(42, fit);
        let diagnostic: crate::Diagnostic =
            serde_json::from_str(&serde_json::to_string(&diagnostic).unwrap()).unwrap();
        // The geometric limit is 2.5006 mm, so ordinary rounding would suggest
        // an unusable setting at three decimal places.
        assert!(diagnostic
            .message
            .contains("The maximum fitting helix radius is 2.500 mm."));
        let coarse = diagnostic.display_message(0, |value| format!("{value:.0}"));
        assert!(coarse.contains("radius is 2 mm."), "{coarse}");
        let preferred = diagnostic.display_message(2, |value| format!("{value:.2}"));
        assert!(preferred.contains("operation 42:"));
        assert!(preferred.contains("radius 3.00 mm"));
        assert!(preferred.contains("(X 0.00, Y 12.35)"));
        assert!(preferred.contains("radius is 2.50 mm."));
        let fine = diagnostic.display_message(4, |value| format!("{value:.4}"));
        assert!(fine.contains("radius 3.0046 mm"));
        assert!(fine.contains("(X 0.0000, Y 12.3457)"));
        assert!(fine.contains("radius is 2.5006 mm."), "{fine}");
        let fitting = PocketHelix::with_radius(&guard, 2.5, 1.0).unwrap();
        assert!(fitting.place(target, (1.0, 0.0), false).is_ok());
    }

    #[test]
    fn pocket_helix_transfer_follows_a_contained_route_around_an_island() {
        let region = Polygon::with_holes(
            contour(&[(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)]),
            vec![contour(&[
                (10.0, 1.0),
                (12.0, 1.0),
                (12.0, 9.0),
                (10.0, 9.0),
            ])],
        )
        .unwrap();
        let from = Point::new(5.0, 5.0);
        let to = Point::new(15.0, 5.0);
        let route = route_inside(&region, from, to).unwrap();
        assert!(route.len() > 1);
        assert_eq!(route.last(), Some(&to));
        let mut previous = from;
        for point in route {
            assert!(crate::linking::contained(
                std::slice::from_ref(&region),
                previous,
                point
            ));
            previous = point;
        }
    }

    #[test]
    fn pocket_helix_rejects_invalid_parameters() {
        for (radius, pitch) in [(0.0, 1.0), (2.0, 0.0), (2.0, f64::NAN)] {
            assert!(PocketHelix::new(&[], Plunge::Helix { radius, pitch }).is_err());
        }
    }
}
