//! Configured leads and shared seams between adjoining adaptive cuts.
use cam_cldata::{MoveKind, Point3, Program, Step, Tag};
use cam_geo::{difference, offset, stroke_path, CapStyle, JoinStyle, Point, Polygon, Polyline};
use cam_model::{Heights, Lead};

use crate::{clearing::ClearJob, frontadvance::LinkedFrontier, leads};

#[derive(Clone, Debug)]
struct LeadPath {
    on: Point,
    off: Point,
    normal: (f64, f64),
    lead: Lead,
    samples: Vec<Point>,
}

impl LeadPath {
    fn new(on: Point, tangent: (f64, f64), lead: Lead, incoming: bool, side: f64) -> Self {
        let normal = (-tangent.1 * side, tangent.0 * side);
        let off = if incoming {
            leads::lead_start_point(on, tangent, normal, lead)
        } else {
            leads::lead_end_point(on, tangent, normal, lead)
        };
        let mut samples = match lead {
            Lead::None => vec![on],
            Lead::Linear { .. } => vec![on, off],
            Lead::Arc { radius } | Lead::ArcSweep { radius, .. } => {
                let angle = match lead {
                    Lead::ArcSweep { angle_deg, .. } => angle_deg.to_radians(),
                    _ => std::f64::consts::FRAC_PI_2,
                };
                let n = leads::arc_lead_density(radius, angle, 0.001);
                leads::lead_samples_n(on, off, normal, lead, n)
            }
        };
        if incoming {
            samples.reverse();
        }
        Self {
            on,
            off,
            normal,
            lead,
            samples,
        }
    }

    fn emit(&self, program: &mut Program, incoming: bool, z: f64, feed: f64, id: u32) {
        let (from, to) = if incoming {
            (self.off, self.on)
        } else {
            (self.on, self.off)
        };
        leads::emit_lead(
            program,
            from,
            to,
            self.on,
            self.normal,
            self.lead,
            z,
            feed,
            Tag::new(
                id,
                if incoming {
                    MoveKind::LeadIn
                } else {
                    MoveKind::LeadOut
                },
            ),
        );
    }
}

#[derive(Clone, Debug)]
struct Link {
    route: Vec<Point>,
    clearance: bool,
    departing: LeadPath,
    arriving: LeadPath,
    shared: bool,
    rapid_only: bool,
}

pub(crate) struct PreparedPath {
    sections: Vec<Vec<Point>>,
    links: Vec<Link>,
    entry: LeadPath,
    exit: LeadPath,
}

fn real_cut(path: &LinkedFrontier, index: usize) -> bool {
    path.moves[index].1 && !path.air[index] && !path.feed_links[index]
}

type SectionGap = (Vec<Point>, bool);

fn sections(path: &LinkedFrontier) -> (Vec<Vec<Point>>, Vec<SectionGap>) {
    let mut cuts = Vec::new();
    let mut gaps = Vec::new();
    let mut i = 1;
    let mut last_end = 0;
    while i < path.moves.len() {
        if !real_cut(path, i) {
            i += 1;
            continue;
        }
        let first = i - 1;
        if !cuts.is_empty() {
            gaps.push((
                path.moves[last_end..=first]
                    .iter()
                    .map(|&(p, _)| p)
                    .collect(),
                path.moves[last_end + 1..=first]
                    .iter()
                    .any(|&(_, cutting)| !cutting),
            ));
        }
        let mut points = vec![path.moves[first].0];
        while i < path.moves.len() && real_cut(path, i) {
            if points.last() != Some(&path.moves[i].0) {
                points.push(path.moves[i].0);
            }
            i += 1;
        }
        last_end = i - 1;
        cuts.push(points);
    }
    (cuts, gaps)
}

fn tangent(points: &[Point], incoming: bool) -> (f64, f64) {
    if incoming {
        points
            .windows(2)
            .find_map(|p| {
                let t = crate::profile::unit(p[1].x - p[0].x, p[1].y - p[0].y);
                (t != (0., 0.)).then_some(t)
            })
            .unwrap_or((1., 0.))
    } else {
        points
            .windows(2)
            .rev()
            .find_map(|p| {
                let t = crate::profile::unit(p[1].x - p[0].x, p[1].y - p[0].y);
                (t != (0., 0.)).then_some(t)
            })
            .unwrap_or((1., 0.))
    }
}

/// A common point on two segments, including collinear shared wall segments.
fn shared_point(a: Point, b: Point, c: Point, d: Point) -> Option<Point> {
    let cross = |u: (f64, f64), v: (f64, f64)| u.0 * v.1 - u.1 * v.0;
    let u = (b.x - a.x, b.y - a.y);
    let v = (d.x - c.x, d.y - c.y);
    let denominator = cross(u, v);
    let ac = (c.x - a.x, c.y - a.y);
    if denominator.abs() > 1e-12 {
        let t = cross(ac, v) / denominator;
        let s = cross(ac, u) / denominator;
        if (0. ..=1.).contains(&t) && (0. ..=1.).contains(&s) {
            return Some(if t == 0. {
                a
            } else if t == 1. {
                b
            } else {
                Point::new(a.x + t * u.0, a.y + t * u.1)
            });
        }
    } else if cross(ac, u).abs() <= 1e-10 && u.0 * v.0 + u.1 * v.1 > 0. {
        let within = |p: Point, x: Point, y: Point| {
            let t = (p.x - x.x) * (y.x - x.x) + (p.y - x.y) * (y.y - x.y);
            let length2 = (y.x - x.x).powi(2) + (y.y - x.y).powi(2);
            length2 > 1e-18 && t >= -1e-12 && t <= length2 + 1e-12
        };
        if within(b, c, d) {
            return Some(b);
        }
        if within(c, a, b) {
            return Some(c);
        }
    }
    None
}

/// Trim only genuinely adjoining curves. Separate pocket ends are not a seam.
fn join_at_seam(
    previous: &mut Vec<Point>,
    next: &mut Vec<Point>,
    reach: f64,
    tolerance: f64,
) -> bool {
    let seam = *previous.last().unwrap();
    if seam.distance(next[0]) <= tolerance {
        next[0] = seam;
        return true;
    }
    let mut best = None;
    let mut walked_out = 0.;
    for i in (0..previous.len().saturating_sub(1)).rev() {
        let mut walked_in = 0.;
        for j in 0..next.len().saturating_sub(1) {
            if let Some(p) = shared_point(previous[i], previous[i + 1], next[j], next[j + 1]) {
                let cost =
                    walked_out + p.distance(previous[i + 1]) + walked_in + next[j].distance(p);
                if cost <= reach && best.is_none_or(|(_, _, _, score)| cost < score) {
                    best = Some((i, j, p, cost));
                }
            }
            walked_in += next[j].distance(next[j + 1]);
            if walked_in > reach {
                break;
            }
        }
        walked_out += previous[i].distance(previous[i + 1]);
        if walked_out > reach {
            break;
        }
    }
    let Some((i, j, seam, _)) = best else {
        return false;
    };
    previous.truncate(i + 1);
    if previous.last() != Some(&seam) {
        previous.push(seam);
    }
    let mut remaining = vec![seam];
    remaining.extend(next[j + 1..].iter().copied().filter(|&p| p != seam));
    *next = remaining;
    true
}

fn as_moves(sections: &[Vec<Point>]) -> Vec<(Point, bool)> {
    sections
        .iter()
        .flat_map(|section| section.iter().enumerate().map(|(i, &p)| (p, i != 0)))
        .collect()
}

fn sweep(points: &[Point], radius: f64) -> Option<Vec<Polygon>> {
    stroke_path(
        &Polyline::new(points.to_vec()),
        radius,
        CapStyle::Round,
        JoinStyle::Round,
    )
    .ok()
}

fn clear_path(points: &[Point], cleared: &[Polygon], radius: f64) -> bool {
    sweep(points, radius)
        .and_then(|s| difference(&s, cleared).ok())
        .is_some_and(|remaining| remaining.iter().map(Polygon::area).sum::<f64>() <= 0.005)
}

pub(crate) fn prepare(
    path: &LinkedFrontier,
    region: &Polygon,
    job: &ClearJob,
) -> Result<Option<PreparedPath>, &'static str> {
    if job.end.is_some() {
        return if job.lead_in == Lead::None && job.lead_out == Lead::None && job.lead_overlap == 0.
        {
            Ok(None)
        } else {
            Err("adaptive seam or lead is not safe")
        };
    }
    let (mut sections, gaps) = sections(path);
    if sections.is_empty() || sections.iter().any(|s| s.len() < 2) {
        return Err("adaptive seam or lead is not safe");
    }
    let mut shared = vec![false; gaps.len()];
    for i in 0..gaps.len() {
        if !gaps[i].1 {
            let (before, after) = sections.split_at_mut(i + 1);
            shared[i] = join_at_seam(
                &mut before[i],
                &mut after[0],
                2. * job.radius,
                job.simplification_tolerance.clamp(cam_geo::GRID_MM, 0.02),
            );
        }
    }
    if sections.iter().any(|s| s.len() < 2) {
        return Err("adaptive seam or lead is not safe");
    }
    if job.lead_in == Lead::None
        && job.lead_out == Lead::None
        && job.lead_overlap == 0.
        && !shared.iter().any(|&b| b)
    {
        return Ok(None);
    }
    if !job.lead_overlap.is_finite() || job.lead_overlap < 0. {
        return Err("adaptive seam or lead is not safe");
    }
    for i in 0..gaps.len() {
        if shared[i] && job.lead_overlap > 0. {
            let length: f64 = sections[i + 1]
                .windows(2)
                .map(|p| p[0].distance(p[1]))
                .sum();
            if job.lead_overlap > length {
                return Err("adaptive seam or lead is not safe");
            }
            let overlap = crate::profile::advance_along_path(&sections[i + 1], job.lead_overlap);
            sections[i].extend(overlap.into_iter().skip(1));
        }
    }
    let material = offset(std::slice::from_ref(region), -job.finish, JoinStyle::Round)
        .map_err(|_| "adaptive lead geometry failed")?;
    let material = material
        .first()
        .ok_or("adaptive lead does not fit cleared stock")?;
    let actual = as_moves(&sections);
    let original: Vec<_> = path
        .moves
        .iter()
        .enumerate()
        .map(|(i, &(p, cut))| (p, cut && !path.air[i] && !path.feed_links[i]))
        .collect();
    if !crate::frontadvance::preserves_swept_material(
        &original, &actual, job.radius, material, 0.005,
    ) {
        return Err("adaptive seam or lead is not safe");
    }
    let guard = offset(
        std::slice::from_ref(material),
        -job.radius,
        JoinStyle::Round,
    )
    .map_err(|_| "adaptive lead geometry failed")?;
    let first = sections[0][0];
    // The initial entry into solid uses the selected plunge strategy. These
    // leads govern subsequent approaches, where cleared stock already exists.
    let entry = LeadPath::new(first, tangent(&sections[0], true), Lead::None, true, 1.);
    let mut cleared = Vec::new();
    let mut links = Vec::new();
    for i in 0..sections.len() {
        cleared.extend(
            sweep(&sections[i], job.radius).ok_or("adaptive lead does not fit cleared stock")?,
        );
        if i == gaps.len() {
            break;
        }
        let departing = [1., -1.]
            .into_iter()
            .map(|side| {
                LeadPath::new(
                    *sections[i].last().unwrap(),
                    tangent(&sections[i], false),
                    job.lead_out,
                    false,
                    side,
                )
            })
            .find(|lead| lead.lead == Lead::None || clear_path(&lead.samples, &cleared, job.radius))
            .ok_or("adaptive departing lead does not fit cleared stock")?;
        let arriving = [1., -1.]
            .into_iter()
            .map(|side| {
                LeadPath::new(
                    sections[i + 1][0],
                    tangent(&sections[i + 1], true),
                    job.lead_in,
                    true,
                    side,
                )
            })
            .find(|lead| lead.lead == Lead::None || clear_path(&lead.samples, &cleared, job.radius))
            .ok_or("adaptive arriving lead does not fit cleared stock")?;
        let direct = vec![departing.off, arriving.off];
        let rapid_only = [departing.off, arriving.off].iter().all(|&p| {
            guard.iter().any(|g| {
                g.contains(p) && crate::frontadvance::boundary_dist(g, p) >= 0.6 - cam_geo::GRID_MM
            })
        });
        let route = if clear_path(&direct, &cleared, job.radius) {
            direct
        } else {
            let mut route = vec![departing.off];
            route.extend(
                gaps[i]
                    .0
                    .iter()
                    .skip(1)
                    .take(gaps[i].0.len().saturating_sub(2))
                    .copied(),
            );
            route.push(arriving.off);
            if !gaps[i].1 && !clear_path(&route, &cleared, job.radius) {
                return Err("adaptive seam or lead is not safe");
            }
            route
        };
        links.push(Link {
            route,
            clearance: gaps[i].1,
            departing,
            arriving,
            shared: shared[i],
            rapid_only,
        });
    }
    let exit_at = |last: &[Point]| {
        [1., -1.]
            .into_iter()
            .map(|side| {
                LeadPath::new(
                    *last.last().unwrap(),
                    tangent(last, false),
                    job.lead_out,
                    false,
                    side,
                )
            })
            .find(|lead| leads::lead_fits(&guard, &lead.samples))
    };
    let mut exit = exit_at(sections.last().unwrap());
    if exit.is_none() {
        let last = sections.last_mut().unwrap();
        let end = *last.last().unwrap();
        let extent = match job.lead_out {
            Lead::Linear { length } => length,
            Lead::Arc { radius } | Lead::ArcSweep { radius, .. } => radius,
            Lead::None => 0.,
        };
        // A sharp internal corner has no room for a tangent departure. Finish
        // the corner, then continue along its wall to a legal exit point.
        'walls: for polygon in &guard {
            for contour in std::iter::once(polygon.outer()).chain(polygon.holes().iter()) {
                let points = contour.points();
                for i in 0..points.len() {
                    let a = points[i];
                    let b = points[(i + 1) % points.len()];
                    let (dx, dy) = (b.x - a.x, b.y - a.y);
                    let length2 = dx * dx + dy * dy;
                    if length2 <= 1e-18 {
                        continue;
                    }
                    let t = (((end.x - a.x) * dx + (end.y - a.y) * dy) / length2).clamp(0., 1.);
                    let on = Point::new(a.x + t * dx, a.y + t * dy);
                    if on.distance(end) > job.simplification_tolerance.max(cam_geo::GRID_MM) {
                        continue;
                    }
                    for toward in [a, b] {
                        let available = on.distance(toward);
                        if available < extent || available < cam_geo::GRID_MM {
                            continue;
                        }
                        let q = Point::new(
                            on.x + (toward.x - on.x) * extent / available,
                            on.y + (toward.y - on.y) * extent / available,
                        );
                        last.push(q);
                        exit = exit_at(last);
                        if exit.is_some() {
                            break 'walls;
                        }
                        last.pop();
                    }
                }
            }
        }
    }
    let exit = exit.ok_or("adaptive final lead does not fit cleared stock")?;
    let prepared = PreparedPath {
        sections,
        links,
        entry,
        exit,
    };
    let verdict =
        crate::clearsim::certify_moves(&prepared.certification_moves(), job.radius, material);
    let reachable = crate::clearsim::reachable(material, job.radius);
    let coverage_limit = 0.02 * reachable.iter().map(Polygon::area).sum::<f64>() + 1.;
    if verdict.max_engagement
        > crate::frontadvance::engagement_bound(job.clearing.engagement, job.radius)
        || verdict.uncut_area > coverage_limit
        || verdict.gouge_area > coverage_limit
    {
        return Err("configured adaptive path failed its material-removal certificate");
    }
    Ok(Some(prepared))
}

impl PreparedPath {
    fn certification_moves(&self) -> Vec<(Point, bool)> {
        let mut moves = vec![(self.entry.off, false)];
        moves.extend(self.entry.samples.iter().skip(1).map(|&p| (p, true)));
        for i in 0..self.sections.len() {
            moves.extend(self.sections[i].iter().skip(1).map(|&p| (p, true)));
            if let Some(link) = self.links.get(i) {
                moves.extend(link.departing.samples.iter().skip(1).map(|&p| (p, true)));
                moves.extend(link.route.iter().skip(1).map(|&p| (p, false)));
                moves.extend(link.arriving.samples.iter().skip(1).map(|&p| (p, true)));
            }
        }
        moves.extend(self.exit.samples.iter().skip(1).map(|&p| (p, true)));
        moves
    }

    pub(crate) fn emit(
        &self,
        program: &mut Program,
        job: &ClearJob,
        heights: &Heights,
        levels: &[f64],
    ) -> Result<(), crate::rings::RingsError> {
        let link = Tag::new(job.id, MoveKind::Link);
        let plunge = Tag::new(job.id, MoveKind::Plunge);
        let entry = self.entry.off;
        let placement = job
            .helix
            .map(|helix| helix.place(entry, self.entry.normal, job.start.is_some()))
            .transpose()?;
        let mut from_z = heights.retract.max(heights.top_of_stock);
        for &z in levels {
            if let Some((helix, placement)) = job.helix.zip(placement.as_ref()) {
                helix.emit(
                    program,
                    placement,
                    from_z,
                    z,
                    heights,
                    job.feed,
                    job.entry_feeds.ramp(job.feed),
                    job.id,
                    None,
                )?;
            } else {
                program.push(Step::Rapid {
                    to: Point3::new(entry.x, entry.y, heights.clearance),
                    tag: link,
                });
                crate::emit::descend_to(program, entry, from_z, heights, job.feed, job.id);
                let tan = tangent(&self.sections[0], true);
                crate::profile::emit_plunge(
                    program,
                    entry,
                    tan,
                    self.entry.normal,
                    from_z,
                    z,
                    leads::guard_plunge(job.guard, entry, self.entry.normal, job.plunge),
                    job.plunge_feed,
                    job.entry_feeds.ramp(job.feed),
                    plunge,
                );
            }
            self.entry
                .emit(program, true, z, job.entry_feeds.lead_in(job.feed), job.id);
            for i in 0..self.sections.len() {
                for p in self.sections[i].iter().skip(1) {
                    program.push(Step::Linear {
                        to: Point3::new(p.x, p.y, z),
                        feed: job.feed,
                        tag: Tag::new(job.id, MoveKind::Cutting),
                    });
                }
                let Some(transition) = self.links.get(i) else {
                    break;
                };
                debug_assert!(
                    !transition.shared
                        || job.lead_overlap > 0.
                        || transition.departing.on == transition.arriving.on
                );
                let raised = if transition.clearance {
                    heights.clearance
                } else {
                    (z + job.lift_height.max(0.2)).min(heights.clearance)
                };
                transition.departing.emit(
                    program,
                    false,
                    raised,
                    job.entry_feeds.lead_out(job.feed),
                    job.id,
                );
                if transition.departing.lead == Lead::None {
                    let p = transition.departing.on;
                    program.push(Step::Rapid {
                        to: Point3::new(p.x, p.y, raised),
                        tag: link,
                    });
                }
                for (j, &p) in transition.route.iter().enumerate().skip(1) {
                    let to = Point3::new(p.x, p.y, raised);
                    if !transition.rapid_only && (j == 1 || j + 1 == transition.route.len()) {
                        program.push(Step::Linear {
                            to,
                            feed: job.feed,
                            tag: link,
                        });
                    } else {
                        program.push(Step::Rapid { to, tag: link });
                    }
                }
                transition.arriving.emit(
                    program,
                    true,
                    z,
                    job.entry_feeds.lead_in(job.feed),
                    job.id,
                );
                if transition.arriving.lead == Lead::None {
                    let p = transition.arriving.on;
                    program.push(Step::Linear {
                        to: Point3::new(p.x, p.y, z),
                        feed: job.plunge_feed,
                        tag: plunge,
                    });
                }
            }
            self.exit.emit(
                program,
                false,
                z,
                job.entry_feeds.lead_out(job.feed),
                job.id,
            );
            let end = self.exit.off;
            program.push(Step::Rapid {
                to: Point3::new(end.x, end.y, heights.clearance),
                tag: Tag::new(job.id, MoveKind::Retract),
            });
            from_z = z;
        }
        Ok(())
    }
}

/// A lifted transition over a verified flat floor. The arcs rise from or
/// descend to the cutting plane, keeping the connection tangent in XY.
pub(crate) struct FloorLink {
    pub start: usize,
    pub end: usize,
    departure: LeadPath,
    arrival: LeadPath,
    route: Vec<Point>,
}

impl FloorLink {
    pub(crate) fn emit(&self, program: &mut Program, job: &ClearJob, z: f64, raised: f64) {
        let tag = Tag::new(job.id, MoveKind::Link);
        if self.departure.lead == Lead::None {
            let p = self.departure.on;
            program.push(Step::Rapid {
                to: Point3::new(p.x, p.y, raised),
                tag,
            });
        } else {
            self.departure.emit(
                program,
                false,
                raised,
                job.entry_feeds.lead_out(job.feed),
                job.id,
            );
        }
        for &p in &self.route[1..] {
            program.push(Step::Rapid {
                to: Point3::new(p.x, p.y, raised),
                tag,
            });
        }
        if self.arrival.lead == Lead::None {
            let p = self.arrival.on;
            program.push(Step::Linear {
                to: Point3::new(p.x, p.y, z),
                feed: job.plunge_feed,
                tag: Tag::new(job.id, MoveKind::Plunge),
            });
        } else {
            self.arrival
                .emit(program, true, z, job.entry_feeds.lead_in(job.feed), job.id);
        }
    }
}

pub(crate) fn floor_links(
    path: &LinkedFrontier,
    job: &ClearJob,
    verified_air: bool,
    cancel: &crate::CancelToken,
) -> Vec<FloorLink> {
    use crate::frontadvance::ClearedFootprint;

    let Some(&first) = path.moves.first() else {
        return Vec::new();
    };
    let mut links = Vec::new();
    let mut footprint = ClearedFootprint::bounded();
    let mut prefix = LinkedFrontier {
        moves: vec![first],
        air: vec![false],
        feed_links: vec![false],
    };
    let mut i = 1;
    while i < path.moves.len() && !cancel.is_cancelled() {
        if !path.air.get(i).copied().unwrap_or(false) {
            prefix.moves.push(path.moves[i]);
            prefix.air.push(false);
            prefix
                .feed_links
                .push(path.feed_links.get(i).copied().unwrap_or(false));
            i += 1;
            continue;
        }
        let start = i;
        while i < path.moves.len() && path.air.get(i).copied().unwrap_or(false) {
            i += 1;
        }
        let end = i - 1;
        let from = path.moves[start - 1].0;
        let to = path.moves[end].0;
        let outgoing = path.moves[..start - 1]
            .iter()
            .rev()
            .find_map(|&(p, _)| {
                (p != from).then(|| crate::profile::unit(from.x - p.x, from.y - p.y))
            })
            .unwrap_or((1., 0.));
        let incoming = path.moves[i..]
            .iter()
            .find_map(|&(p, _)| (p != to).then(|| crate::profile::unit(p.x - to.x, p.y - to.y)))
            .unwrap_or((1., 0.));

        let lead_options = |p, direction, lead, incoming| {
            let mut options = Vec::new();
            if lead != Lead::None {
                options.extend(
                    [1., -1.].map(|side| LeadPath::new(p, direction, lead, incoming, side)),
                );
            }
            options.push(LeadPath::new(p, direction, Lead::None, incoming, 1.));
            options
        };
        let departures = lead_options(from, outgoing, job.lead_out, false);
        let arrivals = lead_options(to, incoming, job.lead_in, true);
        let mut chosen = None;
        'arcs: for count in (0..=2).rev() {
            for departure in &departures {
                for arrival in &arrivals {
                    let arcs = usize::from(departure.lead != Lead::None)
                        + usize::from(arrival.lead != Lead::None);
                    if arcs != count {
                        continue;
                    }
                    let mut route = vec![departure.off];
                    route.extend(path.moves[start..end].iter().map(|&(p, _)| p));
                    route.push(arrival.off);
                    let mut sweep = departure.samples.clone();
                    sweep.extend_from_slice(&route[1..]);
                    sweep.extend_from_slice(&arrival.samples[1..]);
                    if (arcs == 0 && verified_air)
                        || footprint
                            .covers_route(&prefix, &sweep, job.radius)
                            .unwrap_or(false)
                    {
                        chosen = Some(FloorLink {
                            start,
                            end,
                            departure: departure.clone(),
                            arrival: arrival.clone(),
                            route,
                        });
                        break 'arcs;
                    }
                }
            }
        }
        let air = chosen.is_some();
        links.extend(chosen);
        prefix.moves.extend_from_slice(&path.moves[start..i]);
        prefix.air.resize(prefix.moves.len(), air);
        prefix.feed_links.resize(prefix.moves.len(), false);
    }
    links
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_cldata::SpindleDir;
    use cam_geo::Contour;
    use cam_model::{Clearing, EntryFeeds, Plunge};

    fn job(lead_in: Lead, lead_out: Lead, overlap: f64) -> ClearJob<'static> {
        ClearJob {
            lift_height: 0.2,
            id: 7,
            simplification_tolerance: 0.02,
            radius: 3.,
            finish: 0.,
            first: 3.,
            spacing: 2.4,
            clearing: Clearing {
                engagement: 2.4,
                climb: true,
            },
            spindle: SpindleDir::Cw,
            plunge: Plunge::Straight,
            feed: 300.,
            plunge_feed: 100.,
            entry_feeds: EntryFeeds {
                ramp: None,
                lead_in: Some(67.),
                lead_out: Some(89.),
            },
            lead_overlap: overlap,
            lead_in,
            lead_out,
            start: None,
            end: None,
            guard: &[],
            helix: None,
        }
    }

    fn fixture(adjoining: bool) -> (LinkedFrontier, Polygon) {
        let p = |x, y| Point::new(x, y);
        let region = Polygon::new(Contour::new(vec![
            p(0., 0.),
            p(40., 0.),
            p(40., 40.),
            p(0., 40.),
        ]))
        .unwrap();
        // Certify the clearing prefix too: a raster slot would not be a valid
        // adaptive input even though it creates room for the following leads.
        let mut moves =
            crate::frontadvance::front_advance_certified(&region, 3., 0., 2.4, None, 0.02).unwrap();
        moves.extend([(p(15., 15.), true), (p(25., 15.), true)]);
        let mut air = vec![false; moves.len()];
        let mut feed_links = air.clone();
        let next = if adjoining { p(22., 15.) } else { p(30., 30.) };
        moves.extend([(p(25., 20.), true), (p(next.x, 20.), true), (next, true)]);
        air.extend([false, true, false]);
        feed_links.extend([true, false, true]);
        let tail = if adjoining {
            vec![p(25.5, 15.), p(25.5, 17.), p(30., 17.), p(20., 25.)]
        } else {
            vec![p(32., 30.), p(32., 25.), p(20., 25.)]
        };
        moves.extend(tail.into_iter().map(|p| (p, true)));
        air.resize(moves.len(), false);
        feed_links.resize(moves.len(), false);
        (
            LinkedFrontier {
                moves,
                air,
                feed_links,
            },
            region,
        )
    }

    #[test]
    fn cleared_floor_link_rises_and_descends_on_the_configured_arcs() {
        let p = Point::new;
        let mut path = LinkedFrontier {
            moves: vec![
                (p(5., 10.), false),
                (p(35., 10.), true),
                (p(35., 13.), true),
                (p(5., 13.), true),
                (p(5., 16.), true),
                (p(35., 16.), true),
                (p(12., 13.), true),
                (p(13., 13.), true),
                (p(27., 13.), true),
                (p(28., 13.), true),
            ],
            air: vec![false; 10],
            feed_links: vec![false; 10],
        };
        path.air[8] = true;
        let configured = job(Lead::Arc { radius: 0.7 }, Lead::Arc { radius: 0.4 }, 0.);
        let links = floor_links(&path, &configured, false, &crate::CancelToken::new());
        assert_eq!(links.len(), 1);
        let tag = Tag::new(configured.id, MoveKind::Cutting);
        let mut program = Program::new();
        program.push(Step::Linear {
            to: Point3::new(13., 13., -2.),
            feed: 300.,
            tag,
        });
        links[0].emit(&mut program, &configured, -2., -1.8);
        assert!(program.validate().is_ok());
        let motions = program.steps();
        assert!(
            matches!(motions[1], Step::Arc { end, center, feed: 89., tag, .. }
            if tag.kind == MoveKind::LeadOut && end.z == -1.8 && ((center.x - 13.).hypot(center.y - 13.) - 0.4).abs() < 1e-9)
        );
        assert!(matches!(motions[2], Step::Rapid { to, .. } if to.z == -1.8));
        assert!(
            matches!(motions[3], Step::Arc { end, center, feed: 67., tag, .. }
            if tag.kind == MoveKind::LeadIn && end == Point3::new(27., 13., -2.) && ((center.x - 27.).hypot(center.y - 13.) - 0.7).abs() < 1e-9)
        );
        path.moves[8].0 = p(27., 30.);
        assert!(floor_links(&path, &configured, false, &crate::CancelToken::new()).is_empty());
    }

    #[test]
    fn adjoining_cuts_share_the_identical_seam_with_zero_overlap() {
        let (path, region) = fixture(true);
        for lead in [
            Lead::None,
            Lead::Arc { radius: 0.6 },
            Lead::ArcSweep {
                radius: 0.6,
                angle_deg: 120.,
            },
            Lead::Linear { length: 0.6 },
        ] {
            let job = job(lead, lead, 0.);
            let prepared = prepare(&path, &region, &job).unwrap().unwrap();
            assert!(prepared.links[0].shared);
            let seam = *prepared.sections[0].last().unwrap();
            assert_eq!(seam, prepared.sections[1][0]);
            assert_eq!(prepared.links[0].departing.on, seam);
            assert_eq!(prepared.links[0].arriving.on, seam);
            let mut program = Program::new();
            prepared
                .emit(&mut program, &job, &Heights::new(5., 2., 0.), &[-2., -4.])
                .unwrap();
            program.validate().unwrap();
            let mut last_cut = None;
            let mut reentries = 0;
            for step in program.steps() {
                match step {
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => {
                        last_cut = Some(*to)
                    }
                    Step::Arc { feed, tag, .. } | Step::Linear { feed, tag, .. }
                        if tag.kind == MoveKind::LeadIn =>
                    {
                        assert_eq!(*feed, 67.)
                    }
                    Step::Arc { feed, tag, .. } | Step::Linear { feed, tag, .. }
                        if tag.kind == MoveKind::LeadOut =>
                    {
                        assert_eq!(*feed, 89.)
                    }
                    _ => {}
                }
                if let Step::Arc { end, tag, .. } | Step::Linear { to: end, tag, .. } = step {
                    if tag.kind == MoveKind::LeadIn {
                        assert_eq!(
                            Some(*end),
                            last_cut,
                            "the emitted re-entry must return to the exact cut endpoint"
                        );
                        reentries += 1;
                    }
                }
            }
            if lead != Lead::None {
                assert_eq!(reentries, 2);
            }
            let sim = cam_sim::simulate(
                &program,
                [-1., -1., -6.],
                [41., 41., 0.],
                &cam_sim::SimOptions {
                    resolution: 0.25,
                    tool_radius: 3.,
                },
                &[cam_sim::SimTool {
                    number: 1,
                    profile: cam_sim::ToolProfile::flat(3.),
                    noncutting_tip_radius: None,
                }],
            );
            assert!(sim.is_clean(), "{lead:?}: {:?}", sim.collisions);
            if matches!(lead, Lead::Arc { .. } | Lead::ArcSweep { .. }) {
                assert!(program
                    .steps()
                    .iter()
                    .any(|s| matches!(s, Step::Arc { end, tag, .. }
                    if tag.kind == MoveKind::LeadIn && *end == Point3::new(seam.x, seam.y, -2.))));
            }
        }
    }

    #[test]
    fn overlap_is_the_requested_path_distance_even_across_a_corner() {
        let (path, region) = fixture(true);
        let prepared = prepare(
            &path,
            &region,
            &job(Lead::Arc { radius: 0.6 }, Lead::Arc { radius: 0.6 }, 1.25),
        )
        .unwrap()
        .unwrap();
        let seam = prepared.sections[1][0];
        assert_eq!(seam, Point::new(25., 15.));
        assert_eq!(
            *prepared.sections[0].last().unwrap(),
            Point::new(25.5, 15.75)
        );
        let seam_index = prepared.sections[0]
            .iter()
            .rposition(|&p| p == seam)
            .unwrap();
        let length: f64 = prepared.sections[0][seam_index..]
            .windows(2)
            .map(|p| p[0].distance(p[1]))
            .sum();
        assert!((length - 1.25).abs() < 1e-12);
    }

    #[test]
    fn separate_regions_keep_distinct_arrival_and_departure_points() {
        let (path, region) = fixture(false);
        let prepared = prepare(
            &path,
            &region,
            &job(Lead::Arc { radius: 0.6 }, Lead::Arc { radius: 0.6 }, 0.),
        )
        .unwrap()
        .unwrap();
        assert!(!prepared.links[0].shared);
        assert_eq!(*prepared.sections[0].last().unwrap(), Point::new(25., 15.));
        assert_eq!(prepared.sections[1][0], Point::new(30., 30.));
    }

    #[test]
    fn independently_computed_junctions_reuse_one_exact_point() {
        let seam = Point::new(25.123456789, 7.);
        let mut before = vec![Point::new(20., 7.), seam];
        let mut after = vec![Point::new(seam.x + 1e-12, 7.), Point::new(30., 7.)];
        assert!(join_at_seam(&mut before, &mut after, 6., cam_geo::GRID_MM));
        assert_eq!(*before.last().unwrap(), after[0]);
        assert_eq!(after[0], seam);
    }

    #[test]
    fn small_wall_leads_keep_their_radius_and_use_a_guarded_feed_bridge() {
        let (mut path, region) = fixture(true);
        let first = path.feed_links.iter().position(|&b| b).unwrap();
        for (index, point) in [
            (first - 2, Point::new(15., 37.)),
            (first - 1, Point::new(25., 37.)),
            (first, Point::new(25., 36.4)),
            (first + 1, Point::new(22., 36.4)),
            (first + 2, Point::new(22., 37.)),
            (first + 3, Point::new(25.5, 37.)),
            (first + 4, Point::new(25.5, 35.)),
            (first + 5, Point::new(30., 35.)),
        ] {
            path.moves[index].0 = point;
        }
        let configured = job(Lead::Arc { radius: 0.2 }, Lead::Arc { radius: 0.2 }, 0.);
        let prepared = prepare(&path, &region, &configured).unwrap().unwrap();
        assert!(prepared.links[0].shared);
        assert!(!prepared.links[0].rapid_only);
        assert_eq!(prepared.links[0].route.len(), 2);
        assert_eq!(prepared.links[0].arriving.lead, Lead::Arc { radius: 0.2 });
        assert_eq!(prepared.links[0].departing.lead, Lead::Arc { radius: 0.2 });
    }

    #[test]
    fn reference_pocket_keeps_its_configured_adaptive_leads() {
        let region = Polygon::new(Contour::new(vec![
            Point::new(10., -10.),
            Point::new(40., -10.),
            Point::new(40., 10.),
            Point::new(10., 10.),
        ]))
        .unwrap();
        let raw =
            crate::frontadvance::front_advance_certified(&region, 3., 0., 2.4, None, 0.02).unwrap();
        let linked = crate::frontadvance::link_cleared_travel(
            &raw,
            &region,
            3.,
            0.,
            2.4,
            &crate::CancelToken::new(),
            None,
        )
        .unwrap();
        let configured = job(Lead::Arc { radius: 0.6 }, Lead::Arc { radius: 0.6 }, 0.);
        let prepared = prepare(&linked, &region, &configured).unwrap().unwrap();
        assert!(prepared.links.len() >= 4);
    }

    #[test]
    fn a_configured_lead_cannot_cross_untouched_stock() {
        let (path, region) = fixture(true);
        assert!(prepare(
            &path,
            &region,
            &job(Lead::Arc { radius: 50. }, Lead::Arc { radius: 50. }, 0.)
        )
        .is_err());
    }
}
