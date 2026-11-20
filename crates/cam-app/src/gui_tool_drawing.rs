//! Dimensioned cutter profiles shared by the library editor and operation info.
use super::*;
use iced::widget::{column, row};

pub(super) struct ToolCanvas {
    tool: cam_model::Tool,
    selected: Option<Field>,
    compact: bool,
    profile: cam_geo::Profile2D,
    flute_length: f64,
    flutes: u32,
    cutting_direction: CutDir,
    /// Whether to draw the flute helix at all (off for V-bits).
    draw_flutes: bool,
    /// Axial advance per helix turn (drill bits ≈ 3·⌀ for a realistic pitch; other
    /// kinds do one turn over the cutting region).
    flute_pitch: f64,
    /// Sign of the helix lean (+1 leans one way, −1 the other). Derived from the
    /// cutting direction, but inverted for a twist drill so it reads right-hand.
    flute_sign: f64,
    /// Draw a row of 90° square inserts seated at the bottom of the body (face mills),
    /// so the silhouette reads as a real shell mill rather than a bare inverted-T.
    draw_face_inserts: bool,
}

impl ToolCanvas {
    pub(super) fn new(tool: &cam_model::Tool) -> Self {
        let profile = tool.profile();
        // The flute helix spans the actual cutting region — the flute length for an end
        // mill, the cone for a V-bit / drill — read off the profile's cutting segments.
        let cutting_top = profile
            .segs
            .iter()
            .filter(|s| s.cutting)
            .map(|s| s.end.y)
            .fold(0.0_f64, f64::max);
        let cutting_top = cutting_top.max(1e-3);
        Self {
            tool: tool.clone(),
            selected: None,
            compact: false,
            flute_length: cutting_top,
            flutes: tool.flutes,
            cutting_direction: tool.cutting_direction,
            // V-bits, chamfer mills, face mills and thread mills show no flute helix (the
            // cone / shell-mill body / thread teeth read cleaner without it); a drill's
            // helix revolves every ~3·⌀.
            draw_flutes: !matches!(
                tool.kind,
                ToolKind::VBit { .. }
                    | ToolKind::RadiusMill { .. }
                    | ToolKind::ChamferMill { .. }
                    | ToolKind::FaceMill
                    | ToolKind::ThreadMill { .. }
            ),
            flute_pitch: match tool.kind {
                ToolKind::Drill { .. } => (3.0 * tool.diameter).max(1e-3),
                _ => cutting_top,
            },
            flute_sign: {
                let base = if tool.cutting_direction == CutDir::Up {
                    1.0
                } else {
                    -1.0
                };
                // A twist drill leans the opposite way to a down-cut end mill.
                if matches!(tool.kind, ToolKind::Drill { .. }) {
                    -base
                } else {
                    base
                }
            },
            draw_face_inserts: matches!(tool.kind, ToolKind::FaceMill),
            profile,
        }
    }
}

impl canvas::Program<Message> for ToolCanvas {
    type State = ();

    fn draw(
        &self,
        _state: &(),
        renderer: &iced::Renderer,
        theme: &iced::Theme,
        bounds: iced::Rectangle,
        _cursor: iced::mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        use iced::Point as P;
        let mut frame = canvas::Frame::new(renderer, bounds.size());
        let fg = theme.palette().text;
        let faint = Color { a: 0.4, ..fg };

        let max_r = self.profile.max_radius().max(1e-3) as f32;
        let height = self.profile.height().max(1e-3) as f32;
        let margin = if self.compact { 60.0_f32 } else { 100.0 };
        let avail_w = (bounds.width - 2.0 * margin).max(1.0);
        let avail_h = (bounds.height - 100.0).max(1.0);
        // Fit the *mirrored* silhouette (width 2·max_r) and the height, keeping aspect.
        let scale = (avail_w / (2.0 * max_r)).min(avail_h / height);
        let cx = bounds.width / 2.0;
        let bottom = bounds.height - 52.0;
        let map = |r: f64, z: f64, sign: f32| {
            P::new(cx + sign * (r as f32) * scale, bottom - (z as f32) * scale)
        };

        // Axis of revolution: a faint dashed vertical line spanning the whole viewport
        // (top to bottom), not just the tool's extent — a full symmetry reference.
        let axis_dash = [4.0_f32, 4.0];
        let axis = canvas::Path::line(P::new(cx, 0.0), P::new(cx, bounds.height));
        frame.stroke(
            &axis,
            canvas::Stroke {
                line_dash: canvas::LineDash {
                    segments: &axis_dash,
                    offset: 0,
                },
                ..canvas::Stroke::default().with_color(faint).with_width(1.0)
            },
        );

        // The generatrix, per segment, mirrored to both sides.
        let dash = [5.0_f32, 4.0];
        for (pts, cutting) in self.profile.segment_polylines(0.05) {
            for sign in [1.0_f32, -1.0] {
                let path = canvas::Path::new(|b| {
                    let mut first = true;
                    for p in &pts {
                        let sp = map(p.x, p.y, sign);
                        if first {
                            b.move_to(sp);
                            first = false;
                        } else {
                            b.line_to(sp);
                        }
                    }
                });
                let stroke = if cutting {
                    canvas::Stroke::default().with_color(fg).with_width(2.2)
                } else {
                    canvas::Stroke {
                        line_dash: canvas::LineDash {
                            segments: &dash,
                            offset: 0,
                        },
                        ..canvas::Stroke::default().with_color(fg).with_width(1.2)
                    }
                };
                frame.stroke(&path, stroke);
            }
        }

        // Face-mill inserts: a row of small 90° squares seated on the bottom face, their
        // outer edges defining the cutting ⌀ — turns the bare body+arbor silhouette into a
        // recognisable shell mill. (90° square inserts only; other insert shapes deferred.)
        if self.draw_face_inserts {
            let r = self.profile.max_radius();
            let body_h = self.flute_length.max(1e-3);
            let n = self.flutes.max(1);
            // Insert side: a small glyph, never taller than the body (min-then-max avoids
            // clamp's panic when the body is shorter than the floor).
            let s = (r * 0.22).min(body_h * 0.6).max(0.1);
            let insert_fill = Color { a: 0.30, ..fg };
            for i in 0..n {
                // Centres span the diameter; the two end inserts sit at ±(r − s/2) so
                // their outer edge lands exactly on the periphery (±r).
                let cxi = if n == 1 {
                    0.0
                } else {
                    -(r - s / 2.0) + (i as f64) * (2.0 * (r - s / 2.0)) / ((n - 1) as f64)
                };
                let (x0, x1) = (cxi - s / 2.0, cxi + s / 2.0);
                let rect = canvas::Path::new(|b| {
                    b.move_to(map(x0, 0.0, 1.0));
                    b.line_to(map(x1, 0.0, 1.0));
                    b.line_to(map(x1, s, 1.0));
                    b.line_to(map(x0, s, 1.0));
                    b.close();
                });
                frame.fill(&rect, insert_fill);
                frame.stroke(
                    &rect,
                    canvas::Stroke::default().with_color(fg).with_width(1.2),
                );
            }
        }

        // (Thread-mill teeth are part of the generatrix now — the saw-tooth is the cutting
        // boundary itself — so there is no separate overlay: the profile loop above draws
        // the threads solid and the non-cutting bottom/shank dashed.)

        // Flutes + cutting direction. Each flute is projected onto the side view, drawn
        // only where it faces the viewer, and **tapered to the tool's actual radius at
        // that height** (`radius_at`) so on a ball / rounded nose the flutes converge at
        // the tip instead of running full width. Count the lines for the flute number; a
        // helical lean shows the cutting direction (Down vs Up), while straight (axial)
        // flutes read as vertical. (Helix pitch = one turn over the flute length is a
        // legibility choice, not the true helix angle.)
        use std::f64::consts::{PI, TAU};
        let boundary = self.profile.polyline(0.05);
        let radius_at = |z: f64| -> f64 {
            let mut best = 0.0_f64;
            for w in boundary.windows(2) {
                let (a, b) = (w[0], w[1]);
                if z + 1e-9 < a.y.min(b.y) || z - 1e-9 > a.y.max(b.y) {
                    continue;
                }
                let r = if (b.y - a.y).abs() < 1e-9 {
                    a.x.max(b.x)
                } else {
                    let t = ((z - a.y) / (b.y - a.y)).clamp(0.0, 1.0);
                    a.x + t * (b.x - a.x)
                };
                best = best.max(r);
            }
            best
        };
        if self.draw_flutes {
            let n = self.flutes.max(1);
            let flute_len = self.flute_length.max(1e-3);
            let pitch = self.flute_pitch.max(1e-3);
            let flute_col = if self.selected == Some(Field::Flutes) {
                palette::ACCENT_BLUE
            } else {
                Color { a: 0.75, ..fg }
            };
            // More samples when the helix wraps more times (keeps a tight drill smooth).
            let turns = (flute_len / pitch).max(1.0);
            let steps = ((48.0 * turns).ceil() as usize).clamp(48, 240);

            let mut flute_paths: Vec<Vec<P>> = Vec::new();
            match self.cutting_direction {
                CutDir::Straight => {
                    // ~N/2 visible front flutes, spread across the width, tapering to the tip.
                    let front = n.div_ceil(2).max(1);
                    for j in 0..front {
                        let theta = ((j as f64 + 0.5) / front as f64) * PI - PI / 2.0;
                        let pts = (0..=steps)
                            .map(|i| {
                                let z = flute_len * (i as f64) / (steps as f64);
                                map(radius_at(z) * theta.sin(), z, 1.0)
                            })
                            .collect();
                        flute_paths.push(pts);
                    }
                }
                _ => {
                    let s = self.flute_sign;
                    for k in 0..n {
                        let psi0 = (k as f64) * TAU / (n as f64);
                        let mut seg: Vec<P> = Vec::new();
                        for i in 0..=steps {
                            let z = flute_len * (i as f64) / (steps as f64);
                            let psi = psi0 + s * TAU * (z / pitch);
                            if psi.cos() > 0.0 {
                                seg.push(map(radius_at(z) * psi.sin(), z, 1.0));
                            } else if seg.len() >= 2 {
                                flute_paths.push(std::mem::take(&mut seg));
                            } else {
                                seg.clear();
                            }
                        }
                        if seg.len() >= 2 {
                            flute_paths.push(seg);
                        }
                    }
                }
            }
            for pts in flute_paths {
                if pts.len() < 2 {
                    continue;
                }
                let path = canvas::Path::new(|b| {
                    b.move_to(pts[0]);
                    for p in &pts[1..] {
                        b.line_to(*p);
                    }
                });
                frame.stroke(
                    &path,
                    canvas::Stroke::default()
                        .with_color(flute_col)
                        .with_width(1.0),
                );
            }
        }

        self.draw_dimensions(&mut frame, fg, cx, bottom, scale, max_r * scale);

        vec![frame.into_geometry()]
    }
}

pub(super) fn parameter_id(field: Field) -> String {
    format!("cutter-parameter-{field:?}")
}

pub(super) fn is_parameter(field: Field) -> bool {
    matches!(
        field,
        Field::ToolDiameter
            | Field::ToolLength
            | Field::FluteLength
            | Field::ShankDiameter
            | Field::ShankLength
            | Field::NeckLength
            | Field::NeckDiameter
            | Field::CornerRadius
            | Field::TipLength
            | Field::TipRadius
            | Field::TipDiameter
            | Field::PointAngle
            | Field::ChamferAngle
            | Field::ToolThreadPitch
            | Field::Flutes
    )
}

/// Witness points are in the same (radius, height) coordinates as the profile.
#[derive(Debug)]
struct Dimension {
    field: Field,
    label: String,
    geometry: DimensionGeometry,
}

#[derive(Debug)]
enum DimensionGeometry {
    Horizontal {
        radius: f64,
        z: f64,
        above: bool,
    },
    Vertical {
        radius: f64,
        from: f64,
        to: f64,
        left: bool,
        lane: f32,
    },
    Leader {
        radius: f64,
        z: f64,
    },
    Angle {
        degrees: f64,
        apex: f64,
    },
}

fn dimensions(tool: &cam_model::Tool) -> Vec<Dimension> {
    use DimensionGeometry::*;
    use Field::*;
    let r = tool.radius();
    let flute = tool.flute_len();
    let shank = tool.shank_dia() * 0.5;
    let conical = matches!(
        tool.kind,
        ToolKind::VBit { .. } | ToolKind::ChamferMill { .. }
    );
    let mut dims = Vec::new();
    let mut add = |field, name: &str, value, geometry| {
        dims.push(Dimension {
            field,
            label: format!("{name} {} mm", fmt_num(value)),
            geometry,
        });
    };
    add(
        ToolDiameter,
        if conical { "Shank ⌀" } else { "⌀" },
        tool.diameter,
        Horizontal {
            radius: r,
            z: if conical { tool.length } else { 0.0 },
            above: conical,
        },
    );
    add(
        ToolLength,
        "L",
        tool.length,
        Vertical {
            radius: r.max(shank),
            from: 0.0,
            to: tool.length,
            left: true,
            lane: 0.0,
        },
    );
    if !conical {
        add(
            FluteLength,
            if matches!(tool.kind, ToolKind::FaceMill) {
                "Body"
            } else {
                "Lc"
            },
            flute,
            Vertical {
                radius: r,
                from: 0.0,
                to: flute,
                left: false,
                lane: 0.0,
            },
        );
        add(
            ShankDiameter,
            if matches!(tool.kind, ToolKind::FaceMill) {
                "Arbor ⌀"
            } else {
                "Shank ⌀"
            },
            shank * 2.0,
            Horizontal {
                radius: shank,
                z: tool.length,
                above: true,
            },
        );
        if !matches!(tool.kind, ToolKind::FaceMill) {
            add(
                ShankLength,
                "Ls",
                (tool.length - flute).max(0.0),
                Vertical {
                    radius: shank,
                    from: flute,
                    to: tool.length,
                    left: false,
                    lane: 0.0,
                },
            );
        }
        if tool.neck_length > 0.0 {
            add(
                NeckLength,
                "Ln",
                tool.neck_length,
                Vertical {
                    radius: tool.neck_dia() * 0.5,
                    from: flute,
                    to: flute + tool.neck_length,
                    left: false,
                    lane: 48.0,
                },
            );
        }
        if tool.neck_length > 0.0 || matches!(tool.kind, ToolKind::ThreadMill { pitch: None }) {
            let z = if matches!(tool.kind, ToolKind::ThreadMill { pitch: None }) {
                flute * 0.6
            } else {
                flute + tool.neck_length * 0.5
            };
            add(
                NeckDiameter,
                "Neck ⌀",
                tool.neck_dia(),
                Leader {
                    radius: tool.neck_dia() * 0.5,
                    z,
                },
            );
        }
    }
    match tool.kind {
        ToolKind::RadiusMill {
            corner_radius,
            tip_length,
        } => {
            let t = std::f64::consts::FRAC_1_SQRT_2;
            add(
                CornerRadius,
                "R",
                corner_radius,
                Leader {
                    radius: r + corner_radius * (1.0 - t),
                    z: tip_length + corner_radius * t,
                },
            );
            add(
                TipLength,
                "Tip length",
                tip_length,
                Vertical {
                    radius: r,
                    from: 0.0,
                    to: tip_length,
                    left: false,
                    lane: 48.0,
                },
            );
        }
        ToolKind::BullNose { corner_radius } => {
            let t = std::f64::consts::FRAC_1_SQRT_2;
            add(
                CornerRadius,
                "R",
                corner_radius,
                Leader {
                    radius: r - corner_radius + corner_radius * t,
                    z: corner_radius * (1.0 - t),
                },
            );
        }
        ToolKind::VBit { tip_radius, .. } => {
            add(
                TipRadius,
                "Tip R",
                tip_radius,
                Leader {
                    radius: tip_radius * 0.5,
                    z: tip_radius * (1.0 - 3.0_f64.sqrt() * 0.5),
                },
            );
        }
        ToolKind::ChamferMill { tip_diameter, .. } => {
            add(
                TipDiameter,
                "Tip ⌀",
                tip_diameter,
                Horizontal {
                    radius: tip_diameter * 0.5,
                    z: 0.0,
                    above: false,
                },
            );
        }
        ToolKind::ThreadMill { pitch: Some(pitch) } => {
            add(
                ToolThreadPitch,
                "P",
                pitch,
                Vertical {
                    radius: r,
                    from: 0.0,
                    to: pitch,
                    left: false,
                    lane: 48.0,
                },
            );
        }
        _ => {}
    }
    let angle = match tool.kind {
        ToolKind::VBit {
            included_angle_deg,
            tip_radius,
        } => Some((
            PointAngle,
            included_angle_deg,
            -tip_radius * (1.0 / (included_angle_deg.to_radians() * 0.5).sin() - 1.0),
        )),
        ToolKind::ChamferMill {
            included_angle_deg,
            tip_diameter,
        } => Some((
            ChamferAngle,
            included_angle_deg,
            -tip_diameter * 0.5 / (included_angle_deg.to_radians() * 0.5).tan(),
        )),
        ToolKind::Drill { point_angle_deg } => Some((PointAngle, point_angle_deg, 0.0)),
        _ => None,
    };
    if let Some((field, degrees, apex)) = angle {
        dims.push(Dimension {
            field,
            label: format!("{}°", fmt_num(degrees)),
            geometry: Angle { degrees, apex },
        });
    }
    dims
}

impl ToolCanvas {
    pub(super) fn selected(mut self, field: Option<Field>) -> Self {
        self.compact = true;
        self.selected = Some(
            field
                .filter(|f| {
                    *f == Field::Flutes || dimensions(&self.tool).iter().any(|d| d.field == *f)
                })
                .unwrap_or(Field::ToolDiameter),
        );
        self
    }

    fn draw_dimensions(
        &self,
        frame: &mut canvas::Frame,
        fg: Color,
        cx: f32,
        bottom: f32,
        scale: f32,
        radius_px: f32,
    ) {
        use iced::Point as P;
        let map = |r: f64, z: f64| P::new(cx + r as f32 * scale, bottom - z as f32 * scale);
        let accent = palette::ACCENT_BLUE;
        let dims = dimensions(&self.tool);
        for dim in dims
            .iter()
            .filter(|d| !self.compact || self.selected == Some(d.field))
        {
            let color = accent;
            let stroke = canvas::Stroke::default().with_color(color).with_width(1.0);
            let line =
                |frame: &mut canvas::Frame, a, b| frame.stroke(&canvas::Path::line(a, b), stroke);
            let label = |frame: &mut canvas::Frame, position, value: String| {
                frame.fill_text(canvas::Text {
                    content: value,
                    position,
                    color,
                    size: iced::Pixels(12.0),
                    ..Default::default()
                });
            };
            match dim.geometry {
                DimensionGeometry::Horizontal { radius, z, above } => {
                    let a = map(-radius, z);
                    let b = map(radius, z);
                    let y = if above { a.y - 24.0 } else { bottom + 22.0 };
                    let p = P::new(a.x, y);
                    let q = P::new(b.x, y);
                    line(frame, a, P::new(a.x, y + if above { -4.0 } else { 4.0 }));
                    line(frame, b, P::new(b.x, y + if above { -4.0 } else { 4.0 }));
                    dimension_line(frame, p, q, color);
                    label(frame, P::new(cx, y - 17.0), dim.label.clone());
                }
                DimensionGeometry::Vertical {
                    radius,
                    from,
                    to,
                    left,
                    lane,
                } => {
                    let sign = if left { -1.0 } else { 1.0 };
                    let a = map(sign * radius, from);
                    let b = map(sign * radius, to);
                    let lane = if self.compact { 0.0 } else { lane };
                    let x = cx + sign as f32 * (radius_px + 22.0 + lane);
                    line(frame, a, P::new(x + sign as f32 * 4.0, a.y));
                    line(frame, b, P::new(x + sign as f32 * 4.0, b.y));
                    dimension_line(frame, P::new(x, a.y), P::new(x, b.y), color);
                    let position = P::new(x + 6.0, (a.y + b.y) * 0.5 - 8.0);
                    frame.with_save(|frame| {
                        frame.translate(iced::Vector::new(position.x, position.y));
                        frame.rotate(-std::f32::consts::FRAC_PI_2);
                        label(frame, P::ORIGIN, dim.label.clone());
                    });
                }
                DimensionGeometry::Leader { radius, z } => {
                    let tip = map(-radius, z);
                    let end = P::new(cx - radius_px - 16.0, (tip.y - 26.0).max(52.0));
                    line(frame, tip, end);
                    arrow(frame, tip, end, color);
                    label(frame, P::new(8.0, end.y - 18.0), dim.label.clone());
                }
                DimensionGeometry::Angle { degrees, apex } => {
                    let origin = map(0.0, apex);
                    let half = (degrees as f32).to_radians() * 0.5;
                    let arc_r = (radius_px * 0.8).clamp(18.0, 38.0);
                    let at =
                        |a: f32| P::new(origin.x + arc_r * a.sin(), origin.y - arc_r * a.cos());
                    let arc = canvas::Path::new(|b| {
                        b.move_to(at(-half));
                        for i in 1..=32 {
                            b.line_to(at(-half + 2.0 * half * i as f32 / 32.0));
                        }
                    });
                    frame.stroke(&arc, stroke);
                    for a in [-half, half] {
                        line(frame, origin, at(a));
                    }
                    arrow(frame, at(-half), at(-half + 0.15), color);
                    arrow(frame, at(half), at(half - 0.15), color);
                    let anchor = at(half);
                    let end = P::new(cx + radius_px + 12.0, bottom - 12.0);
                    line(frame, anchor, end);
                    label(frame, end, dim.label.clone());
                }
            }
        }
        if self.compact {
            let field = self.selected.unwrap_or(Field::ToolDiameter);
            let title = if field == Field::Flutes {
                format!("{} flutes · {}", self.flutes, self.cutting_direction)
            } else {
                match (self.tool.kind, field) {
                    (ToolKind::VBit { .. } | ToolKind::ChamferMill { .. }, Field::ToolDiameter) => {
                        "Shank diameter (mm)"
                    }
                    (ToolKind::FaceMill, Field::ToolDiameter) => "Cutting diameter (mm)",
                    (ToolKind::FaceMill, Field::FluteLength) => "Body height (mm)",
                    (ToolKind::FaceMill, Field::ShankDiameter) => "Arbor diameter (mm)",
                    (ToolKind::ThreadMill { pitch: None }, Field::FluteLength) => {
                        "Length of cut (mm)"
                    }
                    (ToolKind::ThreadMill { pitch: Some(_) }, Field::FluteLength) => {
                        "Thread length (mm)"
                    }
                    _ => field.label(),
                }
                .to_string()
            };
            frame.fill_text(canvas::Text {
                content: title,
                position: P::new(8.0, 4.0),
                color: accent,
                size: iced::Pixels(12.0),
                ..Default::default()
            });
        } else {
            frame.fill_text(canvas::Text {
                content: format!(
                    "{} flutes · {} · dimensions in mm",
                    self.flutes, self.cutting_direction
                ),
                position: P::new(8.0, 4.0),
                color: fg,
                size: iced::Pixels(12.0),
                ..Default::default()
            });
        }
    }
}

fn arrow(frame: &mut canvas::Frame, tip: iced::Point, toward: iced::Point, color: Color) {
    let dx = toward.x - tip.x;
    let dy = toward.y - tip.y;
    let length = dx.hypot(dy).max(0.001);
    let (ux, uy) = (dx / length, dy / length);
    let head = canvas::Path::new(|b| {
        b.move_to(tip);
        b.line_to(iced::Point::new(
            tip.x + ux * 6.0 - uy * 2.5,
            tip.y + uy * 6.0 + ux * 2.5,
        ));
        b.line_to(iced::Point::new(
            tip.x + ux * 6.0 + uy * 2.5,
            tip.y + uy * 6.0 - ux * 2.5,
        ));
        b.close();
    });
    frame.fill(&head, color);
}

fn dimension_line(frame: &mut canvas::Frame, a: iced::Point, b: iced::Point, color: Color) {
    frame.stroke(
        &canvas::Path::line(a, b),
        canvas::Stroke::default().with_color(color),
    );
    arrow(frame, a, b, color);
    arrow(frame, b, a, color);
}

impl App {
    pub(super) fn operation_drawing_tool(&self, op: &Operation) -> Option<&cam_model::Tool> {
        self.operation_tools
            .iter()
            .chain(self.controller.inspection_setup().tools.iter())
            .find(|tool| tool.number == op.tool())
    }

    pub(super) fn tool_info_overlay(&self) -> Option<Element<'_, Message>> {
        let tool = self.tool_info.as_ref()?;
        let content = column![
            row![text(format!("T{} · {}", tool.number, tool.kind)).size(18), Space::new().width(Length::Fill), inspector_button("Close").on_press(Message::ToolInfoClose)].spacing(12).align_y(Alignment::Center),
            canvas::Canvas::new(ToolCanvas::new(tool)).width(Length::Fill).height((self.window.height - 200.0).clamp(220.0, 520.0)),
            text("L: overall length · Lc: cutting length · Ls: shank length · Ln: neck length · P: thread pitch").size(11),
        ].spacing(10);
        Some(
            container(
                container(content)
                    .padding(16)
                    .width(700)
                    .style(popup_surface),
            )
            .padding(16)
            .width(Length::Fill)
            .height(Length::Fill)
            .center_x(Length::Fill)
            .center_y(Length::Fill)
            .into(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drawing_uses_effective_dimensions_and_correct_witness_points() {
        let tool = cam_model::Tool {
            diameter: 6.0,
            length: 50.0,
            flute_length: 20.0,
            shank_diameter: 8.0,
            neck_length: 5.0,
            neck_diameter: 4.0,
            kind: ToolKind::BullNose { corner_radius: 1.0 },
            ..Default::default()
        };
        let dims = dimensions(&tool);
        let get = |field| &dims.iter().find(|d| d.field == field).unwrap().geometry;
        assert!(matches!(
            get(Field::ToolDiameter),
            DimensionGeometry::Horizontal {
                radius: 3.0,
                z: 0.0,
                ..
            }
        ));
        assert!(matches!(
            get(Field::ShankDiameter),
            DimensionGeometry::Horizontal {
                radius: 4.0,
                z: 50.0,
                ..
            }
        ));
        assert!(matches!(
            get(Field::ShankLength),
            DimensionGeometry::Vertical {
                from: 20.0,
                to: 50.0,
                ..
            }
        ));
        assert!(matches!(
            get(Field::NeckLength),
            DimensionGeometry::Vertical {
                from: 20.0,
                to: 25.0,
                ..
            }
        ));
        assert!(
            matches!(get(Field::CornerRadius), DimensionGeometry::Leader { radius, z } if (*radius - 2.70710678).abs() < 1e-7 && (*z - 0.29289322).abs() < 1e-7)
        );
        let legacy = cam_model::Tool {
            diameter: 6.0,
            length: 50.0,
            ..Default::default()
        };
        let dims = dimensions(&legacy);
        assert_eq!(
            dims.iter()
                .find(|d| d.field == Field::FluteLength)
                .unwrap()
                .label,
            "Lc 50 mm"
        );
        assert_eq!(
            dims.iter()
                .find(|d| d.field == Field::ShankDiameter)
                .unwrap()
                .label,
            "Shank ⌀ 6 mm"
        );
    }

    #[test]
    fn every_editable_cutter_dimension_has_an_annotation() {
        let (mut app, _) = App::new();
        app.workspace = Workspace::Tools;
        app.active_tab = RibbonTab::Tools;
        let kinds = ToolKindPick::ALL
            .iter()
            .map(|kind| kind.to_kind())
            .chain([ToolKind::ThreadMill { pitch: Some(1.25) }]);
        for kind in kinds {
            let tool = crate::tool_library::default_tool(1, kind);
            app.tool_edit = Some(tool.clone());
            let dims = dimensions(&tool);
            for field in app
                .inspector_fields()
                .into_iter()
                .filter(|f| is_parameter(*f) && *f != Field::Flutes)
            {
                assert!(dims.iter().any(|d| d.field == field), "{kind:?}: {field:?}");
            }
        }
        let chamfer = cam_model::Tool {
            diameter: 6.0,
            length: 40.0,
            kind: ToolKind::ChamferMill {
                included_angle_deg: 90.0,
                tip_diameter: 2.0,
            },
            ..Default::default()
        };
        assert!(dimensions(&chamfer).iter().any(|d| matches!(d.geometry, DimensionGeometry::Angle { degrees: 90.0, apex } if (apex + 1.0).abs() < 1e-9)));
    }

    #[test]
    fn cutter_field_focus_and_live_edits_select_the_drawing_dimension() {
        let (mut app, _) = App::new();
        app.workspace = Workspace::Tools;
        app.active_tab = RibbonTab::Tools;
        app.library.tools = vec![crate::tool_library::default_tool(1, ToolKind::EndMill)];
        app.reload_tool_edit();
        let _ = app.update(Message::ToolParameterFocus(Field::FluteLength, true));
        assert_eq!(app.selected_tool_parameter, Some(Field::FluteLength));
        let _ = app.update(Message::ToolParameterFocus(Field::ToolDiameter, false));
        assert_eq!(app.selected_tool_parameter, Some(Field::FluteLength));
        let _ = app.update(Message::FieldChanged(Field::ToolDiameter, "8.5".into()));
        assert_eq!(app.selected_tool_parameter, Some(Field::ToolDiameter));
        assert_eq!(app.preview_tool().unwrap().diameter, 8.5);
        assert_ne!(app.library.tools[0].diameter, 8.5);
        app.reload_tool_edit();
        assert_eq!(app.selected_tool_parameter, None);
    }

    #[test]
    fn operation_info_uses_staged_cutter_and_closes_without_changing_operation() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "drawing test").unwrap();
        app.controller.new_operation(OpKind::Profile);
        let op = app
            .controller
            .inspection_setup()
            .operations
            .last()
            .unwrap()
            .clone();
        let saved = app.controller.inspection_setup().clone();
        let staged = cam_model::Tool {
            number: op.tool(),
            diameter: 9.25,
            length: 80.0,
            ..Default::default()
        };
        app.operation_tools.push(staged.clone());
        let selected = app.operation_drawing_tool(&op).unwrap().clone();
        assert_eq!(selected, staged);
        let _ = app.update(Message::ToolInfoOpen(selected));
        assert_eq!(app.tool_info, Some(staged.clone()));
        let _ = app.update(Message::CadEscape);
        assert!(app.tool_info.is_none());
        assert_eq!(app.controller.inspection_setup(), &saved);
        let _ = app.update(Message::ToolInfoOpen(staged));
        let _ = app.update(Message::ToolInfoClose);
        assert!(app.tool_info.is_none());
        app.operation_tools.clear();
        assert_eq!(
            app.operation_drawing_tool(&op),
            saved.tools.iter().find(|t| t.number == op.tool())
        );
    }

    #[test]
    #[ignore = "requires a graphics adapter; exports cutter drawing previews to /tmp"]
    fn render_cutter_drawing_previews() {
        use canvas::Program;
        use iced::advanced::graphics::geometry::Renderer as _;
        use iced::advanced::renderer::Headless;
        let mut renderer = iced::futures::executor::block_on(<iced::Renderer as Headless>::new(
            iced::Font::DEFAULT,
            iced::Pixels(12.0),
            None,
        ))
        .expect("graphics adapter");
        for (name, tool, selected) in [
            (
                "endmill-info",
                crate::tool_library::default_tool(1, ToolKind::EndMill),
                None,
            ),
            (
                "endmill-length",
                crate::tool_library::default_tool(1, ToolKind::EndMill),
                Some(Field::ToolLength),
            ),
            (
                "bullnose-radius",
                crate::tool_library::default_tool(2, ToolKind::BullNose { corner_radius: 1.0 }),
                Some(Field::CornerRadius),
            ),
            (
                "drill-angle",
                crate::tool_library::default_tool(
                    3,
                    ToolKind::Drill {
                        point_angle_deg: 118.0,
                    },
                ),
                Some(Field::PointAngle),
            ),
        ] {
            let compact = selected.is_some();
            let size = if compact {
                iced::Size::new(320, 270)
            } else {
                iced::Size::new(668, 520)
            };
            let mut drawing = ToolCanvas::new(&tool);
            if compact {
                drawing = drawing.selected(selected);
            }
            for geometry in drawing.draw(
                &(),
                &renderer,
                &iced::Theme::Light,
                iced::Rectangle::with_size(iced::Size::new(size.width as f32, size.height as f32)),
                iced::mouse::Cursor::Unavailable,
            ) {
                renderer.draw_geometry(geometry);
            }
            let pixels = Headless::screenshot(&mut renderer, size, 1.0, Color::WHITE);
            let mut ppm = format!("P6\n{} {}\n255\n", size.width, size.height).into_bytes();
            for pixel in pixels.as_chunks::<4>().0 {
                ppm.extend_from_slice(&pixel[..3]);
            }
            std::fs::write(format!("/tmp/opencam-cutter-{name}.ppm"), ppm).unwrap();
            iced::advanced::Renderer::reset(
                &mut renderer,
                iced::Rectangle::with_size(iced::Size::new(size.width as f32, size.height as f32)),
            );
        }
    }
}
