//! Inspector draft and viewport preview for model and drawing scaling.
use super::*;
use iced::widget::column;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum ScaleField {
    Percent,
    Size,
}

#[derive(Debug, Clone)]
pub(super) enum ScaleEdit {
    Open(TreeNode),
    Value(ScaleField, usize, String),
    Lock(bool),
    Preset(f64),
    Apply,
    Cancel,
}

#[derive(Clone, Copy, PartialEq)]
pub(super) struct ScalePreview {
    pub target: TreeNode,
    pub factors: [f64; 3],
}

pub(super) struct ScaleDraft {
    target: TreeNode,
    name: String,
    axes: usize,
    bounds: [[f64; 3]; 2],
    values: [Option<f64>; 3],
    inputs: [Option<(ScaleField, String)>; 3],
    locked: bool,
    last_axis: usize,
    preview_factors: [f64; 3],
}

impl ScaleDraft {
    pub(super) fn factors(&self) -> Option<[f64; 3]> {
        let [x, y, z] = self.values;
        let factors = [x?, y?, z?];
        (factors
            .into_iter()
            .all(|factor| factor.is_finite() && factor > 0.)
            && self
                .bounds
                .iter()
                .all(|point| (0..3).all(|axis| ((point[axis] * factors[axis]) as f32).is_finite())))
        .then_some(factors)
    }

    pub(super) fn preview(&self) -> ScalePreview {
        ScalePreview {
            target: self.target,
            factors: self.preview_factors,
        }
    }

    pub(super) fn original_frame(&self) -> ([f32; 3], [f32; 3]) {
        (
            self.bounds[0].map(|v| v as f32),
            self.bounds[1].map(|v| v as f32),
        )
    }

    fn size(&self, axis: usize) -> f64 {
        self.bounds[1][axis] - self.bounds[0][axis]
    }

    fn value(&self, field: ScaleField, axis: usize) -> String {
        let input_axis = if self.locked && field == ScaleField::Percent {
            self.last_axis
        } else {
            axis
        };
        if let Some((active, text)) = &self.inputs[input_axis] {
            if *active == field {
                return text.clone();
            }
        }
        let Some(factor) = self.values[axis] else {
            return String::new();
        };
        let value = match field {
            ScaleField::Percent => factor * 100.,
            ScaleField::Size => factor * self.size(axis),
        };
        if value > 0. && !(0.005..1e6).contains(&value) {
            format!("{value:.2e}")
        } else {
            format!("{value:.2}")
        }
    }

    fn set_value(&mut self, field: ScaleField, axis: usize, value: String) {
        if axis >= self.axes {
            return;
        }
        let divisor = match field {
            ScaleField::Percent => 100.,
            ScaleField::Size => self.size(axis),
        };
        if divisor == 0. {
            return;
        }
        let factor = value
            .trim()
            .parse::<f64>()
            .ok()
            .map(|number| number / divisor)
            .filter(|factor| factor.is_finite() && *factor > 0.);
        self.last_axis = axis;
        if self.locked {
            self.values[..self.axes].fill(factor);
            self.inputs.fill(None);
        } else {
            self.values[axis] = factor;
        }
        self.inputs[axis] = Some((field, value));
    }
}

impl App {
    pub(super) fn edit_scale(&mut self, edit: ScaleEdit) {
        if let ScaleEdit::Open(target) = edit {
            if self.controller.cad_import_pending() {
                self.status = "Wait for the geometry import to finish before scaling.".into();
                return;
            }
            let geometry = self.controller.geometry_state();
            let source = match target {
                TreeNode::Asset(id) => geometry.asset(id).map(|asset| {
                    let points = geometry
                        .instances
                        .iter()
                        .filter(|i| i.asset_id == id)
                        .flat_map(|instance| {
                            let body = asset
                                .topology
                                .bodies
                                .iter()
                                .find(|body| body.id == instance.body_id);
                            body.into_iter().flat_map(move |body| {
                                (0..8).map(move |corner| {
                                    instance.placement.transform_point(std::array::from_fn(
                                        |axis| body.bounds[(corner >> axis) & 1][axis],
                                    ))
                                })
                            })
                        });
                    (&*asset.name, 3, geometry_bounds(points))
                }),
                TreeNode::Drawing(index) => {
                    self.controller.drawing_sources().get(index).map(|drawing| {
                        let points = geometry.drawing_regions[drawing.regions.clone()]
                            .iter()
                            .flat_map(|region| region.outer().points())
                            .chain(
                                geometry.drawing_paths[drawing.paths.clone()]
                                    .iter()
                                    .flat_map(|path| path.points()),
                            )
                            .map(|point| [point.x, point.y, 0.]);
                        (drawing.name, 2, geometry_bounds(points))
                    })
                }
                _ => None,
            };
            if let Some((name, axes, Some(bounds))) = source {
                self.scale_draft = Some(ScaleDraft {
                    target,
                    name: name.into(),
                    axes,
                    bounds,
                    values: [Some(1.); 3],
                    inputs: Default::default(),
                    locked: true,
                    last_axis: 0,
                    preview_factors: [1.; 3],
                });
                self.inspector_enabled = true;
            }
            return;
        }
        let Some(draft) = &mut self.scale_draft else {
            return;
        };
        match edit {
            ScaleEdit::Value(field, axis, value) => draft.set_value(field, axis, value),
            ScaleEdit::Lock(locked) => {
                draft.locked = locked;
                if locked {
                    let factor = draft.values[draft.last_axis];
                    draft.values[..draft.axes].fill(factor);
                    let input = draft.inputs[draft.last_axis].take();
                    draft.inputs.fill(None);
                    draft.inputs[draft.last_axis] = input;
                }
            }
            ScaleEdit::Preset(factor) => {
                draft.values[..draft.axes].fill(Some(factor));
                draft.inputs.fill(None);
            }
            ScaleEdit::Cancel => {
                self.scale_draft = None;
                return;
            }
            ScaleEdit::Apply => {
                let Some(factors) = draft.factors() else {
                    return;
                };
                let result = match draft.target {
                    TreeNode::Asset(id) => self.controller.scale_cad_asset(id, factors),
                    TreeNode::Drawing(index) => self
                        .controller
                        .scale_drawing(index, [factors[0], factors[1]]),
                    _ => unreachable!("scale draft target"),
                };
                match result {
                    Ok(()) => {
                        self.scale_draft = None;
                        self.cad_hover = None;
                        self.cad_candidates.clear();
                        self.snap_hover = None;
                        self.refresh_fields();
                        self.status = "Scale applied.".into();
                    }
                    Err(error) => self.status = error,
                }
                return;
            }
            ScaleEdit::Open(_) => unreachable!("handled above"),
        }
        if let Some(factors) = draft.factors() {
            draft.preview_factors = factors;
        }
    }

    pub(super) fn scale_inspector(&self, draft: &ScaleDraft) -> Element<'_, Message> {
        let apply = draft.factors().map(|_| Message::Scale(ScaleEdit::Apply));
        let header = container(inspector_heading(draft.name.clone())).padding([12, 12]);
        let mut fields = column![row![
            inspector_button("mm → inch").on_press(Message::Scale(ScaleEdit::Preset(1. / 25.4))),
            inspector_button("inch → mm").on_press(Message::Scale(ScaleEdit::Preset(25.4))),
        ]
        .spacing(8),]
        .spacing(8)
        .padding(12);

        let mut headings = row![Space::new().width(40)].spacing(4);
        for axis in ["X", "Y", "Z"].into_iter().take(draft.axes) {
            headings = headings.push(
                text(axis)
                    .size(11)
                    .width(Length::Fill)
                    .align_x(Alignment::Center),
            );
        }
        fields = fields.push(headings.push(Space::new().width(20)));

        for (field, label, unit) in [
            (ScaleField::Percent, "Scale", "%"),
            (ScaleField::Size, "Size", "mm"),
        ] {
            let mut values = row![text(label).size(12).width(40)]
                .spacing(4)
                .align_y(Alignment::Center);
            for axis in 0..draft.axes {
                let mut input = text_input("", &draft.value(field, axis))
                    .padding([3, 4])
                    .size(12);
                if field == ScaleField::Percent || draft.size(axis) > 0. {
                    input = input.on_input(move |value| {
                        Message::Scale(ScaleEdit::Value(field, axis, value))
                    });
                }
                values = values.push(input.width(Length::Fill));
            }
            fields = fields.push(values.push(text(unit).size(12).width(20)));
        }
        fields = fields.push(
            checkbox(draft.locked)
                .label("Uniform scale")
                .size(16)
                .text_size(13)
                .on_toggle(|locked| Message::Scale(ScaleEdit::Lock(locked))),
        );
        if apply.is_none() {
            fields = fields.push(text("Enter positive values within the display range.").size(12));
        }
        let footer = inspector_footer(
            row![
                inspector_button("Cancel").on_press(Message::Scale(ScaleEdit::Cancel)),
                Space::new().width(Length::Fill),
                inspector_button("Apply")
                    .style(inspector_primary)
                    .on_press_maybe(apply),
            ]
            .spacing(8)
            .align_y(Alignment::Center),
        );
        column![
            header,
            scrollable(fields).width(Length::Fill).height(Length::Fill),
            footer,
        ]
        .width(Length::Fill)
        .height(Length::Fill)
        .into()
    }
}

fn geometry_bounds(points: impl Iterator<Item = [f64; 3]>) -> Option<[[f64; 3]; 2]> {
    let mut bounds = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
    for point in points {
        for (axis, value) in point.into_iter().enumerate() {
            bounds[0][axis] = bounds[0][axis].min(value);
            bounds[1][axis] = bounds[1][axis].max(value);
        }
    }
    bounds
        .iter()
        .flatten()
        .all(|value| (*value as f32).is_finite())
        .then_some(bounds)
}

pub(super) fn imported_drawing_scene(
    controller: &AppController,
    preview: Option<ScalePreview>,
) -> Scene {
    let drawing = preview.and_then(|preview| match preview.target {
        TreeNode::Drawing(index) if preview.factors != [1.; 3] => {
            let source = controller.drawing_sources().into_iter().nth(index)?;
            let [x, y, _] = preview.factors;
            let (regions, paths) = controller.scaled_drawing_geometry(&source, [x, y]).ok()?;
            Some((source, regions, paths))
        }
        _ => None,
    });
    let mut scene = Scene::new();
    for (index, region) in controller.regions().iter().enumerate() {
        let region = drawing
            .as_ref()
            .filter(|(source, _, _)| source.regions.contains(&index))
            .map_or(region, |(source, regions, _)| {
                &regions[index - source.regions.start]
            });
        scene.add_region(region, PART);
    }
    for (index, path) in controller.open_paths().iter().enumerate() {
        let path = drawing
            .as_ref()
            .filter(|(source, _, _)| source.paths.contains(&index))
            .map_or(path, |(source, _, paths)| {
                &paths[index - source.paths.start]
            });
        scene.add_open_path(path.points(), PART);
    }
    scene
}
