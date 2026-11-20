//! Material-linked cutting data. Presets seed concrete operation values; planning
//! never consults mutable shop data, so saving/reopening cannot change a cut.
use crate::{Operation, Tool};

/// Cutting lubrication requested independently of the machine's output wiring.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Cooling {
    #[default]
    None,
    Mms,
}
impl Cooling {
    pub const ALL: [Self; 2] = [Self::None, Self::Mms];
}
impl std::fmt::Display for Cooling {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::None => "None",
            Self::Mms => "MMS",
        })
    }
}

/// A shared stock material. Physical properties can be added without changing IDs.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Material {
    pub id: u64,
    pub name: String,
    /// Effective specific cutting force kc, N/mm², for the intended cutting conditions.
    /// Unset on legacy materials; never substitute an invented material coefficient.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub specific_cutting_force: Option<f64>,
}

impl Material {
    pub fn validate(&self) -> Result<(), String> {
        if self.id == 0 || self.name.trim().is_empty() {
            return Err("A material needs a nonzero ID and a name.".into());
        }
        if self
            .specific_cutting_force
            .is_some_and(|v| !v.is_finite() || v <= 0.)
        {
            return Err(
                "Specific cutting force must be finite and positive, or unset (N/mm²).".into(),
            );
        }
        Ok(())
    }
}

/// Optional entry/exit feed rates in mm/min. Unset values retain the strategy's
/// cutting-feed fallback, keeping existing projects and presets unchanged.
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct EntryFeeds {
    pub ramp: Option<f64>,
    pub lead_in: Option<f64>,
    pub lead_out: Option<f64>,
}
impl EntryFeeds {
    pub fn validate(&self) -> Result<(), String> {
        if [self.ramp, self.lead_in, self.lead_out]
            .into_iter()
            .flatten()
            .any(|v| !v.is_finite() || v <= 0.)
        {
            return Err("Ramp and lead feeds must be finite and positive, or unset.".into());
        }
        Ok(())
    }
    pub fn ramp(&self, cutting: f64) -> f64 {
        self.ramp.unwrap_or(cutting)
    }
    pub fn lead_in(&self, cutting: f64) -> f64 {
        self.lead_in.unwrap_or(cutting)
    }
    pub fn lead_out(&self, cutting: f64) -> f64 {
        self.lead_out.unwrap_or(cutting)
    }
}

/// Named cutting parameters for one cutter. IDs are local to the cutter.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ToolPreset {
    #[serde(default)]
    pub cooling: Cooling,
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub material_id: Option<u64>,
    /// Zero inherits the job spindle speed, as on an operation.
    pub spindle_rpm: f64,
    pub feed: f64,
    pub plunge_feed: f64,
    #[serde(default)]
    pub entry_feeds: EntryFeeds,
    /// Optional axial depth per pass, mm; only applied where supported.
    #[serde(default)]
    pub stepdown: Option<f64>,
    /// Optional radial overlap, fraction [0, 0.99]; only applied where supported.
    #[serde(default)]
    pub overlap: Option<f64>,
}

impl ToolPreset {
    pub fn validate(&self) -> Result<(), String> {
        self.entry_feeds.validate()?;
        if self.id == 0 || self.name.trim().is_empty() {
            return Err("A preset needs an ID and a name.".into());
        }
        if !self.spindle_rpm.is_finite()
            || self.spindle_rpm < 0.
            || !self.feed.is_finite()
            || self.feed <= 0.
            || !self.plunge_feed.is_finite()
            || self.plunge_feed <= 0.
            || self.stepdown.is_some_and(|v| !v.is_finite() || v <= 0.)
            || self
                .overlap
                .is_some_and(|v| !v.is_finite() || !(0. ..=0.99).contains(&v))
        {
            return Err(
                "Preset feeds and stepdown must be positive; RPM nonnegative; overlap between 0% and 99%."
                    .into(),
            );
        }
        Ok(())
    }
}

impl Tool {
    pub fn validate_presets(&self) -> Result<(), String> {
        let mut ids = std::collections::BTreeSet::new();
        for preset in &self.presets {
            preset.validate()?;
            if !ids.insert(preset.id) {
                return Err("Preset IDs must be unique within a cutter".into());
            }
        }
        Ok(())
    }

    /// The first material-specific preset wins ties, followed by an unlinked
    /// preset. A preset linked to a different material is never auto-selected.
    pub fn preset_for_material(&self, material: Option<u64>) -> Option<&ToolPreset> {
        material
            .and_then(|id| self.presets.iter().find(|p| p.material_id == Some(id)))
            .or_else(|| self.presets.iter().find(|p| p.material_id.is_none()))
    }

    pub fn next_preset_id(&mut self) -> u64 {
        self.preset_id_floor = self
            .preset_id_floor
            .max(self.presets.iter().map(|p| p.id).max().unwrap_or(0))
            + 1;
        self.preset_id_floor
    }
}

impl Operation {
    pub fn cooling(&self) -> Cooling {
        match self {
            Self::Profile(o) => o.cooling,
            Self::Pocket(o) => o.cooling,
            Self::Adaptive(o) => o.cooling,
            Self::Parallel(o) => o.cooling,
            Self::Spiral(o) => o.cooling,
            Self::Face(o) => o.cooling,
            Self::Drill(o) => o.cooling,
            Self::Chamfer(o) => o.cooling,
            Self::Bore(o) => o.cooling,
            Self::Thread(o) => o.cooling,
            Self::Engrave(o) => o.cooling,
            Self::Carve(o) => o.cooling,
        }
    }
    pub fn set_cooling(&mut self, cooling: Cooling) {
        match self {
            Self::Profile(o) => o.cooling = cooling,
            Self::Pocket(o) => o.cooling = cooling,
            Self::Adaptive(o) => o.cooling = cooling,
            Self::Parallel(o) => o.cooling = cooling,
            Self::Spiral(o) => o.cooling = cooling,
            Self::Face(o) => o.cooling = cooling,
            Self::Drill(o) => o.cooling = cooling,
            Self::Chamfer(o) => o.cooling = cooling,
            Self::Bore(o) => o.cooling = cooling,
            Self::Thread(o) => o.cooling = cooling,
            Self::Engrave(o) => o.cooling = cooling,
            Self::Carve(o) => o.cooling = cooling,
        }
    }
    pub fn entry_feeds(&self) -> &EntryFeeds {
        match self {
            Self::Profile(o) => &o.entry_feeds,
            Self::Pocket(o) => &o.entry_feeds,
            Self::Adaptive(o) => &o.entry_feeds,
            Self::Parallel(o) => &o.entry_feeds,
            Self::Spiral(o) => &o.entry_feeds,
            Self::Face(o) => &o.entry_feeds,
            Self::Drill(o) => &o.entry_feeds,
            Self::Chamfer(o) => &o.entry_feeds,
            Self::Bore(o) => &o.entry_feeds,
            Self::Thread(o) => &o.entry_feeds,
            Self::Engrave(o) => &o.entry_feeds,
            Self::Carve(o) => &o.entry_feeds,
        }
    }
    pub fn entry_feeds_mut(&mut self) -> &mut EntryFeeds {
        match self {
            Self::Profile(o) => &mut o.entry_feeds,
            Self::Pocket(o) => &mut o.entry_feeds,
            Self::Adaptive(o) => &mut o.entry_feeds,
            Self::Parallel(o) => &mut o.entry_feeds,
            Self::Spiral(o) => &mut o.entry_feeds,
            Self::Face(o) => &mut o.entry_feeds,
            Self::Drill(o) => &mut o.entry_feeds,
            Self::Chamfer(o) => &mut o.entry_feeds,
            Self::Bore(o) => &mut o.entry_feeds,
            Self::Thread(o) => &mut o.entry_feeds,
            Self::Engrave(o) => &mut o.entry_feeds,
            Self::Carve(o) => &mut o.entry_feeds,
        }
    }

    pub fn tool_preset(&self) -> Option<&ToolPreset> {
        match self {
            Self::Profile(o) => o.tool_preset.as_ref(),
            Self::Pocket(o) => o.tool_preset.as_ref(),
            Self::Adaptive(o) => o.tool_preset.as_ref(),
            Self::Parallel(o) => o.tool_preset.as_ref(),
            Self::Spiral(o) => o.tool_preset.as_ref(),
            Self::Face(o) => o.tool_preset.as_ref(),
            Self::Drill(o) => o.tool_preset.as_ref(),
            Self::Chamfer(o) => o.tool_preset.as_ref(),
            Self::Bore(o) => o.tool_preset.as_ref(),
            Self::Thread(o) => o.tool_preset.as_ref(),
            Self::Engrave(o) => o.tool_preset.as_ref(),
            Self::Carve(o) => o.tool_preset.as_ref(),
        }
    }

    /// Clear the association while retaining all current operation values.
    pub fn clear_tool_preset(&mut self) {
        let slot = match self {
            Self::Profile(o) => &mut o.tool_preset,
            Self::Pocket(o) => &mut o.tool_preset,
            Self::Adaptive(o) => &mut o.tool_preset,
            Self::Parallel(o) => &mut o.tool_preset,
            Self::Spiral(o) => &mut o.tool_preset,
            Self::Face(o) => &mut o.tool_preset,
            Self::Drill(o) => &mut o.tool_preset,
            Self::Chamfer(o) => &mut o.tool_preset,
            Self::Bore(o) => &mut o.tool_preset,
            Self::Thread(o) => &mut o.tool_preset,
            Self::Engrave(o) => &mut o.tool_preset,
            Self::Carve(o) => &mut o.tool_preset,
        };
        *slot = None;
    }

    pub fn apply_tool_preset(&mut self, preset: &ToolPreset) -> Result<(), String> {
        preset.validate()?;
        self.set_cooling(preset.cooling);
        self.set_spindle_rpm(preset.spindle_rpm);
        let feeds = self.entry_feeds_mut();
        if let Some(v) = preset.entry_feeds.ramp {
            feeds.ramp = Some(v);
        }
        if let Some(v) = preset.entry_feeds.lead_in {
            feeds.lead_in = Some(v);
        }
        if let Some(v) = preset.entry_feeds.lead_out {
            feeds.lead_out = Some(v);
        }
        let (feed, plunge, stepdown, overlap, slot) = match self {
            Self::Profile(o) => (
                &mut o.feed,
                Some(&mut o.plunge_feed),
                Some(&mut o.stepdown),
                None,
                &mut o.tool_preset,
            ),
            Self::Pocket(o) => (
                &mut o.clear.feed,
                Some(&mut o.clear.plunge_feed),
                Some(&mut o.clear.stepdown),
                Some(&mut o.clear.overlap),
                &mut o.tool_preset,
            ),
            Self::Parallel(o) => (
                &mut o.params.feed,
                Some(&mut o.params.plunge_feed),
                None,
                None,
                &mut o.tool_preset,
            ),
            Self::Spiral(o) => (
                &mut o.params.feed,
                Some(&mut o.params.plunge_feed),
                None,
                None,
                &mut o.tool_preset,
            ),
            Self::Adaptive(o) => (
                &mut o.params.feed,
                Some(&mut o.params.plunge_feed),
                Some(&mut o.params.stepdown),
                None,
                &mut o.tool_preset,
            ),
            Self::Face(o) => (
                &mut o.feed,
                Some(&mut o.plunge_feed),
                Some(&mut o.stepdown),
                Some(&mut o.overlap),
                &mut o.tool_preset,
            ),
            // A drilling feed is axial: use the preset's plunge feed.
            Self::Drill(o) => {
                if let Some(step) = preset.stepdown {
                    o.peck = Some(step);
                }
                (&mut o.feed, None, None, None, &mut o.tool_preset)
            }
            Self::Chamfer(o) => (
                &mut o.feed,
                Some(&mut o.plunge_feed),
                Some(&mut o.stepdown), // Modeled-face bands; legacy edge mode uses width steps.
                None,
                &mut o.tool_preset,
            ),
            Self::Bore(o) => (
                &mut o.feed,
                Some(&mut o.plunge_feed),
                None,
                None,
                &mut o.tool_preset,
            ),
            Self::Thread(o) => (
                &mut o.feed,
                Some(&mut o.plunge_feed),
                None,
                None,
                &mut o.tool_preset,
            ),
            Self::Engrave(o) => (
                &mut o.feed,
                Some(&mut o.plunge_feed),
                Some(&mut o.stepdown),
                None,
                &mut o.tool_preset,
            ),
            Self::Carve(o) => (
                &mut o.feed,
                Some(&mut o.plunge_feed),
                None,
                None,
                &mut o.tool_preset,
            ),
        };
        *feed = if plunge.is_none() {
            preset.plunge_feed
        } else {
            preset.feed
        };
        if let Some(value) = plunge {
            *value = preset.plunge_feed;
        }
        if let (Some(value), Some(v)) = (stepdown, preset.stepdown) {
            *value = v;
        }
        if let (Some(value), Some(v)) = (overlap, preset.overlap) {
            *value = v;
        }
        *slot = Some(preset.clone());
        if preset.overlap.is_some() {
            match self {
                Self::Pocket(o) => {
                    o.maximum_stepover = None;
                    o.clear.pocket_overlap = None;
                    use crate::OperationParameter::{MaximumStepover, Overlap, PocketOverlap};
                    for parameter in [MaximumStepover, Overlap, PocketOverlap] {
                        o.parameter_expressions.remove(&parameter);
                    }
                }
                Self::Face(o) => {
                    o.absolute_overlap = None;
                    o.parameter_expressions
                        .remove(&crate::OperationParameter::FaceOverlap);
                }
                _ => {}
            }
        }
        Ok(())
    }
}
