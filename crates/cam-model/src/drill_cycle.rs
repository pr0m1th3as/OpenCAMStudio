use crate::DrillOp;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DrillCycleMode {
    RapidOut,
    Counterboring,
    ChipBreaking,
    DeepDrilling,
}

impl DrillCycleMode {
    pub const ALL: [Self; 4] = [
        Self::RapidOut,
        Self::Counterboring,
        Self::ChipBreaking,
        Self::DeepDrilling,
    ];

    pub fn pecks(self) -> bool {
        matches!(self, Self::ChipBreaking | Self::DeepDrilling)
    }
}

impl std::fmt::Display for DrillCycleMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::RapidOut => "Drilling - rapid out",
            Self::Counterboring => "Counterboring - dwell and rapid out",
            Self::ChipBreaking => "Chip breaking - partial retract",
            Self::DeepDrilling => "Deep drilling - full retract",
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct DrillCycleSettings {
    /// Earlier projects inferred the cycle from the optional peck and dwell.
    pub mode: Option<DrillCycleMode>,
    pub peck_reduction: f64,
    pub minimum_peck: f64,
    pub accumulated_peck: f64,
    pub chip_break_distance: f64,
}

impl Default for DrillCycleSettings {
    fn default() -> Self {
        Self {
            mode: None,
            peck_reduction: 0.0,
            minimum_peck: Self::DEFAULT_PECK,
            accumulated_peck: 7.5,
            chip_break_distance: 0.1,
        }
    }
}

impl DrillCycleSettings {
    pub const DEFAULT_PECK: f64 = 0.75;

    pub(crate) fn is_default(&self) -> bool {
        *self == Self::default()
    }
}

impl DrillOp {
    pub fn cycle_mode(&self) -> DrillCycleMode {
        self.cycle.mode.unwrap_or(match (self.peck, self.dwell) {
            (Some(_), _) => DrillCycleMode::DeepDrilling,
            (None, Some(_)) => DrillCycleMode::Counterboring,
            _ => DrillCycleMode::RapidOut,
        })
    }

    pub fn set_cycle_mode(&mut self, mode: DrillCycleMode) {
        if mode.pecks() {
            let peck = self.peck.get_or_insert(DrillCycleSettings::DEFAULT_PECK);
            if self.cycle.mode.is_none() {
                self.cycle.minimum_peck = self.cycle.minimum_peck.min(*peck);
            }
        }
        self.cycle.mode = Some(mode);
    }

    pub fn set_peck(&mut self, value: f64) {
        self.peck =
            (value > 0. || self.cycle.mode.is_some_and(DrillCycleMode::pecks)).then_some(value);
    }

    pub fn minimum_peck(&self) -> f64 {
        match (self.cycle.mode, self.peck) {
            (None, Some(peck)) => self.cycle.minimum_peck.min(peck),
            _ => self.cycle.minimum_peck,
        }
    }
}
