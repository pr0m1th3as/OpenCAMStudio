//! Nominal milling load from effective kc and an assumed rectangular cut section.
//! See docs/cutting-forces.md for the metric equations and their limitations.
use crate::{Operation, Tool, ToolKind};

/// Equivalent mean milling load from either nominal or simulated removal rate.
/// Force is referred to the supplied cutting diameter; torque is independent of it.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CuttingLoad {
    pub removal_rate_mm3_min: f64,
    pub tangential_force_n: f64,
    pub cutting_power_w: f64,
    pub spindle_torque_nm: f64,
}
impl CuttingLoad {
    pub fn from_removal_rate(
        removal_rate_mm3_min: f64,
        specific_cutting_force: f64,
        spindle_rpm: f64,
        cutting_diameter_mm: f64,
    ) -> Result<Self, String> {
        if !removal_rate_mm3_min.is_finite() || removal_rate_mm3_min < 0. {
            return Err("Removal rate must be finite and nonnegative.".into());
        }
        positive(specific_cutting_force, "Specific cutting force")?;
        positive(spindle_rpm, "Spindle speed")?;
        positive(cutting_diameter_mm, "Cutting diameter")?;
        let cutting_power_w = removal_rate_mm3_min * specific_cutting_force / 60_000.;
        let speed = std::f64::consts::PI * cutting_diameter_mm * spindle_rpm / 60_000.;
        positive(speed, "Cutting speed")?;
        let tangential_force_n = cutting_power_w / speed;
        let spindle_torque_nm = cutting_power_w / spindle_rpm * 60. / std::f64::consts::TAU;
        if [cutting_power_w, tangential_force_n, spindle_torque_nm]
            .iter()
            .any(|v| !v.is_finite())
        {
            return Err("Calculated cutting load overflowed.".into());
        }
        Ok(Self {
            removal_rate_mm3_min,
            tangential_force_n,
            cutting_power_w,
            spindle_torque_nm,
        })
    }
}

/// Assumed engagement for a steady milling cut, all dimensions in mm.
/// The effective diameter may be smaller than the nominal diameter of a shaped tool.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CuttingEngagement {
    pub axial_depth_mm: f64,
    pub radial_width_mm: f64,
    pub cutting_diameter_mm: f64,
}

impl CuttingEngagement {
    pub fn validate(self) -> Result<(), String> {
        positive(self.axial_depth_mm, "Axial engagement")?;
        positive(self.radial_width_mm, "Radial engagement")?;
        positive(self.cutting_diameter_mm, "Effective cutting diameter")?;
        if self.radial_width_mm > self.cutting_diameter_mm {
            return Err("Radial engagement must not exceed the effective cutting diameter.".into());
        }
        Ok(())
    }

    pub fn validate_for_tool(self, tool: &Tool) -> Result<(), String> {
        self.validate()?;
        positive(tool.diameter, "Tool diameter")?;
        if self.cutting_diameter_mm > tool.diameter {
            return Err(
                "Effective cutting diameter must not exceed the cutter's nominal diameter.".into(),
            );
        }
        Ok(())
    }

    /// Effective kc is supplied by the user for the actual chip thickness/rake;
    /// this deliberately does not treat reference kc1 as an effective coefficient.
    pub fn estimate(
        self,
        specific_cutting_force: f64,
        feed_mm_min: f64,
        spindle_rpm: f64,
    ) -> Result<CuttingForceEstimate, String> {
        self.validate()?;
        positive(specific_cutting_force, "Specific cutting force")?;
        positive(feed_mm_min, "Cutting feed")?;
        positive(spindle_rpm, "Spindle speed")?;
        let removal_rate_mm3_min = self.axial_depth_mm * self.radial_width_mm * feed_mm_min;
        let cutting_speed_m_min =
            std::f64::consts::PI * self.cutting_diameter_mm * spindle_rpm / 1000.;
        let cutting_power_w = removal_rate_mm3_min * specific_cutting_force / 60_000.;
        let tangential_force_n = cutting_power_w * 60. / cutting_speed_m_min;
        let spindle_torque_nm = tangential_force_n * self.cutting_diameter_mm / 2000.;
        for value in [
            removal_rate_mm3_min,
            cutting_speed_m_min,
            cutting_power_w,
            tangential_force_n,
            spindle_torque_nm,
        ] {
            positive(value, "Calculated load (check input magnitudes)")?;
        }
        Ok(CuttingForceEstimate {
            engagement: self,
            specific_cutting_force,
            feed_mm_min,
            spindle_rpm,
            removal_rate_mm3_min,
            cutting_speed_m_min,
            tangential_force_n,
            cutting_power_w,
            spindle_torque_nm,
        })
    }
}

/// Revolution-average equivalent tangential load during the assumed steady cut.
/// Neither a peak tooth load nor a Cartesian/axial force prediction.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CuttingForceEstimate {
    pub engagement: CuttingEngagement,
    pub specific_cutting_force: f64,
    pub feed_mm_min: f64,
    pub spindle_rpm: f64,
    pub removal_rate_mm3_min: f64,
    pub cutting_speed_m_min: f64,
    pub tangential_force_n: f64,
    pub cutting_power_w: f64,
    pub spindle_torque_nm: f64,
}

impl Operation {
    /// Concrete operation feed; never read a mutable tool preset for an estimate.
    pub fn cutting_feed_mm_min(&self) -> f64 {
        match self {
            Self::Profile(o) => o.feed,
            Self::Pocket(o) => o.clear.feed,
            Self::Adaptive(o) => o.params.feed,
            Self::Parallel(o) => o.params.feed,
            Self::Spiral(o) => o.params.feed,
            Self::Face(o) => o.feed,
            Self::Drill(o) => o.feed,
            Self::Chamfer(o) => o.feed,
            Self::Bore(o) => o.feed,
            Self::Thread(o) => o.feed,
            Self::Engrave(o) => o.feed,
            Self::Carve(o) => o.feed,
        }
    }

    /// A nominal roughing-pass engagement, not stock-contact simulation.
    /// `cutting_depth_mm` is the resolved top minus bottom height, including height overrides.
    pub fn nominal_cutting_engagement(
        &self,
        tool: &Tool,
        cutting_depth_mm: f64,
    ) -> Result<(CuttingEngagement, &'static str), String> {
        if matches!(self, Self::Drill(_)) {
            return Err(
                "Drilling thrust requires a drill-specific model; milling force is unavailable."
                    .into(),
            );
        }
        if !matches!(
            tool.kind,
            ToolKind::EndMill | ToolKind::RoughingMill | ToolKind::FaceMill
        ) {
            return Err(
                "Enter custom engagement and effective diameter for this shaped cutter.".into(),
            );
        }
        positive(tool.diameter, "Tool diameter")?;
        let overlap_width = |overlap: f64| -> Result<f64, String> {
            if !overlap.is_finite() || !(0. ..1.).contains(&overlap) {
                return Err("Overlap must be between 0% (inclusive) and 100% (exclusive).".into());
            }
            Ok(tool.diameter * (1. - overlap))
        };
        let step_depth = |total: f64, step: f64| -> Result<f64, String> {
            positive(total, "Total cutting depth")?;
            positive(step, "Stepdown")?;
            Ok(total.min(step))
        };
        let (axial, radial, basis) = match self {
            Self::Profile(o) => {
                if !o.stepover.is_finite() || o.stepover < 0. {
                    return Err("Profile stepover must be finite and nonnegative.".into());
                }
                let axial = step_depth(cutting_depth_mm, if o.passes.multiple_depths { o.stepdown } else { cutting_depth_mm })?;
                if o.stepover > 0. {
                    (axial, o.stepover.min(tool.diameter), "Nominal roughing pass: stepdown and radial stepover; entry and finishing loads may differ.")
                } else {
                    (axial, tool.diameter, "Assumes full-width slotting at the pass stepdown; side profiling may remove less material.")
                }
            }
            Self::Pocket(o) => {
                let cap = if o.clear.pocket_adaptive_enabled() { o.clear.clearing.engagement } else { 0.0 };
                if !cap.is_finite() {
                    return Err("Pocket engagement must be finite.".into());
                }
                let radial = if cap > 0. {
                    cap.min(tool.diameter)
                } else {
                    let stepover = o.concentric_stepover(tool.diameter);
                    crate::PocketOp::validate_maximum_stepover(stepover, tool.diameter)?;
                    stepover
                };
                (step_depth(cutting_depth_mm, if o.clear.multiple_depths.enabled { o.clear.stepdown } else { cutting_depth_mm })?, radial,
                 "Nominal clearing pass: stepdown and ring spacing or adaptive engagement cap; entries, corners and finishing may differ.")
            }
            Self::Face(o) => (step_depth(cutting_depth_mm, if o.multiple_depths.enabled { o.stepdown } else { cutting_depth_mm })?, if let Some(overlap) = o.absolute_overlap {
                if !overlap.is_finite() || overlap < 0.0 || overlap >= tool.diameter { return Err("Facing overlap must be nonnegative and less than the tool diameter.".into()); }
                tool.diameter - overlap
            } else { overlap_width(o.overlap)? },
                "Nominal facing pass: stepdown and diameter × (1 − overlap)."),
            Self::Adaptive(o) => {
                positive(o.params.optimal_load, "Optimal load")?;
                (step_depth(cutting_depth_mm, o.params.stepdown)?, o.params.optimal_load.min(tool.diameter),
                 "Assumes the configured roughing stepdown and optimal load; shallow layers and rest passes may remove less.")
            }
            _ => return Err("Contact depends on the geometry. Enter custom axial/radial engagement and effective cutting diameter.".into()),
        };
        let engagement = CuttingEngagement {
            axial_depth_mm: axial,
            radial_width_mm: radial,
            cutting_diameter_mm: tool.diameter,
        };
        engagement.validate()?;
        Ok((engagement, basis))
    }
}

fn positive(value: f64, label: &str) -> Result<(), String> {
    if value.is_finite() && value > 0. {
        Ok(())
    } else {
        Err(format!("{label} must be finite and positive."))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn engagement() -> CuttingEngagement {
        CuttingEngagement {
            axial_depth_mm: 2.,
            radial_width_mm: 3.,
            cutting_diameter_mm: 10.,
        }
    }

    #[test]
    fn metric_load_matches_power_and_torque_identities() {
        let e = engagement().estimate(1500., 1000., 10_000.).unwrap();
        assert_eq!(e.removal_rate_mm3_min, 6000.);
        assert_eq!(e.cutting_power_w, 150.);
        assert!((e.tangential_force_n - 28.64788975654116).abs() < 1e-10);
        assert!((e.spindle_torque_nm - 0.1432394487827058).abs() < 1e-10);
        assert!(
            (e.spindle_torque_nm * 2. * std::f64::consts::PI * e.spindle_rpm / 60.
                - e.cutting_power_w)
                .abs()
                < 1e-10
        );
    }

    #[test]
    fn force_scales_with_feed_and_kc_and_inversely_with_rpm() {
        let base = engagement().estimate(1500., 1000., 10_000.).unwrap();
        let feed = engagement().estimate(1500., 2000., 10_000.).unwrap();
        let speed = engagement().estimate(1500., 1000., 20_000.).unwrap();
        let kc = engagement().estimate(3000., 1000., 10_000.).unwrap();
        assert_eq!(feed.tangential_force_n, base.tangential_force_n * 2.);
        assert_eq!(kc.tangential_force_n, base.tangential_force_n * 2.);
        assert_eq!(speed.tangential_force_n, base.tangential_force_n / 2.);
        assert_eq!(speed.cutting_power_w, base.cutting_power_w);
    }

    #[test]
    fn measured_removal_load_matches_nominal_and_zero_removal_is_zero_force() {
        let nominal = engagement().estimate(1500., 1000., 10000.).unwrap();
        let measured = CuttingLoad::from_removal_rate(6000., 1500., 10000., 10.).unwrap();
        assert_eq!(measured.cutting_power_w, nominal.cutting_power_w);
        assert!((measured.tangential_force_n - nominal.tangential_force_n).abs() < 1e-10);
        assert!((measured.spindle_torque_nm - nominal.spindle_torque_nm).abs() < 1e-10);
        let air = CuttingLoad::from_removal_rate(0., 1500., 10000., 10.).unwrap();
        assert_eq!(air.tangential_force_n, 0.);
        assert_eq!(air.cutting_power_w, 0.);
        for bad in [-1., f64::NAN, f64::INFINITY] {
            assert!(CuttingLoad::from_removal_rate(bad, 1500., 10000., 10.).is_err());
        }
        assert!(CuttingLoad::from_removal_rate(1., 1500., 0., 10.).is_err());
        assert!(CuttingLoad::from_removal_rate(1., 0., 10000., 10.).is_err());
        assert!(CuttingLoad::from_removal_rate(f64::MAX, f64::MAX, 10000., 10.).is_err());
    }

    #[test]
    fn invalid_inputs_and_overflow_are_rejected() {
        for bad in [0., -1., f64::NAN, f64::INFINITY] {
            assert!(engagement().estimate(bad, 1000., 10_000.).is_err());
            assert!(engagement().estimate(1500., bad, 10_000.).is_err());
            assert!(engagement().estimate(1500., 1000., bad).is_err());
            for e in [
                CuttingEngagement {
                    axial_depth_mm: bad,
                    ..engagement()
                },
                CuttingEngagement {
                    radial_width_mm: bad,
                    ..engagement()
                },
                CuttingEngagement {
                    cutting_diameter_mm: bad,
                    ..engagement()
                },
            ] {
                assert!(e.estimate(1500., 1000., 10_000.).is_err());
            }
        }
        assert!(engagement().estimate(f64::MAX, f64::MAX, 1.).is_err());
        assert!(CuttingEngagement {
            radial_width_mm: 11.,
            ..engagement()
        }
        .validate()
        .is_err());
        let tool = Tool {
            diameter: 10.,
            ..Default::default()
        };
        assert!(engagement().validate_for_tool(&tool).is_ok());
        assert!(CuttingEngagement {
            cutting_diameter_mm: 11.,
            ..engagement()
        }
        .validate_for_tool(&tool)
        .is_err());
    }

    #[test]
    fn legacy_materials_remain_unset_and_coefficients_round_trip() {
        let mut m: crate::Material = serde_json::from_str(r#"{"id":1,"name":"Steel"}"#).unwrap();
        assert_eq!(m.specific_cutting_force, None);
        m.validate().unwrap();
        m.specific_cutting_force = Some(1800.);
        assert_eq!(
            serde_json::from_str::<crate::Material>(&serde_json::to_string(&m).unwrap()).unwrap(),
            m
        );
        for bad in [0., -10., f64::NAN, f64::INFINITY] {
            m.specific_cutting_force = Some(bad);
            assert!(m.validate().is_err());
        }
    }
}
