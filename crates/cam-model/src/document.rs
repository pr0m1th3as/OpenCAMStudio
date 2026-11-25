//! The document model (P3 first-light slice).
//!
//! A [`Document`] carries a schema version and one [`Setup`]. A [`Setup`] fixes
//! the **heights** (the safety planes), a **stock** description, the available
//! tools, and an ordered list of **operations**. This is deliberately small —
//! enough to drive first light (a profile → G-code) — and grows toward the full
//! `Project → Setup → Stock → Operation → Tool` tree as later phases need it.

use crate::ParallelParams;
use cam_geo::Contour;

use crate::{BoreOp, Tool};

/// The document schema version. Bumped when the on-disk model format changes;
/// present from the start so save-files are versioned before there is a loader.
/// v2: `ToolKind` became a data-carrying enum (per-kind geometry).
/// v3: `Stock` became a part-relative spec (offsets + top + thickness).
/// v4: `Setup` gained a workpiece `origin` (datum) + optional `start_offset`.
/// v5: `Tool` gained nominal cutting data (`nominal_rpm`/`nominal_feed`/
///     `nominal_plunge_feed`) and each `Operation` gained a per-op `spindle_rpm`.
///     All are `#[serde(default)]`, so this bump is a record, not a load barrier —
///     v4 and earlier files still open (the new fields default to 0).
/// v6: each `Operation` gained a per-op `work_offset` — the 1-based work-datum
///     index for multi-WCS output (Okuma `G15 H<n>`). `#[serde(default =
///     "default_work_offset")]` → 1, so v5 and earlier files still open on datum 1.
/// v7: `Setup` gained a work-datum `work_offsets` registry + `replication` order.
///     **Superseded by v8** — those fields were removed before any public release.
/// v8: `Setup` gained `extra_origins` (`Vec<Origin>`): additional workpiece origins
///     for re-fixturing/reorientation, each with its own `index`, `position` and
///     optional `start_offset` (a program start point per orientation), emitted under
///     its own `G15 H<index>`; plus `origin_index` (the base origin's editable `H<n>`,
///     default 1). All `#[serde(default)]`, so v6-and-earlier files (base origin only,
///     H1) open unchanged; the base position/start-point stay in `Setup::origin` /
///     `Setup::start_offset`.
/// v9: `Setup` gained `tool_change_height` (`Option<f64>`): the Z the planner lifts to
///     before a tool change (`M6`) or a manual reorientation (`M00`), above the
///     toolpath `clearance` plane. `#[serde(default)]` → `None`, which resolves to the
///     machine's max Z travel at plan time, so v8-and-earlier files open unchanged.
///     **Also removes** the v4/v8 per-origin program start point (`Setup::start_offset`
///     and `Origin::start_offset`, both `Option<[f64;3]>`): superseded by the
///     unconditional lift-to-tool-change-height then descend-to-clearance that now opens
///     every operation group. Old files still open — serde drops the now-unknown field,
///     so a saved start point becomes "start straight into the first op" (a safe no-op).
/// v10: `PocketOp`'s flat clearing fields (`stepdown`/`overlap`/`offset`/`feed`/
///     `plunge_feed`/`plunge`/`lead_in`/`lead_out`/`lead_overlap`/`clearing`) moved into
///     a nested [`ClearParams`] under `clear`, the struct [`CarveOp`] already used, so
///     one type states what an area-clearing pass takes.
///     **The first bump that is not additive**, and so the first a saved file cannot
///     absorb by serde defaults alone — a flat `stepdown` has to become a `stepdown`
///     inside a `clear` object, which no serde attribute expresses. `schema_version` had
///     been written into every save since v1 and read by nothing; from here it drives
///     [`crate::migrate`], which rewrites the JSON tree before it is deserialized.
/// v11: the saved project gained the **machine** it was built for and the **post** it
///     was built to export through, both previously session-only, so reopening a job
///     does not silently retarget it at whatever control the application last had
///     selected. Both are `Option`, defaulting to absent — a pre-v11 project says
///     nothing about either and leaves the session's own choices alone, which is the
///     only honest reading of a file that never recorded them.
///     Additive, so the step is an identity; the bump is a record of a **file-format**
///     change, since the version numbers the save-file as a whole and not the
///     [`Document`] alone. [`Machine`](crate::Machine) and `PostKind` became
///     serializable to carry it.
/// v12: [`ThreadOp`] gained `gradual` — infeed passes sized for equal *material* rather
///     than equal radial steps, the control [`ChamferOp`] already had. Additive and
///     `false` by default, which is the behaviour every existing file already has, so
///     the step is an identity and no saved thread changes.
/// v13: [`CarveOp`] gained `plunge` — the V-bit's own Z-entry style, which it never had
///     (its clearing pass did, via [`ClearParams`], and the asymmetry was invisible until
///     an export showed seventeen straight plunges up to 3 mm deep). Additive and
///     [`Plunge::Straight`] by default, which is precisely what every existing carve
///     already does, so the step is an identity and no saved carve changes.
/// v14: persistent CAD assets, exact topology snapshots, instances and operation
/// bindings. Legacy operations retain their explicit embedded geometry.
/// v15: projects contain independently framed setups; old single setups are wrapped.
/// v16: manufacturing suppression and stable ID high-water marks persist.
/// v17: per-operation safety planes, with setup heights as legacy defaults.
/// v18: reference-plus-offset definitions for all five operation heights.
/// v19: setup model lists are explicit; an empty list selects no solid bodies.
/// v20: profiles support open chains, multiple contours, and complete silhouettes.
/// v21: named profile geometry entries with independent extraction settings.
/// v22: managed stock material IDs, tool presets and operation preset snapshots.
/// v23: setup-scoped reusable operation patterns.
/// v24: modeled chamfer-face boundaries and axial finishing bands. Older readers
/// must refuse these operations rather than treating their projected face as an edge.
/// v25: open chamfer chains; absent closed flags retain legacy closed rims.
/// v26: adds setup-scoped StockToLeave allowances keyed by operation ID and the
/// Adaptive operation with its `AdaptiveClearing` geometry recipe. Older projects
/// omit allowances and retain their existing offset and depth behavior.
/// v27: introduced an optional adaptive simplification tolerance. That legacy
/// member is now ignored; simplification is derived from the operation tolerance.
/// v28: profile pass controls, tab shapes/counts, and one controller compensation method.
/// v29: helical Bore operations and cylindrical Bore geometry bindings.
/// v30: 3D Parallel finishing and CAD surface bindings.
/// v31: remaining stock inherited from the immediately preceding setup.
/// v32: operation pattern spacing, directions and execution options.
/// v33: a distinct roughing end mill tool type.
/// v34: surface-offset finishing mode.
/// v35: Scallop entry and retraction preferences.
/// v36: Scallop pass ordering.
/// v37: setup fixture bodies and radial/axial fixture clearances.
/// v38: multiple pocket regions sharing operation cutting heights.
/// v39: optional stock contours for pocket and 2D adaptive material footprints.
/// v42: round, tubular and solid-derived stock definitions.
/// v43: optional edge-aligned round stock.
/// v44: round stock supports axes through the setup origin.
/// v45: fixed fourth-axis indexes on individual operations.
/// v46: concave radius mill tool geometry.
/// v47: persistent numeric-operation parameter formulas.
/// v48: 3D Spiral finishing.
/// v49: Parallel cutting turns and repositioning policy.
/// v50: Parallel transition type.
/// v51: groove cutter and slitting saw tool types.
pub const SCHEMA_VERSION: u32 = 51;

/// How repeated instances are traversed when executing a pattern.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PatternOrder {
    #[default]
    ByInstance,
    ByOperation,
    ByTool,
}

/// Interpret a linear distance as an adjacent pitch or an overall extent.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PatternSpacing {
    #[default]
    Spacing,
    Extent,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LinearPatternDirection {
    pub angle_degrees: f64,
    #[serde(default)]
    pub reversed: bool,
    #[serde(default)]
    pub symmetric: bool,
    /// Selected edge, retained alongside the resolved setup-XY angle.
    #[serde(default)]
    pub reference: Option<crate::EntityRef>,
}

/// Additive settings; defaults reproduce legacy X/Y patterns exactly.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct PatternOptions {
    pub spacing: PatternSpacing,
    pub directions: [LinearPatternDirection; 2],
    pub second_direction: bool,
    pub keep_original: bool,
    pub reverse: bool,
    /// A picked circular edge or straight axis parallel to setup Z.
    pub axis: Option<crate::EntityRef>,
}

impl Default for PatternOptions {
    fn default() -> Self {
        Self {
            spacing: PatternSpacing::Spacing,
            directions: [0.0, 90.0].map(|angle_degrees| LinearPatternDirection {
                angle_degrees,
                reversed: false,
                symmetric: false,
                reference: None,
            }),
            second_direction: true,
            keep_original: true,
            reverse: false,
            axis: None,
        }
    }
}

impl PatternOptions {
    pub fn validate(&self, kind: PatternKind) -> Result<(), String> {
        kind.validate()?;
        if self.directions.iter().any(|d| !d.angle_degrees.is_finite()) {
            return Err("pattern direction angles must be finite".into());
        }
        if let PatternKind::Rectangular { nx, ny, dx, dy } = kind {
            if !self.second_direction && ny != 1 {
                return Err("disabled second direction requires a count of one".into());
            }
            let angle =
                (self.directions[1].angle_degrees - self.directions[0].angle_degrees).to_radians();
            if nx > 1 && ny > 1 && angle.sin().abs() < 1e-9 {
                return Err("the two pattern directions must not be parallel".into());
            }
            let scale = |count: u32| {
                if self.spacing == PatternSpacing::Extent {
                    1.0
                } else {
                    count.saturating_sub(1) as f64
                }
            };
            if !(dx.abs() * scale(nx) + dy.abs() * scale(ny)).is_finite() {
                return Err("pattern extent exceeds finite coordinate limits".into());
            }
        }
        Ok(())
    }

    /// Stable placement IDs: zero is the source, even when execution is reversed.
    pub fn instance_indices(&self, kind: PatternKind) -> Vec<usize> {
        let count = kind.instance_count().unwrap_or(0).min(10_000);
        let mut indices: Vec<_> = (usize::from(!self.keep_original)..count).collect();
        if self.reverse {
            indices.reverse();
        }
        indices
    }

    pub fn transform_xy(&self, kind: PatternKind, index: usize) -> Option<([f64; 2], f64)> {
        let PatternKind::Rectangular { nx, ny, dx, dy } = kind else {
            return kind.transform_xy(index);
        };
        if nx == 0 || index >= (nx as usize).checked_mul(ny as usize)? {
            return None;
        }
        let mut offset = [0.0; 2];
        for (axis, (i, count, distance)) in
            [(index % nx as usize, nx, dx), (index / nx as usize, ny, dy)]
                .into_iter()
                .enumerate()
        {
            let direction = &self.directions[axis];
            let step = if direction.symmetric && i > 0 {
                if i % 2 == 1 {
                    i.div_ceil(2) as f64
                } else {
                    -(i as f64 / 2.0)
                }
            } else {
                i as f64
            };
            let pitch = if self.spacing == PatternSpacing::Extent && count > 1 {
                distance / (count - 1) as f64
            } else {
                distance
            };
            let length = step * pitch * if direction.reversed { -1.0 } else { 1.0 };
            let (sin, cos) = direction.angle_degrees.to_radians().sin_cos();
            let sin = if sin.abs() < 1e-15 { 0.0 } else { sin };
            let cos = if cos.abs() < 1e-15 { 0.0 } else { cos };
            offset[0] += length * cos;
            offset[1] += length * sin;
        }
        Some((offset, 0.0))
    }
}

/// Radial wall allowance and axial floor allowance for an operation, in mm.
/// Disabled settings are ignored; zero allowances preserve the operation's
/// existing offsets and generated toolpath exactly.
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StockToLeave {
    /// Whether the allowances are active.
    #[serde(default)]
    pub enabled: bool,
    /// Material left on vertical walls and the XY perimeter.
    #[serde(default)]
    pub radial: f64,
    /// Material left on horizontal floors, measured along setup Z.
    #[serde(default)]
    pub axial: f64,
}

impl StockToLeave {
    /// Active allowances, or zero when disabled.
    pub fn effective(self) -> (f64, f64) {
        if self.enabled {
            (self.radial, self.axial)
        } else {
            (0.0, 0.0)
        }
    }

    /// Validate allowance magnitudes before they reach offset and depth math.
    pub fn validate(self) -> Result<(), String> {
        if !self.radial.is_finite() || self.radial < 0.0 {
            return Err("stock to leave radial allowance must be finite and non-negative".into());
        }
        if !self.axial.is_finite() || self.axial < 0.0 {
            return Err("stock to leave axial allowance must be finite and non-negative".into());
        }
        Ok(())
    }
}

/// Direction used by circular patterns, viewed from setup +Z.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PatternDirection {
    #[default]
    Ccw,
    Cw,
}

/// Distribution of circular pattern instances.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum CircularDistribution {
    Full,
    Partial { sweep_degrees: f64 },
    Spacing { angle_degrees: f64 },
}

/// Geometry and placement parameters for setup XY operation repetition.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum PatternKind {
    Rectangular {
        nx: u32,
        ny: u32,
        dx: f64,
        dy: f64,
    },
    Circular {
        center_xy: [f64; 2],
        count: u32,
        distribution: CircularDistribution,
        direction: PatternDirection,
    },
}

/// A setup-level group that repeats its editable source operations.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct OperationPattern {
    pub id: u64,
    pub name: String,
    /// Ordered source operation IDs; they are a contiguous block in Setup.operations.
    pub members: Vec<u32>,
    pub kind: PatternKind,
    #[serde(default)]
    pub order: PatternOrder,
    #[serde(default)]
    pub suppressed: bool,
    #[serde(default)]
    pub options: PatternOptions,
}

impl PatternKind {
    pub fn instance_count(self) -> Option<usize> {
        match self {
            Self::Rectangular { nx, ny, .. } => (nx as usize).checked_mul(ny as usize),
            Self::Circular { count, .. } => Some(count as usize),
        }
    }
    /// Transform coordinates for an instance, with index zero always identity.
    pub fn transform_xy(self, index: usize) -> Option<([f64; 2], f64)> {
        match self {
            Self::Rectangular { nx, ny, dx, dy } => {
                if index >= (nx as usize).checked_mul(ny as usize)? {
                    return None;
                }
                let x = index % nx as usize;
                let y = index / nx as usize;
                Some(([x as f64 * dx, y as f64 * dy], 0.0))
            }
            Self::Circular {
                count,
                distribution,
                direction,
                ..
            } => {
                if index >= count as usize {
                    return None;
                }
                let sweep = match distribution {
                    CircularDistribution::Full => 360.0,
                    CircularDistribution::Partial { sweep_degrees } => sweep_degrees,
                    CircularDistribution::Spacing { angle_degrees } => angle_degrees,
                };
                let divisor = match distribution {
                    CircularDistribution::Full => count as f64,
                    CircularDistribution::Partial { .. } => count.saturating_sub(1) as f64,
                    CircularDistribution::Spacing { .. } => 1.0,
                };
                if divisor == 0.0 {
                    return None;
                }
                let sign = if direction == PatternDirection::Ccw {
                    1.0
                } else {
                    -1.0
                };
                Some(([0.0, 0.0], sign * sweep * index as f64 / divisor))
            }
        }
    }
    pub fn validate(self) -> Result<(), String> {
        match self {
            Self::Rectangular { nx, ny, dx, dy } => {
                let n = (nx as usize)
                    .checked_mul(ny as usize)
                    .ok_or("pattern instance count overflows")?;
                if nx == 0 || ny == 0 || !(2..=10_000).contains(&n) {
                    return Err(
                        "rectangular pattern requires counts >= 1 and 2..=10,000 instances".into(),
                    );
                }
                if !dx.is_finite()
                    || !dy.is_finite()
                    || (nx > 1 && dx == 0.0)
                    || (ny > 1 && dy == 0.0)
                {
                    return Err("active rectangular pitches must be finite and nonzero".into());
                }
            }
            Self::Circular {
                center_xy,
                count,
                distribution,
                ..
            } => {
                if !(2..=10_000).contains(&count) || !center_xy.iter().all(|v| v.is_finite()) {
                    return Err(
                        "circular pattern requires 2..=10,000 instances and a finite center".into(),
                    );
                }
                if let CircularDistribution::Partial { sweep_degrees } = distribution {
                    if !sweep_degrees.is_finite() || sweep_degrees <= 0.0 || sweep_degrees >= 360.0
                    {
                        return Err(
                            "partial sweep must be greater than 0 and less than 360 degrees".into(),
                        );
                    }
                }
                if let CircularDistribution::Spacing { angle_degrees } = distribution {
                    if !angle_degrees.is_finite()
                        || angle_degrees <= 0.0
                        || angle_degrees * (count - 1) as f64 >= 360.0
                    {
                        return Err(
                            "angular spacing must be positive and span less than 360 degrees"
                                .into(),
                        );
                    }
                }
            }
        }
        Ok(())
    }
}

/// Operation safety planes (also used as legacy setup defaults), all
/// **absolute Z in millimetres**. By
/// convention WCS Z0 is the top of stock, so `top_of_stock` is usually `0.0`.
///
/// Heights are first-class (a core design rule): unsafe Z is a primary hazard
/// and must never be implicit.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Heights {
    /// Z for rapid traverses between features — the highest, safest plane.
    pub clearance: f64,
    /// Z to retract to between passes within a feature (lower than clearance).
    pub retract: f64,
    /// Z of the top of the stock (cutting starts here).
    pub top_of_stock: f64,
}

impl Heights {
    /// Validate finite, ordered absolute safety planes. Negative Z is valid.
    pub fn validate(&self) -> Result<(), String> {
        if ![self.clearance, self.retract, self.top_of_stock]
            .iter()
            .all(|v| v.is_finite())
            || self.clearance < self.retract
            || self.retract < self.top_of_stock
        {
            return Err(
                "Clearance must be at or above retract, and retract at or above stock top".into(),
            );
        }
        Ok(())
    }

    /// A sensible default: clearance and retract above a stock top at Z0.
    pub fn new(clearance: f64, retract: f64, top_of_stock: f64) -> Self {
        Self {
            clearance,
            retract,
            top_of_stock,
        }
    }
}

/// A straight CAD edge whose direction defines the axis of round stock.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct StockAxisReference {
    pub asset_id: u64,
    pub revision: u64,
    pub instance_id: u64,
    pub body_id: u64,
    pub edge_id: u64,
}
impl From<&crate::EntityRef> for StockAxisReference {
    fn from(entity: &crate::EntityRef) -> Self {
        Self {
            asset_id: entity.asset_id,
            revision: entity.revision,
            instance_id: entity.instance_id,
            body_id: entity.body_id,
            edge_id: entity.id,
        }
    }
}
impl StockAxisReference {
    pub fn entity(self) -> crate::EntityRef {
        crate::EntityRef {
            asset_id: self.asset_id,
            revision: self.revision,
            instance_id: self.instance_id,
            body_id: self.body_id,
            id: self.edge_id,
            kind: crate::EntityKind::Edge,
        }
    }
}

/// A round-stock direction selected from a CAD edge or the setup origin axes.
/// The untagged edge form preserves projects saved before origin axes were added.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(untagged)]
pub enum StockAxis {
    Edge(StockAxisReference),
    Origin { origin_axis: StockOriginAxis },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum StockOriginAxis {
    X,
    Y,
    Z,
}
impl StockOriginAxis {
    pub fn index(self) -> usize {
        match self {
            Self::X => 0,
            Self::Y => 1,
            Self::Z => 2,
        }
    }
    pub fn from_index(index: usize) -> Option<Self> {
        [Self::X, Self::Y, Self::Z].get(index).copied()
    }
}

/// Input material for a setup: a model-relative box, or the remaining solid
/// simulated from the preceding setup. Document-level bounds resolution accounts
/// for both model selection and the work axes of inherited stock.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Stock {
    /// A block sized from the part's XY bounding box: grown by `x_offset` on both
    /// X sides and `y_offset` on both Y sides, spanning from `top` down by
    /// `thickness` (so the bottom sits at `top − thickness`). All in mm.
    BoundingBox {
        x_offset: f64,
        y_offset: f64,
        top: f64,
        thickness: f64,
    },
    /// A fixed-size box positioned against the model bounds on each axis.
    FixedBox {
        size: [f64; 3],
        model_position: [f64; 3],
    },
    /// A box with independent offsets from each model face. Negative offsets move the
    /// corresponding stock face inward.
    RelativeBox { offsets: [[f64; 3]; 2] },
    /// Round stock parallel to the selected edge or along an origin axis.
    /// When unset, the direction is setup Z.
    FixedRound {
        diameter: f64,
        inner_diameter: f64,
        length: f64,
        position: f64,
        tube: bool,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        axis: Option<StockAxis>,
    },
    /// Round stock fitted around the model bounds with radial and axial allowances.
    RelativeRound {
        radial: f64,
        inner_diameter: f64,
        bottom: f64,
        top: f64,
        tube: bool,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        axis: Option<StockAxis>,
    },
    /// A selected imported solid supplies the stock geometry.
    FromSolid { instance: Option<u64> },
    /// Remaining material after the preceding setup in document execution order.
    /// Resolve its enclosing bounds through `Document::stock_bounds_for`; the
    /// machining worker supplies the actual simulated solid.
    PreviousSetup,
}

impl Stock {
    /// Resolve to an axis-aligned box `(min, max)` (mm) given the part's XY
    /// bounds. Pure — the caller supplies the bounds (which live outside the
    /// document), keeping the stock definition part-relative and auto-fitting.
    pub fn resolve(&self, min_xy: [f64; 2], max_xy: [f64; 2]) -> ([f64; 3], [f64; 3]) {
        match *self {
            Stock::BoundingBox {
                x_offset,
                y_offset,
                top,
                thickness,
            } => (
                [min_xy[0] - x_offset, min_xy[1] - y_offset, top - thickness],
                [max_xy[0] + x_offset, max_xy[1] + y_offset, top],
            ),
            _ => self.resolve_bounds([min_xy[0], min_xy[1], 0.0], [max_xy[0], max_xy[1], 0.0]),
        }
    }

    /// Resolve box stock against all three model bounds in setup coordinates.
    /// `PreviousSetup` needs `Document::stock_bounds_for`; this local method
    /// can only return the supplied bounds for that document-dependent mode.
    pub fn resolve_bounds(&self, min: [f64; 3], max: [f64; 3]) -> ([f64; 3], [f64; 3]) {
        match *self {
            Self::BoundingBox {
                x_offset,
                y_offset,
                top,
                thickness,
            } => (
                [min[0] - x_offset, min[1] - y_offset, top - thickness],
                [max[0] + x_offset, max[1] + y_offset, top],
            ),
            Self::FixedBox {
                size,
                model_position,
            } => {
                let lo = std::array::from_fn(|i| {
                    min[i] - model_position[i] * (size[i] - (max[i] - min[i]))
                });
                (lo, std::array::from_fn(|i| lo[i] + size[i]))
            }
            Self::FixedRound {
                diameter,
                length,
                position,
                ..
            } => {
                let center = [(min[0] + max[0]) * 0.5, (min[1] + max[1]) * 0.5];
                let z = min[2] - position * (length - (max[2] - min[2]));
                (
                    [center[0] - diameter / 2., center[1] - diameter / 2., z],
                    [
                        center[0] + diameter / 2.,
                        center[1] + diameter / 2.,
                        z + length,
                    ],
                )
            }
            Self::RelativeRound {
                radial,
                bottom,
                top,
                ..
            } => {
                let center = [(min[0] + max[0]) * 0.5, (min[1] + max[1]) * 0.5];
                let radius = ((max[0] - min[0]).hypot(max[1] - min[1])) * 0.5 + radial;
                (
                    [center[0] - radius, center[1] - radius, min[2] - bottom],
                    [center[0] + radius, center[1] + radius, max[2] + top],
                )
            }
            Self::FromSolid { .. } | Self::PreviousSetup => (min, max),
            Self::RelativeBox { offsets } => (
                std::array::from_fn(|i| min[i] - offsets[0][i]),
                std::array::from_fn(|i| max[i] + offsets[1][i]),
            ),
        }
    }
}

/// Which side of a profiled chain the tool runs on.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Side {
    /// Tool outside the closed chain (leaves the enclosed region intact) — the
    /// usual choice for cutting a part free of stock.
    Outside,
    /// Tool inside the closed chain (removes the enclosed region) — e.g. opening
    /// a hole or slot to size.
    Inside,
    /// Tool centre exactly on the chain (engraving/scribing).
    On,
}

impl Side {
    /// Every side, in a stable order — for pickers.
    pub const ALL: [Side; 3] = [Side::Outside, Side::Inside, Side::On];
}

impl std::fmt::Display for Side {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Side::Outside => "Outside",
            Side::Inside => "Inside",
            Side::On => "On",
        })
    }
}

/// The hand of a thread — which way the helix winds. Right-hand is the common
/// case (advances into the work when turned clockwise, viewed from the entering
/// end); left-hand is used where a right-hand thread would loosen in service.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Hand {
    /// Right-hand thread.
    Right,
    /// Left-hand thread.
    Left,
}

impl Hand {
    /// Both hands, in a stable order — for pickers.
    pub const ALL: [Hand; 2] = [Hand::Right, Hand::Left];
}

impl std::fmt::Display for Hand {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Hand::Right => "Right-hand",
            Hand::Left => "Left-hand",
        })
    }
}

/// How cutter-radius compensation is applied.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Comp {
    /// Compute the tool-radius offset in software.
    Computed,
    /// Controller compensation; its effective G41/G42 side follows actual travel.
    #[serde(alias = "ControlLeft", alias = "ControlRight")]
    Control,
}

/// How the tool eases onto (lead-in) or off (lead-out) a profiled contour at the
/// start point, to avoid a witness mark from a direct plunge.
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Lead {
    /// No lead — plunge directly onto the contour (the default).
    #[default]
    None,
    /// A straight lead of `length` mm, tangent (collinear) to the contour.
    Linear { length: f64 },
    /// A tangent arc of `radius` mm onto/off the contour.
    Arc { radius: f64 },
    /// A tangent lead with an explicitly chosen sweep (0 < angle <= 170 degrees).
    ArcSweep { radius: f64, angle_deg: f64 },
}

/// How the tool enters the material in Z at the start of each pass.
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Plunge {
    /// A straight vertical plunge (the default; fine for drills/centre-cutting mills).
    #[default]
    Straight,
    /// Descend at `angle_deg` from horizontal **along the toolpath itself** — a
    /// one-way ramp, arriving on the contour at full depth.
    ///
    /// The tool enters the loop *before* its start point and descends as it travels,
    /// reaching depth exactly where the pass begins. That direction is deliberate: the
    /// stretch the ramp leaves sloped is then the loop's own final stretch, so the pass
    /// re-machines it at full depth without any extra motion. A ramp running forward
    /// from the start would strand that wedge.
    ///
    /// Use [`ZigZag`](Self::ZigZag) instead where there is no room to travel — that is
    /// what an oscillating entry is for.
    Ramp { angle_deg: f64 },
    /// Spiral down on a helix of `radius` mm, `pitch` mm of descent per turn.
    Helix { radius: f64, pitch: f64 },
    /// Back-and-forth ramp of `length` mm at `angle_deg`, for narrow slots — an
    /// oscillation in place, for the case [`Ramp`](Self::Ramp) cannot serve because the
    /// tool has nowhere to travel.
    ZigZag { length: f64, angle_deg: f64 },
}

/// Holding tabs placed by distance, exact count or selected contour locations.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ProfileTabs {
    pub shape: crate::TabShape,
    /// None uses spacing; Some distributes this exact number per contour.
    pub count: Option<u32>,
    pub enabled: bool,
    /// Retained material width along the contour (mm).
    pub width: f64,
    /// Height above the profile floor (mm).
    pub height: f64,
    /// Maximum centre-to-centre distance along the contour (mm).
    pub spacing: f64,
    /// Use selected locations instead of automatic spacing.
    pub manual: bool,
    /// Selected points in the contour's setup XY coordinates.
    pub positions: Vec<[f64; 2]>,
}

impl Default for ProfileTabs {
    fn default() -> Self {
        Self {
            shape: crate::TabShape::Rectangular,
            count: None,
            enabled: false,
            width: 9.0,
            height: 3.0,
            spacing: 180.0,
            manual: false,
            positions: Vec::new(),
        }
    }
}

/// One open or closed contour and its compensation side.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ProfileContour {
    pub chain: Contour,
    pub closed: bool,
    /// Closed loops use Inside/Outside; open chains use right/left respectively.
    pub side: Side,
}

fn profile_closed_default() -> bool {
    true
}

/// A profile operation may contain several independently retracted contours.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ProfileOp {
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub parameter_expressions: std::collections::BTreeMap<crate::OperationParameter, String>,
    /// Optional simultaneous fourth-axis cylindrical wrap.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub wrap: Option<crate::RotaryWrap>,
    /// Additional pass controls. Defaults preserve existing saved cuts.
    #[serde(default)]
    pub passes: crate::ProfilePasses,
    /// None preserves legacy selections; Some(empty) is an explicitly empty collection.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub selections: Option<Vec<crate::ProfileGeometryEntry>>,
    #[serde(default = "profile_closed_default")]
    pub closed: bool,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub additional_chains: Vec<ProfileContour>,
    /// Optional holding tabs (disabled in older projects).
    #[serde(default)]
    pub tabs: ProfileTabs,
    /// Operation id (0-based within the setup); stamped onto every emitted tag.
    pub id: u32,
    /// Tool number to select from the setup's tool list.
    pub tool: u32,
    /// The primary chain to profile, in the part/WCS frame.
    pub chain: Contour,
    /// Which side of the chain the tool runs on.
    pub side: Side,
    /// How radius compensation is applied.
    pub comp: Comp,
    /// Finishing allowance left on the wall (mm): the profile stops this far short
    /// of the chain edge, on the same side as the tool, so a later finishing
    /// operation can take the whole vertical face in one pass. `0.0` (the default)
    /// profiles to size. Applied on top of the tool-radius offset.
    #[serde(default)]
    pub offset: f64,
    /// Cut depth below the reference plane (Z=0), as a positive magnitude (mm).
    /// The final (deepest) pass sits at absolute Z `-depth`.
    pub depth: f64,
    /// Maximum depth removed per pass (mm, > 0).
    pub stepdown: f64,
    /// Radial stepover for XY roughing passes (mm). `0` (the default) cuts a single
    /// pass at the profile; `> 0` clears the material out to the raw stock in
    /// concentric passes stepping in by this much (like `stepdown`, but in XY),
    /// leaving the finishing `offset` on the wall. Needs the stock bounds; without
    /// them it falls back to a single pass.
    #[serde(default)]
    pub stepover: f64,
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the tool's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Selected cutting preset baseline; operation values may be manually overridden.
    #[serde(default)]
    pub tool_preset: Option<crate::ToolPreset>,
    /// Concrete cutting cooling; changing library presets never rewrites saved operations.
    #[serde(default)]
    pub cooling: crate::Cooling,
    /// Independent ramp and lead-in/out feed overrides.
    #[serde(default)]
    pub entry_feeds: crate::EntryFeeds,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed, mm/min.
    pub plunge_feed: f64,
    /// The chosen start point (part XY), if any: the loop is rotated to begin at
    /// the offset vertex nearest this point. `None` starts at the chain's first
    /// vertex (the default).
    pub start: Option<[f64; 2]>,
    /// Preferred exit location in setup XY, reached along the completed toolpath.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end: Option<[f64; 2]>,
    /// Lead-in onto the contour at the start point.
    pub lead_in: Lead,
    /// Lead-out off the contour after the cut.
    pub lead_out: Lead,
    /// Distance (mm, >= 0) to keep cutting past the start point before leading off,
    /// so the lead-in/lead-out junction is re-machined and leaves no witness dent.
    /// `0.0` (the default) leads off exactly at the start, as before.
    #[serde(default)]
    pub lead_overlap: f64,
    /// How the tool enters the material in Z at each pass.
    pub plunge: Plunge,
    /// Milling direction for all profile passes, and constant-engagement settings
    /// for outside roughing (`stepover > 0`). `engagement <= 0` (the default)
    /// keeps plain concentric roughing.
    #[serde(default)]
    pub clearing: Clearing,
}

impl ProfileOp {
    /// Contours in the same stable order used by selection markers and machining.
    pub fn contours(&self) -> impl Iterator<Item = ProfileContour> + '_ {
        std::iter::once(ProfileContour {
            chain: self.chain.clone(),
            closed: self.closed,
            side: self.side,
        })
        .chain(self.additional_chains.iter().cloned())
    }

    pub fn contour_reversed(&self, index: usize) -> bool {
        let mut offset = 0;
        for entry in self.selections.iter().flatten() {
            if index < offset + entry.contour_count {
                return entry.kind == crate::ProfileGeometryKind::Chain
                    && entry.reversed
                    && if index == 0 {
                        self.closed
                    } else {
                        self.additional_chains
                            .get(index - 1)
                            .is_some_and(|c| c.closed)
                    };
            }
            offset += entry.contour_count;
        }
        false
    }

    pub fn set_contour_side(&mut self, index: usize, side: Side) -> bool {
        if index == 0 {
            self.side = side;
            true
        } else if let Some(contour) = self.additional_chains.get_mut(index - 1) {
            contour.side = side;
            true
        } else {
            false
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct DrillGeometry {
    pub select_same_diameter: bool,
    pub auto_merge_hole_segments: bool,
    pub order_by_depth: bool,
    pub order: crate::BoreOrder,
    pub reverse_order: bool,
}

impl DrillGeometry {
    pub fn uses_model_heights(self) -> bool {
        self.select_same_diameter || self.auto_merge_hole_segments || self.order_by_depth
    }

    fn is_default(&self) -> bool {
        *self == Self::default()
    }
}

/// A drilling operation: a set of holes taken to a depth, optionally pecked and
/// dwelled. It is emitted as a Tier-2 cycle intent, so each post lowers it per
/// its capabilities (canned `G83` on Fanuc, explicit pecks on grbl).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DrillOp {
    #[serde(default, skip_serializing_if = "crate::DrillCycleSettings::is_default")]
    pub cycle: crate::DrillCycleSettings,
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub parameter_expressions: std::collections::BTreeMap<crate::OperationParameter, String>,
    #[serde(default, skip_serializing_if = "DrillGeometry::is_default")]
    pub geometry: DrillGeometry,
    /// Setup-space cutting intervals aligned with points for resolved CAD holes.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub hole_heights: Vec<crate::CuttingHeights>,
    /// Operation id.
    pub id: u32,
    /// Tool number.
    pub tool: u32,
    /// Hole positions in the part/WCS frame.
    pub points: Vec<[f64; 2]>,
    /// Hole depth (mm, a positive magnitude) measured **down from where the hole
    /// starts** — i.e. from `top_of_stock + start_offset`. The bottom sits at
    /// absolute Z `top_of_stock + start_offset - depth`.
    pub depth: f64,
    /// Align the full-diameter cutting lip with the programmed bottom.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub tip_through_bottom: bool,
    /// Where the hole begins, as a height **above the stock top** (mm), matching the
    /// Face convention. `0` (the default) starts at the stock top; positive starts it
    /// above the surface (a proud boss); negative below (a recessed/faced surface).
    /// Depth is measured down from here.
    #[serde(default)]
    pub start_offset: f64,
    /// Peck increment (mm, > 0); `None` drills straight.
    pub peck: Option<f64>,
    /// Dwell at the bottom, seconds; `None` for no dwell.
    pub dwell: Option<f64>,
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the tool's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Selected cutting preset baseline; operation values may be manually overridden.
    #[serde(default)]
    pub tool_preset: Option<crate::ToolPreset>,
    /// Concrete cutting cooling; changing library presets never rewrites saved operations.
    #[serde(default)]
    pub cooling: crate::Cooling,
    /// Independent ramp and lead-in/out feed overrides.
    #[serde(default)]
    pub entry_feeds: crate::EntryFeeds,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
    /// Plunge feed, mm/min.
    pub feed: f64,
}

/// Default for a `#[serde(default)]` boolean field that should default to `true`.
pub(crate) fn default_true() -> bool {
    true
}

/// Default for a per-operation `work_offset`: datum 1, the base work coordinate
/// system. 1-based so a missing field (v5 and earlier files) opens on datum 1.
pub(crate) fn default_work_offset() -> u32 {
    1
}

/// Constant-engagement (trochoidal) clearing parameters, shared by pocket clearing
/// and profile outside-roughing. When `engagement > 0` the region is cleared with a
/// path that keeps the tool's radial engagement bounded (higher feeds, kinder to the
/// tool); `engagement <= 0` (the default) falls back to plain concentric-ring
/// clearing, so existing documents are unchanged.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Clearing {
    /// Maximum radial width of cut (mm) — the engagement cap. Bounds how much fresh
    /// material the tool takes at once. `<= 0` disables adaptive clearing.
    #[serde(default)]
    pub engagement: f64,
    /// Climb milling (`true`, the default) vs conventional (`false`) for the
    /// clearing path and profile finishing passes.
    #[serde(default = "default_true")]
    pub climb: bool,
}

impl Default for Clearing {
    fn default() -> Self {
        Self {
            engagement: 0.0,
            climb: true,
        }
    }
}

/// The parameters of an **area-clearing pass** — everything a pocket takes except its
/// geometry and depth.
///
/// Factored out because a carve's clearing pass is not a reduced pocket, it is a pocket
/// over a derived region: the same engine, the same controls, the same expectations. A
/// set of `clear_*` scalars copied beside it would drift from the real thing the first
/// time either gained a control the other did not.
///
/// Both [`PocketOp`] and [`CarveOp`] carry one of these (v10; see
/// [`SCHEMA_VERSION`]) — a control added here reaches both by construction, which is
/// the whole point of the type.
///
/// It is **not** what the clearing engine consumes. `cam-toolpath`'s `ClearJob` adds the
/// derived geometry a run needs (tool radius, first-ring offset, nominal spacing, the
/// lead guard region) and is built per call site from these plus the tool. Collapsing
/// the two would push tool geometry into the document model; keeping them separate means
/// this struct stays *what the operator asked for* and `ClearJob` stays *what that works
/// out to*.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ClearParams {
    /// Keep safe links between depth levels and finishing passes at cutting height.
    #[serde(default)]
    pub keep_tool_down: bool,
    /// Vertical lift above the cutting level during local linking moves, mm.
    /// Zero keeps normal transitions at depth; adaptive wall withdrawals lift
    /// at least 0.2 mm. Distant links still use clearance.
    #[serde(default)]
    pub lift_height: f64,
    /// Maximum depth removed per pass, mm. `0` clears the full depth in one pass.
    #[serde(default)]
    pub stepdown: f64,
    /// Explicit pocket adaptive mode. Missing values preserve legacy behavior based on engagement.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub adaptive_enabled: Option<bool>,
    /// Legacy absolute overlap between concentric pocket passes, mm.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pocket_overlap: Option<f64>,
    /// Pocket depth scheduling settings. Older operations retain their existing multi-pass behavior.
    #[serde(default)]
    pub multiple_depths: crate::pocket::MultipleDepths,
    /// Fraction of the tool diameter that adjacent rings overlap (0..1). The radial
    /// ring spacing is `diameter * (1 - overlap)`.
    #[serde(default)]
    pub overlap: f64,
    /// Finishing allowance left on the walls, mm.
    ///
    /// In a pocket this skin is taken off later by a finishing profile. In a **carve**
    /// it is how far the end mill stays off the carved surface, leaving that skin for
    /// the V-bit — which finishes it better, with the flank of its cone rather than the
    /// corner of a cylinder. Nothing is left behind: the V-bit's floor pass is computed
    /// against what the clearing tool *actually swept*, so a larger allowance simply
    /// hands more of the work back to it.
    #[serde(default)]
    pub offset: f64,
    /// Cutting feed, mm/min.
    #[serde(default)]
    pub feed: f64,
    /// Plunge feed, mm/min.
    #[serde(default)]
    pub plunge_feed: f64,
    /// How the tool enters the material in Z at each pass.
    #[serde(default)]
    pub plunge: Plunge,
    /// Lead-in from cleared stock onto a wall or an adaptive cutting section.
    #[serde(default)]
    pub lead_in: Lead,
    /// Lead-out from a wall or an adaptive cutting section into cleared stock.
    #[serde(default)]
    pub lead_out: Lead,
    /// Distance each ring keeps cutting past its start before retracting, mm, so the
    /// loop-closure junction leaves no witness dent.
    /// Adjoining adaptive cuts overlap by this distance along the next cut;
    /// zero uses exactly the same departure and re-entry point at their seam.
    #[serde(default)]
    pub lead_overlap: f64,
    /// Constant-engagement clearing parameters (engagement cap + climb).
    #[serde(default)]
    pub clearing: Clearing,
}

impl ClearParams {
    pub fn pocket_adaptive_enabled(&self) -> bool {
        self.adaptive_enabled
            .unwrap_or(self.clearing.engagement > 0.0)
    }
}

impl Default for ClearParams {
    fn default() -> Self {
        Self {
            keep_tool_down: false,
            lift_height: 0.0,
            stepdown: 0.0,
            adaptive_enabled: None,
            pocket_overlap: None,
            multiple_depths: Default::default(),
            // Half the diameter: the same spacing a pocket's own default overlap gives.
            overlap: 0.5,
            offset: 0.0,
            feed: 0.0,
            plunge_feed: 0.0,
            plunge: Plunge::Straight,
            lead_in: Lead::Arc { radius: 0.6 },
            lead_out: Lead::Arc { radius: 0.6 },
            lead_overlap: 0.0,
            clearing: Clearing::default(),
        }
    }
}

/// A carve's optional **clearing pass**: which tool clears the flat land, and how.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CarveClearing {
    /// The flat-bottomed tool that clears the flat land at full depth, before the
    /// V-bit runs.
    pub tool: u32,
    /// How it clears. Feeds of `0` inherit the carve's own.
    #[serde(default)]
    pub params: ClearParams,
}

/// A 2.5-D pocket-clearing operation: remove all material inside a closed
/// boundary (leaving any islands standing), in concentric offset rings, in
/// stepdown passes, down to a depth.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PocketOp {
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub parameter_expressions: std::collections::BTreeMap<crate::OperationParameter, String>,
    /// Optional simultaneous fourth-axis cylindrical wrap (including 2D adaptive clearing).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub wrap: Option<crate::RotaryWrap>,
    /// Operation id.
    pub id: u32,
    /// Tool number.
    pub tool: u32,
    /// The closed pocket boundary.
    pub boundary: Contour,
    /// Closed islands to leave uncut (holes within the pocket).
    pub islands: Vec<Contour>,
    /// Additional pockets, each retaining its own boundary and islands.
    /// All regions use the operation cutting heights and clearing settings.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub additional_regions: Vec<cam_geo::Polygon>,
    /// Optional material footprint for open pockets and 2D adaptive clearing.
    #[serde(default)]
    pub stock_contours: crate::StockContours,
    /// Pocket depth below the reference plane (Z=0), as a positive magnitude (mm).
    /// The floor sits at absolute Z `-depth`.
    pub depth: f64,
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the tool's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Selected cutting preset baseline; operation values may be manually overridden.
    #[serde(default)]
    pub tool_preset: Option<crate::ToolPreset>,
    /// Concrete cutting cooling; changing library presets never rewrites saved operations.
    #[serde(default)]
    pub cooling: crate::Cooling,
    /// Independent ramp and lead-in/out feed overrides.
    #[serde(default)]
    pub entry_feeds: crate::EntryFeeds,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
    /// Preferred lead-in location (part XY): the clearing begins on the ring point
    /// nearest here, so the plunge/entry witness mark lands where the machinist
    /// chose. `None` uses the strategy's default entry.
    ///
    /// Stays on the operation rather than joining [`ClearParams`]: it names a place on
    /// *this* pocket's geometry, so it has no meaning for the derived region a carve
    /// clears.
    pub start: Option<[f64; 2]>,
    /// Preferred exit location in setup XY, reached along the completed toolpath.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end: Option<[f64; 2]>,
    /// Maximum XY spacing between concentric roughing passes, mm.
    /// Older operations derive this from their saved overlap and the cutter diameter.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub maximum_stepover: Option<f64>,
    /// How this pocket is cleared — stepdown, wall allowance, feeds,
    /// plunge style, leads, and the engagement cap.
    ///
    /// The same [`ClearParams`] a [`CarveOp`]'s clearing pass carries (v10; before that
    /// these were ten flat fields here, and a control added to one op could silently
    /// miss the other).
    ///
    /// Deliberately **not** `#[serde(default)]`, unlike almost every field added since
    /// v5. A default here would make [`crate::migrate`] optional in the worst possible
    /// way: a v9 pocket, whose clearing parameters are all still flat, would load
    /// without complaint as a pocket with **zero feed and zero stepdown** — a silently
    /// broken operation rather than a refusal. Requiring the field means a document that
    /// reached the deserializer un-migrated fails there, loudly, with the field named.
    pub clear: ClearParams,
    /// Optional radial wall finishing, with compensation computed in the toolpath.
    #[serde(default)]
    pub finishing: crate::PocketFinishing,
}

impl PocketOp {
    pub fn concentric_stepover(&self, diameter: f64) -> f64 {
        self.maximum_stepover.unwrap_or_else(|| {
            diameter
                - self
                    .clear
                    .pocket_overlap
                    .unwrap_or(diameter * self.clear.overlap)
        })
    }

    pub fn validate_maximum_stepover(stepover: f64, diameter: f64) -> Result<(), String> {
        if stepover > 1e-9 && stepover <= diameter && diameter.is_finite() {
            Ok(())
        } else {
            Err("Maximum stepover must be positive and at most the tool diameter".into())
        }
    }

    /// Keeps older overlap formulas and their dependencies live in the inspector.
    pub fn legacy_stepover_expression(&self) -> Option<&'static str> {
        use crate::OperationParameter::{MaximumStepover, Overlap, PocketOverlap};
        if self.maximum_stepover.is_some() || self.parameter_expressions.contains_key(&MaximumStepover)
        {
            None
        } else if self.parameter_expressions.contains_key(&PocketOverlap) {
            Some("tool_diameter - pocketOverlap")
        } else if self.clear.pocket_overlap.is_none()
            && self.parameter_expressions.contains_key(&Overlap)
        {
            Some("tool_diameter - tool_diameter * (overlap / 100)")
        } else {
            None
        }
    }
}

/// 3D surface finishing over a CAD solid or selected faces.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ParallelOp {
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub parameter_expressions: std::collections::BTreeMap<crate::OperationParameter, String>,
    pub id: u32,
    pub tool: u32,
    #[serde(default)]
    pub spindle_rpm: f64,
    #[serde(default)]
    pub tool_preset: Option<crate::ToolPreset>,
    #[serde(default)]
    pub cooling: crate::Cooling,
    #[serde(default)]
    pub entry_feeds: crate::EntryFeeds,
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
    #[serde(default)]
    pub params: ParallelParams,
}

/// 3D spiral finishing over a CAD solid or selected faces.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SpiralOp {
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub parameter_expressions: std::collections::BTreeMap<crate::OperationParameter, String>,
    pub id: u32,
    pub tool: u32,
    #[serde(default)]
    pub spindle_rpm: f64,
    #[serde(default)]
    pub tool_preset: Option<crate::ToolPreset>,
    #[serde(default)]
    pub cooling: crate::Cooling,
    #[serde(default)]
    pub entry_feeds: crate::EntryFeeds,
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
    #[serde(default)]
    pub params: crate::SpiralParams,
}

/// 3D adaptive roughing resolved from an independent CAD machining projection.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AdaptiveOp {
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub parameter_expressions: std::collections::BTreeMap<crate::OperationParameter, String>,
    pub id: u32,
    pub tool: u32,
    #[serde(default)]
    pub spindle_rpm: f64,
    #[serde(default)]
    pub tool_preset: Option<crate::ToolPreset>,
    #[serde(default)]
    pub cooling: crate::Cooling,
    #[serde(default)]
    pub entry_feeds: crate::EntryFeeds,
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
    #[serde(default)]
    pub params: AdaptiveParams,
}

/// Controls for 3D adaptive roughing. The engagement controls are deliberately
/// explicit; invalid or unsupported combinations stop generation with an error.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AdaptiveParams {
    /// Vertical lift above the cutting level during stay-down transitions, mm.
    #[serde(default = "default_adaptive_lift_height")]
    pub lift_height: f64,
    /// Radius of the tangent arc into a cut, mm. Zero disables the arc.
    #[serde(default = "default_adaptive_lead_radius")]
    pub lead_in_radius: f64,
    /// Radius of the tangent arc out of a cut, mm. Zero disables the arc.
    #[serde(default = "default_adaptive_lead_radius")]
    pub lead_out_radius: f64,
    /// Preferred entry and exit locations in setup XY.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub start: Option<[f64; 2]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end: Option<[f64; 2]>,
    /// Maximum geometric deviation accepted when deriving layer boundaries, mm.
    #[serde(default = "default_adaptive_tolerance")]
    pub tolerance: f64,
    /// Target radial engagement, mm.
    #[serde(default = "default_adaptive_load")]
    pub optimal_load: f64,
    /// Maximum roughing stepdown, mm.
    #[serde(default = "default_adaptive_stepdown")]
    pub stepdown: f64,
    /// Maximum axial spacing of fine-rest passes, mm.
    #[serde(default = "default_adaptive_fine_stepdown")]
    pub fine_stepdown: f64,
    /// Cutting feed, mm/min.
    #[serde(default)]
    pub feed: f64,
    /// Plunge feed, mm/min.
    #[serde(default)]
    pub plunge_feed: f64,
    /// Climb milling when true.
    #[serde(default = "default_true")]
    pub climb: bool,
    /// Relation between the cutter and the selected boundary.
    #[serde(default)]
    pub containment: AdaptiveContainment,
    /// Signed shift applied to the selected XY boundary, in mm.
    #[serde(default)]
    pub additional_offset: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AdaptiveContainment {
    #[default]
    Inside,
    Center,
    Outside,
}

impl std::fmt::Display for AdaptiveContainment {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Inside => "Tool inside boundary",
            Self::Center => "Tool center on boundary",
            Self::Outside => "Tool outside boundary",
        })
    }
}

fn default_adaptive_lift_height() -> f64 {
    0.2
}
fn default_adaptive_lead_radius() -> f64 {
    0.6
}
fn default_adaptive_tolerance() -> f64 {
    0.1
}
fn default_adaptive_load() -> f64 {
    2.0
}
fn default_adaptive_stepdown() -> f64 {
    2.0
}
fn default_adaptive_fine_stepdown() -> f64 {
    0.5
}

impl AdaptiveParams {
    /// Spend one quarter of the operation tolerance on frontier simplification and
    /// corner detection. This gives 0.025 mm at the default 0.1 mm tolerance.
    /// Motion remains guarded by the original boundary and path certification.
    pub fn simplification_tolerance(&self) -> f64 {
        self.tolerance / 4.0
    }
}

impl Default for AdaptiveParams {
    fn default() -> Self {
        Self {
            lift_height: default_adaptive_lift_height(),
            lead_in_radius: default_adaptive_lead_radius(),
            lead_out_radius: default_adaptive_lead_radius(),
            start: None,
            end: None,
            tolerance: default_adaptive_tolerance(),
            optimal_load: default_adaptive_load(),
            stepdown: default_adaptive_stepdown(),
            fine_stepdown: default_adaptive_fine_stepdown(),
            feed: 0.0,
            plunge_feed: 0.0,
            climb: true,
            containment: AdaptiveContainment::Inside,
            additional_offset: 0.0,
        }
    }
}

#[cfg(test)]
mod adaptive_params_tests {
    use super::{AdaptiveContainment, AdaptiveParams, ClearParams};

    #[test]
    fn lift_height_and_adaptive_lead_radii_default_and_round_trip() {
        let mut clear: ClearParams = serde_json::from_str("{}").unwrap();
        let mut adaptive: AdaptiveParams = serde_json::from_str("{}").unwrap();
        assert_eq!(clear.lift_height, 0.0);
        assert_eq!(adaptive.lift_height, 0.2);
        assert_eq!(adaptive.lead_in_radius, 0.6);
        assert_eq!(adaptive.lead_out_radius, 0.6);
        clear.lift_height = 2.0;
        adaptive.lift_height = 2.0;
        adaptive.lead_in_radius = 0.35;
        adaptive.lead_out_radius = 0.0;
        let restored: ClearParams =
            serde_json::from_value(serde_json::to_value(clear).unwrap()).unwrap();
        assert_eq!(restored, clear);
        let restored: AdaptiveParams =
            serde_json::from_value(serde_json::to_value(adaptive).unwrap()).unwrap();
        assert_eq!(restored, adaptive);
    }

    #[test]
    fn simplification_follows_tolerance_and_ignores_legacy_override() {
        for tolerance in [0.025, 0.1, 0.5] {
            let params: AdaptiveParams = serde_json::from_value(serde_json::json!({
                "tolerance": tolerance,
                "simplification_tolerance": 0.9
            }))
            .unwrap();
            assert_eq!(params.simplification_tolerance(), tolerance / 4.0);
            let saved = serde_json::to_value(params).unwrap();
            assert!(saved.get("simplification_tolerance").is_none());
            let restored: AdaptiveParams = serde_json::from_value(saved).unwrap();
            assert_eq!(restored.simplification_tolerance(), tolerance / 4.0);
        }
    }

    #[test]
    fn containment_and_offset_round_trip_and_legacy_projects_keep_inside_defaults() {
        let legacy: AdaptiveParams = serde_json::from_value(serde_json::json!({})).unwrap();
        assert_eq!(legacy.containment, AdaptiveContainment::Inside);
        assert_eq!(legacy.additional_offset, 0.0);
        assert_eq!(legacy.simplification_tolerance(), 0.025);

        let configured = AdaptiveParams {
            containment: AdaptiveContainment::Outside,
            additional_offset: -0.75,
            tolerance: 0.025,
            ..legacy
        };
        let restored: AdaptiveParams =
            serde_json::from_value(serde_json::to_value(configured).unwrap()).unwrap();
        assert_eq!(restored, configured);
    }
}

/// A principal axis in the XY plane — used to orient facing passes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Axis {
    /// Parallel cutting lines run along X (stepping over in Y).
    #[default]
    X,
    /// Parallel cutting lines run along Y (stepping over in X).
    Y,
}

impl Axis {
    /// Both axes, in a stable order — for pickers.
    pub const ALL: [Axis; 2] = [Axis::X, Axis::Y];

    /// Infer a facing pass direction from **the boundary edge the user clicked**:
    /// the passes run parallel to the edge nearest `pick`, snapped to the nearer
    /// principal axis. This is the intent when a boundary is picked by clicking one
    /// of its edges — face along the side you pointed at. Degenerate input (fewer
    /// than two vertices) falls back to `X`.
    pub fn along_edge_at(pts: &[cam_geo::Point], pick: cam_geo::Point) -> Axis {
        let n = pts.len();
        if n < 2 {
            return Axis::X;
        }
        let mut best_d2 = f64::MAX;
        let mut axis = Axis::X;
        for i in 0..n {
            let (a, b) = (pts[i], pts[(i + 1) % n]);
            let d2 = point_seg_dist_sq(pick, a, b);
            if d2 < best_d2 {
                best_d2 = d2;
                let (dx, dy) = ((b.x - a.x).abs(), (b.y - a.y).abs());
                axis = if dx >= dy { Axis::X } else { Axis::Y };
            }
        }
        axis
    }

    /// Infer a facing pass direction from a boundary's **longest edge**, snapped to
    /// the nearer principal axis — the fallback when no pick point is available.
    /// Facing along the long side gives the fewest, longest passes. Degenerate input
    /// falls back to `X`.
    pub fn along_longest_edge(pts: &[cam_geo::Point]) -> Axis {
        let n = pts.len();
        if n < 2 {
            return Axis::X;
        }
        let mut best = f64::MIN;
        let mut axis = Axis::X;
        for i in 0..n {
            let a = pts[i];
            let b = pts[(i + 1) % n];
            let (dx, dy) = ((b.x - a.x).abs(), (b.y - a.y).abs());
            let len2 = dx * dx + dy * dy;
            if len2 > best {
                best = len2;
                axis = if dx >= dy { Axis::X } else { Axis::Y };
            }
        }
        axis
    }
}

/// Squared distance from `p` to the segment `a→b` (clamped to the endpoints).
fn point_seg_dist_sq(p: cam_geo::Point, a: cam_geo::Point, b: cam_geo::Point) -> f64 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len2 = dx * dx + dy * dy;
    if len2 <= f64::EPSILON {
        return p.distance_sq(a);
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len2).clamp(0.0, 1.0);
    p.distance_sq(cam_geo::Point::new(a.x + dx * t, a.y + dy * t))
}

impl std::fmt::Display for Axis {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Axis::X => "X",
            Axis::Y => "Y",
        })
    }
}

/// A facing operation: clear the top of the stock over a boundary with parallel
/// passes, in stepdown passes, down to a depth. The passes form a continuous
/// serpentine (one plunge per level, arc turnarounds) rather than lifting between
/// passes.
///
/// Z uses the magnitude convention: `start_offset` is the top cutting plane above
/// the drawing reference (Z=0), `depth` is how far *down* to face as a positive
/// number, so the final faced plane is `start_offset - depth`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FaceOp {
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub parameter_expressions: std::collections::BTreeMap<crate::OperationParameter, String>,
    /// Operation id.
    pub id: u32,
    /// Tool number.
    pub tool: u32,
    /// The area to face (usually the stock outline).
    pub boundary: Contour,
    /// Z of the top cutting plane, above the drawing reference (mm, >= 0). Set
    /// this to face stock that stands proud of Z=0 down toward it; `0` starts the
    /// cut at the reference plane.
    #[serde(default)]
    pub start_offset: f64,
    /// Depth removed *downward* from `start_offset`, as a positive magnitude (mm).
    /// The final faced plane sits at `start_offset - depth`.
    pub depth: f64,
    /// Maximum depth removed per pass (mm, > 0).
    pub stepdown: f64,
    /// Fraction of the tool diameter that adjacent passes overlap (0..1). The pass
    /// spacing is `diameter * (1 - overlap)`, and the first pass is placed so it
    /// cuts a strip of exactly that width along the stock edge.
    #[serde(default)]
    pub overlap: f64,
    /// Facing overlap in millimetres; absent in older percentage-based projects.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub absolute_overlap: Option<f64>,
    /// Axial pass scheduling. Older projects retain their original multi-pass behavior.
    #[serde(default)]
    pub multiple_depths: crate::MultipleDepths,
    /// Distance the tool overshoots past each stock edge (mm, >= 0) before the
    /// 180-degree turnaround arc, so the arc swings clear of the part.
    #[serde(default)]
    pub overshoot: f64,
    /// Orientation of the parallel cutting lines.
    #[serde(default)]
    pub direction: Axis,
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the tool's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Selected cutting preset baseline; operation values may be manually overridden.
    #[serde(default)]
    pub tool_preset: Option<crate::ToolPreset>,
    /// Concrete cutting cooling; changing library presets never rewrites saved operations.
    #[serde(default)]
    pub cooling: crate::Cooling,
    /// Independent ramp and lead-in/out feed overrides.
    #[serde(default)]
    pub entry_feeds: crate::EntryFeeds,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed, mm/min.
    pub plunge_feed: f64,
}

/// A thread-milling operation: cut internal or external threads at a set of hole
/// centres by helically interpolating a thread mill. The thread *form* (a full-
/// profile mill cutting the whole length in one turn vs. a single-form mill
/// stacking one turn per pitch) is chosen by the strategy; this describes the
/// geometry to cut.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ThreadOp {
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub parameter_expressions: std::collections::BTreeMap<crate::OperationParameter, String>,
    /// Operation id.
    pub id: u32,
    /// Tool number.
    pub tool: u32,
    /// Thread-hole (or boss) centres in the part/WCS frame.
    pub points: Vec<[f64; 2]>,
    /// Internal thread (bore) when `true`; external thread (boss) when `false`.
    pub internal: bool,
    /// Thread hand — which way the helix winds.
    pub hand: Hand,
    /// Nominal major diameter of the thread, mm.
    pub major_dia: f64,
    /// Thread pitch (Z advance per turn), mm (> 0).
    pub pitch: f64,
    /// Absolute Z of the top of the threaded length.
    pub z_top: f64,
    /// Absolute Z of the bottom of the threaded length.
    pub z_bottom: f64,
    /// Climb-mill when `true`, conventional when `false` — together with `hand`
    /// and `internal` this fixes the helix direction and travel sense.
    pub climb: bool,
    /// Number of **radial infeed passes**: the thread is cut to full depth in this
    /// many radial steps (each a full helix, stepping the orbit outward for an
    /// internal thread / inward for an external one). `1` (the default) cuts the whole
    /// depth in a single pass; more passes lighten the cut for hard material. How the
    /// steps are *sized* is [`gradual`](Self::gradual).
    #[serde(default)]
    pub passes: u32,
    /// When `true`, size the infeed passes for **equal material per pass** instead of
    /// equal radial steps.
    ///
    /// A thread form is a V, so the groove widens axially as it deepens and the section
    /// removed grows as the square of the radial penetration. Equal radial steps
    /// therefore load the *last* pass hardest — the opposite of what multiple passes are
    /// for, since that is the pass cutting the finished flank. Gradual makes each pass
    /// remove the same area (penetration `∝ √(i/n)`), so the finishing pass is the
    /// lightest. The same reasoning, and the same `√k`, as [`ChamferOp::gradual`].
    #[serde(default)]
    pub gradual: bool,
    /// Extra **spring passes** at the final (full) depth, to clean up elastic spring-back
    /// after the last cutting pass. `0` (the default) adds none.
    #[serde(default)]
    pub spring_passes: u32,
    /// For an internal **blind** hole: how far (mm) the pre-drilled hole extends *below*
    /// the thread bottom (`z_bottom`). `0` (the default) means a **through hole** — no
    /// blind-bottom check. When positive it must be at least [`blind_allowance`](Self::
    /// blind_allowance), else the operation errors (the tool cannot thread flush to a
    /// blind bottom).
    #[serde(default)]
    pub drill_clearance: f64,
    /// Required standoff (mm) between the last thread and the bottom of a blind hole; the
    /// [`drill_clearance`](Self::drill_clearance) is validated against it. `0` (the
    /// default) means *auto* — one thread pitch.
    #[serde(default)]
    pub blind_allowance: f64,
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the tool's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Selected cutting preset baseline; operation values may be manually overridden.
    #[serde(default)]
    pub tool_preset: Option<crate::ToolPreset>,
    /// Concrete cutting cooling; changing library presets never rewrites saved operations.
    #[serde(default)]
    pub cooling: crate::Cooling,
    /// Independent ramp and lead-in/out feed overrides.
    #[serde(default)]
    pub entry_feeds: crate::EntryFeeds,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed for the approach in Z, mm/min.
    pub plunge_feed: f64,
}

/// A planar modeled chamfer strip in setup coordinates. Endpoints at both
/// elevations are paired along the face's tangent, including mitered corners.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ChamferFace {
    pub upper: [[f64; 2]; 2],
    pub lower: [[f64; 2]; 2],
    pub top: f64,
    pub bottom: f64,
    /// Unit XY direction from material toward air.
    pub outward: [f64; 2],
}
impl ChamferFace {
    pub fn width(&self) -> f64 {
        (0..2)
            .map(|i| (self.lower[0][i] - self.upper[0][i]) * self.outward[i])
            .sum()
    }
    pub fn at_height(&self, z: f64) -> [[f64; 2]; 2] {
        let t = (self.top - z) / (self.top - self.bottom);
        std::array::from_fn(|end| {
            std::array::from_fn(|i| {
                self.upper[end][i] + t * (self.lower[end][i] - self.upper[end][i])
            })
        })
    }
    pub fn validate(&self) -> Result<(), String> {
        if !self
            .upper
            .iter()
            .chain(&self.lower)
            .flatten()
            .chain(self.outward.iter())
            .copied()
            .chain([self.top, self.bottom])
            .all(f64::is_finite)
            || self.top <= self.bottom
            || self.width() <= 1e-8
            || (self.outward[0].hypot(self.outward[1]) - 1.).abs() > 1e-6
        {
            return Err("Modeled chamfer requires finite upper/lower boundaries, a positive width and height, and a unit outward direction".into());
        }
        for line in [&self.upper, &self.lower] {
            let tangent = [line[1][0] - line[0][0], line[1][1] - line[0][1]];
            if tangent[0].hypot(tangent[1]) < 1e-8
                || (tangent[0] * self.outward[0] + tangent[1] * self.outward[1]).abs() > 1e-5
            {
                return Err(
                    "Chamfer upper and lower boundaries must be parallel straight edges".into(),
                );
            }
        }
        let tangent = [
            self.upper[1][0] - self.upper[0][0],
            self.upper[1][1] - self.upper[0][1],
        ];
        let tangent_length = tangent[0].hypot(tangent[1]);
        let along =
            |point: [f64; 2]| (point[0] * tangent[0] + point[1] * tangent[1]) / tangent_length;
        if along(self.lower[0]) > along(self.upper[0]) + 1e-5
            || along(self.lower[1]) < along(self.upper[1]) - 1e-5
        {
            return Err("This chamfer narrows toward its lower boundary; its trimmed corners cannot be covered by planar bands without overcutting".into());
        }
        let other_width: f64 = (0..2)
            .map(|i| (self.lower[1][i] - self.upper[1][i]) * self.outward[i])
            .sum();
        if (other_width - self.width()).abs() > 1e-5 {
            return Err("Chamfer face width must be constant".into());
        }
        Ok(())
    }
}

/// A chamfer/bevel along an open or closed edge chain, or modeled bevel faces.
/// The angled cutter runs on the air side at the depth and offset needed to
/// form the requested width; optional width increments divide the cut into passes.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ChamferOp {
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub parameter_expressions: std::collections::BTreeMap<crate::OperationParameter, String>,
    /// Fixed-count passes approaching the edge from the air side.
    #[serde(default)]
    pub contour_roughing: crate::ContourRoughing,
    /// Close the path between its last and first vertices. Legacy files use closed rims.
    #[serde(default = "default_true")]
    pub closed: bool,
    /// Operation id.
    pub id: u32,
    /// Tool number (must select a chamfer/V mill — the strategy reads its angle).
    pub tool: u32,
    /// The edge or connected chain to chamfer, in the part/WCS frame.
    pub chain: Contour,
    /// Exact upper/lower boundaries of modeled planar chamfer faces. Empty uses
    /// the edge strategy and its operator-defined width.
    #[serde(default)]
    pub faces: Vec<ChamferFace>,
    /// Maximum vertical band per pass for modeled faces. Zero automatically uses
    /// the cutter's usable flank; a smaller value adds overlapping finishing bands.
    #[serde(default)]
    pub stepdown: f64,
    /// Which side of the chain holds material; the tool runs on the other (air)
    /// side, offset by its tip radius.
    pub side: Side,
    /// Horizontal chamfer width, mm (> 0). Together with the tool angle this fixes
    /// the finished bevel; `depth` then chooses which section of the flank cuts it.
    pub width: f64,
    /// Absolute Z of the top edge, where the chamfer begins (usually the stock top).
    pub top: f64,
    /// Tip depth below the top edge, mm — where the tool's bottom edge rides, which
    /// selects the section of the cutting flank used. `0` (or anything up to the
    /// natural `width/tan(α)`) uses the very tip at the bevel bottom (the classic
    /// case). Larger plunges the tip deeper into the air so a higher flank section
    /// cuts the same bevel — useful when the tip section is worn or there is no room
    /// to plunge further with the tip.
    #[serde(default)]
    pub depth: f64,
    /// Extra axial distance below the cutting depth, in mm.
    #[serde(default)]
    pub tip_offset: f64,
    /// Chamfer-width increment per pass, mm. `0` (or `>= width`) cuts the whole
    /// bevel in a single pass; otherwise the bevel is reached in steps so the tool
    /// is not overloaded (protects the tool and cleans the cut on wide bevels/hard
    /// stock).
    #[serde(default)]
    pub step: f64,
    /// When `true`, pass widths are sized for **equal material per pass** (widths
    /// `step·√k`) instead of equal width increments, since a fixed width step
    /// removes more material as the bevel widens. `step` sets the first pass.
    #[serde(default)]
    pub gradual: bool,
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the tool's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Selected cutting preset baseline; operation values may be manually overridden.
    #[serde(default)]
    pub tool_preset: Option<crate::ToolPreset>,
    /// Concrete cutting cooling; changing library presets never rewrites saved operations.
    #[serde(default)]
    pub cooling: crate::Cooling,
    /// Independent ramp and lead-in/out feed overrides.
    #[serde(default)]
    pub entry_feeds: crate::EntryFeeds,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed for the approach in Z, mm/min.
    pub plunge_feed: f64,
    /// Preferred start location (part XY): the chamfer loop begins on the point
    /// nearest here (where the lead/entry lands). `None` starts at the chain's
    /// first vertex.
    pub start: Option<[f64; 2]>,
    /// Preferred exit location in setup XY, reached along the completed toolpath.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end: Option<[f64; 2]>,
    /// Lead-in onto the edge at the start point (eases the tool on, off the air
    /// side). `None` (the default) plunges directly onto the edge, as before.
    #[serde(default)]
    pub lead_in: Lead,
    /// Lead-out off the edge after the cut. `None` (the default) retracts in place.
    #[serde(default)]
    pub lead_out: Lead,
    /// Distance (mm, >= 0) to keep cutting past the start point before leading off,
    /// so the loop-closure junction is re-machined and leaves no witness dent.
    /// `0.0` (the default) closes exactly at the start, as before.
    #[serde(default)]
    pub lead_overlap: f64,
    /// Reuse the entry geometry for the exit on edge chamfers.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lead_in_enabled: Option<bool>,
    #[serde(default)]
    pub same_lead_out: bool,
    /// None retains the behavior of the stored lead-out on older projects.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lead_out_enabled: Option<bool>,
}

/// A **V-carve engraving** operation: run a V-bit along a path with its tip *in* the
/// material, so the cone ploughs a V-section groove.
///
/// Distinct from [`ChamferOp`] in three ways that matter:
///
/// - **The tool must be a V-bit** ([`crate::ToolKind::VBit`]), never a chamfer mill —
///   a chamfer mill's bottom is a *non-cutting* flat, so engraving with one would rub
///   rather than cut. The strategy rejects it outright.
/// - **No side offset.** A chamfer runs beside an edge (offset to the air side); an
///   engraving groove is centred *on* the path, so the tool axis follows the chain
///   exactly, with no radius compensation.
/// - **The path may be open.** Lettering and decorative strokes are open polylines,
///   not closed regions — see `closed`.
///
/// The groove's width follows from `depth` and the tool's geometry
/// ([`cam_geo::vtip_half_width`]); it is not specified independently.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct EngraveOp {
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub parameter_expressions: std::collections::BTreeMap<crate::OperationParameter, String>,
    /// Operation id.
    pub id: u32,
    /// Tool number — must select a V-bit (the strategy reads its angle and tip radius).
    pub tool: u32,
    /// The path to engrave, in the part/WCS frame. The tool centre follows it exactly.
    pub chain: Contour,
    /// Whether `chain` is a closed loop (the last vertex joins back to the first) or an
    /// **open** stroke that stops at the last vertex. [`Contour`] is implicitly closed,
    /// so open strokes — the common case for lettering — are carried by this flag.
    /// Defaults to `false`: an engraved chain is a stroke unless said otherwise.
    #[serde(default)]
    pub closed: bool,
    /// Absolute Z of the surface being engraved (usually the stock top).
    pub top: f64,
    /// Engraving depth below `top`, mm (> 0), as a positive magnitude. The groove
    /// width follows from this and the tool's V angle and tip radius.
    pub depth: f64,
    /// Maximum depth per pass, mm. `0` (the default) cuts the full `depth` in one
    /// pass — normal for shallow engraving; a deeper groove steps down.
    #[serde(default)]
    pub stepdown: f64,
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the tool's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Selected cutting preset baseline; operation values may be manually overridden.
    #[serde(default)]
    pub tool_preset: Option<crate::ToolPreset>,
    /// Concrete cutting cooling; changing library presets never rewrites saved operations.
    #[serde(default)]
    pub cooling: crate::Cooling,
    /// Independent ramp and lead-in/out feed overrides.
    #[serde(default)]
    pub entry_feeds: crate::EntryFeeds,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed for the approach in Z, mm/min.
    pub plunge_feed: f64,
    /// Preferred start location (part XY) for a **closed** path: the loop begins at the
    /// vertex nearest here. Ignored for an open stroke, which must start at its own
    /// first vertex. `None` starts at the chain's first vertex.
    #[serde(default)]
    pub start: Option<[f64; 2]>,
    /// Preferred exit location in setup XY, reached along the completed toolpath.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end: Option<[f64; 2]>,
}

/// A **V-carving** operation: the boundary outlines an *area* to carve, and the V-bit's
/// flanks land on that boundary rather than its tip running along it.
///
/// The relation to [`EngraveOp`] is exactly Profile → Pocket: an engraving *is* the path
/// it follows, a carving *consumes* the region a path encloses.
///
/// - **The tool never touches the boundary.** It follows inward offsets of it, and at
///   inward distance `w` it sits at the depth where its groove is `2·w` wide
///   ([`cam_geo::vtip_depth_for_half_width`]). So the carved wall meets the boundary at
///   the surface, and the depth *varies* with the shape.
/// - **Depth is derived, not dictated.** `depth` is a **cap**: the carve reaches it only
///   where the region is wide enough. Where the region is wider still, a flat land
///   remains at `depth` — which is what `clear_tool` is for.
/// - **The path must be closed.** There is no interior to consume otherwise.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CarveOp {
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub parameter_expressions: std::collections::BTreeMap<crate::OperationParameter, String>,
    /// Operation id.
    pub id: u32,
    /// The V-bit — the defining tool, chosen in the creation wizard. The strategy reads
    /// its half-angle and tip radius; the carve's shape follows from them.
    pub tool: u32,
    /// The clearing pass that removes the flat land at full depth **before** the V-bit
    /// runs. `None` means the V-bit does the lot, which cuts but leaves a **ridged**
    /// floor — a cone cannot leave a flat one — and is warned about, not rejected.
    #[serde(default)]
    pub clear: Option<CarveClearing>,
    /// The closed boundary of the region to carve, in the part/WCS frame.
    pub boundary: Contour,
    /// Closed islands left uncut — holes, and the counters of letters. Auto-populated
    /// from the picked region's nesting.
    #[serde(default)]
    pub islands: Vec<Contour>,
    /// Absolute Z of the surface being carved (usually the stock top).
    pub top: f64,
    /// **Maximum** depth below `top`, mm (> 0), as a positive magnitude. The shape sets
    /// the actual depth everywhere; this caps it.
    pub depth: f64,
    /// Hold-off from the boundary, mm. Positive leaves material (same sign convention as
    /// [`ProfileOp::offset`]), shrinking the carved region.
    #[serde(default)]
    pub offset: f64,
    /// Radial spacing of the **wall** rings, mm — a *roughing* control, not a finish
    /// one.
    ///
    /// The finished wall is cut by the deepest ring alone: that pass's flank spans from
    /// the boundary down to its tip, so every shallower ring cuts a narrower V entirely
    /// inside it. They exist to limit how much material one pass takes, and to reach
    /// into convex corners, which the deepest ring cannot. A coarser step therefore
    /// costs tool load, not surface quality. `0` (the default) derives one.
    #[serde(default)]
    pub ring_step: f64,
    /// Target ridge height left on the **flat floor**, mm — the true finish control.
    ///
    /// Where the floor is flat but the tool is a cone, adjacent passes leave a ridge of
    /// `vtip_depth_for_half_width(half the spacing)` between them. Asking for a ridge
    /// height instead of a spacing lets the spacing open up wherever the tool's geometry
    /// allows: a rounded tip clears a much wider band at the same ridge than a sharp one
    /// does. `0` (the default) uses a sensible fine value.
    #[serde(default)]
    pub scallop: f64,
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the V-bit's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    /// Applies to the whole carve, including any clearing pass.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Selected cutting preset baseline; operation values may be manually overridden.
    #[serde(default)]
    pub tool_preset: Option<crate::ToolPreset>,
    /// Concrete cutting cooling; changing library presets never rewrites saved operations.
    #[serde(default)]
    pub cooling: crate::Cooling,
    /// Independent ramp and lead-in/out feed overrides.
    #[serde(default)]
    pub entry_feeds: crate::EntryFeeds,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed for the approach in Z, mm/min.
    pub plunge_feed: f64,
    /// How the V-bit enters the material at each ring.
    ///
    /// A carve can run to hundreds of rings, and until v13 every one of them dropped
    /// **straight** in — one real export showed seventeen vertical plunges up to 3 mm
    /// deep. That is *safe* (a V-bit has a cutting tip, so [`crate::Plunge::Straight`]
    /// passes the plunge guard where a non-centre-cutting end mill would not), but it is
    /// a question of finish and tool life, and the operator had no say in it.
    ///
    /// Applies to the **V-bit's** own rings only. The clearing pass has its own style, in
    /// [`CarveClearing::params`] — they are different tools cutting different things, and
    /// the end mill is the one that cannot plunge at all.
    ///
    /// `#[serde(default)]` is [`crate::Plunge::Straight`], which is exactly what every
    /// pre-v13 carve did, so an older file keeps the path it was cut with.
    #[serde(default)]
    pub plunge: Plunge,
    /// Link the rings **without lifting** where it is safe to do so, instead of
    /// retracting to clearance and plunging afresh for every ring.
    ///
    /// A carve can run to hundreds of rings, and a retract cycle each costs far more
    /// time than the cut. Staying down is safe because the tool at a ring's own depth
    /// never cuts below the intended surface anywhere at or beyond that ring's inward
    /// distance — see the strategy's module docs for why — and the strategy verifies
    /// each individual link against that region, retracting for the ones that fail.
    ///
    /// `false` (the serde default, the conservative reading of an absent field) lifts
    /// between every ring.
    #[serde(default)]
    pub stay_down: bool,
    /// Preferred start location (part XY): each ring begins at the point nearest here.
    /// `None` uses the strategy's default entry.
    #[serde(default)]
    pub start: Option<[f64; 2]>,
    /// Preferred exit location in setup XY, reached along the completed toolpath.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end: Option<[f64; 2]>,
}

/// An operation in a setup. An enum so a setup holds a heterogeneous, ordered
/// list.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Operation {
    /// A profiling operation.
    Profile(ProfileOp),
    /// A drilling operation.
    Drill(DrillOp),
    /// A pocket-clearing operation.
    Pocket(PocketOp),
    /// 3D adaptive roughing over a CAD solid.
    Adaptive(AdaptiveOp),
    /// Equally spaced, surface-following 3D finishing passes.
    Parallel(ParallelOp),
    /// Project a spiral or concentric circles onto a CAD surface.
    Spiral(SpiralOp),
    /// A facing operation.
    Face(FaceOp),
    /// A chamfering operation.
    Chamfer(ChamferOp),
    /// A thread-milling operation.
    Thread(ThreadOp),
    /// Helical milling of circular holes and bosses.
    Bore(BoreOp),
    /// A V-carve engraving operation.
    Engrave(EngraveOp),
    /// A V-carving operation over a region.
    Carve(CarveOp),
}

impl Operation {
    /// The operation's id.
    pub fn id(&self) -> u32 {
        match self {
            Operation::Profile(op) => op.id,
            Operation::Drill(op) => op.id,
            Operation::Pocket(op) => op.id,
            Operation::Adaptive(op) => op.id,
            Operation::Parallel(op) => op.id,
            Operation::Spiral(op) => op.id,
            Operation::Face(op) => op.id,
            Operation::Chamfer(op) => op.id,
            Operation::Bore(op) => op.id,
            Operation::Thread(op) => op.id,
            Operation::Engrave(op) => op.id,
            Operation::Carve(op) => op.id,
        }
    }

    /// The number of the operation's **defining** tool — the one that does the work the
    /// operation is named for. An operation may use more than one; see [`tools`](Self::tools).
    pub fn tool(&self) -> u32 {
        match self {
            Operation::Profile(op) => op.tool,
            Operation::Drill(op) => op.tool,
            Operation::Pocket(op) => op.tool,
            Operation::Adaptive(op) => op.tool,
            Operation::Parallel(op) => op.tool,
            Operation::Spiral(op) => op.tool,
            Operation::Face(op) => op.tool,
            Operation::Chamfer(op) => op.tool,
            Operation::Bore(op) => op.tool,
            Operation::Thread(op) => op.tool,
            Operation::Engrave(op) => op.tool,
            Operation::Carve(op) => op.tool,
        }
    }

    /// Every tool the operation uses, **in cutting order** — so `tools()[0]` is the tool
    /// that must be in the spindle when the operation's fragment begins, and the last is
    /// the one left there when it ends.
    ///
    /// All single-tool operations return just [`tool`](Self::tool). Only [`CarveOp`]
    /// returns two, and only when its clearing tool is set: the end mill clears the flat
    /// land *first*, then the V-bit carves. The operation's own strategy emits the
    /// intervening [`cam_cldata::Step::ToolChange`], since it alone knows the order.
    pub fn tools(&self) -> Vec<u32> {
        match self {
            Operation::Carve(op) => match op.clear.map(|c| c.tool) {
                Some(clear) if clear != op.tool => vec![clear, op.tool],
                _ => vec![op.tool],
            },
            other => vec![other.tool()],
        }
    }

    /// The operation's commanded spindle speed, rpm. `0.0` means "unset" — the
    /// planner falls back to the job default rather than commanding a zero.
    pub fn spindle_rpm(&self) -> f64 {
        match self {
            Operation::Profile(op) => op.spindle_rpm,
            Operation::Drill(op) => op.spindle_rpm,
            Operation::Pocket(op) => op.spindle_rpm,
            Operation::Adaptive(op) => op.spindle_rpm,
            Operation::Parallel(op) => op.spindle_rpm,
            Operation::Spiral(op) => op.spindle_rpm,
            Operation::Face(op) => op.spindle_rpm,
            Operation::Chamfer(op) => op.spindle_rpm,
            Operation::Bore(op) => op.spindle_rpm,
            Operation::Thread(op) => op.spindle_rpm,
            Operation::Engrave(op) => op.spindle_rpm,
            Operation::Carve(op) => op.spindle_rpm,
        }
    }

    /// Set the operation's commanded spindle speed, rpm, whatever its kind.
    pub fn set_spindle_rpm(&mut self, rpm: f64) {
        match self {
            Operation::Profile(op) => op.spindle_rpm = rpm,
            Operation::Drill(op) => op.spindle_rpm = rpm,
            Operation::Pocket(op) => op.spindle_rpm = rpm,
            Operation::Adaptive(op) => op.spindle_rpm = rpm,
            Operation::Parallel(op) => op.spindle_rpm = rpm,
            Operation::Spiral(op) => op.spindle_rpm = rpm,
            Operation::Face(op) => op.spindle_rpm = rpm,
            Operation::Chamfer(op) => op.spindle_rpm = rpm,
            Operation::Bore(op) => op.spindle_rpm = rpm,
            Operation::Thread(op) => op.spindle_rpm = rpm,
            Operation::Engrave(op) => op.spindle_rpm = rpm,
            Operation::Carve(op) => op.spindle_rpm = rpm,
        }
    }

    /// The operation's work-datum index (1-based). Datum 1 is the base work
    /// coordinate system; higher values select further fixtures/setups, which a
    /// post lowers to its dialect (Okuma `G15 H<n>`).
    pub fn work_offset(&self) -> u32 {
        match self {
            Operation::Profile(op) => op.work_offset,
            Operation::Drill(op) => op.work_offset,
            Operation::Pocket(op) => op.work_offset,
            Operation::Adaptive(op) => op.work_offset,
            Operation::Parallel(op) => op.work_offset,
            Operation::Spiral(op) => op.work_offset,
            Operation::Face(op) => op.work_offset,
            Operation::Chamfer(op) => op.work_offset,
            Operation::Bore(op) => op.work_offset,
            Operation::Thread(op) => op.work_offset,
            Operation::Engrave(op) => op.work_offset,
            Operation::Carve(op) => op.work_offset,
        }
    }

    /// Set the operation's work-datum index (1-based), whatever its kind.
    pub fn set_work_offset(&mut self, work_offset: u32) {
        match self {
            Operation::Profile(op) => op.work_offset = work_offset,
            Operation::Drill(op) => op.work_offset = work_offset,
            Operation::Pocket(op) => op.work_offset = work_offset,
            Operation::Adaptive(op) => op.work_offset = work_offset,
            Operation::Parallel(op) => op.work_offset = work_offset,
            Operation::Spiral(op) => op.work_offset = work_offset,
            Operation::Face(op) => op.work_offset = work_offset,
            Operation::Chamfer(op) => op.work_offset = work_offset,
            Operation::Bore(op) => op.work_offset = work_offset,
            Operation::Thread(op) => op.work_offset = work_offset,
            Operation::Engrave(op) => op.work_offset = work_offset,
            Operation::Carve(op) => op.work_offset = work_offset,
        }
    }

    /// Overwrite the operation's id, whatever its kind.
    pub fn set_id(&mut self, id: u32) {
        match self {
            Operation::Profile(op) => op.id = id,
            Operation::Drill(op) => op.id = id,
            Operation::Pocket(op) => op.id = id,
            Operation::Adaptive(op) => op.id = id,
            Operation::Parallel(op) => op.id = id,
            Operation::Spiral(op) => op.id = id,
            Operation::Face(op) => op.id = id,
            Operation::Chamfer(op) => op.id = id,
            Operation::Bore(op) => op.id = id,
            Operation::Thread(op) => op.id = id,
            Operation::Engrave(op) => op.id = id,
            Operation::Carve(op) => op.id = id,
        }
    }

    /// Overwrite the number of the operation's **defining** tool, whatever its kind.
    /// Secondary tools are untouched — to rewrite *every* reference (renumbering), use
    /// [`map_tools`](Self::map_tools).
    pub fn set_tool(&mut self, tool: u32) {
        match self {
            Operation::Profile(op) => op.tool = tool,
            Operation::Drill(op) => op.tool = tool,
            Operation::Pocket(op) => op.tool = tool,
            Operation::Adaptive(op) => op.tool = tool,
            Operation::Parallel(op) => op.tool = tool,
            Operation::Spiral(op) => op.tool = tool,
            Operation::Face(op) => op.tool = tool,
            Operation::Chamfer(op) => op.tool = tool,
            Operation::Bore(op) => op.tool = tool,
            Operation::Thread(op) => op.tool = tool,
            Operation::Engrave(op) => op.tool = tool,
            Operation::Carve(op) => op.tool = tool,
        }
    }

    /// Rewrite **every** tool reference the operation holds through `f` — the defining
    /// tool and any secondary one. Used by tool-number reconciliation (see
    /// [`crate::reconcile`]) when a setup adopts the shop's canonical numbering; a
    /// secondary reference left behind would dangle or, worse, silently point at a
    /// different tool that inherited the old number.
    pub fn map_tools(&mut self, f: impl Fn(u32) -> u32) {
        let t = f(self.tool());
        self.set_tool(t);
        if let Operation::Carve(op) = self {
            if let Some(c) = &mut op.clear {
                c.tool = f(c.tool);
            }
        }
    }

    /// Whether two operations describe the **same work** — identical in every
    /// field *except* their `id` and preset provenance (same kind, tool, geometry, depths,
    /// feeds, leads…). Two such operations emit byte-identical toolpaths, so if
    /// both reach the post the machine cuts the same path twice. Used to flag
    /// exact duplicates before export.
    pub fn same_work(&self, other: &Operation) -> bool {
        // Normalise the id on a copy, then lean on the derived structural
        // equality — which automatically covers any field added later.
        let mut probe = self.clone();
        probe.set_id(other.id());
        probe.clear_tool_preset();
        let mut other = other.clone();
        other.clear_tool_preset();
        probe == other
    }
}

/// An additional workpiece origin beyond the base [`Setup::origin`] — a distinct
/// datum for a re-fixturing/reorientation of the part (Case 1: machine one side,
/// flip, machine another). The `index` is the machine work-coordinate selector the
/// post lowers to a dialect code (Okuma `G15 H<index>`); the `position` is the
/// part-space point that becomes *that* datum's `(0,0,0)`, the same role
/// [`Setup::origin`] plays for the base (index 1). Operations reference which origin
/// they belong to via [`Operation::work_offset`].
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Origin {
    /// The machine work-coordinate index (index 1 is the base by default; editable).
    pub index: u32,
    /// The part-space point this datum is zeroed to. Subtracted from its group's
    /// coordinates at post time, exactly as the base origin is.
    pub position: [f64; 3],
}

/// A machining setup: one fixturing of the stock, its safety planes, and the
/// ordered operations performed in it.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Setup {
    /// Stable identity within the document.
    #[serde(default)]
    pub id: u64,
    /// Setup orientation relative to shared model coordinates.
    #[serde(default)]
    pub frame: crate::SetupFrame,
    /// Material name for the blank.
    #[serde(default)]
    pub material: String,
    /// Link to the managed material registry; name remains a portable display snapshot.
    #[serde(default)]
    pub material_id: Option<u64>,
    /// Model instance identities included in machining for this setup. Empty selects none.
    #[serde(default)]
    pub model_instances: Vec<u64>,
    /// Body instances used as fixtures; excluded from machining models and stock.
    #[serde(default)]
    pub fixture_instances: Vec<u64>,
    /// Keep-out distances around fixture bodies in setup axes, millimetres.
    #[serde(default)]
    pub fixture_clearance: crate::FixtureClearance,
    /// NC program name for this setup.
    #[serde(default)]
    pub program_name: String,
    /// Human-readable name.
    pub name: String,
    /// Legacy safety-plane defaults for operations without an explicit entry.
    pub heights: Heights,
    /// Operation safety planes keyed by stable operation ID. Missing entries use
    /// the legacy setup defaults, preserving existing projects.
    #[serde(
        default,
        skip_serializing_if = "std::collections::BTreeMap::is_empty",
        deserialize_with = "deserialize_operation_map"
    )]
    pub operation_heights: std::collections::BTreeMap<u32, Heights>,
    /// Parametric operation heights; absent entries retain the legacy behavior.
    #[serde(
        default,
        skip_serializing_if = "std::collections::BTreeMap::is_empty",
        deserialize_with = "deserialize_operation_map"
    )]
    pub height_settings: std::collections::BTreeMap<u32, crate::OperationHeightSettings>,
    /// Radial and axial stock allowances keyed by stable operation ID.
    #[serde(
        default,
        skip_serializing_if = "std::collections::BTreeMap::is_empty",
        deserialize_with = "deserialize_operation_map"
    )]
    pub stock_to_leave: std::collections::BTreeMap<u32, StockToLeave>,
    /// The raw stock.
    pub stock: Stock,
    /// Tools available in this setup.
    pub tools: Vec<Tool>,
    /// Operations, in execution order.
    pub operations: Vec<Operation>,
    /// Reusable pattern groups. Group members remain canonical operations above.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub patterns: Vec<OperationPattern>,
    /// Additional workpiece origins (index ≥ 2) beyond the base [`origin`](Self::origin)
    /// (index 1) — one per re-fixturing/reorientation of the part. Each operation runs
    /// under the origin named by its [`work_offset`](Operation::work_offset); a group of
    /// operations on a new origin is a physical reorientation, emitted under its own
    /// `G15 H<index>` with an operator stop before it. Empty (the default) = one origin.
    #[serde(default)]
    pub extra_origins: Vec<Origin>,
    /// The **workpiece origin** (datum): the part-space point that becomes G-code
    /// `(0,0,0)`. The post subtracts it from every emitted coordinate, so the
    /// operator zeros the machine's work offset (G54) at this point. `[0,0,0]`
    /// means the part frame *is* the program frame. Design/sim stay in part space.
    #[serde(default)]
    pub origin: [f64; 3],
    /// The machine work-coordinate index (`G15 H<n>`) the **base** origin selects.
    /// `1` by default; editable and swappable like an extra origin's index.
    #[serde(default = "default_origin_index")]
    pub origin_index: u32,
    /// Optional **tool-change height** (absolute setup Z, mm): the plane the planner lifts to
    /// before a tool change (`M6`) or a manual reorientation (`M00`), then descends from
    /// once the next operation takes over. It sits **above** the toolpath
    /// [`clearance`](Heights::clearance) plane — a tool change wants maximum clearance
    /// (an ATC swing, an operator's hands during a re-fixture), not the cutting-traverse
    /// height. The editor presents this relative to the base workpiece origin.
    /// `None` (the default) resolves to the machine's max WCS Z at plan time
    /// via [`resolved_tool_change_height`](Self::resolved_tool_change_height).
    #[serde(default)]
    pub tool_change_height: Option<f64>,
}

impl Setup {
    /// Stock allowances for an operation; absent entries preserve legacy toolpaths.
    pub fn stock_to_leave_for(&self, operation_id: u32) -> StockToLeave {
        self.stock_to_leave
            .get(&operation_id)
            .copied()
            .unwrap_or_default()
    }

    /// Validate map keys, allowance magnitudes, and operation-specific support.
    pub fn validate_stock_to_leave(&self) -> Result<(), String> {
        for (operation_id, allowances) in &self.stock_to_leave {
            let operation = self
                .operations
                .iter()
                .find(|op| op.id() == *operation_id)
                .ok_or_else(|| {
                    format!("stock-to-leave settings reference missing operation {operation_id}")
                })?;
            allowances.validate()?;
            let (radial, axial) = allowances.effective();
            let supported = match operation {
                Operation::Profile(_)
                | Operation::Pocket(_)
                | Operation::Adaptive(_)
                | Operation::Parallel(_)
                | Operation::Spiral(_) => true,
                Operation::Face(_) => {
                    if radial > 0.0 {
                        return Err(format!("operation {operation_id}: Face supports axial stock to leave only; radial allowance must be 0 mm"));
                    }
                    true
                }
                Operation::Drill(_)
                | Operation::Bore(_)
                | Operation::Chamfer(_)
                | Operation::Thread(_)
                | Operation::Engrave(_)
                | Operation::Carve(_) => false,
            };
            if !supported && (radial > 0.0 || axial > 0.0) {
                return Err(format!("operation {operation_id}: stock to leave is supported for Profile, Pocket, Face, Adaptive, Parallel, and Spiral only"));
            }
        }
        Ok(())
    }

    /// Flattened execution view: a pattern appears at its first source, remaining
    /// grouped sources are omitted, and ordinary operations appear once.
    pub fn execution_items(&self) -> Vec<ExecutionItem<'_>> {
        let mut items = Vec::new();
        let mut emitted = std::collections::BTreeSet::new();
        for op in &self.operations {
            if let Some(pattern) = self.patterns.iter().find(|p| p.members.contains(&op.id())) {
                if emitted.insert(pattern.id) {
                    items.push(ExecutionItem::Pattern(pattern));
                }
            } else {
                items.push(ExecutionItem::Operation(op));
            }
        }
        items
    }

    /// Validate pattern structure and shared datum/order invariants.
    pub fn validate_patterns(&self) -> Result<(), String> {
        let mut ids = std::collections::BTreeSet::new();
        let mut membership = std::collections::BTreeSet::new();
        let positions: std::collections::HashMap<u32, usize> = self
            .operations
            .iter()
            .enumerate()
            .map(|(i, o)| (o.id(), i))
            .collect();
        for p in &self.patterns {
            if p.id == 0 || !ids.insert(p.id) {
                return Err(format!("pattern ID {} is zero or duplicated", p.id));
            }
            p.options.validate(p.kind)?;
            if p.members.is_empty() {
                return Err(format!("pattern '{}' has no source operations", p.name));
            }
            let mut indices = Vec::new();
            let mut datum = None;
            for id in &p.members {
                let i = *positions.get(id).ok_or_else(|| {
                    format!("pattern '{}' references missing operation {id}", p.name)
                })?;
                if !membership.insert(*id) {
                    return Err(format!("operation {id} belongs to multiple patterns"));
                }
                let op = &self.operations[i];
                if datum.is_some_and(|d| d != op.work_offset()) {
                    return Err(format!(
                        "pattern '{}' sources must share one work datum",
                        p.name
                    ));
                }
                datum = Some(op.work_offset());
                indices.push(i);
            }
            let min = *indices.iter().min().unwrap();
            let max = *indices.iter().max().unwrap();
            if max - min + 1 != indices.len() || indices.windows(2).any(|w| w[1] != w[0] + 1) {
                return Err(format!(
                    "pattern '{}' sources must be a contiguous ordered block",
                    p.name
                ));
            }
        }
        Ok(())
    }
    /// Effective safety planes for one operation.
    pub fn heights_for(&self, operation: u32) -> Heights {
        self.operation_heights
            .get(&operation)
            .copied()
            .unwrap_or(self.heights)
    }

    /// The tool-change height to plan with: the explicit
    /// [`tool_change_height`](Self::tool_change_height) if set, else `machine_max_z`
    /// (the top of the machine's Z envelope, relative to the base work datum).
    /// The result is in setup coordinates, like the planner's clearance planes.
    /// Explicit heights are already stored in setup coordinates.
    pub fn resolved_tool_change_height(&self, machine_max_z: f64) -> f64 {
        self.tool_change_height
            .unwrap_or(machine_max_z + self.origin[2])
    }

    /// The part-space position of the origin with the given `index`: the base
    /// [`origin`](Self::origin) when `index == origin_index`, otherwise the matching
    /// [`extra_origins`](Self::extra_origins) entry. Falls back to the base origin for
    /// an unknown index, so a stale [`work_offset`](Operation::work_offset) can never
    /// silently pick a bogus datum.
    pub fn origin_position(&self, index: u32) -> [f64; 3] {
        if index == self.origin_index {
            return self.origin;
        }
        self.extra_origins
            .iter()
            .find(|o| o.index == index)
            .map_or(self.origin, |o| o.position)
    }

    /// Every origin index this setup defines, base first: `origin_index` then each of
    /// [`extra_origins`](Self::extra_origins) in order.
    pub fn origin_indices(&self) -> Vec<u32> {
        std::iter::once(self.origin_index)
            .chain(self.extra_origins.iter().map(|o| o.index))
            .collect()
    }

    /// The lowest unused origin index (for adding a new one): one past the highest.
    pub fn next_origin_index(&self) -> u32 {
        self.origin_indices().into_iter().max().map_or(2, |m| m + 1)
    }
}

/// One item in a setup's flattened execution view.
#[derive(Clone, Copy, Debug)]
pub enum ExecutionItem<'a> {
    Operation(&'a Operation),
    Pattern(&'a OperationPattern),
}

/// Default for a `#[serde(default)]` base-origin index: `1`.
fn default_origin_index() -> u32 {
    1
}

/// One import's consecutive ranges in the shared drawing geometry.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Drawing {
    pub name: String,
    pub regions: usize,
    pub paths: usize,
}

/// The top-level document: shared geometry and an ordered collection of setups.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Document {
    /// User-defined browser names, keyed by stable operation ID.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub operation_names: std::collections::BTreeMap<u32, String>,
    /// Per-operation 3+1 indexing; stable operation IDs are document-wide.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub rotary_indexes: std::collections::BTreeMap<u32, crate::RotaryIndex>,
    /// Schema version this document conforms to.
    pub schema_version: u32,
    /// Independent physical workholding setups, in browser order.
    #[serde(default)]
    pub setups: Vec<Setup>,
    /// Session-only active setup. It is intentionally omitted from project files.
    #[serde(skip)]
    pub active_setup_id: u64,
    /// Self-contained imported geometry and persistent operation selections.
    #[serde(default)]
    pub geometry: crate::GeometryState,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub drawings: Vec<Drawing>,
    /// Operation IDs whose cutting geometry is linked to persistent CAD topology.
    /// This remains separate from the binding map so a missing binding cannot
    /// silently turn a linked operation into an explicit-contour operation.
    #[serde(default)]
    pub cad_linked_operations: std::collections::BTreeSet<u32>,
    /// Suppression changes manufacturing inclusion, independently of visibility.
    #[serde(default)]
    pub excluded_operations: std::collections::BTreeSet<u32>,
    /// Disabled work datums scoped to a physical setup.
    #[serde(default)]
    pub disabled_origins: std::collections::BTreeMap<u64, std::collections::BTreeSet<u32>>,
    /// IDs below these marks are never reused after deletion.
    #[serde(default)]
    pub operation_id_floor: u32,
    #[serde(default)]
    pub setup_id_floor: u64,
    #[serde(default)]
    pub asset_id_floor: u64,
    /// Highest CAD revision issued, including revisions discarded by undo.
    #[serde(default)]
    pub cad_revision_floor: u64,
    #[serde(default)]
    pub instance_id_floor: u64,
    #[serde(default)]
    pub pattern_id_floor: u64,
}

// Active setup is session/view state: it is intentionally omitted from the wire
// format and therefore must not make otherwise identical saved documents unequal.
impl PartialEq for Document {
    fn eq(&self, other: &Self) -> bool {
        self.operation_names == other.operation_names
            && self.schema_version == other.schema_version
            && self.rotary_indexes == other.rotary_indexes
            && self.setups == other.setups
            && self.geometry == other.geometry
            && self.drawings == other.drawings
            && self.cad_linked_operations == other.cad_linked_operations
            && self.excluded_operations == other.excluded_operations
            && self.disabled_origins == other.disabled_origins
            && self.operation_id_floor == other.operation_id_floor
            && self.setup_id_floor == other.setup_id_floor
            && self.asset_id_floor == other.asset_id_floor
            && self.cad_revision_floor == other.cad_revision_floor
            && self.instance_id_floor == other.instance_id_floor
            && self.pattern_id_floor == other.pattern_id_floor
    }
}

impl Document {
    /// Whether one source operation participates in the selected setup's run,
    /// accounting for operation suppression, disabled datum and group suppression.
    pub fn operation_is_included(&self, setup_id: u64, operation_id: u32) -> bool {
        if self.excluded_operations.contains(&operation_id) {
            return false;
        }
        let Some(setup) = self.setups.iter().find(|s| s.id == setup_id) else {
            return false;
        };
        let Some(operation) = setup.operations.iter().find(|op| op.id() == operation_id) else {
            return false;
        };
        if self
            .disabled_origins
            .get(&setup_id)
            .is_some_and(|set| set.contains(&operation.work_offset()))
        {
            return false;
        }
        !setup
            .patterns
            .iter()
            .any(|pattern| pattern.suppressed && pattern.members.contains(&operation_id))
    }

    /// Wrap a setup in a document stamped with the current [`SCHEMA_VERSION`].
    pub fn new(setup: Setup) -> Self {
        let mut setup = setup;
        if setup.id == 0 {
            setup.id = 1;
        }
        Self {
            active_setup_id: setup.id,
            setups: vec![setup],
            ..Self::empty()
        }
    }

    /// A geometry-only project, before its first machining setup is created.
    pub fn empty() -> Self {
        Self {
            schema_version: SCHEMA_VERSION,
            operation_names: Default::default(),
            rotary_indexes: Default::default(),
            active_setup_id: 0,
            setups: Vec::new(),
            geometry: crate::GeometryState::default(),
            drawings: Vec::new(),
            cad_linked_operations: Default::default(),
            excluded_operations: Default::default(),
            disabled_origins: Default::default(),
            operation_id_floor: 0,
            setup_id_floor: 0,
            asset_id_floor: 0,
            cad_revision_floor: 0,
            instance_id_floor: 0,
            pattern_id_floor: 0,
        }
    }
    pub fn active_setup(&self) -> Option<&Setup> {
        self.setups
            .iter()
            .find(|s| s.id == self.active_setup_id)
            .or_else(|| self.setups.first())
    }
    pub fn setup(&self) -> &Setup {
        self.active_setup().expect("document must contain a setup")
    }
    pub fn setup_mut(&mut self) -> &mut Setup {
        let id = self.active_setup_id;
        let index = self.setups.iter().position(|s| s.id == id).unwrap_or(0);
        self.setups
            .get_mut(index)
            .expect("document must contain a setup")
    }
    pub fn next_setup_id(&self) -> u64 {
        self.setups
            .iter()
            .map(|s| s.id)
            .max()
            .unwrap_or(0)
            .saturating_add(1)
            .max(1)
            .max(self.setup_id_floor)
    }
    /// Record allocated IDs before an edit can delete them.
    pub fn record_ids(&mut self) {
        self.asset_id_floor = self.asset_id_floor.max(
            self.geometry
                .assets
                .iter()
                .map(|a| a.id.saturating_add(1))
                .max()
                .unwrap_or(1),
        );
        self.instance_id_floor = self.instance_id_floor.max(
            self.geometry
                .instances
                .iter()
                .map(|i| i.id.saturating_add(1))
                .max()
                .unwrap_or(1),
        );
        self.pattern_id_floor = self.pattern_id_floor.max(
            self.setups
                .iter()
                .flat_map(|s| &s.patterns)
                .map(|p| p.id.saturating_add(1))
                .max()
                .unwrap_or(1),
        );
        self.operation_id_floor = self.operation_id_floor.max(
            self.setups
                .iter()
                .flat_map(|s| &s.operations)
                .map(|o| o.id().saturating_add(1))
                .max()
                .unwrap_or(0),
        );
        self.setup_id_floor = self.setup_id_floor.max(
            self.setups
                .iter()
                .map(|s| s.id.saturating_add(1))
                .max()
                .unwrap_or(1),
        );
    }
}

#[cfg(test)]
mod pattern_tests {
    use super::*;

    #[test]
    fn pattern_options_preserve_legacy_grid_and_deserialization() {
        let pattern: OperationPattern = serde_json::from_value(serde_json::json!({
            "id": 1, "name": "Legacy", "members": [3],
            "kind": { "kind": "rectangular", "nx": 3, "ny": 2, "dx": 25.0, "dy": -10.0 }
        }))
        .unwrap();
        pattern.options.validate(pattern.kind).unwrap();
        for index in 0..6 {
            assert_eq!(
                pattern.options.transform_xy(pattern.kind, index),
                pattern.kind.transform_xy(index)
            );
        }
        assert_eq!(
            pattern.options.instance_indices(pattern.kind),
            vec![0, 1, 2, 3, 4, 5]
        );
    }

    #[test]
    fn linear_extent_rotates_both_directions_and_reverses_independently() {
        let kind = PatternKind::Rectangular {
            nx: 3,
            ny: 2,
            dx: 40.0,
            dy: 15.0,
        };
        let mut options = PatternOptions {
            spacing: PatternSpacing::Extent,
            ..Default::default()
        };
        options.directions[0].angle_degrees = 30.0;
        options.directions[1].angle_degrees = 120.0;
        options.directions[1].reversed = true;
        options.validate(kind).unwrap();
        let (offset, _) = options.transform_xy(kind, 5).unwrap();
        assert!((offset[0] - (40.0 * 30f64.to_radians().cos() + 7.5)).abs() < 1e-10);
        assert!((offset[1] - (20.0 - 15.0 * 120f64.to_radians().sin())).abs() < 1e-10);
    }

    #[test]
    fn symmetric_pattern_keeps_source_identity_and_filters_before_reversing() {
        let kind = PatternKind::Rectangular {
            nx: 5,
            ny: 1,
            dx: 40.0,
            dy: 0.0,
        };
        let mut options = PatternOptions {
            spacing: PatternSpacing::Extent,
            keep_original: false,
            reverse: true,
            ..Default::default()
        };
        options.directions[0].symmetric = true;
        assert_eq!(options.instance_indices(kind), vec![4, 3, 2, 1]);
        let offsets: Vec<_> = (0..5)
            .map(|i| options.transform_xy(kind, i).unwrap().0[0])
            .collect();
        assert_eq!(offsets, vec![0.0, 10.0, -10.0, 20.0, -20.0]);
    }

    #[test]
    fn angular_spacing_and_extent_have_distinct_endpoints() {
        let make = |distribution| PatternKind::Circular {
            center_xy: [5.0, -3.0],
            count: 4,
            distribution,
            direction: PatternDirection::Cw,
        };
        let spacing = make(CircularDistribution::Spacing {
            angle_degrees: 45.0,
        });
        spacing.validate().unwrap();
        assert_eq!(spacing.transform_xy(3).unwrap().1, -135.0);
        assert_eq!(
            make(CircularDistribution::Partial {
                sweep_degrees: 45.0
            })
            .transform_xy(3)
            .unwrap()
            .1,
            -45.0
        );
        assert_eq!(
            make(CircularDistribution::Full).transform_xy(3).unwrap().1,
            -270.0
        );
        assert!(
            make(CircularDistribution::Spacing {
                angle_degrees: 120.0
            })
            .validate()
            .is_err(),
            "duplicate full-circle endpoint is rejected"
        );
    }

    #[test]
    fn pattern_options_reject_parallel_directions_and_overflow() {
        let kind = PatternKind::Rectangular {
            nx: 3,
            ny: 2,
            dx: 20.0,
            dy: 10.0,
        };
        let mut options = PatternOptions::default();
        options.directions[1].angle_degrees = 180.0;
        assert!(options.validate(kind).is_err());
        options.directions[1].angle_degrees = 90.0;
        assert!(options
            .validate(PatternKind::Rectangular {
                nx: 3,
                ny: 2,
                dx: f64::MAX,
                dy: 10.0
            })
            .is_err());
    }

    #[test]
    fn circular_transform_keeps_identity_and_rotates_about_nonzero_center() {
        let kind = PatternKind::Circular {
            center_xy: [1.0, 1.0],
            count: 4,
            distribution: CircularDistribution::Full,
            direction: PatternDirection::Ccw,
        };
        assert_eq!(kind.transform_xy(0), Some(([0.0, 0.0], 0.0)));
        let (translation, angle) = kind.transform_xy(1).unwrap();
        assert_eq!(translation, [0.0, 0.0]);
        assert_eq!(angle, 90.0);
        let (sin, cos) = angle.to_radians().sin_cos();
        let p = [3.0, 2.0];
        let x = 1.0 + (p[0] - 1.0) * cos - (p[1] - 1.0) * sin + translation[0];
        let y = 1.0 + (p[0] - 1.0) * sin + (p[1] - 1.0) * cos + translation[1];
        assert!((x - 0.0).abs() < 1e-12 && (y - 3.0).abs() < 1e-12);
    }

    #[test]
    fn invalid_zero_partial_sweep_is_rejected() {
        assert!(PatternKind::Circular {
            center_xy: [0.0, 0.0],
            count: 3,
            distribution: CircularDistribution::Partial { sweep_degrees: 0.0 },
            direction: PatternDirection::Ccw
        }
        .validate()
        .is_err());
    }
}

// Internally tagged project files buffer JSON object keys as strings.
fn deserialize_operation_map<'de, D, T>(
    deserializer: D,
) -> Result<std::collections::BTreeMap<u32, T>, D::Error>
where
    D: serde::Deserializer<'de>,
    T: serde::Deserialize<'de>,
{
    let encoded =
        <std::collections::BTreeMap<String, T> as serde::Deserialize>::deserialize(deserializer)?;
    let mut values = std::collections::BTreeMap::new();
    for (key, value) in encoded {
        let id = key.parse::<u32>().map_err(serde::de::Error::custom)?;
        if values.insert(id, value).is_some() {
            return Err(serde::de::Error::custom("duplicate operation height ID"));
        }
    }
    Ok(values)
}

#[cfg(test)]
mod tests {
    #[test]
    fn new_clearing_defaults_to_arc_leads_but_missing_saved_leads_remain_none() {
        let defaults = super::ClearParams::default();
        assert_eq!(defaults.lead_in, super::Lead::Arc { radius: 0.6 });
        assert_eq!(defaults.lead_out, super::Lead::Arc { radius: 0.6 });

        let saved: super::ClearParams = serde_json::from_str("{}").unwrap();
        assert_eq!(saved.lead_in, super::Lead::None);
        assert_eq!(saved.lead_out, super::Lead::None);
    }
    use super::*;
    use crate::{Comp, Lead, Plunge, Side};
    use cam_geo::{Contour, Point};

    fn profile(id: u32) -> Operation {
        Operation::Profile(ProfileOp {
            parameter_expressions: Default::default(),
            wrap: None,
            passes: Default::default(),
            selections: None,
            closed: true,
            additional_chains: Vec::new(),
            tabs: Default::default(),
            spindle_rpm: 0.0,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            clearing: Clearing::default(),
            id,
            tool: 1,
            chain: Contour::new(vec![
                Point::new(0.0, 0.0),
                Point::new(10.0, 0.0),
                Point::new(10.0, 10.0),
                Point::new(0.0, 10.0),
            ]),
            side: Side::Outside,
            comp: Comp::Computed,
            offset: 0.0,
            depth: 4.0,
            stepdown: 2.0,
            stepover: 0.0,
            feed: 300.0,
            plunge_feed: 100.0,
            end: None,
            start: None,
            lead_in: Lead::None,
            lead_out: Lead::None,
            lead_overlap: 0.0,
            plunge: Plunge::Straight,
        })
    }

    #[test]
    fn legacy_profiles_stay_closed_and_multiple_open_chains_roundtrip() {
        let Operation::Profile(mut op) = profile(0) else {
            unreachable!()
        };
        let mut legacy = serde_json::to_value(&op).unwrap();
        legacy.as_object_mut().unwrap().remove("closed");
        legacy.as_object_mut().unwrap().remove("additional_chains");
        let restored: ProfileOp = serde_json::from_value(legacy).unwrap();
        assert!(restored.closed);
        assert!(restored.additional_chains.is_empty());
        op.closed = false;
        op.chain = Contour::new(vec![Point::new(0., 0.), Point::new(5., 0.)]);
        op.additional_chains.push(ProfileContour {
            chain: op.chain.clone(),
            closed: false,
            side: Side::Inside,
        });
        let restored: ProfileOp =
            serde_json::from_str(&serde_json::to_string(&op).unwrap()).unwrap();
        assert_eq!(restored, op);
    }

    #[test]
    fn profile_tabs_roundtrip_and_old_projects_default_to_disabled() {
        let Operation::Profile(mut op) = profile(0) else {
            unreachable!()
        };
        let mut legacy = serde_json::to_value(&op).unwrap();
        legacy.as_object_mut().unwrap().remove("tabs");
        let restored: ProfileOp = serde_json::from_value(legacy).unwrap();
        assert_eq!(restored.tabs, ProfileTabs::default());
        op.tabs = ProfileTabs {
            shape: Default::default(),
            count: None,
            enabled: true,
            width: 9.0,
            height: 3.0,
            spacing: 180.0,
            manual: true,
            positions: vec![[5.0, 0.0]],
        };
        let restored: ProfileOp =
            serde_json::from_str(&serde_json::to_string(&op).unwrap()).unwrap();
        assert_eq!(restored, op);
    }

    #[test]
    fn setup_stock_allowances_roundtrip_with_numeric_operation_ids_and_default_legacy() {
        let mut setup = Setup {
            id: 1,
            frame: Default::default(),
            material: String::new(),
            material_id: None,
            model_instances: Vec::new(),
            fixture_instances: Vec::new(),
            fixture_clearance: Default::default(),
            program_name: String::new(),
            name: "stock allowance test".into(),
            heights: Heights::new(10.0, 5.0, 0.0),
            operation_heights: Default::default(),
            height_settings: Default::default(),
            stock_to_leave: Default::default(),
            stock: Stock::BoundingBox {
                x_offset: 0.0,
                y_offset: 0.0,
                top: 0.0,
                thickness: 5.0,
            },
            tools: Vec::new(),
            operations: vec![profile(7)],
            patterns: Vec::new(),
            extra_origins: Vec::new(),
            origin: [0.0; 3],
            origin_index: 1,
            tool_change_height: None,
        };
        setup.stock_to_leave.insert(
            7,
            StockToLeave {
                enabled: true,
                radial: 0.4,
                axial: 0.25,
            },
        );
        let mut encoded = serde_json::to_value(&setup).unwrap();
        assert_eq!(encoded["stock_to_leave"]["7"]["radial"], 0.4);
        let restored: Setup = serde_json::from_value(encoded.clone()).unwrap();
        assert_eq!(restored.stock_to_leave_for(7), setup.stock_to_leave_for(7));

        encoded.as_object_mut().unwrap().remove("stock_to_leave");
        let legacy: Setup = serde_json::from_value(encoded).unwrap();
        assert_eq!(legacy.stock_to_leave_for(7), StockToLeave::default());
    }

    fn carve(clear_tool: Option<u32>) -> Operation {
        Operation::Carve(CarveOp {
            parameter_expressions: Default::default(),
            spindle_rpm: 0.0,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            id: 3,
            tool: 4,
            clear: clear_tool.map(|t| CarveClearing {
                tool: t,
                params: ClearParams::default(),
            }),
            boundary: Contour::new(vec![
                Point::new(0.0, 0.0),
                Point::new(10.0, 0.0),
                Point::new(10.0, 10.0),
            ]),
            islands: Vec::new(),
            top: 0.0,
            depth: 1.0,
            offset: 0.0,
            ring_step: 0.0,
            scallop: 0.0,
            feed: 300.0,
            plunge_feed: 100.0,
            plunge: Plunge::Straight,
            stay_down: false,
            end: None,
            start: None,
        })
    }

    #[test]
    fn a_single_tool_operation_reports_exactly_one_tool() {
        assert_eq!(profile(1).tools(), vec![1]);
    }

    #[test]
    fn a_carve_reports_the_clearing_tool_first() {
        // Cutting order, not declaration order: the end mill takes the bulk out of the
        // flat land before the fine V-bit ever touches it. `tools()[0]` is what must be
        // in the spindle when the operation's fragment begins, so the order is load
        // bearing, not cosmetic.
        assert_eq!(carve(Some(7)).tools(), vec![7, 4]);
        assert_eq!(carve(None).tools(), vec![4]);
        // A clearing tool that *is* the carving tool is one tool, not two — no change
        // should be emitted between the passes.
        assert_eq!(carve(Some(4)).tools(), vec![4]);
    }

    #[test]
    fn map_tools_rewrites_the_secondary_reference_too() {
        // A renumbering that missed `clear_tool` would leave it pointing at whatever
        // tool inherited the old number — silently machining with the wrong cutter.
        let mut op = carve(Some(7));
        op.map_tools(|t| t + 10);
        assert_eq!(op.tools(), vec![17, 14]);
        // set_tool, by contrast, is deliberately narrow: the defining tool only.
        let mut op = carve(Some(7));
        op.set_tool(9);
        assert_eq!(op.tools(), vec![7, 9]);
    }

    #[test]
    fn face_direction_follows_the_picked_edge() {
        // 60×40 rectangle. Clicking a horizontal edge ⇒ pass along X; clicking a
        // vertical edge ⇒ pass along Y — regardless of which is longer.
        let rect = [
            Point::new(0.0, 0.0),
            Point::new(60.0, 0.0),
            Point::new(60.0, 40.0),
            Point::new(0.0, 40.0),
        ];
        // Pick near the bottom edge (horizontal) → X.
        assert_eq!(Axis::along_edge_at(&rect, Point::new(30.0, 0.5)), Axis::X);
        // Pick near the right edge (vertical) → Y, even though it is the short side.
        assert_eq!(Axis::along_edge_at(&rect, Point::new(59.5, 20.0)), Axis::Y);
        // Degenerate input falls back to X.
        assert_eq!(
            Axis::along_edge_at(&[Point::new(1.0, 1.0)], Point::new(0.0, 0.0)),
            Axis::X
        );
    }

    #[test]
    fn face_direction_longest_edge_fallback() {
        // No pick: default to the longest edge. 40×60 ⇒ long side is Y.
        let tall = [
            Point::new(0.0, 0.0),
            Point::new(40.0, 0.0),
            Point::new(40.0, 60.0),
            Point::new(0.0, 60.0),
        ];
        assert_eq!(Axis::along_longest_edge(&tall), Axis::Y);
    }

    #[test]
    fn same_work_ignores_id_only() {
        // Equal in everything but id → same work.
        assert!(profile(0).same_work(&profile(7)));
        assert!(profile(7).same_work(&profile(0)));
    }

    #[test]
    fn same_work_sees_a_real_difference() {
        let a = profile(0);
        let mut b = profile(1);
        if let Operation::Profile(op) = &mut b {
            op.feed = 301.0; // one field differs
        }
        assert!(!a.same_work(&b), "a differing feed is different work");
    }

    #[test]
    fn stock_resolves_offsets_and_thickness() {
        let stock = Stock::BoundingBox {
            x_offset: 2.0,
            y_offset: 5.0,
            top: 0.0,
            thickness: 20.0,
        };
        let (min, max) = stock.resolve([10.0, 10.0], [70.0, 50.0]);
        assert_eq!(
            min,
            [8.0, 5.0, -20.0],
            "grown per-axis, bottom = top - thickness"
        );
        assert_eq!(max, [72.0, 55.0, 0.0]);
    }

    #[test]
    fn round_stock_resolves_and_round_trips() {
        let stock = Stock::RelativeRound {
            radial: 2.,
            inner_diameter: 4.,
            bottom: 1.,
            top: 3.,
            tube: true,
            axis: None,
        };
        let (lo, hi) = stock.resolve_bounds([-3., -4., 2.], [3., 4., 12.]);
        assert_eq!((lo, hi), ([-7., -7., 1.], [7., 7., 15.]));
        let json = serde_json::to_string(&stock).unwrap();
        assert_eq!(serde_json::from_str::<Stock>(&json).unwrap(), stock);
    }

    #[test]
    fn fixed_and_relative_stock_resolve_all_three_axes() {
        let bounds = ([10.0, 20.0, 30.0], [40.0, 60.0, 50.0]);
        let fixed = Stock::FixedBox {
            size: [50.0, 60.0, 40.0],
            model_position: [0.5, 0.0, 1.0],
        };
        assert_eq!(
            fixed.resolve_bounds(bounds.0, bounds.1),
            ([-0.0, 20.0, 10.0], [50.0, 80.0, 50.0])
        );
        let relative = Stock::RelativeBox {
            offsets: [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
        };
        assert_eq!(
            relative.resolve_bounds(bounds.0, bounds.1),
            ([9.0, 18.0, 27.0], [44.0, 65.0, 56.0])
        );
    }

    #[test]
    fn same_work_across_kinds_is_false() {
        let prof = profile(0);
        let drill = Operation::Drill(DrillOp {
            cycle: Default::default(),
            tip_through_bottom: false,
            geometry: Default::default(),
            hole_heights: Vec::new(),
            parameter_expressions: Default::default(),
            spindle_rpm: 0.0,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            id: 0,
            tool: 1,
            points: vec![[0.0, 0.0]],
            depth: 4.0,
            start_offset: 0.0,
            peck: None,
            dwell: None,
            feed: 100.0,
        });
        assert!(!prof.same_work(&drill));
    }
}
