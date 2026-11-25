//! Geometry-field replacement that preserves every other operation parameter.
use super::*;
impl AppController {
    pub fn operation_kind(operation: &Operation) -> OpKind {
        match operation {
            Operation::Profile(_) => OpKind::Profile,
            Operation::Pocket(_) => OpKind::Pocket,
            Operation::Drill(_) => OpKind::Drill,
            Operation::Face(_) => OpKind::Face,
            Operation::Chamfer(_) => OpKind::Chamfer,
            Operation::Bore(_) => OpKind::Bore,
            Operation::Thread(_) => OpKind::Thread,
            Operation::Engrave(_) => OpKind::Engrave,
            Operation::Carve(_) => OpKind::Carve,
            Operation::Adaptive(_) => OpKind::Adaptive,
            Operation::Parallel(_) => OpKind::Parallel,
            Operation::Spiral(_) => OpKind::Spiral,
        }
    }
    /// Resolve a drawing geometry field without committing or resetting cutting data.
    pub fn operation_with_drawing_geometry(
        &self,
        operation: &Operation,
        selected: &[LoopRef],
    ) -> Result<Operation, String> {
        let kind = Self::operation_kind(operation);
        self.planar_operation_eligibility(kind, selected)?;
        let nested: Vec<_> = if kind == OpKind::Carve
            && selected.len() == 1
            && selected[0].part == LoopPart::Outer
        {
            (0..self.regions[selected[0].region].holes().len())
                .map(|hole| LoopRef::hole(selected[0].region, hole))
                .collect()
        } else if op_selects_circles(kind) {
            Vec::new()
        } else {
            selected[1..].to_vec()
        };
        // The prototype supplies geometry only. A staged tool may not yet be in the document.
        let geometry_tool = self
            .operation(operation.id())
            .map(Operation::tool)
            .unwrap_or(operation.tool());
        let replacement = self
            .build_op(kind, selected[0], &nested, geometry_tool, None, None)
            .ok_or("Selected drawing cannot resolve this geometry field")?;
        let mut draft = operation.clone();
        match (&mut draft, replacement) {
            (Operation::Profile(op), Operation::Profile(new)) => {
                op.chain = new.chain;
                op.closed = new.closed;
                let old = op.additional_chains.clone();
                op.additional_chains = new.additional_chains;
                for contour in &mut op.additional_chains {
                    if let Some(previous) = old
                        .iter()
                        .find(|p| p.chain == contour.chain && p.closed == contour.closed)
                    {
                        contour.side = previous.side;
                    }
                }
            }
            (Operation::Pocket(op), Operation::Pocket(new)) => {
                op.boundary = new.boundary;
                op.islands = new.islands;
                op.additional_regions = new.additional_regions;
            }
            (Operation::Drill(op), Operation::Drill(_)) => {
                op.points = self.selected_centers(selected)?;
                op.hole_heights.clear();
            }
            (Operation::Face(op), Operation::Face(new)) => op.boundary = new.boundary,
            (Operation::Chamfer(op), Operation::Chamfer(new)) => {
                op.chain = new.chain;
                op.closed = new.closed;
                if !op.closed {
                    op.start = None;
                    op.end = None;
                    op.lead_overlap = 0.;
                }
            }
            (Operation::Bore(op), Operation::Bore(new)) => {
                op.points = self.selected_centers(selected)?;
                op.diameter = new.diameter;
            }
            (Operation::Thread(op), Operation::Thread(new)) => {
                op.points = self.selected_centers(selected)?;
                op.major_dia = new.major_dia;
            }
            (Operation::Engrave(op), Operation::Engrave(new)) => {
                op.chain = new.chain;
                op.closed = new.closed;
                if !op.closed {
                    op.start = None;
                    op.end = None;
                }
            }
            (Operation::Carve(op), Operation::Carve(new)) => {
                op.boundary = new.boundary;
                op.islands = new.islands;
            }
            (Operation::Adaptive(_), Operation::Adaptive(_)) => {
                return Err("Adaptive Clearing requires a solid CAD body".into());
            }
            _ => return Err("Drawing geometry and operation kind do not match".into()),
        }
        Ok(draft)
    }
    fn selected_centers(&self, selected: &[LoopRef]) -> Result<Vec<[f64; 2]>, String> {
        selected
            .iter()
            .map(|reference| {
                self.loop_contour(*reference)
                    .map(centroid)
                    .ok_or("Selected hole no longer exists".into())
            })
            .collect()
    }
}
