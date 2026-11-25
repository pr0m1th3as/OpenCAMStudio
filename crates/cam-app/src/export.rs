//! Headless controller service.
use super::*;
impl AppController {
    /// Choose whether detected collisions block posting a completed toolpath.
    pub fn set_block_export_on_verification_errors(&mut self, block: bool) {
        if self.block_export_on_verification_errors != block {
            self.block_export_on_verification_errors = block;
            self.nc = None;
        }
    }

    pub(crate) fn verification_collision_severity(&self) -> Severity {
        if self.block_export_on_verification_errors {
            Severity::Error
        } else {
            Severity::Warning
        }
    }

    /// Post the most recent complete run to the selected controller, caching and
    /// returning it. Cancelled, incomplete or failed runs cannot be exported.
    pub fn export_nc(&mut self) -> Result<&str, ExportError> {
        let outcome = self.outcome.as_ref().ok_or(ExportError::NothingToExport)?;
        match outcome.completion {
            JobCompletion::Complete => {}
            JobCompletion::Cancelled => return Err(ExportError::Cancelled),
            JobCompletion::Incomplete => return Err(ExportError::Incomplete),
            JobCompletion::Failed => return Err(ExportError::HasErrors),
        }
        if outcome.has_errors() {
            return Err(ExportError::HasErrors);
        }
        if self.block_export_on_verification_errors {
            type CollisionError = (CollisionKind, fn(usize) -> ExportError);
            let collision_errors: [CollisionError; 3] = [
                (
                    CollisionKind::RapidThroughStock,
                    ExportError::RapidThroughStock,
                ),
                (
                    CollisionKind::NonCuttingThroughStock,
                    ExportError::NonCuttingThroughStock,
                ),
                (CollisionKind::Fixture, ExportError::FixtureCollision),
            ];
            for (kind, error) in collision_errors {
                let count = outcome
                    .collisions
                    .iter()
                    .filter(|hit| hit.kind == kind)
                    .count();
                if count > 0 {
                    return Err(error(count));
                }
            }
        }
        let mut options = self.post_options();
        if self.post_kind == PostKind::Estlcam {
            options.tools = self.setup().tools.clone();
        }
        // Re-reference each group to its own workpiece origin (datum): shift every
        // coordinate by −origin so that group's chosen part point becomes G-code
        // (0,0,0), matched by the operator's touch-off for `G15 H<n>`. Operations under
        // different origins (a reorientation) each subtract their own; with one origin
        // this is the whole job shifted by −origin. Design/sim stay in part space.
        let setup = &self.document.current().setup();
        let program = outcome.program.translated_per_datum(|idx| {
            let o = setup.origin_position(idx);
            [-o[0], -o[1], -o[2]]
        });
        let nc = self.post_kind.post(&program, &self.machine, &options)?;
        self.nc = Some(nc);
        Ok(self.nc.as_deref().unwrap())
    }

    /// Export the current toolpath as G-code to `path`. Runs the same gating as
    /// [`AppController::export_nc`].
    pub fn export_nc_to(&mut self, path: impl AsRef<Path>) -> Result<(), ExportToError> {
        let nc = self.export_nc().map_err(ExportToError::Export)?.to_string();
        std::fs::write(path.as_ref(), nc).map_err(|e| ExportToError::Io(e.to_string()))?;
        Ok(())
    }
}
