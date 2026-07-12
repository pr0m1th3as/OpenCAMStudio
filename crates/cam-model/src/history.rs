//! Undo/redo via snapshots.
//!
//! A core design rule: *all* document mutations go through an edit that the app
//! can stack; the document is never mutated ad hoc. [`History`] is that stack —
//! a small, generic, snapshot-based undo/redo that the GUI drives and that is
//! trivially testable in isolation.
//!
//! Snapshots (a full `clone` of the state per edit) are used rather than
//! inverse-command objects: for a CAM document this is cheap, and it is
//! *impossible* to get an undo wrong, which matters more than a few bytes.

use std::mem;

/// An undo/redo history over a value of type `T`.
///
/// Mutations are made through [`edit`](History::edit); each edit snapshots the
/// prior state onto the undo stack and clears the redo stack. [`undo`] and
/// [`redo`] move between snapshots.
#[derive(Clone, Debug)]
pub struct History<T> {
    past: Vec<T>,
    present: T,
    future: Vec<T>,
}

impl<T: Clone> History<T> {
    /// Start a history at `initial`, with nothing to undo or redo.
    pub fn new(initial: T) -> Self {
        Self {
            past: Vec::new(),
            present: initial,
            future: Vec::new(),
        }
    }

    /// The current state.
    pub fn current(&self) -> &T {
        &self.present
    }

    /// Apply a mutation as a single undoable edit. The state *before* the edit is
    /// pushed onto the undo stack and the redo stack is cleared.
    pub fn edit(&mut self, f: impl FnOnce(&mut T)) {
        self.past.push(self.present.clone());
        f(&mut self.present);
        self.future.clear();
    }

    /// Whether there is a prior state to return to.
    pub fn can_undo(&self) -> bool {
        !self.past.is_empty()
    }

    /// Whether there is an undone state to reapply.
    pub fn can_redo(&self) -> bool {
        !self.future.is_empty()
    }

    /// Step back to the previous state. Returns `false` if there was none.
    pub fn undo(&mut self) -> bool {
        match self.past.pop() {
            Some(prev) => {
                let now = mem::replace(&mut self.present, prev);
                self.future.push(now);
                true
            }
            None => false,
        }
    }

    /// Reapply the most recently undone state. Returns `false` if there was none.
    pub fn redo(&mut self) -> bool {
        match self.future.pop() {
            Some(next) => {
                let now = mem::replace(&mut self.present, next);
                self.past.push(now);
                true
            }
            None => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edit_undo_redo_round_trip() {
        let mut h = History::new(0i32);
        assert!(!h.can_undo() && !h.can_redo());

        h.edit(|v| *v = 1);
        h.edit(|v| *v = 2);
        assert_eq!(*h.current(), 2);
        assert!(h.can_undo() && !h.can_redo());

        assert!(h.undo());
        assert_eq!(*h.current(), 1);
        assert!(h.undo());
        assert_eq!(*h.current(), 0);
        assert!(!h.undo(), "nothing left to undo");

        assert!(h.redo());
        assert_eq!(*h.current(), 1);
        assert!(h.can_redo());
    }

    #[test]
    fn a_new_edit_clears_the_redo_stack() {
        let mut h = History::new(0i32);
        h.edit(|v| *v = 1);
        h.edit(|v| *v = 2);
        h.undo(); // present = 1, future = [2]
        assert!(h.can_redo());

        h.edit(|v| *v = 9); // branches; the redo of 2 is gone
        assert_eq!(*h.current(), 9);
        assert!(!h.can_redo());
        assert!(h.undo() && *h.current() == 1);
    }
}
