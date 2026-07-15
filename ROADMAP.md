# ROADMAP

North-star scope and the incremental path to it. Slow-changing; the *current*
step and its state live in `STATUS.md` / `WORKSTATE.md`.

## North star

Industrial CAM (EdgeCAM-class): 2.5D → 3-axis surfacing → 3+2 → 4/5-axis
simultaneous milling; turning; mill-turn; simulation & verification; a library of
posts. This is a multi-year target reached one shippable slice at a time — never
built all at once.

## First vertical slice — 2.5D milling, end-to-end

Pocket / profile / drill / face → tool → simulate → post to real G-code. This
narrow slice exercises **every architectural seam** with the least geometry pain.

## Build order (each phase independently demoable)

| Phase | Goal | Crates | Done when |
|-------|------|--------|-----------|
| **P0** | Skeleton + CI | workspace, `.github/` | `cam-*` workspace compiles; release + Pages workflows produce a stub artifact; a dep-license check enforces GPLv3-compat |
| **P1** | Robust 2D geometry | `cam-geo` | integer-grid offset/boolean/contains on `i_overlay`, arc-flatten, round joins; headless tests (square, square+hole, sharp corners, self-intersection) green |
| **P2** | Trustworthy G-code out | `cam-cldata`, `cam-post`, `cam-model` (`Machine`/`Post` split) | two-tier IR; grbl post + capabilities + machine params; hand-built toolpath → valid grbl, **golden file + semantic check** |
| **P3** | **First light (4a)** | `cam-model`, `cam-toolpath` | document model (heights, schema version, command/undo hooks); `Strategy` trait (pure, cancellable, diagnostics); `ProfileOp`; **in-code rectangle+hole → profile → grbl `.nc`** (golden + semantic + ASCII-plot) |
| **P4** | Real input (4b) | `cam-import` | DXF read (LINE/ARC/CIRCLE/LWPOLYLINE) + contour chaining + hole nesting; `part.dxf` runs the same pipeline; fixture tests |
| **P5** | See it in a window | `cam-render`, `cam-app` | `wgpu` viewport (part/stock/backplot) + `iced` shell + command system/undo; **strategies run off-thread**; load DXF → set op → backplot → export |
| **P6** | Full 2.5D + 2nd controller | `cam-toolpath`, `cam-post` | `PocketOp`/`DrillOp`/`FaceOp`; **Fanuc post** (canned cycles G83/G80, comp G41/G42); arc-refit pass |
| **P7** | Verification | `cam-sim` | heightfield/dexel material removal + gouge / rapid-through-stock detection; visual **and** automated checks |
| **P8** | Open the doors | `cam-plugin-api` | stabilize `Strategy`/`Post` traits into a `cdylib` ABI + loader + registry; first-party stays static, third-party plugins load |

**P0–P4 are fully headless** — a tested, real G-code generator exists before a day
is spent on the GUI. See `ARCHITECTURE.md` "Core design rules" for the invariants
(machine≠post, first-class heights, static-first-party, pure strategies,
diagnostics, undo, determinism) that P2–P5 must honor.

## Publishing

Local `git init` from **P0** (history + safety from commit one; the bookkeeping
git-exclusion assumes a repo exists). **Public debut at P3–P4**, once first light
works end-to-end (in-code or DXF → readable G-code) — a repo that *does something*.
Optionally push to a **private** GitHub repo earlier so CI actually runs, then flip
to public.

## After the slice (unordered candidates)

- Feeds & speeds calculator; tool library persistence/import.
- Rest-material awareness between operations.
- 3-axis: Z-level roughing, waterline/scallop finishing (needs `cam-geo` 3D and
  real `Kernel` use).
- Waveform/trochoidal roughing (constant chip-load).
- More posts; a post-capability test harness.
- Turning slice (new strategy family + lathe post capabilities).
- Plugin ABI hardening + a plugin registry.

## Deferred by decision (with their seams)

| Deferred | Lands as |
|----------|----------|
| Smooth stock render for curved walls | with **3D milling** — gradient-shaded / dexel or `Kernel`-surface stock, replacing the axis-aligned heightfield walls (cosmetic in 2.5D, verification-critical in 3D) |
| 3D surface machining | new `Strategy` + `Kernel` surface eval |
| 4/5-axis | CL-data gains orientation; new strategies + post capability |
| Turning / mill-turn | new strategy family; lathe post capabilities |
| OCCT kernel backend | alternate `Kernel` impl behind the trait |
| Feature recognition | pre-strategy stage over `cam-kernel` B-rep |
