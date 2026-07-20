# ARCHITECTURE

Stable design reference for OpenCAMStudio. Changes here are deliberate design
decisions, logged in `WORKSTATE.md` → archived to `HISTORY.md`.

## Guiding principle

A **small, stable core**; capability lives in crates that will graduate to
plugins. The two things that vary most across shops — **cutting strategies** and
**post-processors** — are the primary plugin kinds. Machines and tools are data.

## The pipeline

```
CAD geometry ─▶ Setup/WCS ─▶ Stock ─▶ Operation ─▶ Toolpath (CL-data) ─▶ Sim/verify ─▶ Post ─▶ G-code
                              │          ▲                                                ▲
                          raw block   STRATEGY (plugin kind)                       POST-PROCESSOR (plugin kind)
```

## The hourglass

Everything narrows to one controller-neutral **CL-data** (cutter-location) IR:

```
   many strategies  ─┐
   (pocket, profile, ├─▶  CL-data IR  ─┬─▶  many posts (grbl, fluidnc, fanuc, haas)
    drill, face …)  ─┘  (neutral moves) └─▶  simulation / backplot
```

Add a strategy → no post changes. Add a machine → no strategy changes. The IR is
the contract between the two plugin kinds.

## CL-data IR — two-tier (`cam-cldata`)

Canonical units **mm, absolute, in the part/WCS frame**. Posts apply the work
offset (G54…), choose G90/G91, and choose output units. The IR never thinks about
any of that.

**Tier 1 — primitive moves:** `Rapid` (G0), `Linear` (G1), `Arc` (G2/G3 —
first-class, *not* linearized; helical when Z changes; center stored as an
absolute point, post emits I/J), `Dwell` (G4), `Spindle{rpm,dir}` / `SpindleOff`,
`Coolant{flood|mist|off}`, `ToolChange{tool}`, `Comment`.

**Tier 2 — high-level cycle intents:** e.g.
`Drill{ points, z_top, depth, retract, peck?, dwell? }`; profiling carries
`side + comp(computed | G41/G42)`. Each **post lowers a cycle per its
capabilities** — Fanuc emits `G83…/G80`, grbl (no canned cycles) expands the same
intent to explicit G0/G1 pecks. This is the capabilities model doing real work.

**Rules:** every move carries a light **tag** (operation-id + kind:
lead-in/cutting/link/retract) — costs nothing, drives backplot coloring and
correctness checks like "no rapid passes through stock".

## Crate layout (Cargo workspace)

| Crate | Responsibility | Depends on kernel? |
|-------|----------------|--------------------|
| `cam-geo` | 2D toolpath geometry: polygons, **robust offset**, boolean, Z-slicing. On `geo`/`i_overlay`. The CAM heart. Also the **tool revolve generatrix** (`Profile2D`/`ProfileSeg`, line/arc segments tagged cutting/non-cutting; `BottomShape`), the derived cutting-surface properties the operation guards read, and V-groove geometry. | **No** |
| `cam-import` | DXF/DWG read (via `acadrust`) → contour chaining + hole nesting → `cam-geo` regions, **plus the open chains that would not close** (engravable strokes). Also exposes a raw entity-inventory read path (for the deferred custom-tool importer). | No |
| `cam-kernel` | `Kernel` trait + a `truck`-backed impl. Import (STEP/mesh), B-rep hold, booleans for stock. Swappable → OCCT (C++ FFI) later. | is the kernel |
| `cam-model` | Document model: `Project → Setup → Stock → Operation → Tool`. Serde. The save-file format. | No (holds refs) |
| `cam-toolpath` | `Strategy` trait + 2.5D strategies. Consumes `cam-geo`, emits CL-data. | No |
| `cam-cldata` | The CL-data IR: rapid/feed/arc/dwell/toolchange, neutral units. | No |
| `cam-post` | `Post` trait + a controller **capabilities** model. Posts lower CL-data → G-code text. | No |
| `cam-sim` | Backplot + material-removal sim (heightfield for 2.5D), **profile-aware in both directions**: removal *and* collision use the tool's own shape, so a pointed tool is only as wide as its cone has opened at that depth. Renders via `wgpu`. | mesh only |
| `cam-render` | `wgpu` viewport: part, stock, toolpath overlay. | mesh only |
| `cam-plugin-api` | Stable ABI crate; strategies + posts compile as `cdylib` against it. | No |
| `cam-app` | `iced` GUI shell + command system. Wires it together. | via traits |

## Kernel strategy

No general CAD kernel provides production CAM offsetting — every CAM system
implements its own toolpath geometry. So:

- **Toolpath geometry is ours** (`cam-geo`), kernel-independent, on the mature
  pure-Rust `geo`/`i_overlay` stack. This is where the CAM risk lives, isolated.
- **The solid kernel is `truck` to start** (pure Rust, same as OpenCADStudio),
  behind the `Kernel` trait, **swappable for an OpenCASCADE (OCCT) C++ backend**
  if `truck`'s 3D robustness proves insufficient. Pure-Rust now; C++ escape hatch
  later.
- The 2.5D slice barely touches the kernel: import → 2D loops + heightfield stock
  → our offsetting → simulate → post.

## Data model (2.5D scope)

```
Document { schema_version, Setup }   (Machine is app state, not saved)
 └─ Setup            { heights, stock, tools, operations, origin, start_offset }
     ├─ Origin       { part XY/Z datum → G-code (0,0,0); post applies −origin }
     ├─ Stock        { part bbox + per-axis XY offsets, top + thickness }
     ├─ Heights      { clearance, retract, top-of-stock, Z0 convention }
     ├─ ToolLibrary  [ Tool { dia, flute/shank/neck length+⌀, flutes, cutting_dir,
     │                        kind + per-kind params → per-kind revolve Profile2D } ]
     └─ Operation[]  (ordered; each carries its own depth + feeds)
         ├─ FaceOp    { tool, stepdown, stepover, boundary }
         ├─ PocketOp  { tool, boundary loops, islands, depth, stepover, plunge }
         ├─ ProfileOp { tool, chain, side, comp: computed|G41/G42, leads, plunge }
         ├─ DrillOp   { tool, points, depth, peck?, dwell? }
         ├─ ChamferOp { tool (chamfer mill or V-bit), chain, side, width }
         ├─ ThreadOp  { tool (thread mill), points, major_dia, pitch, hand,
         │              passes, spring_passes, blind-hole allowance }
         ├─ EngraveOp { tool (V-bit), chain, closed?, top, depth, stepdown }
         │              no side, no radius comp — the tool centre follows the path
         └─ CarveOp   { tool (V-bit), boundary + islands, top, depth (a CAP),
                        offset, ring_step (wall roughing), scallop (floor finish),
                        stay_down, clear: Option<{ tool, ClearParams }> }
                        the boundary outlines an AREA; depth follows from its width

ClearParams — the parameters of an area-clearing pass, shared so a carve's
clearing pass is a pocket over a derived region rather than a copy of one:
{ stepdown, overlap, offset, feeds, plunge, leads, lead_overlap, Clearing }.
(`PocketOp` still carries these flat; adopting the struct there is a schema
change with saved projects to migrate.)

Machine (distinct from Post): { rapid rate, max spindle, feed limits,
                                work envelope, tool-change pos, safe-Z rule }
```

## Core design rules

- **Machine ≠ Post.** Post = dialect/formatter; **Machine** = physical
  limits/params (both in `cam-model`). The post queries the machine, so one post
  drives many machines.
- **Heights are first-class.** Clearance / retract / top-of-stock / depth live on
  `Setup` + `Operation`. Unsafe Z is a primary hazard — never implicit.
- **First-party strategies & posts are static; the plugin ABI is for third
  parties.** The WASM/web build cannot load `cdylib`s, so built-in capability must
  compile in. Design `Strategy`/`Post` traits ABI-friendly *now* (plain data
  across the seam) so the P8 cdylib extraction is cheap.
- **Strategies are pure, off-thread, cancellable.** `Strategy::compute` is a pure
  fn (inputs → toolpath + diagnostics); `cam-app` runs it as a cancellable
  background task, so the UI never freezes and strategies are trivially testable.
- **Diagnostics, not panics.** Strategies return typed warnings/errors with
  geometry references ("tool too big for pocket", "open contour", "would gouge"),
  surfaced in the UI. Each diagnostic names the operation it came from, so the UI
  can mark *which* one failed.
- **Tools are used per their defined cutting surfaces, and the check is derived,
  not enumerated.** Every tool has a revolve generatrix whose segments are tagged
  cutting or non-cutting; the operation guards ask *that* — does the tool have a
  cylindrical cutting flank, a cutting tip, a flat cutting bottom, and does the cut
  stay on the cutting edge — rather than matching on `ToolKind`. An imported custom
  tool is therefore guarded on identical terms to a built-in, with no new code. The
  split is **possibility vs preference**: cutting with a *non-cutting* surface, or
  running past the cutting edge onto neck/shank, is an **error** (it cannot work);
  a tool that cuts but leaves a worse result is a **warning** (the machinist may
  have a reason). Refusing G-code that physically cannot cut is not overriding the
  operator's judgement. A few rules genuinely are not in the geometry — a twist
  drill's flute side *is* tagged cutting yet must never side-mill — and those are
  stated explicitly, with the reason.
- **An operation may use more than one tool, and the operation orders them.**
  `Operation::tools()` lists every tool an operation uses **in cutting order**, so
  `tools()[0]` is what must be in the spindle when its fragment begins. `build_job`
  emits the change for that one, then **resyncs from the last `ToolChange` in the
  appended fragment** — because a multi-tool strategy emits its own changes, being
  the only thing that knows the order. Anything that rewrites tool numbers must go
  through `Operation::map_tools`, which touches *every* reference: renaming only
  the defining tool leaves the second pointing at whatever inherited the old
  number. The one such operation today is **Carve** (clearing end mill, then
  V-bit); the seam exists so it is not the last.
- **A carve's clearing region is derived, which is why it is one operation.** The
  flat land the depth cap leaves is computed from the carve's boundary, depth,
  hold-off and V-bit. Split into a separate pocket, it would go **silently** stale
  the moment the carve is edited — no marker, no error, a floor at the wrong Z.
  One operation recomputes both from one set of inputs on every run.
- **Undo/redo via commands.** All document mutations go through commands `cam-app`
  can stack; `cam-model` is never mutated ad hoc.
- **Determinism.** Same input → same G-code (integer geometry helps) — required
  for golden-file testing and user trust.

## Post-processor model

A post is **capabilities + a formatter**, not a monolith. CL-data is neutral;
each post declares what its controller supports and lowers accordingly. (A
**Machine** — physical limits — is a separate object the post queries; see Core
design rules.)

Six posts ship, in three output families (the dialect name is never emitted, so
members of a family are byte-identical for milling by design — routing between
*families* is what carries meaning):

| Controller | Family | Notable capabilities |
|-----------|--------|----------------------|
| grbl | grbl | G0/G1/G2/G3 only; **no canned cycles** (expand pecking to explicit moves); limited comp |
| FluidNC | grbl | grbl superset |
| grblHAL | grbl | grbl superset |
| LinuxCNC | LinuxCNC | canned cycles; work offsets; RS274NGC style |
| Fanuc | Fanuc | canned cycles (G81/G83), cutter comp (G41/G42), work offsets (G54–G59), tool changes |
| Haas | Fanuc | Fanuc-like dialect |

## Explicitly out of scope (for now)

3D surface machining, 4/5-axis, turning, mill-turn, feature recognition. All have
a designated seam (a new `Strategy`, a new `Kernel` capability, a new post
capability) so they land as additions, not rewrites. See `ROADMAP.md`.
