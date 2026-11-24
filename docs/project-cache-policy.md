# Project persistence and interactive rotary simulation

Original investigation measurements on 2026-11-09 use the supplied `screw_test.ocam` snapshot
(244,206,320 bytes), containing one wrapped pocket. This is a different snapshot
from the earlier three-operation benchmarks. Implementation measurements below
use a later save of the same project. The playback benchmark uses the
saved simulation accuracy of 0.75 without changing preferences or the input file.

## Implemented project container

New project saves use a ZIP container (archive version 1) with Deflate level 1:

- `project.json`: tagged project JSON, editable CAD/document, view/defaults,
  operation fragments, generated programs, verification fingerprints and reports.
- `stock/<setup-id>/final.bin`: versioned lossless material state, including exact
  grid resolution, f32 height/interval bits, f64 cutter geometry, ownership and
  cut-source/layer references. Numeric arrays use little-endian binary. Small
  metadata uses JSON with exact floating-point round trips.
- `stock/<setup-id>/initial.bin`: initial material state for inherited stock only.
- `stock/<setup-id>/legacy-mesh.json`: transitional lossless compressed final mesh
  for a legacy project that has not been verified again. An inherited initial mesh
  is retained; a defined initial blank is omitted.

The active setup rebuilds its mesh on open, without replaying material removal.
Other verified setups restore when selected. Defined blanks use the existing
viewport reconstruction. Libraries remain tagged JSON. Old tagged and untagged
project JSON still loads and converts on the next save. Older builds cannot read
these new project archives; the document schema and archive version are separate.

Cache CRC/format/reference failures discard that setup's derived verification;
the editable document still opens. Unsupported container versions and damaged
core documents fail to load. The existing input fingerprint still guards reuse
against changes to stock, tools, program, frame, accuracy and verifier revision.
Saves stream to a sibling temporary file, sync it, strictly read back and validate,
then atomically replace the destination. Failed readback preserves the old file.

Release measurements on the latest source file (244,226,520 bytes; view data had
changed since the earlier investigation):

| Work | Result |
| --- | ---: |
| New file after verification at accuracy 0.75 | 484,136 bytes, 99.80% smaller |
| Material-state block before ZIP compression | 8,232,061 bytes |
| Transitional legacy save without re-verifying | 6,920,833 bytes; 0.454 s save; 0.240 s reopen |
| New file decode, migration and stock validation | 0.019 s |
| Controller open including mesh reconstruction and restored verification | 0.064 s |
| Subsequent atomic save including strict readback | 0.030 s |
| Controller reopen including mesh reconstruction | 0.063 s |

The restored program, collision report, final render vertices (including normals
and colors), indices, machined mask, operation ownership and cut-source data
compare exactly with the original JSON project's final result. The save/open
round trip also preserves export eligibility. No stock resampling or precision
reduction is used. Capturing a snapshot occurs only for full job results, not on
every playback update. Completed results retain shared binary snapshots instead
of duplicating display meshes in the saved-verification map.

This implements the persistence change. Playback still has its own traversal and
operation-boundary checkpoints; merely opening the new file does not remove cold
playback seeks or force-analysis work.

## What occupied the legacy file

Approximate compact JSON sizes after parsing:

| Content | Bytes |
| --- | ---: |
| Editable document, including embedded geometry | 124,379 |
| Project without toolpath/verification caches | 125,392 |
| Project with operation cutting cache, without verification | 145,694 |
| Project with cutting cache, generated program and reports, without stock meshes | 1,095,985 |
| Starting-stock display mesh alone | 144,153,024 |
| Final stock vertices and indices | 92,420,673 |
| Final per-vertex machined/operation arrays | 6,507,814 |

The legacy source file is large because it persists triangle meshes and their expanded
per-vertex metadata. The earlier CPU optimization preserved the JSON format. The new stock-state
container removes these arrays for newly verified results.

## Retention policy

| Data | Persistence policy | Reason |
| --- | --- | --- |
| Embedded source CAD/drawing, stable entity IDs, topology and placements | Keep in project | Required for a self-contained editable project; reimport may require an optional CAD backend and may change entity IDs |
| Setups, stock definitions, operations, tools/presets, geometry selections, patterns, origins and machining parameters | Keep in project | User-authored manufacturing intent |
| Defaults, allocation IDs, view state, machine/post provenance | Keep in project | Small; preserves editing and reopening behavior |
| Cutting fragments and generated CL program | Versioned optional cache, normally retain | Small here; other strategies may be expensive to regenerate |
| Verification fingerprint, completion, diagnostics and collisions | Retain with matching generated program | Small; avoid discarding valid verification/export status |
| Starting-stock display mesh for box/round/solid-defined stock | Rebuild on demand | Defined by document; the viewport already reconstructs these blanks |
| Initial stock inherited from a preceding setup | Rebuild from retained initial stock state | Its definition depends on prior machining, not just blank dimensions |
| Final material state: dexel grid and disjoint intervals, ownership and cut-source references | Lossless compressed, versioned stock cache | Expensive to compute; useful for fast remeshing, stock transfer and simulation reuse |
| Final display positions, triangle indices, normals, colors, and repeated per-vertex flags | Rebuild from stock state | Fast and redundant once the actual stock is available |
| Playback checkpoints, removal samples and force trace | RAM/local cache; optionally package selected checkpoints | Improves seeking and force view; derived from the machining inputs |
| GPU buffers, comparison colors and display acceleration structures | Rebuild/cache locally | Display-specific derived data |

The stock cache preserves the existing grid, interval endpoint precision, material
gaps and provenance. Binary arrays use lossless compression; existing verification
fingerprints cover the input program, stock, setup frame, tools, resolution and verifier
revision. Invalid caches can be recomputed without discarding the editable document.

The legacy JSON file **does not contain the resumable dexel state or interior playback
checkpoints**. Removing that mesh without first computing stock state would force material
removal to run again. The roughly 1.1 MB figure therefore describes document/program/report
content, not a complete instant-resume project. The new encoder and restore path
now substitute stock state for the mesh.

## Which reconstruction is actually fast

Release-build measurements through the existing planner and stock helpers:

| Work | Time |
| --- | ---: |
| Open the core editable project | 0.002 s |
| Regenerate toolpath with cutting-fragment cache | 0.001 s |
| Regenerate toolpath without cutting-fragment cache | <0.001 s |
| Construct initial dexel blank | 0.004 s |
| Construct analytic viewport blank, 1,536 vertices | 0.000075 s |
| Construct the large initial stock mesh, 1,190,632 vertices | 0.125 s |
| Compute final remaining stock | 11.297 s |
| Generate its mesh, 771,068 vertices | 0.031 s |
| Rebuild the same mesh with a fresh mesh cache, retaining dexels | 0.033 s |
| Transform mesh vertices to world coordinates | 0.003 s |

The stock grid is 450 × 329, with 0.115556 mm spacing. These are single local
measurements. The distinction is important: **remaining-stock computation is
expensive; stock display-mesh generation from that state is cheap**.
Cached and uncached generation produced identical programs in this test. This
simple pocket is cheap to generate; the result should not be generalized to
expensive 3D or adaptive strategies, whose cutting fragments remain useful caches.

The 144 MB starting mesh is particularly unnecessary for this round blank:
[`uncut_stock_mesh_for_setup`](../crates/cam-app/src/gui_workspace.rs) already
uses the analytic blank for defined stock. Its `initial_stock` result is consumed
for `PreviousSetup`, where the machined input blank genuinely needs upstream data.

## Why playback still feels slow

Measurements of complete-motion prefixes using the same stock cache policy as
playback (percentages refer to step count, not elapsed machining time):

| Request | Stock computation | Meshing |
| --- | ---: | ---: |
| Cold seek to end, ordinary stock coloring | 11.297 s | 0.031 s |
| Rewind to 95% | 10.608 s | 0.029 s |
| Advance from 95% to 96% | 0.216 s | 0.014 s |
| Request the same 96% prefix again | 0.006 s | 0.004 s |
| Cold seek to end with force surface provenance | 11.377 s | 0.032 s |
| Separate full-program removal analysis for forces | 12.098 s | Not applicable |

The rotary cache currently takes checkpoints at operation boundaries. With one
long wrapped operation, a backward seek can lose almost all useful prefix reuse
and replay the cut from its beginning. Playback also owns a different worker/cache
from full verification, so a verified or saved result does not seed its dexel state.

Saved preferences select **Cutting Force** stock coloring and regenerate stock on
rewind. Force coloring starts a separate full-program removal-analysis traversal;
its trace is reused afterward, rather than recalculated every frame. The project
has no material assigned, so those samples cannot yet yield forces in newtons.
The measurement above excludes GUI recoloring and GPU uploads and is not a full
interactive frame-rate measurement.

## Remaining simulation work

1. Share compatible verified stock/checkpoints with playback. Opening simulation
   or seeking to an already computed end should reuse that state.
2. Add bounded checkpoints inside long rotary operations, with copy-on-write
   pages. Rewinds should replay a short suffix rather than the complete operation.
3. Collect optional force-removal samples during the same traversal that produces
   stock, and reuse the trace/provenance. Avoid launching unnecessary duplicate
   traversals; keep force analysis lazy when it is not requested.
4. Measure GUI recoloring and GPU uploads separately, then pass dirty stock tiles
   through to display updates if those become the bottleneck.

Compression of the existing JSON is a useful transitional storage reduction, but
does not create resumable stock or fix expensive backward seeks. Simply dropping
all cached results makes files smaller while increasing recomputation.

## Reproduction

Archive I/O and exact comparison against the legacy reference:

```sh
cargo run --release -p cam-app --example rotary_verify_bench -- /path/screw_test.ocam \
  --simulate --accuracy 0.75 --save /tmp/screw-state.ocam
cargo run --release -p cam-app --example project_io_bench -- /tmp/screw-state.ocam \
  --accuracy 0.75 --compare-stock /path/screw_test.ocam \
  --save-copy /tmp/screw-state-roundtrip.ocam
```

## Rotating-stock display follow-up

The rotating-stock view had a separate CPU bottleneck after stock meshing. It
converted each vertex position and normal into setup coordinates, recalculated
rotation sine/cosine, then converted back to world coordinates. The GUI's dev
build performed those calls for every displayed stock vertex.

The view now creates one `RotaryStockTransform` per published frame, composing
the setup rotation, inverse workpiece rotation and axis-center offset. The bulk
mesh loop runs in the optimized `cam-render` crate. Opaque/transparent meshes,
CAD edges, path lines, cones and drill marks use the same transform; the fixed
tool is still added afterward. Normals use only the rotation, preserving lighting.

In a development-build benchmark with a synthetic 771,068-vertex mesh, matching
this project's final vertex count, the rotation step averaged **1,036.0 ms/frame
before** and **1.295 ms/frame after** over three angles. Vertex copying, material
removal, recoloring and GPU uploads are excluded. This is approximately an 800x
improvement in that display-transform step, not an end-to-end frame-rate claim.

Tests compare positions/normals with the original calculation for A/B rotation,
tilted setup frames, nonzero axis centers, negative angles and multiple turns;
they also check unchanged colors and unit-length normals. Renderer tests and GUI
rotary, indexed playback, force/comparison playback and frame-reuse regressions
pass. The desktop executable was rebuilt with `gui,occt` to retain native CAD
import support. An already running process needs restarting to load the change.

```sh
cargo test -p cam-app --features gui --lib rotating_stock_transform_benchmark -- --ignored --nocapture
```

The stock-computation and backward-seek timings above remain separate work; this
change preserves machining resolution and avoids the extra per-vertex display
trigonometry specific to rotating-stock view.
