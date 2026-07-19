# Ribbon icon credits

All icons here are **GPL-3.0-only**, matching this project.

## Vendored from OpenCADStudio

The following SVGs are taken unmodified from **OpenCADStudio**
(<https://github.com/HakanSeven12/OpenCADStudio>, © its contributors), which is
licensed **GPL-3.0**. Reused here under the same licence so that the CAD → CAM
workflow presents a familiar ribbon. Copyright and licence notices are preserved by
this file.

- `cui_import.svg` — Open Sample
- `cui_export.svg` — Export .nc
- `box3d.svg` — Show stock
- `viewcube.svg` — Show cube
- `zoom_ext.svg` — Reset view
- `copy.svg` — Duplicate operation
- `erase.svg` — Delete operation / tool

## Original to OpenCAMStudio

Drawn for this project in the 24×24 house style (grey `#e0e0e0` with cyan
`#4cc9f0` accents). © OpenCAMStudio contributors, GPL-3.0.

The area-machining Operations glyphs are drawn as an **isometric stock block**
showing what each operation does to it:

- `profile.svg` — Profile operation (contour toolpath around the part)
- `pocket.svg` — Pocket operation (recess cut into the top)
- `face.svg` — Face operation (machined top face)
- `engrave.svg` — Engrave operation (a V-bit ploughing a V-groove into the top face;
  deliberately distinct from `chamfer.svg`, whose bevel sits on a *corner*)
- `carve.svg` — Carve operation (an *area* carved out of the top face: V-sloped walls
  running down to a cleared flat floor, with the bit standing off the boundary so its
  flank is on the wall. Deliberately distinct from `engrave.svg`, whose tool ploughs a
  single narrow groove with its tip and never leaves a floor)

The hole-making Operations use a **flat tool glyph** instead, so they read at a
glance and stay visually distinct from each other:

- `drill.svg` — Drill operation (a twist drill bit)
- `thread.svg` — Threading operation (a threaded screw/shank)

The remaining originals are in the flat 24×24 style:

- `endmill.svg` — New tool
- `renumber.svg` — bulk-renumber the tool library (a slanted `#` hash)
- `import_library.svg`, `export_library.svg` — tool-library import/export (a
  machinist's tool chest with a down/up arrow; deliberately distinct from the
  project `open.svg`/`save.svg` and the CAD/G-code `cui_import.svg`/`cui_export.svg`)
- `machine.svg` — Machine / post-processor setup (a settings gear)
- `undo.svg`, `redo.svg`, `run.svg` — edit / run actions
- `new.svg`, `open.svg`, `save.svg` — project file actions
