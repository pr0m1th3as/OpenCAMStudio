# RELEASING

How to cut an OpenCAMStudio release. The *why* of the numbering scheme is in
`VERSIONING.md`; this file is the mechanical checklist.

## The mechanism, in one sentence

**Pushing a `vX.Y.Z` tag is what publishes.** The `release` workflow
(`.github/workflows/release.yml`) triggers on `tags: v*`, builds all three
platforms (Linux AppImage, macOS `.dmg`, Windows MSI), and runs `gh release create`
with the notes from `docs/release-notes/<tag>.md`. Nothing publishes without a tag;
a `workflow_dispatch` run is a rehearsal that builds `-dev`-suffixed artifacts and
creates no release.

## Checklist

Run from a clean `main` that has the work you intend to ship.

1. **Write the release notes.** Create `docs/release-notes/vX.Y.Z.md`. The workflow
   reads this file verbatim as the GitHub release body; if it is missing the release
   still publishes, but with a generated placeholder body.
2. **Bump the workspace version.** Set `version = "X.Y.Z"` in the root `Cargo.toml`
   `[workspace.package]`. **This is the load-bearing step.** The tag names the
   *artifacts*, but `CARGO_PKG_VERSION` names the *binary* (About box, `--version`);
   skip this and the About box shows a contradictory `0.1.0 (vX.Y.Z, DATE)`. See
   `VERSIONING.md` "The manifest / tag / About-box relationship".
3. **Commit** the notes + bump together, directly to `main`:
   `Release vX.Y.Z` (per project convention, no branch ceremony).
4. **Tag and push.** `git tag vX.Y.Z && git push && git push --tags`. The tag push
   is the trigger — do not tag a commit you have not pushed.
5. **Watch the release job go green.** All three platform builds plus the
   `gh release create` step must succeed. If a build fails, delete the tag
   (`git push --delete origin vX.Y.Z`), fix, and re-tag.
6. **Verify provenance on a real artifact.** Download one build and confirm the About
   box / `--version` reports a **clean** `X.Y.Z` — no `-dirty`, no `-gHASH`, no
   version/tag contradiction. That single check proves the bump in step 2 landed.

## Version-number choice

Feature batch → bump MINOR (`0.1.0 → 0.2.0`). Fix-only release between batches →
bump PATCH (`0.2.0 → 0.2.1`). `1.0.0` is a deliberate statement, not a drift. Full
rationale in `VERSIONING.md`.
