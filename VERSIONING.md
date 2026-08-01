# VERSIONING

How OpenCAMStudio numbers its releases, and why. Policy, not process — the
step-by-step of cutting a release lives in `RELEASING.md`.

**The version number is a message to end users and to ourselves about *significance*
— nothing more.** OpenCAMStudio is an application, not a library: there is no
downstream `Cargo.toml` depending on us, no public API whose breakage a version
contractually signals. So the number's only job is to encode *how much changed*.

- **MINOR = a feature batch.** `0.1.0 → 0.2.0 → 0.3.0 …` Real new capability (a post
  family, a new operation, a new subsystem) bumps the minor digit. This is our normal
  release cadence.
- **PATCH = a fix-only release** shipped *between* feature batches. `0.2.0 → 0.2.1`
  means "same features, we fixed something" — a user can update without relearning
  anything. Do **not** file feature work under a patch bump; that lies about its weight.
- **App version ≠ file compatibility.** `SCHEMA_VERSION` (in `cam-model`) governs
  "can this build open that file"; it moves independently of the app version. Never
  overload the app version with compat semantics — let each number do one job.
- **Old files are migrated forward, never sideways or back.** Since v10 the version
  drives `cam_model::migrate`, which rewrites a saved document's JSON to the current
  shape *before* it is deserialized. The contract:
  - **Forward only.** A file older than the current schema is brought up to it, one
    step per version, and re-saved at the current version. There is no downgrade.
  - **A newer file is refused, not opened.** Opening it would drop every field this
    build does not understand on the next save, silently deleting the newer version's
    work. The user is told to upgrade.
  - **Every version in `OLDEST_SUPPORTED..SCHEMA_VERSION` has a step**, even when that
    step does nothing (v3–v9 are identity: they were additive bumps that serde's
    defaults already absorbed). Bumping `SCHEMA_VERSION` without adding one is a test
    failure, not a surprise on a user's file. Note what that test can and cannot prove:
    it checks a step *exists*, never that an identity step is *honest*. Only opening a
    real file of that vintage shows the difference — which is how `OLDEST_SUPPORTED`
    came to be 3 rather than 1.
  - **`OLDEST_SUPPORTED` is 3.** v2→v3 replaced `Stock::Box { min, max }` with the
    part-relative `Stock::BoundingBox`, and shipped deliberately without a migration
    ("early stage"). No released build has ever opened a v1 or v2 file. Such a file is
    refused **by version**, with a message saying so, rather than being carried into
    serde to fail on `unknown variant 'Box'` — a parse error blames the file's contents
    for what is really an unsupported version, and the user can act on neither.
  - **The version numbers the save-file as a whole**, not the document alone — v11
    added the machine and post to the project wrapper and bumped even though the
    document did not change. A format change that leaves the version alone is the one
    that bites later.
  - **A change of *meaning* need not bump.** v11 both recorded the machine/post and
    *adopted* them on open; 2026-08-01 they became provenance only — recorded, reported,
    never applied — because a machine is shop-local and its envelope is what gates an
    export. The bytes on disk are unchanged, so the version is too. The rule is that the
    version tracks what a reader must **parse**, not what it chooses to **do** with what
    it parsed; the latter belongs in the code's own documentation and its tests.
  - Retiring old versions (raising `OLDEST_SUPPORTED`) is a **breaking change** for the
    files it drops, and belongs in a MAJOR bump with a release note naming them.
- **The manifest bumps at release time, and git provenance covers the gap.** The
  workspace `version` stays at the last *released* value through a dev cycle and is
  bumped to match the tag in the release commit (so a published artifact never
  mislabels itself). Because that leaves every dev build reporting the old semver,
  builds also embed `git describe --tags --dirty --always` (via `cam-app/build.rs`,
  surfaced by `cam_app::version_string` in the About box and `--version`): a clean
  release shows the bare tag (`0.1.0`), a dev build shows `0.1.0 (v0.1.0-47-g748f9ea,
  DATE)`. **This is how an issue is pinned to an exact build** — the semver alone
  can't, and a `-dev` pre-release suffix only narrows it to a cycle, not a commit, so
  we don't use one. (Revisit only if pre-release builds ever leave Andreas's machine
  and reach a third party.)
- **Staying in `0.x` is itself an honest signal:** young, capabilities still landing,
  breaking changes on the table. **`1.0.0` is a statement, not a milestone we drift
  into** — reserve it for "2.5D milling is trustworthy on real machines," a meaningful
  north-star tag rather than an arbitrary one.
- Pre-1.0, we do **not** agonise over strict SemVer breaking-change rules — SemVer
  explicitly lets anything change while `0.x`, and for an app with no API consumers
  those rules are ceremony. "MINOR = features, PATCH = fixes, 1.0 = we mean it" is the
  whole discipline.

## The manifest / tag / About-box relationship

The CI takes the **artifact** version from the **git tag** (`GITHUB_REF_NAME`, via
`.github/scripts/version.sh`), while the **binary's internal** version — the About
box and `--version` — comes from `CARGO_PKG_VERSION`, i.e. `Cargo.toml`. Those two
must agree at release, and `git describe` makes a mismatch *loud* rather than silent:

- **Forgot the manifest bump.** Tag `v0.2.0`, manifest still `0.1.0`: a clean build
  computes `0.1.0 (v0.2.0, DATE)` — the semver and the tag visibly contradict each
  other in the About box. The build is still shippable, but it screams "you skipped
  the bump."
- **Bumped correctly.** Manifest `0.2.0`, tag `v0.2.0`: `git describe` returns exactly
  `v0.2.0`, which matches the version, so the About box shows a clean `0.2.0`.

That is why bumping `Cargo.toml` before tagging is the load-bearing step in
`RELEASING.md`.
