//! Embeds the Windows resource section.
//!
//! Without this the shipped `.exe` has **no `.rsrc` section at all** -- verified by
//! reading the PE headers of a real artifact, which listed only `.text`, `.rdata`,
//! `.data`, `.pdata` and `.reloc`. The consequences are all user-facing: Explorer
//! shows the generic executable icon rather than the application's, and Properties,
//! Task Manager and the installer's Add/Remove entry have no product name, version
//! or copyright to show. An unsigned binary with a blank icon and no publisher is
//! exactly the profile Windows users are taught to distrust, which compounds the
//! SmartScreen warning rather than sitting beside it.
//!
//! **Two gates, and they are not the same question.** A build script is compiled and
//! run on the *host*, and `[target.\'cfg(windows)\'.build-dependencies]` is likewise
//! resolved against the host -- so on a Linux machine `winresource` is not linked at
//! all and the code below must not even be compiled: that is `#[cfg(windows)]`.
//! Whether to *emit* resources is a question about the target, which is
//! `CARGO_CFG_TARGET_OS`. Conflating the two either breaks the Linux build or tries
//! to emit a resource section while cross-compiling away from Windows.

fn main() {
    println!("cargo:rerun-if-changed=assets/opencamstudio.ico");
    emit_git_provenance();
    #[cfg(windows)]
    embed_windows_resources();
}

/// Embed `git describe` output so every build is traceable to an exact commit,
/// surfaced in the About box and `--version`. A clean tagged release reports just
/// the tag (`v0.1.0`); a dev build reports `v0.1.0-47-g748f9ea` (tag +
/// commits-ahead + short hash), with `-dirty` appended when the tree has
/// uncommitted changes. This is what distinguishes an issue filed on a dev build
/// from one on the release -- the semver alone cannot, since the manifest version
/// only changes at release time (see `VERSIONING.md`). Consumed by
/// `cam_app::version_string`.
fn emit_git_provenance() {
    // Rebuild when HEAD moves or the index changes, so the stamp tracks the
    // checkout. Only watch the ref files when they exist -- a source-tarball build
    // has no `.git`, and pointing rerun-if-changed at a missing path would force a
    // rebuild on every invocation.
    for p in ["../../.git/HEAD", "../../.git/index"] {
        if std::path::Path::new(p).exists() {
            println!("cargo:rerun-if-changed={p}");
        }
    }

    // Empty when git is unavailable (a source tarball, or git not installed); the
    // UI then shows the bare `CARGO_PKG_VERSION` with no provenance suffix.
    let describe = run_git(&["describe", "--tags", "--dirty", "--always"]).unwrap_or_default();
    let date = run_git(&["log", "-1", "--format=%cs"]).unwrap_or_default();
    println!("cargo:rustc-env=OCAM_GIT_DESCRIBE={describe}");
    println!("cargo:rustc-env=OCAM_BUILD_DATE={date}");
}

/// Run `git` with `args`, returning trimmed stdout, or `None` if git is missing,
/// this is not a checkout, or the command fails.
fn run_git(args: &[&str]) -> Option<String> {
    let out = std::process::Command::new("git").args(args).output().ok()?;
    if !out.status.success() {
        return None;
    }
    let s = String::from_utf8(out.stdout).ok()?.trim().to_string();
    (!s.is_empty()).then_some(s)
}

#[cfg(windows)]
fn embed_windows_resources() {
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("windows") {
        return; // on Windows, but cross-compiling elsewhere
    }
    let mut res = winresource::WindowsResource::new();
    res.set_icon("assets/opencamstudio.ico");
    // The spaced display name, matching the window title, the .desktop entry and the
    // macOS bundle. Identifiers stay concatenated; this is a string a human reads.
    res.set("ProductName", "Open CAM Studio");
    res.set("FileDescription", "CAM application for CNC toolpath generation");
    res.set("CompanyName", "Andreas Bertsatos");
    res.set(
        "LegalCopyright",
        "Copyright (C) 2026 Andreas Bertsatos. GPL-3.0-only.",
    );
    res.set("OriginalFilename", "opencamstudio.exe");
    if let Err(e) = res.compile() {
        // Do not fail the build over a missing resource compiler; warn loudly instead.
        println!("cargo:warning=windows resources not embedded: {e}");
    }
}
