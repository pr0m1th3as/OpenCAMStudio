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
    #[cfg(windows)]
    embed_windows_resources();
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
