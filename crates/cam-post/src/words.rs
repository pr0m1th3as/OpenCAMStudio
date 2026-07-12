//! Number/word formatting shared by the posts.

/// Format a coordinate with a fixed number of decimals, normalising `-0.000` to
/// `0.000` so output is sign-stable.
pub(crate) fn num(v: f64, precision: usize) -> String {
    let s = format!("{v:.precision$}");
    if s.starts_with('-') && s[1..].bytes().all(|b| b == b'0' || b == b'.') {
        s[1..].to_string()
    } else {
        s
    }
}

/// Format a value with up to `maxdec` decimals but no trailing zeros — for feeds
/// and speeds, where `F300` reads better than `F300.000`. Only the fractional
/// part is trimmed, so integers keep their zeros (`S1000`, not `S1`).
pub(crate) fn compact(v: f64, maxdec: usize) -> String {
    let s = format!("{v:.maxdec$}");
    let s = if s.contains('.') {
        s.trim_end_matches('0').trim_end_matches('.')
    } else {
        &s
    };
    if s.is_empty() || s == "-0" {
        "0".to_string()
    } else {
        s.to_string()
    }
}

/// Make a string safe inside a `(…)` comment by stripping parentheses.
pub(crate) fn sanitize(text: &str) -> String {
    text.replace(['(', ')'], "")
}
