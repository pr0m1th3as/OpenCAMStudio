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

/// Make a string safe inside a `(…)` G-code comment.
///
/// Two jobs:
///
/// - **Strip parentheses.** They delimit the comment, so a nested pair would end it
///   early and feed the remainder to the interpreter as code.
/// - **Reduce to printable ASCII.** grbl tolerates UTF-8 in a comment, but Fanuc and
///   Haas controls accept a restricted character set and DNC/serial links are often
///   7-bit — a stray `°` can corrupt the block or drop the line. Characters common in
///   machining text are transliterated so the meaning survives (`°`→`deg`, `⌀`→`dia`,
///   `×`→`x`, `±`→`+/-`, `µ`→`u`); anything else non-ASCII becomes `?`, and control
///   characters are dropped.
pub(crate) fn sanitize(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for ch in text.chars() {
        match ch {
            '(' | ')' => {}
            '°' => out.push_str("deg"),
            '⌀' | 'Ø' | 'ø' => out.push_str("dia"),
            '×' => out.push('x'),
            '±' => out.push_str("+/-"),
            'µ' | 'μ' => out.push('u'),
            '–' | '—' => out.push('-'),
            '\t' => out.push(' '),
            c if c.is_ascii_graphic() || c == ' ' => out.push(c),
            c if c.is_control() => {}
            _ => out.push('?'),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::sanitize;

    #[test]
    fn parentheses_are_stripped_so_a_comment_cannot_end_early() {
        // A nested pair would close the comment and feed the rest to the interpreter.
        assert_eq!(sanitize("in 3 pass(es)"), "in 3 passes");
        assert_eq!(sanitize(")G0 X0("), "G0 X0");
    }

    #[test]
    fn machining_symbols_transliterate_rather_than_vanish() {
        // grbl tolerates UTF-8 in a comment; Fanuc/Haas and 7-bit DNC links do not.
        // The meaning has to survive the trip.
        assert_eq!(sanitize("60° V-bit"), "60deg V-bit");
        assert_eq!(sanitize("⌀6 endmill"), "dia6 endmill");
        assert_eq!(sanitize("M10×1.5"), "M10x1.5");
        assert_eq!(sanitize("±0.02"), "+/-0.02");
        assert_eq!(sanitize("50µm"), "50um");
    }

    #[test]
    fn the_result_is_always_printable_ascii() {
        for s in ["plain", "60° ⌀6 ±0.02 µ ×", "emoji 🙂 here", "tab\there"] {
            let out = sanitize(s);
            assert!(
                out.chars().all(|c| c.is_ascii_graphic() || c == ' '),
                "{s:?} -> {out:?}"
            );
        }
    }

    #[test]
    fn control_characters_are_dropped_not_turned_into_question_marks() {
        // A stray newline would split one comment into a comment plus a bare line.
        assert_eq!(sanitize("a\nb"), "ab");
        assert_eq!(sanitize("a\rb"), "ab");
    }
}
