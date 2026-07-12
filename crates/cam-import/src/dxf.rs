//! A minimal ASCII-DXF reader.
//!
//! DXF is a flat stream of *group pairs*: a numeric **group code** on one line,
//! its **value** on the next. Geometry lives in the `ENTITIES` section. We read
//! only the handful of entity types a 2.5-D CAM job needs — `LINE`, `ARC`,
//! `CIRCLE`, `LWPOLYLINE` — and ignore everything else. Binary DXF and the
//! legacy `POLYLINE`/`VERTEX` form are intentionally out of scope.

/// A raw DXF entity, straight from the file (angles in degrees, as DXF stores
/// them). Higher layers turn these into `cam-geo` geometry.
#[derive(Clone, Debug, PartialEq)]
pub enum Entity {
    /// A straight segment.
    Line { a: (f64, f64), b: (f64, f64) },
    /// A full circle.
    Circle { center: (f64, f64), radius: f64 },
    /// A circular arc, swept counter-clockwise from `start_deg` to `end_deg`.
    Arc {
        center: (f64, f64),
        radius: f64,
        start_deg: f64,
        end_deg: f64,
    },
    /// A lightweight polyline. Each vertex carries a `bulge` (tangent of a
    /// quarter of the arc's included angle) for the segment that *starts* at it;
    /// zero means a straight segment.
    LwPolyline {
        closed: bool,
        /// `(x, y, bulge)` per vertex.
        verts: Vec<(f64, f64, f64)>,
    },
}

/// Tokenise DXF text into `(group_code, value)` pairs. Lines come in twos; a
/// malformed tail (a code with no value) simply ends the stream.
fn pairs(text: &str) -> Vec<(i32, String)> {
    let mut out = Vec::new();
    let mut lines = text.lines();
    while let (Some(code_line), Some(value_line)) = (lines.next(), lines.next()) {
        if let Ok(code) = code_line.trim().parse::<i32>() {
            out.push((code, value_line.trim().to_string()));
        }
    }
    out
}

/// Extract the supported entities from the `ENTITIES` section. Unsupported
/// entity types are reported by name in `skipped` (deduplicated by the caller).
pub fn read_entities(text: &str) -> (Vec<Entity>, Vec<String>) {
    let pairs = pairs(text);
    let mut entities = Vec::new();
    let mut skipped = Vec::new();
    let mut in_entities = false;

    let mut i = 0;
    while i < pairs.len() {
        let (code, value) = &pairs[i];
        if *code != 0 {
            i += 1;
            continue;
        }
        match value.as_str() {
            "SECTION" => {
                // The next pair (group code 2) names the section.
                in_entities = pairs
                    .get(i + 1)
                    .is_some_and(|(c, v)| *c == 2 && v == "ENTITIES");
                i += 1;
            }
            "ENDSEC" => {
                in_entities = false;
                i += 1;
            }
            "EOF" => break,
            ty if in_entities => {
                // Collect this entity's pairs up to the next group-code-0.
                let start = i + 1;
                let mut j = start;
                while j < pairs.len() && pairs[j].0 != 0 {
                    j += 1;
                }
                match parse_entity(ty, &pairs[start..j]) {
                    Some(e) => entities.push(e),
                    None => skipped.push(ty.to_string()),
                }
                i = j;
            }
            _ => i += 1,
        }
    }

    (entities, skipped)
}

/// Parse one entity's group pairs into an [`Entity`], or `None` if the type is
/// unsupported or the data is incomplete.
fn parse_entity(ty: &str, block: &[(i32, String)]) -> Option<Entity> {
    let f = |code: i32| -> Option<f64> {
        block
            .iter()
            .find(|(c, _)| *c == code)
            .and_then(|(_, v)| v.parse::<f64>().ok())
    };
    match ty {
        "LINE" => Some(Entity::Line {
            a: (f(10)?, f(20)?),
            b: (f(11)?, f(21)?),
        }),
        "CIRCLE" => Some(Entity::Circle {
            center: (f(10)?, f(20)?),
            radius: f(40)?,
        }),
        "ARC" => Some(Entity::Arc {
            center: (f(10)?, f(20)?),
            radius: f(40)?,
            start_deg: f(50)?,
            end_deg: f(51)?,
        }),
        "LWPOLYLINE" => Some(parse_lwpolyline(block)),
        _ => None,
    }
}

/// Parse an `LWPOLYLINE`, walking the group pairs in order so that each `42`
/// (bulge) attaches to the vertex it follows.
fn parse_lwpolyline(block: &[(i32, String)]) -> Entity {
    let mut closed = false;
    let mut verts: Vec<(f64, f64, f64)> = Vec::new();
    for (code, value) in block {
        match code {
            70 => {
                if let Ok(flags) = value.parse::<i32>() {
                    closed = flags & 1 != 0;
                }
            }
            10 => {
                if let Ok(x) = value.parse::<f64>() {
                    verts.push((x, 0.0, 0.0));
                }
            }
            20 => {
                if let (Some(last), Ok(y)) = (verts.last_mut(), value.parse::<f64>()) {
                    last.1 = y;
                }
            }
            42 => {
                if let (Some(last), Ok(b)) = (verts.last_mut(), value.parse::<f64>()) {
                    last.2 = b;
                }
            }
            _ => {}
        }
    }
    Entity::LwPolyline { closed, verts }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RECT_AND_CIRCLE: &str = "\
0\nSECTION\n2\nENTITIES\n\
0\nLINE\n10\n0.0\n20\n0.0\n11\n10.0\n21\n0.0\n\
0\nCIRCLE\n10\n5.0\n20\n5.0\n40\n2.0\n\
0\nARC\n10\n1.0\n20\n2.0\n40\n3.0\n50\n0.0\n51\n90.0\n\
0\nENDSEC\n0\nEOF\n";

    #[test]
    fn reads_supported_entities() {
        let (ents, skipped) = read_entities(RECT_AND_CIRCLE);
        assert_eq!(ents.len(), 3);
        assert!(skipped.is_empty());
        assert_eq!(
            ents[0],
            Entity::Line {
                a: (0.0, 0.0),
                b: (10.0, 0.0)
            }
        );
        assert_eq!(
            ents[1],
            Entity::Circle {
                center: (5.0, 5.0),
                radius: 2.0
            }
        );
        assert!(matches!(ents[2], Entity::Arc { radius, .. } if radius == 3.0));
    }

    #[test]
    fn ignores_entities_outside_the_entities_section() {
        // A LINE in the HEADER section must not be read.
        let dxf = "0\nSECTION\n2\nHEADER\n0\nLINE\n10\n0\n20\n0\n11\n1\n21\n1\n0\nENDSEC\n0\nEOF\n";
        let (ents, _) = read_entities(dxf);
        assert!(ents.is_empty());
    }

    #[test]
    fn parses_lwpolyline_with_closed_flag_and_bulge() {
        let dxf = "0\nSECTION\n2\nENTITIES\n\
0\nLWPOLYLINE\n90\n2\n70\n1\n10\n0.0\n20\n0.0\n42\n1.0\n10\n10.0\n20\n0.0\n\
0\nENDSEC\n0\nEOF\n";
        let (ents, _) = read_entities(dxf);
        assert_eq!(
            ents[0],
            Entity::LwPolyline {
                closed: true,
                verts: vec![(0.0, 0.0, 1.0), (10.0, 0.0, 0.0)],
            }
        );
    }

    #[test]
    fn reports_unsupported_entity_types() {
        let dxf = "0\nSECTION\n2\nENTITIES\n0\nSPLINE\n10\n0\n20\n0\n0\nENDSEC\n0\nEOF\n";
        let (ents, skipped) = read_entities(dxf);
        assert!(ents.is_empty());
        assert_eq!(skipped, vec!["SPLINE".to_string()]);
    }
}
