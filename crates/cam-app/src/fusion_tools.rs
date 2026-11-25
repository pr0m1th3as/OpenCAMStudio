//! Import Autodesk Fusion tool libraries. Unsupported geometry is reported, never guessed.
use cam_model::{CutDir, EntryFeeds, Tool, ToolKind, ToolPreset};
use serde_json::Value;
use std::io::Read;
use std::path::Path;

const MAX_JSON: u64 = 32 * 1024 * 1024;

#[derive(Clone, Debug)]
pub struct ImportEntry {
    pub label: String,
    pub tool: Option<Tool>,
    pub detail: String,
    pub selected: bool,
}

pub fn read(path: &Path) -> Result<Vec<ImportEntry>, String> {
    let file = std::fs::File::open(path).map_err(|e| e.to_string())?;
    let mut zip = zip::ZipArchive::new(file).map_err(|e| format!("Invalid .tools archive: {e}"))?;
    let entry = zip
        .by_name("tools.json")
        .map_err(|_| "The archive has no tools.json".to_string())?;
    if entry.size() > MAX_JSON {
        return Err("tools.json is too large".into());
    }
    let mut json = String::new();
    entry
        .take(MAX_JSON + 1)
        .read_to_string(&mut json)
        .map_err(|e| e.to_string())?;
    if json.len() as u64 > MAX_JSON {
        return Err("tools.json is too large".into());
    }
    parse(&json)
}

fn parse(json: &str) -> Result<Vec<ImportEntry>, String> {
    let root: Value = serde_json::from_str(json).map_err(|e| format!("Invalid tools.json: {e}"))?;
    let data = root
        .get("data")
        .and_then(Value::as_array)
        .ok_or("tools.json has no tool list")?;
    Ok(data
        .iter()
        .enumerate()
        .map(|(i, value)| {
            let label = string(value, "description")
                .filter(|s| !s.trim().is_empty())
                .unwrap_or_else(|| format!("Tool {}", i + 1));
            match convert(value) {
                Ok((tool, warning)) => ImportEntry {
                    label,
                    tool: Some(tool),
                    detail: warning,
                    selected: true,
                },
                Err(reason) => ImportEntry {
                    label,
                    tool: None,
                    detail: reason,
                    selected: false,
                },
            }
        })
        .collect())
}
fn string(v: &Value, key: &str) -> Option<String> {
    v.get(key)?.as_str().map(str::to_owned)
}
fn number(v: &Value, key: &str) -> Option<f64> {
    v.get(key)?.as_f64().filter(|n| n.is_finite())
}
fn positive(v: &Value, key: &str) -> Option<f64> {
    number(v, key).filter(|n| *n > 0.)
}

fn convert(v: &Value) -> Result<(Tool, String), String> {
    let scale = match v.get("unit").and_then(Value::as_str) {
        Some("millimeters") => 1.,
        Some("inches") => 25.4,
        _ => return Err("Unsupported or missing units".into()),
    };
    let g = v.get("geometry").ok_or("Missing geometry")?;
    let diameter = positive(g, "DC").ok_or("Missing cutting diameter")? * scale;
    let length = positive(g, "OAL").ok_or("Missing overall length")? * scale;
    let flute_length = positive(g, "LCF").ok_or("Missing cutting length")? * scale;
    if flute_length > length {
        return Err("Cutting length exceeds overall length".into());
    }
    let flutes = g
        .get("NOF")
        .and_then(Value::as_u64)
        .and_then(|n| u32::try_from(n).ok())
        .filter(|n| *n > 0)
        .ok_or("Missing flute count")?;
    let kind = match v.get("type").and_then(Value::as_str) {
        Some("flat end mill") => ToolKind::EndMill,
        Some("ball end mill") if flute_length >= diameter / 2. => ToolKind::BallMill,
        Some("bull nose end mill") => {
            let radius = positive(g, "RE").ok_or("Missing corner radius")? * scale;
            if radius >= diameter / 2. || radius > flute_length {
                return Err("Invalid corner radius".into());
            }
            ToolKind::BullNose {
                corner_radius: radius,
            }
        }
        Some(t) => return Err(format!("Unsupported tool type: {t}")),
        None => return Err("Missing tool type".into()),
    };
    let description = string(v, "description").unwrap_or_default();
    let lower = description.to_lowercase();
    let cutting_direction = if lower.contains("compression") {
        CutDir::Compression
    } else if lower.contains("downcut") || lower.contains("down cut") {
        CutDir::Down
    } else if lower.contains("upcut") || lower.contains("up cut") {
        CutDir::Up
    } else if lower.contains("straight") || lower.contains("gerade") {
        CutDir::Straight
    } else {
        CutDir::default()
    };
    let mut tool = Tool {
        description,
        manufacturer: string(v, "vendor").unwrap_or_default(),
        product_id: string(v, "product-id").unwrap_or_default(),
        product_link: string(v, "product-link").unwrap_or_default(),
        diameter,
        length,
        flute_length,
        shank_diameter: positive(g, "SFDM").unwrap_or(0.) * scale,
        flutes,
        cutting_direction,
        kind,
        ..Default::default()
    };
    let mut skipped = 0;
    if let Some(presets) = v.pointer("/start-values/presets").and_then(Value::as_array) {
        for p in presets {
            let id = tool.presets.len() as u64 + 1;
            let opt_feed = |key| positive(p, key).map(|n| n * scale);
            let preset = ToolPreset {
                id,
                name: string(p, "name").unwrap_or_default(),
                material_id: None,
                spindle_rpm: number(p, "n").unwrap_or(0.),
                feed: opt_feed("v_f").unwrap_or(0.),
                plunge_feed: opt_feed("v_f_plunge").unwrap_or(0.),
                entry_feeds: EntryFeeds {
                    ramp: opt_feed("v_f_ramp"),
                    lead_in: opt_feed("v_f_leadIn"),
                    lead_out: opt_feed("v_f_leadOut"),
                },
                stepdown: None,
                overlap: None,
                cooling: Default::default(),
            };
            if preset.validate().is_ok() {
                tool.presets.push(preset);
            } else {
                skipped += 1;
            }
        }
    }
    tool.preset_id_floor = tool.presets.len() as u64;
    let mut warnings = Vec::new();
    if skipped > 0 {
        warnings.push(format!("{skipped} invalid preset(s) skipped"));
    }
    if !lower.contains("compression")
        && !lower.contains("downcut")
        && !lower.contains("down cut")
        && !lower.contains("upcut")
        && !lower.contains("up cut")
        && !lower.contains("straight")
        && !lower.contains("gerade")
    {
        warnings.push("Cut direction unspecified; defaults to down-cut".into());
    }
    let warning = warnings.join("; ");
    Ok((tool, warning))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn imports_compression_direction_even_when_description_mentions_both_cuts() {
        let json = r#"{"data":[{"type":"flat end mill","unit":"millimeters","description":"compression upcut downcut","geometry":{"DC":6,"OAL":50,"LCF":20,"SFDM":6,"NOF":2}}]}"#;
        let entries = parse(json).unwrap();
        assert_eq!(entries[0].tool.as_ref().unwrap().cutting_direction, CutDir::Compression);
        assert!(!entries[0].detail.contains("Cut direction unspecified"));
    }
    #[test]
    fn converts_units_presets_and_reports_unsupported_tools() {
        let json = r#"{"data":[{"type":"flat end mill","unit":"inches","description":"upcut","geometry":{"DC":0.25,"OAL":2,"LCF":1,"SFDM":0.25,"NOF":2},"start-values":{"presets":[{"name":"wood","n":12000,"v_f":10,"v_f_plunge":5}]}},{"type":"unsupported","unit":"millimeters","geometry":{"DC":3,"OAL":30,"LCF":10,"NOF":2}}]}"#;
        let entries = parse(json).unwrap();
        assert_eq!(entries[0].label, "upcut");
        let tool = entries[0].tool.as_ref().unwrap();
        assert_eq!(tool.diameter, 6.35);
        assert_eq!(tool.cutting_direction, CutDir::Up);
        assert_eq!(tool.presets[0].feed, 254.);
        assert!(!entries[1].selected);
        assert!(entries[1].detail.contains("Unsupported"));
    }
    #[test]
    fn reads_zip_and_appends_selected_tools_with_free_numbers() {
        use std::io::Write;
        let path =
            std::env::temp_dir().join(format!("fusion-tools-test-{}.tools", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        let mut zip = zip::ZipWriter::new(file);
        zip.start_file(
            "tools.json",
            zip::write::SimpleFileOptions::default()
                .compression_method(zip::CompressionMethod::Deflated),
        )
        .unwrap();
        zip.write_all(br#"{"data":[{"type":"flat end mill","unit":"millimeters","description":"first","geometry":{"DC":2,"OAL":30,"LCF":8,"NOF":2}},{"type":"flat end mill","unit":"millimeters","description":"second","geometry":{"DC":4,"OAL":40,"LCF":12,"NOF":2}}]}"#).unwrap();
        zip.finish().unwrap();
        let mut entries = read(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        entries[0].selected = false;
        let mut library = crate::tool_library::ToolLibrary::default();
        library.add_default();
        for entry in entries.into_iter().filter(|e| e.selected) {
            library.append_imported_tool(entry.tool.unwrap());
        }
        assert_eq!(library.tools.len(), 2);
        assert_eq!(library.tools[1].description, "second");
        assert_eq!(library.tools[1].number, 2);
    }
    #[test]
    fn rejects_invalid_json_and_geometry() {
        assert!(parse("{}").is_err());
        let entries = parse(r#"{"data":[{"type":"flat end mill","unit":"millimeters","geometry":{"DC":2,"OAL":5,"LCF":6,"NOF":2}}]}"#).unwrap();
        assert!(entries[0].tool.is_none());
    }
}
