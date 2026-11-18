//! MCP stdio server and the shared, GUI-independent application command catalog.
mod commands;
pub(crate) mod desktop_link;
mod protocol;
mod shop;

use serde::{de::DeserializeOwned, Serialize};
use serde_json::{json, Map, Value};

pub(crate) use commands::call;
pub use desktop_link::connect_desktop;
#[cfg(feature = "gui")]
pub(crate) use commands::is_read_only;
pub use protocol::serve;
pub(crate) use shop::{call_shop, shop_tools, Shop};
pub type CommandResult = Result<Value, String>;

pub fn tools() -> Vec<Value> {
    commands::tools().into_iter().chain(shop_tools()).collect()
}

/// A headless MCP session has the same shop data and project controller as the GUI.
pub struct Session {
    pub controller: crate::AppController,
    pub library: crate::ToolLibrary,
    pub machines: crate::MachineLibrary,
    pub settings: crate::Settings,
}
impl Default for Session {
    fn default() -> Self {
        Self {
            controller: crate::AppController::new(crate::default_machine()),
            library: crate::ToolLibrary::defaults(),
            machines: crate::MachineLibrary::seeded(),
            settings: crate::Settings::default(),
        }
    }
}
impl Session {
    pub fn from_user_settings() -> Self {
        let (settings, _) = crate::load_settings();
        let (legacy, _) = crate::ToolLibrary::load();
        let (collection, outcome) = crate::config::load::<crate::tool_library::LibraryCollection>();
        let library = if matches!(outcome, crate::config::ConfigLoad::Loaded) {
            collection.libraries[&collection.active].clone()
        } else {
            legacy
        };
        let (machines, _) = crate::load_machines();
        let active = machines
            .resolve(settings.session.machine.as_deref())
            .cloned()
            .unwrap_or_else(|| crate::MachineEntry::new(crate::default_machine()));
        let mut controller = crate::AppController::new(active.machine);
        controller.set_post_kind(active.post);
        controller.set_estlcam_options(active.estlcam);
        controller.set_simulation_accuracy(settings.simulation.accuracy);
        Self {
            controller,
            library,
            machines,
            settings,
        }
    }
    pub fn call(&mut self, name: &str, args: Value) -> CommandResult {
        if shop_tools().iter().any(|t| t["name"] == name) {
            call_shop(
                &mut Shop {
                    controller: &mut self.controller,
                    library: &mut self.library,
                    machines: &mut self.machines,
                    settings: &mut self.settings,
                },
                name,
                args,
            )
        } else {
            call(&mut self.controller, name, args)
        }
    }
}

/// One catalog entry owns both its advertised schema and its implementation.
pub(crate) struct Command<C> {
    pub name: &'static str,
    pub description: &'static str,
    pub read_only: bool,
    pub properties: Value,
    pub run: fn(&mut C, &Arguments) -> CommandResult,
}

impl<C> Command<C> {
    pub fn tool(&self) -> Value {
        let mut properties = self.properties.as_object().unwrap().clone();
        let required: Vec<_> = properties
            .iter_mut()
            .filter_map(|(key, schema)| {
                let optional = schema
                    .as_object_mut()
                    .unwrap()
                    .remove("x-optional")
                    .is_some();
                if optional {
                    *schema = json!({"anyOf":[schema.clone(),{"type":"null"}]});
                }
                (!optional).then(|| key.clone())
            })
            .collect();
        json!({"name": self.name, "description": self.description,
            "inputSchema": {"type":"object", "properties": properties,
                "required": required, "additionalProperties": false},
            "annotations": {"readOnlyHint":self.read_only, "openWorldHint":false}})
    }

    pub fn invoke(&self, context: &mut C, value: Value) -> CommandResult {
        let args = value.as_object().ok_or("arguments must be an object")?;
        let schema = self.tool();
        for key in schema["inputSchema"]["required"].as_array().unwrap() {
            if !args.contains_key(key.as_str().unwrap()) {
                return Err(format!("missing argument {key}"));
            }
        }
        for (key, value) in args {
            let property = self
                .properties
                .get(key)
                .ok_or_else(|| format!("unknown argument {key}"))?;
            validate_argument(value, property).map_err(|error| format!("{key}: {error}"))?;
        }
        (self.run)(context, &Arguments(args.clone()))
    }
}

fn validate_argument(value: &Value, schema: &Value) -> Result<(), String> {
    if value.is_null() && schema.get("x-optional").is_some() {
        return Ok(());
    }
    let valid = match schema["type"].as_str() {
        Some("string") => value.is_string(),
        Some("boolean") => value.is_boolean(),
        Some("integer") => value.as_u64().is_some(),
        Some("number") => value.is_number(),
        Some("object") => value.is_object(),
        Some("array") => value.is_array(),
        _ => true,
    };
    if !valid {
        return Err(format!("expected {}", schema["type"]));
    }
    if let Some(options) = schema["enum"].as_array() {
        if !options.contains(value) {
            return Err(format!("expected one of {options:?}"));
        }
    }
    if let Some(values) = value.as_array() {
        if let Some(length) = schema["minItems"].as_u64() {
            if values.len() < length as usize {
                return Err("too few array entries".into());
            }
        }
        if let Some(length) = schema["maxItems"].as_u64() {
            if values.len() > length as usize {
                return Err("too many array entries".into());
            }
        }
        for value in values {
            validate_argument(value, &schema["items"])?;
        }
    }
    Ok(())
}

pub(crate) struct Arguments(Map<String, Value>);
impl Arguments {
    pub fn get<T: DeserializeOwned>(&self, key: &str) -> Result<T, String> {
        serde_json::from_value(
            self.0
                .get(key)
                .cloned()
                .ok_or_else(|| format!("missing argument {key}"))?,
        )
        .map_err(|error| format!("{key}: {error}"))
    }
    pub fn optional<T: DeserializeOwned>(&self, key: &str) -> Result<Option<T>, String> {
        match self.0.get(key) {
            None | Some(Value::Null) => Ok(None),
            _ => self.get(key).map(Some),
        }
    }
    pub fn flag(&self, key: &str) -> Result<bool, String> {
        Ok(self.optional(key)?.unwrap_or(false))
    }
}

pub(crate) fn serialized(value: impl Serialize) -> CommandResult {
    serde_json::to_value(value).map_err(|error| error.to_string())
}
pub(crate) fn text() -> Value {
    json!({"type":"string"})
}
pub(crate) fn id() -> Value {
    json!({"type":"integer", "minimum":0})
}
pub(crate) fn number() -> Value {
    json!({"type":"number"})
}
pub(crate) fn boolean() -> Value {
    json!({"type":"boolean"})
}
pub(crate) fn object(description: &str) -> Value {
    json!({"type":"object", "description":description})
}
pub(crate) fn any(description: &str) -> Value {
    json!({"description":description})
}
pub(crate) fn array(items: Value) -> Value {
    json!({"type":"array", "items":items})
}
pub(crate) fn vector(size: usize) -> Value {
    json!({"type":"array", "items":number(), "minItems":size, "maxItems":size})
}
pub(crate) fn optional(mut schema: Value) -> Value {
    schema["x-optional"] = json!(true);
    schema
}
pub(crate) fn choice(values: &[&str]) -> Value {
    json!({"type":"string", "enum":values})
}
pub(crate) fn kind() -> Value {
    choice(&[
        "Profile", "Pocket", "Drill", "Face", "Chamfer", "Thread", "Bore", "Engrave", "Carve",
        "Adaptive", "Parallel",
    ])
}

/// Merge a field patch into an existing serialized model; reject misspelled fields.
pub(crate) fn patched<T: Serialize + DeserializeOwned>(
    current: &T,
    patch: Value,
) -> Result<T, String> {
    fn merge(target: &mut Value, patch: Value) -> Result<(), String> {
        if let (Some(target), Some(patch)) = (target.as_object_mut(), patch.as_object()) {
            for (key, value) in patch {
                let field = target
                    .get_mut(key)
                    .ok_or_else(|| format!("unknown field {key}"))?;
                merge(field, value.clone())?;
            }
        } else {
            *target = patch;
        }
        Ok(())
    }
    let mut value = serde_json::to_value(current).map_err(|e| e.to_string())?;
    if !patch.is_object() {
        return Err("patch must be an object".into());
    }
    merge(&mut value, patch)?;
    serde_json::from_value(value).map_err(|e| e.to_string())
}
