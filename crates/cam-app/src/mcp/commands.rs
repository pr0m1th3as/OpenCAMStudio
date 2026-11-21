use super::*;
use crate::{AppController, Selection};
use cam_model::{Operation, Tool};

macro_rules! catalog {
    ($($name:ident [$read:literal] $description:literal {$($key:literal: $schema:expr),* $(,)?} |$c:ident, $a:ident| $body:block)*) => {
        fn catalog() -> Vec<Command<AppController>> {
            vec![$(Command { name:stringify!($name), description:$description, read_only:$read,
                properties:json!({$($key:$schema),*}), run:|$c, $a| $body }),*]
        }
    }
}

pub fn tools() -> Vec<Value> {
    catalog().iter().map(Command::tool).collect()
}

/// Invoke the same command implementation used by both server modes.
pub fn call(controller: &mut AppController, name: &str, arguments: Value) -> CommandResult {
    catalog()
        .into_iter()
        .find(|c| c.name == name)
        .ok_or_else(|| format!("unknown command {name}"))?
        .invoke(controller, arguments)
}

#[cfg(feature = "gui")]
pub(crate) fn is_read_only(name: &str) -> bool {
    catalog()
        .iter()
        .find(|c| c.name == name)
        .is_some_and(|c| c.read_only)
}

fn setup(c: &AppController) -> &cam_model::Setup {
    c.inspection_setup()
}
fn operation(c: &mut AppController, id: u32) -> Result<Operation, String> {
    let owner = c
        .setups()
        .iter()
        .find(|s| s.operations.iter().any(|op| op.id() == id))
        .map(|s| s.id)
        .ok_or("operation not found")?;
    c.activate_setup(owner)?;
    c.operation(id)
        .cloned()
        .ok_or_else(|| "operation not found".into())
}
fn origin(c: &AppController, index: u32) -> Result<(), String> {
    if c.origin_indices().contains(&index) {
        Ok(())
    } else {
        Err("origin not found".into())
    }
}
fn stamp(stamp: crate::JobStamp) -> Value {
    json!({"request_id":stamp.request_id, "revision":stamp.revision, "input_fingerprint":stamp.input_fingerprint})
}
fn report(c: &AppController) -> Value {
    let progress = c.run_progress().map(|p| {
        json!({"phase":format!("{:?}",p.phase),
        "completed":p.completed, "total":p.total, "operation":p.operation})
    });
    let outcome = c.outcome().map(|o| json!({"completion":format!("{:?}",o.completion), "diagnostics":o.diagnostics,
        "collisions":o.collisions.iter().map(|hit| json!({"kind":format!("{:?}",hit.kind),
            "severity":c.verification_collision_severity(),
            "at":hit.at,"message":hit.message,"operation":hit.op_id,"pattern":hit.pattern_id,
            "instance_index":hit.instance_index,"step_index":hit.step_index,"fraction":hit.fraction})).collect::<Vec<_>>(),
        "stock_vertices":o.stock_vertices.len(), "stock_triangles":o.stock_indices.len()/3}));
    json!({"pending":c.run_pending(), "progress":progress, "outcome":outcome,
        "toolpaths":c.toolpath_statuses(), "revision":c.revision()})
}
fn loops(c: &AppController) -> Vec<crate::LoopRef> {
    let mut loops = Vec::new();
    for (i, polygon) in c.regions().iter().enumerate() {
        loops.push(crate::LoopRef::outer(i));
        loops.extend((0..polygon.holes().len()).map(|h| crate::LoopRef::hole(i, h)));
    }
    loops.extend((0..c.open_paths().len()).map(crate::LoopRef::open));
    loops
}
fn loop_schema() -> Value {
    object("Drawing loop reference: {region: unsigned index, part: 'Outer' | 'Open' | {Hole: unsigned hole index}}. Discover with get_drawing_geometry.")
}
fn entities() -> Value {
    array(object("Serialized EntityRef from get_geometry_state: asset_id, instance_id, revision, kind, entity_id."))
}
fn pattern_kind() -> Value {
    json!({"oneOf":[
        {"type":"object","properties":{"kind":{"const":"rectangular"},"nx":id(),"ny":id(),"dx":number(),"dy":number()},
            "required":["kind","nx","ny","dx","dy"],"additionalProperties":false},
        {"type":"object","properties":{"kind":{"const":"circular"},"center_xy":vector(2),"count":id(),
            "direction":choice(&["ccw","cw"]),"distribution":{"oneOf":[
                {"type":"object","properties":{"kind":{"const":"full"}},"required":["kind"],"additionalProperties":false},
                {"type":"object","properties":{"kind":{"const":"partial"},"sweep_degrees":number()},"required":["kind","sweep_degrees"],"additionalProperties":false}]}},
            "required":["kind","center_xy","count","direction","distribution"],"additionalProperties":false}
    ]})
}
fn pattern_order() -> Value {
    choice(&["by_operation", "by_instance"])
}

catalog! {
    get_state [true] "Inspect the current project, setup, tools, operations, machine, selection and undo state. Model objects use the .ocam JSON representation." {} |c,_a| {
        Ok(json!({"document":c.document(), "active_setup_id":c.active_setup_id(),
            "selection":c.selection(), "active_origin":c.active_origin(), "machine":c.machine(),
            "post":c.post_kind(), "estlcam":c.estlcam_options(), "defaults":c.defaults(),
            "source_name":c.source_name(), "path":c.current_path(), "revision":c.revision(),
            "can_undo":c.can_undo(), "can_redo":c.can_redo(), "pending_operation":c.pending_op(),
            "run":report(c)}))
    }
    new_project [false] "Start an empty project." {} |c,_a| { c.new_project(); Ok(json!(true)) }
    open_project [false] "Open a .ocam project by path using normal migration and validation." {"path":text()} |c,a| {
        c.open_project(a.get::<String>("path")?).map_err(|e|e.to_string())?; Ok(json!(true))
    }
    save_project [false] "Save the project. Omit path to use its current file." {"path":optional(text())} |c,a| {
        let path = a.optional::<String>("path")?.map(std::path::PathBuf::from)
            .or_else(||c.current_path().map(ToOwned::to_owned)).ok_or("provide a path for the first save")?;
        c.save_project(&path).map_err(|e|e.to_string())?; serialized(path)
    }
    open_dxf [false] "Import DXF source text with the application's drawing importer." {"text":text(),"name":text()} |c,a| {
        serialized(c.open_dxf(&a.get::<String>("text")?,a.get::<String>("name")?).map_err(|e|e.to_string())?)
    }
    import_cad [false] "Import a CAD file by path synchronously. For large files use start_cad_import. STEP/IGES require an OCCT-enabled build." {"path":text()} |c,a| {
        serialized(c.import_cad(a.get::<String>("path")?).map_err(|e|e.to_string())?)
    }
    start_cad_import [false] "Start background CAD import; inspect with poll_cad_import, then commit_cad_preview." {"path":text()} |c,a| {
        serialized(c.start_cad_import(a.get::<String>("path")?.into()))
    }
    start_cad_replacement [false] "Prepare replacement geometry for an existing CAD asset in the background." {"asset_id":id(),"path":text()} |c,a| {
        serialized(c.start_cad_replacement(a.get("asset_id")?,a.get::<String>("path")?.into())?)
    }
    poll_cad_import [false] "Poll CAD import, returning errors or a ready preview. A ready preview still needs commit_cad_preview." {} |c,_a| {
        let completed = c.poll_cad_import().transpose()?;
        Ok(json!({"pending":c.cad_import_pending(),"completed":completed,
            "replacement":c.cad_import_replacement(),"preview":c.cad_import_preview().map(|p|
                json!({"name":p.name,"items":p.items,"source_units":format!("{:?}",p.source_units),"dimensions_mm":p.dimensions_mm,"diagnostics":p.diagnostics}))}))
    }
    cancel_cad_import [false] "Cancel pending CAD import and discard its preview." {} |c,_a| {c.cancel_cad_import(); Ok(json!(true))}
    commit_cad_preview [false] "Commit the prepared CAD import or replacement as a document edit." {} |c,_a| {serialized(c.commit_cad_preview()?)}
    get_geometry_state [true] "Inspect CAD assets, instances, topology and operation bindings." {} |c,_a| {serialized(c.geometry_state())}
    get_drawing_geometry [true] "List all selectable drawing loops, their references and sampled points." {} |c,_a| {
        serialized(loops(c).into_iter().map(|l| json!({"reference":l,"points":c.loop_points(l)})).collect::<Vec<_>>())
    }
    nearest_drawing_feature [true] "Find a drawing feature near a setup XY point within an aperture in mm." {"point":vector(2),"aperture":number()} |c,a| {
        let aperture:f64=a.get("aperture")?;
        if aperture<=0. {return Err("aperture must be positive".into());}
        serialized(c.nearest_loop(a.get("point")?,aperture))
    }
    set_cad_selection [false] "Validate and replace selected CAD entities. Adaptive selection accepts bodies." {"entities":entities(),"adaptive":optional(boolean())} |c,a| {
        let selected = a.get("entities")?;
        if a.flag("adaptive")? {c.try_set_adaptive_cad_selection(selected)?;} else {c.try_set_cad_selection(selected)?;}
        serialized(c.cad_selection())
    }
    align_cad_top_face [false] "Align a planar CAD face to machining XY at Z0." {"entity":object("EntityRef for a planar face")} |c,a| {
        c.align_cad_top_face(a.get("entity")?)?; Ok(json!(true))
    }
    place_cad_instance [false] "Set a finite rigid placement for a CAD instance." {"id":id(),"placement":object("RigidTransform {rotation: 3x3 array, translation: [x,y,z]}")} |c,a| {
        c.place_cad_instance(a.get("id")?,a.get("placement")?)?; Ok(json!(true))
    }
    remove_cad_asset [false] "Remove a CAD asset; linked operations retain unresolved bindings for review." {"id":id()} |c,a| {serialized(c.remove_cad_asset(a.get("id")?))}
    replace_cad_asset [false] "Replace a serialized GeometryAsset, preserving identity and marking links for reselection." {"id":id(),"asset":object("GeometryAsset from get_geometry_state")} |c,a| {
        c.replace_cad_asset(a.get("id")?,a.get("asset")?)?; Ok(json!(true))
    }
    add_cad_asset [false] "Add a serialized GeometryAsset and its GeometryInstance objects." {"asset":object("GeometryAsset"),"instances":array(object("GeometryInstance"))} |c,a| {
        serialized(c.add_cad_asset(a.get("asset")?,a.get("instances")?)?)
    }
    undo [false] "Undo the last document edit." {} |c,_a| {serialized(c.undo())}
    redo [false] "Redo the last undone document edit." {} |c,_a| {serialized(c.redo())}
    select [false] "Select Setup, Origin, Stock, Machine, {Tool: index} or {Operation: id}." {"selection":any("Serialized Selection")} |c,a| {
        let selection: Selection = a.get("selection")?;
        match selection {
            Selection::Operation(id) => {operation(c,id)?;},
            Selection::Tool(index) if index >= setup(c).tools.len() => return Err("tool index not found".into()),
            _=>{}
        }
        c.select(selection); serialized(c.selection())
    }
    activate_setup [false] "Activate an existing setup by its stable ID." {"id":id()} |c,a| {c.activate_setup(a.get("id")?)?; serialized(c.active_setup_id())}
    add_setup [false] "Create and activate a setup using the selected CAD body instances, with no operations." {} |c,_a| {serialized(c.add_setup())}
    duplicate_setup [false] "Duplicate a setup with remapped operation and pattern IDs." {"id":id()} |c,a| {serialized(c.duplicate_setup(a.get("id")?)?)}
    delete_setup [false] "Delete a setup; the project must retain at least one." {"id":id()} |c,a| {c.delete_setup(a.get("id")?)?; Ok(json!(true))}
    reorder_setup [false] "Move a setup before another setup, or append if before is omitted." {"id":id(),"before":optional(id())} |c,a| {
        c.reorder_setup_to(a.get("id")?,a.optional("before")?)?; Ok(json!(true))
    }
    update_setup [false] "Patch active setup fields atomically with normal setup validation. Read get_state first. Use operation/tool commands to change operations and tools." {"patch":object("Partial Setup fields, e.g. {name:'Top', heights:{clearance:10}, stock:...}")} |c,a| {
        let candidate = patched(setup(c),a.get("patch")?)?;
        c.apply_setup_settings(candidate)?; serialized(setup(c))
    }
    frame_from_model [true] "Preview setup frame and origin from a CAD model instance." {"instance_id":id()} |c,a| {serialized(c.pending_frame_from_model(a.get("instance_id")?)?)}
    frame_from_edges [true] "Preview a setup frame from up to three axis references and flip flags." {"references":array(any("EntityRef or null; exactly three entries")),"flips":array(boolean())} |c,a| {
        serialized(c.pending_frame_from_edges(a.get("references")?,a.get("flips")?)?)
    }
    get_setup_bounds [true] "Return active setup model bounds and resolved stock bounds." {} |c,_a| {
        Ok(json!({"model":c.setup_model_bounds(),"stock":c.stock_box()}))
    }
    select_origin [false] "Select an existing work origin index." {"index":id()} |c,a| {let index=a.get("index")?; origin(c,index)?; c.select_origin(index); serialized(index)}
    add_origin [false] "Add and activate an additional work origin." {} |c,_a| {c.add_origin(); serialized(c.active_origin())}
    delete_origin [false] "Delete an additional work origin and move its operations to the base origin." {"index":id()} |c,a| {
        let index=a.get("index")?; origin(c,index)?;
        if index == c.base_origin_index() {return Err("cannot delete the base origin".into());}
        c.delete_origin(index); Ok(json!(true))
    }
    set_origin_position [false] "Set the active origin position in setup coordinates." {"position":vector(3)} |c,a| {
        let position = a.get("position")?; c.edit_active_origin(|o|*o=position); serialized(c.origin_position(c.active_origin()))
    }
    set_origin_index [false] "Rename the active work origin index, updating its operations and suppression." {"index":id()} |c,a| {
        let index: u32=a.get("index")?;
        if index==0 || (c.origin_indices().contains(&index) && c.active_origin()!=index) {return Err("choose a positive unused origin index".into());}
        c.set_active_origin_index(index); serialized(c.active_origin())
    }
    set_origin_disabled [false] "Suppress or enable an existing origin's toolpaths." {"index":id(),"disabled":boolean()} |c,a| {
        let index=a.get("index")?; origin(c,index)?; c.set_origin_disabled(index,a.get("disabled")?); Ok(json!(true))
    }
    set_operation_origin [false] "Assign an operation to an existing origin in its setup." {"id":id(),"index":id()} |c,a| {
        let id=a.get("id")?; operation(c,id)?; let index=a.get("index")?; origin(c,index)?;
        if setup(c).patterns.iter().any(|p|p.members.contains(&id)) {return Err("dissolve the operation pattern before changing a source origin".into());}
        c.set_operation_origin(id,index); Ok(json!(true))
    }
    get_operation_template [true] "Return editable default operation fields for a kind and embedded tool number. Does not create an operation." {"kind":kind(),"tool":id()} |c,a| {
        let tool: u32=a.get("tool")?;
        if !setup(c).tools.iter().any(|t|t.number==tool) {return Err("tool not found".into());}
        serialized(c.operation_template(a.get("kind")?,tool))
    }
    begin_operation [false] "Begin the drawing operation wizard; set its tool and drawing geometry, then confirm_operation." {"kind":kind()} |c,a| {
        if !c.has_geometry() {return Err("import geometry first".into());}
        c.begin_operation(a.get("kind")?); serialized(c.pending_op())
    }
    begin_operation_from_selection [false] "Begin a drawing operation with an already selected, validated geometry field." {"kind":kind(),"loops":array(loop_schema())} |c,a| {
        c.begin_operation_from_selection(a.get("kind")?,&a.get::<Vec<_>>("loops")?)?;
        serialized(c.pending_op())
    }
    reinitialize_operation [false] "Restart an operation's drawing wizard, replacing it in place when confirmed." {"id":id()} |c,a| {
        let id=a.get("id")?; operation(c,id)?; serialized(c.reinitialize_operation(id))
    }
    set_pending_tool [false] "Choose an embedded tool for the pending operation." {"number":id()} |c,a| {
        let number=a.get("number")?;
        if !setup(c).tools.iter().any(|t|t.number==number) {return Err("tool not found".into());}
        if c.pending_op().is_none() {return Err("begin an operation first".into());}
        c.set_pending_tool(number); serialized(c.pending_op())
    }
    clear_pending_tool [false] "Clear the pending operation's tool choice." {} |c,_a| {c.clear_pending_tool(); Ok(json!(true))}
    set_pending_geometry [false] "Set the pending operation's drawing loops and optional start point." {"loops":array(loop_schema()),"start":optional(vector(2))} |c,a| {
        c.set_pending_operation_geometry(&a.get::<Vec<_>>("loops")?,a.optional("start")?)?;
        Ok(json!({"pending":c.pending_op(),"ready":c.pending_ready()}))
    }
    planar_operation_eligibility [true] "Check whether drawing loops support an operation kind." {"kind":kind(),"loops":array(loop_schema())} |c,a| {
        c.planar_operation_eligibility(a.get("kind")?,&a.get::<Vec<_>>("loops")?)?; Ok(json!(true))
    }
    cancel_operation [false] "Discard the pending drawing operation." {} |c,_a| {c.cancel_operation(); Ok(json!(true))}
    confirm_operation [false] "Commit the pending drawing operation, optionally supplying rpm/feed/plunge_feed." {"cutting":optional(object("CuttingData {rpm,feed,plunge_feed}"))} |c,a| {
        if !c.pending_ready() {return Err("choose a tool and eligible geometry first".into());}
        if let Some(data) = a.optional::<crate::CuttingData>("cutting")? {validate_cutting(data)?;}
        if !c.confirm_operation(a.optional("cutting")?) {return Err("operation could not be created".into());}
        serialized(c.selected_operation())
    }
    create_operation [false] "Commit a complete operation draft in one undo step. Get an operation template first. Optional fields match the shared operation editor." {
        "operation":any("Serialized Operation"),"binding":optional(object("GeometryBinding")),
        "height_settings":optional(object("OperationHeightSettings")),"tools":optional(array(object("Tool"))),
        "stock_to_leave":optional(object("StockToLeave"))
    } |c,a| {
        serialized(c.commit_operation_creation(a.get("operation")?,a.optional("binding")?,
            a.optional("height_settings")?.unwrap_or_default(),a.optional("tools")?.unwrap_or_default(),
            a.optional("stock_to_leave")?.unwrap_or_default())?)
    }
    create_cad_operation [false] "Create a linked operation from qualified CAD entities and an embedded tool." {"kind":kind(),"entities":entities(),"tool":id()} |c,a| {
        serialized(c.create_cad_operation_with_tool(a.get("kind")?,a.get("entities")?,a.get("tool")?)?)
    }
    create_cad_operation_from_tool [false] "Embed a cutter and create a linked CAD operation with explicit cutting data atomically." {"kind":kind(),"entities":entities(),"tool":object("Tool"),"cutting":object("CuttingData {rpm,feed,plunge_feed}")} |c,a| {
        serialized(c.create_cad_operation_from_tool(a.get("kind")?,a.get("entities")?,a.get("tool")?,a.get("cutting")?)?)
    }
    cad_operation_eligibility [true] "Check CAD entity eligibility for an operation kind without modifying the project." {"kind":kind(),"entities":entities()} |c,a| {
        c.preview_cad_operation(a.get("kind")?,&a.get::<Vec<_>>("entities")?)?; Ok(json!(true))
    }
    update_operation [false] "Patch an operation and commit all editor fields atomically. Patch the serialized enum object from get_state; identity and operation kind must stay the same." {
        "id":id(),"patch":object("Partial serialized Operation"),"overrides":optional(object("GeometryOverrides")),
        "tools":optional(array(object("Tool"))),"geometry":optional(entities()),
        "height_settings":optional(object("OperationHeightSettings")),"heights":optional(object("Heights")),
        "stock_to_leave":optional(object("StockToLeave"))
    } |c,a| {
        let id=a.get("id")?; let old=operation(c,id)?;
        let updated: Operation=patched(&old,a.get("patch")?)?;
        if updated.id()!=id || std::mem::discriminant(&old)!=std::mem::discriminant(&updated) {return Err("operation identity and kind cannot change".into());}
        let overrides=a.optional("overrides")?.or_else(||c.geometry_state().bindings.get(&id).map(|b|b.overrides.clone()));
        let tools=a.optional("tools")?.unwrap_or_default(); let geometry=a.optional("geometry")?;
        let heights=a.optional::<cam_model::Heights>("heights")?;
        let settings=a.optional::<cam_model::OperationHeightSettings>("height_settings")?;
        if heights.is_some() && settings.is_some() {return Err("choose heights or height_settings".into());}
        let stock=a.optional("stock_to_leave")?.unwrap_or_else(||setup(c).stock_to_leave_for(id));
        if let Some(settings)=settings.or_else(||if heights.is_none(){setup(c).height_settings.get(&id).cloned()}else{None}) {
            c.apply_operation_height_settings_and_stock(updated,overrides,tools,geometry,settings,stock)?;
        } else { c.apply_operation_settings_with_heights_and_stock(updated,overrides,tools,geometry,heights,stock)?; }
        serialized(c.operation(id))
    }
    edit_cad_geometry [false] "Replace a linked operation's entity selection, validating its binding." {"id":id(),"entities":entities()} |c,a| {
        let id=a.get("id")?; operation(c,id)?; c.edit_cad_geometry(id,a.get("entities")?)?; Ok(json!(true))
    }
    edit_drawing_geometry [false] "Replace a drawing operation's selected loops while retaining its cutting parameters and heights." {"id":id(),"loops":array(loop_schema())} |c,a| {
        let id=a.get("id")?; let old=operation(c,id)?;
        if c.geometry_state().bindings.contains_key(&id) {return Err("use edit_cad_geometry for a linked operation".into());}
        let updated=c.operation_with_drawing_geometry(&old,&a.get::<Vec<_>>("loops")?)?;
        c.apply_operation_settings(updated,None)?; serialized(c.operation(id))
    }
    apply_operation_preset [false] "Apply a cutting preset from an operation's embedded tool using normal preset validation." {"id":id(),"preset_id":id()} |c,a| {
        let id=a.get("id")?; let mut updated=operation(c,id)?;
        let preset_id:u64=a.get("preset_id")?;
        let preset=setup(c).tools.iter().find(|t|t.number==updated.tool()).and_then(|t|t.presets.iter().find(|p|p.id==preset_id))
            .ok_or("tool preset not found")?.clone();
        updated.apply_tool_preset(&preset)?;
        let overrides=c.geometry_state().bindings.get(&id).map(|b|b.overrides.clone());
        c.apply_operation_settings(updated,overrides)?; serialized(c.operation(id))
    }
    override_cad_heights [false] "Override linked CAD top/bottom heights; omitted values clear the respective override." {"id":id(),"top":optional(number()),"bottom":optional(number())} |c,a| {
        let id=a.get("id")?; operation(c,id)?; c.override_cad_heights(id,a.optional("top")?,a.optional("bottom")?)?; Ok(json!(true))
    }
    duplicate_operation [false] "Duplicate an operation in its owning setup." {"id":id()} |c,a| {
        let id=a.get("id")?; operation(c,id)?; c.select(Selection::Operation(id)); c.duplicate_selected_operation(); serialized(c.selected_operation())
    }
    set_operation_rotary_index [false] "Set or clear fixed 3+1 indexing. Geometry/heights use the indexed machine frame; CAD selections are rotated before resolution." {"id":id(),"index":optional(object("RotaryIndex {axis: 'A' | 'B', center: [x,y,z], angle: degrees, tolerance: mm}; omit to clear"))} |c,a| {
        let id=a.get("id")?; operation(c,id)?; c.set_operation_rotary_index(id,a.optional("index")?)?; serialized(c.document().rotary_indexes.get(&id))
    }
    delete_operation [false] "Delete an operation and its associated settings/bindings." {"id":id()} |c,a| {
        let id=a.get("id")?; operation(c,id)?; c.select(Selection::Operation(id)); c.delete_selected_operation(); Ok(json!(true))
    }
    set_operation_excluded [false] "Suppress or enable an operation's toolpath." {"id":id(),"excluded":boolean()} |c,a| {
        let id=a.get("id")?; operation(c,id)?; c.set_operation_excluded(id,a.get("excluded")?); Ok(json!(true))
    }
    move_operation [false] "Move an operation earlier/later within its work-origin group." {"id":id(),"up":boolean()} |c,a| {
        let id=a.get("id")?; operation(c,id)?; c.move_operation(id,a.get("up")?); Ok(json!(true))
    }
    reorder_operation [false] "Move an operation before a sibling or append in destination setup. Cross-setup moves validate frames, bindings and tools." {"id":id(),"destination":id(),"before":optional(id())} |c,a| {
        c.reorder_operation_to(a.get("id")?,a.get("destination")?,a.optional("before")?)?; Ok(json!(true))
    }
    transfer_operation [false] "Copy or move an operation to another setup with validated geometry and remapped tools." {"id":id(),"destination":id(),"copy":boolean()} |c,a| {
        serialized(c.transfer_operation_to_setup(a.get("id")?,a.get("destination")?,a.get("copy")?)?)
    }
    duplicate_operation_groups [true] "Find identical included operations in the active setup." {} |c,_a| {serialized(c.duplicate_operation_groups())}
    create_pattern [false] "Group source operations into a validated linear or circular pattern." {"setup_id":id(),"members":array(id()),"name":text(),"kind":pattern_kind(),"order":pattern_order()} |c,a| {
        serialized(c.create_pattern(a.get("setup_id")?,&a.get::<Vec<_>>("members")?,a.get("name")?,a.get("kind")?,a.get("order")?)?)
    }
    edit_pattern [false] "Edit a pattern's parameters and ordered source membership atomically." {"setup_id":id(),"id":id(),"members":array(id()),"name":text(),"kind":pattern_kind(),"order":pattern_order()} |c,a| {
        c.edit_pattern(a.get("setup_id")?,a.get("id")?,&a.get::<Vec<_>>("members")?,a.get("name")?,a.get("kind")?,a.get("order")?)?; Ok(json!(true))
    }
    dissolve_pattern [false] "Remove a pattern, retaining its source operations." {"setup_id":id(),"id":id()} |c,a| {
        pattern_exists(c,a)?; c.dissolve_pattern(a.get("setup_id")?,a.get("id")?); Ok(json!(true))
    }
    delete_pattern [false] "Delete a pattern and all of its source operations in one edit." {"setup_id":id(),"id":id()} |c,a| {
        pattern_exists(c,a)?; c.delete_pattern_and_operations(a.get("setup_id")?,a.get("id")?); Ok(json!(true))
    }
    set_pattern_suppressed [false] "Suppress or enable all instances of a pattern." {"setup_id":id(),"id":id(),"suppressed":boolean()} |c,a| {
        pattern_exists(c,a)?; c.set_pattern_suppressed(a.get("setup_id")?,a.get("id")?,a.get("suppressed")?); Ok(json!(true))
    }
    add_tool [false] "Add a default tool to the active setup." {} |c,_a| {c.add_tool(); serialized(&setup(c).tools)}
    use_tool [false] "Embed a serialized tool, reconciling identity and numbering." {"tool":object("Tool, including presets and geometry")} |c,a| {
        let tool: Tool=a.get("tool")?; validate_tool(&tool)?; serialized(c.use_tool(tool))
    }
    update_tool [false] "Patch an embedded tool by list index, including geometry, cooling and presets." {"index":id(),"patch":object("Partial Tool fields")} |c,a| {
        let index=a.get("index")?; let old=setup(c).tools.get(index).ok_or("tool index not found")?;
        let tool: Tool=patched(old,a.get("patch")?)?; validate_tool(&tool)?;
        if tool.number!=old.number {return Err("tool number cannot change here; use reconcile_tools".into());}
        c.edit_tool(index,|current|*current=tool); serialized(&setup(c).tools[index])
    }
    delete_tool [false] "Delete an unused embedded tool by list index." {"index":id()} |c,a| {
        let index: usize=a.get("index")?; let tool=setup(c).tools.get(index).ok_or("tool index not found")?;
        if setup(c).operations.iter().any(|op|op.tools().contains(&tool.number)) {return Err("tool is in use".into());}
        c.delete_tool(index); Ok(json!(true))
    }
    prune_unused_tools [false] "Remove unused embedded tools in one undoable edit." {} |c,_a| {c.prune_unused_tools(); serialized(&setup(c).tools)}
    reconcile_tools [false] "Reconcile embedded cutter numbers against shop tools, updating operation references." {"tools":array(object("Tool"))} |c,a| {
        let tools: Vec<Tool>=a.get("tools")?; for tool in &tools {validate_tool(tool)?;}
        let result=c.reconcile_tools(&tools); Ok(json!({"report":format!("{result:?}"),"tools":setup(c).tools}))
    }
    set_machine [false] "Set validated active machine limits and travel envelope." {"machine":object("Machine from get_state")} |c,a| {
        let machine: cam_model::Machine=a.get("machine")?; machine.validate().map_err(|e|e.to_string())?;
        c.set_machine(machine); serialized(c.machine())
    }
    update_machine [false] "Patch active machine fields, validating before publication." {"patch":object("Partial Machine fields")} |c,a| {
        let machine: cam_model::Machine=patched(c.machine(),a.get("patch")?)?;
        machine.validate().map_err(|e|e.to_string())?; c.set_machine(machine); serialized(c.machine())
    }
    set_post_kind [false] "Select a controller dialect for export." {"post":choice(&["Grbl","FluidNc","GrblHal","LinuxCnc","Fanuc","Haas","Okuma","Estlcam"])} |c,a| {c.set_post_kind(a.get("post")?); serialized(c.post_kind())}
    set_estlcam_options [false] "Configure Estlcam coolant and MMS wiring." {"options":object("EstlcamOptions from get_state")} |c,a| {c.set_estlcam_options(a.get("options")?); serialized(c.estlcam_options())}
    set_simulation_accuracy [false] "Set simulation accuracy (0..1); the controller clamps to its supported range." {"accuracy":number()} |c,a| {serialized(c.set_simulation_accuracy(a.get("accuracy")?))}
    start_run [false] "Start asynchronous toolpath generation and stock/collision verification. Poll get_run_status; cancel_run remains available." {} |c,_a| {Ok(stamp(c.start_run()))}
    start_toolpath_run [false] "Run the active setup or a single operation, optionally recomputing cache and including predecessor cuts for simulation." {"operation":optional(id()),"recompute":optional(boolean()),"include_predecessors":optional(boolean())} |c,a| {
        if let Some(id)=a.optional::<u32>("operation")? {operation(c,id)?;}
        Ok(stamp(c.start_toolpath_run_with_predecessors(a.optional("operation")?,a.flag("recompute")?,a.flag("include_predecessors")?)?))
    }
    start_pattern_toolpath_run [false] "Run one active-setup pattern; optionally recompute its source toolpaths." {"id":id(),"recompute":optional(boolean())} |c,a| {
        Ok(stamp(c.start_pattern_toolpath_run(a.get("id")?,a.flag("recompute")?)?))
    }
    get_run_status [true] "Publish completed background work and return progress, diagnostics, collisions and per-operation freshness." {} |c,_a| {c.poll_run(); Ok(report(c))}
    cancel_run [false] "Cancel background toolpath/simulation work and discard export eligibility." {} |c,_a| {c.cancel_run(); Ok(report(c))}
    get_program [true] "Return the completed CL-data program for backplotting or analysis." {} |c,_a| {
        c.poll_run(); serialized(&c.outcome().ok_or("run the job first")?.program)
    }
    get_stock_mesh [true] "Return simulated stock mesh positions, normals, triangles and operation ownership." {} |c,_a| {
        c.poll_run(); let o=c.outcome().ok_or("run the job first")?;
        Ok(json!({"vertices":o.stock_vertices.iter().map(|v|json!({"position":v.position,"normal":v.normal})).collect::<Vec<_>>(),
            "indices":o.stock_indices,"machined":o.stock_machined,"operations":o.stock_operations}))
    }
    get_processing_time [true] "Estimate processing time for the completed program, with optional feed override percent (default 100)." {"feed_override":optional(number())} |c,a| {
        c.poll_run(); let o=c.outcome().filter(|o|o.completion==cam_toolpath::JobCompletion::Complete).ok_or("complete a run first")?;
        let time=o.program.processing_time(c.machine().rapid_rate,c.machine().tool_change_seconds,a.optional("feed_override")?.unwrap_or(100.))?;
        Ok(json!({"total_seconds":time.total_seconds(),"cutting_seconds":time.cutting_seconds(),
            "feed_distance_mm":time.feed_distance_mm,"rapid_distance_mm":time.rapid_distance_mm,
            "feed_seconds":time.feed_seconds,"ramp_helix_seconds":time.ramp_helix_seconds,"plunge_seconds":time.plunge_seconds,
            "rapid_seconds":time.rapid_seconds,"dwell_seconds":time.dwell_seconds,
            "tool_changes":time.tool_changes,"tool_change_seconds":time.tool_change_seconds}))
    }
    get_cache_stats [true] "Inspect toolpath fragment cache reuse/computation counts." {} |c,_a| {let s=c.cache_stats(); Ok(json!({"reused":s.reused,"computed":s.computed}))}
    export_nc [false] "Post completed toolpaths to G-code. Optional path writes a file; otherwise returns text. Exact duplicate operations require allow_duplicates. Detected collisions block export according to preferences. Check get_run_status for verification warnings." {"path":optional(text()),"allow_duplicates":optional(boolean())} |c,a| {
        c.poll_run();
        let duplicates=c.duplicate_operation_groups();
        if !duplicates.is_empty() && !a.flag("allow_duplicates")? {
            return Err(format!("Exact duplicate included operations: {duplicates:?}. Exclude redundant copies or set allow_duplicates to confirm intentional repeated cuts."));
        }
        if let Some(path)=a.optional::<String>("path")? {c.export_nc_to(&path).map_err(|e|e.to_string())?; Ok(json!({"path":path}))}
        else {serialized(c.export_nc().map_err(|e|e.to_string())?)}
    }
}

fn pattern_exists(c: &AppController, a: &Arguments) -> Result<(), String> {
    let setup_id: u64 = a.get("setup_id")?;
    let id: u64 = a.get("id")?;
    if c.setups()
        .iter()
        .any(|s| s.id == setup_id && s.patterns.iter().any(|p| p.id == id))
    {
        Ok(())
    } else {
        Err("pattern not found".into())
    }
}
fn validate_cutting(data: crate::CuttingData) -> Result<(), String> {
    if data.rpm < 0. || data.feed <= 0. || data.plunge_feed <= 0. {
        Err("RPM must be nonnegative and feeds positive".into())
    } else {
        Ok(())
    }
}
fn validate_tool(tool: &Tool) -> Result<(), String> {
    tool.validate_presets()?;
    let library = crate::ToolLibrary {
        tools: vec![tool.clone()],
        ..crate::ToolLibrary::defaults()
    };
    library.validate()
}
