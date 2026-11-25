//! Shop/library commands, shared by headless and desktop sessions.
use super::*;

pub(crate) struct Shop<'a> {
    pub controller: &'a mut crate::AppController,
    pub library: &'a mut crate::ToolLibrary,
    pub machines: &'a mut crate::MachineLibrary,
    pub settings: &'a mut crate::Settings,
}

macro_rules! shop_catalog {
    ($($name:ident [$read:literal] $description:literal {$($key:literal: $schema:expr),* $(,)?} |$c:ident, $a:ident| $body:block)*) => {
        fn catalog<'a>() -> Vec<Command<Shop<'a>>> {
            vec![$(Command { name:stringify!($name), description:$description, read_only:$read,
                properties:json!({$($key:$schema),*}), run:|$c, $a| $body }),*]
        }
    }
}
pub(crate) fn shop_tools() -> Vec<Value> {
    catalog().iter().map(Command::tool).collect()
}
pub(crate) fn call_shop(shop: &mut Shop<'_>, name: &str, args: Value) -> CommandResult {
    catalog()
        .into_iter()
        .find(|c| c.name == name)
        .ok_or_else(|| format!("unknown shop command {name}"))?
        .invoke(shop, args)
}

shop_catalog! {
    get_tool_library [true] "Inspect shop tools, cutting presets and materials." {} |c,_a| {serialized(&c.library)}
    update_tool_library [false] "Patch shop library fields, including complete tools/materials arrays, after validation. Changes stay in this session until save_shop or export_tool_library." {"patch":object("Partial ToolLibrary fields")} |c,a| {
        let mut candidate:crate::ToolLibrary=patched(c.library,a.get("patch")?)?;
        candidate.library_version=crate::LIBRARY_VERSION; candidate.validate()?; *c.library=candidate; serialized(&c.library)
    }
    import_tool_library [false] "Import a .ocam tool library by path, replacing the session library after validation." {"path":text()} |c,a| {
        match crate::OcamFile::read(a.get::<String>("path")?).map_err(|e|e.to_string())? {
            crate::OcamFile::Library(library)=>*c.library=library,
            _=>return Err("file contains a project, not a tool library".into()),
        }
        serialized(&c.library)
    }
    export_tool_library [false] "Export the session tool library as a validated .ocam library." {"path":text()} |c,a| {
        let text=crate::OcamFile::Library(c.library.clone()).to_json().map_err(|e|e.to_string())?;
        std::fs::write(a.get::<String>("path")?,text).map_err(|e|e.to_string())?; Ok(json!(true))
    }
    add_library_tool [false] "Add a cutter to the shop library with reconciled numbering; omit tool for a default cutter." {"tool":optional(object("Tool"))} |c,a| {
        let mut candidate=c.library.clone();
        let number=if let Some(tool)=a.optional::<cam_model::Tool>("tool")? {candidate.add_tool(tool)}
            else {let index=candidate.add_default(); candidate.tools[index].number};
        candidate.validate()?; *c.library=candidate; serialized(number)
    }
    duplicate_library_tool [false] "Duplicate a shop tool by index, assigning a new library number." {"index":id()} |c,a| {
        let index:usize=a.get("index")?; let mut tool=c.library.tools.get(index).cloned().ok_or("tool index not found")?;
        tool.number=c.library.tools.iter().map(|t|t.number).max().unwrap_or(0).checked_add(1).ok_or("tool number range exhausted")?;
        let number=tool.number; c.library.tools.push(tool); serialized(number)
    }
    update_library_tool [false] "Patch shop cutter geometry, cooling, identity and cutting presets by index." {"index":id(),"patch":object("Partial Tool fields")} |c,a| {
        let index:usize=a.get("index")?; let old=c.library.tools.get(index).ok_or("tool index not found")?;
        let tool:cam_model::Tool=patched(old,a.get("patch")?)?;
        if tool.number!=old.number {return Err("use set_library_tool_number to change numbering".into());}
        let mut candidate=c.library.clone(); candidate.tools[index]=tool; candidate.validate()?;
        *c.library=candidate; serialized(&c.library.tools[index])
    }
    delete_library_tool [false] "Remove a tool from the shop library by index; embedded project tools are preserved." {"index":id()} |c,a| {
        let index:usize=a.get("index")?; if index>=c.library.tools.len(){return Err("tool index not found".into());}
        c.library.tools.remove(index); Ok(json!(true))
    }
    set_library_tool_number [false] "Assign a library cutter number, swapping with a conflicting cutter." {"index":id(),"number":id()} |c,a| {
        let index:usize=a.get("index")?; let number:u32=a.get("number")?;
        if index>=c.library.tools.len() || number==0 {return Err("choose an existing tool and positive number".into());}
        c.library.set_number(index,number); serialized(&c.library.tools)
    }
    renumber_tool_library [false] "Renumber all shop tools using an explicit permutation of their list indices." {"order":array(id())} |c,a| {
        let order:Vec<usize>=a.get("order")?;
        let mut sorted=order.clone(); sorted.sort_unstable();
        if sorted!=(0..c.library.tools.len()).collect::<Vec<_>>() {return Err("order must contain each tool index exactly once".into());}
        c.library.set_numbers_in_order(&order); serialized(&c.library.tools)
    }
    embed_library_tool [false] "Embed a shop cutter by index into the active setup." {"index":id()} |c,a| {
        let index:usize=a.get("index")?; let tool=c.library.tools.get(index).cloned().ok_or("tool index not found")?;
        serialized(c.controller.use_tool(tool))
    }
    promote_project_tool [false] "Add an embedded project tool to the shop library by tool number." {"number":id()} |c,a| {
        let number:u32=a.get("number")?;
        let tool=c.controller.document().setup().tools.iter().find(|t|t.number==number).cloned().ok_or("tool not found")?;
        serialized(c.library.add_tool(tool))
    }
    add_material [false] "Create a named shop material with a stable ID." {"name":text()} |c,a| {
        let name:String=a.get("name")?;
        if name.trim().is_empty() || c.library.materials.iter().any(|m|m.name.eq_ignore_ascii_case(name.trim())) {return Err("enter a unique material name".into());}
        let id=c.library.material_id_floor.max(c.library.materials.iter().map(|m|m.id).max().unwrap_or(0))
            .checked_add(1).ok_or("material ID range exhausted")?;
        c.library.material_id_floor=id; c.library.materials.push(cam_model::Material{id,name,specific_cutting_force:None}); serialized(id)
    }
    rename_material [false] "Rename a shop material by stable ID." {"id":id(),"name":text()} |c,a| {
        let id:u64=a.get("id")?; let name:String=a.get("name")?;
        if name.trim().is_empty() || c.library.materials.iter().any(|m|m.id!=id && m.name.eq_ignore_ascii_case(name.trim())) {return Err("enter a unique material name".into());}
        let material=c.library.materials.iter_mut().find(|m|m.id==id).ok_or("material not found")?;
        material.name=name; serialized(material)
    }
    set_material_cutting_force [false] "Set effective specific cutting force kc in N/mm²; omit specific_cutting_force to clear it. Save with save_shop or export_tool_library." {"id":id(),"specific_cutting_force":optional(number())} |c,a| {
        let id:u64=a.get("id")?;
        let material=c.library.materials.iter().find(|m|m.id==id).ok_or("material not found")?;
        let mut candidate=material.clone(); candidate.specific_cutting_force=a.optional("specific_cutting_force")?;
        candidate.validate()?;
        let material=c.library.materials.iter_mut().find(|m|m.id==id).unwrap();
        *material=candidate; serialized(material)
    }
    get_cutting_forces [true] "Estimate nominal steady milling force, net power and torque for an active-setup operation using its stock material kc. Optional engagement supplies axial_depth_mm, radial_width_mm and cutting_diameter_mm. No run is required; excludes transient peaks and drilling thrust." {"operation":id(),"engagement":optional(object("CuttingEngagement in mm"))} |c,a| {
        let id:u32=a.get("operation")?;
        let setup=c.controller.inspection_setup();
        let op=c.controller.operation(id).ok_or("operation not found in active setup")?;
        if matches!(op,cam_model::Operation::Drill(_)) {return Err("Drilling thrust requires a drill-specific model; milling force is unavailable.".into());}
        let material=setup.material_id.and_then(|id|c.library.materials.iter().find(|m|m.id==id)).ok_or("choose a managed stock material available in the shop library")?;
        let kc=material.specific_cutting_force.ok_or("set the stock material's specific cutting force kc first")?;
        let tool=setup.tools.iter().find(|t|t.number==op.tool()).ok_or("operation tool not found")?;
        let (engagement,basis)=if let Some(engagement)=a.optional::<cam_model::CuttingEngagement>("engagement")? {
            (engagement,"Custom assumed engagement; verify the effective diameter and contact section for this cut.")
        } else {
            let legacy=cam_model::OperationHeightSettings::legacy(c.controller.document(),op);
            let heights=setup.height_settings.get(&id).unwrap_or(&legacy).resolve(c.controller.document(),op)?;
            op.nominal_cutting_engagement(tool,heights.top-heights.bottom)?
        };
        let rpm=if op.spindle_rpm()==0. {c.controller.defaults().spindle_rpm} else {op.spindle_rpm()};
        engagement.validate_for_tool(tool)?;
        let estimate=engagement.estimate(kc,op.cutting_feed_mm_min(),rpm)?;
        Ok(json!({"operation":id,"material":material,"estimate":estimate,"basis":basis,
            "limitations":"Nominal steady cut only; excludes transient peaks, radial/axial forces, deflection and machine losses. Remaining-stock contact is not simulated."}))
    }
    delete_material [false] "Delete an unused shop material. Unlink stock and presets before deleting a material in use." {"id":id()} |c,a| {
        let id:u64=a.get("id")?;
        if !c.library.materials.iter().any(|m|m.id==id){return Err("material not found".into());}
        if c.library.tools.iter().any(|t|t.presets.iter().any(|p|p.material_id==Some(id)))
            || c.controller.setups().iter().any(|s|s.material_id==Some(id)) {
            return Err("material is in use; unlink stock and presets first".into());
        }
        c.library.materials.retain(|m|m.id!=id);
        Ok(json!(true))
    }
    get_machine_library [true] "Inspect available shop machines and their postprocessor/output configuration." {} |c,_a| {serialized(&c.machines)}
    add_machine [false] "Add a validated shop machine entry. Returns its unique library name." {"entry":object("MachineEntry {machine,post,estlcam}")} |c,a| {
        let entry:crate::MachineEntry=a.get("entry")?; entry.machine.validate().map_err(|e|e.to_string())?;
        serialized(c.machines.add(entry))
    }
    update_library_machine [false] "Patch a shop machine entry by name, validating all machine limits." {"name":text(),"patch":object("Partial MachineEntry fields")} |c,a| {
        let name:String=a.get("name")?; let old=c.machines.by_name(&name).ok_or("machine not found")?;
        let entry:crate::MachineEntry=patched(old,a.get("patch")?)?;
        entry.machine.validate().map_err(|e|e.to_string())?;
        let active=c.controller.machine().name==name;
        if active {c.controller.set_machine(entry.machine.clone());c.controller.set_post_kind(entry.post);c.controller.set_estlcam_options(entry.estlcam);}
        let name=c.machines.replace(&name,entry);
        if active {c.settings.session.machine=Some(name.clone());}
        serialized(name)
    }
    delete_machine [false] "Remove a shop machine by name. The last machine cannot be removed." {"name":text()} |c,a| {
        let name:String=a.get("name")?;
        if !c.machines.remove(&name){return Err("machine not found or is the last machine".into());}
        if c.controller.machine().name==name {
            let entry=&c.machines.machines[0];
            c.controller.set_machine(entry.machine.clone());c.controller.set_post_kind(entry.post);c.controller.set_estlcam_options(entry.estlcam);
            c.settings.session.machine=Some(entry.name().into());
        }
        Ok(json!(true))
    }
    activate_machine [false] "Select a shop machine and adopt its postprocessor and output wiring." {"name":text()} |c,a| {
        let name:String=a.get("name")?; let entry=c.machines.by_name(&name).ok_or("machine not found")?;
        c.controller.set_machine(entry.machine.clone()); c.controller.set_post_kind(entry.post);
        c.controller.set_estlcam_options(entry.estlcam); c.settings.session.machine=Some(name);
        serialized(c.controller.machine())
    }
    get_preferences [true] "Inspect viewport, snapping, simulation, pane and session preferences." {} |c,_a| {serialized(&c.settings)}
    update_preferences [false] "Patch preferences and clamp numeric values to supported ranges." {"patch":object("Partial Settings fields")} |c,a| {
        let mut settings:crate::Settings=patched(c.settings,a.get("patch")?)?;
        if settings.mcp_enabled != c.settings.mcp_enabled {
            return Err("Change MCP access in the desktop Preferences.".into());
        }
        settings.settings_version=crate::SETTINGS_VERSION; settings.clamp_all();
        c.controller.set_simulation_accuracy(settings.simulation.accuracy);
        c.controller.set_block_export_on_verification_errors(settings.block_export_on_verification_errors);
        *c.settings=settings;
        serialized(&c.settings)
    }
    restore_preferences [false] "Restore default preferences in this session." {} |c,_a| {
        *c.settings=crate::Settings{mcp_enabled:c.settings.mcp_enabled,..Default::default()};
        c.controller.set_simulation_accuracy(c.settings.simulation.accuracy);
        c.controller.set_block_export_on_verification_errors(c.settings.block_export_on_verification_errors);
        serialized(&c.settings)
    }
    save_shop [false] "Persist the current tool library, machine library and preferences to their normal per-user files." {} |c,_a| {
        // Use fallible writes so an MCP caller never receives success for an I/O failure.
        let settings=crate::settings_path().ok_or("settings directory unavailable")?;
        let directory=settings.parent().ok_or("settings directory unavailable")?;
        std::fs::create_dir_all(directory).map_err(|e|e.to_string())?;
        let path=directory.join("tool-libraries.json");
        let (mut collection, outcome)=crate::config::load_from::<crate::tool_library::LibraryCollection>(&path);
        if let crate::config::ConfigLoad::Rejected(why)=outcome { return Err(format!("Cannot save tool libraries: {why}")); }
        if !matches!(outcome, crate::config::ConfigLoad::Loaded) {
            collection=crate::tool_library::LibraryCollection::from_legacy(c.library.clone());
        }
        collection.libraries.insert(collection.active.clone(), c.library.clone());
        crate::config::save_to(&collection,&path).map_err(|e|e.to_string())?;
        c.machines.save_to(&directory.join("machines.json")).map_err(|e|e.to_string())?;
        c.settings.save_to(&settings).map_err(|e|e.to_string())?;
        Ok(json!(true))
    }
}
