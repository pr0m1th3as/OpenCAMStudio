use cam_app::{default_machine, AppController, JobParams, OcamFile, OpKind, Project, ToolLibrary};
use cam_model::{Material, Operation, Tool, ToolPreset, SCHEMA_VERSION};

fn preset(id: u64, material_id: Option<u64>, feed: f64) -> ToolPreset {
    ToolPreset {
        id,
        name: format!("Preset {id}"),
        material_id,
        spindle_rpm: 6000.,
        feed,
        plunge_feed: 120.,
        stepdown: Some(0.75),
        overlap: Some(0.3),
        cooling: cam_model::Cooling::Mms,
        entry_feeds: Default::default(),
    }
}
fn cutter() -> Tool {
    Tool {
        number: 1,
        diameter: 6.,
        length: 40.,
        flutes: 2,
        presets: vec![preset(1, Some(7), 600.), preset(2, Some(8), 250.)],
        ..Default::default()
    }
}
fn controller() -> AppController {
    let mut app = AppController::new(default_machine());
    app.edit_setup(|s| {
        s.material_id = Some(7);
        s.material = "Aluminum".into();
        s.tools = vec![cutter()];
    });
    app
}

#[test]
fn material_match_uses_ids_and_never_defaults_to_another_material() {
    let mut tool = cutter();
    assert_eq!(tool.preset_for_material(Some(7)).unwrap().id, 1);
    assert_eq!(tool.preset_for_material(Some(8)).unwrap().id, 2);
    assert!(tool.preset_for_material(Some(9)).is_none());
    assert!(tool.preset_for_material(None).is_none());
    tool.presets.insert(0, preset(3, None, 400.));
    assert_eq!(tool.preset_for_material(Some(7)).unwrap().id, 1);
    assert_eq!(tool.preset_for_material(Some(9)).unwrap().id, 3);
    assert_eq!(tool.preset_for_material(None).unwrap().id, 3);
}

#[test]
fn all_operation_kinds_apply_presets_and_allow_independent_values() {
    let app = controller();
    for kind in [
        OpKind::Profile,
        OpKind::Pocket,
        OpKind::Face,
        OpKind::Drill,
        OpKind::Chamfer,
        OpKind::Thread,
        OpKind::Bore,
        OpKind::Engrave,
        OpKind::Carve,
        OpKind::Parallel,
        OpKind::Spiral,
    ] {
        let mut op = app.operation_template(kind, 1);
        assert_eq!(op.tool_preset().unwrap().id, 1, "{kind:?}");
        assert_eq!(op.spindle_rpm(), 6000.);
        assert_eq!(op.cooling(), cam_model::Cooling::Mms);
        match &op {
            Operation::Profile(o) => {
                assert_eq!(o.feed, 600.);
                assert_eq!(o.stepdown, 0.75);
            }
            Operation::Pocket(o) => {
                assert_eq!(o.clear.feed, 600.);
                assert_eq!(o.clear.overlap, 0.3);
            }
            Operation::Face(o) => {
                assert_eq!(o.feed, 600.);
                assert_eq!(o.overlap, 0.3);
            }
            Operation::Drill(o) => {
                assert_eq!(o.feed, 120.);
                assert_eq!(o.peck, Some(0.75));
            }
            Operation::Chamfer(o) => {
                assert_eq!(o.feed, 600.);
                assert_eq!(o.step, 0.);
            }
            Operation::Bore(o) => assert_eq!(o.feed, 600.),
            Operation::Thread(o) => assert_eq!(o.feed, 600.),
            Operation::Engrave(o) => {
                assert_eq!(o.feed, 600.);
                assert_eq!(o.stepdown, 0.75);
            }
            Operation::Carve(o) => assert_eq!(o.feed, 600.),
            Operation::Adaptive(o) => assert_eq!(o.params.feed, 600.),
            Operation::Parallel(o) => assert_eq!(o.params.feed, 600.),
            Operation::Spiral(o) => assert_eq!(o.params.feed, 600.),
        }
        op.apply_tool_preset(&preset(2, Some(8), 250.)).unwrap();
        op.set_spindle_rpm(5000.); // A per-operation override does not mutate the baseline.
        assert_eq!(op.tool_preset().unwrap().spindle_rpm, 6000.);
        let encoded = serde_json::to_string(&op).unwrap();
        let saved: Operation = serde_json::from_str(&encoded).unwrap();
        assert_eq!(saved, op);
        op.clear_tool_preset();
        assert_eq!(op.cooling(), cam_model::Cooling::Mms);
        assert_eq!(op.spindle_rpm(), 5000.);
        assert!(
            op.same_work(&saved),
            "preset provenance does not change machining"
        );
    }
}

#[test]
fn overrides_and_all_available_presets_survive_project_and_library_round_trips() {
    let mut app = controller();
    let mut op = app.operation_template(OpKind::Pocket, 1);
    op.apply_tool_preset(&preset(2, Some(8), 250.)).unwrap();
    if let Operation::Pocket(o) = &mut op {
        o.clear.feed = 333.;
        o.clear.stepdown = 0.4;
    }
    app.add_operation(op);
    let project = Project {
        view: None,
        toolpaths: Default::default(),
        schema_version: SCHEMA_VERSION,
        document: app.document().clone(),
        regions: vec![],
        open_paths: vec![],
        defaults: JobParams::default(),
        source_name: String::new(),
        machine: None,
        post: None,
    };
    let file = OcamFile::Project(project.clone());
    let loaded = OcamFile::from_json(&file.to_json().unwrap()).unwrap();
    assert_eq!(loaded, file);
    let lib = ToolLibrary {
        tools: vec![cutter()],
        materials: vec![
            Material {
                id: 7,
                name: "Aluminum".into(),
                specific_cutting_force: Some(700.),
            },
            Material {
                id: 8,
                name: "Steel".into(),
                specific_cutting_force: None,
            },
        ],
        ..Default::default()
    };
    let file = OcamFile::Library(lib);
    assert_eq!(OcamFile::from_json(&file.to_json().unwrap()).unwrap(), file);
    app.edit_setup(|s| {
        s.material = "Aluminum 6061".into();
    });
    assert_eq!(
        app.operation_template(OpKind::Face, 1)
            .tool_preset()
            .unwrap()
            .id,
        1
    );
}

#[test]
fn invalid_preset_does_not_partially_change_operation_and_ids_are_not_reused() {
    let app = controller();
    let mut operation = app.operation_template(OpKind::Face, 1);
    let before = operation.clone();
    for bad in [f64::NAN, f64::INFINITY, -1., 0.] {
        let mut invalid = preset(3, None, bad);
        assert!(operation.apply_tool_preset(&invalid).is_err());
        assert_eq!(operation, before);
        invalid.feed = 500.;
        invalid.overlap = Some(1.);
        assert!(operation.apply_tool_preset(&invalid).is_err());
    }
    let mut tool = cutter();
    let allocated = tool.next_preset_id();
    tool.presets.clear();
    let json = serde_json::to_string(&tool).unwrap();
    let mut loaded: Tool = serde_json::from_str(&json).unwrap();
    assert!(loaded.next_preset_id() > allocated);
    assert_eq!(loaded.identity(), cutter().identity());
}

#[test]
fn legacy_library_and_operation_load_without_new_data() {
    let mut value = serde_json::to_value(OcamFile::Library(ToolLibrary {
        tools: vec![cutter()],
        ..Default::default()
    }))
    .unwrap();
    value["library_version"] = 1.into();
    let obj = value.as_object_mut().unwrap();
    obj.remove("materials");
    obj.remove("material_id_floor");
    let tool = obj["tools"][0].as_object_mut().unwrap();
    tool.remove("presets");
    tool.remove("preset_id_floor");
    let OcamFile::Library(loaded) = OcamFile::from_json(&value.to_string()).unwrap() else {
        panic!()
    };
    assert!(loaded.tools[0].presets.is_empty());
    assert!(loaded.materials.is_empty());
    assert_eq!(loaded.library_version, cam_app::LIBRARY_VERSION);
    let op = controller().operation_template(OpKind::Face, 1);
    let mut value = serde_json::to_value(op).unwrap();
    value["Face"].as_object_mut().unwrap().remove("tool_preset");
    let loaded: Operation = serde_json::from_value(value).unwrap();
    assert!(loaded.tool_preset().is_none());
}

#[test]
fn catalog_details_round_trip_and_old_tools_load_without_them() {
    let tool = Tool {
        description: "6 mm finishing end mill".into(),
        manufacturer: "Example Tools".into(),
        product_id: "EM-6".into(),
        product_link: "https://example.com/EM-6".into(),
        cutter_material: "Carbide".into(),
        coating: "TiAlN".into(),
        ..cutter()
    };
    let library = ToolLibrary {
        tools: vec![tool.clone()],
        ..Default::default()
    };
    let encoded = serde_json::to_string(&library).unwrap();
    let loaded: ToolLibrary = serde_json::from_str(&encoded).unwrap();
    assert_eq!(loaded.tools[0], tool);
    let mut legacy = serde_json::to_value(&tool).unwrap();
    for field in ["description", "manufacturer", "product_id", "product_link", "cutter_material", "coating"] {
        legacy.as_object_mut().unwrap().remove(field);
    }
    let loaded: Tool = serde_json::from_value(legacy).unwrap();
    assert!(loaded.description.is_empty());
    assert!(loaded.manufacturer.is_empty());
    assert!(loaded.product_id.is_empty());
    assert!(loaded.product_link.is_empty());
    assert!(loaded.cutter_material.is_empty());
    assert!(loaded.coating.is_empty());
    assert_eq!(loaded.presets, tool.presets);
    assert_eq!(loaded.identity(), tool.identity());
}

#[test]
fn older_cutting_data_defaults_to_no_cooling() {
    let app = controller();
    let op = app.operation_template(OpKind::Pocket, 1);
    let mut encoded = serde_json::to_value(&op).unwrap();
    let operation = encoded
        .as_object_mut()
        .unwrap()
        .values_mut()
        .next()
        .unwrap()
        .as_object_mut()
        .unwrap();
    operation.remove("cooling");
    let baseline = operation
        .get_mut("tool_preset")
        .unwrap()
        .as_object_mut()
        .unwrap();
    baseline.remove("cooling");
    let decoded: Operation = serde_json::from_value(encoded).unwrap();
    assert_eq!(decoded.cooling(), cam_model::Cooling::None);
    assert_eq!(
        decoded.tool_preset().unwrap().cooling,
        cam_model::Cooling::None
    );
    let a = app.operation_template(OpKind::Pocket, 1);
    let mut b = a.clone();
    b.set_cooling(cam_model::Cooling::None);
    assert!(
        !a.same_work(&b),
        "different coolant requests must not be treated as duplicate work"
    );
}
