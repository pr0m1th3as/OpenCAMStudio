# MCP automation

OpenCAMStudio exposes its application commands to MCP clients over stdio. The
same controller implements GUI edits and MCP edits, including undo/redo, linked
CAD validation, toolpath invalidation, collision verification and export gates.
`tools/list` discovers the command names, argument schemas and read-only hints.

The `Parallel` operation kind creates [3D Parallel finishing](parallel.md) over
CAD bodies or faces using a ball nose cutter. Its `params` include `stepover`,
`angle`, `tolerance`, `both_ways`, `containment`, `additional_offset`, `feed`, and
`plunge_feed`. Body/face selection uses the same multi-reference CAD selection
contract as Adaptive.

For [stock from the previous setup](multi-setup.md), activate a setup after the
first and call `update_setup` with `{"patch":{"stock":"PreviousSetup"}}`.
Generation and simulation recalculate the preceding setup's included operations.
`get_setup_bounds` reports the enclosing original blank in the active setup's axes.

## Connect to an open desktop window

1. In Open CAM Studio, open **Preferences → MCP**, select **Enable MCP**, and click **OK**.
2. Reopen **Preferences → MCP**, select **Copy client configuration** and add the `opencamstudio` entry to your MCP client's configuration. Reload the server in the client if required.
3. The client can now use tools such as `get_state` with the project in the open window.

The setting is saved and starts MCP each time you open the desktop app. Turn it
off to disconnect the client and stop automatic startup. Quitting the app also
disconnects the client.

The app listens on `127.0.0.1` and shows the current port. The copied client
configuration uses the stdio bridge `opencamstudio-mcp --mcp-connect`, which finds
the current port and access token in the per-user configuration directory. Reuse
the same client configuration after restarting the app. If the client stays open,
reload its MCP server to reconnect. Each window accepts one client at a time.
When several windows have MCP enabled, the bridge uses the most recently
published connection.

If you set up the older client configuration with an explicit port and token,
copy the new configuration once to enable automatic discovery.

## Launch a new session from the client

```sh
# Headless server; no graphics or windowing dependencies.
cargo build -p cam-app --bin opencamstudio-mcp
./target/debug/opencamstudio-mcp

# Desktop-capable server.
cargo build -p cam-app --features gui --bin opencamstudio-mcp
./target/debug/opencamstudio-mcp --mcp-gui
```

Add `occt` to the feature list for STEP/IGES support. DXF/DWG does not require it.
Use `--setup /absolute/path/part.ocam` in either mode to load an existing project
before serving requests. Relative paths resolve from the process working directory.

The desktop executable also accepts `--mcp` and `--mcp-gui`. Use the dedicated
`opencamstudio-mcp` executable for MCP client configuration, especially on Windows,
where release GUI applications have no console. Release packaging includes this
executable beside the desktop executable; on macOS both are inside the app's
`Contents/MacOS` directory. An AppImage can be extracted to access its
`usr/bin/opencamstudio-mcp`, using the extracted `usr/lib` as `LD_LIBRARY_PATH`.

A typical MCP client's server configuration is:

```json
{
  "mcpServers": {
    "opencamstudio": {
      "command": "/absolute/path/opencamstudio-mcp",
      "args": ["--mcp-gui", "--setup", "/absolute/path/part.ocam"]
    }
  }
}
```

Omit `--mcp-gui` for a headless session. Each server process owns one project
session. With `--mcp-gui` the MCP client launches a new window and commands control
that window's controller and undo history. Closing the window or closing the MCP client's
stdin ends the desktop server. Headless mode exits on stdin EOF.

## Available commands

The catalog covers application actions, with parameter edits exposed as validated
field patches instead of individual keystrokes or file dialogs. Call `get_state`
before editing to discover the current serialized model and stable IDs. Coordinates
and distances are in millimetres; feeds are mm/min, spindle speeds are RPM, camera
angles are degrees, and playback/processing times are seconds.

| Area | Commands |
| --- | --- |
| Project | `get_state`, `new_project`, `open_project`, `save_project`, `undo`, `redo`, `select` |
| Drawing import and selection | `open_dxf`, `import_cad`, `get_drawing_geometry`, `nearest_drawing_feature`, `planar_operation_eligibility` |
| Background CAD import | `start_cad_import`, `start_cad_replacement`, `poll_cad_import`, `commit_cad_preview`, `cancel_cad_import` |
| CAD assets and selection | `get_geometry_state`, `set_cad_selection`, `add_cad_asset`, `replace_cad_asset`, `remove_cad_asset`, `place_cad_instance`, `align_cad_top_face`, `cad_operation_eligibility` |
| Setups and stock | `activate_setup`, `add_setup`, `duplicate_setup`, `delete_setup`, `reorder_setup`, `update_setup`, `frame_from_model`, `frame_from_edges`, `get_setup_bounds` |
| Work origins | `select_origin`, `add_origin`, `delete_origin`, `set_origin_position`, `set_origin_index`, `set_origin_disabled`, `set_operation_origin` |
| Operation creation | `get_operation_template`, `begin_operation`, `begin_operation_from_selection`, `set_pending_tool`, `clear_pending_tool`, `set_pending_geometry`, `confirm_operation`, `cancel_operation`, `create_operation`, `create_cad_operation`, `create_cad_operation_from_tool`, `reinitialize_operation` |
| Operation editing | `update_operation`, `edit_drawing_geometry`, `edit_cad_geometry`, `override_cad_heights`, `apply_operation_preset`, `duplicate_operation`, `delete_operation`, `set_operation_excluded`, `move_operation`, `reorder_operation`, `transfer_operation`, `duplicate_operation_groups` |
| Patterns | `create_pattern`, `edit_pattern`, `dissolve_pattern`, `delete_pattern`, `set_pattern_suppressed` |
| Embedded tools | `add_tool`, `use_tool`, `update_tool`, `delete_tool`, `prune_unused_tools`, `reconcile_tools` |
| Shop tools and presets | `get_tool_library`, `update_tool_library`, `import_tool_library`, `export_tool_library`, `add_library_tool`, `duplicate_library_tool`, `update_library_tool`, `delete_library_tool`, `set_library_tool_number`, `renumber_tool_library`, `embed_library_tool`, `promote_project_tool` |
| Materials and cutting forces | `add_material`, `rename_material`, `set_material_cutting_force`, `delete_material`, `get_cutting_forces`; setup material and tool presets are editable through `update_setup` and `update_library_tool`. See [cutting-force assumptions](cutting-forces.md). |
| Machines and posts | `set_machine`, `update_machine`, `set_post_kind`, `set_estlcam_options`, `get_machine_library`, `add_machine`, `update_library_machine`, `delete_machine`, `activate_machine` |
| Toolpaths and verification | `start_run`, `start_toolpath_run`, `start_pattern_toolpath_run`, `get_run_status`, `cancel_run`, `get_program`, `get_stock_mesh`, `get_cache_stats`, `get_processing_time`, `set_simulation_accuracy`, `export_nc` |
| Preferences | `get_preferences`, `update_preferences`, `restore_preferences`, `save_shop` |
| Desktop only | `get_desktop_state`, `desktop_set_view`, `desktop_set_pane`, `desktop_select_ribbon`, `desktop_set_visibility`, `desktop_isolate_instance`, `desktop_open_editor`, `desktop_command`, `desktop_simulation`, `get_desktop_output` |

`update_setup` covers setup name, work coordinates, model membership, stock,
safety heights, material and postprocess settings. `update_operation` covers all
operation fields, including passes, leads, entry/exit positions, compensation,
tabs, cooling, cutting presets, geometry, reference-based heights and stock to
leave. A patch merges object fields and replaces arrays; misspelled patch fields
are rejected. Operation IDs and kinds cannot be changed through a patch.

Automatic CAD feature verification is advisory. A usable selection can proceed
with informational diagnostics when automatic wall, access or side recognition
fails. Geometry-data errors remain distinct from these findings. Selection and
operation creation also append local structured verification observations for
later analysis; see [Geometry verification](geometry-verification.md).

For example, after inspecting `get_state`, change a profile feed with:

```json
{"name":"update_operation","arguments":{"id":0,"patch":{"Profile":{"feed":250}}}}
```

The object uses the same enum representation as the saved project. To change a
setup name or clearance height:

```json
{"name":"update_setup","arguments":{"patch":{"name":"Top side","heights":{"clearance":15}}}}
```

A rectangular operation pattern uses:

```json
{
  "name": "create_pattern",
  "arguments": {
    "setup_id": 1,
    "members": [0],
    "name": "Two copies",
    "kind": {"kind":"rectangular","nx":2,"ny":1,"dx":80,"dy":0},
    "order": "by_instance"
  }
}
```

Circular patterns use `kind: "circular"`, `center_xy`, `count`, `direction`
(`cw`/`ccw`) and `distribution` (`{"kind":"full"}` or
`{"kind":"partial","sweep_degrees":180}`).

The shop library, machines and preferences start from the user's normal settings.
MCP shop edits stay in memory until `save_shop`, or until a normal desktop action
persists the corresponding settings. `export_tool_library` writes a separate
`.ocam` library. Project edits require `save_project` to write a project file.

## Jobs and desktop interaction

1. Import geometry and create or edit the required operations.
2. Call `start_run`, or use `start_toolpath_run` for an operation scope.
3. Poll `get_run_status` until `pending` is false. Inspect completion, diagnostics
   and collisions. `cancel_run` cancels work and clears export eligibility.
4. In desktop mode, use `desktop_simulation` with action `open` to inspect playback.
   Other actions include play, pause, seek, speed and point/operation stepping.
5. Call `export_nc` to return G-code, or supply `path` to write it. Incomplete or
   failed planning and post/machine validation errors block export. Stock and
   fixture collisions block export by default. The preference
   `block_export_on_verification_errors` controls whether `get_run_status` reports
   collisions as errors or warnings and whether they block export.

Exact duplicate included operations require `allow_duplicates: true` on export,
matching the desktop's confirmation for intentional repeated cuts.

Background CAD import similarly requires polling and an explicit
`commit_cad_preview` before document changes are published.

Desktop edits already staged in an inspector are preserved: MCP mutations return
an actionable error until those edits are applied or cancelled. Read commands and
job cancellation remain available. `desktop_command` accepts `apply_editor` or
`cancel_editor`, as well as view toggles and presentation actions. The desktop
state response reports dirty editor status. Camera, pane and playback tools are
advertised only when the server controls a desktop session.

## Transport

The server implements the [MCP stdio transport](https://modelcontextprotocol.io/specification/2025-11-25/basic/transports):
one UTF-8 JSON-RPC message per line, initialization/capability negotiation,
`notifications/initialized`, `ping`, `tools/list` and `tools/call`. Diagnostics go
to stderr; stdout carries protocol messages only. Tool failures return
`isError: true`; malformed protocol requests return JSON-RPC errors. No TCP/HTTP
listener is opened. Each input message is limited to 16 MiB.
