//! MCP tools: definitions (name, description, input schema) and dispatch to
//! the [`Host`].

use serde_json::{Value, json};

use crate::{Host, HostError, HostResult, base64};

fn obj(properties: Value, required: &[&str]) -> Value {
    json!({
        "type": "object",
        "properties": properties,
        "required": required,
        "additionalProperties": false,
    })
}

fn tool(name: &str, title: &str, description: &str, input_schema: Value) -> Value {
    json!({
        "name": name,
        "title": title,
        "description": description,
        "inputSchema": input_schema,
    })
}

/// Every tool, as returned by `tools/list`.
pub fn tool_definitions() -> Vec<Value> {
    let path = |d: &str| json!({ "type": "string", "description": d });
    vec![
        tool(
            "list_commands",
            "List commands",
            "Lists every command id with its label and source (session built-in or module).",
            obj(json!({}), &[]),
        ),
        tool(
            "describe_command",
            "Describe command",
            "Returns the documentation and the JSON Schema of the parameters of a command.",
            obj(
                json!({ "id": { "type": "string", "description": "Command id." } }),
                &["id"],
            ),
        ),
        tool(
            "execute",
            "Execute command",
            "Runs a command with JSON parameters and returns its output. Changes to the case are undoable.",
            obj(
                json!({
                    "id": { "type": "string", "description": "Command id (see list_commands)." },
                    "params": { "type": "object", "description": "Parameters, as described by describe_command." }
                }),
                &["id"],
            ),
        ),
        tool(
            "batch",
            "Execute commands",
            "Runs several commands in order and returns their outputs. Stops at the first failure; the commands before it stay applied.",
            obj(
                json!({
                    "commands": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "id": { "type": "string" },
                                "params": { "type": "object" }
                            },
                            "required": ["id"],
                            "additionalProperties": false
                        }
                    }
                }),
                &["commands"],
            ),
        ),
        tool(
            "inspect",
            "Inspect case",
            "JSON summary of the project and the case: scans, meshes with vertex/face counts, module state, steps, undo depth, view. Pass a path (`scans.0`, `modules.aligners`, or a JSON pointer) for one part in full.",
            obj(
                json!({ "path": path("Dotted path or JSON pointer; whole summary when missing.") }),
                &[],
            ),
        ),
        tool(
            "open",
            "Open project",
            "Opens a .dcprj project file. Damaged or newer files open read-only.",
            obj(
                json!({ "path": path("Path of the .dcprj file.") }),
                &["path"],
            ),
        ),
        tool(
            "save",
            "Save project",
            "Saves the project as .dcprj, to path or to the open file.",
            obj(
                json!({ "path": path("Target path (.dcprj is appended when missing); the open file when missing.") }),
                &[],
            ),
        ),
        tool(
            "new_project",
            "New project",
            "Starts a new empty project. Unsaved changes are dropped.",
            obj(
                json!({ "patient_code": { "type": "string", "description": "Pseudonymised patient code." } }),
                &[],
            ),
        ),
        tool(
            "import_scan",
            "Import scans",
            "Imports STL, OBJ or PLY scans into the case as one undoable step. Arches are converted to \
millimetres; a bite scan stays as it is in the file, with a `units_suspect` warning when its size fits \
another unit. Give exactly one of `path` (one file) or `paths` (several files). Without a role the import \
decides: the file name, then a free arch, marked provisional when it is a guess. A scan with a role or a \
name hint replaces the one with the same role; a scan with neither never replaces a confirmed one and \
fails with `needs_role` when it finds no place; a case holds at most 2 bite scans. Runs the module's \
`*.import-scans` command when one is registered (it can also recognise the arch from the geometry), otherwise `case.import`. The result \
lists the imported scans with their `role_source`, the `warnings` (code and message) and the `failed` files; \
it is an error when nothing was imported.",
            obj(
                json!({
                    "path": path("Path of one mesh file. Give this or `paths`."),
                    "paths": {
                        "type": "array",
                        "items": { "type": "string" },
                        "minItems": 1,
                        "description": "Paths of several mesh files. Give this or `path`."
                    },
                    "role": {
                        "type": "string",
                        "enum": ["auto", "upper", "lower", "bite"],
                        "description": "Arch or bite for the file; with `paths`, for every file, whatever its name. So `upper` or `lower` on two or more files fails with `role_clash` for all of them, and only the first 2 stay with `bite`. `auto` or missing lets the import decide, file by file."
                    }
                }),
                &[],
            ),
        ),
        tool(
            "undo",
            "Undo",
            "Reverts the last change to the case.",
            obj(json!({}), &[]),
        ),
        tool(
            "redo",
            "Redo",
            "Re-applies the last undone change.",
            obj(json!({}), &[]),
        ),
        tool(
            "steps",
            "Processing steps",
            "Lists the processing steps and their state; with rewind, goes back to the state right before that step.",
            obj(
                json!({ "rewind": { "type": "string", "description": "Step id to go back to." } }),
                &[],
            ),
        ),
        tool(
            "render",
            "Render view",
            "Renders the current view of the case as a PNG image (view parameters from execute view.set). Not available until the headless renderer lands.",
            obj(
                json!({
                    "width": { "type": "integer", "minimum": 1 },
                    "height": { "type": "integer", "minimum": 1 },
                    "preset": { "type": "string", "enum": ["front", "back", "left", "right", "top", "bottom", "iso"] }
                }),
                &[],
            ),
        ),
    ]
}

fn str_arg<'a>(args: &'a Value, key: &str) -> HostResult<&'a str> {
    args.get(key)
        .and_then(Value::as_str)
        .ok_or_else(|| HostError::new(format!("missing string argument `{key}`")))
}

fn opt_str_arg<'a>(args: &'a Value, key: &str) -> HostResult<Option<&'a str>> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(s)) => Ok(Some(s)),
        Some(_) => Err(HostError::new(format!("argument `{key}` must be a string"))),
    }
}

fn text_result(v: &Value) -> Value {
    let text = serde_json::to_string_pretty(v).unwrap_or_else(|_| v.to_string());
    let mut out = json!({ "content": [{ "type": "text", "text": text }] });
    if v.is_object() {
        out["structuredContent"] = v.clone();
    }
    out
}

fn error_result(msg: &str) -> Value {
    json!({ "content": [{ "type": "text", "text": msg }], "isError": true })
}

/// Runs tool `name`; `None` when no such tool exists.
pub(crate) fn call<H: Host>(host: &mut H, name: &str, args: &Value) -> Option<Value> {
    let r: HostResult = match name {
        "list_commands" => host.list(),
        "describe_command" => str_arg(args, "id").and_then(|id| host.describe(id)),
        "execute" => str_arg(args, "id").and_then(|id| {
            let params = args.get("params").cloned().unwrap_or(Value::Null);
            host.execute(id, params)
        }),
        "batch" => parse_batch(args).and_then(|cmds| host.batch(cmds)),
        "inspect" => opt_str_arg(args, "path").and_then(|p| host.inspect(p)),
        "open" => str_arg(args, "path").and_then(|p| host.open(p)),
        "save" => opt_str_arg(args, "path").and_then(|p| host.save(p)),
        "new_project" => opt_str_arg(args, "patient_code").and_then(|code| {
            host.execute(
                "project.new",
                json!({ "patient_code": code.unwrap_or_default() }),
            )
        }),
        "import_scan" => import_scan(host, args),
        "undo" => host.undo(),
        "redo" => host.redo(),
        "steps" => opt_str_arg(args, "rewind").and_then(|step| match step {
            Some(step) => host.execute("steps.rewind", json!({ "step": step })),
            None => host.execute("steps.list", Value::Null),
        }),
        "render" => return Some(render(host, args)),
        _ => return None,
    };
    Some(match r {
        Ok(v) => text_result(&v),
        Err(e) => error_result(&e.0),
    })
}

/// Id of the command that imports scans: the one a module registers with the
/// [`IMPORT_SCANS_SUFFIX`](dc_module_api::IMPORT_SCANS_SUFFIX) convention
/// (it can recognise the arch from the geometry), else `case.import`.
fn import_command<H: Host>(host: &mut H) -> String {
    host.list()
        .ok()
        .and_then(|list| {
            list.as_array()?
                .iter()
                .filter_map(|c| c.get("id").and_then(Value::as_str).or_else(|| c.as_str()))
                .find(|id| dc_module_api::is_import_scans_command(id))
                .map(str::to_owned)
        })
        .unwrap_or_else(|| "case.import".to_owned())
}

/// The `import_scan` tool: `path` or `paths`, and a role.
fn import_scan<H: Host>(host: &mut H, args: &Value) -> HostResult {
    let path = opt_str_arg(args, "path")?;
    let paths = match args.get("paths") {
        None | Some(Value::Null) => None,
        Some(Value::Array(items)) => Some(
            items
                .iter()
                .map(|p| {
                    p.as_str().ok_or_else(|| {
                        HostError::new("argument `paths` must be an array of strings")
                    })
                })
                .collect::<HostResult<Vec<&str>>>()?,
        ),
        Some(_) => {
            return Err(HostError::new(
                "argument `paths` must be an array of strings",
            ));
        }
    };
    let mut params = match (path, paths) {
        (Some(p), None) => json!({ "path": p }),
        (None, Some(paths)) if !paths.is_empty() => {
            json!({ "files": paths.iter().map(|p| json!({ "path": p })).collect::<Vec<_>>() })
        }
        _ => {
            return Err(HostError::new(
                "give exactly one of `path` (one file) or `paths` (a non-empty list of files)",
            ));
        }
    };
    if let Some(role) = opt_str_arg(args, "role")? {
        params["role"] = json!(role);
    }
    let id = import_command(host);
    host.execute(&id, params)
}

fn parse_batch(args: &Value) -> HostResult<Vec<(String, Value)>> {
    let items = args
        .get("commands")
        .and_then(Value::as_array)
        .ok_or_else(|| HostError::new("missing array argument `commands`"))?;
    items
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let id = c
                .get("id")
                .and_then(Value::as_str)
                .ok_or_else(|| HostError::new(format!("commands[{i}]: missing `id`")))?;
            Ok((
                id.to_owned(),
                c.get("params").cloned().unwrap_or(Value::Null),
            ))
        })
        .collect()
}

/// The render hook: an image result when the host can render, an error
/// result otherwise.
fn render<H: Host>(host: &mut H, args: &Value) -> Value {
    match host.render(args) {
        Ok(Some(png)) => json!({
            "content": [{ "type": "image", "data": base64::encode(&png), "mimeType": "image/png" }]
        }),
        Ok(None) => error_result(
            "render is not available yet: the headless renderer (dc-raster) is not wired in",
        ),
        Err(e) => error_result(&e.0),
    }
}
