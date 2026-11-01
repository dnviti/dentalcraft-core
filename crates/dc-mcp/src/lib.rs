//! MCP server for DentalCraft (port plan B1.1).
//!
//! Hand-rolled JSON-RPC 2.0 over stdio: one JSON message per line
//! (newline-delimited), MCP protocol [`PROTOCOL_VERSION`], `serde_json`
//! only, no async runtime. The server works against a [`Host`]: the
//! headless [`dc_session::Session`] (implemented here) or, later, a remote
//! host talking to a running app (`dc-app --control <port>`).
//!
//! Methods: `initialize`, `notifications/initialized`, `ping`,
//! `tools/list`, `tools/call`. Tool failures are results with
//! `isError: true`; malformed requests, unknown methods and unknown tools
//! are JSON-RPC errors.
//!
//! ```ignore
//! let mut session = dc_session::Session::new();
//! let stdin = std::io::stdin().lock();
//! dc_mcp::serve_stdio(&mut session, stdin, std::io::stdout().lock())?;
//! ```

mod base64;
mod session_host;
mod tools;

use std::io::{BufRead, Write};

use serde_json::{Value, json};

pub use tools::tool_definitions;

/// MCP protocol version spoken by the server.
pub const PROTOCOL_VERSION: &str = "2025-06-18";
/// `serverInfo.name`.
pub const SERVER_NAME: &str = "dentalcraft";

/// Instructions returned by `initialize`.
pub const INSTRUCTIONS: &str = "DentalCraft is a digital dentistry application (clear aligner planning). \
Every action is a command with an id and JSON parameters: list_commands shows the ids, describe_command \
gives the parameter schema and the docs, execute runs one (batch runs several in order). Typical flow: \
new_project or open (.dcprj) -> import_scan (STL/OBJ/PLY, one path or several paths, role auto/upper/lower/bite) -> execute module \
commands (aligners.*) -> inspect to read the case back (scans, meshes, module state, steps; pass a path \
such as `scans.0` or `modules.aligners` for details) -> save. Changes to the case are undoable (undo, \
redo). Coordinates are millimetres.";

/// JSON-RPC error codes.
pub mod codes {
    /// Invalid JSON.
    pub const PARSE_ERROR: i64 = -32700;
    /// Not a valid request object.
    pub const INVALID_REQUEST: i64 = -32600;
    /// Unknown method.
    pub const METHOD_NOT_FOUND: i64 = -32601;
    /// Invalid method parameters (also unknown tools).
    pub const INVALID_PARAMS: i64 = -32602;
}

/// Error reported by a [`Host`] operation; becomes an `isError` tool result.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("{0}")]
pub struct HostError(pub String);

impl HostError {
    /// Error with this message.
    pub fn new(msg: impl Into<String>) -> Self {
        Self(msg.into())
    }
}

/// Result of a [`Host`] operation.
pub type HostResult<T = Value> = Result<T, HostError>;

/// What the MCP server drives: a headless session or a running app.
///
/// Every method returns JSON for the agent. `new_project`, `import_scan`
/// and `steps` are run through [`Host::execute`]: with the built-in command
/// ids (`project.new`, `steps.list`, `steps.rewind`), and `import_scan` with
/// the `*.import-scans` command that [`Host::list`] shows, else `case.import`.
pub trait Host {
    /// Runs command `id` with `params`; returns its output.
    fn execute(&mut self, id: &str, params: Value) -> HostResult;
    /// Runs the commands in order; returns the array of outputs.
    fn batch(&mut self, commands: Vec<(String, Value)>) -> HostResult;
    /// The available commands.
    fn list(&mut self) -> HostResult;
    /// Schema and documentation of command `id`.
    fn describe(&mut self, id: &str) -> HostResult;
    /// Summary of the case, or the part at `path`.
    fn inspect(&mut self, path: Option<&str>) -> HostResult;
    /// Undoes the last change.
    fn undo(&mut self) -> HostResult;
    /// Redoes the last undone change.
    fn redo(&mut self) -> HostResult;
    /// Opens a project file.
    fn open(&mut self, path: &str) -> HostResult;
    /// Saves the project (to `path`, or to the open file).
    fn save(&mut self, path: Option<&str>) -> HostResult;
    /// Renders the view as PNG bytes; `Ok(None)` when rendering is not
    /// available (the default, until the headless rasteriser exists).
    fn render(&mut self, params: &Value) -> HostResult<Option<Vec<u8>>> {
        let _ = params;
        Ok(None)
    }
}

impl<T: Host + ?Sized> Host for &mut T {
    fn execute(&mut self, id: &str, params: Value) -> HostResult {
        (**self).execute(id, params)
    }
    fn batch(&mut self, commands: Vec<(String, Value)>) -> HostResult {
        (**self).batch(commands)
    }
    fn list(&mut self) -> HostResult {
        (**self).list()
    }
    fn describe(&mut self, id: &str) -> HostResult {
        (**self).describe(id)
    }
    fn inspect(&mut self, path: Option<&str>) -> HostResult {
        (**self).inspect(path)
    }
    fn undo(&mut self) -> HostResult {
        (**self).undo()
    }
    fn redo(&mut self) -> HostResult {
        (**self).redo()
    }
    fn open(&mut self, path: &str) -> HostResult {
        (**self).open(path)
    }
    fn save(&mut self, path: Option<&str>) -> HostResult {
        (**self).save(path)
    }
    fn render(&mut self, params: &Value) -> HostResult<Option<Vec<u8>>> {
        (**self).render(params)
    }
}

/// A JSON-RPC error response body.
#[derive(Debug, Clone, PartialEq)]
struct RpcError {
    code: i64,
    message: String,
}

impl RpcError {
    fn new(code: i64, message: impl Into<String>) -> Self {
        Self {
            code,
            message: message.into(),
        }
    }
}

/// The MCP server state around a [`Host`].
pub struct Server<H: Host> {
    host: H,
    initialized: bool,
}

impl<H: Host> Server<H> {
    /// Server over `host`.
    pub fn new(host: H) -> Self {
        Self {
            host,
            initialized: false,
        }
    }

    /// The host.
    pub fn host(&self) -> &H {
        &self.host
    }

    /// The host, mutable.
    pub fn host_mut(&mut self) -> &mut H {
        &mut self.host
    }

    /// `true` once the client sent `notifications/initialized`.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Handles one line of input; returns the response line (without the
    /// newline), or `None` for notifications and blank lines.
    pub fn handle_line(&mut self, line: &str) -> Option<String> {
        let line = line.trim();
        if line.is_empty() {
            return None;
        }
        let msg: Value = match serde_json::from_str(line) {
            Ok(v) => v,
            Err(e) => {
                return Some(error_response(
                    Value::Null,
                    &RpcError::new(codes::PARSE_ERROR, format!("parse error: {e}")),
                ));
            }
        };
        self.handle(msg).map(|v| v.to_string())
    }

    /// Handles one parsed message; returns the response, or `None` for
    /// notifications.
    pub fn handle(&mut self, msg: Value) -> Option<Value> {
        let Value::Object(obj) = msg else {
            return Some(error_value(
                Value::Null,
                &RpcError::new(
                    codes::INVALID_REQUEST,
                    "expected a JSON-RPC request object (batches are not supported)",
                ),
            ));
        };
        let id = obj.get("id").cloned();
        let method = obj.get("method").and_then(Value::as_str);
        let valid_id = matches!(
            id,
            None | Some(Value::String(_)) | Some(Value::Number(_)) | Some(Value::Null)
        );
        let (Some(method), true, true) =
            (method, obj.get("jsonrpc") == Some(&json!("2.0")), valid_id)
        else {
            // a response from the client (no method) is ignored
            if method.is_none() && (obj.contains_key("result") || obj.contains_key("error")) {
                return None;
            }
            return Some(error_value(
                id.unwrap_or(Value::Null),
                &RpcError::new(codes::INVALID_REQUEST, "invalid JSON-RPC 2.0 request"),
            ));
        };
        let params = obj.get("params").cloned().unwrap_or(Value::Null);
        let Some(id) = id else {
            self.notification(method);
            return None;
        };
        Some(match self.request(method, params) {
            Ok(result) => json!({ "jsonrpc": "2.0", "id": id, "result": result }),
            Err(e) => error_value(id, &e),
        })
    }

    fn notification(&mut self, method: &str) {
        if method == "notifications/initialized" {
            self.initialized = true;
        }
        // other notifications (cancelled, progress, ...) are ignored:
        // requests run synchronously and cannot be interrupted
    }

    fn request(&mut self, method: &str, params: Value) -> Result<Value, RpcError> {
        match method {
            "initialize" => Ok(json!({
                "protocolVersion": PROTOCOL_VERSION,
                "capabilities": { "tools": { "listChanged": false } },
                "serverInfo": {
                    "name": SERVER_NAME,
                    "title": "DentalCraft",
                    "version": env!("CARGO_PKG_VERSION"),
                },
                "instructions": INSTRUCTIONS,
            })),
            "ping" => Ok(json!({})),
            "tools/list" => Ok(json!({ "tools": tool_definitions() })),
            "tools/call" => {
                let name = params
                    .get("name")
                    .and_then(Value::as_str)
                    .ok_or_else(|| RpcError::new(codes::INVALID_PARAMS, "missing tool name"))?;
                let args = match params.get("arguments") {
                    None | Some(Value::Null) => json!({}),
                    Some(v @ Value::Object(_)) => v.clone(),
                    Some(_) => {
                        return Err(RpcError::new(
                            codes::INVALID_PARAMS,
                            "arguments must be an object",
                        ));
                    }
                };
                tools::call(&mut self.host, name, &args).ok_or_else(|| {
                    RpcError::new(codes::INVALID_PARAMS, format!("unknown tool: {name}"))
                })
            }
            _ => Err(RpcError::new(
                codes::METHOD_NOT_FOUND,
                format!("method not found: {method}"),
            )),
        }
    }
}

fn error_value(id: Value, e: &RpcError) -> Value {
    json!({
        "jsonrpc": "2.0",
        "id": id,
        "error": { "code": e.code, "message": e.message },
    })
}

fn error_response(id: Value, e: &RpcError) -> String {
    error_value(id, e).to_string()
}

/// Serves MCP over newline-delimited JSON: reads requests from `reader`
/// until end of input and writes one response line per request to
/// `writer` (flushed after each).
pub fn serve_stdio<H: Host>(
    host: H,
    reader: impl BufRead,
    mut writer: impl Write,
) -> std::io::Result<()> {
    let mut server = Server::new(host);
    for line in reader.lines() {
        let line = line?;
        if let Some(resp) = server.handle_line(&line) {
            writer.write_all(resp.as_bytes())?;
            writer.write_all(b"\n")?;
            writer.flush()?;
        }
    }
    Ok(())
}
