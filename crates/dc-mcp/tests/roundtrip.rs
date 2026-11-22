//! MCP round trips over in-memory pipes against a headless session.

use std::io::{BufRead, BufReader, Cursor, Write};

use dc_mcp::{Host, HostResult, PROTOCOL_VERSION, Server, serve_stdio};
use dc_mesh::TriMesh;
use dc_session::Session;
use glam::Vec3;
use serde_json::{Value, json};

fn write_tetra(path: &std::path::Path) {
    let mesh = TriMesh::new(
        vec![
            Vec3::ZERO,
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(0.0, 10.0, 0.0),
            Vec3::new(0.0, 0.0, 10.0),
        ],
        vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]],
    );
    dc_mesh::io::write(path, &mesh, dc_mesh::MeshFormat::Stl).unwrap();
}

fn request(id: u64, method: &str, params: Value) -> String {
    json!({ "jsonrpc": "2.0", "id": id, "method": method, "params": params }).to_string()
}

fn call(id: u64, name: &str, arguments: Value) -> String {
    request(
        id,
        "tools/call",
        json!({ "name": name, "arguments": arguments }),
    )
}

/// Text content of a tool result, parsed as JSON.
fn payload(resp: &Value) -> Value {
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    serde_json::from_str(text).unwrap()
}

#[test]
fn initialize_list_execute_undo_over_buffers() {
    let dir = tempfile::tempdir().unwrap();
    let stl = dir.path().join("upper.stl");
    write_tetra(&stl);

    let lines = [
        request(
            1,
            "initialize",
            json!({
                "protocolVersion": PROTOCOL_VERSION,
                "capabilities": {},
                "clientInfo": { "name": "test", "version": "0" }
            }),
        ),
        json!({ "jsonrpc": "2.0", "method": "notifications/initialized" }).to_string(),
        String::new(),
        request(2, "tools/list", json!({})),
        call(
            3,
            "execute",
            json!({ "id": "case.import", "params": { "path": stl.to_str().unwrap() } }),
        ),
        call(4, "inspect", json!({ "path": "scans" })),
        call(5, "undo", json!({})),
        call(6, "inspect", json!({ "path": "scans" })),
        call(7, "redo", json!({})),
        request(8, "ping", Value::Null),
    ];
    let input = lines.join("\n") + "\n";
    let mut session = Session::new();
    let mut out = Vec::new();
    serve_stdio(&mut session, Cursor::new(input), &mut out).unwrap();

    let resp: Vec<Value> = String::from_utf8(out)
        .unwrap()
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect();
    // one response per request, none for the notification and the blank line
    assert_eq!(resp.len(), 8);
    let ids: Vec<u64> = resp.iter().map(|r| r["id"].as_u64().unwrap()).collect();
    assert_eq!(ids, (1..=8).collect::<Vec<_>>());

    let init = &resp[0]["result"];
    assert_eq!(init["protocolVersion"], PROTOCOL_VERSION);
    assert_eq!(init["serverInfo"]["name"], "dentalcraft");
    assert!(
        init["instructions"]
            .as_str()
            .unwrap()
            .contains("list_commands")
    );
    assert!(init["capabilities"]["tools"].is_object());

    let tools = resp[1]["result"]["tools"].as_array().unwrap();
    let names: Vec<&str> = tools.iter().map(|t| t["name"].as_str().unwrap()).collect();
    for n in [
        "list_commands",
        "describe_command",
        "execute",
        "batch",
        "inspect",
        "open",
        "save",
        "new_project",
        "import_scan",
        "undo",
        "redo",
        "steps",
        "render",
    ] {
        assert!(names.contains(&n), "{n} missing");
    }
    assert!(tools.iter().all(|t| t["inputSchema"]["type"] == "object"));

    let imported = &resp[2]["result"];
    assert_eq!(imported.get("isError"), None);
    assert_eq!(imported["structuredContent"]["role"], "upper");
    assert_eq!(payload(&resp[2])["faces"], 4);
    assert_eq!(payload(&resp[3]).as_array().unwrap().len(), 1);
    assert_eq!(payload(&resp[4])["done"], true);
    assert_eq!(payload(&resp[5]), json!([]));
    assert_eq!(payload(&resp[6])["done"], true);
    assert_eq!(resp[7]["result"], json!({}));

    // the session is left with the redone import
    assert_eq!(session.state().case.scans.len(), 1);
}

#[test]
fn errors_tool_errors_and_protocol_errors() {
    let mut server = Server::new(Session::new());
    let mut send = |line: &str| -> Value {
        serde_json::from_str(&server.handle_line(line).expect("a response")).unwrap()
    };

    // protocol errors
    assert_eq!(send("{not json")["error"]["code"], -32700);
    assert_eq!(send("[1, 2]")["error"]["code"], -32600);
    assert_eq!(
        send(r#"{"id": 1, "method": "ping"}"#)["error"]["code"],
        -32600
    );
    assert_eq!(
        send(&request(1, "nope/nope", json!({})))["error"]["code"],
        -32601
    );
    assert_eq!(
        send(&call(2, "no_such_tool", json!({})))["error"]["code"],
        -32602
    );
    assert_eq!(
        send(&request(3, "tools/call", json!({ "arguments": {} })))["error"]["code"],
        -32602
    );

    // tool errors are results with isError
    let r = send(&call(4, "execute", json!({ "id": "nope" })));
    assert_eq!(r["result"]["isError"], true);
    assert!(
        r["result"]["content"][0]["text"]
            .as_str()
            .unwrap()
            .contains("unknown command")
    );
    let r = send(&call(5, "execute", json!({})));
    assert_eq!(r["result"]["isError"], true);
    let r = send(&call(6, "save", json!({})));
    assert_eq!(r["result"]["isError"], true);
    let r = send(&call(7, "render", json!({ "width": 64 })));
    assert_eq!(r["result"]["isError"], true);
    assert!(
        r["result"]["content"][0]["text"]
            .as_str()
            .unwrap()
            .contains("not available")
    );
    let r = send(&call(
        8,
        "batch",
        json!({ "commands": [
        { "id": "view.set", "params": { "preset": "top" } },
        { "id": "nope" }
    ] }),
    ));
    assert_eq!(r["result"]["isError"], true);
    assert!(
        r["result"]["content"][0]["text"]
            .as_str()
            .unwrap()
            .contains("command 1 (nope)")
    );

    // the other tools succeed on an empty session
    let ok = |r: &Value| assert_eq!(r["result"].get("isError"), None, "{r}");
    ok(&send(&call(9, "list_commands", json!({}))));
    let r = send(&call(
        10,
        "describe_command",
        json!({ "id": "case.import" }),
    ));
    ok(&r);
    assert_eq!(
        payload(&r)["params_schema"]["oneOf"],
        json!([{"required": ["path"]}, {"required": ["files"]}])
    );
    ok(&send(&call(
        11,
        "new_project",
        json!({ "patient_code": "PZ-1" }),
    )));
    let r = send(&call(12, "steps", json!({})));
    ok(&r);
    assert_eq!(payload(&r)["rev"], 0);
    let r = send(&call(13, "inspect", json!({})));
    ok(&r);
    assert_eq!(payload(&r)["case"]["patient_code"], "PZ-1");
    assert_eq!(payload(&r)["view"]["preset"], "top");

    // notifications and client responses get no answer
    assert!(
        server
            .handle_line(r#"{"jsonrpc":"2.0","method":"notifications/cancelled","params":{}}"#)
            .is_none()
    );
    assert!(
        server
            .handle_line(r#"{"jsonrpc":"2.0","id":99,"result":{}}"#)
            .is_none()
    );
    assert!(
        server
            .handle_line(r#"{"jsonrpc":"2.0","method":"notifications/initialized"}"#)
            .is_none()
    );
    assert!(server.is_initialized());
}

#[test]
fn save_open_and_import_through_tools_over_os_pipes() {
    let dir = tempfile::tempdir().unwrap();
    let stl = dir.path().join("lower.stl");
    write_tetra(&stl);
    let project = dir.path().join("p.dcprj");

    let (req_r, mut req_w) = std::io::pipe().unwrap();
    let (resp_r, resp_w) = std::io::pipe().unwrap();
    let server = std::thread::spawn(move || {
        serve_stdio(Session::new(), BufReader::new(req_r), resp_w).unwrap();
    });
    let mut responses = BufReader::new(resp_r);
    let mut ask = |line: String| -> Value {
        writeln!(req_w, "{line}").unwrap();
        let mut buf = String::new();
        responses.read_line(&mut buf).unwrap();
        serde_json::from_str(&buf).unwrap()
    };

    ask(request(
        1,
        "initialize",
        json!({ "protocolVersion": PROTOCOL_VERSION }),
    ));
    let r = ask(call(
        2,
        "import_scan",
        json!({ "path": stl.to_str().unwrap(), "role": "lower" }),
    ));
    assert_eq!(r["result"]["structuredContent"]["role"], "lower");
    let r = ask(call(
        3,
        "save",
        json!({ "path": project.to_str().unwrap() }),
    ));
    assert_eq!(r["result"].get("isError"), None, "{r}");
    ask(call(4, "new_project", json!({})));
    let r = ask(call(5, "inspect", json!({ "path": "scans" })));
    assert_eq!(payload(&r), json!([]));
    let r = ask(call(
        6,
        "open",
        json!({ "path": project.to_str().unwrap() }),
    ));
    assert_eq!(payload(&r)["scans"], 1);
    let r = ask(call(7, "inspect", json!({ "path": "scans.0.vertices" })));
    assert_eq!(payload(&r), 4);

    drop(ask);
    drop(req_w);
    server.join().unwrap();
}

/// A host that can render, to exercise the image result.
struct Painter;

impl Host for Painter {
    fn execute(&mut self, _: &str, _: Value) -> HostResult {
        Ok(json!({}))
    }
    fn batch(&mut self, _: Vec<(String, Value)>) -> HostResult {
        Ok(json!([]))
    }
    fn list(&mut self) -> HostResult {
        Ok(json!([]))
    }
    fn describe(&mut self, _: &str) -> HostResult {
        Ok(json!({}))
    }
    fn inspect(&mut self, _: Option<&str>) -> HostResult {
        Ok(json!({}))
    }
    fn undo(&mut self) -> HostResult {
        Ok(json!({}))
    }
    fn redo(&mut self) -> HostResult {
        Ok(json!({}))
    }
    fn open(&mut self, _: &str) -> HostResult {
        Ok(json!({}))
    }
    fn save(&mut self, _: Option<&str>) -> HostResult {
        Ok(json!({}))
    }
    fn render(&mut self, _: &Value) -> HostResult<Option<Vec<u8>>> {
        Ok(Some(b"foobar".to_vec()))
    }
}

#[test]
fn render_hook_returns_an_image() {
    let mut server = Server::new(Painter);
    let r: Value =
        serde_json::from_str(&server.handle_line(&call(1, "render", json!({}))).unwrap()).unwrap();
    let c = &r["result"]["content"][0];
    assert_eq!(c["type"], "image");
    assert_eq!(c["mimeType"], "image/png");
    assert_eq!(c["data"], "Zm9vYmFy");
}

fn tool_result(server: &mut Server<impl Host>, id: u64, name: &str, args: Value) -> Value {
    let line = server.handle_line(&call(id, name, args)).unwrap();
    serde_json::from_str::<Value>(&line).unwrap()["result"].clone()
}

#[test]
fn import_scan_takes_paths_auto_and_returns_the_warnings() {
    let dir = tempfile::tempdir().unwrap();
    let (a, b, c) = ("scan.stl", "upper.stl", "morso.stl");
    for name in [a, b, c] {
        write_tetra(&dir.path().join(name));
    }
    let path = |n: &str| dir.path().join(n).to_str().unwrap().to_owned();
    let mut server = Server::new(Session::new());

    // one file, role auto: the import decides and says it is a guess
    let r = tool_result(
        &mut server,
        1,
        "import_scan",
        json!({ "path": path(a), "role": "auto" }),
    );
    assert_eq!(r.get("isError"), None, "{r}");
    let out = &r["structuredContent"];
    assert_eq!(out["role"], "lower");
    assert_eq!(out["role_source"], "provisional");
    let warnings = out["warnings"].as_array().unwrap();
    let codes: Vec<&str> = warnings
        .iter()
        .map(|w| w["code"].as_str().unwrap())
        .collect();
    assert!(codes.contains(&"provisional"), "{codes:?}");
    assert!(
        warnings
            .iter()
            .all(|w| !w["message"].as_str().unwrap().is_empty())
    );
    // the text content carries the same JSON
    let text: Value = serde_json::from_str(r["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(text["warnings"], out["warnings"]);

    // several files: the provisional one moves aside for the named upper arch
    let r = tool_result(
        &mut server,
        2,
        "import_scan",
        json!({ "paths": [path(b), path(c)] }),
    );
    assert_eq!(r.get("isError"), None, "{r}");
    let out = &r["structuredContent"];
    let names: Vec<&str> = out["imported"]
        .as_array()
        .unwrap()
        .iter()
        .map(|s| s["source_name"].as_str().unwrap())
        .collect();
    assert_eq!(names, [b, c]);
    assert_eq!(out["role"], "upper");
    assert_eq!(out["role_source"], "file_name");
    assert_eq!(out["failed"], json!([]));
    assert_eq!(out["imported"][1]["role"], "bite");
    assert_eq!(server_scans(&mut server), 3);

    // a partial batch succeeds and lists the failure; nothing imported is an error
    let r = tool_result(
        &mut server,
        3,
        "import_scan",
        json!({ "paths": [path("missing.stl"), path(c)] }),
    );
    assert_eq!(r.get("isError"), None, "{r}");
    assert_eq!(r["structuredContent"]["failed"][0]["code"], "unreadable");
    let r = tool_result(
        &mut server,
        4,
        "import_scan",
        json!({ "paths": [path("missing.stl")] }),
    );
    assert_eq!(r["isError"], true);
    assert!(
        r["content"][0]["text"]
            .as_str()
            .unwrap()
            .contains("missing.stl")
    );
}

/// With `paths`, the role goes to every file, whatever its name.
#[test]
fn import_scan_role_applies_to_every_path() {
    let dir = tempfile::tempdir().unwrap();
    for name in ["a.stl", "lower.stl", "c.stl"] {
        write_tetra(&dir.path().join(name));
    }
    let path = |n: &str| dir.path().join(n).to_str().unwrap().to_owned();
    let mut server = Server::new(Session::new());

    // both files claim the upper arch, so both fail and nothing is imported
    let r = tool_result(
        &mut server,
        1,
        "import_scan",
        json!({ "paths": [path("a.stl"), path("lower.stl")], "role": "upper" }),
    );
    assert_eq!(r["isError"], true);
    let text = r["content"][0]["text"].as_str().unwrap();
    assert!(
        text.contains("a.stl") && text.contains("lower.stl"),
        "{text}"
    );
    assert_eq!(server_scans(&mut server), 0);

    // a bite role goes to all of them, and the case keeps two
    let r = tool_result(
        &mut server,
        2,
        "import_scan",
        json!({ "paths": [path("a.stl"), path("lower.stl"), path("c.stl")], "role": "bite" }),
    );
    assert_eq!(r.get("isError"), None, "{r}");
    let imported = r["structuredContent"]["imported"].as_array().unwrap();
    assert_eq!(imported.len(), 2);
    assert!(imported.iter().all(|s| s["role"] == "bite"));
    let codes = r["structuredContent"]["warnings"].to_string();
    assert!(codes.contains("bite_limit"), "{codes}");

    // and the tool says so
    let line = server
        .handle_line(&request(3, "tools/list", json!({})))
        .unwrap();
    let list: Value = serde_json::from_str(&line).unwrap();
    let tools = list["result"]["tools"].as_array().unwrap();
    let tool = tools.iter().find(|t| t["name"] == "import_scan").unwrap();
    let doc = tool["inputSchema"]["properties"]["role"]["description"]
        .as_str()
        .unwrap();
    assert!(
        doc.contains("every file") && doc.contains("role_clash"),
        "{doc}"
    );
}

fn server_scans(server: &mut Server<impl Host>) -> usize {
    let r = tool_result(server, 90, "inspect", json!({ "path": "scans" }));
    let text = r["content"][0]["text"].as_str().unwrap();
    serde_json::from_str::<Value>(text)
        .unwrap()
        .as_array()
        .unwrap()
        .len()
}

#[test]
fn import_scan_rejects_path_and_paths_together_or_neither() {
    let mut server = Server::new(Session::new());
    for args in [
        json!({}),
        json!({ "path": "a.stl", "paths": ["b.stl"] }),
        json!({ "paths": [] }),
        json!({ "paths": "a.stl" }),
        json!({ "paths": [1] }),
        json!({ "path": "a.stl", "role": 3 }),
    ] {
        let r = tool_result(&mut server, 1, "import_scan", args.clone());
        assert_eq!(r["isError"], true, "{args}");
    }
    let r = tool_result(&mut server, 2, "inspect", json!({ "path": "scans" }));
    assert_eq!(r.get("isError"), None);
}

/// Records what the tools execute and lists the commands it is given.
struct Recorder {
    commands: Vec<&'static str>,
    executed: Vec<(String, Value)>,
}

impl Host for Recorder {
    fn execute(&mut self, id: &str, params: Value) -> HostResult {
        self.executed.push((id.to_owned(), params));
        Ok(json!({ "warnings": [] }))
    }
    fn batch(&mut self, _: Vec<(String, Value)>) -> HostResult {
        Ok(json!([]))
    }
    fn list(&mut self) -> HostResult {
        Ok(json!(
            self.commands
                .iter()
                .map(|id| json!({ "id": id, "source": "x", "label": "x" }))
                .collect::<Vec<_>>()
        ))
    }
    fn describe(&mut self, _: &str) -> HostResult {
        Ok(json!({}))
    }
    fn inspect(&mut self, _: Option<&str>) -> HostResult {
        Ok(json!({}))
    }
    fn undo(&mut self) -> HostResult {
        Ok(json!({}))
    }
    fn redo(&mut self) -> HostResult {
        Ok(json!({}))
    }
    fn open(&mut self, _: &str) -> HostResult {
        Ok(json!({}))
    }
    fn save(&mut self, _: Option<&str>) -> HostResult {
        Ok(json!({}))
    }
}

#[test]
fn import_scan_prefers_the_module_import_scans_command() {
    let mut with_module = Server::new(Recorder {
        commands: vec!["case.import", "aligners.case.import-scans", "aligners.auto"],
        executed: Vec::new(),
    });
    tool_result(
        &mut with_module,
        1,
        "import_scan",
        json!({ "paths": ["a.stl", "b.stl"], "role": "bite" }),
    );
    tool_result(
        &mut with_module,
        2,
        "import_scan",
        json!({ "path": "c.stl" }),
    );
    let host = with_module.host();
    assert_eq!(host.executed[0].0, "aligners.case.import-scans");
    assert_eq!(
        host.executed[0].1,
        json!({ "files": [{ "path": "a.stl" }, { "path": "b.stl" }], "role": "bite" })
    );
    assert_eq!(host.executed[1].0, "aligners.case.import-scans");
    assert_eq!(host.executed[1].1, json!({ "path": "c.stl" }));

    // a look-alike id does not count; without the command, case.import runs
    let mut without = Server::new(Recorder {
        commands: vec![
            "case.import",
            "aligners.case.import-scans-x",
            "import-scans.run",
        ],
        executed: Vec::new(),
    });
    tool_result(
        &mut without,
        1,
        "import_scan",
        json!({ "path": "a.stl", "role": "auto" }),
    );
    let host = without.host();
    assert_eq!(host.executed[0].0, "case.import");
    assert_eq!(
        host.executed[0].1,
        json!({ "path": "a.stl", "role": "auto" })
    );
}

/// `batch` and `import_scan` refuse lists longer than the schema's
/// `maxItems` before the host runs anything.
#[test]
fn long_lists_are_refused() {
    let tools = dc_mcp::tool_definitions();
    let batch = tools.iter().find(|t| t["name"] == "batch").unwrap();
    let max = batch["inputSchema"]["properties"]["commands"]["maxItems"]
        .as_u64()
        .unwrap() as usize;
    let mut server = Server::new(Painter);
    let commands = vec![json!({ "id": "steps.list" }); max + 1];
    let r = tool_result(&mut server, 1, "batch", json!({ "commands": commands }));
    assert_eq!(r["isError"], true, "{r}");
    assert!(
        r["content"][0]["text"]
            .as_str()
            .unwrap()
            .contains("at most")
    );
    let commands = vec![json!({ "id": "steps.list" }); max];
    let r = tool_result(&mut server, 2, "batch", json!({ "commands": commands }));
    assert!(r.get("isError").is_none(), "{r}");
    let paths = vec![json!("a.stl"); max + 1];
    let r = tool_result(&mut server, 3, "import_scan", json!({ "paths": paths }));
    assert_eq!(r["isError"], true, "{r}");
    assert!(
        r["content"][0]["text"]
            .as_str()
            .unwrap()
            .contains("at most")
    );
}
