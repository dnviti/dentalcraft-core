//! The `print.*` commands through a headless session: listed with their
//! schemas and translated labels, run on the session, refused with their
//! stable codes.

use dc_command::CommandError;
use dc_i18n::Lang;
use dc_session::{Session, SessionError};
use serde_json::{Value, json};

#[test]
fn print_commands_are_listed_run_and_refused() {
    let mut s = Session::new();
    let ids: Vec<String> = s.list_commands().into_iter().map(|c| c.id).collect();
    for info in dc_print::commands::command_infos() {
        assert!(ids.iter().any(|i| i == info.id), "{} not listed", info.id);
        let d = s.describe(info.id).unwrap();
        assert_eq!(d.source, dc_session::SESSION_SOURCE);
        assert_eq!(d.params_schema["type"], "object", "{}", info.id);
    }
    dc_i18n::set_language(Lang::It).unwrap();
    assert_eq!(
        s.describe("print.prepare").unwrap().label,
        "Prepara piatti di stampa"
    );
    dc_i18n::set_language(Lang::En).unwrap();
    assert_eq!(
        s.describe("print.prepare").unwrap().label,
        "Prepare print plates"
    );
    let v = s.execute("print.profiles", Value::Null).unwrap();
    assert_eq!(v["profiles"][0]["id"], "saturn-4-ultra-16k");
    assert_eq!(v["profiles"][0]["machine"]["name"], "DENTAL 45");
    let m = s.execute("print.machine-params", Value::Null).unwrap();
    assert_eq!(m["z_travel_mm"], 230.0);
    assert_eq!(m["vat_max_ml"], 870.0);
    assert_eq!(m["entries"].as_array().unwrap().len(), 160);
    dc_i18n::set_language(Lang::It).unwrap();
    assert_eq!(
        s.describe("print.machine-params").unwrap().label,
        "Parametri macchina della stampante"
    );
    dc_i18n::set_language(Lang::En).unwrap();

    // no resin values: refused as an invalid call, before reading anything
    let e = s
        .execute(
            "print.prepare",
            json!({"dir": "/nowhere", "out_dir": "/nowhere/out"}),
        )
        .unwrap_err();
    assert!(
        matches!(&e, SessionError::Command { source: CommandError::Invalid(m), .. }
            if m.starts_with("print-no-resin-preset: missing layer_um")),
        "{e}"
    );
    // with them, the missing folder is the error
    let e = s
        .execute(
            "print.prepare",
            json!({
                "dir": "/nowhere", "out_dir": "/nowhere/out", "layer_um": 50,
                "exposure_s": 2.5, "bottom_exposure_s": 30.0, "bottom_layers": 4,
                "rest_s": 1.5, "resin_name": "Clear aligner", "density_g_ml": 1.1
            }),
        )
        .unwrap_err();
    assert!(e.to_string().contains("print-no-inputs"), "{e}");
    assert!(!s.is_dirty());
}
