//! [`Host`] for the headless [`Session`].

use std::path::Path;

use dc_session::{Session, SessionError};
use serde_json::{Value, json};

use crate::{Host, HostError, HostResult};

impl From<SessionError> for HostError {
    fn from(e: SessionError) -> Self {
        HostError(e.to_string())
    }
}

fn to_json(v: impl serde::Serialize) -> HostResult {
    serde_json::to_value(v).map_err(|e| HostError(e.to_string()))
}

impl Host for Session {
    fn execute(&mut self, id: &str, params: Value) -> HostResult {
        Ok(Session::execute(self, id, params)?)
    }

    fn batch(&mut self, commands: Vec<(String, Value)>) -> HostResult {
        Ok(Value::Array(Session::batch(self, commands)?))
    }

    fn list(&mut self) -> HostResult {
        to_json(self.list_commands())
    }

    fn describe(&mut self, id: &str) -> HostResult {
        to_json(Session::describe(self, id)?)
    }

    fn inspect(&mut self, path: Option<&str>) -> HostResult {
        Ok(Session::inspect(self, path)?)
    }

    fn undo(&mut self) -> HostResult {
        Ok(Session::execute(self, "edit.undo", json!({}))?)
    }

    fn redo(&mut self) -> HostResult {
        Ok(Session::execute(self, "edit.redo", json!({}))?)
    }

    fn open(&mut self, path: &str) -> HostResult {
        Ok(Session::open(self, Path::new(path))?)
    }

    fn save(&mut self, path: Option<&str>) -> HostResult {
        Ok(Session::save(self, path.map(Path::new))?)
    }

    // render: the default (`None`) until dc-raster provides a headless
    // renderer reading `Session::view`.
}
