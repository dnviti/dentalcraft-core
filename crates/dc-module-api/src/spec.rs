//! Agent-drivable commands: a stable id, i18n keys for the label and the
//! docs, the JSON Schema of the parameters and a pure run function.
//!
//! A [`CommandSpec`] is what the CLI and the MCP server list, describe and
//! execute; the shell runs the same spec through [`SpecCommand`], which
//! plugs it into the undo stack as a [`Command`].

use dc_case::CaseState;
use dc_command::{Command, CommandError, JobCtx};
use serde_json::Value;

/// What a command run returns: the new case state and a JSON output
/// (summary, measures, warnings...) for the caller.
pub type CommandOutput = (CaseState, Value);

/// Run function of a [`CommandSpec`]: reads the parameters from JSON
/// (validated against the schema by the caller or by the function itself),
/// computes the new state without side effects, reports progress and checks
/// cancellation through the [`JobCtx`].
pub type CommandRunFn = fn(&CaseState, &Value, &JobCtx) -> Result<CommandOutput, CommandError>;

/// A registered command with parameters, as exposed to agents.
#[derive(Debug, Clone, Copy)]
pub struct CommandSpec {
    /// Stable id, e.g. `aligners.preparation.segmentation.automatic`.
    pub id: &'static str,
    /// i18n key of the label.
    pub label_key: &'static str,
    /// i18n key of the documentation (what it does, what the parameters
    /// mean).
    pub doc_key: &'static str,
    /// JSON Schema of the parameters object (from schemars).
    pub params_schema: fn() -> Value,
    /// The command itself.
    pub run: CommandRunFn,
}

impl CommandSpec {
    /// JSON Schema of the parameters.
    pub fn schema(&self) -> Value {
        (self.params_schema)()
    }

    /// Runs the command on `state` with `params`.
    pub fn execute(
        &self,
        state: &CaseState,
        params: &Value,
        ctx: &JobCtx,
    ) -> Result<CommandOutput, CommandError> {
        (self.run)(state, params, ctx)
    }

    /// The command bound to `params`, ready for the undo stack.
    pub fn command(&self, params: Value) -> SpecCommand {
        SpecCommand {
            spec: *self,
            params,
        }
    }
}

/// A [`CommandSpec`] bound to its parameters, usable as an undoable
/// [`Command`]. The JSON output of the run is dropped.
#[derive(Debug, Clone)]
pub struct SpecCommand {
    spec: CommandSpec,
    params: Value,
}

impl SpecCommand {
    /// The spec.
    pub fn spec(&self) -> &CommandSpec {
        &self.spec
    }

    /// The bound parameters.
    pub fn params(&self) -> &Value {
        &self.params
    }
}

impl Command for SpecCommand {
    fn id(&self) -> &'static str {
        self.spec.id
    }

    fn label_key(&self) -> &str {
        self.spec.label_key
    }

    fn apply(&self, state: &CaseState, ctx: &JobCtx) -> Result<CaseState, CommandError> {
        self.spec
            .execute(state, &self.params, ctx)
            .map(|(state, _)| state)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn schema() -> Value {
        json!({ "type": "object", "properties": { "n": { "type": "integer" } } })
    }

    fn run(state: &CaseState, params: &Value, ctx: &JobCtx) -> Result<CommandOutput, CommandError> {
        ctx.check_cancelled()?;
        let n = params.get("n").and_then(Value::as_i64).unwrap_or(0);
        if n < 0 {
            return Err(CommandError::Invalid("n < 0".into()));
        }
        Ok((state.clone(), json!({ "doubled": n * 2 })))
    }

    const SPEC: CommandSpec = CommandSpec {
        id: "demo.double",
        label_key: "demo-double",
        doc_key: "demo-double-doc",
        params_schema: schema,
        run,
    };

    #[test]
    fn executes_and_describes() {
        let ctx = JobCtx::detached();
        let state = CaseState::default();
        let (_, out) = SPEC.execute(&state, &json!({ "n": 21 }), &ctx).unwrap();
        assert_eq!(out, json!({ "doubled": 42 }));
        assert!(SPEC.execute(&state, &json!({ "n": -1 }), &ctx).is_err());
        assert_eq!(SPEC.schema()["type"], "object");
    }

    #[test]
    fn bound_spec_is_an_undoable_command() {
        let cmd = SPEC.command(json!({ "n": 1 }));
        assert_eq!(cmd.id(), "demo.double");
        assert_eq!(cmd.label_key(), "demo-double");
        assert_eq!(cmd.params(), &json!({ "n": 1 }));
        assert_eq!(cmd.spec().doc_key, "demo-double-doc");
        let ctx = JobCtx::detached();
        assert!(cmd.apply(&CaseState::default(), &ctx).is_ok());
        let bad = SPEC.command(json!({ "n": -3 }));
        assert!(bad.apply(&CaseState::default(), &ctx).is_err());
    }
}
