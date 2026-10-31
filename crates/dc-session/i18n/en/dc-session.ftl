# Built-in commands of the headless session (dc-session): labels and docs
# shown by the CLI and the MCP server.

session-cmd-project-new = New project
session-cmd-project-new-doc = Starts a new empty project (unsaved). Parameters: patient_code (optional pseudonymised code). Clears the undo history and the step history.
session-cmd-project-open = Open project
session-cmd-project-open-doc = Opens a .dcprj project file. Parameters: path. Damaged or newer files open read-only; the load issues are returned.
session-cmd-project-save = Save project
session-cmd-project-save-doc = Saves the project as .dcprj (atomic write, one .bak kept). Parameters: path (optional; defaults to the open file, the .dcprj extension is added when missing).
session-cmd-case-import = Import scan
session-cmd-case-import-doc = Imports an STL, OBJ or PLY scan into the case, converted to millimetres. Parameters: path, role (upper, lower or bite; guessed from the file name when missing). A scan replaces the one with the same role. Undoable.
session-cmd-edit-undo = Undo
session-cmd-edit-undo-doc = Reverts the last change to the case.
session-cmd-edit-redo = Redo
session-cmd-edit-redo-doc = Re-applies the last undone change.
session-cmd-steps-list = List steps
session-cmd-steps-list-doc = Lists the processing step records and, when a pipeline is known, the state of every step.
session-cmd-steps-rewind = Go back to step
session-cmd-steps-rewind-doc = Goes back to the state right before a processing step: the records of that step and of the following ones are removed. Parameters: step.
session-cmd-view-set = Set view
session-cmd-view-set-doc = Stores view parameters for the renderer (e.g. preset, width, height, visible). Keys set to null are removed. Does not change the case.
