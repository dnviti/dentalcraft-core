# Built-in commands of the headless session (dc-session): labels and docs
# shown by the CLI and the MCP server.

session-cmd-project-new = New project
session-cmd-project-new-doc = Starts a new empty project (unsaved). Parameters: patient_code (optional pseudonymised code). Clears the undo history and the step history.
session-cmd-project-open = Open project
session-cmd-project-open-doc = Opens a .dcprj project file. Parameters: path. Damaged or newer files open read-only; the load issues are returned.
session-cmd-project-save = Save project
session-cmd-project-save-doc = Saves the project as .dcprj (atomic write, one .bak kept). Parameters: path (optional; defaults to the open file, the .dcprj extension is added when missing).
session-cmd-case-import = Import scan
session-cmd-case-import-doc = Imports STL, OBJ or PLY scans into the case, as one undoable step. Arches are converted to millimetres; a bite scan, known from its role or its file name, stays as it is in the file, with a units_suspect warning when its size fits another unit. Parameters: path (one file) or files (a list of objects with path and an optional role), exactly one of the two; role (auto, upper, lower or bite). With auto or no role the import decides: the file name, otherwise a free arch, marked provisional because it is a guess (a module may offer its own import-scans command that also recognises the arch from the geometry). With files, role is the default for the entries without their own role. A scan with a role or a name hint replaces the one with the same role, except that a provisional scan moves to the free arch instead; a scan with neither never replaces one and fails with needs_role when no arch is free. An upper or lower role given to two or more files fails with role_clash for all of them. A case holds at most 2 bite scans. While a module holds data this version cannot read, a file that would replace or move an arch fails with arches_locked; free arches and bite scans still work. Output: scan_id, role, role_source (explicit, file_name, detected or provisional), source_name, source_units, mesh, vertices, faces and replaced describe the first imported scan; imported lists every imported scan with the same fields plus path; warnings (code, key, args, message), failed (path, code, reason, detail), moved, cleanup (faces removed) and module_cleared report the rest. A batch with some failed files still succeeds, with failed filled in; when nothing is imported the command fails with the reasons and the case is unchanged.
session-cmd-case-set-role = Set scan role
session-cmd-case-set-role-doc = Changes the role of a scan, as one undoable step. Parameters: from (upper, lower or bite: the role the scan has now), to (the role it gets), scan_id (only needed when from is bite and the case has two bite scans). The role becomes explicit. A scan already on the target arch takes the from role, so the two swap. A case holds at most 2 bite scans: moving a third one to bite fails. While a module holds data this version cannot read, a change that touches an arch fails. When an arch changes, the world frame is reset and what the modules computed from those scans is cleared (module_cleared). Output: scan_id, source_name, from, to, role_source, swapped (the scan that took the other role, or null), world_frame_reset, module_cleared.
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

# Used by the messages below: reads their $role (upper, lower, bite).
session-role = { $role ->
    [upper] upper arch
    [lower] lower arch
   *[bite] bite
}

session-import-error-path-or-files = Give exactly one of path (one file) or files (a list of files).
session-import-error-no-files = The list of files is empty: give at least one file.
session-import-nothing = No scan was imported. { $reasons }

session-set-role-error-bad-id = { $id } is not a valid scan id.
session-set-role-error-no-scan = The case has no scan with the role { session-role }.
session-set-role-error-needs-scan-id = The case has more than one scan with the role { session-role }: say which one with scan_id.
session-set-role-error-wrong-scan = The scan { $id } does not have the role { session-role }.
session-set-role-error-bite-full = A case holds at most { $max } bite scans: change the role of one of them first.
