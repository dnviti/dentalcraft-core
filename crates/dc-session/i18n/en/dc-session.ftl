# Built-in commands of the headless session (dc-session): labels and docs
# shown by the CLI and the MCP server.

session-cmd-project-new = New project
session-cmd-project-new-doc = Starts a new empty project (unsaved). Parameters: patient_code (optional pseudonymised code). Clears the undo history and the step history.
session-cmd-project-open = Open project
session-cmd-project-open-doc = Opens a .dcprj project file. Parameters: path. Damaged or newer files open read-only; the load issues are returned.
session-cmd-project-save = Save project
session-cmd-project-save-doc = Saves the project as .dcprj (atomic write, one .bak kept). Parameters: path (optional; defaults to the open file, the .dcprj extension is added when missing).
session-cmd-case-import = Import scan
session-cmd-case-import-doc = Imports STL, OBJ or PLY scans into the case, as one undoable step. Arches are converted to millimetres; a bite scan, known from its role or its file name, stays as it is in the file, with a units_suspect warning when its size fits another unit. Parameters: path (one file) or files (a list of objects with path and an optional role), exactly one of the two; role (auto, upper, lower or bite). With auto or no role the import decides: the file name, otherwise a free arch, marked provisional because it is a guess (a module may offer its own import-scans command that also recognises the arch from the geometry). With files, role is the default for the entries without their own role. A scan with a role or a name hint replaces the one with the same role, except that a provisional scan moves to the free arch instead; a scan with neither never replaces one and fails with needs_role when no arch is free. An upper or lower role given to two or more files fails with role_clash for all of them. A case holds at most 2 bite scans. While a module holds data this version cannot read, a file that would replace or move an arch fails with arches_locked; free arches and bite scans still work. Output: scan_id, role, role_source (explicit, file_name, detected, detected_unsure or provisional), source_name, source_units, mesh, vertices, faces and replaced describe the first imported scan; imported lists every imported scan with the same fields plus path; warnings (code, key, args, message), failed (path, code, reason, detail), moved, cleanup (faces removed) and module_cleared report the rest. A batch with some failed files still succeeds, with failed filled in; when nothing is imported the command fails with the reasons and the case is unchanged.
session-cmd-case-set-role = Set scan role
session-cmd-case-set-role-doc = Changes the role of a scan, as one undoable step. Parameters: from (upper, lower or bite: the role the scan has now), to (the role it gets), scan_id (only needed when from is bite and the case has two bite scans). The role becomes explicit; with from equal to to nothing else changes, so it confirms a role that was only a guess, including detected_unsure (recognised from the geometry, but worth checking). A scan already on the target arch takes the from role, so the two swap. A case holds at most 2 bite scans: moving a third one to bite fails. While a module holds data this version cannot read, a change that touches an arch fails. When an arch changes, the world frame is reset and what the modules computed from those scans is cleared (module_cleared). Output: scan_id, source_name, from, to, role_source, swapped (the scan that took the other role, or null), world_frame_reset, module_cleared.
session-cmd-edit-undo = Undo
session-cmd-edit-undo-doc = Reverts the last change to the case.
session-cmd-edit-redo = Redo
session-cmd-edit-redo-doc = Re-applies the last undone change.
session-cmd-steps-list = List steps
session-cmd-steps-list-doc = Lists the processing step records and, when a pipeline is known, the state of every step.
session-cmd-steps-rewind = Go back to step
session-cmd-steps-rewind-doc = Goes back to the state right before a processing step: the records of that step and of the following ones are removed. Parameters: step.
session-cmd-view-set = Set view
session-cmd-view-set-doc = Stores view parameters for the renderer (e.g. preset, width, height, visible, model scan|teeth, stage). Keys set to null are removed. Does not change the case.

# 3D scanner commands (scan.*), alpha.
scan-cmd-devices = List scanners
scan-cmd-devices-doc = Lists the connected 3D scanners of every backend (backend, id, name, serial) and the state of each backend (available, with an error_code when it cannot be used, e.g. sdk-missing). The simulated scanner backend sim is always available.
scan-cmd-connect = Connect scanner
scan-cmd-connect-doc = Connects a scanner. Parameters (all optional): backend (revopoint-usb: a Revopoint MINI 2 on USB, Linux, no SDK needed; revopoint-sdk: through the Revopoint SDK installed by the user; sim: the simulated scanner), device (id from the list of scanners). Without them the first hardware scanner found is used, else the simulated upper typodont. options (sim backend only) tunes the simulated device: realtime, frames (stop after this many), width, height, fps, noise_mm, dropout, period_frames, seed.
scan-cmd-disconnect = Disconnect scanner
scan-cmd-disconnect-doc = Ends any acquisition, discards the live scan and closes the scanner. The scan already stopped and edited stays available.
scan-cmd-start = Start scanning
scan-cmd-start-doc = Starts the acquisition, or continues the stopped scan. Frames are tracked and fused into the live point cloud.
scan-cmd-pause = Pause scanning
scan-cmd-pause-doc = Stops fusing frames; tracking continues, so the scanner can be repositioned and the scan resumed.
scan-cmd-resume = Resume scanning
scan-cmd-resume-doc = Resumes fusing frames after a pause.
scan-cmd-stop = Stop scanning
scan-cmd-stop-doc = Ends the acquisition. The fused point cloud becomes the scan to clean up, mesh and import. Returns the number of points.
scan-cmd-reset = Reset scan
scan-cmd-reset-doc = Discards the acquired scan and its edits; the scanner stays connected.
scan-cmd-status = Scan status
scan-cmd-status-doc = Returns the acquisition state (state, device, frames, fused_frames, tracking_lost, tracking_ok, points, fps, error, world_angles_deg: the turntable rotate and tilt angles when the world frame was set; world_from_camera: the scan world frame from the camera of the first fused frame, column-major 3x4; rig_world: the world is the calibrated rig frame, +Z the turntable axis up from the platter, origin on the platter, +X toward the scanner, otherwise +Z is the true up measured by the scanner's IMU when imu_world, else the camera's up; imu_world: the up came from the scanner's IMU, oriented by a stored calibration of this scanner; scanner_deg: the scanner tilt, positive looking down, and yaw in that frame), auto (progress of the latest automatic turntable scan: state, step, steps, tilt_deg, error, or null) and, once stopped, the editable scan (points, mesh_vertices, mesh_faces, can_undo, can_redo).
scan-cmd-settings = Scan settings
scan-cmd-settings-doc = Sets and returns the acquisition settings. Parameters (all optional, applied when the scan is not running): live_voxel_mm (0.05 to 5), min_depth_mm, max_depth_mm, exposure_us (0 for automatic exposure), roi_radius_mm (5 to 200, default 45) and roi_height_mm (5 to 300, default 60): the cylinder around the turntable axis that the frames are cropped to when a turntable calibration is in use, use_roi (default true).
scan-cmd-cloud = Edit point cloud
scan-cmd-cloud-doc = Applies one edit to the point cloud of the stopped scan, undoable with scan.undo. Parameter op is an object with the field op: downsample (voxel_mm), remove-statistical (k, std_ratio), remove-radius (radius_mm, min_neighbors), isolate (radius_mm), smooth (k, iterations, strength), delete-aabb (min, max), delete-sphere (center, radius_mm), delete-plane (point, normal), delete-indices (indices), estimate-normals (k). A cloud edit drops the mesh.
scan-cmd-mesh-build = Build mesh
scan-cmd-mesh-build-doc = Builds the mesh of the stopped scan. Parameter: resolution_mm (0.05 to 5, the smaller the finer and heavier). A scan that kept its stops (automatic stop-and-capture) and whose cloud is unedited is rebuilt from the stops, like scan.reconstruct (0.05 to 1 mm); otherwise the mesh is built from the point cloud. Returns the vertex and face counts and method (stops or cloud).
scan-cmd-reconstruct = Rebuild fine mesh
scan-cmd-reconstruct-doc = Rebuilds the mesh of the stopped scan from the stops an automatic stop-and-capture scan kept, at the resolution of the scanner, as an undoable step: the poses of the stops are refined against each other, then the stops are fused into a fine volume and meshed. Parameters (optional): resolution_mm (0.05 to 1, default 0.1), refine (default true). Returns the vertex and face counts and the report under reconstruction. Fails with scan-no-stops for hand-held and continuous scans.
scan-cmd-mesh = Repair mesh
scan-cmd-mesh-doc = Applies one repair to the mesh, undoable with scan.undo. Parameter op is an object with the field op: fill-holes (max_edges), smooth (iterations, lambda, mu), remove-islands (min_fraction), decimate (ratio), finish (min_island_fraction, default 1: the main component only; max_error_mm, default 0.015): cleans the mesh, closes every hole except the open base and optimises it within max_error_mm in one step, with its report under finish.
scan-cmd-undo = Undo scan edit
scan-cmd-undo-doc = Reverts the last cloud or mesh edit of the scan. Does not touch the case.
scan-cmd-redo = Redo scan edit
scan-cmd-redo-doc = Re-applies the last undone cloud or mesh edit of the scan.
scan-cmd-export = Export scan
scan-cmd-export-doc = Writes the scan. Parameters: path, what (cloud: PLY point cloud; mesh: STL, PLY or OBJ by the extension of path, PLY for any other; stops: the stops kept by an automatic stop-and-capture scan, into the directory path, stops.json and one depth file per stop, for offline analysis).
scan-cmd-import = Import scan into the case
scan-cmd-import-doc = Imports the scan mesh into the case as an undoable step, like an imported file. Parameter: role (upper, lower or bite; optional). Uses the import command of the active module when it has one, else case.import. Needs a mesh: build it first.
scan-cmd-auto = Automatic turntable scan
scan-cmd-auto-doc = Scans the object on the turntable automatically, in the background. Needs a connected scanner and turntable and a turntable calibration matching them (turntable.calibrate): the frames are cropped to the turntable and the table angles guide the tracking. Without a calibration it fails with rig-uncalibrated, unless allow_uncalibrated is true (continuous only, nothing cropped: use it only when the background is beyond the depth range). With a calibration it stops and captures (mode stop-and-capture): for each tilt the table tilts, then stops every step_deg over rotation_deg and fuses the median of frames_per_stop frames at each stop; mode continuous turns without stopping. Parameters (all optional): tilts_deg (default [-20, -10, 0, 10, 20]: within ±20 an object only resting on the platter stays put), rotation_deg (default 360), mode, step_deg (default 7.5), frames_per_stop (default 3), deg_per_s, settle_s (default 0.5), allow_uncalibrated (default false), finish (default true: at the end the acquisition stops and the mesh is rebuilt from the stops and finished, cleaned, closed and optimised, ready in the scan editor), resolution_mm (resolution of that mesh, 0.05 to 1, default 0.1), refine (refine the poses of the stops first and leave out the views that disagree with the others, an object that moved on the platter, so they never make double walls; the reconstruction then reports rejected_stops, rejected_passes and the warning scan-views-rejected; default true). Returns at once with started and steps; follow it with scan.status (auto: phase, reconstruction, finish). During the finish the acquisition is over: scan.status state is stopped (no frame is tracked, the scanner stream and its fill light are off), turntable.status busy is false, and auto stays running with phase refine, fuse, extract, clean, fill-holes, then optimise. scan.pause and scan.resume pause and resume it, scan.stop ends it; with finish false the scan stays paused when it is done, ready for scan.stop. When the Bluetooth link to the turntable drops, the scan reconnects it (up to 30 s) and goes on at the stop where it was (auto: reconnecting, reconnects); when it does not come back the scan fails with ble-disconnected. A stop-and-capture scan that fails or is cancelled keeps its stops: the scan is stopped, auto kept_stops tells how many, and scan.reconstruct and scan.export (stops) work on them.

# Turntable commands (turntable.*), alpha.
turntable-cmd-devices = List turntables
turntable-cmd-devices-doc = Searches for turntables over Bluetooth for timeout_s seconds (0 to 30, default 3; 0 skips the search) and lists them (backend, id, name) with the state of each backend (available, with an error_code when it cannot be used, e.g. ble-no-adapter). The simulated turntable of the sim backend is always listed.
turntable-cmd-connect = Connect turntable
turntable-cmd-connect-doc = Connects a turntable. Parameters (all optional): backend (revopoint-ble: the Revopoint Dual-axis Turntable over Bluetooth; sim: the simulated turntable, which the simulated scanner sees turning), device (id from the list of turntables). Without them the first turntable found over Bluetooth is used, else the simulated one. options (sim backend only): fast (complete motions at once when no simulated scanner paces them). Returns the turntable, its axes and whether a matching calibration is in use.
turntable-cmd-disconnect = Disconnect turntable
turntable-cmd-disconnect-doc = Stops any automatic scan or calibration and closes the turntable. The region of interest and the motion prior of the calibration stop applying.
turntable-cmd-move = Move turntable
turntable-cmd-move-doc = Moves one axis of the turntable. Parameters: axis (rotate or tilt), and exactly one of by_deg (relative, signed) and to_deg (absolute; the tilt range is usually -30 to 30). Returns once the table accepted the command; busy in the status stays true until the motion ends. Refused while an automatic scan or a calibration runs.
turntable-cmd-speed = Turntable speed
turntable-cmd-speed-doc = Sets the speed of one axis. Parameters: axis (rotate or tilt), deg_per_s (degrees per second; the table clamps it to its range, the Revopoint table rotates at 4 to 20 degrees per second).
turntable-cmd-stop = Stop turntable
turntable-cmd-stop-doc = Stops every axis at once, and any automatic scan or calibration.
turntable-cmd-home = Turntable to zero
turntable-cmd-home-doc = Returns the rotation and the tilt to zero. Returns once the table accepted the command.
turntable-cmd-status = Turntable status
turntable-cmd-status-doc = Returns connected (false once the link to the table dropped, with info still set: a running job reconnects it, otherwise connect again), info, axes (kind, range, feedback), rotate_deg and tilt_deg (estimated from the commands and the readings), busy (a motion or a job is under way), calibrated (a calibration matching this scanner and this turntable is in use) and calibration_job (progress of the latest calibration: state, step, steps, error, reconnecting, reconnects).
turntable-cmd-calibrate = Calibrate turntable
turntable-cmd-calibrate-doc = Measures the turntable axes as seen by the fixed scanner, without tools: put a static object on the platter, connect the scanner and the turntable, and do not move them. Runs in the background: the table first turns once to learn the static background, then turns again in steps while the scanner tracks the object, then tilts both ways; the axes are fitted to the tracked poses and saved. Parameters (all optional): steps (3 to 36, default 8), step_deg (5 to 120, default 45; 180 to 1080 degrees in all). Returns at once; follow it with turntable.status (calibration_job), then read it with turntable.calibration. The scan acquired during the calibration is discarded. When the Bluetooth link to the turntable drops, the calibration reconnects it (up to 30 s) and goes on at the stop where it was.
turntable-cmd-calibration = Turntable calibration
turntable-cmd-calibration-doc = Returns the stored turntable calibration, or null: axis_point, axis_dir, platter_up, tilt_axis_point, tilt_axis_dir (scanner camera frame, millimetres), scanner_serial, turntable_id, created, fit_rms_deg, fit_rms_mm.

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
