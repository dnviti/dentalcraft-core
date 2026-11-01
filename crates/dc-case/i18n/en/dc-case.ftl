# Scan import pipeline (dc-case): warnings, errors and progress, shared by the
# app, the CLI and the MCP server.

case-import-progress-reading = Reading scans

# Referenced by the messages below: reads their $role (upper, lower, bite).
case-import-role = { $role ->
    [upper] upper arch
    [lower] lower arch
   *[bite] bite
}

case-import-warn-rescaled = { $name }: rescaled × { $factor } to millimetres (the file was in { $units }).
case-import-warn-units-suspect = { $name }: the size looks like { $units }, but bite and follow-up scans are never rescaled: it was read as millimetres. Check the file.
case-import-warn-tiny = { $name }: the model measures only { $size } mm. Check that the file is in millimetres.
case-import-warn-low-polygons = { $name }: only { $faces } faces, too few for a clinical scan. Check that this is the right file.
case-import-warn-obj-objects-merged = { $name }: the { $objects } objects of the OBJ file were merged into one scan.
case-import-warn-bad-faces-removed = { $name }: faces removed: { $degenerate } degenerate, { $duplicate } duplicate.
case-import-warn-replaced = { $name } replaces { $old } as { case-import-role }.
case-import-warn-arch-conflict = The scan { $name } looks like the other arch, but that place is taken: it was placed as { case-import-role }. Arch conflict, please check.
case-import-warn-provisional = { $name }: the arch could not be recognised, so it is placed as { case-import-role } for now. Please check.
case-import-warn-provisional-moved = { $name } was placed provisionally: it is now { case-import-role }, to make room for the new scan.
case-import-warn-bite-limit-replaced = A case holds at most { $max } bite scans: { $name } replaces { $old }.
case-import-warn-bite-limit-skipped = A case holds at most { $max } bite scans: { $name } was not imported.

case-import-error-cancelled = Import cancelled.
case-import-error-unsupported = { $name }: unsupported file format. Use STL, OBJ or PLY.
case-import-error-unreadable = { $name }: the file cannot be read.
case-import-error-invalid = { $name }: the file is damaged or is not a valid scan.
case-import-error-empty = { $name }: the file contains no usable faces.
case-import-error-needs-role = { $name }: both arches are already taken. Choose its role in the Import scans panel, under Role of the next scans, and import it again (a command takes it in the role parameter).
case-import-error-role-clash = { $name }: more than one file of this import is meant as { case-import-role }. Say which file goes there.
case-import-error-arches-locked = This project holds data of the { $module } module that this version cannot read, so the arches cannot be replaced or moved: update DentalCraft. A scan can still go on a free arch, and the bite scans can change.
