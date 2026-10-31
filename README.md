# dentalcraft-core

Engine of DentalCraft, a Rust workspace without application shell: meshes and geometry, case model, `.dcprj` project I/O, commands and undo, background jobs, the module API, scene description, i18n and theme. It is headless and testable without a GPU.

This repository is a submodule of the [`dentalcraft`](https://github.com/dnviti-personal-org/dentalcraft) superproject, which holds the plan, the architecture decisions and the pinned versions of all DentalCraft repositories. Clone the superproject with `--recurse-submodules` to work on the whole project.

- Visibility: private.
- Versioning: [release-please](https://github.com/googleapis/release-please) on `main` (Conventional Commits, tags `vX.Y.Z`).
- CI: GitHub Actions on the `golgari-dev` runner scale set.
