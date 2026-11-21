# Contributing to dentalcraft-core

Thank you for your interest. This is a small project with one maintainer, so please keep changes focused and talk before you do something big.

Before you start, read the [README](README.md), the [Code of Conduct](CODE_OF_CONDUCT.md) and the [licence](LICENSE.md).

## Before you write code

- Open an issue first for anything larger than a small fix, so the direction is agreed before you spend time.
- Do not add new dependencies unless the change needs them.
- **Never commit patient data**: no real scans, no `.dcprj` files from real cases, no Teeth3DS or other third-party datasets. Test data must be synthetic.
- DentalCraft is not a certified medical device. Do not present a change as clinically validated.

## Licence of your contribution

This project is source-available under the [PolyForm Noncommercial License 1.0.0](LICENSE.md). By sending a contribution (a pull request, a patch, or any other code, text or data) you agree to the following:

1. You wrote it, or you have the right to submit it under these terms (see the sign-off below).
2. You keep the copyright in your contribution.
3. You grant Daniele Viti a perpetual, worldwide, non-exclusive, royalty-free, irrevocable licence to use, copy, modify, distribute and sublicense your contribution, as part of this project or on its own, **under any licence terms, including commercial ones**. This lets the author relicense the project, for example to offer commercial licences.
4. Your contribution is also available to everyone under the PolyForm Noncommercial License 1.0.0, like the rest of the project.

This text is a plain-language agreement, not legal advice, and it has not been reviewed by a lawyer. If you cannot agree to it, do not send a contribution.

## Sign-off (Developer Certificate of Origin)

Every commit needs a `Signed-off-by` line with your real name and email. It says you accept the [Developer Certificate of Origin 1.1](https://developercertificate.org/). Git adds it for you:

```sh
git commit -s -m "fix: describe the change"
```

A pull request with unsigned commits is not merged.

## Development loop

This repository depends on no other DentalCraft repository. `dentalcraft-aligners` and `dentalcraft-app` pin it by tag.

Install the Rust toolchain pinned in `rust-toolchain.toml` (`rustup` does it on first use). No C++ toolchain is needed.

Run the same checks before you open a pull request:

```sh
cargo fmt --check
cargo clippy --all-targets -- -D warnings
cargo test
```

All three must pass. Tests run headless, without a GPU.

To change several repositories together, work in the [`dentalcraft`](https://github.com/dnviti/dentalcraft) superproject. Its `.cargo/config.toml` redirects the git dependencies on core and aligners to the working trees, so a change in core is visible in aligners and app without a release. Cargo then rewrites `Cargo.lock` with path sources: never commit that. Run `git checkout -- Cargo.lock` first.

## Branches, commits and pull requests

- Work on a branch named `feat/...` or `fix/...`, and open a pull request against `main`.
- Use [Conventional Commits](https://www.conventionalcommits.org/): `feat:`, `fix:`, `perf:`, `docs:`, `test:`, `refactor:`, `chore:`. The pull request is squash-merged and its title becomes the commit message, so the title must follow the same format.
- Release versions, `CHANGELOG.md`, `.release-please-manifest.json` and tags are made by [release-please](https://github.com/googleapis/release-please). Do not edit them by hand.
- Do not change `.github/workflows/*` or the release configuration in a pull request unless an issue asks for it.

## Rules every change follows

- **English and Italian.** Every user-visible string is a Fluent key with an English and an Italian text (`i18n/en/<crate>.ftl` and `i18n/it/<crate>.ftl`). Tests check that both languages have the same keys. Do not hard-code text.
- **Same behaviour on every platform.** A change must work the same on Linux, Windows and macOS. Linux is the reference platform. If a part is platform-specific (scanner, Bluetooth, files, rendering), the other platforms need their equivalent in the same pull request, or the pull request says plainly what is missing.
- **Laws are formal proofs.** When a rule must always hold, write it as a [Kani](https://github.com/model-checking/kani) proof harness in a `#[cfg(kani)] mod kani_proofs` next to the pure function that decides it. If the proof fails, fix the code; never weaken the law to make it pass. Core holds most of the Kani harnesses (tooth numbering, voxel packing, mesh decimation keys, project schema checks). The conventions and the list of current laws are in [`docs/kani.md`](https://github.com/dnviti/dentalcraft/blob/main/docs/kani.md) of the superproject. Kani runs locally and is not part of `cargo test`.
- **Agent-drivable.** A new user action is a registered command with an id, a parameter schema and documentation, so it works from the command line and over MCP as well as from the window.

## Reporting problems

- Bugs and ideas: open an issue. Describe what you did, what you expected and what happened. Do not attach patient data.
- Security problems: do not open a public issue. Follow [`SECURITY.md`](SECURITY.md).
