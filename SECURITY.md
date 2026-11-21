# Security policy

## Reporting a vulnerability

Please report security problems **privately**. Do not open a public issue or pull request.

1. Use GitHub private vulnerability reporting: open <https://github.com/dnviti/dentalcraft-core/security/advisories/new> (Security tab, "Report a vulnerability"). Or
2. Send an email to hello@dentalcraft.app with the subject `Security: dentalcraft-core`.

Please include the version (or commit), your operating system, steps to reproduce, and what you think the impact is. **Never send real patient data, scans or project files.** If a sample is needed, make a synthetic one.

This is a one-person project. Reports are handled on a best-effort basis, with no guaranteed response time. Please give a reasonable time to fix a problem before you disclose it in public. You will be credited in the fix unless you prefer not to be.

## Scope

In scope: the engine crates in this repository (project file parsing, mesh readers, scanner and Bluetooth drivers, the MCP server).

Out of scope: the closed DentalCraft account server, the website, and third-party software and services. A problem in a dependency should be reported to its own project.

## Supported versions

DentalCraft is in alpha. Only the latest release receives fixes.

## Medical-device disclaimer

DentalCraft is **not a certified medical device**. It has no regulatory clearance and has not been validated for clinical use. Do not use it on patients, or to make clinical decisions, without independent professional validation. The software is provided as is, without warranty, as the [licence](LICENSE.md) says. A flaw in the output of an algorithm is a bug to report as an issue; a flaw that exposes data, runs unwanted code or bypasses a check is a security problem to report as described above.
