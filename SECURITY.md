# Security Policy

## Scope

This repository is a **fork** of [ProjectPhysX/FluidX3D](https://github.com/ProjectPhysX/FluidX3D)
with patches and documentation specific to the Intel Arc Pro B70
(Battlemage / xe driver). It contains a CFD solver source tree, build
scripts, and configuration files. No network services, no daemon, no
remote API. Direct security risks are limited to local execution risks
of running an LBM solver on user data.

## What Counts as a Security Issue

- A patch in `src/` that introduces a memory-safety bug not present
  upstream (buffer overflow, use-after-free in B70 patches, etc.)
- A documented configuration in `src/setup.cpp` or env-var (`FLUIDX3D_WINDOW`)
  that, if followed, creates a security risk on the user's machine
- Leaked credentials, tokens, or personal data in a committed file
- Malicious links in documentation
- Any modification that weakens the upstream FluidX3D's protections

## Reporting

Please **do not** open a public issue for security findings.

Instead, contact the maintainer directly via the email address listed
on the GitHub profile (https://github.com/heikogleu-dev), or use
GitHub's private security advisory mechanism on this repository:

→ Security tab → Report a vulnerability

You will receive an acknowledgement within 7 days.

## Out of Scope

- **Bugs in upstream FluidX3D** (not introduced by this fork's patches)
  should be reported to [ProjectPhysX/FluidX3D](https://github.com/ProjectPhysX/FluidX3D/issues)
- **Bugs in the Intel xe-driver / Compute Runtime / oneAPI** should be
  reported to the respective upstream projects (Intel Compute Runtime,
  freedesktop xe kernel, Intel oneAPI). The xe-driver `-EINVAL` cleanup
  race documented in this repo is one such upstream issue — we work
  around it but the fix belongs in xe.
- **Security of input STL files** — the user is responsible for the
  meshes they load. FluidX3D's STL parser is upstream code; we do not
  modify it.

If you are unsure where a bug belongs, open a regular issue and we
will help triage.

## Supported Versions

Only the current `master` branch is actively maintained. Older
commits / tags are kept for historical reference only.
