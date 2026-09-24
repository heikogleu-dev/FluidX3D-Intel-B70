# Summary of Modifications

MaxAttack CFD Bench is a modified version of **FluidX3D** by Dr. Moritz Lehmann
(<https://github.com/ProjectPhysX/FluidX3D>). This file summarises what was changed, as required by
`NOTICE.md`. It states facts taken from the code and the commit history. Performance and accuracy
figures appear only where they were measured on the development machine; nothing is extrapolated.

**Divergence from upstream** (`git diff upstream/master..HEAD`): 799 commits, 292 files changed,
54 595 insertions, 3 597 deletions. Within `src/` alone: 11 files, 20 007 insertions, 1 633
deletions — the bulk in `setup.cpp` (+12 959), `kernel.cpp` (+4 523) and `lbm.cpp` (+3 063).

**What was *not* changed:** `LICENSE.md` is byte-identical to upstream (verified by checksum). No
copyright or licence line has been removed from any source file. Internal identifiers — class names,
OpenCL kernel names, macros, file names under `src/`, build targets and the binary name `FluidX3D` —
are deliberately left untouched so that upstream changes can still be merged.

---

## 1. Multi-resolution near/far coupling

A second, coarser lattice covers the far field while the fine lattice resolves the vehicle. The two
are coupled each outer step; the near domain advances several steps per coarse step. Added
throughout `src/setup.cpp` and `src/lbm.cpp`, with the coupling cadence derived from the resolution
ratio rather than configured by hand. Documented in `DOPPEL-DOMAENE.md`.

Also in this area: a rescaling band between the domains, inlet/outlet treatment for the coarse
domain, and a grid-velocity convention for the moving ground. See `EINLASS-AUSLASS.md`,
`GITTERGESCHWINDIGKEIT.md`, `BAND-ARTEFAKT-8MM.md`.

## 2. Two-device topology

The fine domain runs on a discrete GPU, the coarse domain on an integrated GPU of the same machine,
with the far field held in system RAM rather than VRAM. This is *not* upstream's N-way domain
decomposition — it is a near/far split across two devices of different speed, and upstream's
multi-GPU path is not exercised by this fork. Device selection, per-device memory accounting and a
VRAM census read from `/proc/*/fdinfo` were added.

## 3. Wall-model chain

The largest addition. Upstream has no wall model; a vehicle case at practical resolution needs one.
Added, all under `src/kernel.cpp` and configured from `src/setup.cpp`:

- a **facet-based wall model** — sub-cell boundary geometry with per-facet wall shear from a
  Spalding profile, coupled to the cell through an interpolated momentum exchange;
- **interpolated bounce-back (ELIBB)** with a moving-least-squares blend for sub-cell wall distance;
- a **rank cascade** over the wall-link set with a symmetric rank-1 downdate, a pseudo-inverse
  branch, a least-squares fallback and an explicit rank-0 class;
- **subgrid-scale treatment** near the wall, including a Π-band formulation;
- a **wall-cell reconstruction** (`CFD_FAC_REK`) in Kupershtokh exact-difference form, currently
  under calibration and switched off by default.

Documented in `WANDMODELL.md`, `FACETTEN.md` and the plan files in the repository root.

## 4. Memory layout

Density stored in 16 bit instead of 32 bit in the perturbation representation, block tiling of the
collision kernel, and removal of buffers the vehicle case does not need. The device footprint is
**47 B per cell** against upstream's 93 B for FP32 — measured on this machine, from the run report.

## 5. Numerical safeguards

Velocity and density clamping with a budget accounting that reports how much the clamps moved the
solution, positivity handling at boundary cells, and hull checks on quantised fields. These exist
because clamping silently shifts forces; the accounting makes that visible instead of absorbing it.

## 6. Instrumentation and acceptance layer

Every added mechanism carries an **action-path counter** in a device-side counter buffer, read back
on the host and checked against an is = should acceptance at the end of a run. A switch without a
firing counter is treated as a hard error. Counters cover gate visits, histogram bins of the
quantities a mechanism depends on, and constant mirrors that prove a block reached the *compiled*
kernel rather than only the JIT text. The slot allocation is maintained in a single place
(`src/lbm.cpp`, at the buffer allocation).

## 7. Case setup, tooling and validation rigs

- Vehicle, channel, sphere and torus cases with an environment-variable configuration surface
  (`CFD_*`), and a base-line file that run lines are diffed against before a run.
- A locked run queue (`werkzeuge/lauf_queue.sh`): one runner, one chain, one watchdog, with a status
  file and a process census before and after each series.
- Force decomposition written per run (pressure, friction, per-zone), field hashing for
  bit-identity checks, slice export, and comparison tooling against an OpenFOAM 13 reference on the
  same STL.
- A separate D3Q27 build path (`werkzeuge/bau_q27.sh`).

## 8. User-visible naming

Console banner, graphics overlay and window title identify this build as MaxAttack CFD Bench and
name FluidX3D and its author as the origin. The upstream README is preserved verbatim as
`README_UPSTREAM.md`, including its reference list.

---

*Upstream FluidX3D is the faster and more general program. This fork narrows it to one case — a road
vehicle on one workstation — and adds what that case needs. For anything else, use upstream.*
