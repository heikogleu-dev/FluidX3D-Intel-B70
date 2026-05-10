# Contributing

Thanks for considering a contribution! This repo is a fork of
[ProjectPhysX/FluidX3D](https://github.com/ProjectPhysX/FluidX3D)
focused on real-world testing and patching of the FluidX3D Lattice
Boltzmann solver on **Intel Arc Pro B70 (Battlemage / xe driver)**.
Contributions are very welcome — particularly:

## High-Value Contributions

- **Reproducing a benchmark on different hardware** — Arc A-series
  (A770, A750), other Battlemage variants (B580, B570, B60), or other
  OpenCL devices. Open an issue with your numbers so we can compare.
- **Reporting a different result on B70** — kernel / driver / oneAPI /
  BIOS differences are interesting data points. Please include `dmesg`
  excerpts (especially `xe` lines) and `clinfo -l` output.
- **Workarounds for the xe-driver shutdown race** — if you find a
  cleaner alternative to `_exit(0)` that doesn't require skipping C++
  destructors, please share it.
- **Performance regressions or improvements** when a new oneAPI /
  Intel Compute Runtime / xe-driver release changes the picture.
- **Setup variants** — different vehicle STLs, Reynolds numbers,
  resolutions, FP modes (FP16S vs FP16C vs FP32) — open an issue or PR
  with your setup file and the achieved MLUPS / GB/s.
- **Doc fixes** — typos, clarifications, missing prerequisites in the
  build instructions.

## How to Contribute

1. **Open an issue first** for non-trivial changes — it's faster to
   align on direction than to rework a large PR.
2. Fork → branch → edit → push → open a PR against `master`.
3. Reference the relevant section of `MODIFICATIONS.md` or `README.md`
   in your PR description.
4. Include reproduction steps for any new benchmark / measurement.
5. **Mark your modified files** with a marker comment (per FluidX3D
   license clause 1) — see how it's done in the existing patches at
   `src/defines.hpp`, `src/graphics.cpp`, etc.

## Documentation Style

- Real numbers, not "feels faster". Steady-state mean of multiple runs.
- Include hardware + software versions: kernel, xe-driver, oneAPI,
  Intel Compute Runtime, FluidX3D commit.
- Console output + `dmesg` / `journalctl -k` context for crashes.
- Honest verdicts — negative results are valuable.
- For graphics / visualisation issues: include a screenshot.

## License

This repository is a fork of [ProjectPhysX/FluidX3D](https://github.com/ProjectPhysX/FluidX3D)
and inherits its license (see [`LICENSE.md`](LICENSE.md)). Specifically:

- **Non-commercial use only.** Contributions that introduce commercial
  features, paid plugins, or are made on behalf of a commercial product
  cannot be accepted.
- **No military / defence use.** Contributions targeting military
  applications cannot be accepted.
- **No AI training.** Contributions that build pipelines for training
  models on the source code are out of scope.
- **Attribution.** All modified files must remain "plainly marked as
  such" — keep the marker comment at the top, update `MODIFICATIONS.md`,
  and use clear per-feature commits.

By opening a PR you confirm:

1. Your contribution is your own work (or you have the right to submit it).
2. Your contribution is licensed under the same terms as the upstream
   FluidX3D license — see [`LICENSE.md`](LICENSE.md).
3. You agree that your contribution may be redistributed under the
   FluidX3D license as part of this fork.

## Upstream interaction

If your patch is general-purpose (i.e. not B70 / xe-driver / Linux
specific), please also consider sending it to the upstream
[ProjectPhysX/FluidX3D](https://github.com/ProjectPhysX/FluidX3D) so
that all FluidX3D users benefit. This fork's purpose is to document
hardware-specific issues — general improvements belong upstream.
