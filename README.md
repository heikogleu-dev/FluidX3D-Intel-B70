# FluidX3D — Intel Arc Pro B70 (Battlemage) Fork

**Pioneer documentation: FluidX3D 3.6 LBM solver verified at 99.5 % peak bandwidth on Intel Arc Pro B70 Pro (BMG-G31, full Battlemage, xe driver, oneAPI OpenCL 26.05). 4× faster than the RTX 3060 Ti reference. Includes Linux/xe-driver shutdown-crash workaround, HiDPI font, windowed mode with env-var control, FORCE_FIELD enabled with VTK + CSV export of solid-boundary forces.**

This is a fork of [ProjectPhysX/FluidX3D](https://github.com/ProjectPhysX/FluidX3D). For the original project documentation see [README_UPSTREAM.md](README_UPSTREAM.md). All changes vs upstream are tracked file-by-file in [MODIFICATIONS.md](MODIFICATIONS.md). License is **unchanged** — see [LICENSE.md](LICENSE.md), non-commercial / non-military use only.

---

## Status

| Aspect | Status |
|---|---|
| Compute (LBM kernels) | ✅ **4 917 MLUPS @ 16.85 M cells, 605 GB/s = 99.5 % of B70 spec (608 GB/s)** |
| Allocation / memory layout | ✅ 56.6 B/cell empirical (D3Q19 + FP16C), matches theory; max ~449 M cells fit in B70's 28.6 GB |
| FP16C precision (DDFs) | ✅ stable, no observed divergence over 10 000 steps |
| `FORCE_FIELD` extension | ✅ enabled, force-field on TYPE_S boundaries written as VTK + CSV |
| Build (Linux + X11 + OpenCL ICD) | ✅ clean compile in 10 s with GCC 15.2 |
| Linux x11 windowed mode | ✅ default 2560×1440, env-var configurable |
| Linux xe-driver clean shutdown | ⚠️ requires `_exit(0)` workaround (see below) |
| Live visualisation FPS | ⚠️ 0–1 FPS at large window — FluidX3D's CPU software renderer is the bottleneck, **not** the GPU |
| Multi-GPU on B70 | ❓ untested (single-tile B70) |

## Hardware target

- **GPU:** Intel Arc Pro B70 (Pro variant), BMG-G31, 32 Xe2-HPG cores, 32 RT units, 256 XMX engines
- **VRAM:** 32 GB ECC GDDR6, 256-bit, 608 GB/s spec
- **Driver:** Linux `xe` (not `i915`), tested on `26.05.037020`
- **OpenCL exposes:** 28 567 MB usable VRAM, max single-buffer alloc 4 894 MB
- **Compute units (FluidX3D self-report):** 256 @ 2 800 MHz = 4 096 cores, 22.94 TFLOPs/s FP32

For raw clinfo / lspci snapshots see [MODIFICATIONS.md § Verified performance](MODIFICATIONS.md#verified-performance-post-modification).

## What's modified vs upstream

Quick summary (full detail in [`MODIFICATIONS.md`](MODIFICATIONS.md)):

- **`src/defines.hpp`** — enable `VOLUME_FORCE` + `FORCE_FIELD`
- **`src/graphics.hpp`** — `FONT_WIDTH 6→12`, `FONT_HEIGHT 11→22` (HiDPI)
- **`src/graphics.cpp`** — auto-start (`key_P = true`), 2× pixel-doubled `draw_text()`, windowed-mode override with `FLUIDX3D_WINDOW=WxH`, decorated X11 window
- **`src/setup.cpp`** — half-domain test grid 650×144×180 (16.85 M cells), `lbm.run(10000u)`, VTK + force-CSV export, `_exit(0)` workaround for xe-driver shutdown race
- **`README.md`** → renamed to `README_UPSTREAM.md`; this file replaces it.

Per-file: top-of-file marker comment points at `MODIFICATIONS.md`. Per-feature: separate commits on `master`.

## Build

```bash
git clone https://github.com/heikogleu-dev/FluidX3D-Intel-B70.git
cd FluidX3D-Intel-B70
mkdir -p export                        # run-time outputs go here
ln -s /path/to/your/scenes scenes      # provide your own STLs (vehicle.stl etc.)
./make.sh                              # builds AND runs (default behaviour)
# OR manual build only:
g++ src/*.cpp -o bin/FluidX3D -std=c++17 -pthread -O -Wno-comment \
    -I./src/OpenCL/include -L./src/OpenCL/lib -lOpenCL \
    -I./src/X11/include -L./src/X11/lib -lX11 -lXrandr
```

Required system packages on Ubuntu / oneAPI stack:
```
intel-opencl-icd  intel-igc-opencl-2  ocl-icd-opencl-dev  opencl-c-headers
libx11-dev  libxrandr-dev  build-essential
```

## Run

```bash
./bin/FluidX3D                          # auto-fastest device, 2560×1440 windowed, auto-start
./bin/FluidX3D 1                        # explicit B70 (device id depends on platform order; 1 here)
FLUIDX3D_WINDOW=1920x1080 ./bin/FluidX3D
FLUIDX3D_WINDOW=fullscreen ./bin/FluidX3D
```

Outputs land under `export/`:
- `u-<step>.vtk`, `rho-<step>.vtk`, `flags-<step>.vtk`, `F-<step>.vtk` — STRUCTURED_POINTS, scalar field name `data` in ParaView (NB: not `rho`/`u`/`F`)
- `mesh-<step>.vtk` — the input STL as VTK
- `forces_solid_cells.csv` — `step,x,y,z,Fx_lbm,Fy_lbm,Fz_lbm,Fx_SI,Fy_SI,Fz_SI` per non-zero force on TYPE_S cells

### Quick force aggregation

```bash
awk -F, 'NR>1 && $4>0 && $4<179 && $3<143 {fx+=$8; fy+=$9; fz+=$10}
         END {printf "Vehicle (half-domain): Fx=%.1f Fy=%.1f Fz=%.1f N\n", fx, fy, fz}' \
    export/forces_solid_cells.csv
```
Multiply Fx, Fz by 2 for the symmetry-corrected full-vehicle drag/lift; Fy → 0 by symmetry.

### Pressure from `rho` in ParaView

LBM has no separate pressure field. Use a `Calculator` filter on `rho-<step>.vtk`:
```
result = (data - 1) * 65333    # → Pa, valid for the half-domain setup (650×144×180, 30 m/s, lbm_u=0.075)
```
Constant 65333 = `rho_air × c_phys² / 3` with `c_phys = cell_size / dt = 6.92 mm / 17.3 µs = 400 m/s`. Recompute for other grid/velocity choices.

## Crash workaround — xe-driver shutdown race

On Linux with the `xe` driver (B70), unmodified FluidX3D crashes with `SIGSEGV` after a finite `lbm.run(N)` returns and exports run. Kernel log signature:

```
xe 0000:04:00.0: [drm] Tile0: GT0: Timedout job: seqno=…, guc_id=…, flags=0x20 in FluidX3D
xe 0000:04:00.0: [drm] Tile0: GT0: Fault response: Unsuccessful -EINVAL    (×10)
xe 0000:04:00.0: [drm] Xe device coredump has been created
```

Userspace then segfaults during C++ destructor cleanup with either `double free or corruption (out)` or `libc++abi: Pure virtual function called!`. Data flushed before this point (VTK / CSV) is **intact** — the failure is during teardown only.

**Workaround:** call `_exit(0)` immediately after the last export. This skips C++ destructors and atexit handlers, terminating the process before the cleanup sequence the driver chokes on. Implemented in `src/setup.cpp` after the VTK/CSV block.

To re-test if a future driver release fixes the issue, comment out the `_exit(0)` line and watch `journalctl -k --since "1 min ago" | grep xe`.

## Performance baseline

| Test | Grid | FP | Steps | MLUPS | GB/s | Steps/s | Wall |
|---|---|---|---:|---:|---:|---:|---:|
| Smoke (CC#2) | 650 × 144 × 180 | FP16C | 100 | 3 342 | 411 | 198 | < 1 s + init |
| Throughput (CC#3) | 650 × 144 × 180 | FP16C | 10 000 | **4 917** | **605** | 292 | ~30 s + init |
| Full domain (CC#1, interrupted at step 10 585) | 1299 × 288 × 361 | FP16C | n/a | 5 331 | 592 | 39 | n/a |

The 16.85 M-cell run sustains **99.5 % of the 608 GB/s spec bandwidth** — confirming LBM is bandwidth-limited and the B70 saturates its GDDR6 subsystem. ~4× the effective LBM bandwidth of an RTX 3060 Ti reference (~150 GB/s effective).

## Companion repos

- **Paraview / OSPRay / B70 ray-tracing** — [heikogleu-dev/Paraview---Intel-B70-Pro-OSPRAY-Raytracing-Pathtracing](https://github.com/heikogleu-dev/Paraview---Intel-B70-Pro-OSPRAY-Raytracing-Pathtracing)
- **OpenFOAM v2512 + PETSc + Kokkos + SYCL** — [heikogleu-dev/Openfoam-v2512-Petsc-Kokkos-Sycl-Intel-B70](https://github.com/heikogleu-dev/Openfoam-v2512-Petsc-Kokkos-Sycl-Intel-B70)
- **OpenFOAM 13 GPU offloading (Ginkgo SYCL)** — [heikogleu-dev/Openfoam13---GPU-Offloading-Intel-B70-Pro](https://github.com/heikogleu-dev/Openfoam13---GPU-Offloading-Intel-B70-Pro)

## License & attribution

Original FluidX3D © 2022–2026 Dr. Moritz Lehmann. License unchanged from upstream — see [`LICENSE.md`](LICENSE.md). Restrictions (paraphrased; the full text is binding):

1. Origin must not be misrepresented; altered source versions must be plainly marked as such ✓ (this README, `MODIFICATIONS.md`, per-file headers, commit history)
2. **Non-commercial use only**
3. **No military / defence use**
4. No AI training on the source
5. If altered binaries / data are published, altered source must be too ✓ (this repo)
6. Cite the FluidX3D references in scientific publications
7. License notice may not be removed ✓ (LICENSE.md preserved unchanged)

The trademark "FluidX3D" is protected by German Werktitelschutz (§ 5 Abs. 3 MarkenG). This fork uses the name in compliance with clause 1 — the project is plainly identified as a derivative work.
