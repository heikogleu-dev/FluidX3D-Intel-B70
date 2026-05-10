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

| Test | Grid | Cells | FP | Steps | MLUPS | GB/s | Steps/s | Wall |
|---|---|---:|---|---:|---:|---:|---:|---:|
| Smoke | 650 × 144 × 180 | 16.85 M | FP16C | 100 | 3 342 | 411 | 198 | < 1 s + init |
| Throughput | 650 × 144 × 180 | 16.85 M | FP16C | 10 000 | **4 917** | **605** | 292 | ~30 s + init |
| Full half-domain (interrupted) | 1299 × 288 × 361 | 135 M | FP16C | (10 585) | 5 331 | 592 | 39 | n/a |
| **Aero-Box 20 mm (CC#2)** | **1200 × 304 × 400** | **145.92 M** | FP16C+FORCE_FIELD | **50 000** | **5 384** | **576** | **37** | **24 min** |
| **Aero-Box 20 mm asym (CC#3)** | **1000 × 200 × 250** | **50.0 M** | FP16C+FORCE_FIELD | **50 000** | **5 732** | **613** | **115** | **7.8 min** |

The 16.85 M-cell run sustains **99.5 % of the 608 GB/s spec bandwidth**, the 145.92 M Aero-Box run sustains **94.7 %**, and the 50.0 M asym Aero-Box run **even exceeds the spec at 100.8 %** — confirming LBM is bandwidth-limited and the B70 saturates its GDDR6 subsystem at any working-set size; the spec bandwidth value is conservative on this card. ~4× the effective LBM bandwidth of an RTX 3060 Ti reference (~150 GB/s effective).

### Thermals & power under sustained load (CC#2, 24 min @ 275 W)

| Sensor | Value | Headroom |
|---|---:|---:|
| GPU package (`pkg`) | **71 °C** (steady) | ~20 °C to throttle |
| GDDR6 junction (`vram`) | **84 °C** (steady) | ~11 °C to 95 °C throttle |
| Memory controller (`mctrl`) | 53 °C | trivial |
| Fan | 2913 → 2965 RPM (1156 RPM idle) | spins up smoothly under load |
| Power (Ressourcenmonitor) | 275 W | ~80 % of TDP |
| VRAM allocation | 12.4 GB (incl. desktop) ≙ 40 % | ~17 GB room for finer grids |

ASRock Creator B70 Pro's 3-fan workstation cooler handles 275 W indefinitely without thermal throttling. The 16 individual GDDR6-channel sensors (`vram_ch_0` … `vram_ch_15` under `/sys/class/drm/card0/device/hwmon/hwmon7/`) confirm uniform load across the 256-bit bus — this is a real bandwidth-saturated workload, not a single-channel hotspot.

### Force-field runs status (CC#2 → CC#3)

**CC#2 (135 M cells, 24 m × 6 m × 8 m box) had a vehicle-positioning bug:** the translation `0 - (vctr.y - vbbox.y * 0.5f)` placed the *entire* vehicle in the positive-Y region instead of straddling the symmetry plane. The half-domain symmetry-wall acted as a side-channel right next to the car, producing inflated drag and the wrong sign on lift. Fixed in commit `86e07da` by changing the translate to `0 - vctr.y` (vehicle Y-center on Y=0). Visually verified by Heiko in the volume-rendered velocity field.

**CC#3 (50 M cells, 20 m × 4 m × 5 m asym box, 50 000 steps, post-fix):** half-domain converged statistically from step ~7500 onward (Fx within ±5.8 % over the last 20 000 steps). Wall-time only **7.8 min** total.

```
Fx (Drag, half-dom, last 20 k)  = 1058 ± 61 N    →  full-vehicle  ≈ 2116 N (vs. expected 500–600 N → 3.8× too high)
Fz (Lift, half-dom, last 20 k)  =  +342 N        →  full-vehicle  ≈ +683 N (still positive — sign should be negative for downforce)
```

Drag dropped 45 % vs CC#2 (1927 N → 1058 N half-dom) and the half-domain mathematics is now physically meaningful, but the **absolute values are still ~4× the OpenFOAM reference**. The remaining gap is attributed to (in suspected order of impact):

1. **Symmetry boundary at Y=0 not explicitly free-slip** — FluidX3D's default streaming at Y=0 is periodic, which combined with the no-slip wall at Y_max yields a non-pure mirror condition. A true symmetry plane is needed for half-domain aero.
2. **Ground clearance only 1 cell = 20 mm** (real cars 100–150 mm) — moving wall directly under the car produces an exaggerated Bernoulli-lift contribution that flips downforce to lift.
3. **20 mm cell-size** under-resolves boundary layers (high y+) and rear-spoiler / diffuser detail.
4. **Vehicle si_length** was 4.0 m in CC#2/CC#3, real geometry is **4.5 m × 1.8 m × 1.1 m**. Fixed in setup.cpp post-CC#3 (changes Reynolds by +12.5 %, time-step scaling, and effective resolution per vehicle length).

For aero-converged values: CC#4 with explicit symmetry-plane handling, 5-cell ground clearance, optionally 16 mm cells, 200 k+ steps (~6–10 h wall-time on B70).

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
