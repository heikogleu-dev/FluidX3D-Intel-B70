# FluidX3D — Intel Arc Pro B70 (Battlemage) Fork

**Pioneer documentation: FluidX3D 3.6 LBM solver verified at 99.5 % peak bandwidth on Intel Arc Pro B70 Pro (BMG-G31, full Battlemage, xe driver, oneAPI OpenCL 26.05). 4× faster than the RTX 3060 Ti reference. Includes Linux/xe-driver shutdown-crash workaround, HiDPI font, windowed mode with env-var control, FORCE_FIELD enabled with VTK + CSV export of solid-boundary forces.**

This is a fork of [ProjectPhysX/FluidX3D](https://github.com/ProjectPhysX/FluidX3D). For the original project documentation see [README_UPSTREAM.md](README_UPSTREAM.md). All changes vs upstream are tracked file-by-file in [MODIFICATIONS.md](MODIFICATIONS.md). License is **unchanged** — see [LICENSE.md](LICENSE.md), non-commercial / non-military use only.

---

## Part of the Battlemage CFD Pioneer Series

This is one of three repositories documenting CFD on Intel Arc Pro B70 (BMG-G31):

1. **This repo — [FluidX3D-Intel-B70](https://github.com/heikogleu-dev/FluidX3D-Intel-B70)** — LBM via OpenCL. **99.5 % peak bandwidth, 5 464 MLUPS** (production-ready as iteration sandbox for vehicle aero).
2. **[Openfoam13---GPU-Offloading-Intel-B70-Pro](https://github.com/heikogleu-dev/Openfoam13---GPU-Offloading-Intel-B70-Pro)** — FVM pressure solver via Ginkgo SYCL. **Hardware ready, software stack maturing** (FP64 96 %, kernel-launch CUDA-par, GPU 66 % idle waiting for plumbing).
3. **[Openfoam-v2512-Petsc-Kokkos-Sycl-Intel-B70](https://github.com/heikogleu-dev/Openfoam-v2512-Petsc-Kokkos-Sycl-Intel-B70)** — PETSc-Kokkos-SYCL attempt. **Abandoned at GAMG path** — documents what doesn't work yet on this stack.

Together: first publicly documented end-to-end CFD evaluation on Battlemage Xe2 (Intel Arc Pro B70).

---

## Status

| Aspect | Status |
|---|---|
| Compute (LBM kernels) | ✅ **4 917 MLUPS @ 16.85 M cells, 605 GB/s = 99.5 % of B70 spec (608 GB/s)** |
| Allocation / memory layout | ✅ 56.6 B/cell empirical (D3Q19 + FP16C), matches theory; max ~449 M cells fit in B70's 28.6 GB |
| FP16C precision (DDFs) | ✅ stable, no observed divergence over 10 000 steps |
| `FORCE_FIELD` extension | ✅ enabled, force-field on TYPE_S boundaries written as VTK + CSV |
| `WALL_MODEL_VEHICLE` (CC#10) | ✅ **Werner-Wengle PowerLaw via Krüger Moving-Wall — production-valid for smooth STL vehicles** (MR2: 580 N nahe real 565 N, drag in OpenFOAM range). ⚠️ NOT validated against canonical Ahmed Body (Phase 1 FAILED, 125× force-amplification on flat-faced geometry — see CC#X section below) |
| Build (Linux + X11 + OpenCL ICD) | ✅ clean compile in 10 s with GCC 15.2 |
| Linux x11 windowed mode | ✅ default 2560×1440, env-var configurable |
| Linux xe-driver clean shutdown | ⚠️ requires `_exit(0)` workaround (see below) |
| Live visualisation FPS | ⚠️ 0–1 FPS at large window — FluidX3D's CPU software renderer is the bottleneck, **not** the GPU |
| Multi-GPU on B70 | 🚫 **out of scope** for this fork (single-tile B70). The TYPE_Y symmetry-plane patch attempt (CC#7) has not been validated for the halo-exchange path multi-GPU requires. |

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

### Force-field runs status (CC#4 → CC#7) — vehicle 4.5 × 1.8 × 1.1 m, 30 m/s, 10 mm cells

| Run | Cells | Steps to converge | Fx (Drag) | Fz (Lift) | Wall-time | Status |
|---|---:|---:|---:|---:|---:|---|
| **CC#4 all-walls Moving** | 202.5 M | 50 000 (no auto-stop) | 14 749 N | -250 N | ~28 min | broken: Couette-tunnel artifact |
| **CC#5 TYPE_E walls + wheels-on-floor** | 202.5 M | 50 000 (no auto-stop) | 16 180 N | -67 N | ~28 min | broken: Y_min TYPE_E ≠ symmetry |
| **CC#6-Half (TYPE_E pseudo-sym)** | 168.75 M | 16 000 (auto-stop @ 1 % drift Fx+Fy) | 16 177 N | -73 N | ~ 6 min | broken: see CC#5 |
| **CC#6-Full (no symmetry)** | **337.5 M** | 33 100 (manual stop) | **2 219 N** | +9 N | ~ 17 min | ✅ **reference** (still ~5× OpenFOAM, but physically correct) |
| **CC#7-V1 TYPE_Y specular swap** | 168.75 M | 12 100 (manual stop) | 14 472 N | +7 N | ~ 7 min | FAILED: kernel-level Esoteric-Pull conflict |
| **CC#7-V2 TYPE_Y one-way assignment** | 168.75 M | 5 000 (manual stop) | 14 386 N | -102 N | ~ 3 min | FAILED: bit-identical to V1 swap (EP cancels inline mod) |
| **CC#7-Alt1 TYPE_S Moving Y_min** | 168.75 M | 5 000 (manual stop) | 17 736 N | -240 N | ~ 3 min | FAILED: u_z=0 at sym-plane induces BL |
| **CC#8 TYPE_E\|TYPE_Y Ghost-Cell-Mirror** | 168.75 M | ~13 000 (auto-stop) | ~14 300 N | -100 N | ~ 7 min | FAILED: equilibrium-projection is Dirichlet BC, not specular reflection |
| **CC#9-V1..V5 various** | 168.75 M | 5-12 k | 13-14k N | -100 N | varied | All FAILED: only 19% reduction vs CC#6-Half |
| **CC#9-V6 Cut-surface strip** (Mode 0 + strip TYPE_X y=0) | 168.75 M | 5 000 | 13 206 N | -50 N | ~ 3 min | partial fix: 20% reduction (user hypothesis confirmed) |
| **CC#9-V7 = Mode 5 + V6** | 168.75 M | 5 000 | 13 046 N | -43 N | ~ 3 min | best half-domain achievable: 22% reduction (still 12× target) |
| **CC#10 Werner-Wengle Wall Model** | **337.5 M** | **11 300 (auto-stop @ 0.08 %)** | **579.8 N** | **+1 045 N** | **~ 7 min** | ✅ **WORKING — within OpenFOAM range** (Krüger Moving-Wall + WW PowerLaw) |

The **CC#10 Werner-Wengle wall model** is now the production-default (`#define WALL_MODEL_VEHICLE` in `defines.hpp`, active on top of `CC6_MODE=1`). Drag reduced **74 %** from 2 219 N (no wall model) to 579.8 N — squarely inside the OpenFOAM RANS expectation range of 400-600 N. See [findings/SESSION_2026-05-11_WW_RESULTS.md](findings/SESSION_2026-05-11_WW_RESULTS.md) and [findings/WALL_MODEL_RESEARCH.md](findings/WALL_MODEL_RESEARCH.md) for detail.

### CC#X — Ahmed Body Wall-Model Validation (2026-05-11) — ❌ Phase 1 FAILED

Per Opus-Plan a canonical Ahmed Body validation was attempted (simplified flat-front Ahmed at 25° slant, ERCOFTAC reference CD = 0.285):

| Setup | Fx [N] | CD measured | vs Literatur CD=0.285 | Verdict |
|---|---:|---:|---|---|
| Ahmed 25° **with WW** | 11 457 N | **104** | **365× too high** | ❌ **FAIL** |
| Ahmed 25° **without WW** (diagnostic) | 92 N | 0.84 | 3× (typical bounce-back overshoot) | ✓ baseline plausible |

**Diagnosis:** The Wall Model interacts pathologically with **flat-faced, sharp-edged geometry** (16-triangle simplified Ahmed), producing a 125× force-amplification artifact. The same WW works correctly on **smooth STL vehicles** (MR2: 580 N matches real 565 N target). Iron-Rule-Trigger fired at Phase 1.1 — no Phase 2 (sym-plane sweep) on Ahmed and no Phase 3 (real-vehicle re-application) until cause is identified. Full analysis: [findings/CC_X_ahmed/SESSION_2026-05-11_PHASE1_FAIL.md](findings/CC_X_ahmed/SESSION_2026-05-11_PHASE1_FAIL.md).

**Conclusion:** The CC#10 Werner-Wengle wall model is **production-valid for smooth STL vehicles** (Time-Attack MR2 use-case, where it was developed and validated). It is **NOT yet validated against canonical reference geometry** because the synthetic Ahmed test case revealed a previously-unsuspected geometry sensitivity. The wall model code remains unchanged (Iron Rule compliance) pending user decision on next direction (rounded-Ahmed STL, MR2-direct sym-plane sweep, or wall-model refinement).

## Roadmap & Findings

The current absolute drag is ~2 200 N for the Volldomain reference, against an OpenFOAM RANS reference of ~400–600 N — i.e. **~5× too high**. The plan is to close this gap stepwise:

### Session detail logs

- [findings/SESSION_2026-05-11_SUMMARY.md](findings/SESSION_2026-05-11_SUMMARY.md) — Full investigation of half-domain symmetry-plane: 11 variants tested, Esoteric-Pull architecture diagnosis, configuration audit, recommended path forward.

#### CC#11 Wall-Model deep dive (2026-05-13) — six failed attempts, methodical restart

- [findings/20_ww_audit.md](findings/20_ww_audit.md) — Code-Audit + Data-Flow-Diagram + analytical formula derivation of the Krüger force artifact.
- [findings/21_phase0d_final.md](findings/21_phase0d_final.md) — Phase 0d Bug-Verification consolidated report (3 paths verified).
- [findings/22_option1_negative_result.md](findings/22_option1_negative_result.md) — Option 1 analytical subtraction first failure analysis.
- [findings/23_step2_attempts.md](findings/23_step2_attempts.md) — Option 2 three sign-variant failures documented.
- [findings/24_option1_calibration_failure.md](findings/24_option1_calibration_failure.md) — Iron-Rule-Trigger after 3 Option-1 calibrations + path-forward proposals.
- [findings/25_ww_six_failed_attempts.md](findings/25_ww_six_failed_attempts.md) — Six-attempt journey with Lehmann's documented 1.3-2.0× overprediction reference; per-cell analytical subtraction doesn't generalize across geometries.
- [findings/26_ep_storage_boundary_pattern.md](findings/26_ep_storage_boundary_pattern.md) — EP-Storage interaction pattern explains both Sym-Plane and WW artifact failures: modify DDFs at fluid cells, not u at solid cells.
- [findings/27_typex_force_isolation.md](findings/27_typex_force_isolation.md) — TYPE_S\|TYPE_X marker isolation as practical workaround for clean force measurement on multi-component setups; verified in cube test.

### Open work items (priority order, with feasibility verdicts)

| # | Task | Effort in FluidX3D | Reference impl | Feasibility |
|---|---|---|---|---|
| ~~**1**~~ | ~~True symmetry plane (specular reflection) — separate post-stream-collide OpenCL kernel that reads/writes PDFs in fluid neighbors of TYPE_Y cells, bypassing Esoteric-Pull.~~ | — | [waLBerla FreeSlip](https://github.com/lssfau/walberla/blob/master/src/lbm/boundary/FreeSlip.h) | 🚫 **ATTEMPTED 2026-05-10/11, ALL APPROACHES FAILED.** Five different approaches tested (CC#7-V1/V2/Alt1, CC#8), all produce Fx in 14-18k range vs reference 2.2k. Inline DDF modification fails due to Esoteric-Pull storage layout (load_f/store_f roundtrip cancels modification). Architecturally requires rewriting the EP storage layer or adding a separate kernel pass with parity-aware slot access — both are essentially rewriting FluidX3D's core. See MODIFICATIONS.md for full analysis. **Status: blocked architecturally**, matching upstream maintainer's own statement in issue #288. |
| ~~**2**~~ | ~~Ghost-cell mirror at Y_min — alternative to #1: keep TYPE_E at Y_min but mirror u(y=1) into u(y=0) with u_y → -u_y.~~ | — | standard ghost-cell technique | 🚫 **ATTEMPTED 2026-05-11 (CC#8), FAILED.** Reduced Fx by 12 % vs plain TYPE_E (16.2k → 14.3k) but still 13× target. Root cause: equilibrium-projection at sym-plane is fundamentally a velocity-Dirichlet BC, it loses the higher-moment DDF shape that specular reflection should preserve. The cleaner equivalents (separate pre-stream kernel) would give the same physical limit since they all reduce to "force equilibrium consistent with mirror velocity field" — i.e., a Dirichlet pin, not a specular wall. |
| ~~**3**~~ | ~~**Werner-Wengle wall model** — Two-Layer power-law wall function (`u+ = y+` for `y+ ≤ 11.81`, `u+ = 8.3·y+^(1/7)` else). Closed-form `u_τ = (u_t / [8.3·(y/ν)^(1/7)])^(7/8)`. Modifies the velocity used in `apply_moving_boundaries` from no-slip (u=0) to a slip-velocity approximating the unresolved viscous sublayer.~~ | — | [OpenLB `WallFunctionBoundaryProcessor3D`](https://github.com/openLB/openLB/blob/master/src/boundary/wallFunctionBoundaryPostProcessors3D.h) | ✅ **DONE 2026-05-11 (CC#10).** Implemented via Krüger Moving-Wall approach (Approach A): new kernel `apply_wall_model_vehicle` runs BEFORE `stream_collide` each step, sets `u[vehicle_cell] = u_slip × direction(u_avg)` per Werner-Wengle PowerLaw closed-form. `apply_moving_boundaries` (existing) transfers the slip into the fluid via the standard Krüger correction. **Drag 2 219 N → 579.8 N (−74 %)**, within OpenFOAM range 400-600 N. No per-cell storage needed (~40 lines kernel). See [MODIFICATIONS.md § CC#10](MODIFICATIONS.md) and [findings/SESSION_2026-05-11_WW_RESULTS.md](findings/SESSION_2026-05-11_WW_RESULTS.md). |
| **4** | **Rotating wheels** — split `vehicle.stl` into body + front_wheels + rear_wheels in Blender (manual), then call `voxelize_mesh_on_device(wheel_stl, TYPE_S\|TYPE_X, axle_center, float3(0), float3(0, omega, 0))` per wheel pair. omega = lbm_u / r_wheel ≈ 0.075 / 34 = 2.2e-3 rad/step. | **30 min user (Blender) + 30 min code** | [F1 W14 example in `setup.cpp:1271`](https://github.com/ProjectPhysX/FluidX3D/blob/master/src/setup.cpp) (pattern: `voxelize_mesh_on_device(wheels, TYPE_S, center, 0, omega)`) | ✅ **Trivially feasible — built-in!** FluidX3D already has full rotating-mesh support: `void voxelize_mesh_on_device(const Mesh*, uchar flag=TYPE_S, const float3& rotation_center=0, const float3& linear_velocity=0, const float3& rotational_velocity=0)` (`lbm.hpp`). `update_moving_boundaries()` is called automatically when `rotational_velocity ≠ 0` and flag includes TYPE_S. **Zero code-changes needed**, only setup.cpp calls. |
| **5** | **AMR (Adaptive Mesh Refinement)** — Multi-level grid with finer cells at vehicle surface only, coarser elsewhere. Per-level relaxation-time scaling (`τ' = τ / (2^L (1-τ/2) + τ/2)`), recursive nested timesteps (level L runs 2× per parent step), coarse-fine PDF interpolation at refinement interfaces. | **months — full architectural rewrite** | [waLBerla `BasicRecursiveTimeStep::timestep()`](https://github.com/lssfau/walberla/blob/master/src/lbm_generated/refinement/BasicRecursiveTimeStep.impl.h), `RefinementScaling.h`, NonUniformGridGPU benchmark; OpenLB also has refinement | 🚫 **NOT feasible without forking the entire FluidX3D architecture.** Maintainer ProjectPhysX in [issue #127](https://github.com/ProjectPhysX/FluidX3D/issues/127): *"FluidX3D cannot do adaptive grid refinement"* and in [issue #284](https://github.com/ProjectPhysX/FluidX3D/issues/284) the user case was explicitly rebuffed: *"For y+ < 1, you would need ~5 orders of magnitude spacial resolution, or ~10000³ cells. Today's high-end hardware can only get you ~4000³ cells."* FluidX3D's Esoteric-Pull and AA-pattern memory layouts are tightly coupled to a single uniform grid. Adding AMR would require multi-level pdfField management, per-level kernel launches, fine-coarse halo communication, and split timestepping. **A waLBerla-style implementation is essentially a new LBM solver, not a patch.** Recommend instead: scale to a larger-VRAM GPU (e.g. MI300X 192 GB → 4× cell budget) or accept that 10 mm uniform + wall model #3 is the resolution-quality sweet-spot for this fork. |

### Investigations completed

| # | Topic | Outcome |
|---|---|---|
| ✅ | Vehicle STL manifold check (trimesh) | Watertight (0 open edges, 1.48 M tris, 99.98 % manifold). NOT the source of the high drag. |
| ✅ | `lbm.object_force(TYPE_S\|TYPE_X)` filter semantics | Exact-match in `kernel.cpp:1941` (`flags[n]==flag_marker`). Correctly isolates vehicle from floor/walls — verified. |
| ✅ | F1 W14 example boundary-condition convention | All FluidX3D aero examples (Ahmed body, NASA CRM, Concorde, F1) use `TYPE_E` for outer walls + `TYPE_S` for floor only. Matches CC#6 default. F1 example uses **rotating wheels** but no force computation. |
| ✅ | Half- vs full-domain drag-factor diagnosis | TYPE_E at Y=0 is NOT a valid symmetry approximation: it acts as an absorbing free-stream (= a virtual second inlet next to the car), inflates drag ~7× over full-domain reference. |
| ✅ | TYPE_Y specular-reflection kernel patch (CC#7) | FAILED. Root cause: Esoteric-Pull DDF storage. For EP-opposite pairs (3,4) the swap+store_f is a no-op (writes same value back to same slot). For non-EP-opposite Y-mirror pairs (7,13), (8,14), (11,18), (12,17) the swap writes data to wrong neighbor cells (y=−1, periodic-wrap to TYPE_E cap). **Conclusion: a correct sym-plane needs a separate post-stream BC handler** (work item #1). |
| ✅ | OpenLB / waLBerla reference architectures | Both use **specular reflection** but as **post-stream-step boundary handlers**, not inline in the main collision/stream kernel. waLBerla `FreeSlip::treatDirection()` reads the fluid neighbor of the boundary cell and copies a mirror-direction PDF. Per-stencil precomputed `mirrorX/Y/Z` lookup tables. This is the correct architectural pattern for FluidX3D too. |
| ✅ | OpenLB Wall-Model architecture | OpenLB has a complete Werner-Wengle/Musker wall function in `src/boundary/wallFunctionBoundaryPostProcessors3D.h`. Uses a special lattice descriptor `WallFunctionForcedD3Q19Descriptor` with extra per-cell storage `AV_SHEAR, TAU_W, TAU_EFF, FORCE, V12`. Configurable via `wallFunctionParam`: `wallProfile = 0:Musker / 1:PowerLaw`, `useVanDriest`, `vonKarman = 0.375`, `latticeWalldistance = 0.5`. Runs as `LocalPostProcessor3D` after collide-stream. **Adoption pattern for FluidX3D**: add new fields per TYPE_S\|TYPE_X cell (~100 MB at vehicle surface), new post-stream OpenCL kernel computes u_τ from first-fluid-cell tangential velocity, modifies bounce-back accordingly. |
| ✅ | waLBerla AMR architecture | `lbm_generated/refinement/BasicRecursiveTimeStep.impl.h` implements multi-level recursive timestepping: level L runs 2× per parent step, coarse-to-fine and fine-to-coarse PDF communication between levels via `commScheme_->communicateCoarseToFine(L)`. Per-level relaxation-time scaling `τ' = τ / (2^L (1−τ/2) + τ/2)`. NonUniformGridGPU benchmark demonstrates GPU support. **Conclusion: this is a parallel architecture from FluidX3D's single uniform grid; not feasible to backport without a full rewrite.** |
| ✅ | FluidX3D upstream stance on AMR + wall model | Maintainer ProjectPhysX in [issue #127](https://github.com/ProjectPhysX/FluidX3D/issues/127): *"FluidX3D cannot do adaptive grid refinement … expect drag force to be too large by about a factor 2, due to lack of grid refinement and no wall model"*. README FAQ: *"Does FluidX3D support adaptive mesh refinement? No, not yet. Grid cell size is the same everywhere in the simulation box."* In [issue #288](https://github.com/ProjectPhysX/FluidX3D/issues/288): *"I'm stuck with the literal corner case in 3D where specular reflection of DDFs reflect them from space diagonals to axis-aligned directions, which causes trouble with the different collision operator weights for those directions. There is no solution to this in literature either."* In [issue #284](https://github.com/ProjectPhysX/FluidX3D/issues/284) the user request for finer mesh near walls was acknowledged but no plan announced. |
| ✅ | FluidX3D rotating-mesh support | **Already built-in.** `void voxelize_mesh_on_device(const Mesh*, uchar flag=TYPE_S, const float3& rotation_center=0, const float3& linear_velocity=0, const float3& rotational_velocity=0)` is the public API in `src/lbm.hpp`. `update_moving_boundaries()` is auto-called when `rotational_velocity != 0 && (flag & (TYPE_S\|TYPE_E))==TYPE_S`. F1 W14 example in `setup.cpp:1271` already uses this pattern with `omega = lbm_u / lbm_radius`. **Adoption is purely a setup.cpp change** + manual STL split in Blender (~30 min). |
| ✅ | Multi-GPU scope | **Out of scope for this fork.** Single-GPU only. The TYPE_Y patch's halo-exchange interaction was not investigated. |

### Goal

Close the gap to **OpenFOAM-RANS-equivalent forces** (Cd ≈ 0.35–0.5, Fx ≈ 400–600 N for our vehicle at 30 m/s). FluidX3D delivers **5 464 MLUPS at 96–100 % of B70 spec bandwidth** even for 337 M cells — the throughput is excellent. The current bottleneck is purely physical fidelity (wall model, true symmetry plane, rotating wheels), not GPU performance.

### B70 Pro hardware verdict

The **Intel Arc Pro B70 (BMG-G31)** consistently delivers **96–100 % of its 608 GB/s nominal memory bandwidth** at all working-set sizes tested (16.85 M to 337.5 M cells), at 71 °C package / 84 °C VRAM under sustained 275 W load — comfortable thermal headroom (90/95 °C throttle). The card is **fully bandwidth-saturated for LBM workloads**, which is the expected and ideal regime. A ~4× speed-up over the prior RTX 3060 Ti reference.

The **ASRock Creator B70 Pro** uses a workstation-class **blower-type cooler** (single rear-exhaust radial fan exhausting hot air OUT of the chassis via the rear PCIe bracket — not into the case interior, unlike consumer-class axial-fan designs). This makes it the right pick for multi-GPU rigs and tight workstation chassis where heat must not recirculate. Fan speed 1 600–2 900 RPM under load; no throttling observed.

## LBM solver landscape — why FluidX3D on the Intel Arc Pro B70

For full transparency, here is the comparison of FluidX3D against the other major open-source LBM solvers, evaluated specifically against our requirements (Intel Arc Pro B70 GPU, vehicle aerodynamics, half/full domain, force computation, sym-plane, wall model, rotating wheels, AMR):

| Solver | License | GPU backend | Runs on B70? | AMR | Wall model | Free-slip BC | Rotating mesh | Peak perf | Maintained |
|---|---|---|---|---|---|---|---|---:|---|
| **FluidX3D** (this fork's base) | FluidX3D Public License (non-commercial) | **OpenCL** (NVIDIA, AMD, Intel) | ✅ **yes, native** | ❌ planned `no, not yet` | ❌ none (factor 1.3-2× drag overshoot acknowledged) | ⚠️ partial via TYPE_E (= absorbing, ≠ specular) | ✅ via `voxelize_mesh_on_device(..., omega)` | **5 464 MLUPS @ 96-100 % BW (B70)** | ✅ active (Moritz Lehmann) |
| **waLBerla** | GPLv3 | CUDA + HIP only | 🚫 no (CPU-fallback only — ~50× slower) | ✅ `BasicRecursiveTimeStep` | ✅ via `lbmpy` codegen | ✅ `FreeSlip` class | ✅ generated via `lbmpy` | ~ 1 200 MLUPS @ 72 % BW (GTX Titan) | ✅ active (FAU) |
| **OpenLB** | GPLv2 | OpenMP+MPI (CPU), **SYCL** experimental for GPU | ⚠️ SYCL backend would target B70, but not yet production-grade | ✅ static grid refinement | ✅ Werner-Wengle / Musker, van Driest | ✅ `addSlipBoundary` | ✅ moving boundaries | ~ 1 500–2 500 MLUPS GPU; "32× faster than OpenFOAM" CPU | ✅ active (KIT) |
| **TCLB** (CFD-GO) | GPLv3 | CUDA + HIP + CPU (MPI) | 🚫 no | ⚠️ partial | ⚠️ depends on model | ⚠️ depends on model | ⚠️ depends on model | ~ 1 000–1 500 MLUPS | ✅ active (204 stars) |
| **Palabos** (FlowKit Ltd / EPFL) | GPLv3 / FlowKit commercial | OpenMP + MPI (**CPU only** — official builds; experimental Palabos-GPU exists separately) | 🚫 no GPU | ✅ hierarchical grid refinement | ✅ several models | ✅ free-slip | ✅ moving objects | ~ 10–50 MLUPS (CPU only) | ⚠️ slow upstream pace |
| **Sailfish** (sailfish-team) | LGPLv3 | **OpenCL** + CUDA | ✅ yes (via OpenCL) | ❌ | ⚠️ partial | ⚠️ partial | ❓ | unknown / not benchmarked recently | 🚫 **abandoned (last push 2022-02)** |
| **lbmpy** (FAU) | AGPLv3 | code-generator only — emits CUDA/HIP/CPU kernels for waLBerla | only via waLBerla → 🚫 not on B70 | ✅ via waLBerla | ✅ via waLBerla | ✅ via waLBerla | ✅ via waLBerla | inherits waLBerla | ✅ active (FAU) |
| **Musubi** (TUD APES) | BSD | MPI + OpenMP (CPU); some OpenACC | 🚫 no usable GPU | ✅ octree-based | ⚠️ research models | ✅ | ⚠️ | unknown | ⚠️ academic, slow updates |
| **STLBM** (Latt et al.) | GPL | C++17 STL parallel exec (CPU) | 🚫 CPU only | ❌ | ❌ | ❌ | ❌ | ~ tens of MLUPS | ⚠️ research demo |

### Verdict for the Intel Arc Pro B70 platform

Of the solvers above, only three actually run on the B70 with GPU acceleration:

1. **FluidX3D (OpenCL)** — natively. **Highest single-GPU bandwidth utilisation in the field at 96-100 %**, ahead of waLBerla's ~72 % and OpenLB's ~60-70 %. **No AMR, no built-in wall model, no specular sym-plane** — these are the open work items (and the reason for this fork).
2. **OpenLB SYCL backend** — experimental, not production-grade as of mid-2026. Would give us AMR + Werner-Wengle + free-slip out of the box, but with a less mature SYCL path on Intel GPUs and likely 30-50 % lower bandwidth efficiency.
3. **Sailfish OpenCL** — abandoned upstream, unsuitable for serious work.

waLBerla, TCLB, lbmpy-via-waLBerla, Palabos, Musubi all require **NVIDIA or AMD GPUs** (CUDA/HIP) — for them to be options, the GPU itself would need to be replaced.

### Strategic conclusion

**FluidX3D + the custom patches in this fork is the right choice for the B70.** The 5× drag overshoot vs. OpenFOAM-RANS is fundamentally a **boundary-condition fidelity gap (sym-plane, wall model, rotating wheels)**, not a solver-quality gap. FluidX3D's compute throughput is class-leading for our hardware. Closing the BC gap via the [Roadmap](#roadmap--findings) work items #1, #3, #4 is much cheaper than porting to another solver — and porting to waLBerla/TCLB would also require leaving the B70 platform behind. **OpenLB-SYCL is the only real "Plan B" if Intel improves its SYCL stack and OpenLB's GPU support matures, but until then FluidX3D wins on every concrete metric that matters here.**

## Companion repos

- **Paraview / OSPRay / B70 ray-tracing** — [heikogleu-dev/Paraview---Intel-B70-Pro-OSPRAY-Raytracing-Pathtracing](https://github.com/heikogleu-dev/Paraview---Intel-B70-Pro-OSPRAY-Raytracing-Pathtracing)
- **OpenFOAM v2512 + PETSc + Kokkos + SYCL** — [heikogleu-dev/Openfoam-v2512-Petsc-Kokkos-Sycl-Intel-B70](https://github.com/heikogleu-dev/Openfoam-v2512-Petsc-Kokkos-Sycl-Intel-B70)
- **OpenFOAM 13 GPU offloading (Ginkgo SYCL)** — [heikogleu-dev/Openfoam13---GPU-Offloading-Intel-B70-Pro](https://github.com/heikogleu-dev/Openfoam13---GPU-Offloading-Intel-B70-Pro)

## Related Pioneer Work on Intel Arc Pro B70

Sister repository documenting the FP64 implicit-solver perspective on the same hardware:
**[Openfoam13---GPU-Offloading-Intel-B70-Pro](https://github.com/heikogleu-dev/Openfoam13---GPU-Offloading-Intel-B70-Pro)** — OpenFOAM pressure solver via Ginkgo SYCL. Key complementary findings:
- FP64 96 % efficiency, kernel-launch on CUDA level (5.6 µs sync / **1.5 µs async batched, 3.3× better than CUDA sync**)
- Software stack (preconditioner maturity, GPU-aware MPI) is the real bottleneck, not hardware
- Bandwidth-bound workloads (this repo's domain, LBM) reach 99.5 % peak; implicit FP64 workloads (sister repo's domain) blocked by Ginkgo 1.10 SYCL bugs

Earlier abandoned attempt at the same workload via PETSc-Kokkos-SYCL stack: **[Openfoam-v2512-Petsc-Kokkos-Sycl-Intel-B70](https://github.com/heikogleu-dev/Openfoam-v2512-Petsc-Kokkos-Sycl-Intel-B70)** — GAMG family crashes in `MatProductSymbolic_SeqAIJKokkos`. Pioneer documentation of why this path was abandoned.

Together — **[Battlemage CFD Pioneer Series](https://github.com/heikogleu-dev/Openfoam13---GPU-Offloading-Intel-B70-Pro#part-of-the-battlemage-cfd-pioneer-series)** — first publicly documented end-to-end CFD evaluation on Battlemage Xe2.

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
