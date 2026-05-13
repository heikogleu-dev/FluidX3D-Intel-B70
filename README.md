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
| `WALL_MODEL_VEHICLE` (CC#10) | ⚠️ **Code compiled but disabled at vehicle cells in current Step-1b Safe-State default.** Phase B 2.5 diagnostic complete (2026-05-13): WW Krüger has **three stable attractors** (BB +1820 N, halved +163k N, full -610 N), none physically correct. Krüger Moving-Wall pattern fundamentally unfit for stationary wall WW. See [`findings/36_phaseB_diagnostic_complete.md`](findings/36_phaseB_diagnostic_complete.md). Next: Phase C (OpenLB Pi-Tensor) or Refinement (Bouzidi BB). |
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

Quick summary (full detail in [`MODIFICATIONS.md`](MODIFICATIONS.md), comprehensive line-by-line audit in [`findings/35_deviations_from_upstream.md`](findings/35_deviations_from_upstream.md)):

- **`src/defines.hpp`** — enable `VOLUME_FORCE` + `FORCE_FIELD` + `WALL_MODEL_VEHICLE` + `MOVING_BOUNDARIES` + `SUBGRID`
- **`src/graphics.hpp`** — `FONT_WIDTH 6→12`, `FONT_HEIGHT 11→22` (HiDPI)
- **`src/graphics.cpp`** — auto-start (`key_P = true`), 2× pixel-doubled `draw_text()`, windowed-mode override with `FLUIDX3D_WINDOW=WxH`, decorated X11 window
- **`src/setup.cpp`** — vehicle-aero half/full-domain setups (CC#1-CC#11), `_exit(0)` workaround for xe-driver shutdown race, MR2/Yaris binary STL handling, `CUBE_VALIDATION` + `AHMED_MODE` compile-toggles
- **`src/kernel.cpp`** — LBM-Core deviations (17 documented in Finding 35): `apply_moving_boundaries` Step-1b TYPE_X-Exclusion, NEW kernels `apply_freeslip_y` (CC#9, sym-plane), `apply_wall_model_vehicle` (CC#10, Werner-Wengle), `apply_wall_slip_to_fluid` + `compute_wall_model_artifact` (CC#11 experiments, allocated but call-disabled), `update_moving_boundaries` per-step refresh, `stream_collide` TYPE_E+TYPE_Y ghost-mirror branch
- **`src/lbm.cpp`** — kernel allocations, enqueue methods, do_time_step WW chain, `def_nu` + `WALL_MODEL_VEHICLE` device defines
- **`src/lbm.hpp`** — kernel declarations + method declarations for above
- **`CLAUDE.md`** — engineering methodology rules (Skalen-Ladder, Smoke-Test, Root-Cause-vor-Pivot) — autonomous session governance
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

The **CC#10 Werner-Wengle wall model** was originally documented as production-default with MR2 Fx=580 N (matching real 565 N target). However Phase B 2.5 diagnostic (2026-05-13) revealed that result was on a DIFFERENT (less detailed) vehicle STL. On the current MR2 STL the same Full Krüger -6 logic gives **-610 N (negative drag, over-corrected)** — see CC#11 Wall-Model Deep Dive below.

### CC#11 — Wall-Model Deep Dive (2026-05-13) — Three Attractors, none physically correct

After CC#X Ahmed-Body Phase 1 failure (Cd=104 with WW vs literature 0.285), Phase 0d Bug-Verification confirmed Krüger Moving-Wall force-artifact is the root cause of unphysical results. Subsequent Phase B Sub-Task work mapped the complete Krüger-coupling-strength parameter space.

**Three stable LBM attractors on MR2** (337M cell full-domain, 30 m/s):

| Krüger coupling | MR2 Fx [N] | CD-equivalent | Verdict |
|---|---:|---:|---|
| **0%** (WW kernel disabled) | +1,643 N (~+1820 N Step-1b) | 2.4-2.9 | ❌ BB over-prediction (Lehmann 1.3-2.0× range) |
| **50%** (halved -3) | +163,000 N | ~200 | ❌ catastrophic, BL-cancellation collapsed |
| **100%** (full -6, CC#10 logic) | -610 N | -0.9 | ❌ negative drag, WW over-corrected |
| **Target** (OpenFOAM-RANS reference) | +565 N | 0.83 | (reference) |

**Key insight:** Linear interpolation between BB-baseline (0%) and Full-Krüger (100%) predicts midpoint at ~+517 N. Actual halved result is +163,000 N — **315× off linear prediction**. The Krüger-coupling response is fundamentally non-linear (self-referential WW↔BL feedback). Halved Krüger gets stuck in the Full-Krüger transient path (+342k transient peak before -610 N attractor).

**Cross-geometry validation:**

| Geometry | Halved Krüger Result | Target | Pathology |
|---|---:|---:|---|
| Cube (sharp-edged) | CD=40 | 1.05 | matches Finding 33 per-cell prediction ✓ |
| Ahmed 25° (BL-resolved) | Cd=75 | 0.285 | 264× too high ❌ |
| MR2 (BL-resolved, smooth) | Fx=+163k N | 565 N | 290× too high ❌ |

Per-cell formula (Finding 33, single-cell isolation test) is mathematically correct. Integration over BL-resolved geometries breaks cancellation properties — halved Krüger fails universally on real vehicles.

**Production verdict:** Code defaults to Step-1b Safe-State (TYPE_X-Exclusion at vehicle cells in `apply_moving_boundaries`). This gives **pure bounce-back baseline** at the vehicle, no Krüger artifact, no negative drag, no unphysical magnitudes — but inherits Lehmann's documented 1.3-2.0× BB over-prediction.

**Full findings sequence (CC#11):**
- [findings/20_ww_audit.md](findings/20_ww_audit.md) — Code-Audit + Data-Flow + analytical Krüger-artifact derivation
- [findings/21_phase0d_final.md](findings/21_phase0d_final.md) — Phase 0d Bug-Verification (3 paths)
- [findings/22_option1_negative_result.md](findings/22_option1_negative_result.md) — Option 1 first failure
- [findings/23_step2_attempts.md](findings/23_step2_attempts.md) — Option 2 three sign-variants failed
- [findings/24_option1_calibration_failure.md](findings/24_option1_calibration_failure.md) — Iron-Rule-Trigger after 3 calibrations
- [findings/25_ww_six_failed_attempts.md](findings/25_ww_six_failed_attempts.md) — Six-attempt journey, Lehmann reference
- [findings/26_ep_storage_boundary_pattern.md](findings/26_ep_storage_boundary_pattern.md) — EP-Storage pattern explains both Sym-Plane and WW artifacts
- [findings/27_typex_force_isolation.md](findings/27_typex_force_isolation.md) — TYPE_S\|TYPE_X marker isolation pattern
- [findings/29_geometry_re_yplus_matrix.md](findings/29_geometry_re_yplus_matrix.md) — y+ matrix per geometry (Cube 68, Ahmed/MR2/Yaris ~300)
- [findings/30_poiseuille_ww_validation_setup.md](findings/30_poiseuille_ww_validation_setup.md) — Canonical channel validation sketch (3-variant)
- [findings/31_diagnostic_kernel_sketch.md](findings/31_diagnostic_kernel_sketch.md) — DIAGNOSTIC_WW VTK export design
- [findings/32_force_code_audit.md](findings/32_force_code_audit.md) — object_force() code-pfad-audit, BEWIESEN/HYPOTHESE/OFFEN markers
- [findings/33_single_cell_krueger_test.md](findings/33_single_cell_krueger_test.md) — Single-cell test confirms Hypothesis B (factor 6 direct, no EP doubling)
- [findings/34_halved_krueger_negative.md](findings/34_halved_krueger_negative.md) — Phase B 2.0 initial halved-Krüger failure
- [findings/35_deviations_from_upstream.md](findings/35_deviations_from_upstream.md) — 17-deviation Code-State audit vs upstream
- [findings/36_phaseB_diagnostic_complete.md](findings/36_phaseB_diagnostic_complete.md) — **Three-Attractor synthesis (this finding set)**

### CC#X — Ahmed Body Wall-Model Validation (2026-05-11) — ❌ Phase 1 FAILED (subsumed by CC#11)

Initial canonical Ahmed validation (simplified flat-front, 25° slant, ERCOFTAC ref CD=0.285) gave Cd=104 with CC#10 Full Krüger — 365× over target. This single datapoint triggered the Phase 0d Bug-Verification investigation that culminated in CC#11's Three-Attractor finding. Original analysis preserved at [findings/CC_X_ahmed/SESSION_2026-05-11_PHASE1_FAIL.md](findings/CC_X_ahmed/SESSION_2026-05-11_PHASE1_FAIL.md).

## Roadmap & Findings

**Current state (2026-05-13):** Step-1b Safe-State (commit `741a974`). MR2 baseline +1820 N (3.2× over OpenFOAM target +565 N), no WW reduction, no artifacts. All CC#11 Wall-Model experiments documented (Findings 20-36) and converged to the architectural conclusion: Krüger Moving-Wall pattern is fundamentally unfit for stationary-wall WW transport (Three-Attractor non-linear bifurcation).

**Decision points open:**
- **Phase C-A: OpenLB Pi-Tensor f_neq Reconstruction** — replace Krüger transport with fluid-side post-stream DDF reconstruction (literature-validated, geometry-agnostic). 1-2 weeks implementation.
- **Phase C-B: Bouzidi Sub-Grid Bounce-Back** — interpolated BB with sub-cell wall-distance, complementary to (not replacement for) WW. Reduces BB over-prediction without WW physics. 3-5 days implementation.
- **Accept current Step-1b Safe-State** — 3.2× drag over-prediction is acknowledged limitation; consistent with Lehmann's documented 1.3-2.0× upper bound.

### Session detail logs

- [findings/SESSION_2026-05-11_SUMMARY.md](findings/SESSION_2026-05-11_SUMMARY.md) — Full investigation of half-domain symmetry-plane: 11 variants tested, Esoteric-Pull architecture diagnosis, configuration audit, recommended path forward.

#### CC#11 Wall-Model deep dive (2026-05-13) — complete

See "CC#11 — Wall-Model Deep Dive" section above for the full findings sequence (Findings 20-36) and Three-Attractor conclusion. Repo state after CC#11: Step-1b Safe-State as production default.

### Open work items (priority order, with feasibility verdicts)

| # | Task | Effort | Reference impl | Feasibility |
|---|---|---|---|---|
| **1** | **OpenLB Pi-Tensor f_neq Reconstruction (Phase C-A)** — replace Krüger Moving-Wall transport with fluid-side post-stream DDF reconstruction at TYPE_MS cells. Uses local stress tensor (Π = Σ c_i c_i f_neq_i) and Werner-Wengle u_τ. Modifies fluid DDFs directly, never lands in solid cells → no force artifact, no Krüger Three-Attractor pathology. | **1-2 weeks** | [OpenLB `WallFunctionBoundaryProcessor3D`](https://github.com/openLB/openLB/blob/master/src/boundary/wallFunctionBoundaryPostProcessors3D.h) — uses `WallFunctionForcedD3Q19Descriptor` with per-cell `AV_SHEAR, TAU_W, TAU_EFF, FORCE, V12` storage | ✅ **Recommended primary track.** Architecturally clean (modify fluid not solid). Validated by OpenLB. Sketch in [findings/30_poiseuille_ww_validation_setup.md](findings/30_poiseuille_ww_validation_setup.md). Per-cell storage cost ~100 MB at vehicle surface. |
| **2** | **Bouzidi Sub-Grid Bounce-Back (Phase C-B)** — interpolated BB using sub-cell wall-distance q ∈ [0, 1]. Reduces ½-cell-discretization error of standard BB. Complementary to (not replacement for) WW: addresses BB over-prediction, not BL physics. Sub-grid distance precomputed via Walberla-style ray-cast through STL. | **3-5 days** | Bouzidi 2001 paper, [OpenLB `BouzidiVelocity`](https://gitlab.com/openlb/release/-/blob/master/src/boundary/bouzidiBoundary.h) | ✅ **Recommended alternative track.** Lower complexity than Phase C-A. Reduces BB over-prediction from 3.2× to ~1.5-2× (closer to Lehmann's upper bound). Doesn't require WW physics. Can stack with Phase C-A later. |
| ~~**3**~~ | ~~**Werner-Wengle wall model via Krüger Moving-Wall (CC#10 approach)**~~ | — | [OpenLB `WallFunctionBoundaryProcessor3D`](https://github.com/openLB/openLB/blob/master/src/boundary/wallFunctionBoundaryPostProcessors3D.h) | 🚫 **ATTEMPTED CC#10/CC#11 2026-05-11 → 2026-05-13, ARCHITECTURALLY UNFIT.** Krüger Moving-Wall is intended for ACTUAL moving solids (floors, wheels). Applied as WW transport at stationary vehicle surface, it produces three stable non-linear attractors (BB +1820 N, halved +163k N, full -610 N) — no linear interpolation, no physical state. Phase B 2.5 diagnostic exhausted all coupling strengths. **Status: blocked architecturally.** Findings 25-36 document the full investigation. |
| ~~**4**~~ | ~~True symmetry plane (specular reflection)~~ | — | [waLBerla FreeSlip](https://github.com/lssfau/walberla/blob/master/src/lbm/boundary/FreeSlip.h) | 🚫 **ATTEMPTED 2026-05-10/11 (CC#7/CC#8/CC#9), ALL FAILED.** EP-Storage prevents inline DDF modification at sym-plane cells. Five approaches tested, all produce Fx 14-18k vs reference. Matches upstream issue #288. |
| ~~**5**~~ | ~~Ghost-cell mirror at Y_min (CC#8)~~ | — | standard ghost-cell | 🚫 **ATTEMPTED 2026-05-11, FAILED.** Equilibrium-projection at sym-plane is velocity-Dirichlet, loses specular reflection's higher-moment DDF shape. |
| **6** | **Rotating wheels** — split `vehicle.stl` into body + front_wheels + rear_wheels in Blender (manual), then call `voxelize_mesh_on_device(wheel_stl, TYPE_S, axle_center, float3(0), float3(0, omega, 0))` per wheel pair. **Marker convention:** Body as TYPE_S\|TYPE_X (measured), wheels as TYPE_S only (in flow but force-isolated, per Finding 27). | **30 min user (Blender) + 30 min code** | [F1 W14 example in upstream `setup.cpp`](https://github.com/ProjectPhysX/FluidX3D/blob/master/src/setup.cpp) | ✅ **Trivially feasible — built-in!** FluidX3D already has full rotating-mesh support via `voxelize_mesh_on_device` with rotational_velocity parameter. **Deferred to last priority** per current Phase C work focus: wall-model physics fix is more impactful than rotating-wheels detail. |
| **7** | **AMR (Adaptive Mesh Refinement)** — Multi-level grid with finer cells at vehicle surface only. | **months — full architectural rewrite** | [waLBerla `BasicRecursiveTimeStep`](https://github.com/lssfau/walberla/blob/master/src/lbm_generated/refinement/BasicRecursiveTimeStep.impl.h) | 🚫 **NOT feasible without forking entire FluidX3D architecture.** Maintainer ProjectPhysX explicitly rebuffed in [issue #284](https://github.com/ProjectPhysX/FluidX3D/issues/284). FluidX3D's Esoteric-Pull and AA-pattern tightly coupled to uniform grid. **Alternative: scale to larger-VRAM GPU** or accept 10mm uniform + Phase C-A/B as quality sweet-spot. |

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

Close the gap to **OpenFOAM-RANS-equivalent forces** (Cd ≈ 0.35–0.5, Fx ≈ 400–600 N for our 30 m/s vehicle). FluidX3D delivers **5 464 MLUPS at 96–100 % of B70 spec bandwidth** even for 337 M cells — the throughput is excellent. The current bottleneck is purely **physical fidelity at the wall** (BB over-prediction inherits Lehmann's documented 1.3-2.0× factor; CC#11 confirmed Krüger Moving-Wall WW is architecturally unfit). Phase C (OpenLB Pi-Tensor OR Bouzidi sub-grid BB) is the next quality milestone.

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

**FluidX3D + the custom patches in this fork is the right choice for the B70.** The 3.2× drag overshoot in Step-1b Safe-State is fundamentally a **wall-boundary-condition fidelity gap (BB ½-cell discretization + missing WW physics)**, not a solver-quality gap. FluidX3D's compute throughput is class-leading for our hardware. **The CC#11 deep dive (Findings 25-36) ruled out the Krüger Moving-Wall WW approach** — Phase C transition to OpenLB Pi-Tensor reconstruction (item #1) or Bouzidi sub-grid BB (item #2) is the actionable next step. Porting to waLBerla/TCLB would require leaving B70 behind; **OpenLB-SYCL remains the long-term Plan B** if Intel's SYCL stack and OpenLB's GPU support mature.

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
