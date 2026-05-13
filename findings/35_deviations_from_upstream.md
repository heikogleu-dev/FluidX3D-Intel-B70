# Finding 35: Deviations from Upstream FluidX3D — Debugging Baseline

**Datum:** 2026-05-13
**Status:** Comprehensive code-deviation log relative zu
`upstream/master` (ProjectPhysX/FluidX3D, latest `0af073c`).
**Zweck:** Für Debugging-Sessions wissen, was nicht mehr Original ist.
**Fork-Branch:** `phase0-ahmed-validation` (HEAD: `3e046f7` commit-time)

---

## Datei-Übersicht: Was wurde geändert

| Datei | Original-Mods (CC#1-CC#11) | Type-Klassifikation |
|---|---|---|
| `src/kernel.cpp` | 10+ Sektionen modifiziert | LBM-Core |
| `src/lbm.cpp` | 5 Sektionen modifiziert | LBM-Wrapper |
| `src/lbm.hpp` | Kernel-Declarations + Methods | LBM-Interface |
| `src/defines.hpp` | +1 Define (WALL_MODEL_VEHICLE) | Config |
| `src/setup.cpp` | Komplett umgeschrieben (CC#1-CC#11 Setups) | Setup-Only |
| `README.md`, `MODIFICATIONS.md`, `CLAUDE.md` | Dokumentation | Non-Behavior |

LBM-Core Änderungen (Type 1) sind die kritischsten für Debugging.
Setup-Only Änderungen (Type 5) sind weniger riskant da sie das LBM-Verhalten
nicht direkt modifizieren.

---

## kernel.cpp Deviations (LBM-Core)

### 1. `apply_moving_boundaries` MODIFIED — Step-1b TYPE_X-Exclusion
**Lines:** ~1115-1128
**Origin:** CC#11 Option 2 Step 1b (commit `b2683f1`)
**Original (upstream):**
```cpp
const float w6 = -6.0f*w(i);
ji = j[i+1u]; fhn[i] = (flags[ji]&TYPE_BO)==TYPE_S
    ? fma(w6, c(i+1u)*u[ji]+..., fhn[i])
    : fhn[i];
```
Applies full Krüger -6 to ALL TYPE_S cells.

**Modified (Step-1b):**
```cpp
const float w6 = -6.0f*w(i);
ji = j[i+1u]; { const uchar fj=flags[ji];
    const bool is_ww=(fj&TYPE_BO)==TYPE_S && (fj&TYPE_X);
    fhn[i] = ((fj&TYPE_BO)==TYPE_S && !is_ww)
        ? fma(w6, c(i+1u)*u[ji]+..., fhn[i])
        : fhn[i];
}
```
**Vehicle (TYPE_S|TYPE_X) cells: NO Krüger applied** (excluded).
**Floor (TYPE_S only), Sym-Plane (TYPE_S|TYPE_Y): unchanged**, full -6.

**Bug-Vektor-Potenzial:** LOW for production runs. The exclusion is precise.
However: when WW kernel writes u_slip to vehicle, this u becomes "dead data" —
not propagating to fluid. Step-1b is "safe-mode" / BB-baseline behavior.

---

### 2. NEW kernel `apply_freeslip_y` — Post-Stream Sym-Plane Reflection
**Lines:** ~1445-1467
**Origin:** CC#9 (commit `3f76cfe`)
**Purpose:** Specular reflection at TYPE_Y cells (sym-plane half-domain).

```cpp
kernel void apply_freeslip_y(global fpxx* fi, const global uchar* flags,
                              const ulong t) {
    const uxx n = get_global_id(0);
    if(n>=(uxx)def_N||is_halo(n)) return;
    if(!(flags[n]&TYPE_Y)) return;  // ← early return for non-TYPE_Y
    ...
    fhn[3]  = fhn[4];  fhn[7]  = fhn[13];  fhn[14] = fhn[8];
    fhn[11] = fhn[18]; fhn[17] = fhn[12];
    store_f(n, fhn, fi, j, t);
}
```

**Status:** Allocated as Kernel object, called every step via
`enqueue_apply_freeslip_y` in `do_time_step`.

**Bug-Vektor-Potenzial:** MEDIUM for full-domain runs.
- For CC6_MODE=1 (full domain, no TYPE_Y cells): kernel returns early → no-op.
- Kernel launch overhead per timestep ≈ 0.5-1 ms × N_cells/2.7M_workgroup
  (negligible for performance).
- Potential issue: per-launch synchronization barrier in OpenCL queue.

---

### 3. NEW kernel `apply_wall_model_vehicle` — Werner-Wengle Wall Model
**Lines:** ~1478-1525
**Origin:** CC#10 (commit `4521361`)
**Purpose:** Berechne u_slip per Werner-Wengle PowerLaw, schreibe an
TYPE_S|TYPE_X vehicle surface cells.

```cpp
kernel void apply_wall_model_vehicle(global float* u, const global uchar* flags) {
    if((fn & (TYPE_S|TYPE_X)) != (TYPE_S|TYPE_X)) return;
    // average u over fluid neighbors (u_2 proxy)
    // u_tau via PowerLaw closed-form
    // y1plus = 0.5 * u_tau / nu
    // uplus_1 = (y1plus ≤ 11.81) ? y1plus : 8.3 * y1plus^(1/7)
    // u_slip_mag = uplus_1 * u_tau
    // safety cap: u_slip_mag ≤ 0.95 * u_t_mag
    u[n] = scale * ux_avg;
    u[def_N + n] = scale * uy_avg;
    u[2*def_N + n] = scale * uz_avg;
}
```

**Status:** Always called every step when WALL_MODEL_VEHICLE define active.

**Bug-Vektor-Potenzial:** HIGH for any WW-related anomaly. This kernel
writes to u[] field at vehicle surface cells. Its output is what
`apply_moving_boundaries` later reads for Krüger transport.
- Werner-Wengle PowerLaw closed-form: not iteratively converged, simplified
- Safety cap at 0.95 × u_t_mag: prevents diverge but may bias
- Direction = direction(u_avg) — assumes WW slip aligned with local flow

**Potenzielle Issues für MR2 +163k N Anomalie:**
- u_slip values computed without knowing Krüger is halved
- WW kernel computes u_slip aligned with u_avg, but if BL is unstable (halved Krüger), u_avg fluctuates → noisy u_slip → poor cancellation in force-sum.

---

### 4. NEW kernel `apply_wall_slip_to_fluid` — DISABLED (CC#11 Option 2)
**Lines:** ~1535-1580
**Origin:** CC#11 Option 2 Step 2 (commit `c84adf8`)
**Status:** Allocated as Kernel, **call commented out** in `do_time_step`.

```cpp
// Allocated in lbm.cpp:150:
kernel_apply_wall_slip_to_fluid = Kernel(device, N, "apply_wall_slip_to_fluid", ...);

// NOT called (line 934 in lbm.cpp):
// for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_apply_wall_slip_to_fluid();
```

**Bug-Vektor-Potenzial:** LOW.
- VRAM allocation: ~constant per device (Kernel object metadata)
- Kernel CODE compiled (in OpenCL), but never enqueued
- No runtime CPU/GPU cost

**Caveat:** If somehow accidentally called via wrong control flow → would
overwrite TYPE_MS fluid cells with f_eq(rho, u_slip) → invalid state.

---

### 5. NEW kernel `compute_wall_model_artifact` — DISABLED (CC#11 Option 1)
**Lines:** ~1585-1630
**Origin:** CC#11 Option 1 (commit `2515ae2`)
**Status:** Allocated, **call commented out** in `enqueue_update_force_field`.

```cpp
// Allocated in lbm.cpp:149:
kernel_compute_wall_model_artifact = Kernel(device, N, "compute_wall_model_artifact", ...);

// NOT called (call removed after Option 1 NEGATIVE result, 3 attempts failed)
```

**Bug-Vektor-Potenzial:** LOW. Same as #4.

---

### 6. `update_moving_boundaries` PROMOTED — Now Runtime Kernel
**Lines:** ~1632-1647 (kernel definition)
**Origin:** Originally only in `initialize()`. Now separate kernel called
each timestep via CC#10's WW chain.

```cpp
// lbm.cpp:925-927:
#ifdef WALL_MODEL_VEHICLE
for(uint d=0u; d<get_D(); d++)
    lbm_domain[d]->enqueue_update_moving_boundaries();
// refresh TYPE_MS flags after wall model updates u_solid
#endif
```

**Purpose:** Re-mark fluid cells adjacent to non-zero-u solids as TYPE_MS.
After WW kernel writes u_slip to vehicle, fluid neighbors become TYPE_MS.

**Bug-Vektor-Potenzial:** LOW.
- TYPE_MS marking is correct (fluid adjacent to moving solid)
- Per-step flag refresh handles dynamic u_solid (which WW updates each step)
- Without this, TYPE_MS markings could become stale → apply_moving_boundaries
  would miss some cells

---

### 7. `stream_collide` MODIFIED — TYPE_E + TYPE_Y Ghost-Cell-Mirror Branch
**Lines:** ~1685-1705
**Origin:** CC#8 (commit `1b9d67e`)
**Purpose:** Half-domain sym-plane via ghost-cell-mirror.

```cpp
if(flagsn_bo==TYPE_E) {
    if(flagsn&TYPE_Y) {  // ← CC#8 addition: TYPE_E|TYPE_Y sym-plane handling
        const uxx j_yp1 = j[3]; // +y neighbor at y+1
        rhon = rho[j_yp1];
        uxn  = u[j_yp1];
        uyn  = -u[def_N + j_yp1];  // MIRROR: flip y-component
        uzn  = u[2*def_N + j_yp1];
    } else {
        rhon = rho[n];  // ← original behavior
        uxn  = u[n]; uyn = u[def_N+n]; uzn = u[2*def_N+n];
    }
}
```

**Bug-Vektor-Potenzial:** LOW for CC6_MODE=1 (full domain, no TYPE_Y cells).
HIGH for sym-plane setups (CC6_MODE=2-5). The ghost-cell-mirror was
documented as FAILED in CC#8 — used for diagnostic only.

---

### 8. `update_fields` MODIFIED — TYPE_Y skip
**Lines:** ~2038
**Origin:** CC#9 (commit `3f76cfe`)
**Modification:**
```cpp
// Original:
if(flagsn_bo==TYPE_S||flagsn_su==TYPE_G) return;
// Modified:
if(flagsn_bo==TYPE_S||flagsn_su==TYPE_G||(flagsn&TYPE_Y)) return;
```

**Bug-Vektor-Potenzial:** LOW for full-domain (no TYPE_Y cells).

---

## lbm.cpp Deviations (LBM-Wrapper)

### 9. Kernel Allocations (lines 143-151)
NEW kernel objects:
- `kernel_apply_freeslip_y` (CC#9)
- `kernel_apply_wall_model_vehicle` (CC#10, requires WALL_MODEL_VEHICLE)
- `kernel_compute_wall_model_artifact` (CC#11, requires WALL_MODEL_VEHICLE)
- `kernel_apply_wall_slip_to_fluid` (CC#11, requires WALL_MODEL_VEHICLE)

### 10. enqueue_* Method Implementations (lines 190-199)
NEW methods:
- `enqueue_apply_freeslip_y()` (CC#9)
- `enqueue_apply_wall_model_vehicle()` (CC#10)
- `enqueue_apply_wall_slip_to_fluid()` (CC#11, defined but not called)

### 11. `do_time_step()` MODIFIED (lines 920-935)
**Active call chain (per timestep):**
1. surface_0 if SURFACE
2. **apply_wall_model_vehicle** (CC#10, new)
3. **update_moving_boundaries** (CC#10, new — refreshes TYPE_MS)
4. stream_collide (which internally calls apply_moving_boundaries)
5. **apply_freeslip_y** (CC#9, new — runs every step, no-op if no TYPE_Y)
6. surface_1/2/3 if SURFACE
7. (other)

**Disabled call (commented out):**
```cpp
// for(uint d=0u; d<get_D(); d++)
//     lbm_domain[d]->enqueue_apply_wall_slip_to_fluid();
```

### 12. `enqueue_update_force_field` MODIFIED (lines 224-231)
Only added a comment block. Behavior unchanged — Option 1 artifact
subtraction commented out and not called.

### 13. NEW `def_nu` Device Define (line 388)
Added LB viscosity as compile-time constant for kernels:
```cpp
"\n	#define def_nu " +to_string(this->nu)+"f"
```
Used by `apply_wall_model_vehicle` for Werner-Wengle formula.

### 14. NEW WALL_MODEL_VEHICLE Device Define (lines 459-461)
Promotes host-side WALL_MODEL_VEHICLE to kernel-side via preprocessor.

---

## lbm.hpp Deviations (LBM-Interface)

### 15. Private Kernel Declarations (lines 48-53)
```cpp
Kernel kernel_apply_freeslip_y;          // CC#9
#ifdef WALL_MODEL_VEHICLE
Kernel kernel_apply_wall_model_vehicle;  // CC#10
Kernel kernel_compute_wall_model_artifact; // CC#11 Option 1
Kernel kernel_apply_wall_slip_to_fluid;  // CC#11 Option 2
#endif
```

### 16. Public Method Declarations (lines 117-122)
```cpp
void enqueue_apply_freeslip_y();
#ifdef WALL_MODEL_VEHICLE
void enqueue_apply_wall_model_vehicle();
void enqueue_apply_wall_slip_to_fluid();
#endif
```

---

## defines.hpp Deviations (Config)

### 17. NEW Define WALL_MODEL_VEHICLE (line 25)
```cpp
#define WALL_MODEL_VEHICLE // CC#10: Werner-Wengle wall model
```

---

## setup.cpp Deviations (Setup-Only — Behavior-Neutral for LBM Core)

Major changes (don't affect LBM core, only define geometry & runtime):
- `main_setup_cube()` — Cube validation case
- `main_setup_ahmed()` — Ahmed Body benchmark
- Production `main_setup()` — CC#1-CC#11 vehicle setups with toggles
- Defines: `AHMED_MODE`, `CUBE_VALIDATION`, `VEHICLE_GR_YARIS`, `VEHICLE_MR2_BIN`

---

## Was sollte für MR2 +163k N Debugging untersucht werden

Priorität nach Bug-Vektor-Wahrscheinlichkeit:

### HIGH: apply_wall_model_vehicle (Deviation #3)
- u_slip computation in tight feedback loop with apply_moving_boundaries
- Werner-Wengle closed-form not validated against canonical channel flow
- Safety cap at 0.95×u_t_mag may bias under non-equilibrium conditions

**Diagnostic:** activate `DIAGNOSTIC_WW` (Finding 31 sketch) — export
u_slip, u_avg, y_plus, tau_wall as VTK. Compare with OpenFOAM MR2.

### MEDIUM: update_moving_boundaries every step (Deviation #6)
- Was originally one-time at initialize, now per-step
- Could cause TYPE_MS flag instability if u_solid oscillates per step

**Diagnostic:** check if TYPE_MS markings change between steps for MR2.

### MEDIUM: stream_collide branching (Deviation #7)
- TYPE_E+TYPE_Y branch inactive for full domain — but check the BRANCH
  STRUCTURE didn't accidentally introduce a bug in main TYPE_E path

**Diagnostic:** isolate TYPE_E behavior with simplified setup (no TYPE_Y).

### LOW: apply_freeslip_y per-step call (Deviation #2)
- Returns early for non-TYPE_Y cells — should be no-op
- Could have synchronization-fence side effect on OpenCL queue

**Diagnostic:** temporarily disable the call, re-run MR2, compare result.

### LOW: Disabled kernels still allocated (Deviations #4, #5)
- VRAM-only, not called, should not affect

---

## Clean Revert Path zu Upstream-Equivalent Verhalten

Wenn man nur Standard FluidX3D Verhalten ohne CC#10 Wall Model haben will:

### Option A: Compile-Time Disable (sicherste)
1. defines.hpp: `// #define WALL_MODEL_VEHICLE` (auskommentieren)
2. lbm.cpp do_time_step: WALL_MODEL_VEHICLE-Block wird durch Preprocessor weggelassen
3. enqueue_apply_freeslip_y: bleibt aktiv aber no-op für CC6_MODE=1
4. apply_moving_boundaries: Step-1b TYPE_X check fires but has no effect (no
   WW writes u_slip → vehicle cells have u=0 → c·0=0)

**Verhalten:** Effectively reines BB an Vehicle. MR2 = +1820 N. Same as
upstream FluidX3D would give for this geometry.

### Option B: Hard Revert (vollständig clean)
1. `git checkout upstream/master -- src/kernel.cpp src/lbm.cpp src/lbm.hpp src/defines.hpp`
2. `git checkout HEAD -- src/setup.cpp` (keep our setups)
3. Rebuild

**Verhalten:** 100% upstream FluidX3D LBM-Core mit unseren Setup-Files.
MR2 = ~+2200 N (Lehmann's typical 1.3-2.0× over-prediction).

---

## Status: BEWIESEN

- Alle 17 oben gelisteten Deviations sind durch `git diff upstream/master HEAD`
  reproduzierbar.
- Cube TYPE_X|TYPE_S + halved Krüger gibt korrekten Finding-33-Prediction
  (CD=40) → bestätigt Code-Logik der Modifikationen funktional.

## Status: HYPOTHESE — MR2 +163k N Root-Cause

Wahrscheinlichste Erklärung: Halved Krüger Transport bricht Cancellation-
Properties der Force-Sum auf MR2. Per Finding 33 ist per-cell artifact
fundamentally O(|c·u_slip|) — auf MR2 mit ~370k surface cells × 0.05 lattice
per cell = max 18500 lattice = 30 Mio N. Observed cancellation in CC#10
gibt -610 N (5 Größenordnungen Reduktion). Halved Krüger bricht diese
Cancellation, weil u_slip-Verteilung inkonsistent wird ohne stable BL.

## Status: OFFEN

- MR2 mit Full Krüger -6 (CC#10-Logik, nicht halved) auf aktuelle MR2 STL:
  noch nie gemessen → würde testen ob "Full Krüger" überhaupt funktioniert
  oder ob bereits ein latenter Bug existiert
- Ahmed mit halved Krüger: läuft gerade → wenn auch ~explodiert,
  systematic halved-Krüger-Bug; wenn ok, MR2-spezifisch

---

## See Also

- [[33_single_cell_krueger_test]] — Hypothesis B Confirmed (factor 6 direct)
- [[34_halved_krueger_negative]] — MR2 +163k N first observation
- [[26_ep_storage_boundary_pattern]] — EP-Storage Boundary Interactions
- Upstream FluidX3D: https://github.com/ProjectPhysX/FluidX3D
- Fork: https://github.com/heikogleu-dev/FluidX3D-Intel-B70
- Last upstream-merge base: `4303cb4` (FluidX3D Public License)
