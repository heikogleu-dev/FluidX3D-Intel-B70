# Finding 38: Bouzidi Sub-Grid Bounce-Back — Implementation Plan

**Datum:** 2026-05-13
**Status:** Phase C-B initiated. Design + Step 1 (Tier 1 Poiseuille smoke test).
**Methodik:** CLAUDE.md Rules — Skalen-Ladder (Tier 1→4), Smoke-Test first,
Root-Cause-vor-Pivot. Bouzidi is q-aware BB, orthogonal to halved-Krüger.

---

## Bouzidi Math Recap

**Standard BB** assumes wall at exactly 0.5 cell-spacing from cell-center. Incoming DDF
f_i reflects to outgoing f_(opp_i) at the wall:
```
f_(opp_i)_new = f_i_post   // reflect with no shift
```

**Bouzidi 2001 linear interpolation** with actual sub-cell wall distance q ∈ [0, 1]:

```
if q < 0.5:
    f_(opp_i)_new = 2q × f_i_post  +  (1−2q) × f_i_post_neighbor
else:
    f_(opp_i)_new = (1/(2q)) × f_i_post  +  (1 − 1/(2q)) × f_(opp_i)_neighbor
```

Where:
- `f_i_post`: post-collision DDF in direction i at current cell (this fluid)
- `f_i_post_neighbor`: post-collision DDF in direction i at 1 cell BEHIND current
   (away from wall)
- `f_(opp_i)_neighbor`: post-collision DDF in direction opp_i at 1 cell BEHIND

**Reduces to standard BB** when q = 0.5.

**Accuracy:** O(h²) at the wall, vs O(h) for standard BB. Reduces drag overshoot
typical of half-cell wall-distance approximation.

---

## FluidX3D Integration Approach

### Where to insert Bouzidi

`apply_moving_boundaries` (kernel.cpp:1115-1128) already modifies fluid DDFs adjacent
to TYPE_S cells. Bouzidi can replace/augment this:

**Current Step-1b code** at fluid cell with TYPE_S neighbor in direction j[i+1]:
```cpp
// Krüger correction: fhn[i] += w6 × c · u_solid
// (= no-op for stationary wall where u_solid=0)
```

**Bouzidi extension** (linear, axis-aligned wall first):
```cpp
const float q = q_buffer[cell_idx][i]; // per-direction per-cell sub-cell distance
const float f_i_post = fhn[i];               // already loaded
const float f_i_neighbor = ...;               // load from j[i+1u]'s neighbor
const float f_opp_neighbor = ...;             // load from cell's f_opp at neighbor

if (q < 0.5f) {
    fhn[i+1u] = 2.0f*q * f_i_post + (1.0f-2.0f*q) * f_i_neighbor;
} else {
    fhn[i+1u] = (0.5f/q) * f_i_post + (1.0f - 0.5f/q) * f_opp_neighbor;
}
```

**Issue:** Requires accessing DDFs from 2-cells-away (`_neighbor` terms). Esoteric-Pull
storage makes this tricky. Need careful indexing.

### Simpler alternative: post-stream kernel

Run Bouzidi as separate kernel AFTER stream_collide. Reads pre-stream + post-stream
DDFs from fluid cells adjacent to TYPE_S. Modifies post-stream DDFs in place.

**Pros:** Cleaner separation. Direct access to needed DDF values.
**Cons:** Extra kernel launch overhead. Doubles DDF memory traffic.

For initial Step 1 (Poiseuille): use the simpler post-stream-kernel approach.

---

## q-Storage Architecture

For Step 1 (Poiseuille hardcoded q test): no storage needed — q is a runtime constant.

For Step 2+ (per-cell q per direction):

### Option A: Sparse Storage
- Allocate `Memory<float> bouzidi_q` of size N_wall_cells × 18 (D3Q19 non-rest)
- Allocate `Memory<uint> wall_cell_idx` of size N → maps global cell → bouzidi_q index
- Wall-cell list precomputed at voxelization
- VRAM: ~370k cells × 18 × 4B + 337M × 4B = 28 MB + 1.35 GB ≈ 1.4 GB
- Lookup buffer dominates at 1.35 GB. **Borderline acceptable** for B70 28GB budget.

### Option B: Bit-packed in flags
- Reuse 2 unused bits in flags for "has_q_data" marker
- Store q values in a separate dense buffer at TYPE_MS cells only
- Still need indirection lookup → similar VRAM

### Option C: Dense per-cell scalar (no per-direction)
- Single q per cell (average over directions, or normal-direction only)
- VRAM: 337M × 4B = 1.35 GB (single buffer)
- Loss of per-direction precision but simpler
- **Acceptable for first implementation**, refinement later

**Decision for Step 2:** Start with **Option A (sparse per-direction)** as canonical
Bouzidi. Fall back to Option C if VRAM becomes constraint.

---

## Voxelization Extension Design

Existing `voxelize_mesh_on_device(Mesh*, flag, ...)` ray-casts through STL and sets
binary flag. Extension:

```cpp
void voxelize_mesh_on_device(const Mesh* mesh, uchar flag,
                              const float3& rotation_center = 0,
                              const float3& linear_velocity = 0,
                              const float3& rotational_velocity = 0,
                              Memory<float>* bouzidi_q = nullptr);   // ← NEW
```

When `bouzidi_q != nullptr`:
- Ray-cast through STL not just for inside/outside test but also intersection distance
- For each fluid cell with TYPE_S neighbor, compute q in that direction
- Write q to bouzidi_q buffer at the appropriate sparse index

**Complexity:** Existing ray-cast logic exists internally. Need to expose
per-direction intersection distance. Likely 20-40 lines of additional ray-cast
code.

---

## Tier 1 Smoke Test: 2D Poiseuille with Offset Wall

**Goal:** Verify Bouzidi formula produces lower L2-error than BB for known analytical
case. No q-storage needed (hardcoded q at wall cells).

### Setup
- 2D D2Q9 channel (or D3Q19 with periodic Y/Z, focus on Z-flow)
- Channel height H lattice units (e.g., H=64)
- Wall at z=q (q ∈ {0.1, 0.3, 0.5, 0.7, 0.9}) and z=H-q
- Driven by volume force `f_x` (existing Poiseuille setup pattern)
- Periodic in X (or use long X-direction for spatial average)

### Analytical Reference
Parabolic velocity profile (incompressible, fully-developed laminar):
```
u(z) = u_max × (1 - ((z - z_center) / (H_eff/2))²)
```
where z_center = (q + (H-q))/2 = H/2 (centered), H_eff = H - 2q.

u_max = f_x × H_eff² / (8 ν) (Hagen-Poiseuille for plane channel).

### Test Variants
1. **BB-baseline**: standard FluidX3D, q = 0.5 implicit. Reference error level.
2. **Bouzidi (q != 0.5)**: vary q ∈ {0.1, 0.3, 0.7, 0.9}, apply Bouzidi formula.
3. **Bouzidi (q = 0.5)**: should reproduce BB exactly (correctness check).

### Pass Criteria
- L2-error vs analytical: **Bouzidi < BB by factor 3-10×** (for q ≠ 0.5)
- Bouzidi @ q=0.5 ≡ BB (within numerical precision)

### Iron Rules
- Max 3 attempts per variant before STOP for diagnosis
- Smoke-test 100 steps BEFORE 10000-step full convergence run
- Sanity-check first: BB-baseline gives expected error level (a few %)

---

## Step 1 Implementation Outline

### Files to modify
- `src/setup.cpp`: New `main_setup_bouzidi_poiseuille()` function (under `#define BOUZIDI_TEST 1` toggle, dispatch in `main_setup()`)
- `src/kernel.cpp`: New kernel `apply_bouzidi_2d_poiseuille()` (single-direction hardcoded q)
- `src/lbm.cpp`: Allocate kernel + enqueue method
- `src/lbm.hpp`: Method declarations

### Step 1 Code Sketch

In kernel.cpp:
```cpp
kernel void apply_bouzidi_2d_poiseuille(global fpxx* fi, const global uchar* flags,
                                         const ulong t, const float q_wall) {
    const uxx n = get_global_id(0);
    if(n>=(uxx)def_N || is_halo(n)) return;
    if((flags[n] & TYPE_BO) != TYPE_F) return; // only fluid cells
    // Check if neighbor in -z direction (i=6 in D3Q19) is TYPE_S
    uxx j[def_velocity_set];
    neighbors(n, j);
    if((flags[j[6]] & TYPE_BO) != TYPE_S) return;

    // Load post-stream DDFs
    float fhn[def_velocity_set];
    load_f(n, fhn, fi, j, t);

    // Bouzidi for direction 5 (i.e. -z DDF) — incoming from wall
    // f_5 = incoming +z direction, f_6 = outgoing -z direction
    const float q = q_wall;
    const float f_5_post = fhn[5];
    const float f_5_neighbor = load_f_single(...); // requires extra access
    const float f_6_neighbor = load_f_single(...);

    if(q < 0.5f) {
        fhn[6] = 2.0f*q * f_5_post + (1.0f-2.0f*q) * f_5_neighbor;
    } else {
        fhn[6] = (0.5f/q) * f_5_post + (1.0f - 0.5f/q) * f_6_neighbor;
    }

    store_f(n, fhn, fi, j, t);
}
```

**Complexity:** Need `load_f_single` helper or manual computation. ~40 lines kernel.

### Step 1 Validation
- Run for 5000 steps (Poiseuille converges fast)
- Output velocity profile u(z) along Z at mid-channel
- Compute L2-error vs analytical for q ∈ {0.1, 0.3, 0.5, 0.7, 0.9}
- CSV: `bin/poiseuille_bouzidi_q{value}.csv`

### Step 1 Success Criteria
- All 5 q values produce parabolic profile (not blown up)
- L2-error < 5% for q=0.5 (matches BB)
- L2-error < 5% for q=0.1, 0.9 (Bouzidi correctly handles wall shift)
- (BB without Bouzidi at q=0.1 would give 30-50% error — to be measured for comparison)

---

## Tier-Ladder Plan

| Tier | Test | Reference | Pass Criterion |
|---|---|---|---|
| **1** | 2D Poiseuille hardcoded q | Analytical parabolic profile | L2-error < 5% for q ∈ {0.1..0.9} |
| **2** | Stokes sphere drag (q from voxelization) | F_Stokes = 6πμRu | error < 5% |
| **3** | Ahmed 25° (q from STL voxelization) | ERCOFTAC Cd=0.285 | error < 30% |
| **4** | MR2 (q from STL voxelization) | OpenFOAM Fx=565 N | within 50% (~285-845 N) |

**Iron Rule:** Tier 1 must pass before Tier 2. Tier 2 before Tier 3. No skipping.

---

## Status: PLANNED — Step 1 Implementation Starts Now

Following sections will be filled with implementation results as Tier 1 progresses.

---

## See Also

- [[36_phaseB_diagnostic_complete]] — Three-Attractor pathology (different problem)
- [[37_sphere_q_dependence_refuted]] — Bouzidi orthogonal to halved-Krüger
- [[30_poiseuille_ww_validation_setup]] — Earlier Poiseuille sketch (different purpose)
- Bouzidi 2001: "Momentum transfer of a Boltzmann-lattice fluid with boundaries"
- OpenLB BouzidiVelocity: https://gitlab.com/openlb/release/-/blob/master/src/boundary/bouzidiBoundary.h
