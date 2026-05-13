# Phase 5b-pre: `couple_fields()` Module Architecture — Design Document

**Datum:** 2026-05-13
**Branch:** `plan-refresh-multires`
**Status:** Design. Implementation begins after this doc is reviewed.
**Vorgängerphase:** 5a sponge layer code-ready (opt-in for compact Mid-boxes).

## Goal

Build the **Schwarz Multi-Resolution coupling module** that transfers field data
between two LBM simulations at a shared boundary plane. Phase 5b-pre validates
the data-extraction/re-application pipeline on a **single domain** before
actual two-domain coupling (Phase 5b).

## Architecture

### Data Structures

```cpp
struct PlaneSpec {
    uint3 origin;       // cell-index of plane lower corner (e.g. (200, 0, 0) for X-plane at cell 200)
    uint2 extent;       // plane width × height in cells (Y × Z for X-plane)
    uint axis;          // 0=X-normal, 1=Y-normal, 2=Z-normal
    float cell_size;    // SI cell-spacing (for resolution-ratio computation in Phase 5b)
};

struct CouplingOptions {
    bool smooth_plane;          // apply 2D Gauss smoothing to extracted plane
    int smoothing_kernel_size;  // 3 or 5 (3x3 or 5x5 kernel)
    bool propagate_one_step;    // after writing plane as TYPE_E, run target sim 1 step
    bool export_csv;            // diagnostic CSV: plane_t{step}_stats.csv
    bool export_vtk;            // diagnostic VTK: plane_t{step}.vtk (2D)
};
```

### Function Signature

```cpp
// Extract field data (u, rho) at source_plane, optionally smooth, write to target_plane as TYPE_E BC
void couple_fields(LBM& src_sim, LBM& tgt_sim,
                   const PlaneSpec& src_plane, const PlaneSpec& tgt_plane,
                   const CouplingOptions& opts);

// Self-coupling overload: src_sim == tgt_sim, src_plane == tgt_plane
// Validates the pipeline (should be no-op invariant)
inline void couple_fields_self(LBM& sim, const PlaneSpec& plane, const CouplingOptions& opts) {
    couple_fields(sim, sim, plane, plane, opts);
}
```

## Pipeline (Phase 5b-pre Same-Resolution Same-Sim)

For self-coupling on single LBM:

1. **Sync src→host**: `src_sim.u.read_from_device()`, `src_sim.rho.read_from_device()`
2. **Extract plane data**: For each cell in `src_plane`, copy (u_x, u_y, u_z, rho) into a host `std::vector<PlaneCell>`
3. **(Optional) Smooth**: 2D Gauss kernel applied to each component
4. **Sync tgt→host**: `tgt_sim.flags.read_from_device()` (only if first call — flags rarely change)
5. **Write target plane**: For each cell in `tgt_plane`:
   - Set `tgt_sim.flags[n] = TYPE_E`
   - Set `tgt_sim.u.x[n]/.y[n]/.z[n]` from smoothed plane data
   - Set `tgt_sim.rho[n]` from plane data
6. **Sync host→target**: `tgt_sim.flags.write_to_device()`, `tgt_sim.u.write_to_device()`, `tgt_sim.rho.write_to_device()`
7. **(Optional)** Run 1 target step: `tgt_sim.run(1)` to propagate the new BC into interior
8. **(Optional)** Diagnostics: CSV stats (mean, max, min per component) + VTK 2D export

## Self-Coupling Invariance Test (Phase 5b-pre Validation)

**Setup:**
- Single LBM, Pure-BB Yaris CC#6 full domain (15×5×4.5m, 337M cells)
- Plane at x=200 (mid-domain, away from vehicle which is at x=400-800)
- Run baseline: 5000 steps without coupling → record Fx

**Test:**
- Same setup but with `couple_fields_self(lbm, plane_x200, opts)` called every 100 steps
- Run 5000 steps → record Fx
- Compare with baseline: should differ by < 0.5% (numerical noise only)

**If invariance holds:** Pipeline works. Phase 5b can proceed with confidence.
**If invariance fails:** Bug in extraction/application, fix before Phase 5b.

## Why Self-Coupling First?

- No second sim needed — minimal setup complexity
- Pipeline tested in isolation from cross-resolution interpolation
- If self-coupling fails, two-domain coupling will fail worse
- ~10x faster validation cycle than full Phase 5b

## Implementation Effort Estimate

| Sub-Task | Lines of Code | Time |
|---|---:|---|
| PlaneSpec + CouplingOptions structs in lbm.hpp | ~30 | 30 min |
| couple_fields() body (steps 1-6) | ~80 | 2-3 h |
| 2D Gauss smoothing helper | ~30 | 30 min |
| CSV/VTK plane diagnostics | ~50 | 1-2 h |
| Self-coupling test setup in setup.cpp (#define toggle) | ~60 | 1 h |
| Build + run + validate invariance | — | 1 h |
| **Total** | ~250 | **6-8 h** (1 day) |

## Resolution-Ratio Handling (Future Phase 5b)

For Phase 5b (Double-Res Mid + Far at different cell sizes):
- src_plane.cell_size and tgt_plane.cell_size will differ (e.g. 0.01m and 0.02m)
- Bilinear interpolation needed: tgt cells receive weighted average of src cells in their footprint
- 2:1 ratio: each tgt cell = average of 2×2 src cells (downsample) or replicated to 4 tgt cells (upsample)
- Phase 5b-pre does NOT exercise this path — same-resolution only

## Diagnostic Outputs

CSV per call: `plane_t{step}_stats.csv` with columns
- timestamp, plane_id, n_cells, ux_mean, ux_std, ux_min, ux_max, uy_mean, ..., rho_mean, ...

VTK per call: `plane_t{step}.vtk` — 2D STRUCTURED_POINTS with u_x, u_y, u_z, rho fields.

These are critical for Phase 5b RMS-drift convergence diagnosis later.

## STOPP-Gate Criteria (per pivot directive)

> Nach Phase 5b-pre: Self-Coupling muss invariant sein → sonst STOPP

Strict pass: |Fx_self_coupled - Fx_baseline| / Fx_baseline < 1% (within statistical noise of Yaris run).

## Files to Modify

- **`src/lbm.hpp`**: NEW `struct PlaneSpec` and `struct CouplingOptions` declarations
- **`src/lbm.cpp`** (or new file `src/coupling.cpp`): NEW `couple_fields()` implementation
- **`src/setup.cpp`**: NEW `BOUZIDI_SELF_COUPLING_TEST` toggle + integration in main_setup

Recommended: keep coupling code in `lbm.cpp` (FluidX3D convention, no new files).

## STL for Smoke Test

Per user directive 2026-05-13: use `scenes/vehicle.stl` (canonical MR2 model).
Set `#define VEHICLE_GR_YARIS 0` and ensure `#define VEHICLE_MR2_BIN` is commented out.

## Open Questions for Implementation

1. **rho synchronization:** Is `lbm.rho` automatically updated by stream_collide?
   FluidX3D's UPDATE_FIELDS define controls this. Check defines.hpp — if not active, may need explicit `lbm.update_fields()` before plane read.

2. **TYPE_E BC effect:** Writing TYPE_E to interior cells creates a forced equilibrium boundary mid-domain. In FluidX3D's stream_collide, TYPE_E cells get `f_eq(rho, u)` recomputed each step from stored `rho`/`u`. So setting u/rho once should hold persistently — no need to re-apply every step (unless we want to track time-evolution).

3. **Self-coupling frequency:** Apply every step? Every 100 steps? Every chunk?
   For pipeline test: every 100 steps. For Phase 5b: aligned with outer-loop iterations.

4. **flags persistence:** Once we write `TYPE_E` to interior cells, they stay TYPE_E. If we later want to "decouple," we'd need to write back `TYPE_F`. Not relevant for Phase 5b-pre (single static plane).

## See Also

- Pivot directive Phase 5b-pre: "couple_fields() Modul-Architektur + Self-Coupling Test"
- [[PHASE_5A_SPONGE_IRON_RULE_TRIGGER_2026-05-13]] — sponge ready, default off
- [[PIVOT_BB_SANITY_2026-05-13]] — Pure-BB Yaris baseline +1055 N reference
- waLBerla ExternalForce/ParallelBoundaryHandling (reference)
- OpenFOAM cellMotion / mapFields (offline mapping pattern, relevant for ref)
