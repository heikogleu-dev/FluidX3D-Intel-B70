# Phase 5a — Sponge Layer Implementation

**Datum:** 2026-05-13
**Branch:** `plan-refresh-multires` (next commit)
**Status:** Implementation complete, smoke test in progress.

## Goal

Non-reflecting outlet als Voraussetzung für Multi-Resolution-Coupling.
Verhindert Wake-Reflection bei kompakten Near-Field-Boxes durch graduelle
DDF-Damping-Zone vor TYPE_E Outlet.

## Approach

**DDF-Equilibrium-Blending** (statt direkter Velocity-Manipulation oder
local viscosity bump):

In Sponge-Zone (letzte `SPONGE_DEPTH_CELLS` in +X Richtung):
```
strength(x) = SPONGE_STRENGTH × (x - x_sponge_start) / SPONGE_DEPTH_CELLS
            ∈ [0, SPONGE_STRENGTH]
fhn[i] = (1 - strength) × fhn[i] + strength × f_eq(rho_local, u_inlet)
```

- strength=0 am Sponge-Start (smooth transition, kein Force-Impact)
- strength=SPONGE_STRENGTH am Outlet (max damping)
- Lokale Dichte ρ wird beibehalten (Mass-Conservation)
- u-Velocity wird zu freestream u_inlet gedämpft
- f_eq via Standard FluidX3D calculate_f_eq()

## Code-Diffs

### `defines.hpp` (3 new defines):
```cpp
#define SPONGE_LAYER         // toggle (commented out for production)
#define SPONGE_DEPTH_CELLS 50 // 5-10% of typical 1500-cell domain
#define SPONGE_STRENGTH 0.5f  // max damping (0=off, 1=full reset)
```

### `kernel.cpp` (1 new kernel):
`apply_sponge_layer(fi, flags, t, u_inlet)` — Post-stream DDF blending.
~30 lines, uses existing `load_f` / `store_f` / `calculate_f_eq`.

### `lbm.hpp`/`lbm.cpp`:
- Kernel allocation, enqueue method
- Public `float sponge_u_inlet = 0.0f` LBM member (0 = disabled, setup
  sets to lbm_u to activate)
- Hook in `do_time_step()` post-stream (after stream_collide + freeslip_y)

### `setup.cpp` (production main_setup):
- `lbm.sponge_u_inlet = lbm_u;` after LBM construction
- `#ifdef SPONGE_LAYER` guard

## Pass Criteria

1. **Build:** clean compile, no warnings ✓
2. **Drag invariant:** Yaris with sponge ≈ Yaris without sponge (within ±10%)
   because sponge is far from vehicle (15m domain, sponge starts at x=14.5m,
   vehicle at x≈4m → sponge ~10m downstream, doesn't affect vehicle drag)
3. **No instability:** simulation runs to convergence without NaN/SEGV
4. **Smooth outlet:** VTK inspection shows velocity field smoothly transitions
   to u_inlet in sponge zone (post-test, ParaView)

## Expected Result

Yaris Pure-BB without sponge: Fx ≈ +1055 N (last 50 chunks, from PIVOT_BB_SANITY)
Yaris Pure-BB with sponge: Fx ≈ same ±10% (sponge zone 10m downstream of vehicle)

If significantly different (>15%): sponge is influencing vehicle drag —
investigate sponge depth/strength or position.

## Multi-Res Use Case (Future Phase 5b)

Mid-Resolution box (e.g. 4L × 2L × 1.5L) has outlet only ~1L behind vehicle.
Wake oscillations would reflect from compact outlet without sponge.
Sponge zone ensures wake dissipates smoothly into freestream before outlet,
preventing artificial pressure waves in the Mid box.

## Iron-Rule Status

- 5a.1: Defines ✓
- 5a.2: Kernel ✓
- 5a.3: Wiring (lbm.hpp, lbm.cpp, setup.cpp) ✓
- 5a.4: Build clean ✓
- 5a.5: Smoke test running
- 5a.6: 30k stability check pending after smoke pass
