# Phase 5a Sponge — Iron-Rule-Trigger nach 3 Attempts

**Datum:** 2026-05-13
**Branch:** `plan-refresh-multires`
**Status:** ❌ Iron-Rule-Trigger. 3 sponge-Varianten alle ~26% des Baseline-Drag.

## Test-Matrix

| Variant | Formula | Strength | Yaris Fx (last 50) |
|---|---|---:|---:|
| Baseline (no sponge) | — | — | **+1055 N (std 455)** |
| v1 | blend toward f_eq(u_inlet) | 0.5 | +278 N (std 495) |
| v2 | blend toward f_eq(u_inlet) | 0.01 | +275 N (std 495) |
| v3 | blend toward f_eq(u_local) — f_neq damping | 0.1 | +276 N (std 495) |

All 3 sponge variants converge to ~276 N — **74% drag reduction**. Statistically identical despite formula + strength differences.

## Root-Cause (informed analysis)

**Sponge zone position vs wake length:**
- Full Domain: 15m × 5m × 4.5m, vehicle at x=4–8m, sponge at x=14.5–15m
- Wake of bluff body extends 3–5× vehicle length = 12–20m downstream
- Sponge cell range x=14.5–15m CATCHES the wake recirculation

**Mechanism:**
- Any sponge formula at any non-trivial strength accumulates over thousands of timesteps
- f_neq damping kills wake oscillations cumulatively even at strength=0.01
- Result: wake recirculation cut off → pressure deficit behind vehicle reduced → drag drops
- The 74% reduction is the wake-deficit contribution; the residual ~276 N is friction + nose pressure

**Why all 3 variants identical:**
- Sponge zone is 6.5m downstream of vehicle
- Local u_local in sponge ≈ u_inlet (wake mostly dissipated)
- So f_eq(u_inlet) ≈ f_eq(u_local) at sponge zone
- All 3 formulas effectively equivalent in this regime

## Implication for Phase 5a goal

**Sponge is NOT needed for full-domain setups** where outlet is far enough from vehicle (here 7m downstream — TYPE_E alone is sufficient, wake naturally dissipates).

**Sponge IS needed for compact Multi-Resolution Mid-boxes** where outlet is close (~1L behind vehicle = 4m). In that geometry, wake STILL extends past the outlet and reflects off TYPE_E without sponge.

## Decision: Iron-Rule Reframe

Per pivot directive Iron-Rule:
> Phase 5a: Sponge Layer muss keinen Einfluss auf Single-Domain Drag haben → sonst STOPP

→ FAIL on full-domain test. STOP triggered.

**Reframe per CLAUDE.md Rule 3 "Root-Cause vor Pivot":**
- Root-cause is understood: full-domain test inappropriate target for sponge validation
- Sponge IS useful for its actual use-case (compact Multi-Res boxes)
- Need different validation target before declaring Phase 5a complete

## Recommended Next Action

### Option A: Validate Sponge on COMPACT Box (Tier-2-correct test)
- Create 4L × 2L × 1.5L Mid-box setup with same Yaris vehicle
- Vehicle x-center at L/2, outlet at 3.5L (only 3L behind vehicle)
- Wake will extend into outlet — sponge becomes necessary
- Compare: Mid-box-no-sponge vs Mid-box-sponge-v3
- Expected: sponge reduces wake-reflection oscillations without (much) changing mean drag
- Aufwand: medium (new setup + 1 short test run)

### Option B: Skip Phase 5a, Directly Phase 5b-pre
- For full-domain initial Multi-Res tests, sponge isn't needed
- Add sponge only when Multi-Res Mid-box geometry triggers visible reflections
- Move directly to `couple_fields()` architecture (Phase 5b-pre)
- Aufwand: low — saves time

### Option C: Different Sponge Design (less aggressive)
- Use grid-stretched buffer zone (PowerFLOW style) — not feasible in FluidX3D uniform grid
- Use Robin BC at outlet (pressure-velocity blending) — non-trivial implementation
- Use viscosity ramp (Smagorinsky-style increased nu in last cells) — likely similar wake-kill issue
- Aufwand: high — uncertain success

## Recommended: Option B (pragmatic) + Option A (when needed)

Move to Phase 5b-pre `couple_fields()` architecture. Re-introduce sponge in Phase 5b when compact Mid-box demonstrates outlet reflection issues. Keep current sponge code in repo as opt-in (`#define SPONGE_LAYER` toggle, default off post-test).

## Code State

- Kernel `apply_sponge_layer` exists and works mechanically
- Default `SPONGE_LAYER` define: stays defined (sponge available)
- `sponge_u_inlet` runtime parameter: default 0.0 (sponge inactive unless setup enables)
- Setup `main_setup()` sets `lbm.sponge_u_inlet = lbm_u` — **needs disable** for production-default
- Per pragmatic choice (Option B): comment out the setup activation line; sponge code stays available
