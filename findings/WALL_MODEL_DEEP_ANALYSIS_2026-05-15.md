# Wall Model Deep Analysis — Failure Modes & Forward-Path

**Datum:** 2026-05-15 (after WW Multi-Res failure)
**Branch:** `plan-refresh-multires`
**Ziel:** Präzise dokumentieren WAS in jeder bisherigen Wall-Model-Implementation versagt hat und welche Fixes architektonisch denkbar sind.

## TL;DR — 4 distinkte Failure-Modes identifiziert

| # | Failure-Mode | Root Cause | Fix-Möglichkeit |
|---|---|---|---|
| **F1** | Three-Attractor Bifurcation (Krüger -3, -6, off) | Self-referential `u_slip(u_avg(coupling))` | Eliminate self-reference — use pre-computed wall normal + fixed law-of-the-wall |
| **F2** | Negative Drag Steady-State (Full Krüger -6 single-domain) | `u_wall = 0.95 × u_avg` injects forward momentum into wall cells | u_wall should represent SLIP at half-cell, not direct fluid velocity |
| **F3** | Long Transient never escapes in Multi-Res (340k N) | Back-coupling injects pathological transient values across Far↔Near, prevents escape | Run WW only in Mode 1 (no back-coupling) — UNTESTED |
| **F4** | u_avg averaging fails on thin/sharp STL features | Loop over ALL fluid neighbors averages opposite-direction velocities (wing top vs bottom, splitter top vs bottom) | Wall-normal projection — only average neighbors in dominant outward direction |

## Vollständige WW Failure-Historie (chronologisch)

### 2026-05-11: CC#10 SUCCESS (auf älterer vehicle.stl)

- Single-Domain Full-Volldomain (337.5 M cells @ 10mm)
- Krüger −6 (Full)
- **Fx = 580 N** ✓ in OpenFOAM-Range (400-600 N)
- Source: [SESSION_2026-05-11_WW_RESULTS](findings/SESSION_2026-05-11_WW_RESULTS.md)

### 2026-05-12 to 13: 6 Failed Attempts auf aktueller MR2-STL

Per [CURRENT_STATE_2026-05-13](commit 8bca3e6) Tabelle:
- CC#11 Option 1 analytische Subtraktion → doesn't generalize
- CC#11 Option 2 Step 1: disable WW u_solid → diagnosis-only
- CC#11 Option 2 Step 1b: TYPE_X exclusion sparse → baseline +1820 N
- CC#11 Option 2 Step 2: 3 sign-variants on cube → all FAILED
- Finding 33: per-cell test → Hypothesis B confirmed
- Finding 36: **Three-Attractor synthesis** (KEY FINDING)

### 2026-05-13: Three-Attractor confirmed (Finding 36)

| Krüger-Strength | MR2 Fx | Verdict |
|---|---:|---|
| 0 (off) | +1,643 N | Pure-BB baseline (3.2× OpenFOAM) |
| −3 (halved) | **+163,000 N** | Stuck in transient, local maximum |
| −6 (full, single-domain) | **−610 N** | Over-corrected, negative drag |

Bifurcation-Diagram: highly non-linear in coupling strength. Self-referential `u_slip(u_avg(coupling))` creates 3 distinct stable points.

**Critical Insight from Finding 36:** Full Krüger transient passes through **+342,510 N** before settling to −610 N over 6900 steps.

### 2026-05-15: My Mode 3 WW Test

Setup: Multi-Res Mode 3 (Additive Schwarz) + TYPE_S Floor + Full Krüger −6.

Result after 3 chunks (300 steps):
- Fx_far = +292,757 N
- Fx_near = +346,359 N
- Fz_far = +14,429 N (climbing)
- **Sat in Full-Krüger-Transient region** (+342k peak from Finding 36 reproduced)

**Hypothesis:** Mode 3 back-coupling may PREVENT escape from transient → eternal pathology. Possibly Mode 1 (no back-coupling) would let Multi-Res escape to −610 N steady, just like single-domain.

## Tiefe Failure-Analyse pro Mode

### Failure F1: Three-Attractor Self-Reference

**Mathematics:**
```
u_avg = mean(u over fluid neighbors at y_2 = 1.5 lu from wall)
u_tau = WW_PowerLaw(u_avg, ν)            # nonlinear in u_avg
u_slip = u_tau × u+(y_1+)                # depends on u_avg via u_tau
u[wall_cell] = 0.95 × u_avg ê_u_avg      # SAFETY CAP, kicks in always at high Re
```

Wall model UPDATES u[wall], Krüger READS u[wall] in stream_collide, fluid neighbors evolve, NEXT step's u_avg includes those evolved values. **Feedback loop**.

For high-Re aerodynamics: `u_tau` is computed from `u_avg`, but `u_avg` already reflects WW's previous correction. Without external anchor (e.g., free-stream value, surface normal), the system has multiple stable fixed-points.

**Why CC#10 worked on old STL but fails on MR2:**  
Hypothesis: old STL had **smoother surface** → u_avg more uniform across vehicle cells → WW converged to single attractor. MR2 has **sharp features + thin sections** → u_avg distribution multimodal → multiple attractors possible.

**Fix possibility:** Pre-compute **wall normal direction** for each vehicle cell (via voxelization gradient or SDF). Project u_avg onto tangent plane → robust against thin features.

### Failure F2: Negative Drag (−610 N) Steady-State

**Code (kernel.cpp:1553):**
```cpp
u[wall_cell] = scale × u_avg    // scale ≈ 0.95 (safety cap)
```

**Physical interpretation:**  
u_wall = 95% of fluid velocity AT y_2 = 1.5 lu from wall, in DIRECTION of u_avg.

Krüger Moving-Wall then says: fluid sees a wall MOVING at 95% of its own velocity in same direction.

**For a stationary obstacle (vehicle in airflow):**  
This is fundamentally WRONG. u_wall should represent a SLIP velocity AT the wall surface (typically small, e.g., 5-15% of free-stream for log-law). Setting it to 95% effectively REMOVES wall friction → fluid passes obstacle freely → low drag.

But wait — the result is **NEGATIVE drag**, not just low drag. Why?

**Numerical hypothesis:**  
With u_wall ≈ 0.95 × u_avg in u_avg direction, Krüger Δf transfer is BACKWARDS relative to physics:
```
Δf_i = -6 × w_i × ρ × (c_i · u_wall)
```
Fluid neighbors receive momentum boost in u_wall direction. For wall behind vehicle (rear surface), u_avg of fluid is mostly +x (wake flow). u_wall = 0.95 × u_avg also +x. Krüger transfers +x momentum to fluid → accelerates wake → REDUCES base pressure → negative drag.

**Fix possibility:**  
WW should set u_wall = TARGET WALL SLIP, not u_avg. From law-of-the-wall:
- u_slip at wall surface ≈ u_tau × u+(0) = 0 (no-slip on wall itself)
- But for half-way BB, "wall" is at y = 0.5 lu, so u_wall = u_tau × u+(0.5) for that point
- Krüger Moving-Wall transfer makes adjacent fluid see this wall velocity

Currently the code does: `u[wall] = u_tau × u+(0.5) × ê_u_avg = u_slip ê_u_avg`. This IS the slip at y = 0.5. **u_slip is small in absolute terms** because u_tau is small (u_tau ~ 3-5% of u_avg in log law).

But then `u_slip / u_avg = u+(0.5) × u_tau / u_avg ≈ u+(0.5) × (1/u+(1.5))`. For y+(0.5) and y+(1.5), with log-law u+ ∝ ln(y+), this ratio is small. So u_wall SHOULD be small.

BUT the safety cap `u_slip > 0.95 × u_avg → u_slip = 0.95 × u_avg` makes u_wall huge. **The safety cap is the immediate cause of the −610 N attractor.**

**Fix:** Remove the safety cap, OR replace with a more restrictive cap (e.g., u_slip ≤ 0.3 × u_avg).

### Failure F3: Multi-Res Mode 3 Transient Lock-In

**My observation (3 chunks):**  
Multi-Res Mode 3 + WW → Fx ≈ 340k N (Three-Attractor transient region, matches Finding 36's +342k peak).

**Hypothesis:** Mode 3 back-coupling at chunk boundary injects pathological u-values from Far ↔ Near. The injection RESETS the relaxation toward steady-state attractor. → Transient never escapes to −610 N (or wherever the multi-res equivalent attractor is).

**Fix possibility:**  
- **Test Mode 1 + WW**: no back-coupling → maybe escapes to single-domain attractor (negative drag). NOT TESTED YET.
- Disable WW for first N chunks, then enable → bypass transient. Risky.
- WW + extended chunk_far (chunk=500 statt 100) → fewer coupling injections per WW evolution → maybe escapes. UNTESTED.

### Failure F4: u_avg Averaging Breakdown on Thin Features

**MR2 STL has:**
- Splitter lip: thin (~5-10mm) horizontal blade at front
- Wing: thin trailing edge
- Diffuser strakes: thin vertical fins under rear

**At a vehicle voxel ON a thin feature:**
- Fluid neighbors exist on BOTH sides of the feature (above AND below the splitter, e.g.)
- Loop in kernel.cpp:1526 averages over ALL 18 fluid neighbors uniformly
- For splitter cell: u_above (low, stagnation) + u_below (high, Venturi)
- u_avg = mean of these = unphysical "average flow" across the feature

**Effect on WW:**
- u_tau computed from confused u_avg
- u_slip might be very large (if u_below dominates) or near-zero (if cancellation)
- Krüger transfer uses this u_slip → unphysical momentum injection

**Fix possibility — Wall Normal Projection:**

For each TYPE_S|TYPE_X cell, identify the dominant outward direction:
```cpp
// Compute "fluid neighbor count" per direction → outward normal
const float3 n_out = compute_outward_normal(flags, j);  // from cell flags pattern
// Only consider fluid neighbors in +n_out half-space
for(uint i=1u; i<def_velocity_set; i++) {
    if(c_i · n_out > 0 && fluid(j[i])) {
        ux_avg += u[j[i]]; ...
    }
}
```

This avoids averaging across thin features. For splitter cell:
- If outward normal is "+z" (top side of splitter), only sample cells above splitter
- u_avg = u_above (low velocity from stagnation) → physically meaningful

**Implementation effort:** ~1 day. Add normal-detection logic in kernel + adjust averaging.

## Han 2021 Wall Function (Path B) — Reassessed

**WALL_MODEL_RESEARCH.md notes (Risk section):**
> "u_2 Stencil-Problem: 'Zelle 2 in Normalenrichtung' funktioniert sauber nur für ortogonale Wände (Boden, Decke, Symm). Für STL-Vehicle-Surface bräuchte man Sub-Grid Wall Distance (signed distance field) → später."

So **Han 2021's approach ALSO has the wall-normal problem** for complex STL. Han 2021 in their paper only applies wall model to flat floor, NOT to vehicle.

**Implication:** Switching to Han 2021 doesn't automatically fix the MR2 vehicle issue. The architectural problem (wall normal on STL) persists.

For Han 2021 + vehicle: would need same wall-normal computation as Fix F4.

## Forward-Path Recommendations

Ranked by effort × success probability:

### Path I — Test WW with Mode 1 (no back-coupling) — **~30 min**

**Hypothesis:** Mode 3 back-coupling prevents transient escape. Mode 1 might let single-domain Multi-Res converge to similar attractor as single-domain CC#10 (i.e., −610 N).

**Test:** `PHASE_5B_COUPLE_MODE=1` + `WALL_MODEL_VEHICLE` ON, current MR2 STL.

**Outcome interpretation:**
- If Fx → ~−610 N: Multi-Res back-coupling was the F3 cause. Then we KNOW WW can converge but to UNPHYSICAL state (negative drag). Still abandoned, but understood.
- If Fx stays at 340k N: Multi-Res itself is the issue, not back-coupling. WW broken in any Multi-Res setup.
- If Fx → reasonable value (~600 N): WW + Mode 1 + TYPE_S works! Validate further.

**Risk:** Low. Quick test, informative.

### Path II — Implement Wall-Normal Projection (Fix F4) — **~1 day**

**Approach:**  
1. Pre-compute outward normal for each vehicle cell (during voxelization or init step).
2. Modify apply_wall_model_vehicle to only average u over neighbors in +n_out half-space.
3. Re-test on Mode 3 + TYPE_S.

**Outcome:** Likely improves dramatically on MR2 STL by fixing F4 (thin features). May still have F2 (safety cap) but at least u_avg correct.

**Risk:** Medium. New code, needs validation.

### Path III — Remove/Reduce Safety Cap (Fix F2) — **~1 hour**

**Approach:**  
Change `if (u_slip > 0.95 × u_avg) u_slip = 0.95 × u_avg` to:
- `u_slip = min(u_slip, 0.3 × u_avg)` (much smaller cap)
- Or: `u_slip = u_slip` (no cap, trust law-of-the-wall)
- Or: `u_slip = u_tau × u+(0.5)` with safety only against numerical NaN

**Outcome:** Might reduce the negative-drag steady state (less momentum injection into fluid). Won't fix F1 self-reference or F4 thin-feature issues.

**Risk:** Low. Easy revert.

### Path IV — Pi-Tensor f_neq Reconstruction (Han 2021 OpenLB Pattern) — **1-2 weeks**

**Approach:**  
Replace `u_slip + Krüger` with full Pi-tensor wall stress reconstruction. Recommendation from Finding 36.

**Outcome:** Mathematically cleanest. Probably works. But major implementation.

**Risk:** High effort, postpone unless others fail.

### Path V — Accept Pure-BB Mode 3 + TYPE_S Baseline — **0 effort**

Current Production:
- Fx_far = 1477 N, Fx_near = 1481 N (Pure-BB ~2.5× target)
- Fz_far = −32 N (neutral), Fz_near = −556 N (downforce ✓)
- Physik direktional korrekt, Magnitude über Target
- Stabil, validiert, schon committed

For development workflow this is usable. Drag-Magnitude-Korrektur kann separat über Calibration-Faktor (post-processing) erfolgen wenn nötig.

## Empfohlene Reihenfolge

1. **Path I** (Mode 1 + WW test) — 30 min, definitive Klärung ob Mode 3 oder WW selbst die Ursache ist
2. Bei Mode 3 als Schuldiger: **Path III** (safety cap reduce) — try less invasive fix
3. Bei beidem fail: **Path II** (wall normal projection) — substantial fix
4. Bei allem fail: **Path V** (akzeptieren) oder **Path IV** (full Pi-tensor)

## Was JETZT (ohne weitere Runs) gesichert ist

- Mode 3 + TYPE_S Pure-BB baseline (Production-Run committed as `a277377`)
- VTKs gesichert als unabhängige Copies in `export/results_mode3_typeS_2026-05-15/`
- WW disabled, repo clean
- 5 distinkte Finding-Docs dokumentieren Wall-Model-Status

User-Direktive nötig: welcher Pfad (I/II/III/IV/V) als nächstes?

## See Also

- [WW_MULTIRES_FAILURE_2026-05-15](findings/WW_MULTIRES_FAILURE_2026-05-15.md) — meine Mode 3 + WW Test-Daten
- [WALL_MODEL_STATUS_2026-05-15](findings/WALL_MODEL_STATUS_2026-05-15.md) — 7 Pfade-Übersicht
- [WALL_MODEL_RESEARCH](findings/WALL_MODEL_RESEARCH.md) — Approach A/B/C Literaturvergleich
- [SESSION_2026-05-11_WW_RESULTS](findings/SESSION_2026-05-11_WW_RESULTS.md) — CC#10 580 N success
- Commit 741a974 (phase0-ahmed-validation) — Finding 36 Three-Attractor
- Commit 8bca3e6 — CURRENT_STATE 2026-05-13 strategic pivot
