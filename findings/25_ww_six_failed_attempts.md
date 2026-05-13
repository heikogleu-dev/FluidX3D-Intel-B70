# Finding 25: Wall Model — Six Failed Attempts Journey

**Datum:** 2026-05-13
**Status:** Iron-Rule-Trigger after 6 attempts. Code reverted to Safe-State Step-1b (commit e676970).
**Folge:** Methodical restart with Foundation Validation Phase A (diagnose-data-first, no implementation).

---

## Reference Statement from FluidX3D Author

ProjectPhysX/FluidX3D src/setup.cpp (Ahmed body example):
> "expect Cd to be too large by a factor 1.3-2.0x; need wall model"

Lehmann himself documents that bounce-back without wall model overpredicts
Cd by 30-100%. Our MR2 baseline shows ~3.2× overprediction — at the upper
edge / above Lehmann's range, consistent with our higher Re (~9M vs Ahmed's
~700k).

---

## Attempt Matrix

| # | Approach | Cube CD | MR2 Fx | Outcome |
|---|---|---:|---:|---|
| 1 | Option 1 initial (WW write disabled) | n/a | n/a | dead-code (u_n=0 → dF=0) |
| 2 | Option 1 + TYPE_MS fix | -171 | untested | 2× over-subtract |
| 3 | Option 1 factor 6 (cube-calibrated) | 0.4±125 osc | -293880 N | doesn't scale to MR2 |
| 4 | Option 2 Krueger +6 | 28.7 | — | flow accelerated |
| 5 | Option 2 Krueger -6 | -43.9 | — | negative drag |
| 6 | Option 2 f_eq reset | 128.9 | — | f_neq destroyed |

---

## Key Architectural Insight

Per-cell analytical subtraction calibrated on **one** geometry (Cube) does
not generalize to **another** geometry (MR2). Reason: per-cell u_slip
direction varies — Cube has mostly uniform +X, MR2 has complex 3D variation
(wake recirculation, side flows, curvature regions).

A correct WW implementation must operate either:
- At fluid post-stream cells (OpenLB f_neq reconstruction), OR
- Via local viscosity modification (Malaspinas-style, eddy viscosity)

NOT via after-the-fact analytical subtraction of force artifact.

---

## Safe-State (commit e676970)

- `apply_moving_boundaries`: TYPE_X-Exclusion AKTIV → kein Krüger an Vehicle-Cells
- `apply_wall_model_vehicle` (WW kernel): u_slip-Berechnung läuft, aber Krüger
  ignoriert TYPE_X-Cells → Output ist "tot"
- `compute_wall_model_artifact` Kernel-Call: DEAKTIVIERT
- `apply_wall_slip_to_fluid` Kernel-Call: DEAKTIVIERT
- Net: Pure bounce-back at vehicle wall, no WW reduction, no artifact

---

## Validation Results in Safe-State

| Body | Cube | MR2 | Yaris |
|---|---:|---:|---:|
| BB Baseline (Fx, N) | +1.4 | +1820 | (BB) |
| Target (Fx, N) | +1.4 | +565 | +463 |
| Overprediction Factor | 1.0× | 3.2× | TBD |
| Lehmann Reference Range | — | 1.3-2.0× | — |

Cube matches Hoerner reference (CD=1.05) — confirms BB is correct for sharp
edges where no WW needed. MR2 overprediction is the WW gap we need to close.

---

## Lessons for Phase A

1. Validate WW-mechanic on canonical test (Poiseuille channel) BEFORE complex
   geometries — caught early what we missed for 6 attempts.
2. Generate diagnostic data (u_slip distribution, y+ values) BEFORE choosing
   implementation method.
3. Compare against trusted reference (Heiko's OpenFOAM MR2) for ground truth.
4. Re-range-dependency: WW correction factor likely Re-dependent, single
   calibration won't fit all geometries.

See [[26_ep_storage_boundary_pattern]] and [[27_typex_force_isolation]] for
the architectural patterns extracted from this attempt series.
