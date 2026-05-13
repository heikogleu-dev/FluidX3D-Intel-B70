# Finding 34: Halved Krüger NEGATIVE — Iron-Rule-Trigger after Cube Pass + MR2 Catastrophic Failure

**Datum:** 2026-05-13
**Status:** ❌ **Iron-Rule-Trigger nach Test 2/4.** Halved Krüger ist KEIN
universeller Production-Path. Linear interpolation between BB-baseline and
full-Krüger fails — WW-kernel ↔ Krüger feedback is fundamentally non-linear.
**Folge:** Phase B Primary Track → OpenLB Pi-Tensor f_neq Reconstruction.

---

## Test-Strategie Recap

Plan (aus User-Direktive, Option 2 in Phase B Sub-Task 2):
- TYPE_S|TYPE_X = WW-Activation-Toggle (halved Krüger -3 an diesen Cells)
- TYPE_S only = floor/wheels/sym-plane (full Krüger -6, unchanged from Eq 5.27)
- Cube als TYPE_S only voxelisieren → kein WW → BB-Baseline-Verhalten

Code-Edit:
- `kernel.cpp` apply_moving_boundaries (line 1115-1128): Step-1b
  TYPE_X-Exclusion ersetzt durch `wf = (fj & TYPE_X) ? -3.0 : -6.0`
- `setup.cpp` main_setup_cube: cube als `TYPE_S` (vorher `TYPE_S | TYPE_X`),
  `object_force(TYPE_S)` (vorher `TYPE_S | TYPE_X`)

---

## Test 1: Cube — PASSED ✅

| Metric | Wert |
|---|---:|
| Last 50 chunks Fx mean | +1.512 N |
| Last 25 chunks Fx mean | +1.520 N |
| Target Fx | +1.447 N (CD = 1.05 Hoerner) |
| Computed CD | 1.097 |
| Match | **97% within target (5% over)** ✅ |
| Iron-Rule check (CD > 1.5 or < 0.5) | OK |

Cube mit TYPE_S only → apply_wall_model_vehicle skippt (Filter requires TYPE_X)
→ u_solid bleibt 0 → Krüger c·u = 0 → keine Modifikation → pure BB-Baseline.

Erwartet, bestätigt. Floor/wheel Pfad funktioniert.

---

## Test 2: MR2 — CATASTROPHIC FAILURE ❌

| Metric | Wert |
|---|---:|
| Steady-state Fx (steps 4000-5000) | **+163,200 ± 5,000 N** |
| Target Fx | +565 N (±20% = +452..+678 N) |
| Predicted (linear interp BB↔CC#10) | +605 N |
| Actual / Predicted ratio | **270×** |
| Actual / Target ratio | **290×** |
| Iron-Rule check (200 < Fx < 1000) | **❌ TRIGGER, STOP** |

### Detailed Convergence Data

| Step | t_si [s] | Fx [N] | Fy [N] | Fz [N] |
|---:|---:|---:|---:|---:|
|  500 | 0.0125 | 1.680e5 | -5.0 | 1.51e4 |
| 1000 | 0.0250 | 1.583e5 | -5.9 | 1.79e4 |
| 2500 | 0.0625 | 1.588e5 | 48.1 | 1.77e4 |
| 5000 | 0.1250 | 1.688e5 | 46.4 | 1.77e4 |

Fx ist **statistisch stabil bei ~+163k N ab Step 500** (4.6× MR2 BB-baseline
of +1820 N). Nicht ein Transient, sondern stationärer falscher Operating Point.

Fz ebenfalls explosion-level: +18k N (Target -1337 N für MR2 Time-Attack
ground effect).

---

## Linear Interpolation Hypothesis FAILED

**Vor dem Test:**
| Approach | MR2 Fx | Quelle |
|---|---:|---|
| BB Baseline (Step-1b, Krüger excluded) | +1820 N | commit 8222a36 |
| Full Krüger -6 (CC#10) | -610 N | Finding 25 |
| Linear-midpoint prediction (halved Krüger) | +605 N | Naive interpolation |

**Tatsächlich:** +163,200 N — komplett aus dem Interpolations-Range raus.

### Root-Cause Hypothese

Die `apply_wall_model_vehicle` Kernel berechnet u_slip aus Fluid-Nachbarn-
Geschwindigkeiten. Mit halved Krüger:
1. Halved Krüger gibt nur HALBE Wand-Momentum-Injection an Fluid
2. Fluid-Nachbar-Geschwindigkeiten bleiben höher (BL ist weniger geretardiert)
3. WW-Kernel sieht höhere u_avg → berechnet höhere u_slip
4. Halved Krüger × größere u_slip → Artifact-Magnitude bleibt hoch
5. Self-Reinforcing-Loop produziert Operating-Point bei +163k N

**Konzeptionell:** Die WW-Krüger-Kopplung ist NICHT linear in der Krüger-
Coupling-Stärke. Die Werner-Wengle PowerLaw berechnet u_slip nicht in
Kenntnis dass Krüger halbiert wurde — sie überkompensiert.

---

## Cube Test wieso passed?

Cube hatte TYPE_S only (kein TYPE_X). Daher:
- apply_wall_model_vehicle Filter `(fn & (TYPE_S|TYPE_X)) != (TYPE_S|TYPE_X) return`
  → Cube-cells nicht WW-prozessiert → u[wall_cell] bleibt 0
- apply_moving_boundaries: cube cells sind TYPE_S without TYPE_X → wf = w6
  Aber u[wall_cell] = 0 → c · u = 0 → keine Modifikation

Cube war "pure BB" Test des Floor-Path. Kein WW-Krüger-Coupling. Daher gut.

Hätten wir Cube mit TYPE_S|TYPE_X (wie Original-Setup) getestet, hätten wir
ähnliche Über-Korrektur gesehen (vermutlich CD ~ 40+, wie in Finding 33's
Prediction). Cube-TYPE_S-Workaround verschleierte die fundamentale
WW-Coupling-Non-Linearity.

---

## Status: BEWIESEN

- ✅ Cube als TYPE_S only mit halved kernel produziert clean BB-baseline (CD=1.10)
- ❌ MR2 mit halved Krüger explodiert auf +163k N (290× target)
- Linear interpolation BB↔Full-Krüger ist **fundamental falsch**
- WW-Kernel ↔ Krüger Kopplung ist **non-linear in Krüger-Strength**

## Status: HYPOTHESE — Non-linearity Mechanism

Self-reinforcing Feedback: halved Krüger → underdamped BL → higher u_avg →
higher u_slip → larger artifact than naively halved. Reasonable but
not empirically isolated.

Verification (would require): Run with HALF u_slip prescription externally
(not via WW kernel) + halved Krüger. If result is +605 N (linear pred),
hypothesis confirmed. Not in scope here.

## Status: OFFEN — Ahmed and Yaris

Test 3 (Ahmed 25°) and Test 4 (Yaris) **not executed** due to Iron-Rule
trigger after Test 2. Both vehicles use TYPE_S|TYPE_X marker similar to MR2
→ same coupling expected → likely similar failure.

If user wants empirical confirmation: 2-3 more runs ahead. Not recommended
given clear failure mode of fundamental mechanism.

---

## Path Forward Recommendation

### Phase B: Halved Krüger CANCELLED

The user's Iron Rule 4 ("Bei Crash oder unklare Resultate: STOP, Heiko
fragen") triggered. Halving the Krüger coefficient is NOT a viable
production fix:
- Cube paradox: cannot be tested without TYPE_X marker workaround
- MR2 catastrophic failure: non-linear coupling makes interpolation invalid
- No reasonable calibration parameter to tune (factor -3 vs -2 vs -4 likely
  all produce similarly bad results)

### Phase C: OpenLB Pi-Tensor f_neq Reconstruction (Primary Track)

The remaining structural fix per Finding 25 hierarchy:
- Modifies fluid-side DDFs (post-stream) using local stress tensor
- No Krüger correction at solid cells → no force artifact
- Geometry-agnostic mathematical framework
- 1-2 weeks implementation, validated by OpenLB literature

### Fallback: Accept BB-Baseline (Step-1b Safe-State)

Current production state (commit 8222a36):
- TYPE_X-Exclusion in apply_moving_boundaries
- Pure BB drag, no WW reduction, no artifact
- MR2 BB = +1820 N (3.2× over target, but stable and known)
- Acceptable for development if WW work is paused

---

## Code-Stand nach Iron-Rule-Trigger

Files restored from `.before-halved` backups:
- `src/kernel.cpp` ← Step-1b TYPE_X-Exclusion logic active (kein halved Krüger)
- `src/setup.cpp` ← CUBE_VALIDATION=0, cube wieder TYPE_S|TYPE_X, kein active Test
- Rebuild verified clean compile
- Repo state matches commit 8222a36 exactly

Backups deleted after restore.

---

## See Also

- [[33_single_cell_krueger_test]] — Hypothesis B confirmed (factor 6 direct)
- [[25_ww_six_failed_attempts]] — Original WW attempt journey
- [[32_force_code_audit]] — Section 6 single-cell test outcome
- Failed run log: `/tmp/mr2_halved.log`
- Cube success CSV: `bin/forces_cube_val.csv` (last write)
- MR2 failure CSV: `bin/forces_cc6_full.csv` (last write)
