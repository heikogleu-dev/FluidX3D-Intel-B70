# Current State 2026-05-13 — Strategic Pivot to Multi-Resolution

**Datum:** 2026-05-13
**Status:** WW-Repair-Track gepausiert, Multi-Resolution wird Hauptpfad.
**Rationale:** Wall-Model-Reparatur-Versuche blockierten das eigentliche
Pioneer-Ziel. Multi-Res ist orthogonal zu WW und kann mit Pure-BB validiert
werden.

---

## Branch-Inventar

| Branch | Letzter Commit | Zweck | Status |
|---|---|---|---|
| `master` | `661e85b` | Clean Phase 0 FAIL state | confirmed working baseline |
| `plan-refresh-multires` | `9b8ca3e` | master + Multi-Res roadmap doc | bereit für Pivot-Continuation |
| `phase0-ahmed-validation` | `52e6d94` | WW deep-dive + Bouzidi WIP | preserved, paused for resume |

**Empfohlener Pivot-Workflow:**
- `plan-refresh-multires` als Multi-Res-Arbeits-Branch nutzen
- WW/Bouzidi-Arbeit auf `phase0-ahmed-validation` einfrieren
- Beim Bouzidi-Resume: cherry-pick benötigte Findings vom Phase0 → Multi-Res-Branch

---

## Was wurde seit Phase 0 FAIL (Commit 9b0a36e) versucht

### Chronologische Commit-History auf `phase0-ahmed-validation`

| Commit | Was | Ergebnis |
|---|---|---|
| `e508456` | Phase 0d Bug Verification (3 paths) | WW-Krüger-Artifact identifiziert |
| `2515ae2` | CC#11 Option 1 analytical subtraction | NEGATIV — doesn't generalize |
| `de7fd29` | CC#11 Option 2 Step 1: disable WW u_solid write | diagnosis verified |
| `b2683f1` | CC#11 Option 2 Step 1b: TYPE_X-Exclusion sparse arch | safe-state baseline |
| `c84adf8` | CC#11 Option 2 Step 2: 3 sign-variants tested | all FAILED on cube |
| `42d647e` | CC#11 Option 1 + 2 exhausted | full OpenLB needed |
| `e676970` | findings: option1 calibration failure documented | Iron-Rule trigger |
| `f75bade` | findings 25-27: 6-attempt journey + EP-storage pattern + TYPE_X marker | pioneer-pattern preserved |
| `8222a36` | findings 29-32: Phase A diagnostics (Re/y+, Poiseuille sketch, code-audit) | foundation data |
| `d1131f9` | finding 33: single-cell test confirms Hypothesis B (factor 6 direct) | per-cell math validated |
| `05ad435` | finding 34: halved Krüger NEGATIVE result (MR2 +163k N) | Iron-Rule trigger |
| `3e046f7` | CLAUDE.md: engineering methodology rules added | sprint governance |
| `38cdcce` | finding 35: comprehensive deviation log vs upstream | 17 deviations documented |
| `741a974` | finding 36: Three-Attractor synthesis | Krüger architecturally unfit |
| `798d7db` | README + MODIFICATIONS overhaul (CC#11) | repo aligned with findings |
| `d058564` | finding 37: sphere test refutes q-dependence hypothesis | Re×cells×cancellation is cause |
| `5a256ab` | finding 38: Bouzidi implementation plan | design doc |
| `12291d3` | **Bouzidi Step 1 PASSED** (Poiseuille L2 0.76-3.51%) | Tier 1 validation ✓ |
| `5056846` | README + MODIFICATIONS Phase C-B Step 1 success | repo current |
| `52e6d94` | Bouzidi Step 2A WIP (SoA fix, untested) | **paused per pivot** |

### WW-Varianten getestet (durch Findings 25-36 dokumentiert)

| Approach | MR2 Fx | Verdict |
|---|---:|---|
| Step-1b BB-Baseline (no WW transport at vehicle) | +1820 N | ✓ stable, 3.2× over OpenFOAM 565 N |
| CC#10 Full Krüger -6 (auf alter STL) | +580 N | ✓ matched target but auf ANDERER vehicle.stl |
| CC#10 Full Krüger -6 (aktuelle MR2 STL) | **-610 N** | ❌ negative drag, over-corrected |
| Halved Krüger -3 (Phase B Sub-Task 2) | **+163,000 N** | ❌ unphysical, Three-Attractor pathology |
| Diagnostic A: WW kernel disabled | +1,643 N | ✓ confirms LBM-Core healthy |
| Diagnostic B: Full Krüger -6 on current MR2 | -610 N | matches Finding 25 exact |
| Sphere + halved Krüger | CD=38 (target 0.4, 96× off) | refutes q-dependence hypothesis |

**Conclusion:** Krüger Moving-Wall ist architectural unfit für stationary-wall WW.
Three stable attractors (BB +1820, halved +163k, full -610), no continuum between.
NOT q-dependent. Cancellation breakdown at BL-resolved + high-Re geometries.

---

## Konkreter Stand Bouzidi-Versuch

### Bouzidi Step 1 — PASSED (Tier 1)
- 2D Poiseuille mit hardcoded q ∈ {0.1, 0.3, 0.5, 0.7, 0.9}
- L2-error 0.76-3.51%, alle unter 5% pass criterion
- Bouzidi @ q=0.5 reproduces standard BB
- Commit: `12291d3`
- CSV files: `bin/bouzidi_poiseuille_q{10,30,50,70,90}.csv` (lokal, .gitignore'd)

### Bouzidi Step 2A — PAUSED (UNTESTED)
- Per-cell q-field + generalized 18-direction Bouzidi kernel
- Sphere setup (analytical ray-sphere q computation)
- Initial test FAILED (flow collapsed: Fx -5695 → 1e-5 in one step)
- **Root cause identified:** AoS-vs-SoA index mismatch in q_field access
  (FluidX3D uses SoA: `q[i*def_N + n]`, not AoS: `q[n*velocity_set + i]`)
- **Fix applied** (commit `52e6d94`) but NOT verified by re-run
- Status: code-fix preserved on branch, awaiting Bouzidi-resume to re-validate

### Bouzidi Step 2B+ — NOT STARTED
- voxelize_mesh extension for STL ray-cast q
- Ahmed body validation (Tier 3)
- MR2 validation (Tier 4)

### Resume-Path bei Bouzidi-Wiederaufnahme
1. Switch to `phase0-ahmed-validation` branch
2. Build with `BOUZIDI_SPHERE_TEST = 1` in setup.cpp
3. Run sphere validation, verify SoA fix produces physical CD (~ 0.4 expected at Re=5000)
4. If pass: proceed Step 2B voxelize extension for STL
5. If fail: deeper EP-storage debug needed

---

## Code-State Comparison (Pivot-Branch vs Current)

### `plan-refresh-multires` (Multi-Res Pivot Target)
- Master + Multi-Res roadmap doc
- KEINE WW-Krüger-Artifacts (clean Pure-BB baseline)
- `WALL_MODEL_VEHICLE` define existiert nicht (oder ist nicht aktiv)
- Setup: original Phase 0 vehicle setup

### `phase0-ahmed-validation` (Current, Bouzidi WIP)
- Alle 17 deviations vs upstream FluidX3D dokumentiert (Finding 35)
- `WALL_MODEL_VEHICLE` define aktiv (für WW kernel)
- `BOUZIDI_Q_FIELD` define aktiv (für Step 2A)
- `BOUZIDI_TEST = 0` (Step 1 Poiseuille off)
- `BOUZIDI_SPHERE_TEST = 1` (Step 2A sphere on, untested SoA fix)
- Setup.cpp: CUBE_VALIDATION, AHMED_MODE, MR2/Yaris toggles, Bouzidi Poiseuille + Sphere setups

### Pivot-Recommended Action
- Switch zu `plan-refresh-multires` für Multi-Res Implementation
- WW define dort möglicherweise inaktiv (Pure-BB baseline)
- Sauberer Code-State zum Aufbau der Coupling-Infrastructure

---

## WW als Side-Track in Pivot-Plan

Per Strategic-Pivot-Direktive:
- **Phase 1 Bouzidi**: PAUSED auf `phase0-ahmed-validation`, resume nach Phase 5b
- **Phase 2 Stress-Integration**: nicht gestartet, pausierbar wenn aufgenommen
- **Phasen 0b/0c/3/4**: archiviert bis WW-Repair-Pfad geklärt

WW-Repair vs Multi-Res sind orthogonal:
- WW: korrigiert Force-Magnitude (3.2× BB-overshoot)
- Multi-Res: korrigiert Spatial Resolution + erlaubt Refinement-Studien
- Multi-Res-Konvergenz ist von absoluten Drag-Werten unabhängig
- Pure-BB ist akzeptable Multi-Res-Baseline (konstanter Overshoot-Faktor)

---

## Pioneer-Wert-Erhaltung

**Phase 0 WW-Investigation:** 16 Findings (20-37) dokumentieren architectural
limits von Krüger Moving-Wall als WW-Mechanismus. Diese pioneer-Erkenntnis
ist erhalten auf `phase0-ahmed-validation` branch.

**Bouzidi Step 1:** Tier 1 Validation PASSED — funktionierender Bouzidi-Code
auf Branch verfügbar für späteren Resume.

**Phase 5a-5c Multi-Res:** wird der Pioneer-Beitrag — First documented 3D
iterative Schwarz Multi-Resolution coupling for external automotive aero on
consumer-GPU LBM.

---

## See Also

- [[36_phaseB_diagnostic_complete]] — WW Three-Attractor synthesis
- [[37_sphere_q_dependence_refuted]] — q-dependence hypothesis test
- [[38_bouzidi_implementation_plan]] — Bouzidi Step 1 results + Step 2 design
- Commit `52e6d94` — Bouzidi Step 2A WIP (preserved for resume)
- Strategic Pivot directive received 2026-05-13
