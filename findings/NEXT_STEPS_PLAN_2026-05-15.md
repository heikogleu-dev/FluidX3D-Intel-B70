# Next-Steps Plan — Autonomes Vorgehen mit Diagnostik

**Datum:** 2026-05-15
**Status:** Aktiv. Mein Standard-Workflow ab jetzt.
**User-Direktive:** Selbstständig, mit Diagnostik, Fehlerbehebung, Recherche, Doku, Rollback. Nicht abbrechen ohne Root-Cause.

## Aktueller Stand

- **Mode 3 + α=0.20 Reference-Run läuft** (PID 369949, watcher `bc6vc3g8y` öffnet ParaView)
- **User-Entscheidungen heute:**
  - Performance: kein chunk-Vergrößern (enges Coupling beibehalten)
  - VRAM: kein Y-Shrink nötig (28 GB akzeptabel als Headroom)
- **Offene Bugs / Issues:**
  - H2 Y=0 Voxelisation-Artifact in ALL Near-runs (low priority cosmetic)
  - Pure-BB Floor → BL zu dick → Vor-Fahrzeug-Strömung nur 10 m/s statt 30
  - Pure-BB Vehicle → Drag 2.5× über Target

## Schritt-für-Schritt Plan mit Methodology

### Step 1 — α=0.20 Reference-Run Validation (running)

**Hypothese:** α=0.20 reduziert sichtbare Near→Far Coupling-Kante (User-Beobachtung von Mode 3 α=0.10). Mode 3 Additive Schwarz sollte stabiler sein als Mode 2 sequential bei α=0.20.

**Diagnostik:**
- Forces-Trajektorie via CSV (forces_phase5b_dr_mode3.csv)
- Konvergenz delta-criterion
- Erwartet: chunks_max ≈ 170 (ähnlich Mode 3 + α=0.10 = 174 chunks)
- Auto-stop bei dFx_far/Fx_far < 2%

**Success-Kriterien:**
- Forces stabil im 1300-1600 N Range (wie Mode 3 + α=0.10 Production)
- Keine Oszillation > ±200 N
- Sichtbar reduzierte Kante in ParaView Y=0 Slice

**Failure-Handling:**
- Wenn Forces explodieren (Fx > 10k N): α=0.20 too aggressive für Mode 3 → rollback zu α=0.15
- Wenn Forces unverändert (Fx ≈ 1477 N): α=0.20 hat keinen Effekt → versuche α=0.30 oder andere Lösung
- Wenn Kante visuell unverändert: H3 (Coupling-Diskontinuität) ist nicht die Ursache → andere Hypothese

**Rollback-Punkt:** Aktueller commit a277377 (Mode 3 + α=0.10 Production)

### Step 2 — Floor-Only Wall Model (Path II.5, main task — promoted from Step 3 per User 2026-05-15)

**Hypothese:** Han 2021 Wall Function am Floor (TYPE_S z=0, no TYPE_X) fixt die zu dicke Boundary-Layer am Boden ohne Vehicle-WW-Pathology zu triggern (weil Floor flat = keine F4 thin-feature issues).

**Implementation Plan (siehe `findings/PATH_II_5_FLOOR_WW_DESIGN_2026-05-15.md`):**

Schritt 3a: **Diagnose Pre-Implementation**
- Verify Floor cells haben einheitliche fluid-neighbor pattern (= +z only)
- Verify Y=0 vehicle voxelization issue ist nicht das gleiche Issue

Schritt 3b: **Implementation (~3h)**
- `defines.hpp`: `#define WALL_MODEL_FLOOR`
- `kernel.cpp`: neuer `apply_wall_model_floor` kernel (Han 2021 pattern, relative-velocity formulation für rolling road)
- `lbm.hpp` + `lbm.cpp`: Kernel-init + enqueue
- `do_time_step`: call before stream_collide
- Build + Smoke-Test

Schritt 3c: **Tier-1 Validation Run (5-10 chunks)**
- Hypothese: Forces sollten stabil bleiben (kein Three-Attractor)
- Diagnose-Output: Floor cells u_x distribution
- Success-Kriterium: Forces im 1000-2000 N range (kein 340k Pathology)
- Failure-Handling: kill run, analyze why (z.B. relative-velocity formula wrong → fix, oder TYPE_X wheel-cells get accidentally processed → filter-fix)

Schritt 3d: **Tier-2 Production Run (~50 min)**
- Hypothese: Fx_far reduziert von 1477 N → 1100-1300 N. Vor-Fahrzeug-Strömung > 25 m/s (statt 10 m/s)
- ParaView: Inspect Y=0 slice für floor BL thickness

**Success-Kriterien:**
1. Forces im realistischen Range (1000-2000 N)
2. Floor-BL physikalisch dünn (< 10 cm)
3. Freestream vor Fahrzeug = ~28-30 m/s

**Failure-Handling-Cascade:**
- Path II.5 fail → analyze: ist's Krüger-Mechanismus broken? Oder relative-velocity formulation issue? Oder TYPE_S floor conflicting mit kernel filter?
- Bei klaren Fix → fix + retry
- Bei Architektur-Block → dokumentieren als finding_XX_floor_ww_failure, rollback, **WICHTIG: failure ist UNTERSTANDEN** dann erst weiter
- Next: Path II (Wall-Normal Vehicle) oder Path IV (Pi-Tensor Han 2021)

### Step 3 — Vehicle-Wall-Model (only if Floor-WW succeeds) — promoted from Step 4

**Bedingung:** Step 3 must produce physikalisch korrekte Floor-BL (Tier-2 passed).

**Hypothese:** Sparse Wall-Normal-Projection am Vehicle (Path II) reduziert Drag-Overprediction von 2.5× auf ~1.3× (Lehmann-Range).

**Implementation:** ~1-2 days. Pre-compute outward normal pro vehicle cell, modify u_avg averaging to only consider +n_out half-space neighbors.

**Diagnostik:** Compare Fx mit OpenFOAM RANS reference (400-600 N erwartet).

**Failure-Handling:** Wenn Three-Attractor return → rollback, sammle Daten, evaluate Path IV (Pi-Tensor).

### Step 4 — Performance (lower priority per User) — promoted from Step 5

Nur wenn alles obige funktioniert:
- PERF-F V5 Multi-Stream PCIe Pipelining (~1 day, 2-3% gain, Phase A schon in opencl.hpp)
- PERF-H GPU-Resident Coupling (~3-5 days, 10-15% gain, refactor shared context)

### Step 5 — H2 Voxelisation-Fix (DEMOTED per User 2026-05-15) — cosmetic only

**Hypothese:** Vehicle Y=0 cell-center alignment in Near (cell 269 exakt) erzeugt fp-noise-Voxelisierungs-Artefakt am Y=0 Slice.

**Begründung Demotion:** Forces sind unverändert (2.5mm offset = 0.13% des vehicle width), nur Y=0 Slice-Visualisierung betroffen. Aerodynamisch irrelevant.

**Code-Change (1 Zeile):**
```cpp
const float near_vehicle_y_center_cell = (0.0f - (-1.345f)) / dx_near + 0.5f; // 269.0 → 269.5 (off-grid)
```

**Diagnostik:** Visualizer Y=0 slice nach Run, Vergleich mit ALL previous runs.

**Success-Kriterien:** Y=0 slice zeigt clean flow ohne phantom Roof→Wing reversal.

**Failure-Handling:**
- Wenn artifact bleibt: H2 ist nicht (allein) die Ursache → check H3 (z_max coupling plane) via comparing Y=0 of Far vs Near

**Rollback:** trivial 1-line revert.

## Stop-Conditions

Ich **stoppe und konsultiere User** wenn:
1. Architekturänderung der core LBM-Pipeline nötig wird (z.B. shared OpenCL context refactor)
2. >2 unabhängige Fix-Versuche ohne Verständnis fail
3. Path-Aufwand > 1 day in einem Schritt
4. User-Direktive konflikt entsteht (z.B. "VRAM sparen" vs "wall model braucht buffer")

Ich **dokumentiere ohne stop** wenn:
1. Failure verstanden + Root-Cause klar
2. Fix-Versuch < 30 min absehbar
3. Rollback eindeutig zu funktionierendem state möglich

## Aktuelle Todo-Reihenfolge (2026-05-15 reorder per User)

1. **Warte α=0.20 Run ab** (laufend) → Watcher öffnet ParaView
2. **Sichte α=0.20 Daten** mit User → User-Go für nächsten Schritt
3. **Floor-Only Wall Model implementieren** → Tier-1 Smoke + Tier-2 Production
4. **Vehicle-WW (Path II)** falls Floor-WW erfolgreich
5. **Performance V5/H** (lower priority)
6. **H2 Y=0 Voxelization-Fix** (LAST priority, cosmetic only)

Bei jedem Schritt: Hypothese + Failure-Handling per [feedback_diagnostic_methodology] memory.

Bei jedem Run: zugehöriger finding-doc mit Hypothese vorher, Daten nachher, Verdict.

## Recherche-Quellen wenn ich nicht weiterkomme

- [WALL_MODEL_RESEARCH.md](findings/WALL_MODEL_RESEARCH.md) — Approach A/B/C details
- [PATH_IV_PI_TENSOR_DESIGN](findings/PATH_IV_PI_TENSOR_DESIGN_2026-05-15.md) — Han 2021 paper details
- Krüger LBM Book Ch. 5.3.4 (Moving Wall BC) - meine Notizen + Web
- OpenLB Documentation (wallFunctionBoundaryPostProcessors3D)
- FluidX3D upstream commits (für API-Patterns)
- Git history phase0-ahmed-validation branch (für vergangene WW-Versuche)

## See Also

- [WALL_MODEL_DEEP_ANALYSIS_2026-05-15](findings/WALL_MODEL_DEEP_ANALYSIS_2026-05-15.md) — F1-F4 Failure Modes
- [PATH_II_5_FLOOR_WW_DESIGN_2026-05-15](findings/PATH_II_5_FLOOR_WW_DESIGN_2026-05-15.md) — Floor-WW Detail-Design
- Memory [feedback_diagnostic_methodology] — Standard-Workflow
