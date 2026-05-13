# CC#11 Option 1 — Analytical Subtraction Doesn't Generalize Across Geometries

**Datum:** 2026-05-13
**Status:** ❌ Iron-Rule-Trigger (3 attempts), Option 1 strukturell unzureichend
**Folge:** Option 2d (OpenLB Pi-Tensor) oder Phase 1 Bouzidi (Roadmap)

---

## Iteration-Journey

### Attempt 1 — Initial (with WW write disabled)
- WW kernel had `u[n]=u_slip` write DISABLED (Step 2 prep)
- Option 1 kernel ran but read u[n]=0 → c·u=0 → dF=0
- Sign-flip test gave bit-identical results
- **Conclusion:** Kernel ran but did nothing because input was zero

### Attempt 2 — TYPE_MS Bug Fix
- Discovery: `!(fj & (TYPE_S|TYPE_E|TYPE_T))` check WRONGLY excluded TYPE_MS cells
- TYPE_MS = TYPE_S|TYPE_E (0x03), the EXACT cells where Krueger fires
- Fixed: `(fj_bo != TYPE_S && fj_bo != TYPE_E)` catches both pure fluid AND TYPE_MS
- Cube test: F jumped from +110 to -109 → subtraction now ACTIVE
- **But 2× over-subtracted** (artifact was +110, subtracted +220)

### Attempt 3 — Factor Calibration
- Halved formula factor from +12 to +6
- Cube test: F_mean ≈ 0.5 N (target baseline 1.4 N) ✓ within noise
- **BUT per-step oscillation ±125 N** around mean
- MR2 test: F = **-293,880 N catastrophic**

### Iron-Rule-Trigger
Per spec: "Maximum 3 Ansätze pro Phase-Subtask. Bei 3× Fail: Subtask als 'blocked' markieren, weiter zur nächsten Phase."

3 attempts: ✗ (initial dead-code) ✗ (bug fix, calibration off) ✗ (factor 6, doesn't scale to MR2)

→ Option 1 ist **blocked**.

---

## Architektonische Erkenntnis

Mein per-Cell-Formel:
```
ΔF_artifact = Σ_(corrected_DDFs) c_i × 2 × Δf_i × constant_factor
```

Funktioniert konzeptionell, aber praktisch:

**Cube:** Wenige flache Faces, u_slip mostly +X (uniform), per-Cell-Beitrag berechenbar, faktorisiert.

**MR2:** Viele curved Surfaces, u_slip variiert stark per Cell. Cells in Wake haben u_avg in -X (recirculation). Cells im Drag-Region haben +X. Cells am Side haben ±Y. 

→ Per-Cell-Beiträge addieren sich nicht-trivial. Mein Formel-Faktor (kalibriert auf Cube) **gibt für MR2 falsche Magnitude**.

Konzeptionell brauche ich nicht eine GLOBAL-konstante Faktorisierung sondern eine GEOMETRY-EXACT Subtraction die das Krüger-Detail im EP-Storage reflektiert. Das geht nur via gleichem Algorithmus wie Krüger selbst.

---

## Verifikation: Cube vs MR2 Empirisch

| Setup | Cube CD | Cube Fx | MR2 Fx | Status |
|---|---:|---:|---:|---|
| WW disabled (write off) | 1.10 | +1.5 N | +1820 N | ✓ Baseline |
| WW + Krüger (no subtract) | 80 | +110 N | -610 N | ❌ Artifact |
| WW + Krüger + Option1 (factor 12) | -171 | -236 N | (untested) | ❌ Over-subtract |
| **WW + Krüger + Option1 (factor 6)** | **0.4 (osz±91)** | **±125 N** | **−293,880 N** | ❌ Doesn't scale |
| Krüger TYPE_X-excluded (Step 1b safe) | 1.10 | +1.5 N | +1820 N | = baseline, no WW effect |

---

## Path Forward — Sortiert nach Heiko's "best to worst" Reihenfolge

### Best: Full OpenLB Regularized BC (1-2 Wochen)
- Echte WW-Lösung
- Pi-Tensor f_neq Reconstruction
- Komplex aber im Literatur empirisch validiert (Latt-Chopard, OpenLB, Han 2021)
- Direct Modification an Fluid-Cell DDFs ohne moving-wall trick
- Würde Drag-Reduktion-Effekt korrekt produzieren

### Better: Phase 1 Bouzidi Interpolated Bounce-Back (1-2 Wo, schon Roadmap)
- Sub-grid distance-aware BB
- Kompatibel mit Esoteric-Pull
- Reduziert Geometrie-Discretization-Fehler
- Kann mit existierendem WW-Kernel kombiniert werden (TYPE_X-exclusion bleibt, neue Sub-Grid-Logik)

### Pragmatic: Akzeptiere Bounce-Back-Baseline
- Aktueller Safe-State (Step 1b)
- MR2 Fx ≈ +1820 N (3× over target 565 N — typischer BB-Overshoot bei coarse-grid)
- KEINE WW-Reduction, KEIN Artefakt, KEIN Sign-Bug
- Phase 1 Bouzidi als nächste Stufe

### Worst: Empirical Per-Geometry-Calibration
- Bestimme Faktor pro Vehicle-STL einzeln durch Cube/Box-Calibration runs
- Fragil, nicht skalierbar, hässlich

---

## Code-Status nach Iron-Rule-Trigger

- `apply_moving_boundaries`: TYPE_X-Exclusion AKTIV (kein Krüger an Vehicle WW-Cells)
- `apply_wall_model_vehicle` (WW kernel): schreibt u_slip in u[wall_cell] (jetzt nur "Daten", da Krüger TYPE_X überspringt)
- `compute_wall_model_artifact` Kernel-Call: DEAKTIVIERT in update_force_field
- `apply_wall_slip_to_fluid` Kernel-Call: DEAKTIVIERT in do_time_step
- Effekt: Pure bounce-back at vehicle wall. Force-Baseline ohne Artefakt.

Code-Tree enthält alle drei Kernels als Dokumentation/Referenz, falls man später wieder darauf zurück will.
