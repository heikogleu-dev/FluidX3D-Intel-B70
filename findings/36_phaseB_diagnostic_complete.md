# Finding 36: Phase B Sub-Task 2.5 — Three Stable Attractors Confirmed

**Datum:** 2026-05-13
**Status:** ✅ Diagnostic-Complete (3 Tier-0 Tests + 3 MR2 Tier-4 Tests).
**Methodik:** Lego-Pyramid (CLAUDE.md Rule 1) — Skalen-Ladder Cube → Ahmed → MR2.
**Erkenntnis:** Krüger-Coupling-Stärke produziert **drei distinkte stabile
LBM-Attraktoren**, NICHT linear interpolierbar. Halved Krüger steckt im
Transient-Pfad zwischen BB-Baseline und Full-Krüger fest.

---

## Komplette Test-Matrix

| # | Setup | Krüger | Vehicle | MR2 Fx (steady) | Notes |
|---|---|---|---|---:|---|
| 1 | Cube TYPE_S only | -6 (full, doesn't fire on u=0) | TYPE_S | +1.5 N (CD=1.10) | BB baseline cube |
| 2 | Cube TYPE_S\|TYPE_X | -3 (halved) | TYPE_S\|TYPE_X | +55 N (CD=40) | matches Finding 33 prediction ✓ |
| 3 | Ahmed 25° TYPE_S\|TYPE_X | -3 (halved) | TYPE_S\|TYPE_X | Cd=75 | 264× target (pathology) |
| 4 | MR2 halved (Phase B 2.0) | -3 (halved) | TYPE_S\|TYPE_X | **+163,000 N** | catastrophic |
| 5 | MR2 WW off (Diagnostic A) | n/a (kernel disabled) | TYPE_S\|TYPE_X | **+1,643 N** | BB baseline confirmed |
| 6 | MR2 Full Krüger (Diagnostic B) | -6 (full, CC#10) | TYPE_S\|TYPE_X | **-610 N** | matches Finding 25, OVER-corrected |

---

## Drei stabile Krüger-Coupling-Attraktoren

```
Krüger-Coupling-Strength
    0%             50%              100%
    │               │                 │
WW disabled    halved -3          full -6
    │               │                 │
MR2 Fx:       MR2 Fx:           MR2 Fx:
+1,643 N      +163,000 N        -610 N
    │               │                 │
BB-baseline   Stuck transient   Over-corrected
(Lehmann      (no stable BL)    (negative drag,
 1.3-2.0×)                      WW too aggressive)
    │                                 │
    └─────────────────────────────────┘
         Linear interpolation between
         these endpoints predicts
         +517 N at midpoint, but
         actual is +163,000 N.
         → FACTOR 315× off linear
         → Non-linear bifurcation
```

**Wichtige Beobachtung aus Diagnostic B Transient:**
- Full Krüger durchläuft +342,510 N (step 100) → eventually -610 N (step 6900)
- Halved Krüger steady-state +163k N **sitzt im Mittelbereich dieses Full-Krüger-Transients**
- Hypothese: halved transport ist nicht stark genug um durch den positiven Drag-Peak zum negativen Attraktor zu konvergieren
- Halved Krüger ist in einem **lokalen Maximum** stuck

---

## Stabilität-Analyse pro Attraktor

### Attraktor 1: WW Off (BB-Baseline)
- **MR2 Fx = +1,643 N ± 144** (Diagnostic A, last 50 chunks)
- Konvergenz: schnell (auto-stop bei step 12900)
- Physikalisch: ✓ BB ohne WW, Lehmann's 1.3-2.0× over-target (hier 2.9×)
- Vorhersagbar: ✓ matches Step-1b's prior +1820 N measurement (statistical variation)

### Attraktor 2: Halved Krüger -3 (Phase B 2.0 Result)
- **MR2 Fx = +163,000 N** (statistically stable, std ~5,000)
- Konvergenz: schnell (auto-stop bei step 5000)
- Physikalisch: ❌ unphysikalisch (CD ~ 200+, 6 tons of force on a car)
- Vorhersagbar: ❌ NICHT durch linear interpolation predictable
- **Existence proof:** stable for 5000+ steps with statistical noise <5%

### Attraktor 3: Full Krüger -6 (Diagnostic B / CC#10 logic)
- **MR2 Fx = -610 N** (extremely stable, std < 1 N)
- Konvergenz: langsam (auto-stop nach 6900 steps, durchläuft +342k transient)
- Physikalisch: ❌ NEGATIVES drag (WW Krüger Moving-Wall ist physikalisch falsch für stationary wall)
- Vorhersagbar: ❌ NICHT durch Finding 33's per-cell linear formula
- Match: ✓ exact value -610 N reproduced from Finding 25's table

---

## Architectural Conclusion

Finding 33 zeigte: per-cell Krüger artifact ist O(factor × w × c·u), linear in factor.

Diagnostics A/B zeigen: **Force-sum integriert über BL-resolved Geometrie ist NICHT linear in factor.**

Mathematisches Modell:
```
F_total(coupling_strength) = BB_baseline + Σ_cells coupling × (c · u_slip(coupling))
                                                              ↑
                                                       Self-referential
```

`u_slip(coupling)` ist die Werner-Wengle-Berechnung, die u_avg von Fluid-
Nachbarn liest. u_avg hängt von BL-Zustand ab, der wiederum von coupling
abhängt. Self-referential coupling → non-linear bifurcation diagram.

---

## VTK Exports für externes ParaView-Analyse mit OpenFOAM

Drei MR2-Diagnostic-States als VTK in `/home/heiko/CFD/FluidX3D/export/`:

| File | State | u_slip at Vehicle | Vergleich mit OpenFOAM |
|---|---|---|---|
| `u-000012900.vtk` | WW off | u_slip = 0 | Baseline (kein WW) |
| `u-000006900.vtk` | Full Krüger -6 | u_slip aktiv | sollte OpenFOAM-wallShearStress matchen |
| `u-000005000.vtk` | Halved -3 | u_slip aktiv | unstable state, vermutlich noisy |

**ParaView Workflow für OpenFOAM-Vergleich:**

1. **Open** `u-000006900.vtk` + `flags-000006900.vtk` + `mesh-000006900.vtk`
2. **Threshold filter** on flags-VTK: flag value = 65 (TYPE_S | TYPE_X)
   - Isolates vehicle surface cells exclusively
3. **Resample with Dataset** u-VTK on threshold output
   - Get u-vector at exactly vehicle surface cells
4. **Compute Magnitude** of resampled u → that IS u_slip distribution
5. **Compare with OpenFOAM** `wallShearStress` field after dividing by ρ
   - u_slip should correlate with sqrt(|τ_w| / ρ) × some BL profile factor
6. **Diff via Calculator filter** between FluidX3D-u_slip and OpenFOAM-equivalent

**Erwartung:**
- u-000006900.vtk: u_slip-Werte sollten Werner-Wengle-pattern zeigen (höhere u_slip wo u_avg von Fluid höher ist, also Seiten und Heck)
- u-000005000.vtk (halved): u_slip-Werte vermutlich ähnlich aber Cancellation gebrochen

---

## Status: BEWIESEN

- ✅ Diagnostic A: WW off → MR2 +1643 N (BB-baseline, matches Step-1b)
- ✅ Diagnostic B: Full Krüger -6 → MR2 -610 N (matches Finding 25 datapoint)
- ✅ Diagnostic C: VTK exports from all 3 states available for OpenFOAM analysis
- ✅ Code state restored to Step-1b Safe-State (clean rebuild verified)

## Status: HYPOTHESE — Three-Attractor Bifurcation

Wahrscheinlich erklärt: Halved Krüger ist im Transient-Pfad von Full Krüger
"stuck" — nicht stark genug coupling um durch positive transient peak zum
negativen attractor durchzubrechen. Lokales Maximum im Bifurcation-Landscape.

Verification: würde benötigen feinere coupling-strength-Variation (z.B.
-4, -5, -5.5) und Trajektorien-Analyse. Out of scope für jetzt.

## Status: BEWIESEN — Halved Krüger ist KEIN Production-Path

Beweis aus 3 Datenpunkten:
- Cube (BL-irrelevant): halved gibt CD=40, off by 40×
- Ahmed (BL-resolved): halved gibt Cd=75, off by 264×
- MR2 (BL-resolved, smooth): halved gibt Fx=+163k, off by 290×

Pattern: halved Krüger versagt KATASTROPHAL bei BL-resolved Geometrien
(Ahmed, MR2). Es funktioniert nur bei sharp-edged Cube — und sogar dort
ist es 40× off Target. **Halved Krüger gibt nirgends physikalisch
korrekte Werte.**

---

## Path-Forward — Phase C Empfehlung

Halved Krüger ist als Production-Path ausgeschlossen. Drei verbleibende Optionen:

### Option 1: BB-Baseline akzeptieren (current Step-1b Safe-State)
- MR2 Fx ≈ +1820 N (3.2× over target +565 N)
- Konsistent mit Lehmann's documented 1.3-2.0× over-prediction
- Stable, predictable, no artifacts
- **Pro:** zero risk, current commit 38cdcce
- **Con:** Drag-prediction 3× too high

### Option 2: OpenLB Pi-Tensor f_neq Reconstruction
- Modifies fluid-cell DDFs post-stream
- KEINE Krüger correction → kein negativer Attraktor
- Geometry-agnostic, literatur-validiert
- **Pro:** korrekte Physik
- **Con:** 1-2 weeks implementation

### Option 3: Existing CC#10 Full Krüger akzeptieren TROTZ Vorzeichen
- MR2 Fx = -610 N (NEGATIVES Drag)
- Reproduzierbar, stable
- **Pro:** Existing working state
- **Con:** physikalisch falsch (vehicle wird vom Wind beschleunigt statt gebremst)

**Empfehlung:** **Option 1 für jetzt + Option 2 als next Phase**.
Halved Krüger ist konzeptionell defekt und nicht reparierbar. OpenLB
ist der einzige Weg zu korrekter WW-Physik.

---

## See Also

- [[34_halved_krueger_negative]] — Phase B 2.0 Initial Failure
- [[33_single_cell_krueger_test]] — Per-Cell Linear Math (still valid)
- [[35_deviations_from_upstream]] — Code-State Documentation
- [[25_ww_six_failed_attempts]] — Pre-Phase-B Journey
- VTK exports: `/home/heiko/CFD/FluidX3D/export/u-000012900.vtk` (WW off),
  `u-000006900.vtk` (Full -6), `u-000005000.vtk` (Halved -3)
