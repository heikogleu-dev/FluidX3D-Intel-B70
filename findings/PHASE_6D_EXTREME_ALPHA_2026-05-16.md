# Phase 6D — Extreme α 0.0↔1.0 + Plateau (autonomous test result)

**Datum:** 2026-05-16 abend
**Setup:** Mode 3 PERF-G + Phase 5.1 distance-based wall model + Phase 6C-style Plateau-Ramp Blending mit **α-Range 0.0↔1.0** (statt 6C 0.05↔0.50)
**Branch:** `master` (commit 2bd2526 enthält Phase 7 code; 6D selber wurde mit -DPHASE6_ALPHA_HIGH=1.0f -DPHASE6_ALPHA_LOW=0.0f gebaut)

## Konzept

Test: Wie verhält sich das Coupling, wenn am Boundary α exakt 0.0/1.0 (hard overwrite/no update) verwendet wird, aber mit 1-Far-Cell-Plateau und linear-Ramp ins Innere?

```
Forward Far→Near:
  d = 0..4 (Plateau): α = 1.00 konstant   (Far überschreibt Near hart)
  d = 5..39:          α = 1.00 → 0.00 linear

Back Near→Far:
  dfar = 0 (Plateau): α = 0.00 konstant   (Far ignoriert Near komplett)
  dfar = 1..7:        α = 0.00 → 1.00 linear
```

## Final 25-Chunk-Means (Chunks 33-57)

| | 6C (0.05↔0.50 + Plateau) | **6D (0.0↔1.0 + Plateau)** | Δ |
|---|---:|---:|---:|
| Konvergenz-Chunk | 103 | **57** | 2× schneller |
| Wallclock | 58 min | **~26 min** | 2× schneller |
| Convergence Tol | 1.54 % | **7.87 % σ/μ Fx_near** | 5× volatiler |
| Fx_far | 2 183 | **3 269** | +49.8 % |
| **Fx_near** | 1 179 | **2 037** | **+72.8 %** |
| Fz_far | +350 | **+29** | −92 % |
| **Fz_near** | −892 | **−1 379** | **+54.6 % Downforce** |
| **Cd (A=1.85 m²)** | 1.16 | **2.00** | **+73 %** |

## Δ vs Phase 6C (Plateau-Effekt unter Extreme α)

| Größe | 6C → 6D | Physikalische Interpretation |
|---|---:|---|
| Fx_near | 1179 → 2037 (+73%) | α=1.0 forward überschreibt Near's BL hart mit Far's 20mm-Daten → Vehicle "sieht" zu coarse pressure-gradient → Drag artificiell hoch |
| Fz_near | −892 → −1379 (+55%) | Downforce stärker, weil Underbody-Strömung von Far's Floor-BC dominiert |
| σ/μ Fx_near | 1.54 → 7.87% | Lokale Schwingungen zwischen Coupling-Boundary und Vehicle |
| Konvergenz | 103 → 57 chunks | Trigger feuerte früh **wegen** der höheren Schwingungs-Amplitude im Sliding-Window-Mean — System ist **nicht** echt konvergent, nur das Konvergenz-Kriterium (2% Δ Mean) wird durch Oszillation maskiert |

## Verdict

**Phase 6D ist numerisch stabil aber produziert ein PHYSIKALISCH SCHLECHTERES Resultat als 6C:**

1. **+73 % Drag**: α=1.0 forward macht das Vehicle "blind" für Near's eigene BL — Far's 20mm-Pressure-Gradient diktiert die Near-Boundary, was Near's 4mm-BL-Resolution-Vorteil ZERSTÖRT
2. **5× volatiler**: Sub-chunk-Oszillation hoch, weil Boundary-State nach jedem Coupling-Pass komplett anders aussieht
3. **Convergence-Tol trigger ist falsch positiv**: System ist nicht physikalisch konvergent — Trigger wurde nur durch Mean-over-25-Window-Glättung erreicht trotz hoher Schwingungs-Amplitude
4. **Falsche Richtung**: Wir wollen Cd RUNTER (Phase 5.1 1.47 → 6C 1.16 → Ziel < 1.0). 6D bringt 2.00 → **doppelter Drag**

## Phase 7-Konsequenz

**Phase 7 wird mit Phase 6C-α-Werten (0.05↔0.50) konfiguriert**, nicht mit 6D's extreme α. Der Plateau-Mechanismus aus 6C ist sauber, aber das α-Range muss moderat bleiben damit die feinere Resolution-Auflösung wirklich zur Geltung kommt.

In `src/setup.cpp` Phase 7-Funktion sind bereits ALPHA_LOW=0.05f / ALPHA_HIGH=0.50f als Default eingebaut.

## Files

- VTKs: `export/dr_far/{u,rho,F,flags,mesh}-000005700.vtk` + `export/dr_near/{u,rho,F,flags}-000028500.vtk`
- Force-CSV: `bin/forces_phase5b_dr_mode3_phase6D.csv` (57 chunks)
- Log: `/tmp/fluidx3d_phase6D.log`
- ParaView open: PID 641914

## See Also

- [PHASE_6C_PLATEAU_2026-05-16.md](PHASE_6C_PLATEAU_2026-05-16.md) — Phase 6C (besseres Default)
- [PHASE_7_TRIPLE_RES_DESIGN_2026-05-16.md](PHASE_7_TRIPLE_RES_DESIGN_2026-05-16.md) — Phase 7 Design (code complete)
