# Phase 6 Blending — A/B-Test α=0.25 vs α=0.50

**Datum:** 2026-05-16
**Setup:** Mode 3 PERF-G + 4mm Near / 20mm Far / 5:1 + Phase 5.1 distance-based wall model + **40-Cell Spiegelrampe Blending**
**Branch:** `master`

## Konzept (User-Vorschlag 2026-05-16)

40 Near-Cell Blending-Band an allen 5 Forward+Back-Coupling-Boundaries:
- **Forward Far→Near**: α(d) linear von **ALPHA_HIGH** (Boundary, d=0) → **0.05** (deep, d=39)
- **Back Near→Far**: α(d) linear von **0.05** (Boundary, dfar=0) → **ALPHA_HIGH** (deep Far cell, dfar=7)

Implementiert in `setup.cpp` Mode 3 run-loop (commit `840866c` rev'd zu Spiegelrampe nach commit `512b1ab`-Stand). Macro `PHASE6_ALPHA_HIGH` umschaltbar.

## Final 25-Chunk-Means

| | Phase 5.1 (no blending) | **Run 6A** (α_high=0.25) | **Run 6B** (α_high=0.50) |
|---|---:|---:|---:|
| Konvergenz-Chunk | 88 | **98** | **103** |
| Wallclock | ~49 min | ~54 min | ~57 min |
| **Fx_far** (Coupling-Träger) | 2 564 | 2 291 | **2 120** |
| **Fx_near** (Drag, physikalisch) | 1 510 | 1 252 | **1 180** |
| **Fz_far** (Lift Far) | +417 | +388 | +344 |
| **Fz_near** (Downforce) | −809 | −877 | **−882** |
| Fy_far / Fy_near | 11 / −9 | 6 / −6 | 20 / −4 |
| **Cd_near** (aus Fx_near, A=1.85 m²) | **1.47** | **1.22** | **1.15** |

## Δ vs Phase 5.1 Baseline

| Größe | Run 6A | **Run 6B** |
|---|---:|---:|
| Drag Reduction Fx_near | **−17.1 %** | **−21.9 %** ✅ |
| Downforce Stärker Fz_near | **+8.4 %** | **+9.0 %** |
| Cd-Reduktion | −17 % | **−22 %** |

## Verdict

**Run 6B (aggressive α=0.50) ist der eindeutige Gewinner.** Bei marginal mehr Konvergenz-Zeit liefert es **6 % weniger Drag und ähnliche Downforce wie 6A**. Cd 1.15 ist signifikant besser als die 1.47 von Phase 5.1.

OpenFOAM-Target bleibt ~Cd 0.4-0.6 (Fx 400-600 N). Wir sind 2× drüber, aber:
- Voxelisierungs-Staircase auf Vehicle (Bouzidi parked)
- 5:1 Far-Coarsening (Triple-Res with iGPU adressiert)
- Wake-Recovery zu kurz (Triple-Res adressiert)

→ **Phase 6 Spiegelrampe Blending validated** als richtiger Pfad. Phase 7 Triple-Res darauf aufbauen.

## Beobachtungen zur Konvergenz

- Run 6A: Auto-Stop bei chunk 98 mit |dFx|/|Fx|=1.057%
- Run 6B: Auto-Stop bei chunk 103 mit |dFx|/|Fx|=1.164%
- Beide langsamer als Phase 5.1 (chunk 88) wegen oszillativeren Fx_far durch stärkere Coupling-Bewegung
- **Fx_near sehr stabil** in beiden Phase-6-Runs (±2 % final 25 chunks)
- **Fz_near stabilisiert** auf physikalisch realistischen Wert (~−880 N = 90 kg Downforce bei 30 m/s)

## Physikalische Interpretation

Phase 6 Blending führt zu:
1. **Schwächere Coupling-Reflektion** an Near-Far-Interface — die 40-Cell-Gradient-Zone glättet die TYPE_E-BC-Diskontinuität
2. **Bessere Near-interne Strömungs-Entwicklung** — Near's high-res Vortex-Detail nicht mehr durch hart aufgeprägte BC-Reflektion gestört
3. **Far profitiert von Near's high-res Content** — Back-Coupling mit α=0.50 bei tiefen Near-Schichten transportiert echte high-res Wake-Info nach Far
4. **Konsequenz**: realistischere Vehicle-Aero-Quantitäten, Drag konvergiert deutlich niedriger

Run 6B's α=0.50 ist **doppelt so stark wie der Phase 5.1 / 5-Mode-3-Default** (α=0.20) — aber gerade DAS löst den Wake-Reflexions-Engpass, weil mehr Coupling-Information transportiert wird.

## Performance

| | Phase 5.1 | 6A | 6B |
|---|---:|---:|---:|
| s/chunk | 33 | 33 | 33 |
| Coupling Calls pro Chunk | 10 | **240** (5 forward × 40 + 5 back × 8) | 240 |
| Total wallclock | 49 min | 54 min | 57 min |

→ 24× mehr Coupling-Calls kosten kaum Wallclock (parallel_for auf kleinen Planes ist sehr günstig). **Blending ist Performance-neutral.**

## VRAM Usage

Unverändert zu Phase 5.1: 27.4 GiB B70 (Near 23.6 + Far 3.4 + Overhead 0.4).

## Konsequenz für Phase 7

**Phase 6B α=0.50 wird Default-Blending** ab Phase 7. Triple-Res mit iGPU baut auf diesem Setup auf:
- Near 4mm / Far 12mm / Coarse 24mm (3:1:2 cascade)
- Near 6.0×2.592×1.608m
- Far 8.808×4.008×2.616m (Y/Z slight shrink für VRAM-Headroom)
- Coarse 16.8×9.0×7.584m on iGPU
- B70 Total Estimate: ~29 GiB (3 GiB Headroom)
- STL-native `si_length = 4.4364f` (per Memory `fluidx3d_stl_native_size`)

Phase 6B Blending-Pattern wird in Phase 7 sowohl Near↔Far ALS AUCH Far↔Coarse angewendet (analog 3:1 und 2:1 Cascade).

## Erwartung Phase 7

Mit Triple-Res zusätzlich zum Phase-6B-Stand:
- Wake-Recovery 2.45 L hinter Tail (vs 1.7 L heute) → erwartete weitere −5..−15 % Drag
- Cd Phase 7 optimistisch: **~1.00–1.05** (von 1.15 Phase 6B)
- Drag prognostisch: **1050-1100 N** (vs Phase 5.1 1510)
- **Σ Reduction Phase 5.1 → Phase 7: −27 bis −30 %**

## Files

- 6A VTKs: `export_phase6A_archive/dr_far/*-000009800.vtk` + `dr_near/*-000049000.vtk`
- 6B VTKs: `export/dr_far/*-000010300.vtk` + `dr_near/*-000051500.vtk`
- 6A Force-CSV: `bin/forces_phase5b_dr_mode3_phase6A.csv` (98 chunks)
- 6B Force-CSV: `bin/forces_phase5b_dr_mode3_phase6B.csv` (103 chunks)
- 6A Log: `/tmp/fluidx3d_phase6A.log`
- 6B Log: `/tmp/fluidx3d_phase6B.log`

## See Also

- [PHASE_5_1_FINAL_RESULT_2026-05-16.md](PHASE_5_1_FINAL_RESULT_2026-05-16.md) — Baseline ohne Blending
- [merge_far_near_DESIGN.md](merge_far_near_DESIGN.md) — alternativer Python-Post-Process-Pfad (jetzt überholt durch In-Sim-Blending)
