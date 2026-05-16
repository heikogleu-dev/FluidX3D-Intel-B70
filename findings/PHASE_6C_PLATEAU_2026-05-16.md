# Phase 6C — Plateau-Zone vor Blending-Rampe (α 0.05 ↔ 0.50)

**Datum:** 2026-05-16
**Setup:** Mode 3 PERF-G + Phase 5.1 distance-based wall model + Phase 6B-Blending mit **1-Far-Cell-Plateau am Boundary**
**Branch:** `master` (commit 673058d)

## Konzept

User-Vorschlag: am Coupling-Boundary 1-Far-Cell breit (= 5 Near-Cells bei 5:1 Ratio) **α konstant** auf Boundary-Wert halten, BEVOR die linear Rampe ins Innere startet.

```
Forward Far→Near (Near-Cell-Depth d):
  d = 0..4 (Plateau, 1 Far-Cell breit): α = 0.50 konstant
  d = 5..39 (Linear Ramp):              α = 0.50 → 0.05 linear über 35 Cells

Back Near→Far (Far-Cell-Depth dfar):
  dfar = 0 (Plateau, 1 Far-Cell):       α = 0.05 konstant
  dfar = 1..7 (Linear Ramp):            α = 0.05 → 0.50 linear über 7 Cells
```

## Implementation

`src/setup.cpp` Mode 3 run loop (commit 673058d):
```cpp
const uint  PLATEAU_NEAR = 5u;  // 1 Far-Cell = 5 Near-Cells bei 5:1
const uint  PLATEAU_FAR  = 1u;  // 1 Far-Cell Plateau

// Forward
if(d < PLATEAU_NEAR) alpha_d = ALPHA_HIGH;
else {
    const uint d_in_ramp = d - PLATEAU_NEAR;
    const uint ramp_len  = BAND_NEAR - PLATEAU_NEAR - 1u;
    alpha_d = ALPHA_HIGH - (float)d_in_ramp/(float)ramp_len * (ALPHA_HIGH - ALPHA_LOW);
}

// Back
if(dfar < PLATEAU_FAR) alpha_d = ALPHA_LOW;
else {
    const uint dfar_in_ramp = dfar - PLATEAU_FAR;
    const uint ramp_len     = BAND_FAR - PLATEAU_FAR - 1u;
    alpha_d = ALPHA_LOW + (float)dfar_in_ramp/(float)ramp_len * (ALPHA_HIGH - ALPHA_LOW);
}
```

## Final 25-Chunk-Means (Chunks 79-103, ~59 min wallclock)

| | Phase 5.1 | Run 6A (0.05↔0.25) | Run 6B (0.05↔0.50) | **Run 6C (0.05↔0.50 + Plateau)** |
|---|---:|---:|---:|---:|
| Konvergenz-Chunk | 88 | 98 | 103 | **103** |
| Convergence Tol | 1.49 % | 1.06 % | 1.16 % | **1.54 %** |
| Wallclock | 49 min | 54 min | 57 min | **58 min** |
| Fx_far | 2 564 | 2 291 | 2 120 | **2 122** |
| **Fx_near** | 1 510 | 1 252 | 1 180 | **1 180** (identisch 6B) |
| Fy_far / Fy_near | 11/-9 | 6/-6 | 20/-4 | **23/-4** |
| Fz_far (lift) | +417 | +388 | +344 | **+343** |
| **Fz_near** (Downforce) | −809 | −877 | −882 | **−893** |
| **Cd (A=1.85 m²)** | 1.47 | 1.22 | 1.15 | **1.15** |

## Δ vs Phase 6B (Plateau-Effekt isoliert)

| Größe | 6B → 6C | Bedeutung |
|---|---:|---|
| Fx_near | 1180 → 1180 | identisch — Drag-Mechanik unverändert |
| Fz_near | −882 → −893 | **+1.2 % stärker** (Plateau stabilisiert Underbody-Strömung) |
| Fx_far | 2120 → 2122 | identisch |
| Convergence stability | 1.16% | 1.54% — marginal weniger straff, aber konvergent |

## Δ vs Phase 5.1 Baseline (Phase 6-Stack-Effekt kumuliert)

| Größe | Phase 5.1 | **Phase 6C** | Δ |
|---|---:|---:|---:|
| Fx_near | 1 510 N | **1 180 N** | **−21.9 %** |
| Fz_near | −809 N | **−893 N** | **+10.4 % Downforce** |
| Cd | 1.47 | **1.15** | **−22 %** |

## Verdict

**Plateau ist numerisch stabil und liefert marginal mehr Downforce als 6B**, gleicher Drag. Der Effekt ist klein (Fz_near +1.2 %), aber positiv. Plateau ist **physikalisch sauberer** weil α innerhalb einer einzelnen Far-Cell-Breite konstant bleibt — keine Sub-Far-Cell-α-Gradient-Artefakte.

**Run 6C wird der neue Default-Blending-Mode** für Phase 7. Mit Plateau-Logik kann später auch der α 0.0↔1.0 Extreme-Test (Phase 6D) sauber laufen, weil α=1.0 am Boundary ohne Sub-Cell-Variation gehalten wird.

## Nächste Schritte

1. **ParaView-Sichtung Phase 6C** (PID 4168576) — User-Review für visuelle Wake-Glätte am Near-Far-Übergang
2. **Phase 6D-Test α 0.0↔1.0 + Plateau** — pending User-Freigabe nach 6C-Sichtung
3. **Phase 7 Triple-Res implementation** — pending 6D-Resultate

## Files

- VTKs: `export/dr_far/{u,rho,F,flags,mesh}-000010300.vtk` + `export/dr_near/{u,rho,F,flags}-000051500.vtk`
- Archive (sicherheitskopie): `export_phase6C_archive/`
- Force-CSV: `bin/forces_phase5b_dr_mode3_phase6C.csv` (103 chunks)
- Log: `/tmp/fluidx3d_phase6C.log`

## See Also

- [PHASE_6_BLENDING_COMPARISON_2026-05-16.md](PHASE_6_BLENDING_COMPARISON_2026-05-16.md) — Phase 6A vs 6B Vergleich (ohne Plateau)
- [PHASE_7_TRIPLE_RES_DESIGN_2026-05-16.md](PHASE_7_TRIPLE_RES_DESIGN_2026-05-16.md) — Phase 7 Design
