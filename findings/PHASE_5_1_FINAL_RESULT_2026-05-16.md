# Phase 5.1 — Distance-based WALL_VISC_BOOST + Floor Restored — Final Result

**Datum:** 2026-05-16
**Run:** Mode 3 PERF-G + 4 mm Near / 20 mm Far / 5:1 ratio + **Phase 5.1 distance-based wall model** (20 mm physical penetration both domains, Vehicle + Floor cells)
**Branch:** `master`
**Wallclock:** ~49 min (chunk 1 → auto-stop chunk 88)
**Auto-Stop:** triggered chunk 88 = 8 800 Far-steps, `|dFx_far|/|Fx_far| = 1.494%`

## Motivation

User-Audit-Frage 2026-05-16: ist die Phase 3-5 hardcoded `MAX_WALL_DISTANCE=3` physikalisch korrekt? Antwort: NEIN.

| Phase | Near Eindringtiefe | Far Eindringtiefe | Konsistenz |
|---|---:|---:|---|
| Phase 3-5 (hardcoded 3 Layer) | 3 × 4 = 12 mm | 3 × 20 = 60 mm | ❌ 5× Diskrepanz |
| **Phase 5.1 (TARGET_BL_DEPTH=20 mm)** | 5 × 4 = **20 mm** | 1 × 20 = **20 mm** | ✓ konsistent |

Plus: Floor-Cells, die in Phase 5 ausgenommen waren, sind wieder im Boost-Bereich — Krüger-Moving-Wall-Transfer-Imperfektion der ersten 1-2 Floor-Cells bekommt Mixing-Length-Damping ohne 60mm-Overshoot.

## Code-Änderungen (commit `a7eaf8f`)

`src/lbm.cpp::populate_wall_adj_flag(const float dx_si_override)`:
- Neue Per-LBM-Signatur — explizites dx_si statt globalem `units` (das in Multi-Res auf Far gesetzt ist)
- `MAX_WALL_DISTANCE = max(1u, ceil(TARGET_BL_DEPTH_SI / dx_si))` — physisch konsistent
- TARGET_BL_DEPTH_SI = 0.020 m (typische Auto-BL bei 30 m/s, Hucho / Pope)
- Layer-count Array dynamisch via `std::vector<ulong>` (statt fixed `[4]` Overflow-Risiko)
- Floor-Cells wieder dabei: `(fj & TYPE_S) != 0u` matcht Vehicle + Floor

`src/setup.cpp`:
- Aufruf `lbm_far.populate_wall_adj_flag(dx_far)` und `lbm_near.populate_wall_adj_flag(dx_near)` mit explizitem dx
- print_info aktualisiert: "Phase 5.1 distance-based"

## Layer-Counts (Per-Domain dx)

| Domain | dx | Layer 1 cells | Total Layers | Total wall-adj % |
|---|---:|---:|---:|---:|
| Far | 20 mm | 312 827 | 1 | 0.713 % |
| Near | 4 mm | (Layer 1) | 5 | (TBD aus log) |

## Final Forces (mean over last 25 chunks, 64–88)

| | Phase 4 (3 Layer V+F) | Phase 5 (3 Layer V only) | **Phase 5.1 (Dist-based 20mm V+F)** | Δ vs Phase 4 |
|---|---:|---:|---:|---:|
| **Fx_far** (drag) | 2 557 N | 2 657 N | **2 564 N** | +0.3 % |
| **Fx_near** (drag, **physical signal**) | 1 494 N | 1 501 N | **1 510 N** | +1.1 % |
| Fy_far | 23 | 26 | 11.2 | ~0 ✓ |
| Fy_near | 3.4 | −10 | −8.5 | ~0 ✓ |
| Fz_far (lift) | +379 | +407 | +417 | +10 % |
| **Fz_near** (downforce) | −799 N | −736 N | **−808.6 N** | **+1 % stronger** |

## Wichtigste Erkenntnis

**Phase 5.1 liefert die BESTE Force-Kombination der drei Varianten:**
- Drag (Fx_near) **marginal höher als Phase 4** (+1.1 %) — wenig Verschlechterung trotz erweiterter ν_t-Region
- Downforce (Fz_near) **leicht besser als Phase 4** (+1 %) und **deutlich besser als Phase 5** (+10 %)
- **Physikalisch defensibel** — konsistente Eindringtiefe über beide Domänen, Mixing-Length-Anwendung im real-existierenden BL-Bereich (20 mm)

Die Distance-Konsistenz fix hat **die physikalische Sauberkeit** ohne Quantitäts-Verschlechterung gebracht. Phase 5 (Floor-only-removal) hatte zu wenig Downforce — Phase 5.1 stellt das wieder her aber **mit korrekter Tiefe**, nicht mit dem fehlerhaften 60mm-Overshoot von Phase 4.

## Beobachtungen zur Force-Trajektorie

- Fx_far oszilliert weiter wild (700–4180 N) — Phänomen bleibt: 20 mm Far ist zu coarse für die Vehicle-BL, Wake-Topology bei Auto-X+ schwankt stark
- Fx_near stabil bei 1490–1590 N (±3%) — Near's 4 mm liefert reproduzierbare Drag-Quantität
- Fz_near stabil bei −750…−920 N (±10%) — Underbody/Top-Pressure-Verteilung gut definiert

## Konsequenz für Roadmap

Wandmodell-Iteration ist physikalisch **abgeschlossen**:
1. ✅ Phase 3.1 (u_τ-Closed-Form mit y_lu + Van Driest) — physikalisch korrekt
2. ✅ Phase 5.1 (distance-based + Floor) — Multi-Res-konsistent

Weitere Wand-Verfeinerungen (Spalding, Pi-Tensor, Sigma-SGS) bringen marginal — der echte Hebel liegt nun bei:
- **Phase 6**: User-Blending-Konzept (40-Cell-Spiegelrampe mit α 0.05↔0.25) → Wake-Kante glätten
- **Phase 7**: Triple-Res mit iGPU (32 mm Coarse, 18.4×9.2×7.6 m Box) → Wake-Recovery massiv verbessern → erwartete Drag-Reduktion 5-10 %

## Performance

- 33 s/chunk (gleich wie Phase 4/5)
- 88 chunks × 33 s ≈ 48 min wallclock
- Compute-bound, nicht coupling-bound
- Per-LBM dx_si Bugfix in `populate_wall_adj_flag` jetzt für Multi-Res-Setups robust

## VRAM

- Far: 2.84 GB
- Near: 24.5 GB
- Wall-adj flag + Coupling-Buffers: ~0.4 GB
- **Total: ~27.4 GB / 32 GB B70** | Headroom 4.6 GB ✓

## Files

- Far VTKs: `export/dr_far/{u,rho,flags,F,mesh}-000008800.vtk` (5 files)
- Near VTKs: `export/dr_near/{u,rho,flags,F}-000044000.vtk` (4 files)
- Forces CSV: `bin/forces_phase5b_dr_mode3.csv` (88 chunks)
- Log: `/tmp/fluidx3d_phase5_1.log`

VTK-Header gefixt + Mesh-Translate-Korrektur kumulativ (5.49, 0, 2.24) für korrekte Visualisierungs-Ausrichtung an Voxel-Treppen.

## See Also

- [PHASE_5_FINAL_RESULT_2026-05-16.md](PHASE_5_FINAL_RESULT_2026-05-16.md) — Phase 5 (vehicle-only) Vergleich
- [PHASE_4_FINAL_RESULT_2026-05-16.md](PHASE_4_FINAL_RESULT_2026-05-16.md) — Phase 4 (3 Layer Vehicle+Floor) Vergleich
- [WALL_VISC_BOOST_PHASE3_PHYSICS_REVIEW_2026-05-16.md](WALL_VISC_BOOST_PHASE3_PHYSICS_REVIEW_2026-05-16.md) — physikalische Begründung von Van Driest + y_lu
