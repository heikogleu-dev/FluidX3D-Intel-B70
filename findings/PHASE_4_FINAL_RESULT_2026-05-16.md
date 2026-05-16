# Phase 4 Aggressive — Final Result (2026-05-16)

**Run:** Mode 3 PERF-G + 4 mm Near / 20 mm Far / 5:1 ratio + Phase 3.1 wall fix + tapered α blending
**Branch:** `master` (commit `840866c`)
**Wallclock:** ~50 min (chunk 1 → auto-stop chunk 87)
**Auto-Stop:** triggered chunk 87 = 8 700 Far-steps, `|dFx_far|/|Fx_far| = 1.890%` over recent 25 chunks

## Final Forces (mean over last 25 chunks, 63–87)

| | Phase 3 (5mm/15mm, 3:1) | **Phase 4 (4mm/20mm, 5:1)** | Δ |
|---|---:|---:|---:|
| **Fx_far** (drag) | 1 484 N | **2 557 N** | **+72 %** ❌ |
| **Fx_near** (drag) | 1 574 N | **1 494 N** | **−5 %** ✓ |
| Fy_far | ~0 | +23 N | ~0 ✓ |
| Fy_near | ~0 | +3.4 N | ~0 ✓ |
| Fz_far | n/a | +379 N (lift) | n/a |
| **Fz_near** (downforce) | −552 N | **−799 N** | **+45 % stärker** ✅ |

OpenFOAM-Target bleibt 400–600 N — Phase 4 Near liegt bei 1494 N, weiter 2.5× drüber.

## Wichtigste Erkenntnis

**Phase 4 spaltet das Force-Bild stark zwischen Far und Near:**

- **Fx_far +72 % gestiegen** — die 20 mm Far-Zellen sind zu coarse für die Vehicle-BL (nur ~1 Far-Cell breit). Drag wird durch Staircase-Voxelisierung + grobe BL systematisch überschätzt. Das ist **kein physikalisches Ergebnis**, sondern numerische Diskretisierungs-Artefakt.
- **Fx_near −5 %** — physikalisch relevant. Phase 4 ist marginal besser im Vehicle-Drag.
- **Fz_near +45 % Downforce** — der echte Gewinn. Erklärt sich durch:
  - 4 mm Near vs 5 mm Phase 3: feinere Underbody-Auflösung (250 cells/m vs 200 cells/m)
  - 20 mm Bodenfreiheit vs 15 mm: mehr Air-Mass-Flow im Unterboden → stärkerer Venturi
  - Tapered Z+ Blending: weniger Constraint vom Top → bessere Druckverteilung über Auto
- **Mode 3 Coupling intakt**: Fx_near stabil, Fy ≈ 0 in beiden Domänen, keine Pathologie.

## Far ist jetzt nur noch Coupling-Träger

Bei 5:1 Ratio ist die Far-Domäne primär da, um Near's TYPE_E-Boundary mit physisch korrekten Inlet/Outlet/Side-Werten zu versorgen. Die Force-Quantität an Far ist:
- **Höher als physisch** durch Coarse-Grid-Diskretisierung
- **Nicht das Ziel-Signal** — wir bewerten Phase 4 Erfolg an Near (4 mm Auflösung am Vehicle)

Das ist ein bewusster Trade-Off: Far-Bandbreite (Coarse 20mm) gegen Near-Auflösung (Fine 4mm) tauschen.

## Force-Trajektorie

| Chunk-Range | Fx_far mean | Fx_near mean | Fz_near mean |
|---|---:|---:|---:|
| 1–10 (transient) | 11 800 N | 5 200 N | wechselnd |
| 30–40 | 3 200 N | 1 700 N | −1 050 N |
| 60–70 | 2 600 N | 1 510 N | −820 N |
| **63–87 (final)** | **2 557 N** | **1 494 N** | **−799 N** |

Konvergenz bei chunk 87 statt erwarteten 100–150 → Auto-Stop griff früher als gedacht. Trotz Fx_far-Oszillationen 1260–3816 N war der 25-Chunk-Mittel innerhalb 2% Toleranz.

## Wirkung der drei orthogonalen Änderungen

| Änderung | Effekt im Run |
|---|---|
| **Phase 3.1** (u_τ + Van Driest) | Saubere Wall-Distance-Formel, kein systematischer u_τ-Overshoot mehr in Layer 2/3. Im Force-Signal nicht direkt isolierbar. |
| **4mm Near / 20mm Far / 5:1** | Größter Effekt: Fz_near +45 %, Fx_near −5 %, Fx_far +72 % (Far-Coarsening). |
| **Tapered α Blending** | Wake-Übergang Near→Far visuell zu prüfen. ParaView-Sichtung pending. |

## Performance

- 33 s/chunk (vs Phase 3 17.6 s/chunk = 1.88× langsamer)
- Cell-step-Ratio: (100×43.9M + 500×378M) / (100×160M + 300×242M) = 193G/88G = 2.19× mehr Cell-Work
- **Compute-bound, NICHT coupling-bound** — 100× mehr Coupling-Calls (durch Tapered Band) dominieren nicht
- Total wallclock 50 min für 87 chunks → 0.57 chunks/min steady-state

## VRAM Auslastung (gemessen aus FluidX3D Memory Usage)

- Far: 2.84 GB
- Near: 24.5 GB
- Wall-adj flag (1 B/cell) + Coupling buffers: ~0.4 GB
- **Total: ~27.4 GB / 32 GB B70** | Headroom 4.6 GB ✓

## Nächste Schritte

1. **ParaView GPU Sichtung Phase 4 vs Phase 3** (parallel laufend, PID 538626) — verifizieren ob Tapered Blending die "Kante" im Wake gemildert hat
2. **Bei zufriedenstellender Sichtung**: Triple-Res mit iGPU als Wake-Extension Outer-Domain (Pioneer-Beitrag, ~2 Tage Aufwand)
3. **Alternative**: Bouzidi BB reaktivieren für Underbody Voxelisierungs-Heilung (orthogonal zu Wall-Model)
4. **Falls Phase 4 visuell nicht überzeugt**: Rollback auf Phase 3 5 mm + tapered Blending isoliert testen

## Files

- Far VTKs (5 files, 43.9 M cells): `export/dr_far/{u,rho,flags,F,mesh}-000008700.vtk`
- Near VTKs (4 files, 378 M cells): `export/dr_near/{u,rho,flags,F}-000043500.vtk`
- Forces CSV: `bin/forces_phase5b_dr_mode3.csv` (87 chunks)
- Log: `/tmp/fluidx3d_phase4.log`

VTK-Header gefixed (post-run via `fix_vtk_origin.py` Far+Near, `fix_vtk_mesh.py` translate=(5.5, 0, 2.25)). Per Memory [[fluidx3d_vtk_multires_quirk]]: globaler `units`-Bug in Source bleibt offen, Post-Run-Fix bleibt vorerst nötig.

## See Also

- [PHASE_3_FINAL_RESULT_2026-05-16.md](PHASE_3_FINAL_RESULT_2026-05-16.md) — direkter Phase-3-Vergleich
- [WALL_VISC_BOOST_PHASE3_PHYSICS_REVIEW_2026-05-16.md](WALL_VISC_BOOST_PHASE3_PHYSICS_REVIEW_2026-05-16.md) — Physics-Review die Phase 3.1 motivierte
- [TODO_2026-05-16.md](TODO_2026-05-16.md) — ursprüngliche Phase-4-Planung
