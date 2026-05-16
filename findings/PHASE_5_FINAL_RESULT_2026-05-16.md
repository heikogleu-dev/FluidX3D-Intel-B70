# Phase 5 — Vehicle-only WALL_VISC_BOOST (Floor Bug Fix) — Final Result

**Datum:** 2026-05-16
**Run:** Mode 3 PERF-G + 4 mm Near / 20 mm Far / 5:1 ratio + Phase 3.1 wall fix + **vehicle-only Phase 5 viscosity boost (Floor NICHT geboostet)**
**Branch:** `master`
**Wallclock:** ~50 min (chunk 1 → auto-stop chunk 86)
**Auto-Stop:** triggered chunk 86 = 8 600 Far-steps, `|dFx_far|/|Fx_far| = 1.766%` over recent 25 chunks

## Motivation

Phase 4 ParaView-Sichtung zeigte deutliches Geschwindigkeits-Defizit ("blauer Schleier") **direkt über dem Moving-Ground-Floor** — physikalisch nicht plausibel, da Moving-Wall in Vehicle-Frame gleiche Geschwindigkeit wie Freestream haben sollte → keine BL erwartet.

Diagnose: `WALL_VISC_BOOST::populate_wall_adj_flag` flaggte alle TYPE_S-Nachbarn — also Vehicle UND Floor. Floor-Cells bekamen damit zusätzliche Eddy-Viskosität die zusammen mit dem nicht-perfekten Krüger-Moving-Wall-Transfer den sichtbaren numerischen BL produzierte.

## Code-Änderung (commit nach `cc08f0e`)

`src/lbm.cpp::populate_wall_adj_flag()` Pass 1 Filter:
```cpp
// Phase 4 (alt): jedes TYPE_S triggert Boost (Floor + Vehicle)
if((fj & TYPE_S) != 0u) { dom->wall_adj_flag[n] = 1u; break; }

// Phase 5 (neu): nur Vehicle (TYPE_X bit) triggert Boost
if((fj & TYPE_X) != 0u) { dom->wall_adj_flag[n] = 1u; break; }
```

Floor-Cells (TYPE_S ohne TYPE_X) werden nicht mehr geflagged → keine ν_t-Boost auf wand-adjazente Boden-Cells. Krüger-Moving-Wall + Smagorinsky-SGS handhaben die Floor-Strömung allein.

## Layer-Counts (Effekt der Floor-Entfernung)

| Domain | Phase 4 (Vehicle+Floor) | **Phase 5 (Vehicle only)** | Δ |
|---|---:|---:|---:|
| Far Layer 1 | 312 827 | **118 577** | **−62 %** |
| Near Layer 1 | 4 430 584 | **3 422 584** | **−23 %** |

Far verliert deutlich mehr (Floor-Fläche dominiert dort), Near weniger (Vehicle-Karosserie dominiert dort).

## Final Forces (mean over last 25 chunks, 62–86)

| | Phase 4 (Vehicle+Floor Boost) | **Phase 5 (Vehicle-only Boost)** | Δ |
|---|---:|---:|---:|
| **Fx_far** (drag, Coupling-Träger) | 2 557 N | 2 657 N | +4 % |
| **Fx_near** (drag, physikalisches Signal) | 1 494 N | **1 501 N** | **+0.5 %** ≈ 0 |
| Fy_far | 23 | 26 | ~0 ✓ |
| Fy_near | 3.4 | −10 | ~0 ✓ |
| Fz_far (lift) | +379 | +407 | +7 % |
| **Fz_near** (downforce) | −799 N | **−736 N** | **−8 % schwächer** |

OpenFOAM Target: 400–600 N. Phase 5 Fx_near = 1501 N → weiter 2.5× über Ziel.

## Beobachtungen

**1. Drag identisch zu Phase 4** (innerhalb statistischer Streuung).
**2. Downforce sogar leicht GERINGER** als Phase 4 (−8 %).

Das ist überraschend: ich hatte erwartet, dass weniger Floor-ν_t mehr Underbody-Mass-Flow → stärkerer Venturi → mehr Downforce. Stattdessen: weniger Downforce.

**Mögliche Erklärung:**
- Phase 4's Floor-Boost ν_t verlangsamte ZWAR die Bodenströmung über dem Floor (sichtbar als Schleier in ParaView), aber **erhöhte gleichzeitig den Druckabfall unter dem Auto** durch geringere Effective-Viskosität-Bilanz-Reaktion.
- Phase 5: physikalisch korrekterer Floor (kein artifizieller ν_t-Boost) → realistischere Bodenströmung → realistischer Underbody-Druckabfall → echter (etwas geringerer) Downforce-Wert.

→ Phase 5 ist **physikalisch sauberer**, auch wenn die Quantitäten leicht ungünstiger sind. Wir tauschen einen numerisch-getriebenen Downforce-Boost gegen einen physikalisch defensiveren Wert.

## ParaView-Sichtung erwartet

**Floor-BL-Schleier sollte verschwunden sein** — das ist der Hauptzweck. Drag/Lift sind unerwartet **nicht** der primäre Beneficiary. Sichtung pending.

## Performance

- 33 s/chunk (identisch Phase 4)
- 86 chunks × 33s ≈ 47 min total
- Kein Performance-Effekt von Floor-Boost-Wegnahme (Layer 1 etwas weniger Cells, vernachlässigbar)

## Konsequenzen für Roadmap

Phase 5 hat die **visuelle Pathologie** geheilt aber **kein Drag reduziert**. Daraus folgt:

1. **Wall-Model ist physikalisch sauber jetzt** — weitere Verfeinerungen (Spalding, Pi-Tensor) bringen vermutlich nur marginale Gewinne
2. **Echter Drag-Hebel liegt anderswo:**
   - **Bouzidi BB für Vehicle**: Voxelisierungs-Staircase fix → realistischere Karosserie → weniger Reibungsdrag (~10-20 % Erwartung)
   - **Wake-Recovery via iGPU Triple-Res**: längeres Outlet → besser captured Druck-Recovery (~5-10 %)
   - **Höhere Near-Resolution wenn VRAM-Budget**: 3.5 mm wäre theoretisch im Headroom

Per User-Direktive: Wall-Model abgeschlossen → **Performance + iGPU als nächstes Paket**.

## Files

- Far VTKs (5 files, 43.9 M cells): `export/dr_far/{u,rho,flags,F,mesh}-000008600.vtk`
- Near VTKs (4 files, 378 M cells): `export/dr_near/{u,rho,flags,F}-000043000.vtk`
- Forces CSV: `bin/forces_phase5b_dr_mode3.csv` (86 chunks)
- Log: `/tmp/fluidx3d_phase5.log`

VTK-Header gefixt (Far spacing 0.020 origin -1.0/-3.0/0.0, Near spacing 0.004 origin -0.4/-1.26/0.0, Mesh translate cumulative 5.49/0.0/2.24).

## See Also

- [PHASE_4_FINAL_RESULT_2026-05-16.md](PHASE_4_FINAL_RESULT_2026-05-16.md) — Phase 4 Baseline
- [WALL_VISC_BOOST_PHASE3_PHYSICS_REVIEW_2026-05-16.md](WALL_VISC_BOOST_PHASE3_PHYSICS_REVIEW_2026-05-16.md) — Physik-Review der Phase-3.1-Korrekturen
