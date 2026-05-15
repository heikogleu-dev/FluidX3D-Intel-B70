# WW (CC#10) im Multi-Res Mode 3 + TYPE_S Test — FAILURE

**Datum:** 2026-05-15
**Branch:** `plan-refresh-multires`
**Status:** ❌ WW-Krüger reproduziert Three-Attractor Pathology — disabled.

## Test-Setup

- `WALL_MODEL_VEHICLE` aktiv (CC#10 Werner-Wengle + Krüger Moving-Wall)
- Mode 3 (PERF-G Concurrent Additive Schwarz)
- TYPE_S Moving-Wall Floor
- Far 13.5m × 8m × 5m @ 15mm (160 M cells)
- Near 6.6m × 2.7m × 1.695m @ 5mm (242 M cells)
- Vehicle: aktuelle `scenes/vehicle.stl` (MR2 Time-Attack mit Splitter + Diffusor + Wing)

## Ergebnis (3 chunks vor manual stop)

| chunk | Fx_far [N] | Fz_far [N] | Fx_near [N] | Fz_near [N] |
|:-:|---:|---:|---:|---:|
| 1 | 323,990 | +2,795 | 344,966 | −2,565 |
| 2 | 291,656 | +4,155 | 342,279 | −5,284 |
| 3 | 292,757 | +14,429 | 346,359 | −5,145 |

Time-Attack Target: Fx ~600 N, Fz_near ~−1200 N

**Fx ist 500× über Target. Pathology in BEIDEN Domains** trotz unterschiedlicher Resolution (Far 15mm vs Near 5mm).

## Diagnose

Reproduziert die in [finding 36](commit 741a974) dokumentierte "Three-Attractor Pathology" der Krüger Moving-Wall auf der aktuellen MR2-STL:

| Krüger-Variant | MR2 Fx | Verdict |
|---|---:|---|
| Step-1b Baseline (Pure-BB, no WW transport) | +1820 N | 3.2× over OpenFOAM, akzeptabel |
| Full Krüger −6 (alte STL CC#10) | +580 N | ✓ matched target |
| Full Krüger −6 (aktuelle MR2-STL single-domain) | **−610 N** | ❌ over-corrected |
| Halved Krüger −3 (Phase B Sub-Task 2) | **+163,000 N** | ❌ unphysisch |
| **Full Krüger −6 (Multi-Res Mode 3, dieser Test)** | **+292,000 N (Far) / +346,000 N (Near)** | ❌ NEW failure mode |

## User-Vorschlag "WW nur in Near" — implizit getestet

Datenpunkt: **Near (5mm) zeigt 346,359 N** = höher als Far.

Schlussfolgerung: Near-only WW würde NICHT helfen. Die Pathology ist NICHT durch Far's gröbere Resolution verursacht. Sie ist inhärent in der Krüger Moving-Wall Interaktion mit der aktuellen MR2-STL Geometrie:

Vermutete Ursache (nicht final geprüft): 
- **Scharfe Splitter-Lippe** (~5-10mm Dicke) erzeugt extreme u_avg-Gradienten an Vehicle-Surface-Cells
- WW PowerLaw `u_tau ∝ |u_avg|^0.875` skaliert nichtlinear → schon kleine u_avg-Übertreibung führt zu großem u_slip
- Krüger-Forcing `−6 w_i ρ (c_i · u_slip)` amplifiziert weiter
- Mode 3 Back-Coupling wirft Near's pathologische u-Werte als Far-Boundary zurück → positive Feedback-Loop
- Three-Attractor entsteht

Test (NICHT durchgeführt, würde Aufwand benötigen): WW + Mode 1 (one-way) statt Mode 3 → eliminiert Back-Coupling-Loop. Aber Mode 1 + TYPE_E hatte Fx=1257 N stabil → Hypothesentest würde 2 separate Runs brauchen.

## Konsequenz

WW-Krüger ist **definitiv abandoned** für aktuelle MR2-STL. Drei stable Attraktoren gefunden über alle Tests:
1. Pure-BB: ~1820 N (3.2× Target, stabil)
2. WW-Krüger −6: −610 oder +292,000 N (instabil/over-corrected)
3. Halved Krüger −3: +163,000 N (Three-Attractor)

→ Krüger Moving-Wall ist auf MR2 architektonisch unfit.

## Forward-Path

Aus [WALL_MODEL_STATUS_2026-05-15](findings/WALL_MODEL_STATUS_2026-05-15.md), nun mit Update:

- ~~Q1: WW-Calibration auf MR2-STL~~ — eliminiert
- **Q3: Wall Function (Han 2021 / OpenLB-Pattern)** — neuer empfohlener Pfad
  - 0 GB VRAM (wie WW)
  - Spalding/Musker Profile via Newton-Raphson direkt in stream_collide
  - Mathematically robuster (continuous u+ profile, no piecewise switch wie Werner-Wengle)
  - Aufwand 3-5 Tage

Alternative falls Wall Function zu aufwändig: Continue mit Pure-BB Mode 3 + TYPE_S baseline (Fx 1477 N, Fz_near −556 N), akzeptiere 2.5× Drag-Overprediction als Pure-BB-Charakteristik.

## Re-Build State

- `defines.hpp`: `// #define WALL_MODEL_VEHICLE` (kommentiert mit Failure-Note)
- `bin/FluidX3D`: rebuilt ohne WW
- Zurück zu Mode 3 + TYPE_S baseline (174-chunk Production confirmed: Fz_near −556 N downforce)

## See Also

- [WALL_MODEL_STATUS_2026-05-15](findings/WALL_MODEL_STATUS_2026-05-15.md)
- [SESSION_2026-05-11_WW_RESULTS](findings/SESSION_2026-05-11_WW_RESULTS.md) — CC#10 580 N success auf alter STL
- [PHASE_5B_DR_PRODUCTION_2026-05-15](findings/PHASE_5B_DR_PRODUCTION_2026-05-15.md) — Mode 3 + TYPE_S baseline
- Commit 741a974 (phase0-ahmed-validation) — finding 36 Three-Attractor synthesis
