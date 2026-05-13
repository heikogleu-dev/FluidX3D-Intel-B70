# Phase 5b: Dual-Domain Same-Resolution Schwarz Coupling — Design

**Datum:** 2026-05-13
**Branch:** `plan-refresh-multires`
**Status:** Implementiert, Run läuft (15000 steps).

## Ziel

Validiere zwei nebeneinander laufende LBM-Instanzen auf einer GPU mit
Schwarz-Coupling über 5 gemeinsame Flächen. Selbe Auflösung (10 mm) in
beiden Domains — kein bilineares Upsampling nötig. Wenn das funktioniert,
sind die Mechanik-Bausteine für Phase 5b-DR (Double-Res) bewiesen.

## Designentscheidungen

### Far-Field geschrumpft (per User 2026-05-13)

User-Vorgabe: "den gesamten windtunnel kannst du auch sonst gern,
gerade vorn, noch einkürzen. achte aber auf blockade, das die nicht
über 10% geht".

| Achse | Alt (CC#6-Full) | Neu (Phase 5b) | Änderung |
|---|---|---|---|
| X    | 1500 cells (15m) | **1000 cells (10m)** | -33% |
| Y    | 500 cells (5m)   | 500 cells (5m) | unverändert (Blockage) |
| Z    | 450 cells (4.5m) | 450 cells (4.5m) | unverändert (Blockage) |
| Cells | 337.5 M | **225 M** | -33% |
| Vehicle X-center | cell 400 | **cell 350** | shift nach vorn |
| Anlauf vor Fahrzeug | 1.75 m | **1.25 m** | -29% (gewünscht) |
| Wake hinter Fahrzeug | 8.75 m | 4.25 m | -51% |
| Blockage | 8.9% | **8.9%** | unverändert (< 10% ✓) |

Y/Z blieben gleich, weil bei MR2 Frontfläche ≈ 2.0 m² und Tunnel-Querschnitt
5×4.5 = 22.5 m² die Blockage bei 8.9% liegt — Verkleinerung würde >10% ergeben.

### Near-Field (per User 0.5m vor/seitlich/oben, 2m hinten)

| Achse | Margin zum Fahrzeug | Extent | Zellen |
|---|---|---|---|
| X_min | 0.5 m vor       | 7.0 m | 700 |
| X_max | 2.0 m hinter    | 7.0 m | 700 |
| Y_min/max | 0.5 m je Seite | 3.0 m | 300 |
| Z_max | 0.5 m oberhalb  | 1.85 m | 185 |
| Z_min | 0 m (Boden geteilt) | — | — |

**Near total: 700 × 300 × 185 = 38.85 M Cells**

Origin in Far-Koord: (75, 100, 0) | Vehicle in Near-Koord: X-center 275,
Y-center 150, Z-min 1.

Buffer zu Far-Wänden:
- Far inlet → Near X_min: 75 cells = 0.75 m
- Far outlet → Near X_max: 225 cells = 2.25 m
- Far Y_min/Y_max → Near: 100 cells = 1.0 m je Seite
- Far Z_max → Near Z_max: 265 cells = 2.65 m
- Floor: geteilt (Z=0 TYPE_S in beiden)

### Tatsächliche Vehicle-Geometrie (aus Smoke-Run Log)

```
Far Vehicle BBox:  X[125.0, 575.0] Y[156.8, 343.2] Z[1.0, 123.5]
Near Vehicle BBox: X[50.0,  500.0] Y[56.8,  243.2] Z[1.0, 123.5]
```

→ Vehicle reell: L = 4.5 m (450 cells, gestreckt von ~4 m STL),
W = 1.864 m (186 cells), H = 1.225 m (123 cells).

→ Margins in Near (gemessen, in Cells):
- X-min vor: 50 cells = 0.5 m ✓
- X-max hinter: 700 - 500 = 200 cells = 2.0 m ✓
- Y-min/max: 56.8 / 56.8 cells = 0.57 m ✓ (knapp über 0.5 m wegen Vehicle-Breite)
- Z-max: 184 - 123.5 = 60.5 cells = 0.605 m ✓

## Memory-Budget

Smoke-Run gemessen (Intel Arc Pro B70, 32.6 GB VRAM):

| Domain | Cells | GPU [MB] | Byte/Cell |
|---|---:|---:|---:|
| Far  | 225.0 M | 14373 | 63.9 |
| Near |  38.85 M |  2480 | 63.8 |
| **Σ**  | **263.85 M** | **16853** | — |

Free VRAM: 32655 - 16853 = ~15.8 GB. **Comfortable headroom** für Phase
5b-DR-Erweiterung (höhere Auflösung im Near).

## Coupling-Pipeline

Schritt-für-Schritt pro Chunk (= 100 LBM-Steps):
1. Far runs 100 steps
2. Far → Near coupling über 5 Flächen:
   - X_min: Far x=75 → Near x=0   (YZ-plane, Y[100..399] / Z[1..184])
   - X_max: Far x=775 → Near x=699
   - Y_min: Far y=100 → Near y=0   (XZ-plane, X[75..774] / Z[1..184])
   - Y_max: Far y=400 → Near y=299
   - Z_max: Far z=184 → Near z=184 (XY-plane, X[75..774] / Y[100..399])
3. Near runs 100 steps
4. Drag-Messung in beiden (Fx, Fy, Fz via object_force(TYPE_S|TYPE_X))
5. CSV-Append

**Wichtig:** Z=0 (Boden TYPE_S) ist in beiden Domains identisch und wird
NICHT gekoppelt — wir starten die XZ/YZ-Plane-Extent erst bei Z=1, sonst
würde couple_fields() den Floor mit TYPE_E überschreiben.

### Coupling-Overhead (gemessen)

Pro Chunk:
- LBM-Run Far: ~4 s (225 M Cells × 100 steps)
- LBM-Run Near: ~0.7 s (38.85 M × 100 steps)
- 5× couple_fields: ~4-5 s (full-grid read/write per Memory_Container API)
- Drag-Messung: <0.1 s

**Total ~10-15 s pro Chunk → 150 chunks ≈ 25-40 min wallclock.**

Optimierungspotenzial: Partial-buffer-sync statt full-grid (Faktor ~10x
weniger PCIe-Traffic). Deferred bis Phase 5b-DR wenn nötig.

## Pass-Kriterium

Phase 5b PASS, wenn nach Konvergenz (≥ 12000 steps):

|Δ Fx_near vs Fx_far|/|Fx_far| < statistische Streuung (~10%)

D.h. Fx_near ≈ Fx_far innerhalb der 1-σ Schwankungsbreite (~±150 N
basierend auf Phase 5b-pre).

Wenn PASS → Phase 5b-DR (Double-Res mit Far @ 20mm, Near @ 10mm) freigegeben.
Wenn FAIL → Diagnose: ist Coupling fehlerhaft? Fehler im Plane-Indexing?
Far-Wake-Effekt zu klein wegen kurzer Domain?

## Code-Änderungen

`src/setup.cpp`:
- NEW `#define PHASE_5B_DUAL_DOMAIN 1` (default off post-test)
- NEW `void main_setup_phase5b_dual()` (~120 lines)
- Branch in `main_setup()` zu neuer Funktion wenn Toggle 1

Kein Change in `lbm.hpp`/`lbm.cpp` — `couple_fields()` aus Phase 5b-pre
wird unverändert für 5 Planes wiederverwendet.

## Iron-Rule Status

- WW (`WALL_MODEL_VEHICLE`): DEAKTIVIERT (per Pivot)
- Sponge: code-ready, OFF (Phase 5a Iron-Rule, kein full-domain)
- Bouzidi/Stress-Integration: PAUSIERT
- Two-LBM init: ✅ validated (Smoke 500 steps)
- Coupling Mechanik: ✅ validated (5 chunks × 5 planes ohne Crash)

## See Also

- [[PHASE_5B_PRE_SELF_COUPLING_PASSED_2026-05-13]] — Single-LBM Self-Coupling validated
- [[PHASE_5B_PRE_COUPLE_FIELDS_DESIGN_2026-05-13]] — couple_fields() API
- [[PHASE_5A_SPONGE_IRON_RULE_TRIGGER_2026-05-13]] — Sponge nur für Mid-Boxes
- [[PIVOT_BB_SANITY_2026-05-13]] — Pure-BB Baseline
- [[CURRENT_STATE_2026-05-13]] — Branch-Inventar
