# Phase 3 Multi-Cell WALL_VISC_BOOST — Final Result (2026-05-16)

**Run:** Mode 3 PERF-G Concurrent Additive Schwarz + WALL_VISC_BOOST Phase 3 (3-layer BFS)
**Branch:** `master` (commit `554b3e0`)
**Wallclock:** ~38 min (chunk 1 → auto-stop chunk 150)
**Auto-Stop:** triggered chunk 150 = 15 000 Far-steps, `|dFx_far|/|Fx_far| < 2%` over recent 5000 Far-steps

## Final Forces (mean over last 25 chunks, 126–150)

| | Phase 2 EOD (1 Layer) | **Phase 3 (3 Layer)** | Δ | OpenFOAM Target |
|---|---:|---:|---:|---:|
| **Fx_far** (drag) | 1 464 N | **1 484 N** | **+1.4 %** | 400–600 N |
| **Fx_near** (drag) | 1 597 N | **1 574 N** | **−1.4 %** | 400–600 N |
| Fy_far | ~0 | 0.1 N | ~0 ✓ | ~0 |
| Fy_near | ~0 | 10.2 N | ~0 ✓ | ~0 |
| **Fz_near** (downforce) | −563 N | **−552 N** | **−1.9 %** | n/a |

**Konvergenz-Trajektorie** (chunk-Range mittlere Fx_far):
- 1–10: transient peak 21 487 → 6 500 N
- 30–40: 1 950 N
- 60–80: 1 800 N
- 100–125: 1 600 N
- 126–150 (final): **1 484 N** ✓

## Wichtigste Erkenntnis

**Phase 3 Multi-Cell hat forcemäßig praktisch KEINEN Effekt vs Phase 2.**

Trotz:
- 3× tieferer BL-Coverage (45 mm Far / 15 mm Near vs Phase 2's 15 mm / 5 mm)
- 5× stärkerem ν_t-Boost in Layer 3 vs Layer 1 (1.025·u_τ vs 0.205·u_τ)
- ~3× mehr betroffener Zellen (Far 2.04 M / Near 8.52 M vs Phase 2 692 k / 2.95 M)

→ Integrierte Drag/Downforce-Werte sind innerhalb statistischer Streuung **identisch** (±2 %).

## Interpretation

Wall-Shear-Stress wird dominant durch **Layer 1** (direkt wand-adjazente Zellen) bestimmt. Die zusätzlichen Layer 2 und 3 fügen Eddy-Viskosität in Zonen ein, wo:
- Smagorinsky-SGS bereits ähnliche turbulente Viskosität liefert
- y+ schon im defect/wake region ist (Layer 2: y+ ≈ 750, Layer 3: y+ ≈ 1250 für Near)
- Die Strömung ohnehin turbulent ist und ν_t_total = ν + ν_t,Smag + ν_t,Prandtl nur marginal von Phase 3 erhöht wird

**Phase 3 ist marginal redundant** zur bereits-aktiven Smagorinsky-LES. Visueller BL-Effekt in ParaView mag sichtbarer sein (3× mehr Zellen affected), aber Drag/Lift-Quantitäten ändern sich nicht.

## Bewertung der vor-Run-Hypothesen

| Vorhersage (vorher) | Realität (Run-Daten) |
|---|---|
| "Drag tendiert höher zu konvergieren" (16:09 Update) | ❌ falsch — End-Drag bei 1484 N, fast identisch Phase 2 (1464 N) |
| "Phase 3 zeigt deutlich stärkeren Wand-Effekt" | ❌ falsch — visuelles Effekt vorhanden, Force-Effekt vernachlässigbar |
| "u_τ-Overshoot an Layer 2/3" (Physics Review) | ✅ existiert, aber kompensiert durch Smagorinsky-Substraktion oder ist sub-dominant |
| "Outer-BL Overshoot ohne Cap" | ✅ existiert, aber kompensiert in der Integration |

Die zwei systematischen Physik-Probleme aus `WALL_VISC_BOOST_PHASE3_PHYSICS_REVIEW_2026-05-16.md` sind **mathematisch vorhanden, aber praktisch nicht detektierbar** in den Force-Integralen für diese Geometrie und Re. Vermutlich heben sich die overshoots in Layer 2/3 mit der Smagorinsky-SGS-Vorbelegung gegenseitig auf.

## Mode 3 Coupling — funktioniert wie designt

- Fx_far ≈ Fx_near (1484 vs 1574, Δ 6 %) — physikalisch konsistente Domains
- Fy_far ≈ Fy_near ≈ 0 — Strömung sauber symmetrisch
- Coupling-Lag von 1 Chunk pro Direction wirkt stabilisierend (keine Mode-2-Oszillationen)

## Vergleich zum OpenFOAM-Target

Beide Phasen liegen weiterhin bei **~3× OpenFOAM-Referenz** (400–600 N). Die Drag-Reduktion (Phase 2 oder Phase 3 vs nackter Pure-BB ≈ 2 200 N) ist nur ~30 % — wir bräuchten ~75 % für OpenFOAM-Range. Die zusätzliche BL-Coverage in Phase 3 reicht dafür nicht aus.

## Konsequenz für nächste Schritte

Da Phase 3 keine Quantitäts-Verbesserung gegen Phase 2 bringt, ist **vertikale Vertiefung mehr Cells** kein vielversprechender Pfad. Die echte Hebelarme sind:

### Phase 3.1 — sofort (~30 min)
- u_τ einmalig aus Layer-1-Geschwindigkeit (statt aktuelle Layer-Local-u-Fehlkonstruktion)
- Van-Driest-Damping `[1−exp(−y+/26)]²` anhängen
- **Erwartung**: marginale Verbesserung (alle Layer 2+3 Effekte werden ohnehin nicht gesehen)
- **Wert**: physikalisch korrekter Code statt mathematisch dubioser Stand

### Phase 4 — Geometrie-aware Boost (~4–6 h)
- Wall-Normalen aus TYPE_S-Pattern in `populate_wall_adj_flag`
- Region-Klassifikation: Front-Stagnation / Mittendrittel / Lee
- Region-spezifische Boost-Profile (Layer-Anzahl, Outer-Cap)
- **Erwartung**: signifikanter Effekt — Front bekommt dünne BL korrekt, Lee bekommt separierten BL korrekt
- Cebeci-Smith / Baldwin-Lomax-artiges 2-Layer-Modell

### Sigma-SGS-Modell — Alternative zu allem (~3–5 h)
- Toda & Nicoud 2009: ν_t läuft algorithmisch O(y³) zur Wand, **keine** Wand-Distanz-Berechnung, **keine** Damping-Funktion
- Eleganter aber: Smagorinsky muss ersetzt werden
- Phase 3 + 4 wären dann obsolet

### Alternative Hebelarme (orthogonal zur Wand-Modellierung)
- **Bouzidi BB** (parked): adressiert Voxelisierungs-Staircase am Unterboden — komplett anderer Effekt als BL-Modellierung
- **Höhere Far-Resolution**: 12 mm statt 15 mm Far + 4 mm Near. VRAM-Budget grenzwertig.

## Files

- Far VTKs: `export/dr_far/{u,rho,flags,F,mesh}-000015000.vtk` (5 files, 0.16M cells)
- Near VTKs: `export/dr_near/{u,rho,flags,F}-000045000.vtk` (4 files, 0.24M cells, mesh-vtk fehlt — kann Far's mesh wiederverwendet werden)
- Forces CSV: `bin/forces_phase5b_dr_mode3.csv` (150 chunks)
- Log: `/tmp/fluidx3d_phase3.log`

## See Also

- [WALL_VISC_BOOST_PHASE3_PHYSICS_REVIEW_2026-05-16.md](WALL_VISC_BOOST_PHASE3_PHYSICS_REVIEW_2026-05-16.md) — physik-theoretische Review + Phase 4 design sketch
- [SESSION_2026-05-15_SUMMARY.md](SESSION_2026-05-15_SUMMARY.md) — Phase 2 EOD-Stand
- [TODO_2026-05-16.md](TODO_2026-05-16.md) — ursprüngliche Phase 3 Test-Planung
