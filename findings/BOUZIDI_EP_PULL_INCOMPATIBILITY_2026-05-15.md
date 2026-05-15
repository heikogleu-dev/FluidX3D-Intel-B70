# Sparse Bouzidi BB — EP-pull Architektur-Inkompatibilität

**Datum:** 2026-05-15
**Status:** ❌ Bouzidi post-stream im FluidX3D EP-pull-Layout funktioniert NICHT für komplexe STL-Geometrie. Architektonisches Limit gefunden, dokumentiert.
**Verdict:** Pivot zu anderem Pfad nötig.

## Was wir gelernt haben (chronologisch)

### Phase 1: Sparse-Infrastructure Test (PASSED)
- Active-cell-list mit 211k Far + 2.24M Near wall-adjacent fluid cells (=0.13% / 0.93%)
- VRAM-Overhead nur ~195 MB (sparse Q-storage)
- Bei q=0.5 (= Pure-BB equivalent): forces **bit-identisch zu Mode 3 baseline** ✓
- **Schlüssel-Insight:** load_f + store_f ist NICHT no-op ohne Modifikation — EP-pull "implicit stream" verschiebt Daten zwischen Slots
- **Fix:** kernel skippt store_f wenn `modified == false` → korrekte no-op-Behavior

### Phase 2a: q=0.4 Test (FAILED)

**Test 1 — Original Step 2A Formula** (mit fhn_nb):
```
fhn[opp_i] = 2q*fhn[i] + (1-2q)*fhn_nb[i]    [q<0.5]
```

Chunk 3 Forces: Fx_far=4,484 / Fz_near=+864 (LIFT statt downforce baseline -909)
Chunk 5 Forces: Fx_far=-959 (NEGATIVE drag, drift)

**Test 2 — "Korrigierte" Formula** (EP-pull derivation, ohne fhn_nb):
```
fhn[opp_i] = 2q*fhn[opp_i] + (1-2q)*fhn[i]    [q<0.5]
```

Selbe Pathology — Forces driften zu negative drag.

**Test 3 — Option A: Step 1 EXACT formula + first-wall-only break**:
```
für ALLE direction-pairs: if wall → bouzidi → break; [nur erste Wand]
```

Chunk 10 Forces: **Fx_far ≈ 2.9e-4** (10^-7 von baseline). **Strömung kollabiert komplett zu zero/laminar state.**

## Architektur-Root-Cause: EP-pull Neutralization

Das ist in FluidX3D's eigenem Code dokumentiert! kernel.cpp:1717-1719 (CC#7 deprecated comment):

> "DEPRECATED: CC#7-V1/V2 TYPE_Y inline specular reflection (swap AND one-way assignment were both tried, both produced bit-identical Fx≈14400 N — **Esoteric-Pull layout effectively neutralises inline DDF modifications at TYPE_Y cells**)"

CC#7 ist exakt das gleiche Phenomenon: inline-DDF-modifications in EP-pull post-stream werden architektonisch NEUTRALISIERT oder fehlerhaft propagiert. Damit damalige Autoren-Lösung war CC#8 Ghost-Cell-Mirror (komplett anderer Mechanismus).

## Warum apply_freeslip_y funktioniert aber Bouzidi nicht

apply_freeslip_y benutzt **mirror-symmetric** modifications:
```c
fhn[3]  = fhn[4];   // Y-direction mirror
fhn[7]  = fhn[13];  // diagonal Y-mirror
// etc — alles mirror-pairs
```

Mirror operations preservieren Y-momentum balance. Store_f propagiert konsistent.

Bouzidi ist **non-mirror, weighted interpolation** mit verschiedenen Cell-Sources (Cell n own DDFs + nachbar fhn_nb). Diese mixing injiziert Information die nicht zur EP-pull's "implicit stream" Erwartung passt → corruption / collapse.

## Warum Step 1 PASSED in Poiseuille aber für MR2 nicht

Step 1 (`apply_bouzidi_z_walls`) PASSED auf Poiseuille channel flow:
- Nur z-walls (axis-aligned)
- Strömung quer zu Walls (entlang x)
- Cells haben EINE wall direction (entweder +z oder -z, nicht beides)
- Geometrie ist trivial — keine Convex/Concave corners

Mein MR2 Setup:
- Komplexe STL Vehicle: Splitter-Lippe, Wing-Profile, Diffusor-Strakes
- Cells haben oft 2+ wall directions (corners, edges)
- Multi-direction modifications führen zu inkonsistenten store_f operations
- Even single-direction (Option A) kollabiert weil EP-pull-Layout context komplexer ist als Poiseuille

## Was funktioniert NICHT (alle getestet)

| Approach | Result |
|---|---|
| Original formula, multi-direction, sparse | Negative drag drift |
| Corrected formula (no fhn_nb), sparse | Same negative drag |
| Step 1 EXACT formula, single wall direction, sparse | Flow collapses to zero |
| q=0.5 (Pure-BB equiv) | Works (= baseline) but useless |

## Was funktionieren KÖNNTE (untested, höhere Komplexität)

| Approach | Estimated Aufwand | Erfolg-Wahrscheinlichkeit |
|---|---|---|
| Dense iteration (every cell load_f+store_f) — Step 1's pattern, all 18 directions | 1-2h test | 30% (still EP-pull issues likely) |
| Ghost-Cell-Mirror pattern like CC#8 für vehicle | 2-3 Tage | 40-50% (workaround, fragile) |
| **Modify stream_collide directly to apply Bouzidi inline** | 2-3 Tage | 20% (CC#7 deprecated comment warns this gets NEUTRALIZED) |
| Pi-Tensor f_neq Reconstruction (Han 2021) — modifies f_neq at fluid cells, not f_i directly | 1-2 Wochen | 60-70% (different mechanism, sidesteps EP-pull issue) |
| Use different LBM variant (e.g., MRT or non-EP storage) — major fork | 2+ Wochen | high but huge investment |

## Verdict + Recommendation

**Sparse Bouzidi BB im aktuellen FluidX3D-Branch ist nicht praktikabel** mit den aktuellen architektonischen Constraints. Die EP-pull-Layout-Inkompatibilität mit custom DDF modifications für komplexe Geometrien ist ein bekanntes Limit (CC#7 deprecated comment).

**Empfehlungs-Pfad:** Pi-Tensor f_neq Reconstruction (Path IV Han 2021) ist mathematisch der saubere Wall-Function-Approach. Modifies f_neq an FLUID cells (nicht boundary DDFs) → umgeht das EP-pull-Inline-Modification-Problem.

ABER: 1-2 Wochen Aufwand. User-Direktive nötig.

**Alternative-Pragmatischer-Pfad:** Akzeptiere Mode 3 + α=0.15 + TYPE_S Pure-BB als Production-Baseline (Fx ~1480 N, Fz_near ~-552 N, stable, turbulent realistic). Pure-BB-Drag-Overpredict (~2.5x target) is acceptable for current development stage — exakter targets brauchen weiteres post-processing oder Wall-Model.

## Phase 1 Code-State

- `BOUZIDI_VEHICLE` define + kernel `apply_bouzidi_sparse` BLEIBT im Source
- `compute_bouzidi_cells_active(q_default)` BLEIBT
- Beim Test mit q=0.5 = no-op (validated baseline-match)
- Für nächsten Production-Run: einfach `compute_bouzidi_cells_active`-Calls in setup.cpp **kommentieren** (oder `q_default = 0.5f` lassen → no-op)

Code-Infrastruktur ist **wiederverwendbar** falls Pi-Tensor oder anderer Approach später eine sparse-cell-list braucht.

## See Also

- [WALL_MODEL_DEEP_ANALYSIS_2026-05-15](findings/WALL_MODEL_DEEP_ANALYSIS_2026-05-15.md)
- [PATH_BOUZIDI_SPARSE_DESIGN_2026-05-15](findings/PATH_BOUZIDI_SPARSE_DESIGN_2026-05-15.md)
- [FLOOR_WW_RESULT_2026-05-15](findings/FLOOR_WW_RESULT_2026-05-15.md)
- [PATH_IV_PI_TENSOR_DESIGN_2026-05-15](findings/PATH_IV_PI_TENSOR_DESIGN_2026-05-15.md)
- src/kernel.cpp:1717-1719 — CC#7 EP-pull-Neutralization documentation
- Commit `12291d3` (phase0-ahmed-validation) — Bouzidi Step 1 Poiseuille PASSED (simple axis-aligned only)
