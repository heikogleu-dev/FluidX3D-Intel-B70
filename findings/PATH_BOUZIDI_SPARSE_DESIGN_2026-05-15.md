# Sparse Bouzidi BB — Design & Phased Implementation

**Datum:** 2026-05-15
**Trigger:** User-Befund Mode 3 + α=0.15 + Floor-WW Production zeigt **Totwassergebiete am Vehicle-Underbody durch Pure-BB Voxelisierungs-Staircase**. WW-Krueger architektonisch unzureichend ([FLOOR_WW_RESULT_2026-05-15](findings/FLOOR_WW_RESULT_2026-05-15.md)). Bouzidi adressiert Geometrie-Fehler statt y+ Fehler.

## Hypothese

**Sub-Grid Bouzidi BB an Vehicle-Surface-Cells eliminiert Staircase-induced Strömungsablösungen** an scharfen Geometrien (Splitter-Lippe, Diffusor-Strakes, Wing-Trailing-Edge), reduziert dadurch Totwasser-Bereiche an Underbody.

**Mechanismus:** Bouzidi-Formel transferiert DDFs gemäß **sub-cell q-Fraction** (Distanz Cell-Center zu Wand / Cell-Größe) statt 0.5 (Pure-BB-Annahme).

```
Pure-BB:     Wand IMMER bei q=0.5 (Mitte zwischen Solid und Fluid Cell)
Bouzidi:     Wand bei q ∈ (0, 1) — wo Triangle wirklich ist
```

→ Voxelisierungs-Staircase auf 1-cell-Resolution reduziert.

## VRAM-Constraint → Sparse Architektur erforderlich

Dense q-field (Step 2A WIP auf `phase0-ahmed-validation`): N × 19 × 4 bytes = **18 GB** für Near allein → overflow.

**Sparse Architektur:** q-field **nur für wall-adjacent fluid cells** (Cells mit mindestens einem TYPE_S|TYPE_X Nachbarn). Bei MR2-Vehicle: ~0.1-0.5% aller cells → 100-1000× kleiner.

## Phased Implementation (per Methodologie)

### Phase 1 — Sparse-Infrastructure Sanity-Check (~3-4 Std)

**Hypothese:** Sparse-Datenstruktur + kernel runtime funktioniert. Bei q=0.5 sollten Forces **identisch zu Pure-BB Baseline** sein (Bouzidi at q=0.5 = Standard-BB).

**Code:**
1. `LBM_Domain` neue Members: `Memory<uxx> bouzidi_active_cells`, `Memory<float> bouzidi_q_data`, `uint bouzidi_N_active`, `Kernel kernel_apply_bouzidi_sparse`
2. `LBM` neue Methods + Member: `bool bouzidi_enabled = false`, `compute_bouzidi_cells_active(...)` (host-side enumeration of wall-adjacent fluid cells)
3. `kernel.cpp` neuer kernel `apply_bouzidi_sparse`: iteriert über `bouzidi_active_cells[]`, anwendet Bouzidi-Formel
4. `do_time_step`: conditional call wenn `bouzidi_enabled`
5. `setup.cpp`: aktiviere für Mode 3 nach voxelize

**Success-Kriterium Phase 1:**
- ✅ Build OK
- ✅ Run startet ohne Crash
- ✅ Tier-1 Smoke (5 chunks): Forces innerhalb ±5% von Mode 3 Baseline (= Pure-BB equivalent at q=0.5)
- ✅ VRAM-Overhead < 100 MB

**Failure-Handling:**
- Crash: kernel-debug, gucke welche cells active sind, prüfe indices
- Forces deutlich anders: q=0.5 doch nicht Pure-BB-equivalent → check Bouzidi-Formel-Implementation
- Memory-Allocation-Fail: reduziere conservative N_active estimate

### Phase 2 — Real q Computation per Direction (~6-8 Std)

**Hypothese:** Ray-Direction-March vom Cell-Center entlang c_i bis erste TYPE_S|TYPE_X-Cell gibt eine reasonable Approximation des q-Werts. Bei axis-aligned walls exakt. Bei diagonalen Geometrien (Splitter-Lippe, Diffusor-Strakes) approx.

**Approach: Voxel-March q Estimation**

Statt komplex ray-triangle intersection auf STL: nutze die schon-voxelisierten flags. Für jeden Active-Cell + Direction:

```
cell_n is fluid, j[i] is solid (wall) — get q for direction i
walk from cell_n along c_i: 
  - distance to cell_j[i] center = 1.0 (in lu)
  - q = distance to actual wall ≈ 0.5 (if wall is exactly midway, standard BB)
  - BUT for staircased geometries, wall might be elsewhere
```

Simplification: für jeden active cell, **iteriere nicht direction-by-direction sondern teste fractional distance**:
- step from cell center in fractional sub-cells (0.1, 0.2, ..., 0.9 of c_i)
- bei jedem step, lookup ob position innerhalb solid voxel
- q ≈ erster step wo solid

OR even simpler: use cell-flag-pattern. Eine cell mit 2+ solid-neighbors in opposite directions hat thin wall → q=0.5 unverändert. Single-direction wall → q wahrscheinlich nahe 0.5.

**Pragmatisches Phase 2a:**  
Erstmal q=0.5 lassen (Pure-BB equivalent). Test ob Sparse-Infrastructure stabil bleibt. Wenn ja, **Phase 2b** echte q-Berechnung.

**Phase 2b (real q):**  
Implementation TBD. Optionen:
- Sub-Voxel-Resampling via secondary voxelization at 4× resolution
- Ray-Triangle intersection on STL mesh (analog Step 2A sphere)
- Vector signed-distance-field interpolation

### Phase 3 — Validation Tier-2 (~2-4 Std)

**Hypothese:** Bouzidi mit echten q-Werten reduziert Underbody-Totwasser sichtbar.

**Success-Kriterium:**
- ParaView Underbody-Slice: weniger ausgedehnte dunkelblaue (low velocity) Bereiche unter Vehicle
- Fx_far/Fx_near: ≤ Mode 3 Baseline (= weniger künstliche Drag durch Staircase-Separation)
- Fz_near: gleich oder besser (Downforce nicht degradiert)

**Failure-Handling:**
- Wenn keine Verbesserung: q-Berechnung möglicherweise fehlerhaft → diagnose q distribution per cell
- Wenn Pathology: kernel-bug → rollback

## Stop-Conditions

Konsultiere User:
- Wenn Phase 2b Ray-Triangle-Intersection benötigt → komplex, evtl. anderer Approach (sparse via FluidX3D's existing voxelize_mesh kernel adapter)
- Wenn N_active deutlich >1M (cells) → VRAM Reserve neu prüfen

## Code-Patterns aus phase0-ahmed-validation Branch

Kernel `apply_bouzidi_general` (Step 2A) als Referenz:
```c
// Bouzidi-Formel (Bouzidi 2001 linear interpolation):
if(q < 0.5f) {
    fhn[opp_i] = 2.0f*q * fhn[i] + (1.0f - 2.0f*q) * fhn_nb[i];
} else {
    fhn[opp_i] = (0.5f/q) * fhn[i] + (1.0f - 0.5f/q) * fhn_nb[opp_i];
}
```

Sparse-Variant adaptiert: read `q = q_data[direction * N_active + i_active]`.

## Estimated Total Aufwand

- Phase 1: 3-4h
- Phase 2a (q=0.5 prolongation): 0h (already done in Phase 1)
- Phase 2b (real q computation): 6-8h (most uncertain)
- Phase 3: 2-4h
- Doc + Commit: 1h

**Total:** 12-17h ≈ 2 Tage. Matches initial estimate.

## See Also

- [WALL_MODEL_DEEP_ANALYSIS_2026-05-15](findings/WALL_MODEL_DEEP_ANALYSIS_2026-05-15.md)
- [FLOOR_WW_RESULT_2026-05-15](findings/FLOOR_WW_RESULT_2026-05-15.md)
- Commit `12291d3` (phase0-ahmed-validation) — Bouzidi Step 1 Poiseuille PASSED
- Commit `52e6d94` (phase0-ahmed-validation) — Bouzidi Step 2A WIP (sphere, untested)
