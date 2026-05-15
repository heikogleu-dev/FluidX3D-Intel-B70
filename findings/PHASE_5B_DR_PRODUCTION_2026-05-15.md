# Phase 5b-DR Production Day — 2026-05-15

**Branch:** `plan-refresh-multires`
**Status:** 🟡 Mode 2 Production konvergiert aber **unplausible Felder** in ParaView. Mode 1 Test läuft (PID 211771 in dieser Session).

## TL;DR

1. ✅ **Gestern (2026-05-14 abend):** α-Sweep validiert α=0.10 symm Mode 2 stable mit TYPE_E + Vehicle z=0 wheel-contact-fix (siehe [[PHASE_5B_DR_ALPHA_SWEEP_2026-05-15]])
2. ✅ **Heute Mode 2 Production run:** Far 13.5m anlauf 1.5m + 8m × 5m, Vehicle z=1 (15mm), α=0.10 symm, auto-stop 2% → konvergiert chunk 69 in 20 min wallclock
3. 🔴 **ParaView-Befund:** Mesh/Field Position-Mismatch (gefixt via Transform); Diffusor-Einlass unplausibel; Near-Strömung ÜBER Fahrzeug zu langsam; Fz_near +1270 N (LIFT statt erwartet −1200 N DOWNFORCE)
4. ⚙️ **Hypothesen:** (a) sequenzielles Mode 2 Back-Coupling-Lag erzeugt Phasenversatz, (b) TYPE_E Floor blockiert Venturi-Physik unter Splitter, (c) Coupling-Plane-Indizierung evtl. falsch nach Far-Geometrie-Änderung
5. ▶️ **Aktuell läuft:** Mode 1 (kein Back-Coupling) + α=0.5 forward soft-BC + TYPE_E unchanged → isoliert Back-Coupling als Variable
6. 📝 **Performance-Designs dokumentiert (NICHT implementiert):** PERF-F V5 (Multi-Stream PCIe), PERF-G (Concurrent LBM = Additive Schwarz = symmetric lag)

## Geometrie-Änderungen heute (Production-Config)

| Parameter | Gestern Abend (HEAD) | Heute Production |
|---|---|---|
| Far Box | 1067 × 534 × 334 (16m × 8m × 5m, anlauf ~4m) | **900 × 534 × 334 (13.5m × 8m × 5m, anlauf 1.5m)** |
| Far cells | 190.3 M | 160.5 M |
| Near Box | 1320 × 540 × 339 (6.6m × 2.7m × 1.695m) | gleich |
| Near cells | 241.6 M | 241.6 M |
| Near origin in Far cells | (233, 177, 0) | **(67, 177, 0)** |
| Vehicle Far z (LBM cells) | 0 (wheels on ground) | **1 (15mm clearance)** |
| Vehicle Near z | 1 | **3 (matches Far 15mm)** |
| Floor BC | TYPE_E | TYPE_E (gleich) |
| Coupling | Mode 2 symm α=0.10 | gleich |
| Convergence-Stop | manuell | **2% Δ Fx_far über 5000 Far-steps** |

**VRAM:** ~26 GB (Far 10.3 + Near 15.5 GB) — Near dominiert 60%, Anlauf-Reduktion shrinks nur Far (~−2 GB).

## Production Run Result (Mode 2 symm α=0.10, chunk 69 konvergiert)

```
Fx_far  : ~790 N    (Time-Attack target 600 N → +32%)
Fx_near : ~474 N    (Half of Far via Multi-Res back-coupling)
Fz_far  : +90 N
Fz_near : +1270 N   (LIFT — falsch; erwartet −1200 N DOWNFORCE)
```

Konvergenz-Stop trigger bei chunk 69: dFx_far/Fx_far = 0.212% < 2%.

## ParaView-Rückschläge & Diagnose

### Rückschlag 1: Mesh/Field Position-Mismatch

**Beobachtung:** Vehicle STL erschien weit von den Far+Near-Bounding-Boxes entfernt.

**Root Cause:** `LBM::write_mesh_to_vtk` ([src/lbm.cpp:1112](src/lbm.cpp#L1112)) verwendet `offset = LBM::center()` → Mesh in LBM-box-zentriertem Koord-System. Aber `fix_vtk_origin.py` setzt Field-ORIGIN auf Welt-Position. → Mismatch.

**Fix:** ParaView Transform-Translate auf Mesh:
- Far: `(+5.25, +0.005, +2.505)` (Far box center in Welt-Koords)
- Near: `(+2.805, +0.010, +0.8475)`

**Memory festgehalten:** [[paraview-vtk-alignment]] (2026-05-15)

### Rückschlag 2: GPU/Ubuntu-Crash + ParaView-Reboot

ParaView mit allen 5 VTKs auf B70's 32 GB hat GPU-Pfad geseucht und Ubuntu-Crash ausgelöst. Workaround: VTKs sequenziell laden statt simultan visible.

### Rückschlag 3: make.sh Auto-Run Doppelläufe

**Beobachtung:** `make.sh` Zeile 33 ruft automatisch `bin/FluidX3D` nach erfolgreicher Compilation auf. Bei Background-Build via Claude wurde dadurch ein zweiter Run gestartet ohne dass ich's gemerkt habe → **55 GB VRAM Overflow auf 32 GB B70**.

**Fix:** [make.sh:33](make.sh#L33) Auto-Run **deaktiviert** per User-Direktive 2026-05-15.

**Memory festgehalten:** [[feedback-check-running-processes]] — `pgrep -af FluidX3D` zwingend vor jedem Start.

### Rückschlag 4 (offen): Unplausible Strömung in ParaView

Nach Mesh-Alignment-Fix sichtbar:
- **Near:** Diffusor-Einlass-Strömung unplausibel; Bodenströmung im Near schwächer als in Far; Strömung ÜBER Fahrzeug zu langsam
- **Far:** Bodenströmung plausibler aber Gesamtströmung um Fahrzeug unplausibel
- **Moving Ground** in Far OK aussehend, in Near nicht — obwohl Floor-BC-Code identisch ist ([setup.cpp:944,956](src/setup.cpp#L944))

Vermutete Ursachen (nicht alle geprüft):
- **H1: Sequenzielles Mode 2 Back-Coupling erzeugt Phasenversatz** (8.4 cm physikalisch bei 45 m/s × 1.87 ms Chunk-Dauer)
- **H2: TYPE_E Floor blockiert Venturi-Physik** unter Splitter (kein Boden-Beschleunigung möglich)
- **H3: Coupling-Plane-Indizierung evtl. falsch** nach Geometrie-Änderung (Near origin Far cell 233 → 67)
- **H4: Vehicle-Voxelisierung in Near** könnte abweichen von Far (3:1 ratio, anti-aliasing)

## Aktuelle Untersuchung: Mode 1 + α=0.5

User-Vorschlag 2026-05-15 nachmittag: **Mode 1 (kein Back-Coupling) + α=0.5 forward soft-BC + TYPE_E unverändert** testen.

Isoliert H1 (Back-Coupling-Lag) — wenn Mode 1 plausible Felder zeigt → Mode 2 Back-Coupling ist das Problem → PERF-G Concurrent LBM löst es (siehe unten). Wenn Mode 1 weiterhin unplausibel → H2/H3/H4 als Root-Cause weiter untersuchen.

Code-Change ([setup.cpp:236](src/setup.cpp#L236) + [setup.cpp:1002](src/setup.cpp#L1002)): `PHASE_5B_COUPLE_MODE=1`, `opts.alpha = 0.5f`.

## Performance-Designs (NICHT implementiert, dokumentiert)

### PERF-F V5: Multi-Stream PCIe Pipelining
- Doc: [[PERF_F_V5_DESIGN_2026-05-15]]
- Aufwand: ~1 Tag
- Gain: 2-3% MLUPs (auf PERF-D batched aufbauend)
- Status: Phase A (2nd CommandQueue in opencl.hpp) ✅ done; Phase B/C pending

### PERF-G: Concurrent LBM (Additive Schwarz)
- Doc: [[PERF_G_CONCURRENT_LBM_2026-05-15]]
- Aufwand: ~1 Tag
- Gain: **20-30% MLUPs + physikalisch konsistenteres Coupling** (symmetric 1-chunk lag statt asymmetric)
- User-Insight 2026-05-15: serielles Mode 2 hat asymmetrischen Phasenversatz → "verwischt das Bild"
- Status: nur Design

## Lessons Learned

1. **make.sh läuft FluidX3D auto** nach successful build — Code-Mechanik zwingend lesen vor Background-Builds
2. **Coordinate-System-Inconsistenzen** zwischen VTK-Output-Pfaden (POLYDATA mesh vs STRUCTURED_POINTS field) sind nicht offensichtlich und müssen via Transform gefixt werden
3. **Doppelte FluidX3D-Prozesse auf 32 GB B70 = VRAM-Overflow** → User-Visibilität (intel_gpu_top oder VRAM dashboard) hat das gerettet, nicht meine pgrep
4. **Multiplikatives vs Additives Schwarz** matters: sequenzielle Coupling kann subtile Phasenversätze einbauen die transiente Aerodynamik verfälschen. Klassisches Schwarz-Theorie war hier blind-spot
5. **VRAM-Analyse vor Geometrie-Änderung:** Anlauf-Reduktion bringt fast nichts wenn Near-Domain 60% des VRAM hält

## Nächste Schritte (Reihenfolge)

1. ⏳ Mode 1 Run abwarten (in progress)
2. ⏳ ParaView-Inspektion durch User → Entscheidung ob Coupling-Logik gefixt ist
3. **Wenn Mode 1 plausibel:** PERF-G implementieren (zwei Probleme einmal gelöst)
4. **Wenn Mode 1 unplausibel:** H2/H3/H4 untersuchen
   - H2: TYPE_S Floor mit korrekter Rolling-Road-Direction zurück
   - H3: Coupling-Plane-Indizes verifizieren (assert plane positions match geometry)
   - H4: Vehicle-Voxelisierung in Far vs Near vergleichen (cell-count, BBox sanity)

## See Also

- [[PHASE_5B_DR_ALPHA_SWEEP_2026-05-15]] — gestrige α-Sweep Validation
- [[PHASE_5B_DR_RESULT_2026-05-14]] — initial DR + 95% GPU utilization
- [[paraview-vtk-alignment]] — Mesh/Field Position-Fix
- [[feedback-check-running-processes]] — Doppelläufe vermeiden
- [[PERF_F_V5_DESIGN_2026-05-15]] — Multi-Stream PCIe
- [[PERF_G_CONCURRENT_LBM_2026-05-15]] — Concurrent LBM + Additive Schwarz
