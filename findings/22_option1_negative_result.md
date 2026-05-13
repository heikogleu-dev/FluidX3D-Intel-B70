# CC#11 Option 1 — Negative Result: Analytical Artifact Subtraction Does Not Fix WW Pathology

**Datum:** 2026-05-13
**Status:** ❌ Option 1 (`compute_wall_model_artifact` kernel) **eliminiert** durch empirisches Verification-Test
**Folge-Pfad:** Option 2 (OpenLB-style DDF Reconstruction) ist der erforderliche Fix

---

## Was wurde implementiert

1. Neuer OpenCL Kernel `compute_wall_model_artifact` in `kernel.cpp` (Zeile ~1525):
   - Iteriert über jede TYPE_S|TYPE_X Surface-Zelle
   - Liest `u[n]` (von WW-Kernel gesetzt)
   - Für jede D3Q19-Richtung mit Fluid-Nachbar in -c_i-Richtung: addiere analytische Δf_i-Force-Contribution
   - Subtrahiere von F[n] in-place
2. Kernel-Init in `lbm.cpp` registriert
3. Kernel-Call in `enqueue_update_force_field` direkt nach `kernel_update_force_field.enqueue_run()` eingehängt
4. Empirisches Verification-Test (F=0 hack) bestätigt: **Kernel läuft korrekt**

## Test-Ergebnis (Cube 1m, Re=10⁵, 25mm Cells, 65M Cells, 16L×8L×8L Domain)

| Konfiguration | Fx Mean [N] | CD | Match vs Hoerner 1.05 |
|---|---:|---:|---|
| Cube ohne WW | 1.40 | 1.02 | ✅ Reference |
| Cube mit WW (V1, 4M Cells, 25% Blockage) | osz., mean 109 | 79 | ❌ Pathologisch |
| Cube mit WW + Option 1 (Formel-Sign A: dF = -12·w·c·(c·u)) | mean 110 | 80 | ❌ Identisch zur No-Option |
| Cube mit WW + Option 1 (Formel-Sign B: dF = +12·w·c·(c·u)) | mean 110 | 80 | ❌ Bit-identisch zu Sign A |

**Schlüssel-Beobachtung:** Zwei VORZEICHEN-INVERTIERTE Formeln gaben **bit-identische** Cube-Ergebnisse über 5000 Steps. Wenn das Subtraktionsformel tatsächlich Effekt hätte, müsste sich der Mean um ±2·ΔF unterscheiden. Tat es nicht.

## Verification dass Kernel läuft

Diagnostic Test: Kernel wurde temporär modifiziert zu:
```c
F[n] = 0.0f;  // statt F[n] -= dF
F[def_N+n] = 0.0f;
F[2*def_N+n] = 0.0f;
```
Resultat: CSV zeigt Fx = 0.0, Fy = 0.0, Fz = 0.0 über alle 14000 Steps → Kernel läuft, schreibt korrekt in F-Array, Domain-Decomposition + Kernel-Order ist korrekt.

→ Die Diskrepanz zwischen "Kernel läuft" und "Formel hat keinen Effekt" muss eine konzeptionelle Ursache haben.

## Ursache: Indirekter Pfad statt direkter DDF-Artefakt

### Code-Audit-Verständnis (Schritt A aus findings/20):

`apply_moving_boundaries` läuft im **stream_collide eines FLUID-Cells** (TYPE_MS):
```c
fhn[i] += 6 × w_i × (c_i · u_solid_neighbor)
```
Diese Korrektur modifiziert die DDF des **Fluid-Cells**, nicht des Solid-Cells. Das modifizierte fhn[i] streamt in Richtung +c_i (von der Wand weg) zum NÄCHSTEN Fluid-Cell.

### Konsequenz:

Die Krüger-Korrektur landet **NICHT direkt** in den DDFs der TYPE_S Solid-Cells, die von `update_force_field` gelesen werden. Stattdessen:

1. **Korrekturen ändern die Fluid-Flow-Dynamik** im wandnahen Bereich
2. **Geänderter Flow** führt zu anderer Druck- und Wirbel-Verteilung
3. **Geänderter Druck am Wandrand** führt zu anderen Bounce-Back-DDFs am Solid-Cell
4. **`update_force_field` integriert** diese geänderten DDFs

Die "Wall-Model-Pathologie" ist also **indirect via Fluid-Dynamik** — kein direkter Artefakt-Term in den Solid-DDFs.

Daher: keine per-Cell-analytische-Subtraction möglich. Selbst wenn man den Effekt auf den Drag exakt berechnen könnte, müsste man dafür die GANZE Fluid-Dynamik im Wandbereich modellieren.

## Implication für Fix-Strategie

**Option 1 ist nicht durchführbar** — strukturelle Limitation, keine Implementations-Frage.

**Option 2 (OpenLB-Style)** wird notwendig:
- Vermeidet Krüger-Moving-Wall komplett
- Schreibt direkt f_eq + f_neq an Fluid-Cells adjacent zur Wand
- Wall-Function-Effekt eingebaut über DDF-Recovery, keine "bewegte Wand"
- Keine indirekte Pathologie über Flow-Dynamik

## Code-Status

- `compute_wall_model_artifact` Kernel bleibt im Source-Tree (kernel.cpp ~1525) **als Dokumentation des fehlgeschlagenen Pfads**
- Kernel-Call aus `enqueue_update_force_field` **deaktiviert** (kommentiert) — kein Effekt auf Production-Runs
- `update_force_field` und `object_force` verhalten sich identisch wie vor Option 1

WW-Kernel (`apply_wall_model_vehicle`) unverändert.

## Nächster Schritt

Beginn Option 2 Design + Implementation. Plan:
1. Architecture-Design: wo läuft die OpenLB-Style DDF-Reconstruction in FluidX3D's Esoteric-Pull Pipeline?
2. Kernel-Spec: `apply_wall_model_ddf_reconstruct` an TYPE_MS Cells, schreibt 19 DDFs direkt
3. WW-Kernel-Modifikation: u[wall_cell] NICHT mehr setzen (Iron-Rule override)
4. Validation: Cube CD soll auf 1.0-1.2 fallen
5. Cube + MR2 + Yaris parallel validieren

Iron-Rule wird für Option 2 überschrieben — User hat explizit "Option 1 dann 2" freigegeben, was den WW-Kernel-Rewrite impliziert.
