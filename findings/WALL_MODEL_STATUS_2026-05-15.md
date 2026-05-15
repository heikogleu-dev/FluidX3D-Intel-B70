# Wall Model Status & Forward-Path — 2026-05-15

**Branch:** `plan-refresh-multires`
**Trigger:** Mode 3 + TYPE_S Production zeigt Fx_far ≈ 1477 N (2.5× über Time-Attack Target 600 N) — Drag-Overprediction durch fehlendes Wall Model.
**Status:** Research-Doc, kein Code-Change. Nächster Schritt erfordert User-Direktive.

## TL;DR

| Pfad | Geschätzter Effekt auf Fx | VRAM-Fit auf 32 GB B70 | Implementation | Risiko |
|---|---:|:-:|---|:-:|
| **(A) WW reaktivieren auf MR2-STL** | bekannt: ~−610 N (negativ!) auf aktueller STL | ✅ kein extra VRAM | toggle `WALL_MODEL_VEHICLE` | 🔴 over-corrected, unphysisch |
| **(B) WW kalibrieren (Skalierung statt -6)** | Ziel: ~600 N | ✅ | 1-2 Tage Calibration + Sweep | 🟡 architektur-unklar |
| **(C) Bouzidi Step 2A Sphere-Test fortsetzen** | Validation only, kein Drag-Fix | ✅ klein | resumieren von [phase0-ahmed-validation 52e6d94](commit) | 🟢 Step 1 PASSED |
| **(D) Bouzidi auf Multi-Res Vehicle anwenden** | reduziert Voxelisierungs-Fehler (~10-20% Drag-Reduktion) | ⚠️ Near 5mm + q-field = **34 GB → overflow** | mittlere Erweiterung | 🟡 VRAM |
| **(E) Sparse-Bouzidi (q-field nur an Vehicle-Surface)** | Drag-Reduktion ~10-20% bei <1% VRAM-Overhead | ✅ trivial | neue Erweiterung ~1d | 🟢 sauber |
| **(F) Far-only Bouzidi + Near Pure-BB** | Far genauer, Near unverändert; Mode 3 back-coupling mischt | ✅ Far 10.3+12.2=22.5 GB | mittel | 🟡 Asymmetrie |
| **(G) Wall Function direkt im stream_collide** (Spalding/Musker) | echter Wall Model, Ziel Fx ~600 N | ✅ kein extra VRAM | 3-5 Tage | 🟡 nicht-trivial |

**Meine Empfehlung: (E) Sparse-Bouzidi** — kombiniert beste VRAM-Effizienz mit zukunftssicherem Geometrie-Fix. Plus parallel **(B) WW-Calibration** zur Drag-Magnitude-Korrektur.

## Vollständiger Kontext

### Werner-Wengle (CC#10) — Historie

**Funktionierte EINMAL (2026-05-11):** Auf einer **älteren `vehicle.stl`** (CC#10 baseline) lieferte WW Fx = **580 N** — perfekt im OpenFOAM-Range. Dokumentiert in [SESSION_2026-05-11_WW_RESULTS.md](findings/SESSION_2026-05-11_WW_RESULTS.md).

**Versagt auf aktueller MR2-STL:**

| WW-Variante | Aktuelle MR2-STL Fx | Verdict |
|---|---:|---|
| Step-1b Baseline (Pure-BB, no WW transport) | +1820 N | 3.2× over OpenFOAM 565 N (akzeptabel als Baseline) |
| CC#10 Full Krüger −6 (aktuelle STL) | **−610 N** | ❌ negativ, over-corrected |
| Halved Krüger −3 (Phase B) | **+163,000 N** | ❌ Three-Attractor pathology |

CC#11 schloss: "Krüger architektonisch unfit auf MR2-STL" (siehe [finding 36: Three-Attractor synthesis](commit 741a974)).

**Vermutete Ursache (nicht abschließend bewiesen):** Aktuelle MR2-STL hat **scharfe Splitter-Kanten + ground-effect-Geometrie** die der WW-Wand-Tangenten-Projektion nicht passen. Die "−6 multiplier" in Krüger Moving-Wall kalibriert auf einer glatteren Geometrie zu stark.

### Bouzidi BB — Status

**Step 1 PASSED** (axis-aligned z-walls, Poiseuille): L2-Error 0.76-3.51%, sub-percent steady-state. Commit [12291d3](commit).

**Step 2A WIP** ([52e6d94](commit)): GPU-side analytical q-field für Sphere. SoA-indexing bug gefixt nach erstem Run. **Nicht final getestet** — strategischer Pivot zu Multi-Res unterbrochen.

```cpp
// Bouzidi q-field Layout (SoA):
q_field[i*def_N + n] // q-value für Direction i, Cell n
// 19 floats × 4 bytes = 76 bytes/cell extra
```

### VRAM-Constraints für Bouzidi auf Multi-Res

Aktuelles Setup (Far 160M @ 15mm + Near 242M @ 5mm):

```
Base LBM (FP16C):
  Far  = 10.3 GB
  Near = 15.5 GB
  Total Base = 25.8 GB

+ Bouzidi q-field (FP32, 19 floats/cell):
  Far  = 12.2 GB
  Near = 18.4 GB
  Total q-field = 30.6 GB

→ Grand total with Bouzidi everywhere: 56.4 GB → DOES NOT FIT 32 GB B70
```

**Optionen:**
- **Sparse q-field**: q-field nur für Cells mit TYPE_S-Nachbarn (= ~Vehicle-Surface-Cells, ~0.1% von def_N) → trivial VRAM
- **Far-only Bouzidi**: 10.3 + 12.2 = 22.5 GB Far + 15.5 GB Near = 37.8 GB total → overflow
- **FP16 q-field**: 50% Reduktion → Far 6.1 + Near 9.2 = 15.3 GB extra → Total 41 GB → noch overflow
- **Sparse FP16 q-field**: <1 GB total → komfortabel

### Empfohlener Forward-Path

#### Phase 1 — WW-Calibration auf MR2-STL (Skalierungs-Sweep)

Hypothesis: Krüger-Multiplier −6 (CC#10) ist auf MR2-STL zu groß. Zwei Sweeps:

```cpp
// kernel.cpp apply_moving_boundaries (Krüger forcing term)
// Aktuell: Δf_i = -6 × w_i × ρ × (c_i · u_solid)
// Test:    Δf_i = -K × w_i × ρ × (c_i · u_solid) mit K ∈ {-3, -1, +1, +3, +6}
```

Test:
1. Toggle `WALL_MODEL_VEHICLE` an
2. Modifiziere Krüger-Multiplier zu einer Variablen (`#define KRUGER_MULTIPLIER -3.0f`)
3. Sweep K ∈ {−6, −3, −1, +1, +3} → finde K wo Fx ≈ 600 N

Aufwand: ~2 Std Code + 5×~30 min Runs = **~5 Stunden**.
Risiko: Falls keine K-Wahl plausibel funktioniert → WW IST architektonisch unfit → Pfad fail-safe Bouzidi.

#### Phase 2 — Sparse Bouzidi q-field Architektur (parallel)

Idee: q-field NUR für Cells die TYPE_S-Nachbarn haben.

```cpp
// Neue Struktur: array-of-structs oder hash-map
struct BouzidiCell {
    uxx cell_idx;
    float q[19];
};
std::vector<BouzidiCell> sparse_q;
```

Allocation post-voxelize: iteriere alle Fluid-Cells, prüfe TYPE_S-Nachbarn, alloziere q-record nur wenn relevant. Typisch <1% aller Cells → trivial VRAM.

Aufwand: **~2 Tage** (architektonische Erweiterung + kernel-side hash-map lookup).

#### Phase 3 — Bouzidi auf MR2-Vehicle (nach Sparse-Implementation)

1. `init_bouzidi_q_vehicle(vehicle_mesh)` Kernel — analytische ray-triangle intersection für jede Vehicle-Triangle
2. Apply in stream_collide kernel — Bouzidi-formula statt standard BB für Vehicle-Surface-Cells
3. Validation: Vergleich Pure-BB vs Sparse-Bouzidi auf Sphere, dann auf MR2

Aufwand: **~2-3 Tage**.

### Konkurrenz-Vorschlag (Alternative zu allem oben): Wall Function direkt

OpenLB's [Han-2021 WFB](references) (Spalding-Profile via Newton-Raphson auf u+) ist mathematisch sauberer als Krüger und kompatibel mit Esoteric-Pull Layout. Implementation ~3-5 Tage, aber **einmalig sauber**.

Falls Phase 1 WW-Calibration fail → Direkt zu (G) Wall Function.

## Konkreter nächster Schritt (auf User-Direktive)

**Option Q1 (5h, fast feedback):** WW-Calibration Sweep auf MR2-STL  
→ Toggle WW + variable Multiplier + Sweep, finde K wo Fx ≈ 600 N

**Option Q2 (2-3 Tage, sauber):** Sparse Bouzidi implementieren + auf Multi-Res anwenden  
→ Architektur-Erweiterung, dann Vehicle-Surface anwenden

**Option Q3 (3-5 Tage, langfristig sauber):** Wall Function direkt in stream_collide  
→ Han 2021 / OpenLB Pattern

## See Also

- [PHASE_5B_DR_PRODUCTION_2026-05-15](findings/PHASE_5B_DR_PRODUCTION_2026-05-15.md) — Mode 3 + TYPE_S erfolg
- [SESSION_2026-05-11_WW_RESULTS](findings/SESSION_2026-05-11_WW_RESULTS.md) — CC#10 580 N success auf alter STL
- [WALL_MODEL_RESEARCH](findings/WALL_MODEL_RESEARCH.md) — Approach A/B/C Vergleich
- `phase0-ahmed-validation` branch — Bouzidi Step 2A WIP code
