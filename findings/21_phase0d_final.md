# Phase 0d Final Report — Wall-Model Bug Verification & Fix Recommendation

**Datum:** 2026-05-13
**Verification-Status:** **FULL** (alle 3 Schritte A, B, C konvergent)
**Empfehlung:** **Option 1 (Force-Artefakt analytisch subtrahieren)** als Quick-Fix, **Option 2 (OpenLB-Style DDF-Reconstruction)** als architekturell-sauberer Long-Term-Fix.

---

## Executive Summary

Die WW-Implementierung (`apply_wall_model_vehicle` + Krüger-Moving-Wall via `apply_moving_boundaries`) erzeugt durch Schreiben von `u[wall_cell]` einen unphysikalischen Moving-Wall-Force-Beitrag, der von `update_force_field` mit-integriert wird. Dies wurde durch drei unabhängige Pfade vollständig verifiziert:

1. **Code-Audit (Schritt A):** Analytisch hergeleitete Artefakt-Formel `ΔF_artifact = -4 · u_solid` pro vollständig umgebener Zelle (D3Q19 Isotropie-Identität)
2. **Cube Minimal-Test (Schritt B):** 1 m³ Würfel bei Re = 10⁵, freie Anströmung:
   - **Ohne WW: CD = 1.02** (vs Hoerner-Literatur 1.05, 97 % Match, ✓)
   - **Mit WW: CD = 79 oszillierend** (vs 1.05, **77× zu hoch**, ✗)
3. **OpenLB Cross-Reference (Schritt C):** `WallFunctionBoundaryProcessor3D` schreibt **f_eq + f_neq** direkt an Fluid-Zellen, **nicht u an Solid-Zellen** → vermeidet Moving-Wall-Artefakt architektonisch

---

## 1. Code-Audit (Schritt A) — Quintessenz

Vollständiger Audit in [`findings/20_ww_audit.md`](20_ww_audit.md). Kernpunkte:

### Data-Flow:
```
apply_wall_model_vehicle: u[wall_cell] ← scale × u_avg  (≈ 0.92 × u_∞)
   ↓
update_moving_boundaries: flags |= TYPE_MS bei Fluid-Cells mit u_solid≠0 Nachbar
   ↓
stream_collide (inline apply_moving_boundaries):
   Δf_i = -6 × w_i × ρ × (c_i · u_solid)
   fhn[i] += Δf_i  ← modifizierte DDFs werden über Streaming in TYPE_S Cells geschrieben
   ↓
update_force_field: F[n] = 2 × Σ c_i × f_i  ← liest modifizierte DDFs!
   ↓
object_force: atomic-sum F[n] über Cells mit flags == TYPE_S|TYPE_X
```

### Analytische Artefakt-Formel (D3Q19):
Für eine vollständig von Solid umgebene Zelle mit u_solid = (u_x, 0, 0):
```
ΔF_x = 2 × Σ_i c_i_x × Δf_i
     = -12 × Σ_i w_i × c_i_x × (c_i · u_solid)
     = -12 × (1/3) × u_x      ← Lattice-Isotropie Σ w_i c_i² = c_s² = 1/3
     = -4 × u_x  [LB units pro Cell]
```

Für Surface-Cells mit n_solid Solid-Nachbarn von 18 D3Q19-Stencil-Richtungen:
```
ΔF_x ≈ -4 × (n_solid / 18) × u_solid_x  pro Cell
```

---

## 2. Cube Minimal-Test (Schritt B) — Quantitative Isolation

**Setup V1:** 1 m³ Cube bei Re = 10⁵ (u = 1.5 m/s, ν = 1.5e-5, ρ = 1.225)
- Domain 8L × 4L × 4L bei 20 mm Cells = 4 M Zellen
- Frei aufgehängt im Strömungsfeld (keine Rolling-Road, kein Y/Z-Wall-Kontakt)
- Free-stream TYPE_E auf allen 6 Domain-Seiten
- Reference CD = 1.05 (Hoerner, Re-unabhängig im turbulenten Bereich) → F_target ≈ 1.45 N

**Ergebnisse:**

| Variante | Fx [N] | CD measured | vs Lit 1.05 | Verdict |
|---|---:|---:|---|---|
| **Ohne WW** (`#define WALL_MODEL_VEHICLE` auskommentiert) | **1.404** | **1.019** | -3 % | ✅ Perfekt |
| **Mit WW** (Production-Default) | osz. -21 bis +200, "konvergiert" 109 | osz. -15 bis +145, "konvergiert" 79 | **+7400 %** | ❌ Pathologisch |

**Caveat:** V1 hat 25 % Y/Z-Blockage (zu eng). V2-Re-Run mit 12.5 % Blockage (16L × 8L × 8L Domain, 65 M Cells) wurde gestartet aber durch GPU-Hang abgebrochen. Da das Signal "ohne WW" → 1.02 vs "mit WW" → 79 einen **Faktor ~80×** beträgt und Blockage-Korrekturen typisch < 10 % sind, ist die Schlussfolgerung **robust gegen die Blockage-Unsicherheit**.

**Interpretation:**
- Bounce-Back alleine liefert auf simpelster Bluff-Body-Geometrie **PHYSIKALISCH KORREKT** (CD = 1.02 vs Lit 1.05)
- Wall-Model addiert in identischer Konfiguration einen **Faktor 77× falschen Bias** (Oszillation zudem)
- → WW-Kernel ist **alleinige Ursache** der Force-Pathologie

---

## 3. OpenLB Cross-Reference (Schritt C) — Architektonische Diskrepanz

Source: `openLB/openLB/src/boundary/wallFunctionBoundaryPostProcessors3D.hh` (Marc Haussmann, 2018)

### OpenLB-Methode (`ComputeWallFunction`):

```cpp
void ComputeWallFunction(BlockLattice3D& blockLattice, int x, int y, int z) {
  Cell& cell_bc = blockLattice.get(x,y,z);  // ← BOUNDARY-CELL ist FLUID-CELL adjacent to wall
  T u_bc[3], rho_bc, fneq_bc[19];

  ComputeUWall(...);          // u_bc = Werner-Wengle / Musker → wall function output
  ComputeRhoWall(...);        // rho_bc via Zou-He
  ComputeFneqWall(...);       // f_neq aus zweite-Ordnung-Gradient & Pi-Tensor

  // ★ DIRECT DDF-RECONSTRUCTION an FLUID-CELL ★
  for (int iPop = 0; iPop < 19; ++iPop) {
    cell_bc[iPop] = dynamics->computeEquilibrium(iPop, rho_bc, u_bc, uSqr_bc) + fneq_bc[iPop];
  }
}
```

### Architektonische Schlüssel-Unterschiede

| Aspekt | OpenLB | FluidX3D CC#10 |
|---|---|---|
| Wo wird WW appliziert | An **Fluid-Cells** adjacent zur Wand | An **Solid-Cells** der Wand (TYPE_S\|TYPE_X) |
| Was wird modifiziert | DDFs direkt: `f[i] = f_eq(u_wall) + f_neq` | `u[wall_cell] = u_slip` |
| Krüger-Moving-Wall-Term | **Nicht verwendet** | **Triggert apply_moving_boundaries** → Δf_i Krüger-Korrektur |
| Force-Berechnung-Konflikt | Keine — Force aus τ_w direkt integriert | **Ja — Moving-Wall-Term landet in object_force** |
| f_neq Reconstruction | Zou-He + Pi-Tensor (regularized BC) | n/a — DDFs implizit über Bounce-Back |

→ OpenLB **vermeidet das Problem architektonisch** indem es DDFs direkt setzt statt eine bewegte Wand zu simulieren. Unsere Krüger-Moving-Wall-Implementierung war ein konzeptioneller Shortcut, der einen Force-Side-Effect hat.

---

## 4. Theorie-Verifikation-Matrix (final)

| Hypothese | A: Theory | B: Test | C: Cross-Ref | Status |
|---|:---:|:---:|:---:|:---:|
| WW schreibt u_solid an Wand-Cells | ✓ | ✓ | n/a | **VERIFIED** |
| Krüger Δf_i wird zu DDFs addiert | ✓ | n/a | n/a | **VERIFIED** |
| update_force_field liest modifizierte DDFs | ✓ | ✓ | n/a | **VERIFIED** |
| Artefakt ist Faktor 77× über realer Drag | n/a | ✓ | n/a | **VERIFIED** |
| OpenLB nutzt andere Architektur | n/a | n/a | ✓ | **VERIFIED** |
| Bounce-Back alleine liefert korrekten CD | n/a | ✓ | n/a | **VERIFIED** (cube ohne WW = 1.02 vs lit 1.05) |

---

## 5. Fix-Strategien — Detaillierter Vergleich

### Option 1: Force-Artefakt analytisch subtrahieren — **EMPFOHLEN als Quick Fix**

**Konzept:** WW-Kernel bleibt unverändert (Fluid-Strömung profitiert vom Wall-Function-Effekt). Nach `update_force_field` zusätzlicher Kernel `compute_wall_model_artifact` berechnet ΔF analytisch und subtrahiert von Total-Force.

**Implementation:**
1. Neuer Kernel `compute_wall_model_artifact(u, flags, F_artifact)`:
   - Für jede TYPE_S|TYPE_X Zelle: lese `u[n]`, zähle Solid-Nachbarn `n_solid`
   - Berechne `ΔF_artifact[n] = -4 × (n_solid / 18) × u[n]` (vektoriell für x,y,z)
2. `update_force_field` und `compute_wall_model_artifact` schreiben in separate F-Arrays
3. Erweiterung von `object_force` oder neuem Reduce: `F_total = F_measured - F_artifact`

**Pro:**
- WW-Kernel unverändert (Iron Rule respektiert)
- Reine Post-Processing-Subtraktion
- ~1 Tag Implementation + Validation
- Validierbar via Cube-Test (sollte CD ≈ 1.0–1.2 ergeben, im Rahmen WW-Korrektur)

**Contra:**
- Analytische Formel ist Approximation (Lattice-Isotropie-Identität gilt streng nur isotrop)
- Empirischer Korrektur-Faktor evtl. nötig
- Konzeptionell unsauber: simuliere falsch, korrigiere post-hoc

### Option 2: WW à la OpenLB (DDF-Reconstruction, kein Moving-Wall) — **EMPFOHLEN als Long-Term**

**Konzept:** Wand bleibt stationär (u[wall_cell] = 0). Wall-Function-Effekt wird über direkte DDF-Modifikation an Fluid-Cells adjacent zur Wand realisiert (regularized BC pattern).

**Implementation:**
1. Neuer Kernel `apply_wall_function_ddf` läuft an TYPE_MS Fluid-Cells (NICHT an TYPE_S Solid-Cells):
   - Computiere u_wall via Werner-Wengle PowerLaw (wie jetzt schon)
   - Computiere f_neq via Pi-Tensor-Reconstruction (Zou-He + zweite Momente)
   - Setze `f[i] = f_eq(rho_bc, u_wall) + f_neq[i]` für alle 19 Populationen
2. `apply_wall_model_vehicle` schreibt NICHT mehr u[wall_cell]
3. Krüger-Moving-Wall bleibt für legitimen Use-Case (rotating wheels, rolling road)

**Pro:**
- Architektonisch sauber (matches OpenLB / Latt-Chopard literature)
- Kein Force-Artefakt
- Pioneer-Value als erste FluidX3D Implementation einer regularized Wall-Function-BC

**Contra:**
- 1-2 Wochen Aufwand
- Erfordert Pi-Tensor-Computation aus DDFs (non-trivial)
- Iron-Rule-Override für WW-Kernel-Rewrite notwendig
- Validierung mit Channel-Flow Reference erforderlich

### Option 3: Bouzidi Interpolated BB (Phase 1 Roadmap)

Schon geplant. Sub-Grid-Distance-aware Bounce-Back ist kompatibel mit Wall-Functions ohne Moving-Wall-Trick. 7-15 Tage. Adressiert auch BB-Resolution-Pathology aus Phase 0c-Plan (durch GPU-Hang im V2-Test noch offen).

---

## 6. Empfehlung & STOPP-Gate

**Empfohlener Pfad:**

1. **Sofort: Option 1 implementieren** (1 Tag, minimal-invasiv, validierbar):
   - Neuer Kernel `compute_wall_model_artifact` in `kernel.cpp`
   - Erweitere `update_force_field`-Flow um Subtraktion
   - Cube-Test soll dann CD ≈ 1.0–1.2 ergeben (kleine WW-Korrektur)
   - MR2 Time-Attack Re-Run soll Fx ≈ +500 N geben (statt -610 N)
   - Yaris Re-Run soll Fx ≈ +400 N geben (statt -250 N)

2. **Bei erfolgreichem Option 1: Phase 0c (BB-Resolution-Sweep) re-starten**:
   - Quantifiziere ob restliche BB-Pathology Resolution-skalierend ist
   - Informiert Phase 1 (Bouzidi) Priorisierung

3. **Phase 1 (Bouzidi)** als geplante saubere Lösung beibehalten — adressiert auch das Resolution-Problem

**STOPP-Gate für Heiko:**
- WW-Kernel `apply_wall_model_vehicle` selbst bleibt unverändert (Iron Rule)
- Aber neuer Kernel `compute_wall_model_artifact` zu schreiben erfordert User-OK
- Dieser Bericht ist das STOPP-Gate. **Warte auf Heiko's Entscheidung für Option 1 / Option 2 / andere Strategie.**

---

## 7. V2-Cube-Test Status (Hardware-Caveat)

V2 (16L × 8L × 8L, 12.5 % Blockage, 65 M Cells) wurde gestartet, aber **GPU-Hang nach ~1 Minute Init-Phase** verursachte System-Reboot. Möglicherweise unabhängige Hardware-Instabilität, möglicherweise Numerical-Instability der WW-Pathology bei größerer Surface-Cell-Anzahl (mehr Cells × Artefakt → größere DDF-Oszillation → schlechtere FP16-Konvergenz → Crash).

**Konsequenz:** V1-Daten (4 M Cells, 25 % Blockage, stabil durchgelaufen) sind die einzigen verifizierten Cube-Daten. Da Signal-to-Noise V1 schon 80× ist (Blockage typisch ≤10 % Korrektur), ist die Conclusion robust ohne V2.

V2-Retry kann nach Option-1-Implementation oder als Sanity-Check erfolgen — derzeit nicht kritisch für die Diagnose.

---

## 8. Files committed

- `findings/20_ww_audit.md` — Code-Audit (Schritt A)
- `findings/21_phase0d_final.md` — dieser Report (Schritt D)
- `src/setup.cpp` — `CUBE_VALIDATION` toggle, `main_setup_cube` für Diagnose-Tests
- `bin/forces_cube_val_WW.csv` — V1-WW-Daten (gitignored, aber lokal vorhanden)

WW-Kernel-Code (`apply_wall_model_vehicle` in kernel.cpp) **unverändert**.
Production-Default (`AHMED_MODE=0`, `VEHICLE_MR2_BIN`, WW aktiv) — User kann sofort die jetzige WW-Implementierung weiter nutzen (mit dem dokumentierten Force-Bias).
