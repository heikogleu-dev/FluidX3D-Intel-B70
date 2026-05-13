# Wall-Model Bug Diagnosis — Krüger Moving-Wall produces unphysical force artifact

**Datum:** 2026-05-13
**Trigger:** User intervention "Stop! ... Selbst wenn ein Model rückwärts im Raum simuliert wird, kann der Drag niemals negativ sein. Der Fehler ist im Wandmodell!"

---

## Smoking-Gun Datenmatrix

Alle Runs identische Production-WW-Implementation (`apply_wall_model_vehicle` Kernel + Krüger Moving-Wall via `apply_moving_boundaries`), identische 337.5 M Cell Volldomain:

| STL | Geometrie | Fx [N] mit WW | Erwarteter Drag | Δ vs Erwartung |
|---|---|---:|---:|---:|
| Alte `vehicle.stl` (Original) | Smooth, 1.48 M tri, ohne Aero | **+580 N** | ~565 N (OpenFOAM) | +15 N (gut) |
| **`vehicle-mr2-bin.stl`** (Time-Attack mit Aero) | Smooth, 991 k tri | **−610 N** | **+565 N** verified | **−1 175 N** |
| **`vehicle-alt-bin.stl`** (GR Yaris Stock) | Smooth, 5.34 M tri, Spiegel/Doors | **−250 N** | **+463 N** verified | **−713 N** |
| `ahmed_25.stl` (16 tri simplified, flat front) | Sharp edges, flat front | **+11 457 N** | ~31 N (ERCOFTAC) | **+11 426 N** |
| `ahmed_25deg_m_bin.stl` (canonical, 28 k tri, rounded nose) | Smooth nose + flat back/slant | **+19 802 N** | ~31 N | **+19 771 N** |

**Pattern:**
- 2 von 5 Tests gaben **negativen Drag** (physikalisch unmöglich)
- Alle 4 neuen STLs liefern falsche Drag-Werte
- Nur die OLD `vehicle.stl` gibt einen plausiblen Wert — ist sehr wahrscheinlich ein **Glücksfall, kein Validierungserfolg**

---

## Ursache: Moving-Wall-Artefakt in der Force-Berechnung

### Mechanik

Mein WW-Kernel setzt:
```c
u[wall_cell] = scale × u_avg ≈ 0.92 × u_∞ in +X-Richtung
```

Krüger's `apply_moving_boundaries` in `stream_collide` modifiziert dann DDFs:
```c
Δf_i = -6 × w_i × ρ × (c_i · u_solid)
```

Das soll die ANSCHEINENDE Wand-Geschwindigkeit fürs Fluid herstellen (Wall Function Effekt auf BL-Profil). **Aber:**

`object_force(TYPE_S|TYPE_X)` integriert die per-cell Forces aus dem **Momentum-Austausch** — und der MEMENTUM-AUSTAUSCH durch eine "bewegte Wand" gibt dem Fluid Impuls in Strömungsrichtung. Per Newton's 3rd Law spürt die Wand **eine Gegen-Kraft in −X-Richtung** → **negativer Drag-Beitrag**.

Theoretische Magnitude pro Wand-Zelle:
```
ΔF_x_pro_zelle ≈ 2ρ × u_solid_x (in LB)
```

Für u_solid ≈ 0.07 LB, ρ ≈ 1: ΔF_x ≈ -0.14 LB pro Wand-Zelle. Summiert über alle TYPE_S|TYPE_X-Surface-Zellen ergibt das einen großen negativen Beitrag, der vom physikalischen Drag abgezogen wird.

### Warum die alte `vehicle.stl` "funktionierte"

Mit Bounce-Back ohne WW gab die alte STL Fx ≈ 2 219 N. Mit WW Fx ≈ 580 N. Differenz ≈ 1 640 N (≈ Größenordnung des Moving-Wall-Artefakts für diese Body-Größe). Das Ergebnis fiel zufällig in den OpenFOAM-Erwartungsbereich 400-600 N — aber das war Korrelation, nicht Kausalität.

Die OLD STL gibt **−1 640 N WW-Artefakt vs +2 219 N Bounce-Back-Overshoot = 580 N Netto**. Real-Target unbekannt für diese STL.

Für die neuen STLs ist der Bounce-Back-Overshoot kleiner (bessere Geometrie, dünneres Mesh) und der Moving-Wall-Artefakt **größer** (mehr Surface-Cells durch komplexere Geometrie) → Netto wird **negativ**.

Für Ahmed-Body ist die Force-Berechnung pathologisch in andere Richtung — vermutlich weil die Wake-Cells eine andere u_avg-Verteilung erzeugen, die das Artefakt-Vorzeichen umkehrt.

---

## Fix-Strategien

### Option 1: Force-Artefakt analytisch subtrahieren (Quick Fix)

**Idee:** WW-Kernel bleibt unverändert (gut für Fluid-Strömung). Nach `update_force_field` zusätzlich analytisches ΔF_artifact berechnen und subtrahieren.

**Implementation:**
1. Neuer Kernel `subtract_wall_model_artifact`:
   - Für jede TYPE_S|TYPE_X Surface-Zelle: lese u_solid (= u[wall_cell])
   - Berechne ΔF_artifact_x = 2ρ × u_solid_x × (count of fluid neighbors / 19)
   - Akkumuliere in einem separaten F_artifact-Array
2. `object_force` ändern oder erweitern: F_total = F_measured − F_artifact

**Pro:**
- WW-Kernel unverändert (Iron Rule respektiert)
- Reine Post-Processing-Korrektur
- 1 Tag Implementation + Test

**Contra:**
- Analytische Formel evtl. nicht exakt für komplexe Geometrie
- Tunable Faktor evtl. nötig

### Option 2: WW ohne Wall-Movement (saubere Implementation)

**Idee:** Wall Function so implementieren, dass die Wand **stationär** bleibt aber das Fluid-Profil korrekt ist.

**Methode A — Effective-Viscosity-Modifikation:**
Erhöhe nu_local in Fluid-Zellen adjacent zur Wand, so dass das BL-Profil dem PowerLaw entspricht. Force-Berechnung unverändert (kein Moving-Wall).

**Methode B — Direct-DDF-Modification:**
Anstatt u[wall_cell] zu setzen, modifiziere die outgoing DDFs an Wand-Zellen direkt mit der Wall-Function-Shear-Stress. Force ist dann pure Pressure + WW-Shear, beides positiv.

**Pro:**
- Physikalisch sauber
- Kein Force-Artefakt
- Pioneer-Value bei korrekter Implementation

**Contra:**
- 1-2 Wochen Aufwand
- Erfordert WW-Kernel-Rewrite (Iron Rule muss überschrieben werden)
- Validierung notwendig auf bekannter Geometrie

### Option 3: Bouzidi Interpolated Bounce-Back (Roadmap Phase 1)

Bereits geplant — Sub-Grid-Distance-aware Bounce-Back ist kompatibel mit Wall Functions und vermeidet das Moving-Wall-Problem komplett.

**Pro:**
- Bereits auf der Roadmap
- Adressiert auch BB-Pathology (Phase 0c-Befund)

**Contra:**
- 7-15 Tage Aufwand
- Architektur-Studie zuerst nötig (Esoteric-Pull Kompatibilität)

---

## Empfehlung

**Reihenfolge:**
1. **Option 1 (Force-Artefakt subtrahieren)** als Quick Fix — bestätigt Diagnose und stellt vermutlich MR2 ≈ +565 N und Yaris ≈ +463 N her ohne WW-Kernel-Änderung
2. Wenn Option 1 funktioniert: weiter zu Phase 1 (Bouzidi) für saubere Architektur
3. Option 2 nur wenn weder Option 1 noch Phase 1 ausreichend

---

## Wall-Model-Code Status

WW-Code (`apply_wall_model_vehicle` Kernel) **bleibt unverändert** bis User-Entscheidung. Setup.cpp wurde zurückgesetzt (X-Flip-Test wieder entfernt — das war kein zielführender Pfad).

Iron Rule "Wall-Model-Code NICHT anfassen" gilt formell weiter — User-Statement "Der Fehler ist im Wandmodell" indiziert aber Bereitschaft zur Modifikation. Warte auf konkrete Freigabe für eine der 3 Optionen.

---

## Files Status

- `src/defines.hpp` — `WALL_MODEL_VEHICLE` aktiv
- `src/setup.cpp` — `AHMED_MODE=0`, `VEHICLE_GR_YARIS=0`, `VEHICLE_MR2_BIN` defined → läuft Time-Attack MR2 standardmäßig (Target +565 N, gemessen −610 N → −1 175 N Differenz wartet auf Fix)
- `src/kernel.cpp`, `src/lbm.cpp`, `src/lbm.hpp` — Wall-Model unverändert
- X-Flip-Diagnostik in setup.cpp entfernt (war wrong path)
- `bin/forces_*.csv` — Zeitreihen der diversen Tests (gitignored)
