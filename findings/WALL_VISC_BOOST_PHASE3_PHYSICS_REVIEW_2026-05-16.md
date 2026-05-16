# WALL_VISC_BOOST Phase 3 — physics review + Phase 4 design sketch

**Datum:** 2026-05-16
**Trigger:** User-Frage zur physikalischen Korrektheit des Multi-Cell-Wandmodells.
**Status:** Phase 3 läuft (Chunk 42/~150 zum Zeitpunkt der Review), Forces stabilisiert bei Fx_far ≈ Fx_near ≈ 1.9 kN.

## TL;DR

Phase 3 Multi-Cell hat **zwei systematische Physik-Probleme** und einen **korrekt kalibrierten** Tiefen-Bereich:

| Aspekt | Verdikt |
|---|---|
| Eindringtiefe (15 mm Near, 45 mm Far) | ✅ physikalisch plausibel für Mittendrittel der Karosserie, ⚠️ zu tief am Frontstoß, zu flach im Wake |
| u_τ-Formel an Layer 2/3 | ❌ Werner-Wengle-Konstanten gelten nur für y_p=0.5 lu; an Layer 2 (y=1.5) und Layer 3 (y=2.5) wird u_τ systematisch überschätzt |
| `ν_t = κ·y·u_τ` ohne Damping/Cap | ❌ Formel nur im Log-Layer (y+ ≈ 30–300) valide; bei uns ist Near Layer 3 schon bei y+ ≈ 1250, Far Layer 3 bei y+ ≈ 3750 — defect/wake region, lineares Wachstum überschätzt ν_t |
| Wirkungs-Richtung | ✅ stärkerer Damping mit Wand-Distanz ist physikalisch korrekt (Log-Layer) — nur die Magnitude ist zu groß in den äußeren Layern |

## Detail: u_τ-Closed-Form-Problem

Der Phase-3-Kernel-Block (kernel.cpp:1933 ff):
```c
const float u_mag = sqrt(sq(uxn)+sq(uyn)+sq(uzn));      // u am AKTUELLEN cell n
const float u_visc2 = 2.0f*nu*u_mag;                    // <-- hardgecodet für y_p=0.5 lu
const float u_log2  = 0.0246384f * pow(nu,0.25f) * pow(u_mag,1.75f);
const float u_tau   = sqrt(max(u_visc2, u_log2));
const float y_center = (float)wall_dist - 0.5f;
const float nu_t_target = 0.41f * y_center * u_tau;
```

Werner & Wengle 1991 leiten die Closed-Form-Konstanten `2·ν·u_t` und `0.0246384`
aus der **Integration der Power-Law-Geschwindigkeit über die erste Wand-Zelle**
ab, mit y_p = 0.5 lu. Phase 3 ruft die exakt-selbe Formel auch an Layer 2 (y=1.5 lu)
und Layer 3 (y=2.5 lu) auf und nimmt das lokale `u_mag` (das dort logischerweise
größer ist, da weiter von Wand entfernt). Das produziert:

| Layer | y_actual | Formel-Annahme | u_τ-Verzerrung |
|---|---:|---:|---|
| 1 | 0.5 lu | 0.5 lu | ✓ korrekt |
| 2 | 1.5 lu | (implizit 0.5) | u_τ überschätzt |
| 3 | 2.5 lu | (implizit 0.5) | u_τ deutlich überschätzt |

**Standard-Vorgehen in der Literatur** (Cebeci-Smith, Baldwin-Lomax, alle Wall-Function-Familien):
1. u_t **einmal** an Layer 1 lesen
2. u_τ **einmal** mit y_p=0.5 berechnen
3. ν_t = κ·y·u_τ an Layer 1..N mit **gleichem** u_τ und unterschiedlichem y anwenden

## Detail: Gültigkeitsbereich `ν_t = κ·y·u_τ`

Lineares Wachstum gilt nur im **Log-Layer**, in der LBM-Literatur einheitlich
angegeben mit `30 < y+ < 300` (Pope 2000, Werner-Wengle 1991, alle SGS-Reviews).

Für unsere Auto-Geometrie bei 30 m/s, geschätztes u_τ ≈ 1.5 m/s (Reibung mittlerer
Karosserie-Bereich), ν_air = 1.5·10⁻⁵ m²/s:

| Layer | y_Near (m) | y+_Near | y_Far (m) | y+_Far | Regime |
|---|---:|---:|---:|---:|---|
| 1 | 0.0025 | ≈ 250 | 0.0075 | ≈ 750 | Near✓ / Far × |
| 2 | 0.0075 | ≈ 750 | 0.0225 | ≈ 2 250 | beide × |
| 3 | 0.0125 | ≈ 1 250 | 0.0375 | ≈ 3 750 | beide × |

→ **Nur Near Layer 1 ist sauber im Log-Layer.** Alle anderen Layers sind im
Defect-Layer oder schon im outer-BL/free-stream-Bereich, wo die echte ν_t-Kurve
**abflacht oder fällt** (Cebeci-Smith outer cap, Klebanoff-Intermittenz).

Phase 3 lineares Wachstum produziert dort Overshoot. Aber der Overshoot ist
in derselben Richtung wie der physikalische Effekt (mehr Damping) — das Wandmodell
ist also **zu aggressiv, aber nicht falsch herum**.

## Physikalisch korrekte Korrekturen (alle in Standard-Lehrbüchern)

### Van Driest (1956)
```
ν_t = (κ·y)² · |dU/dy| · [1 - exp(-y+/A+)]²
A+ = 26 (Van Driest), modernere DNS-Fits: 33
```
Wirkt im viscous sublayer (y+<5) → ν_t → 0. Bei y+>50 geht der Damping-Term
gegen 1, also kein Effekt im Log-Layer. **Erster Fix: an unsere Mixing-Length-Formel
einfach den `[1-exp(-y+/26)]²` Term anhängen.**

### Cebeci-Smith Outer-Cap (1974)
```
ν_t = min(ν_t_inner, ν_t_outer)
ν_t_inner = κ·y·u_τ · [1-exp(-y+/26)]²
ν_t_outer = 0.0168 · u_τ · δ · F_Klebanoff(y/δ)
F_Klebanoff(η) = 1 / (1 + 5.5·η^6)
```
δ = lokale BL-Dicke. Für unsere Karosserie ca. 5-30 mm je nach Position
(siehe nächster Abschnitt).

### Sigma-SGS-Modell (Toda & Nicoud 2009)
ν_t läuft **algorithmisch O(y³)** gegen 0 an der Wand, **ohne** explizite
Wand-Distanz, **ohne** Damping-Funktion. Alternative zum Smagorinsky+Mixing-Length-Stack.
Hat in LBM-Literatur (Nathen et al 2017, Krastev & Silva 2018) gute Validierungen.

## Geometrie-aware Boost — Phase 4 Design

User-Idee 2026-05-16: *"Kann man intelligent nach Wandausrichtung und
Anströmrichtung/winkel separieren ob es Vorderkante, Mittendrittel oder Heck ist?"*

**Antwort: ja, das ist exakt der Schritt zu Cebeci-Smith / Baldwin-Lomax.**
Wir haben alle Zutaten:

### Pro Wand-adjazente Zelle:
1. **Wand-Normale** `n_wall`: aus TYPE_S-Nachbar-Pattern in BFS-Pass 1. Kostenlos zur populate_wall_adj_flag-Erweiterung.
2. **Lokale Strömungsrichtung** `u_local / |u_local|`: schon im Kernel.
3. **Anström-Winkel** `cos α = -n_wall · û_local`.

### Region-Klassifikation:
| Region | cos α | Charakter | Phase-4-Boost-Profil |
|---|---:|---|---|
| Front-Stagnation | > +0.7 | windward, dünne BL | Nur Layer 1, voller κ·y·u_τ |
| Front-Schräge | +0.3 … +0.7 | aufbauende BL | Layer 1–2, Van-Driest |
| Mitte / Seite / Dach | −0.3 … +0.3 | etablierte BL | Layer 1–3 wie Phase 3 |
| Rückseite / Lee | −0.7 … −0.3 | separierender BL | Layer 1–4, Outer-Cap |
| Wake-Kern | < −0.7 | freie Turbulenz | Wall-Model aus |

### Optional: x_from_leading_edge als zweite Achse
Flat-Plate-BL: `δ(x) ≈ 0.37·x / Re_x^(0.2)`. Mit Frontstoß-X-Koordinate
des Fahrzeug-BBox-Min lässt sich δ_local an jeder Wand-Zelle abschätzen:
- `MAX_WALL_DISTANCE_local = ceil(δ_local / dx)`
- Outer-Cap `ν_t ≤ 0.0168·u_τ·δ_local`

### Implementierungs-Aufwand
- Datenstruktur: `Memory<uchar3> wall_normal` (3 B/cell, ~720 MB Near+Far gesamt), gepackte 8-Bit-Vektoren
- populate_wall_adj_flag-Erweiterung: ~20 LOC für Normalen-Berechnung + Propagation im BFS
- Kernel: ~15 LOC für dot-product + Region-Selektor + skalierter Boost
- Setup: x_LE aus Fahrzeug-BBox auslesen
- **Gesamt: 4–6 h für sauberes Phase 4 mit Wand-Normalen + Region-Selektor**

## Empfohlene Schritte

1. **Phase 3 Run zu Ende laufen lassen** + ParaView-Sichtung (~15 min noch)
2. **Wenn Phase 3 visuell und force-mäßig im erwarteten Bereich** → kleiner Fix: u_τ einmalig aus Layer 1, Van-Driest-Damping anhängen. ~30 min Code.
3. **Phase 4 mit Wand-Normalen + cos α-Region-Selektor** als Hauptpfad. ~4–6 h.
4. **Sigma-SGS als Backup-Pfad** falls Phase 4 zu komplex wird oder gleichzeitig Smagorinsky-Stack vereinfacht werden soll. ~3–5 h.

## Sources

- Werner H., Wengle H. (1991): "Large-eddy simulation of turbulent flow over and around a cube in a plate channel", 8th Symp. Turbulent Shear Flows, Munich.
- Van Driest E.R. (1956): "On Turbulent Flow Near a Wall", J. Aerosol Sci. 23.
- Cebeci T., Smith A.M.O. (1974): "Analysis of Turbulent Boundary Layers", Academic Press.
- Baldwin B.S., Lomax H. (1978): "Thin Layer Approximation and Algebraic Model for Separated Turbulent Flows", AIAA 78-257.
- Pope S.B. (2000): "Turbulent Flows", Cambridge, §7.1.4 wall units, §10.2 mixing-length.
- Toda S., Nicoud F. (2009): "Sigma SGS Model".
- Nathen P. et al (2017): "Adaptive σ-Smagorinsky in LBM".
- ANSYS Fluent Theory Guide § 4.12.6 LES Near-Wall Treatment.
- TRACE Werner-Wengle-Closed-Form Documentation (DLR).
- Wikipedia: Mixing length model, Boundary layer thickness.
- CFD-Online Wiki: Near-wall treatment for LES models.

## See Also

- [WALL_TREATMENT_SUMMARY_2026-05-15.md](WALL_TREATMENT_SUMMARY_2026-05-15.md) — 5 gescheiterte DDF-Modifikations-Ansätze + WALL_VISC_BOOST als einziger Survivor
- [TODO_2026-05-16.md](TODO_2026-05-16.md) — ursprüngliche Phase-3-Test-Planung (jetzt erweitert um Phase 4 hier)
- [BOUZIDI_EP_PULL_INCOMPATIBILITY_2026-05-15.md](BOUZIDI_EP_PULL_INCOMPATIBILITY_2026-05-15.md) — warum DDF-Mod-Ansätze in EP-pull nicht funktionieren
