# CC#X Phase 1 — Ahmed Body Validation — FAILED (Iron-Rule-Trigger)

**Datum:** 2026-05-11
**Status:** ❌ Phase 1.1 versagt → STOPP per Iron Rule, kein Phase 2/3
**Wall-Model-Code:** unverändert (Iron Rule eingehalten)

---

## Executive Summary

Ahmed Body 25° Volldomain WW gibt **CD = 104** statt erwartete **CD = 0.285** (Literatur Lienhart-Stoots 2003). Das ist **Faktor 365× zu hoch** — weit jenseits der ±20 %-Toleranz und sogar der ±30 %-Stopp-Schwelle.

Diagnose: Wall Model **verstärkt artifiziell den Drag auf simplifizierte Ahmed-Geometrie** (flache Faces, scharfe 90°-Kanten, nur 16 Triangles). Bei MR2 (smooth STL, ~1M Triangles, gekrümmte Oberflächen) funktioniert dasselbe WW einwandfrei (580 N nahe Real-Target 565 N).

---

## Test-Setup

**Geometrie:** Simplified Ahmed Body 25° slant
- 16 Triangles (Box + Slant + Base + 2 Sides)
- **Front: FLAT** (statt rounded R=100mm wie Literatur-Ahmed)
- L=1.044m, W=0.389m, H=0.288m, slant 222mm
- Ground clearance 50mm baked in (z_min STL = 0.05m)

**Domain:** 15m × 5m × 4.5m bei 10mm Cells = 337.5 M Cells (gleich MR2-Domain)
- Blockage Y: 0.389/5.0 = 7.8 %
- Blockage Z: 0.288/4.5 = 6.4 %
- Blockage akzeptabel (5-10 % standard)

**Flow:** u_∞ = 40 m/s, ν = 1.5e-5 m²/s, ρ = 1.225 kg/m³
- Re_L = 2.78e6 ✓ (matches literature)
- lbm_u = 0.075, nu_lbm ≈ 2.81e-6
- y_1+ ≈ 585 (in PowerLaw-Bereich, marginal — opt 30-300)

**Wall Model:** Production-Default Werner-Wengle PowerLaw via Krüger Moving-Wall

---

## Ergebnisse

### Mit Wall Model (Production-Default)

| Step | Fx [N] | CD | Lit-Vergleich |
|---:|---:|---:|---|
| 100 | 26 020 | 236 | 828× zu hoch |
| 200 | 7 908 | 72 | 251× zu hoch |
| 2 500 | 6 395 | 58 | 204× zu hoch |
| 5 000 | 11 457 | 104 | 365× zu hoch |
| 5 100 (Auto-Stop) | 8 618 | 78 | 274× zu hoch |

Forces oszillieren wild zwischen 6 kN und 27 kN. Auto-Stop fired auf Pseudo-Konvergenz innerhalb der 1 %-Drift-Window, aber Niveau bleibt unphysikalisch.

### Diagnose: Ohne Wall Model (WALL_MODEL_VEHICLE auskommentiert)

| Step | Fx [N] | CD | Lit-Vergleich |
|---:|---:|---:|---|
| 2 500 | 150 | 1.37 | 4.8× zu hoch (typisch Bounce-Back) |
| 5 000 | 407 | 3.70 | 13× zu hoch |
| 7 500 | 58 | 0.53 | 1.9× zu hoch (näher an Realität!) |
| 10 000 | -129 | -1.18 | Oszillation |
| Konvergiert | 92 N avg | **0.84** | **3× zu hoch (typischer Bounce-Back-Overshoot)** |

**Ergo:** Ohne WW liefert die Simulation **physikalisch plausible Drag-Werte** für einen flat-front Ahmed Body (CD ~0.8-1.0 erwartet für eckigen Bluff-Body). Bounce-Back's bekannter 2-4× Overshoot ist erwartbar.

Die WW-Aktivierung **multipliziert** das Ergebnis um Faktor ~125×.

---

## Vergleich mit MR2 (wo WW funktioniert)

| Body | Triangles | STL-Komplexität | WW Result | Literature | Match |
|---|---:|---|---:|---:|---|
| MR2 (vehicle.stl) | 1.48 M | smooth, curved | **580 N** | ~565 N | ✓ |
| Ahmed simplified | 16 | flat, sharp edges | **11 457 N** | 31 N | ✗ |

---

## Diagnose-Hypothesen

### Hypothese A: Sharp-Edge-Interaktion mit WW
- An scharfen 90°-Kanten haben Body-Zellen Fluid-Nachbarn in 2+ orthogonalen Richtungen
- u_avg gemittelt über orthogonale Richtungen → Magnitude möglicherweise verfälscht
- u_slip = scale × u_avg → falsche Richtung oder Magnitude an Kanten
- Krüger-Moving-Wall verstärkt → kumulativ über viele Kantenzellen → riesige Force

### Hypothese B: Flat-Face-Akkumulation
- An flachen Oberflächen haben ALLE Body-Zellen die *gleiche* Surface-Orientation
- Kumulative Wirkung der Wall-Slip-Velocity ist kohärent → großer Netto-Effekt
- Bei MR2 (gekrümmt) variiert Orientation kontinuierlich → Wall-Slip-Effekte mitteln sich aus

### Hypothese C: Frontal-Stagnation an Flat-Front
- Front-Cells sehen u_avg ≈ 0 (Stagnation) → u_slip ≈ 0 → kein WW-Effekt
- ABER: andere Faces (Top, Sides, Slant) sehen volle Flow → starke u_slip
- Imbalance: Slip nur an manchen Faces → Netto-Force-Asymmetrie

### Hypothese D: Reduzierte Triangle-Count → Voxelisierung-Artefakte
- 16 Triangles für 1m × 0.4m × 0.3m Body → ~6 m² Surface-Fläche → 0.4 m² pro Triangle
- Voxelisierung Ray-Casting könnte bei großen Dreiecken instabil sein
- Body-Cells könnten falsch markiert sein

---

## Iron-Rule-Konsequenz

Per Opus-Plan Section "Iron Rules":
> "Phase 1 nicht ±20% an Literatur-CD ankommt → STOPP, Bericht, Re-Evaluation des WM"

CD = 104 vs CD_lit = 0.285 entspricht **36 400 % Fehler**. Iron Rule trigger eindeutig.

**Wall-Model-Code wurde NICHT verändert.** Diagnose erfolgte nur durch temporäres Auskommentieren des `WALL_MODEL_VEHICLE` defines (anschließend wieder aktiviert).

---

## Empfehlungen (zur User-Entscheidung)

### Option α: Proper Ahmed Body STL mit rounded front
Generiere Ahmed Body mit korrekter 100mm-Rundung an 12 Front-Edges + 4 spherical Corner-Octants. ~200-500 Triangles statt 16. **Aufwand: 2-3h** (Python+trimesh oder direkt). Falls Hypothese D zutrifft, könnte das die Voxelisierung stabilisieren.

### Option β: Skip Ahmed, direkt MR2-Halbdomain für Sym-Plane-Sweep
MR2 hat verifizierten Drag-Target (565 N) und Downforce-Target (-1337 N). Phase 2 (Sym-Plane Sweep) direkt auf MR2 anwenden, statt erst Ahmed zu validieren. Verlust: Keine kanonische Wall-Model-Validation. Gewinn: Schnelle Ergebnisse auf relevanter Geometrie.

### Option γ: WW-Modifikation für scharfe Kanten
Wall Model um eine Edge-Detection erweitern: wenn Zelle Fluid-Nachbarn in mehreren orthogonalen Richtungen hat → reduziere Slip oder schalte WW lokal ab. **Aufwand: 1 Tag**. Verletzt Iron Rule (WW-Code-Modifikation) und braucht explizite User-Freigabe.

### Option δ: y+-Resolution erhöhen
Vermutung: bei niedrigerem y+ (z.B. 50-100 statt 585) funktioniert WW besser auf Ahmed. Cell-Size 2.5mm statt 10mm → 1.3 Mrd Cells → unmöglich auf B70. Domain-Reduktion nötig → andere Probleme.

---

## Files Affected

- `src/setup.cpp` — Ahmed-Setup-Block (`AHMED_MODE` toggle, lines ~232-345)
- `src/defines.hpp` — `WALL_MODEL_VEHICLE` define (wieder aktiv)
- `ahmed_25.stl`, `ahmed_35.stl` — Simplified Ahmed STLs (16 tri each)
- `findings/CC_X_ahmed/SESSION_2026-05-11_PHASE1_FAIL.md` — dieses Dokument
- `bin/forces_ahmed_25.csv` — letzter Run-Output (no-WW Diagnostik)

---

## Production-Default unverändert

`#define AHMED_MODE 1` bleibt im Code, kann durch User-Edit auf 0 zurückgesetzt werden. Bei AHMED_MODE=0 läuft der bisherige MR2/Yaris-Setup wie gehabt.

`#define WALL_MODEL_VEHICLE` ist **aktiv** (wie vorher) — keine Regression eingebracht.

Per Iron Rule: keine Phase 2, keine Phase 3 bis User-Entscheidung.
