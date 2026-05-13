# Finding 29: Re/y+ Matrix per Geometry — WW-Need Range Analysis

**Datum:** 2026-05-13
**Status:** Foundation-data for WW-Restart Phase A. Empirische y+ pro
Test-Geometrie zur Bestimmung: welche braucht Wall-Modeling, welche nicht.

---

## Methodik

y+ Berechnung (Turbulent flat-plate BL approximation):
```
C_f ≈ 0.058 × Re_L^(-1/5)        (Skin-friction coefficient)
u_τ = u_∞ × sqrt(C_f / 2)         (Friction velocity)
y   = 0.5 × Δx                    (BB convention: first fluid cell)
y+  = y × u_τ / ν                 (Dimensionless wall distance)
```

Faustregel:
- y+ < 5 → Viscous sublayer, BB approx OK, **kein WW nötig**
- 5 < y+ < 30 → Buffer layer, **ambig**
- y+ > 30 → Log-law region, **WW essentiell**

---

## Geometry-by-Geometry Matrix

| Geometry | L [m] | u_∞ [m/s] | ν [m²/s] | Re_L | Δx [mm] | C_f | u_τ [m/s] | y [m] | **y+** | WW-Need |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| Cube | 1.0 | 1.5 | 1.5e-5 | 1.0e5 | 25 | 0.00580 | 0.081 | 0.0125 | **68** | ⚠️ log-law, but pressure-drag dominates at sharp edges — WW peripheral |
| Ahmed 25° | 1.044 | 40 | 1.5e-5 | 2.78e6 | 6 | 0.00299 | 1.55 | 0.003 | **310** | ✅ WW essential |
| MR2 Time-Attack | 4.0 | 30 | 1.48e-5 | 8.1e6 | 8.54 | 0.00242 | 1.04 | 0.00427 | **300** | ✅ WW essential |
| GR Yaris | ~4.0 | 30 | 1.48e-5 | ~8.1e6 | 8.54 | 0.00242 | 1.04 | 0.00427 | **300** | ✅ WW essential |
| Volldomain Reference | 4.0 | 30 | 1.48e-5 | 8.1e6 | 10 | 0.00242 | 1.04 | 0.005 | **351** | ✅ WW essential |

---

## Schlüsselbeobachtungen

### 1. Cube ist ein Sonderfall
y+=68 liegt im Log-Law-Bereich. Theoretisch sollte WW helfen, **aber**:
- Sharp-edge Separation am front face: Flow trennt sich sofort, keine
  ausgebildete BL.
- Drag = Pressure-Drag (>95%), nicht Skin-Friction.
- WW operiert nur an attached BL. An separated regions wirkt es nicht.
- Empirisch: Cube CD=1.10 mit BB matcht Hoerner CD=1.05 → WW nicht
  benötigt, BB tut den richtigen Job (pressure-drag korrekt erfasst).

### 2. Smooth STL Vehicles (MR2/Yaris)
y+ ≈ 300 — klar im Log-Law-Bereich. WW essentiell. BB allein gibt:
- MR2: 3.2× over-target (1820 N vs 565 N)
- Yaris: untested aber wohl ~3× over-target

Lehmann's "1.3-2.0× Cd too large" stimmt qualitativ (BB ist deutlich zu
hoch), aber unsere 3.2× liegt am oberen Ende seines Bereichs, weil Re_L
unserer Vehicle Tests (~8M) über Ahmed's Re (~2.8M) liegt.

### 3. Ahmed Body
y+ ≈ 310, ähnlich Vehicles. Aber: 16-Triangle simplified Ahmed hat
flat-faced sharp-edged structure → ähnlich Cube, separation an Kanten.
Wenn WW dort versagte (Phase 1: 125× force-amplification), war das wegen
geometry-sensitivity der WW-Implementation, nicht wegen y+-Range.

---

## Implications für Phase B (Method Decision)

| Method | Cube | Ahmed | MR2/Yaris | Conclusion |
|---|:---:|:---:|:---:|---|
| Pure BB (Step-1b Safe) | ✅ | ❌ | ❌ | Cube OK, Vehicles overpredict |
| Halved Krüger (-3 statt -6) | ❌ ↑drag | ? | partial fix | Cube broken, Vehicle ~2× residual |
| OpenLB Pi-Tensor f_neq | ✅ | ✅ | ✅ | All geometries handled correctly |
| Local Viscosity Modification | ✅ | ✅ | ✅ | Implementable, Smagorinsky-style |
| Bouzidi Phase 1 (Sub-grid BB) | improved | improved | improved | Reduces overshoot but doesn't solve WW |

→ Empfehlung: für Phase-B-Decision wird vor allem die Komplex-vs-Geometrie-
Trade-Off relevant. Bouzidi-Phase-1 verbessert BB für alle, könnte für
Cube alleine sogar perfekt sein. OpenLB-Pi-Tensor ist die einzige
all-geometries-correct Methode.

---

## Caveats

- y+ Berechnung basiert auf flat-plate BL approximation. Real-vehicle hat
  Druckgradienten, Krümmung, separation, recirculation → lokale y+ kann
  ±50% variieren.
- C_f = 0.058·Re^(-1/5) ist Re-Bereich-spezifisch (10^5 < Re < 10^7
  validiert). Außerhalb ungenau.
- u_∞ = freestream velocity. An separated regions wirkt lokale Geschwindigkeit
  weniger → effective y+ niedriger als hier geschätzt.

Diese Schätzungen sind **Größenordnungs-korrekt**, sollten als Indikator,
nicht als exakte Zahlen interpretiert werden.

---

## See Also

- [[25_ww_six_failed_attempts]] — Empirische Cube vs MR2 Vergleichsdaten
- [[27_typex_force_isolation]] — Force-Marker-Strategie
- TEIL A.2 (`/tmp/force_code_audit.md`) — Force-Computation Code-Path Audit
