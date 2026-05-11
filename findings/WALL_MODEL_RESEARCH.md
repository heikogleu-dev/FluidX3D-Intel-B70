# Wall Model Recherche — FluidX3D B70 Pro

**Datum:** 2026-05-11
**Kontext:** Sport-Car Aero-Simulation, Volldomain Fx = 2219 N (Faktor 4-5× über OpenFOAM RANS ~400-600 N erwartet)
**Ursache:** FluidX3D nutzt nur Bounce-Back ohne Wall Model → bei Re=9.12M und Δx=10mm liegt y+ der ersten Fluidzelle bei ~3000-5000 (sollte <30 für log-law, <1 für Direct Resolution)
**Ziel:** Realistische Drag/Lift via Werner-Wengle Wall Model

---

## 1. Architekturen-Vergleich

| Approach | Methode | Quelle | Komplexität | Erwartete Genauigkeit |
|----------|---------|--------|-------------|----------------------|
| **A** | Krüger "Moving Wall" mit u_slip aus WW-PowerLaw | Krüger LBM Book Ch. 5.3.4 + Han 2021 inspired | **1-2 Tage** | ⭐⭐⭐ |
| **B** | Wall Function Boundary (WFB) à la Han 2021 | Han, Chen, Sagaut 2021 (Phys Fluids 33) | 1 Woche | ⭐⭐⭐⭐ |
| **C** | OpenLB regularized BC (f_eq+f_neq) | OpenLB `wallFunctionBoundaryPostProcessors3D` | 2 Wochen | ⭐⭐⭐⭐⭐ |

---

## 2. Werner-Wengle Wall Function (PowerLaw, geschlossene Form)

OpenLB-Implementierung (`functors/lattice/wallFunction3D.hh`):

```
A = 8.3                    // PowerLaw-Koeffizient
B = 1/7                    // PowerLaw-Exponent
y_switch = (A/2)^(2/(1-B))/2  ≈ 11.81  // y+ Übergang viskos↔log

if (y+ ≤ y_switch):  u+ = y+                          // viskose Schicht
else:                u+ = A × (y+)^B                  // log-Schicht
```

**Geschlossene Form für τ_w bei gegebenem (u_2, y_2)** an Zelle in 2. Reihe:

```
u_visc = sqrt(2 × ν × |u_2| / y_2)                            // viskos-Annahme
u_log  = (0.0246384 × ν^0.25 × |u_2|^1.75 / y_2^0.25)^0.5     // log-Annahme

u_tau = max(u_visc, u_log)
τ_w = ρ × u_tau²
```

**Spalding/Musker** (höhere Genauigkeit, Newton-Raphson nötig — OpenLB nutzt das für WFB):

```
Musker:  y+ = u+ + exp(-κB) × [exp(κu+) - 1 - κu+ - (κu+)²/2 - (κu+)³/6]
         (κ=0.41, B=5.5)
         Newton-Raphson auf u+ bis Residuum < 1e-6
```

Für FluidX3D-Erst-Implementation: **PowerLaw reicht** (kein Solver nötig, ~5 FLOPs/Zelle).

---

## 3. Approach A — Krüger Moving-Wall mit u_slip (RECOMMENDED)

### Konzept

Statt no-slip (u_solid = 0) an Wand → setze u_solid auf einen "effective slip velocity" der die Wand-Schubspannung emuliert.

**Idee:** Wenn u_2 (Fluid 2 Zellen entfernt) bekannt, τ_w via Werner-Wengle. Daraus folgt Geschwindigkeit u_1 in 1. Fluidzelle:

```
u_1/u_tau = f(y_1+) = f(y_1 × u_tau / ν)
```

In FluidX3D existiert bereits `apply_moving_boundaries` (Krüger Eq. 5.27):
```
f_i^opp(x_wall) = f_i(x_wall) - 6 × w_i × ρ × (c_i · u_solid) / c_s²
```

Wenn wir `lbm.u[wall_cell] = u_slip × tangent_dir` setzen, übernimmt der existierende Moving-Wall Code den Rest. Kein Eingriff in stream_collide nötig.

### Implementation-Plan (analog freeslip_y Pattern)

**Files:**
1. `src/defines.hpp` — neue Compile-Flag `#define WALL_MODEL_WW` (optional, default an)
2. `src/kernel.cpp` — neuer Kernel `apply_wall_model` (läuft VOR stream_collide)
3. `src/lbm.hpp` — Kernel-Member + Methode `enqueue_apply_wall_model()`
4. `src/lbm.cpp` — Kernel-Init + do_time_step Call
5. `src/setup.cpp` — Wand-Zellen markieren mit `TYPE_W` (bit 5, frei)

**Pseudo-Code Kernel:**
```c
kernel void apply_wall_model(global float* u, const global uchar* flags, ...) {
   const uxx n = get_global_id(0);
   if(n>=def_N||is_halo(n)) return;
   if(!(flags[n]&TYPE_W)) return;   // nur Wand-Zellen

   // Finde Normalenrichtung (in unserem Fall: +y für Boden)
   // Lese u an Zelle 2 (y+2 in Lattice-Einheiten)
   uxx j[def_velocity_set];
   neighbors(n, j);
   uxx n_y2 = neighbor in +y direction, distance 2;
   float3 u2 = vload3(n_y2, u);

   // Tangentialkomponente extrahieren (Normal=+y)
   float u_t_mag = sqrt(u2.x*u2.x + u2.z*u2.z);
   float3 t_dir = (float3)(u2.x/u_t_mag, 0.0f, u2.z/u_t_mag);

   // Werner-Wengle PowerLaw
   const float nu = def_nu;
   const float y2 = 2.0f;                            // Lattice-units!
   float u_visc = sqrt(2.0f * nu * u_t_mag / y2);
   float u_log  = pow(0.0246384f * pow(nu,0.25f) * pow(u_t_mag,1.75f) / pow(y2,0.25f), 0.5f);
   float u_tau = max(u_visc, u_log);

   // u+ an y_1 = 1
   float y1plus = u_tau / nu;
   float uplus_1 = (y1plus <= 11.81f) ? y1plus : (8.3f * pow(y1plus, 1.0f/7.0f));
   float u1_mag = uplus_1 * u_tau;

   // u_slip = u_1 (gerichtet)
   vstore3((float3)(u1_mag*t_dir.x, 0.0f, u1_mag*t_dir.z), n, u);
}
```

### Erwartetes Ergebnis

- **Volldomain Fx:** 2219 N → ~1100-1500 N (Reduktion 30-50%)
- **Downforce:** ~0 N → -100 bis -300 N
- **y+:** Modell-Schaltzone bei y_1+ ≈ 11.81; in unserem Fall (Re=9.12M, Δx=10mm) liegt y_1+ ≈ 50-200 → Log-Bereich, PowerLaw aktiv

### Risiken

- **u_2 Stencil-Problem:** "Zelle 2 in Normalenrichtung" funktioniert sauber nur für ortogonale Wände (Boden, Decke, Symm). Für STL-Vehicle-Surface bräuchte man Sub-Grid Wall Distance (signed distance field) → später.
- **Vehicle-Surface:** Approach A funktioniert zunächst nur für Boden (y_min Wall). Vehicle bleibt no-slip Bounce-Back. Das ist OK weil:
  - Boden ist die dominante Wand-Fläche für Wakefluss
  - Vehicle hat Re_lokal viel niedriger (lokale Geometrie)
  - Han 2021 macht das auch so (Wall Model nur an Boden, Vehicle bleibt BB)

---

## 4. Approach B — Wall Function Boundary (Han 2021)

**Reference:** Han, Chen, Sagaut, "A new wall function for high-Reynolds turbulent flow in lattice Boltzmann method", Physics of Fluids 33, 065123 (2021)

### Konzept

Statt Geschwindigkeit zu setzen → modifiziere DDFs direkt an der Wand mit τ_w aus WW.

```
f_i_new(x_wall) = f_i_eq(ρ, u_slip) + f_i_neq(τ_w)
```

mit `f_i_neq` aus Chapman-Enskog Entwicklung:
```
f_i_neq = -(w_i × Δt / c_s²) × (Q_i : (ρ × ∇u_sgs))
```

wobei `∇u_sgs = τ_w / (ρ × ν_eff)` der Wandgradient ist.

### Pro/Contra

✅ Höhere Genauigkeit (Han 2021 zeigt 10-15% bessere Drag-Vorhersage bei Re=10^6)
✅ Funktioniert auch für gekrümmte Wände (mit Distance Field)
❌ Komplexer (DDFs müssen direkt manipuliert werden, nicht nur u_solid)
❌ Eingriff in stream_collide oder neuer Post-Stream-Kernel mit voller DDF-Reconstruction nötig
❌ Q_i Tensor-Berechnung (Memory-Bandbreite!)

**Aufwand:** ~1 Woche.

---

## 5. Approach C — OpenLB regularized BC

**Reference:** OpenLB `wallFunctionBoundaryPostProcessors3D.hh`

### Konzept

Vollständige Regularisierte Bouzidi-BC mit:
- Wall Function (Musker via Newton-Raphson)
- Van Driest Damping für SGS Viskosität
- DDFs komplett neu zusammengesetzt (f_eq + f_neq aus regularisierter Π_neq)

### Pro/Contra

✅ State-of-the-Art Genauigkeit
✅ Funktioniert für beliebige Wand-Orientierungen (mit Normal Field)
❌ ~500 Zeilen Code
❌ Newton-Raphson im Kernel (verlangsamt)
❌ Normal Field muss pro Zelle gespeichert/berechnet werden (Memory!)

**Aufwand:** ~2 Wochen.

---

## 6. Wie machen es andere LBM-Codes?

| Code | Wall Model | Bemerkung |
|------|-----------|-----------|
| **OpenLB** | Musker + Newton-Raphson + Van Driest | wallFunctionBoundaryPostProcessors3D |
| **waLBerla** | WallModel via Boundary Handling (PowerLaw, Spalding) | Generic, in `lbm/boundary/WallFunction.h` |
| **Palabos** | Werner-Wengle via dynamicsHelpers | dynamics/wallFunctionDynamics.h |
| **TCLB** | Tangential momentum BC + WW | Models/d3q19_les |
| **Sailfish CFD** | Bouzidi + WW PowerLaw | wall_models.mako |
| **LUMA** | Spalding via iterativen Solver | regularised_LES_inlet.cpp |
| **Han 2021** | Maßgeschneidertes WFB (siehe Approach B) | Forschungs-Code |
| **STAR-CCM+** (FluidX3D-CFL ≠ STAR aber FYI) | All-y+ Wall Treatment | Smooth blend zwischen viskos/log |

**Beobachtung:** Die meisten Industrie-Codes nutzen **PowerLaw oder Spalding mit Newton-Raphson**. Reines PowerLaw ist Standard für Erst-Implementation, weil keine Iteration nötig.

---

## 7. Empfehlung

**Start mit Approach A** weil:
1. Minimal-invasiv (1 neuer Kernel, analog zu existierendem freeslip_y Pattern)
2. Wiederverwendet existierende `apply_moving_boundaries` Logik
3. Erwartete Drag-Reduktion 30-50% bringt uns nahe an OpenFOAM-Werte
4. 1-2 Tage Aufwand → schnelles Iterieren möglich
5. Wenn nicht ausreichend → Approach B (WFB) als nächster Schritt

**Validierungs-Strategie:**
1. Volldomain (CC6_MODE=1) als Baseline (Fx = 2219 N)
2. Wall Model an Boden aktivieren → Fx, Lift messen
3. Vergleich mit OpenFOAM steady-state RANS (Fx ~400-600 N, Lift -100..-300 N)
4. Sweet-Spot zwischen LES-resolved und RANS-approximated erwartet

---

## 8. Quellen

- Krüger et al., "The Lattice Boltzmann Method - Principles and Practice", Springer 2017, Ch. 5.3.4 (Moving Wall BC)
- Han, Chen, Sagaut, "A new wall function for high-Reynolds turbulent flow in lattice Boltzmann method", Phys Fluids 33, 065123 (2021)
- Werner, Wengle, "Large-eddy simulation of turbulent flow over and around a cube in a plate channel", 8th Symp. Turbulent Shear Flows (1991)
- Musker, "Explicit Expression for the Smooth Wall Velocity Distribution in a Turbulent Boundary Layer", AIAA Journal 17(6), 1979
- Spalding, "A Single Formula for the 'Law of the Wall'", J. Appl. Mech. 28, 1961
- OpenLB v1.7 Source: `src/functors/lattice/wallFunction3D.hh`, `src/dynamics/wallFunctionBoundaryPostProcessors3D.hh`
- waLBerla Source: `src/lbm/boundary/WallFunction.h`
- Palabos Source: `src/complexDynamics/wallFunctionDynamics.h`
