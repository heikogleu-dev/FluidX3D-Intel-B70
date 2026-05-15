# Path IV — Pi-Tensor f_neq Reconstruction (Han 2021) Design

**Datum:** 2026-05-15
**Status:** Design only (Implementation pending erfolglosen Path I+III)
**Ziel:** Mathematisch sauberer Wall Model der Krüger Moving-Wall Hack komplett ersetzt.
**Aufwand:** 1-2 Wochen
**VRAM-Overhead:** 0 GB (DDFs werden direkt modifiziert)

## Motivation

Die bisherige WW-Implementierung (CC#10) verwendet Krüger Moving-Wall als Transfer-Mechanismus:
```
u[wall_cell] := u_slip × ê_u_avg
→ apply_moving_boundaries: f_i_new = f_i - 6 w_i ρ (c_i · u_wall)
```

**Problem:** Krüger Moving-Wall wurde für **echte moving walls** (rolling road, rotating sphere) designt. Es transferiert `u_wall` als Wand-Geschwindigkeit ins Fluid via DDF-Forcing. Für Wand-Modell-Anwendung (wo `u_wall` eine künstliche slip-velocity repräsentiert) sind die transfer-coefficients (−6 in D3Q19) nicht geeignet — produziert die Three-Attractor-Bifurcation aus Finding 36.

## Han 2021 Wall Function Boundary (WFB) Pattern

**Reference:** Han, Chen, Sagaut, "A new wall function for high-Reynolds turbulent flow in lattice Boltzmann method", Physics of Fluids **33**, 065123 (2021).

### Kernidee

Statt `u_wall` zu setzen und Krüger zu nutzen, **modifiziere DDFs direkt** an wall-adjacent fluid cells (nicht an Wand-Cells!) so dass sie der Wall-Function-Lösung entsprechen:

```
f_i(wall_adjacent_fluid) = f_i_eq(ρ_local, u_slip) + f_i_neq(τ_wall)
```

Wobei:
- `f_i_eq` = standard equilibrium mit slip velocity `u_slip` aus law-of-the-wall
- `f_i_neq` = non-equilibrium part rekonstruiert aus wall shear stress `τ_w` via Chapman-Enskog

### Mathematische Formulierung

**Schritt 1: Wall shear stress via WW PowerLaw (geschlossene Form)**

```
y_2+ = u_2 × y_2 × ρ / μ  (y_2 = 1 lu für half-way BB)
u_tau aus WW PowerLaw:
  u_visc² = 2 × ν × u_2 / y_2
  u_log²  = 0.0246384 × ν^0.25 × u_2^1.75 / y_2^0.25
  u_tau   = sqrt(max(u_visc², u_log²))
τ_w = ρ × u_tau²
```

**Schritt 2: Slip velocity u_1 bei y_1 = 0.5 lu (= wall-adjacent fluid cell-center)**

```
y_1+ = u_tau × y_1 / ν
u+(y_1+) = (y_1+ ≤ 11.81) ? y_1+ : 8.3 × y_1+^(1/7)
u_slip = u_tau × u+(y_1+) × t̂  (in tangentialer Richtung)
```

**Schritt 3: Pi-Tensor (Wall Shear Stress Tensor) rekonstruiert f_neq**

Chapman-Enskog: f_neq ∝ Q_i : (∇u)

Wand-Approximation: Wall-Schubspannungs-Tensor S_wall = (1/μ) × τ_w × (n̂ × t̂_symmetric)

```
Pi_wall = -2 × ν × ρ × S_wall  (Wall stress tensor)
```

Mit n̂ = Wand-Normal, t̂ = Wand-Tangente in Strömungsrichtung.

```
f_i_neq = -w_i × (1.0 + def_tau_inv) × Q_i : Pi_wall / c_s²
```

Wobei `Q_i = c_i ⊗ c_i - c_s² × I` (3D outer product minus isotropic).

**Schritt 4: Total DDF**

```
f_i(wall_adjacent) = f_eq(ρ, u_slip) + f_neq
```

### Hauptunterschiede zu CC#10 Krüger

| Aspekt | CC#10 Krüger | Han 2021 Pi-Tensor |
|---|---|---|
| Wo wird modifiziert | u[wall_cell] | f_i an wall-adjacent fluid cells |
| Transfer-Mechanismus | apply_moving_boundaries (Krüger −6) | direkt DDF-Manipulation (kein Krüger) |
| Mathematische Basis | empirisch (Krüger Eq. 5.27) | Chapman-Enskog Pi-Tensor |
| Self-Reference | u_slip = f(u_avg) — iterative bifurcation | u_slip = f(u_2) one-shot at y_2 (clean) |
| Failure Modes | F1+F2+F3 (Three-Attractor, neg drag, lock-in) | Nur F4 (thin-feature averaging, falls vorhanden) |

## Implementation-Plan

### Phase A: Wall-Normal Computation (Pre-requisite)

**File:** `src/lbm.cpp` (post-voxelize step)

Für jede TYPE_S|TYPE_X Vehicle-Cell, berechne dominante outward direction:
```cpp
// Pseudo-code post-voxelize_mesh_on_device:
parallel_for(N, [&](ulong n) {
    if(flags[n] & TYPE_X) {
        int3 n_out = (0,0,0);
        for(uint i=1u; i<def_velocity_set; i++) {
            if(!(flags[j[i]] & TYPE_S)) {  // fluid neighbor
                n_out += int3(c(i), c(def_velocity_set+i), c(2u*def_velocity_set+i));
            }
        }
        // Store dominant direction sign per axis in spare flag bits, or in separate buffer
        wall_normal[n] = normalize(float3(n_out));
    }
});
```

**VRAM:** float3 wall_normal[N] = 12 bytes/cell × N. Far 160M = 1.8 GB, Near 242M = 2.7 GB → 4.5 GB total.

**Sparse alternative:** Nur für TYPE_X cells (~0.1% von N) → < 5 MB total. Hash-map oder lookup-array.

### Phase B: New Kernel `apply_wall_function_pi_tensor`

**File:** `src/kernel.cpp` (replaces `apply_wall_model_vehicle`)

```c
kernel void apply_wall_function_pi_tensor(global fpxx* fi, const global float* u, 
                                          const global float* rho, const global uchar* flags, 
                                          const global float3* wall_normal) {
    const uxx n = get_global_id(0);
    if(n >= def_N || is_halo(n)) return;
    const uchar fn = flags[n];
    if((fn & (TYPE_S|TYPE_X)) != (TYPE_S|TYPE_X)) return;  // vehicle cells only
    
    const float3 n_out = wall_normal[n];
    if(length(n_out) < 0.5f) return;  // skip cells with unclear normal
    
    // Find fluid neighbor in +n_out direction (y_2 cell)
    uxx j[def_velocity_set];
    neighbors(n, j);
    int i_y2 = find_closest_neighbor(j, flags, n_out);
    if(i_y2 < 0) return;  // no fluid neighbor in outward direction
    
    const ulong nj = j[i_y2];
    const float3 u_2 = (float3)(u[nj], u[def_N+nj], u[2*def_N+nj]);
    
    // Tangential component (project u_2 perpendicular to n_out)
    const float u_n = dot(u_2, n_out);
    const float3 u_t = u_2 - u_n * n_out;
    const float u_t_mag = length(u_t);
    if(u_t_mag < 1e-6f) return;
    const float3 t_hat = u_t / u_t_mag;
    
    // Werner-Wengle PowerLaw
    const float nu = def_nu;
    const float y_2 = 1.0f;  // 1 lu
    const float u_visc2 = 2.0f * nu * u_t_mag / y_2;
    const float u_log2  = 0.0246384f * pow(nu, 0.25f) * pow(u_t_mag, 1.75f) / pow(y_2, 0.25f);
    const float u_tau   = sqrt(max(u_visc2, u_log2));
    
    // u_slip at y_1 = 0.5 lu (half-way BB plane)
    const float y_1 = 0.5f;
    const float y_1_plus = u_tau * y_1 / nu;
    const float u_plus_1 = (y_1_plus <= 11.81f) ? y_1_plus : 8.3f * pow(y_1_plus, 1.0f/7.0f);
    const float u_slip_mag = u_tau * u_plus_1;
    const float3 u_slip = u_slip_mag * t_hat;
    
    // Pi-Tensor for wall stress
    const float rho_n = rho[n];
    const float tau_w = rho_n * u_tau * u_tau;
    // Wall stress tensor Pi: symmetric, t̂⊗n̂ + n̂⊗t̂ structure
    float Pi_xx = -2.0f * nu * rho_n * (t_hat.x * n_out.x);  // ... etc 9 components
    // ... compute full Pi tensor ...
    
    // Modify f_i at adjacent fluid cell (NOT at wall cell)
    // Loop over directions pointing INTO fluid (away from wall)
    for(uint i=1u; i<def_velocity_set; i++) {
        const float3 c_i = (float3)(c(i), c(def_velocity_set+i), c(2*def_velocity_set+i));
        if(dot(c_i, n_out) <= 0) continue;  // skip directions toward wall
        
        const ulong nj_i = j[i];
        if(flags[nj_i] & TYPE_S) continue;  // skip if neighbor is solid (shouldn't happen for outward direction)
        
        // Compute f_eq at neighbor with u_slip
        const float w_i = weights[i];
        const float feq_i = w_i * rho_n * (1.0f + 3.0f*dot(c_i, u_slip) + ...);  // standard f_eq
        
        // Q_i tensor contraction
        const float Q_dot_Pi = c_i.x*c_i.x*Pi_xx + c_i.y*c_i.y*Pi_yy + ...;  // 9 components
        
        // f_neq contribution
        const float fneq_i = -w_i * Q_dot_Pi / (2.0f * c_s2 * c_s2);  // Chapman-Enskog
        
        // Write to f_i at adjacent fluid cell
        store_f(nj_i, feq_i + fneq_i, fi, j_at_nj, t);
    }
}
```

**Aufwand:** ~1 Woche für sauberen, validierten Kernel.

### Phase C: Validation Tier-1 (Poiseuille channel flow)

Bevor MR2-Test:
1. Channel flow at Re=10^5 mit Pi-Tensor wall model
2. Vergleich u(y)-Profile gegen analytic law-of-the-wall
3. Vergleich τ_w_predicted gegen τ_w_measured

**Erfolgskriterium:** u(y) folgt log law in 30 < y+ < 300, L2 error < 5%.

### Phase D: Validation Tier-2 (Sphere at Re=10^5)

1. Sphere geometry mit Pi-Tensor wall model
2. Vergleich CD mit experimentellen Daten (Schlichting)
3. **Erfolgskriterium:** CD im Bereich 0.4-0.5 (experimentelle Streuung ±10%)

### Phase E: MR2 Application

Erst nach Tier-1+2 PASS:
1. Aktiviere Pi-Tensor auf MR2 Multi-Res Mode 3 + TYPE_S
2. Vergleich Fx/Fz mit Time-Attack targets
3. **Erfolgskriterium:** Fx im 400-800 N range, Fz_near < −800 N

## Risiken & Open Questions

### R1: Wall-Normal für komplexe STL

Vehicle-Cells mit "ambiguer" Normal (z.B. an scharfen Kanten wo > 1 dominante outward-direction) brauchen Spezialbehandlung:
- Cell mit `length(n_out) < threshold` → skip wall function, fall back to standard BB
- Oder: per-direction Wall-Function-Anwendung (komplex aber genau)

### R2: Half-Way BB Geometrie

Standard FluidX3D BB ist half-way (Wand zwischen Solid und Fluid Cell). Pi-Tensor wall function angenommen Wand AT solid-cell-center → off-by-half-cell error. Korrektur durch:
- y_1 = 0.5 lu (was wir verwenden, korrekt für half-way BB)
- u_slip bei y_1 anstatt bei wall

### R3: Coupling-Plane Interaktion

In Mode 3 Multi-Res: wall-modifizierte DDFs an Near-Vehicle-Surface werden via Mode 3 forward coupling auf Far übertragen. Die Pi-Tensor f_neq Beiträge sollten beim Resampling erhalten bleiben (linear in fields). Validation needed.

### R4: Performance-Impact

Neuer Kernel läuft pro Schritt für jede TYPE_S|TYPE_X Cell. ~0.1% aller Cells. Performance-Impact erwartet <5% pro Schritt.

## Comparison Path-Übersicht

| Path | VRAM | Aufwand | Erwartete Erfolgswahrscheinlichkeit |
|---|---:|---|:-:|
| I (WW + Mode 1 test) | 0 GB | 30 min | Diagnostic only |
| II (Wall-Normal Projection) | <1 GB (sparse) | 1 Tag | 40% Fix |
| III (Reduce Safety Cap) | 0 GB | 1 Std | 25% Fix |
| **IV (Pi-Tensor Han 2021)** | **0 GB** | **1-2 Wochen** | **80% Fix** |
| V (Accept Pure-BB baseline) | 0 GB | 0 | n/a (use as-is) |

Path IV ist der einzige der **mathematisch saubere Wall Function** für komplexe Geometrie liefert. Höchster Aufwand aber höchste Erfolgswahrscheinlichkeit.

## Empfehlung

1. **Heute:** Path I läuft (30 min Diagnose). Path III ready in parallel.
2. **Falls Path I/III nicht überzeugend:** Path II (Wall-Normal Projection) für Quick-Fix-Versuch.
3. **Long-term (nächste Session):** Path IV implementation start, Phase A Wall-Normal Computation als erstes (lokal, ohne Pi-Tensor).
4. **Backup:** Path V immer verfügbar (Mode 3 + TYPE_S baseline, Fx 1477 N stabil).

## References

- Han, Chen, Sagaut, "A new wall function for high-Reynolds turbulent flow in lattice Boltzmann method", Phys. Fluids 33, 065123 (2021). DOI: 10.1063/5.0048396
- Krüger et al., "The Lattice Boltzmann Method: Principles and Practice", Springer 2016, Ch. 5.3.4 (Moving Wall BC)
- OpenLB Documentation, `wallFunctionBoundaryPostProcessors3D.h` (Spalding/Musker implementation reference)
- Werner & Wengle, "Large-eddy simulation of turbulent flow over and around a cube in a plate channel", 8th Symposium on Turbulent Shear Flows 1991
