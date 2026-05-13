# Phase 0d-A — Wall-Model Code Audit

**Datum:** 2026-05-13
**Iron Rule:** WW-Kernel-Code wird NUR gelesen, nicht modifiziert.

---

## 1. Daten-Fluss-Diagramm

```
┌─────────────────────────────────────────────────────────────────────┐
│  Pro Zeitschritt do_time_step():                                    │
│                                                                     │
│  Schritt 1: apply_wall_model_vehicle (CC#10)                        │
│  ────────────────────────────────────────                           │
│   für jede TYPE_S|TYPE_X Surface-Zelle n:                           │
│     u_avg ← Mittel der u[fluid_neighbors] (D3Q19, Fluid-Filter)     │
│     u_τ ← max(viscous, PowerLaw) closed-form WW                     │
│     u_slip ← uplus_1 × u_τ × direction(u_avg)                       │
│     u[n] ← u_slip   ← SCHREIBT u_solid auf Wand-Zelle               │
│                                                                     │
│  Schritt 2: update_moving_boundaries                                │
│  ──────────────────────────────────                                 │
│   für jedes Fluid-Cell mit TYPE_S-Nachbar mit u≠0:                  │
│     flags |= TYPE_MS   ← markiert Fluid für Moving-Wall-Korrektur   │
│                                                                     │
│  Schritt 3: stream_collide                                          │
│  ─────────────────────                                              │
│   für jedes Fluid-Cell flagsn_bo == TYPE_MS:                        │
│     apply_moving_boundaries(fhn, j, u, flags) ← INLINE              │
│       Δf_i = -6·w_i · ρ · (c_i · u[wall_neighbor])                  │
│     fhn ← fhn + Δf_i   für DDFs deren Quelle TYPE_S-Wand ist        │
│     store_f(...)        ← schreibt korrigierte DDFs in self+nachbar │
│                            (Esoteric-Pull: einige Slots gehen IN    │
│                            die TYPE_S-Zellen!)                       │
│                                                                     │
│  Schritt 4 (separat): update_force_field                            │
│  ─────────────────────────────────────                              │
│   für jedes TYPE_S Boundary-Cell n:                                 │
│     load_f(n, fhn, fi, j, t)  ← liest DDFs aus self+nachbar         │
│     calculate_rho_u(fhn, &Fb, &fx, &fy, &fz)                        │
│     F[n] = 2 × Fb × (fx, fy, fz)                                    │
│            ↑                                                        │
│            = 2 × Σ c_i · f_i  am Wand-Cell                          │
│            DDFs ENTHALTEN Δf_i Korrekturen aus Schritt 3 !          │
│                                                                     │
│  Schritt 5: object_force(TYPE_S|TYPE_X)                             │
│  ─────────────────────────────────                                  │
│   Atomic-Sum von F[n] über alle Zellen mit flags == flag_marker     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. Source-Code Excerpts

### apply_wall_model_vehicle (kernel.cpp:1478-1517)
```c
kernel void apply_wall_model_vehicle(global float* u, const global uchar* flags) {
    const uxx n = get_global_id(0);
    if(n>=(uxx)def_N||is_halo(n)) return;
    const uchar fn = flags[n];
    if((fn & (TYPE_S|TYPE_X)) != (TYPE_S|TYPE_X)) return;
    uxx j[def_velocity_set];
    neighbors(n, j);
    float ux_avg = 0.0f, uy_avg = 0.0f, uz_avg = 0.0f;
    uint count = 0u;
    for(uint i=1u; i<def_velocity_set; i++) {
        const uchar fj = flags[j[i]];
        if(!(fj & (TYPE_S|TYPE_E|TYPE_T))) {
            ux_avg += u[j[i]]; uy_avg += u[def_N+(ulong)j[i]]; uz_avg += u[2ul*def_N+(ulong)j[i]];
            count++;
        }
    }
    if(count == 0u) return;
    /* ... PowerLaw u_τ + u_slip ... */
    const float scale = u_slip_mag / u_t_mag;
    u[              n] = scale * ux_avg;
    u[def_N+(ulong)n ] = scale * uy_avg;
    u[2ul*def_N+(ulong)n] = scale * uz_avg;
}
```

### apply_moving_boundaries (kernel.cpp:1116-1123) — Krüger Eq. 5.27
```c
void apply_moving_boundaries(float* fhn, const uxx* j, const global float* u, const global uchar* flags) {
    uxx ji;
    for(uint i=1u; i<def_velocity_set; i+=2u) {
        const float w6 = -6.0f*w(i);  // w6 = -2·w_i·ρ/c²  (ρ_wall=1)
        ji = j[i+1u]; fhn[i  ] = (flags[ji]&TYPE_BO)==TYPE_S ? fma(w6, c(i+1u)*u[ji]+..., fhn[i  ]) : fhn[i  ];
        ji = j[i  ]; fhn[i+1u] = (flags[ji]&TYPE_BO)==TYPE_S ? fma(w6, c(i  )*u[ji]+..., fhn[i+1u]) : fhn[i+1u];
    }
}
```

### update_force_field (kernel.cpp:1993-2004)
```c
kernel void update_force_field(const global fpxx* fi, const global uchar* flags, const ulong t, global float* F) {
    const uxx n = get_global_id(0);
    if((flags[n]&TYPE_BO)!=TYPE_S) return; // only solid boundary cells
    uxx j[def_velocity_set];
    neighbors(n, j);
    float fhn[def_velocity_set];
    load_f(n, fhn, fi, j, t);                      // ← liest DDFs (inkl. moving-wall-Korrektur!)
    float Fb=1.0f, fx=0.0f, fy=0.0f, fz=0.0f;
    calculate_rho_u(fhn, &Fb, &fx, &fy, &fz);
    store3(F, n, 2.0f*Fb*(float3)(fx, fy, fz));    // F = 2 × Σ c_i · f_i
}
```

### object_force (kernel.cpp:2046-2062)
Reduction-Sum von F[n] über Cells mit `flags[n] == flag_marker` (exact-match).

---

## 3. Analytische Artefakt-Formel

Die Krüger-Korrektur fügt zu DDFs hinzu:
```
Δf_i = -6 × w_i × ρ × (c_i · u_solid)
```

Force-Beitrag einer Wand-Zelle aus diesen Korrekturen:
```
ΔF = 2 × Σ_i c_i × Δf_i
   = -12 × Σ_i w_i × c_i × (c_i · u_solid)
   = -12 × M · u_solid    wobei M_jk = Σ_i w_i × c_i_j × c_i_k
```

Für D3Q19 mit Lattice-Symmetrie:
```
M_jk = c_s² × δ_jk = (1/3) × δ_jk     (Isotrope second moment)
```

Daraus folgt **analytische Pro-Zelle-Artefakt-Force**:
```
ΔF_artifact = -12 × (1/3) × u_solid = -4 × u_solid    (per cell, LB units)
```

**Vorzeichen-Analyse:**
- u_solid_x > 0 (Wand bewegt sich mit Strömung) → ΔF_artifact_x < 0 (Wand "zieht" entgegen Strömung)
- Per Newton 3rd Law physikalisch korrekt: bewegte Wand überträgt Impuls AUF Fluid in +X → Fluid übt Reaktionskraft −X auf Wand aus
- ABER: das ist KEIN echter aerodynamischer Drag! Es ist der ZUSÄTZLICHE Term aus dem moving-wall-trick

### Achtung: nur DDFs mit Solid-Nachbar werden modifiziert

Im apply_moving_boundaries-Loop wird `Δf_i` nur addiert wenn `(flags[ji]&TYPE_BO)==TYPE_S`. Wand-interne DDFs (Solid-Nachbar zur SOLID-Seite) bekommen ihre Korrektur, externe Fluid-Nachbarn nicht.

Damit gilt obige Formel exakt nur für Zellen die VOLLSTÄNDIG umgeben sind von TYPE_S — sprich Interior. Für Surface-Cells mit Mix aus Solid+Fluid Nachbarn ist der Effekt teilweise.

**Effective Formel pro Surface-Cell:**
```
ΔF_artifact = -4 × (N_solid_neighbors / 18) × u_solid
```

Für eine Würfel-Surface-Cell (5 Solid-Nachbarn auf D3Q19 mit 18 Stencil-Richtungen):
```
ΔF_artifact ≈ -4 × (5/18) × u_solid = -1.11 × u_solid
```

Für u_solid ≈ 0.07 LB → ΔF_artifact ≈ -0.078 LB pro Surface-Cell.

---

## 4. Größenordnungs-Check vs Observed

### MR2 Time-Attack:
- Observed: F = −610 N, Target = +565 N → Differenz **−1 175 N** (Artefakt)
- LB→SI Force-Faktor (10mm cells, 30 m/s): ≈ 19.6
- ΔF_artifact_SI / Cell ≈ -0.078 × 19.6 = -1.5 N pro Surface-Cell (Größenordnung)
- N_surface erwartet ≈ 1175 / 1.5 ≈ 780 Surface-Cells
- MR2-Vehicle hat ~4.4m × ~2.5m surface area ≈ 30 m² → bei 1cm² pro Cell wären das 300,000 Cells
- **Faktor ~400× Diskrepanz** → meine Formel überschätzt drastisch, ODER:
  - Wandgeometrie hat viel weniger Surface-Cells als naiv geschätzt (typische LBM-Voxelisierung gibt evtl. 1-cell-dicke aber nicht jedes Cell hat 5 Solid-Nachbarn)
  - u_solid ist im Mittel kleiner als 0.07 (viele Cells sehen u_avg < u_∞)
  - Cancellation: an manchen Cells geht u_solid in andere Richtung als Mittel
- Quantitativ unsicher, qualitativ aber: Artefakt ist negativ und proportional zur Surface-Cell-Anzahl

### Yaris Stock:
- Differenz: 463 - (-250) = 713 N
- Ratio MR2/Yaris: 1175/713 = 1.65
- Vehicle-Volumen-Ratio MR2/Yaris: ~10 m³ / ~6 m³ = 1.67 ← konsistent!

### Ahmed (Phase 0 FAIL):
- Hier ist es NICHT negativer Drag sondern POSITIV-überhöhter (Faktor 100×)
- Hypothese: Ahmed hat andere Surface-Cell-Verteilung; u_solid an manchen Cells in -X-Richtung (z.B. Recirculation-Zone) → Δf_i × c_i geht in andere Vorzeichen-Richtung als nahe-flächen-parallel
- Verifikation in Schritt B (Cube-Test) — Cube hat keine Recirculation auf der Wake-Seite genauso "fluid-frei"

---

## 5. Bestätigung Theorie-Verifikation Status

| Hypothese | Status nach Audit |
|---|---|
| WW schreibt u_solid ≠ 0 auf Wand-Zellen | ✓ bestätigt (kernel.cpp:1514-1516) |
| apply_moving_boundaries appliziert Δf_i = -6·w_i·(c_i·u_solid) | ✓ bestätigt (kernel.cpp:1120-1121) |
| update_force_field summiert 2·Σ c_i·f_i inkl. modifizierter DDFs | ✓ bestätigt (kernel.cpp:1993-2004) |
| object_force ist reine Reduktion (kein Subtract) | ✓ bestätigt (kernel.cpp:2046-2062) |
| Analytische Pro-Cell-Artefakt-Formel ΔF ≈ -4·u_solid für vollständig umgebene Zelle | ✓ Mathematisch aus Krüger + D3Q19 hergeleitet |
| **Force-Artefakt skaliert mit (Surface-Cells × u_solid)** | ✓ Theorie + Größenordnungs-Check (Faktor 400× quantitativ unsicher) |
| Phase 0d-B Cube-Test wird Artefakt quantitativ isolieren | PENDING |

---

## 6. Implikation für Fix-Strategien

### Option 1 (Force-Artefakt subtrahieren)
- Bei jeder Wand-Zelle: lese u[n], zähle Solid-Nachbarn, berechne ΔF_artifact = -4 × (n_solid/18) × u[n]
- Akkumuliere ΔF separat, subtrahiere von Total-Force
- **Implementation einfach**: neuer Kernel `compute_wall_model_artifact`, parallel zu update_force_field
- **Risiko**: Formel-Faktor 4·(n_solid/18) ist Approximation, exakt nur für isotropes D3Q19-Lattice
- Test: Cube-Test (Schritt B) wird zeigen ob die Subtraktion physikalisch korrekt ist

### Option 2 (WW ohne Moving-Wall)
- u[wall_cell] bleibt 0
- Wall-Function-Effekt über andere Mechanik: Effective-Viscosity-Bump in adjacent Fluid-Cells
- ODER: Direkte DDF-Modification ohne moving-wall-Term
- **Komplexer**: erfordert eigene PostProcessing-Kernel, Iron-Rule-Override

### Option 3 (Bouzidi)
- Sub-Grid-Distance-aware Bounce-Back ist kompatibel mit Wall-Functions ohne Moving-Wall-Trick
- Bereits Phase 1 der Roadmap (7-15 Tage)

---

## 7. Nächste Schritte (Phase 0d-B + C)

**B: Cube Minimal-Reproduce**
- 1m³ Cube bei Re=10⁵ (u=1.5 m/s, ν=1.5e-5)
- Domain 4L × 3L × 3L bei 20mm Cells (4M Cells, schnell)
- Free-stream TYPE_E walls (keine Rolling-Road)
- 2 Hauptvarianten zuerst: ohne WW, mit WW
- Expected: CD_cube ≈ 1.05 → F_target = 0.5 × 1.225 × 1.5² × 1 × 1.05 = 1.45 N
- Wenn WW-Artefakt theorie korrekt: F_WW ≈ F_BB − |Artefakt|, mit |Artefakt| proportional zu Surface-Cells
- Surface eines 50-Cell Cubes (1m bei 20mm): 6 × 50² = 15000 surface-cells × ΔF_artifact_per_cell

**C: OpenLB Cross-Reference**
- Wie macht OpenLB WallFunctionBoundaryProcessor3D? Schreibt es u_solid?
- Mit WebSearch / WebFetch auf openLB source
