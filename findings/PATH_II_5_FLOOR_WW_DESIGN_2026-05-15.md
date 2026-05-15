# Path II.5 — Floor-Only Wall Model Design

**Datum:** 2026-05-15
**Status:** Design + Code-Plan, Implementation pending α=0.20 Reference-Test Resultat
**Aufwand:** ~3-4 Stunden
**VRAM-Overhead:** 0 GB
**Triggered by:** User-Diagnose 2026-05-15 nach Mode 3 + TYPE_S ParaView Sichtung:
> "vor dem Fahrzeug am Moving Ground die Luft bereits bei unter 10m/s ist und somit zu langsam auf das Fahrzeug trifft, obwohl wir 30m/s simulieren."

## Diagnose: Pure-BB Floor BL ist zu dick

Bei TYPE_S Moving Wall + Pure-BB:
- Floor u_x = 30 m/s (rolling road)
- Fluid 1 cell darüber (15mm in Far): u_x bestimmt durch Stokes-Layer + Bounce-Back
- Bei Re_floor = 30 m/s × 13.5m / 1.48e-5 = 2.7×10⁷ → fully turbulent
- y_1+ ≈ 2700 (deep log law) → Pure-BB überresolviert die viskose Schicht massiv
- Effektive BL-Dicke ~30 cells × 15mm = 45 cm → bremst freestream noch 30+ cm über Boden
- User-Beobachtung: Luft vor Fahrzeug auf 10 m/s statt 30 m/s

**Echte turbulente BL bei diesen Bedingungen:**
- δ ≈ 0.37 × L × Re^(-1/5) = 0.37 × 13.5 × (2.7e7)^(-0.2) = 0.06 m = 6 cm

Faktor 7-8× Überprediktion durch Pure-BB.

## Lösung: Han 2021 Wall Function am Floor anwenden

Han 2021's WFB war von Anfang an **für flache Boden-Wände designt**, nicht für Vehicle-Surface. Genau unser Anwendungsfall.

**Warum am Floor garantiert OK funktioniert (im Gegensatz zu Vehicle):**

| Failure Mode | Vehicle (MR2 STL) | Floor (z=0) |
|---|---|---|
| F1: Self-reference / Three-Attractor | komplex (varied u_avg) | flat → uniform u_avg → kein Bifurcation |
| F2: Negative-drag (Krüger transfer) | drag direction conflict | floor is friction-source only |
| F3: Multi-Res transient | unklar | floor identisch in Far+Near |
| F4: Thin-feature averaging | wing/splitter sharp | floor flat → kein Issue |

Floor = der EINE Fall wo WW physikalisch + numerisch sauber ist.

## Implementation-Plan (3-4 Stunden)

### Schritt 1 — Neuer Kernel (kernel.cpp)

Statt das existing `apply_wall_model_vehicle` zu erweitern: separater Kernel `apply_wall_model_floor`, weil:
- Filter-Logic anders (TYPE_S without TYPE_X, only z==0)
- Wall normal trivially +z
- Vergleich zu Vehicle-Variante als Reference behalten

```c
)+"#ifdef WALL_MODEL_FLOOR"+R(
)+R(kernel void apply_wall_model_floor(global float* u, const global uchar* flags) {
    const uxx n = get_global_id(0);
    if(n >= def_N || is_halo(n)) return;
    const uchar fn = flags[n];
    // Only TYPE_S floor cells (no TYPE_X = not vehicle)
    if((fn & (TYPE_S|TYPE_X)) != TYPE_S) return;
    const uint3 xyz = coordinates(n);
    if(xyz.z != 0u) return;  // only z=0 floor
    
    // Get +z neighbor (1 cell up) as u_2
    const uxx j_up = n + (uxx)(def_Nx * def_Ny);  // +1 in z
    if(j_up >= def_N) return;
    if((flags[j_up] & (TYPE_S|TYPE_E|TYPE_T)) != 0) return;  // skip if not fluid
    
    // u_2 at y_2 = 1 lu above floor
    const float ux_2 = u[j_up];
    const float uy_2 = u[def_N + (ulong)j_up];
    const float uz_2 = u[2ul*def_N + (ulong)j_up];
    
    // Tangential component (subtract +z component)
    const float u_t_x = ux_2;  // tangential X
    const float u_t_y = uy_2;  // tangential Y
    // uz_2 is normal component — discarded for wall model
    const float u_t_mag = sqrt(u_t_x*u_t_x + u_t_y*u_t_y);
    if(u_t_mag < 1e-6f) return;
    
    // Reference frame: rolling road moves at u_road in +x direction at lbm_u
    // Use RELATIVE velocity for wall model: u_rel = u_2 - u_road
    // u_road = (lbm_u, 0, 0)
    const float u_rel_x = u_t_x - DEF_LBM_U;  // lbm_u as compile-time const
    const float u_rel_mag = sqrt(u_rel_x*u_rel_x + u_t_y*u_t_y);
    if(u_rel_mag < 1e-6f) return;  // no slip needed when fluid matches road speed
    
    const float nu = def_nu;
    const float y_2 = 1.0f;  // 1 lu above floor
    // Werner-Wengle PowerLaw on RELATIVE velocity
    const float u_visc2 = 2.0f * nu * u_rel_mag / y_2;
    const float u_log2  = 0.0246384f * pow(nu, 0.25f) * pow(u_rel_mag, 1.75f) / pow(y_2, 0.25f);
    const float u_tau   = sqrt(max(u_visc2, u_log2));
    
    // Slip at y_1 = 0.5 lu above floor
    const float y_1_plus = 0.5f * u_tau / nu;
    const float u_plus_1 = (y_1_plus <= 11.81f) ? y_1_plus : 8.3f * pow(y_1_plus, 1.0f/7.0f);
    const float u_slip_rel = u_tau * u_plus_1;
    
    // Cap: u_slip_rel must not exceed u_rel_mag (else fluid would accelerate beyond freestream)
    const float scale = (u_slip_rel < u_rel_mag) ? u_slip_rel / u_rel_mag : 1.0f;
    
    // u at floor = u_road + scale × u_rel (slip in fluid frame, added to road frame)
    u[n              ] = DEF_LBM_U + scale * u_rel_x;
    u[def_N + (ulong)n] = scale * u_t_y;
    // u_z stays whatever (typically 0 for floor)
}
)+"#endif"+R( // WALL_MODEL_FLOOR
```

Wichtig: **Relative Geschwindigkeit** im Frame der bewegten Straße. Wall slip wird nur auf das Δu = (u_fluid - u_road) angewendet.

### Schritt 2 — Build-System (lbm.hpp, lbm.cpp, defines.hpp)

```cpp
// defines.hpp:
#define WALL_MODEL_FLOOR // 2026-05-15: Han-2021-Pattern wall function applied only to flat moving-road floor (z=0). Vehicle bleibt Pure-BB (komplexe STL → F4-Risiko).

// lbm.hpp LBM_Domain:
Kernel kernel_apply_wall_model_floor;
void enqueue_apply_wall_model_floor();

// lbm.cpp Init:
kernel_apply_wall_model_floor = Kernel(device, N, "apply_wall_model_floor", u, flags);

// lbm.cpp do_time_step (additional to WW_VEHICLE block):
#ifdef WALL_MODEL_FLOOR
    for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_apply_wall_model_floor();
    // update_moving_boundaries already called via TYPE_S floor with u_x update
#endif
```

### Schritt 3 — Test Tier-1 (~30 min)

Run Mode 3 + α (best from α=0.20 test) + WALL_MODEL_FLOOR for ~50 chunks:
- **Erfolg-Kriterium 1**: Far freestream at y=10cm above floor stays > 25 m/s (vs current ~10 m/s)
- **Erfolg-Kriterium 2**: Forces stabil, KEIN Drift wie WW-Vehicle
- **Erfolg-Kriterium 3**: BL Profil im ParaView slice oben am Boden plausibel dünn

### Schritt 4 — Validation Tier-2

Compare drag values:
- Mode 3 + TYPE_S Pure-BB Baseline: Fx_far=1477N, Fz_near=-556N
- Mode 3 + Floor-WW: Expected Fx leicht niedriger (besseres Auflösung floor effekt), Fz_near möglicherweise stärker

## Risiken & Mitigation

### R1: Relative Velocity Reference Frame
Wall model auf rolling road braucht u_rel = u - u_road. Falls falsch implementiert → ähnliche Pathology wie Vehicle-WW.
**Mitigation:** Tier-1 mit kurzem Run (5-10 chunks) → verify forces stabil bevor full run.

### R2: TYPE_X Wheel-Contact-Patches
Wheel contact am z=0 sind TYPE_S|TYPE_X with u_x=lbm_u. Floor-WW kernel filters `(fn & TYPE_X) == 0` → skip → korrekt.

### R3: Coupling-Plane Wall-Model Interaction
Far und Near Floor sind beide bei z=0. Floor-WW wirkt auf beide unabhängig. Coupling planes sind ABOVE z=0 (z=1..112 für Far, z=1..338 für Near). Kein Konflikt mit Floor-WW.

## Estimated Result

**Bei korrekter Implementation:**
- BL Floor: 30 cells dick → 5-8 cells dick (= ~10 cm physikalisch, realistisch)
- Vor-Fahrzeug Strömung: ~28-30 m/s (heute 10 m/s)
- Drag Fx_far: 1477 N → 1100-1300 N (~10-20% Reduktion durch korrektes Floor)
- Fz_near: -556 N → -700 bis -900 N (etwas mehr Downforce durch besser durchströmtes Ground-Effect)

Wenn Floor-WW funktioniert, **könnten wir es später auf Vehicle erweitern** mit Wall-Normal-Projection (Path II) für noch besseren Drag-Fit. Aber Floor-Only ist der sichere erste Schritt.

## See Also

- [WALL_MODEL_DEEP_ANALYSIS_2026-05-15](findings/WALL_MODEL_DEEP_ANALYSIS_2026-05-15.md) — F1-F4 failure modes
- [PATH_IV_PI_TENSOR_DESIGN_2026-05-15](findings/PATH_IV_PI_TENSOR_DESIGN_2026-05-15.md) — full mathematical wall function (1-2 weeks)
- [WALL_MODEL_RESEARCH](findings/WALL_MODEL_RESEARCH.md) — Han 2021 reference details
