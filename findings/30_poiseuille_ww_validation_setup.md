# Finding 30: Poiseuille Channel WW-Validation — 3-Variant Setup Sketch

**Datum:** 2026-05-13
**Status:** Code-Skizze, NICHT kompiliert, NICHT gerunnt. Awaiting Heiko's
review before integration in setup.cpp.

**Ziel:** Canonical turbulent plane-channel-flow Validation, vergleichbar mit
Lehmann's `main_setup() { // Poiseuille flow validation }` (laminar) und
Krüger p.256 (typical L2 errors 2-5%). Three variants to isolate WW-mechanic
correctness from geometry effects.

---

## Reference Targets

- **Laminar Poiseuille** (existing in setup.cpp:854): parabolic profile
  `u(y) = u_max × (R²-r²)/R²`, validated to 2-5% L2-error.
- **Turbulent channel log-law** (Krüger p.138): `u+ = (1/κ)·log(y+) + B`
  with κ=0.41, B=5.0 for y+ > 30.
- **Lee-Moser DNS 2015** Re_τ = 5200: bulk Reynolds Re_b ≈ 200000.

For our purposes: Re_τ = 1000 (matches MR2 y+ ≈ 300 range with 3 cells
across BL).

---

## Test Geometry

```
   Z (wall-normal)
   ^
   |
   +---H_top (TYPE_S, u=0 or u=u_slip)---+
   |                                     |
   |          turbulent channel          |
   |          u_x = u(z) (log-law)       | → X (streamwise, periodic)
   |                                     |
   +---H_bot (TYPE_S, u=0 or u=u_slip)---+

Y: periodic (spanwise)
```

Channel dimensions (lattice units):
- `Nx = 64`  (streamwise, periodic)
- `Ny = 64`  (spanwise, periodic)
- `Nz = 128` (wall-normal, walls at z=0 and z=Nz-1)

Physical: H = Nz × Δx = 128 × 1.5625mm = 0.2m, Re_τ = 1000 → u_τ ≈ 0.015 m/s,
ν = 1.5e-5, driven by volume-force g_x ≈ ρ × u_τ² / H_half.

---

## Variant A: Pure BB (Baseline)

```cpp
void main_setup_poiseuille_BB() {
    const float lbm_u_bulk = 0.05f;  // bulk velocity (lattice units, << c_s)
    const float Re_tau = 1000.0f;
    const uint Nx = 64u, Ny = 64u, Nz = 128u;
    const float lbm_nu = 0.01f;  // tau = 0.53 — turbulent stable
    const float H_half = (float)(Nz/2);

    // Driving force from u_tau² balance: g = u_tau² / H_half
    const float lbm_u_tau = lbm_u_bulk * 0.05f;  // ≈ u_b/20 typical
    const float lbm_g = lbm_u_tau*lbm_u_tau / H_half;

    LBM lbm(Nx, Ny, Nz, lbm_nu, lbm_g, 0.0f, 0.0f);

    parallel_for(lbm.get_N(), [&](ulong n) {
        uint x=0, y=0, z=0; lbm.coordinates(n, x, y, z);
        if(z==0u || z==Nz-1u) {
            lbm.flags[n] = TYPE_S;          // BB walls, no TYPE_X
            // u remains 0 → pure no-slip
        } else {
            // Initialize with log-law profile to accelerate convergence
            const float z_dim = (z < Nz/2u) ? (float)z + 0.5f : (float)(Nz-1u-z) + 0.5f;
            const float y_plus = z_dim * lbm_u_tau / lbm_nu;
            const float u_log = (y_plus > 11.81f)
                ? lbm_u_tau * (2.44f*log(y_plus) + 5.0f)
                : lbm_u_tau * y_plus;
            lbm.u.x[n] = fmin(u_log, lbm_u_bulk * 1.2f);
        }
    });

    // Run + collect z-profile for L2-error vs log-law
    lbm.run(0u);  // init
    for(uint chunk=0u; chunk<200u; chunk++) {
        lbm.run(1000u);  // 200k steps total
        lbm.u.read_from_device();
        // Average u_x over X,Y per Z, compute L2 vs log-law
        // ... (export to CSV: z, u_avg(z), u_log_ref(z), error(z))
    }
}
```

**Expected:**
- u_x near wall (z=1): ≈ 0.5 lattice (BB linear extrapolation), NOT log-law
- Bulk velocity over-predicted compared to true turbulent profile
- L2-error vs log-law: 30-100% (BB-only is wrong for turbulent BL)

---

## Variant B: Krüger Moving-Wall with u_slip (current WW pattern)

```cpp
void main_setup_poiseuille_WW_Krueger() {
    // ... same dimensions, nu, force as Variant A ...

    LBM lbm(Nx, Ny, Nz, lbm_nu, lbm_g, 0.0f, 0.0f);

    parallel_for(lbm.get_N(), [&](ulong n) {
        uint x=0, y=0, z=0; lbm.coordinates(n, x, y, z);
        if(z==0u || z==Nz-1u) {
            lbm.flags[n] = TYPE_S | TYPE_X;   // WW-marked walls

            // Set u_slip from Werner-Wengle PowerLaw on FIRST FLUID cell above
            // For simplicity: u_slip = u_log at y+ = 0.5 (half-cell offset)
            const float y_plus_wall = 0.5f * lbm_u_tau / lbm_nu;
            const float u_slip = (y_plus_wall > 11.81f)
                ? lbm_u_tau * (2.44f*log(y_plus_wall) + 5.0f)
                : lbm_u_tau * y_plus_wall;
            lbm.u.x[n] = u_slip;
            lbm.u.y[n] = 0.0f;
            lbm.u.z[n] = 0.0f;
        } else {
            // ... log-law initial as in A ...
        }
    });

    // BUT: this uses the CC#10 apply_moving_boundaries Krüger correction
    // → the same force artifact as on Cube/MR2 would appear here too,
    //   IF we measured force on the wall.
    // For Poiseuille, we don't measure wall-force; we measure u_x(z) profile.
    // → THIS test cleanly isolates whether WW-slip-mechanic produces the
    //   right LOG-LAW PROFILE, without contamination from force-artifact.

    lbm.run(0u);
    for(uint chunk=0u; chunk<200u; chunk++) {
        lbm.run(1000u);
        // ... profile export + L2-error ...
    }
}
```

**Expected:**
- u_x at z=1: ≈ u_slip ≈ u_log(y+ = 0.5) (close to BB-extrapolation but with
  WW-slip "boost")
- Bulk velocity matches log-law profile
- L2-error vs log-law: 2-15% (typical WW quality)

**Diagnose-Wert:** wenn dies funktioniert, ist die WW-Krüger-Mechanik prinzipiell
korrekt — der Cube/MR2-Bug ist Force-Measurement-Artifact, NICHT Slip-Bug.

---

## Variant C: TYPE_X-Exclusion (current Safe-State)

```cpp
void main_setup_poiseuille_WW_excluded() {
    // ... same as B, walls TYPE_S | TYPE_X with u_slip set ...
    // BUT: apply_moving_boundaries skips TYPE_X → u_slip is "dead data"
    // → Effective: pure BB at walls (= same as Variant A)

    // Use case: confirm that Safe-State really IS equivalent to BB
    // (no Krüger correction propagates to fluid even though u_slip is set)
}
```

**Expected:**
- Identical to Variant A (BB-baseline)
- L2-error vs log-law: 30-100%
- Bulk velocity matches Variant A within numerical noise

**Diagnose-Wert:** confirms TYPE_X-Exclusion really has zero effect on flow,
independent of what u_slip is set to.

---

## Output Pipeline (alle Varianten)

CSV per chunk:
```
chunk, t_step, z, u_x_avg(z), u_log_ref(z), error(z)
```

Plot in Python:
```python
import pandas as pd, matplotlib.pyplot as plt
for variant in ['BB', 'WW_Krueger', 'WW_excluded']:
    df = pd.read_csv(f'poiseuille_{variant}.csv')
    df_last = df[df.chunk == df.chunk.max()]
    plt.semilogx(df_last.y_plus, df_last.u_x_avg / lbm_u_tau, label=variant)
plt.semilogx(y_plus_ref, u_plus_ref, 'k--', label='log-law')
plt.xlabel('y+'); plt.ylabel('u+')
plt.legend(); plt.savefig('poiseuille_validation.png')
```

---

## Erwartete Ergebnisse + Decision Tree

| Variant | u+ at y+=10 | u+ at y+=100 | L2 vs log-law | Interpretation |
|---|---:|---:|---:|---|
| A (BB) | ~10 (linear) | ~80 (over-shoot) | 50-100% | BB doesn't capture log-law |
| B (WW Krüger) | ~14 (log) | ~16 (log) | 2-15% | WW-mechanic correct |
| C (TYPE_X-excl) | ~10 (linear) | ~80 | 50-100% | Same as A |

**Falls B nicht das log-law-Profil produziert** (L2 > 30%):
→ Werner-Wengle PowerLaw-Closed-Form ist falsch parametrisiert
→ ODER `apply_moving_boundaries` Krüger-Coefficient ist falsch
→ ODER beide bauen sich gegenseitig auf zu falschem Profil

**Falls B funktioniert (L2 < 15%):**
→ WW-Mechanik IS korrekt
→ Cube/MR2-Failure war PUR Force-Measurement-Artifact (update_force_field)
→ Path-Forward: Force-Subtraction-Methode oder TYPE_X-Exclusion in
  update_force_field statt in apply_moving_boundaries

---

## Implementation-Anmerkungen für Heiko

1. **Wo einfügen:** Ich empfehle Block ab Zeile 916 in setup.cpp (nach
   existing Stokes drag), als 3 separate Funktionen plus eine Dispatcher.
2. **Compile-Toggle:** `#define POISEUILLE_VALIDATION 0/1/2/3` analog zu
   `AHMED_MODE`. 0=off, 1=Variant A, 2=Variant B, 3=Variant C.
3. **Required extensions:** VOLUME_FORCE, MOVING_BOUNDARIES, SUBGRID. KEINE
   FORCE_FIELD (Wand-Force interessiert hier nicht).
4. **Runtime:** ~200k steps @ 64×64×128 = ca. 524k cells → ca. 1-2 min auf
   B70 (extrapoliert aus MR2-Performance 5464 MLUPS).
5. **No STL needed** — pure flag-marking, ähnlich main_setup_cube.
6. **Iron-Rule:** Jede Variante 1× laufen lassen, bei Iteration auf bessere
   Init-Conditions max 2 Re-Run pro Variante. Wenn nach 3 Re-Runs immer
   noch L2 > 50% bei Variant B → fundamentales WW-Mechanik-Problem, Phase
   B → OpenLB Pi-Tensor direkt.

---

## See Also

- [[29_geometry_re_yplus_matrix]] — y+ Berechnung für reale Geometrien
- TEIL A.2 (`/tmp/force_code_audit.md`) — Force-Computation Code-Path
- [[25_ww_six_failed_attempts]] — Empirische Cube/MR2 Daten
