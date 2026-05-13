# Finding 31: u_slip/y+/τ_w Diagnostic Kernel — Code Sketch

**Datum:** 2026-05-13
**Status:** Code-Skizze, NICHT kompiliert, NICHT gerunnt. Awaiting Heiko's
review before integration.

**Ziel:** Per-Wall-Cell Diagnose-Felder exportieren, die direkt mit Heikos
OpenFOAM-MR2-Ergebnissen verglichen werden können — als Ground-Truth-Probe
ob unser Werner-Wengle PowerLaw die richtigen Werte liefert.

---

## Strategie: VRAM-Light Compile-Toggle

Per `#define DIAGNOSTIC_WW` werden 4 zusätzliche Felder allokiert. Ohne
Define: keine zusätzliche VRAM-Kosten in Production-Builds.

VRAM-Kosten bei aktivem Diagnostic (MR2 Voll-Domain 168M Cells):
| Field | Type | Size |
|---|---|---:|
| u_slip_diag | float×3 | 1.92 GB |
| u_avg_diag | float×3 | 1.92 GB |
| y_plus_diag | float×1 | 640 MB |
| tau_wall_diag | float×3 | 1.92 GB |
| **Total** | | **6.4 GB** |

→ Innerhalb B70 32 GB Budget. **Alternative für VRAM-Limited:** alloziere
nur an Wall-Cells (sparse, ~0.1% von 168M = 168k cells × ~30 bytes = 5 MB).

Für Diagnose nur 1× Export reichen 6.4 GB temporär. Production immer ohne
DIAGNOSTIC_WW kompiliert.

---

## defines.hpp Addition

```cpp
// #define DIAGNOSTIC_WW       // CC#11 Phase A: u_slip, u_avg, y+, tau_wall export
                                //  - enables compute_ww_diagnostic kernel
                                //  - allocates 4 extra float buffers at wall cells
                                //  - VTK export with vehicle_diag_<step>.vtk
                                //  - turn OFF for production runs
```

---

## lbm.hpp Additions (sketch)

```cpp
class LBM_Domain {
public:
#ifdef DIAGNOSTIC_WW
    Memory<float> u_slip_diag;     // u_slip vector per wall cell
    Memory<float> u_avg_diag;      // u_avg (input to WW) per wall cell
    Memory<float> y_plus_diag;     // y+ scalar per wall cell
    Memory<float> tau_wall_diag;   // wall shear stress vector
    Kernel kernel_compute_ww_diagnostic;
    void enqueue_compute_ww_diagnostic();
#endif
};
```

## lbm.cpp Init (sketch)

```cpp
#ifdef DIAGNOSTIC_WW
LBM_Domain::LBM_Domain(...) : ... 
    , u_slip_diag(device, 3u*N, 1u)
    , u_avg_diag(device, 3u*N, 1u)
    , y_plus_diag(device, N, 1u)
    , tau_wall_diag(device, 3u*N, 1u)
{
    ...
    kernel_compute_ww_diagnostic = Kernel(device, N, "compute_ww_diagnostic",
        flags, u, rho,
        u_slip_diag, u_avg_diag, y_plus_diag, tau_wall_diag,
        lbm_u_inf, lbm_nu);
}
#endif
```

---

## kernel.cpp — New Diagnostic Kernel (sketch)

```cpp
#ifdef DIAGNOSTIC_WW
)+R(kernel void compute_ww_diagnostic(
    const global uchar* flags,
    const global float* u,
    const global float* rho,
    global float* u_slip_out,
    global float* u_avg_out,
    global float* y_plus_out,
    global float* tau_wall_out,
    const float lbm_u_inf,
    const float lbm_nu)
{
    const uxx n = get_global_id(0);
    if(n>=(uxx)def_N || is_halo(n)) return;

    const uchar fn = flags[n];
    // Only diagnose at vehicle WW cells (TYPE_S|TYPE_X)
    if((fn & (TYPE_S|TYPE_X)) != (TYPE_S|TYPE_X)) {
        // Zero out for clean VTK
        store3(u_slip_out, n, (float3)(0.0f, 0.0f, 0.0f));
        store3(u_avg_out, n, (float3)(0.0f, 0.0f, 0.0f));
        y_plus_out[n] = 0.0f;
        store3(tau_wall_out, n, (float3)(0.0f, 0.0f, 0.0f));
        return;
    }

    // 1. Compute u_avg from fluid neighbors (same logic as apply_wall_model_vehicle)
    uxx j[def_velocity_set];
    neighbors(n, j);
    float ux_avg=0, uy_avg=0, uz_avg=0; int count=0;
    for(uint i=1u; i<def_velocity_set; i++) {
        const uxx ji = j[i];
        if((flags[ji] & TYPE_BO) != TYPE_S) {  // fluid neighbor
            ux_avg += u[ji];
            uy_avg += u[def_N + (ulong)ji];
            uz_avg += u[2ul*def_N + (ulong)ji];
            count++;
        }
    }
    if(count > 0) {
        ux_avg /= (float)count;
        uy_avg /= (float)count;
        uz_avg /= (float)count;
    }
    store3(u_avg_out, n, (float3)(ux_avg, uy_avg, uz_avg));

    // 2. Compute u_slip via Werner-Wengle PowerLaw (CLOSED-FORM from CC#10)
    const float u_t = sqrt(ux_avg*ux_avg + uy_avg*uy_avg + uz_avg*uz_avg);
    // First fluid cell distance: 0.5 cells (BB convention)
    const float y_lattice = 0.5f;
    // Closed-form u_tau from PowerLaw: u+ = 8.3 × y+^(1/7) for y+ > 11.81
    // → u_tau = (u_t / [8.3 × (y/nu)^(1/7)])^(7/8)
    const float y_over_nu = y_lattice / lbm_nu;
    const float u_tau_pl = pow(u_t / (8.3f * pow(y_over_nu, 1.0f/7.0f)), 7.0f/8.0f);
    // Test y+ for laminar/turbulent branch:
    const float y_plus_test = y_lattice * u_tau_pl / lbm_nu;
    float u_tau;
    if(y_plus_test < 11.81f) {
        // Linear sublayer: u+ = y+, u_tau = sqrt(nu * u_t / y)
        u_tau = sqrt(lbm_nu * u_t / y_lattice);
    } else {
        u_tau = u_tau_pl;
    }
    // u_slip = u_t × direction(u_avg)  
    // Note: u_slip magnitude here equals u_t (full tangential velocity), the WW
    // effect propagates through Krüger correction. Different definitions exist;
    // here we follow CC#10 convention.
    float3 dir = (u_t > 1e-9f)
        ? (float3)(ux_avg, uy_avg, uz_avg) / u_t
        : (float3)(0.0f, 0.0f, 0.0f);
    store3(u_slip_out, n, u_t * dir);  // same as u_avg in current convention

    // 3. y+ scalar
    y_plus_out[n] = y_lattice * u_tau / lbm_nu;

    // 4. Wall shear stress: tau_w = rho × u_tau²
    const float rho_n = (count > 0) ? rho[j[1]] : 1.0f;  // sample fluid neighbor rho
    const float tau_w_mag = rho_n * u_tau * u_tau;
    store3(tau_wall_out, n, tau_w_mag * dir);
}
)+R(
#endif // DIAGNOSTIC_WW
```

---

## lbm.cpp Export Pipeline (sketch)

```cpp
#ifdef DIAGNOSTIC_WW
void LBM::export_ww_diagnostic(const string& path, const ulong step) {
    // After ~1000 timesteps when flow has developed
    for(uint d=0; d<get_D(); d++) lbm_domain[d]->enqueue_compute_ww_diagnostic();
    finish_queue();

    lbm_domain[0]->u_slip_diag.read_from_device();
    lbm_domain[0]->u_avg_diag.read_from_device();
    lbm_domain[0]->y_plus_diag.read_from_device();
    lbm_domain[0]->tau_wall_diag.read_from_device();

    // Write 4 VTK files at path/vehicle_diag_<step>_<field>.vtk
    write_vtk_vector(path + "u_slip_step" + step_str + ".vtk",
                     lbm_domain[0]->u_slip_diag.x, ...);
    // analog für u_avg, y_plus, tau_wall
}
#endif
```

---

## setup.cpp Integration Point (sketch)

```cpp
// In main_setup() loop, after sufficient timesteps:
for(uint chunk=0; chunk<num_chunks; chunk++) {
    lbm.run(100u);
    // ... existing force CSV logic ...

#ifdef DIAGNOSTIC_WW
    // Export at chunk 10, 50, 100 (= step 1000, 5000, 10000)
    if(chunk == 10u || chunk == 50u || chunk == 100u) {
        lbm.export_ww_diagnostic("export/", lbm.get_t());
    }
#endif
}
```

---

## OpenFOAM-Vergleich Pipeline (ParaView)

1. Lade FluidX3D-Export `vehicle_diag_step10000_y_plus.vtk` in ParaView.
2. Lade OpenFOAM-Output (MR2 sym-plane case) — vermutlich `yPlus` field.
3. Resample FluidX3D-Diagnose auf OpenFOAM-Mesh (oder umgekehrt).
4. Calculator-Filter: `yPlus_FluidX3D - yPlus_OpenFOAM`.
5. Plot Histogram der Difference.

Erwartet:
- Wenn FluidX3D y+ deutlich höher als OpenFOAM → BL ist zu grob, WW
  versucht zu viel zu kompensieren.
- Wenn FluidX3D y+ deutlich niedriger als OpenFOAM → WW unter-prediktet
  Friction Velocity, zu schwache Wirkung.
- Wenn matching → WW-Mechanik tut die richtige Arbeit, Cube/MR2-Problem
  liegt in Force-Measurement (update_force_field artifact).

---

## Iron-Rule für Phase A.4 Run

- **Max 2 Runs** mit Diagnostic-Buffer (jeder ~2 min auf B70)
- **Output-Verification:** check VTK reads cleanly in ParaView vor Compare
- **VRAM-Check:** if compile aborts wegen OOM, reduce zu sparse-storage
  (nur Wall-Cells)
- Wenn nach 2 Runs keine clean comparison möglich → Phase A abbrechen,
  manuelle Force-Mass-Balance per Hand stattdessen

---

## Anmerkungen für Code-Review (Heiko)

1. **Werner-Wengle Branching:** Mein Sketch hat eine if-else für Laminar/
   Turbulent Sublayer. Original CC#10 hat das vielleicht eleganter gelöst —
   bitte mit `apply_wall_model_vehicle` in kernel.cpp:1478 vergleichen.
2. **Density-Sampling:** Ich sample `rho` vom ersten Fluid-Nachbarn `j[1]`,
   was richtungs-spezifisch (+X) ist. Robuster wäre Average wie bei u_avg.
3. **VTK-Export Format:** FluidX3D nutzt STRUCTURED_POINTS. Diese 4 Felder
   sind nur an Wall-Cells nicht-null → Sparse-Pattern in dichtem Grid. Bei
   Visualisierung "Cells > Threshold magnitude > 0" filtern.
4. **Heikos VRAM-Constraint:** wenn 6.4 GB extra zu viel → Plan B: nur
   `y_plus_diag` (640 MB) zuerst exportieren, andere Felder nach 2. Run.

---

## See Also

- [[30_poiseuille_ww_validation_setup]] — 3-Variant Poiseuille (parallel test)
- [[29_geometry_re_yplus_matrix]] — y+ Targets pro Geometrie
- `src/kernel.cpp:1478` — Existing apply_wall_model_vehicle (Reference)
