# Phase 7 — Triple-Resolution mit iGPU als Outer-Wake-Domain — Design Spec

**Status:** Design Complete, Implementation pending Phase 6C/6D Resultate
**Datum:** 2026-05-16

## Konfiguration

3:1:2 Resolution-Cascade auf 2 GPUs:

| Domain | Device | dx | World-Box | Cells | VRAM |
|---|---|---:|---|---:|---:|
| **Near** | B70 (renderD129) | 4 mm | X[−0.4, +5.6] Y[±1.296] Z[0, +1.608] | 1500×648×402 = 391 M | 23.6 GiB |
| **Far** | B70 (renderD129) | 12 mm | X[−0.808, +8] Y[±2.004] Z[0, +2.616] | 734×334×218 = 53.4 M | 3.4 GiB |
| **Coarse** | iGPU (renderD128, Arrow Lake) | 24 mm | X[−1.6, +16.8] Y[±4.5] Z[0, +7.584] | 700×375×316 = 83 M | 5.0 GiB sys RAM |

**B70 Total: ~29 GiB / 32 GiB** | **iGPU: 5 GiB / 91 GiB shared**

## Vehicle Setup (STL-native per Memory)

```cpp
const float si_length = 4.4364f;  // Original STL bbox X-Extent, kein artificial Rescaling mehr
```

Vehicle bbox SI: 4.4364 × 1.8379 × 1.2077 m. Vehicle nur in Near + Far voxelisiert (NICHT in Coarse — bei 24 mm zu coarse).

## Blending (Phase 6C/6D-Pattern angewendet auf beide Cascade-Ebenen)

Phase 6 Spiegelrampe mit Plateau auf jeder Cascade-Ebene:

| Cascade | Band Near-Cells | Band Far-Cells | Band Physisch | Plateau am Boundary |
|---|---:|---:|---:|---:|
| **Near↔Far** (3:1) | 36 | 12 | 144 mm | 3 Near-Cells = 1 Far-Cell |
| **Far↔Coarse** (2:1) | — | 12 (Far-Outer) | 144 mm | 2 Far-Cells = 1 Coarse-Cell |

Cell-Counts pro Cascade müssen Multiple des Ratios sein:
- Near↔Far: BAND_NEAR = 3k (multiple of 3); BAND_FAR = k
- Far↔Coarse: BAND_FAR_OUTER = 2m (multiple of 2); BAND_COARSE = m

α-Bereich (zu validieren via Phase 6C/6D):
- 6C: 0.05 ↔ 0.50 mit Plateau
- 6D: 0.0 ↔ 1.0 mit Plateau (falls 6C stabil)

## Per-LBM Device Selection — Implementations-Pfad

FluidX3D's aktuelle Device-Auswahl in `LBM::LBM(...)` Konstruktoren ruft `smart_device_selection(D)` auf, das die schnellste verfügbare Device-Klasse für alle D Domain-Splits wählt. Für Triple-Res mit verschiedenen GPUs (B70 + iGPU) brauchen wir explizite Per-LBM-Device-Übergabe.

### Vorgeschlagene API-Erweiterung

Neue LBM-Konstruktor-Variante in `src/lbm.hpp`:
```cpp
class LBM {
public:
    // ... bestehende Konstruktoren ...

    // Phase 7 2026-05-16: Single-Device LBM mit explicit Device_Info (bypasst smart_device_selection).
    // Für Multi-GPU-Setups wo verschiedene LBM-Instanzen auf verschiedenen Devices laufen sollen.
    LBM(const uint3 N, const float nu, const Device_Info& device_info,
        const float fx=0.0f, const float fy=0.0f, const float fz=0.0f,
        const float sigma=0.0f, const float alpha=0.0f, const float beta=0.0f,
        const uint particles_N=0u, const float particles_rho=1.0f);
};
```

Implementation in `src/lbm.cpp`:
```cpp
LBM::LBM(const uint3 N, const float nu, const Device_Info& device_info,
         const float fx, const float fy, const float fz,
         const float sigma, const float alpha, const float beta,
         const uint particles_N, const float particles_rho) {
    this->Nx = N.x; this->Ny = N.y; this->Nz = N.z;
    this->Dx = 1u; this->Dy = 1u; this->Dz = 1u;
    const vector<Device_Info> device_infos = {device_info};  // single device, explicit
    sanity_checks_constructor(device_infos, this->Nx, this->Ny, this->Nz, 1u, 1u, 1u,
                              nu, fx, fy, fz, sigma, alpha, beta, particles_N, particles_rho);
    lbm_domain = new LBM_Domain*[1u];
    lbm_domain[0] = new LBM_Domain(device_info, this->Nx, this->Ny, this->Nz,
                                    1u, 1u, 1u, 0, 0, 0,
                                    nu, fx, fy, fz, sigma, alpha, beta,
                                    particles_N, particles_rho);
    sanity_checks_initialization();
    initialized = false;
}
```

### Setup.cpp Triple-Res-Block

```cpp
void main_setup_phase7_triple_res() {
    // ... physics constants ...
    const float dx_near = 0.004f, dx_far = 0.012f, dx_coarse = 0.024f;
    const float si_length = 4.4364f; // STL-native!

    // Per-domain Units objects (global units = Far for compatibility)
    Units units_near, units_far, units_coarse;
    units_near.set_m_kg_s(si_length/dx_near, lbm_u, 1.0f, si_length, si_u, si_rho);
    units_far .set_m_kg_s(si_length/dx_far,  lbm_u, 1.0f, si_length, si_u, si_rho);
    units_coarse.set_m_kg_s(si_length/dx_coarse, lbm_u, 1.0f, si_length, si_u, si_rho);
    units = units_far;
    const float nu_near   = units_near.nu(si_nu);
    const float nu_far    = units_far.nu(si_nu);
    const float nu_coarse = units_coarse.nu(si_nu);

    // Device selection
    const vector<Device_Info>& devices = get_devices();
    Device_Info dev_b70 = select_device_with_name(devices, "Intel(R) Graphics [0xe223]");
    Device_Info dev_igpu = select_device_with_name(devices, "Intel(R) Graphics"); // ohne Hex-ID

    // Three LBM instances, each on its own device
    LBM lbm_near  (uint3(1500u, 648u, 402u), nu_near,   dev_b70);   // Near on B70
    LBM lbm_far   (uint3( 734u, 334u, 218u), nu_far,    dev_b70);   // Far on B70 (separate queue)
    LBM lbm_coarse(uint3( 700u, 375u, 316u), nu_coarse, dev_igpu);  // Coarse on iGPU

    // BC setup similar to Phase 5b-DR but with Coarse domain added
    // ...

    // Triple-cascade coupling planes:
    //   Near ↔ Far (3:1), Far ↔ Coarse (2:1)
    // Each pair gets: 5 forward + 5 back + 36/12 layer Blending mit Plateau

    // Run-Loop:
    //   Coarse 25 steps (per Coarse-Chunk = 50 Far-steps = 150 Near-steps for 3:1:2 ratio)
    //   Concurrent: lbm_near.run_async + lbm_far.run_async + lbm_coarse.run_async
    //   Sync, dann Coupling
}
```

## Coupling-Topology

Pro Coarse-Chunk (gleiche SI-Zeit für alle drei):
- **Coarse**: 25 Steps × 83 M cells = 2.08 G cell-steps → iGPU ~5 s
- **Far**: 50 Steps × 53.4 M = 2.67 G cell-steps → B70 ~0.4 s
- **Near**: 150 Steps × 391 M = 58.65 G cell-steps → B70 ~10 s

**B70 limiter**: ~10 s pro Coarse-Chunk. iGPU bei 50% Auslastung. Massive Reserve.

Pro Chunk Coupling:
- **Near↔Far**: 240 couple_fields-Calls (5 forward × 36 + 5 back × 12) — wie Phase 6
- **Far↔Coarse**: 60 couple_fields-Calls (5 forward × 12 + 5 back × 6) — neu
- **Total**: 300 couple_fields-Calls / Chunk. ~5-10 sek Wallclock-Overhead.

iGPU↔B70 Data-Transfer pro Chunk:
- Coarse → Far Coupling: ~10 MB Plane-Data
- Far → Coarse Back: ~5 MB
- Via Host-RAM: ~3 ms Transfer-Zeit (negligible)

## Erwartete Force-Werte (vs Phase 6B Baseline Fx_near=1180, Fz_near=−882)

| Quelle | Erwarteter Δ Drag | Erwarteter Δ Downforce |
|---|---:|---:|
| Far 20→12 mm (+ feinere Wake-Cell-Resolution) | −2 bis −5 % | +0 bis +5 % |
| Coarse 18.4 m Wake-Extension | −5 bis −15 % | +5 bis +10 % |
| STL-native (4.5→4.4364) | ~0 % | ~0 % |
| Phase 6C/6D Plateau-Blending | −1 bis −3 % | +0 bis +3 % |
| **Σ Phase 7 vs Phase 6B** | **−8 bis −23 %** | **+5 bis +18 %** |
| **Σ Phase 7 vs Phase 5.1** | **−28 bis −40 %** | **+14 bis +27 %** |

Force-Prognose Phase 7:
- Fx_near: ~900-1100 N (Cd 0.88-1.07)
- Fz_near: ~−950 bis −1050 N (~100 kg Downforce)

Erstmals **Cd unter 1.0 in greifbarer Nähe**, was im realistic-Race-Car-Drag-Bereich liegt.

## Implementations-Aufwand

| Komponente | Aufwand | Quelle |
|---|---:|---|
| Per-LBM-Device-Selection in lbm.cpp + lbm.hpp | 1.5 h | neu |
| Triple-Domain-Setup-Block in setup.cpp | 2 h | analog Phase 5b-DR |
| Far↔Coarse Coupling-Planes + Blending-Loop | 2 h | analog Near↔Far Phase 6 |
| Run-Loop Concurrent 3-Device | 1 h | analog Mode 3 PERF-G |
| Build + Sanity-Test Tier-1 (10 chunks) | 30 min | |
| Production Run | 60-90 min | |
| Doku + Commit + Push | 30 min | |
| **Total** | **~7-8 h** | |

## Trigger-Condition

Phase 7 wird gebaut **nachdem**:
1. Phase 6C (Plateau + α 0.05-0.50) konvergiert + visuell überzeugt
2. Phase 6D (Plateau + α 0.0-1.0) konvergiert + ist stabil (kein NaN)
3. User explizit Phase 7 freigibt

Bei dem Ergebnis können wir die Phase-6-Best-α-Werte direkt in Phase 7 Blending übernehmen.

## See Also

- [PHASE_6_BLENDING_COMPARISON_2026-05-16.md](PHASE_6_BLENDING_COMPARISON_2026-05-16.md) — 6A vs 6B Blending α-Test
- [fluidx3d_stl_native_size memory](../../../.claude/projects/-home-heiko/memory/fluidx3d_stl_native_size.md) — STL-native si_length policy
- [PHASE_5_1_FINAL_RESULT_2026-05-16.md](PHASE_5_1_FINAL_RESULT_2026-05-16.md) — Baseline distance-based wall model
