# Session 2026-05-15 — Complete Summary

**Branch:** `plan-refresh-multires`
**Status:** Erfolgreicher Tag. Mode 3 Production + Wall-Function-Architektur etabliert.

## Wichtige Ergebnisse Heute

### 1. Mode 3 PERF-G Concurrent LBM — IMPLEMENTED ✓
- `LBM::run_async()` + `LBM::finish()` für concurrent Far||Near execution
- 2nd cl_queue Infrastructure in opencl.hpp
- Additive Schwarz mit symmetric 1-chunk-lag → physikalisch konsistenter als Mode 2 sequential
- PERF: ~5-10% wallclock-overhead, dafür ~2× consistent forces zwischen Far+Near
- Commits: `bf589f3` (PERF-G + Bouzidi infrastructure), earlier ones

### 2. α-Sweep Validierung
- α=0.10: sichtbare Coupling-Kante, stable
- α=0.20: 24% schneller konvergiert, etwas oszillativer
- **α=0.15** (User-Entscheidung): bester Compromiss, smooth Übergänge
- Commit: `607c0e2`

### 3. TYPE_S Moving-Wall Floor — IMPLEMENTED ✓
- TYPE_E floor blockierte Venturi → Fz_near war LIFT (+290)
- TYPE_S moving wall mit u_x=lbm_u erlaubt Venturi → **Fz_near = -552 N (DOWNFORCE)** ✓
- Wheel-contact-fix bleibt (z=0 vehicle cells get lbm_u)
- Commit: `a277377`

### 4. Wall Function — 6 Approaches getestet, 1 erfolgreich

**FAILED (alle 5 — EP-pull-Architektur-Inkompatibel):**

| Approach | Failure Mode |
|---|---|
| WW Vehicle (CC#10 Krüger) | Three-Attractor Pathology (−610/+163k/+290k N) |
| WW Floor (Path II.5) | NoOp (u_2 ≈ u_road) |
| Bouzidi Sparse BB | Flow collapse oder negativer drag |
| WALL_SLIP V1 (full overwrite) | Catastrophic −200k N |
| WALL_SLIP V2 (BLEND) | f_neq decay → forces ~150 N |

Alle 5: EP-pull-Layout neutralisiert/korrumpiert custom DDF/feq modifications für komplexe STL.
Bestätigt in `kernel.cpp:1717` (CC#7 deprecated comment).

**SUCCESS — WALL_VISC_BOOST (Option B):**
- Modifies LBM relaxation rate `w` (= effective viscosity) at wall-adj cells
- KEINE DDF-modifications → natural EP-pull compatibility
- Extension to Smagorinsky kernel
- Phase 1 (hardcoded boost=0.3): 25-30% Far drag reduction (artificially strong)
- Phase 2 (physically correct Prandtl mixing-length): ~5% reduction (modest but correct)
- **Phase 3 (Multi-Cell, 3 layers — built, untested)**: ν_t = κ·y·u_tau scales linear with distance
- Vehicle + Floor cells both flagged + boosted
- Commit: `0697ba2`, `c56acad`, `4cccdaf`

### 5. Research-First Methodology — Memory Created
- User-Direktive: ausgiebige Recherche vor Implementation
- 6 LBM-Wall-Function papers gelesen + integriert
- Memory `feedback_research_first.md` erweitert
- Confirmed: FluidX3D author's hint "modify local effective viscosity" = der einzige praktische Pfad in EP-pull

## Validated Production Configuration

Stand End-of-Day (commit `4cccdaf`):
- Mode 3 (PERF-G Additive Schwarz) ✓
- α=0.15 forward+back ✓
- TYPE_S Moving Wall Floor ✓
- WALL_VISC_BOOST Phase 2 (Vehicle + Floor) ✓
- Final Forces: Fx_far=1,464 N, Fx_near=1,597 N, Fz_near=-563 N
- Wallclock: 38 min, 150 chunks zur Konvergenz

User-Feedback ParaView Sichtung:
> "Ja, ich kann eine Veränderung sehen, aber scheinbar ist der Effekt noch nicht stark oder tief genug von der Wand in die angrenzenden Zellen hinein. So vom optischen im Vergleich zu OpenFOAM."

User-Observation Mode 3 Multi-Res Coupling:
> "Wirklich interessant zu sehen wie stark das Farfield vom Nearfield bei der Wirbelauflösung durch die Kopplung profitiert!"

→ Bestätigt PERF-G Additive Schwarz Design-Hypothese: symmetric back-coupling
INjektion bringt Near's high-Res turbulent content in Far's lower-Res Domain.
Far bekommt damit "verfeinerte Wirbel-Auflösung" trotz coarse 15mm cells.
Das ist ein ECHTER Multi-Resolution-Vorteil über reine Resolution-Increase.

## Findings-Docs erstellt heute

- `findings/PERF_F_V5_DESIGN_2026-05-15.md` — Multi-Stream PCIe pipelining design
- `findings/PERF_G_CONCURRENT_LBM_2026-05-15.md` — Concurrent LBM design
- `findings/PHASE_5B_DR_PRODUCTION_2026-05-15.md` — Mode 2 Production day
- `findings/WALL_MODEL_STATUS_2026-05-15.md` — 7 wall-model paths overview
- `findings/WALL_MODEL_DEEP_ANALYSIS_2026-05-15.md` — F1-F4 failure modes
- `findings/PATH_II_5_FLOOR_WW_DESIGN_2026-05-15.md` — Floor-only WW design
- `findings/FLOOR_WW_RESULT_2026-05-15.md` — Floor-WW finding
- `findings/PATH_IV_PI_TENSOR_DESIGN_2026-05-15.md` — Han 2021 Pi-Tensor design
- `findings/PATH_BOUZIDI_SPARSE_DESIGN_2026-05-15.md` — Sparse Bouzidi design
- `findings/BOUZIDI_EP_PULL_INCOMPATIBILITY_2026-05-15.md` — Bouzidi failure analysis
- `findings/WALL_TREATMENT_SUMMARY_2026-05-15.md` — 4-approach summary
- `findings/SESSION_2026-05-11_WW_RESULTS.md` — älterer (referenz)

## Memories Updated/Created

- `paraview_vtk_alignment.md` — Mesh transform (5.25, 0.005, 2.505)
- `paraview_wayland_xcb.md` — QT_QPA_PLATFORM=xcb pflicht
- `feedback_diagnostic_methodology.md` — Nie blind abbrechen ohne Root-Cause
- `feedback_check_running_processes.md` — pgrep vor FluidX3D-Start
- `feedback_research_first.md` — Recherche ausgiebig nutzen (User-Direktive)

## See Also

- `findings/NEXT_STEPS_PLAN_2026-05-15.md` — Master-Plan (Step-by-step)
- Repo: github.com/heikogleu-dev/FluidX3D-Intel-B70 branch `plan-refresh-multires`
