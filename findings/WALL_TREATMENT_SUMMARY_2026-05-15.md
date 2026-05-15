# Wall Treatment Pfad-Übersicht — Session 2026-05-15 Summary

**Status:** 4 distinkte wall-treatment Ansätze in FluidX3D EP-pull-Architektur getestet. ALLE versagen für komplexe MR2-STL Vehicle. Pure-BB Mode 3 + α=0.15 + TYPE_S Floor bleibt einzige stable baseline.

## Was getestet wurde

| # | Pfad | Mechanismus | Result auf MR2 |
|---|---|---|---|
| 1 | **WW Vehicle (CC#10)** | `u[wall_cell] = u_slip`, Krüger Moving-Wall transfer | Three-Attractor Pathology (−610 / +163k / +290k N je nach Krüger-Multiplier) |
| 2 | **WW Floor (Path II.5)** | gleicher Mechanismus für TYPE_S Floor mit rolling-road | NO-OP — u_2 ≈ u_road in Pure-BB steady-state |
| 3 | **Sparse Bouzidi BB** | Sub-cell q-fraction DDF interpolation post-stream | Flow collapse zu zero (single-direction) oder Pathology (multi-direction) |
| 4 | **Multi-cell u-prescription (WALL_SLIP)** | DDFs=feq(slip_factor*u_local) at wall-adj fluid cells | **Catastrophic** −200k N drag at slip=1.0 |

## Gemeinsames Architektur-Failure-Pattern

Alle 4 Ansätze modifizieren DDFs oder feq an wall-adjacent fluid cells. FluidX3D's **Esoteric-Pull (EP) Layout** ist inkompatibel mit solchen Modifikationen für komplexe Geometrien:

1. **EP-pull Layout** koppelt load_f/store_f mit implicit streaming → load+store without proper modification corrupts data (Phase 1 Bouzidi insight)
2. **Inline modifications in stream_collide** werden NEUTRALISIERT (kernel.cpp:1717-1719 CC#7 deprecation comment)
3. **Custom post-stream kernels** funktionieren NUR wenn:
   - Mirror-symmetric (apply_freeslip_y: TYPE_Y mirror)
   - Axis-aligned simple walls (Bouzidi Step 1 Poiseuille z-walls)
   - NOT for: complex STL, multi-direction walls, non-symmetric modifications

apply_freeslip_y ist der einzige funktionierende Wall-Treatment in FluidX3D's EP-pull weil:
- Strikt mirror-symmetric (Y-direction DDFs gespiegelt)
- Erhält Y-momentum balance
- Wird auf TYPE_Y cells angewandt die geometrisch trivial sind (flat Y_min plane)

Bouzidi und WALL_SLIP injizieren **non-symmetric information** (gewichtete Interpolation, slip-decay) → EP-pull-Konsistenz bricht.

## Bestätigung im Code

Architektur-Limit ist EXPLICIT dokumentiert in `kernel.cpp` Lines 1717-1719:

> "DEPRECATED: CC#7-V1/V2 TYPE_Y inline specular reflection (swap AND one-way assignment were both tried, both produced bit-identical Fx≈14400 N — **Esoteric-Pull layout effectively neutralises inline DDF modifications at TYPE_Y cells**)"

Wir haben jetzt diesen Pattern unabhängig 4× reproduziert. Die EP-pull-Layout-Inkompatibilität ist real und fundamental.

## Pfade die untestet bleiben (höhere Aufwände)

| Pfad | Mechanismus | Aufwand | Erfolg-Wahrscheinlichkeit |
|---|---|---|---|
| **Pi-Tensor f_neq (Han 2021)** | Direct modification of **non-equilibrium part** f_neq an wall-adj cells via stress tensor reconstruction | 1-2 Wochen | 60-70% — andere mathematische Basis, sidesteps inline DDF/feq overwrite |
| **Modify stream_collide (replace BB)** | Replace BB at wall-adj cells with Bouzidi formula inside stream_collide kernel | 2-3 Tage | 30% (CC#7 comment suggests this gets neutralized) |
| **MRT collision (statt SRT)** | Multi-Relaxation-Time scheme — andere relaxation behavior, evtl. compatible mit DDF mods | 1-2 Wochen | unklar |
| **Switch to non-EP-pull FluidX3D fork** | Use FluidX3D variant ohne Esoteric-Pull, e.g. AB-style | 2+ Wochen | hoch aber huge investment |
| **External wall correction via FORCE_FIELD** | Add body force at wall-adj cells pulling u toward target | 3-5 Tage | 40-50% (forcing-magnitude tuning critical) |

## Empfehlung

**Kurzfristig (heute):** **Pure-BB Mode 3 + α=0.15 + TYPE_S Floor Baseline akzeptieren** für aktuelle Production. Fx ~1500 N (2.5× target), Fz_near ~-552 N (downforce ✓, korrekte Richtung).

**Mittelfristig:** **Path IV Pi-Tensor f_neq (Han 2021)** als ernsthaften Wall-Function-Approach. 1-2 Wochen Investment. Theoretisch der EINZIGE sauber-EP-pull-kompatible Ansatz weil f_neq-Modifikation Eingriff in NICHT-equilibrium Komponente (Stress) statt direkten DDF-Werten.

**Alternative:** **External Force-Field Pull** — gentle body force pulling fluid u toward law-of-the-wall value. Sidesteps DDF-modification entirely. 3-5 Tage. Risiko: forcing tuning.

## Code-State

Alle wall-treatment Code-Strukturen BLEIBEN im Source als Reference + Reusable-Infrastructure:
- `WALL_MODEL_VEHICLE`: WW (disabled in defines)
- `WALL_MODEL_FLOOR`: WW Floor (disabled in defines)
- `BOUZIDI_VEHICLE`: Sparse infrastructure (compute_bouzidi_cells_active wird genutzt für WALL_SLIP cell-list)
- `WALL_SLIP_VEHICLE`: kernel + integration (wall_slip_factor stays 0 → inactive)

Setup.cpp commented out alle Aktivierungen. Bei zukünftigen Versuchen einfach Compile-Flag + Aktivierung wieder einfügen.

## See Also

- [WW_MULTIRES_FAILURE_2026-05-15](findings/WW_MULTIRES_FAILURE_2026-05-15.md)
- [FLOOR_WW_RESULT_2026-05-15](findings/FLOOR_WW_RESULT_2026-05-15.md)
- [BOUZIDI_EP_PULL_INCOMPATIBILITY_2026-05-15](findings/BOUZIDI_EP_PULL_INCOMPATIBILITY_2026-05-15.md)
- [PATH_IV_PI_TENSOR_DESIGN_2026-05-15](findings/PATH_IV_PI_TENSOR_DESIGN_2026-05-15.md)
- src/kernel.cpp:1717-1719 — CC#7 EP-pull neutralization documentation
