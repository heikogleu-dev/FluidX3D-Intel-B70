# Modifications vs upstream ProjectPhysX/FluidX3D

**Fork base:** [ProjectPhysX/FluidX3D@4303cb4](https://github.com/ProjectPhysX/FluidX3D/commit/4303cb4) (FluidX3D v3.6, Jan 2026).

**Modifications:** Heiko Gleu, May 2026, for Intel Arc Pro B70 (BMG-G31, xe driver) on Linux. Verified runtime: oneAPI OpenCL 26.05.037020, GCC 15.2, kernel 7.0.

License clause 1 — _"Altered source versions must be plainly marked as such"_ — is fulfilled by:
- A 3-line marker comment at the top of every modified source file pointing to this document
- This `MODIFICATIONS.md` listing every change file-by-file
- A visible per-feature commit history on the `master` branch

The license itself (see [`LICENSE.md`](LICENSE.md)) is **unchanged**. Non-commercial / non-military use only.

---

## `src/defines.hpp`

- Uncommented `#define VOLUME_FORCE` (line 18). Required by `FORCE_FIELD`. Default volume force is `(0, 0, 0)` so behaviour is unchanged unless setup explicitly applies a force.
- Uncommented `#define FORCE_FIELD` (line 19). Enables `lbm.update_force_field()` and per-cell `lbm.F`. Allocates extra 12 B/cell VRAM (~200 MB at 16.85 M cells, ~1.6 GB at 135 M cells).

## `src/graphics.hpp`

- `FONT_HEIGHT  11 → 22`
- `FONT_WIDTH    6 → 12`
- 2× pixel-doubled font for readability on HiDPI primary displays (verified on 8192×3456 Display).

## `src/graphics.cpp`

- **Auto-start:** `key_P = false → true` in the global key-state initialiser. The simulation runs immediately on launch instead of starting paused; pressing `P` still toggles pause/resume normally.
- **`draw_text()`:** rewritten with a 2× scale factor. Each source glyph pixel is drawn as a 2×2 block; horizontal advance uses `FONT_WIDTH` (now 12 px); the legacy `q`-glyph offset (`character==113`) doubled to `+2`.
- **Windowed mode (`main()`):** new override block after the XRandR primary-monitor query. Default behaviour:
  - `FLUIDX3D_WINDOW` unset → 2560×1440 window, centred on primary monitor
  - `FLUIDX3D_WINDOW=WxH` (e.g. `1920x1080`) → custom size, centred
  - `FLUIDX3D_WINDOW=fullscreen` → original full-monitor behaviour
- **Motif-Hints decorations:** `0b0000000l → 0b1111111l`. The X11 window now has a title bar with close/minimise/maximise buttons; can be dragged and resized via the WM. Pre-fork the window was a chromeless full-monitor surface.

## `src/setup.cpp`

Modifications to the active `main_setup()` (the "Windkanal halbe Domain HighRes" block, line ~140):

- **Grid:** `uint3(1299u, 288u, 361u) → uint3(650u, 144u, 180u)` (axis-halved). 134.7 M cells → 16.85 M cells; cell size 3.46 mm → 6.92 mm. Faster test/iteration on B70; full 134 M still fits in B70's 28.6 GB VRAM if reverted.
- **Run length:** `lbm.run()` (infinite) → `lbm.run(10000u)`. Finite test run; can be set to `100u` for a smoke test or to a higher count for converged aerodynamics.
- **VTK output (after run):** added `lbm.{u,rho,flags,F}.write_device_to_vtk(export_path)` and `lbm.write_mesh_to_vtk(vehicle, export_path)` to `~/CFD/FluidX3D/export/`.
- **CSV force export (after run):** new block writes `forces_solid_cells.csv` with one row per solid cell that has non-zero force. Schema: `step,x,y,z,Fx_lbm,Fy_lbm,Fz_lbm,Fx_SI,Fy_SI,Fz_SI`. SI conversion via FluidX3D's `units.si_F(1.0f)`. ~690 k rows for the current setup; trivially aggregable with `awk` for drag/lift sums.
- **`_exit(0)` after exports:** workaround for an xe-driver bug. On Linux 7.0 / xe 26.05.037020, FluidX3D's regular C++ destructor cleanup triggers a chain of `[drm] Tile0: GT0: Fault response: Unsuccessful -EINVAL` followed by SIGSEGV in user space. Data are flushed to disk before `_exit(0)`, so no data loss. See README "Crash workaround" section for the full kernel-log signature.

## Build / repository hygiene

- `.gitignore`: added `export/`, `_portbackup/`, `*.vtk`, `*.csv`, `scenes` (the last is a symlink to the user's data drive and varies per machine).
- `README.md` → `README_UPSTREAM.md` (original FluidX3D documentation preserved unchanged).
- New `README.md` documents the B70 fork specifically.

## Verified performance (post-modification)

| Run | Cells | Steps | MLUPS | Bandwidth | Steps/s | Wall-time |
|---|---:|---:|---:|---:|---:|---:|
| Half-domain test | 16.85 M | 10 000 | **4 917** | **605 GB/s** | 292 | ~30 s |
| Full domain | 135.05 M | (run interrupted ~10 585) | 5 331 | 592 GB/s | 39 | n/a |

605 GB/s ≙ 99.5 % of the B70 spec (608 GB/s) — the LBM workload is fully bandwidth-limited and the GPU is saturating its memory subsystem.
