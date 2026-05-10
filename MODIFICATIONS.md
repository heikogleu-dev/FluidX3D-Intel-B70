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

Two distinct test setups have been built on top of the upstream Windkanal block:

### CC#1/CC#3 (small half-domain, 16.85 M cells, 10 000 steps) — preserved under `#if 0`

Original 1299 × 288 × 361 grid was axis-halved to 650 × 144 × 180. 10 000-step finite test run with full VTK + per-cell force CSV export. This block is **kept in the source under `#if 0` / `#endif`** so the small-grid smoke test can be re-enabled with a single line change.

- Grid: `uint3(650u, 144u, 180u)` (16.85 M cells, 6.92 mm cell size)
- `lbm.run(10000u)`
- VTK exports: u, rho, flags, F, mesh
- Per-cell CSV `forces_solid_cells.csv` (one row per non-zero TYPE_S cell)

### CC#2 (active) — Aero-Box 20 mm uniform, 145.92 M cells, 50 000 steps with aggregate force CSV

The currently-active `main_setup()` (line ~140 onwards):

- **Grid:** `uint3(1200u, 304u, 400u)` = **145.92 M cells**, cell size **20 mm uniform**, half-domain box 24 m × 6.08 m × 8 m. Symmetry plane on Y_min, vehicle X-center at cell index 350 (= 7 m from inlet).
- **Vehicle marker:** `lbm.voxelize_mesh_on_device(vehicle, TYPE_S | TYPE_X)` so `lbm.object_force(TYPE_S | TYPE_X)` isolates vehicle cells from floor / ceiling / outer wall (which only carry `TYPE_S`).
- **Boundaries:** moving floor at z=0 (TYPE_S, u=lbm_u in +x), no-slip ceiling (z=Nz-1) and outer wall (y=Ny-1), inlet/outlet on x=0/Nx-1 (TYPE_E with u=lbm_u). Vehicle cells protected from overwrite via TYPE_X check in the parallel-for.
- **Run loop:** 500 chunks × 100 steps = **50 000 steps total**. Per chunk: `lbm.run(100); lbm.update_force_field(); F = lbm.object_force(TYPE_S|TYPE_X); fcsv << F`. Aggregate force CSV with schema `step,t_si,Fx_si,Fy_si,Fz_si` — 500 rows total, much smaller than the per-cell CSV.
- **Final VTK:** u, rho, flags, F, plus mesh. ~4.2 GB total.
- **`_exit(0)`** after exports — same xe-driver workaround as CC#1/CC#3.
- **Verified runtime (CC#2 2026-05-10):** 24 min wall-time, 5384 MLUPS steady, 576 GB/s = **94.7 % of B70 spec bandwidth**, no xe-driver faults, rc=0 clean exit. See README "Performance baseline" for thermals (71 °C pkg / 84 °C VRAM-junction steady).

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
