# Modifications vs upstream ProjectPhysX/FluidX3D

**Fork base:** [ProjectPhysX/FluidX3D@4303cb4](https://github.com/ProjectPhysX/FluidX3D/commit/4303cb4) (FluidX3D v3.6, Jan 2026).

**Modifications:** Heiko Gleu, May 2026, for Intel Arc Pro B70 (BMG-G31, xe driver) on Linux. Verified runtime: oneAPI OpenCL 26.05.037020, GCC 15.2, kernel 7.0.

**Scope:** **Single-GPU only.** Multi-GPU (domain decomposition `LBM(N, Dx, Dy, Dz, nu)` with Dx·Dy·Dz>1) is intentionally not supported in this fork. The TYPE_Y symmetry-plane patch in CC#7 has not been validated for the halo-exchange path that multi-GPU requires.

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

### CC#7 (current active default) — Halbdomain mit echter Symmetrieebene (TYPE_Y specular reflection)

CC#7 introduces a new boundary-condition class `TYPE_Y` (bit 7, was reserved upstream) for specular-reflection symmetry planes. CC#6 had shown empirically that `TYPE_E` is **not** a valid Y=0 symmetry approximation: half-domain Fx (16 177 N) was ~7× the half of full-domain Fx (2 219 N / 2). CC#7 fixes this with a true reflection BC.

**Implementation (D3Q19 only):**

In `kernel.cpp`:
- `stream_collide` kernel: after `load_f()`, before MOVING_BOUNDARIES — TYPE_Y cells get DDF y-mirror swap then `store_f()` + `return`. Y-mirror DDF pairs (5 pairs in D3Q19): (3,4), (7,13), (8,14), (11,18), (12,17). All other DDFs (no y-component) are unchanged.
- `update_fields` kernel: TYPE_Y cells skipped (`if(flagsn_bo==TYPE_S||flagsn_su==TYPE_G||(flagsn&TYPE_Y)) return;`). rho/u of TYPE_Y cells stay at init values.

In `setup.cpp`:
- New compile-time toggle `#define CC6_MODE` with values `0` (CC#6-Half TYPE_E), `1` (CC#6-Full reference), `2` (CC#7-Half TYPE_Y).
- Mode 2: Y_min cells get `TYPE_Y` instead of `TYPE_E`, everything else identical to CC#6-Half (10 mm, 1500×250×450 = 168.75 M cells, vehicle X-center @ cell 400).

**Bit topology (no conflicts):**
| Mask | Bits | Members |
|---|---|---|
| TYPE_BO | 0x03 | TYPE_S, TYPE_E, TYPE_MS=TYPE_S\|TYPE_E |
| TYPE_T  | 0x04 | TYPE_T |
| TYPE_SU | 0x38 | TYPE_F, TYPE_I, TYPE_G |
| user    | 0xC0 | TYPE_X (vehicle marker), **TYPE_Y (sym-plane)** |

TYPE_Y (0x80) lies outside TYPE_BO+TYPE_SU, so it can be set additionally to TYPE_MS (e.g. when a sym-plane cell is adjacent to a moving-wall floor) without breaking the bounceback/moving-boundary logic — the TYPE_Y check in `stream_collide` runs *before* the MOVING_BOUNDARIES check.

**Auto-Stop:** simplified to `|dFx|/|Fx| < 2 %` (Fy/Fz omitted — both are near zero in the full-domain reference and not relative-convergeable; the goal is to match Fx within 10% of full-domain/2).

**Force CSV:** `bin/forces_cc7_half_sym.csv`.

**Validation criterion:** CC#7-Half × 2 must match CC#6-Full within ±10 % on Fx. Reference: CC#6-Full Fx = 2 219 N (50-chunk average over steps 28200-33100). Target window: CC#7-Half Fx ∈ [999, 1221] N.

**First-pass result (2026-05-10): FAILED.** CC#7-Half stabilised at Fx ≈ 14 472 N (50-chunk avg over steps 7200-12100, |dFx|=2.27 %, killed before convergence). That is **× 13 over target** and only ~10 % below CC#6-Half (TYPE_E pseudo-sym, 16 177 N).

**Alt 1 (TYPE_S Moving-Wall at Y_min, CC6_MODE=3, 2026-05-10): FAILED, Fx ≈ 17 736 N** — even higher than TYPE_E. Y_min as no-slip wall (with u=(lbm_u, 0, 0) for symmetry-conformity in u_y, u_z) creates an unwanted boundary layer in u_z (which should equal the interior u_z, not zero).

**Alt 3 — Diagnostic root cause (2026-05-10):** TYPE_Y cell counter shows 650 654 / 673 500 (97 %) cells correctly marked at Y_min plane (the small deficit is the Vehicle-Y_min overlap region where TYPE_X cells take precedence). So the setup-branch is correct.

**Root cause is in the kernel.cpp swap interaction with FluidX3D's Esoteric-Pull DDF storage:**

In Esoteric-Pull, opposite-direction pairs (i, i+1) are stored alternating between self-cell and neighbor-cell, with parity-dependent slot indices. `load_f(n, fhn, fi, j, t)` reads:
- fhn[i]   from self n   at slot t%2?i:i+1   (for odd i)
- fhn[i+1] from neighbor j[i] at slot t%2?i+1:i

`store_f` writes the post-collision DDFs back symmetrically. **For an EP-opposite pair like (3, 4)**, the swap-and-store_f sequence ends up **writing the same values back to the same memory locations** — net effect = no-op:
- fhn[3] read from self-slot-A, fhn[4] read from j[3]-slot-B
- swap: fhn[3] := old V_at_j[3]_slot_B, fhn[4] := old V_at_self_slot_A
- store_f: fhn[3] → j[3]-slot-B (same location it was loaded from!), fhn[4] → self-slot-A (same location)

For **non-EP-opposite Y-mirror pairs (7, 13), (8, 14), (11, 18), (12, 17)**, the swap-and-store does move data, but to the **wrong neighbor cells**: e.g. fhn[13] (representing direction (+1, -1, 0)) gets written to j[13] = (+x, -y) neighbor, which at a Y_min sym-plane cell is OUTSIDE the half-domain (y = -1). FluidX3D wraps this periodically to y = Ny-1 (a TYPE_E cell), where the value is immediately overwritten by the equilibrium-boundary forcing.

Net effect: the swap produces ~10 % drag reduction (a partial perturbation of the Y_min flow) but no actual specular-reflection symmetry plane.

**A correct CC#7 implementation would require a separate kernel that runs after stream_collide, reads DDFs from neighbors directly, and writes the specular-reflected values without using EP storage layout.** Deferred until a future iteration.

### Session 2026-05-11: CC#7-V2 + CC#8 — confirmed architectural dead-end

**CC#7-V2 (TYPE_Y one-way assignment instead of swap, 2026-05-11):** Re-implemented the inline-in-`stream_collide` TYPE_Y patch using OpenLB's `SlipBoundaryProcessor3D` pattern: `fhn[iPop] = fhn[reflectionPop[iPop]]` (one-way assignment from "into-fluid" to "from-fluid" mirror direction). **Result: Fx = 14 386.4 N at step 5000 — bit-identical to CC#7-V1 swap result.** This proves that the Esoteric-Pull storage layout effectively cancels ANY inline DDF modification at a single cell: regardless of whether we swap or assign, the `load_f` → modify → `store_f` round-trip writes the values back to slots that have already been read from the same cells (with parity-dependent slot indexing), producing memory state equivalent to the unmodified case for the (3,4) EP-opposite pair, while non-EP-opposite Y-mirror pairs scatter their values to wrong neighbor cells (the y=-1 periodic-wrap problem).

**CC#8 (Ghost-Cell-Mirror via TYPE_E|TYPE_Y, 2026-05-11):** Different approach — instead of touching DDFs, hook into FluidX3D's existing `TYPE_E` `EQUILIBRIUM_BOUNDARIES` branch in `stream_collide`. Sym-plane cells get `TYPE_E | TYPE_Y` flag. In the kernel, when computing equilibrium DDFs for a TYPE_E cell, if the TYPE_Y bit is set, the input `(rho, u_x, u_y, u_z)` is taken from the +y neighbor's velocity array with `u_y → -u_y` (mirror). This is race-tolerant (1-step delay is acceptable for steady-state aero) and avoids EP storage interaction entirely. **Result: Fx converges to ~14 200-14 500 N — about 12 % lower than plain TYPE_E (CC#6-Half = 16 177 N) but still 13× the target.** The reason: equilibrium-projection at the sym-plane is fundamentally a velocity-Dirichlet boundary condition, not a specular reflection. It enforces `(rho, u)` consistent with a mirror state but loses the higher-moment shape of the incoming DDF distribution — the very thing that specular reflection should preserve.

### Architectural verdict

After five distinct approaches (TYPE_E pseudo-sym, TYPE_Y inline swap, TYPE_Y inline assignment, TYPE_S Moving-Wall, TYPE_E|TYPE_Y Ghost-Cell-Mirror), all producing Fx in the 14 000-17 700 N range against a Volldomain reference of 2 219 N, we conclude: **a true specular-reflection symmetry plane is architecturally not feasible in FluidX3D without rewriting the Esoteric-Pull storage layer or adding a separate boundary-handler pass with parity-aware slot access.** This matches the upstream maintainer's own statement in [issue #288](https://github.com/ProjectPhysX/FluidX3D/issues/288): *"I'm stuck with the literal corner case in 3D where specular reflection of DDFs reflect them from space diagonals to axis-aligned directions … no solution to this in literature either."*

Production default reverted to `CC6_MODE=1` (Volldomain reference, Fx = 2 219 N). The deprecated TYPE_Y inline patch is preserved under `#if 0` in `kernel.cpp:1488` for documentation. CC#8 ghost-mirror remains available as `CC6_MODE=4` for users who prioritise compute time over correctness (~12 % drag reduction vs plain TYPE_E, but still order-of-magnitude wrong).

**Reference implementations researched (2026-05-10):**

- **waLBerla `FreeSlip::treatDirection()`** ([source](https://github.com/lssfau/walberla/blob/master/src/lbm/boundary/FreeSlip.h)): operates as a **post-stream-step boundary handler** (not inline in the main kernel). For each direction `dir` from a fluid cell `n` to a boundary cell `b`, the algorithm:
  1. Detects the wall normal by checking which neighbor of `b` is fluid (`isPartOfMaskSet`).
  2. Looks up the reflected direction `ref_dir = mirrorX/Y/Z[inverseDir[dir]]` from precomputed tables.
  3. Copies `pdfField(n, invDirIdx(dir)) = pdfField(n + wnx, n + wny, n + wnz, idx[ref_dir])` — i.e. the reflected PDF source is in a fluid *neighbor* of `n` along the wall-tangent direction, not in `n` or `b` itself.
- **OpenLB `addSlipBoundary`**: same algorithmic family — specular reflection in post-stream BC handlers. Initially 2D-only, later extended to D3Q15/19/27 stencils.

The architectural pattern is consistent across both: **boundary handlers run after the main collide-stream kernel and read/write PDFs in fluid cells using stencil-aware lookup tables**. FluidX3D's Esoteric-Pull-coupled `apply_moving_boundaries` is *inline* in stream_collide and only handles velocity-Dirichlet, not specular reflection. For CC#7-V2, the correct architecture is to follow the waLBerla pattern: a new OpenCL kernel `apply_freeslip_y` running over the fluid cells at `y=1` (one above the sym-plane), with a precomputed `y_mirror_idx[19]` lookup table.

The TYPE_Y kernel patch, the CC6_MODE=2 setup branch, and the diagnostic counter remain in the source tree under `#define CC7_DIAGNOSE`. Production default is set to **CC6_MODE=1 (Volldomain reference)**.

### CC#6 (deactivated, reference baseline) — Aero-Box 10 mm, 168.75 M / 337.5 M cells, Auto-Stop on <1% force-drift

The currently-active `main_setup()` is a compile-time-toggleable Half/Full-Domain block (line ~197 onwards), driven by `#define CC6_FULL_DOMAIN false|true`:

- **Half-domain grid:** `uint3(1500u, 250u, 450u)` = **168.75 M cells** (Y[0, 2.5m] sym at Y_min)
- **Full-domain grid:** `uint3(1500u, 500u, 450u)` = **337.5 M cells** (Y[-2.5, +2.5m], no symmetry)
- **Cell size:** 10 mm uniform; X = 15 m (X[-4, +11]); Z = 4.5 m
- **Vehicle X-center at cell 400** (= 4 m from inlet, 11 m to outlet — more upstream room than CC#3-CC#5)
- **Vehicle Z-min = 1 cell** (wheels on ground, 10 mm sub-cell offset for moving-floor BC)
- **Walls (FluidX3D-Aero convention):** floor=`TYPE_S` Moving-Wall +x (rolling road); ceiling/Y_min/Y_max/inlet/outlet=`TYPE_E` free-stream u_x=lbm_u
- **Vehicle marker:** `voxelize_mesh_on_device(vehicle, TYPE_S|TYPE_X)`. `object_force(TYPE_S|TYPE_X)` filter is **exact-match** in `kernel.cpp:1941` (`flags[n]==flag_marker`), so it correctly isolates vehicle cells from floor/walls (which only carry `TYPE_S` resp. `TYPE_E`).
- **Auto-Stop:** sliding-window of 5000 steps (50 chunks × 100). Convergence test: `|Fx_recent_avg − Fx_prev_avg| / |Fx_recent_avg| < 1 %` AND same for Fy. Earliest exit: 10 000 steps. Hard cap: 100 000 steps.
- **Force CSV:** `bin/forces_cc6_half.csv` or `bin/forces_cc6_full.csv` with schema `step,t_si,Fx_si,Fy_si,Fz_si`.
- **Final VTK exports:** u, rho, flags, F, mesh.
- **`_exit(0)`** after exports (xe-driver cleanup-race workaround).

### CC#5 (previous, deactivated) — Aero-Box 10 mm half-domain, TYPE_E walls, 50k steps fixed-length

CC#5 was the first run with the FluidX3D-correct boundary configuration (TYPE_E free-stream walls) after CC#4 had been broken by all-walls TYPE_S Moving Wall (which created an artificial 3D Couette flow and gave Fx = 14749 N). CC#5 result: **Fx = 16 180 N steady, 5464 MLUPS, 585 GB/s, ~7.8 min wall-time at 202.5 M cells**. Force still 50-80× higher than OpenFOAM reference — diagnosed cause is **missing wall model** (FluidX3D's own Ahmed-body comment: "expect Cd to be too large by a factor 1.3-2.0x; need wall model"). CC#5 grid was `uint3(1500u, 300u, 450u)` (15m × 3m × 4.5m), Vehicle X-center at cell 300, Z-min at cell 1.

### CC#3 (deactivated) — Aero-Box 20 mm asym, 50 M cells, 50 000 steps, half-domain symmetry fix applied

The currently-active `main_setup()` (line ~197 onwards):

- **Grid:** `uint3(1000u, 200u, 250u)` = **50.0 M cells**, cell size **20 mm uniform**, half-domain asym box **20 m × 4 m × 5 m** (X[-5, +15], Y[0, 4], Z[0, 5]). Symmetry plane on Y_min. Vehicle X-center at cell index **250** (= 5 m from inlet, 13 m wake).
- **Vehicle position (post-bug-fix):** `vehicle->translate(float3(250 - vctr.x, 0 - vctr.y, 1 - (vctr.z - vbbox.z * 0.5f)))`. The Y translation puts the vehicle **center on Y=0** so the symmetry plane slices it down the middle — only the Y≥0 half is voxelized. Sanity-check `if(vmin.y > 0.5f || vmax.y < -0.5f) _exit(2)` aborts the run before voxelization if the half-domain geometry is wrong (ensures the CC#2 bug cannot recur silently).
- **Vehicle scale:** `si_length = 4.5 m` (corrected post-CC#3 from 4.0 m to actual geometry — real vehicle is 4.5 m × 1.8 m × 1.1 m).
- **Vehicle marker:** `lbm.voxelize_mesh_on_device(vehicle, TYPE_S | TYPE_X)` so `lbm.object_force(TYPE_S | TYPE_X)` isolates vehicle cells from floor / ceiling / outer wall (which only carry `TYPE_S`).
- **Boundaries:** moving floor at z=0 (TYPE_S, u=lbm_u in +x), no-slip ceiling (z=Nz-1) and outer wall (y=Ny-1), inlet/outlet on x=0/Nx-1 (TYPE_E with u=lbm_u). Vehicle cells protected from overwrite via TYPE_X check in the parallel-for.
- **Run loop:** 500 chunks × 100 steps = **50 000 steps total**. Per chunk: `lbm.run(100); lbm.update_force_field(); F = lbm.object_force(TYPE_S|TYPE_X); fcsv << F`. Aggregate force CSV `bin/forces_cc3.csv` with schema `step,t_si,Fx_si,Fy_si,Fz_si` — 500 rows total.
- **Final VTK:** u, rho, flags, F, plus mesh. ~1.4 GB total at 50 M cells.
- **`_exit(0)`** after exports — same xe-driver workaround as CC#1.
- **Verified runtime (CC#3 2026-05-10):** **7.8 min wall-time**, 5732 MLUPS steady, **613 GB/s = 100.8 % of nominal B70 spec bandwidth** (the spec is conservative on this card at this working-set size), no xe-driver faults, rc=0 clean exit. Force convergence statistical from step ~7500 (Fx ±5.8 % over last 20k steps). Absolute force values still 3.8× higher than OpenFOAM reference — see README "Force-field runs status" for the suspected causes (symmetry boundary, ground clearance, resolution).

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
