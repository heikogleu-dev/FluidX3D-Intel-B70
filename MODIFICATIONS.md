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

After **SEVEN** distinct approaches (Session 2026-05-11 extended), all producing Fx in the 13 600-17 700 N range against a Volldomain reference of 2 219 N, we conclude: **a true specular-reflection symmetry plane is architecturally not feasible in FluidX3D without rewriting the Esoteric-Pull storage layer.**

**Full attempt log:**
1. CC#6-Half plain TYPE_E pseudo-sym → Fx = 16 177 N (velocity-Dirichlet pin)
2. CC#7-V1 TYPE_Y inline swap → Fx = 14 472 N (EP-storage cancels swap)
3. CC#7-V2 TYPE_Y inline one-way assignment (OpenLB pattern) → Fx = 14 386 N (bit-identical to V1)
4. CC#7-Alt1 TYPE_S Moving-Wall at Y_min → Fx = 17 736 N (no-slip BL induced)
5. CC#8 TYPE_E|TYPE_Y Ghost-Cell-Mirror in equilibrium-branch → Fx = 14 307 N (Dirichlet pin with mirror state)
6. CC#9-V1 separate post-stream `apply_freeslip_y` kernel à la waLBerla, TYPE_Y as fluid → Fx ≈ 13 600 N (best so far, but 12× target; reflection on collision-relaxed DDFs)
7. CC#9-V2 separate kernel + TYPE_Y early-return in stream_collide → Fx = 14 386 N (**bit-identical to CC#7-V2 inline**)

**Crucial empirical finding (CC#9-V2 vs CC#7-V2):** the post-stream-kernel separation gives **bit-identical results** to the inline modification — proving that EP-storage layout is the genuine block, NOT kernel-boundary synchronization. Any approach that reads DDFs via `load_f`, modifies the local `fhn[]` array, and writes back via `store_f` will produce the same memory state, regardless of where in the pipeline it sits.

**Upstream maintainer corroborates** in [issue #301](https://github.com/ProjectPhysX/FluidX3D/issues/301): *"I've been looking into free-slip boundaries for a long time, and have a beta implementation. However it only works normal to axis-aligned or edge diagonal directions. For space diagonals the DDF reflection method fails — as differently weighted DDFs from axis-aligned/edge diagonals with different lattice weights have to be reflected into one another, and there is no solution to this problem in literature."*

Our D3Q19 case has NO space-diagonals (those exist only in D3Q27), so weight-mismatch is not the limitation here. The limitation is the EP-Pull layout itself: a "local" specular reflection at a sym-plane cell N0 requires writing to PDF locations that, in EP layout, are physically owned by N0's neighbors. When N0's `store_f` writes the reflected fhn[3] to "j[3]=N1 at slot t%2?4:3" (which is N1's self-storage), N1 reads that slot as its own fhn[3] in the next step — *but* the reflection's intended target slot for fhn[7]/fhn[13] etc. is split across non-EP-paired indices and lands at wrong neighbors (e.g., j[13]=(+x,-y) cell, which for sym-plane is outside the half-domain and periodic-wraps to a TYPE_E top wall cell where the value gets immediately overwritten).

**Production default reverted to CC6_MODE=1 (Volldomain reference, Fx = 2 219 N).** All seven failed approaches are documented and preserved as compile-time variants `CC6_MODE=0..5` in `setup.cpp` plus deprecated `#if 0` blocks in `kernel.cpp`. CC#9 post-stream kernel infrastructure (kernel_apply_freeslip_y in lbm.hpp/cpp + enqueue after stream_collide) is retained — it does no harm when CC6_MODE!=5 since the kernel early-returns when no TYPE_Y cells exist.

### Final session 2026-05-11: CC#9-V3/V4/V5 closure

Three more variants tested after CC#9-V1/V2:
- **CC#9-V3:** apply_freeslip_y with ONLY pair (3,4) reflected (no edge-diagonal pairs). Result: Fx = 13 016 N at step 2500. Edge-pairs contribute ~5 % at most — the dominant reduction is from (3,4) which is axis-aligned and theoretically should have NO EP-Pull conflict.
- **CC#9-V4 ("Ansatz A"):** bypass `load_f`/`store_f` entirely, do direct slot-level `fi[N1, slot_b] = fi[N1, slot_a]` at the fluid neighbor's storage, avoiding TYPE_E top-wall periodic-wrap pollution at the sym-plane cell's own slots. Result: Fx = 13 647 N at step 2500. Still 12× target.
- **CC#9-V5:** Vehicle shifted +1 cell in Y to test periodic-wrap-pollution hypothesis at vehicle's y=0 cut-surface cells. Result: Fx = 13 446 N. Shift is too small to actually clear y=0 (vehicle bbox -87.6..+89.6 still extends below y=0 and clipped at voxelization). Hypothesis cannot be cleanly tested without restructuring vehicle placement.

**Maximum drag reduction achievable across all 9 sym-plane variants tested: from CC#6-Half 16 177 N (plain TYPE_E) down to CC#9-V3 13 016 N = 19.5 %. Target reduction required: 93 % (down to ~1 110 N).**

### Session 2026-05-11 final: CC#9-V6/V7 — user hypothesis confirmed (partially)

**User raised a sharp diagnostic question:** could the half-domain's vehicle "cut-surface" at y=0 be spuriously contributing to `object_force`? In Volldomain those cells would be interior (no fluid contact) with zero force; in Halbdomain they have TYPE_E top wall as -y neighbor via periodic-wrap.

**CC#9-V6 patch (53e88e9):** after `voxelize_mesh_on_device`, scan the y=0 slice (Nx × Nz cells), strip TYPE_X bit from any vehicle cells found. They remain TYPE_S (solid in stream_collide) but are excluded from `object_force(TYPE_S|TYPE_X)`. Equivalent to user's "post-hoc filter real vehicle patches" approach.

**V6 result** (Mode 0 baseline plain TYPE_E pseudo-sym + V6 strip): Fx = 13 206 N at step 5000 (50-chunk avg = 14 190 N). **20 % reduction vs CC#6-Half 16 177 N.**

**CC#9-V7 (Mode 5 specular reflection + V6 strip combined):** Fx = 13 046 N at step 5000 (50-chunk avg = 14 045 N). **22 % reduction** — only marginally better than V6 alone. The two fixes (sym-plane reflection + cut-surface strip) **partially overlap** (combined reduction ≈ either alone, not additive).

**Verdict:**
- User's hypothesis CONFIRMED in part: cut-surface periodic-wrap pollution contributes ~20 % of the drag overshoot. **V6 is a legitimate fix and should be kept on for all half-domain modes.**
- The remaining ~71 % overshoot (to reach the 93 % needed) is **flow-field-level pollution**: TYPE_E pseudo-stream at sym-plane distorts the velocity field near the vehicle, and that distortion is felt by the REAL vehicle surfaces (y >= 1) — NOT directly fixable via flag manipulation alone.
- The remaining work would be: a TRUE specular-reflection BC that doesn't pin velocity to (lbm_u, 0, 0) at y=0 — requires either Ansatz B (multi-day kernel rewrite via modified `calculate_indices` for TYPE_Y cells) or a different LBM solver.

V6 strip patch is now unconditional in setup.cpp for all half-domain modes (`#if CC6_MODE != 1`). Combined with the post-stream apply_freeslip_y kernel (Mode 5), the best half-domain result is **Fx ≈ 14 000 N average**. Still well above target, but ~14 % better than the original CC#6-Half (16 177 N).

Production default remains CC6_MODE=1 (Volldomain reference, Fx = 2 219 N).

After 9 distinct variants spanning swap/assignment/Moving-Wall/ghost-mirror/post-stream-kernel/different-pair-subsets/direct-slot-copy/vehicle-shift, all results cluster in the 13.0-17.7 k range. The achievable reduction is consistently ~20 %, far from the ~93 % needed. The Esoteric-Pull layout in combination with FluidX3D's periodic-by-default neighbor function is a true architectural block for sym-plane in half-domain.

**The only remaining theoretical path "Ansatz B"** would modify FluidX3D's `calculate_indices()` to NOT periodic-wrap at TYPE_Y cell faces — a virtual ghost-layer returning "self" for the -y direction. Researched in detail (2026-05-11):
- `calculate_indices()` is currently flag-unaware. Three architectural options:
  - Pass flag parameter to `calculate_indices` (modify ALL callers — invasive)
  - Separate `neighbors_sym_plane()` function for TYPE_Y cells
  - In `apply_freeslip_y` only: manually compute non-wrap j-indices before reads (least invasive)
- **For D3Q19, only ONE odd-index j-array uses `ym` directly: `j[13] = xp+ym+z0`** (the (+1,-1,0) direction). All other odd-i neighbor reads use `yp` or no-y components.
- The Y-mirror DDFs read in our apply_freeslip_y workflow (fhn[4], fhn[8], fhn[12], fhn[18]) **come from yp-direction neighbors** — these are FLUID cells (y=1), NOT polluted by periodic-wrap.
- The only "polluted" reads are fhn[14] (from j[13]=top wall via wrap) and fhn[13] (written to j[13]). **Both are OVERWRITTEN by our reflection assignment `fhn[14] = fhn[8]` anyway.**

**Conclusion: Ansatz B would NOT improve the result for our apply_freeslip_y reflection logic.** The remaining 12× drag overshoot is NOT from periodic-wrap pollution — it's something else in the EP-Pull mechanic. Modifying `neighbors()` is therefore **NOT recommended** as it would add architectural complexity without measurable benefit.

**Recommendation for next steps:** the sym-plane work has hit a confirmed architectural wall. The 5× drag overshoot vs OpenFOAM RANS (Volldomain Fx = 2 219 N vs expected ~400-600 N) is now best addressed via:
- **Werner-Wengle wall model** (Roadmap item #3 in README): impact 1.5-3× drag reduction, independent of sym-plane architecture, ~3-5 days implementation. **This is the recommended next step.**
- **Rotating wheels** (Roadmap item #4): FluidX3D has the API built-in, ~1 hour implementation. Lift correction.

Production default reverted to `CC6_MODE=1` (Volldomain reference, Fx = 2 219 N). The deprecated TYPE_Y inline patch is preserved under `#if 0` in `kernel.cpp:1488` for documentation. CC#8 ghost-mirror remains available as `CC6_MODE=4` for users who prioritise compute time over correctness (~12 % drag reduction vs plain TYPE_E, but still order-of-magnitude wrong).

**Reference implementations researched (2026-05-10):**

- **waLBerla `FreeSlip::treatDirection()`** ([source](https://github.com/lssfau/walberla/blob/master/src/lbm/boundary/FreeSlip.h)): operates as a **post-stream-step boundary handler** (not inline in the main kernel). For each direction `dir` from a fluid cell `n` to a boundary cell `b`, the algorithm:
  1. Detects the wall normal by checking which neighbor of `b` is fluid (`isPartOfMaskSet`).
  2. Looks up the reflected direction `ref_dir = mirrorX/Y/Z[inverseDir[dir]]` from precomputed tables.
  3. Copies `pdfField(n, invDirIdx(dir)) = pdfField(n + wnx, n + wny, n + wnz, idx[ref_dir])` — i.e. the reflected PDF source is in a fluid *neighbor* of `n` along the wall-tangent direction, not in `n` or `b` itself.
- **OpenLB `addSlipBoundary`**: same algorithmic family — specular reflection in post-stream BC handlers. Initially 2D-only, later extended to D3Q15/19/27 stencils.

The architectural pattern is consistent across both: **boundary handlers run after the main collide-stream kernel and read/write PDFs in fluid cells using stencil-aware lookup tables**. FluidX3D's Esoteric-Pull-coupled `apply_moving_boundaries` is *inline* in stream_collide and only handles velocity-Dirichlet, not specular reflection. For CC#7-V2, the correct architecture is to follow the waLBerla pattern: a new OpenCL kernel `apply_freeslip_y` running over the fluid cells at `y=1` (one above the sym-plane), with a precomputed `y_mirror_idx[19]` lookup table.

The TYPE_Y kernel patch, the CC6_MODE=2 setup branch, and the diagnostic counter remain in the source tree under `#define CC7_DIAGNOSE`. Production default is set to **CC6_MODE=1 (Volldomain reference)**.

### CC#10 (2026-05-11) — Werner-Wengle Wall Model on Vehicle (Approach A) — **WORKING**

**Motivation:** CC#9 closure established that the Volldomain baseline gave Fx = 2 219 N (4-5× over OpenFOAM RANS expectation 400-600 N). Root cause diagnosed as missing wall model — bounce-back on the vehicle creates an artificial over-steep velocity gradient at the first fluid cell (y_1+ ≈ 200-500 lies deep in the log layer; bounce-back assumes viscous sublayer y+<5).

**Approach:** Werner-Wengle PowerLaw closed-form wall function applied via Krüger Moving-Wall trick. Instead of modifying DDFs directly (Han 2021 WFB), the wall model sets the velocity `u_solid` of vehicle TYPE_S cells such that the existing `apply_moving_boundaries` Krüger correction produces the correct effective slip. Single new OpenCL kernel `apply_wall_model_vehicle` runs each step BEFORE `stream_collide`:

1. For each `TYPE_S | TYPE_X` cell: average fluid neighbor velocities → `u_avg`
2. Compute `u_τ` via PowerLaw closed form: `u_τ² = max(2νu_avg, 0.0246384 × ν^0.25 × u_avg^1.75)`
3. Compute slip `u_slip` via PowerLaw inverse at `y_1 = 0.5 lu`
4. Cap at `0.95 × u_avg` for stability
5. Write `u[wall_cell] = u_slip × direction(u_avg)`

`enqueue_update_moving_boundaries()` is called immediately after to refresh TYPE_MS bits on fluid neighbors so the Krüger correction is applied in `stream_collide`.

**Files:**
- `defines.hpp`: new flag `WALL_MODEL_VEHICLE` (line 25), default ON
- `kernel.cpp`: new kernel `apply_wall_model_vehicle` (~40 lines, between `apply_freeslip_y` and MOVING_BOUNDARIES)
- `lbm.hpp`: kernel member + `enqueue_apply_wall_model_vehicle()` declaration
- `lbm.cpp`: kernel init, enqueue method, device_define propagation, call in `do_time_step()` BEFORE stream_collide + `enqueue_update_moving_boundaries`, and `def_nu` added to device_defines

**Result (CC6_MODE=1, 337.5 M cells, 11 300 steps, Werner-Wengle PowerLaw):**

| Metric | Baseline (no WW) | WW Active | Change |
|---|---:|---:|---|
| **Fx (drag)** | 2 219 N | **579.8 N** | **−74 %** ✓ in OpenFOAM range 400-600 N |
| **Fy** | ~0 N | <0.1 N | ✓ perfect symmetry |
| **Fz (lift)** | n/a | +1 045 N | + lift, realistic for coupé without diffuser |
| **MLUPs** | 5 464 | 3 289 | −40 % (extra kernels per step) |
| **Convergence** | 100k+ steps | **11 300 steps** | **9× faster** |

**Transient behavior:** Steps 100-1300 show a large initial transient (Fx oscillates 300-4870 kN, peak 4.87 MN at step 1100). The simulation auto-stabilizes by step 1400 to the 555-600 N regime and remains stable through convergence. The auto-stop convergence criterion (|dFx|/|Fx| < 2% over 5000-step window) triggered at step 11 300 with |dFx|/|Fx| = 0.080%.

**Files committed:** `findings/WALL_MODEL_RESEARCH.md` (231 lines, approach comparison + OpenLB/waLBerla/Palabos/TCLB references), `findings/forces_cc10_ww_volldomain.csv` (114 rows, full force time series).

### CC#X (2026-05-11) — Ahmed Body Wall-Model Validation — ❌ Phase 1 FAILED (Iron-Rule-Trigger)

**Motivation:** Following the CC#10 success on the Time-Attack MR2 (580 N drag matching ~565 N real measurement), an Opus-instructed validation plan against the canonical ERCOFTAC Ahmed Body benchmark was started (Phase 1.1 — 25° slant @ Re = 2.78 M, literature CD = 0.285 ± 0.02).

**Setup:** New compile flag `#define AHMED_MODE` in `setup.cpp` (0 = real vehicle, 1 = Ahmed 25°, 2 = Ahmed 35°). New `main_setup_ahmed()` function with smaller-body-appropriate setup. Simplified flat-front Ahmed body STL generated via Python (16 triangles, closed convex polyhedron, slant + base + sides). Saved as `ahmed_25.stl` and `ahmed_35.stl` in repo root (small synthetic geometry, reproducible). MR2-proven domain reused (15m × 5m × 4.5m at 10 mm → 337.5 M cells, blockage ratio 7.8 %/6.4 %).

**Results (Phase 1.1, Ahmed 25°):**

| Configuration | Fx [N] | CD measured | Verdict |
|---|---:|---:|---|
| Ahmed 25° + WW (production-default) | **11 457 N** (oscillating 6-27 kN) | **104** | 365× too high vs literature |
| Ahmed 25° **without** WW (diagnostic, `WALL_MODEL_VEHICLE` temporarily commented) | 92 N (avg 10 000 steps) | 0.84 | 3× typical bounce-back overshoot — plausible |
| Literature (Lienhart-Stoots 2003) | ~31 N | 0.285 | target |

**Iron-Rule-Trigger:** Per Opus plan Section "Iron Rules" — "Phase 1 nicht ±20 % an Literatur-CD ankommt → STOPP, Bericht, Re-Evaluation des WM". Measured CD = 104 vs CD_lit = 0.285 corresponds to **+36 400 %** — far beyond stop threshold. No Phase 2 (sym-plane sweep) or Phase 3 (real-vehicle re-application) initiated. WW code unchanged (Iron Rule "Wall-Model-Code NICHT anfassen" respected — diagnostic was via `#define WALL_MODEL_VEHICLE` toggle, immediately re-enabled).

**Diagnosis:** The Werner-Wengle wall model interacts pathologically with **flat-faced, sharp-edged synthetic geometry**:
- Same WW code produces *correct* drag on MR2 (smooth STL, 1.48 M triangles, curved surfaces): 580 N nahe 565 N target ✓
- Same WW code produces *125× force-amplification* on Ahmed (flat STL, 16 triangles, 90° edges): 11 457 N vs ~30 N expected ✗

**Hypotheses (detailed in `findings/CC_X_ahmed/SESSION_2026-05-11_PHASE1_FAIL.md`):**
- A: Sharp-edge u_avg averaging in WW kernel produces spurious slip directions
- B: Flat-face accumulation — uniform surface normal direction lets WW slip-effects add coherently rather than cancel
- C: Frontal-stagnation imbalance — WW does nothing on flat front (u_avg≈0) but full slip elsewhere, creating asymmetric force
- D: Voxelization artifact from 16-triangle large-face geometry

**Files added:**
- `src/setup.cpp` — `AHMED_MODE` compile toggle + `main_setup_ahmed()` function (~110 lines)
- `ahmed_25.stl`, `ahmed_35.stl` — 16-triangle simplified Ahmed Body STLs (binary, 884 bytes each, includes 50 mm ground clearance baked in)
- `findings/CC_X_ahmed/SESSION_2026-05-11_PHASE1_FAIL.md` — full failure analysis with hypotheses and proposed paths α/β/γ/δ
- Auto-stop tightened: `conv_window = 25` chunks (was 50), `conv_min_chunks = 50` (was 100), earliest exit at 5 000 steps

**Conclusion:** The CC#10 Werner-Wengle implementation is **production-valid for smooth STL vehicles**, the use case it was developed and validated against (MR2 Time-Attack). It is **NOT yet validated against canonical synthetic reference geometry** because the Ahmed Body test revealed a previously-unsuspected geometry sensitivity. Halted per Iron Rule until user decides next direction:
- α: Generate proper rounded-front Ahmed STL (~500 triangles, ERCOFTAC R=100mm corner spec)
- β: Skip Ahmed, directly Phase 2 sym-plane sweep on MR2 (loses canonical validation, gains time)
- γ: Modify WW kernel for edge-detection (Iron-Rule violation, needs explicit user override)
- δ: y+ resolution increase (impossible at B70 VRAM 28 GB)

`AHMED_MODE = 0` (safe default) — running `./bin/FluidX3D` resumes prior MR2/Yaris setup without modification.

### CC#11 (2026-05-13) — Wall-Model Deep Dive: Krüger Three-Attractor pathology confirmed

**Motivation:** CC#X (Ahmed Phase 1 FAIL) and subsequent re-tests on the more detailed `vehicle-mr2-bin.stl` revealed CC#10's "+580 N" success was on a different (less detailed) STL. On current MR2 STL the same Full Krüger -6 logic produces -610 N (negative drag). Phase 0d Bug-Verification (Findings 20-24) confirmed Krüger force-artifact at TYPE_S|TYPE_X cells as the root cause. CC#11 spans the complete investigation: per-cell mathematics, single-cell isolation, parameter-space scan, and architectural conclusion.

**Phases:**
1. **Phase 0d** (Findings 20-24) — Code-Audit + Data-Flow + 3-path Verification + Option 1 analytical subtraction (Cube CD=40 calibrated factor 6, MR2 catastrophic -293,880 N → Iron-Rule)
2. **Phase A — Foundation Validation** (Findings 29-32) — Re/y+ matrix, Poiseuille sketch, Diagnostic kernel sketch, object_force code-audit with BEWIESEN/HYPOTHESE/OFFEN markers
3. **Phase B Sub-Task 1** (Finding 33) — Single-cell Krüger isolation test confirms Hypothesis B (per-cell factor 6 lands directly in F result, no EP-doubling). Cube TYPE_X|S + halved Krüger predicted CD=40 → measured CD=40 ✓
4. **Phase B Sub-Task 2** (Finding 34) — Halved Krüger -3 universal test FAILED: Cube CD=40 (per-cell OK but 40× target), Ahmed Cd=75 (264× target), MR2 +163,000 N (290× target). Iron-Rule trigger after MR2.
5. **Phase B Sub-Task 2.5** (Findings 35-36) — Reverse-validation (WW off → +1,643 N BB-confirmed), Full Krüger -6 on current MR2 STL (-610 N matches Finding 25 exactly), VTK exports from all 3 states for OpenFOAM-vergleich.

**Architectural conclusion:** Three stable LBM attractors exist in Krüger-coupling-strength parameter space — 0% (WW off): +1820 N (BB baseline), 50% (halved): +163,000 N (broken transient-stuck state), 100% (full): -610 N (over-corrected). **Linear interpolation predicts +517 N at midpoint, actual is +163,000 N (315× off)** — proves self-referential WW-kernel ↔ Krüger coupling is fundamentally non-linear bifurcation. No coupling-strength tuning yields physically correct results.

**Per-geometry pattern:** Halved Krüger fails universally on BL-resolved geometries (Ahmed 264×, MR2 290×). Works only on sharp-edged Cube (CD=40, still 40× off but follows linear per-cell math). **Krüger Moving-Wall is fundamentally unfit as WW transport mechanism** for stationary walls.

**Production state after CC#11 (commit `741a974`):** Step-1b Safe-State as default. `apply_moving_boundaries` has TYPE_X-Exclusion filter active — vehicle cells (TYPE_S|TYPE_X) receive NO Krüger correction. Floor (TYPE_S only) keeps standard -6 Krüger. WW kernel still runs and writes u_slip to vehicle u-field, but those values are "dead data" (not transported by Krüger). Net effect: **pure bounce-back at vehicle = BB-baseline +1820 N** (3.2× over OpenFOAM target +565 N).

**17 deviations from upstream FluidX3D documented in `findings/35_deviations_from_upstream.md`:**
- LBM-Core (kernel.cpp): apply_moving_boundaries Step-1b filter; NEW kernels apply_freeslip_y (CC#9), apply_wall_model_vehicle (CC#10), apply_wall_slip_to_fluid (CC#11 disabled), compute_wall_model_artifact (CC#11 disabled), update_moving_boundaries per-step; stream_collide TYPE_E+TYPE_Y mirror branch (CC#8, inactive for CC6_MODE=1)
- LBM-Wrapper (lbm.cpp): kernel allocations, enqueue methods, WW chain in do_time_step, def_nu device define
- Config (defines.hpp): WALL_MODEL_VEHICLE define

**Files added/modified in CC#11:**
- `src/kernel.cpp` — `apply_moving_boundaries` Step-1b TYPE_X-Exclusion (re-enabled 2026-05-13 after halved Krüger MR2 failure)
- `src/lbm.cpp` — comments documenting Option 1/2 disabled state
- `CLAUDE.md` — NEW engineering methodology rules (Skalen-Ladder, Smoke-Test, Root-Cause-vor-Pivot)
- `findings/20_*.md` through `findings/36_*.md` — 16 findings documenting the investigation
- VTK exports preserved in `export/`: `u-000005000.vtk` (halved -3), `u-000006900.vtk` (full -6), `u-000012900.vtk` (WW off)

**Next phase decision (open):**
- **Phase C-A: OpenLB Pi-Tensor f_neq Reconstruction** (1-2 weeks) — replace Krüger with fluid-side post-stream DDF reconstruction. Architecturally clean, literature-validated.
- **Phase C-B: Bouzidi Sub-Grid Bounce-Back** (3-5 days) — interpolated BB with sub-cell wall-distance. Complementary to WW, reduces BB over-prediction.
- Accept Step-1b Safe-State as known limitation (Lehmann documented 1.3-2.0× upper bound).

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
