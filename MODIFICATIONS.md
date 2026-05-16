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
- **`WALL_MODEL_VEHICLE` commented out** (2026-05-13 pivot): CC#10/CC#11 deep-dive on `phase0-ahmed-validation` branch revealed Krüger Moving-Wall is architecturally unfit for stationary-wall WW (Three-Attractor non-linear bifurcation). Pure-BB baseline used for Multi-Resolution work. WW kernel code remains intact in source.
- **NEW `SPONGE_LAYER` toggle** (Phase 5a): defines `SPONGE_LAYER`, `SPONGE_DEPTH_CELLS=50`, `SPONGE_STRENGTH=0.1f`. Activates non-reflecting outlet damping at last 50 X-cells. Default: opt-in (sponge code compiled but `lbm.sponge_u_inlet=0.0` keeps it inactive unless setup explicitly enables). 3-variant Iron-Rule test on full-domain Yaris found sponge cuts wake recirculation → 74% drag drop. Use only for compact Multi-Res Mid-boxes where outlet is < 3L behind vehicle.
- **NEW `BOUZIDI_VEHICLE` toggle** (2026-05-15): sparse Bouzidi sub-grid bounce-back infrastructure on vehicle surface. Sparse-cell index buffer `bouzidi_active_cells` precomputed at setup; per-step kernel only touches those cells. Architecture preserved; EP-pull-compatible kernel still pending. Currently a no-op in the run loop. Future companion to `WALL_VISC_BOOST` (orthogonal: addresses underbody voxelization staircase).
- **NEW `WALL_SLIP_VEHICLE` toggle** (2026-05-15): DDF slip-imprint wall model, V1 full overwrite + V2 BLEND variants. Both **FAILED** in production (V1: −200 k N catastrophic, V2: f_neq decay → forces ≈ 150 N). Preserved as documented dead-end for future contributors. Default off.
- **NEW `WALL_VISC_BOOST` toggle** (2026-05-15, **production-active on master**): activates the viscosity-modification wall model — the only wall model on this fork that survives Multi-Resolution Mode 3 on complex STL because it does NOT modify DDFs. See `src/kernel.cpp` and `src/lbm.cpp` sections below.

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

## `src/kernel.cpp`

- **NEW kernel `apply_sponge_layer`** (Phase 5a 2026-05-13): Post-stream DDF blending toward local equilibrium in last `SPONGE_DEPTH_CELLS` of +X domain. Formula: `fhn[i] = (1-strength) × fhn[i] + strength × f_eq(rho_local, u_local)`. Designed for compact Multi-Resolution Mid-boxes to dampen wake oscillations before TYPE_E outlet. Iron-Rule trigger on full-domain test (cuts wake) — code retained as opt-in feature. See `findings/PHASE_5A_SPONGE_IRON_RULE_TRIGGER_2026-05-13.md`.

## `src/lbm.cpp` + `src/lbm.hpp`

- **NEW `Kernel kernel_apply_sponge_layer`** allocation (Phase 5a). Enqueue method `enqueue_apply_sponge_layer(float u_inlet)`. Conditional call in `do_time_step()` after stream_collide (gated by `lbm.sponge_u_inlet != 0.0`).
- **NEW public LBM member `float sponge_u_inlet = 0.0f`** (Phase 5a runtime toggle). Default 0 = disabled. Setup sets to `lbm_u` for activation.
- **NEW `struct PlaneSpec`** (Phase 5b-pre 2026-05-13): describes a coupling plane with `origin` (uint3 cell-index lower corner), `extent_a`/`extent_b` (plane width/height in cells perpendicular to axis), `axis` (0=X-normal, 1=Y-normal, 2=Z-normal), and `cell_size` (SI cell-spacing for resolution-ratio in Phase 5b).
- **NEW `struct CouplingOptions`** (Phase 5b-pre): runtime toggles for 2D Gauss smoothing (`smooth_plane`, `smoothing_kernel_size`), CSV diagnostics (`export_csv`), VTK plane export (`export_vtk`).
- **NEW `LBM::couple_fields()` method** (~95 lines in lbm.cpp): Schwarz Multi-Resolution coupling. Reads source-plane field data (u, rho) from one LBM instance, optionally applies 2D Gauss smoothing, writes to target-plane as TYPE_E boundary cells on another (or same) instance. Same-resolution only in 5b-pre; bilinear upsampling for resolution-ratio deferred to Phase 5b. Inline `couple_fields_self()` overload for single-domain pipeline validation. **Self-coupling invariance test PASSED 2026-05-13:** MR2 Pure-BB full-domain baseline +1651 N vs self-coupled +1625 N (Δ -1.6%, within statistical noise std ±134-143). See `findings/PHASE_5B_PRE_SELF_COUPLING_PASSED_2026-05-13.md`.
- **Phase 5b Dual-Domain Same-Resolution Coupling** (2026-05-13): NEW `#define PHASE_5B_DUAL_DOMAIN` toggle (default 0) and `void main_setup_phase5b_dual()` function (~160 lines in setup.cpp). Instantiates two independent LBM instances on single GPU: Far 1000×500×450 = 225 M cells (10m × 5m × 4.5m @ 10 mm, shrunk from CC#6-Full per user 2026-05-13 to reduce runtime, X anlauf 1.25m, X wake 4.25m, blockage 8.9%) + Near 700×300×185 = 38.85 M cells (7m × 3m × 1.85m @ 10 mm, compact around vehicle: 0.5m vor/sides/top, 2m wake). GPU memory measured: Far 14.4 GB + Near 2.5 GB = ~17 GB on Arc Pro B70 (32 GB total).
- **Phase 5b Coupling-Mode toggle** (2026-05-13): NEW `#define PHASE_5B_COUPLE_MODE` (default 1) selects coupling direction inside `main_setup_phase5b_dual()`: 0 = no coupling (Near alone with uniform-freestream TYPE_E, used for verification), 1 = one-way Far→Near at 5 outer planes per chunk (production setting for Phase 5b), 2 = naive bidirectional Far↔Near with 5 forward planes + 5 back-coupling planes at overlap-band `band=20` cells inside Near. Mode 2 found **fundamentally unstable** without proper Schwarz outer-loop iteration — std_Near explodes from 7% to 48% with band=20 due to resonant cavity between Far's outer TYPE_E and internal back-coupling TYPE_E shell. Mode 1 is the Phase 5b winner with Δ +14.3% bias (Near vs Far drag). Bias mechanism characterized: Mode 0 verification shows pure-reflection causes +59.2% bias; one-way Far→Near coupling reduces this 76% to +14.3%, confirming reflection is dominant cause (not pure geometric confinement). See `findings/PHASE_5B_DUAL_DOMAIN_RESULT_2026-05-13.md` for full 4-mode comparison + paths forward (Phase 5b-DR Double-Res as next; Phase 5b-Refined with proper Schwarz outer-loop + mass-flux correction deferred).
- **`CouplingOptions::alpha`** (Phase 5b refined Mode 2 2026-05-13): NEW `float alpha = 1.0f` field for soft-BC blend in `couple_fields()`. When `alpha < 1`, target cells are updated as `tgt = (1-alpha) * tgt_current + alpha * src_value` instead of hard overwrite. Stabilizes iterative Schwarz coupling by dampening oscillations. Default 1.0 (hard overwrite). Used with `0.2` for back-coupling Near→Far in refined Mode 2 same-res (band=3 + α=0.2): stable but +19.6% bias — Mode 1 (one-way) remained Phase 5b winner.
- **`CouplingOptions::sync_pcie`** (PERF-D optimization 2026-05-14): NEW `bool sync_pcie = true` field. When false, `couple_fields()` skips all PCIe sync operations — caller must manually `read_from_device()` source u/rho before batch + manually `write_to_device()` target buffers after batch. Enables ~5-10× speedup with multiple coupling planes per chunk (single read/write per chunk instead of one per plane). Used in Phase 5b-DR setup: 99% sustained GPU utilization confirmed (vs ~35% cyclic before PERF-D).
- **Bilinear up/downsample in `couple_fields()`** (Phase 5b-DR 2026-05-14): NEW ~35-line block in `lbm.cpp::couple_fields()` performs bilinear interpolation when `src_plane.extent != tgt_plane.extent`. Linear scale factor `scale = (src_extent - 1) / (tgt_extent - 1)`, fractional source index sampled via 4-corner weighted average. Handles both upsample (Far→Near in DR) and downsample (Near→Far back-coupling). Works for arbitrary integer or non-integer ratios. Required for Multi-Resolution Schwarz coupling between domains with different cell-sizes.
- **Phase 5b-DR Double-Resolution Schwarz Coupling** (2026-05-14): NEW `#define PHASE_5B_DR` toggle (default 0) and `void main_setup_phase5b_dr()` function (~200 lines in setup.cpp). Pfad A config: Far 16×8×5m @ 15mm = 190M cells (12.2 GB) + Near 6.6×2.7×1.695m @ 5mm = 242M cells (15.5 GB) = 27.7 GB total VRAM on Arc Pro B70 (32 GB). 3:1 cell-size ratio with exact integer Near-in-Far alignment. Time-step sync: dt_far = 3 × dt_near → Far runs `chunk_far` steps, Near runs `3*chunk_far` steps per chunk for same SI time. Per-LBM `Units` objects (local Units instances + global `units` set to Far's scale). DR Mode 1 (one-way Far→Near, chunk=100): **Fx_far 4835 N ±13%, Fx_near 2181 N ±2%, delta -54.9%** — Resolution-Effekt klar bestätigt, user-Prognose -30 to -50% direkt verifiziert. DR Mode 2 (bidirectional band=2 α=0.2 chunk=25): **UNSTABLE feedback-loop**, Fx_near collapses to 844 N ±78%. See `findings/PHASE_5B_DR_RESULT_2026-05-14.md` for full analysis incl. 2-GPU architecture trade-offs.

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

605 GB/s ≙ 99.5 % of the B70 spec (608 GB/s) — the LBM workload is fully bandwidth-limited and the GPU is saturating its memory subsystem. See README § "Why effective bandwidth > 608 GB/s" for the relationship between FluidX3D's reported number and physical DRAM bandwidth (the metric counts a theoretical AA-pattern read+write; Esoteric-Pull halves real DRAM traffic, L2 hits count toward the metric).

---

# Session 2026-05-15 — Mode 3 PERF-G Concurrent LBM + `WALL_VISC_BOOST`

This session closed two outstanding gaps in the Multi-Resolution stack on top of Phase 5b-DR's one-way Far→Near baseline:

1. **Mode 3 PERF-G** — concurrent additive-Schwarz coupling using a second OpenCL queue, both directions blended with α = 0.20, symmetric 1-chunk lag
2. **`WALL_VISC_BOOST`** — the first wall model on this fork that survives Multi-Resolution Mode 3 on complex STL, by modifying the LBM relaxation rate (not DDFs)

Both are now the production-default on the `master` branch. The findings docs in `findings/` carry the day-by-day timeline; this section is the durable changelog.

## `src/opencl.hpp`

- **NEW second `cl_queue` member** in the device wrapper. Used by `LBM::run_async()` to dispatch a chunk on Near while Far is mid-chunk on the primary queue. Two-queue model is required for Mode 3 PERF-G's concurrent Additive Schwarz: neither domain blocks on the other within a chunk; coupling reads see the previous chunk's data on both sides (symmetric 1-chunk lag = canonical additive-Schwarz invariant).

## `src/lbm.hpp` + `src/lbm.cpp` — Mode 3 + WALL_VISC_BOOST infrastructure

### Mode 3 PERF-G — concurrent run

- **NEW `LBM::run_async(uint steps)`** — non-blocking variant of `run()`. Dispatches the chunk on the secondary `cl_queue` and returns immediately.
- **NEW `LBM::finish()`** — explicit barrier on the secondary queue. Pairs with `run_async()` for the outer chunk loop: `lbm_far.run_async(chunk); lbm_near.run(chunk); lbm_far.finish();`.
- Wallclock overhead vs Mode 1 (sequential one-way) is **~5–10 %**, paying for the bidirectional coupling. The benefit is **physically consistent Fx_far ≈ Fx_near** (Mode 1 left a 55 % drag gap that signalled the back-coupling was missing).

### `WALL_VISC_BOOST` — wall-distance flag buffer

- **NEW `Memory<uchar> wall_adj_flag`** on `LBM_Domain` (allocated when `WALL_VISC_BOOST` is defined): per-cell wall-distance label, `0 = interior`, `1/2/3 = layer index from nearest TYPE_S wall`.
- **NEW `LBM::populate_wall_adj_flag()`** — multi-pass BFS expansion on the host, executed after all BC parallel-for loops have written the final `flags` state:
  - Pass 1: for every fluid cell, scan 6 axis neighbours; if any is `TYPE_S`, set `wall_adj_flag[n] = 1`.
  - Passes 2..`MAX_WALL_DISTANCE` (=3): for every interior fluid cell, scan neighbours; if any is layer-`(k-1)`, set `wall_adj_flag[n] = k`.
  - Vehicle marker (`TYPE_S | TYPE_X`) AND floor (`TYPE_S` without `TYPE_X`) are both detected — both contribute eddy-viscosity layers.
  - Crucial timing constraint: must run **after** the BC parallel-for loops and must **NOT** call `flags.read_from_device()`, which would overwrite host-side BC state with stale GPU initial values.

### Friend access for sparse Bouzidi

- `LBM_Domain` declares `friend class LBM` to give the parent class direct access to the sparse-cell index buffer used by the parked Bouzidi infrastructure. Required by `compute_bouzidi_cells_active()` (precompute pass, runs at setup).

## `src/kernel.cpp` — `WALL_VISC_BOOST` collision-side modification

In the SRT collision branch of `stream_collide`, between the equilibrium computation and `store_f`, the relaxation rate `w` is replaced for wall-adjacent cells:

```c
#ifdef WALL_VISC_BOOST
const uchar wall_dist = wall_adj_flag[n];
if (wall_dist != (uchar)0u) {
    const float u_mag = sqrt(sq(uxn) + sq(uyn) + sq(uzn));
    if (u_mag > 1e-6f) {
        const float nu = def_nu;
        // Werner-Wengle PowerLaw closed-form u_tau:
        const float u_visc2 = 2.0f * nu * u_mag;
        const float u_log2  = 0.0246384f * pow(nu, 0.25f) * pow(u_mag, 1.75f);
        const float u_tau   = sqrt(max(u_visc2, u_log2));
        // Prandtl mixing-length eddy viscosity at cell-centre distance:
        const float y_center    = (float)wall_dist - 0.5f;        // 0.5, 1.5, 2.5 lu
        const float nu_t_target = 0.41f * y_center * u_tau;       // κ·y·u_τ
        // Convert ν → relaxation rate:
        const float nu_current = (1.0f / w - 0.5f) / 3.0f;
        const float nu_new     = nu_current + nu_t_target;
        w = 1.0f / (3.0f * nu_new + 0.5f);
    }
}
#endif
```

**Why it works where five DDF-modifying approaches failed:** modifying `w` changes the local effective viscosity used by the collision operator — it does not touch the DDFs themselves. Esoteric-Pull's `load_f` / `store_f` round-trip is therefore preserved unchanged, and the modification survives the streaming step exactly as intended. This is the path Lehmann himself pointed to in [Discussion #58](https://github.com/ProjectPhysX/FluidX3D/discussions/58): *"Wall Model via local viscosity modification"*. Five DDF-modifying paths (CC#10 Krüger WW in Multi-Res, Path II.5 Floor-WW, Bouzidi Sparse, WALL_SLIP V1 full overwrite, WALL_SLIP V2 BLEND) all failed first; full failure analysis in `findings/BOUZIDI_EP_PULL_INCOMPATIBILITY_2026-05-15.md`.

**Parameter-order constraint (production trap):** when extending `stream_collide` arguments with `wall_adj_flag`, it MUST be added to `add_parameters()` in `lbm.cpp` **after** the optional `FORCE_FIELD F` / `SURFACE` / `TEMPERATURE` blocks, to match the kernel signature order. Earlier insertion produced `CL_OUT_OF_RESOURCES (-5)` on `clEnqueueNDRangeKernel` — parameter-binding mismatch with the GPU side. Fixed and documented in `findings/WALL_TREATMENT_SUMMARY_2026-05-15.md`.

## `src/setup.cpp` — Mode 3 production setup

`PHASE_5B_COUPLE_MODE == 3` is the production default (`#define PHASE_5B_COUPLE_MODE 3`). Inside `main_setup_phase5b_dr()`:

- `opts.alpha = 0.20f` (Far → Near forward blending)
- `opts_back.alpha = 0.20f` (Near → Far back-coupling, same blend ratio for symmetric Schwarz)
- α-sweep (0.10 / 0.15 / 0.20) confirmed 0.20 as the best compromise: 24 % faster convergence than 0.10 with smooth transitions, no oscillation onset.
- Floor at `z = 0` is `TYPE_S` with `u_x = lbm_u` (moving wall, road velocity) on BOTH domains — this is the regime change that restored the underbody Venturi: `Fz_near = −552 N (downforce)` vs `+290 N` lift on TYPE_E floor.
- Wheel-contact cells (vehicle cells at `z = 0`) inherit `lbm_u` so the wheel base behaves correctly.
- After all BC parallel-for loops complete and before the outer chunk loop, both domains call `populate_wall_adj_flag()` when `WALL_VISC_BOOST` is active.

### Outer chunk loop (Mode 3)

```cpp
lbm_far.run_async(chunk_far);            // dispatch Far on secondary queue
lbm_near.run(3 * chunk_far);             // run Near on primary queue (3× steps for 3:1 ratio)
lbm_far.finish();                        // barrier on Far before coupling
lbm_far.couple_fields(lbm_near, ...);    // back-coupling Near → Far with α = 0.2
lbm_near.couple_fields(lbm_far, ...);    // forward Far → Near with α = 0.2
```

### Production result (commit `4cccdaf`, EOD 2026-05-15)

| | Far (15 mm) | Near (5 mm) |
|---|---:|---:|
| Fx (drag) | 1 464 N | 1 597 N |
| Fz (lift/downforce) | — | **−563 N** |
| Convergence | 150 chunks | 38 min wallclock |

Phase 2 (single cell layer) was the EOD 2026-05-15 production setting. Phase 3 (multi-cell BFS to 3 layers, hardcoded y_p=0.5 in u_τ formula) tested 2026-05-16: forces statistically identical to Phase 2 (within ±2 %). Phase 3.1 (Phase 4 production) corrects the two physics issues — explicit y_lu in Werner-Wengle closed form + Van Driest damping `(1-exp(-y+/26))²`. See § "Session 2026-05-16" below.

---

# Session 2026-05-16 — Phase 4 Aggressive: 4 mm Near + 20 mm Far + 5:1 ratio + Phase 3.1 wall fix + tapered α blending

Phase 4 brings together three orthogonal improvements over the Phase 3 baseline, validated 2026-05-16 with a 50-minute production run. Tagged `production-phase4-2026-05-16`. Commit `840866c` for code, `cc08f0e` for finding documentation.

## `src/kernel.cpp` — Phase 3.1 WALL_VISC_BOOST upgrade

The Phase 3 block hardcoded the Werner-Wengle closed-form constants `2.0*nu*u_mag` and `0.0246384*pow(nu,0.25f)*pow(u_mag,1.75f)` for y_p = 0.5 lu (the first wall-adjacent cell distance). Phase 3.1 corrects both issues:

1. **Explicit `y_lu`** — the closed-form now reads
   ```c
   const float y_lu    = (float)wall_dist - 0.5f;
   const float u_visc2 = nu * u_mag / y_lu;
   const float u_log2  = 0.02462f * pow(u_mag, 1.75f) * pow(nu, 0.25f) / pow(y_lu, 0.25f);
   ```
   so Layer 2 (y_lu=1.5) and Layer 3 (y_lu=2.5) get physically correct u_τ instead of the systematic overshoot of the y_p=0.5-baked-in constants.

2. **Van Driest damping** (Van Driest 1956, A+ = 26):
   ```c
   const float y_plus  = y_lu * u_tau / nu;
   const float vd_damp = (1.0f - exp(-y_plus / 26.0f)) * (1.0f - exp(-y_plus / 26.0f));
   const float nu_t_target = 0.41f * y_lu * u_tau * vd_damp;
   ```
   ν_t → 0 in the viscous sublayer (y+<5), correcting the pure-linear `κ·y·u_τ` growth that was only valid in the log layer (y+ ≈ 30–300).

Empirical effect on integrated forces: ~2 % (below noise of Phase 3 vs Phase 2). The change is primarily about physical defensibility, not Drag-Quantitäts-Korrektur.

## `src/lbm.hpp` — `CouplingOptions.keep_flags`

New struct field `bool keep_flags = false;` controls whether `couple_fields()` writes `flags[n] = TYPE_E` on target cells. When `true`, only `u/rho` are blended (TYPE_F preserved). Enables the tapered band coupling where inner-band cells remain interior fluid but get gentle nudging toward Far's interpolated value.

## `src/lbm.cpp` — `couple_fields()` respects `keep_flags`

The TYPE_E write at line ~1490 is now gated:
```cpp
if(!opts.keep_flags) tgt_sim.flags[n] = TYPE_E;
```
The α-blend logic unchanged. With `keep_flags=true` + `alpha<1`, the cell stays TYPE_F and gets soft-blended.

## `src/setup.cpp` — Phase 5b-DR refactored for Phase 4 Aggressive

### Cell sizes and box dimensions (lines 866–905)

| | Phase 3 | **Phase 4** |
|---|---:|---:|
| `dx_far` | 0.015 m | **0.020 m** |
| `dx_near` | 0.005 m | **0.004 m** |
| Ratio | 3:1 | **5:1** |
| Far cells | 900×534×334 = 160 M | **650×300×225 = 44 M** |
| Near cells | 1320×540×339 = 242 M | **1600×630×375 = 378 M** |
| Far world | X[−1.5,+12] Y[±4] Z[0,+5.01] | **X[−1,+12] Y[±3] Z[0,+4.5]** |
| Near world | X[−0.495,+6.105] Y[±1.345] Z[0,+1.695] | **X[−0.4,+6] Y[±1.26] Z[0,+1.5]** |
| Near origin in Far cells | (67, 177, 0) | **(30, 87, 0)** |
| Vehicle clearance | 15 mm (1 Far cell at 15 mm) | **20 mm (1 Far cell at 20 mm = 5 Near cells)** |
| `chunk_near` multiplier | × 3 | **× 5** |
| Total VRAM | 25.8 GB | **27.4 GB** |

All Near box corners exactly align on Far cells (Near offset in Far cells: 30, 87, 0 — all integer; Near extents in Far cells: 320, 126, 75 — exact 5× of Near cell counts). Sub-cell coupling-plane mapping is therefore perfectly clean at 5:1.

### Forward coupling planes — 5:1 dimensions (lines 998–1012)

All 5 forward planes (X_min/max, Y_min/max, Z_max) updated for 5:1 ratio with new Near origin. Coupling Z range: Far Z=1..74 (74 cells, skip floor z=0 and skip cells beyond Near top), Near Z=1..370 (370 cells = 74×5). Top 4 Near cells (Z=371..374, adjacent to TYPE_E top wall) uncoupled — same pattern as old setup at 3:1.

### Tapered α blending (Mode 3 run loop, lines 1106–1145)

After the 5 outer-boundary forward calls (d=0, hard TYPE_E, α=0.20 — original behaviour), a loop adds 19 more layers per direction with soft blending:

```cpp
const uint BAND = 20u; // 80 mm physical at 4 mm Near dx
const float alpha_max = opts.alpha;
for(uint d = 1u; d < BAND; d++) {
    const float taper = 1.0f - (float)d / (float)BAND;
    CouplingOptions opts_d = opts;
    opts_d.alpha      = alpha_max * taper * taper;  // quadratic decay
    opts_d.keep_flags = true;                        // preserve TYPE_F
    const uint dsrc = d / 5u;                        // Far cell shift (integer)
    // Shift target d cells inward and source dsrc Far cells correspondingly,
    // then call couple_fields for each of the 5 boundary directions.
    // ...
}
```

Result: 5 × 20 = 100 forward coupling calls per chunk (vs 5 in Phase 3). Per-call cost is small (parallel_for on plane), so the wall-clock impact is <5 % despite the 20× call count.

The band creates an 80 mm sponge zone where Near's high-res turbulence is gradually relaxed toward Far's coarser representation, eliminating the visible "Kante" the user observed in Phase 3 ParaView screenshots.

## Phase 4 Production Run Result

Mode 3 PERF-G + WALL_VISC_BOOST Phase 3.1 + Tapered α + 4 mm Near + TYPE_S moving floor, MR2 vehicle at 30 m/s.

| Force | Phase 3 (5mm/15mm) | **Phase 4 (4mm/20mm)** | Δ |
|---|---:|---:|---:|
| Fx_far (drag, Coupling carrier) | 1 484 N | 2 557 N | +72 % (coarsening — expected) |
| **Fx_near** (drag, **physical signal**) | 1 574 N | **1 494 N** | **−5 %** |
| Fy_far / Fy_near | ~0 | +23 / +3.4 N | ~0 ✓ symmetric |
| **Fz_near** (downforce) | −552 N | **−799 N** | **+45 % stronger** |

- Run converged at chunk 87 (auto-stop, `|dFx_far|/|Fx_far| < 2%`), 50 min wallclock.
- 33 s/chunk vs Phase 3's 17.6 s/chunk — compute-bound (cell-step ratio 2.19×), not coupling-bound.
- Far is now intentionally a coupling carrier only. Fx_far rise is a numerical artifact of 20 mm cells too coarse to resolve the vehicle BL (~1 Far-cell wide). Fx_near remains the physically-relevant drag signal.

Detailed analysis: [findings/PHASE_4_FINAL_RESULT_2026-05-16.md](findings/PHASE_4_FINAL_RESULT_2026-05-16.md).

## Production tag

`production-phase4-2026-05-16` at commit `cc08f0e` marks this configuration as the stable production checkpoint. Future experimental work (Triple-Res with iGPU as outer wake-extension, Bouzidi BB reactivation, etc.) branches off this tag.

## Branch / repository policy

As of 2026-05-16: **`master` is the only long-lived branch** on this fork. Feature work happens on `master` directly. The earlier `plan-refresh-multires` and `phase0-ahmed-validation` feature branches were consolidated:
- `plan-refresh-multires` → fast-forward-merged into `master` (it carried the same commits, just on a feature branch by accident from earlier days)
- `phase0-ahmed-validation` → preserved as the immutable tag [`archive/phase0-ahmed-validation`](https://github.com/heikogleu-dev/FluidX3D-Intel-B70/releases/tag/archive/phase0-ahmed-validation). The 22 unique commits (Bouzidi Step 1 PASSED, WW deep-dive Findings 20–37 architectural-limit documentation) are reachable via that tag for future resume.

If you need to revisit a parked path, branch off the archive tag:
```bash
git checkout -b resume-bouzidi archive/phase0-ahmed-validation
```
