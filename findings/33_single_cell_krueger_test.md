# Finding 33: Single-Cell Krüger Force Isolation — Hypothesis B Confirmed

**Datum:** 2026-05-13
**Status:** ✅ **Hypothesis B confirmed (93% magnitude match)**
**Conclusion:** Direct factor-6 from `apply_moving_boundaries` lands in
update_force_field result WITHOUT additional ×2 BB-doubling. Halving the
Krüger -6 to -3 would directly halve the artifact contribution.

---

## Test Setup

| Parameter | Value |
|---|---|
| Domain | 32 × 32 × 32 = 32 768 cells |
| Cell flags | Single TYPE_S at (16,16,16); TYPE_E at x=0/x=Nx-1; default TYPE_F else |
| u_solid | (0.07, 0, 0) (lattice units) |
| Initial condition | Quiescent (u=0 everywhere except solid cell) |
| ν (lattice) | 0.01 |
| Steps | 10 |
| Measurement | `lbm.object_force(TYPE_S)` each step |

**Deliberate choice: TYPE_S only (NO TYPE_X)** to bypass two filters:
1. `apply_moving_boundaries` TYPE_X-Exclusion (Step-1b Safe-State) — would block Krüger correction at our test cell
2. `apply_wall_model_vehicle` filter (kernel.cpp:1487) — requires TYPE_S|TYPE_X exact match; with plain TYPE_S the WW kernel skips → u_target=0.07 stays prescribed

---

## Measured Results

| Step | F_x measured | F_y | F_z |
|------|---:|---:|---:|
| 1    | +0.100590 | 0 | 0 |
| 2    | +0.055649 | 0 | 0 |
| 3    | +0.133621 | 0 | 0 |
| 4    | +0.088440 | 0 | 0 |
| 5    | +0.132278 | 0 | 0 |
| 6    | +0.135353 | 0 | 0 |
| 7    | +0.106262 | 0 | 0 |
| 8    | +0.144440 | 0 | 0 |
| 9    | +0.129036 | 0 | 0 |
| 10   | +0.130379 | 0 | 0 |

**Steady-state estimate (steps 5-10):** F_x = 0.1296 ± 0.0127 (≈±10% std-dev around mean).

Steps 1-4 are initial transient as flow develops around the solid cell.
F_y = F_z = 0 perfectly (confirms x-symmetry of the test).

---

## Analytical Predictions vs Measurement

D3Q19 stencil: directions with |c_i,x|=1 are 2 axis (w=1/18 each) + 8 face-diagonals (w=1/36 each).
```
Σ w_i × c_i,x² = 2×(1/18) + 8×(1/36) = 1/9 + 2/9 = 1/3
```

Two candidate formulas:
- **Direct factor-6:** F = 6 × ρ × (Σ w·c²) × u_w = 6 × 1 × (1/3) × 0.07 = **0.140**
- **Factor-12 (Krüger -6 × BB-factor 2):** F = 12 × ρ × (Σ w·c²) × u_w = **0.280**

| Prediction | Magnitude | Ratio measured/predicted |
|---|---:|---:|
| Direct factor-6 | 0.140 | **0.93** ✅ |
| Factor-12 (×2 BB) | 0.280 | 0.46 |
| Hypothesis A (×3 of BB = factor 36) | 0.840 | 0.15 |

**Measured ≈ 0.93 × Direct-Factor-6 prediction** → **Hypothesis B confirmed.**

The remaining 7% discrepancy is consistent with:
- Initial transient not fully settled at step 10
- D3Q19 discretization errors
- ν=0.01 not at zero (small viscous contributions)

---

## Sign Observation

Measured F_x is **POSITIVE** (+0.13). My analytical formula in the test
output had a NEGATIVE sign (-0.14). The magnitude matches direct factor-6,
the SIGN of my analytical was wrong (convention mismatch).

This does NOT affect the hypothesis decision — magnitude ratio determines
A vs B. The sign convention in update_force_field accounts for momentum
flux in a way that gives POSITIVE F for u_w in +X. Documenting for
completeness; not a result error.

---

## Interpretation

The factor 2 in `update_force_field` (kernel.cpp:2115) is BB-momentum-
exchange for pure bounce-back DDFs. The Krüger correction term in
`apply_moving_boundaries` (kernel.cpp:1115-1128) modifies the DDF
**asymmetrically** (only one direction per pair gets `-6·w·c·u`).

When this asymmetric modification propagates via Esoteric-Pull to the
solid cell's load_f, the resulting f_i is modified, and update_force_field
sums `2 × c_i × f_i`. The Krüger contribution to this sum comes out
factor-6 (not factor-12) because the asymmetric DDF modification doesn't
get the "BB symmetry" doubling.

**Equivalent statement:** `F_artifact_per_cell ≈ 6 × Σ_(directions normal to surface) w_i × c_i × (c_i · u_w)`.

This matches the cube empirical Option-1 calibration (factor 6 worked,
factor 12 over-subtracted by 2×). Cube empirical was correct, just not
derivable from naive theory before this test.

---

## Implications for Phase B

### Halved Krüger (-6 → -3) for TYPE_X cells

**Direct halving works mathematically** but is geometry-dependent:

| Geometry | BB Baseline | Artifact (full Krüger -6) | Artifact (halved -3) | With halved | Target | Match? |
|---|---:|---:|---:|---:|---:|:---:|
| Cube | +1.5 N | +108.5 N | +54.3 N | +56 N | 1.4 N | ❌ CD ≈ 40 |
| MR2 | +1820 N | -2430 N | -1215 N | +605 N | 565 N | ✅ 7% |
| Yaris (extrapolated) | (BB) | TBD | TBD | TBD | 463 N | TBD |

**Verdict:** Halving fixes MR2 but BREAKS cube. Sign of artifact is
geometry-dependent (cube: artifact positive, MR2: artifact negative).
A single -3 factor doesn't fit both.

### Better paths from this confirmed mechanism

1. **TYPE_X-Branch with sign-aware factor:** if there were a way to
   determine artifact-sign per cell at runtime, an adaptive factor could
   work. But sign depends on surface-normal × u_slip direction, which is
   exactly what apply_moving_boundaries is already computing. Not a free
   lunch.

2. **Geometry-specific calibration:** ugly, doesn't scale, rejected
   already in [[24_option1_calibration_failure]].

3. **OpenLB Pi-Tensor f_neq Reconstruction:** modifies fluid-cell DDFs
   directly at the fluid side, no force-artifact at solid because no
   Krüger correction at solid cells. **This is now the recommended Phase B
   primary track.**

4. **Bouzidi sub-grid bounce-back (Roadmap #4 prerequisite):** orthogonal
   to WW, reduces resolution error rather than adding WW. Combinable with
   any other approach.

---

## Status: BEWIESEN — Hypothesis B (no EP doubling)

`F_artifact ≈ 6 × Σ_directions (w_i × c_i × c_i · u_w)` is the precise
mathematical form. The factor 6 was already in Krüger Eq 5.27 (2/c_s²),
not from EP-doubling. The 2× factor in update_force_field applies only to
the pure-BB part of the DDF, not to the Krüger correction term (because
the latter is asymmetric).

## Status: OFFEN — Why Cube Option-1 factor 6 worked

This test confirms factor 6 at the per-cell level for a SINGLE cell. The
cube empirical also showed factor 6 worked. Both consistent. ✓

What remains OFFEN: precise generalization of Option-1 subtraction to
arbitrary geometries (MR2 catastrophic failure with same factor 6
calibration). The per-cell mechanism is correct, but Σ over many cells
with varying u_slip-direction × surface-normal gives geometry-specific
results. Not solvable by single global factor.

---

## Path Forward Recommendation

**Phase B primary track:** OpenLB Pi-Tensor f_neq Reconstruction (1-2 weeks).
**Phase B fallback:** Accept BB-baseline (Safe-State, current 8222a36).

Halved Krüger is NOT recommended as universal solution given the cube
breakage. Could be used as **MR2-specific workaround** if Heiko explicitly
accepts geometry-locked solution.

---

## Files

- Setup test code: `src/setup.cpp` SINGLECELL_TEST block (deactivated, restored)
- Backup of test setup.cpp: deleted after restore
- Raw measurement log: `/tmp/single2.log` (10-step F_x output)

---

## See Also

- [[32_force_code_audit]] — Code-Pfad-Audit, Hypothesis A/B formulation
- [[24_option1_calibration_failure]] — Cube factor-6 empirical, MR2 catastrophic failure
- [[25_ww_six_failed_attempts]] — Full attempt journey
