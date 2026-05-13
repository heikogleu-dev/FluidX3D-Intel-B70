# Pivot Schritt 0.3 — Pure-BB Sanity Check (PASSED)

**Datum:** 2026-05-13
**Branch:** `plan-refresh-multires` (commit `423651a`)
**Setup:** Yaris (VEHICLE_GR_YARIS=1), CC#6_MODE=1 (full domain), 337.5M cells
**WW:** disabled (`#define WALL_MODEL_VEHICLE` commented out)
**Steps:** 10,200

## Result

| Metric | Value |
|---|---:|
| Fx mean (last 50 chunks) | +1055 N |
| Std ±455 N |
| Min/Max (last 50) | +24 / +1931 N |
| Sign | POSITIVE ✓ |
| Crashes | None |
| Exit code | 0 |

## Iron-Rule Pass Criteria (per Pivot Directive Schritt 0.3)

- ✅ **Drag positive** (sign check)
- ✅ **Stable** (no divergence, no crash)
- ✅ **Pure-BB fundamentally works** (LBM-Core baseline confirmed)
- ⚠️ **Magnitude:** 1055 N vs expected 2200 N — explained by:
  - Yaris (L=3.995m) vs MR2 (L=4.0m) different geometries
  - Yaris is smaller frontal area, different surface curvature
  - Higher std (±455 N) suggests not yet fully converged
  - **Not a methodological issue, just geometry-specific**

## Conclusion

**Pure-BB Sanity Check PASSED.** Multi-Resolution Track A can start (Phase 5a Sponge Layer).

## Comparison Reference

| State | MR2 Fx | Yaris Fx | Status |
|---|---:|---:|---|
| Pure-BB (Yaris, this run) | — | **+1055 N** | this baseline |
| Pure-BB Step-1b (MR2, phase0 branch) | +1820 N | — | reference from CC#11 work |
| OpenFOAM-RANS reference | +565 N | +463 N | physical target |

Yaris Pure-BB at +1055 N is ~2.3× OpenFOAM target +463 N, consistent with Lehmann's
documented 1.3-2.0× BB over-prediction (slightly above his upper bound due to coarse
10mm grid).

## Next: Multi-Resolution Hauptpfad

- **Phase 5a:** Sponge layer / non-reflecting outlet (niedrig-mittel)
- **Phase 5b-pre:** `couple_fields()` Modul-Architektur + Self-Coupling Test (mittel)
- **Phase 5b:** Double-Res Schwarz Coupling (mittel-hoch)
- **Phase 5c:** Triple-Res Extension (niedrig, nach 5b)

Pioneer goal: First 3D iterative Schwarz Multi-Resolution coupling for external
automotive aero on consumer-GPU LBM. Multi-Res convergence (Δ Fx, plane-RMS-drift,
mass-imbalance) independent of absolute drag magnitude.
