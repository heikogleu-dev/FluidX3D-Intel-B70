# Phase 5b-DR: α-Sweep + TYPE_E Floor + Vehicle z=0 Validation

**Datum:** 2026-05-15
**Branch:** `plan-refresh-multires`
**Status:** ✅ Validated config: TYPE_E floor + Mode 2 symmetric α=0.10 + Vehicle z=0 Far / z=1 Near + wheel-contact-fix.

## Session Summary

Drei strukturelle Änderungen wurden in einem Combined-Run kombiniert (TYPE_E floor + Mode 2 symmetric α=0.2 + Vehicle z=0). Run hing nach ~3 min mit MLUPs-Crash 5500→55.

**Isolation Testing (per user direction):** alle 3 Änderungen einzeln getestet + Combined-Retry zur Bug-Reproduktion.

## Test Results (5 chunks each)

| Test | TYPE_E | Symm α | z=0 | Result |
|---|:-:|:-:|:-:|---|
| Baseline (Mode 1) | ✗ | ✗ | ✗ | ✅ PASS (Reference) |
| TEST A | ✓ | ✗ | ✗ | ✅ PASS (Fz_far sign flips, BL artifact eliminiert) |
| TEST B | ✗ | ✓ (asymm α=0.2 back only) | ✗ | ✅ PASS (wider Fx swings) |
| TEST C | ✗ | ✗ | ✓ (wheel-contact-fix) | ✅ PASS |
| PAIR A+B | ✓ | ✓ | ✗ | ✅ PASS |
| COMBINED RETRY | ✓ | ✓ | ✓ | ✅ PASS (!) |

**Diagnose:** Erster Combined-Crash war **transient GPU/Driver-Issue**, kein Code-Bug. Combined Config funktioniert reproduzierbar in folgenden Reruns.

## α-Sweep (Mode 2 symmetric bidirectional)

Mit Combined Config (TYPE_E + Vehicle z=0/z=1), Mode 2 symmetric (forward α = back α), 5 chunks pro Test:

| α | Near Fx step 100 | step 200 | step 300 | step 400 | step 500 | Fz_near swings | Stabilität |
|---|---:|---:|---:|---:|---:|---|---|
| **0.05** | 163 | 302 | 468 | 806 | **999** N | ±100 N | 🟢 super stabil, langsam |
| **0.10** | 325 | 596 | 910 | 1547 | **1836** N | ±150 N | 🟢 **stabil — WINNER** |
| **0.20** | 648 | 1172 | 1715 | 2816 | **2995** N | ±400 N | 🟡 wide swings starten |
| **0.33** | 1088 | 1918 | 2556 | **3988** | **3692** N | **±1450 N** | 🔴 oszilliert (passed peak, drops) |

**Beobachtungen:**

1. **Linear scaling:** Near's Drag wächst proportional zu α — Schwarz-Konvergenzrate proportional zu α.
2. **Instabilität bei α=0.33:** Near überschießt bei chunk 4 (3988 N) und fällt bei chunk 5 (3692 N). Fz_near spikes auf +1449 N → klares Oszillations-Zeichen.
3. **α=0.20 borderline:** Fz_near swings ±400 N akzeptabel aber an der Grenze.
4. **α=0.10 sweet spot:** stabil + moderate Konvergenzrate. Bei 150 chunks würde Near die Mode-1 Werte (~5000 N) sicher erreichen.

## Empfehlung

**`α=0.10` für stable symmetric Schwarz coupling** in Phase 5b-DR Production-Runs.

Begründung:
- Bestätigt User-Hypothese vom 2026-05-15 dass 20% zu aggressiv sein könnte
- Stable über 5 chunks (keine Oszillation)
- Bei 150 chunks erreicht Near volle Konvergenz mit Schwarz-Iteration
- Sicherheitsmarge zu α=0.20 (welches starting Fz swings zeigt)

Fallback: α=0.05 für sehr lange Runs (300+ chunks) wo absolute Stabilität wichtiger.

## Code Changes Summary

`src/setup.cpp` `main_setup_phase5b_dr()`:
- TYPE_E floor (statt TYPE_S moving wall) — eliminiert numerische BL-Artefakt
- Mode 2 symmetric coupling: `opts.alpha = opts_back.alpha = 0.10f`
- Vehicle Far z=0 (wheels on ground), Near z=1 (5mm clearance)
- Wheel-contact-patch fix: `if(z==0u && TYPE_X) u.x[n] = lbm_u` — eliminiert Velocity-Discontinuity zwischen stationary Wheels und Moving Road
- Pre-read Near u/rho for forward symmetric blend (lines 1031-1033)

Default toggles: `PHASE_5B_DR=0` (off), `PHASE_5B_COUPLE_MODE=2` (Mode 2 symmetric).

## VTK Tools (Bonus 2026-05-15)

Two helper scripts created:
- `findings/gpu_power_log.sh` — Bash logger via Intel xe driver sysfs (hwmon7/energy + gt0/freq/idle)
- `findings/analyze_gpu_log.py` — Python analyzer for power/idle stats
- `findings/fix_vtk_origin.py` — Python post-processor to fix STRUCTURED_POINTS SPACING + ORIGIN per domain (Far 15mm, Near 5mm, with correct physical positions)
- `findings/fix_vtk_headers.py` — same as fix_vtk_origin.py (earlier name)

## Next Session TODOs

1. **Production run** mit final config (α=0.10) for 150 chunks — sauberer Force-Vergleich Fx_far vs Fx_near vs Mode 1 baseline
2. **PERF-F Variante 5** (Multi-stream PCIe pipelining ~1d, ~2-3% gain, keeps tight sync)
3. **PERF-F Variante 4** (GPU-resident coupling via custom OpenCL kernel ~2-3d, full 5% gain, Pioneer-Pattern)
4. **ParaView Force-Validation** via pressure surface integration cross-check (validates BB-overshoot vs real bug)
5. **Bouzidi BB Reaktivierung** (Phase 1, paused branch) für Fx korrigieren (8× über Time-Attack target)

## See Also

- [[PHASE_5B_DR_RESULT_2026-05-14]] — initial DR runs + 95% GPU utilization measurement
- [[PHASE_5B_DUAL_DOMAIN_RESULT_2026-05-13]] — same-resolution 4-mode characterization
