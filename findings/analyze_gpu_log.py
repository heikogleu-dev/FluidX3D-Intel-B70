#!/usr/bin/env python3
"""Analyze GPU power log for actual utilization."""
import sys
import csv

if len(sys.argv) < 2:
    print("Usage: analyze_gpu_log.py <gpu_log.csv> [start_time_skip_s]")
    sys.exit(1)

path = sys.argv[1]
skip_s = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

samples = []
with open(path) as f:
    rdr = csv.DictReader(f)
    for r in rdr:
        try:
            samples.append({
                't': float(r['time_s']),
                'E': int(r['energy_uJ']),
                'freq': int(r['act_freq_MHz']),
                'idle_ms': int(r['idle_residency_ms']),
                'temp': int(r['temp_pkg_C']),
                'fan': int(r['fan_rpm']),
            })
        except (ValueError, KeyError):
            continue

if len(samples) < 2:
    print(f"Only {len(samples)} samples — need at least 2")
    sys.exit(1)

t0 = samples[0]['t']
# Filter: skip first skip_s seconds (e.g., startup)
samples_main = [s for s in samples if s['t'] - t0 >= skip_s]
print(f"Total samples: {len(samples)} (analyzing {len(samples_main)} after skip {skip_s:.1f}s)")
if len(samples_main) < 2:
    print("Not enough samples after skip")
    sys.exit(1)

# Power-over-time: P = ΔE/Δt
powers = []
freqs = []
idle_fracs = []
for i in range(1, len(samples_main)):
    s_prev = samples_main[i-1]
    s_cur = samples_main[i]
    dt = s_cur['t'] - s_prev['t']
    if dt <= 0: continue
    dE_uJ = s_cur['E'] - s_prev['E']
    power_W = dE_uJ / dt / 1e6  # µJ→J / s = W
    powers.append(power_W)
    freqs.append(s_cur['freq'])
    didle_ms = s_cur['idle_ms'] - s_prev['idle_ms']
    idle_frac = didle_ms / 1000.0 / dt  # fraction of dt spent idle
    idle_fracs.append(min(1.0, max(0.0, idle_frac)))

duration = samples_main[-1]['t'] - samples_main[0]['t']
TDP = 275.0
mean_power = sum(powers) / len(powers)
max_power = max(powers)
min_power = min(powers)
mean_idle_frac = sum(idle_fracs) / len(idle_fracs)
busy_samples = sum(1 for f in freqs if f >= 1000)  # ≥ 1 GHz = busy
boost_samples = sum(1 for f in freqs if f >= 2500)  # near max boost

print(f"\n=== GPU Power Analysis ({duration:.1f}s duration) ===")
print(f"  Mean power:     {mean_power:6.1f} W  ({100*mean_power/TDP:5.1f}% of TDP {TDP:.0f}W)")
print(f"  Max  power:     {max_power:6.1f} W  ({100*max_power/TDP:5.1f}% of TDP)")
print(f"  Min  power:     {min_power:6.1f} W  ({100*min_power/TDP:5.1f}% of TDP)")
print(f"\n=== Frequency Analysis ===")
print(f"  Mean act_freq:  {sum(freqs)/len(freqs):6.0f} MHz")
print(f"  ≥1 GHz samples: {busy_samples}/{len(freqs)} ({100*busy_samples/len(freqs):.1f}%)")
print(f"  ≥2.5 GHz boost: {boost_samples}/{len(freqs)} ({100*boost_samples/len(freqs):.1f}%)")
print(f"\n=== Idle-Residency Analysis ===")
print(f"  Mean idle fraction (from idle_residency_ms delta): {100*mean_idle_frac:.1f}%")
print(f"  ⇒ Compute fraction (1 - idle): {100*(1-mean_idle_frac):.1f}%")
print(f"\n=== Histogram of power values (10 bins from {min_power:.0f}W to {max_power:.0f}W) ===")
bins = 10
bin_w = (max_power - min_power) / bins if max_power > min_power else 1
counts = [0]*bins
for p in powers:
    idx = min(bins-1, int((p - min_power) / bin_w))
    counts[idx] += 1
for i, c in enumerate(counts):
    lo = min_power + i*bin_w
    hi = min_power + (i+1)*bin_w
    bar = '█' * int(50 * c / max(counts))
    print(f"  {lo:5.0f}-{hi:5.0f}W : {bar} {c}")

print(f"\n=== Per-sample trace (last 30 samples) ===")
print(f"  {'time':>6} {'P_W':>6} {'freq_MHz':>9} {'idle_frac':>10}")
for i in range(max(0, len(powers)-30), len(powers)):
    s_idx = i + 1
    t_rel = samples_main[s_idx]['t'] - samples_main[0]['t']
    print(f"  {t_rel:6.1f} {powers[i]:6.1f} {freqs[i]:9d} {100*idle_fracs[i]:9.1f}%")
