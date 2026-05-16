#!/usr/bin/env python3
"""
High-resolution B70 + iGPU utilization profiler for detecting idle gaps in Phase 7.
Samples at 0.5s rate over user-configurable duration, plots utilization + freq + power.

Usage: profile_b70_idle.py [--duration 120] [--rate 0.5] [--output /tmp/b70_profile.png]
"""
import argparse, time, os, sys

ap = argparse.ArgumentParser()
ap.add_argument("--duration", type=float, default=120.0, help="profile length (s), default 120")
ap.add_argument("--rate",     type=float, default=0.5,   help="sample interval (s), default 0.5")
ap.add_argument("--output",   default="/tmp/b70_profile.png")
ap.add_argument("--pid",      type=int, default=None, help="FluidX3D PID for fdinfo iGPU compute time")
args = ap.parse_args()

# Auto-detect FluidX3D PID if not given
if args.pid is None:
    import subprocess
    r = subprocess.run(["pgrep","-f","bin/FluidX3D"], capture_output=True, text=True)
    pids = r.stdout.split()
    if not pids:
        print("ERROR: FluidX3D not running, no --pid given", file=sys.stderr); sys.exit(1)
    args.pid = int(pids[0])
print(f"PID={args.pid} duration={args.duration}s rate={args.rate}s")

# Locate iGPU fd (i915 renderD128)
fd_igpu = None
for fdname in os.listdir(f"/proc/{args.pid}/fd"):
    try:
        t = os.readlink(f"/proc/{args.pid}/fd/{fdname}")
        if t == "/dev/dri/renderD128":
            fd_igpu = fdname; break
    except OSError: pass
print(f"iGPU fd = {fd_igpu}")

B70_IDLE = "/sys/class/drm/card0/device/tile0/gt0/gtidle/idle_residency_ms"
B70_FREQ = "/sys/class/drm/card0/device/tile0/gt0/freq0/act_freq"
B70_NRG  = "/sys/class/drm/card0/device/hwmon/hwmon7/energy1_input"
B70_T_PKG= "/sys/class/drm/card0/device/hwmon/hwmon7/temp2_input"
IGPU_FREQ= "/sys/class/drm/card1/gt_act_freq_mhz"
def rd(p):
    try:
        with open(p) as f: return f.read().strip()
    except: return None
def igpu_ns():
    try:
        with open(f"/proc/{args.pid}/fdinfo/{fd_igpu}") as f:
            for line in f:
                if line.startswith("drm-engine-compute:"):
                    return int(line.split()[1])
    except: pass
    return None

samples = []
t_start = time.time()
t_prev = t_start
b70_idle_prev = int(rd(B70_IDLE) or 0)
b70_nrg_prev = int(rd(B70_NRG) or 0)
igpu_ns_prev = igpu_ns() or 0

n_samples = int(args.duration / args.rate)
print(f"Sampling {n_samples} ticks...", flush=True)
for i in range(n_samples):
    # busy-wait to next sample time for accuracy
    next_t = t_start + (i+1) * args.rate
    sleep_dt = next_t - time.time()
    if sleep_dt > 0: time.sleep(sleep_dt)
    t_now = time.time()
    dt = t_now - t_prev
    dt_ms = dt * 1000
    b70_idle = int(rd(B70_IDLE) or 0)
    b70_freq = int(rd(B70_FREQ) or 0)
    b70_nrg  = int(rd(B70_NRG)  or 0)
    b70_tpkg = int(rd(B70_T_PKG) or 0) / 1000.0
    igpu_freq= int(rd(IGPU_FREQ) or 0)
    igpu_ns_now = igpu_ns() or 0
    util_b70 = max(0.0, min(100.0, 100 - (b70_idle - b70_idle_prev) / dt_ms * 100))
    power_b70 = (b70_nrg - b70_nrg_prev) / 1e6 / dt  # μJ → J → W
    util_igpu = max(0.0, min(100.0, (igpu_ns_now - igpu_ns_prev) / 1e9 / dt * 100))
    samples.append({
        "t": t_now - t_start, "b70_util": util_b70, "b70_freq": b70_freq, "b70_power": power_b70, "b70_tpkg": b70_tpkg,
        "igpu_util": util_igpu, "igpu_freq": igpu_freq,
    })
    b70_idle_prev = b70_idle; b70_nrg_prev = b70_nrg; igpu_ns_prev = igpu_ns_now; t_prev = t_now

print(f"Done sampling. min/mean/max B70 util: {min(s['b70_util'] for s in samples):.0f}/{sum(s['b70_util'] for s in samples)/len(samples):.0f}/{max(s['b70_util'] for s in samples):.0f}%")
print(f"min/mean/max iGPU util: {min(s['igpu_util'] for s in samples):.0f}/{sum(s['igpu_util'] for s in samples)/len(samples):.0f}/{max(s['igpu_util'] for s in samples):.0f}%")
# Idle-gap stats
b70_idle_ticks = sum(1 for s in samples if s['b70_util'] < 10)
print(f"B70 ticks <10% util: {b70_idle_ticks}/{len(samples)} ({100*b70_idle_ticks/len(samples):.1f}%)")

# Plot
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
t = [s['t'] for s in samples]
fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
axes[0].plot(t, [s['b70_util']  for s in samples], color="#1f77b4", lw=1.2, label="B70 util")
axes[0].plot(t, [s['igpu_util'] for s in samples], color="#ff7f0e", lw=1.0, label="iGPU util", alpha=0.85)
axes[0].axhline(80, color="gray", ls="--", lw=0.6, alpha=0.5)
axes[0].set_ylabel("Utilization [%]")
axes[0].set_ylim(0, 105)
axes[0].legend(loc="upper right"); axes[0].grid(alpha=0.3)
axes[0].set_title(f"Phase 7 GPU profile — duration {args.duration:.0f}s, sample rate {args.rate}s (PID {args.pid})")

axes[1].plot(t, [s['b70_freq']  for s in samples], color="#1f77b4", lw=1.0, label="B70 freq")
axes[1].plot(t, [s['igpu_freq'] for s in samples], color="#ff7f0e", lw=1.0, label="iGPU freq", alpha=0.85)
axes[1].set_ylabel("Frequency [MHz]")
axes[1].legend(loc="upper right"); axes[1].grid(alpha=0.3)

axes[2].plot(t, [s['b70_power'] for s in samples], color="#d62728", lw=1.0, label="B70 power [W]")
ax2b = axes[2].twinx()
ax2b.plot(t, [s['b70_tpkg'] for s in samples], color="#2ca02c", lw=1.0, label="B70 pkg temp [°C]")
axes[2].set_ylabel("Power [W]"); ax2b.set_ylabel("Temp [°C]")
axes[2].set_xlabel("Time [s]")
axes[2].grid(alpha=0.3)
lines1, labels1 = axes[2].get_legend_handles_labels()
lines2, labels2 = ax2b.get_legend_handles_labels()
axes[2].legend(lines1+lines2, labels1+labels2, loc="upper right")
plt.tight_layout()
plt.savefig(args.output, dpi=110)
print(f"Saved plot: {args.output}")
