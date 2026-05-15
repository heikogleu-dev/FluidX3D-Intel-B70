#!/bin/bash
# Launch script for next Phase 5b-DR run: TYPE_E Floor + Mode 1 soft forward (alpha=0.10).
# Run AFTER current TYPE_S Mode 2 run finishes.
#
# Steps:
#   1. Archive current TYPE_S Mode 2 VTKs + CSV
#   2. Edit setup.cpp: TYPE_S -> TYPE_E (lines 944, 956), MODE 2 -> MODE 1, Mode 1 alpha 1.0 -> 0.10
#   3. Build
#   4. Launch run with nohup + line-buffered log
#   5. Apply post-run fix pipeline when done (manual step)

set -e
ROOT=/home/heiko/CFD/FluidX3D
cd "$ROOT"

echo "=== Next Run Preparation: TYPE_E Floor + Mode 1 soft alpha=0.10 ==="

# 1. Archive current run outputs
if [ -d export/dr_far ] && [ -n "$(ls -A export/dr_far 2>/dev/null)" ]; then
    echo "[1/5] Archiving current TYPE_S Mode 2 outputs..."
    mv export/dr_far  export/dr_far_typeS_mode2_$(date +%Y%m%d_%H%M)
    mv export/dr_near export/dr_near_typeS_mode2_$(date +%Y%m%d_%H%M)
    mkdir -p export/dr_far export/dr_near
fi

if [ -f bin/forces_phase5b_dr_mode2.csv ]; then
    cp bin/forces_phase5b_dr_mode2.csv bin/forces_phase5b_dr_mode2_typeS_$(date +%Y%m%d_%H%M).csv
    echo "    Force-CSV archived."
fi

# 2. Edit setup.cpp
echo "[2/5] Editing setup.cpp for TYPE_E + Mode 1 alpha=0.10..."
# Restore TYPE_E floor
sed -i 's|if(z==0u) { lbm_far.flags\[n\] = TYPE_S;|if(z==0u) { lbm_far.flags[n] = TYPE_E;|' src/setup.cpp
sed -i 's|if(z==0u) { lbm_near.flags\[n\] = TYPE_S;|if(z==0u) { lbm_near.flags[n] = TYPE_E;|' src/setup.cpp
# Change Mode 2 -> Mode 1
sed -i 's|^#define PHASE_5B_COUPLE_MODE 2 .*|#define PHASE_5B_COUPLE_MODE 1 // Production 2026-05-16: Mode 1 one-way Far->Near with soft alpha=0.10 + TYPE_E floor|' src/setup.cpp
# Change Mode 1 alpha 1.0 -> 0.10
sed -i 's|opts.alpha        = 1.0f;        // Mode 1: hard overwrite|opts.alpha        = 0.10f;       // Mode 1 SOFT forward alpha=0.10 (Production 2026-05-16: avoid hard overwrite oscillation)|' src/setup.cpp

# Sanity check edits
echo "    Verifying edits:"
grep -n 'flags\[n\] = TYPE_' src/setup.cpp | grep -E 'z==0u' | head -2
grep -n 'PHASE_5B_COUPLE_MODE [12]' src/setup.cpp | head -2
grep -n 'opts.alpha .* 0.10f' src/setup.cpp | head -3

# 3. Build
echo "[3/5] Building..."
rm -f bin/FluidX3D
./make.sh 2>&1 | tail -10

if [ ! -f bin/FluidX3D ]; then
    echo "ERROR: build failed"
    exit 1
fi

# Note: make.sh launches binary automatically. If you want to launch manually:
# nohup stdbuf -oL ./bin/FluidX3D > /tmp/dr_mode1_run.log 2>&1 &

echo ""
echo "=== Done. Run launched by make.sh. Tail /tmp/dr_mode1_run.log or bin/forces_phase5b_dr.csv ==="
echo "After run finishes, apply VTK post-fix: ./findings/post_run_fix_vtks.sh"
