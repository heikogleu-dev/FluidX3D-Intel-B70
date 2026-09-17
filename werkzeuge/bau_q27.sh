#!/bin/bash
# bau_q27.sh -- baut das D3Q27-Binary nach bin_q27/FluidX3D (17.09.2026, Heiko Option 1: 8-mm-A/B D3Q27 gegen D3Q19 ohne Wandmodell).
# Objektdateien nach temp/q27/, damit der Standardbau (makefile -> temp/*.o -> bin/FluidX3D) unberuehrt bleibt.
# Flags wortgleich zum makefile-Ziel Linux (CFLAGS -O2, OpenCL-Include/Lib), plus -DCFD_VELSET27 (defines.hpp).
# NICHT make.sh benutzen: das startet das Binary nach dem Bau. Der Dateiname MUSS FluidX3D bleiben (pgrep -x FluidX3D in der Queue).
set -eu
cd "$(dirname "$0")/.."
mkdir -p temp/q27 bin_q27
rm -f bin_q27/FluidX3D   # Pruefagent 17.09. MITTEL-1: sonst bliebe bei Kompilierfehler das alte Binary stehen und liefe unter neuem Commit
CF="-std=c++17 -pthread -O2 -Wno-comment -DCFD_VELSET27 -I./src/OpenCL/include"
pids=()
for q in graphics info kernel lbm lodepng main setup shapes; do
	g++ -c src/$q.cpp -o temp/q27/$q.o $CF & pids+=($!)
done
for p in "${pids[@]}"; do wait $p; done
g++ temp/q27/*.o -o bin_q27/FluidX3D $CF -L./src/OpenCL/lib -lOpenCL
echo "gebaut: bin_q27/FluidX3D ($(sha256sum bin_q27/FluidX3D | cut -c1-16)), Commit $(git rev-parse --short HEAD)$( [ -n "$(git status --porcelain)" ] && echo ' SCHMUTZIG' )"
