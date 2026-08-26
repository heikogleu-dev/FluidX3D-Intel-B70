#!/usr/bin/env bash
# scratch_gate.sh — 2-Sekunden-Gate gegen die Scratch-Fehlerklasse (Befund 2026-08-26):
# waechst eine Kernel-Schleife ueber IGCs Unroll-Budget, werden laufzeitindizierte
# private Arrays (fhn/fpre/j/c()/w()) speicherheimisch -> private_size>0 im .zeinfo
# -> Faktor ~100 Laufzeit (2 statt 240 MLUPs, "0 GB/s"; g13-g15). Dieses Gate baut den
# AKTUELLEN src/kernel.cpp zur .cl (gen_main.cpp, Defines des Kanal-Referenzfalls),
# kompiliert offline per ocloc (KEIN GPU-Lauf) fuer iGPU und B70 und schlaegt fehl, sobald
# stream_collide in IRGENDEINEM Arm private_size>0 ODER spill_size>0 traegt (seit Rang-1-Remat).
# Legitimes Spill-Wachstum erfordert eine BEWUSSTE Lockerung dieses Gates, nie ein stilles.
#
# Aufruf: werkzeuge/scratch_gate/scratch_gate.sh    (beliebiges Arbeitsverzeichnis)
# Exit 0 = sauber, Exit 1 = Scratch ODER Spill zurueck. Referenz 26.08.2026 nachmittags
# (Rang-1-Remat): stream_collide private 0 UND spill 0 in BEIDEN Armen auf BEIDEN Geraeten.
# Historie: vor Unroll-Fix private 4256/8512; vor Remat spill 448/832 (Prod) bzw. 672/1216 (ELIBB).
set -eu
HIER="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HIER/../.." && pwd)"
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT

g++ -O1 -c "$REPO/src/kernel.cpp" -o "$T/kernel.o"
g++ -O1 "$HIER/gen_main.cpp" "$T/kernel.o" -o "$T/gen"
"$T/gen" on  "$T/elibb_on.cl"  >/dev/null
"$T/gen" off "$T/elibb_off.cl" >/dev/null

rc=0
for dev in 0x7d67 0xe223; do
  for arm in on off; do
    zeile=$("$HIER/igc_offline.sh" "$T/elibb_$arm.cl" "$dev" stream_collide | tail -1)
    echo "ELIBB=$arm $dev: $zeile"
    if ! echo "$zeile" | grep -q "private_size=0 "; then
      echo ">>> SCRATCH-GATE VERLETZT: privates Memory in stream_collide ($arm/$dev)"; rc=1
    fi
    if ! echo "$zeile" | grep -q "spill_size=0"; then
      echo ">>> SPILL-GATE VERLETZT: Register-Spill in stream_collide ($arm/$dev) -- Rang-1-Remat-Regression"; rc=1
    fi
  done
done
exit $rc
