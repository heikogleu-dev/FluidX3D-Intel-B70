#!/bin/bash
# ★ Iron Rule 4 (Heiko, 2026-08-16): EIN Laeufer, EINE Kette, EIN Waechter.
# Jeder GPU-Lauf startet NUR ueber dieses Skript. Es verweigert den Start, wenn schon ein
# FluidX3D laeuft (kein stiller Doppellauf mehr), fuehrt die Serie strikt sequenziell,
# schreibt Zustand+Herzschlag nach logs/queue_status.txt (Status = 1x cat) und raeumt am
# Ende nach sich auf. Aufruf: werkzeuge/lauf_queue.sh serie.txt
# serie.txt: eine Zeile je Lauf: <ENV-Zuweisungen> :: <RUN_NAME>
#   Beispiel: CFD_CASE=kugel CFD_KUGEL_DX=40 CFD_FACETTEN=3 :: j4_a3
set -u
cd "$(dirname "$0")/.." || exit 1
Q=logs/queue_status.txt; mkdir -p logs
if pgrep -x FluidX3D >/dev/null 2>&1; then
	echo "VERWEIGERT: FluidX3D laeuft bereits (PID $(pgrep -x FluidX3D | tr '\n' ' '))." | tee -a "$Q"; exit 2
fi
if [ -f logs/queue.lock ]; then
	echo "VERWEIGERT: queue.lock existiert (PID $(cat logs/queue.lock)) -- alte Kette pruefen/loeschen." | tee -a "$Q"; exit 3
fi
echo $$ > logs/queue.lock
trap 'rm -f logs/queue.lock' EXIT
: > "$Q"
n=0; gesamt=$(grep -vc '^\s*\(#\|$\)' "$1")
while IFS= read -r zeile; do
	case "$zeile" in ''|'#'*) continue;; esac
	env_teil="${zeile%%::*}"; name="${zeile##*::}"; name="$(echo "$name" | tr -d ' ')"
	n=$((n+1))
	echo "[$(date +%H:%M:%S)] START $n/$gesamt: $name" | tee -a "$Q"
	env $env_teil CFD_RUN_NAME="$name" bin/FluidX3D 2 > "logs/$name.log" 2>&1
	rc=$?
	echo "[$(date +%H:%M:%S)] ENDE  $n/$gesamt: $name (rc=$rc, cf=$(tail -1 "export/$name/kanal_zeit.csv" 2>/dev/null | cut -d, -f6))" | tee -a "$Q"
done < "$1"
echo "[$(date +%H:%M:%S)] SERIE FERTIG ($n Laeufe)" | tee -a "$Q"
