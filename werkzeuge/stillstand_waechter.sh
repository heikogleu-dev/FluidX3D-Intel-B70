#!/bin/bash
# stillstand_waechter.sh <RUN_NAME> -- NUR Diagnose, toetet nichts.
# Anlass 29.08.2026: p4_abdeckung fror bei Schritt 1901 ein; Prozess lebte, Log wuchs (nur die
# Fortschrittszeile), der Schrittzaehler stand. "Prozess da" heisst nicht "rechnet".
# Prueft alle 120 s den SCHRITTZAEHLER (Feld 5 der Fortschrittszeile: MLUPS GB/s Steps/s
# CurrentStep %). 3x unveraendert bei lebendem Prozess = STILLSTAND (exit 1). Queue fertig = exit 0.
# Erste Fassung las Feld 4 (Schritte/s, konstant) und schlug Fehlalarm -- 30.08. korrigiert.
set -u
cd "$(dirname "$0")/.." || exit 2
R="${1:?Aufruf: stillstand_waechter.sh <RUN_NAME>}"; L="logs/$R.log"
letzter=""; gleich=0
for i in $(seq 1 300); do
  sleep 120
  if ! pgrep -x FluidX3D >/dev/null 2>&1; then
    grep -q "ENDE.*$R" logs/queue_status.txt 2>/dev/null && { echo "FERTIG: $(grep "ENDE.*$R" logs/queue_status.txt | tail -1)"; exit 0; }
    echo "PROZESS WEG ohne ENDE-Zeile fuer $R -- pruefen!"; exit 1
  fi
  schritt=$(tail -c 400 "$L" 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | tr -s ' |' ' ' | grep -oE '[0-9]+ [0-9]+ GB/s [0-9]+ [0-9]+ [0-9]+%' | tail -1 | awk '{print $5}')
  echo "[$(date +%H:%M)] Schritt=${schritt:-?}"
  if [ -n "$schritt" ] && [ "$schritt" = "$letzter" ]; then
    gleich=$((gleich+1))
    [ "$gleich" -ge 3 ] && { echo "STILLSTAND: Schrittzaehler seit 6 min unveraendert bei $schritt, Prozess lebt."; exit 1; }
  else gleich=0; fi
  letzter="$schritt"
done
echo "Waechter-Zeitlimit"; exit 1
