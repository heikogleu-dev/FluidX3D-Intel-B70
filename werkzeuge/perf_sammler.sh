#!/bin/bash
# perf_sammler.sh <RUN_NAME> -- schreibt jede neue [LEISTUNG]/[PHASEN]-Meldung als CSV-Zeile
# nach export/<RUN_NAME>_perf.csv. Rein lesend, kein perf-Attach an den Produktionslauf.
set -u
cd "$(dirname "$0")/.." || exit 2
R="${1:?Aufruf: perf_sammler.sh <RUN_NAME>}"; L="logs/$R.log"; OUT="export/${R}_perf.csv"
echo "zeit,index,ms_je_grobschritt,kopplung_gf_pct,nahfeld_pct,fernfeld_pct,kraefte_pct,schnitte_pct,host_cpu_pct" > "$OUT"
n_alt=0
while true; do
  sleep 180
  P=$(pgrep -x FluidX3D | head -1)
  if [ -z "$P" ]; then
    grep -q "ENDE.*$R" logs/queue_status.txt 2>/dev/null && { echo "Sammler fertig: $(wc -l < "$OUT") Zeilen in $OUT"; exit 0; }
    continue
  fi
  [ -f "$L" ] || continue
  cpu=$(ps -o pcpu= -p "$P" 2>/dev/null | tr -d ' ')
  Z=$(sed 's/\x1b\[[0-9;]*m//g' "$L" | tr '|' '\n' | sed 's/[[:space:]]*$//' | awk '{if($0~/^ *Info:/){if(b!="")print b;b=$0}else if($0~/^ +[^ ]/){sub(/^ +/,"",$0);b=b" "$0}}END{if(b!="")print b}' | grep -aE '\[LEISTUNG\]|\[PHASEN\]')
  n_neu=$(echo "$Z" | grep -c LEISTUNG)
  if [ "$n_neu" -gt "$n_alt" ]; then
    idx=$(echo "$Z" | grep LEISTUNG | tail -1 | grep -oE 'Index = [0-9]+' | grep -oE '[0-9]+')
    ms=$(echo "$Z" | grep LEISTUNG | tail -1 | grep -oE 'Schritt [0-9.]+ ms' | grep -oE '[0-9.]+')
    ph=$(echo "$Z" | grep PHASEN | tail -1)
    g(){ echo "$ph" | grep -oE "$1 [0-9.]+" | grep -oE '[0-9.]+$'; }
    echo "$(date +%H:%M:%S),${idx:-},${ms:-},$(g 'grob->fein'),$(g 'Schritte'),$(g 'entnehmen'),$(g 'Kraefte'),$(g 'Schnitte'),${cpu:-}" >> "$OUT"
    n_alt=$n_neu
  fi
done
