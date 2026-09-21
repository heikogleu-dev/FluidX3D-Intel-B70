#!/bin/bash
# gpu_sammler.sh <NAME> [intervall_s] -- schreibt die BUSY-Anteile beider GPUs waehrend einer
# Laufserie nach export/<NAME>_gpu.csv. REIN LESEND: startet nichts, haengt sich an nichts an,
# kein perf-Attach. Quelle ist /proc/<pid>/fdinfo des Laufprozesses (Muster vram_sammler.sh).
#
# WARUM: bis 21.09.2026 kam jede Aussage "das Fernfeld versteckt sich" aus unserem EIGENEN
# Phasenzaehler ([PHASEN] in setup.cpp). Das ist dasselbe Instrument, das die Frage stellt.
# fdinfo ist der Treiberzaehler und damit ein FREMDES Instrument -- es hat am 21.09. die
# Verdeckung erstmals unabhaengig belegt (B70 94,3 % / iGPU 86,2 %, Lauf p4_regel6).
#
# Zaehler (beide Geraete melden unterschiedlich, deshalb zwei Wege):
#   B70  pdev 0000:04:00.0, Treiber xe   : drm-cycles-ccs / drm-total-cycles-ccs -> Anteil direkt
#   iGPU pdev 0000:00:02.0, Treiber i915 : drm-engine-compute [ns]               -> gegen Wanduhr
# Ausgegeben wird der Anteil ueber das jeweils letzte Intervall, nicht seit Laufbeginn.
set -u
# ★ PFLICHT: ohne LC_ALL=C druckt awks %.1f unter de_DE ein DEZIMALKOMMA und zerlegt damit die
# CSV-Spalten ("0,0" statt "0.0"). Am 21.09.2026 beim ersten Lauf dieses Sammlers passiert.
export LC_ALL=C
cd "$(dirname "$0")/.." || exit 2
N="${1:?Aufruf: gpu_sammler.sh <NAME> [intervall_s]}"; IV="${2:-30}"
OUT="export/${N}_gpu.csv"
echo "# GPU-Auslastung aus /proc/<pid>/fdinfo waehrend der Serie $N, Intervall ${IV}s." > "$OUT"
echo "# b70_busy_pct = drm-cycles-ccs/drm-total-cycles-ccs (pdev 04:00.0, xe)" >> "$OUT"
echo "# igpu_busy_pct = drm-engine-compute[ns]/Wanduhr[ns] (pdev 00:02.0, i915)" >> "$OUT"
echo "# Versteckt sich das Fernfeld, ist igpu_busy_pct KLEINER als b70_busy_pct." >> "$OUT"
echo "wanduhr,sekunden,b70_busy_pct,igpu_busy_pct,lauf" >> "$OUT"

# fd-Nummern je pdev suchen -- sie sind nicht fest, deshalb je Messung neu bestimmen.
fd_fuer() { # $1 = pid, $2 = pdev
  local f
  for f in /proc/$1/fdinfo/*; do
    grep -q "^drm-pdev:.*$2" "$f" 2>/dev/null && { echo "$f"; return 0; }
  done
  return 1
}

t0=$(date +%s); alt=""
while [ -f logs/queue.lock ]; do
  P=$(pgrep -x FluidX3D | head -1)
  if [ -n "$P" ]; then
    FB=$(fd_fuer "$P" "0000:04:00.0" || true)
    FI=$(fd_fuer "$P" "0000:00:02.0" || true)
    if [ -n "${FB:-}" ] && [ -n "${FI:-}" ]; then
      ccs=$(grep -m1 "^drm-cycles-ccs:"       "$FB" 2>/dev/null | awk '{print $2}')
      tot=$(grep -m1 "^drm-total-cycles-ccs:" "$FB" 2>/dev/null | awk '{print $2}')
      eng=$(grep -m1 "^drm-engine-compute:"   "$FI" 2>/dev/null | awk '{print $2}')
      now=$(date +%s%N)
      if [ -n "${ccs:-}" ] && [ -n "${tot:-}" ] && [ -n "${eng:-}" ]; then
        if [ -n "$alt" ]; then
          # Felder: $1 t_alt, $2 ccs_alt, $3 tot_alt, $4 eng_alt, $5 t_neu, $6 ccs, $7 tot, $8 eng
          echo "$alt $now $ccs $tot $eng" | awk \
            -v n="$N" -v uhr="$(date +%H:%M:%S)" -v s=$(( $(date +%s) - t0 )) '{
            dt=$5-$1; if(dt<=0) exit;                       # Wanduhr des Intervalls [ns]
            dccs=$6-$2; dtot=$7-$3; deng=$8-$4;
            b = dtot>0 ? 100*dccs/dtot : -1;                 # B70: Zyklen gegen Gesamtzyklen
            i = 100*deng/dt;                                 # iGPU: Compute-ns gegen Wanduhr-ns
            printf "%s,%d,%.1f,%.1f,%s\n", uhr, s, b, i, n;
          }' >> "$OUT"
        fi
        alt="$now $ccs $tot $eng"
      fi
    fi
  fi
  sleep "$IV"
done
echo "gpu_sammler: Serie beendet, $(($(wc -l < "$OUT")-5)) Messpunkte in $OUT"
