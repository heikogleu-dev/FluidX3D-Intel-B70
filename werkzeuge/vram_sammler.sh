#!/bin/bash
# vram_sammler.sh <RUN_NAME> [intervall_s] -- schreibt den FREIEN B70-VRAM waehrend eines Laufs
# nach export/<RUN_NAME>_vram.csv. REIN LESEND: startet nichts, haengt sich an nichts an, beendet
# sich selbst, wenn kein FluidX3D mehr laeuft (Muster perf_sammler.sh).
# Quelle ist visible_avail aus dem xe-DRM-Debugfs ueber /usr/local/bin/b70-vram (passwortloses sudo).
# Der Prozess selbst kann das Debugfs nicht lesen ("Frei-Wert NICHT lesbar" im Lauflog) -- deshalb
# ueberhaupt dieser Sammler. Heiko-Vorgabe: VRAM immer als FREIE MB, nie als Prozent.
set -u
cd "$(dirname "$0")/.." || exit 2
R="${1:?Aufruf: vram_sammler.sh <RUN_NAME> [intervall_s]}"; IV="${2:-10}"
OUT="export/${R}_vram.csv"; L="logs/$R.log"
echo "# freier B70-VRAM (visible_avail) waehrend $R, Intervall ${IV}s -- Quelle xe-Debugfs via b70-vram" > "$OUT"
echo "wanduhr,sekunden_seit_start,frei_MiB,t_phys_s" >> "$OUT"
t0=$(date +%s); gesehen=0
while true; do
  if pgrep -x FluidX3D >/dev/null 2>&1; then
    gesehen=1
  else
    [ "$gesehen" -eq 1 ] && { echo "vram_sammler: $R beendet, $(($(wc -l < "$OUT")-2)) Messpunkte in $OUT"; exit 0; }
  fi
  frei=$(sudo -n /usr/local/bin/b70-vram 2>/dev/null | awk -F'[: ]+' '/visible_avail/{gsub(/MiB/,"",$2); print $2}')
  # physikalische Zeit aus der letzten Fortschrittszeile des Lauflogs (rein lesend)
  tp=$(sed 's/\x1b\[[0-9;]*m//g' "$L" 2>/dev/null | grep -ao 't_si[= ]*[0-9.]*' | tail -1 | grep -o '[0-9.]*$')
  printf "%s,%s,%s,%s\n" "$(date +%H:%M:%S)" "$(( $(date +%s)-t0 ))" "${frei:-NA}" "${tp:-}" >> "$OUT"
  sleep "$IV"
done
