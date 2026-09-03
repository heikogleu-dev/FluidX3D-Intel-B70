#!/bin/bash
# vram_sammler.sh <NAME> [intervall_s] -- schreibt den FREIEN B70-VRAM waehrend einer Laufserie
# nach export/<NAME>_vram.csv. REIN LESEND: startet nichts, haengt sich an nichts an. Er laeuft,
# solange die Queue laeuft (logs/queue.lock), und beendet sich danach von selbst -- so ueberlebt er
# die Luecken ZWISCHEN den Armen einer Serie (Muster perf_sammler.sh).
# Quelle ist visible_avail aus dem xe-DRM-Debugfs ueber /usr/local/bin/b70-vram (passwortloses sudo);
# der Laufprozess selbst darf das Debugfs nicht lesen ("Frei-Wert NICHT lesbar" im Lauflog) -- deshalb
# ueberhaupt dieser Sammler. Heiko-Vorgabe: VRAM immer als FREIE MB, nie als Prozent.
set -u
cd "$(dirname "$0")/.." || exit 2
N="${1:?Aufruf: vram_sammler.sh <NAME> [intervall_s]}"; IV="${2:-10}"
OUT="export/${N}_vram.csv"
echo "# freier B70-VRAM (visible_avail, xe-Debugfs) waehrend der Serie $N, Intervall ${IV}s" > "$OUT"
echo "wanduhr,sekunden,frei_MiB,lauf" >> "$OUT"
t0=$(date +%s)
# ★ 03.09.: ERST auf die Queue WARTEN. Wer den Sammler vor der Queue startet, findet queue.lock
# noch nicht und beendet sich sofort mit leerer CSV -- genau so beim ersten 4-mm-Versuch passiert.
for i in $(seq 1 60); do [ -f logs/queue.lock ] && break; sleep 2; done
if [ ! -f logs/queue.lock ]; then echo "vram_sammler: nach 120 s keine laufende Queue -- Abbruch."; exit 3; fi
while [ -f logs/queue.lock ]; do
  frei=$(sudo -n /usr/local/bin/b70-vram 2>/dev/null | awk -F'[: ]+' '/visible_avail/{gsub(/MiB/,"",$2); print $2}')
  lauf=$(awk '/^\[.*\] START /{n=$0} END{sub(/^.*START [0-9]+\/[0-9]+: /,"",n); print n}' logs/queue_status.txt 2>/dev/null)
  printf "%s,%s,%s,%s\n" "$(date +%H:%M:%S)" "$(( $(date +%s)-t0 ))" "${frei:-NA}" "${lauf:-}" >> "$OUT"
  sleep "$IV"
done
echo "vram_sammler: Serie beendet, $(($(wc -l < "$OUT")-2)) Messpunkte in $OUT"
