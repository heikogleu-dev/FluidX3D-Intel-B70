#!/usr/bin/env bash
# gt_reset_waechter.sh — EIN Waechter (Iron Rule: ein Laeufer, eine Kette, ein Waechter)
# gegen die am 22.08. gemessene Todesart von f4_wandfrei_prod: GuC-Engine-Reset der B70
# (Desktop teilt sich die GPU) toetet den OpenCL-Kontext; der naechste Enqueue stirbt mit
# CL_OUT_OF_RESOURCES und exit(1) OHNE Abschluss-Dump. Der ccs-Reset lag 27 s VOR dem
# Prozessende — Zeit genug, die Stopp-Datei zu setzen und den kompletten Abschlusspfad
# (inkl. VTK) noch laufen zu lassen.
#
# Aufruf:  werkzeuge/gt_reset_waechter.sh [stoppdatei]      (Default /tmp/cfd_stop)
# Verhalten: folgt journalctl -k; bei "Engine reset" oder "Timedout job" auf einer xe-GPU
# wird die Stoppdatei angelegt und der Fund protokolliert. Beendet sich selbst, sobald
# kein FluidX3D-Prozess mehr laeuft (kein Zombie-Waechter).
set -u
STOP="${1:-/tmp/cfd_stop}"
LOGF="$(dirname "$0")/../logs/gt_reset_waechter.log"
echo "$(date '+%F %T') Waechter gestartet (Stoppdatei: $STOP)" >> "$LOGF"
journalctl -kf --no-pager -o short-iso 2>/dev/null | while read -r zeile; do
	if ! pgrep -x FluidX3D >/dev/null; then
		echo "$(date '+%F %T') kein FluidX3D mehr -- Waechter beendet sich." >> "$LOGF"
		exit 0
	fi
	case "$zeile" in
	*"Engine reset"*|*"Timedout job"*)
		echo "$(date '+%F %T') GT-RESET ERKANNT: $zeile" >> "$LOGF"
		touch "$STOP"
		echo "$(date '+%F %T') Stoppdatei $STOP gesetzt -- Lauf faehrt kontrolliert herunter." >> "$LOGF"
		;;
	esac
done
