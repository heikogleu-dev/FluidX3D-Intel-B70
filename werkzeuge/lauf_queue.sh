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
	echo "VERWEIGERT: FluidX3D laeuft bereits (PID $(pgrep -x FluidX3D | tr '\n' ' '))." >&2; exit 2
fi
if [ -f logs/queue.lock ]; then
	echo "VERWEIGERT: queue.lock existiert (PID $(cat logs/queue.lock)) -- alte Kette pruefen/loeschen." >&2; exit 3
fi
( set -o noclobber; echo $$ > logs/queue.lock ) 2>/dev/null || { echo "VERWEIGERT: queue.lock-Wettlauf (zweite Kette gleichzeitig gestartet)." >&2; exit 3; } # atomar (Gross-Audit TOCTOU)
trap 'rm -f logs/queue.lock' EXIT INT TERM
: > "$Q"
n=0; gesamt=$(awk '{gsub(/^[ \t]+|[ \t]+$/,"")} $0==""{next} substr($0,1,1)=="#"{next} {n=gsub(/::/,"::"); if(n!=1) next; split($0,t,"::"); gsub(/ /,"",t[2]); if(t[2]!="") c++} END{print c+0}' "$1") # B8: exakt wie der Schleifenfilter inkl. Leername-Skip, mawk-portabel
hb() { while [ -f logs/queue.lock ] && kill -0 $$ 2>/dev/null; do echo "[$(date +%H:%M:%S)] LAEUFT (Herzschlag)" >> "$Q"; sleep 120; done; }
hb & HB=$!
# ★ R2-Befund: Signal-Handler MUSS exit-en -- sonst setzt bash die Schleife nach dem Handler
# fort und die Kette laeuft ohne Lock/Herzschlag weiter (Iron Rule 4 im Abbruchpfad gebrochen).
trap 'rm -f logs/queue.lock; kill $HB 2>/dev/null' EXIT
trap 'rm -f logs/queue.lock; kill $HB 2>/dev/null; trap - EXIT; exit 130' INT TERM
while IFS= read -r zeile; do
	# ★ IR3-Abschluss-Loop: Zeile erst TRIMMEN, dann filtern -- eine Whitespace-Zeile startete
	# vorher einen UNBENANNTEN Default-Lauf, eine eingerueckte #-Zeile liess env das '#' ausfuehren.
	zeile="$(printf '%s' "$zeile" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')"
	case "$zeile" in ''|'#'*) continue;; esac
	case "$zeile" in *'::'*) ;; *) echo "UEBERSPRUNGEN (kein '::'): $zeile" | tee -a "$Q"; continue;; esac
	env_teil="${zeile%%::*}"; name="${zeile##*::}"; name="$(echo "$name" | tr -d ' ')"
	[ -n "$name" ] || { echo "UEBERSPRUNGEN (leerer Name): $zeile" | tee -a "$Q"; continue; }
	[ "${zeile#*::}" = "${zeile##*::}" ] || { echo "UEBERSPRUNGEN (mehrfaches '::'): $zeile" | tee -a "$Q"; continue; }
	n=$((n+1))
	echo "[$(date +%H:%M:%S)] START $n/$gesamt: $name" | tee -a "$Q"
	env $env_teil CFD_RUN_NAME="$name" bin/FluidX3D "${CFD_QUEUE_DEV:-2}" < /dev/null > "logs/$name.log" 2>&1
	rc=$?
	m=""; [ $rc -ne 0 ] && m=" FEHLER"
	echo "[$(date +%H:%M:%S)] ENDE  $n/$gesamt: $name (rc=$rc$m, cf=$(tail -1 "export/$name/kanal_zeit.csv" 2>/dev/null | cut -d, -f6))" | tee -a "$Q"
done < "$1"
echo "[$(date +%H:%M:%S)] SERIE FERTIG ($n Laeufe)" | tee -a "$Q"
