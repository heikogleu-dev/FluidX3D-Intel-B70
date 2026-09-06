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
	# ★★ 06.09.2026 GPU-ZUSTANDSWAECHTER VOR DEM START. Zweimal belegt (05.09. 20:01 -> wp_x_bud2,
	# 06.09. 11:02 -> xc_frei_mittig): endet der Teardown des VORIGEN Laufs mit "Engine memory CAT
	# error", haengt der NAECHSTE Lauf auf der B70 nach "Allocating memory" -- ein Thread bei 100 %,
	# nie ein Zeitschritt, kein Log-Fortschritt. Ein blosser "Engine reset bcs" ist dagegen harmlos
	# (am 06.09. dreimal gefolgt von einem sauberen Lauf). Der Waechter wartet einmal ab und sagt
	# BESCHEID; er bricht nicht ab, weil der Zusammenhang zwar zweifach belegt, aber nicht bewiesen ist.
	if [ "${CFD_QUEUE_DEV:-2}" = "1" ] && command -v journalctl >/dev/null 2>&1; then
		cat_n=$(journalctl -k --since '-3min' --no-pager 2>/dev/null | grep -ac 'Engine memory CAT error')
		if [ "${cat_n:-0}" -gt 0 ]; then
			echo "[$(date +%H:%M:%S)] GPU-WAECHTER: $cat_n CAT-Error(s) in den letzten 3 min -- warte 60 s vor $name (Haengegefahr, 2x belegt)" | tee -a "$Q"
			sleep 60
		fi
	fi
	echo "[$(date +%H:%M:%S)] START $n/$gesamt: $name" | tee -a "$Q"
	# ★★ 06.09.2026 FORTSCHRITTSWAECHTER. Der Herzschlag bezeugt nur, dass der PROZESS lebt, nicht
	# dass er RECHNET. Am 06.09. stand xf_elibb_pur 2 h 10 min nach "Allocating memory" bei 100 % auf
	# einem Thread, waehrend die Statusdatei im Zweiminutentakt "LAEUFT" schrieb -- zwei Stunden
	# Kartenzeit fuer nichts. Die 60-s-Karenz des CAT-Waechters oben hatte diesmal NICHT gereicht
	# (beim Lauf davor schon). Deshalb: Lauf im Hintergrund starten, nach $HANG_S Sekunden pruefen, ob
	# das Log die Gitterzeile traegt. Fehlt sie, ist es die bekannte Haengesignatur (nur export/<lauf>/code
	# vorhanden, ein Thread Rl) -- dann toeten, laenger warten und EINMAL wiederholen.
	HANG_S=${CFD_QUEUE_HANG_S:-150}
	versuch=0; rc=0
	while :; do
		versuch=$((versuch+1))
		env $env_teil CFD_RUN_NAME="$name" bin/FluidX3D "${CFD_QUEUE_DEV:-2}" < /dev/null > "logs/$name.log" 2>&1 &
		pid=$!
		gestartet=0
		for _ in $(seq 1 $HANG_S); do
			sleep 1
			kill -0 $pid 2>/dev/null || { gestartet=1; break; }              # schon fertig = kein Haenger
			grep -aq 'Grid Resolution' "logs/$name.log" 2>/dev/null && { gestartet=1; break; }
		done
		if [ $gestartet -eq 1 ]; then wait $pid; rc=$?; break; fi
		echo "[$(date +%H:%M:%S)] HAENGER erkannt: $name kam in ${HANG_S}s nicht ueber die Allokation (Versuch $versuch) -- toete PID $pid" | tee -a "$Q"
		kill -TERM $pid 2>/dev/null; sleep 5; kill -KILL $pid 2>/dev/null; wait $pid 2>/dev/null
		if [ $versuch -ge 2 ]; then
			echo "[$(date +%H:%M:%S)] AUFGEGEBEN nach 2 Versuchen: $name (Geraetezustand -- CAT-Error-Muster, siehe journalctl -k)" | tee -a "$Q"
			rc=99; break
		fi
		echo "[$(date +%H:%M:%S)] warte 120 s und wiederhole $name" | tee -a "$Q"; sleep 120
	done
	m=""; [ $rc -ne 0 ] && m=" FEHLER"
	# ★★ 06.09.2026 WAECHTERAUSGABEN SICHTBAR MACHEN. print_error bricht den Lauf NICHT ab, also
	# meldet rc weiterhin 0 -- am 06.09. trugen 15 von 80 Lauflogs einen unbemerkten Error, darunter
	# "K2 verletzt: Abnahmelauf disqualifiziert". Und der naive Filter findet ihn nicht: die Rohbytes
	# sind `Error^[[0m:`, der Farb-Reset steht ZWISCHEN Wort und Doppelpunkt, `tr -d '\033'` entfernt
	# nur das ESC-Byte und laesst `[0m` stehen. Deshalb hier der vollstaendige ANSI-Filter.
	ent="$(sed -r 's/\x1b\[[0-9;]*[mK]//g' "logs/$name.log")"
	n_err=$(printf '%s' "$ent" | grep -c 'Error:')
	n_wrn=$(printf '%s' "$ent" | grep -c 'Warning:')
	[ "$n_err" -gt 0 ] && m="$m ERRORS=$n_err"
	echo "[$(date +%H:%M:%S)] ENDE  $n/$gesamt: $name (rc=$rc$m, err=$n_err, warn=$n_wrn, cf=$(tail -1 "export/$name/kanal_zeit.csv" 2>/dev/null | cut -d, -f6))" | tee -a "$Q"
	if [ "$n_err" -gt 0 ]; then
		printf '%s' "$ent" | grep 'Error:' | sed 's/.*Error: */          ! /' | cut -c1-100 | tee -a "$Q"
	fi
done < "$1"
echo "[$(date +%H:%M:%S)] SERIE FERTIG ($n Laeufe)" | tee -a "$Q"
