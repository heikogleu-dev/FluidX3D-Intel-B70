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
	# ★★ 15.09.2026 ABSTURZSPERRE: Atomik-Testhaken (CFD_KLEMM_HAKEN, CFD_POSITIV_HAKEN) ausserhalb der Kugel NIE auf einer GPU.
	# Am 15.09. 15:49 hat CFD_KLEMM_HAKEN=1 im 8-mm-Fahrzeug die B70 lahmgelegt (device wedged, CL_OUT_OF_RESOURCES, Desktop hing).
	# Zweite Sperre neben der im Code (lbm.cpp, Gittergrenze): die Queue startet solche Zeilen gar nicht erst.
	# LUECKE geschlossen 15.09. abends: gekoppelte Faelle waehlen ihre Geraete SELBST (CFD_DEV_FINE/CFD_DEV_COARSE, sonst B70+iGPU) --
	# CFD_QUEUE_DEV=0 zwingt sie NICHT auf die CPU (belegt: kl_s0d_dd16_h1_cpu, von der Code-Sperre gefangen). Erlaubt ist ein Haken
	# ausserhalb der Kugel darum nur mit CFD_DEV_FINE=0 UND CFD_DEV_COARSE=0 in der Zeile.
	# ★ Audit-Schleife 16.09.2026, Befund B7: das Muster war '=[1-9]' und traf damit WEDER "=01" NOCH "=+1" -- atoi() im Host liefert in
	# beiden Faellen 1, der Haken war also AKTIV, waehrend die Queue die Zeile durchliess (nachgestellt am 16.09.). Anker waren zudem
	# Leerzeichen, ein TAB zwischen zwei Zuweisungen haette sie umgangen. Jetzt: optionales '+' und fuehrende Nullen, [[:space:]] als Anker.
	# Zusaetzlich die GEERBTE Umgebung pruefen: 'env $env_teil' erbt exportierte Variablen, ein 'export CFD_KLEMM_HAKEN=1' in der Startschale
	# war fuer die Queue unsichtbar. Die Code-Sperre in lbm.cpp faengt den gefaehrlichen Rest weiterhin vor dem Geraetebau ab.
	# ★ Nachpruefung 16.09.2026 (Befund M4): die geerbte Umgebung zaehlt NUR, wenn die Zeile die Variable nicht selbst setzt --
	# "env CFD_KLEMM_HAKEN=0 ..." ueberschreibt den geerbten Wert, der Lauf ist dann harmlos und darf nicht verweigert werden.
	geerbt_haken=0
	if ! echo " $env_teil " | grep -q '[[:space:]]CFD_KLEMM_HAKEN='; then
		[ -n "${CFD_KLEMM_HAKEN:-}" ] && [ "${CFD_KLEMM_HAKEN:-0}" != "0" ] && geerbt_haken=1
	fi
	if ! echo " $env_teil " | grep -q '[[:space:]]CFD_POSITIV_HAKEN='; then
		[ -n "${CFD_POSITIV_HAKEN:-}" ] && [ "${CFD_POSITIV_HAKEN:-0}" != "0" ] && geerbt_haken=1
	fi
	if { echo " $env_teil " | grep -Eq '[[:space:]]CFD_(KLEMM|POSITIV|FAC_APG)_HAKEN=\+?0*[1-9]' || [ "$geerbt_haken" = "1" ]; } && ! echo " $env_teil " | grep -q ' CFD_CASE=kugel ' \
	   && ! { echo " $env_teil " | grep -q ' CFD_DEV_FINE=0 ' && echo " $env_teil " | grep -q ' CFD_DEV_COARSE=0 '; }; then
		echo "[$(date +%H:%M:%S)] VERWEIGERT $n/$gesamt: $name -- Atomik-Testhaken ausserhalb der Kugel ohne CFD_DEV_FINE=0/CFD_DEV_COARSE=0 (Absturzsperre 15.09.; Ausloeser: $([ "$geerbt_haken" = 1 ] && echo "GEERBTE UMGEBUNG" || echo "Zeilenmuster"))" | tee -a "$Q"
		continue
	fi
	# ★★ 22.09.2026 DIAGNOSESPERRE FUER ZEITMESSER. Anlass: am 21.09. stand CFD_TIMER_FERN=1 in einer
	# PRODUKTIONSZEILE -- +101 % Wanduhr, ein ganzer Lauf verloren. Eine print_warning reicht nachweislich
	# NICHT: CFD_TIMER_FERN warnt seit dem 16.09., und genau das ist trotzdem passiert. Eine Queue, die
	# nachts laeuft, liest keine Warnungen.
	# KEIN Verbot, sondern ein NAMENSZWANG: der Arm darf fahren, wenn der Laufname "timer" traegt. Dann
	# heisst auch jede Logdatei so, und niemand liest die Wanduhr spaeter als Leistungsmass.
	# Muster exakt wie die Atomik-Sperre oben, inklusive der Lehren vom 16.09. (Befund B7): '+' und
	# fuehrende Nullen zulassen ("=01" und "=+1" ergeben in atoi() ebenfalls 1), [[:space:]] als Anker
	# statt Leerzeichen (ein TAB haette die Sperre sonst umgangen), und die GEERBTE Umgebung mitpruefen,
	# aber nur, wenn die Zeile die Variable nicht selbst setzt (Befund M4).
	geerbt_timer=0
	for V in CFD_TIMER_FERN CFD_TIMER_APG; do
		if ! echo " $env_teil " | grep -q "[[:space:]]$V="; then
			eval "w=\${$V:-0}"; [ "$w" != "0" ] && geerbt_timer=1
		fi
	done
	if { echo " $env_teil " | grep -Eq '[[:space:]]CFD_TIMER_(FERN|APG)=\+?0*[1-9]' || [ "$geerbt_timer" = "1" ]; } \
	   && ! echo "$name" | grep -q 'timer'; then
		echo "[$(date +%H:%M:%S)] VERWEIGERT $n/$gesamt: $name -- Diagnose-Zeitmesser (CFD_TIMER_FERN/CFD_TIMER_APG) ohne 'timer' im Laufnamen. Diese Arme SERIALISIEREN, ihre Wanduhr ist kein Leistungsmass (Sperre 22.09.; Anlass CFD_TIMER_FERN am 21.09., +101 Prozent Wanduhr)" | tee -a "$Q"
		continue
	fi
	if [ "${CFD_QUEUE_DEV:-2}" = "1" ] && command -v journalctl >/dev/null 2>&1; then
		cat_n=$(journalctl -k --since '-3min' --no-pager 2>/dev/null | grep -ac 'Engine memory CAT error')
		if [ "${cat_n:-0}" -gt 0 ]; then
			echo "[$(date +%H:%M:%S)] GPU-WAECHTER: $cat_n CAT-Error(s) in den letzten 3 min -- warte 60 s vor $name (Haengegefahr, 2x belegt)" | tee -a "$Q"
			sleep 60
		fi
	fi
	# ★ 17.09.2026 BINARY-WAHL (D3Q27-A/B): CFD_VELSET=27 in der ZEILE waehlt bin_q27/FluidX3D (gebaut von werkzeuge/bau_q27.sh).
	# Geerbtes CFD_VELSET zaehlt nicht fuer die Wahl -- main_setup bricht dann ab, weil Build-Satz und CFD_VELSET nicht passen.
	BIN=bin/FluidX3D
	if echo " $env_teil " | grep -Eq '[[:space:]]CFD_VELSET=27[[:space:]]'; then BIN=bin_q27/FluidX3D; fi
	# 17.09.2026 UPSTREAM-LAUF (Heiko: Upstream-FluidX3D, Fahrzeug Single-Domain 4 mm): CFD_BIN=upstream in der Zeile waehlt den
	# festen Pfad des Upstream-Klons (Zweig mr2-singledomain). Positivliste, kein freier Pfad; zusammen mit CFD_VELSET=27 verweigert.
	if echo " $env_teil " | grep -Eq '[[:space:]]CFD_BIN=upstream[[:space:]]'; then
		if [ "$BIN" != "bin/FluidX3D" ]; then
			echo "[$(date +%H:%M:%S)] VERWEIGERT $n/$gesamt: $name -- CFD_BIN=upstream und CFD_VELSET=27 in einer Zeile" | tee -a "$Q"; continue
		fi
		BIN=/home/heiko/CFD/FluidX3D-upstream/bin/FluidX3D
	elif echo " $env_teil " | grep -Eq '[[:space:]]CFD_BIN='; then
		echo "[$(date +%H:%M:%S)] VERWEIGERT $n/$gesamt: $name -- CFD_BIN nur mit dem Wert upstream erlaubt" | tee -a "$Q"; continue
	fi
	if [ ! -x "$BIN" ]; then
		echo "[$(date +%H:%M:%S)] VERWEIGERT $n/$gesamt: $name -- Binary $BIN fehlt oder ist nicht ausfuehrbar" | tee -a "$Q"
		continue
	fi
	# ★★ 22.09.2026 PLATTENPLATZ. Anlass: am 21.09. lief die Platte auf 0 Bytes, p4_regel6 rechnete
	# danach 28 min normal weiter und schrieb KEINE einzige Kraft mehr -- std::ofstream setzt bei
	# ENOSPC badbit und verwirft danach still jeden Schreibvorgang, bis clear() gerufen wird. Das ruft
	# niemand. 15 von 17 CSV standen still, waehrend der Lauf lief. Hier wird der Stand nur FESTGEHALTEN
	# (Vorher/Nachher, und der Nachher-Wert traegt den Verdachtstest unten); die harte SCHRANKE gehoert
	# in den Code, wo Gittergroesse, CFD_VTK_DT und CFD_T_END bekannt sind und der Bedarf exakt ist.
	frei_vor=$(df --output=avail . 2>/dev/null | tail -1 | tr -d " ")
	t_vor=$(date +%s)
	echo "[$(date +%H:%M:%S)] START $n/$gesamt: $name (Binary $BIN, frei $(( ${frei_vor:-0} / 1048576 )) GB)" | tee -a "$Q"
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
		env $env_teil CFD_RUN_NAME="$name" "$BIN" "${CFD_QUEUE_DEV:-2}" < /dev/null > "logs/$name.log" 2>&1 &
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
	# ★★ 06.09.2026 WAECHTERAUSGABEN SICHTBAR MACHEN.
	# ★ BERICHTIGT 10.09. nachts: der Satz hier lautete "print_error bricht den Lauf NICHT ab,
	# also meldet rc weiterhin 0". DAS IST FALSCH. Beide Fassungen von print_error in
	# src/utilities.hpp enden auf exit(1), belegt an logs/pt3_komma.log (6 kB statt 170 kB,
	# Log endet unmittelbar hinter dem Error, rc=1). Der 06.09.-Befund "15 von 80 Logs mit
	# unbemerktem Error" bleibt richtig -- diese Errors feuern am LAUFENDE, nach den CSVs,
	# die Physik ueberlebt, rc ist trotzdem 1. Die Annahme steuert, wie Abnahmen gebaut
	# werden: ein Waechter vor den uebrigen Abnahmen reisst sie alle mit.
	# Am 06.09. trugen 15 von 80 Lauflogs einen unbemerkten Error, darunter
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
	# ★★ 22.09.2026 PLATTENSTAND NACH DEM LAUF (Uebergabe 21.09. §5).
	# Der ENOSPC-Fall vom 21.09. war von aussen UNSICHTBAR: rc=0, keine Errors, Log lief weiter, nur die
	# CSVs standen. Was die Queue dazu beitragen kann, ist der Plattenstand vorher/nachher; die
	# eigentliche Erkennung sitzt im Code (ofstream-Zustand an der Sample-Kadenz, setup.cpp).
	frei_nach=$(df --output=avail . 2>/dev/null | tail -1 | tr -d " ")
	t_nach=$(date +%s)
	echo "          Platte: $(( ${frei_vor:-0} / 1048576 )) -> $(( ${frei_nach:-0} / 1048576 )) GB frei (verbraucht $(( (${frei_vor:-0} - ${frei_nach:-0}) / 1048576 )) GB), Wanduhr $(( (t_nach - t_vor) / 60 )) min" | tee -a "$Q"
	if [ "${frei_nach:-1}" -eq 0 ]; then
		echo "          !! PLATTE VOLL nach $name -- jede weitere CSV-Zeile geht STILL verloren (ofstream badbit). Kette pruefen, bevor der naechste Lauf startet." | tee -a "$Q"
	fi
	# ★ 22.09.2026 VERWORFEN, und der Grund gehoert hierher: ich hatte hier einen mtime-Waechter
	# ("CSV, die mehr als 10 % der Laufdauer vor dem Laufende zuletzt geschrieben wurde"). Er faengt den
	# Schadensfall p4_regel6 vom 21.09. sauber (14 CSV standen 2083 s vor der letzten Ausgabe still),
	# ABER er schlaegt am GESUNDEN Lauf p4_regel7 falsch an: facetten_histogramme.csv wird EINMAL beim
	# Facettenaufbau geschrieben und liegt dort 8426 s vor Laufende. Ein Waechter, der am guten Lauf
	# meldet, wird weggesehen -- und dann meldet er auch beim schlechten umsonst. Die tragfaehige
	# Pruefung ist der STREAM-ZUSTAND im Code (ofstream fail/bad nach jedem Schreibblock), nicht die
	# Dateizeit von aussen. Siehe Uebergabe 21.09. §5 Punkt 2.
	# ★★ 10.09.2026 KRAFTVERLAUF ALS STANDBILD-SERIE (Heiko-Vorgabe): alle 100 ms physikalisch
	# ein Bild nach export/<lauf>/kraftverlauf_000300ms.png usw., dazu kraftverlauf.png ueber den
	# ganzen Lauf. Laeuft NACH dem Lauf im SELBEN Kettenglied -- kein eigener Waechterprozess,
	# Iron Rule 4 bleibt gewahrt. Die Bilder entstehen aus cd_facetten.csv, also nachtraeglich
	# genau so, wie sie waehrend des Laufs entstanden waeren. Fehler hier duerfen die Kette NICHT
	# abbrechen: es ist Auswertung, nicht Messung.
	# ★ 22.09.2026: die Gatterung war STILL -- bei leerer oder fehlender cd_facetten.csv blieben die
	# Kraftbilder kommentarlos aus (Uebergabe 21.09. §5). Jetzt sagt sie, warum sie nichts tut.
	if [ "${CFD_QUEUE_KRAFTBILD:-1}" != "0" ] && [ ! -s "export/$name/cd_facetten.csv" ]; then
		echo "          HINWEIS: keine Kraftbilder -- export/$name/cd_facetten.csv fehlt oder ist leer (Kraftausgabe hat nie oder nicht mehr geschrieben)." | tee -a "$Q"
	fi
	if [ "${CFD_QUEUE_KRAFTBILD:-1}" != "0" ] && [ -s "export/$name/cd_facetten.csv" ]; then
		if python3 werkzeuge/kraftverlauf.py "$name" --serie "${CFD_QUEUE_KRAFTBILD_MS:-100}" > "logs/$name.kraftverlauf.log" 2>&1; then
			echo "          Kraftverlauf: $(grep -c 'geschrieben:' "logs/$name.kraftverlauf.log") Bilder in export/$name/" | tee -a "$Q"
		else
			echo "          HINWEIS: kraftverlauf.py fehlgeschlagen ($name) -- siehe logs/$name.kraftverlauf.log" | tee -a "$Q"
		fi
	fi
done < "$1"
echo "[$(date +%H:%M:%S)] SERIE FERTIG ($n Laeufe)" | tee -a "$Q"
