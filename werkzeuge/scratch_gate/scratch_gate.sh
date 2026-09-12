#!/usr/bin/env bash
# scratch_gate.sh — 2-Sekunden-Gate gegen die Scratch-Fehlerklasse (Befund 2026-08-26):
# waechst eine Kernel-Schleife ueber IGCs Unroll-Budget, werden laufzeitindizierte
# private Arrays (fhn/fpre/j/c()/w()) speicherheimisch -> private_size>0 im .zeinfo
# -> Faktor ~100 Laufzeit (2 statt 240 MLUPs, "0 GB/s"; g13-g15). Dieses Gate baut den
# AKTUELLEN src/kernel.cpp zur .cl (gen_main.cpp, Defines des Kanal-Referenzfalls),
# kompiliert offline per ocloc (KEIN GPU-Lauf) fuer iGPU und B70 und schlaegt fehl, sobald
# ein Kernel private_size>0 ODER spill_size>0 traegt (seit Rang-1-Remat).
# Legitimes Spill-Wachstum erfordert eine BEWUSSTE Lockerung dieses Gates, nie ein stilles.
#
# ★ 11.09.2026 — GESAMTDECKUNG. Das Gate prueft ab jetzt JEDEN Kernel, nicht nur
# stream_collide. Anlass ist ein Befund der zweiten Agentenrunde: fac_nachbar_ab traegt
# private_size=7296 (= 228 B der c()-Tabelle x 32 Lanes) und ist damit GENAU die
# Fehlerklasse, gegen die dieses Gate gebaut wurde -- unentdeckt, weil hier bis heute
# "stream_collide" fest verdrahtet stand. Ein Waechter, der nur an einer Stelle hinsieht,
# ist kein Waechter.
#
# Aufruf: werkzeuge/scratch_gate/scratch_gate.sh    (beliebiges Arbeitsverzeichnis)
# Exit 0 = sauber, Exit 1 = Scratch ODER Spill zurueck. Referenz 26.08.2026 nachmittags
# (Rang-1-Remat): stream_collide private 0 UND spill 0 in BEIDEN Armen auf BEIDEN Geraeten.
# Historie: vor Unroll-Fix private 4256/8512; vor Remat spill 448/832 (Prod) bzw. 672/1216 (ELIBB).
set -eu
HIER="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HIER/../.." && pwd)"
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT

# ── BEKANNTE, AUSDRUECKLICH ERKLAERTE ABWEICHUNGEN ────────────────────────────────────
# Format: "<kernelname>:<private>:<spill>". Nur exakt diese Werte gelten als bekannt --
# waechst die Zahl, schlaegt das Gate zu. Ein Eintrag hier ist eine SCHULD, kein Freibrief:
# er gehoert entfernt, sobald der Befund behoben ist, und er braucht immer eine Begruendung.
#
#   (leer) — der Eintrag fac_nachbar_ab:7296:0 / :3648:0 wurde am 11.09.2026 behoben und
#   entfernt: der laufzeitindizierte c(ib)-Zugriff in kernel.cpp ist durch eine Mitschrift
#   in der Schleife ersetzt. Das Gate hat den Eintrag selbst als veraltet gemeldet.
BEKANNT=""

g++ -O1 -c "$REPO/src/kernel.cpp" -o "$T/kernel.o"
g++ -O1 "$HIER/gen_main.cpp" "$T/kernel.o" -o "$T/gen"
# ★ 11.09.2026: VIER Arme statt zwei. Ohne den PTRT-Arm prueft das Gate den
# Produktionsstand gar nicht -- der #ifdef PTRT-Block in kernel.cpp bleibt inert,
# solange PTRT nicht definiert ist.
# ★ 12.09.2026 RHO-ARM (TODO 2 Schritt 4). Ohne ihn prueft das Gate den 2-Byte-rho-Stand nicht:
# store_rho/load_rho sind dann die Identitaet und der Rueckleser in store_rho_diag fehlt ganz.
# Zwoelf statt vier Arme; das Gate bleibt damit unter fuenfzehn Sekunden.
"$T/gen" on  on  off off off "$T/e1p1.cl" >/dev/null
"$T/gen" on  off off off off "$T/e1p0.cl" >/dev/null
"$T/gen" off on  off off off "$T/e0p1.cl" >/dev/null
"$T/gen" off off off off off "$T/e0p0.cl" >/dev/null
"$T/gen" on  on  on  off off "$T/e1p1r.cl" >/dev/null
"$T/gen" on  off on  off off "$T/e1p0r.cl" >/dev/null
"$T/gen" off on  on  off off "$T/e0p1r.cl" >/dev/null
"$T/gen" off off on  off off "$T/e0p0r.cl" >/dev/null
# ★ 12.09.2026 (Audit-Schleife, Pruefer B): die beiden PRODUKTIONSARME. Nach TODO 2 laeuft die
# Produktion mit CFD_RHO_SPARSAM und CFD_U_SPARSAM, und deren Zweige haengen an stream_collide --
# dem Kernel, an dem sich Scratch entscheidet. Ohne diese zwei Arme prueft das Gate acht Varianten,
# aber nicht die, die gerechnet wird. Kein volles Kreuz (16 Arme): geprueft wird der Produktionspunkt
# ELIBB an, PTRT an, SPARSAM an, beide rho-Formate.
"$T/gen" on  on  off on  off "$T/e1p1s.cl" >/dev/null
"$T/gen" on  on  on  on  off "$T/e1p1rs.cl" >/dev/null
# ★ 12.09.2026 U-ARM (TODO 2 Schritt 4 fuer u). Zwei weitere Arme statt eines vollen Kreuzes (32):
# geprueft wird der PRODUKTIONSPUNKT ELIBB an, PTRT an, SPARSAM an -- einmal mit u16 allein und
# einmal mit beiden 2-Byte-Feldern. Das ist die Kombination, die gerechnet wird; ein Gate, das
# acht Varianten prueft und die gefahrene nicht, hat dieses Projekt schon einmal bezahlt.
"$T/gen" on  on  off on  on  "$T/e1p1su.cl" >/dev/null
"$T/gen" on  on  on  on  on  "$T/e1p1rsu.cl" >/dev/null

rc=0
neu_bekannt=""
for dev in 0x7d67 0xe223; do
  for arm in e1p1 e1p0 e0p1 e0p0 e1p1r e1p0r e0p1r e0p0r e1p1s e1p1rs e1p1su e1p1rsu; do
    ausgabe=$("$HIER/igc_offline.sh" "$T/$arm.cl" "$dev" ALLE || true)
    # ★ 11.09.2026: BAUFEHLER IST NICHT SCRATCH. Vorher fiel ein gescheiterter Bau in beide
    # Gates, weil die Zeile dann schlicht kein "private_size=0" enthielt -- das Gate meldete
    # also "Scratch zurueck", wo in Wahrheit drei Defines fehlten. Zwei verschiedene Befunde
    # unter einer Meldung sind schlimmer als gar keine Meldung.
    if echo "$ausgabe" | grep -q "BUILD FEHLGESCHLAGEN"; then
      echo ">>> BAUFEHLER (nicht Scratch!) in $arm/$dev -- Defines der Zwillingsliste gegen lbm.cpp pruefen"
      rc=1; continue
    fi
    n_kernel=0
    while IFS= read -r zeile; do
      case "$zeile" in *": simd="*) ;; *) continue ;; esac
      n_kernel=$((n_kernel+1))
      kn=${zeile%%:*}
      pv=$(echo "$zeile" | grep -oE 'private_size=[0-9]+' | cut -d= -f2)
      sp=$(echo "$zeile" | grep -oE 'spill_size=[0-9]+'   | cut -d= -f2)
      if [ "${pv:-0}" = "0" ] && [ "${sp:-0}" = "0" ]; then continue; fi
      if echo " $BEKANNT " | grep -q " $kn:$pv:$sp "; then
        echo "    bekannt: $kn private=$pv spill=$sp ($arm/$dev) -- siehe BEKANNT-Liste im Kopf"
        neu_bekannt="$neu_bekannt $kn"
        continue
      fi
      echo ">>> SCRATCH/SPILL-GATE VERLETZT: $kn private=$pv spill=$sp ($arm/$dev)"
      rc=1
    done <<< "$ausgabe"
    # ★ Ein Gate, das nichts findet, weil es nichts SIEHT, ist der eigentliche Defekt.
    # Deshalb ist eine leere Kernelliste selbst ein Fehler.
    if [ "$n_kernel" -lt 5 ]; then
      echo ">>> GATE BLIND: nur $n_kernel Kernel im .zeinfo von $arm/$dev -- Auswertung pruefen"
      rc=1
    else
      echo "$arm $dev: $n_kernel Kernel geprueft"
    fi
  done
done

# Eine BEKANNT-Zeile, die nie zutrifft, ist behoben oder falsch -- beides gehoert gemeldet.
for eintrag in $BEKANNT; do
  kn=${eintrag%%:*}
  case " $neu_bekannt " in *" $kn "*) ;; *)
    echo ">>> BEKANNT-LISTE VERALTET: '$eintrag' trifft nirgends mehr -- Zeile aus dem Kopf entfernen"; rc=1 ;;
  esac
done
exit $rc
