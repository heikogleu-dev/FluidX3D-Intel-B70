#!/bin/bash
# u_format.sh FP32|FP16 -- schaltet das SPEICHERFORMAT VON u in src/defines.hpp und baut neu.
# Muster: rho_format.sh (TODO 2 Schritt 4 fuer rho) und zahlenformat.sh (dasselbe fuer die
# Verteilungen). Warum ein Uebersetzungsschalter und kein env-Schalter: der Puffertyp ist ein
# C++-Typ (Memory<velxx>), der steht zur Laufzeit fest.
#   FP32 = u als drei float32, 12 Byte je Zelle -- der Stand vor dem 12.09.2026.
#   FP16 = u als drei FP16S, 6 Byte je Zelle. Spart bei 4 mm 2971 MiB VRAM im Nahfeld
#          und noch einmal dieselbe Menge System-RAM fuer den Hostspiegel.
# Der FP32-Arm MUSS bitgleich zum Stand davor bleiben; das ist das einzige Sicherheitsnetz
# dieses Umbaus (Schritt 4 aendert Werte, Bitgleichheit im FP16-Arm ist ausgeschlossen).
set -eu
cd "$(dirname "$0")/.."
case "${1:-}" in
  FP32) N='//#define U_FP16' ;;
  FP16) N='#define U_FP16'   ;;
  *) echo "Aufruf: u_format.sh FP32|FP16" >&2; exit 2 ;;
esac
python3 - "$N" <<'PY'
import io,re,sys
p="src/defines.hpp"; s=io.open(p,encoding="utf-8",newline="").read()
z=s.split("\n"); n=0
for i,l in enumerate(z):
    if re.match(r'^\s*(//)?#define U_FP16\b', l):
        rest=l[l.find("U_FP16")+len("U_FP16"):]   # den Erklaerkommentar dahinter behalten
        z[i]=sys.argv[1]+rest; n+=1
assert n==1, f"U_FP16: {n} Treffer statt 1 -- nichts geschrieben"
io.open(p,"w",encoding="utf-8",newline="").write("\n".join(z))
PY
grep -nE '^\s*(//)?#define U_FP16' src/defines.hpp | cut -c1-60
# Bauwaechter wortgleich zu rho_format.sh: ohne pipefail geht der Exit-Status von make verloren
# (Build-RC-Falle dieses Projekts, dritte Auflage). Ausgabe mitschreiben, RC lesen, dann melden.
LOG="$(mktemp)"
set +e
make -j"$(nproc)" Linux > "$LOG" 2>&1
RC=$?
set -e
if [ "$RC" -ne 0 ]; then echo "BAU FEHLGESCHLAGEN (make RC=$RC):"; tail -20 "$LOG"; rm -f "$LOG"; exit 1; fi
# Auf das g++-Diagnoseformat verankert statt auf " error": 39 Zeilen in src/ enthalten das Wort
# im Klartext, eine davon in einer Warnung gedruckt meldete sonst faelschlich "BAU FEHLGESCHLAGEN".
grep -E '^[^ ]+:[0-9]+:[0-9]+: (fatal )?error:' "$LOG" && { echo "BAU FEHLGESCHLAGEN (Fehlerdiagnose trotz RC=0)"; rm -f "$LOG"; exit 1; }
rm -f "$LOG"
echo "gebaut: u = $1   ($(md5sum bin/FluidX3D | cut -d' ' -f1))"
