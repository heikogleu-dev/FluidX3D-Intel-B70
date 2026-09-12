#!/bin/bash
# rho_format.sh FP32|FP16 -- schaltet das SPEICHERFORMAT VON RHO in src/defines.hpp und baut neu.
# Muster: zahlenformat.sh (das dasselbe fuer die Verteilungen tut). Warum ein Uebersetzungsschalter
# und kein env-Schalter: der Puffertyp ist ein C++-Typ (Memory<rhoxx>), der steht zur Laufzeit fest.
#   FP32 = rho als float32, 4 Byte je Zelle -- der Stand vor dem 12.09.2026.
#   FP16 = rho als FP16S(rho-1), 2 Byte je Zelle. Spart bei 4 mm 990 MiB VRAM im Nahfeld.
# Der FP32-Arm MUSS bitgleich zum Stand davor bleiben; das ist das einzige Sicherheitsnetz dieses
# Umbaus (Schritt 4 aendert Werte, Bitgleichheit im FP16-Arm ist ausgeschlossen).
set -eu
cd "$(dirname "$0")/.."
case "${1:-}" in
  FP32) N='//#define RHO_FP16' ;;
  FP16) N='#define RHO_FP16'   ;;
  *) echo "Aufruf: rho_format.sh FP32|FP16" >&2; exit 2 ;;
esac
python3 - "$N" <<'PY'
import io,re,sys
p="src/defines.hpp"; s=io.open(p,encoding="utf-8",newline="").read()
z=s.split("\n"); n=0
for i,l in enumerate(z):
    if re.match(r'^\s*(//)?#define RHO_FP16\b', l):
        rest=l[l.find("RHO_FP16")+len("RHO_FP16"):]   # den Erklaerkommentar dahinter behalten
        z[i]=sys.argv[1]+rest; n+=1
assert n==1, f"RHO_FP16: {n} Treffer statt 1 -- nichts geschrieben"
io.open(p,"w",encoding="utf-8",newline="").write("\n".join(z))
PY
grep -nE '^\s*(//)?#define RHO_FP16' src/defines.hpp | cut -c1-60
# ★ BERICHTIGT 12.09. (Pruefagent, NIEDRIG): hier stand "make ... | grep -i error && exit 1".
# Ohne pipefail geht der Exit-Status von make verloren, die ganze Bauausgabe verschwindet im grep,
# und ein Bau, der auf eine Art scheitert die kein " error" druckt, haette "gebaut" gemeldet --
# mit dem md5 des VORIGEN Binaries. Das ist die Build-RC-Falle dieses Projekts in ihrer dritten
# Auflage. Jetzt: Ausgabe mitschreiben, RC von make lesen, erst dann melden.
LOG="$(mktemp)"
set +e
make -j"$(nproc)" Linux > "$LOG" 2>&1
RC=$?
set -e
if [ "$RC" -ne 0 ]; then echo "BAU FEHLGESCHLAGEN (make RC=$RC):"; tail -20 "$LOG"; rm -f "$LOG"; exit 1; fi
grep -iE ' error|Error ' "$LOG" && { echo "BAU FEHLGESCHLAGEN (Fehlertext trotz RC=0)"; rm -f "$LOG"; exit 1; }
rm -f "$LOG"
echo "gebaut: rho = $1   ($(md5sum bin/FluidX3D | cut -d' ' -f1))"
