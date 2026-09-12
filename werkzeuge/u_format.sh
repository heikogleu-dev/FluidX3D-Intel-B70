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

# ── HOST-ZENSUS ────────────────────────────────────────────────────────────────────────
# Das Gegenstueck zum Typ-Zensus auf dem OpenCL-Quelltext (lbm.cpp) -- fuer die HOSTSEITE gibt es
# keines, und ohne dieses hier haette der einzige stille Hostfehler dieses Umbaus keinen Fang.
# Die Kapsel U_Feld macht "lbm.u.x[n]" sicher: der Stellvertreter wandelt ueber u_unpack/u_pack,
# und ein roher velxx& ist nicht erreichbar. Sie deckt aber NICHT den Direktzugriff auf den
# DOMAENENpuffer, "L.lbm_domain[0]->u.x[n]" -- das ist Memory<velxx>::Pointer und liefert ein
# rohes Speicherwort. ushort nach float ist keine Verengung und warnt nicht: es uebersetzt, es
# laeuft, und es rechnet Muell. Genau eine solche Stelle gab es (setup.cpp, schreibe_wandprofil),
# sie ist von Hand auf u_unpack umgestellt. Soll ab jetzt: jeder Treffer steht in u_unpack(...).
python3 - <<'PY2'
import io,re,sys
schlecht=[]
for p in ("src/setup.cpp","src/lbm.cpp"):
    for nr,l in enumerate(io.open(p,encoding="utf-8",newline="").read().split("\n"), 1):
        if l.lstrip().startswith("//"): continue          # Kommentarzeilen zaehlen nicht
        for m in re.finditer(r'->u\.[xyz]\[', l):
            # Zulaessig ist genau ein Kontext: der Zugriff steht als Argument in u_unpack(...).
            # Dazu vom Treffer aus RUECKWAERTS ueber die Zugriffskette laufen (Bezeichner, Punkte,
            # Pfeile, Indexklammern) und pruefen, ob davor u_unpack( steht. Ein Test auf "endet
            # direkt mit u_unpack(" reicht NICHT -- zwischen dem Aufruf und dem ->u.x[ steht die
            # ganze Kette "L.lbm_domain[0]". Genau daran ist die erste Fassung dieses Waechters
            # gescheitert: sie meldete die BEHOBENE Stelle als Verletzung, und der Negativtest
            # sah deshalb gleich aus wie der Positivlauf -- er bewies nichts.
            k = m.start()
            while k > 0 and (l[k-1].isalnum() or l[k-1] in "_.[]>-"): k -= 1
            if not l[:k].endswith("u_unpack("):
                schlecht.append(f"{p}:{nr}: {l.strip()[:100]}")
if schlecht:
    print("HOST-ZENSUS VERLETZT -- roher Zugriff auf den Domaenenpuffer (Soll 0):")
    for x in schlecht: print("  "+x)
    sys.exit(1)
print("Host-Zensus: 0 rohe Zugriffe auf ->u.x/y/z (Soll 0)")
PY2
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
