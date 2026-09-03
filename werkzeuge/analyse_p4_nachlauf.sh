#!/bin/bash
# analyse_p4_nachlauf.sh -- Auswertung der 4-mm-Kette NACH dem Serienende (Heiko 03.09.: "lass den
# Lauf jetzt durch und dann kannst Du die restliche Analyse machen"). Alle Schnitte vom 501-ms-Stand.
set -e
V=/home/heiko/CFD/FluidX3D-v2; C=/home/heiko/CFD-Cases/mr2v40H; cd "$V"
if pgrep -x FluidX3D >/dev/null; then echo "VERWEIGERT: FluidX3D laeuft noch."; exit 2; fi

# --- 1. OF13-Schnitt bei z = 0,300 m erzeugen (existiert nicht; vorhanden war nur z = 1,836 m) ---
if [ ! -f "$C/postProcessing/sampleZ300/1200/zp300.xy" ]; then
  echo "== OF13-Sample z=0,300 m"
  ( . /opt/openfoam13/etc/bashrc && cd "$C" && postProcess -func sampleZ300 -time 1200 ) \
    > "$V/logs/of13_sampleZ300.log" 2>&1 && echo "   ok" || { echo "   FEHLER, siehe logs/of13_sampleZ300.log"; tail -5 "$V/logs/of13_sampleZ300.log"; }
fi
Z300="$C/postProcessing/sampleZ300/1200/zp300.xy"
Y025="$C/postProcessing/sampleY0/1200/y0.xy"

# --- 2. Differenzschnitte gegen OF13, je Lauf, IMMER vom 501-ms-Stand ---
for L in p4_fdwand p4_nb; do
  VTK="$V/export/$L/feld_nah_000501ms.vtk"
  [ -f "$VTK" ] || { echo "== $L: feld_nah_000501ms.vtk fehlt -- uebersprungen"; continue; }
  echo "== $L (501 ms)"
  [ -f "$Z300" ] && python3 werkzeuge/diff_of13_zslice.py "$VTK" "export/$L/diff_of13_z300_501ms.png" "$Z300"
  python3 werkzeuge/diff_of13_yslice.py "$VTK" "export/$L/diff_of13_y025_501ms.png" "$Y025"
done

# --- 3. Grenzschicht am Dach (Feld-Daten, Iron Rule 5): Baender -> Profil -> Vergleich ---
for L in p4_fdwand p4_nb; do
  for t in 000300 000450 000501; do
    F="$V/export/$L/feld_nah_${t}ms.vtk"
    [ -f "$F" ] && [ ! -f "werkzeuge/abl_dach/dach_${L}_${t}ms.csv.npz" ] && \
      python3 werkzeuge/abl_dach/fx_band.py "$F" "werkzeuge/abl_dach/dach_${L}_${t}ms.csv.npz"
  done
  ls werkzeuge/abl_dach/dach_${L}_*.npz >/dev/null 2>&1 && \
    python3 werkzeuge/abl_dach/fx_profil2.py "$L" werkzeuge/abl_dach/dach_${L}_*.csv.npz
done
[ -f werkzeuge/abl_dach/prof2_p4_nb.npz ] && python3 werkzeuge/abl_dach/vergleich_nb.py p4_fdwand p4_nb
echo "FERTIG."
