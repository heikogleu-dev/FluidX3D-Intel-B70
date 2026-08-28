#!/usr/bin/env python3
"""Erzeugt eine Basis-Referenzdatei aus der LAUF.txt eines gesicherten Laufs.

MASCHINELL, NIE VON HAND: eine handgepflegte Referenz weicht irgendwann von dem ab, was
tatsaechlich lief -- genau das ist am 28.08.2026 passiert (logs/f4_vollumfang_serie.txt trug
CFD_VTK_JEDE=1, die LAUF.txt des getaggten Laufs stattdessen CFD_VTK_DT=0.15 und CFD_SLICE_DT=0;
die Seriendatei war als Referenz schon ungueltig).

Die EINHEIT je Schalter entscheidet, wie der Waechter bei anderer Aufloesung umrechnet:
  phys        Meter/Sekunden/Millimeter/dimensionslos -- bleibt gleich
  zellen_fein skaliert mit dx_ref/dx  (Feingitter-Zellen)
  zellen_grob bleibt gleich           (Grobzellen; Codedoktrin setup.cpp:3798-3801)
  index_grob  absoluter Grobzell-Index -- skaliert, Ergebnis oft uneindeutig
  modus       Schaltzahl/Flag -- bleibt gleich
  ausgabe     beruehrt die Loesung nicht -- ungeprueft
"""
import sys, re, os
EINHEIT = {
 "CFD_DX":"phys", "CFD_T_WARMUP":"phys", "CFD_T_END":"phys", "CFD_NEAR_VOR_MM":"phys",
 "CFD_NEAR_LX":"phys", "CFD_NEAR_LY":"phys", "CFD_NEAR_LZ":"phys",
 "CFD_SPONGE_N":"zellen_grob_laenge",   # Sonderfall: gleiche WELTlaenge, siehe Waechter
 "CFD_KRAFT_ZBAND":"zellen_fein",
 "CFD_N2F_BAND_N":"zellen_grob",
 "CFD_N2F_BAND_WAKE_ABSTAND":"zellen_grob_laenge",
 "CFD_N2F_BAND_WAKE_START_X":"index_grob",
 "CFD_BODEN_EQ":"modus","CFD_BODEN_EQ_DOWN":"modus","CFD_BODEN_EQ_ABSTAND":"modus",
 "CFD_FERN_BODEN_EQ":"modus","CFD_FERN_BODEN_EQ_DOWN":"modus","CFD_FERN_EINLASS_EQ":"modus",
 "CFD_FACETTEN":"modus","CFD_FAC_SATGATE":"modus","CFD_FAC_ALPHA":"modus",
 "CFD_KOPPLUNG_GLATT":"modus","CFD_N2F_SCHALE":"modus","CFD_N2F_BAND":"modus",
 "CFD_N2F_BAND_PROFIL":"modus","CFD_N2F_BAND_PLATEAU":"modus","CFD_N2F_BAND_WANDFREI":"modus",
 "CFD_N2F_BAND_WAKE":"modus","CFD_FAC_UTKORR":"modus","CFD_FAC_ELIBB":"modus",
 "CFD_FACETTEN_YWMIN":"modus","CFD_FAC_CD_EVERY":"ausgabe","CFD_VTK_ENDE":"ausgabe",
 "CFD_VTK_DT":"ausgabe","CFD_SLICE_DT":"ausgabe","CFD_RUN_NAME":"ausgabe",
 "CFD_CASE":"modus",
}
if len(sys.argv)<3: sys.exit("Aufruf: basis_aus_lauf.py <LAUF.txt> <ziel.basis>")
s=open(sys.argv[1]).read()
m=re.search(r'Umgebung.*?\n(.*?)(\n\n|\Z)', s, re.S)
env=dict(re.findall(r'(CFD_[A-Z_0-9]+)=([^\s]+)', m.group(1) if m else s))
commit=(re.search(r'Git-Commit\s*:\s*([0-9a-f]+)', s) or [None,"unbekannt"])[1]
dx=env.get("CFD_DX","?")
unbekannt=[k for k in env if k not in EINHEIT]
with open(sys.argv[2],"w") as f:
    f.write(f"# Basis-Referenz, MASCHINELL erzeugt aus {sys.argv[1]}\n")
    f.write(f"# Quelle-Commit: {commit}\n# dx_ref: {dx}\n")
    f.write("# Spalten: NAME WERT EINHEIT\n")
    if unbekannt:
        f.write("# ACHTUNG, Einheit unbekannt (als modus gefuehrt, bitte einordnen): "+", ".join(sorted(unbekannt))+"\n")
    for k in sorted(env):
        if k=="CFD_RUN_NAME": continue
        f.write(f"{k} {env[k]} {EINHEIT.get(k,'modus')}\n")
print(f"geschrieben: {sys.argv[2]}  ({len(env)-1} Schalter, Quelle {commit[:7]}, dx {dx})")
if unbekannt: print("  Einheit unbekannt bei:", ", ".join(sorted(unbekannt)))
