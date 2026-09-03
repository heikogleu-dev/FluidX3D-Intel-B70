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
import sys, os, re
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
 "CFD_SGS_FDWAND":"modus","CFD_FAC_NACHBAR":"modus",  # ★ 03.09.2026 in die Basis aufgenommen (Heiko-Entscheid)
 "CFD_KOPPLUNG_GLATT":"modus","CFD_N2F_SCHALE":"modus","CFD_N2F_BAND":"modus",
 "CFD_N2F_BAND_PROFIL":"modus","CFD_N2F_BAND_PLATEAU":"modus","CFD_N2F_BAND_WANDFREI":"modus",
 "CFD_N2F_BAND_WAKE":"modus","CFD_FAC_UTKORR":"modus","CFD_FAC_ELIBB":"modus",
 "CFD_FACETTEN_YWMIN":"modus","CFD_FAC_CD_EVERY":"ausgabe","CFD_VTK_ENDE":"ausgabe",
 "CFD_VTK_DT":"ausgabe","CFD_SLICE_DT":"ausgabe","CFD_RUN_NAME":"ausgabe",
 "CFD_CASE":"modus",
}
# ★ KORREKTUREN AN DER QUELLE (Heiko 28.08.): der Baseline-Lauf traegt CFD_SLICE_DT=0 und
# schreibt damit GAR KEINE Slices -- ein Defekt, den ich selbst eingebaut hatte und der sich
# ueber die Referenz in jeden neuen Lauf fortgepflanzt haette. Heiko: "sliceausgabe muss an
# sein! kostet nichts". Die Referenz ist eine VORLAGE, kein Archiv: sie traegt den richtigen
# Wert, und die Abweichung vom aufgezeichneten Lauf wird im Kopf vermerkt.
KORREKTUR = { "CFD_SLICE_DT": ("0.1", "Baseline hatte 0 = Slices AUS; das war ein Fehler, nicht Absicht") }
# ★ 03.09.2026: Diagnostik- und Meta-Schalter gehoeren NICHT in die Basis. Ohne diese Liste
# uebernaehme das Werkzeug jeden Beobachter aus dem Quelllauf -- CFD_FAC_KDIAG allein sind am
# 4-mm-Fahrzeug 119 MiB VRAM je Lauf (3.129.185 Facetten x 40 B), und der Basis-Waechter wuerde
# ihn kuenftig bei JEDEM Lauf erzwingen. Sie sind Messinstrumente, keine Modellkonfiguration.
AUSSCHLUSS = {
    "CFD_RUN_NAME",          # Laufname
    "CFD_BASIS_ABWEICHUNG",  # Meta: die Deklaration gegen genau diese Datei
    "CFD_QUEUE_DEV",         # Geraetewahl der Queue, keine Physik
    "CFD_FAC_KDIAG",         # Klassen-Diagnostik (40 B/Facette)
    "CFD_SGS_GDIAG",         # g-Diagnose (32 B/Facette)
    "CFD_SGS_DIAG",          # nu_t-Histogramme
    "CFD_FAC_QDUMP",         # q-Dump je Link
    "CFD_FELD_HASH",         # Bitanker
    "CFD_DUMP_CL",           # Kernelquelltext-Dump
}
if len(sys.argv)<3: sys.exit("Aufruf: basis_aus_lauf.py <LAUF.txt> <ziel.basis>")
s=open(sys.argv[1]).read()
m=re.search(r'Umgebung.*?\n(.*?)(\n\n|\Z)', s, re.S)
env=dict(re.findall(r'(CFD_[A-Z_0-9]+)=([^\s]+)', m.group(1) if m else s))
commit=(re.search(r'Git-Commit\s*:\s*([0-9a-f]+)', s) or [None,"unbekannt"])[1]
dx=env.get("CFD_DX","?")
unbekannt=[k for k in env if k not in EINHEIT and k not in AUSSCHLUSS]
# ★ 03.09.2026: Die Werte kommen maschinell aus dem Lauf -- die BEGRUENDUNGEN aber (warum ein Wert
# so ist, welcher Heiko-Entscheid dahinter steht, welche Messung ihn traegt) sind das Gedaechtnis
# dieser Datei. Beim ersten Lauf dieses Werkzeugs gegen die bestehende Basis waeren 29 solche Zeilen
# spurlos verschwunden. Deshalb: alles ab der Marke unten wird aus der Zieldatei UEBERNOMMEN.
BEGRUENDUNGSMARKE = "# --- BEGRUENDUNGEN (bleiben bei Neuerzeugung erhalten) ---"
uebernommen = []
if os.path.exists(sys.argv[2]):
    alt_zeilen = open(sys.argv[2]).read().splitlines()
    if BEGRUENDUNGSMARKE in alt_zeilen:
        i = alt_zeilen.index(BEGRUENDUNGSMARKE)
        uebernommen = [z for z in alt_zeilen[i+1:] if z.startswith("#")]
    else:  # Erstlauf: alle Kommentare retten, die nicht vom Werkzeug selbst stammen
        eigen = ("# Basis-Referenz", "# Quelle-Commit", "# dx_ref", "# Spalten",
                 "# ACHTUNG, Einheit unbekannt", "# KORRIGIERT gegen den Lauf")
        uebernommen = [z for z in alt_zeilen if z.startswith("#") and not z.startswith(eigen)]

with open(sys.argv[2],"w") as f:
    f.write(f"# Basis-Referenz, MASCHINELL erzeugt aus {sys.argv[1]}\n")
    f.write(f"# Quelle-Commit: {commit}\n# dx_ref: {dx}\n")
    f.write("# Spalten: NAME WERT EINHEIT\n")
    if uebernommen:
        f.write(BEGRUENDUNGSMARKE+"\n")
        for z in uebernommen: f.write(z+"\n")
    if unbekannt:
        f.write("# ACHTUNG, Einheit unbekannt (als modus gefuehrt, bitte einordnen): "+", ".join(sorted(unbekannt))+"\n")
    for k,(v,grund) in KORREKTUR.items():
        if k in env and env[k]!=v: f.write(f"# KORRIGIERT gegen den Lauf: {k} {env[k]} -> {v} ({grund})\n")
    for k in sorted(env):
        if k in AUSSCHLUSS: continue  # ★ 03.09.: Diagnostik/Meta raus
        wert = KORREKTUR[k][0] if k in KORREKTUR else env[k]
        f.write(f"{k} {wert} {EINHEIT.get(k,'modus')}\n")
print(f"geschrieben: {sys.argv[2]}  ({len(env)-1} Schalter, Quelle {commit[:7]}, dx {dx})")
if unbekannt: print("  Einheit unbekannt bei:", ", ".join(sorted(unbekannt)))
