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
  zellen_grob_laenge  gleiche WELTlaenge: skaliert mit dx_ref/dx (dx_c = ratio*dx); Heiko 16.09.2026:
              Abstaende/Masse bleiben physikalisch fest
  index_grob  absoluter Grobzell-Index -- skaliert, Ergebnis oft uneindeutig
  schritte_fein  Zeitschritte, WERT BLEIBT: auf dx_ref und u_lat 0,075 definiert; env_schritte rechnet im
              Lauf laut um (u_lat seit 12.09., dx seit 16.09.2026), der Waechter prueft den ROHEN Wert --
              ein von Hand umgerechneter Wert (7500 bei 8 mm) faellt damit als ABWEICHEND auf
  modus       Schaltzahl/Flag -- bleibt gleich
  ausgabe     beruehrt die Loesung nicht -- ungeprueft
  band_oberkante_mm  (17.09.2026, Heiko) WERT = Sollhoehe H [mm] der wirksamen Kontaktband-Oberkante ueber Welt-z = 0;
              der Lauf traegt N = kraft_zband_regel(H, dx) (basis_zeile.py = setup.cpp): (N-1/2)*dx am naechsten an H,
              Gleichstand -> niedriger, N >= 3 = Keil- UND Deckellage (Heiko 17.09. Option 1; z = 0 ist Fahrbahn). Beim Erzeugen/Nachziehen wird aus der Zellzahl
              des Laufs NICHT zurueckgerechnet (nicht eindeutig) -- der Wert kommt aus SOLL_MM und wird gegen N geprueft.
  lagen       (17.09.2026, Heiko) bewusst GITTERFESTE Lagenzahl -- bleibt gleich, die Dicke waechst mit dx

Modus --nachziehen (16.09.2026): basis_aus_lauf.py --nachziehen <ziel.basis> [--grund=TEXT] [NAME=WERT ...]
  setzt die Einheitenspalte JEDER Datenzeile aus EINHEIT neu (Werte unangetastet), haengt genannte
  Schalter an (Fehler, wenn schon vorhanden) und vermerkt das unter der Begruendungsmarke.
  ★ 17.09.2026: AUSNAHME band_oberkante_mm -- wechselt eine Zeile in diese Einheit, wird ihr Wert (Zellen bei dx_ref)
  durch SOLL_MM ersetzt, nachdem geprueft ist, dass die Regel bei dx_ref genau diese Zellzahl liefert; sonst Abbruch.
"""
import sys, os, re
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from basis_zeile import kraft_zband_regel  # ★ 17.09.2026: EINE Python-Quelle der Kanten-Regel (Gegenstueck setup.cpp)
EINHEIT = {
 "CFD_DX":"phys", "CFD_T_WARMUP":"phys", "CFD_T_END":"phys", "CFD_NEAR_VOR_MM":"phys",
 "CFD_NEAR_LX":"phys", "CFD_NEAR_LY":"phys", "CFD_NEAR_LZ":"phys",
 "CFD_SPONGE_N":"zellen_grob_laenge",   # Sonderfall: gleiche WELTlaenge, siehe Waechter
 "CFD_KRAFT_ZBAND":"band_oberkante_mm",  # ★ 17.09.2026 Heiko (Skalierungsaudit Punkt 1): war zellen_fein = llround(16/dx), Oberkante N*dx beschriftet
 "CFD_N2F_BAND_N":"zellen_grob_laenge",   # ★ 16.09.2026 Heiko: Banddicke bleibt eine LAENGE (war zellen_grob = Anzahl bleibt)
 "CFD_N2F_BAND_WAKE_ABSTAND":"zellen_grob_laenge",
 "CFD_N2F_BAND_WAKE_START_X":"index_grob",
 "CFD_BODEN_EQ":"lagen","CFD_BODEN_EQ_DOWN":"lagen","CFD_BODEN_EQ_ABSTAND":"zellen_fein",  # ★ 16.09.: Chebyshev-Abstand in Feinzellen = Laenge; ★ 17.09. Heiko (Punkt 4): Lagen bewusst gitterfest
 "CFD_FERN_BODEN_EQ":"lagen","CFD_FERN_BODEN_EQ_DOWN":"lagen","CFD_FERN_EINLASS_EQ":"lagen",  # ★ 17.09.: war modus
 "CFD_FACETTEN":"modus","CFD_FAC_SATGATE":"modus","CFD_FAC_ALPHA":"modus",
 "CFD_SGS_FDWAND":"modus","CFD_FAC_NACHBAR":"modus",  # ★ 03.09.2026 in die Basis aufgenommen (Heiko-Entscheid)
 "CFD_KOPPLUNG_GLATT":"modus","CFD_N2F_SCHALE":"modus","CFD_N2F_BAND":"modus",
 "CFD_N2F_BAND_PROFIL":"modus","CFD_N2F_BAND_PLATEAU":"zellen_grob_laenge","CFD_N2F_BAND_WANDFREI":"zellen_grob_laenge",  # ★ 16.09.: Lagen = Laengen
 "CFD_N2F_BAND_WAKE":"modus","CFD_N2F_BAND_WAKE_START":"modus",
 "CFD_FAC_PINV":"modus","CFD_RHO_SPARSAM":"modus","CFD_U_SPARSAM":"modus",
 "CFD_SGS_BAND":"modus","CFD_FAC_APG":"modus","CFD_FAC_UTKORR":"modus","CFD_FAC_ELIBB":"modus",
 "CFD_FACETTEN_YWMIN":"modus","CFD_FAC_CD_EVERY":"ausgabe","CFD_VTK_ENDE":"ausgabe",
 "CFD_VTK_DT":"ausgabe","CFD_SLICE_DT":"ausgabe","CFD_RUN_NAME":"ausgabe",
 "CFD_CASE":"modus",
 # ★ 16.09.2026 (TODO 4a): schrittbasierte Schalter und die bisher ungefuehrten Laengen/Zeiten
 "CFD_SLICE_NEAR_STEPS":"schritte_fein","CFD_SGS_SISM_AB":"schritte_fein","CFD_SGS_SISM_T":"schritte_fein",
 "CFD_SGS_SISM":"modus","CFD_FAR_LX":"phys","CFD_PERF_AB":"phys",
 "CFD_FACETTEN_KANTE_KOH":"modus","CFD_FACETTEN_NORMQUELLE":"modus","CFD_FACETTEN_YWKLEMME":"modus","CFD_F_LISTE":"modus",  # ★ 16.09. (Pruefagent): standen in der Basis, aber nicht hier
 "CFD_Y_VERSATZ":"modus",  # ★ 16.09. 21:55 Heiko: Y-Halbzellen-Versatz ist Standard (1); Bitgleich-Regressionen deklarieren 0
 # ★ 17.09.2026 Heiko (Skalierungsaudit Punkte 12 und 6): Produktionsschalter, die in jeder Serienzeile standen, aber ungeprueft liefen
 "CFD_PTRT":"modus","CFD_FAC_DETEPS":"modus","CFD_POSITIV":"modus","CFD_U_KLEMME":"modus",  # dimensionslos: omega_g, Rauschboden-Faktor, Schaltstufen
 "CFD_ZAEHL_TAKT":"schritte_fein",          # zaehl_takt() rechnet mit schritt_skal() um (lbm.cpp) -- Rohwert bleibt
 "CFD_SCHRITTE_PRO_ZELLE":"modus",          # u_lat = 1/N; Bandkraft ~ 1/u_lat^2 -- ein anderer Wert startet nur deklariert
}
# ★ 17.09.2026: Sollhoehen der Einheit band_oberkante_mm [mm]. Gegenstueck KRAFT_ZBAND_SOLL_MM (setup.cpp) -- der Waechter bricht ab, wenn beide auseinanderlaufen.
SOLL_MM = { "CFD_KRAFT_ZBAND": 16.0 }
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
    # ★ 21.09.2026 KASTEN-REGELWERK (Heiko): die Boxmasse sind KEINE Schalter mehr, sondern werden in
    # setup.cpp aus Fahrzeugmassen und Randfaktoren abgeleitet (BOX-REGELWERK.md). Ohne diesen Ausschluss
    # schreibt eine Neuerzeugung sie wortlos aus irgendeiner alten LAUF.txt zurueck in die Basis -- 257
    # LAUF.txt unter export/ tragen allein CFD_NEAR_VOR_MM. Der Waechter wuerde sie dann wieder erzwingen
    # und das Regelwerk waere ausgehebelt. Wer eine Box bewusst uebersteuert, deklariert das in der
    # Serienzeile; die Ist=Soll-Abnahme im Lauf meldet es.
    "CFD_NEAR_LX", "CFD_NEAR_LY", "CFD_NEAR_LZ",  # Nahfeldbox -> Regelwerk
    "CFD_FAR_LX", "CFD_FAR_LY", "CFD_FAR_LZ",     # Fernfeldbox -> Regelwerk
    "CFD_FAR_X0", "CFD_NEAR_OFF_X",               # Weltlage der Kaesten -> Regelwerk
    "CFD_NEAR_VOR_MM",                            # geht im Regel-X- = 0,1 L auf (21.09.)
    "CFD_N2F_BAND_WAKE_START_X",                  # ersetzt durch CFD_N2F_BAND_WAKE_START=3 (Automatik)
}
BEGRUENDUNGSMARKE = "# --- BEGRUENDUNGEN (bleiben bei Neuerzeugung erhalten) ---"
if len(sys.argv)>=3 and sys.argv[1]=="--nachziehen":
    # ★ 16.09.2026 (PLAN-DX-UMRECHNUNG §D): Einheiten aus EINHEIT neu setzen, Werte NICHT anfassen, genannte
    # Schalter ergaenzen. KEINE Neuerzeugung aus einem Lauf -- Heiko: kein neuer Bezug, basis/ bleibt.
    ziel=sys.argv[2]; grund="(kein Grund angegeben)"; neu={}
    for a in sys.argv[3:]:
        if a.startswith("--grund="): grund=a.split("=",1)[1]; continue
        if "=" not in a: sys.exit(f"Argument '{a}' ist kein NAME=WERT (oder --grund=TEXT).")
        k,v=a.split("=",1); neu[k]=v
    zeilen=open(ziel).read().splitlines()
    kopf=[]; daten={}; kommentare=[]
    for z in zeilen:
        if z.startswith("#"): (kommentare if BEGRUENDUNGSMARKE in kopf else kopf).append(z); continue
        f=z.split()
        if len(f)>=3: daten[f[0]]=(f[1],f[2])
    if BEGRUENDUNGSMARKE not in kopf: sys.exit("Basis ohne Begruendungsmarke -- nicht nachziehbar.")
    geaendert=[]
    dx_ref_kopf=next((float(z.split(":",1)[1]) for z in kopf if z.startswith("# dx_ref:")), None)
    for k,(v,e) in list(daten.items()):
        e2=EINHEIT.get(k)
        if e2 is None: print(f"  WARNUNG: {k} nicht in EINHEIT -- Einheit '{e}' bleibt", file=sys.stderr); continue
        if e2=="band_oberkante_mm":  # ★ 17.09.2026: der Wert wechselt die Bedeutung (Zellen -> Sollhoehe mm)
            if k not in SOLL_MM: sys.exit(f"{k}: Einheit band_oberkante_mm ohne Eintrag in SOLL_MM.")
            h=SOLL_MM[k]
            if e!="band_oberkante_mm":
                if dx_ref_kopf is None: sys.exit("Basis ohne '# dx_ref:' -- Zellzahl nicht pruefbar.")
                n_regel=kraft_zband_regel(h, dx_ref_kopf)
                if abs(float(v)-n_regel)>1e-9: sys.exit(f"{k}: Basiswert {v} ({e}) bei dx_ref {dx_ref_kopf:g} mm ist nicht die Regel-Zellzahl {n_regel} fuer {h:g} mm -- Umstellung verweigert, erst klaeren.")
                geaendert.append(f"{k} {e}->{e2} (Wert {v} Zellen -> Sollhoehe {h:g} mm; Regel bei dx_ref {dx_ref_kopf:g} mm: N = {n_regel}, Oberkante {(n_regel-0.5)*dx_ref_kopf:.3f} mm)")
                daten[k]=(f"{h:g}",e2); continue
            if abs(float(v)-h)>1e-9: sys.exit(f"{k}: Basis traegt {v} mm, SOLL_MM {h:g} mm -- eine Quelle nachziehen (auch KRAFT_ZBAND_SOLL_MM in setup.cpp).")
        if e2!=e: geaendert.append(f"{k} {e}->{e2}"); daten[k]=(v,e2)
    for k,v in neu.items():
        if k in daten: sys.exit(f"{k} steht schon in der Basis ({daten[k][0]}) -- nichts angehaengt.")
        if k not in EINHEIT: sys.exit(f"{k} hat keine Einheit in EINHEIT -- erst dort eintragen.")
        if EINHEIT[k]=="band_oberkante_mm" and abs(float(v)-SOLL_MM.get(k,float("nan")))>1e-9: sys.exit(f"{k}={v}: Einheit band_oberkante_mm erwartet die Sollhoehe aus SOLL_MM ({SOLL_MM.get(k)}).")  # ★ 17.09.
        daten[k]=(v,EINHEIT[k])
    import datetime
    vermerk=("# NACHGEZOGEN (basis_aus_lauf.py --nachziehen, "+datetime.date.today().isoformat()+", Grund: "+grund+"): Einheiten "
             +("; ".join(geaendert) if geaendert else "unveraendert")+("; ergaenzt "+", ".join(f"{k} {v}" for k,v in neu.items()) if neu else "")+".")
    with open(ziel,"w") as f:
        for z in kopf: f.write(z+"\n")
        f.write(vermerk+"\n")
        for z in kommentare: f.write(z+"\n")
        for k in sorted(daten): f.write(f"{k} {daten[k][0]} {daten[k][1]}\n")
    print(f"nachgezogen: {ziel} -- {len(geaendert)} Einheiten geaendert ({', '.join(geaendert)}), {len(neu)} ergaenzt ({', '.join(neu)})")
    sys.exit(0)
if len(sys.argv)<3: sys.exit("Aufruf: basis_aus_lauf.py <LAUF.txt> <ziel.basis>  |  basis_aus_lauf.py --nachziehen <ziel.basis> [NAME=WERT ...]")
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
    # ★ 17.09.2026: band_oberkante_mm traegt die Sollhoehe, nicht die Zellzahl des Laufs; weicht der Lauf von der Regel ab, steht es im Kopf
    band = {}
    for k in env:
        if EINHEIT.get(k)=="band_oberkante_mm" and k not in AUSSCHLUSS:
            h=SOLL_MM[k]; band[k]=f"{h:g}"
            try: dx_f=float(dx); n_regel=kraft_zband_regel(h, dx_f)
            except ValueError: n_regel=None
            if n_regel is None or abs(float(env[k])-n_regel)>1e-9:
                f.write(f"# KORRIGIERT gegen den Lauf: {k} {env[k]} Zellen -> Sollhoehe {h:g} mm (Regel bei dx {dx}: N = {n_regel}; der Lauf wich ab)\n")
                print(f"  WARNUNG: {k}={env[k]} im Lauf, Regel bei dx {dx} ergibt {n_regel} -- Basis traegt trotzdem {h:g} mm", file=sys.stderr)
    for k in sorted(env):
        if k in AUSSCHLUSS: continue  # ★ 03.09.: Diagnostik/Meta raus
        wert = KORREKTUR[k][0] if k in KORREKTUR else band.get(k, env[k])
        f.write(f"{k} {wert} {EINHEIT.get(k,'modus')}\n")
print(f"geschrieben: {sys.argv[2]}  ({len(env)-1} Schalter, Quelle {commit[:7]}, dx {dx})")
if unbekannt: print("  Einheit unbekannt bei:", ", ".join(sorted(unbekannt)))
