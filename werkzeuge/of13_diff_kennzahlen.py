#!/usr/bin/env python3
"""of13_diff_kennzahlen.py -- die EINE Stelle, an der die Konvention der OF13-Differenzschnitte steht,
und die einzige Ausgabe, die ueber Laeufe hinweg vergleichbar ist.

Anlass (Heiko 03.09.2026): "Bekommst das standardisiert ausgegeben, solche Diff Slices zu OF13 haben
wir ja schon paar Mal gemacht." Bis heute druckten diff_of13_yslice.py und diff_of13_zslice.py ihre
Kennzahlen NUR auf die Konsole -- nach dem Schliessen des Terminals war der Vergleich zum vorigen Lauf
weg, und die Konvention stand doppelt in zwei Kopfkommentaren.

KONVENTION (verbindlich fuer alle OF13-Differenzschnitte):
  dU   = |u|_OF13 - |u|_FX   [m/s]      (Betrag der Geschwindigkeit, nicht komponentenweise)
  rot  = OF13 schneller  |  blau = OF13 langsamer, FX ueberbeschleunigt
  Skala +-15 m/s geklemmt; SCHWARZ = Solid ODER kein OF13-Datenpunkt in Reichweite
  x_v2 = x_OF13 + 2,2063 m; y und z sind zwischen beiden Systemen IDENTISCH
  (Quelle der Zuordnung: /home/heiko/CFD-Cases/mr2v40H/system/sampleVergleich2608, Kopf)
  CAVEAT: FX ist ein LES-MOMENTANBILD, OF13 ein RANS-MITTEL. Aussagekraeftig ist die
  grossraeumige Struktur, nicht das einzelne Wirbelpaar.

Die Kennzahlen landen in export/of13_diff_kennzahlen.csv (eine Zeile je Schnitt, anhaengend).
Das Bild ist zum SICHTEN; gemessen wird an dieser Tabelle (Iron Rule 5).
"""
import os, csv, datetime
import numpy as np

SPALTEN = ["zeitstempel","lauf","zeit_ms","ebene","ebene_m","n_auswertbar","rms","mittel",
           "p05","p50","p95","max_abs","anteil_clip_pct","anteil_gt2_pct","png"]

def anhaengen(w, ebene, ebene_m, vtk_pfad, png_pfad, csv_pfad=None):
    """w = maskiertes dU-Array (nur auswertbare Zellen). Gibt den Pfad der CSV zurueck."""
    w = np.asarray(w).ravel()
    if w.size == 0: return None
    hier = os.path.dirname(os.path.abspath(__file__))
    if csv_pfad is None: csv_pfad = os.path.join(hier, "..", "export", "of13_diff_kennzahlen.csv")
    csv_pfad = os.path.abspath(csv_pfad)
    # Lauf und Zeit aus dem VTK-Pfad: .../export/<lauf>/feld_nah_000501ms.vtk
    b = os.path.basename(vtk_pfad); lauf = os.path.basename(os.path.dirname(os.path.abspath(vtk_pfad)))
    zeit = "".join(c for c in b.split("_")[-1] if c.isdigit()) or "?"
    zeile = dict(zeitstempel=datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
                 lauf=lauf, zeit_ms=zeit, ebene=ebene, ebene_m=f"{ebene_m:+.4f}",
                 n_auswertbar=int(w.size), rms=f"{np.sqrt(np.mean(w**2)):.4f}",
                 mittel=f"{np.mean(w):+.4f}", p05=f"{np.percentile(w,5):+.3f}",
                 p50=f"{np.percentile(w,50):+.3f}", p95=f"{np.percentile(w,95):+.3f}",
                 max_abs=f"{np.max(np.abs(w)):.3f}",
                 anteil_clip_pct=f"{100*np.mean(np.abs(w)>15):.3f}",
                 anteil_gt2_pct=f"{100*np.mean(np.abs(w)>2):.2f}", png=os.path.basename(png_pfad))
    neu = not os.path.exists(csv_pfad)
    os.makedirs(os.path.dirname(csv_pfad), exist_ok=True)
    with open(csv_pfad, "a", newline="") as f:
        s = csv.DictWriter(f, fieldnames=SPALTEN)
        if neu:
            f.write("# OF13-Differenzschnitte, Konvention siehe werkzeuge/of13_diff_kennzahlen.py\n")
            f.write("# dU = |u|_OF13 - |u|_FX [m/s]; positiv = OF13 schneller. FX ist LES-Momentanbild, OF13 RANS-Mittel.\n")
            s.writeheader()
        s.writerow(zeile)
    print(f"Kennzahlen angehaengt: {csv_pfad}  (ebene {ebene} {ebene_m:+.4f} m, RMS {zeile['rms']} m/s)")
    return csv_pfad

def tabelle(csv_pfad=None):
    """Druckt die gesammelten Schnitte als Tabelle -- der eigentliche Vergleich ueber Laeufe."""
    hier = os.path.dirname(os.path.abspath(__file__))
    if csv_pfad is None: csv_pfad = os.path.join(hier, "..", "export", "of13_diff_kennzahlen.csv")
    if not os.path.exists(csv_pfad): print("noch keine Kennzahlen abgelegt."); return
    R = [r for r in csv.DictReader(l for l in open(csv_pfad) if not l.startswith("#"))]
    print(f"{'Lauf':16s} {'ms':>6s} {'Ebene':9s} {'m':>9s} {'n':>9s} {'RMS':>7s} {'Mittel':>8s} {'p05':>7s} {'p95':>7s} {'|dU|>2':>7s}")
    for r in R:
        print(f"{r['lauf']:16s} {r['zeit_ms']:>6s} {r['ebene']:9s} {r['ebene_m']:>9s} {r['n_auswertbar']:>9s} "
              f"{r['rms']:>7s} {r['mittel']:>8s} {r['p05']:>7s} {r['p95']:>7s} {r['anteil_gt2_pct']:>6s}%")

if __name__ == "__main__": tabelle()
