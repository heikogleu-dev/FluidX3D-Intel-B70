#!/usr/bin/env python3
"""Kraftbericht je 50-ms-Fenster (Heiko 16.09.2026): fuer jedes Fenster das MITTEL ueber genau
dieses Fenster, nicht kumulativ. Mehrere Laeufe nebeneinander -- auch bei verschiedenem dx, weil
ueber Zeitfenster gemittelt wird und nicht ueber Sample-Paare. Aufruf: fenster_50ms.py <lauf> [<lauf>...] [--spalte X]"""
import sys, os, csv, io, statistics as st
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lauf_meta
args = [a for a in sys.argv[1:] if not a.startswith('--')]
spalten = [a.split('=',1)[1] for a in sys.argv[1:] if a.startswith('--spalte=')] or ['cd_druck_rest','cz_druck_rest','cd_reib','cz_reib']
# ★ 16.09.2026: cd_druck_rest/cz_druck_rest SIND cd_rest/cz_rest des eingebauten [BERICHT]
# (setup.cpp:9103/9106, cd_bericht.csv: (FK.px-FK.pbx)/qA) -- gegengeprueft an sechs Fenstern,
# Abweichung <= 0,0014. Die Reibung ist darin NICHT enthalten und darf NICHT hinzuaddiert werden:
# cd_reib ist ein Fenstermittel seit Warmup und streut kaum, eine Summe taeuscht Signifikanz vor.
def lies(n):
    d = {}
    with open(f"export/{n}/cd_facetten.csv") as f:
        for row in csv.DictReader(l for l in f if not l.startswith('#')):
            d[float(row['time_s'])] = {k: float(v) for k, v in row.items()}
    return d
L = [(n, lies(n)) for n in args]
# ★ 17.09.2026 (Pruefbefund 3): cd/cz_druck_rest haengen an der Bandkante N (seit 17.09. N = max(3, ceil(16/dx)), alte Laeufe 8 mm 2,
# 3,75 mm 4) -- verschiedene Sprossen haben verschiedene Kanten; das wird LAUT gemeldet, nicht still nebeneinandergestellt.
if any(s_.endswith("_rest") for s_ in spalten):
    BZ, BAND_GLEICH, BAND_KURZ = lauf_meta.band_vergleich([(n, os.path.join("export", n)) for n in args])
    print("\n".join(BZ))
else:
    BAND_GLEICH, BAND_KURZ = True, ""
tmax = max(max(d) for _, d in L)
grenzen = [(t/1000.0, (t+50)/1000.0) for t in range(200, int(tmax*1000)+1, 50)]
for s in spalten:
    print(f"\n=== {s} — Mittel je 50-ms-Fenster (n = Samples im Fenster) ===")
    print("  Fenster        " + "".join(f"{n:>26s}" for n, _ in L))
    for t0, t1 in grenzen:
        zeile = f"  {t0*1000:5.0f}-{t1*1000:5.0f} ms "
        for n, d in L:
            v = [d[t][s] for t in d if t0 <= t < t1]
            zeile += f"{(f'{sum(v)/len(v):+.4f} +- {st.pstdev(v)/len(v)**0.5:.4f} (n={len(v)})' if len(v)>1 else (f'{v[0]:+.4f} (n=1)' if v else '--')):>26s}"
        print(zeile)
    if len(L) == 2:
        print("  " + "-"*14 + " Differenz " + L[1][0] + " - " + L[0][0])
        for t0, t1 in grenzen:
            va = [L[0][1][t][s] for t in L[0][1] if t0 <= t < t1]; vb = [L[1][1][t][s] for t in L[1][1] if t0 <= t < t1]
            if len(va) > 1 and len(vb) > 1:
                se = (st.pstdev(va)**2/len(va) + st.pstdev(vb)**2/len(vb))**0.5
                d = sum(vb)/len(vb) - sum(va)/len(va)
                print(f"  {t0*1000:5.0f}-{t1*1000:5.0f} ms  {d:+.4f} +- {se:.4f}  {'*' if abs(d)>3*se else ''}")
if not BAND_GLEICH: print("\n" + BAND_KURZ)
