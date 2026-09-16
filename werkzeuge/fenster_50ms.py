#!/usr/bin/env python3
"""Kraftbericht je 50-ms-Fenster (Heiko 16.09.2026): fuer jedes Fenster das MITTEL ueber genau
dieses Fenster, nicht kumulativ. Mehrere Laeufe nebeneinander -- auch bei verschiedenem dx, weil
ueber Zeitfenster gemittelt wird und nicht ueber Sample-Paare. Aufruf: fenster_50ms.py <lauf> [<lauf>...] [--spalte X]"""
import sys, csv, io, statistics as st
args = [a for a in sys.argv[1:] if not a.startswith('--')]
spalten = [a.split('=',1)[1] for a in sys.argv[1:] if a.startswith('--spalte=')] or ['cd_druck_rest','cz_druck_rest','cd_reib','cz_reib']
def lies(n):
    d = {}
    with open(f"export/{n}/cd_facetten.csv") as f:
        for row in csv.DictReader(l for l in f if not l.startswith('#')):
            d[float(row['time_s'])] = {k: float(v) for k, v in row.items()}
    return d
L = [(n, lies(n)) for n in args]
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
