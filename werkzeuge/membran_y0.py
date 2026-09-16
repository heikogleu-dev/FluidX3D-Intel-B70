#!/usr/bin/env python3
"""Mittelebenen-Membran messen (16.09.2026): Anteil der Wandzellen mit Materialdicke 1 je y-Zeile um y = 0,
aus export/<lauf>/facetten_histogramme.csv (Spalte n = linearer Index, solid_dicke) und dem Lauflog (Gitter, y-Ursprung).
Ohne Versatz: dy = +-1 mit ~55-82 % (Blech; 8 mm 73 %, 3,75 mm 82 %; 4 mm/p4_register zufaellig frei, 1 ulp); Massstab sind die
NACHBARZEILEN (0,3-1,1 %), nicht das Gesamtmittel. Exit 0 = weg (max < 10 % in jc-2..jc+2). Aus dem Repo-Wurzelverzeichnis aufrufen."""
import sys, re, csv, collections
lauf = sys.argv[1]
log = open(f"logs/{lauf}.log", errors="ignore").read()
log = re.sub(r'\x1b\[[0-9;]*m', '', log)
m = re.search(r'Grid Resolution \|\s*(\d+) x (\d+) x (\d+)', log); Nx, Ny = int(m.group(1)), int(m.group(2))
y0 = float(re.search(r'Nahfeld x\[[^\]]*\] y\[(-?[0-9.]+);', log).group(1))
try: dx = float(re.search(r'CFD_DX=([0-9.]+)', open(f"export/{lauf}/code/LAUF.txt", errors="ignore").read()).group(1)) * 1e-3
except Exception: dx = float(re.search(r'CFD_DX=([0-9.]+)', log).group(1)) * 1e-3
j0 = (0.0 - y0) / dx
rows = collections.defaultdict(lambda: [0, 0]); tot = [0, 0]
with open(f"export/{lauf}/facetten_histogramme.csv") as f:
    for r in csv.reader(l for l in f if not l.startswith('#')):
        n = int(r[8]); d = int(r[9]); j = (n // Nx) % Ny
        rows[j][0] += 1; tot[0] += 1
        if d == 1: rows[j][1] += 1; tot[1] += 1
jc = int(round(j0)); mx = 0.0
print(f"{lauf}: y = 0 bei feinem Index {j0:.3f}; Wandzellen {tot[0]}, Dicke 1 gesamt {100*tot[1]/tot[0]:.1f} %")
print("  Zeile  Wandzellen  Dicke-1  Anteil")
for j in range(jc - 3, jc + 4):
    a, b = rows[j]; sh = 100 * b / a if a else 0.0
    if abs(j - jc) <= 2: mx = max(mx, sh)  # mit Versatz liegen die Nachbarzeilen bei jc und jc+1 (Pruefagent N5)
    print(f"  {j:5d}  {a:9d}  {b:7d}  {sh:5.1f} %")
print("Membran:", "WEG" if mx < 10.0 else "VORHANDEN", f"(max {mx:.1f} % in den Zeilen um y = 0)")
sys.exit(0 if mx < 10.0 else 1)
