#!/usr/bin/env python3
"""Zeitreihentest zweier Laeufe (16.09.2026): je Sample die Differenz B-A aus cd_facetten.csv,
Vorzeichenkonstanz, Vorzeichenwechsel, Blockmittel -- Wirkung oder Rauschen? (Lehre Klemmen-Audit:
billiger und schaerfer als ein zweiter Lauf). Aufruf: zeitreihe_ab.py <lauf_A> <lauf_B> [spalte ...]"""
import sys, csv, math
def lies(name):
    p = f"export/{name}/cd_facetten.csv"; z = {}
    with open(p) as f:
        r = csv.DictReader(l for l in f if not l.startswith('#'))
        for row in r: z[round(float(row['time_s']), 6)] = {k: float(v) for k, v in row.items()}
    return z
a, b = sys.argv[1], sys.argv[2]
spalten = sys.argv[3:] or ['cd_druck_rest', 'cz_druck_rest', 'cd_reib', 'cz_reib', 'cd_druck', 'cz_druck']
A, B = lies(a), lies(b); ts = sorted(set(A) & set(B))
print(f"{b} - {a}: {len(ts)} gemeinsame Samples von {len(A)}/{len(B)} (t {ts[0]:.4f}..{ts[-1]:.4f} s)")
for s in spalten:
    d = [B[t][s] - A[t][s] for t in ts]; n = len(d)
    m = sum(d) / n; sd = math.sqrt(sum((x - m) ** 2 for x in d) / max(n - 1, 1)); se = sd / math.sqrt(n)
    pos = sum(1 for x in d if x > 0); wechsel = sum(1 for i in range(1, n) if (d[i] > 0) != (d[i - 1] > 0))
    h = n // 2; b1 = sum(d[:h]) / h; b2 = sum(d[h:]) / (n - h)
    ma = sum(A[t][s] for t in ts) / n
    urteil = "SYSTEMATISCH" if (pos == n or pos == 0) else ("wahrscheinlich" if abs(m) > 3 * se and (pos >= 0.9 * n or pos <= 0.1 * n) else "Rauschen")
    print(f"  {s:14s} A-Mittel {ma:+.4f} | Diff {m:+.4f} +- {se:.4f} (sd {sd:.4f}) | >0: {pos}/{n} | Wechsel {wechsel} | Bloecke {b1:+.4f} / {b2:+.4f} | {urteil}")
