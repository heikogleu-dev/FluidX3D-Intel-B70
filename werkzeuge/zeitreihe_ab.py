#!/usr/bin/env python3
"""Zeitreihentest zweier Laeufe (16.09.2026): je Sample die Differenz B-A aus cd_facetten.csv,
Vorzeichenkonstanz, Vorzeichenwechsel, Blockmittel -- Wirkung oder Rauschen? (Lehre Klemmen-Audit:
billiger und schaerfer als ein zweiter Lauf). Aufruf: zeitreihe_ab.py <lauf_A> <lauf_B> [spalte ...]"""
import sys, os, csv, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lauf_meta
def lies(name):
    p = f"export/{name}/cd_facetten.csv"; z = {}
    with open(p) as f:
        r = csv.DictReader(l for l in f if not l.startswith('#'))
        for row in r: z[round(float(row['time_s']), 6)] = {k: float(v) for k, v in row.items()}
    return z
a, b = sys.argv[1], sys.argv[2]
spalten = sys.argv[3:] or ['cd_druck_rest', 'cz_druck_rest', 'cd_reib', 'cz_reib', 'cd_druck', 'cz_druck']
# ★ 17.09.2026 (Pruefbefund 3): cd/cz_druck_rest haengen an der Kontaktband-Kante N -- seit 17.09. N = max(3, ceil(16/dx)), alte Laeufe
# 8 mm N = 2, 3,75 mm N = 4. Kante beider Laeufe aus dem Laufprotokoll vergleichen und bei Abweichung LAUT warnen (kein stiller Vergleich).
_bz, _band_gleich, _band_kurz = lauf_meta.band_vergleich([(a, os.path.join("export", a)), (b, os.path.join("export", b))])
print("\n".join(_bz))
A, B = lies(a), lies(b); ts = sorted(set(A) & set(B))
# ★ 16.09.2026: bei UNTERSCHIEDLICHEM dx fallen die Sample-Zeiten nicht aufeinander (dt = u_lat*dx/u_si).
# Dann ist der Sample-Paar-Test nicht anwendbar -- stattdessen Blockmittel ueber ein gemeinsames Zeitfenster.
# ★ 17.09.2026 (SKALIERUNG-BEFUNDE Nebenbefund 10): die Entscheidung hing an "weniger als 10 gemeinsame Zeitstempel". 4 mm (1 ms)
# gegen 8 mm (2 ms) hat aber JEDEN zweiten 4-mm-Zeitstempel gemeinsam -> Paartest auf dem halben Raster, still. Jetzt aus den
# Daten: gepaart nur bei GLEICHEM Abtastraster (Median-Abstand, 0,1 %) UND >= 90 % Deckung der Zeitstempel im gemeinsamen Fenster.
# ★ 17.09.2026 (Pruefbefund 1): die Zeitstempel sind auf 1 us gerundet (lies: round(t, 6)); bei 0,9375 ms wechseln die Abstaende
# 937/938 us, und der Median kippt je nach erster Zeile -- 1 us Unterschied lag ueber 0,1 % (0,94 us) -> faelschlich BLOCKMITTEL.
# Toleranz jetzt mindestens 1,5 us absolut (Rundung 0,5 us je Stempel, zwei Stempel je Abstand, dazu float32-Jitter).
def _raster(z):
    t = sorted(z); d = sorted(t[i+1] - t[i] for i in range(len(t) - 1))
    return d[len(d)//2] if d else float("nan")
dtA, dtB = _raster(A), _raster(B)
t0 = max(min(A), min(B)); t1 = min(max(A), max(B))
nA = sum(1 for t in A if t0 <= t <= t1); nB = sum(1 for t in B if t0 <= t <= t1)
gleiches_raster = abs(dtA - dtB) <= max(1.5e-6, 1e-3*max(dtA, dtB))
deckung = len(ts)/max(1, nA, nB)
print(f"Abtastraster {a} {dtA*1e3:.4f} ms | {b} {dtB*1e3:.4f} ms; gemeinsames Fenster {t0:.4f}..{t1:.4f} s mit {nA}/{nB} Samples, "
      f"{len(ts)} gemeinsame Zeitstempel ({100*deckung:.0f} %) -> " + ("GEPAART" if gleiches_raster and deckung >= 0.9 else "BLOCKMITTEL"))
if not (gleiches_raster and deckung >= 0.9):
    print(f"{b} vs {a}: " + ("verschiedenes Abtastraster" if not gleiches_raster else f"nur {100*deckung:.0f} % gemeinsame Zeitstempel (Phasenversatz)")
          + f" -> BLOCKMITTEL ueber {t0:.4f}..{t1:.4f} s ({nA} / {nB} Samples; Vorzeichentest NICHT anwendbar)")
    import statistics as st
    for s_ in spalten:
        va=[A[t][s_] for t in sorted(A) if t0<=t<=t1]; vb=[B[t][s_] for t in sorted(B) if t0<=t<=t1]
        ma, mb = sum(va)/len(va), sum(vb)/len(vb)
        sa = st.pstdev(va)/len(va)**0.5 if len(va)>1 else 0.0
        sb = st.pstdev(vb)/len(vb)**0.5 if len(vb)>1 else 0.0
        se = (sa*sa+sb*sb)**0.5
        h = len(va)//2; k = len(vb)//2
        d1 = sum(vb[:k])/k - sum(va[:h])/h; d2 = sum(vb[k:])/(len(vb)-k) - sum(va[h:])/(len(va)-h)
        u = "Unterschied" if abs(mb-ma) > 3*se and d1*d2 > 0 else "im Rauschen"
        print(f"  {s_:14s} A {ma:+.4f} | B {mb:+.4f} | Diff {mb-ma:+.4f} +- {se:.4f} | Bloecke {d1:+.4f} / {d2:+.4f} | {u}")
    if not _band_gleich: print(_band_kurz)
    raise SystemExit(0)
print(f"{b} - {a}: {len(ts)} gemeinsame Samples von {len(A)}/{len(B)} (t {ts[0]:.4f}..{ts[-1]:.4f} s)")
for s in spalten:
    d = [B[t][s] - A[t][s] for t in ts]; n = len(d)
    m = sum(d) / n; sd = math.sqrt(sum((x - m) ** 2 for x in d) / max(n - 1, 1)); se = sd / math.sqrt(n)
    pos = sum(1 for x in d if x > 0); wechsel = sum(1 for i in range(1, n) if (d[i] > 0) != (d[i - 1] > 0))
    neg = sum(1 for x in d if x < 0)
    h = n // 2; b1 = sum(d[:h]) / h; b2 = sum(d[h:]) / (n - h)
    ma = sum(A[t][s] for t in ts) / n
    # ★ 17.09.2026: Nulldifferenzen zaehlten als "nicht > 0" -> bitgleiche Laeufe hiessen SYSTEMATISCH. Jetzt Vorzeichen getrennt, alles 0 = IDENTISCH.
    urteil = ("IDENTISCH" if pos == 0 and neg == 0 else "SYSTEMATISCH" if (pos == n or neg == n)
              else ("wahrscheinlich" if abs(m) > 3 * se and (pos >= 0.9 * n or neg >= 0.9 * n) else "Rauschen"))
    print(f"  {s:14s} A-Mittel {ma:+.4f} | Diff {m:+.4f} +- {se:.4f} (sd {sd:.4f}) | >0: {pos}/{n} | Wechsel {wechsel} | Bloecke {b1:+.4f} / {b2:+.4f} | {urteil}")
if not _band_gleich: print(_band_kurz)
