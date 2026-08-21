#!/usr/bin/env python3
# Auswertung von schnitt_diff_letzter.csv (Iron Rule 5: gemessen wird hier, nicht am PNG).
import sys, csv, math
p = sys.argv[1] if len(sys.argv)>1 else "export/f8_standard_final_diff/schnitt_diff_letzter.csv"
d=[]; xs={}
n_tot=n_solid=n_ung=0; t_ms=None
with open(p) as f:
    for r in csv.DictReader(f):
        n_tot+=1; t_ms=r["t_ms"]
        if r["nah_solid"]=="1": n_solid+=1; continue
        if r["fern_ungueltig"]=="1": n_ung+=1; continue
        v=float(r["d_betrag_ms"]); x=float(r["x_m"]); d.append(v)
        b=round(x,1); s=xs.setdefault(b,[0,0.0,0.0]); s[0]+=1; s[1]+=v*v; s[2]=max(s[2],abs(v))
d.sort(); n=len(d)
q=lambda f: d[min(n-1,int(f*n))]
print(f"Datei {p}  t = {t_ms} ms")
print(f"Zellen gesamt {n_tot}, Nah-Solid {n_solid}, ohne Fernwert {n_ung}, auswertbar {n}")
print(f"RMS {math.sqrt(sum(v*v for v in d)/n):.4f} m/s | Mittel {sum(d)/n:+.4f} | min {d[0]:+.3f} | max {d[-1]:+.3f}")
print(f"Quantile: p01 {q(.01):+.2f}  p05 {q(.05):+.2f}  p50 {q(.50):+.2f}  p95 {q(.95):+.2f}  p99 {q(.99):+.2f}")
print(f"|d| > 15 m/s (Skalen-Clip): {sum(1 for v in d if abs(v)>15)/n*100:.2f} %   |d| > 5: {sum(1 for v in d if abs(v)>5)/n*100:.1f} %")
print("\nx-Baender (0,1 m) -- RMS / max|d|:")
for b in sorted(xs):
    c,s2,mx=xs[b]
    print(f"  x={b:+5.1f} m  n={c:5d}  RMS {math.sqrt(s2/c):6.3f}  max {mx:6.2f}")
