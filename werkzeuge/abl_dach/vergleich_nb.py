#!/usr/bin/env python3
"""vergleich_nb.py -- Grenzschicht-Kennzahlen zweier prof2_<name>.npz nebeneinander (Feld-Daten, Iron Rule 3).
Aufruf: vergleich_nb.py <ref> <arm> [x1 x2 ...]   (x in m, Default 2.30 2.50 2.70 2.90 3.10)
Gibt je x: delta_90 (s90), delta_99, H, u_t erste Fluidzelle (ut1), negfrac; dazu Abloeseort aus ux1 (wie bericht2.durch)."""
import sys, os, numpy as np
SP=os.path.dirname(os.path.abspath(__file__))
def glatt(a,w):
    k=np.ones(2*w+1)/(2*w+1); return np.convolve(np.pad(a,w,mode="edge"),k,mode="valid")
def durch(x,v,xend=3.62):
    m=x<=xend; xx,vv=x[m],v[m]; ok=vv<0
    if not ok.any() or not ok[-1]: return np.nan
    i=len(ok)-1
    while i>0 and ok[i-1]: i-=1
    return xx[i]
ref,arm=sys.argv[1],sys.argv[2]
xs=[float(v) for v in sys.argv[3:]] or [2.30,2.50,2.70,2.90,3.10]
A={k:np.load(f"{SP}/prof2_{k}.npz") for k in (ref,arm)}
print(f"{'x [m]':>6s} | {'delta_90 mm':>22s} | {'delta_99 mm':>22s} | {'H':>16s} | {'ut1 m/s':>16s} | {'negfrac':>14s}")
print(f"{'':>6s} | {ref:>10s} {arm:>10s} | {ref:>10s} {arm:>10s} | {ref:>7s} {arm:>7s} | {ref:>7s} {arm:>7s} | {ref:>6s} {arm:>6s}")
for xq in xs:
    z=f"{xq:6.2f} |"
    for key,fmt,sc in (("s90","%10.1f",1e3),("d99","%10.1f",1e3),("H","%7.2f",1),("ut1","%7.2f",1),("negfrac","%6.3f",1)):
        for k in (ref,arm):
            d=A[k]; i=int(np.argmin(np.abs(d["x"]-xq))); v=d[key][i]
            z+=" "+(fmt%(v*sc) if np.isfinite(v) else fmt.replace("f","s")%"nan")
        z+=" |"
    print(z)
print()
for k in (ref,arm):
    d=A[k]; w=int(round(0.024/float(d["dx"])))
    print(f"Abloeseort (1. Fluidzelle, +-24 mm geglaettet, bis 3,62 m): {k:10s} x_s = {durch(d['x'],glatt(np.nan_to_num(d['ux1']),w)):.3f} m   (n_proben {int(d['n_proben'])}, dx {float(d['dx'])*1e3:.0f} mm)")
