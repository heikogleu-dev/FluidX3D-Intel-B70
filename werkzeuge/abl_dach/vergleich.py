#!/usr/bin/env python3
"""vergleich.py -- Abloeseort, Druckgradient und Grenzschicht: FX gegen OpenFOAM 13.
Liest prof2_*.npz (FX, aus fx_profil2.py) und prof_of13.npz/of13_dachlinie.csv (Referenz).
"""
import numpy as np, os, warnings
warnings.filterwarnings("ignore")
SP=os.path.dirname(os.path.abspath(__file__)); Q=0.5*1.225*30.0**2

def glatt(a,w):
    k=np.ones(2*w+1)/(2*w+1); return np.convolve(np.pad(a,w,mode="edge"),k,mode="valid")

def durchgehend(x, v, xend=3.66):
    """kleinstes x, ab dem v bis xend durchgehend < 0 ist"""
    m=x<=xend
    xx,vv=x[m],v[m]
    ok=vv<0
    if not ok.any() or not ok[-1]: return np.nan
    i=len(ok)-1
    while i>0 and ok[i-1]: i-=1
    return xx[i]

def erste_null(x, v, x0=2.6, x1=3.9):
    """erster Vorzeichenwechsel von v (negativ->positiv) im Fenster, linear interpoliert"""
    m=(x>=x0)&(x<=x1); xx,vv=x[m],v[m]
    for i in range(len(vv)-1):
        if vv[i]<0<=vv[i+1]:
            return xx[i]+(xx[i+1]-xx[i])*(-vv[i])/(vv[i+1]-vv[i])
    return np.nan

# ---------- OF13
O=np.load(SP+"/prof_of13.npz")
xo, cfo, cpo, UTo, so = O["x"], O["cf"], O["cp"], O["ut"], O["s"]
print("="*100); print("1) ABLOESEORT")
print(f"OF13  cf-Nulldurchgang (wallShearStress, tangential): x_v2 = {erste_null(xo,cfo):.3f} m")
for smm in (2.0,4.0,6.0):
    k=int(np.argmin(np.abs(so-smm/1000)))
    ut=UTo[:,k]
    print(f"OF13  u_t(s={smm:.0f} mm) Vorzeichenwechsel:                x_v2 = {erste_null(xo,np.nan_to_num(ut,nan=1)):.3f} m"
          f"   (durchgehend<0 ab {durchgehend(xo,np.nan_to_num(ut,nan=1)):.3f})")
print()
NAM=[("p4_ref","FX 4 mm SATGATE=1  t=300 ms          "),
     ("p4_satgate0","FX 4 mm SATGATE=0  t=300/450/501 ms  "),
     ("p4_v3b","FX 4 mm V3b        t=500 ms          "),
     ("w_ref","FX 8 mm SATGATE=1  t=300 ms          "),
     ("w_ref_3t","FX 8 mm SATGATE=1  t=300/450/500 ms  "),
     ("w_satgate0","FX 8 mm SATGATE=0  t=300/450/500 ms  "),
     ("w_bb_3t","FX 8 mm reines BB  t=300/450/500 ms  ")]
F={}
for k,lab in NAM:
    d=np.load(SP+f"/prof2_{k}.npz"); F[k]=d
    x=d["x"]; w=int(round(0.024/float(d["dx"])))
    uxg=glatt(np.nan_to_num(d["ux1"]),w); utg=glatt(np.nan_to_num(d["ut1"]),w)
    nf =glatt(np.nan_to_num(d["negfrac"]),w)
    print(f"{lab} u_x(1.Zelle) durchgehend<0 ab x_v2 = {durchgehend(x,uxg):6.3f} | "
          f"u_t {durchgehend(x,utg):6.3f} | Nulldurchgang u_t {erste_null(x,utg):6.3f} | "
          f"Anteil u_x<0 >50 % ab {durchgehend(x,0.5-nf):6.3f}")

print(); print("="*100); print("2) DRUCKVERLAUF  (cp und dp/dx; p = cp*q, q = 551.25 Pa)")
SEG=[("Dachplateau       ",2.30,2.60),("Heckscheibe oben  ",2.65,3.00),
     ("Heckscheibe Mitte ",3.00,3.35),("Heckscheibe unten ",3.35,3.62),
     ("Heckscheibe gesamt",2.65,3.62)]
def dpdx(x,cp,a,b):
    m=(x>=a)&(x<=b)
    if m.sum()<3: return np.nan,np.nan,np.nan
    A=np.polyfit(x[m],cp[m]*Q,1)
    return A[0], (cp[m]*Q)[0], (cp[m]*Q)[-1]
kopf=f"{'Abschnitt':20s}"+"".join(f"{l.split()[1]+l.split()[2]:>14s}" for _,l in [(0,'x OF13 -')] ) if False else None
print(f"{'Abschnitt':20s} {'x-Bereich':16s} {'OF13':>12s} "+ " ".join(f"{k:>12s}" for k,_ in NAM))
for lab,a,b in SEG:
    zeile=f"{lab:20s} {a:.2f}..{b:.2f} m   "
    g,_,_=dpdx(xo,cpo,a,b); zeile+=f"{g:12.0f} "
    for k,_ in NAM:
        d=F[k]; x=d["x"]; w=int(round(0.024/float(d["dx"])))
        cpg=glatt(np.nan_to_num(d["cp"]),w)
        g,_,_=dpdx(x,cpg,a,b); zeile+=f"{g:12.0f} "
    print(zeile+" Pa/m")
print()
print(f"{'cp-Werte':20s} {'x':16s} {'OF13':>12s} "+" ".join(f"{k:>12s}" for k,_ in NAM))
for xs in (2.45,2.70,3.00,3.20,3.40,3.60):
    zeile=f"{'cp @ x_v2':20s} {xs:.2f} m          "
    zeile+=f"{np.interp(xs,xo,cpo):12.3f} "
    for k,_ in NAM:
        d=F[k]; x=d["x"]; w=int(round(0.024/float(d["dx"])))
        zeile+=f"{np.interp(xs,x,glatt(np.nan_to_num(d['cp']),w)):12.3f} "
    print(zeile)

print(); print("="*100); print("3) GRENZSCHICHT (wandnormal, u_e = Maximum laengs des Strahls)")
print(f"{'x_v2 [m]':10s} {'Groesse':10s} {'OF13':>10s} "+" ".join(f"{k:>12s}" for k,_ in NAM))
for xs in (2.45,2.90,3.20,3.40,3.55):
    for gr,arr in (("d99 [mm]","d99"),("d* [mm]","dstern"),("theta[mm]","theta"),("H [-]","H")):
        zeile=f"{xs:<10.2f} {gr:10s} "
        v=np.interp(xs,xo,O[arr]); zeile+=f"{(1000*v if 'mm' in gr else v):10.2f} "
        for k,_ in NAM:
            d=F[k]; v=np.interp(xs,d["x"],d[arr]); zeile+=f"{(1000*v if 'mm' in gr else v):12.2f} "
        print(zeile)
    # Zellen in delta99
    zeile=f"{xs:<10.2f} {'Zellen/d99':10s} {'--':>10s} "
    for k,_ in NAM:
        d=F[k]; v=np.interp(xs,d["x"],d["d99"]); zeile+=f"{v/float(d['dx']):12.1f} "
    print(zeile); print()
