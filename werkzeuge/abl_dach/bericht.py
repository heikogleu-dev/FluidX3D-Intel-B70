#!/usr/bin/env python3
"""bericht.py -- Endauswertung: Abloeseort, Druckgradient, Grenzschicht. FX gegen OF13."""
import numpy as np, os, warnings; warnings.filterwarnings("ignore")
SP=os.path.dirname(os.path.abspath(__file__)); Q=0.5*1.225*30.0**2
def glatt(a,w):
    k=np.ones(2*w+1)/(2*w+1); return np.convolve(np.pad(a,w,mode="edge"),k,mode="valid")
def durch(x,v,xend=3.62):
    m=x<=xend; xx,vv=x[m],v[m]; ok=vv<0
    if not ok.any() or not ok[-1]: return np.nan
    i=len(ok)-1
    while i>0 and ok[i-1]: i-=1
    return xx[i]
def null(x,v,x0=2.6,x1=3.95):
    m=(x>=x0)&(x<=x1);xx,vv=x[m],v[m]
    for i in range(len(vv)-1):
        if vv[i]<0<=vv[i+1]: return xx[i]+(xx[i+1]-xx[i])*(-vv[i])/(vv[i+1]-vv[i])
    return np.nan
O=np.load(SP+"/prof_of13.npz")
xo,so,UTo,CPSo=O["x"],O["s"],O["ut"],O["cps"]
NAM=[("p4_ref","FX 4mm SG1 t=300ms"),("p4_satgate0","FX 4mm SG0 3 Zeiten"),
     ("p4_v3b","FX 4mm V3b t=500ms"),("w_ref","FX 8mm SG1 t=300ms"),
     ("w_ref_3t","FX 8mm SG1 3 Zeiten"),("w_satgate0","FX 8mm SG0 3 Zeiten"),
     ("w_bb_3t","FX 8mm BB  3 Zeiten")]
F={k:np.load(SP+f"/prof2_{k}.npz") for k,_ in NAM}

def cp_bei(d,smm):
    """cp auf dem Strahl in smm mm Wandabstand (naechster Stuetzpunkt)"""
    k=int(np.argmin(np.abs(d["s"]-smm/1000.0)))
    return d["cpsg"][:,k] if "cpsg" in d else d["cps"][:,k]

print("#"*104)
print("A) ABLOESEORT  (x_v2; Heckscheibe endet bei x_v2 = 3.66 m, Dachscheitel 2.40-2.50 m)")
print("#"*104)
print(f"{'Rechnung':22s} {'Kriterium':44s} {'x_s [m]':>9s}")
print(f"{'OF13':22s} {'tau_w tangential wechselt Vorzeichen (cf=0)':44s} {null(xo,O['cf']):9.3f}")
for smm in (2,4,8,20):
    k=int(np.argmin(np.abs(so-smm/1000)))
    ut=np.nan_to_num(UTo[:,k],nan=1.0)
    print(f"{'OF13':22s} {'u_t in '+str(smm)+' mm Wandabstand < 0 (durchgehend)':44s} {durch(xo,ut):9.3f}")
for k,lab in NAM:
    d=F[k]; x=d["x"]; w=int(round(0.024/float(d["dx"])))
    ux=glatt(np.nan_to_num(d["ux1"]),w); ut=glatt(np.nan_to_num(d["ut1"]),w)
    nf=glatt(np.nan_to_num(d["negfrac"]),w)
    print(f"{lab:22s} {'u_x erste Fluidzelle < 0 (B58-Kriterium)':44s} {durch(x,ux):9.3f}")
    print(f"{lab:22s} {'u_t erste Fluidzelle < 0':44s} {durch(x,ut):9.3f}")
    print(f"{lab:22s} {'Rueckstroemanteil > 50 % der Proben':44s} {durch(x,0.5-nf):9.3f}")
    for smm in (20,):
        kk=int(np.argmin(np.abs(d["s"]-smm/1000)))
        print(f"{lab:22s} {'u_t in '+str(smm)+' mm Wandabstand < 0':44s} {durch(x,glatt(np.nan_to_num(d['utg'][:,kk]),w)):9.3f}")
    print()

print("#"*104); print("B) DRUCK auf 24 mm Wandabstand (dp/dn~0 in der Grenzschicht; unbeeinflusst von der Voxeltreppe)")
print("#"*104)
SEG=[("Dachplateau",2.30,2.60),("Heckscheibe oben",2.65,3.00),("Heckscheibe Mitte",3.00,3.35),
     ("Heckscheibe unten",3.35,3.62),("Heckscheibe gesamt",2.65,3.62)]
def fit(x,cp,a,b):
    m=(x>=a)&(x<=b)
    return np.polyfit(x[m],cp[m]*Q,1)[0] if m.sum()>3 else np.nan
kopf=f"{'dp/dx [Pa/m]':20s} {'Bereich':14s} {'OF13':>8s} "+" ".join(f"{k:>12s}" for k,_ in NAM)
print(kopf)
cpo24=np.nan_to_num(CPSo[:,int(np.argmin(np.abs(so-0.024)))],nan=0.0)
for lab,a,b in SEG:
    z=f"{lab:20s} {a:.2f}-{b:.2f} m  {fit(xo,cpo24,a,b):8.0f} "
    for k,_ in NAM:
        d=F[k]; z+=f"{fit(d['x'],cp_bei(d,24),a,b):12.0f} "
    print(z)
print()
print(f"{'cp(24 mm) @ x_v2':20s} {'':14s} {'OF13':>8s} "+" ".join(f"{k:>12s}" for k,_ in NAM))
for xs in (2.40,2.60,2.80,3.00,3.20,3.40,3.60):
    z=f"{'':20s} {xs:.2f} m        {np.interp(xs,xo,cpo24):8.3f} "
    for k,_ in NAM:
        d=F[k]; z+=f"{np.interp(xs,d['x'],cp_bei(d,24)):12.3f} "
    print(z)

print(); print("#"*104); print("C) GRENZSCHICHT")
print("#"*104)
print(f"{'x_v2':6s} {'Groesse':12s} {'OF13':>9s} "+" ".join(f"{k:>12s}" for k,_ in NAM))
for xs in (2.45,2.90,3.20,3.45):
    for gr in ("s50","s90","d99","dstern","theta","H"):
        z=f"{xs:<6.2f} {gr:12s} "
        v=np.interp(xs,xo,O[gr]) if gr in O else np.nan
        z+=f"{(1000*v if gr!='H' else v):9.2f} "
        for k,_ in NAM:
            d=F[k]; v=np.interp(xs,d["x"],d[gr]); z+=f"{(1000*v if gr!='H' else v):12.2f} "
        print(z)
    for smm in (5,10,20,50):
        z=f"{xs:<6.2f} {'u_t@'+str(smm)+'mm':12s} "
        k5=int(np.argmin(np.abs(so-smm/1000))); z+=f"{np.interp(xs,xo,np.nan_to_num(UTo[:,k5])):9.2f} "
        for k,_ in NAM:
            d=F[k]; kk=int(np.argmin(np.abs(d["s"]-smm/1000)))
            z+=f"{np.interp(xs,d['x'],d['utg'][:,kk]):12.2f} "
        print(z)
    print(f"{xs:<6.2f} {'Zellen in':12s} {'d99(OF13)':>9s} "+" ".join(
        f"{np.interp(xs,xo,O['d99'])/float(F[k]['dx']):12.1f}" for k,_ in NAM))
    print()
