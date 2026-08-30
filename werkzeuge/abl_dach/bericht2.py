#!/usr/bin/env python3
"""bericht2.py -- Abschlusstabellen: Abloeseort bei festem Wandabstand, Wanddruck, dp/dx."""
import numpy as np, os, warnings; warnings.filterwarnings("ignore")
SP=os.path.dirname(os.path.abspath(__file__)); Q=0.5*1.225*30.0**2; XE=3.62
def glatt(a,w):
    k=np.ones(2*w+1)/(2*w+1); return np.convolve(np.pad(a,w,mode="edge"),k,mode="valid")
def durch(x,v,xend=XE):
    m=x<=xend; xx,vv=x[m],v[m]; ok=vv<0
    if not ok.any() or not ok[-1]: return np.nan
    i=len(ok)-1
    while i>0 and ok[i-1]: i-=1
    return xx[i]
O=np.load(SP+"/prof_of13.npz"); xo,so=O["x"],O["s"]
NAM=[("p4_ref","FX 4mm SG1 t=300"),("p4_satgate0","FX 4mm SG0 3 Zeiten"),("p4_v3b","FX 4mm V3b t=500"),
     ("w_ref","FX 8mm SG1 t=300"),("w_ref_3t","FX 8mm SG1 3 Zeiten"),
     ("w_satgate0","FX 8mm SG0 3 Zeiten"),("w_bb_3t","FX 8mm BB 3 Zeiten")]
F={k:np.load(SP+f"/prof2_{k}.npz") for k,_ in NAM}
print("A) x_s [m] = Beginn der DURCHGEHENDEN Rueckstroemung bis zum Ende der Heckscheibe (x_v2 = 3.62 m)")
print(f"{'Wandabstand':14s} {'OF13':>8s} "+" ".join(f"{k:>12s}" for k,_ in NAM))
zeile=f"{'tau_w (cf=0)':14s} "
# OF13 cf-Nullstelle
cf=O["cf"]
xs=np.nan
for i in range(len(xo)-1):
    if xo[i]>2.6 and cf[i]<0<=cf[i+1]: xs=xo[i]+(xo[i+1]-xo[i])*(-cf[i])/(cf[i+1]-cf[i]); break
print(f"{'tau_w (cf=0)':14s} {xs:8.3f} "+" ".join(f"{'--':>12s}" for _ in NAM))
print(f"{'1. Fluidzelle':14s} {'--':>8s} "+" ".join(
    f"{durch(F[k]['x'],glatt(np.nan_to_num(F[k]['ux1']),int(round(0.024/float(F[k]['dx']))))):12.3f}" for k,_ in NAM))
for smm in (8,16,20,24,32,48):
    z=f"{'u_t @ '+str(smm)+' mm':14s} "
    ko=int(np.argmin(np.abs(so-smm/1000))); z+=f"{durch(xo,O['ut'][:,ko]):8.3f} "
    for k,_ in NAM:
        d=F[k]; w=int(round(0.024/float(d["dx"]))); kk=int(np.argmin(np.abs(d["s"]-smm/1000)))
        z+=f"{durch(d['x'],glatt(np.nan_to_num(d['utg'][:,kk]),w)):12.3f} "
    print(z)

print(); print("B) WANDDRUCK cp (OF13: Eigentuemerzelle 0.31 mm; FX: erste Fluidzelle) -- ueber +-24 mm in x geglaettet")
D=np.genfromtxt(SP+"/of13_dachlinie.csv",delimiter=",",names=True)
xw=D["x_v2_m"]; cpw=D["cp_wand"]
print(f"{'x_v2 [m]':10s} {'OF13':>8s} "+" ".join(f"{k:>12s}" for k,_ in NAM))
for xs in np.arange(2.2,3.71,0.1):
    z=f"{xs:<10.2f} {np.interp(xs,xw,cpw):8.3f} "
    for k,_ in NAM:
        d=F[k]; w=int(round(0.024/float(d["dx"])))
        z+=f"{np.interp(xs,d['x'],glatt(np.nan_to_num(d['cp']),w)):12.3f} "
    print(z)
print()
print("C) dp/dx [Pa/m] aus dem Wanddruck (lineare Ausgleichsgerade im Abschnitt)")
SEG=[("Dachplateau",2.30,2.60),("Heckschb. oben",2.65,3.00),("Heckschb. Mitte",3.00,3.35),
     ("Heckschb. unten",3.35,3.62),("Heckschb. gesamt",2.65,3.62),("Dach+Heckschb.",2.30,3.62)]
def fit(x,cp,a,b):
    m=(x>=a)&(x<=b); return np.polyfit(x[m],cp[m]*Q,1)[0] if m.sum()>3 else np.nan
print(f"{'Abschnitt':18s} {'Bereich':14s} {'OF13':>8s} "+" ".join(f"{k:>12s}" for k,_ in NAM))
for lab,a,b in SEG:
    z=f"{lab:18s} {a:.2f}-{b:.2f} m  {fit(xw,cpw,a,b):8.0f} "
    for k,_ in NAM:
        d=F[k]; w=int(round(0.024/float(d["dx"])))
        z+=f"{fit(d['x'],glatt(np.nan_to_num(d['cp']),w),a,b):12.0f} "
    print(z)
print()
print("D) Streuung der FX-Momentanwerte: cp-Standardabweichung ueber die y-Ebenen/Zeiten, x in 2.3..3.6")
for k,lab in NAM:
    d=F[k]; m=(d["x"]>2.3)&(d["x"]<3.6)
    roh=np.nan_to_num(d["cp"])[m]; w=int(round(0.024/float(d["dx"])))
    print(f"  {lab:22s} cp roh sd(x-zu-x) = {np.std(np.diff(roh)):.3f}, "
          f"nach Glaettung {np.std(np.diff(glatt(np.nan_to_num(d['cp']),w)[m])):.3f}, n_Proben={int(d['n_proben'])}")
