#!/usr/bin/env python3
"""OF13-Referenz, Oberhaut der Mittelebene: Kontur, Wandschubspannung, Wanddruck.

Kette (alles aus ~/CFD-Cases/mr2v40H, Zeitschritt 1200):
  1200/C.gz  boundaryField[vehicle]        -> of13_vehicle_C.npy   (2 648 253 Flaechenmitten)
  1200/wallShearStress.gz  bF[vehicle]     -> of13_vehicle_TW.npy  (tau_w/rho, m^2/s^2)
  |y|<20 mm, je 10-mm-x-Bin die Facetten mit z > z_max-30 mm  -> "Oberhaut" (12 428 Facetten)
  constant/polyMesh/owner.gz Zeile 21+FaceID -> Eigentuemerzelle je Oberhaut-Facette
  1200/p.gz, 1200/U.gz Zeile 22+ZellID       -> p (m^2/s^2) und U (m/s) der ERSTEN Fluidzelle
p ist kinematisch; p_Pa = rho*p mit rho = 1.225. cp = p/(0.5*U_inf^2), U_inf = 30 m/s.
Vorzeichen wallShearStress (OpenFOAM): bei anliegender Stroemung in +x ist tau_x NEGATIV;
Abloesung = Vorzeichenwechsel nach POSITIV.
"""
import numpy as np, os
SP=os.path.dirname(os.path.abspath(__file__)); XOFF, UINF, RHO = 2.2063, 30.0, 1.225

C  = np.load(SP+"/of13_vehicle_C.npy").astype(np.float64)
TW = np.load(SP+"/of13_vehicle_TW.npy").astype(np.float64)
sel= np.load(SP+"/of13_oberhaut_idx.npy")
own= np.loadtxt(SP+"/of13_oberhaut_owner.txt", dtype=np.int64, usecols=(1,), converters={0:lambda s:0}) \
     if False else None
# owner-Datei: "<zeile>: <zelle>"
raw = np.array([l.replace(":"," ").split() for l in open(SP+"/of13_oberhaut_owner.txt")], dtype=np.int64)
zeile2zelle = dict(zip(raw[:,0], raw[:,1]))
faceline = 102962857+sel+21
zelle = np.array([zeile2zelle[l] for l in faceline])

def lies(pfad, ncol):
    d={}
    for l in open(pfad):
        t=l.split()
        d[int(t[0])-22] = np.array(t[1:1+ncol], dtype=np.float64)
    return d
P = lies(SP+"/of13_oberhaut_p.txt",1); CC = lies(SP+"/of13_oberhaut_C.txt",3); UU = lies(SP+"/of13_oberhaut_U.txt",3)
pw   = np.array([P[c][0] for c in zelle])
ccw  = np.array([CC[c]   for c in zelle])
uuw  = np.array([UU[c]   for c in zelle])

fc, tw = C[sel], TW[sel]
DXB=0.010
b=np.floor((fc[:,0]+2.30)/DXB).astype(int)
rows=[]
for u in np.unique(b):
    s=b==u
    if s.sum()<3: continue
    rows.append([fc[s,0].mean(), fc[s,2].mean(), s.sum(),
                 tw[s,0].mean(), tw[s,2].mean(),
                 pw[s].mean(), np.linalg.norm(ccw[s]-fc[s],axis=1).mean(),
                 np.linalg.norm(uuw[s],axis=1).mean()])
R=np.array(rows)
x,z,n,twx,twz,pcell,dw,umag = R.T
dx=np.gradient(x); dz=np.gradient(z); L=np.hypot(dx,dz); tx,tz=dx/L,dz/L
tau_t=twx*tx+twz*tz
cf=tau_t/(0.5*UINF**2)
cp=pcell/(0.5*UINF**2)
ang=np.degrees(np.arctan2(-dz,dx))
np.savetxt(SP+"/of13_dachlinie.csv", np.c_[x, x+XOFF, z, ang, n, tau_t, cf, cp, pcell*RHO, dw, umag],
   delimiter=",", fmt="%.6g",
   header="x_of13_m,x_v2_m,z_m,winkel_grad,n_facetten,tau_t_m2s2,cf,cp_wand,p_wand_Pa,wandabstand_m,u_erstezelle_ms", comments="")
print("of13_dachlinie.csv:", len(x), "Bins;  Wandabstand der ersten Zelle: median",
      f"{1000*np.median(dw):.2f} mm, min {1000*dw.min():.2f}, max {1000*dw.max():.2f}")
sel2=(x+XOFF>2.0)&(x+XOFF<3.95)
for i in np.where(sel2)[0][::3]:
    print(f"x_v2 {x[i]+XOFF:6.3f} z {z[i]:6.3f} ang {ang[i]:6.2f} n {int(n[i]):3d} "
          f"tau_t {tau_t[i]:+8.4f} cf {cf[i]:+9.6f} cp {cp[i]:+7.3f} p {pcell[i]*RHO:+8.1f} Pa dw {1000*dw[i]:5.2f}mm")
