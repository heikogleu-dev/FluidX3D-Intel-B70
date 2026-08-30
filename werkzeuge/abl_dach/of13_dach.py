#!/usr/bin/env python3
"""OF13-Dachlinie: Kontur, tau_w (tangential) und Wanddruck (erste Fluidzelle) auf der
Mittelebene, von der Frontscheibe bis hinter die Heckscheibe.

Quellen (Zeitschritt 1200 des Falls ~/CFD-Cases/mr2v40H):
  of13_vehicle_C.npy   Flaechenmitten Patch "vehicle"  (constant/polyMesh/boundary -> 2648253 Flaechen, aus 1200/C.gz)
  of13_vehicle_TW.npy  wallShearStress je Flaeche      (aus 1200/wallShearStress.gz, Einheit m^2/s^2 = tau/rho)
  of13_slab_y003.txt   Zellmitten + p(m^2/s^2) + U(m/s) im Band |y|<0.03 m (aus 1200/C.gz,p.gz,U.gz)
Ausgabe: of13_dachlinie.csv
"""
import numpy as np, os
SP = os.path.dirname(os.path.abspath(__file__))
XOFF, UINF, RHO = 2.2063, 30.0, 1.225

C  = np.load(SP+"/of13_vehicle_C.npy").astype(np.float64)
TW = np.load(SP+"/of13_vehicle_TW.npy").astype(np.float64)
S  = np.loadtxt(SP+"/of13_slab_y003.txt")           # x y z p Ux Uy Uz
np.save(SP+"/of13_slab.npy", S.astype(np.float32))

HY = 0.020                                          # Mittelstreifen-Halbbreite
m  = np.abs(C[:,1]) < HY
c, t = C[m], TW[m]

DXB = 0.010                                          # x-Bin
xr0, xr1 = -0.10, 1.60                               # OF13-x: Dachscheitel-Bereich .. hinter Heckscheibe
bins = np.arange(xr0, xr1+1e-9, DXB)
zroof, xroof, twx, twz, nfac = [], [], [], [], []
for i in range(len(bins)-1):
    s = (c[:,0] >= bins[i]) & (c[:,0] < bins[i+1])
    if s.sum() < 3: continue
    zmax = c[s,2].max()
    o = s & (C[m][:,2] > zmax - 0.030)               # nur die OBERE Haut in diesem Bin
    xroof.append(c[o,0].mean()); zroof.append(c[o,2].mean())
    twx.append(t[o,0].mean()); twz.append(t[o,2].mean()); nfac.append(o.sum())
xroof=np.array(xroof); zroof=np.array(zroof); twx=np.array(twx); twz=np.array(twz); nfac=np.array(nfac)

# Tangente entlang der Kontur (zentrale Differenz)
dx = np.gradient(xroof); dz = np.gradient(zroof); L = np.hypot(dx,dz)
tx, tz = dx/L, dz/L
tau_t = twx*tx + twz*tz                              # tangentiale Wandschubspannung / rho  [m^2/s^2]
cf    = tau_t/(0.5*UINF**2)

# --- Wanddruck: naechstgelegene Zellmitte ueber der Kontur (erste Fluidzelle)
sx, sy, sz, sp = S[:,0], S[:,1], S[:,2], S[:,3]
cp_w = np.full(len(xroof), np.nan); dwall = np.full(len(xroof), np.nan)
nx, nz = -tz, tx                                     # Normale nach OBEN (tx>0 -> nz=tx>0)
for i,(xw,zw) in enumerate(zip(xroof,zroof)):
    d = np.hypot(sx-xw, sz-zw)
    k = (d < 0.060)
    if k.sum()==0: continue
    # nur Zellen auf der Aussenseite (positive Normalkomponente)
    dn = (sx[k]-xw)*nx[i] + (sz[k]-zw)*nz[i]
    kk = dn > 0.0
    if kk.sum()==0: continue
    j = np.argmin(dn[kk])
    cp_w[i] = sp[k][kk][j]/(0.5*UINF**2)
    dwall[i] = dn[kk][j]

hdr = ("x_of13_m,x_v2_m,z_m,winkel_grad,n_facetten,tau_t_m2s2,cf,cp_wand,wandabstand_m")
ang = np.degrees(np.arctan2(-dz,dx))
np.savetxt(SP+"/of13_dachlinie.csv",
           np.c_[xroof, xroof+XOFF, zroof, ang, nfac, tau_t, cf, cp_w, dwall],
           delimiter=",", header=hdr, comments="", fmt="%.6g")
print("geschrieben of13_dachlinie.csv,", len(xroof), "Bins")
for i in range(0,len(xroof),5):
    print(f"x_v2 {xroof[i]+XOFF:6.3f}  z {zroof[i]:6.3f}  ang {ang[i]:6.2f}  nf {nfac[i]:4d}  "
          f"tau_t {tau_t[i]:+9.4f}  cf {cf[i]:+8.5f}  cp {cp_w[i]:+7.3f}  dw {1000*dwall[i]:5.2f} mm")
