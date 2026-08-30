#!/usr/bin/env python3
"""fx_profil.py -- FX: wandnormale Grenzschichtprofile und wandtangentiale Geschwindigkeit
ueber Dach und Heckscheibe, aus dem y-Band (fx_band.py-npz).

Wand = Voxelkoerper (flags & TYPE_S). Je y-Ebene und x-Station:
  z_w = Oberkante der hoechsten Solidzelle;  Kontur ueber +-GLATT Zellen in x geglaettet
  -> Tangente t, Normale n;  entlang n wird u bilinear aus der (x,z)-Ebene abgetastet.
u_t = u.t;  u_e = max(u_t) laengs des Strahls;  delta99 = kleinstes s mit u_t >= 0.99*u_e;
delta* = Int (1-u_t/u_e) ds ; theta = Int u_t/u_e (1-u_t/u_e) ds ; H = delta*/theta.
Abgetastete Punkte in Solidzellen werden verworfen (u_t = 0 gesetzt, s zaehlt weiter).
"""
import sys, os
import numpy as np
TYPE_S = 0x01; U_LAT = 0.075; CP_FAK = 2.0/(3.0*U_LAT*U_LAT); Q_INF = 0.5*1.225*30.0**2

def glatt(a, w):
    k = np.ones(2*w+1)/(2*w+1)
    return np.convolve(np.pad(a, w, mode="edge"), k, mode="valid")

def lade(npz):
    d = np.load(npz)
    return (d["u"], d["rho"], d["flags"], tuple(d["dims"]), tuple(d["orig"]), float(d["dx"]))

def profile(npz, x0=2.0, x1=4.05, smm=40.0, nmax=0.35, glatt_mm=24.0):
    U, RHO, FL, (Nx,Ny,Nz), (ox,oy,oz), dx = lade(npz)
    ny = U.shape[1]
    x = ox + np.arange(Nx)*dx; z = oz + np.arange(Nz)*dx
    i0, i1 = int(np.searchsorted(x, x0)), int(np.searchsorted(x, x1))
    k0, k1 = int(np.searchsorted(z, 0.30)), int(np.searchsorted(z, 1.62))
    solid = (FL & TYPE_S) != 0
    ns = int(round(nmax/dx))
    sdist = (np.arange(ns)+0.5)*dx
    w = max(1, int(round(0.001*glatt_mm/dx)))
    NX = i1-i0
    UT = np.full((ny, NX, ns), np.nan)
    ZW = np.full((ny, NX), np.nan); CP = np.full((ny, NX), np.nan)
    UX1 = np.full((ny, NX), np.nan)
    for j in range(ny):
        sc = solid[k0:k1, j, i0:i1]
        hat = sc.any(axis=0)
        kt = np.where(hat, k0 + (k1-k0-1) - np.argmax(sc[::-1], axis=0), -1)
        zw = np.where(hat, z[np.clip(kt,0,Nz-1)] + 0.5*dx, np.nan)
        ZW[j] = zw
        gut = hat & (kt+1 < Nz)
        idx = np.arange(NX)
        kf = np.clip(kt+1, 0, Nz-1)
        fluid1 = gut & ~solid[kf, j, i0+idx]
        CP[j, fluid1]  = (RHO[kf[fluid1], j, i0+idx[fluid1]]-1.0)*CP_FAK
        UX1[j, fluid1] = U[kf[fluid1], j, i0+idx[fluid1], 0]
        zg = glatt(np.nan_to_num(zw, nan=np.nanmean(zw)), w)
        dxx = np.gradient(x[i0:i1]); dzz = np.gradient(zg); L = np.hypot(dxx,dzz)
        tx, tz = dxx/L, dzz/L
        nx_, nz_ = -tz, tx                      # nach oben (tx>0)
        # Abtastpunkte
        px = x[i0:i1][:,None] + nx_[:,None]*sdist[None,:]
        pz = zw[:,None]       + nz_[:,None]*sdist[None,:]
        fi = (px-ox)/dx; fk = (pz-oz)/dx
        i_ = np.clip(np.floor(fi).astype(int), 0, Nx-2); a = fi-i_
        k_ = np.clip(np.floor(fk).astype(int), 0, Nz-2); b = fk-k_
        for c,comp in enumerate((0,2)):
            pass
        def bil(F):
            return ((1-a)*(1-b)*F[k_, j, i_] + a*(1-b)*F[k_, j, i_+1]
                    + (1-a)*b*F[k_+1, j, i_] + a*b*F[k_+1, j, i_+1])
        ux = bil(U[:,:,:,0]); uz = bil(U[:,:,:,2])
        so = (solid[k_, j, i_] | solid[k_, j, i_+1] | solid[k_+1, j, i_] | solid[k_+1, j, i_+1])
        ut = ux*tx[:,None] + uz*tz[:,None]
        ut[so] = np.nan
        UT[j] = ut
    ut = np.nanmean(UT, axis=0)                 # (NX, ns)
    zw = np.nanmean(ZW, axis=0); cp = np.nanmean(CP, axis=0); ux1 = np.nanmean(UX1, axis=0)
    negfrac = np.nanmean(UX1 < 0.0, axis=0)
    return dict(x=x[i0:i1], s=sdist, ut=ut, zw=zw, cp=cp, ux1=ux1, negfrac=negfrac, dx=dx,
                utneg=np.nanmean(UT[:,:,0] < 0.0, axis=0))

def kenn(res):
    """delta99, delta*, theta, H je x-Station"""
    x, s, ut = res["x"], res["s"], res["ut"]
    d99=[]; ds=[]; th=[]
    for i in range(len(x)):
        u = ut[i].copy()
        u = np.where(np.isnan(u), 0.0, u)
        ue = np.max(u)
        if ue <= 1e-6: d99.append(np.nan); ds.append(np.nan); th.append(np.nan); continue
        ke = int(np.argmax(u))
        j = np.argmax(u >= 0.99*ue)
        d99.append(s[j])
        f = u[:ke+1]/ue
        ss = s[:ke+1]
        ds.append(np.trapz(1-f, ss)); th.append(np.trapz(f*(1-f), ss))
    d99=np.array(d99); ds=np.array(ds); th=np.array(th)
    return d99, ds, th, ds/np.where(th>0, th, np.nan)

if __name__ == "__main__":
    npz, out = sys.argv[1], sys.argv[2]
    r = kenn_res = profile(npz)
    d99, ds, th, H = kenn(r)
    np.savez_compressed(out, **r, d99=d99, dstern=ds, theta=th, H=H)
    print(os.path.basename(npz), "->", out, " NX", len(r["x"]), " ns", len(r["s"]))
