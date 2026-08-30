#!/usr/bin/env python3
"""fx_dach.py -- FX-Dachlinie aus einem y-Band (fx_band.py-npz).

Alles wird aus dem VOXELKOERPER abgeleitet (flags & TYPE_S = 0x01), nie aus der STL.
Je (y,x)-Saeule wird von oben nach unten die HOECHSTE Solidzelle im Fenster
z in [z_min_koerper, z_max_koerper] gesucht -> Dachzelle. Die Zelle darueber ist die
ERSTE FLUIDZELLE; genau darauf beruht das Abloesekriterium aus AUDIT-BEFUNDE B58
("u_x in der ersten Fluidzelle").

Druck: p_lat = (rho-1)/3 (Gittereinheiten); cp = p_lat/(0.5*u_lat^2) = (rho-1)*118.5185
mit u_lat = 0.075 (setup.cpp:4051); p_Pa = cp*0.5*si_rho*si_u^2 = cp*551.25
(si_u = 30 m/s, si_rho = 1.225 -- setup.cpp:4043-4051).

Aufruf: fx_dach.py <band.npz> <out.csv> [x0_m] [x1_m]
"""
import sys, os
import numpy as np
TYPE_S = 0x01
U_LAT  = 0.075
CP_FAK = 2.0/(3.0*U_LAT*U_LAT)      # 118.5185: cp aus (rho-1)
Q_INF  = 0.5*1.225*30.0**2          # 551.25 Pa

def dachlinie(npz, x0=1.6, x1=4.6, zmin=0.30, zmax=1.60):
    d = np.load(npz)
    U, RHO, FL = d["u"], d["rho"], d["flags"]        # (Nz, ny, Nx, 3) / (Nz,ny,Nx)
    Nx, Ny, Nz = d["dims"]; ox, oy, oz = d["orig"]; dx = float(d["dx"])
    ny = U.shape[1]
    x = ox + np.arange(Nx)*dx
    z = oz + np.arange(Nz)*dx
    i0, i1 = np.searchsorted(x, x0), np.searchsorted(x, x1)
    k0, k1 = np.searchsorted(z, zmin), np.searchsorted(z, zmax)
    solid = (FL & TYPE_S) != 0                        # (Nz, ny, Nx)
    out = []
    for i in range(i0, i1):
        ztop_l, ux_l, ut_l, cp_l, umag_l = [], [], [], [], []
        for j in range(ny):
            col = solid[k0:k1, j, i]
            if not col.any(): continue
            kt = k0 + np.max(np.where(col)[0])        # hoechste Solidzelle
            if kt+1 >= Nz or solid[kt+1, j, i]: continue
            ztop_l.append(z[kt]); 
            ux_l.append(U[kt+1, j, i, 0]); umag_l.append(np.linalg.norm(U[kt+1, j, i]))
            cp_l.append((RHO[kt+1, j, i]-1.0)*CP_FAK)
        if len(ztop_l) == 0: continue
        out.append([x[i], np.mean(ztop_l), np.mean(ux_l), np.mean(cp_l), np.mean(umag_l),
                    len(ztop_l), np.mean(np.array(ux_l) < 0.0)])
    R = np.array(out)
    return R, dx

def main():
    npz, out = sys.argv[1], sys.argv[2]
    x0 = float(sys.argv[3]) if len(sys.argv) > 3 else 1.6
    x1 = float(sys.argv[4]) if len(sys.argv) > 4 else 4.6
    R, dx = dachlinie(npz, x0, x1)
    x, ztop, ux, cp, umag, n, frac = R.T
    zw = ztop + 0.5*dx                                 # Wandlage = Oberkante der Dachzelle
    dxg = np.gradient(x); dzg = np.gradient(zw); L = np.hypot(dxg, dzg)
    ang = np.degrees(np.arctan2(-dzg, dxg))
    np.savetxt(out, np.c_[x, zw, ang, n, ux, ux/30.0, cp, cp*Q_INF, frac, umag],
               delimiter=",", fmt="%.6g",
               header="x_v2_m,z_wand_m,winkel_grad,n_saeulen,ux_erstezelle_ms,ux_rel,cp_erstezelle,p_erstezelle_Pa,anteil_ux_negativ,umag_erstezelle_ms",
               comments="")
    print(f"{os.path.basename(npz)}: {len(x)} x-Stationen, dx = {dx*1000:.0f} mm -> {out}")

if __name__ == "__main__": main()
