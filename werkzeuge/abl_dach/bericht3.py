#!/usr/bin/env python3
"""bericht3.py -- Dach/Heckscheibe gegen OF13, parametrisiert (17.09.2026; bericht2.py hatte die Laufnamen fest).
Aufruf: bericht3.py name1 [name2 ...]   (liest prof2_<name>.npz und prof_of13.npz aus diesem Ordner)
Tabellen: A Abloeseort je Kriterium, B Wanddruck cp, C Grenzschicht-Integralgroessen, D u_t(s) an Stationen,
E Rueckstromanteil in der ersten Fluidzelle. FX = Momentanfelder, y-Band +-40 mm gemittelt; OF13 = RANS, |y|<30 mm.
Konventionen unveraendert aus bericht2.py (Glaettung +-24 mm, Kriterium 'durchgehende Rueckstroemung bis x_v2 = XE').
★ 17.09.2026 nach Pruefagent: cp beider Seiten auf den Totaldruck-Bezug ueber dem Dach gestellt (FX: cp_ref in prof2_*.npz,
OF13: cp_ref in prof_of13.npz); je Wandabstand werden die TATSAECHLICH benutzten Stuetzstellen beider Seiten gedruckt."""
import sys, os, warnings
import numpy as np
warnings.filterwarnings("ignore")
SP = os.path.dirname(os.path.abspath(__file__)); XE = 3.62
NAM = sys.argv[1:]
def glatt(a, w):
    k = np.ones(2*w+1)/(2*w+1); return np.convolve(np.pad(a, w, mode="edge"), k, mode="valid")
def durch(x, v, xend=XE):
    m = x <= xend; xx, vv = x[m], v[m]; ok = vv < 0
    if not ok.any() or not ok[-1]: return np.nan
    i = len(ok)-1
    while i > 0 and ok[i-1]: i -= 1
    return xx[i]
O = np.load(SP+"/prof_of13.npz"); xo, so = O["x"], O["s"]
F = {k: np.load(SP+f"/prof2_{k}.npz") for k in NAM}
W = {k: int(round(0.024/float(F[k]["dx"]))) for k in NAM}
def at(d, key, xq, kk=None):
    i = int(np.argmin(np.abs(d["x"]-xq))); v = d[key][i] if kk is None else d[key][i, kk]
    return v
kopf = lambda t: print(f"{t:14s} {'OF13':>8s} " + " ".join(f"{k:>11s}" for k in NAM))

print("A) Abloeseort x_s [m]: Beginn der durchgehenden Rueckstroemung bis x_v2 =", XE)
kopf("Kriterium")
cf = O["cf"]; xs = np.nan
for i in range(len(xo)-1):   # OpenFOAM-Vorzeichen: anliegend cf<0, Abloesung = Wechsel nach positiv
    if xo[i] > 2.6 and cf[i] < 0 <= cf[i+1]: xs = xo[i]+(xo[i+1]-xo[i])*(-cf[i])/(cf[i+1]-cf[i]); break
print(f"{'tau_w (cf=0)':14s} {xs:8.3f} " + " ".join(f"{'--':>11s}" for _ in NAM))
print(f"{'1. Fluidzelle':14s} {'--':>8s} " + " ".join(f"{durch(F[k]['x'], glatt(np.nan_to_num(F[k]['ux1']), W[k])):11.3f}" for k in NAM))
for smm in (8, 16, 24, 32, 48):
    ko = int(np.argmin(np.abs(so-smm/1000)))
    z = f"{'u_t @ '+str(smm)+' mm':14s} {durch(xo, O['ut'][:, ko]):8.3f} "
    for k in NAM:
        d = F[k]; kk = int(np.argmin(np.abs(d["s"]-smm/1000)))
        z += f"{durch(d['x'], glatt(np.nan_to_num(d['utg'][:, kk]), W[k])):11.3f} "
    print(z)

ST = np.round(np.arange(2.30, 3.96, 0.10), 2)
print("\nB) Wanddruck cp (OF13 Eigentuemerzelle; FX erste Fluidzelle, +-24 mm geglaettet)")
D = np.genfromtxt(SP+"/of13_dachlinie.csv", delimiter=",", names=True); xw, cpw = D["x_v2_m"], D["cp_wand"] - float(O["cp_ref"])
for k in NAM:
    if "cp_ref" not in F[k].files: raise SystemExit(f"prof2_{k}.npz ohne cp_ref -- mit dem Stand vom 17.09. neu rechnen")
print("   cp_ref: OF13 %+.4f | " % float(O["cp_ref"]) + " ".join(f"{k} {np.round(F[k]['cp_ref'],4).tolist()}" for k in NAM))
kopf("x_v2 [m]")
for xq in ST:
    print(f"{xq:<14.2f} {np.interp(xq, xw, cpw):8.3f} " + " ".join(
        f"{np.interp(xq, F[k]['x'], glatt(np.nan_to_num(F[k]['cp']), W[k])):11.3f}" for k in NAM))

print("\nC) Grenzschicht: delta99 [mm] / H  (OF13 aus 2-mm-Bins; FX aus dem geglaetteten Profil)")
kopf("x_v2 [m]")
for xq in ST:
    io = int(np.argmin(np.abs(xo-xq)))
    z = f"{xq:<14.2f} {1e3*O['d99'][io]:4.0f}/{O['H'][io]:3.1f} "
    for k in NAM: z += f"{1e3*at(F[k],'d99',xq):6.0f}/{at(F[k],'H',xq):4.2f}"
    print(z)

print("\nD) u_t [m/s] im Wandabstand s (Zeilen) an Stationen; je Zelle OF13 | " + " | ".join(NAM))
print("   benutzte Stuetzstellen s [mm] je Zeile: OF13 " + " ".join(f"{1e3*so[int(np.argmin(np.abs(so-v/1000)))]:.1f}" for v in (2,4,8,16,32,64,128))
      + " | FX " + " ".join(f"{1e3*F[NAM[0]]['s'][int(np.argmin(np.abs(F[NAM[0]]['s']-v/1000)))]:.2f}" for v in (2,4,8,16,32,64,128)))
for xq in (2.50, 2.90, 3.20, 3.40, 3.55, 3.65, 3.75):
    io = int(np.argmin(np.abs(xo-xq)))
    print(f"  x_v2 = {xq:.2f} m")
    for smm in (2, 4, 8, 16, 32, 64, 128):
        ko = int(np.argmin(np.abs(so-smm/1000)))
        z = f"    s={smm:4d} mm  {O['ut'][io, ko]:6.1f} |"
        for k in NAM:
            d = F[k]; kk = int(np.argmin(np.abs(d["s"]-smm/1000)))
            z += f" {at(d,'utg',xq,kk):6.1f}"
        print(z)

print("\nE) Rueckstromanteil erste Fluidzelle (Anteil y-Ebenen x Zeitpunkte mit u_x<0), Mittel je 100-mm-Abschnitt")
kopf("Abschnitt")
for a in np.round(np.arange(3.00, 3.91, 0.10), 2):
    z = f"{a:.2f}-{a+0.1:.2f}     {'--':>8s} "
    for k in NAM:
        d = F[k]; m = (d["x"] >= a) & (d["x"] < a+0.1); z += f"{np.nanmean(d['negfrac'][m]):11.3f} "
    print(z)
