#!/usr/bin/env python3
"""stromauf.py -- Haube/Scheibe/Dach FX gegen OF13 an Stationen (17.09.2026; ersetzt die Ad-hoc-Tabelle des Vormittags).
Aufruf: stromauf.py name1 [name2 ...]  (prof2_<name>.npz mit cp_ref, prof_of13.npz mit cp_ref; x-Bereich per DACH_X0/X1 gerechnet)
Je Station: u_t an den NAECHSTEN Stuetzstellen zu s = 2 / 8 / 16 mm (tatsaechliche s werden gedruckt), cp bezugskorrigiert, delta99, delta90."""
import sys, os, warnings
import numpy as np
warnings.filterwarnings("ignore")
SP = os.path.dirname(os.path.abspath(__file__))
def glatt(a, w):
    k = np.ones(2*w+1)/(2*w+1); return np.convolve(np.pad(a, w, mode="edge"), k, mode="valid")
O = np.load(SP+"/prof_of13.npz"); D = np.genfromtxt(SP+"/of13_dachlinie.csv", delimiter=",", names=True)
N = sys.argv[1:]; F = {k: np.load(SP+f"/prof2_{k}.npz") for k in N}
for k in N:
    if "cp_ref" not in F[k].files: raise SystemExit(f"prof2_{k}.npz ohne cp_ref")
def s90(u, s):
    ok = ~np.isnan(u)
    if ok.sum() < 5: return np.nan
    uu = np.interp(s, s[ok], u[ok]); return s[np.argmax(uu >= 0.9*uu.max())]
SO = [round(float(O["s"][int(np.argmin(abs(O["s"]-v/1000)))])*1e3, 2) for v in (2, 8, 16)]
print(f"Stuetzstellen OF13 s = {SO} mm; FX s = {[round(float(F[N[0]]['s'][int(np.argmin(abs(F[N[0]]['s']-v/1000)))])*1e3,2) for v in (2,8,16)]} mm; u_lat FX {[F[k]['u_lat'].tolist() for k in N]}; cp_ref OF13 {float(O['cp_ref']):+.4f}, FX " + " ".join(f"{k} {np.round(F[k]['cp_ref'],4).tolist()}" for k in N))
print("x_v2 | OF13 u_t(s1,s2,s3) cp d99 d90 || je FX-Arm dasselbe")
for xq in np.round(np.arange(0.30, 3.71, 0.10), 2):
    io = int(np.argmin(abs(O["x"]-xq))); u = lambda sm: O["ut"][io, int(np.argmin(abs(O["s"]-sm/1000)))]
    z = f"{xq:4.2f} | {u(2):5.1f} {u(8):5.1f} {u(16):5.1f} cp {np.interp(xq, D['x_v2_m'], D['cp_wand'])-float(O['cp_ref']):+.2f} d99 {1e3*O['d99'][io]:4.0f} d90 {1e3*s90(O['ut'][io], O['s']):4.0f}"
    for k in N:
        d = F[k]; i = int(np.argmin(abs(d["x"]-xq))); w = int(round(0.024/float(d["dx"])))
        v = lambda sm: d["utg"][i, int(np.argmin(abs(d["s"]-sm/1000)))]
        z += f" || {v(2):5.1f} {v(8):5.1f} {v(16):5.1f} cp {np.interp(xq, d['x'], glatt(np.nan_to_num(d['cp']), w)):+.2f} d99 {1e3*d['d99'][i]:4.0f} d90 {1e3*d['s90'][i]:4.0f}"
    print(z)
