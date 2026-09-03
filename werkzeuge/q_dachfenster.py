#!/usr/bin/env python3
"""q_dachfenster.py -- q-Verteilung am Fahrzeugdach aus fac_q_dump_<lauf>_D<Nx>.csv (CFD_FAC_QDUMP=1).

Beantwortet die Frage vom 03.09.2026: bewegt mehr Taubin-Glaettung (CFD_FACETTEN_REMESH_ITER) das q
am FLACHEN DACH von 0,5 weg? Dort ist heute chi = 0 und ELIBB bitgleich reines BB (B58: 99,6 % der
Links innerhalb |q-0,5| <= 0,1). Die Kugel-Eichung vom 22.08. sagt dazu nichts -- dort war q_min das
bindende Kriterium, hier ist es die Unbeweglichkeit von q.

Aufruf: q_dachfenster.py <dump1.csv> [dump2.csv ...]
Dump-Format (lbm.cpp:698): fid,x,y,z,nx,ny,nz,yw,qb1..qb18   mit q = qb/254, qb=0 = kein Schnitt.
x,y,z sind ZELLINDIZES. Weltkoordinate: x_welt = X0 + i*DX (Default aus p4_ref, gleiche Box-Schalter).
"""
import sys, os, numpy as np
DX = 0.004
X0 = -0.3178398 - 2*DX          # aus export/p4_ref/einlass_saeule_nah.csv (x_f=2 -> -0,3178398 m)
DACH = (2.30, 3.60)             # Dach + Heckscheibe, x_v2
RAMPE = (3.10, 3.60)            # der flach abfallende Teil -- dort sitzt die Abloesung
def auswerten(pfad):
    A = np.loadtxt(pfad, delimiter=',', comments='#')
    if A.ndim == 1: A = A[None, :]
    xw = X0 + A[:, 1]*DX
    nz = A[:, 6]
    qb = A[:, 8:26]
    out = {}
    for name, (a, b) in (("DACH", DACH), ("RAMPE", RAMPE)):
        m = (xw >= a) & (xw <= b) & (nz > 0.7)     # nach oben zeigende Facetten im Fenster
        if not m.any(): out[name] = None; continue
        q = qb[m].ravel(); q = q[q > 0]/254.0      # nur geschnittene Links
        if len(q) == 0: out[name] = None; continue
        out[name] = dict(n_fac=int(m.sum()), n_links=len(q),
                         nah05=100.0*np.mean(np.abs(q-0.5) <= 0.1),
                         rms=float(np.sqrt(np.mean((q-0.5)**2))),
                         qmin=float(q.min()), qmax=float(q.max()))
    return out
print(f"{'Lauf':16s} {'Fenster':7s} {'n_fac':>8s} {'n_links':>9s} {'|q-0,5|<=0,1':>13s} {'RMS(q-0,5)':>11s} {'q_min':>7s} {'q_max':>7s}")
for p in sys.argv[1:]:
    nam = os.path.basename(p).replace('fac_q_dump_', '').replace('.csv', '')
    R = auswerten(p)
    for f in ("DACH", "RAMPE"):
        r = R.get(f)
        if r is None: print(f"{nam:16s} {f:7s} {'-- keine Facetten im Fenster':>40s}"); continue
        print(f"{nam:16s} {f:7s} {r['n_fac']:8d} {r['n_links']:9d} {r['nah05']:12.1f}% {r['rms']:11.4f} {r['qmin']:7.3f} {r['qmax']:7.3f}")
print("\nLESART: sinkt '|q-0,5|<=0,1' deutlich unter die heutigen ~99,6 %, bewegt die Glaettung die Wandlage")
print("am flachen Dach -- dann greift ELIBB dort erstmals. q_min ist die Gegenschranke (a2 = (1-q)/q).")
