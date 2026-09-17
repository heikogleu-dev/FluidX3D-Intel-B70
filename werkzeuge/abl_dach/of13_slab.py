#!/usr/bin/env python3
"""of13_slab.py -- erzeugt of13_slab.npy (x y z p Ux Uy Uz, float32) fuer of13_profil.py neu.

Die Datei lag am 30.08. vor, war aber nicht versioniert (*.npz/*.npy in .gitignore) und fehlte am
17.09.2026. Quelle: ~/CFD-Cases/mr2v40H/1200/{C,p,U}.gz, internalField (Zellmitten, p kinematisch).
Auswahl wie im Kopf von of13_dachlinie.py dokumentiert: |y| < 0,03 m.
Aufruf: of13_slab.py [fall] [zeit] [y_halb_m]
"""
import sys, os, gzip
import numpy as np
FALL = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/CFD-Cases/mr2v40H")
ZEIT = sys.argv[2] if len(sys.argv) > 2 else "1200"
YH = float(sys.argv[3]) if len(sys.argv) > 3 else 0.03
SP = os.path.dirname(os.path.abspath(__file__))

def internal(pfad, k):
    with gzip.open(pfad, "rt", errors="replace") as f:
        for z in f:
            if z.startswith("internalField"): break
        else: raise SystemExit(f"internalField fehlt in {pfad}")
        n = None
        for z in f:
            s = z.strip()
            if not s: continue
            if n is None and s.isdigit(): n = int(s); continue
            if n is not None and s == "(": break
        rest = f.read()
    rest = rest[:rest.index("\n)")]
    if k > 1: rest = rest.replace("(", " ").replace(")", " ")
    w = np.fromstring(rest, sep=" ", dtype=np.float64)
    if w.size != n * k: raise SystemExit(f"{pfad}: {w.size} statt {n*k} Werte")
    return w.reshape(n, k) if k > 1 else w

C = internal(os.path.join(FALL, ZEIT, "C.gz"), 3); print("C", C.shape, flush=True)
m = np.abs(C[:, 1]) < YH
print(f"  |y|<{YH}: {int(m.sum())} Zellen von {len(m)}", flush=True)
p = internal(os.path.join(FALL, ZEIT, "p.gz"), 1)
if p.size != len(m): raise SystemExit("p-Laenge passt nicht zu C")
U = internal(os.path.join(FALL, ZEIT, "U.gz"), 3)
if U.shape[0] != len(m): raise SystemExit("U-Laenge passt nicht zu C")
S = np.column_stack([C[m], p[m], U[m]]).astype(np.float32)
np.save(os.path.join(SP, "of13_slab.npy"), S)
print(f"of13_slab.npy: {S.shape}, x {S[:,0].min():.3f}..{S[:,0].max():.3f}, p {S[:,3].min():.1f}..{S[:,3].max():.1f}")
