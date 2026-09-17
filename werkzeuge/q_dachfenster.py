#!/usr/bin/env python3
"""q_dachfenster.py -- q-Verteilung am Fahrzeugdach aus fac_q_dump_<lauf>_D<Nx>.csv (CFD_FAC_QDUMP=1).

Beantwortet die Frage vom 03.09.2026: bewegt mehr Taubin-Glaettung (CFD_FACETTEN_REMESH_ITER) das q
am FLACHEN DACH von 0,5 weg? Dort ist heute chi = 0 und ELIBB bitgleich reines BB (B58: 99,6 % der
Links innerhalb |q-0,5| <= 0,1). Die Kugel-Eichung vom 22.08. sagt dazu nichts -- dort war q_min das
bindende Kriterium, hier ist es die Unbeweglichkeit von q.

Aufruf: q_dachfenster.py [--lauf NAME] <dump1.csv> [[--lauf NAME] dump2.csv ...]   (--lauf gilt nur fuer die direkt folgende Datei)
Dump-Format (lbm.cpp:698): fid,x,y,z,nx,ny,nz,yw,qb1..qb18   mit q = qb/254, qb=0 = kein Schnitt.
x,y,z sind ZELLINDIZES. Weltkoordinate: x_welt = X0 + i*DX.
★ 17.09.2026 (SKALIERUNG-BEFUNDE Nebenbefund 10): DX stand fest auf 0,004 und X0 auf p4_ref -- auf 8 / 3,75 / 16 mm lagen die
Fenster DACH/RAMPE damit an falscher Stelle, ohne Meldung. X0 und DX kommen jetzt aus dem LAUF des Dumps (Dateiname
fac_q_dump_<lauf>_D<Nx>.csv oder --lauf NAME davor): export/<lauf>/einlass_saeule_nah.csv-Kopf (x_f=2 / x_f=10, 7 Stellen,
die Quelle der alten Konstante) -> feld_nah_*.vtk-Kopf (ORIGIN/SPACING) -> Abbruch. Die Quelle wird gedruckt; D<Nx> wird
gegen DIMENSIONS geprueft, wo ein VTK vorliegt.
"""
import sys, os, re, glob, numpy as np
WURZEL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DACH = (2.30, 3.60)             # Dach + Heckscheibe, x_v2
RAMPE = (3.10, 3.60)            # der flach abfallende Teil -- dort sitzt die Abloesung

def gitter(lauf, nx_datei=None):
    """(X0, DX, quelle) des Nahfelds von export/<lauf>. SystemExit ohne Quelle."""
    d = os.path.join(WURZEL, "export", lauf)
    vtk = sorted(glob.glob(os.path.join(d, "feld_nah_*.vtk")))
    nx_vtk = None
    if vtk:
        with open(vtk[0], "rb") as f: kopf = f.read(1024).decode("ascii", "replace")
        nx_vtk = int(re.search(r"DIMENSIONS (\d+)", kopf).group(1))
        if nx_datei is not None and nx_datei != nx_vtk:
            raise SystemExit(f"FEHLER: Dump sagt D{nx_datei}, {vtk[0]} hat Nx = {nx_vtk} -- falscher Lauf?")
    dx_vtk = x0_vtk = None
    if vtk:
        m = re.search(r"ORIGIN (\S+) \S+ \S+\s+SPACING (\S+)", kopf); x0_vtk, dx_vtk = float(m.group(1)), float(m.group(2))
    es = os.path.join(d, "einlass_saeule_nah.csv")
    if os.path.exists(es):
        m = re.search(r"x_f=2 -> (-?[0-9.]+) m, x_f=10 -> (-?[0-9.]+) m", open(es).readline())
        if m:   # 7 Stellen: genauer als ORIGIN im VTK (6 Stellen); DX aus SPACING, wo vorhanden (die Differenz x10-x2 traegt 1e-7 Rundung)
            x2, x10 = float(m.group(1)), float(m.group(2)); dx = dx_vtk if dx_vtk else (x10 - x2)/8.0
            return x2 - 2*dx, dx, f"{es} Kopf (x_f=2 -> {x2}, x_f=10 -> {x10})" + (f", DX aus {os.path.basename(vtk[0])}" if dx_vtk else ", DX = (x10-x2)/8")
    if vtk:
        return x0_vtk, dx_vtk, f"{vtk[0]} Kopf"
    raise SystemExit(f"FEHLER: X0/DX fuer Lauf {lauf} nicht bestimmbar (weder einlass_saeule_nah.csv noch feld_nah_*.vtk in {d})")

def auswerten(pfad, X0, DX):
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
arg = sys.argv[1:]; lauf_arg = None; i = 0
while i < len(arg):
    if arg[i] == "--lauf": lauf_arg = arg[i+1]; i += 2; continue
    p = arg[i]; i += 1
    nam = os.path.basename(p).replace('fac_q_dump_', '').replace('.csv', '')
    m = re.match(r"fac_q_dump_(.+)_D(\d+)\.csv$", os.path.basename(p))
    lauf = lauf_arg or (m.group(1) if m else None)
    if lauf is None: raise SystemExit(f"FEHLER: Laufname aus {p} nicht lesbar -- --lauf NAME davor angeben")
    # ★ 17.09.2026 (Pruefbefund 2): --lauf gilt NUR fuer die naechste Datei. Vorher blieb lauf_arg fuer alle folgenden Dumps stehen
    # (falsches X0/DX still) und die D<Nx>-Pruefung entfiel fuer sie. D<Nx> aus dem Dateinamen wird jetzt immer geprueft, auch mit --lauf.
    lauf_arg = None
    X0, DX, quelle = gitter(lauf, int(m.group(2)) if m else None)
    print(f"{nam}: Lauf {lauf}, X0 {X0:+.7f} m, DX {DX*1e3:.4f} mm aus {quelle}")
    R = auswerten(p, X0, DX)
    for f in ("DACH", "RAMPE"):
        r = R.get(f)
        if r is None: print(f"{nam:16s} {f:7s} {'-- keine Facetten im Fenster':>40s}"); continue
        print(f"{nam:16s} {f:7s} {r['n_fac']:8d} {r['n_links']:9d} {r['nah05']:12.1f}% {r['rms']:11.4f} {r['qmin']:7.3f} {r['qmax']:7.3f}")
print("\nLESART: sinkt '|q-0,5|<=0,1' deutlich unter die heutigen ~99,6 %, bewegt die Glaettung die Wandlage")
print("am flachen Dach -- dann greift ELIBB dort erstmals. q_min ist die Gegenschranke (a2 = (1-q)/q).")
