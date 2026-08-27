#!/usr/bin/env python3
# of13_kraft_zband.py -- OF13-Fahrzeugkraft nach HOEHENBAND zerlegen (27.08.2026).
#
# WOZU: FluidX3D-v2 zieht vom Facetten-Druckpfad das unterste z-Band (Latsch, 16 mm) ab,
# weil es dort ein Voxel-Artefakt gibt (0,25 % der Wandzellen tragen 30,3 % des Druck-Cd).
# OF13 zieht NICHTS ab. Der Vergleich 0,8052 gegen 0,559 stellt also "FX ohne Latschband"
# gegen "OF13 mit allem". Dieses Werkzeug misst, wieviel OF13 im selben Band traegt --
# erst dann ist bekannt, ob der FX-Abzug nur Artefakt oder auch echte Kraft entfernt.
#
# VERFAHREN (rein lesend, der Referenzfall wird NICHT veraendert):
#   p ist auf dem vehicle-Patch zeroGradient -> Druck der OWNER-Zelle je Randflaeche.
#   wallShearStress liegt als calculated-Randfeld vor (2.648.253 Vektoren).
#   Sf aus faces+points (Fan-Triangulierung um den Punktmittelwert, exakt fuer ebene und
#   fuer windschiefe Polygone gleichermassen wie OpenFOAMs primitiveMesh).
#   F_druck = rhoInf * SUM p_kin * Sf      (Sf zeigt aus dem Fluid heraus)
#   F_reib  = -rhoInf * SUM tau * |Sf|     (Minus: wallShearStress traegt es bereits -- die
#                                          Kontrollsumme gegen forces.dat hat es aufgedeckt)
# ABNAHME: die Summen muessen forces.dat bei derselben Zeit reproduzieren -- das Werkzeug
# bricht ab, wenn die Abweichung > 1 % ist (sonst waere die Bandzerlegung wertlos).
#
# Aufruf: of13_kraft_zband.py [fall=~/CFD-Cases/mr2v40H] [zeit=1200] [band_mm=16,32,64]
import sys, os, gzip, subprocess
import numpy as np

FALL = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/CFD-Cases/mr2v40H")
ZEIT = sys.argv[2] if len(sys.argv) > 2 else "1200"
BAENDER_MM = [float(v) for v in (sys.argv[3].split(",") if len(sys.argv) > 3 else ["16", "32", "64"])]
RHO_INF, A_REF, U_INF = 1.225, 1.85, 30.0
Q_INF = 0.5 * RHO_INF * U_INF**2

def oeffne(pfad):
    return gzip.open(pfad + ".gz", "rt", errors="replace") if os.path.exists(pfad + ".gz") else open(pfad, errors="replace")

def patch_info(fall, name):
    txt = open(os.path.join(fall, "constant/polyMesh/boundary"), errors="replace").read()
    i = txt.find("\n    " + name + "\n")
    if i < 0: raise SystemExit(f"Patch {name} nicht in boundary")
    blk = txt[i:txt.find("}", i)]
    nf = int(blk.split("nFaces")[1].split(";")[0])
    sf = int(blk.split("startFace")[1].split(";")[0])
    return nf, sf

def liste_schwanz(pfad, n):
    """Letzte n Eintraege einer OpenFOAM-Liste (eine Zeile je Eintrag), ueber zcat|tail."""
    roh = pfad + (".gz" if os.path.exists(pfad + ".gz") else "")
    cat = "zcat" if roh.endswith(".gz") else "cat"
    aus = subprocess.run(f"{cat} '{roh}' | tail -n {n + 6}", shell=True, capture_output=True, text=True).stdout
    return [z for z in aus.splitlines() if z and z[0] not in ")(/"][-n:]

def block_werte(pfad, marker, komponenten=1):
    """Liest den nonuniform-List-Block nach `marker`. Der Block geht in EINEM
    np.fromstring ins Array -- eine Zeilenschleife ueber 34 Mio. Werte laeuft in
    Python zu lange (gemessen 27.08.: Punkte 37,5 Mio. ~2 min, Faces waeren Stunden)."""
    with oeffne(pfad) as f:
        for zeile in f:
            if marker in zeile: break
        else: raise SystemExit(f"{marker} nicht in {pfad}")
        n = None
        for zeile in f:
            s = zeile.strip()
            if not s: continue
            if n is None:
                if s.isdigit(): n = int(s); continue
                if s.startswith("uniform"): raise SystemExit(f"{marker}: uniform-Feld")
                continue
            if s == "(": break
        rest = f.read()
    rest = rest[:rest.index("\n)")]
    if komponenten > 1: rest = rest.replace("(", " ").replace(")", " ")
    w = np.fromstring(rest, sep=" " if komponenten > 1 else "\n", dtype=np.float64)
    if w.size != n * komponenten: raise SystemExit(f"{marker}: {w.size} statt {n*komponenten} Werte")
    return w.reshape(n, komponenten) if komponenten > 1 else w

print(f"OF13-Fall {FALL}, Zeit {ZEIT}")
nf, sf = patch_info(FALL, "vehicle")
print(f"Patch vehicle: {nf} Randflaechen ab Flaeche {sf}")

print("  points ...", flush=True)
with oeffne(os.path.join(FALL, "constant/polyMesh/points")) as f:
    n = None
    for zeile in f:
        t = zeile.strip()
        if n is None and t.isdigit(): n = int(t); continue
        if n is not None and t == "(": break
    roh = f.read()
roh = roh[:roh.index("\n)")].replace("(", " ").replace(")", " ")
pts = np.fromstring(roh, sep=" ", dtype=np.float64).reshape(n, 3); del roh
print(f"  {n} Punkte, z {pts[:,2].min():.3f}..{pts[:,2].max():.3f} m", flush=True)

print("  faces (Schwanz) ...", flush=True)
zeilen = liste_schwanz(os.path.join(FALL, "constant/polyMesh/faces"), nf)
if len(zeilen) != nf: raise SystemExit(f"faces: {len(zeilen)} statt {nf}")
ns = np.fromiter((int(z[:z.index("(")]) for z in zeilen), dtype=np.int64, count=nf)
idx = np.fromstring(" ".join(z[z.index("(")+1:z.rindex(")")] for z in zeilen),
                    sep=" ", dtype=np.float64).astype(np.int64)
del zeilen
if idx.size != ns.sum(): raise SystemExit(f"faces: {idx.size} Indizes statt {ns.sum()}")
off = np.concatenate(([0], np.cumsum(ns)))[:-1]
print(f"  Eckenzahlen: {dict(zip(*[v.tolist() for v in np.unique(ns, return_counts=True)]))}", flush=True)
Sf = np.zeros((nf, 3)); Cz = np.zeros(nf)
for k in np.unique(ns):                              # je Eckenzahl EIN vektorisierter Block
    m = ns == k
    ids = idx[off[m][:, None] + np.arange(k)]        # (M,k)
    P = pts[ids]                                     # (M,k,3)
    c = P.mean(axis=1)
    a = P - c[:, None, :]; b = np.roll(P, -1, axis=1) - c[:, None, :]
    tri = 0.5 * np.cross(a, b)                       # Fan-Dreiecke um den Punktmittelwert
    Sf[m] = tri.sum(axis=1)
    at = np.linalg.norm(tri, axis=2)
    zc = (P[:, :, 2] + np.roll(P[:, :, 2], -1, axis=1) + c[:, 2:3]) / 3.0
    Cz[m] = (at * zc).sum(axis=1) / np.maximum(at.sum(axis=1), 1e-30)
    del ids, P, a, b, tri, at, zc
del idx, pts
print(f"  |Sf| gesamt {np.linalg.norm(Sf,axis=1).sum():.4f} m2, z-Schwerpunkte {Cz.min():.4f}..{Cz.max():.4f} m", flush=True)

print("  owner (Schwanz) + p ...", flush=True)
own = np.array([int(v) for v in liste_schwanz(os.path.join(FALL, "constant/polyMesh/owner"), nf)], dtype=np.int64)
p = block_werte(os.path.join(FALL, ZEIT, "p"), "internalField")
print(f"  p: {p.size} Zellwerte, {p.min():.1f}..{p.max():.1f} (kinematisch)", flush=True)
pw = p[own]
tau = block_werte(os.path.join(FALL, ZEIT, "wallShearStress"), "vehicle", 3)
if tau.shape[0] != nf: raise SystemExit(f"wallShearStress: {tau.shape[0]} statt {nf}")

A = np.linalg.norm(Sf, axis=1)
F_druck = RHO_INF * (pw[:, None] * Sf)
F_reib = -RHO_INF * (tau * A[:, None])   # OpenFOAMs wallShearStress traegt das Minus schon
                                         # (tau = -(nu_eff*dev2(grad U))&n); ohne dieses Vorzeichen
                                         # kam die Abnahme auf exakt -1x forces.dat (gemessen 27.08.)

soll = None
for zeile in open(os.path.join(FALL, "postProcessing/forces", sorted(os.listdir(os.path.join(FALL,"postProcessing/forces")))[-1], "forces.dat"), errors="replace"):
    if zeile.startswith("#"): continue
    if zeile.split()[0] == ZEIT:
        z = zeile.replace("(", " ").replace(")", " ").split()
        soll = dict(p=(float(z[1]), float(z[2]), float(z[3])), v=(float(z[4]), float(z[5]), float(z[6])))
print("\n=== ABNAHME gegen forces.dat ===")
for name, ist, sl in (("Druck", F_druck.sum(axis=0), soll and soll["p"]), ("Reibung", F_reib.sum(axis=0), soll and soll["v"])):
    if sl is None: print(f"  {name}: kein forces.dat-Eintrag zu t={ZEIT}"); continue
    d = [abs(i - s) / max(abs(s), 1e-9) for i, s in zip(ist, sl)]
    print(f"  {name}  Fx {ist[0]:+11.3f} (Soll {sl[0]:+11.3f}, {100*d[0]:5.2f} %) | "
          f"Fz {ist[2]:+11.3f} (Soll {sl[2]:+11.3f}, {100*d[2]:5.2f} %)")
    if max(d[0], d[2]) > 0.01:
        print("  ACHTUNG: Abweichung > 1 % -- die Bandzerlegung unten ist NICHT belastbar.")

print(f"\n=== HOEHENBANDER (Cd/Cz auf A_ref {A_REF} m2, q_inf {Q_INF:.1f} Pa) ===")
cd = lambda F: F[0] / (Q_INF * A_REF); cz = lambda F: F[2] / (Q_INF * A_REF)
ges_p, ges_v = F_druck.sum(axis=0), F_reib.sum(axis=0)
print(f"  GESAMT      Cd_p {cd(ges_p):+.4f}  Cd_v {cd(ges_v):+.4f}  Cd {cd(ges_p+ges_v):+.4f} | "
      f"Cz_p {cz(ges_p):+.4f}  Cz_v {cz(ges_v):+.4f}  Cz {cz(ges_p+ges_v):+.4f}")
for mm in BAENDER_MM:
    m = Cz < mm / 1000.0
    bp, bv = F_druck[m].sum(axis=0), F_reib[m].sum(axis=0)
    print(f"  z < {mm:5.1f} mm  Flaechen {int(m.sum()):7d} ({100*m.mean():5.2f} %)  Flaeche {A[m].sum():7.4f} m2 ({100*A[m].sum()/A.sum():5.2f} %)")
    print(f"               Cd_p {cd(bp):+.4f} ({100*cd(bp)/cd(ges_p):+6.2f} % von Cd_p)  "
          f"Cz_p {cz(bp):+.4f} ({100*cz(bp)/cz(ges_p):+6.2f} %)  Cd_v {cd(bv):+.4f}")
