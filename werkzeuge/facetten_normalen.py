#!/usr/bin/env python3
"""facetten_normalen.py -- Facettennormalen eines Laufs aus den VTK-Flags nachbauen (17.09.2026).

HERKUNFT: Pruefagent zum D3Q27-Zensus (17.09.), Skript p2_normalen.py, ins Repo uebernommen. Nachgebaut ist baue_facetten
mit CFD_FACETTEN_NORMQUELLE=1 (V3b): 3^3-Fenster, Kipp gegen die naechste Achsflaeche und gegen den PCA-Schwerpunkt,
ein Glaettungspass (Gewicht max(1, eigene_links), Nachbarn ohne K1). Anlass: die Normalen sind nicht exportiert
(CFD_FAC_ZENSUS_VTK=0), und die Annahme "generische Normale" war fuer die D3Q27-Ein-Link-Klasse systematisch falsch
(V3b-Normalen haben exakte Nullkomponenten: n_x 35 %, n_y 42 %, n_z 18 % der aktiven Facetten bei p375_e).

EINGEBAUTE ABNAHMEN (Abbruch bei Abweichung): eigene_links und n_punkte == CSV in ALLEN Zeilen; K1 == CSV-Bit 1;
K2 (Kohaerenz < 1/sqrt3) == CSV-Bit 2; Kipps gegen PCA-Schwerpunkt und Duennteil-Rueckfall == Log ("Vorzeichen gekippt");
K2-Zahl == Log; winkel_grad gegen CSV max |d| < 1e-3 Grad; achse == CSV zu 100 %.
Der exakte Gegenbeweis steckt in d3q27_zensus.py: der Host-Klassifikator mit diesen Normalen muss den statischen
Zensus des Laufs (Rang, Entkopplung, Wanderung) EXAKT treffen.
Ausgabe: export/<lauf>/facetten_normalen.npz (n, nf = Normale float32, K1, klasse).
Aufruf: facetten_normalen.py <export/lauf> [t_ms des VTK fuer die Flags, Vorgabe 000501]
"""
import sys, os, time, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from zonen_kraft import kopf, protokoll
import d3q27_zensus as Z
import re

lauf = sys.argv[1]
tms = sys.argv[2] if len(sys.argv) > 2 else "000501"
vtk = os.path.join(lauf, f"feld_nah_{tms}ms.vtk")
d = kopf(vtk); Nx, Ny, Nz = d["dims"]; NXY = Nx*Ny
t0 = time.time()
FL = np.fromfile(vtk, dtype=np.uint8, count=Nx*Ny*Nz, offset=d["off_flags"])
C, sp = Z.lies_csv(os.path.join(lauf, "facetten_histogramme.csv"))
n = C["n"].astype(np.int64); N = n.size
kl = C["klasse"].astype(np.int64); el = C["eigene_links"].astype(np.int64); npk = C["n_punkte"].astype(np.int64)
if not np.all(np.diff(n) > 0): raise SystemExit("CSV nicht aufsteigend nach n")
x = n % Nx; y = (n // Nx) % Ny; z = n // NXY
print(f"{N} Facetten, CSV gelesen {time.time()-t0:.0f} s")

FZ = np.array([(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1),(1,1,0),(-1,-1,0),(1,0,1),(-1,0,-1),(0,1,1),(0,-1,-1),(1,-1,0),(-1,1,0),(1,0,-1),(-1,0,1),(0,1,-1),(0,-1,1)], np.int64)
W = np.array([(dx, dy, dz) for dz in (-1,0,1) for dy in (-1,0,1) for dx in (-1,0,1)], np.int64)   # Schleifenordnung dz,dy,dx
O5 = np.array([(a, b, c) for c in range(-2,3) for b in range(-2,3) for a in range(-2,3)], np.int64)
def i5(v): return (v[..., 0]+2) + 5*((v[..., 1]+2) + 5*(v[..., 2]+2))
WC = W[:, None, :] + FZ[None, :, :]            # 27x18x3
IDX_WC = i5(WC)                                # in O5
P = W[:, None, :] + 0.5*FZ[None, :, :]         # Mittelpunkt relativ zur Zelle
off5 = O5[:, 0] + Nx*(O5[:, 1] + Ny*O5[:, 2])
offW = W[:, 0] + Nx*(W[:, 1] + Ny*W[:, 2])

nf = np.zeros((N, 3), np.float32); K1 = np.zeros(N, bool); eig = np.zeros(N, np.int64); npc = np.zeros(N, np.int64)
koh = np.zeros(N); kipp_pca = 0; rueck = 0
CH = 40000
for s in range(0, N, CH):
    e = min(N, s+CH); nn = n[s:e]; zz = z[s:e]
    wall = FL[nn[:, None] + off5[None, :]] == 0x41                 # z-Gueltigkeit unten separat (z>=1 -> z-2 >= -1!)
    z5 = zz[:, None] + O5[None, :, 2]
    wall &= (z5 >= 0) & (z5 < Nz)
    zW = zz[:, None] + W[None, :, 2]
    fw_ = FL[nn[:, None] + offW[None, :]]; fluid = ((fw_ == 0) | (fw_ == 3)) & (zW >= 1) & (zW <= Nz-2)
    L = fluid[:, :, None] & wall[:, IDX_WC]                          # (m,27,18); zn0-Gueltigkeit steckt in wall
    npz_ = L.sum(axis=(1, 2)); npc[s:e] = npz_
    eig[s:e] = L[:, 13, :].sum(axis=1)
    La = L[:, :, :6]
    nq = La.sum(axis=(1, 2))
    ns = -(La[:, :, :, None] * FZ[None, None, :6, :]).sum(axis=(1, 2)).astype(np.float64)
    l6 = np.sqrt((ns**2).sum(axis=1))
    dd = np.where(La, (P[None, :, :6, :]**2).sum(axis=3), np.inf).reshape(len(nn), -1)
    jmin = np.argmin(dd, axis=1)                                    # erster minimaler Eintrag in Schleifenordnung
    wmin, imin = jmin // 6, jmin % 6
    c6rel = P[wmin, imin]                                           # q - Zelle
    n6 = np.where(l6[:, None] < 1e-12, -FZ[imin].astype(np.float64), ns/np.where(l6 > 0, l6, 1)[:, None])
    rueck += int(((l6 < 1e-12) & (nq > 0) & (npz_ >= 6)).sum())
    yw6 = (n6 * (-c6rel)).sum(axis=1)
    n6 = np.where(yw6[:, None] < 0, -n6, n6)
    # PCA-Schwerpunkt absolut wie C++: cx = Sum(abs)/np
    Srel = (L[:, :, :, None] * P[None, :, :, :]).sum(axis=(1, 2))
    cell = np.stack([x[s:e], y[s:e], zz], axis=1).astype(np.float64)
    npf = np.maximum(npz_, 1).astype(np.float64)
    cen = (npz_[:, None]*cell + Srel) / npf[:, None]
    dv = cell - cen
    ywp = n6[:, 0]*dv[:, 0] + n6[:, 1]*dv[:, 1] + n6[:, 2]*dv[:, 2]
    k1 = (npz_ < 6) | (nq == 0)
    kp = (ywp < 0) & ~k1
    kipp_pca += int(kp.sum())
    e6 = np.where(kp[:, None], -n6, n6)
    e6[k1] = 0.0
    nf[s:e] = e6.astype(np.float32); K1[s:e] = k1
    koh[s:e] = np.where(nq > 0, l6/np.maximum(nq, 1), 0.0)
    if s % (CH*25) == 0: print(f"  {e}/{N}  {time.time()-t0:.0f} s", flush=True)
print(f"Zellschleife fertig {time.time()-t0:.0f} s")
L = open(protokoll(lauf)["log"], errors="replace").read()
L = re.sub(r"\x1b\[[0-9;]*m", "", L); L = re.sub(r"\|\s*\n\|\s*", " ", L); L = re.sub(r"\s+", " ", L)
mk = re.search(r"Vorzeichen gekippt (\d+) \(Duennteil-Rueckfall (\d+)\)", L); mk2 = re.search(r"K2\(Kante\) (\d+)", L)
def pruef(name, ok, info=""):
    print(f"Check {name}: {'ok' if ok else 'ABWEICHUNG'} {info}")
    if not ok: raise SystemExit(f"FEHLER: {name}")
pruef("eigene_links == CSV (alle Zeilen)", np.array_equal(eig, el), int((eig != el).sum()))
pruef("n_punkte == CSV (alle Zeilen)", np.array_equal(npc, npk), int((npc != npk).sum()))
pruef("K1 == CSV-Bit 1", np.array_equal(K1, (kl & 1) > 0))
pruef("K2 (Kohaerenz < 1/sqrt3) == CSV-Bit 2", np.array_equal(koh < 0.5773502692, (kl & 2) > 0), int((koh < 0.5773502692).sum()))
pruef("K2-Zahl == Log", mk2 is not None and int((koh < 0.5773502692).sum()) == int(mk2.group(1)), f"Log {mk2.group(1) if mk2 else 'FEHLT'}")
pruef("Kipps gegen PCA-Schwerpunkt == Log", mk is not None and kipp_pca == int(mk.group(1)), f"{kipp_pca} / Log {mk.group(1) if mk else 'FEHLT'}")
pruef("Duennteil-Rueckfall == Log", mk is not None and rueck == int(mk.group(2)), f"{rueck} / Log {mk.group(2) if mk else 'FEHLT'}")

# Glaettung: 1 Pass, w = max(1, eigene_links), 3^3-Nachbarfacetten (nicht K1)
G = nf.copy()
wgt = np.maximum(1, el).astype(np.float64)
for s in range(0, N, 400000):
    e = min(N, s+400000); nn = n[s:e]
    S = np.zeros((e-s, 3))
    for k in range(27):
        q = nn + offW[k]
        j = np.searchsorted(n, q); j = np.minimum(j, N-1)
        ok = (n[j] == q) & ~K1[j]
        S += np.where(ok[:, None], wgt[j, None]*nf[j].astype(np.float64), 0.0)
    l = np.sqrt((S**2).sum(axis=1))
    upd = (l > 1e-12) & ~K1[s:e]
    G[s:e][upd] = (S[upd]/l[upd, None]).astype(np.float32)
amax = np.max(np.abs(G), axis=1)
w_rek = np.degrees(np.arccos(np.minimum(1.0, amax.astype(np.float32)).astype(np.float64)))
dw = np.abs(w_rek - C["winkel_grad"])
ach = np.argmax(np.abs(G), axis=1)   # Tie-Break kleinste Achse wie C++
print(f"Glaettung fertig {time.time()-t0:.0f} s")
pruef("winkel_grad gegen CSV (6 Stellen)", dw.max() < 1e-3, f"max |d| = {dw.max():.2e} Grad")
pruef("achse == CSV", bool(np.all(ach == C["achse"].astype(np.int64))), f"{float((ach == C['achse'].astype(np.int64)).mean()):.6f}")
aus = os.path.join(lauf, "facetten_normalen.npz")
np.savez_compressed(aus, n=n, nf=G, K1=K1, klasse=kl)
print("geschrieben:", aus)
