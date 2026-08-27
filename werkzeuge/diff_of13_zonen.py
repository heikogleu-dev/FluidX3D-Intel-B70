#!/usr/bin/env python3
# diff_of13_zonen.py -- Zonen-Zerlegung des OF13-Diffs (27.08.2026, Pruefung der CC-Auftraege).
# Beantwortet die Falsifikationsfrage "Bias gleichverteilt oder konzentriert?" mit Zahlen
# statt am Bild (Iron Rule 5). Konvention identisch zu diff_of13_yslice.py:
#   dU = |u|_OF13 - |u|_FX auf y = 0,025 m, XOFF 2,2063, OF13-Binning 20 mm + 10x Dilatation.
# Aufruf: diff_of13_zonen.py <feld_XXX.vtk> [of13_xy]
import sys, os, warnings
import numpy as np

OF13_STD = "/home/heiko/CFD-Cases/mr2v40H/postProcessing/sampleY0/1200/y0.xy"
Y_SOLL, XOFF = 0.025, 2.2063

def lies_header(f):
    for _ in range(20):
        s = f.readline().decode("ascii", "replace").strip()
        if s.startswith("DIMENSIONS"): dims = tuple(int(v) for v in s.split()[1:4])
        elif s.startswith("ORIGIN"):   orig = tuple(float(v) for v in s.split()[1:4])
        elif s.startswith("SPACING"):  spac = tuple(float(v) for v in s.split()[1:4])
        elif s.startswith("VECTORS"):  return dims, orig, spac, f.tell()
    raise SystemExit("VECTORS nicht gefunden")

vtk  = sys.argv[1]
of13 = sys.argv[2] if len(sys.argv) > 2 else OF13_STD
with open(vtk, "rb") as f:
    (Nx, Ny, Nz), orig, spac, off = lies_header(f)
    yq = int(round((Y_SOLL - orig[1]) / spac[1]))
    ebene = np.empty((Nz, Nx, 3), dtype=np.float32)
    for z in range(Nz):
        f.seek(off + ((z * Ny + yq) * Nx) * 12)
        ebene[z] = np.frombuffer(f.read(Nx * 12), dtype=">f4").reshape(Nx, 3)
fx = np.linalg.norm(ebene.astype(np.float64), axis=2)
solid = fx == 0.0
x0, z0, dx = orig[0], orig[2], spac[0]
print(f"FX-Ebene {Nx}x{Nz} @ {dx*1000:.0f} mm, y-Index {yq} (Welt-y {orig[1]+yq*spac[1]:+.3f} m)")

d = np.loadtxt(of13)
ox, oz, ou = d[:, 0] + XOFF, d[:, 2], np.linalg.norm(d[:, 3:6], axis=1)
B = 0.020
bnx, bnz = int(np.ceil(Nx * dx / B)), int(np.ceil(Nz * dx / B))
ix, iz = ((ox - x0) / B).astype(int), ((oz - z0) / B).astype(int)
g = (ix >= 0) & (ix < bnx) & (iz >= 0) & (iz < bnz)
summe = np.zeros((bnz, bnx)); anz = np.zeros((bnz, bnx))
np.add.at(summe, (iz[g], ix[g]), ou[g]); np.add.at(anz, (iz[g], ix[g]), 1.0)
feld = np.where(anz > 0, summe / np.maximum(anz, 1), np.nan)
for _ in range(10):
    p = np.pad(feld, 1, constant_values=np.nan)
    st = np.stack([p[a:a+bnz, b:b+bnx] for a in range(3) for b in range(3)])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore"); m = np.nanmean(st, axis=0)
    feld = np.where(np.isnan(feld), m, feld)
zi = np.minimum(((np.arange(Nz) * dx) / B).astype(int), bnz - 1)
xi = np.minimum(((np.arange(Nx) * dx) / B).astype(int), bnx - 1)
of = feld[np.ix_(zi, xi)]
maske = solid | np.isnan(of)
du = of - fx

# Wandabstand IN DER EBENE (Chebyshev, Zellen) durch iterative Dilatation der Solidmaske.
dist = np.full(solid.shape, 999, dtype=np.int32); cur = solid.copy(); dist[cur] = 0
kmax = max(4, int(round(0.100 / dx)) + 1)
for k in range(1, kmax + 1):
    p = np.pad(cur, 1); nb = np.zeros_like(cur)
    for a in range(3):
        for b in range(3): nb |= p[a:a+Nz, b:b+Nx]
    neu = nb & ~cur; dist[neu & (dist == 999)] = k; cur = nb

X = (x0 + np.arange(Nx) * dx)[None, :].repeat(Nz, 0)
Z = (z0 + np.arange(Nz) * dx)[:, None].repeat(Nx, 1)

def rep(name, m):
    v = du[m & ~maske]
    if v.size < 50:
        print(f"{name:34s} n={v.size:8d}  (zu wenig)"); return
    print(f"{name:34s} n={v.size:8d} Mittel {v.mean():+6.2f} p50 {np.median(v):+6.2f} "
          f"RMS {np.sqrt(np.mean(v**2)):5.2f} | OF13 {of[m & ~maske].mean():5.1f} FX {fx[m & ~maske].mean():5.1f} m/s")

w = du[~maske]
print(f"GESAMT: n={w.size} RMS {np.sqrt(np.mean(w**2)):.3f} Mittel {w.mean():+.3f} p50 {np.median(w):+.2f} m/s")
n32, n100 = max(1, int(round(0.032/dx))), max(2, int(round(0.100/dx)))
print("--- nach Wandabstand in der Ebene:")
rep(f"Wand <={n32} Zellen (~32 mm)", (dist >= 1) & (dist <= n32))
rep(f"Wand {n32+1}..{n100} Zellen (~100 mm)", (dist > n32) & (dist <= n100))
rep(f"fern >{n100} Zellen (>100 mm)", dist > n100)
print("--- nach x-Zone x z-Zone (Fahrzeug x 0..4,44 m):")
xz = [("vor Fahrzeug x<0", X < 0), ("Front 0..1,5", (X >= 0) & (X < 1.5)),
      ("Mitte 1,5..3,0", (X >= 1.5) & (X < 3.0)), ("Heck 3,0..4,44", (X >= 3.0) & (X < 4.44)),
      ("Nachlauf x>4,44", X >= 4.44)]
zz = [("z<0,15 Unterboden", Z < 0.15), ("z 0,15..1,3 Aufbau", (Z >= 0.15) & (Z < 1.3)),
      ("z 1,3..1,94 Deckelzone", (Z >= 1.3) & (Z < 1.94)), ("z>1,94 darueber", Z >= 1.94)]
for xn, xm in xz:
    rep(xn, xm)
    for zn, zm in zz: rep("   " + zn, xm & zm)
print("--- wandfern (>100 mm) je x-Zone -- der Ueberbeschleunigungsverlauf:")
for xn, xm in xz: rep("   fern " + xn, xm & (dist > n100))
