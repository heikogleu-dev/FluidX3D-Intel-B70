#!/usr/bin/env python3
# §8.3 REKONSTRUKTION-PLAN: wieviele FACETTENZELLEN ueberschreibt boden_eq?
# Daten: export/p4_regel7 (4 mm, 21.09.2026). Nur lesend, keine GPU.
import numpy as np, os
Nx,Ny,Nz = 1917,693,493; N = Nx*Ny*Nz
import sys
NZ, NZ_DOWN, X_SPLIT, ABSTAND = 2, 1, 113, int(sys.argv[1]) if len(sys.argv)>1 else 2     # aus logs/p4_regel7.log: "z=1..2, ab Nase DOWN=1 (split-Voxel 113), ABSTAND=2"
VTK = "export/p4_regel7/feld_nah_001001ms.vtk"
TYPE_S, TYPE_BO = 0x01, 0x03

n = np.loadtxt("/tmp/claude-1000/-home-heiko-CFD-FluidX3D/82099a59-c17e-437f-9dfe-9247e20f655b/scratchpad/facn.txt", dtype=np.int64)
print(f"Facettenzellen gesamt            : {len(n):>12,}")
z = n // (Nx*Ny); rest = n % (Nx*Ny); y = rest // Nx; x = rest % Nx
nz_eff = np.where(x >= X_SPLIT, NZ_DOWN, NZ)
kand = (nz_eff > 0) & (z >= 1) & (z <= nz_eff)
print(f"davon in der boden_eq-Zone z<=nz_eff: {kand.sum():>12,}  ({100.0*kand.sum()/len(n):.4f} %)")
if kand.sum() == 0:
    print("\nERGEBNIS: boden_eq und die Facettenmenge sind an diesem Gitter DISJUNKT."); raise SystemExit

flags = np.memmap(VTK, dtype=np.uint8, mode="r", offset=os.path.getsize(VTK)-N, shape=(N,))
kx, ky, kz, kn = x[kand], y[kand], z[kand], n[kand]
# boden_eq steigt selbst aus, wenn die Zelle SOLID oder TYPE_E ist
bo = flags[kn] & TYPE_BO
lebend = (bo != 0x01) & (bo != 0x02)
print(f"davon weder TYPE_S noch TYPE_E    : {lebend.sum():>12,}")
kx, ky, kz, kn = kx[lebend], ky[lebend], kz[lebend], kn[lebend]
# ABSTAND-Aussparung: Solid in dz in [0,a], dx/dy in [-a,a]  (Fahrbahn z=0 zaehlt bewusst NICHT)
a = ABSTAND; nah = np.zeros(len(kn), dtype=bool)
for dz in range(0, a+1):
    for dy in range(-a, a+1):
        for dx in range(-a, a+1):
            xx, yy, zz = kx+dx, ky+dy, kz+dz
            ok = (xx>=0)&(yy>=0)&(xx<Nx)&(yy<Ny)&(zz<Nz)
            idx = (xx + (yy + zz*Ny)*Nx)
            tref = np.zeros(len(kn), dtype=bool)
            tref[ok] = (flags[idx[ok]] & TYPE_BO) == TYPE_S
            nah |= tref
print(f"davon durch ABSTAND={a} ausgespart  : {nah.sum():>12,}  ({100.0*nah.sum()/max(1,len(kn)):.2f} %)")
treffer = (~nah).sum()
print(f"\nERGEBNIS -- Facettenzellen, die boden_eq WIRKLICH ueberschreibt: {treffer:,}")
print(f"  = {100.0*treffer/len(n):.6f} % aller Facettenzellen")
if treffer:
    zz = kz[~nah]
    print(f"  z-Verteilung: " + ", ".join(f"z={v}: {c:,}" for v,c in zip(*np.unique(zz, return_counts=True))))
    print(f"  x-Spanne: {kx[~nah].min()} .. {kx[~nah].max()} (X_SPLIT={X_SPLIT})")
