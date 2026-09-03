#!/usr/bin/env python3
# diff_of13_yslice.py — Diff-Slice FX-Lauf gegen OpenFOAM-13-Referenz (27.08.2026, Heiko-Auftrag:
# "diff slice zwischen dem heutigen lauf und of13 mit rang +/-15 m/s").
# KONVENTION EXAKT WIE das V1-Bild docs/diff_baseline_vs_of13_500ms.png:
#   dU = |u|_OF13 - |u|_FX auf der Ebene y = 0,025 m; rot = OF13 schneller, blau = OF13
#   langsamer / FX ueberbeschleunigt; Skala +/-15 m/s; schwarz = Fahrzeug (FX-Solid) ODER
#   kein OF13-Datenpunkt in Reichweite (Koerperinneres).
# OF13-Quelle: postProcessing/sampleY0/1200/y0.xy (konvergiertes RANS-Sample, 90.597 Punkte,
# Spalten x y z Ux Uy Uz p). Interpolation numpy-only: 20-mm-Binning (Mittel je Bin),
# 3x Nachbar-Dilatation zum Lueckenfuellen, Zellen ohne erreichbare Daten = maskiert.
# Caveat (wie beim V1-Bild dokumentiert): FX ist ein LES-Momentanbild, OF13 ein RANS-Mittel --
# im Nachlauf zeigen sich aufgeloeste Wirbel gegen ein glattes Mittel; aussagekraeftig sind
# die Mittelstrom-Regionen (Dach, Front, Unterboden).
# Aufruf: diff_of13_yslice.py <feld_nah_XXXms.vtk> [out.png] [of13_xy=Standardpfad]
# Iron Rule 5: das PNG ist Sichtung; die gedruckten RMS/Quantile sind die Messgroessen.
import sys, os, re
import numpy as np
import sys as _sys, os as _os
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import of13_diff_kennzahlen as _kz  # ★ 03.09.: standardisierte Kennzahlen-Ablage (Heiko)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OF13_STD = "/home/heiko/CFD-Cases/mr2v40H/postProcessing/sampleY0/1200/y0.xy"
Y_SOLL = 0.025  # Ebene des OF13-Samples (V1-Konvention "Y = 0.025 m")
XOFF = 2.2063   # x_v2 = x_OF13 + XOFF -- ETABLIERTE Zuordnung aus export/vergleich_of13_2026-08-26/
                # (vergleich_render.py, von Heiko am 26.08. gesichtet); z unveraendert.

def lies_header(f):
    dims=orig=spac=None
    for _ in range(20):
        s=f.readline().decode("ascii","replace").strip()
        if s.startswith("DIMENSIONS"): dims=tuple(int(x) for x in s.split()[1:4])
        elif s.startswith("ORIGIN"): orig=tuple(float(x) for x in s.split()[1:4])
        elif s.startswith("SPACING"): spac=tuple(float(x) for x in s.split()[1:4])
        elif s.startswith("VECTORS"): return dims,orig,spac,f.tell()
    raise SystemExit("VECTORS nicht gefunden")

vtk=sys.argv[1]
of13=sys.argv[3] if len(sys.argv)>3 else OF13_STD
with open(vtk,"rb") as f:
    (Nx,Ny,Nz),orig,spac,off=lies_header(f)
    yq=int(round((Y_SOLL-orig[1])/spac[1]))
    ebene=np.empty((Nz,Nx,3),dtype=np.float32)
    for z in range(Nz):
        f.seek(off+((z*Ny+yq)*Nx)*12)
        ebene[z]=np.frombuffer(f.read(Nx*12),dtype=">f4").reshape(Nx,3)
fx=np.linalg.norm(ebene.astype(np.float64),axis=2)
solid=fx==0.0
x0,z0,dx=orig[0],orig[2],spac[0]
print(f"FX-Ebene: y-Index {yq} (Welt-y {orig[1]+yq*spac[1]:+.3f} m, Soll {Y_SOLL}), {Nx}x{Nz} Zellen")

d=np.loadtxt(of13)
ox,oz,ou=d[:,0]+XOFF,d[:,2],np.linalg.norm(d[:,3:6],axis=1)
B=0.020  # Bin-Kante 20 mm
bnx=int(np.ceil(Nx*dx/B)); bnz=int(np.ceil(Nz*dx/B))
ix=((ox-x0)/B).astype(int); iz=((oz-z0)/B).astype(int)
g=(ix>=0)&(ix<bnx)&(iz>=0)&(iz<bnz)
summe=np.zeros((bnz,bnx)); anz=np.zeros((bnz,bnx))
np.add.at(summe,(iz[g],ix[g]),ou[g]); np.add.at(anz,(iz[g],ix[g]),1.0)
feld=np.where(anz>0,summe/np.maximum(anz,1),np.nan)
print(f"OF13-Binning: {int(g.sum())} Punkte in {bnx}x{bnz} Bins, gefuellt {int((anz>0).sum())} ({100*(anz>0).mean():.0f} %)")
import warnings
for _ in range(10):  # Nachbar-Dilatation (Mittel der gueltigen 3x3-Nachbarn; OF-Sample ist fern vom Koerper duenn)
    p=np.pad(feld,1,constant_values=np.nan)
    st=np.stack([p[a:a+bnz,b:b+bnx] for a in range(3) for b in range(3)])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        m=np.nanmean(st,axis=0)
    feld=np.where(np.isnan(feld),m,feld)
keine_daten_b=np.isnan(feld)
zi=np.minimum(((np.arange(Nz)*dx)/B).astype(int),bnz-1)
xi=np.minimum(((np.arange(Nx)*dx)/B).astype(int),bnx-1)
of=feld[np.ix_(zi,xi)]; keine=keine_daten_b[np.ix_(zi,xi)]
du=of-fx
maske=solid|keine
w=du[~maske]
print(f"dU = |u|_OF13 - |u|_FX: auswertbar {w.size} Zellen | RMS {np.sqrt(np.mean(w**2)):.3f} m/s | "
      f"Mittel {np.mean(w):+.3f} | p05 {np.percentile(w,5):+.2f} | p50 {np.percentile(w,50):+.2f} | "
      f"p95 {np.percentile(w,95):+.2f} | |dU|>15 (Clip): {100*np.mean(np.abs(w)>15):.2f} %")
v=np.clip(du,-15.0,15.0)
img=np.empty((Nz,Nx,3),dtype=np.uint8)
neg=v<0  # blau=OF13 langsamer/FX schneller, rot=OF13 schneller (V1-Konvention)
t_n=np.clip(-v/15.0,0,1); t_p=np.clip(v/15.0,0,1)
c_n=np.rint(255*(1-t_n)).astype(np.uint8); c_p=np.rint(255*(1-t_p)).astype(np.uint8)
img[...,0]=np.where(neg,c_n,255); img[...,1]=np.where(neg,c_n,c_p); img[...,2]=np.where(neg,255,c_p)
img[maske]=(0,0,0)
img=img[::-1]
m=re.search(r"(\d+)ms",os.path.basename(vtk)); ms=(m.group(1) if m else "0").zfill(6)
out=sys.argv[2] if len(sys.argv)>2 else os.path.join(os.path.dirname(os.path.abspath(vtk)),f"diff_of13_{ms}ms.png")
plt.imsave(out,img)
print(f"geschrieben: {out}")

# ★ 03.09.2026: standardisierte Kennzahlen-Ablage (Heiko: "bekommst das standardisiert ausgegeben").
# Bewusst AM ENDE -- out wird erst bei der Bildausgabe gesetzt; w ist das oben maskierte dU.
_kz.anhaengen(w, "y", float(Y_SOLL), vtk, out)
