#!/usr/bin/env python3
# diff_of13_zslice.py — Diff-Slice FX gegen OpenFOAM 13 auf einer WAAGERECHTEN Ebene (z = const).
# Heiko-Auftrag 29.08.2026 (Freigabeliste D4): "wuerde eher nochmal den z slice im vergleich zu
# of13 anschauen ... wenn dort kein abdruck sichtbar ist, ist das meines achtens nicht berechtigt".
#
# ANLASS: die Nahfeldbox praegt auf fuenf Seiten rho hart aus dem 16-mm-Gitter auf
# (kernel.cpp apply_kopplung_drive); die Rueckkopplung korrigiert nur u, nie rho. Mit der neuen
# Box (CFD_NEAR_LZ 1,8560) sitzt die Decke bei z = 1,856 m, also 0,656 m ueber dem Dachscheitel.
# Zwei Audit-Agenten hielten das fuer zu eng. Die Frage ist mit Daten entscheidbar:
# hinterlaesst die Decke einen ABDRUCK in der Stroemung, oder nicht?
#
# WARUM GENAU DIESE EBENE: die OF13-Referenz hat einen fertigen z-Schnitt bei z = 1,836 m
# (postProcessing/sampleVergleich2608/1200/zp10.xy, 10.092 Punkte) -- das sind 20 mm UNTER
# unserer Deckenebene. Besser koennte die Probe nicht liegen.
#
# KONVENTION WORTGLEICH zu diff_of13_yslice.py (damit beide Bilder vergleichbar sind):
#   dU = |u|_OF13 - |u|_FX; rot = OF13 schneller, blau = OF13 langsamer / FX ueberbeschleunigt;
#   Skala +/-15 m/s geklemmt; schwarz = Solid ODER kein OF13-Datenpunkt in Reichweite.
#   x_v2 = x_OF13 + XOFF mit XOFF = 2,2063 (etablierte Zuordnung, von Heiko am 26.08. gesichtet).
# Bildzeile 0 oben: hier ist die Bildachse y (nicht z), y waechst nach unten wie im y-Schnitt z.
#
# CAVEAT wie beim y-Schnitt: FX ist ein LES-Momentanbild, OF13 ein RANS-Mittel. Aussagekraeftig
# ist die grossraeumige Struktur, nicht das einzelne Wirbelpaar. Fuer die Deckenfrage genuegt das:
# ein Randabdruck waere ein FLAECHIGES, ortsfestes Muster, kein Wirbel.
#
# Aufruf: diff_of13_zslice.py <feld_nah_XXXXXXms.vtk> [out.png] [of13.xy]
import sys, os, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OF13_STD = "/home/heiko/CFD-Cases/mr2v40H/postProcessing/sampleVergleich2608/1200/zp10.xy"
XOFF = 2.2063

def lies_header(f):
    dims = orig = spac = None
    for _ in range(20):
        s = f.readline().decode("ascii", "replace").strip()
        if s.startswith("DIMENSIONS"): dims = tuple(int(x) for x in s.split()[1:4])
        elif s.startswith("ORIGIN"):   orig = tuple(float(x) for x in s.split()[1:4])
        elif s.startswith("SPACING"):  spac = tuple(float(x) for x in s.split()[1:4])
        elif s.startswith("VECTORS"):  return dims, orig, spac, f.tell()
    raise SystemExit("VECTORS nicht gefunden")

vtk  = sys.argv[1]
of13 = sys.argv[3] if len(sys.argv) > 3 else OF13_STD

# --- OF13 zuerst lesen: die Ebene des Samples bestimmt, welche FX-Ebene geschnitten wird.
d = np.loadtxt(of13)
z_of = float(np.median(d[:, 2]))
if np.ptp(d[:, 2]) > 1e-6:
    print(f"WARNUNG: das OF13-Sample ist nicht eben (z-Spanne {np.ptp(d[:,2]):.4f} m) -- Median {z_of:.4f} m verwendet.")

with open(vtk, "rb") as f:
    (Nx, Ny, Nz), orig, spac, off = lies_header(f)
    zq = int(round((z_of - orig[2]) / spac[2]))
    if not (0 <= zq < Nz):
        raise SystemExit(f"OF13-Ebene z = {z_of:.4f} m liegt ausserhalb der FX-Domaene "
                         f"z[{orig[2]:.3f},{orig[2]+(Nz-1)*spac[2]:.3f}] -- nichts zu vergleichen.")
    # Eine z-Ebene liegt im STRUCTURED_POINTS zusammenhaengend: Offset z*Ny*Nx, Laenge Ny*Nx.
    f.seek(off + (zq * Ny * Nx) * 12)
    ebene = np.frombuffer(f.read(Nx * Ny * 12), dtype=">f4").reshape(Ny, Nx, 3)

fx = np.linalg.norm(ebene.astype(np.float64), axis=2)   # (Ny, Nx)
solid = fx == 0.0
x0, y0, dx = orig[0], orig[1], spac[0]
z_fx = orig[2] + zq * spac[2]
print(f"FX-Ebene: z-Index {zq} (Welt-z {z_fx:+.4f} m), OF13-Ebene z = {z_of:+.4f} m, "
      f"Versatz {1000*(z_fx-z_of):+.1f} mm | {Nx}x{Ny} Zellen")
print(f"FX-Domaenendecke liegt bei z = {orig[2]+(Nz-1)*spac[2]:.4f} m, also "
      f"{1000*(orig[2]+(Nz-1)*spac[2]-z_fx):.0f} mm ueber dieser Ebene.")

ox, oy, ou = d[:, 0] + XOFF, d[:, 1], np.linalg.norm(d[:, 3:6], axis=1)
B = 0.020
bnx = int(np.ceil(Nx * dx / B)); bny = int(np.ceil(Ny * dx / B))
ix = ((ox - x0) / B).astype(int); iy = ((oy - y0) / B).astype(int)
g = (ix >= 0) & (ix < bnx) & (iy >= 0) & (iy < bny)
summe = np.zeros((bny, bnx)); anz = np.zeros((bny, bnx))
np.add.at(summe, (iy[g], ix[g]), ou[g]); np.add.at(anz, (iy[g], ix[g]), 1.0)
feld = np.where(anz > 0, summe / np.maximum(anz, 1), np.nan)
print(f"OF13-Binning: {int(g.sum())} Punkte in {bnx}x{bny} Bins, gefuellt {int((anz>0).sum())} "
      f"({100*(anz>0).mean():.0f} %)")

import warnings
for _ in range(10):
    p = np.pad(feld, 1, constant_values=np.nan)
    st = np.stack([p[a:a+bny, b:b+bnx] for a in range(3) for b in range(3)])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        m = np.nanmean(st, axis=0)
    feld = np.where(np.isnan(feld), m, feld)
keine_b = np.isnan(feld)
yi = np.minimum(((np.arange(Ny) * dx) / B).astype(int), bny - 1)
xi = np.minimum(((np.arange(Nx) * dx) / B).astype(int), bnx - 1)
of = feld[np.ix_(yi, xi)]; keine = keine_b[np.ix_(yi, xi)]
du = of - fx
maske = solid | keine
w = du[~maske]
print(f"dU = |u|_OF13 - |u|_FX: auswertbar {w.size} Zellen | RMS {np.sqrt(np.mean(w**2)):.3f} m/s | "
      f"Mittel {np.mean(w):+.3f} | p05 {np.percentile(w,5):+.2f} | p50 {np.percentile(w,50):+.2f} | "
      f"p95 {np.percentile(w,95):+.2f} | |dU|>15 (Clip): {100*np.mean(np.abs(w)>15):.2f} %")

# ★ RANDABDRUCK-TEST, der eigentliche Zweck: waere die Decke schuld, muesste dU zu den
# SEITENraendern der Nahfeldbox hin systematisch anwachsen. Deshalb dU ueber den Abstand
# zum naechsten y-Rand gemittelt -- ein Abdruck erscheint als monotone Flanke, Turbulenz nicht.
print("\nRANDABDRUCK-TEST (Mittel |dU| ueber den Abstand zum naechsten y-Rand der Nahfeldbox):")
absty = np.minimum(np.arange(Ny), Ny - 1 - np.arange(Ny))[:, None] * dx
for lo, hi in [(0.0, 0.05), (0.05, 0.10), (0.10, 0.20), (0.20, 0.40), (0.40, 0.80), (0.80, 99.0)]:
    sel = (~maske) & (absty >= lo) & (absty < hi)
    if sel.sum() > 0:
        print(f"  {lo*1000:5.0f}..{hi*1000:5.0f} mm vom Rand: n={sel.sum():7d}  "
              f"Mittel dU {du[sel].mean():+7.3f}  |dU| {np.abs(du[sel]).mean():6.3f} m/s")

v = np.clip(du, -15.0, 15.0)
img = np.empty((Ny, Nx, 3), dtype=np.uint8)
neg = v < 0
t_n = np.clip(-v/15.0, 0, 1); t_p = np.clip(v/15.0, 0, 1)
c_n = np.rint(255*(1-t_n)).astype(np.uint8); c_p = np.rint(255*(1-t_p)).astype(np.uint8)
img[..., 0] = np.where(neg, c_n, 255); img[..., 1] = np.where(neg, c_n, c_p); img[..., 2] = np.where(neg, 255, c_p)
img[maske] = (0, 0, 0)
img = img[::-1]
m = re.search(r"(\d+)ms", os.path.basename(vtk)); ms = (m.group(1) if m else "0").zfill(6)
out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.abspath(vtk)),
                                                         f"diff_of13_z_{ms}ms.png")
plt.imsave(out, img)
print(f"\ngeschrieben: {out}")
