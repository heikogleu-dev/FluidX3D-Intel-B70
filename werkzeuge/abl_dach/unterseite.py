#!/usr/bin/env python3
"""unterseite.py -- Unterboden/Diffusor in der Mittelebene: FX-Baender (dach_band_<t>ms.npz aus fx_band.py) gegen OF13 (of13_slab.npy).
17.09.2026, erste Fassung UNGEPRUEFT; ★ 17.09.2026 selbst geprueft und berichtigt (Iron Rule 2: Diff-Pruefung steht aus).

FX: je (y-Ebene, x-Spalte) von z=1 aufwaerts die erste Solidzelle ((flags&3)==1) mit mindestens einer Fluidzelle darunter
= Unterseite; erste Fluidzelle darunter (Wandabstand 0,5 dx) liefert u_x; dazu u_x k8/k16 Zellen tiefer (k = round(8 bzw. 16 mm/dx),
WANDABSTAND (k + 0,5) dx -- wird je Serie im Kopf genannt, bei 4 mm 10/18 mm), sofern noch Fluid. Spalten, in denen Solid direkt auf
der Strasse steht (Kontakt), entfallen.
Hoehen z_w sind KOERPERHOEHEN = Welt-z - dk*dx (★ berichtigt: vorher Welthoehen; der Upstream-Bodenspalt hebt den Koerper um dk*dx,
dk aus lauf_meta.bodenspalt des Laufordners der npz, im Kopf angesagt; dk unbekannt -> WARNUNG und Welthoehe).
OF13: Zellen |y| < 0,03 m (of13_slab.npy: x y z p Ux Uy Uz, OF13-Rahmen, x_v2 = x_OF13 + 2,2063); je 20-mm-x-Abschnitt die
unterste Luecke > 12 mm in den sortierten Zell-z unterhalb 0,6 m = Unterseite; s = Abstand unter der wandnaechsten Zellmitte;
Zellen mit s in [0,4], [6,10], [14,18] mm liefern u_x-Mittel (Wandabstand = s + halbe Hoehe der ersten Zelle, Prismenlagen < 1 mm).
★ BERICHTIGT (Selbstpruefung 17.09.):
  1. neg (Rueckstromanteil) nahm "s <= 4 mm" OHNE s >= 0 -- damit ALLE Zellen OBERHALB der Luecke (Hohlraum im Fahrzeug, z. B. x 0,70:
     z 0,545 m, u_x ~ 1 m/s, teils negativ). Deshalb neg 0,58 bei u1 +25 (x 0,7-0,8) und 0,48-0,72 bei u1 +20 (x 2,9-3,3). Jetzt dasselbe
     Fenster wie u1: 0 <= s <= 4 mm -> neg 0,00 in allen Abschnitten mit Unterseite.
  2. Abschnitte ohne Zellen oberhalb der Unterseite bis 0,6 m fanden KEINE Luecke und fielen still weg (x 0,74-2,90: OF13-Spalten leer).
     Jetzt zaehlt die Obergrenze 0,6 m als Luecke (Zellstapel endet unter 0,588 m = Unterseite).
  3. Hinter dem Heck (x >= 4,38) fand die Lueckensuche Abstaende zwischen groben Fluidzellen (2-4 Zellen mit 0 <= s <= 4 mm je
     20-mm-Fenster statt >= 176 an einer echten Unterseite). Jetzt gilt eine Unterseite nur mit >= OF_MIN_WANDZELLEN Zellen in 0 <= s <= 4 mm (gemessen: echte
     Unterseiten 176-400, Fehltreffer 2-4).
  Grenze der Methode (bleibt): zu = hoechste Unterseite im 20-mm-Fenster; an schraegen Flaechen (Diffusor 3,6-4,4 m, Steigung bis ~0,3)
  liegt die Wand am vorderen Fensterende bis ~6 mm tiefer -- die s-Fenster sind dort entsprechend unscharf.
Aufruf: unterseite.py name=export/<lauf>/dach_band_<t>ms.npz [name=...]   (x-Abschnitte 100 mm, x_v2 0,2..4,45 m)
"""
import sys, os
import numpy as np
SP = os.path.dirname(os.path.abspath(__file__)); XOFF = 2.2063
sys.path.insert(0, os.path.join(SP, ".."))
import lauf_meta
X0, X1, DXB = 0.2, 4.45, 0.10
kanten = np.arange(X0, X1 + 1e-9, DXB)
OF_ZMAX, OF_LUECKE = 0.6, 0.012     # Suchraum unter 0,6 m; Luecke > 12 mm = Koerper (Zellabstand unter dem Fahrzeug < 12 mm)
OF_MIN_WANDZELLEN = 50              # Zellen in 0 <= s <= 4 mm: echte Unterseite 176-400 je 20-mm-Fenster, Fehltreffer im Nachlauf 2-4 (17.09. gemessen)

def fx(npz):
    d = np.load(npz); U, FL = d["u"], d["flags"]; Nz, ny, Nx = FL.shape
    ox, oy, oz = [float(v) for v in d["orig"]]; dx = float(d["dx"])
    dk, dk_q = lauf_meta.bodenspalt(lauf_meta.lauf_dir_aus(npz))
    if dk is None:
        print(f"WARNUNG {npz}: Bodenspalt dk {dk_q} -- z_w bleibt WELThoehe (nicht lagegleich mit OF13)", file=sys.stderr); dk = 0
    oz = oz - dk*dx                     # Koerper-z = Welt-z - dk*dx
    solid = (FL & 3) == 1
    ux1 = np.full((ny, Nx), np.nan); ux8 = np.full((ny, Nx), np.nan); ux16 = np.full((ny, Nx), np.nan); zw = np.full((ny, Nx), np.nan)
    k8, k16 = int(round(0.008/dx)), int(round(0.016/dx))
    for j in range(ny):
        for i in range(Nx):
            col = solid[1:, j, i]                       # ab z=1
            if not col.any(): continue
            ks = int(np.argmax(col)) + 1                 # erste Solidzelle ab z=1
            if ks < 2: continue                          # Kontakt: Solid direkt auf der Strasse
            zw[j, i] = oz + (ks - 0.5)*dx
            ux1[j, i] = U[ks-1, j, i, 0]
            if ks-1-k8 >= 1: ux8[j, i] = U[ks-1-k8, j, i, 0]
            if ks-1-k16 >= 1: ux16[j, i] = U[ks-1-k16, j, i, 0]
    xs = ox + np.arange(Nx)*dx
    out = []
    for a, b in zip(kanten[:-1], kanten[1:]):
        m = (xs >= a) & (xs < b)
        v1 = ux1[:, m]; ok = np.isfinite(v1)
        if ok.sum() < 10: out.append(None); continue
        out.append(dict(zw=np.nanmean(zw[:, m]), u1=np.nanmean(v1), neg=float((v1[ok] < 0).mean()),
                        u8=np.nanmean(ux8[:, m]) if np.isfinite(ux8[:, m]).any() else np.nan,
                        u16=np.nanmean(ux16[:, m]) if np.isfinite(ux16[:, m]).any() else np.nan))
    return out, (f"dx {dx*1e3:.2f} mm, {ny} y-Ebenen, u_lat {float(d['u_lat']) if 'u_lat' in d else float('nan'):.4f}; u1/u8/u16 bei Wandabstand "
                 f"{0.5*dx*1e3:.1f}/{(k8+0.5)*dx*1e3:.1f}/{(k16+0.5)*dx*1e3:.1f} mm; Bodenspalt dk {dk} ({dk_q}) -> z_w = Welt-z - {dk*dx*1e3:.1f} mm")

def of13():
    S = np.load(os.path.join(SP, "of13_slab.npy")); x = S[:, 0] + XOFF; z = S[:, 2]; ux = S[:, 4]
    out = []
    for a, b in zip(kanten[:-1], kanten[1:]):
        zs_, u1s, u8s, u16s, negs = [], [], [], [], []
        for a2 in np.arange(a, b - 1e-9, 0.02):
            m = (x >= a2) & (x < a2 + 0.02) & (z < OF_ZMAX)
            if m.sum() < 20: continue
            zz = z[m]; uu = ux[m]; o = np.argsort(zz); zz, uu = zz[o], uu[o]
            # ★ berichtigt (2): die Obergrenze OF_ZMAX zaehlt als Luecke -- sonst fehlt die Unterseite, wo darueber bis 0,6 m keine Zelle liegt
            lu = np.nonzero(np.diff(np.append(zz, OF_ZMAX)) > OF_LUECKE)[0]
            if lu.size == 0: continue
            zu = zz[lu[0]]; s = zu - zz                         # Wandabstand unter der Unterseite (Zellmitte der obersten Zelle als Wandnaeherung)
            wand = (s >= 0.0) & (s <= 0.004)
            if int(wand.sum()) < OF_MIN_WANDZELLEN: continue    # ★ berichtigt (3): keine Prismenlagen -> keine Wand (Luecke zwischen groben Fluidzellen)
            zw_ = zu + 0.0005
            for lo, hi, L in ((0.0, 0.004, u1s), (0.006, 0.010, u8s), (0.014, 0.018, u16s)):
                mm = (s >= lo) & (s <= hi)
                if mm.any(): L.append(uu[mm].mean())
            negs.append((uu[wand] < 0).mean())                  # ★ berichtigt (1): dasselbe Fenster wie u1 (vorher s <= 4 mm ohne s >= 0 = Hohlraum darueber)
            zs_.append(zw_)
        if not zs_: out.append(None); continue
        f = lambda L: float(np.mean(L)) if L else np.nan
        out.append(dict(zw=f(zs_), u1=f(u1s), neg=f(negs), u8=f(u8s), u16=f(u16s)))
    return out

args = sys.argv[1:]
if not args: raise SystemExit(__doc__)
serien = [("OF13", of13(), f"Zellen |y|<0,03 m, s-Fenster 0-4 / 6-10 / 14-18 mm unter der wandnaechsten Zellmitte (Wandabstand + ~0,1-0,5 mm), "
                           f"Unterseite nur mit >= {OF_MIN_WANDZELLEN} Zellen in 0-4 mm; z_w = OF13-z (Koerper)")]
for a in args:
    n, p = a.split("=", 1); r, info = fx(p); serien.append((n, r, info))
print("# unterseite.py (17.09.2026, selbst berichtigt, Diff-Pruefung ausstehend) -- Unterboden Mittelebene, x-Abschnitte 100 mm; z_w = KOERPERhoehe der Unterseite [mm]")
print("#   (FX: Welt-z - dk*dx); u1 = u_x erste Zelle / OF13 0<=s<=4 mm; u8/u16 = u_x rund 8/16 mm unter der Wand (FX-Wandabstand je Serie unten);")
print("#   neg = Rueckstromanteil im u1-Fenster.")
for n, _, info in serien: print(f"#   {n}: {info}")
kopf = "x_v2 [m]  | " + " | ".join(f"{n:^30s}" for n, _, _ in serien)
print(kopf); print("          | " + " | ".join(f"{'z_w':>5s} {'u1':>6s} {'u8':>6s} {'u16':>6s} {'neg':>4s}" for _ in serien))
for k, (a, b) in enumerate(zip(kanten[:-1], kanten[1:])):
    zeile = f"{a:4.2f}-{b:4.2f} | "
    teile = []
    for n, r, _ in serien:
        e = r[k]
        teile.append(" "*30 if e is None else f"{e['zw']*1e3:5.0f} {e['u1']:+6.1f} {e['u8']:+6.1f} {e['u16']:+6.1f} {e['neg']:4.2f}")
    print(zeile + " | ".join(teile))
