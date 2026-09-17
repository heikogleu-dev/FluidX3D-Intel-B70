#!/usr/bin/env python3
# interface_serie.py -- Interface-Schnittserien durch die FERNdomaene gegen OF13 (27.08.2026,
# Heiko-Auftrag: "in 8 Zellen Abstaenden, jeweils vom Fahrzeug aus, zugeschnitten auf das
# Format der Nahdomaene -- ich moechte sehen, ab wo der Abdruck klein genug ist").
#
# ZWECK: die Nahfeld-Box gezielt dimensionieren. Fuer jede Kopplungsrichtung wird eine Serie
# von Ebenen durch das GROBGITTER gelegt, beginnend an der Fahrzeughuelle und dann in
# Schritten von 8 Grobzellen (8 x 16 mm = 128 mm) nach aussen. Je Ebene wird der Unterschied
# zu OF13 gemessen; wo er unter das Rauschband faellt, darf die Nahfeld-Grenze liegen.
#
# KONVENTION wie in export/vergleich_of13_2026-08-26 (die Bilder, auf die sich der Auftrag
# bezieht):  d = |u|_FX - |u|_OF13  in m/s,  Skala +-15,  rot = FX schneller, blau = FX
# langsamer, schwarz = Solid oder kein OF13-Datenpunkt.
# ACHTUNG, nicht verwechseln: werkzeuge/diff_of13_yslice.py benutzt die UMGEKEHRTE
# V1-Konvention (OF13 minus FX). Beide sind richtig, aber nicht vergleichbar.
#
# Iron Rule 5: die PNGs sind Sichtung. Gemessen wird an den gedruckten Statistiken und der
# CSV -- die entstehen direkt aus den Feldern, nicht aus dem Bild.
#
# Aufruf: interface_serie.py <feld_fern.vtk> <of13_zellen.npz> <out_dir> [n_ebenen=16]
#   Nahfeld-Box und Fahrzeughuelle kommen seit 17.09.2026 aus dem Lauf (feld_nah_*.vtk bzw. Log im selben Ordner), s. geometrie().
import sys, os, re, warnings
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

XOFF_HINWEIS = "OF13-Zellen liegen im Cache bereits in v2-Koordinaten (x_v2 = x_OF13 + 2,2063)."
SKALA = 15.0
SCHRITT_ZELLEN = 8          # Heikos Vorgabe
BIN = None                  # OF13-Binning: 2 Grobzellen -- wird nach dem Lesen des Fern-VTK gesetzt (2*dx_c)
DILAT = 10
# Slab-Halbdicke fuer die OF13-Auswahl. MUSS mindestens die halbe groebste Zellkante sein,
# sonst faellt eine Ebene in eine Luecke zwischen zwei Grobzell-Lagen und liefert nichts:
# mit 0,08 m sind am 27.08. mehrere Ebenen ausgefallen (z = 3,000 m: "zu wenig Zellen") und
# die auswertbare Zellzahl sprang zwischen 20.304 und 51.606. Das OF13-Basisgitter ist
# 24x12x8 m auf 120x60x40 Zellen MIT Grading, die groebsten Zellen liegen bei ~250 mm.
SLAB = 0.15
# Bei mehreren OF13-Zellen je Zielbin gewinnt die der Ebene NAECHSTE (Naeherung an
# "nearest cell", was bei einem Finite-Volumen-Feld die richtige Auslegung ist: der Zellwert
# gilt im ganzen Zellvolumen). Mittelwertbildung ueber verschiedene Verfeinerungsstufen
# waere hier falsch -- sie mischte eine 4-mm-Wandzelle mit einer 250-mm-Fernzelle.

# --- Geometrie AUS DEM LAUF (★ 17.09.2026, SKALIERUNG-BEFUNDE Nebenbefund 10) -----------------------------------------
# Bis 17.09. standen hier die Nahfeld-Box und die Fahrzeughuelle des Laufs f4_vollumfang_mls fest (y0 -1,24, ny 621, nz 485,
# dx 4 mm) -- auf JEDER heutigen Sprosse falsch (NEAR_LY/LZ geaendert, 8/3,75/16 mm, CFD_Y_VERSATZ), ohne Meldung.
# Jetzt:  NAH = Kopf eines feld_nah_*.vtk im Ordner des Fern-VTK (exakt; gleiche Zeit bevorzugt), sonst Log-Zeile
#               "Nahfeld x[..] y[..] z[..]" (3 Nachkommastellen, LAUT) + feine Zellweite aus "Unit Conversion", sonst Abbruch.
#         FZG = Fahrzeughuelle wie am 27.08. definiert: Bounding-Box der FLUIDzellen neben dem Fahrzeug (0x41) = 0x41-Box +-1 Zelle;
#               unten die unterste Fluidlage neben dem Koerper (z-Index 0 ist Fahrbahn). Aus den flags des Nah-VTK, sonst
#               (LAUT, grob) aus den flags des Fern-VTK. Der Y-Versatz steckt damit automatisch im Voxelkoerper.
def _nah_quelle(fern_vtk):
    d = os.path.dirname(os.path.abspath(fern_vtk))
    m = re.search(r"feld_fern_(\d+ms)\.vtk$", os.path.basename(fern_vtk))
    gleich = os.path.join(d, f"feld_nah_{m.group(1)}.vtk") if m else None
    if gleich and os.path.exists(gleich): return gleich
    kand = sorted(p for p in os.listdir(d) if p.startswith("feld_nah_") and p.endswith(".vtk"))
    return os.path.join(d, kand[-1]) if kand else None

def _huelle(vtk):
    """(FZG-dict, Text) aus den flags eines dd-Feld-VTK: 0x41-Box +-1 Zelle, unten die unterste Fluidlage ueber der Fahrbahn."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from zonen_kraft import kopf
    k = kopf(vtk); nx, ny, nz = k["dims"]; ox, oy, oz = k["orig"]; dx = k["spac"][0]
    FL = np.memmap(vtk, dtype=np.uint8, mode="r", offset=k["off_flags"], shape=(nz, ny, nx))
    imin = jmin = kmin = 10**9; imax = jmax = kmax = -1
    for kk in range(nz):
        e = np.asarray(FL[kk]) == 0x41
        if not e.any(): continue
        jj = np.nonzero(e.any(axis=1))[0]; ii = np.nonzero(e.any(axis=0))[0]
        kmin = min(kmin, kk); kmax = kk; jmin = min(jmin, jj[0]); jmax = max(jmax, jj[-1]); imin = min(imin, ii[0]); imax = max(imax, ii[-1])
    if kmax < 0: raise SystemExit(f"FEHLER: keine Fahrzeugzelle (0x41) in {vtk}")
    klo = kmin - 1 if kmin - 1 >= 1 else kmin      # z-Index 0 ist Fahrbahn (TYPE_S), keine Fluidlage
    imin, imax, jmin, jmax, kmin, kmax, klo = (int(v) for v in (imin, imax, jmin, jmax, kmin, kmax, klo))   # Python-int -> Python-float: np.float64-Skalare
    # wuerden float32-OF13-Caches in of13_ebene() auf float64 heben (NEP 50) und die Slab-Auswahl gegen die alte Fassung verschieben
    f = dict(x=(ox + (imin-1)*dx, ox + (imax+1)*dx), y=(oy + (jmin-1)*dx, oy + (jmax+1)*dx), z=(oz + klo*dx, oz + (kmax+1)*dx))
    return f, f"0x41-Box i {imin}..{imax} j {jmin}..{jmax} k {kmin}..{kmax} in {os.path.basename(vtk)} (dx {dx*1e3:.4g} mm)"

def geometrie(fern_vtk):
    nah_vtk = _nah_quelle(fern_vtk)
    if nah_vtk:
        with open(nah_vtk, "rb") as f: kopf_ = f.read(1024)
        nx, ny, nz = map(int, re.search(rb"DIMENSIONS (\d+) (\d+) (\d+)", kopf_).groups())
        x0, y0, z0 = map(float, re.search(rb"ORIGIN (\S+) (\S+) (\S+)", kopf_).groups())
        dx = float(re.search(rb"SPACING (\S+)", kopf_).group(1))
        nah = dict(x0=x0, y0=y0, z0=z0, dx=dx, nx=nx, ny=ny, nz=nz); q_nah = f"Kopf {nah_vtk}"
        fzg, q_fzg = _huelle(nah_vtk)
        return nah, fzg, q_nah, q_fzg
    d = os.path.dirname(os.path.abspath(fern_vtk)); name = os.path.basename(d)
    log = os.path.join(d, "..", "..", "logs", name + ".log")
    if not os.path.exists(log): raise SystemExit(f"FEHLER: weder feld_nah_*.vtk in {d} noch Log {log} -- Nahfeld-Box unbekannt")
    t = re.sub(r"\s+", " ", re.sub(r"\|\s*\n\|\s*", " ", re.sub(r"\x1b\[[0-9;]*m", "", open(log, errors="replace").read())))
    m = re.search(r"Nahfeld x\[(-?[0-9.]+);(-?[0-9.]+)\] y\[(-?[0-9.]+);(-?[0-9.]+)\] z\[(-?[0-9.]+);(-?[0-9.]+)\]", t)
    u = re.search(r"Unit Conversion: 1 cell = ([0-9.]+) mm", t)
    if not (m and u): raise SystemExit(f"FEHLER: Log {log} ohne 'Nahfeld x[..]'/'Unit Conversion' -- Nahfeld-Box unbekannt")
    a = [float(v) for v in m.groups()]; dx = float(u.group(1))*1e-3
    nah = dict(x0=a[0], y0=a[2], z0=a[4], dx=dx, nx=int(round((a[1]-a[0])/dx))+1, ny=int(round((a[3]-a[2])/dx))+1, nz=int(round((a[5]-a[4])/dx))+1)
    print("WARNUNG: kein feld_nah_*.vtk -- Nahfeld-Box aus der Log-Zeile (3 Nachkommastellen, Ursprung auf 1 mm genau) "
          "und Fahrzeughuelle aus den GROBEN flags des Fern-VTK (eine Grobzelle Unschaerfe).")
    fzg, q_fzg = _huelle(fern_vtk)
    return nah, fzg, f"Log {os.path.abspath(log)}", q_fzg + " [GROB]"

def lies_fern(pfad):
    with open(pfad, "rb") as f:
        kopf = f.read(1024)
        nx, ny, nz = map(int, re.search(rb"DIMENSIONS (\d+) (\d+) (\d+)", kopf).groups())
        ox, oy, oz = map(float, re.search(rb"ORIGIN (\S+) (\S+) (\S+)", kopf).groups())
        dx = float(re.search(rb"SPACING (\S+)", kopf).group(1))
        tag = b"VECTORS u float\n"; u0 = kopf.index(tag) + len(tag)
    u = np.memmap(pfad, dtype=">f4", mode="r", offset=u0, shape=(nz, ny, nx, 3))
    return u, dict(nx=nx, ny=ny, nz=nz, ox=ox, oy=oy, oz=oz, dx=dx)

def of13_ebene(C, achse, wert, achse1, gitter1, achse2, gitter2, slab):
    """OF13 auf ein 2D-Gitter der Ebene achse=wert binnen (Mittel je Bin) + Dilatation."""
    m = np.abs(C[achse] - wert) < slab
    if m.sum() < 100: return None, int(m.sum())
    a1, a2, uu = C[achse1][m], C[achse2][m], C["mag"][m]
    dist = np.abs(C[achse][m] - wert)
    n1, n2 = len(gitter1), len(gitter2)
    b1 = int(np.ceil(n1 * (gitter1[1]-gitter1[0]) / BIN)) + 1
    b2 = int(np.ceil(n2 * (gitter2[1]-gitter2[0]) / BIN)) + 1
    i1 = ((a1 - gitter1[0]) / BIN).astype(int); i2 = ((a2 - gitter2[0]) / BIN).astype(int)
    g = (i1 >= 0) & (i1 < b1) & (i2 >= 0) & (i2 < b2)
    feld = np.full((b2, b1), np.nan)
    o = np.argsort(-dist[g])                       # absteigender Abstand -> die naechste Zelle schreibt zuletzt
    feld[i2[g][o], i1[g][o]] = uu[g][o]
    for _ in range(DILAT):
        p = np.pad(feld, 1, constant_values=np.nan)
        st = np.stack([p[a:a+b2, b:b+b1] for a in range(3) for b in range(3)])
        with warnings.catch_warnings():
            warnings.simplefilter("ignore"); mm = np.nanmean(st, axis=0)
        feld = np.where(np.isnan(feld), mm, feld)
    j1 = np.clip(((gitter1 - gitter1[0]) / BIN).astype(int), 0, b1-1)
    j2 = np.clip(((gitter2 - gitter2[0]) / BIN).astype(int), 0, b2-1)
    return feld[np.ix_(j2, j1)], int(m.sum())

def bild(d, maske, pfad, titel, xlabel, ylabel, ausdehnung):
    v = np.clip(np.where(maske, 0.0, d), -SKALA, SKALA)
    rgb = np.ones(v.shape + (3,), dtype=np.float32)
    tp = np.clip(v / SKALA, 0, 1); tn = np.clip(-v / SKALA, 0, 1)
    rgb[..., 1] -= tp; rgb[..., 2] -= tp          # rot: FX schneller
    rgb[..., 0] -= tn; rgb[..., 1] -= tn          # blau: FX langsamer
    rgb[maske] = 0.0
    h, w = v.shape
    fig, ax = plt.subplots(figsize=(max(4.0, w/44), max(2.6, h/44)), dpi=125)
    ax.imshow(rgb, origin="lower", extent=ausdehnung, interpolation="nearest", aspect="equal")
    ax.set_title(titel, fontsize=8); ax.set_xlabel(xlabel, fontsize=7); ax.set_ylabel(ylabel, fontsize=7)
    ax.tick_params(labelsize=6)
    fig.tight_layout(); fig.savefig(pfad); plt.close(fig)

def statistik(d, maske, fxm, ofm):
    w = d[~maske]
    if w.size < 50: return None
    return dict(n=int(w.size), mittel=float(w.mean()), rms=float(np.sqrt(np.mean(w**2))),
                fx=float(fxm[~maske].mean()), of=float(ofm[~maske].mean()),
                p05=float(np.percentile(w, 5)), p50=float(np.median(w)), p95=float(np.percentile(w, 95)),
                a1=float(np.mean(np.abs(w) > 1.0)), a2=float(np.mean(np.abs(w) > 2.0)),
                amax=float(np.abs(w).max()))

vtk = sys.argv[1]; cache = sys.argv[2]; out = sys.argv[3]
NEB = int(sys.argv[4]) if len(sys.argv) > 4 else 16
os.makedirs(out, exist_ok=True)
u, M = lies_fern(vtk)
DX = M["dx"]; SCHRITT = SCHRITT_ZELLEN * DX
BIN = 2*DX      # ★ 17.09.2026: "2 Grobzellen" war fest 0,032 m (nur bei dx_c 16 mm richtig)
NAH, FZG, Q_NAH, Q_FZG = geometrie(vtk)
NAH_X = (NAH["x0"], NAH["x0"] + NAH["nx"]*NAH["dx"])
NAH_Y = (NAH["y0"], NAH["y0"] + NAH["ny"]*NAH["dx"])
NAH_Z = (NAH["z0"], NAH["z0"] + NAH["nz"]*NAH["dx"])
print(f"Fernfeld {M['nx']}x{M['ny']}x{M['nz']} @ {DX*1000:.4g} mm, Ursprung ({M['ox']:.4f},{M['oy']:.4f},{M['oz']:.4f}); OF13-Binning {BIN*1000:.4g} mm")
print(f"Nahfeld-Box {NAH['nx']}x{NAH['ny']}x{NAH['nz']} @ {NAH['dx']*1000:.4g} mm aus {Q_NAH}")
print(f"Fahrzeughuelle x [{FZG['x'][0]:+.4f},{FZG['x'][1]:+.4f}] y [{FZG['y'][0]:+.4f},{FZG['y'][1]:+.4f}] z [{FZG['z'][0]:+.4f},{FZG['z'][1]:+.4f}] m aus {Q_FZG}")
print(f"Nahfeld-Ausschnitt x [{NAH_X[0]:.3f},{NAH_X[1]:.3f}] y [{NAH_Y[0]:.3f},{NAH_Y[1]:.3f}] z [{NAH_Z[0]:.3f},{NAH_Z[1]:.3f}] m")
print(f"Schritt {SCHRITT_ZELLEN} Grobzellen = {SCHRITT*1000:.0f} mm (Heikos Vorgabe in ZELLEN -- auf 8 mm sind das 256 mm), {NEB} Ebenen je Serie. {XOFF_HINWEIS}\n")

z = np.load(cache)
C = dict(x=z["cx"], y=z["cy"], z=z["cz"])
C["mag"] = np.linalg.norm(z["U"].astype(np.float64), axis=1)
print(f"OF13-Cache: {C['x'].size} Zellen, |u| Mittel {C['mag'].mean():.2f} m/s")
# ---- ABNAHME der OF13-Interpolation (Iron Rule: kein Ergebnis aus ungeprueftem Werkzeug).
# In der ungestoerten Anstroemung weit vor dem Fahrzeug MUSS die Rekonstruktion die
# Einlassgeschwindigkeit treffen; tut sie das nicht, stimmt Zuordnung oder Slab nicht.
_pruef_y = np.linspace(-1.0, 1.0, 41); _pruef_z = np.linspace(0.5, 3.0, 41)
_of, _n = of13_ebene(C, "x", -2.0, "y", _pruef_y, "z", _pruef_z, SLAB)
if _of is None or np.isnan(_of).all():
    raise SystemExit("ABNAHME GESCHEITERT: keine OF13-Daten bei x = -2,0 m")
_m = float(np.nanmean(_of)); _s = float(np.nanstd(_of)); _l = float(np.mean(np.isnan(_of)))
print(f"ABNAHME Anstroemung x = -2,0 m (2,0 m vor dem Fahrzeug): |u|_OF13 = {_m:.3f} +- {_s:.3f} m/s "
      f"(Soll 30,0; Luecken {100*_l:.1f} %)")
if abs(_m - 30.0) > 0.6 or _s > 0.6:
    raise SystemExit("ABNAHME GESCHEITERT: Anstroemung wird nicht getroffen -- Zuordnung/Slab pruefen, "
                     "die Serie waere nicht belastbar.")
print("  -> bestanden, Serie ist belastbar.\n")

# Indexbereiche des Nahfeld-Ausschnitts im Grobgitter
ix = np.arange(max(0, int(np.floor((NAH_X[0]-M["ox"])/DX))), min(M["nx"], int(np.ceil((NAH_X[1]-M["ox"])/DX))))
iy = np.arange(max(0, int(np.floor((NAH_Y[0]-M["oy"])/DX))), min(M["ny"], int(np.ceil((NAH_Y[1]-M["oy"])/DX))))
iz = np.arange(max(0, int(np.floor((NAH_Z[0]-M["oz"])/DX))), min(M["nz"], int(np.ceil((NAH_Z[1]-M["oz"])/DX))))
wx = M["ox"] + ix*DX; wy = M["oy"] + iy*DX; wz = M["oz"] + iz*DX
print(f"Ausschnitt im Grobgitter: {len(ix)} x {len(iy)} x {len(iz)} Zellen\n")

SERIEN = [
    ("xm", "x", FZG["x"][0], -1, "vor dem Fahrzeug (stromauf, Kopplungsebene x-)"),
    ("xp", "x", FZG["x"][1], +1, "hinter dem Fahrzeug (stromab)"),
    ("ym", "y", FZG["y"][0], -1, "seitlich (Kopplungsebene y-)"),
    ("yp", "y", FZG["y"][1], +1, "seitlich (Kopplungsebene y+)"),
    ("zp", "z", FZG["z"][1], +1, "ueber dem Fahrzeug (Kopplungsebene z+)"),
]
NAHGRENZE = dict(xm=NAH_X[0], xp=NAH_X[1], ym=NAH_Y[0], yp=NAH_Y[1], zp=NAH_Z[1])
START = {t: st for t, _, st, _, _ in SERIEN}   # Startflaeche je Serie = Fahrzeughuelle
csv = open(os.path.join(out, "interface_serie.csv"), "w")
csv.write("# d = |u|_FX - |u|_OF13 (m/s), Ebenen durch die FERNdomaene, Ausschnitt = Nahfeld-Format\n")
csv.write("serie,abstand_m,welt_m,jenseits_nahgrenze,n,u_fx_mittel,u_of13_mittel,mittel,rms,p05,p50,p95,anteil_ueber_1ms,anteil_ueber_2ms,max_abs\n")
zus = {}
for tag, achse, start, vz, was in SERIEN:
    print(f"--- Serie {tag}: {was}; Start an der Fahrzeughuelle {achse} = {start:+.3f} m, "
          f"Nahfeld-Grenze {achse} = {NAHGRENZE[tag]:+.3f} m")
    reihe = []
    for k in range(NEB):
        wert = start + vz*k*SCHRITT
        if achse == "x":
            i = int(round((wert-M["ox"])/DX))
            if not (0 <= i < M["nx"]): break
            fx = np.linalg.norm(np.array(u[iz[0]:iz[-1]+1, iy[0]:iy[-1]+1, i], dtype=np.float64), axis=-1)
            of, nz_ = of13_ebene(C, "x", wert, "y", wy, "z", wz, SLAB)
            ausd = (wy[0], wy[-1], wz[0], wz[-1]); xl, yl = "y [m]", "z [m]"
        elif achse == "y":
            j = int(round((wert-M["oy"])/DX))
            if not (0 <= j < M["ny"]): break
            fx = np.linalg.norm(np.array(u[iz[0]:iz[-1]+1, j, ix[0]:ix[-1]+1], dtype=np.float64), axis=-1)
            of, nz_ = of13_ebene(C, "y", wert, "x", wx, "z", wz, SLAB)
            ausd = (wx[0], wx[-1], wz[0], wz[-1]); xl, yl = "x [m]", "z [m]"
        else:
            kk = int(round((wert-M["oz"])/DX))
            if not (0 <= kk < M["nz"]): break
            fx = np.linalg.norm(np.array(u[kk, iy[0]:iy[-1]+1, ix[0]:ix[-1]+1], dtype=np.float64), axis=-1)
            of, nz_ = of13_ebene(C, "z", wert, "x", wx, "y", wy, SLAB)
            ausd = (wx[0], wx[-1], wy[0], wy[-1]); xl, yl = "x [m]", "y [m]"
        if of is None:
            print(f"  {tag}{k:02d} {achse}={wert:+.3f}: nur {nz_} OF13-Zellen im Slab -- uebersprungen"); continue
        d = fx - of
        maske = (fx == 0.0) | np.isnan(of)
        s = statistik(d, maske, fx, of)
        if s is None:
            print(f"  {tag}{k:02d} {achse}={wert:+.3f}: zu wenig auswertbare Zellen"); continue
        abst = abs(wert - start); jens = (abs(wert-start) > abs(NAHGRENZE[tag]-start))
        reihe.append((abst, s))
        csv.write(f"{tag},{abst:.3f},{wert:.4f},{int(jens)},{s['n']},{s['fx']:.4f},{s['of']:.4f},{s['mittel']:.4f},{s['rms']:.4f},"
                  f"{s['p05']:.4f},{s['p50']:.4f},{s['p95']:.4f},{s['a1']:.5f},{s['a2']:.5f},{s['amax']:.3f}\n")
        titel = (f"{tag}{k:02d}  {achse} = {wert:+.3f} m   (Abstand vom Fahrzeug {abst*1000:.0f} mm"
                 f"{', JENSEITS der Nahfeld-Grenze' if jens else ''})\n"
                 f"d = |u|_FX - |u|_OF13   Mittel {s['mittel']:+.2f}  RMS {s['rms']:.2f}  "
                 f"|d|>1 m/s: {100*s['a1']:.1f} %   [Skala +-15 m/s]")
        bild(d, maske, os.path.join(out, f"diff_{tag}{k:02d}.png"), titel, xl, yl, ausd)
        print(f"  {tag}{k:02d} {achse}={wert:+8.3f}  Abstand {abst*1000:6.0f} mm  n={s['n']:6d}  "
              f"|u|_FX {s['fx']:5.2f} |u|_OF {s['of']:5.2f}  Mittel {s['mittel']:+6.2f}  RMS {s['rms']:5.2f}  |d|>1: {100*s['a1']:5.1f} %  "
              f"|d|>2: {100*s['a2']:5.1f} %{'   <- jenseits Nahfeld' if jens else ''}")
    zus[tag] = reihe
csv.close()

fig, axe = plt.subplots(1, 2, figsize=(11, 4), dpi=130)
farben = dict(xm="tab:blue", xp="tab:red", ym="tab:green", yp="tab:olive", zp="tab:purple")
for tag, reihe in zus.items():
    if not reihe: continue
    a = np.array([r[0] for r in reihe]); r1 = np.array([r[1]["rms"] for r in reihe])
    a1 = np.array([100*r[1]["a1"] for r in reihe])
    axe[0].plot(a*1000, r1, "o-", color=farben[tag], label=tag, ms=3)
    axe[1].plot(a*1000, a1, "o-", color=farben[tag], label=tag, ms=3)
    g = abs(NAHGRENZE[tag] - START[tag])
    for ax in axe: ax.axvline(g*1000, color=farben[tag], ls=":", lw=1, alpha=0.6)
axe[0].set_ylabel("RMS von d [m/s]"); axe[1].set_ylabel("Anteil |d| > 1 m/s [%]")
for ax in axe:
    ax.set_xlabel("Abstand von der Fahrzeughuelle [mm]"); ax.grid(alpha=0.3); ax.legend(fontsize=7)
axe[0].set_title("Abklingen des Unterschieds zu OF13 (Ebenen durch die Ferndomaene)", fontsize=9)
axe[1].set_title("gepunktet = heutige Nahfeld-Grenze je Richtung", fontsize=9)
fig.tight_layout(); fig.savefig(os.path.join(out, "verlauf.png")); plt.close(fig)
print(f"\nCSV + Bilder + verlauf.png in {out}")
