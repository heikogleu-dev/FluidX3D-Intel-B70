#!/usr/bin/env python3
"""totaldruck_kette.py -- traegt der Totaldruckverlust vor dem Heckfluegel die Handschrift der Dachgrenzschicht? (17.09.2026)
Heiko 17.09.: "Dass der Heckfluegel nicht liefert, liegt an der schlechten Anstroemung durch Abloesung am Dach."
Gemessen wird 1 - cp0 (Totaldruckverlust, cp0 = cp + |u|^2/U_inf^2, beide Seiten auf den Totaldruck-Bezug ueber dem Dach gestellt,
also cp0 = 1 in verlustfreier Stroemung) an Stationen x_v2 und je y-Streifen, von der Dachmitte bis vor die Fluegelvorderkante.
FX: Momentanfelder p375_e/p375_b 501 ms (u_lat und cp_ref aus Log bzw. zonenkraft_*ms.npz). OF13: RANS-Mittel t=1200, Zellmitten.
Ausgabe je (x, y-Streifen): Hoehe ueber der Wand/dem Deck, bis zu der 1-cp0 > 0,1 reicht (Verlustschichtdicke), und mittleres
1-cp0 sowie u_x im Fluegelspalt (Deck bis Fluegelunterkante) -- das ist die Anstroemung der Fluegelunterseite.
Aufruf: totaldruck_kette.py [lauf:t_ms ...]   (ohne Argumente wie bisher p375_e:000501 p375_b:000501; OF13 steht beim ERSTEN Feld)
★ 17.09.2026 (SKALIERUNG-BEFUNDE Nebenbefund 10): Laufliste war fest; die |y|-Streifen lagen um WELT-y = 0. Jetzt Felder als Argument
und Streifen KOERPERBEZOGEN (y_Welt = y_Koerper + Y-Versatz des Laufs aus dem Protokoll, zonen_kraft.protokoll/lauf_meta.py).
★ 17.09.2026 (Pruefbefund 9/10): Hoehen KOERPERBEZOGEN -- der Upstream-Bodenspalt hebt den FX-Koerper um dk*dx (lauf_meta.bodenspalt);
FX-z-Fenster 0,6..1,35 m und die gedruckte Wandhoehe z_w sind Koerper-z = Welt-z - dk*dx, die OF13-Hoehen ueber der Wand (OF13-z minus
FX-Wandhoehe) damit lagegleich. Y-Versatz mit dem VTK bestimmt; unbekannt -> WARNUNG (0 angenommen)."""
import os, sys, gzip
import numpy as np
HIER = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HIER, ".."))
from zonen_kraft import kopf, protokoll
import lauf_meta
EXP = os.path.join(HIER, "..", "..", "export"); XOFF = 2.2063
X_ST = [2.30, 2.90, 3.30, 3.60, 3.90, 4.05]
Y_ST = [(0.0, 0.2), (0.2, 0.4), (0.4, 0.6), (0.6, 0.8)]

def of13_zellen(cache):
    if os.path.exists(cache): return np.load(cache)["S"]
    def internal(pfad, k):
        with gzip.open(pfad, "rt", errors="replace") as f:
            for z in f:
                if z.startswith("internalField"): break
            n = None
            for z in f:
                s = z.strip()
                if not s: continue
                if n is None and s.isdigit(): n = int(s); continue
                if n is not None and s == "(": break
            r = f.read()
        r = r[:r.index("\n)")]
        if k > 1: r = r.replace("(", " ").replace(")", " ")
        return np.fromstring(r, sep=" ").reshape(n, k) if k > 1 else np.fromstring(r, sep=" ")
    F = os.path.expanduser("~/CFD-Cases/mr2v40H/1200")
    C = internal(F + "/C.gz", 3); m = (C[:, 0]+XOFF > 2.2) & (C[:, 0]+XOFF < 4.2) & (np.abs(C[:, 1]) < 0.85) & (C[:, 2] > 0.6) & (C[:, 2] < 1.6)
    p = internal(F + "/p.gz", 1); U = internal(F + "/U.gz", 3)
    S = np.column_stack([C[m], p[m], U[m]]).astype(np.float32); np.savez_compressed(cache, S=S); return S

S = of13_zellen(os.path.join(EXP, "of13_heckregion_1200.npz"))
sl = np.load(os.path.join(HIER, "of13_slab.npy")).astype(np.float64)
mr = (sl[:, 0]+XOFF > 0.3) & (sl[:, 0]+XOFF < 3.0) & (sl[:, 2] > 1.5) & (sl[:, 2] < 1.8)
CPREF_OF = float(np.median(sl[mr, 3] + 0.5*((sl[mr, 4:7]**2).sum(1) - 900.0))/450.0); del sl
ox_, oy_, oz_ = S[:, 0].astype(float)+XOFF, S[:, 1].astype(float), S[:, 2].astype(float)
of_cp0 = (S[:, 3]/450.0 - CPREF_OF) + (S[:, 4:7].astype(float)**2).sum(1)/900.0
of_ux = S[:, 4].astype(float)

def fx_feld(lauf, tms):
    vtk = os.path.join(EXP, lauf, f"feld_nah_{tms}ms.vtk"); d = kopf(vtk); Nx, Ny, Nz = d["dims"]; ox, oy, oz = d["orig"]; dx = d["spac"][0]
    P = protokoll(os.path.join(EXP, lauf), dx, vtk); ul = P["u_lat"]; z = np.load(os.path.join(EXP, lauf, f"zonenkraft_{tms}ms.npz"))
    yv = P["y_versatz_mm"]*1e-3
    dk, dk_q = lauf_meta.bodenspalt(os.path.join(EXP, lauf))
    print(f"{lauf} {tms} ms: u_lat {ul} ({P.get('u_lat_quelle', 'Log')}), Y-Versatz {P['y_versatz_mm']} mm ({P.get('y_versatz_quelle', 'Log')})")
    if not P["y_versatz_bekannt"]:
        print(f"WARNUNG {lauf}: Y-Versatz UNBEKANNT -- 0 angenommen; die |y|-Streifen liegen bei CFD_Y_VERSATZ=1 eine halbe Zelle neben der Koerpermitte")
    if dk is None:
        print(f"WARNUNG {lauf}: Bodenspalt dk {dk_q} -- 0 angenommen; Hoehen dann ggf. um dk*dx gegen OF13 verschoben"); dk = 0
    print(f"{lauf}: Bodenspalt dk = {dk} Zellen ({dk_q}) -> Koerper-z = Welt-z - {dk*dx*1e3:.3f} mm (z-Fenster, z_w und OF13-Hoehen koerperbezogen)")
    cpf = 2.0/(3.0*ul*ul); cpref = (float(z["rho_inf_mitte"][0]) - 1.0)*cpf
    FL = np.memmap(vtk, dtype=np.uint8, mode="r", offset=d["off_flags"], shape=(Nz, Ny, Nx))
    RH = np.memmap(vtk, dtype=">f4", mode="r", offset=d["off_rho"], shape=(Nz, Ny, Nx))
    UU = np.memmap(vtk, dtype=">f4", mode="r", offset=d["off_u"], shape=(Nz, Ny, Nx, 3))
    return dict(FL=FL, RH=RH, UU=UU, ox=ox, oy=oy, oz=oz - dk*dx, dx=dx, cpf=cpf, cpref=cpref, Nz=Nz, yv=yv)   # oz koerperbezogen (Welt - dk dx)

if len(sys.argv) > 1:
    FELDER = []
    for a in sys.argv[1:]:
        if ":" not in a: raise SystemExit(f"Feld '{a}': Form lauf:t_ms, z. B. p375_e:000501")
        l, t = a.split(":", 1); t = t.zfill(6); FELDER.append((f"{l}@{int(t)}", l, t))
else:
    FELDER = (("e501", "p375_e", "000501"), ("b501", "p375_b", "000501"))
FXF = {k: fx_feld(l, t) for k, l, t in FELDER}
K_OF = FELDER[0][0]      # OF13-Spalte beim ersten Feld (Fassung vor 17.09.: fest e501)
print(f"cp_ref OF13 {CPREF_OF:+.4f} | " + " ".join(f"{k} {v['cpref']:+.4f}" for k, v in FXF.items()))
print("Je Station und |y|-Streifen: Wandhoehe z_w (oberste Fahrzeugzelle unter z 1,3 bzw. Deck), Verlustschicht d10 = Hoehe ueber z_w,")
print("bis zu der der Streifenmittelwert 1-cp0 > 0,10 bleibt; ausserdem 1-cp0 bei 10/30/60 mm ueber z_w.  OF13 | " + " | ".join(FXF))
for xq in X_ST:
    for ya, yb in Y_ST:
        zeile = f"x {xq:.2f} |y| {ya:.1f}-{yb:.1f} |"
        # OF13: Zellen im Streifen x+-5 mm, y-Band, Profil ueber z in 5-mm-Bins; Wand = unterste belegte z-Bin-Grenze ist unsicher -> z_w aus FX-Maske
        for k, F in FXF.items():
            dx = F["dx"]; i = int(round((xq - F["ox"])/dx))
            ja, jb = int(round((ya + F["yv"] - F["oy"])/dx)), int(round((yb + F["yv"] - F["oy"])/dx))      # koerperbezogen: +Y-Versatz
            jc, jd = int(round((-yb + F["yv"] - F["oy"])/dx)), int(round((-ya + F["yv"] - F["oy"])/dx))
            ka, kb = int(round((0.6 - F["oz"])/dx)), int(round((1.35 - F["oz"])/dx))
            js = np.r_[jc:jd, ja:jb]
            fl = np.asarray(F["FL"][ka:kb, :, i])[:, js]; r = np.asarray(F["RH"][ka:kb, :, i])[:, js].astype(float)
            u = np.asarray(F["UU"][ka:kb, :, i])[:, js].astype(float)
            fz = fl == 0x41
            # Wandhoehe je Spalte: oberste Fahrzeugzelle unterhalb des Fluegels (erste Luecke von unten nach oben ueber dem Koerper)
            cp0 = (r - 1.0)*F["cpf"] - F["cpref"] + (u**2).sum(-1)/900.0
            cp0[fl != 0] = np.nan
            zz = F["oz"] + (ka + np.arange(kb-ka))*dx
            zw = np.full(len(js), np.nan)
            for c in range(len(js)):
                idx = np.nonzero(fz[:, c])[0]
                if idx.size:
                    run_end = idx[0]
                    while run_end + 1 < fz.shape[0] and fz[run_end+1, c]: run_end += 1
                    zw[c] = zz[run_end] + 0.5*dx
            zwm = np.nanmedian(zw)
            if not np.isfinite(zwm): zeile += f" {k}: kein Koerper |"; continue
            hoehe = zz - zwm
            prof = np.nanmean(1.0 - cp0, axis=1)
            ok = hoehe > 0
            d10 = np.nan
            h_ = hoehe[ok]; p_ = prof[ok]
            if p_.size and p_[0] > 0.1:
                j = np.argmax(p_ <= 0.1) if np.any(p_ <= 0.1) else len(p_)-1
                d10 = h_[j]
            v = lambda hm: p_[int(np.argmin(np.abs(h_ - hm/1000)))] if p_.size else np.nan
            if k == K_OF:
                mo = (np.abs(ox_ - xq) < 0.006) & (np.abs(oy_) >= ya) & (np.abs(oy_) < yb)
                ho = oz_[mo] - zwm; lo = 1.0 - of_cp0[mo]
                vo = lambda hm: np.mean(lo[np.abs(ho - hm/1000) < 0.004]) if np.any(np.abs(ho - hm/1000) < 0.004) else np.nan
                bins = np.arange(0.0, 0.40, 0.005); dob = np.nan
                for b0 in bins:
                    mm_ = (ho >= b0) & (ho < b0 + 0.005)
                    if mm_.sum() >= 3 and np.mean(lo[mm_]) <= 0.1: dob = b0; break
                zeile += f" OF13 d10 {1e3*dob:5.0f} mm, 1-cp0 @10/30/60 {vo(10):+.2f} {vo(30):+.2f} {vo(60):+.2f} |"
            zeile += f" {k} z_w {zwm:.3f} d10 {1e3*d10:5.0f} mm, 1-cp0 @10/30/60 {v(10):+.2f} {v(30):+.2f} {v(60):+.2f} |"
        print(zeile)
