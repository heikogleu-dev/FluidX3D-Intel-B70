#!/usr/bin/env python3
"""zonen_kraft.py -- FX-Fahrzeugkraft aus einem Nahfeld-VTK nach Zonen zerlegen (17.09.2026).

WOZU (Heiko 16./17.09.): die Abtriebsluecke gegen OF13 (Cz_rest ~75 %) ORTSGENAU finden. Der Facettenpfad
des Solvers (cd_facetten.csv) liefert nur Summen und ein Hoehenband. Dieses Werkzeug zerlegt die Kraft je
Wandlink in Bins (x-Streifen x Hoehe x Orientierung), damit dieselbe Zerlegung auf die OF13-Randflaechen
(werkzeuge/of13_kraft_zband.py, OF13_CACHE) gelegt werden kann -> werkzeuge/zonen_vergleich.py.

VERFAHREN (Planungsagent 17.09., Antworten 1-8; rein lesend):
  Link = Fluidzelle (flags 0x00 oder 0x03) -> Fahrzeugzelle (flags == 0x41) in Richtung c (D3Q19, 18 Richtungen).
  Je Link drei Skalare, die mit c multipliziert die Kraft auf das Fahrzeug geben (Gittereinheiten):
    g  = 2 w                       -> G = Sum 2 w c    (Geometrie; dF/d rho_ref = -G)
    p1 = 2 w (rho - 1)             -> P1               (Druckproxy, Bezug rho = 1; an ebener Wand = (rho-1)/3 je Zellflaeche)
    e1 = 2 (f_eq(rho,u) - w)       -> E1               (Gleichgewichts-Impulsaustausch, Bezug rho = 1)
  Kraft mit anderem Bezug: P = P1 - (rho_ref - 1) G  (ebenso E). rho, u = Werte der FLUIDZELLE (u aus dem VTK, SI -> Gitter).
  Einheiten: F_SI = F_lat * rho_SI * dx^2 * (si_u/u_lat)^2 ; C = F_SI / (q_inf A_ref).

EINGEBAUTE ABNAHMEN (Abbruch = SystemExit, Warnung = markiert in der Ausgabe):
  A1 Layout (Marker, Dateigroesse) wie fx_band.py; u_lat aus dem Laufprotokoll (NIE Vorgabe -- Falle 17.09.: x2,78).
  A2 Flag-Zensus: nur {0x00,0x01,0x02,0x03,0x41}; z=0 nur 0x01; 0x41 bei z<4 == "Band-Census" im Log;
     Facettenzellen (Fluid mit >=1 Fahrzeuglink) == Zeilenzahl facetten_histogramme.csv.
  A3 Schliessung EXAKT: Sum 2wc ueber (Fluid->Fahrzeug) + (Nicht-Fluid->Fahrzeug) == 0 je Komponente
     (jede Gerade tritt gleich oft in den Koerper ein wie aus). Prueft Vorzeichen und Vollstaendigkeit der Linksuche.
  A4 Gegen den Facettenpfad (cd_facetten.csv, Momentanwert zum selben t): |dCz| <= 0,05, |dCd| <= 0,08 fuer P1;
     Band (Fahrzeugzellen z-Index < 4) gegen cd/cz_druck_band. Grenzen VOR dem ersten Lauf festgelegt (Planungsagent).
     ★ BERICHTIGT nach dem ersten Lauf (17.09., offen deklariert): der Plan nannte E1 als Vergleichsgroesse. E1 ist
     UNPROJIZIERT und enthaelt den tangentialen Bounce-Back-Impulsfluss 2w*3(c.u)c (p375_e 501 ms: Cd +11,6); der
     Facettenpfad projiziert F auf die Facettennormale (setup.cpp:4247-4264) und entfernt ihn. Der normalprojizierte
     Gleichgewichtsanteil ist an der GITTERPARALLELEN ebenen Wand exakt P1 (Plan, Antwort 1), am Voxelkoerper nicht (auch
     zellweise normalprojiziert liegt E1 bei Cd +1,24, Pruefagent M1). P1 ist die diskrete Form von Sum p n dA -- dieselbe
     Groesse wie OF13 p*Sf. A4 gilt deshalb fuer P1, getrennt GESAMT / BAND / REST (Pruefagent: das Gesamt bestand nur durch
     Kompensation Rest +0,05 gegen Band -0,07). E1 bleibt Information.
  A5 Freistrom: Median |u| im Einlassstreifen muss si_u treffen (+-3 %), sonst stimmt die u-Umrechnung nicht.
  A6 Bezugsdruck rho_inf aus der Totaldruck-Invariante rho*exp(1,5(|u|^2-u_inf^2)) im Potentialgebiet ueber dem Dach
     (Median, IQR) und Kontrolle Seitenstreifen; beide werden ausgegeben, die Differenz ist die Bezugsunsicherheit.

AUSGABE: export/<lauf>/zonenkraft_<t>ms.npz (Bins roh) und zonenkraft_<t>ms.txt (dieser Bericht).
Aufruf: zonen_kraft.py <export/lauf> <t_ms, z.B. 000501>
"""
import sys, os, re
import numpy as np

TYPE_FAHRZEUG, FLUID_WERTE, ERLAUBT = 0x41, (0x00, 0x03), {0x00, 0x01, 0x02, 0x03, 0x41}
SI_U, SI_RHO, A_REF = 30.0, 1.225, 1.85
Q_INF = 0.5 * SI_RHO * SI_U**2
# Bins -- reine Auswerteraster, keine Modellkonstanten. x: 100-mm-Streifen; z: Grenzen an der Kontaktband-Hoehe
# (15 mm = 4 Zellen, Solver-Band) und sonst grob nach Bauhoehe; Orientierung nach der Zellnormale (ORI unten).
XB0, XBW, NXB = -0.2, 0.1, 48
ZKANTEN = np.array([0.015, 0.06, 0.12, 0.20, 0.40, 0.70, 1.00, 1.30])      # 9 Bins: [0,.015) ... [1.30, inf)
NZB = len(ZKANTEN) + 1
ORI = ("oben", "seite", "unten")    # nach der ZELLNORMALE der Fluidzelle, m = Sum 2wc ueber ihre Fahrzeuglinks (zeigt ins Fahrzeug):
                                    # m_z < 0 Koerper darunter (Oberseite), m_z > 0 Unterseite, m_z == 0 Seite.
                                    # ★ Pruefagent 17.09. H1: die erste Fassung sortierte je Link nach sign(c_z) -- an einer
                                    # senkrechten Wand fielen die Diagonallinks halb nach oben, halb nach unten; die projizierte
                                    # Flaeche "oben" war 8,63 gegen OF13 7,05 (Vielfache von A_ref, nicht m2; mit Zellnormale 7,08).
NBIN = NXB * NZB * 3 * 2            # letzter Faktor: Band-Flag (Fahrzeugzelle z-Index < 4)

def richtungen():
    C, W = [], []
    for cx in (-1, 0, 1):
        for cy in (-1, 0, 1):
            for cz in (-1, 0, 1):
                s = abs(cx) + abs(cy) + abs(cz)
                if s == 1: C.append((cx, cy, cz)); W.append(1.0/18.0)
                elif s == 2: C.append((cx, cy, cz)); W.append(1.0/36.0)
    return np.array(C, dtype=np.int64), np.array(W)

def kopf(pfad):
    with open(pfad, "rb") as f: roh = f.read(1024)
    txt = roh.decode("ascii", "replace")
    i = txt.index("VECTORS u float\n") + len("VECTORS u float\n")
    d = {}
    for zeile in txt[:i].split("\n"):
        t = zeile.split()
        if not t: continue
        if t[0] == "DIMENSIONS": d["dims"] = tuple(int(v) for v in t[1:4])
        elif t[0] == "ORIGIN": d["orig"] = tuple(float(v) for v in t[1:4])
        elif t[0] == "SPACING": d["spac"] = tuple(float(v) for v in t[1:4])
    Nx, Ny, Nz = d["dims"]; n = Nx*Ny*Nz
    s1 = b"\nSCALARS rho float 1\nLOOKUP_TABLE default\n"; s2 = b"\nSCALARS flags unsigned_char 1\nLOOKUP_TABLE default\n"
    d["off_u"] = i; d["off_rho"] = i + n*12 + len(s1); d["off_flags"] = d["off_rho"] + n*4 + len(s2)
    if d["off_flags"] + n != os.path.getsize(pfad): raise SystemExit("A1 FEHLER: Layout passt nicht zur Dateigroesse")
    with open(pfad, "rb") as f:
        f.seek(i + n*12); assert f.read(len(s1)) == s1, "A1 FEHLER: rho-Marker"
        f.seek(d["off_rho"] + n*4); assert f.read(len(s2)) == s2, "A1 FEHLER: flags-Marker"
    return d

def protokoll(lauf_dir):
    name = os.path.basename(os.path.normpath(lauf_dir))
    pfad = os.path.join(lauf_dir, "..", "..", "logs", name + ".log")
    txt = open(pfad, errors="replace").read()
    flach = re.sub(r"\x1b\[[0-9;]*m", "", txt); flach = re.sub(r"\|\s*\n\|\s*", " ", flach); flach = re.sub(r"\s+", " ", flach)
    m = re.search(r"Freistrom im Speicherwort: u_lat = ([0-9.]+)", flach)
    if not m: raise SystemExit(f"A1 FEHLER: u_lat nicht im Log {pfad}")
    u_lat = float(m.group(1))
    b = re.search(r"Band-Census 0x41: (\d+) von \d+ Zellen", flach)
    zb = re.search(r"KRAFT-ZBAND aktiv: unterste (\d+) Zellen", flach)
    v = re.search(r"Koerper um ([0-9.]+) mm nach \+y versetzt", flach)
    return dict(log=os.path.abspath(pfad), u_lat=u_lat, band_census=int(b.group(1)) if b else None,
                zband=int(zb.group(1)) if zb else None, y_versatz_mm=float(v.group(1)) if v else 0.0)

def facetten_zeilen(lauf_dir):
    p = os.path.join(lauf_dir, "facetten_histogramme.csv")
    if not os.path.exists(p): return None
    n = 0
    with open(p, "rb") as f:
        for blk in iter(lambda: f.read(1 << 24), b""): n += blk.count(b"\n")
    with open(p) as f: kopfz = sum(1 for _ in range(1) if f.readline().startswith("#"))
    return n - kopfz

def main():
    lauf_dir, tms = sys.argv[1], sys.argv[2]
    vtk = os.path.join(lauf_dir, f"feld_nah_{tms}ms.vtk")
    d = kopf(vtk); Nx, Ny, Nz = d["dims"]; ox, oy, oz = d["orig"]; dx = d["spac"][0]
    P = protokoll(lauf_dir); u_lat = P["u_lat"]
    FKRAFT = SI_RHO * dx*dx * (SI_U/u_lat)**2
    KC = FKRAFT / (Q_INF * A_REF)
    print(f"# zonen_kraft.py  {vtk}\n# Laufprotokoll {P['log']}")
    print(f"A1 Layout ok: {Nx}x{Ny}x{Nz}, dx {dx*1e3:.4f} mm, ORIGIN {ox:+.5f} {oy:+.5f} {oz:+.5f}")
    print(f"A1 u_lat {u_lat} (Log) -> 1 Kraft-Gittereinheit = {FKRAFT:.5f} N = {KC:.4e} in C; Y-Versatz {P['y_versatz_mm']} mm (wirkt auf den Koerper, nicht auf ORIGIN)")
    FL = np.memmap(vtk, dtype=np.uint8, mode="r", offset=d["off_flags"], shape=(Nz, Ny, Nx))
    RH = np.memmap(vtk, dtype=">f4", mode="r", offset=d["off_rho"], shape=(Nz, Ny, Nx))
    UU = np.memmap(vtk, dtype=">f4", mode="r", offset=d["off_u"], shape=(Nz, Ny, Nx, 3))

    # --- A2 Flag-Zensus je Ebene + Fahrzeug-Bounding-Box
    werte = set(); kmin = imin = jmin = 10**9; kmax = imax = jmax = -1; n_band = 0
    for k in range(Nz):
        e = np.asarray(FL[k]); u_ = np.unique(e); werte |= set(int(v) for v in u_)
        if k == 0 and set(int(v) for v in u_) != {0x01}: raise SystemExit(f"A2 FEHLER: z=0 traegt {sorted(u_)} statt nur 0x01")
        m = e == TYPE_FAHRZEUG
        if m.any():
            jj, ii = np.nonzero(m); kmin = min(kmin, k); kmax = k
            imin = min(imin, ii.min()); imax = max(imax, ii.max()); jmin = min(jmin, jj.min()); jmax = max(jmax, jj.max())
            if k < (P["zband"] or 4): n_band += int(m.sum())
    if not werte <= ERLAUBT: raise SystemExit(f"A2 FEHLER: unerwartete Flags {sorted(werte - ERLAUBT)}")
    print(f"A2 Flags ok: {sorted(hex(v) for v in werte)}; Fahrzeug-Box i {imin}..{imax} j {jmin}..{jmax} k {kmin}..{kmax}")
    if P["band_census"] is not None:
        st = "ok" if n_band == P["band_census"] else "ABWEICHUNG"
        print(f"A2 0x41 unter z-Index {P['zband']}: {n_band} (Log {P['band_census']}) {st}")
        if n_band != P["band_census"]: raise SystemExit("A2 FEHLER: Band-Census passt nicht -- Maske oder Datei falsch")
    if imin < 2 or jmin < 2 or imax > Nx-3 or jmax > Ny-3 or kmax > Nz-3: raise SystemExit("A2 FEHLER: Fahrzeug beruehrt den Kastenrand")
    i0, i1, j0, j1, k1 = imin-2, imax+3, jmin-2, jmax+3, kmax+3
    # Unter z=0 eine kuenstliche Strassenlage (k0 = -1): so liegen die Strasse->Reifen-Uebergaenge bei z=0/1 im
    # Innenbereich der Linksuche und die Schliessungsprobe A3 bleibt exakt. Die Lage beruehrt kein Fahrzeug.
    k0 = -1
    fl = np.concatenate([np.full((1, j1-j0, i1-i0), 0x01, np.uint8), np.asarray(FL[0:k1, j0:j1, i0:i1])])
    rho = np.concatenate([np.ones((1, j1-j0, i1-i0)), np.asarray(RH[0:k1, j0:j1, i0:i1]).astype(np.float64)])
    u = np.concatenate([np.zeros((1, j1-j0, i1-i0, 3)), np.asarray(UU[0:k1, j0:j1, i0:i1]).astype(np.float64)]) * (u_lat / SI_U)
    nz, ny, nx = fl.shape
    fahr = fl == TYPE_FAHRZEUG
    fluid = (fl == FLUID_WERTE[0]) | (fl == FLUID_WERTE[1])
    nichtfahr = ~fahr
    print(f"   TYPE_E-Zellen in der Box: {int((fl == 0x02).sum())} (Soll 0 -- sonst Randnaehe)")
    print(f"   Box {nx}x{ny}x{nz} = {fl.size/1e6:.1f} Mio Zellen, Fahrzeugzellen {int(fahr.sum())}")

    C, W = richtungen()
    xs = ox + (i0 + np.arange(nx)) * dx; ys = oy + (j0 + np.arange(ny)) * dx; zs = oz + (k0 + np.arange(nz)) * dx
    jy0 = int(round((0.0 - oy) / dx)) - j0     # Fahrzeugzellen am y-Index von y = 0 (nur OHNE Versatz eine Membran, p375_b: Index 354)
    nlinks = np.zeros(NBIN); G = np.zeros((NBIN, 3)); P1 = np.zeros((NBIN, 3)); E1 = np.zeros((NBIN, 3))
    geo_fluid = np.zeros(3); geo_rest = np.zeros(3); n_rest_fluid = 0
    membran = np.zeros(3); membran_n = 0
    hat_link = np.zeros(fl.shape, dtype=bool)
    usq = (u*u).sum(axis=3)
    # Durchgang 1: Zellnormale m_z = Sum 2 w c_z je Fluidzelle (nur z-Komponente wird fuer die Orientierung gebraucht)
    mz = np.zeros(fl.shape, dtype=np.float32)
    for c, w in zip(C, W):
        cx, cy, cz = (int(v) for v in c)
        if cz == 0: continue
        sq = (slice(1, nz-1), slice(1, ny-1), slice(1, nx-1))
        sz = (slice(1+cz, nz-1+cz), slice(1+cy, ny-1+cy), slice(1+cx, nx-1+cx))
        mz[sq] += np.where(fluid[sq] & fahr[sz], np.float32(2*w*cz), np.float32(0))
    for c, w in zip(C, W):
        cx, cy, cz = (int(v) for v in c)
        # Quelle q (Fluid) im Innenbereich [1:-1]^3, Ziel = q + c
        sq = (slice(1, nz-1), slice(1, ny-1), slice(1, nx-1))
        sz = (slice(1+cz, nz-1+cz), slice(1+cy, ny-1+cy), slice(1+cx, nx-1+cx))
        ziel_fahr = fahr[sz]
        link = fluid[sq] & ziel_fahr
        rest = nichtfahr[sq] & ~fluid[sq] & ziel_fahr          # Strasse/TYPE_E -> Fahrzeug, nur fuer A3
        geo_rest += 2*w*int(rest.sum())*c
        kk, jj, ii = np.nonzero(link); kk += 1; jj += 1; ii += 1
        hat_link[kk, jj, ii] = True
        r = rho[kk, jj, ii]; cu = u[kk, jj, ii] @ c.astype(np.float64); uq = usq[kk, jj, ii]
        feq_w = w*r*(1.0 + 3.0*cu + 4.5*cu*cu - 1.5*uq) - w
        xb = np.clip(np.floor((xs[ii] + 0.5*cx*dx - XB0)/XBW).astype(np.int64), 0, NXB-1)
        zb = np.searchsorted(ZKANTEN, zs[kk] + 0.5*cz*dx, side="right")
        ori = (1 + np.sign(np.round(mz[kk, jj, ii].astype(np.float64)*36.0))).astype(np.int64)   # *36: exakte Ganzzahl, kein Float-Rest
        band = ((k0 + kk + cz) < (P["zband"] or 4)).astype(np.int64)
        idx = ((xb*NZB + zb)*3 + ori)*2 + band
        # je Bin Skalar-Summen, der Kraftvektor ist Skalar * c (c ist je Richtung konstant)
        cnt = np.bincount(idx, minlength=NBIN).astype(np.float64); cc = c.astype(np.float64)
        nlinks += cnt
        G += (cnt*2*w)[:, None]*cc
        P1 += np.bincount(idx, weights=2*w*(r - 1.0), minlength=NBIN)[:, None]*cc
        E1 += np.bincount(idx, weights=2*feq_w, minlength=NBIN)[:, None]*cc
        geo_fluid += 2*w*kk.size*c
        mm = (jj + cy) == jy0
        if mm.any(): membran += (2*w*(r[mm]-1.0)).sum()*c; membran_n += int(mm.sum())

    # --- A3 Schliessung
    summe = geo_fluid + geo_rest
    print(f"A3 Schliessung Sum 2wc: Fluid->Fz {geo_fluid.round(6)}, Rest->Fz {geo_rest.round(6)}, Summe {summe.round(9)} "
          + ("ok" if np.allclose(summe, 0.0, atol=1e-9) else "FEHLER"))
    if not np.allclose(summe, 0.0, atol=1e-9): raise SystemExit("A3 FEHLER: Linksuche nicht geschlossen")
    if not np.allclose(G.sum(axis=0), geo_fluid, atol=1e-6): raise SystemExit("A3 FEHLER: Binsumme G != Linksumme")
    nfac = int(hat_link.sum()); soll = facetten_zeilen(lauf_dir)
    print(f"A2 Facettenzellen (Fluid mit >=1 Fahrzeuglink): {nfac}; facetten_histogramme.csv: {soll} "
          + ("ok" if soll is None or soll == nfac else "ABWEICHUNG (nur Hinweis: CSV kann Randlagen ausschliessen)"))
    print(f"   Links gesamt {int(nlinks.sum())}")

    # --- A5 Freistrom und A6 Bezugsdruck (aus dem vollen Kasten, ausserhalb der Box)
    def region(x0, x1, y0, y1, z0, z1, betrag_y=True):
        ia, ib = int(np.ceil((x0-ox)/dx)), int(np.floor((x1-ox)/dx))+1
        ka, kb = int(np.ceil((z0-oz)/dx)), int(np.floor((z1-oz)/dx))+1
        out = []
        for jsel in ((y0, y1), (-y1, -y0)) if betrag_y else ((y0, y1),):
            ja, jb = int(np.ceil((jsel[0]-oy)/dx)), int(np.floor((jsel[1]-oy)/dx))+1
            f_ = np.asarray(FL[ka:kb:2, ja:jb:2, ia:ib:2]); r_ = np.asarray(RH[ka:kb:2, ja:jb:2, ia:ib:2]).astype(np.float64)
            v_ = np.asarray(UU[ka:kb:2, ja:jb:2, ia:ib:2]).astype(np.float64)
            m_ = (f_ == 0x00)
            out.append((r_[m_], v_[m_]))
        return np.concatenate([o[0] for o in out]), np.concatenate([o[1] for o in out])
    r_e, v_e = region(ox + 4*dx, ox + 20*dx, 1.15, 1.30, 0.3, 1.5)   # seitlich vor dem Fahrzeug, ausserhalb des Staugebiets
    um = np.median(np.linalg.norm(v_e, axis=1))
    print(f"A5 Einlass-Seitenstreifen (x < ORIGIN+20 dx, |y| 1,15-1,30): Median |u| = {um:.3f} m/s (si_u {SI_U}) " + ("ok" if abs(um/SI_U-1) < 0.03 else "WARNUNG"))
    def rho_inf(r_, v_):
        ul = v_*(u_lat/SI_U); q = (ul*ul).sum(axis=1)
        ri = r_*np.exp(1.5*(q - u_lat*u_lat))
        return float(np.median(ri)), float(np.percentile(ri, 25)), float(np.percentile(ri, 75)), ri.size
    r1, v1 = region(0.3, 3.0, 0.0, 0.9, 1.50, 1.80, betrag_y=False)
    ri_dach = rho_inf(r1, v1)
    r3, v3 = region(0.3, 3.0, -0.03, 0.03, 1.50, 1.80, betrag_y=False)   # dieselbe Region wie der OF13-Bezug (of13_slab.npy, |y|<0,03)
    ri_mitte = rho_inf(r3, v3)
    r2, v2 = region(0.3, 3.0, 1.15, 1.30, 0.30, 1.20)
    ri_seite = rho_inf(r2, v2)
    cpf = 2.0/(3.0*u_lat*u_lat)
    print(f"A6 rho_inf ueber dem Dach   : Median {ri_dach[0]:.7f} (IQR {ri_dach[1]:.7f}..{ri_dach[2]:.7f}, n {ri_dach[3]}) = cp_ref {(ri_dach[0]-1)*cpf:+.4f}")
    print(f"A6 rho_inf Seitenstreifen   : Median {ri_seite[0]:.7f} (IQR {ri_seite[1]:.7f}..{ri_seite[2]:.7f}, n {ri_seite[3]}) = cp_ref {(ri_seite[0]-1)*cpf:+.4f}")
    print(f"A6 rho_inf ueber dem Dach |y|<0,03 (Bezug fuer den OF13-Vergleich): Median {ri_mitte[0]:.7f} (n {ri_mitte[3]}) = cp_ref {(ri_mitte[0]-1)*cpf:+.4f}")
    print(f"   Bezugsunsicherheit Dach - Seite: d cp_ref = {(ri_dach[0]-ri_seite[0])*cpf:+.4f}")
    print("   HINWEIS (Pruefagent M6): die Regionen liegen 7-96 Zellen unter/neben dem Nahfeldrand -- 'Potentialgebiet' ist nicht belegt.")

    # --- A4 Abnahme gegen den Facettenpfad
    tsoll = int(tms) / 1000.0
    zeilen = [l for l in open(os.path.join(lauf_dir, "cd_facetten.csv")) if not l.startswith("#")]
    kopfz = zeilen[0].strip().split(","); daten = np.array([[float(v) for v in l.split(",")] for l in zeilen[1:]])
    it = int(np.argmin(np.abs(daten[:, 0] - tsoll))); fz = dict(zip(kopfz, daten[it]))
    if abs(fz["time_s"] - tsoll) > 5e-4: raise SystemExit(f"A4 FEHLER: kein cd_facetten-Sample bei t={tsoll} (naechstes {fz['time_s']})")
    bandm = np.zeros(NBIN, dtype=bool); bandm[1::2] = True
    E1g, P1g = E1.sum(axis=0)*KC, P1.sum(axis=0)*KC
    E1b = E1[bandm].sum(axis=0)*KC
    P1b = P1[bandm].sum(axis=0)*KC
    dcz, dcd = P1g[2] - fz["cz_druck"], P1g[0] - fz["cd_druck"]
    ok4 = abs(dcz) <= 0.05 and abs(dcd) <= 0.08
    dczb, dcdb = P1b[2] - fz["cz_druck_band"], P1b[0] - fz["cd_druck_band"]
    dczr, dcdr = (P1g[2]-P1b[2]) - fz["cz_druck_rest"], (P1g[0]-P1b[0]) - fz["cd_druck_rest"]
    ok4b = abs(dczb) <= 0.05 and abs(dcdb) <= 0.08
    ok4r = abs(dczr) <= 0.05 and abs(dcdr) <= 0.08
    print(f"A4 Facettenpfad t={fz['time_s']:.5f}: cd_druck {fz['cd_druck']:+.4f} cz_druck {fz['cz_druck']:+.4f} | band cd {fz['cd_druck_band']:+.4f} cz {fz['cz_druck_band']:+.4f}")
    print(f"   E1 (Gleichgew.-MEA unprojiziert, nur Info): Cd {E1g[0]:+.4f} Cz {E1g[2]:+.4f} Cy {E1g[1]:+.4f} | Band Cd {E1b[0]:+.4f} Cz {E1b[2]:+.4f}")
    print(f"A4 P1 (Druckproxy, Bezug 1): Cd {P1g[0]:+.4f} Cz {P1g[2]:+.4f} Cy {P1g[1]:+.4f} | Band Cd {P1b[0]:+.4f} Cz {P1b[2]:+.4f}")
    print(f"A4 Rest ohne Band: P1 Cd {P1g[0]-P1b[0]:+.4f} Cz {P1g[2]-P1b[2]:+.4f} | Facettenpfad cd_druck_rest {fz['cd_druck_rest']:+.4f} cz_druck_rest {fz['cz_druck_rest']:+.4f}")
    print(f"   Bezugsempfindlichkeit gesamt (offene Kontaktflaeche): dCz/dcp_ref = {-G.sum(axis=0)[2]*KC*1.5*u_lat*u_lat:+.4f}")
    print(f"A4 GESAMT dCz {dcz:+.4f} dCd {dcd:+.4f} -> " + ("bestanden" if ok4 else "NICHT bestanden"))
    print(f"A4 BAND   dCz {dczb:+.4f} dCd {dcdb:+.4f} -> " + ("bestanden" if ok4b else "NICHT bestanden"))
    print(f"A4 REST   dCz {dczr:+.4f} dCd {dcdr:+.4f} -> " + ("bestanden" if ok4r else "NICHT bestanden"))
    print("   (Grenzen |dCz| <= 0,05, |dCd| <= 0,08. P1 ist die diskrete Form von Sum p n dA; der Facettenpfad ist normalprojizierter"
          " MEA -- an der gitterparallelen ebenen Wand identisch, am Voxelkoerper nicht. Band- und Restabweichung sind nicht"
          " lokalisiert und gehoeren als systematischer Anteil in jeden Zonen-Fehlerbalken.)")
    print(f"   Links an Fahrzeugzellen mit y-Index {jy0+j0} (Membran nur ohne Versatz): {membran_n} Links, P1 Cz {membran[2]*KC:+.5f} Cd {membran[0]*KC:+.5f}")

    # --- Geometrie: Bodenfreiheit der Mittellinie (|y| < 0,1 m), ohne z-Index <= 1
    ym = np.abs(ys - P["y_versatz_mm"]*1e-3) < 0.1
    hx = []
    for a in np.arange(XB0, XB0 + NXB*XBW, XBW):
        im = (xs >= a) & (xs < a + XBW)
        sub = fahr[2:, :, :][:, ym][:, :, im]
        ks = np.nonzero(sub.any(axis=(1, 2)))[0]
        hx.append((k0 + 2 + ks[0] - 1)*dx if ks.size else np.nan)   # Unterkante Zelle k bei (k-0,5)dx, Strassenwand bei 0,5dx
    out = os.path.join(lauf_dir, f"zonenkraft_{tms}ms.npz")
    np.savez_compressed(out, G=G, P1=P1, E1=E1, n=nlinks, KC=KC, u_lat=u_lat, dx=dx, rho_inf_dach=np.array(ri_dach),
                        rho_inf_seite=np.array(ri_seite), rho_inf_mitte=np.array(ri_mitte), abnahme_a4_band=ok4b, abnahme_a4_rest=ok4r,
                        a4_rest_dcz=dczr, a4_band_dcz=dczb, orientierung="zellnormale", XB0=XB0, XBW=XBW, NXB=NXB, ZKANTEN=ZKANTEN, h_mitte=np.array(hx),
                        fac_cd=fz["cd_druck"], fac_cz=fz["cz_druck"], fac_t=fz["time_s"], abnahme_a4=ok4, cp_faktor=cpf,
                        y_versatz_mm=P["y_versatz_mm"])
    print(f"geschrieben: {out}")

if __name__ == "__main__":
    main()
