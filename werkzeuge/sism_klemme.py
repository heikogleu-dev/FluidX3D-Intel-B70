#!/usr/bin/env python3
"""sism_klemme.py -- Wie gross waere der SISM-Klemmanteil im INNENFELD, wenn zellweises SISM gebaut wuerde?

Die Frage entscheidet, ob der Vollfeldarm ueberhaupt gebaut wird (Vorpruefung 08.09.2026):
Die Klemme fmax(0, |S| - |<S>|) setzt nu_t EXAKT 0, wo Sbar > |S|. Passiert das im Innenfeld
flaechig, ist zellweises SISM ein WANDFREI-Arm -- und WANDFREI divergierte am 8-mm-Fahrzeug
nach 175 Schritten auf EINER Wandschicht.

Gerechnet wird mit dem KERNEL-IDENTISCHEN Stencil (kernel.cpp, Kernel sgs_fdwand):
    g[i][a] = 0.5*(u_i(n+e_a) - u_i(n-e_a)),  Solid-Nachbarn zaehlen als u = 0
    S_ia    = 0.5*(g[i][a] + g[a][i])
    |S|     = sqrt(2 * sum_ia S_ia^2)
Solid-Test wie im Kernel: (flags & TYPE_BO) == TYPE_S mit TYPE_BO = 0x03 (lbm.cpp:1421).
<S> ist das ZEITMITTEL der sechs unabhaengigen S-Komponenten, Sbar = |<S>| ueber dieselbe
6-Komponenten-Norm wie sism_norm6() im Host und sbar im Kernel.

RICHTUNG DES ERGEBNISSES: das Zeitmittel ueber das ganze Fenster liefert das KLEINSTE Sbar
(gemessener T-Trend 07.09.: kleineres T -> groesseres Sbar). Der hier bestimmte Klemmanteil
ist damit eine UNTERE SCHRANKE. Kommt schon die gross heraus, ist der Vollfeldarm tot.

Aufruf: sism_klemme.py <export/LAUF> [t_min_ms]     (Default 201 = Messfenster)
"""
import sys, os, glob, re
import numpy as np

TYPE_BO, TYPE_S = 0x03, 0x01

def kopf(pfad):
    with open(pfad, "rb") as f: roh = f.read(1024)
    txt = roh.decode("ascii", "replace")
    i = txt.index("VECTORS u float\n") + len("VECTORS u float\n")
    d = {}
    for zeile in txt[:i].split("\n"):
        t = zeile.split()
        if not t: continue
        if t[0] == "DIMENSIONS": d["dims"] = tuple(int(v) for v in t[1:4])
        elif t[0] == "ORIGIN":   d["orig"] = tuple(float(v) for v in t[1:4])
        elif t[0] == "SPACING":  d["spac"] = tuple(float(v) for v in t[1:4])
    d["off_u"] = i
    Nx, Ny, Nz = d["dims"]; np_ = Nx*Ny*Nz
    s1 = b"\nSCALARS rho float 1\nLOOKUP_TABLE default\n"
    s2 = b"\nSCALARS flags unsigned_char 1\nLOOKUP_TABLE default\n"
    d["off_rho"]   = i + np_*12 + len(s1)
    d["off_flags"] = d["off_rho"] + np_*4 + len(s2)
    if d["off_flags"] + np_ != os.path.getsize(pfad):
        raise SystemExit(f"Layout passt nicht bei {pfad}")
    return d

def lies_u(pfad, d):
    """u als (3, Nz, Ny, Nx) float32 -- im VTK interleaved (x schnellste Achse)."""
    Nx, Ny, Nz = d["dims"]; n = Nx*Ny*Nz
    a = np.memmap(pfad, dtype=">f4", mode="r", offset=d["off_u"], shape=(n, 3))
    u = np.asarray(a, dtype=np.float32).T.reshape(3, Nz, Ny, Nx)
    return u

def lies_flags(pfad, d):
    Nx, Ny, Nz = d["dims"]; n = Nx*Ny*Nz
    a = np.memmap(pfad, dtype=np.uint8, mode="r", offset=d["off_flags"], shape=(n,))
    return np.asarray(a).reshape(Nz, Ny, Nx)

def S_komponenten(u, solid):
    """Die sechs unabhaengigen S-Komponenten, Kernel-identisch. Rueckgabe je (Nz,Ny,Nx),
    Randzellen (eine Lage) sind NaN -- dort fehlt ein Nachbar."""
    g = np.empty((3, 3) + u.shape[1:], dtype=np.float32); g[:] = np.nan
    # Achse a: 0=x (letzte Achse), 1=y, 2=z (erste Achse)
    for a, ax in enumerate((3, 2, 1)):
        up = np.roll(u, -1, axis=ax); um = np.roll(u, 1, axis=ax)
        sp = np.roll(solid, -1, axis=ax-1); sm = np.roll(solid, 1, axis=ax-1)
        for i in range(3):
            a_ = np.where(sp, np.float32(0.0), up[i])
            b_ = np.where(sm, np.float32(0.0), um[i])
            g[i, a] = 0.5*(a_-b_)
    # Rand ungueltig machen (roll wickelt periodisch)
    for ax in (1, 2, 3):
        sl0 = [slice(None)]*4; sl0[ax] = 0
        sl1 = [slice(None)]*4; sl1[ax] = -1
        g[(slice(None),)+tuple(sl0)] = np.nan
        g[(slice(None),)+tuple(sl1)] = np.nan
    S = np.stack([g[0,0], g[1,1], g[2,2],
                  0.5*(g[0,1]+g[1,0]), 0.5*(g[0,2]+g[2,0]), 0.5*(g[1,2]+g[2,1])])
    return S

def norm6(S):
    """|S| = sqrt(2*(S0^2+S1^2+S2^2 + 2*(S3^2+S4^2+S5^2))) -- wie sism_norm6() und kernel sbar."""
    return np.sqrt(2.0*(S[0]**2 + S[1]**2 + S[2]**2 + 2.0*(S[3]**2 + S[4]**2 + S[5]**2)))

def main():
    lauf = sys.argv[1].rstrip("/")
    t_min = float(sys.argv[2]) if len(sys.argv) > 2 else 201.0
    dat = sorted(glob.glob(os.path.join(lauf, "feld_nah_*ms.vtk")))
    paare = []
    for p in dat:
        m = re.search(r"feld_nah_(\d+)ms\.vtk$", p)
        if m and float(m.group(1)) >= t_min: paare.append((float(m.group(1)), p))
    if len(paare) < 3: raise SystemExit(f"nur {len(paare)} Felder ab {t_min} ms -- zu wenig (CFD_VTK_DT zu grob?)")
    print(f"{os.path.basename(lauf)}: {len(paare)} Felder ab {t_min} ms: {[int(t) for t,_ in paare]}")

    d = kopf(paare[0][1]); Nx, Ny, Nz = d["dims"]; ox, oy, oz = d["orig"]; dx = d["spac"][0]
    flags = lies_flags(paare[0][1], d)
    solid = (flags & TYPE_BO) == TYPE_S
    fluid = (flags & TYPE_BO) == 0                      # weder Solid noch Equilibrium
    print(f"  Gitter {Nx}x{Ny}x{Nz}, dx = {dx*1000:.1f} mm, {fluid.sum()/1e6:.1f} Mio Fluidzellen")

    # --- Durchgang 1: <S> akkumulieren, dazu Split-Half (A = erste Haelfte, B = zweite)
    n = len(paare); h = n//2
    Ssum = np.zeros((6, Nz, Ny, Nx), dtype=np.float64)
    SsumA = np.zeros_like(Ssum); SsumB = np.zeros_like(Ssum)
    absS = []                                            # |S| je Zeitpunkt, spaeter gebraucht
    for k, (t, p) in enumerate(paare):
        u = lies_u(p, d)
        S = S_komponenten(u, solid)
        Ssum += np.nan_to_num(S, nan=0.0)
        if k < h: SsumA += np.nan_to_num(S, nan=0.0)
        else:     SsumB += np.nan_to_num(S, nan=0.0)
        absS.append(norm6(S).astype(np.float32))
        del u, S
        print(f"    gelesen t = {int(t)} ms", flush=True)
    Sm  = (Ssum/n).astype(np.float32)
    SmA = (SsumA/h).astype(np.float32); SmB = (SsumB/(n-h)).astype(np.float32)
    del Ssum, SsumA, SsumB
    sbar = norm6(Sm)
    # Rauschbias: |<S>|^2 ist bei endlicher Stichprobe zu gross. <S>_A : <S>_B ist erwartungstreu.
    kreuz = 2.0*(SmA[0]*SmB[0] + SmA[1]*SmB[1] + SmA[2]*SmB[2]
                 + 2.0*(SmA[3]*SmB[3] + SmA[4]*SmB[4] + SmA[5]*SmB[5]))
    sbar_ehrlich = np.sqrt(np.maximum(kreuz, 0.0)).astype(np.float32)
    del SmA, SmB, kreuz

    # --- Regionen (Fahrzeug: Nase bei x~0, Heck ~4,44 m; Dach z~1,2 m)
    x = ox + np.arange(Nx)*dx; z = oz + np.arange(Nz)*dx
    X = x[None, None, :]; Z = z[:, None, None]
    reg = {
        "gesamt":          fluid,
        "Nachlauf x>4,5m": fluid & (X > 4.5),
        "Dachschicht":     fluid & (X > 2.0) & (X < 3.7) & (Z > 1.0) & (Z < 1.6),
        "Unterboden":      fluid & (X > 0.5) & (X < 4.4) & (Z < 0.15),
        "Anstroemung":     fluid & (X < -0.5),
    }

    # --- Durchgang 2: Klemmanteil = Anteil (Zelle,Zeit) mit |S| < Sbar
    print(f"\n  KLEMMANTEIL (Anteil der Zelle-Zeit-Paare mit |S| < Sbar; nu_t waere dort EXAKT 0)")
    print(f"  {'Region':18s} {'Zellen':>10s} {'roh':>8s} {'entrauscht':>11s} {'Sbar/<|S|>':>11s}")
    for name, maske in reg.items():
        m = maske & np.isfinite(sbar)
        nz = int(m.sum())
        if nz == 0: continue
        klemm_roh = 0.0; klemm_ehr = 0.0; summe_abs = 0.0
        for A in absS:
            gut = m & np.isfinite(A)
            klemm_roh += float((A[gut] < sbar[gut]).sum())
            klemm_ehr += float((A[gut] < sbar_ehrlich[gut]).sum())
            summe_abs += float(A[gut].sum())
        ges = nz*len(absS)
        mabs = summe_abs/ges
        verh = float(np.nanmean(sbar[m]))/mabs if mabs > 0 else float("nan")
        print(f"  {name:18s} {nz:10d} {100.0*klemm_roh/ges:7.1f}% {100.0*klemm_ehr/ges:10.1f}% {verh:11.3f}")

    # --- Glattheit von <S>: entscheidet ueber die 4-mm-Variante "Feld vergroebern"
    s4 = sbar[::4, ::4, ::4]
    from numpy.lib.stride_tricks import sliding_window_view
    gut = np.isfinite(sbar[:-4, :-4, :-4]) & np.isfinite(sbar[4:, 4:, 4:])
    a1 = sbar[:-4, :-4, :-4][gut]; a2 = sbar[4:, 4:, 4:][gut]
    if a1.size > 1000:
        r = float(np.corrcoef(a1, a2)[0, 1])
        print(f"\n  <S>-Korrelation ueber Stride 4: r = {r:.3f}  "
              f"({'glatt -> Vergroeberung tragbar' if r > 0.9 else 'zu rau -> Vergroeberung verfaelscht'})")

    # --- Lagenprofil ab Wand (nur wo Solid in der Naehe): Klemmanteil je Wandabstand
    print(f"\n  LAGENPROFIL (Klemmanteil je Abstand zur naechsten Solidzelle, entrauscht)")
    # Wandabstand ohne scipy: iterative 6er-Dilatation der Solidmaske (Lage 1..8 reicht).
    lage_von = np.zeros(solid.shape, dtype=np.uint8)   # 0 = Solid oder >8 entfernt
    rand = solid.copy()
    for lage in range(1, 9):
        nb = np.zeros_like(rand)
        for ax in (0, 1, 2):
            nb |= np.roll(rand, 1, axis=ax); nb |= np.roll(rand, -1, axis=ax)
        neu_ = nb & ~rand & (lage_von == 0) & ~solid
        lage_von[neu_] = lage
        rand = rand | neu_
    for lage in range(1, 9):
        m = fluid & (lage_von == lage) & np.isfinite(sbar_ehrlich)
        nz = int(m.sum())
        if nz < 1000: continue
        kl = 0.0
        for A in absS:
            gut = m & np.isfinite(A)
            kl += float((A[gut] < sbar_ehrlich[gut]).sum())
        print(f"    Lage {lage} ({nz:9d} Zellen): {100.0*kl/(nz*len(absS)):5.1f}%")

if __name__ == "__main__":
    main()
