#!/usr/bin/env python3
"""d3q27_zensus.py -- Stufe 1 aus PLAN-D3Q27-WANDZELLEN.md: heben die 8 Eckrichtungen den Tangentialrang der Facettenzellen?

FASSUNG 2 (17.09.2026, nach Pruefagent): Fassung 1 nahm "generische Normalen" an und meldete fuer die Ein-Link-Klasse
72 gehobene Zellen -- mit den echten Normalen sind es 56 %. Ursache: V3b-Normalen haben exakte Nullkomponenten, und die
Eckgerade der Ein-Link-Zellen ist eine ACHSE, also reicht eine Nullkomponente fuer n senkrecht zur Linkvariation.
Jetzt: echte Normalen aus facetten_normalen.py und der vektorisierte Host-Klassifikator (Nachbau von klassifiziere()
in setup.cpp zensus_statische_klassen, Pruefagent p3). Die geometrische Form (affine Dimension) bleibt als SCHRANKE.

EINGEBAUTE ABNAHMEN (Abbruch bei Abweichung):
  B1 eigene_links (CSV) == D3Q19-Fahrzeuglinks (flags == 0x41), alle Zeilen; fuer aktive Facetten zusaetzlich
     Kernel-Praedikat (flags&3)==1 == Fahrzeuglink in allen 26 Richtungen (keine Strassen-Ecklinks).
  B2 aktive Facetten und Linkhistogramm == Log.
  B3 Host-Klassifikator mit den nachgebauten Normalen trifft den statischen Zensus des Laufs EXAKT:
     Rang 2/1/0, entkoppelt/gekoppelt, Wanderungsmatrix roh -> ALPHA2.
Nur nach B3 gelten die D3Q27-Zahlen als gerechnet; sie bleiben STATISCHE Loesbarkeit (nicht Laufzeit-Rueckfall, nicht
Wirkung -- rekonstruierte Eckpopulationen streamen nicht, PLAN Teil A Punkt 3).
Aufruf: d3q27_zensus.py <export/lauf> [t_ms fuer Flags/u, Vorgabe 000501]
"""
import sys, os, re
import numpy as np

HIER = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HIER)
from zonen_kraft import kopf, protokoll   # dieselben geprueften Leser (A1)

D19 = [(cx, cy, cz) for cx in (-1, 0, 1) for cy in (-1, 0, 1) for cz in (-1, 0, 1) if 1 <= abs(cx)+abs(cy)+abs(cz) <= 2]
ECK = [(cx, cy, cz) for cx in (-1, 1) for cy in (-1, 1) for cz in (-1, 1)]
ALLE = D19 + ECK                     # Bits 0..17 D3Q19, 18..25 Ecken
CV = np.array(ALLE, dtype=np.int64)

def lies_csv(pfad):
    with open(pfad, "rb") as f:
        kopfz = f.readline().decode()
        roh = f.read()
    a = np.fromstring(roh.replace(b"\n", b","), sep=",")
    spalten = kopfz.split("--")[-1].strip().split(",")
    return dict(zip(spalten, a.reshape(-1, len(spalten)).T)), spalten

def log_zensus(pfad):
    t = open(pfad, errors="replace").read()
    t = re.sub(r"\x1b\[[0-9;]*m", "", t); t = re.sub(r"\|\s*\n\|\s*", " ", t); t = re.sub(r"\s+", " ", t)
    L = {}
    m = re.search(r"STATISCHER KLASSENZENSUS \(vor dem ersten Zeitschritt, (\d+) aktive Facetten, (\d+) ohne fid\)", t)
    if m: L["aktiv"], L["ohne_fid"] = int(m.group(1)), int(m.group(2))
    for r in (2, 1, 0):
        m = re.search(rf"Rang {r} \([^)]*\) (\d+) \(", t)
        if m: L[f"rang{r}"] = int(m.group(1))
    W = np.full((3, 3), -1)
    for r in (0, 1, 2):
        m = re.search(rf"roh Rang {r}: ->Rang0: (\d+) ->Rang1: (\d+) ->Rang2: (\d+)", t)
        if m: W[r] = [int(m.group(i)) for i in (1, 2, 3)]
    L["wanderung"] = W
    m = re.search(r"Wandlinks je Facette \(Kernel-Gate\): Mittel [0-9.]+; Verteilung: (.*?) \|", t)
    if m: L["links"] = {int(a): int(b) for a, b in re.findall(r"(\d+): (\d+)", m.group(1))}
    m = re.search(r"entkoppelt (\d+) \| gekoppelt (\d+)", t)
    if m: L["entkoppelt"] = int(m.group(1))
    return L

W19 = np.array([1/18 if abs(a)+abs(b)+abs(c) == 1 else 1/36 for (a, b, c) in D19] + [0.0]*8)
W27 = np.array([2/27 if abs(a)+abs(b)+abs(c) == 1 else (1/54 if abs(a)+abs(b)+abs(c) == 2 else 1/216) for (a, b, c) in ALLE])

def klassifiziere(maske, w, nv, alpha2=True, rel_snn=None, mit_kond=False):
    """Vektorisierter Nachbau von klassifiziere() (setup.cpp zensus_statische_klassen), Schwellen wortgleich:
    Entkopplung Snn<1e-8 oder Kopplung <= 1e-6*Snn*(A11+A22); Rang 0 bei lmax<=1e-12, Rang 1 bei lmin/lmax<1e-9.
    rel_snn: zusaetzlich der Kernel-Waechter Snn < rel*Snn_roh -> Snn = 0 (kernel.cpp ALPHA2-Block, statischer Teil)."""
    c = CV.astype(np.float64); M = maske.astype(np.float64) * w[None, :]
    S0 = M.sum(1); S1 = M @ c
    G = np.einsum("nk,ka,kb->nab", 6.0*M, c, c)
    Groh = G.copy()
    if alpha2:
        Dd = np.where(S0 > 0, 6.0/np.where(S0 > 0, S0, 1), 0.0)
        G = G - Dd[:, None, None]*S1[:, :, None]*S1[:, None, :]
    k = np.argmin(np.abs(nv), axis=1)
    h = np.zeros_like(nv); h[np.arange(len(nv)), k] = 1.0
    e1 = np.cross(h, nv); e1 /= np.linalg.norm(e1, axis=1)[:, None]
    e2 = np.cross(nv, e1)
    q = lambda u, Mx, v: np.einsum("na,nab,nb->n", u, Mx, v)
    A11, A22, A12 = q(e1, G, e1), q(e2, G, e2), q(e1, G, e2)
    Snn, Sn1, Sn2 = q(nv, G, nv), q(e1, G, nv), q(e2, G, nv)
    if rel_snn is not None:
        Snn = np.where(Snn < rel_snn*q(nv, Groh, nv), 0.0, Snn)
    ent = (Snn < 1e-8) | ((Sn1*Sn1 + Sn2*Sn2) <= 1e-6*Snn*(A11 + A22))
    sd = np.where(ent, 1.0, Snn)
    A11 = np.where(ent, A11, A11 - Sn1*Sn1/sd); A22 = np.where(ent, A22, A22 - Sn2*Sn2/sd); A12 = np.where(ent, A12, A12 - Sn1*Sn2/sd)
    tr = A11 + A22; det = A11*A22 - A12*A12
    disc = np.maximum(tr*tr - 4*det, 0.0)
    lmax = 0.5*(tr + np.sqrt(disc)); lmin = 0.5*(tr - np.sqrt(disc))
    vh = np.where(lmax > 0, np.where(lmin > 0, lmin/np.where(lmax > 0, lmax, 1), 0.0), -1.0)
    rg = np.where(~(lmax > 1e-12), 0, np.where(vh < 1e-9, 1, 2))
    if not mit_kond: return rg, ent
    # ★ 21.09.2026 (Heiko): der Rang allein hat die Frage "hebt D3Q27 die Wandzellen" verdeckt beantwortet --
    # er ist eine Stufenfunktion. Was im Kernel wirklich entscheidet, sind die GROESSEN dahinter:
    #   Gt11/Gt22/dett = A11/A22/det NACH der Schur-Elimination (kernel.cpp: dett >= 1e-4*Gt11*Gt22 + det_eps),
    #   und die Verstaerkung 1/lmax .. 1/lmin, mit der ein tangentiales Residuum in s1 uebersetzt wird.
    # VORBEHALT, der mitgedruckt wird: A11/A22 haengen an der hier KONSTRUIERTEN Basis (e1 = h x n, h = Achse mit
    # kleinstem |n|), der Kernel legt t1 in die STROEMUNG. Basisunabhaengig und damit belastbar sind lmax, lmin,
    # lmin/lmax und det -- die stehen deshalb zuerst.
    return rg, ent, {"Gt11": A11, "Gt22": A22, "dett": det, "lmax": lmax, "lmin": lmin, "kond": vh}

def affine_dim(LK):
    """Affine Dimension der Linkmenge je Zelle (Schranke: r_t in [aff-1, min(2,aff)]), ueber eindeutige Masken."""
    bits = (LK.astype(np.int64) << np.arange(LK.shape[1])).sum(1)
    uq, inv = np.unique(bits, return_inverse=True); out = np.zeros(uq.size, np.int64)
    for i, m in enumerate(uq):
        P = CV[[b for b in range(LK.shape[1]) if (int(m) >> b) & 1]]
        out[i] = np.linalg.matrix_rank((P - P[0]).astype(float)) if len(P) > 1 else 0
    return out[inv]

def remesh_flaechen(pfad):
    roh = open(pfad, "rb").read()
    i = roh.index(b"POINTS"); e = roh.index(b"\n", i); npt = int(roh[i:e].split()[1])
    j = roh.index(b"POLYGONS"); ej = roh.index(b"\n", j); ntr = int(roh[j:ej].split()[1])
    P = np.fromstring(roh[e+1:j], sep=" ").reshape(npt, 3)
    k = roh.find(b"_DATA", ej); ende = roh.rfind(b"\n", 0, k) if k > 0 else len(roh)
    T = np.fromstring(roh[ej+1:ende], sep=" ", dtype=np.int64).reshape(ntr, 4)
    if not np.all(T[:, 0] == 3): raise SystemExit("Remesh: nicht nur Dreiecke")
    a, b, c = P[T[:, 1]], P[T[:, 2]], P[T[:, 3]]
    return 0.5*np.linalg.norm(np.cross(b-a, c-a), axis=1), (a[:, 0]+b[:, 0]+c[:, 0])/3.0

def main():
    lauf = sys.argv[1]; tms = sys.argv[2] if len(sys.argv) > 2 else "000501"
    vtk = os.path.join(lauf, f"feld_nah_{tms}ms.vtk")
    d = kopf(vtk); Nx, Ny, Nz = d["dims"]; ox, oy, oz = d["orig"]; dx = d["spac"][0]
    log = protokoll(lauf)["log"]
    if log is None:   # ★ 17.09.2026 (Pruefbefund 8): vorher TypeError in open(None)
        raise SystemExit(f"FEHLER: kein Laufprotokoll logs/{os.path.basename(os.path.normpath(lauf))}.log zu {lauf} -- B2/B3 brauchen den "
                         "statischen Klassenzensus und das Linkhistogramm aus dem Log; ohne Log keine Abnahme, kein Zensus")
    LZ = log_zensus(log)
    C, sp = lies_csv(os.path.join(lauf, "facetten_histogramme.csv"))
    n_alle = C["n"].astype(np.int64)
    NRp = os.path.join(lauf, "facetten_normalen.npz")
    if not os.path.exists(NRp): raise SystemExit(f"fehlt {NRp} -- zuerst werkzeuge/facetten_normalen.py {lauf}")
    NR = np.load(NRp)
    if not np.array_equal(NR["n"], n_alle): raise SystemExit("Normalen-Datei passt nicht zur CSV")
    print(f"# d3q27_zensus.py (Fassung 2) {lauf}: {n_alle.size} Zeilen, Normalen aus {NRp}")
    aktiv = C["klasse"].astype(np.int64) == 0
    n = n_alle[aktiv]; N = n.size
    nf = NR["nf"][aktiv].astype(np.float64); nv = nf/np.linalg.norm(nf, axis=1)[:, None]
    FL = np.fromfile(vtk, dtype=np.uint8, count=Nx*Ny*Nz, offset=d["off_flags"])
    omax = int(np.abs(CV[:, 0] + Nx*(CV[:, 1] + Ny*CV[:, 2])).max())
    if n_alle.max() + omax >= FL.size or n_alle.min() - omax < 0: raise SystemExit("Index am Rand")
    LKa = np.zeros((n_alle.size, 26), bool); LKk = np.zeros((n_alle.size, 26), bool)
    for b, (cx, cy, cz) in enumerate(ALLE):
        f_ = FL[n_alle + cx + Nx*(cy + Ny*cz)]
        LKa[:, b] = f_ == 0x41                     # Fahrzeuglink (Definition von eigene_links in der CSV)
        LKk[:, b] = (f_ & 3) == 1                  # Kernel-Praedikat TYPE_S ohne TYPE_E (zaehlt auch die Strasse 0x01)
    del FL
    def pruef(name, ok, info=""):
        print(f"{name}: {'ok' if ok else 'ABWEICHUNG'} {info}")
        if not ok: raise SystemExit(f"FEHLER {name}")
    pruef("B1 eigene_links == D3Q19-Fahrzeuglinks (flags == 0x41), alle Zeilen", np.array_equal(LKa[:, :18].sum(1), C["eigene_links"].astype(np.int64)))
    pruef("B1 aktive Facetten: Kernel-Praedikat == Fahrzeuglink in allen 26 Richtungen (keine Strassenlinks)",
          np.array_equal(LKa[aktiv], LKk[aktiv]), f"abweichende Zellen {int((LKa[aktiv] != LKk[aktiv]).any(1).sum())}")
    del LKk
    LK = LKa[aktiv]; l19 = LK[:, :18].sum(1); lc = LK[:, 18:].sum(1)
    pruef("B2 aktive Facetten == Log", N == LZ.get("aktiv"), f"{N} / {LZ.get('aktiv')}")
    h = np.bincount(l19, minlength=20); hl = LZ.get("links", {})
    pruef("B2 Linkhistogramm == Log", all(int(h[k]) == hl.get(k, 0) for k in range(1, 20)))

    m19 = LK.copy(); m19[:, 18:] = False
    rg19, ent19 = klassifiziere(m19, W19, nv, True)
    rr19, _ = klassifiziere(m19, W19, nv, False)
    ist = [int((rg19 == r).sum()) for r in (2, 1, 0)]; soll = [LZ.get(f"rang{r}") for r in (2, 1, 0)]
    pruef("B3 Rang 2/1/0 == Log", ist == soll, f"{ist} / {soll}")
    W = np.zeros((3, 3), np.int64); np.add.at(W, (rr19, rg19), 1)
    pruef("B3 Wanderung roh->ALPHA2 == Log", np.array_equal(W, LZ["wanderung"]), f"{W.tolist()}")
    pruef("B3 entkoppelt == Log", LZ.get("entkoppelt") is not None and int(ent19.sum()) == LZ["entkoppelt"], f"{int(ent19.sum())} / {LZ.get('entkoppelt')}")
    vz_bad = int(((nv*(LK[:, :18, None]*CV[None, :18, :]).sum(1)).sum(1) >= 0).sum())
    pruef("B3 Vorzeichen: Facettennormale zeigt ins Fluid (n . Sum c_Wand < 0), Toleranz 1e-5 (Duennteil-Einzelfaelle)", vz_bad <= 1e-5*N, f"Verstoesse {vz_bad} von {N}")

    rg27, _ = klassifiziere(LK, W27, nv, True)
    rr27, _ = klassifiziere(LK, W27, nv, False)
    rgk27, _ = klassifiziere(LK, W27, nv, True, rel_snn=1e-4)
    rgk19, _ = klassifiziere(m19, W19, nv, True, rel_snn=1e-4)
    aff19, aff27 = affine_dim(m19), affine_dim(LK)
    exakt = np.sum(nf == 0, axis=1) == 2
    faca = 1.0/np.maximum(np.max(np.abs(nf), axis=1), 1.0/np.sqrt(3.0))
    A = lambda m: faca[m].sum()*dx*dx

    print("\n=== D3Q19 -> D3Q27 (L19 + Ecken, D3Q27-Gewichte), ALPHA2, echte Normalen ===")
    M = np.zeros((3, 3), np.int64); np.add.at(M, (rg19, rg27), 1)
    for r in range(3): print(f"  D3Q19 Rang {r} ({int((rg19==r).sum()):8d}): " + "  ".join(f"->27 Rang {k}: {M[r,k]:8d}" for k in range(3)))
    B = rg19 == 0
    print(f"\n  Rangboden D3Q19: {int(B.sum())} Zellen, {A(B):.3f} m2 (Facettenflaeche gesamt {A(np.ones(N,bool)):.3f} m2)")
    for lab, m in (("1 Link", B & (l19 == 1)), ("2 Links", B & (l19 == 2)), (">=3 Links", B & (l19 >= 3))):
        k = int(m.sum()); hb = m & (rg27 >= 1)
        if k == 0: continue
        print(f"   {lab:9s}: {k:8d} | gehoben {int(hb.sum()):8d} = {100*hb.sum()/k:6.2f} %, {A(hb):.3f} m2 | Kernel-Snn-Waechter: {int((m & (rgk27>=1)).sum())}"
              f" | Schranken aus affiner Dim: {int((m & (aff27 >= 2)).sum())} .. {int((m & (aff27 >= 1)).sum())} | roh (ohne Massenerhaltung) Rang>=1: {int((m & (rr27>=1)).sum())}")
    for lab, m in (("achsparallel (n exakt Achse)", B & exakt), ("gekippt", B & ~exakt)):
        k = int(m.sum()); print(f"   {lab:28s}: {k:8d} Zellen, gehoben {int((m & (rg27>=1)).sum())}")
    print(f"  D3Q19 mit Kernel-Snn-Waechter: Rang 2/1/0 = {[int((rgk19==r).sum()) for r in (2,1,0)]} (statische Obergrenze ist damit nicht streng)")

    # ★★ 21.09.2026 KONDITIONIERUNG STATT RANG (Heiko-Auftrag: "billig ohne Bau testen").
    # Der Rang ist eine Stufenfunktion und hat die Frage verdeckt beantwortet: er sagt, OB eine Zelle
    # loesbar ist, nicht WIE GUT. Der Kernel entscheidet an dett >= 1e-4*Gt11*Gt22 + det_eps und reisst
    # danach am SATGATE, wenn s1 zu gross wird -- beides haengt an den Eigenwerten des 2x2 nach der
    # Schur-Elimination. Hier stehen sie fuer BEIDE Geschwindigkeitssaetze nebeneinander, je D3Q19-Rangklasse.
    _, _, K19 = klassifiziere(m19, W19, nv, True, mit_kond=True)
    _, _, K27 = klassifiziere(LK,  W27, nv, True, mit_kond=True)
    print("\n=== KONDITIONIERUNG D3Q19 gegen D3Q27 (Median je Klasse; ALPHA2, echte Normalen) ===")
    print("  lmax/lmin/det sind BASISUNABHAENGIG und damit die belastbaren Groessen.")
    print("  Gt11/Gt22 haengen an der hier konstruierten Tangentialbasis (e1 = h x n); der Kernel legt t1 in die")
    print("  STROEMUNG -- die Spalten sind deshalb ein Indikator fuer die Groessenordnung, kein Kernelwert.")
    print(f"  {'Klasse':22s} {'n':>9s} | {'lmax19':>10s} {'lmax27':>10s} {'d%':>7s} | {'lmin19':>10s} {'lmin27':>10s} |"
          f" {'det19':>10s} {'det27':>10s} {'d%':>7s} | {'Gt11_19':>9s} {'Gt11_27':>9s} {'d%':>7s}")
    def med(a, m):
        v = a[m]; return float(np.median(v)) if v.size else float("nan")
    # ★ Prozent NUR gegen einen Bezugswert oberhalb der Rangschwelle 1e-12 (dieselbe Schwelle wie in
    # klassifiziere). Darunter ist der Nenner Rauschen, und 100*(b-a)/a liefert Zahlen wie 7e17 % --
    # eine Zahl, die aussieht wie ein Befund und keiner ist. Dann steht "Rausch->real" bzw. "--".
    def dpz(a, b):
        if not (a == a and b == b): return None
        if abs(a) < 1e-12: return "R->real" if abs(b) > 1e-12 else "--"
        return f"{100.0*(b-a)/a:+6.1f}%"
    for lab, m in (("D3Q19 Rang 2", rg19 == 2), ("D3Q19 Rang 1", rg19 == 1), ("D3Q19 Rang 0", rg19 == 0),
                   ("  davon 1 Link", (rg19 == 0) & (l19 == 1)), ("  davon 2 Links", (rg19 == 0) & (l19 == 2)),
                   ("Rang1 -> 27 Rang2", (rg19 == 1) & (rg27 == 2)), ("Rang0 -> 27 Rang>=1", (rg19 == 0) & (rg27 >= 1))):
        k = int(m.sum())
        if k == 0: continue
        a, b = med(K19["lmax"], m), med(K27["lmax"], m)
        c, e = med(K19["lmin"], m), med(K27["lmin"], m)
        f_, g = med(K19["dett"], m), med(K27["dett"], m)
        h_, i_ = med(K19["Gt11"], m), med(K27["Gt11"], m)
        print(f"  {lab:22s} {k:9d} | {a:10.3e} {b:10.3e} {str(dpz(a,b)):>8s} | {c:10.3e} {e:10.3e} |"
              f" {f_:10.3e} {g:10.3e} {str(dpz(f_,g)):>8s} | {h_:9.3e} {i_:9.3e} {str(dpz(h_,i_)):>8s}")
    print("  LESART: steigt lmax/det, ist die Zelle unter D3Q27 BESSER konditioniert; sinkt sie, ist sie schlechter")
    print("  gekoppelt -- dann braucht dieselbe Wandschubspannung ein groesseres s1 und reisst eher das SATGATE.")

    # Nutzrichtung (INDIKATOR, Momentanfeld): tangentiale Richtung der Linkvariation gegen u_t der Zelle
    U = np.memmap(vtk, dtype=">f4", mode="r", offset=d["off_u"], shape=(Nx*Ny*Nz, 3))
    print("\n  Nutzrichtung der gehobenen Zellen (INDIKATOR aus dem Momentanfeld): |cos(d_t, u_t)| >= 0,5")
    for lab, m in (("1 Link gehoben", B & (l19 == 1) & (rg27 >= 1)), ("2 Links gehoben", B & (l19 == 2) & (rg27 >= 1))):
        idx = np.nonzero(m)[0]
        if idx.size == 0: continue
        u = np.asarray(U[n[idx]]).astype(np.float64); un = (u*nv[idx]).sum(1); ut = u - un[:, None]*nv[idx]
        bits = (LK[idx].astype(np.int64) << np.arange(26)).sum(1); uq, inv = np.unique(bits, return_inverse=True)
        dirs = np.zeros((uq.size, 3))
        for kk, mm in enumerate(uq):
            P = CV[[b for b in range(26) if (int(mm) >> b) & 1]].astype(float); Q = P - P.mean(0)
            dirs[kk] = np.linalg.eigh(Q.T @ Q)[1][:, -1]
        dv = dirs[inv]; dt = dv - (dv*nv[idx]).sum(1)[:, None]*nv[idx]
        cosv = np.abs((dt*ut).sum(1))/np.maximum(np.linalg.norm(dt, axis=1)*np.linalg.norm(ut, axis=1), 1e-30)
        print(f"   {lab:16s}: {idx.size:8d} Zellen, davon ausgerichtet {100*np.mean(cosv >= 0.5):5.1f} % ({A(m) and faca[idx][cosv>=0.5].sum()*dx*dx:.3f} von {A(m):.3f} m2)")

    # Flaechen-Gate: eigene Remesh-Flaeche (derselbe Voxelkoerper, geglaettet) und OF13 (andere Netzung, gleiche STL)
    ar, xr = remesh_flaechen(os.path.join(lauf, "remesh_flaeche.vtk"))
    O = np.load(os.path.join(HIER, "..", "export", "of13_vehicle_flaechen_1200.npz"))
    oa = np.linalg.norm(O["Sf"], axis=1); ox13 = O["C"][:, 0] + 2.2063; onz = -O["Sf"][:, 2]/np.maximum(oa, 1e-30)
    X = ox + (n % Nx)*dx
    BER = [("Nase/Splitter", -0.2, 0.6), ("Vorderrad/Haube", 0.6, 1.2), ("Scheibe", 1.2, 2.0), ("Dach", 2.0, 2.8),
           ("Heckscheibe/Hinterrad", 2.8, 3.7), ("Deck/Diffusor", 3.7, 4.0), ("Fluegel/Heck", 4.0, 4.6)]
    print(f"\n=== Flaechen-Gate [m2]: Remesh (eigener Koerper) | OF13 | FX Facetten alle | D3Q19 Rang>=1 | D3Q27 Rang>=1 | Quoten 19/Remesh, 27/Remesh ===")
    for lab, a, b in BER + [("GESAMT", -9, 9)]:
        mx = (X >= a) & (X < b); rm = ar[(xr >= a) & (xr < b)].sum(); of = oa[(ox13 >= a) & (ox13 < b)].sum()
        print(f"  {lab:22s} {rm:7.3f} {of:7.3f} {A(mx):8.3f} {A(mx & (rg19>=1)):8.3f} {A(mx & (rg27>=1)):8.3f} | {A(mx & (rg19>=1))/rm:5.3f} {A(mx & (rg27>=1))/rm:5.3f}")
    print("  nach Orientierung der Facettennormale (n_z >= 0,5 oben | <= -0,5 unten | sonst Seite), gegen OF13 gleiche Einteilung:")
    for lab, mf, mo in (("oben", nv[:, 2] >= 0.5, onz >= 0.5), ("unten", nv[:, 2] <= -0.5, onz <= -0.5), ("Seite", np.abs(nv[:, 2]) < 0.5, np.abs(onz) < 0.5)):
        print(f"   {lab:6s} OF13 {oa[mo].sum():7.3f} | FX alle {A(mf):7.3f} 19 {A(mf & (rg19>=1)):7.3f} 27 {A(mf & (rg27>=1)):7.3f}")
    out = os.path.join(lauf, "d3q27_zensus_zellen.npz")
    np.savez_compressed(out, n=n, rg19=rg19.astype(np.int8), rg27=rg27.astype(np.int8), rr19=rr19.astype(np.int8), rr27=rr27.astype(np.int8),
                        rgk27=rgk27.astype(np.int8), aff19=aff19.astype(np.int8), aff27=aff27.astype(np.int8), l19=l19.astype(np.int8), lc=lc.astype(np.int8))
    print(f"geschrieben: {out}")

if __name__ == "__main__":
    main()
