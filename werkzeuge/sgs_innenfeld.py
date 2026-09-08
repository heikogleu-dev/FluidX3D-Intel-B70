#!/usr/bin/env python3
"""sgs_innenfeld.py -- Was taeten WALE / Sigma / Vreman / AMD im INNENFELD (Lage 2..12), gemessen an
den VTK-Feldern eines Laufs, ohne eine Zeile Solver-Code?

Anlass (08.09.2026): die g-Diagnose vom 02.09. hat WALE und Sigma nur an WANDZELLEN gemessen (Lage 1),
und dort ist die Stroemung praktisch reine Scherung (Omega/S = 0,99) -- dort verschwinden alle vier
Modelle per Konstruktion. Im Innenfeld sind sie NIE gemessen worden. Dort ist die Stroemung
dreidimensional turbulent (sism_klemme.py: Sbar/|S| im Nachlauf 0,35) -- dort haetten sie etwas zu tun.
Frage: liefert eines der lokalen (speicherfreien) Modelle in Lage 2..8 eine moderate nu_t-Absenkung
in der Groessenordnung von SISM (Faktor ~0,5), oder schaltet es ab (Faktor < 0,1)?

Operatoren WOERTLICH wie setup.cpp gdiag_sensoren (dieselben Formeln, mit denen der Kernel an der
Wand gemessen hat), dazu Vreman (2004) und AMD (Rozema et al. 2015). Gradient wie sism_klemme.py
(kernel-identischer Zentraldifferenzen-Stencil, Solid-Nachbarn = u=0, Solid-Test (flags&3)==1).

Vergleichsgroesse je Zelle: R_m = nu_t,m / nu_t,Smag mit LITERATURKONSTANTEN
  Smag:   nu_t = 0.030021*|S|               (Projekt: (C*Delta)^2 = 0.030021, C = 0.1733, Delta = 1)
  WALE:   nu_t = C_w^2 * Op_W,  C_w = 0.325  (Nicoud & Ducros 1999)
  Sigma:  nu_t = C_s^2 * D_sig, C_s = 1.35   (Nicoud et al. 2011)
  Vreman: nu_t = c * sqrt(B_beta/(a:a)), c = 2.5*0.1733^2 = 0.0751  (Vreman 2004, c = 2.5 C_Smag^2)
  AMD:    nu_t = C * max(0, -(d_k u_i)(d_k u_j) S_ij)/((d_l u_m)(d_l u_m)), C = 0.3 (Rozema 2015, ZD)
Zusaetzlich das ROHE Operatorverhaeltnis Op/|S| fuer WALE und Sigma -- das ist die Groesse, die die
g-Diagnose als 'WALE/FD' bzw. 'Sigma/FD' meldet, damit Wand (Kernel) und Innenfeld (hier) vergleichbar sind.

Aufruf: sgs_innenfeld.py --selbsttest
        sgs_innenfeld.py <export/LAUF> [t_min_ms]
"""
import sys, os, glob, re
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sism_klemme import kopf, lies_u, lies_flags, TYPE_BO, TYPE_S

C_SMAG2 = 0.030021          # (C*Delta)^2 des Projekts
C_WALE  = 0.325
C_SIGMA = 1.35
C_VREM  = 2.5*0.1733**2
C_AMD   = 0.3

def g_tensor(u, solid):
    """g[i][a] = d u_i / d x_a, kernel-identisch (0.5*(u+ - u-), Solid-Nachbar = 0). Rand NaN."""
    sh = u.shape[1:]
    g = np.empty((3, 3) + sh, dtype=np.float32); g[:] = np.nan
    for a, ax in enumerate((3, 2, 1)):            # a: 0=x (letzte Achse), 1=y, 2=z
        up = np.roll(u, -1, axis=ax); um = np.roll(u, 1, axis=ax)
        sp = np.roll(solid, -1, axis=ax-1); sm = np.roll(solid, 1, axis=ax-1)
        for i in range(3):
            g[i, a] = 0.5*(np.where(sp, np.float32(0.0), up[i]) - np.where(sm, np.float32(0.0), um[i]))
    for ax in (1, 2, 3):
        s0 = [slice(None)]*4; s0[ax] = 0; s1 = [slice(None)]*4; s1[ax] = -1
        g[(slice(None),)+tuple(s0)] = np.nan; g[(slice(None),)+tuple(s1)] = np.nan
    return g

def operatoren(g):
    """Liefert dict mit |S|, Op_WALE, D_sigma, nu_Vreman, nu_AMD je Zelle (alle ohne Modellkonstante
    ausser Vreman/AMD, die per Definition schon nu_t sind). Formeln wie gdiag_sensoren."""
    S = 0.5*(g + np.swapaxes(g, 0, 1))
    SS = np.einsum('ia...,ia...->...', S, S)
    snorm = np.sqrt(2.0*SS)
    # WALE: gg = g.g, Sd = sym(gg) - tr(gg)/3 I
    gg = np.einsum('ik...,ka...->ia...', g, g)
    tr3 = (gg[0, 0] + gg[1, 1] + gg[2, 2])/3.0
    Sd = 0.5*(gg + np.swapaxes(gg, 0, 1))
    for i in range(3): Sd[i, i] -= tr3
    SdSd = np.einsum('ia...,ia...->...', Sd, Sd)
    wale = (SdSd*np.sqrt(SdSd))/(SS*SS*np.sqrt(SS) + SdSd*np.sqrt(np.sqrt(SdSd)) + 1e-30)
    del gg, Sd
    # Sigma: Singulaerwerte von g = sqrt(Eigenwerte von g^T g)
    M = np.einsum('ia...,ib...->ab...', g, g)                 # M[a][b] = sum_i g[i][a] g[i][b]
    Mm = np.moveaxis(M, (0, 1), (-2, -1)).astype(np.float64)   # (..., 3, 3)
    ok = np.isfinite(Mm).all(axis=(-2, -1))
    ew = np.zeros(Mm.shape[:-2] + (3,), dtype=np.float64)
    ew[ok] = np.linalg.eigvalsh(Mm[ok])                        # aufsteigend
    del M, Mm
    s3, s2, s1 = np.sqrt(np.maximum(ew[..., 0], 0)), np.sqrt(np.maximum(ew[..., 1], 0)), np.sqrt(np.maximum(ew[..., 2], 0))
    sigma = np.where(s1 > 1e-30, s3*(s1-s2)*(s2-s3)/(s1*s1 + 1e-300), 0.0).astype(np.float32)
    del ew
    # Vreman: alpha_ij = d u_j / d x_i = g[j][i]; beta_ij = sum_m alpha_mi alpha_mj (Delta = 1)
    a = np.swapaxes(g, 0, 1)                                   # a[i][j] = g[j][i]
    beta = np.einsum('mi...,mj...->ij...', a, a)
    Bb = (beta[0, 0]*beta[1, 1] - beta[0, 1]**2 + beta[0, 0]*beta[2, 2] - beta[0, 2]**2 + beta[1, 1]*beta[2, 2] - beta[1, 2]**2)
    aa = np.einsum('ij...,ij...->...', a, a)
    vreman = np.where(aa > 1e-30, C_VREM*np.sqrt(np.maximum(Bb, 0.0)/(aa + 1e-300)), 0.0).astype(np.float32)
    del beta, Bb
    # AMD: nu = C * max(0, -(d_k u_i)(d_k u_j) S_ij) / (d_l u_m d_l u_m); d_k u_i = g[i][k]
    num = -np.einsum('ik...,jk...,ij...->...', g, g, S)
    amd = np.where(aa > 1e-30, C_AMD*np.maximum(num, 0.0)/(aa + 1e-300), 0.0).astype(np.float32)
    del num, aa, a, S
    return dict(snorm=snorm.astype(np.float32), wale=wale.astype(np.float32), sigma=sigma, vreman=vreman, amd=amd)

def selbsttest():
    G = [[[0,1,0],[0,0,0],[0,0,0]], [[0,1,0],[-1,0,0],[0,0,0]], [[0,1,0],[-0.3,0,0.4],[0.2,0.5,0]],
         [[1,0,0],[0,-0.5,0],[0,0,-0.5]], [[0,1,0],[0.6,0,0],[0,0,0]]]
    soll = [[1.000000000,0.000000000,0.000000000],[0.000000000,0.903602004,0.000000000],
            [1.157583690,0.158731813,0.025153979],[1.732050808,0.075313192,0.000000000],[1.600000000,0.058159040,0.000000000]]
    g = np.zeros((3, 3, 1, 1, 1), dtype=np.float32); fehler = 0
    for f in range(5):
        for i in range(3):
            for a in range(3): g[i, a, 0, 0, 0] = G[f][i][a]
        o = operatoren(g)
        ist = [float(o['snorm'][0,0,0]), float(o['wale'][0,0,0]), float(o['sigma'][0,0,0])]
        for k, name in enumerate(('snorm','wale','sigma')):
            if abs(ist[k]-soll[f][k]) > 1e-5*(1.0+abs(soll[f][k])):
                print(f"SELBSTTEST VERLETZT: Fall {f+1} {name}: ist {ist[k]:.9f} soll {soll[f][k]:.9f}"); fehler += 1
        # Vreman/AMD gegen von Hand gerechnete Literaturwerte. Vreman verschwindet in reiner SCHERUNG (sein
        # Hauptmerkmal), aber NICHT in Festkoerperrotation: g=[[0,1,0],[-1,0,0],[0,0,0]] -> beta11=beta22=1,
        # beta12=0 -> B_beta=1, alpha:alpha=2 -> nu = c*sqrt(1/2) = 0.05310. Genau das kritisiert Nicoud 2011
        # und motiviert damit Sigma. Die erste Fassung dieses Tests forderte 0 -- Erwartung falsch, Operator richtig.
        if f == 0:
            for name in ('vreman','amd'):
                v = float(o[name][0,0,0])
                if abs(v) > 1e-6: print(f"SELBSTTEST VERLETZT: Fall 1 {name} = {v:.3e}, Soll 0 (reine Scherung)"); fehler += 1
        if f == 1:
            v = float(o['vreman'][0,0,0]); sv = C_VREM*np.sqrt(0.5)
            if abs(v-sv) > 1e-6: print(f"SELBSTTEST VERLETZT: Fall 2 vreman = {v:.6f}, Soll c*sqrt(1/2) = {sv:.6f} (Festkoerperrotation)"); fehler += 1
            v = float(o['amd'][0,0,0])
            if abs(v) > 1e-6: print(f"SELBSTTEST VERLETZT: Fall 2 amd = {v:.3e}, Soll 0 (Rotation)"); fehler += 1
    print("Selbsttest", "VERLETZT" if fehler else "bestanden: 5 Referenztensoren x (|S|, WALE, Sigma) auf 1e-5 wie setup.cpp sgs_gdiag_selbsttest; Vreman = 0 in reiner Scherung und = c*sqrt(1/2) in Festkoerperrotation (Literatur), AMD = 0 in beiden.")
    return fehler == 0

def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--selbsttest':
        sys.exit(0 if selbsttest() else 1)
    lauf = sys.argv[1].rstrip('/'); t_min = float(sys.argv[2]) if len(sys.argv) > 2 else 201.0
    paare = sorted((float(m.group(1)), p) for p in glob.glob(os.path.join(lauf, 'feld_nah_*ms.vtk'))
                   for m in [re.search(r'feld_nah_(\d+)ms\.vtk$', p)] if m and float(m.group(1)) >= t_min)
    d = kopf(paare[0][1]); Nx, Ny, Nz = d['dims']; ox, oy, oz = d['orig']; dx = d['spac'][0]
    flags = lies_flags(paare[0][1], d); solid = (flags & TYPE_BO) == TYPE_S; fluid = (flags & TYPE_BO) == 0
    print(f"{os.path.basename(lauf)}: {len(paare)} Felder ab {t_min} ms, Gitter {Nx}x{Ny}x{Nz}, dx {dx*1000:.1f} mm")
    # Wandabstand (Lage 1..12) per Dilatation
    lage = np.zeros(solid.shape, dtype=np.uint8); rand = solid.copy()
    for L in range(1, 13):
        nb = np.zeros_like(rand)
        for ax in (0, 1, 2): nb |= np.roll(rand, 1, axis=ax); nb |= np.roll(rand, -1, axis=ax)
        neu = nb & ~rand & (lage == 0) & ~solid; lage[neu] = L; rand |= neu
    x = ox + np.arange(Nx)*dx; z = oz + np.arange(Nz)*dx; X = x[None, None, :]; Z = z[:, None, None]
    reg = {'Nachlauf': fluid & (X > 4.5), 'Dachschicht': fluid & (X > 2.0) & (X < 3.7) & (Z > 1.0) & (Z < 1.6),
           'Unterboden': fluid & (X > 0.5) & (X < 4.4) & (Z < 0.15)}
    modelle = ('wale', 'sigma', 'vreman', 'amd')
    konst = dict(wale=C_WALE**2, sigma=C_SIGMA**2, vreman=1.0, amd=1.0)   # Vreman/AMD sind schon nu_t
    # Histogramme von log10(R) je (Modell, Klasse), Bins -5..+2 in 0.1
    edges = np.arange(-5.0, 2.05, 0.1); nb_ = len(edges)-1
    klassen = [f'Lage {L}' for L in range(1, 13)] + list(reg.keys())
    H = {m: {k: np.zeros(nb_, dtype=np.int64) for k in klassen} for m in modelle}
    Hroh = {m: {k: np.zeros(nb_, dtype=np.int64) for k in klassen} for m in ('wale', 'sigma')}
    for t, p in paare:
        u = lies_u(p, d); g = g_tensor(u, solid); del u
        o = operatoren(g); del g
        nsm = C_SMAG2*o['snorm']
        gut = fluid & np.isfinite(nsm) & (nsm > 1e-12)
        for m in modelle:
            R = np.where(gut, konst[m]*o[m]/(nsm + 1e-30), np.nan)
            lr = np.log10(np.maximum(R, 1e-6))
            for L in range(1, 13):
                mk = gut & (lage == L); H[m][f'Lage {L}'] += np.histogram(lr[mk], bins=edges)[0]
            for name, mk0 in reg.items():
                mk = gut & mk0 & (lage >= 2); H[m][name] += np.histogram(lr[mk], bins=edges)[0]
            if m in Hroh:
                Rr = np.where(gut, o[m]/(o['snorm'] + 1e-30), np.nan); lrr = np.log10(np.maximum(Rr, 1e-6))
                for L in range(1, 13):
                    mk = gut & (lage == L); Hroh[m][f'Lage {L}'] += np.histogram(lrr[mk], bins=edges)[0]
                for name, mk0 in reg.items():
                    mk = gut & mk0 & (lage >= 2); Hroh[m][name] += np.histogram(lrr[mk], bins=edges)[0]
            del R, lr
        del o, nsm, gut
        print(f"  t = {int(t)} ms verarbeitet", flush=True)
    mids = 0.5*(edges[:-1] + edges[1:])
    def stat(h):
        n = h.sum();
        if n == 0: return (np.nan, np.nan, np.nan, np.nan)
        cdf = np.cumsum(h)/n; med = 10**mids[np.searchsorted(cdf, 0.5)]
        ab = h[mids < -1.0].sum()/n; halb = h[mids < np.log10(0.5)].sum()/n
        mean = (h*10**mids).sum()/n
        return (med, mean, ab, halb)
    print("\nR = nu_t,Modell / nu_t,Smagorinsky (mit Literaturkonstanten). 'Abschalter' = Anteil R < 0.1, 'unter SISM-Niveau' = Anteil R < 0.5")
    for k in klassen:
        print(f"\n  {k:12s} " + "  ".join(f"{m:>22s}" for m in modelle))
        z1 = f"  {'Median R':12s} "; z2 = f"  {'R<0.1':12s} "; z3 = f"  {'R<0.5':12s} "
        for m in modelle:
            med, mean, ab, halb = stat(H[m][k])
            z1 += f"  {med:22.3f}"; z2 += f"  {100*ab:21.1f}%"; z3 += f"  {100*halb:21.1f}%"
        print(z1); print(z2); print(z3)
    print("\nROHES Operatorverhaeltnis Op/|S| (die Groesse der g-Diagnose 'WALE/FD', 'Sigma/FD'); Wand-Referenz 02.09.: Kanal WALE 1,9e-5 Sigma 8,6e-5, Fahrzeug 8 mm WALE 0,01-0,04 Sigma 0,002-0,006")
    for k in ['Lage 1', 'Lage 2', 'Lage 3', 'Lage 5', 'Lage 8', 'Lage 12', 'Nachlauf', 'Dachschicht', 'Unterboden']:
        z = f"  {k:12s}"
        for m in ('wale', 'sigma'):
            med, mean, ab, halb = stat(Hroh[m][k]); z += f"   {m} Median {med:.3e} Mittel {mean:.3e}"
        print(z)

if __name__ == '__main__':
    main()
