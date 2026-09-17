#!/usr/bin/env python3
"""zonen_vergleich.py (Fassung 2, 17.09.2026 nach Pruefagent) -- FX-Zonenkraft (zonen_kraft.py) gegen OF13-Randflaechen (of13_kraft_zband.py, OF13_CACHE).

GRUNDSATZ (17.09.2026): ein konstanter Bezugsdruck kuerzt sich nur ueber eine GESCHLOSSENE Flaeche. Ein x-Streifen
schneidet den Koerper mit zwei Ebenen x = const, deren Schnittflaechen kein S_z tragen -- die Summe Oberseite+Unterseite
eines Streifens ist fuer Cz deshalb bezugsunabhaengig (bis auf die offene Reifenaufstandsflaeche). Die Aufteilung
oben/unten oder nach Hoehe ist es NICHT: dort steht die Empfindlichkeit dCz/dcp_ref in einer eigenen Spalte, und eine
Differenz wird nur gedeutet, wenn sie groesser ist als 0,09 * |dCz/dcp_ref|. 0,09 = groesste gemessene Bezugsdifferenz
FX Dachmitte gegen Seitenstreifen (0,052/0,086/0,068/0,055 je Feld, zonenkraft_*ms.txt A6; Pruefrunde 2 MITTEL 2 -- vorher 0,05,
das war das Minimum). OF13 haengt NICHT von der Region ab (+0,0229/+0,0235/+0,0233). Spalte "dCz(Seitenbezug)" = FX mit Seitenstreifen-Bezug.

FX: vier Momentanfelder (p375_e/p375_b x 300/501 ms), P1 (Druckproxy); Orientierung nach der Zellnormale.
OF13: RANS-Mittel t = 1200, Flaechen je Bin nach Mittelpunkt x_v2 = x + 2,2063 und Vorzeichen von Sf_z.
BEZUG (Pruefagent H1/H2): beide Seiten auf den Totaldruck-Bezug derselben Region ueber dem Dach (x 0,3-3,0, z 1,5-1,8,
|y|<0,03): Cz_korr = Cz + S*cp_ref, FX je Feld aus zonenkraft_*ms.npz (rho_inf_mitte), OF13 aus of13_slab.npy.
REGEL "gleichsinnig" (ersetzt das zu weiche "BELEGT" der Fassung 1): gleiches Vorzeichen der korrigierten Differenz in
allen Feldern (MINDESTENS 2 -- ★ 17.09.2026 Pruefbefund 11: mit einem Feld ist "gleiches Vorzeichen" trivial und die halbe Spannweite 0;
dann wird die Kennung LAUT abgelehnt, nicht vergeben), |Mittel| > max(0,05 ; halbe Spannweite ; 0,05*max|S|) UND projizierte Flaechen passen
(|S_fx - S_of| <= 0,1 + 0,15*max|S|, sonst Kennung "S!"). Es bleiben MOMENTANBEFUNDE: e und b sind zum selben Zeitpunkt
nicht unabhaengig (rms 0,003 je Streifen), effektiv 2 Zeitpunkte; FX driftet noch (cz_rest 50-ms-Mittel -1,05 -> -0,94);
dazu der systematische Anteil aus A4 (Rest +0,05, Band -0,07 gegen den Facettenpfad, nicht lokalisiert).
Aufruf: zonen_vergleich.py [lauf:t_ms ...]
   ohne Argumente wie bisher export/p375_{e,b}/zonenkraft_{000300,000501}ms.npz; sonst die genannten Felder, z. B.
   zonen_vergleich.py p4_pu8:000501 p4_pu8:000300   (dazu immer export/of13_vehicle_flaechen_1200.npz)
★ 17.09.2026 (SKALIERUNG-BEFUNDE Befund 1 + Nebenbefund 10): Laufliste war fest p375; die OF13-Bandkante (erste z-Binkante) war fest
15 mm = N dx auf 3,75 mm. Jetzt: Felder als Argument; die erste z-Kante kommt aus der npz (zonen_kraft.py schreibt die WIRKSAME
FX-Bandoberkante des Laufs in Fahrzeugkoordinaten, (N - 0,5) dx - dk dx bzw. Log-Zeile KRAFT-ZBAND-KANTE) und gilt fuer FX UND OF13.
Alle Felder muessen dasselbe Binraster tragen (verschiedene Sprossen = verschiedene Bandkanten -> getrennt aufrufen).
npz aus der Fassung vor dem 17.09. (erste Kante 15 mm, ohne kante_quelle) werden LAUT markiert, nicht still verwendet.
"""
import os, sys
import numpy as np
HIER = os.path.dirname(os.path.abspath(__file__)); EXP = os.path.join(HIER, "..", "export")
if len(sys.argv) > 1:
    FELDER = []
    for a in sys.argv[1:]:
        if ":" not in a: raise SystemExit(f"Feld '{a}': Form lauf:t_ms, z. B. p375_e:000501")
        l, t = a.split(":", 1); t = t.zfill(6); FELDER.append((f"{l}@{int(t)}", l, t))
else:
    FELDER = [("e501", "p375_e", "000501"), ("e300", "p375_e", "000300"), ("b501", "p375_b", "000501"), ("b300", "p375_b", "000300")]
XOFF = 2.2063
F = {k: np.load(os.path.join(EXP, l, f"zonenkraft_{t}ms.npz")) for k, l, t in FELDER}
d0 = F[FELDER[0][0]]; XB0, XBW, NXB = float(d0["XB0"]), float(d0["XBW"]), int(d0["NXB"]); ZK = d0["ZKANTEN"]; NZB = len(ZK)+1
for k in F:
    if not F[k]["abnahme_a4"]: raise SystemExit(f"{k}: Abnahme A4 nicht bestanden -- kein Vergleich")
    if str(F[k].get("orientierung", "")) != "zellnormale": raise SystemExit(f"{k}: npz aus Fassung 1 (Link-Orientierung) -- neu rechnen")
    if int(F[k]["NXB"]) != NXB or len(F[k]["ZKANTEN"]) != len(ZK) or not np.allclose(F[k]["ZKANTEN"][1:], ZK[1:]):
        raise SystemExit("Binraster verschieden")
    if not np.allclose(F[k]["ZKANTEN"][0], ZK[0], rtol=0, atol=1e-9):
        raise SystemExit(f"Erste z-Kante (wirksame Bandkante) verschieden: {FELDER[0][0]} {ZK[0]*1e3:.3f} mm, {k} {float(F[k]['ZKANTEN'][0])*1e3:.3f} mm "
                         "-- Felder verschiedener Sprossen/Baender nicht mischen, getrennt aufrufen")
for k in F:
    if "kante_quelle" in F[k].files:
        print(f"Bandkante {k}: {float(F[k]['ZKANTEN'][0])*1e3:.3f} mm (Fahrzeug/OF13) -- {str(F[k]['kante_quelle'])}, N = {int(F[k]['zband_n'])}, dk = {int(F[k]['zband_dk'])}")
    else:
        print(f"WARNUNG {k}: npz aus zonen_kraft.py vor dem 17.09. -- erste z-Kante {float(F[k]['ZKANTEN'][0])*1e3:.3f} mm ist N dx, NICHT die wirksame "
              "Bandoberkante (N - 0,5) dx (Befund 1); Zeilen 'z 0..' und die Bandzuordnung der OF13-Flaechen tragen diese alte Kante -- zonen_kraft.py neu rechnen.")

def fx_cube(k):
    d = F[k]; KC = float(d["KC"]); ul = float(d["u_lat"])
    P1 = d["P1"].reshape(NXB, NZB, 3, 2, 3).sum(axis=3)          # Band-Flag zusammenfassen
    G = d["G"].reshape(NXB, NZB, 3, 2, 3).sum(axis=3)
    S = -G[..., 2]*KC*1.5*ul*ul                                   # dCz/dcp_ref je (x, z, ori)
    cpref = (float(d["rho_inf_mitte"][0]) - 1.0)*float(d["cp_faktor"])
    cpseite = (float(d["rho_inf_seite"][0]) - 1.0)*float(d["cp_faktor"])
    return P1[..., 2]*KC + S*cpref, P1[..., 0]*KC, S, P1[..., 2]*KC + S*cpseite   # Cz (Bezug Dachmitte), Cd, S, Cz (Seitenbezug)

O = np.load(os.path.join(EXP, "of13_vehicle_flaechen_1200.npz"))
Sf, Cm, pw = O["Sf"], O["C"], O["pw"]; R, A, Q = float(O["rho_inf"]), float(O["a_ref"]), float(O["q_inf"])
xb = np.clip(np.floor((Cm[:, 0] + XOFF - XB0)/XBW).astype(int), 0, NXB-1)
zb = np.searchsorted(ZK, Cm[:, 2], side="right")
ori = 1 + np.sign(Sf[:, 2]).astype(int)
idx = (xb*NZB + zb)*3 + ori
of_cz = np.bincount(idx, weights=R*pw*Sf[:, 2], minlength=NXB*NZB*3).reshape(NXB, NZB, 3)/(Q*A)
of_cd = np.bincount(idx, weights=R*pw*Sf[:, 0], minlength=NXB*NZB*3).reshape(NXB, NZB, 3)/(Q*A)
of_s = np.bincount(idx, weights=-Sf[:, 2], minlength=NXB*NZB*3).reshape(NXB, NZB, 3)/A      # dCz/dcp_ref
# Cache-Abnahme gegen forces.dat selbst pruefen (Pruefagent N2: der Cache wird auch bei verfehlter Abnahme geschrieben)
FALL = os.path.expanduser("~/CFD-Cases/mr2v40H"); fdir = os.path.join(FALL, "postProcessing/forces", sorted(os.listdir(os.path.join(FALL, "postProcessing/forces")))[-1])
soll = None
for z_ in open(os.path.join(fdir, "forces.dat"), errors="replace"):
    if not z_.startswith("#") and z_.split() and z_.split()[0] == str(O["zeit"]):
        t_ = z_.replace("(", " ").replace(")", " ").split(); soll = np.array([float(t_[1]), float(t_[2]), float(t_[3])])
if soll is None or np.max(np.abs(O["F_druck_summe"] - soll)/np.maximum(np.abs(soll), 1e-9)) > 0.01:
    raise SystemExit(f"OF13-Cache verfehlt forces.dat (Soll {soll}, Ist {O['F_druck_summe']})")
S_ = np.load(os.path.join(HIER, "abl_dach", "of13_slab.npy")).astype(np.float64)
mr = (S_[:, 0]+XOFF > 0.3) & (S_[:, 0]+XOFF < 3.0) & (S_[:, 2] > 1.5) & (S_[:, 2] < 1.8)
CPREF_OF = float(np.median(S_[mr, 3] + 0.5*((S_[mr, 4:7]**2).sum(1) - 900.0))/(0.5*900.0)); del S_
print(f"OF13-Flaechen nach Hoehe gebinnt mit denselben Kanten; erste Kante = FX-Bandkante {ZK[0]*1e3:.3f} mm (Flaechenmitte z < Kante -> Band-Bin)")
print(f"Bezug cp_ref (Totaldruck ueber dem Dach, |y|<0,03): OF13 {CPREF_OF:+.4f} | FX " + " ".join(f"{k} {(float(F[k]['rho_inf_mitte'][0])-1)*float(F[k]['cp_faktor']):+.4f}" for k in F))
of_cz = of_cz + of_s*CPREF_OF

FX = {k: fx_cube(k) for k in F}
MIN_FELDER = 2   # "gleichsinnig" braucht mindestens zwei Felder (Vorzeichen und Spannweite sind sonst nicht pruefbar)
if len(F) < MIN_FELDER:
    print("!" * 100)
    print(f"WARNUNG: nur {len(F)} Feld ({', '.join(F)}) -- die Regel 'gleichsinnig' verlangt mindestens {MIN_FELDER} Felder "
          "(gleiches Vorzeichen und halbe Spannweite sind mit einem Feld trivial). Die Kennung wird in KEINER Zeile vergeben.")
    print("!" * 100)
xk = XB0 + XBW*np.arange(NXB+1)
def fx_sum(k, sel): return tuple(float(a[sel].sum()) for a in FX[k][:3])
def zeile(lab, sel, dcp=True):
    fz = [fx_sum(k, sel) for k in F]; oz, os_ = float(of_cz[sel].sum()), float(of_s[sel].sum())
    dz = np.array([f[0] - oz for f in fz]); sfx = np.mean([f[2] for f in fz])
    hs = 0.5*(dz.max() - dz.min()); smax = max(abs(sfx), abs(os_)); schwelle = max(0.05, hs, 0.09*smax)
    s_ok = abs(sfx - os_) <= 0.1 + 0.15*smax
    gleich = len(fz) >= MIN_FELDER and (np.all(dz > 0) or np.all(dz < 0)) and abs(dz.mean()) > schwelle and s_ok
    dseite = np.mean([float(FX[k][3][sel].sum()) for k in F]) - oz
    print(f"{lab:34s} OF13 {oz:+7.3f} | FX " + " ".join(f"{f[0]:+7.3f}" for f in fz) + f" | dCz(Seitenbezug) {dseite:+7.3f}" +
          f" | dCz {dz.mean():+7.3f} [{dz.min():+.3f}..{dz.max():+.3f}]" +
          (f" | S fx {sfx:+6.2f} of {os_:+6.2f}" if dcp else "") + ("" if s_ok else "  S!") + ("  gleichsinnig" if gleich else ""))
    return dz.mean()

print("Cz FX gegen OF13 (Druck, ohne Reibung, beide bezugskorrigiert). FX-Spalten: " + " ".join(k for k in F) + ". dCz = FX - OF13 (positiv = FX weniger Abtrieb).")
print("S = dCz/dcp_ref. 'gleichsinnig' = Regel im Kopf erfuellt; 'S!' = projizierte Flaechen passen nicht. MOMENTANBEFUNDE, 2 Zeitpunkte.\n")
alle = np.ones((NXB, NZB, 3), bool)
zeile("GESAMT", alle)

print("\n=== A) x-Streifen 100 mm, Ober+Unterseite zusammen (bezugsunabhaengig) ===")
for i in range(NXB):
    sel = np.zeros_like(alle); sel[i] = True
    if abs(of_cz[sel].sum()) < 1e-6 and all(abs(fx_sum(k, sel)[0]) < 1e-6 for k in F): continue
    zeile(f"x {xk[i]:+.2f}..{xk[i+1]:+.2f}", sel)
print("\nKumulativ von der Nase (Mittel der 4 FX-Felder minus OF13):")
acc = 0.0; z = ""
for i in range(NXB):
    sel = np.zeros_like(alle); sel[i] = True
    acc += np.mean([fx_sum(k, sel)[0] for k in F]) - float(of_cz[sel].sum())
    if (i % 5) == 4: z += f"  x<{xk[i+1]:.1f}: {acc:+.3f}"
print(z)

# Bereiche aus der Geometrie (Belege im Tagesprotokoll 17.09.): Achsen x 0,893/3,308 (Log N2F-BAND RADSTAND, Grobzellen
# 237/398); Saugspitze Dachvorderkante x 2,0 (Dachprofil); Heckscheibe bis Deckel x 3,68 (Voxelmaske Mittelebene);
# Fluegel ab x 4,0 (Log Anbauteil x 4,01..4,43, Maske 4,10..4,40). Auf das 100-mm-Raster gerundet.
BER = [("Nase/Splitter",      -0.2, 0.6), ("Vorderrad/Haube",   0.6, 1.2), ("Scheibe",   1.2, 2.0),
       ("Dach",                2.0, 2.8), ("Heckscheibe/Hinterrad", 2.8, 3.7), ("Deck/Diffusor", 3.7, 4.0),
       ("Fluegel/Heck",        4.0, 4.6)]
def xsel(a, b):
    s = np.zeros_like(alle); ia, ib = int(round((a-XB0)/XBW)), int(round((b-XB0)/XBW)); s[ia:ib] = True; return s
print("\n=== B) Bereiche, gesamt / Oberseite / Unterseite (oben/unten bezugsempfindlich: Spalte S) ===")
for lab, a, b in BER:
    s = xsel(a, b); zeile(f"{lab} gesamt", s)
    so = s.copy(); so[..., 1:] = False; zeile(f"   oben", so)
    su = s.copy(); su[..., :2] = False; zeile(f"   unten", su)

print("\n=== C) Hoehe der Linkmitte / Flaechenmitte (alle x) -- NUR MIT S-ABGLEICH LESEN: die FX-Unterseite liegt ~2,4 mm tiefer,")
print("    die Bin-Grenze 0,12 m schneidet den flachen Unterboden (Pruefagent H1) ===")
zk = np.concatenate([[0.0], ZK, [np.inf]])
def _zf(v): return f"{v:.3f}" if (not np.isfinite(v) or abs(v*1e3 - round(v*1e3)) < 1e-9) else f"{v:.6f}"
for j in range(NZB):
    s = np.zeros_like(alle); s[:, j] = True; zeile(f"z {_zf(zk[j])}..{_zf(zk[j+1])} gesamt", s)
    so = s.copy(); so[..., 1:] = False; zeile("   oben", so)
    su = s.copy(); su[..., :2] = False; zeile("   unten", su)

print("\n=== D) Fluegel-Kasten x 3,9..4,6, z >= 1,0 (oben+unten) ===")
s = xsel(3.9, 4.6); s[:, :7] = False; zeile("Fluegel gesamt", s)
so = s.copy(); so[..., 1:] = False; zeile("   oben", so)
su = s.copy(); su[..., :2] = False; zeile("   unten", su)
s = xsel(3.9, 4.6); s[:, 7:] = False; zeile("x 3,9..4,6 unter z 1,0", s)

print("\n=== E) Bodenfreiheit Mittellinie |y| < 0,1 m [mm]: FX (Unterkante Voxel ueber Strassenwand) gegen OF13 (tiefste Unterseitenflaeche) ===")
ym = (np.abs(Cm[:, 1]) < 0.1) & (Sf[:, 2] > 0) & (Cm[:, 2] > 0.02)
# Fassung vor 17.09.: fest e501/b501; jetzt die Felder zum spaetesten Zeitpunkt je Lauf (Vorgabeliste: unveraendert e501, b501)
EKEYS = ("e501", "b501") if len(sys.argv) <= 1 else tuple(k for k, l, t in FELDER if t == max(tt for kk, ll, tt in FELDER if ll == l))
EBESCHR = "e/b" if len(sys.argv) <= 1 else "/".join(EKEYS)
z = ""
for i in range(NXB):
    m = ym & (xb == i)
    if not m.any(): continue
    h = " ".join(f"{1e3*float(F[k]['h_mitte'][i]):5.1f}" for k in EKEYS)
    z += f"\n  x {xk[i]:+.2f}: OF13 {1e3*Cm[m, 2].min():6.1f} | FX {EBESCHR} {h}"
print(z)
