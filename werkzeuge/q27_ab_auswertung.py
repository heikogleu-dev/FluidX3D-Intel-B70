#!/usr/bin/env python3
"""q27_ab_auswertung.py -- 8-mm-A/B D3Q27 gegen D3Q19 ohne Wandmodell (Heiko 17.09.2026, Option 1): Kraefte und Kosten.

KRAEFTE: kraft_zband.csv (object_force = Impulsaustausch INKLUSIVE Reibung, BB-Arme haben keinen Facettenpfad und damit kein
cd_druck_rest). "rest" = ohne das Kontaktband (Fahrzeugzellen z-Index < N = CFD_KRAFT_ZBAND des Laufs; wirksam z = 1..N-1, Oberkante
  (N - 0,5) dx -- z = 0 ist Fahrbahn, SKALIERUNG-BEFUNDE Befund 1). ★ 17.09.2026: Regel seit heute N = max(3, ceil(16 mm/dx)) -> 8 mm N = 3,
  Kante 20 mm (Keil- UND Deckellage im Band). Die A/B-Laeufe q19_bb8/q27_bb8 liefen davor mit N = 2 (Kante 12 mm, Deckellage z = 2 im
  REST, BAND-ARTEFAKT-8MM.md). N und Kante beider Arme werden aus dem Laufprotokoll gelesen und verglichen; Abweichung = WARNUNG.
  Cd_rest = Fx_rest_N / (q_inf A_ref), Cz_rest = Spalte Cz_rest. Nur Arm gegen Arm lesbar, nicht gegen OF13 oder cd_druck_rest.
  Fenster ab T_WARMUP (0,201 s): Mittel, 50-ms-Blockmittel, GEPAARTE Differenz je Abtastzeit mit Blockfehler (SEM ueber die
  50-ms-Bloecke der Differenzreihe) und Vorzeichenzaehlung.
KOSTEN: Durchsatz aus den Fortschrittszeilen des Logs (MLUPs je Domaene, Median im Fenster), Wanduhr aus der Queue-Statusdatei,
  Speicherplan je Domaene aus dem Log. Keine Hochrechnung.
Aufruf: q27_ab_auswertung.py [bezug=q19_bb8] [arm=q27_bb8]
"""
import sys, os, re, csv
import numpy as np
HIER = os.path.dirname(os.path.abspath(__file__)); WURZEL = os.path.join(HIER, "..")
sys.path.insert(0, HIER)
import lauf_meta
REF = sys.argv[1] if len(sys.argv) > 1 else "q19_bb8"; ARM = sys.argv[2] if len(sys.argv) > 2 else "q27_bb8"
Q, A = 0.5*1.225*30.0**2, 1.85; T0 = 0.201

def zband(lauf):
    p = os.path.join(WURZEL, "export", lauf, "kraft_zband.csv")
    zeilen = [l for l in open(p) if not l.startswith("#")]
    r = list(csv.DictReader(zeilen))
    t = np.array([float(x["time_s"]) for x in r])
    cd = np.array([float(x["Fx_rest_N"]) for x in r])/(Q*A)
    cz = np.array([float(x["Cz_rest"]) for x in r])
    cdg = (np.array([float(x["Fx_rest_N"]) + float(x["Fx_band_N"]) for x in r]))/(Q*A)
    czg = (np.array([float(x["Fz_rest_N"]) + float(x["Fz_band_N"]) for x in r]))/(Q*A)
    return t, dict(Cd_rest=cd, Cz_rest=cz, Cd_ges=cdg, Cz_ges=czg)

def bloecke(t, v, breite=0.05):
    k = np.floor((t - T0)/breite).astype(int); out = []
    for b in np.unique(k):
        m = k == b
        if m.sum() >= 3: out.append(v[m].mean())
    return np.array(out)

tr, R = zband(REF); ta, Aa = zband(ARM)
print(f"# q27_ab_auswertung.py  Bezug {REF}  Arm {ARM}  (object_force inkl. Reibung, ohne Kontaktband; Fenster ab {T0} s)")
# ★ 17.09.2026 (Pruefbefund 3): *_rest haengt an der Bandkante -- beide Arme vergleichen, Abweichung LAUT (Kopf und Ende)
BZ, BAND_GLEICH, BAND_KURZ = lauf_meta.band_vergleich([(l, os.path.join(WURZEL, "export", l)) for l in (REF, ARM)])
print("\n".join(BZ))
gemeinsam = np.intersect1d(np.round(tr, 6), np.round(ta, 6)); gemeinsam = gemeinsam[gemeinsam >= T0]
ir = np.isin(np.round(tr, 6), gemeinsam); ia = np.isin(np.round(ta, 6), gemeinsam)
print(f"gemeinsame Abtastzeiten im Fenster: {gemeinsam.size} ({gemeinsam.min() if gemeinsam.size else float('nan'):.4f} .. {gemeinsam.max() if gemeinsam.size else float('nan'):.4f} s)")
print(f"{'Groesse':8s} {'Bezug Mittel':>13s} {'Arm Mittel':>11s} | {'Diff gepaart':>12s} {'+- Block-SEM':>12s} {'sigma':>6s} | {'Vorz. +/-':>9s} | Blockmittel Bezug / Arm (50 ms)")
for g in ("Cd_rest", "Cz_rest", "Cd_ges", "Cz_ges"):
    r_, a_ = R[g][ir], Aa[g][ia]; d = a_ - r_
    bd = bloecke(gemeinsam, d); sem = bd.std(ddof=1)/np.sqrt(bd.size) if bd.size > 1 else float("nan")
    br, ba = bloecke(gemeinsam, r_), bloecke(gemeinsam, a_)
    print(f"{g:8s} {r_.mean():+13.4f} {a_.mean():+11.4f} | {d.mean():+12.4f} {sem:12.4f} {abs(d.mean())/sem if sem > 0 else float('nan'):6.1f} | {int((d > 0).sum()):4d}/{int((d < 0).sum()):<4d} | "
          + " ".join(f"{x:+.3f}" for x in br) + "  /  " + " ".join(f"{x:+.3f}" for x in ba))

def kosten(lauf):
    txt = re.sub(r"\x1b\[[0-9;]*m", "", open(os.path.join(WURZEL, "logs", lauf + ".log"), errors="replace").read())
    mlups = []
    for z in txt.splitlines():
        m = re.match(r"\|\s+(\d+) \|\s+(\d+) GB/s \|\s+(\d+) \|\s+(\d+)\s+(\d+)% \|", z)
        if m: mlups.append((int(m.group(4)), int(m.group(1)), int(m.group(2))))
    plan = re.findall(r"SPEICHERPLAN je Domaene: bekannt (\d+) MB", txt)
    grid = re.findall(r"Grid Resolution \|\s+(\d+) x (\d+) x (\d+) = (\d+)", txt)
    return mlups, plan, grid
print("\nKOSTEN (aus den Logs; Fortschrittszeilen zeigen die Domaene, deren Schleife druckt):")
for lauf in (REF, ARM):
    ml, plan, grid = kosten(lauf)
    spaet = [x for x in ml if x[0] > 0]
    med = np.median([x[1] for x in spaet[len(spaet)//3:]]) if spaet else float("nan")
    bw = np.median([x[2] for x in spaet[len(spaet)//3:]]) if spaet else float("nan")
    print(f"  {lauf:10s} Gitter {grid} | Speicherplan je Domaene {plan} MB | MLUPs Median (spaete 2/3 der Zeilen) {med:.0f}, Bandbreite {bw:.0f} GB/s, Zeilen {len(ml)}")
st = open(os.path.join(WURZEL, "logs", "queue_status.txt")).read()
for lauf in (REF, ARM):
    s = re.search(rf"\[(\d\d):(\d\d):(\d\d)\] START \d+/\d+: {lauf} ", st); e = re.search(rf"\[(\d\d):(\d\d):(\d\d)\] ENDE\s+\d+/\d+: {lauf} ", st)
    if s and e:
        sek = lambda m: int(m.group(1))*3600 + int(m.group(2))*60 + int(m.group(3))
        print(f"  Wanduhr {lauf}: {sek(e) - sek(s)} s (Queue START..ENDE, inkl. Aufbau und VTK-Ausgabe)")
if not BAND_GLEICH: print("\n" + BAND_KURZ)
