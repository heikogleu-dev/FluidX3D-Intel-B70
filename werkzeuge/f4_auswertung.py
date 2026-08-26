#!/usr/bin/env python3
# f4_auswertung.py — Standard-Abnahmeblick fuer einen fahrzeug_dd-Lauf (gebaut 26.08.2026
# nachts fuer f4_vollumfang_mls, laeuft aber fuer jeden dd-Lauf mit forces.csv + Log).
# Iron Rule 5: alle Zahlen aus Feld-CSVs bzw. dem Laufprotokoll, umbruchfest geparst
# (Werkzeugfalle: die Konsolen-Box bricht Zeilen um und verschluckt grep-Zahlen).
# Aufruf: werkzeuge/f4_auswertung.py <run_name> [vergleichs_run]   (Default-Vergleich: f4_wandfrei_v2)
import sys, re, csv, os, statistics

OF13_CD, OF13_CZ = 0.599, -1.301  # Referenz OpenFOAM 13 (~/CFD-Cases/mr2v40H)
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def fenster(run, tmin=0.2):
    p = os.path.join(REPO, "export", run, "forces.csv")
    rows = [r for r in csv.DictReader(open(p)) if float(r["time_s"]) >= tmin]
    if not rows: raise SystemExit(f"FEHLER: keine Samples t>={tmin} in {p}")
    cd = [float(r["Cd"]) for r in rows]; cz = [float(r["Cz"]) for r in rows]
    def blocksem(v, nb=8):
        bs = max(1, len(v)//nb); mittel = [statistics.mean(v[i*bs:(i+1)*bs]) for i in range(nb) if v[i*bs:(i+1)*bs]]
        return statistics.stdev(mittel)/ (len(mittel)**0.5) if len(mittel) > 1 else float("nan")
    return dict(n=len(rows), t0=float(rows[0]["time_s"]), t1=float(rows[-1]["time_s"]),
                cd=statistics.mean(cd), cz=statistics.mean(cz), cd_sem=blocksem(cd), cz_sem=blocksem(cz))

def log_flach(run):
    p = os.path.join(REPO, "logs", run + ".log")
    if not os.path.exists(p): return ""
    t = open(p, errors="replace").read().replace("\n", "").replace("|", "")
    return re.sub(r"\s+", " ", t)  # Box-Umbrueche neutralisiert

def zieh(txt, muster, name, fmt=lambda m: m.group(1)):
    m = re.search(muster, txt)
    print(f"  {name}: " + (fmt(m) if m else "NICHT GEFUNDEN -- im Log nachsehen!"))
    return m

run = sys.argv[1] if len(sys.argv) > 1 else "f4_vollumfang_mls"
ref = sys.argv[2] if len(sys.argv) > 2 else "f4_wandfrei_v2"

print(f"=== ABNAHMEBLICK {run} (Fenstermittel t>=0,2 s aus forces.csv) ===")
try:
    f = fenster(run)
    print(f"  Fenster {f['t0']:.3f}..{f['t1']:.3f} s, n={f['n']}")
    print(f"  Cd = {f['cd']:.4f} +- {f['cd_sem']:.4f} (Block-SEM 8)   [OF13: {OF13_CD}]")
    print(f"  Cz = {f['cz']:+.4f} +- {f['cz_sem']:.4f} (Block-SEM 8)   [OF13: {OF13_CZ}]")
    try:
        r = fenster(ref)
        print(f"  Vergleich {ref}: Cd {r['cd']:.4f} -> {f['cd']:.4f} (Delta {f['cd']-r['cd']:+.4f}), "
              f"Cz {r['cz']:+.4f} -> {f['cz']:+.4f} (Delta {f['cz']-r['cz']:+.4f})")
        sig = abs(f['cz']-r['cz']) / max(1e-12, (f['cz_sem']**2 + r['cz_sem']**2)**0.5)
        print(f"  Cz-Delta / kombinierte Block-SEM = {sig:.1f}  (>3 = deutlich ausserhalb des Rauschens)")
    except (FileNotFoundError, SystemExit, KeyError, TypeError):
        print(f"  (Vergleichslauf {ref} nicht lesbar -- uebersprungen)")
except (FileNotFoundError, SystemExit, KeyError, TypeError) as e:
    print(f"  forces.csv nicht lesbar/fremdes Format ({e}) -- CSV-Teil uebersprungen, Werkzeug zielt auf dd-Laeufe.")

print("=== WAECHTER & WIRKPFADE (aus dem Log, umbruchfest) ===")
t = log_flach(run)
if not t:
    print("  KEIN LOG gefunden -- nur CSV-Teil verfuegbar."); sys.exit(0)
# ELIBB-Wirkpfade: neues Format (ab 468b183) mit MLS[68], Altformat ohne -- beide lesbar
m = re.search(r"ELIBB\[67\] (\d+), MLS\[68\] (\d+)", t) or re.search(r"ELIBB\[67\] (\d+)", t)
if m and m.lastindex == 2:
    print(f"  Wirkpfad ELIBB[67] / MLS[68]: {m.group(1)} / {m.group(2)}"
          + ("   <-- MLS[68]=0: q>0,5-Zweig feuerte NIE, pruefen!" if m.group(2) == "0" else "  (beide > 0 = Kette nachweislich aktiv)"))
elif m: print(f"  Wirkpfad ELIBB[67]: {m.group(1)} (Altformat ohne MLS[68] -- Lauf vor 468b183)")
else:   print("  Wirkpfad ELIBB[67]/MLS[68]: NICHT GEFUNDEN -- ELIBB aus oder Log unvollstaendig!")
m = re.search(r"FELD-HASH\(u\) = (\d+)", t)
print(f"  FELD-HASH: {m.group(1)}" if m else "  FELD-HASH: nicht vorhanden (nur Kanal-Laeufe drucken ihn)")
print("  Facetten Ist=Soll-Hardcheck: " + ("FEHLERTEXT IM LOG -- pruefen!" if ("Ist != Soll" in t or "Lookup oder Bindung defekt" in t) else "kein Fehlertext (= bestanden; print_error haette abgebrochen)"))
zieh(t, r"Delta-m = (-?[0-9.]+)", "iMEM Delta-m")
zieh(t, r"(KIPP-WAECHTER AUSGELOEST[^.]*)", "Kipp-Waechter", lambda m: m.group(1)) if "AUSGELOEST" in t else print("  Kipp-Waechter: nicht ausgeloest")
if "K2 verletzt" in t: print("  K2: VERLETZT (am dd-Fall unerwartet -- pruefen)")
vram = [int(x) for x in re.findall(r"Memory Usage CPU \d+ MB, GPU 1x (\d+) MB", t)]
print(f"  VRAM B70 belegt (MB): {max(vram)} (Feindomaene; alle Domaenen: {vram})" if vram else "  VRAM: Memory-Usage-Zeile nicht gefunden")
m = re.findall(r" (\d+) \s*(\d+) GB/s ", t)
if m: print(f"  MLUPs final (kumulativ): {m[-1][0]}, {m[-1][1]} GB/s")
for w in ("Stoppdatei", "VTK-Feld-Dump", "Engine reset"):
    if w.lower() in t.lower(): print(f"  HINWEIS: '{w}' kommt im Log vor -- Kontext lesen.")
print("=== Ende Abnahmeblick ===")
