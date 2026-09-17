#!/usr/bin/env python3
"""lauf_meta.py -- Laufkonstanten fuer die Auswertewerkzeuge AUS DEM LAUF lesen statt hart codieren (17.09.2026).

Heiko 17.09.2026: "Skripte muessen immer passen" -- jedes Werkzeug muss auf 4 / 8 / 3,75 / 16 mm, bei jedem u_lat
(SCHRITTE_PRO_ZELLE 8 -> 0,125, Alt-Serien 0,075) und mit/ohne CFD_Y_VERSATZ richtig rechnen. Befundliste:
SKALIERUNG-BEFUNDE-2026-09-17.md, Nebenbefunde 9-11. Nur lesend; keine Datei wird geschrieben.

GRUNDSATZ: jede Groesse kommt mit ihrer QUELLE zurueck, und das Werkzeug DRUCKT sie (laut statt still). Wo keine
Quelle traegt, gibt es None bzw. einen Abbruch -- nie einen stillen Vorgabewert.

Quellen je Groesse, in dieser Reihenfolge:
  u_lat       Log "Freistrom im Speicherwort: u_lat = X"  (wie zonen_kraft.py seit 17.09.; float32-Wert des Solvers)
              -> Log "Gittergeschwindigkeit u_lat = X"
              -> Log "Unit Conversion: 1 cell = D mm, 1 s = N time steps"  (u_lat = si_u/(N*D), si_u = 30 m/s wie im Solver)
              -> code/LAUF.txt CFD_U_LAT bzw. CFD_SCHRITTE_PRO_ZELLE (u_lat = 1/N)
  y_versatz   Log "Koerper um X mm nach +y versetzt" bzw. "-> kein Versatz"
              -> Log ohne Mittelebenen-Zeile (Binary vor dem Schalter 16.09.): 0 -- auch wenn LAUF.txt CFD_Y_VERSATZ=1 traegt, denn ein
                 Binary ohne die Zeile hat den Versatz NICHT angewandt (dann 0 mit WARNUNG in der Quelle, ★ 17.09. Pruefbefund)
              -> nur OHNE Log: LAUF.txt CFD_Y_VERSATZ + VTK-Kopf nach der Solver-Regel (0,5 dx, nur wenn y = 0 auf einer feinen Zellmitte
                 liegt); ohne VTK UNBEKANNT
  zband       N    Log "KRAFT-ZBAND-KANTE: N = n" -> Log "KRAFT-ZBAND aktiv: unterste N Zellen" -> Kopf von kraft_zband.csv
                   ("# zband_zellen=N", vom Binary geschrieben) -> LAUF.txt CFD_KRAFT_ZBAND NUR, wenn es gar kein Log gibt (Rohwert,
                   mit WARNUNG). Log vorhanden, aber ohne Bandzeile und ohne CSV: N = None (das Binary hat das Band nicht bestaetigt).
                   CFD_KRAFT_ZBAND=0 (LAUF.txt, Log ohne Bandzeile): Band AUS (aus = True, N = 0, KEINE Kante -- ★ 17.09. Pruefbefund:
                   vorher entstand eine negative Kante -0,5 dx).
              dk   Log "Verschiebung dk = K ganze Zellen" bzw. "+ K Zellen Bodenspalt-Verschiebung" (Upstream-Bodenspalt); Log ohne
                   diese Zeile: 0; ohne Log bei einem Upstream-Lauf: UNBEKANNT (0 mit WARNUNG), sonst 0 (v2 kennt keinen Bodenspalt)
              Kante WIRKSAME Oberkante des Kontaktbands: Log "KRAFT-ZBAND-KANTE ... wirksame Oberkante h mm ueber Welt-z = 0"
                   -> Kopf von kraft_zband.csv "oberkante_mm=" (Binaries ab 17.09.)
                   -> Rueckfall (N - 0,5) dx (Befund 1: Zeile z = 0 ist Fahrbahn, wirksam sind z = 1..N-1, Halfway-Wand bei +dx/2).
              In FAHRZEUGKOORDINATEN (= OF13-Koordinaten) liegt die Kante um dk*dx tiefer: der Upstream-Bodenspalt hebt den
              Koerper um dk ganze Zellen an, das Band zaehlt diese Zellen mit ("unterste 6 Zellen = 4 Band + 2 Verschiebung").
"""
import os, re, math

SI_U = 30.0      # m/s -- im Solver fuer alle Faelle fest verdrahtet (Log: "Alle vier Faelle haben si_u = 30 m/s hart verdrahtet")
WURZEL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def lauf_dir_aus(pfad):
    """Laufordner zu einem Ordner export/<lauf> oder einer Datei darin."""
    p = os.path.abspath(pfad)
    return p if os.path.isdir(p) else os.path.dirname(p)


def log_pfad(lauf_dir):
    """logs/<lauf>.log -- zuerst relativ zum Laufordner (export/<lauf>/../../logs, so rechnet zonen_kraft.py), dann Repo-Wurzel."""
    name = os.path.basename(os.path.normpath(lauf_dir))
    for p in (os.path.join(lauf_dir, "..", "..", "logs", name + ".log"), os.path.join(WURZEL, "logs", name + ".log")):
        if os.path.exists(p): return os.path.abspath(p)
    return None


def flach(txt):
    """Konsolen-Box neutralisieren (Werkzeugfalle: Umbrueche verschlucken Zahlen) -- identisch zu zonen_kraft.protokoll."""
    f = re.sub(r"\x1b\[[0-9;]*m", "", txt); f = re.sub(r"\|\s*\n\|\s*", " ", f); return re.sub(r"\s+", " ", f)


_CACHE = {}
def log_flach(lauf_dir):
    p = log_pfad(lauf_dir)
    if p is None: return None, None
    if p not in _CACHE: _CACHE[p] = flach(open(p, errors="replace").read())
    return _CACHE[p], p


def lauf_txt(lauf_dir):
    """CFD_*-Umgebung aus export/<lauf>/code/LAUF.txt (letzter Wert gewinnt), {} wenn nicht vorhanden."""
    p = os.path.join(lauf_dir, "code", "LAUF.txt")
    if not os.path.exists(p): return {}
    return dict(re.findall(r"\b(CFD_[A-Z0-9_]+)=(\S+)", open(p, errors="replace").read()))


def vtk_kopf(pfad):
    """DIMENSIONS/ORIGIN/SPACING eines STRUCTURED_POINTS-VTK (nur der ASCII-Kopf)."""
    with open(pfad, "rb") as f: roh = f.read(2048)
    d = {}
    for z in roh.split(b"\n")[:12]:
        t = z.decode("ascii", "replace").split()
        if not t: continue
        if t[0] == "DIMENSIONS": d["dims"] = tuple(int(v) for v in t[1:4])
        elif t[0] == "ORIGIN": d["orig"] = tuple(float(v) for v in t[1:4])
        elif t[0] == "SPACING": d["spac"] = tuple(float(v) for v in t[1:4])
    if len(d) != 3: raise SystemExit(f"VTK-Kopf unvollstaendig: {pfad}")
    return d


def dx_m(lauf_dir):
    """(feine Zellweite in m oder None, quelle): feld_nah_*.vtk-Kopf -> LAUF.txt CFD_DX -> Kopf kraft_zband.csv dx_mm -> Log 'Unit Conversion'.
    ★ 17.09.2026: die ERSTE 'Unit Conversion'-Zeile ist nicht immer das Nahfeld (p4_pu_ABGEBROCHEN_u075: erst 12 mm, dann 4/16 mm) --
    deshalb vorher der vom Binary geschriebene CSV-Kopf; der Log-Rueckfall bleibt als letzter Weg und nennt sich."""
    vt = sorted(p for p in os.listdir(lauf_dir) if p.startswith("feld_nah_") and p.endswith(".vtk")) if os.path.isdir(lauf_dir) else []
    if vt: return vtk_kopf(os.path.join(lauf_dir, vt[0]))["spac"][0], f"{vt[0]} SPACING"
    env = lauf_txt(lauf_dir)
    if "CFD_DX" in env: return float(env["CFD_DX"])*1e-3, "LAUF.txt CFD_DX"
    pz = os.path.join(lauf_dir, "kraft_zband.csv")
    if os.path.exists(pz):
        with open(pz, errors="replace") as f: m = re.search(r"\bdx_mm=([0-9.]+)", f.readline())
        if m: return float(m.group(1))*1e-3, "Kopf kraft_zband.csv (dx_mm)"
    txt, lp = log_flach(lauf_dir)
    m = re.search(r"Unit Conversion: 1 cell = ([0-9.]+) mm", txt) if txt else None
    if m: return float(m.group(1))*1e-3, f"Log {lp} (erste Unit-Conversion-Zeile -- nicht sicher das Nahfeld)"
    return None, "UNBEKANNT"


def u_lat(lauf_dir):
    """(u_lat, quelle). SystemExit, wenn keine Quelle traegt."""
    txt, lp = log_flach(lauf_dir)
    if txt:
        m = re.search(r"Freistrom im Speicherwort: u_lat = ([0-9.]+)", txt)
        if m: return float(m.group(1)), f"Log {lp} (Freistrom im Speicherwort)"
        m = re.search(r"Gittergeschwindigkeit u_lat = ([0-9.]+)", txt)
        if m: return float(m.group(1)), f"Log {lp} (Gittergeschwindigkeit u_lat)"
        m = re.search(r"Unit Conversion: 1 cell = ([0-9.]+) mm, 1 s = ([0-9.]+) time steps", txt)
        if m:
            return SI_U/(float(m.group(2))*float(m.group(1))*1e-3), f"Log {lp} (Unit Conversion {m.group(1)} mm / {m.group(2)} Schritte je s, si_u {SI_U})"
    env = lauf_txt(lauf_dir)
    if "CFD_U_LAT" in env: return float(env["CFD_U_LAT"]), "LAUF.txt CFD_U_LAT"
    if "CFD_SCHRITTE_PRO_ZELLE" in env: return 1.0/float(env["CFD_SCHRITTE_PRO_ZELLE"]), "LAUF.txt CFD_SCHRITTE_PRO_ZELLE (1/N)"
    raise SystemExit(f"FEHLER: u_lat fuer {lauf_dir} nicht bestimmbar (kein Log {os.path.basename(lauf_dir)}.log mit u_lat-Zeile, "
                     "keine LAUF.txt mit CFD_U_LAT/CFD_SCHRITTE_PRO_ZELLE) -- kein stiller Vorgabewert (Falle 17.09.: Faktor 2,78)")


def y_versatz_m(lauf_dir, vtk=None):
    """(versatz in m oder None, quelle). Der Versatz wirkt auf den KOERPER, nicht auf ORIGIN: Mittelebene bei y = +versatz."""
    txt, lp = log_flach(lauf_dir)
    env = lauf_txt(lauf_dir)
    if txt:
        m = re.search(r"Koerper um ([0-9.]+) mm nach \+y versetzt", txt)
        if m: return float(m.group(1))*1e-3, f"Log {lp}"
        if re.search(r"CFD_Y_VERSATZ = \d+ -> kein Versatz", txt): return 0.0, f"Log {lp} (kein Versatz)"
        if env.get("CFD_Y_VERSATZ", "0") in ("", "0"):
            return 0.0, f"Log {lp} ohne Mittelebenen-Zeile (Lauf vor dem Schalter 16.09.), LAUF.txt ohne CFD_Y_VERSATZ=1"
        # ★ 17.09.2026 (Pruefbefund): Log da, Zeile fehlt, LAUF.txt sagt Versatz -- das Binary kannte den Schalter nicht und hat den
        # Koerper NICHT versetzt. Frueher lief dieser Fall still in die VTK-Regel (0,5 dx) -- eine halbe Zelle daneben.
        return 0.0, (f"WARNUNG: LAUF.txt CFD_Y_VERSATZ={env['CFD_Y_VERSATZ']}, aber Log {lp} ohne Mittelebenen-Zeile -- das Binary hat den "
                     "Versatz NICHT angewandt (Schalter unbekannt) -> 0, nicht 0,5 dx")
    if "CFD_Y_VERSATZ" in env:
        if env["CFD_Y_VERSATZ"] in ("", "0"): return 0.0, "LAUF.txt CFD_Y_VERSATZ=0 (kein Log)"
        if vtk is None:
            return None, f"UNBEKANNT (kein Log; LAUF.txt CFD_Y_VERSATZ={env['CFD_Y_VERSATZ']}, VTK fehlt fuer die Solver-Regel)"
        k = vtk_kopf(vtk); j = -k["orig"][1]/k["spac"][1]
        auf_mitte = abs(j - round(j)) < 1e-3
        return (0.5*k["spac"][1] if auf_mitte else 0.0), f"LAUF.txt CFD_Y_VERSATZ={env['CFD_Y_VERSATZ']} + Solver-Regel am VTK-Kopf (y=0 auf {'Zellmitte' if auf_mitte else 'Zellflaeche'}; kein Log)"
    return None, "UNBEKANNT (kein Log, keine LAUF.txt mit CFD_Y_VERSATZ)"


def bodenspalt(lauf_dir):
    """(dk ganze Zellen oder None, quelle) -- Upstream-Bodenspalt: der Koerper liegt um dk*dx hoeher als in Fahrzeug-/OF13-Koordinaten.
    Koerper-z = Welt-z - dk*dx."""
    txt, lp = log_flach(lauf_dir)
    if txt:
        m = re.search(r"Verschiebung dk = (\d+) ganze Zellen", txt) or re.search(r"KRAFT-ZBAND aktiv: unterste \d+ Zellen = \d+ Zellen Band [^+]{0,40}\+ (\d+) Zellen Bodenspalt", txt)
        if m: return int(m.group(1)), f"Log {lp} (Bodenspalt-Verschiebung)"
        return 0, f"Log {lp} ohne Bodenspalt-Zeile (kein Bodenspalt)"
    if ist_upstream(lauf_dir, None):
        return None, "UNBEKANNT (Upstream-Lauf ohne Log -- dk haengt an der Geometrie, nicht an LAUF.txt)"
    return 0, "kein Log; v2-Lauf (kein Bodenspalt-Schalter)"


def ist_upstream(lauf_dir, txt):
    lt = os.path.join(lauf_dir, "code", "LAUF.txt")
    if txt and (("Upstream-Voxelizer" in txt) or ("FluidX3D-upstream" in txt)): return True
    return os.path.exists(lt) and "FluidX3D-Upstream" in open(lt, errors="replace").read(400)


def zband_csv_kopf(lauf_dir):
    """(N oder None, oberkante_m oder None) aus der ersten Kopfzeile von kraft_zband.csv (vom Binary geschrieben)."""
    p = os.path.join(lauf_dir, "kraft_zband.csv")
    if not os.path.exists(p): return None, None
    with open(p, errors="replace") as f: z = f.readline()
    n = re.search(r"zband_zellen=(\d+)", z); k = re.search(r"oberkante_mm=(-?[0-9.]+)", z)
    return (int(n.group(1)) if n else None), (float(k.group(1))*1e-3 if k else None)


def zband(lauf_dir, dx):
    """Kontaktband des Laufs. dict(N, dk, census, kante_welt_m, kante_fz_m, quelle_N, quelle_kante, upstream, aus, dk_quelle, warnung).
    N = None, wenn keine Quelle traegt (dann auch keine Kante). aus = True: CFD_KRAFT_ZBAND=0 -- N = 0 und KEINE Kante."""
    txt, lp = log_flach(lauf_dir)
    env = lauf_txt(lauf_dir)
    out = dict(N=None, dk=0, census=None, kante_welt_m=None, kante_fz_m=None, quelle_N=None, quelle_kante=None, upstream=False,
               aus=False, dk_quelle=None, warnung=[])
    kante_log = None
    if txt:
        mk = re.search(r"KRAFT-ZBAND-KANTE: N = (\d+) Zellen.{0,200}?wirksame Oberkante (-?[0-9.]+) mm", txt)
        ma = re.search(r"KRAFT-ZBAND aktiv: unterste (\d+) Zellen", txt)
        if mk:
            out["N"] = int(mk.group(1)); out["quelle_N"] = f"Log {lp} (KRAFT-ZBAND-KANTE)"; kante_log = float(mk.group(2))*1e-3
        elif ma:
            out["N"] = int(ma.group(1)); out["quelle_N"] = f"Log {lp} (KRAFT-ZBAND aktiv)"
        b = re.search(r"Band-Census 0x41: (\d+) von \d+ Zellen", txt)
        if b: out["census"] = int(b.group(1))
    dk, out["dk_quelle"] = bodenspalt(lauf_dir)
    if dk is None:
        out["warnung"].append(f"Bodenspalt dk {out['dk_quelle']} -- 0 angenommen, Fahrzeugkoordinaten ggf. um dk*dx falsch")
        dk = 0
    out["dk"] = dk
    out["upstream"] = ist_upstream(lauf_dir, txt)
    n_csv, kante_csv = zband_csv_kopf(lauf_dir)
    if out["N"] is None and n_csv is not None:
        out["N"] = n_csv; out["quelle_N"] = "Kopf kraft_zband.csv (zband_zellen, vom Binary geschrieben; Log ohne KRAFT-ZBAND-Zeile)"
        if kante_csv is not None: kante_log = kante_csv
    if out["N"] is None and "CFD_KRAFT_ZBAND" in env:
        roh = env["CFD_KRAFT_ZBAND"]
        if roh in ("", "0"):
            # ★ 17.09.2026 (Pruefbefund): 0 = Band AUS -- keine Kante (vorher (0 - 0,5) dx = negative Kante, OF13 rechnete "z < -2 mm")
            out["N"] = 0; out["aus"] = True
            out["quelle_N"] = f"LAUF.txt CFD_KRAFT_ZBAND={roh or '(leer)'}" + (f", Log {lp} ohne Bandzeile" if txt else " (kein Log)")
            return out
        if txt:
            # ★ 17.09.2026 (Pruefbefund): Log da, aber ohne Bandzeile -- das Binary hat das Band nicht bestaetigt (altes Binary oder
            # abgeschnittenes Log). Frueher still der LAUF.txt-Rohwert.
            out["warnung"].append(f"Log {lp} ohne KRAFT-ZBAND-Zeile und keine kraft_zband.csv, LAUF.txt sagt CFD_KRAFT_ZBAND={roh} -- "
                                  "vom Binary NICHT bestaetigt, N unbekannt")
            out["quelle_N"] = f"UNBESTAETIGT (LAUF.txt CFD_KRAFT_ZBAND={roh}, Log ohne Bandzeile)"
            return out
        out["N"] = int(roh); out["quelle_N"] = "LAUF.txt CFD_KRAFT_ZBAND (kein Log, keine kraft_zband.csv -- Rohwert, nicht die Solver-Zeile)"
        out["warnung"].append(f"N = {roh} nur aus LAUF.txt (kein Log, keine kraft_zband.csv) -- vom Binary nicht bestaetigt")
    if out["N"] is None: return out
    if kante_log is not None:
        out["kante_welt_m"] = kante_log
        out["quelle_kante"] = "Log-Zeile KRAFT-ZBAND-KANTE (wirksame Oberkante)" if n_csv is None or out["quelle_N"].startswith("Log") else "Kopf kraft_zband.csv (oberkante_mm)"
    else:
        out["kante_welt_m"] = (out["N"] - 0.5)*dx
        out["quelle_kante"] = f"RUECKFALL (N - 0,5) dx = ({out['N']} - 0,5) x {dx*1e3:.4g} mm (Log ohne KRAFT-ZBAND-KANTE-Zeile)"
    # Fahrzeugkoordinaten: im Rueckfall direkt (N - dk - 0,5) dx (keine Float-Differenz), sonst Welt-Kante minus dk*dx
    out["kante_fz_m"] = (out["N"] - out["dk"] - 0.5)*dx if kante_log is None else out["kante_welt_m"] - out["dk"]*dx
    if out["dk"]:
        out["quelle_kante"] += f"; Bodenspalt dk = {out['dk']} Zellen -> Fahrzeugkoordinaten {out['kante_welt_m']*1e3:.3f} - {out['dk']*dx*1e3:.3f} mm"
    return out


def zband_text(zb):
    """Eine Zeile zum Drucken (Kanten mit 3 Nachkommastellen: 13,125 mm darf nicht als 13,12 erscheinen)."""
    w = "".join(f" WARNUNG: {t}." for t in zb.get("warnung", []))
    if zb.get("aus"): return f"KRAFT-ZBAND AUS ({zb['quelle_N']}) -- kein Kontaktband, keine Bandkante; *_rest = Gesamt" + w
    if zb["N"] is None: return f"KRAFT-ZBAND: keine Quelle ({zb['quelle_N'] or 'weder Log-Zeile noch kraft_zband.csv noch LAUF.txt'}) -- Band unbekannt" + w
    return (f"KRAFT-ZBAND N = {zb['N']} ({zb['quelle_N']}), dk = {zb['dk']}, wirksame Oberkante Welt {zb['kante_welt_m']*1e3:.3f} mm, "
            f"Fahrzeug/OF13 {zb['kante_fz_m']*1e3:.3f} mm [{zb['quelle_kante']}]" + (" [Upstream-Lauf]" if zb["upstream"] else "") + w)


def band_vergleich(laeufe):
    """Kontaktband mehrerer Laeufe fuer Paar-/Fensterwerkzeuge (★ 17.09.2026, Pruefbefund 3): cd/cz_druck_rest (cd_facetten.csv) und
    *_rest (kraft_zband.csv) haengen an N -- seit 17.09. N = max(3, ceil(16/dx)) (8 mm 3, 3,75 mm 5), alte Laeufe 8 mm 2, 3,75 mm 4.
    laeufe: Liste (name, lauf_dir). Rueckgabe (zeilen, gleich, kurz): zeilen = Kopfzeilen zum Drucken (mit WARNUNG bei Abweichung),
    gleich = alle Kanten bekannt und gleich (Fahrzeugkoordinaten, 1 um), kurz = einzeilige Wiederholung fuers Ende (leer, wenn gleich)."""
    zeilen, sig = [], []
    for name, d in laeufe:
        dx, qdx = dx_m(d)
        zb = zband(d, dx if dx is not None else float("nan"))
        if dx is None and not zb.get("aus"):
            zeilen.append(f"BANDKANTE {name}: dx UNBEKANNT ({qdx}) -- " + zband_text(zb)); sig.append(None); continue
        zeilen.append(f"BANDKANTE {name}" + (f" (dx {dx*1e3:.3f} mm)" if dx is not None else "") + ": " + zband_text(zb))
        if zb.get("aus"): sig.append("AUS")
        elif zb["N"] is None or zb["kante_fz_m"] is None or zb["warnung"]: sig.append(None)
        else: sig.append((int(round(zb["kante_fz_m"]*1e6)), zb["N"], zb["dk"]))
    kanten = {s[0] if isinstance(s, tuple) else s for s in sig}
    if None in kanten:
        kurz = ("WARNUNG BANDKANTE: mindestens ein Lauf ohne gesicherte Bandkante (" + ", ".join(n for (n, _), s in zip(laeufe, sig) if s is None)
                + ") -- *_rest-Vergleich UNGESICHERT")
        zeilen.append("!" * 100); zeilen.append(kurz); zeilen.append("!" * 100)
        return zeilen, False, kurz
    if len(kanten) > 1:
        kurz = ("WARNUNG BANDKANTE VERSCHIEDEN: " + " | ".join(f"{n} " + ("AUS" if s == "AUS" else f"{s[0]/1e3:.3f} mm (N {s[1]}, dk {s[2]})")
                                                          for (n, _), s in zip(laeufe, sig))
                + " -- cd/cz_druck_rest und *_rest zerlegen VERSCHIEDENE Koerperbereiche; Differenzen enthalten den Bandunterschied, KEIN reiner Arm-Effekt")
        zeilen.append("!" * 100); zeilen.append(kurz); zeilen.append("!" * 100)
        return zeilen, False, kurz
    if len({s for s in sig}) > 1:
        zeilen.append("HINWEIS BANDKANTE: gleiche wirksame Kante (Fahrzeugkoordinaten), aber verschiedene Zellzahl N/dk -- Bandbereich gleich, Gitter verschieden")
    else:
        zeilen.append("BANDKANTE gleich in allen Laeufen: " + ("AUS" if sig[0] == "AUS" else f"{sig[0][0]/1e3:.3f} mm (Fahrzeug/OF13)"))
    return zeilen, True, ""


if __name__ == "__main__":
    import sys
    for a in sys.argv[1:]:
        d = lauf_dir_aus(a if os.path.exists(a) else os.path.join(WURZEL, "export", a))
        vt = sorted(p for p in os.listdir(d) if p.startswith("feld_nah_") and p.endswith(".vtk")) if os.path.isdir(d) else []
        vtk = os.path.join(d, vt[0]) if vt else None
        dx = vtk_kopf(vtk)["spac"][0] if vtk else float(lauf_txt(d).get("CFD_DX", "nan"))*1e-3
        print(f"== {os.path.basename(d)}  dx {dx*1e3:.4g} mm")
        try: print("  u_lat", u_lat(d))
        except SystemExit as e: print("  u_lat", e)
        print("  y_versatz", y_versatz_m(d, vtk))
        print("  " + zband_text(zband(d, dx)))
