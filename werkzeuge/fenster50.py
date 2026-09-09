#!/usr/bin/env python3
"""fenster50.py -- 50-ms-Fenstermittel der REST-Kraftbeiwerte, gepaart ueber Zeitschritte.

Heiko-Vorgabe 09.09.2026: "alle 50 ms ab 200 ms die gemittelten Cd und Cz Rest, passend von
den Timesteps zu den Vergleichslaeufen."

Warum REST und nicht Gesamt: der Band-Anteil ist die kuenstliche Reifenaufpraegung (~-0,7 Cz);
die Aussage traegt der Rest. Warum gepaart ueber Zeitschritte: die Laeufe schreiben ihre erste
Zeile nicht am selben ms (km_s4_sism ab 0,202 s, p4_nb ab 0,201 s). Ungepaart gemittelt
vergleicht man verschiedene Zeitpunkte und haelt den Versatz fuer ein Signal.

Aufruf:  werkzeuge/fenster50.py LAUF_REFERENZ LAUF2 [LAUF3 ...]
         --ab 0.200 --breite 0.050 --bloecke 5 --csv DATEI
Der ERSTE Lauf ist die Referenz; fuer alle weiteren wird zusaetzlich die gepaarte Differenz
je Fenster mit Block-SEM ausgewiesen.
"""
import sys, os, csv, math, argparse

WURZEL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Groesse -> (Datei, Spaltenname). Reihenfolge = Ausgabereihenfolge.
# STANDARD sind die ZWEI Groessen des Laufberichts. Ihre Definition steht in setup.cpp:7709:
#   ber_cd = cdg - cdb  und  ber_cz = czg - czb, mit cdg = si_F(FK.px)/(q_inf*A_ref).
# Das sind ZEICHENGLEICH die Spalten cd_druck_rest / cz_druck_rest in cd_facetten.csv -- der
# Laufbericht nennt sie nur "cd_rest"/"cz_rest". Wer die Reibungsanteile oder den Gesamt-Cz_rest
# aus kraft_zband.csv sehen will, gibt --mehr an; die Aussage tragen die zwei oben.
GROESSEN = [
    ("Cd_rest (= cd_druck_rest)", "cd_facetten.csv", "cd_druck_rest"),
    ("Cz_rest (= cz_druck_rest)", "cd_facetten.csv", "cz_druck_rest"),
]
GROESSEN_MEHR = [
    ("cd_reib",         "cd_facetten.csv", "cd_reib"),
    ("cz_reib",         "cd_facetten.csv", "cz_reib"),
    ("Cz_rest_gesamt",  "kraft_zband.csv", "Cz_rest"),
]

def lies(lauf, datei, spalte):
    """{ms(int): wert(float)} aus export/<lauf>/<datei>. Kommentarzeilen (#) uebersprungen."""
    pfad = os.path.join(WURZEL, "export", lauf, datei)
    if not os.path.isfile(pfad): return None, f"fehlt: {pfad}"
    with open(pfad) as fh:
        zeilen = [z for z in fh if not z.lstrip().startswith("#")]
    if not zeilen: return None, f"leer: {pfad}"
    r = csv.DictReader(zeilen)
    if spalte not in (r.fieldnames or []):
        return None, f"Spalte '{spalte}' fehlt in {datei} (hat: {','.join(r.fieldnames or [])})"
    d = {}
    for z in r:
        try:
            t = float(z["time_s"]); v = float(z[spalte])
        except (TypeError, ValueError):
            continue
        if math.isfinite(v): d[int(round(t*1000.0))] = v   # Schluessel: ganze ms
    return d, None

def mittel(xs): return sum(xs)/len(xs) if xs else float("nan")

def sd(xs):
    if len(xs) < 2: return float("nan")
    m = mittel(xs); return math.sqrt(sum((x-m)**2 for x in xs)/(len(xs)-1))

def block_sem(diffs, nb):
    """SEM der gepaarten Differenz ueber nb gleich grosse Bloecke -- gegen Autokorrelation.
    Weniger als 2 besetzte Bloecke: kein SEM (nan), nicht etwa 0."""
    if len(diffs) < nb or nb < 2: return float("nan"), 0
    gr = len(diffs)//nb
    mb = [mittel(diffs[i*gr:(i+1)*gr]) for i in range(nb)]
    mb = [m for m in mb if math.isfinite(m)]
    if len(mb) < 2: return float("nan"), len(mb)
    return sd(mb)/math.sqrt(len(mb)), len(mb)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("laeufe", nargs="+")
    ap.add_argument("--ab", type=float, default=0.200)
    ap.add_argument("--breite", type=float, default=0.050)
    ap.add_argument("--bloecke", type=int, default=5)
    ap.add_argument("--csv", default="")
    ap.add_argument("--mehr", action="store_true", help="zusaetzlich Reibungsanteile und Cz_rest gesamt")
    a = ap.parse_args()

    global GROESSEN
    if a.mehr: GROESSEN = GROESSEN + GROESSEN_MEHR

    ab_ms, br_ms = int(round(a.ab*1000)), int(round(a.breite*1000))
    ref = a.laeufe[0]

    # --- einlesen ---
    daten = {}   # (lauf, groesse) -> {ms: wert}
    fehlend = []
    for lauf in a.laeufe:
        for name, datei, spalte in GROESSEN:
            d, err = lies(lauf, datei, spalte)
            if err: fehlend.append(f"  {lauf}/{name}: {err}")
            daten[(lauf, name)] = d or {}

    if fehlend:
        print("HINWEIS -- nicht gelesen (die betroffenen Zeilen bleiben leer):")
        print("\n".join(fehlend)); print()

    # --- gemeinsame Zeitschritte JE GROESSE (Iron Rule: gepaart heisst gleicher Zeitschritt) ---
    print(f"Laeufe: {' | '.join(a.laeufe)}   (Referenz: {ref})")
    print(f"Fenster: {br_ms} ms ab {ab_ms} ms, gepaart ueber gemeinsame Zeitschritte, "
          f"Block-SEM ueber {a.bloecke} Bloecke je Fenster\n")

    ausgabe = []
    teil_gesehen = False
    for name, _, _ in GROESSEN:
        reihen = [daten[(l, name)] for l in a.laeufe]
        if any(not r for r in reihen):
            print(f"### {name}: uebersprungen (mindestens ein Lauf ohne Daten)\n"); continue
        gemeinsam = sorted(set.intersection(*[set(r.keys()) for r in reihen]))
        gemeinsam = [t for t in gemeinsam if t >= ab_ms]
        if not gemeinsam:
            print(f"### {name}: keine gemeinsamen Zeitschritte ab {ab_ms} ms\n"); continue

        # Deckungsprobe: je Lauf die Punkte ab ab_ms, damit sichtbar ist, WER kuerzer ist
        # (ein noch laufender Arm ist kurz, ein Versatz der ersten Zeile ist etwas anderes).
        einzeln = [(l, len([t for t in r if t >= ab_ms])) for l, r in zip(a.laeufe, reihen)]
        je = ", ".join(f"{l[:16]} {n}" for l, n in einzeln)
        kuerzest = min(n for _, n in einzeln)
        versatz = kuerzest - len(gemeinsam)   # verloren TROTZ gleicher Laenge = echter Zeitversatz
        hinweis = f"  [{len(gemeinsam)} gepaarte Zeitschritte | je Lauf ab {ab_ms} ms: {je}"
        hinweis += f" | {versatz} durch Zeitversatz verworfen]" if versatz else " | kein Zeitversatz]"
        print(f"### {name}{hinweis}")

        kopf = f"{'Fenster [ms]':>14} {'n':>4}  "
        for l in a.laeufe:
            kopf += f" {l[:16]:>17}"
            if l != ref: kopf += f" {'Delta+-BlockSEM':>21} {'sigma':>6}"
        print(kopf)

        t_max = max(gemeinsam)
        start = ab_ms
        while start <= t_max:
            ende = start + br_ms
            fenster = [t for t in gemeinsam if start <= t < ende]
            # Den Rest NUR anhaengen, wenn er kurz ist (der bekannte Fall: 450-500 plus die eine
            # 501-ms-Probe). Frueher hing hier jeder Rest am letzten Fenster -- bei einem noch
            # LAUFENDEN Arm verschmolz dadurch das erste Fenster mit dem Anbruch zu "200-262",
            # und die 50-ms-Aufloesung war weg, ohne dass es auffiel.
            rest = [t for t in gemeinsam if t >= ende]
            if rest and len(rest) <= max(1, br_ms//5):
                fenster = fenster + rest; ende = max(rest) + 1
            teil = ""
            if not fenster: break
            if ende - start < br_ms or (start + br_ms > t_max + 1):
                teil = " *"   # angebrochenes Fenster: der Lauf endet hier (oder laeuft noch)
                teil_gesehen = True
            zeile = f"{start:>6}-{ende:<7} {len(fenster):>4}{teil:<2}"
            m_ref = mittel([daten[(ref, name)][t] for t in fenster])
            for l in a.laeufe:
                xs = [daten[(l, name)][t] for t in fenster]
                zeile += f" {mittel(xs):>17.5f}"
                if l != ref:
                    diffs = [daten[(l, name)][t] - daten[(ref, name)][t] for t in fenster]
                    dm = mittel(diffs); se, nb = block_sem(diffs, a.bloecke)
                    if math.isfinite(se) and se > 0.0:
                        zeile += f" {dm:>+11.5f}+-{se:.5f} {abs(dm)/se:>5.1f}s"
                    else:
                        zeile += f" {dm:>+11.5f}+-  n/a    n/a"
                    ausgabe.append((name, start, ende, l, ref, m_ref, mittel(xs), dm, se, nb, len(fenster)))
            print(zeile)
            start = ende
        print()

    if teil_gesehen:
        print("* = angebrochenes Fenster: der Lauf endet dort oder laeuft noch. "
              "Diese Zeile ist KEIN 50-ms-Mittel und nicht mit den vollen Fenstern vergleichbar.\n")

    if a.csv:
        with open(a.csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["groesse","fenster_ab_ms","fenster_bis_ms","lauf","referenz",
                        "mittel_referenz","mittel_lauf","delta","block_sem","n_bloecke","n_proben"])
            for r in ausgabe: w.writerow([f"{x:.6g}" if isinstance(x,float) else x for x in r])
        print(f"CSV geschrieben: {a.csv}")

if __name__ == "__main__":
    main()
