#!/usr/bin/env python3
"""Kraftverlauf als Bild: cd_rest und cz_rest oben, Aenderungsrate unten.

    werkzeuge/kraftverlauf.py LAUF [LAUF ...]
        --raster 10      Zeitraster in ms (Vorgabe 10)
        --fenster 100    Bezugsfenster der Aenderungsrate in ms (Vorgabe 100)
        --roh            Punktabtastung statt Fenstermittel (siehe unten)
        --von / --bis    Zeitgrenzen in ms
        --serie 100      Standbild-Serie: alle 100 ms physikalisch ein Bild nach
                         export/<lauf>/kraftverlauf_000300ms.png usw., dazu ein
                         Abschlussbild kraftverlauf.png ueber den ganzen Lauf.
                         Je Lauf ein eigener Ordner; schliesst --aus aus.
        --aus DATEI.png  Ausgabedatei (nur ohne --serie)

WAS ES ZEIGT
  Oben:  cd_rest (durchgezogen) und cz_rest (gestrichelt) ueber der Zeit.
  Unten: die Aenderung gegenueber dem Wert VOR <fenster> ms, in Prozent:
         100 * (x(t) - x(t-fenster)) / |x(t-fenster)|.
         Das ist ein Einschwing-Mass: geht es gegen null, steht die Groesse.

ZWEI DINGE, DIE MAN WISSEN MUSS, BEVOR MAN DAS BILD DEUTET

1. ES GIBT KEINE DATEN VOR DEM WARMUP. cd_druck_rest und cz_druck_rest sind
   laut Kopfzeile von cd_facetten.csv KUMULATIV SEIT DEM WARMUP. Bei
   CFD_T_WARMUP=0.201 beginnt die Datei bei 202 ms. Ein Diagramm ab 0 ms ist
   fuer diese zwei Groessen nicht herstellbar; die Achse beginnt dort, wo die
   Daten beginnen. (forces.csv reicht zwar ab 1 ms, traegt aber Cd/Cz aus
   object_force -- mit Phantomreibung und ohne Bandabzug, also NICHT dasselbe.)

2. DIE SCHREIBKADENZ HAENGT AM ZEITSCHRITT, NICHT AN DER UHR. Der Loeser legt
   alle 25 groben (= 100 feinen) Schritte eine Zeile ab. Physikalisch sind das
   1,00 ms bei 4 mm und 2,00 ms bei 8 mm -- die grobe Sprosse tastet also nur
   halb so dicht ab (Nyquist 250 statt 500 Hz).

3. DIE MOMENTANWERTE SCHWANKEN STARK. cz_druck_rest springt von Abtastung zu
   Abtastung zwischen etwa -0,63 und -1,51 (gemessen an p4_voll). Eine
   Aenderungsrate auf Punktabtastungen waere daher fast reines Rauschen.
   Deshalb ist die Vorgabe das MITTEL ueber das jeweilige Rasterfenster, nicht
   der Einzelwert. --roh schaltet auf Punktabtastung um, dann steht es auch so
   im Bild.
"""
import sys, os, csv, argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WURZEL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# cz durchgezogen, cd gestrichelt (Heiko-Vorgabe 10.09.). Eigene Achse je Groesse:
# cd liegt bei 0,5 und cz bei -1,1 -- auf einer Achse waere die Struktur des einen
# im Massstab des anderen nicht mehr lesbar.
GROESSEN = [("cd_druck_rest", "cd_rest", "--", "links"),
            ("cz_druck_rest", "cz_rest", "-", "rechts")]
ACHSE = {"cz_rest": (-1.5, 0.0), "cd_rest": (0.0, 0.75)}  # cz invertiert: 0 oben
RATE_GRENZE = 15.0


def lies(lauf):
    """Zeitreihe aus cd_facetten.csv, Zeit in ms."""
    p = os.path.join(WURZEL, "export", lauf, "cd_facetten.csv")
    if not os.path.exists(p):
        raise SystemExit(f"{lauf}: cd_facetten.csv fehlt ({p})")
    t, sp = [], {k: [] for k, _, _, _ in GROESSEN}
    with open(p) as f:
        for row in csv.DictReader(r for r in f if not r.startswith("#")):
            try:
                zeit = float(row["time_s"]) * 1000.0
            except (TypeError, ValueError, KeyError):
                continue
            werte = {}
            for k, _, _, _ in GROESSEN:
                try:
                    werte[k] = float(row[k])
                except (TypeError, ValueError, KeyError):
                    werte = None
                    break
            if werte is None:
                continue
            t.append(zeit)
            for k, v in werte.items():
                sp[k].append(v)
    if not t:
        # ★ 10.09.2026: 17 Altlaeufe scheiterten hier mit "keine verwertbaren Zeilen". Grund war
        # NICHT ein Lesefehler, sondern ein anderes Dateiformat: vor der Bandaufspaltung trug
        # cd_facetten.csv die Spalten t_si,cd_druck_x,cd_druck_z -- cd_rest/cz_rest gibt es dort
        # gar nicht. Die Meldung nennt das jetzt, statt einen Defekt vorzutaeuschen.
        with open(p) as f:
            kopf = next((r for r in f if not r.startswith("#")), "").strip()
        raise SystemExit(f"{lauf}: keine verwertbaren Zeilen -- benoetigt werden die Spalten "
                         f"time_s, {', '.join(k for k, _, _, _ in GROESSEN)}; die Datei hat: {kopf}")
    return np.array(t), {k: np.array(v) for k, v in sp.items()}


def rastern(t, y, raster, roh):
    """Auf das Zeitraster bringen: Fenstermittel (Vorgabe) oder Punktabtastung."""
    if len(t) == 0:
        return np.array([]), np.array([])
    start = np.floor(t.min() / raster) * raster
    ende = np.ceil(t.max() / raster) * raster
    kanten = np.arange(start, ende + raster, raster)
    mitte, wert = [], []
    for i in range(len(kanten) - 1):
        m = (t >= kanten[i]) & (t < kanten[i + 1])
        if not m.any():
            continue
        mitte.append(0.5 * (kanten[i] + kanten[i + 1]))
        # Punktabtastung nimmt den Wert, der der Fenstermitte am naechsten liegt.
        wert.append(y[m][np.argmin(np.abs(t[m] - mitte[-1]))] if roh else y[m].mean())
    return np.array(mitte), np.array(wert)


def rate(t, y, fenster):
    """Aenderung gegenueber dem Wert vor <fenster> ms, in Prozent."""
    out = np.full(len(t), np.nan)
    for i, ti in enumerate(t):
        j = np.argmin(np.abs(t - (ti - fenster)))
        if abs(t[j] - (ti - fenster)) > 0.51 * (t[1] - t[0] if len(t) > 1 else 1.0):
            continue
        if y[j] == 0.0:
            continue
        out[i] = 100.0 * (y[i] - y[j]) / abs(y[j])
    return out


def zeichne(laeufe, a, aus, bis):
    """Ein Bild fuer <laeufe>, Zeitgrenze <bis> in ms (None = bis zum Datenende)."""
    plt.style.use("dark_background")
    fig, (o, u) = plt.subplots(2, 1, figsize=(12.5, 8.5), sharex=True,
                               gridspec_kw={"height_ratios": [3, 2]})
    fig.patch.set_facecolor("#12141a")
    o2 = o.twinx()          # cz auf der rechten Achse (Heiko-Vorgabe)
    achse = {"links": o, "rechts": o2}
    for ax in (o, o2, u):
        ax.set_facecolor("#12141a")
    farben = ["#4db8ff", "#ff9f40", "#7ee787", "#ff7b9c", "#c792ea"]
    spanne, griffe = [], []

    for n, lauf in enumerate(laeufe):
        t, sp = lies(lauf)
        farbe = farben[n % len(farben)]
        for k, name, stil, seite in GROESSEN:
            tr, yr = rastern(t, sp[k], a.raster, a.roh)
            if a.von is not None:
                m = tr >= a.von; tr, yr = tr[m], yr[m]
            if bis is not None:
                m = tr <= bis; tr, yr = tr[m], yr[m]
            if len(tr) == 0:
                continue
            spanne += [tr.min(), tr.max()]
            beschr = f"{lauf} {name}" if len(laeufe) > 1 else name
            li, = achse[seite].plot(tr, yr, stil, color=farbe, lw=1.7, label=beschr)
            griffe.append(li)
            u.plot(tr, rate(tr, yr, a.fenster), stil, color=farbe, lw=1.7, label=beschr)

    o.set_ylabel("cd_rest  (gestrichelt)", color="#d8dee9")
    o2.set_ylabel("cz_rest  (durchgezogen)", color="#d8dee9")
    o.set_ylim(*ACHSE["cd_rest"])
    o2.set_ylim(*ACHSE["cz_rest"])
    # ★ Serienmodus: die ZEITACHSE muss ueber alle Standbilder GLEICH sein, sonst zeigt jedes
    # Bild denselben Kurvenabschnitt breitgezogen und der Zuwachs ist nicht zu sehen.
    if a.achse_bis is not None:
        o.set_xlim(a.von if a.von is not None else 0.0, a.achse_bis)
    o.grid(alpha=0.18, color="#8b98b0")
    if griffe:
        o.legend(handles=griffe, fontsize=8, ncol=2, facecolor="#1b1e26",
                 edgecolor="#3a4152", labelcolor="#d8dee9")
    art = "Punktabtastung" if a.roh else "Fenstermittel"
    titel = f"cd_rest und cz_rest, Raster {a.raster:.0f} ms ({art})"
    if len(laeufe) == 1:
        titel = f"{laeufe[0]} -- " + titel
    if bis is not None:
        titel += f", Stand {bis:.0f} ms"
    o.set_title(titel, fontsize=11, color="#eceff4")

    u.set_ylabel(f"Aenderung gegen t-{a.fenster:.0f} ms  [%]", color="#d8dee9")
    u.set_xlabel("Zeit [ms]", color="#d8dee9")
    u.set_ylim(-RATE_GRENZE, RATE_GRENZE)
    u.axhline(0.0, color="#8b98b0", lw=0.9)
    for sg in (-5, 5):
        u.axhline(sg, color="#4a5266", lw=0.8, ls=":")
    u.grid(alpha=0.18, color="#8b98b0")
    if griffe:
        u.legend(fontsize=8, ncol=2, facecolor="#1b1e26", edgecolor="#3a4152",
                 labelcolor="#d8dee9")

    if spanne:
        # Werte ausserhalb der Skala nicht verschweigen -- sonst sieht ein
        # abgeschnittener Ausschlag aus wie ein ruhiger Verlauf.
        hinweis = (f"Daten ab {min(spanne):.0f} ms. Untere Skala auf +-{RATE_GRENZE:.0f} % "
                   f"begrenzt; groessere Ausschlaege laufen aus dem Bild.")
        fig.text(0.01, 0.005, hinweis, fontsize=8, color="#7d8799")
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    os.makedirs(os.path.dirname(os.path.abspath(aus)), exist_ok=True)
    fig.savefig(aus, dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)   # ★ ohne das haelt matplotlib im Serienmodus jede Figur offen
    return aus


def serie(lauf, a):
    """Alle <a.serie> ms physikalisch ein Standbild nach export/<lauf>/.

    Jedes Bild traegt den Verlauf VON ANFANG AN BIS ZUR MARKE -- so wie er zu
    diesem Zeitpunkt des Laufes ausgesehen hat. Die Zeitachse und beide
    Werteachsen sind ueber die ganze Serie fest, damit die Bilder
    vergleichbar sind (und als Folge durchblaetterbar).
    """
    schritt = int(round(a.serie))
    if schritt <= 0 or abs(schritt - a.serie) > 1e-9:
        raise SystemExit("--serie braucht ein ganzzahliges Raster in ms, z. B. --serie 100")
    t, _ = lies(lauf)
    ziel = os.path.join(WURZEL, "export", lauf)
    a.achse_bis = float(t.max())   # Achsenende = Laufende, nicht die naechste Marke
    marken = [m for m in range(schritt, int(t.max() // schritt) * schritt + 1, schritt)
              if m >= t.min()]
    geschrieben = []
    for m in marken:
        geschrieben.append(zeichne([lauf], a, os.path.join(ziel, f"kraftverlauf_{m:06d}ms.png"), float(m)))
    # Abschlussbild ueber den GANZEN Lauf -- die Marken enden bei 500 ms, die Daten bei 501.
    geschrieben.append(zeichne([lauf], a, os.path.join(ziel, "kraftverlauf.png"), a.bis))
    print(f"{lauf}: {len(t)} Abtastungen, {t.min():.0f} bis {t.max():.0f} ms"
          f" -> {len(geschrieben)} Bilder in export/{lauf}/")
    for g in geschrieben:
        print(f"  geschrieben: {os.path.relpath(g, WURZEL)}")
    if not marken:
        print(f"  HINWEIS: keine Marke im Datenbereich (Daten ab {t.min():.0f} ms,"
              f" Raster {schritt} ms) -- nur das Abschlussbild.")
    return geschrieben


def main(argv):
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("laeufe", nargs="+")
    ap.add_argument("--raster", type=float, default=10.0)
    ap.add_argument("--fenster", type=float, default=100.0)
    ap.add_argument("--roh", action="store_true")
    ap.add_argument("--von", type=float, default=None)
    ap.add_argument("--bis", type=float, default=None)
    ap.add_argument("--serie", type=float, default=None)
    ap.add_argument("--aus", default=None)
    a = ap.parse_args(argv)
    a.achse_bis = None

    if a.serie is not None:
        if a.aus is not None:
            raise SystemExit("--aus und --serie schliessen sich aus: die Serie schreibt nach export/<lauf>/")
        for lauf in a.laeufe:
            serie(lauf, a)
        return

    aus = a.aus or os.path.join(WURZEL, "export", f"kraftverlauf_{'_'.join(a.laeufe)}.png")
    zeichne(a.laeufe, a, aus, a.bis)
    print(f"geschrieben: {aus}")
    for lauf in a.laeufe:
        t, _ = lies(lauf)
        print(f"  {lauf}: {len(t)} Abtastungen, {t.min():.0f} bis {t.max():.0f} ms")


if __name__ == "__main__":
    main(sys.argv[1:])
