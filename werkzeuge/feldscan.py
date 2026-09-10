#!/usr/bin/env python3
"""Ausreisserpruefung auf dem GANZEN Nahfeld, nur auf echten Fluidzellen.

    werkzeuge/feldscan.py LAUF [LAUF ...]           letzter Zeitpunkt je Lauf
    werkzeuge/feldscan.py --zeit LAUF               alle Zeitpunkte eines Laufs
    werkzeuge/feldscan.py --wand LAUF               Abstand der Ausreisser zur Wand
    werkzeuge/feldscan.py --schwelle 80 LAUF ...    andere Schwelle (Vorgabe 60 m/s)

WARUM ES DIESES WERKZEUG GIBT (10.09.2026). Die Feldpruefung lief bis dahin ueber
den Laengsschnitt y=NY//2 und ueber ALLE Zellen. Beides war falsch:

  * DER SCHNITT UNTERTREIBT UM ZWEI GROESSENORDNUNGEN. Am Schnitt hatte
    km_s4_sism 0 Zellen ueber 60 m/s und galt als sauber; im ganzen Feld sind
    es 1720. Auf dieser Grundlage wurde einen Tag lang der falsche Verdaechtige
    verfolgt. Ein voller 3D-Scan eines 8,8-GB-VTK kostet 10 Sekunden.
  * OHNE FLUIDFILTER SIEHT JEDER LAUF GLEICH KAPUTT AUS. Was in einer Solid-
    oder Equilibrium-Zelle steht, ist kein Stroemungswert. ABER: die Marke 0x03
    (TYPE_MS) ist Fluid, nicht Solid -- eine Fluidzelle neben einer BEWEGTEN
    Wand. Der erste Anlauf dieses Werkzeugs hat sie als Rand weggefiltert und
    dabei 234 echte Fluidzellen mit 100-330 m/s an den Radaufstandsflaechen
    unterschlagen. Sie sind ein EIGENER, vorbestehender Defekt.

Immer gegen einen gepaarten Bezugslauf lesen, nicht gegen eine absolute
Schwelle: Mittel und 99,99-Perzentil bleiben bei isolierten Ausreissern
unauffaellig. Und ueber mehrere Zeitpunkte (--zeit): eine WACHSENDE Zahl ist
eine Instabilitaet, eine stehende ein Artefakt.
"""
import sys, os
import numpy as np

TYPE_S, TYPE_E = 1, 2   # src/defines.hpp
# ACHTUNG, am 10.09.2026 falsch gemacht: TYPE_MS = 0x03 ist KEINE Solidmarke.
# src/setup.cpp:915 sagt es woertlich -- "NUR 0x01 allein ist Solid, TYPE_MS (0x03)
# ist Fluid": 0x03 markiert FLUIDzellen neben einer BEWEGTEN Wand (Fahrbahn, Raeder).
# Echtes Solid ist (flags&3)==1, eine Equilibrium-Randzelle ist (flags&3)==2.
# Host-Fassung desselben Idioms: src/setup.cpp:915.
def ist_fluid(F):
    b = F & 3
    return (b == 0) | (b == 3)
SLAB = 16               # z-Scheiben; das ganze u-Feld sind 6,2 GiB bei 4 mm


def kopf(pfad):
    """DIMENSIONS/ORIGIN/SPACING und die Byte-Offsets aller Datenfelder.

    Die Offsets werden GELESEN, nicht geraten: hinter u folgen rho und flags,
    und die Laenge der Zwischenkoepfe haengt am Namen des Skalars.
    """
    with open(pfad, 'rb') as f:
        h = f.read(4096)
    marke = b"VECTORS u float\n"
    txt = h.split(marke)[0].split(b"\n")
    hol = lambda w: [l for l in txt if l.startswith(w)][0].split()
    d = hol(b"DIMENSIONS"); NX, NY, NZ = int(d[1]), int(d[2]), int(d[3])
    o = hol(b"ORIGIN");     ox, oy, oz = float(o[1]), float(o[2]), float(o[3])
    dx = float(hol(b"SPACING")[1])
    off_u = h.index(marke) + len(marke)
    N = NX * NY * NZ
    felder, off = {}, off_u + N * 12
    while True:
        with open(pfad, 'rb') as f:
            f.seek(off); schwanz = f.read(200)
        k = schwanz.find(b"LOOKUP_TABLE default\n")
        if k < 0 or not schwanz.strip():
            break
        zeile = schwanz[:schwanz.index(b"\n", 1)].strip().split()
        name, typ = zeile[1].decode(), zeile[2].decode()
        start = off + k + len(b"LOOKUP_TABLE default\n")
        felder[name] = (start, typ)
        off = start + N * (4 if typ == "float" else 1)
    return dict(NX=NX, NY=NY, NZ=NZ, ox=ox, oy=oy, oz=oz, dx=dx,
                off_u=off_u, felder=felder)


def vtks(lauf):
    d = f"export/{lauf}"
    return [f"{d}/{x}" for x in sorted(os.listdir(d)) if x.startswith("feld_nah_")]


def scan(pfad, schwelle=60.0):
    """|u|max, Ueberschreitungszahlen und Trefferliste -- nur echte Fluidzellen."""
    k = kopf(pfad)
    NX, NY, NZ = k["NX"], k["NY"], k["NZ"]
    u = np.memmap(pfad, dtype='>f4', mode='r', offset=k["off_u"], shape=(NZ, NY, NX, 3))
    fl = np.memmap(pfad, dtype=np.uint8, mode='r', offset=k["felder"]["flags"][0],
                   shape=(NZ, NY, NX))
    # Die Faecher folgen der SCHWELLE, nicht festen 60/80/100 -- sonst passte bei
    # --schwelle 200 die Aufteilung 'davon frei / an bewegter Wand' nicht zur Summe
    # daneben, und die Zeile log frueher irrefuehrend (Pruefbefund 10.09.).
    stufen = (schwelle, 2.0*schwelle, 5.0*schwelle//3.0)
    stufen = (schwelle, round(4.0*schwelle/3.0), round(5.0*schwelle/3.0))
    gmax, zahl, treffer = 0.0, {s: 0 for s in stufen}, []
    # GETRENNT ZAEHLEN. Die beiden Defekte haben nichts miteinander zu tun:
    #   ms  = Fluidzelle neben BEWEGTER Wand (0x03) -- Radaufstand/Fahrbahn,
    #         vorbestehend, in JEDEM Lauf, auf BEIDEN Sprossen.
    #   frei= gewoehnliche Fluidzelle (0x00) -- hier wirkt SISM.
    # Zusammengezaehlt verdeckt der erste den zweiten.
    zahl_ms, zahl_frei = 0, 0
    gmax_frei, gmax_ms = 0.0, 0.0
    for z0 in range(0, NZ, SLAB):
        z1 = min(z0 + SLAB, NZ)
        A = np.array(u[z0:z1], dtype=np.float32)
        F = np.array(fl[z0:z1])
        # Solid und Equilibrium raus, TYPE_MS bleibt DRIN -- das sind Fluidzellen.
        mg = np.where(ist_fluid(F), np.sqrt((A * A).sum(-1)), 0.0)
        gmax = max(gmax, float(mg.max()))
        # GETRENNTE MAXIMA. Ein gemeinsames |u|max verdeckt genau die Groesse, um die es
        # geht: fuer rd8_aus stand hier 138,71 (Radaufstand), waehrend das freie Fluid nur
        # 45,43 erreicht -- die Schlagzeile meldete den falschen Defekt (Pruefbefund 10.09.).
        gmax_frei = max(gmax_frei, float(np.where((F & 3) == 0, mg, 0.0).max()))
        gmax_ms = max(gmax_ms, float(np.where((F & 3) == 3, mg, 0.0).max()))
        for s in zahl:
            zahl[s] += int((mg > s).sum())
        ueber = mg > schwelle
        zahl_ms += int((ueber & ((F & 3) == 3)).sum())
        zahl_frei += int((ueber & ((F & 3) == 0)).sum())
        for (dz, y, x) in np.argwhere(mg > schwelle):
            treffer.append((float(mg[dz, y, x]), int(x), int(y), int(z0 + dz)))
        del A, F, mg
    treffer.sort(reverse=True)
    return k, gmax, zahl, treffer, zahl_frei, zahl_ms, gmax_frei, gmax_ms, stufen


def wandabstand(pfad, treffer, R=4):
    """Chebyshev-Abstand jedes Treffers zur naechsten Solidzelle.

    Abstand 1 = direkter Wandnachbar = Wirkzone der Wandmodelle (SGS-Lage 1).
    Die Grundrate danebengesetzt, sonst sagt der Anteil nichts.
    """
    k = kopf(pfad)
    NX, NY, NZ = k["NX"], k["NY"], k["NZ"]
    fl = np.memmap(pfad, dtype=np.uint8, mode='r', offset=k["felder"]["flags"][0],
                   shape=(NZ, NY, NX))

    def abstand(x, y, z):
        # SOLID IST (flags&3)==1, NICHT (flags&TYPE_S). TYPE_MS = 0x03 traegt das
        # Solid-Bit MIT, ist aber Fluid -- ein "& TYPE_S" zaehlt jede Fluidzelle an
        # einer bewegten Wand als Wand und schiebt echte Lage-2-Zellen nach Lage 1.
        # Etabliertes Idiom: (flags&TYPE_BO)==TYPE_S, src/kernel.cpp:1733.
        for r in range(1, R + 1):
            F = np.array(fl[max(0, z - r):min(NZ, z + r + 1),
                            max(0, y - r):min(NY, y + r + 1),
                            max(0, x - r):min(NX, x + r + 1)])
            if ((F & 3) == TYPE_S).any():
                return r
        return R + 1

    from collections import Counter
    c = Counter(abstand(x, y, z) for (m, x, y, z) in treffer)
    rng = np.random.default_rng(7)
    n = d1 = 0
    while n < 4000:
        z, y, x = (int(rng.integers(1, NZ - 1)), int(rng.integers(1, NY - 1)),
                   int(rng.integers(1, NX - 1)))
        if not ist_fluid(fl[z, y, x]):
            continue
        n += 1
        d1 += abstand(x, y, z) == 1
    return c, 100.0 * d1 / n


def main(argv):
    schwelle, modus, laeufe = 60.0, "letzt", []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--schwelle":
            i += 1; schwelle = float(argv[i])
        elif a in ("--zeit", "--wand"):
            modus = a[2:]
        else:
            laeufe.append(a)
        i += 1
    if not laeufe:
        print(__doc__); return 1

    for lauf in laeufe:
        pfade = vtks(lauf)
        if not pfade:
            print(f"{lauf}: kein feld_nah_*.vtk"); continue
        if modus != "zeit":
            pfade = pfade[-1:]
        print(f"=== {lauf} ===")
        for p in pfade:
            k, gmax, zahl, treffer, n_frei, n_ms, gm_f, gm_ms, st = scan(p, schwelle)
            print(f"  {os.path.basename(p):26s} |u|max FREI {gm_f:7.2f} | an bewegter Wand {gm_ms:7.2f} m/s")
            print(f"  {'':26s} ueber {st[0]:.0f}: {zahl[st[0]]:7d} (frei {n_frei:7d}, bewegte Wand {n_ms:5d})"
                  f"   ueber {st[1]:.0f}: {zahl[st[1]]:6d}   ueber {st[2]:.0f}: {zahl[st[2]]:5d}")
            if modus == "letzt":
                for (m, x, y, z) in treffer[:8]:
                    print(f"      {m:7.2f}  ijk=({x},{y},{z})  xyz=("
                          f"{k['ox']+x*k['dx']:6.3f},{k['oy']+y*k['dx']:6.3f},"
                          f"{k['oz']+z*k['dx']:6.3f})")
            if modus == "wand":
                c, grund = wandabstand(p, treffer)
                ges = max(1, len(treffer))
                print(f"      Chebyshev-Abstand zur Wand (1 = SGS-Lage 1):")
                for r in sorted(c):
                    print(f"        {r if r <= 4 else '>4'}: {c[r]:6d}"
                          f"  ({100.0*c[r]/ges:5.1f} %)")
                print(f"      Grundrate beliebiger Fluidzellen bei Abstand 1: {grund:.2f} %")
            sys.stdout.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
