#!/usr/bin/env python3
"""persistenz_vtk.py <lauf> [ausgabe.vtk] -- macht aus export/<lauf>/facetten_persistenz.csv
ein VTK zum SICHTEN in ParaView: ein Punkt je Facettenzelle in Weltkoordinaten.

Heiko 09.09.2026: "die zurueckgefallenen Zellen mit 0 anstatt 1 markieren, das zusammen mit
einem Facetten-VTK auf die Facetten projizieren, damit ich optisch sehe wo die Probleme sind".

SKALARE:
  wandmodell_aktiv  1 = das Modell lief in JEDEM Besuch, 0 = es fiel IMMER zurueck (= 1 - Rate)
  rueckfallrate     der Rohwert
  eigene_links      Zahl der Solid-Links (Anbindungsgrad)
  kippwinkel_grad   Winkel der Normalen zur dominanten Achse -- der Treiber des Rang-1-Defekts
  zielklasse        1 fuer (5 Links, Kipp >= 1 Grad, y_w != 0,50), sonst 0
  lage              1 = ECHTE Wandzelle (>= 3 Solid-Links, Wandabstand 0,5 Zellen)
                    2 = zweite Reihe (1-2 Links, rein diagonal angebunden, Wandabstand ~1,1 Zellen).
                    Lage 2 faellt zu 100 % zurueck und ist dabei VERMUTLICH KORREKT -- diese Zellen
                    sind rangdefizit (ein Link = eine Gleichung, das System braucht drei), und die
                    Wandarbeit leistet der Nachbar, der direkt an der Flaeche sitzt. Sie sind
                    19,3 % der Punkte und wuerden das Bild sonst zudecken: BEIM SICHTEN AUF
                    lage == 1 THRESHOLDEN.
VEKTOR:
  normale

WARNUNG (Iron Rule 3): das VTK ist zum SICHTEN. Gemessen wird an facetten_persistenz.csv und
den Klassen-CSVs, nie am gerenderten Bild.

Gitterparameter werden aus einem feld_nah_*.vtk desselben Laufs gelesen (ORIGIN/SPACING/DIMENSIONS),
damit die Punkte deckungsgleich zum Feld und zu remesh_flaeche.vtk liegen.
"""
import sys, os, csv, math, glob

def gitter(lauf):
    """ORIGIN, SPACING, DIMENSIONS aus einem beliebigen feld_nah-VTK des Laufs."""
    kand = sorted(glob.glob(f"export/{lauf}/feld_nah_*.vtk"))
    if not kand: raise SystemExit(f"kein feld_nah_*.vtk in export/{lauf} -- Gitterparameter unbekannt")
    org=spc=dim=None
    with open(kand[0], "rb") as fh:
        for _ in range(12):
            z = fh.readline().decode("ascii", "replace").strip()
            if z.startswith("ORIGIN"):     org = [float(v) for v in z.split()[1:4]]
            elif z.startswith("SPACING"):  spc = [float(v) for v in z.split()[1:4]]
            elif z.startswith("DIMENSIONS"): dim = [int(v) for v in z.split()[1:4]]
            if org and spc and dim: break
    if not (org and spc and dim): raise SystemExit(f"ORIGIN/SPACING/DIMENSIONS nicht gefunden in {kand[0]}")
    return org, spc, dim, os.path.basename(kand[0])

def main():
    if len(sys.argv) < 2: raise SystemExit(__doc__)
    lauf = sys.argv[1]
    aus  = sys.argv[2] if len(sys.argv) > 2 else f"export/{lauf}/facetten_persistenz.vtk"
    org, spc, dim, quelle = gitter(lauf)
    NX, NY = dim[0], dim[1]
    print(f"Gitter aus {quelle}: {dim[0]}x{dim[1]}x{dim[2]}, ORIGIN {org}, SPACING {spc[0]}")

    pfad = f"export/{lauf}/facetten_persistenz.csv"
    zeilen = [z for z in open(pfad) if not z.startswith("#")]
    P = []
    for row in csv.DictReader(zeilen):
        try:
            n=int(row["n"]); L=int(row["eigene_links"]); yw=float(row["yw"])
            nx,ny,nz=float(row["nx"]),float(row["ny"]),float(row["nz"]); rate=float(row["rate"])
        except (TypeError, ValueError, KeyError):
            continue
        x=n%NX; y=(n//NX)%NY; z=n//(NX*NY)
        amax=max(abs(nx),abs(ny),abs(nz))
        kipp=math.degrees(math.acos(min(1.0,amax)))
        ziel=1 if (L==5 and amax<=0.99984770 and round(yw*100)!=50) else 0
        lage=1 if L>=3 else 2
        P.append((org[0]+x*spc[0], org[1]+y*spc[1], org[2]+z*spc[2], 1.0-rate, rate, L, kipp, ziel, nx,ny,nz, lage))
    print(f"{len(P):,} Facetten")

    with open(aus, "w") as f:
        f.write("# vtk DataFile Version 3.0\n")
        f.write(f"Wandmodell-Abdeckung je Facette ({lauf}) -- 1 = immer angewandt, 0 = immer zurueckgefallen\n")
        f.write("ASCII\nDATASET POLYDATA\n")
        f.write(f"POINTS {len(P)} float\n")
        for p in P: f.write(f"{p[0]:.5f} {p[1]:.5f} {p[2]:.5f}\n")
        f.write(f"VERTICES {len(P)} {2*len(P)}\n")
        for i in range(len(P)): f.write(f"1 {i}\n")
        f.write(f"POINT_DATA {len(P)}\n")
        for name, idx, typ in (("wandmodell_aktiv",3,"float"), ("rueckfallrate",4,"float"),
                               ("eigene_links",5,"int"), ("kippwinkel_grad",6,"float"), ("zielklasse",7,"int"),
                               ("lage",11,"int")):
            f.write(f"SCALARS {name} {typ} 1\nLOOKUP_TABLE default\n")
            for p in P: f.write((f"{p[idx]:.5f}\n" if typ=="float" else f"{p[idx]}\n"))
        f.write("VECTORS normale float\n")
        for p in P: f.write(f"{p[8]:.5f} {p[9]:.5f} {p[10]:.5f}\n")
    print(f"geschrieben: {aus}  ({os.path.getsize(aus)/1048576:.1f} MB)")
    n1=sum(1 for p in P if p[11]==1); n2=len(P)-n1
    print(f"  Lage 1 (echte Wandzellen, >=3 Links): {n1:,} = {n1/len(P)*100:.1f} %")
    print(f"  Lage 2 (zweite Reihe, 1-2 Links):     {n2:,} = {n2/len(P)*100:.1f} %  -- faellt zu 100 % zurueck, VERMUTLICH KORREKT")
    print("ParaView: zusammen mit export/<lauf>/remesh_flaeche.vtk laden, nach 'wandmodell_aktiv' einfaerben.")
    print("          ZUERST Threshold lage==1 setzen -- sonst faerben die 19 % Lage-2-Zellen das Bild zu.")
    print("          Danach Threshold zielklasse==1 fuer die eigentliche Problemklasse.")

if __name__ == "__main__":
    main()
