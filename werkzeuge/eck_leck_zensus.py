#!/usr/bin/env python3
"""eck_leck_zensus.py -- wo koppelt D3Q27 Fluid ueber Eckverbindungen, die fuer D3Q19 lokal dicht sind? (17.09.2026)

ANLASS: Diff-Pruefagent zum D3Q27-A/B (MITTEL-2): D3Q27 streamt entlang der 8 Raumdiagonalen. Sind bei einer Eckverbindung
Fluid A -> Fluid B = A + c (c = (+-1,+-1,+-1)) alle 6 Zwischenzellen des Wuerfels (3 Flaechen-, 3 Kantennachbarn von A in
Richtung c) Fahrzeug-Solid, gibt es fuer D3Q19 keinen Weg aus hoechstens zwei Schritten -- D3Q27 tauscht dort Masse und Impuls
durch eine Stelle, die fuer D3Q19 lokal dicht ist (Duennteile, Lamellen, Treppenecken). Der A/B trennt dann nicht nur
"Isotropie", sondern auch "Durchlaessigkeit". Dieser Zensus beziffert das VOR der Deutung, rein aus den Flags.
Zaehlung je Paar einmal (nur die 4 Richtungen mit cz = +1; die Gegenrichtung ist dieselbe Verbindung), je x-Bereich.
Aufruf: eck_leck_zensus.py <feld_nah_*.vtk>
"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from zonen_kraft import kopf

vtk = sys.argv[1]
d = kopf(vtk); Nx, Ny, Nz = d["dims"]; ox, oy, oz = d["orig"]; dx = d["spac"][0]
FL = np.fromfile(vtk, dtype=np.uint8, count=Nx*Ny*Nz, offset=d["off_flags"]).reshape(Nz, Ny, Nx)
fluid = (FL == 0x00) | (FL == 0x03)
fahr = FL == 0x41
BER = [("Nase/Splitter", -9.0, 0.6), ("Vorderrad/Haube", 0.6, 1.2), ("Scheibe", 1.2, 2.0), ("Dach", 2.0, 2.8),
       ("Heckscheibe/Hinterrad", 2.8, 3.7), ("Deck/Diffusor", 3.7, 4.0), ("Fluegel/Heck", 4.0, 9.0)]
xs = ox + np.arange(Nx)*dx
print(f"# eck_leck_zensus.py {vtk}: {Nx}x{Ny}x{Nz}, dx {dx*1e3:.3f} mm, Fluid {int(fluid.sum())}, Fahrzeug {int(fahr.sum())}")
gesamt = 0; je_ber = np.zeros(len(BER), np.int64); eck_fluid = 0; dicht19_zwei = 0
S = lambda a, cx, cy, cz: a[1+cz:Nz-1+cz, 1+cy:Ny-1+cy, 1+cx:Nx-1+cx]
for cx in (-1, 1):
    for cy in (-1, 1):
        cz = 1
        A = fluid[1:-1, 1:-1, 1:-1]; B = S(fluid, cx, cy, cz)
        paar = A & B
        eck_fluid += int(paar.sum())
        zw = S(fahr, cx, 0, 0) & S(fahr, 0, cy, 0) & S(fahr, 0, 0, cz) & S(fahr, cx, cy, 0) & S(fahr, cx, 0, cz) & S(fahr, 0, cy, cz)
        leck = paar & zw
        n = int(leck.sum()); gesamt += n
        if n:
            ii = np.nonzero(leck)[2] + 1
            xm = xs[ii] + 0.5*cx*dx
            for b, (lab, a0, a1) in enumerate(BER):
                je_ber[b] += int(((xm >= a0) & (xm < a1)).sum())
print(f"Eckverbindungen Fluid-Fluid (je Paar einmal): {eck_fluid}")
print(f"davon fuer D3Q19 lokal dicht (alle 6 Zwischenzellen Fahrzeug): {gesamt} = {100.0*gesamt/max(eck_fluid,1):.4f} %")
for (lab, _, _), n in zip(BER, je_ber):
    print(f"   {lab:24s} {int(n):8d}")
