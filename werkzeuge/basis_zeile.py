#!/usr/bin/env python3
# basis_zeile.py <dx_mm> -- gibt die Serienzeile fuer eine Sprosse aus basis/fahrzeug_dd.basis aus,
# mit DENSELBEN Umrechnungsregeln wie pruefe_basis (setup.cpp): zellen_fein, zellen_grob_laenge
# und index_grob skalieren mit dx_ref/dx; phys, modus, zellen_grob bleiben. Nicht-eindeutige
# Rundungen werden auf stderr gemeldet und gehoeren in CFD_BASIS_ABWEICHUNG.
# Anlass 28.08.2026: eine von Hand rekonstruierte 8-mm-Zeile kostete einen Messvormittag.
import sys, os
dx=float(sys.argv[1]); pfad=os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","basis","fahrzeug_dd.basis")
dx_ref=None; teile=[]
for z in open(pfad):
    z=z.strip()
    if z.startswith("# dx_ref:"): dx_ref=float(z.split(":")[1]); continue
    if not z or z.startswith("#"): continue
    f=z.split()
    if len(f)<3: continue
    name,wert,einheit=f[0],f[1],f[2]
    if name=="CFD_DX": teile.append(f"{name}={dx:g}"); continue
    if einheit in ("zellen_fein","zellen_grob_laenge","index_grob"):
        roh=float(wert)*dx_ref/dx
        if abs(roh-round(roh))>1e-9: print(f"NICHT EINDEUTIG: {name} {wert} x {dx_ref/dx:g} = {roh} -> {round(roh)} (deklarieren!)", file=sys.stderr)
        wert=str(int(round(roh)))
    teile.append(f"{name}={wert}")
print(" ".join(teile))
