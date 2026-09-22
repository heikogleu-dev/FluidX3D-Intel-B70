#!/usr/bin/env python3
"""zonen_h.py -- H = delta*/theta je Dachzone aus prof2_<name>.npz (fx_profil2.py), MEDIAN ueber die x-Stationen der Zone.
Aufruf: zonen_h.py name1 [name2 ...]
Definition rekonstruiert 22.09.2026 gegen Tagesprotokoll B16/B17 (k2_basis_8 1,859/2,004, h8_sism0 1,985/2,253 exakt getroffen;
Mittelwert und dstern/theta-Verhaeltnis treffen NICHT). Zonen: Saugspitze x 2,00-2,30, Dachplateau 2,30-2,90 (x_v2, Fahrzeugkoordinate).
d99 = Median der Zone in mm. OF13-Bezug Dachplateau H = 1,174 (Protokoll B16). 8 mm ist KEINE Cd/Cz-Aussage; H ist die Grenzschichtkennzahl."""
import sys, os, numpy as np
D=os.path.dirname(os.path.abspath(__file__))
ZONEN=[(2.00,2.30,'Saugspitze'),(2.30,2.90,'Dachplateau')]
print(f"{'Arm':16s} | " + " | ".join(f"H {l} {a:.2f}-{b:.2f} (n) | d99 mm" for a,b,l in ZONEN))
for n in sys.argv[1:]:
    f=f"{D}/prof2_{n}.npz"
    if not os.path.exists(f): print(f"{n:16s} | FEHLT: {f}"); continue
    z=np.load(f); x=z['x']; H=z['H']; d99=z['d99']
    cells=[]
    for a,b,l in ZONEN:
        m=(x>=a)&(x<b)&np.isfinite(H)
        cells.append(f"{np.nanmedian(H[m]):.3f} ({m.sum()}) | {1e3*np.nanmedian(d99[m]):.0f}")
    print(f"{n:16s} | " + " | ".join(cells))
