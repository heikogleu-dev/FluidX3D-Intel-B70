#!/usr/bin/env python3
"""h_zeitreihe.py -- H je Dachzone fuer ALLE feld_nah_*.vtk eines Laufs (22.09.2026, Protokoll B25/B27): fx_band -> fx_profil2 je Zeitpunkt,
Median je Zone (zonen_h-Definition), dann Mittel +- sd ueber die Zeitpunkte und Differenz zweier Laeufe in sigma (Welch).
Aufruf: h_zeitreihe.py <lauf_A> <lauf_B> [t_min_s=0.22]   -- u_lat aus CFD_U_LAT (8 mm: 0.125), keine Cd/Cz-Aussage."""
import sys, os, glob, subprocess, numpy as np
D=os.path.dirname(os.path.abspath(__file__)); R=os.path.abspath(os.path.join(D,'..','..'))
ZONEN=[(2.00,2.30,'Saugspitze'),(2.30,2.90,'Dachplateau')]
tmin=float(sys.argv[3]) if len(sys.argv)>3 else 0.22
def zeitreihe(lauf):
    E=f"{R}/export/{lauf}"; out={l:[] for _,_,l in ZONEN}; ts=[]
    # Zeitpunkte = Vereinigung aus vorhandenen VTK-Dumps und schon extrahierten Baendern (die VTK werden nach der Extraktion geloescht, B28)
    ts_alle=sorted(set([os.path.basename(f)[9:15] for f in glob.glob(f"{E}/feld_nah_*ms.vtk")]+[os.path.basename(f)[10:16] for f in glob.glob(f"{E}/dach_band_*ms.npz")]))
    for t in ts_alle:
        tsec=int(t)/1000.0
        if tsec<tmin: continue
        npz=f"{E}/dach_band_{t}ms.npz"; vtk=f"{E}/feld_nah_{t}ms.vtk"
        if not os.path.exists(npz):
            if not os.path.exists(vtk): continue
            subprocess.run([sys.executable,f"{D}/fx_band.py",vtk,npz],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=True)
        prof=f"{D}/prof2_{lauf}_t{t}.npz"
        if not os.path.exists(prof): subprocess.run([sys.executable,f"{D}/fx_profil2.py",f"{lauf}_t{t}",npz],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=True,env={**os.environ,"CFD_U_LAT":os.environ.get("CFD_U_LAT","0.125")})
        z=np.load(prof); x=z['x']; H=z['H']; ts.append(tsec)
        for a,b,l in ZONEN:
            m=(x>=a)&(x<b)&np.isfinite(H); out[l].append(float(np.nanmedian(H[m])))
    return ts,{l:np.array(v) for l,v in out.items()}
A,B=sys.argv[1],sys.argv[2]; tA,hA=zeitreihe(A); tB,hB=zeitreihe(B)
print(f"{A}: {len(tA)} Zeitpunkte {tA[0]:.3f}..{tA[-1]:.3f} s | {B}: {len(tB)} Zeitpunkte")
for _,_,l in ZONEN:
    a,b=hA[l],hB[l]; ma,mb=a.mean(),b.mean(); sa,sb=a.std(ddof=1),b.std(ddof=1); se=np.sqrt(sa**2/len(a)+sb**2/len(b)); d=mb-ma
    print(f"{l:12s} {A}: {ma:.3f} +- {sa:.3f} (n={len(a)}) | {B}: {mb:.3f} +- {sb:.3f} (n={len(b)}) | Differenz {d:+.3f} +- {se:.3f} = {d/se:+.1f} sigma")
    print("   "+A+": "+" ".join(f"{v:.2f}" for v in a)); print("   "+B+": "+" ".join(f"{v:.2f}" for v in b))
