#!/usr/bin/env python3
"""Cd/Cz-Vergleich im 100-ms-Raster gegen einen Bezugslauf, auf den passenden Zeitschritt.

Heiko-Vorgabe 2026-08-22: nicht Fenstermittel, sondern der Verlauf -- damit sichtbar wird,
AB WANN die Kopplung wirkt und ob die Arme auseinanderlaufen oder nur verschoben sind.
Gemittelt wird je Marke ueber +-10 ms, sonst ist der Momentanwert reines Rauschen
(Cz oszilliert druckdominiert; ein Einzelpunkt taeuscht -- alte Projektlehre).
"""
import csv, math, os, sys

def reihe(pfad, spalte):
    if not os.path.exists(pfad): return []
    aus=[]
    for r in csv.DictReader(l for l in open(pfad) if not l.startswith('#')):
        try: aus.append((float(r['time_s']), float(r[spalte])))
        except (ValueError, TypeError, KeyError): pass
    return aus

def bei(reihe_, t, halb=0.010):
    v=[x for tt,x in reihe_ if abs(tt-t)<=halb]
    if not v: return None, None, 0
    m=sum(v)/len(v)
    s=math.sqrt(sum((a-m)**2 for a in v)/max(1,len(v)-1)) if len(v)>1 else 0.0
    return m, 2*s/math.sqrt(len(v)), len(v)

def block(titel, csvname, spalten, neu, bez, marken):
    print(f"\n{titel}")
    kopf=f"{'t [s]':>6}"
    for sp,lab in spalten: kopf += f" | {lab+' Bezug':>16} {lab+' neu':>16} {'Delta':>9}"
    print(kopf); print("-"*len(kopf))
    for t in marken:
        zeile=f"{t:6.2f}"
        for sp,lab in spalten:
            a,ae,an = bei(reihe(f'export/{bez}/{csvname}', sp), t)
            b,be,bn = bei(reihe(f'export/{neu}/{csvname}', sp), t)
            if a is None: zeile += f" | {'--':>16} {'--':>16} {'--':>9}"
            elif b is None: zeile += f" | {a:9.4f}+-{ae:5.4f} {'laeuft noch':>16} {'--':>9}"
            else:
                d=b-a; sig=math.sqrt(ae*ae+be*be)
                zeile += f" | {a:9.4f}+-{ae:5.4f} {b:9.4f}+-{be:5.4f} {d:+7.4f}"
                if sig>0: zeile += f"" 
        print(zeile)

neu = sys.argv[1] if len(sys.argv)>1 else 'f4_kopplung_prod'
bez = sys.argv[2] if len(sys.argv)>2 else 'f4_std_diff2'
marken=[0.1,0.2,0.3,0.4,0.5]
print(f"NEU: export/{neu}   BEZUG: export/{bez}   (Mittel je Marke ueber +-10 ms)")
block("object_force (ganzes Fahrzeug) -- forces.csv", 'forces.csv', [('Cd','Cd'),('Cz','Cz')], neu, bez, marken)
block("Facetten-Druckanteil -- cd_facetten.csv (erst ab t_warmup = 0,2 s)", 'cd_facetten.csv', [('cd_druck','cd_dr'),('cz_druck','cz_dr')], neu, bez, marken)
block("Z-Band-Zerlegung -- kraft_zband.csv (rest = OBERHALB des Bodenbands)", 'kraft_zband.csv', [('cz_druck_rest','cz_rest'),('cz_druck_band','cz_band')], neu, bez, marken)
n=reihe(f'export/{neu}/forces.csv','Cd')
if n: print(f"\nFortschritt: t = {n[-1][0]:.3f} s von 0,500 s ({100*n[-1][0]/0.5:.0f} %)")
