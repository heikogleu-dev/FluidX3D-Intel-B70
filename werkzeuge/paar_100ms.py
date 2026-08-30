#!/usr/bin/env python3
# paar_100ms.py -- GEPAARTER Cd/Cz-Vergleich zweier Laeufe an denselben Zeitmarken (Heiko 30.08.2026:
# "reporte bitte regelmaessig die timestamp genauen Werte von satgate 0 zum jetzigen 1 alle 100ms").
# Quelle: cd_facetten.csv, Spalten cd_druck_rest / cz_druck_rest (Karosserie ohne Radkontakt-Band,
# 1-ms-Kadenz ab t_warmup 0,2 s). NICHT forces.csv -- deren Cd ist Phantom-Reibung (B62).
# Je Marke: Mittel ueber +-10 ms und 2*SEM (n~21); Delta = neu - bezug, dazu Delta/sigma.
# Marke 0,20 ist die einzige Ausnahme: Fenster 0,201..0,211 (Warmup-Ende, halbseitig).
# Aufruf: paar_100ms.py <neu> <bezug> [marke]   -- ohne Marke: alle verfuegbaren.
import csv, math, os, sys
def reihe(lauf, sp):
    p=f"export/{lauf}/cd_facetten.csv"
    if not os.path.exists(p): return []
    aus=[]
    for r in csv.DictReader(l for l in open(p) if not l.startswith('#')):
        try: aus.append((float(r['time_s']), float(r[sp])))
        except (ValueError, TypeError, KeyError): pass
    return aus
def bei(rh, t):
    lo,hi = (0.2005,0.2115) if abs(t-0.2)<1e-6 else (t-0.0105, t+0.0105)
    v=[x for tt,x in rh if lo<=tt<=hi]
    if len(v)<5: return None
    m=sum(v)/len(v); s=math.sqrt(sum((a-m)**2 for a in v)/(len(v)-1))
    return m, 2*s/math.sqrt(len(v)), len(v)
neu, bez = sys.argv[1], sys.argv[2]
marken = [float(sys.argv[3])] if len(sys.argv)>3 else [0.2,0.3,0.4,0.5]
R={l:{sp:reihe(l,sp) for sp in ('cd_druck_rest','cz_druck_rest')} for l in (neu,bez)}
tn = R[neu]['cd_druck_rest'][-1][0] if R[neu]['cd_druck_rest'] else 0.0
for t in marken:
    z=f"t={t:.2f}s"
    for sp,lab in (('cd_druck_rest','cd_rest'),('cz_druck_rest','cz_rest')):
        a=bei(R[bez][sp],t); b=bei(R[neu][sp],t)
        if a is None or b is None: z+=f" | {lab}: {'--' if a is None else f'{a[0]:.4f}'} -> {'laeuft noch' if b is None else f'{b[0]:.4f}'}"; continue
        d=b[0]-a[0]; sig=math.sqrt(a[1]**2+b[1]**2)/2
        z+=f" | {lab}: {bez} {a[0]:+.4f}+-{a[1]:.4f}  {neu} {b[0]:+.4f}+-{b[1]:.4f}  Delta {d:+.4f} ({d/sig if sig>0 else 0:+.1f} sigma, n={b[2]})"
    print(z)
print(f"[{neu} steht bei t = {tn:.3f} s]")
