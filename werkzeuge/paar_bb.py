#!/usr/bin/env python3
# paar_bb.py -- gepaarter Vergleich REINES BOUNCE-BACK gegen Wandmodell (Heiko 30.08.).
#
# WARUM EIN EIGENES WERKZEUG: ein BB-Lauf (CFD_FACETTEN=0) schreibt KEINE cd_facetten.csv --
# der Facettenpfad existiert dort nicht. Gemeinsame Groesse beider Arme ist allein das
# object_force-Instrument (kraft_zband.csv: Fx_rest_N/Fz_rest_N = Karosserie ohne Radkontaktband).
#
# ★ INSTRUMENTEN-WARNUNG (B62, ehrlich auszuweisen): object_force setzt reine Reflexion voraus.
#   - Im BB-Arm ist diese Annahme EXAKT erfuellt -> die Zahl ist die richtige.
#   - Im Wandmodell-Arm ist sie VERLETZT (Facetten ersetzen Links durch Tausch+tau) -> die Zahl
#     traegt Phantom-Reibung. Beleg: w_ref bei t=0,478 s Cz_rest(object_force) = +0,330 gegen
#     cz_druck_rest(Facettenpfad) = -0,204 -- verschiedene VORZEICHEN.
#   Der Vergleich beantwortet deshalb NICHT "welcher Arm hat den besseren Cz", sondern nur:
#   "wie unterscheiden sich die Arme, GEMESSEN MIT DEMSELBEN (im Wandmodell-Arm verzerrten) Mass".
#   Die belastbare Wandmodell-Zahl steht in cd_facetten.csv und wird zusaetzlich ausgewiesen.
#
# Normierung: q_inf*A_ref = 0,5*1,225*30^2 * 1,85 m2 = 1019,8 N (Projektkonvention, Log "A_ref = 1.8500 m2").
# Aufruf: paar_bb.py <bb_lauf> <wm_lauf> [marke]
import csv, math, os, sys
QA = 0.5*1.225*30.0*30.0*1.85
def reihe(lauf, datei, sp):
    p=f"export/{lauf}/{datei}"
    if not os.path.exists(p): return []
    aus=[]
    for r in csv.DictReader(l for l in open(p) if not l.startswith('#')):
        try:
            v=r.get(sp)
            if v is None or v=='': continue
            aus.append((float(r['time_s']), float(v)))
        except (ValueError, TypeError, KeyError): pass
    return aus
def bei(rh, t):
    lo,hi = (0.2005,0.2115) if abs(t-0.2)<1e-6 else (t-0.0105, t+0.0105)
    v=[x for tt,x in rh if lo<=tt<=hi]
    if len(v)<3: return None
    m=sum(v)/len(v); s=math.sqrt(sum((a-m)**2 for a in v)/(len(v)-1)) if len(v)>1 else 0.0
    return m, 2*s/math.sqrt(len(v)), len(v)
bb, wm = sys.argv[1], sys.argv[2]
marken = [float(sys.argv[3])] if len(sys.argv)>3 else [0.2,0.3,0.4,0.5]
R={}
for l in (bb,wm):
    R[l]={'cd':[(t,f/QA) for t,f in reihe(l,'kraft_zband.csv','Fx_rest_N')],
          'cz':[(t,f/QA) for t,f in reihe(l,'kraft_zband.csv','Fz_rest_N')]}
Rw={'cd':reihe(wm,'cd_facetten.csv','cd_druck_rest'), 'cz':reihe(wm,'cd_facetten.csv','cz_druck_rest')}
print("GEPAART, JE ARM MIT SEINEM GUELTIGEN INSTRUMENT:")
print(f"  {bb:14s} = reines BB   -> object_force (kraft_zband): Reflexionsannahme exakt erfuellt")
print(f"  {wm:14s} = Wandmodell  -> Facettenpfad (cd_facetten): object_force waere dort phantombehaftet")
for t in marken:
    a=bei(Rw['cd'],t); b=bei(Rw['cz'],t); c=bei(R[bb]['cd'],t); d=bei(R[bb]['cz'],t)
    z=f"t={t:.2f}s"
    for wmv,bbv,lab in ((a,c,'cd_rest'),(b,d,'cz_rest')):
        if wmv is None or bbv is None:
            z+=f" | {lab}: {wm} {'--' if wmv is None else f'{wmv[0]:+.4f}'} / {bb} {'laeuft noch' if bbv is None else f'{bbv[0]:+.4f}'}"
            continue
        dd=bbv[0]-wmv[0]; sig=math.sqrt(wmv[1]**2+bbv[1]**2)/2
        z+=f" | {lab}: {wm} {wmv[0]:+.4f}+-{wmv[1]:.4f}  {bb} {bbv[0]:+.4f}+-{bbv[1]:.4f}  Delta {dd:+.4f} ({dd/sig if sig>0 else 0:+.1f} sigma)"
    print(z)
print(f"\nKONTROLLE -- object_force fuer BEIDE (im Arm '{wm}' phantombehaftet, nur zur Einordnung der Verzerrung):")
for t in marken:
    z=f"t={t:.2f}s"
    for sp,lab in (('cd','cd_rest'),('cz','cz_rest')):
        a=bei(R[wm][sp],t); b=bei(R[bb][sp],t)
        if a is None or b is None:
            z+=f" | {lab}: {wm} {'--' if a is None else f'{a[0]:+.4f}'} / {bb} {'laeuft noch' if b is None else f'{b[0]:+.4f}'}"
            continue
        d=b[0]-a[0]; sig=math.sqrt(a[1]**2+b[1]**2)/2
        z+=f" | {lab}: {wm} {a[0]:+.4f}+-{a[1]:.4f}  {bb} {b[0]:+.4f}+-{b[1]:.4f}  Delta {d:+.4f} ({d/sig if sig>0 else 0:+.1f} sigma, n={b[2]})"
    print(z)
for l in (bb,wm):
    r=R[l]['cd']
    print(f"[{l} steht bei t = {r[-1][0]:.3f} s]" if r else f"[{l}: keine Daten]")
