#!/usr/bin/env python3
# basis_zeile.py <dx_mm> -- gibt die Serienzeile fuer eine Sprosse aus basis/fahrzeug_dd.basis aus,
# mit DENSELBEN Umrechnungsregeln wie pruefe_basis (setup.cpp): zellen_fein, zellen_grob_laenge
# und index_grob skalieren mit dx_ref/dx; phys, modus, zellen_grob bleiben; schritte_fein bleibt
# (16.09.2026: env_schritte rechnet im Lauf um, die Zeile traegt den 4-mm-Wert). Nicht-eindeutige
# Rundungen werden auf stderr gemeldet und gehoeren in CFD_BASIS_ABWEICHUNG.
# ★ 17.09.2026 (Heiko, SKALIERUNG-BEFUNDE-2026-09-17.md Punkte 1 und 4): zwei neue Einheiten.
#   band_oberkante_mm  WERT = Sollhoehe H [mm] der wirksamen Kontaktband-Oberkante ueber Welt-z = 0; die Zeile traegt
#                      N = kraft_zband_regel(H, dx): (N-1/2)*dx am naechsten an H, Gleichstand -> niedriger, N >= 3
#                      (z = 0 ist Fahrbahn, wirksam z = 1..N-1). WORTGLEICH zu setup.cpp; basis_aus_lauf.py importiert sie.
#                      ★ Minimum 3 = mindestens Keil- (z = 1) UND Deckellage (z = 2, Ueberhang-Unterseite ueber den Keilzellen
#                      vor/hinter dem Latsch) -- Heiko 17.09.2026 Punkt 6 Option 1, BAND-ARTEFAKT-8MM.md. Bei 8 mm N = 3 statt 2.
#   lagen              bewusst gitterfeste Lagenzahl -- Wert bleibt.
# Eine UNBEKANNTE Einheit bricht ab (vorher fiel sie still auf "Wert bleibt" durch -- dieselbe Klasse wie M6 im Waechter).
# Anlass 28.08.2026: eine von Hand rekonstruierte 8-mm-Zeile kostete einen Messvormittag.
import sys, os, math

N_MIN = 3  # = KRAFT_ZBAND_N_MIN (setup.cpp): Keillage z = 1 + Deckellage z = 2
def kraft_zband_regel(soll_mm, dx_mm):
    """N >= N_MIN (3: Keil- und Deckellage) mit (N-1/2)*dx am naechsten an soll_mm; Gleichstand (Toleranz 1e-4 mm) -> niedrigere Kante. = setup.cpp"""
    if not (soll_mm > 0 and dx_mm > 0): sys.exit(f"kraft_zband_regel: Sollhoehe {soll_mm} mm und dx {dx_mm} mm muessen positiv sein.")
    n_best, d_best = N_MIN, abs((N_MIN - 0.5)*dx_mm - soll_mm)
    for n in range(N_MIN + 1, N_MIN + 1 + math.ceil(soll_mm/dx_mm) + 1):
        d = abs((n - 0.5)*dx_mm - soll_mm)
        if d < d_best - 1e-4: n_best, d_best = n, d
    return n_best

EINHEITEN = {"phys","modus","zellen_grob","zellen_fein","zellen_grob_laenge","index_grob","schritte_fein","ausgabe",
             "band_oberkante_mm","lagen"}  # = Liste im Waechter (setup.cpp pruefe_basis) plus 'ausgabe'

if __name__ == "__main__":
    dx=float(sys.argv[1]); pfad=os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","basis","fahrzeug_dd.basis")
    dx_ref=None; teile=[]
    for z in open(pfad):
        z=z.strip()
        if z.startswith("# dx_ref:"): dx_ref=float(z.split(":")[1]); continue
        if not z or z.startswith("#"): continue
        f=z.split()
        if len(f)<3: continue
        name,wert,einheit=f[0],f[1],f[2]
        if einheit not in EINHEITEN: sys.exit(f"UNBEKANNTE EINHEIT '{einheit}' bei {name} -- erst hier und im Waechter (setup.cpp) einordnen.")
        if name=="CFD_DX": teile.append(f"{name}={dx:g}"); continue
        if einheit in ("schritte_fein","lagen"): pass  # ★ 16.09.2026: schritte_fein Wert bleibt, env_schritte rechnet im Lauf um (u_lat x dx); ★ 17.09.: lagen bewusst gitterfest
        elif einheit=="band_oberkante_mm":  # ★ 17.09.2026
            h=float(wert); n=kraft_zband_regel(h, dx); alt=int(math.floor(h/dx+0.5))
            print(f"KANTE: {name} Soll {h:g} mm -> N = {n} (z = 1..{n-1} wirksam, wirksame Oberkante {(n-0.5)*dx:.3f} mm ueber Welt-z = 0, dx {dx:.3f} mm)"
                  + (f"; alte Umrechnung llround({h:g}/dx) = {alt} ({(alt-0.5)*dx:.3f} mm)" if alt!=n else ""), file=sys.stderr)
            wert=str(n)
        elif einheit in ("zellen_fein","zellen_grob_laenge","index_grob"):
            roh=float(wert)*dx_ref/dx
            ll=int(math.floor(roh+0.5))  # ★ 16.09.: llround (half away from zero) wie setup.cpp, NICHT round() (half-to-even: 0,5 -> 0)
            if abs(roh-ll)>1e-9: print(f"NICHT EINDEUTIG: {name} {wert} x {dx_ref/dx:g} = {roh} -> {ll} (deklarieren!)", file=sys.stderr)
            wert=str(ll)
        teile.append(f"{name}={wert}")
    print(" ".join(teile))
