#!/usr/bin/env python3
# Diff-Schnitt |u_nah| - |u_fern| aus den beiden gerenderten Schnitt-PNGs eines dd-Laufs.
#
# NOTBEHELF, ausdruecklich als solcher gekennzeichnet (Iron Rule 5): fuer f8_standard_final
# existiert kein Feld-Dump des letzten Zeitschritts (keine VTK, keine Ebenen-CSV), nur die
# beiden Schnittbilder. Diese Auswertung invertiert deren BEKANNTE Farbskala aus
# render_yslice (setup.cpp): 0,5*u_ref blau -> u_ref weiss -> 1,5*u_ref rot, Solid schwarz.
#
# Drei Grenzen, die im Ergebnis MARKIERT und nicht weggemittelt werden:
#  1. KLEMMUNG. Alles unter 0,5*u_ref ist im PNG derselbe reine Blauwert. Im Nahfeld sind das
#     bei t=492 ms 27,8 % der Ebene = praktisch das ganze Totwasser. Dort ist die Differenz
#     NICHT rekonstruierbar -> eigene Graustufe, kein Zahlenwert, nicht in der Statistik.
#  2. QUERVERSATZ. Der Fern-Schnitt liegt auf cNy/2, die Nah-Mittelebene bildet auf cNy/2+0,5
#     ab -- eine halbe Grobzelle (16 mm bei dx_c=32 mm) daneben. Aus PNGs nicht heilbar.
#  3. QUANTISIERUNG. 256 Farbstufen ueber u_ref -> ~0,059 m/s je Stufe bei u_ref=30 m/s.
import sys, math
from PIL import Image

if len(sys.argv)<11:
    sys.exit("Aufruf: diff_aus_png.py <nah.png> <fern.png> <out.png> <out.csv> <u_ref> <span> "
             "<near_x0> <dx_f> <far_x0> <dx_c>\n"
             "  near_x0/far_x0 = Ursprung in m, dx = Schrittweite in m (aus dem Lauf-Log).\n"
             "  Beispiel 8 mm, NEAR_VOR=96: ... 30 15 -0.326 0.008 -2.662 0.032")
NEAR_X0, DX_F = float(sys.argv[7]), float(sys.argv[8])
FAR_X0,  DX_C = float(sys.argv[9]), float(sys.argv[10])
nah_png, fern_png, out_png, out_csv = sys.argv[1:5]
U_REF   = float(sys.argv[5])   # Anstroemung [m/s], = Skalenmitte
SPAN    = float(sys.argv[6])   # Diff-Skala: -SPAN blau, 0 weiss, +SPAN rot
# ★ 2026-08-21: diese vier Werte waren HARTKODIERT auf den 8-mm-Lauf mit CFD_NEAR_VOR_MM=96.
# Aendert sich die Nahfeld-Box (andere NEAR_VOR/NEAR_LY/NEAR_LZ), rechnet das Werkzeug jede
# Weltkoordinate still falsch -- ohne Fehlermeldung. Jetzt Pflichtangabe aus dem Lauf-Log
# ("Nahfeld x[...]" / "Fernfeld x[...]") bzw. der jeweiligen LAUF.txt.


def entfaerben(p):
    """PNG-Pixel -> (|u| in m/s, status). status: 'ok' | 'solid' | 'klemm'"""
    r,g,b = p
    if r==0 and g==0 and b==0:      return 0.0, 'solid'
    if r==0 and g==0 and b==255:    return 0.5*U_REF, 'klemm'   # untere Klemme (Totwasser)
    if r==255 and g==0 and b==0:    return 1.5*U_REF, 'klemm'   # obere Klemme
    if b==255 and r==g:             return 0.5*U_REF + 0.5*U_REF*(r/255.0), 'ok'   # blau -> weiss
    if r==255 and g==b:             return U_REF     + 0.5*U_REF*(1.0-g/255.0), 'ok' # weiss -> rot
    return 0.0, 'solid'  # kommt in render_yslice-Bildern nicht vor

nah  = Image.open(nah_png ).convert('RGB'); NX, NZ = nah.size
fern = Image.open(fern_png).convert('RGB'); CX, CZ = fern.size
pn, pf = nah.load(), fern.load()

out = Image.new('RGB', (NX, NZ))
po  = out.load()
csv = open(out_csv, 'w')
csv.write("# Notbehelf-Rekonstruktion aus PNG-Farbskala -- siehe Kopf von werkzeuge/diff_aus_png.py\n")
csv.write("# status: ok | nah_solid | nah_klemm (Totwasser, nicht rekonstruierbar) | fern_ungueltig\n")
csv.write("x_idx,z_idx,x_m,z_m,u_nah_ms,u_fern_ms,d_ms,status\n")

n_ok=n_solid=n_klemm=n_fung=0; s2=0.0; dmax=0.0; dmin=0.0
for zi in range(NZ):
    z_w = zi*DX_F
    zc  = z_w/DX_C
    for xi in range(NX):
        x_w = NEAR_X0 + xi*DX_F
        un, st = entfaerben(pn[xi, NZ-1-zi])            # Bildzeile 0 = oben, z waechst nach oben
        if st == 'solid':
            po[xi, NZ-1-zi] = (0,0,0); n_solid+=1
            csv.write(f"{xi},{zi},{x_w:.4f},{z_w:.4f},,,,nah_solid\n"); continue
        if st == 'klemm':
            po[xi, NZ-1-zi] = (85,85,85); n_klemm+=1     # dunkelgrau = Nahfeld geklemmt
            csv.write(f"{xi},{zi},{x_w:.4f},{z_w:.4f},,,,nah_klemm\n"); continue
        # --- Fernfeld bilinear am selben Weltpunkt (y-Versatz siehe Kopf, aus PNG nicht heilbar)
        xc = (x_w - FAR_X0)/DX_C
        i0 = min(max(int(math.floor(xc)), 0), CX-2); k0 = min(max(int(math.floor(zc)), 0), CZ-2)
        tx, tz = xc-i0, zc-k0
        wsum=0.0; acc=0.0; ungueltig=False
        for dk in (0,1):
            for di in (0,1):
                w = (tx if di else 1.0-tx)*(tz if dk else 1.0-tz)
                if w <= 0.0: continue
                uf, sf = entfaerben(pf[i0+di, CZ-1-(k0+dk)])
                if sf == 'klemm': ungueltig = True; break
                if sf == 'solid': continue                # Solid faellt aus der Summe
                wsum += w; acc += w*uf
            if ungueltig: break
        if ungueltig or wsum <= 0.0:
            po[xi, NZ-1-zi] = (170,170,170); n_fung+=1    # hellgrau = kein gueltiger Fernwert
            csv.write(f"{xi},{zi},{x_w:.4f},{z_w:.4f},{un:.4f},,,fern_ungueltig\n"); continue
        uf = acc/wsum
        d  = un - uf
        n_ok+=1; s2 += d*d; dmax=max(dmax,d); dmin=min(dmin,d)
        dc = max(-SPAN, min(SPAN, d))
        if dc < 0.0: t=1.0+dc/SPAN; c=(int(255*t+0.5), int(255*t+0.5), 255)
        else:        t=1.0-dc/SPAN; c=(255, int(255*t+0.5), int(255*t+0.5))
        po[xi, NZ-1-zi] = c
        csv.write(f"{xi},{zi},{x_w:.4f},{z_w:.4f},{un:.4f},{uf:.4f},{d:+.4f},ok\n")
csv.close(); out.save(out_png)
n = NX*NZ
print(f"Bild {NX}x{NZ} = {n} px")
print(f"  auswertbar        {n_ok:7d} ({100*n_ok/n:5.1f} %)")
print(f"  Nah-Solid schwarz {n_solid:7d} ({100*n_solid/n:5.1f} %)")
print(f"  Nah GEKLEMMT dgrau{n_klemm:7d} ({100*n_klemm/n:5.1f} %)  <- Totwasser, Differenz nicht rekonstruierbar")
print(f"  Fern ungueltig hgr{n_fung:7d} ({100*n_fung/n:5.1f} %)")
if n_ok:
    print(f"  d = |u_nah|-|u_fern| ueber die auswertbaren Zellen: RMS {math.sqrt(s2/n_ok):.3f} m/s, min {dmin:+.2f}, max {dmax:+.2f}")
