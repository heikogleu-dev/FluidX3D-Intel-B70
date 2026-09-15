#!/usr/bin/env python3
# RHO_RAND C2d (15.09.2026): A/B-Vergleich zweier dd-Laeufe am VOLLEN 3D-Feld (Iron Rule 5: Felddaten, keine Bilder).
#   A = Produktionszeile (rho-Puffer, heutiger Ausgabewert), B = CFD_RHO_RAND=1 (Ausgabe = Nachkollisionssumme, Entscheidung (b)).
# Soll: u in allen Zellen BITGLEICH; rho weicht nur im Rahmen der FP16S-Rundung ab (RHO_RAND-PLAN.md §14a: max cp <= ~0,0034),
# ausgenommen Klemmzellen (Pufferwort auf der Klemmgrenze 0,5/1,5) -- dort ist die Nachkollisionssumme != geklemmtes rho.
# Nur den ENDDUMP vergleichen: RHO_SPARSAM schreibt rho im A-Arm nur an Host-Lese-Schritten voll; der Enddump liegt
# in der letzten Sample-Periode (rho_voll_zwang), Zwischendumps koennen im A-Arm veraltetes rho tragen.
# Aufruf: werkzeuge/rho_rand_ab.py <lauf_A> <lauf_B> [vtk-Name, Vorgabe: juengster feld_nah_*.vtk in A]
import sys, os, glob, mmap
import numpy as np

U_LAT = 0.075
CP = (2.0/3.0)/(U_LAT*U_LAT)

def lade(pfad):
    f = open(pfad, 'rb'); m = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    kopf_ende = m.find(b'POINT_DATA')
    kopf = m[:kopf_ende].decode('ascii', 'replace')
    dims = [int(v) for v in kopf.split('DIMENSIONS')[1].split('\n')[0].split()]
    np_ = dims[0]*dims[1]*dims[2]
    iu = m.find(b'VECTORS u float\n') + len(b'VECTORS u float\n')
    u = np.frombuffer(m, dtype='>f4', count=3*np_, offset=iu)
    ir = m.find(b'SCALARS rho float 1\nLOOKUP_TABLE default\n', iu+12*np_-1) + len(b'SCALARS rho float 1\nLOOKUP_TABLE default\n')
    rho = np.frombuffer(m, dtype='>f4', count=np_, offset=ir)
    iflag = m.find(b'SCALARS flags unsigned_char 1\nLOOKUP_TABLE default\n', ir+4*np_-1) + len(b'SCALARS flags unsigned_char 1\nLOOKUP_TABLE default\n')
    flags = np.frombuffer(m, dtype=np.uint8, count=np_, offset=iflag)
    return dims, u, rho, flags

def main():
    if len(sys.argv) < 3: print(__doc__ if __doc__ else 'Aufruf: rho_rand_ab.py <lauf_A> <lauf_B> [vtk]'); sys.exit(2)
    a, b = sys.argv[1], sys.argv[2]
    basis = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'export')
    if len(sys.argv) > 3: name = sys.argv[3]
    else: name = sorted(os.path.basename(p) for p in glob.glob(os.path.join(basis, a, 'feld_nah_*.vtk')))[-1]
    da, ua, ra, fa = lade(os.path.join(basis, a, name))
    db, ub, rb, fb = lade(os.path.join(basis, b, name))
    print(f'Datei {name}: DIMENSIONS {da} (A) / {db} (B), {len(ra)} Zellen')
    if da != db: print('ABBRUCH: unterschiedliche Gitter'); sys.exit(1)
    u_gleich = np.array_equal(ua.view(np.uint32), ub.view(np.uint32))
    n_u_abw = int(np.count_nonzero(ua.view(np.uint32) != ub.view(np.uint32)))
    print(f'u bitgleich: {u_gleich} ({n_u_abw} von {len(ua)} Komponenten verschieden, Soll 0)')
    f_gleich = np.array_equal(fa, fb)
    print(f'flags gleich: {f_gleich}')
    d = np.abs(rb.astype(np.float64) - ra.astype(np.float64))
    bo = fa & 3
    klemm = (ra <= 0.5) | (ra >= 1.5)
    klassen = [('Fluid', (bo == 0) & ~klemm), ('TYPE_MS', (bo == 3) & ~klemm), ('TYPE_E', bo == 2), ('TYPE_S', bo == 1), ('Klemmzellen (A auf Grenze)', klemm & (bo != 1) & (bo != 2))]
    print(f'rho |B - A| je Klasse (cp = (2/3) drho / u_lat^2, u_lat {U_LAT}):')
    for kn, maske in klassen:
        n = int(np.count_nonzero(maske))
        if n == 0: print(f'  {kn:28s}: 0 Zellen'); continue
        v = d[maske]
        ng = int(np.count_nonzero(v == 0.0))
        q = np.quantile(v, [0.5, 0.999])
        print(f'  {kn:28s}: {n:10d} Zellen, gleich {ng:10d}, max {v.max():.3e} (cp {v.max()*CP:.4f}), Median {q[0]:.3e} (cp {q[0]*CP:.5f}), 99,9 % {q[1]:.3e} (cp {q[1]*CP:.4f})')
    nicht_klemm = ~klemm & (bo != 1) & (bo != 2)
    grenze = 3.4e-3/CP
    n_ueber = int(np.count_nonzero(d[nicht_klemm] > grenze))
    print(f'rho-Abweichung ueber cp 0,0034 (Messwert §14a) ausserhalb Klemmzellen: {n_ueber} Zellen (zur Sichtung, kein hartes Soll)')
    ok = u_gleich and f_gleich and int(np.count_nonzero(d[bo == 2])) == 0 and int(np.count_nonzero(d[bo == 1])) == 0
    print('ERGEBNIS: ' + ('u/flags bitgleich, TYPE_E/TYPE_S rho gleich' if ok else 'ABWEICHUNG -- siehe oben'))
    sys.exit(0 if ok else 1)

main()
