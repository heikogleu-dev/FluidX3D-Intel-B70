#!/usr/bin/env python3
# B1: Facettenzellen je SIMD-Block (lineare Indexreihenfolge n, Bloecke zu 16/32/64) -- nur messen.
# Quellen: export/apg8_a1_b70/facetten_histogramme.csv (Spalte n = linearer Zellindex, klasse 0 = aktive Facette)
#          export/apg8_a1_b70/feld_nah_000300ms.vtk  (SCALARS flags unsigned_char, 845x333x233, x schnellste Achse)
import numpy as np, sys, time, os
t0 = time.time()
EXP = "/home/heiko/CFD/FluidX3D-v2/export/apg8_a1_b70/"
OUT = "/tmp/claude-1000/-home-heiko-CFD-FluidX3D/c3832f5f-1a97-451a-83ce-1e6b2a621d99/scratchpad/b1/"
Nx, Ny, Nz = 845, 333, 233
N = Nx*Ny*Nz
FLAGS_OFF = 1049003606  # Byte nach "LOOKUP_TABLE default\n" (grep -b), FLAGS_OFF + N == Dateigroesse
TYPE_S, TYPE_E, TYPE_BO = 0x01, 0x02, 0x03
WAND_FLAG = 0x41
out = []
def P(*a):
    s = " ".join(str(x) for x in a); print(s, flush=True); out.append(s)

# ---- 1. Facettenliste aus der CSV
vtk = EXP+"feld_nah_000300ms.vtk"
assert os.path.getsize(vtk) == FLAGS_OFF+N, (os.path.getsize(vtk), FLAGS_OFF+N)
csv = np.loadtxt(EXP+"facetten_histogramme.csv", delimiter=",", skiprows=1, usecols=(4, 8), dtype=np.int64)
klasse, n_all = csv[:, 0], csv[:, 1]
assert n_all.min() >= 0 and n_all.max() < N
assert len(np.unique(n_all)) == len(n_all), "doppelte n"
fac_n = n_all[klasse == 0]
mark_n = n_all[klasse != 0]
P(f"CSV: {len(n_all)} wandnahe Zellen, davon klasse==0 (aktive Facetten) {len(fac_n)}, markiert {len(mark_n)}  [Log: 766234 / 719574 / 46660]")

# ---- 2. Flags aus der VTK
flags = np.fromfile(vtk, dtype=np.uint8, count=N, offset=FLAGS_OFF)
vals, cnts = np.unique(flags, return_counts=True)
P("Flags-Histogramm (Wert: Anzahl): " + ", ".join(f"0x{v:02x}:{c}" for v, c in zip(vals, cnts)))
bo = flags & TYPE_BO
n_fluid = int(np.count_nonzero(bo == 0)); n_S = int(np.count_nonzero(bo == TYPE_S)); n_E = int(np.count_nonzero(bo == TYPE_E)); n_MS = int(np.count_nonzero(bo == TYPE_BO))
P(f"Zellen N={N}: Fluid(bo=0) {n_fluid} ({100*n_fluid/N:.2f} %), TYPE_S {n_S} ({100*n_S/N:.2f} %), TYPE_E {n_E}, TYPE_MS {n_MS}; wand_flag 0x41: {int(np.count_nonzero(flags==WAND_FLAG))}")
f3 = flags.reshape(Nz, Ny, Nx)
for z in (0, 1, Nz-1):
    v, c = np.unique(f3[z], return_counts=True); P(f"  Ebene z={z}: " + ", ".join(f"0x{a:02x}:{b}" for a, b in zip(v, c)))

# ---- 3. Konsistenz CSV <-> VTK: Facettenzellen sind Fluid und haben einen 0x41-Nachbarn in D3Q19 (belegt Nx/Ny-Layout)
P(f"CSV-Zellen mit bo==0 im VTK: {int(np.count_nonzero(bo[n_all]==0))} von {len(n_all)}; bo-Werte der CSV-Zellen: {dict(zip(*[list(map(int,a)) for a in np.unique(bo[n_all], return_counts=True)]))}")
wand = (f3 == WAND_FLAG)
D3Q19 = [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1),
         (1,1,0),(-1,-1,0),(1,0,1),(-1,0,-1),(0,1,1),(0,-1,-1),(1,-1,0),(-1,1,0),(1,0,-1),(-1,0,1),(0,1,-1),(0,-1,1)]
def nachbar_wand(offsets):
    acc = np.zeros_like(wand)
    for dx, dy, dz in offsets:
        s = np.roll(wand, (dy, dx), axis=(1, 2))  # x/y periodisch wie im Facetten-Scan
        if dz > 0:   acc[dz:] |= s[:-dz]          # Nachbar bei z+dz -> Zelle z sieht wand[z+dz]; hier: Zelle z hat Nachbar (z+dz)
        elif dz < 0: acc[:dz] |= s[-dz:]
        else:        acc |= s
    return acc
# Vorzeichen: Zelle (x,y,z) hat Wandnachbarn bei (x+dx,y+dy,z+dz). np.roll(w, +d) verschiebt w[i] nach i+d, also roll(w,-d)[i] = w[i+d].
def nachbar_wand(offsets):
    acc = np.zeros_like(wand)
    for dx, dy, dz in offsets:
        s = np.roll(wand, (-dy, -dx), axis=(1, 2))
        if dz > 0:   acc[:-dz] |= s[dz:]
        elif dz < 0: acc[-dz:] |= s[:dz]
        else:        acc |= s
    return acc
fluid3 = (f3 & TYPE_BO) == 0
zmask = np.zeros((Nz, 1, 1), dtype=bool); zmask[1:Nz-1] = True
w18 = nachbar_wand(D3Q19) & fluid3 & zmask
w6 = nachbar_wand(D3Q19[:6]) & fluid3 & zmask
n18, n6 = int(w18.sum()), int(w6.sum())
csv_in18 = int(np.count_nonzero(w18.reshape(-1)[n_all]))
P(f"Naeherung aus dem Flags-Feld: Fluid mit 0x41-Nachbar in 18er-Nachbarschaft {n18} (CSV 766234, Abweichung {n18-766234:+d}), in 6er-Nachbarschaft {n6} ({100*n6/766234:.1f} % der 766234); CSV-Zellen mit 18er-Treffer: {csv_in18}/{len(n_all)}")
del w18, w6, wand

# ---- 4. Blockstatistik
def block_stats(name, cell_idx, B):
    nb = (N + B - 1)//B
    k = np.bincount(cell_idx//B, minlength=nb)
    kS = np.bincount(np.flatnonzero(bo == TYPE_S)//B, minlength=nb)
    kF = np.bincount(np.flatnonzero(bo == 0)//B, minlength=nb)
    hit = k >= 1
    nhit = int(hit.sum())
    hist = np.bincount(np.minimum(k, B), minlength=B+1)
    wait_all = int((B - k[hit]).sum())              # alle uebrigen Lanes in Facettenbloecken (inkl. Solid/E/MS)
    wait_fluid = int((kF[hit] - k[hit]).sum())      # nur Fluid-Lanes, die im Facettenblock auf den Pfad warten
    kbar = float(k[hit].mean()) if nhit else 0.0
    P(f"[{name} B={B}] Bloecke {nb}, mit >=1 {name}: {nhit} ({100*nhit/nb:.2f} % aller, {100*nhit/int((kF>0).sum()):.2f} % der Bloecke mit Fluid); k-Mittel in Trefferbloecken {kbar:.2f} = {100*kbar/B:.1f} % Lane-Nutzung")
    P(f"   Histogramm k=0..{B}: " + " ".join(str(int(h)) for h in hist))
    P(f"   Wartende Slots sum(B-k) = {wait_all} = {100*wait_all/n_fluid:.2f} % aller Fluidzellen ({100*wait_all/N:.2f} % aller Zellen); davon Fluid-Lanes {wait_fluid} = {100*wait_fluid/n_fluid:.2f} % der Fluidzellen")
    return k, kS, kF, nb

for B in (16, 32, 64):
    k, kS, kF, nb = block_stats("Facette", fac_n, B)
    # Solid
    allS = kS == B; mixS = (kS >= 1) & (kS < B); noS = kS == 0
    wasteS = int(kS[mixS & (kF > 0)].sum())
    P(f"[TYPE_S B={B}] Bloecke ganz solid {int(allS.sum())} ({100*allS.sum()/nb:.2f} %), gemischt {int(mixS.sum())} ({100*mixS.sum()/nb:.2f} %), ohne Solid {int(noS.sum())} ({100*noS.sum()/nb:.2f} %); Solid-Lanes in gemischten Bloecken mit Fluid {wasteS} = {100*wasteS/N:.2f} % aller Zellen")
    P(f"   Histogramm kS=0..{B}: " + " ".join(str(int(h)) for h in np.bincount(np.minimum(kS, B), minlength=B+1)))
    # Kombination: Block "sauber" = nur Fluid ohne Facette; "divergent" = Facette und/oder gemischt Solid
    div = (k >= 1) | mixS
    P(f"[Kombi B={B}] Bloecke mit Facette ODER gemischtem Solid: {int(div.sum())} ({100*div.sum()/nb:.2f} %); nur Facette (kein Solid im Block) {int(((k>=1)&noS).sum())}; Facette UND Solid {int(((k>=1)&(kS>=1)).sum())}; Solid-gemischt ohne Facette {int((mixS&(k==0)).sum())}")
    if B == 16:
        np.save(OUT+"k_fac16.npy", k.astype(np.uint8)); np.save(OUT+"k_S16.npy", kS.astype(np.uint8))
        # wandnah gesamt (inkl. markierte): dieselbe Statistik
        kw = np.bincount(n_all//B, minlength=nb); hw = kw >= 1
        P(f"[wandnah gesamt B=16] Bloecke mit >=1 der 766234: {int(hw.sum())} ({100*hw.sum()/nb:.2f} %), k-Mittel {float(kw[hw].mean()):.2f}")
        # Verteilung der Facettenbloecke ueber z-Ebenen? (Streuung): Anteil Facettenbloecke innerhalb der F-BBox unbekannt -> Anteil im Fahrzeug-x-Bereich
        zb = (np.flatnonzero(k>=1)*B)//(Nx*Ny)
        P(f"   Facettenbloecke ueber z: min {int(zb.min())} max {int(zb.max())}; Zeilen (x-Reihen) mit >=1 Facette: {len(np.unique(fac_n//Nx))} von {Ny*Nz}")
P(f"Laufzeit {time.time()-t0:.1f} s")
open(OUT+"b1_ergebnis.txt", "w").write("\n".join(out)+"\n")
