#!/usr/bin/env python3
# Flaechenbilanz der Kugel aus dem Zensus-VTK gegen die EXAKTE Flaeche 4*pi*R^2.
# Aufruf: kugel_flaeche.py <vtk> <R_zellen>
import sys, math
vtk, R = sys.argv[1], float(sys.argv[2]); A_exakt = 4*math.pi*R*R
f=open(vtk); line=f.readline()
while not line.startswith('POINTS'): line=f.readline()
N=int(line.split()[1]); pts=[tuple(map(float,f.readline().split())) for _ in range(N)]
vals={}; line=f.readline()
while line:
    if line.startswith('SCALARS'):
        nm=line.split()[1]; f.readline()
        vals[nm]=[f.readline().strip() for _ in range(N)]
    elif line.startswith('VECTORS') and 'normale' in line:
        nrm=[tuple(map(float,f.readline().split())) for _ in range(N)]
    elif line.startswith('CELL_DATA'): break
    line=f.readline()
rg=[int(v) for v in vals['rang_downdate']]; nl=[int(v) for v in vals['n_wandlinks']]; ax=[float(v) for v in vals['achsnaehe']]
cap=math.sqrt(3.0)
def faca(n):
    m=max(abs(n[0]),abs(n[1]),abs(n[2])); return min(1.0/m,cap) if m>0 else cap
fa=[faca(n) for n in nrm]
S_all=sum(fa); S_cov=sum(v for v,r in zip(fa,rg) if r>=1); S_ge2=sum(v for v,l in zip(fa,nl) if l>=2)
print(f"Facetten {N:,}   R = {R:.3f}   4*pi*R^2 = {A_exakt:.1f} Zellen^2\n")
print(f"Sum faca ALLE            = {S_all:9.1f}   / exakt = {S_all/A_exakt:.4f}   (Vorhersage exakte Kugel: 1,2213)")
print(f"Sum faca rang >= 1       = {S_cov:9.1f}   / exakt = {S_cov/A_exakt:.4f}   (These: ~1,00)")
print(f"Sum faca n_wandlinks>=2  = {S_ge2:9.1f}   / exakt = {S_ge2/A_exakt:.4f}")
print(f"Verhaeltnis alle/bedient = {S_all/S_cov:.4f}   (Fahrzeug 4 mm: 1,2537)\n")
# Voxelkoerper-Radius aus den Facettenpositionen (Schwerpunkt + mittlerer Abstand) als Gegenprobe
cx=sum(p[0] for p in pts)/N; cy=sum(p[1] for p in pts)/N; cz=sum(p[2] for p in pts)/N
rr=[math.dist(p,(cx,cy,cz)) for p in pts]; rm=sum(rr)/N
print(f"Facettenschwerpunkt ({cx:.2f},{cy:.2f},{cz:.2f}), mittlerer Facettenabstand {rm:.3f} (nominal R={R:.3f}, Wandzelle ~R+0,5..1)")
# Achsnaehe-Verteilung gegen die analytische Vorhersage
edges=[0.577,0.65,0.75,0.85,0.95,0.99,1.0001]; pred=[3.05,21.96,29.99,29.97,12.04,2.99]
lab=["0,577-0,65","0,65-0,75","0,75-0,85","0,85-0,95","0,95-0,99","0,99-1,00"]
print("\nAchsnaehe   gemessen  analytisch   | Rang0-Anteil im Eimer")
for i in range(6):
    idx=[k for k in range(N) if edges[i]<=ax[k]<edges[i+1]]
    r0=sum(1 for k in idx if rg[k]==0)
    print(f"  {lab[i]:12s} {100*len(idx)/N:6.2f} %  {pred[i]:6.2f} %   | {100*r0/max(1,len(idx)):5.1f} %")
from collections import Counter
c=Counter(nl); print("\nLinkverteilung:", " ".join(f"{k}:{c[k]}" for k in sorted(c)), f"  Mittel {sum(nl)/N:.2f}")
r=Counter(rg); print(f"Rang: 2={100*r[2]/N:.2f}%  1={100*r[1]/N:.2f}%  0={100*r[0]/N:.2f}%   (Fahrzeug 8 mm: 47,20 / 33,63 / 19,17)")
