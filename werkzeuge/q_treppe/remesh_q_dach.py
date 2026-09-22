import numpy as np, sys, time
lauf=sys.argv[1]; Nx,Ny,Nz=map(int,sys.argv[2:5]); x0,y0,z0,dx=map(float,sys.argv[5:9]); OFF=float(sys.argv[9])
t0=time.time()
vtk='export/%s/remesh_flaeche.vtk'%lauf
with open(vtk) as f:
    for _ in range(4): f.readline()
    npts=int(f.readline().split()[1])
P=np.loadtxt(vtk,skiprows=5,max_rows=npts,dtype=np.float64)
with open(vtk) as f:
    for i,l in enumerate(f):
        if l.startswith('POLYGONS'): ntri=int(l.split()[1]); hdr=i; break
T=np.loadtxt(vtk,skiprows=hdr+1,max_rows=ntri,dtype=np.int64,usecols=(1,2,3))
print('geladen',npts,ntri,'%.0f s'%(time.time()-t0))
# Gitterkoordinaten: Zellmitten ganzzahlig, Vertices halbzahlig
G=(P-np.array([x0,y0,z0]))/dx
D=np.load('werkzeuge/q_treppe/dach_%s.npz'%lauf)
xi,yi,zi,yw,wink=D['xi'],D['yi'],D['zi'],D['yw'],D['wink']
# Dreiecke im Dachkasten vorfiltern
A=G[T[:,0]];B=G[T[:,1]];C=G[T[:,2]]
bx0=np.minimum(np.minimum(A[:,0],B[:,0]),C[:,0]); bx1=np.maximum(np.maximum(A[:,0],B[:,0]),C[:,0])
by0=np.minimum(np.minimum(A[:,1],B[:,1]),C[:,1]); by1=np.maximum(np.maximum(A[:,1],B[:,1]),C[:,1])
bz0=np.minimum(np.minimum(A[:,2],B[:,2]),C[:,2]); bz1=np.maximum(np.maximum(A[:,2],B[:,2]),C[:,2])
keep=(bx1>=xi.min()-2)&(bx0<=xi.max()+2)&(by1>=yi.min()-2)&(by0<=yi.max()+2)&(bz1>=zi.min()-2)&(bz0<=zi.max()+2)
A,B,C=A[keep],B[keep],C[keep]; bx0,bx1,by0,by1=bx0[keep],bx1[keep],by0[keep],by1[keep]
print('Dreiecke im Kasten',keep.sum())
# Bins nach (floor x, floor y)
from collections import defaultdict
bins=defaultdict(list)
fx0=np.floor(bx0).astype(int); fx1=np.floor(bx1).astype(int); fy0=np.floor(by0).astype(int); fy1=np.floor(by1).astype(int)
for t in range(len(A)):
    for xx in range(fx0[t],fx1[t]+1):
        for yy in range(fy0[t],fy1[t]+1): bins[(xx,yy)].append(t)
def raycast(ox,oy,oz,d):
    # alle Kandidaten der Zelle: Bins der Zelle und der Nachbarn in Richtung d
    cand=set()
    for xx in (int(np.floor(ox)),int(np.floor(ox+d[0]))):
        for yy in (int(np.floor(oy)),int(np.floor(oy+d[1]))):
            cand.update(bins.get((xx,yy),()))
    if not cand: return -1.0
    idx=np.fromiter(cand,dtype=int)
    a,b,c=A[idx],B[idx],C[idx]
    e1=b-a; e2=c-a; dv=np.array(d,dtype=float)
    p=np.cross(dv,e2); det=np.einsum('ij,ij->i',e1,p)
    ok=np.abs(det)>1e-12
    inv=np.where(ok,1.0/np.where(ok,det,1.0),0.0)
    tv=np.array([ox,oy,oz])-a
    u=np.einsum('ij,ij->i',tv,p)*inv
    q=np.cross(tv,e1); v=np.einsum('j,ij->i',dv,q)*inv
    t=np.einsum('ij,ij->i',e2,q)*inv
    hit=ok&(u>=-1e-9)&(u<=1+1e-9)&(v>=-1e-9)&(u+v<=1+1e-9)&(t>1e-9)
    if not hit.any(): return -1.0
    return float(t[hit].min())
links={'(0,0,-1)':(0,0,-1),'(1,0,-1)':(1,0,-1),'(-1,0,-1)':(-1,0,-1),'(0,1,-1)':(0,1,-1),'(0,-1,-1)':(0,-1,-1)}
res={k:np.full(len(xi),-1.0) for k in links}
for i in range(len(xi)):
    for k,d in links.items(): res[k][i]=raycast(float(xi[i])+OFF,float(yi[i])+OFF,float(zi[i])+OFF,d)
print('raycast fertig %.0f s'%(time.time()-t0))
for k in links:
    q=res[k]; m=(q>0)&(q<=1.0)
    qb=np.clip(np.rint(q[m]*254),1,254)
    print('Link %s: getroffen(0<t<=1) %.1f %%  kein Schnitt/t>1: %.1f %% | q: median %.4f mittel %.4f std %.4f | qb==127 exakt 0,5: %.1f %%  |q-0,5|<=0,02: %.1f %%  |q-0,5|<=0,1: %.1f %%  q<0,4: %.1f %%  q>0,6: %.1f %%'%(k,100*m.mean(),100*(1-m.mean()),np.median(q[m]),q[m].mean(),q[m].std(),100*np.mean(qb==127),100*np.mean(np.abs(q[m]-0.5)<=0.02),100*np.mean(np.abs(q[m]-0.5)<=0.1),100*np.mean(q[m]<0.4),100*np.mean(q[m]>0.6)))
q=res['(0,0,-1)']
h,e=np.histogram(q[(q>0)&(q<=1)],bins=[0,0.2,0.3,0.4,0.45,0.48,0.499,0.501,0.52,0.55,0.6,0.7,0.8,1.0])
print('q-Histogramm Normallink Dach:',' '.join('%g-%g:%d'%(e[i],e[i+1],h[i]) for i in range(len(h))))
mid=np.abs(yi-np.rint((0-y0)/dx))<=1
o=np.argsort(xi[mid])
print('Laengsprofil y~0 (x: z / y_w Facette / q_remesh Normallink / q_ebene):')
print(' '.join('%d:%d/%.2f/%.3f/%.3f'%(xi[mid][k],zi[mid][k],yw[mid][k],q[mid][k],yw[mid][k]/np.cos(np.deg2rad(wink[mid][k]))) for k in o[:75]))
np.savez('/tmp/claude-1000/-home-heiko-CFD-FluidX3D/82099a59-c17e-437f-9dfe-9247e20f655b/scratchpad/remeshq_%s.npz'%lauf,**res)
