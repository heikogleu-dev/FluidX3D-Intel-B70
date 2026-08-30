#!/usr/bin/env python3
"""of13_profil.py -- OF13: wandnormale Grenzschichtprofile ueber Dach/Heckscheibe,
Mittelebene, aus of13_slab_y003.txt (Zellmitten + p + U, |y| < 30 mm) und der aus dem
Fahrzeugpatch gewonnenen Kontur of13_dachlinie.csv.
Gleiche Kennzahlen wie fx_profil.py: u_e = max(u_t), delta99, delta*, theta, H.
"""
import numpy as np, os
SP=os.path.dirname(os.path.abspath(__file__)); XOFF=2.2063

def glatt(a,w):
    k=np.ones(2*w+1)/(2*w+1); return np.convolve(np.pad(a,w,mode="edge"),k,mode="valid")

D = np.genfromtxt(SP+"/of13_dachlinie.csv", delimiter=",", names=True)
S = np.load(SP+"/of13_slab.npy").astype(np.float64)     # x y z p Ux Uy Uz
sx, sz, sux, suz = S[:,0], S[:,2], S[:,4], S[:,6]

m = (D["x_v2_m"]>1.95)&(D["x_v2_m"]<4.10)
xw, zw = D["x_of13_m"][m], D["z_m"][m]
zg = glatt(zw, 2)
dx=np.gradient(xw); dz=np.gradient(zg); L=np.hypot(dx,dz); tx,tz=dx/L,dz/L
nx_,nz_=-tz,tx

BIN=0.002; NB=175
smid=(np.arange(NB)+0.5)*BIN
UT=np.full((len(xw),NB), np.nan)
for i in range(len(xw)):
    dxx = sx-xw[i]; dzz = sz-zw[i]
    tt = dxx*tx[i]+dzz*tz[i]; nn = dxx*nx_[i]+dzz*nz_[i]
    k = (np.abs(tt)<0.015)&(nn>0)&(nn<NB*BIN)
    if k.sum()<10: continue
    b = (nn[k]/BIN).astype(int)
    ut = sux[k]*tx[i]+suz[k]*tz[i]
    su=np.zeros(NB); cn=np.zeros(NB)
    np.add.at(su,b,ut); np.add.at(cn,b,1.0)
    UT[i]=np.where(cn>0, su/np.maximum(cn,1), np.nan)

d99=[];ds=[];th=[]
for i in range(len(xw)):
    u=UT[i].copy()
    ok=~np.isnan(u)
    if ok.sum()<5: d99.append(np.nan);ds.append(np.nan);th.append(np.nan);continue
    uu=np.interp(smid, smid[ok], u[ok])
    ue=uu.max(); ke=int(np.argmax(uu))
    j=int(np.argmax(uu>=0.99*ue))
    d99.append(smid[j])
    f=uu[:ke+1]/ue; ss=smid[:ke+1]
    ds.append(np.trapezoid(1-f,ss)); th.append(np.trapezoid(f*(1-f),ss))
d99=np.array(d99);ds=np.array(ds);th=np.array(th);H=ds/np.where(th>0,th,np.nan)
np.savez_compressed(SP+"/prof_of13.npz", x=xw+XOFF, s=smid, ut=UT, zw=zw,
                    d99=d99, dstern=ds, theta=th, H=H,
                    cp=D["cp_wand"][m], cf=D["cf"][m], tau=D["tau_t_m2s2"][m])
print("prof_of13.npz:", len(xw), "Stationen")
for i in range(0,len(xw),6):
    print(f"x_v2 {xw[i]+XOFF:6.3f} z {zw[i]:6.3f}  d99 {1000*d99[i]:6.1f} mm  d* {1000*ds[i]:6.2f}  "
          f"th {1000*th[i]:6.2f}  H {H[i]:5.2f}  cf {D['cf'][m][i]:+9.6f} cp {D['cp_wand'][m][i]:+6.3f}")
