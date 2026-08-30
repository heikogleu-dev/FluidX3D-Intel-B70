#!/usr/bin/env python3
"""fx_profil2.py -- FX-Dachauswertung (Fassung 2), mit Zeit- und Spannweitenmittel.

Aufruf: fx_profil2.py <name> <band1.npz> [band2.npz ...]
Ergebnis: prof2_<name>.npz mit
  x, s, ut(x,s)  wandtangentiale Geschwindigkeit auf wandnormalen Strahlen (Mittel ueber
                 alle y-Ebenen des Bandes UND alle uebergebenen Zeitpunkte)
  ut1            u_t in der ERSTEN FLUIDZELLE (kein Interpolat, direkt die Zelle ueber der Wand)
  ux1            u_x   ebendort (das B58-Kriterium)
  negfrac        Anteil der (y,t)-Stichproben mit u_x < 0 in der ersten Fluidzelle
  cp             cp in der ersten Fluidzelle, (rho-1)*2/(3*u_lat^2), u_lat = 0.075
  zw             Wandlage (Oberkante der obersten Solidzelle)
  d99,dstern,theta,H  aus dem geglaetteten Profil (Fenster +-24 mm in x)
Wand ausschliesslich aus flags & TYPE_S (Voxelkoerper).
"""
import sys, os
import numpy as np
TYPE_S=0x01; U_LAT=0.075; CP_FAK=2.0/(3.0*U_LAT*U_LAT); Q_INF=0.5*1.225*30.0**2

def glatt(a,w):
    k=np.ones(2*w+1)/(2*w+1); return np.convolve(np.pad(a,w,mode="edge"),k,mode="valid")

def einer(npz, x0, x1, nmax, glatt_mm):
    d=np.load(npz); U,RHO,FL=d["u"],d["rho"],d["flags"]
    Nx,Ny,Nz=[int(v) for v in d["dims"]]; ox,oy,oz=[float(v) for v in d["orig"]]; dx=float(d["dx"])
    ny=U.shape[1]
    x=ox+np.arange(Nx)*dx; z=oz+np.arange(Nz)*dx
    i0,i1=int(np.searchsorted(x,x0)),int(np.searchsorted(x,x1))
    k0,k1=int(np.searchsorted(z,0.30)),int(np.searchsorted(z,1.62))
    solid=(FL&TYPE_S)!=0
    ns=int(round(nmax/dx)); sd=(np.arange(ns)+0.5)*dx
    w=max(1,int(round(0.001*glatt_mm/dx))); NX=i1-i0; idx=np.arange(NX)
    UT=np.full((ny,NX,ns),np.nan); CPS=np.full((ny,NX,ns),np.nan); UT1=np.full((ny,NX),np.nan)
    UX1=np.full((ny,NX),np.nan); CP=np.full((ny,NX),np.nan); ZW=np.full((ny,NX),np.nan)
    for j in range(ny):
        sc=solid[k0:k1,j,i0:i1]; hat=sc.any(axis=0)
        kt=np.where(hat,k0+(k1-k0-1)-np.argmax(sc[::-1],axis=0),-1)
        zw=np.where(hat,z[np.clip(kt,0,Nz-1)]+0.5*dx,np.nan); ZW[j]=zw
        kf=np.clip(kt+1,0,Nz-1); f1=hat&(kt+1<Nz)&~solid[np.clip(kt+1,0,Nz-1),j,i0+idx]
        zg=glatt(np.nan_to_num(zw,nan=np.nanmean(zw)),w)
        dxx=np.gradient(x[i0:i1]); dzz=np.gradient(zg); L=np.hypot(dxx,dzz); tx,tz=dxx/L,dzz/L
        UT1[j,f1]=(U[kf[f1],j,i0+idx[f1],0]*tx[f1]+U[kf[f1],j,i0+idx[f1],2]*tz[f1])
        UX1[j,f1]=U[kf[f1],j,i0+idx[f1],0]
        CP[j,f1]=(RHO[kf[f1],j,i0+idx[f1]]-1.0)*CP_FAK
        nx_,nz_=-tz,tx
        px=x[i0:i1][:,None]+nx_[:,None]*sd[None,:]; pz=zw[:,None]+nz_[:,None]*sd[None,:]
        fi=(px-ox)/dx; fk=(pz-oz)/dx
        i_=np.clip(np.floor(fi).astype(int),0,Nx-2); a=fi-i_
        k_=np.clip(np.floor(fk).astype(int),0,Nz-2); b=fk-k_
        def bil(F): return ((1-a)*(1-b)*F[k_,j,i_]+a*(1-b)*F[k_,j,i_+1]
                            +(1-a)*b*F[k_+1,j,i_]+a*b*F[k_+1,j,i_+1])
        # F kann (Nz,ny,Nx) (rho) oder (Nz,ny,Nx,3)[...,c] (u) sein -- Indizierung identisch
        ut=bil(U[:,:,:,0])*tx[:,None]+bil(U[:,:,:,2])*tz[:,None]
        # nur verwerfen, wenn die ZELLE, in der der Punkt liegt, Solid ist
        ii=np.clip(np.round(fi).astype(int),0,Nx-1); kk=np.clip(np.round(fk).astype(int),0,Nz-1)
        ut[solid[kk,j,ii]]=np.nan
        # erster Abtastpunkt: echte erste Fluidzelle statt Interpolat
        ut[:,0]=UT1[j]
        UT[j]=ut
        cps=(bil(RHO[:,:,:][:,:,:])-1.0)*CP_FAK if False else (bil(RHO)-1.0)*CP_FAK
        cps[solid[kk,j,ii]]=np.nan; cps[:,0]=CP[j]
        CPS[j]=cps
    return dict(x=x[i0:i1],s=sd,dx=dx,UT=UT,CPS=CPS,UT1=UT1,UX1=UX1,CP=CP,ZW=ZW,glatt_w=w)

def main():
    name=sys.argv[1]; npzs=sys.argv[2:]
    R=[einer(p,2.0,4.05,0.35,24.0) for p in npzs]
    x=R[0]["x"]; s=R[0]["s"]; dx=R[0]["dx"]; w=R[0]["glatt_w"]
    UT=np.concatenate([r["UT"] for r in R],axis=0)
    UT1=np.concatenate([r["UT1"] for r in R],axis=0)
    UX1=np.concatenate([r["UX1"] for r in R],axis=0)
    CP=np.concatenate([r["CP"] for r in R],axis=0)
    CPS=np.concatenate([r["CPS"] for r in R],axis=0)
    ut=np.nanmean(UT,axis=0); ut1=np.nanmean(UT1,axis=0); ux1=np.nanmean(UX1,axis=0)
    cp=np.nanmean(CP,axis=0); cps=np.nanmean(CPS,axis=0); negfrac=np.nanmean(UX1<0.0,axis=0); zw=np.nanmean(R[0]["ZW"],axis=0)
    # x-Glaettung der Profile (gleiche physikalische Fensterbreite in beiden Aufloesungen)
    utg=np.array([glatt(np.nan_to_num(ut[:,k]),w) for k in range(ut.shape[1])]).T
    cpsg=np.array([glatt(np.nan_to_num(cps[:,k]),w) for k in range(cps.shape[1])]).T
    d99=[];ds=[];th=[]
    for i in range(len(x)):
        u=utg[i]
        ue=np.nanmax(u); ke=int(np.nanargmax(u))
        if not np.isfinite(ue) or ue<=1e-6: d99.append(np.nan);ds.append(np.nan);th.append(np.nan);continue
        j=int(np.argmax(u>=0.99*ue)); d99.append(s[j])
        f=u[:ke+1]/ue; ss=s[:ke+1]
        ds.append(np.trapezoid(1-f,ss)); th.append(np.trapezoid(f*(1-f),ss))
    d99=np.array(d99);ds=np.array(ds);th=np.array(th);H=ds/np.where(th>0,th,np.nan)
    s50=[];s90=[]
    for i in range(len(x)):
        u=utg[i]; ue=np.nanmax(u)
        if not np.isfinite(ue) or ue<=1e-6: s50.append(np.nan);s90.append(np.nan);continue
        j5=np.argmax(u>=0.5*ue); j9=np.argmax(u>=0.9*ue)
        s50.append(s[j5]); s90.append(s[j9])
    s50=np.array(s50);s90=np.array(s90)
    out=f"{os.path.dirname(os.path.abspath(__file__))}/prof2_{name}.npz"
    np.savez_compressed(out,x=x,s=s,dx=dx,ut=ut,utg=utg,cps=cps,cpsg=cpsg,ut1=ut1,ux1=ux1,cp=cp,
                        negfrac=negfrac,zw=zw,d99=d99,dstern=ds,theta=th,H=H,s50=s50,s90=s90,n_proben=UT1.shape[0])
    print(f"{name}: {len(npzs)} Zeitpunkte x {R[0]['UT'].shape[0]} y-Ebenen = {UT1.shape[0]} Proben -> {out}")
main()
