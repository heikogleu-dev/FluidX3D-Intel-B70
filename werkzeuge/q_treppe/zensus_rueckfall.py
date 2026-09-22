import numpy as np, sys
np.set_printoptions(linewidth=200)
def lade_hist(pfad):
    d=np.genfromtxt(pfad,delimiter=',',skip_header=1)
    # yw,winkel,r21,r10,klasse,achse,n_punkte,eigene_links,n,solid_dicke
    return dict(yw=d[:,0],winkel=d[:,1],klasse=d[:,4].astype(int),achse=d[:,5].astype(int),npkt=d[:,6].astype(int),el=d[:,7].astype(int),n=d[:,8].astype(np.int64),dicke=d[:,9].astype(int))
def dekod(n,Nx,Ny):
    x=n%Nx; y=(n//Nx)%Ny; z=n//(Nx*Ny); return x,y,z
def zensus(name,h,Nx,Ny,dx,x0=None,y0=None,zz=None):
    x,y,z=dekod(h['n'],Nx,Ny)
    print('\n########', name, 'Zellen', len(x), ' n_max', h['n'].max(), ' Nx*Ny*Nz-Check n_max <', Nx*Ny*(z.max()+1))
    print('Facetten-Box x %d..%d  y %d..%d  z %d..%d'%(x.min(),x.max(),y.min(),y.max(),z.min(),z.max()))
    aktiv=h['klasse']==0
    print('klasse==0 (aktiv): %d = %.2f %%'%(aktiv.sum(),100*aktiv.mean()))
    if x0 is None: x0=-(x.min()+0.5)*dx-0.0007  # Nase auf x_v2~0 wie k3 (Zelle 58 -> -0,0007)
    if y0 is None: y0=-((y.min()+y.max())/2+0.5)*dx
    xw=x0+dx*(x+0.5); yw_=y0+dx*(y+0.5); zw=dx*(z+0.5)
    print('Welt: x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f'%(xw.min(),xw.max(),yw_.min(),yw_.max(),zw.min(),zw.max()))
    el=h['el']
    # Linkklassen
    def lk(e): return np.where(e<=1,0,np.where(e==2,1,np.where(e<=5,2,3)))  # 0: <=1, 1: 2, 2: 3-5, 3: >=6
    lkn=['L<=1','L=2','L3-5','L>=6']
    xz=np.digitize(xw,[1.7,2.0,3.6]); xzn=['Front<1.7','Scheibe1.7-2.0','Dach2.0-3.6','Heck>3.6']
    zzb=[0.32,0.64,0.96]; zzone=np.digitize(zw,zzb); zzn=['z<0.32','z0.32-0.64','z0.64-0.96','z>=0.96']
    fl=np.abs(yw_); flz=np.digitize(fl,[0.6,0.85]); fln=['|y|<0.6','|y|0.6-0.85','|y|>=0.85']
    for titel,zone,zn in [('x-Zone',xz,xzn),('z-Zone',zzone,zzn),('|y|-Zone',flz,fln)]:
        print('\n-- %s: Zellen je Linkklasse (aktive Facetten, klasse==0) --'%titel)
        print('%-16s %8s | %8s %8s %8s %8s | %7s %7s %7s | %6s %6s'%('Zone','n_aktiv','L<=1','L=2','L3-5','L>=6','L<=1%','L=2%','L>=6%','verw%','dicke1%'))
        for zi_,zname in enumerate(zn):
            m=aktiv&(zone==zi_); ma=(zone==zi_)
            if m.sum()==0: continue
            c=[(m&(lk(el)==k)).sum() for k in range(4)]
            print('%-16s %8d | %8d %8d %8d %8d | %7.2f %7.2f %7.2f | %6.2f %6.2f'%(zname,m.sum(),*c,100*c[0]/m.sum(),100*c[1]/m.sum(),100*c[3]/m.sum(),100*(ma&~aktiv).sum()/ma.sum(),100*(m&(h['dicke']==1)).sum()/m.sum()))
        # Anteil der Ein-Link-Zellen dieser Zone an ALLEN Ein-Link-Zellen
        tot1=(aktiv&(el<=1)).sum(); tot2=(aktiv&(el==2)).sum()
        print('   Verteilung der Ein-Link-Zellen ueber die Zonen (%% aller %d):'%tot1, ' '.join('%s %.1f'%(zn[k],100*(aktiv&(el<=1)&(zone==k)).sum()/tot1) for k in range(len(zn))))
        print('   Verteilung der Zwei-Link-Zellen (%% aller %d):'%tot2, ' '.join('%s %.1f'%(zn[k],100*(aktiv&(el==2)&(zone==k)).sum()/tot2) for k in range(len(zn))))
    # Kreuztabelle x-Zone x z-Zone fuer L<=1
    print('\n-- Kreuztabelle x-Zone x z-Zone: Anteil L<=1 an aktiven Facetten der Zelle (%) / n_aktiv --')
    print('%-16s'%'' + ''.join('%22s'%zn for zn in zzn))
    for xi_,xn in enumerate(xzn):
        row=''
        for zi_ in range(4):
            m=aktiv&(xz==xi_)&(zzone==zi_)
            row+='%14.2f /%6d'%((100*(m&(el<=1)).sum()/max(1,m.sum())),m.sum())
        print('%-16s'%xn+row)
    # Winkel/Dicke der Ein-Link-Zellen
    m1=aktiv&(el<=1)
    print('\nEin-Link-Zellen: winkel Median %.1f, q10 %.1f, q90 %.1f Grad; solid_dicke Verteilung 1/2/3/>=4: %s; achse 0/1/2: %s; yw Median %.3f'%(
        np.median(h['winkel'][m1]),np.percentile(h['winkel'][m1],10),np.percentile(h['winkel'][m1],90),
        [int((m1&(h['dicke']==k)).sum()) for k in (1,2,3)]+[int((m1&(h['dicke']>=4)).sum())],[int((m1&(h['achse']==k)).sum()) for k in (0,1,2)],np.median(h['yw'][m1])))
    m5=aktiv&(el==5)
    print('5-Link-Zellen: winkel Median %.1f, q90 %.1f; dicke1 %%: %.1f'%(np.median(h['winkel'][m5]),np.percentile(h['winkel'][m5],90),100*(m5&(h['dicke']==1)).sum()/m5.sum()))
    # eigene_links Verteilung gesamt
    print('eigene_links Verteilung (aktiv):', ' '.join('%d:%.2f%%'%(k,100*(aktiv&(el==k)).sum()/aktiv.sum()) for k in range(0,12) if (aktiv&(el==k)).sum()>0))
    return dict(x=x,y=y,z=z,xw=xw,yw_=yw_,zw=zw,aktiv=aktiv,el=el,xz=xz,zzone=zzone,flz=flz)
if __name__=='__main__':
    which=sys.argv[1]
    if which=='k3':
        h=lade_hist('export/k3_pf_rang_8/facetten_histogramme.csv'); r=zensus('k3_pf_rang_8 (8 mm, ba8f4bf, Y_VERSATZ=1)',h,961,349,0.008,x0=-0.46875,y0=-1.392)
    elif which=='d8':
        h=lade_hist('export/d8_kdiag_an/facetten_histogramme.csv'); r=zensus('d8_kdiag_an (8 mm, daab954)',h,845,333,0.008)
    elif which=='p4':
        h=lade_hist('export/p4_neu/facetten_histogramme.csv'); r=zensus('p4_neu (4 mm, 8c2a66d)',h,1689,661,0.004,x0=-0.32584,y0=-1.320)
