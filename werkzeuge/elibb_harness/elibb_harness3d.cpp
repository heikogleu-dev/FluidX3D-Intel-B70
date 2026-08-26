// Harness F — 3D-Minifall SCHRAEGE WAND (2026-08-25, nach der Kugel-Falsifikation)
// Zweck: die GPU-Falsifikation (q>0,5-Zweig injiziert Impuls) auf der CPU reproduzieren
// und Kandidaten-Operanden dagegen testen. Der 1D-Saeulen-Harness konnte das NICHT sehen.
//
// Aufbau: gekippter Kanal wie CFD_KANAL_KIPP=45 in Miniatur. x periodisch (Stroemung),
// y-z tragen die 45-Grad-Treppe (Kriterium wie setup.cpp kipp_solid: (z - y) mod Nz < Tv,
// hier vereinfacht auf eine Doppelwand). Antrieb: konstante Volumenkraft fx (Guo-lite:
// u-Shift + Kraftterm 1. Ordnung). BGK, Stoerform f^ = f - w wie FluidX3D.
// Referenzen je Schema: (a) Impulsbilanz-Residuum (Summe u_x muss saettigen, Drag = Kraft),
// (b) effektive Wandlage aus der Nullstellen-Extrapolation des u_x-Profils senkrecht zur
// Wand, (c) q=0,5-Anker (qb=127 -> bitgleich HWBB).
// Schemata: 0=HWBB, 1=Blende AKTUELL (nebb beide Zweige), 2=nur q<0,5,
//           3=KANDIDAT (wird nach dem Planungsagenten eingesetzt; Platzhalter = 1).
#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>
#include <cstdint>
#include <cstdlib>

static constexpr int Q=19;
static const int CX[Q]={0,1,-1,0,0,0,0,1,-1,1,-1,0,0,1,-1,1,-1,0,0};
static const int CY[Q]={0,0,0,1,-1,0,0,1,-1,0,0,1,-1,-1,1,0,0,1,-1};
static const int CZ[Q]={0,0,0,0,0,1,-1,0,0,1,-1,1,-1,0,0,-1,1,-1,1};
static const float W[Q]={1.f/3,1.f/18,1.f/18,1.f/18,1.f/18,1.f/18,1.f/18,
    1.f/36,1.f/36,1.f/36,1.f/36,1.f/36,1.f/36,1.f/36,1.f/36,1.f/36,1.f/36,1.f/36,1.f/36};
static inline int OPP(int i){ return i==0?0:(i%2==1?i+1:i-1); }

static constexpr int NX=24, NY=24, NZ=24;             // x klein (periodisch), Treppe in y-z
static constexpr long NC=(long)NX*NY*NZ;
static inline long IDX(int x,int y,int z){ return (long)x+((long)y+(long)z*NY)*NX; }
static inline long IDXF(long n,int slot){ return (long)slot*NC+n; }
// 45-Grad-Treppe: Solid, wenn ((z - y) mod NZ) < 4  (Doppelwand ueber die Periodik wie im Kanal)
static bool MODUS_KUGEL=false; static bool MODUS_FLACH=false; static float FLACH_Q=0.6f;
static const float KR=6.0f, KCX=NX*0.5f-0.5f, KCY=NY*0.5f-0.5f, KCZ=NZ*0.5f-0.5f;
static inline bool SOLID(int x, int y, int z){
    if(MODUS_FLACH){ return z<2||z>=NZ-2; }
    if(MODUS_KUGEL){ const float dx=x-KCX,dy=y-KCY,dz=z-KCZ; return dx*dx+dy*dy+dz*dz<KR*KR; }
    int m=((z-y)%NZ+NZ)%NZ; return m<4; }
static inline void NB(int x,int y,int z,int i,int&nx,int&ny,int&nz){
    nx=(x+CX[i]+NX)%NX; ny=(y+CY[i]+NY)%NY; nz=(z+CZ[i]+NZ)%NZ; }

static void load_f(int x,int y,int z,float*fhn,const float*fi,long t){
    const long n=IDX(x,y,z);
    fhn[0]=fi[IDXF(n,0)];
    for(int i=1;i<Q;i+=2){
        int ax,ay,az; NB(x,y,z,i,ax,ay,az); const long jn=IDX(ax,ay,az);
        fhn[i]=fi[IDXF(n,(t%2)?i:i+1)]; fhn[i+1]=fi[IDXF(jn,(t%2)?i+1:i)];
    }
}
static void store_f(int x,int y,int z,const float*fhn,float*fi,long t){
    const long n=IDX(x,y,z);
    fi[IDXF(n,0)]=fhn[0];
    for(int i=1;i<Q;i+=2){
        int ax,ay,az; NB(x,y,z,i,ax,ay,az); const long jn=IDX(ax,ay,az);
        fi[IDXF(jn,(t%2)?i+1:i)]=fhn[i]; fi[IDXF(n,(t%2)?i:i+1)]=fhn[i+1];
    }
}
static void macro(const float*f,float&rho,float&ux,float&uy,float&uz){
    rho=ux=uy=uz=0.f; for(int i=0;i<Q;i++){rho+=f[i];ux+=CX[i]*f[i];uy+=CY[i]*f[i];uz+=CZ[i]*f[i];}
    rho+=1.f; ux/=rho; uy/=rho; uz/=rho;
}
// q je Link aus der Facettenebene: wahre Wand = Ebene (z - y) = 3,5 (Mitte zwischen letzter
// Solid- (m=3) und erster Fluidschicht... wir nehmen die exakte Treppenflanke: Ebene durch
// (z-y) = 4 - 0,5 = 3,5, Normal n = (0,-1,1)/sqrt(2)). y_w je Fluidzelle = ((z-y)-3,5)/sqrt(2).
static float q_of_kugel(int x,int y,int z,int d){
    // exakter Ray-Sphere-Schnitt: p(s)=x0+s*c, |p-K|=KR; kleinste s>0
    const float ox=x-KCX, oy=y-KCY, oz=z-KCZ;
    const float cx=CX[d], cy=CY[d], cz=CZ[d];
    const float A=cx*cx+cy*cy+cz*cz, B=2.f*(ox*cx+oy*cy+oz*cz), C=ox*ox+oy*oy+oz*oz-KR*KR;
    const float D=B*B-4.f*A*C; if(D<0.f) return -1.f;
    const float s1=(-B-sqrtf(D))/(2.f*A);
    return s1>0.f?s1:-1.f; // Bruchteil der Linklaenge (A normiert)
}
static float q_of(int y,int z,int d){ // d = wandzeigende Richtung (vom Fluid aus)
    const float m=(float)(((z-y)%NZ+NZ)%NZ);
    const float yw=(m-3.5f)/1.41421356f;                      // Normalabstand in Zellen
    const float ndc=(-(float)CY[d]+(float)CZ[d])/1.41421356f; // n . c_d
    if(ndc>=-1e-6f) return -1.f;                              // nicht wandzeigend
    const float sq=yw/(-ndc);
    return sq;
}
int main(int argc,char**argv){
    if(argc>3&&argv[3][0]=='k') MODUS_KUGEL=true;
    if(argc>3&&argv[3][0]=='f'){ MODUS_FLACH=true; if(argc>4) FLACH_Q=(float)atof(argv[4]); }
    const float FX=(argc>1)?(float)atof(argv[1]):4e-5f, wrx=1.9f; const long TMAX=(argc>2)?atol(argv[2]):12000;
    printf(MODUS_KUGEL?"Harness F: MINIKUGEL r=6 in %dx%dx%d":"Harness F: 45-Grad-Treppe %dx%dx%d, fx=%.1e, omega=%.2f, %ld Schritte\n",NX,NY,NZ,FX,wrx,TMAX);
    printf("%-22s %12s %12s %12s %10s\n","Schema","Sum u_x(T)","Drag/Kraft","max|u|","Urteil");
    auto blende=[](float q,float fs,float nebb)->float{
        return (q<=0.5f)?fmaf(2.f*q,fs,(1.f-2.f*q)*nebb):fmaf(0.5f/q,fs,(1.f-0.5f/q)*nebb); };
    const float chifak=1.f/(1.f/wrx+0.5f); // MLS: 1/(tau0+0,5), tau0=1/omega (hergeleitet, kein Handwert)
    int fails=0;
    for(int schema=0; schema<4; schema++){
        std::vector<float> fi(NC*Q,0.f);
        double su_prev=0.0; double su=0.0; float umax=0.f; bool kaputt=false;
        for(long t=0;t<TMAX&&!kaputt;t++){
            su=0.0; umax=0.f;
            for(int z=0;z<NZ;z++)for(int y=0;y<NY;y++)for(int x=0;x<NX;x++){
                if(SOLID(x,y,z)) continue;
                float h[Q]; load_f(x,y,z,h,fi.data(),t);
                // --- ELIBB-Blende (Schema>0) ---
                if(schema>0){
                    float fpre[Q]; for(int i=0;i<Q;i++) fpre[i]=h[i];
                    float rho,ux,uy,uz; macro(h,rho,ux,uy,uz);
                    const float u2=ux*ux+uy*uy+uz*uz;
                    for(int i=1;i<Q;i++){
                        const int ib=OPP(i);
                        int ax,ay,az; NB(x,y,z,ib,ax,ay,az);
                        if(!SOLID(ax,ay,az)) continue;
                        float sq;
                        if(MODUS_FLACH){ const float ndc_=(z<NZ/2? -(float)CZ[ib] : (float)CZ[ib]); sq = ndc_>0.f ? FLACH_Q/ndc_ : -1.f; }
                        else sq=MODUS_KUGEL?q_of_kugel(x,y,z,ib):q_of(y,z,ib);
                        if(sq<=0.f||sq>1.5f) continue;      // kein Schnitt -> BB
                        if(sq>1.f) sq=1.f;                   // q=1-Klemme wie B1
                        if(schema==2&&sq>0.5f) continue;     // nur q<0,5-Zweig
                        if(fabsf(sq-0.5f)<1e-6f) continue;   // Identitaets-Kurzschluss
                        const float wi=W[i];
                        const float cup=-(CX[i]*ux+CY[i]*uy+CZ[i]*uz);
                        const float feq_ib=wi*(rho*(1.f+3.f*cup+4.5f*cup*cup-1.5f*u2)-1.f);
                        if(schema==3){
                            // MLS PROJIZIERT (Baustein 1, 26.08.): Kernel-Transkription des neuen q>0,5-
                            // Zweigs (kernel.cpp MLS-Block; Zitat JCP 161 (2000) 680 / PRE 65 041203).
                            // chi=(2q-1)/(tau0+0,5), u_bf=(1-3/(2q))u_t; quadratische Terme mit u_t
                            // (Interim I2: tangential projiziert statt volles u_f -- wie im Kernel).
                            if(sq>0.5f){
                                float px=ux,py=uy,pz=uz;
                                if(MODUS_FLACH){ pz=0.f; }
                                else if(MODUS_KUGEL){ const float gx=x-KCX,gy=y-KCY,gz=z-KCZ,gl=sqrtf(gx*gx+gy*gy+gz*gz)+1e-12f; const float un=(px*gx+py*gy+pz*gz)/gl; px-=un*gx/gl; py-=un*gy/gl; pz-=un*gz/gl; }
                                else { const float un=(-py+pz)/1.41421356f; py+=un/1.41421356f; pz-=un/1.41421356f; }
                                const float p2=px*px+py*py+pz*pz;
                                const float chi=(2.f*sq-1.f)*chifak;   // chifak = 1/(tau0+0,5), tau0 = 1/wrx
                                const float cupt=-(CX[i]*px+CY[i]*py+CZ[i]*pz);
                                const float cub=(1.f-1.5f/sq)*cupt;    // c_ib . u_bf
                                const float fst=wi*(rho*(1.f+3.f*cub+4.5f*cupt*cupt-1.5f*p2)-1.f);
                                h[i]=fmaf(1.f-chi,fpre[i],chi*fst);    // q=0,5 => chi=0 => Identitaet
                            }
                            else { const float nebb=wi*(rho-1.f)+(fpre[ib]-feq_ib); h[i]=blende(sq,fpre[i],nebb); }
                        } else {
                            const float nebb=wi*(rho-1.f)+(fpre[ib]-feq_ib);
                            h[i]=blende(sq,fpre[i],nebb);
                        }
                    }
                }
                // --- Guo-lite Kollision mit Kraft fx ---
                float rho,ux,uy,uz; macro(h,rho,ux,uy,uz);
                ux+=0.5f*FX/rho;
                const float u2=ux*ux+uy*uy+uz*uz;
                for(int i=0;i<Q;i++){
                    const float cu=CX[i]*ux+CY[i]*uy+CZ[i]*uz;
                    const float feq=W[i]*(rho*(1.f+3.f*cu+4.5f*cu*cu-1.5f*u2)-1.f);
                    h[i]=h[i]+wrx*(feq-h[i])+3.f*W[i]*CX[i]*FX; // Kraftterm 1. Ordnung
                }
                store_f(x,y,z,h,fi.data(),t);
                su+=ux; const float au=fabsf(ux)+fabsf(uy)+fabsf(uz); if(au>umax) umax=au;
                if(!std::isfinite(ux)) kaputt=true;
            }
            if(t==TMAX-2) su_prev=su;
        }
        // Drag/Kraft: im Gleichgewicht ist d(Sum u)/dt = 0 -> Wand nimmt genau die Kraft.
        // Naeherungsmass: relative Restaenderung je Schritt gegen den Antrieb.
        long nf=0; for(int z=0;z<NZ;z++)for(int y=0;y<NY;y++)for(int x=0;x<NX;x++) if(!SOLID(x,y,z)) nf++;
        const double dsu=(su-su_prev); const double antrieb=(double)FX*(double)nf; // je Schritt
        const double dragq=1.0-dsu/antrieb; // 1 = Wand schluckt alles (stationaer), >1 unmoeglich... <0 = Injektion
        const char* nm[4]={"HWBB","Blende ALT 0,5/q","nur q<0,5","MLS"};
        bool ok = std::isfinite((float)su) && !kaputt && su>0.0 && dragq>0.5 && dragq<1.5 && umax<0.35f;
        // Injektionskriterium: Sum u_x muss SAETTIGEN; waechst es am Ende noch deutlich schneller
        // als der Antrieb erklaert oder ist es negativ/instabil -> Verletzung.
        printf("%-22s %12.4f %12.4f %12.5f %10s\n", nm[schema], su, dragq, umax, ok?"ok":"VERLETZT");
        if(schema<3 && !ok && schema!=1) fails++; // Schema 1 DARF verletzen (soll die GPU-Falsifikation zeigen)
        if(schema==1 && ok) printf("  ACHTUNG: Schema 1 zeigt die Injektion NICHT -- Harness reproduziert die GPU-Falsifikation nicht!\n");
    }
    printf("Fertig (%d harte Verletzungen ausserhalb Schema 1)\n", fails);
    return fails;
}
