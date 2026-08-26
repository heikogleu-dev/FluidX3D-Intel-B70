// B0 — Esoteric-Pull-Harness fuer ELIBB (V2-Fassung, 2026-08-25)
// Neufassung statt V1-Port: der V1-Harness (archiv/single_cell_test, 2026-05-31) hat sich
// in Test 1 nachweislich in der Wandslot-Logik verheddert (dokumentiert im Quelltext).
// Hier wird die Slot-Logik nicht "plausibel nachgebaut", sondern WOERTLICH aus
// FluidX3D-v2/src/kernel.cpp (load_f ~1415-1420, store_f ~1436-1441, Stand f4c6a04)
// transkribiert und dann VERMESSEN statt vermutet.
//
// Fragen (die zwei offenen P2-Auflagen + das Entscheid-Gate):
//  A  Wandslot-Verzoegerung: nach wie vielen Schritten kommt die eigene Auslauf-
//     population an einer Solid-Wand zurueck? (Fullway-Pruefagent: 2. Hier bestaetigen.)
//  B  q-Indizierung: fhn[i] hat Streaming-Ursprung j[opposite(i)] -- welcher fhn-Index
//     wird bei Solid-Nachbar in Richtung d rekonstruiert, und welches q gehoert dazu?
//  C  q=0,5-Gate der PoU-Blende: bitgleich zu unveraendertem HWBB? (inkl. -0.0-Kante)
//  D  Massendrift der Blende ueber N Schritte bei q != 0,5 (ELIBB erhaelt Masse nicht --
//     die Drift wird GEMESSEN, nicht angenommen).
//
// Bauen+Laufen (CPU, kein Bezug zu bin/FluidX3D):
//   g++ -O2 -o elibb_harness elibb_harness.cpp && ./elibb_harness
#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>
#include <cstdint>

static constexpr int Q = 19;
// D3Q19 WOERTLICH aus src/kernel.cpp:940-942 (D3Q19-Zweig)
static const int CX[Q] = {0, 1,-1, 0, 0, 0, 0, 1,-1, 1,-1, 0, 0, 1,-1, 1,-1, 0, 0};
static const int CY[Q] = {0, 0, 0, 1,-1, 0, 0, 1,-1, 0, 0, 1,-1,-1, 1, 0, 0, 1,-1};
static const int CZ[Q] = {0, 0, 0, 0, 0, 1,-1, 0, 0, 1,-1, 1,-1, 0, 0,-1, 1,-1, 1};
static const float W[Q] = {1.f/3.f,
    1.f/18.f,1.f/18.f,1.f/18.f,1.f/18.f,1.f/18.f,1.f/18.f,
    1.f/36.f,1.f/36.f,1.f/36.f,1.f/36.f,1.f/36.f,1.f/36.f,
    1.f/36.f,1.f/36.f,1.f/36.f,1.f/36.f,1.f/36.f,1.f/36.f};
static inline int OPP(int i){ return i==0 ? 0 : (i%2==1 ? i+1 : i-1); } // Paarung wie ib in kernel.cpp:1846

// Gitter: 1x1xNZ-Saeule, z=0 Solid, z=NZ-1 Solid (Deckel), dazwischen Fluid. Periodisch in x/y
// (Selbstnachbar). Stoerform wie FluidX3D: gespeichert wird f - w_i.
static constexpr int NZ = 8;
static inline int IDXF(int n, int slot){ return slot*NZ + n; } // index_f(n,i) = i*N + n
static inline int NB(int n, int i){ int z=n+CZ[i]; if(CX[i]!=0||CY[i]!=0) {} ; if(z<0) z=0; if(z>=NZ) z=NZ-1; return z; } // z-Klemme; x/y wickeln auf sich selbst
static inline bool SOLID(int n){ return n==0 || n==NZ-1; }

// load_f/store_f WOERTLICH (Nicht-SPARSE-Zweig):
static void load_f(int n, float* fhn, const float* fi, long t){
    fhn[0] = fi[IDXF(n,0)];
    for(int i=1;i<Q;i+=2){
        fhn[i  ] = fi[IDXF(n,        (t%2) ? i   : i+1)];
        fhn[i+1] = fi[IDXF(NB(n,i),  (t%2) ? i+1 : i  )];
    }
}
static void store_f(int n, const float* fhn, float* fi, long t){
    fi[IDXF(n,0)] = fhn[0];
    for(int i=1;i<Q;i+=2){
        fi[IDXF(NB(n,i), (t%2) ? i+1 : i  )] = fhn[i  ];
        fi[IDXF(n,       (t%2) ? i   : i+1)] = fhn[i+1];
    }
}
static void macro(const float* fhn, float& rho, float& ux, float& uy, float& uz){
    rho=ux=uy=uz=0.f;
    for(int i=0;i<Q;i++){ rho+=fhn[i]; ux+=CX[i]*fhn[i]; uy+=CY[i]*fhn[i]; uz+=CZ[i]*fhn[i]; }
    rho += 1.0f; // Stoerform
    ux/=rho; uy/=rho; uz/=rho;
}
static void collide(float* fhn, float w_relax){
    float rho,ux,uy,uz; macro(fhn,rho,ux,uy,uz);
    const float u2=ux*ux+uy*uy+uz*uz;
    for(int i=0;i<Q;i++){
        const float cu=CX[i]*ux+CY[i]*uy+CZ[i]*uz;
        const float feq=W[i]*(rho*(1.f+3.f*cu+4.5f*cu*cu-1.5f*u2)-1.f); // Stoerform-feq
        fhn[i]=fhn[i]+w_relax*(feq-fhn[i]);
    }
}
static double mass(const float* fi){ // Summe ueber Fluidzellen: je Zelle Summe der 19 Slots, die load_f bei t lesen wuerde -- fuer Drift reicht die rohe Slotsumme (konstante Slots der Solids ausgenommen)
    double m=0.0; for(int n=1;n<NZ-1;n++){ float fhn[Q]; load_f(n,fhn,fi,0); for(int i=0;i<Q;i++) m+=fhn[i]; }
    return m;
}

extern "C" int test_E();
extern "C" int test_G();
int main(){
    int fails=0;
    // ---------- A: Wandslot-Verzoegerung ----------
    {
        std::vector<float> fi(NZ*Q, 0.0f);
        // Marker in die Auslaufpopulation Richtung Wand (-z, i=6) der Zelle n=1 setzen,
        // dann ohne Kollision nur load/store laufen lassen und beobachten, wann der
        // Marker als Einlauf (i=5, +z) zurueckkommt.
        const float MARK=0.125f;
        long t=0;
        { float fhn[Q]={0}; load_f(1,fhn,fi.data(),t); fhn[6]=MARK; store_f(1,fhn,fi.data(),t); }
        int back=-1;
        for(t=1;t<=4 && back<0;t++){
            float fhn[Q]; load_f(1,fhn,fi.data(),t);
            if(fhn[5]==MARK) back=(int)t;
            // Solidzellen fuehren KEIN stream_collide aus (kernel.cpp:2108-Aequivalent) -> nichts tun
            store_f(1,fhn,fi.data(),t);
        }
        printf("A  Wandslot-Verzoegerung: Marker (i=6 -> Wand) kommt als i=5 zurueck nach %d Schritt(en) -- %s\n",
               back, back==2 ? "FULLWAY (2), wie der Pruefagent" : "ABWEICHUNG");
        if(back!=2){ fails++; }
    }
    // ---------- B: q-Indizierung ----------
    {
        // Zelle n=1, Wand bei -z. Solid-zeigende Richtungen d: alle mit CZ[d]<0 (d=6,10,12,15,17
        // in dieser Tabelle: CZ = -1 fuer i=6,10,12,15,17). Der Kernel prueft
        // (flags[j[ib]]&TYPE_BO)==TYPE_S mit ib=OPP(i): fhn[i] stammt aus j[OPP(i)].
        // Also wird fhn[i] genau dann rekonstruiert, wenn der Nachbar in Richtung OPP(i) Solid
        // ist -- und das q gehoert zum LINK OPP(i) (vom Fluid zur Wand gemessen).
        printf("B  q-Indizierung (Zelle n=1, Wand -z):\n");
        int cnt=0;
        for(int i=1;i<Q;i++){
            const int ib=OPP(i);
            const int nb=NB(1,ib);
            if(SOLID(nb) && (CX[ib]!=0||CY[ib]!=0||CZ[ib]!=0) && CZ[ib]<0){
                printf("   fhn[%2d] (c=%+d%+d%+d) stammt aus Richtung ib=%2d (c=%+d%+d%+d, zeigt zur Wand) -> q = q[ib=%d]\n",
                       i, CX[i],CY[i],CZ[i], ib, CX[ib],CY[ib],CZ[ib], ib);
                cnt++;
            }
        }
        printf("   %d rekonstruierbare Links (Soll fuer -z-Ebene in D3Q19: 5) -- %s\n", cnt, cnt==5?"OK":"FEHLER");
        if(cnt!=5) fails++;
    }
    // ---------- C: q=0,5-Gate (bitgleich) + -0.0-Kante ----------
    {
        auto blende=[](float q, float fs, float nebb)->float{
            return (q<=0.5f) ? fmaf(2.0f*q, fs, (1.0f-2.0f*q)*nebb)
                             : fmaf(0.5f/q, fs, (1.0f-0.5f/q)*nebb);
        };
        // zwei unabhaengige Laeufe: Referenz (nichts tun = implizites HWBB) vs. Blende q=0,5
        std::vector<float> fa(NZ*Q), fb(NZ*Q);
        for(int n=0;n<NZ;n++) for(int i=0;i<Q;i++){ // leichte Anfangsstoerung im Fluid
            float v = SOLID(n) ? 0.0f : 1e-3f*std::sin(0.7f*n+1.3f*i);
            fa[IDXF(n,i)]=v; fb[IDXF(n,i)]=v;
        }
        const float wrx=1.6f;
        bool bit=true;
        for(long t=0;t<50 && bit;t++){
            for(int n=1;n<NZ-1;n++){
                float ha[Q], hb[Q];
                load_f(n,ha,fa.data(),t); load_f(n,hb,fb.data(),t);
                // Arm B: Blende mit q=0,5 auf jeden wandstaemmigen Link anwenden
                for(int i=1;i<Q;i++){
                    const int ib=OPP(i);
                    if(SOLID(NB(n,ib))){
                        float rho,ux,uy,uz; macro(hb,rho,ux,uy,uz);
                        const float u2=ux*ux+uy*uy+uz*uz;
                        const float cu=-(CX[i]*ux+CY[i]*uy+CZ[i]*uz); // ★ W3-Fix: feq der GEGENrichtung ib (c_ib = -c_i) -- wie der Kernel
                        const float feq_op=W[i]*(rho*(1.f+3.f*cu+4.5f*cu*cu-1.5f*u2)-1.f);
                        const float nebb=W[i]*(rho-1.f)+(hb[ib]-feq_op);
                        hb[i]=blende(0.5f, hb[i], nebb);
                    }
                }
                collide(ha,wrx); collide(hb,wrx);
                store_f(n,ha,fa.data(),t); store_f(n,hb,fb.data(),t);
            }
            for(size_t k=0;k<fa.size() && bit;k++){
                uint32_t ba,bb; std::memcpy(&ba,&fa[k],4); std::memcpy(&bb,&fb[k],4);
                if(ba!=bb){
                    // -0.0 vs +0.0 gesondert ausweisen
                    if(fa[k]==0.0f && fb[k]==0.0f) printf("C  Hinweis: -0.0/+0.0-Divergenz bei t=%ld Slot %zu (erwartete Kante)\n",t,k);
                    else { printf("C  BITDIVERGENZ bei t=%ld Slot %zu: %a vs %a\n",t,k,fa[k],fb[k]); bit=false; }
                }
            }
        }
        printf("C  q=0,5-Gate: %s\n", bit ? "BITGLEICH ueber 50 Schritte (bis auf ggf. -0.0-Kanten)" : "VERLETZT");
        if(!bit) fails++;
    }
    // ---------- D: Massendrift bei q != 0,5 ----------
    {
        auto blende=[](float q, float fs, float nebb)->float{
            return (q<=0.5f) ? fmaf(2.0f*q, fs, (1.0f-2.0f*q)*nebb)
                             : fmaf(0.5f/q, fs, (1.0f-0.5f/q)*nebb);
        };
        for(float q : {0.25f, 0.735f}){
            std::vector<float> fi(NZ*Q);
            for(int n=0;n<NZ;n++) for(int i=0;i<Q;i++)
                fi[IDXF(n,i)] = SOLID(n) ? 0.0f : 1e-3f*std::sin(0.7f*n+1.3f*i);
            const double m0=mass(fi.data());
            const float wrx=1.6f;
            for(long t=0;t<200;t++){
                for(int n=1;n<NZ-1;n++){
                    float h[Q]; load_f(n,h,fi.data(),t);
                    for(int i=1;i<Q;i++){
                        const int ib=OPP(i);
                        if(SOLID(NB(n,ib))){
                            float rho,ux,uy,uz; macro(h,rho,ux,uy,uz);
                            const float u2=ux*ux+uy*uy+uz*uz;
                            const float cu=-(CX[i]*ux+CY[i]*uy+CZ[i]*uz); // ★ W3-Fix wie oben
                            const float feq_op=W[i]*(rho*(1.f+3.f*cu+4.5f*cu*cu-1.5f*u2)-1.f);
                            const float nebb=W[i]*(rho-1.f)+(h[ib]-feq_op);
                            h[i]=blende(q,h[i],nebb);
                        }
                    }
                    collide(h,wrx); store_f(n,h,fi.data(),t);
                }
            }
            const double m1=mass(fi.data());
            printf("D  q=%.3f: Massendrift ueber 200 Schritte = %.3e (absolut, Stoerform-Summe %0.3e -> %0.3e)\n",
                   q, m1-m0, m0, m1);
        }
        printf("D  (Drift wird in B3 je Facette in fac_tau[4] GEBUCHT, nicht wegdefiniert)\n");
    }
    fails += test_E();
    fails += test_G();
    printf("\n%s (%d Fehler)\n", fails==0?"HARNESS BESTANDEN":"HARNESS VERLETZT", fails);
    return fails;
}

// ==================== Test E (2026-08-25, nach Kugel-Falsifikation) ====================
// Der Kugellauf g1_kugel_an lieferte Cd = -4,95 -- die Blende INJIZIERT Impuls. Dieser
// Test misst, was der urspruengliche Harness nie gemessen hat: das VORZEICHEN des
// tangentialen Impulsaustauschs der Blende. Aufbau: 1x1x8-Saeule, Fluid mit
// Anfangsimpuls u_x = 0,05, Waende oben+unten. Eine NO-SLIP-Wand muss u_x abbauen
// (Drag ueber die Diagonallinks), eine Spiegelwand laesst es konstant, eine
// injizierende Wand baut es AUF. Referenz: reines HWBB. Varianten:
//   P0 aktuelle Kernel-Form:  nebb = w(rho-1) + (f_ib - feq_ib(u))
//   P1 Vorzeichen-Flip:       nebb = w(rho-1) - (f_ib - feq_ib(u))
//   P2 feq gleicher Richtung: nebb = w(rho-1) + (f_ib - feq_i(u))
//   P3 reine Gleichgewichtswand: nebb = w(rho-1)
extern "C" int test_E();
int test_E(){
    auto blende=[](float q, float fs, float nebb)->float{
        return (q<=0.5f) ? fmaf(2.0f*q, fs, (1.0f-2.0f*q)*nebb)
                         : fmaf(0.5f/q, fs, (1.0f-0.5f/q)*nebb);
    };
    const float wrx=1.9f, U0=0.05f;
    printf("\n=== TEST E: Vorzeichen des tangentialen Impulsaustauschs (U0=%.3f, 400 Schritte)\n", U0);
    printf("%-8s", "Variante");
    for(const char* qs : {"q=0.25","q=0.40","q=0.60","q=0.735"}) printf("  %10s", qs);
    printf("  (Werte: Sum u_x nach 400 Schritten; HWBB-Referenz zuerst)\n");
    // Referenz HWBB (Blende aus)
    double ref=0.0;
    {
        std::vector<float> fi(NZ*Q);
        for(int n=0;n<NZ;n++) for(int i=0;i<Q;i++){
            float rho=1.f, cu=CX[i]*U0;
            fi[IDXF(n,i)] = SOLID(n) ? 0.0f : W[i]*(rho*(1.f+3.f*cu+4.5f*cu*cu-1.5f*U0*U0)-1.f);
        }
        for(long t=0;t<400;t++) for(int n=1;n<NZ-1;n++){
            float h[Q]; load_f(n,h,fi.data(),t); collide(h,wrx); store_f(n,h,fi.data(),t);
        }
        double su=0.0; for(int n=1;n<NZ-1;n++){ float h[Q]; load_f(n,h,fi.data(),400); float r,ux,uy,uz; macro(h,r,ux,uy,uz); su+=ux; }
        ref=su; printf("%-8s", "HWBB"); printf("  %10.6f (alle q gleich)\n", ref);
    }
    int fails=0;
    for(int var=0; var<4; var++){
        printf("P%-7d", var);
        for(float q : {0.25f,0.40f,0.60f,0.735f}){
            std::vector<float> fi(NZ*Q);
            for(int n=0;n<NZ;n++) for(int i=0;i<Q;i++){
                float rho=1.f, cu=CX[i]*U0;
                fi[IDXF(n,i)] = SOLID(n) ? 0.0f : W[i]*(rho*(1.f+3.f*cu+4.5f*cu*cu-1.5f*U0*U0)-1.f);
            }
            for(long t=0;t<400;t++) for(int n=1;n<NZ-1;n++){
                float h[Q]; load_f(n,h,fi.data(),t);
                float fpre[Q]; for(int i=0;i<Q;i++) fpre[i]=h[i];
                float rho,ux,uy,uz; macro(h,rho,ux,uy,uz);
                const float u2=ux*ux+uy*uy+uz*uz;
                for(int i=1;i<Q;i++){
                    const int ib=OPP(i);
                    if(!SOLID(NB(n,ib))) continue;
                    const float wi=W[i];
                    const float cup=-(CX[i]*ux+CY[i]*uy+CZ[i]*uz);       // c_ib . u
                    const float cui= (CX[i]*ux+CY[i]*uy+CZ[i]*uz);       // c_i . u
                    const float feq_ib=wi*(rho*(1.f+3.f*cup+4.5f*cup*cup-1.5f*u2)-1.f);
                    const float feq_i =wi*(rho*(1.f+3.f*cui+4.5f*cui*cui-1.5f*u2)-1.f);
                    float nebb;
                    switch(var){
                        case 0: nebb=wi*(rho-1.f)+(fpre[ib]-feq_ib); break;
                        case 1: nebb=wi*(rho-1.f)-(fpre[ib]-feq_ib); break;
                        case 2: nebb=wi*(rho-1.f)+(fpre[ib]-feq_i);  break;
                        default: nebb=wi*(rho-1.f); break;
                    }
                    h[i]=blende(q,fpre[i],nebb);
                }
                collide(h,wrx); store_f(n,h,fi.data(),t);
            }
            double su=0.0; bool fin=true;
            for(int n=1;n<NZ-1;n++){ float h[Q]; load_f(n,h,fi.data(),400); float r,ux,uy,uz; macro(h,r,ux,uy,uz); if(!std::isfinite(ux)) fin=false; su+=ux; }
            printf("  %10.6f", fin?su:NAN);
            // Abnahme je Variante/q: 0 < Su < HWBB-Referenz*1.05 und endlich (Drag vorhanden, keine Injektion)
            if(!(fin && su>0.0 && su<ref*1.05)) fails++;
        }
        printf("\n");
    }
    printf("Kriterium je Zelle: endlich, >0, <= 1,05*HWBB (staerkerer Abbau ist ok -- q<0,5 rueckt die Wand naeher).\n");
    printf("TEST E: %d Verletzungen ueber 16 Zellen\n", fails);
    return fails;
}

// ==================== Test G (Umbau 2026-08-26, Physik-Kette Baustein 1) ====================
// KOHAERENTES-q-GATE: ebene Saeule, ALLE Wandlinks mit demselben q>0,5 -- die m2-Panel-Klasse.
// Umbau 26.08.: der projizierte K1'-Zweig ist nach der hergeleiteten Neutralkurve
// lambda_krit=4(2-omega)/(omega-1) selbst instabil (bei tau=0,51: lambda_krit~0,163, d.h.
// q>=0,58 DIV) -- er ist jetzt der DOKUMENTATIONS-Arm (var 0). Abnahme-Arm (var 1) ist die
// MLS-Blende (Kernel-Transkription, Zitat JCP 161 (2000) 680 / PRE 65 041203 (2002)):
// chi=(2q-1)/(tau0+0,5) ist fuer q<=1, tau>=0,5 ein Konvexblend -- MUSS bei allen fuenf q
// ueber 30000 Schritte stabil sein. BEIDE Arme tangential projiziert (Interim I2 wie im
// Kernel) -> die FORMEL ist die einzige Variable. Historie (unprojiziert vs. projiziert,
// Kernel-Audit Befund 1, 25.08.): git-Stand vor diesem Umbau.
extern "C" int test_G();
int test_G(){
    printf("\n=== TEST G: kohaerentes q>0,5 (K1'-alt vs. MLS, 30000 Schritte, tau=0,51)\n");
    printf("%-12s %10s %10s %10s %10s %10s\n","Variante","q=0.504","q=0.52","q=0.60","q=0.75","q=1.00");
    const float tau0=0.51f; const float wrx=1.0f/tau0;
    const float chifak=1.f/(tau0+0.5f); // MLS: 1/(tau0+0,5) -- hergeleitet, kein Handwert
    int fails=0;
    for(int var=0; var<2; var++){ // 0 = K1' projiziert (alt, Doku), 1 = MLS projiziert (Abnahme)
        printf("%-12s", var? "MLS":"K1-alt");
        for(float q : {0.504f,0.52f,0.60f,0.75f,1.00f}){
            std::vector<float> fi(NZ*Q);
            for(int n=0;n<NZ;n++) for(int i=0;i<Q;i++)
                fi[IDXF(n,i)] = SOLID(n)?0.f:1e-4f*std::sin(0.7f*n+1.3f*i);
            bool kaputt=false; long tk=-1;
            for(long t=0;t<30000&&!kaputt;t++){
                for(int n=1;n<NZ-1;n++){
                    float h[Q]; load_f(n,h,fi.data(),t);
                    float fpre[Q]; for(int i=0;i<Q;i++) fpre[i]=h[i];
                    float rho,ux,uy,uz; macro(h,rho,ux,uy,uz);
                    // Wand -z/+z: Normale (0,0,+-1). Tangentialprojektion = (ux,uy,0).
                    for(int i=1;i<Q;i++){
                        const int ib=OPP(i);
                        if(!SOLID(NB(n,ib))) continue;
                        const float wi=W[i];
                        const float bb=fpre[i];
                        const float px=ux, py=uy, pz=0.f; // BEIDE Arme tangential projiziert (I2)
                        const float p2=px*px+py*py+pz*pz;
                        const float cupt=-(CX[i]*px+CY[i]*py+CZ[i]*pz);
                        if(var){ // MLS (Kernel-Transkription)
                            const float chi=(2.f*q-1.f)*chifak;
                            const float cub=(1.f-1.5f/q)*cupt; // c_ib . u_bf, u_bf=(1-3/(2q))u_t
                            const float fst=wi*(rho*(1.f+3.f*cub+4.5f*cupt*cupt-1.5f*p2)-1.f);
                            h[i]=fmaf(1.f-chi,bb,chi*fst);
                        } else { // K1' alt: Doku der hergeleiteten Instabilitaet
                            const float feq_ibt=wi*(rho*(1.f+3.f*cupt+4.5f*cupt*cupt-1.5f*p2)-1.f);
                            h[i]=fmaf((2.f*q-1.f)/q, wi*(rho-1.f)-feq_ibt, bb);
                        }
                    }
                    collide(h,wrx); store_f(n,h,fi.data(),t);
                    if(!std::isfinite(h[0])||std::fabs(h[0])>1.f){ kaputt=true; tk=t; }
                }
            }
            if(kaputt) printf("  DIV@%-6ld", tk);
            else       printf("  %8s","stabil");
            if(var==1&&kaputt) fails++;              // MLS MUSS stabil sein
        }
        printf("\n");
    }
    printf("TEST G: %d Verletzungen (nur der MLS-Arm zaehlt; K1-alt DARF divergieren)\n", fails);
    return fails;
}
