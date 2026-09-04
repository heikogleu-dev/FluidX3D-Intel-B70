#include "setup.hpp"
#include <fstream>
#include <cstring>
#include <filesystem>
#include <cstdio>
#include <set> // ★ P9c N2F-SCHALE: Deduplizierung der Schalen-Zellliste
#include <map> // ★ Gradient-Blend: Zelle -> (Gewicht, Lage), max-Gewicht bei Lagen-Duplikaten

static bool f_nur_solid_an_setup() { const char* e = getenv("CFD_F_NUR_SOLID"); return e==nullptr||e[0]=='\0'||atoi(e)>0; } // Default AN, wortgleich zu lbm.cpp
extern char** environ;

// =============================================================================================
// KUGEL IM KANAL MIT MITBEWEGTEM BODEN
//
// D = 450 mm, U = 30 m/s, ISA-Meereshoehe. Validierungsfall gegen OpenFOAM 13 und gegen die
// Standard-Widerstandskurve (Clift/Grace/Weber), letztere ueber CFD_KUGEL_NU bei Re_D = 100..1000
// erreichbar -- dort ist die Grenzschicht aufgeloest und es braucht kein Turbulenzmodell.
//
// Dies ist der Neuaufbau auf frischem Upstream (2026-08-08). Bewusst OHNE: Wandmodell,
// NUT_PATH_A, Mozaffari-APG, Bodenpraegung, Abloesesonde. Die kommen einzeln zurueck, jede erst
// dann, wenn eine Messung zeigt, dass sie eine Zahl veraendert.
// ★ KORREKTUR 2026-08-08: hier stand auch "Slice-Renderer" auf dieser Liste -- der ist laengst da
// (render_yslice, weiter unten) und hat am selben Tag den Motorraum sichtbar gemacht. Genau die
// Drift zwischen Kopf und Code, die V1 unlesbar gemacht hat.
//
// Vier Messfehler des alten Baums sind hier an der Quelle behoben:
//   1. CSV mit PUNKT als Dezimaltrennzeichen. Das deutsche Komma hat am 2026-08-08 einen
//      Parser still scheitern lassen -- genau die Fehlerklasse, die uns Tage gekostet hat.
//   2. Block-SEM statt Populations-Sigma hinter dem "+-". Das Momentansignal ist stark
//      autokorreliert, seine Standardabweichung ist KEIN Fehlerbalken des Mittelwerts.
//      Zusaetzlich wird die Blocksensitivitaet ausgewiesen: waechst der SEM mit der
//      Blocklaenge weiter, ist die Autokorrelation nicht aufgeloest und die Zahl zu gut.
//   3. Cd wird auf BEIDE Bezugsflaechen ausgegeben -- nominal (Literaturkonvention) und
//      effektiv (die tatsaechlich voxelisierte Silhouette). Der Unterschied ist
//      aufloesungsabhaengig (+8,0 % bei 12 mm, +3,5 % bei 6 mm) und taeuschte bisher eine
//      Gitterkonvergenz vor, die es so nicht gibt.
//   4. Alle Schalter werten ihren WERT aus. "CFD_X=0" heisst aus, nicht an.
// =============================================================================================

// Schalter-Helfer env_on/env_f/env_u: seit C2/E2 (2026-08-15) in utilities.hpp -- eine Semantik
// fuer alle Uebersetzungseinheiten. Historie (Pruefer-Befund M1 zu env_u) steht dort.

// Konservative SAT-Voxelisierung (Akenine-Moeller, "Fast 3D Triangle-Box Overlap", 2001).
// Box = LBM-Zelle (Zentrum c, Halbweite h in Gittereinheiten) gegen Dreieck (V0,V1,V2).
// Markiert JEDE geschnittene Zelle. Upstreams Ray-Paritaets-Voxelizer verliert bei duennen
// achsparallelen Teilen ganze Zellreihen; an der Kugel liess er die Symmetrieebene fast hohl
// (72 statt ~1064 Solidzellen). Offline validiert, deshalb kommt er mit.
static inline bool sat_tri_box_overlap(const float3& V0, const float3& V1, const float3& V2, const float cx, const float cy, const float cz, const float h) {
	const float3 c = float3(cx, cy, cz);
	const float3 v0 = V0-c, v1 = V1-c, v2 = V2-c;
	if(fmin(fmin(v0.x,v1.x),v2.x)> h || fmax(fmax(v0.x,v1.x),v2.x)<-h) return false;
	if(fmin(fmin(v0.y,v1.y),v2.y)> h || fmax(fmax(v0.y,v1.y),v2.y)<-h) return false;
	if(fmin(fmin(v0.z,v1.z),v2.z)> h || fmax(fmax(v0.z,v1.z),v2.z)<-h) return false;
	const float3 f0 = v1-v0, f1 = v2-v1, f2 = v0-v2;
	const float3 nrm = cross(f0, f1);
	const float rN = h*(fabs(nrm.x)+fabs(nrm.y)+fabs(nrm.z));
	const float dN = nrm.x*v0.x+nrm.y*v0.y+nrm.z*v0.z;
	if(fabs(dN)>rN) return false;
	const float3 F[3] = { f0, f1, f2 };
	for(int e=0; e<3; e++) {
		const float fx=F[e].x, fy=F[e].y, fz=F[e].z;
		const float ax[3][3] = { {0.0f,-fz,fy}, {fz,0.0f,-fx}, {-fy,fx,0.0f} };
		for(int k=0; k<3; k++) {
			const float A=ax[k][0], B=ax[k][1], C=ax[k][2];
			const float r = h*(fabs(A)+fabs(B)+fabs(C));
			const float p0=A*v0.x+B*v0.y+C*v0.z, p1=A*v1.x+B*v1.y+C*v1.z, p2=A*v2.x+B*v2.y+C*v2.z;
			if(fmin(fmin(p0,p1),p2)>r || fmax(fmax(p0,p1),p2)<-r) return false;
		}
	}
	return true;
}

// Standardfehler des Mittelwerts ueber Blockmittel. Gibt -1 zurueck, wenn zu wenige Samples.
static double block_sem(const std::vector<double>& v, const uint nblocks) {
	const size_t n = v.size();
	if(n < (size_t)2u*nblocks || nblocks<2u) return -1.0;
	const size_t bs = n/nblocks;
	std::vector<double> m(nblocks, 0.0);
	for(uint b=0u; b<nblocks; b++) { double s=0.0; for(size_t i=(size_t)b*bs; i<(size_t)(b+1u)*bs; i++) s+=v[i]; m[b]=s/(double)bs; }
	double mu=0.0; for(uint b=0u; b<nblocks; b++) mu+=m[b]; mu/=(double)nblocks;
	double var=0.0; for(uint b=0u; b<nblocks; b++) var+=(m[b]-mu)*(m[b]-mu);
	return sqrt(var/(double)(nblocks-1u)/(double)nblocks);
}


// Ein y-Schnitt als PNG. Blau = langsam, weiss = Anstroemung, rot = schnell; Solid schwarz.
// u und flags muessen vorher vom Device gelesen sein -- der Aufrufer entscheidet, wie oft das passiert,
// denn es kostet einen vollen Transfer.
// ★ KADENZ-UMBAU (Heiko 27.08.2026): Zeit + korrigierte Cd/Cz als Text im Slice-Bild.
// Font = die 96 Druck-ASCII-Glyphen (32..127) des 6x11-Monospace-Fonts aus graphics.cpp
// (dort hinter #ifdef GRAPHICS und damit hier nicht linkbar -- Kopie; gleiche Bitlogik
// inkl. q-Sonderfall aus draw_text, graphics.cpp:307ff).
static const ulong slice_font[96] = {
	0x0000000000000000ull, 0x2082082080082000ull, 0x5145000000000000ull, 0x514F94514F945000ull, 0x21CAA870AA9C2000ull, 0x4AA50421052A9000ull,
	0x2145085628A27400ull, 0x2082000000000000ull, 0x0842082082040800ull, 0x4081041041084000ull, 0xA9C72A0000000000ull, 0x000208F882000000ull,
	0x0000000000041080ull, 0x000000F800000000ull, 0x00000000000C3000ull, 0x0821042104208000ull, 0x7228A28A28A27000ull, 0x0862928820820800ull,
	0x722882108420F800ull, 0x7228823028A27000ull, 0x10C51493E1041000ull, 0xFA083C082082F000ull, 0x722820F228A27000ull, 0xF821042084104000ull,
	0x7228A27228A27000ull, 0x7228A27820A27000ull, 0x00030C00030C0000ull, 0x00030C00030C1080ull, 0x0021084081020000ull, 0x00003E03E0000000ull,
	0x0204081084200000ull, 0x7228842080082000ull, 0x7A1B75D75D6E81E0ull, 0x20851453E8A28800ull, 0xF228A2F228A2F000ull, 0x7228208208227000ull,
	0xF228A28A28A2F000ull, 0xFA0820F20820F800ull, 0xFA0820F208208000ull, 0x722820B228A27000ull, 0x8A28A2FA28A28800ull, 0xF88208208208F800ull,
	0xF820820820A27000ull, 0x8A4928C289248800ull, 0x820820820820F800ull, 0x8B6DAAAA28A28800ull, 0x8B2CAAAAA9A68800ull, 0xFA28A28A28A2F800ull,
	0xF228A2F208208000ull, 0x7228A28A28A46800ull, 0xF228A2F289228800ull, 0x7228207020A27000ull, 0xF882082082082000ull, 0x8A28A28A28A27000ull,
	0x8A28A25145082000ull, 0x8A28A28AAA945000ull, 0x8A25142145228800ull, 0x8A25142082082000ull, 0xF82104210420F800ull, 0x3882082082083800ull,
	0x8204102041020800ull, 0xE08208208208E000ull, 0x2148800000000000ull, 0x000000000000F800ull, 0x2040000000000000ull, 0x0000007027A27000ull,
	0x820820F228A2F000ull, 0x0000007A08207800ull, 0x0820827A28A27800ull, 0x000000722FA07000ull, 0x1882087082082000ull, 0x0000007A28A27827ull,
	0x820820F228A28800ull, 0x0002002082082000ull, 0x0002002082082284ull, 0x4104104946144800ull, 0x4104104104103000ull, 0x000000F2AAAAA800ull,
	0x000000F228A28800ull, 0x0000007228A27000ull, 0x0000007124927104ull, 0x0000007249247041ull, 0x0000007124104000ull, 0x0000003907827000ull,
	0x2082087082081800ull, 0x0000008A28A27800ull, 0x0000008A25142000ull, 0x0000008AA7145000ull, 0x0000008942148800ull, 0x0000008A25142084ull,
	0x000000F84210F800ull, 0x0841042041040800ull, 0x2082082082082000ull, 0x8104102104108000ull, 0x000010A840000000ull, 0x0000000000000000ull
};
static void zeichne_text(Image* img, const uint x0, const uint y0, const string& s, const uint skal) {
	// Heiko 27.08.: schlichte SCHWARZE Zeichen direkt aufs Feld, kein Balken (oben links ist Freistrom).
	for(uint i=0u; i<(uint)s.length(); i++) {
		const int c = (int)s[i];
		const ulong bits = slice_font[c>=32&&c<128 ? c-32 : 0];
		for(uint k=0u; k<64u; k++) if((bits>>(63u-k))&1ull) {
			const uint bx = x0+(i*6u+k%6u+(c=='q'?1u:0u))*skal, by = y0+(k/6u)*skal;
			for(uint a=0u; a<skal; a++) for(uint b=0u; b<skal; b++) {
				const uint px=bx+b, py=by+a;
				if(px<img->width()&&py<img->height()) img->set_color((int)px, (int)py, 0x000000);
			}
		}
	}
}
// ★ 04.09.2026 KASKADEN-ABNAHME (Antwort auf Befund B83). Der EXAKTE Solve -- entkoppelt wie
// gekoppelt -- war der einzige Zweig der Degenerationskaskade ohne Zaehler. Folge: wer den
// Vollranganteil wissen wollte, musste ihn per Subtraktion schaetzen, und genau dabei ist mir am
// 03.09. Slot 14 als "Rang 2 voll" durchgegangen -- der ist aber der gekoppelte SKALAR-Rueckfall.
// Ein fehlender Zaehler hat also nicht nur eine Messung verhindert, sondern eine falsche erzeugt.
// Seit Slot 78/79 ist die Kaskade lueckenlos und schliesst sich gegen den Wirkpfad. Und weil der
// Ausloeser ein NENNERFEHLER war, druckt diese Zeile jeden Anteil MIT seinem Nenner.
static void pruefe_kaskade(const uint* H, const string& ort, const bool messnur, const bool pinv) {
	const ulong wp=(ulong)H[7], s9=(ulong)H[9], s17=(ulong)H[17];
	if(wp==0ull) return;
	// ★ 04.09.2026, am ersten Einsatz gelernt: unter MESS-NUR steigt apply_facette_imem VOR der
	// Degenerationskaskade aus (Slot 75 ist der MESSNUR-Wirkpfad) -- alle sechs Kaskadenslots sind
	// dann konstruktiv null, und mein Waechter hat den Arm za_messnur deshalb hart abgebrochen.
	// Der Waechter hatte in der Sache recht und in der Regel unrecht. Statt ihn nur auszunehmen,
	// wird MESS-NUR hier zur EIGENEN Abnahme: die Kaskade MUSS leer sein und Slot 75 MUSS feuern.
	if(messnur) {
		const ulong ks=(ulong)H[78]+(ulong)H[79]+(ulong)H[12]+(ulong)H[13]+(ulong)H[14]+(ulong)H[15], s75=(ulong)H[75];
		if(ks!=0ull) print_error("["+ort+"] MESS-NUR, aber die Solver-Kaskade hat gezaehlt (78+79+12+13+14+15 = "+to_string(ks)+"). Der MESSNUR-Ausstieg greift nicht -- der Arm ist NICHT reines Bounce-Back.");
		else if(s75==0ull) print_error("["+ort+"] MESS-NUR angefordert, aber Slot 75 = 0 -- stiller No-Op, der Ausstieg wird nie erreicht.");
		else print_info("["+ort+"] MESS-NUR-ABNAHME: Kaskade konstruktiv leer, Wirkpfad Slot 75 = "+to_string(s75)+" von "+to_string(wp)+" Besuchen ("+to_string((float)(100.0*(double)s75/(double)wp),2u)+" %). Kein Wandmodellimpuls, Facetten nur gemessen.");
		return;
	}
	const ulong s12=(ulong)H[12], s13=(ulong)H[13], s14=(ulong)H[14], s15=(ulong)H[15];
	const ulong s78=(ulong)H[78], s79=(ulong)H[79], s27=(ulong)H[27], s80=(ulong)H[80];
	const ulong erreicht = wp>(s9+s17) ? wp-s9-s17 : 0ull;
	const ulong summe = s78+s79+s12+s13+s14+s15;
	// ★ 04.09.2026 (Host-Audit H1): als einzige der drei Abnahmen hatte diese keinen Wickel-Ausstieg --
	// pruefe_rueckfall_buchung und pruefe_kraftpfad pruefen beide auf >= 0xF0000000. Wickelt Slot 7,
	// steht der NENNER klein und die Summe gross: der Waechter meldete dann "KASKADE UNVOLLSTAENDIG"
	// bei einer voellig intakten Kaskade. summe > wp ist der zuverlaessige Detektor dafuer, denn ohne
	// Wickel ist die Summe konstruktiv hoechstens der Wirkpfad.
	if(summe>wp) {
		print_warning("["+ort+"] SOLVER-KASKADE nicht auswertbar: Summe "+to_string(summe)+" > Wirkpfad "+to_string(wp)
			+" -- Slot 7 hat den uint-Bereich ueberlaufen. Die Kaskade selbst ist in Ordnung (die Zaehler sind es), nur ihr Nenner nicht."
			+" Anteile werden deshalb NICHT gedruckt, statt falsche zu drucken. Abhilfe: Lauf kuerzen oder Slot 7 saettigend machen.");
		return;
	}
	if(s78>=0xF0000000ull||s79>=0xF0000000ull) {
		print_warning("["+ort+"] SOLVER-KASKADE: Slot 78/79 sind GESAETTIGT (>= 0xF0000000), 12/13/14/15 wickeln dagegen -- gemischte Zaehlweise."
			+" Die Anteile darunter sind Untergrenzen, die Schluss-Identitaet ist nicht mehr aussagekraeftig.");
	}
	const double n = (double)wp;
	print_info("["+ort+"] SOLVER-KASKADE (Anteile je am WIRKPFAD "+to_string(wp)+", nicht an einer Teilmenge):");
	print_info("   exakt entkoppelt [78] "+to_string(s78)+" ("+to_string((float)(100.0*(double)s78/n),2u)+" %)"
		+" | exakt gekoppelt [79] "+to_string(s79)+" ("+to_string((float)(100.0*(double)s79/n),2u)+" %)");
	print_info("   Skalar entkoppelt [12] "+to_string(s12)+" ("+to_string((float)(100.0*(double)s12/n),2u)+" %)"
		+" | "+(pinv?string("Pseudoinverse"):string("Skalar"))+" gekoppelt [14] "+to_string(s14)+" ("+to_string((float)(100.0*(double)s14/n),2u)+" %)  -- BEIDE: Solve-Zweig GEWAEHLT, nicht notwendig angewandt");
	print_info("   Rang 0 entkoppelt [13] "+to_string(s13)+" ("+to_string((float)(100.0*(double)s13/n),2u)+" %, davon rohe Tangentialmomente [27] "
		+to_string(s27)+" = "+to_string((float)(100.0*(double)s27/n),2u)+" % des Wirkpfads)"
		+" | Rang 0 gekoppelt [15] "+to_string(s15)+" ("+to_string((float)(100.0*(double)s15/n),2u)+" %)  -- RUECKFALL, reine Linkgeometrie");
	// ★ 04.09.2026 (Kernel-Audit M3): die Kaskadenslots 78/79/12/14 inkrementieren VOR QUERGATE[64],
	// SATGATE[10] und sn-Gate[16]. Ein Besuch, der danach in den Rueckfall kippt, steht in 14 UND in
	// 10/16/64 UND in 69. "Gewaehlt" ist deshalb nicht "angewandt" -- die alte Zeile behauptete genau
	// das Etikett, gegen das B83 gebaut wurde. Angewandt = gewaehlt MINUS 10/16/64.
	print_info("   Lesehinweis: 78/79/12/14 = Solve-Zweig vor den Gates. ANGEWANDT = diese minus Quergate[64] "
		+to_string((ulong)H[64])+", Sattigungsgate[10] "+to_string((ulong)H[10])+", sn-Gate[16] "+to_string((ulong)H[16])+".");
	// ★ 04.09.2026 (Kernel-Audit M2): Slot 80 feuerte im Kernel, wurde aber nirgends gelesen -- waehrend
	// vier print_info woertlich "Wirkpfad Slot 80" versprechen. Ein Schalter ohne BERICHTETEN Zaehler ist
	// nach Projektregel derselbe harte Fehler wie einer ohne Zaehler: der Arm ist im Binary nicht nachweisbar.
	if(pinv) {
		if(s80==0ull) print_error("["+ort+"] CFD_FAC_PINV angefordert, aber Slot 80 = 0 -- die Rang-1-Pseudoinverse wird NIE erreicht (stiller No-Op). Entweder faellt der Vollrangzweig [79] nicht durch, oder die Akzeptanzschwelle tr >= 1e-4*(G11+G22) haelt nicht.");
		else print_info("   Rang-1-Pseudoinverse [80] "+to_string(s80)+" ("+to_string((float)(100.0*(double)s80/n),2u)+" % des Wirkpfads) -- Teilmenge von [14]; Rest von [14] = "+to_string(s14>s80?s14-s80:(ulong)0)+" (Skalarleiter, nur unter LSQ).");
	} else if(s80!=0ull) print_error("["+ort+"] Slot 80 = "+to_string(s80)+" OHNE CFD_FAC_PINV -- der Pseudoinversen-Zweig laeuft, obwohl er nicht angefordert wurde.");
	if(summe!=erreicht) {
		const ulong d = summe>erreicht ? summe-erreicht : erreicht-summe;
		const double rel = 100.0*(double)d/(double)(erreicht>0ull?erreicht:1ull);
		if(rel>0.5) print_error("["+ort+"] KASKADE UNVOLLSTAENDIG: 78+79+12+13+14+15 = "+to_string(summe)
			+" gegen Wirkpfad - Skips = "+to_string(erreicht)+" (Abweichung "+to_string((float)rel,2u)
			+" %). Ein Pfad durch apply_facette_imem wird NICHT gezaehlt -- jede Anteilsangabe aus diesen Slots ist damit unbelegt.");
		else print_warning("["+ort+"] Kaskade schliesst auf "+to_string((float)rel,3u)+" % (Saettigung/Zaehlfenster) -- Summe "+to_string(summe)+" gegen "+to_string(erreicht)+".");
	} else print_info("   Abnahme: 78+79+12+13+14+15 = "+to_string(summe)+" == Wirkpfad - Skips[9] - PEMA[17]. Kaskade lueckenlos.");
}
// ★ BUCHUNGSSCHLUSS-DETEKTOR (Baustein 2/1, 27.08.): jeder Gate-Rueckfall bucht GENAU EINMAL (Slot 69).
// Alle beteiligten Zaehler sind t%100-gegatet am selben t und je Besuch exklusiv -> Identitaet
// 69 == 13+15+64 (+10+16 unter SATGATE; ohne SATGATE sind 10/16 Klemmen, die angewandt buchen).
// ★★ g-DIAGNOSE (31.08.2026): Host-Referenz der drei Sensoren + Selbsttest (Iron Rule 3: die
// Diagnostik beweist sich an kleinen Testfaellen, bevor sie misst). Referenzwerte aus der
// unabhaengigen double-Rechnung (numpy-SVD, 31.08.); Toleranz 1e-5 relativ.
static void jacobi3(double M[3][3], double ew[3], double ev[3][3]); // Vorwaertsdeklaration (Definition weiter unten, beim Facettenbau)
static void gdiag_sensoren(const double g[3][3], double* snorm, double* wale, double* sigm) {
	double S[3][3], SS=0.0;
	for(int i=0;i<3;i++) for(int a=0;a<3;a++) { S[i][a]=0.5*(g[i][a]+g[a][i]); SS+=S[i][a]*S[i][a]; }
	*snorm = sqrt(2.0*SS);
	double gg[3][3];
	for(int i=0;i<3;i++) for(int a=0;a<3;a++) gg[i][a]=g[i][0]*g[0][a]+g[i][1]*g[1][a]+g[i][2]*g[2][a];
	const double tr3=(gg[0][0]+gg[1][1]+gg[2][2])/3.0;
	double SdSd=0.0;
	for(int i=0;i<3;i++) for(int a=0;a<3;a++) { const double Sd=0.5*(gg[i][a]+gg[a][i])-(i==a?tr3:0.0); SdSd+=Sd*Sd; }
	*wale = (SdSd*sqrt(SdSd))/(SS*SS*sqrt(SS)+SdSd*sqrt(sqrt(SdSd))+1e-300);
	double M[3][3]={{0,0,0},{0,0,0},{0,0,0}}; // G = g^T g
	for(int a=0;a<3;a++) for(int b=0;b<3;b++) for(int i=0;i<3;i++) M[a][b]+=g[i][a]*g[i][b];
	double ew[3], ev[3][3]; jacobi3(M, ew, ev);
	double s_[3]={sqrt(fmax(ew[0],0.0)),sqrt(fmax(ew[1],0.0)),sqrt(fmax(ew[2],0.0))};
	std::sort(s_, s_+3, std::greater<double>());
	*sigm = s_[0]>1e-30 ? s_[2]*(s_[0]-s_[1])*(s_[1]-s_[2])/(s_[0]*s_[0]) : 0.0;
}
static void sgs_gdiag_selbsttest() {
	static bool einmal=false; if(einmal) return; einmal=true;
	const double G[5][3][3] = {
		{{0,1,0},{0,0,0},{0,0,0}},          // reine Scherung
		{{0,1,0},{-1,0,0},{0,0,0}},         // reine Rotation
		{{0,1,0},{-0.3,0,0.4},{0.2,0.5,0}}, // Scherung+Rotation
		{{1,0,0},{0,-0.5,0},{0,0,-0.5}},    // axiale Dehnung
		{{0,1,0},{0.6,0,0},{0,0,0}}};       // Stufe
	const double soll[5][3] = { // snorm, wale, sigma -- unabhaengige numpy-Referenz 31.08.
		{1.000000000, 0.000000000, 0.000000000},
		{0.000000000, 0.903602004, 0.000000000},
		{1.157583690, 0.158731813, 0.025153979},
		{1.732050808, 0.075313192, 0.000000000},
		{1.600000000, 0.058159040, 0.000000000}};
	for(int f=0; f<5; f++) {
		double sn, wa, si; gdiag_sensoren(G[f], &sn, &wa, &si);
		const double ist[3]={sn,wa,si};
		for(int k=0;k<3;k++) if(fabs(ist[k]-soll[f][k])>1e-5*(1.0+fabs(soll[f][k])))
			print_error("g-DIAGNOSE-Selbsttest VERLETZT: Fall "+to_string(f+1)+", Sensor "+to_string(k)
				+": ist "+to_string((float)ist[k],6u)+" gegen Referenz "+to_string((float)soll[f][k],6u)+" -- Formeln nicht anfassen, Fehler suchen.");
	}
	print_info("g-DIAGNOSE-Selbsttest bestanden: 5 Referenztensoren x 3 Sensoren (|S|, WALE, Sigma) auf 1e-5 gegen die unabhaengige double-Referenz.");
}
// ★ Endbericht der g-Diagnose: Mittel je Treppenklasse (eigene_links, y_w) -- die Zahlen, die den
// Modellstreit entscheiden: Pi/FD (Geistermoden?), WALE/FD und Sigma/FD (was liefern die Kandidaten?),
// Omega/S (wie rein ist die Scherung?).
static void bericht_gdiag(LBM_Domain* D, const std::vector<Facette>& F, const string& out_dir, const string& ort) {
	if(D!=nullptr&&!D->gdiag_on&&env_u("CFD_SGS_GDIAG",0u)>0u)
		print_error("["+ort+"] g-Diagnose war ANGEFORDERT (CFD_SGS_GDIAG), an dieser Domaene aber nie alloziert -- stiller No-Op. Statik-Lebensdauer pruefen (B1-Falle: ein spaeteres Parsing hat s_sgs_gdiag ueberschrieben, bevor alloc_facetten_domain lief; genau so am 02.09. den ersten w_gdiag-Lauf verloren)."); // Iron Rule: Schalter ohne feuernden Zaehler = harter Fehler
	if(D==nullptr||!D->gdiag_on) return;
	D->fac_gd.read_from_device();
	struct Agg { ulong n=0ull; double v=0.0, sfd=0.0, spi=0.0, wa=0.0, si=0.0, om=0.0, ns=0.0; };
	std::map<std::pair<uint,int>,Agg> M; ulong k=0ull;
	for(const Facette& f : F) { if(f.klasse!=0u) continue; if(8ull*k+7ull>=D->fac_gd.length()) break;
		const float* a=&D->fac_gd[8ull*k]; k++;
		Agg& g=M[std::make_pair(f.eigene_links,(int)lround(100.0f*f.yw))];
		g.n++; g.v+=a[5]; g.sfd+=a[0]; g.spi+=a[1]; g.wa+=a[2]; g.si+=a[3]; g.om+=a[4]; g.ns+=a[6]; }
	std::vector<std::pair<std::pair<uint,int>,Agg>> V(M.begin(), M.end());
	std::sort(V.begin(), V.end(), [](const std::pair<std::pair<uint,int>,Agg>& x, const std::pair<std::pair<uint,int>,Agg>& y){ return x.second.n>y.second.n; });
	print_info("["+ort+"] g-DIAGNOSE (CFD_SGS_GDIAG): "+to_string(k)+" Wandzellen in "+to_string((ulong)V.size())+" Klassen. Mittel je Besuch (Gittereinheiten):");
	print_info("  links  y_w     n_fac   Besuche/fac  |S|_FD     |S|_Pi     Pi/FD   WALE/FD  Sigma/FD  |Om|/|S|  nsolid");
	std::ofstream fk(out_dir+"facetten_gdiag.csv"); fk.precision(7);
	fk << "# g-Diagnose ("<<ort<<", CFD_SGS_GDIAG): Mittel je Besuch, gruppiert nach (eigene_links, y_w auf 0,01)\n";
	fk << "eigene_links,yw,n_fac,besuche_je_fac,s_fd,s_pi,pi_zu_fd,wale_zu_fd,sigma_zu_fd,omega_zu_s,nsolid_je_besuch\n";
	uint zeilen=0u;
	for(const auto& e : V) { const Agg& g=e.second; const double v=g.v>0.0?g.v:1.0;
		const double sfd=g.sfd/v, spi=g.spi/v, wa=g.wa/v, si=g.si/v, om=g.om/v, ns=g.ns/v;
		const double pf=sfd>0.0?spi/sfd:0.0, wf=sfd>0.0?wa/sfd:0.0, gf=sfd>0.0?si/sfd:0.0, os=sfd>0.0?om/sfd:0.0;
		fk << e.first.first << "," << 0.01*e.first.second << "," << g.n << "," << g.v/(double)g.n << "," << sfd << "," << spi << "," << pf << "," << wf << "," << gf << "," << os << "," << ns << "\n";
		if(zeilen++<16u) { char z[300]; snprintf(z, sizeof(z), "  %5u  %5.2f  %7lu  %10.1f  %9.3e  %9.3e  %7.2f  %7.3f  %8.3f  %8.3f  %6.2f", e.first.first, 0.01*e.first.second, (unsigned long)g.n, g.v/(double)g.n, sfd, spi, pf, wf, gf, os, ns); print_info(string(z)); } }
	fk.close(); print_info("  CSV: "+out_dir+"facetten_gdiag.csv ("+to_string((ulong)V.size())+" Klassen). LESART: Pi/FD >> 1 = Geistermoden des Wandmodells im Smagorinsky-Tensor; WALE/FD bzw. Sigma/FD = was die Kandidaten relativ zu Smagorinsky liefern wuerden (Smagorinsky misst |S|); |Om|/|S| = 1 bei reiner Scherung.");
}
// ★ KLASSEN-DIAGNOSTIK (Weg-1-Plan Stufe 0, 30.08.): fac_kd je Facette -> Mittel je Treppenklasse (eigene_links, y_w).
// Beantwortet je Klasse: welches u_t geht ins Modell, welches Ziel (tw physikalisch, twe angewandt), wie gross ist
// der Linkaustausch |P1| dagegen, was wird angewandt (s1) und gebucht (phi1), wie oft faellt die Klasse zurueck.
static void bericht_klassen(LBM_Domain* D, const std::vector<Facette>& F, const string& out_dir, const double tau_ziel, const string& ort) {
	if(D==nullptr||!D->fac_kdiag_on) return;
	D->fac_kd.read_from_device();
	struct Agg { ulong n=0ull; double v=0.0, ut=0.0, tw=0.0, twe=0.0, p1=0.0, s1=0.0, phi=0.0, rf=0.0, uta=0.0, ywa=0.0, twa=0.0, va=0.0; }; // twa/va (04.09.): tw und Besuche NUR ueber angewandte Besuche
	std::map<std::pair<uint,int>,Agg> M; ulong k=0ull;
	for(const Facette& f : F) { if(f.klasse!=0u) continue; if(12ull*k+11ull>=D->fac_kd.length()) break;
		const float* a=&D->fac_kd[12ull*k]; k++;
		Agg& g=M[std::make_pair(f.eigene_links,(int)lround(100.0f*f.yw))];
		g.n++; g.v+=a[7]; g.ut+=a[0]; g.tw+=a[1]; g.twe+=a[2]; g.p1+=a[3]; g.s1+=a[4]; g.phi+=a[5]; g.rf+=a[6]; g.uta+=a[8]; g.ywa+=a[9]; g.twa+=a[10]; g.va+=a[11]; }
	std::vector<std::pair<std::pair<uint,int>,Agg>> V(M.begin(), M.end());
	std::sort(V.begin(), V.end(), [](const std::pair<std::pair<uint,int>,Agg>& x, const std::pair<std::pair<uint,int>,Agg>& y){ return x.second.n>y.second.n; });
	print_info("["+ort+"] KLASSEN-DIAGNOSTIK (CFD_FAC_KDIAG): "+to_string(k)+" Facetten in "+to_string((ulong)V.size())+" Klassen (eigene_links, y_w). Mittel je Besuch"+(tau_ziel>0.0?" -- tw/Ziel gegen tau_ziel "+to_string((float)tau_ziel,9u):string(""))+":");
	print_info("  links  y_w     n_fac    Besuche/fac   u_t        u_t_abt    y_abt   tw         twe        |P1|       s1         phi1       Rueckfall%   tw/Ziel  |P1|/twe   s1/u_t"); // s1/u_t (03.09.): Slip relativ zur Zellgeschwindigkeit = Randwert-Inkonsistenz des FD-Sensors (sgs_fdwand setzt u=0 am Solid)
	std::ofstream fk(out_dir+"facetten_klassen.csv"); fk.precision(7);
	fk << "# Klassen-Diagnostik ("<<ort<<", CFD_FAC_KDIAG): Mittel je Facettenbesuch, gruppiert nach (eigene_links, y_w auf 0,01); tw_ziel = 0 wenn kein Ziel bekannt\n";
	fk << "eigene_links,yw,n_fac,besuche_je_fac,ut,ut_abtast,yw_abtast,tw,twe,absP1,s1,phi1,rueckfall_pct,tw_ziel,absP1_twe,s1_ut\n";
	uint zeilen=0u;
	for(const auto& e : V) { const Agg& g=e.second; const double v=g.v>0.0?g.v:1.0;
		const double ut=g.ut/v, tw=g.tw/v, twe=g.twe/v, p1=g.p1/v, s1=g.s1/v, phi=g.phi/v, rf=100.0*g.rf/v, tz=tau_ziel>0.0?tw/tau_ziel:0.0, pt=twe!=0.0?p1/fabs(twe):0.0, uta=g.uta/v, ywa=g.ywa/v, su=ut!=0.0?s1/ut:0.0;
		fk << e.first.first << "," << 0.01*e.first.second << "," << g.n << "," << g.v/(double)g.n << "," << ut << "," << uta << "," << ywa << "," << tw << "," << twe << "," << p1 << "," << s1 << "," << phi << "," << rf << "," << tz << "," << pt << "," << su << "\n";
		if(zeilen++<16u) { char z[320]; snprintf(z, sizeof(z), "  %5u  %5.2f  %8lu  %11.1f  %9.5f  %9.5f  %6.2f  %9.3e  %9.3e  %9.3e  %+9.3e  %+9.3e  %8.1f    %6.3f   %7.2f  %+7.3f", e.first.first, 0.01*e.first.second, (unsigned long)g.n, g.v/(double)g.n, ut, uta, ywa, tw, twe, p1, s1, phi, rf, tz, pt, su); print_info(string(z)); } }
	fk.close(); print_info("  CSV: "+out_dir+"facetten_klassen.csv ("+to_string((ulong)V.size())+" Klassen)");
}
// ★ KRAFTPFAD-WAECHTER (30.08.): Slot 70 zaehlt Zellen, an denen das Wandmodell-Residuum als
// Volumenkraft eingetragen wurde. Modus 1: exakt die Rueckfallzellen (== Slot 69). Modus 0: muss 0 sein.
// Modus 2: alle Solve-Zellen (Wirkpfad minus Slot 9/17). LESEHINWEIS Modus 2 (Pruefbefund B4): der Slip-Solve laeuft
// dort weiter und s wird verworfen -- Slots 10/12/14/16/64 zaehlen dann, was der Slip-Pfad GETAN HAETTE, nicht was
// angewandt wurde. R1/R2 haengen nicht von s ab, die Kraft ist davon unberuehrt.
static void pruefe_kraftpfad(const ulong h70, const ulong h69, const ulong h7, const ulong h9, const ulong h17, const ulong h71, const uint modus, const string& ort) {
	if(modus>0u) print_info("["+ort+"] Kraftpfad ANLAUF (t<100, ungegatet, Slot 71): "+to_string(h71)+" Kraftzellen-Ereignisse -- die Anlauf-Transiente ist fuer die t%100-Stichprobe der Slots 69/70 unsichtbar (Bitanker-Befund 30.08.: kipp0 = fac_N genau einmal).");
	if(modus==0u) { if(h70!=0ull) print_error("["+ort+"] Kraftpfad Slot 70 = "+to_string(h70)+" ohne CFD_FAC_KRAFT -- Zaehler feuert ohne Schalter."); return; }
	if(h70>=0xF0000000ull) { print_info("["+ort+"] Kraftpfad Slot 70 saettigt -- Identitaet nicht pruefbar."); return; }
	if(modus==1u) { if(h70!=h69) print_error("["+ort+"] Kraftpfad Slot 70 = "+to_string(h70)+" != Rueckfall Slot 69 = "+to_string(h69)+" -- Modus 1 muss GENAU die Rueckfallzellen tragen.");
		else print_info("["+ort+"] KRAFTPFAD Modus 1: "+to_string(h70)+" Kraftzellen == Slot 69 -- jede Rueckfallzelle traegt ihr Residuum als Kraft ("+to_string((float)(h7>0ull?100.0*(double)h70/(double)h7:0.0),1u)+" % des Wirkpfads)."); }
	else { // ★ Pruefbefund B3 (30.08.): Modus 2 hat ein Soll. Slot 7 zaehlt VOR den beiden fruehen Returns (Slot 9 u_t~0, Slot 17 PEMA-u_t~0); danach kein Return mehr bis zur Kraftbuchung.
		const ulong soll = h7-(h9+h17<=h7?h9+h17:h7);
		if(h70!=soll) print_error("["+ort+"] Kraftpfad Slot 70 = "+to_string(h70)+" != Soll 7-9-17 = "+to_string(soll)+" (Wirkpfad "+to_string(h7)+", u_t~0 "+to_string(h9)+", PEMA-u_t~0 "+to_string(h17)+") -- Modus 2 muss GENAU alle Solve-Zellen tragen.");
		else print_info("["+ort+"] KRAFTPFAD Modus 2: "+to_string(h70)+" Kraftzellen == Wirkpfad-9-17 = "+to_string((float)(h7>0ull?100.0*(double)h70/(double)h7:0.0),1u)+" % des Wirkpfads (alle Solve-Zellen; Rueckfaelle darin "+to_string(h69)+")."); }
}
// ★ 04.09.2026 ZIELERFUELLUNG. Beantwortet die Frage, die "Anteil MIT Modell" NICHT beantwortet:
// wieviel vom Wandschub-Ziel kommt in den angewandten Zellen tatsaechlich an? Die Rang-1-Pseudoinverse
// erfuellt das Ziel nur in Richtung ihres einen erreichbaren Eigenvektors -- eine Zelle kann als
// "angewandt" zaehlen und trotzdem nur einen Bruchteil aufpraegen. Slots 81-89, Kernel ~2430.
static void bericht_zielerfuellung(const uint* H, const ulong wp, const ulong s9, const ulong s17, const ulong s69, const string& ort) {
	const ulong n=(ulong)H[81];
	if(n==0ull) { if(wp>0ull) print_warning("["+ort+"] ZIELERFUELLUNG: Slot 81 = 0, obwohl der Wirkpfad "+to_string(wp)+" zaehlt -- das Histogramm wurde nie erreicht (MESS-NUR? frueher Ausstieg?)."); return; }
	const ulong ohne=(ulong)H[82];
	ulong b[9], sum=0ull; for(uint i=0u;i<9u;i++) { b[i]=(ulong)H[83u+i]; sum+=b[i]; }
	const ulong ang=ohne+sum;
	const double dn=(double)n;
	print_info("["+ort+"] ZIELERFUELLUNG des Wandmodells (Stichprobe: jeder 100. Schritt, jede 64. Facette):");
	// Gegenprobe: zwei unabhaengige Instrumente muessen denselben angewandten Anteil sehen.
	const ulong erreicht = wp>(s9+s17) ? wp-s9-s17 : 0ull;
	const double q_hist = 100.0*(double)ang/dn;
	const double q_slot = erreicht>0ull ? 100.0*(double)(erreicht>s69?erreicht-s69:0ull)/(double)erreicht : 0.0;
	print_info("   angewandt in der Stichprobe: "+to_string(ang)+" von "+to_string(n)+" = "+to_string((float)q_hist,2u)
		+" %  |  GEGENPROBE aus Slot 7/9/17/69: "+to_string((float)q_slot,2u)+" %");
	if(fabs(q_hist-q_slot)>2.0) print_warning("   Die beiden Instrumente weichen um "+to_string((float)fabs(q_hist-q_slot),2u)
		+" Prozentpunkte ab. Sie messen dieselbe Groesse auf verschiedenen Wegen -- eine Abweichung ueber 2 pp heisst, dass eines von beiden nicht misst, was sein Name sagt.");
	if(ang==0ull) { print_warning("   Kein einziger angewandter Besuch in der Stichprobe -- die Verteilung unten ist leer."); return; }
	const double da=(double)ang;
	const char* nam[9] = {"r <= -10       GEGENRICHTUNG, um Dekaden zu stark","-10 < r <= -1  GEGENRICHTUNG, mind. volle Zielstaerke","-1 < r < 0     GEGENRICHTUNG, schwach",
	                      " 0 <= r < 0,5  stark unter Ziel"," 0,5 - 0,9     unter Ziel"," 0,9 - 1,1     TREFFER",
	                      " 1,1 - 2,0     ueber Ziel"," 2 - 10        stark ueber Ziel"," r >= 10       um Dekaden zu stark"};
	print_info("   r = aufgepraegter Tangentialimpuls / Wandschub-Ziel, JE BESUCH (exakt erfuellt => r = 1):");
	for(uint i=0u;i<9u;i++) print_info("     "+string(nam[i])+"  "+to_string(b[i])+"  ("+to_string((float)(100.0*(double)b[i]/da),2u)+" % der angewandten)");
	if(ohne>0ull) print_info("     ohne Ziel  (twe = 0)         "+to_string(ohne)+"  ("+to_string((float)(100.0*(double)ohne/da),2u)+" % -- kein Wandschub gefordert, r undefiniert)");
	print_info("   KERNAUSSAGE: "+to_string((float)(100.0*(double)b[5]/da),2u)+" % der ANGEWANDTEN Besuche treffen ihr Ziel auf +-10 %. "
		+"Bezogen auf ALLE Wandbesuche sind das "+to_string((float)(100.0*(double)b[5]/dn),2u)+" % -- das ist die Zahl, die 'Anteil mit Modell' ("
		+to_string((float)q_hist,2u)+" %) NICHT sagt.");
}
static void pruefe_rueckfall_buchung(const ulong h69, const ulong h10, const ulong h13, const ulong h15, const ulong h16, const ulong h64, const bool satgate, const string& ort) {
	const ulong soll = h13+h15+h64+(satgate?h10+h16:0ull);
	const string formel = string("13+15+64")+(satgate?"+10+16":"");
	if(h69>=0xF0000000ull) print_info("["+ort+"] Rueckfall-Buchung Slot 69 saettigt ("+to_string(h69)+", Soll "+to_string(soll)+") -- Identitaet nicht pruefbar.");
	else if(h69!=soll) print_error("["+ort+"] Rueckfall-Buchung Slot 69 = "+to_string(h69)+" != Soll "+to_string(soll)+" ("+formel+") -- Rueckfall bucht NICHT genau einmal (Doppelzaehlungs-Detektor).");
	else print_info("["+ort+"] Rueckfall-Buchung Slot 69 = "+to_string(h69)+" == Soll ("+formel+") -- jeder Gate-Rueckfall bucht genau einmal (P-only).");
}
static void render_yslice(LBM& L, const uint Nx, const uint Ny, const uint Nz, const uint y_slice,
                          const float u2si, const float u_ref_si, const int t_ms, const string& dir, const string& tag, const string& info="") {
	Image img(Nx, Nz);
	for(uint z=0u; z<Nz; z++) for(uint x=0u; x<Nx; x++) {
		const ulong n = (ulong)x + (ulong)Nx*((ulong)y_slice + (ulong)Ny*(ulong)z);
		int col;
		// ★ KORREKTUR 2026-08-08 (Pruefer-Befund M4): hier stand `(flags & (TYPE_S|TYPE_X)) != 0`.
		// TYPE_MS -- die Markierung fuer FLUIDzellen neben einer bewegten Wand -- ist 0x03 und enthaelt
		// damit TYPE_S. Die alte Maske malte also einen Saum aus Fluidzellen rund um Boden und Fahrzeug
		// schwarz: das Diagnosebild log genau dort ueber die Geometrie, wo man am genauesten hinsieht.
		// Richtig ist der Vergleich auf die BEIDEN Randbits: nur 0x01 allein ist Solid, 0x03 ist es nicht.
		if((L.flags[n]&(TYPE_S|TYPE_E))==TYPE_S) col = 0x000000; // Solid (Boden oder Fahrzeug)
		else {
			const float ux=L.u.x[n], uy=L.u.y[n], uz=L.u.z[n];
			// Skala relativ zur Anstroemung: 0.5*u_ref (blau) .. u_ref (weiss) .. 1.5*u_ref (rot)
			float v = u2si*sqrt(ux*ux+uy*uy+uz*uz);
			v = fmin(1.5f*u_ref_si, fmax(0.5f*u_ref_si, v));
			int r,g,b;
			if(v<=u_ref_si) { const float t=(v-0.5f*u_ref_si)/(0.5f*u_ref_si); r=(int)(255.0f*t+0.5f); g=r; b=255; }
			else            { const float t=(v-u_ref_si)/(0.5f*u_ref_si);      r=255; g=(int)(255.0f*(1.0f-t)+0.5f); b=g; }
			col = (r<<16)|(g<<8)|b;
		}
		img.set_color(x, Nz-1u-z, col); // Bildzeile 0 ist oben, z waechst nach oben
	}
	// ★ Heiko 2026-08-08: alle Ausgaben eines Laufs in EINEN Ordner, keine Unterordner. Der Name traegt
	// die Zuordnung (nah/fern) und die Zeit in Millisekunden, damit die Dateien von selbst sortieren.
	if(!info.empty()) zeichne_text(&img, 8u, 8u, info, 3u); // Kadenz-Einblendung (Heiko 27.08.): t + cd/cz_rest
	string ms = to_string(t_ms); while(ms.length()<6u) ms = "0"+ms;
	write_png(dir+"schnitt_"+tag+"_"+ms+"ms.png", &img);
}

// --------------------------------------------------------------------------------------------
// DIFF-SCHNITT nah GEGEN fern (Heiko-Auftrag 2026-08-21, Diagnose der Kopplungsrichtung).
// ANSICHT EXAKT WIE "schnitt_nah": y-Mittelebene des FEINgitters ueber die volle Nahfeld-
// Ausdehnung, gleiche Bildgroesse, gleiche Orientierung. Gezeigt wird an derselben WELTposition
//   d = |u_nah| - |u_fern|   in m/s,   blau = -15, weiss = 0, rot = +15.
// Das Fernfeld wird dazu TRILINEAR auf den Feinzellmittelpunkt interpoliert, in der Deckungs-
// punkt-Konvention der Kopplung (fein f  <->  grob NF_O + f/ratio, siehe Listenbau der Schale).
// Das ist hier keine Kosmetik: bei geradem cey faellt die Feinmittelebene fNy/2 GENAU ZWISCHEN
// zwei Grobzellen (fNy/2=158, ratio=4 -> yc = NF_OY+39,5), eine Nachbarzelle waere um eine halbe
// Grobzelle = 16 mm querversetzt und wuerde eine Scheindifferenz malen.
// Ecken, die im Fernfeld Solid sind (Treppenkoerper/Fahrbahn), fallen aus der Summe; die
// verbleibenden Gewichte werden renormiert. Ist KEINE Ecke gueltig, wird die Zelle grau gemalt
// und in der CSV als fern_ungueltig=1 gefuehrt -- ein grauer Fleck ist damit "kein Fernwert",
// nicht "Differenz null".
// IRON RULE 5: das PNG ist nur die Sichtung. GEMESSEN wird an schnitt_diff_letzter.csv -- dieselbe
// Ebene zellweise mit BEIDEN Geschwindigkeitsvektoren. Sie wird bei jedem Schnitt neu geschrieben
// und haelt damit immer den zuletzt gerechneten Zeitschritt (abbruchfest, und jede andere
// Faerbung/Metrik ist daraus ohne neuen Lauf ableitbar).
// ★ Gemeinsame j0-Konvention des Diff-Schnitts (Pruefagent MITTEL-1, 2026-08-26: die Formel
// existiert nur EINMAL): yc = NF_OY + y_f/ratio, j0 = clamp(floor(yc), 0, cNy-2). Wird von
// render_yslice_diff UND dem Slice-Ebenen-Read-Block benutzt -- ein Auseinanderlaufen der
// beiden Stellen waere ein stiller Ebenen-Versatz.
static uint diff_j0(const uint NF_OY, const uint y_f, const uint ratio, const uint cNy, float* yc_out=nullptr) {
	const float yc = (float)NF_OY + (float)y_f/(float)ratio;
	if(yc_out) *yc_out = yc; // Auditor-A NIEDRIG-3: yc und j0 aus EINER Quelle, damit ty nie gegen j0 laufen kann
	return (uint)fmin(fmax(floor(yc), 0.0f), (float)cNy-2.0f);
}

static void render_yslice_diff(LBM& F, LBM& C,
                               const uint fNx, const uint fNy, const uint fNz,
                               const uint cNx, const uint cNy, const uint cNz,
                               const uint NF_OX, const uint NF_OY, const uint NF_OZ, const uint ratio,
                               const float u2si, const float d_span,
                               const float near_x0, const float near_z0, const float dx_f,
                               const int t_ms, const string& dir) {
	const uint y_f = fNy/2u;
	float yc; const int j0 = (int)diff_j0(NF_OY, y_f, ratio, cNy, &yc); // gemeinsame Quelle fuer yc UND j0 (schleifeninvariant)
	Image img(fNx, fNz);
	string ms = to_string(t_ms); while(ms.length()<6u) ms = "0"+ms;
	std::ofstream csv(dir+"schnitt_diff_letzter.csv"); csv.precision(6);
	csv << "# d_betrag_ms ist SIGNIERT: |u_nah| - |u_fern| (nicht ||u_nah-u_fern||); Solid-Zeilen tragen Init-u (Instrumenten-Audit 2026-08-22)\n";
	csv << "t_ms,x_idx,z_idx,x_m,z_m,unah_x,unah_y,unah_z,ufern_x,ufern_y,ufern_z,d_betrag_ms,nah_solid,fern_ungueltig\n";
	ulong n_fluid=0ull, n_nsolid=0ull, n_ungueltig=0ull; double sum2=0.0, dmax=0.0;
	for(uint z=0u; z<fNz; z++) for(uint x=0u; x<fNx; x++) {
		const ulong n = (ulong)x + (ulong)fNx*((ulong)y_f + (ulong)fNy*(ulong)z);
		// Solid-Maske exakt wie render_yslice: NUR 0x01 allein ist Solid, TYPE_MS (0x03) ist Fluid.
		const bool nah_solid = (F.flags[n]&(TYPE_S|TYPE_E))==TYPE_S;
		const float nx_=u2si*F.u.x[n], ny_=u2si*F.u.y[n], nz_=u2si*F.u.z[n];
		// ---- Fernfeld trilinear am selben Weltpunkt
		const float xc = (float)NF_OX + (float)x/(float)ratio;
		const float zc = (float)NF_OZ + (float)z/(float)ratio;
		const int i0 = (int)fmin(fmax(floor(xc), 0.0f), (float)cNx-2.0f);
		const int k0 = (int)fmin(fmax(floor(zc), 0.0f), (float)cNz-2.0f);
		const float tx=xc-(float)i0, ty=yc-(float)j0, tz=zc-(float)k0;
		float wsum=0.0f, fx_=0.0f, fy_=0.0f, fz_=0.0f;
		for(int dk=0; dk<2; dk++) for(int dj=0; dj<2; dj++) for(int di=0; di<2; di++) {
			const float w = (di?tx:1.0f-tx)*(dj?ty:1.0f-ty)*(dk?tz:1.0f-tz);
			if(w<=0.0f) continue;
			const ulong m = (ulong)(i0+di) + (ulong)cNx*((ulong)(j0+dj) + (ulong)cNy*(ulong)(k0+dk));
			if((C.flags[m]&(TYPE_S|TYPE_E))==TYPE_S) continue; // Solid faellt aus der Summe
			wsum += w; fx_ += w*C.u.x[m]; fy_ += w*C.u.y[m]; fz_ += w*C.u.z[m];
		}
		const bool fern_ungueltig = !(wsum>0.0f);
		if(!fern_ungueltig) { fx_=u2si*fx_/wsum; fy_=u2si*fy_/wsum; fz_=u2si*fz_/wsum; }
		else { fx_=0.0f; fy_=0.0f; fz_=0.0f; }
		const float bn = sqrt(nx_*nx_+ny_*ny_+nz_*nz_), bf = sqrt(fx_*fx_+fy_*fy_+fz_*fz_);
		const float d = bn-bf;
		// ---- Faerbung
		int col;
		if(nah_solid)            { col = 0x000000; n_nsolid++; }      // Nahfeld-Solid: schwarz wie im nah-Schnitt
		else if(fern_ungueltig)  { col = 0x808080; n_ungueltig++; }   // kein gueltiger Fernwert: grau
		else {
			n_fluid++; sum2 += (double)d*(double)d; dmax = fmax(dmax, (double)fabs(d));
			const float dc = fmin(d_span, fmax(-d_span, d));
			int r,g,b;
			if(dc<0.0f) { const float t=1.0f+dc/d_span; r=(int)(255.0f*t+0.5f); g=r; b=255; } // -span blau -> 0 weiss
			else        { const float t=1.0f-dc/d_span; r=255; g=(int)(255.0f*t+0.5f); b=g; } // 0 weiss -> +span rot
			col = (r<<16)|(g<<8)|b;
		}
		img.set_color(x, fNz-1u-z, col); // Bildzeile 0 ist oben, z waechst nach oben -- wie render_yslice
		csv << t_ms << "," << x << "," << z << "," << (near_x0+(float)x*dx_f) << "," << (near_z0+(float)z*dx_f) << ","
		    << nx_ << "," << ny_ << "," << nz_ << "," << fx_ << "," << fy_ << "," << fz_ << "," << d << ","
		    << (nah_solid?1u:0u) << "," << (fern_ungueltig?1u:0u) << "\n";
	}
	csv << std::flush;
	write_png(dir+"schnitt_diff_"+ms+"ms.png", &img);
	print_info("[DIFF-SCHNITT] t = "+to_string(t_ms)+" ms: "+to_string(n_fluid)+" auswertbare Zellen, RMS |d| = "
		+to_string((float)(n_fluid>0ull?sqrt(sum2/(double)n_fluid):0.0),4u)+" m/s, max |d| = "+to_string((float)dmax,3u)
		+" m/s (Skala +-"+to_string(d_span,1u)+"); Nahfeld-Solid "+to_string(n_nsolid)+", ohne gueltigen Fernwert "
		+to_string(n_ungueltig)+" (grau). Echtdaten: "+dir+"schnitt_diff_letzter.csv");
}

// ★ CFD_SLICE_PRUEF (Muster CFD_FAC_GPU_PRUEF): vergleicht fuer EINE y-Ebene den Gather-Pfad
// (lese_yslice_in_host) gegen den frisch VOLL gelesenen Host-Stand. Aufruf NACH vollen
// u/rho/flags-Reads desselben Zeitschritts (kein Kernel dazwischen -> Soll: exakt 0).
// Laesst die Host-Arrays im Gather-Zustand zurueck.
static void pruefe_slice_ebene(LBM& L, const uint Nx, const uint Ny, const uint Nz, const uint y, const string& tag) {
	const ulong np = (ulong)Nx*(ulong)Nz;
	std::vector<float> su(np*3ull), sr(np); std::vector<uchar> sf(np);
	for(uint z=0u; z<Nz; z++) for(uint x=0u; x<Nx; x++) {
		const ulong g=(ulong)x+(ulong)z*(ulong)Nx, n=(ulong)x+((ulong)y+(ulong)z*(ulong)Ny)*(ulong)Nx;
		su[g*3ull]=L.u.x[n]; su[g*3ull+1ull]=L.u.y[n]; su[g*3ull+2ull]=L.u.z[n]; sr[g]=L.rho[n]; sf[g]=L.flags[n];
	}
	L.lese_yslice_in_host(y);
	float dmax=0.0f; ulong nfl=0ull, nnan=0ull; // nnan: NaN-Zustandswechsel -- fmax ist NaN-blind (Auditor-A NIEDRIG-4)
	for(uint z=0u; z<Nz; z++) for(uint x=0u; x<Nx; x++) {
		const ulong g=(ulong)x+(ulong)z*(ulong)Nx, n=(ulong)x+((ulong)y+(ulong)z*(ulong)Ny)*(ulong)Nx;
		dmax = fmax(dmax, fabs(su[g*3ull]-L.u.x[n])); dmax = fmax(dmax, fabs(su[g*3ull+1ull]-L.u.y[n]));
		dmax = fmax(dmax, fabs(su[g*3ull+2ull]-L.u.z[n])); dmax = fmax(dmax, fabs(sr[g]-L.rho[n]));
		nnan += (ulong)(std::isnan(su[g*3ull])!=std::isnan(L.u.x[n])) + (ulong)(std::isnan(su[g*3ull+1ull])!=std::isnan(L.u.y[n]))
		      + (ulong)(std::isnan(su[g*3ull+2ull])!=std::isnan(L.u.z[n])) + (ulong)(std::isnan(sr[g])!=std::isnan(L.rho[n]));
		if(sf[g]!=L.flags[n]) nfl++;
	}
	print_info("SLICE-PRUEF "+tag+" y="+to_string(y)+": max |Delta| u/rho = "+to_string(dmax,9u)
		+", flags-Differenzen "+to_string(nfl)+" von "+to_string(np)
		+(nnan>0ull?", NaN-WECHSEL "+to_string(nnan)+" (FEHLER!)":"")+" (Soll: 0 / 0).");
}


// --------------------------------------------------------------------------------------------
// VTK-FELDEXPORT fuer den Doppel-Domaenen-Fall (Heiko-Auftrag 2026-08-21).
// ANLASS: fuer f8_standard_final gab es KEINEN Feld-Dump -- der Vergleich nah gegen fern liess
// sich nur aus den Schnitt-PNGs rekonstruieren, und dort sind 27,8 % der Nahfeld-Ebene an der
// unteren Farbklemme (0,5*u_ref) = praktisch das ganze Totwasser. Genau die Region, um die es
// bei der Rueckkopplung geht, war damit nicht auswertbar.
//
// WARUM NICHT Memory_Container::write_vtk (lbm.hpp) -- zwei stille Fehler in diesem Fall:
//  1. Es rechnet mit dem GLOBALEN `units` (lbm.cpp Zeile 6). Der dd-Fall fuehrt aber ZWEI
//     Einheitensysteme (units_fine/units_coarse, setup.cpp), die sich in dx und dt um ratio
//     unterscheiden. Mindestens eine der beiden Domaenen bekaeme lautlos die falsche
//     Ortsschrittweite und den falschen SI-Faktor.
//  2. ORIGIN ist fest auf die MITTE der jeweiligen Box gesetzt (0,5-0,5*N). Nah- und Fernfeld
//     haben verschiedene Groessen und verschiedene Weltlagen; die beiden Dateien laegen im
//     Betrachter also NICHT uebereinander -- und ohne Deckung ist ein Differenzvergleich
//     wertlos. Hier wird die ECHTE Weltlage geschrieben (x0/y0/z0 der jeweiligen Domaene).
//
// Format: VTK legacy, BINARY, STRUCTURED_POINTS, Big-Endian (Formatvorgabe, daher
// reverse_bytes). Je Datei drei Felder unter einem POINT_DATA-Block:
//   VECTORS u float          Geschwindigkeit in m/s (SI, bereits umgerechnet)
//   SCALARS rho float        Dichte (Gitter-Einheiten, wie im Loeser)
//   SCALARS flags unsigned_char  Zelltypen -- 0x01 Solid, 0x02 Equilibrium/Kopplung, 0x03 MS ...
// stride>1 tastet jede stride-te Zelle ab (SPACING waechst entsprechend mit); die Weltlage des
// ersten Punktes bleibt exakt, damit auch abgetastete Dateien noch deckungsgleich liegen.
//
// Der Aufrufer muss u, rho und flags VORHER vom Geraet gelesen haben.
static ulong g_vtk_dateien = 0ull, g_vtk_bytes = 0ull; // Wirkpfad-Zaehler (Iron Rule: Nachweis im Binary)
// ★ Weltlage fuer das Guete-VTK (Heiko 29.08.). baue_facetten kennt nur Gitterindizes; damit die
// Punktwolke DECKUNGSGLEICH ueber den Feld-VTKs liegt, setzt das Setup diese vier Werte vorher.
// Default 0/0/0/1 = Gitterkoordinaten (wie remesh_flaeche.vtk), also nie stillschweigend falsch.
static float vtk_x0 = 0.0f, vtk_y0 = 0.0f, vtk_z0 = 0.0f, vtk_dx = 1.0f;
static void schreibe_vtk_feld(LBM& L, const uint Nx, const uint Ny, const uint Nz,
                              const float x0, const float y0, const float z0, const float dx,
                              const float u2si, const uint stride, const string& datei) {
	const uint Sx=(Nx+stride-1u)/stride, Sy=(Ny+stride-1u)/stride, Sz=(Nz+stride-1u)/stride;
	const ulong np = (ulong)Sx*(ulong)Sy*(ulong)Sz;
	std::ofstream f(datei, std::ios::out|std::ios::binary);
	if(!f) { print_warning("VTK: "+datei+" konnte nicht geoeffnet werden -- Dump uebersprungen."); return; }
	const float sp = dx*(float)stride;
	f << "# vtk DataFile Version 3.0\nFluidX3D-v2 dd-Feld " << datei.substr(datei.rfind('/')+1) << "\nBINARY\nDATASET STRUCTURED_POINTS\n"
	  << "DIMENSIONS " << Sx << " " << Sy << " " << Sz << "\n"
	  << "ORIGIN "  << to_string(x0,6u) << " " << to_string(y0,6u) << " " << to_string(z0,6u) << "\n"
	  << "SPACING " << to_string(sp,6u) << " " << to_string(sp,6u) << " " << to_string(sp,6u) << "\n"
	  << "POINT_DATA " << np << "\n";
	// ---- u als Vektorfeld in m/s
	f << "VECTORS u float\n";
	{
		std::vector<float> buf((size_t)Sx*3u);
		for(uint z=0u; z<Sz; z++) for(uint y=0u; y<Sy; y++) {
			for(uint x=0u; x<Sx; x++) {
				const ulong n = (ulong)(x*stride) + (ulong)Nx*((ulong)(y*stride) + (ulong)Ny*(ulong)(z*stride));
				buf[3u*x   ] = reverse_bytes(u2si*L.u.x[n]);
				buf[3u*x+1u] = reverse_bytes(u2si*L.u.y[n]);
				buf[3u*x+2u] = reverse_bytes(u2si*L.u.z[n]);
			}
			f.write((char*)buf.data(), (std::streamsize)(buf.size()*sizeof(float)));
		}
	}
	// ---- rho in Gitter-Einheiten (Druck folgt daraus ueber (rho-1)*cs2 und den SI-Druckfaktor)
	f << "\nSCALARS rho float 1\nLOOKUP_TABLE default\n";
	{
		std::vector<float> buf(Sx);
		for(uint z=0u; z<Sz; z++) for(uint y=0u; y<Sy; y++) {
			for(uint x=0u; x<Sx; x++) buf[x] = reverse_bytes(L.rho[(ulong)(x*stride) + (ulong)Nx*((ulong)(y*stride) + (ulong)Ny*(ulong)(z*stride))]);
			f.write((char*)buf.data(), (std::streamsize)(buf.size()*sizeof(float)));
		}
	}
	// ---- flags (uchar, kein Byte-Tausch noetig)
	f << "\nSCALARS flags unsigned_char 1\nLOOKUP_TABLE default\n";
	{
		std::vector<uchar> buf(Sx);
		for(uint z=0u; z<Sz; z++) for(uint y=0u; y<Sy; y++) {
			for(uint x=0u; x<Sx; x++) buf[x] = L.flags[(ulong)(x*stride) + (ulong)Nx*((ulong)(y*stride) + (ulong)Ny*(ulong)(z*stride))];
			f.write((char*)buf.data(), (std::streamsize)buf.size());
		}
	}
	f.close();
	const ulong bytes = np*(3ull*4ull+4ull+1ull);
	g_vtk_dateien++; g_vtk_bytes += bytes;
	print_info("[VTK] "+datei+": "+to_string(Sx)+"x"+to_string(Sy)+"x"+to_string(Sz)+" = "+to_string(np)+" Punkte, "
		+to_string((float)bytes/1048576.0f,1u)+" MB, Ursprung ("+to_string(x0,3u)+", "+to_string(y0,3u)+", "+to_string(z0,3u)
		+") m, Schrittweite "+to_string(sp*1000.0f,1u)+" mm"+(stride>1u?" (Abtastung 1:"+to_string(stride)+")":""));
}

// SAT-Schale plus Void-Fill. Upstreams Ray-Paritaets-Voxelizer laesst achsparallele Ebenen loechrig
// (an der Kugel: 72 statt ~1064 Solidzellen in der Symmetrieebene), und die SAT-Schale allein ergaebe
// einen HOHLKOERPER mit durchstroemtem Inneren -- gemessen 224 statt ~1100 Zellen im Mittelschnitt.
// Beides zusammen ergibt die dichte Geometrie. Von beiden Faellen benutzt.
static void sat_shell_and_void_fill(LBM& lbm, Mesh* mesh, const uint Nx, const uint Ny, const uint Nz) {
	const int bx0=(int)fmax(0.0f, floor(mesh->pmin.x)-2.0f), bx1=(int)fmin((float)Nx-1.0f, ceil(mesh->pmax.x)+2.0f);
	const int by0=(int)fmax(0.0f, floor(mesh->pmin.y)-2.0f), by1=(int)fmin((float)Ny-1.0f, ceil(mesh->pmax.y)+2.0f);
	const int bz0=(int)fmax(0.0f, floor(mesh->pmin.z)-2.0f), bz1=(int)fmin((float)Nz-1.0f, ceil(mesh->pmax.z)+2.0f);
	auto idx = [&](const int x, const int y, const int z) { return (ulong)x + ((ulong)y + (ulong)z*(ulong)Ny)*(ulong)Nx; };
	ulong added = 0ull;
	for(uint tri=0u; tri<mesh->triangle_number; tri++) {
		const float3 &V0=mesh->p0[tri], &V1=mesh->p1[tri], &V2=mesh->p2[tri];
		const int tx0=(int)fmax((float)bx0, floor(fmin(fmin(V0.x,V1.x),V2.x))-1.0f), tx1=(int)fmin((float)bx1, ceil(fmax(fmax(V0.x,V1.x),V2.x))+1.0f);
		const int ty0=(int)fmax((float)by0, floor(fmin(fmin(V0.y,V1.y),V2.y))-1.0f), ty1=(int)fmin((float)by1, ceil(fmax(fmax(V0.y,V1.y),V2.y))+1.0f);
		const int tz0=(int)fmax((float)bz0, floor(fmin(fmin(V0.z,V1.z),V2.z))-1.0f), tz1=(int)fmin((float)bz1, ceil(fmax(fmax(V0.z,V1.z),V2.z))+1.0f);
		for(int z=tz0; z<=tz1; z++) for(int y=ty0; y<=ty1; y++) for(int x=tx0; x<=tx1; x++) {
			const ulong n = idx(x,y,z);
			if((lbm.flags[n]&TYPE_S)!=0u) continue;
			if(sat_tri_box_overlap(V0, V1, V2, (float)x, (float)y, (float)z, 0.5f)) { lbm.flags[n] = TYPE_S|TYPE_X; added++; }
		}
	}
	print_info("SAT-Schale: "+to_string(added)+" Zellen ergaenzt");

	std::vector<bool> reach((size_t)((ulong)Nx*Ny*Nz), false);
	std::vector<ulong> stack;
	auto seed = [&](const int x, const int y, const int z) {
		const ulong n = idx(x,y,z);
		if((lbm.flags[n]&TYPE_X)==0u && !reach[(size_t)n]) { reach[(size_t)n]=true; stack.push_back(n); }
	};
	for(int z=bz0; z<=bz1; z++) for(int y=by0; y<=by1; y++) { seed(bx0,y,z); seed(bx1,y,z); }
	for(int z=bz0; z<=bz1; z++) for(int x=bx0; x<=bx1; x++) { seed(x,by0,z); seed(x,by1,z); }
	for(int y=by0; y<=by1; y++) for(int x=bx0; x<=bx1; x++) { seed(x,y,bz0); seed(x,y,bz1); }
	while(!stack.empty()) {
		const ulong n = stack.back(); stack.pop_back();
		const int z=(int)(n/((ulong)Nx*Ny)), y=(int)((n/(ulong)Nx)%(ulong)Ny), x=(int)(n%(ulong)Nx);
		const int nb[6][3] = {{x-1,y,z},{x+1,y,z},{x,y-1,z},{x,y+1,z},{x,y,z-1},{x,y,z+1}};
		for(int k=0; k<6; k++) {
			const int xx=nb[k][0], yy=nb[k][1], zz=nb[k][2];
			if(xx<bx0||xx>bx1||yy<by0||yy>by1||zz<bz0||zz>bz1) continue;
			const ulong nn = idx(xx,yy,zz);
			if((lbm.flags[nn]&TYPE_X)==0u && !reach[(size_t)nn]) { reach[(size_t)nn]=true; stack.push_back(nn); }
		}
	}
	// ★ 2026-08-22 (Heiko: alle drei Glaettungsstufen sehen am Radhaus gleich aus -- also war
	// es schon im VOXELFELD zu, nicht erst in der Flaeche). Die Flutung oben laeuft ueber SECHS
	// Nachbarn. Der LBM stroemt aber ueber 18 Richtungen: ein Hohlraum, der nur ueber eine
	// Diagonale nach aussen offen ist, ist fuer die Stroemung erreichbar, fuer diese Flutung
	// nicht -- und wird zugemacht. Zweite Flutung mit der Gitterkonnektivitaet als DIAGNOSE
	// (kein Eingriff), damit die Zahl auf dem Tisch liegt, bevor jemand daran dreht.
	const int c18[18][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},
	                      {1,1,0},{-1,-1,0},{1,0,1},{-1,0,-1},{0,1,1},{0,-1,-1},
	                      {1,-1,0},{-1,1,0},{1,0,-1},{-1,0,1},{0,1,-1},{0,-1,1}};
	std::vector<bool> reach18((size_t)((ulong)Nx*Ny*Nz), false);
	{
		std::vector<ulong> st2;
		auto seed2 = [&](const int x, const int y, const int z) {
			const ulong n = idx(x,y,z);
			if((lbm.flags[n]&TYPE_X)==0u && !reach18[(size_t)n]) { reach18[(size_t)n]=true; st2.push_back(n); }
		};
		for(int z=bz0; z<=bz1; z++) for(int y=by0; y<=by1; y++) { seed2(bx0,y,z); seed2(bx1,y,z); }
		for(int z=bz0; z<=bz1; z++) for(int x=bx0; x<=bx1; x++) { seed2(x,by0,z); seed2(x,by1,z); }
		for(int y=by0; y<=by1; y++) for(int x=bx0; x<=bx1; x++) { seed2(x,y,bz0); seed2(x,y,bz1); }
		while(!st2.empty()) {
			const ulong n = st2.back(); st2.pop_back();
			const int z=(int)(n/((ulong)Nx*Ny)), y=(int)((n/(ulong)Nx)%(ulong)Ny), x=(int)(n%(ulong)Nx);
			for(int k=0; k<18; k++) {
				const int xx=x+c18[k][0], yy=y+c18[k][1], zz=z+c18[k][2];
				if(xx<bx0||xx>bx1||yy<by0||yy>by1||zz<bz0||zz>bz1) continue;
				const ulong nn = idx(xx,yy,zz);
				if((lbm.flags[nn]&TYPE_X)==0u && !reach18[(size_t)nn]) { reach18[(size_t)nn]=true; st2.push_back(nn); }
			}
		}
	}
	const bool konn18 = env_u("CFD_VOIDFILL_KONN", 6u)>=18u; // Default 6 = bitidentisch
	ulong filled = 0ull, nur18 = 0ull;
	int dx0=1<<30, dx1=-1, dy0=1<<30, dy1=-1, dz0=1<<30, dz1=-1;
	for(int z=bz0; z<=bz1; z++) for(int y=by0; y<=by1; y++) for(int x=bx0; x<=bx1; x++) {
		const ulong n = idx(x,y,z);
		if((lbm.flags[n]&TYPE_X)!=0u) continue;
		const bool r6=reach[(size_t)n], r18=reach18[(size_t)n];
		if(!r6&&r18) { // nur ueber eine Diagonale erreichbar -- 6er-Flutung wuerde zumachen
			nur18++;
			if(x<dx0)dx0=x; if(x>dx1)dx1=x; if(y<dy0)dy0=y; if(y>dy1)dy1=y; if(z<dz0)dz0=z; if(z>dz1)dz1=z;
		}
		const bool zu = konn18 ? !r18 : !r6;
		if(zu) { lbm.flags[n] = TYPE_S|TYPE_X; filled++; }
	}
	print_info("Void-Fill: "+to_string(filled)+" eingeschlossene Zellen als solid markiert (Konnektivitaet "+string(konn18?"18 = Gitter":"6")+").");
	if(nur18>0ull) print_warning("Void-Fill DIAGNOSE: "+to_string(nur18)+" Zellen sind NUR ueber eine Diagonale von aussen erreichbar -- die 6er-Flutung macht sie zu, die Stroemung koennte hinein. Bereich X["+to_string(dx0)+","+to_string(dx1)+"] Y["+to_string(dy0)+","+to_string(dy1)+"] Z["+to_string(dz0)+","+to_string(dz1)+"]. Schalter CFD_VOIDFILL_KONN=18.");
	else print_info("Void-Fill DIAGNOSE: 6er- und 18er-Flutung erreichen dieselben Zellen -- die Konnektivitaet ist hier kein Thema.");

	// ---------------------------------------------------------------- Hohlraum-Ueberwachung
	// ★★ DER OFFENE INNENRAUM IST GEWOLLT. NICHT ZUSCHMIEREN. ★★
	// Heiko 2026-08-08: das ist der mitsimulierte MOTORRAUM. Die Kuehlung ist beim MR2 der schwierige
	// Teil und muss mitgerechnet werden -- die Durchstroemung des Motorraums gehoert zur Aufgabe, nicht
	// zu den Fehlern. Wer den Void-Fill so lange "verbessert", bis der Wagen massiv ist, entfernt genau
	// die Physik, um die es geht. Der Void-Fill ist damit richtig, wie er ist: er schliesst nur, was von
	// aussen NICHT erreichbar ist, und laesst die durchstroemten Raeume offen.
	//
	// Diese Zaehlung bleibt trotzdem -- als UEBERWACHUNG, nicht als Fehlersuche. Sie sagt, wie gross der
	// offene Innenraum ist, und macht sichtbar, wenn er sich unbemerkt aendert: weil jemand am Voxelizer
	// dreht, die Aufloesung wechselt oder die STL getauscht wird. Gemessen am 2026-08-08:
	//   fein  (4 mm): 11 900 828 Zellen = 0,762 m3 = 19,1 % des Solidvolumens (3,99 m3)
	//   grob (16 mm):    162 772 Zellen = 0,667 m3 = 15,2 %
	// Zwei Aufloesungen, dasselbe Volumen -- es ist Geometrie, keine Diskretisierung.
	//
	// Kriterium: eine Nicht-Solidzelle gilt als innenliegend, wenn sie in ALLEN DREI Achsen zwischen
	// Solid eingeschlossen ist (links und rechts, vorn und hinten, oben und unten je ein Treffer).
	// Der Spalt unter dem Wagen faellt dadurch heraus (in z nicht eingeschlossen); die Radhaeuser
	// fallen NICHT heraus und sind mitgezaehlt, obwohl sie Aussenraum sind.
	{
		const ulong nbx = (ulong)(bx1-bx0+1), nby = (ulong)(by1-by0+1), nbz = (ulong)(bz1-bz0+1);
		std::vector<uchar> enc((size_t)(nbx*nby*nbz), 0u); // Bit 0/1/2 = in x/y/z eingeschlossen
		auto bidx = [&](const int x, const int y, const int z) { return (ulong)(x-bx0) + nbx*((ulong)(y-by0) + nby*(ulong)(z-bz0)); };
		auto is_solid = [&](const int x, const int y, const int z) { return (lbm.flags[idx(x,y,z)]&TYPE_S)!=0u; };
		for(int z=bz0; z<=bz1; z++) for(int y=by0; y<=by1; y++) { // x-Achse
			int first=-1, last=-1;
			for(int x=bx0; x<=bx1; x++) if(is_solid(x,y,z)) { if(first<0) first=x; last=x; }
			if(first>=0) for(int x=first; x<=last; x++) if(!is_solid(x,y,z)) enc[(size_t)bidx(x,y,z)] |= 1u;
		}
		for(int z=bz0; z<=bz1; z++) for(int x=bx0; x<=bx1; x++) { // y-Achse
			int first=-1, last=-1;
			for(int y=by0; y<=by1; y++) if(is_solid(x,y,z)) { if(first<0) first=y; last=y; }
			if(first>=0) for(int y=first; y<=last; y++) if(!is_solid(x,y,z)) enc[(size_t)bidx(x,y,z)] |= 2u;
		}
		for(int y=by0; y<=by1; y++) for(int x=bx0; x<=bx1; x++) { // z-Achse
			int first=-1, last=-1;
			for(int z=bz0; z<=bz1; z++) if(is_solid(x,y,z)) { if(first<0) first=z; last=z; }
			if(first>=0) for(int z=first; z<=last; z++) if(!is_solid(x,y,z)) enc[(size_t)bidx(x,y,z)] |= 4u;
		}
		ulong hollow = 0ull; int hx0=1<<30, hx1=-1, hy0=1<<30, hy1=-1, hz0=1<<30, hz1=-1;
		for(int z=bz0; z<=bz1; z++) for(int y=by0; y<=by1; y++) for(int x=bx0; x<=bx1; x++) {
			if(enc[(size_t)bidx(x,y,z)]==7u) {
				hollow++;
				hx0=min(hx0,x); hx1=max(hx1,x); hy0=min(hy0,y); hy1=max(hy1,y); hz0=min(hz0,z); hz1=max(hz1,z);
			}
		}
		ulong solid_cells = 0ull;
		for(int z=bz0; z<=bz1; z++) for(int y=by0; y<=by1; y++) for(int x=bx0; x<=bx1; x++) if(is_solid(x,y,z)) solid_cells++;
		if(hollow>0ull) {
			print_warning("Hohlraum im Koerper: "+to_string(hollow)+" Fluidzellen liegen in allen drei Achsen zwischen Solid ("
				+to_string(100.0f*(float)hollow/(float)max(1ull,solid_cells),1u)+" % des Solidvolumens), Bereich X["+to_string(hx0)+","+to_string(hx1)
				+"] Y["+to_string(hy0)+","+to_string(hy1)+"] Z["+to_string(hz0)+","+to_string(hz1)+"]. Die Schale ist dort nach aussen offen --"
				" der Void-Fill kann nur fuellen, was er nicht erreicht.");
		} else print_info("Hohlraum-Pruefung: keine innenliegenden Fluidzellen, der Koerper ist massiv.");
	}
	// Kein write_to_device() noetig: LBM::initialize() laedt rho, u und flags beim ersten run() hoch.
}

static // ---------------------------------------------------------------------------- Dichte-Klemme berichten
// ★★ Heikos Einwand 2026-08-09: "rho clamp ist doch auch nur ne Kruecke die man benoetigt wenn der
// Code falsch ist oder etwas falsch parametrisiert ist." Richtig -- in einem korrekten Low-Mach-LBM
// liegt rho bei 1 +- 0,02. Deshalb ist der Zaehler wichtiger als die Klemme selbst: er sagt, ob ein
// Lauf ueberhaupt ein Ergebnis ist. Bleibt er null, war die Klemme ein nie ausloesender Waechter.
// Ist er gross, rechnete der Lauf stellenweise auf einem geklemmten, also verfaelschten Feld.
void berichte_dichteklemme(LBM& L, const char* wo, ulong& summe) {
	// ★ Re-Audit R2 (Rest von Befund 2): SGS_WANDFREI bekommt seinen Wirkpfad-Nachweis -- Slot 6,
	// im Kernel gegatet t%100. Null Treffer bei gesetztem Schalter = lautloser No-Op = harter Fehler.
	if(LBM_Domain::s_sgs_wandfrei) {
		ulong wz=0ull;
		for(uint d=0u; d<L.get_D(); d++) { L.lbm_domain[d]->rho_clamp_hits.read_from_device(); wz+=(ulong)L.lbm_domain[d]->rho_clamp_hits[6]; }
		print_info(string("  SGS_WANDFREI ")+wo+": "+to_string(wz)+" gezaehlte Gate-Zellen (solider 18er-Nachbar; t%100)");
		if(wz==0ull) print_error(string("CFD_SGS_WANDFREI war gesetzt, aber der Wirkpfad-Zaehler ist NULL (")+wo+") -- lautloser No-Op.");
	}
	// ★ P0-DIAGNOSTIK (2026-08-23): nu_t/nu_0-Dekadenhistogramm, Slots 28..32.
	// JE DOMAENE getrennt -- die Lehre aus der Dichteklemme, wo die Summe ueber beide Gitter
	// einen Faktor 7,5 im Fernfeld maskiert hat (5,4 Mio nah gegen 92.144 fern). Diese
	// Funktion bekommt genau ein LBM-Objekt und ein Ortslabel, die Trennung ist also gegeben.
	if(LBM_Domain::s_sgs_diag) {
		ulong h[5]={0ull,0ull,0ull,0ull,0ull}, ges=0ull;
		for(uint d=0u; d<L.get_D(); d++) {
			L.lbm_domain[d]->rho_clamp_hits.read_from_device();
			for(uint b=0u; b<5u; b++) h[b]+=(ulong)L.lbm_domain[d]->rho_clamp_hits[30u+b];
		}
		for(uint b=0u; b<5u; b++) ges+=h[b];
		if(ges>0ull) {
			const double p100=100.0/(double)ges;
			print_info(string("  nu_t/nu_0 ")+wo+" (Dekaden, jede 64. Zelle per Hash, t%100, ab Warmlaufende, ohne TYPE_E): <1 "+to_string(h[0])+" ("+to_string((float)((double)h[0]*p100),1u)+" %), 1-10 "+to_string(h[1])+" ("+to_string((float)((double)h[1]*p100),1u)+" %), 10-100 "+to_string(h[2])+" ("+to_string((float)((double)h[2]*p100),1u)+" %), 100-1000 "+to_string(h[3])+" ("+to_string((float)((double)h[3]*p100),1u)+" %), >=1000 "+to_string(h[4])+" ("+to_string((float)((double)h[4]*p100),1u)+" %)");
			const double hoch=(double)(h[2]+h[3]+h[4])*p100;
			if(hoch>50.0) print_warning(string("  nu_t/nu_0 ")+wo+": "+to_string((float)hoch,1u)+" % der Stichproben liegen ueber dem Zehnfachen der molekularen Viskositaet -- die Loesung ist dort SGS-dominiert, nicht molekular.");
			// ★ Pruefbefund 10: ohne Ist!=Soll-Test wickelt der uint-Zaehler still und liefert
			// plausibel aussehende falsche Prozente. Bei 4 mm waere das nach 12,3 ms passiert.
			// Die n&7-Stichprobe und das Emissionsgate entschaerfen es; der Waechter bleibt.
			// Wandnaechste Lage getrennt -- das ist die Zahl, die zwischen den beiden Lesarten
			// entscheidet (zu viel gegen zu wenig wandnahe Mischung).
			ulong hw[5]={0ull,0ull,0ull,0ull,0ull}, gw=0ull;
			for(uint d=0u; d<L.get_D(); d++) for(uint b=0u; b<5u; b++) hw[b]+=(ulong)L.lbm_domain[d]->rho_clamp_hits[35u+b];
			for(uint b=0u; b<5u; b++) gw+=hw[b];
			if(gw>0ull) {
				const double q=100.0/(double)gw;
				/* MASSSTAB, korrigiert nach Pruefbefund 4 (2026-08-23): NICHT kappa*y+, sondern
				   0,714*kappa*y+. Setzt man die Gleichgewichtsscherung |S| = u_tau/(kappa*y1) in
				   FluidX3Ds eigene Konstante ein, liefert Smagorinsky (C*Delta)^2/(kappa^2*y1^2)
				   = 0,030021/(0,41^2*0,25) = 0,714 der noetigen Wandschicht-Viskositaet -- und
				   zwar AUFLOESUNGSUNABHAENGIG. Bei y+ = 70,7 (Facettenmedian Nahfeld 8 mm) sind
				   das rund 21, nicht 29. Die Bingrenze 15-30 ist damit die Gleichgewichtszone.
				   ACHTUNG: diese Eichung gilt NUR fuer das Nahfeld bei 8 mm. Das Fernfeld hat ein
				   rund viermal hoeheres y+, dort liegt die Erwartung bei rund 85 -- die Bins sind
				   dort NICHT aussagekraeftig und werden nur nachrichtlich gedruckt. */
				print_info(string("  nu_t/nu_0 ")+wo+" WANDNAECHSTE LAGE (Erwartung = 0,714*kappa*y+ mit dem GEMESSENEN u_tau dieses Laufs -- NICHT mit dem Sollwert; Bins 5/15/30/60): <5 "+to_string(hw[0])+" ("+to_string((float)((double)hw[0]*q),1u)+" %), 5-15 "+to_string(hw[1])+" ("+to_string((float)((double)hw[1]*q),1u)+" %), 15-30 "+to_string(hw[2])+" ("+to_string((float)((double)hw[2]*q),1u)+" %), 30-60 "+to_string(hw[3])+" ("+to_string((float)((double)hw[3]*q),1u)+" %), >=60 "+to_string(hw[4])+" ("+to_string((float)((double)hw[4]*q),1u)+" %)");
				/* KEINE Lesart-Warnung mehr. Eine einzelne Aufloesung entscheidet die Frage
				   "zu viel oder zu wenig modellierte Mischung" nicht: Modeled-Stress Depletion ist
				   als Delta-ABHAENGIGKEIT definiert (feiner = weniger Mischung = fruehere
				   Abloesung) und braucht die Reihe 16/8/4 mm. Und ein Ueberschuss gegen das
				   Gleichgewicht sagt nicht, ob die Konstante zu gross oder die Scherung |S|
				   spurios ist -- an Voxeltreppen ist |S| bauartbedingt ueberhoeht. Die frueheren
				   beiden Warnungen an dieser Stelle haben genau diesen unzulaessigen Schluss
				   gezogen und sind entfernt (Pruefbefunde 5 und 6, 2026-08-23). */
			}
			ulong hv[5]={0ull,0ull,0ull,0ull,0ull}, gv=0ull;
			for(uint d=0u; d<L.get_D(); d++) for(uint b=0u; b<5u; b++) hv[b]+=(ulong)L.lbm_domain[d]->rho_clamp_hits[40u+b];
			for(uint b=0u; b<5u; b++) gv+=hv[b];
			if(gv>0ull) {
				const double q2=100.0/(double)gv;
				print_info(string("  nu_t/nu_0 ")+wo+" WANDNAH und ANLIEGEND (u_x > 0, "+to_string((float)(100.0*(double)gv/(double)max(1ull,gw)),1u)+" % der Lage): <5 "+to_string((float)((double)hv[0]*q2),1u)+" %, 5-15 "+to_string((float)((double)hv[1]*q2),1u)+" %, 15-30 "+to_string((float)((double)hv[2]*q2),1u)+" %, 30-60 "+to_string((float)((double)hv[3]*q2),1u)+" %, >=60 "+to_string((float)((double)hv[4]*q2),1u)+" %");
			}
			// Oberen Schwanz aufloesen (Slots 43..46)
			{
				ulong ht[4]={0ull,0ull,0ull,0ull}, gt=0ull;
				for(uint d=0u; d<L.get_D(); d++) for(uint b=0u; b<4u; b++) ht[b]+=(ulong)L.lbm_domain[d]->rho_clamp_hits[45u+b];
				for(uint b=0u; b<4u; b++) gt+=ht[b];
				if(gt>0ull) { const double q3=100.0/(double)gt;
					print_info(string("  nu_t/nu_0 ")+wo+" oberer Schwanz (Anteil der Zellen mit rv>=60): 60-120 "+to_string((float)((double)ht[0]*q3),1u)+" %, 120-240 "+to_string((float)((double)ht[1]*q3),1u)+" %, 240-480 "+to_string((float)((double)ht[2]*q3),1u)+" %, >=480 "+to_string((float)((double)ht[3]*q3),1u)+" %");
					ulong mt=0ull; for(uint b=0u; b<4u; b++) if(ht[b]>mt) mt=ht[b];
					if(mt>3865470566ull) print_error(string("  nu_t/nu_0 ")+wo+" oberer Schwanz: Wickelgefahr (Bin ueber 90 % des uint-Bereichs).");
				}
			}
			/* WICKELWAECHTER JE DOMAENE (Pruefbefund 13): ueber die Summe zu pruefen ist blind --
			   ein Zaehler, der in EINER Domaene bereits gewickelt IST, steht danach niedrig und
			   faellt nie auf. Die Rohwerte je Domaene sind uint, also wird jeder einzeln geprueft. */
			ulong maxb=0ull;
			for(uint d=0u; d<L.get_D(); d++) for(uint sl=30u; sl<49u; sl++) { // ★ Pruefbefund 3-D: war 28 -- seit der Verschiebung fielen Geschwindigkeitsklemme (28) und SPONGE (29) unter eine nu_t-Meldung
				const ulong v=(ulong)L.lbm_domain[d]->rho_clamp_hits[sl];
				if(v>maxb) maxb=v;
			}
			if(maxb>3865470566ull) print_error(string("  nu_t/nu_0 ")+wo+": ein Bin steht bei "+to_string(maxb)+" und damit ueber 90 % des uint-Bereichs -- WICKELGEFAHR, die Prozente sind nicht mehr belastbar. Lauf kuerzen oder Stichprobe ausduennen.");
		} else print_error(string("  nu_t/nu_0 ")+wo+": Histogramm LEER, obwohl CFD_SGS_DIAG gesetzt ist -- lautloser No-Op. Seit 03.09. steht der DIAG-Block hinter dem FDWAND-if/else und zaehlt auch WANDFREI-Zellen (rv=0, Bin <5) mit -- ein leeres Histogramm heisst jetzt: Emission (CFD_SGS_DIAG -> #define SGS_DIAG) oder Warmlaufsperre def_sgs_diag_ab pruefen");
	}
	{	// ★ Pruefbefund 3-B/3-A (2026-08-25): WICKELWAECHTER fuer ALLE uebrigen Slots, unabhaengig
		// von CFD_SGS_DIAG. Gezaehlt wird je Facetten- bzw. Zellbesuch auf jedem 100. Schritt; bei
		// grossen Gittern wickelt uint. Ein gewickelter Bin sieht klein und harmlos aus -- genau die
		// Falle, an der die RHO_CLAMP-Zahlen tagelang hingen.
		ulong mx=0ull; uint mxs=0u; ulong mx7=0ull;
		for(uint d=0u; d<L.get_D(); d++) { L.lbm_domain[d]->rho_clamp_hits.read_from_device();
			for(uint sl=0u; sl<(uint)L.lbm_domain[d]->rho_clamp_hits.length(); sl++) { const ulong v=(ulong)L.lbm_domain[d]->rho_clamp_hits[sl];
				// ★ 04.09.2026 (Host-Audit H1): die Schleife stand auf 68 und war seit dem 27.08. blind fuer
				// alles darueber -- 68 (MLS), 69 (Rueckfall), 70/71 (Kraft), 72-79 (NACHBAR/MESSNUR/FDWAND/
				// F-Liste/Vollrang), 80 (Pseudoinverse). Jetzt bis zur Pufferlaenge.
				// SLOT 7 IST DIE AUSNAHME: er wickelt am Fahrzeugmassstab KONSTRUKTIV und wird deshalb bei
				// der Wirkpfad-Abnahme bewusst mod 2^32 gegen sein Soll geprueft. Ihn in dieselbe Toepfe zu
				// werfen hiesse, einen eingeplanten Wickel per exit(1) zu bestrafen -- gemessen: 4 mm,
				// 0,301 s Physik -> Slot 7 = 941.884.685 (logs/zh_pinv4.log), die exit(1)-Schwelle liegt
				// beim 4,1-fachen, also bei rund 1,24 s. Mit der Laufverlaengerung vom selben Tag ist das
				// erreichbar, und der Abbruch faellt VOR die gesamte Endauswertung.
				if(sl==7u) { if(v>mx7) mx7=v; continue; }
				if(v>mx) { mx=v; mxs=sl; } } }
		// ★ g12-Befund (2026-08-25 nacht): SAETTIGUNG ist der GEWOLLTE Endzustand der saettigenden
		// Zaehler (Parken ab 0xF0000000) -- der Waechter hat sie als "Wickelgefahr" gemeldet und
		// per exit(1) drei komplette 8-mm-Laeufe am BERICHT getoetet (die Physik-CSVs ueberlebten).
		// Jetzt: >= 0xF0000000 = gesaettigt (Info, Zahl ist eine UNTERGRENZE); nur der Bereich
		// dazwischen ist echte Wickelgefahr nicht-saettigender Zaehler.
		if(mx>=4026531840ull) print_info(string("  Zaehler ")+wo+": Slot "+to_string(mxs)+" GESAETTIGT (>= 0xF0000000) -- Zahl ist eine Untergrenze, kein Wickel.");
		else if(mx>3865470566ull) print_error(string("  Zaehler ")+wo+": Slot "+to_string(mxs)+" steht bei "+to_string(mx)+", ueber 90 % des uint-Bereichs -- WICKELGEFAHR, alle daraus gerechneten Prozente sind wertlos.");
		else if(mx>2147483648ull) print_warning(string("  Zaehler ")+wo+": Slot "+to_string(mxs)+" steht bei "+to_string(mx)+" (ueber die Haelfte des uint-Bereichs) -- bei laengerem Lauf wickelt er.");
		// Slot 7 gesondert: WARNUNG statt exit(1), aber mit der Konsequenz benannt -- er ist der NENNER
		// der Solver-Kaskade und der Rueckfallquote. Wickelt er, sind nicht die Physik-CSVs falsch,
		// sondern jeder Prozentsatz, der ihn als Grundgesamtheit benutzt.
		if(mx7>=4294967296ull||mx7>3865470566ull) print_warning(string("  Zaehler ")+wo+": WIRKPFAD-Slot 7 steht bei "+to_string(mx7)
			+" und damit nahe am uint-Wickel (2^32 = 4294967296). Die Wirkpfad-ABNAHME haelt (sie vergleicht mod 2^32), aber ALLE ANTEILE mit Slot 7 als Nenner"
			+" -- Solver-Kaskade, Rueckfallquote, Modellabdeckung -- sind ab dem Wickel wertlos. Kein Abbruch: die Physik und die CSVs sind unberuehrt.");
	}
	// ★ 2026-08-25: Ansage der beiden neuen Wirkpfad-Zaehler. Beide gegatet (t%100), also
	// Stichproben, keine Ereigniszahlen.
	{
		ulong vk=0ull, sp=0ull;
		for(uint d=0u; d<L.get_D(); d++) { L.lbm_domain[d]->rho_clamp_hits.read_from_device();
			vk+=(ulong)L.lbm_domain[d]->rho_clamp_hits[28]; sp+=(ulong)L.lbm_domain[d]->rho_clamp_hits[29]; }
		if(vk>0ull) print_warning(string("  GESCHWINDIGKEITSKLEMME ")+wo+": "+to_string(vk)+" Treffer (saettigend). Wo sie greift, ist der Impuls NICHT erhalten -- f_eq traegt rho*u_geklemmt statt j+F/2.");
		else print_info(string("  Geschwindigkeitsklemme ")+wo+": 0 Treffer (Impuls ungestoert).");
		{ // ★ Pruefbefund A4: Slot 59 wurde NIRGENDS gelesen -- ein reiner Schreibzaehler.
			ulong bw=0ull; for(uint d=0u; d<L.get_D(); d++) bw+=(ulong)L.lbm_domain[d]->rho_clamp_hits[59];
			if(bw>0ull) print_info(string("  Bewegtwand-Term ")+wo+": "+to_string(bw)+" Randzellen-Auswertungen mit u_w != 0 (saettigend).");
			else print_info(string("  Bewegtwand-Term ")+wo+": 0 -- keine mitbewegte Wand in dieser Domaene.");
		}
		{ // ★ 2026-08-25 Guo-Korrektur: Wirkpfad UND Groesse (Slots 60..63)
			ulong g[4]={0ull,0ull,0ull,0ull}, gs=0ull;
			for(uint d=0u; d<L.get_D(); d++) for(uint k=0u; k<4u; k++) { const ulong v=(ulong)L.lbm_domain[d]->rho_clamp_hits[60u+k]; g[k]+=v; gs+=v; }
			if(gs>0ull) print_info(string("  Guo-Korrektur ")+wo+": relative Aenderung von |Pi^neq| -- <0,1%: "+to_string(100.0*(double)g[0]/(double)gs,1u)+"%, <1%: "+to_string(100.0*(double)g[1]/(double)gs,1u)+"%, <10%: "+to_string(100.0*(double)g[2]/(double)gs,1u)+"%, >=10%: "+to_string(100.0*(double)g[3]/(double)gs,1u)+"%");
			else if(LBM_Domain::s_sgs_guo) print_warning(string("  Guo-Korrektur ")+wo+": Zaehler NULL -- entweder kein SUBGRID/VOLUME_FORCE in dieser Domaene, oder lautloser No-Op.");
		}
		// ★ Pruefbefund X-1 (2026-08-25, HOCH): s_sponge_n ist eine STATIK. Im dd-Fall traegt sie beim
		// Bericht den FERNFELD-Wert, waehrend hier das NAHFELD geprueft wird -- das Nahfeld hat gar
		// keine Zone, sein Zaehler ist zu Recht null. Der Waechter haette also im Normalfall der
		// dd-Produktion (CFD_SPONGE_N>0) zugeschlagen, und print_error ruft exit(1): der Lauf waere
		// VOR der Cd/Cz-Ausgabe gestorben. Jetzt Ansage statt Abbruch, und die Statik wird als das
		// benannt, was sie ist. Ein echter Zonen-Waechter braucht ein Domaenen-Member (Folgearbeit).
		ulong spk=0ull; for(uint d=0u; d<L.get_D(); d++) spk+=(ulong)L.lbm_domain[d]->rho_clamp_hits[66];
		if(sp>0ull) print_info(string("  SPONGE ")+wo+": "+to_string(sp)+" Zonenbesuche, davon "+to_string(spk)+" mit greifender wmin-Klemme ("+to_string(sp>0ull?100.0*(double)spk/(double)sp:0.0,1u)+" %). Beide Zaehler saettigend, getrennte Slots.");
		else if(LBM_Domain::s_sponge_n>0u) print_warning(string("  SPONGE ")+wo+": Zaehler NULL, waehrend die (domaenenuebergreifende) Statik n="+to_string(LBM_Domain::s_sponge_n)+" meldet. Im dd-Fall ist das fuers Nahfeld ERWARTET (keine Zone); in einer Einzelgitter-Domaene waere es ein lautloser No-Op.");
	}
#ifdef RHO_CLAMP
	ulong u=0ull, o=0ull; L.rho_clamp_hits_total(u, o);
	summe += u+o;
	// ★ Pruefbefund X-3 (2026-08-25): seit der Gatung sind das STICHPROBEN auf t%100, keine
	// Ereigniszahlen. Null hier schliesst Klemmungen zwischen den Abtastpunkten NICHT aus -- der
	// Text darf also keine Unbedenklichkeitsbescheinigung mehr sein.
	print_info(string("  RHO_CLAMP ")+wo+": "+to_string(u+o)+" Treffer (untere Grenze "+to_string(u)+", obere "+to_string(o)+"); Zaehler SAETTIGT bei 4026531840 statt zu wickeln");
#else
	(void)L; (void)wo; (void)summe;
#endif // RHO_CLAMP
}
void dichteklemme_fazit(const ulong summe) {
#ifdef RHO_CLAMP
	if(summe==0ull) print_info("  NULL Treffer -- die Klemme hat nie gegriffen, das Feld ist physikalisch geblieben. (Der Zaehler ist saettigend, nicht gegatet: eine Null ist wieder eine echte Aussage.)");
	else print_warning("Die Dichte-Klemme hat "+to_string(summe)+" mal gegriffen: rho hat den physikalischen Bereich verlassen. Dieser Lauf rechnete stellenweise auf einem GEKLEMMTEN Feld und ist kein belastbares Ergebnis -- die Ursache liegt im Betriebspunkt (fehlende Volumenviskositaet bei w gegen 2), nicht in der Klemme.");
#else
	(void)summe;
#endif // RHO_CLAMP
}

// ---------------------------------------------------------------------------- Lauf-Sicherung
// ★★ Heiko 2026-08-09: MIT JEDEM LAUF eine vollstaendige Sicherung in den Export-Ordner.
// Zweck: Monate spaeter noch feststellen koennen, mit WELCHEM Stand eine Zahl entstanden ist --
// ohne Git-Archaeologie und ohne die Annahme, der Arbeitsbaum sei seither unveraendert.
//
// Drei Dinge waren an der Vorgaengerfassung falsch, alle behoben:
//  (1) Sie lief NUR im Doppel-Domaenen-Fall. Jetzt in allen vier.
//  (2) Sie zaehlte SECHS Dateien von Hand auf -- von neunzehn. kernel.hpp, utilities.hpp, units.hpp,
//      main.cpp, shapes.*, info.*, setup.hpp fehlten alle. Eine neu angelegte Quelldatei waere still
//      aus der Sicherung gefallen, und niemand haette es gemerkt. Jetzt wird das VERZEICHNIS
//      durchlaufen: was in src/ liegt, wird gesichert, ohne Liste, die veralten kann.
//  (3) Sie sicherte den Quelltext, aber nicht den ZUSTAND: kein Commit, kein Hinweis darauf, ob der
//      Baum schmutzig war, keine Umgebungsvariablen. Genau die entscheiden aber ueber das Ergebnis.
void sichere_lauf(const string& out_dir, const string& fall) {
	const string dst = out_dir+"code/";
	std::error_code ec; std::filesystem::create_directories(dst, ec);
	// --- alle Quelldateien, per Verzeichnisdurchlauf statt Handliste
	uint n_files = 0u; ulong n_bytes = 0ull;
	const string src_dir = get_exe_path()+"../src/";
	for(const auto& e : std::filesystem::directory_iterator(src_dir, ec)) {
		if(!e.is_regular_file()) continue;
		const string ext = e.path().extension().string();
		if(ext!=".cpp" && ext!=".hpp" && ext!=".h" && ext!=".c") continue;
		std::filesystem::copy_file(e.path(), dst+e.path().filename().string(),
			std::filesystem::copy_options::overwrite_existing, ec);
		if(!ec) { n_files++; n_bytes += (ulong)e.file_size(ec); }
	}
	for(const char* b : {"make.sh", "makefile"}) // Uebersetzungsvorschrift gehoert dazu
		std::filesystem::copy_file(get_exe_path()+"../"+b, dst+b, std::filesystem::copy_options::overwrite_existing, ec);
	// --- Zustandsbericht
	auto ausgabe_von = [](const string& cmd)->string {
		string r; FILE* f = popen(cmd.c_str(), "r"); if(!f) return "(nicht ermittelbar)";
		char buf[4096]; while(fgets(buf, sizeof(buf), f)) r += buf; pclose(f);
		while(!r.empty() && (r.back()=='\n'||r.back()=='\r')) r.pop_back();
		return r.empty() ? string("(leer)") : r;
	};
	const string repo = "cd '"+get_exe_path()+"..' && ";
	const string commit = ausgabe_von(repo+"git rev-parse HEAD 2>/dev/null");
	const string schmutz = ausgabe_von(repo+"git status --porcelain 2>/dev/null");
	std::ofstream m(dst+"LAUF.txt");
	m << "Lauf-Sicherung\n==============\n\n";
	m << "Fall            : " << fall << "\nOrdner          : " << out_dir << "\n";
	m << "Gesichert       : " << n_files << " Quelldateien, " << (n_bytes/1024ull) << " kB\n\n";
	m << "Git-Commit      : " << commit << "\n";
	m << "Arbeitsbaum     : " << (schmutz=="(leer)" ? "SAUBER -- der Commit oben beschreibt den Code vollstaendig"
		: "SCHMUTZIG -- der Commit allein reicht NICHT, siehe aenderungen.diff") << "\n";
	if(schmutz!="(leer)") m << "\nGeaenderte Dateien:\n" << schmutz << "\n";
	m << "\nUmgebung (alle CFD_*, sie entscheiden ueber das Ergebnis):\n";
	uint n_env = 0u;
	for(char** e = environ; *e; e++) if(strncmp(*e, "CFD_", 4)==0) { m << "  " << *e << "\n"; n_env++; }
	if(n_env==0u) m << "  (keine gesetzt -- alle Vorgabewerte)\n";
	m.close();
	// --- bei schmutzigem Baum den vollstaendigen Unterschied mitsichern; nur so ist der Lauf reproduzierbar
	if(schmutz!="(leer)") { string cmd = repo+"git diff HEAD > '"+dst+"aenderungen.diff' 2>/dev/null"; if(system(cmd.c_str())){} }
	print_info("Lauf-Sicherung: "+dst+" ("+to_string(n_files)+" Quelldateien, LAUF.txt mit Commit und Umgebung"
		+(schmutz=="(leer)" ? ", Baum sauber)" : ", Baum SCHMUTZIG -> aenderungen.diff)"));
}

// ---------------------------------------------------------------------------- Turbulenter Kanal
// ★★ A1 der Arbeitsliste, 2026-08-15: die ERSTE EXTERNE REFERENZ des Projekts. Referenz ist
// Lee & Moser 2015 (JFM 774, 395; turbulence.oden.utexas.edu/channel2015): bei Re_tau = 5186 ist
// U_b+ = 24,104 und c_f = 3,4424e-3 (aus deren Profildatei selbst integriert, 2026-08-09).
//
// AUFBAU: Wandnormale z (wie die Fahrbahn), x und y periodisch (FluidX3D ist von Haus aus in allen
// Achsen periodisch -- es wird schlicht KEIN Rand gesetzt), zwei ruhende Waende z=0 und z=Nz-1.
// Antrieb ueber die Volumenkraft fx -- der erste Fall, der VOLUME_FORCE wirklich benutzt.
//
// ★ FESTE DURCHFLUSSRATE (CFR), nicht feste Kraft: der Arm ohne Wandmodell hat U_b+ = 1,154*N
// (hergeleitet und gegengeprueft), bei fester Kraft laeuft er auf feinen Gittern in die
// Kompressibilitaet (Ma_Mitte = 1 schon ab N ~ 107). Bei CFR ist U_b beschraenkt und die
// BENOETIGTE Kraft ist die Messung: tau_w = f*delta exakt, c_f = 2*tau_w/U_b^2.
// Regler (Benocci/Pinelli): f += K*[(U_ziel-U_b) + (U_b_alt-U_b)], alle 100 Schritte.
//
// ★ MESSVORRICHTUNG VOR MESSUNG (A2): zwei unabhaengige tau_w -- Kraftbilanz f*delta gegen
// Impulsaustausch object_force/(2*Nx*Ny) -- muessen uebereinstimmen, und die Gesamtspannungsbilanz
// muss 1 - z/delta ergeben. Beides braucht KEINE DNS-Datei und disqualifiziert einen kaputten
// Aufbau, bevor irgendein Literaturvergleich Sinn ergibt.
//
// ★ RELAMINARISIERUNGS-WAECHTER: bei y+_1 = 137 ist der wandnahe Zyklus unaufgeloest; kippt die
// LES ins Laminare, geht U_b+ gegen Re_tau/3 (~1700) und man misst das statt des Modells.
// U_b+ > 200 => Abbruch mit Ansage (Gegenpruefer-Befund).
// ---------------------------------------------------------------------------- C1b Stufe 1: Facettenbau (reine Diagnose)
// FACETTEN-PLAN.md A1-A4 + Revision: TLS-Ausgleichsebene ueber Halfway-BB-Linkmittelpunkte der
// 3^3-Nachbarschaft (Default; CFD_FACETTEN_FENSTER) jeder wandnahen Fluidzelle. Host, double, kein Kernel-Eingriff, keine Flag-Bits.
// wand_flag: 0x41 am Fahrzeug (Fahrbahn/Latsch = Ausschluss), TYPE_S im Kanal-Ankerfall.
// struct Facette: seit Stufe 2 in lbm.hpp (alloc_facetten braucht den Typ)
// ======================================================================= ELIBB P1 (2026-08-22)
// REMESH DER VOXEL-AUSSENWAND (Heiko-Vorgabe, Arbeitsliste 11a): geschlossene Dreiecksflaeche
// ueber dem FINALEN flags-Feld (nicht der STL -- der effektive Solid traegt SAT-Schale,
// Void-Fill, Duennteile), dann q je Link fuer das kommende Facetten-ELIBB (Plan:
// FACETTEN-ELIBB-PLAN.md). P1 = Bau + DIAGNOSE (Histogramme, Abdeckung); der fac_q-Upload
// und der Kernel folgen in P2. Gate: CFD_FACETTEN_REMESH=1, Default aus = bitidentisch.
// Verfahren: NAIVE SURFACE NETS auf dem Binaerfeld -- je Grenzflaeche solid|fluid ein Quad
// auf dem dualen Eckgitter, garantiert geschlossen und mannigfaltig, keine MC-Tabellen.
// Glaettung: TAUBIN lambda|mu (0,5 / -0,53) mit harter Vertex-Klemme +-0,5 Zelle -- die
// Klemme begrenzt den q-Fehler konstruktiv und haelt die Flaeche nahe der effektiven
// Halfway-Wand (entschaerft den y_w-Eichkonflikt, Arbeitsliste 9).
struct RemeshQ {
	std::vector<float> vx, vy, vz;        // Vertices (geglaettet), Gitterkoordinaten (Eckpunkte)
	std::vector<uint>  tri;               // Dreiecke, 3 Indizes je Dreieck
	ulong quads=0ull;
};
static bool ray_tri(const double ox,const double oy,const double oz, const double dx,const double dy,const double dz,
                    const double ax,const double ay,const double az, const double bx,const double by,const double bz,
                    const double cx,const double cy,const double cz, double* t_out) {
	// Moeller-Trumbore, double (Host, einmalig -- Robustheit vor Tempo)
	const double e1x=bx-ax, e1y=by-ay, e1z=bz-az, e2x=cx-ax, e2y=cy-ay, e2z=cz-az;
	const double px=dy*e2z-dz*e2y, py=dz*e2x-dx*e2z, pz=dx*e2y-dy*e2x;
	const double det=e1x*px+e1y*py+e1z*pz;
	if(fabs(det)<1e-12) return false;
	const double inv=1.0/det, tx=ox-ax, ty=oy-ay, tz=oz-az;
	const double u=(tx*px+ty*py+tz*pz)*inv; if(u<-1e-9||u>1.0+1e-9) return false;
	const double qx=ty*e1z-tz*e1y, qy=tz*e1x-tx*e1z, qz=tx*e1y-ty*e1x;
	const double v=(dx*qx+dy*qy+dz*qz)*inv; if(v<-1e-9||u+v>1.0+1e-9) return false;
	const double t=(e2x*qx+e2y*qy+e2z*qz)*inv; if(t<=1e-9) return false;
	*t_out=t; return true;
}
static void remesh_facetten_diag(LBM& L, const uint Nx, const uint Ny, const uint Nz,
                                 const uchar wand_flag, const string& out_dir, const double* kugel_ref=nullptr,
                                 const Mesh* stl=nullptr,
                                 std::unordered_map<ulong,std::array<uchar,18>>* q_out=nullptr) { // ★ B1-Stufe 2: q je (Zelle, Richtung) aus der GEGLAETTETEN Flaeche
	const auto idx=[&](const uint x,const uint y,const uint z){ return (ulong)x+((ulong)y+(ulong)z*(ulong)Ny)*(ulong)Nx; };
	const auto solid=[&](const int x,const int y,const int z){
		if(x<0||y<0||z<0||x>=(int)Nx||y>=(int)Ny||z>=(int)Nz) return false;
		return (L.flags[idx((uint)x,(uint)y,(uint)z)]&(TYPE_S|TYPE_E))==TYPE_S && L.flags[idx((uint)x,(uint)y,(uint)z)]==wand_flag;
	};
	// ---- 1) Grenzflaechen sammeln: Quad je solid(wand_flag)|fluid-Paar, Ecken auf dem dualen Gitter
	std::unordered_map<ulong,uint> vmap; RemeshQ M;
	std::vector<std::array<uint,4>> quads;
	const auto vkey=[&](const uint x,const uint y,const uint z){ return (ulong)x+((ulong)y+(ulong)z*(ulong)(Ny+1u))*(ulong)(Nx+1u); };
	const auto vgen=[&](const uint x,const uint y,const uint z)->uint{
		const ulong k=vkey(x,y,z); auto it=vmap.find(k);
		if(it!=vmap.end()) return it->second;
		const uint id=(uint)M.vx.size(); vmap.emplace(k,id);
		M.vx.push_back((float)x); M.vy.push_back((float)y); M.vz.push_back((float)z);
		return id;
	};
	const int fd[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
		if(!solid((int)x,(int)y,(int)z)) continue;
		for(uint f=0u; f<6u; f++) {
			const int nx2=(int)x+fd[f][0], ny2=(int)y+fd[f][1], nz2=(int)z+fd[f][2];
			if(nx2<0||ny2<0||nz2<0||nx2>=(int)Nx||ny2>=(int)Ny||nz2>=(int)Nz) continue;
			const uchar bo=L.flags[idx((uint)nx2,(uint)ny2,(uint)nz2)]&(TYPE_S|TYPE_E);
			if(bo==TYPE_S) continue;                                        // solid|solid: innen
			// Quad-Ecken der gemeinsamen Flaeche (Eckgitter: Zelle x belegt [x, x+1])
			uint c[4];
			if(fd[f][0]!=0) { const uint fx=x+(fd[f][0]>0?1u:0u);
				c[0]=vgen(fx,y,z); c[1]=vgen(fx,y+1u,z); c[2]=vgen(fx,y+1u,z+1u); c[3]=vgen(fx,y,z+1u); }
			else if(fd[f][1]!=0) { const uint fy=y+(fd[f][1]>0?1u:0u);
				c[0]=vgen(x,fy,z); c[1]=vgen(x+1u,fy,z); c[2]=vgen(x+1u,fy,z+1u); c[3]=vgen(x,fy,z+1u); }
			else { const uint fz=z+(fd[f][2]>0?1u:0u);
				c[0]=vgen(x,y,fz); c[1]=vgen(x+1u,y,fz); c[2]=vgen(x+1u,y+1u,fz); c[3]=vgen(x,y+1u,fz); }
			quads.push_back({c[0],c[1],c[2],c[3]});
		}
	}
	M.quads=(ulong)quads.size();
	if(M.quads==0ull) { print_warning("REMESH: keine Grenzflaechen gefunden (wand_flag pruefen)."); return; }
	// ---- 1b) ABNAHME TOPOLOGIE: Kantenzensus + Euler-Charakteristik. Eine geschlossene,
	// mannigfaltige Flaeche hat jede Kante genau zweimal und chi = V-E+F = 2-2g. Inzidenz 1
	// = offener Rand (am Latsch gewollt), >=3 = nicht mannigfaltig (Quetschkante an diagonal
	// beruehrenden Voxeln -- dort mittelt die Glaettung UEBER den Spalt hinweg, und keine
	// Klemme bremst das). Kostet ein paar Zeilen und faellt vor jedem Raycast auf.
	/* Quetschkanten-Fix, Stufe als Schalter: 0 = aus (STANDARD), 1 = nur die verschweissten
	   Vertices, 2 = zusaetzlich ihr 1-Ring. GEMESSEN 2026-08-23, Fahrzeug 8 mm, ITER 8:
	   Diagonalschnitte 1176 (aus) / 1240 (Stufe 1) / 2122 (Stufe 2), Restspalt-Mittel
	   1,0033 / 1,0027 / 1,0126. Festhalten macht es SCHLECHTER, nicht besser: der
	   eingefrorene Vertex bleibt als Spitze in der geglaetteten Umgebung stehen und wird
	   von Diagonallinks geschnitten. Deshalb Standard aus. Die 560 Quetschkanten bleiben
	   ein topologischer Makel -- die richtige Antwort waere Vertex-Splitting
	   (Schaefer/Ju/Warren, Manifold Dual Contouring, TVCG 2007), nicht Einfrieren. */
	const uint quetsch_stufe=env_u("CFD_REMESH_QUETSCH", 0u);
	std::vector<uchar> v_fest; // Vertices an Quetschkanten -- werden nicht geglaettet
	{
		std::unordered_map<ulong,uint> kanten;
		for(const auto& qd : quads) for(uint e=0u; e<4u; e++) {
			const uint a=qd[e], b=qd[(e+1u)&3u];
			const ulong k=(ulong)min(a,b)*4294967296ull+(ulong)max(a,b);
			kanten[k]++;
		}
		ulong e1=0ull, e2=0ull, e3=0ull;
		v_fest.assign(M.vx.size(), (uchar)0u);
		for(const auto& kv : kanten) {
			if(kv.second==1u) e1++; else if(kv.second==2u) e2++; else {
				e3++;
				/* An einer Kante mit Inzidenz >=3 sind zwei Waende zu EINEM Vertex verschweisst
				   (vmap schluesselt nur auf die Gitterecke). Die Glaettung mittelt dort UEBER
				   den Spalt hinweg und zieht ihn aktiv zu -- das begrenzt keine Klemme, weil
				   der Vertex sich von seinem Original gar nicht wegbewegen muss. Solche
				   Vertices bleiben stehen. Sauberer waere Vertex-Splitting (Schaefer/Ju/Warren,
				   Manifold Dual Contouring, TVCG 2007); bei 560 von 1,56 Mio Kanten ist
				   Festhalten die verhaeltnismaessige Antwort. */
				if(quetsch_stufe>0u) { v_fest[(uint)(kv.first>>32)]=(uchar)1u; v_fest[(uint)(kv.first&4294967295ull)]=(uchar)1u; }
			}
		}
		ulong nfest=0ull; for(const uchar f : v_fest) if(f) nfest++;
		if(nfest>0ull) print_info("REMESH TOPOLOGIE: "+to_string(nfest)+" Vertices an Quetschkanten festgehalten (nicht geglaettet).");
		const long long chi=(long long)M.vx.size()-(long long)kanten.size()+(long long)M.quads;
		print_info("REMESH TOPOLOGIE: V="+to_string((ulong)M.vx.size())+", E="+to_string((ulong)kanten.size())+", F="+to_string(M.quads)+" -> chi="+to_string((long)chi)+" (geschlossen+mannigfaltig: 2-2g).");
		if(e1>0ull||e3>0ull) print_warning("REMESH TOPOLOGIE: "+to_string(e1)+" Kanten mit Inzidenz 1 (offener Rand), "+to_string(e3)+" mit Inzidenz >=3 (NICHT mannigfaltig, Quetschkante). Inzidenz 2: "+to_string(e2)+".");
		else print_info("REMESH TOPOLOGIE: alle "+to_string(e2)+" Kanten mit Inzidenz 2 -- geschlossen und mannigfaltig.");
	}
	// ---- 2) Taubin-Glaettung auf den Vertices (Nachbarn = Quad-Kanten), Klemme +-0,5 Zelle
	const uint nv=(uint)M.vx.size();
	std::vector<std::vector<uint>> adj(nv);
	for(const auto& q : quads) for(uint e=0u; e<4u; e++) {
		const uint a=q[e], b=q[(e+1u)&3u];
		adj[a].push_back(b); adj[b].push_back(a);
	}
	if(quetsch_stufe>=2u) { // 1-Ring dazu: die Nachbarn waren der Grund, warum Stufe 1 wirkungslos blieb
		std::vector<uchar> ring=v_fest; ulong nring=0ull;
		for(uint v=0u; v<nv; v++) if(v_fest[v]) for(const uint nb : adj[v]) if(!ring[nb]) { ring[nb]=(uchar)1u; nring++; }
		v_fest.swap(ring);
		print_info("REMESH TOPOLOGIE: Quetschkanten-Stufe 2 -- zusaetzlich "+to_string(nring)+" Vertices im 1-Ring festgehalten.");
	}
	std::vector<float> ox=M.vx, oy=M.vy, oz=M.vz;                       // Originale fuer die Klemme
	std::vector<float> tx(nv), ty(nv), tz(nv);
	const float lam=0.5f, mu=-0.53f;
	// Taubin daempft bei lam/mu jedes Merkmal unter rund 11 Zellen Periode und loescht alles
	// unter rund 6 aus. Wie viel Glaettung optimal ist, entscheidet die Kugel gegen die
	// analytische Loesung -- deshalb die Iterationszahl als Schalter statt als 15.
	const uint n_it=env_u("CFD_FACETTEN_REMESH_ITER", 8u);
	for(uint it=0u; it<n_it; it++) for(uint pass=0u; pass<2u; pass++) {
		const float f=(pass==0u)?lam:mu;
		for(uint v=0u; v<nv; v++) {
			if(adj[v].empty()||v_fest[v]) { tx[v]=M.vx[v]; ty[v]=M.vy[v]; tz[v]=M.vz[v]; continue; }
			double sx=0.0, sy=0.0, sz=0.0;
			for(const uint n : adj[v]) { sx+=M.vx[n]; sy+=M.vy[n]; sz+=M.vz[n]; }
			const double k=1.0/(double)adj[v].size();
			tx[v]=M.vx[v]+f*(float)(sx*k-M.vx[v]); ty[v]=M.vy[v]+f*(float)(sy*k-M.vy[v]); tz[v]=M.vz[v]+f*(float)(sz*k-M.vz[v]);
		}
		for(uint v=0u; v<nv; v++) {                                      // harte Klemme: q-Fehler begrenzt
			M.vx[v]=fmin(ox[v]+0.5f, fmax(ox[v]-0.5f, tx[v]));
			M.vy[v]=fmin(oy[v]+0.5f, fmax(oy[v]-0.5f, ty[v]));
			M.vz[v]=fmin(oz[v]+0.5f, fmax(oz[v]-0.5f, tz[v]));
		}
	}
	// ---- 3) Triangulieren. Die Bins entstehen je Abtastung neu (die Dreiecke liegen je
	// nach Vertex-Satz woanders), siehe q_scan.
	for(const auto& qd : quads) { M.tri.insert(M.tri.end(),{qd[0],qd[1],qd[2]}); M.tri.insert(M.tri.end(),{qd[0],qd[2],qd[3]}); }
	const ulong ntri=(ulong)M.tri.size()/3ull;
	// D3Q19-Richtungen 1..18. Reihenfolge gegen die Kernel-Tabelle c() (kernel.cpp:940-942)
	// spaltenweise verifiziert (Pruefagent 2026-08-22) -- P2 darf sich darauf stuetzen.
	const int cd[18][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},
	                     {1,1,0},{-1,-1,0},{1,0,1},{-1,0,-1},{0,1,1},{0,-1,-1},
	                     {1,-1,0},{-1,1,0},{1,0,-1},{-1,0,1},{0,1,-1},{0,-1,1}};
	// ---- 3b) ENGSTELLENMASS je Fluidzelle: freie Weite in Zellen, Minimum ueber die drei
	// Achsen (Deckel 9 = offen). freie_weite==1 heisst 1-Zellen-Spalt -- Kuehlerlamelle,
	// Radhausruecken. Genau dort hat Heiko am 2026-08-22 das Zuschmieren gesehen; ohne
	// diese Zahl bliebe der Befund ein Bildbefund (Iron Rule 3).
	const auto freie_weite=[&](const uint x,const uint y,const uint z,uint* achse=nullptr)->uint {
		uint w=9u; if(achse!=nullptr) *achse=0u;
		for(uint a=0u; a<3u; a++) {
			int d0[3]={0,0,0}; d0[a]=1;
			uint run=1u;
			for(int s=-1; s<=1; s+=2) for(uint k=1u; k<9u; k++) {
				const int xx=(int)x+s*(int)k*d0[0], yy=(int)y+s*(int)k*d0[1], zz=(int)z+s*(int)k*d0[2];
				if(xx<0||yy<0||zz<0||xx>=(int)Nx||yy>=(int)Ny||zz>=(int)Nz) break;
				if((L.flags[idx((uint)xx,(uint)yy,(uint)zz)]&(TYPE_S|TYPE_E))!=0u) break;
				run++;
			}
			if(run<w) { w=run; if(achse!=nullptr) *achse=a; }
		}
		return w;
	};
	// ---- 4) q-Abtastung. Laeuft ZWEIMAL -- rohe Voxeltreppe gegen geglaettete Flaeche.
	// Erst der Vergleich beantwortet, ob das Remesh ueberhaupt etwas bringt; eine einzelne
	// q-Verteilung um 0,5 beweist gar nichts (Pruefbefund HOCH, 2026-08-22).
	struct QStat {
		ulong zellen=0ull, links=0ull, getroffen=0ull, fallback=0ull, parity_bad=0ull, kein_schnitt=0ull;
		ulong qh[10]={0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull};
		double qsum=0.0, qmin=1e30;
		ulong n_ax=0ull, n_di=0ull; double s_ax=0.0, s_di=0.0;
		ulong n_eng=0ull, n_off=0ull; double s_eng=0.0, s_off=0.0, qmin_eng=1e30;
		ulong n_ref=0ull, n_ref_aus=0ull; double e_sum=0.0, e2_sum=0.0, e_max=0.0;
		ulong ff_ax=0ull, ff_ax_kap=0ull, ff_di=0ull, ff_di_kap=0ull;
		ulong fwh[10]={0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull};
		/* RESTSPALT: fuer jede Zelle mit freier Weite 1 ist der verbleibende Weg
		   q+ plus q- entlang der engsten Achse. DAS ist die Kennzahl, die der
		   Durchschusstest haette sein sollen -- er ist fuer genau diesen Fall blind,
		   weil der Laengslink ZWISCHEN beiden Waenden hindurchlaeuft. */
		ulong n_spalt=0ull, spalt_h[11]={0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull};
		double spalt_sum=0.0, spalt_min=1e30;
		/* AUFDICKUNG: Fluidmitten, die INNERHALB der analytischen Kugel liegen. Genau die
		   hat die alte Auswertung einseitig weggeworfen -- daher war der Bias ein
		   Zensurartefakt statt einer Aussage ueber die Flaeche. */
		ulong n_innen=0ull; double ue_sum=0.0, ue_min=1e30;
		ulong n_ref_u=0ull; double e_sum_u=0.0, e2_sum_u=0.0;
	};
	/* Der alte Bin-Schluessel nutzte (Nz+2) als y-Schrittweite statt (Ny+2) und KOLLIDIERTE
	   dadurch tatsaechlich (Pruefbefund 9 -- "latent" war falsch). Folgenlos nur, weil die
	   aliasierten Dreiecke Nz bzw. Ny Zellen entfernt liegen und ray_tri sie verwirft. Als
	   Schalter erhalten, damit das Bitgleichheits-A/B in EINEM Build nachvollziehbar ist. */
	const uint bk_alt=env_u("CFD_REMESH_BINKEY_ALT", 0u);
	const auto bkey=[&](const long bx,const long by,const long bz)->ulong {
		const ulong sy=(ulong)(bk_alt>0u ? Nz+2u : Ny+2u);
		return (ulong)(bx+1)+((ulong)(by+1)+(ulong)(bz+1)*sy)*(ulong)(Nx+2u);
	};
	std::unordered_map<ulong,std::array<uchar,18>>* qfill = nullptr; // vom GEGLAETTET-Aufruf gesetzt
	const auto q_scan=[&](const std::vector<float>& VX,const std::vector<float>& VY,const std::vector<float>& VZ)->QStat {
		QStat S;
		std::unordered_map<ulong,std::vector<uint>> bins;
		for(ulong t=0ull; t<ntri; t++) {
			const uint a=M.tri[3ull*t], b=M.tri[3ull*t+1ull], c2=M.tri[3ull*t+2ull];
			const int x0=(int)floor(fmin(VX[a],fmin(VX[b],VX[c2]))), x1=(int)floor(fmax(VX[a],fmax(VX[b],VX[c2])));
			const int y0=(int)floor(fmin(VY[a],fmin(VY[b],VY[c2]))), y1=(int)floor(fmax(VY[a],fmax(VY[b],VY[c2])));
			const int z0=(int)floor(fmin(VZ[a],fmin(VZ[b],VZ[c2]))), z1=(int)floor(fmax(VZ[a],fmax(VZ[b],VZ[c2])));
			for(int zz=z0; zz<=z1; zz++) for(int yy=y0; yy<=y1; yy++) for(int xx=x0; xx<=x1; xx++)
				bins[bkey(xx,yy,zz)].push_back((uint)t);
		}
		std::vector<uint> kand; std::vector<double> tref;
		for(uint z=1u; z<Nz-1u; z++) for(uint y=1u; y<Ny-1u; y++) for(uint x=1u; x<Nx-1u; x++) {
			const ulong n=idx(x,y,z);
			if((L.flags[n]&(TYPE_S|TYPE_E))!=0u) continue;
			bool wandnah=false;
			for(uint i=0u; i<18u&&!wandnah; i++) if(solid((int)x+cd[i][0],(int)y+cd[i][1],(int)z+cd[i][2])) wandnah=true;
			if(!wandnah) continue;
			S.zellen++;
			uint eng_achse=0u; const uint fw=freie_weite(x,y,z,&eng_achse);
			const double cx0=(double)x+0.5, cy0=(double)y+0.5, cz0=(double)z+0.5; // Zellmitte im Eckgitter
			S.fwh[fw>9u?9u:fw]++;
			double q_link[18]; for(uint i=0u; i<18u; i++) q_link[i]=-1.0;
			for(uint i=0u; i<18u; i++) {
				const int nx2=(int)x+cd[i][0], ny2=(int)y+cd[i][1], nz2=(int)z+cd[i][2];
				const bool ziel_solid=solid(nx2,ny2,nz2);
				/* ABNAHME DURCHSCHUSS: eine Verbindung zwischen ZWEI Fluidzellen darf die
				   Flaeche nie schneiden -- beide Enden liegen auf der Fluidseite. Schneidet sie
				   doch, hat sich die Flaeche in den Fluidraum gewoelbt und macht einen Weg zu,
				   den das Voxelgitter offen hat. Das ist Heikos "zugeschmiert" als Zahl
				   (Iron Rule 3). Nur Achslinks sind eindeutig; Diagonallinks streifen an
				   Solid-Ecken bauartbedingt und werden getrennt und nur nachrichtlich gezaehlt. */
				if(!ziel_solid) {
					if(nx2<0||ny2<0||nz2<0||nx2>=(int)Nx||ny2>=(int)Ny||nz2>=(int)Nz) continue;
					if((L.flags[idx((uint)nx2,(uint)ny2,(uint)nz2)]&(TYPE_S|TYPE_E))!=0u) continue;
					if(i<6u) S.ff_ax++; else S.ff_di++;
					const double fx0=(double)cd[i][0], fy0=(double)cd[i][1], fz0=(double)cd[i][2];
					kand.clear();
					for(uint kk=0u; kk<2u; kk++) {
						const int bx=(int)x+(int)kk*cd[i][0], by=(int)y+(int)kk*cd[i][1], bz=(int)z+(int)kk*cd[i][2];
						auto itb=bins.find(bkey(bx,by,bz));
						if(itb!=bins.end()) kand.insert(kand.end(), itb->second.begin(), itb->second.end());
					}
					std::sort(kand.begin(), kand.end()); kand.erase(std::unique(kand.begin(),kand.end()), kand.end());
					bool kaputt=false;
					for(const uint t : kand) {
						const uint a=M.tri[3u*t], b=M.tri[3u*t+1u], c2=M.tri[3u*t+2u];
						double tt;
						if(ray_tri(cx0,cy0,cz0, fx0,fy0,fz0, VX[a],VY[a],VZ[a], VX[b],VY[b],VZ[b], VX[c2],VY[c2],VZ[c2], &tt))
							if(tt>1e-6&&tt<1.0-1e-6) { kaputt=true; break; }
					}
					if(kaputt) { if(i<6u) S.ff_ax_kap++; else S.ff_di_kap++; }
					continue;
				}
				S.links++;
				const double dxl=(double)cd[i][0], dyl=(double)cd[i][1], dzl=(double)cd[i][2];
				kand.clear();
				for(uint kk=0u; kk<2u; kk++) { // Start- und Zielzelle reichen bei |c|<=sqrt2
					const int bx=(int)x+(int)kk*cd[i][0], by=(int)y+(int)kk*cd[i][1], bz=(int)z+(int)kk*cd[i][2];
					auto itb=bins.find(bkey(bx,by,bz));
					if(itb!=bins.end()) kand.insert(kand.end(), itb->second.begin(), itb->second.end());
				}
				std::sort(kand.begin(), kand.end()); kand.erase(std::unique(kand.begin(),kand.end()), kand.end());
				tref.clear();
				for(const uint t : kand) {
					const uint a=M.tri[3u*t], b=M.tri[3u*t+1u], c2=M.tri[3u*t+2u];
					double tt;
					if(ray_tri(cx0,cy0,cz0, dxl,dyl,dzl, VX[a],VY[a],VZ[a], VX[b],VY[b],VZ[b], VX[c2],VY[c2],VZ[c2], &tt))
						if(tt<=1.0+1e-6) tref.push_back(tt);
				}
				std::sort(tref.begin(), tref.end());
				/* ABNAHME PARITY: die Verbindung startet im Fluid und endet im Solid, muss die
				   geschlossene Flaeche also UNGERADE oft schneiden. Gerade = Treffer auf einer
				   Gegenflaeche oder gefaltete Flaeche. Achtung: die Quad-Diagonale gehoert zu
				   ZWEI Dreiecken, und Achslinks treffen genau darauf -- gleiche t zusammenfassen,
				   sonst ist jede Parity trivial gerade. */
				ulong nkr=0ull; double vor=-1.0;
				for(const double tv : tref) { if(vor<0.0||tv-vor>1e-7) nkr++; vor=tv; }
				if(nkr==0ull) { S.kein_schnitt++; S.fallback++; continue; } // Kernel faellt auf HWBB
				if((nkr&1ull)==0ull) S.parity_bad++;
				const double q=fmin(1.0, tref[0]); // erster Schnitt = die gesuchte Wand
				S.getroffen++; S.qsum+=q; if(q<S.qmin) S.qmin=q; q_link[i]=q;
				{ int hb=(int)(q*10.0); if(hb>9) hb=9; if(hb<0) hb=0; S.qh[hb]++; }
				if(i<6u) { S.n_ax++; S.s_ax+=q; } else { S.n_di++; S.s_di+=q; }
				if(fw<=2u) { S.n_eng++; S.s_eng+=q; if(q<S.qmin_eng) S.qmin_eng=q; } else { S.n_off++; S.s_off+=q; }
				if(kugel_ref!=nullptr) { // analytischer Eintrittspunkt in die Kugel
					const double ex=cx0-kugel_ref[0], ey=cy0-kugel_ref[1], ez=cz0-kugel_ref[2], R=kugel_ref[3];
					const double aa=dxl*dxl+dyl*dyl+dzl*dzl, bb=2.0*(ex*dxl+ey*dyl+ez*dzl), cc2=ex*ex+ey*ey+ez*ez-R*R;
					const double disc=bb*bb-4.0*aa*cc2;
					bool ok=false; double tq=0.0;
					if(disc>=0.0) { tq=(-bb-sqrt(disc))/(2.0*aa); ok=(tq>0.0&&tq<=1.0); }
					if(ok) { const double e=q-tq; S.n_ref++; S.e_sum+=e; S.e2_sum+=e*e; if(fabs(e)>S.e_max) S.e_max=fabs(e); }
					else S.n_ref_aus++;
					/* Unzensiert: tq<=0 heisst, die Fluidmitte liegt schon INNERHALB der
					   analytischen Kugel -- der Voxelkoerper ist dort dicker als die STL.
					   Solche Links mit tq=0 mitzaehlen statt verwerfen, sonst misst der
					   Bias die Zensur und nicht die Flaeche (Pruefbefund 2). */
					if(disc>=0.0) {
						const double tq_u=fmin(1.0, fmax(0.0, tq));
						const double eu=q-tq_u; S.n_ref_u++; S.e_sum_u+=eu; S.e2_sum_u+=eu*eu;
					}
				}
			}
			if(qfill!=nullptr) { // ★ B1-Stufe 2: q je Richtung kodieren -- NACH der Richtungs-Schleife, q_link ist komplett (die erste Fassung sass IN der Schleife und haette spaete Links verloren)
				std::array<uchar,18> enc; bool any=false;
				for(uint i2=0u; i2<18u; i2++) {
					uchar qb2=0u;
					if(q_link[i2]>0.0&&q_link[i2]<=1.0) { qb2=(uchar)fmin(fmax((double)(int)(q_link[i2]*254.0+0.5),1.0),254.0); any=true; }
					enc[i2]=qb2;
				}
				(*qfill)[n]=enc; // ★ Host-Audit Befund 5: auch OHNE Schnitt eintragen (18x0 -> Konsument macht BB) -- sonst fiele die Zelle auf die auf Kruemmung widerlegte Ebenen-Quelle zurueck
				(void)any;
			}
			// RESTSPALT je Zelle mit 1-Zellen-Spalt: q+ plus q- entlang der engsten Achse.
			if(fw==1u) {
				const double qp=q_link[2u*eng_achse], qm=q_link[2u*eng_achse+1u];
				if(qp>=0.0&&qm>=0.0) {
					const double rest=qp+qm; // in Zellen; unverformt waere es 1,0
					S.n_spalt++; S.spalt_sum+=rest; if(rest<S.spalt_min) S.spalt_min=rest;
					int hb=(int)(rest*10.0); if(hb>10) hb=10; if(hb<0) hb=0; S.spalt_h[hb]++;
				}
			}
			if(kugel_ref!=nullptr) { // AUFDICKUNG: liegt die Fluidmitte innerhalb der Kugel?
				const double ex=cx0-kugel_ref[0], ey=cy0-kugel_ref[1], ez=cz0-kugel_ref[2];
				const double r=sqrt(ex*ex+ey*ey+ez*ez)-kugel_ref[3];
				if(r<0.0) { S.n_innen++; }
				S.ue_sum+=r; if(r<S.ue_min) S.ue_min=r;
			}
		}
		return S;
	};
	const auto bericht=[&](const string& na, const QStat& S) {
		print_info("REMESH ["+na+"] "+to_string(S.zellen)+" wandnahe Zellen, "+to_string(S.links)+" Solid-Links; getroffen "+to_string(S.getroffen)+" ("+to_string((float)(100.0*(double)S.getroffen/(double)max(1ull,S.links)),1u)+" %), FALLBACK->HWBB "+to_string(S.fallback)+".");
		print_info("REMESH ["+na+"] q: Mittel "+to_string((float)(S.qsum/(double)max(1ull,S.getroffen)),4u)+", min "+to_string((float)S.qmin,4u)+" | Achs "+to_string((float)(S.s_ax/(double)max(1ull,S.n_ax)),4u)+" ("+to_string(S.n_ax)+"), Diag "+to_string((float)(S.s_di/(double)max(1ull,S.n_di)),4u)+" ("+to_string(S.n_di)+").");
		print_info("REMESH ["+na+"] ENGSTELLEN (freie Weite <=2 Zellen): "+to_string(S.n_eng)+" Links, q-Mittel "+to_string((float)(S.s_eng/(double)max(1ull,S.n_eng)),4u)+", min "+to_string((float)S.qmin_eng,4u)+" || offen: "+to_string(S.n_off)+" Links, q-Mittel "+to_string((float)(S.s_off/(double)max(1ull,S.n_off)),4u)+".");
		if(S.parity_bad>0ull||S.kein_schnitt>0ull) print_warning("REMESH ["+na+"] ABNAHME PARITY VERFEHLT: "+to_string(S.parity_bad)+" Links mit GERADER Schnittzahl, "+to_string(S.kein_schnitt)+" ohne Schnitt. Soll 0/0.");
		else print_info("REMESH ["+na+"] ABNAHME PARITY bestanden (jede Verbindung schneidet die Flaeche ungerade oft).");
		if(S.n_ref>0ull) {
			print_info("REMESH ["+na+"] KUGEL-REFERENZ zensiert (nur t in (0,1], wie bisher): "+to_string(S.n_ref)+" Links, Bias "+to_string((float)(S.e_sum/(double)S.n_ref),5u)+", RMS "+to_string((float)sqrt(S.e2_sum/(double)S.n_ref),5u)+", max "+to_string((float)S.e_max,4u)+" ("+to_string(S.n_ref_aus)+" verworfen).");
			print_info("REMESH ["+na+"] KUGEL-REFERENZ UNZENSIERT (t geklemmt auf [0,1], massgeblich): "+to_string(S.n_ref_u)+" Links, Bias "+to_string((float)(S.e_sum_u/(double)max(1ull,S.n_ref_u)),5u)+", RMS "+to_string((float)sqrt(S.e2_sum_u/(double)max(1ull,S.n_ref_u)),5u)+".");
		}
		/* ACHSDURCHSCHUSS ist NICHT scharf und wird deshalb nur noch nachrichtlich gefuehrt:
		   im 1-Zellen-Spalt laeuft der Laengslink ZWISCHEN beiden Waenden hindurch und wird
		   auch bei fast vollstaendigem Verschluss nie geschnitten (Pruefbefund 4, 2026-08-22).
		   Die scharfe Zahl ist der RESTSPALT weiter unten. */
		print_info("REMESH ["+na+"] Achsdurchschuss (nachrichtlich, NICHT scharf): "+to_string(S.ff_ax_kap)+"/"+to_string(S.ff_ax)+" gekappt.");
		/* DIAGONALSCHNITTE sind der einzige reagierende Indikator und damit jetzt ABNAHME.
		   Bei der rohen Voxelflaeche sind sie bauartbedingt (der Link streift die Solid-Ecke);
		   bei der geglaetteten Flaeche sind die Ecken gerundet, dort sind es echte Schnitte. */
		if(S.ff_di_kap>0ull) print_warning("REMESH ["+na+"] ABNAHME DIAGONALSCHNITT: "+to_string(S.ff_di_kap)+" von "+to_string(S.ff_di)+" diagonalen Fluidverbindungen gekappt (Soll 0 fuer die geglaettete Flaeche; bei TREPPE ist der Streiffall an Solid-Ecken bauartbedingt).");
		else print_info("REMESH ["+na+"] ABNAHME DIAGONALSCHNITT bestanden: 0 von "+to_string(S.ff_di)+".");
		if(S.n_spalt>0ull) {
			const double m=S.spalt_sum/(double)S.n_spalt;
			print_info("REMESH ["+na+"] ABNAHME RESTSPALT ("+to_string(S.n_spalt)+" Zellen mit 1-Zellen-Spalt): mittlerer Restweg "+to_string((float)m,4u)+" Zellen, ENGSTER "+to_string((float)S.spalt_min,4u)+" (unverformt waere 1,0).");
			string h="REMESH ["+na+"] RESTSPALT-HISTOGRAMM (0,0-1,0+ in Zehnteln): "; for(int k2=0;k2<11;k2++) h+=to_string(S.spalt_h[k2])+(k2<10?" ":""); print_info(h);
			if(m<0.85) print_warning("REMESH ["+na+"] RESTSPALT: die 1-Zellen-Spalte sind im Mittel auf "+to_string((float)(100.0*m),1u)+" % ihrer Voxelweite eingeengt.");
		}
		if(S.n_innen>0ull||S.ue_min<1e29) print_info("REMESH ["+na+"] AUFDICKUNG gegen die analytische Kugel: "+to_string(S.n_innen)+" von "+to_string(S.zellen)+" wandnahen Fluidmitten liegen INNERHALB der Kugel, mittlerer Radialabstand "+to_string((float)(S.ue_sum/(double)max(1ull,S.zellen)),4u)+" Zellen, kleinster "+to_string((float)S.ue_min,4u)+".");
		{ string w="REMESH ["+na+"] FREIE WEITE der wandnahen Zellen (1..>=9 Zellen): "; for(int k2=1;k2<10;k2++) w+=to_string(S.fwh[k2])+(k2<9?" ":""); print_info(w); }
		string h="REMESH ["+na+"] q-HISTOGRAMM (Dezile 0,0-1,0): "; for(int k2=0;k2<10;k2++) h+=to_string(S.qh[k2])+(k2<9?" ":""); print_info(h);
	};
	print_info("REMESH (ELIBB P1): "+to_string(M.quads)+" Grenzquads, "+to_string((ulong)M.vx.size())+" Vertices, "+to_string(ntri)+" Dreiecke (Surface Nets + Taubin "+to_string(n_it)+"x, Klemme +-0,5 Zelle).");
	// ---- 4b) STL-DURCHGANG -- ausschliesslich als MASS FUER DIE AUFDICKUNG des Voxelkoerpers.
	// ★ HEIKO 2026-08-23: "nie auf STL Basis, wir rechnen immer auf Basis des Voxelizers."
	// q kommt also NIE von hier. Der Durchgang beantwortet nur, wie weit Voxelwand und STL
	// auseinanderliegen -- gemessen 55,70 % der Solid-Links mit der STL HINTER der Voxelwand,
	// mittleres t 1,782. Das ist ein Fehler erster Ordnung neben dem q-Fehler der Flaeche.
	// Koordinaten: sat_shell setzt
	// die Zellmitte auf (x,y,z) (setup.cpp:299), der Remesh auf (x+0,5,...) -- die STL liegt
	// also um eine halbe Zelle versetzt, Ursprung des Strahls ist hier (x,y,z).
	if(stl!=nullptr) {
		std::unordered_map<ulong,std::vector<uint>> sbins;
		for(uint t=0u; t<stl->triangle_number; t++) {
			const float3 &A=stl->p0[t], &B=stl->p1[t], &C=stl->p2[t];
			const int x0=(int)floor(fmin(A.x,fmin(B.x,C.x))), x1=(int)floor(fmax(A.x,fmax(B.x,C.x)));
			const int y0=(int)floor(fmin(A.y,fmin(B.y,C.y))), y1=(int)floor(fmax(A.y,fmax(B.y,C.y)));
			const int z0=(int)floor(fmin(A.z,fmin(B.z,C.z))), z1=(int)floor(fmax(A.z,fmax(B.z,C.z)));
			if(x1<-1||y1<-1||z1<-1||x0>(int)Nx||y0>(int)Ny||z0>(int)Nz) continue;
			for(int zz=max(-1,z0); zz<=min((int)Nz,z1); zz++) for(int yy=max(-1,y0); yy<=min((int)Ny,y1); yy++) for(int xx=max(-1,x0); xx<=min((int)Nx,x1); xx++)
				sbins[(ulong)(xx+1)+((ulong)(yy+1)+(ulong)(zz+1)*(ulong)(Ny+2u))*(ulong)(Nx+2u)].push_back(t);
		}
		/* Die STL-Koordinaten kommen aus place() (Zellen ab Domaenenursprung), die SAT-Schale
		   setzt die Zellmitte auf (x,y,z), der FluidX3D-Voxelierer dagegen auf die zentrierte
		   position()-Konvention. Zwei Systeme -- also nicht lesen, sondern MESSEN: beide
		   Ursprungsannahmen laufen mit, die mit der hoeheren Trefferquote ist die richtige. */
		ulong nl=0ull, n_eng=0ull;
		// ★ L0-ZENSUS (Planungsagent Dachabloesung 30.08., Heiko-Freigabe 18:55): derselbe STL-Durchgang, eingeschraenkt
		// auf einen x-Bereich in Zellen dieser Domaene (CFD_ZENSUS_X0..X1, z. B. das flache Dach i_x 742-829 bei 4 mm, B58),
		// getrennt nach vertikalen Links (0,0,+-1) und allen. Frage: liegt die wahre Wand dort innerhalb einer Linklaenge
		// (dann kann eine Blende q in (0,1] sie darstellen) oder dahinter (dann nicht -- Geometriefehler erster Ordnung).
		const uint zx0=env_u("CFD_ZENSUS_X0",0u), zx1=env_u("CFD_ZENSUS_X1",0u); const bool zroi=zx1>zx0;
		ulong nl_r[2]={0ull,0ull}, ntr_r[2]={0ull,0ull}, nh_r[2]={0ull,0ull}, nle_r[2]={0ull,0ull}; double th_r[2]={0.0,0.0}, sq_r[2]={0.0,0.0};
		ulong nlv_r[2]={0ull,0ull}, ntrv_r[2]={0ull,0ull}, nhv_r[2]={0ull,0ull}; double thv_r[2]={0.0,0.0};
		ulong ntr[2]={0ull,0ull}, nleer[2]={0ull,0ull}, neng_tr[2]={0ull,0ull};
		ulong nkeine[2]={0ull,0ull}, nhinter[2]={0ull,0ull}; double th_sum[2]={0.0,0.0}, th_max[2]={0.0,0.0};
		double sq[2]={0.0,0.0}, sq_eng[2]={0.0,0.0}, qmin[2]={1e30,1e30};
		ulong qh2[2][10]={{0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull},{0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull}};
		std::vector<uint> kand;
		/* FAIRE PRUEFUNG (Pruefbefund 8): der alte Test brach bei t>1 ab und konnte
		   "keine STL da" nicht von "die Voxelwand steht VOR der STL" unterscheiden. Jetzt
		   wird bis t<=4 gesucht und zusaetzlich gemeldet, ob ueberhaupt Dreiecke im Suchraum
		   lagen. Erst damit ist die Aussage ueber die STL belegt statt behauptet. */
		const uint stl_kk=4u;
		const auto schuss=[&](const double ox_,const double oy_,const double oz_,const uint i,double* qout,bool* leer=nullptr)->bool {
			kand.clear();
			for(uint kk=0u; kk<=stl_kk; kk++) {
				const int bx=(int)floor(ox_)+(int)kk*cd[i][0], by=(int)floor(oy_)+(int)kk*cd[i][1], bz=(int)floor(oz_)+(int)kk*cd[i][2];
				auto itb=sbins.find((ulong)(bx+1)+((ulong)(by+1)+(ulong)(bz+1)*(ulong)(Ny+2u))*(ulong)(Nx+2u));
				if(itb!=sbins.end()) kand.insert(kand.end(), itb->second.begin(), itb->second.end());
			}
			std::sort(kand.begin(), kand.end()); kand.erase(std::unique(kand.begin(),kand.end()), kand.end());
			if(leer!=nullptr) *leer=kand.empty();
			double best=1e30;
			for(const uint t : kand) {
				const float3 &A=stl->p0[t], &B=stl->p1[t], &C=stl->p2[t];
				double tt;
				if(ray_tri(ox_,oy_,oz_, (double)cd[i][0],(double)cd[i][1],(double)cd[i][2],
				           (double)A.x,(double)A.y,(double)A.z, (double)B.x,(double)B.y,(double)B.z, (double)C.x,(double)C.y,(double)C.z, &tt))
					if(tt<best) best=tt;
			}
			if(best>1e29) return false;  // wirklich kein Schnitt im gesamten Suchraum
			*qout=best; return true;     // NICHT geklemmt -- der Aufrufer entscheidet
		};
		for(uint z=1u; z<Nz-1u; z++) for(uint y=1u; y<Ny-1u; y++) for(uint x=1u; x<Nx-1u; x++) {
			const ulong n=idx(x,y,z);
			if((L.flags[n]&(TYPE_S|TYPE_E))!=0u) continue;
			bool wandnah=false;
			for(uint i=0u; i<18u&&!wandnah; i++) if(solid((int)x+cd[i][0],(int)y+cd[i][1],(int)z+cd[i][2])) wandnah=true;
			if(!wandnah) continue;
			const uint fw=freie_weite(x,y,z);
			const double sx0=(double)x, sy0=(double)y, sz0=(double)z; // STL-Konvention: Zellmitte = (x,y,z)
			for(uint i=0u; i<18u; i++) {
				const int nx2=(int)x+cd[i][0], ny2=(int)y+cd[i][1], nz2=(int)z+cd[i][2];
				if(!solid(nx2,ny2,nz2)) continue;
				nl++; if(fw<=2u) n_eng++;
				const bool inroi = zroi&&x>=zx0&&x<=zx1, vert = (cd[i][0]==0&&cd[i][1]==0);
				for(uint v=0u; v<2u; v++) {
					const double off=(v==0u)?0.0:0.5;
					double qs; bool leer=false;
					if(inroi) { nl_r[v]++; if(vert) nlv_r[v]++; }
					if(!schuss(sx0+off,sy0+off,sz0+off,i,&qs,&leer)) { nleer[v]++; if(leer) nkeine[v]++; if(inroi) nle_r[v]++; continue; }
					if(qs>1.0+1e-6) { nhinter[v]++; th_sum[v]+=qs; if(qs>th_max[v]) th_max[v]=qs; if(inroi) { nh_r[v]++; th_r[v]+=qs; if(vert) { nhv_r[v]++; thv_r[v]+=qs; } } continue; } // STL liegt HINTER der Voxelwand
					ntr[v]++; sq[v]+=qs; if(qs<qmin[v]) qmin[v]=qs; if(inroi) { ntr_r[v]++; sq_r[v]+=qs; if(vert) ntrv_r[v]++; }
					{ int hb=(int)(qs*10.0); if(hb>9) hb=9; if(hb<0) hb=0; qh2[v][hb]++; }
					if(fw<=2u) { neng_tr[v]++; sq_eng[v]+=qs; }
				}
			}
		}
		for(uint v=0u; v<2u; v++) {
			const string na=(v==0u)?"STL Ursprung (x,y,z)":"STL Ursprung (x+0,5)";
			print_info("REMESH ["+na+"]: "+to_string(nl)+" Solid-Links. Mit STL-Schnitt innerhalb einer Linklaenge: "+to_string(ntr[v])+" ("+to_string((float)(100.0*(double)ntr[v]/(double)max(1ull,nl)),2u)+" %). STL liegt HINTER der Voxelwand (t in (1,"+to_string(stl_kk)+"]): "+to_string(nhinter[v])+" ("+to_string((float)(100.0*(double)nhinter[v]/(double)max(1ull,nl)),2u)+" %), mittleres t "+to_string((float)(th_sum[v]/(double)max(1ull,nhinter[v])),3u)+", groesstes "+to_string((float)th_max[v],2u)+". GAR KEIN Dreieck im Suchraum: "+to_string(nkeine[v])+". Sonstige ohne Schnitt: "+to_string(nleer[v]-nkeine[v])+".");
			print_info("REMESH ["+na+"] q: Mittel "+to_string((float)(sq[v]/(double)max(1ull,ntr[v])),4u)+", min "+to_string((float)qmin[v],4u)+" | Engstellen (Weite<=2): "+to_string(neng_tr[v])+" von "+to_string(n_eng)+" getroffen, q-Mittel "+to_string((float)(sq_eng[v]/(double)max(1ull,neng_tr[v])),4u)+".");
			string h="REMESH ["+na+"] q-HISTOGRAMM (Dezile 0,0-1,0): "; for(int k2=0;k2<10;k2++) h+=to_string(qh2[v][k2])+(k2<9?" ":""); print_info(h);
			if(zroi) { // ★ L0-ZENSUS-Ausgabe
				const double pn=(double)max(1ull,nl_r[v]), pv=(double)max(1ull,nlv_r[v]);
				print_info("ZENSUS-ROI ["+na+"] x in ["+to_string(zx0)+","+to_string(zx1)+"] Zellen: "+to_string(nl_r[v])+" Solid-Links | innerhalb einer Linklaenge "+to_string(ntr_r[v])+" ("+to_string((float)(100.0*(double)ntr_r[v]/pn),1u)+" %, q-Mittel "+to_string((float)(sq_r[v]/(double)max(1ull,ntr_r[v])),3u)+") | STL DAHINTER "+to_string(nh_r[v])+" ("+to_string((float)(100.0*(double)nh_r[v]/pn),1u)+" %, mittleres t "+to_string((float)(th_r[v]/(double)max(1ull,nh_r[v])),3u)+") | kein Schnitt bis t<=4: "+to_string(nle_r[v])+" ("+to_string((float)(100.0*(double)nle_r[v]/pn),1u)+" %)");
				print_info("ZENSUS-ROI ["+na+"] nur VERTIKALE Links (0,0,+-1): "+to_string(nlv_r[v])+" | innerhalb "+to_string(ntrv_r[v])+" ("+to_string((float)(100.0*(double)ntrv_r[v]/pv),1u)+" %) | dahinter "+to_string(nhv_r[v])+" ("+to_string((float)(100.0*(double)nhv_r[v]/pv),1u)+" %, mittleres t "+to_string((float)(thv_r[v]/(double)max(1ull,nhv_r[v])),3u)+") -- Kriterium L0: < 50 % innerhalb => Blenden-Schiene am Dach gestrichen.");
			}
		}
	}
	bericht("TREPPE", q_scan(ox,oy,oz));       // rohe Voxelflaeche = Bezug
	qfill = q_out; // ★ B1-Stufe 2: NUR der geglaettete Scan fuellt die Karte
	bericht("GEGLAETTET", q_scan(M.vx,M.vy,M.vz));
	qfill = nullptr;
	if(q_out!=nullptr) print_info("ELIBB Stufe 2: Remesh-q-Karte mit "+to_string((ulong)q_out->size())+" Zellen gefuellt (geglaettete Flaeche).");
	// VTK der Flaeche fuer die Sichtpruefung (Iron Rule 3 gilt fuers MESSEN; sichten ist erlaubt)
	// ★ Heiko 29.08.: "bekommst das allgemein aufgefangen, so dass es immer richtig sitzt?"
	// Diese Flaeche stand in ROHEN Gitterkoordinaten, das Guete-VTK und die Feld-VTKs in
	// Weltkoordinaten -- in ParaView lagen sie deshalb weder deckungsgleich noch im selben
	// Massstab. Jetzt derselbe Raum, aus denselben vier Statiken. Stehen sie auf ihren
	// Defaults (0/0/0/1), bleibt es wie bisher bei Gitterkoordinaten -- und die Meldung sagt es.
	std::ofstream vtk(out_dir+"remesh_flaeche.vtk");
	vtk << "# vtk DataFile Version 3.0\nRemesh ELIBB P1\nASCII\nDATASET POLYDATA\nPOINTS " << M.vx.size() << " float\n";
	for(uint v=0u; v<nv; v++) vtk << (vtk_x0+vtk_dx*M.vx[v]) << " " << (vtk_y0+vtk_dx*M.vy[v]) << " " << (vtk_z0+vtk_dx*M.vz[v]) << "\n";
	vtk << "POLYGONS " << ntri << " " << 4ull*ntri << "\n";
	for(ulong t=0ull; t<ntri; t++) vtk << "3 " << M.tri[3ull*t] << " " << M.tri[3ull*t+1ull] << " " << M.tri[3ull*t+2ull] << "\n";
	vtk.close();
	print_info(string("REMESH: Flaeche geschrieben -> ")+out_dir+"remesh_flaeche.vtk ("
		+((vtk_dx==1.0f&&vtk_x0==0.0f&&vtk_y0==0.0f&&vtk_z0==0.0f)?string("GITTERkoordinaten -- die Aufrufstelle hat keine Weltlage gesetzt"):string("Weltkoordinaten -- deckungsgleich mit den Feld- und Guete-VTKs"))+").");
}

// 3x3-Jacobi in double: Eigenvektor zum kleinsten Eigenwert von M (symmetrisch).
static void jacobi3(double M[3][3], double ew[3], double ev[3][3]) {
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) ev[i][j] = (i==j)?1.0:0.0;
	for(uint sweep=0u; sweep<32u; sweep++) {
		double off = fabs(M[0][1])+fabs(M[0][2])+fabs(M[1][2]);
		if(off<1e-30) break;
		for(int p=0;p<2;p++) for(int q=p+1;q<3;q++) {
			if(fabs(M[p][q])<1e-300) continue;
			const double theta = 0.5*(M[q][q]-M[p][p])/M[p][q];
			const double t = (theta>=0.0?1.0:-1.0)/(fabs(theta)+sqrt(theta*theta+1.0));
			const double c = 1.0/sqrt(t*t+1.0), sn = t*c;
			for(int k=0;k<3;k++) {
				const double mkp=M[k][p], mkq=M[k][q];
				M[k][p]=c*mkp-sn*mkq; M[k][q]=sn*mkp+c*mkq;
			}
			for(int k=0;k<3;k++) {
				const double mpk=M[p][k], mqk=M[q][k];
				M[p][k]=c*mpk-sn*mqk; M[q][k]=sn*mpk+c*mqk;
				const double vkp=ev[k][p], vkq=ev[k][q];
				ev[k][p]=c*vkp-sn*vkq; ev[k][q]=sn*vkp+c*vkq;
			}
		}
	}
	for(int i=0;i<3;i++) ew[i]=M[i][i];
}
// D3Q19-Richtungen 1..18 (Host-Kopie der Kernel-Tabelle; nur fuer den Facettenbau).
static const int FZ_C[19][3] = {{0,0,0},{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},
	{1,1,0},{-1,-1,0},{1,0,1},{-1,0,-1},{0,1,1},{0,-1,-1},{1,-1,0},{-1,1,0},{1,0,-1},{-1,0,1},{0,1,-1},{0,-1,1}};
std::vector<Facette> baue_facetten(LBM& L, const uint Nx, const uint Ny, const uint Nz,
                                   const uchar wand_flag, const string& out_dir, const char* wo,
                                   const bool z_per=false) { // I2: Torus-Kipp wickelt auch z
	// ★ Fenster-A/B 2026-08-15 (Fahrzeug 8 mm, je 753.592 Zellen): 3^3 gewinnt klar -- r21-q90
	// 0,21/0,27/0,42 und Orientierungskonflikte 2,7/5,4/9,0 % fuer 3/5/7; 7^3 driftet y_w auf 0,617.
	// Groessere Fenster sehen Kruemmung und Zweitflaechen, keine bessere Wand.
	const int R = (int)min(3u, max(1u, env_u("CFD_FACETTEN_FENSTER", 1u))); // Radius: 1=3^3 (Default, geeicht), 2=5^3, 3=7^3
	if(env_u("CFD_FACETTEN_FENSTER", 1u)>3u) print_warning("CFD_FACETTEN_FENSTER > 3 wird auf 3 geklemmt (Audit R3: vorher stille Klemme).");
	// ★ Stufe-3-Schritt 6: K4-Untergrenze als deklarierter Messarm-Parameter. Default 0,2 UNVERAENDERT
	// (Fahrzeug-Eichung!); der 26,6-Grad-Torus faehrt 0,15, weil seine tragende m0-Lage bei
	// y_w=0,187 liegt (Census 2026-08-16 bestaetigte die Formelblatt-Prognose 0,184).
	const float yw_min = env_f("CFD_FACETTEN_YWMIN", 0.2f);
	if(env_u("CFD_KANAL_KIPP",0u)==26u&&env_u("CFD_FACETTEN",0u)>0u&&yw_min>=0.187f) print_error("kipp=26 braucht CFD_FACETTEN_YWMIN<0,187 (tragende m0-Lage bei y_w=0,187) -- sonst 33 % BB-Loecher und K3-Abbruch nach Stunden (IR3-Audit).");
	if(yw_min!=0.2f) print_warning("CFD_FACETTEN_YWMIN = "+to_string(yw_min,3u)+" (Default 0,2) -- deklarierter Messarm, Ergebnisse entsprechend kennzeichnen.");
	if(getenv("CFD_FACETTEN_FENSTER")!=nullptr&&env_u("CFD_FACETTEN_FENSTER", 1u)==0u) print_warning("CFD_FACETTEN_FENSTER=0 wird auf 1 gehoben (Radius 0 gibt es nicht).");
	const uint np_max = (uint)((2*R+1)*(2*R+1)*(2*R+1))*18u; // absolutes Maximum geschnittener Links im Fenster
	std::vector<double> px(np_max), py(np_max), pz(np_max);
	// ★ DREIFACHVERGLEICH DER NORMALENQUELLE (Heiko-Auftrag 28.08.). Standard AUS -> bitidentisch.
	//   V1 = heute: PCA ueber die Mitten ALLER 18 geschnittenen Links (px/py/pz oben).
	//   V2 = nur die ACHSLINKS, also die echten Voxelflaechen, ebenfalls per PCA.
	//   V3 = HEIKOS IDEE: flaechengewichtete Vektorsumme der Voxelflaechen-Normalen; y_w gegen
	//        den flaechengewichteten Flaechenschwerpunkt. Kein Eigenwertproblem, also
	//        konstruktiv KEIN K2/K3 -- die Klasse "Kante" kann dort gar nicht entstehen.
	// Der Remesh wird dafuer NICHT gebraucht: die rohe Voxeloberflaeche SIND die achsparallelen
	// Solid/Fluid-Flaechen, die diese Schleife ohnehin schon findet (B36: die rohe Flaeche
	// liefert q exakt 0,5 auf allen Links, ihre Geometrie steckt vollstaendig in den Achslinks).
	const bool vgl_an = env_u("CFD_FACETTEN_VERGLEICH", 0u)>0u;
	// ★ AKTIVE NORMALENQUELLE (Heiko-Auftrag 28.08.). 0 = V1 wie bisher (Default, bitidentisch),
	// 1 = V3b: Heikos Flaechennormalen-Summe + Duennteil-Rueckfall + y_w gegen die naechste
	// Flaeche. Belege: B39/B40 (Verwerfung 21,88 -> 7,26 % bei 8 mm, K1 und K2 strukturell null),
	// B41/B42 (Genauigkeit exakt V3, mein Sichtbarkeitsfilter war der Schaden und ist raus),
	// B44 (V1s Winkelfehler saettigt bei 4,69 Grad, V3 konvergiert weiter auf 4,54).
	// NICHT belegt (B43): dass das die Kraefte verbessert. Genau dafuer ist dieser Schalter da.
	// ★ Heiko 29.08.: "auf der Aussenwand darf nichts verworfen werden". Zwei Schalter, beide
	// Default AUS, damit der heutige Stand bitgleich bleibt und der A/B einvariabel ist.
	//   YWKLEMME: ein zu kleines y_w macht die Facette nicht ungueltig -- es macht EINE Zahl
	//     unsicher (die Abtasthoehe des Spalding-Arguments Y = u_t*y_w/nu). Bei y_w = 0,16 ist
	//     Y noch rund 850, also mitten im gueltigen Bereich; singulaer wird es erst bei y_w = 0.
	//     Statt zu verwerfen wird auf yw_min geklemmt -- dieselbe Bauform wie der q-Boden
	//     (s_fac_qmin, lbm.cpp:593). Gemessen 4 mm: 459.224 Zellen, ALLE an der Untergrenze,
	//     Haeufung bei 0,14..0,18 -- das sind die diskreten y_w-Klassen der geneigten Treppe.
	//   KANTE_KOH: unter NORMQUELLE=1 kommt die Normale NICHT mehr aus der PCA, der Kantentest
	//     misst aber weiter deren Fitguete r21. Wir verwerfen also wegen eines Fits, den wir
	//     nicht benutzen. Mit dem Schalter entscheidet stattdessen V3bs eigenes Guetemass:
	//     die Kohaerenz |Summe|/nq gegen 1/sqrt(3) (hergeleitet, siehe Waechter weiter unten).
	//     Gemessen liegen dort nur 4,03 % statt 9,55 % bei r21.
	const bool yw_klemme = env_u("CFD_FACETTEN_YWKLEMME", 0u)>0u;
	const bool kante_koh = env_u("CFD_FACETTEN_KANTE_KOH", 0u)>0u;
	const bool guete_vtk = env_u("CFD_FACETTEN_VTK", 0u)>0u;
	ulong n_geklemmt=0ull, n_kante_koh=0ull;
	const uint normquelle = env_u("CFD_FACETTEN_NORMQUELLE", 0u);
	// ★ ZERLEGUNG (Pruefagent 28.08.): NORMQUELLE aenderte SECHS Dinge zugleich, der A/B war
	// damit nicht einvariabel. Drei davon sind trennbar und werden es hier:
	//  (a) die Normale selbst -- bleibt an NORMQUELLE, samt Achse, Flaechenfaktor und
	//      Grazing-Guard-Richtung. Die vier sind physikalisch NICHT trennbar: wer die Ebene
	//      wechselt, muss auch deren Achse und deren Guard nehmen.
	//  (b) der y_w-Anker -- eigener Schalter. V3bs y_w ist eine ANDERE GROESSE als V1s, nicht
	//      derselbe Wert genauer: gegen die naechste Flaeche gemessen ist er fuer achsbenachbarte
	//      Zellen hart <= 0,5. Das ist eine zweite physikalische Hypothese und gehoert nicht in
	//      denselben Schalter wie die Normale.
	//  (c) der Kantentest -- war unter V3b GAR NICHT MEHR GESTELLT, weil r21/r10 nicht neu
	//      gesetzt wurden. Das war Bequemlichkeit, keine Entscheidung: die PCA laeuft ohnehin
	//      vollstaendig durch, r21 liegt kostenlos vor. 46,4 % des gemessenen Abdeckungsgewinns
	//      kamen allein daher. Jetzt quellenunabhaengig.
	const uint ywquelle  = env_u("CFD_FACETTEN_YWQUELLE", 0u);   // 0 = PCA-Schwerpunkt (V1), 1 = naechste Flaeche
	ulong yw_aktiv=0ull; // ★ Wirkpfad des y_w-Ankers -- ohne ihn war der Schalter ein stiller No-Op
	// ★ Pruefagent M1: als float war (double)0.15f = 0,15000000596 -- die Schwelle WANDERTE um
	// 6e-9 gegen das alte Literal 0.15. Facetten mit r21 in diesem Band haetten Bit 2 verloren
	// und waeren damit von ausgeschlossen auf AKTIV gekippt. Bit 2 ist ein Physik-Gate, kein
	// Diagnosebit. Als double gelesen ist der Default wieder bitgleich.
	const double kante_r21 = (getenv("CFD_FACETTEN_KANTE")==nullptr) ? 0.15 : atof(getenv("CFD_FACETTEN_KANTE"));
	if(kante_r21<=0.0) print_error("CFD_FACETTEN_KANTE muss > 0 sein (sehr gross = Test aus).");
	if(normquelle>0u&&getenv("CFD_FACETTEN_KANTE")==nullptr)
		print_warning("CFD_FACETTEN_NORMQUELLE=1 ohne ausdrueckliches CFD_FACETTEN_KANTE -- der Kantentest laeuft mit dem Default 0,15 WEITER. Das ist Absicht (frueher fiel er still weg); wer ihn abschalten will, setzt ihn ausdruecklich.");
	if(getenv("CFD_FACETTEN_KANTE")!=nullptr) print_warning("CFD_FACETTEN_KANTE = "+to_string((float)kante_r21,3u)+" (Default 0,15, geeicht 2026-08-15) -- deklarierter Messarm.");
	if(ywquelle>0u) print_warning("CFD_FACETTEN_YWQUELLE=1 -- y_w gegen die naechste Voxelflaeche statt gegen den PCA-Schwerpunkt. ANDERE GROESSE, nicht derselbe Wert genauer.");
	const bool v3_noetig = vgl_an||normquelle>0u||ywquelle>0u; // ★ H1: der y_w-Anker braucht c6, also den V3-Block
	ulong nq_aktiv=0ull; // Eintritte in den V3b-Schreibblock
	// ★ Audit F 29.08.: nq_aktiv/yw_aktiv zaehlten den EINTRITT, nicht die WIRKUNG -- sie
	// meldeten "755344 von 755344" und konnten einen No-Op konstruktiv nie aufdecken. Die
	// folgenden Zaehler vergleichen gegen den V1-Zustand DIREKT VOR dem Schreiben.
	ulong nq_n_geaendert=0ull, nq_yw_geaendert=0ull, nq_kipp=0ull;
	ulong yw_n_geaendert=0ull, yw_yw_geaendert=0ull, yw_kipp=0ull, yw_ori_neu=0ull;
	if(normquelle>1u) print_error("CFD_FACETTEN_NORMQUELLE="+to_string(normquelle)+" ist nicht belegt -- nur 0 und 1 (Pruefagent M2: sonst warnt der Lauf 'V3b aktiv' und rechnet V1).");
	if(normquelle==1u) print_warning("CFD_FACETTEN_NORMQUELLE=1 -- die Normale kommt aus V3b (Voxelflaechen-Summe), NICHT aus der 18-Link-PCA. Wirkt auf fac_geo (Wandfunktion UND Flaechenfaktor 1/|n_achse|) sowie auf den ELIBB-Ebenen-q sq=yw/(-ndc). Deklarierter Messarm.");
	std::vector<double> qx(v3_noetig?np_max:0u), qy(v3_noetig?np_max:0u), qz(v3_noetig?np_max:0u);
	std::vector<uchar>  qd(v3_noetig?np_max:0u); // Richtungsindex der Achsflaeche -> Normale = -FZ_C[qd]
	std::vector<uchar>  qt(v3_noetig?np_max:0u); // ★ DUENNTEIL-FLAGGE je Flaeche (Heiko 28.08.):
	// "wenn rueckseitig des dreiecks keine weitere solidzelle ist, ist das dann ein duennteil".
	// Test ist O(1): hinter der Solidzelle S in derselben Richtung nachsehen. Kein Solid -> S ist
	// in dieser Richtung genau EINE Zelle dick, die Flaeche gehoert also zu einem Duennteil.
	ulong v4_duenn=0ull, v4_rueck=0ull, v4_unten=0ull, v4_oben=0ull; // Wirkpfad-Zaehler V4/V5
	// ★ ENTARTUNGSWAECHTER (Pruefagent 28.08.): V3b faengt bisher nur die EXAKTE Ausloeschung
	// ab (|Summe| < 1e-12). Die viel haeufigere FAST-Ausloeschung ist ungeschuetzt: an einem
	// 1-Zellen-Blech oder in einem 1-Zellen-Spalt ist die Summe klein, aber nicht null -- etwa
	// drei Flaechen gegen zwei. Die Restrichtung ist dann Rauschen, wird auf Laenge 1 normiert
	// und traegt in der Glaettung volles Gewicht. Und genau dort waechst die Abdeckung am
	// staerksten: 38,5 % der Neuzugaenge sitzen an Teilen <= 2 Zellen Dicke.
	// |Summe|/nq ist das Mass dafuer (1 = alle Flaechen gleichgerichtet, 0 = vollstaendige
	// Ausloeschung). Es wird ohnehin gerechnet und bisher weggeworfen.
	std::vector<double> v3b_koh; ulong v3b_schwach=0ull;
	// ★ Guete je Facette, PARALLEL zu F (Audit E: an BEIDEN F.push_back-Stellen mitschieben!).
	// Bit 1 Duennteil-Rueckfall | Bit 2 schwache Kohaerenz | Bit 4 y_w geklemmt
	std::vector<uchar> gt_v; std::vector<float> koh_v;
	// ★ KUGEL-GRUNDWAHRHEIT (CFD_FACETTEN_KUGELREF=1). Die Kugel ist der EINZIGE Fall mit
	// analytischer Normalen: (Zelle - Schwerpunkt)/|...|. Am Fahrzeug gibt es keine, dort ist
	// nur die VERWERFUNG vergleichbar, nicht die Genauigkeit. Schwerpunkt hier selbst bestimmen,
	// damit weder Signatur noch Aufrufstellen angefasst werden muessen.
	const bool kref_an = vgl_an&&env_u("CFD_FACETTEN_KUGELREF", 0u)>0u;
	double kmx=0.0, kmy=0.0, kmz=0.0;
	std::vector<float> nv1, nv2, nv3, nv4; // Normalen fuer den Vergleich gegen die Wahrheit
	if(kref_an) {
		ulong nsol=0ull;
		for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++)
			if(L.flags[(ulong)x+((ulong)y+(ulong)z*(ulong)Ny)*(ulong)Nx]==wand_flag) { kmx+=(double)x; kmy+=(double)y; kmz+=(double)z; nsol++; } // idx() ist hier noch nicht deklariert
		if(nsol==0ull) print_error("KUGELREF: keine Wandzelle gefunden -- Referenz unbrauchbar.");
		else { kmx/=(double)nsol; kmy/=(double)nsol; kmz/=(double)nsol;
			print_info("  KUGELREF: "+to_string(nsol)+" Solidzellen, Schwerpunkt ("+to_string((float)kmx,2u)+", "
				+to_string((float)kmy,2u)+", "+to_string((float)kmz,2u)+"), volumenaequivalenter Radius "
				+to_string((float)pow(3.0*(double)nsol/(4.0*M_PI),1.0/3.0),3u)+" Zellen."); }
	}
	std::vector<float> a14, yw4v, yw4nv, yw4av, yw5v; std::vector<uchar> kl4v, kl5v;
	// ★ V5 = V3 + Sichtbarkeit + Rueckfall, ABER OHNE Abstandsgewicht (alle sichtbaren Flaechen
	// gleich). V4 hat beide Zusaetze zugleich bekommen; die Kugel (B41) zeigt V1 6,67 -> V3 9,89
	// -> V4 13,54 Grad, also kostet mindestens einer davon Genauigkeit. V5 trennt sie.
	std::vector<float> nv5, nv6;
	// ★ V3b = HEIKOS V3 OPTIMIERT, ohne den schaedlichen Sichtbarkeitsfilter. Kugel B41/V5:
	// V3 9,89/5,53 gegen V4 13,54/9,47 und V5 14,96/9,60 Grad -- nicht das Abstandsgewicht
	// kostet die Genauigkeit, sondern die SICHTBARKEIT (halbiert am konvexen Koerper die
	// beitragenden Flaechen). V3b nimmt daher NUR die zwei Zusaetze, die Verwerfungen beheben
	// und die Mittelung nicht antasten: (a) Duennteil-Rueckfall auf die naechste Flaeche, wenn
	// die Vektorsumme entartet, (b) y_w gegen die naechste Flaeche statt gegen den Schwerpunkt.
	std::vector<float> yw6v; std::vector<uchar> kl6v; ulong v6_rueck=0ull; // V4 = V3 + Sichtbarkeit + Abstandsgewicht
	std::vector<float> a12, a13, yw2v, yw3v; std::vector<uchar> kl2v, kl3v, kl1v;
	// ★ PRUEFAGENT H1 (28.08.): die Tabelle stellte V1 NACH der Glaettung gegen V2..V3b DAVOR --
	// F[i].klasse ist der Endzustand (die Glaettung bewertet K4 neu, Bit 8), kl2v..kl6v stammen
	// aus der Zellschleife. Der Vergleich enthielt damit einen unbekannten Anteil "mit gegen ohne
	// Glaettung". kl1v haelt V1s Klasse VOR der Glaettung; die Tabelle zeigt jetzt beides.
	if(vgl_an) print_info(string("Facetten (")+wo+"): NORMALEN-DREIFACHVERGLEICH aktiv (CFD_FACETTEN_VERGLEICH=1) -- reine Diagnose, Physik unveraendert.");
	print_info(string("Facetten (")+wo+"): Fenster "+to_string(2*R+1)+"^3 (CFD_FACETTEN_FENSTER="+to_string((uint)R)+")");
	auto idx = [&](const uint x, const uint y, const uint z) { return (ulong)x+((ulong)y+(ulong)z*(ulong)Ny)*(ulong)Nx; };
	auto ist_wand  = [&](const ulong n) { return L.flags[n]==wand_flag; };
	auto ist_fluid = [&](const ulong n) { const uchar f=L.flags[n]&(TYPE_S|TYPE_E); return f==0u; };
	// Kandidaten: Fluidzellen mit mindestens einem wand_flag-Nachbarn unter den 18 Richtungen.
	std::vector<Facette> F;
	ulong k1=0ull,k2=0ull,k3=0ull,k4=0ull,kori=0ull;
	std::vector<double> hist_yw, hist_winkel; // Audit 3/3 N3: r21/r10-Histogramme waren tot (leben in der Facette/CSV)
	// ★ Stufe 2 (F6): x/y PERIODISCH gewickelt -- die z-WFB behandelt alle Wandzellen, der
	// Facettenpfad muss es auch (sonst kein Ist=Soll und keine Kanal-Aequivalenz). z bleibt 1..Nz-2.
	// ★ Audit-Performance 29.08.: das Zellgate laeuft ueber JEDE Fluidzelle der Domaene
	// (4 mm: 1689x621x483 ~ 5,07e8) mal 18 Nachbarn, und jeder Aufruf trug ZWEI Ganzzahl-
	// Divisionen -- rund 3,6e10 idiv, einkernig. Die Offsets sind |d| <= 2 (5^3-Fenster),
	// also greift immer der erste Zweig; der Modulo bleibt als exakt gleichwertiger
	// Rueckfall stehen, damit die Funktion fuer JEDE Eingabe dasselbe liefert wie vorher.
	auto wrap_ = [](const int v, const int N) -> uint {
		if(v>=0&&v<N) return (uint)v;
		if(v<0&&v>=-N) return (uint)(v+N);
		if(v>=N&&v<2*N) return (uint)(v-N);
		return (uint)((v%N+N)%N); };
	auto wx = [&](const int v) { return wrap_(v,(int)Nx); };
	auto wy = [&](const int v) { return wrap_(v,(int)Ny); };
	auto wz = [&](const int v) { return wrap_(v,(int)Nz); };
	const uint z_lo = z_per?0u:1u, z_hi = z_per?Nz:(Nz-1u);
	for(uint z=z_lo; z<z_hi; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
		const ulong n = idx(x,y,z);
		if(!ist_fluid(n)) continue;
		bool wandnah=false, ms_nah=false; int snx=0,sny=0,snz=0;
		for(uint i=1u; i<19u; i++) {
			const int zn0=(int)z+FZ_C[i][2]; if(!z_per&&(zn0<0||zn0>=(int)Nz)) continue;
			const uint zn=(uint)(z_per?(int)wz(zn0):zn0);
			const int xn=(int)x+FZ_C[i][0], yn=(int)y+FZ_C[i][1];
			const ulong nn2 = idx(wx(xn),wy(yn),zn);
			// ★ IR3-Audit HOCH 1: GEOMETRIE ungewickelt (zn0), Wrap NUR im Speicherindex --
			// vorher zerstoerte der gewickelte snz die Orientierungs-Gegenprobe am z-Saum.
			if(!wandnah&&ist_wand(nn2)) { wandnah=true; snx=xn; sny=yn; snz=zn0; }
			// ★ Kugel-Befund (Ist!=Soll, -777): Nachbarn neben BEWEGTEN Waenden werden zur Laufzeit
			// TYPE_MS und vom Zellgate ausgeschlossen -- host-seitig ist MS unsichtbar (entsteht erst
			// in initialize). Hier ueber u!=0 der Solidzelle erkennen und als eigene Klasse zaehlen.
			if((L.flags[nn2]&(TYPE_S|TYPE_E))==TYPE_S&&(L.u.x[nn2]!=0.0f||L.u.y[nn2]!=0.0f||L.u.z[nn2]!=0.0f)) ms_nah=true;
		}
		if(!wandnah) continue;
		// Stuetzpunkte: geschnittene Links (Fluid->wand_flag) aller Zellen im 5^3-Fenster.
		uint np=0u, eigene=0u, verworfen=0u; // Puffer np_max = (2R+1)^3 * 18, dynamisch (Nachpruefer B1)
		uint nq=0u; double nsx=0.0, nsy=0.0, nsz=0.0, csx=0.0, csy=0.0, csz=0.0; // V2/V3: Achslinks + Normalensumme
		for(int dz=-R; dz<=R; dz++) for(int dy=-R; dy<=R; dy++) for(int dx2=-R; dx2<=R; dx2++) {
			const int cx=(int)x+dx2, cy=(int)y+dy, cz0=(int)z+dz;
			if(!z_per&&(cz0<1||cz0>=(int)Nz-1)) continue; // z hart (Standard) bzw. periodisch (Torus)
			const uint czi=(uint)(z_per?(int)wz(cz0):cz0); // NUR fuer den Index
			const int cz=cz0; // GEOMETRIE ungewickelt (IR3-Audit HOCH 1: Saum-Stuetzpunkte lagen ±Nz daneben)
			const ulong nc = idx(wx(cx),wy(cy),czi);
			if(!ist_fluid(nc)) continue;
			for(uint i=1u; i<19u; i++) {
				const int zn0=cz+FZ_C[i][2]; if(!z_per&&(zn0<0||zn0>=(int)Nz)) continue;
				const uint zni=(uint)(z_per?(int)wz(zn0):zn0); // Index gewickelt, Geometrie (zn0) nicht
				const int zn=zn0;
				const int xn=cx+FZ_C[i][0], yn=cy+FZ_C[i][1];
				if(!ist_wand(idx(wx(xn),wy(yn),zni))) continue;
				if(np<np_max) { px[np]=0.5*((double)cx+(double)xn); py[np]=0.5*((double)cy+(double)yn); pz[np]=0.5*((double)cz+(double)zn); np++; if(dx2==0&&dy==0&&dz==0) eigene++; }
				else verworfen++; // kann bei np_max = Fenstermaximum nie greifen -- Waechter statt stiller Annahme
				if(v3_noetig&&i<7u) { // NUR Achslinks: das sind genau die Voxel-Grenzflaechen (Flaeche 1, Normale -FZ_C ins Fluid)
					const double mx=0.5*((double)cx+(double)xn), my=0.5*((double)cy+(double)yn), mz=0.5*((double)cz+(double)zn);
					if(nq<np_max) {
						// Zelle HINTER der Solidzelle, gleiche Richtung: Geometrie ungewickelt, Index gewickelt
						const int hz0=zn+FZ_C[i][2]; const int hx=xn+FZ_C[i][0], hy=yn+FZ_C[i][1];
						const bool rand = (!z_per&&(hz0<0||hz0>=(int)Nz));
						const bool hinten_solid = rand ? true : ist_wand(idx(wx(hx),wy(hy),(uint)(z_per?(int)wz(hz0):hz0)));
						qx[nq]=mx; qy[nq]=my; qz[nq]=mz; qd[nq]=(uchar)i; qt[nq]=hinten_solid?(uchar)0u:(uchar)1u; nq++;
					}
					nsx-=(double)FZ_C[i][0]; nsy-=(double)FZ_C[i][1]; nsz-=(double)FZ_C[i][2];
					csx+=mx; csy+=my; csz+=mz;
				}
			}
		}
		uchar gt=0u; float gt_koh=0.0f; // Guete DIESER Zelle (Bit 1 Rueckfall, 2 schwache Kohaerenz, 4 y_w geklemmt)
		Facette f; f.n=n; f.n_punkte=np; f.eigene_links=eigene; f.klasse=0u; f.cx_=f.cy_=f.cz_=0.0f; f.r21_=0.0f; f.r10_=0.0f;
		if(verworfen>0u) f.klasse|=32u; // Ueberlauf -- markieren+zaehlen, nicht still rechnen (A4)
		if(ms_nah) f.klasse|=64u; // bewegte-Wand-Naehe: Zellgate schloesse sie ohnehin aus -- gezaehlt statt Soll-Luecke
		if(np<6u) { f.klasse|=1u; k1++; f.nx=f.ny=f.nz=0.0f; f.yw=0.0f; f.achse=0u;
			// ★ Vergleichsvektoren MUESSEN mitwachsen, sonst verrutschen die Indizes gegen F
			// (hier faellt V1 selbst schon aus; V2/V3 werden als "keine Normale" gefuehrt).
			if(kref_an) for(int r_=0;r_<18;r_++) { (r_<3?nv1:(r_<6?nv2:(r_<9?nv3:(r_<12?nv4:(r_<15?nv5:nv6))))).push_back(0.0f); }
			// ★ Pruefagent M4: die Gates hier muessen GENAU denen im Normalpfad entsprechen,
			// sonst wachsen die Vektoren unterschiedlich weit und die Kreuztabellen verrutschen.
			// ★ Audit E 29.08.: yw5v/kl5v standen hier unter v3_noetig, im Normalpfad aber
			// UNGEGATET -- bei NORMQUELLE=1 ohne VERGLEICH waeren alle Vergleichsvektoren um
			// die Zahl der K1-Facetten kuerzer als F gewesen. Alle Leser stehen unter vgl_an,
			// also gaten jetzt BEIDE Seiten auf vgl_an. (Am Fahrzeug K1=0, daher bisher folgenlos.)
			if(vgl_an) { yw5v.push_back(0.0f); kl5v.push_back((uchar)1u);
				yw6v.push_back(0.0f); kl6v.push_back((uchar)1u);
				a12.push_back(-1.0f); a13.push_back(-1.0f); a14.push_back(-1.0f); yw2v.push_back(0.0f); yw3v.push_back(0.0f); yw4v.push_back(0.0f); yw4nv.push_back(0.0f); yw4av.push_back(0.0f); kl2v.push_back((uchar)1u); kl3v.push_back((uchar)1u); kl4v.push_back((uchar)1u); kl1v.push_back(f.klasse); }
			gt_v.push_back((uchar)0u); koh_v.push_back(0.0f); // Guete parallel zu F halten (Audit E)
			F.push_back(f); continue; }
		double cx=0.0, cy=0.0, cz=0.0;
		for(uint i=0u;i<np;i++) { cx+=px[i]; cy+=py[i]; cz+=pz[i]; }
		cx/=np; cy/=np; cz/=np;
		f.cx_=(float)cx; f.cy_=(float)cy; f.cz_=(float)cz;
		double M[3][3]={{0,0,0},{0,0,0},{0,0,0}};
		for(uint i=0u;i<np;i++) {
			const double dxp=px[i]-cx, dyp=py[i]-cy, dzp=pz[i]-cz;
			M[0][0]+=dxp*dxp; M[0][1]+=dxp*dyp; M[0][2]+=dxp*dzp;
			M[1][1]+=dyp*dyp; M[1][2]+=dyp*dzp; M[2][2]+=dzp*dzp;
		}
		M[1][0]=M[0][1]; M[2][0]=M[0][2]; M[2][1]=M[1][2];
		double ew[3], ev[3][3]; jacobi3(M, ew, ev);
		int imin=0, imax=0;
		for(int i=1;i<3;i++) { if(ew[i]<ew[imin]) imin=i; if(ew[i]>ew[imax]) imax=i; }
		const int imid = 3-imin-imax==3 ? 0 : 3-imin-imax; // bei imin==imax (entartet) -> 0
		double nxd=ev[0][imin], nyd=ev[1][imin], nzd=ev[2][imin];
		const double nl = sqrt(nxd*nxd+nyd*nyd+nzd*nzd);
		nxd/=nl; nyd/=nl; nzd/=nl;
		// Orientierung ins Fluid: y_w = n*(zelle-c) > 0; Gegenprobe gegen den Solidnachbarn.
		double yw = nxd*((double)x-cx)+nyd*((double)y-cy)+nzd*((double)z-cz);
		if(yw<0.0) { nxd=-nxd; nyd=-nyd; nzd=-nzd; yw=-yw; }
		const double gegen = nxd*((double)x-(double)snx)+nyd*((double)y-(double)sny)+nzd*((double)z-(double)snz);
		if(gegen<=0.0) { f.klasse|=16u; kori++; }
		// Konditionsklassen: Schwellen VORLAEUFIG (werden aus den Histogrammen dieses Laufs bestimmt).
		const double r21 = ew[imax]>1e-30 ? ew[imin]/fmax(ew[imid],1e-30) : 1.0;   // K2: Kante
		const double r10 = ew[imax]>1e-30 ? ew[imid]/ew[imax] : 0.0;               // K3: Linie
		f.r21_=(float)r21; f.r10_=(float)r10;
		if(r21>kante_r21) { f.klasse|=2u; k2++; } // Schwelle geeicht (Fenster-A/B): saubere Population endet bei q80=0,10, Kantenschwanz beginnt dahinter
		if(r10<0.02) { f.klasse|=4u; k3++; }
		// ★ KLEMMEN statt VERWERFEN, auch im V1-Pfad -- sonst waere der Schalter kein
		// einvariabler A/B, sondern zusaetzlich ein V1/V3b-Unterschied.
		if(yw_klemme&&yw<(double)yw_min) { yw=(double)yw_min; f.yw=(float)yw; gt|=(uchar)4u; n_geklemmt++; }
		if(yw<(double)yw_min||yw>2.0) { f.klasse|=8u; k4++; }
		f.nx=(float)nxd; f.ny=(float)nyd; f.nz=(float)nzd; f.yw=(float)yw;
		const double ax=fabs(nxd), ay=fabs(nyd), az=fabs(nzd);
		f.achse = (ax>=ay&&ax>=az) ? 0u : ((ay>=az) ? 1u : 2u); // Tie-Break: kleinste Achsnummer
		if(v3_noetig) { // ---- V2 (Achslinks-PCA) und V3 (Heikos Normalensumme), rein diagnostisch
			double n2x=0.0,n2y=0.0,n2z=0.0, yw2=0.0; uchar k2b=0u;
			// ★ Audit-Performance 29.08.: V2 ist REIN DIAGNOSTISCH -- alle Leser stehen unter
			// vgl_an. Gerechnet wurde es aber unter v3_noetig, also auch im Produktionsarm:
			// eine zweite vollstaendige jacobi3-Eigenzerlegung je Facette, bei 4 mm 3,28 Mio mal.
			if(!vgl_an) k2b|=1u;
			else if(nq<6u) k2b|=1u; // K1-Aequivalent: zu wenig Stuetzpunkte
			else {
				double bx=0.0,by=0.0,bz=0.0;
				for(uint i=0u;i<nq;i++) { bx+=qx[i]; by+=qy[i]; bz+=qz[i]; }
				bx/=nq; by/=nq; bz/=nq;
				double M2[3][3]={{0,0,0},{0,0,0},{0,0,0}};
				for(uint i=0u;i<nq;i++) {
					const double dxp=qx[i]-bx, dyp=qy[i]-by, dzp=qz[i]-bz;
					M2[0][0]+=dxp*dxp; M2[0][1]+=dxp*dyp; M2[0][2]+=dxp*dzp;
					M2[1][1]+=dyp*dyp; M2[1][2]+=dyp*dzp; M2[2][2]+=dzp*dzp;
				}
				M2[1][0]=M2[0][1]; M2[2][0]=M2[0][2]; M2[2][1]=M2[1][2];
				double e2[3], v2[3][3]; jacobi3(M2, e2, v2);
				int i2n=0,i2x=0; for(int i=1;i<3;i++){ if(e2[i]<e2[i2n]) i2n=i; if(e2[i]>e2[i2x]) i2x=i; }
				const int i2m = 3-i2n-i2x==3 ? 0 : 3-i2n-i2x;
				n2x=v2[0][i2n]; n2y=v2[1][i2n]; n2z=v2[2][i2n];
				const double l2=sqrt(n2x*n2x+n2y*n2y+n2z*n2z); n2x/=l2; n2y/=l2; n2z/=l2;
				yw2 = n2x*((double)x-bx)+n2y*((double)y-by)+n2z*((double)z-bz);
				if(yw2<0.0) { n2x=-n2x; n2y=-n2y; n2z=-n2z; yw2=-yw2; }
				const double r21b = e2[i2x]>1e-30 ? e2[i2n]/fmax(e2[i2m],1e-30) : 1.0;
				const double r10b = e2[i2x]>1e-30 ? e2[i2m]/e2[i2x] : 0.0;
				if(r21b>0.15) k2b|=2u;
				if(r10b<0.02) k2b|=4u;
				if(yw2<(double)yw_min||yw2>2.0) k2b|=8u;
			}
			// naechste Flaeche OHNE Sichtbarkeitsfilter (fuer V3b) und ihre eigene Normale
			double d6=1e30, c6x=0.0,c6y=0.0,c6z=0.0, n6nx=0.0,n6ny=0.0,n6nz=0.0;
			for(uint j=0u;j<nq;j++) {
				const int dj=(int)qd[j];
				const double ux=(double)x-qx[j], uy=(double)y-qy[j], uz=(double)z-qz[j];
				const double dd=ux*ux+uy*uy+uz*uz;
				if(dd<d6) { d6=dd; c6x=qx[j]; c6y=qy[j]; c6z=qz[j];
					n6nx=-(double)FZ_C[dj][0]; n6ny=-(double)FZ_C[dj][1]; n6nz=-(double)FZ_C[dj][2]; }
			}
			double n3x=0.0,n3y=0.0,n3z=0.0, yw3=0.0; uchar k3b=0u;
			const double l3=sqrt(nsx*nsx+nsy*nsy+nsz*nsz);
			if(nq==0u||l3<1e-12) k3b|=1u; // keine Flaeche ODER Vektorsumme hebt sich auf (geschlossene Duennstelle)
			else {
				n3x=nsx/l3; n3y=nsy/l3; n3z=nsz/l3;
				const double gx=csx/(double)nq, gy=csy/(double)nq, gz=csz/(double)nq;
				yw3 = n3x*((double)x-gx)+n3y*((double)y-gy)+n3z*((double)z-gz);
				if(yw3<0.0) { n3x=-n3x; n3y=-n3y; n3z=-n3z; yw3=-yw3; }
				if(yw3<(double)yw_min||yw3>2.0) k3b|=8u;
			}
			// ---- V4: HEIKOS VERFAHREN VERBESSERT. Zwei Aenderungen gegen V3, beide rein geometrisch:
			//   (1) SICHTBARKEIT: eine Voxelflaeche zaehlt nur, wenn die Zelle VOR ihr liegt,
			//       also n_i*(Zelle - c_i) > 0. Die Rueckseite eines einzelligen Blechs zeigt
			//       exakt entgegengesetzt und loeschte in V3 die Summe aus (V3-K1 = 2.233 bei
			//       8 mm). Mit dem Filter kann das konstruktiv nicht mehr passieren.
			//   (2) ABSTANDSGEWICHT w = 1/(1+d^2): die direkt angrenzende Flaeche (d=0,5) wiegt
			//       0,80, eine aus der Fensterecke (d~2,1) nur 0,18. V3 wog beide gleich, was
			//       die Normale von entfernter Geometrie mitziehen liess.
			// y_w wird ZWEIFACH gemeldet: gegen den gewichteten Schwerpunkt (wie V3) und gegen
			// die NAECHSTE sichtbare Flaeche. Der zweite Weg ist der Kandidat gegen K4 -- in
			// einer konkaven Ecke ist der wahre Wandabstand der zur naechsten Wand, nicht zum
			// Schwerpunkt aller umliegenden. Beide Zahlen nebeneinander, damit die Wahl belegt
			// statt gesetzt wird.
			double n4x=0.0,n4y=0.0,n4z=0.0, gx4=0.0,gy4=0.0,gz4=0.0, wsum=0.0; uint nsicht=0u, nduenn=0u;
			double n5x=0.0,n5y=0.0,n5z=0.0; // V5: gleiche Sichtbarkeit, Gewicht 1
			double dnah=1e30, cnx=0.0,cny=0.0,cnz=0.0, nnx=0.0,nny=0.0,nnz=0.0;
			for(uint j=0u;j<nq;j++) {
				const int di=(int)qd[j];
				const double nix=-(double)FZ_C[di][0], niy=-(double)FZ_C[di][1], niz=-(double)FZ_C[di][2];
				const double vx=(double)x-qx[j], vy=(double)y-qy[j], vz=(double)z-qz[j];
				if(nix*vx+niy*vy+niz*vz<=0.0) continue; // Rueckseite -- fuer diese Zelle unsichtbar
				const double d2=vx*vx+vy*vy+vz*vz, w=1.0/(1.0+d2);
				n4x+=w*nix; n4y+=w*niy; n4z+=w*niz;
				n5x+=nix; n5y+=niy; n5z+=niz;
				gx4+=w*qx[j]; gy4+=w*qy[j]; gz4+=w*qz[j]; wsum+=w; nsicht++;
				if(qt[j]) nduenn++;
				if(d2<dnah) { dnah=d2; cnx=qx[j]; cny=qy[j]; cnz=qz[j]; nnx=nix; nny=niy; nnz=niz; }
			}
			if(nduenn>0u) v4_duenn++;
			double yw4=0.0, yw4n=0.0, yw4a=0.0; uchar k4b=0u;
			const double l4=sqrt(n4x*n4x+n4y*n4y+n4z*n4z);
			if(nsicht==0u) k4b|=1u; // gar keine sichtbare Flaeche -- kein Rueckfall moeglich
			else if(l4<1e-12) {
				// ★ SONDERBEHANDLUNG DUENNTEIL (Heiko 28.08.): die gewichtete Summe ist entartet.
				// Das passiert genau dort, wo sich gegenueberliegende sichtbare Flaechen aufheben --
				// also an einzelligen Teilen und einzelligen Spalten. Statt zu verwerfen wird die
				// NAECHSTE sichtbare Flaeche genommen; ihre Normale ist dort die einzig sinnvolle.
				v4_rueck++;
				n4x=nnx; n4y=nny; n4z=nnz;
				yw4  = n4x*((double)x-cnx)+n4y*((double)y-cny)+n4z*((double)z-cnz);
				yw4n = yw4; yw4a = yw4;
				if(yw4<0.0) { n4x=-n4x; n4y=-n4y; n4z=-n4z; yw4=-yw4; yw4n=-yw4n; }
				if(yw4<(double)yw_min) { k4b|=8u; v4_unten++; } else if(yw4>2.0) { k4b|=8u; v4_oben++; }
				if(yw4n<(double)yw_min||yw4n>2.0) k4b|=16u;
			}
			else {
				n4x/=l4; n4y/=l4; n4z/=l4;
				gx4/=wsum; gy4/=wsum; gz4/=wsum;
				yw4  = n4x*((double)x-gx4)+n4y*((double)y-gy4)+n4z*((double)z-gz4);
				yw4n = n4x*((double)x-cnx)+n4y*((double)y-cny)+n4z*((double)z-cnz);
				if(yw4<0.0) { n4x=-n4x; n4y=-n4y; n4z=-n4z; yw4=-yw4; yw4n=-yw4n; }
				if(yw4<(double)yw_min) k4b|=8u; else if(yw4>2.0) k4b|=8u;   // Bit 8 = K4 nach Schwerpunkt
				if(yw4n<(double)yw_min) { k4b|=16u; v4_unten++; }            // Bit 16 = K4 nach naechster Flaeche
				else if(yw4n>2.0) { k4b|=16u; v4_oben++; }                   // getrennt gezaehlt: Unter- gegen Obergrenze
				// ★ V4c: y_w AUSGERICHTET. Die Obergrenze feuert nie (gemessen: 0), es ist immer
				// die Untergrenze. Ursache: die NAECHSTE Flaeche muss nicht die sein, welche die
				// Konsensnormale traegt. In einer konkaven Ecke ist die naechste Flaeche z. B. der
				// Boden, waehrend n4 von vielen Seitenflaechen dominiert wird -- dann steht
				// (Zelle - c_naechste) fast senkrecht auf n4 und die Projektion geht gegen null.
				// Abhilfe ohne neue Konstante: y_w als gewichtetes Mittel ueber die sichtbaren
				// Flaechen, GEWICHTET MIT DER AUSRICHTUNG (n_i*n4)_+ -- Flaechen, die die
				// Konsensnormale tragen, bestimmen den Wandabstand, querstehende nicht.
				// ★ Audit-Performance 29.08.: V4c ist rein diagnostisch (widerlegt, B47) und war
				// der zweite O(nq)-Lauf je Facette -- jetzt nur noch fuer den Vergleichsbericht.
				if(!vgl_an) yw4a = yw4n;
				else {	double zs=0.0, ns_=0.0;
					for(uint j2=0u;j2<nq;j2++) {
						const int dj=(int)qd[j2];
						const double jx=-(double)FZ_C[dj][0], jy=-(double)FZ_C[dj][1], jz=-(double)FZ_C[dj][2];
						const double ux=(double)x-qx[j2], uy=(double)y-qy[j2], uz=(double)z-qz[j2];
						if(jx*ux+jy*uy+jz*uz<=0.0) continue;                  // dieselbe Sichtbarkeit wie oben
						const double al=jx*n4x+jy*n4y+jz*n4z; if(al<=0.0) continue; // Ausrichtung
						const double wj=al/(1.0+(ux*ux+uy*uy+uz*uz));
						zs+=wj*(n4x*ux+n4y*uy+n4z*uz); ns_+=wj;
					}
					yw4a = (ns_>1e-12) ? zs/ns_ : yw4n;
					if(yw4a<(double)yw_min||yw4a>2.0) k4b|=32u;               // Bit 32 = K4 nach Ausrichtung
				}
			}
			// ---- V5 auswerten: identische Sichtbarkeit und identischer Rueckfall wie V4,
			// aber ALLE sichtbaren Flaechen gleich gewichtet. y_w wie V4b (naechste Flaeche).
			double yw5=0.0; uchar k5b=0u;
			// ★ Audit-Performance 29.08.: V5 ist rein diagnostisch (isolierte den Sichtbarkeits-
			// filter als Schaden, B45) -- ausserhalb des Vergleichsberichts nicht mehr rechnen.
			if(!vgl_an) k5b|=1u;
			else {	const double l5=sqrt(n5x*n5x+n5y*n5y+n5z*n5z);
				if(nsicht==0u) k5b|=1u;
				else {
					if(l5<1e-12) { n5x=nnx; n5y=nny; n5z=nnz; }   // derselbe Duennteil-Rueckfall
					else { n5x/=l5; n5y/=l5; n5z/=l5; }
					yw5 = n5x*((double)x-cnx)+n5y*((double)y-cny)+n5z*((double)z-cnz);
					if(yw5<0.0) { n5x=-n5x; n5y=-n5y; n5z=-n5z; yw5=-yw5; }
					if(yw5<(double)yw_min||yw5>2.0) k5b|=8u;
				}
			}
			// ---- V3b auswerten
			double n6x=n3x, n6y=n3y, n6z=n3z, yw6=0.0; uchar k6b=0u;
			{	const double l6=sqrt(nsx*nsx+nsy*nsy+nsz*nsz);
				if(nq==0u) k6b|=1u;
				else {
					const double koh = (nq>0u) ? l6/(double)nq : 0.0; // Kohaerenz der Flaechensumme
				// ★ Pruefagent: 0,25 war ein Handwert. Herleitbar ohne neue Konstante: die
				// Flaechennormalen stammen aus sechs Achsrichtungen; liegt KEIN gegenlaeufiges
				// Paar vor, ist |Summe|/nq >= 1/sqrt(3) (Minimum bei gleich vielen Flaechen in
				// drei Richtungen). koh < 1/sqrt(3) BEWEIST also Ausloeschung.
				if(v3_noetig) { v3b_koh.push_back(koh); if(koh<0.5773502692) v3b_schwach++; }
				gt_koh = (float)koh; if(koh<0.5773502692) gt |= (uchar)2u;
				if(l6<1e-12) { n6x=n6nx; n6y=n6ny; n6z=n6nz; v6_rueck++; gt |= (uchar)1u; } // Duennteil-Rueckfall
					else { n6x=nsx/l6; n6y=nsy/l6; n6z=nsz/l6; }
					yw6 = n6x*((double)x-c6x)+n6y*((double)y-c6y)+n6z*((double)z-c6z);
					if(yw6<0.0) { n6x=-n6x; n6y=-n6y; n6z=-n6z; yw6=-yw6; }
					if(yw6<(double)yw_min||yw6>2.0) k6b|=8u;
				}
			}
			auto wink=[&](const double bx,const double by,const double bz){
				const double c=fabs(nxd*bx+nyd*by+nzd*bz);
				return (float)(acos(fmin(1.0,c))*180.0/3.14159265358979); };
			if(vgl_an) { a14.push_back((k4b&1u)?-1.0f:wink(n4x,n4y,n4z));
				yw4v.push_back((float)yw4); yw4nv.push_back((float)yw4n); yw4av.push_back((float)yw4a); kl4v.push_back(k4b); }
			if(kref_an) {
				nv1.push_back((float)nxd); nv1.push_back((float)nyd); nv1.push_back((float)nzd);
				nv2.push_back((float)n2x); nv2.push_back((float)n2y); nv2.push_back((float)n2z);
				nv3.push_back((float)n3x); nv3.push_back((float)n3y); nv3.push_back((float)n3z);
				nv4.push_back((float)n4x); nv4.push_back((float)n4y); nv4.push_back((float)n4z);
				nv5.push_back((float)n5x); nv5.push_back((float)n5y); nv5.push_back((float)n5z);
				nv6.push_back((float)n6x); nv6.push_back((float)n6y); nv6.push_back((float)n6z);
			}
			if(vgl_an) { yw6v.push_back((float)yw6); kl6v.push_back(k6b); kl1v.push_back(f.klasse); }
			// ---- V3b AKTIV SCHALTEN: nur hier aendert sich, was in der Facette landet.
			if(normquelle==1u) {
				nq_aktiv++;
				// ★ Pruefagent: Bit 2 (Kante) und Bit 4 (Linie) bleiben JETZT STEHEN. Sie stammen
				// aus der PCA, die auch unter V3b vollstaendig durchlaeuft, und sind eine Aussage
				// ueber die PUNKTWOLKE -- unabhaengig davon, woher die Normale kommt. Sie hier zu
				// loeschen war der Grund, warum der A/B nicht einvariabel war.
				const float pnx_=f.nx, pny_=f.ny, pnz_=f.nz, pyw_=f.yw; // V1-Zustand fuer den Wirkpfad-Nachweis
				f.klasse &= (uchar)~(uchar)(1u|8u|16u);       // nur K1, K4 und Orientierung neu bewerten
				f.klasse |= (uchar)(k6b&(uchar)1u);           // K1 aus V3b; K4 unten aus dem TATSAECHLICH gespeicherten y_w
				double e6x=n6x, e6y=n6y, e6z=n6z;             // effektive Normale -- ggf. gekippt (Audit C)
				if(ywquelle>0u) { // ★ Pruefagent B-1: hier fehlte die Klemme, und Bit 8 kam aus dem
					// UNGEKLEMMTEN yw6 -- mit CFD_FACETTEN_YWQUELLE=1 waere YWKLEMME ein stiller No-Op gewesen.
					double yw6k = yw6;
					if(yw_klemme&&yw6k<(double)yw_min) { yw6k=(double)yw_min; gt|=(uchar)4u; }
					f.yw=(float)yw6k; f.cx_=(float)c6x; f.cy_=(float)c6y; f.cz_=(float)c6z;
					if(yw6k<(double)yw_min||yw6k>2.0) f.klasse|=8u; } // aus dem GESPEICHERTEN y_w, nicht aus k6b
				else { // y_w bleibt V1s Groesse: gegen den PCA-Schwerpunkt, aber mit der NEUEN Normalen
					// ★ Audit C+D 29.08.: hier stand fabs(ywp). Das liess die NORMALE stehen und
					// meldete die Wand auf der falschen Seite -- f.yw und f.n konnten gegenlaeufig
					// sein, und fac_geo[8k+3] speist sowohl die Wandfunktion als auch ELIBB-q.
					// Zweiter Fehler: K4 wurde aus k6b Bit 8 gesetzt, das aus yw6 (Ankermass der
					// naechsten Flaeche) stammt -- entschieden wurde also ueber eine ANDERE Groesse
					// als die gespeicherte. Jetzt: Vorzeichen durch KIPPEN, K4 aus ywp selbst.
					double ywp = n6x*((double)x-cx)+n6y*((double)y-cy)+n6z*((double)z-cz);
					if(ywp<0.0) { e6x=-e6x; e6y=-e6y; e6z=-e6z; ywp=-ywp; nq_kipp++; }
					// ★ KLEMMEN statt VERWERFEN (Heiko 29.08.), nur unter CFD_FACETTEN_YWKLEMME.
					if(yw_klemme&&ywp<(double)yw_min) { ywp=(double)yw_min; gt|=(uchar)4u; n_geklemmt++; }
					f.yw=(float)ywp; /* cx_/cy_/cz_ bleiben der PCA-Schwerpunkt */
					if(ywp<(double)yw_min||ywp>2.0) f.klasse|=8u; } // EINZIGER Abnehmer: die
				// y_w-Neuberechnung der Glaettung (setup.cpp ~1619). Stuende hier der PCA-Schwerpunkt,
				// wuerde die Glaettung V3bs y_w still gegen einen fremden Bezug ueberschreiben.
				// r21_/r10_ bleiben die PCA-Werte -- sie beschreiben die Punktwolke, nicht die Normale.
				// ★ KANTENTEST AUS V3bs EIGENEM GUETEMASS (Heiko 29.08.), nur unter
				// CFD_FACETTEN_KANTE_KOH. r21 beschreibt die Guete des PCA-Fits -- unter V3b
				// benutzen wir diesen Fit gar nicht mehr. Das eigene Mass ist die Kohaerenz.
				if(kante_koh) { const uchar k2_alt=(uchar)(f.klasse&2u);
					f.klasse &= (uchar)~2u; if(gt&2u) f.klasse|=2u;
					if((uchar)(f.klasse&2u)!=k2_alt) n_kante_koh++; }
				f.nx=(float)e6x; f.ny=(float)e6y; f.nz=(float)e6z;
				const double geg6 = e6x*((double)x-(double)snx)+e6y*((double)y-(double)sny)+e6z*((double)z-(double)snz);
				if(geg6<=0.0) f.klasse|=16u;                  // Orientierungs-Gegenprobe bleibt -- gegen die GEKIPPTE Normale
				const double q6x=fabs(e6x), q6y=fabs(e6y), q6z=fabs(e6z);
				f.achse = (q6x>=q6y&&q6x>=q6z) ? 0u : ((q6y>=q6z) ? 1u : 2u);
				if(f.nx!=pnx_||f.ny!=pny_||f.nz!=pnz_) nq_n_geaendert++;
				if(f.yw!=pyw_) nq_yw_geaendert++;
			}
			if(vgl_an) { yw5v.push_back((float)yw5); kl5v.push_back(k5b);
				a12.push_back((k2b&1u)?-1.0f:wink(n2x,n2y,n2z));
				a13.push_back((k3b&1u)?-1.0f:wink(n3x,n3y,n3z));
				yw2v.push_back((float)yw2); yw3v.push_back((float)yw3);
				kl2v.push_back(k2b); kl3v.push_back(k3b); }
			// ★ Pruefagent H1: der y_w-ANKER stand INNERHALB des normquelle-Blocks und war ohne
			// NORMQUELLE=1 ein stiller No-Op -- der Arm, der ihn allein messen sollte, haette
			// garantiert die Zahlen des Bezugsarms geliefert und daraus "der Anker kostet nichts"
			// belegt. Jetzt quellenunabhaengig, mit eigenem Wirkpfad-Zaehler.
			// Vorzeichen wie bei V1 durch KIPPEN der Normalen, nicht per fabs (H2).
			if(ywquelle>0u&&!(f.klasse&1u)) {
				yw_aktiv++;
				const float qnx_=f.nx, qny_=f.ny, qnz_=f.nz, qyw_=f.yw;
				double ywn = (double)f.nx*((double)x-c6x)+(double)f.ny*((double)y-c6y)+(double)f.nz*((double)z-c6z);
				if(ywn<0.0) { f.nx=-f.nx; f.ny=-f.ny; f.nz=-f.nz; ywn=-ywn; yw_kipp++; }
				// ★ Pruefagent B-1: dieser Block ueberschrieb f.yw und setzte Bit 8 ungeklemmt --
				// er loeschte damit auch die Klemme aus dem V1-Pfad wieder.
				if(yw_klemme&&ywn<(double)yw_min) { ywn=(double)yw_min; gt|=(uchar)4u; }
				f.yw=(float)ywn; f.cx_=(float)c6x; f.cy_=(float)c6y; f.cz_=(float)c6z;
				f.klasse &= (uchar)~8u; if(ywn<(double)yw_min||ywn>2.0) f.klasse|=8u;
				// ★ Audit C2 29.08.: das Kippen ueberschreibt die Orientierung, die vorher durch
				// die Gegenprobe gegen den Wandnachbarn validiert war -- Bit 16 blieb aber stehen,
				// wie es war. Also die Gegenprobe nach dem Kippen WIEDERHOLEN.
				const double geg_n = (double)f.nx*((double)x-(double)snx)+(double)f.ny*((double)y-(double)sny)+(double)f.nz*((double)z-(double)snz);
				const uchar ori_vor = (uchar)(f.klasse&16u);
				f.klasse &= (uchar)~16u; if(geg_n<=0.0) f.klasse|=16u;
				if((uchar)(f.klasse&16u)!=ori_vor) yw_ori_neu++;
				if(f.nx!=qnx_||f.ny!=qny_||f.nz!=qnz_) yw_n_geaendert++;
				if(f.yw!=qyw_) yw_yw_geaendert++;
			}
		}
		gt_v.push_back(gt); koh_v.push_back(gt_koh); // Guete parallel zu F halten (Audit E)
		F.push_back(f);
		// Erstpass-Histogramme entfernt (Gross-Audit N18): wurden vor jeder Nutzung geleert -- der Endzustands-Pass fuellt neu.
	}
	// Normalen-Glaettung (A6): 1 Pass, flaechengewichtet (w = n_punkte), 3^3-Nachbarfacetten.
	{
		// ★ Audit-Performance: das Indexfeld war ueber die GANZE Domaene angelegt --
		// bei 8 mm 845x317x241 = 64,6 Mio Eintraege = 258 MB, bei 4 mm rund 2,06 GB, obwohl nur
		// die wandnahen Zellen je vorkommen. lbm.cpp macht es fuer fac_idx laengst richtig und
		// meldet dafuer "0.52 GB gespart". Hier dieselbe Bounding-Box, ueber die Facetten selbst
		// bestimmt (eine Facette liegt IMMER an der Wand, die BBox ist also eng).
		// ★ Audit H 29.08.: bei leerem F blieb bx0=Nx, bx1=0 -- (bx1-bx0+1u) lief als uint
		// auf ~4,29e9 um und die Allokation warf, wo der alte Voll-Domaenen-Index harmlos war.
		if(F.empty()) print_warning("Glaettung: keine Facetten vorhanden -- uebersprungen.");
		else {
		uint bx0=Nx,by0=Ny,bz0=Nz,bx1=0u,by1=0u,bz1=0u;
		for(const Facette& fb : F) {
			const uint zc=(uint)(fb.n/((ulong)Nx*(ulong)Ny)), yc=(uint)((fb.n/(ulong)Nx)%(ulong)Ny), xc=(uint)(fb.n%(ulong)Nx);
			bx0=min(bx0,xc); by0=min(by0,yc); bz0=min(bz0,zc);
			bx1=max(bx1,xc); by1=max(by1,yc); bz1=max(bz1,zc);
		}
		const ulong bnx=(ulong)(bx1-bx0+1u), bny=(ulong)(by1-by0+1u), bnz=(ulong)(bz1-bz0+1u);
		std::vector<uint> feld(bnx*bny*bnz, 0xFFFFFFFFu);
		auto bidx=[&](const uint x_,const uint y_,const uint z_)->ulong {
			return (ulong)(x_-bx0)+((ulong)(y_-by0)+(ulong)(z_-bz0)*bny)*bnx; };
		auto in_box=[&](const uint x_,const uint y_,const uint z_) {
			return x_>=bx0&&x_<=bx1&&y_>=by0&&y_<=by1&&z_>=bz0&&z_<=bz1; };
		print_info("  Glaettungsindex ueber die Facetten-BBox "+to_string(bnx)+"x"+to_string(bny)+"x"+to_string(bnz)
			+" = "+to_string((ulong)(bnx*bny*bnz*4ull/1048576ull))+" MB statt "+to_string((ulong)((ulong)Nx*Ny*Nz*4ull/1048576ull))+" MB ueber die Domaene.");
		for(uint i=0u; i<(uint)F.size(); i++) {
			const uint zc=(uint)(F[i].n/((ulong)Nx*(ulong)Ny)), yc=(uint)((F[i].n/(ulong)Nx)%(ulong)Ny), xc=(uint)(F[i].n%(ulong)Nx);
			feld[bidx(xc,yc,zc)]=i;
		}
		std::vector<Facette> G=F;
		for(uint i=0u; i<(uint)F.size(); i++) {
			if(F[i].klasse&1u) continue;
			uint x,y,z; L.coordinates(F[i].n, x, y, z);
			double sx=0.0, sy=0.0, sz=0.0;
			for(int dz=-1; dz<=1; dz++) for(int dy=-1; dy<=1; dy++) for(int dx2=-1; dx2<=1; dx2++) {
				const int cx2=(int)x+dx2, cy2=(int)y+dy, cz20=(int)z+dz;
				if(!z_per&&(cz20<0||cz20>=(int)Nz)) continue;
				const uint cz2=(uint)(z_per?(int)wz(cz20):cz20);
				const uint wxx=wx(cx2), wyy=wy(cy2);
				if(!in_box(wxx,wyy,cz2)) continue; // ausserhalb der Facetten-BBox gibt es keine Facette
				const uint j = feld[bidx(wxx,wyy,cz2)];
				if(j==0xFFFFFFFFu||(F[j].klasse&1u)) continue;
				const double w = (double)max(1u, F[j].eigene_links); // Flaechenproxy der FACETTENZELLE (Nachpruefer-Randnotiz: Fenstersummen ueberlappen fast vollstaendig -> de facto uniform)
				sx+=w*F[j].nx; sy+=w*F[j].ny; sz+=w*F[j].nz;
			}
			const double l = sqrt(sx*sx+sy*sy+sz*sz);
			if(l>1e-12) { G[i].nx=(float)(sx/l); G[i].ny=(float)(sy/l); G[i].nz=(float)(sz/l);
				const double ax=fabs(sx/l), ay=fabs(sy/l), az=fabs(sz/l);
				G[i].achse = (ax>=ay&&ax>=az) ? 0u : ((ay>=az) ? 1u : 2u);
				// ★ Nachpruefer B3: y_w gegen die GEGLAETTETE Ebene neu (Plan A6) -- Schwerpunkt liegt in der Facette.
				double ywn = (sx/l)*((double)x-(double)F[i].cx_)+(sy/l)*((double)y-(double)F[i].cy_)+(sz/l)*((double)z-(double)F[i].cz_);
				if(ywn<0.0) G[i].klasse|=16u; // Audit R3: Orientierungskipp durch die Glaettung -- markieren, fabs darf ihn nicht verdecken
				G[i].yw = (float)fabs(ywn);
				if(yw_klemme&&G[i].yw<yw_min) { G[i].yw=yw_min; gt_v[i]|=(uchar)4u; n_geklemmt++; } // klemmen statt verwerfen
				G[i].klasse &= (uchar)~8u; if(G[i].yw<yw_min||G[i].yw>2.0f) G[i].klasse|=8u; // K4 neu bewerten
			}
		}
		F=std::move(G); // ★ Audit: war Copy-Assignment -- 56 B je Facette, bei 4 mm 183 MB umsonst
		} // Ende des nicht-leeren Zweigs (Audit H)
	}
	// ★ Nachpruefer B4: Histogramme aus dem ENDzustand (nach Glaettung) -- Konsole und CSV sehen
	// dieselbe Population; B2: "markiert" zaehlt ZELLEN mit klasse!=0, nicht die Zaehlersumme.
	hist_yw.clear(); hist_winkel.clear();
	ulong markiert=0ull; k4=0ull; kori=0ull; ulong k_ueberlauf=0ull, k_ms=0ull;
	k1=0ull; k2=0ull; k3=0ull; // ★ auch K1..K3 aus der ENDbitmaske -- sonst meldete der Zensus bei
	// NORMQUELLE=1 weiter die Inline-Zaehler des V1-Pfades, obwohl die Bits geloescht sind. // kori seit R3-Nachschliff aus der ENDbitmaske (Glaettungs-Kipp zaehlt mit; Bit 16 ist bewusst sticky)
	for(const Facette& f : F) {
		if(f.klasse!=0u) markiert++;
		if(f.klasse&1u)  k1++;
		if(f.klasse&2u)  k2++;
		if(f.klasse&4u)  k3++;
		if(f.klasse&8u)  k4++;
		if(f.klasse&32u) k_ueberlauf++;
		if(f.klasse&64u) k_ms++;
		if(f.klasse&16u) kori++;
		if(f.klasse&1u) continue;
		hist_yw.push_back((double)f.yw);
		hist_winkel.push_back(acos(fmin(1.0,(double)fmax(fabs(f.nx),fmax(fabs(f.ny),fabs(f.nz)))))*180.0/3.14159265358979);
	}
	// Bericht + Histogramm-CSV.
	// ★ Audit-Performance 29.08.: das Lambda sortierte bei JEDEM Aufruf. Elf Aufrufe ueber
	// je bis zu 3,28 Mio double bedeuteten zehn Sortierungen von bereits Sortiertem.
	// Jetzt einmal sortieren (sortiere), danach nur noch indizieren.
	auto sortiere = [](std::vector<double>& v) { std::sort(v.begin(), v.end()); };
	auto quantil = [](const std::vector<double>& v, const double q) {
		if(v.empty()) return 0.0;
		return v[(size_t)fmin((double)v.size()-1.0, q*(double)v.size())]; };
	const ulong nf=(ulong)F.size();
	if(v3_noetig&&!v3b_koh.empty()) {
		std::sort(v3b_koh.begin(), v3b_koh.end());
		auto q=[&](const double t){ return v3b_koh[(size_t)fmin((double)v3b_koh.size()-1.0, t*(double)v3b_koh.size())]; };
		print_info("  V3b-KOHAERENZ |Summe|/nq: Median "+to_string((float)q(0.5),3u)+", q10 "+to_string((float)q(0.1),3u)
			+", q01 "+to_string((float)q(0.01),3u)+" -- unter 1/sqrt(3)=0,577 (beweist Ausloeschung, hergeleitet aus den sechs Achsrichtungen): "
			+to_string(v3b_schwach)+" = "+to_string(100.0f*(float)v3b_schwach/(float)v3b_koh.size(),2u)+" %");
		if(normquelle==1u&&v3b_schwach*20ull>(ulong)v3b_koh.size())
			print_warning("V3b: mehr als 5 % der Facetten haben eine schwach kohaerente Flaechensumme -- ihre Normalenrichtung ist dort ueberwiegend Ausloeschungsrest. Die betroffenen Zellen sitzen erfahrungsgemaess an Duennteilen.");
	}
	// ★ Audit F 29.08.: geprueft wird jetzt die WIRKUNG (Vergleich gegen den V1-Zustand
	// unmittelbar vor dem Schreiben), nicht der Eintritt in den Block. Der alte Waechter
	// konnte nur den Totalausfall sehen und meldete sonst tautologisch "n von n".
	if(ywquelle>0u) {
		if(yw_yw_geaendert==0ull&&yw_n_geaendert==0ull) print_error("CFD_FACETTEN_YWQUELLE=1, aber WEDER y_w NOCH eine Normale hat sich gegenueber dem Zustand davor geaendert -- stiller No-Op.");
		else print_info("  y_w-ANKER AKTIV: "+to_string(yw_aktiv)+" Eintritte, davon y_w geaendert "+to_string(yw_yw_geaendert)
			+", Normale gekippt "+to_string(yw_kipp)+", Orientierungsbit neu bewertet "+to_string(yw_ori_neu)+".");
	}
	if(normquelle==1u) {
		if(nq_n_geaendert==0ull) print_error("CFD_FACETTEN_NORMQUELLE=1, aber KEINE einzige Normale unterscheidet sich von der PCA-Normalen -- stiller No-Op.");
		else print_info("  NORMALENQUELLE V3b AKTIV: "+to_string(nq_aktiv)+" von "+to_string((ulong)F.size())
			+" Eintritte, Normale wirklich geaendert "+to_string(nq_n_geaendert)+", y_w geaendert "+to_string(nq_yw_geaendert)
			+", Vorzeichen gekippt "+to_string(nq_kipp)+" (Duennteil-Rueckfall "+to_string(v6_rueck)+").");
	}
	print_info(string("Facetten (")+wo+"): "+to_string(nf)+" wandnahe Fluidzellen; Klassen: K1(<6 Punkte) "+to_string(k1)
		+", K2(Kante) "+to_string(k2)+", K3(Linie) "+to_string(k3)+", K4(y_w, nach Glaettung) "+to_string(k4)
		+", Orientierung "+to_string(kori)+", Punktueberlauf "+to_string(k_ueberlauf)+", bewegte-Wand-Naehe "+to_string(k_ms));
	if(nf>0ull) print_info("  markierte ZELLEN (klasse!=0, Nachpruefer B2 -- Bitmaske, keine Zaehlersumme): "
		+to_string(markiert)+" = "+to_string(100.0f*(float)markiert/(float)nf,1u)
		+" % (Schwelle r21>"+to_string((float)kante_r21,3u)+", Default 0,15 geeicht 2026-08-15 aus Fenster-A/B, r10<0,02 Sicherheitsnetz -- K3 war ueberall leer)");
	sortiere(hist_yw); sortiere(hist_winkel); // einmal, danach nur noch indizieren (Audit-Performance)
	print_info("  y_w: Median "+to_string((float)quantil(hist_yw,0.5),3u)+", q10 "+to_string((float)quantil(hist_yw,0.1),3u)
		+", q90 "+to_string((float)quantil(hist_yw,0.9),3u)+" (Anker parallelwandig: exakt 0,500)");
	// ★ NORMALEN-KOMPONENTEN (28.08.): der 26-Grad-Arm mit V3b meldet eine y-Reibung von +0,1074
	// gegen V1s -0,0171 -- sechsmal groesser und fast so gross wie die x-Reibung (0,1285). Die
	// Treppe ist in y INVARIANT, dort darf keine Querkraft entstehen. Der Winkel zur dominanten
	// Achse kann das nicht zeigen (acos(max|n_i|) ist blind dafuer, WELCHE der beiden anderen
	// Komponenten den Rest traegt). Also je Komponente einzeln.
	{	std::vector<double> hx, hy, hz;
		for(const Facette& f : F) { if(f.klasse&1u) continue;
			hx.push_back((double)fabs(f.nx)); hy.push_back((double)fabs(f.ny)); hz.push_back((double)fabs(f.nz)); }
		sortiere(hx); sortiere(hy); sortiere(hz);
		if(!hx.empty()) print_info("  Normalen-Komponenten |n| Median/q90: x "+to_string((float)quantil(hx,0.5),4u)+"/"+to_string((float)quantil(hx,0.9),4u)
			+"  y "+to_string((float)quantil(hy,0.5),4u)+"/"+to_string((float)quantil(hy,0.9),4u)
			+"  z "+to_string((float)quantil(hz,0.5),4u)+"/"+to_string((float)quantil(hz,0.9),4u));
	}
	print_info("  Winkel zur dominanten Achse: Median "+to_string((float)quantil(hist_winkel,0.5),1u)
		+" Grad, q90 "+to_string((float)quantil(hist_winkel,0.9),1u)+" Grad");
	// ★ SOLID-DICKE (Heiko 28.08.: "natuerlich hat der mr2 gurney, canards und luftleitbleche
	// ... sind selbst bei 8mm noch deutlich durch sat voxelizer sichtbar"). Der Facettenzensus
	// kannte bis hier KEINE Bauteildicke -- damit war der Taubin-Duennteileffekt (B37:
	// projizierte Stirnflaeche +5,8 % bei 2 Zellen Merkmalshoehe gegen +1,1 % bei 10) in
	// 755.344 Zellen unsichtbar. Gegenstueck zu freie_weite (oben, Zeile 1092 ff.):
	// je Wandnachbar die Solid-Lauflaenge entlang der drei Achsen, davon das Minimum = lokale
	// Bauteildicke; je Facettenzelle das Minimum ueber ihre Wandnachbarn, also das DUENNSTE
	// Teil, das sie beruehrt. Lauflaenge bei 9 Zellen gekappt (nur 1..4 sind die Frage).
	// Wrap wie ueberall in dieser Funktion: Geometrie ungewickelt, NUR der Speicherindex wickelt.
	// ★ 29.08.: der Zensus lief IMMER, ohne Schalter -- rund 860 Flag-Zugriffe je Facette. Er
	// bleibt per Default AN (Iron Rule 2: Diagnostik gehoert in den Code, und er ist der einzige
	// unabhaengige Gegencheck zum Duennteilbefund), ist jetzt aber abschaltbar. Aus heisst:
	// Histogramm und CSV-Spalte tragen 0, und der V4-Wirkpfad-Abgleich entfaellt mit Ansage.
	const bool dicke_an = env_u("CFD_FACETTEN_DICKE", 1u)>0u;
	if(!dicke_an) print_warning("CFD_FACETTEN_DICKE=0: der Solid-Dicke-Zensus ist AUS -- solid_dicke ist ueberall 0 und der V4-Wirkpfad-Abgleich entfaellt.");
	std::vector<uchar> sdicke(F.size(), (uchar)0u); // ★ auch fuer die CSV -- Kreuztabelle Klasse gegen Dicke
	ulong n_dicke1=0ull; // ★ Gegenprobe fuer den Duennteil-Waechter (siehe Bericht)
	{
		ulong dh[10]={0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull,0ull}; ulong ohne_nachbar=0ull;
		size_t fi_=0ull;
		if(dicke_an) for(const Facette& f : F) {
			const uint zc=(uint)(f.n/((ulong)Nx*(ulong)Ny));
			const uint yc=(uint)((f.n/(ulong)Nx)%(ulong)Ny);
			const uint xc=(uint)(f.n%(ulong)Nx);
			uint dmin=10u;
			for(uint i=1u; i<19u; i++) {
				const int zn0=(int)zc+FZ_C[i][2]; if(!z_per&&(zn0<0||zn0>=(int)Nz)) continue;
				const uint zni=(uint)(z_per?(int)wz(zn0):zn0);
				const int xn=(int)xc+FZ_C[i][0], yn=(int)yc+FZ_C[i][1];
				if(!ist_wand(idx(wx(xn),wy(yn),zni))) continue;
				uint dachse=10u;
				for(uint a=0u; a<3u; a++) {
					uint lauf=1u;
					for(int sg=-1; sg<=1; sg+=2) for(uint k=1u; k<9u; k++) {
						int px=xn, py=yn, pz=zn0;
						if(a==0u) px+=sg*(int)k; else if(a==1u) py+=sg*(int)k; else pz+=sg*(int)k;
						// ★ Pruefagent M5: der Rand MUSS wie in hinten_solid als Solid gelten. Vorher
						// brach der Lauf hier ab (= "duenn"), waehrend der Rueckseitentest ihn als
						// Solid wertete (= "nicht duenn"). Am kipp0-Kanal, wo die Wand buendig bei
						// z=0 endet, haetten beide Wege gegensaetzlich gemeldet und der gekoppelte
						// Waechter haette faelschlich "Rueckseitentest defekt" geschlagen.
						if(!z_per&&(pz<0||pz>=(int)Nz)) { lauf=9u; break; }
						if(!ist_wand(idx(wx(px),wy(py),(uint)(z_per?(int)wz(pz):pz)))) break;
						lauf++;
					}
					if(lauf>9u) lauf=9u;
					if(lauf<dachse) dachse=lauf;
				}
				if(dachse<dmin) dmin=dachse;
			}
			if(dmin>9u) { ohne_nachbar++; fi_++; continue; } // Waechter: kann bei wandnahen Zellen nicht auftreten
			dh[dmin]++; sdicke[fi_++]=(uchar)dmin;
		}
		string h="  SOLID-DICKE der beruehrten Bauteile (1..>=9 Zellen): ";
		for(uint d=1u; d<10u; d++) h+=to_string(dh[d])+(d<9u?" ":"");
		print_info(h);
		n_dicke1=dh[1];
		const ulong duenn=dh[1]+dh[2]+dh[3];
		if(nf>0ull) print_info("  davon an Teilen <=3 Zellen dick: "+to_string(duenn)+" = "
			+to_string(100.0f*(float)duenn/(float)nf,2u)+" % (Gurney/Canard/Leitblech-Population -- "
			+"B37: dort dickt Taubin die projizierte Stirnflaeche um bis zu 5,8 % auf)");
		if(ohne_nachbar>0ull) print_error("SOLID-DICKE: "+to_string(ohne_nachbar)+" Facettenzellen ohne Wandnachbarn -- unmoeglich, Zaehler oder Wrap defekt.");
	}
	// ★ BERICHT DREIFACHVERGLEICH (Heiko-Auftrag 28.08.). Reine Diagnose.
	// ★ Pruefagent M6: dieser Waechter stand im if(vgl_an)-Block und lief im PRODUKTIONSARM
	// (NORMQUELLE=1 ohne VERGLEICH) gar nicht mit -- dort war nq_aktiv==0 der einzige Wirkpfad-
	// Waechter, und der faengt nur den Totalausfall. Jetzt unbedingt.
	// ★ WAECHTER MIT GEGENPROBE statt Pauschalregel. Erste Fassung meldete jedes v4_duenn==0
	// als harten Fehler und schoss damit die Kugel ab -- deren Solid-Dicke-Histogramm ist
	// 0 0 222 0 456 0 116, sie hat also gar kein Teil unter 3 Zellen und NULL ist dort die
	// richtige Antwort. Der Rueckseitentest (Heiko) und die Lauflaengenmessung sind zwei
	// unabhaengige Wege zur selben Aussage -- geprueft wird jetzt ihre UEBEREINSTIMMUNG.
	if(!v3_noetig) { /* ohne V3-Pfad gibt es keine Duennteil-Flaggen -- nichts zu pruefen */ }
	// ★ 29.08.: ohne Dickenzensus ist n_dicke1 konstruktiv 0 -- der Abgleich wuerde dann
	// faelschlich "Rueckseitentest ohne Gegenstueck" melden. Also ausdruecklich aussetzen.
	else if(!dicke_an) { /* Dickenzensus aus -- der Gegencheck hat keine Datenbasis */ }
	// ★ Audit L 29.08.: war print_error (= exit). Beide Messwege sind DIAGNOSEN, und die
	// Richtung "Dicke sagt duenn, Rueckseitentest schweigt" ist nicht ausgeschlossen: der
	// Rueckseitentest zaehlt nur SICHTBARE Flaechen, der Dickenzensus auch Diagonalnachbarn.
	// Ein reines Kreuzmass darf einen 1,5-Stunden-Lauf nicht abschiessen -- jetzt Warnung.
	else if(n_dicke1>0ull&&v4_duenn==0ull)
		print_warning("V4-Wirkpfad: "+to_string(n_dicke1)+" Zellen an 1-Zellen-Teilen, aber KEINE Duennteil-Flaeche -- Rueckseitentest pruefen (Diagnose, kein Abbruch).");
	else if(n_dicke1==0ull&&v4_duenn>0ull)
		print_warning("V4-Wirkpfad: "+to_string(v4_duenn)+" Duennteil-Flaechen, aber KEIN 1-Zellen-Teil im Dickenzensus -- die beiden Messwege widersprechen sich (Diagnose, kein Abbruch).");
	else if(v4_duenn==0ull)
		print_info("   V4-Wirkpfad: kein 1-Zellen-Teil im Fall (Dickenzensus bestaetigt) -- Rueckseitentest korrekt still.");
	if(normquelle==1u&&v6_rueck>0ull) print_info("   V3b-Rueckfall hat "+to_string(v6_rueck)
		+" mal gefeuert (entartete Vektorsumme an einzelligen Teilen oder Spalten) -- diese Zellen haetten sonst keine Normale.");
	if(vgl_an) {
		// ★ Pruefagent M3: vorher wurden 2 von 15 Parallelvektoren geprueft, und nach dem
		// print_error lief der Block WEITER und indizierte ungeprueft bis F.size() -- bei echtem
		// Versatz undefiniertes Verhalten statt Abbruch. Jetzt alle, und Rueckgabe statt Weiterlauf.
		{	const size_t n_=F.size(); bool ok_=true;
			const size_t L_[15]={a12.size(),a13.size(),a14.size(),yw2v.size(),yw3v.size(),yw4v.size(),
				yw4nv.size(),yw4av.size(),yw5v.size(),yw6v.size(),kl1v.size(),kl2v.size(),kl3v.size(),kl4v.size(),kl5v.size()};
			for(int q_=0;q_<15;q_++) if(L_[q_]!=n_) ok_=false;
			if(kl6v.size()!=n_||sdicke.size()!=n_) ok_=false;
			if(!ok_) { print_error("VERGLEICH: Indexversatz gegen die Facettenzahl "+to_string((ulong)n_)
				+" -- Kreuztabellen waeren still falsch, Bericht wird uebersprungen."); return F; }
		}
		const ulong nn=(ulong)F.size();
		auto bits=[&](const int v, const ulong i)->uchar {
			return (v==0)?kl1v[i]:((v==1)?F[i].klasse:((v==2)?kl2v[i]:((v==3)?kl3v[i]:((v==7)?kl5v[i]:((v==8)?kl6v[i]:kl4v[i]))))); };
		auto zaehl=[&](const int v, const uchar bit)->ulong {
			ulong c=0ull; for(ulong i=0ull;i<nn;i++) if(bits(v,i)&bit) c++; return c; };
		// v=4 -> V4 mit y_w aus dem Schwerpunkt (Bit 8), v=5 -> V4 mit y_w aus der naechsten Flaeche (Bit 16)
		auto verw=[&](const int v)->ulong {
			const uchar maske = (v==5) ? (uchar)(1u|2u|4u|16u) : ((v==6) ? (uchar)(1u|2u|4u|32u) : (uchar)15u);
			const int vq = (v>=7)?v:((v>=5)?4:v);
			ulong c=0ull; for(ulong i=0ull;i<nn;i++) if(bits(vq,i)&maske) c++; return c; };
		// ★ H1: V1 erscheint jetzt ZWEIMAL -- vor der Glaettung (gleichbasiert mit V2..V3b) und
		// danach (das, was der Loeser bekaeme). Die Differenz IST der Glaettungsanteil.
		{	ulong vpre=0ull; for(ulong i=0ull;i<nn;i++) if(kl1v[i]&(uchar)15u) vpre++;
			print_info("   V1 VOR der Glaettung (gleichbasiert): OHNE FACETTE "+to_string(vpre)+" = "
				+to_string(nn>0ull?100.0f*(float)vpre/(float)nn:0.0f,2u)+" %  -- die Differenz zur V1-Zeile unten ist der GLAETTUNGSANTEIL");
		}
		print_info("   V4-Wirkpfad: Zellen mit mindestens einer DUENNTEIL-Flaeche (hinter dem Dreieck kein Solid) "
			+to_string(v4_duenn)+" = "+to_string(nn>0ull?100.0f*(float)v4_duenn/(float)nn:0.0f,2u)
			+" % | Rueckfall auf die naechste Flaeche (Summe entartet) "+to_string(v4_rueck)
			+" | K4 bei V4b: Untergrenze "+to_string(v4_unten)+", Obergrenze "+to_string(v4_oben)
			+" | V3b-Rueckfall "+to_string(v6_rueck));
		print_info("  ---- NORMALENQUELLEN im Vergleich (K1..K4 gleich definiert; V1 zaehlt zusaetzlich Orientierung/Ueberlauf/MS, hier ausgeklammert)");
		const char* nm[8]={"V1 alle 18 Linkmitten (heute)     ","V2 nur Achslinks (PCA)            ",
		                   "V3 Flaechennormalen-Summe         ","V4 = V3 + Sicht + Abstandsgewicht ",
		                   "V4b wie V4, y_w aus Nachbarflaeche","V4c wie V4, y_w ausgerichtet      ",
		                   "V5 = V3 + Sicht + Rueckfall (o.G.)","V3b = V3 + Rueckfall + y_w nah   "};
		for(int v=1; v<=8; v++) {
			const ulong vv=verw(v);
			const int vq=(v>=7)?v:((v>=5)?4:v); const uchar k4bit=(v==5)?(uchar)16u:((v==6)?(uchar)32u:(uchar)8u);
			print_info(string("   ")+nm[v-1]+": K1 "+to_string(zaehl(vq,1u))+", K2(Kante) "+to_string(zaehl(vq,2u))
				+", K3 "+to_string(zaehl(vq,4u))+", K4(y_w) "+to_string(zaehl(vq,k4bit))
				+"  -> OHNE FACETTE "+to_string(vv)+" = "+to_string(nn>0ull?100.0f*(float)vv/(float)nn:0.0f,2u)+" %");
		}
		auto med=[&](std::vector<float> v)->std::pair<float,float> {
			if(v.empty()) return {0.0f,0.0f};
			std::sort(v.begin(), v.end());
			return { v[v.size()/2], v[(size_t)fmin((double)v.size()-1.0, 0.9*(double)v.size())] }; };
		{ std::vector<float> w2,w3,w4;
		  for(ulong i=0ull;i<nn;i++) { if(a12[i]>=0.0f) w2.push_back(a12[i]); if(a13[i]>=0.0f) w3.push_back(a13[i]); if(a14[i]>=0.0f) w4.push_back(a14[i]); }
		  const auto m2=med(w2), m3=med(w3), m4=med(w4);
		  print_info("   Winkel gegen V1: V2 Median "+to_string(m2.first,2u)+" (q90 "+to_string(m2.second,2u)
			+"), V3 "+to_string(m3.first,2u)+" (q90 "+to_string(m3.second,2u)+"), V4 "+to_string(m4.first,2u)
			+" (q90 "+to_string(m4.second,2u)+") Grad"); }
		{ std::vector<float> y1,y2,y3;
		  for(ulong i=0ull;i<nn;i++) { if(!(F[i].klasse&1u)) y1.push_back(F[i].yw); if(!(kl2v[i]&1u)) y2.push_back(yw2v[i]); if(!(kl3v[i]&1u)) y3.push_back(yw3v[i]); }
		  std::vector<float> y4,y4n;
		  for(ulong i=0ull;i<nn;i++) if(!(kl4v[i]&1u)) { y4.push_back(yw4v[i]); y4n.push_back(yw4nv[i]); }
		  const auto q1=med(y1), q2=med(y2), q3=med(y3), q4=med(y4), q4n=med(y4n);
		  print_info("   y_w Median/q90: V1 "+to_string(q1.first,3u)+"/"+to_string(q1.second,3u)
			+"  V2 "+to_string(q2.first,3u)+"/"+to_string(q2.second,3u)+"  V3 "+to_string(q3.first,3u)+"/"+to_string(q3.second,3u)
			+"  V4 "+to_string(q4.first,3u)+"/"+to_string(q4.second,3u)+"  V4b "+to_string(q4n.first,3u)+"/"+to_string(q4n.second,3u)); }
		if(kref_an&&nv1.size()!=3ull*nn) print_error("KUGELREF: Normalenvektor "+to_string((ulong)nv1.size())
			+" != 3 x "+to_string(nn)+" -- die Grundwahrheitstabelle entfaellt (Pruefagent M9: vorher still).");
		if(kref_an&&nv1.size()==3ull*nn) { // ---- WINKELFEHLER GEGEN DIE ANALYTISCHE KUGELNORMALE
			std::vector<float> e1,e2,e3,e4,e5,e6;
			for(ulong i=0ull;i<nn;i++) {
				if(F[i].klasse&1u) continue;
				const uint zc=(uint)(F[i].n/((ulong)Nx*(ulong)Ny));
				const uint yc=(uint)((F[i].n/(ulong)Nx)%(ulong)Ny);
				const uint xc=(uint)(F[i].n%(ulong)Nx);
				double wx_=(double)xc-kmx, wy_=(double)yc-kmy, wz_=(double)zc-kmz;
				const double wl=sqrt(wx_*wx_+wy_*wy_+wz_*wz_); if(wl<1e-9) continue;
				wx_/=wl; wy_/=wl; wz_/=wl;
				auto fe=[&](const std::vector<float>& v)->float {
					const double c=fabs((double)v[3ull*i]*wx_+(double)v[3ull*i+1ull]*wy_+(double)v[3ull*i+2ull]*wz_);
					return (float)(acos(fmin(1.0,c))*180.0/3.14159265358979); };
				e1.push_back(fe(nv1)); e2.push_back(fe(nv2)); e3.push_back(fe(nv3)); e4.push_back(fe(nv4)); e5.push_back(fe(nv5)); e6.push_back(fe(nv6));
			}
			auto med2=[&](std::vector<float> v)->std::pair<float,float> {
				if(v.empty()) return {0.0f,0.0f}; std::sort(v.begin(),v.end());
				return { v[v.size()/2], v[(size_t)fmin((double)v.size()-1.0,0.9*(double)v.size())] }; };
			const auto q1=med2(e1), q2=med2(e2), q3=med2(e3), q4=med2(e4), q5=med2(e5), q6=med2(e6);
			print_info("   WINKELFEHLER GEGEN DIE ANALYTISCHE KUGELNORMALE (Median / q90, "+to_string((ulong)e1.size())+" Zellen):");
			print_info("     V1 "+to_string(q1.first,2u)+" / "+to_string(q1.second,2u)
				+"   V2 "+to_string(q2.first,2u)+" / "+to_string(q2.second,2u)
				+"   V3 "+to_string(q3.first,2u)+" / "+to_string(q3.second,2u)
				+"   V4 "+to_string(q4.first,2u)+" / "+to_string(q4.second,2u)
				+"   V5 "+to_string(q5.first,2u)+" / "+to_string(q5.second,2u)
				+"   V3b "+to_string(q6.first,2u)+" / "+to_string(q6.second,2u)+" Grad");
		}
		// Aufschluesselung nach Solid-Dicke -- ohne sie geht der Duennteileffekt in 755k Zellen unter (B38)
		const uint gl[4]={1u,2u,3u,4u}; const char* gn[4]={"Dicke 1   ","Dicke 2   ","Dicke 3   ","Dicke >=4 "};
		for(uint g=0u; g<4u; g++) {
			std::vector<float> w2,w3,w4; ulong n_g=0ull, v1=0ull, v2c=0ull, v3c=0ull, v4c=0ull, v4bc=0ull, v4cc=0ull;
			for(ulong i=0ull;i<nn;i++) {
				const uint d=(uint)sdicke[i];
				const bool drin = (g<3u) ? (d==gl[g]) : (d>=4u);
				if(!drin) continue;
				n_g++;
				if(F[i].klasse&(uchar)15u) v1++;
				if(kl2v[i]&(uchar)15u) v2c++;
				if(kl3v[i]&(uchar)15u) v3c++;
				if(kl4v[i]&(uchar)15u) v4c++;
				if(kl4v[i]&(uchar)(1u|2u|4u|16u)) v4bc++;
				if(kl4v[i]&(uchar)(1u|2u|4u|32u)) v4cc++;
				if(a12[i]>=0.0f) w2.push_back(a12[i]);
				if(a13[i]>=0.0f) w3.push_back(a13[i]);
				if(a14[i]>=0.0f) w4.push_back(a14[i]);
			}
			if(n_g==0ull) continue;
			const auto m2=med(w2), m3=med(w3), m4=med(w4);
			print_info(string("   ")+gn[g]+": "+to_string(n_g)+" Zellen | ohne Facette V1 "
				+to_string(100.0f*(float)v1/(float)n_g,1u)+" V2 "+to_string(100.0f*(float)v2c/(float)n_g,1u)
				+" V3 "+to_string(100.0f*(float)v3c/(float)n_g,1u)+" V4 "+to_string(100.0f*(float)v4c/(float)n_g,1u)
				+" V4b "+to_string(100.0f*(float)v4bc/(float)n_g,1u)+" V4c "+to_string(100.0f*(float)v4cc/(float)n_g,1u)+" % | Winkel gegen V1: V2 "
				+to_string(m2.first,2u)+" V3 "+to_string(m3.first,2u)+" V4 "+to_string(m4.first,2u)+" Grad");
		}
	}
	std::ofstream fh(out_dir+"facetten_histogramme.csv");
	fh << "# Facetten-Diagnose ("<<wo<<"), Stufe 1 -- yw,winkel_grad,r21,r10,klasse,achse,n_punkte,eigene_links,n,solid_dicke\n"; // ★ solid_dicke angehaengt (Heiko 28.08.), Spaltenzahl waechst -- Auswerter lesen nach Namen // ★ K4-Ring-Etappe: Zellindex n als letzte Spalte -- ohne ihn war keine DIAGZ-Zielwahl aus dem Census moeglich
	{ size_t fi_=0ull; for(const Facette& f : F) { fh << f.yw << "," << (acos(fmin(1.0f,fmax(fabs(f.nx),fmax(fabs(f.ny),fabs(f.nz)))))*180.0f/3.14159265f)
		<< "," << f.r21_ << "," << f.r10_ << "," << (uint)f.klasse << "," << (uint)f.achse << "," << f.n_punkte << "," << f.eigene_links << "," << f.n
		<< "," << (uint)sdicke[fi_++] << "\n"; } }
	fh.close();
	print_info("  CSV: "+out_dir+"facetten_histogramme.csv");

	// ================= GUETEKLASSE JE WANDZELLE, VTK fuer ParaView (Heiko 29.08.) =================
	// "Kategorie 0 ist der schlechteste Fall, aufsteigend der bessere." Punktwolke an den
	// Zellmittelpunkten in WELTkoordinaten -- sie liegt damit deckungsgleich ueber den Feld-VTKs.
	//   0 KEIN WANDMODELL   klasse!=0, die Zelle faellt auf reines Bounce-Back zurueck (schlechtester Fall)
	//   1 NORMALE UNSICHER  Kohaerenz |Summe|/nq < 1/sqrt(3): die Flaechensumme ist ueberwiegend
	//                       Ausloeschungsrest, die Richtung traegt wenig Information
	//   2 NORMALE GROB      Duennteil-Rueckfall: Vektorsumme entartet, Normale = naechste Voxelflaeche
	//                       (grob, aber eindeutig definiert)
	//   3 Y_W GEKLEMMT      Normale gut, aber die Abtasthoehe war geometriebedingt unter yw_min
	//                       und wurde auf yw_min gesetzt statt die Zelle zu verwerfen
	//   4 VOLL              kohaerente Flaechensumme, y_w im Fenster, kein Rueckfall
	// Bei mehreren Treffern gilt der SCHLECHTESTE (kleinste) Wert.
	if(guete_vtk&&!F.empty()) {
		if(gt_v.size()!=F.size()||koh_v.size()!=F.size())
			print_error("Guete-VTK: gt_v/koh_v sind nicht deckungsgleich mit F ("+to_string((ulong)gt_v.size())+"/"+to_string((ulong)koh_v.size())+" gegen "+to_string((ulong)F.size())+") -- der Parallel-Push ist verrutscht.");
		std::vector<uchar> kat(F.size(), (uchar)4u);
		ulong hist[5]={0ull,0ull,0ull,0ull,0ull};
		for(size_t i=0ull;i<F.size();i++) {
			uchar k=4u;
			if(gt_v[i]&4u) k=3u;                 // geklemmt
			if(gt_v[i]&1u) k=2u;                 // Duennteil-Rueckfall
			if(gt_v[i]&2u) k=1u;                 // schwache Kohaerenz
			if(F[i].klasse!=0u) k=0u;            // verworfen schlaegt alles
			kat[i]=k; hist[k]++;
		}
		const string dat = out_dir+"facetten_guete.vtk";
		std::ofstream v(dat, std::ios::out|std::ios::binary);
		if(!v) print_warning("Guete-VTK: "+dat+" nicht schreibbar -- uebersprungen.");
		else {
			const ulong np_ = (ulong)F.size();
			v << "# vtk DataFile Version 3.0\nFacetten-Guete " << wo << " (0 schlechtester Fall .. 4 voll)\nBINARY\nDATASET POLYDATA\n"
			  << "POINTS " << np_ << " float\n";
			{	std::vector<float> b(3ull*np_);
				for(size_t i=0ull;i<np_;i++) { uint px_,py_,pz_; L.coordinates(F[i].n, px_, py_, pz_);
					b[3ull*i   ] = reverse_bytes(vtk_x0 + vtk_dx*((float)px_+0.5f));
					b[3ull*i+1u] = reverse_bytes(vtk_y0 + vtk_dx*((float)py_+0.5f));
					b[3ull*i+2u] = reverse_bytes(vtk_z0 + vtk_dx*((float)pz_+0.5f)); }
				v.write((const char*)b.data(), (std::streamsize)(b.size()*sizeof(float))); }
			v << "\nVERTICES " << np_ << " " << 2ull*np_ << "\n";
			{	std::vector<int> c(2ull*np_);
				for(size_t i=0ull;i<np_;i++) { c[2ull*i]=reverse_bytes((int)1); c[2ull*i+1u]=reverse_bytes((int)i); }
				v.write((const char*)c.data(), (std::streamsize)(c.size()*sizeof(int))); }
			v << "\nPOINT_DATA " << np_ << "\n";
			auto schreibe_uc = [&](const string& nm, const std::vector<uchar>& d) {
				v << "SCALARS " << nm << " unsigned_char 1\nLOOKUP_TABLE default\n";
				v.write((const char*)d.data(), (std::streamsize)d.size()); v << "\n"; };
			auto schreibe_f = [&](const string& nm, const std::vector<float>& d) {
				v << "SCALARS " << nm << " float 1\nLOOKUP_TABLE default\n";
				const size_t blk = 65536ull; std::vector<float> b(blk); // ★ D-3: war ein Vollpuffer (13 MB je Feld)
				for(size_t i0=0ull;i0<d.size();i0+=blk) {
					const size_t n_ = min(blk, d.size()-i0);
					for(size_t j=0ull;j<n_;j++) b[j]=reverse_bytes(d[i0+j]);
					v.write((const char*)b.data(), (std::streamsize)(n_*sizeof(float))); }
				v << "\n"; };
			schreibe_uc("kategorie", kat);
			{	std::vector<uchar> kl_bin(np_), ac(np_); std::vector<float> yw(np_), wk(np_), r21(np_);
				for(size_t i=0ull;i<np_;i++) { kl_bin[i]=(uchar)(F[i].klasse?1u:0u); ac[i]=(uchar)F[i].achse; yw[i]=F[i].yw; r21[i]=F[i].r21_;
					wk[i]=acos(fmin(1.0f,fmax(fabs(F[i].nx),fmax(fabs(F[i].ny),fabs(F[i].nz)))))*180.0f/3.14159265f; }
				// ★ Heiko 29.08.: reine Zahlenskalen, keine Buchstaben-Zahlen-Kuerzel. Neben der
				// Kategorie steht deshalb JEDE Verwerfungsursache als eigenes 0/1-Feld -- danach
				// laesst sich in ParaView direkt einfaerben und schwellen, ohne Bitmasken zu lesen.
				{	std::vector<uchar> u_kante(np_), u_wandabstand(np_), u_orientierung(np_),
					                   u_bewegtewand(np_), u_wenigpunkte(np_), u_linie(np_),
					                   u_ueberlauf(np_), u_rueckfall(np_), u_geklemmt(np_), u_unsicher(np_);
					for(size_t i=0ull;i<np_;i++) {
						const uchar c=F[i].klasse, g=gt_v[i];
						u_wenigpunkte[i]  = (uchar)((c& 1u)?1u:0u);
						u_kante[i]        = (uchar)((c& 2u)?1u:0u);
						u_linie[i]        = (uchar)((c& 4u)?1u:0u);
						u_wandabstand[i]  = (uchar)((c& 8u)?1u:0u);
						u_orientierung[i] = (uchar)((c&16u)?1u:0u);
						u_ueberlauf[i]    = (uchar)((c&32u)?1u:0u); // ★ Pruefagent D-2: fehlte -- eine nur
						u_bewegtewand[i]  = (uchar)((c&64u)?1u:0u); //    deswegen verworfene Zelle war unauffindbar
						u_rueckfall[i]    = (uchar)((g& 1u)?1u:0u);
						u_unsicher[i]     = (uchar)((g& 2u)?1u:0u);
						u_geklemmt[i]     = (uchar)((g& 4u)?1u:0u); }
					schreibe_uc("verworfen_kante", u_kante);
					schreibe_uc("verworfen_wandabstand", u_wandabstand);
					schreibe_uc("verworfen_orientierung", u_orientierung);
					schreibe_uc("verworfen_bewegtewand", u_bewegtewand);
					schreibe_uc("verworfen_wenigpunkte", u_wenigpunkte);
					schreibe_uc("verworfen_linie", u_linie);
					schreibe_uc("verworfen_ueberlauf", u_ueberlauf);
					schreibe_uc("normale_rueckfall", u_rueckfall);
					schreibe_uc("normale_unsicher", u_unsicher);
					schreibe_uc("wandabstand_geklemmt", u_geklemmt); }
				schreibe_uc("verworfen", kl_bin);   // 1 = diese Zelle hat KEIN Wandmodell (klasse!=0)
				schreibe_uc("bauteildicke", sdicke);// lokale Bauteildicke in Zellen (9 = gekappt)
				schreibe_uc("dominante_achse", ac); // 0 = x, 1 = y, 2 = z
				schreibe_f("wandabstand", yw);      // y_w in Zellen
				schreibe_f("kohaerenz", koh_v);     // |Summe|/nq, unter 0,577 ist die Normale Ausloeschungsrest
				schreibe_f("winkel", wk);           // Grad zur dominanten Achse
				schreibe_f("fitguete", r21);        // Eigenwertverhaeltnis des PCA-Fits (klein = gute Ebene)
				{	// ★ Pruefagent D-3: hier standen drei Zwischenpuffer nx_/ny_/nz_ (39 MB bei
					// 3,28 Mio Facetten) NEBEN dem Schreibpuffer. Direkt aus F fuellen, blockweise
					// schreiben wie der Referenzschreiber (setup.cpp:329) -- Spitze faellt von
					// rund 128 MB auf unter 1 MB.
					v << "VECTORS normale float\n";
					const size_t blk = 65536ull; std::vector<float> b(3ull*blk);
					for(size_t i0=0ull;i0<np_;i0+=blk) {
						const size_t n_ = min(blk, np_-i0);
						for(size_t j=0ull;j<n_;j++) { b[3ull*j]=reverse_bytes(F[i0+j].nx); b[3ull*j+1u]=reverse_bytes(F[i0+j].ny); b[3ull*j+2u]=reverse_bytes(F[i0+j].nz); }
						v.write((const char*)b.data(), (std::streamsize)(3ull*n_*sizeof(float))); }
					v << "\n"; }
			}
			v.close();
			const bool welt = !(vtk_dx==1.0f&&vtk_x0==0.0f&&vtk_y0==0.0f&&vtk_z0==0.0f);
			print_info("  GUETE-VTK: "+dat+" ("+to_string(np_)+" Punkte, "
				+(welt?string("Weltkoordinaten -- liegt ueber den Feld-VTKs"):string("GITTERkoordinaten -- die Aufrufstelle hat keine Weltlage gesetzt"))+").");
			print_info("   Kategorien: 0 kein Wandmodell "+to_string(hist[0])+" ("+to_string(100.0f*(float)hist[0]/(float)np_,1u)+" %)"
				+" | 1 Normale unsicher "+to_string(hist[1])+" ("+to_string(100.0f*(float)hist[1]/(float)np_,1u)+" %)"
				+" | 2 Normale grob "+to_string(hist[2])+" ("+to_string(100.0f*(float)hist[2]/(float)np_,1u)+" %)"
				+" | 3 y_w geklemmt "+to_string(hist[3])+" ("+to_string(100.0f*(float)hist[3]/(float)np_,1u)+" %)"
				+" | 4 voll "+to_string(hist[4])+" ("+to_string(100.0f*(float)hist[4]/(float)np_,1u)+" %)");
		}
	}
	// ★ Pruefagent B-2: n_geklemmt zaehlte EREIGNISSE -- bei NORMQUELLE=1 feuert erst der
	// V1-Pfad, dann V3b, ggf. noch die Glaettung fuer DIESELBE Zelle. Der Vergleich gegen die
	// Zellzahl war damit bis zu dreifach zu gross. Jetzt aus gt_v: eine Zelle, ein Zaehler.
	if(yw_klemme) { ulong zellen=0ull; for(uchar g_ : gt_v) if(g_&4u) zellen++;
		print_info("  Y_W-KLEMME AKTIV: "+to_string(zellen)+" ZELLEN auf yw_min = "+to_string(yw_min,3u)
			+" geklemmt statt verworfen ("+to_string(n_geklemmt)+" Klemmereignisse ueber alle drei Pfade)."); }
	if(kante_koh) print_info("  KANTENTEST AUS KOHAERENZ AKTIV: "+to_string(n_kante_koh)+" Facetten haben ihr Kantenbit gegenueber r21 geaendert.");
	return F;
}

// ---------------------------------------------------------------------------- C1b: Cd-Auslesepfad (FACETTEN-CD-PFAD.md)
// Hybride Zerlegung: DRUCK solid-seitig aus F per n-Projektion (Normalkomponente des Impuls-
// austauschs bleibt an getauschten Links erster Ordnung gueltig), REIBUNG fluid-seitig exakt aus
// dem komponentenweisen Akkumulator (Fenster-Delta / Schritte). Solidzellen ohne tauschenden
// Facettennachbarn: voller F (dort gilt reiner BB). fbi-Formel WOERTLICH wie messe_yplus.
struct FacKraft { double px,py,pz, rx,ry,rz; ulong n_voll,n_proj,n_unklar; double pbx,pby,pbz;
                  double ux,uy,uz; bool ukraft_ok; }; // pb* = Band-Druckanteil (z<zband; 0 bei zband==0)
// ★ u* = KRAFTANTEIL DER "UNKLAREN" ZELLEN (2026-08-27, Zensus-Auftrag Mehrfachfacetten).
// Unklar heisst: |Summe der Facettennormalen der 18er-Nachbarschaft| < 0,5 -- die Normalen heben
// sich weg, weil in der Nachbarschaft GEGENLAEUFIGE Wandseiten liegen (duenne Platte, Spalt,
// Kante). Genau diesen Fall kann EIN TLS-Fit je Wandzelle nicht trennen; die Zellen gehen
// konservativ VOLL (unprojiziert) in die Kraft ein. Der ZAEHLER n_unklar lief immer mit, sein
// KRAFTGEWICHT war nie sichtbar -- und nur das entscheidet, ob ein Cluster-Umbau lohnt.
// Gefuellt wird u* NUR im Host-Pfad (CFD_FAC_GPU=0 oder CFD_FAC_GPU_PRUEF=1); im reinen
// GPU-Pfad bleibt ukraft_ok false, damit niemand eine 0 als Messwert liest (stiller No-Op).
// ★ FORK Kraft-Zerlegung (CFD_KRAFT_ZBAND): zband>0 zerlegt NUR den Druckanteil zusaetzlich in
// Band (z<zband) und Rest (Rest = Gesamt - Band, double). fac_tau (Reibung) traegt keine z-Position --
// die Reibungszerlegung ist Folgearbeit (fac_geo[6] als z-Traeger). zband==0 ist ausdrucksgleich alt.
// ★ 03.09.2026 -- fac_idx ist eine Bitmaske mit Blockpraefixsumme (8 B je 32 F-BBox-Zellen statt
// 4 B je Zelle). AUSDRUCKSGLEICH zum Kernel-Helfer fac_fid() in kernel.cpp; CFD_FAC_GPU_PRUEF
// vergleicht Host- und GPU-Pfad zahlenscharf, beide muessen deshalb dieselbe Rechnung machen.
static inline uint fid_aus_maske(const Memory<uint>& idx, const ulong fbi) {
	const ulong ib = 2ull*(fbi>>5);
	const uint l = (uint)(fbi&31ull);
	const uint maske = idx[ib];
	if(((maske>>l)&1u)==0u) return 0xFFFFFFFFu; // keine oder markierte Facette
	return idx[ib+1ull] + (uint)__builtin_popcount(maske & ((1u<<l)-1u));
}
FacKraft kraft_facetten(LBM& L, const uint Nx, const uint Ny, const uint Nz, const uchar marker,
                        const ulong fenster, const std::vector<double>& snap, const bool z_per=false, const bool flags_aktuell=false, const uint zband=0u) {
	L.update_force_field();
	LBM_Domain* D = L.lbm_domain[0];
	// ★ GPU-Reduktion des Druckanteils (Kernel kraft_facetten_gpu): CFD_FAC_GPU=1 (Default) rechnet
	// auf dem Geraet OHNE Voll-F-Transfer; =0 ist der alte Host-Pfad WORTGLEICH erhalten;
	// CFD_FAC_GPU_PRUEF=1 rechnet BEIDE Pfade und druckt die maximale Relativabweichung px/py/pz
	// plus die Zaehlerstaende (GPU vs Host, muessen EXAKT gleich sein).
	const bool gpu   = env_u("CFD_FAC_GPU", 1u)>0u;
	const bool pruef = env_u("CFD_FAC_GPU_PRUEF", 0u)>0u;
	const bool host_rechnen = !gpu||pruef; // Kontrollarm bzw. Pruefdoppel
	if(host_rechnen) { D->finish_queue(); D->F.read_from_device(); } // im reinen GPU-Zweig ENTFAELLT der Voll-F-Transfer; finish davor (Pruefagent M): auf Zero-Copy-Geraeten ist der Read ein No-Op und erzwingt KEINE Ausfuehrung des enqueueten update_force_field -- ohne finish laese der Host-Arm das F des VORIGEN Updates (Spiegel des GPU-Versatz-Fixes)
	// ★ Profiler-Befund 2026-08-19 (Heiko): flags sind nach initialize() STATISCH -- der
	// Voll-Domaenen-Read je Sample war reine PCIe-Verschwendung (~0,5 GB im dd). Heisse
	// Schleifen lesen EINMAL nach run(0) und uebergeben flags_aktuell=true.
	if(host_rechnen&&!flags_aktuell) L.flags.read_from_device();
	FacKraft K; K.px=K.py=K.pz=K.rx=K.ry=K.rz=0.0; K.n_voll=K.n_proj=K.n_unklar=0ull; K.pbx=K.pby=K.pbz=0.0;
	K.ux=K.uy=K.uz=0.0; K.ukraft_ok=host_rechnen; // u* nur im Host-Pfad belastbar (siehe FacKraft)
	double hbx=0.0, hby=0.0, hbz=0.0; // Host-Band-Druck (nur zband>0 && host_rechnen befuellt)
	const ulong FN = (ulong)D->fbnx*(ulong)D->fbny*(ulong)D->fbnz;
	const bool fac = D->facetten_on;
	if(fac) {
		// fac_tau.read_from_device() MUSS auch im GPU-Zweig bleiben: der dd-Fall liest direkt nach
		// diesem Aufruf die Host-Spiegel [4]/[5] ("fac_tau frisch durch kraft_facetten").
		D->fac_tau.read_from_device();
		if(host_rechnen) D->fac_tau_n.read_from_device(); // nur der Host-Druckpfad braucht den Spiegel -- die GPU liest fac_tau_n auf dem Geraet
		const double fs = fmax(1.0,(double)fenster);
		for(ulong i=0ull; i<D->fac_N; i++) {
			K.rx += ((double)D->fac_tau[6ull*i+1ull]-(snap.empty()?0.0:snap[3ull*i+0ull]))/fs;
			K.ry += ((double)D->fac_tau[6ull*i+2ull]-(snap.empty()?0.0:snap[3ull*i+1ull]))/fs;
			K.rz += ((double)D->fac_tau[6ull*i+3ull]-(snap.empty()?0.0:snap[3ull*i+2ull]))/fs;
		}
		// ★★ PRAEZISIONSWAECHTER, dritte Fassung (2026-08-25). Die ersten beiden waren beide falsch:
		// die erste nahm das rohe Maximum von acc/inc und rief print_error -- sie hat zwei laufende
		// Messungen abgeschossen; die zweite mass acc gegen den FENSTERMITTELWERT des Zuwachses und
		// explodierte, sobald sich Beitraege wegheben, ohne dass ein Bit verloren geht.
		// Das richtige Mass steht bereits im Puffer: fac_tau_n[i] IST die Zahl der Additionen in
		// diesen Akkumulator. Ab 2^24 Additionen gleicher Groessenordnung verpufft jede weitere
		// spurlos, ab 2^20 bleiben dem Zuwachs 4 Bit. Vorzeichenwechsel machen den Akkumulator nur
		// KLEINER, also ist n eine konservative obere Schranke -- und sie deckt Slot 0 (tw) mit ab,
		// der das gemeldete mittlere tau_w traegt und den die zweite Fassung ausliess.
		{
			D->fac_tau_n.read_from_device();
			ulong nmax=0ull; for(ulong i=0ull; i<D->fac_N; i++) { const ulong v=(ulong)D->fac_tau_n[i]; if(v>nmax) nmax=v; }
			static uint gemeldet = 0u;
			if(nmax>=1048576ull&&gemeldet<3u) { gemeldet++;
				print_warning("fac_tau-Praezision: bis zu "+to_string(nmax)+" Additionen in einen float32-Akkumulator (ab 2^20 = 1048576 bleiben dem Zuwachs 4 Bit, ab 2^24 = 16777216 verpufft er spurlos). Kein Abbruch -- aber die Zahlen aus fac_tau tragen ab hier nicht mehr die volle Stellenzahl.");
			}
		}
	}
	if(host_rechnen) {
	auto wxp = [&](const int v) { return (uint)((v%(int)Nx+(int)Nx)%(int)Nx); };
	auto wyp = [&](const int v) { return (uint)((v%(int)Ny+(int)Ny)%(int)Ny); };
	for(uint z=D->fbz0; z<D->fbz0+D->fbnz; z++) for(uint y=D->fby0; y<D->fby0+D->fbny; y++) for(uint x=D->fbx0; x<D->fbx0+D->fbnx; x++) {
		const ulong n = (ulong)x+((ulong)y+(ulong)z*(ulong)Ny)*(ulong)Nx;
		if(L.flags[n]!=marker) continue;
		const ulong fbi = (ulong)(x-D->fbx0)+((ulong)(y-D->fby0)+(ulong)(z-D->fbz0)*(ulong)D->fbny)*(ulong)D->fbnx;
		ulong fs_; const bool hat_slot_ = D->f_slot_host(fbi, fs_); const ulong FST_=D->f_stride(); // ★ 03.09. F-Markerliste
		const double Fx=hat_slot_?(double)D->F[fs_]:0.0, Fy=hat_slot_?(double)D->F[fs_+FST_]:0.0, Fz=hat_slot_?(double)D->F[fs_+2ull*FST_]:0.0;
		double nxm=0.0, nym=0.0, nzm=0.0; bool kontaminiert=false;
		if(fac) for(uint i=1u; i<19u; i++) {
			// z_per (Torus-Kipp) wickelt z wie der Kernel; Standardfall klemmt hart (R3-Notiz eingeloest).
			const int zn0=(int)z+FZ_C[i][2]; if(!z_per&&(zn0<0||zn0>=(int)Nz)) continue;
			const int zn=z_per?(int)((zn0%(int)Nz+(int)Nz)%(int)Nz):zn0;
			const uint xn=wxp((int)x+FZ_C[i][0]), yn=wyp((int)y+FZ_C[i][1]);
			if(xn<D->fbx0||yn<D->fby0||(uint)zn<D->fbz0||xn>=D->fbx0+D->fbnx||yn>=D->fby0+D->fbny||(uint)zn>=D->fbz0+D->fbnz) continue;
			const ulong fbi2=(ulong)(xn-D->fbx0)+((ulong)(yn-D->fby0)+(ulong)((uint)zn-D->fbz0)*(ulong)D->fbny)*(ulong)D->fbnx;
			const uint fid = fid_aus_maske(D->fac_idx, fbi2);
			if(fid==0xFFFFFFFFu||D->fac_tau_n[fid]==0u) continue;
			kontaminiert=true;
			nxm+=(double)D->fac_geo[8ull*fid]; nym+=(double)D->fac_geo[8ull*fid+1ull]; nzm+=(double)D->fac_geo[8ull*fid+2ull];
		}
		if(!kontaminiert) { K.px+=Fx; K.py+=Fy; K.pz+=Fz; K.n_voll++; if(zband>0u&&z<zband) { hbx+=Fx; hby+=Fy; hbz+=Fz; } continue; } // Band-Mitschrift NUR bei zband>0 (Kraft-Zerlegung)
		const double l = sqrt(nxm*nxm+nym*nym+nzm*nzm);
		if(l<0.5) { K.px+=Fx; K.py+=Fy; K.pz+=Fz; K.n_unklar++; K.ux+=Fx; K.uy+=Fy; K.uz+=Fz; if(zband>0u&&z<zband) { hbx+=Fx; hby+=Fy; hbz+=Fz; } continue; } // Gegennormalen (Spalt): konservativ voll -- u* zaehlt DENSELBEN Beitrag getrennt mit (Zensus, keine Physikaenderung)
		const double nx2=nxm/l, ny2=nym/l, nz2=nzm/l;
		const double fn = Fx*nx2+Fy*ny2+Fz*nz2;
		K.px+=fn*nx2; K.py+=fn*ny2; K.pz+=fn*nz2; K.n_proj++;
		if(zband>0u&&z<zband) { hbx+=fn*nx2; hby+=fn*ny2; hbz+=fn*nz2; } // Band-Mitschrift (Kraft-Zerlegung)
	}
	} // host_rechnen
	if(gpu) {
		const FacKraft KH = K; // Host-Druckergebnis fuer den Pruefdruck sichern (nur unter pruef gerechnet)
		if(!D->kf_bound||D->kf_marker!=marker||D->kf_zper!=z_per) { // Erstbindung oder Schluesselwechsel
			if(!flags_aktuell&&!host_rechnen) L.flags.read_from_device(); // flags-Spiegel sicherstellen (host_rechnen hat ihn oben schon geholt)
			std::vector<ulong> liste; // Markerzellen in DERSELBEN Dreifachschleifen-Scan-Reihenfolge wie der Host-Pfad
			for(uint z=D->fbz0; z<D->fbz0+D->fbnz; z++) for(uint y=D->fby0; y<D->fby0+D->fbny; y++) for(uint x=D->fbx0; x<D->fbx0+D->fbnx; x++) {
				const ulong n = (ulong)x+((ulong)y+(ulong)z*(ulong)Ny)*(ulong)Nx;
				if(L.flags[n]!=marker) continue;
				liste.push_back(n);
			}
			D->bind_kraft_facetten(liste, marker, z_per);
		}
		double gpx=0.0, gpy=0.0, gpz=0.0; ulong gv=0ull, gq=0ull, gu=0ull;
		D->kraft_facetten_gpu(gpx, gpy, gpz, gv, gq, gu);
		K.px=gpx; K.py=gpy; K.pz=gpz; K.n_voll=gv; K.n_proj=gq; K.n_unklar=gu;
		if(!pruef) { K.ux=K.uy=K.uz=0.0; K.ukraft_ok=false; } // reiner GPU-Pfad: Zaehler ja, Kraftanteil nein
		if(pruef) {
			auto rel=[](const double a, const double b){ const double s=fmax(fabs(a),fabs(b)); return s>1e-12?fabs(a-b)/s:0.0; }; // R1-N3: symmetrisch wie relb -- sonst Scheinalarm 1,0 bei py~0
			const double rmax = fmax(rel(K.px,KH.px), fmax(rel(K.py,KH.py), rel(K.pz,KH.pz)));
			print_info("FAC_GPU-PRUEF: max. Relativabweichung px/py/pz = "+to_string((float)rmax,9u)
				+" (px GPU "+to_string((float)K.px,6u)+" / Host "+to_string((float)KH.px,6u)
				+", pz GPU "+to_string((float)K.pz,6u)+" / Host "+to_string((float)KH.pz,6u)
				+"); Zaehler GPU/Host: voll "+to_string(K.n_voll)+"/"+to_string(KH.n_voll)
				+", proj "+to_string(K.n_proj)+"/"+to_string(KH.n_proj)
				+", unklar "+to_string(K.n_unklar)+"/"+to_string(KH.n_unklar)
				+((K.n_voll!=KH.n_voll||K.n_proj!=KH.n_proj||K.n_unklar!=KH.n_unklar)?" -- ZAEHLER-DIFFERENZ (Achtung: an der l~0,5-Schwelle kann float-GPU vs double-Host ehrlich kippen -- erst Zellen ansehen, dann urteilen; Pruefagent N1)!":" (Zaehler exakt gleich)"));
		}
		if(zband>0u) { // ★ FORK Kraft-Zerlegung: Band-Druck (z<zband) auf dem zweiten Bindungs-Slot
			if(!D->kfb_bound||D->kfb_marker!=marker||D->kfb_zper!=z_per||D->kfb_zband!=zband) { // Erstbindung oder Schluesselwechsel (EIGENE kfb-Schluessel, Pruefagent M)
				if(!flags_aktuell&&!host_rechnen) L.flags.read_from_device(); // flags-Spiegel sicherstellen (ggf. doppelt mit der Hauptslot-Erstbindung -- einmalig beim Bind, toleriert)
				std::vector<ulong> liste_b; // dieselbe Dreifachschleifen-Scan-Reihenfolge; z>=zband wird HERAUSgefiltert (behalten wird das Band z<zband)
				for(uint z=D->fbz0; z<D->fbz0+D->fbnz; z++) for(uint y=D->fby0; y<D->fby0+D->fbny; y++) for(uint x=D->fbx0; x<D->fbx0+D->fbnx; x++) {
					if(z>=zband) continue;
					const ulong n = (ulong)x+((ulong)y+(ulong)z*(ulong)Ny)*(ulong)Nx;
					if(L.flags[n]!=marker) continue;
					liste_b.push_back(n);
				}
				D->bind_kraft_facetten(liste_b, marker, z_per, true);
				D->kfb_zband = zband;
			}
			double bpx=0.0, bpy=0.0, bpz=0.0; ulong bv=0ull, bq=0ull, bu=0ull;
			D->kraft_facetten_gpu(bpx, bpy, bpz, bv, bq, bu, true);
			K.pbx=bpx; K.pby=bpy; K.pbz=bpz;
			if(pruef) {
				auto relb=[](const double a, const double b){ const double s=fmax(fabs(a),fabs(b)); return s>1e-12?fabs(a-b)/s:0.0; }; // Pruefagent N: symmetrisch + absolute Untergrenze, sonst Dauerfehlalarm 1.0 bei ~0-Komponenten (py im Band)
				print_info("FAC_GPU-PRUEF Band (z<"+to_string(zband)+"): max. Relativabweichung px/py/pz = "
					+to_string((float)fmax(relb(K.pbx,hbx), fmax(relb(K.pby,hby), relb(K.pbz,hbz))),9u)
					+" (pz GPU "+to_string((float)K.pbz,6u)+" / Host "+to_string((float)hbz,6u)+"), Bandzellen GPU "+to_string(bv+bq+bu));
			}
		}
	}
	if(!gpu&&zband>0u) { K.pbx=hbx; K.pby=hby; K.pbz=hbz; } // reiner Host-Pfad: Band aus der Host-Mitschrift
	return K;
}

void main_setup_kanal() {
	// ---- Parameter. u_tau_lat = 0,003 fest (U_b ~ 0,072 nahe am Fahrzeug-u_lat 0,075).
	const uint  N        = env_u("CFD_KANAL_N", 38u);        // Zellen ueber die VOLLE Kanalhoehe
	const float Re_tau   = env_f("CFD_KANAL_RETAU", 5186.0f);
	const float utau_lat = env_f("CFD_KANAL_UTAU", 0.003f);
	const float Ub_plus_ziel = env_f("CFD_KANAL_UBPLUS", 24.104f); // Lee & Moser bei Re_tau 5186
	const float ett      = env_f("CFD_KANAL_ETT", 80.0f);    // Wirbelumschlagzeiten gesamt
	const float ett_warm = env_f("CFD_KANAL_WARM", 20.0f);   // davon verworfen
	// ★ I2 Torus-Kipp (FACETTEN-STUFE3.md): schraege Doppelwand, die ueber den z-Wrap auf sich
	// selbst schliesst -- exakt homogen, NULL Kernel-Aenderung. kipp==0 bleibt WORTGLEICH (jede
	// Formel via Ternaere mit dem alten Ausdrucksbaum -- Bitgleichheits-Pflicht des Kontrollarms).
	const uint kipp = env_u("CFD_KANAL_KIPP", 0u); // 0 / 45 / 26 (= atan(1/2) = 26,565 Grad)
	if(kipp!=0u&&kipp!=45u&&kipp!=26u) print_error("CFD_KANAL_KIPP kennt nur 0, 45 und 26.");
	if(kipp>0u&&env_u("CFD_WANDFUNKTION",0u)>0u) print_error("z-WFB an gekippten Waenden ist physisch falsch -- CFD_KANAL_KIPP nur mit CFD_FACETTEN.");
	const uint pk = 1u, qk = (kipp==26u) ? 2u : 1u;
	const float cosa = (kipp==45u) ? 0.70710678f : (kipp==26u) ? 0.89442719f : 1.0f;
	const uint Tv = (kipp==45u) ? 4u : (kipp==26u) ? 3u : 0u; // vertikale Slabdicke (F2)
	const uint  Ny = (kipp==0u) ? ((uint)round(3.14159265f*(0.5f*(float)N))/2u)*2u
	                            : env_u("CFD_KANAL_NY", (kipp==45u) ? 60u : 90u);
	const uint  Nz = (kipp==0u) ? (N+2u) : (pk*Ny/qk); // Einkanal-Schliessung q*Nz = p*Ny (F1)
	if(kipp>0u&&(Ny%qk!=0u||qk*Nz!=pk*Ny)) print_error("Torus-Schliessung verletzt: q*Nz muss p*Ny sein (Ny durch q teilbar waehlen).");
	const float delta_lat = (kipp==0u) ? 0.5f*(float)N : 0.5f*(float)(Nz-Tv)*cosa; // delta_eff (F3)
	const float dxp      = (kipp==0u) ? 2.0f*Re_tau/(float)N : Re_tau/delta_lat;   // dx+ (alt wortgleich)
	const float nu_lat   = (kipp==0u) ? utau_lat/dxp : utau_lat*delta_lat/Re_tau;  // (F4; alt wortgleich)
	const float Ub_ziel  = Ub_plus_ziel*utau_lat;            // Zielgeschwindigkeit (Gitter)
	const float f0       = utau_lat*utau_lat/delta_lat;      // Startkraft: tau_w = f*delta_eff exakt (F5)
	const uint  Nx = ((uint)round(2.0f*3.14159265f*delta_lat)/2u)*2u; // 2*pi*delta(_eff), gerade
	const float T_ett    = delta_lat/utau_lat;               // 1 Wirbelumschlag in Schritten
	const ulong n_steps  = (ulong)(ett*T_ett);
	const ulong n_warm   = (ulong)(ett_warm*T_ett);
	print_info("Kanal: "+to_string(Nx)+" x "+to_string(Ny)+" x "+to_string(Nz)+" = "+to_string((ulong)Nx*Ny*Nz)
		+" Zellen, Re_tau(Ziel) = "+to_string(Re_tau,0u)+", dx+ = "+to_string(dxp,1u)+", y+_1 = "+to_string(0.5f*dxp,1u));
	print_info("Kanal: tau = "+to_string(0.5f+3.0f*nu_lat,7u)+", U_b(Ziel) = "+to_string(Ub_ziel,4u)
		+", "+to_string(n_steps)+" Schritte ("+to_string(ett,0u)+" ETT, davon "+to_string(ett_warm,0u)+" Warmlauf)");
	// Randbedingungen der uebrigen Faelle hier AUSDRUECKLICH inert -- sagen, nicht annehmen:
	print_info("Kanal: kein TYPE_E, kein Druck-Auslass, keine Daempfungszone -- x/y sind PERIODISCH (Standard).");
	// R2 (Inventar stumm-inerter Schalter): was der Kanal prinzipbedingt ignoriert, wird angesagt.
	if(env_u("CFD_SPONGE_N",0u)>0u)      print_warning("CFD_SPONGE_N wird im Kanal NICHT angewandt (periodisch, kein Rand zu daempfen).");
	if(env_u("CFD_PO_HART",0u)>0u||getenv("CFD_PO_SIGMA")!=nullptr) print_warning("CFD_PO_HART/CFD_PO_SIGMA wirken nur mit Druck-Auslass -- der Kanal hat keinen."); // R2-Nachpruefer: env_u statt getenv-Falle
	if(env_on("CFD_SPARSE_TILES"))       print_warning("CFD_SPARSE_TILES wird im Kanal NICHT angewandt.");
	LBM_Domain::s_sponge_n = 0u; LBM_Domain::s_sponge_a = 3000.0f; LBM_Domain::s_sponge_wmin = 0.5f; LBM_Domain::s_sgs_wandfrei = env_u("CFD_SGS_WANDFREI", 0u)>0u; LBM_Domain::s_sgs_guo = env_u("CFD_SGS_GUO", 1u)>0u; LBM_Domain::s_sgs_diag = env_u("CFD_SGS_DIAG", 0u)>0u; LBM_Domain::s_sgs_diag_ab = (ulong)env_u("CFD_SGS_DIAG_AB", 0u);
	// ★ Wandfunktions-Bounce-Back: CFD_WANDFUNKTION=1 voll, =2 nur Free-Slip-Tausch (Zwischenarm).
	{ const uint wf = env_u("CFD_WANDFUNKTION", 0u); LBM_Domain::s_wandfunktion = wf>0u; LBM_Domain::s_wf_tau = (wf==2u) ? 0.0f : 1.0f;
	  if(wf>0u) print_info(string("Wandfunktion (Han et al. 2021): ")+(wf==2u?"NUR FREE-SLIP-TAUSCH (Zwischenarm)":"voll (Spalding, kappa=0,41, B=5,5)")); }
	// ★ C1b: Facettenpfad am Kanal. Arme 1/2 = Paartausch (Kontrollarm), 3/4 = iMEM (FACETTEN-IMEM.md).
	{ const uint fc = env_u("CFD_FACETTEN", 0u);
	  if(fc>4u) print_error("CFD_FACETTEN kennt nur 0..4 (1/2 Paartausch voll/Tausch, 3/4 iMEM voll/Nullziel).");
	  LBM_Domain::s_facetten = fc>0u; LBM_Domain::s_fac_imem = fc>=3u;
	  LBM_Domain::s_fac_ema = (fc>=3u) ? env_f("CFD_FAC_EMA", 0.0f) : 0.0f;
	  LBM_Domain::s_fac_pema = (fc>=3u) ? env_f("CFD_FAC_PEMA", 0.0f) : 0.0f;
	  LBM_Domain::s_fac_diagz = (fc>=3u&&getenv("CFD_FAC_DIAGZ")!=nullptr) ? (long)atoll(getenv("CFD_FAC_DIAGZ")) : -1l;
	  LBM_Domain::s_fac_pinv = env_u("CFD_FAC_PINV", 0u); if(LBM_Domain::s_fac_pinv>0u&&env_u("CFD_FAC_LSQ",0u)>0u) print_error("CFD_FAC_PINV und CFD_FAC_LSQ schliessen sich aus -- PINV ersetzt denselben Zweig, LSQ waere still wirkungslos (der #elif faellt durch). Einen von beiden waehlen."); if(LBM_Domain::s_fac_pinv>0u) print_info("RANG-1-PSEUDOINVERSE (CFD_FAC_PINV, 04.09.2026): im gekoppelten Zweig ersetzt Moore-Penrose die achsenparallele Skalarleiter -- Division ueber die SPUR (groesster Eigenwert) statt ueber Gt11. Grund: fuer die ebene Voxelwand ist tr(Gt) exakt 1/3 und kippungsunabhaengig, waehrend Gt11 mit der Stroemungsrichtung gegen 0 laeuft und die Akzeptanzschwelle 1e-4 dann Verstaerkung bis 1e4 durchlaesst. Wirkpfad Slot 80; erwartet fallen Slot 10 UND Slot 16, weil der Eigenvektor Sn.v = 0 exakt erfuellt und damit keine Normalkompensation mehr erzeugt wird."); LBM_Domain::s_fac_idx_voll = env_u("CFD_FAC_IDX_VOLL", 0u); if(LBM_Domain::s_fac_idx_voll>0u) print_info("CFD_FAC_IDX_VOLL=1: fac_idx in der ALTEN Vollfeldform -- deklarierter A/B-Arm gegen die Bitmaske (03.09.). Die Ergebnisse MUESSEN bitgleich sein, unterscheiden darf sich nur der Speicher."); LBM_Domain::s_f_liste = env_u("CFD_F_LISTE", 0u); if(LBM_Domain::s_f_liste>0u) print_info("F-MARKERLISTE (CFD_F_LISTE, 03.09.2026, Befunde B78b/B80/B81): F wird nur fuer WANDsolidzellen alloziert -- 8 mm gemessen: Nahfeld 238 -> 14 MiB, Fernfeld 4 -> 0 MiB. ABGENOMMEN ueber alle drei Sprossen bitgleich (CPU 5/5, iGPU 5/5 und dreimal reproduziert, B70 8-mm-Fahrzeug 19/19), Slot 77 = 0. Der urspruengliche Defekt war NICHT die Liste, sondern die Reihenfolge: die JIT-Defines entstanden vor dem Setzen der Schalter (B81)."); if(LBM_Domain::s_f_liste>0u&&!f_nur_solid_an_setup()) print_error("CFD_F_LISTE braucht CFD_F_NUR_SOLID (Default an): der Kontrollarm CFD_F_NUR_SOLID=0 liest F an JEDER Fluidzelle, und dort gibt es unter der Markerliste keinen Speicherplatz mehr -- die Kombination waere still falsch."); LBM_Domain::s_fac_satgate = fc>=3u&&env_u("CFD_FAC_SATGATE", 0u)>0u; LBM_Domain::s_fac_kraft = fc>=3u ? min(2u, env_u("CFD_FAC_KRAFT", 0u)) : 0u; LBM_Domain::s_fac_kdiag = fc>=3u ? env_u("CFD_FAC_KDIAG", 0u) : 0u; if(fc<3u&&(env_u("CFD_FAC_NACHBAR",0u)>0u||env_u("CFD_FAC_KDIAG",0u)>0u)) print_error("CFD_FAC_NACHBAR/CFD_FAC_KDIAG brauchen CFD_FACETTEN=3 (iMEM) -- bei CFD_FACETTEN="+to_string((ulong)fc)+" wuerde der Schalter still auf 0 gesetzt (No-Op-Waechter 03.09.)."); LBM_Domain::s_fac_nachbar = fc>=3u ? env_u("CFD_FAC_NACHBAR", 0u) : 0u; LBM_Domain::s_fac_messnur = fc>=3u ? env_u("CFD_FAC_MESSNUR", 0u) : 0u; LBM_Domain::s_sgs_gdiag = fc>=1u ? env_u("CFD_SGS_GDIAG", 0u) : 0u; LBM_Domain::s_sgs_fdwand = fc>=1u ? env_u("CFD_SGS_FDWAND", 0u) : 0u; if(LBM_Domain::s_sgs_fdwand>0u) { print_info("SGS-GEISTERMODEN-FIX (CFD_SGS_FDWAND, 02.09.): an Facettenzellen kommt die SGS-Relaxationsrate aus |S|_FD des u-Felds (FD-Kernel je Schritt, ein Schritt Versatz) statt aus dem wandmodell-kontaminierten Pi-Tensor (B66/B69: Pi/FD 2,3-3,4). Wirkpfad Slot 76 (B70)."); if(env_u("CFD_FAC_MESSNUR",0u)>0u) print_warning("SGS_FDWAND + MESS-NUR: der Arm ist dann NICHT mehr reines Bounce-Back -- das Kollisions-w an Wandzellen kommt aus dem FD-Pfad (bewusste Kombination fuer BB+FDWAND-Messungen, aber nicht mit alten BB-Bezuegen bitvergleichbar)."); if(env_u("CFD_SGS_WANDFREI",0u)>0u) print_warning("SGS_FDWAND + SGS_WANDFREI: WANDFREI hat an Wandzellen VORRANG -- FDWAND ist dort wirkungslos (Slot 76 bleibt 0). Fuer den FDWAND-Arm WANDFREI abschalten."); } if(LBM_Domain::s_sgs_gdiag>0u) { sgs_gdiag_selbsttest(); print_info("g-DIAGNOSE (CFD_SGS_GDIAG, 31.08.): Messkernel ueber die Wandzellen -- |S|_FD (u-Feld, geistermodenfrei), |S|_Pi (fneq, wie Smagorinsky), D_WALE, D_Sigma, |Omega|. Physik unangetastet, Bericht am Laufende."); } if(LBM_Domain::s_fac_messnur>0u&&env_u("CFD_FAC_NACHBAR",0u)>0u) print_warning("MESS-NUR + NACHBAR: die Nachbarabtastung liegt hinter dem MESS-NUR-Ausstieg und ist WIRKUNGSLOS (Slots 72-74 bleiben 0)."); if(LBM_Domain::s_fac_messnur>0u&&env_u("CFD_FAC_KDIAG",0u)>0u) print_warning("MESS-NUR + KDIAG: die Klassen-Diagnostik wird nie akkumuliert -- die Tabelle am Laufende ist eine Nulltabelle."); if(LBM_Domain::s_fac_messnur>0u&&env_u("CFD_FAC_KRAFT",0u)>0u) print_error("MESS-NUR + KRAFT ist unsinnig (kein Wandmodell -> kein Residuum; Modus 2 stuerbe irrefuehrend am Kraftpfad-Pruefer). Kombination aufloesen."); if(LBM_Domain::s_fac_messnur>0u) print_warning("MESS-NUR (CFD_FAC_MESSNUR, 30.08.): der Kernel wendet KEIN Wandmodell an -- die Wand ist reines Bounce-Back. Facetten werden nur gebaut und gemessen, damit der Druckpfad (cd_facetten.csv) als Aepfel-mit-Aepfeln-Bezug zu einem Wandmodell-Arm dient. Der REIBUNGSanteil ist in diesem Arm konstruktiv 0; belastbar ist der Druckanteil (bei Cz 99,5 %). Slot 75 = Wirkpfad (2. Umzug, B70). Die ELIBB-Blende wird unter MESS-NUR seit B-4 ebenfalls uebersprungen -- der Arm ist exakt reines Bounce-Back."); if(LBM_Domain::s_fac_nachbar>0u) print_info("NACHBARABTASTUNG (CFD_FAC_NACHBAR, 30.08.): Wandmodell-Eingang (u_t, Wandabstand) aus der zweiten Fluidzelle entlang der Normale statt aus der Wandzelle -- Stufenschatten-Fix. Slots 72 (angewandt) / 73 (kein Fluidnachbar) / 74 (Nachbar steht still; 2. Umzug 02.09., B70 -- 35-48 gehoeren SGS_DIAG ueber berechnete Indizes)."); if(LBM_Domain::s_fac_kraft>0u) print_info(string("iMEM-KRAFTPFAD (Weg F, 30.08.): Modus ")+to_string(LBM_Domain::s_fac_kraft)+(LBM_Domain::s_fac_kraft==1u?string(" -- Residuum R als Volumenkraft an RUECKFALLZELLEN (statt s=0); Slot 70, Soll == Slot 69."):string(" -- ALLE Facettenzellen per Kraft, Additivterm aus (Diskriminator gegen den Slip-Pfad); Slot 70."))); if(LBM_Domain::s_fac_kraft>0u) print_warning("KRAFTPFAD (Pruefpunkt 8, 30.08.): object_force/forces.csv (Impulsaustausch an Koerperzellen) sieht die Volumenkraft NICHT -- eine Guo-Kraft im Fluid hat keine Newton-3-Reaktion am Koerper. Der Reibungsanteil an Kraftzellen steht allein in der fac_tau-Buchung (cd_reib/cd_rest); object_force-Abgleiche (K4, Fx_far) weichen um genau den Kraftanteil ab.");
	  if(LBM_Domain::s_fac_satgate) print_info("iMEM-Saettigungs-Gate aktiv (a-strich): Budget-Riss -> BB-Rueckfall statt Klemme (Slots 10/16 = Rueckfaelle; seit Buchungsschluss 27.08. buchen Rueckfaelle P-only, Slot 69).");
	  LBM_Domain::s_fac_alpha = (fc>=3u) ? env_u("CFD_FAC_ALPHA", 0u) : 0u; LBM_Domain::s_fac_lsq = env_u("CFD_FAC_LSQ", 0u)>0u; LBM_Domain::s_fac_quergate = env_u("CFD_FAC_QUERGATE", 0u)>0u; LBM_Domain::s_fac_elibb = env_u("CFD_FAC_ELIBB", 0u)>0u; LBM_Domain::s_fac_elibb_pur = env_u("CFD_FAC_ELIBB", 0u)==2u; LBM_Domain::s_fac_qmin = env_f("CFD_FAC_QMIN", 0.1f); LBM_Domain::s_fac_kappa = env_f("CFD_FAC_KAPPA", 0.4f); LBM_Domain::s_fac_utkorr = env_f("CFD_FAC_UTKORR", 1.0f); LBM_Domain::s_fac_qkappe = env_f("CFD_FAC_QKAPPE", 1.0f); LBM_Domain::s_fac_qdiag = env_u("CFD_FAC_QDIAG", 0u); LBM_Domain::s_sgs_guo = env_u("CFD_SGS_GUO", 1u)>0u;
	  if(LBM_Domain::s_fac_alpha>2u) print_error("CFD_FAC_ALPHA kennt nur 0..2 (1 = Massenkorrektur, 2 = + Momenten-Downdate). Die Stufe 3 (A2-Rueckfall) wurde am 2026-08-25 als beweisbar wirkungslos zurueckgenommen -- q_i ist fuer Einzellink-Facetten unter alpha identisch null.");
	  LBM_Domain::s_fac_apg = (fc>=3u) ? env_f("CFD_FAC_APG", 0.0f) : 0.0f;
	  if(LBM_Domain::s_fac_apg!=0.0f&&LBM_Domain::s_fac_pema>0.0f) print_error("CFD_FAC_APG + CFD_FAC_PEMA sind noch NICHT kombiniert (gefilterte Kette braucht eigenen APG-Zweig -- eigener Bauabschnitt).");
	  if(LBM_Domain::s_fac_apg!=0.0f) print_info("APG-Messarm aktiv: tw-Ziel um kappa*y_w*dp/ds korrigiert, kappa = "+to_string(LBM_Domain::s_fac_apg,4u)+" -- Slot 19 zaehlt beide Klemmen (0 / 2*tw).");
	  if(LBM_Domain::s_fac_alpha>0u) print_info(string("iMEM-alpha-Massenkorrektur Stufe ")+to_string(LBM_Domain::s_fac_alpha)+(LBM_Domain::s_fac_alpha==2u?string(" (Masse + Momenten-Downdate: Impulsziel inkl. alpha exakt)"):string(" (NUR Masse -- injiziert alpha*S1-Impuls, reiner Messarm)"))+" -- Slot 18 zaehlt alpha>u_t.");
	  if(LBM_Domain::s_fac_ema>0.0f) print_warning("CFD_FAC_EMA (Loesungs-Filterung) ist in J3 WIDERLEGT -- nur noch als A/B-Arm sinnvoll.");
	  if(LBM_Domain::s_fac_ema>0.0f&&LBM_Domain::s_fac_pema>0.0f) print_warning("CFD_FAC_EMA und CFD_FAC_PEMA GLEICHZEITIG: zwei kompoundierende Lags -- als Messarm wertlos (IR3-Audit).");
	  if(LBM_Domain::s_fac_pema>0.0f) print_info("iMEM-PEMA aktiv (Weg A, Eingangs-Filterung): alpha = "+to_string(LBM_Domain::s_fac_pema,5u)+", Zeitkonstante ~"+to_string((uint)(1.0f/LBM_Domain::s_fac_pema))+" Schritte");
	  LBM_Domain::s_fac_tau = (fc==2u||fc==4u) ? 0.0f : 1.0f;
	  LBM_Domain::s_fac_budget = fmax(0.25f, fmin(4.0f, env_f("CFD_FAC_BUDGET", 1.0f)));       // 1a-B4t: Tangentialbudget-Skalar (geklemmt 0,25..4; Ansage unten)
	  LBM_Domain::s_fac_budget_sn = fmax(0.25f, fmin(4.0f, env_f("CFD_FAC_BUDGET_SN", 1.0f))); // 1a-Bsn: sn-Budget-Skalar
	  if(LBM_Domain::s_fac_budget!=1.0f) print_info("FACETTEN BUDGET (1a-B4t): Tangentialbudget x "+to_string(LBM_Domain::s_fac_budget,2u)+" (|s1| <= 2ut*k, |s2| <= ut*k). Die +-2ut-Budgets sind Design, nie geeicht (Planungsagent 2026-08-22). Erfolgskriterium: Slot-10-Anteil faellt UND cd_druck/cz_rest Richtung OF13 UND y+-Median nicht > +15 %.");
	  if(LBM_Domain::s_fac_budget_sn!=1.0f) print_info("FACETTEN BUDGET_SN (1a-Bsn): sn-Budget x "+to_string(LBM_Domain::s_fac_budget_sn,2u)+". Verschlechtert sich cd_druck > 2 %, ist der Arm verworfen (sn beruehrt den Druckpfad).");
	  LBM_Domain::s_boden_eq_n = 0u; LBM_Domain::s_boden_eq_down = 0u; LBM_Domain::s_boden_eq_split = 0xFFFFFFFFu; LBM_Domain::s_boden_eq_abstand = 0u; LBM_Domain::s_einlass_eq_n = 0u; LBM_Domain::s_schale_alpha = 0.0f; LBM_Domain::s_schale_paritaet = false; if(getenv("CFD_BODEN_EQ")||getenv("CFD_BODEN_EQ_DOWN")||getenv("CFD_BODEN_EQ_ABSTAND")||getenv("CFD_FERN_BODEN_EQ")||getenv("CFD_FERN_EINLASS_EQ")) print_warning("Die BODEN_EQ/EINLASS_EQ-Familie wird im kanal NICHT angewandt (parallele Waende, periodisches x)."); // B3/R3
	if(getenv("CFD_KOPPLUNG_ZEITINTERP")||getenv("CFD_KOPPLUNG_GLATT")) print_warning("CFD_KOPPLUNG_ZEITINTERP/GLATT werden im kanal NICHT angewandt (nur fahrzeug_dd; M3).");
	if(getenv("CFD_SLICE_NEAR_STEPS")||getenv("CFD_VTK_JEDE")||getenv("CFD_VTK_BEHALTE")) print_warning("CFD_SLICE_NEAR_STEPS/CFD_VTK_JEDE/CFD_VTK_BEHALTE werden in diesem Fall NICHT angewandt (nur fahrzeug_dd; Kadenz-Umbau 27.08.).");
	  { const char* n2f_[] = {"CFD_N2F_SCHALE","CFD_N2F_VOLUMEN","CFD_N2F_BAND","CFD_N2F_BAND_N","CFD_N2F_BAND_PROFIL","CFD_N2F_BAND_UNTERBODEN","CFD_N2F_BAND_WAKE","CFD_N2F_BAND_NURWAKE","CFD_N2F_BAND_WAKE_START","CFD_N2F_BAND_WAKE_START_X","CFD_N2F_BAND_WAKE_ABSTAND","CFD_N2F_PARITAET"}; for(const char* b : n2f_) if(getenv(b)) print_warning(string(b)+" ist gesetzt, wird aber NUR im fahrzeug_dd-Fall angewandt (P9c; die neun BAND-/WAKE-/PARITAET-Schalter fehlten bis 2026-08-22 in dieser Ansage -- Pruefagent-S1)."); } // Ansage-Doktrin
	  if(env_u("CFD_FERN_FACETTEN", 0u)>0u) print_warning("CFD_FERN_FACETTEN wird im kanal NICHT angewandt (nur fahrzeug_dd -- P8; Ansage-Doktrin).");
	  if(fc>0u) print_info(string("Facettenpfad: ")+(fc==1u?"Paartausch voll (Kontrollarm)":fc==2u?"Paartausch NUR TAUSCH":fc==3u?"iMEM voll (Slip-Velocity-BB)":"iMEM NULLZIEL (tau=0)")); }
	const string out_dir = get_exe_path()+"../export/"+(getenv("CFD_RUN_NAME")?string(getenv("CFD_RUN_NAME")):string("kanal"))+"/";
	create_folder(out_dir);
	sichere_lauf(out_dir, "kanal");

	LBM lbm(Nx, Ny, Nz, nu_lat, f0, 0.0f, 0.0f);
	// ---- Waende + Anfangsfeld: Reichardt-Profil + divergenzfreie Stoerung (kein weisses Rauschen --
	// bei tau nahe 0,5 sind akustische Moden praktisch ungedaempft, eine nicht divergenzfreie
	// Stoerung klingelt sehr lange; die Lehre aus dem Fernfeld).
	const float kappa=0.41f;
	auto reichardt = [&](const float yp) {
		return log(1.0f+kappa*yp)/kappa + 7.8f*(1.0f-exp(-yp/11.0f)-(yp/11.0f)*exp(-0.33f*yp));
	};
	const float ph = env_f("CFD_KANAL_PHASE", 0.0f); // Stoerphase fuer Wiederholbarkeitsmessungen (0.0 = numerisch identisch: x+0.0f==x)
	// Torus-Kipp: solid nach F1 (ganzzahlig exakt), Reichardt ueber den SENKRECHTEN Wandabstand d_perp
	// (F6); Stoerungs-u_z ist nicht wandtangential -- klingt im Warmlauf ab (Formelblatt Schritt 2).
	auto kipp_solid = [&](const uint xx, const uint yy, const uint zz) {
		(void)xx; const long g = ((long)qk*(long)zz-(long)pk*(long)yy)%((long)qk*(long)Nz);
		return (g+(long)qk*(long)Nz)%((long)qk*(long)Nz) < (long)qk*(long)Tv; };
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(kipp>0u) {
			if(kipp_solid(x,y,z)) { lbm.flags[n] = TYPE_S; continue; }
			const long m = (((long)qk*(long)z-(long)pk*(long)y)%((long)qk*(long)Nz)+(long)qk*(long)Nz)%((long)qk*(long)Nz)-(long)qk*(long)Tv;
			const float dperp = ((float)m+0.5f)/sqrt((float)(pk*pk+qk*qk));
			const float zwk = fmin(dperp, 2.0f*delta_lat-dperp);
			const float upk = reichardt(zwk*(utau_lat/nu_lat));
			const float whk = fmin(dperp/delta_lat, 2.0f-dperp/delta_lat);
			const float Ak = 0.10f*Ub_ziel, kxk = 2.0f*3.14159265f/(float)Nx;
			const float szk = sin(0.5f*3.14159265f*whk);
			lbm.u.x[n] = upk*utau_lat + Ak*szk*szk*cos(kxk*(float)x+ph);
			lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
			continue;
		}
		if(z==0u || z==Nz-1u) { lbm.flags[n] = TYPE_S; continue; } // ruhende Waende, u bleibt 0
		const float zw   = fmin((float)z-0.5f, (float)(Nz-1u)-0.5f-(float)z); // Wandabstand (Halfway)
		const float up   = reichardt(zw*dxp);
		const float zh   = ((float)z-0.5f)/delta_lat; // 0..2 ueber die Kanalhoehe
		// Stromfunktions-artige Stoerung. ★ Nachpruefer-Befund: NICHT exakt divergenzfrei, wie hier
		// zuvor behauptet -- im z-Term steht eine Naeherung statt der Stammfunktion von sin^2. Der
		// Divergenzrest ist O(A*kx) und klingt im Warmlauf ab; fuer c_f nach 20 ETT ohne Belang,
		// aber die Behauptung war falsch und der Anspruch (kein Akustik-Klingeln saeen) nur teilweise erfuellt.
		const float A = env_f("CFD_KANAL_STOER", 0.10f)*Ub_ziel, kx = 2.0f*3.14159265f/(float)Nx, ky2 = 2.0f*3.14159265f*2.0f/(float)Ny; // ★ 31.08.: Stoeramplitude schaltbar -- CFD_KANAL_STOER=0 liefert das UNGESTOERTE Reichardt-Profil als analytischen Testfall der g-Diagnose (D_WALE = D_Sigma = 0, |Omega| = |S|)
		const float sz = sin(0.5f*3.14159265f*fmin(zh, 2.0f-zh));
		lbm.u.x[n] = up*utau_lat + A*sz*sz*cos(kx*(float)x+ph)*cos(ky2*(float)y);
		lbm.u.z[n] = -A*kx/(0.5f*3.14159265f/delta_lat+1e-9f)*0.5f*sin(kx*(float)x+ph)*sz*cos(ky2*(float)y);
		lbm.u.y[n] = 0.0f;
	}
	{	// Ma-Kontrolle des Anfangsfelds -- eine ueberschnelle Zelle im Startfeld waere ein stiller
		// Parameterfehler, der erst nach Minuten als Divergenz auffiele.
		float umax=0.0f; for(ulong n=0ull;n<lbm.get_N();n++) umax=fmax(umax,fabs(lbm.u.x[n])+fabs(lbm.u.y[n])+fabs(lbm.u.z[n]));
		print_info("Kanal: max|u| im Anfangsfeld = "+to_string(umax,4u)+" (Grenze c_s = 0,577)");
		if(umax>0.4f) print_error("Anfangsfeld zu schnell (max|u| = "+to_string(umax,4u)+") -- CFD_KANAL_UTAU oder CFD_KANAL_UBPLUS pruefen.");
	}
	// ★ C1b Stufe 1, Anker-Test: am parallelwandigen Kanal MUESSEN alle Facetten exakt n=ez,
	// y_w=0,500, Klasse 0 liefern (FACETTEN-PLAN A3, Gegenpruefer-verifiziert). =2: nur Diagnose.
	if(env_u("CFD_FACETTEN_DIAG", 0u)>0u) {
		// Randstreifen x/y (1..N-2-Ausschluss, kein periodischer Wrap) fehlen bewusst: ~5 % Diagnose-only (Nachpruefer B5).
		const string cdir = out_dir+"census/"; create_folder(cdir); // Gross-Audit N21: nicht den Echtbau-Census ueberschreiben
		baue_facetten(lbm, Nx, Ny, Nz, TYPE_S, cdir, kipp>0u?"Kipp-Census":"Kanal-Anker", kipp>0u);
		if(env_u("CFD_FACETTEN_DIAG", 0u)==2u) _exit(0);
	}
	std::vector<Facette> FF; // Funktionsscope: der Klassenbericht am Laufende braucht die Liste (Weg-1 Stufe 0)
	if(env_u("CFD_FACETTEN", 0u)>0u) {
		FF = baue_facetten(lbm, Nx, Ny, Nz, TYPE_S, out_dir, kipp>0u?"Torus-Kipp":"Kanal", kipp>0u);
		lbm.lbm_domain[0]->alloc_f_liste(&lbm.flags[0], Nx, Ny, Nz); // ★ 03.09. F-Markerliste (steigt bei CFD_F_LISTE=0 selbst aus) -- im Kanal verdrahtet, damit der Listenarm auf CPU/iGPU pruefbar ist und nicht zuerst auf der B70 laufen muss
		lbm.alloc_facetten(FF, nullptr, env_u("CFD_SGS_GDIAG", 0u), env_u("CFD_SGS_FDWAND", 0u)); // Parameter statt Statik (02.09.)
		// F2/F7: Facettenzahl ist geometrisch exakt abzaehlbar -- harte Pruefung faengt jeden
		// vergessenen z-Wrap mechanisch (Formelblatt Schritt 5).
		// GESAMTzahl (aktiv + markiert) ist die geometrische F2-Invariante -- fac_N allein zaehlt nur
		// die aktiven (der erste Serienlauf starb an genau dieser Verwechslung).
		if(kipp==45u&&FF.size()!=(size_t)4u*Nx*Ny) print_error("Torus 45: Facetten gesamt != 4*Nx*Ny ("+to_string((ulong)FF.size())+" statt "+to_string((ulong)4u*Nx*Ny)+").");
		if(kipp==26u&&FF.size()!=(size_t)3u*Nx*Ny) print_error("Torus 26: Facetten gesamt != 3*Nx*Ny ("+to_string((ulong)FF.size())+" statt "+to_string((ulong)3u*Nx*Ny)+").");
	}
	lbm.run(0u, n_steps); // initialisieren

	// ---- Zeitschleife mit CFR-Regler und Ebenenstatistik
	std::ofstream zcsv(out_dir+"kanal_zeit.csv"); zcsv.precision(8);
	// ★ Audit-Nacharbeit 1 (hoch): unter WANDFUNKTION ist cf_impulsaustausch PRINZIPIELL UNGUELTIG --
	// update_force_field setzt reine Reflexion voraus ("2x because fi are reflected"), die WFB
	// ersetzt genau diese Links durch Tausch+tau. Das Kriterium "beide c_f-Wege gleich" gilt NUR
	// im Arm ohne Wandfunktion. Der Planungsagent hatte diese Kennzeichnung angewiesen; sie fehlte.
	if(env_u("CFD_WANDFUNKTION", 0u)>0u||env_u("CFD_FACETTEN", 0u)>0u) zcsv << "# ACHTUNG: cf_impulsaustausch unter WANDFUNKTION/FACETTEN UNGUELTIG (Reflexionsannahme verletzt) -- nur cf_kraftbilanz zaehlt\n";
	zcsv << "# Hinweis: f_lat ist das FRISCH geregelte f (wirkt ab dem Folgechunk); cf_kraftbilanz rechnet mit dem im Chunk WIRKENDEN f -- Rueckrechnung cf=2*f_lat*delta/Ub^2 weicht deshalb ab\n";
	zcsv << "schritt,ett,Ub_lat,Ub_plus,f_lat,cf_kraftbilanz,cf_impulsaustausch\n" << std::flush;
	const uint regel_alle = 100u;
	float f_akt = f0, Ub_alt = Ub_ziel;
	// Statistik-Akkumulatoren je z-Ebene (double, Host)
	std::vector<double> su(Nz,0.0), suu(Nz,0.0), sww(Nz,0.0), suw(Nz,0.0), suz(Nz,0.0); ulong n_stat=0ull;
	const float K = env_f("CFD_KANAL_K", 0.05f); // Reglerverstaerkung, bewusst trraege
	std::vector<double> fac_snap; std::vector<double> fac_snap_tw; std::vector<ulong> fac_snap_n; // ★ 2026-08-25 Pruefbefund C(c): tw (Slot 0) und der Besuchszaehler wurden NIE geschnappt
	ulong fac_snap_step=0ull; double fac_fsum=0.0, fac_fn=0.0; // Cd-Pfad (K2/K3)
	std::ofstream diag_csv; // Iron Rule 3: Diagnose-Facetten-Zeitreihe
	for(ulong step=0ull; step<n_steps; step+=(ulong)regel_alle) { // Audit-Nacharbeit 18: letzter Chunk gekappt, vorher bis zu 99 Schritte Ueberzug
		const ulong chunk = min((ulong)regel_alle, n_steps-step); // Re-Audit R2: auch fuers CSV-Etikett verwenden
		lbm.run(chunk, n_steps); // run(steps, total) kappt selbst NICHT (2. Arg ist nur Laufzeitschaetzung)
		lbm.lbm_domain[0]->sgs_gdiag_gpu(); // ★ g-Diagnose: ein Mess-Launch je Chunk (no-op ohne CFD_SGS_GDIAG)
		// U_b und Ebenensummen aus u (Vollread -- bei diesen Groessen billig: N=38 -> 3,4 MB)
		lbm.u.read_from_device();
		double Ub=0.0; ulong nf=0ull;
		std::vector<double> pu(Nz,0.0), puu(Nz,0.0), pww(Nz,0.0), puw(Nz,0.0), pz_(Nz,0.0);
		for(ulong n=0ull; n<lbm.get_N(); n++) {
			uint x=0u,y=0u,z=0u; lbm.coordinates(n,x,y,z);
			if(kipp>0u) { if(lbm.flags[n]==TYPE_S) continue; } // Torus: Fluid per Flag (F7-Schritt 7)
			else if(z==0u||z==Nz-1u) continue;
			const double ux=(double)lbm.u.x[n], uz=(double)lbm.u.z[n];
			Ub+=ux; nf++; pu[z]+=ux; puu[z]+=ux*ux; pww[z]+=uz*uz; puw[z]+=ux*uz; pz_[z]+=uz;
		}
		Ub/=(double)nf;
		const double Ub_plus = Ub/(double)utau_lat;
		// ★ Relaminarisierungs-/Entgleisungswaechter
		if(Ub_plus>200.0) { print_error("Kanal relaminarisiert oder entgleist: U_b+ = "+to_string((float)Ub_plus,1u)+" (ueber 200). Lauf wertlos."); }
		// ★ Audit-Nacharbeit 12: tau_kraft aus dem f bilden, das in DIESEM Chunk gewirkt hat --
		// vorher stand hier das frisch geregelte f_akt (Ein-Chunk-Versatz im cf_kraftbilanz).
		const float f_wirk = f_akt;
		// CFR-Regler
		const float f_neu = f_akt + K*(float)((Ub_ziel-(float)Ub) + (Ub_alt-(float)Ub))*utau_lat*utau_lat/delta_lat/fmax(1e-12f,utau_lat);
		Ub_alt=(float)Ub; f_akt=fmax(0.0f, f_neu); lbm.set_fx(f_akt);
		// zwei unabhaengige tau_w
		const double tau_kraft = (double)f_wirk*(double)delta_lat;              // exakt: f*delta
		lbm.update_force_field();
		const float3 Fw = lbm.object_force(TYPE_S);                             // beide Waende
		const double tau_mem = fabs((double)Fw.x)/(2.0*(double)Nx*(double)Ny);  // je Flaeche
		const double cf_k = 2.0*tau_kraft/fmax(1e-30,Ub*Ub), cf_m = 2.0*tau_mem/fmax(1e-30,Ub*Ub);
		zcsv << (step+chunk) << "," << (double)(step+chunk)/(double)T_ett << "," << Ub << "," // R2: gelaufene Schritte, nicht nominelle (letzter Chunk ist gekappt)
		     << Ub_plus << "," << f_akt << "," << cf_k << "," << cf_m << "\n" << std::flush;
		// Statistik erst nach dem Warmlauf akkumulieren
		if(step>=n_warm) { for(uint z=0u;z<Nz;z++){su[z]+=pu[z];suu[z]+=puu[z];sww[z]+=pww[z];suw[z]+=puw[z];suz[z]+=pz_[z];} n_stat+=(ulong)Nx*Ny; }
		// Iron Rule 3: Diagnose-Facette je Chunk in CSV sampeln
		if(LBM_Domain::s_fac_diagz>=0l&&lbm.lbm_domain[0]->fac_diagz_on) {
			lbm.lbm_domain[0]->fac_diag.read_from_device();
			if(!diag_csv.is_open()) { diag_csv.open(out_dir+"facetten_diagz.csv"); diag_csv << "schritt,ut,twe,P1,P2,s1,s2,sn,phi1,phi2,G11,G22,Snn,Sn1,Sn2,t_kernel,rhon,alpha,dp_ds\n"; }
			diag_csv << (step+chunk);
			for(uint q=0u;q<16u;q++) diag_csv << "," << lbm.lbm_domain[0]->fac_diag[q];
			diag_csv << "," << lbm.lbm_domain[0]->fac_diag[17] << "," << lbm.lbm_domain[0]->fac_diag[18] << "\n" << std::flush; // [17] alpha, [18] dp_ds (0 im Aus-Arm)
		}
		// ★ Cd-Pfad: Akkumulator-Snapshot am Warmup-Ende, f_wirk-Mittel ab dort (K2-Referenz)
		if(env_u("CFD_FACETTEN",0u)>0u&&step>=n_warm) {
			// ★ Audit R3 (MITTEL): der Snapshot steht NACH lbm.run(chunk) bei step+chunk Schritten --
			// vorher etikettierte fac_snap_step=step das Fenster einen Chunk zu frueh, und f_wirk des
			// Snapshot-Chunks ging in die K2-Referenz ein, obwohl sein Delta nicht im Akkumulator liegt.
			if(fac_snap.empty()) { fac_snap_step=step+chunk;
				lbm.lbm_domain[0]->fac_tau.read_from_device();
				fac_snap.resize(3ull*lbm.lbm_domain[0]->fac_N);
				fac_snap_tw.resize(lbm.lbm_domain[0]->fac_N); fac_snap_n.resize(lbm.lbm_domain[0]->fac_N);
				lbm.lbm_domain[0]->fac_tau_n.read_from_device();
				for(ulong i=0ull;i<lbm.lbm_domain[0]->fac_N;i++){ fac_snap[3ull*i]=(double)lbm.lbm_domain[0]->fac_tau[6ull*i+1ull]; fac_snap[3ull*i+1ull]=(double)lbm.lbm_domain[0]->fac_tau[6ull*i+2ull]; fac_snap[3ull*i+2ull]=(double)lbm.lbm_domain[0]->fac_tau[6ull*i+3ull];
					fac_snap_tw[i]=(double)lbm.lbm_domain[0]->fac_tau[6ull*i]; fac_snap_n[i]=(ulong)lbm.lbm_domain[0]->fac_tau_n[i]; } }
			else { fac_fsum+=(double)f_wirk*(double)chunk; fac_fn+=(double)chunk; }
		}
	}
	// ---- Profil + Spannungsbilanz
	std::ofstream pcsv(out_dir+"kanal_profil.csv"); pcsv.precision(8);
	// ★★ NACHPRUEFER-BEFUND 2026-08-15: hier wurde mit dem ZIEL-u_tau normiert statt mit dem
	// GEMESSENEN. Bei c_f-Abweichung ist das ein systematischer Skalenfehler in ALLEN +-Groessen
	// (im ersten Lauf Faktor 1,78: der Lauf sass real bei Re_tau ~ 9200 und y+_1 ~ 243, nicht
	// 5186/137), und die Spannungsbilanz KONNTE mit den Spalten nicht aufgehen. Jetzt: u_tau_ist
	// aus der Kraftbilanz des Laufendes, beide Werte im Kopf ausgewiesen.
	const double utau_ist = sqrt((double)f_akt*(double)delta_lat);
	print_info("Kanal: u_tau IST = "+to_string((float)utau_ist,6u)+" (aus f_akt, Ein-Chunk-Versatz -- CSV-Kopf) gegen Ziel "+to_string(utau_lat,6u)
		+" (Faktor "+to_string((float)(utau_ist/(double)utau_lat),3u)+") -> Re_tau IST = "+to_string((float)(utau_ist*(double)delta_lat/(double)nu_lat),0u));
	pcsv << "# utau_ist=" << utau_ist << " utau_ziel=" << utau_lat << " nu_lat=" << nu_lat << "\n";
	if(kipp>0u) pcsv << "# ACHTUNG kipp>0: Ebenenmittel durch Nx*Ny geteilt (enthaelt Solid-Anteil ~3-7 % zu klein) und zw = VERTIKALER Abstand -- an gekippten Waenden nur als Verlaufsindikator brauchbar (Gross-Audit M).\n";
	pcsv << "z,yplus,Uplus,uu_plus,ww_plus,uw_plus,tau_gesamt_soll\n";
	const double ut2=utau_ist*utau_ist;
	for(uint z=1u; z<Nz-1u; z++) {
		const double m=su[z]/(double)n_stat, mz=suz[z]/(double)n_stat;
		const double zw=fmin((double)z-0.5,(double)(Nz-1u)-0.5-(double)z);
		const double dxp_ist = utau_ist/(double)nu_lat; // Wandeinheiten aus dem IST-u_tau
		pcsv << z << "," << zw*dxp_ist << "," << m/utau_ist << ","
		     << (suu[z]/(double)n_stat-m*m)/ut2 << "," << (sww[z]/(double)n_stat-mz*mz)/ut2 << ","
		     << (suw[z]/(double)n_stat-m*mz)/ut2 << "," << 1.0-zw/(double)delta_lat << "\n";
	}
	pcsv.close();
	{ ulong h=0ull; berichte_dichteklemme(lbm, "Kanal", h); dichteklemme_fazit(h); }
	if(env_u("CFD_WANDFUNKTION", 0u)>0u) { // Wirkpfad-Nachweis: Zaehler auslesen
		lbm.lbm_domain[0]->rho_clamp_hits.read_from_device();
		const ulong wz=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[2], kl=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[3], sk=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[4], sp=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[5];
		const ulong soll=(ulong)Nx*(ulong)Ny*2ull*(ulong)((n_steps+99ull)/100ull);
		// Audit-Nacharbeit 11: Slot 3 = NUR tau-Klemme, Slot 4 = u_t~0-Skips (vorher vermischt)
		print_info("Wandfunktion-Wirkpfad: "+to_string(wz)+" gezaehlte Wandzellen-Updates (Soll "+to_string(soll)+"), tau-Klemme "+to_string(kl)+", u_t~0-Skips "+to_string(sk)+", Ein-Zellen-Spalte "+to_string(sp));
		if(wz==0ull) print_error("Wandfunktion war eingeschaltet, aber der Wirkpfad-Zaehler ist NULL -- lautloser No-Op.");
	}
	if(env_u("CFD_FACETTEN", 0u)>0u) { // ★ Stufe 2: Wirkpfad-Nachweis, Soll EXAKT (F7)
		lbm.lbm_domain[0]->rho_clamp_hits.read_from_device();
		const ulong wz=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[7], kl=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[8];
		const ulong sk=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[9], zu=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[11];
		const ulong soll=lbm.lbm_domain[0]->fac_N*(ulong)((n_steps+99ull)/100ull);
		const ulong s12=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[12], s13=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[13], s10=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[10];
		print_info("Facetten-Wirkpfad: "+to_string(wz)+" (Soll "+to_string(soll)+"; Ereignis-Slots t%100-gesampelt seit 405be0f), tau-Klemme "+to_string(kl)
			+", u_t~0-Skips "+to_string(sk)+", ohne offenes Paar "+to_string(zu)
			+(env_u("CFD_FACETTEN",0u)>=3u?(", iMEM: u_s-Klemme/Gate "+to_string(s10)+", Skalar-Fallback "+to_string(s12)+", ELIBB "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[67])+", MLS[68] "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[68])+", Rueckfall-Buchung[69] "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[69])+", Quergate "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[64])+", LSQ-Rueckfall "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[65])+", ohne Tangential-Link "+to_string(s13)
			+", 3x3: Rang2 "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[14])+", Rang0-BB "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[15])+", sn-Klemme/Gate "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[16])+", PEMA-utb "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[17])+", alpha>ut "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[18])+", APG-Klemme "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[19])):string("")));
		{ const uint* H=lbm.lbm_domain[0]->rho_clamp_hits.data(); pruefe_rueckfall_buchung(H[69],H[10],H[13],H[15],H[16],H[64],LBM_Domain::s_fac_satgate,"Kanal"); pruefe_kraftpfad(H[70],H[69],H[7],H[9],H[17],H[71],LBM_Domain::s_fac_kraft,"Kanal"); pruefe_kaskade(H,"Kanal",LBM_Domain::s_fac_messnur>0u,LBM_Domain::s_fac_pinv>0u); bericht_zielerfuellung(H,(ulong)H[7],(ulong)H[9],(ulong)H[17],(ulong)H[69],"Kanal"); }
		bericht_klassen(lbm.lbm_domain[0], FF, out_dir, (double)utau_lat*(double)utau_lat, "Kanal");
		bericht_gdiag(lbm.lbm_domain[0], FF, out_dir, "Kanal");
		// ★ 03.09. NACHBAR-Wirkpfad (Slots 72/73/74) -- bis heute NIRGENDS im Host ausgelesen (Iron Rule: Schalter ohne feuernden Zaehler = harter Fehler).
		if(env_u("CFD_FAC_NACHBAR",0u)>0u&&LBM_Domain::s_fac_messnur==0u) { const ulong n72=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[72], n73=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[73], n74=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[74]; const ulong ns=n72+n73+n74;
			if(n72==0ull) print_error("[Kanal] CFD_FAC_NACHBAR war angefordert, aber Slot 72 = 0 -- stiller No-Op (Emission? kein Fluidnachbar: "+to_string(n73)+", Nachbar still: "+to_string(n74)+").");
			else { if(n72>=0xF0000000ull||n73>=0xF0000000ull||n74>=0xF0000000ull) print_error("[Kanal] NACHBAR-Zaehler GESAETTIGT (72="+to_string(n72)+", 73="+to_string(n73)+", 74="+to_string(n74)+") -- die Prozente sind NICHT belastbar. Lauf kuerzen oder Stichprobe ausduennen."); // ★ 03.09.: 4 mm laeuft auf 1,57e9 = 37 % des uint-Bereichs
				else if(n72+n73+n74>3865470566ull) print_warning("[Kanal] NACHBAR-Zaehler ueber 90 % des uint-Bereichs ("+to_string(n72+n73+n74)+") -- WICKELGEFAHR beim naechsten laengeren Lauf.");
				print_info("[Kanal] NACHBAR-Wirkpfad: Slot 72 angewandt = "+to_string(n72)+" ("+to_string((float)(ns>0ull?100.0*(double)n72/(double)ns:0.0),1u)+" % der Nachbar-Entscheide), 73 kein Fluidnachbar = "+to_string(n73)+", 74 Nachbar still = "+to_string(n74)+" (t%100-Stichprobe; UTKORR wirkt nur an 73/74)."); } }
		if(env_u("CFD_SGS_FDWAND",0u)>0u) { const ulong s39=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[76]; if(s39==0ull) print_error("[Kanal] SGS_FDWAND war angefordert, aber Slot 76 = 0 -- stiller No-Op (Emission? Rebind? Enqueue?)."); else print_info("[Kanal] SGS_FDWAND-Wirkpfad Slot 76 = "+to_string(s39)+" (Soll ~ fac_N * ceil(n/100) abzueglich WANDFREI-Zellen)."); }
		// ★ 03.09. F-MARKERLISTE: Slot 77 zaehlt jedes store3_F IN der F-BBox, das keinen Slot fand.
		// Das ist der Ersatz fuer den F-Waechter, der unter der Liste gegenstandslos wird: waere die
		// Host-Maske keine echte Obermenge des Kernel-Praedikats, ginge dort still Kraft verloren.
		if(lbm.lbm_domain[0]->f_liste_on) { const ulong s77=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[77];
			if(s77>0ull) print_error("[Kanal] F-MARKERLISTE: Slot 77 = "+to_string(s77)+" Schreibversuche OHNE Slot. Die Host-Maske ist KEINE Obermenge des Kernel-Praedikats -- an diesen Zellen geht Kraft still verloren. Ergebnis nicht verwenden.");
			else print_info("[Kanal] F-MARKERLISTE: Slot 77 = 0 -- jede vom Kernel beschriebene Zelle hatte einen Slot ("+to_string(lbm.lbm_domain[0]->f_slots)+" Slots).");
		}
		if(env_u("CFD_FACETTEN",0u)>=3u) { // ★ 2026-08-25 Stoerform-Groessenmessung (49..58)
			const uint* h = &lbm.lbm_domain[0]->rho_clamp_hits[0];
			ulong so=0ull, sp=0ull; for(uint k=0u; k<5u; k++) { so+=(ulong)h[49u+k]; sp+=(ulong)h[54u+k]; }
			if(so>0ull) {
				string zo="", zp="";
				const string lab[5] = {"<0.01","0.01-0.1","0.1-1","1-10",">=10"};
				for(uint k=0u; k<5u; k++) { zo+=lab[k]+":"+to_string(100.0*(double)h[49u+k]/(double)so,1u)+"% "; zp+=lab[k]+":"+to_string(100.0*(double)h[54u+k]/(double)sp,1u)+"% "; }
				print_info("  Stoerform-Offset |2*(S1.t)| / |Ziel def_fac_tau*twe|: "+zo);
				print_info("  P-Betrag         |P| / |Ziel|                       : "+zp);
			}
		}
		if(env_u("CFD_FACETTEN",0u)>=3u) { // Delta-m + Normalkontamination global (Auflage 2: Kanal-Soll exakt 0)
			lbm.lbm_domain[0]->fac_tau.read_from_device(); // ★ IR3-Audit MITTEL: vorher las die Schleife die VERALTETE Warmup-Kopie -- alle bisherigen N1-Zahlen waren falsch gefenstert!
			double dm=0.0, nk=0.0;
			for(ulong i=0ull;i<lbm.lbm_domain[0]->fac_N;i++){ dm+=(double)lbm.lbm_domain[0]->fac_tau[6ull*i+4ull]; nk+=(double)lbm.lbm_domain[0]->fac_tau[6ull*i+5ull]; }
			print_info("iMEM-Erhaltung: Delta-m gesamt = "+to_string((float)dm,9u)+" (Kanal-Soll exakt 0), Normalkontamination = "+to_string((float)nk,9u));
			if(kipp==0u&&dm!=0.0) print_warning("Delta-m am parallelen Kanal nicht exakt 0 -- S1-Komponentenpfad pruefen.");
		}
		if(LBM_Domain::s_fac_elibb_pur) { // ★ Pruefbefund Messlogik-3 (2026-08-25): der Pur-Return sitzt VOR Slot 7 -- das alte Soll kannte den Pur-Modus nicht und print_error (=exit) toetete den projizierten Cd-Pfad. Pur-Soll: Slot 7 = 0, ELIBB-Wirkpfad (Slot 67) > 0.
			if(wz!=0ull) print_error("Pur-Arm: Slot 7 muesste 0 sein -- Return-Position verschoben?");
			else if((ulong)lbm.lbm_domain[0]->rho_clamp_hits[67]==0ull) print_error("Pur-Arm: ELIBB-Wirkpfad (Slot 67) ist NULL -- lautloser No-Op.");
			else print_info("Pur-Arm (Kanal): Slot 7 = 0 (konstruktiv), ELIBB-Wirkpfad "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[67])+" -- Abnahme in Pur-Form bestanden.");
		} else if(wz!=(soll&0xFFFFFFFFull)) print_error("Facetten-Wirkpfad Ist != Soll -- Lookup oder Bindung defekt."); // Audit 1/3: Soll mod 2^32 -- der uint-Zaehler wickelt am Fahrzeugmassstab (fac_N~1e6 x 5000 Gates)
		if(kipp==0u&&env_u("CFD_FACETTEN",0u)<3u&&zu!=0ull) print_warning("Am parallelen Kanal muessen ALLE Paare offen sein -- "+to_string(zu)+" Zellen ohne Tausch.");
		lbm.lbm_domain[0]->fac_tau.read_from_device(); lbm.lbm_domain[0]->fac_tau_n.read_from_device();
		// ★ 2026-08-25 FIX: y_w war hier HARTKODIERT 0,5. Der dd-Pfad warnt ausdruecklich davor
		// ("NICHT hartkodiert 0,5 -- Audit-Rest #6 nicht wiederholen"), der Kanal wiederholte ihn.
		// An gekippten Waenden ist y_w im Median 0,698 (26 Grad) bzw. 1,040 (45 Grad) -- die
		// gemeldeten y+ waren also um Faktor 1,4 bzw. 2,1 zu klein. Jetzt je Facette mit ihrem
		// EIGENEN y_w aus fac_geo[8i+3], danach gemittelt.
		lbm.lbm_domain[0]->fac_geo.read_from_device();
		// ★★ 2026-08-25 Pruefbefund C(c), HOCH: dieses Mittel war ein VOLLLAUF-Mittel, waehrend
		// "Reibung x" und "Soll" strikt gefenstert sind (60 von 80 ETT). Der Fensterbruch betrug
		// rund 1 Prozentpunkt -- das Zwei- bis Dreifache des gesamten Anwendungspfad-Signals. Damit
		// war meine Zerlegung "der Ueberschuss sitzt im Modell, nicht in der Anwendung" NICHT
		// gedeckt. Jetzt wird tw ueber DASSELBE Fenster gemittelt wie die Kraft: Differenz des
		// Akkumulators geteilt durch die Differenz der Besuchszahl.
		const bool gef = !fac_snap_tw.empty()&&fac_snap_tw.size()==lbm.lbm_domain[0]->fac_N;
		double stau=0.0, syp=0.0; ulong ntau=0ull;
		for(ulong i=0ull; i<lbm.lbm_domain[0]->fac_N; i++) {
			const double acc=(double)lbm.lbm_domain[0]->fac_tau[6ull*i], nb=(double)lbm.lbm_domain[0]->fac_tau_n[i];
			const double a0=gef?fac_snap_tw[i]:0.0, n0=gef?(double)fac_snap_n[i]:0.0;
			const double dn=nb-n0; if(dn<=0.0) continue;
			const double tw_i=(acc-a0)/dn;
			stau+=tw_i; ntau++; syp+=sqrt(fmax(0.0,tw_i))*(double)lbm.lbm_domain[0]->fac_geo[8ull*i+3ull]/(double)nu_lat;
		}
		if(ntau>0ull) { const double mtau=stau/(double)ntau, yp=syp/(double)ntau;
			// ★ Pruefbefund C(b): mit 8 Dezimalen (to_string deckelt dort) blieben bei 4,4e-6 genau DREI
			// signifikante Stellen, also +-0,11 % -- zu grob fuer die 0,1-bis-0,5-%-Frage, um die es geht.
			// Deshalb zusaetzlich in 1e-6-Einheiten mit vier Nachkommastellen.
			print_info("Facetten-tau-Akkumulator: "+to_string(ntau)+" Zellen, mittleres tau_w = "+to_string((float)(mtau*1e6),4u)+"e-6"+(gef?" (GEFENSTERT wie die Kraft)":" (VOLLLAUF -- kein Schnappschuss vorhanden)")+", y+ = "+to_string((float)yp,2u)); }
		// ★ 03.09. DOPPELBUCHUNGS-DETEKTOR (Planungsagent; Heiko: "Doppel-/Falschzaehlung ausschliessen"). Im periodischen Kanal ist die
		// Volumenkraft die einzige Quelle und die Wand die einzige Senke. Traegt das Wandmodell die Senke GENAU EINMAL (Zielsetzung
		// R1 = -tau_Ziel - P1, kernel.cpp), ist <tau_w,Modell>/u_tau_ist^2 ~ 1 (wk_kipp0_cpu 02.09.: 1,024; xa_kd_kipp0_igpu 03.09.).
		// Eine ADDITIVE Buchung (Wandmodell UND BB-Austausch an derselben Flaeche) verlangte q = 1/(1+|P1|/twe) (vgl. KDIAG-Spalte
		// |P1|/twe). nu_t ist KEINE Senke: unter Zielsetzung ist der Wandaustausch auf tw gepinnt, unabhaengig von nu_t -- der Quotient
		// beweist die Zielsetzung, und GENAU DIE macht eine SGS-Doppelbuchung an der Wandflaeche unmoeglich (Planungsagent 03.09.).
		// Pruefagent 03.09.: Nenner = Fenstermittel der Antriebskraft (wie K2), nicht das frisch geregelte f_akt; scharf nur bei
		// kipp 0 UND Rueckfall-Slot 69 == 0 -- an Rueckfallzellen traegt BB, dann ist das Modellmittel allein kein Senkenmass.
		{ const double fq_=fac_fn>0.0?fac_fsum/fac_fn:(double)f_akt;
		  const bool scharf=(kipp==0u&&(ulong)lbm.lbm_domain[0]->rho_clamp_hits[69]==0ull);
		  if(ntau>0ull&&fq_>0.0) { const double q=(stau/(double)ntau)/(fq_*(double)delta_lat);
			const string urteil = !scharf ? "unscharf (kipp>0 oder Rueckfallzellen: BB-Anteil fehlt im Modellmittel -- kein Doppelbuchungsmass)" : ((q>0.95&&q<1.05) ? "Wand traegt die Senke genau einmal (Zielsetzung)" : (q<0.6 ? "ADDITIV-VERDACHT (Modell traegt nur einen Teil der Senke)" : "ABWEICHUNG -- Buchung pruefen")); // scharf ZUERST (xb_nb_kipp26: q=0,195 bei 51 % Rueckfall hiess sonst "ADDITIV-VERDACHT")
			print_info("[Kanal] DOPPELBUCHUNGS-DETEKTOR: <tau_w,Modell>/(f_Fenster*delta) = "+to_string((float)q,4u)+" -> "+urteil+(scharf?" [kipp 0, Rueckfall 0: scharf]":" [kipp>0 oder Rueckfall>0: nur Verlaufsindikator]"));
		} }
		// ★ Cd-Pfad-Validierung K2/K3 (FACETTEN-CD-PFAD.md)
		if(!fac_snap.empty()&&n_steps>fac_snap_step) {
			const FacKraft FK = kraft_facetten(lbm, Nx, Ny, Nz, TYPE_S, n_steps-fac_snap_step, fac_snap, kipp>0u);
			const double fq = fac_fn>0.0 ? fac_fsum/fac_fn : 0.0;
			const double soll_rx = (kipp==0u) ? fq*(double)delta_lat*2.0*(double)Nx*(double)Ny // f*delta je Saeule, beide Waende (alt wortgleich)
			                                  : fq*(double)Nx*(double)Ny*(double)(Nz-Tv);      // F5: f*V_fluid (Torus)
			print_info("Cd-Pfad Kanal: Reibung x = "+to_string((float)FK.rx,9u)+" (Soll f*delta*Flaeche = "+to_string((float)soll_rx,9u)
				+", Verhaeltnis "+to_string((float)(soll_rx!=0.0?FK.rx/soll_rx:0.0),4u)+"), Reibung y = "+to_string((float)FK.ry,9u));
			print_info("Cd-Pfad Kanal: Druck x = "+to_string((float)FK.px,9u)+" (K3-Soll exakt 0), n_voll "+to_string(FK.n_voll)
				+", projiziert "+to_string(FK.n_proj)+", unklar "+to_string(FK.n_unklar));
			// K2 ist ein STATIONARITAETS-Kriterium -- im Transientenfenster (<5000 Schritte) wird es
			// angesagt uebersprungen statt einen legitimen Kurztest zu killen (R3-Nachschliff).
			if(n_steps-fac_snap_step<5000ull) print_warning("K2-Pruefung UEBERSPRUNGEN: Fenster "+to_string(n_steps-fac_snap_step)+" Schritte ist transient (hart erst ab 5000) -- dieser Lauf ist KEIN Abnahmelauf.");
			else if(LBM_Domain::s_fac_messnur>0u) print_info("K2 im MESS-NUR-Arm uebersprungen (das Wandmodell wendet nichts an, der Reibungspfad ist konstruktiv leer -- 31.08./02.09., gleiche Logik wie der Pur-Guard bei K3).");
			else if(soll_rx!=0.0&&fabs(FK.rx/soll_rx-1.0)>0.01) print_error("K2 verletzt: Reibungspfad weicht >1 % von der Kraftbilanz ab -- Abnahmelauf disqualifiziert.");
			if(LBM_Domain::s_fac_elibb_pur) print_info("K3 im Pur-Arm uebersprungen (fac_tau_n bleibt konstruktiv 0 -> n_voll-Kriterium gilt nicht; B3-Pruefbefund 3)."); // Pur-Guard wie beim Slot-7-Fix
			else { // ★ Instrumentenfix 30.08. (Freigabe G): exakte double-Gleichheit als Nullkriterium ist falsch -- x26_ref druckte "-0.00000000",
				// FK.px war ~1e-12 != 0.0 und der Arm endete mit rc=1 "K3 verletzt" (der alte j4q_t26a4a1 hatte zufaellig exakt -0.0). Toleranz relativ zur Reibung.
				const double tol_px = 1e-6*fabs(FK.rx)+1e-12;
				if(fabs(FK.px)>tol_px||FK.n_unklar!=0ull||FK.n_voll!=0ull) print_error("K3 verletzt: Druck_x = "+to_string((float)FK.px)+" (|.| > "+to_string((float)tol_px)+") oder unerwartete Voll-/Unklar-Zellen am parallelen Kanal.");
				else if(FK.px!=0.0) print_info("K3: Druck_x = "+to_string((float)FK.px)+" innerhalb Toleranz "+to_string((float)tol_px)+" (nicht exakt 0, Rundungsrest der double-Summe)."); }
		}
	}
	// Feld-Hash (FNV-1a ueber die u-Bitmuster) fuer den Bitvergleich der Aequivalenzarme
	if(env_u("CFD_FELD_HASH", 0u)>0u) {
		lbm.u.read_from_device();
		ulong h=1469598103934665603ull;
		for(ulong i=0ull; i<3ull*lbm.get_N(); i++) { uint b; const float v=(i<lbm.get_N())?lbm.u.x[i%lbm.get_N()]:((i<2ull*lbm.get_N())?lbm.u.y[i%lbm.get_N()]:lbm.u.z[i%lbm.get_N()]); memcpy(&b,&v,4u); h^=(ulong)b; h*=1099511628211ull; }
		print_info("FELD-HASH(u) = "+to_string(h));
	}
	print_info("Kanal fertig: kanal_zeit.csv (U_b+, c_f beide Wege) und kanal_profil.csv (U+, Spannungen).");
	print_info("Referenz Lee & Moser 5186: U_b+ = 24,104, c_f = 3,4424e-3.");
	_exit(0);
}

// ---------------------------------------------------------------------------- y+ messen statt korrelieren
// ★★ 0A der Arbeitsliste, 2026-08-15. Die Zahl "y+ = 137" war eine PLATTENKORRELATION bei x = L --
// nie gemessen. Auf ihr stand die ganze Wandmodell- und Kanalplanung. Diese Funktion misst sie.
//
// WAS SIE TUT: update_force_field liefert die Impulsaustauschkraft je Solid-Zelle (F, SoA, in der
// F-Box). Fuer jede FAHRZEUG-Zelle (flags == TYPE_S|TYPE_X, genau die zaehlt object_force) mit
// GENAU EINEM Fluid-Flaechennachbarn ist die Wandnormale exakt bekannt -- dort wird die
// Tangentialkraft (die beiden Komponenten senkrecht zur Normalen) als Wandschubspannung genommen

// (Zellflaeche = 1 in Gittereinheiten), daraus u_tau = sqrt(tau_w) (rho_lat = 1) und
// y+ = u_tau * 0,5 / nu_lat (erste Fluidzellmitte liegt eine halbe Zelle von der Wand).
//
// WARUM NUR EINDEUTIG ORIENTIERTE ZELLEN: an Treppen und Kanten mischt der Impulsaustausch Druck-
// und Reibungsanteil untrennbar (Gegenpruefung 2026-08-09: Vorfaktor schwankt um Faktor 3,5, an
// Kanten kommt wandnormale Transpiration dazu). Die anliegenden Flaechen -- Dach, Haube, Flanken,
// Unterboden -- sind genau die, um die es bei y+ geht. Der Anteil vermessener Zellen wird
// ausgegeben; ist er klein, ist das ein Befund ueber die Voxelisierung, nicht ueber y+.
//
// ★ U1-FALLE (HYGIENE-BEFUNDE): lbm.F der Huelle rechnet mit der VOLLEN Domaenengroesse, der
// Puffer ist BBox-gross. Hier wird deshalb direkt der Domaenenpuffer mit der BBox-Indizierung aus
// f_bbox() gelesen: fbi = (x-fbx0) + (y-fby0)*fbnx + (z-fbz0)*fbnx*fbny, SoA-Stride F_N.
// ★ 03.09.2026 F-LISTEN-ZENSUS (CFD_F_LISTE_ZENSUS=1; Default AUS = kein Code, kein Zeitverbrauch,
// bitgleich). Beantwortet die EINE offene Vorfrage des Speicherpostens B78(b) "F auf Solid-Markerliste":
// wie lang waere die Liste ueberhaupt? update_force_field (kernel.cpp) schreibt F ausschliesslich an
// Zellen mit (flags&TYPE_BO)==TYPE_S UND mindestens einem Nicht-Solid unter den 18 D3Q19-Nachbarn
// (has_fluid_neighbor). Genau dieses Praedikat wird hier gezaehlt -- nicht das 6-Richtungs-Kriterium
// aus messe_yplus, das eine echte Teilmenge davon ist.
// EINSCHRAENKUNG, ausdruecklich: der Host wickelt x/y periodisch und ueberspringt z ausserhalb des
// Gitters, waehrend der Kernel neighbors() in ALLEN Richtungen wickelt. An den z-Raendern weicht das
// Praedikat deshalb um wenige Zellen ab. Fuer die GROESSENORDNUNGS-Entscheidung (Bauform A gegen B)
// genuegt das; fuer einen spaeteren Listenbau NICHT -- der gehoert auf das Geraet, mit genau der
// has_fluid_neighbor-Zeile des Kernels, damit es das Praedikat nur EINMAL gibt.
void f_listen_zensus(LBM& L, const uint Nx, const uint Ny, const uint Nz, const char* wo) {
	LBM_Domain* D = L.lbm_domain[0];
	L.flags.read_from_device();
	const ulong FN = (ulong)D->fbnx*(ulong)D->fbny*(ulong)D->fbnz;
	ulong n_solid=0ull, n_wand=0ull;
	for(uint z=(uint)D->fbz0; z<D->fbz0+D->fbnz; z++) for(uint y=(uint)D->fby0; y<D->fby0+D->fbny; y++) for(uint x=(uint)D->fbx0; x<D->fbx0+D->fbnx; x++) {
		const ulong n = (ulong)x + ((ulong)y + (ulong)z*(ulong)Ny)*(ulong)Nx;
		if((L.flags[n]&(TYPE_S|TYPE_E))!=TYPE_S) continue; // wortgleich zum Kernel-Gate (TYPE_BO = TYPE_S|TYPE_E = 0x03; TYPE_MS traegt das S-Bit)
		n_solid++;
		for(uint i=1u; i<19u; i++) {
			const int zn=(int)z+FZ_C[i][2]; if(zn<0||zn>=(int)Nz) continue;
			const uint xn=(uint)((((int)x+FZ_C[i][0])%(int)Nx+(int)Nx)%(int)Nx);
			const uint yn=(uint)((((int)y+FZ_C[i][1])%(int)Ny+(int)Ny)%(int)Ny);
			const ulong nn = (ulong)xn + ((ulong)yn + (ulong)zn*(ulong)Ny)*(ulong)Nx;
			if((L.flags[nn]&(TYPE_S|TYPE_E))!=TYPE_S) { n_wand++; break; }
		}
	}
	const ulong b_heute = 12ull*FN;                       // F als volles BBox-Feld, 3 float je Zelle
	const ulong b_formA = 12ull*n_solid + 4ull*n_solid;   // Liste ueber ALLE Solidzellen + uint-Index je Zelle
	const ulong b_formB = 12ull*n_wand  + 8ull*((FN+31ull)/32ull); // Wandliste + Bitmaske/Praefixsumme (dieselbe Maschinerie wie fac_idx)
	print_info(string("--- F-LISTEN-ZENSUS, ")+wo+" (CFD_F_LISTE_ZENSUS) ---");
	print_info("  F-BBox "+to_string((ulong)D->fbnx)+"x"+to_string((ulong)D->fbny)+"x"+to_string((ulong)D->fbnz)+" = "+to_string(FN)+" Zellen | F heute "+to_string((ulong)(b_heute/1048576ull))+" MiB");
	print_info("  Solidzellen in der BBox: "+to_string(n_solid)+" | davon WANDsolid (>=1 Nicht-Solid unter 18 Links): "+to_string(n_wand)+" ("+to_string((float)(100.0*(double)n_wand/(double)max(1ull,n_solid)),2u)+" % der Solidzellen, "+to_string((float)(100.0*(double)n_wand/(double)FN),3u)+" % der BBox)");
	print_info("  Bauform A (alle Solidzellen + Index): "+to_string((ulong)(b_formA/1048576ull))+" MiB -> Gewinn "+to_string((ulong)((b_heute>b_formA?b_heute-b_formA:0ull)/1048576ull))+" MiB");
	print_info("  Bauform B (nur Wandsolid + Bitmaske): "+to_string((ulong)(b_formB/1048576ull))+" MiB -> Gewinn "+to_string((ulong)((b_heute>b_formB?b_heute-b_formB:0ull)/1048576ull))+" MiB");
	print_info("  ENTSCHEIDUNGSREGEL (03.09.): traegt Bauform B unter 1,2 GB, lohnt der Umbau den Eingriff nicht.");
}
void messe_yplus(LBM& L, const uint Nx, const uint Ny, const uint Nz, const float nu_lat, const float dx, const float dt, const float si_rho, const string& out_dir, const char* wo) {
	L.update_force_field();
	LBM_Domain* D = L.lbm_domain[0];
	D->F.read_from_device(); L.flags.read_from_device();
	const ulong FN = (ulong)D->fbnx*(ulong)D->fbny*(ulong)D->fbnz;
	const uchar VEH = (uchar)(TYPE_S|TYPE_X);
	const int dirs[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
	std::vector<float> yp; yp.reserve(1u<<20);
	ulong n_veh_wand=0ull, n_eindeutig=0ull;
	double tau_sum=0.0;
	for(uint z=(uint)D->fbz0; z<D->fbz0+D->fbnz; z++) for(uint y=(uint)D->fby0; y<D->fby0+D->fbny; y++) for(uint x=(uint)D->fbx0; x<D->fbx0+D->fbnx; x++) {
		const ulong n = (ulong)x + ((ulong)y + (ulong)z*(ulong)Ny)*(ulong)Nx;
		if(L.flags[n]!=VEH) continue;
		int nrm=-1; uint n_fluid=0u;
		for(int d=0; d<6; d++) {
			const int xn=(int)x+dirs[d][0], yn=(int)y+dirs[d][1], zn=(int)z+dirs[d][2];
			if(xn<0||yn<0||zn<0||xn>=(int)Nx||yn>=(int)Ny||zn>=(int)Nz) continue;
			const ulong nn = (ulong)xn + ((ulong)yn + (ulong)zn*(ulong)Ny)*(ulong)Nx;
			// Fluid = nicht Solid im Sinne der Maske (TYPE_MS = 0x03 traegt das S-Bit -- die bekannte Falle)
			if((L.flags[nn]&(TYPE_S|TYPE_E))!=TYPE_S) { n_fluid++; nrm=d; }
		}
		if(n_fluid==0u) continue;
		n_veh_wand++;
		if(n_fluid!=1u) continue; // nur eindeutig orientierte Zellen, siehe Kopfkommentar
		n_eindeutig++;
		const ulong fbi = (ulong)(x-D->fbx0) + (ulong)(y-D->fby0)*(ulong)D->fbnx + (ulong)(z-D->fbz0)*(ulong)D->fbnx*(ulong)D->fbny;
		ulong fs_; if(!D->f_slot_host(fbi, fs_)) continue; // ★ 03.09. F-Markerliste: Zelle ohne Slot traegt konstruktiv F=0
		const ulong FST=D->f_stride();
		const float Fx=D->F[fs_], Fy=D->F[FST+fs_], Fz=D->F[2ull*FST+fs_];
		const int ax = dirs[nrm][0]!=0 ? 0 : (dirs[nrm][1]!=0 ? 1 : 2);
		const float Ft = ax==0 ? sqrt(Fy*Fy+Fz*Fz) : (ax==1 ? sqrt(Fx*Fx+Fz*Fz) : sqrt(Fx*Fx+Fy*Fy));
		const float tau_lat = Ft; // Flaeche = 1 Gitterzelle
		tau_sum += (double)tau_lat;
		yp.push_back(sqrt(tau_lat)*0.5f/nu_lat);
	}
	if(yp.empty()) { print_warning(string(wo)+": keine eindeutig orientierte Fahrzeug-Wandzelle gefunden -- y+ nicht messbar."); return; }
	std::sort(yp.begin(), yp.end());
	auto q = [&](const double f) { return yp[(size_t)fmin((double)yp.size()-1.0, f*(double)yp.size())]; };
	const float stress_si = si_rho*(dx/dt)*(dx/dt); // Pa je Gitter-Spannungseinheit
	print_info(string("--- y+ GEMESSEN, ")+wo+" (nur eindeutig orientierte Wandzellen) ---");
	print_info("  Fahrzeug-Wandzellen: "+to_string(n_veh_wand)+", davon eindeutig orientiert: "+to_string(n_eindeutig)
		+" ("+to_string(100.0f*(float)n_eindeutig/(float)fmax(1ull,n_veh_wand),1u)+" %)");
	print_info("  y+ Quartile: 25 % = "+to_string(q(0.25),1u)+" | MEDIAN = "+to_string(q(0.5),1u)+" | 75 % = "+to_string(q(0.75),1u)
		+" | 95 % = "+to_string(q(0.95),1u));
	print_info("  tau_w Mittel = "+to_string((float)(tau_sum/(double)yp.size())*stress_si,3u)+" Pa (Plattenkorrelation erwartete ~1,3 Pa; V1 mass am Unterboden Median 0,28 Pa)");
	print_info("  zum Vergleich: die Korrelation sagte y+ = 137 -- der MEDIAN oben ist die gemessene Wahrheit");
	// Histogramm in die CSV, damit die Verteilung nachnutzbar ist
	std::ofstream f(out_dir+"yplus_histogramm.csv");
	f << "yplus_bin_bis,anzahl\n";
	const float bins[12] = {1.0f,5.0f,10.0f,30.0f,60.0f,100.0f,150.0f,200.0f,300.0f,400.0f,600.0f,1e9f};
	size_t i0=0;
	for(const float b : bins) { size_t i1=i0; while(i1<yp.size() && yp[i1]<b) i1++; f << b << "," << (i1-i0) << "\n"; i0=i1; }
	f.close();
	print_info("  Histogramm: "+out_dir+"yplus_histogramm.csv");
}

// ---------------------------------------------------------------------------- Wandprofil ueber die ZEIT
// ★★ B2 der Arbeitsliste, 2026-08-15. Der Wandwirksamkeits-Nachweis lief nur bei t = 0 und meldete
// dort brav 1,001 -- der tote Bodenstreifen waechst aber erst ueber ~180 ms heran (Schnitt-Befund
// von Heiko). Ein Nachweis, der nur den Anfangszustand prueft, ist fuer diese Fehlerklasse BLIND.
// Deshalb: je Sample das u_x-Profil ueber der Fahrbahn an einer festen freien Saeule in die CSV.
// Kosten: gelesen wird NUR die x-Komponente der untersten acht z-Ebenen -- die liegen im


// SoA-Layout kontiguierlich am Pufferanfang (33 MB je Sample im Nahfeld statt 6 GB Vollread).
bool finde_messsaeule(LBM& L, const uint Nx, const uint Ny, const uint Nz, uint& xs, uint& ys) {
	(void)Nz; L.flags.read_from_device(); ys = Ny/2u;
	for(uint k=1u; k<=8u; k++) {
		const uint xt = max(4u, (Nx*k)/40u); bool ok=true;
		for(uint z=1u; z<8u && ok; z++) {
			const ulong n = (ulong)xt + ((ulong)ys + (ulong)z*(ulong)Ny)*(ulong)Nx;
			if((L.flags[n]&(TYPE_S|TYPE_E))==TYPE_S) ok=false; // Maske, nicht Bit -- TYPE_MS traegt das S-Bit
		}
		if(ok) { xs=xt; return true; }
	}
	return false;
}
void schreibe_wandprofil(LBM& L, const uint Nx, const uint Ny, const uint xs, const uint ys, const float u_lat, const double t_si, std::ofstream& f) {
	L.lbm_domain[0]->u.read_from_device_1d(0ull, (ulong)Nx*(ulong)Ny*8ull, 0); // nur u_x, z = 0..7
	f << t_si;
	for(uint z=1u; z<8u; z++) {
		const ulong n = (ulong)xs + ((ulong)ys + (ulong)z*(ulong)Ny)*(ulong)Nx;
		f << "," << L.lbm_domain[0]->u.x[n]/u_lat;
	}
	f << "\n" << std::flush; // waehrend des Laufs lesbar, und ein Absturz kostet keine Reihe
}

// ---------------------------------------------------------------------------- Mitbewegte Waende pruefen
// ★★ WARUM DIESE FUNKTION MEHR TUT ALS FLAGS ZAEHLEN -- die Lehre aus V1, 2026-08-09 nachgeprueft.
//
// V1s Bodengeschwindigkeit war ein VOLLSTAENDIGES No-op (V1-Commit 2f705ba). Der Guard lautete dort
// `if((fn & TYPE_BO) != 0u) return;` -- und weil TYPE_MS == TYPE_BO == 0x03 ist, sprang die Funktion
// fuer genau die Zellen heraus, fuer die sie gedacht war. Die FLAGS waren dabei alle korrekt gesetzt;
// V1s eigener Commit nennt "Lage z=1 hat 101250 Zellen mit 0x03".
//
// Ein Audit, der nur Flags und Wand-u zaehlt, haette diesen Fehler also MIT VOLLER PUNKTZAHL
// bestanden. Er prueft die beiden EINGAENGE von apply_moving_boundaries und keinen einzigen AUSGANG.
// Deshalb kommen hier zwei Nachweise dazu:
//
//  (A) DER IMPULSTERM SELBST, host-seitig nachgerechnet. apply_moving_boundaries addiert je Link zu
//      einem soliden Nachbarn -6*w_i*(c_i . u_wand). Fuer eine Fluidzelle auf z=1 ueber einer in +x
//      laufenden Fahrbahn zeigen FUENF Links nach z-1: (0,0,-1), (+-1,0,-1), (0,+-1,-1) -- D3Q19 hat
//      keine Raumdiagonalen. Bei DREI davon ist c_i . u_wand exakt null (der senkrechte und die
//      beiden yz-Diagonalen). Der senkrechte Link uebertraegt also GAR NICHTS.
//      (Zahlen berichtigt nach Nachpruefung: hier stand "sechs Links, vier null".) Der gesamte
//      Uebertrag haengt an den beiden xz-Diagonalen (+-1,0,-1) mit w = 1/36:
//         2 * 6 * (1/36) * u_lat = u_lat/3  pro Schritt.
//      Eine Flag-Zaehlung sieht davon nichts. Gezaehlt wird deshalb, wie viele MS-Zellen einen
//      Beitrag von EXAKT NULL haben -- das faengt jede Aenderung an Geschwindigkeitssatz oder
//      Wandorientierung, die den Uebertrag geometrisch totlegt.
//
//  (B) DER NACHWEIS DURCH DEN KERNEL HINDURCH (pruefe_wandwirksamkeit, weiter unten). Nur der faengt
//      den V1-Fall. Er nutzt aus, dass u_wand = u_lat = u_inf ist: eine INTAKTE mitbewegte Wand
//      erzeugt damit GAR KEINE Grenzschicht, u_x muss ueber der Fahrbahn auf u_inf stehenbleiben.
//      Eine ruhende Wand baut sofort ein Defizit auf -- und zwar nicht diffusionsbegrenzt, sondern
//      mit u_lat/3 pro Schritt.
void audit_bewegte_waende(LBM& L, const uint Nx, const uint Ny, const uint Nz, const float dx, const float u_lat, const char* wo, const bool bodenkontakt_erwartet) {
	L.flags.read_from_device();
	// ★ ABGEWOGEN, nicht uebersehen: ausgewertet wird von u nur die Ebene z = 0, und die liegt wegen
	// des SoA-Layouts kontiguierlich am Pufferanfang -- ein Teil-Read spart im Fahrzeugfall 6,02 GB.
	// Den gibt es aber nur auf Memory, nicht auf dem Memory_Container, ueber den L.u laeuft. Fuer eine
	// EINMALIGE Diagnose (rund 0,6 s bei 10 GB/s) eine Kernklasse zu erweitern, die jeder Puffer
	// benutzt, ist das Risiko nicht wert. Bleibt bewusst der volle Lesevorgang.
	L.u.read_from_device();
	const uchar VEH = (uchar)(TYPE_S|TYPE_X);
	const uchar MS  = (uchar)(TYPE_S|TYPE_E); // TYPE_MS ist nur geraeteseitig 0x03
	ulong n_veh=0ull, n_veh_z0=0ull, n_contact=0ull, n_road_z0=0ull, n_road_moving=0ull;
	ulong n_road_still_fluid=0ull; // ruhende Wandzelle MIT Fluidnachbar darueber -- nur das ist ein Fehler
	ulong n_ms_z1=0ull, n_fluid_z1=0ull, n_ms_total=0ull, n_ms_ohne_impuls=0ull;
	float abw_max = 0.0f; // groesste relative Abweichung des Impulsterms vom Sollwert
	uint z_min_veh = Nz;
	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
		const ulong n = (ulong)x + ((ulong)y + (ulong)z*(ulong)Ny)*(ulong)Nx;
		const uchar fl = L.flags[n];
		if(fl==VEH) { n_veh++; if(z<z_min_veh) z_min_veh=z; if(z==0u) n_veh_z0++; }
		if((fl&(TYPE_S|TYPE_E))==MS) n_ms_total++;
		if(z==0u && (fl&(TYPE_S|TYPE_E))==TYPE_S) {
			n_road_z0++;
			if(L.u.x[n]!=0.0f) n_road_moving++;
			else {
				// ★ Eine stillstehende Wandzelle ist NUR dann ein Fehler, wenn ueber ihr ueberhaupt
				// Fluid steht. initialize() nullt u fuer Solidzellen mit ausschliesslich soliden
				// Nachbarn (kernel.cpp) -- unter dem Reifenlatsch ist das Bauart, kein Defekt.
				const ulong n1 = (ulong)x + ((ulong)y + (ulong)Ny)*(ulong)Nx; // z=1
				// ★★ ZUM ZWEITEN MAL DIESELBE FALLE, und diesmal im selben Commit, der sie feierte:
				// hier stand (flags & TYPE_S) == 0. Eine mitbewegte FLUIDZELLE traegt aber TYPE_MS =
				// 0x03, und 0x03 & TYPE_S ist nicht null -- sie galt also als "kein Fluid", und die
				// Warnung war im Inneren der Fahrbahn praktisch tot. Fluid ist alles, was NICHT
				// (flags & (TYPE_S|TYPE_E)) == TYPE_S ist.
				if((L.flags[n1]&(TYPE_S|TYPE_E))!=TYPE_S) n_road_still_fluid++;
			}
		}
		if(z==1u) {
			const bool ms = (fl&(TYPE_S|TYPE_E))==MS;
			if(ms) n_ms_z1++;
			if((fl&(TYPE_S|TYPE_E))==0u || ms) n_fluid_z1++;
			const ulong n0 = (ulong)x + (ulong)y*(ulong)Nx;
			if(fl==VEH && (L.flags[n0]&(TYPE_S|TYPE_E))==TYPE_S) n_contact++;
			// ★ (A) Impulsterm nachrechnen: nur die xz-Diagonalen zur Fahrbahn tragen.
			if(ms && (L.flags[n0]&(TYPE_S|TYPE_E))==TYPE_S) {
				float beitrag = 0.0f;
				for(int ddx=-1; ddx<=1; ddx++) { // Links (ddx,0,-1); w = 1/36 fuer ddx!=0, 1/18 fuer ddx==0
					const int xn = (int)x+ddx; if(xn<0 || xn>=(int)Nx) continue;
					const ulong nn = (ulong)xn + (ulong)y*(ulong)Nx; // z=0
					if((L.flags[nn]&(TYPE_S|TYPE_E))!=TYPE_S) continue;
					const float w_i = (ddx==0) ? (1.0f/18.0f) : (1.0f/36.0f);
					beitrag += fabs(6.0f*w_i*(float)ddx*L.u.x[nn]); // c_i . u = ddx*u_x
				}
				if(beitrag==0.0f) n_ms_ohne_impuls++;
				else { const float rel = fabs(beitrag-u_lat/3.0f)/(u_lat/3.0f); if(rel>abw_max) abw_max=rel; }
			}
		}
	}
	const float erwartet = u_lat/3.0f;
	print_info(string("--- Mitbewegte Waende, ")+wo+" ---");
	if(n_veh>0ull) {
		print_info("  Koerperzellen (flags == 0x41, genau die zaehlt object_force): "+to_string(n_veh)
			+"; unterste bei z = "+to_string(z_min_veh)+" ("+to_string((float)z_min_veh*dx*1000.0f,1u)+" mm ueber der Fahrbahn)");
		print_info("  Direkter Aufstand: "+to_string(n_contact)+" Koerperzellen auf z = 1 sitzen unmittelbar auf einer Fahrbahnzelle"
			+(bodenkontakt_erwartet ? "" : " (hier KEINER erwartet -- der Koerper schwebt bauartgemaess)"));
	}
	print_info("  Fahrbahn z = 0: "+to_string(n_road_z0)+" Wandzellen, davon "+to_string(n_road_moving)+" mit u_x != 0");
	print_info("  Mitbewegte Wand: "+to_string(n_ms_z1)+" von "+to_string(n_fluid_z1)+" Fluidzellen auf z = 1 tragen TYPE_MS;"
		+" im ganzen Gitter "+to_string(n_ms_total));
	print_info("  Impulsuebertrag je MS-Bodenzelle: Soll "+to_string(erwartet,5u)+" pro Schritt; groesste Abweichung "
		+to_string(100.0f*abw_max,2u)+" %; Zellen mit Beitrag EXAKT NULL: "+to_string(n_ms_ohne_impuls));
	// ★ Nachpruefer-Befund: vorher wurde der Sollwert nur HINGESCHRIEBEN und nichts verglichen -- eine
	// Fahrbahn mit halber oder doppelter Geschwindigkeit haette den Nachweis ohne Befund bestanden.
	// ★ Re-Audit R2 (Befund 16): der alte Text behauptete "Randzellen mit zwei Wandflaechen sind
	// ausgenommen" -- diese Ausnahme gab es im Code NIE. Ausgenommen sind nur Zellen mit Beitrag
	// exakt null; eine Zelle mit stillem zweiten Wandnachbarn (z. B. neben dem Reifenlatsch)
	// geht mit ihrem Teil-Beitrag in abw_max ein und KANN diese Warnung ausloesen.
	if(abw_max>0.02f) print_warning(string(wo)+": der Impulsterm weicht um bis zu "+to_string(100.0f*abw_max,2u)
		+" % vom Sollwert u_lat/3 ab. Achtung: Zellen mit zweitem (auch stillem) Wandnachbarn tragen Teil-Beitraege bei und koennen diese Warnung legitim ausloesen -- erst Einzelzellen pruefen, dann der Wandgeschwindigkeit misstrauen.");
	print_info("  Koerperzellen auf z = 0: "+to_string(n_veh_z0)+" (muss 0 sein -- sie wurden an die Strasse uebergeben)");
	print_info("  Fahrbahnzellen ohne Bewegung: "+to_string(n_road_z0-n_road_moving)+", davon MIT Fluid darueber: "+to_string(n_road_still_fluid)
		+" (nur die zweite Zahl ist ein Fehler -- unter dem Reifenlatsch ist Stillstand Bauart)");
	// ★ Nachpruefer-Befund: die Fahrbahn koennte auch GANZ fehlen. Dann ist n_road_z0 = 0, und ohne
	// diesen Fall haette KEINE einzige Warnung gefeuert -- ein Wirksamkeitsnachweis, der bei
	// fehlender Wand gruenes Licht gibt.
	if(n_road_z0==0ull) print_warning(string(wo)+": es gibt ueberhaupt KEINE Fahrbahnzelle auf z = 0.");
	if(n_veh_z0>0ull) print_warning(string(wo)+": es liegen noch Koerperzellen auf z = 0 -- die Uebergabe an die Strasse hat nicht gegriffen.");
	if(bodenkontakt_erwartet && n_contact==0ull) print_warning(string(wo)+": KEIN direkter Aufstand -- der Wagen schwebt.");
	if(n_road_still_fluid>0ull) print_warning(string(wo)+": "+to_string(n_road_still_fluid)+" Fahrbahnzellen MIT Fluid darueber stehen still.");
	if(n_road_z0>0ull && n_ms_z1==0ull) print_warning(string(wo)+": KEINE TYPE_MS-Zelle ueber der Fahrbahn -- der mitbewegte Boden ist wirkungslos.");
	if(n_ms_ohne_impuls>0ull) print_warning(string(wo)+": "+to_string(n_ms_ohne_impuls)+" MS-Bodenzellen bekommen einen Impulsterm von EXAKT NULL.");
}

// ★★ (B) DER NACHWEIS DURCH DEN KERNEL. Der einzige, der V1s Fehlerklasse faengt: dort waren alle
// Flags korrekt, nur der Konsument sprang heraus. Ausgenutzt wird, dass u_wand = u_lat = u_inf ist --
// eine intakte mitbewegte Wand erzeugt KEINE Grenzschicht. Gemessen wird eine z-Saeule weit vor dem
// Koerper; steht dort u_x auf u_inf, ueberträgt die Wand wirklich Impuls. Faellt es ab, ist die Wand
// effektiv eine ruhende Platte -- egal was die Flags sagen.
void pruefe_wandwirksamkeit(LBM& L, const uint Nx, const uint Ny, const uint Nz, const float u_lat, const char* wo) {
	(void)Nz;
	L.u.read_from_device(); L.flags.read_from_device();
	// ★ DIE MESSSAEULE MUSS FREI STEHEN. Erster Versuch nahm x = Nx/20 fest -- im Fahrzeugfall lief die
	// Saeule damit DURCH DEN WAGEN (Profil z5 = z6 = 0,000, weil das Solidzellen sind), und die
	// Warnung feuerte gegen die eigene Messstelle statt gegen die Wand. Gefunden im Regressionslauf
	// 2026-08-09. Jetzt werden mehrere Stellen stromauf durchprobiert und die erste genommen, deren
	// Saeule z = 1..7 vollstaendig Fluid ist; findet sich keine, wird das GESAGT statt gewarnt.
	uint xs=0u, ys=Ny/2u; bool frei=false;
	for(uint k=1u; k<=8u && !frei; k++) {
		const uint xt = max(4u, (Nx*k)/40u); // 2,5 %, 5 %, ... der Laenge -- alles weit stromauf
		bool ok=true;
		for(uint z=1u; z<8u && ok; z++) {
			const ulong n = (ulong)xt + ((ulong)ys + (ulong)z*(ulong)Ny)*(ulong)Nx;
			// ★★ MASKE, NICHT NUR DAS BIT. Nach flags.read_from_device() tragen wandnahe FLUIDZELLEN
			// TYPE_MS = 0x03, und darin steckt das TYPE_S-Bit. Ein Test auf (flags & TYPE_S) haelt
			// also jede mitbewegte Fluidzelle fuer solid -- und weil z = 1 ueber der Fahrbahn genau
			// das ist, scheiterte JEDE Saeule sofort und der Nachweis meldete "keine freie Saeule".
			// Dieselbe Falle wurde in render_yslice schon einmal gefunden; ich bin trotzdem wieder
			// hineingelaufen. Solid ist ausschliesslich (flags & (TYPE_S|TYPE_E)) == TYPE_S.
			if((L.flags[n]&(TYPE_S|TYPE_E))==TYPE_S) ok=false; // echte Wand oder Koerper -- Saeule unbrauchbar
		}
		if(ok) { xs=xt; frei=true; }
	}
	if(!frei) { print_info(string("  Wandwirksamkeit ")+wo+": keine freie Messsaeule stromauf gefunden -- Nachweis uebersprungen (KEIN Befund ueber die Wand)."); return; }
	string profil=""; float u_min=1e30f, u_max=-1e30f;
	for(uint z=1u; z<8u; z++) {
		const ulong n = (ulong)xs + ((ulong)ys + (ulong)z*(ulong)Ny)*(ulong)Nx;
		const float r = L.u.x[n]/u_lat; u_min = fmin(u_min, r); u_max = fmax(u_max, r);
		profil += (z>1u?", ":"")+string("z")+to_string(z)+"="+to_string(r,3u);
	}
	print_info(string("  Wandwirksamkeit ")+wo+" bei x="+to_string(xs)+", y="+to_string(ys)+" (u_x/u_inf): "+profil);
	// ★ Nachpruefer-Befund: der Test war EINSEITIG. Der Sollwert ist exakt 1,000 -- eine Wand, die
	// 30 % zu schnell laeuft (Skalenfehler, doppeltes u_lat, falsche Einheitenumrechnung), erzeugte
	// einen Ueberschuss und waere NIE gemeldet worden. Jetzt wird in beide Richtungen geprueft.
	if(u_min<0.90f) print_warning(string(wo)+": ueber der mitbewegten Fahrbahn faellt u_x auf "+to_string(u_min,3u)
		+" von u_inf ab. Bei u_wand = u_inf darf es KEINE Grenzschicht geben -- die Wand uebertraegt keinen Impuls (genau V1s Fehlerbild).");
	if(u_max>1.10f) print_warning(string(wo)+": u_x steigt auf "+to_string(u_max,3u)
		+" von u_inf -- die Wand ist zu SCHNELL oder es steht eine Versperrung im Weg.");
}

void main_setup_kugel() {
	// ---------------------------------------------------------------- Physik
	const float si_u   = 30.0f;                          // Anstroemung und Bodengeschwindigkeit [m/s]
	const float si_rho = 1.225f;                         // ISA Meereshoehe [kg/m^3]
	// ν per Env fuer die Validierung gegen die Standard-Widerstandskurve.
	// ★ KORREKTUR 2026-08-08 (Gutachter-Befund, selbst nachgerechnet): tau ist NICHT "0.53..0.58".
	// Bei dx=12mm und u_lat=0.075 gilt tau = 0.5844 bei Re_D=100, aber nur **0.5084** bei Re_D=1000.
	// Der obere Wert liegt damit sehr nah an 0.5, wo die effektive Wandposition des SRT-Bounce-Back
	// stark viskositaetsabhaengig wird (Ginzburg/Verhaeghe/d'Humieres 2008) -- der effektive
	// Kugelradius verschiebt sich also mit tau. Wer bei Re=1000 misst, misst das mit.
	// Damit lassen sich Voxelisierung, Bounce-Back und Kraftintegration isoliert pruefen.
	// Der Defaultwert ist die Luft bei 20 Grad C. (Der alte Baum hatte hier 1.48e-5 stehen und
	// gleichzeitig 1.51e-5 dokumentiert -- ein Widerspruch, der nie aufgeloest wurde.)
	const float si_nu  = env_f("CFD_KUGEL_NU", 1.48e-5f);
	const float D      = 0.450f;                         // Kugeldurchmesser [m]
	const float dx     = 0.001f*env_f("CFD_KUGEL_DX", 12.0f);
	const float si_zc  = env_f("CFD_KUGEL_ZC", 0.300f);  // Kugelmittelpunkt ueber Boden [m]
	const float u_lat  = 0.075f;
	const float dt     = u_lat*dx/si_u;
	const float nu_lat = si_nu*dt/(dx*dx);
	const float tau    = 3.0f*nu_lat + 0.5f;
	const float Re_D   = si_u*D/si_nu;

	// ---------------------------------------------------------------- Domaene (Abstaende ab Kugelmittelpunkt)
	const int off_3D    = (int)ceil(3.0f*D/dx);          // x-, y+-, z+
	const int off_9D    = (int)ceil(9.0f*D/dx);          // x+
	const int Dx_center = off_3D;
	const int Dy_center = off_3D;
	const int Dz_center = (int)(si_zc/dx + 0.5f);
	const uint Nx = (uint)(Dx_center + off_9D + 1);
	const uint Ny = (uint)(2*off_3D + 1);
	const uint Nz = (uint)(Dz_center + off_3D + 1);
	const float gap_si = si_zc - 0.5f*D;                 // Bodenspalt [m]

	// Mitbewegter Boden: alle vier Kanalwaende laufen mit U. CFD_KUGEL_MG=0 macht sie statisch.
	const bool moving_ground = getenv("CFD_KUGEL_MG")==nullptr ? true : env_on("CFD_KUGEL_MG");
	const float u_wall = moving_ground ? u_lat : 0.0f;

	// Block-Tiling (sparse solid). MUSS vor dem LBM-Konstruktor gesetzt werden, weil allocate() dann fi
	// nur als Platzhalter anlegt. VRAM-gegen-Tempo-Regler, physikalisch bit-neutral:
	//   CFD_TILE=8  -> etwa -40 % Durchsatz, dafuer die groesste VRAM-Ersparnis
	//   CFD_TILE=16 -> etwa -28 % Durchsatz, dafuer nur die halbe Ersparnis
	// Lohnt sich nur, wenn ein Fall sonst nicht in den Speicher passt. An der Kugel ist der Gewinn
	// klein, weil fast nichts voll solid ist -- der Nutzen liegt bei gefuellten Modellen.
	LBM_Domain::s_sparse_tiles_on = env_on("CFD_SPARSE_TILES");
	if(LBM_Domain::s_sparse_tiles_on) {
		const uint T = max(1u, env_u("CFD_TILE", 8u)); // Kantenlaenge 0 gibt es nicht
		if(T!=8u && T!=16u && T!=32u && T!=64u) print_error("CFD_TILE muss 8, 16, 32 oder 64 sein (erhalten: "+to_string(T)+").");
		LBM_Domain::s_sparse_T = T;
		print_info("Block-Tiling AKTIV, T="+to_string(T)+" (VRAM sparen auf Kosten von Durchsatz)");
	}

	units.set_m_kg_s(D/dx, u_lat, 1.0f, D, si_u, si_rho);
	// ★ Die Daempfungszone gibt es in diesem Fall NICHT, und das wird GESAGT statt still geschluckt:
	// die Zone rampt am DOMAENENRAND, und hier sind y+/y- und z+ mitbewegte Waende -- eine Zone dort
	// verdickte kuenstlich die Wandgrenzschichten. Sinnvoll ist sie nur in fernfeld und fahrzeug_dd,
	// dort nur am groben Gitter. Ein still wirkungsloser Schalter waere genau die Fehlerklasse,
	// die dieses Projekt sonst jagt.
	if(env_u("CFD_SPONGE_N", 0u)>0u) print_warning("CFD_SPONGE_N ist gesetzt, wird in diesem Fall aber NICHT angewandt (die Zone rampt am Domaenenrand, dort stehen hier mitbewegte Waende). Nur fernfeld und fahrzeug_dd nutzen sie.");
	LBM_Domain::s_sponge_n = 0u; LBM_Domain::s_sponge_a = 3000.0f; LBM_Domain::s_sponge_wmin = 0.5f; LBM_Domain::s_sgs_wandfrei = env_u("CFD_SGS_WANDFREI", 0u)>0u; LBM_Domain::s_sgs_guo = env_u("CFD_SGS_GUO", 1u)>0u; LBM_Domain::s_sgs_diag = env_u("CFD_SGS_DIAG", 0u)>0u; LBM_Domain::s_sgs_diag_ab = (ulong)env_u("CFD_SGS_DIAG_AB", 0u); // alle drei, damit keine Instanz einen Wert der vorigen erbt
	// ★ Audit-Nacharbeit 6/10: Wandfunktions-Statiken an JEDER Konstruktorstelle setzen; der Schalter
	// gilt nur im Kanal -- hier wird er angesagt statt lautlos verschluckt.
	LBM_Domain::s_wandfunktion = false; LBM_Domain::s_wf_tau = 1.0f;
	if(env_u("CFD_WANDFUNKTION", 0u)>0u) print_warning("CFD_WANDFUNKTION wird in diesem Fall NICHT angewandt (nur kanal; Fahrzeug braucht erst Relativgeschwindigkeit und Facetten).");
	// ★ C1b: Kugel ist der Lackmustest des iMEM (Einzellink-Zellen!). Arme 1/2 Paartausch, 3/4 iMEM.
	{ const uint fc = env_u("CFD_FACETTEN", 0u);
	  if(fc>4u) print_error("CFD_FACETTEN kennt nur 0..4 (1/2 Paartausch voll/Tausch, 3/4 iMEM voll/Nullziel).");
	  LBM_Domain::s_facetten = fc>0u; LBM_Domain::s_fac_imem = fc>=3u;
	  LBM_Domain::s_fac_ema = (fc>=3u) ? env_f("CFD_FAC_EMA", 0.0f) : 0.0f;
	  LBM_Domain::s_fac_pema = (fc>=3u) ? env_f("CFD_FAC_PEMA", 0.0f) : 0.0f;
	  if(getenv("CFD_FAC_DIAGZ")!=nullptr) print_warning("CFD_FAC_DIAGZ ist im Kugelfall noch NICHT verdrahtet (IR3-Audit) -- Diagnose nur im Kanal/Torus.");
	  LBM_Domain::s_fac_pinv = env_u("CFD_FAC_PINV", 0u); if(LBM_Domain::s_fac_pinv>0u&&env_u("CFD_FAC_LSQ",0u)>0u) print_error("CFD_FAC_PINV und CFD_FAC_LSQ schliessen sich aus -- PINV ersetzt denselben Zweig, LSQ waere still wirkungslos (der #elif faellt durch). Einen von beiden waehlen."); if(LBM_Domain::s_fac_pinv>0u) print_info("RANG-1-PSEUDOINVERSE (CFD_FAC_PINV, 04.09.2026): im gekoppelten Zweig ersetzt Moore-Penrose die achsenparallele Skalarleiter -- Division ueber die SPUR (groesster Eigenwert) statt ueber Gt11. Grund: fuer die ebene Voxelwand ist tr(Gt) exakt 1/3 und kippungsunabhaengig, waehrend Gt11 mit der Stroemungsrichtung gegen 0 laeuft und die Akzeptanzschwelle 1e-4 dann Verstaerkung bis 1e4 durchlaesst. Wirkpfad Slot 80; erwartet fallen Slot 10 UND Slot 16, weil der Eigenvektor Sn.v = 0 exakt erfuellt und damit keine Normalkompensation mehr erzeugt wird."); LBM_Domain::s_fac_idx_voll = env_u("CFD_FAC_IDX_VOLL", 0u); if(LBM_Domain::s_fac_idx_voll>0u) print_info("CFD_FAC_IDX_VOLL=1: fac_idx in der ALTEN Vollfeldform -- deklarierter A/B-Arm gegen die Bitmaske (03.09.). Die Ergebnisse MUESSEN bitgleich sein, unterscheiden darf sich nur der Speicher."); LBM_Domain::s_f_liste = env_u("CFD_F_LISTE", 0u); if(LBM_Domain::s_f_liste>0u) print_info("F-MARKERLISTE (CFD_F_LISTE, 03.09.2026, Befunde B78b/B80/B81): F wird nur fuer WANDsolidzellen alloziert -- 8 mm gemessen: Nahfeld 238 -> 14 MiB, Fernfeld 4 -> 0 MiB. ABGENOMMEN ueber alle drei Sprossen bitgleich (CPU 5/5, iGPU 5/5 und dreimal reproduziert, B70 8-mm-Fahrzeug 19/19), Slot 77 = 0. Der urspruengliche Defekt war NICHT die Liste, sondern die Reihenfolge: die JIT-Defines entstanden vor dem Setzen der Schalter (B81)."); if(LBM_Domain::s_f_liste>0u&&!f_nur_solid_an_setup()) print_error("CFD_F_LISTE braucht CFD_F_NUR_SOLID (Default an): der Kontrollarm CFD_F_NUR_SOLID=0 liest F an JEDER Fluidzelle, und dort gibt es unter der Markerliste keinen Speicherplatz mehr -- die Kombination waere still falsch."); LBM_Domain::s_fac_satgate = fc>=3u&&env_u("CFD_FAC_SATGATE", 0u)>0u; LBM_Domain::s_fac_kraft = fc>=3u ? min(2u, env_u("CFD_FAC_KRAFT", 0u)) : 0u; LBM_Domain::s_fac_kdiag = fc>=3u ? env_u("CFD_FAC_KDIAG", 0u) : 0u; if(fc<3u&&(env_u("CFD_FAC_NACHBAR",0u)>0u||env_u("CFD_FAC_KDIAG",0u)>0u)) print_error("CFD_FAC_NACHBAR/CFD_FAC_KDIAG brauchen CFD_FACETTEN=3 (iMEM) -- bei CFD_FACETTEN="+to_string((ulong)fc)+" wuerde der Schalter still auf 0 gesetzt (No-Op-Waechter 03.09.)."); LBM_Domain::s_fac_nachbar = fc>=3u ? env_u("CFD_FAC_NACHBAR", 0u) : 0u; LBM_Domain::s_fac_messnur = fc>=3u ? env_u("CFD_FAC_MESSNUR", 0u) : 0u; LBM_Domain::s_sgs_gdiag = fc>=1u ? env_u("CFD_SGS_GDIAG", 0u) : 0u; LBM_Domain::s_sgs_fdwand = fc>=1u ? env_u("CFD_SGS_FDWAND", 0u) : 0u; if(LBM_Domain::s_sgs_fdwand>0u) { print_info("SGS-GEISTERMODEN-FIX (CFD_SGS_FDWAND, 02.09.): an Facettenzellen kommt die SGS-Relaxationsrate aus |S|_FD des u-Felds (FD-Kernel je Schritt, ein Schritt Versatz) statt aus dem wandmodell-kontaminierten Pi-Tensor (B66/B69: Pi/FD 2,3-3,4). Wirkpfad Slot 76 (B70)."); if(env_u("CFD_FAC_MESSNUR",0u)>0u) print_warning("SGS_FDWAND + MESS-NUR: der Arm ist dann NICHT mehr reines Bounce-Back -- das Kollisions-w an Wandzellen kommt aus dem FD-Pfad (bewusste Kombination fuer BB+FDWAND-Messungen, aber nicht mit alten BB-Bezuegen bitvergleichbar)."); if(env_u("CFD_SGS_WANDFREI",0u)>0u) print_warning("SGS_FDWAND + SGS_WANDFREI: WANDFREI hat an Wandzellen VORRANG -- FDWAND ist dort wirkungslos (Slot 76 bleibt 0). Fuer den FDWAND-Arm WANDFREI abschalten."); } if(LBM_Domain::s_sgs_gdiag>0u) { sgs_gdiag_selbsttest(); print_info("g-DIAGNOSE (CFD_SGS_GDIAG, 31.08.): Messkernel ueber die Wandzellen -- |S|_FD (u-Feld, geistermodenfrei), |S|_Pi (fneq, wie Smagorinsky), D_WALE, D_Sigma, |Omega|. Physik unangetastet, Bericht am Laufende."); } if(LBM_Domain::s_fac_messnur>0u&&env_u("CFD_FAC_NACHBAR",0u)>0u) print_warning("MESS-NUR + NACHBAR: die Nachbarabtastung liegt hinter dem MESS-NUR-Ausstieg und ist WIRKUNGSLOS (Slots 72-74 bleiben 0)."); if(LBM_Domain::s_fac_messnur>0u&&env_u("CFD_FAC_KDIAG",0u)>0u) print_warning("MESS-NUR + KDIAG: die Klassen-Diagnostik wird nie akkumuliert -- die Tabelle am Laufende ist eine Nulltabelle."); if(LBM_Domain::s_fac_messnur>0u&&env_u("CFD_FAC_KRAFT",0u)>0u) print_error("MESS-NUR + KRAFT ist unsinnig (kein Wandmodell -> kein Residuum; Modus 2 stuerbe irrefuehrend am Kraftpfad-Pruefer). Kombination aufloesen."); if(LBM_Domain::s_fac_messnur>0u) print_warning("MESS-NUR (CFD_FAC_MESSNUR, 30.08.): der Kernel wendet KEIN Wandmodell an -- die Wand ist reines Bounce-Back. Facetten werden nur gebaut und gemessen, damit der Druckpfad (cd_facetten.csv) als Aepfel-mit-Aepfeln-Bezug zu einem Wandmodell-Arm dient. Der REIBUNGSanteil ist in diesem Arm konstruktiv 0; belastbar ist der Druckanteil (bei Cz 99,5 %). Slot 75 = Wirkpfad (2. Umzug, B70). Die ELIBB-Blende wird unter MESS-NUR seit B-4 ebenfalls uebersprungen -- der Arm ist exakt reines Bounce-Back."); if(LBM_Domain::s_fac_nachbar>0u) print_info("NACHBARABTASTUNG (CFD_FAC_NACHBAR, 30.08.): Wandmodell-Eingang (u_t, Wandabstand) aus der zweiten Fluidzelle entlang der Normale statt aus der Wandzelle -- Stufenschatten-Fix. Slots 72 (angewandt) / 73 (kein Fluidnachbar) / 74 (Nachbar steht still; 2. Umzug 02.09., B70 -- 35-48 gehoeren SGS_DIAG ueber berechnete Indizes)."); if(LBM_Domain::s_fac_kraft>0u) print_info(string("iMEM-KRAFTPFAD (Weg F, 30.08.): Modus ")+to_string(LBM_Domain::s_fac_kraft)+(LBM_Domain::s_fac_kraft==1u?string(" -- Residuum R als Volumenkraft an RUECKFALLZELLEN (statt s=0); Slot 70, Soll == Slot 69."):string(" -- ALLE Facettenzellen per Kraft, Additivterm aus (Diskriminator gegen den Slip-Pfad); Slot 70."))); if(LBM_Domain::s_fac_kraft>0u) print_warning("KRAFTPFAD (Pruefpunkt 8, 30.08.): object_force/forces.csv (Impulsaustausch an Koerperzellen) sieht die Volumenkraft NICHT -- eine Guo-Kraft im Fluid hat keine Newton-3-Reaktion am Koerper. Der Reibungsanteil an Kraftzellen steht allein in der fac_tau-Buchung (cd_reib/cd_rest); object_force-Abgleiche (K4, Fx_far) weichen um genau den Kraftanteil ab."); LBM_Domain::s_fac_elibb = false; LBM_Domain::s_fac_qmin = 0.1f; LBM_Domain::s_fac_lsq = false; LBM_Domain::s_fac_quergate = false; // Statik-Symmetrie (H1)
	  LBM_Domain::s_fac_diagz = -1l; // ★ Audit 2/3: 9. Statik an dieser Stelle -- DIAGZ ist an der Kugel (noch) nicht verdrahtet, Warnung oben
	  if(LBM_Domain::s_fac_satgate) print_info("iMEM-Saettigungs-Gate aktiv (a-strich): Budget-Riss -> BB-Rueckfall statt Klemme (Slots 10/16 = Rueckfaelle; seit Buchungsschluss 27.08. buchen Rueckfaelle P-only, Slot 69)."); // Audit 3/3 N2: Ansage-Doktrin auch an der Kugel
	  if(LBM_Domain::s_fac_ema>0.0f) print_warning("CFD_FAC_EMA (Loesungs-Filterung) ist in J3 WIDERLEGT -- nur noch als A/B-Arm sinnvoll.");
	  if(LBM_Domain::s_fac_ema>0.0f&&LBM_Domain::s_fac_pema>0.0f) print_warning("CFD_FAC_EMA und CFD_FAC_PEMA GLEICHZEITIG: zwei kompoundierende Lags -- als Messarm wertlos (IR3-Audit).");
	  if(LBM_Domain::s_fac_pema>0.0f) print_info("iMEM-PEMA aktiv (Weg A, Eingangs-Filterung): alpha = "+to_string(LBM_Domain::s_fac_pema,5u)+", Zeitkonstante ~"+to_string((uint)(1.0f/LBM_Domain::s_fac_pema))+" Schritte");
	  LBM_Domain::s_fac_alpha = (fc>=3u) ? env_u("CFD_FAC_ALPHA", 0u) : 0u; LBM_Domain::s_fac_lsq = env_u("CFD_FAC_LSQ", 0u)>0u; LBM_Domain::s_fac_quergate = env_u("CFD_FAC_QUERGATE", 0u)>0u; LBM_Domain::s_fac_elibb = env_u("CFD_FAC_ELIBB", 0u)>0u; LBM_Domain::s_fac_elibb_pur = env_u("CFD_FAC_ELIBB", 0u)==2u; LBM_Domain::s_fac_qmin = env_f("CFD_FAC_QMIN", 0.1f); LBM_Domain::s_fac_kappa = env_f("CFD_FAC_KAPPA", 0.4f); LBM_Domain::s_fac_utkorr = env_f("CFD_FAC_UTKORR", 1.0f); LBM_Domain::s_fac_qkappe = env_f("CFD_FAC_QKAPPE", 1.0f); LBM_Domain::s_fac_qdiag = env_u("CFD_FAC_QDIAG", 0u); LBM_Domain::s_sgs_guo = env_u("CFD_SGS_GUO", 1u)>0u;
	  if(LBM_Domain::s_fac_alpha>2u) print_error("CFD_FAC_ALPHA kennt nur 0..2 (1 = Massenkorrektur, 2 = + Momenten-Downdate). Die Stufe 3 (A2-Rueckfall) wurde am 2026-08-25 als beweisbar wirkungslos zurueckgenommen -- q_i ist fuer Einzellink-Facetten unter alpha identisch null.");
	  LBM_Domain::s_fac_apg = (fc>=3u) ? env_f("CFD_FAC_APG", 0.0f) : 0.0f;
	  if(LBM_Domain::s_fac_apg!=0.0f&&LBM_Domain::s_fac_pema>0.0f) print_error("CFD_FAC_APG + CFD_FAC_PEMA sind noch NICHT kombiniert (gefilterte Kette braucht eigenen APG-Zweig -- eigener Bauabschnitt).");
	  if(LBM_Domain::s_fac_apg!=0.0f) print_info("APG-Messarm aktiv: tw-Ziel um kappa*y_w*dp/ds korrigiert, kappa = "+to_string(LBM_Domain::s_fac_apg,4u)+" -- Slot 19 zaehlt beide Klemmen (0 / 2*tw).");
	  if(LBM_Domain::s_fac_alpha>0u) print_info(string("iMEM-alpha-Massenkorrektur Stufe ")+to_string(LBM_Domain::s_fac_alpha)+(LBM_Domain::s_fac_alpha==2u?string(" (Masse + Momenten-Downdate: Impulsziel inkl. alpha exakt)"):string(" (NUR Masse -- injiziert alpha*S1-Impuls, reiner Messarm)"))+" -- Slot 18 zaehlt alpha>u_t.");
	  if(fc>0u&&env_f("CFD_FACETTEN_YWMIN",0.2f)>=0.187f) print_warning("Kugel: der K4-Ring liegt bei y_w=0,188 -- Default-YWMIN 0,2 schliesst ihn stumm aus (J4-Befund #2). Fuer volle Abdeckung CFD_FACETTEN_YWMIN=0.15 setzen (deklarierter Messarm).");
	  LBM_Domain::s_fac_tau = (fc==2u||fc==4u) ? 0.0f : 1.0f;
	  LBM_Domain::s_fac_budget = fmax(0.25f, fmin(4.0f, env_f("CFD_FAC_BUDGET", 1.0f)));       // 1a-B4t: Tangentialbudget-Skalar (geklemmt 0,25..4; Ansage unten)
	  LBM_Domain::s_fac_budget_sn = fmax(0.25f, fmin(4.0f, env_f("CFD_FAC_BUDGET_SN", 1.0f))); // 1a-Bsn: sn-Budget-Skalar
	  if(LBM_Domain::s_fac_budget!=1.0f) print_info("FACETTEN BUDGET (1a-B4t): Tangentialbudget x "+to_string(LBM_Domain::s_fac_budget,2u)+" (|s1| <= 2ut*k, |s2| <= ut*k). Die +-2ut-Budgets sind Design, nie geeicht (Planungsagent 2026-08-22). Erfolgskriterium: Slot-10-Anteil faellt UND cd_druck/cz_rest Richtung OF13 UND y+-Median nicht > +15 %.");
	  if(LBM_Domain::s_fac_budget_sn!=1.0f) print_info("FACETTEN BUDGET_SN (1a-Bsn): sn-Budget x "+to_string(LBM_Domain::s_fac_budget_sn,2u)+". Verschlechtert sich cd_druck > 2 %, ist der Arm verworfen (sn beruehrt den Druckpfad).");
	  LBM_Domain::s_boden_eq_n = env_u("CFD_BODEN_EQ", 0u); LBM_Domain::s_boden_eq_u = u_lat; // Kugel: optionaler Messarm
	  if(LBM_Domain::s_boden_eq_n>0u) print_info("BODEN_EQ Kugel aktiv: z=1.."+to_string(LBM_Domain::s_boden_eq_n)+" (V1-Port; OHNE V1-side_nz -- y/z-Waende unbehandelt, XL-B5-Notiz)."); // B3
	  LBM_Domain::s_boden_eq_abstand = env_u("CFD_BODEN_EQ_ABSTAND", 0u);
	  if(LBM_Domain::s_boden_eq_abstand>3u&&LBM_Domain::s_boden_eq_n>0u) print_warning("CFD_BODEN_EQ_ABSTAND > 3: Scan-Kosten wachsen kubisch (XL-R2).");
	  LBM_Domain::s_boden_eq_down = 0u; LBM_Domain::s_boden_eq_split = 0xFFFFFFFFu; LBM_Domain::s_einlass_eq_n = 0u; LBM_Domain::s_schale_alpha = 0.0f; LBM_Domain::s_schale_paritaet = false; // Statik-Symmetrie VOLL (Pruefagent R2 N2): Kugel = uniformes Band ohne Split
	  if(getenv("CFD_BODEN_EQ_DOWN")||getenv("CFD_FERN_BODEN_EQ")||getenv("CFD_FERN_EINLASS_EQ")) print_warning("CFD_BODEN_EQ_DOWN/CFD_FERN_BODEN_EQ/CFD_FERN_EINLASS_EQ wirken an der Kugel NICHT (uniformes Band ohne Split; XL-R2).");
	  if(getenv("CFD_KOPPLUNG_ZEITINTERP")||getenv("CFD_KOPPLUNG_GLATT")) print_warning("CFD_KOPPLUNG_ZEITINTERP/GLATT werden an der Kugel NICHT angewandt (nur fahrzeug_dd; M3).");
	if(getenv("CFD_SLICE_NEAR_STEPS")||getenv("CFD_VTK_JEDE")||getenv("CFD_VTK_BEHALTE")) print_warning("CFD_SLICE_NEAR_STEPS/CFD_VTK_JEDE/CFD_VTK_BEHALTE werden in diesem Fall NICHT angewandt (nur fahrzeug_dd; Kadenz-Umbau 27.08.).");
	  { const char* n2f_[] = {"CFD_N2F_SCHALE","CFD_N2F_VOLUMEN","CFD_N2F_BAND","CFD_N2F_BAND_N","CFD_N2F_BAND_PROFIL","CFD_N2F_BAND_UNTERBODEN","CFD_N2F_BAND_WAKE","CFD_N2F_BAND_NURWAKE","CFD_N2F_BAND_WAKE_START","CFD_N2F_BAND_WAKE_START_X","CFD_N2F_BAND_WAKE_ABSTAND","CFD_N2F_PARITAET"}; for(const char* b : n2f_) if(getenv(b)) print_warning(string(b)+" ist gesetzt, wird aber NUR im fahrzeug_dd-Fall angewandt (P9c; die neun BAND-/WAKE-/PARITAET-Schalter fehlten bis 2026-08-22 in dieser Ansage -- Pruefagent-S1)."); } // Ansage-Doktrin
	  if(env_u("CFD_FERN_FACETTEN", 0u)>0u) print_warning("CFD_FERN_FACETTEN wird an der Kugel NICHT angewandt (nur fahrzeug_dd -- P8; Ansage-Doktrin).");
	  if(LBM_Domain::s_boden_eq_n>3u) print_warning("CFD_BODEN_EQ > 3 verletzt die Heiko-Vorgabe (max 3, besser 2).");
	  if(LBM_Domain::s_boden_eq_n>0u&&getenv("CFD_KUGEL_MG")&&env_u("CFD_KUGEL_MG",1u)==0u) print_warning("BODEN_EQ mit CFD_KUGEL_MG=0: statischer Boden + u_road-Aufpraegung widersprechen sich (XL-3 B8).");
	  if(fc>0u) print_info(string("Facettenpfad Kugel: ")+(fc==1u?"Paartausch voll":fc==2u?"Paartausch NUR TAUSCH":fc==3u?"iMEM voll":"iMEM NULLZIEL")+" -- Impulsaustausch-Cd an behandelten Links kontaminiert, nur der projizierte Cd-Pfad zaehlt."); }
	LBM lbm(Nx, Ny, Nz, nu_lat);

	print_info("=============== Kugel im Kanal (Neuaufbau auf Upstream 8986874) ===============");
	print_info("D = "+to_string(D*1000.0f,1u)+" mm, dx = "+to_string(dx*1000.0f,3u)+" mm, Re_D = "+to_string((uint)Re_D));
	print_info("U = "+to_string(si_u,1u)+" m/s, rho = "+to_string(si_rho,4u)+" kg/m3, nu = "+to_string(si_nu,3u)+" m2/s");
	print_info("Gitter: "+to_string(Nx)+" x "+to_string(Ny)+" x "+to_string(Nz)+" = "+to_string((ulong)Nx*Ny*Nz)+" Zellen");
	print_info("u_lat = "+to_string(u_lat,4u)+", tau = "+to_string(tau,5u)+", dt = "+to_string(dt*1e6f,3u)+" us");
	print_info("Bodenspalt h = "+to_string(gap_si*1000.0f,1u)+" mm, h/D = "+to_string(gap_si/D,3u));
	print_info(moving_ground ? "Boden und Seitenwaende: MITBEWEGT mit U" : "Boden und Seitenwaende: STATISCH (No-Slip)");

	// ---------------------------------------------------------------- Kugel voxelisieren
	// STL ROH lesen und dann EXPLIZIT ueber die gemessene Bounding-Box skalieren. Die
	// auto-reposition-Variante von read_stl() hat eine andere Skalensemantik als erwartet --
	// sie lieferte eine Kugel mit R_eff = 7,05 statt 18,75 Zellen, also 38 % der Sollgroesse.
	// Explizit skalieren heisst: die Zahl ist nachrechenbar, statt von einer Konvention abzuhaengen.
	const string stl_path = get_exe_path()+"../Kugel.stl";
	Mesh* kugel = read_stl(stl_path);
	const float3 bbox0 = kugel->get_bounding_box_size();
	print_info("STL geladen, BBox roh: "+to_string(bbox0.x,4u)+" x "+to_string(bbox0.y,4u)+" x "+to_string(bbox0.z,4u));
	kugel->scale((D/dx)/bbox0.x);
	const float3 ctr = kugel->get_bounding_box_center();
	kugel->translate(float3((float)Dx_center-ctr.x, (float)Dy_center-ctr.y, (float)Dz_center-ctr.z));
	lbm.voxelize_mesh_on_device(kugel, TYPE_S|TYPE_X);
	lbm.flags.read_from_device();

	sat_shell_and_void_fill(lbm, kugel, Nx, Ny, Nz);

	// Flaechenaequivalenter Ist-Radius aus dem echten Querschnitt in der Symmetrieebene.
	// Daraus die effektive Stirnflaeche -- die Zahl, um die Cd systematisch zu hoch liegt.
	ulong sym_solid = 0ull;
	for(uint z=0u; z<Nz; z++) for(uint x=0u; x<Nx; x++) {
		if((lbm.flags[(ulong)x + ((ulong)Dy_center + (ulong)z*(ulong)Ny)*(ulong)Nx]&TYPE_X)!=0u) sym_solid++;
	}
	const float R_analytic = 0.5f*D/dx;
	const float R_eff      = sqrt((float)sym_solid/(float)M_PI);
	const float A_eff_ratio = R_analytic>0.0f ? (R_eff/R_analytic)*(R_eff/R_analytic) : 1.0f;
	print_info("Symmetrieebene: "+to_string(sym_solid)+" Solidzellen -> R_eff = "+to_string(R_eff,2u)
		+" Z gegen R_analytisch "+to_string(R_analytic,2u)+" Z");
	print_info("Effektive Stirnflaeche: x"+to_string(A_eff_ratio,4u)+" = +"+to_string(100.0f*(A_eff_ratio-1.0f),1u)+" % gegenueber nominal");

	// ---------------------------------------------------------------- Randbedingungen
	// CFD_KUGEL_FREE=1: y+-, z+- werden FREISTROM (TYPE_E) statt mitbewegte Waende. Dann steht die Kugel
	// frei, ohne Bodeneffekt und ohne Kanalblockage -- die Voraussetzung, um gegen die
	// Standard-Widerstandskurve (Clift/Grace/Weber) zu messen. Der Bodeneffekt bei h/D = 0,167 senkt Cd
	// um rund 20 % und macht jeden Literaturvergleich sinnlos.
	const bool free_stream = env_on("CFD_KUGEL_FREE");
	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
		const ulong n = (ulong)x + ((ulong)y + (ulong)z*(ulong)Ny)*(ulong)Nx;
		if((lbm.flags[n]&(TYPE_S|TYPE_X))!=0u) continue;           // Kugel unangetastet
		if(!free_stream && (z==0u || z==Nz-1u || y==0u || y==Ny-1u)) { // Boden, Decke, Seitenwaende
			lbm.flags[n] = TYPE_S; lbm.u.x[n] = u_wall; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
		} else if(x==0u || x==Nx-1u || (free_stream && (z==0u || z==Nz-1u || y==0u || y==Ny-1u))) {
			lbm.flags[n] = TYPE_E; lbm.u.x[n] = u_lat; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
		} else {
			lbm.u.x[n] = u_lat; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
		}
	}
	if(free_stream) print_info("FREISTROM: y+- und z+- sind TYPE_E statt Wand -- Kugel steht frei, kein Bodeneffekt");

	// Druck-Auslass an x+. NICHT Zou-He -- siehe die ausfuehrliche Einordnung bei
	// LBM_Domain::set_pressure_outlet_faces. Es ist ein Gleichgewichtsrand mit vorgeschriebener Dichte
	// und extrapolierter Geschwindigkeit. Echtes Zou-He ist fuer D3Q19 gar nicht eindeutig definiert:
	// es braucht hoechstens vier einwaerts zeigende Links, eine ebene D3Q19-Flaeche hat fuenf
	// (Maier/Bernard/Grunau, Phys. Fluids 8, 1996).
	// Ohne diesen Rand erzwingt der blosse TYPE_E-Auslass Druck UND Geschwindigkeit und ueberbestimmt
	// den Ausfluss. Nach der Randbedingungs-Schleife aufrufen: er sammelt die dort gesetzten TYPE_E-Zellen.
	// CFD_PO_FACES ueberschreibt die Maske. Default 2 = nur x_max. Im Freistrom-Fall lassen sich damit
	// auch die Seitenflaechen als Auslass fahren (0x3E = x_max + y+- + z+-) -- das ist zugleich der Test,
	// ob Kanten und Ecken sauber behandelt werden, denn dort liegt eine Zelle auf zwei oder drei Flaechen.
	const uint po_faces = env_u("CFD_PO_FACES", 2u);
	lbm.set_pressure_outlet_faces(po_faces, env_f("CFD_PO_RHO", 1.0f));

	// Tiles erst JETZT klassifizieren: der Klassifizierer liest die Flags, und die Waende oben sind
	// ebenfalls TYPE_S. Liefe finalize schon nach der Voxelisierung, klassifizierte es gegen ein
	// Zwischenbild der Geometrie. Bei ausgeschaltetem Sparse ist der Aufruf ein No-op.
	lbm.finalize_sparse_tiles();

	// ---------------------------------------------------------------- Laufsteuerung
	// Laufzeit in DURCHSPUELUNGEN (Domaenenlaenge / Anstroemung) statt in Sekunden. Zwei als Default,
	// solange diagnostiziert wird: die erste ist Anlauf, ueber die zweite wird gemittelt.
	const float t_flush  = (float)Nx*dx/si_u;
	const float t_end    = env_f("CFD_T_END", 2.0f*t_flush);
	const float t_warmup = env_f("CFD_T_WARMUP", 1.0f*t_flush);
	print_info("Eine Durchspuelung = "+to_string(t_flush,4u)+" s; Default sind zwei davon.");
	const uint  sample_every = max(1u, env_u("CFD_SAMPLE_EVERY", 10u)); // 0 waere eine Endlosschleife -- hier klemmen, nicht im Helfer
	const ulong n_steps  = (ulong)(t_end/dt + 0.5f);
	const string run_name = getenv("CFD_RUN_NAME") ? string(getenv("CFD_RUN_NAME")) : string("kugel");
	const string out_dir = get_exe_path()+"../export/"+run_name+"/";
	create_folder(out_dir);
	sichere_lauf(out_dir, "kugel"); // ★ Heiko 2026-08-09: Sicherung MIT JEDEM Lauf, in allen vier Faellen

	const float A_nom = (float)M_PI*(0.5f*D)*(0.5f*D);
	const float q_inf = 0.5f*si_rho*si_u*si_u;
	print_info("Laufzeit "+to_string(t_end,3u)+" s = "+to_string(n_steps)+" Schritte, Mittelung ab "+to_string(t_warmup,3u)+" s");
	print_info("A_nominal = "+to_string(A_nom,5u)+" m2, q_inf = "+to_string(q_inf,2u)+" Pa");

	// ---------------------------------------------------------------- Zeitschleife
	std::vector<double> ts, fx, fy, fz;
	std::vector<double> fac_snap; ulong fac_snap_step=0ull; // Cd-Pfad
	// ★ Druck-ZEITMITTEL (Plan E6, Morgen-TODO vom 2026-08-15): der Druckanteil wird an der
	// Sample-Kadenz gemittelt statt als End-Momentaufnahme gelesen. CFD_FAC_CD_EVERY duennt aus.
	double fac_px=0.0, fac_py=0.0, fac_pz=0.0; ulong fac_pn=0ull, fac_cd_i=0ull;
	std::ofstream fac_csv;
	// ★ FORK Kraft-Zerlegung (CFD_KRAFT_ZBAND, dasselbe Env wie im dd-Fall): die Kugel schwebt frei,
	// das Band muss ~0 liefern -- Negativ-Kontrolle. unset/0 = AUS = bitidentisch. EINMAL gelesen.
	const uint zb = env_u("CFD_KRAFT_ZBAND", 0u);
	double zb_fx_band=0.0, zb_fz_band=0.0, zb_fx_rest=0.0, zb_fz_rest=0.0, zb_selftest_max=0.0; ulong zb_nn=0ull;
	if(zb>0u&&zb>=Nz) print_error("CFD_KRAFT_ZBAND ("+to_string(zb)+") >= Nz ("+to_string(Nz)+") -- das Band muss unter der Domaenendecke bleiben.");
	if(zb>0u) print_info("KRAFT-ZBAND aktiv (Kugel, Negativ-Kontrolle): unterste "+to_string(zb)+" Zellen = "+to_string((float)zb*dx*1000.0f,2u)+" mm (dx = "+to_string(dx*1000.0f,2u)+" mm). GITTERBAND -- zwischen DX-Sprossen nicht direkt vergleichbar.");
	ts.reserve(n_steps/sample_every + 2ull);
	fx.reserve(n_steps/sample_every + 2ull); fy.reserve(fx.capacity()); fz.reserve(fx.capacity());
	std::unordered_map<ulong,std::array<uchar,18>> elibb_qmap; // ★ B1-Stufe 2: Remesh-q-Karte
	const bool elibb_an_kugel = env_u("CFD_FACETTEN",0u)>=3u&&env_u("CFD_FAC_ELIBB",0u)>0u;
	if(env_u("CFD_FACETTEN_REMESH", 0u)>0u||elibb_an_kugel) { // ★ ELIBB P1 + Stufe 2: bei ELIBB laeuft das Remesh PFLICHTIG (q-Quelle); die Kugel ist der einzige Fall mit ANALYTISCHEM q
		// Mitte und Radius kommen aus dem Voxelfeld selbst -- Schwerpunkt der Solidzellen und
		// volumenaequivalenter Radius. Die position()-Konvention (Zellindex gegen zentrierten
		// Positionsraum) ist hier uneindeutig; ein 0,5-Zellen-Irrtum wuerde einen Bias
		// vortaeuschen, den es nicht gibt. Beide Durchgaenge nutzen DIESELBE Referenz, der
		// Vergleich Treppe gegen Geglaettet bleibt also auch bei kleinem Radiusfehler gueltig.
		double sx=0.0, sy=0.0, sz=0.0; ulong nv_=0ull;
		for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++)
			if(lbm.flags[(ulong)x+((ulong)y+(ulong)z*(ulong)Ny)*(ulong)Nx]==(uchar)(TYPE_S|TYPE_X)) {
				sx+=(double)x+0.5; sy+=(double)y+0.5; sz+=(double)z+0.5; nv_++; }
		if(nv_>0ull) {
			const double R_vol=pow(3.0*(double)nv_/(4.0*M_PI), 1.0/3.0);
			const double kref[4]={sx/(double)nv_, sy/(double)nv_, sz/(double)nv_, R_vol};
			print_info("ELIBB P1 (Kugel): "+to_string(nv_)+" Solidzellen, Schwerpunkt ("+to_string((float)kref[0],2u)+", "+to_string((float)kref[1],2u)+", "+to_string((float)kref[2],2u)+"), volumenaequivalenter Radius "+to_string((float)R_vol,3u)+" Zellen (nominal "+to_string(0.5f*D/dx,3u)+").");
			remesh_facetten_diag(lbm, Nx, Ny, Nz, (uchar)(TYPE_S|TYPE_X), out_dir, kref, nullptr, elibb_an_kugel?&elibb_qmap:nullptr);
		} else print_warning("ELIBB P1 (Kugel): keine Solidzellen mit TYPE_S|TYPE_X -- Remesh uebersprungen.");
	}
	if(env_u("CFD_FACETTEN_DIAG", 0u)>0u&&env_u("CFD_FACETTEN", 0u)==0u) { // ★ Audit 3/3 M2: der Schalter war im Kugelfall stummer No-Op (DIAG=2 lief als Vollsimulation weiter)
		baue_facetten(lbm, Nx, Ny, Nz, (uchar)(TYPE_S|TYPE_X), out_dir, "Kugel-Census");
		if(env_u("CFD_FACETTEN_DIAG", 0u)==2u) _exit(0);
	}
	if(env_u("CFD_FACETTEN", 0u)>0u) {
		const ulong census_v = [&]{ ulong c=0ull; for(ulong n=0ull;n<lbm.get_N();n++) if(lbm.flags[n]==(TYPE_S|TYPE_X)) c++; return c; }();
		std::vector<Facette> FF = baue_facetten(lbm, Nx, Ny, Nz, (uchar)(TYPE_S|TYPE_X), out_dir, "Kugel");
		if(env_u("CFD_FACETTEN_DIAG", 0u)==2u) _exit(0); // Schritt-0-Diagnose auch im aktiven Arm
		const ulong census_n = [&]{ ulong c=0ull; for(ulong n=0ull;n<lbm.get_N();n++) if(lbm.flags[n]==(TYPE_S|TYPE_X)) c++; return c; }();
		if(census_v!=census_n) print_error("Facettenbau hat den 0x41-Census veraendert ("+to_string(census_v)+" -> "+to_string(census_n)+") -- object_force-Falle!");
		if(env_u("CFD_F_LISTE",0u)>0u) print_error("CFD_F_LISTE ist im KUGELFALL nicht verdrahtet (alloc_f_liste wird dort nicht gerufen) -- der Schalter waere ein stiller No-Op mit 1-Element-F. Fall verdrahten oder Schalter weglassen.");
		lbm.alloc_facetten(FF, elibb_an_kugel&&!elibb_qmap.empty()?&elibb_qmap:nullptr, 0u, 0u); if(env_u("CFD_SGS_FDWAND",0u)>0u) print_warning("CFD_SGS_FDWAND ist im KUGELFALL NICHT verdrahtet -- Schalter wird ignoriert."); if(env_u("CFD_SGS_GDIAG",0u)>0u) print_warning("CFD_SGS_GDIAG ist im KUGELFALL NICHT verdrahtet (kein Mess-Enqueue, kein Bericht -- Pruefbefund B-5). Schalter wird ignoriert; Verdrahtung bei Bedarf nachziehen.");
	}
	lbm.run(0u, n_steps); // initialisieren ohne Zeitschritt
	// ★ Mitbewegte Waende pruefen. Bodenkontakt hier bewusst NICHT erwartet: die Kugel schwebt frei.
	// Bei CFD_KUGEL_MG=0 (statische Waende) oder CFD_KUGEL_FREE=1 (Freistrom statt Waenden) ist
	// n_ms == 0 die KORREKTE Erwartung -- dann wird der Audit uebersprungen statt falsch zu warnen.
	// Ein Audit, der nach einer bewussten Umschaltung Alarm schlaegt, wird beim zweiten Mal ignoriert.
	if(moving_ground && !free_stream) audit_bewegte_waende(lbm, Nx, Ny, Nz, dx, u_lat, "Kugelkanal", false);
	for(ulong step=0ull; step<n_steps; step+=(ulong)sample_every) {
		const ulong chunk = min((ulong)sample_every, n_steps-step);
		lbm.run(chunk, n_steps);
		// ★ EINMALIG nach dem ersten Rechen-Abschnitt: der Nachweis DURCH den Kernel. Nur er faengt
		// V1s Fehlerklasse (alle Flags korrekt, aber der Konsument sprang heraus) -- siehe die
		// Begruendung bei pruefe_wandwirksamkeit().
		// ★ Nachpruefer-Befund: dieser Aufruf stand OHNE den Guard, den (A) hat. Mit CFD_KUGEL_MG=0
		// steht die Wand bewusst still, es entsteht eine echte Grenzschicht -- und der Nachweis warnte
		// bei JEDEM solchen Lauf. Genau die Sorte Warnung, die man nach dem zweiten Mal ignoriert.
		if(step==0ull && moving_ground && !free_stream) pruefe_wandwirksamkeit(lbm, Nx, Ny, Nz, u_lat, "Kugelkanal");
		// Kein update_fields() mehr noetig: UPDATE_FIELDS ist eingeschaltet (defines.hpp), stream_collide
		// schreibt u und rho jeden Schritt selbst. Damit sieht der Druck-Auslass das aktuelle Innenfeld
		// statt eines bis zu CFD_SAMPLE_EVERY Schritte alten -- und die Slices zeigen den echten Zustand.
		// (Das explizite update_force_field() hier war redundant: enqueue_object_force ruft
		// enqueue_update_force_field intern, mit t_last_force_field-Guard.)
		const float3 F_lat = lbm.object_force(TYPE_S|TYPE_X);
		if(zb>0u) { // ★ KRAFT-ZBAND (Negativ-Kontrolle): sequenziell nach object_force (object_sum wiederverwendet)
			const float3 Fb = lbm.object_force_zband((uchar)(TYPE_S|TYPE_X), 0u, zb);
			const float3 Fr = lbm.object_force_zband((uchar)(TYPE_S|TYPE_X), zb, Nz);
			const double skala = fmax(fmax(fabs((double)F_lat.x), fabs((double)F_lat.z)), 1e-30); // Fz-Nulldurchgang: absolute Toleranz gegen max(|Fx|,|Fz|)
			zb_selftest_max = fmax(zb_selftest_max, fmax(fmax(fabs(((double)Fb.x+(double)Fr.x)-(double)F_lat.x), fabs(((double)Fb.y+(double)Fr.y)-(double)F_lat.y)), fabs(((double)Fb.z+(double)Fr.z)-(double)F_lat.z))/skala); // R1-N2
			zb_fx_band+=(double)units.si_F(Fb.x); zb_fz_band+=(double)units.si_F(Fb.z);
			zb_fx_rest+=(double)units.si_F(Fr.x); zb_fz_rest+=(double)units.si_F(Fr.z); zb_nn++;
		}
		ts.push_back((double)((float)(step+chunk)*dt));
		{	// ★ Gross-Audit M: Waechter wie im dd-Fall (NaN / 3x bitgleich / Explosion)
			static float Fxp=1e30f, Fzp=1e30f; static uint nfroz=0u;
			string grund="";
			if(!std::isfinite(F_lat.x)||!std::isfinite(F_lat.z)) grund="die Kraft ist keine Zahl mehr";
			else if(nfroz>=2u) grund="die Kraft steht seit drei Abtastungen BITGLEICH"; // B7: bewusst -- ein absichtlich laminarer Messarm wuerde hier fallen (dann Waechter-Env nachruesten)
			else if((double)((float)(step+chunk)*dt)>0.02&&(fabs((double)units.si_F(F_lat.x))>20.0*(double)q_inf*(double)A_nom||fabs((double)units.si_F(F_lat.z))>20.0*(double)q_inf*(double)A_nom)) grund="|Cd| oder |Cz| ueber 20 (Explosion)"; // R2: dritter Zweig wie dd/fahrzeug
			// LATENT (R2): static nfroz/Fxp ueberleben einen zweiten Fall-Aufruf im Prozess -- heute unerreichbar (ein Setup je Prozess).
			if(grund!="") { // R3: Teilreihe retten wie im fahrzeug-Fall
				std::ofstream fr(out_dir+"forces_abbruch.csv"); fr.precision(8); fr<<"time_s,Fx_N,Fy_N,Fz_N\n";
				const size_t nn=std::min(ts.size(), fx.size());
				for(size_t i2=0u;i2<nn;i2++) fr<<ts[i2]<<","<<fx[i2]<<","<<fy[i2]<<","<<fz[i2]<<"\n"; fr.close();
				print_error("Kugel-Lauf gekippt bei Schritt "+to_string(step+chunk)+": "+grund+". Teilreihe: forces_abbruch.csv");
			}
			nfroz = (F_lat.x==Fxp&&F_lat.z==Fzp) ? nfroz+1u : 0u; Fxp=F_lat.x; Fzp=F_lat.z;
		}
		// ★ Druck-Zeitmittel: je Kadenz-Sample im Mittelungsfenster die Druckprojektion summieren.
		// kraft_facetten ruft update_force_field erneut -- der t_last_force_field-Guard macht das
		// zum No-Op (R3-Pruefer), die Reibungs-Slots des Ergebnisses werden hier ignoriert
		// (Reibung bleibt exaktes Fenster-Delta am Laufende).
		if(env_u("CFD_FACETTEN",0u)>0u&&ts.back()>=(double)t_warmup) {
			fac_cd_i++;
			if((fac_cd_i-1ull)%(ulong)max(1u,env_u("CFD_FAC_CD_EVERY",1u))==0ull) {
				const std::vector<double> leer;
				if(fac_pn==0ull) lbm.flags.read_from_device(); // einmalig: TYPE_MS aus initialize() in den Host-Spiegel
				const FacKraft FS = kraft_facetten(lbm, Nx, Ny, Nz, (uchar)(TYPE_S|TYPE_X), 1ull, leer, false, true);
				fac_px+=FS.px; fac_py+=FS.py; fac_pz+=FS.pz; fac_pn++;
				if(!fac_csv.is_open()) { fac_csv.open(out_dir+"cd_facetten.csv"); fac_csv << "# Druck-Zeitreihe des projizierten Cd-Pfads (Reibung: exaktes Fenster-Delta im Endreport)\nt_si,cd_druck_x,cd_druck_z\n"; }
				fac_csv << ts.back() << "," << (double)units.si_F((float)FS.px)/((double)q_inf*(double)A_nom) << "," << (double)units.si_F((float)FS.pz)/((double)q_inf*(double)A_nom) << "\n" << std::flush;
			}
		}
		// ★ Cd-Pfad: Akkumulator-Snapshot beim ersten Sample im Mittelungsfenster
		if(env_u("CFD_FACETTEN",0u)>0u&&ts.back()>=(double)t_warmup&&fac_snap.empty()) { fac_snap_step=step+chunk;
			lbm.lbm_domain[0]->fac_tau.read_from_device();
			fac_snap.resize(3ull*lbm.lbm_domain[0]->fac_N);
			for(ulong i=0ull;i<lbm.lbm_domain[0]->fac_N;i++){ fac_snap[3ull*i]=(double)lbm.lbm_domain[0]->fac_tau[6ull*i+1ull]; fac_snap[3ull*i+1ull]=(double)lbm.lbm_domain[0]->fac_tau[6ull*i+2ull]; fac_snap[3ull*i+2ull]=(double)lbm.lbm_domain[0]->fac_tau[6ull*i+3ull]; } }
		fx.push_back((double)units.si_F(F_lat.x));
		fy.push_back((double)units.si_F(F_lat.y));
		fz.push_back((double)units.si_F(F_lat.z));
	}

	// ---------------------------------------------------------------- Auswertung
	std::vector<double> cd_w, cz_w;
	for(size_t i=0u; i<ts.size(); i++) if(ts[i]>=(double)t_warmup) {
		cd_w.push_back(fx[i]/((double)q_inf*(double)A_nom));
		cz_w.push_back(fz[i]/((double)q_inf*(double)A_nom));
	}
	// CSV mit PUNKT als Dezimaltrennzeichen -- das deutsche Komma hat schon einen Parser
	// still scheitern lassen, und stilles Scheitern ist der teuerste Fehler in diesem Projekt.
	{
		std::ofstream f(out_dir+"forces.csv");
		f << "time_s,Fx_N,Fy_N,Fz_N,Cd_nominal,Cz_nominal,Cd_effective,Cz_effective\n";
		f.precision(8);
		for(size_t i=0u; i<ts.size(); i++) {
			const double cd = fx[i]/((double)q_inf*(double)A_nom), cz = fz[i]/((double)q_inf*(double)A_nom);
			f << ts[i] << "," << fx[i] << "," << fy[i] << "," << fz[i] << ","
			  << cd << "," << cz << "," << cd/(double)A_eff_ratio << "," << cz/(double)A_eff_ratio << "\n";
		}
		f.close();
		print_info("CSV geschrieben: "+out_dir+"forces.csv ("+to_string((uint)ts.size())+" Zeilen)");
	}
	// ★ Hygiene E7a, 2026-08-15: hier stand `return` -- und lief damit in genau den Intel-Teardown-
	// Crash ("double free or corruption", rc=134 auf dem regulaeren Rueckweg), den der Kommentar am
	// Fallende begruendet. _exit(0) wie an allen Fallenden. (Der Nachpruefer hat eine zweite
	// Begruendung gestrichen, die ich hier faelschlich uebertragen hatte: der Kugelfall ist der
	// else-Zweig der Fallauswahl, nach einem return liefe KEIN weiterer Fall.)
	// ★ Komplett-Audit 2026-08-17 (Pruefer 2, MITTEL): das _exit hier verschluckte Dichteklemme,
	// K4-Neutralitaet, Facetten-Ist=Soll und Delta-m -- genau die No-Op-Detektoren eines kurzen
	// Smoke-Tests. Jetzt entfaellt nur die STATISTIK; alle Pruefpfade laufen immer.
	const bool stat_ok = cd_w.size()>=16u;
	if(!stat_ok) print_warning("Zu wenige Samples im Mittelungsfenster -- Cd-Statistik entfaellt, Pruefpfade laufen trotzdem.");
	print_info("---------------------------------------------------------------");
	{ ulong h=0ull; berichte_dichteklemme(lbm, "Gitter", h); dichteklemme_fazit(h); }
	if(stat_ok) {
	double mcd=0.0, mcz=0.0;
	for(size_t i=0u; i<cd_w.size(); i++) { mcd+=cd_w[i]; mcz+=cz_w[i]; }
	mcd/=(double)cd_w.size(); mcz/=(double)cz_w.size();
	double sd=0.0; for(size_t i=0u; i<cd_w.size(); i++) sd += (cd_w[i]-mcd)*(cd_w[i]-mcd);
	sd = sqrt(sd/(double)cd_w.size());
	print_info("Zeitmittel ab t = "+to_string(t_warmup,3u)+" s ueber "+to_string((uint)cd_w.size())+" Samples:");
	print_info("  Cd (nominale Flaeche)   = "+to_string((float)mcd,4u)+"     Cz = "+to_string((float)mcz,4u));
	print_info("  Cd (effektive Flaeche)  = "+to_string((float)(mcd/(double)A_eff_ratio),4u)
		+"     Cz = "+to_string((float)(mcz/(double)A_eff_ratio),4u));
	// Blocksensitivitaet: waechst der SEM mit der Blocklaenge weiter, ist die Autokorrelation
	// nicht aufgeloest und der schmeichelhafteste Wert waere eine Selbsttaeuschung.
	print_info("  Block-SEM von Cd (die ehrliche Zahl ist die bei WENIGEN Bloecken):");
	for(uint k : {4u, 8u, 16u, 32u}) {
		const double se = block_sem(cd_w, k);
		if(se>=0.0) print_info("      "+to_string(k)+" Bloecke: +- "+to_string((float)se,5u)+"   (2 sigma = "+to_string((float)(200.0*se/mcd),3u)+" % von Cd)");
	}
	print_info("  Populations-Sigma des Momentansignals: +- "+to_string((float)sd,5u)+"  -- KEIN Fehlerbalken, nur zum Vergleich");
	} // stat_ok
	if(zb>0u&&zb_nn>0ull) { // ★ KRAFT-ZBAND-Endreport (Kugel schwebt frei -> Band ~0 = Negativ-Kontrolle)
		print_info("KRAFT-ZBAND Kugel (unterste "+to_string(zb)+" Zellen = "+to_string((float)zb*dx*1000.0f,2u)+" mm; GITTERBAND -- zwischen DX-Sprossen nicht direkt vergleichbar), "+to_string(zb_nn)+" Samples (ALLE, inkl. Anlauf):");
		print_info("  Band-Mittel: Fx = "+to_string((float)(zb_fx_band/(double)zb_nn),6u)+" N, Fz = "+to_string((float)(zb_fz_band/(double)zb_nn),6u)+" N (Soll ~0 -- Negativ-Kontrolle)");
		print_info("  Rest-Mittel: Fx = "+to_string((float)(zb_fx_rest/(double)zb_nn),6u)+" N, Fz = "+to_string((float)(zb_fz_rest/(double)zb_nn),6u)+" N");
		print_info("  Selbsttest-Maximum |Band+Rest-Gesamt|/max(|Fx|,|Fz|): "+to_string((float)zb_selftest_max,9u)+" (Soll < 5e-5)");
	if(zb_selftest_max>=5e-5) print_warning("ZBAND-Selbsttest ueber 5e-5 -- Zerlegung nicht belastbar (float-Atomik-Marge ist 3,5x, das hier ist mehr).");
	}
	if(env_u("CFD_FACETTEN", 0u)==0u&&env_u("CFD_FAC_K4", 0u)>0u) { // K4: Neutralitaet des neuen Pfads im AUS-Arm
		const std::vector<double> leer;
		const FacKraft FK0 = kraft_facetten(lbm, Nx, Ny, Nz, (uchar)(TYPE_S|TYPE_X), 1ull, leer);
		const float3 Fo = lbm.object_force(TYPE_S|TYPE_X);
		print_info("K4 (AUS-Arm): kraft_facetten Fx = "+to_string((float)FK0.px,6u)+" vs object_force Fx = "+to_string(Fo.x,6u)
			+" (rel. Abw. "+to_string((float)(Fo.x!=0.0f?fabs(FK0.px/(double)Fo.x-1.0):0.0),8u)+", Soll < 1e-5)");
	}
	if(env_u("CFD_BODEN_EQ",0u)>0u) { // Pruefagent R2 N3: Wirkpfad-Nachweis auch am Kugel-Messarm (Iron Rule 3)
		lbm.lbm_domain[0]->rho_clamp_hits.read_from_device();
		const ulong bq=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[20];
		print_info("BODEN_EQ-Wirkpfad Kugel: "+to_string(bq)+" Band-Resets (t%100-Stichprobe).");
		if(bq==0ull) print_error("CFD_BODEN_EQ gesetzt, aber Kugel-Wirkpfad NULL -- lautloser No-Op.");
	}
	if(env_u("CFD_FACETTEN", 0u)>0u) { // ★ Stufe-2-Commit 3: Wirkpfad Ist=Soll + tau-Akkumulator an der Kugel
		lbm.lbm_domain[0]->rho_clamp_hits.read_from_device();
		const ulong wz=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[7], kl=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[8];
		const ulong sk=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[9], zu=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[11];
		const ulong soll=lbm.lbm_domain[0]->fac_N*(ulong)((n_steps+99ull)/100ull);
		print_info("Facetten-Wirkpfad Kugel: "+to_string(wz)+" (Soll "+to_string(soll)+"), tau-Klemme "+to_string(kl)+", u_t~0-Skips "+to_string(sk)+", ohne offenes Paar "+to_string(zu)
			+(env_u("CFD_FACETTEN",0u)>=3u?(", iMEM: u_s-Klemme/Gate "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[10])+", Skalar "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[12])
			+", LSQ-Rueckfall "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[65])+", ohneTang "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[13])+" (davon mit rohen Tangentialmomenten [27], NICHT ELIBB-heilbar -- Rang, s. B83: "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[27])+")"+", Rang2 "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[14])
			+", Rang0-BB "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[15])+", sn-Klemme/Gate "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[16])+", PEMA-utb "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[17])+", alpha>ut "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[18])+", APG-Klemme "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[19])+", ELIBB[67] "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[67])+", MLS[68] "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[68])+", Rueckfall-Buchung[69] "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[69])+", Quergate[64] "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[64])):string("")));
		{ const uint* H=lbm.lbm_domain[0]->rho_clamp_hits.data(); pruefe_rueckfall_buchung(H[69],H[10],H[13],H[15],H[16],H[64],LBM_Domain::s_fac_satgate,"Kugel"); pruefe_kraftpfad(H[70],H[69],H[7],H[9],H[17],H[71],LBM_Domain::s_fac_kraft,"Kugel"); pruefe_kaskade(H,"Kugel",LBM_Domain::s_fac_messnur>0u,LBM_Domain::s_fac_pinv>0u); bericht_zielerfuellung(H,(ulong)H[7],(ulong)H[9],(ulong)H[17],(ulong)H[69],"Kugel"); }
		if(env_u("CFD_FACETTEN",0u)>=3u) { double dm=0.0, nk=0.0;
			lbm.lbm_domain[0]->fac_tau.read_from_device(); // ★ Nachpruefer Stufe-3: Stale-Fix auch hier (Kanal-M-Fix war nicht nachgezogen)
			for(ulong i3=0ull;i3<lbm.lbm_domain[0]->fac_N;i3++){ dm+=(double)lbm.lbm_domain[0]->fac_tau[6ull*i3+4ull]; nk+=(double)lbm.lbm_domain[0]->fac_tau[6ull*i3+5ull]; }
			print_info("iMEM-Erhaltung Kugel: Delta-m = "+to_string((float)dm,6u)+", Normal-Rest = "+to_string((float)nk,6u)); }
		if(LBM_Domain::s_fac_elibb_pur) { // ★ Pruefbefund Messlogik-3 (2026-08-25): der Pur-Return sitzt VOR Slot 7 -- das alte Soll kannte den Pur-Modus nicht und print_error (=exit) toetete den projizierten Cd-Pfad. Pur-Soll: Slot 7 = 0, ELIBB-Wirkpfad (Slot 67) > 0.
			if(wz!=0ull) print_error("Pur-Arm: Slot 7 muesste 0 sein -- Return-Position verschoben?");
			else if((ulong)lbm.lbm_domain[0]->rho_clamp_hits[67]==0ull) print_error("Pur-Arm: ELIBB-Wirkpfad (Slot 67) ist NULL -- lautloser No-Op.");
			else print_info("Pur-Arm (Kugel): Slot 7 = 0 (konstruktiv), ELIBB-Wirkpfad "+to_string((ulong)lbm.lbm_domain[0]->rho_clamp_hits[67])+" -- Abnahme in Pur-Form bestanden.");
		} else if(wz!=(soll&0xFFFFFFFFull)) print_error("Facetten-Wirkpfad Ist != Soll an der Kugel."); // Soll mod 2^32 (Audit 1/3)
		lbm.lbm_domain[0]->fac_tau.read_from_device(); lbm.lbm_domain[0]->fac_tau_n.read_from_device();
		double stau=0.0; ulong ntau=0ull;
		for(ulong i2=0ull; i2<lbm.lbm_domain[0]->fac_N; i2++) if(lbm.lbm_domain[0]->fac_tau_n[i2]>0u) { stau+=(double)lbm.lbm_domain[0]->fac_tau[6ull*i2]/(double)lbm.lbm_domain[0]->fac_tau_n[i2]; ntau++; }
		if(ntau>0ull) print_info("Facetten-tau Kugel: "+to_string(ntau)+" Tauschzellen, mittleres tau_w = "+to_string((float)(stau/(double)ntau),9u));
		print_info("ACHTUNG: Cd oben enthaelt den Impulsaustausch-Reibungsanteil -- an getauschten Links ist er ein PHANTOM (AUDIT-Befund 1 verallgemeinert). Fuer A/B nur die VERSCHIEBUNG zwischen den Armen werten.");
		// ★ NEUER Cd-Pfad (K4/K5): Druck projiziert + Reibung exakt aus dem Fenster-Delta
		if(!fac_snap.empty()&&n_steps>fac_snap_step) {
			const FacKraft FKu = kraft_facetten(lbm, Nx, Ny, Nz, (uchar)(TYPE_S|TYPE_X), n_steps-fac_snap_step, fac_snap);
			// Cd-Normierung: dieselbe Kette wie die Zeitreihe (units.si_F, q_inf*A_nom).
			// ★ Druck = ZEITMITTEL ueber die Kadenz-Samples (E6); Reibung = exaktes Fenster-Delta.
			const double cd_druck_ende = (double)units.si_F((float)FKu.px)/((double)q_inf*(double)A_nom);
			const double cd_druck = fac_pn>0ull ? (double)units.si_F((float)(fac_px/(double)fac_pn))/((double)q_inf*(double)A_nom) : cd_druck_ende;
			const double cd_reib  = (double)units.si_F((float)FKu.rx)/((double)q_inf*(double)A_nom);
			print_info("Cd-Pfad Kugel: Cd_druck = "+to_string((float)cd_druck,4u)+" (Zeitmittel, "+to_string(fac_pn)+" Samples; Endwert "+to_string((float)cd_druck_ende,4u)+"), Cd_reibung = "+to_string((float)cd_reib,4u)
				+", Summe = "+to_string((float)(cd_druck+cd_reib),4u)+" (nominale Flaeche)");
			if(fac_csv.is_open()) print_info("CSV: "+out_dir+"cd_facetten.csv ("+to_string(fac_pn)+" Zeilen)");
			print_info("Cd-Pfad Kugel (LATTICE-Einheiten, Verschiebung zaehlt): Druck x = "+to_string((float)FKu.px,6u)
				+", Reibung x = "+to_string((float)FKu.rx,6u)+" | n_voll "+to_string(FKu.n_voll)+", projiziert "+to_string(FKu.n_proj)+", unklar "+to_string(FKu.n_unklar));
		}
	}
	print_info("---------------------------------------------------------------");

	// ★ FELD-HASH (2026-08-24). Wortgleich zu dem im Kanalfall (setup.cpp:1699), damit die
	// Frage entscheidbar wird, die den ganzen Determinismus-Punkt gattert: weicht die LOESUNG
	// zwischen zwei identischen Laeufen ab, oder nur die KRAFTMELDUNG? Die Kraefte kommen aus
	// einer Reduktion ueber Millionen Zellen; weicht nur sie ab, ist der Bitvergleich ueber
	// eine feste Reduktionsordnung wiederherstellbar. Weicht das u-Feld ab, ist es
	// Loesungsdivergenz und jeder A/B braucht dauerhaft Statistik.
	// Rein lesend, gegatet, Default aus.
	if(env_u("CFD_FELD_HASH", 0u)>0u) {
		lbm.u.read_from_device();
		ulong h=1469598103934665603ull;
		for(ulong i=0ull; i<3ull*lbm.get_N(); i++) {
			uint b; const float v=(i<lbm.get_N())?lbm.u.x[i%lbm.get_N()]:((i<2ull*lbm.get_N())?lbm.u.y[i%lbm.get_N()]:lbm.u.z[i%lbm.get_N()]);
			memcpy(&b, &v, 4);
			// ★ WORTWEISE, nicht byteweise -- exakt wie der Kanalfall (setup.cpp:1702). Das ist
			// KEIN lehrbuchtreues FNV-1a (das mischt byteweise), aber es ist die Variante, die im
			// Projekt schon als Regressionsanker steht. Meine erste Fassung hier war byteweise:
			// dasselbe Feld haette eine ANDERE Zahl ergeben, und jeder kuenftige Vergleich
			// Kanal gegen Kugel waere still falsch gewesen (Pruefbefund 6, 2026-08-24).
			h ^= (ulong)b; h *= 1099511628211ull;
		}
		print_info("FELD-HASH(u) = "+to_string(h));
	}

	// Der Intel-Runtime-Teardown laeuft beim regulaeren Rueckweg in ein "double free or
	// corruption (out)" (rc=134), nachdem alle Dateien geschrieben sind. Upstream hat denselben
	// Exit-Bug an anderer Stelle und musste den "sauberen" Fix einen Tag spaeter zurueckrudern.
	_exit(0);
}

// =============================================================================================
// FAHRZEUG (MR2), SINGLE-DOMAIN
//
// Referenz ist der OpenFOAM-13-Fall mr2v40H: Cd = 0.599, Cz = -1.301 (Fenster 1100-1200,
// nachgemessen 2026-08-07; die frueher im Code stehenden 0.59 / -1.27 waren veraltet).
//
// BEWUSST OHNE Lagrava-Latt-Kopplung. Der alte Baum fuhr Fein- und Grobgitter gekoppelt; das ist
// die komplizierteste Komponente und kommt als eigener, messbarer Schritt. Hier laeuft erst der
// Fall selbst, damit man sieht, was ohne sie herauskommt.
//
// SPEICHER: bei dx=4mm braeuchte die volle Domaene 33.6 GB (fi 38 B + rho 4 + u 12 + flags 1 +
// F 12 = 67 B/Zelle x 501 M) und passt NICHT in die 32 GB der B70. Nachgerechnet, nicht geschaetzt.
// ★ KORREKTUR 2026-08-08 (Pruefer-Befund): hier stand "Default ist deshalb dx=5mm" und "die
// F-Bounding-Box ist noch nicht portiert". Beides ueberholt -- der Default ist 4 mm, und die
// F-Bounding-Box wird 40 Zeilen weiter unten gesetzt (seit Commit 76c80be). Genau daran waere sonst
// jemand haengengeblieben, der die Speicherrechnung nachvollziehen will.
// =============================================================================================
static void main_setup_fahrzeug() {
	const float si_u      = 30.0f;
	const float si_rho    = 1.225f;
	// ★ Hygiene E4, 2026-08-15: hier stand 1,48e-5, waehrend fahrzeug_dd und fernfeld 1,51e-5
	// benutzen -- ein A/B fahrzeug gegen fahrzeug_dd ohne gesetztes CFD_NU verglich damit STILL
	// zwei Viskositaeten (Pruefer-Befund, zweifach bestaetigt). Jetzt ueberall die
	// OpenFOAM-Referenzviskositaet. Der alte Wert ist per CFD_NU=1.48e-5 jederzeit zurueckholbar.
	const float si_nu     = env_f("CFD_NU", 1.51e-5f);
	const float si_length = 4.4364f;   // Fahrzeuglaenge laut STL-Konvention des Projekts
	const float A_ref     = 1.85f;     // Projekt-Konvention; die STL misst 1.8597 (0.5 % groesser)
	const float dx        = 0.001f*env_f("CFD_DX", 4.0f);
	const float u_lat     = 0.075f;
	const float dt        = u_lat*dx/si_u;
	const float nu_lat    = si_nu*dt/(dx*dx);
	const float tau       = 3.0f*nu_lat + 0.5f;

	// Domaene physikalisch statt in Zellen, damit dx frei waehlbar bleibt. Die Masse stammen aus der
	// 4mm-Baseline des alten Baums (1665 x 621 x 485 Zellen) und sind hier in Meter umgerechnet.
	const float Lx = 6.660f, Ly = 2.484f, Lz = 1.940f;
	const float bow_from_xmin = 0.22984f; // Fahrzeugnase liegt so weit hinter dem Einlass
	const uint Nx = (uint)(Lx/dx + 0.5f), Ny = (uint)(Ly/dx + 0.5f), Nz = (uint)(Lz/dx + 0.5f);
	// Fahrzeuglage: Default -3 mm UNTER z=0 (OF13-Referenzlage 2026-08-20, Reifen eingesenkt wie snappy). Der alte Baum liess es
	// 16 mm schweben (4 Zellen bei 4 mm) -- ein rein numerischer Versatz ohne physikalische Entsprechung,
	// der den Unterbodenspalt kuenstlich vergroessert und damit genau die Groesse verfaelscht, um die es
	// beim Abtrieb geht. CFD_Z_OFFSET_MM stellt den alten Zustand her, falls man A/B fahren will.
	const float z_offset_cells = 0.001f*env_f("CFD_Z_OFFSET_MM", -3.0f)/dx; // OF13-Referenzlage -3 mm (Heiko 2026-08-20)

	LBM_Domain::s_sparse_tiles_on = env_on("CFD_SPARSE_TILES");
	if(LBM_Domain::s_sparse_tiles_on) {
		const uint T = max(1u, env_u("CFD_TILE", 8u)); // Kantenlaenge 0 gibt es nicht
		if(T!=8u && T!=16u && T!=32u && T!=64u) print_error("CFD_TILE muss 8, 16, 32 oder 64 sein (erhalten: "+to_string(T)+").");
		LBM_Domain::s_sparse_T = T;
	}

	// STL ZUERST lesen und platzieren -- die F-Bounding-Box muss feststehen, BEVOR der LBM-Konstruktor
	// F alloziert. Ohne das faellt F auf die volle Domaene: bei 4 mm rund 6 GB extra, und der Fall
	// passt nicht mehr in die 32 GB der B70. Der alte Baum machte denselben Reorder aus demselben Grund.
	Mesh* veh = read_stl(get_exe_path()+"../scenes/vehicle.stl");
	{
		const float3 bbox0 = veh->get_bounding_box_size();
		print_info("STL BBox roh: "+to_string(bbox0.x,4u)+" x "+to_string(bbox0.y,4u)+" x "+to_string(bbox0.z,4u));
		veh->scale((si_length/dx)/bbox0.x);
	}
	{
		const float3 bb = veh->get_bounding_box_size(), ctr = veh->get_bounding_box_center();
		veh->translate(float3(
			bow_from_xmin/dx + 0.5f*bb.x - ctr.x,   // Nase bei bow_from_xmin hinter dem Einlass
			0.5f*(float)(Ny-1u) - ctr.y,            // mittig in y
			z_offset_cells - (ctr.z - 0.5f*bb.z))); // Unterkante auf die Fahrbahn (z_offset_cells: Default -3 mm seit OF13-Referenzlage)
	}
	print_info("Fahrzeuglage Einzelgitter: Unterkante bei "+to_string(env_f("CFD_Z_OFFSET_MM",-3.0f),1u)+" mm relativ z=0 (OF13-Referenzlage; XL-3 B7).");
	{	// F-Box = Fahrzeug plus Rand. Der Rand muss die wandnahen Fluidzellen mitnehmen, auf die
		// update_force_field schreibt; 4 Zellen decken die Reichweite von load_f (hoechstens 2) sicher ab.
		const uint M = 4u;
		const uint x0=(uint)fmax(0.0f, veh->pmin.x-(float)M), x1=(uint)fmin((float)Nx-1.0f, veh->pmax.x+(float)M);
		const uint y0=(uint)fmax(0.0f, veh->pmin.y-(float)M), y1=(uint)fmin((float)Ny-1.0f, veh->pmax.y+(float)M);
		const uint z0=(uint)fmax(0.0f, veh->pmin.z-(float)M), z1=(uint)fmin((float)Nz-1.0f, veh->pmax.z+(float)M);
		LBM_Domain::set_force_bbox(x0, y0, z0, x1-x0+1u, y1-y0+1u, z1-z0+1u);
	}

	units.set_m_kg_s(si_length/dx, u_lat, 1.0f, si_length, si_u, si_rho);
	// ★ Die Daempfungszone gibt es in diesem Fall NICHT, und das wird GESAGT statt still geschluckt:
	// die Zone rampt am DOMAENENRAND, und hier sind y+/y- und z+ mitbewegte Waende -- eine Zone dort
	// verdickte kuenstlich die Wandgrenzschichten. Sinnvoll ist sie nur in fernfeld und fahrzeug_dd,
	// dort nur am groben Gitter. Ein still wirkungsloser Schalter waere genau die Fehlerklasse,
	// die dieses Projekt sonst jagt.
	if(env_u("CFD_SPONGE_N", 0u)>0u) print_warning("CFD_SPONGE_N ist gesetzt, wird in diesem Fall aber NICHT angewandt (die Zone rampt am Domaenenrand, dort stehen hier mitbewegte Waende). Nur fernfeld und fahrzeug_dd nutzen sie.");
	LBM_Domain::s_sponge_n = 0u; LBM_Domain::s_sponge_a = 3000.0f; LBM_Domain::s_sponge_wmin = 0.5f; LBM_Domain::s_sgs_wandfrei = env_u("CFD_SGS_WANDFREI", 0u)>0u; LBM_Domain::s_sgs_guo = env_u("CFD_SGS_GUO", 1u)>0u; LBM_Domain::s_sgs_diag = env_u("CFD_SGS_DIAG", 0u)>0u; LBM_Domain::s_sgs_diag_ab = (ulong)env_u("CFD_SGS_DIAG_AB", 0u); // alle drei, damit keine Instanz einen Wert der vorigen erbt
	// ★ Audit-Nacharbeit 6/10: Wandfunktions-Statiken an JEDER Konstruktorstelle setzen; der Schalter
	// gilt nur im Kanal -- hier wird er angesagt statt lautlos verschluckt.
	LBM_Domain::s_wandfunktion = false; LBM_Domain::s_wf_tau = 1.0f;
	if(env_u("CFD_WANDFUNKTION", 0u)>0u) print_warning("CFD_WANDFUNKTION wird in diesem Fall NICHT angewandt (nur kanal; Fahrzeug braucht erst Relativgeschwindigkeit und Facetten).");
	LBM_Domain::s_facetten = false; LBM_Domain::s_fac_imem = false; LBM_Domain::s_fac_ema = 0.0f; LBM_Domain::s_fac_pema = 0.0f; LBM_Domain::s_fac_satgate = false; LBM_Domain::s_fac_kraft = 0u; LBM_Domain::s_fac_kdiag = 0u; LBM_Domain::s_fac_nachbar = 0u; LBM_Domain::s_fac_messnur = 0u; LBM_Domain::s_sgs_fdwand = 0u; LBM_Domain::s_sgs_gdiag = 0u; LBM_Domain::s_fac_alpha = 0u; LBM_Domain::s_fac_lsq = env_u("CFD_FAC_LSQ", 0u)>0u; LBM_Domain::s_fac_quergate = env_u("CFD_FAC_QUERGATE", 0u)>0u; LBM_Domain::s_fac_elibb = env_u("CFD_FAC_ELIBB", 0u)>0u; LBM_Domain::s_fac_elibb_pur = env_u("CFD_FAC_ELIBB", 0u)==2u; LBM_Domain::s_fac_qmin = env_f("CFD_FAC_QMIN", 0.1f); LBM_Domain::s_fac_kappa = env_f("CFD_FAC_KAPPA", 0.4f); LBM_Domain::s_fac_utkorr = env_f("CFD_FAC_UTKORR", 1.0f); LBM_Domain::s_fac_qkappe = env_f("CFD_FAC_QKAPPE", 1.0f); LBM_Domain::s_fac_qdiag = env_u("CFD_FAC_QDIAG", 0u); LBM_Domain::s_sgs_guo = env_u("CFD_SGS_GUO", 1u)>0u; LBM_Domain::s_fac_apg = 0.0f; LBM_Domain::s_boden_eq_n = 0u; LBM_Domain::s_boden_eq_down = 0u; LBM_Domain::s_boden_eq_split = 0xFFFFFFFFu; LBM_Domain::s_boden_eq_abstand = 0u; LBM_Domain::s_einlass_eq_n = 0u; LBM_Domain::s_schale_alpha = 0.0f; LBM_Domain::s_fac_diagz = -1l; LBM_Domain::s_fac_tau = 1.0f; // C1b: Aktivierung folgt je Fall (fahrzeug/dd: Stufe 5)
	if(env_u("CFD_FACETTEN", 0u)>0u) print_warning("CFD_FACETTEN wird in diesem Fall noch NICHT angewandt (aktiv: kanal, kugel, facetten_test; Fahrzeug folgt mit Stufe 5).");
	if(getenv("CFD_SLICE_NEAR_STEPS")||getenv("CFD_VTK_JEDE")||getenv("CFD_VTK_BEHALTE")) print_warning("CFD_SLICE_NEAR_STEPS/CFD_VTK_JEDE/CFD_VTK_BEHALTE werden in diesem Fall NICHT angewandt (nur fahrzeug_dd; Kadenz-Umbau 27.08.).");
	if(env_u("CFD_FERN_FACETTEN", 0u)>0u) print_warning("CFD_FERN_FACETTEN wird im Einzelgitter-Fahrzeugfall NICHT angewandt (nur fahrzeug_dd -- P8; Ansage-Doktrin).");
	{ const char* n2f_[] = {"CFD_N2F_SCHALE","CFD_N2F_VOLUMEN","CFD_N2F_BAND","CFD_N2F_BAND_N","CFD_N2F_BAND_PROFIL","CFD_N2F_BAND_UNTERBODEN","CFD_N2F_BAND_WAKE","CFD_N2F_BAND_NURWAKE","CFD_N2F_BAND_WAKE_START","CFD_N2F_BAND_WAKE_START_X","CFD_N2F_BAND_WAKE_ABSTAND","CFD_N2F_PARITAET"}; for(const char* b : n2f_) if(getenv(b)) print_warning(string(b)+" ist gesetzt, wird aber NUR im fahrzeug_dd-Fall angewandt (P9c; die neun BAND-/WAKE-/PARITAET-Schalter fehlten bis 2026-08-22 in dieser Ansage -- Pruefagent-S1)."); } // Ansage-Doktrin
	{ const char* bq_[] = {"CFD_BODEN_EQ","CFD_BODEN_EQ_DOWN","CFD_BODEN_EQ_ABSTAND","CFD_FERN_BODEN_EQ","CFD_FERN_BODEN_EQ_DOWN","CFD_FERN_EINLASS_EQ","CFD_KOPPLUNG_ZEITINTERP","CFD_KOPPLUNG_GLATT"}; for(const char* b : bq_) if(getenv(b)) print_warning(string(b)+" ist gesetzt, wird im Einzelgitter-Fahrzeugfall aber NICHT angewandt (fahrzeug_dd; CFD_BODEN_EQ/ABSTAND auch kugel, CFD_FERN_EINLASS_EQ auch fernfeld -- XL-R2/R3)."); }
	LBM lbm(Nx, Ny, Nz, nu_lat);

	print_info("=================== Fahrzeug MR2, Single-Domain ===================");
	print_info("dx = "+to_string(dx*1000.0f,2u)+" mm, Gitter "+to_string(Nx)+" x "+to_string(Ny)+" x "+to_string(Nz)
		+" = "+to_string((ulong)Nx*Ny*Nz)+" Zellen ("+to_string((float)((ulong)Nx*Ny*Nz)/1e6f,1u)+" M)");
	print_info("Box "+to_string((float)Nx*dx,3u)+" x "+to_string((float)Ny*dx,3u)+" x "+to_string((float)Nz*dx,3u)+" m");
	print_info("U = "+to_string(si_u,1u)+" m/s, Re_L = "+to_string((uint)(si_u*si_length/si_nu))
		+", u_lat = "+to_string(u_lat,4u)+", tau = "+to_string(tau,5u));
	print_info("Ziel (OpenFOAM 13, mr2v40H): Cd = 0.599, Cz = -1.301");

	// ---------------------------------------------------------------- Fahrzeug voxelisieren
	print_info("Fahrzeug BBox im Gitter: X["+to_string(veh->pmin.x,1u)+","+to_string(veh->pmax.x,1u)
		+"] Y["+to_string(veh->pmin.y,1u)+","+to_string(veh->pmax.y,1u)
		+"] Z["+to_string(veh->pmin.z,1u)+","+to_string(veh->pmax.z,1u)+"]");
	lbm.voxelize_mesh_on_device(veh, TYPE_S|TYPE_X);
	lbm.flags.read_from_device();
	sat_shell_and_void_fill(lbm, veh, Nx, Ny, Nz); // derselbe Voxelizer wie beim Kugelfall

	// ---------------------------------------------------------------- Randbedingungen
	// ---------------------------------------------------------------- Kontaktflaeche
	// Fahrzeugzellen AUF der Bodenebene z=0 verlieren das TYPE_X-Bit und werden damit Teil der Strasse:
	// sie bleiben solid, laufen mit u_road mit, und object_force(TYPE_S|TYPE_X) summiert sie NICHT mehr mit.
	//
	// Warum das noetig ist, gemessen: mit dem Fahrzeug auf dem Boden liegen 2694 Zellen mit TYPE_S|TYPE_X
	// auf der Domaenen-RANDebene. Dort rechnet calculate_indices periodisch, load_f greift also ueber den
	// Rand auf die gegenueberliegende Ebene. Zusammen mit den null- und einzelligen Fluidspalten an den
	// vier Reifenaufstandsflaechen (2694/5142/3374 Schalenzellen in z=0/1/2) explodierte der Fall:
	// Fz erreichte bei 2.8 ms -11.4 Millionen N, bei 3.8 ms stand nan in der Reihe, danach fror das Feld
	// bei +343640 N ein. Das ist die auf die FP16C-Grenze gesaettigte DDF-Ablage dieser Zellen, keine Kraft.
	//
	// ABGRENZUNG zum alten Baum, korrigiert: der macht es ANDERS, nicht gleich. Er BEHAELT TYPE_X und
	// SETZT u_lat (../FluidX3D/src/setup.cpp:1909) -- die Zellen bleiben dort also Objekt und werden
	// weiter mitsummiert; er umgeht das Problem nur dadurch, dass das Fahrzeug 16 mm schwebt und die
	// Kontaktflaeche gar nicht existiert. Hier wird TYPE_X entfernt UND u_lat gesetzt.
	//
	// REIHENFOLGE IST TRAGEND: das muss NACH dem Void-Fill stehen. Der flutet auf `(flags & TYPE_X)==0`
	// und seine Box beginnt bei z=0; liefe diese Schleife davor, waeren die Latschzellen Flutungs-Saaten
	// und die Flutung koennte durch den Latsch ins Fahrzeuginnere laufen -- der Wagen waere hohl.
	//
	// WAS DIESE AENDERUNG NICHT TUT: sie stabilisiert nichts. TYPE_X wird device-seitig nur im Voxelizer
	// und in der Graphics-Faerbung gelesen; alle Solverkernel maskieren mit TYPE_BO. Sie ist ausserhalb
	// von object_force beweisbar wirkungsfrei -- die Divergenz hat eine andere Ursache (Lambda).
	//
	// Die Spalt-Begruendung oben (2694/5142/3374 in z=0/1/2) rechtfertigt diese Aenderung uebrigens NICHT:
	// sie gilt fuer z=1 und z=2 genauso, und die bleiben unangetastet. Sie gehoert zur Lambda-Frage.
	// Was diese Aenderung rechtfertigt, ist allein: z=0 ist die periodische Nahtebene und liegt buendig
	// mit der Fahrbahnwand.
	{
		ulong contact = 0ull;
		for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
			const ulong n = (ulong)x + (ulong)y*(ulong)Nx; // z = 0
			if((lbm.flags[n]&TYPE_X)!=0u) {
				lbm.flags[n] &= (uchar)~TYPE_X;
				// ★ u HIER setzen, nicht auf die Randschleife hoffen. Deren Waechter lautet
				// `(flags & (TYPE_S|TYPE_X)) != 0 -> continue`, und 0x01 & 0x41 = 0x01 ist ungleich null --
				// die Zellen werden dort also uebersprungen. Ohne diese Zeilen blieben sie bei u=0 stehen:
				// eine ruhende Insel mitten in einer mit u_road laufenden Fahrbahn, genau im Nullspalt.
				// Von einem unabhaengigen Pruefer gefunden; mein Kommentar behauptete vorher das Gegenteil.
				lbm.u.x[n] = u_lat; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
				contact++;
			}
		}
		print_info("Kontaktflaeche: "+to_string(contact)+" Fahrzeugzellen auf z=0 an die Strasse uebergeben (TYPE_X entfernt)");
	}

	// x- = Einlass, x+ = Auslass, ALLE y- und z-Flaechen = feste mitbewegte Waende (Heiko-Vorgabe).
	// ★ 2026-08-08: vorher standen y+-, z+ auf TYPE_E-Freistrom. Damit prägten FUENF Flaechen die
	// Geschwindigkeit auf, der Fall war massiv ueberbestimmt und es stroemte GAR NICHTS -- der
	// 250-ms-Slice war ueber die ganze Domaene gleichfoermig, die PNGs bei 150/200/250 ms sogar
	// byte-gleich gross. Der Kugelfall lief die ganze Zeit richtig, weil er mitbewegte Waende hat.
	// Jetzt sind beide Faelle gleich aufgebaut: nur Ein- und Auslass sind TYPE_E.
	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
		const ulong n = (ulong)x + ((ulong)y + (ulong)z*(ulong)Ny)*(ulong)Nx;
		if((lbm.flags[n]&(TYPE_S|TYPE_X))!=0u) continue;
		if(z==0u || z==Nz-1u || y==0u || y==Ny-1u) {       // Boden, Decke, Seiten: mitbewegte Wand
			lbm.flags[n] = TYPE_S; lbm.u.x[n] = u_lat; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
		} else if(x==0u || x==Nx-1u) {                     // Ein- und Auslass
			lbm.flags[n] = TYPE_E; lbm.u.x[n] = u_lat; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
		} else { lbm.u.x[n] = u_lat; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f; }
	}
	lbm.set_pressure_outlet_faces(env_u("CFD_PO_FACES", 2u), env_f("CFD_PO_RHO", 1.0f));
	lbm.finalize_sparse_tiles();

	// ---------------------------------------------------------------- Lauf
	// Laufzeit in DURCHSPUELUNGEN statt in Sekunden: eine Durchspuelung = Domaenenlaenge / Anstroemung.
	// Solange wir diagnostizieren, sind zwei der sinnvolle Default -- die erste ist Anlauf, ueber die
	// zweite wird gemittelt. In Sekunden waere derselbe Wert bei jeder Domaenengroesse etwas anderes.
	const float t_flush  = (float)Nx*dx/si_u;
	const float t_end    = env_f("CFD_T_END", 2.0f*t_flush);
	const float t_warmup = env_f("CFD_T_WARMUP", 1.0f*t_flush);
	print_info("Eine Durchspuelung = "+to_string(t_flush,4u)+" s; Default sind zwei davon.");
	const uint  sample_every = max(1u, env_u("CFD_SAMPLE_EVERY", 10u)); // 0 waere eine Endlosschleife -- hier klemmen, nicht im Helfer
	const float slice_dt = env_f("CFD_SLICE_DT", 0.0f); // 0 = keine Slices
	const bool slice_gpu = env_u("CFD_SLICE_GPU", 1u)>0u; // ★ Slice-Ebenen-Read 2026-08-26 (PRUEF-Arm nur im dd-Fall)
	if(slice_dt>0.0f) print_info(string("Slice-Transportweg (einzel): ")+(slice_gpu?"Ebenen-Gather (CFD_SLICE_GPU=1)":"Voll-Read-Altpfad (CFD_SLICE_GPU=0)")+"."); // Auditor-B B-3
	// ★ Pruefagent NIEDRIG-2 (Hauskonvention "Schalter ohne Wirkpfad meldet sich"):
	if(env_u("CFD_SLICE_PRUEF", 0u)>0u) print_warning("CFD_SLICE_PRUEF wirkt nur im dd-Fall -- im Einzelgitterfall ohne Wirkpfad.");
	bool slice_cp_ok = false; // ★ Slice-Ebenen-Read: Kopplungspuffer wird lazy beim ersten Ereignis angelegt
	const ulong n_steps  = (ulong)(t_end/dt + 0.5f);
	const string out_dir = get_exe_path()+"../export/"+(getenv("CFD_RUN_NAME")?string(getenv("CFD_RUN_NAME")):string("fahrzeug"))+"/";
	create_folder(out_dir);
	// ★ C1b Stufe 1: Facettenbau NACH Kontaktflaechen-Uebergabe UND Randsetzung (Revision Auflage 6),
	// wand_flag 0x41 -- Fahrbahn und Latsch sind per Konstruktion Ausschluss. =2: nur Diagnose.
	if(env_u("CFD_FACETTEN_DIAG", 0u)>0u) {
		const ulong census_vorher = [&]{ ulong c=0ull; for(ulong n=0ull;n<lbm.get_N();n++) if(lbm.flags[n]==(TYPE_S|TYPE_X)) c++; return c; }();
		baue_facetten(lbm, Nx, Ny, Nz, (uchar)(TYPE_S|TYPE_X), out_dir, "Fahrzeug");
		const ulong census_nachher = [&]{ ulong c=0ull; for(ulong n=0ull;n<lbm.get_N();n++) if(lbm.flags[n]==(TYPE_S|TYPE_X)) c++; return c; }();
		if(census_vorher!=census_nachher) print_error("Facettenbau hat den 0x41-Zellcensus veraendert ("+to_string(census_vorher)+" -> "+to_string(census_nachher)+") -- object_force-Falle!");
		print_info("Facetten: 0x41-Zellcensus unveraendert ("+to_string(census_vorher)+") -- object_force unberuehrt.");
		if(env_u("CFD_FACETTEN_DIAG", 0u)==2u) _exit(0);
	}
	sichere_lauf(out_dir, "fahrzeug"); // ★ Heiko 2026-08-09: Sicherung MIT JEDEM Lauf, in allen vier Faellen
	const float q_inf = 0.5f*si_rho*si_u*si_u;
	const uint y_mid = Ny/2u;
	print_info("Laufzeit "+to_string(t_end,3u)+" s = "+to_string(n_steps)+" Schritte, Mittelung ab "+to_string(t_warmup,3u)+" s");

	std::vector<double> ts, fx, fz;
	float slice_next = 0.0f;
	lbm.run(0u, n_steps);
	audit_bewegte_waende(lbm, Nx, Ny, Nz, dx, u_lat, "Fahrzeug, Einzelgitter", true);
	for(ulong step=0ull; step<n_steps; step+=(ulong)sample_every) {
		const ulong chunk = min((ulong)sample_every, n_steps-step);
		lbm.run(chunk, n_steps);
		// ★ EINMALIG nach dem ersten Rechen-Abschnitt: der Nachweis DURCH den Kernel. Nur er faengt
		// V1s Fehlerklasse (alle Flags korrekt, aber der Konsument sprang heraus) -- siehe die
		// Begruendung bei pruefe_wandwirksamkeit().
		if(step==0ull) pruefe_wandwirksamkeit(lbm, Nx, Ny, Nz, u_lat, "Fahrzeug, Einzelgitter");
		lbm.update_force_field();  // u/rho schreibt stream_collide selbst (UPDATE_FIELDS)
		const float3 F = lbm.object_force(TYPE_S|TYPE_X);
		const double t_si = (double)((float)(step+chunk)*dt);
		ts.push_back(t_si); fx.push_back((double)units.si_F(F.x)); fz.push_back((double)units.si_F(F.z));
		{	// ★ Gross-Audit M: NaN-/Einfrier-/Explosions-Waechter (dd-Muster) -- vorher lief eine
			// Divergenz hier stundenlang weiter, und ein Abbruch verlor SAEMTLICHE Samples.
			static double Fxp=1e300, Fzp=1e300; static uint nfroz=0u;
			const double q_A=(double)q_inf*A_ref;
			string grund="";
			if(!std::isfinite(fx.back())||!std::isfinite(fz.back())) grund="die Kraft ist keine Zahl mehr";
			else if(nfroz>=2u) grund="die Kraft steht seit drei Abtastungen BITGLEICH (Zahlenformat gesaettigt)";
			else if(t_si>0.02&&(fabs(fx.back())>20.0*q_A||fabs(fz.back())>20.0*q_A)) grund="|Cd| oder |Cz| ueber 20";
			if(grund!="") {
				std::ofstream fr(out_dir+"forces_abbruch.csv"); fr.precision(8); fr<<"time_s,Fx_N,Fz_N\n";
				for(size_t i2=0u;i2<ts.size();i2++) fr<<ts[i2]<<","<<fx[i2]<<","<<fz[i2]<<"\n"; fr.close();
				print_error("Lauf gekippt bei t = "+to_string((float)t_si,5u)+" s: "+grund+". Teilreihe: forces_abbruch.csv");
			}
			nfroz = (fx.back()==Fxp&&fz.back()==Fzp) ? nfroz+1u : 0u; Fxp=fx.back(); Fzp=fz.back();
		}
		if(slice_dt>0.0f && (float)t_si>=slice_next) {
			slice_next = (float)t_si + slice_dt;
			// ★ Slice-Ebenen-Read 2026-08-26: Kopplungspuffer beim ERSTEN Ereignis anlegen (braucht die
			// initialisierte Domaene; der Einzelgitterfall ruft alloc_coupling_planes sonst nirgends).
			// Nur fuer D=1 gebaut -- Mehr-Domaenen-Läufe nehmen den wortgleichen Altpfad.
			const bool gpu_ok = slice_gpu && lbm.get_D()==1u;
			if(gpu_ok && !slice_cp_ok) { lbm.alloc_coupling_planes((ulong)Nx*(ulong)Nz); slice_cp_ok = true; }
			if(gpu_ok) lbm.lese_yslice_in_host(y_mid);
			else { lbm.u.read_from_device(); lbm.flags.read_from_device(); }
			render_yslice(lbm, Nx, Ny, Nz, y_mid, si_u/u_lat, si_u, (int)((float)t_si*1000.0f+0.5f), out_dir, "einzel");
			print_info("[SLICE] t = "+to_string((float)t_si,3u)+" s");
		}
	}

	// ---------------------------------------------------------------- Auswertung
	{
		std::ofstream f(out_dir+"forces.csv"); f.precision(8);
		f << "time_s,Fx_N,Fz_N,Cd,Cz\n";
		for(size_t i=0u; i<ts.size(); i++)
			f << ts[i] << "," << fx[i] << "," << fz[i] << "," << fx[i]/((double)q_inf*A_ref) << "," << fz[i]/((double)q_inf*A_ref) << "\n";
		f.close();
		print_info("CSV: "+out_dir+"forces.csv ("+to_string((uint)ts.size())+" Zeilen)");
	}
	std::vector<double> cd, cz;
	for(size_t i=0u; i<ts.size(); i++) if(ts[i]>=(double)t_warmup) {
		cd.push_back(fx[i]/((double)q_inf*A_ref)); cz.push_back(fz[i]/((double)q_inf*A_ref));
	}
	const bool stat_ok = cd.size()>=16u; // ★ Audit 2/3: Dichteklemme lief hinter dem _exit nie bei Kurzlaeufen
	if(!stat_ok) print_warning("Zu wenige Samples -- Cd-Statistik entfaellt, Dichteklemme laeuft trotzdem.");
	print_info("---------------------------------------------------------------");
	{ ulong h=0ull; berichte_dichteklemme(lbm, "Gitter", h); dichteklemme_fazit(h); }
	if(stat_ok) {
	double mcd=0.0, mcz=0.0;
	for(size_t i=0u; i<cd.size(); i++) { mcd+=cd[i]; mcz+=cz[i]; }
	mcd/=(double)cd.size(); mcz/=(double)cz.size();
	print_info("Zeitmittel ab "+to_string(t_warmup,3u)+" s ueber "+to_string((uint)cd.size())+" Samples:");
	print_info("  Cd = "+to_string((float)mcd,4u)+"   (OF13: 0.599, Abweichung "+to_string((float)(100.0*(mcd/0.599-1.0)),1u)+" %)");
	print_info("  Cz = "+to_string((float)mcz,4u)+"   (OF13: -1.301, Abweichung "+to_string((float)(100.0*(mcz/-1.301-1.0)),1u)+" %)");
	print_info("  Block-SEM von Cd (ehrlich ist die Zahl bei WENIGEN Bloecken):");
	for(uint k : {4u, 8u, 16u}) { const double se=block_sem(cd,k); if(se>=0.0) print_info("      "+to_string(k)+" Bloecke: +- "+to_string((float)se,5u)); }
	} // stat_ok
	print_info("---------------------------------------------------------------");
	_exit(0);
}

// =============================================================================================
// FAHRZEUG MR2 -- DOPPEL-DOMAENE (grobes Fernfeld + feines Nahfeld)
//
// WARUM ES DIESEN FALL GIBT. Der Einzelgitter-Fahrzeugfall hat eine Versperrung von 38,4 %:
// das Fahrzeug fuellt ueber ein Drittel des Kanalquerschnitts. Windkanalpraxis liegt unter 5 %,
// die OpenFOAM-Referenz mr2v40H bei 1,93 %. Ein Cd aus 38,4 % Versperrung ist mit dieser
// Referenz nicht vergleichbar, egal wie sauber der Rest gerechnet ist -- die Maskell-Korrektur
// (ARC R&M 3400) laesst dort ein Cd zwischen 0,8 und 1,4 erwarten statt 0,599.
// ★ KORREKTUR 2026-08-08 (Pruefer-Befund): hier stand, ein Einzelgitter ueber die OpenFOAM-Box
// haette bei 4 mm "1500x750x500 = 562 Mio Zellen". Das ist die Zellzahl bei 16 mm. Bei 4 mm waeren
// es 6001x3001x2001 = 36 MILLIARDEN Zellen, also das 64-fache. Die Begruendung des Falls stand
// damit auf einer falschen Rechnung -- sie wird durch die richtige nur staerker.
//
// GEOMETRIE. Die Maße stammen aus dem V1-Fahrzeugfall, wo sie durchgerechnet waren; Weltkoordinaten
// nach V1-Konvention (Fahrzeugnase bei x = 0, Mittelebene y = 0, Fahrbahn z = 0). Details bei den
// Zahlen weiter unten.
//
// KOPPLUNG. Einweg, grob -> fein: das Fernfeld schreibt jede grobe Zeitschrittdauer die (rho,u)
// auf die fuenf Aussenflaechen der Nahfeld-Box (der Boden ist beiden gemeinsam), kubisch auf
// feine Aufloesung interpoliert. Das Nahfeld rechnet dazwischen `ratio` Schritte. Der Auslass
// des Nahfelds bleibt frei (Druckrand) -- sonst koennte der aufgeloeste Nachlauf nicht abstroemen.
// Details und die Liste des bewusst NICHT Portierten stehen bei den Kernels in kernel.cpp.
//
// WAS DIESER AUFBAU NICHT LEISTET, ausdruecklich:
//   * Die Rueckwirkung des Fahrzeugs auf das Fernfeld laeuft NUR ueber das grobe Gitter, in dem
//     das Fahrzeug ebenfalls voxelisiert ist -- nicht ueber die aufgeloeste Nahfeld-Loesung.
//   * Die Nahfeld-Raender liegen im GESTOERTEN Feld und werden als volles Dirichlet aus einer
//     16-mm-Loesung vorgegeben: 0,23 m vor der Nase, 0,33 m neben der Flanke, 0,68 m ueber dem Dach.
//     Der kurze Einlauf ist ABSICHT (Heiko) -- er wirkt der toten Stroemung in den unteren 5 bis
//     20 mm entgegen. Ein Pruefer hat ihn am 2026-08-08 als groessten konstruktiven Hebel auf Cd
//     benannt; das ist richtig gesehen und trotzdem so gewollt. Wer ihn verlaengern will, misst
//     es ueber CFD_NEAR_OFF_X gegen den jetzigen Stand -- er "korrigiert" ihn nicht stillschweigend.
//   * OpenFOAM hat auf Decke und Seiten zeroGradient, hier steht Freistrom-Dirichlet. Bei 2,74 %
//     Versperrung wirkt das wie eine etwas weichere geschlossene Kanalwand; ueber Maskell geschaetzt
//     unter 3 % auf Cd, eher zu hoch. Nicht gemessen.
//
// WAS DEN VERGLEICH MIT OPENFOAM VOR ALLEM BEGRENZT -- und das ist keine Randbedingung:
// mr2v40H rechnet STATIONAeR mit RANS/k-omega-SST (constant/turbulenceProperties, foamRun mit
// deltaT=1 als Pseudozeit). Hier laeuft instationaeres LBM mit Smagorinsky bei tau = 0,50003 und
// ohne Wandmodell. Der Unterschied RANS gegen aufgeloeste Instationaritaet plus fehlende
// Wandbehandlung ist bei einem Fahrzeug typisch 5 bis 15 % im Cd -- er dominiert alles andere
// auf dieser Liste. Eine Abweichung in dieser Groessenordnung ist also KEIN Hinweis auf einen
// Fehler im Aufbau. [selbst nachgelesen 2026-08-08]
// =============================================================================================
// ★ BASIS-WAECHTER (Heiko 28.08.2026, nach einem entwerteten Messvormittag). Ein ganzer
// A/B lief mit ELF fehlenden Schaltern, weil die Konfiguration aus einem 7 Tage alten Lauf
// rekonstruiert wurde, dessen Name "standard_final" lautete. Darunter die komplette
// Nah->Fern-Rueckkopplung und CFD_FAC_UTKORR -- gemessener Preis rund 12 Prozent Abtrieb.
// Der Code konnte das strukturell nicht bemerken: die vorhandene Pruefung warnt nur, wenn
// N2F-Unterschalter OHNE CFD_N2F_SCHALE gesetzt sind. Der umgekehrte Fall -- alle fehlen --
// war weder Fehler noch Warnung.
// Der Waechter FORDERT KEINE GLEICHHEIT, er RECHNET UM: aus der Einheit in basis/*.basis und
// dem CFD_DX des Laufs leitet er den Sollwert ab. Ein 8-mm-Lauf deklariert nur CFD_DX und
// bekommt SPONGE_N, KRAFT_ZBAND, WAKE_ABSTAND und WAKE_START_X vorgerechnet. Wo die
// Umrechnung nicht eindeutig ist, nennt er BEIDE Kandidaten und verlangt eine Wahl.
// Und er meldet FEHLENDE Schalter genauso hart wie abweichende -- das ist die Haelfte, die
// im entwerteten Lauf gefehlt hat.
#include <sstream>   // ★ Audit-Notiz: istringstream kam bisher nur transitiv ueber <regex>
extern char** environ; // ★ H3: fuer die Gegenrichtung (im Lauf gesetzt, in der Referenz unbekannt)
struct BasisZeile { string name, wert, einheit; };
static void pruefe_basis(const string& basisdatei, const float dx_lauf) {
	if(getenv("CFD_BASIS")!=nullptr&&string(getenv("CFD_BASIS"))=="aus") {
		print_warning("BASIS-WAECHTER ABGESCHALTET (CFD_BASIS=aus) -- dieser Lauf ist NICHT gegen die Baseline geprueft. Der Notausgang ist absichtlich laut.");
		return;
	}
	std::ifstream f(basisdatei);
	// ★ M7, bewusst entschieden: eine fehlende Referenz bricht ab. Ein dd-Lauf ohne
	// Konfigurationsnachweis ist genau das, was am 28.08. einen Vormittag gekostet hat.
	// Wer aus einer exportierten Code-Kopie startet, kopiert basis/ mit oder setzt CFD_BASIS=aus.
	if(!f) print_error("BASIS-WAECHTER: "+basisdatei+" nicht lesbar. Erzeugen mit werkzeuge/basis_aus_lauf.py <LAUF.txt> "+basisdatei+", oder CFD_BASIS=aus setzen (laut warnend).");
	float dx_ref=0.0f; std::vector<BasisZeile> B; string zeile;
	while(std::getline(f,zeile)) {
		if(zeile.empty()) continue;
		// ★ Audit-Notiz: find() traf JEDE Kommentarzeile im File, ein spaeterer Treffer
		// ueberschrieb den Kopfwert. Jetzt nur die erste Nennung.
		if(zeile[0]=='#') { if(dx_ref<=0.0f) { const size_t k=zeile.find("dx_ref:"); if(k!=string::npos) dx_ref=(float)atof(zeile.substr(k+7).c_str()); } continue; }
		std::istringstream is(zeile); BasisZeile b; if(!(is>>b.name>>b.wert>>b.einheit)) continue; B.push_back(b);
	}
	if(dx_ref<=0.0f) { print_error("BASIS-WAECHTER: kein 'dx_ref:' im Kopf von "+basisdatei+" -- Umrechnung unmoeglich."); return; }
	if(!(dx_lauf>0.0f)) { print_error("BASIS-WAECHTER: CFD_DX = "+to_string(dx_lauf,4u)+" ist nicht positiv -- die Umrechnung waere undefiniert (M5)."); return; }
	const double skal = (double)dx_ref/(double)dx_lauf; // Feingitter-Zellen: kleineres dx -> mehr Zellen
	// Deklarierte Abweichungen, Format CFD_A=1,CFD_B=2 -- OHNE Leerzeichen: der Serienlaeufer
	// spaltet sein env-Stueck unquotiert an Leerzeichen, ein Freitextgrund zerlegte die Zeile.
	// Der Grund gehoert als Kommentar in den Serienkopf, nicht ins env.
	std::unordered_map<string,string> erlaubt;
	if(const char* d=getenv("CFD_BASIS_ABWEICHUNG")) {
		string t(d), stueck; std::istringstream ds(t);
		while(std::getline(ds,stueck,',')) {
			while(!stueck.empty()&&(stueck.front()==' '||stueck.front()=='\t')) stueck.erase(stueck.begin()); // ★ Audit-Notiz: " CFD_B" traf nie
			while(!stueck.empty()&&(stueck.back()==' '||stueck.back()=='\t')) stueck.pop_back();
			if(stueck.empty()) continue;
			const size_t g=stueck.find('=');
			if(g==string::npos) { print_warning("CFD_BASIS_ABWEICHUNG: Stueck ohne '=' ignoriert: '"+stueck+"'"); continue; }
			erlaubt[stueck.substr(0,g)]=stueck.substr(g+1);
		}
	}
	std::vector<string> fehlt, weicht_ab; string vorschlag, zu_setzen; ulong n_geprueft=0ull;
	for(const BasisZeile& b : B) {
		if(b.einheit=="ausgabe") {
			// ★ Audit I2 29.08.: uebersprungen, WEIL Ausgabe keine Physik ist -- aber still.
			// Der Korrekturvermerk zu CFD_SLICE_DT in der Referenz suggerierte eine Absicherung,
			// die es hier nie gab (der einzige echte Schutz ist der harte Slice-Riegel).
			// Jetzt wenigstens sichtbar: Abweichung melden, ohne den Lauf anzuhalten.
			const char* ist_a = getenv(b.name.c_str());
			const string ist_s = (ist_a==nullptr) ? string("(ungesetzt)") : string(ist_a);
			if(ist_a==nullptr||(fabs(atof(ist_a)-atof(b.wert.c_str()))>=1e-9&&ist_s!=b.wert))
				print_info("  BASIS (Ausgabe, nicht erzwungen): "+b.name+" Referenz "+b.wert+", Lauf "+ist_s+".");
			continue; }
		if(b.name=="CFD_DX"||b.name=="CFD_CASE") continue;  // die Sprosse selbst bzw. der Fall
		n_geprueft++;
		if(b.einheit!="phys"&&b.einheit!="modus"&&b.einheit!="zellen_grob"&&b.einheit!="zellen_fein"
		   &&b.einheit!="zellen_grob_laenge"&&b.einheit!="index_grob")
			print_error("BASIS-WAECHTER: unbekannte Einheit '"+b.einheit+"' bei "+b.name+" -- sie fiele still auf 'Wert bleibt' zurueck (M6).");
		string soll=b.wert, hinweis;
		// ★ Audit I 29.08.: 'phys', 'modus' und 'zellen_grob' fielen still auf "Wert bleibt"
		// durch -- genau die Rueckfallklasse, die M6 schliessen sollte. Sie sind wirklich
		// sprossenunabhaengig, aber das gehoert hingeschrieben, nicht durchgereicht:
		//   phys        physikalische Groesse (m, s, m/s) -- von dx unberuehrt
		//   modus       Schalterstellung/Zaehlwert ohne Laengenbezug
		//   zellen_grob ANZAHL grober Zellen -- die Anzahl bleibt, ihre Laenge waechst mit dx_c
		//               (dx_c = dx_f*ratio); wer eine feste LAENGE will, nimmt zellen_grob_laenge.
		if(b.einheit=="zellen_fein"||b.einheit=="zellen_grob_laenge"||b.einheit=="index_grob") {
			const double roh = atof(b.wert.c_str())*skal;
			const long unten=(long)floor(roh), oben=(long)ceil(roh);
			soll = to_string((ulong)llround(roh));
			if(unten!=oben) hinweis = " (nicht eindeutig: "+to_string((ulong)unten)+" oder "+to_string((ulong)oben)+" -- Wahl deklarieren)";
		}
		const char* ist_c = getenv(b.name.c_str());
		if(ist_c==nullptr) {
			// ★ M4: ein bewusst weggelassener Schalter war bisher NICHT deklarierbar -- der
			// continue stand vor dem Lookup, einziger Ausweg war CFD_BASIS=aus (schaltet alles ab).
			// Sentinel "-" heisst "absichtlich ungesetzt".
			auto itf=erlaubt.find(b.name);
			if(itf!=erlaubt.end()&&itf->second=="-") continue;
			fehlt.push_back(b.name+" (Soll "+soll+", Einheit "+b.einheit+")"+hinweis);
			zu_setzen+=(zu_setzen.empty()?"":" ")+b.name+"="+soll; continue; }
		const string ist(ist_c);
		// Wertvergleich, nicht Stringvergleich -- sonst schlaegt "8" gegen "08" oder "1.5" gegen "1.50" an.
		const bool gleich = fabs(atof(ist.c_str())-atof(soll.c_str()))<1e-9 || ist==soll;
		if(gleich) continue;
		auto it=erlaubt.find(b.name);
		if(it!=erlaubt.end()&&(fabs(atof(it->second.c_str())-atof(ist.c_str()))<1e-9||it->second==ist)) continue;
		weicht_ab.push_back(b.name+": Soll "+soll+", Ist "+ist+" ("+b.einheit+")"+hinweis);
		vorschlag+=(vorschlag.empty()?"":",")+b.name+"="+ist;
	}
	// ★ H3: die Schleife lief nur ueber die BASIS und fragte getenv. Der Spiegelfall -- im Lauf
	// gesetzt, in der Basis unbekannt -- war unsichtbar, und der Waechter meldete trotzdem
	// "deckt sich mit der Baseline". Genau so ist ein Lauf mit CFD_FACETTEN_NORMQUELLE=1 als
	// deckungsgleich durchgegangen. Besonders heikel: CFD_RATIO ist die stille Voraussetzung
	// der ganzen Umrechnung (dx_c = dx_f*ratio).
	{	std::unordered_map<string,int> bekannt;
		for(const BasisZeile& b : B) bekannt[b.name]=1;
		ulong extra=0ull;
		for(char** e=environ; *e!=nullptr; e++) {
			const string ev(*e); const size_t g=ev.find('=');
			if(g==string::npos||ev.compare(0,4,"CFD_")!=0) continue;
			const string nm=ev.substr(0,g);
			if(bekannt.count(nm)||nm=="CFD_BASIS"||nm=="CFD_BASIS_ABWEICHUNG"||nm=="CFD_RUN_NAME") continue;
			print_warning("BASIS ZUSAETZLICH: "+nm+"="+ev.substr(g+1)+" -- steht nicht in der Referenz, wird also NICHT geprueft.");
			if(nm=="CFD_RATIO") print_error("CFD_RATIO ist gesetzt, steht aber nicht in der Referenz -- die Umrechnung zellen_grob/index_grob haengt an unveraendertem ratio (dx_c = dx_f*ratio). Referenz erneuern oder Schalter entfernen.");
			extra++;
		}
		if(extra>0ull) print_info("  "+to_string(extra)+" Schalter ausserhalb der Referenz (siehe Warnungen) -- die Entwarnung unten gilt NUR fuer die gefuehrten.");
	}
	print_info("BASIS-WAECHTER: "+basisdatei+", dx_ref "+to_string(dx_ref,2u)+" gegen Lauf-dx "+to_string(dx_lauf,2u)
		+", "+to_string((ulong)n_geprueft)+" von "+to_string((ulong)B.size())+" Schaltern geprueft (Rest: ausgabe/Fall/Sprosse), "+to_string((ulong)erlaubt.size())+" Abweichungen deklariert.");
	if(fehlt.empty()&&weicht_ab.empty()) { print_info("  Keine Abweichung unter den gefuehrten Schaltern (nach Umrechnung)."); return; }
	// ★ JE BEFUND EINE EIGENE ZEILE. Die erste Fassung packte alles in EINEN print_error --
	// der Konsolenumbruch zerschnitt die Namen und machte ausgerechnet die kopierbare
	// Vorschlagszeile unlesbar. Ein Waechter, dessen Meldung man nicht lesen kann, wird
	// umgangen; deshalb ist das kein Schoenheitsfehler.
	for(const string& x : fehlt)     print_warning("BASIS FEHLT:      "+x);
	for(const string& x : weicht_ab) print_warning("BASIS ABWEICHEND: "+x);
	// Zusaetzlich in eine Datei, weil die Vorschlagszeile beliebig lang werden kann und
	// zum Kopieren gedacht ist.
	{	const string bp = get_exe_path()+"../logs/basis_abweichung.txt";
		std::ofstream bf(bp);
		if(bf) { bf << "# Basis-Waechter, Abweichungen gegen " << basisdatei << "\n";
			for(const string& x : fehlt)     bf << "FEHLT:      " << x << "\n";
			for(const string& x : weicht_ab) bf << "ABWEICHEND: " << x << "\n";
			if(!zu_setzen.empty()) bf << "\n# ZU SETZEN (fehlende Schalter -- oder mit '-' als absichtlich ungesetzt deklarieren):\n" << zu_setzen << "\n";
			if(!vorschlag.empty()) bf << "\n# ZU DEKLARIEREN (abweichende Werte):\nCFD_BASIS_ABWEICHUNG=" << vorschlag << "\n";
			bf.close();
			print_info("  Vollstaendig und kopierbar in: "+bp); }
	}
	print_error("BASIS-WAECHTER: "+to_string((ulong)fehlt.size())+" Schalter FEHLEN, "+to_string((ulong)weicht_ab.size())
		+" weichen ab (Einzelheiten in den Zeilen darueber). Wenn das gewollt ist, im Serienkopf begruenden und CFD_BASIS_ABWEICHUNG setzen -- die fertige Zeile steht in logs/basis_abweichung.txt.");
}

static void main_setup_fahrzeug_dd() {
	pruefe_basis(get_exe_path()+"../basis/fahrzeug_dd.basis", env_f("CFD_DX", 4.0f)); // ★ VOR jedem teuren Schritt
	// ★ Audit J 29.08.: der Slice-Riegel stand erst NACH baue_facetten und dem ELIBB-Remesh --
	// bei 4 mm also nach ueber zehn Minuten Aufbau. Die Entscheidung haengt aber nur an zwei
	// getenv, also faellt sie hier. Entscheidungstabelle wortgleich zur Stelle weiter unten.
	{	const bool s_ = getenv("CFD_SLICE_NEAR_STEPS")!=nullptr, d_ = getenv("CFD_SLICE_DT")!=nullptr;
		const ulong ns_ = s_ ? (ulong)env_u("CFD_SLICE_NEAR_STEPS", 5000u) : (d_ ? 0ull : 5000ull);
		const float dt_ = (ns_==0ull&&d_) ? env_f("CFD_SLICE_DT", 0.010f) : 0.0f;
		if(ns_==0ull&&dt_<=0.0f) print_error("Dieser Lauf schriebe KEINE Slices (CFD_SLICE_DT=0 und CFD_SLICE_NEAR_STEPS ungesetzt/0). Slice-Ausgabe ist Pflicht -- sie kostet praktisch nichts und ohne sie ist der Lauf hinterher nicht ansehbar. CFD_SLICE_DT auf einen positiven Wert setzen (0.1 = alle 100 ms)."); }
	bool nahfeld_pinv = false; // ★ 04.09. (Kernel-Audit M2): PINV-Zustand der NAHFELD-Domaene -- dieselbe Falle wie nahfeld_satgate, das Static wird unten fuer lbm_c ueberschrieben
	bool nahfeld_satgate = false; // SATGATE-Zustand der NAHFELD-Domaene (Funktionsscope; gesetzt im Fein-Block, gelesen vom Rueckfall-Detektor am Ende)
	bool nahfeld_messnur = false; // ★ 03.09. (Pruefagent Pass 2): dito fuer MESSNUR -- der Slot-72-Leser am Ende braucht den NAHFELD-Zustand, die Statik ist dann fuer lbm_c genullt
	uint nahfeld_kraft = 0u; // ★ Pruefbefund B1 (30.08.): dito fuer den Kraftpfad-Modus -- das Static wird unten fuer lbm_c (Fernfeld, CFD_FERN_FACETTEN) ueberschrieben; ohne Sicherung prueft der Nahfeld-Waechter Modus 0 gegen Slot 70 > 0 und bricht mit exit(1) mitten im Abschlussbericht ab.
	// ---------------------------------------------------------------- Physik
	const float si_u      = 30.0f;
	const float si_rho    = 1.225f;
	const float si_nu     = env_f("CFD_NU", 1.51e-5f); // ★ Referenzwert aus mr2v40H/constant/transportProperties.
	                                                  // V1 rechnete mit 1.48e-5 und dokumentierte gleichzeitig 1.51e-5 --
	                                                  // ein Widerspruch, der nie aufgeloest wurde. Hier gilt die Referenz.
	const float si_length = 4.4364f;
	const float A_ref     = 1.85f;
	const float u_lat     = 0.075f;

	// ratio ist der einzige Regler, der ueber die Kosten entscheidet: die groben Zellen skalieren
	// mit 1/ratio^3 und die groben Schritte mit 1/ratio, das Fernfeld kostet also 1/ratio^4.
	// ★ KORREKTUR 2026-08-08 (Pruefer-Befund): hier stand "Default 8 (dx_c = 32 mm)" -- der Code hat 4.
	// Der Kommentar stammte aus meinem ersten Entwurf mit der OpenFOAM-Box und war nach der Umstellung
	// auf die V1-Masse stehengeblieben. Bei ratio = 8 ginge 12,2720/0,032 nicht einmal auf einem halben
	// Gitterpunkt auf. DEFAULT IST 4 (dx_c = 16 mm), und das ist die V1-Wahl.
	// Gemessen 2026-08-08: B70 4648 MLUPs, iGPU 594 MLUPs (12,8 %). Bei ratio = 4 kosten vier feine
	// Schritte 0,432 s und ein grober 0,343 s -- das Fernfeld braucht 79 % der feinen Zeit und
	// verschwindet gerade noch dahinter. CFD_RATIO=8 waere billiger, kostet aber Aufloesung an der
	// Grenzflaeche und ist der A/B dazu.
	const uint  ratio = max(2u, env_u("CFD_RATIO", 4u));
	// ★ dx = 0 waere eine Division durch null in dt_f und nu_lat, ohne Meldung. Untergrenze 0,1 mm.
	const float dx_f  = 0.001f*fmax(0.1f, env_f("CFD_DX", 4.0f));
	const float dx_c  = dx_f*(float)ratio;
	const float dt_f  = u_lat*dx_f/si_u;
	const float dt_c  = (float)ratio*dt_f;
	const float nu_lat_f = si_nu*dt_f/(dx_f*dx_f);
	const float nu_lat_c = si_nu*dt_c/(dx_c*dx_c);

	// ---------------------------------------------------------------- Fernfeld = OpenFOAM-Box
	// Die Maße stammen aus dem V1-Fahrzeugfall (phase7g), wo sie durchgerechnet waren. Sie stehen hier
	// PHYSIKALISCH statt in Zellen, damit dx frei bleibt; bei dx_f = 4 mm und ratio = 4 kommen die
	// V1-Zellzahlen exakt zurueck (nachgerechnet 2026-08-08):
	//   Fernfeld 12.2720 x 7.6640 x 8.8160 m / 16 mm = 768 x 480 x 552 = 203.5 M Zellen
	//   Nahfeld   6.6560 x 2.4800 x 1.9360 m /  4 mm = 1665 x 621 x 485 = 501.5 M Zellen
	//   Nahfeld-Fussabdruck grob: Ursprung (152, 162, 0), Ausdehnung (417, 156, 122)
	// WARUM GENAU DIESE GROESSEN. Sie balancieren die beiden Geraete aus. Gemessen am 2026-08-08:
	// B70 4648 MLUPs, iGPU 594 MLUPs. Vier feine Schritte kosten 0.432 s, ein grober 0.343 s -- das
	// Fernfeld braucht 79 % der feinen Zeit und verschwindet damit gerade noch dahinter. Ein groesseres
	// Fernfeld (etwa die volle OpenFOAM-Box bei 16 mm: 565 M Zellen) waere der Flaschenhals.
	// Die Versperrung betraegt damit 2.74 % gegen 1.93 % bei OpenFOAM -- Windkanalpraxis liegt unter
	// 5 %, und der Einzelgitter-Fall lag bei 38.4 %.
	// ★ `(uint)(L/dx + 1.5f)` ist KEIN Runden: 8.0f/0.128f ergibt in float 62.499996, +1.5 sind
	// 63.999996, und die Ganzzahl-Umwandlung schneidet auf 63 ab -- eine Zelle zu wenig, lautlos und
	// nur bei bestimmten dx. Am 2026-08-08 im Kleinlauf aufgefallen, daher floor(x+0.5).
	auto n_cells = [](const float len, const float dx) { return (uint)floor(len/dx + 0.5f) + 1u; };
	const float far_Lx  = env_f("CFD_FAR_LX",  12.2720f), far_Ly  = env_f("CFD_FAR_LY",  7.6640f), far_Lz  = env_f("CFD_FAR_LZ",  8.8160f);
	// ★★ AUF GANZE GROBE ZELLEN SCHNAPPEN, 2026-08-09. Die V1-Werte gehen bei dx_c = 16 mm glatt auf
	// (416 / 155 / 121 grobe Zellen), bei jeder anderen Aufloesung nicht: bei 18 mm werden daraus
	// 369,78 / 137,78 / 107,56. Die Deckungskonvention fein = (grob-1)*ratio + 1 verlangt aber, dass
	// Boxlaenge UND Versatz ganzzahlige Vielfache von dx_c sind -- sonst liegen die Entnahmeebenen
	// raeumlich auseinander. Genau das hat die Kopplungspruefung bei dx = 4,5 mm gefangen: fein
	// 6,43016 m gegen grob 6,42816 m, also 2 mm bzw. 0,44 feine Zellen. Der Lauf wurde zu Recht
	// verweigert. Mit dem Schnappen ist die Konfiguration AUFLOESUNGSUNABHAENGIG -- Voraussetzung
	// fuer jede Gitterstudie, und die brauchen wir ohnehin.
	auto auf_grobe_zelle = [&](const float L) { return dx_c*(float)max(1, (int)floor(L/dx_c + 0.5f)); };
	// ★ Heiko 2026-08-21: Near-Box-Einlass weiter vor die Nase ("80-100 mm"), damit das x--Interface
	// den Zulauf VOR dem Staugebiet abgreift. Variante B (Planungsagent): near_off_x um Delta KLEINER,
	// near_Lx um Delta GROESSER -- nur die Einlass-Ebene wandert, das Heck-Ende bleibt weltfest
	// (reines Schieben haette den Druckauslass 96 mm an die Rezirkulation gerueckt und den A/B
	// verquickt). Messarm-Wert 96 = Vielfaches von 16 UND 32 mm (beide Grobgitter). Default 0 = alt.
	const float near_vor_roh = 0.001f*env_f("CFD_NEAR_VOR_MM", 0.0f);
	if(!std::isfinite(near_vor_roh)) print_error("CFD_NEAR_VOR_MM ist keine Zahl."); // Pruefagent M2: NaN verpuffte sonst lautlos (NaN-Vergleiche schweigen), Cast waere UB
	const float near_vor = dx_c*(float)max(0, (int)floor(near_vor_roh/dx_c + 0.5f));
	if(near_vor_roh<0.0f) print_warning("CFD_NEAR_VOR_MM < 0 wird auf 0 geklemmt (Box-Verkuerzung ist kein Messarm).");
	else if(fabs(near_vor-near_vor_roh)>1e-6f) print_warning("CFD_NEAR_VOR_MM liegt nicht auf dem "+to_string(dx_c*1000.0f,0u)+"-mm-Raster -- gerundet auf "+to_string(near_vor*1000.0f,0u)+" mm.");
	const float near_Lx = auf_grobe_zelle(env_f("CFD_NEAR_LX",  6.6560f)) + near_vor;
	const float near_Ly = auf_grobe_zelle(env_f("CFD_NEAR_LY", 2.4800f));
	const float near_Lz = auf_grobe_zelle(env_f("CFD_NEAR_LZ", 1.9360f));
	// Weltkoordinaten nach V1-Konvention: die Fahrzeugnase liegt bei x = 0, der Einlass 0.6 Fahrzeug-
	// laengen davor (bei NEAR_VOR=0). Das ist BEWUSST kurz -- Heiko 2026-08-08: der geringe Einlaufweg wirkt der toten
	// Stroemung in den unteren 5 bis 20 mm und der dadurch stagnierenden Unterbodenstroemung entgegen.
	// Wer das fuer einen Fehler haelt und "korrigiert", macht den Unterboden wieder falsch.
	const float far_x0  = env_f("CFD_FAR_X0", -0.6f*si_length);       // -2.66184 m
	const float near_off_x = auf_grobe_zelle(env_f("CFD_NEAR_OFF_X", 2.4320f)) - near_vor; // ebenfalls auf ganze grobe Zellen; near_vor zieht die Einlass-Ebene vor
	if(near_off_x<dx_c) print_error("near_off_x < eine Grobzelle: die Near-Box ragte vor den Fernfeld-Einlass (CFD_NEAR_VOR_MM zu gross oder CFD_NEAR_OFF_X zu klein).");
	const float near_x0 = far_x0 + near_off_x;  // -0.22984 m bei NEAR_VOR=0 und dx_c = 16 mm, V1-Wert
	if(near_vor>0.0f) print_info("NEAR_VOR aktiv: Einlass-Interface "+to_string(near_vor*1000.0f,0u)+" mm weiter vor der Nase (jetzt "+to_string(-near_x0,3u)+" m), Heck-Ende weltfest; +"+to_string((uint)floor(near_vor/dx_f+0.5f))+" feine x-Schichten.");
	const float veh_x0  = 0.0f;                                      // Nase
	const float veh_x1  = veh_x0 + si_length;                        // Heck
	// Das Fahrzeug steht AUF der Fahrbahn (Heiko-Vorgabe). V1 liess es 16 mm schweben.
	// ★ HEIKO 2026-08-20: REFERENZLAGE = OF13 -- Reifen 3 mm in den Boden eingesenkt (STL z_min=-3
	// im mr2v40H-Fall, snappy-Kontaktflecken-Konvention). Default war 0 (Aufstand auf z=0); die
	// Einsenkung verbreitert die Latsch-Flecken wie in der Referenz. Vertraeglichkeit: Voxelbereich
	// z<0 wird von der z=0-Fahrbahnreihe absorbiert, handover_contact/Klasse-64/BODEN_EQ unveraendert.
	const float veh_z0  = 0.001f*env_f("CFD_Z_OFFSET_MM", -3.0f);
	if(getenv("CFD_Z_OFFSET_MM")==nullptr) print_info("Fahrzeuglage: OF13-Referenz -3 mm (Reifen eingesenkt; CFD_Z_OFFSET_MM uebersteuert).");

	const uint cNx = n_cells(far_Lx, dx_c), cNy = n_cells(far_Ly, dx_c), cNz = n_cells(far_Lz, dx_c);
	const uint cex = n_cells(near_Lx, dx_c), cez = n_cells(near_Lz, dx_c);
	// Deckungspunkt-Konvention: die Nahfeld-Ecke MUSS auf einem groben Gitterpunkt liegen und es muss
	// fein = (grob-1)*ratio+1 gelten. Sonst faellt kein grober Punkt auf einen feinen, und die Ebene
	// laege um einen Bruchteil einer Zelle verschoben -- ein Fehler, den weder eine Norm noch ein
	// Kraftverlauf als solchen zeigt.
	// y: die Box wird um die Mittelebene zentriert, NF_OY = (cNy-cey)/2. Damit das aufgeht, muss cey
	// dieselbe Paritaet wie cNy haben. Bei den V1-Werten stimmt das (480 und 156, beide gerade);
	// bei abweichendem dx wird cey um eins erhoeht statt die Symmetrie aufzugeben -- eine unsymmetrische
	// Nahfeld-Box waere ein stiller Fehler in genau der Groesse, die hier am empfindlichsten ist.
	uint cey = n_cells(near_Ly, dx_c);
	if(((cNy^cey)&1u)!=0u) cey++;
	const uint NF_OX = (uint)floor((near_x0-far_x0)/dx_c + 0.5f);
	const uint NF_OY = (cNy-cey)/2u;
	const uint NF_OZ = 0u; // Fahrbahn ist beiden Gittern gemeinsam
	const float far_y0  = -0.5f*(float)(cNy-1u)*dx_c;   // Mittelebene y=0 in der Mitte des Fernfelds
	const float near_y0 = far_y0 + (float)NF_OY*dx_c;
	const float near_z0 = 0.0f;

	const uint fNx = (cex-1u)*ratio + 1u, fNy = (cey-1u)*ratio + 1u, fNz = (cez-1u)*ratio + 1u;

	if(NF_OX+cex>cNx || NF_OY+cey>cNy || NF_OZ+cez>cNz) { print_error("Nahfeld ragt aus dem Fernfeld heraus."); _exit(1); }

	// ---------------------------------------------------------------- Geraete
	const vector<Device_Info>& devs = get_devices();
	Device_Info dev_fine = select_device_with_most_flops(devs);
	Device_Info dev_coarse = dev_fine;
	{	// Grobgitter auf das schnellste ANDERE Geraet, das keine CPU ist.
		float best = -1.0f;
		for(uint i=0u; i<(uint)devs.size(); i++)
			if(devs[i].id!=dev_fine.id && devs[i].is_gpu && devs[i].tflops>best) { best=devs[i].tflops; dev_coarse=devs[i]; }
	}
	if(getenv("CFD_DEV_FINE"))   dev_fine   = select_device_with_id(env_u("CFD_DEV_FINE",   0u), devs);
	if(getenv("CFD_DEV_COARSE")) dev_coarse = select_device_with_id(env_u("CFD_DEV_COARSE", 0u), devs);
	if(dev_coarse.id==dev_fine.id) print_warning("Nur EIN Geraet gefunden: beide Gitter teilen sich denselben Speicher. Das wird knapp.");

	print_info("=================== Fahrzeug MR2, Doppel-Domaene ===================");
	print_info("Fein  (Geraet "+to_string(dev_fine.id)+", "+dev_fine.name+"): "+to_string(fNx)+" x "+to_string(fNy)+" x "+to_string(fNz)
		+" @ "+to_string(dx_f*1000.0f,2u)+" mm = "+to_string((float)((ulong)fNx*fNy*fNz)/1e6f,1u)+" M Zellen");
	print_info("Grob  (Geraet "+to_string(dev_coarse.id)+", "+dev_coarse.name+"): "+to_string(cNx)+" x "+to_string(cNy)+" x "+to_string(cNz)
		+" @ "+to_string(dx_c*1000.0f,2u)+" mm = "+to_string((float)((ulong)cNx*cNy*cNz)/1e6f,1u)+" M Zellen");
	print_info("Fernfeld  x["+to_string(far_x0,3u)+";"+to_string(far_x0+(float)(cNx-1u)*dx_c,3u)+"] y["+to_string(far_y0,3u)+";"+to_string(far_y0+(float)(cNy-1u)*dx_c,3u)+"] z[0;"+to_string((float)(cNz-1u)*dx_c,3u)+"] m");
	print_info("Nahfeld   x["+to_string(near_x0,3u)+";"+to_string(near_x0+(float)(fNx-1u)*dx_f,3u)+"] y["+to_string(near_y0,3u)+";"+to_string(near_y0+(float)(fNy-1u)*dx_f,3u)+"] z[0;"+to_string((float)(fNz-1u)*dx_f,3u)+"] m");
	print_info("Nahfeld-Fussabdruck grob: Ursprung ("+to_string(NF_OX)+","+to_string(NF_OY)+","+to_string(NF_OZ)+"), Ausdehnung ("+to_string(cex)+","+to_string(cey)+","+to_string(cez)+")");
	{
		const float A_far = (float)(cNy-1u)*dx_c*(float)(cNz-1u)*dx_c;
		print_info("Versperrung im Fernfeld: A_ref/A_quer = "+to_string(A_ref,3u)+"/"+to_string(A_far,2u)+" = "+to_string(100.0f*A_ref/A_far,2u)+" %  (OpenFOAM mr2v40H: 1.93 %)");
		// Der kurze Einlauf ist ABSICHT (Heiko): er wirkt der toten Stroemung in den unteren 5 bis 20 mm
		// und der dadurch stagnierenden Unterbodenstroemung entgegen. Das ist keine Nachlaessigkeit
		// gegenueber OpenFOAM, sondern der Grund, warum die Kopplung ueberhaupt gebaut wurde.
		print_info("Einlauf vor der Nase "+to_string((veh_x0-far_x0)/si_length,2u)+" L (bewusst kurz), Nachlauf hinter dem Heck "
			+to_string((far_x0+(float)(cNx-1u)*dx_c-veh_x1)/si_length,2u)+" L  (OpenFOAM: 1.08 L / 3.33 L)");
		print_info("Nahfeld-Einlauf vor der Nase "+to_string((veh_x0-near_x0),3u)+" m, Nahfeld-Nachlauf hinter dem Heck "+to_string(near_x0+(float)(fNx-1u)*dx_f-veh_x1,3u)+" m (selbstdokumentierend, XL-R4/NEAR_VOR).");
	}
#ifdef TRT
	print_info("Kollisionsoperator: TRT (Lambda-Wandlage aktiv).");
#else
	print_info("Kollisionsoperator: SRT (TRT samt Lambda-Wandlage in defines.hpp auskommentiert -- gemessene Entscheidung 2026-08-09).");
#endif // TRT
	print_info("dt_fein = "+to_string(dt_f,8u)+" s, dt_grob = "+to_string(dt_c,7u)+" s, ratio = "+to_string(ratio));

	// ---------------------------------------------------------------- Netze platzieren
	// Zweimal gelesen, weil jede Domaene ihre eigenen Gitterkoordinaten hat.
	auto place = [&](Mesh* m, const float dx, const float ox, const float oy, const float oz) {
		const float3 bb0 = m->get_bounding_box_size();
		m->scale((si_length/dx)/bb0.x);
		const float3 bb = m->get_bounding_box_size(), ctr = m->get_bounding_box_center();
		m->translate(float3((veh_x0-ox)/dx + 0.5f*bb.x - ctr.x,   // Nase auf Weltposition
		                    (0.0f  -oy)/dx               - ctr.y, // Mittelebene auf y = 0
		                    (veh_z0-oz)/dx + 0.5f*bb.z - ctr.z)); // Unterkante auf die Fahrbahn
	};
	Mesh* veh_f = read_stl(get_exe_path()+"../scenes/vehicle.stl"); place(veh_f, dx_f, near_x0, near_y0, near_z0);
	Mesh* veh_c = read_stl(get_exe_path()+"../scenes/vehicle.stl"); place(veh_c, dx_c, far_x0,  far_y0,  0.0f);

	// ---------------------------------------------------------------- Feines Gitter bauen
	Units units_fine, units_coarse;
	units_fine  .set_m_kg_s(si_length/dx_f, u_lat, 1.0f, si_length, si_u, si_rho);
	units_coarse.set_m_kg_s(si_length/dx_c, u_lat, 1.0f, si_length, si_u, si_rho);
	units = units_fine; // Kraefte kommen aus dem Nahfeld

	LBM_Domain::s_sparse_tiles_on = env_on("CFD_SPARSE_TILES");
	if(LBM_Domain::s_sparse_tiles_on) {
		const uint T = max(1u, env_u("CFD_TILE", 8u));
		if(T!=8u && T!=16u && T!=32u && T!=64u) print_error("CFD_TILE muss 8, 16, 32 oder 64 sein.");
		LBM_Domain::s_sparse_T = T;
	}
	{	// F-Box VOR dem Konstruktor: allocate() legt F sonst ueber die volle Domaene.
		const uint M = 4u;
		const uint x0=(uint)fmax(0.0f, veh_f->pmin.x-(float)M), x1=(uint)fmin((float)fNx-1.0f, veh_f->pmax.x+(float)M);
		const uint y0=(uint)fmax(0.0f, veh_f->pmin.y-(float)M), y1=(uint)fmin((float)fNy-1.0f, veh_f->pmax.y+(float)M);
		const uint z0=(uint)fmax(0.0f, veh_f->pmin.z-(float)M), z1=(uint)fmin((float)fNz-1.0f, veh_f->pmax.z+(float)M);
		LBM_Domain::set_force_bbox(x0, y0, z0, x1-x0+1u, y1-y0+1u, z1-z0+1u);
	}
	// ★★ DAEMPFUNGSZONE AM NAHFELD: AUSDRUECKLICH AUS, und das ist eine harte Aussage.
	// Vorpruefung 2026-08-09, nachgerechnet aus der STL-Lage: zwischen der Einlassflaeche x- und der
	// Fahrzeugnase liegen nur 57,5 Zellen (230 mm) -- der Nahfeld-Einlauf ist bewusst kurz gehalten.
	// Eine 64-Zellen-Zone UEBERDECKTE die Nase um 7 Zellen, und die Staupunktstroemung davor liefe
	// durch Faktor 145 (x=50), 422 (x=40), 751 (x=32). Auch N=32 rettet nichts: dann bleiben 100 mm
	// vor der Nase, mitten im Staugebiet. Seitlich verengte N=64 die freie Breite auf 1,97 m bei
	// 1,84 m Fahrzeugbreite -- ein virtueller Kanal 68 mm neben der breitesten Stelle.
	// Dazu kommt das urspruengliche Argument: vier der fuenf Nahfeld-Raender werden von der Kopplung
	// GETRIEBEN, und die Uebergabe geschieht in der EINEN Randzellschicht. Bei N=64 haette diese Zelle
	// den Faktor 2906 -- die Kopplung wuerde genau an ihrer Eintrittsstelle verschmiert.
	// Es gibt also KEINE vertretbare Zonenbreite am Nahfeld. Deshalb hier hart auf null.
	LBM_Domain::s_sponge_n = 0u; LBM_Domain::s_sponge_a = 3000.0f; LBM_Domain::s_sponge_wmin = 0.5f; LBM_Domain::s_sgs_wandfrei = env_u("CFD_SGS_WANDFREI", 0u)>0u; LBM_Domain::s_sgs_guo = env_u("CFD_SGS_GUO", 1u)>0u; LBM_Domain::s_sgs_diag = env_u("CFD_SGS_DIAG", 0u)>0u; LBM_Domain::s_sgs_diag_ab = (ulong)env_u("CFD_SGS_DIAG_AB", 0u); // alle drei, damit keine Instanz einen Wert der vorigen erbt
	// ★ Audit-Nacharbeit 6/10: Wandfunktions-Statiken an JEDER Konstruktorstelle setzen; der Schalter
	// gilt nur im Kanal -- hier wird er angesagt statt lautlos verschluckt.
	LBM_Domain::s_wandfunktion = false; LBM_Domain::s_wf_tau = 1.0f;
	if(env_u("CFD_WANDFUNKTION", 0u)>0u) print_warning("CFD_WANDFUNKTION wird in diesem Fall NICHT angewandt (nur kanal; Fahrzeug braucht erst Relativgeschwindigkeit und Facetten).");
	// ★ STUFE 5 (Plan 2026-08-18): Facetten/iMEM im NAHFELD. Kugel-Muster; alle 9 Statiken.
	{ const uint fc = env_u("CFD_FACETTEN", 0u);
	  if(fc>4u) print_error("CFD_FACETTEN kennt nur 0..4 (1/2 Paartausch voll/Tausch, 3/4 iMEM voll/Nullziel).");
	  LBM_Domain::s_facetten = fc>0u; LBM_Domain::s_fac_imem = fc>=3u;
	  LBM_Domain::s_fac_ema = (fc>=3u) ? env_f("CFD_FAC_EMA", 0.0f) : 0.0f;
	  LBM_Domain::s_fac_pema = (fc>=3u) ? env_f("CFD_FAC_PEMA", 0.0f) : 0.0f;
	  LBM_Domain::s_fac_diagz = -1l;
	  if(getenv("CFD_FAC_DIAGZ")!=nullptr) print_warning("CFD_FAC_DIAGZ ist im dd-Fall NICHT verdrahtet -- Ketten-Diagnose nur im Kanal/Torus.");
	  LBM_Domain::s_fac_pinv = env_u("CFD_FAC_PINV", 0u); if(LBM_Domain::s_fac_pinv>0u&&env_u("CFD_FAC_LSQ",0u)>0u) print_error("CFD_FAC_PINV und CFD_FAC_LSQ schliessen sich aus -- PINV ersetzt denselben Zweig, LSQ waere still wirkungslos (der #elif faellt durch). Einen von beiden waehlen."); if(LBM_Domain::s_fac_pinv>0u) print_info("RANG-1-PSEUDOINVERSE (CFD_FAC_PINV, 04.09.2026): im gekoppelten Zweig ersetzt Moore-Penrose die achsenparallele Skalarleiter -- Division ueber die SPUR (groesster Eigenwert) statt ueber Gt11. Grund: fuer die ebene Voxelwand ist tr(Gt) exakt 1/3 und kippungsunabhaengig, waehrend Gt11 mit der Stroemungsrichtung gegen 0 laeuft und die Akzeptanzschwelle 1e-4 dann Verstaerkung bis 1e4 durchlaesst. Wirkpfad Slot 80; erwartet fallen Slot 10 UND Slot 16, weil der Eigenvektor Sn.v = 0 exakt erfuellt und damit keine Normalkompensation mehr erzeugt wird."); LBM_Domain::s_fac_idx_voll = env_u("CFD_FAC_IDX_VOLL", 0u); if(LBM_Domain::s_fac_idx_voll>0u) print_info("CFD_FAC_IDX_VOLL=1: fac_idx in der ALTEN Vollfeldform -- deklarierter A/B-Arm gegen die Bitmaske (03.09.). Die Ergebnisse MUESSEN bitgleich sein, unterscheiden darf sich nur der Speicher."); LBM_Domain::s_f_liste = env_u("CFD_F_LISTE", 0u); if(LBM_Domain::s_f_liste>0u) print_info("F-MARKERLISTE (CFD_F_LISTE, 03.09.2026, Befunde B78b/B80/B81): F wird nur fuer WANDsolidzellen alloziert -- 8 mm gemessen: Nahfeld 238 -> 14 MiB, Fernfeld 4 -> 0 MiB. ABGENOMMEN ueber alle drei Sprossen bitgleich (CPU 5/5, iGPU 5/5 und dreimal reproduziert, B70 8-mm-Fahrzeug 19/19), Slot 77 = 0. Der urspruengliche Defekt war NICHT die Liste, sondern die Reihenfolge: die JIT-Defines entstanden vor dem Setzen der Schalter (B81)."); if(LBM_Domain::s_f_liste>0u&&!f_nur_solid_an_setup()) print_error("CFD_F_LISTE braucht CFD_F_NUR_SOLID (Default an): der Kontrollarm CFD_F_NUR_SOLID=0 liest F an JEDER Fluidzelle, und dort gibt es unter der Markerliste keinen Speicherplatz mehr -- die Kombination waere still falsch."); LBM_Domain::s_fac_satgate = fc>=3u&&env_u("CFD_FAC_SATGATE", 0u)>0u; LBM_Domain::s_fac_kraft = fc>=3u ? min(2u, env_u("CFD_FAC_KRAFT", 0u)) : 0u; LBM_Domain::s_fac_kdiag = fc>=3u ? env_u("CFD_FAC_KDIAG", 0u) : 0u; if(fc<3u&&(env_u("CFD_FAC_NACHBAR",0u)>0u||env_u("CFD_FAC_KDIAG",0u)>0u)) print_error("CFD_FAC_NACHBAR/CFD_FAC_KDIAG brauchen CFD_FACETTEN=3 (iMEM) -- bei CFD_FACETTEN="+to_string((ulong)fc)+" wuerde der Schalter still auf 0 gesetzt (No-Op-Waechter 03.09.)."); LBM_Domain::s_fac_nachbar = fc>=3u ? env_u("CFD_FAC_NACHBAR", 0u) : 0u; LBM_Domain::s_fac_messnur = fc>=3u ? env_u("CFD_FAC_MESSNUR", 0u) : 0u; LBM_Domain::s_sgs_gdiag = fc>=1u ? env_u("CFD_SGS_GDIAG", 0u) : 0u; LBM_Domain::s_sgs_fdwand = fc>=1u ? env_u("CFD_SGS_FDWAND", 0u) : 0u; if(LBM_Domain::s_sgs_fdwand>0u) { print_info("SGS-GEISTERMODEN-FIX (CFD_SGS_FDWAND, 02.09.): an Facettenzellen kommt die SGS-Relaxationsrate aus |S|_FD des u-Felds (FD-Kernel je Schritt, ein Schritt Versatz) statt aus dem wandmodell-kontaminierten Pi-Tensor (B66/B69: Pi/FD 2,3-3,4). Wirkpfad Slot 76 (B70)."); if(env_u("CFD_FAC_MESSNUR",0u)>0u) print_warning("SGS_FDWAND + MESS-NUR: der Arm ist dann NICHT mehr reines Bounce-Back -- das Kollisions-w an Wandzellen kommt aus dem FD-Pfad (bewusste Kombination fuer BB+FDWAND-Messungen, aber nicht mit alten BB-Bezuegen bitvergleichbar)."); if(env_u("CFD_SGS_WANDFREI",0u)>0u) print_warning("SGS_FDWAND + SGS_WANDFREI: WANDFREI hat an Wandzellen VORRANG -- FDWAND ist dort wirkungslos (Slot 76 bleibt 0). Fuer den FDWAND-Arm WANDFREI abschalten."); } if(LBM_Domain::s_sgs_gdiag>0u) { sgs_gdiag_selbsttest(); print_info("g-DIAGNOSE (CFD_SGS_GDIAG, 31.08.): Messkernel ueber die Wandzellen -- |S|_FD (u-Feld, geistermodenfrei), |S|_Pi (fneq, wie Smagorinsky), D_WALE, D_Sigma, |Omega|. Physik unangetastet, Bericht am Laufende."); } if(LBM_Domain::s_fac_messnur>0u&&env_u("CFD_FAC_NACHBAR",0u)>0u) print_warning("MESS-NUR + NACHBAR: die Nachbarabtastung liegt hinter dem MESS-NUR-Ausstieg und ist WIRKUNGSLOS (Slots 72-74 bleiben 0)."); if(LBM_Domain::s_fac_messnur>0u&&env_u("CFD_FAC_KDIAG",0u)>0u) print_warning("MESS-NUR + KDIAG: die Klassen-Diagnostik wird nie akkumuliert -- die Tabelle am Laufende ist eine Nulltabelle."); if(LBM_Domain::s_fac_messnur>0u&&env_u("CFD_FAC_KRAFT",0u)>0u) print_error("MESS-NUR + KRAFT ist unsinnig (kein Wandmodell -> kein Residuum; Modus 2 stuerbe irrefuehrend am Kraftpfad-Pruefer). Kombination aufloesen."); if(LBM_Domain::s_fac_messnur>0u) print_warning("MESS-NUR (CFD_FAC_MESSNUR, 30.08.): der Kernel wendet KEIN Wandmodell an -- die Wand ist reines Bounce-Back. Facetten werden nur gebaut und gemessen, damit der Druckpfad (cd_facetten.csv) als Aepfel-mit-Aepfeln-Bezug zu einem Wandmodell-Arm dient. Der REIBUNGSanteil ist in diesem Arm konstruktiv 0; belastbar ist der Druckanteil (bei Cz 99,5 %). Slot 75 = Wirkpfad (2. Umzug, B70). Die ELIBB-Blende wird unter MESS-NUR seit B-4 ebenfalls uebersprungen -- der Arm ist exakt reines Bounce-Back."); if(LBM_Domain::s_fac_nachbar>0u) print_info("NACHBARABTASTUNG (CFD_FAC_NACHBAR, 30.08.): Wandmodell-Eingang (u_t, Wandabstand) aus der zweiten Fluidzelle entlang der Normale statt aus der Wandzelle -- Stufenschatten-Fix. Slots 72 (angewandt) / 73 (kein Fluidnachbar) / 74 (Nachbar steht still; 2. Umzug 02.09., B70 -- 35-48 gehoeren SGS_DIAG ueber berechnete Indizes)."); if(LBM_Domain::s_fac_kraft>0u) print_info(string("iMEM-KRAFTPFAD (Weg F, 30.08.): Modus ")+to_string(LBM_Domain::s_fac_kraft)+(LBM_Domain::s_fac_kraft==1u?string(" -- Residuum R als Volumenkraft an RUECKFALLZELLEN (statt s=0); Slot 70, Soll == Slot 69."):string(" -- ALLE Facettenzellen per Kraft, Additivterm aus (Diskriminator gegen den Slip-Pfad); Slot 70."))); if(LBM_Domain::s_fac_kraft>0u) print_warning("KRAFTPFAD (Pruefpunkt 8, 30.08.): object_force/forces.csv (Impulsaustausch an Koerperzellen) sieht die Volumenkraft NICHT -- eine Guo-Kraft im Fluid hat keine Newton-3-Reaktion am Koerper. Der Reibungsanteil an Kraftzellen steht allein in der fac_tau-Buchung (cd_reib/cd_rest); object_force-Abgleiche (K4, Fx_far) weichen um genau den Kraftanteil ab."); LBM_Domain::s_fac_elibb = false; LBM_Domain::s_fac_qmin = 0.1f; LBM_Domain::s_fac_lsq = false; LBM_Domain::s_fac_quergate = false; // Statik-Symmetrie (H1)
	nahfeld_pinv = LBM_Domain::s_fac_pinv>0u; // ★ 04.09. (Kernel-Audit M2): s.o.
	nahfeld_satgate = LBM_Domain::s_fac_satgate; // ★ Pruefbefund 5a (27.08.): das Static wird unten fuer lbm_c ueberschrieben -- der Nahfeld-Detektor braucht DIESEN Zustand
	nahfeld_kraft = LBM_Domain::s_fac_kraft; // ★ Pruefbefund B1 (30.08.): gleiche Falle fuer den Kraftpfad
	nahfeld_messnur = LBM_Domain::s_fac_messnur>0u; // ★ 03.09.: gleiche Falle fuer das MESSNUR-Gating des Slot-72-Lesers
	  if(LBM_Domain::s_fac_satgate) print_info("iMEM-Saettigungs-Gate aktiv (a-strich): Budget-Riss -> BB-Rueckfall statt Klemme (Slots 10/16 = Rueckfaelle; seit Buchungsschluss 27.08. buchen Rueckfaelle P-only, Slot 69).");
	  if(LBM_Domain::s_fac_ema>0.0f) print_warning("CFD_FAC_EMA (Loesungs-Filterung) ist in J3 WIDERLEGT -- nur noch als A/B-Arm sinnvoll.");
	  if(LBM_Domain::s_fac_ema>0.0f&&LBM_Domain::s_fac_pema>0.0f) print_warning("CFD_FAC_EMA und CFD_FAC_PEMA GLEICHZEITIG: als Messarm wertlos (IR3-Audit).");
	  if(LBM_Domain::s_fac_pema>0.0f) print_info("iMEM-PEMA aktiv (Weg A): alpha = "+to_string(LBM_Domain::s_fac_pema,5u));
	  LBM_Domain::s_fac_alpha = (fc>=3u) ? env_u("CFD_FAC_ALPHA", 0u) : 0u; LBM_Domain::s_fac_lsq = env_u("CFD_FAC_LSQ", 0u)>0u; LBM_Domain::s_fac_quergate = env_u("CFD_FAC_QUERGATE", 0u)>0u; LBM_Domain::s_fac_elibb = env_u("CFD_FAC_ELIBB", 0u)>0u; LBM_Domain::s_fac_elibb_pur = env_u("CFD_FAC_ELIBB", 0u)==2u; LBM_Domain::s_fac_qmin = env_f("CFD_FAC_QMIN", 0.1f); LBM_Domain::s_fac_kappa = env_f("CFD_FAC_KAPPA", 0.4f); LBM_Domain::s_fac_utkorr = env_f("CFD_FAC_UTKORR", 1.0f); LBM_Domain::s_fac_qkappe = env_f("CFD_FAC_QKAPPE", 1.0f); LBM_Domain::s_fac_qdiag = env_u("CFD_FAC_QDIAG", 0u); LBM_Domain::s_sgs_guo = env_u("CFD_SGS_GUO", 1u)>0u;
	  if(LBM_Domain::s_fac_alpha>2u) print_error("CFD_FAC_ALPHA kennt nur 0..2 (1 = Massenkorrektur, 2 = + Momenten-Downdate). Die Stufe 3 (A2-Rueckfall) wurde am 2026-08-25 als beweisbar wirkungslos zurueckgenommen -- q_i ist fuer Einzellink-Facetten unter alpha identisch null.");
	  LBM_Domain::s_fac_apg = (fc>=3u) ? env_f("CFD_FAC_APG", 0.0f) : 0.0f;
	  if(LBM_Domain::s_fac_apg!=0.0f&&LBM_Domain::s_fac_pema>0.0f) print_error("CFD_FAC_APG + CFD_FAC_PEMA sind noch NICHT kombiniert (gefilterte Kette braucht eigenen APG-Zweig -- eigener Bauabschnitt).");
	  if(LBM_Domain::s_fac_apg!=0.0f) print_info("APG-Messarm aktiv: tw-Ziel um kappa*y_w*dp/ds korrigiert, kappa = "+to_string(LBM_Domain::s_fac_apg,4u)+" -- Slot 19 zaehlt beide Klemmen (0 / 2*tw).");
	  if(LBM_Domain::s_fac_alpha>0u) print_info(string("iMEM-alpha-Massenkorrektur Stufe ")+to_string(LBM_Domain::s_fac_alpha)+" -- Slot 18 zaehlt alpha>u_t.");
	  if(fc==3u&&(LBM_Domain::s_fac_alpha<2u||!LBM_Domain::s_fac_satgate)) print_warning("Arm 3 ohne SATGATE+ALPHA2 an gekruemmter Geometrie -- Kugel-J4-Lehre: nur als bewusster Messarm fahren.");
	  if(env_u("CFD_FERN_FACETTEN",0u)==3u&&(env_u("CFD_FAC_ALPHA",0u)<2u||env_u("CFD_FAC_SATGATE",0u)==0u)) print_warning("FERN_FACETTEN Arm 3 ohne SATGATE+ALPHA2 -- Kugel-J4-Lehre gilt am Treppenkoerper erst recht (P8-N2).");
	  if((fc>0u||env_u("CFD_FERN_FACETTEN",0u)>0u)&&env_f("CFD_FACETTEN_YWMIN",0.2f)>=0.187f) print_warning("dd: YWMIN-Default 0,2 schliesst 15-38 % der GENEIGTEN Flaechen (Scheiben!) still als K4 aus -- Slice-Agent 2026-08-20; CFD_FACETTEN_YWMIN=0.15 ist der deklarierte Messarm (Dachabloesung!).");
	  if(fc==4u&&LBM_Domain::s_fac_apg!=0.0f) print_warning("Arm 4 (Nullziel) + APG: tw/[0]-Akkumulator und y+-Report tragen APG-Korrektur, Ziel bleibt 0 -- reine Diagnose-Kombination (Gross-Audit N17).");
	  LBM_Domain::s_fac_tau = (fc==2u||fc==4u) ? 0.0f : 1.0f;
	  LBM_Domain::s_fac_budget = fmax(0.25f, fmin(4.0f, env_f("CFD_FAC_BUDGET", 1.0f)));       // 1a-B4t: Tangentialbudget-Skalar (geklemmt 0,25..4; Ansage unten)
	  LBM_Domain::s_fac_budget_sn = fmax(0.25f, fmin(4.0f, env_f("CFD_FAC_BUDGET_SN", 1.0f))); // 1a-Bsn: sn-Budget-Skalar
	  if(LBM_Domain::s_fac_budget!=1.0f) print_info("FACETTEN BUDGET (1a-B4t): Tangentialbudget x "+to_string(LBM_Domain::s_fac_budget,2u)+" (|s1| <= 2ut*k, |s2| <= ut*k). Die +-2ut-Budgets sind Design, nie geeicht (Planungsagent 2026-08-22). Erfolgskriterium: Slot-10-Anteil faellt UND cd_druck/cz_rest Richtung OF13 UND y+-Median nicht > +15 %.");
	  if(LBM_Domain::s_fac_budget_sn!=1.0f) print_info("FACETTEN BUDGET_SN (1a-Bsn): sn-Budget x "+to_string(LBM_Domain::s_fac_budget_sn,2u)+". Verschlechtert sich cd_druck > 2 %, ist der Arm verworfen (sn beruehrt den Druckpfad).");
	  LBM_Domain::s_boden_eq_n = env_u("CFD_BODEN_EQ", 0u); LBM_Domain::s_boden_eq_u = u_lat; LBM_Domain::s_boden_eq_abstand = env_u("CFD_BODEN_EQ_ABSTAND", 0u); LBM_Domain::s_einlass_eq_n = 0u; LBM_Domain::s_schale_alpha = 0.0f; // V1-Port NAHFELD; u_road folgt dem Setup (XL-B5); Abstand = Heiko-Reifenschutz; einlass_eq EXPLIZIT 0 fuers Feingitter (Pruefagent M1: Statik-Doktrin, nicht nur Initialisierer); Schalen-alpha EXPLIZIT 0 -- lbm_f traegt spaeter eine Extract-Liste, darf aber NIE blenden (P9c-Wirkpfad-Soll nah==0)
	  if(LBM_Domain::s_boden_eq_abstand>3u&&(LBM_Domain::s_boden_eq_n>0u||env_u("CFD_FERN_BODEN_EQ",0u)>0u)) print_warning("CFD_BODEN_EQ_ABSTAND > 3: der Scan kostet (2A+1)^2*(A+1) Flag-Reads je Bandzelle je Schritt -- stiller Perf-Fresser (XL-R2).");
	  if(LBM_Domain::s_boden_eq_n>3u) print_warning("CFD_BODEN_EQ > 3 verletzt die Heiko-Vorgabe (max 3, besser 2) -- Kraefteverfaelschung waechst mit N.");
	  LBM_Domain::s_boden_eq_down = env_u("CFD_BODEN_EQ_DOWN", 0u); // V1-x_split: ab Nase (Default 0 = unterm Wagen aus)
	  if(LBM_Domain::s_boden_eq_n==0u&&LBM_Domain::s_boden_eq_down>0u) print_warning("CFD_BODEN_EQ_DOWN ohne CFD_BODEN_EQ ist ein stiller No-Op (Guard haengt an N).");
	  LBM_Domain::s_boden_eq_split = (uint)fmax(0.0f, (veh_x0-near_x0)/dx_f);
	  if(LBM_Domain::s_boden_eq_n>0u) print_info("BODEN_EQ NAHFELD aktiv (V1-Port): z=1.."+to_string(LBM_Domain::s_boden_eq_n)+", ab Nase DOWN="+to_string(LBM_Domain::s_boden_eq_down)+" (split-Voxel "+to_string(LBM_Domain::s_boden_eq_split)+"; V1-Abweichung: DOWN=0 = AUS ab Nase), ABSTAND="+to_string(LBM_Domain::s_boden_eq_abstand)+" (reifennahe Aussparung; 0 = V1-Verhalten). ACHTUNG: Aufpraegung kontaminiert die Kraefte solid-adjazenter Zellen (XL-3 M3).");
	  if(fc>0u) print_info(string("Facettenpfad NAHFELD: ")+(fc==1u?"Paartausch voll":fc==2u?"Paartausch NUR TAUSCH":fc==3u?"iMEM voll":"iMEM NULLZIEL")
	  	+(env_u("CFD_FERN_FACETTEN",0u)==0u?string(" -- Fernfeld bleibt bewusst reines BB (16-mm-Treppenkoerper = Offen-Punkt 8).")
	  	:string(" -- Fernfeld faehrt seinen EIGENEN Facettenpfad (CFD_FERN_FACETTEN, P8)."))); }
	LBM lbm_f(uint3(fNx, fNy, fNz), nu_lat_f, dev_fine);

	// ---------------------------------------------------------------- Grobes Gitter bauen
	// Block-Tiling und F-Box sind read-once und stehen nach dem ersten Konstruktor wieder auf aus;
	// das Grobgitter bekommt hier ausdruecklich seine EIGENE F-Box und kein Tiling (bei 32 mm sind
	// zu wenige Kacheln voll solid, als dass es sich lohnte).
	{
		const uint M = 4u;
		const uint x0=(uint)fmax(0.0f, veh_c->pmin.x-(float)M), x1=(uint)fmin((float)cNx-1.0f, veh_c->pmax.x+(float)M);
		const uint y0=(uint)fmax(0.0f, veh_c->pmin.y-(float)M), y1=(uint)fmin((float)cNy-1.0f, veh_c->pmax.y+(float)M);
		const uint z0=(uint)fmax(0.0f, veh_c->pmin.z-(float)M), z1=(uint)fmin((float)cNz-1.0f, veh_c->pmax.z+(float)M);
		LBM_Domain::set_force_bbox(x0, y0, z0, x1-x0+1u, y1-y0+1u, z1-z0+1u);
	}
	// ★ Daempfungszone NUR am Fernfeld. Dort ist Platz: Abstand Rand -> naechste Kopplungs-Entnahmeebene
	// betraegt x- NF_OX (152 bei NEAR_VOR=0), x+ 199, y+- 162, z+ 430 Zellen. N=64 laesst ueberall >= 88 Zellen Luft.
	// Obergrenze N <= NF_OX-32 (32er-Reserve zur Entnahmeebene x-); N=32/64 sind komfortabel.
	// Bewusst NACH lbm_f gesetzt und ohne Selbstruecksetzung: lbm_f wird ZUERST konstruiert, ein
	// read-once haette die Zone also genau der falschen Domaene gegeben.
	LBM_Domain::s_sponge_n = env_u("CFD_SPONGE_N", 0u);
	LBM_Domain::s_sponge_a = env_f("CFD_SPONGE_A", 3000.0f);
	LBM_Domain::s_sponge_wmin = env_f("CFD_SPONGE_WMIN", 0.5f); LBM_Domain::s_sgs_wandfrei = env_u("CFD_SGS_WANDFREI", 0u)>0u; LBM_Domain::s_sgs_guo = env_u("CFD_SGS_GUO", 1u)>0u; LBM_Domain::s_sgs_diag = env_u("CFD_SGS_DIAG", 0u)>0u; LBM_Domain::s_sgs_diag_ab = (ulong)env_u("CFD_SGS_DIAG_AB", 0u);
	LBM_Domain::s_wandfunktion = false; LBM_Domain::s_wf_tau = 1.0f; LBM_Domain::s_fac_budget = 1.0f; LBM_Domain::s_fac_budget_sn = 1.0f; LBM_Domain::s_schale_paritaet = false; LBM_Domain::s_facetten = false; LBM_Domain::s_fac_imem = false; LBM_Domain::s_fac_ema = 0.0f; LBM_Domain::s_fac_pema = 0.0f; LBM_Domain::s_fac_lsq = env_u("CFD_FAC_LSQ", 0u)>0u; LBM_Domain::s_fac_quergate = env_u("CFD_FAC_QUERGATE", 0u)>0u; LBM_Domain::s_fac_elibb = env_u("CFD_FAC_ELIBB", 0u)>0u; LBM_Domain::s_fac_elibb_pur = env_u("CFD_FAC_ELIBB", 0u)==2u; LBM_Domain::s_fac_qmin = env_f("CFD_FAC_QMIN", 0.1f); LBM_Domain::s_fac_kappa = env_f("CFD_FAC_KAPPA", 0.4f); LBM_Domain::s_fac_utkorr = env_f("CFD_FAC_UTKORR", 1.0f); LBM_Domain::s_fac_qkappe = env_f("CFD_FAC_QKAPPE", 1.0f); LBM_Domain::s_fac_qdiag = env_u("CFD_FAC_QDIAG", 0u); LBM_Domain::s_sgs_guo = env_u("CFD_SGS_GUO", 1u)>0u; LBM_Domain::s_fac_satgate = false; LBM_Domain::s_fac_kraft = 0u; LBM_Domain::s_fac_kdiag = 0u; LBM_Domain::s_fac_nachbar = 0u; LBM_Domain::s_fac_messnur = 0u; LBM_Domain::s_sgs_fdwand = 0u; LBM_Domain::s_sgs_gdiag = 0u; LBM_Domain::s_fac_alpha = 0u; LBM_Domain::s_fac_apg = 0.0f; LBM_Domain::s_boden_eq_n = 0u; LBM_Domain::s_boden_eq_down = 0u; LBM_Domain::s_boden_eq_split = 0xFFFFFFFFu; LBM_Domain::s_boden_eq_abstand = 0u; LBM_Domain::s_einlass_eq_n = 0u; LBM_Domain::s_schale_alpha = 0.0f; LBM_Domain::s_fac_diagz = -1l; LBM_Domain::s_fac_tau = 1.0f; // Statik-Symmetrie VOLL (IR3-Abschluss-Loop)
	if(LBM_Domain::s_sponge_n>0u&&LBM_Domain::s_sponge_n+32u>NF_OX) print_error("CFD_SPONGE_N ueber "+to_string(NF_OX>=32u?NF_OX-32u:0u)+" kaeme im Fernfeld der Kopplungs-Entnahmeebene x- ("+to_string(NF_OX)+" Zellen) zu nahe (32er-Reserve; Grenze folgt NEAR_VOR).");
	LBM_Domain::s_boden_eq_n = env_u("CFD_FERN_BODEN_EQ", 0u); LBM_Domain::s_boden_eq_u = u_lat; LBM_Domain::s_boden_eq_abstand = env_u("CFD_BODEN_EQ_ABSTAND", 0u); // u_road Setup-treu (XL-B5); Abstand gilt fuer beide Felder
	LBM_Domain::s_boden_eq_down = env_u("CFD_FERN_BODEN_EQ_DOWN", 0u);
	if(LBM_Domain::s_boden_eq_n>3u) print_warning("CFD_FERN_BODEN_EQ > 3 verletzt die Heiko-Vorgabe (max 3, besser 2) -- am Grobgitter wiegt jede Zelle 4x (XL-R3).");
	if(LBM_Domain::s_boden_eq_n==0u&&LBM_Domain::s_boden_eq_down>0u) print_warning("CFD_FERN_BODEN_EQ_DOWN ohne CFD_FERN_BODEN_EQ ist ein stiller No-Op (Guard haengt an N; XL-R3).");
	LBM_Domain::s_boden_eq_split = (uint)fmax(0.0f, (veh_x0-far_x0)/dx_c);
	if(LBM_Domain::s_boden_eq_n>0u) print_info("BODEN_EQ FERNFELD aktiv (V1-Port): z=1.."+to_string(LBM_Domain::s_boden_eq_n)+" post-stream auf u_road-Equilibrium.");
	// ★ EINLASS_EQ (V1-Port apply_inlet_velocity): NUR FERNFELD -- deshalb hier ZWISCHEN lbm_f- und
	// lbm_c-Konstruktion gesetzt (read-once: lbm_f fror oben die EXPLIZITE 0 ein, Pruefagent M1). Nah-x- ist
	// Kopplungsebene und bleibt unberuehrt.
	LBM_Domain::s_einlass_eq_n = env_u("CFD_FERN_EINLASS_EQ", 0u); LBM_Domain::s_einlass_eq_u = u_lat;
	if(LBM_Domain::s_einlass_eq_n>3u) print_warning("CFD_FERN_EINLASS_EQ > 3 verletzt die V1-Vorgabe (V1: 3) -- die Clamp-Schicht frisst sich in die Domaene.");
	if(LBM_Domain::s_einlass_eq_n>0u) print_info("EINLASS_EQ FERNFELD aktiv (V1-Port apply_inlet_velocity): x=1.."+to_string(LBM_Domain::s_einlass_eq_n)+" post-stream auf u_inf-Equilibrium (rho lokal), nur FERNFELD; Nah-x- ist Kopplungsebene und bleibt unberuehrt.");
	// ★ P9c N2F-SCHALE (Heiko-Idee): near->far-Schalen-Rueckkopplung, alpha NUR FERNFELD -- deshalb
	// hier ZWISCHEN lbm_f- und lbm_c-Konstruktion (Muster CFD_FERN_EINLASS_EQ; lbm_f fror oben die
	// EXPLIZITE 0 ein). Default 0 = AUS = bitidentisch (kein alloc, kein Enqueue, keine Logzeile).
	// EINMAL gelesen (env-Read-Falle), die Konstante traegt bis in Listenbau/Zeitschleife.
	const float n2f_alpha = env_f("CFD_N2F_SCHALE", 0.0f);
	if(!(n2f_alpha>=0.0f&&n2f_alpha<=1.0f)) print_error("CFD_N2F_SCHALE muss in [0;1] liegen (Blendfaktor alpha; NaN faengt dieser Test mit)."); // NaN-Vergleiche schweigen -- Negativform (Pruefagent-M2-Klasse)
	// ★ PARITAETSARM (CFD_N2F_PARITAET=1, Pruefagent-M2, 2026-08-22). Setzt das KERNEL-alpha exakt
	// auf 0, laesst aber den ganzen Mechanismus laufen: Listenbau, Extract auf dem Nahfeld, Upload
	// ins Fernfeld, Kernel-Enqueue, Zaehler. Damit ist der FNEQ-Arm beweispflichtig -- er MUSS bei
	// a == 0 exakt degenerieren (u2 == u_lokal -> feq == feq_loc -> gespeichert == geladen).
	// Ein alpha=0 ueber CFD_N2F_SCHALE waere kein Ersatz: das schaltet den Arm host-seitig ganz ab.
	const uint n2f_paritaet = min(1u, env_u("CFD_N2F_PARITAET", 0u));
	LBM_Domain::s_schale_alpha = n2f_alpha; LBM_Domain::s_schale_paritaet = (n2f_paritaet>0u);
	if(n2f_paritaet>0u) print_warning("CFD_N2F_PARITAET=1: Kernel-alpha ist EXAKT 0 -- dieser Lauf traegt KEINE Rueckkopplungs-Physik. Er beweist nur, dass der Blend bei a = 0 ein exaktes No-Op ist (Slot 23 muss 0 sein).");
	// ★ GRADIENT-BLEND (Heiko-Slice-Befund c646253: uniforme alpha=0,5-Schale zeichnet ein sicht-
	// bares Kopplungsartefakt ins Far -> Lagen-Gradient innen 1 -> aussen 1/N, Heikos urspruengliche
	// Idee). Envs EINMAL gelesen (env-Read-Falle), Konstanten tragen bis Listenbau/Zeitschleife:
	//  CFD_N2F_SCHALE_LAGEN      N Lagen ab veh_c-BBox+o0 (Default 4); 0 = ALTVERHALTEN (uniforme
	//                            Doppellage bei rand~100mm, Gewicht 1,0) = Kontrollarm.
	//  CFD_N2F_SCHALE_FNEQ       1 = Nichtgleichgewichtsanteil im Blend erhalten (Kernel-modus Bit 0).
	//  CFD_N2F_SCHALE_XPLUS_SKAL Gewichtsskalierung [0;1] NUR fuer x+-Zellen (Wake-Messarm, Default 1,0).
	//  CFD_N2F_SCHALE_IDENT      1 = Debug: Blend als exaktes No-Op (Paritaetsbeweis der f-Paarung).
	const uint  n2f_lagen = env_u("CFD_N2F_SCHALE_LAGEN", 4u);
	// ★ DEFAULT-FLIP 2026-08-22 abends (Heiko: "FNEQ=1 ja, wird Standard"). Beleg: der EQ-Arm
	// aequilibriert a-UNABHAENGIG (fneq-Loeschung = ~75x Smagorinsky-Zusatzdissipation im Band,
	// Numerik-Pruefer) und erzeugte den binaeren Textur-Abdruck an der Bandgrenze (Arm 1:
	// +150 % Texturkante). FNEQ-A/B b8_breit_n16_fneq: Kante weg (+8 % statt +171 %), Lauf
	// stabil, Paarungsbeweis Slots 25/26 mit 0 Verletzungen bestanden. Der ehrliche Preis:
	// cd_druck-Effekt der Kopplung schrumpft von -0,19 auf -0,11 (der Rest war EQ-Artefakt).
	// Betrifft NUR aktive Kopplung -- Default-AUS (CFD_N2F_SCHALE=0) bleibt bitidentisch.
	// Alle EQ-Altlaeufe vor diesem Datum sind gegen FNEQ-Laeufe nicht direkt vergleichbar.
	const uint  n2f_fneq  = min(1u, env_u("CFD_N2F_SCHALE_FNEQ", 1u));
	const uint  n2f_ident = min(1u, env_u("CFD_N2F_SCHALE_IDENT", 0u));
	const uint  n2f_modus = n2f_ident ? 2u : n2f_fneq; // IDENT schlaegt FNEQ (Debug-Arm)
	const float n2f_xskal = env_f("CFD_N2F_SCHALE_XPLUS_SKAL", 1.0f);
	if(!(n2f_xskal>=0.0f&&n2f_xskal<=1.0f)) print_error("CFD_N2F_SCHALE_XPLUS_SKAL muss in [0;1] liegen (Gewichtsskalierung; NaN faengt dieser Test mit).");
	// ★ VOLUMEN-Blend (Heiko-Bild, 2026-08-21): CFD_N2F_VOLUMEN=1 ersetzt den SCHALEN-Listenbauer
	// durch einen VOLUMEN-Listenbauer -- Rot-Kern (Fahrzeug-BBox+100mm, Gewichtsfeld 1,0) + lineare
	// Rampe auf 0,0 an der Gruen-Box (Nahfeld-Fussabdruck -100mm je Achse); Schutzzone Gruen->
	// Fussabdruck bleibt zellfrei. Default 0 = Altverhalten (Schale) UNVERAENDERT. gewicht/modus-
	// Infrastruktur, schale_blend-/schale_extract-Kernel und Waechter werden WIEDERVERWENDET.
	const uint n2f_volumen = min(1u, env_u("CFD_N2F_VOLUMEN", 0u));
	// ★ BAND-Blend (Heiko 2026-08-21/22, BAUPLAN-KOPPLUNG.md §2): CFD_N2F_BAND=1 ersetzt den
	// Listenbauer durch ein Band, das VOM FAHRZEUG NACH AUSSEN reicht statt einer Schale um dessen
	// Bounding-Box. Der Unterschied ist keine Feinheit: die Fahrzeug-BBox misst ~2,4 Mio Grobzellen,
	// der voxelisierte Koerper nur ~1,05 Mio -- 55 % des BBox-Inhalts ist Fluid, und die heutige
	// Schale liegt an Motorhaube, A-Saeule und ueber dem Heck teils einen halben Meter von der
	// Karosserie weg. Der A/B Schale(LAGEN=N) gegen Band(N) isoliert genau diese Variable.
	//
	// Abstandsmass: Chebyshev zur Menge der Fahrzeug-Voxel des GROBGITTERS, per N Runden a drei
	// 1-D-Dilatationen (separabel, deterministisch, kein BFS). Lage = d-1, damit Lagen-Census,
	// Waechter-Aussenlage und Negations-Schwelle unveraendert weitertragen.
	//  CFD_N2F_BAND         1 = Band-Arm (Default 0). Schliesst CFD_N2F_VOLUMEN aus.
	//  CFD_N2F_BAND_N       Bandbreite in GROBZELLEN (Default 8). Auf der 8-mm-Sprosse ist eine
	//                       Grobzelle 32 mm, auf der 4-mm-Sprosse 16 mm -- gleiche Zellzahl heisst
	//                       doppelte Metertiefe. Das ist gewollt (der Mechanismus behandelt ein
	//                       Grobgitter-Artefakt), muss beim Sprossen-Vergleich aber bedacht werden.
	//  CFD_N2F_BAND_PROFIL  0 = linear (Default, bitgleiche Formel wie der Schalen-Gradient),
	//                       1 = cos^2 (eigener Messarm; Heiko: erst nach dem linearen Arm).
	//  CFD_N2F_BAND_UNTERBODEN  0 = Zellen im Unterbodenspalt weglassen (Vergleichsarm; Default 1).
	const uint  n2f_band     = min(1u, env_u("CFD_N2F_BAND", 0u));
	const uint  n2f_band_n   = min(253u, max(1u, env_u("CFD_N2F_BAND_N", 8u))); // Obergrenze 253: dt nutzt 254 (Wake-Kern) und 255 (Fahrzeug-Saat) als Marken -- ab N=254 schriebe die Dilatation eine Bandzelle als Kern (Pruefagent B9)
	const uint  n2f_band_prof= min(3u, env_u("CFD_N2F_BAND_PROFIL", 2u)); // 0 = linear, 1 = cos^2, 2 = Plateau+geometrisch, 3 = Heiko-Tabelle (STANDARD ab 2026-08-23)
	/* ★ PROFIL 3 -- Heikos urspruengliche Vorgabe, woertlich als Tabelle statt als gerechnete
	   Halbierung: 50 / 50 / 25 / 12,5 / 6,8 / 3,4 / 1,7 / 0 Prozent ABSOLUTE Staerke.
	   Hier stehen die RELATIVEN Gewichte (a = alpha * w); bei alpha = 0,5 ergeben sie exakt
	   die Prozentwerte oben. Der Unterschied zu Profil 2 sitzt in den letzten drei Stufen:
	   die reine Halbierung gaebe 6,25 / 3,125 / 1,5625. Die Tabelle wird bei WANDFREI
	   verschoben gezaehlt, genau wie Profil 2, und endet bei der Bandbreite. */
	static const float n2f_tab_heiko[7]={1.0f, 1.0f, 0.5f, 0.25f, 0.136f, 0.068f, 0.034f};
	const uint  n2f_band_ub  = min(1u, env_u("CFD_N2F_BAND_UNTERBODEN", 1u));
	// ★ WAKE-KASTEN (Heiko 2026-08-22). Statt Frontalprojektion + alles stromab ein schlichter
	// achsparalleler Kasten: y-Weite und z-Hoehe der Fahrzeug-BBox, x von einem waehlbaren Start
	// bis kurz vor den Nahfeld-Auslass. Vier Zahlen statt einer Silhouettenmaske -- und weil der
	// Kasten als ZWEITE SAATMENGE in dieselbe Distanztransformation geht, bekommt er denselben
	// Auslauf wie das Koerperband, ohne eine Zeile Sonderlogik.
	//  CFD_N2F_BAND_WAKE          1 = Kasten dazu (Default 0 = nur Koerperband).
	//  CFD_N2F_BAND_WAKE_START    0 = am DACHSCHEITEL (Default, Heiko-Vorschlag: erfasst Heckdeck
	//                             und Backlight mit, wo die Nah-Fern-Differenz am groessten ist),
	//                             1 = am HECK (konservativ, reiner Nachlauf) -- Ein-Variablen-A/B.
	//  CFD_N2F_BAND_WAKE_ABSTAND  Grobzellen, die vor dem Nahfeld-Auslass FREI bleiben (Default 16).
	//                             Grund ist NICHT Zirkularitaet -- x+ ist Druckauslass und wird
	//                             nicht getrieben --, sondern die Randkontamination der feinen
	//                             Loesung: die letzte Fussabdruck-Spalte bildet auf die feine
	//                             TYPE_E-Auslassebene ab; von dort wuerde man u_inf ins Totwasser
	//                             einspeisen, also das Gegenteil der Absicht.
	const uint n2f_wake      = min(1u, env_u("CFD_N2F_BAND_WAKE", 0u));
	// ★ NUR-WAKE-ARM (Heiko 2026-08-22): Rueckkopplung AUSSCHLIESSLICH aus dem Wake-Kasten, ohne
	// das Koerperband. Das Fahrzeug bleibt SPERRE (die Dilatation darf nicht hindurchlaufen und
	// seine Zellen duerfen nicht in die Liste), hoert aber auf, QUELLE zu sein. Dafuer braucht es
	// einen dritten dt-Wert: 253 = Sperre ohne Quelleigenschaft. 254 = Wake-Kern (Quelle),
	// 255 = Fahrzeug-Saat (Quelle). Ohne die Trennung waere "kein Koerperband" nicht ausdrueckbar:
	// dt=0 liesse die Dilatation durch den Wagen laufen, dt!=0 macht ihn zur Quelle.
	const uint n2f_nurwake   = min(1u, env_u("CFD_N2F_BAND_NURWAKE", 0u));
	// ★ WANDFREI (Arbeitsliste 1b, Heiko-Freigabe 2026-08-22 abends): die ersten k Lagen des
	// KOERPERbands werden nicht gelistet -- das Band zieht sich von der Wand zurueck. Messgrund:
	// die Wand-Schreibung verschlechtert die Fernfeld-Abloesung aktiv (11,2 % gegen 43,7 %
	// ungekoppelt gegen 35,6 % nah; das 4x4x4-Blockmittel traegt keine 32-mm-Abloeseschichten).
	// Auch wandnahe KASTENKERN-Zellen (Chebyshev <= k zum Fahrzeug) werden entfernt -- im w3-Lauf 30 % des Kastens. Default 0 = bitidentisch.
	const uint n2f_wandfrei  = env_u("CFD_N2F_BAND_WANDFREI", 0u);
	if(n2f_wandfrei>0u&&n2f_band==0u) print_warning("CFD_N2F_BAND_WANDFREI ohne CFD_N2F_BAND=1 ist ein stiller No-Op (Ansage-Doktrin).");
	if(n2f_band>0u&&n2f_wandfrei>=n2f_band_n) print_error("CFD_N2F_BAND_WANDFREI >= N: das ganze Koerperband waere leer -- dafuer gibt es CFD_N2F_BAND_NURWAKE=1 (Fahrzeug als reine Sperre, sauberere Semantik).");
	if(n2f_band>0u&&n2f_wandfrei>0u) print_info("N2F-BAND WANDFREI (1b): Koerperband-Lagen 1.."+to_string(n2f_wandfrei)+" werden NICHT gelistet -- das Band beginnt erst bei Lage "+to_string(n2f_wandfrei+1u)+". Wandnahe Kastenkern-Zellen werden MIT entfernt (Scan). Damit ist auch CFD_FERN_FACETTEN freigegeben (nur ohne NURWAKE (die Lage-1-Sperre entfaellt, s. dort).");
	if(n2f_nurwake>0u&&n2f_band==0u) print_warning("CFD_N2F_BAND_NURWAKE=1 ohne CFD_N2F_BAND=1 ist ein stiller No-Op -- der Schalter wirkt ausschliesslich im BAND-Listenbauer (Pruefagent-B-N6).");
	const uint n2f_wake_start= min(2u, env_u("CFD_N2F_BAND_WAKE_START", 0u)); // 0 = hoechster Punkt, 1 = Heck, 2 = Radstandmitte
	const uint n2f_wake_abst = env_u("CFD_N2F_BAND_WAKE_ABSTAND", 16u);
	if(n2f_band>0u&&n2f_volumen>0u) print_error("CFD_N2F_BAND=1 und CFD_N2F_VOLUMEN=1 schliessen sich aus -- beide ersetzen denselben Listenbauer. Genau einen Arm waehlen.");
	// Gewichtsprofil als eine Funktion, damit Census-Ausgabe und Listenbau garantiert dasselbe
	// rechnen (getrennte Formeln waren im Altbau schon einmal auseinandergelaufen).
	// ★ PROFIL 2: PLATEAU + GEOMETRISCH (Heiko 2026-08-22 nachmittags, aus den 4-mm-Diff-Slices).
	// Sein Befund: das Band ist "zu breit und zu schwach". Linear und cos^2 verteilen das Gewicht
	// ueber die ganze Tiefe; die gemessene Fehlerkurve (b8_kontrolle, Karosseriezone) faellt aber
	// nach 3 bis 4 Lagen steil ab. Die Vorgabe lautet: volle Staerke als PUFFERZONE ueber die
	// ersten Lagen, dann Halbierung je Lage, letzte Lage exakt 0.
	// Heikos Beispiel fuer N=8 bei alpha 0,5: 50/50/25/12,5/6,8/3,4/1,7/0 Prozent wirksames a.
	// Als Gewicht w (a = alpha*w): 1 / 1 / 0,5 / 0,25 / 0,125 / 0,0625 / 0,03125 / 0.
	// Seine drei letzten Zahlen liegen ~9 % ueber der reinen Halbierung -- Rundung; hier steht die
	// exakte Halbierung, damit das Profil eine geschlossene Form hat.
	// Vorlaeufer: V1 fuhr 2026-05-16 eine "Plateau-Rampe" nach demselben Gedanken (Phase 6C).
	const uint n2f_band_plateau = max(1u, min(n2f_band_n>1u?n2f_band_n-1u:1u, env_u("CFD_N2F_BAND_PLATEAU", 2u)));
	if(n2f_band>0u&&n2f_band_prof==2u&&n2f_band_n<2u) print_error("CFD_N2F_BAND_PROFIL=2 braucht N >= 2: bei N=1 ist die einzige Lage die Nulllage -- das ganze Band waere gewichtslos (B-P3).");
	if(n2f_band>0u&&n2f_band_prof==2u&&n2f_wandfrei+n2f_band_plateau>=n2f_band_n-1u) print_error("PROFIL 2: WANDFREI ("+to_string(n2f_wandfrei)+") + PLATEAU ("+to_string(n2f_band_plateau)+") >= N-1 ("+to_string(n2f_band_n-1u)+") -- es bliebe KEINE Rampenlage, die aeusserste gelistete Lage truege volles Gewicht und stuende als harte Kante am Bandrand (Pruefagent 0e01748, Punkt 5). N erhoehen oder Plateau/Wandfrei senken.");
	if(n2f_band>0u&&n2f_band_prof==2u&&n2f_wandfrei>0u) print_info("N2F-BAND PROFIL 2 + WANDFREI: das Plateau zaehlt AB Lage "+to_string(n2f_wandfrei+1u)+" (verschoben, nicht gefressen) -- wirksames Profil: Lagen 1.."+to_string(n2f_wandfrei)+" frei, dann "+to_string(n2f_band_plateau)+" Lagen voll, dann Halbierung, Lage "+to_string(n2f_band_n)+" = 0.");
	if(getenv("CFD_N2F_BAND_PLATEAU")&&n2f_band_prof!=2u) print_warning("CFD_N2F_BAND_PLATEAU ist gesetzt, wirkt aber nur bei CFD_N2F_BAND_PROFIL=2 (B3; Ansage-Doktrin).");
	if(n2f_band>0u&&n2f_band_prof==3u) { // die Tabelle reicht 7 Stufen weit -- das Band vielleicht nicht
		const uint stufen = (n2f_band_n>n2f_wandfrei+1u) ? n2f_band_n-1u-n2f_wandfrei : 0u;
		if(stufen<7u) print_warning("N2F-BAND PROFIL 3: die Tabelle hat 7 Stufen, das Band traegt bei N="+to_string(n2f_band_n)+" und WANDFREI="+to_string(n2f_wandfrei)+" aber nur "+to_string(stufen)+". Die Stufen "+to_string(stufen+1u)+"..7 FEUERN NICHT. Fuer alle sieben braucht es CFD_N2F_BAND_N="+to_string(n2f_wandfrei+8u)+".");
		if(stufen>7u) print_warning("N2F-BAND PROFIL 3: das Band traegt "+to_string(stufen)+" Stufen, die Tabelle nur 7 -- die Lagen dahinter sind gewichtslos.");
	}
	auto band_w = [&](const uint d) { // d = 1..N, gibt w in (0,1]; a = alpha*w
		if(n2f_band_prof==3u) {                                   // Heiko-Tabelle, woertlich
			if(d>=n2f_band_n) return 0.0f;                        // aeusserste Lage exakt 0 -- keine Kante
			const uint ds = (d>n2f_wandfrei) ? d-n2f_wandfrei : 1u;
			return ds<=7u ? n2f_tab_heiko[ds-1u] : 0.0f;          // hinter der Tabelle: aus
		}
		if(n2f_band_prof==2u) {                                   // Plateau + geometrisch
			// ★ 2026-08-22 spaet (Heiko): das Plateau zaehlt AB WANDFREI+1 -- der Puffer sitzt
			// HINTER der freien Wandzone statt von ihr gefressen zu werden. Synthese der beiden
			// Befunde: Wand-Schreibung unterdrueckt Abtrieb (Wandrueckzug, -0,27 Cz) UND das Band
			// war zu schwach (Plateau). wandfrei=0 -> exakt das bisherige Profil, bitidentisch.
			const uint ds = (d>n2f_wandfrei) ? d-n2f_wandfrei : 1u;   // verschobene Tiefe (Lagen <= wandfrei sind ohnehin nicht gelistet)
			if(d>=n2f_band_n) return 0.0f;                        // aeusserste Lage exakt 0 -- keine Kante
			if(ds<=n2f_band_plateau) return 1.0f;                 // Pufferzone volle Staerke, ab wandfrei+1
			return (float)pow(0.5, (double)(ds-n2f_band_plateau));
		}
		return n2f_band_prof==0u ? (float)(n2f_band_n+1u-d)/(float)n2f_band_n
		                         : (float)(cos(0.5*3.14159265358979*(double)(d-1u)/(double)n2f_band_n)*cos(0.5*3.14159265358979*(double)(d-1u)/(double)n2f_band_n));
	};
	if(n2f_band>0u) { // Iron Rule 8: die WIRKSAME Staerke je Lage ins Protokoll, nicht nur den Profilnamen
		string zl="N2F-BAND WIRKSAME STAERKE je Lage (a = alpha*w, in Prozent): ";
		for(uint d=1u; d<=n2f_band_n; d++) {
			if(d<=n2f_wandfrei) { zl+="L"+to_string(d)+"=frei "; continue; }
			zl+="L"+to_string(d)+"="+to_string(100.0f*n2f_alpha*band_w(d),1u)+" ";
		}
		print_info(zl);
	}
	if(n2f_alpha>0.0f&&n2f_paritaet>0u) print_warning("CFD_N2F_PARITAET=1: der Kernel bekommt alpha EXAKT 0, der Enqueue laeuft aber. Alle folgenden Ansagen mit \"a = ...\" nennen den GESETZTEN Wert -- WIRKSAM ist ueberall 0. Dieser Lauf ist ein BEWEISLAUF: seine Ergebnisdateien (forces.csv, interface_druck.csv, band_bilanz.csv) sind KEIN alpha-Arm und duerfen nicht als solcher ausgewertet werden.");
	if(n2f_alpha>0.0f) {
		print_info("N2F-SCHALE aktiv (P9c, Heiko): Fernfeld-Schale um die Fahrzeug-BBox wird post-stream mit dem Nahfeld-Blockmittel relaxiert, alpha = "+to_string(n2f_alpha,3u)+" (u_neu = (1-a)*u_far + a*u_near, rho bleibt lokal; a = alpha*gewicht je Zelle).");
		if(n2f_band>0u&&n2f_nurwake>0u) print_info("N2F-BAND NUR-WAKE (CFD_N2F_BAND_NURWAKE=1, Heiko 2026-08-22): das KOERPERBAND entfaellt. Das Fahrzeug bleibt Sperre fuer die Distanztransformation (dt = 253), ist aber KEINE Quelle mehr -- die Rueckkopplung geht ausschliesslich vom Wake-Kasten aus, mit derselben Rampe nach aussen. Nebenwirkung, die den Arm interessant macht: die karosserienahe Zellschicht wird nicht mehr ueberschrieben, damit liest update_force_field im Fernfeld wieder echte Rueckprall-Verteilungen und Fx_far ist als Kriterium zurueck (ausser dort, wo der Kasten selbst ans Fahrzeug stoesst).");
		if(n2f_band>0u&&n2f_nurwake==0u) print_warning("N2F-BAND mit Koerperband: Lage 1 ueberschreibt die karosserienahe Fernfeld-Schicht, aus der update_force_field liest -- Fx_far_N (forces.csv) und Fx_grob_N (band_bilanz.csv) sind in DIESEM Arm KEINE Kraefte, sondern lesen den Blend (Befund 79d6441). Nicht als Kriterium verwenden; sauber sind der Nahfeld-Facettenpfad und die Feld-CSVs.");
		if(n2f_band>0u) {
			print_info("N2F-BAND aktiv (CFD_N2F_BAND=1, Heiko 2026-08-22): BAND-Blend ersetzt den Schalen-Listenbauer -- "
				+to_string(n2f_band_n)+" Lagen VOM FAHRZEUG NACH AUSSEN (Chebyshev-Abstand zu den Fahrzeug-Voxeln des Grobgitters, nicht zur Bounding-Box), Profil "
				+(n2f_band_prof==0u?string("linear"):n2f_band_prof==1u?string("cos^2"):n2f_band_prof==3u?string("Heiko-Tabelle 50/50/25/12,5/6,8/3,4/1,7"):("Plateau("+to_string(n2f_band_plateau)+" Lagen voll)+geometrisch"))+" (w[1]="+to_string(band_w(1u),3u)+" ... w["+to_string(n2f_band_n)+"]="+to_string(band_w(n2f_band_n),3u)
				+"), Unterbodenspalt "+(n2f_band_ub>0u?"ENTHALTEN":"AUSGENOMMEN (Vergleichsarm)")+"; Modus "+to_string(n2f_modus)+(n2f_modus==2u?" (IDENT-Debug)":n2f_modus==1u?" (FNEQ)":" (EQ)")+".");
			if(n2f_wake>0u) print_info("N2F-BAND WAKE-KASTEN aktiv: achsparalleler Kasten in y/z-Ausdehnung der Fahrzeug-BBox, x von "+(n2f_wake_start==2u?string("RADSTANDMITTE"):n2f_wake_start==0u?string("HOECHSTER PUNKT"):string("HECK"))+" bis "+to_string(n2f_wake_abst)+" Grobzellen vor dem Nahfeld-Auslass; geht als ZWEITE SAATMENGE in dieselbe Distanztransformation, bekommt also denselben Auslauf wie das Koerperband.");
			if(n2f_modus==0u) print_warning("N2F-BAND: CFD_N2F_SCHALE_FNEQ=0 EXPLIZIT gesetzt -- der EQ-Arm ist seit 2026-08-22 NICHT mehr Standard (a-unabhaengige fneq-Loeschung, binaerer Textur-Abdruck). Nur noch fuer Vergleichslaeufe gegen EQ-Altbestand verwenden. Alte Empfehlung: CFD_N2F_SCHALE_FNEQ=1 -- Lage 1 liegt DIREKT an der Karosserie, dort traegt der Nichtgleichgewichtsanteil Scherinformation (der EQ-Arm verwirft sie je Blend).");
			if(getenv("CFD_N2F_SCHALE_LAGEN")||getenv("CFD_N2F_SCHALE_XPLUS")||getenv("CFD_N2F_SCHALE_XMINUS")||getenv("CFD_N2F_SCHALE_XPLUS_SKAL")) print_warning("CFD_N2F_SCHALE_LAGEN/XPLUS/XMINUS/XPLUS_SKAL gelten NUR im Schalen-Modus -- im BAND-Modus werden sie NICHT angewandt (Ansage-Doktrin).");
		}
		else if(n2f_volumen>0u) {
			print_info("N2F-VOLUMEN aktiv (CFD_N2F_VOLUMEN=1, Heiko-Bild): VOLUMEN-Blend ersetzt den Schalen-Listenbauer -- Rot-Kern = Fahrzeug-BBox+100mm (Gewichtsfeld 1,0, wirksam alpha), lineare Rampe (Chebyshev-artig je Achse) auf 0,0 an der Gruen-Box = Nahfeld-Fussabdruck-100mm; Schutzzone Gruen->Fussabdruck zellfrei; z- offen (Volumen endet unten am boden_eq-Band); Modus "+to_string(n2f_modus)+(n2f_modus==2u?" (IDENT-Debug)":n2f_modus==1u?" (FNEQ: Nichtgleichgewichtsanteil erhalten, seit 2026-08-22 Default)":" (EQ -- seit 2026-08-22 NICHT mehr Standard)")+".");
			if(n2f_modus==0u) print_info("N2F-VOLUMEN EMPFEHLUNG: CFD_N2F_SCHALE_FNEQ=1 setzen -- der Volumen-Blend greift bis in den Rot-Kern nahe der Karosserie, dort traegt der Nichtgleichgewichtsanteil Scherinformation (EQ-Arm wuerde sie je Blend verwerfen).");
			if(getenv("CFD_N2F_SCHALE_LAGEN")||getenv("CFD_N2F_SCHALE_XPLUS")||getenv("CFD_N2F_SCHALE_XMINUS")||getenv("CFD_N2F_SCHALE_XPLUS_SKAL")) print_warning("CFD_N2F_SCHALE_LAGEN/XPLUS/XMINUS/XPLUS_SKAL gelten NUR im Schalen-Modus -- im VOLUMEN-Modus (CFD_N2F_VOLUMEN=1) werden sie NICHT angewandt (Ansage-Doktrin).");
		}
		else if(n2f_lagen>0u) print_info("N2F-SCHALE GRADIENT: "+to_string(n2f_lagen)+" Lagen, Gewichte w_k = (N-k)/N linear (innen 1,000 -> aussen "+to_string(1.0f/(float)n2f_lagen,3u)+"); x+-Skalierung "+to_string(n2f_xskal,3u)+"; Modus "+to_string(n2f_modus)+(n2f_modus==2u?" (IDENT-Debug)":n2f_modus==1u?" (FNEQ: Nichtgleichgewichtsanteil erhalten, seit 2026-08-22 Default)":" (EQ -- seit 2026-08-22 NICHT mehr Standard)")+".");
		else print_info("N2F-SCHALE KONTROLLARM (CFD_N2F_SCHALE_LAGEN=0): Altverhalten -- uniforme Doppellage bei rand~100mm, Gewicht 1,0; Modus "+to_string(n2f_modus)+".");
		if(n2f_ident>0u) print_warning("CFD_N2F_SCHALE_IDENT=1: Blend ist ein EXAKTES No-Op (Debug-Paritaetsbeweis) -- dieser Lauf traegt KEINE Schalen-Physik.");
		if(n2f_ident>0u&&n2f_fneq>0u) print_warning("CFD_N2F_SCHALE_FNEQ wird von CFD_N2F_SCHALE_IDENT=1 ueberstimmt (Debug-Arm).");
		if(n2f_volumen==0u&&n2f_lagen==0u&&getenv("CFD_N2F_SCHALE_XPLUS_SKAL")) print_warning("CFD_N2F_SCHALE_XPLUS_SKAL wird im Kontrollarm LAGEN=0 NICHT angewandt (uniformes Gewicht 1,0 ist die Definition des Altverhaltens).");
		if(n2f_volumen==0u&&n2f_xskal==0.0f) print_warning("CFD_N2F_SCHALE_XPLUS_SKAL=0: x+-Zellen bleiben mit Gewicht 0 in der Liste und wuerden auf ihr LOKALES Gleichgewicht projiziert (a=0 ist im EQ-Arm KEIN No-Op; im FNEQ-Arm nahezu, in IDENT exakt ein No-Op) -- zum Abschalten der Flaeche CFD_N2F_SCHALE_XPLUS=0 nutzen.");
	}
	if(n2f_alpha==0.0f&&(getenv("CFD_N2F_BAND")||getenv("CFD_N2F_BAND_N")||getenv("CFD_N2F_BAND_PROFIL")||getenv("CFD_N2F_BAND_UNTERBODEN")||getenv("CFD_N2F_VOLUMEN")||getenv("CFD_N2F_SCHALE_XPLUS")||getenv("CFD_N2F_SCHALE_XMINUS")||getenv("CFD_N2F_SCHALE_MITTEL")||getenv("CFD_N2F_SCHALE_LAGEN")||getenv("CFD_N2F_SCHALE_FNEQ")||getenv("CFD_N2F_SCHALE_XPLUS_SKAL")||getenv("CFD_N2F_SCHALE_IDENT")||getenv("CFD_N2F_BAND_WAKE")||getenv("CFD_N2F_BAND_NURWAKE")||getenv("CFD_N2F_BAND_WAKE_START")||getenv("CFD_N2F_BAND_WAKE_START_X")||getenv("CFD_N2F_BAND_WAKE_ABSTAND")||getenv("CFD_N2F_BAND_PLATEAU")||getenv("CFD_N2F_BAND_WANDFREI")||getenv("CFD_N2F_PARITAET"))) print_warning("CFD_N2F_BAND/_N/_PROFIL/_UNTERBODEN, CFD_N2F_BAND_WAKE/_START/_START_X/_ABSTAND, CFD_N2F_PARITAET, CFD_N2F_VOLUMEN und CFD_N2F_SCHALE_XPLUS/XMINUS/MITTEL/LAGEN/FNEQ/XPLUS_SKAL/IDENT ohne CFD_N2F_SCHALE>0 sind stille No-Ops (P9c; die WAKE-/PARITAET-Schalter fehlten bis 2026-08-22 in dieser Liste -- getenv ist Exaktvergleich, Pruefagent-B8).");
	// ★ P8 (Offen-Punkt 8): FACETTEN AUCH IM FERNFELD. Der 32-mm-Treppenkoerper verdraengt +15,5 %
	// Volumen (Schritt-0-Census fc0edcf) und praegt dem Nahfeld ueber die Kopplungsebenen ein zu
	// grobes Druckfeld auf -- der Facettenpfad macht die Fernfeld-Wand druckkonsistenter. Schalter
	// CFD_FERN_FACETTEN=0..4, Arm-Codierung wie CFD_FACETTEN (Default 0 = bitidentisch reines BB).
	// HIER ZWISCHEN lbm_f- und lbm_c-Konstruktion (Muster CFD_FERN_BODEN_EQ/EINLASS_EQ): die Kernel-
	// Emission liest die Statik im KONSTRUKTOR, Zeile oben hat sie fuer lbm_c bereits auf AUS genullt.
	// CFD_FAC_ALPHA/CFD_FAC_SATGATE/CFD_FACETTEN_YWMIN gelten mit (DIESELBEN Envs fuer beide Gitter);
	// EMA/PEMA/APG/DIAGZ bleiben im Fernfeld bewusst AUS (Messarme, nur Nahfeld verdrahtet).
	{ const uint ffc = env_u("CFD_FERN_FACETTEN", 0u);
	  if(ffc>4u) print_error("CFD_FERN_FACETTEN kennt nur 0..4 (1/2 Paartausch voll/Tausch, 3/4 iMEM voll/Nullziel).");
	  if(ffc>=3u&&env_u("CFD_FAC_ALPHA", 0u)>2u) print_error("CFD_FAC_ALPHA kennt nur 0..2 (gilt auch fuer CFD_FERN_FACETTEN).");
	  LBM_Domain::s_facetten = ffc>0u; LBM_Domain::s_fac_imem = ffc>=3u;
	  LBM_Domain::s_fac_lsq = env_u("CFD_FAC_LSQ", 0u)>0u; LBM_Domain::s_fac_quergate = env_u("CFD_FAC_QUERGATE", 0u)>0u; LBM_Domain::s_fac_elibb = env_u("CFD_FAC_ELIBB", 0u)>0u; LBM_Domain::s_fac_elibb_pur = env_u("CFD_FAC_ELIBB", 0u)==2u; LBM_Domain::s_fac_qmin = env_f("CFD_FAC_QMIN", 0.1f); LBM_Domain::s_fac_kappa = env_f("CFD_FAC_KAPPA", 0.4f); LBM_Domain::s_fac_utkorr = env_f("CFD_FAC_UTKORR", 1.0f); LBM_Domain::s_fac_qkappe = env_f("CFD_FAC_QKAPPE", 1.0f); LBM_Domain::s_fac_qdiag = env_u("CFD_FAC_QDIAG", 0u); LBM_Domain::s_fac_pinv = env_u("CFD_FAC_PINV", 0u); if(LBM_Domain::s_fac_pinv>0u&&env_u("CFD_FAC_LSQ",0u)>0u) print_error("CFD_FAC_PINV und CFD_FAC_LSQ schliessen sich aus -- PINV ersetzt denselben Zweig, LSQ waere still wirkungslos (der #elif faellt durch). Einen von beiden waehlen."); if(LBM_Domain::s_fac_pinv>0u) print_info("RANG-1-PSEUDOINVERSE (CFD_FAC_PINV, 04.09.2026): im gekoppelten Zweig ersetzt Moore-Penrose die achsenparallele Skalarleiter -- Division ueber die SPUR (groesster Eigenwert) statt ueber Gt11. Grund: fuer die ebene Voxelwand ist tr(Gt) exakt 1/3 und kippungsunabhaengig, waehrend Gt11 mit der Stroemungsrichtung gegen 0 laeuft und die Akzeptanzschwelle 1e-4 dann Verstaerkung bis 1e4 durchlaesst. Wirkpfad Slot 80; erwartet fallen Slot 10 UND Slot 16, weil der Eigenvektor Sn.v = 0 exakt erfuellt und damit keine Normalkompensation mehr erzeugt wird."); LBM_Domain::s_fac_idx_voll = env_u("CFD_FAC_IDX_VOLL", 0u); if(LBM_Domain::s_fac_idx_voll>0u) print_info("CFD_FAC_IDX_VOLL=1: fac_idx in der ALTEN Vollfeldform -- deklarierter A/B-Arm gegen die Bitmaske (03.09.). Die Ergebnisse MUESSEN bitgleich sein, unterscheiden darf sich nur der Speicher."); LBM_Domain::s_f_liste = env_u("CFD_F_LISTE", 0u); if(LBM_Domain::s_f_liste>0u) print_info("F-MARKERLISTE (CFD_F_LISTE, 03.09.2026, Befunde B78b/B80/B81): F wird nur fuer WANDsolidzellen alloziert -- 8 mm gemessen: Nahfeld 238 -> 14 MiB, Fernfeld 4 -> 0 MiB. ABGENOMMEN ueber alle drei Sprossen bitgleich (CPU 5/5, iGPU 5/5 und dreimal reproduziert, B70 8-mm-Fahrzeug 19/19), Slot 77 = 0. Der urspruengliche Defekt war NICHT die Liste, sondern die Reihenfolge: die JIT-Defines entstanden vor dem Setzen der Schalter (B81)."); if(LBM_Domain::s_f_liste>0u&&!f_nur_solid_an_setup()) print_error("CFD_F_LISTE braucht CFD_F_NUR_SOLID (Default an): der Kontrollarm CFD_F_NUR_SOLID=0 liest F an JEDER Fluidzelle, und dort gibt es unter der Markerliste keinen Speicherplatz mehr -- die Kombination waere still falsch."); LBM_Domain::s_fac_satgate = ffc>=3u&&env_u("CFD_FAC_SATGATE", 0u)>0u; LBM_Domain::s_fac_kraft = ffc>=3u ? min(2u, env_u("CFD_FAC_KRAFT", 0u)) : 0u; LBM_Domain::s_fac_kdiag = ffc>=3u ? env_u("CFD_FAC_KDIAG", 0u) : 0u; LBM_Domain::s_fac_nachbar = ffc>=3u ? env_u("CFD_FAC_NACHBAR", 0u) : 0u; LBM_Domain::s_fac_messnur = ffc>=3u ? env_u("CFD_FAC_MESSNUR", 0u) : 0u; /* s_sgs_gdiag hier NICHT anfassen (B1-Statik-Falle, 02.09. selbst hineingelaufen: das ffc-Parsing laeuft VOR alloc_facetten_domain der NAHFELD-Domaene -- ein 0u hier machte die Diagnose am Fahrzeug zum stillen No-Op, im Kanal unsichtbar). Das Fernfeld ruft alloc_facetten_domain nur unter CFD_FERN_FACETTEN; eine dortige Mit-Messung waere harmlos. */ if(LBM_Domain::s_fac_kraft>0u) print_info(string("iMEM-KRAFTPFAD (Weg F, 30.08.): Modus ")+to_string(LBM_Domain::s_fac_kraft)+" -- Zellkraft statt Slip an "+(LBM_Domain::s_fac_kraft==1u?string("Rueckfallzellen (Slot 70, Soll == Slot 69)"):string("ALLEN Facettenzellen (Diskriminator)"))); if(LBM_Domain::s_fac_kraft>0u) print_warning("KRAFTPFAD FERNFELD (Pruefpunkt 8, 30.08.): object_force/forces.csv (Impulsaustausch an Koerperzellen) sieht die Volumenkraft NICHT -- eine Guo-Kraft im Fluid hat keine Newton-3-Reaktion am Koerper. Der Reibungsanteil an Kraftzellen steht allein in der fac_tau-Buchung (cd_reib/cd_rest); object_force-Abgleiche (K4, Fx_far) weichen um genau den Kraftanteil ab.");
	  LBM_Domain::s_fac_alpha = (ffc>=3u) ? env_u("CFD_FAC_ALPHA", 0u) : 0u; LBM_Domain::s_fac_lsq = env_u("CFD_FAC_LSQ", 0u)>0u; LBM_Domain::s_fac_quergate = env_u("CFD_FAC_QUERGATE", 0u)>0u; LBM_Domain::s_fac_elibb = env_u("CFD_FAC_ELIBB", 0u)>0u; LBM_Domain::s_fac_elibb_pur = env_u("CFD_FAC_ELIBB", 0u)==2u; LBM_Domain::s_fac_qmin = env_f("CFD_FAC_QMIN", 0.1f); LBM_Domain::s_fac_kappa = env_f("CFD_FAC_KAPPA", 0.4f); LBM_Domain::s_fac_utkorr = env_f("CFD_FAC_UTKORR", 1.0f); LBM_Domain::s_fac_qkappe = env_f("CFD_FAC_QKAPPE", 1.0f); LBM_Domain::s_fac_qdiag = env_u("CFD_FAC_QDIAG", 0u); LBM_Domain::s_sgs_guo = env_u("CFD_SGS_GUO", 1u)>0u;
	  LBM_Domain::s_fac_tau = (ffc==2u||ffc==4u) ? 0.0f : 1.0f;
	  if(ffc>0u) print_info(string("Facettenpfad FERNFELD (P8): ")+(ffc==1u?"Paartausch voll":ffc==2u?"Paartausch NUR TAUSCH":ffc==3u?"iMEM voll":"iMEM NULLZIEL")
	  	+" -- 16/32-mm-Treppenkoerper, YWMIN-Vorsicht: CFD_FACETTEN_YWMIN ist eine ZELLWEITE und gilt fuer BEIDE Gitter, am Grobgitter also die "+to_string(ratio)+"-fache physische Distanz."
	  	+" ALPHA="+to_string(LBM_Domain::s_fac_alpha)+"/SATGATE="+to_string(LBM_Domain::s_fac_satgate?1u:0u)+" gelten mit; EMA/PEMA/APG/DIAGZ bleiben AUS."
	  	+" ACHTUNG: Fx_far (forces.csv) ist in diesem Arm PHANTOMBEHAFTET (object_force an facettenbehandelten Links); kraft_facetten bleibt Nahfeld-only.");
	}
	LBM lbm_c(uint3(cNx, cNy, cNz), nu_lat_c, dev_coarse);

	// ---------------------------------------------------------------- Voxelisieren, beide Gitter
	// Das Fahrzeug MUSS auch im groben Gitter stehen. Sonst traegt das Fernfeld die Verdraengung nicht,
	// und die Nahfeld-Raender bekaemen ungestoerte Anstroemung aufgepraegt -- der Wagen staende dann in
	// einer Stroemung, die nicht weiss, dass er da ist.
	// ★ Audit-Nacharbeit 13 (ersetzt den faktisch falschen Kommentar von 2026-08-08): voxelize_mesh_
	// on_device liest vor initialize() SELBST flags UND u zurueck (lbm.cpp, !initialized-Zweig) --
	// die Behauptung "hier wird nur flags gelesen" stimmte nicht. Unveraendert wahr ist der Kern:
	// initialize() laedt spaeter das Host-u hoch und ueberschreibt es bedingungslos; heute folgenlos,
	// weil das Fahrzeug steht und beide Seiten 0 sind. Wer dem Fahrzeug je Geschwindigkeit gibt,
	// muss die Reihenfolge Voxelisieren->initialize() neu pruefen -- sonst ist sie lautlos weg, und initialize()
	// setzt dann auch kein TYPE_MS an der Fahrzeugwand, die Wand waere also nicht mitbewegt.
	// Nicht vorsorglich eingebaut, weil das Ruecklesen von u 6 GB ueber PCIe kostet und heute nichts tut.
	lbm_f.voxelize_mesh_on_device(veh_f, TYPE_S|TYPE_X); lbm_f.flags.read_from_device();
	sat_shell_and_void_fill(lbm_f, veh_f, fNx, fNy, fNz);
	lbm_c.voxelize_mesh_on_device(veh_c, TYPE_S|TYPE_X); lbm_c.flags.read_from_device();
	sat_shell_and_void_fill(lbm_c, veh_c, cNx, cNy, cNz);

	// ★ P8/P9 Schritt 0: VERDRAENGUNGS-CENSUS (reine AUSGABE, keine Physik). Beide Gitter tragen
	// dasselbe Fahrzeug, aber als unterschiedlich grobe Treppenkoerper -- WIE unterschiedlich, war
	// bisher nirgends beziffert. Hier direkt nach der Voxelisierung (flags liegen durch
	// read_from_device + sat_shell_and_void_fill ohnehin im Host-Spiegel, kein zusaetzlicher
	// GPU-Read): je Gitter das 0x41-Zellvolumen und die yz-projizierte Stirnflaeche (Anzahl
	// (y,z)-Saeulen mit mindestens einer 0x41-Zelle). Bewusst VOR der Kontaktflaechen-Uebergabe,
	// damit beide Gitter mit identischer Zellmenge verglichen werden (z=0-Zellen noch enthalten).
	{
		auto verdraengung = [](LBM& L, const uint Nx, const uint Ny, const uint Nz, ulong& n_cells, ulong& n_cols) {
			n_cells=0ull; n_cols=0ull;
			for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) {
				bool hit=false;
				for(uint x=0u; x<Nx; x++) {
					if((L.flags[(ulong)x+((ulong)y+(ulong)z*(ulong)Ny)*(ulong)Nx]&(TYPE_S|TYPE_X))==(TYPE_S|TYPE_X)) { n_cells++; hit=true; }
				}
				if(hit) n_cols++;
			}
		};
		ulong nf=0ull, cf=0ull, nc=0ull, cc=0ull;
		verdraengung(lbm_f, fNx, fNy, fNz, nf, cf);
		verdraengung(lbm_c, cNx, cNy, cNz, nc, cc);
		const double vol_f = (double)nf*(double)dx_f*(double)dx_f*(double)dx_f, vol_c = (double)nc*(double)dx_c*(double)dx_c*(double)dx_c;
		const double a_f = (double)cf*(double)dx_f*(double)dx_f, a_c = (double)cc*(double)dx_c*(double)dx_c;
		print_info("VERDRAENGUNGS-CENSUS fein ("+to_string(dx_f*1000.0f,0u)+" mm): "+to_string(nf)+" 0x41-Zellen = "+to_string((float)vol_f,4u)
			+" m3, Stirnflaeche (yz-Projektion) "+to_string(cf)+" Saeulen = "+to_string((float)a_f,4u)+" m2");
		print_info("VERDRAENGUNGS-CENSUS fern ("+to_string(dx_c*1000.0f,0u)+" mm): "+to_string(nc)+" 0x41-Zellen = "+to_string((float)vol_c,4u)
			+" m3, Stirnflaeche (yz-Projektion) "+to_string(cc)+" Saeulen = "+to_string((float)a_c,4u)+" m2");
		if(vol_f>0.0 && a_f>0.0) {
			const double dvol = 100.0*(vol_c/vol_f-1.0), da = 100.0*(a_c/a_f-1.0);
			print_info("VERDRAENGUNGS-CENSUS: der "+to_string(dx_c*1000.0f,0u)+"-mm-Treppenkoerper verdraengt "+to_string((float)fabs(dvol),2u)
				+" % "+(dvol>=0.0?"MEHR":"WENIGER")+" Volumen als der "+to_string(dx_f*1000.0f,0u)+"-mm-Treppenkoerper; Stirnflaeche "
				+to_string((float)fabs(da),2u)+" % "+(da>=0.0?"groesser":"kleiner")+".");
		}
	}

	// ---------------------------------------------------------------- Kontaktflaeche, beide Gitter
	// Fahrzeugzellen auf z=0 werden Teil der Fahrbahn (TYPE_X weg, u = u_road). Begruendung ausfuehrlich
	// im Einzelgitter-Fall weiter oben; sie gilt hier unveraendert und fuer beide Aufloesungen.
	auto handover_contact = [&](LBM& L, const uint Nx, const uint Ny, const char* who) {
		ulong contact = 0ull;
		for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
			const ulong n = (ulong)x + (ulong)y*(ulong)Nx;
			if((L.flags[n]&TYPE_X)!=0u) {
				L.flags[n] &= (uchar)~TYPE_X;
				L.u.x[n] = u_lat; L.u.y[n] = 0.0f; L.u.z[n] = 0.0f;
				contact++;
			}
		}
		print_info(string(who)+": "+to_string(contact)+" Fahrzeugzellen auf z=0 an die Strasse uebergeben");
	};
	handover_contact(lbm_f, fNx, fNy, "Kontaktflaeche fein");
	handover_contact(lbm_c, cNx, cNy, "Kontaktflaeche grob");

	// ---------------------------------------------------------------- tatsaechliche Stirnflaeche
	// ★ Pruefer-Befund 2026-08-08: place() skaliert die STL UNIFORM ueber die Laenge. Die STL ist aber
	// 4,4341 m lang statt 4,4364 -- der Faktor 1,000524 streckt damit auch Breite und Hoehe um 0,05 %,
	// obwohl die Hoehe vorher exakt stimmte. Dazu kommt die Voxelisierung. Cd wird hier auf A_ref = 1,85
	// normiert (Projektkonvention, damit die Zahl mit OpenFOAM vergleichbar bleibt); die WIRKLICH
	// voxelisierte Silhouette wird daneben ausgewiesen. Der Unterschied ist kein Rundungsfehler,
	// sondern ein systematischer Versatz von Cd -- er gehoert sichtbar, nicht in eine Fussnote.
	float A_eff = 0.0f;
	{
		ulong sil = 0ull;
		for(uint z=0u; z<fNz; z++) for(uint y=0u; y<fNy; y++) {
			for(uint x=0u; x<fNx; x++) {
				if((lbm_f.flags[(ulong)x+((ulong)y+(ulong)z*(ulong)fNy)*(ulong)fNx]&TYPE_X)!=0u) { sil++; break; }
			}
		}
		A_eff = (float)sil*dx_f*dx_f;
		print_info("Stirnflaeche: A_ref = "+to_string(A_ref,4u)+" m2 (Projektkonvention, Normierung von Cd) gegen "
			+to_string(A_eff,4u)+" m2 tatsaechlich voxelisiert ("+to_string(sil)+" Zellen, "
			+to_string(100.0f*(A_eff/A_ref-1.0f),2u)+" % Abweichung). Cd waere auf A_eff um diesen Betrag kleiner.");
		// Der Reifenlatsch ist hier NICHT mitgezaehlt (TYPE_X wurde dort entfernt) -- konsistent zu
		// object_force(TYPE_S|TYPE_X), das ihn ebenfalls nicht mitsummiert.
	}

	// ---------------------------------------------------------------- Randbedingungen
	// GROB: z=0 mitbewegte Fahrbahn, x- Einlass, x+ Druckauslass, y+-/z+ Freistrom.
	// FEIN: z=0 mitbewegte Fahrbahn, x+ Druckauslass (der Nachlauf MUSS abstroemen koennen),
	//       x-/y+-/z+ sind TYPE_E und werden jede grobe Zeitschrittdauer aus dem Fernfeld getrieben.
	auto set_bcs = [&](LBM& L, const uint Nx, const uint Ny, const uint Nz) {
		for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
			const ulong n = (ulong)x + ((ulong)y + (ulong)z*(ulong)Ny)*(ulong)Nx;
			if((L.flags[n]&TYPE_S)!=0u) continue; // Fahrzeug und bereits gesetzte Fahrbahnzellen in Ruhe lassen
			if(z==0u) { L.flags[n] = TYPE_S; L.u.x[n] = u_lat; L.u.y[n] = 0.0f; L.u.z[n] = 0.0f; }
			else if(x==0u || x==Nx-1u || y==0u || y==Ny-1u || z==Nz-1u) { L.flags[n] = TYPE_E; L.u.x[n] = u_lat; L.u.y[n] = 0.0f; L.u.z[n] = 0.0f; }
			else { L.u.x[n] = u_lat; L.u.y[n] = 0.0f; L.u.z[n] = 0.0f; }
		}
	};
	set_bcs(lbm_f, fNx, fNy, fNz);
	set_bcs(lbm_c, cNx, cNy, cNz);
	{	// ★ FERN-BODENKLEMME (Heiko 2026-08-20, V1-apply_floor_velocity-Klasse): die untersten N
		// Fluidzeilen des FERNFELDS werden als TYPE_E auf u_inf geklemmt -- Diagnose-Arm gegen die
		// beobachtete, physikalisch unmoegliche Bodengrenzschicht ab Fernfeld-Einlass (Slice-Punkt 5).
		// Solid (Fahrzeug/Latsch) und bestehende Raender bleiben unangetastet; die Kopplungs-
		// Entnahmeebenen liegen im Klemmband und speisen das Near damit automatisch korrekt.
		const uint fbk = env_u("CFD_FERN_BODENKLEMME", 0u);
		if(fbk>0u) { ulong nk=0ull;
			for(uint z=1u; z<=min(fbk, cNz-2u); z++) for(uint y=0u; y<cNy; y++) for(uint x=0u; x<cNx; x++) {
				const ulong n=(ulong)x+((ulong)y+(ulong)z*(ulong)cNy)*(ulong)cNx;
				if((lbm_c.flags[n]&(TYPE_S|TYPE_E))!=0u) continue;
				lbm_c.flags[n]=TYPE_E; lbm_c.u.x[n]=u_lat; lbm_c.u.y[n]=0.0f; lbm_c.u.z[n]=0.0f; nk++;
			}
			print_info("FERN-BODENKLEMME aktiv (V1-Moving-Floor-Fix-Klasse): Fernfeld z=1.."+to_string(fbk)+" als TYPE_E u_inf ("+to_string(nk)+" Zellen; Diagnose-Arm).");
		}
	}
	// ★ C1b Stufe 1: jede Instanz baut ihre Facetten aus den EIGENEN flags (FACETTEN-PLAN Stufe 5);
	// hier nach set_bcs (Revision Auflage 6). Nahfeld ist das Abnahmegitter.
	std::unordered_map<ulong,std::array<uchar,18>> elibb_qmap_dd; // ★ B1-Stufe 2 (Funktionsscope, VOR beiden Fuellstellen)
	if(env_u("CFD_FACETTEN_DIAG", 0u)>0u) {
		// out_dir des Falls entsteht erst weiter unten -- hier derselbe Ausdruck lokal.
		const string fac_dir = get_exe_path()+"../export/"+(getenv("CFD_RUN_NAME")?string(getenv("CFD_RUN_NAME")):string("fahrzeug_dd"))+"/";
		create_folder(fac_dir);
		// ★ Audit 2/3 MITTEL: beide Gitter schrieben in DENSELBEN Dateinamen -- das Fernfeld
		// ueberschrieb den Nahfeld-Census kommentarlos. Jetzt je ein Unterordner.
		const string fdn = fac_dir+"nah/", fdf = fac_dir+"fern/"; create_folder(fdn); create_folder(fdf);
		vtk_x0=near_x0; vtk_y0=near_y0; vtk_z0=near_z0; vtk_dx=dx_f; // Guete-VTK in WELTkoordinaten (deckungsgleich mit feld_nah_*.vtk)
		baue_facetten(lbm_f, fNx, fNy, fNz, (uchar)(TYPE_S|TYPE_X), fdn, "dd-Nahfeld");
		const bool elibb_an_dd = env_u("CFD_FACETTEN",0u)>=3u&&env_u("CFD_FAC_ELIBB",0u)>0u;
		if(env_u("CFD_FACETTEN_REMESH", 0u)>0u||elibb_an_dd) { // ★ ELIBB P1 + Stufe 2 (Pflicht-q-Quelle bei ELIBB)
			print_info("ELIBB P1: Remesh der Nahfeld-Voxelaussenwand (Surface Nets + Taubin) ...");
			remesh_facetten_diag(lbm_f, fNx, fNy, fNz, (uchar)(TYPE_S|TYPE_X), fdn, nullptr, veh_f, elibb_an_dd?&elibb_qmap_dd:nullptr);
		}
		vtk_x0=far_x0; vtk_y0=far_y0; vtk_z0=0.0f; vtk_dx=dx_c; // ★ Pruefagent D-1: das Fernfeld erbte die NAHFELD-Lage
		baue_facetten(lbm_c, cNx, cNy, cNz, (uchar)(TYPE_S|TYPE_X), fdf, "dd-Fernfeld");
		if(env_u("CFD_FACETTEN_DIAG", 0u)==2u) _exit(0);
	}
	// ★ Stufe 5: aktiver Facettenbau NUR Nahfeld (nach set_bcs/Kontaktflaeche -- Revision Auflage 6).
	std::vector<Facette> FFn;
	if(env_u("CFD_FACETTEN", 0u)>0u) {
		const string fdir = get_exe_path()+"../export/"+(getenv("CFD_RUN_NAME")?string(getenv("CFD_RUN_NAME")):string("fahrzeug_dd"))+"/";
		create_folder(fdir);
		const ulong census_v = [&]{ ulong c=0ull; for(ulong n=0ull;n<lbm_f.get_N();n++) if(lbm_f.flags[n]==(TYPE_S|TYPE_X)) c++; return c; }();
		vtk_x0=near_x0; vtk_y0=near_y0; vtk_z0=near_z0; vtk_dx=dx_f; // ★ D-1: auch der PRODUKTIVE Pfad braucht die Weltlage
		FFn = baue_facetten(lbm_f, fNx, fNy, fNz, (uchar)(TYPE_S|TYPE_X), fdir, "dd-Nahfeld");
		// ★ B1-Stufe 2: bei ELIBB ist das Remesh die PFLICHT-q-Quelle -- auch ohne CFD_FACETTEN_DIAG
		// (der Diagnoseblock oben laeuft im Normalfall nicht; die erste Verdrahtung hing daran).
		if(env_u("CFD_FACETTEN",0u)>=3u&&env_u("CFD_FAC_ELIBB",0u)>0u&&elibb_qmap_dd.empty()) { // ★ Host-Audit Befund 2: ohne >=3 lief der teure Remesh bei Paartausch ins Leere
			print_info("ELIBB Stufe 2: Remesh der Nahfeld-Voxelaussenwand fuer fac_q ...");
			remesh_facetten_diag(lbm_f, fNx, fNy, fNz, (uchar)(TYPE_S|TYPE_X), fdir, nullptr, veh_f, &elibb_qmap_dd);
		}
		const ulong census_n = [&]{ ulong c=0ull; for(ulong n=0ull;n<lbm_f.get_N();n++) if(lbm_f.flags[n]==(TYPE_S|TYPE_X)) c++; return c; }();
		if(census_v!=census_n) print_error("Facettenbau hat den 0x41-Census veraendert ("+to_string(census_v)+" -> "+to_string(census_n)+") -- object_force-Falle!");
		if(env_u("CFD_FACETTEN_DIAG", 0u)==2u) _exit(0); // Schritt-0-Diagnose auch im aktiven Arm
	}
	// ★ P8: aktiver Facettenbau FERNFELD (CFD_FERN_FACETTEN, unabhaengig von CFD_FACETTEN) -- NACH
	// set_bcs, Kontaktflaeche und FERN-BODENKLEMME. baue_facetten rechnet rein in GITTEREINHEITEN
	// (keine dx-Parameter in der Signatur; y_w und YWMIN sind Zellweiten und skalieren automatisch
	// mit dx_c). BODENKLEMME-Zellen (TYPE_E) sind ausgeschlossen: ist_fluid verlangt
	// flags&(TYPE_S|TYPE_E)==0, ist_wand exakt 0x41 -- geklemmte Zellen tragen also weder Facette
	// noch zaehlen sie als Wand. FERN_BODEN_EQ/EINLASS_EQ (post-stream-Resets in z=1..N/x=1..N)
	// koennen facettenbehandelte Zellen ueberschreiben -- harmlos (das EQ-Reset greift NACH der
	// Facetten-Kollision desselben Schritts, die Wandbehandlung des Folgeschritts sieht nur das
	// Equilibrium-Feld), aber hiermit deklariert. CSVs in fern/ (Audit-2/3-Lektion: das Fernfeld
	// ueberschrieb sonst den Nahfeld-Census kommentarlos).
	std::vector<Facette> FFc;
	if(env_u("CFD_FERN_FACETTEN", 0u)>0u) {
		const string fdir = get_exe_path()+"../export/"+(getenv("CFD_RUN_NAME")?string(getenv("CFD_RUN_NAME")):string("fahrzeug_dd"))+"/";
		create_folder(fdir); const string fdirc = fdir+"fern/"; create_folder(fdirc);
		const ulong census_v = [&]{ ulong c=0ull; for(ulong n=0ull;n<lbm_c.get_N();n++) if(lbm_c.flags[n]==(TYPE_S|TYPE_X)) c++; return c; }();
		vtk_x0=far_x0; vtk_y0=far_y0; vtk_z0=0.0f; vtk_dx=dx_c;
		FFc = baue_facetten(lbm_c, cNx, cNy, cNz, (uchar)(TYPE_S|TYPE_X), fdirc, "dd-Fernfeld (P8)");
		const ulong census_n = [&]{ ulong c=0ull; for(ulong n=0ull;n<lbm_c.get_N();n++) if(lbm_c.flags[n]==(TYPE_S|TYPE_X)) c++; return c; }();
		if(census_v!=census_n) print_error("Facettenbau (Fernfeld) hat den 0x41-Census veraendert ("+to_string(census_v)+" -> "+to_string(census_n)+") -- object_force/Fx_far-Falle!");
	}
	if(getenv("CFD_PO_FACES")) print_warning("CFD_PO_FACES wird im dd-Fall NICHT angewandt (Maske hart x_max -- Gross-Audit M10)."); // Ansage-Doktrin
	lbm_f.set_pressure_outlet_faces(2u, env_f("CFD_PO_RHO", 1.0f)); // Bit 2 = x_max
	lbm_c.set_pressure_outlet_faces(2u, env_f("CFD_PO_RHO", 1.0f));
	// ★★ 2026-08-25, Audit-Befund geschlossen: "Fernfeld-Einlass im PRODUKTIONSFALL ueberbestimmt --
	// set_velocity_inlet_faces steht NUR in main_setup_fernfeld". Der Schalter haengt jetzt auch hier,
	// auf DERSELBEN Flaechenmaske 45 (x-, y-, y+, z+); nur das Fernfeld, das Nahfeld hat an x- die
	// Kopplungsebene. DEFAULT AUS, und zwar aus MESSUNG, nicht aus Vorsicht: im leeren groben Kanal
	// war der Arm mit schwebendem rho SCHLECHTER (Streuung 0,0712 gegen 0,0695, ueber 10 % daneben
	// 12,90 gegen 10,94 %) und der Massenstrom sackte monoton ab. Die Ursache der Reflexion ist nicht
	// die doppelte Vorgabe, sondern dass der TYPE_E-Reset den Nichtgleichgewichtsanteil verwirft --
	// dagegen hilft erst ein Rand, der f^neq traegt (Zou/He oder charakteristisch). Steht im Fahrplan.
	if(env_on("CFD_FERN_VI")) lbm_c.set_velocity_inlet_faces(45u);
	else print_info("Geschwindigkeits-Einlass Fernfeld AUS (CFD_FERN_VI=0, gemessener Default): rho bleibt am Einlass festgenagelt, der Rand reflektiert -- bekannt und angesagt.");
	lbm_f.finalize_sparse_tiles();
	lbm_c.finalize_sparse_tiles();
	// ★ 03.09.2026 F-MARKERLISTE: dasselbe Fenster wie alloc_facetten -- Geometrie steht, initialize()
	// kommt spaeter. UNKONDITIONAL (haengt nicht an CFD_FACETTEN); alloc_f_liste steigt selbst aus,
	// wenn CFD_F_LISTE=0 ist.
	lbm_f.lbm_domain[0]->alloc_f_liste(&lbm_f.flags[0], fNx, fNy, fNz);
	// ★ 03.09.: CFD_F_LISTE ist eine GLOBALE Statik -- damit bekommt AUCH das Fernfeld f_liste_on und
	// damit den 1-Element-Platzhalter fuer F. Wird alloc_f_liste dort nicht gerufen, faengt es der
	// Waechter in initialize() (genau so passiert, xz_fl8 03.09.). Der Gewinn ist im Fernfeld klein
	// (F-BBox 286x123x80), aber Konsistenz ist keine Geschmacksfrage: beide Domaenen oder keine.
	lbm_c.lbm_domain[0]->alloc_f_liste(&lbm_c.flags[0], cNx, cNy, cNz);
	if(env_u("CFD_FACETTEN", 0u)>0u) lbm_f.alloc_facetten(FFn, (env_u("CFD_FACETTEN",0u)>=3u&&env_u("CFD_FAC_ELIBB",0u)>0u&&!elibb_qmap_dd.empty())?&elibb_qmap_dd:nullptr, env_u("CFD_SGS_GDIAG", 0u), env_u("CFD_SGS_FDWAND", 0u)); // vor run(0) -- der run()-Guard verlangt die Bindung; Stufe-2-Karte wenn vorhanden
	if(env_u("CFD_FERN_FACETTEN", 0u)>0u) lbm_c.alloc_facetten(FFc); // P8: alloc_facetten_domain nutzt die INSTANZ-F-BBox von lbm_c (Fahrzeug+4 in Grobzellen, oben gesetzt) -- der Wachhund "Facette ausserhalb der F-BBox" prueft die Deckung hart

	// ---------------------------------------------------------------- Randbedingungen NACHZAEHLEN
	// Nicht der Code oben wird berichtet, sondern der ZUSTAND danach: fuer jede der sechs Flaechen
	// beider Gitter, welcher Typ dort wirklich steht und welche Geschwindigkeit vorgegeben ist.
	// Der Unterschied ist wesentlich -- die Voxelisierung, die Kontaktflaechen-Uebergabe und der
	// Druckauslass greifen alle in dieselben Zellen, und was am Ende steht, entscheidet die
	// Reihenfolge. Eine Tabelle aus dem Speicher luegt darueber nicht.
	{
		auto census = [&](LBM& L, const uint Nx, const uint Ny, const uint Nz, const char* who) {
			print_info(string("--- Randbedingungen ")+who+" -----------------------------------------");
			const char* fname[6] = {"x- (Einlass) ", "x+ (Auslass) ", "y- (Seite)   ", "y+ (Seite)   ", "z- (Fahrbahn)", "z+ (Decke)   "};
			for(uint f=0u; f<6u; f++) {
				ulong n_solid=0ull, n_equil=0ull, n_veh=0ull, n_fluid=0ull, n_other=0ull;
				double ux_sum=0.0; ulong ux_n=0ull;
				const uint x0 = (f==0u)?0u:((f==1u)?Nx-1u:0u),           x1 = (f==0u)?0u:((f==1u)?Nx-1u:Nx-1u);
				const uint y0 = (f==2u)?0u:((f==3u)?Ny-1u:0u),           y1 = (f==2u)?0u:((f==3u)?Ny-1u:Ny-1u);
				const uint z0 = (f==4u)?0u:((f==5u)?Nz-1u:0u),           z1 = (f==4u)?0u:((f==5u)?Nz-1u:Nz-1u);
				for(uint z=z0; z<=z1; z++) for(uint y=y0; y<=y1; y++) for(uint x=x0; x<=x1; x++) {
					const ulong n = (ulong)x + ((ulong)y + (ulong)z*(ulong)Ny)*(ulong)Nx;
					const uchar fl = L.flags[n];
					if((fl&TYPE_X)!=0u) n_veh++;
					const uchar bo = fl&(TYPE_S|TYPE_E);
					if(bo==TYPE_S)      { n_solid++; ux_sum+=(double)L.u.x[n]; ux_n++; }
					else if(bo==TYPE_E) { n_equil++; ux_sum+=(double)L.u.x[n]; ux_n++; }
					else if(bo==0u)     { n_fluid++; }
					else                { n_other++; } // 0x03: auf dem Host TYPE_S|TYPE_E, geraeteseitig TYPE_MS.
					                                   // Vor initialize() kann das hier nicht auftreten, danach schon --
					                                   // wer diesen Census hinter run(0u) schiebt, zaehlt einen NORMALZUSTAND als unklar.
				}
				print_info(string("  ")+fname[f]+": Wand "+to_string(n_solid)+", Gleichgewicht "+to_string(n_equil)
					+", freies Fluid "+to_string(n_fluid)+(n_other? (", UNKLAR "+to_string(n_other)) : string(""))
					+(n_veh? (", davon Fahrzeug "+to_string(n_veh)) : string(""))
					+" | mittleres u_x = "+to_string(ux_n? (float)(ux_sum/(double)ux_n) : 0.0f, 5u)+" (u_inf = "+to_string(u_lat,5u)+")");
				if(n_other>0ull) print_warning(string(who)+" "+fname[f]+": Zellen mit TYPE_S UND TYPE_E gleichzeitig.");
				if(n_fluid>0ull && f!=1u) print_warning(string(who)+" "+fname[f]+": "+to_string(n_fluid)+" Zellen ohne Randbedingung -- dort rechnet der Loeser periodisch ueber den Rand.");
			}
		};
		census(lbm_c, cNx, cNy, cNz, "Fernfeld");
		census(lbm_f, fNx, fNy, fNz, "Nahfeld");
		print_info("Erwartung: Fahrbahn z- = Wand mit u_x = u_inf (mitbewegt); alle uebrigen Flaechen Gleichgewicht.");
		print_info("Im Nahfeld werden x-, y+-, z+ jeden groben Schritt aus dem Fernfeld getrieben; x+ bleibt Druckauslass.");
	}

	// ---------------------------------------------------------------- Kopplungsebenen
	auto mk = [](const uint ox, const uint oy, const uint oz, const uint ea, const uint eb, const uint ax) {
		PlaneSpec p; p.origin=uint3(ox,oy,oz); p.extent_a=ea; p.extent_b=eb; p.axis=ax; return p;
	};
	// Fuenf Flaechen: x-, x+, y-, y+, z+. Der Boden faellt weg (gemeinsame Fahrbahn).
	// Die grobe Entnahmeebene liegt GENAU auf der zugehoerigen feinen Randebene -- das ist der
	// ganze Sinn der Deckungspunkt-Konvention.
	const PlaneSpec cp[5] = {
		mk(NF_OX,           NF_OY,           NF_OZ,           cey, cez, 0u),
		mk(NF_OX+cex-1u,    NF_OY,           NF_OZ,           cey, cez, 0u),
		mk(NF_OX,           NF_OY,           NF_OZ,           cex, cez, 1u),
		mk(NF_OX,           NF_OY+cey-1u,    NF_OZ,           cex, cez, 1u),
		mk(NF_OX,           NF_OY,           NF_OZ+cez-1u,    cex, cey, 2u) };
	const PlaneSpec fp[5] = {
		mk(0u,        0u,        0u,        fNy, fNz, 0u),
		mk(fNx-1u,    0u,        0u,        fNy, fNz, 0u),
		mk(0u,        0u,        0u,        fNx, fNz, 1u),
		mk(0u,        fNy-1u,    0u,        fNx, fNz, 1u),
		mk(0u,        0u,        fNz-1u,    fNx, fNy, 2u) };
	const char* face_name[5] = {"x-", "x+", "y-", "y+", "z+"};
	// x+ (Index 1) wird NICHT getrieben: dort steht der Druckauslass des Nahfelds.
	const bool drive_face[5] = {true, false, true, true, true};

	// ---------------------------------------------------------------- Nachpruefen, bevor gerechnet wird
	// Drei Dinge, die still falsch sein koennten und es dann fuer den ganzen Lauf blieben.
	// ★ KORREKTUR 2026-08-08 (Pruefer-Befund): hier stand ueberall print_error(...) gefolgt von bad++,
	// und am Ende eine Zusammenfassung. print_error ruft aber exit(1) -- alles ab dem ersten Befund war
	// unerreichbar, die Sammlung also toter Code, und man saehe von mehreren Fehlern immer nur den
	// ersten. Jetzt wird gesammelt (print_warning) und AM ENDE einmal abgebrochen: man sieht alle
	// Beanstandungen auf einmal, und abgebrochen wird trotzdem.
	{
		uint bad = 0u;
		for(uint p=0u; p<5u; p++) { // (1) Deckungspunkt-Konvention je Ebene
			const uint ea_exp = (cp[p].extent_a-1u)*ratio+1u, eb_exp = (cp[p].extent_b-1u)*ratio+1u;
			if(ea_exp!=fp[p].extent_a || eb_exp!=fp[p].extent_b) {
				print_warning(string("Ebene ")+face_name[p]+": grob "+to_string(cp[p].extent_a)+"x"+to_string(cp[p].extent_b)
					+" ergaebe fein "+to_string(ea_exp)+"x"+to_string(eb_exp)+", die feine Ebene ist aber "
					+to_string(fp[p].extent_a)+"x"+to_string(fp[p].extent_b)+"."); bad++;
			}
		}
		// (2) Die groben Entnahmeebenen duerfen das FAHRZEUG nicht schneiden -- sonst speiste eine
		//     Fahrzeugzelle als vermeintliche Stroemung in den Nahfeld-Rand.
		//     Die FAHRBAHN (z=0) dagegen liegt bewusst mit drin: NF_OZ = 0, die Ebenen x+- und y+-
		//     enthalten also die Zeile z=0, und das sind mitbewegte Wandzellen mit rho=1 und
		//     u = (u_road, 0, 0). Als Stuetzstelle fuer die feinen Zellen z=1..ratio-1 ist das genau
		//     der richtige Wandwert -- sie werden mitgezaehlt und ausgewiesen, aber nicht beanstandet.
		//     ★ Das gilt nur, solange der Boden mitbewegt ist. Wuerde er je auf Haftbedingung ohne
		//     Mitbewegung umgestellt, speiste diese Zeile u=0 in den Nahfeld-Rand.
		for(uint p=0u; p<5u; p++) {
			ulong hit = 0ull, road = 0ull;
			for(uint b=0u; b<cp[p].extent_b; b++) for(uint a=0u; a<cp[p].extent_a; a++) {
				uint x=cp[p].origin.x, y=cp[p].origin.y, z=cp[p].origin.z;
				if(cp[p].axis==0u)      { y+=a; z+=b; }
				else if(cp[p].axis==1u) { x+=a; z+=b; }
				else                    { x+=a; y+=b; }
				const uchar fl = lbm_c.flags[(ulong)x+((ulong)y+(ulong)z*(ulong)cNy)*(ulong)cNx];
				if((fl&TYPE_X)!=0u) hit++;
				else if((fl&(TYPE_S|TYPE_E))==TYPE_S) road++;
			}
			if(hit>0ull) { print_warning(string("Grobe Entnahmeebene ")+face_name[p]+" schneidet das Fahrzeug in "+to_string(hit)+" Zellen. Die Nahfeld-Box ist dort zu eng."); bad++; }
			if(road>0ull) print_info(string("Grobe Entnahmeebene ")+face_name[p]+" enthaelt "+to_string(road)+" Fahrbahnzellen (mitbewegte Wand, korrekte Stuetzstelle).");
		}
		// (3) Weltkoordinaten-Deckung der Ebenen, unabhaengig von den Indizes nachgerechnet.
		const float dxy[5][2] = {{near_x0, far_x0+(float)cp[0].origin.x*dx_c}, {near_x0+(float)(fNx-1u)*dx_f, far_x0+(float)cp[1].origin.x*dx_c},
		                         {near_y0, far_y0+(float)cp[2].origin.y*dx_c}, {near_y0+(float)(fNy-1u)*dx_f, far_y0+(float)cp[3].origin.y*dx_c},
		                         {(float)(fNz-1u)*dx_f, (float)cp[4].origin.z*dx_c}};
		for(uint p=0u; p<5u; p++) if(fabs(dxy[p][0]-dxy[p][1])>1e-4f) {
			print_warning(string("Ebene ")+face_name[p]+" liegt raeumlich auseinander: fein "+to_string(dxy[p][0],5u)+" m, grob "+to_string(dxy[p][1],5u)+" m."); bad++;
		}
		if(bad>0u) print_error("Kopplungspruefung: "+to_string(bad)+" Beanstandung(en) -- siehe oben. Lauf nicht gestartet.");
		else print_info("Kopplungspruefung: Deckungspunkte, Fahrzeugfreiheit und Weltlage aller fuenf Ebenen in Ordnung.");
	}

	// ---------------------------------------------------------------- P9c N2F-Schale: Listenbau
	// ★ NACH der Kopplungspruefung (der Kasten haengt an denselben Deckungspunkt-Konventionen).
	// GRADIENT-VARIANTE (Default, CFD_N2F_SCHALE_LAGEN=N>0): N Ein-Zell-Lagen um die Fahrzeug-BBox
	// des GROBGITTERS (veh_c->pmin/pmax, VOR der M=4-Aufweitung der F-Box) auf den Kasten-Offsets
	// o0+k, k=0..N-1; o0=1, bei CFD_FERN_FACETTEN>0 o0=2 (die innerste Lage bleibt von den
	// FERN_FACETTEN-Zellen weg -- ERSETZT den alten rand<2-Guard, Pruefagent N1). Gewichte
	// w_k = (N-k)/N linear (innen 1 -> aussen 1/N), x+-Zellen zusaetzlich *CFD_N2F_SCHALE_XPLUS_SKAL.
	// Dedupliziert per std::map<ulong, (w, lage)> -- bei Duplikaten (Lagen-Kollision durch
	// Rand-Klemmung, Flaechen-Kanten) gewinnt das MAXIMALE Gewicht. KONTROLLARM
	// (CFD_N2F_SCHALE_LAGEN=0): Altverhalten -- Doppellage bei rand = floor(0.1/dx_c+0.5) (~100 mm),
	// uniformes Gewicht 1,0. Fuenf Flaechen x-, x+, y-, y+, z+ (z- entfaellt, gemeinsame Fahrbahn),
	// Flaechen-Maske JE LAGE; z >= max(1, CFD_FERN_BODEN_EQ+1) -- das boden_eq-Band bleibt
	// unangetastet. FLAECHEN-MASKE (Heiko-Verfeinerung 1): y-, y+, z+ AN; x- AUS (Zirkularitaet --
	// an der Anstroemung weiss das Near nichts Neues; CFD_N2F_SCHALE_XMINUS=1 schaltet zu); x+ AN,
	// per CFD_N2F_SCHALE_XPLUS=0 abschaltbar (Wake-Messarm).
	std::vector<ulong> n2f_liste_c, n2f_liste_f; // grobe Schalenzellen + feine Deckungspunkte, GLEICHE Reihenfolge (Map-Iteration, aufsteigend deterministisch)
	std::vector<float> n2f_gewicht; // Zellgewicht [0;1] (Lagen-Rampe), gleiche Reihenfolge
	std::vector<uint>  n2f_lage;    // Lagenindex je Zelle (0 = innen), gleiche Reihenfolge; Waechter/Negations-Nachweis/Kipp laufen NUR auf der aeussersten Lage
	// ★ ZONENMARKE je Listenzelle (host-seitig, geht NICHT aufs Geraet). Bit 0 = gehoert zum
	// Wake-Kasten oder seinem Auslauf, Bit 1 = liegt stromauf der Fahrzeugnase. Zwei vorhandene
	// Waechter brauchen das, sonst schlagen sie im Kastengebiet falsch an:
	//  - Der u-NEGATIONS-NACHWEIS misst mittleres u_x gegen ~0,52 u_inf. Im Nachlauf faellt u_x
	//    physikalisch darunter -> Fehlabbruch. Er laeuft deshalb nur stromauf der Nase, wo u_x zu
	//    jeder Zeit nahe +u_inf liegt. Das macht ihn schaerfer als vorher, nicht schwaecher.
	//  - Das KIPP-KRITERIUM ("10 Samples monoton steigend") ist am Anfahrtransienten geeicht. Im
	//    Wake waechst die Nah-Fern-Diskrepanz ueber die ersten ~0,1 s physikalisch monoton -> es
	//    waere ein Fehlalarm-Generator. Es laeuft deshalb nur auf der Aussenlage des KOERPERBANDS.
	std::vector<uchar> n2f_marke;
	// Kastengrenzen fuer die Impulsbilanz (0 = kein Kasten). Sie sind die Antwort auf den
	// staerksten Einwand gegen diesen Arm: das aufgepraegte Impulsdefizit liegt in der
	// Groessenordnung des GESAMTEN Fahrzeugwiderstands (~790 N gegen 611 N bei Cd 0,599), und
	// das Fernfeld-Fahrzeug erzeugt sein eigenes Defizit weiter. Ob daraus eine Doppelzaehlung
	// wird, entscheidet keine Ueberlegung, sondern die Bilanz ueber die sechs Kastenflaechen.
	uint bil_x0=0u, bil_x1=0u, bil_y0=0u, bil_y1=0u, bil_z0=0u, bil_z1=0u; bool bil_an=false;
	// ★ B-P1/B-P4: bei Profil 2 ist Lage N-1 gewichtslos und wird nicht mehr gelistet -- Waechter,
	// Negations-Nachweis und Kipp-Metrik laufen auf der aeussersten Lage MIT Gewicht (N-2). Sonst
	// waeren sie strukturell blind (im FNEQ-Arm speichert a=0 ftrue unabhaengig vom
	// Negationsvorzeichen -- genau der Fehler, den der Nachweis fangen soll, erreichte die
	// Messlage nicht mehr).
	const uint n2f_lage_aussen = (n2f_band>0u) ? (n2f_band_prof==2u&&n2f_band_n>1u ? n2f_band_n-2u : n2f_band_n-1u) : ((n2f_volumen>0u) ? 1u : ((n2f_lagen>0u) ? n2f_lagen-1u : 0u)); // BAND: Lage = d-1; Profil 2: N-2 (s.o.) // VOLUMEN: lage 1 = Aussenband (aeusserste Gewichtsstufe); Kontrollarm: alle Zellen sind Lage 0 = "aussen" (Altverhalten: Waechter ueber alles)
	float n2f_w_band = 0.0f; // VOLUMEN: groesstes Gewichtsfeld im Aussenband -- traegt in die Negations-Schwelle (w_aussen = alpha*n2f_w_band)
	ulong n2f_nanblocks = 0ull;
	const uint n2f_mittel = min(1u, env_u("CFD_N2F_SCHALE_MITTEL", 1u)); // 1 = Blockmittel ueber ratio^3 Feinzellen (Default), 0 = Punktwert am Deckungspunkt
	if(n2f_mittel==0u&&(env_u("CFD_N2F_VOLUMEN",0u)>0u||n2f_band>0u)) print_error("CFD_N2F_SCHALE_MITTEL=0 ist im VOLUMEN- und im BAND-Modus GESPERRT: Punktwert-Extraktion prueft keine Flags -- ein Deckungspunkt im Fein-Solid (Voxelisierungs-Differenz 4 vs 16 mm!) wuerde still vergiftet ins Blend laufen (Pruefagent M2).");
	if(n2f_alpha>0.0f&&n2f_band>0u) { // ---------------- BAND-Listenbauer (Heiko 2026-08-22; ERSETZT Schale/Volumen)
		// ★ RICHTIGSTELLUNG 2026-08-22 (Pruefagent-Befund 2): hier stand "redundante Zweitsicherung,
		// seit der Blend VOR boden_eq laeuft". Das ist FALSCH und waere eine Einladung gewesen, den
		// Ausschluss zu entfernen. Er ist TRAGEND: ohne ihn schriebe der Blend in Zellen, die
		// boden_eq unmittelbar danach vollstaendig ueberschreibt -- ein teilweise wirkungsloser
		// Blend (lautloser No-Op) plus zusaetzliche EsoPull-Nachbarkopplung ueber die Bandgrenze.
		// ★ _DOWN-LUECKE (Pruefagent 2026-08-22): boden_eq nutzt nz_eff = (x>=x_split) ? nz_down : nz.
		// Setzt jemand CFD_FERN_BODEN_EQ_DOWN groesser als CFD_FERN_BODEN_EQ, reicht der Bodenfix
		// stromab HOEHER als z_lo -- der Blend schriebe dann in Zellen, die boden_eq danach
		// ueberschreibt. Das Maximum beider deckt beide Faelle.
		const uint z_lo = max(1u, max(env_u("CFD_FERN_BODEN_EQ", 0u), env_u("CFD_FERN_BODEN_EQ_DOWN", 0u))+1u); // boden_eq-Band bleibt unangetastet -- TRAGEND, nicht redundant
		// ★ Pruefagent 2026-08-22 spaet (HOCH): die NURWAKE-Schiene der Freigabe war UNSICHER --
		// im NURWAKE-Arm ist d der KASTEN-Abstand, nicht der Wandabstand; wandnahe RAMPEN-Zellen
		// blieben gelistet und ueberschrieben die FERN_FACETTEN-Schicht still (N1-Klasse).
		// Bewiesen sicher ist allein wandfrei>0 UND nurwake==0 (EsoPull-Slot-Analyse verifiziert).
		if(env_u("CFD_FERN_FACETTEN",0u)>0u&&!(n2f_wandfrei>0u&&n2f_nurwake==0u)) print_error("CFD_N2F_BAND=1 mit CFD_FERN_FACETTEN>0: freigegeben ist NUR die Kombination CFD_N2F_BAND_WANDFREI>=1 ohne NURWAKE (im NURWAKE-Arm ist der Bandabstand Kasten-, nicht Wandabstand -- wandnahe Rampenzellen ueberschrieben die Facettenschicht still). Lage 1 IST definitionsgemaess die karosserienahe Zellschicht, also genau die FERN_FACETTEN-Menge -- der Blend ueberschriebe deren fi NACH apply_facette still (Pruefagent-N1-Klasse). Kombination nur mit CFD_N2F_BAND_WANDFREI>=1 oder NURWAKE freigegeben (1b, 2026-08-22).");
		// ---- SAATMENGE. HIER SITZT DIE GEFAEHRLICHSTE FALLE DES GANZEN ARMS:
		// Die FAHRBAHN ist bei z=0 flaechendeckend TYPE_S. Mit `flags&TYPE_S` als Saat waere das
		// "Band ums Fahrzeug" ein N Zellen dicker Teppich ueber cNx*cNy Zellen -- bei 4 mm sind das
		// 2,95 Mio Zellen statt ~0,85 Mio, und zwar lautlos. Der Test muss auf BEIDE Bits gehen:
		// 0x41 = TYPE_S|TYPE_X ist das Fahrzeug, exakt wie im Verdraengungs-Census.
		std::vector<uchar> dt((size_t)cNx*(size_t)cNy*(size_t)cNz, 0u); // 0 = unerreicht, 1..N = Lage d, 255 = Saat
		ulong saat=0ull; uint sx0=cNx, sx1=0u, sy0=cNy, sy1=0u, sz0=cNz, sz1=0u;
		for(uint z=0u; z<cNz; z++) for(uint y=0u; y<cNy; y++) for(uint x=0u; x<cNx; x++) {
			const ulong nn=(ulong)x+((ulong)y+(ulong)z*(ulong)cNy)*(ulong)cNx;
			if(lbm_c.flags[nn]!=(TYPE_S|TYPE_X)) continue; // EXAKTE Gleichheit wie der Verdraengungs-Census (setup.cpp) -- eine Maske faenge zusaetzlich Zellen mit weiteren Bits und die Quervergleichszahl waere wertlos
			dt[(size_t)nn]=(uchar)(n2f_nurwake>0u ? 253u : 255u); saat++; // 253 = Sperre ohne Quelleigenschaft (NUR-WAKE)
			sx0=min(sx0,x); sx1=max(sx1,x); sy0=min(sy0,y); sy1=max(sy1,y); sz0=min(sz0,z); sz1=max(sz1,z);
		}
		// ★ NUR-WAKE: das Fahrzeug traegt je nach Arm 255 (Quelle) oder 253 (nur Sperre). Jede Stelle,
		// die "ist das eine Fahrzeugzelle?" fragt, muss BEIDE sehen -- sonst faende die
		// Radstanderkennung im Nur-Wake-Arm kein Fahrzeug und der Fahrbahn-Waechter kein Fahrzeug
		// auf z=0 (er wuerde still gruenes Licht geben, wo er warnen soll).
		auto ist_fzg=[&](const uchar d){ return d==255u||d==253u; };
		// DACHSCHEITEL: kleinstes x, an dem das Fahrzeug seine groesste Hoehe erreicht. Zweiter Lauf,
		// weil sz1 erst nach dem ersten feststeht. Das ist die Vorderkante des Daches.
		uint x_dach=cNx, x_radmitte=0u;
		for(uint y=sy0; y<=sy1; y++) for(uint x=sx0; x<=sx1; x++) if(dt[(size_t)((ulong)x+((ulong)y+(ulong)sz1*(ulong)cNy)*(ulong)cNx)]>=253u) { x_dach=min(x_dach,x); break; }
		if(x_dach>=cNx) x_dach=sx0; // kein Treffer auf der obersten Ebene (duenne Anbauteile) -> Fahrzeugfront
		// HOEHENPROFIL laengs x, damit die Wahl des Wake-Kastenstarts auf Daten steht und nicht auf
		// der Annahme "hoechster Punkt = Dach". Bei diesem Fahrzeug ist der hoechste Punkt der
		// HECKFLUEGEL, nicht die Kabine -- ohne dieses Profil sieht man das nicht.
		{
			string prof; const uint schritte=12u; uint hprof[12]={0}, hx[12]={0};
			for(uint k=0u; k<schritte; k++) {
				const uint x = sx0 + (sx1-sx0)*k/(schritte-1u);
				uint hmax=0u;
				for(uint z=sz1+1u; z-->0u; ) { bool tr=false; for(uint y=sy0; y<=sy1&&!tr; y++) if(ist_fzg(dt[(size_t)((ulong)x+((ulong)y+(ulong)z*(ulong)cNy)*(ulong)cNx)])) tr=true; /* ★ B-N1: stand auf ==255u -- im Nur-Wake-Arm (Fahrzeug = 253) meldete das Profil fuer JEDES x die Hoehe 0 */ if(tr) { hmax=z; break; } }
				prof += (k?" ":"") + to_string(x) + ":" + to_string(hmax);
				hprof[k]=hmax; hx[k]=x;
			}
			// ★ ANBAUTEIL-ERKENNUNG (Heiko-Befund 2026-08-22 nachmittags): "hoechster Punkt" ist bei
			// diesem Fahrzeug NICHT das Dach. Gemessen: 158:37 171:37 184:34 196:29 209:38 -- die
			// Kabine erreicht 37 bei x=158, faellt auf 29 (Motordeckel) und der HECKFLUEGEL steigt
			// bei x=209 auf 38. CFD_N2F_BAND_WAKE_START=0 setzte den Kastenstart damit bei 91 % der
			// Fahrzeuglaenge statt bei 54 %, ohne ein Wort. Ein globales Maximum, VOR dem eine echte
			// Senke liegt, ist ein Anbauteil und kein Dach -- das wird jetzt angesagt, samt der
			// Grobzelle, die der Nutzer stattdessen einsetzen kann.
			uint k_max=0u; for(uint k=1u; k<schritte; k++) if(hprof[k]>hprof[k_max]) k_max=k;
			uint senke=0u, k_kabine=0u;
			for(uint k=0u; k<k_max; k++) { // tiefste Stelle VOR dem globalen Maximum
				if(hprof[k]>hprof[k_kabine]) k_kabine=k;
			}
			for(uint k=k_kabine; k<k_max; k++) senke = max(senke, (uint)(hprof[k_kabine]>hprof[k] ? hprof[k_kabine]-hprof[k] : 0u));
			if(k_kabine<k_max && senke>=3u)
				print_warning("N2F-BAND HOEHENPROFIL: der hoechste Punkt (x = "+to_string(hx[k_max])+", z = "+to_string(hprof[k_max])+") ist ein ANBAUTEIL, kein Dach -- davor liegt eine Senke von "+to_string(senke)+" Grobzellen (Kabinenmaximum z = "+to_string(hprof[k_kabine])+" bei x = "+to_string(hx[k_kabine])+"). CFD_N2F_BAND_WAKE_START=0 setzt den Kastenstart damit bei "+to_string((float)(100.0*(double)(hx[k_max]-sx0)/(double)max(1u,sx1-sx0)),0u)+" % der Fahrzeuglaenge statt bei "+to_string((float)(100.0*(double)(hx[k_kabine]-sx0)/(double)max(1u,sx1-sx0)),0u)+" %. Wer den Kasten AB KABINENDACH will: CFD_N2F_BAND_WAKE_START_X="+to_string(hx[k_kabine])+".");
			// RADSTAND (Heiko-Vorschlag 2026-08-22): die Raeder sind die Fahrzeugzellen auf den
			// untersten z-Ebenen. Ihr x-Histogramm hat zwei Haufen -- Vorder- und Hinterachse.
			// Der Mittelpunkt dazwischen ist ein BEGRUENDETER Kastenstart, im Unterschied zum
			// hoechsten Punkt, der bei diesem Fahrzeug zufaellig am Heckfluegel liegt.
			{
				std::vector<uint> hist(sx1-sx0+1u, 0u);
				// NUR die UNTERSTE Ebene. Erste Fassung nahm sz0 und sz0+1 -- damit dominierte der
				// FRONTSPLITTER das Histogramm und die "Vorderachse" landete 3 Grobzellen hinter der
				// Fahrzeugnase (gemessen 2026-08-22: x = 86 bei Fahrzeugfront 83). Der Splitter ist
				// flach und breit, die Reifenlatsche beruehren als einzige wirklich den Boden.
				for(uint y=sy0; y<=sy1; y++) for(uint x=sx0; x<=sx1; x++)
					if(ist_fzg(dt[(size_t)((ulong)x+((ulong)y+(ulong)sz0*(ulong)cNy)*(ulong)cNx)])) hist[x-sx0]++;
				uint vx=0u, hx=0u; const uint mitte=(sx1-sx0)/2u; uint bv=0u, bh=0u;
				for(uint k=0u; k<=mitte; k++)            if(hist[k]>bv) { bv=hist[k]; vx=sx0+k; }   // staerkster Haufen vorn
				for(uint k=mitte+1u; k<hist.size(); k++) if(hist[k]>bh) { bh=hist[k]; hx=sx0+k; }   // staerkster Haufen hinten
				x_radmitte = (bv>0u&&bh>0u) ? (vx+hx)/2u : x_dach;
				if(bv>0u&&bh>0u&&(vx-sx0<3u||sx1-hx<3u)) print_warning("N2F-BAND RADSTAND: eine erkannte Achse liegt weniger als 3 Grobzellen von der Fahrzeugkante entfernt (vorn "+to_string(vx-sx0)+", hinten "+to_string(sx1-hx)+") -- das ist eher ein Splitter oder Diffusor als ein Radaufstand. Kastenstart lieber ueber CFD_N2F_BAND_WAKE_START_X aus dem Hoehenprofil setzen.");
				print_info("N2F-BAND RADSTAND: Vorderachse bei Grobzelle x = "+to_string(vx)+" ("+to_string(bv)+" Latschzellen auf der untersten Ebene), Hinterachse x = "+to_string(hx)+" ("+to_string(bh)+"); Mitte x = "+to_string(x_radmitte)+". Als Kastenstart ueber CFD_N2F_BAND_WAKE_START=2 waehlbar.");
			}
			print_info("N2F-BAND HOEHENPROFIL (Grobzellen, x:z_max ueber alle y): "+prof+" -- hoechster Punkt bei x = "+to_string(x_dach)+" (z = "+to_string(sz1)+"), Fahrzeug x["+to_string(sx0)+".."+to_string(sx1)+"]. Wer den Wake-Kasten AB KABINENDACH will, liest den x-Wert hier ab und setzt CFD_N2F_BAND_WAKE_START_X.");
		}
		if(saat==0ull) print_error("N2F-BAND: keine einzige Fahrzeug-Grobzelle (0x41) gefunden -- Saatmenge leer, das Band waere leer. Voxelisierung des Fernfelds pruefen.");
		// Fahrbahn-Falle als HARTER Test, nicht als Kommentar. ERSTE FASSUNG WAR FALSCH (2026-08-22):
		// sie verglich die 3D-Zellzahl der Saat gegen die 2D-Fahrbahnflaeche und schlug damit am
		// richtigen Fahrzeug an -- bei 8 mm hat der Fernfeld-Wagen 146.167 Zellen, die Fahrbahnebene
		// aber nur 384x240 = 92.160. Ein Volumen ist zwangslaeufig groesser als eine Flaeche.
		// Richtig ist der Test auf der EBENE z=0: die Fahrbahn ist dort flaechendeckend, das Fahrzeug
		// beruehrt sie nur mit den Aufstandsflaechen (bei 8 mm 744 Zellen, und denen wird TYPE_X
		// ohnehin entzogen). Mehr als ein Zehntel der Ebene bedeutet: die Maske faengt die Strasse.
		ulong saat_z0=0ull;
		for(uint y=0u; y<cNy; y++) for(uint x=0u; x<cNx; x++) if(ist_fzg(dt[(size_t)((ulong)x+(ulong)y*(ulong)cNx)])) saat_z0++;
		if(saat_z0>(ulong)((double)cNx*(double)cNy*0.10)) print_error("N2F-BAND: "+to_string(saat_z0)+" Saatzellen liegen auf der Ebene z=0, das sind ueber 10 % der Fahrbahnflaeche ("+to_string(cNx)+"x"+to_string(cNy)+") -- die Maske faengt die STRASSE, nicht das Fahrzeug. Soll: flags == TYPE_S|TYPE_X (exakt).");
		// ---- WAKE-KASTEN als ZWEITE Saatmenge (Heiko 2026-08-22). Er kommt mit Gewicht 1 in die
		// Liste UND dient der Distanztransformation als Quelle -- damit laeuft der Auslauf in y, z
		// und stromab automatisch mit demselben Profil wie am Koerperband, ohne Sonderlogik.
		if(n2f_nurwake>0u&&n2f_wake==0u) print_error("CFD_N2F_BAND_NURWAKE=1 ohne CFD_N2F_BAND_WAKE=1: das Fahrzeug ist dann nur noch Sperre und es gibt GAR KEINE Saat -- die Zellliste bliebe leer und der ganze Arm ein No-Op.");
		uint wx0=0u, wx1=0u, wy0=0u, wy1=0u, wz1=0u; ulong wake_n=0ull;
		if(n2f_wake>0u) {
			const uint wake_start_x = env_u("CFD_N2F_BAND_WAKE_START_X", 0u); // 0 = aus; sonst Grobzell-x, ueberstimmt _START
			wx0 = wake_start_x>0u ? wake_start_x : (n2f_wake_start==2u ? x_radmitte : (n2f_wake_start==0u ? x_dach : sx1+1u));
			if(wake_start_x>0u) print_info("N2F-BAND WAKE: Start explizit auf Grobzelle x = "+to_string(wake_start_x)+" gesetzt (CFD_N2F_BAND_WAKE_START_X ueberstimmt CFD_N2F_BAND_WAKE_START).");
			const int x_ende = (int)(NF_OX+cex-1u) - (int)n2f_wake_abst;
			if(x_ende<=(int)wx0) print_error("N2F-BAND WAKE: Kastenstart (x = "+to_string(wx0)+") liegt hinter dem Ende (x = "+to_string(x_ende)+"). CFD_N2F_BAND_WAKE_ABSTAND ("+to_string(n2f_wake_abst)+" Grobzellen) ist groesser als der Nachlauf im Fussabdruck -- Abstand senken oder Nahfeld-Box verlaengern.");
			wx1 = (uint)x_ende; wy0 = sy0; wy1 = sy1; wz1 = sz1;
			for(uint z=z_lo; z<=wz1; z++) for(uint y=wy0; y<=wy1; y++) for(uint x=wx0; x<=wx1; x++) {
				const size_t k=(size_t)((ulong)x+((ulong)y+(ulong)z*(ulong)cNy)*(ulong)cNx);
				if(dt[k]!=0u) continue; // Fahrzeug bleibt Saat (255)
				dt[k]=254u; wake_n++;   // 254 = Wake-Kern: Quelle der Dilatation UND Listenzelle mit w = 1
			}
			print_info("N2F-BAND WAKE-CENSUS: Kasten grob x["+to_string(wx0)+".."+to_string(wx1)+"] y["+to_string(wy0)+".."+to_string(wy1)+"] z["+to_string(z_lo)+".."+to_string(wz1)+"] = "+to_string(wake_n)+" Kernzellen (Start "+(n2f_wake_start==2u?"RADSTANDMITTE x="+to_string(x_radmitte):n2f_wake_start==0u?"HOECHSTER PUNKT x="+to_string(x_dach):"HECK x="+to_string(sx1+1u))+", Fahrzeug-BBox x["+to_string(sx0)+".."+to_string(sx1)+"] y["+to_string(sy0)+".."+to_string(sy1)+"] z[..%"+to_string(sz1)+"]); frei vor dem Nahfeld-Auslass: "+to_string((int)(NF_OX+cex-1u)-(int)wx1)+" Grobzellen.");
			if(wake_n==0ull) print_error("N2F-BAND WAKE: Kasten enthaelt keine einzige freie Zelle -- Geometrie pruefen.");
			bil_x0=wx0; bil_x1=wx1; bil_y0=wy0; bil_y1=wy1; bil_z0=z_lo; bil_z1=wz1; bil_an=true;
		}
		// ---- CHEBYSHEV-DISTANZTRANSFORMATION, separabel: N Runden a drei 1-D-Dilatationen.
		// Deterministisch, keine Warteschlange. Arbeitsbereich ist die Saat-BBox um N+1 aufgeblasen
		// und aufs Gitter geklemmt -- alles ausserhalb kann per Konstruktion keine Lage <= N bekommen.
		//
		// ★ FEHLER B1, gefunden vom Pruefagenten 2026-08-22, HOCH: hier stand die BBox der
		// FAHRZEUG-Saat allein (sx0..sx1). Der Wake-Kasten ist aber eine ZWEITE Saat und reicht bis
		// wx1 = NF_OX+cex-1-abstand, also weit stromab von sx1+N+1. Dilatationsschleife UND
		// Listenschleife laufen ueber denselben Bereich -- alles im Kasten mit x > ax1 blieb auf
		// dt=254 stehen, wurde gezaehlt, im Census angesagt, in die Bilanzbox uebernommen und NIE
		// in n2f_liste_c gelegt. Kein Zaehler bemerkte es, weil ausgelassen_zlo/_ub/ausserhalb nur
		// Zellen erfassen, die die Schleife ueberhaupt besucht hat.
		// Nachgerechnet an den eigenen Protokollen: wake_heck sagte Kasten x[223..268] = 96.048
		// Kernzellen an, die Arbeitsbox endete bei x=227 -- 41 Scheiben x 58 x 36 = 85.608 Zellen
		// (89 %) waren ein lautloser No-Op. Der A/B "Start Dach gegen Heck gegen Radmitte" verglich
		// damit drei Arme, die alle bei derselben Grobzelle enden. Diese Laeufe sind entwertet.
		const uint ux0=min(sx0,(n2f_wake>0u?wx0:sx0)), ux1=max(sx1,(n2f_wake>0u?wx1:sx1)); // Vereinigung Saat A + Saat B
		const uint uy0=min(sy0,(n2f_wake>0u?wy0:sy0)), uy1=max(sy1,(n2f_wake>0u?wy1:sy1));
		const uint uz0=min(sz0,(n2f_wake>0u?z_lo:sz0)), uz1=max(sz1,(n2f_wake>0u?wz1:sz1));
		const uint ax0=(uint)max(0,(int)ux0-(int)n2f_band_n-1), ax1=min(cNx-1u, ux1+n2f_band_n+1u);
		const uint ay0=(uint)max(0,(int)uy0-(int)n2f_band_n-1), ay1=min(cNy-1u, uy1+n2f_band_n+1u);
		const uint az0=(uint)max(0,(int)uz0-(int)n2f_band_n-1), az1=min(cNz-1u, uz1+n2f_band_n+1u);
		auto belegt=[&](const uint x,const uint y,const uint z){ return dt[(size_t)((ulong)x+((ulong)y+(ulong)z*(ulong)cNy)*(ulong)cNx)]!=0u; };
		for(uint runde=1u; runde<=n2f_band_n; runde++) {
			std::vector<ulong> neu; // erst sammeln, dann setzen -- sonst frisst sich eine Runde selbst weiter
			for(uint z=az0; z<=az1; z++) for(uint y=ay0; y<=ay1; y++) for(uint x=ax0; x<=ax1; x++) {
				if(belegt(x,y,z)) continue;
				bool nachbar=false;
				for(int dz=-1; dz<=1&&!nachbar; dz++) for(int dy=-1; dy<=1&&!nachbar; dy++) for(int dx=-1; dx<=1&&!nachbar; dx++) {
					if(dx==0&&dy==0&&dz==0) continue;
					const int xx=(int)x+dx, yy=(int)y+dy, zz=(int)z+dz;
					if(xx<0||yy<0||zz<0||xx>=(int)cNx||yy>=(int)cNy||zz>=(int)cNz) continue;
					// ★ NUR-WAKE: 253 ist Sperre, keine Quelle -- sonst waere das Koerperband wieder da.
					const uchar dn2=dt[(size_t)((ulong)xx+((ulong)yy+(ulong)zz*(ulong)cNy)*(ulong)cNx)];
					if(dn2!=0u&&dn2!=253u) nachbar=true;
				}
				if(nachbar) neu.push_back((ulong)x+((ulong)y+(ulong)z*(ulong)cNy)*(ulong)cNx);
			}
			for(const ulong nn : neu) dt[(size_t)nn]=(uchar)runde;
		}
		// ---- LISTE. Reihenfolge z,y,x aufsteigend = aufsteigend im linearen Index (deterministisch,
		// Muster des Volumen-Listenbauers; keine std::map noetig).
		std::vector<ulong> lage_n(n2f_band_n,0ull), lage_nan(n2f_band_n,0ull), lage_ub(n2f_band_n,0ull), lage_te(n2f_band_n,0ull);
		ulong ausgelassen_zlo=0ull, ausgelassen_ub=0ull, ausgelassen_ub_kern=0ull, ausserhalb=0ull, ausgelassen_w0=0ull, ausgelassen_wf=0ull, ausgelassen_wf_kern=0ull;
		const int n2f_r=(int)ratio, n2f_w0=-(n2f_r/2);
		for(uint z=az0; z<=az1; z++) for(uint y=ay0; y<=ay1; y++) for(uint x=ax0; x<=ax1; x++) {
			const ulong nn=(ulong)x+((ulong)y+(ulong)z*(ulong)cNy)*(ulong)cNx;
			const uchar d=dt[(size_t)nn];
			if(d==0u||d==255u||d==253u) continue;              // unerreicht oder Fahrzeug (Saat 255 bzw. reine Sperre 253)
			const bool wake_kern = (d==254u);                  // Wake-Kasten: Lage 0, volles Gewicht
			if(z<z_lo) { ausgelassen_zlo++; continue; }
			// UNTERBODEN-Kennung: liegt in derselben (x,y)-Saeule OBERHALB eine Saatzelle, ist die
			// Zelle unter dem Fahrzeug. Billig und robust auch bei nicht-quaderfoermigem Koerper.
			bool unterboden=false;
			for(uint zz=z+1u; zz<=sz1&&!unterboden; zz++) if(ist_fzg(dt[(size_t)((ulong)x+((ulong)y+(ulong)zz*(ulong)cNy)*(ulong)cNx)])) unterboden=true;
			if(unterboden&&n2f_band_ub==0u) { ausgelassen_ub++; if(wake_kern) ausgelassen_ub_kern++; continue; } // Kern getrennt: geht ins WAKE-ABNAHME-Soll (Pruefagent-Befund 3)
			// Deckungspunkt muss im Nahfeld-Fussabdruck liegen, sonst gibt es keine feine Quelle.
			if(x<NF_OX||x>=NF_OX+cex||y<NF_OY||y>=NF_OY+cey||z>=NF_OZ+cez) { ausserhalb++; continue; }
			const uint lage = wake_kern ? 0u : (uint)d-1u;
			// ★★ ORDNUNGS-FIX 2026-08-22 abends (die WAKE-ABNAHME hat ihn gefangen, b8_1b_w3 rc=1):
			// die drei Auslass-Filter (WANDFREI Koerperband, WANDFREI Kastenkern, w<=0) standen NACH
			// n2f_marke/n2f_liste_c.push_back -- jede uebersprungene Zelle desynchronisierte die
			// Listen (liste_c laenger als gewicht/lage/liste_f). Kein abgeschlossener Lauf betroffen
			// (alle bisherigen Arme linear N=16 ohne w<=0, WANDFREI erst heute). Jetzt: ALLE Filter
			// zuerst, ALLE Pushes danach, als geschlossener Block.
			if(!wake_kern&&(uint)d<=n2f_wandfrei) { ausgelassen_wf++; continue; }   // 1b WANDFREI: Koerperband-Lagen 1..k
			if(wake_kern&&n2f_wandfrei>0u) {                                        // 1b: wandnahe Kastenkern-Zellen (Scan, s. Kommentar am Schalter)
				bool wandnah=false; const int k=(int)n2f_wandfrei;
				for(int dz=-k; dz<=k&&!wandnah; dz++) for(int dy=-k; dy<=k&&!wandnah; dy++) for(int dx2=-k; dx2<=k&&!wandnah; dx2++) {
					const int xx=(int)x+dx2, yy=(int)y+dy, zz=(int)z+dz;
					if(xx<0||yy<0||zz<0||xx>=(int)cNx||yy>=(int)cNy||zz>=(int)cNz) continue;
					if(ist_fzg(dt[(size_t)((ulong)xx+((ulong)yy+(ulong)zz*(ulong)cNy)*(ulong)cNx)])) wandnah=true;
				}
				if(wandnah) { ausgelassen_wf_kern++; continue; }
			}
			{ const float w_ = wake_kern ? 1.0f : band_w(d); if(w_<=0.0f) { ausgelassen_w0++; continue; } } // Profil-2-Nulllage (B-P2/B2)
			const uchar bo=lbm_c.flags[nn]&(TYPE_S|TYPE_E);
			if(bo==TYPE_E) lage_te[lage]++;                     // z. B. FERN_BODENKLEMME; seit c1a6843 zaehlt lage_te nur noch GELISTETE Zellen (WANDFREI/w0-gefilterte nicht mehr) -- Bedeutungswandel dokumentiert
			const bool zone_wake = n2f_wake>0u && (wake_kern || x>=wx0); // Obermenge, s. Kommentar oben
			n2f_marke.push_back((uchar)(zone_wake?1u:0u));
			n2f_liste_c.push_back(nn);
			n2f_gewicht.push_back(wake_kern ? 1.0f : band_w(d));
			n2f_lage.push_back(lage);
			lage_n[lage]++; if(unterboden) lage_ub[lage]++;
			const uint fdx=(x-NF_OX)*ratio, fdy=(y-NF_OY)*ratio, fdz=(z-NF_OZ)*ratio;
			n2f_liste_f.push_back((ulong)fdx+((ulong)fdy+(ulong)fdz*(ulong)fNy)*(ulong)fNx);
			uint fluid=0u;
			for(int dz=n2f_w0; dz<n2f_w0+n2f_r&&fluid==0u; dz++) for(int dy=n2f_w0; dy<n2f_w0+n2f_r&&fluid==0u; dy++) for(int dx=n2f_w0; dx<n2f_w0+n2f_r&&fluid==0u; dx++) {
				const int xx=(int)fdx+dx, yy=(int)fdy+dy, zz=(int)fdz+dz;
				if(xx<0||yy<0||zz<0||xx>=(int)fNx||yy>=(int)fNy||zz>=(int)fNz) continue;
				const uchar bof=lbm_f.flags[(ulong)xx+((ulong)yy+(ulong)zz*(ulong)fNy)*(ulong)fNx]&(TYPE_S|TYPE_E);
				if(bof!=TYPE_S&&bof!=TYPE_E) fluid++;           // TYPE_MS zaehlt als Fluid -- Kernel-Konvention
			}
			if(fluid==0u) { n2f_nanblocks++; lage_nan[wake_kern?0u:(uint)d-1u]++; }
		}
		// ★ BEWEIS DER MAXIMUM-EIGENSCHAFT (Heiko-Auflage 2026-08-22): jede Fernzelle darf nur das
		// HOECHSTE Gewicht aus Saat A (Fahrzeug) oder Saat B (Wake-Kasten) bekommen.
		// Das ist hier strukturell erfuellt -- die Dilatation laeuft von BEIDEN Saaten gleichzeitig,
		// eine Zelle wird in der Runde belegt, in der sie ZUERST erreicht wird, und einmal belegte
		// Zellen werden nie ueberschrieben (`if(belegt(...)) continue`). Der Abstand ist damit der
		// zur NAECHSTEN Saat, und weil band_w monoton faellt, ist das genau das Maximum.
		// Ein Strukturargument steht in diesem Baum aber unter Beweispflicht. Der Test dazu ist
		// billig und direkt: waere eine Zelle zu SPAET belegt worden (also mit zu kleinem Gewicht),
		// haette sie einen Nachbarn, dessen Lage um mindestens 2 kleiner ist -- eine Dilatation
		// schreitet je Runde genau eine Zelle weit. Findet der Test nichts, ist jede Zelle so frueh
		// belegt, wie es die naechste Saat erlaubt.
		{
			ulong verletzt=0ull, geprueft=0ull;
			for(uint z=az0; z<=az1; z++) for(uint y=ay0; y<=ay1; y++) for(uint x=ax0; x<=ax1; x++) {
				const uchar d=dt[(size_t)((ulong)x+((ulong)y+(ulong)z*(ulong)cNy)*(ulong)cNx)];
				if(d==0u||d>=253u) continue; // unerreicht, Fahrzeug-Sperre (253), Wake-Kern (254) oder Fahrzeug-Saat (255)
				geprueft++;
				uint dmin=255u;
				for(int dz=-1; dz<=1; dz++) for(int dy=-1; dy<=1; dy++) for(int dx=-1; dx<=1; dx++) {
					const int xx=(int)x+dx, yy=(int)y+dy, zz=(int)z+dz;
					if(xx<0||yy<0||zz<0||xx>=(int)cNx||yy>=(int)cNy||zz>=(int)cNz) continue;
					const uchar dn=dt[(size_t)((ulong)xx+((ulong)yy+(ulong)zz*(ulong)cNy)*(ulong)cNx)];
					if(dn==0u) continue;
					if(dn==253u) continue;                        // Sperre ist KEINE Saat (Nur-Wake-Arm)
					dmin = min(dmin, (dn>=254u) ? 0u : (uint)dn); // beide Saaten zaehlen als Abstand 0
				}
				if(dmin+1u<(uint)d) verletzt++;
			}
			if(verletzt>0ull) print_error("N2F-BAND MAXIMUM-BEWEIS GESCHEITERT: "+to_string(verletzt)+" von "+to_string(geprueft)+" Bandzellen tragen ein zu KLEINES Gewicht -- sie haben einen Nachbarn, der mindestens zwei Lagen naeher an einer Saat liegt. Die Distanztransformation gibt damit nicht das Maximum aus Saat A und Saat B zurueck.");
			// ★ FEHLER B2 (Pruefagent 2026-08-22, HOCH): der Teil oben zeigt LOKALE KONSISTENZ --
			// keine gelabelte Zelle ist zu spaet belegt. Er zeigt NICHT die VOLLSTAENDIGKEIT: dass
			// jede Zelle, die ein Gewicht bekommen muesste, auch eines hat. Genau darum konnte er B1
			// nicht finden -- `if(d==0u||d>=254u) continue` uebersprang sowohl die nie gelisteten
			// Kastenzellen als auch deren unerreichte Nachbarn. Ein Beweis, der den Fehler
			// strukturell nicht sehen kann, ist keiner. Der Gegentest: eine Zelle mit d==0
			// (unerreicht) darf KEINEN Nachbarn mit d < N haben -- sonst haette die Dilatation sie
			// erreichen muessen und der Arbeitsbereich ist zu klein.
			ulong luecke=0ull;
			for(uint z=az0; z<=az1; z++) for(uint y=ay0; y<=ay1; y++) for(uint x=ax0; x<=ax1; x++) {
				if(dt[(size_t)((ulong)x+((ulong)y+(ulong)z*(ulong)cNy)*(ulong)cNx)]!=0u) continue;
				bool nah=false;
				for(int dz=-1; dz<=1&&!nah; dz++) for(int dy=-1; dy<=1&&!nah; dy++) for(int dx=-1; dx<=1&&!nah; dx++) {
					const int xx=(int)x+dx, yy=(int)y+dy, zz=(int)z+dz;
					if(xx<0||yy<0||zz<0||xx>=(int)cNx||yy>=(int)cNy||zz>=(int)cNz) continue;
					const uchar dn=dt[(size_t)((ulong)xx+((ulong)yy+(ulong)zz*(ulong)cNy)*(ulong)cNx)];
					if(dn!=0u&&dn!=253u&&(dn>=254u||(uint)dn<n2f_band_n)) nah=true; // Saat oder Lage < N: haette weitergereicht werden muessen (253 = Sperre, keine Saat)
				}
				if(nah) luecke++;
			}
			if(luecke>0ull) print_error("N2F-BAND VOLLSTAENDIGKEITS-BEWEIS GESCHEITERT: "+to_string(luecke)+" unerreichte Zellen haben einen Nachbarn mit Lage < "+to_string(n2f_band_n)+" oder eine Saat als Nachbarn -- die Dilatation haette sie erreichen muessen. Der Arbeitsbereich ist zu klein (das war Fehler B1).");
			// ★ Iron Rule 8: der Kasten beweist seine Wirkung im Binary. Angesagte Kernzellen gegen
			// tatsaechlich GELISTETE. Ohne diesen Test war B1 ein lautloser Teil-No-Op.
			// ★ WIRKNACHWEIS NUR-WAKE (Iron Rule 8): im Nur-Wake-Arm darf KEINE gelistete Zelle einen
			// Fahrzeugnachbarn haben, ohne ueber den Kasten erreicht worden zu sein. Direkt pruefbar:
			// keine Lage-0-Zelle (Abstand 1) darf an das Fahrzeug grenzen, ohne Kastenzelle zu sein.
			if(n2f_nurwake>0u) {
				ulong am_fzg=0ull;
				for(ulong i=0ull; i<(ulong)n2f_liste_c.size(); i++) {
					const ulong nn=n2f_liste_c[i];
					if(dt[(size_t)nn]==254u) continue;                  // Kastenzelle: darf ueberall liegen
					const uint x=(uint)(nn%(ulong)cNx), y=(uint)((nn/(ulong)cNx)%(ulong)cNy), z=(uint)(nn/((ulong)cNx*(ulong)cNy));
					bool nb=false;
					for(int dz=-1; dz<=1&&!nb; dz++) for(int dy=-1; dy<=1&&!nb; dy++) for(int dx=-1; dx<=1&&!nb; dx++) {
						const int xx=(int)x+dx, yy=(int)y+dy, zz=(int)z+dz;
						if(xx<0||yy<0||zz<0||xx>=(int)cNx||yy>=(int)cNy||zz>=(int)cNz) continue;
						if(dt[(size_t)((ulong)xx+((ulong)yy+(ulong)zz*(ulong)cNy)*(ulong)cNx)]==253u) nb=true;
					}
					if(nb) am_fzg++;
				}
				// ★ B-N2 (Pruefagent 2026-08-22): der Test hier war als Sensor INVERTIERT und ohne
				// Kriterium. Er suchte 253-Nachbarn, und 253 existiert nur, WENN NURWAKE an ist --
				// im Fehlerfall (Koerperband doch noch aktiv) haette er 0 gemeldet, also die
				// beruhigendste Zahl. Die Behauptung "im Arm MIT Koerperband waeren es alle N"
				// konnte der Code gar nicht erzeugen. Ersetzt durch eine echte Ist=Soll-Abnahme:
				// im Nur-Wake-Arm MUSS jede gelistete Nicht-Kastenzelle innerhalb von N Lagen um
				// den Kasten liegen. Eine einzige Zelle weiter stromauf beweist, dass die
				// Fahrzeug-Saat doch noch reicht.
				ulong zu_weit=0ull;
				for(ulong i=0ull; i<(ulong)n2f_liste_c.size(); i++) {
					const ulong nn=n2f_liste_c[i];
					if(dt[(size_t)nn]==254u) continue;
					const uint x=(uint)(nn%(ulong)cNx);
					if(wx0>n2f_band_n && x+n2f_band_n < wx0) zu_weit++;
				}
				if(zu_weit>0ull) print_error("N2F-BAND NUR-WAKE ABNAHME GESCHEITERT: "+to_string(zu_weit)+" gelistete Rampenzellen liegen weiter als "+to_string(n2f_band_n)+" Lagen stromauf des Kastens (x < "+to_string(wx0-n2f_band_n)+"). Im Nur-Wake-Arm kann die Dilatation dorthin nicht reichen -- die Fahrzeug-Saat ist NICHT abgeschaltet.");
				else print_info("N2F-BAND NUR-WAKE ABNAHME: 0 gelistete Rampenzellen stromauf von x = "+to_string(wx0>n2f_band_n?wx0-n2f_band_n:0u)+" (Ist == Soll) -- die Rueckkopplung geht nachweislich nur vom Kasten aus. Davon "+to_string(am_fzg)+" Rampenzellen grenzen an das Fahrzeug; das ist erlaubt (der Kasten reicht dorthin), aber es sind ueberschriebene karosserienahe Zellen.");
				// ★ B-N3: die eigentliche Frage, die der Commit unbeantwortet liess -- WIEVIEL
				// Karosserie ueberschreibt der Kasten selbst? Der Kasten startet bei wx0 = x_dach
				// und ueberlappt die Fahrzeug-BBox; seine Kernzellen tragen w = 1,0. Solange das
				// gilt, ist Fx_far AUCH in diesem Arm kein sauberes Kriterium.
				ulong kern_am_fzg=0ull;
				for(const ulong nn : n2f_liste_c) {
					if(dt[(size_t)nn]!=254u) continue;
					const uint x=(uint)(nn%(ulong)cNx), y=(uint)((nn/(ulong)cNx)%(ulong)cNy), z=(uint)(nn/((ulong)cNx*(ulong)cNy));
					bool nb=false;
					for(int dz=-1; dz<=1&&!nb; dz++) for(int dy=-1; dy<=1&&!nb; dy++) for(int dx=-1; dx<=1&&!nb; dx++) {
						const int xx=(int)x+dx, yy=(int)y+dy, zz=(int)z+dz;
						if(xx<0||yy<0||zz<0||xx>=(int)cNx||yy>=(int)cNy||zz>=(int)cNz) continue;
						if(ist_fzg(dt[(size_t)((ulong)xx+((ulong)yy+(ulong)zz*(ulong)cNy)*(ulong)cNx)])) nb=true;
					}
					if(nb) kern_am_fzg++;
				}
				if(kern_am_fzg>0ull) print_warning("N2F-BAND NUR-WAKE: "+to_string(kern_am_fzg+am_fzg)+" ueberschriebene Zellen grenzen an das Fahrzeug ("+to_string(kern_am_fzg)+" davon KASTENKERN mit vollem Gewicht). Der Kasten startet bei x = "+to_string(wx0)+" und ueberlappt die Fahrzeug-BBox x["+to_string(sx0)+".."+to_string(sx1)+"] um "+to_string(sx1>=wx0?sx1-wx0+1u:0u)+" Grobzellen. FOLGE: update_force_field liest dort die Verteilungen des Blends, Fx_far ist AUCH in diesem Arm kein sauberes Kriterium. Wer Fx_far zurueckwill, setzt CFD_N2F_BAND_WAKE_START=1 (Kastenstart am Heck) oder CFD_N2F_BAND_WAKE_START_X hinter sx1 = "+to_string(sx1)+".");
			}
			if(n2f_wake>0u) {
				ulong kern_gelistet=0ull;
				for(const ulong nn : n2f_liste_c) if(dt[(size_t)nn]==254u) kern_gelistet++;
				const ulong wake_soll = wake_n - ausgelassen_wf_kern - ausgelassen_ub_kern; // minus WANDFREI und minus Unterboden-Filter (Pruefagent-Befund 3: im UNTERBODEN=0-Arm brach die Abnahme sonst faelschlich ab) // 1b: wandnahe Kernzellen bewusst nicht gelistet
				if(kern_gelistet!=wake_soll) print_error("N2F-BAND WAKE ABNAHME GESCHEITERT: Census-Soll (Kernzellen minus WANDFREI) "+to_string(wake_soll)+" an, gelistet sind "+to_string(kern_gelistet)+" ("+to_string((float)(100.0*(double)kern_gelistet/(double)max(1ull,wake_n)),1u)+" %). Die Differenz ist ein LAUTLOSER No-Op -- genau Fehler B1 vom 2026-08-22.");
				else print_info("N2F-BAND WAKE ABNAHME: "+to_string(kern_gelistet)+" Kernzellen gelistet == Soll ("+to_string(wake_n)+" Census minus "+to_string(ausgelassen_wf_kern)+" WANDFREI).");
			}
			if(verletzt==0ull&&luecke==0ull) print_info("N2F-BAND MAXIMUM-BEWEIS: "+to_string(geprueft)+" Bandzellen geprueft, 0 Verletzungen; VOLLSTAENDIGKEIT: 0 unerreichte Zellen mit zu nahem Nachbarn -- jede Zelle traegt das HOECHSTE Gewicht aus Fahrzeug-Saat und Wake-Kasten (Abstand zur naechsten Saat, band_w ist monoton nicht-steigend).");
		}
		if(n2f_liste_c.empty()) print_error("N2F-BAND: Zellliste leer -- kein Bandkandidat hat Fussabdruck, z_lo und Unterboden-Maske ueberlebt.");
		n2f_w_band = band_w(n2f_lage_aussen+1u); // Gewicht der WAECHTERLAGE (bei Profil 2: N-1. Lage; band_w(N) waere 0 und machte die Schwelle wirkungslos)
		// ---- CENSUS (Iron Rule: Diagnostik gehoert in den Code)
		print_info("N2F-BAND SAAT-CENSUS: "+to_string(saat)+" Fahrzeug-Grobzellen (0x41), davon "+to_string(saat_z0)+" auf z=0; Quervergleich: der Verdraengungs-Census oben zaehlt dieselbe Menge VOR dem Entzug von TYPE_X an den Aufstandsflaechen -- die Differenz MUSS die Kontaktflaechen-Zahl sein. BBox x["+to_string(sx0)+".."+to_string(sx1)+"] y["+to_string(sy0)+".."+to_string(sy1)+"] z["+to_string(sz0)+".."+to_string(sz1)+"]; Arbeitsbox x["+to_string(ax0)+".."+to_string(ax1)+"] y["+to_string(ay0)+".."+to_string(ay1)+"] z["+to_string(az0)+".."+to_string(az1)+"]; Metrik Chebyshev (an Koerperkanten diagonal bis "+to_string((float)n2f_band_n*1.732f,1u)+" Zellen dick -- bekannt, Ersatz waere Chamfer-3-4-5).");
		ulong ges=0ull; for(uint k=0u; k<n2f_band_n; k++) ges+=lage_n[k];
		print_info("N2F-BAND ZELL-CENSUS: "+to_string(ges)+" Zellen in "+to_string(n2f_band_n)+" Lagen; ausgelassen: "+to_string(ausgelassen_zlo)+" im boden_eq-Band, "+to_string(ausgelassen_ub)+" Unterboden (Vergleichsarm), "+to_string(ausserhalb)+" ausserhalb des Nahfeld-Fussabdrucks, "+to_string(ausgelassen_w0)+" mit Gewicht 0 (Profil-2-Nulllage, nicht gelistet), "+to_string(ausgelassen_wf)+" WANDFREI (Koerperband-Lagen 1.."+to_string(n2f_wandfrei)+" zurueckgezogen, dazu "+to_string(ausgelassen_wf_kern)+" wandnahe KASTENKERN-Zellen -- sonst waere sie im EQ-Arm eine fneq-Loeschschale); fluidleere Bloecke (NaN-Skip im Blend): "+to_string(n2f_nanblocks)+".");
		for(uint k=0u; k<n2f_band_n; k++) print_info("N2F-BAND LAGEN-CENSUS: Lage "+to_string(k)+" (Abstand "+to_string(k+1u)+(k+1u<=n2f_wandfrei?string(", WANDFREI -- nicht gelistet"):(", w = "+to_string(band_w(k+1u),3u)+", a = "+to_string(n2f_alpha*band_w(k+1u),3u)))+"): "+to_string(lage_n[k])+" Zellen, davon Unterboden "+to_string(lage_ub[k])+", fluidleer "+to_string(lage_nan[k])+", TYPE_E (Blend-Skip) "+to_string(lage_te[k])+(k==n2f_lage_aussen?" [WAECHTER-LAGE]":""));
		if(lage_n[0]>0ull&&lage_nan[0]*2ull>lage_n[0]) print_warning("N2F-BAND: Lage 0 (staerkstes Gewicht) ist zu ueber 50 % fluidleer -- der Arm wirkt deutlich schwaecher, als sein alpha behauptet. Ursache ist die Voxelisierungs-Differenz fein/grob; im Verdikt mit angeben.");
		if(lage_n[n2f_lage_aussen]==0ull) print_error("N2F-BAND: die AEUSSERSTE Lage ist leer -- Waechter, Negations-Nachweis und Kipp-Kriterium haetten keine einzige Zelle.");
		// ---- ABSTAND zu den getriebenen Entnahmeebenen (Zirkularitaetsschutz, Muster Schale)
		int mnx=(int)cNx, mxx=-1, mny=(int)cNy, mxy=-1, mxz=-1;
		for(const ulong nn : n2f_liste_c) {
			const int x=(int)(nn%(ulong)cNx), y=(int)((nn/(ulong)cNx)%(ulong)cNy), z=(int)(nn/((ulong)cNx*(ulong)cNy));
			mnx=min(mnx,x); mxx=max(mxx,x); mny=min(mny,y); mxy=max(mxy,y); mxz=max(mxz,z);
		}
		const int d_xm=mnx-(int)NF_OX, d_ym=mny-(int)NF_OY, d_yp=(int)(NF_OY+cey-1u)-mxy, d_zp=(int)(NF_OZ+cez-1u)-mxz;
		const int d_min=min(min(d_xm,d_ym),min(d_yp,d_zp));
		print_info("N2F-BAND: Abstand zu den getriebenen Entnahmeebenen (Grobzellen): x- "+to_string(d_xm)+", y- "+to_string(d_ym)+", y+ "+to_string(d_yp)+", z+ "+to_string(d_zp)+" (Soll >= 2, komfortabel >= 4; x+ ist Druckauslass und wird NICHT getrieben -- dort gilt die Regel nicht).");
		if(d_min<2) print_error("N2F-BAND kommt einer getriebenen Entnahmeebene naeher als 2 Grobzellen -- die Rueckkopplung schriebe direkt in die Quelle der Hinkopplung. Nahfeld-Box verbreitern (CFD_NEAR_LY/LZ/NEAR_VOR_MM) oder CFD_N2F_BAND_N senken.");
		else if(d_min<4) print_warning("N2F-BAND naeher als 4 Grobzellen an einer getriebenen Entnahmeebene -- eng; Kasten oder Bandbreite pruefen.");
		print_info("N2F-BAND TRANSFER: je Kopplungsfenster 2 x 12 Bytes/Zelle = "+to_string((float)((ulong)n2f_liste_c.size()*24ull)/1048576.0f,2u)+" MB; Geraetepuffer je Domaene "+to_string((float)((ulong)n2f_liste_c.size()*36ull)/1048576.0f,2u)+" MB.");
	}
	else if(n2f_alpha>0.0f&&n2f_volumen>0u) { // ---------------- VOLUMEN-Listenbauer (Heiko-Bild; ERSETZT den Schalen-Listenbauer)
		const uint z_lo = max(1u, max(env_u("CFD_FERN_BODEN_EQ", 0u), env_u("CFD_FERN_BODEN_EQ_DOWN", 0u))+1u); // ★ B1 2026-08-22 abends: _DOWN-Luecke wie im Band-Bauer geschlossen -- boden_eq nutzt nz_eff=(x>=x_split)?nz_down:nz und laeuft NACH dem Blend // Ausschluss wie gehabt: das boden_eq-Band (z=1..N) bleibt unangetastet; Fahrbahn z- offen (kein z--Deckel)
		if(env_u("CFD_FERN_FACETTEN",0u)>0u) print_error("CFD_N2F_VOLUMEN=1 mit CFD_FERN_FACETTEN>0: der Rot-Kern enthaelt die karosserienahen FERN_FACETTEN-Zellen -- der Blend ueberschriebe deren fi NACH apply_facette still (Pruefagent-N1-Klasse). Kombination nicht freigegeben.");
		const uint sz_rand = max(1u, (uint)floor(0.1f/dx_c+0.5f)); // 100 mm auf dx_c gerastert (wie n2f_rand): Rot-Aufweitung UND Schutzzonen-Breite
		// ROT-Box: Fahrzeug-BBox + sz_rand; innerhalb Gewichtsfeld konstant 1,0 (alpha traegt die Staerke); z- offen bis z_lo
		const uint rx0=(uint)fmax(0.0f, veh_c->pmin.x-(float)sz_rand), rx1=(uint)fmin((float)cNx-1.0f, veh_c->pmax.x+(float)sz_rand);
		const uint ry0=(uint)fmax(0.0f, veh_c->pmin.y-(float)sz_rand), ry1=(uint)fmin((float)cNy-1.0f, veh_c->pmax.y+(float)sz_rand);
		const uint rz1=(uint)fmin((float)cNz-1.0f, veh_c->pmax.z+(float)sz_rand);
		if(rz1<z_lo) print_error("N2F-Volumen: Rot-Oberkante liegt unter dem boden_eq-Band (rz1 < z_lo) -- kein Rot-Kern moeglich.");
		// GRUEN-Box: Nahfeld-Fussabdruck MINUS sz_rand nach innen; die Schutzzone Gruen->Fussabdruck-Rand bleibt zellfrei
		if(cex<=2u*sz_rand||cey<=2u*sz_rand||cez<=sz_rand+1u) print_error("N2F-Volumen: Nahfeld-Fussabdruck ("+to_string(cex)+"x"+to_string(cey)+"x"+to_string(cez)+" Grobzellen) ist zu klein fuer die Schutzzonen-Breite "+to_string(sz_rand)+" -- keine Gruen-Box moeglich.");
		const uint gx0=NF_OX+sz_rand, gx1=NF_OX+cex-1u-sz_rand;
		const uint gy0=NF_OY+sz_rand, gy1=NF_OY+cey-1u-sz_rand;
		const uint gz1=NF_OZ+cez-1u-sz_rand;
		// Rampenbreiten Rot->Gruen je Achse/Seite (Grobzellen); Gewicht faellt darueber linear 1,0 -> 0,0
		const int q_xm=(int)rx0-(int)gx0, q_xp=(int)gx1-(int)rx1, q_ym=(int)ry0-(int)gy0, q_yp=(int)gy1-(int)ry1, q_zp=(int)gz1-(int)rz1;
		print_info("N2F-Volumen Rampenbreiten (Grobzellen): x- "+to_string(q_xm)+" / x+ "+to_string(q_xp)+" / y- "+to_string(q_ym)+" / y+ "+to_string(q_yp)+" / z+ "+to_string(q_zp)+"."); // Pruefagent M1
		{ const int qs[5]={q_xm,q_xp,q_ym,q_yp,q_zp}; const char* qn[5]={"x-","x+","y-","y+","z+"}; for(int qi=0;qi<5;qi++) if(qs[qi]>=1&&qs[qi]<=10) print_warning(string("N2F-Volumen: Seite ")+qn[qi]+" hat nur "+to_string(qs[qi])+" Rampenstufen -- kleinste Stufe >= 0,1, dort KEIN w<0,1-Aussenband (Waechter/Negation ohne Referenz auf dieser Seite; Pruefagent M1)."); }
		if(q_xm<1||q_xp<1||q_ym<1||q_yp<1||q_zp<1) print_error("N2F-Volumen: Rot-Box beruehrt oder ueberragt die Gruen-Box (Rampenbreiten x- "+to_string(q_xm)+" / x+ "+to_string(q_xp)+" / y- "+to_string(q_ym)+" / y+ "+to_string(q_yp)+" / z+ "+to_string(q_zp)+" Zellen) -- keine Rampe moeglich; Nahfeld-Box (CFD_NEAR_LX/LY/LZ) vergroessern.");
		// Zellschleife ueber die GRUEN-Box in aufsteigender Indexreihenfolge (deterministisch wie der
		// Schalen-Listenbauer): w = 1 - max ueber Achsen(Abstand ausserhalb Rot / Rampenbreite), auf
		// [0;1] geklemmt; w<=0 (Gruen-Rand und alles dahinter) kommt NICHT in die Liste -- die Rampe
		// erreicht 0, bevor irgendeine Kopplungs-/Entnahmeebene beruehrt wird. TYPE_S/TYPE_E filtert
		// der Kernel (deshalb zaehlt der Wirkpfad-Nachweis ehrlich Listenzahl minus Solid-Skips).
		ulong cen_kern=0ull, cen_rampe=0ull;
		float wmin_pos=1.0f;
		for(uint z=z_lo; z<=gz1; z++) for(uint y=gy0; y<=gy1; y++) for(uint x=gx0; x<=gx1; x++) {
			float fab=0.0f; // normierter Chebyshev-artiger Abstand ausserhalb Rot (z- traegt NICHT bei: Volumen endet unten am z_lo)
			if(x<rx0) fab=fmax(fab,(float)(rx0-x)/(float)q_xm); else if(x>rx1) fab=fmax(fab,(float)(x-rx1)/(float)q_xp);
			if(y<ry0) fab=fmax(fab,(float)(ry0-y)/(float)q_ym); else if(y>ry1) fab=fmax(fab,(float)(y-ry1)/(float)q_yp);
			if(z>rz1) fab=fmax(fab,(float)(z-rz1)/(float)q_zp);
			const float w = 1.0f-fmin(1.0f,fab);
			if(w<=0.0f) continue;
			n2f_liste_c.push_back((ulong)x+((ulong)y+(ulong)z*(ulong)cNy)*(ulong)cNx);
			n2f_gewicht.push_back(w);
			if(w==1.0f) cen_kern++; else { cen_rampe++; wmin_pos=fmin(wmin_pos,w); }
			const uint fdx=(x-NF_OX)*ratio, fdy=(y-NF_OY)*ratio, fdz=(z-NF_OZ)*ratio; // Deckungspunkt (fein = (grob-NF_O)*ratio)
			n2f_liste_f.push_back((ulong)fdx+((ulong)fdy+(ulong)fdz*(ulong)fNy)*(ulong)fNx);
		}
		if(n2f_liste_c.empty()) print_error("N2F-Volumen: Zellliste leer (Gruen-Box ohne Zellen oberhalb z_lo?).");
		if(cen_kern==0ull) print_error("N2F-Volumen: Rot-Kern leer (w=1 nirgends) -- Rot-Box pruefen.");
		if(cen_rampe==0ull) print_error("N2F-Volumen: Rampe leer (alle Rampenbreiten 1 Zelle?) -- das Volumen fiele ohne Uebergang auf 0.");
		// AUSSENBAND (Waechter-Referenzflaeche = die AEUSSERSTE Gewichtsstufe): alle Zellen mit
		// w < 0,1 -> lage 1, Rest lage 0. Fallback bei sehr schmaler Rampe (kein w < 0,1 vorhanden):
		// die kleinste vorkommende Gewichtsstufe wird zum Aussenband.
		float band_schwelle = 0.1f;
		if(wmin_pos>=0.1f) { band_schwelle = wmin_pos*(1.0f+1e-6f); print_warning("N2F-Volumen: keine Rampenzelle mit w < 0,1 (kleinstes Rampengewicht "+to_string(wmin_pos,3u)+") -- das Aussenband faellt auf die kleinste Gewichtsstufe zurueck."); }
		ulong band_n=0ull;
		n2f_lage.resize(n2f_liste_c.size(), 0u);
		for(ulong i=0ull; i<(ulong)n2f_liste_c.size(); i++) if(n2f_gewicht[i]<band_schwelle) { n2f_lage[i]=1u; band_n++; n2f_w_band=fmax(n2f_w_band, n2f_gewicht[i]); }
		if(band_n==0ull) print_error("N2F-Volumen: Aussenband leer -- Waechter/Negations-Nachweis/Kipp haetten keine Referenzflaeche.");
		// Census der fluidleeren Bloecke (Extract schreibt dort den NaN-Marker, der Blend ueberspringt
		// sie per Bit-Test) -- gesamt und getrennt fuers Aussenband (Waechter-Gueltigkeit).
		ulong band_nan=0ull;
		if(n2f_mittel==1u) {
			const int n2f_r=(int)ratio, n2f_w0=-(n2f_r/2); // Fenster-Konvention wie im Kernel: [-r/2, r-r/2)
			for(ulong i=0ull; i<(ulong)n2f_liste_c.size(); i++) {
				const ulong ff = n2f_liste_f[i];
				const uint fdx=(uint)(ff%(ulong)fNx), fdy=(uint)((ff/(ulong)fNx)%(ulong)fNy), fdz=(uint)(ff/((ulong)fNx*(ulong)fNy));
				uint fluid=0u;
				for(int dz=n2f_w0; dz<n2f_w0+n2f_r&&fluid==0u; dz++) for(int dy=n2f_w0; dy<n2f_w0+n2f_r&&fluid==0u; dy++) for(int dx=n2f_w0; dx<n2f_w0+n2f_r&&fluid==0u; dx++) {
					const int xx=(int)fdx+dx, yy=(int)fdy+dy, zz=(int)fdz+dz;
					if(xx<0||yy<0||zz<0||xx>=(int)fNx||yy>=(int)fNy||zz>=(int)fNz) continue;
					const uchar bo = lbm_f.flags[(ulong)xx+((ulong)yy+(ulong)zz*(ulong)fNy)*(ulong)fNx]&(TYPE_S|TYPE_E);
					if(bo!=TYPE_S&&bo!=TYPE_E) fluid++; // TYPE_MS (S|E) zaehlt als Fluid -- gleiche Konvention wie der Kernel
				}
				if(fluid==0u) { n2f_nanblocks++; if(n2f_lage[i]==1u) band_nan++; }
			}
		}
		// ABSTANDS-CHECK (harter Fehler): die aeusserste Listenzelle muss von JEDER getriebenen Ebene
		// (Fussabdruck-Rand x-, x+, y-, y+, z+) mindestens die Schutzzonen-Breite entfernt sein.
		int mnx=(int)cNx, mxx=-1, mny=(int)cNy, mxy=-1, mxz=-1;
		for(ulong i=0ull; i<(ulong)n2f_liste_c.size(); i++) {
			const ulong nn = n2f_liste_c[i];
			const int x=(int)(nn%(ulong)cNx), y=(int)((nn/(ulong)cNx)%(ulong)cNy), z=(int)(nn/((ulong)cNx*(ulong)cNy));
			mnx=min(mnx,x); mxx=max(mxx,x); mny=min(mny,y); mxy=max(mxy,y); mxz=max(mxz,z);
		}
		if(mnx<(int)NF_OX||mxx>(int)(NF_OX+cex-1u)||mny<(int)NF_OY||mxy>(int)(NF_OY+cey-1u)||mxz>(int)(NF_OZ+cez-1u))
			print_error("N2F-Volumen ragt aus dem Nahfeld-Fussabdruck -- Deckungspunkte waeren undefiniert (Konstruktionsfehler).");
		const int d_xm=mnx-(int)NF_OX, d_xp=(int)(NF_OX+cex-1u)-mxx, d_ym=mny-(int)NF_OY, d_yp=(int)(NF_OY+cey-1u)-mxy, d_zp=(int)(NF_OZ+cez-1u)-mxz;
		const int d_min = min(min(min(d_xm,d_xp),min(d_ym,d_yp)),d_zp);
		print_info("N2F-Volumen: Abstand der aeussersten Listenzelle zu den Kopplungs-/Entnahmeebenen (Grobzellen): x- "+to_string(d_xm)+", x+ "+to_string(d_xp)+", y- "+to_string(d_ym)+", y+ "+to_string(d_yp)+", z+ "+to_string(d_zp)+" (Soll >= Schutzzonen-Breite "+to_string(sz_rand)+").");
		if(d_min<(int)sz_rand) print_error("N2F-Volumen: aeusserste Listenzelle naeher als die Schutzzonen-Breite ("+to_string(sz_rand)+" Grobzellen) an einer getriebenen Ebene -- die Rueckkopplung schriebe in die Naehe der Quelle der Hinkopplung.");
		// CENSUS + Transfer-Ansage
		print_info("N2F-Volumen ZELL-CENSUS: Rot-Box x["+to_string(rx0)+".."+to_string(rx1)+"] y["+to_string(ry0)+".."+to_string(ry1)+"] z["+to_string(z_lo)+".."+to_string(rz1)+"] (BBox+"+to_string(sz_rand)+"), Gruen-Box x["+to_string(gx0)+".."+to_string(gx1)+"] y["+to_string(gy0)+".."+to_string(gy1)+"] z["+to_string(z_lo)+".."+to_string(gz1)+"] (Fussabdruck-"+to_string(sz_rand)+"); gesamt "+to_string((ulong)n2f_liste_c.size())+" Zellen = Rot-Kern (w=1) "+to_string(cen_kern)+" + Rampe "+to_string(cen_rampe)+"; Aussenband (w < "+to_string(band_schwelle,3u)+", Waechter-Referenz) "+to_string(band_n)+" Zellen, davon fluidleer "+to_string(band_nan)+"; fluidleere Bloecke gesamt (NaN-Skip im Blend): "+to_string(n2f_nanblocks)+".");
		print_info("N2F-Volumen TRANSFER: je Kopplungsfenster 2 x 12 Bytes/Zelle (Nahfeld-Extract-Read + Fernfeld-Upload) = "+to_string((float)((ulong)n2f_liste_c.size()*24ull)/1048576.0f,2u)+" MB; Geraetepuffer je Domaene siehe alloc-Ansage.");
		if(n2f_nanblocks==(ulong)n2f_liste_c.size()) print_error("N2F-Volumen: ALLE Bloecke fluidleer -- der Blend waere ein vollstaendiger No-Op.");
		if(band_n>0ull&&band_n==band_nan) print_error("N2F-Volumen: das AUSSENBAND ist komplett fluidleer -- Waechter/Negations-Nachweis/Kipp haetten keine einzige gueltige Zelle.");
	}
	else if(n2f_alpha>0.0f) {
		const uint z_lo = max(1u, max(env_u("CFD_FERN_BODEN_EQ", 0u), env_u("CFD_FERN_BODEN_EQ_DOWN", 0u))+1u); // ★ B1 2026-08-22 abends: _DOWN-Luecke wie im Band-Bauer geschlossen -- boden_eq nutzt nz_eff=(x>=x_split)?nz_down:nz und laeuft NACH dem Blend // boden_eq-Band (z=1..N) unangetastet
		const bool m_xm = env_u("CFD_N2F_SCHALE_XMINUS", 0u)>0u;
		const bool m_xp = env_u("CFD_N2F_SCHALE_XPLUS", 1u)>0u;
		std::map<ulong, std::pair<float,uint>> smap; ulong roh[5] = {0ull,0ull,0ull,0ull,0ull}; // Zelle -> (Gewicht, Lage)
		auto sammle = [&](const uint f, const uint x_a, const uint x_b, const uint y_a, const uint y_b, const uint z_a, const uint z_b, const float w, const uint lage) {
			for(uint z=z_a; z<=z_b; z++) for(uint y=y_a; y<=y_b; y++) for(uint x=x_a; x<=x_b; x++) {
				roh[f]++;
				const ulong nn = (ulong)x+((ulong)y+(ulong)z*(ulong)cNy)*(ulong)cNx;
				const auto it = smap.find(nn);
				if(it==smap.end()) smap.emplace(nn, std::make_pair(w, lage));
				else if(w>it->second.first) it->second = std::make_pair(w, lage); // max-Gewicht gewinnt (Plan-Vorgabe)
			}
		};
		uint bx0=0u, bx1=0u, by0=0u, by1=0u, bz1=0u; // Aussenkasten (Census-Print + bz1-Check)
		uint n2f_rand = 0u, n2f_o0k = 0u; // nur der jeweils aktive Arm belegt seinen Wert (Census-Print)
		if(n2f_lagen==0u) { // ---- KONTROLLARM: Altverhalten (uniforme Doppellage, Gewicht 1,0)
			n2f_rand = max(1u, (uint)floor(0.1f/dx_c+0.5f));
			if(env_u("CFD_FERN_FACETTEN",0u)>0u&&n2f_rand<2u) print_error("N2F-Schale mit rand<2 traefe FERN_FACETTEN-Zellen (Blend wuerde deren fi NACH apply_facette still ueberschreiben; Pruefagent N1). Alter Guard -- gilt nur im Kontrollarm LAGEN=0.");
			bx0=(uint)fmax(0.0f, veh_c->pmin.x-(float)n2f_rand); bx1=(uint)fmin((float)cNx-1.0f, veh_c->pmax.x+(float)n2f_rand);
			by0=(uint)fmax(0.0f, veh_c->pmin.y-(float)n2f_rand); by1=(uint)fmin((float)cNy-1.0f, veh_c->pmax.y+(float)n2f_rand);
			bz1=(uint)fmin((float)cNz-1.0f, veh_c->pmax.z+(float)n2f_rand);
			if(bz1<z_lo) print_error("N2F-Schale: Kasten-Oberkante liegt unter dem boden_eq-Band (bz1 < z_lo) -- keine Schalenzelle moeglich.");
			if(m_xm) { sammle(0u, bx0, bx0, by0, by1, z_lo, bz1, 1.0f, 0u); if(bx0>0u)     sammle(0u, bx0-1u, bx0-1u, by0, by1, z_lo, bz1, 1.0f, 0u); }
			if(m_xp) { sammle(1u, bx1, bx1, by0, by1, z_lo, bz1, 1.0f, 0u); if(bx1+1u<cNx) sammle(1u, bx1+1u, bx1+1u, by0, by1, z_lo, bz1, 1.0f, 0u); }
			sammle(2u, bx0, bx1, by0, by0, z_lo, bz1, 1.0f, 0u); if(by0>0u)     sammle(2u, bx0, bx1, by0-1u, by0-1u, z_lo, bz1, 1.0f, 0u);
			sammle(3u, bx0, bx1, by1, by1, z_lo, bz1, 1.0f, 0u); if(by1+1u<cNy) sammle(3u, bx0, bx1, by1+1u, by1+1u, z_lo, bz1, 1.0f, 0u);
			sammle(4u, bx0, bx1, by0, by1, bz1, bz1, 1.0f, 0u); if(bz1+1u<cNz) sammle(4u, bx0, bx1, by0, by1, bz1+1u, bz1+1u, 1.0f, 0u);
		} else { // ---- GRADIENT: N Ein-Zell-Lagen auf Offsets o0+k, w_k = (N-k)/N
			n2f_o0k = env_u("CFD_FERN_FACETTEN",0u)>0u ? 2u : 1u; // o0; =2 haelt die innerste Lage von FERN_FACETTEN-Zellen weg (ersetzt den rand<2-Guard)
			for(uint k=0u; k<n2f_lagen; k++) {
				const float off = (float)(n2f_o0k+k);
				const uint kx0=(uint)fmax(0.0f, veh_c->pmin.x-off), kx1=(uint)fmin((float)cNx-1.0f, veh_c->pmax.x+off);
				const uint ky0=(uint)fmax(0.0f, veh_c->pmin.y-off), ky1=(uint)fmin((float)cNy-1.0f, veh_c->pmax.y+off);
				const uint kz1=(uint)fmin((float)cNz-1.0f, veh_c->pmax.z+off);
				const float wk = (float)(n2f_lagen-k)/(float)n2f_lagen;
				if(m_xm) sammle(0u, kx0, kx0, ky0, ky1, z_lo, kz1, wk, k);
				if(m_xp) sammle(1u, kx1, kx1, ky0, ky1, z_lo, kz1, wk*n2f_xskal, k); // Wake-Messarm: x+ separat skalierbar
				sammle(2u, kx0, kx1, ky0, ky0, z_lo, kz1, wk, k);
				sammle(3u, kx0, kx1, ky1, ky1, z_lo, kz1, wk, k);
				if(kz1>=z_lo) sammle(4u, kx0, kx1, ky0, ky1, kz1, kz1, wk, k); else print_warning("N2F-Schale: z+-Flaeche der Lage "+to_string(k)+" laege im boden_eq-Band (kz1 < z_lo) -- uebersprungen (Pruefagent M1)."); // M1: der Check unten deckt nur die AUSSENLAGE
				if(k==n2f_lagen-1u) { bx0=kx0; bx1=kx1; by0=ky0; by1=ky1; bz1=kz1; }
			}
			if(bz1<z_lo) print_error("N2F-Schale: Aussenlagen-Oberkante liegt unter dem boden_eq-Band (bz1 < z_lo) -- keine Schalenzelle moeglich.");
		}
		if(smap.empty()) print_error("N2F-Schale: Zellliste leer (Flaechen-Maske schaltet alles ab?).");
		// Setup-Checks: (a) Schale muss im Near-Fussabdruck liegen (sonst gibt es keinen Deckungspunkt),
		// (b) Abstand zu JEDER GETRIEBENEN Entnahmeebene (x-, y-, y+, z+; x+ ist Druckauslass) >= 2
		// Grobzellen sonst Fehler, < 4 Warnung -- die Rueckkopplung darf nicht in die Zellen schreiben,
		// aus denen die Hinkopplung im selben Fenster liest.
		int mnx=(int)cNx, mxx=-1, mny=(int)cNy, mxy=-1, mnz=(int)cNz, mxz=-1;
		for(const auto& e : smap) {
			const ulong nn = e.first;
			const int x=(int)(nn%(ulong)cNx), y=(int)((nn/(ulong)cNx)%(ulong)cNy), z=(int)(nn/((ulong)cNx*(ulong)cNy));
			mnx=min(mnx,x); mxx=max(mxx,x); mny=min(mny,y); mxy=max(mxy,y); mnz=min(mnz,z); mxz=max(mxz,z);
		}
		if(mnx<(int)NF_OX||mxx>(int)(NF_OX+cex-1u)||mny<(int)NF_OY||mxy>(int)(NF_OY+cey-1u)||mxz>(int)(NF_OZ+cez-1u))
			print_error("N2F-Schale ragt aus dem Nahfeld-Fussabdruck (x["+to_string(mnx)+".."+to_string(mxx)+"] y["+to_string(mny)+".."+to_string(mxy)+"] z[.."+to_string(mxz)+"] gegen Fussabdruck x["+to_string(NF_OX)+".."+to_string(NF_OX+cex-1u)+"] y["+to_string(NF_OY)+".."+to_string(NF_OY+cey-1u)+"] z[.."+to_string(NF_OZ+cez-1u)+"]) -- Deckungspunkte waeren undefiniert.");
		const int d_xm = mnx-(int)NF_OX, d_ym = mny-(int)NF_OY, d_yp = (int)(NF_OY+cey-1u)-mxy, d_zp = (int)(NF_OZ+cez-1u)-mxz;
		const int d_min = min(min(d_xm,d_ym),min(d_yp,d_zp));
		print_info("N2F-Schale: Abstand zu den getriebenen Entnahmeebenen (Grobzellen): x- "+to_string(d_xm)+", y- "+to_string(d_ym)+", y+ "+to_string(d_yp)+", z+ "+to_string(d_zp)+" (Soll >= 2, komfortabel >= 4).");
		if(d_min<2) print_error("N2F-Schale kommt einer getriebenen Entnahmeebene naeher als 2 Grobzellen -- die Rueckkopplung schriebe direkt in die Quelle der Hinkopplung.");
		else if(d_min<4) print_warning("N2F-Schale naeher als 4 Grobzellen an einer getriebenen Entnahmeebene -- eng; Kasten oder CFD_NEAR_LX/LY/LZ pruefen.");
		// Listen in Map-Reihenfolge (deterministisch aufsteigend) + Host-Census der fluidleeren
		// Bloecke: der Extract schreibt dort den NaN-Marker, der Blend ueberspringt sie per Bit-Test --
		// wie viele das sind, wird HIER angesagt (kein eigener Diag-Slot, Plan-Vorgabe).
		n2f_liste_c.reserve(smap.size()); n2f_liste_f.reserve(smap.size()); n2f_gewicht.reserve(smap.size()); n2f_lage.reserve(smap.size());
		std::vector<ulong> lage_census(max(n2f_lagen,1u), 0ull), lage_nan(max(n2f_lagen,1u), 0ull); // Census PRO LAGE (Plan-Vorgabe)
		const int n2f_r=(int)ratio, n2f_w0=-(n2f_r/2); // Fenster-Konvention wie im Kernel: [-r/2, r-r/2)
		for(const auto& e : smap) {
			const ulong nn = e.first;
			const uint x=(uint)(nn%(ulong)cNx), y=(uint)((nn/(ulong)cNx)%(ulong)cNy), z=(uint)(nn/((ulong)cNx*(ulong)cNy));
			n2f_liste_c.push_back(nn);
			n2f_gewicht.push_back(e.second.first);
			n2f_lage.push_back(e.second.second);
			lage_census[e.second.second]++;
			const uint fdx=(x-NF_OX)*ratio, fdy=(y-NF_OY)*ratio, fdz=(z-NF_OZ)*ratio; // Deckungspunkt (fein = (grob-NF_O)*ratio)
			n2f_liste_f.push_back((ulong)fdx+((ulong)fdy+(ulong)fdz*(ulong)fNy)*(ulong)fNx);
			if(n2f_mittel==1u) {
				uint fluid=0u;
				for(int dz=n2f_w0; dz<n2f_w0+n2f_r&&fluid==0u; dz++) for(int dy=n2f_w0; dy<n2f_w0+n2f_r&&fluid==0u; dy++) for(int dx=n2f_w0; dx<n2f_w0+n2f_r&&fluid==0u; dx++) {
					const int xx=(int)fdx+dx, yy=(int)fdy+dy, zz=(int)fdz+dz;
					if(xx<0||yy<0||zz<0||xx>=(int)fNx||yy>=(int)fNy||zz>=(int)fNz) continue;
					const uchar bo = lbm_f.flags[(ulong)xx+((ulong)yy+(ulong)zz*(ulong)fNy)*(ulong)fNx]&(TYPE_S|TYPE_E);
					if(bo!=TYPE_S&&bo!=TYPE_E) fluid++; // TYPE_MS (S|E) zaehlt als Fluid -- gleiche Konvention wie der Kernel
				}
				if(fluid==0u) { n2f_nanblocks++; lage_nan[e.second.second]++; }
			}
		}
		print_info("N2F-Schale ZELL-CENSUS: Aussenkasten grob x["+to_string(bx0)+".."+to_string(bx1)+"] y["+to_string(by0)+".."+to_string(by1)+"] z["+to_string(z_lo)+".."+to_string(bz1)+"], "+(n2f_lagen>0u?to_string(n2f_lagen)+" Lagen ab Offset o0 = "+to_string(n2f_o0k):string("Doppellage, rand = ")+to_string(n2f_rand)+" Zellen")+"; roh x- "+to_string(roh[0])+" / x+ "+to_string(roh[1])+" / y- "+to_string(roh[2])+" / y+ "+to_string(roh[3])+" / z+ "+to_string(roh[4])+" -> dedupliziert "+to_string((ulong)smap.size())+" Zellen; fluidleere Bloecke (NaN-Skip im Blend): "+to_string(n2f_nanblocks)+".");
		for(uint k=0u; k<max(n2f_lagen,1u); k++) print_info("N2F-Schale LAGEN-CENSUS: Lage "+to_string(k)+" (Offset "+(n2f_lagen>0u?to_string(n2f_o0k+k):to_string(n2f_rand)+"/+1")+", w = "+to_string(n2f_lagen>0u?(float)(n2f_lagen-k)/(float)n2f_lagen:1.0f,3u)+(n2f_lagen>0u&&m_xp&&n2f_xskal!=1.0f?", x+ w = "+to_string(((float)(n2f_lagen-k)/(float)n2f_lagen)*n2f_xskal,3u):"")+"): "+to_string(lage_census[k])+" Zellen, davon fluidleer "+to_string(lage_nan[k])+(k==n2f_lage_aussen?" [WAECHTER-LAGE]":"")+".");
		print_info(string("N2F-Schale Flaechen-Maske (Heiko): x- ")+(m_xm?"AN (CFD_N2F_SCHALE_XMINUS=1)":"AUS (Default -- Zirkularitaet, Anstroemung)")+", x+ "+(m_xp?"AN (Default; CFD_N2F_SCHALE_XPLUS=0 schaltet ab)":"AUS (CFD_N2F_SCHALE_XPLUS=0, Wake-Messarm)")+", y-/y+/z+ AN; Modus: "+(n2f_mittel?"Blockmittel ratio^3 (nur Fluid)":"Punktwert (CFD_N2F_SCHALE_MITTEL=0)"));
		if(n2f_nanblocks==(ulong)smap.size()) print_error("N2F-Schale: ALLE Bloecke fluidleer -- der Blend waere ein vollstaendiger No-Op.");
		if(n2f_lagen>0u&&lage_census[n2f_lage_aussen]==lage_nan[n2f_lage_aussen]) print_error("N2F-Schale: die AEUSSERSTE Lage ist komplett fluidleer -- Waechter/Negations-Nachweis/Kipp haetten keine einzige gueltige Zelle.");
	}

	// ---------------------------------------------------------------- Laufsteuerung
	const float t_flush  = (float)(cNx-1u)*dx_c/si_u; // Durchspuelung des FERNfelds (far_x0 kuerzt sich weg)
	const float t_end    = env_f("CFD_T_END", 2.0f*t_flush);
	const float t_warmup = env_f("CFD_T_WARMUP", 1.0f*t_flush);
	ulong n_outer  = (ulong)(t_end/dt_c + 0.5f); // ★ 04.09.2026 NICHT MEHR const -- die Laufverlaengerung
	// (CFD_ZIEL_DATEI, s. Zeitschleife) darf ihn waehrend des Laufs anheben. Alles, was VOR der Schleife
	// aus n_outer gerechnet wird (VTK-Rotation kp, verify_at2), bleibt auf der urspruenglichen Planung --
	// das ist gewollt und wird beim Verlaengern angesagt.
	const uint  sample_every = max(1u, env_u("CFD_SAMPLE_EVERY", 25u)); // in groben Schritten
	// ★ KADENZ-UMBAU (Heiko 27.08.): Ausgabe-Kadenz in NEAR-STEPS statt Sekunden-Uhr. Default 5000
	// Near-Steps (= 1250 Outer = 50 ms bei 4 mm; gitterschritt-, nicht zeitfest -- bei 8 mm 100 ms).
	// Entscheidungstabelle (S=CFD_SLICE_NEAR_STEPS, D=CFD_SLICE_DT): S>0 -> Kadenz-Modus (Warnung,
	// falls D auch gesetzt); S=0 UND D gesetzt -> Legacy-Uhr wortgleich; S=0 allein -> alles aus;
	// S ungesetzt UND D gesetzt -> Legacy (Rueckwaertskompat.); beide ungesetzt -> Kadenz 5000.
	const bool kad_s_gesetzt = getenv("CFD_SLICE_NEAR_STEPS")!=nullptr, kad_d_gesetzt = getenv("CFD_SLICE_DT")!=nullptr;
	const ulong slice_ns = kad_s_gesetzt ? (ulong)env_u("CFD_SLICE_NEAR_STEPS", 5000u) : (kad_d_gesetzt ? 0ull : 5000ull);
	const float slice_dt = (slice_ns==0ull&&kad_d_gesetzt) ? env_f("CFD_SLICE_DT", 0.010f) : 0.0f;
	// ★ Heiko 28.08.: "sliceausgabe muss an sein! kostet nichts". Der Baseline-Lauf trug
	// CFD_SLICE_DT=0 und schrieb damit gar keine Slices; ueber die Referenz hat sich das in
	// eine ganze Messreihe fortgepflanzt, und die Bilder mussten hinterher aus den VTK-Feldern
	// nachgerendert werden. Ein Schalter, der keine Physik beruehrt, kann eine Messreihe
	// trotzdem unauswertbar machen -- deshalb hier ein harter Riegel statt einer Warnung.
	if(slice_ns==0ull&&slice_dt<=0.0f) print_error("Dieser Lauf schriebe KEINE Slices (CFD_SLICE_DT=0 und CFD_SLICE_NEAR_STEPS ungesetzt/0). Slice-Ausgabe ist Pflicht -- sie kostet praktisch nichts und ohne sie ist der Lauf hinterher nicht ansehbar. CFD_SLICE_DT auf einen positiven Wert setzen (0.1 = alle 100 ms).");
	const bool slices_an = slice_ns>0ull||slice_dt>0.0f;
	if(kad_s_gesetzt&&kad_d_gesetzt&&slice_ns>0ull) print_warning("CFD_SLICE_NEAR_STEPS und CFD_SLICE_DT beide gesetzt -- die Near-Step-Kadenz gewinnt, die Sekunden-Uhr ist wirkungslos.");
	if(slice_ns>0ull) {
		print_info("SLICE-KADENZ: alle "+to_string(slice_ns)+" Near-Steps = "+to_string(slice_ns/(ulong)ratio)+" Outer = "+to_string((float)((double)slice_ns/(double)ratio*(double)dt_c*1000.0),1u)+" ms Physik (CFD_SLICE_NEAR_STEPS; 0 = aus; Sekunden-Uhr nur noch als Legacy ueber explizites CFD_SLICE_DT).");
		if(slice_ns%((ulong)sample_every*(ulong)ratio)!=0ull) print_warning("CFD_SLICE_NEAR_STEPS ("+to_string(slice_ns)+") ist kein Vielfaches von CFD_SAMPLE_EVERY*ratio ("+to_string((ulong)sample_every*(ulong)ratio)+") -- Ereignisse quantisieren auf den naechsten Sample-Punkt.");
	}
	// ★ Slice-Ebenen-Read (Perf-Hebel 2026-08-26): CFD_SLICE_GPU=1 (Default) holt je Slice-Ereignis
	// nur die konsumierten y-Ebenen per Device-Gather (~14 MB bei 4 mm) statt der vollen Felder
	// (~8,65 GB); =0 ist der wortgleiche Altpfad. CFD_SLICE_PRUEF=1 rechnet BEIDE Wege und druckt
	// die maximale Abweichung der Ebene (Muster CFD_FAC_GPU_PRUEF).
	const bool slice_gpu   = env_u("CFD_SLICE_GPU", 1u)>0u;
	const bool slice_pruef = env_u("CFD_SLICE_PRUEF", 0u)>0u;
	// ★ Auditor-B B-3 (Ansage-Doktrin): der Transportweg ist Default AN und muss im Log stehen.
	if(slices_an) print_info(string("Slice-Transportweg: ")+(slice_pruef?"PRUEF-Arm (beide Wege, Vergleich je Ebene)":(slice_gpu?"Ebenen-Gather (CFD_SLICE_GPU=1)":"Voll-Read-Altpfad (CFD_SLICE_GPU=0)"))+".");
	else if(slice_pruef) print_warning("CFD_SLICE_PRUEF=1 ohne aktive Slice-Kadenz -- keine Slice-Ereignisse, kein Wirkpfad."); // Auditor-B B-4
	// ★ KIPP-WAECHTER-SCHARFSCHALTUNG (Heiko 2026-08-21, Befund aus f4_std_diff): der Waechter stand
	// fest auf t > 0,02 s. Diese Zahl ist auf der 8-mm-Sprosse geeicht. Der Impulsstart-Transient
	// erreicht auf BEIDEN Sprossen ein Vielfaches der Schwelle (8 mm: |Cd| 41,25 / 4 mm: 49,24, je bei
	// t ~ 1-2 ms) -- bis 20 ms ist er auf 8 mm auf 14,4 abgeklungen, auf 4 mm steht er noch bei 17-20.
	// f4_std_diff riss dort mit 20,06; f4_neustandard2 ueberlebte dieselbe Stelle mit 18,62, also mit
	// 7 % Rest. Das war Glueck, keine Auslegung. Bei t = 21 ms ist die Stroemung 0,63 m weit gelaufen,
	// das Fahrzeug ist 4,4 m lang -- ein Cd existiert dort noch gar nicht.
	// GEMESSENE Grundlage des neuen Werts: im Fenster 0,05..0,2 s liegt max|Cd| bei 13,82 (8 mm,
	// f8_standard_final) bzw. 14,53 (4 mm, f4_neustandard2) -- 27 % Luft zur Schwelle 20.
	const float kipp_ab = env_f("CFD_KIPP_AB", 0.05f);
	if(!(kipp_ab>=0.0f)) print_error("CFD_KIPP_AB darf nicht negativ sein (Scharfschaltzeit des Kipp-Waechters in Sekunden; NaN faengt dieser Test mit).");
	if(kipp_ab>=t_end) print_warning("CFD_KIPP_AB ("+to_string(kipp_ab,3u)+" s) liegt bei oder hinter CFD_T_END -- der Kipp-Waechter wird in diesem Lauf NIE scharf.");
	print_info("KIPP-WAECHTER: scharf ab t > "+to_string(kipp_ab,3u)+" s (CFD_KIPP_AB), Schwelle |Cd| bzw. |Cz| > 20. Davor wirken nur die NaN-/Nichtendlich-Zweige. Der Impulsstart-Transient liegt bei t ~ 1-2 ms bei |Cd| ~ 41 (8 mm) bis 49 (4 mm) und ist keine Explosion.");
	// ★ VTK-FELDEXPORT (Heiko 2026-08-21): Default AUS -- ein Dump kostet Sekunden und ueber ein
	// Gigabyte, das gehoert nicht in jeden Screening-Lauf. CFD_VTK_ENDE=1 schreibt EINEN Dump beider
	// Domaenen am Laufende (der Fall, um den es ging: das Feld des LETZTEN Zeitschritts), CFD_VTK_DT>0
	// zusaetzlich an einer eigenen Kadenz. CFD_VTK_STRIDE tastet ab (2 -> 1/8 der Punkte).
	// ★ DIFF-SCHNITT: EINMAL gelesen (env-Read-Falle -- die Konstanten tragen bis in die Zeitschleife,
	// wie bei CFD_N2F_SCHALE/CFD_FERN_EINLASS_EQ). Vorher standen beide env-Aufrufe IM Schleifenkoerper.
	const bool  diff_an   = getenv("CFD_DIFF_SCHNITT") ? env_u("CFD_DIFF_SCHNITT", 1u)>0u : (slice_ns>0ull ? false : true); // Kadenz-Modus: Default AUS (Heiko 26.08. "spaeter drauf zurueck"), CFD_DIFF_SCHNITT=1 reaktiviert
	const float diff_span = env_f("CFD_DIFF_SPAN", 15.0f);
	if(!(diff_span>0.0f)) print_error("CFD_DIFF_SPAN muss groesser als 0 sein (Halbbreite der Diff-Farbskala in m/s; NaN faengt dieser Test mit)."); // Negativform, faengt NaN
	if(slices_an&&diff_an) print_info("DIFF-SCHNITT aktiv: je Schnitt zusaetzlich |u_nah|-|u_fern| an derselben Weltposition (Fernfeld trilinear auf den Feinzellmittelpunkt), Skala +-"
		+to_string(diff_span,1u)+" m/s (blau/weiss/rot), Solid schwarz -> schnitt_diff_<ms>ms.png + schnitt_diff_letzter.csv (Echtdaten, Iron Rule 5). Laeuft auf der CPU aus dem Hostspeicher, KEIN zusaetzlicher Device-Read.");
	else if(slices_an) print_info(string("DIFF-SCHNITT AUS (")+(slice_ns>0ull&&!getenv("CFD_DIFF_SCHNITT")?"Kadenz-Modus-Default; CFD_DIFF_SCHNITT=1 reaktiviert":"CFD_DIFF_SCHNITT=0")+").");
	// ★ SAUBERER STOPP (Heiko 2026-08-21). V1 hatte das (/tmp/cfd_stop), V2 nicht -- beim Neuaufbau
	// nicht mitportiert. Ohne den Mechanismus ist ein laufender Lauf nur per kill zu beenden, und
	// dabei geht ALLES verloren, was hinter der Zeitschleife steht: der VTK-Dump am Laufende, die
	// Endauswertung, die Mittelwerte. Die CSVs ueberleben (sie werden je Sample geflusht) -- das Feld
	// nicht. Die Datei anlegen -> naechste Sample-Kadenz verlaesst die Schleife regulaer, danach
	// laeuft der komplette Abschlusspfad. Stale-Datei-Falle (V1-Lehre): beim Start wird eine
	// liegengebliebene Datei entfernt, sonst stoppt der naechste Lauf sofort nach dem ersten Sample.
	const string stop_datei = getenv("CFD_STOP_DATEI") ? string(getenv("CFD_STOP_DATEI")) : string("/tmp/cfd_stop");
	std::remove(stop_datei.c_str());
	const string ziel_datei = getenv("CFD_ZIEL_DATEI") ? string(getenv("CFD_ZIEL_DATEI")) : string("/tmp/cfd_ziel_ms");
	std::remove(ziel_datei.c_str()); // wie die Stoppdatei: eine liegengebliebene darf den naechsten Lauf nicht treffen
	print_info("SAUBERER STOPP: \"touch "+stop_datei+"\" verlaesst die Zeitschleife an der naechsten Sample-Kadenz (alle "
		+to_string(sample_every)+" grobe Schritte) und laeuft den vollstaendigen Abschlusspfad -- Endauswertung, CSV-Schluss"
		+(env_u("CFD_VTK_ENDE",0u)>0u?string(" UND VTK-Feld-Dump"):string(" (VTK nur mit CFD_VTK_ENDE=1)"))+". Pfad ueber CFD_STOP_DATEI aenderbar.");
	print_info("LAUFVERLAENGERUNG: \"echo <ms> > "+ziel_datei+"\" hebt die Endzeit an der naechsten Sample-Kadenz an (nur VERLAENGERN; Verkuerzen geht ueber die Stoppdatei, die den vollen Abschlusspfad laeuft). Pfad ueber CFD_ZIEL_DATEI aenderbar.");
	const float vtk_dt     = env_f("CFD_VTK_DT", 0.0f);
	const bool  vtk_ende   = env_u("CFD_VTK_ENDE", 0u)>0u;
	const uint  vtk_stride = max(1u, env_u("CFD_VTK_STRIDE", 1u));
	// ★ KADENZ-UMBAU (Heiko 27.08.): VTK an der Near-Step-Kadenz (jeder JEDE-te Kadenzpunkt) mit
	// Behalte-Rotation -- "nur die letzten zwei VTKs behalten" spart Platte (12,1 GB je Doppeldump
	// bei 4mm/Stride 1); der Enddump (CFD_VTK_ENDE) rotiert NIE mit.
	const uint  vtk_jede    = env_u("CFD_VTK_JEDE", 0u);    // 0 = aus; 1 = jeder Kadenzpunkt
	const uint  vtk_behalte = env_u("CFD_VTK_BEHALTE", 2u); // 0 = alle behalten
	if(vtk_jede>0u&&slice_ns==0ull) print_warning("CFD_VTK_JEDE>0 ohne Near-Step-Kadenz (CFD_SLICE_NEAR_STEPS=0) -- wirkungslos.");
	if(vtk_jede>0u&&vtk_dt>0.0f) print_warning("CFD_VTK_JEDE und CFD_VTK_DT beide aktiv -- zwei VTK-Uhren gleichzeitig (Enddump-Dedup greift, Rotation auf beide).");
	const string out_dir = get_exe_path()+"../export/"+(getenv("CFD_RUN_NAME")?string(getenv("CFD_RUN_NAME")):string("fahrzeug_dd"))+"/";
	create_folder(out_dir);
	sichere_lauf(out_dir, "fahrzeug_dd");
	const float q_inf = 0.5f*si_rho*si_u*si_u;
	print_info("Eine Fernfeld-Durchspuelung = "+to_string(t_flush,4u)+" s; Laufzeit "+to_string(t_end,3u)+" s = "
		+to_string(n_outer)+" grobe x "+to_string(ratio)+" feine Schritte, Mittelung ab "+to_string(t_warmup,3u)+" s");
	// ★ Pruefer-Befund 2026-08-08: die Pruefung auf genug Samples stand HINTER der Zeitschleife.
	// Bei 2,5 Stunden Laufzeit heisst das: erst nach dem vollen Lauf erfaehrt man, dass die Parameter
	// gar keine auswertbare Reihe ergeben. Sie gehoert hierher, VOR den ersten Zeitschritt.
	{
		const long n_expect = (long)((double)fmax(0.0f, t_end-t_warmup)/((double)sample_every*(double)dt_c));
		print_info("Erwartete Samples nach dem Warmlauf: "+to_string((uint)max(0L, n_expect))+" (mindestens 16 noetig, ab 32 traegt der Block-SEM ueber 16 Bloecke)");
		if(n_expect<16L) { print_error("Diese Parameter ergeben nur "+to_string((uint)max(0L, n_expect))+" Samples nach dem Warmlauf. Lauf nicht gestartet -- CFD_T_END, CFD_T_WARMUP oder CFD_SAMPLE_EVERY anpassen."); _exit(1); }
	}

	// ---------------------------------------------------------------- Initialisieren und Kopplung anlegen
	lbm_f.run(0u); // nur initialisieren
	lbm_c.run(0u);
	// ★ SPEICHER-SCHLUSSBILANZ (Heiko 29.08.: "was wir wirklich nutzen"). Die Zeile
	// "Memory Usage" der Info-Box (info.cpp:73) liest memory_used ZU FRUEH -- Kopplungspuffer,
	// N2F-Schale, kf_liste und slice_flags entstehen erst danach. Hier ist der Aufbau
	// vollstaendig, also steht hier der SPITZENWERT. Dazu der gemessene freie Speicher aus dem
	// DRM-Debugfs, der als einziger den Desktop-Anteil mitsieht.
	{	const ulong belegt_f = (ulong)lbm_f.lbm_domain[0]->get_device().info.memory_used;
		const ulong kap_f    = (ulong)lbm_f.lbm_domain[0]->get_device().info.memory;
		const ulong belegt_c = (ulong)lbm_c.lbm_domain[0]->get_device().info.memory_used;
		const ulong frei_g   = vram_frei_gemessen();
		print_info("SPEICHER-SCHLUSSBILANZ (Aufbau vollstaendig, das ist der Spitzenwert):");
		print_info("   Nahfeld  belegt "+to_string(belegt_f)+" MB von "+to_string(kap_f)+" MB rekonstruiert"
			+"  ->  rechnerisch frei "+to_string((ulong)(kap_f>belegt_f?kap_f-belegt_f:0ull))+" MB");
		print_info("   Fernfeld belegt "+to_string(belegt_c)+" MB (System-RAM, kein VRAM-Deckel)");
		if(frei_g>0ull) print_info("   GEMESSEN frei auf der Karte: "+to_string(frei_g)
			+" MB (DRM-Debugfs -- enthaelt den Desktop-Anteil, im Gegensatz zur Rechnung oben)."
			+string(frei_g<1024ull?" ★ UNTER 1 GB -- das ist Heikos Untergrenze.":""));
		else print_info("   Gemessener Frei-Wert nicht lesbar (Debugfs braucht Rechte) -- die Zahl oben"
			" ist eine RECHNUNG aus device.info.memory, und die ist ihrerseits die 20/19-Rekonstruktion"
			" von opencl.hpp:170. Sie sieht den Desktop-Anteil NICHT.");
	}

	// ---------------------------------------------------------------- Bodenkontakt und mitbewegte Wand
	// Drei Fragen, die sich NUR nach initialize() beantworten lassen, weil erst dieser Kernel die
	// TYPE_MS-Markierung setzt -- und weil genau daran im alten Baum schon einmal alles haengen blieb:
	// dort war die Bodengeschwindigkeit ein vollstaendiges No-op, weil ein MS-Waechter nie ausloeste.
	//
	//  (1) STEHT DER WAGEN AUF DEM BODEN? Gezaehlt wird nicht die Absicht, sondern der Zustand:
	//      an wie vielen Stellen liegt eine Fahrzeugzelle DIREKT auf einer Fahrbahnzelle.
	//  (2) BEWEGT SICH DIE FAHRBAHN WIRKLICH? Der mitbewegte Rand wirkt ueber apply_moving_boundaries,
	//      und das laeuft AUSSCHLIESSLICH auf Zellen mit flags&TYPE_BO == TYPE_MS. Ohne TYPE_MS ueber
	//      der Fahrbahn ist der bewegte Boden eine ruhende Wand -- ohne jede Fehlermeldung.
	//  (3) GEHT DIE AUFSTANDSFLAECHE IN DIE KRAFT EIN? object_force vergleicht flags[n] == flag_marker
	//      auf EXAKTE Gleichheit (kernel.cpp, object_force). Fahrzeug ist 0x41, die uebergebene
	//      Aufstandsflaeche 0x01 -- sie faellt damit aus der Summe. Hier wird beides ausgezaehlt.
	{
		audit_bewegte_waende(lbm_f, fNx, fNy, fNz, dx_f, u_lat, "Nahfeld", true);
		audit_bewegte_waende(lbm_c, cNx, cNy, cNz, dx_c, u_lat, "Fernfeld", true);
	}
	ulong max_cp = 0ull;
	for(uint p=0u; p<5u; p++) max_cp = max(max_cp, (ulong)cp[p].extent_a*(ulong)cp[p].extent_b);
	// ★ Die Bilanzebenen des Wake-Kastens laufen ueber DENSELBEN Kopplungspuffer. Ihre Groesse
	// explizit einrechnen -- sonst haengt die Korrektheit daran, dass die Kopplungsebenen zufaellig
	// groesser sind (bei 8 mm sind sie es: 16.960 gegen 7.076; bei anderen Aufloesungen nicht
	// zwingend). extract_plane_macros bricht bei zu kleinem Puffer hart ab, aber erst zur Laufzeit.
	if(bil_an) {
		const ulong bx=(ulong)(bil_x1-bil_x0+1u), by=(ulong)(bil_y1-bil_y0+1u), bz=(ulong)(bil_z1-bil_z0+1u);
		const ulong bmax = max(max(by*bz, bx*bz), bx*by);
		if(bmax>max_cp) print_info("N2F-BAND BILANZ: groesste Kastenflaeche "+to_string(bmax)+" Zellen ueberschreitet die Kopplungsebenen ("+to_string(max_cp)+") -- Kopplungspuffer wird entsprechend groesser angelegt.");
		max_cp = max(max_cp, bmax);
	}
	// ★ Slice-Ebenen-Read: die y-Ebenen beider Domaenen laufen ueber DENSELBEN Kopplungspuffer
	// (Plan Variante b) -- ihre Groesse explizit einrechnen, exakt die Lektion der Bilanzebenen oben.
	if(slices_an && (slice_gpu||slice_pruef)) {
		const ulong smax = max((ulong)fNx*(ulong)fNz, (ulong)cNx*(ulong)cNz);
		if(smax>max_cp) print_info("SLICE-EBENEN-READ: y-Ebene "+to_string(smax)+" Zellen ueberschreitet die Kopplungsebenen ("+to_string(max_cp)+") -- Kopplungspuffer wird entsprechend groesser angelegt.");
		max_cp = max(max_cp, smax);
	}
	lbm_c.alloc_coupling_planes(max_cp); // entnimmt
	lbm_f.alloc_coupling_planes(max_cp); // empfaengt dieselben Ebenen

	std::vector<float> face[5];
	// ★ ZEITINTERP Stufe 1 (Host-Mix): face_alt[p] haelt den Ebenen-Stand des VORIGEN Kopplungs-
	// fensters (kopiert NACH dem Drive, VOR dem Extract), face_mix ist der Mischpuffer je Substep.
	// Beide bleiben leer, solange CFD_KOPPLUNG_ZEITINTERP unset/0 ist (bitidentischer Altpfad).
	std::vector<float> face_alt[5];
	std::vector<float> face_mix;
	lbm_c.run(1u); // ein grober Schritt, damit ein Zustand zum Entnehmen existiert
	// ★ KORREKTUR 2026-08-08 (Pruefer-Befund): hier lief die Schleife ueber ALLE FUENF Ebenen, obwohl
	// x+ gar nicht getrieben wird (dort steht der Druckauslass). Die grobe x+-Ebene wurde also jeden
	// groben Schritt entnommen -- mit blockierendem finish_queue, 304 kB Device-Read und einer
	// Host-Kopierschleife -- und dann von niemandem gelesen. Genau das Muster, das diesen Baum
	// ueberhaupt noetig gemacht hat: extrahiert, transferiert, weggeworfen.
	for(uint p=0u; p<5u; p++) if(drive_face[p]) lbm_c.extract_plane_macros(cp[p], face[p]);

	// ★ P9c N2F-SCHALE: alloc NACH run(0)/run(1) (Kernel brauchen initialisierte, FINAL gebundene
	// Puffer -- Nahfeld-fi steht erst nach finalize_sparse_tiles; das Grobgitter faehrt im dd-Fall
	// ohne Tiling) + 1-Outer-VORLAUF wie die Hinkopplung in der Zeile darueber: das erste Blend im
	// ersten run_async sieht damit ein echtes Nahfeld-Blockmittel statt des Ctor-Nullpuffers.
	std::vector<float> n2f_unear, n2f_ufar;
	std::ofstream swcsv; double sw_rms_prev = -1.0; uint sw_steigend = 0u; bool n2f_neg_geprueft = false;
	if(n2f_alpha>0.0f) {
		lbm_f.alloc_schale(n2f_liste_f, n2f_gewicht, ratio, n2f_modus);   // Deckungspunkt-Indizes, Blockmittel-Fenster ratio^3 (Gewichte hier inert -- lbm_f blendet NIE, alpha=0)
		lbm_c.alloc_schale(n2f_liste_c, n2f_gewicht, 1u, n2f_modus);      // Schalenzellen; ratio=1 -- hier laeuft nur der Waechter-Extract (mittel=0) und der Blend
		lbm_f.schale_extract_u(n2f_unear, n2f_mittel);
		lbm_c.schale_upload_unear(n2f_unear);
		swcsv.open(out_dir+"schale_waechter.csv"); swcsv.precision(8);
		// ★ CSV-SCHEMA-ERWEITERUNG (Gradient-Blend, hiermit ANGESAGT): n_gueltig/rms_lat/max_lat/
		// rms_rel_uinf beziehen sich jetzt auf die AEUSSERSTE Lage (n2f_lage == N-1; im Kontrollarm
		// LAGEN=0 unveraendert auf alle Zellen); NEU dahinter: n_gueltig_alle und rms_alle_lat =
		// dieselbe Metrik ueber ALLE Lagen (Diagnose; Kipp-Kriterium haengt NUR an der Aussenlage).
		// ★ B6 (Pruefagent 2026-08-22): n_gueltig hat mit der Wake-Trennung die BEDEUTUNG gewechselt --
		// es ist jetzt "Aussenlage OHNE Wake-Zone" statt "Aussenlage". Belegt: wake_rad (vor der
		// Trennung) n_gueltig = 21.940; max_bew (danach, gleiche Geometrie) n_gueltig = 10.276 plus
		// n_wake = 11.664, Summe exakt 21.940. Wer rms_lat oder n_gueltig ueber diese Grenze hinweg
		// vergleicht, vergleicht verschiedene Grundgesamtheiten.
		swcsv << "# n_gueltig/rms_lat/max_lat: Aussenlage OHNE Wake-Zone (Bedeutungswechsel 2026-08-22, vorher: ganze Aussenlage). n_wake/rms_wake_lat: Aussenlage IN der Wake-Zone. n_kern/rms_kern_lat/max_kern_lat: ALLE Lage-0-Zellen der Wake-Zone (Kastenkern PLUS Koerperband-Lage-0 mit x>=wx0 -- beide volles Gewicht; B-P5) -- bis 2026-08-22 von keiner Metrik erfasst. n_gueltig_alle/rms_alle_lat: alle Bandlagen.\n";
		swcsv << "time_s,n_gueltig,rms_lat,max_lat,rms_rel_uinf,n_gueltig_alle,rms_alle_lat,n_wake,rms_wake_lat,n_kern,rms_kern_lat,max_kern_lat\n" << std::flush;
		print_info("N2F-SCHALE-Waechter aktiv: an der Sample-Kadenz RMS/Max ||u_near-u_far|| ueber "+(n2f_volumen>0u?string("das VOLUMEN-AUSSENBAND (aeusserste Gewichtsstufe, lage ")+to_string(n2f_lage_aussen):string("die AEUSSERSTE Schalen-Lage (Lage ")+to_string(n2f_lage_aussen))+"; Fernfeld-Punktwerte via schale_extract mittel=0) -> "+out_dir+"schale_waechter.csv; CSV-Schema ERWEITERT um n_gueltig_alle,rms_alle_lat (alle Lagen, Diagnose); Kipp-Kriterium (nur Aussenlage/-band): RMS > 0,5*u_inf nach Warmup ODER 10 Samples monoton steigend -> Abbruch.");
	}

	// ---------------------------------------------------------------- Zeitschleife
	// Der grobe Schritt laeuft ASYNCHRON auf dem zweiten Geraet, waehrend das Nahfeld seine ratio
	// Schritte rechnet.
	// ★ KORREKTUR 2026-08-08 (Pruefer-Befund): hier stand "Verzug". Es ist ein VORLAUF. Vor der
	// Schleife macht das grobe Gitter einen Schritt, danach wird entnommen; das Nahfeld durchlaeuft
	// also das Intervall [k, k+1] mit dem groben Zustand an dessen ENDE. Am Intervallanfang eilt der
	// Rand um dt_c voraus, am Ende stimmt er. Der Betrag ist derselbe -- bei dt_c = 4e-5 s wandert
	// die Stroemung 1,2 mm, weniger als eine feine Zelle --, aber die Richtung war falsch beschrieben.
	// Als Praediktor-Halteschema ist ein Vorlauf gegenueber einem Nachlauf eher guenstiger.
	std::vector<double> ts, fx, fz, fx_c; // fx_c war write-only (Hygiene E6b) -- wird jetzt unten ausgewiesen
	// ★ VTK-Erstdump-Fix (Perf-Audit Achse 1, Rang 3): vtk_next=0 liess den ersten Zwischendump
	// bei t=1 ms feuern -- 11,5 GB Anfangszustands-Muell je 4mm-Lauf (f4_wandfrei_v2 belegt).
	// Erste Marke jetzt bei vtk_dt; der Enddump (CFD_VTK_ENDE) ist davon unabhaengig.
	float slice_next = 0.0f, vtk_next = vtk_dt;
	ulong slice_ns_next = slice_ns, kad_punkt = 0ull; int vtk_ms_letzt = -1; std::vector<int> vtk_rotation; // Kadenz-Zustand + Behalte-Rotation (27.08.)
	double t_si_letzt = 0.0; bool stop_angefordert = false; // gesetzt je Aussenschritt bzw. beim sauberen Stopp
	// ★ 2026-08-22, Befund aus dem Stopp-Rauchtest: die Wirkpfad-Sollwerte unten rechneten mit
	// n_outer, also der GEPLANTEN Schrittzahl. Nach einem sauberen Stopp ist die tatsaechliche
	// kleiner -- ein voellig korrekter Lauf meldete sich als "Ist != Soll, Bindung defekt" und
	// endete mit rc=1. Wer den Lauf gerettet hat, haette ihn danach fuer ungueltig gehalten.
	ulong n_outer_ist = 0ull; // Aussenschritte, die WIRKLICH gelaufen sind (== n_outer ohne Stopp)
	if(vtk_dt>0.0f||vtk_ende||(slice_ns>0ull&&vtk_jede>0u)) { // Kosten VOR dem ersten Zeitschritt ansagen, nicht erst beim Schreiben
		auto mb = [&](const uint Nx, const uint Ny, const uint Nz) {
			const ulong np=(ulong)((Nx+vtk_stride-1u)/vtk_stride)*(ulong)((Ny+vtk_stride-1u)/vtk_stride)*(ulong)((Nz+vtk_stride-1u)/vtk_stride);
			return (float)(np*17ull)/1048576.0f; };
		print_info("VTK-FELDEXPORT aktiv (Abtastung 1:"+to_string(vtk_stride)+"): je Dump nah "+to_string(mb(fNx,fNy,fNz),0u)
			+" MB + fern "+to_string(mb(cNx,cNy,cNz),0u)+" MB = "+to_string(mb(fNx,fNy,fNz)+mb(cNx,cNy,cNz),0u)+" MB; "
			+(vtk_ende?string("EIN Dump am Laufende"):string("kein Dump am Laufende"))
			+(vtk_dt>0.0f?", zusaetzlich alle "+to_string(vtk_dt*1000.0f,0u)+" ms":"")
			+". ORIGIN/SPACING sind die ECHTE Weltlage beider Gitter -- die Dateien liegen im Betrachter deckungsgleich uebereinander.");
		if(slice_ns>0ull&&vtk_jede>0u) {
			const ulong kp = ((ulong)n_outer*(ulong)ratio/slice_ns)/(ulong)vtk_jede;
			print_info("VTK AN DER KADENZ (CFD_VTK_JEDE="+to_string(vtk_jede)+", CFD_VTK_BEHALTE="+to_string(vtk_behalte)+"): voraussichtlich "+to_string(kp)+" Doppeldumps a "+to_string(mb(fNx,fNy,fNz)+mb(cNx,cNy,cNz),0u)+" MB"
				+(vtk_behalte>0u?"; Platte haelt maximal "+to_string((float)(vtk_behalte+1u)*(mb(fNx,fNy,fNz)+mb(cNx,cNy,cNz))/1024.0f,1u)+" GB (Rotation)":"; KEINE Rotation -- "+to_string((float)kp*(mb(fNx,fNy,fNz)+mb(cNx,cNy,cNz))/1024.0f,1u)+" GB gesamt")+".");
		}
	}
	auto vtk_rotiere = [&](const int t_ms_neu) { // Behalte-Rotation: nur die letzten N Zeitpunkte auf Platte
		vtk_rotation.push_back(t_ms_neu);
		while(vtk_behalte>0u && vtk_rotation.size()>(size_t)vtk_behalte) {
			string ma = to_string(vtk_rotation.front()); while(ma.length()<6u) ma = "0"+ma;
			std::remove((out_dir+"feld_nah_"+ma+"ms.vtk").c_str()); std::remove((out_dir+"feld_fern_"+ma+"ms.vtk").c_str());
			print_info("[VTK] Rotation: feld_{nah,fern}_"+ma+"ms.vtk geloescht (CFD_VTK_BEHALTE="+to_string(vtk_behalte)+").");
			vtk_rotation.erase(vtk_rotation.begin());
		}
	};
	Clock outer_clock; double t_acc = 0.0; ulong n_acc = 0ull;
	double Fx_prev = 1e300, Fz_prev = 1e300; uint n_frozen = 0u; // fuer den Einfrier-Test, siehe Zeitschleife
	// ★ Pruefer-Befund 2026-08-08: die CSV entstand bisher ERST NACH der Schleife. Bei 2,5 Stunden
	// Laufzeit heisst das: ein Abbruch durch Treiber, Speicher, Stromausfall oder Strg-C kostet
	// SAEMTLICHE Samples. Jetzt wird jede Zeile sofort geschrieben und geleert -- die Datei ist damit
	// zu jedem Zeitpunkt vollstaendig bis zum letzten Sample, und ein abgebrochener Lauf bleibt
	// auswertbar. Die Vektoren bleiben zusaetzlich fuer die Statistik am Ende.
	std::ofstream fcsv(out_dir+"forces.csv"); fcsv.precision(8);
	fcsv << "time_s,Fx_N,Fz_N,Cd,Cz,Fx_far_N\n" << std::flush;
	// ★ Stufe 5: Facetten-Cd-Pfad (Hybrid, Kugel-Muster E6): Druck als Zeitmittel an der Kadenz,
	// Reibung als exaktes Fenster-Delta ab Snapshot. C7-Notiz VOR der Schleife (ARBEITSLISTE 167):
	// die aufgepraegte Schubspannung wirkt als Impulssenke, sie verschiebt keinen Abloesepunkt --
	// Gleichgewichts-Wandmodell ohne APG-Term (FACETTEN.md Paragraph 4 Punkt 0).
	std::vector<double> fac_snap; ulong fac_snap_outer=0ull, fac_pn=0ull, fac_smp=0ull;
	double fac_px=0.0, fac_pz=0.0, fac_dm=0.0, fac_rest=0.0, fac_dm0=0.0, fac_rest0=0.0;
	std::ofstream fac_csv;
	const ulong fac_cd_every = (ulong)max(1u, env_u("CFD_FAC_CD_EVERY", 4u));
	const bool fac_an_zs = env_u("CFD_FACETTEN", 0u)>0u; // env-Read VOR der Zeitschleife (Doktrin; stand im Sample-Block)
	double kad_cd_rest=0.0, kad_cz_rest=0.0; bool kad_cdcz_da=false; // letzte korrigierte Werte fuer Bild-Einblendung + [KADENZ]-Status (27.08.)
	// ★ LAUFBERICHT (Heiko 29.08.): "alle 100ms physikalisch einen bericht ueber aktuelle
	// ueber 50ms gemittelte korrigierte cd und cz". Korrigiert = REST = Gesamt minus
	// Radkontakt-Band, dieselbe Groesse, auf der die Messarme verglichen werden. Der
	// vorhandene [KADENZ]-Druck zeigt die MOMENTANwerte am Sample; ein Momentanwert
	// schwankt im Ablauf um ein Vielfaches der Armunterschiede und taugt nicht zum Mitlesen.
	// Gleitendes Zeitfenster, keine Blockmittel -- der Bericht soll den Verlauf zeigen.
	const double ber_dt   = (getenv("CFD_BERICHT_DT")     ==nullptr) ? 0.1  : atof(getenv("CFD_BERICHT_DT"));      // Berichtsabstand [s Physik]
	const double ber_fen  = (getenv("CFD_BERICHT_FENSTER")==nullptr) ? 0.05 : atof(getenv("CFD_BERICHT_FENSTER")); // Mittelungsfenster [s Physik]
	std::vector<double> ber_t, ber_cd, ber_cz; double ber_next = -1.0; ulong ber_n_aus=0ull;
	std::ofstream ber_csv;
	// ★ AKTIVES SLOT-LOGGING (Heiko 30.08.: "kann man da nicht irgendwie ein aktives Logging
	// waehrend des Laufs einbauen?"). Anlass: p4_fp16s_voll starb bei Schritt 5381 mit
	// CL_OUT_OF_RESOURCES, und die Rueckfallquote (Slot 69 / Wirkpfad) war mit ihm verloren --
	// die Zaehler wurden bisher NUR im Abschlussbericht gelesen. 70 uint vom Geraet kosten
	// nichts; sie werden jetzt an jedem [BERICHT] mitgelesen, kumuliert UND als Fensterdelta.
	/* ★ 04.09. (Audit): rho_clamp_hits ist seit lbm.cpp:354 96 gross -- Feld und Schleife standen noch auf 80 und schnitten die neuen Slots ab */ ulong slot_alt[96]; for(uint i=0u;i<96u;i++) slot_alt[i]=0ull; bool slot_alt_da=false;
	std::ofstream slot_csv;
	// ★ ZENSUS Mehrfachfacetten (27.08.): Zellklassen der Kraftschleife + Kraftgewicht der
	// "unklaren" Zellen. Die Zaehler sind geometrisch und ueber alle Samples konstant -- der
	// letzte Stand genuegt; das Kraftgewicht ist nur im Host-Pfad belastbar (FacKraft.ukraft_ok).
	ulong zen_voll=0ull, zen_proj=0ull, zen_unklar=0ull; double zen_ux=0.0, zen_uz=0.0, zen_px=0.0, zen_pz=0.0;
	bool zen_da=false, zen_ukraft=false;
	// ★ P8/P9 Schritt 0: INTERFACE-DRUCK-INSTRUMENT (reine AUSGABE, keine Physik, kein Schalter --
	// Muster unterboden_sonde). Die face[p]-Puffer der 4 getriebenen Ebenen liegen nach
	// extract_plane_macros jeden Outer ohnehin auf dem Host (rho an Index 4*i+0); an der
	// Sample-Kadenz wird daraus je Ebene rho_min/mittel/max gezogen und als Delta-p = (rho-1)*cs2
	// (cs2 = 1/3 lattice) via units_coarse.si_p in Pa ausgewiesen. KEIN zusaetzlicher GPU-Read.
	std::ofstream ipcsv(out_dir+"interface_druck.csv"); ipcsv.precision(8);
	ipcsv << "time_s,ebene,rho_min,rho_mittel,rho_max,dp_min_pa,dp_mittel_pa,dp_max_pa\n" << std::flush;
	// ★ BILANZ-CSV des Wake-Kastens (Heiko-Auflage, 2026-08-22). Sechs Begrenzungsflaechen des
	// Kastens auf dem GROBGITTER, an der Sample-Kadenz per extract_plane_macros ausgelesen:
	//   mdot_netto  Summe rho*(u.n)*dA ueber alle sechs Flaechen. SOLL ~ 0. Die Abweichung IST die
	//               kuenstliche Massenquelle, die der Blend erzeugt (er setzt u und laesst rho
	//               lokal, damit ist div(rho*u) im Rampengebiet nicht mehr null).
	//   Fx_bilanz   Impulsfluss + Druckanteil in x ueber dieselben Flaechen. Gegen die Fahrzeug-
	//               kraft zu lesen: konvergiert er dagegen, ist die Aufpraegung konsistent;
	//               schiesst er darueber hinaus, zaehlen Fernfeld-Koerper und Blend DOPPELT.
	// Kein zusaetzlicher Kernel -- extract_plane_macros liefert (rho,u) jeder achsennormalen Ebene.
	std::ofstream bilcsv;
	if(bil_an) {
		bilcsv.open(out_dir+"band_bilanz.csv"); bilcsv.precision(8);
		bilcsv << "# Vorzeichen: Fx_summe ist die LINKE Seite der Impulsbilanz und traegt damit das umgekehrte Vorzeichen von Fx_fein/Fx_grob -- -Fx_summe gegen die Fahrzeugkraft stellen. Es fehlen bewusst der instationaere Term d/dt Int(rho*u_x)dV (erst nach t_warmup lesen) und der Reibanteil. rho_min/rho_max werden NACH dem Flag-Filter nur ueber FLUID gebildet (Solid-Test ueber flags seit 1f851ea); n_verworfen zaehlt die uebersprungenen Nicht-Fluid-Zellen. mdot_*-Spalten sind GITTEREINHEITEN (rho_lat*u_lat je Zellflaeche); nur die *_N-Spalten sind SI-Newton. Fx_xp_N ist der Fluss durch die stromabwaertige Ebene ALLEIN (Bauplan-Abnahmekriterium).\n";
		bilcsv << "time_s,mdot_xm,mdot_xp,mdot_ym,mdot_yp,mdot_zm,mdot_zp,mdot_netto,mdot_netto_rel,Fx_impuls_N,Fx_druck_N,Fx_summe_N,Fx_fein_N,Fx_grob_N,rho_min,rho_max,Fx_xp_N,n_verworfen,n_fluid,n_an_klemme\n" << std::flush;
		print_info("N2F-BAND BILANZ aktiv: Massen- und x-Impulsbilanz ueber die sechs Begrenzungsflaechen des Wake-Kastens (grob x["+to_string(bil_x0)+".."+to_string(bil_x1)+"] y["+to_string(bil_y0)+".."+to_string(bil_y1)+"] z["+to_string(bil_z0)+".."+to_string(bil_z1)+"]) an der Sample-Kadenz -> "+out_dir+"band_bilanz.csv. mdot_netto SOLL ~ 0; die Abweichung ist die kuenstliche Massenquelle des Blends. VORZEICHEN: Fx_summe ist die linke Seite der Impulsbilanz und traegt damit das UMGEKEHRTE Vorzeichen von Fx_fein/Fx_grob -- also -Fx_summe gegen die Fahrzeugkraft lesen; schiesst er darueber, zaehlen Fernfeld-Koerper und Blend doppelt. Der instationaere Term d/dt Integral(rho*u_x)dV fehlt bewusst, erst nach t_warmup lesen.");
	}
	print_info("INTERFACE-DRUCK-Instrument aktiv: rho-Statistik der 4 getriebenen Kopplungsebenen (x-, y-, y+, z+) an der Sample-Kadenz, Delta-p = (rho-1)*cs2 in Pa -> "+out_dir+"interface_druck.csv (reine Ausgabe aus den vorhandenen Host-Kopplungspuffern).");
	// ★ SONDE einlass_saeule_nah (Iron Rule 5 -- Echtdaten statt PNG-Analysen fuer die Near-Inlet-
	// Schlieren): zwei Feingitter-z-Saeulen bei x_f = 2 und x_f = 10 hinter dem Near-x--Rand,
	// y = fNy/2, ALLE z; ux/uy/uz u_inf-normiert + rho + Solid-Flag. GESCHRIEBEN AM SLICE-HOOK:
	// dort liegen lbm_f.u und lbm_f.flags fuer render_yslice ohnehin schon auf dem Host -- ein
	// eigener u-Read je Sample kaeme sonst OBENDRAUF. Der Hook laeuft nur bei CFD_SLICE_DT>0, die
	// Sonde schreibt damit an der SLICE-Kadenz, nicht an der Sample-Kadenz (dokumentierte
	// Kopplung an den Hook statt eines eigenen Device-Reads). Einziger Zusatz-Read am Hook: rho
	// (1/3 der u-Transfergroesse). Immer aktiv (reine Diagnose, kein Schalter); bei CFD_SLICE_DT=0
	// bleibt nur die Kopfzeile stehen. Welt-x in der Kopfzeile (NEAR_VOR-Fallstrick: die Saeulen
	// wandern mit dem Einlass-Interface -- A/B ueber Welt-x vergleichen, nicht ueber den Index).
	const uint sonde_xf[2] = {2u, 10u};
	std::ofstream sonde_csv(out_dir+"einlass_saeule_nah.csv"); sonde_csv.precision(7);
	sonde_csv << "# Welt-x der Saeulen (NEAR_VOR-Fallstrick -- ueber Welt-x vergleichen, nicht ueber den Index): x_f=2 -> "
	          << (near_x0+2.0f*dx_f) << " m, x_f=10 -> " << (near_x0+10.0f*dx_f) << " m; y = fNy/2 = " << fNy/2u
	          << "; geschrieben an der Slice-Kadenz (aktive Kadenz noetig)\n";
	sonde_csv << "time_s,x_f,z,ux_rel,uy_rel,uz_rel,rho,solid\n" << std::flush;
	print_info("SONDE einlass_saeule_nah aktiv (Iron Rule 5): z-Saeulen x_f=2 (Welt-x "+to_string(near_x0+2.0f*dx_f,3u)+" m) und x_f=10 (Welt-x "+to_string(near_x0+10.0f*dx_f,3u)+" m), y = "+to_string(fNy/2u)+", "+to_string(fNz)+" z-Zellen -> "+out_dir+"einlass_saeule_nah.csv; am SLICE-Hook gekoppelt (nur bei aktiver Slice-Kadenz; einziger Zusatz-Read: rho).");
	// ★ FORK Kraft-Zerlegung nach z-Region (Heiko-Vorgabe): CFD_KRAFT_ZBAND = unterste N Zellen ab z=0
	// (inkl.) vs Rest. unset/0 = AUS = bitidentisch (null neue Kernelaufrufe/Logzeilen/Dateien).
	// EINMAL gelesen, nicht je Zelle/Sample (env-Read-Falle).
	const uint zb = env_u("CFD_KRAFT_ZBAND", 0u);
	std::ofstream zcsv;
	double zb_cd_band=0.0, zb_cz_band=0.0, zb_cd_rest=0.0, zb_cz_rest=0.0, zb_selftest_max=0.0; ulong zb_nn=0ull;
	std::vector<double> zb_cz_rest_reihe; // fuer Block-SEM 4/8/16
	if(zb>0u) {
		if(zb>=fNz) print_error("CFD_KRAFT_ZBAND ("+to_string(zb)+") >= fNz ("+to_string(fNz)+") -- das Band muss unter der Domaenendecke bleiben.");
		// Einmaliger Band-Census der 0x41-Zellen aus dem Host-flags-Spiegel (Muster Facettenbau-Census):
		// zeigt zugleich, dass z=0 leer ist (die Kontaktflaechen-Uebergabe hebt Fahrzeugzellen auf z>=1).
		ulong zc_band=0ull, zc_z0=0ull, zc_ges=0ull;
		for(ulong n2=0ull; n2<lbm_f.get_N(); n2++) if(lbm_f.flags[n2]==(TYPE_S|TYPE_X)) {
			zc_ges++; const uint zz=(uint)(n2/((ulong)fNx*(ulong)fNy)); if(zz<zb) zc_band++; if(zz==0u) zc_z0++;
		}
		print_info("KRAFT-ZBAND aktiv: unterste "+to_string(zb)+" Zellen = "+to_string((float)zb*dx_f*1000.0f,2u)+" mm (dx = "+to_string(dx_f*1000.0f,2u)
			+" mm); Band-Census 0x41: "+to_string(zc_band)+" von "+to_string(zc_ges)+" Zellen, davon z=0: "+to_string(zc_z0)+" (Soll 0).");
		print_warning("GITTERBAND -- zwischen DX-Sprossen nicht direkt vergleichbar (Bandhoehe skaliert mit dx, nicht mit der Geometrie).");
		if(zc_band==0ull) print_warning("KRAFT-ZBAND: Band-Census = 0 -- die Zerlegung liefert nur Nullen im Band.");
		zcsv.open(out_dir+"kraft_zband.csv"); zcsv.precision(8);
		zcsv << "# zband_zellen=" << zb << " dx_mm=" << dx_f*1000.0f << " band_mm=" << (float)zb*dx_f*1000.0f
		     << " -- GITTERBAND, zwischen DX-Sprossen nicht direkt vergleichbar\n";
		zcsv << "# ACHTUNG: Fx/Fz aus object_force -- an facettenbehandelten Links PHANTOM-Reibung; fuer A/B nur die VERSCHIEBUNG zwischen Armen werten\n";
		zcsv << "time_s,Fx_band_N,Fz_band_N,Fx_rest_N,Fz_rest_N,Cz_band,Cz_rest,selbsttest_rel,cz_druck_band,cz_druck_rest\n" << std::flush;
	}
	if(env_u("CFD_KOPPLUNG_BODENBAND", 0u)>0u) print_info("BODENBAND-Messarm aktiv: unterste "+to_string(env_u("CFD_KOPPLUNG_BODENBAND",0u))+" Grobzeilen der x--Einlasskopplung: DEFIZIT-ANHEBUNG fmax(u_far, w*u_inf), Rampe bis 2N (B4-Korrektur: keine Ersetzung) -- OF13-Befund.");
	// ★ EBENEN-GLAETTUNG gegen die Near-Inlet-Schlieren (Diagnose-Verdikt A, 2026-08-19): die Far-
	// Streifen wachsen bis zur Entnahmeebene nach (Faktor 24, lambda ~ 3 Grobzellen) und das Feingitter
	// verstaerkt sie auf lambda = ratio. CFD_KOPPLUNG_GLATT = Anzahl 1-2-1-Binomialdurchgaenge ueber die
	// EXTRAHIERTEN Ebenendaten face[p] (alle 4 Komponenten rho/ux/uy/uz, separabel erst a-, dann
	// b-Richtung, Raender einseitig geklemmt), NUR auf den getriebenen Ebenen. 0/unset = kein Filter =
	// bitidentisch. Der 1-2-1-Kern daempft lambda = 3 Zellen pro Durchgang und Richtung auf ~25 % --
	// ein Durchgang trifft also genau die diagnostizierte Stoerwellenlaenge, DC-Anteil bleibt exakt.
	// REIHENFOLGE (nachgeprueft, siehe Zeitschleife): Filter VOR dem BODENBAND-Messarm (der hebt
	// face-Index 4i+1 NACH der Extraktion an -- der Filter darf die Kante dieser Anhebung nicht
	// verschmieren) und VOR dem Drive (der Wirksamkeitsnachweis vergleicht die feinen Deckungspunkte
	// gegen DENSELBEN face[p]-Host-Puffer, bleibt also bit-konsistent -- gleiche Logik wie beim
	// Bodenband). Das Interface-Druck-Instrument liest face[p] am ENDE des Outers (frisch extrahiert,
	// Filter laeuft erst am Anfang des naechsten) und bleibt damit ebenfalls unveraendert.
	const uint glatt_n = env_u("CFD_KOPPLUNG_GLATT", 0u);
	std::vector<float> glatt_tmp;
	if(glatt_n>0u) {
		glatt_tmp.resize((ulong)max_cp*4ull);
		string glatt_ebenen = "";
		for(uint p=0u; p<5u; p++) if(drive_face[p]) glatt_ebenen += string(glatt_ebenen.empty()?"":", ")+face_name[p];
		print_info("KOPPLUNG-GLATT aktiv: "+to_string(glatt_n)+" 1-2-1-Binomialdurchgaenge (a- und b-Richtung, Raender einseitig geklemmt) ueber rho/ux/uy/uz der getriebenen Kopplungsebenen ("+glatt_ebenen+"), im Kopplungsfenster VOR Bodenband und Drive -- Near-Inlet-Schlieren-Filter (Diagnose-Verdikt A).");
		print_info("KOPPLUNG-GLATT-Hinweis zum Wirksamkeitsnachweis: jede Ebene wird UNABHAENGIG gefiltert; auf den GETEILTEN Kantenspalten (x-Einlasskante der y-/z+-Ebenen) gewinnt der zuletzt getriebene x--Wert. Der Nachweis der y-/z+-Ebenen darf dort deshalb in der 1e-4-GROESSENORDNUNG abweichen (CPU-Abnahme 2026-08-19: max 6,6e-4, ausschliesslich Kantenspalten x=0; x- selbst bleibt bitgenau). Er bricht nicht hart ab -- kein Skip noetig.");
	}
	// ★ ZEITINTERP Stufe 1 (Host-Mix, Planungsagenten-Plan) gegen die ratio-EIGENANTWORT der Kopplung:
	// der Altpfad treibt die TYPE_E-Raender EINMAL je Kopplungsfenster mit dem groben END-Stand und
	// haelt sie dann ratio feine Schritte fest -- ein Treppenhalteglied, dessen Sprungantwort das
	// Feingitter mit lambda = ratio Zellen quittiert (Near-Inlet-Schlieren-Verdacht). Mit
	// CFD_KOPPLUNG_ZEITINTERP=1 laeuft der Drive stattdessen je feinem Substep s = 1..ratio mit dem
	// Host-Mix w = s/ratio zwischen dem vorigen (face_alt) und dem aktuellen Fenster-Stand (face).
	// Der LETZTE Substep (w == 1) uebergibt face[p] UNVERMISCHT -- Bit-Anker: der Kopplungs-Verify
	// vergleicht die Deckungspunkte gegen genau diesen Puffer. Erster Outer = w=1-Fallback
	// (face_alt noch leer), bitidentisch zum Altpfad. 0/unset = exakt bisheriges Verhalten.
	// EINMAL gelesen (env-Read-Falle).
	const uint zinterp = env_u("CFD_KOPPLUNG_ZEITINTERP", 0u);
	if(zinterp>0u) {
		face_mix.resize((ulong)max_cp*4ull);
		print_info("KOPPLUNG-ZEITINTERP aktiv (Stufe 1, Host-Mix): je Kopplungsfenster "+to_string(ratio)+" Substeps, Ebenen-Drive mit w = s/"+to_string(ratio)+" linear zwischen vorigem und aktuellem Fenster-Stand; Substep w = 1 treibt face[] UNVERMISCHT (Bit-Anker fuer den Kopplungs-Verify); erster Outer = w=1-Fallback (bitidentisch zum Altpfad).");
	}
	if(env_u("CFD_FACETTEN", 0u)>0u) print_info("C7-Notiz: Facetten-Schubspannung = Impulssenke (Gleichgewichtsmodell, kein APG) -- Abloeselage bleibt modellfrei; Cd/Cz-Bewegung dokumentieren, nicht versprechen.");
	// ★ B2: Wandprofil ueber die Zeit, je Gitter eine Saeule und eine CSV (Begruendung bei
	// schreibe_wandprofil). Die Saeule wird EINMAL gesucht -- die Flags sind nach initialize statisch.
	uint wp_fx=0u, wp_fy=0u, wp_cx=0u, wp_cy=0u;
	const bool wp_f_ok = finde_messsaeule(lbm_f, fNx, fNy, fNz, wp_fx, wp_fy);
	const bool wp_c_ok = finde_messsaeule(lbm_c, cNx, cNy, cNz, wp_cx, wp_cy);
	if(wp_f_ok&&near_vor>0.0f) print_info("Messsaeule nah bei Welt-x "+to_string(near_x0+(float)wp_fx*dx_f,3u)+" m (wandert mit fNx -- A/B ueber Welt-x vergleichen, NEAR_VOR-Fallstrick).");
	std::ofstream wpf(out_dir+"wandprofil_nah.csv"), wpc(out_dir+"wandprofil_fern.csv");
	wpf.precision(6); wpc.precision(6);
	wpf << "time_s,z1,z2,z3,z4,z5,z6,z7\n"; wpc << "time_s,z1,z2,z3,z4,z5,z6,z7\n";
	if(!wp_f_ok) print_warning("Nahfeld: keine freie Messsaeule -- Wandprofil entfaellt.");
	if(!wp_c_ok) print_warning("Fernfeld: keine freie Messsaeule -- Wandprofil entfaellt.");
	const ulong verify_at2 = min(n_outer>0ull ? n_outer-1ull : 0ull, (ulong)env_u("CFD_DD_VERIFY_AT", 200u));

	// ---------------------------------------------------------------- Leistungsindex und Phasenprofil
	// ★ Aus V1 nachgezogen (Heiko), dort knowledge/performance.md. Zwei Groessen, die V2 bisher fehlten:
	//
	//  LEISTUNGSINDEX = Wandsekunden je physikalischer Sekunde. Kleiner ist besser. Er ist die einzige
	//  Zahl, die ueber Aufloesungen und Gitterverhaeltnisse hinweg vergleichbar bleibt -- MLUPs sind es
	//  nicht, weil sie die Zellzahl schon eingerechnet haben und bei zwei Domaenen ohnehin unbrauchbar
	//  werden (die Konsolenanzeige mischt die grobe Zellzahl mit der feinen Schrittzeit).
	//  V1-Referenz: Standardlauf 2026-06-23, dieselbe Konfiguration, Index ~12.000.
	//
	//  PHASENPROFIL: wo die Zeit WIRKLICH sitzt. V1s Lehre dazu steht in performance.md und ist teuer
	//  bezahlt: ein Theorie-Audit suchte den Hebel im Async-Overlap, waehrend die groesste Quelle eine
	//  96-MB-Verschwendung je Transfer war -- die Messung fand sie in Minuten, das Audit gar nicht.
	//  Deshalb wird hier gemessen, nicht schaetzt.
	auto t_now = []() { return std::chrono::steady_clock::now(); };
	double ph_kopplung=0.0, ph_fein=0.0, ph_grob=0.0, ph_kraft=0.0, ph_schnitt=0.0; // Sekunden, seit dem letzten Bericht
	ulong ph_n = 0ull;
	auto wall_begin = t_now();
	double t_phys_begin = 0.0;

	for(ulong outer=0ull; outer<n_outer; outer++) {
		outer_clock.start();
		const auto _t0 = t_now();
		lbm_c.run_async(1u);
		// ★ KOPPLUNG-GLATT (Diagnose-Verdikt A): 1-2-1-Binomialfilter ueber die im VORIGEN Outer
		// extrahierten face[p]-Hostpuffer, ZWISCHEN Extraktion und Lift. Bewusst HIER und nicht direkt
		// nach dem Extract am Outer-Ende: (1) VOR dem BODENBAND-Block darunter -- dessen fmax-Anhebung
		// (Index 4i+1) behaelt so ihre scharfe Kante; (2) das Interface-Druck-Instrument an der
		// Sample-Kadenz liest so weiter die UNGEFILTERTEN Extraktionswerte (Diagnose bleibt ehrlich);
		// (3) der Wirksamkeitsnachweis unten vergleicht gegen DIESEN (gefilterten) Puffer und bleibt
		// bit-konsistent. Separabel: erst a-Richtung face->glatt_tmp, dann b-Richtung glatt_tmp->face;
		// Raender einseitig geklemmt (f[-1]:=f[0]). Host-Kosten: 4 Ebenen x 4 floats, vernachlaessigbar
		// gegen die 304-kB-Device-Reads derselben Schleife.
		if(glatt_n>0u) for(uint p=0u; p<5u; p++) {
			if(!drive_face[p]) continue;
			const ulong ea=(ulong)cp[p].extent_a, eb=(ulong)cp[p].extent_b;
			for(uint pass=0u; pass<glatt_n; pass++) {
				for(ulong b=0ull; b<eb; b++) for(ulong a=0ull; a<ea; a++) { // a-Richtung
					const ulong am=(a>0ull?a-1ull:0ull), ap=(a+1ull<ea?a+1ull:ea-1ull);
					const ulong i=(a+b*ea)*4ull, im=(am+b*ea)*4ull, ip=(ap+b*ea)*4ull;
					for(uint c=0u; c<4u; c++) glatt_tmp[i+c] = 0.25f*(face[p][im+c]+2.0f*face[p][i+c]+face[p][ip+c]);
				}
				for(ulong b=0ull; b<eb; b++) for(ulong a=0ull; a<ea; a++) { // b-Richtung
					const ulong bm=(b>0ull?b-1ull:0ull), bp=(b+1ull<eb?b+1ull:eb-1ull);
					const ulong i=(a+b*ea)*4ull, im=(a+bm*ea)*4ull, ip=(a+bp*ea)*4ull;
					for(uint c=0u; c<4u; c++) face[p][i+c] = 0.25f*(glatt_tmp[im+c]+2.0f*glatt_tmp[i+c]+glatt_tmp[ip+c]);
				}
			}
		}
		// ★ BODENBAND-Messarm (Heiko/OF13-Befund 2026-08-19): das Fernfeld liefert am x-Einlass eine
		// kollabierte Bodenschicht (0,63 u_inf bei -2,4 m), weil sein 16-mm-Spalt praktisch zu ist --
		// OF13 zeigt dort 1,2-1,4 u_inf (postProcessing/sampleY0). CFD_KOPPLUNG_BODENBAND=N ersetzt in
		// der GROBEN x--Einlassflaeche die untersten N Zellreihen per fmax auf w*u_inf HEBEN (Rampe bis
		// 2N), NUR ux/uy/uz -- rho bleibt Fernfeld. 0/ungesetzt = exakt bisheriges Verhalten.
		{	static const uint bb_n = min(env_u("CFD_KOPPLUNG_BODENBAND", 0u), cez/2u>1u?cez/2u-1u:0u); // R2-2: Band+Rampe muessen unter der Decke bleiben (sonst z+-Verify-Schein); LATENT (B9): static ueberlebt zweiten Fall-Aufruf im Prozess; fmax maskiert zudem Fernfeld-NaN im Band still (statt Bit-Test im Drive-Kernel)
			if(bb_n>0u) { const uint ea=cp[0].extent_a; // Ebene x-: a=y, b=z (gid = a + b*ea)
				for(uint b=0u; b<min(2u*bb_n, cp[0].extent_b); b++) {
					const float w_bb = (b<bb_n) ? 1.0f : 1.0f-(float)(b-bb_n+1u)/(float)(bb_n+1u); // 1 im Band, Rampe darueber
					for(uint a=0u; a<ea; a++) { const ulong e4=((ulong)a+(ulong)b*(ulong)ea)*4ull;
						// ★ Lauf-4/6-Lehre (s5d_aus_bb8): harte Ersetzung ueberfuettert (Cd 1,07-Strahl).
						// DEFIZIT-ANHEBUNG: nur kollabierte Werte auf w*u_inf heben, gesunde Fernfeld-
						// Struktur (inkl. uy/uz) bleibt unangetastet.
						face[0][e4+1ull] = fmax(face[0][e4+1ull], w_bb*u_lat); }
				}
				// B3-1: dieselbe Anhebung auf den GETEILTEN Kantenspalten der y-Ebenen (a=0 = x-Einlass-
				// Ecke; Achse 1 spannt a=x, b=z) -- sonst wandert die 1-Zellen-Naht nur zum y-Verify.
				for(uint p2=2u; p2<4u; p2++) {
					const uint ea2=cp[p2].extent_a;
					for(uint b2=0u; b2<min(2u*bb_n, cp[p2].extent_b); b2++) {
						const float w2 = (b2<bb_n) ? 1.0f : 1.0f-(float)(b2-bb_n+1u)/(float)(bb_n+1u);
						const ulong e42=((ulong)0u+(ulong)b2*(ulong)ea2)*4ull;
						face[p2][e42+1ull] = fmax(face[p2][e42+1ull], w2*u_lat);
					}
				}
			}
		}
		// ★ ZEITINTERP Stufe 1 (Host-Mix): Altpfad (zinterp=0 ODER erster Outer, face_alt noch leer =
		// w=1-Fallback, bitidentisch) treibt EINMAL mit dem Fenster-Endstand und laesst ratio Schritte
		// laufen. Sonst laeuft je Substep s = 1..ratio erst der Drive mit dem Host-Mix w = s/ratio
		// (face_mix = (1-w)*face_alt + w*face), dann EIN feiner Schritt; der letzte Substep (w == 1)
		// uebergibt face[p] UNVERMISCHT -- Bit-Anker fuer den Kopplungs-Verify unten. Ebenen-Reihenfolge
		// je Substep wie im Altpfad {2,3,4,0}: y/z zuerst, x- ZULETZT (Tiefen-Audit B5 -- sonst
		// ueberschreiben die y-Ebenen die gelifteten Kantenspalten, 1-Zellen-Naht, und der
		// Kopplungsnachweis meldet Schein-Abweichungen).
		const bool zi_alt = (zinterp==0u) || face_alt[0].empty();
		if(zi_alt) {
			for(uint p=2u; p<5u; p++) if(drive_face[p]) lbm_f.drive_boundary_from_coarse(fp[p], face[p], cp[p].extent_a, cp[p].extent_b, ratio); // Tiefen-Audit B5: y/z zuerst...
			if(drive_face[0]) lbm_f.drive_boundary_from_coarse(fp[0], face[0], cp[0].extent_a, cp[0].extent_b, ratio); // ...x- ZULETZT -- sonst ueberschreiben die y-Ebenen die gelifteten Kantenspalten (1-Zellen-Naht) und der Kopplungsnachweis meldet Schein-Abweichungen
		}
		const auto _t1 = t_now();
		double zi_drive = 0.0; // Drive-/Mix-Anteil des Substep-Loops -- wird unten ph_kopplung zugebucht (nicht ph_fein)
		if(zi_alt) {
			lbm_f.run((ulong)ratio, n_outer*(ulong)ratio);
		} else {
			static const uint zi_reihe[4] = {2u, 3u, 4u, 0u}; // wie Altpfad: y/z zuerst, x- zuletzt
			for(uint s=1u; s<=ratio; s++) {
				const float w = (float)s/(float)ratio; // s == ratio -> w == 1.0f exakt
				const auto _tz0 = t_now();
				for(uint zi_p=0u; zi_p<4u; zi_p++) { // (pi ist ein Makro -- utilities.hpp)
					const uint p = zi_reihe[zi_p];
					if(!drive_face[p]) continue;
					if(w==1.0f) {
						lbm_f.drive_boundary_from_coarse(fp[p], face[p], cp[p].extent_a, cp[p].extent_b, ratio); // UNVERMISCHT (Bit-Anker)
					} else {
						const ulong nn = (ulong)cp[p].extent_a*(ulong)cp[p].extent_b*4ull;
						for(ulong i=0ull; i<nn; i++) face_mix[i] = (1.0f-w)*face_alt[p][i] + w*face[p][i];
						lbm_f.drive_boundary_from_coarse(fp[p], face_mix, cp[p].extent_a, cp[p].extent_b, ratio);
					}
				}
				zi_drive += std::chrono::duration<double>(t_now()-_tz0).count();
				lbm_f.run(1u, n_outer*(ulong)ratio);
			}
		}
		const auto _t2 = t_now();

		// ------------------------------------------------------------ Wirksamkeitsnachweis (einmal)
		// "Laeuft" ist nicht "wirkt". Diese Pruefung beantwortet beides getrennt:
		//  (A) KOMMT ES AN? An jedem Deckungspunkt (a und b beide Vielfache von ratio) ist die kubische
		//      Interpolation die IDENTITAET. Der Nahfeld-Randwert muss dort also BIT-GENAU dem groben
		//      Wert entsprechen, den der Host verschickt hat. Das prueft die ganze Kette in einem:
		//      Entnahme-Kernel, Indexabbildung, Host-Transfer, Upload, Gewichte, Schreibpfad.
		//  (B) BEWIRKT ES ETWAS? Waere die Kopplung ein No-op, stuenden die TYPE_E-Zellen exakt auf
		//      ihrem Anfangswert u_x = u_lat. Die groesste Abweichung davon ist das Mass dafuer, dass
		//      das Fernfeld tatsaechlich etwas ANDERES vorgibt als ungestoerten Freistrom.
		//  (C) WIDERSPRECHEN SICH ZWEI FLAECHEN AN IHRER KANTE? Die fuenf feinen Ebenen ueberlappen sich
		//      an den Kanten, und die spaeter aufgerufene ueberschreibt die fruehere. Diese Pruefung
		//      deckt das MIT AB, ohne dass es dafuer eigenen Code braucht: die Kantenzellen der
		//      x--Ebene werden anschliessend von y-/y+ erneut beschrieben; wenn der Vergleich der
		//      x--Ebene gegen IHRE grobe Quelle danach noch bit-genau aufgeht, haben beide dasselbe
		//      geschrieben. Gemessen am 2026-08-08: geht auf. Das war vorher nur ein Argument.
		// TYPE_E-Zellen behalten rho/u ueber die Unterschritte unveraendert (stream_collide aktualisiert
		// die Felder nur fuer Nicht-TYPE_E), die Werte hier sind also genau die geschriebenen.
		// ★ ZEITINTERP-PRAEZISIERUNG: auch mit CFD_KOPPLUNG_ZEITINTERP=1 gilt (A) BIT-GENAU gegen
		// face[p] -- der LETZTE Substep des Fensters (w == 1) treibt face[p] UNVERMISCHT (Bit-Anker,
		// siehe Drive/Run-Block oben), und die TYPE_E-Zellen tragen hier den Stand des letzten Drives.
		// Bei outer == 0 laeuft ohnehin der w=1-Fallback (face_alt leer, Altpfad). Die Zwischen-Mixe
		// der Substeps s < ratio sind zum Verify-Zeitpunkt bereits ueberschrieben und tauchen hier
		// nicht auf -- "identisch" beweist also die Kette, NICHT die Abwesenheit der Zwischen-Drives.
		// Zweimal geprueft, und das aus einem Grund: beim ERSTEN groben Schritt ist das Fernfeld noch
		// ueberall Freistrom, dort kann (B) gar nichts anderes zeigen als "gleich u_inf" -- eine
		// wirkungslose Kopplung waere von einer wirksamen nicht zu unterscheiden. (A) dagegen greift
		// sofort. Der zweite Zeitpunkt liegt weit genug hinten, dass das Fernfeld das Fahrzeug spuert.
		if(outer==0ull || outer==verify_at2) {
			lbm_f.u.read_from_device(); lbm_f.rho.read_from_device(); lbm_f.flags.read_from_device();
			// ★★ Nachpruefer-Befund 2026-08-09, und der schmerzhafteste: der Wirksamkeitsnachweis der
			// mitbewegten Wand lief in ALLEN Faellen -- NUR NICHT HIER. Also ausgerechnet nicht im
			// Produktionsfall, aus dem Cd und Cz gegen OpenFOAM fallen; die anderen drei sind
			// Diagnosefaelle. Und er kostet hier NULL zusaetzliche Rueckwaerts-Reads, weil u und flags
			// in der Zeile darueber ohnehin schon geholt werden. Meine Commit-Botschaft sagte
			// "in allen vier Faellen" und meinte damit nur den statischen Teil.
			// ★ NUR das Nahfeld hier. Das Fernfeld laeuft ASYNCHRON (run_async weiter oben) und ist an
			// dieser Stelle noch nicht fertig -- sein Nachweis steht nach lbm_c.finish().
			if(outer==0ull) pruefe_wandwirksamkeit(lbm_f, fNx, fNy, fNz, u_lat, "Nahfeld");
			for(uint p=0u; p<5u; p++) {
				if(!drive_face[p]) continue;
				ulong n_e=0ull, n_coin=0ull, n_bad=0ull, n_outlet_edge=0ull; float maxdev=0.0f, maxrel=0.0f;
				uint bx=0u, by=0u, bz=0u; // Ort der ersten Abweichung -- eine Zahl allein sagt nicht, WO es klemmt
				for(uint b=0u; b<fp[p].extent_b; b++) for(uint a=0u; a<fp[p].extent_a; a++) {
					uint x=fp[p].origin.x, y=fp[p].origin.y, z=fp[p].origin.z;
					if(fp[p].axis==0u)      { y+=a; z+=b; }
					else if(fp[p].axis==1u) { x+=a; z+=b; }
					else                    { x+=a; y+=b; }
					const ulong n = (ulong)x + ((ulong)y + (ulong)z*(ulong)fNy)*(ulong)fNx;
					if((lbm_f.flags[n]&(TYPE_S|TYPE_E))!=TYPE_E) continue; // Fahrbahn und Fahrzeug tragen hier nichts bei
					const bool on_outlet = (x==fNx-1u); // Auslasskante -- siehe Begruendung unten
					if(!on_outlet) { n_e++; maxrel = fmax(maxrel, fabs(lbm_f.u.x[n]-u_lat)/u_lat); }
					if(a%ratio!=0u || b%ratio!=0u) continue; // kein Deckungspunkt -> hier ist die Interpolation nicht die Identitaet
					// ★ Die Auslasskante wird GEZAEHLT, aber nicht bewertet, und zwar begruendet:
					// apply_pressure_outlet schreibt rho und u auf x = fNx-1 in JEDEM Zeitschritt und damit
					// NACH der Kopplung. Dort steht folglich der Auslasswert, nicht der geliftete.
					// Nachgemessen am 2026-08-08: ohne diese Ausnahme meldete die Pruefung 15 Abweichungen
					// auf den y-Flaechen und 21 auf z+ -- ausnahmslos bei x = fNx-1, also genau die Anzahl der
					// Deckungspunkte dieser einen Kante. Das ist die gewollte Rangfolge: am Auslass gilt der Auslass.
					if(on_outlet) { n_outlet_edge++; continue; }
					{
						n_coin++;
						const ulong cb = ((ulong)(b/ratio)*(ulong)cp[p].extent_a + (ulong)(a/ratio))*4ull;
						const float d = fmax(fmax(fabs(lbm_f.rho[n]-face[p][cb]), fabs(lbm_f.u.x[n]-face[p][cb+1ull])),
						                     fmax(fabs(lbm_f.u.y[n]-face[p][cb+2ull]), fabs(lbm_f.u.z[n]-face[p][cb+3ull])));
						maxdev = fmax(maxdev, d);
						if(d>1.0e-6f) { if(n_bad==0ull) { bx=x; by=y; bz=z; } n_bad++; }
					}
				}
				print_info(string("[KOPPLUNG ")+face_name[p]+"] "+to_string(n_e)+" TYPE_E-Zellen, davon "+to_string(n_coin)
					+" Deckungspunkte (plus "+to_string(n_outlet_edge)+" auf der Auslasskante, dort gilt der Auslass); groesste Abweichung dort "+to_string(maxdev,9u)
					+(n_bad? (" -- "+to_string(n_bad)+" ueber 1e-6, erste bei ("+to_string(bx)+","+to_string(by)+","+to_string(bz)+") von ("+to_string(fNx-1u)+","+to_string(fNy-1u)+","+to_string(fNz-1u)+")") : " (identisch)")
					+"; groesste Abweichung vom Freistrom "+to_string(100.0f*maxrel,2u)+" % von u_inf");
				if(n_e==0ull) print_warning(string("Flaeche ")+face_name[p]+" hat KEINE TYPE_E-Zelle -- diese Kopplungsflaeche ist wirkungslos.");
				if(maxrel<1.0e-6f) print_warning(string("Flaeche ")+face_name[p]+" steht exakt auf Freistrom -- das Fernfeld gibt dort (noch) nichts Eigenes vor.");
			}
		}

		lbm_c.finish();
		// ★★ HIER und nicht frueher, und das hat der Nachweis selbst aufgedeckt: erst stand er oben im
		// Pruefblock -- also ZWISCHEN lbm_c.run_async() und lbm_c.finish(). Damit wurde das grobe u
		// gelesen, WAEHREND sein Schritt noch lief, und das Profil kam als gleichfoermige 0,836 heraus
		// (dasselbe Gitter allein gerechnet liefert 1,000). Ein Wettlauf, kein physikalischer Befund --
		// erkennbar daran, dass das Profil ueber alle z KONSTANT war statt zur Freistroemung anzusteigen.
		if(outer==0ull) pruefe_wandwirksamkeit(lbm_c, cNx, cNy, cNz, u_lat, "Fernfeld");
		// ★ ZEITINTERP: getriebener Fenster-Endstand wird zum face_alt des NAECHSTEN Fensters --
		// zwingend VOR dem Extract (der ueberschreibt face[p] mit dem neuen groben Stand). Kopiert
		// wird der Stand NACH GLATT-Filter und BODENBAND-Anhebung dieses Fensters: exakt das, was
		// bei w = 1 getrieben wurde -- der Mix des naechsten Fensters startet also stetig dort.
		if(zinterp>0u) for(uint p=0u; p<5u; p++) if(drive_face[p]) face_alt[p] = face[p];
		for(uint p=0u; p<5u; p++) if(drive_face[p]) lbm_c.extract_plane_macros(cp[p], face[p]); // nur die vier getriebenen Flaechen -- x+ ist Druckauslass, siehe oben
		if(n2f_alpha>0.0f) { // ★ P9c KOPPLUNGSFENSTER (nach lbm_c.finish() + Extracts, VOR dem naechsten run_async): Nahfeld-Blockmittel entnehmen und ins Fernfeld laden -- 1-Outer-Vorlauf wie die Hinkopplung; das Blend des naechsten groben Schritts liest diesen Stand
			lbm_f.schale_extract_u(n2f_unear, n2f_mittel);
			lbm_c.schale_upload_unear(n2f_unear);
		}
		const auto _t3 = t_now();
		t_acc += outer_clock.stop(); n_acc++;
		ph_kopplung += std::chrono::duration<double>(_t1-_t0).count() + zi_drive; // ZEITINTERP: Drive-/Mix-Zeit des Substep-Loops gehoert zur Kopplung...
		ph_fein     += std::chrono::duration<double>(_t2-_t1).count() - zi_drive; // ...und wird dem feinen Fenster wieder abgezogen (zi_drive = 0 im Altpfad)
		ph_grob     += std::chrono::duration<double>(_t3-_t2).count();
		ph_n++;

		if((outer+1ull)%(ulong)sample_every==0ull) {
			const auto _t4 = t_now();
			// (Die expliziten update_force_field()-Aufrufe hier waren redundant: enqueue_object_force
			// ruft enqueue_update_force_field intern, mit t_last_force_field-Guard.)
			const float3 F = lbm_f.object_force(TYPE_S|TYPE_X);
			const float3 Fc = lbm_c.object_force(TYPE_S|TYPE_X);
			// ★ KRAFT-ZBAND: Band/Rest SEQUENZIELL nach object_force (object_sum wird wiederverwendet).
			float3 Fb = float3(0.0f, 0.0f, 0.0f), Fr = float3(0.0f, 0.0f, 0.0f); double zb_rel = 0.0;
			bool zb_fac_now = false; double zb_czdb = 0.0, zb_czdr = 0.0; // Druck-Zerlegung (nur im Facetten-Arm an der Kadenz befuellt)
			if(zb>0u) {
				Fb = lbm_f.object_force_zband((uchar)(TYPE_S|TYPE_X), 0u, zb);
				Fr = lbm_f.object_force_zband((uchar)(TYPE_S|TYPE_X), zb, fNz);
				// Selbsttest je Komponente gegen das vorhandene F; Skala = max(|Fx|,|Fz|), damit der
				// Fz-Nulldurchgang nicht als Scheinfehler explodiert (absolute Toleranz dort).
				const double skala = fmax(fmax(fabs((double)F.x), fabs((double)F.z)), 1e-30);
				zb_rel = fmax(fmax(fabs(((double)Fb.x+(double)Fr.x)-(double)F.x), fabs(((double)Fb.y+(double)Fr.y)-(double)F.y)), fabs(((double)Fb.z+(double)Fr.z)-(double)F.z))/skala; // R1-N2: Fy mitgeprueft
			}
			const double t_si = (double)((float)(outer+1ull)*dt_c);
			t_si_letzt = t_si; // n_outer_ist NICHT hier -- s. Schleifenende (Pruefagent-B-2)
			if(wp_f_ok) schreibe_wandprofil(lbm_f, fNx, fNy, wp_fx, wp_fy, u_lat, t_si, wpf);
			if(wp_c_ok) schreibe_wandprofil(lbm_c, cNx, cNy, wp_cx, wp_cy, u_lat, t_si, wpc);
			// ★ P8/P9 Schritt 0: Interface-Druck aus den face[p]-Puffern. Die stehen hier FRISCH: die
			// Extract-Schleife lief in DIESEM Outer vor dem Sample-Block, und die BODENBAND-Anhebung
			// (naechster Outer) fasst nur Index 4*i+1 (ux) an -- rho (4*i+0) ist der unveraenderte
			// Fernfeld-Wert. Nur Lesen, keine Physik.
			for(uint p=0u; p<5u; p++) {
				if(!drive_face[p]) continue;
				const ulong n_pl = (ulong)cp[p].extent_a*(ulong)cp[p].extent_b;
				const ulong i0 = (p==4u) ? 0ull : (ulong)cp[p].extent_a; // Pruefagent M2: erste b-Zeile (z=0, Fahrbahn-Solid mit eingefrorenem rho=1) maskiert sonst rho_min der Ebenen x-/y-/y+; z+ hat keine Fahrbahn. Mit CFD_FERN_BODENKLEMME>1 blieben weitere eingefrorene TYPE_E-Zeilen enthalten (Diagnose-Arm, bewusst toleriert).
				double s = 0.0; float rmin = face[p][4ull*i0], rmax = rmin;
				for(ulong i=i0; i<n_pl; i++) { const float r = face[p][4ull*i]; s += (double)r; rmin = fmin(rmin, r); rmax = fmax(rmax, r); }
				const double rmean = s/(double)(n_pl-i0);
				const double dpmin  = (double)units_coarse.si_p((rmin-1.0f)/3.0f);
				const double dpmean = (double)units_coarse.si_p((float)(rmean-1.0)/3.0f);
				const double dpmax  = (double)units_coarse.si_p((rmax-1.0f)/3.0f);
				ipcsv << t_si << "," << face_name[p] << "," << rmin << "," << rmean << "," << rmax << ","
				      << dpmin << "," << dpmean << "," << dpmax << "\n" << std::flush;
			}
			if(bil_an) { // ★ BILANZ ueber die sechs Kastenflaechen (Massen- und x-Impulsstrom)
				// Vorzeichenkonvention: n zeigt AUS dem Kasten heraus. mdot > 0 = Ausstrom.
				// Jede Flaeche wird mit extract_plane_macros gelesen; dieselbe Mechanik wie die
				// Kopplungsebenen, nur auf anderen Indizes. rho/u kommen in Gittereinheiten,
				// die Umrechnung nach SI laeuft ueber units_coarse.
				static std::vector<float> bf; // Wiederverwendeter Host-Puffer
				const uint bx=bil_x1-bil_x0+1u, by=bil_y1-bil_y0+1u, bz=bil_z1-bil_z0+1u;
				const PlaneSpec bp[6] = { mk(bil_x0,bil_y0,bil_z0, by,bz, 0u), mk(bil_x1,bil_y0,bil_z0, by,bz, 0u),
				                          mk(bil_x0,bil_y0,bil_z0, bx,bz, 1u), mk(bil_x0,bil_y1,bil_z0, bx,bz, 1u),
				                          mk(bil_x0,bil_y0,bil_z0, bx,by, 2u), mk(bil_x0,bil_y0,bil_z1, bx,by, 2u) };
				const float nrm[6][3] = {{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}};
				double md[6]={0,0,0,0,0,0}, fimp=0.0, fdru=0.0, rmn=1e30, rmx=-1e30, mein=0.0, fimp_xp=0.0, fdru_xp=0.0; ulong nverw=0ull, nfl=0ull, n_ausser=0ull; uint rmn_f=0u, rmx_f=0u;
				const char* fname[6] = {"x-","x+","y-","y+","z-","z+"};
				// ★ SOLID-TEST UEBER DIE FLAGS, NICHT UEBER rho (Korrektur 2026-08-22 nachmittags).
				// Vorher stand hier `if(!(r>0.5&&r<2.0)) continue;` als Solid-Ersatz. Das ist in beide
				// Richtungen falsch: eine Solidzelle mit rho == 1 kommt durch, und eine echte
				// Dichte-Entgleisung wird als "ungueltig" verworfen. Gemessen an b8_nurwake: der
				// Dichte-Waechter brach ab mit min 0,5513 / max 1,1424, BEIDE auf der x--Flaeche --
				// und die schneidet bei wx0 = 209 mitten durch das Fahrzeug. Es war kein
				// Nachlauf-Befund, es war Karosserie. Die Flags liegen auf dem Host (der
				// Listenbauer liest sie), die Geometrie ist statisch -- der Test kostet nichts.
				// Die Indexabbildung ist woertlich die von plane_cell_index (kernel.cpp).
				auto ebenen_idx=[&](const PlaneSpec& q, const ulong i) {
					const uint a=(uint)(i%(ulong)q.extent_a), b=(uint)(i/(ulong)q.extent_a);
					const uint x = q.axis==0u ? q.origin.x : q.origin.x+a;
					const uint y = q.axis==0u ? q.origin.y+a : (q.axis==1u ? q.origin.y : q.origin.y+b);
					const uint z = q.axis==2u ? q.origin.z : q.origin.z+b;
					return (ulong)x+((ulong)y+(ulong)z*(ulong)cNy)*(ulong)cNx;
				};
				for(uint f=0u; f<6u; f++) {
					lbm_c.extract_plane_macros(bp[f], bf);
					const ulong np=(ulong)bp[f].extent_a*(ulong)bp[f].extent_b;
					for(ulong i=0ull; i<np; i++) {
						const double r=(double)bf[4ull*i], ux=(double)bf[4ull*i+1ull], uy=(double)bf[4ull*i+2ull], uz=(double)bf[4ull*i+3ull];
						const ulong nc=ebenen_idx(bp[f], i);
						if(nc>=(ulong)cNx*(ulong)cNy*(ulong)cNz || (lbm_c.flags[nc]&(TYPE_S|TYPE_E))!=0u) { nverw++; continue; } // KEIN Fluid: Karosserie, Fahrbahn oder Kopplungsrand
						nfl++; if(r<=(double)RHO_CLAMP_MIN+0.01||r>=(double)RHO_CLAMP_MAX-0.01) n_ausser++; // Anteil an der Solver-Dichteklemme
						if(r<rmn) { rmn=r; rmn_f=f; } if(r>rmx) { rmx=r; rmx_f=f; } // nur ueber FLUID -- und ohne rho-Fenster, damit eine echte Entgleisung sichtbar bleibt
						const double un = ux*(double)nrm[f][0] + uy*(double)nrm[f][1] + uz*(double)nrm[f][2];
						md[f] += r*un;                                      // Gittereinheiten, dA = 1 Zelle
						if(un<0.0) mein += -r*un;                           // Einstrom getrennt, als Bezugsgroesse
						fimp += r*ux*un;                                    // Impulsfluss in x
						fdru += (r-1.0)/3.0*(double)nrm[f][0];              // Druckanteil: (rho-1)*cs2 * n_x
						if(f==1u) { fimp_xp += r*ux*un; fdru_xp += (r-1.0)/3.0; } // ★ Bauplan Paragraf 4: das Abnahmekriterium ist der Impulsdefizit-Fluss durch die STROMABWAERTIGE Ebene allein -- aus der Summe ueber alle sechs Flaechen ist er nicht rueckrechenbar
					}
				}
				// ★ BAUPLAN Paragraf 2, zweite Haelfte des Wake-Kippkriteriums (|rho-1| > 0,1). Sie fehlte
				// bisher ersatzlos -- aus der geforderten Konjunktion war ein Einzelkriterium geworden.
				// Hier ist der richtige Ort: der Waechter am Bandrand hat kein rho, die Bilanz schon.
				// ★ NICHT MEHR HART (Korrektur 2026-08-22 nachmittags). Bauplan Paragraf 2 verlangt
				// |rho-1| > 0,1 als halbes Kippkriterium. Dieser Wert ist auf DIESEN Fall nicht
				// geeicht: der KONTROLLLAUF ganz ohne Kopplung meldet 5.499.754 Treffer der
				// Solver-Dichteklemme (RHO_CLAMP_MIN 0,5 / MAX 1,5, defines.hpp:48-50), der
				// Band-Arm 5.793.062 -- plus 5 Prozent. Die Ausschlaege sind Grundzustand, nicht
				// Folge der Aufpraegung. Ein harter Abbruch daran toetet auch den Kontrollarm; er
				// hat b8_bw_n4 gekostet.
				// Aussagekraeftig ist nicht das Extremum ueber ~27.000 Flaechenzellen (ein einziger
				// Ausreisser genuegt), sondern der ANTEIL an der Klemme.
				const double anteil = nfl>0ull ? (double)n_ausser/(double)nfl : 0.0;
				if(t_si>=(double)t_warmup&&anteil>0.02) { if(bilcsv.is_open()) bilcsv.close(); if(swcsv.is_open()) swcsv.close(); print_error("N2F-BAND WAKE-KIPP (Dichte): "+to_string((float)(100.0*anteil),2u)+" % der Fluidzellen auf den Kastenflaechen ("+to_string(n_ausser)+" von "+to_string(nfl)+") stehen an der Solver-Dichteklemme. Ueber 2 Prozent rechnet das Fernfeld dort nichts Physikalisches mehr. Abgebrochen."); }
				else if(t_si>=(double)t_warmup&&(rmx-1.0>0.1||1.0-rmn>0.1)) print_warning("N2F-BAND Dichte im Wake-Kasten: min "+to_string((float)rmn,4u)+" auf Flaeche "+string(fname[rmn_f])+", max "+to_string((float)rmx,4u)+" auf Flaeche "+string(fname[rmx_f])+" -- ausserhalb 1 +- 0,1 (Bauplan Paragraf 2). An der Klemme stehen "+to_string((float)(100.0*anteil),2u)+" %. EINORDNUNG: der Kontrolllauf OHNE Kopplung meldet dieselbe Klemme 5,5 Mio mal -- die Schwelle 0,1 ist auf diesen Fall nicht geeicht, deshalb Warnung statt Abbruch. Verlauf in band_bilanz.csv (n_fluid, n_an_klemme).");
				const double mnet = md[0]+md[1]+md[2]+md[3]+md[4]+md[5];
				// Gitter -> SI: rho_lat*u_lat^2 wird zu rho_si*u_si^2, dazu die Zellflaeche.
				const double A1  = (double)dx_c*(double)dx_c;                                  // Zellflaeche [m2]
				const double sqf = (double)si_rho*((double)si_u/(double)u_lat)*((double)si_u/(double)u_lat)*A1; // [N] je Gittereinheit
				const double fx_imp = fimp*sqf;
				const double fx_dru = fdru*sqf;
				// ★ VORZEICHEN (Pruefagent-B5, 2026-08-22): die x-Impulsbilanz ueber ein Kontrollvolumen
				// lautet  ∮ρu_x(u·n)dA + ∮p·n_x dA = F_x(Koerper -> Fluid) = -D.  Fx_summe_N ist genau
				// diese linke Seite und traegt damit das UMGEKEHRTE Vorzeichen von Fx_fein_N/Fx_grob_N
				// (= +D aus object_force). Beim Lesen also -Fx_summe gegen Fx_fein stellen, nicht
				// Fx_summe. Zwei Terme FEHLEN bewusst und begrenzen die Aussage: der instationaere
				// Anteil d/dt ∫ρu_x dV (im Anfahrtransienten NICHT klein -- erst nach t_warmup lesen)
				// und der Reibanteil an den Kastenflaechen (klein, da alle Flaechen im Fluid liegen).
				bilcsv << t_si << "," << md[0] << "," << md[1] << "," << md[2] << "," << md[3] << "," << md[4] << "," << md[5]
				       << "," << mnet << "," << (mein>0.0? mnet/mein : 0.0)
				       << "," << fx_imp << "," << fx_dru << "," << (fx_imp+fx_dru)
				       << "," << (double)units_fine.si_F(F.x) << "," << (double)units_coarse.si_F(Fc.x)
				       << "," << rmn << "," << rmx << "," << ((fimp_xp+fdru_xp)*sqf) << "," << nverw << "," << nfl << "," << n_ausser << "\n" << std::flush;
			}
			if(n2f_alpha>0.0f) { // ★ P9c WAECHTER an der Sample-Kadenz: schale_extract(mittel=0) auf lbm_c liest das
				// grobe u-FELD der Schalenzellen; verglichen wird gegen das in DIESEM Outer hochgeladene
				// Nahfeld-Blockmittel (fein@(k+1)*dt_c gegen grob@(k+2)*dt_c -- 1 Grobschritt Versatz (4e-5 s), messtechnisch irrelevant, aber so ist es ehrlich). NaN-Eintraege (fluidleere
				// Bloecke, Census beim Listenbau) werden uebersprungen.
				lbm_c.schale_extract_u(n2f_ufar, 0u);
				// ★ GRADIENT: Waechter/Negations-Nachweis/Kipp NUR auf der AEUSSERSTEN Lage (n2f_lage ==
				// N-1; Kontrollarm LAGEN=0: alle Zellen sind Lage 0 = Altverhalten). Grund: die inneren
				// Lagen werden mit a bis alpha*1,0 aktiv ans Nahfeld gezogen -- dort ist ||u_near-u_far||
				// konstruktionsbedingt klein und wuerde den Waechter beschoenigen; die Aussenlage (w = 1/N)
				// ist die freieste und damit die ehrlichste Messstelle der Rueckkopplungsschleife.
				// Diagnose-Ergaenzung: dieselbe Metrik ueber ALLE Lagen (CSV-Spalten n_gueltig_alle,rms_alle_lat).
				double sw_s2=0.0, sw_d2max=0.0, sw_mux=0.0, sw_s2_alle=0.0, sw_s2_wake=0.0, sw_s2_kern=0.0, sw_d2max_kern=0.0; ulong sw_ng=0ull, sw_ng_alle=0ull, sw_ng_wake=0ull, sw_nmux=0ull, sw_ng_kern=0ull;
				for(ulong i=0ull; i<(ulong)n2f_liste_c.size(); i++) {
					const float ax=n2f_unear[3ull*i], ay=n2f_unear[3ull*i+1ull], az=n2f_unear[3ull*i+2ull];
					if(!std::isfinite(ax)||!std::isfinite(ay)||!std::isfinite(az)) continue; // NaN-Marker
					const double du=(double)ax-(double)n2f_ufar[3ull*i], dv=(double)ay-(double)n2f_ufar[3ull*i+1ull], dw=(double)az-(double)n2f_ufar[3ull*i+2ull];
					const double d2=du*du+dv*dv+dw*dw;
					sw_s2_alle+=d2; sw_ng_alle++;
					// ★ FEHLER B4 (Pruefagent 2026-08-22, MITTEL): alle Waechtermetriken liefen NUR
					// auf der Aussenlage. Der Wake-KERN traegt aber Lage 0 und damit das STAERKSTE
					// Gewicht (a = alpha*1,0) im verdaechtigsten Gebiet -- und wurde von keiner
					// einzigen Metrik erfasst. Die WAKE-KIPP-Schwelle wachte ueber die am
					// SCHWAECHSTEN gewichteten Zellen der Wake-Zone. Gemessen an max_bew: 11.664
					// von 177.743 Zellen. Der Kern bekommt jetzt seine eigene Metrik.
					const uchar mk = n2f_marke.empty() ? 0u : n2f_marke[i];
					if((mk&1u)!=0u&&n2f_lage[i]==0u) { sw_s2_kern+=d2; sw_d2max_kern=fmax(sw_d2max_kern,d2); sw_ng_kern++; }
					if(n2f_lage[i]!=n2f_lage_aussen) continue; // Waechter-Metriken: nur Aussenlage
					if((mk&1u)==0u) { sw_s2+=d2; sw_d2max=fmax(sw_d2max,d2); sw_ng++; }  // Kipp-Metrik: NUR Koerperband
					else { sw_s2_wake+=d2; sw_ng_wake++; }                                 // Wake-Zone getrennt gefuehrt
					// ★ NEGATIONS-NACHWEIS: alles AUSSER der Wake-Zone. Erste Fassung schraenkte auf
					// "stromauf der Nase" ein -- das war eine Ueberkorrektur und machte den Test
					// SCHLECHTER: die Aussenlage liegt dort 4 Grobzellen vor dem Fahrzeug, also im
					// STAUPUNKTGEBIET, wo u_x physikalisch abfaellt. Gemessen: 0,477 u_inf ueber
					// 572 Zellen, knapp unter der Schwelle 0,533 -- ein korrekter Lauf brach ab.
					// Das eigentliche Problem war nur der Nachlauf; Flanken und Dach tragen u_x
					// nahe +u_inf und sind die richtige Messmenge.
					if((mk&1u)==0u) { sw_mux += (double)n2f_ufar[3ull*i]; sw_nmux++; }
				}
				const double sw_rms = sw_ng? sqrt(sw_s2/(double)sw_ng) : 0.0;
				const double sw_rms_alle = sw_ng_alle? sqrt(sw_s2_alle/(double)sw_ng_alle) : 0.0;
				const double sw_rms_wake = sw_ng_wake? sqrt(sw_s2_wake/(double)sw_ng_wake) : 0.0;
				const double sw_rms_kern = sw_ng_kern? sqrt(sw_s2_kern/(double)sw_ng_kern) : 0.0; // B4: Wake-Kern, Lage 0, volles Gewicht
				swcsv << t_si << "," << sw_ng << "," << sw_rms << "," << sqrt(sw_d2max) << "," << sw_rms/(double)u_lat << "," << sw_ng_alle << "," << sw_rms_alle << "," << sw_ng_wake << "," << sw_rms_wake << "," << sw_ng_kern << "," << sw_rms_kern << "," << sqrt(sw_d2max_kern) << "\n" << std::flush;
				// ★ EIGENE Schwelle fuer die Wake-Zone. Dort ist die Nah-Fern-Differenz physikalisch
				// gross (gemessen 4 mm: Diff-RMS bis 8,56 m/s = 0,29 u_inf im Totwasser) -- die
				// 0,5-u_inf-Kippschwelle des Koerperbands waere dort blind, eine strengere ein
				// Fehlalarm. Verdachtsschwelle: mehr als die Anstroemung selbst kann kein Nachlauf.
				if(sw_ng_kern>0ull&&t_si>=(double)t_warmup&&sw_rms_kern>1.5*(double)u_lat) { if(bilcsv.is_open()) bilcsv.close(); if(swcsv.is_open()) swcsv.close(); print_error("N2F-BAND WAKE-KERN-KIPP: RMS ||u_near-u_far|| im Wake-KERN (Lage 0, volles Gewicht) = "+to_string((float)(sw_rms_kern/(double)u_lat),3u)+" u_inf > 1,5 ueber "+to_string(sw_ng_kern)+" Zellen. Dort wird mit a = alpha aufgepraegt; eine Abweichung dieser Groesse heisst, die Aufpraegung traegt nicht. Abgebrochen."); }
				if(sw_ng_wake>0ull&&t_si>=(double)t_warmup&&sw_rms_wake>1.0*(double)u_lat) { if(bilcsv.is_open()) bilcsv.close(); if(swcsv.is_open()) swcsv.close(); print_error("N2F-BAND WAKE-KIPP: RMS ||u_near-u_far|| in der Wake-Zone = "+to_string((float)(sw_rms_wake/(double)u_lat),3u)+" u_inf > 1,0 -- das Fernfeld weicht dort um mehr als die Anstroemung ab, das kann kein Nachlauf mehr sein. Abgebrochen."); }
				if(!n2f_neg_geprueft) { // ★ u-NEGATIONS-NACHWEIS (XL-B8, im Blend TRAGEND): funktionaler Beweis am
					// laufenden Binary. Der Blend liest post-stream u EXAKT NEGIERT und muss es zurueckdrehen;
					// waere das Vorzeichen falsch, stuende u_neu = (1-a)*(-u)+a*u_near, und der FIXPUNKT
					// dieser Iteration liegt bei u* = w/(2-w)*u_near (w = wirksames a der Zelle) -- das
					// Schalen-u_x des u-FELDS (das die Blend-Wirkung des Vorschritts traegt) fiele dorthin.
					// Gemessen wird auf der AUSSENLAGE, dort ist w_aussen = alpha/N (Gradient) bzw. alpha
					// (Kontrollarm LAGEN=0). Richtig behandelt bleibt es nahe +u_inf (frueher Lauf,
					// Stroemung noch kaum entwickelt). Zusaetzlich Testzelle 0 als Zahlenpaar.
					n2f_neg_geprueft = true;
					// ★ 2026-08-22: der Divisor MUSS die Zahl der Zellen sein, ueber die sw_mux
					// tatsaechlich summiert wurde -- seit der Nachweis auf "stromauf der Nase"
					// eingeschraenkt ist, ist das sw_nmux und nicht mehr sw_ng. Mit dem alten
					// Divisor meldete der Nachweis 0,0266 u_inf statt ~1 und brach einen
					// korrekten Lauf ab (gemessen, erster Lauf nach der Einschraenkung).
					const double m_ux = sw_nmux? sw_mux/(double)sw_nmux : 0.0;
					if(sw_nmux==0ull) print_error("u-Negations-Nachweis: keine einzige Aussenlagen-Zelle AUSSERHALB der Wake-Zone -- der Nachweis haette keine Datengrundlage. Bandbreite oder Nahfeld-Box pruefen. (Der Text nannte hier bis 2026-08-22 'stromauf der Fahrzeugnase'; diese Einschraenkung war eine Ueberkorrektur und wurde verworfen -- sie legte den Bezug in die Stagnation und brach einen richtigen Lauf ab.)");
					const double w_aussen = (n2f_band>0u||n2f_volumen>0u) ? (double)n2f_alpha*(double)n2f_w_band : ((n2f_lagen>0u) ? (double)n2f_alpha/(double)n2f_lagen : (double)n2f_alpha); // wirksames a der Aussenstufe (VOLUMEN: groesstes Gewicht im Aussenband; Schale: XPLUS_SKAL drueckt x+ ggf. darunter -- Schwelle bleibt konservativ)
					const double falschziel = w_aussen/(2.0-w_aussen); // Falschziel-FIXPUNKT (Plan-Vorgabe; ersetzt die alte Ein-Schritt-Schaetzung 2a-1)
					const double schwelle = fmax(0.5, 0.5*(1.0+falschziel)); // Mittelpunkt Falschziel<->Soll(+1); verallgemeinert Pruefagent M1 (alt: fmax(0.5, alpha) = Mittelpunkt von 2a-1 und 1)
					print_info("N2F-Schale u-NEGATIONS-NACHWEIS (1. Waechter-Sample, Aussenlage "+to_string(n2f_lage_aussen)+", w_aussen = "+to_string((float)w_aussen,3u)+", ueber "+to_string(sw_nmux)+" Zellen ausserhalb der Wake-Zone): mittleres Schalen-u_x im Fernfeld-u-Feld = "+to_string((float)(m_ux/(double)u_lat),4u)+" u_inf (Soll nahe +1; falsches Vorzeichen truege es Richtung Fixpunkt w/(2-w) = "+to_string((float)falschziel,3u)+" u_inf; Schwelle "+to_string((float)schwelle,3u)+"); Testzelle 0: u_far = ("+to_string(n2f_ufar[0],6u)+","+to_string(n2f_ufar[1],6u)+","+to_string(n2f_ufar[2],6u)+") vs u_near = ("+to_string(n2f_unear[0],6u)+","+to_string(n2f_unear[1],6u)+","+to_string(n2f_unear[2],6u)+") lat.");
					if(m_ux<schwelle*(double)u_lat) {
						swcsv.close(); if(bilcsv.is_open()) bilcsv.close(); /* B10: bilcsv fehlte in den symmetrischen Abbruchpfaden */ fcsv.close(); ipcsv.close(); if(zb>0u) zcsv.close(); if(sonde_csv.is_open()) sonde_csv.close();
						print_error("u-Negations-Nachweis FEHLGESCHLAGEN: Schalen-u_x = "+to_string((float)(m_ux/(double)u_lat),4u)+" u_inf < Schwelle "+to_string((float)schwelle,3u)+" -- der Blend arbeitet mit falschem Vorzeichen (XL-B8) oder die Schale ist vergiftet. Abgebrochen.");
					}
				}
				string kipp = "";
				if(t_si>=(double)t_warmup&&sw_rms>0.5*(double)u_lat) kipp = "RMS (Aussenlage) > 0,5*u_inf nach Warmup";
				// ★ Monotonie-Klausel GEEICHT am ersten Abnahmelauf (2026-08-19): ungefloort kippte sie
				// bei RMS = 0,063 u_inf (t = 0,00496 s) -- das war der normale ANFAHRTRANSIENT (die
				// Near-Far-Diskrepanz waechst, solange sich die Umstroemung des Treppenkoerpers aufbaut;
				// Zuwachs 1-3 % je Sample, kein exponentielles Wachstum), keine Divergenz. Die Klausel
				// ist der FRUEHWARN-Detektor der Rueckkopplungsschleife und zaehlt deshalb erst im
				// Eskalationsband RMS >= 0,2*u_inf (unterhalb: Streak-Reset); die harte 0,5-Schwelle
				// oben bleibt unveraendert.
				sw_steigend = (sw_rms_prev>=0.0&&sw_rms>sw_rms_prev&&sw_rms>=0.2*(double)u_lat) ? sw_steigend+1u : 0u; sw_rms_prev = sw_rms;
				if(kipp==""&&sw_steigend>=10u) kipp = "RMS ueber 10 Samples monoton steigend (im Band >= 0,2*u_inf)";
				if(kipp!="") {
					swcsv.close(); if(bilcsv.is_open()) bilcsv.close(); /* B10: bilcsv fehlte in den symmetrischen Abbruchpfaden */ fcsv.close(); ipcsv.close(); if(zb>0u) zcsv.close(); if(sonde_csv.is_open()) sonde_csv.close(); // Abbruchpfad symmetrisch (R-N2-Muster)
					print_error("N2F-Schale GEKIPPT bei t = "+to_string((float)t_si,5u)+" s: "+kipp+" (RMS = "+to_string((float)sw_rms,6u)+" lat = "+to_string((float)(sw_rms/(double)u_lat),3u)+" u_inf). Die CSV bis hierher steht in "+out_dir+"schale_waechter.csv.");
				}
			}
			const double Fx_si = (double)units_fine.si_F(F.x), Fz_si = (double)units_fine.si_F(F.z);
			// ★ NaN-WAECHTER (Pruefer-Befund 2026-08-08). Ohne ihn kostet eine Divergenz den ganzen Lauf:
			// die CSV liefe 2,5 Stunden mit nan voll, Mittelwert und Block-SEM lieferten nan, und gewarnt
			// haette nichts. Genau das ist am 2026-08-08 passiert -- bei 3,8 ms stand nan in der Reihe,
			// danach fror das Feld ein. In LBM breitet sich ein nan aus und verschwindet nie wieder;
			// Weiterrechnen waere nicht Geduld, sondern Verschwendung. Besonders wichtig, solange
			// RHO_CLAMP fehlt und tau bei 0,50003 steht.
			// ★★ NACHGESCHAERFT 2026-08-08, am eigenen Lauf gelernt: der NaN-Test allein REICHT NICHT.
			// dd_lauf01 kippte bei 0,15 s NICHT in nan, sondern in die FP16C-Saettigung. Danach stand Fz bei
			// exakt 343 063 N und Fx bei 172,478 N -- Abtastung fuer Abtastung BITGLEICH, endliche Zahlen,
			// keine Warnung. Der Lauf lief zwei Stunden lang tot weiter. Das ist keine Kraft, sondern
			// die auf die Zahlenformatgrenze gelaufene DDF-Ablage.
			// ★ KORRIGIERT 2026-08-15: hier stand "V1 fror am selben Punkt bei 343 640 N ein --
			// derselbe Mechanismus". DIE ZAHL WAR FREI ERFUNDEN. Ein Pruefer hat 639 CSVs, alle Logs
			// und 997 Commit-Botschaften durchsucht: sie existiert in V1 nirgends, und kein
			// gekoppelter V1-Lauf fror bei 0,15 s ein (16 liefen darueber hinaus, drei bis 0,5 s).
			// Die Behauptung diente als Scheinbeleg fuer einen gemeinsamen Mechanismus.
			// Drei Tests statt einem. Zwei bitgleiche Kraftwerte hintereinander gibt es in einer
			// abgeloesten Stroemung nicht; drei sind ein Beweis.
			{
				const double Fxf = (double)units_coarse.si_F(Fc.x);
				const double q_A = (double)q_inf*A_ref;
				string grund = "";
				if(!std::isfinite(Fx_si) || !std::isfinite(Fz_si) || !std::isfinite(Fxf)) grund = "die Kraft ist keine Zahl mehr";
				else if(n_frozen>=2u) grund = "die Kraft steht seit drei Abtastungen BITGLEICH -- das Feld ist eingefroren (Zahlenformat gesaettigt)";
				else if(t_si>(double)kipp_ab && (fabs(Fx_si)>20.0*q_A || fabs(Fz_si)>20.0*q_A)) grund = "die Kraft ist unphysikalisch gross (|Cd| oder |Cz| ueber 20, gemessen ab CFD_KIPP_AB = "+to_string(kipp_ab,3u)+" s)";
				if(grund!="") {
					fcsv << std::flush; fcsv.close(); ipcsv.close(); if(swcsv.is_open()) swcsv.close(); if(bilcsv.is_open()) bilcsv.close(); /* B10: bilcsv fehlte in den symmetrischen Abbruchpfaden */ if(sonde_csv.is_open()) sonde_csv.close(); // R-N2: Abbruchpfad symmetrisch (Zeilen sind ohnehin geflusht); P9c: Waechter-CSV mit schliessen
					print_error("Lauf gekippt bei t = "+to_string((float)t_si,5u)+" s (grober Schritt "+to_string((ulong)(outer+1ull))
						+"): "+grund+". Fx = "+to_string((float)Fx_si,3u)+" N, Fz = "+to_string((float)Fz_si,3u)
						+" N. Abgebrochen. Die CSV bis hierher steht in "+out_dir+"forces.csv -- dort ist zu sehen, wann es kippt.");
				}
				n_frozen = (Fz_si==Fz_prev && Fx_si==Fx_prev) ? n_frozen+1u : 0u;
				Fz_prev = Fz_si; Fx_prev = Fx_si;
			}
			const double Fx_far = (double)units_coarse.si_F(Fc.x);
			ts.push_back(t_si); fx.push_back(Fx_si); fz.push_back(Fz_si); fx_c.push_back(Fx_far);
			fcsv << t_si << "," << Fx_si << "," << Fz_si << "," << Fx_si/((double)q_inf*A_ref) << ","
			     << Fz_si/((double)q_inf*A_ref) << "," << Fx_far << "\n" << std::flush; // sofort auf Platte, siehe oben
			if(fac_an_zs && t_si>=(double)t_warmup && (++fac_smp)%fac_cd_every==0ull) { // ★ Stufe 5 (PCIe ~2,5 GB je Aufruf -- Kadenz!)
				LBM_Domain* df = lbm_f.lbm_domain[0];
				if(fac_snap.empty()) { // Reibungs-Snapshot am Fensteranfang (erst ab Warmup)
					df->fac_tau.read_from_device();
					lbm_f.flags.read_from_device(); // einmalig fuer alle folgenden kraft_facetten-Aufrufe (flags statisch)
					for(ulong i0=0ull;i0<df->fac_N;i0++){ fac_dm0+=(double)df->fac_tau[6ull*i0+4ull]; fac_rest0+=(double)df->fac_tau[6ull*i0+5ull]; } // Audit S5: [4]/[5]-Snapshot -- Fenster-Delta statt Warmup-Historie
					fac_snap.resize(3ull*df->fac_N);
					for(ulong i=0ull;i<df->fac_N;i++){ fac_snap[3ull*i]=(double)df->fac_tau[6ull*i+1ull]; fac_snap[3ull*i+1ull]=(double)df->fac_tau[6ull*i+2ull]; fac_snap[3ull*i+2ull]=(double)df->fac_tau[6ull*i+3ull]; }
					fac_snap_outer = outer+1ull;
				} else {
					lbm_f.lbm_domain[0]->sgs_gdiag_gpu(); // ★ g-Diagnose an der Facetten-Sample-Kadenz (no-op ohne CFD_SGS_GDIAG)
					const FacKraft FK = kraft_facetten(lbm_f, fNx, fNy, fNz, (uchar)(TYPE_S|TYPE_X), (outer+1ull-fac_snap_outer)*(ulong)ratio, fac_snap, false, true, zb);
					fac_px += FK.px; fac_pz += FK.pz; fac_pn++;
					zen_voll=FK.n_voll; zen_proj=FK.n_proj; zen_unklar=FK.n_unklar; zen_da=true; // ★ ZENSUS: Zellklassen (geometrisch konstant)
					zen_px=FK.px; zen_pz=FK.pz; zen_ukraft=FK.ukraft_ok; if(FK.ukraft_ok) { zen_ux=FK.ux; zen_uz=FK.uz; }
					if(zb>0u) { // ★ KRAFT-ZBAND: NUR der Druckanteil ist zerlegt -- fac_tau (Reibung) traegt keine z-Position (Folgearbeit fac_geo[6]); Rest = Gesamt - Band (double)
						const double qA_zb=(double)q_inf*A_ref;
						zb_czdb = (double)units_fine.si_F((float)FK.pbz)/qA_zb;
						zb_czdr = (double)units_fine.si_F((float)(FK.pz-FK.pbz))/qA_zb;
						zb_fac_now = true;
					}
					double dm_b=0.0, rest_b=0.0; // fac_tau frisch durch kraft_facetten (kein Extra-Transfer)
					for(ulong i=0ull;i<df->fac_N;i++){ dm_b+=(double)df->fac_tau[6ull*i+4ull]; rest_b+=(double)df->fac_tau[6ull*i+5ull]; }
					fac_dm=dm_b-fac_dm0; fac_rest=rest_b-fac_rest0; // FENSTER-Delta (Audit S5): Warmup-Historie abgezogen
					if(!fac_csv.is_open()) { fac_csv.open(out_dir+"cd_facetten.csv"); fac_csv.precision(8); fac_csv << "# DREI ZEITBASEN (Instrumenten-Audit 2026-08-22): cd/cz_druck = MOMENTAN am Sample; cd/cz_reib = FENSTER-MITTEL seit Warmup; dm/rest = KUMULATIV seit Warmup (wachsend). Kadenz = Sample x CFD_FAC_CD_EVERY.\n";
		fac_csv << "time_s,cd_druck,cz_druck,cd_reib,cz_reib,dm,rest,cd_druck_band,cd_druck_rest,cz_druck_band,cz_druck_rest\n"; } // ★ 2026-08-25: Band/Rest auch fuer x -- der ehrliche Karosserie-Cd (Radkontakt-Band ist kein Zielkanal)
					const double qA=(double)q_inf*A_ref;
					const double cdb=(double)units_fine.si_F((float)FK.pbx)/qA, cdg=(double)units_fine.si_F((float)FK.px)/qA;
					const double czb=(double)units_fine.si_F((float)FK.pbz)/qA, czg=(double)units_fine.si_F((float)FK.pz)/qA;
					kad_cd_rest=cdg-cdb; kad_cz_rest=czg-czb; kad_cdcz_da=true; // Puffer fuer die Kadenz-Einblendung
					ber_t.push_back(t_si); ber_cd.push_back(cdg-cdb); ber_cz.push_back(czg-czb); // Laufbericht: gleitendes Fenster
					fac_csv << t_si << "," << cdg << "," << czg << ","
					        << (double)units_fine.si_F((float)FK.rx)/qA << "," << (double)units_fine.si_F((float)FK.rz)/qA << "," << fac_dm << "," << fac_rest
					        << "," << cdb << "," << (cdg-cdb) << "," << czb << "," << (czg-czb) << "\n" << std::flush;
					if(LBM_Domain::s_fac_elibb&&fabs(fac_dm)>1e-4*(double)df->fac_N) { static bool dm_einmal=false; if(!dm_einmal) { dm_einmal=true; print_info("Delta-m traegt unter ELIBB den REALEN Blenden-Massenfluss (B3) -- Gelb-Band-Schwelle gilt dort nicht; Fenster-Delta = "+to_string((float)fac_dm,6u)+" (einmalige Ansage, weiter in der CSV)."); } }
					else if(fabs(fac_dm)>1e-4*(double)df->fac_N&&!LBM_Domain::s_fac_elibb) print_warning("Delta-m Gelb-Band gerissen: "+to_string((float)fac_dm,6u)+" bei fac_N = "+to_string(df->fac_N)+" (provisorische Schwelle 1e-4*fac_N auf das FENSTER-Delta -- Arm-4-Eichung: Rauschbett ~0,12 kumulativ, Schwelle ~1 vormerken)."); // Torus lief mit -14,9 UNBEWACHT -- nie wieder
				}
			}
			if(zb>0u) { // ★ KRAFT-ZBAND: CSV-Zeile sofort auf Platte (Muster forces.csv) + Mittel-Akkumulatoren ab Warmlauf
				const double Fxb=(double)units_fine.si_F(Fb.x), Fzb=(double)units_fine.si_F(Fb.z);
				const double Fxr=(double)units_fine.si_F(Fr.x), Fzr=(double)units_fine.si_F(Fr.z);
				const double qA_zb=(double)q_inf*A_ref;
				zcsv << t_si << "," << Fxb << "," << Fzb << "," << Fxr << "," << Fzr << ","
				     << Fzb/qA_zb << "," << Fzr/qA_zb << "," << zb_rel;
				if(zb_fac_now) zcsv << "," << zb_czdb << "," << zb_czdr << "\n" << std::flush;
				else zcsv << ",,\n" << std::flush; // Druckspalten nur an der Facetten-Kadenz gefuellt
				if(t_si>=(double)t_warmup) {
					zb_cd_band+=Fxb/qA_zb; zb_cz_band+=Fzb/qA_zb; zb_cd_rest+=Fxr/qA_zb; zb_cz_rest+=Fzr/qA_zb; zb_nn++;
					zb_cz_rest_reihe.push_back(Fzr/qA_zb);
					zb_selftest_max = fmax(zb_selftest_max, zb_rel);
				}
			}
			const auto _t5 = t_now();
			// ★ LAUFBERICHT alle ber_dt ueber das letzte ber_fen-Fenster (Heiko 29.08.).
		if(ber_dt>0.0&&!ber_t.empty()) {
			// ★ Rauchtest 29.08.: stand auf "erstes Sample + ber_dt" und rastete erst DANACH aufs
			// Raster -- bei kurzen Laeufen fiel dadurch ein Bericht ganz aus. Jetzt sofort aufs
			// Raster, sobald das Mittelungsfenster ueberhaupt gefuellt sein kann.
			if(ber_next<0.0) ber_next = (floor((ber_t.front()+ber_fen)/ber_dt)+1.0)*ber_dt;
			while((size_t)ber_n_aus<ber_t.size()&&ber_t[ber_n_aus]<ber_t.back()-ber_fen) ber_n_aus++; // Fensteranfang nachziehen
			if(t_si>=ber_next) {
				double mcd=0.0, mcz=0.0; ulong nb=0ull;
				for(size_t i=ber_n_aus;i<ber_t.size();i++) { mcd+=ber_cd[i]; mcz+=ber_cz[i]; nb++; }
				if(nb>0ull) {
					mcd/=(double)nb; mcz/=(double)nb;
					double vcd=0.0, vcz=0.0;
					for(size_t i=ber_n_aus;i<ber_t.size();i++) { vcd+=(ber_cd[i]-mcd)*(ber_cd[i]-mcd); vcz+=(ber_cz[i]-mcz)*(ber_cz[i]-mcz); }
					const double scd = nb>1ull ? sqrt(vcd/(double)(nb-1ull)) : 0.0;
					const double scz = nb>1ull ? sqrt(vcz/(double)(nb-1ull)) : 0.0;
					print_info("[BERICHT] t = "+to_string((float)t_si,3u)+" s | Fenster "+to_string((float)(ber_fen*1000.0),0u)
						+" ms, n = "+to_string(nb)+" | cd_rest = "+to_string((float)mcd,4u)+" +- "+to_string((float)scd,4u)
						+" | cz_rest = "+to_string((float)mcz,4u)+" +- "+to_string((float)scz,4u)
						+(t_si<(double)t_warmup?"  (noch im Warmlauf)":""));
					if(!ber_csv.is_open()) { ber_csv.open(out_dir+"cd_bericht.csv"); ber_csv.precision(8);
						ber_csv << "# Laufbericht (Heiko 29.08.): alle " << ber_dt << " s Physik das Mittel ueber die letzten "
						        << ber_fen << " s. rest = Gesamt minus Radkontakt-Band. sd = Streuung IM Fenster, kein SEM der Messreihe.\n";
						ber_csv << "time_s,n,cd_rest_mittel,cd_rest_sd,cz_rest_mittel,cz_rest_sd,warmup\n"; }
					ber_csv << t_si << "," << nb << "," << mcd << "," << scd << "," << mcz << "," << scz
					        << "," << (t_si<(double)t_warmup?1:0) << "\n" << std::flush;
					// ---- Slot-Zaehler JETZT lesen, nicht erst im Abschlussbericht.
					{	LBM_Domain* d0 = lbm_f.lbm_domain[0];
						d0->rho_clamp_hits.read_from_device();
						const uint* H = d0->rho_clamp_hits.data();
						const ulong wp=(ulong)H[7], r69=(ulong)H[69], s10=(ulong)H[10], s13=(ulong)H[13], s15=(ulong)H[15], s16=(ulong)H[16], s64=(ulong)H[64];
						auto pz=[](ulong a, ulong b){ return b>0ull ? 100.0*(double)a/(double)b : 0.0; };
						// Fensterdelta seit dem letzten Bericht -- das ist die AKTUELLE Quote, die kumulierte
						// traegt den Warmlauf und die Anlaufphase mit.
						const ulong dwp = slot_alt_da ? wp-slot_alt[7] : wp, d69 = slot_alt_da ? r69-slot_alt[69] : r69;
						const bool satt = (wp>=0xF0000000ull)||(r69>=0xF0000000ull);
						print_info("[SLOTS] t = "+to_string((float)t_si,3u)+" s | Wirkpfad "+to_string(wp)+" | Rueckfall[69] "+to_string(r69)
							+" = "+to_string((float)pz(r69,wp),1u)+" % kumuliert, "+to_string((float)pz(d69,dwp),1u)+" % im Fenster"
							+" | u_s-Gate "+to_string((float)pz(s10,wp),1u)+" % | ohneTang "+to_string((float)pz(s13,wp),1u)
							+" % | Rang0 "+to_string((float)pz(s15,wp),1u)+" % | sn-Gate "+to_string((float)pz(s16,wp),1u)+" % | Quergate "+to_string((float)pz(s64,wp),1u)+" % | Kraft[70] "+to_string((float)pz((ulong)H[70],wp),1u)+" %"
							+(satt?"  ★ GESAETTIGT -- Prozent nicht mehr belastbar":""));
						if(!slot_csv.is_open()) { slot_csv.open(out_dir+"slots_verlauf.csv"); slot_csv.precision(8);
							slot_csv << "# Slot-Zaehler je [BERICHT] (30.08.2026). Kumuliert seit Laufstart, t%100-gesampelt; *_fenster = Delta seit dem vorigen Bericht. Quote = Slot/Wirkpfad[7].\n";
							slot_csv << "time_s,wirkpfad,rueckfall69,quote_kum_pct,quote_fenster_pct,us_gate10,ohnetang13,rang0_15,sn_gate16,quergate64,kraft70,gesaettigt\n"; }
						slot_csv << t_si << "," << wp << "," << r69 << "," << pz(r69,wp) << "," << pz(d69,dwp) << "," << s10 << "," << s13 << "," << s15 << "," << s16 << "," << s64 << "," << (ulong)H[70] << "," << (satt?1:0) << "\n" << std::flush;
						for(uint i=0u;i<96u;i++) slot_alt[i]=(ulong)H[i]; slot_alt_da=true;
					}
				}
				ber_next = (floor(t_si/ber_dt)+1.0)*ber_dt;  // an das Raster binden, nicht aufaddieren (kein Drift)
			}
			if(ber_n_aus>4096ull) { // Puffer gelegentlich verdichten, damit er nicht ueber den Lauf waechst
				ber_t.erase(ber_t.begin(), ber_t.begin()+(long)ber_n_aus);
				ber_cd.erase(ber_cd.begin(), ber_cd.begin()+(long)ber_n_aus);
				ber_cz.erase(ber_cz.begin(), ber_cz.begin()+(long)ber_n_aus); ber_n_aus=0ull; }
		}
		const ulong ns_ist = (outer+1ull)*(ulong)ratio; // exakte Near-Steps: BEIDE Pfade laufen ratio feine Schritte je Outer (Plan 27.08., setup.cpp:4699/4717)
			bool slice_jetzt = false;
			if(slice_ns>0ull) { if(ns_ist>=slice_ns_next) { slice_jetzt = true; slice_ns_next = (ns_ist/slice_ns+1ull)*slice_ns; kad_punkt++; } }
			else if(slice_dt>0.0f && (float)t_si>=slice_next) { slice_jetzt = true; slice_next = (float)t_si + slice_dt; }
			if(slice_jetzt) {
				const int t_ms = (int)((float)t_si*1000.0f+0.5f);
				const string kad_info = "t="+to_string(t_ms)+"ms  cd="+(kad_cdcz_da?to_string((float)kad_cd_rest,2u):string("--"))+"  cz="+(kad_cdcz_da?to_string((float)kad_cz_rest,2u):string("--"));
				// ★ Slice-Ebenen-Read (Perf-Hebel 2026-08-26, Plan Variante b): nur die konsumierten
				// y-Ebenen holen statt der vollen Felder (4 mm nah: ~14 MB statt ~8,65 GB je Ereignis).
				// Altpfad wortgleich unter CFD_SLICE_GPU=0; CFD_SLICE_PRUEF rechnet beide Wege.
				if(slice_pruef) {
					lbm_f.u.read_from_device(); lbm_f.flags.read_from_device(); lbm_f.rho.read_from_device();
					pruefe_slice_ebene(lbm_f, fNx, fNy, fNz, fNy/2u, "nah");
				} else if(slice_gpu) {
					lbm_f.lese_yslice_in_host(fNy/2u); // deckt Slice, Sonde UND Diff-Nahseite (alle auf y = fNy/2)
				} else {
					lbm_f.u.read_from_device(); lbm_f.flags.read_from_device();
				}
				render_yslice(lbm_f, fNx, fNy, fNz, fNy/2u, si_u/u_lat, si_u, t_ms, out_dir, "nah", kad_info);
				// ★ SONDE einlass_saeule_nah (Iron Rule 5, Ansage beim CSV-Anlegen oben): die Sonde liegt
				// auf der Ebene y = fNy/2 -- im Gather- und im Pruef-Pfad ist rho dort schon frisch,
				// nur der Altpfad braucht den vollen rho-Read. Je Sample geflusht (Abbruch-fest).
				if(!slice_gpu&&!slice_pruef) lbm_f.rho.read_from_device();
				for(uint ci=0u; ci<2u; ci++) {
					const uint sx = sonde_xf[ci], sy = fNy/2u;
					for(uint sz2=0u; sz2<fNz; sz2++) {
						const ulong n = (ulong)sx+((ulong)sy+(ulong)sz2*(ulong)fNy)*(ulong)fNx;
						sonde_csv << t_si << "," << sx << "," << sz2 << "," << lbm_f.u.x[n]/u_lat << ","
						          << lbm_f.u.y[n]/u_lat << "," << lbm_f.u.z[n]/u_lat << "," << lbm_f.rho[n] << ","
						          << (((lbm_f.flags[n]&TYPE_S)!=0u)?1u:0u) << "\n";
					}
				}
				sonde_csv << std::flush;
				if(slice_pruef) {
					lbm_c.u.read_from_device(); lbm_c.flags.read_from_device(); lbm_c.rho.read_from_device();
					pruefe_slice_ebene(lbm_c, cNx, cNy, cNz, cNy/2u, "fern");
					if(diff_an) { // ★ MITTEL-1 (Pruefagent): auch die Diff-Ebenen gegen den Vollpfad pruefen
						const uint j0 = diff_j0(NF_OY, fNy/2u, ratio, cNy);
						if(j0!=cNy/2u) pruefe_slice_ebene(lbm_c, cNx, cNy, cNz, j0, "fern-diff");
						if(j0+1u!=cNy/2u) pruefe_slice_ebene(lbm_c, cNx, cNy, cNz, j0+1u, "fern-diff");
					}
				} else if(slice_gpu) {
					lbm_c.lese_yslice_in_host(cNy/2u);
					if(diff_an) { // die zwei Trilinear-Ebenen des Diff-Schnitts (gemeinsamer j0-Helfer)
						const uint j0 = diff_j0(NF_OY, fNy/2u, ratio, cNy);
						if(j0!=cNy/2u) lbm_c.lese_yslice_in_host(j0);
						if(j0+1u!=cNy/2u) lbm_c.lese_yslice_in_host(j0+1u);
					}
				} else {
					lbm_c.u.read_from_device(); lbm_c.flags.read_from_device();
				}
				render_yslice(lbm_c, cNx, cNy, cNz, cNy/2u, si_u/u_lat, si_u, t_ms, out_dir, "fern", kad_info);
				// ★ DIFF-SCHNITT nah-fern (Heiko 2026-08-21): KEIN zusaetzlicher Device-Read -- lbm_f.u/flags
				// und lbm_c.u/flags liegen fuer die beiden Schnitte oben ohnehin schon auf dem Host. Skala
				// ueber CFD_DIFF_SPAN (Default 15 m/s = Heiko-Vorgabe blau -15 / weiss 0 / rot +15).
				if(diff_an)
					render_yslice_diff(lbm_f, lbm_c, fNx, fNy, fNz, cNx, cNy, cNz, NF_OX, NF_OY, NF_OZ, ratio,
					                   si_u/u_lat, diff_span, near_x0, near_z0, dx_f, t_ms, out_dir);
				print_info("[KADENZ] t = "+to_string((float)t_si,3u)+" s | Near-Step "+to_string(ns_ist)+" | cd_rest "+(kad_cdcz_da?to_string((float)kad_cd_rest,4u):string("--"))+" | cz_rest "+(kad_cdcz_da?to_string((float)kad_cz_rest,4u):string("--")));
				fcsv.flush(); if(fac_csv.is_open()) fac_csv.flush(); if(zb>0u) zcsv.flush(); sonde_csv.flush(); // Guertel-und-Hosentraeger: Zeilen flushen bereits einzeln (Plan-Befund B2), hier nur die Kadenz-Garantie
			}
			if(slice_ns>0ull&&vtk_jede>0u&&slice_jetzt&&kad_punkt%(ulong)vtk_jede==0ull) { // ★ VTK an der Near-Step-Kadenz (27.08.)
				const int t_ms = (int)((float)t_si*1000.0f+0.5f);
				string ms = to_string(t_ms); while(ms.length()<6u) ms = "0"+ms;
				lbm_f.u.read_from_device(); lbm_f.rho.read_from_device(); lbm_f.flags.read_from_device();
				schreibe_vtk_feld(lbm_f, fNx, fNy, fNz, near_x0, near_y0, near_z0, dx_f, si_u/u_lat, vtk_stride, out_dir+"feld_nah_"+ms+"ms.vtk");
				lbm_c.u.read_from_device(); lbm_c.rho.read_from_device(); lbm_c.flags.read_from_device();
				schreibe_vtk_feld(lbm_c, cNx, cNy, cNz, far_x0, far_y0, 0.0f, dx_c, si_u/u_lat, vtk_stride, out_dir+"feld_fern_"+ms+"ms.vtk");
				vtk_ms_letzt = t_ms; vtk_rotiere(t_ms);
			}
			// ★ VTK-Kadenz: EIGENE Uhr, unabhaengig von CFD_SLICE_DT. Die Lesevorgaenge stehen hier
			// bewusst noch einmal -- der Slice-Block laeuft an einer anderen Kadenz und kann in diesem
			// Fenster ausgefallen sein; ein Dump aus halb altem Hostspeicher waere ein stiller Fehler.
			if(vtk_dt>0.0f && (float)t_si>=vtk_next) {
				vtk_next = (float)t_si + vtk_dt;
				const int t_ms = (int)((float)t_si*1000.0f+0.5f);
				string ms = to_string(t_ms); while(ms.length()<6u) ms = "0"+ms;
				lbm_f.u.read_from_device(); lbm_f.rho.read_from_device(); lbm_f.flags.read_from_device();
				schreibe_vtk_feld(lbm_f, fNx, fNy, fNz, near_x0, near_y0, near_z0, dx_f, si_u/u_lat, vtk_stride, out_dir+"feld_nah_"+ms+"ms.vtk");
				lbm_c.u.read_from_device(); lbm_c.rho.read_from_device(); lbm_c.flags.read_from_device();
				schreibe_vtk_feld(lbm_c, cNx, cNy, cNz, far_x0, far_y0, 0.0f, dx_c, si_u/u_lat, vtk_stride, out_dir+"feld_fern_"+ms+"ms.vtk");
				vtk_ms_letzt = t_ms; vtk_rotiere(t_ms);
			}
			ph_kraft += std::chrono::duration<double>(_t5-_t4).count();
			ph_schnitt += std::chrono::duration<double>(t_now()-_t5).count();

			// ★ SAUBERER STOPP: erst HIER, nach Kraeften, Sonden, Schnitt und VTK-Kadenz -- der
			// angebrochene Aussenschritt ist damit vollstaendig ausgewertet, bevor die Schleife
			// endet. Die Datei wird beim Erkennen entfernt (der Abschlusspfad laeuft trotzdem).
			if(access(stop_datei.c_str(), F_OK)==0) {
				std::remove(stop_datei.c_str());
				stop_angefordert = true;
				print_info("[STOPP] "+stop_datei+" erkannt bei t = "+to_string((float)t_si,4u)+" s (grober Schritt "
					+to_string((ulong)(outer+1ull))+" von "+to_string(n_outer)+"). Zeitschleife wird regulaer verlassen, Abschlusspfad laeuft.");
			}

			// ★ 04.09.2026 LAUFVERLAENGERUNG. Der Vorgaengerfork hatte das (/tmp/cfd_target_outer); beim
			// V2-Neuaufbau ist es nicht mitgekommen -- genau wie die Stoppdatei, die auch erst nachtraeglich
			// kam. Ohne sie muss die Lauflaenge VOR dem Start erraten werden, und ein zu kurz geratener Lauf
			// kostet die volle Rechenzeit ein zweites Mal. Am 04.09. selbst passiert: 301 statt der ueblichen
			// 501 ms, weil ich aus 41-ms-Laeufen hochgerechnet und die festen Aufbaukosten (Voxelierung,
			// Facettenbau) mitskaliert habe -- die sind aber konstant, nicht proportional.
			// Inhalt der Datei: neue physikalische ENDZEIT in MILLISEKUNDEN. Nur Verlaengern wird angenommen;
			// Verkuerzen geht ueber die Stoppdatei, weil die den vollstaendigen Abschlusspfad laeuft.
			if(access(ziel_datei.c_str(), F_OK)==0) {
				double ziel_ms = 0.0;
				{ std::ifstream zf(ziel_datei); zf >> ziel_ms; }
				const ulong neu = (ziel_ms>0.0) ? (ulong)((float)(0.001*ziel_ms)/dt_c + 0.5f) : 0ull;
				// ★ 04.09.2026 (Host-Audit M4): das Loeschen stand VOR der Pruefung. access() sieht die von
				// "echo" per O_CREAT|O_TRUNC bereits angelegte, aber noch nicht beschriebene Datei; der Lauf
				// las dann 0, warnte -- und hatte die Datei schon geloescht. Die Verlaengerung war damit
				// still weg, obwohl der Bediener sie korrekt geschrieben hat. Jetzt: nur nach erfolgreichem
				// Parsen entfernen, sonst beim naechsten Sample erneut versuchen.
				if(ziel_ms>0.0) std::remove(ziel_datei.c_str());
				if(!(ziel_ms>0.0)) print_warning("[ZIEL] "+ziel_datei+" war noch leer oder enthielt keine positive Zahl -- unveraendert stehengelassen, der naechste Sample-Punkt liest erneut. Erwartet wird die neue Endzeit in Millisekunden, z. B. \"echo 501 > "+ziel_datei+"\".");
				else if(neu<=n_outer) print_warning("[ZIEL] "+to_string((float)ziel_ms,1u)+" ms entspricht "+to_string(neu)+" groben Schritten und ist nicht laenger als die aktuelle Planung ("+to_string(n_outer)+") -- ignoriert. Verkuerzen geht ueber "+stop_datei+".");
				else {
					// ★ 04.09.2026 (Host-Audit H1): die Verlaengerung gegen die ZAEHLERLAST pruefen. Slot 7 ist
					// der Nenner jeder Modellabdeckungs-Angabe und wickelt bei fac_N * ceil(n*ratio/100) >= 2^32.
					// Das Soll ist hier ohne Geraetelesung ausrechenbar (dieselbe Formel wie in der Abnahme).
					{	const ulong facN = lbm_f.lbm_domain[0]->fac_N;
						if(facN>0ull) {
							const ulong s7 = facN*(ulong)((neu*(ulong)ratio+99ull)/100ull);
							if(s7>=4294967296ull) {
								const ulong n_max = (4294967295ull/facN)*100ull/(ulong)ratio;
								print_warning("[ZIEL] ACHTUNG: bei "+to_string((float)ziel_ms,1u)+" ms erreicht der Wirkpfad-Zaehler (Slot 7) "+to_string(s7)
									+" und ueberlaeuft den uint-Bereich (2^32). Physik, CSVs und Kraefte bleiben davon unberuehrt -- aber jeder ANTEIL mit Slot 7 als Nenner"
									+" (Solver-Kaskade, Rueckfallquote, Modellabdeckung) ist danach wertlos. Ohne Wickel bleibt der Lauf bis rund "
									+to_string((float)(1000.0*(double)n_max*(double)dt_c),0u)+" ms. Die Verlaengerung wird trotzdem ausgefuehrt.");
							}
						}
					}
					const ulong alt = n_outer; n_outer = neu;
					print_info("[ZIEL] Lauf verlaengert bei t = "+to_string((float)t_si,4u)+" s: "+to_string(alt)+" -> "+to_string(n_outer)
						+" grobe Schritte (Endzeit "+to_string((float)ziel_ms,1u)+" ms). ANGESAGT: die VTK-Rotation und der Kopplungs-Pruefpunkt wurden VOR der Schleife aus der urspruenglichen Planung gerechnet und bleiben darauf.");
				}
			}

			// ---------------------------------------------------- Leistungsbericht
			{
				const double wall = std::chrono::duration<double>(t_now()-wall_begin).count();
				const double phys = t_si - t_phys_begin;
				const double idx  = phys>0.0 ? wall/phys : 0.0;
				const double ges  = ph_kopplung+ph_fein+ph_grob+ph_kraft+ph_schnitt;
				auto pct = [&](const double v) { return ges>0.0 ? to_string((float)(100.0*v/ges),1u) : string("0.0"); };
				print_info("[LEISTUNG] Index = "+to_string((float)idx,0u)+" s_wall/s_phys (V1-Referenz ~12000, kleiner ist besser)"
					+" | je grobem Schritt "+to_string((float)(ges/(double)max(1ull,ph_n)*1000.0),1u)+" ms");
				print_info("[PHASEN]   Kopplung grob->fein "+pct(ph_kopplung)+" % | Nahfeld "+to_string(ratio)+" Schritte "+pct(ph_fein)
					+" % | Fernfeld synchronisieren und entnehmen "+pct(ph_grob)+" % | Kraefte "+pct(ph_kraft)+" % | Schnitte "+pct(ph_schnitt)+" %");
				ph_kopplung=ph_fein=ph_grob=ph_kraft=ph_schnitt=0.0; ph_n=0ull;
				wall_begin = t_now(); t_phys_begin = t_si; // naechstes Fenster
			}
		}
		// ★ FEHLER B-2 (Pruefagent 2026-08-22, HOCH): n_outer_ist stand in der Sample-Kadenz und war
		// damit der letzte GESAMPELTE Aussenschritt, nicht der letzte GELAUFENE. Bei n_outer, das
		// kein Vielfaches von CFD_SAMPLE_EVERY ist, wurde das Facetten-Soll (Zeilen ~4325/4365) zu
		// klein und brach einen voellig korrekten Lauf mit "Ist != Soll" ab -- genau der Fehler, den
		// die Umstellung von n_outer auf n_outer_ist beheben sollte, nur verschoben. Ausserdem haette
		// CFD_SAMPLE_EVERY damit ein hartes Abnahmegatter gesteuert (Kadenz-Neutralitaet verletzt).
		// Hier gehoert sie hin: unbedingt, am Ende jedes wirklich gelaufenen Aussenschritts.
		n_outer_ist = outer+1ull;
		if(stop_angefordert) break;
	}
	// ★ VTK am LAUFENDE: der Fall, fuer den dieser Export gebaut wurde -- das Feld des LETZTEN
	// gerechneten Zeitschritts, aus dem sich jede Ebene, jede Komponente und jede Differenz
	// nah gegen fern spaeter offline ziehen laesst, ohne den Lauf zu wiederholen.
	if(vtk_ende) {
		const int t_ms = (int)((float)t_si_letzt*1000.0f+0.5f); // WIRKLICH erreichte Zeit -- bei sauberem Stopp ist das NICHT n_outer*dt_c
		string ms = to_string(t_ms); while(ms.length()<6u) ms = "0"+ms;
		if(t_ms==vtk_ms_letzt) print_info("[VTK] Enddump uebersprungen -- der letzte Kadenzpunkt hat dasselbe t_ms ("+ms+") bereits geschrieben (Dedup; Datei bleibt, rotiert nicht mehr).");
		else {
		lbm_f.u.read_from_device(); lbm_f.rho.read_from_device(); lbm_f.flags.read_from_device();
		schreibe_vtk_feld(lbm_f, fNx, fNy, fNz, near_x0, near_y0, near_z0, dx_f, si_u/u_lat, vtk_stride, out_dir+"feld_nah_"+ms+"ms.vtk");
		lbm_c.u.read_from_device(); lbm_c.rho.read_from_device(); lbm_c.flags.read_from_device();
		schreibe_vtk_feld(lbm_c, cNx, cNy, cNz, far_x0, far_y0, 0.0f, dx_c, si_u/u_lat, vtk_stride, out_dir+"feld_fern_"+ms+"ms.vtk");
		}
	}
	// Wirkpfad-Nachweis: ein Schalter ohne feuernden Zaehler ist ein harter Fehler (Iron Rule).
	if(vtk_dt>0.0f||vtk_ende||(slice_ns>0ull&&vtk_jede>0u)) {
		if(g_vtk_dateien==0ull) print_error("CFD_VTK_ENDE/CFD_VTK_DT/CFD_VTK_JEDE war gesetzt, es wurde aber KEINE einzige VTK-Datei geschrieben -- stiller No-Op.");
		else print_info("[VTK] Wirkpfad: "+to_string(g_vtk_dateien)+" Dateien, "+to_string((float)g_vtk_bytes/1073741824.0f,2u)+" GB Feld-Daten geschrieben (Rotation loescht nachtraeglich, der Zaehler zaehlt GESCHRIEBENE).");
	}
	if(slice_ns>0ull) { // Wirkpfad der Near-Step-Kadenz (Iron Rule: Schalter ohne feuernden Zaehler = harter Fehler)
		// ★ Pruefagent M1 (27.08.): Soll auf den LETZTEN GESAMPELTEN Punkt beziehen -- Kadenzpunkte
		// feuern nur an Sample-Punkten; sonst falsch-exit(1) bei krummem slice_ns + Ende vor dem
		// naechsten Sample-Punkt (gueltiger Kurzlauf verlor die komplette Auswertung).
		const ulong kad_soll = ((n_outer_ist/(ulong)sample_every)*(ulong)sample_every*(ulong)ratio)/slice_ns;
		if(kad_punkt==0ull&&kad_soll>0ull) print_error("SLICE-KADENZ war aktiv (CFD_SLICE_NEAR_STEPS="+to_string(slice_ns)+"), aber KEIN Kadenzpunkt gefeuert -- stiller No-Op.");
		else print_info("[KADENZ] Wirkpfad: "+to_string(kad_punkt)+" Kadenzpunkte (Soll "+to_string(kad_soll)+").");
	}
	if(stop_angefordert) print_info("[STOPP] Lauf regulaer beendet bei t = "+to_string((float)t_si_letzt,4u)+" s statt der geplanten "
		+to_string((float)n_outer*dt_c,4u)+" s. Alle Ausgaben sind vollstaendig; die Mittelwerte unten beziehen sich auf das VERKUERZTE Fenster.");
	if(n_acc>0ull) print_info("Mittlere Zeit je grobem Schritt: "+to_string((float)(t_acc/(double)n_acc),4u)+" s ("+to_string(ratio)+" feine Schritte inklusive)");

	// ---------------------------------------------------------------- Auswertung
	fcsv.close(); // die Zeilen stehen bereits einzeln auf Platte, siehe Schleife
	ipcsv.close(); print_info("CSV: "+out_dir+"interface_druck.csv (Interface-Druck der 4 getriebenen Ebenen, waehrend des Laufs geschrieben)"); // _exit(0) ruft keine Destruktoren
	sonde_csv.close(); print_info("CSV: "+out_dir+"einlass_saeule_nah.csv (Einlass-Saeulen-Sonde x_f=2/10, Slice-Kadenz, waehrend des Laufs geschrieben)"); // _exit(0) ruft keine Destruktoren
	if(n2f_alpha>0.0f) { swcsv.close(); if(bilcsv.is_open()) bilcsv.close(); /* B10: bilcsv fehlte in den symmetrischen Abbruchpfaden */ print_info("CSV: "+out_dir+"schale_waechter.csv (N2F-Schalen-Waechter, waehrend des Laufs geschrieben)"); } // P9c; _exit(0) ruft keine Destruktoren
	if(zb>0u) { zcsv.close(); print_info("CSV: "+out_dir+"kraft_zband.csv (Band/Rest-Zerlegung, waehrend des Laufs geschrieben)"); } // _exit(0) am Fallende ruft keine Destruktoren -- explizit schliessen
	print_info("CSV: "+out_dir+"forces.csv ("+to_string((uint)ts.size())+" Zeilen, waehrend des Laufs geschrieben)");
	std::vector<double> cd, cz;
	for(size_t i=0u; i<ts.size(); i++) if(ts[i]>=(double)t_warmup) {
		cd.push_back(fx[i]/((double)q_inf*A_ref)); cz.push_back(fz[i]/((double)q_inf*A_ref));
	}
	// ★ Audit-Nacharbeit 14: y+ VOR dem Samples-Waechter messen -- vorher fiel die Messung bei
	// kurzen Laeufen (<16 Kraft-Samples) mit dem _exit zusammen weg, obwohl sie unabhaengig davon ist.
	if(env_u("CFD_F_LISTE_ZENSUS", 0u)>0u) f_listen_zensus(lbm_f, fNx, fNy, fNz, "Nahfeld"); // ★ 03.09.: Vorfrage zu B78(b), Default aus
	if(env_u("CFD_YPLUS", 1u)>0u) { print_info("HINWEIS (WM-Blick C): messe_yplus ist am Fahrzeug DRUCKKONTAMINIERT (tangentiale F-Komponente an Voxeltreppen) -- nur als Anker-Kontinuitaet lesen, nicht absolut."); messe_yplus(lbm_f, fNx, fNy, fNz, nu_lat_f, dx_f, dt_f, si_rho, out_dir, "Nahfeld"); }
	const bool stat_ok = cd.size()>=16u; // ★ Audit 2/3: Dichteklemme/Fx-Anker liefen hinter dem _exit nie bei Kurzlaeufen
	if(!stat_ok) print_warning("Zu wenige Samples -- Cd-Statistik entfaellt, Dichteklemme/Fx-Anker laufen trotzdem.");
	double mcd=0.0, mcz=0.0;
	if(stat_ok) {
	for(size_t i=0u; i<cd.size(); i++) { mcd+=cd[i]; mcz+=cz[i]; }
	mcd/=(double)cd.size(); mcz/=(double)cz.size();
	} // stat_ok
	if(zen_da) { // ★ ZENSUS Mehrfachfacetten (27.08.): Gate-Wert fuer den Cluster-Umbau.
	// AUSSERHALB stat_ok (Pruefagent-Befund B4, Muster von Zeile 5584): stat_ok verlangt >= 16
	// Kraftsamples -- genau ein kurzer Zensuslauf haette sonst NICHTS gedruckt.
		const ulong zges = zen_voll+zen_proj+zen_unklar;
		print_info("FACETTEN-ZELLKLASSEN der Kraftschleife: "+to_string(zges)+" Markerzellen -- unbehandelt (voll) "
			+to_string(zen_voll)+" ("+to_string(100.0f*(float)zen_voll/(float)max(1ull,zges),2u)+" %), projiziert "
			+to_string(zen_proj)+" ("+to_string(100.0f*(float)zen_proj/(float)max(1ull,zges),2u)+" %), UNKLAR "
			+to_string(zen_unklar)+" ("+to_string(100.0f*(float)zen_unklar/(float)max(1ull,zges),2u)+" %).");
		print_info("  unklar = |Summe der Nachbar-Facettennormalen| < 0,5, also gegenlaeufige Wandseiten in EINER"
			" Nachbarschaft (duenne Platte, Spalt, Kante). Diese Zellen gehen konservativ VOLL in die Kraft.");
		if(zen_ukraft) {
			const bool nenner_ok = fabs(zen_px)>1e-12&&fabs(zen_pz)>1e-12; // 1e-30 lag weit unter der Arbeitsskala (px ist O(1e-3..1e-1))
			const double axl = nenner_ok ? 100.0*zen_ux/zen_px : 0.0;
			const double azl = nenner_ok ? 100.0*zen_uz/zen_pz : 0.0;
			print_info("  KRAFTGEWICHT der unklaren Zellen (Host-Pfad, letztes Sample) am FACETTEN-DRUCKANTEIL"
				" (Reibung NICHT enthalten -- als Cd-Anteil zitiert waere die Zahl zu gross): Fx "+to_string((float)axl,2u)
				+" %, Fz "+to_string((float)azl,2u)+" % (Gittereinheiten: Fx_unklar "+to_string((float)zen_ux,6u)+" von "+to_string((float)zen_px,6u)
				+", Fz_unklar "+to_string((float)zen_uz,6u)+" von "+to_string((float)zen_pz,6u)+"). Vorzeichenbehaftet -- fuer das Gate den BETRAG lesen.");
			print_info("  GATE Cluster-Umbau: erst ein RELEVANTER Kraftanteil rechtfertigt ihn -- Promille ohne Kraft = Akte zu.");
		} else print_info("  Kraftgewicht NICHT gemessen: der reine GPU-Pfad reduziert nur die Zaehler. Fuer den Gate-Wert"
			" einen Zensuslauf mit CFD_FAC_GPU=0 (Host allein, sauber) fahren -- CFD_FAC_GPU_PRUEF=1 ginge auch,\n"
			" mischt aber GPU-Nenner mit Host-Zaehler (Pruefagent-Befund B6, praktisch 3e-8..5e-5).");
	}
	print_info("---------------------------------------------------------------");
		{	// ★ Hygiene E6b: fx_c wurde den ganzen Lauf befuellt und nie gelesen. Jetzt als EIN Anker
		// ausgewiesen: das Mittel der Fernfeld-Fahrzeugkraft AB WARMLAUF (derselbe Filter wie Cd/Cz --
		// der Nachpruefer fand das Mittel ueber alles inkl. Anlaufstoss irrefuehrend, zu Recht).
		// Die volle Zeitreihe steht ohnehin als Spalte Fx_far_N in forces.csv; wer Spruenge oder
		// Vorzeichenwechsel sucht, schaut dort.
		double m=0.0; uint nm=0u;
		for(size_t i=0; i<fx_c.size(); i++) if(ts[i]>=(double)t_warmup) { m += fx_c[i]; nm++; }
		if(nm>0u) print_info("Fernfeld-Fahrzeugkraft Fx (Mittel ab Warmlauf): "+to_string((float)(m/(double)nm),1u)+" N ueber "+to_string(nm)+" Samples (Zeitreihe: Spalte Fx_far_N in forces.csv)"
			+(env_u("CFD_FERN_FACETTEN",0u)>0u?string(" -- ACHTUNG P8: PHANTOMBEHAFTET (object_force an facettenbehandelten Links des Fernfelds), nur als Arm-DIFFERENZ werten."):string("")));
	}
	{ ulong h=0ull; berichte_dichteklemme(lbm_f, "Nahfeld", h); berichte_dichteklemme(lbm_c, "Fernfeld", h); dichteklemme_fazit(h); }
	if(stat_ok) {
	// ★ 03.09.2026 INSTRUMENTEN-ETIKETT (Befund B79). Diese Zeilen stammen aus object_force, also aus
	// dem Impulsaustausch an den Koerperzellen. Sobald die Facettenkette laeuft, traegt dieser Pfad
	// PHANTOM-REIBUNG an facettenbehandelten Links -- der Kopf von kraft_zband.csv sagt das seit
	// jeher, der Report sagte es NICHT und stellte die Zahl trotzdem gegen OF13. Gemessen ueber vier
	// Laeufe (p4_nb 8,50 | p4_ref 8,88 | w_nb 7,54 | w_fdwand 7,84 gegen OF13 0,599) ist Cd damit um
	// gut das Vierzehnfache aufgeblasen, und bei 8 mm kippt object_force sogar das VORZEICHEN von Cz
	// (+0,34 statt negativ). Der gueltige absolute Bezug ist der Facettenpfad in cd_facetten.csv:
	// cd_druck_rest + cd_reib = 0,5924 gegen OF13 0,599, also -1,1 % (p4_nb).
	// Die OF13-Prozente werden deshalb nur noch OHNE Facettenkette gedruckt.
	const bool phantom = env_u("CFD_FACETTEN", 0u)>0u;
	print_info("Zeitmittel ab "+to_string(t_warmup,3u)+" s ueber "+to_string((uint)cd.size())+" Samples:");
	if(phantom) {
		print_info("  ACHTUNG: die folgenden Cd/Cz stammen aus object_force und sind an facettenbehandelten");
		print_info("           Links PHANTOMBEHAFTET -- NICHT gegen OF13 stellen, nur als Arm-DIFFERENZ werten.");
		print_info("           Gueltiger absoluter Bezug: cd_facetten.csv (cd_druck_rest + cd_reib bzw. cz).");
		print_info("  Cd = "+to_string((float)mcd,4u)+"   (object_force, phantombehaftet)");
		print_info("  Cz = "+to_string((float)mcz,4u)+"   (object_force, phantombehaftet)");
	} else {
		print_info("  Cd = "+to_string((float)mcd,4u)+"   (OpenFOAM 13: 0.599, Abweichung "+to_string((float)(100.0*(mcd/0.599-1.0)),1u)+" %)");
		print_info("  Cz = "+to_string((float)mcz,4u)+"   (OpenFOAM 13: -1.301, Abweichung "+to_string((float)(100.0*(mcz/-1.301-1.0)),1u)+" %)");
	}
	for(uint k : {4u, 8u, 16u}) { const double se=block_sem(cd,k); if(se>=0.0) print_info("      Block-SEM Cd ueber "+to_string(k)+" Bloecke: +- "+to_string((float)se,5u)); }
	for(uint k : {4u, 8u, 16u}) { const double se=block_sem(cz,k); if(se>=0.0) print_info("      Block-SEM Cz ueber "+to_string(k)+" Bloecke: +- "+to_string((float)se,5u)); } // WM-Blick C: Cz lief ohne Fehlerbalken -- ehrlich >=0,03, Delta-Cz-0,1-Aussagen sind 2-sigma
	if(zb>0u&&zb_nn>0ull) { // ★ KRAFT-ZBAND-Endreport (Zeitmittel ab Warmlauf ueber dieselben Samples)
		const double mcd_b=zb_cd_band/(double)zb_nn, mcz_b=zb_cz_band/(double)zb_nn;
		const double mcd_r=zb_cd_rest/(double)zb_nn, mcz_r=zb_cz_rest/(double)zb_nn;
		print_info("KRAFT-ZBAND (unterste "+to_string(zb)+" Zellen = "+to_string((float)zb*dx_f*1000.0f,2u)+" mm; GITTERBAND -- zwischen DX-Sprossen nicht direkt vergleichbar), "+to_string(zb_nn)+" Samples:");
		print_info("  Band: Cd = "+to_string((float)mcd_b,4u)+"   Cz = "+to_string((float)mcz_b,4u));
		print_info("  Rest: Cd = "+to_string((float)mcd_r,4u)+"   Cz = "+to_string((float)mcz_r,4u));
		for(uint k : {4u, 8u, 16u}) { const double se=block_sem(zb_cz_rest_reihe,k); if(se>=0.0) print_info("      Block-SEM Cz_rest ueber "+to_string(k)+" Bloecke: +- "+to_string((float)se,5u)); }
		print_info("  Selbsttest-Maximum |Band+Rest-Gesamt|/max(|Fx|,|Fz|): "+to_string((float)zb_selftest_max,9u)+" (Soll < 5e-5)");
	if(zb_selftest_max>=5e-5) print_warning("ZBAND-Selbsttest ueber 5e-5 -- Zerlegung nicht belastbar (float-Atomik-Marge ist 3,5x, das hier ist mehr).");
		if(phantom) print_info("  Cz_rest = "+to_string((float)mcz_r,4u)+" (object_force, phantombehaftet -- der OF13-Vergleich stand hier bis 03.09. UNGEKENNZEICHNET und war die Quelle der 'Faktor-2-Diskrepanz' gegen cd_facetten.csv; Befund B79)");
		else print_info("  Cz_rest vs OF13 -1,301: "+to_string((float)mcz_r,4u)+" (Abweichung "+to_string((float)(100.0*(mcz_r/-1.301-1.0)),1u)+" %)");
	}
	} // stat_ok
	{	// ★ UNTERBODEN-SONDE (Heiko 2026-08-19: Unterboden in ALLEN s5b-Slices tot, arm-unabhaengig).
		// Je x-Spalte unter dem Fahrzeug: mittleres u_x/u_inf ueber alle Fluidzellen im Spalt
		// zwischen Fahrbahn (z=1) und Unterbodenflaeche (erste TYPE_X-Zelle der Saeule). Laeuft in
		// JEDEM Arm -- das A/B AUS vs. Facetten entscheidet Architektur- vs. Mechanismus-Ursache.
		lbm_f.u.read_from_device(); lbm_f.flags.read_from_device();
		std::ofstream ub(out_dir+"unterboden_sonde.csv"); ub << "x_m,u_rel,n_zellen" << std::endl; ub.precision(6);
		double su_min=1e300, su_sum=0.0; ulong nx_mit=0ull;
		for(uint x=0u; x<fNx; x++) {
			double su=0.0; ulong nc=0ull;
			for(uint y=0u; y<fNy; y++) {
				uint zdach=0u; // erste Fahrzeugzelle der Saeule
				for(uint z=1u; z<fNz; z++) { const ulong n=(ulong)x+((ulong)y+(ulong)z*(ulong)fNy)*(ulong)fNx; if((lbm_f.flags[n]&TYPE_X)!=0u) { zdach=z; break; } }
				if(zdach<2u) continue; // kein Fahrzeug ueber dieser Saeule oder Latsch (zdach=1 = Kontakt)
				if((float)zdach*dx_f>0.35f) continue; // R2-Nachpruefer: Ueberhang-Saeulen (Spiegel/Fluegel) zaehlen sonst freie Seitenstroemung als "Spalt"
				for(uint z=1u; z<zdach; z++) { const ulong n=(ulong)x+((ulong)y+(ulong)z*(ulong)fNy)*(ulong)fNx;
					if((lbm_f.flags[n]&(TYPE_S|TYPE_E))==0u) { su+=(double)lbm_f.u.x[n]; nc++; } } // schliesst TYPE_MS (z=1-Schicht) BEWUSST aus: Band-Mitschleppung ist keine Durchstroemung (R2)
			}
			if(nc>0ull) { const double ur=su/((double)nc*(double)u_lat);
				ub << (near_x0+(double)x*dx_f) << "," << ur << "," << nc << std::endl;
				su_min=fmin(su_min,ur); su_sum+=ur; nx_mit++; }
		}
		if(nx_mit>0ull) print_info("Unterboden-Sonde: u_x/u_inf Mittel "+to_string((float)(su_sum/(double)nx_mit),3u)
			+", Minimum "+to_string((float)su_min,3u)+" ueber "+to_string(nx_mit)+" x-Spalten (Soll deutlich > 0; CSV: unterboden_sonde.csv)");
		if(nx_mit>0ull&&su_sum/(double)nx_mit<0.1) print_warning("UNTERBODEN TOT (< 10 % u_inf im Mittel) -- Abtrieb kann so nicht entstehen (Cz-Blocker, arm-unabhaengig pruefen).");
		{	// ★ Einlass-Saeule (Heiko 2026-08-21, Streifen-Verdacht): 20 cm hinter dem FAR-Einlass,
			// y-Mitte, ALLE z -- Staggered-Beweis waere eine 2-Zellen-Oszillation in ux(z).
			lbm_c.u.read_from_device();
			const uint ex=(uint)(0.2f/dx_c+0.5f), ey=cNy/2u; std::ofstream ep(out_dir+"einlass_saeule.csv"); ep.precision(6);
			ep << "# x = Einlass+" << 0.2f << " m (Zelle " << ex << ", dx_c " << dx_c*1000.0f << " mm), y-Mitte" << std::endl << "z_m,ux_rel,uy_rel,uz_rel" << std::endl;
			double flips=0.0, dmax=0.0; float vor=0.0f, letzte_d=0.0f; bool hat_vor=false;
			for(uint z=0u; z<cNz; z++) { const ulong n=(ulong)ex+((ulong)ey+(ulong)z*(ulong)cNy)*(ulong)cNx;
				const float ux=lbm_c.u.x[n]/u_lat, uy=lbm_c.u.y[n]/u_lat, uz=lbm_c.u.z[n]/u_lat;
				ep << (double)z*(double)dx_c << "," << ux << "," << uy << "," << uz << std::endl;
				if(hat_vor) { const float d=ux-vor; dmax=fmax(dmax,(double)fabs(d)); if(z>=2u&&((d>0.0f)!=(letzte_d>0.0f))&&fabs(d)>1e-3f&&fabs(letzte_d)>1e-3f) flips+=1.0; letzte_d=d; } else letzte_d=0.0f;
				vor=ux; hat_vor=true; }
			print_info("Einlass-Saeule (x=+0,2 m, y-Mitte): Nachbar-Vorzeichenwechsel in dux/dz = "+to_string((float)flips,0u)+" von "+to_string(cNz-2u)+" moeglichen, max|dux| = "+to_string((float)dmax,4u)+" u_inf (2-Zellen-Oszillation = Staggered-Beweis). CSV: einlass_saeule.csv");
		}
		{	// ★ Boden-Laengsprofil (Heiko 2026-08-19, V1-Instrument): u_x in z=1..5 entlang x ueber den
			// Mittelstreifen (y_mid +- 25 Zellen), INKLUSIVE Einlaufstrecke -- macht den beobachteten
			// Einbruch direkt nach dem Einlass quantitativ (Verdacht: Fernfeld-Erbe, Spalt grob <2 Zellen).
			std::ofstream bl(out_dir+"boden_laengsprofil.csv");
			bl << "# dx_mm=" << dx_f*1000.0f << " band_z=1..5 = " << dx_f*1000.0f << ".." << 5.0f*dx_f*1000.0f << " mm, y_mid+-" << 25u << " Zellen -- Tiefen-Audit B2: GITTERBAND, zwischen DX-Sprossen NICHT direkt vergleichbar" << std::endl;
			bl << "x_m,u_rel_z1_5,n_zellen" << std::endl; bl.precision(6);
			const uint ymid=fNy/2u, yb=min(25u, fNy/2u>1u?fNy/2u-1u:0u); // B6: uint-Unterlauf bei schmalem Nahfeld
			for(uint x=0u; x<fNx; x++) {
				double su2=0.0; ulong nc2=0ull;
				for(uint y=ymid-yb; y<=ymid+yb; y++) for(uint z=1u; z<=5u; z++) {
					const ulong n=(ulong)x+((ulong)y+(ulong)z*(ulong)fNy)*(ulong)fNx;
					if((lbm_f.flags[n]&(TYPE_S|TYPE_E))==0u) { su2+=(double)lbm_f.u.x[n]; nc2++; }
				}
				if(nc2>0ull) bl << (near_x0+(double)x*dx_f) << "," << su2/((double)nc2*(double)u_lat) << "," << nc2 << std::endl;
			}
			print_info("Boden-Laengsprofil geschrieben: boden_laengsprofil.csv (Mittelstreifen, z=1..5).");
		}
	}
	if(env_u("CFD_FACETTEN", 0u)==0u&&env_u("CFD_FAC_K4", 0u)>0u) { // ★ K4 am FAHRZEUG (Heiko 2026-08-18): Hybrid-Schaetzer vs object_force am unbehandelten BB -- ohne diese Eichung ist kein AUS-vs-Facetten-Cd-Vergleich belastbar
		const std::vector<double> leer;
		const FacKraft FK0 = kraft_facetten(lbm_f, fNx, fNy, fNz, (uchar)(TYPE_S|TYPE_X), 1ull, leer);
		const float3 Fo = lbm_f.object_force(TYPE_S|TYPE_X);
		print_info("K4 dd (AUS-Arm): kraft_facetten Fx = "+to_string((float)FK0.px,6u)+" vs object_force Fx = "+to_string(Fo.x,6u)
			+" (rel. Abw. "+to_string((float)(Fo.x!=0.0f?fabs(FK0.px/(double)Fo.x-1.0):0.0),8u)+", Soll < 5e-5 -- Atomik-Reihenfolge skaliert mit N, produktiv gemessen 1,4e-5); Fz: "+to_string((float)FK0.pz,6u)+" vs "+to_string(Fo.z,6u));
	}
	if(env_u("CFD_BODEN_EQ",0u)>0u||env_u("CFD_FERN_BODEN_EQ",0u)>0u) { // XL-3 M2: BODEN_EQ-Wirkpfad-Nachweis IM Binary (Iron Rule 3 -- die V1-Vorlage apply_floor_velocity war jahrelang stiller No-Op, 2f705ba)
		lbm_f.lbm_domain[0]->rho_clamp_hits.read_from_device(); lbm_c.lbm_domain[0]->rho_clamp_hits.read_from_device();
		const ulong bqf=(ulong)lbm_f.lbm_domain[0]->rho_clamp_hits[20], bqc=(ulong)lbm_c.lbm_domain[0]->rho_clamp_hits[20];
		print_info("BODEN_EQ-Wirkpfad: Nahfeld "+to_string(bqf)+", Fernfeld "+to_string(bqc)+" Band-Resets (t%100-Stichprobe; ABSTAND-Aussparungen senken den Zaehler ehrlich).");
		if(env_u("CFD_BODEN_EQ",0u)>0u&&bqf==0ull) print_error("CFD_BODEN_EQ gesetzt, aber Nahfeld-Wirkpfad NULL -- lautloser No-Op.");
		if(env_u("CFD_FERN_BODEN_EQ",0u)>0u&&bqc==0ull) print_error("CFD_FERN_BODEN_EQ gesetzt, aber Fernfeld-Wirkpfad NULL -- lautloser No-Op.");
	}
	if(env_u("CFD_FERN_EINLASS_EQ",0u)>0u) { // ★ EINLASS_EQ-Wirkpfad-Nachweis (Muster BODEN_EQ, Slot 21)
		lbm_f.lbm_domain[0]->rho_clamp_hits.read_from_device(); lbm_c.lbm_domain[0]->rho_clamp_hits.read_from_device();
		const ulong eqf=(ulong)lbm_f.lbm_domain[0]->rho_clamp_hits[21], eqc=(ulong)lbm_c.lbm_domain[0]->rho_clamp_hits[21];
		print_info("EINLASS_EQ-Wirkpfad: Fernfeld "+to_string(eqc)+" Spalten-Resets (t%100-Stichprobe), Nahfeld "+to_string(eqf)+" (Soll 0 -- Nah-x- ist Kopplungsebene).");
		if(eqc==0ull) print_error("CFD_FERN_EINLASS_EQ gesetzt, aber Fernfeld-Wirkpfad NULL -- lautloser No-Op.");
		if(eqf!=0ull) print_error("Nahfeld zaehlt EINLASS_EQ-Wirkpfad -- es MUSS unberuehrt bleiben (read-once-Bruch?).");
	}
	if(n2f_alpha>0.0f) { // ★ P9c: N2F-SCHALE-Wirkpfad-Endnachweis Slot 22 (Muster EINLASS_EQ)
		lbm_f.lbm_domain[0]->rho_clamp_hits.read_from_device(); lbm_c.lbm_domain[0]->rho_clamp_hits.read_from_device();
		const ulong swf=(ulong)lbm_f.lbm_domain[0]->rho_clamp_hits[22], swc=(ulong)lbm_c.lbm_domain[0]->rho_clamp_hits[22];
		print_info("N2F-SCHALE-Wirkpfad: Fernfeld "+to_string(swc)+" Blend-Zellen (t%100-Stichprobe; Solid-/NaN-Skips senken den Zaehler ehrlich), Nahfeld "+to_string(swf)+" (Soll 0 -- der Blend laeuft NUR im Fernfeld).");
		// t laeuft im Blend 1..n_outer (Vorlauf-run(1) hatte t=0 VOR alloc_schale) -- t%100 feuert
		// floor(n_outer/100) mal; bei Laeufen unter 100 groben Schritten ist 0 also KEIN Befund.
		if(swc==0ull) { if(n_outer_ist<100ull) print_warning("N2F-SCHALE-Wirkpfad 0, aber der Lauf hat unter 100 grobe Schritte -- die t%100-Stichprobe hat nie gefeuert (kein Befund; laenger laufen fuer den Nachweis)."); else print_error("CFD_N2F_SCHALE gesetzt, aber Fernfeld-Wirkpfad NULL -- lautloser No-Op (Iron Rule 3)."); }
		// ★ PARITAETSNACHWEIS (Pruefagent-M2, 2026-08-22): der FNEQ-Arm muss bei alpha == 0 exakt
		// degenerieren. Slot 23 zaehlt geraeteintern jede gespeicherte Verteilung, die vom geladenen
		// Wert abweicht -- ein Dateivergleich kann das im dd-Fall nicht leisten (die Aktivierung
		// macht das Fernfeld ueber po_mean nichtdeterministisch, gemessen).
		// ★★ PAARUNGS- UND MOMENTEN-BEWEIS (Slots 25/26, gebaut 2026-08-22 mittags als Ersatz fuer
		// den alten Zaehler). Er rechnet die Momente aus ftrue NEU und stellt sie gegen die aus fhn:
		// rho muss invariant, u exakt negiert sein. Ist die EsoPull-Paarung falsch, negiert u nicht.
		// Anders als Slot 23/24 haengt er NICHT an alpha und laeuft in BEIDEN Armen, die ftrue
		// benutzen (FNEQ und IDENT). Der alte Zaehler war gegen diese Fehlerklasse strukturell
		// blind -- er verglich x+(y-x) gegen y, und beide Fehler kuerzen sich darin heraus.
		if(n2f_modus==1u||n2f_modus==2u) {
			const ulong pn=(ulong)lbm_c.lbm_domain[0]->rho_clamp_hits[25];
			const double pm=(double)lbm_c.lbm_domain[0]->rho_clamp_hits[26]*1.0e-9;
			if(swc==0ull) print_info("N2F-PAARUNGSBEWEIS: keine Stichprobe (Wirkpfad 0) -- keine Aussage.");
			else if(pn>0ull) print_error("N2F-PAARUNGSBEWEIS GESCHEITERT: in "+to_string(pn)+" Zell-Stichproben negieren die Momente aus ftrue NICHT die aus fhn (groesste relative Abweichung "+to_string((float)pm,9u)+", Schwelle 1e-5). Die EsoPull-Paarung pair(0)=0, pair(2k-1)=2k, pair(2k)=2k-1 stimmt nicht mehr, oder calculate_rho_u wurde geaendert. Der ganze Rueckkopplungsarm blendet dann gegen die falsche Verteilung.");
			else print_info("N2F-PAARUNGSBEWEIS BESTANDEN: 0 Zell-Stichproben verletzen die Negations-Identitaet, groesste relative Abweichung "+to_string((float)pm,9u)+" (Schwelle 1e-5). rho ist invariant, u exakt negiert -- die EsoPull-Paarung traegt.");
		}
		// ★ FEHLER (Pruefagent 2026-08-22): CFD_N2F_PARITAET=1 ohne CFD_N2F_SCHALE_FNEQ=1 fuhr einen
		// vollen GPU-Lauf, kuendigte einen Beweis an und lieferte keine einzige Beweiszeile -- der
		// Zaehlcode liegt hinter dem EQ-Ruecksprung, und dieser Block lief nur fuer modus==1.
		if(n2f_paritaet>0u&&n2f_modus==0u) print_error("CFD_N2F_PARITAET=1 im EQ-Arm (Modus 0): der EQ-Arm benutzt ftrue gar nicht, es gibt nichts zu beweisen -- der Lauf waere ein voller GPU-Lauf ohne eine einzige Beweiszeile. CFD_N2F_SCHALE_FNEQ=1 (Modus 1) oder CFD_N2F_SCHALE_IDENT (Modus 2) dazusetzen.");
		if(n2f_modus==1u) {
			const ulong par=(ulong)lbm_c.lbm_domain[0]->rho_clamp_hits[23];
			if(n2f_paritaet>0u) {
				// ★ WARUM DER TEST EIN MASS BRAUCHT UND KEIN "MUSS NULL SEIN" (gemessen 2026-08-22):
				// bei a == 0 ist u2 == u_lokal EXAKT (1.0f*x+0.0f*y == x), also feq == feq_loc
				// bitgleich. Die Schlusszeile lautet aber feq[i] += ftrue[i] - feq_loc[i], und
				// a + (b - a) ist in Gleitkomma NICHT exakt b: wo feq_loc und ftrue verschiedene
				// Groessenordnungen haben, verliert die Differenz Stellen, die die Addition nicht
				// zurueckholt. Der FNEQ-Arm KANN also gar kein bitexaktes No-Op sein -- die
				// Forderung war falsch gestellt, nicht die Implementierung.
				// Aussagekraeftig ist die GROESSE: Rundung liegt bei 1-2 ULP. Alles darueber ist
				// ein Bruch der Paarung oder der Momentenrechnung und damit ein echter Befund.
				const double rel=(double)lbm_c.lbm_domain[0]->rho_clamp_hits[24]*1.0e-9;
				// Schwelle = Aufloesung des Speicherformats. FP16C traegt rund 11 Bit Mantisse,
				// also ~4,9e-4. Was darunter liegt, ist im Feld nicht darstellbar und damit kein
				// Unterschied; was darueber liegt, ist einer.
				// ★ Schwelle AN DAS SPEICHERFORMAT gekoppelt (Pruefagent 2026-08-22): sie stand hart als
				// 4,9e-4 da, ohne #ifdef. Wer defines.hpp auf FP32 stellt, bekaeme einen Test, der um
				// vier Groessenordnungen zu lax ist und schweigt.
#ifdef FP16C
				const double aufl = 4.9e-4;  const string auflname = "FP16C (1-4-11)";
#elif defined(FP16S)
				const double aufl = 9.8e-4;  const string auflname = "FP16S (1-5-10)";
#else
				const double aufl = 1.2e-7;  const string auflname = "FP32";
#endif
				if(rel>aufl) print_error("N2F-RUNDUNGSTEST GESCHEITERT: bei alpha = 0 weichen "+to_string(par)+" ZELLEN ab, groesste RELATIVE Abweichung "+to_string((float)rel,9u)+" -- ueber der Aufloesung des Speicherformats "+auflname+" ("+to_string((float)aufl,9u)+") und damit im Feld darstellbar. Erste Verdaechtige: alpha kommt nicht als exakte 0 im Kernel an, oder calculate_f_eq wurde geaendert.");
				else print_info("N2F-RUNDUNGSTEST BESTANDEN: alpha = 0, "+to_string(par)+" Zellen weichen in float32 ab, groesste RELATIVE Abweichung "+to_string((float)rel,9u)+", unter der Aufloesung des Speicherformats "+auflname+" ("+to_string((float)aufl,9u)+"). GRENZE DER AUSSAGE: dieser Test misst NUR die Rundung von x+(y-x) gegen y und dass alpha als exakte 0 ankommt. Paarung und Momentenrechnung kuerzen sich darin heraus -- die pruefen die Slots 25/26 (Paarungsbeweis). Ein bitexaktes No-Op ist mit feq += ftrue - feq_loc ohnehin nicht erreichbar (a+(b-a) rundet).");
			}
			else print_info("N2F-Blend-Paritaetszaehler (Slot 23, Diagnose): "+to_string(par)+" abweichende Verteilungen bei alpha = "+to_string(n2f_alpha,7u)+" -- bei alpha > 0 ist ein Wert > 0 ERWARTET (der Blend soll ja wirken); der Beweis laeuft ueber einen eigenen alpha=0-Arm.");
		}
		if(swf!=0ull) print_error("Nahfeld zaehlt Schalen-Blend-Wirkpfad -- es MUSS unberuehrt bleiben (alpha-Statik-Bruch: s_schale_alpha muss fuer lbm_f EXPLIZIT 0 sein).");
	}
	if(env_u("CFD_FACETTEN", 0u)>0u) { // ★ Stufe 5: Pruefpfade IMMER (ausserhalb stat_ok -- Audit-R1-Muster)
		LBM_Domain* df = lbm_f.lbm_domain[0];
		df->rho_clamp_hits.read_from_device();
		const ulong wz=(ulong)df->rho_clamp_hits[7], soll=df->fac_N*(ulong)((n_outer_ist*(ulong)ratio+99ull)/100ull); // n_outer_IST: nach sauberem Stopp ist die geplante Zahl falsch
		print_info("Facetten-Wirkpfad Nahfeld: "+to_string(wz)+" (Soll "+to_string(soll)+" mod 2^32), tau-Klemme "+to_string((ulong)df->rho_clamp_hits[8])
			+", u_t~0-Skips "+to_string((ulong)df->rho_clamp_hits[9])
			+(env_u("CFD_FACETTEN",0u)>=3u?(", iMEM: u_s-Klemme/Gate "+to_string((ulong)df->rho_clamp_hits[10])+", Skalar "+to_string((ulong)df->rho_clamp_hits[12])
			+", LSQ-Rueckfall "+to_string((ulong)df->rho_clamp_hits[65])+", ohneTang "+to_string((ulong)df->rho_clamp_hits[13])+" (davon mit rohen Tangentialmomenten [27], NICHT ELIBB-heilbar -- Rang, s. B83: "+to_string((ulong)df->rho_clamp_hits[27])+")"+", Rang2 "+to_string((ulong)df->rho_clamp_hits[14])+", Rang0-BB "+to_string((ulong)df->rho_clamp_hits[15])
			+", sn-Klemme/Gate "+to_string((ulong)df->rho_clamp_hits[16])+", PEMA-utb "+to_string((ulong)df->rho_clamp_hits[17])+", alpha>ut "+to_string((ulong)df->rho_clamp_hits[18])+", APG-Klemme "+to_string((ulong)df->rho_clamp_hits[19])+", ELIBB[67] "+to_string((ulong)df->rho_clamp_hits[67])+", MLS[68] "+to_string((ulong)df->rho_clamp_hits[68])+", Rueckfall-Buchung[69] "+to_string((ulong)df->rho_clamp_hits[69])+", Quergate[64] "+to_string((ulong)df->rho_clamp_hits[64])):string("")));
		{ const uint* H=df->rho_clamp_hits.data(); pruefe_rueckfall_buchung(H[69],H[10],H[13],H[15],H[16],H[64],nahfeld_satgate,"Nahfeld"); pruefe_kraftpfad(H[70],H[69],H[7],H[9],H[17],H[71],nahfeld_kraft,"Nahfeld"); pruefe_kaskade(H,"Nahfeld",nahfeld_messnur,nahfeld_pinv); bericht_zielerfuellung(H,(ulong)H[7],(ulong)H[9],(ulong)H[17],(ulong)H[69],"Nahfeld"); }
		bericht_klassen(df, FFn, out_dir, 0.0, "Nahfeld");
		bericht_gdiag(df, FFn, out_dir, "Nahfeld");
		// ★ 03.09. NACHBAR-Wirkpfad (Slots 72/73/74) -- bis heute NIRGENDS im Host ausgelesen (Iron Rule: Schalter ohne feuernden Zaehler = harter Fehler).
		if(env_u("CFD_FAC_NACHBAR",0u)>0u&&!nahfeld_messnur) { const ulong n72=(ulong)df->rho_clamp_hits[72], n73=(ulong)df->rho_clamp_hits[73], n74=(ulong)df->rho_clamp_hits[74]; const ulong ns=n72+n73+n74;
			if(n72==0ull) print_error("[Nahfeld] CFD_FAC_NACHBAR war angefordert, aber Slot 72 = 0 -- stiller No-Op (Emission? kein Fluidnachbar: "+to_string(n73)+", Nachbar still: "+to_string(n74)+").");
			else { if(n72>=0xF0000000ull||n73>=0xF0000000ull||n74>=0xF0000000ull) print_error("[Nahfeld] NACHBAR-Zaehler GESAETTIGT (72="+to_string(n72)+", 73="+to_string(n73)+", 74="+to_string(n74)+") -- die Prozente sind NICHT belastbar. Lauf kuerzen oder Stichprobe ausduennen."); // ★ 03.09.: 4 mm laeuft auf 1,57e9 = 37 % des uint-Bereichs
				else if(n72+n73+n74>3865470566ull) print_warning("[Nahfeld] NACHBAR-Zaehler ueber 90 % des uint-Bereichs ("+to_string(n72+n73+n74)+") -- WICKELGEFAHR beim naechsten laengeren Lauf.");
				print_info("[Nahfeld] NACHBAR-Wirkpfad: Slot 72 angewandt = "+to_string(n72)+" ("+to_string((float)(ns>0ull?100.0*(double)n72/(double)ns:0.0),1u)+" % der Nachbar-Entscheide), 73 kein Fluidnachbar = "+to_string(n73)+", 74 Nachbar still = "+to_string(n74)+" (t%100-Stichprobe; UTKORR wirkt nur an 73/74)."); } }
		if(env_u("CFD_SGS_FDWAND",0u)>0u) { const ulong s39=(ulong)df->rho_clamp_hits[76]; if(s39==0ull) print_error("[Nahfeld] SGS_FDWAND war angefordert, aber Slot 76 = 0 -- stiller No-Op."); else print_info("[Nahfeld] SGS_FDWAND-Wirkpfad Slot 76 = "+to_string(s39)+"."); }
		// ★ 03.09. F-MARKERLISTE: Slot 77 zaehlt jedes store3_F IN der F-BBox, das keinen Slot fand.
		// Das ist der Ersatz fuer den F-Waechter, der unter der Liste gegenstandslos wird: waere die
		// Host-Maske keine echte Obermenge des Kernel-Praedikats, ginge dort still Kraft verloren.
		if(df->f_liste_on) { const ulong s77=(ulong)df->rho_clamp_hits[77];
			if(s77>0ull) print_error("[Nahfeld] F-MARKERLISTE: Slot 77 = "+to_string(s77)+" Schreibversuche OHNE Slot. Die Host-Maske ist KEINE Obermenge des Kernel-Praedikats -- an diesen Zellen geht Kraft still verloren. Ergebnis nicht verwenden.");
			else print_info("[Nahfeld] F-MARKERLISTE: Slot 77 = 0 -- jede vom Kernel beschriebene Zelle hatte einen Slot ("+to_string(df->f_slots)+" Slots).");
		}
		if(LBM_Domain::s_fac_elibb_pur) { // ★ Pruefbefund Messlogik-3 (2026-08-25): der Pur-Return sitzt VOR Slot 7 -- das alte Soll kannte den Pur-Modus nicht und print_error (=exit) toetete den projizierten Cd-Pfad. Pur-Soll: Slot 7 = 0, ELIBB-Wirkpfad (Slot 67) > 0.
			if(wz!=0ull) print_error("Pur-Arm: Slot 7 muesste 0 sein -- Return-Position verschoben?");
			else if((ulong)df->rho_clamp_hits[67]==0ull) print_error("Pur-Arm: ELIBB-Wirkpfad (Slot 67) ist NULL -- lautloser No-Op.");
			else print_info("Pur-Arm (Nahfeld): Slot 7 = 0 (konstruktiv), ELIBB-Wirkpfad "+to_string((ulong)df->rho_clamp_hits[67])+" -- Abnahme in Pur-Form bestanden.");
		} else if(wz!=(soll&0xFFFFFFFFull)) print_error("Facetten-Wirkpfad Ist != Soll im Nahfeld -- Lookup oder Bindung defekt.");
		lbm_c.lbm_domain[0]->rho_clamp_hits.read_from_device();
		// ★ P8: die Negativ-Kontrolle gilt NUR im BB-Fernfeld -- bei CFD_FERN_FACETTEN>0 ZAEHLT das
		// Fernfeld seinen Wirkpfad (Ist=Soll prueft der eigene Block unten), der harte Fehlabbruch
		// hier waere sonst ein Eigentor.
		if(env_u("CFD_FERN_FACETTEN", 0u)==0u) {
			if((ulong)lbm_c.lbm_domain[0]->rho_clamp_hits[7]!=0ull) print_error("Fernfeld zaehlt Facetten-Wirkpfad -- es MUSS unberuehrt sein (CFD_FERN_FACETTEN=0).");
			else print_info("Negativ-Kontrolle: Fernfeld-Wirkpfad = 0 (unberuehrt, wie gefordert).");
		} else print_info("Negativ-Kontrolle Fernfeld UEBERSPRUNGEN (CFD_FERN_FACETTEN aktiv -- Ist=Soll-Pruefung im P8-Block unten).");
		df->fac_tau.read_from_device(); df->fac_tau_n.read_from_device(); // Stale-Falle: frisch lesen
		double dm=0.0, nk=0.0; for(ulong i=0ull;i<df->fac_N;i++){ dm+=(double)df->fac_tau[6ull*i+4ull]; nk+=(double)df->fac_tau[6ull*i+5ull]; }
		print_info("iMEM-Erhaltung Nahfeld: Delta-m = "+to_string((float)dm,6u)+", Normal-Rest = "+to_string((float)nk,6u)+" (KUMULATIV seit Start; CSV-Spalten dm/rest = Fenster-Delta ab Snapshot -- R2-Etikett)");
		{	// ★ Facetten-y+: je Facette y_w aus fac_geo (NICHT hartkodiert 0,5 -- Audit-Rest #6 nicht wiederholen)
			std::vector<float> yp; yp.reserve(df->fac_N);
			std::ofstream ycsv(out_dir+"yplus_facetten.csv"); ycsv << "yplus\n";
			for(ulong i=0ull;i<df->fac_N;i++) if(df->fac_tau_n[i]>0u) {
				const float tw=df->fac_tau[6ull*i]/(float)df->fac_tau_n[i];
				const float ypl=sqrt(fmax(0.0f,tw))*df->fac_geo[8ull*i+3ull]/nu_lat_f;
				yp.push_back(ypl); ycsv << ypl << "\n";
			}
			if(!yp.empty()) { std::sort(yp.begin(), yp.end());
				print_info("Facetten-y+ (Akkumulator, y_w je Facette): Median "+to_string(yp[yp.size()/2ull],1u)
					+", q25 "+to_string(yp[yp.size()/4ull],1u)+", q75 "+to_string(yp[(3ull*yp.size())/4ull],1u)
					+" ueber "+to_string((ulong)yp.size())+" Facetten (BB-Anker: 1122). CSV: yplus_facetten.csv"); }
			// ★ 04.09.2026 KONTAMINATIONSMASS des y+ (Befund aus dem Rueckfall-Bericht 03.09.).
			// fac_tau[6i] summiert tw ueber ALLE Besuche -- auch ueber Rueckfallbesuche, wo tw ein
			// "haette"-Wert ist, der nie aufgepraegt wurde. Am 8-mm-Fahrzeug sind das 42,8 %. Die Zeile
			// oben (und yplus_facetten.csv) erben das. Mit KDIAG-Slot [10]/[11] steht jetzt daneben das
			// y+ NUR ueber angewandte Besuche -- die Differenz ist die Kontamination, in einer Zahl.
			if(df->fac_kdiag_on&&df->fac_kd.length()>=12ull) { df->fac_kd.read_from_device();
				std::vector<float> ya; ya.reserve(df->fac_N); ulong n_ohne=0ull;
				std::ofstream acsv(out_dir+"yplus_facetten_angewandt.csv"); acsv << "yplus_angewandt\n";
				for(ulong i=0ull;i<df->fac_N;i++) {
					const float va=df->fac_kd[12ull*i+11ull];
					if(va<=0.0f) { n_ohne++; continue; } // Facette, die NIE angewandt hat -- im kontaminierten y+ steht sie trotzdem drin
					const float twa=df->fac_kd[12ull*i+10ull]/va;
					const float ypa=sqrt(fmax(0.0f,twa))*df->fac_geo[8ull*i+3ull]/nu_lat_f;
					ya.push_back(ypa); acsv << ypa << "\n";
				}
				acsv.close();
				if(!ya.empty()) { std::sort(ya.begin(), ya.end());
					const float mk=yp.empty()?0.0f:yp[yp.size()/2ull], ma=ya[ya.size()/2ull];
					print_info("Facetten-y+ NUR ueber ANGEWANDTE Besuche: Median "+to_string(ma,1u)
						+", q25 "+to_string(ya[ya.size()/4ull],1u)+", q75 "+to_string(ya[(3ull*ya.size())/4ull],1u)
						+" ueber "+to_string((ulong)ya.size())+" Facetten. CSV: yplus_facetten_angewandt.csv");
					print_info("  KONTAMINATION: "+to_string(n_ohne)+" von "+to_string(df->fac_N)+" Facetten ("
						+to_string((float)(100.0*(double)n_ohne/(double)(df->fac_N>0ull?df->fac_N:1ull)),2u)
						+" %) haben NIE angewandt und stehen trotzdem im kontaminierten y+; Median verschiebt sich "
						+to_string(mk,1u)+" -> "+to_string(ma,1u)+" ("+to_string((float)(mk>0.0f?100.0*((double)ma/(double)mk-1.0):0.0),1u)+" %).");
					print_info("  Fuer y+-gestuetzte Aussagen gilt ab sofort die ANGEWANDTE Reihe; die kontaminierte bleibt nur als Anschluss an aeltere Laeufe stehen.");
				} else print_warning("y+ angewandt: KEINE Facette hat je angewandt -- das waere ein Totalrueckfall und ist zu pruefen.");
			}
		}
		if(fac_pn>0ull) { const double qA=(double)q_inf*A_ref;
			print_info("Cd-Pfad Nahfeld: Cd_druck = "+to_string((float)((double)units_fine.si_F((float)(fac_px/(double)fac_pn))/qA),4u)
				+" (Zeitmittel, "+to_string(fac_pn)+" Samples), Cz_druck = "+to_string((float)((double)units_fine.si_F((float)(fac_pz/(double)fac_pn))/qA),4u)
				+" -- Reibung: letzte Zeile cd_facetten.csv. ACHTUNG Audit S5: cd_reib ist residuendominiert (88 % zielUNabhaengige Querresiduen der Rang-2-Pfade) -- ehrlicher Zielanteil = ARM-DIFFERENZ, nicht der Absolutwert."); }
		print_info("ACHTUNG: forces.csv/Cd oben enthaelt an behandelten Links PHANTOM-Reibung (object_force; gilt ebenso fuer kraft_zband.csv Fx/Fz_band/rest -- dieselbe Zerlegung) -- fuer A/B nur die VERSCHIEBUNG zwischen den Armen werten.");
	}
	if(env_u("CFD_FERN_FACETTEN", 0u)>0u) { // ★ P8: Wirkpfad-Nachweis FERNFELD (Muster Nahfeld; das Grobgitter laeuft n_outer Schritte, Ereignis-Slots t%100-gesampelt)
		LBM_Domain* dc = lbm_c.lbm_domain[0];
		dc->rho_clamp_hits.read_from_device();
		const ulong wzc=(ulong)dc->rho_clamp_hits[7], sollc=dc->fac_N*(ulong)(n_outer_ist/100ull + 1ull); // n_outer_IST (s. o.) // Pruefagent H1: das Grobgitter macht n_outer+1 Schritte (Vorlauf-run(1) vor der Schleife!) -- t%100 feuert bei t=0,100,..,<=n_outer = floor(n_outer/100)+1 mal; die ceil-Formel haette bei rundem n_outer falsch hart abgebrochen
		print_info("Facetten-Wirkpfad Fernfeld (P8): "+to_string(wzc)+" (Soll "+to_string(sollc)+" mod 2^32), tau-Klemme "+to_string((ulong)dc->rho_clamp_hits[8])
			+", u_t~0-Skips "+to_string((ulong)dc->rho_clamp_hits[9])
			+(env_u("CFD_FERN_FACETTEN",0u)>=3u?(", iMEM: u_s-Klemme/Gate "+to_string((ulong)dc->rho_clamp_hits[10])+", Skalar "+to_string((ulong)dc->rho_clamp_hits[12])
			+", LSQ-Rueckfall "+to_string((ulong)dc->rho_clamp_hits[65])+", ohneTang "+to_string((ulong)dc->rho_clamp_hits[13])+" (davon mit rohen Tangentialmomenten [27], NICHT ELIBB-heilbar -- Rang, s. B83: "+to_string((ulong)dc->rho_clamp_hits[27])+")"+", Rang2 "+to_string((ulong)dc->rho_clamp_hits[14])+", Rang0-BB "+to_string((ulong)dc->rho_clamp_hits[15])
			+", sn-Klemme/Gate "+to_string((ulong)dc->rho_clamp_hits[16])+", alpha>ut "+to_string((ulong)dc->rho_clamp_hits[18])+" (PEMA/APG im Fernfeld AUS)"):string("")));
		if(LBM_Domain::s_fac_elibb_pur) { if(wzc!=0ull) print_error("Pur-Arm Fernfeld: Slot 7 muesste 0 sein."); else print_info("Pur-Arm (Fernfeld): Slot 7 = 0 (konstruktiv), ELIBB-Wirkpfad "+to_string((ulong)dc->rho_clamp_hits[67])+"."); }
		else if(wzc!=(sollc&0xFFFFFFFFull)) print_error("Facetten-Wirkpfad Ist != Soll im Fernfeld -- Lookup oder Bindung defekt (P8).");
		if(env_u("CFD_FERN_FACETTEN",0u)>=3u) { // iMEM-Erhaltung analog Nahfeld (Delta-m/Normal-Rest aus dem kumulativen Akkumulator)
			dc->fac_tau.read_from_device(); dc->fac_tau_n.read_from_device();
			double dmc=0.0, nkc=0.0; for(ulong i=0ull;i<dc->fac_N;i++){ dmc+=(double)dc->fac_tau[6ull*i+4ull]; nkc+=(double)dc->fac_tau[6ull*i+5ull]; }
			print_info("iMEM-Erhaltung Fernfeld: Delta-m = "+to_string((float)dmc,6u)+", Normal-Rest = "+to_string((float)nkc,6u)+" (KUMULATIV seit Start)");
		}
		print_info("ACHTUNG P8: Fx_far (forces.csv) und der Fernfeld-Fahrzeugkraft-Anker oben sind in diesem Arm PHANTOMBEHAFTET (object_force an facettenbehandelten Links); kraft_facetten bleibt Nahfeld-only -- fuer A/B nur die VERSCHIEBUNG werten.");
	}
	print_info("---------------------------------------------------------------");
	_exit(0);
}

// =============================================================================================
// FERNFELD ALLEIN -- Diagnosefall
//
// WOZU. Der Doppel-Domaenen-Lauf dd_lauf01 kippte am 2026-08-08 bei 0,15 s, und die Schnitte
// zeigten ein Fernfeld, das vom Einlass her klingelt: bei 51 ms eine schmale oszillierende Saeule
// bei x = 0, bei 151 ms horizontale Streifen ueber die ganze Domaene. Solange Fahrzeug UND
// Kopplung mitlaufen, laesst sich nicht sagen, ob das grobe Gitter von selbst ringt oder erst
// durch eines von beidem. Dieser Fall trennt das: dasselbe Gitter, dasselbe tau, dieselben
// Raender -- aber leer und ungekoppelt. Ringt es hier schon, ist die Ursache eindeutig Rand
// plus Viskositaet, und nichts anderes.
//
// Er laeuft auf dem SCHNELLSTEN Geraet, nicht auf der iGPU: die Physik ist dieselbe, aber ein
// Schritt kostet 0,044 statt 0,343 s. Eine Diagnose, die acht Mal so lange braucht wie noetig,
// wird nicht oft genug wiederholt.
//
// Schalter: CFD_FERN_NU (Faktor auf nu, Default 1 = wie im gekoppelten Fall)
//           CFD_FERN_VEH=1 (Fahrzeug doch voxelisieren -- trennt "leer" von "mit Koerper")
// =============================================================================================
static void main_setup_fernfeld() {
	const float si_u = 30.0f, si_rho = 1.225f, si_length = 4.4364f, u_lat = 0.075f;
	const float si_nu = env_f("CFD_NU", 1.51e-5f);
	const uint  ratio = max(2u, env_u("CFD_RATIO", 4u));
	const float dx_f  = 0.001f*fmax(0.1f, env_f("CFD_DX", 4.0f));
	const float dx    = dx_f*(float)ratio;                 // Zellweite des Fernfelds
	const float dt    = u_lat*dx/si_u;
	const float nu_faktor = env_f("CFD_FERN_NU", 1.0f);    // Weg 2 aus EINLASS-AUSLASS.md
	const float nu_lat = si_nu*dt/(dx*dx)*nu_faktor;
	const float tau    = 3.0f*nu_lat + 0.5f;

	auto n_cells = [](const float len, const float d) { return (uint)floor(len/d + 0.5f) + 1u; };
	const float far_Lx = env_f("CFD_FAR_LX", 12.2720f), far_Ly = env_f("CFD_FAR_LY", 7.6640f), far_Lz = env_f("CFD_FAR_LZ", 8.8160f);
	const float far_x0 = env_f("CFD_FAR_X0", -0.6f*si_length);
	const uint Nx = n_cells(far_Lx, dx), Ny = n_cells(far_Ly, dx), Nz = n_cells(far_Lz, dx);
	const float far_y0 = -0.5f*(float)(Ny-1u)*dx;

	units.set_m_kg_s(si_length/dx, u_lat, 1.0f, si_length, si_u, si_rho);
	print_info("=================== Fernfeld allein (Diagnose) ===================");
	print_info("Gitter "+to_string(Nx)+" x "+to_string(Ny)+" x "+to_string(Nz)+" @ "+to_string(dx*1000.0f,2u)+" mm = "
		+to_string((float)((ulong)Nx*Ny*Nz)/1e6f,1u)+" M Zellen, identisch zum gekoppelten Fernfeld");
	print_info("tau = "+to_string(tau,7u)+" (nu-Faktor "+to_string(nu_faktor,2u)+"), Lambda = (tau-0.5)^2 = "+to_string((tau-0.5f)*(tau-0.5f),12u));
	print_info("Randbedingungen wie im gekoppelten Fall: z- mitbewegte Fahrbahn, x- / y+- / z+ Gleichgewicht, x+ Druckauslass.");

	const bool mit_fahrzeug = env_on("CFD_FERN_VEH");
	if(env_on("CFD_SPARSE_TILES")) print_warning("CFD_SPARSE_TILES wird im fernfeld-Fall NICHT angewandt."); // R2-Inventar
	// ★ Audit-Nacharbeit 5: die Kontaktflaechen-Uebergabe (Fz-Explosionsmechanismus, im dd-Fall
	// gefixt) ist HIER nicht nachgezogen -- der Diagnosearm mit Fahrzeug kann sich selbst
	// kontaminieren. Bis zum Nachzug wird das angesagt statt still gerechnet.
	if(mit_fahrzeug) print_warning("CFD_FERN_VEH=1: Kontaktflaechen-Uebergabe ist im fernfeld-Fall NICHT implementiert (Audit-Befund 5) -- Fz-Werte dieses Diagnosearms nicht belastbar.");
	Mesh* veh = nullptr;
	if(mit_fahrzeug) {
		veh = read_stl(get_exe_path()+"../scenes/vehicle.stl");
		const float3 bb0 = veh->get_bounding_box_size();
		veh->scale((si_length/dx)/bb0.x);
		const float3 bb = veh->get_bounding_box_size(), ctr = veh->get_bounding_box_center();
		veh->translate(float3((0.0f-far_x0)/dx + 0.5f*bb.x - ctr.x, (0.0f-far_y0)/dx - ctr.y, 0.5f*bb.z - ctr.z));
	}
	// ★ Daempfungszone -- im Diagnosefall das eine Gitter. Gemessen 2026-08-09: N=64 traegt bis
	// 0,20 s, u_max saettigt bei 0,69 gegen 2,6 des Kontrollarms. Grenze nach oben: N <= 120,
	// darueber kaeme die Zone den Kopplungs-Entnahmeebenen des dd-Falls zu nahe (x- hat 152 Zellen).
	LBM_Domain::s_sponge_n = env_u("CFD_SPONGE_N", 0u);
	LBM_Domain::s_sponge_a = env_f("CFD_SPONGE_A", 3000.0f);
	LBM_Domain::s_sponge_wmin = env_f("CFD_SPONGE_WMIN", 0.5f);
	// ★ Wandfunktion: BEWUSST nur im Kanal verdrahtet. Am Fahrzeug traefe die z-Wand-Logik die
	// MITBEWEGTE Fahrbahn (u_t wird absolut genommen -- an einer bewegten Wand falsch) und die
	// Karosserie braucht die Facetten (C1b). Bis dahin: ueberall sonst hart aus.
	LBM_Domain::s_wandfunktion = false; LBM_Domain::s_wf_tau = 1.0f; LBM_Domain::s_fac_budget = 1.0f; LBM_Domain::s_fac_budget_sn = 1.0f; LBM_Domain::s_schale_paritaet = false; LBM_Domain::s_facetten = false; LBM_Domain::s_fac_lsq = env_u("CFD_FAC_LSQ", 0u)>0u; LBM_Domain::s_fac_quergate = env_u("CFD_FAC_QUERGATE", 0u)>0u; LBM_Domain::s_fac_elibb = env_u("CFD_FAC_ELIBB", 0u)>0u; LBM_Domain::s_fac_elibb_pur = env_u("CFD_FAC_ELIBB", 0u)==2u; LBM_Domain::s_fac_qmin = env_f("CFD_FAC_QMIN", 0.1f); LBM_Domain::s_fac_kappa = env_f("CFD_FAC_KAPPA", 0.4f); LBM_Domain::s_fac_utkorr = env_f("CFD_FAC_UTKORR", 1.0f); LBM_Domain::s_fac_qkappe = env_f("CFD_FAC_QKAPPE", 1.0f); LBM_Domain::s_fac_qdiag = env_u("CFD_FAC_QDIAG", 0u); LBM_Domain::s_sgs_guo = env_u("CFD_SGS_GUO", 1u)>0u;; LBM_Domain::s_fac_imem = false; LBM_Domain::s_fac_ema = 0.0f; LBM_Domain::s_fac_pema = 0.0f; LBM_Domain::s_fac_satgate = false; LBM_Domain::s_fac_kraft = 0u; LBM_Domain::s_fac_kdiag = 0u; LBM_Domain::s_fac_nachbar = 0u; LBM_Domain::s_fac_messnur = 0u; LBM_Domain::s_sgs_fdwand = 0u; LBM_Domain::s_sgs_gdiag = 0u; LBM_Domain::s_fac_alpha = 0u; LBM_Domain::s_fac_apg = 0.0f; LBM_Domain::s_boden_eq_n = 0u; LBM_Domain::s_boden_eq_down = 0u; LBM_Domain::s_boden_eq_split = 0xFFFFFFFFu; LBM_Domain::s_boden_eq_abstand = 0u; LBM_Domain::s_einlass_eq_n = 0u; LBM_Domain::s_schale_alpha = 0.0f; LBM_Domain::s_fac_diagz = -1l; LBM_Domain::s_fac_tau = 1.0f; // Statik-Symmetrie VOLL (IR3-Abschluss-Loop)
	if(env_u("CFD_WANDFUNKTION", 0u)>0u) print_warning("CFD_WANDFUNKTION wird in diesem Fall NICHT angewandt (nur kanal).");
	{ const char* n2f_[] = {"CFD_N2F_SCHALE","CFD_N2F_VOLUMEN","CFD_N2F_BAND","CFD_N2F_BAND_N","CFD_N2F_BAND_PROFIL","CFD_N2F_BAND_UNTERBODEN","CFD_N2F_BAND_WAKE","CFD_N2F_BAND_NURWAKE","CFD_N2F_BAND_WAKE_START","CFD_N2F_BAND_WAKE_START_X","CFD_N2F_BAND_WAKE_ABSTAND","CFD_N2F_PARITAET"}; for(const char* b : n2f_) if(getenv(b)) print_warning(string(b)+" ist gesetzt, wird aber NUR im fahrzeug_dd-Fall angewandt (P9c; die neun BAND-/WAKE-/PARITAET-Schalter fehlten bis 2026-08-22 in dieser Ansage -- Pruefagent-S1)."); } // Ansage-Doktrin
	if(env_u("CFD_FACETTEN", 0u)>0u) print_warning("CFD_FACETTEN wird im fernfeld-Fall NICHT angewandt (Audit R3: die 6. Stelle hatte die Ansage schon wieder ausgelassen).");
	if(env_u("CFD_FERN_FACETTEN", 0u)>0u) print_warning("CFD_FERN_FACETTEN wird im fernfeld-Fall NICHT angewandt (nur fahrzeug_dd -- P8; Ansage-Doktrin).");
	if(getenv("CFD_SLICE_NEAR_STEPS")||getenv("CFD_VTK_JEDE")||getenv("CFD_VTK_BEHALTE")) print_warning("CFD_SLICE_NEAR_STEPS/CFD_VTK_JEDE/CFD_VTK_BEHALTE werden in diesem Fall NICHT angewandt (nur fahrzeug_dd; Kadenz-Umbau 27.08.).");
	if(env_u("CFD_FACETTEN_DIAG", 0u)>0u) print_warning("CFD_FACETTEN_DIAG wird im fernfeld-Fall NICHT angewandt.");
	// ★ Nachpruefer-Befund 2026-08-15: diese sechste Konstruktorstelle FEHLTE in der Verdrahtung von
	// CFD_SGS_WANDFREI -- der Schalter waere im fernfeld-Fall still wirkungslos gewesen (die
	// Commit-Behauptung "alle 5 Aufrufstellen" hatte schlicht falsch gezaehlt: es sind sechs).
	LBM_Domain::s_sgs_wandfrei = env_u("CFD_SGS_WANDFREI", 0u)>0u; LBM_Domain::s_sgs_guo = env_u("CFD_SGS_GUO", 1u)>0u; LBM_Domain::s_sgs_diag = env_u("CFD_SGS_DIAG", 0u)>0u; LBM_Domain::s_sgs_diag_ab = (ulong)env_u("CFD_SGS_DIAG_AB", 0u);
	// ★ Nachpruefer-Befund 2026-08-09: die Obergrenze stand hier nur im Kommentar, geprueft wurde sie
	// nur im dd-Fall. Damit lief CFD_SPONGE_N=400 im Diagnosefall ungeprueft durch.
	if(LBM_Domain::s_sponge_n>120u) print_error("CFD_SPONGE_N ueber 120 ist nicht vorgesehen (im dd-Fall kaeme die Zone der Kopplungs-Entnahmeebene x- bei 152 Zellen zu nahe; der Diagnosefall bleibt vergleichbar).");
	// ★ EINLASS_EQ (V1-Port apply_inlet_velocity): im Diagnosefall ist dies das EINZIGE Gitter --
	// der Schalter wirkt hier direkt am TYPE_E-Einlass x- (derselbe Name wie im dd-Fall, dort nur Fernfeld).
	LBM_Domain::s_einlass_eq_n = env_u("CFD_FERN_EINLASS_EQ", 0u); LBM_Domain::s_einlass_eq_u = u_lat;
	if(LBM_Domain::s_einlass_eq_n>3u) print_warning("CFD_FERN_EINLASS_EQ > 3 verletzt die V1-Vorgabe (V1: 3) -- die Clamp-Schicht frisst sich in die Domaene.");
	if(LBM_Domain::s_einlass_eq_n>0u) print_info("EINLASS_EQ aktiv (V1-Port apply_inlet_velocity): x=1.."+to_string(LBM_Domain::s_einlass_eq_n)+" post-stream auf u_inf-Equilibrium (rho lokal); x=0 bleibt TYPE_E.");
	LBM lbm(uint3(Nx, Ny, Nz), nu_lat);
	if(mit_fahrzeug) {
		lbm.voxelize_mesh_on_device(veh, TYPE_S|TYPE_X); lbm.flags.read_from_device();
		sat_shell_and_void_fill(lbm, veh, Nx, Ny, Nz);
	} else lbm.flags.read_from_device();

	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
		const ulong n = (ulong)x + ((ulong)y + (ulong)z*(ulong)Ny)*(ulong)Nx;
		if((lbm.flags[n]&TYPE_S)!=0u) continue;
		if(z==0u) { lbm.flags[n] = TYPE_S; lbm.u.x[n] = u_lat; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f; }
		else if(x==0u || x==Nx-1u || y==0u || y==Ny-1u || z==Nz-1u) { lbm.flags[n] = TYPE_E; lbm.u.x[n] = u_lat; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f; }
		else { lbm.u.x[n] = u_lat; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f; }
	}
	if(getenv("CFD_PO_FACES")) print_warning("CFD_PO_FACES wird im fernfeld-Fall NICHT angewandt (Maske hart x_max -- Gross-Audit).");
	lbm.set_pressure_outlet_faces(2u, env_f("CFD_PO_RHO", 1.0f));
	// ★★ GEMESSEN UND VERWORFEN, 2026-08-08. Der Geschwindigkeits-Einlass (u vorgeschrieben, rho aus
	// der Innenzelle) sollte die Ueberbestimmung des Randes beheben. Er tut es NICHT -- A/B im leeren
	// Kanal, beide Arme mit demselben Binary, der Kontrollarm reproduzierte den Vorlauf bitgenau:
	//   bei 0,08 s: Streuung 0,0695 (aus) gegen 0,0712 (an), ueber 10 % daneben 10,94 gegen 12,90 %.
	//   Und der Massenstrom sackt ab: u im Mittel faellt monoton auf 0,9696 und weiter.
	// LEHRE: meine Erklaerung war zur Haelfte falsch. Das Problem ist nicht, DASS rho und u beide
	// vorgegeben sind, sondern dass der Gleichgewichts-Reset den Nichtgleichgewichtsanteil verwirft.
	// Die Dichte schweben zu lassen beseitigt den Reset nicht -- es nimmt dem Einlass nur den
	// Druckanker, und die Domaene entleert sich langsam. DEFAULT DESHALB AUS; CFD_FERN_VI=1 fuer
	// eigene Versuche. Der Kernel bleibt, weil er korrekt ist und fuer andere Faelle taugen kann.
	const bool vi_an = env_on("CFD_FERN_VI");
	if(vi_an) lbm.set_velocity_inlet_faces(45u);
	else print_info("Geschwindigkeits-Einlass AUS (CFD_FERN_VI=0): rho bleibt am Einlass festgenagelt, alter Zustand.");

	const float t_flush = (float)(Nx-1u)*dx/si_u;
	const float t_end   = env_f("CFD_T_END", 0.5f*t_flush);
	const ulong n_steps = (ulong)(t_end/dt + 0.5f);
	const uint  sample_every = max(1u, env_u("CFD_SAMPLE_EVERY", 100u));
	const float slice_dt = env_f("CFD_SLICE_DT", 0.010f);
	const string out_dir = get_exe_path()+"../export/"+(getenv("CFD_RUN_NAME")?string(getenv("CFD_RUN_NAME")):string("fernfeld"))+"/";
	create_folder(out_dir);
	{ const char* bs_[] = {"CFD_BODEN_EQ","CFD_BODEN_EQ_DOWN","CFD_BODEN_EQ_ABSTAND","CFD_FERN_BODEN_EQ","CFD_FERN_BODEN_EQ_DOWN","CFD_FERN_BODENKLEMME","CFD_KOPPLUNG_BODENBAND","CFD_KOPPLUNG_ZEITINTERP"}; for(const char* b : bs_) if(getenv(b)) print_warning(string(b)+" ist gesetzt, wird im fernfeld-Fall aber NICHT angewandt (fahrzeug_dd; CFD_BODEN_EQ auch kugel -- XL-3 B8)."); }
	sichere_lauf(out_dir, "fernfeld"); // ★ Heiko 2026-08-09: Sicherung MIT JEDEM Lauf, in allen vier Faellen
	print_info("Laufzeit "+to_string(t_end,4u)+" s = "+to_string(n_steps)+" Schritte (eine Durchspuelung = "+to_string(t_flush,4u)+" s)");

	// RAUSCHMASS. Nicht das Auge entscheidet, sondern eine Zahl: die Standardabweichung von u_x
	// ueber alle freien Fluidzellen, auf u_inf normiert. In einem ungestoerten Kanal ohne Koerper
	// gehoert sie nahe null zu bleiben. Dazu die groesste Abweichung und der Anteil der Zellen,
	// die um mehr als 10 % danebenliegen -- der trennt "ein paar Ausreisser" von "global verrauscht".
	std::ofstream csv(out_dir+"rauschen.csv"); csv.precision(8);
	csv << "time_s,u_mittel_rel,u_streuung_rel,u_max_rel,anteil_ueber_10prozent\n" << std::flush;
	float slice_next = 0.0f;
	lbm.run(0u);
	// ★ Vorpruefer-Befund 2026-08-09: auch der Fernfeld-Diagnosefall hat eine mitbewegte Fahrbahn
	// (z = 0, unbedingt, unabhaengig von CFD_FERN_VEH) -- er gehoert also mitgeprueft. Genau in
	// diesem Fall wurde die Daempfungszone vermessen.
	audit_bewegte_waende(lbm, Nx, Ny, Nz, dx, u_lat, "Fernfeld allein", false);
	for(ulong step=0ull; step<n_steps; step+=(ulong)sample_every) {
		const ulong chunk = min((ulong)sample_every, n_steps-step);
		lbm.run(chunk, n_steps);
		// ★ EINMALIG nach dem ersten Rechen-Abschnitt: der Nachweis DURCH den Kernel. Nur er faengt
		// V1s Fehlerklasse (alle Flags korrekt, aber der Konsument sprang heraus) -- siehe die
		// Begruendung bei pruefe_wandwirksamkeit().
		if(step==0ull) pruefe_wandwirksamkeit(lbm, Nx, Ny, Nz, u_lat, "Fernfeld allein");
		lbm.u.read_from_device(); lbm.flags.read_from_device();
		double s1=0.0, s2=0.0; ulong nf=0ull, nbad=0ull; float umax=0.0f;
		for(uint z=1u; z<Nz-1u; z++) for(uint y=1u; y<Ny-1u; y++) for(uint x=1u; x<Nx-1u; x++) {
			const ulong n = (ulong)x + ((ulong)y + (ulong)z*(ulong)Ny)*(ulong)Nx;
			if((lbm.flags[n]&(TYPE_S|TYPE_E))!=0u) continue; // nur freie Fluidzellen
			const float ux = lbm.u.x[n]; const float r = ux/u_lat;
			s1 += (double)r; s2 += (double)r*(double)r; nf++;
			if(fabs(r-1.0f)>0.10f) nbad++;
			umax = fmax(umax, fabs(r-1.0f));
		}
		const double mu = nf? s1/(double)nf : 0.0;
		const double sd = nf? sqrt(fmax(0.0, s2/(double)nf - mu*mu)) : 0.0;
		const double t_si = (double)((float)(step+chunk)*dt);
		csv << t_si << "," << mu << "," << sd << "," << umax << "," << (nf? (double)nbad/(double)nf : 0.0) << "\n" << std::flush;
		print_info("[RAUSCHEN] t = "+to_string((float)t_si,4u)+" s: u_x im Mittel "+to_string((float)mu,4u)+" von u_inf, Streuung "
			+to_string((float)sd,5u)+", groesste Abweichung "+to_string(100.0f*umax,1u)+" %, ueber 10 % daneben: "
			+to_string((float)(100.0*(nf? (double)nbad/(double)nf : 0.0)),2u)+" % der Zellen");
		if(slice_dt>0.0f && (float)t_si>=slice_next) {
			slice_next = (float)t_si + slice_dt;
			render_yslice(lbm, Nx, Ny, Nz, Ny/2u, si_u/u_lat, si_u, (int)((float)t_si*1000.0f+0.5f), out_dir, "fern");
		}
		if(!std::isfinite(sd) || sd>1.0) { print_error("Fernfeld allein ist auseinandergelaufen (Streuung "+to_string((float)sd,4u)+" von u_inf) bei t = "+to_string((float)t_si,4u)+" s."); }
	}
	{	// ★ Einlass-Saeule (Heiko 2026-08-21): 20 cm hinter dem Einlass, y-Mitte, alle z (wie dd-Fall)
		lbm.u.read_from_device();
		const uint ex=(uint)(0.2f/dx+0.5f), ey=Ny/2u; std::ofstream ep(out_dir+"einlass_saeule.csv"); ep.precision(6);
		ep << "# x = Einlass+0.2 m (Zelle " << ex << ", dx " << dx*1000.0f << " mm), y-Mitte" << std::endl << "z_m,ux_rel,uy_rel,uz_rel" << std::endl;
		double flips=0.0, dmax=0.0; float vor=0.0f, letzte_d=0.0f; bool hat_vor=false;
		for(uint z=0u; z<Nz; z++) { const ulong n=(ulong)ex+((ulong)ey+(ulong)z*(ulong)Ny)*(ulong)Nx;
			const float ux=lbm.u.x[n]/u_lat, uy=lbm.u.y[n]/u_lat, uz=lbm.u.z[n]/u_lat;
			ep << (double)z*(double)dx << "," << ux << "," << uy << "," << uz << std::endl;
			if(hat_vor) { const float d=ux-vor; dmax=fmax(dmax,(double)fabs(d)); if(z>=2u&&((d>0.0f)!=(letzte_d>0.0f))&&fabs(d)>1e-3f&&fabs(letzte_d)>1e-3f) flips+=1.0; letzte_d=d; }
			vor=ux; hat_vor=true; }
		print_info("Einlass-Saeule (x=+0,2 m, y-Mitte): Vorzeichenwechsel dux/dz = "+to_string((float)flips,0u)+" von "+to_string(Nz-2u)+", max|dux| = "+to_string((float)dmax,4u)+" u_inf. CSV: einlass_saeule.csv");
	}
	// Audit-Nacharbeit 7: auch der fernfeld-Diagnosefall meldet die Dichte-Klemme.
	if(env_u("CFD_FERN_EINLASS_EQ",0u)>0u) { // ★ EINLASS_EQ-Wirkpfad-Nachweis (Iron Rule 3, Slot 21; Muster BODEN_EQ)
		lbm.lbm_domain[0]->rho_clamp_hits.read_from_device();
		const ulong eq=(ulong)lbm.lbm_domain[0]->rho_clamp_hits[21];
		print_info("EINLASS_EQ-Wirkpfad: "+to_string(eq)+" Spalten-Resets (t%100-Stichprobe).");
		if(eq==0ull) print_error("CFD_FERN_EINLASS_EQ gesetzt, aber Wirkpfad NULL -- lautloser No-Op.");
	}
	{ ulong h=0ull; berichte_dichteklemme(lbm, "Fernfeld-Diagnose", h); dichteklemme_fazit(h); }
	print_info("CSV: "+out_dir+"rauschen.csv");
	_exit(0);
}


// ---------------------------------------------------------------------------- C1b Stufe 2: T1+T2
// T1: leitet die Paartabelle UNABHAENGIG aus FZ_C ab und prueft sie gegen die Host-Kopie der im
// Kernel hartkodierten Tabelle (faengt Transkriptionsfehler mechanisch); danach Gate-Logik an
// synthetischen Geometrien. T2: Mini-Domaene mit 45-Grad-Treppe, Arm AUS gegen Arm FACETTEN=2
// (reiner Tausch) nach GENAU 1 Schritt -- Differenzen duerfen NUR an Zellen mit >=1 offenem Paar
// liegen, alle anderen muessen BITGLEICH sein (R1: DDF-Integritaet an fluiden Urspruengen).
static uint fz_opp(const uint i) { return (i==0u) ? 0u : ((i%2u==1u) ? i+1u : i-1u); }
struct FacPaar { uint ip, im, g1, g2; }; // Tausch fhn[ip]<->fhn[im], Gates j[g1], j[g2]
// Host-Kopie der Kernel-Tabelle (apply_facette): [achse][seite 0=+n,1=-n][paar]
static const FacPaar FAC_TAB[3][2][2] = {
	{ {{7,13,8,14},{9,15,10,16}}, {{14,8,13,7},{16,10,15,9}} },   // achse 0: nx>0 (-x-Wand), nx<0 (+x)
	{ {{7,14,8,13},{11,17,12,18}}, {{13,8,14,7},{18,12,17,11}} }, // achse 1: ny>0, ny<0
	{ {{9,16,10,15},{11,18,12,17}}, {{15,10,16,9},{17,12,18,11}} } // achse 2: nz>0 (Boden), nz<0 (Decke)
};
void main_setup_facetten_test() {
	if(getenv("CFD_BODEN_EQ")||getenv("CFD_FERN_BODEN_EQ")||getenv("CFD_FERN_EINLASS_EQ")) print_warning("Die BODEN_EQ/EINLASS_EQ-Familie wird in facetten_test NICHT angewandt (XL-R3).");
	if(getenv("CFD_KOPPLUNG_ZEITINTERP")) print_warning("CFD_KOPPLUNG_ZEITINTERP wird in diesem Fall NICHT angewandt (nur fahrzeug_dd; Pruefagent M3).");
	if(getenv("CFD_FERN_FACETTEN")) print_warning("CFD_FERN_FACETTEN wird in facetten_test NICHT angewandt (nur fahrzeug_dd; P8-M1).");
	{ const char* n2f_[] = {"CFD_N2F_SCHALE","CFD_N2F_VOLUMEN","CFD_N2F_BAND","CFD_N2F_BAND_N","CFD_N2F_BAND_PROFIL","CFD_N2F_BAND_UNTERBODEN","CFD_N2F_BAND_WAKE","CFD_N2F_BAND_NURWAKE","CFD_N2F_BAND_WAKE_START","CFD_N2F_BAND_WAKE_START_X","CFD_N2F_BAND_WAKE_ABSTAND","CFD_N2F_PARITAET"}; for(const char* b : n2f_) if(getenv(b)) print_warning(string(b)+" ist gesetzt, wird aber NUR im fahrzeug_dd-Fall angewandt (P9c; die neun BAND-/WAKE-/PARITAET-Schalter fehlten bis 2026-08-22 in dieser Ansage -- Pruefagent-S1)."); } // Ansage-Doktrin
	print_info("facetten_test: ALLE CFD_FACETTEN*/CFD_FAC_*-Env-Werte werden IGNORIERT (Arme hart verdrahtet); kein sichere_lauf, fester Ordner export/facetten_test (Gross-Audit-Ansage).");
	if(env_u("CFD_SGS_WANDFREI",0u)>0u||env_u("CFD_SPONGE_N",0u)>0u) print_warning("CFD_SGS_WANDFREI/CFD_SPONGE_N/CFD_SPARSE_TILES/CFD_WANDFUNKTION sind im facetten_test WIRKUNGSLOS (B10).");
	print_info("C1b T1: Paartabelle unabhaengig aus FZ_C herleiten und gegen die Kernel-Kopie pruefen.");
	uint fehler=0u;
	for(uint a=0u; a<3u; a++) for(uint s=0u; s<2u; s++) {
		const int sign = s==0u ? 1 : -1; // Normale zeigt in +a bzw. -a
		// einlaufende Diagonalen: c[a] == sign und genau eine weitere Komponente != 0
		uint gefunden=0u;
		for(uint ta=0u; ta<3u; ta++) { // tangentiale Achsen aufsteigend (Paarreihenfolge!)
			if(ta==a) continue;
			for(uint i=7u; i<19u; i++) {
				if(FZ_C[i][a]!=sign) continue;
				int nc=0; for(uint kx=0u;kx<3u;kx++) if(FZ_C[i][kx]!=0) nc++;
				if(nc!=2 || FZ_C[i][ta]<=0) continue; // Mitglied mit POSITIVER Tangentialkomponente = ip
				// Partner: gleiches c[a], gespiegelte Tangentiale
				uint im=0u;
				for(uint p2=7u; p2<19u; p2++) if(FZ_C[p2][a]==sign && FZ_C[p2][ta]==-FZ_C[i][ta] && FZ_C[p2][3u-a-ta]==0) im=p2;
				const FacPaar soll = FAC_TAB[a][s][gefunden];
				const uint g1=fz_opp(i), g2=fz_opp(im);
				if(soll.ip!=i||soll.im!=im||soll.g1!=g1||soll.g2!=g2) {
					print_warning("T1: Tabellenfehler achse "+to_string(a)+" seite "+to_string(s)+" paar "+to_string(gefunden)
						+": hergeleitet ("+to_string(i)+","+to_string(im)+","+to_string(g1)+","+to_string(g2)
						+") vs Kernel ("+to_string(soll.ip)+","+to_string(soll.im)+","+to_string(soll.g1)+","+to_string(soll.g2)+")");
					fehler++;
				}
				gefunden++;
			}
		}
		if(gefunden!=2u) { print_warning("T1: achse "+to_string(a)+" seite "+to_string(s)+": "+to_string(gefunden)+" statt 2 Paare hergeleitet."); fehler++; }
	}
	if(fehler>0u) print_error("T1 GESCHEITERT: "+to_string(fehler)+" Tabellenfehler.");
	print_info("T1 bestanden: alle 12 Paare (6 Seiten x 2) stimmen mit der Kernel-Tabelle ueberein.");
	// T1b: Gate-Logik an der 45-Grad-Treppe (solid wenn z<y, Zelle auf z==y): y-Paar zu, x-Paar offen.
	{
		auto solid = [](const int x, const int y, const int z) { (void)x; return z<y; };
		const int x=8,y=8,z=8; uint offen=0u, zu=0u;
		for(uint p=0u; p<2u; p++) { // Bodenseite (nz>0), achse 2
			const FacPaar& q = FAC_TAB[2][0][p];
			const bool g = solid(x+FZ_C[q.g1][0],y+FZ_C[q.g1][1],z+FZ_C[q.g1][2]) && solid(x+FZ_C[q.g2][0],y+FZ_C[q.g2][1],z+FZ_C[q.g2][2]);
			if(g) offen++; else zu++;
		}
		if(offen!=1u||zu!=1u) print_error("T1b GESCHEITERT: 45-Grad-Eckzelle muss genau 1 offenes (x) und 1 geschlossenes (y) Paar haben, hat "+to_string(offen)+"/"+to_string(zu)+".");
		print_info("T1b bestanden: Eckzelle der 45-Grad-Treppe -- x-Paar offen, y-Paar zu (R1-Beispiel des Gegenpruefers).");
	}
	// ---- T2: Mini-Domaene, Arm AUS vs Arm FACETTEN=2, 1 Schritt, Differenz-Lokalisierung.
	print_info("C1b T2: 32x16x24, 45-Grad-Treppenboden, AUS vs NUR-TAUSCH nach 1 Schritt.");
	const uint Nx=32u, Ny=16u, Nz=24u; const float nu_lat=0.01f;
	auto treppe = [&](const uint x, const uint y, const uint z) { (void)y;
		if(env_u("CFD_T2_FLACH",0u)>0u) return z==0u; // DIAGNOSE: Kanal-Geometrie im Mini-Format
		return (int)z < (int)((x%8u)); }; // Periode 8 teilt Nx=32
	auto init = [&](LBM& L) {
		for(ulong n=0ull; n<L.get_N(); n++) {
			uint x,y,z; L.coordinates(n,x,y,z);
			if(z==Nz-1u||treppe(x,y,z)) {
				L.flags[n]=TYPE_S; L.u.x[n]=0.0f;
				// ★ LEHRSTUECK (kostete eine Stunde Bisektion): der erste Fix gab den Wandzellen
				// u!=0, um die Paarsymmetrie zu brechen -- initialize() machte damit ALLE
				// Fluidnachbarn zu TYPE_MS, und genau die schliesst das Zellgate aus (MS-Guard,
				// Audit-Befund 3). Der Facettenpfad lief keinen einzigen Schritt. Wandzellen
				// bleiben u=0; die Sichtbarkeit des Tauschs liefert der ZWEITE Schritt (Phase 1
				// ist bitgleich, Schritt 1 schreibt asymmetrische Kollisionswerte in die Slots).
			} else L.u.x[n]=0.05f; // tangentiale Anstroemung, damit u_t != 0
		}
	};
	// ★ ZWEI Schritte, mit Begruendung: nach Aequilibrium-Init tragen die von Solidzellen
	// stammenden Slots SYMMETRISCHE Werte (initialize baut Solid-Equilibrium mit u=0, beide
	// Paarpartner sind wertgleich) -- der Tausch von Schritt 1 ist unsichtbar, die Arme sind nach
	// Schritt 1 bitgleich (wird als Phase 1 MITGEPRUEFT). Erst Schritt 2 liest die asymmetrischen
	// Kollisionswerte aus Schritt 1 -- der Tausch wird sichtbar und bleibt exakt zelllokal,
	// WEIL die Felder bis dahin identisch waren.
	std::vector<float> ua_x, ua_y, ua_z, ua1_x; std::vector<Facette> FF;
	{ // Arm A: AUS
		LBM_Domain::s_facetten=false; LBM_Domain::s_fac_imem=false; LBM_Domain::s_fac_ema=0.0f; LBM_Domain::s_fac_pema=0.0f; LBM_Domain::s_fac_satgate=false; LBM_Domain::s_fac_kraft = 0u; LBM_Domain::s_fac_kdiag = 0u; LBM_Domain::s_fac_nachbar = 0u; LBM_Domain::s_fac_messnur = 0u; LBM_Domain::s_sgs_fdwand = 0u; LBM_Domain::s_sgs_gdiag = 0u; LBM_Domain::s_fac_alpha=0u; LBM_Domain::s_fac_apg=0.0f; LBM_Domain::s_boden_eq_n=0u; LBM_Domain::s_boden_eq_down=0u; LBM_Domain::s_boden_eq_split=0xFFFFFFFFu; LBM_Domain::s_boden_eq_abstand=0u; LBM_Domain::s_einlass_eq_n=0u; LBM_Domain::s_schale_alpha=0.0f; LBM_Domain::s_fac_diagz=-1l; LBM_Domain::s_fac_tau=1.0f;
		LBM a(Nx,Ny,Nz,nu_lat); init(a); a.run(1u,2u); a.u.read_from_device();
		ua1_x.resize(3ull*a.get_N()); for(ulong n=0ull; n<a.get_N(); n++) { ua1_x[n]=a.u.x[n]; ua1_x[n+a.get_N()]=a.u.y[n]; ua1_x[n+2ull*a.get_N()]=a.u.z[n]; } // Nachpruefer: alle DREI Komponenten
		a.run(1u,2u); a.u.read_from_device();
		ua_x.resize(a.get_N()); ua_y.resize(a.get_N()); ua_z.resize(a.get_N());
		for(ulong n=0ull; n<a.get_N(); n++) { ua_x[n]=a.u.x[n]; ua_y[n]=a.u.y[n]; ua_z[n]=a.u.z[n]; }
	}
	ulong diff_erlaubt=0ull, diff_verboten=0ull, gleich_erwartet=0ull;
	{ // Arm B: FACETTEN=2 (reiner Tausch)
		LBM_Domain::s_facetten=true; LBM_Domain::s_fac_imem=false; LBM_Domain::s_fac_ema=0.0f; LBM_Domain::s_fac_pema=0.0f; LBM_Domain::s_fac_satgate=false; LBM_Domain::s_fac_kraft = 0u; LBM_Domain::s_fac_kdiag = 0u; LBM_Domain::s_fac_nachbar = 0u; LBM_Domain::s_fac_messnur = 0u; LBM_Domain::s_sgs_fdwand = 0u; LBM_Domain::s_sgs_gdiag = 0u; LBM_Domain::s_fac_alpha=0u; LBM_Domain::s_fac_apg=0.0f; LBM_Domain::s_boden_eq_n=0u; LBM_Domain::s_boden_eq_down=0u; LBM_Domain::s_boden_eq_split=0xFFFFFFFFu; LBM_Domain::s_boden_eq_abstand=0u; LBM_Domain::s_einlass_eq_n=0u; LBM_Domain::s_schale_alpha=0.0f; LBM_Domain::s_fac_diagz=-1l; LBM_Domain::s_fac_tau=0.0f;
		LBM b(Nx,Ny,Nz,nu_lat); init(b);
		const string t2_dir = get_exe_path()+"../export/facetten_test/"; create_folder(t2_dir);
		FF = baue_facetten(b, Nx, Ny, Nz, TYPE_S, t2_dir, "T2-Treppe");
		b.alloc_facetten(FF);
		b.run(1u,2u); b.u.read_from_device();
		{ ulong d1=0ull; for(ulong n=0ull; n<b.get_N(); n++) if(ua1_x[n]!=b.u.x[n]||ua1_x[n+b.get_N()]!=b.u.y[n]||ua1_x[n+2ull*b.get_N()]!=b.u.z[n]) d1++;
		  print_info("T2 Phase 1 (nach 1 Schritt): "+to_string(d1)+" u-Differenzen in allen 3 Komponenten (Erwartung 0 -- Tausch wertgleich wegen Solid-Symmetrie)");
		  if(d1>0ull) print_warning("Phase 1 nicht bitgleich -- Lokalisierung von Phase 2 gilt nur eingeschraenkt."); }
		b.run(1u,2u); b.u.read_from_device();
		b.lbm_domain[0]->rho_clamp_hits.read_from_device();
		b.lbm_domain[0]->fac_tau_n.read_from_device();
		ulong swsum=0ull; for(ulong i=0ull;i<b.lbm_domain[0]->fac_N;i++) swsum+=(ulong)b.lbm_domain[0]->fac_tau_n[i];
		print_info("T2-Slots: Wirkpfad[7]="+to_string((ulong)b.lbm_domain[0]->rho_clamp_hits[7])
			+" ohnePaar[11]="+to_string((ulong)b.lbm_domain[0]->rho_clamp_hits[11])
			+" Skips[9]="+to_string((ulong)b.lbm_domain[0]->rho_clamp_hits[9])
			+" Tauschzellen-Beitraege="+to_string(swsum)+" (fac_N="+to_string(b.lbm_domain[0]->fac_N)+")");
		// Host-Vorhersage: fuer jede AKTIVE Facette Gates auswerten
		std::vector<uchar> erwartet(b.get_N(), 0u); // 1 = Differenz erlaubt (>=1 Paar offen)
		for(const Facette& f : FF) {
			if(f.klasse!=0u) continue;
			uint x,y,z; b.coordinates(f.n,x,y,z);
			const float na[3]={f.nx,f.ny,f.nz}; const uint a2=f.achse; const uint seite = na[a2]>0.0f ? 0u : 1u;
			uint offen=0u;
			for(uint p=0u; p<2u; p++) {
				const FacPaar& q = FAC_TAB[a2][seite][p];
				auto sol=[&](const uint gi){ const int xx=(int)x+FZ_C[gi][0], yy=(int)y+FZ_C[gi][1], zz=(int)z+FZ_C[gi][2];
					const uint wxx=(uint)((xx%(int)Nx+(int)Nx)%(int)Nx), wyy=(uint)((yy%(int)Ny+(int)Ny)%(int)Ny);
					if(zz<0||zz>=(int)Nz) return false; return (b.flags[(ulong)wxx+((ulong)wyy+(ulong)zz*(ulong)Ny)*(ulong)Nx]&(TYPE_S|TYPE_E))==TYPE_S; };
				if(sol(q.g1)&&sol(q.g2)) offen++;
			}
			if(offen>0u) erwartet[f.n]=1u;
		}
		for(ulong n=0ull; n<b.get_N(); n++) {
			const bool d = (ua_x[n]!=b.u.x[n])||(ua_y[n]!=b.u.y[n])||(ua_z[n]!=b.u.z[n]);
			if(d) { if(erwartet[n]) diff_erlaubt++; else diff_verboten++; }
			else if(erwartet[n]) gleich_erwartet++;
		}
	}
	print_info("T2: Differenzen an erlaubten Zellen "+to_string(diff_erlaubt)+", an VERBOTENEN "+to_string(diff_verboten)
		+", erwartete ohne Differenz "+to_string(gleich_erwartet)+" (u_t~0 oder Tausch symmetrisch: legitim)");
	if(diff_verboten>0ull) print_error("T2 GESCHEITERT: "+to_string(diff_verboten)+" Zellen ohne offenes Paar haben sich veraendert -- der Tausch zerstoert gestreamte DDFs (R1-Verletzung).");
	if(diff_erlaubt==0ull) print_error("T2 GESCHEITERT: keine einzige erlaubte Differenz -- der Facettenpfad ist ein lautloser No-Op.");
	print_info("T2 bestanden: Tausch wirkt genau an den Zellen mit offenen Paaren, alle anderen bitgleich.");

	// ================================================================ iMEM-Tests (FACETTEN-IMEM.md I0)
	// T3a: Host-Referenz der Momente/Kaskade in double an synthetischen Linkmengen.
	{
		static const double WH[19] = {1.0/3.0, 1.0/18,1.0/18,1.0/18,1.0/18,1.0/18,1.0/18,
			1.0/36,1.0/36,1.0/36,1.0/36,1.0/36,1.0/36,1.0/36,1.0/36,1.0/36,1.0/36,1.0/36,1.0/36};
		auto momente = [&](const std::vector<uint>& L, const double nh[3], const double uu[3],
		                   double& G11, double& G22, double& G12, double S1[3], double& ut, double t1[3]) {
			const double und = nh[0]*uu[0]+nh[1]*uu[1]+nh[2]*uu[2];
			const double utv[3] = {uu[0]-und*nh[0], uu[1]-und*nh[1], uu[2]-und*nh[2]};
			ut = sqrt(utv[0]*utv[0]+utv[1]*utv[1]+utv[2]*utv[2]);
			for(int d2=0;d2<3;d2++) t1[d2] = ut>0.0 ? utv[d2]/ut : 0.0;
			const double t2[3] = {nh[1]*t1[2]-nh[2]*t1[1], nh[2]*t1[0]-nh[0]*t1[2], nh[0]*t1[1]-nh[1]*t1[0]};
			G11=G22=G12=0.0; S1[0]=S1[1]=S1[2]=0.0;
			for(const uint i : L) {
				const double cx=FZ_C[i][0], cy=FZ_C[i][1], cz=FZ_C[i][2], wi=WH[i];
				const double ct1=cx*t1[0]+cy*t1[1]+cz*t1[2], ct2=cx*t2[0]+cy*t2[1]+cz*t2[2];
				G11+=6.0*wi*ct1*ct1; G22+=6.0*wi*ct2*ct2; G12+=6.0*wi*ct1*ct2;
				S1[0]+=wi*cx; S1[1]+=wi*cy; S1[2]+=wi*cz;
			}
		};
		uint f3=0u;
		// (1) ebene Wand: G-Momente + Free-Slip-Identitaet s1 = rho*u_x
		{
			const std::vector<uint> L={5,9,16,11,18}; const double nh[3]={0,0,1}, uu[3]={0.05,0,0};
			double G11,G22,G12,S1[3],ut,t1[3]; momente(L,nh,uu,G11,G22,G12,S1,ut,t1);
			if(fabs(G11-1.0/3.0)>1e-15||fabs(G22-1.0/3.0)>1e-15||fabs(G12)>1e-15) { print_warning("T3a(1): G-Momente ebene Wand falsch."); f3++; }
			// Phi^f aus feq (geshiftet; Shift hebt sich, Symmetrie): P1 = 2*sum ct1*(feq[opp(i)]-w)
			double P1=0.0;
			for(const uint i : L) { const uint o=fz_opp(i);
				const double cu=FZ_C[o][0]*uu[0]+FZ_C[o][1]*uu[1]+FZ_C[o][2]*uu[2];
				const double feq=WH[o]*(3.0*cu+4.5*cu*cu-1.5*(uu[0]*uu[0])); // rho=1, geshiftet (feq-w)
				const double ct1=FZ_C[i][0]*t1[0]+FZ_C[i][1]*t1[1]+FZ_C[i][2]*t1[2];
				P1+=2.0*ct1*feq; }
			const double s1 = (0.0-P1)/G11; // Nullziel (Free-Slip-Grenzfall, Gl. 11)
			if(fabs(P1-(-uu[0]/3.0))>1e-12) { print_warning("T3a(1): Phi^f != -rho*u_x/3 (ist "+to_string((float)P1,9u)+")."); f3++; }
			if(fabs(s1-uu[0])>1e-12) { print_warning("T3a(1): Free-Slip s1 != u_x (ist "+to_string((float)s1,9u)+")."); f3++; }
		}
		// (2) 45-Grad-Lage-0: S1 = (0,-5,+5)/36 parallel zu +n
		{
			const std::vector<uint> L={4,5,18,8,13,16,9}; const double nh[3]={0,-1.0/sqrt(2.0),1.0/sqrt(2.0)}, uu[3]={0.05,0,0};
			double G11,G22,G12,S1[3],ut,t1[3]; momente(L,nh,uu,G11,G22,G12,S1,ut,t1);
			if(fabs(S1[0])>1e-15||fabs(S1[1]+5.0/36.0)>1e-15||fabs(S1[2]-5.0/36.0)>1e-15) { print_warning("T3a(2): S1 != (0,-5,+5)/36."); f3++; }
			// Delta-m-Leck fuer tangentiales u_s: S1 || n => S1*u_s = 0 fuer JEDES tangentiale u_s
			const double us[3]={t1[0]*0.1, t1[1]*0.1, t1[2]*0.1};
			if(fabs(S1[0]*us[0]+S1[1]*us[1]+S1[2]*us[2])>1e-12) { print_warning("T3a(2): Delta-m-Leck an 45-Grad-Lage-0 nicht 0."); f3++; }
		}
		// (3) Einzellink diagonal -> Skalar-Fallback; (4) Einzellink normal -> kein Tangential-Link
		{
			const double nh[3]={0,0,1}, uu[3]={0.05,0,0}; double G11,G22,G12,S1[3],ut,t1[3];
			momente({9},nh,uu,G11,G22,G12,S1,ut,t1);
			const double det=G11*G22-G12*G12;
			if(!(det<1e-4*G11*G22||G22<1e-8)||G11<1e-8) { print_warning("T3a(3): Einzellink diagonal landet nicht im Skalar-Fallback."); f3++; }
			momente({5},nh,uu,G11,G22,G12,S1,ut,t1);
			if(G11>=1e-8) { print_warning("T3a(4): Normal-Einzellink hat G11 != 0."); f3++; }
			// (5) Spiegelfall {11} (Nachpruefer-Befund 2): reiner Quer-Link -- G11==0, G22>0.
			// Muss im Quer-Skalar-Zweig landen (frueher: det=0-Division im 2x2!).
			momente({11},nh,uu,G11,G22,G12,S1,ut,t1);
			const double det5=G11*G22-G12*G12;
			const bool b1 = det5>=1e-4*G11*G22&&G11>=1e-8&&G22>=1e-8, b2 = G11>=1e-8;
			if(b1||b2||G22<1e-8) { print_warning("T3a(5): Quer-Einzellink {11} landet nicht im Quer-Skalar-Zweig."); f3++; }
		}
		if(f3>0u) print_error("T3a GESCHEITERT: "+to_string(f3)+" Referenzpruefungen.");
		print_info("T3a bestanden: Momente, Free-Slip-Identitaet, S1-Vorzeichen, Kaskadenpfade (double-Referenz).");
	}
	// T2-iMEM: Arm 3 (VOLLES Ziel) vs AUS nach GENAU 1 Schritt. Arm 4 (Nullziel) ist im ersten
	// Schritt wertneutral (Wand-Slots tragen geshiftete Nullen -> P1=0 -> s=0 -- dieselbe
	// Schritt-1-Unsichtbarkeit wie der Paartausch, am gefixten def_fac_tau gemessen); erst das
	// volle Ziel -twe modifiziert sofort, und die 1-Schritt-Lokalisierung (Auflage 3) bleibt exakt.
	{
		ulong di_erlaubt=0ull, di_verboten=0ull, di_still=0ull;
		LBM_Domain::s_facetten=true; LBM_Domain::s_fac_imem=true; LBM_Domain::s_fac_ema=0.0f; LBM_Domain::s_fac_pema=0.0f; LBM_Domain::s_fac_satgate=false; LBM_Domain::s_fac_kraft = 0u; LBM_Domain::s_fac_kdiag = 0u; LBM_Domain::s_fac_nachbar = 0u; LBM_Domain::s_fac_messnur = 0u; LBM_Domain::s_sgs_fdwand = 0u; LBM_Domain::s_sgs_gdiag = 0u; LBM_Domain::s_fac_alpha=0u; LBM_Domain::s_fac_apg=0.0f; LBM_Domain::s_boden_eq_n=0u; LBM_Domain::s_boden_eq_down=0u; LBM_Domain::s_boden_eq_split=0xFFFFFFFFu; LBM_Domain::s_boden_eq_abstand=0u; LBM_Domain::s_einlass_eq_n=0u; LBM_Domain::s_schale_alpha=0.0f; LBM_Domain::s_fac_diagz=-1l; LBM_Domain::s_fac_tau=1.0f;
		LBM d(Nx,Ny,Nz,nu_lat); init(d);
		const string t2i_dir = get_exe_path()+"../export/facetten_test/t2imem/"; create_folder(t2i_dir); // Audit 2/3: nicht den T2-Treppen-Census ueberschreiben
		std::vector<Facette> FD = baue_facetten(d, Nx, Ny, Nz, TYPE_S, t2i_dir, "T2-iMEM");
		d.alloc_facetten(FD);
		d.run(1u,1u); d.u.read_from_device();
		d.lbm_domain[0]->rho_clamp_hits.read_from_device();
		// Host-Vorhersage: Modifikation <=> klasse==0 && ut>=1e-6 && tangential wirksame Linkmenge
		std::vector<uchar> mod(d.get_N(), 0u);
		{
			static const double WH2[19] = {1.0/3.0, 1.0/18,1.0/18,1.0/18,1.0/18,1.0/18,1.0/18,
				1.0/36,1.0/36,1.0/36,1.0/36,1.0/36,1.0/36,1.0/36,1.0/36,1.0/36,1.0/36,1.0/36,1.0/36};
			for(const Facette& f : FD) {
				if(f.klasse!=0u) continue;
				uint x,y,z; d.coordinates(f.n,x,y,z);
				const double nh[3]={f.nx,f.ny,f.nz}, uu[3]={0.05,0,0};
				const double und=nh[0]*uu[0]; const double utv[3]={uu[0]-und*nh[0],-und*nh[1],-und*nh[2]};
				const double ut=sqrt(utv[0]*utv[0]+utv[1]*utv[1]+utv[2]*utv[2]);
				if(ut<1e-6) continue;
				const double t1[3]={utv[0]/ut,utv[1]/ut,utv[2]/ut};
				const double t2[3]={nh[1]*t1[2]-nh[2]*t1[1], nh[2]*t1[0]-nh[0]*t1[2], nh[0]*t1[1]-nh[1]*t1[0]};
				double G11=0.0,G22=0.0;
				for(uint i=1u;i<19u;i++){ const uint ib=fz_opp(i);
					const int xn=(int)x+FZ_C[ib][0], yn=(int)y+FZ_C[ib][1], zn=(int)z+FZ_C[ib][2];
					const uint wxx=(uint)((xn%(int)Nx+(int)Nx)%(int)Nx), wyy=(uint)((yn%(int)Ny+(int)Ny)%(int)Ny);
					if(zn<0||zn>=(int)Nz) continue;
					if((d.flags[(ulong)wxx+((ulong)wyy+(ulong)zn*(ulong)Ny)*(ulong)Nx]&(TYPE_S|TYPE_E))!=TYPE_S) continue;
					const double ct1=FZ_C[i][0]*t1[0]+FZ_C[i][1]*t1[1]+FZ_C[i][2]*t1[2];
					const double ct2=FZ_C[i][0]*t2[0]+FZ_C[i][1]*t2[1]+FZ_C[i][2]*t2[2];
					G11+=6.0*WH2[i]*ct1*ct1; G22+=6.0*WH2[i]*ct2*ct2; }
				if(G11>=1e-8||G22>=1e-8) mod[f.n]=1u;
			}
		}
		for(ulong n2=0ull; n2<d.get_N(); n2++) {
			const bool df = (ua1_x[n2]!=d.u.x[n2])||(ua1_x[n2+d.get_N()]!=d.u.y[n2])||(ua1_x[n2+2ull*d.get_N()]!=d.u.z[n2]);
			if(df) { if(mod[n2]) di_erlaubt++; else di_verboten++; }
			else if(mod[n2]) di_still++;
		}
		print_info("T2-iMEM (1 Schritt, Arm 3): Differenzen erlaubt "+to_string(di_erlaubt)+", VERBOTEN "+to_string(di_verboten)
			+", vorhergesagt ohne Differenz "+to_string(di_still)+"; Wirkpfad[7]="+to_string((ulong)d.lbm_domain[0]->rho_clamp_hits[7])
			+" Skalar[12]="+to_string((ulong)d.lbm_domain[0]->rho_clamp_hits[12])+" ohneTangential[13]="+to_string((ulong)d.lbm_domain[0]->rho_clamp_hits[13]));
		if(di_verboten>0ull) print_error("T2-iMEM GESCHEITERT: "+to_string(di_verboten)+" Zellen ohne vorhergesagte Modifikation haben sich veraendert.");
		if(di_erlaubt==0ull) print_error("T2-iMEM GESCHEITERT: keine einzige erlaubte Differenz -- iMEM-Pfad lautloser No-Op.");
		print_info("T2-iMEM bestanden: iMEM wirkt genau an den vorhergesagten Zellen (1-Schritt-Lokalisierung).");
	}
	_exit(0);
}

void main_setup() { // Fallauswahl: CFD_CASE = kugel (Default) | kanal | fahrzeug | fahrzeug_dd | fernfeld | facetten_test
	const char* c = getenv("CFD_CASE");
	// ★ Hygiene E7b: hier fehlte das `else` -- das trug nur, weil fernfeld immer per _exit endet.
	// Kehrte es je normal zurueck, liefe zusaetzlich der Kugelfall (Default-Zweig).
	if(c!=nullptr && string(c)=="kanal") main_setup_kanal();
	else if(c!=nullptr && string(c)=="facetten_test") main_setup_facetten_test();
	else if(c!=nullptr && string(c)=="fernfeld") main_setup_fernfeld();
	else if(c!=nullptr && string(c)=="fahrzeug_dd") main_setup_fahrzeug_dd();
	else if(c!=nullptr && string(c)=="fahrzeug") main_setup_fahrzeug();
	else main_setup_kugel();
}
