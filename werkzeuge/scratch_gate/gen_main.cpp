// Statischer OpenCL-C-Generator: baut den Kernel-Quelltext exakt wie lbm.cpp
// (device_defines + get_opencl_c_code), aber ohne OpenCL/GPU — Defines fuer den
// g15-Kanal (62x30x22, D3Q19, FP16C, FACETTEN=3, SATGATE, ALPHA=2, ELIBB an/aus)
// aus export/g15_fix_an/code/LAUF.txt rekonstruiert. NUR LESEN am Repo.
#include <string>
#include <fstream>
#include <iostream>
#include <iterator>
using std::string;

string get_opencl_c_code(); // aus kernel.hpp via kernel.o
string positiv_defines(const unsigned modus, const unsigned haken, const unsigned facette, const bool fp16s, const unsigned long long N, const unsigned Nx, const unsigned Ny); // ★ 15.09.2026 Klemmen Stufe 1 P1a: kernel.o, dieselbe Quelle wie lbm.cpp

static string device_defines(const bool elibb, const bool ptrt, const bool rho16, const bool sparsam, const bool u16, const bool rand) {
	string s =
	"\n#define cl_workgroup_size 64u"
	"\n#ifdef cl_khr_fp64"
	"\n#pragma OPENCL EXTENSION cl_khr_fp64 : enable"
	"\n#endif"
	"\n#ifdef cl_khr_fp16"
	"\n#pragma OPENCL EXTENSION cl_khr_fp16 : enable"
	"\n#endif"
	"\n#ifdef cl_khr_int64_base_atomics"
	"\n#pragma OPENCL EXTENSION cl_khr_int64_base_atomics : enable"
	"\n#endif"
	"\n#define def_Nx 62u"
	"\n#define def_Ny 30u"
	"\n#define def_Nz 22u"
	"\n#define def_N 40920ul"
	"\n#define uxx uint"
	"\n#define def_GNx 62u"
	"\n#define def_GNy 30u"
	"\n#define def_GNz 22u"
	"\n#define def_Dx 1u"
	"\n#define def_Dy 1u"
	"\n#define def_Dz 1u"
	"\n#define def_Ox 0"
	"\n#define def_Oy 0"
	"\n#define def_Oz 0"
	"\n#define def_Ax 660u"
	"\n#define def_Ay 1364u"
	"\n#define def_Az 1860u"
	"\n#define def_domain_offset_x 0.0f"
	"\n#define def_domain_offset_y 0.0f"
	"\n#define def_domain_offset_z 0.0f"
	"\n#define D3Q19"
	"\n#define def_velocity_set 19u"
	"\n#define def_dimensions 3u"
	"\n#define def_transfers 5u"
	"\n#define def_c 0.57735027f"
	"\n#define def_w 1.99993038f"
	"\n#define def_w0 (1.0f/3.0f)"
	"\n#define def_ws (1.0f/18.0f)"
	"\n#define def_we (1.0f/36.0f)"
	"\n#define SRT"
	"\n#define TYPE_S 0x01"
	"\n#define TYPE_E 0x02"
	"\n#define TYPE_T 0x04"
	"\n#define TYPE_F 0x08"
	"\n#define TYPE_I 0x10"
	"\n#define TYPE_G 0x20"
	"\n#define TYPE_X 0x40"
	"\n#define TYPE_Y 0x80"
	"\n#define REG_E(i) (feq[i])"
	"\n#define RHO_CLAMP"
	"\n#define RHO_CLAMP_MIN 0.5000f"
	"\n#define RHO_CLAMP_MAX 1.5000f"
	"\n#define KLEMM_BILANZ" // ★ 15.09.2026 Klemmen S0b: Produktionsvorgabe CFD_KLEMM_BILANZ=1
	"\n#define def_klemm_s 16384.0f"
	"\n#define def_u2max (def_c*def_c)" // ★ Z2b (lbm.cpp-Emission unter KLEMM_BILANZ)
	"\n#define def_tor_lo (1.0f-1.5625f*0.500000f)"
	"\n#define def_tor_hi (1.0f+1.5625f*0.500000f)"
	"\n#define FACETTEN"
	"\n#define def_fac_Y 86206.89844f"
	"\n#define def_fac_utkorr 1.000000f"
	"\n#define def_fac_tau 1.0000f"
	"\n#define def_fac_chifak 0.99998259f" // MLS chi-Nenner 1/(tau0+0,5); hergeleitet aus def_w 1.99993038: tau0=1/w=0.50001741
	"\n#define def_fac_budget 1.0000f"
	"\n#define def_fac_budget_sn 1.0000f"
	// ★ 11.09.2026: def_zaehl_takt ist UNBEDINGT emittiert (lbm.cpp, zaehl_takt()) und steht in
	// 71 Kernel-Gattern. Fehlt er hier, scheitert der Bau -- das Gate hat genau das gemeldet,
	// und zwar als BAUFEHLER und nicht als Scratch. Diese Unterscheidung wurde heute frueh
	// eingebaut und hat sich damit zum ersten Mal bewaehrt.
	"\n#define def_zaehl_takt 100ul"
	"\n#define def_wf_spalding_it 3u"
	"\n#define FACETTEN_IMEM"
	"\n#define FACETTEN_SATGATE"
	"\n#define FACETTEN_ALPHA"
	"\n#define FACETTEN_ALPHA2";
	if(elibb) s += "\n#define FACETTEN_ELIBB";
	// ★ P-TRT-ARM 11.09.2026. Ohne ihn prueft das Gate den Produktionsstand NICHT: der
	// #ifdef PTRT-Block in kernel.cpp bleibt inert, solange PTRT nicht definiert ist.
	// omega_g = 1,90 ist der gemessene Produktionswert (konvergierte von-Neumann-Kurve).
	if(ptrt) s += "\n#define PTRT"
	                "\n#define def_omega_g 1.90000000f";
	s +=
	"\n#define TYPE_MS 0x03"
	"\n#define TYPE_BO 0x03"
	"\n#define TYPE_IF 0x18"
	"\n#define TYPE_IG 0x30"
	"\n#define TYPE_GI 0x38"
	"\n#define TYPE_SU 0x38"
	"\n#define TYPE_XY 0xC0"
	"\n#define fpxx ushort"
	"\n#define fpxx_copy ushort"
	"\n#define load(p,o) half_to_float_custom((p)[o])"
	"\n#define store(p,o,x) (p)[o]=float_to_half_custom(x)"
	// ★ 12.09.2026 ZWILLING zu lbm.cpp (TODO 2 Schritt 4). Fehlt das hier, baut der Kernel nicht
	// und das Gate meldet BAUFEHLER statt Scratch -- genau so ist es beim ersten Lauf passiert.
	; s += rho16
	? (string)"\n#define RHO_FP16"
	          "\n#define rhoxx half"
	          "\n#define load_rho(p,o) (vload_half(o,p)*3.0517578E-5f+1.0f)"
	          "\n#define load_drho(p,o) (vload_half(o,p)*3.0517578E-5f)"
	          "\n#define store_rho(p,o,x) vstore_half_rte(((x)-1.0f)*32768.0f,o,p)"
	: (string)"\n#define rhoxx float"
	          "\n#define load_rho(p,o) ((p)[o])"
	          "\n#define load_drho(p,o) ((p)[o]-1.0f)"
	          "\n#define store_rho(p,o,x) ((p)[o]=(x))";
	// ★ 12.09.2026: u-Zwilling (TODO 2 Schritt 4). Ohne ihn scheitert der BAU der .cl, sobald
	// kernel.cpp velxx/load_u/store_u benutzt -- und das Gate meldete das frueher als
	// "SCRATCH-GATE VERLETZT", also als etwas voellig anderes. Wortgleich zu lbm.cpp.
	s += u16
	? (string)"\n#define U_FP16"
	          "\n#define velxx half"
	          "\n#define load_u(p,o) (vload_half(o,p)*3.0517578E-5f)"
	          "\n#define store_u(p,o,x) vstore_half_rte((x)*32768.0f,o,p)"
	: (string)"\n#define velxx float"
	          "\n#define load_u(p,o) ((p)[o])"
	          "\n#define store_u(p,o,x) ((p)[o]=(x))";
	// ★ 12.09.2026 (Audit-Schleife, Pruefer B): der PRODUKTIONSSTAND nach TODO 2 ist SPARSAM, und
	// die SPARSAM-Zweige haengen genau dem registerkritischsten Kernel (stream_collide) je eine
	// coordinates()-Rechnung, zwei Bedingungsketten und eine Atomik an. Ohne diese Defines erklaerte
	// das Gate einen Kernel fuer scratch-frei, den es nie uebersetzt hat.
	// def_SM* sind die Schreibmasken-Box; die Werte sind die des Gate-Kanalfalls (62x30x22), also
	// eine Box, die die halbe Domaene deckt -- fuer die Registerfrage zaehlt der CODE, nicht die Lage.
	; s += sparsam
	? (string)"\n#define RHO_SPARSAM"
	          "\n#define U_SPARSAM"
	          "\n#define RHO_SMBOX"
	          "\n#define def_SMX0 8u"  "\n#define def_SMY0 4u"  "\n#define def_SMZ0 4u"
	          "\n#define def_SMNX 46u" "\n#define def_SMNY 22u" "\n#define def_SMNZ 14u"
	: (string)"";
	// ★ 15.09.2026 RHO_RAND C2b (RHO_RAND-C2-PLAN.md §5/§6): der PRODUKTIONSPUNKT unter RHO_RAND -- RHO_RAND ersetzt
	// RHO_SPARSAM/RHO_SMBOX im Nahfeld, U_SPARSAM bleibt. def_RR_N = Randschale des Gate-Kanals 62x30x22:
	// 40920 - 58*26*18 = 13776. Nie zusammen mit sparsam (die Aufrufe im Gate halten das ein).
	s += rand
	? (string)"\n#define RHO_RAND"
	          "\n#define def_RR_N 13776ul"
	          "\n#define U_SPARSAM"
	          "\n#define def_SMX0 8u"  "\n#define def_SMY0 4u"  "\n#define def_SMZ0 4u"
	          "\n#define def_SMNX 46u" "\n#define def_SMNY 22u" "\n#define def_SMNZ 14u"
	: (string)"";
	s +=
	"\n#define UPDATE_FIELDS"
	"\n#define VOLUME_FORCE"
	"\n#define MOVING_BOUNDARIES"
	"\n#define EQUILIBRIUM_BOUNDARIES"
	"\n#define FORCE_FIELD"
	"\n#define F_NUR_SOLID" // Gate-Paritaet: Produktions-Default seit F-Null-Read-Gate (26.08.)
	"\n#define SUBGRID"
	"\n#define def_FBX0 0u"
	"\n#define def_FBY0 0u"
	"\n#define def_FBZ0 0u"
	"\n#define def_FBNX 62u"
	"\n#define def_FBNY 30u"
	"\n#define def_FBNZ 22u"
	"\n#define def_FBN 40920ul"
	"\n#define TS_P"
	// ★ NACHGEZOGEN 11.09.2026: diese drei fehlten und liessen den BAU scheitern -- das Gate war
	// damit seit dem 09./10.09. funktionslos. Es meldete den Baufehler ausserdem als
	// "SCRATCH-GATE VERLETZT", also als etwas voellig anderes. Beides behoben.
	//   def_fac_isogate / def_fac_deteps: lbm.cpp:1591-1592, unbedingt emittiert, Default 0.
	//   F_STRIDE: lbm.cpp:1718-1719. Hier die F_LISTE-Fassung, weil die Produktion seit dem
	//   09.09. mit CFD_F_LISTE=1 faehrt -- sie traegt einen zusaetzlichen Speicherlesezugriff
	//   und ist damit die schaerfere Variante fuer ein Register-Gate.
	"\n#define def_fac_isogate 0.0000f"
	"\n#define def_fac_deteps 0.0000f"
	"\n#define F_STRIDE ((ulong)f_maske[2ul*((def_FBN+31ul)/32ul)])"
	"\n#define TS_A";
	return s;
}

int main(int argc, char** argv) {
	// ★ 15.09.2026 Klemmen S0a -- PRODUKTIONSPARITAET: statt handgepflegter Kanal-Defines die ECHTEN Defines eines
	// Laufs. Quelle: CFD_DUMP_CL=1 (lbm.cpp) schreibt device_defines+Kernel nach /tmp/fx3d_kernel_dump_N.cl; der Vorspann
	// VOR get_opencl_c_code() wird als defs-Datei abgelegt (werkzeuge/scratch_gate/defs_*.txt). Anlass: gen_main kannte
	// weder SGS_FDWAND/SISM noch SPONGE -- der Buchungsort der Klemmen-Stufe-0 laege im Produktionsbau ungeprueft.
	//   gen nurcode <ausgabe>            -> nur get_opencl_c_code() (zum Abschneiden des Vorspanns aus einem Dump)
	//   gen datei <defs.txt> <ausgabe.cl> -> defs + aktueller Kernel
	//   gen datei <defs.txt> <ausgabe.cl> pos<modus>[f][h<haken>] -> zusaetzlich positiv_defines() (Klemmen Stufe 1, z. B. pos2fh1)
	if(argc>=3&&string(argv[1])=="nurcode") { std::ofstream f(argv[2]); f << get_opencl_c_code(); return 0; }
	if(argc>=4&&(string(argv[1])=="datei"||string(argv[1])=="dateih3")) { // dateih3: zusaetzlich KLEMM_HAKEN3 (Gate-Arm fuer den Negativhaken)
		std::ifstream d(argv[2]); if(!d) { std::cerr << "gen: defs-Datei fehlt: " << argv[2] << "\n"; return 2; }
		const string defs((std::istreambuf_iterator<char>(d)), std::istreambuf_iterator<char>());
		// Geraete-Vorspann wie opencl.hpp enable_device_capabilities() (ohne die geraeteabhaengigen Patches) -- der
		// CFD_DUMP_CL-Dump enthaelt ihn NICHT, weil Device() ihn erst beim Bauen voranstellt.
		const string geraet = "\n #define cl_workgroup_size 64u\n #ifdef cl_khr_fp64\n #pragma OPENCL EXTENSION cl_khr_fp64 : enable\n #endif"
			"\n #ifdef cl_khr_fp16\n #pragma OPENCL EXTENSION cl_khr_fp16 : enable\n #endif"
			"\n #ifdef cl_khr_int64_base_atomics\n #pragma OPENCL EXTENSION cl_khr_int64_base_atomics : enable\n #endif";
		// Die Dumps vom 15.09. (b9329a5) sind VOR Klemmen-S0b entstanden -- KLEMM_BILANZ (Produktionsvorgabe) wird angehaengt.
		const string klemm = (defs.find("#define KLEMM_BILANZ")==string::npos ? string("\n #define KLEMM_BILANZ\n #define def_klemm_s 16384.0f") : string(""))
			+(defs.find("#define def_u2max")==string::npos ? string("\n #define def_u2max (def_c*def_c)\n #define def_tor_lo (1.0f-1.5625f*0.500000f)\n #define def_tor_hi (1.0f+1.5625f*0.500000f)") : string("")) // ★ Z2b: Schnappschuesse vor Z2b tragen die Huellen-Defines nicht
			+(string(argv[1])=="dateih3" ? string("\n #define KLEMM_HAKEN3") : string(""));
		string pos = "";
		if(argc>=5) { // ★ 15.09.2026 Klemmen Stufe 1 P1a: Positiv-Arme ueber die EMISSIONSFUNKTION selbst, keine Zwillingsliste
			string a = argv[4];
			// ★ Z2d: optionales Endzeichen 'u' = zusaetzlich U_BETRAG (CFD_U_KLEMME=1); "u" allein = nur U_BETRAG
			bool ub = false; if(!a.empty()&&a.back()=='u') { ub = true; a.pop_back(); }
			if(a.empty()) { pos = "\n #define U_BETRAG"; goto schreiben; }
			// Pruefbefund P1a NIEDRIG 4: exakt pos<1|2>[f][h<1..3>], sonst Abbruch -- keine stille Umdeutung (pos1h, pos12, pos1x)
			size_t q = 4; const bool fmt = a.size()>=4&&a.substr(0, 3)=="pos"&&(a[3]=='1'||a[3]=='2');
			bool fac = false; unsigned haken = 0u; bool ok = fmt;
			if(ok&&q<a.size()&&a[q]=='f') { fac = true; q++; }
			if(ok&&q<a.size()&&a[q]=='h') { if(q+1<a.size()&&a[q+1]>='1'&&a[q+1]<='3') { haken = (unsigned)(a[q+1]-'0'); q += 2; } else ok = false; }
			if(ok&&q!=a.size()) ok = false;
			if(!ok) { std::cerr << "gen: 4. Argument exakt pos<1|2>[f][h<1..3>][u] oder u: " << argv[4] << "\n"; return 2; }
			unsigned long long N_defs = 1ull; unsigned Nx_defs = 1u, Ny_defs = 1u;
			{ size_t pn = defs.find("#define def_N "); if(pn!=string::npos) N_defs = std::stoull(defs.substr(pn+14));
			  pn = defs.find("#define def_Nx "); if(pn!=string::npos) Nx_defs = (unsigned)std::stoul(defs.substr(pn+15));
			  pn = defs.find("#define def_Ny "); if(pn!=string::npos) Ny_defs = (unsigned)std::stoul(defs.substr(pn+15)); }
			pos = positiv_defines((unsigned)(a[3]-'0'), haken, fac ? 1u : 0u, defs.find("#define fpxx half")!=string::npos, N_defs, Nx_defs, Ny_defs); // N, Nx, Ny aus den defs der defs (Stichprobenperiode wie im Lauf)
			if(ub) pos += "\n #define U_BETRAG";
		}
		schreiben:
		const string code = geraet + defs + klemm + pos + get_opencl_c_code();
		std::ofstream f(argv[3]); f << code; f.close();
		std::cout << "geschrieben: " << argv[3] << " (" << code.size() << " Bytes, Defines aus " << argv[2] << ")\n";
		return 0;
	}
	// Aufruf: gen <elibb:on|off> <ptrt:on|off> <rho16:on|off> <sparsam:on|off> <u16:on|off> <rand:on|off> <ausgabe.cl>
	if(argc<8) { std::cerr << "Aufruf: gen <elibb:on|off> <ptrt:on|off> <rho16:on|off> <sparsam:on|off> <u16:on|off> <rand:on|off> <ausgabe.cl>\n"; return 2; }
	const bool elibb   = string(argv[1])=="on";
	const bool ptrt    = string(argv[2])=="on";
	const bool rho16   = string(argv[3])=="on";
	const bool sparsam = string(argv[4])=="on";
	const bool u16     = string(argv[5])=="on";
	const bool rand    = string(argv[6])=="on";
	if(rand&&sparsam) { std::cerr << "gen: rand und sparsam schliessen sich aus (RHO_RAND ersetzt RHO_SPARSAM)\n"; return 2; }
	const string out = argv[7];
	const string code = device_defines(elibb, ptrt, rho16, sparsam, u16, rand) + get_opencl_c_code();
	std::ofstream f(out);
	f << code;
	f.close();
	std::cout << "geschrieben: " << out << " (" << code.size() << " Bytes, ELIBB="
	          << (elibb?"an":"aus") << ", PTRT=" << (ptrt?"an":"aus") << ", RHO16=" << (rho16?"an":"aus") << ", SPARSAM=" << (sparsam?"an":"aus") << ", U16=" << (u16?"an":"aus") << ", RAND=" << (rand?"an":"aus") << ")\n";
	return 0;
}
