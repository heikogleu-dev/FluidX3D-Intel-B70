// Statischer OpenCL-C-Generator: baut den Kernel-Quelltext exakt wie lbm.cpp
// (device_defines + get_opencl_c_code), aber ohne OpenCL/GPU — Defines fuer den
// g15-Kanal (62x30x22, D3Q19, FP16C, FACETTEN=3, SATGATE, ALPHA=2, ELIBB an/aus)
// aus export/g15_fix_an/code/LAUF.txt rekonstruiert. NUR LESEN am Repo.
#include <string>
#include <fstream>
#include <iostream>
using std::string;

string get_opencl_c_code(); // aus kernel.hpp via kernel.o

static string device_defines(const bool elibb, const bool ptrt, const bool rho16) {
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
	// Aufruf: gen <elibb:on|off> <ptrt:on|off> <rho16:on|off> <ausgabe.cl>
	if(argc<5) { std::cerr << "Aufruf: gen <elibb:on|off> <ptrt:on|off> <rho16:on|off> <ausgabe.cl>\n"; return 2; }
	const bool elibb = string(argv[1])=="on";
	const bool ptrt  = string(argv[2])=="on";
	const bool rho16 = string(argv[3])=="on";
	const string out = argv[4];
	const string code = device_defines(elibb, ptrt, rho16) + get_opencl_c_code();
	std::ofstream f(out);
	f << code;
	f.close();
	std::cout << "geschrieben: " << out << " (" << code.size() << " Bytes, ELIBB="
	          << (elibb?"an":"aus") << ", PTRT=" << (ptrt?"an":"aus") << ", RHO16=" << (rho16?"an":"aus") << ")\n";
	return 0;
}
