// Statischer OpenCL-C-Generator: baut den Kernel-Quelltext exakt wie lbm.cpp
// (device_defines + get_opencl_c_code), aber ohne OpenCL/GPU — Defines fuer den
// g15-Kanal (62x30x22, D3Q19, FP16C, FACETTEN=3, SATGATE, ALPHA=2, ELIBB an/aus)
// aus export/g15_fix_an/code/LAUF.txt rekonstruiert. NUR LESEN am Repo.
#include <string>
#include <fstream>
#include <iostream>
using std::string;

string get_opencl_c_code(); // aus kernel.hpp via kernel.o

static string device_defines(const bool elibb) {
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
	"\n#define def_fac_budget 1.0000f"
	"\n#define def_fac_budget_sn 1.0000f"
	"\n#define def_wf_spalding_it 3u"
	"\n#define FACETTEN_IMEM"
	"\n#define FACETTEN_SATGATE"
	"\n#define FACETTEN_ALPHA"
	"\n#define FACETTEN_ALPHA2";
	if(elibb) s += "\n#define FACETTEN_ELIBB";
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
	"\n#define TS_A";
	return s;
}

int main(int argc, char** argv) {
	const bool elibb = argc>1 && string(argv[1])=="on";
	const string out = argc>2 ? argv[2] : "kernel_dump.cl";
	const string code = device_defines(elibb) + get_opencl_c_code();
	std::ofstream f(out);
	f << code;
	f.close();
	std::cout << "geschrieben: " << out << " (" << code.size() << " Bytes, ELIBB=" << (elibb?"an":"aus") << ")\n";
	return 0;
}
