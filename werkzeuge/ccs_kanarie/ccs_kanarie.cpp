// ccs_kanarie.cpp — empirischer Detektor fuer den Xe-Flat-CCS-Aufrundungsfehler
// (CC-Auftrag Heiko 26.08.2026; Kontext: Torvalds-Commit 818bebeb63dd / dri-devel 08/2026:
// get_flat_ccs_offset() rundet die CCS-Basis auf 128 KiB AUF -> die letzten 2 KiB der
// Seite landen im VRAM-Allocator-Pool, die Kompressionseinheit schreibt dort ohne PTE/BO
// -> stille Korruption am OBEREN Ende des nutzbaren VRAM).
//
// PRINZIP: VRAM in Bloecken bis zur Erschoepfung fuellen (damit die oberste Seite sicher
// in einem eigenen Puffer liegt), jeden Block mit blockeigenem 64-bit-Muster fuellen,
// SOFORT on-device verifizieren (Kontrolle: 0 Fehler, sonst ist das Werkzeug defekt),
// dann WARTEN (Desktop-Kompression arbeitet), erneut verifizieren. Jede Abweichung nach
// bestandener Sofortkontrolle = Fremdschreiber im eigenen Puffer -> Bug-Nachweis.
// KEIN Nachweis moeglich, wenn die oberste Seite trotz Fuellung nicht in unseren
// Puffern liegt (Allocator-Platzierung) -- deshalb wird die Restluft mitgedruckt.
//
// Aufruf: ccs_kanarie <device_id> [warte_s=300] [reserve_mb=0]
//   reserve_mb > 0 = sanfter Modus (laesst Luft fuer den Desktop; Deckung der obersten
//   Seite dann NICHT garantiert -- nur fuer Vorab-Proben, nicht fuer das Verdikt).
// Exit: 0 = PASS (keine Korruption in unseren Puffern), 1 = KORRUPTION, 2 = Werkzeugfehler.
// NUR DIAGNOSE: kein FluidX3D-Bezug, kein Kernel-Eingriff. Census-Pflicht wie GPU-Lauf.
#define CL_TARGET_OPENCL_VERSION 300
#include <CL/cl.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

static const char* KSRC =
"__kernel void pruefe(__global const ulong* p, const ulong muster, const ulong n,"
"                     __global uint* fehler, __global ulong* erste) {"
"  const ulong i = get_global_id(0);"
"  if(i>=n) return;"
"  if(p[i]!=muster) { if(atomic_inc(fehler)==0u) erste[0]=i; }"
"}";

static void die(const char* msg, cl_int err) { fprintf(stderr, "WERKZEUGFEHLER: %s (cl_err %d)\n", msg, err); exit(2); }

int main(int argc, char** argv) {
	const cl_uint dev_wunsch = argc>1 ? (cl_uint)atoi(argv[1]) : 0u;
	const int warte_s   = argc>2 ? atoi(argv[2]) : 300;
	const long reserve_mb = argc>3 ? atol(argv[3]) : 0l;
	cl_int err = 0;
	cl_uint np = 0; clGetPlatformIDs(0, nullptr, &np); if(np==0u) die("keine OpenCL-Plattform", 0);
	std::vector<cl_platform_id> plats(np); clGetPlatformIDs(np, plats.data(), nullptr);
	std::vector<cl_device_id> devs;
	for(cl_platform_id p : plats) {
		cl_uint nd=0; if(clGetDeviceIDs(p, CL_DEVICE_TYPE_ALL, 0, nullptr, &nd)!=CL_SUCCESS||nd==0u) continue;
		std::vector<cl_device_id> d(nd); clGetDeviceIDs(p, CL_DEVICE_TYPE_ALL, nd, d.data(), nullptr);
		for(cl_device_id x : d) devs.push_back(x);
	}
	for(cl_uint i=0u; i<(cl_uint)devs.size(); i++) { char nm[256]={0}; clGetDeviceInfo(devs[i], CL_DEVICE_NAME, 255, nm, nullptr); printf("Device %u: %s%s\n", i, nm, i==dev_wunsch?"  <== gewaehlt":""); }
	if(dev_wunsch>=(cl_uint)devs.size()) die("device_id ausserhalb der Liste", 0);
	cl_device_id dev = devs[dev_wunsch];
	cl_ulong gmem=0, maxalloc=0; clGetDeviceInfo(dev, CL_DEVICE_GLOBAL_MEM_SIZE, sizeof(gmem), &gmem, nullptr);
	clGetDeviceInfo(dev, CL_DEVICE_MAX_MEM_ALLOC_SIZE, sizeof(maxalloc), &maxalloc, nullptr);
	printf("GLOBAL_MEM %.1f MB, MAX_ALLOC %.1f MB, Wartezeit %d s, Reserve %ld MB\n", gmem/1048576.0, maxalloc/1048576.0, warte_s, reserve_mb);
	cl_context ctx = clCreateContext(nullptr, 1, &dev, nullptr, nullptr, &err); if(err) die("Context", err);
	cl_command_queue q = clCreateCommandQueueWithProperties(ctx, dev, nullptr, &err); if(err) die("Queue", err);
	cl_program prog = clCreateProgramWithSource(ctx, 1, &KSRC, nullptr, &err); if(err) die("Program", err);
	// -cl-intel-greater-than-4GB-buffer-required: dieselbe Option wie der Solver (>4-GB-Puffer)
	if(clBuildProgram(prog, 1, &dev, "-cl-std=CL3.0 -cl-intel-greater-than-4GB-buffer-required", nullptr, nullptr)!=CL_SUCCESS) {
		size_t ls=0; clGetProgramBuildInfo(prog, dev, CL_PROGRAM_BUILD_LOG, 0, nullptr, &ls);
		std::string log(ls, '\0'); clGetProgramBuildInfo(prog, dev, CL_PROGRAM_BUILD_LOG, ls, log.data(), nullptr);
		fprintf(stderr, "%s\n", log.c_str()); die("Build", 0);
	}
	cl_kernel k = clCreateKernel(prog, "pruefe", &err); if(err) die("Kernel", err);
	cl_mem fehler_b = clCreateBuffer(ctx, CL_MEM_READ_WRITE, 4, nullptr, &err); if(err) die("fehler_b", err);
	cl_mem erste_b  = clCreateBuffer(ctx, CL_MEM_READ_WRITE, 8, nullptr, &err); if(err) die("erste_b", err);

	// Fuellen bis zur Erschoepfung: Blockgroesse halbiert sich bei Fehlschlag, Untergrenze 2 MB.
	struct Block { cl_mem m; cl_ulong bytes; cl_ulong muster; };
	std::vector<Block> bloecke;
	cl_ulong gesamt=0, block=maxalloc;
	const cl_ulong reserve = (cl_ulong)reserve_mb*1048576ull, untergrenze=2ull*1048576ull;
	while(block>=untergrenze) {
		if(reserve>0ull && gesamt+block+reserve>gmem) { if(block==untergrenze) break; block=block/2ull<untergrenze?untergrenze:block/2ull; continue; }
		cl_mem m = clCreateBuffer(ctx, CL_MEM_READ_WRITE, block, nullptr, &err);
		if(err==CL_SUCCESS) { // Allokation ist lazy -- erst das Fill bindet die Seiten
			const cl_ulong muster = 0xCC5CAFE000000000ull ^ (cl_ulong)bloecke.size();
			cl_int ferr = clEnqueueFillBuffer(q, m, &muster, 8, 0, block, 0, nullptr, nullptr);
			if(ferr==CL_SUCCESS) ferr = clFinish(q);
			if(ferr!=CL_SUCCESS) { clReleaseMemObject(m); if(block==untergrenze) break; block/=2ull; continue; }
			bloecke.push_back({m, block, muster}); gesamt+=block;
			printf("  Block %2zu: %8.1f MB (gesamt %8.1f MB)\n", bloecke.size()-1, block/1048576.0, gesamt/1048576.0);
		} else { if(block==untergrenze) break; block=block/2ull<untergrenze?untergrenze:block/2ull; }
	}
	printf("Belegt: %.1f MB von %.1f MB -- Restluft %.1f MB (je kleiner, desto sicherer liegt die oberste Seite in unseren Puffern)\n", gesamt/1048576.0, gmem/1048576.0, (gmem-gesamt)/1048576.0);
	if(bloecke.empty()) die("keine Allokation gelungen", 0);

	auto verifiziere = [&](const char* phase)->cl_ulong {
		cl_ulong summe=0;
		for(size_t i=0; i<bloecke.size(); i++) {
			const cl_uint null32=0u; const cl_ulong null64=0ull;
			clEnqueueWriteBuffer(q, fehler_b, CL_TRUE, 0, 4, &null32, 0, nullptr, nullptr);
			clEnqueueWriteBuffer(q, erste_b, CL_TRUE, 0, 8, &null64, 0, nullptr, nullptr);
			const cl_ulong n = bloecke[i].bytes/8ull;
			clSetKernelArg(k, 0, sizeof(cl_mem), &bloecke[i].m);
			clSetKernelArg(k, 1, sizeof(cl_ulong), &bloecke[i].muster);
			clSetKernelArg(k, 2, sizeof(cl_ulong), &n);
			clSetKernelArg(k, 3, sizeof(cl_mem), &fehler_b);
			clSetKernelArg(k, 4, sizeof(cl_mem), &erste_b);
			size_t g = (size_t)((n+63ull)/64ull*64ull);
			if(clEnqueueNDRangeKernel(q, k, 1, nullptr, &g, nullptr, 0, nullptr, nullptr)!=CL_SUCCESS) die("NDRange", 0);
			clFinish(q);
			cl_uint f=0; cl_ulong e=0;
			clEnqueueReadBuffer(q, fehler_b, CL_TRUE, 0, 4, &f, 0, nullptr, nullptr);
			clEnqueueReadBuffer(q, erste_b, CL_TRUE, 0, 8, &e, 0, nullptr, nullptr);
			if(f>0u) printf("  %s: Block %zu -- %u fehlerhafte 8-B-Worte, erstes bei Byte-Offset 0x%llx\n", phase, i, f, (unsigned long long)(e*8ull));
			summe += f;
		}
		printf("%s: %llu fehlerhafte Worte ueber %zu Bloecke\n", phase, (unsigned long long)summe, bloecke.size());
		return summe;
	};

	if(verifiziere("SOFORTKONTROLLE")>0ull) die("Sofortkontrolle verletzt -- Werkzeug/Transfer defekt, KEIN Bug-Verdikt moeglich", 0);
	printf("Warte %d s (Desktop-/Kompressionsaktivitaet laeuft normal weiter)...\n", warte_s);
	for(int r=warte_s; r>0; r-=30) { std::this_thread::sleep_for(std::chrono::seconds(r<30?r:30)); printf("  ... noch %d s\n", r-30>0?r-30:0); fflush(stdout); }
	const cl_ulong schaden = verifiziere("ENDKONTROLLE");
	for(Block& b : bloecke) clReleaseMemObject(b.m);
	if(schaden>0ull) { printf("VERDIKT: KORRUPTION NACHGEWIESEN -- Fremdschreiber im eigenen VRAM-Puffer (Flat-CCS-Verdacht).\n"); return 1; }
	printf("VERDIKT: PASS -- keine Korruption in %.1f MB ueber %d s (Deckungsluecke: Restluft oben, s. o.).\n", gesamt/1048576.0, warte_s);
	return 0;
}
