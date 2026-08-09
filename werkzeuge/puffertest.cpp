// Isolierter Geraetetest -- KEIN LBM, keine Physik, keine Randbedingung.
// Frage: rechnet und speichert ein OpenCL-Geraet oberhalb einer bestimmten Puffergroesse noch korrekt?
//
// Ablauf je Groesse:
//   1. Host fuellt einen Puffer mit einem bekannten Muster (Wert = Index mod 1000, als float).
//   2. Puffer auf das Geraet schreiben.
//   3. Ein trivialer Kernel addiert 1.0f auf jedes Element -- damit ist auch die RECHNUNG beteiligt,
//      nicht nur der Kopiervorgang.
//   4. Zurueckholen und JEDES Element gegen das erwartete Ergebnis pruefen.
// Ein Fehler hier kann nichts mit Stroemungsmechanik zu tun haben.
#define CL_HPP_TARGET_OPENCL_VERSION 300
#define CL_HPP_MINIMUM_OPENCL_VERSION 120
#define CL_HPP_ENABLE_EXCEPTIONS
#include <CL/opencl.hpp>
#include <cstdio>
#include <vector>
#include <string>

int main(int argc, char** argv) {
	const int want = argc>1 ? atoi(argv[1]) : 0;
	const bool zero_copy = argc>2 ? atoi(argv[2])!=0 : true;
	std::vector<cl::Platform> plats; cl::Platform::get(&plats);
	std::vector<cl::Device> devs;
	for(auto& p : plats) { std::vector<cl::Device> d; try { p.getDevices(CL_DEVICE_TYPE_ALL, &d); } catch(...) { continue; } for(auto& x : d) devs.push_back(x); }
	if(want>=(int)devs.size()) { printf("Geraet %d gibt es nicht (%zu vorhanden)\n", want, devs.size()); return 1; }
	cl::Device dev = devs[want];
	printf("Geraet %d: %s\n", want, dev.getInfo<CL_DEVICE_NAME>().c_str());
	printf("  globaler Speicher %.1f GB, groesste einzelne Allokation %.2f GB, Zero-Copy im Test: %s\n\n",
		(double)dev.getInfo<CL_DEVICE_GLOBAL_MEM_SIZE>()/1e9,
		(double)dev.getInfo<CL_DEVICE_MAX_MEM_ALLOC_SIZE>()/1e9, zero_copy?"ja":"nein");

	cl::Context ctx(dev); cl::CommandQueue q(ctx, dev);
	const char* src = "kernel void plus1(global float* a, const ulong n) { const ulong i = get_global_id(0); if(i<n) a[i] += 1.0f; }";
	cl::Program prog(ctx, std::string(src));
	try { prog.build({dev}); } catch(...) { printf("Uebersetzung fehlgeschlagen:\n%s\n", prog.getBuildInfo<CL_PROGRAM_BUILD_LOG>(dev).c_str()); return 1; }
	cl::Kernel k(prog, "plus1");

	printf("%9s | %13s | %10s | %s\n", "Groesse", "Elemente", "Fehler", "Beispiel (erwartet -> gelesen)");
	printf("----------+---------------+------------+------------------------------\n");
	for(double mb : {256.0, 512.0, 1024.0, 1536.0, 2048.0, 2328.0, 3072.0}) {
		const size_t n = (size_t)(mb*1048576.0/4.0);
		const size_t bytes = n*4ull;
		if(bytes > dev.getInfo<CL_DEVICE_MAX_MEM_ALLOC_SIZE>()) { printf("%7.0f MB | %13zu | %-10s | ueber MAX_MEM_ALLOC_SIZE\n", mb, n, "--"); continue; }
		std::vector<float> host(n);
		for(size_t i=0; i<n; i++) host[i] = (float)(i%1000ull);
		cl_mem_flags fl = CL_MEM_READ_WRITE | (zero_copy ? CL_MEM_USE_HOST_PTR : CL_MEM_COPY_HOST_PTR);
		cl::Buffer buf;
		try { buf = cl::Buffer(ctx, fl, bytes, host.data()); } catch(cl::Error& e) { printf("%7.0f MB | %13zu | %-10s | Allokation fehlgeschlagen (%d)\n", mb, n, "--", e.err()); continue; }
		try {
			if(!zero_copy) q.enqueueWriteBuffer(buf, CL_TRUE, 0, bytes, host.data());
			k.setArg(0, buf); k.setArg(1, (cl_ulong)n);
			q.enqueueNDRangeKernel(k, cl::NullRange, cl::NDRange(((n+63)/64)*64), cl::NDRange(64));
			q.finish();
			std::vector<float> back(n);
			if(zero_copy) { float* p = (float*)q.enqueueMapBuffer(buf, CL_TRUE, CL_MAP_READ, 0, bytes); memcpy(back.data(), p, bytes); q.enqueueUnmapMemObject(buf, p); q.finish(); }
			else q.enqueueReadBuffer(buf, CL_TRUE, 0, bytes, back.data());
			size_t bad = 0; size_t first = 0; float got = 0.0f, exp = 0.0f;
			for(size_t i=0; i<n; i++) { const float e = (float)(i%1000ull)+1.0f; if(back[i]!=e) { if(!bad) { first=i; got=back[i]; exp=e; } bad++; } }
			if(bad==0) printf("%7.0f MB | %13zu | %10s | alle Elemente korrekt\n", mb, n, "0");
			else printf("%7.0f MB | %13zu | %10zu | ab Index %zu: %.1f -> %.1f\n", mb, n, bad, first, exp, got);
		} catch(cl::Error& e) { printf("%7.0f MB | %13zu | %-10s | OpenCL-Fehler %d\n", mb, n, "--", e.err()); }
	}
	return 0;
}
