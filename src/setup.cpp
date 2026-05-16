// MODIFIED FILE — see MODIFICATIONS.md for changes vs upstream ProjectPhysX/FluidX3D.
// Fork: github.com/heikogleu-dev/FluidX3D-Intel-B70 — Intel Arc Pro B70 (Battlemage) patches.
// Original copyright: (c) 2022-2026 Dr. Moritz Lehmann, see LICENSE.md.
#include "setup.hpp"
// PERF-G 2026-05-15: concurrent LBM execution (Far ∥ Near auf einer GPU) via LBM::run_async() + finish() — keine host-threads, queues parallelisieren via OpenCL driver



#ifdef BENCHMARK
#include "info.hpp"
void main_setup() { // benchmark; required extensions in defines.hpp: BENCHMARK, optionally FP16S or FP16C
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	uint mlups = 0u; {

		//LBM lbm( 32u,  32u,  32u, 1.0f);
		//LBM lbm( 64u,  64u,  64u, 1.0f);
		//LBM lbm(128u, 128u, 128u, 1.0f);
		LBM lbm(256u, 256u, 256u, 1.0f); // default
		//LBM lbm(384u, 384u, 384u, 1.0f);
		//LBM lbm(512u, 512u, 512u, 1.0f);

		//const uint memory = 1488u; // memory occupation in MB (for multi-GPU benchmarks: make this close to as large as the GPU's VRAM capacity)
		//const uint3 lbm_N = (resolution(float3(1.0f, 1.0f, 1.0f), memory)/4u)*4u; // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
		//LBM lbm(1u*lbm_N.x, 1u*lbm_N.y, 1u*lbm_N.z, 1u, 1u, 1u, 1.0f); // 1 GPU
		//LBM lbm(2u*lbm_N.x, 1u*lbm_N.y, 1u*lbm_N.z, 2u, 1u, 1u, 1.0f); // 2 GPUs
		//LBM lbm(2u*lbm_N.x, 2u*lbm_N.y, 1u*lbm_N.z, 2u, 2u, 1u, 1.0f); // 4 GPUs
		//LBM lbm(2u*lbm_N.x, 2u*lbm_N.y, 2u*lbm_N.z, 2u, 2u, 2u, 1.0f); // 8 GPUs

		// #########################################################################################################################################################################################
		for(uint i=0u; i<1000u; i++) {
			lbm.run(10u, 1000u*10u);
			mlups = max(mlups, to_uint((double)lbm.get_N()*1E-6/info.runtime_lbm_timestep_smooth));
		}
	} // make lbm object go out of scope to free its memory
	print_info("Peak MLUPs/s = "+to_string(mlups));
#if defined(_WIN32)
	wait();
#endif // Windows
} /**/
#endif // BENCHMARK



/*/*void main_setup() { // WINDKANAL mit ursprünglichen Abmessungen & Flussrichtung (+x); required extensions: FP16C, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = uint3(910u, 455u, 325u); // Grid-Verhältnis ~7 : 3.5 : 2.5; preserves physical dimensions
	const float lbm_u = 0.075f; // LBM velocity (standard)
	const float si_T = 15.0f; // Simulationszeit 15 Sekunden
	const float si_u = 30.0f; // 30 m/s Wind
	const float si_length = 4.0f; // Fahrzeuglänge: 4.0 m (echte Modell-Länge)
	const float cell_size = 7.0f / (float)lbm_N.x; // Box-Länge 7 m → Zellgröße ~8.54 mm
	const float lbm_length = si_length / cell_size; // 4.0 m / 8.54 mm ≈ 469 Zellen Fahrzeuglänge
	const float si_width = 1.8f; // Fahrzeugbreite 1.8 m
	const float si_nu = 1.48E-5f, si_rho = 1.225f; // Luft bei 15°C
	units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
	const ulong lbm_T = units.t(si_T);
	print_info("Re (basierend auf Länge) = "+to_string(to_uint(units.si_Re(si_length, si_u, si_nu))));
	LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu);
	// ###################################################################################### define geometry ######################################################################################
	Mesh* vehicle = read_stl(get_exe_path()+"../scenes/vehicle.stl");
	// Skalierung entlang der X-Achse (Modell ist in X-Richtung orientiert)
	const float3 bbox = vehicle->get_bounding_box_size();
	const float vehicle_length_x = bbox.x;
	const float scale = lbm_length / vehicle_length_x; // mappe 4.5 m SI auf X-Länge des STL
	vehicle->scale(scale);
	// Positionierung: x/y zentrieren, z auf Boden mit definierter Bodenfreiheit
	const float ground_clearance_cells = 1.0f; // ~1 Zelle Bodenfreiheit
	const float3 offset = float3(
		lbm.center().x - vehicle->get_bounding_box_center().x,
		lbm.center().y - vehicle->get_bounding_box_center().y,
		ground_clearance_cells - vehicle->pmin.z);
	vehicle->translate(float3(
		(1.0f/3.0f)*(float)lbm_N.x - vehicle->get_bounding_box_center().x, // nach vorn auf 1/3 der Box
		lbm.center().y - vehicle->get_bounding_box_center().y,              // seitlich zentriert
		offset.z));
	vehicle->set_center(vehicle->get_center_of_mass()); // Rotation um Schwerpunkt
	lbm.voxelize_mesh_on_device(vehicle);
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		// Solids: Boden (z=0), Seitenwände (y), Decke (z=Nz-1)
		if(z==0u) lbm.flags[n] = TYPE_S; // Boden: statisch
		if(y==0u||y==Ny-1u) lbm.flags[n] = TYPE_S; // Seitenwände: no-slip
		if(z==Nz-1u) lbm.flags[n] = TYPE_S; // Decke: no-slip
		// Inlet/Outlet auf x-Flächen
		if(x==0u||x==Nx-1u) lbm.flags[n] = TYPE_E; // Einlass/Auslass
		// Strömung in +x Richtung für Fluidzellen
		if(lbm.flags[n]!=TYPE_S) { lbm.u.x[n] = lbm_u; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f; }
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_Q_CRITERION; // Nur Q-Criterion (Wirbel), Wände ausgeblendet
	lbm.run(10000u); // 10000 Steps Test-Run, dann VTK + Force-Field-Export + CSV
	lbm.update_force_field(); // Kräfte auf TYPE_S (Fahrzeug + Boden + Decke) berechnen
	const string export_path = get_exe_path()+"../export/";
	lbm.u.write_device_to_vtk(export_path);          // velocity field
	lbm.rho.write_device_to_vtk(export_path);        // density field
	lbm.flags.write_device_to_vtk(export_path);      // cell type flags
	lbm.F.write_device_to_vtk(export_path);          // force field on solid boundaries
	lbm.write_mesh_to_vtk(vehicle, export_path);     // vehicle STL as VTK
	{ // CSV-Export der Forces auf Solid-Cells (für manuelle Auswertung)
		std::ofstream csv(export_path+"forces_solid_cells.csv");
		csv << "step,x,y,z,Fx_lbm,Fy_lbm,Fz_lbm,Fx_SI,Fy_SI,Fz_SI\n";
		const float Fconv = units.si_F(1.0f);
		const ulong N = lbm.get_N(); const uint Nx2=lbm.get_Nx(), Ny2=lbm.get_Ny(); ulong written = 0ull;
		for(ulong n=0ull; n<N; n++) {
			if(lbm.flags[n]==TYPE_S && (lbm.F.x[n]!=0.0f || lbm.F.y[n]!=0.0f || lbm.F.z[n]!=0.0f)) {
				const uint x = (uint)(n%Nx2), y = (uint)((n/Nx2)%Ny2), z = (uint)(n/((ulong)Nx2*Ny2));
				csv << lbm.get_t() << "," << x << "," << y << "," << z << ","
				    << lbm.F.x[n] << "," << lbm.F.y[n] << "," << lbm.F.z[n] << ","
				    << lbm.F.x[n]*Fconv << "," << lbm.F.y[n]*Fconv << "," << lbm.F.z[n]*Fconv << "\n";
				written++;
			}
		}
		csv.close();
		print_info("CSV-Force-Export: "+to_string(written)+" Solid-Cells mit F!=0");
	}
	print_info("VTK + Force-Field + CSV-Export abgeschlossen: "+export_path);
	std::fflush(nullptr);
	_exit(0); // umgehe xe-driver Cleanup-Race (bekannter -EINVAL fault); Daten sind bereits flushed
} /**/

/*void main_setup() { // WINDKANAL halbe Domain - andere Seite mit Symmetrieebene & Flussrichtung (+x); required extensions: FP16C, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = uint3(1143u, 285u, 408u); // Grid-Verhältnis ~7 : 1.75 : 2.5; halbe Y-Ausdehnung mit Symmetrie, ~133.8 Mio Zellen
	const float lbm_u = 0.075f; // LBM velocity (standard)
	const float si_T = 15.0f; // Simulationszeit 15 Sekunden
	const float si_u = 30.0f; // 30 m/s Wind
	const float si_length = 4.0f; // Fahrzeuglänge: 4.0 m (echte Modell-Länge)
	const float cell_size = 7.0f / (float)lbm_N.x; // Box-Länge 7 m → Zellgröße ~8.54 mm
	const float lbm_length = si_length / cell_size; // 4.0 m / 8.54 mm ≈ 469 Zellen Fahrzeuglänge
	const float si_width = 1.8f; // Fahrzeugbreite 1.8 m
	const float si_nu = 1.48E-5f, si_rho = 1.225f; // Luft bei 15°C
	units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
	const ulong lbm_T = units.t(si_T);
	print_info("Re (basierend auf Länge) = "+to_string(to_uint(units.si_Re(si_length, si_u, si_nu))));
	LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu);
	// ###################################################################################### define geometry ######################################################################################
	Mesh* vehicle = read_stl(get_exe_path()+"../scenes/vehicle.stl");
	// Skalierung entlang der X-Achse (Modell ist in X-Richtung orientiert)
	const float3 bbox = vehicle->get_bounding_box_size();
	const float vehicle_length_x = bbox.x;
	const float scale = lbm_length / vehicle_length_x; // mappe 4.5 m SI auf X-Länge des STL
	vehicle->scale(scale);
	// Positionierung: x/y zentrieren (nur halbe Domain in Y), z auf Boden mit definierter Bodenfreiheit
	const float ground_clearance_cells = 1.0f; // ~1 Zelle Bodenfreiheit
	vehicle->translate(float3(
		(1.0f/3.0f)*(float)lbm_N.x - vehicle->get_bounding_box_center().x, // nach vorn auf 1/3 der Box
		-2.0f * vehicle->get_bounding_box_center().y,                     // auf andere Seite der Symmetrieebene spiegeln (negative Y)
		ground_clearance_cells - vehicle->pmin.z));
	vehicle->set_center(vehicle->get_center_of_mass()); // Rotation um Schwerpunkt
	lbm.voxelize_mesh_on_device(vehicle);
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		// Boden: moving wall mit 30 m/s in +x (Fahrzeug fährt)
		if(z==0u) {
			lbm.flags[n] = TYPE_S;
			lbm.u.x[n] = lbm_u; // Bewegter Boden in Strömungsrichtung
			lbm.u.y[n] = 0.0f;
			lbm.u.z[n] = 0.0f;
		}
		if(z==Nz-1u) lbm.flags[n] = TYPE_S; // Decke: no-slip
		// Symmetrieebene bei y=0 (Slip-Bedingung), Wand bei y=Ny-1
		if(y==Ny-1u) lbm.flags[n] = TYPE_S; // Außenwand: no-slip
		// Inlet/Outlet auf x-Flächen
		if(x==0u||x==Nx-1u) lbm.flags[n] = TYPE_E; // Einlass/Auslass
		// Strömung in +x Richtung für Fluidzellen
		if(lbm.flags[n]!=TYPE_S) { lbm.u.x[n] = lbm_u; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f; }
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_Q_CRITERION; // Nur Q-Criterion (Wirbel), Wände ausgeblendet
	lbm.run(10000u); // 10000 Steps Test-Run, dann VTK + Force-Field-Export + CSV
	lbm.update_force_field(); // Kräfte auf TYPE_S (Fahrzeug + Boden + Decke) berechnen
	const string export_path = get_exe_path()+"../export/";
	lbm.u.write_device_to_vtk(export_path);          // velocity field
	lbm.rho.write_device_to_vtk(export_path);        // density field
	lbm.flags.write_device_to_vtk(export_path);      // cell type flags
	lbm.F.write_device_to_vtk(export_path);          // force field on solid boundaries
	lbm.write_mesh_to_vtk(vehicle, export_path);     // vehicle STL as VTK
	{ // CSV-Export der Forces auf Solid-Cells (für manuelle Auswertung)
		std::ofstream csv(export_path+"forces_solid_cells.csv");
		csv << "step,x,y,z,Fx_lbm,Fy_lbm,Fz_lbm,Fx_SI,Fy_SI,Fz_SI\n";
		const float Fconv = units.si_F(1.0f);
		const ulong N = lbm.get_N(); const uint Nx2=lbm.get_Nx(), Ny2=lbm.get_Ny(); ulong written = 0ull;
		for(ulong n=0ull; n<N; n++) {
			if(lbm.flags[n]==TYPE_S && (lbm.F.x[n]!=0.0f || lbm.F.y[n]!=0.0f || lbm.F.z[n]!=0.0f)) {
				const uint x = (uint)(n%Nx2), y = (uint)((n/Nx2)%Ny2), z = (uint)(n/((ulong)Nx2*Ny2));
				csv << lbm.get_t() << "," << x << "," << y << "," << z << ","
				    << lbm.F.x[n] << "," << lbm.F.y[n] << "," << lbm.F.z[n] << ","
				    << lbm.F.x[n]*Fconv << "," << lbm.F.y[n]*Fconv << "," << lbm.F.z[n]*Fconv << "\n";
				written++;
			}
		}
		csv.close();
		print_info("CSV-Force-Export: "+to_string(written)+" Solid-Cells mit F!=0");
	}
	print_info("VTK + Force-Field + CSV-Export abgeschlossen: "+export_path);
	std::fflush(nullptr);
	_exit(0); // umgehe xe-driver Cleanup-Race (bekannter -EINVAL fault); Daten sind bereits flushed
} /**/

// ============== CC#6: Aero-Box 14×2.5×4.5m (half) | 14×5×4.5m (full), 10mm uniform, Auto-Stop bei <1% Force-Drift ==============
// Compile-time toggle CC6_FULL_DOMAIN: false=Halbdomain (Y[0,2.5m]), true=Volldomain (Y[-2.5,+2.5m])
//
// Halbdomain: 1500×250×450 = 168.75 M cells (~10 GB). Volldomain: 1500×500×450 = 337.5 M cells (~19 GB).
//
// Geometrie-Updates ggü. CC#5:
//   - X: Domain bleibt 15m, aber Vehicle wird um 1m nach +X verschoben (Inlet rückt -1m weg)
//        Vehicle X-center @ Cell 400 statt 300 → 4m Anlauf, 11m Wake (statt 3m / 12m)
//   - Y: Halbdomain um 0.5m gekürzt (Y_max: 3m → 2.5m). Volldomain entsprechend symmetrisch.
//   - Z: unverändert 4.5m (450 cells).
//
// Wall-config (identisch zu CC#5, FluidX3D-Aero-Konvention):
//   - Boden Z=0: TYPE_S Moving-Wall +x (rolling road)
//   - Alle anderen Außenwände: TYPE_E free-stream u_x=lbm_u
//
// Auto-Stop-Konvergenz:
//   - Tracking: Sliding-Window von 50 Chunks (5000 Steps).
//   - Convergence-Test: |Fx_recent_avg - Fx_prev_avg| / |Fx_recent_avg| < 1% UND dito für Fy.
//   - Frühester Exit: nach 100 Chunks (10000 Steps), damit beide Fenster gefüllt sind.
//   - Max-Run: 1000 Chunks (100k Steps).
//
// Ground clearance: 1 cell = 10 mm.
// CC6_MODE: 0 = Halbdomain TYPE_E pseudo-sym (CC#6-Half, Drag 16k N — broken)
//           1 = Volldomain (CC#6-Full, no symmetry — REFERENCE Drag 2.2k N)
//           2 = Halbdomain TYPE_Y inline specular-reflection (CC#7-V1/V2 — FAILED, EP-storage no-op)
//           3 = Halbdomain TYPE_S Moving-Wall am Y_min (CC#7-Alt1 — FAILED, Drag 17.7k N)
//           4 = Halbdomain TYPE_E|TYPE_Y Ghost-Cell-Mirror (CC#8 — FAILED, Drag 14.3k N)
//           5 = Halbdomain TYPE_Y + separate post-stream apply_freeslip_y kernel (CC#9 — waLBerla pattern, FAILED ~13.5-14.4k)
// V6 strip-TYPE_X-from-y=0 patch applies to all half-domain modes (0,2,3,4,5) — gives ~20% drag reduction.
// V7 = Mode 5 + V6 was the best combined half-domain achievable: Fx ≈ 14045 N (still 12× target).
// V8 = Mode 3 with sym-plane = TYPE_S|TYPE_Y → isolated diagnostic: Fx_sym = 3.4 MN bookkeeping confirms
//      the sym-plane is a pseudo-rolling-road, NOT a free-slip mirror (Fy_sym ≠ 0, Fz_sym ≠ 0).
#define CC6_MODE 1
#define CC7_DIAGNOSE 0  // 1 = print TYPE_Y cell count + abort after few steps
#define CC7_DIAG_MAXSTEPS 1000u
#define VEHICLE_GR_YARIS 0  // 1 = Toyota GR Yaris (vehicle-alt-bin.stl). 0 = scenes/vehicle.stl (canonical MR2 — used for smoke tests per user direction 2026-05-13)
#define AHMED_MODE 0        // CC#X: 0 = Real Vehicle (Yaris/MR2 setup), 1 = Ahmed 25° (Phase 1 FAILED, see findings/CC_X_ahmed/SESSION_2026-05-11_PHASE1_FAIL.md), 2 = Ahmed 35°
#define SELF_COUPLING_TEST 0 // Phase 5b-pre 2026-05-13: validate couple_fields() pipeline via single-domain self-coupling. Temporarily off for baseline.
#define PHASE_5B_DUAL_DOMAIN 0 // Phase 5b 2026-05-13: two-LBM-instance same-resolution Schwarz coupling (Far 225M + Near 38.85M @ 10mm). Set to 1 to activate; default 0 for clean baseline.
#define PHASE_5B_COUPLE_MODE 3 // 2026-05-15: Mode 3 PERF-G Additive Schwarz (concurrent Far||Near, symmetric 1-chunk lag) + α=0.20 Test (von User vorgeschlagen — Mode 3 Production zeigte sichtbare Near→Far Kante bei α=0.10, also stärkere Coupling testen).
#define PHASE_5B_DR 1          // Phase 5b-DR Production 2026-05-16: Far 13.5m anlauf 1.5m + TYPE_S Moving Wall floor + Mode 2 symm α=0.10 + Vehicle z=1/z=3 + auto-stop 2% over 5000 Far-steps
#define PHASE_7_TRIPLE_RES 0   // Phase 7 Triple-Resolution 2026-05-16 — Run 2 prepared (chunk_far=100, Vehicle-in-Coarse, multi-field convergence). Auf 1 setzen + rebuild + run für Production. See findings/PHASE_7_RUN1_2026-05-16.md.

#if AHMED_MODE>0
// ============================================================================
// CC#X Phase 1 — Ahmed Body Wall-Model Validation (Volldomain, simplified flat-front Ahmed)
// Reference: Lienhart & Stoots 2003. CD literature: 0.285 (25°) / 0.378 (35°)
// Domain: 8L × 2L × 2L at 5 mm cells → 1672 × 416 × 416 = 289 M cells
// u_∞ = 40 m/s, ν = 1.5e-5, Re_L = 2.78e6
// ============================================================================
void main_setup_ahmed() {
#if AHMED_MODE==1
	const int slant_deg = 25;
	const string ahmed_stl = "../ahmed_25.stl";
	const string csv_name = "forces_ahmed_25.csv";
	const float CD_lit = 0.285f;
	const float CL_lit = +0.345f;
#elif AHMED_MODE==2
	const int slant_deg = 35;
	const string ahmed_stl = "../ahmed_35.stl";
	const string csv_name = "forces_ahmed_35.csv";
	const float CD_lit = 0.378f;
	const float CL_lit = -0.105f;
#endif
	const float si_L = 1.044f;        // Ahmed body length [m]
	const float si_W = 0.389f;        // Ahmed body width  [m]
	const float si_H = 0.288f;        // Ahmed body height [m]
	const float si_A = 0.1124f;       // frontal area [m²]
	const float lbm_u = 0.075f;
	const float si_u = 40.0f;
	const float cell_size = 0.010f;   // CC#X retry: 10 mm (use proven MR2 domain, smaller blockage)
	const float lbm_length = si_L / cell_size; // 104.4 cells per body length
	const float si_nu = 1.5E-5f, si_rho = 1.225f;
	// Domain: 15m × 5m × 4.5m (proven MR2 domain → 7.8% Y-blockage, 6.4% Z-blockage for Ahmed)
	const uint Nx = 1500u;
	const uint Ny = 500u;
	const uint Nz = 450u;
	const uint3 lbm_N = uint3(Nx, Ny, Nz);
	const string label = "CC#X-AHMED-" + to_string(slant_deg) + "deg";
	units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_L, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
	print_info(label+" Re(L) = "+to_string(to_uint(units.si_Re(si_L, si_u, si_nu))));
	print_info(label+" Cells: "+to_string(Nx)+" x "+to_string(Ny)+" x "+to_string(Nz)+" = "+to_string((ulong)Nx*Ny*Nz));
	print_info(label+" Literature: CD="+to_string(CD_lit,3u)+", CL="+to_string(CL_lit,3u)+" (Lienhart-Stoots 2003)");
	LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu);

	// Vehicle laden
	Mesh* vehicle = read_stl(get_exe_path()+ahmed_stl);
	const float3 bbox_orig = vehicle->get_bounding_box_size();
	vehicle->scale(lbm_length / bbox_orig.x); // scale STL to lbm_length cells in X
	const float3 vbbox = vehicle->get_bounding_box_size();
	const float3 vctr  = vehicle->get_bounding_box_center();
	// Body placement: X-center @ cell 400 (= 4m from inlet, 11m wake — same as MR2), Y-center, Z-min at clearance (baked into STL)
	const float x_target = 400.0f;
	const float y_target = (float)(Ny / 2u);
	vehicle->translate(float3(
		x_target - vctr.x,
		y_target - vctr.y,
		0.0f                              // STL already has z_bot=clearance baked in → no Z-shift needed
	));
	const float3 vmin = vehicle->pmin, vmax = vehicle->pmax;
	print_info(label+" Ahmed BBox in cells: X["+to_string(vmin.x,1u)+", "+to_string(vmax.x,1u)+"] Y["+to_string(vmin.y,1u)+", "+to_string(vmax.y,1u)+"] Z["+to_string(vmin.z,1u)+", "+to_string(vmax.z,1u)+"]");

	// Voxelize Ahmed body FIRST (marks body cells with TYPE_S|TYPE_X; standard FluidX3D pattern)
	lbm.voxelize_mesh_on_device(vehicle, TYPE_S|TYPE_X);

	// Floor + Walls (rolling road convention: Z=0 floor moves at u_∞)
	// Body cells (TYPE_X) protected via early-return — their u stays at default (0)
	parallel_for(lbm.get_N(), [&](ulong n) {
		uint x=0, y=0, z=0;
		lbm.coordinates(n, x, y, z);
		if((lbm.flags[n] & TYPE_X) != 0u) return;   // Ahmed body cells protected
		if(z==0u) {
			lbm.flags[n] = TYPE_S;
			lbm.u.x[n] = lbm_u; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
		} else if(x==0u || x==Nx-1u || y==0u || y==Ny-1u || z==Nz-1u) {
			lbm.flags[n] = TYPE_E;
		}
		if(lbm.flags[n]!=TYPE_S) { lbm.u.x[n] = lbm_u; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f; }
	});

	// Run-Loop mit Auto-Stop (CC#X verschärfte Konvergenz: 25 chunks window, 50 chunks min, 2% tol)
	const string force_csv_path = get_exe_path() + "../bin/" + csv_name;
	std::ofstream fcsv(force_csv_path);
	fcsv << "step,t_si,Fx_si,Fy_si,Fz_si\n";
	fcsv << std::scientific;
	const uint chunk = 100u;
	const uint chunks_max = 500u;       // hard cap 50k steps
	const uint conv_window = 25u;       // 2500 steps window
	const uint conv_min_chunks = 50u;   // earliest exit 5000 steps
	const float conv_tol = 0.02f;
	std::vector<float3> Fhist;
	Fhist.reserve(chunks_max);
	print_info(label+" Run start: max "+to_string(chunks_max*chunk)+" Steps; Auto-Stop |dFx/Fx|<2% over 2500 Steps (earliest @5000)");
	uint final_chunks = chunks_max;
	for(uint c = 0u; c < chunks_max; c++) {
		lbm.run(chunk);
		lbm.update_force_field();
		const float3 F_lbm = lbm.object_force(TYPE_S|TYPE_X);
		const float3 F_si  = float3(units.si_F(F_lbm.x), units.si_F(F_lbm.y), units.si_F(F_lbm.z));
		Fhist.push_back(F_si);
		const ulong step = (ulong)(c+1u) * chunk;
		const float t_si = units.si_t(step);
		fcsv << step << "," << t_si << "," << F_si.x << "," << F_si.y << "," << F_si.z << "\n";
		if((c+1u) % 10u == 0u) fcsv.flush();
		if((c+1u) % 25u == 0u) {
			const float CD_meas = F_si.x / (0.5f * si_rho * si_A * si_u * si_u);
			const float CL_meas = F_si.z / (0.5f * si_rho * si_A * si_u * si_u);
			print_info("step="+to_string(step)+" t="+to_string(t_si,3u)+"s Fx="+to_string(F_si.x,1u)+"N Fz="+to_string(F_si.z,1u)+"N | CD="+to_string(CD_meas,3u)+" (lit "+to_string(CD_lit,3u)+") CL="+to_string(CL_meas,3u)+" (lit "+to_string(CL_lit,3u)+")");
		}
		if(c+1u >= conv_min_chunks) {
			float3 recent_avg(0.0f), prev_avg(0.0f);
			for(uint k=0u; k<conv_window; k++) recent_avg += Fhist[Fhist.size()-1u-k];
			for(uint k=0u; k<conv_window; k++) prev_avg   += Fhist[Fhist.size()-1u-conv_window-k];
			recent_avg /= (float)conv_window;
			prev_avg   /= (float)conv_window;
			const float dx = (recent_avg.x!=0.0f) ? fabs(recent_avg.x - prev_avg.x) / fabs(recent_avg.x) : 1.0f;
			if(dx < conv_tol) {
				const float CD_final = recent_avg.x / (0.5f * si_rho * si_A * si_u * si_u);
				const float CL_final = recent_avg.z / (0.5f * si_rho * si_A * si_u * si_u);
				print_info(label+" CONVERGED at step "+to_string(step)+": |dFx|/|Fx|="+to_string(dx*100.0f,3u)+"%  Fx="+to_string(recent_avg.x,1u)+"N (CD="+to_string(CD_final,3u)+", lit "+to_string(CD_lit,3u)+")  Fz="+to_string(recent_avg.z,1u)+"N (CL="+to_string(CL_final,3u)+", lit "+to_string(CL_lit,3u)+")");
				final_chunks = c+1u;
				break;
			}
		}
	}
	fcsv.close();
	const string export_path = get_exe_path()+"../export/";
	lbm.u.write_device_to_vtk(export_path);
	lbm.flags.write_device_to_vtk(export_path);
	lbm.F.write_device_to_vtk(export_path);
	print_info(label+" done after "+to_string(final_chunks)+" chunks ("+to_string(final_chunks*chunk)+" steps).");
	std::fflush(nullptr);
	_exit(0);
}
#endif // AHMED_MODE>0

#if PHASE_5B_DUAL_DOMAIN
void main_setup_phase5b_dual(); // forward decl
#endif
#if PHASE_5B_DR
void main_setup_phase5b_dr();   // forward decl — Double-Resolution Schwarz
#endif
#if PHASE_7_TRIPLE_RES
void main_setup_phase7_triple_res(); // forward decl — Phase 7 Triple-Resolution (Near 4mm + Far 12mm B70, Coarse 24mm iGPU)
#endif

void main_setup() { // CC#6/CC#7 Aero-Box 10mm, Auto-Stop bei <2% Force-Drift. Required: FP16C, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID, VOLUME_FORCE, FORCE_FIELD.
#if AHMED_MODE>0
	main_setup_ahmed();
	return;
#endif
#if PHASE_7_TRIPLE_RES
	main_setup_phase7_triple_res();
	return;
#endif
#if PHASE_5B_DR
	main_setup_phase5b_dr();
	return;
#endif
#if PHASE_5B_DUAL_DOMAIN
	main_setup_phase5b_dual();
	return;
#endif
	// ============== Domain Setup ==============
#if CC6_MODE==1
	const uint3 lbm_N = uint3(1500u, 500u, 450u);  // 337.5 M Cells: 15m × 5m × 4.5m (Volldomain, Y[-2.5,+2.5m])
	const string label = "CC#6-FULL";
#elif CC6_MODE==2
	const uint3 lbm_N = uint3(1500u, 250u, 450u);  // 168.75 M Cells: 15m × 2.5m × 4.5m (Halbdomain, TYPE_Y inline sym — FAILED)
	const string label = "CC#7-HALF-SYM";
#elif CC6_MODE==3
	const uint3 lbm_N = uint3(1500u, 250u, 450u);  // 168.75 M Cells: 15m × 2.5m × 4.5m (Halbdomain, TYPE_S moving-wall sym-approx an Y_min — FAILED)
	const string label = "CC#7-ALT1-MOVING";
#elif CC6_MODE==4
	const uint3 lbm_N = uint3(1500u, 250u, 450u);  // 168.75 M Cells: 15m × 2.5m × 4.5m (Halbdomain, TYPE_E|TYPE_Y Ghost-Cell-Mirror an Y_min)
	const string label = "CC#8-GHOST-MIRROR";
#elif CC6_MODE==5
	const uint3 lbm_N = uint3(1500u, 250u, 450u);  // 168.75 M Cells: 15m × 2.5m × 4.5m (Halbdomain, TYPE_Y + post-stream apply_freeslip_y kernel)
	const string label = "CC#9-POSTSTREAM-FREESLIP";
#else
	const uint3 lbm_N = uint3(1500u, 250u, 450u);  // 168.75 M Cells: 15m × 2.5m × 4.5m (Halbdomain, TYPE_E pseudo-sym an Y_min)
	const string label = "CC#6-HALF";
#endif
	const float lbm_u = 0.075f;                    // standard LBM velocity scale
	const float si_u = 30.0f;                      // 30 m/s Wind
#if VEHICLE_GR_YARIS
	const float si_length = 3.995f;                // Toyota GR Yaris MY21 length [m]
#else
	const float si_length = 4.5f;                  // default vehicle length [m]
#endif
	const float cell_size = 0.010f;                // 10 mm uniform
	const float lbm_length = si_length / cell_size; // Vehicle X-Länge in LBM-cells (Yaris: 399.5, default: 450)
	const float si_nu = 1.48E-5f, si_rho = 1.225f; // Luft 15°C
	units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
#if VEHICLE_GR_YARIS
	print_info(label+" Vehicle: Toyota GR Yaris MY21 (Fully Stock), L="+to_string(si_length,3u)+"m, Frontal Area=2.078 m^2 (per OpenFOAM RANS reference)");
#else
	print_info(label+" Vehicle: default sport car, L="+to_string(si_length,3u)+"m");
#endif
	print_info(label+" Re(L) = "+to_string(to_uint(units.si_Re(si_length, si_u, si_nu))));
	print_info(label+" Cells: "+to_string(lbm_N.x)+" x "+to_string(lbm_N.y)+" x "+to_string(lbm_N.z)+" = "+to_string((ulong)lbm_N.x*lbm_N.y*lbm_N.z));
#if CC6_MODE==1
	print_info(label+" Box: X[-4m,+11m]=15m | Y[-2.5m,+2.5m]=5m (FULL) | Z[0,4.5m]=4.5m | Vehicle X-center @ cell 400 / Y-center @ 250");
#else
	print_info(label+" Box: X[-4m,+11m]=15m | Y[0,2.5m]=2.5m (HALF) | Z[0,4.5m]=4.5m | Vehicle X-center @ cell 400 / Y-center @ 0");
#endif
	print_info(label+" Walls: floor=TYPE_S Moving-Wall (rolling road); rest=TYPE_E free-stream u_x=lbm_u");
	LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu);
#ifdef SPONGE_LAYER
	// lbm.sponge_u_inlet = lbm_u; // Phase 5a Iron-Rule trigger 2026-05-13: sponge cuts off wake on full-domain (74% drag drop). Re-enable only for compact Multi-Res Mid-boxes where outlet is < ~3L from vehicle. See findings/PHASE_5A_SPONGE_IRON_RULE_TRIGGER_2026-05-13.md
#endif

	// ============== Vehicle: STL laden + positionieren ==============
#if VEHICLE_GR_YARIS
	Mesh* vehicle = read_stl(get_exe_path()+"../vehicle-alt-bin.stl"); // Toyota GR Yaris MY21 Stock (5.34M tri, 99.985% watertight, binary STL, units=meters)
#else
	Mesh* vehicle = read_stl(get_exe_path()+"../scenes/vehicle.stl");
#endif
	const float3 bbox_orig = vehicle->get_bounding_box_size();
	vehicle->scale(lbm_length / bbox_orig.x); // X-Achse auf 450 cells (4.5 m / 10 mm)
	const float3 vbbox = vehicle->get_bounding_box_size();
	const float3 vctr  = vehicle->get_bounding_box_center();
#if CC6_MODE==1
	const float vehicle_y_target = (float)(lbm_N.y / 2u); // Volldomain: Y-mitte = Cell 250
#elif CC6_MODE==5
	const float vehicle_y_target = 1.0f;                  // CC#9-V5 DIAGNOSE: vehicle shifted up 1 cell to avoid periodic-wrap pollution of y=0 vehicle cells
#else
	const float vehicle_y_target = 0.0f;                  // Halbdomain: Y-Center auf 0 → Vehicle wird vom Y_min sym-plane mittig durchschnitten
#endif
	vehicle->translate(float3(
		400.0f - vctr.x,                       // X-Center bei Cell 400 (= 4m vom Inlet, 11m bis Outlet)
		vehicle_y_target - vctr.y,             // Halb: 0; Voll: Ny/2
		1.0f   - (vctr.z - vbbox.z * 0.5f)));  // Z-Min auf 1 (Räder auf Boden)

	// Geometrie-Sanity-Check
	const float3 vmin = vehicle->pmin, vmax = vehicle->pmax;
	print_info("Vehicle BBox after translate: X["+to_string(vmin.x,1u)+", "+to_string(vmax.x,1u)+"] Y["+to_string(vmin.y,1u)+", "+to_string(vmax.y,1u)+"] Z["+to_string(vmin.z,1u)+", "+to_string(vmax.z,1u)+"]");
#if CC6_MODE!=1
#if CC6_MODE==5
	if(vmin.y > 1.5f || vmax.y < 0.5f) {
		print_info("FATAL: CC#9-V5 Vehicle should sit just above y=0! Aborting.");
		std::fflush(nullptr); _exit(2);
	}
	print_info("CC#9-V5 OK: Vehicle Y-bbox shifted up by 1 cell, no vehicle cells at y=0");
#else
	if(vmin.y > 0.5f || vmax.y < -0.5f) {
		print_info("FATAL: Half-domain Vehicle does not cross Y=0! Aborting before voxelization.");
		std::fflush(nullptr); _exit(2);
	}
	print_info("Halbdomain-Check OK: Vehicle Y-bbox crosses Y=0");
#endif
#endif

	lbm.voxelize_mesh_on_device(vehicle, TYPE_S|TYPE_X); // Vehicle: TYPE_S|TYPE_X für object_force-Filter

	// ============== Boundaries (FluidX3D-Aero-Konvention) ==============
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz();
	parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if((lbm.flags[n] & TYPE_X) != 0u) return; // Vehicle nicht überschreiben
		if(z==0u) {                               // Boden Z=0: rolling road, TYPE_S moving wall +x
			lbm.flags[n] = TYPE_S;
			lbm.u.x[n] = lbm_u; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
#if CC6_MODE==2
		} else if(y==0u) {                        // CC#7: Y_min = TYPE_Y inline sym (FAILED, EP-storage)
			lbm.flags[n] = TYPE_Y;
			lbm.u.x[n] = lbm_u; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
#elif CC6_MODE==3
		} else if(y==0u) {                        // CC#7-Alt1 / V8 diagnostic: Y_min = TYPE_S|TYPE_Y Moving-Wall — TYPE_Y marker for isolated force measurement via object_force(TYPE_S|TYPE_Y)
			lbm.flags[n] = TYPE_S | TYPE_Y;
			lbm.u.x[n] = lbm_u; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
#elif CC6_MODE==4
		} else if(y==0u) {                        // CC#8: Y_min = TYPE_E|TYPE_Y Ghost-Cell-Mirror
			lbm.flags[n] = TYPE_E | TYPE_Y;
			lbm.u.x[n] = lbm_u; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f; // initial; kernel overrides w/ mirror per step
#elif CC6_MODE==5
		} else if(y==0u) {                        // CC#9: Y_min = TYPE_Y, processed by post-stream apply_freeslip_y kernel
			lbm.flags[n] = TYPE_Y;
			lbm.u.x[n] = lbm_u; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
#endif
		} else if(x==0u || x==Nx-1u || y==0u || y==Ny-1u || z==Nz-1u) { // Inlet/Outlet/Y_min(CC#6-Half:TYPE_E)/Y_max/Decke: TYPE_E free-stream
			lbm.flags[n] = TYPE_E;
			lbm.u.x[n] = lbm_u; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
		} else {                                  // Fluid: initial flow +x
			lbm.u.x[n] = lbm_u; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
		}
	});

#if CC6_MODE!=1
	// CC#9-V6 (NACH parallel_for!): Strip TYPE_X from vehicle cells at y=0 cut-surface.
	// In Halbdomain they spuriously contribute to object_force because their -y neighbor is
	// the TYPE_E top wall via periodic-wrap. Stripping TYPE_X excludes them from object_force.
	// Order matters: must run AFTER parallel_for to avoid relabeling-bug (V8 found this).
	parallel_for(Nx*Nz, [&](ulong i) {
		const uint x = (uint)(i % Nx), z = (uint)(i / Nx);
		const ulong n = (ulong)x + (ulong)0u*Nx + (ulong)z*Nx*Ny; // y=0 slice
		if((lbm.flags[n] & (TYPE_S|TYPE_X)) == (TYPE_S|TYPE_X)) {
			lbm.flags[n] = TYPE_S; // strip TYPE_X, keep solid
		}
	});
#endif

#if CC7_DIAGNOSE && CC6_MODE==2
	// === CC#7 DIAGNOSE: count TYPE_Y, TYPE_S|TYPE_X, TYPE_E, TYPE_S(only) cells ===
	{ ulong cy=0, cv=0, ce=0, cs=0, cf=0;
	  for(ulong nn=0; nn<lbm.get_N(); nn++) {
	    const uchar fl = lbm.flags[nn];
	    if (fl & TYPE_Y) cy++;
	    else if ((fl & TYPE_X) && (fl & TYPE_S)) cv++;
	    else if (fl & TYPE_E) ce++;
	    else if (fl & TYPE_S) cs++;
	    else cf++;
	  }
	  print_info("CC#7-DIAG cells: TYPE_Y="+to_string(cy)+" Vehicle(S|X)="+to_string(cv)+" TYPE_E="+to_string(ce)+" TYPE_S(floor)="+to_string(cs)+" Fluid="+to_string(cf));
	  print_info("CC#7-DIAG expected TYPE_Y count: 1500*1*449 = 673500 (Y_min plane minus z=0 floor cells)");
	}
#endif

	// ============== Run-Schleife mit Auto-Stop bei Force-Konvergenz ==============
#if CC6_MODE==1
#if VEHICLE_GR_YARIS
	const string force_csv_path = get_exe_path()+"../bin/forces_cc10_yaris.csv";
#else
	const string force_csv_path = get_exe_path()+"../bin/forces_cc6_full.csv";
#endif
#elif CC6_MODE==2
	const string force_csv_path = get_exe_path()+"../bin/forces_cc7_half_sym.csv";
#elif CC6_MODE==3
	const string force_csv_path = get_exe_path()+"../bin/forces_cc7_alt1_moving.csv";
#elif CC6_MODE==4
	const string force_csv_path = get_exe_path()+"../bin/forces_cc8_ghost_mirror.csv";
#elif CC6_MODE==5
	const string force_csv_path = get_exe_path()+"../bin/forces_cc9_poststream_freeslip.csv";
#else
	const string force_csv_path = get_exe_path()+"../bin/forces_cc6_half.csv";
#endif
	std::ofstream fcsv(force_csv_path);
#if CC6_MODE==3
	fcsv << "step,t_si,Fx_si,Fy_si,Fz_si,Fx_sym,Fy_sym,Fz_sym\n"; // CC#7-Alt1 V8: separate columns for sym-plane force isolated via TYPE_S|TYPE_Y
#else
	fcsv << "step,t_si,Fx_si,Fy_si,Fz_si\n";
#endif
	fcsv << std::scientific;

	const uint chunk = 100u;            // 1 chunk = 100 Steps
#if CC7_DIAGNOSE
	const uint chunks_max = CC7_DIAG_MAXSTEPS / chunk; // DIAGNOSE: nur kurzer Run
#else
	const uint chunks_max = 1000u;      // Hard cap = 100.000 Steps
#endif
	const uint conv_window = 25u;       // CC#X: 25 chunks = 2500 Steps Sliding-Window (was 50)
	const uint conv_min_chunks = 50u;   // CC#X: Frühester Exit = 50 chunks (5000 Steps), damit recent+prev je 25 chunks haben (was 100)
	const float conv_tol = 0.02f;       // 2 % relative Änderung (Fx only — Fy/Fz zu klein für rel. Konvergenz)
	std::vector<float3> Fhist;
	Fhist.reserve(chunks_max);
	print_info(label+" Run start: max "+to_string(chunks_max*chunk)+" Steps; Auto-Stop bei |dFx/Fx| < 2% über 5000 Steps (frühester Exit nach 10000 Steps)");
#if SELF_COUPLING_TEST
	// Phase 5b-pre: self-coupling at x=200 mid-domain plane (away from vehicle x=400-800)
	// If invariant: pipeline works → Phase 5b can proceed with confidence
	PlaneSpec self_plane;
	self_plane.origin = uint3(200u, 0u, 0u);
	self_plane.extent_a = lbm.get_Ny();
	self_plane.extent_b = lbm.get_Nz();
	self_plane.axis = 0u; // X-normal plane
	self_plane.cell_size = (float)(si_length / (float)lbm_length);
	CouplingOptions self_opts;
	self_opts.smooth_plane = false; // no smoothing for self-coupling (test pipeline only)
	self_opts.export_csv = true;
	print_info("SELF_COUPLING_TEST ACTIVE: plane x=200, YZ-extent "+to_string(self_plane.extent_a)+"x"+to_string(self_plane.extent_b)+", every 5 chunks");
#endif
	uint final_chunks = chunks_max;
	for(uint c = 0u; c < chunks_max; c++) {
		lbm.run(chunk);
#if SELF_COUPLING_TEST
		if((c+1u) % 5u == 0u) { // every 500 steps
			lbm.couple_fields_self(self_plane, self_opts);
		}
#endif
		lbm.update_force_field();
		const float3 F_lbm = lbm.object_force(TYPE_S|TYPE_X);
		const float3 F_si  = float3(units.si_F(F_lbm.x), units.si_F(F_lbm.y), units.si_F(F_lbm.z));
		Fhist.push_back(F_si);
		const ulong step = (ulong)(c+1u) * chunk;
		const float t_si = units.si_t(step);
#if CC6_MODE==3
		// V8 diagnostic: separately measure force on TYPE_S|TYPE_Y sym-plane cells
		const float3 F_sym_lbm = lbm.object_force(TYPE_S|TYPE_Y);
		const float3 F_sym_si  = float3(units.si_F(F_sym_lbm.x), units.si_F(F_sym_lbm.y), units.si_F(F_sym_lbm.z));
		fcsv << step << "," << t_si << "," << F_si.x << "," << F_si.y << "," << F_si.z << "," << F_sym_si.x << "," << F_sym_si.y << "," << F_sym_si.z << "\n";
#else
		fcsv << step << "," << t_si << "," << F_si.x << "," << F_si.y << "," << F_si.z << "\n";
#endif
		if((c+1u) % 25u == 0u) fcsv.flush();
		if((c+1u) % 50u == 0u) {
			print_info("step="+to_string(step)+" t="+to_string(t_si, 3u)+"s  Fx="+to_string(F_si.x, 1u)+"N  Fy="+to_string(F_si.y, 1u)+"N  Fz="+to_string(F_si.z, 1u)+"N");
		}
		// Convergence test (Fx only — Fy/Fz im Volldomain nahe Null und unkonvergierbar)
		if(c+1u >= conv_min_chunks) {
			float3 recent_avg(0.0f), prev_avg(0.0f);
			for(uint k=0u; k<conv_window; k++) recent_avg += Fhist[Fhist.size()-1u-k];
			for(uint k=0u; k<conv_window; k++) prev_avg   += Fhist[Fhist.size()-1u-conv_window-k];
			recent_avg /= (float)conv_window;
			prev_avg   /= (float)conv_window;
			const float dx = (recent_avg.x!=0.0f) ? fabs(recent_avg.x - prev_avg.x) / fabs(recent_avg.x) : 1.0f;
			if(dx < conv_tol) {
				print_info(label+" CONVERGED at step "+to_string(step)+": |dFx|/|Fx|="+to_string(dx*100.0f, 3u)+"%  Fx_avg="+to_string(recent_avg.x, 1u)+"N  Fz_avg="+to_string(recent_avg.z, 1u)+"N");
				final_chunks = c+1u;
				break;
			}
		}
	}
	fcsv.close();

	// ============== Final VTK-Snapshot ==============
	const string export_path = get_exe_path()+"../export/";
	lbm.u.write_device_to_vtk(export_path);
	lbm.rho.write_device_to_vtk(export_path);
	lbm.flags.write_device_to_vtk(export_path);
	lbm.F.write_device_to_vtk(export_path);
	lbm.write_mesh_to_vtk(vehicle, export_path);
	print_info(label+" done after "+to_string(final_chunks)+" chunks ("+to_string(final_chunks*chunk)+" steps). Force-CSV: "+force_csv_path+" | VTKs: "+export_path);
	std::fflush(nullptr);
	_exit(0); // umgehe xe-driver Cleanup-Race
} // CC#6 BLOCK END

#if PHASE_5B_DUAL_DOMAIN
// ============================================================================
// Phase 5b: Dual-Domain Same-Resolution Schwarz Coupling — 2026-05-13
// Two LBM instances on single GPU:
//   Far:  1000 × 500 × 450 = 225 M cells (10m × 5m × 4.5m @ 10mm, shrunk per user)
//   Near:  700 × 300 × 185 = 38.85 M cells (compact, 0.5m vor/sides/oben + 2m hinter vehicle)
// Near origin in Far coords: (75, 100, 0)
// Vehicle: Far cell (350, 250, ~70) | Near cell (275, 150, ~70) — same physical position
// Coupling: Far → Near at 5 outer faces every chunk (no back-coupling in 5b-A)
// Blockage: 2.0/22.5 = 8.9% ✓
// ============================================================================
void main_setup_phase5b_dual() {
	// ===== Common physics =====
	const float lbm_u    = 0.075f;
	const float si_u     = 30.0f;
	const float si_length= 4.5f;     // vehicle length (m), scales STL to lbm_length
	const float cell_size= 0.010f;   // 10 mm uniform — SAME for Far and Near (5b same-res)
	const float lbm_length = si_length / cell_size; // 450 cells per vehicle length
	const float si_nu = 1.48E-5f, si_rho = 1.225f;
	units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);

	print_info("==================== PHASE 5b: DUAL-DOMAIN SAME-RES SCHWARZ ====================");
	print_info("Re(L) = "+to_string(to_uint(units.si_Re(si_length, si_u, si_nu))));

	// ===== Far Domain (shrunk for faster runs) =====
	const uint3 far_N = uint3(1000u, 500u, 450u);  // 225 M cells | 10m × 5m × 4.5m
	const uint far_vehicle_x = 350u;               // Vehicle X-center | anlauf 125 cells (1.25m), wake 425 cells (4.25m)
	print_info("Far: "+to_string(far_N.x)+"x"+to_string(far_N.y)+"x"+to_string(far_N.z)+" = "+to_string((ulong)far_N.x*far_N.y*far_N.z)+" cells | Vehicle X-center @ cell "+to_string(far_vehicle_x));
	LBM lbm_far(far_N, 1u, 1u, 1u, lbm_nu);

	// ===== Near Domain (compact per user spec 2026-05-13) =====
	const uint3 near_N = uint3(700u, 300u, 185u);  // 38.85 M cells | 7m × 3m × 1.85m
	const uint3 near_origin = uint3(75u, 100u, 0u); // Near (0,0,0) = Far (75, 100, 0)
	const uint near_vehicle_x = far_vehicle_x - near_origin.x; // 275
	print_info("Near: "+to_string(near_N.x)+"x"+to_string(near_N.y)+"x"+to_string(near_N.z)+" = "+to_string((ulong)near_N.x*near_N.y*near_N.z)+" cells | Origin in Far: ("+to_string(near_origin.x)+","+to_string(near_origin.y)+","+to_string(near_origin.z)+") | Vehicle X-center @ Near cell "+to_string(near_vehicle_x));
	print_info("Total Far+Near = "+to_string((ulong)far_N.x*far_N.y*far_N.z + (ulong)near_N.x*near_N.y*near_N.z)+" cells | FP16C ~67 bytes/cell");
	LBM lbm_near(near_N, 1u, 1u, 1u, lbm_nu);

	// ===== Vehicle: voxelize in both domains =====
	Mesh* vehicle_far = read_stl(get_exe_path()+"../scenes/vehicle.stl"); // canonical MR2 binary
	const float3 bbox_orig = vehicle_far->get_bounding_box_size();
	vehicle_far->scale(lbm_length / bbox_orig.x); // scale STL X-axis to 450 cells = 4.5m
	const float3 vbbox = vehicle_far->get_bounding_box_size();
	const float3 vctr  = vehicle_far->get_bounding_box_center();
	vehicle_far->translate(float3(
		(float)far_vehicle_x         - vctr.x,         // X-center @ cell 350
		(float)(far_N.y/2u)          - vctr.y,         // Y-center @ cell 250
		1.0f - (vctr.z - vbbox.z * 0.5f)));            // Z-min @ cell 1 (wheels on floor)
	const float3 vmin_f = vehicle_far->pmin, vmax_f = vehicle_far->pmax;
	print_info("Far Vehicle BBox: X["+to_string(vmin_f.x,1u)+","+to_string(vmax_f.x,1u)+"] Y["+to_string(vmin_f.y,1u)+","+to_string(vmax_f.y,1u)+"] Z["+to_string(vmin_f.z,1u)+","+to_string(vmax_f.z,1u)+"]");
	lbm_far.voxelize_mesh_on_device(vehicle_far, TYPE_S|TYPE_X);

	Mesh* vehicle_near = read_stl(get_exe_path()+"../scenes/vehicle.stl"); // separate Mesh* for Near
	vehicle_near->scale(lbm_length / bbox_orig.x);
	const float3 vctr2  = vehicle_near->get_bounding_box_center();
	const float3 vbbox2 = vehicle_near->get_bounding_box_size();
	vehicle_near->translate(float3(
		(float)near_vehicle_x        - vctr2.x,        // X-center @ Near cell 275
		(float)(near_N.y/2u)         - vctr2.y,        // Y-center @ Near cell 150
		1.0f - (vctr2.z - vbbox2.z * 0.5f)));          // Z-min @ Near cell 1
	const float3 vmin_n = vehicle_near->pmin, vmax_n = vehicle_near->pmax;
	print_info("Near Vehicle BBox: X["+to_string(vmin_n.x,1u)+","+to_string(vmax_n.x,1u)+"] Y["+to_string(vmin_n.y,1u)+","+to_string(vmax_n.y,1u)+"] Z["+to_string(vmin_n.z,1u)+","+to_string(vmax_n.z,1u)+"]");
	lbm_near.voxelize_mesh_on_device(vehicle_near, TYPE_S|TYPE_X);

	// ===== Boundaries: Far (CC#6-Full pattern) =====
	const uint NxF = lbm_far.get_Nx(), NyF = lbm_far.get_Ny(), NzF = lbm_far.get_Nz();
	parallel_for(lbm_far.get_N(), [&](ulong n) { uint x=0u,y=0u,z=0u; lbm_far.coordinates(n,x,y,z);
		if((lbm_far.flags[n] & TYPE_X) != 0u) return;
		if(z==0u) { lbm_far.flags[n] = TYPE_S; lbm_far.u.x[n]=lbm_u; lbm_far.u.y[n]=0.0f; lbm_far.u.z[n]=0.0f; }     // rolling road
		else if(x==0u || x==NxF-1u || y==0u || y==NyF-1u || z==NzF-1u) { lbm_far.flags[n]=TYPE_E; lbm_far.u.x[n]=lbm_u; lbm_far.u.y[n]=0.0f; lbm_far.u.z[n]=0.0f; }
		else { lbm_far.u.x[n]=lbm_u; lbm_far.u.y[n]=0.0f; lbm_far.u.z[n]=0.0f; }
	});

	// ===== Boundaries: Near — Floor TYPE_S, outer faces TYPE_E (will be overwritten by Far→Near coupling each chunk) =====
	const uint NxN = lbm_near.get_Nx(), NyN = lbm_near.get_Ny(), NzN = lbm_near.get_Nz();
	parallel_for(lbm_near.get_N(), [&](ulong n) { uint x=0u,y=0u,z=0u; lbm_near.coordinates(n,x,y,z);
		if((lbm_near.flags[n] & TYPE_X) != 0u) return;
		if(z==0u) { lbm_near.flags[n] = TYPE_S; lbm_near.u.x[n]=lbm_u; lbm_near.u.y[n]=0.0f; lbm_near.u.z[n]=0.0f; }   // SAME floor as Far
		else if(x==0u || x==NxN-1u || y==0u || y==NyN-1u || z==NzN-1u) { lbm_near.flags[n]=TYPE_E; lbm_near.u.x[n]=lbm_u; lbm_near.u.y[n]=0.0f; lbm_near.u.z[n]=0.0f; }
		else { lbm_near.u.x[n]=lbm_u; lbm_near.u.y[n]=0.0f; lbm_near.u.z[n]=0.0f; }
	});

	// ===== 5 Coupling Planes Far → Near (skip Z=0 row to preserve floor TYPE_S in Near) =====
	// axis: 0=YZ plane (X-normal), 1=XZ plane (Y-normal), 2=XY plane (Z-normal)
	// extent_a/extent_b run over the two non-axis dimensions in order (Y,Z) | (X,Z) | (X,Y)
	auto mk = [&](uint ox,uint oy,uint oz, uint ea,uint eb, uint ax){ PlaneSpec p; p.origin=uint3(ox,oy,oz); p.extent_a=ea; p.extent_b=eb; p.axis=ax; p.cell_size=cell_size; return p; };
	// X_min: Far x=75, Near x=0   | Y over [100..399] in Far / [0..299] in Near | Z over [1..184]
	const PlaneSpec src_xmin = mk(75u,  100u, 1u,   300u, 184u, 0u);  const PlaneSpec tgt_xmin = mk(0u,   0u,   1u, 300u, 184u, 0u);
	// X_max: Far x=775, Near x=699 (last interior cell)
	const PlaneSpec src_xmax = mk(775u, 100u, 1u,   300u, 184u, 0u);  const PlaneSpec tgt_xmax = mk(699u, 0u,   1u, 300u, 184u, 0u);
	// Y_min: Far y=100, Near y=0
	const PlaneSpec src_ymin = mk(75u,  100u, 1u,   700u, 184u, 1u);  const PlaneSpec tgt_ymin = mk(0u,   0u,   1u, 700u, 184u, 1u);
	// Y_max: Far y=400, Near y=299
	const PlaneSpec src_ymax = mk(75u,  400u, 1u,   700u, 184u, 1u);  const PlaneSpec tgt_ymax = mk(0u,   299u, 1u, 700u, 184u, 1u);
	// Z_max: Far z=184, Near z=184 (last cell, full XY of Near)
	const PlaneSpec src_zmax = mk(75u,  100u, 184u, 700u, 300u, 2u);  const PlaneSpec tgt_zmax = mk(0u,   0u,   184u, 700u, 300u, 2u);

	// ===== Back-coupling planes (Near → Far): inner overlap band, 5 cells INSIDE Near's outer boundary =====
	// Far targets are corresponding interior cells in Far (offset by Near origin + 5-cell band depth).
	// Verified: none of these planes overlap Far's vehicle (vehicle BBox X[125,575] Y[156,343] Z[1,123]).
	const uint band = 3u; // overlap-band thickness in cells (= 30 mm at 10mm/cell). 2026-05-13 final: thin band + α-blend stabilizes naive bidirectional. Earlier tests: band=5 → +22% drift, band=20 → +38% UNSTABLE (resonance). Trend: thinner = more stable.
	// X_min back: Near x=5 → Far x=80
	const PlaneSpec bk_src_xmin = mk(band,            0u,   1u, 300u, 184u, 0u);  const PlaneSpec bk_tgt_xmin = mk(75u+band,             100u, 1u, 300u, 184u, 0u);
	// X_max back: Near x=694 → Far x=770
	const PlaneSpec bk_src_xmax = mk(near_N.x-1u-band, 0u,   1u, 300u, 184u, 0u);  const PlaneSpec bk_tgt_xmax = mk(75u+near_N.x-1u-band,  100u, 1u, 300u, 184u, 0u);
	// Y_min back: Near y=5 → Far y=105
	const PlaneSpec bk_src_ymin = mk(0u,   band,            1u, 700u, 184u, 1u);  const PlaneSpec bk_tgt_ymin = mk(75u,  100u+band,             1u, 700u, 184u, 1u);
	// Y_max back: Near y=294 → Far y=394
	const PlaneSpec bk_src_ymax = mk(0u,   near_N.y-1u-band, 1u, 700u, 184u, 1u);  const PlaneSpec bk_tgt_ymax = mk(75u,  100u+near_N.y-1u-band,  1u, 700u, 184u, 1u);
	// Z_max back: Near z=179 → Far z=179
	const PlaneSpec bk_src_zmax = mk(0u,   0u,   near_N.z-1u-band, 700u, 300u, 2u);  const PlaneSpec bk_tgt_zmax = mk(75u,  100u, near_N.z-1u-band,   700u, 300u, 2u);

	CouplingOptions opts;        // forward Far→Near: hard overwrite (α=1) — Near's outer TYPE_E cells are designed-to-overwrite
	opts.smooth_plane = false;   // pipeline first, smoothing later
	opts.export_csv   = false;   // 5 planes × chunks = too many CSVs
	opts.alpha        = 1.0f;    // hard overwrite for forward (production setting Mode 1)
	CouplingOptions opts_back;   // back-coupling Near→Far: damped α=0.2 to prevent Far drift and resonance oscillation
	opts_back.smooth_plane = false;
	opts_back.export_csv   = false;
	opts_back.alpha        = 0.2f; // soft-BC blend: each chunk pulls Far's overlap cells only 20% toward Near's value

	// ===== Run-Loop with Far → Near coupling each chunk =====
#if PHASE_5B_COUPLE_MODE==0
	const string force_csv_path = get_exe_path()+"../bin/forces_phase5b_nocoupling.csv";
#elif PHASE_5B_COUPLE_MODE==2
	const string force_csv_path = get_exe_path()+"../bin/forces_phase5b_bidirectional_band3_alpha02.csv";
#else
	const string force_csv_path = get_exe_path()+"../bin/forces_phase5b_dual.csv";
#endif
	std::ofstream fcsv(force_csv_path);
	fcsv << "step,t_si,Fx_far,Fy_far,Fz_far,Fx_near,Fy_near,Fz_near\n";
	fcsv << std::scientific;
	const uint chunk = 100u;
	const uint chunks_max = 150u; // 15000 steps — convergence expected ~12-15k per Phase 5b-pre baseline; std±9% on Far alone
#if PHASE_5B_COUPLE_MODE==0
	print_info("Phase 5b Run: max "+to_string(chunks_max*chunk)+" steps | MODE 0 = NO coupling (Near alone, verification)");
#elif PHASE_5B_COUPLE_MODE==2
	print_info("Phase 5b Run: max "+to_string(chunks_max*chunk)+" steps | MODE 2 = BIDIRECTIONAL Schwarz (Far↔Near, 5 planes each direction, band="+to_string(band)+" cells)");
#else
	print_info("Phase 5b Run: max "+to_string(chunks_max*chunk)+" steps | MODE 1 = one-way Far→Near (5 planes)");
#endif
	std::vector<float3> Fhist_far, Fhist_near;
	Fhist_far.reserve(chunks_max); Fhist_near.reserve(chunks_max);

	for(uint c = 0u; c < chunks_max; c++) {
		// 1. Run Far for chunk
		lbm_far.run(chunk);
		// 2. Couple Far → Near at 5 outer faces (interleaved with Far's run, before Near runs on new BC)
#if PHASE_5B_COUPLE_MODE>=1
		lbm_far.couple_fields(lbm_near, src_xmin, tgt_xmin, opts);
		lbm_far.couple_fields(lbm_near, src_xmax, tgt_xmax, opts);
		lbm_far.couple_fields(lbm_near, src_ymin, tgt_ymin, opts);
		lbm_far.couple_fields(lbm_near, src_ymax, tgt_ymax, opts);
		lbm_far.couple_fields(lbm_near, src_zmax, tgt_zmax, opts);
#endif
		// 3. Run Near for chunk
		lbm_near.run(chunk);
		// 3b. Back-couple Near → Far at inner overlap band (Option C / full Schwarz alternating) — DAMPED (α<1) to stabilize
#if PHASE_5B_COUPLE_MODE>=2
		lbm_near.couple_fields(lbm_far, bk_src_xmin, bk_tgt_xmin, opts_back);
		lbm_near.couple_fields(lbm_far, bk_src_xmax, bk_tgt_xmax, opts_back);
		lbm_near.couple_fields(lbm_far, bk_src_ymin, bk_tgt_ymin, opts_back);
		lbm_near.couple_fields(lbm_far, bk_src_ymax, bk_tgt_ymax, opts_back);
		lbm_near.couple_fields(lbm_far, bk_src_zmax, bk_tgt_zmax, opts_back);
#endif
		// 4. Measure drag in both
		lbm_far.update_force_field();
		const float3 F_far_lbm = lbm_far.object_force(TYPE_S|TYPE_X);
		const float3 F_far_si  = float3(units.si_F(F_far_lbm.x), units.si_F(F_far_lbm.y), units.si_F(F_far_lbm.z));
		Fhist_far.push_back(F_far_si);
		lbm_near.update_force_field();
		const float3 F_near_lbm = lbm_near.object_force(TYPE_S|TYPE_X);
		const float3 F_near_si  = float3(units.si_F(F_near_lbm.x), units.si_F(F_near_lbm.y), units.si_F(F_near_lbm.z));
		Fhist_near.push_back(F_near_si);
		const ulong step = (ulong)(c+1u) * chunk;
		const float t_si = units.si_t(step);
		fcsv << step << "," << t_si << "," << F_far_si.x << "," << F_far_si.y << "," << F_far_si.z << "," << F_near_si.x << "," << F_near_si.y << "," << F_near_si.z << "\n";
		if((c+1u) % 10u == 0u) fcsv.flush();
		if((c+1u) % 25u == 0u) {
			print_info("step="+to_string(step)+"  Fx_far="+to_string(F_far_si.x,1u)+"N  Fx_near="+to_string(F_near_si.x,1u)+"N  delta="+to_string(F_near_si.x-F_far_si.x,1u)+"N");
		}
	}
	fcsv.close();
	print_info("Phase 5b done. Force-CSV: "+force_csv_path);
	std::fflush(nullptr);
	_exit(0); // xe-driver cleanup-race workaround
}
#endif // PHASE_5B_DUAL_DOMAIN

#if PHASE_5B_DR
// ============================================================================
// Phase 5b-DR: Double-Resolution Schwarz Coupling — 2026-05-14
// Pfad A (per user 2026-05-14 evening): Near at 5mm with slightly shrunk box for clean 3:1 ratio.
//   Far:  16m × 8m × 5m @ 15mm     = 1067 × 534 × 334 = 190 M cells  (Windkanal x=[-4,+12], y=[-4,+4], z=[0,+5])
//   Near:  6.6m × 2.7m × 1.695m @ 5mm = 1320 × 540 × 339 = 242 M cells (vehicle-centered, slightly tighter margins than user-spec)
// 3:1 resolution ratio → exact integer Near-in-Far alignment per axis (Near = 3 × Far overlap cells).
// Time-step sync: dt_far = 3 × dt_near (gleiche SI-Zeit pro Chunk: Far 100 steps, Near 300 steps).
// Vehicle bow @ x=0 SI. Near origin in Far cells: (233, 177, 0).
// Coupling Mode controlled by PHASE_5B_COUPLE_MODE (1=one-way Far→Near, 2=bidirectional with band=2 Near + α=0.2 back-coupling).
// PERF-D: batched PCIe sync (1 read + 1 write per direction) via opts.sync_pcie=false flag.
// ============================================================================
void main_setup_phase5b_dr() {
	// ===== Common physics =====
	const float lbm_u   = 0.075f;     // dimensionless LBM velocity (stability constraint, same for both LBMs)
	const float si_u    = 30.0f;      // 30 m/s freestream
	const float si_nu   = 1.48E-5f, si_rho = 1.225f; // air @ 15°C
	const float si_length = 4.5f;     // vehicle length [m]

	// ===== Cell sizes (5:1 ratio — Pfad A Aggressive 2026-05-16) =====
	const float dx_far  = 0.020f;     // Far cell-size: 20 mm
	const float dx_near = 0.004f;     // Near cell-size: 4 mm (5:1 ratio, exact clean integer mapping)

	// Per-LBM Units objects (global `units` will be set to Far for compatibility with other prints)
	Units units_far, units_near;
	const float lbm_length_far  = si_length / dx_far;   // 300 cells
	const float lbm_length_near = si_length / dx_near;  // 900 cells
	units_far .set_m_kg_s(lbm_length_far,  lbm_u, 1.0f, si_length, si_u, si_rho);
	units_near.set_m_kg_s(lbm_length_near, lbm_u, 1.0f, si_length, si_u, si_rho);
	units = units_far; // global units = Far's scale (used by print_info / generic conversions)
	const float lbm_nu_far  = units_far .nu(si_nu);
	const float lbm_nu_near = units_near.nu(si_nu);

	print_info("==================== PHASE 5b-DR: DOUBLE-RESOLUTION SCHWARZ (Pfad A 5:1 — Phase 4 Aggressive) ====================");
	print_info("Re(L) = "+to_string(to_uint(units_far.si_Re(si_length, si_u, si_nu))));
	print_info("Far cell = 20 mm, Near cell = 4 mm | ratio 5:1 | dt_far = 5 * dt_near");
#if PHASE_5B_COUPLE_MODE==2
	print_info("MODE 2: SEQUENTIAL bidirectional (Multiplicative Schwarz), band=2 Near, α=0.10 both directions, PERF-D batched sync");
#elif PHASE_5B_COUPLE_MODE==3
	print_info("MODE 3: CONCURRENT Far||Near (PERF-G Additive Schwarz, symmetric 1-chunk lag), Phase 6 Spiegelrampe-Blending 40-Cell-Band, α 0.05↔ALPHA_HIGH (Forward decay boundary→deep, Back rise boundary→deep)");
#else
	print_info("MODE 1: SEQUENTIAL one-way Far→Near (5 planes), α=0.5 forward soft-BC, PERF-D batched sync");
#endif

	// ===== Far Domain (13 × 6 × 4.5 m @ 20 mm) — Phase 4 Aggressive 2026-05-16 =====
	// Far world: X[-1, +12], Y[-3, +3], Z[0, +4.5]. Vehicle front @ x=0 → Far cell 50.
	const uint3 far_N = uint3(650u, 300u, 225u);  // 43.875 M cells | 13.0m × 6.0m × 4.5m @ 20mm | ~2.84 GB @ FP16C
	print_info("Far: "+to_string(far_N.x)+"x"+to_string(far_N.y)+"x"+to_string(far_N.z)+" = "+to_string((ulong)far_N.x*far_N.y*far_N.z)+" cells | 13.0m x 6.0m x 4.5m @ 20mm | ~2.84 GB @ FP16C");
	LBM lbm_far(far_N, 1u, 1u, 1u, lbm_nu_far);

	// ===== Near Domain (6.4 × 2.52 × 1.5 m @ 4 mm, exact 5:1 of 320 × 126 × 75 Far cells) =====
	// Near world: X[-0.4, +6.0], Y[-1.26, +1.26], Z[0, +1.5]. Vehicle front @ x=0 → Near cell 100.
	const uint3 near_N = uint3(1600u, 630u, 375u);  // 378.0 M cells | exact 5:1 mapping
	const uint3 near_origin = uint3(30u, 87u, 0u);  // Near (0,0,0) = Far (30, 87, 0) = physical (-0.4, -1.26, 0) m
	print_info("Near: "+to_string(near_N.x)+"x"+to_string(near_N.y)+"x"+to_string(near_N.z)+" = "+to_string((ulong)near_N.x*near_N.y*near_N.z)+" cells | 6.4m x 2.52m x 1.5m @ 4mm | ~24.5 GB");
	print_info("Near origin in Far cells: ("+to_string(near_origin.x)+","+to_string(near_origin.y)+","+to_string(near_origin.z)+") = (-0.4, -1.26, 0) m");
	print_info("Total Far+Near = "+to_string((ulong)far_N.x*far_N.y*far_N.z + (ulong)near_N.x*near_N.y*near_N.z)+" cells | total VRAM ~27.4 GB (Far 2.84 + Near 24.5)");
	LBM lbm_near(near_N, 1u, 1u, 1u, lbm_nu_near);

#ifdef WALL_MODEL_FLOOR
	// Path II.5 (2026-05-15): activate Floor WW wall model for both LBMs with rolling-road velocity = lbm_u
	lbm_far .wall_floor_u_road = lbm_u;
	lbm_near.wall_floor_u_road = lbm_u;
	print_info("WALL_MODEL_FLOOR active: Werner-Wengle floor wall model with rolling-road u_road = "+to_string(lbm_u,4u)+" LBM (= "+to_string(si_u,1u)+" m/s SI)");
#endif
// BOUZIDI_VEHICLE activation moved to AFTER voxelize_mesh_on_device (where flags are populated)


	// ===== Vehicle: voxelize in both domains at SAME physical position =====
	// Vehicle bow at x=0, Y-centered at y=0, Z-bottom at z=0
	Mesh* vehicle_far = read_stl(get_exe_path()+"../scenes/vehicle.stl"); // canonical MR2 binary
	const float3 bbox_orig = vehicle_far->get_bounding_box_size();
	vehicle_far->scale(lbm_length_far / bbox_orig.x); // scale STL X-extent to lbm_length_far cells (= 4.5m)
	const float3 vbbox_f = vehicle_far->get_bounding_box_size();
	const float3 vctr_f  = vehicle_far->get_bounding_box_center();
	// Far cell coords for vehicle (Phase 4 2026-05-16: Far origin at x=-1.0m, vehicle bow at x=0 → Far cell 50)
	const float far_vehicle_x_center_cell = (si_length * 0.5f) / dx_far + 1.0f / dx_far; // = 112.5 + 50 = 162.5 (vehicle X-center @ Far world x=2.25m)
	const float far_vehicle_y_center_cell = (float)far_N.y * 0.5f; // Y-center @ Far cell 150 = world y=0 centered (Far Y span [-3, +3])
	vehicle_far->translate(float3(
		far_vehicle_x_center_cell - vctr_f.x,
		far_vehicle_y_center_cell - vctr_f.y,
		1.0f - (vctr_f.z - vbbox_f.z * 0.5f)));     // Vehicle Far z=1 cell = 20mm clearance (1-cell offset, 5mm above old 15mm spec because new Far dx=20mm. Matches Near 5-cell offset at 4mm = 20mm)
	const float3 vmin_f = vehicle_far->pmin, vmax_f = vehicle_far->pmax;
	print_info("Far Vehicle BBox: X["+to_string(vmin_f.x,1u)+","+to_string(vmax_f.x,1u)+"] Y["+to_string(vmin_f.y,1u)+","+to_string(vmax_f.y,1u)+"] Z["+to_string(vmin_f.z,1u)+","+to_string(vmax_f.z,1u)+"] (Far cells)");
	lbm_far.voxelize_mesh_on_device(vehicle_far, TYPE_S|TYPE_X);

	Mesh* vehicle_near = read_stl(get_exe_path()+"../scenes/vehicle.stl");
	vehicle_near->scale(lbm_length_near / bbox_orig.x); // scale STL X-extent to lbm_length_near cells (= 4.5m at 5mm = 900 cells)
	const float3 vbbox_n = vehicle_near->get_bounding_box_size();
	const float3 vctr_n  = vehicle_near->get_bounding_box_center();
	// Near coords (Phase 4 2026-05-16): Near origin at SI (-0.4, -1.26, 0) m
	// Vehicle bow at SI x=0 → Near cell (0-(-0.4))/0.004 = 100. Vehicle X-center at SI 2.25m → Near cell (2.25-(-0.4))/0.004 = 662.5
	// Vehicle Y-center at SI y=0 → Near cell (0-(-1.26))/0.004 = 315
	const float near_vehicle_x_center_cell = (0.0f - (-0.4f)) / dx_near + (si_length * 0.5f) / dx_near; // 100 + 562.5 = 662.5
	const float near_vehicle_y_center_cell = (0.0f - (-1.26f)) / dx_near; // 1.26 / 0.004 = 315
	vehicle_near->translate(float3(
		near_vehicle_x_center_cell - vctr_n.x,
		near_vehicle_y_center_cell - vctr_n.y,
		5.0f - (vctr_n.z - vbbox_n.z * 0.5f)));  // Vehicle Near z=5 cell = 20mm clearance (matches Far cell 1 at 20mm via 5:1 ratio)
	const float3 vmin_n = vehicle_near->pmin, vmax_n = vehicle_near->pmax;
	print_info("Near Vehicle BBox: X["+to_string(vmin_n.x,1u)+","+to_string(vmax_n.x,1u)+"] Y["+to_string(vmin_n.y,1u)+","+to_string(vmax_n.y,1u)+"] Z["+to_string(vmin_n.z,1u)+","+to_string(vmax_n.z,1u)+"] (Near cells)");
	lbm_near.voxelize_mesh_on_device(vehicle_near, TYPE_S|TYPE_X);

#ifdef BOUZIDI_VEHICLE
	// 2026-05-15: Sparse Bouzidi infrastructure preserved as reusable building-block. DISABLED.
	// compute_bouzidi_cells_active was used by WALL_SLIP V1/V2 — both failed.
	// Bouzidi kernel itself fails in EP-pull layout (CC#7-style neutralization). Code stays for reference.
#endif
// WALL_VISC_BOOST populate_wall_adj_flag moved to AFTER BC loops (line ~998+) — required so floor TYPE_S is set first
#ifdef WALL_SLIP_VEHICLE
	// 2026-05-15: BOTH V1 (full overwrite) AND V2 (BLEND) FAILED.
	// V1: catastrophic -200k N (turbulence killed by full DDF=feq overwrite)
	// V2: f_neq decay → Fx=153 N at chunk 5 (90% f_neq preservation insufficient, simple blending mathematically
	//     decays non-equilibrium content over time, confirmed by research finding).
	// Research-Erkenntnis: Malaspinas-Sagaut 2014 requires PHYSICAL f_neq reconstruction from wall stress,
	// not simple blending. Simpler EP-pull-compatible Pfad: local viscosity modification (Smagorinsky-Extension).
	// lbm_far.wall_slip_factor = 0.7f; lbm_near.wall_slip_factor = 0.7f;
	// lbm_far.wall_slip_blend  = 0.1f; lbm_near.wall_slip_blend  = 0.1f;
#endif

	// ===== Far Boundaries: TYPE_S Moving-Wall Floor (Rolling Road) + Wheel-contact-patches =====
	// 2026-05-16: REVERT TYPE_E → TYPE_S floor. ParaView Inspection 2026-05-15 zeigte: TYPE_E floor verhindert Venturi/Ground-Effect
	// unter Splitter (Fz_near +1270N LIFT statt -1200N DOWNFORCE). TYPE_S Moving Wall mit u_x=lbm_u erlaubt Air-Acceleration
	// unter dem Fahrzeug (Pressure-Drop → Downforce). Wheel-contact-fix (z==0 && TYPE_X) bleibt — verhindert Discontinuity.
	const uint NxF = lbm_far.get_Nx(), NyF = lbm_far.get_Ny(), NzF = lbm_far.get_Nz();
	parallel_for(lbm_far.get_N(), [&](ulong n) { uint x=0u,y=0u,z=0u; lbm_far.coordinates(n,x,y,z);
		if((lbm_far.flags[n] & TYPE_X) != 0u) {                                                      // Vehicle TYPE_S|TYPE_X cells
			if(z==0u) lbm_far.u.x[n] = lbm_u;                                                        //   Wheel contact patches at z=0: move with road (lbm_u, prevent discontinuity)
			return;
		}
		if(z==0u) { lbm_far.flags[n] = TYPE_S; lbm_far.u.x[n]=lbm_u; lbm_far.u.y[n]=0.0f; lbm_far.u.z[n]=0.0f; }  // TYPE_S Moving Wall floor (Rolling Road)
		else if(x==0u || x==NxF-1u || y==0u || y==NyF-1u || z==NzF-1u) { lbm_far.flags[n]=TYPE_E; lbm_far.u.x[n]=lbm_u; lbm_far.u.y[n]=0.0f; lbm_far.u.z[n]=0.0f; }
		else { lbm_far.u.x[n]=lbm_u; lbm_far.u.y[n]=0.0f; lbm_far.u.z[n]=0.0f; }
	});

	// ===== Near Boundaries: same pattern (TYPE_S Moving Wall floor) =====
	const uint NxN = lbm_near.get_Nx(), NyN = lbm_near.get_Ny(), NzN = lbm_near.get_Nz();
	parallel_for(lbm_near.get_N(), [&](ulong n) { uint x=0u,y=0u,z=0u; lbm_near.coordinates(n,x,y,z);
		if((lbm_near.flags[n] & TYPE_X) != 0u) {
			if(z==0u) lbm_near.u.x[n] = lbm_u;
			return;
		}
		if(z==0u) { lbm_near.flags[n] = TYPE_S; lbm_near.u.x[n]=lbm_u; lbm_near.u.y[n]=0.0f; lbm_near.u.z[n]=0.0f; }  // TYPE_S Moving Wall floor (Rolling Road)
		else if(x==0u || x==NxN-1u || y==0u || y==NyN-1u || z==NzN-1u) { lbm_near.flags[n]=TYPE_E; lbm_near.u.x[n]=lbm_u; lbm_near.u.y[n]=0.0f; lbm_near.u.z[n]=0.0f; }
		else { lbm_near.u.x[n]=lbm_u; lbm_near.u.y[n]=0.0f; lbm_near.u.z[n]=0.0f; }
	});

#ifdef WALL_VISC_BOOST
	// MOVED HERE (2026-05-15 fix): populate AFTER BC loops set floor TYPE_S, so floor-adjacency is detected.
	lbm_far .populate_wall_adj_flag(dx_far);   // Phase 5.1: explicit per-LBM dx_si — 20mm Far → 1 layer
	lbm_near.populate_wall_adj_flag(dx_near);  // Phase 5.1: explicit per-LBM dx_si —  4mm Near → 5 layers
	print_info("WALL_VISC_BOOST Phase 5.1 distance-based: Werner-Wengle + Prandtl mixing-length nu_t = κ·y·u_τ + Van Driest damping. Vehicle + Floor walls. Target BL depth ~20mm (auto-scaled per-domain to dx).");
#endif

	// ===== 5 Forward Coupling Planes Far → Near (bilinear 5:1 upsample, Phase 4 2026-05-16) =====
	// Near origin in Far cells: (30, 87, 0). Far inside Near: X 30..349 (320), Y 87..212 (126), Z 0..74 (75).
	// Skip Z=0 floor row (shared TYPE_S between domains). Coupling Z range: Far 1..74 (74), Near 1..370 (370) — top 4 Near cells uncoupled near TYPE_E top wall.
	auto mk = [&](uint ox,uint oy,uint oz, uint ea,uint eb, uint ax, float cs){ PlaneSpec p; p.origin=uint3(ox,oy,oz); p.extent_a=ea; p.extent_b=eb; p.axis=ax; p.cell_size=cs; return p; };
	// X_min: Far x=30, Near x=0. axis=0 → extent_a=Y (126→630), extent_b=Z (74→370)
	const PlaneSpec src_xmin = mk(30u,  87u,  1u,   126u, 74u,  0u, dx_far);   const PlaneSpec tgt_xmin = mk(0u,    0u,   1u, 630u, 370u, 0u, dx_near);
	// X_max: Far x=349 (30+320-1), Near x=1599
	const PlaneSpec src_xmax = mk(349u, 87u,  1u,   126u, 74u,  0u, dx_far);   const PlaneSpec tgt_xmax = mk(1599u, 0u,   1u, 630u, 370u, 0u, dx_near);
	// Y_min: Far y=87, Near y=0. axis=1 → extent_a=X (320→1600), extent_b=Z (74→370)
	const PlaneSpec src_ymin = mk(30u,  87u,  1u,   320u, 74u,  1u, dx_far);   const PlaneSpec tgt_ymin = mk(0u,    0u,   1u, 1600u, 370u, 1u, dx_near);
	// Y_max: Far y=212 (87+126-1), Near y=629
	const PlaneSpec src_ymax = mk(30u,  212u, 1u,   320u, 74u,  1u, dx_far);   const PlaneSpec tgt_ymax = mk(0u,    629u, 1u, 1600u, 370u, 1u, dx_near);
	// Z_max: Far z=74 (top of Near's Z extent), Near z=374. axis=2 → extent_a=X (320→1600), extent_b=Y (126→630)
	const PlaneSpec src_zmax = mk(30u,  87u,  74u,  320u, 126u, 2u, dx_far);   const PlaneSpec tgt_zmax = mk(0u,    0u,   374u, 1600u, 630u, 2u, dx_near);

#if PHASE_5B_COUPLE_MODE==2 || PHASE_5B_COUPLE_MODE==3
	// ===== 5 Back-Coupling Planes Near → Far (bilinear 5:1 DOWNsample, Phase 4 2026-05-16) =====
	// band = 2 Near cells (8mm physical depth) inside Near's outer boundary. Far target: 1 Far cell INSIDE Near's overlap.
	const uint bn = 2u; // band depth in Near cells
	// X_min back: src Near x=2, tgt Far x=31 (Near boundary Far cell 30 + 1)
	const PlaneSpec bk_src_xmin = mk(bn,             0u,             1u, 630u,  370u, 0u, dx_near); const PlaneSpec bk_tgt_xmin = mk(31u,  87u,  1u, 126u, 74u,  0u, dx_far);
	// X_max back: src Near x=1597, tgt Far x=348 (Near boundary Far cell 349 - 1)
	const PlaneSpec bk_src_xmax = mk(near_N.x-1u-bn, 0u,             1u, 630u,  370u, 0u, dx_near); const PlaneSpec bk_tgt_xmax = mk(348u, 87u,  1u, 126u, 74u,  0u, dx_far);
	// Y_min back: src Near y=2, tgt Far y=88
	const PlaneSpec bk_src_ymin = mk(0u,             bn,             1u, 1600u, 370u, 1u, dx_near); const PlaneSpec bk_tgt_ymin = mk(30u,  88u,  1u, 320u, 74u,  1u, dx_far);
	// Y_max back: src Near y=627, tgt Far y=211
	const PlaneSpec bk_src_ymax = mk(0u,             near_N.y-1u-bn, 1u, 1600u, 370u, 1u, dx_near); const PlaneSpec bk_tgt_ymax = mk(30u,  211u, 1u, 320u, 74u,  1u, dx_far);
	// Z_max back: src Near z=372, tgt Far z=73
	const PlaneSpec bk_src_zmax = mk(0u,             0u,             near_N.z-1u-bn, 1600u, 630u, 2u, dx_near); const PlaneSpec bk_tgt_zmax = mk(30u,  87u,  73u, 320u, 126u, 2u, dx_far);
#endif // PHASE_5B_COUPLE_MODE == 2 || 3

	// Coupling options — SYMMETRIC α-sweep test 2026-05-15 (with pre-read fix on lines 1031-1033)
	CouplingOptions opts;
	opts.smooth_plane = false;
	opts.export_csv   = false;
#if PHASE_5B_COUPLE_MODE==2
	opts.alpha        = 0.10f;       // SYMMETRIC forward α=0.10 (validated 2026-05-15 α-sweep: stable, moderate convergence; 0.33 oscillated, 0.20 had Fz swings)
#elif PHASE_5B_COUPLE_MODE==3
	opts.alpha        = 0.20f;       // 2026-05-15 EOD: α=0.20 als final (User-Direktive nach Mode 3 Production sichtung). Begründung: stärkere Coupling → mehr Wirbel-Transport Near→Far ("Far profitiert von Near's Wirbelauflösung"). α=0.20 24% schneller konvergiert + ähnliche Final-Forces wie α=0.15.
#else
	opts.alpha        = 0.5f;        // 2026-05-15 Mode 1 test: soft-BC α=0.5 forward (statt 1.0 hard overwrite) — User-Vorschlag nach Mode 2 α=0.10 Near-Strömung unplausibel
#endif
	opts.sync_pcie    = false;       // PERF-D: caller batches PCIe sync
#if PHASE_5B_COUPLE_MODE==2 || PHASE_5B_COUPLE_MODE==3
	CouplingOptions opts_back;
	opts_back.smooth_plane = false;
	opts_back.export_csv   = false;
	opts_back.alpha        = 0.20f;  // 2026-05-15 EOD: SYMMETRIC back α=0.20 (matches forward, stronger coupling for vortex transport Near→Far)
	opts_back.sync_pcie    = false;
#endif

	// ===== Run-Loop with time-step sync + PERF-D batched PCIe sync =====
	// Far 100 steps, Near 300 steps per chunk (3:1 ratio → same SI Δt)
#if PHASE_5B_COUPLE_MODE==2
	const string force_csv_path = get_exe_path()+"../bin/forces_phase5b_dr_mode2.csv";
#elif PHASE_5B_COUPLE_MODE==3
	const string force_csv_path = get_exe_path()+"../bin/forces_phase5b_dr_mode3.csv";
#else
	const string force_csv_path = get_exe_path()+"../bin/forces_phase5b_dr.csv";
#endif
	std::ofstream fcsv(force_csv_path);
	fcsv << "step_far,t_si,Fx_far,Fy_far,Fz_far,Fx_near,Fy_near,Fz_near\n";
	fcsv << std::scientific;
	const uint chunk_far  = 100u;           // chunk=100 stable (chunk=25 caused feedback-loop in earlier Mode 2 test)
	const uint chunk_near = chunk_far * 5u; // Phase 4: dt_near = dt_far / 5 → Near needs 5× steps for same SI time at Pfad A 5:1 ratio (was ×3)
	const uint chunks_max = 500u;           // Production 2026-05-15: hard cap 50000 Far-steps, but convergence auto-stop typically at chunk 50-150
	// Convergence stop (per user 2026-05-15): 2% delta over 5000 Far-steps = 50 chunks
	const uint conv_window     = 25u;       // sliding window: 25 chunks = 2500 Far-steps
	const uint conv_min_chunks = 50u;       // earliest exit at chunk 50 (5000 Far-steps as user spec)
	const float conv_tol       = 0.02f;     // 2% relative Δ Fx_far for convergence
	std::vector<float3> Fhist_far;
	Fhist_far.reserve(chunks_max);
	print_info("Phase 5b-DR Run: max "+to_string(chunks_max*chunk_far)+" Far-steps | auto-stop bei |dFx_far|/|Fx_far|<2% über 5000 Far-steps (earliest exit chunk 50)");

	uint final_chunks = chunks_max;
	for(uint c = 0u; c < chunks_max; c++) {
#if PHASE_5B_COUPLE_MODE==3
		// ===== PERF-G: Concurrent Far + Near compute (Additive Schwarz, symmetric 1-chunk lag) =====
		// Beide LBMs laufen parallel auf ihren eigenen cl_queues. Boundary-Werte wurden am Ende des
		// vorigen Chunks gesetzt (oder bei c=0 von initial setup). Beide sehen Partner mit 1 chunk lag,
		// symmetric — entspricht klassischem Additive Schwarz im Zeit-Stepping. Coupling wird NACH dem
		// concurrent run angewandt und wirkt auf chunk c+1.
		// PERF-G impl: 1st chunk needs blocking init via run(); subsequent chunks use run_async() so
		// both LBMs queue all their kernels to their own cl_queue without per-step finish_queue,
		// allowing OpenCL driver to interleave compute on Intel DMA + EU engines.
		if(c == 0u) {
			// First chunk: blocking run() to trigger initialize() — sequential to avoid init race
			lbm_far .run(chunk_far );
			lbm_near.run(chunk_near);
		} else {
			lbm_far .run_async(chunk_far ); // queue 100 Far steps, return immediately
			lbm_near.run_async(chunk_near); // queue 300 Near steps, return immediately
			lbm_far .finish();              // wait for Far to complete (cl_queue barrier)
			lbm_near.finish();              // wait for Near to complete (separate queue)
		}

		// Beide jetzt bei t_{c+1}. Coupling mit fresh state beider Domains.
		// PERF-D: batched reads (1 read per array per LBM, deckt alle 10 coupling planes)
		lbm_far .u   .read_from_device();
		lbm_far .rho .read_from_device();
		lbm_far .flags.read_from_device();
		lbm_near.u   .read_from_device();
		lbm_near.rho .read_from_device();
		lbm_near.flags.read_from_device();

		// Phase 6C Blending 2026-05-16: 40-Cell-Spiegelrampe + 1-Far-Cell-Plateau am Boundary (User-Vorschlag).
		//   PLATEAU_NEAR = 5 Near-Cells = 1 Far-Cell breit (bei 5:1 Ratio) — α bleibt am Boundary-Wert konstant
		//   bevor die linear Rampe startet. Vermeidet Sub-Far-Cell-α-Gradienten und stabilisiert Coupling.
		//   Forward Far→Near: d=0..PLATEAU-1 → α=ALPHA_HIGH; d=PLATEAU..BAND-1 → linear ALPHA_HIGH→ALPHA_LOW
		//   Back Near→Far: dfar=0 (1 Far-Cell-Plateau) → α=ALPHA_LOW; dfar=1..BAND_FAR-1 → linear ALPHA_LOW→ALPHA_HIGH
		//   ALPHA_HIGH/LOW umschaltbar via Macro (Phase 6C: 0.05/0.50, Phase 6D test: 0.0/1.0)
		#ifndef PHASE6_ALPHA_HIGH
		#define PHASE6_ALPHA_HIGH 0.50f
		#endif
		#ifndef PHASE6_ALPHA_LOW
		#define PHASE6_ALPHA_LOW 0.05f
		#endif
		const float ALPHA_LOW  = PHASE6_ALPHA_LOW;
		const float ALPHA_HIGH = PHASE6_ALPHA_HIGH;
		const uint  BAND_NEAR    = 40u;          // Near cells = 160 mm physisch bei 4 mm dx
		const uint  BAND_FAR     = BAND_NEAR/5u; // 8 Far cells (5:1 Ratio)
		const uint  PLATEAU_NEAR = 5u;           // 1 Far-Cell = 5 Near-Cells Plateau am Boundary (forward)
		const uint  PLATEAU_FAR  = 1u;           // 1 Far-Cell Plateau am Boundary (back)

		// === FORWARD Far→Near: 5 Boundaries × 40 Near-Target-Tiefen mit 5-Near-Cell-Plateau ===
		for(uint d = 0u; d < BAND_NEAR; d++) {
			float alpha_d;
			if(d < PLATEAU_NEAR) {
				alpha_d = ALPHA_HIGH; // Plateau-Zone (5 Near-Cells = 1 Far-Cell breit) am Boundary
			} else {
				const uint d_in_ramp = d - PLATEAU_NEAR;
				const uint ramp_len  = BAND_NEAR - PLATEAU_NEAR - 1u; // 34
				const float t = (float)d_in_ramp / (float)ramp_len;    // 0..1
				alpha_d = ALPHA_HIGH - t * (ALPHA_HIGH - ALPHA_LOW);   // Ramp ALPHA_HIGH→ALPHA_LOW
			}
			CouplingOptions opts_d = opts;
			opts_d.alpha      = alpha_d;
			opts_d.keep_flags = (d > 0u);                                       // nur Layer 0 (Boundary) setzt TYPE_E
			const uint dsrc = d / 5u;                                           // Far-Cell shift (5:1)
			PlaneSpec tgt, src;
			// X_min: tgt deeper inward = origin.x increases
			tgt = tgt_xmin; tgt.origin.x = d;
			src = src_xmin; src.origin.x = src_xmin.origin.x + dsrc;
			lbm_far.couple_fields(lbm_near, src, tgt, opts_d);
			// X_max: tgt deeper inward = origin.x decreases
			tgt = tgt_xmax; tgt.origin.x = tgt_xmax.origin.x - d;
			src = src_xmax; src.origin.x = src_xmax.origin.x - dsrc;
			lbm_far.couple_fields(lbm_near, src, tgt, opts_d);
			// Y_min: tgt deeper = origin.y increases
			tgt = tgt_ymin; tgt.origin.y = d;
			src = src_ymin; src.origin.y = src_ymin.origin.y + dsrc;
			lbm_far.couple_fields(lbm_near, src, tgt, opts_d);
			// Y_max: tgt deeper = origin.y decreases
			tgt = tgt_ymax; tgt.origin.y = tgt_ymax.origin.y - d;
			src = src_ymax; src.origin.y = src_ymax.origin.y - dsrc;
			lbm_far.couple_fields(lbm_near, src, tgt, opts_d);
			// Z_max: tgt deeper = origin.z decreases
			tgt = tgt_zmax; tgt.origin.z = tgt_zmax.origin.z - d;
			src = src_zmax; src.origin.z = src_zmax.origin.z - dsrc;
			lbm_far.couple_fields(lbm_near, src, tgt, opts_d);
		}

		// === BACK Near→Far: 5 Boundaries × 8 Far-Target-Tiefen (Near-Source-Center pro 5-Cell-Block) mit 1-Far-Cell-Plateau ===
		for(uint dfar = 0u; dfar < BAND_FAR; dfar++) {
			float alpha_d;
			if(dfar < PLATEAU_FAR) {
				alpha_d = ALPHA_LOW; // Plateau-Zone (1 Far-Cell breit) am Boundary
			} else {
				const uint dfar_in_ramp = dfar - PLATEAU_FAR;
				const uint ramp_len     = BAND_FAR - PLATEAU_FAR - 1u; // 6
				const float t = (float)dfar_in_ramp / (float)ramp_len; // 0..1
				alpha_d = ALPHA_LOW + t * (ALPHA_HIGH - ALPHA_LOW);    // Ramp ALPHA_LOW→ALPHA_HIGH
			}
			CouplingOptions opts_d = opts_back;
			opts_d.alpha = alpha_d;
			// Near source depth: center of 5-cell block für dieses Far-Cell. dfar=0 → Near=2, dfar=7 → Near=37
			const uint dnear = 2u + 5u*dfar;
			PlaneSpec src, tgt;
			// X_min back
			src = bk_src_xmin; src.origin.x = dnear;
			tgt = bk_tgt_xmin; tgt.origin.x = 31u + dfar;
			lbm_near.couple_fields(lbm_far, src, tgt, opts_d);
			// X_max back
			src = bk_src_xmax; src.origin.x = near_N.x - 1u - dnear;
			tgt = bk_tgt_xmax; tgt.origin.x = 348u - dfar;
			lbm_near.couple_fields(lbm_far, src, tgt, opts_d);
			// Y_min back
			src = bk_src_ymin; src.origin.y = dnear;
			tgt = bk_tgt_ymin; tgt.origin.y = 88u + dfar;
			lbm_near.couple_fields(lbm_far, src, tgt, opts_d);
			// Y_max back
			src = bk_src_ymax; src.origin.y = near_N.y - 1u - dnear;
			tgt = bk_tgt_ymax; tgt.origin.y = 211u - dfar;
			lbm_near.couple_fields(lbm_far, src, tgt, opts_d);
			// Z_max back
			src = bk_src_zmax; src.origin.z = near_N.z - 1u - dnear;
			tgt = bk_tgt_zmax; tgt.origin.z = 73u - dfar;
			lbm_near.couple_fields(lbm_far, src, tgt, opts_d);
		}

		// PERF-D: batched writes
		lbm_near.flags.write_to_device();
		lbm_near.u    .write_to_device();
		lbm_near.rho  .write_to_device();
		lbm_far .flags.write_to_device();
		lbm_far .u    .write_to_device();
		lbm_far .rho  .write_to_device();
#else
		// ===== Modes 1 + 2: Sequential (Multiplicative Schwarz oder Forward-only) =====
		// 1. Run Far for chunk
		lbm_far.run(chunk_far);

		// 2. PERF-D: batched Far→Near forward coupling (1 read + 1 write instead of 5 each)
		lbm_far.u.read_from_device();         // sync Far src ONCE for all 5 forward planes
		lbm_far.rho.read_from_device();
		lbm_near.flags.read_from_device();    // sync Near tgt flags ONCE (forward writes TYPE_E)
#if PHASE_5B_COUPLE_MODE==2
		// Mode 2 symmetric: forward α=0.2 needs current Near u/rho for blending
		lbm_near.u.read_from_device();
		lbm_near.rho.read_from_device();
#endif
		lbm_far.couple_fields(lbm_near, src_xmin, tgt_xmin, opts);
		lbm_far.couple_fields(lbm_near, src_xmax, tgt_xmax, opts);
		lbm_far.couple_fields(lbm_near, src_ymin, tgt_ymin, opts);
		lbm_far.couple_fields(lbm_near, src_ymax, tgt_ymax, opts);
		lbm_far.couple_fields(lbm_near, src_zmax, tgt_zmax, opts);
		lbm_near.flags.write_to_device();     // sync Near tgt back to device ONCE
		lbm_near.u.write_to_device();
		lbm_near.rho.write_to_device();

		// 3. Run Near for 3× chunk steps (same SI Δt as Far)
		lbm_near.run(chunk_near);

#if PHASE_5B_COUPLE_MODE==2
		// 4. PERF-D: batched Near→Far back-coupling for Mode 2 (1 read each + 1 write Far)
		lbm_near.u.read_from_device();        // Near src u/rho (just changed by Near.run)
		lbm_near.rho.read_from_device();
		lbm_far.flags.read_from_device();     // Far tgt flags
		lbm_far.u.read_from_device();         // for α-blend, need current Far u/rho
		lbm_far.rho.read_from_device();
		lbm_near.couple_fields(lbm_far, bk_src_xmin, bk_tgt_xmin, opts_back);
		lbm_near.couple_fields(lbm_far, bk_src_xmax, bk_tgt_xmax, opts_back);
		lbm_near.couple_fields(lbm_far, bk_src_ymin, bk_tgt_ymin, opts_back);
		lbm_near.couple_fields(lbm_far, bk_src_ymax, bk_tgt_ymax, opts_back);
		lbm_near.couple_fields(lbm_far, bk_src_zmax, bk_tgt_zmax, opts_back);
		lbm_far.flags.write_to_device();
		lbm_far.u.write_to_device();
		lbm_far.rho.write_to_device();
#endif
#endif // PHASE_5B_COUPLE_MODE==3 vs sequential

		// 5. Force diagnostics (force_field already current after last LBM run on each domain)
		lbm_far.update_force_field();
		const float3 F_far_lbm = lbm_far.object_force(TYPE_S|TYPE_X);
		const float3 F_far_si  = float3(units_far.si_F(F_far_lbm.x), units_far.si_F(F_far_lbm.y), units_far.si_F(F_far_lbm.z));
		lbm_near.update_force_field();
		const float3 F_near_lbm = lbm_near.object_force(TYPE_S|TYPE_X);
		const float3 F_near_si  = float3(units_near.si_F(F_near_lbm.x), units_near.si_F(F_near_lbm.y), units_near.si_F(F_near_lbm.z));
		const ulong step = (ulong)(c+1u) * chunk_far;
		const float t_si = units_far.si_t(step);
		fcsv << step << "," << t_si << "," << F_far_si.x << "," << F_far_si.y << "," << F_far_si.z << "," << F_near_si.x << "," << F_near_si.y << "," << F_near_si.z << "\n";
		fcsv.flush();  // 2026-05-15: flush EVERY chunk so crash mid-run preserves data
		print_info("step_far="+to_string(step)+"  Fx_far="+to_string(F_far_si.x,1u)+"N  Fx_near="+to_string(F_near_si.x,1u)+"N  Fz_far="+to_string(F_far_si.z,1u)+"  Fz_near="+to_string(F_near_si.z,1u));
		Fhist_far.push_back(F_far_si);

		// Convergence test on Fx_far (per user 2026-05-15: 2% Delta over 5000 Far-steps = 50 chunks)
		if(c+1u >= conv_min_chunks) {
			float3 recent_avg(0.0f), prev_avg(0.0f);
			for(uint k=0u; k<conv_window; k++) recent_avg += Fhist_far[Fhist_far.size()-1u-k];
			for(uint k=0u; k<conv_window; k++) prev_avg   += Fhist_far[Fhist_far.size()-1u-conv_window-k];
			recent_avg /= (float)conv_window;
			prev_avg   /= (float)conv_window;
			const float dx = (recent_avg.x != 0.0f) ? fabs(recent_avg.x - prev_avg.x) / fabs(recent_avg.x) : 1.0f;
			if(dx < conv_tol) {
				print_info("CONVERGED at chunk "+to_string(c+1u)+" (step_far "+to_string(step)+"): |dFx_far|/|Fx_far|="+to_string(dx*100.0f, 3u)+"%  Fx_far_avg="+to_string(recent_avg.x, 1u)+"N");
				final_chunks = c+1u;
				break;
			}
		}
	}
	fcsv.close();
	print_info("Phase 5b-DR Run done after "+to_string(final_chunks)+" chunks ("+to_string(final_chunks*chunk_far)+" Far-steps)");

	// ===== VTK Export — both domains to separate subfolders for ParaView =====
	// In ParaView: open both dr_far/*.vtk AND dr_near/*.vtk; they auto-overlay correctly via STRUCTURED_POINTS SPACING (15mm vs 7.5mm) and ORIGIN (0,0,0 in lbm-cell coords).
	const string export_far  = get_exe_path()+"../export/dr_far/";
	const string export_near = get_exe_path()+"../export/dr_near/";
	lbm_far.u.write_device_to_vtk(export_far);
	lbm_far.rho.write_device_to_vtk(export_far);
	lbm_far.flags.write_device_to_vtk(export_far);
	lbm_far.F.write_device_to_vtk(export_far);
	lbm_near.u.write_device_to_vtk(export_near);
	lbm_near.rho.write_device_to_vtk(export_near);
	lbm_near.flags.write_device_to_vtk(export_near);
	lbm_near.F.write_device_to_vtk(export_near);
	lbm_far.write_mesh_to_vtk(vehicle_far, export_far);  // vehicle STL once (same physical mesh in both domains)
	print_info("Phase 5b-DR done. Force-CSV: "+force_csv_path);
	print_info("VTKs: Far = "+export_far+" | Near = "+export_near);
	std::fflush(nullptr);
	_exit(0); // xe-driver cleanup-race workaround
}
#endif // PHASE_5B_DR

#if 0 // ===== OLD CC#1/CC#3 main_setup (10000 step test, 16.85M cells), deactivated 2026-05-10 for CC#2 (50000 step Aero-Box) =====
void main_setup() { // WINDKANAL halbe Domain TestRes (CC#2): 100 steps, VTK + force-field export. Required: FP16C, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS, VOLUME_FORCE, FORCE_FIELD
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = uint3(650u, 144u, 180u); // halbierte Auflösung (von 1299x288x361): 16.85 Mio Zellen, jede Achse / 2
	const float lbm_u = 0.075f; // LBM velocity (standard)
	const float si_T = 15.0f; // Simulationszeit 15 Sekunden
	const float si_u = 30.0f; // 30 m/s Wind
	const float si_length = 4.0f; // Fahrzeuglänge: 4.0 m (echte Modell-Länge)
	const float cell_size = 4.5f / (float)lbm_N.x; // Box-Länge 4.5 m → Zellgröße ~3.46 mm
	const float lbm_length = si_length / cell_size; // 4.0 m / 3.46 mm ≈ 1156 Zellen Fahrzeuglänge
	const float si_width = 1.8f; // Fahrzeugbreite 1.8 m
	const float si_nu = 1.48E-5f, si_rho = 1.225f; // Luft bei 15°C
	units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
	const ulong lbm_T = units.t(si_T);
	print_info("Re (basierend auf Länge) = "+to_string(to_uint(units.si_Re(si_length, si_u, si_nu))));
	LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu);
	// ###################################################################################### define geometry ######################################################################################
	Mesh* vehicle = read_stl(get_exe_path()+"../scenes/vehicle.stl");
	// Skalierung entlang der X-Achse (Modell ist in X-Richtung orientiert)
	const float3 bbox = vehicle->get_bounding_box_size();
	const float vehicle_length_x = bbox.x;
	const float scale = lbm_length / vehicle_length_x; // mappe 4.0 m SI auf X-Länge des STL
	vehicle->scale(scale);
	// Positionierung: mittig im Windkanal (X und Y)
	const float ground_clearance_cells = 1.0f; // ~1 Zelle Bodenfreiheit
	vehicle->translate(float3(
		(0.5f)*(float)lbm_N.x - vehicle->get_bounding_box_center().x, // mittig in X-Richtung
		-2.0f * vehicle->get_bounding_box_center().y,                 // auf andere Seite der Symmetrieebene spiegeln (negative Y)
		ground_clearance_cells - vehicle->pmin.z));
	vehicle->set_center(vehicle->get_center_of_mass()); // Rotation um Schwerpunkt
	lbm.voxelize_mesh_on_device(vehicle);
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		// Boden: moving wall mit 30 m/s in +x (Fahrzeug fährt)
		if(z==0u) {
			lbm.flags[n] = TYPE_S;
			lbm.u.x[n] = lbm_u; // Bewegter Boden in Strömungsrichtung
			lbm.u.y[n] = 0.0f;
			lbm.u.z[n] = 0.0f;
		}
		if(z==Nz-1u) lbm.flags[n] = TYPE_S; // Decke: no-slip
		// Symmetrieebene bei y=0 (Slip-Bedingung), Wand bei y=Ny-1
		if(y==Ny-1u) lbm.flags[n] = TYPE_S; // Außenwand: no-slip
		// Inlet/Outlet auf x-Flächen
		if(x==0u||x==Nx-1u) lbm.flags[n] = TYPE_E; // Einlass/Auslass
		// Strömung in +x Richtung für Fluidzellen
		if(lbm.flags[n]!=TYPE_S) { lbm.u.x[n] = lbm_u; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f; }
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_Q_CRITERION; // Nur Q-Criterion (Wirbel), Wände ausgeblendet
	lbm.run(10000u); // 10000 Steps Test-Run, dann VTK + Force-Field-Export + CSV
	lbm.update_force_field(); // Kräfte auf TYPE_S (Fahrzeug + Boden + Decke) berechnen
	const string export_path = get_exe_path()+"../export/";
	lbm.u.write_device_to_vtk(export_path);          // velocity field
	lbm.rho.write_device_to_vtk(export_path);        // density field
	lbm.flags.write_device_to_vtk(export_path);      // cell type flags
	lbm.F.write_device_to_vtk(export_path);          // force field on solid boundaries
	lbm.write_mesh_to_vtk(vehicle, export_path);     // vehicle STL as VTK
	{ // CSV-Export der Forces auf Solid-Cells (für manuelle Auswertung)
		std::ofstream csv(export_path+"forces_solid_cells.csv");
		csv << "step,x,y,z,Fx_lbm,Fy_lbm,Fz_lbm,Fx_SI,Fy_SI,Fz_SI\n";
		const float Fconv = units.si_F(1.0f);
		const ulong N = lbm.get_N(); const uint Nx2=lbm.get_Nx(), Ny2=lbm.get_Ny(); ulong written = 0ull;
		for(ulong n=0ull; n<N; n++) {
			if(lbm.flags[n]==TYPE_S && (lbm.F.x[n]!=0.0f || lbm.F.y[n]!=0.0f || lbm.F.z[n]!=0.0f)) {
				const uint x = (uint)(n%Nx2), y = (uint)((n/Nx2)%Ny2), z = (uint)(n/((ulong)Nx2*Ny2));
				csv << lbm.get_t() << "," << x << "," << y << "," << z << ","
				    << lbm.F.x[n] << "," << lbm.F.y[n] << "," << lbm.F.z[n] << ","
				    << lbm.F.x[n]*Fconv << "," << lbm.F.y[n]*Fconv << "," << lbm.F.z[n]*Fconv << "\n";
				written++;
			}
		}
		csv.close();
		print_info("CSV-Force-Export: "+to_string(written)+" Solid-Cells mit F!=0");
	}
	print_info("VTK + Force-Field + CSV-Export abgeschlossen: "+export_path);
	std::fflush(nullptr);
	_exit(0); // umgehe xe-driver Cleanup-Race (bekannter -EINVAL fault); Daten sind bereits flushed
} /**/
#endif // OLD CC#1/CC#3 main_setup

/*void main_setup() { // WINDKANAL halbe Domain - andere Seite mit Symmetrieebene & Flussrichtung (+x); required extensions: FP16C, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	LBM lbm(128u, 128u, 128u, 1u, 1u, 1u, 0.01f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		const float A = 0.25f;
		const uint periodicity = 1u;
		const float a=(float)Nx/(float)periodicity, b=(float)Ny/(float)periodicity, c=(float)Nz/(float)periodicity;
		const float fx = (float)x+0.5f-0.5f*(float)Nx;
		const float fy = (float)y+0.5f-0.5f*(float)Ny;
		const float fz = (float)z+0.5f-0.5f*(float)Nz;
		lbm.u.x[n] =  A*cosf(2.0f*pif*fx/a)*sinf(2.0f*pif*fy/b)*sinf(2.0f*pif*fz/c);
		lbm.u.y[n] = -A*sinf(2.0f*pif*fx/a)*cosf(2.0f*pif*fy/b)*sinf(2.0f*pif*fz/c);
		lbm.u.z[n] =  A*sinf(2.0f*pif*fx/a)*sinf(2.0f*pif*fy/b)*cosf(2.0f*pif*fz/c);
		lbm.rho[n] = 1.0f-sq(A)*3.0f/4.0f*(cosf(4.0f*pif*fx/a)+cosf(4.0f*pif*fy/b));
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_STREAMLINES;
	lbm.run();
	//lbm.run(1000u); lbm.u.read_from_device(); println(lbm.u.x[lbm.index(Nx/2u, Ny/2u, Nz/2u)]); wait(); // test for binary identity
} /**/



/*void main_setup() { // 2D Taylor-Green vortices (use D2Q9); required extensions in defines.hpp: INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	LBM lbm(1024u, 1024u, 1u, 0.02f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		const float A = 0.2f;
		const uint periodicity = 5u;
		const float a=(float)Nx/(float)periodicity, b=(float)Ny/(float)periodicity;
		const float fx = (float)x+0.5f-0.5f*(float)Nx;
		const float fy = (float)y+0.5f-0.5f*(float)Ny;
		lbm.u.x[n] =  A*cosf(2.0f*pif*fx/a)*sinf(2.0f*pif*fy/b);
		lbm.u.y[n] = -A*sinf(2.0f*pif*fx/a)*cosf(2.0f*pif*fy/b);
		lbm.rho[n] = 1.0f-sq(A)*3.0f/4.0f*(cosf(4.0f*pif*fx/a)+cosf(4.0f*pif*fy/b));
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FIELD;
	lbm.graphics.slice_mode = 3;
	lbm.run();
} /**/



/*void main_setup() { // Poiseuille flow validation; required extensions in defines.hpp: VOLUME_FORCE
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint R = 63u; // channel radius (default: 63)
	const float umax = 0.1f; // maximum velocity in channel center (must be < 0.57735027f)
	const float tau = 1.0f; // relaxation time (must be > 0.5f), tau = nu*3+0.5
	const float nu = units.nu_from_tau(tau); // nu = (tau-0.5)/3
	const uint H = 2u*(R+1u);
#ifndef D2Q9
	LBM lbm(H, lcm(sq(H), WORKGROUP_SIZE)/sq(H), H, nu, 0.0f, units.f_from_u_Poiseuille_3D(umax, 1.0f, nu, R), 0.0f); // 3D
#else // D2Q9
	LBM lbm(lcm(H, WORKGROUP_SIZE)/H, H, 1u, nu, units.f_from_u_Poiseuille_2D(umax, 1.0f, nu, R), 0.0f, 0.0f); // 2D
#endif // D2Q9
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
#ifndef D2Q9
		if(!cylinder(x, y, z, lbm.center(), float3(0u, Ny, 0u), 0.5f*(float)min(Nx, Nz)-1.0f)) lbm.flags[n] = TYPE_S; // 3D
#else // D2Q9
		if(y==0u||y==Ny-1u) lbm.flags[n] = TYPE_S; // 2D
#endif // D2Q9
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	double error_min = max_double;
	while(true) { // main simulation loop
		lbm.run(1000u);
		lbm.u.read_from_device();
		double error_dif=0.0, error_sum=0.0;
#ifndef D2Q9
		for(uint x=0u; x<Nx; x++) {
			for(uint y=Ny/2u; y<Ny/2u+1u; y++) {
				for(uint z=0; z<Nz; z++) {
					const uint n = x+(y+z*Ny)*Nx;
					const double r = (double)sqrt(sq(x+0.5f-0.5f*(float)Nx)+sq(z+0.5f-0.5f*(float)Nz)); // radius from channel center
					if(r<R) {
						const double unum = (double)sqrt(sq(lbm.u.x[n])+sq(lbm.u.y[n])+sq(lbm.u.z[n])); // numerical velocity
						const double uref = umax*(sq(R)-sq(r))/sq(R); // theoretical velocity profile u = G*(R^2-r^2)
						error_dif += sq(unum-uref); // L2 error (Krüger p. 138)
						error_sum += sq(uref);
					}
				}
			}
		}
#else // D2Q9
		for(uint x=Nx/2u; x<Nx/2u+1u; x++) {
			for(uint y=1u; y<Ny-1u; y++) {
				const uint n = x+(y+0u*Ny)*Nx;
				const double r = (double)(y+0.5f-0.5f*(float)Ny); // radius from channel center
				const double unum = (double)sqrt(sq(lbm.u.x[n])+sq(lbm.u.y[n])); // numerical velocity
				const double uref = umax*(sq(R)-sq(r))/sq(R); // theoretical velocity profile u = G*(R^2-r^2)
				error_dif += sq(unum-uref); // L2 error (Krüger p. 138)
				error_sum += sq(uref);
			}
		}
#endif // D2Q9
		if(sqrt(error_dif/error_sum)>=error_min) { // stop when error has converged
			print_info("Poiseuille flow error converged after "+to_string(lbm.get_t())+" steps to "+to_string(100.0*error_min, 3u)+"%"); // typical expected L2 errors: 2-5% (Krüger p. 256)
			wait();
			exit(0);
		}
		error_min = fmin(error_min, sqrt(error_dif/error_sum));
		print_info("Poiseuille flow error after t="+to_string(lbm.get_t())+" is "+to_string(100.0*error_min, 3u)+"%"); // typical expected L2 errors: 2-5% (Krüger p. 256)
	}
} /**/



/*void main_setup() { // Stokes drag validation; required extensions in defines.hpp: FORCE_FIELD, EQUILIBRIUM_BOUNDARIES
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const ulong dt = 100ull; // check error every dt time steps
	const float R = 32.0f; // sphere radius
	const float Re = 0.01f; // Reynolds number
	const float nu = 1.0f; // kinematic shear viscosity
	const float rho = 1.0f; // density
	const uint L = to_uint(8.0f*R); // simulation box size
	const float u = units.u_from_Re(Re, 2.0f*R, nu); // velocity
	LBM lbm(L, L, L, nu); // flow driven by equilibrium boundaries
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E;
		if(sphere(x, y, z, lbm.center(), R)) {
			lbm.flags[n] = TYPE_S|TYPE_X; // flag boundary cells for force summation additionally with TYPE_X
		} else {
			lbm.rho[n] = units.rho_Stokes(lbm.position(x, y, z), float3(-u, 0.0f, 0.0f), R, rho, nu);
			const float3 un = units.u_Stokes(lbm.position(x, y, z), float3(-u, 0.0f, 0.0f), R);
			lbm.u.x[n] = un.x;
			lbm.u.y[n] = un.y;
			lbm.u.z[n] = un.z;
		}
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	double E1=1000.0, E2=1000.0;
	while(true) { // main simulation loop
		lbm.run(dt);
		const float3 force = lbm.object_force(TYPE_S|TYPE_X);
		const double F_theo = units.F_Stokes(rho, u, nu, R);
		const double F_sim = (double)length(force);
		const double E0 = fabs(F_sim-F_theo)/F_theo;
		print_info(to_string(lbm.get_t())+", expected: "+to_string(F_theo, 6u)+", measured: "+to_string(F_sim, 6u)+", error = "+to_string((float)(100.0*E0), 1u)+"%");
		if(converged(E2, E1, E0, 1E-4)) { // stop when error has sufficiently converged
			print_info("Error converged after "+to_string(lbm.get_t())+" steps to "+to_string(100.0*E0, 1u)+"%");
			wait();
			break;
		}
		E2 = E1;
		E1 = E0;
	}
} /**/



/*void main_setup() { // cylinder in rectangular duct; required extensions in defines.hpp: VOLUME_FORCE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const float Re = 25000.0f;
	const float D = 64.0f;
	const float u = rsqrt(3.0f);
	const float w=D, l=12.0f*D, h=3.0f*D;
	const float nu = units.nu_from_Re(Re, D, u);
	const float f = units.f_from_u_rectangular_duct(w, D, 1.0f, nu, u);
	LBM lbm(to_uint(w), to_uint(l), to_uint(h), nu, 0.0f, f, 0.0f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		lbm.u.y[n] = 0.1f*u;
		if(cylinder(x, y, z, float3(lbm.center().x, 2.0f*D, lbm.center().z), float3(Nx, 0u, 0u), 0.5f*D)) lbm.flags[n] = TYPE_S;
		if(x==0u||x==Nx-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_S; // x and z non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_Q_CRITERION;
	lbm.run();
} /**/



/*void main_setup() { // Taylor-Couette flow; required extensions in defines.hpp: MOVING_BOUNDARIES, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	LBM lbm(96u, 96u, 192u, 1u, 1u, 1u, 0.04f);
	// ###################################################################################### define geometry ######################################################################################
	const uint threads = (uint)thread::hardware_concurrency();
	vector<uint> seed(threads);
	for(uint t=0u; t<threads; t++) seed[t] = 42u+t;
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), threads, [&](ulong n, uint t) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(!cylinder(x, y, z, lbm.center(), float3(0u, 0u, Nz), (float)(Nx/2u-1u))) lbm.flags[n] = TYPE_S;
		if( cylinder(x, y, z, lbm.center(), float3(0u, 0u, Nz), (float)(Nx/4u   ))) {
			const float3 relative_position = lbm.relative_position(n);
			lbm.u.x[n] =  relative_position.y;
			lbm.u.y[n] = -relative_position.x;
			lbm.u.z[n] = (1.0f-random(seed[t], 2.0f))*0.001f;
			lbm.flags[n] = TYPE_S;
		}
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_STREAMLINES;
	lbm.run();
	//lbm.run(4000u); lbm.u.read_from_device(); println(lbm.u.x[lbm.index(Nx/4u, Ny/4u, Nz/2u)]); wait(); // test for binary identity
} /**/



/*void main_setup() { // lid-driven cavity; required extensions in defines.hpp: MOVING_BOUNDARIES, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint L = 128u;
	const float Re = 1000.0f;
	const float u = 0.1f;
	LBM lbm(L, L, L, units.nu_from_Re(Re, (float)(L-2u), u));
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(z==Nz-1) lbm.u.y[n] = u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_S; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_STREAMLINES;
	lbm.run();
} /**/



/*void main_setup() { // 2D Karman vortex street; required extensions in defines.hpp: D2Q9, FP16S, EQUILIBRIUM_BOUNDARIES, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint R = 16u;
	const float Re = 250.0f;
	const float u = 0.10f;
	LBM lbm(16u*R, 32u*R, 1u, units.nu_from_Re(Re, 2.0f*(float)R, u));
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(cylinder(x, y, z, float3(Nx/2u, Ny/4u, Nz/2u), float3(0u, 0u, Nz), (float)R)) lbm.flags[n] = TYPE_S;
		else lbm.u.y[n] = u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_FIELD;
	lbm.graphics.slice_mode = 3;
	lbm.run();
} /**/



/*void main_setup() { // particle test; required extensions in defines.hpp: VOLUME_FORCE, FORCE_FIELD, MOVING_BOUNDARIES, PARTICLES, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint L = 128u;
	const float Re = 1000.0f;
	const float u = 0.1f;
	LBM lbm(L, L, L, units.nu_from_Re(Re, (float)(L-2u), u), 0.0f, 0.0f, -0.00001f, cb(L/4u), 2.0f);
	// ###################################################################################### define geometry ######################################################################################
	uint seed = 42u;
	for(ulong n=0ull; n<lbm.particles->length(); n++) {
		lbm.particles->x[n] = random_symmetric(seed, 0.5f*lbm.size().x/4.0f);
		lbm.particles->y[n] = random_symmetric(seed, 0.5f*lbm.size().y/4.0f);
		lbm.particles->z[n] = random_symmetric(seed, 0.5f*lbm.size().z/4.0f);
	}
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(z==Nz-1) lbm.u.y[n] = u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_S; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_STREAMLINES|VIS_PARTICLES;
	lbm.run();
} /**/



/*void main_setup() { // delta wing; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint L = 128u;
	const float Re = 100000.0f;
	const float u = 0.075f;
	LBM lbm(L, 4u*L, L, units.nu_from_Re(Re, (float)L, u));
	// ###################################################################################### define geometry ######################################################################################
	const float3 offset = float3(lbm.center().x, 0.0f, lbm.center().z);
	const float3 p0 = offset+float3(  0*(int)L/64,  5*(int)L/64,  20*(int)L/64);
	const float3 p1 = offset+float3(-20*(int)L/64, 90*(int)L/64, -10*(int)L/64);
	const float3 p2 = offset+float3(+20*(int)L/64, 90*(int)L/64, -10*(int)L/64);
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(triangle(x, y, z, p0, p1, p2)) lbm.flags[n] = TYPE_S;
		else lbm.u.y[n] = u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
	lbm.run();
} /**/



/*void main_setup() { // NASA Common Research Model; required extensions in defines.hpp: FP16C, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 1.5f, 1.0f/3.0f), 2000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float Re = 10000000.0f;
	const float u = 0.075f;
	LBM lbm(lbm_N, units.nu_from_Re(Re, (float)lbm_N.x, u));
	// ###################################################################################### define geometry ######################################################################################
	// model: https://commonresearchmodel.larc.nasa.gov/high-lift-crm/high-lift-crm-geometry/assembled-geometry/, .stp file converted to .stl with https://imagetostl.com/convert/file/stp/to/stl
	Mesh* half = read_stl(get_exe_path()+"../stl/crm-hl_reference_ldg.stl", lbm.size(), float3(0.0f), float3x3(float3(0, 0, 1), radians(90.0f)), 1.0f*lbm_N.x);
	half->translate(float3(-0.5f*(half->pmax.x-half->pmin.x), 0.0f, 0.0f));
	Mesh* mesh = new Mesh(2u*half->triangle_number, float3(0.0f));
	for(uint i=0u; i<half->triangle_number; i++) {
		mesh->p0[i] = half->p0[i];
		mesh->p1[i] = half->p1[i];
		mesh->p2[i] = half->p2[i];
	}
	half->rotate(float3x3(float3(1, 0, 0), radians(180.0f))); // mirror-copy half
	for(uint i=0u; i<half->triangle_number; i++) {
		mesh->p0[half->triangle_number+i] = -half->p0[i];
		mesh->p1[half->triangle_number+i] = -half->p1[i];
		mesh->p2[half->triangle_number+i] = -half->p2[i];
	}
	delete half;
	mesh->find_bounds();
	mesh->rotate(float3x3(float3(1, 0, 0), radians(-10.0f)));
	mesh->translate(float3(0.0f, 0.0f, -0.5f*(mesh->pmin.z+mesh->pmax.z)));
	mesh->translate(float3(0.0f, -0.5f*lbm.size().y+mesh->pmax.y+0.5f*(lbm.size().x-(mesh->pmax.x-mesh->pmin.x)), 0.0f));
	mesh->translate(lbm.center());
	lbm.voxelize_mesh_on_device(mesh);
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
	lbm.run();
} /**/



/*void main_setup() { // Concorde; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 3.0f, 0.5f), 2084u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float si_u = 300.0f/3.6f;
	const float si_length=62.0f, si_width=26.0f;
	const float si_T = 1.0f;
	const float si_nu=1.48E-5f, si_rho=1.225f;
	const float lbm_length = 0.56f*(float)lbm_N.y;
	const float lbm_u = 0.075f;
	units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
	const ulong lbm_T = units.t(si_T);
	print_info("Re = "+to_string(to_uint(units.si_Re(si_width, si_u, si_nu))));
	LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu);
	// ###################################################################################### define geometry ######################################################################################
	const float3 center = float3(lbm.center().x, 0.52f*lbm_length, lbm.center().z+0.03f*lbm_length);
	const float3x3 rotation = float3x3(float3(1, 0, 0), radians(-10.0f))*float3x3(float3(0, 0, 1), radians(90.0f))*float3x3(float3(1, 0, 0), radians(90.0f));
	lbm.voxelize_stl(get_exe_path()+"../stl/concord_cut_large.stl", center, rotation, lbm_length); // https://www.thingiverse.com/thing:1176931/files
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
	lbm.run(0u, lbm_T); // initialize simulation
	lbm.write_status();
	while(lbm.get_t()<=lbm_T) { // main simulation loop
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
		if(lbm.graphics.next_frame(lbm_T, 10.0f)) {
			lbm.graphics.set_camera_free(float3(0.491343f*(float)Nx, -0.882147f*(float)Ny, 0.564339f*(float)Nz), -78.0f, 6.0f, 22.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/front/");
			lbm.graphics.set_camera_free(float3(1.133361f*(float)Nx, 1.407077f*(float)Ny, 1.684411f*(float)Nz), 72.0f, 12.0f, 20.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/back/");
			lbm.graphics.set_camera_centered(0.0f, 0.0f, 25.0f, 1.648722f);
			lbm.graphics.write_frame(get_exe_path()+"export/side/");
			lbm.graphics.set_camera_centered(0.0f, 90.0f, 25.0f, 1.648722f);
			lbm.graphics.write_frame(get_exe_path()+"export/top/");
			lbm.graphics.set_camera_free(float3(0.269361f*(float)Nx, -0.179720f*(float)Ny, 0.304988f*(float)Nz), -56.0f, 31.6f, 100.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/wing/");
			lbm.graphics.set_camera_free(float3(0.204399f*(float)Nx, 0.340055f*(float)Ny, 1.620902f*(float)Nz), 80.0f, 35.6f, 34.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/follow/");
		}
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
		lbm.run(1u, lbm_T); // run dt time steps
	}
	lbm.write_status();
} /**/



/*void main_setup() { // Boeing 747; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 0.5f), 880u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_Re = 1000000.0f;
	const float lbm_u = 0.075f;
	const ulong lbm_T = 10000ull;
	LBM lbm(lbm_N, units.nu_from_Re(lbm_Re, (float)lbm_N.x, lbm_u));
	// ###################################################################################### define geometry ######################################################################################
	const float size = 1.0f*lbm.size().x;
	const float3 center = float3(lbm.center().x, 0.55f*size, lbm.center().z);
	const float3x3 rotation = float3x3(float3(1, 0, 0), radians(-15.0f));
	lbm.voxelize_stl(get_exe_path()+"../stl/techtris_airplane.stl", center, rotation, size); // https://www.thingiverse.com/thing:2772812/files
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	lbm.graphics.set_camera_free(float3(1.0f*(float)Nx, -0.4f*(float)Ny, 2.0f*(float)Nz), -33.0f, 42.0f, 68.0f);
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<lbm_T) { // main simulation loop
		if(lbm.graphics.next_frame(lbm_T, 10.0f)) lbm.graphics.write_frame(); // render enough frames 10 seconds of 60fps video
		lbm.run(1u, lbm_T);
	}
#else // GRAPHICS && !INTERACTIVE_GRAPHICS
	lbm.run();
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
} /**/



/*void main_setup() { // Star Wars X-wing; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 0.5f), 880u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_Re = 100000.0f;
	const float lbm_u = 0.075f;
	const ulong lbm_T = 50000ull;
	LBM lbm(lbm_N, units.nu_from_Re(lbm_Re, (float)lbm_N.x, lbm_u));
	// ###################################################################################### define geometry ######################################################################################
	const float size = 1.0f*lbm.size().x;
	const float3 center = float3(lbm.center().x, 0.55f*size, lbm.center().z);
	const float3x3 rotation = float3x3(float3(0, 0, 1), radians(180.0f));
	lbm.voxelize_stl(get_exe_path()+"../stl/X-Wing.stl", center, rotation, size); // https://www.thingiverse.com/thing:353276/files
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<lbm_T) { // main simulation loop
		if(lbm.graphics.next_frame(lbm_T, 30.0f)) {
			lbm.graphics.set_camera_free(float3(1.0f*(float)Nx, -0.4f*(float)Ny, 2.0f*(float)Nz), -33.0f, 42.0f, 68.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/t/");
			lbm.graphics.set_camera_free(float3(0.5f*(float)Nx, -0.35f*(float)Ny, -0.7f*(float)Nz), -33.0f, -40.0f, 100.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/b/");
			lbm.graphics.set_camera_free(float3(0.0f*(float)Nx, 0.51f*(float)Ny, 0.75f*(float)Nz), 90.0f, 28.0f, 80.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/f/");
			lbm.graphics.set_camera_free(float3(0.7f*(float)Nx, -0.15f*(float)Ny, 0.06f*(float)Nz), 0.0f, 0.0f, 100.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/s/");
		}
		lbm.run(1u, lbm_T);
	}
#else // GRAPHICS && !INTERACTIVE_GRAPHICS
	lbm.run();
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
} /**/



/*void main_setup() { // Star Wars TIE fighter; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 1.0f), 1760u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_Re = 100000.0f;
	const float lbm_u = 0.075f;
	const ulong lbm_T = 50000ull;
	const ulong lbm_dt = 28ull;
	LBM lbm(lbm_N, units.nu_from_Re(lbm_Re, (float)lbm_N.x, lbm_u));
	// ###################################################################################### define geometry ######################################################################################
	const float size = 0.65f*lbm.size().x;
	const float3 center = float3(lbm.center().x, 0.6f*size, lbm.center().z);
	const float3x3 rotation = float3x3(float3(1, 0, 0), radians(90.0f));
	Mesh* mesh = read_stl(get_exe_path()+"../stl/DWG_Tie_Fighter_Assembled_02.stl", lbm.size(), center, rotation, size); // https://www.thingiverse.com/thing:2919109/files
	lbm.voxelize_mesh_on_device(mesh);
	lbm.flags.read_from_device();
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_FLAG_SURFACE|VIS_Q_CRITERION;
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<lbm_T) { // main simulation loop
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
		if(lbm.graphics.next_frame(lbm_T, 30.0f)) {
			lbm.graphics.set_camera_free(float3(1.0f*(float)Nx, -0.4f*(float)Ny, 0.63f*(float)Nz), -33.0f, 33.0f, 80.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/t/");
			lbm.graphics.set_camera_free(float3(0.3f*(float)Nx, -1.5f*(float)Ny, -0.45f*(float)Nz), -83.0f, -10.0f, 40.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/b/");
			lbm.graphics.set_camera_free(float3(0.0f*(float)Nx, 0.57f*(float)Ny, 0.7f*(float)Nz), 90.0f, 29.5f, 80.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/f/");
			lbm.graphics.set_camera_free(float3(2.5f*(float)Nx, 0.0f*(float)Ny, 0.0f*(float)Nz), 0.0f, 0.0f, 50.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/s/");
		}
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
		lbm.run(lbm_dt, lbm_T);
		const float3x3 rotation = float3x3(float3(0.2f, 1.0f, 0.1f), radians(0.4032f)); // create rotation matrix to rotate mesh
		lbm.unvoxelize_mesh_on_device(mesh);
		mesh->rotate(rotation); // rotate mesh
		lbm.voxelize_mesh_on_device(mesh);
	}
} /**/



/*void main_setup() { // radial fan; required extensions in defines.hpp: FP16S, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(3.0f, 3.0f, 1.0f), 181u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_Re = 100000.0f;
	const float lbm_u = 0.1f;
	const ulong lbm_T = 48000ull;
	const ulong lbm_dt = 10ull;
	LBM lbm(lbm_N, 1u, 1u, 1u, units.nu_from_Re(lbm_Re, (float)lbm_N.x, lbm_u));
	// ###################################################################################### define geometry ######################################################################################
	const float radius = 0.25f*(float)lbm_N.x;
	const float3 center = float3(lbm.center().x, lbm.center().y, 0.36f*radius);
	const float lbm_omega=lbm_u/radius, lbm_domega=lbm_omega*lbm_dt;
	Mesh* mesh = read_stl(get_exe_path()+"../stl/FAN_Solid_Bottom.stl", lbm.size(), center, 2.0f*radius); // https://www.thingiverse.com/thing:6113/files
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u) lbm.flags[n] = TYPE_S; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_FLAG_SURFACE|VIS_Q_CRITERION;
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<lbm_T) { // main simulation loop
		lbm.voxelize_mesh_on_device(mesh, TYPE_S, center, float3(0.0f), float3(0.0f, 0.0f, lbm_omega));
		lbm.run(lbm_dt, lbm_T);
		mesh->rotate(float3x3(float3(0.0f, 0.0f, 1.0f), lbm_domega)); // rotate mesh
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
		if(lbm.graphics.next_frame(lbm_T, 30.0f)) {
			lbm.graphics.set_camera_free(float3(0.353512f*(float)Nx, -0.150326f*(float)Ny, 1.643939f*(float)Nz), -25.0f, 61.0f, 100.0f);
			lbm.graphics.write_frame();
		}
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
	}
} /**/



/*void main_setup() { // electric ducted fan (EDF); required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 1.5f, 1.0f), 8000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_Re = 1000000.0f;
	const float lbm_u = 0.1f;
	const ulong lbm_T = 180000ull;
	const ulong lbm_dt = 4ull;
	LBM lbm(lbm_N, units.nu_from_Re(lbm_Re, (float)lbm_N.x, lbm_u));
	// ###################################################################################### define geometry ######################################################################################
	const float3 center = lbm.center();
	const float3x3 rotation = float3x3(float3(0, 0, 1), radians(180.0f));
	Mesh* stator = read_stl(get_exe_path()+"../stl/edf_v39.stl", 1.0f, rotation); // https://www.thingiverse.com/thing:3014759/files
	Mesh* rotor = read_stl(get_exe_path()+"../stl/edf_v391.stl", 1.0f, rotation); // https://www.thingiverse.com/thing:3014759/files
	const float scale = 0.98f*stator->get_scale_for_box_fit(lbm.size()); // scale stator and rotor to simulation box size
	stator->scale(scale);
	rotor->scale(scale);
	stator->translate(lbm.center()-stator->get_bounding_box_center()-float3(0.0f, 0.2f*stator->get_max_size(), 0.0f)); // move stator and rotor to simulation box center
	rotor->translate(lbm.center()-rotor->get_bounding_box_center()-float3(0.0f, 0.41f*stator->get_max_size(), 0.0f));
	stator->set_center(stator->get_center_of_mass()); // set rotation center of mesh to its center of mass
	rotor->set_center(rotor->get_center_of_mass());
	const float lbm_radius=0.5f*rotor->get_max_size(), omega=lbm_u/lbm_radius, domega=omega*(float)lbm_dt;
	lbm.voxelize_mesh_on_device(stator, TYPE_S, center);
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]==0u) lbm.u.y[n] = 0.3f*lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<lbm_T) { // main simulation loop
		lbm.voxelize_mesh_on_device(rotor, TYPE_S, center, float3(0.0f), float3(0.0f, omega, 0.0f));
		lbm.run(lbm_dt, lbm_T);
		rotor->rotate(float3x3(float3(0.0f, 1.0f, 0.0f), domega)); // rotate mesh
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
		if(lbm.graphics.next_frame(lbm_T, 30.0f)) {
			lbm.graphics.set_camera_centered(-70.0f+100.0f*(float)lbm.get_t()/(float)lbm_T, 2.0f, 60.0f, 1.284025f);
			lbm.graphics.write_frame();
		}
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
	}
} /**/



/*void main_setup() { // aerodynamics of a cow; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 1.0f), 1000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float si_u = 1.0f;
	const float si_length = 2.4f;
	const float si_T = 10.0f;
	const float si_nu=1.48E-5f, si_rho=1.225f;
	const float lbm_length = 0.65f*(float)lbm_N.y;
	const float lbm_u = 0.075f;
	units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
	const ulong lbm_T = units.t(si_T);
	print_info("Re = "+to_string(to_uint(units.si_Re(si_length, si_u, si_nu))));
	LBM lbm(lbm_N, lbm_nu);
	// ###################################################################################### define geometry ######################################################################################
	const float3x3 rotation = float3x3(float3(1, 0, 0), radians(180.0f))*float3x3(float3(0, 0, 1), radians(180.0f));
	Mesh* mesh = read_stl(get_exe_path()+"../stl/Cow_t.stl", lbm.size(), lbm.center(), rotation, lbm_length); // https://www.thingiverse.com/thing:182114/files
	mesh->translate(float3(0.0f, 1.0f-mesh->pmin.y+0.1f*lbm_length, 1.0f-mesh->pmin.z)); // move mesh forward a bit and to simulation box bottom, keep in mind 1 cell thick box boundaries
	lbm.voxelize_mesh_on_device(mesh);
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(z==0u) lbm.flags[n] = TYPE_S; // solid floor
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u; // initialize y-velocity everywhere except in solid cells
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all other simulation box boundaries are inflow/outflow
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	lbm.graphics.set_camera_centered(-40.0f, 20.0f, 78.0f, 1.25f);
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		if(lbm.graphics.next_frame(lbm_T, 10.0f)) lbm.graphics.write_frame();
		lbm.run(1u, lbm_T);
	}
#else // GRAPHICS && !INTERACTIVE_GRAPHICS
	lbm.run();
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
} /**/



/*void main_setup() { // Space Shuttle; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 4.0f, 0.8f), 1000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_Re = 10000000.0f;
	const float lbm_u = 0.075f;
	const ulong lbm_T = 108000ull;
	LBM lbm(lbm_N, 2u, 4u, 1u, units.nu_from_Re(lbm_Re, (float)lbm_N.x, lbm_u)); // run on 2x4x1 = 8 GPUs
	// ###################################################################################### define geometry ######################################################################################
	const float size = 1.25f*lbm.size().x;
	const float3 center = float3(lbm.center().x, 0.55f*size, lbm.center().z+0.05f*size);
	const float3x3 rotation = float3x3(float3(1, 0, 0), radians(-20.0f))*float3x3(float3(0, 0, 1), radians(270.0f));
	Clock clock;
	lbm.voxelize_stl(get_exe_path()+"../stl/Full_Shuttle.stl", center, rotation, size); // https://www.thingiverse.com/thing:4975964/files
	println(print_time(clock.stop()));
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_FLAG_SURFACE|VIS_Q_CRITERION;
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	lbm.write_status();
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		if(lbm.graphics.next_frame(lbm_T, 30.0f)) {
			lbm.graphics.set_camera_free(float3(-1.435962f*(float)Nx, 0.364331f*(float)Ny, 1.344426f*(float)Nz), -205.0f, 36.0f, 74.0f); // top
			lbm.graphics.write_frame(get_exe_path()+"export/top/");
			lbm.graphics.set_camera_free(float3(-1.021207f*(float)Nx, -0.518006f*(float)Ny, 0.0f*(float)Nz), -137.0f, 0.0f, 74.0f); // bottom
			lbm.graphics.write_frame(get_exe_path()+"export/bottom/");
		}
		lbm.run(1u, lbm_T);
	}
	lbm.write_status();
#else // GRAPHICS && !INTERACTIVE_GRAPHICS
	lbm.run();
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
} /**/



/*void main_setup() { // Starship; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 2.0f), 1000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_Re = 10000000.0f;
	const float lbm_u = 0.05f;
	const ulong lbm_T = 108000ull;
	LBM lbm(lbm_N, 1u, 1u, 1u, units.nu_from_Re(lbm_Re, (float)lbm_N.x, lbm_u));
	// ###################################################################################### define geometry ######################################################################################
	const float size = 1.6f*lbm.size().x;
	const float3 center = float3(lbm.center().x, lbm.center().y+0.05f*size, 0.18f*size);
	lbm.voxelize_stl(get_exe_path()+"../stl/StarShipV2.stl", center, size); // https://www.thingiverse.com/thing:4912729/files
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.z[n] = lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_FLAG_SURFACE|VIS_Q_CRITERION;
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	lbm.write_status();
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		if(lbm.graphics.next_frame(lbm_T, 20.0f)) {
			lbm.graphics.set_camera_free(float3(2.116744f*(float)Nx, -0.775261f*(float)Ny, 1.026577f*(float)Nz), -38.0f, 37.0f, 60.0f); // top
			lbm.graphics.write_frame(get_exe_path()+"export/top/");
			lbm.graphics.set_camera_free(float3(0.718942f*(float)Nx, 0.311263f*(float)Ny, -0.498366f*(float)Nz), 32.0f, -40.0f, 104.0f); // bottom
			lbm.graphics.write_frame(get_exe_path()+"export/bottom/");
			lbm.graphics.set_camera_free(float3(1.748119f*(float)Nx, 0.442782f*(float)Ny, 0.087945f*(float)Nz), 24.0f, 2.0f, 92.0f); // side
			lbm.graphics.write_frame(get_exe_path()+"export/side/");
		}
		lbm.run(1u, lbm_T);
	}
	lbm.write_status();
#else // GRAPHICS && !INTERACTIVE_GRAPHICS
	lbm.run();
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
} /**/



/*void main_setup() { // Ahmed body; required extensions in defines.hpp: FP16C, FORCE_FIELD, EQUILIBRIUM_BOUNDARIES, SUBGRID, optionally INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint memory = 10000u; // available VRAM of GPU(s) in MB
	const float lbm_u = 0.05f;
	const float box_scale = 6.0f;
	const float si_u = 60.0f;
	const float si_nu=1.48E-5f, si_rho=1.225f;
	const float si_width=0.389f, si_height=0.288f, si_length=1.044f;
	const float si_A = si_width*si_height+2.0f*0.05f*0.03f;
	const float si_T = 0.25f;
	const float si_Lx = units.x(box_scale*si_width);
	const float si_Ly = units.x(box_scale*si_length);
	const float si_Lz = units.x(0.5f*(box_scale-1.0f)*si_width+si_height);
	const uint3 lbm_N = resolution(float3(si_Lx, si_Ly, si_Lz), memory); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	units.set_m_kg_s((float)lbm_N.y, lbm_u, 1.0f, box_scale*si_length, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
	const ulong lbm_T = units.t(si_T);
	const float lbm_length = units.x(si_length);
	print_info("Re = "+to_string(to_uint(units.si_Re(si_width, si_u, si_nu))));
	LBM lbm(lbm_N, lbm_nu);
	// ###################################################################################### define geometry ######################################################################################
	Mesh* mesh = read_stl(get_exe_path()+"../stl/ahmed_25deg_m.stl", lbm.size(), lbm.center(), float3x3(float3(0, 0, 1), radians(90.0f)), lbm_length);
	mesh->translate(float3(0.0f, units.x(0.5f*(0.5f*box_scale*si_length-si_width))-mesh->pmin.y, 1.0f-mesh->pmin.z));
	lbm.voxelize_mesh_on_device(mesh, TYPE_S|TYPE_X); // https://github.com/nathanrooy/ahmed-bluff-body-cfd/blob/master/geometry/ahmed_25deg_m.stl converted to binary
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(z==0u) lbm.flags[n] = TYPE_S;
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==Nz-1u) lbm.flags[n] = TYPE_E;
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_FIELD;
	lbm.graphics.field_mode = 1;
	lbm.graphics.slice_mode = 1;
	//lbm.graphics.set_camera_centered(20.0f, 30.0f, 10.0f, 1.648722f);
	lbm.run(0u, lbm_T); // initialize simulation
#if defined(FP16S)
	const string path = get_exe_path()+"FP16S/"+to_string(memory)+"MB/";
#elif defined(FP16C)
	const string path = get_exe_path()+"FP16C/"+to_string(memory)+"MB/";
#else // FP32
	const string path = get_exe_path()+"FP32/"+to_string(memory)+"MB/";
#endif // FP32
	//lbm.write_status(path);
	//write_file(path+"Cd.dat", "# t\tCd\n");
	const float3 lbm_com = lbm.object_center_of_mass(TYPE_S|TYPE_X);
	print_info("com = "+to_string(lbm_com.x, 2u)+", "+to_string(lbm_com.y, 2u)+", "+to_string(lbm_com.z, 2u));
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		Clock clock;
		const float3 lbm_force = lbm.object_force(TYPE_S|TYPE_X);
		//const float3 lbm_torque = lbm.object_torque(lbm_com, TYPE_S|TYPE_X);
		//print_info("F="+to_string(lbm_force.x, 2u)+","+to_string(lbm_force.y, 2u)+","+to_string(lbm_force.z, 2u)+", T="+to_string(lbm_torque.x, 2u)+","+to_string(lbm_torque.y, 2u)+","+to_string(lbm_torque.z, 2u)+", t="+to_string(clock.stop(), 3u));
		const float Cd = units.si_F(lbm_force.y)/(0.5f*si_rho*sq(si_u)*si_A); // expect Cd to be too large by a factor 1.3-2.0x; need wall model
		print_info("Cd = "+to_string(Cd, 3u)+", t = "+to_string(clock.stop(), 3u));
		//write_line(path+"Cd.dat", to_string(lbm.get_t())+"\t"+to_string(Cd, 3u)+"\n");
		lbm.run(1u, lbm_T);
	}
	//lbm.write_status(path);
} /**/



/*void main_setup() { // Cessna 172 propeller aircraft; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 0.8f, 0.25f), 8000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_u = 0.075f;
	const float lbm_width = 0.95f*(float)lbm_N.x;
	const ulong lbm_dt = 4ull; // revoxelize rotor every dt time steps
	const float si_T = 1.0f;
	const float si_width = 11.0f;
	const float si_u = 226.0f/3.6f;
	const float si_nu=1.48E-5f, si_rho=1.225f;
	units.set_m_kg_s(lbm_width, lbm_u, 1.0f, si_width, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
	const ulong lbm_T = units.t(si_T);
	print_info("Re = "+to_string(to_uint(units.si_Re(si_width, si_u, si_nu))));
	print_info(to_string(si_T, 3u)+" seconds = "+to_string(lbm_T)+" time steps");
	LBM lbm(lbm_N, units.nu(si_nu));
	// ###################################################################################### define geometry ######################################################################################
	Mesh* plane = read_stl(get_exe_path()+"../stl/Cessna-172-Skyhawk-body.stl"); // https://www.thingiverse.com/thing:814319/files
	Mesh* rotor = read_stl(get_exe_path()+"../stl/Cessna-172-Skyhawk-rotor.stl"); // plane and rotor separated with Microsoft 3D Builder
	const float scale = lbm_width/plane->get_bounding_box_size().x; // scale plane and rotor to simulation box size
	plane->scale(scale);
	rotor->scale(scale);
	const float3 offset = lbm.center()-plane->get_bounding_box_center(); // move plane and rotor to simulation box center
	plane->translate(offset);
	rotor->translate(offset);
	plane->set_center(plane->get_center_of_mass()); // set rotation center of mesh to its center of mass
	rotor->set_center(rotor->get_center_of_mass());
	const float lbm_radius=0.5f*rotor->get_max_size(), omega=-lbm_u/lbm_radius, domega=omega*(float)lbm_dt;
	lbm.voxelize_mesh_on_device(plane);
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		lbm.voxelize_mesh_on_device(rotor, TYPE_S, rotor->get_center(), float3(0.0f), float3(0.0f, omega, 0.0f)); // revoxelize mesh on GPU
		lbm.run(lbm_dt, lbm_T); // run dt time steps
		rotor->rotate(float3x3(float3(0.0f, 1.0f, 0.0f), domega)); // rotate mesh
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
		if(lbm.graphics.next_frame(lbm_T, 5.0f)) {
			lbm.graphics.set_camera_free(float3(0.192778f*(float)Nx, -0.669183f*(float)Ny, 0.657584f*(float)Nz), -77.0f, 27.0f, 100.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/a/");
			lbm.graphics.set_camera_free(float3(0.224926f*(float)Nx, -0.594332f*(float)Ny, -0.277894f*(float)Nz), -65.0f, -14.0f, 100.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/b/");
			lbm.graphics.set_camera_free(float3(-0.000000f*(float)Nx, 0.650189f*(float)Ny, 1.461048f*(float)Nz), 90.0f, 40.0f, 100.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/c/");
		}
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
	}
} /**/



/*void main_setup() { // Bell 222 helicopter; required extensions in defines.hpp: FP16C, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 1.2f, 0.3f), 8000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_u = 0.16f;
	const float lbm_length = 0.8f*(float)lbm_N.x;
	const float si_T = 0.34483f; // 2 revolutions of the main rotor
	const ulong lbm_dt = 4ull; // revoxelize rotor every dt time steps
	const float si_length=12.85f, si_d=12.12f, si_rpm=348.0f;
	const float si_u = si_rpm/60.0f*si_d*pif;
	const float si_nu=1.48E-5f, si_rho=1.225f;
	units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
	const ulong lbm_T = units.t(si_T);
	LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu);
	// ###################################################################################### define geometry ######################################################################################
	Mesh* body = read_stl(get_exe_path()+"../stl/Bell-222-body.stl"); // https://www.thingiverse.com/thing:1625155/files
	Mesh* main = read_stl(get_exe_path()+"../stl/Bell-222-main.stl"); // body and rotors separated with Microsoft 3D Builder
	Mesh* back = read_stl(get_exe_path()+"../stl/Bell-222-back.stl");
	const float scale = lbm_length/body->get_bounding_box_size().y; // scale body and rotors to simulation box size
	body->scale(scale);
	main->scale(scale);
	back->scale(scale);
	const float3 offset = lbm.center()-body->get_bounding_box_center(); // move body and rotors to simulation box center
	body->translate(offset);
	main->translate(offset);
	back->translate(offset);
	body->set_center(body->get_center_of_mass()); // set rotation center of mesh to its center of mass
	main->set_center(main->get_center_of_mass());
	back->set_center(back->get_center_of_mass());
	const float main_radius=0.5f*main->get_max_size(), main_omega=lbm_u/main_radius, main_domega=main_omega*(float)lbm_dt;
	const float back_radius=0.5f*back->get_max_size(), back_omega=-lbm_u/back_radius, back_domega=back_omega*(float)lbm_dt;
	lbm.voxelize_mesh_on_device(body);
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] =  0.2f*lbm_u;
		if(lbm.flags[n]!=TYPE_S) lbm.u.z[n] = -0.1f*lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		lbm.voxelize_mesh_on_device(main, TYPE_S, main->get_center(), float3(0.0f), float3(0.0f, 0.0f, main_omega)); // revoxelize mesh on GPU
		lbm.voxelize_mesh_on_device(back, TYPE_S, back->get_center(), float3(0.0f), float3(back_omega, 0.0f, 0.0f)); // revoxelize mesh on GPU
		lbm.run(lbm_dt, lbm_T); // run dt time steps
		main->rotate(float3x3(float3(0.0f, 0.0f, 1.0f), main_domega)); // rotate mesh
		back->rotate(float3x3(float3(1.0f, 0.0f, 0.0f), back_domega)); // rotate mesh
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
		if(lbm.graphics.next_frame(lbm_T, 10.0f)) {
			lbm.graphics.set_camera_free(float3(0.528513f*(float)Nx, 0.102095f*(float)Ny, 1.302283f*(float)Nz), 16.0f, 47.0f, 96.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/a/");
			lbm.graphics.set_camera_free(float3(0.0f*(float)Nx, -0.114244f*(float)Ny, 0.543265f*(float)Nz), 90.0f+degrees((float)lbm.get_t()/(float)lbm_dt*main_domega), 36.0f, 120.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/b/");
			lbm.graphics.set_camera_free(float3(0.557719f*(float)Nx, -0.503388f*(float)Ny, -0.591976f*(float)Nz), -43.0f, -21.0f, 75.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/c/");
			lbm.graphics.set_camera_centered(58.0f, 9.0f, 88.0f, 1.648722f);
			lbm.graphics.write_frame(get_exe_path()+"export/d/");
			lbm.graphics.set_camera_centered(0.0f, 90.0f, 100.0f, 1.100000f);
			lbm.graphics.write_frame(get_exe_path()+"export/e/");
			lbm.graphics.set_camera_free(float3(0.001612f*(float)Nx, 0.523852f*(float)Ny, 0.992613f*(float)Nz), 90.0f, 37.0f, 94.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/f/");
		}
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
	}
} /**/



/*void main_setup() { // Mercedes F1 W14 car; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 0.5f), 4000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_u = 0.075f;
	const float lbm_length = 0.8f*(float)lbm_N.y;
	const float si_T = 0.25f;
	const float si_u = 100.0f/3.6f;
	const float si_length=5.5f, si_width=2.0f;
	const float si_nu=1.48E-5f, si_rho=1.225f;
	units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
	const ulong lbm_T = units.t(si_T);
	print_info("Re = "+to_string(to_uint(units.si_Re(si_width, si_u, si_nu))));
	LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu);
	// ###################################################################################### define geometry ######################################################################################
	Mesh* body = read_stl(get_exe_path()+"../stl/mercedesf1-body.stl"); // https://downloadfree3d.com/3d-models/vehicles/sports-car/mercedes-f1-w14/
	Mesh* front_wheels = read_stl(get_exe_path()+"../stl/mercedesf1-front-wheels.stl"); // wheels separated, decals removed and converted to .stl in Microsoft 3D Builder
	Mesh* back_wheels = read_stl(get_exe_path()+"../stl/mercedesf1-back-wheels.stl"); // to avoid instability from too small gaps: remove front wheel fenders and move out right back wheel a bit
	const float scale = lbm_length/body->get_bounding_box_size().y; // scale parts
	body->scale(scale);
	front_wheels->scale(scale);
	back_wheels->scale(scale);
	const float3 offset = float3(lbm.center().x-body->get_bounding_box_center().x, 1.0f-body->pmin.y+0.25f*back_wheels->get_min_size(), 4.0f-back_wheels->pmin.z);
	body->translate(offset);
	front_wheels->translate(offset);
	back_wheels->translate(offset);
	body->set_center(body->get_center_of_mass()); // set rotation center of mesh to its center of mass
	front_wheels->set_center(front_wheels->get_center_of_mass());
	back_wheels->set_center(back_wheels->get_center_of_mass());
	const float lbm_radius=0.5f*back_wheels->get_min_size(), omega=lbm_u/lbm_radius;
	lbm.voxelize_mesh_on_device(body);
	lbm.voxelize_mesh_on_device(front_wheels, TYPE_S, front_wheels->get_center(), float3(0.0f), float3(omega, 0.0f, 0.0f)); // make wheels rotating
	lbm.voxelize_mesh_on_device(back_wheels, TYPE_S, back_wheels->get_center(), float3(0.0f), float3(omega, 0.0f, 0.0f)); // make wheels rotating
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==Nz-1u) lbm.flags[n] = TYPE_E;
		if(z==0u) lbm.flags[n] = TYPE_S;
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		if(lbm.graphics.next_frame(lbm_T, 30.0f)) {
			lbm.graphics.set_camera_free(float3(0.779346f*(float)Nx, -0.315650f*(float)Ny, 0.329444f*(float)Nz), -27.0f, 19.0f, 100.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/a/");
			lbm.graphics.set_camera_free(float3(0.556877f*(float)Nx, 0.228191f*(float)Ny, 1.159613f*(float)Nz), 19.0f, 53.0f, 100.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/b/");
			lbm.graphics.set_camera_free(float3(0.220650f*(float)Nx, -0.589529f*(float)Ny, 0.085407f*(float)Nz), -72.0f, 16.0f, 86.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/c/");
			const float progress = (float)lbm.get_t()/(float)lbm_T;
			const float A = 75.0f, B = -160.0f;
			lbm.graphics.set_camera_centered(A+progress*(B-A), -5.0f, 100.0f, 1.648721f);
			lbm.graphics.write_frame(get_exe_path()+"export/d/");
		}
		lbm.run(1u, lbm_T);
	}
#else // GRAPHICS && !INTERACTIVE_GRAPHICS
	lbm.run();
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
} /**/



/*void main_setup() { // hydraulic jump; required extensions in defines.hpp: FP16S, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SURFACE, SUBGRID, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint memory = 208u; // GPU VRAM in MB
	const float si_T = 100.0f; // simulated time in [s]

	const float3 si_N = float3(0.96f, 3.52f, 0.96f); // box size in [m]
	const float si_p1 = si_N.y*3.0f/20.0f; // socket length in [m]
	const float si_h1 = si_N.z*2.0f/5.0f; // socket height in [m]
	const float si_h2 = si_N.z*3.0f/5.0f; // water height in [m]

	const float si_Q = 0.25f; // inlet volumetric flow rate in [m^3/s]
	const float si_A_inlet = si_N.x*(si_h2-si_h1); // inlet cross-section area in [m^2]
	const float si_A_outlet = si_N.x*si_h1; // outlet cross-section area in [m^2]
	const float si_u_inlet = si_Q/si_A_inlet; // inlet average flow velocity in [m/s]
	const float si_u_outlet = si_Q/si_A_outlet; // outlet average flow velocity in [m/s]

	float const si_nu = 1.0E-6f; // kinematic shear viscosity [m^2/s]
	const float si_rho = 1000.0f; // water density [kg/m^3]
	const float si_g = 9.81f; // gravitational acceleration [m/s^2]
	//const float si_sigma = 73.81E-3f; // water surface tension [kg/s^2] (no need to use surface tension here)

	const uint3 lbm_N = resolution(si_N, memory); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_u_inlet = 0.075f; // velocity in LBM units for pairing lbm_u with si_u --> lbm_u in LBM units will be equivalent si_u in SI units
	units.set_m_kg_s((float)lbm_N.y, lbm_u_inlet, 1.0f, si_N.y, si_u_inlet, si_rho); // calculate 3 independent conversion factors (m, kg, s)

	const float lbm_nu = units.nu(si_nu); // kinematic shear viscosity
	const ulong lbm_T = units.t(si_T); // how many time steps to compute to cover exactly si_T seconds in real time
	const float lbm_f = units.f(si_rho, si_g); // force per volume
	//const float lbm_sigma = units.sigma(si_sigma); // surface tension (not required here)

	const uint lbm_p1 = to_uint(units.x(si_p1));
	const uint lbm_h1 = to_uint(units.x(si_h1));
	const uint lbm_h2 = to_uint(units.x(si_h2));
	const float lbm_u_outlet = units.u(si_u_outlet);

	LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu, 0.0f, 0.0f, -lbm_f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(z<lbm_h2) {
			lbm.flags[n] = TYPE_F;
			lbm.rho[n] = units.rho_hydrostatic(0.0005f, z, lbm_h2);
		}
		if(y<lbm_p1&&z<lbm_h1) lbm.flags[n] = TYPE_S;
		if(y<=1u&&x>0u&&x<Nx-1u&&z>=lbm_h1&&z<lbm_h2) {
			lbm.flags[n] = y==0u ? TYPE_S : TYPE_F;
			lbm.u.y[n] = lbm_u_inlet;
		}
		if(y==Ny-1u&&x>0u&&x<Nx-1u&&z>0u) {
			lbm.flags[n] = TYPE_E;
			lbm.u.y[n] = lbm_u_outlet;
		}
		if(x==0u||x==Nx-1u||y==0u||z==0u) lbm.flags[n] = TYPE_S; // sides and bottom non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = lbm.get_D()==1u ? VIS_PHI_RAYTRACE : VIS_PHI_RASTERIZE;
	lbm.run();
	//lbm.run(1000u); lbm.u.read_from_device(); println(lbm.u.x[lbm.index(Nx/2u, Ny/4u, Nz/4u)]); wait(); // test for binary identity
} /**/



/*void main_setup() { // dam break; required extensions in defines.hpp: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	LBM lbm(128u, 256u, 256u, 0.005f, 0.0f, 0.0f, -0.0002f, 0.0001f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(z<Nz*6u/8u && y<Ny/8u) lbm.flags[n] = TYPE_F;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_S; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = lbm.get_D()==1u ? VIS_PHI_RAYTRACE : VIS_PHI_RASTERIZE;
	lbm.run();
} /**/



/*void main_setup() { // liquid metal on a speaker; required extensions in defines.hpp: FP16S, VOLUME_FORCE, MOVING_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint L = 128u;
	const float u = 0.09f; // peak velocity of speaker membrane
	const float f = 0.0005f;
	const float frequency = 0.01f; // amplitude = u/(2.0f*pif*frequency);
	LBM lbm(L, L, L*3u/4u, 0.01f, 0.0f, 0.0f, -f, 0.005f);
	// ###################################################################################### define geometry ######################################################################################
	const uint threads = (uint)thread::hardware_concurrency();
	vector<uint> seed(threads);
	for(uint t=0u; t<threads; t++) seed[t] = 42u+t;
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), threads, [&](ulong n, uint t) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(z<Nz/3u && x>0u&&x<Nx-1u&&y>0u&&y<Ny-1u&&z>0u&&z<Nz-1u) {
			lbm.rho[n] = units.rho_hydrostatic(f, (float)z, (float)(Nz/3u));
			lbm.u.x[n] = random_symmetric(seed[t], 1E-9f);
			lbm.u.y[n] = random_symmetric(seed[t], 1E-9f);
			lbm.u.z[n] = random_symmetric(seed[t], 1E-9f);
			lbm.flags[n] = TYPE_F;
		}
		if(z==0u) lbm.u.z[n] = 1E-16f;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_S; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = lbm.get_D()==1u ? VIS_PHI_RAYTRACE : VIS_PHI_RASTERIZE;
	lbm.run(0u); // initialize simulation
	while(true) { // main simulation loop
		lbm.u.read_from_device();
		const float uz = u*sinf(2.0f*pif*frequency*(float)lbm.get_t());
		for(uint z=0u; z<1u; z++) {
			for(uint y=1u; y<Ny-1u; y++) {
				for(uint x=1u; x<Nx-1u; x++) {
					const uint n = x+(y+z*Ny)*Nx;
					lbm.u.z[n] = uz;
				}
			}
		}
		lbm.u.write_to_device();
		lbm.run(1u);
	}
} /**/



/*void main_setup() { // breaking waves on beach; required extensions in defines.hpp: FP16S, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const float f = 0.001f; // make smaller
	const float u = 0.12f; // peak velocity of speaker membrane
	const float frequency = 0.0007f; // amplitude = u/(2.0f*pif*frequency);
	LBM lbm(128u, 640u, 96u, 0.01f, 0.0f, 0.0f, -f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		const uint H = Nz/2u;
		if(z<H) {
			lbm.flags[n] = TYPE_F;
			lbm.rho[n] = units.rho_hydrostatic(f, (float)z, (float)H);
		}
		if(plane(x, y, z, float3(lbm.center().x, 128.0f, 0.0f), float3(0.0f, -1.0f, 8.0f))) lbm.flags[n] = TYPE_S;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_S; // all non periodic
		if(y==0u && x>0u&&x<Nx-1u&&z>0u&&z<Nz-1u) lbm.flags[n] = TYPE_E;
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE | (lbm.get_D()==1u ? VIS_PHI_RAYTRACE : VIS_PHI_RASTERIZE);
	lbm.run(0u); // initialize simulation
	while(true) { // main simulation loop
		lbm.u.read_from_device();
		const float uy = u*sinf(2.0f*pif*frequency*(float)lbm.get_t());
		const float uz = 0.5f*u*cosf(2.0f*pif*frequency*(float)lbm.get_t());
		for(uint z=1u; z<Nz-1u; z++) {
			for(uint y=0u; y<1u; y++) {
				for(uint x=1u; x<Nx-1u; x++) {
					const uint n = x+(y+z*Ny)*Nx;
					lbm.u.y[n] = uy;
					lbm.u.z[n] = uz;
				}
			}
		}
		lbm.u.write_to_device();
		lbm.run(100u);
	}
} /**/



/*void main_setup() { // river; required extensions in defines.hpp: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	LBM lbm(128u, 384u, 96u, 0.02f, 0.0f, -0.00007f, -0.0005f, 0.01f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		const int R = 20, H = 32;
		if(z==0) lbm.flags[n] = TYPE_S;
		else if(z<H) {
			lbm.flags[n] = TYPE_F;
			lbm.u.y[n] = -0.1f;
		}
		if(cylinder(x, y, z, float3(Nx*2u/3u, Ny*2u/3u, Nz/2u)+0.5f, float3(0u, 0u, Nz), (float)R)) lbm.flags[n] = TYPE_S;
		if(cuboid(x, y, z, float3(Nx/3u, Ny/3u, Nz/2u)+0.5f, float3(2u*R, 2u*R, Nz))) lbm.flags[n] = TYPE_S;
		if(x==0u||x==Nx-1u) lbm.flags[n] = TYPE_S; // x non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = lbm.get_D()==1u ? VIS_PHI_RAYTRACE : VIS_PHI_RASTERIZE;
	lbm.run();
} /**/



/*void main_setup() { // raindrop impact; required extensions in defines.hpp: FP16C, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 1.0f, 0.85f), 4000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	float lbm_D = (float)lbm_N.x/5.0f;
	const float lbm_u = 0.05f; // impact velocity in LBM units
	const float si_T = 0.003f; // simulated time in [s]
	const float inclination = 20.0f; // impact angle [°], 0 = vertical
	const int select_drop_size = 12;
	//                            0        1        2        3        4        5        6        7        8        9       10       11       12       13 (13 is for validation)
	const float si_Ds[] = { 1.0E-3f, 1.5E-3f, 2.0E-3f, 2.5E-3f, 3.0E-3f, 3.5E-3f, 4.0E-3f, 4.5E-3f, 5.0E-3f, 5.5E-3f, 6.0E-3f, 6.5E-3f, 7.0E-3f, 4.1E-3f };
	const float si_us[] = {   4.50f,   5.80f,   6.80f,   7.55f,   8.10f,   8.45f,   8.80f,   9.05f,   9.20f,   9.30f,   9.40f,   9.45f,   9.55f,   7.21f };
	float const si_nu = 1.0508E-6f; // kinematic shear viscosity [m^2/s] at 20°C and 35g/l salinity
	const float si_rho = 1024.8103f; // fluid density [kg/m^3] at 20°C and 35g/l salinity
	const float si_sigma = 73.81E-3f; // fluid surface tension [kg/s^2] at 20°C and 35g/l salinity
	const float si_g = 9.81f; // gravitational acceleration [m/s^2]
	const float si_D = si_Ds[select_drop_size]; // drop diameter [m] (1-7mm)
	const float si_u = si_us[select_drop_size]; // impact velocity [m/s] (4.50-9.55m/s)
	units.set_m_kg_s(lbm_D, lbm_u, 1.0f, si_D, si_u, si_rho); // calculate 3 independent conversion factors (m, kg, s)
	const float lbm_nu = units.nu(si_nu);
	const ulong lbm_T = units.t(si_T);
	const float lbm_f = units.f(si_rho, si_g);
	const float lbm_sigma = units.sigma(si_sigma);
	print_info("D = "+to_string(si_D, 6u));
	print_info("Re = "+to_string(units.si_Re(si_D, si_u, si_nu), 6u));
	print_info("We = "+to_string(units.si_We(si_D, si_u, si_rho, si_sigma), 6u));
	print_info("Fr = "+to_string(units.si_Fr(si_D, si_u, si_g), 6u));
	print_info("Ca = "+to_string(units.si_Ca(si_u, si_rho, si_nu, si_sigma), 6u));
	print_info("Bo = "+to_string(units.si_Bo(si_D, si_rho, si_g, si_sigma), 6u));
	print_info(to_string(to_uint(1000.0f*si_T))+" ms = "+to_string(units.t(si_T))+" LBM time steps");
	const float lbm_H = 0.4f*(float)lbm_N.x;
	const float lbm_R = 0.5f*lbm_D; // drop radius
	LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu, 0.0f, 0.0f, -lbm_f, lbm_sigma); // calculate values for remaining parameters in simulation units
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(sphere(x, y, z, float3(0.5f*(float)Nx, 0.5f*(float)Ny-2.0f*lbm_R*tan(inclination*pif/180.0f), lbm_H+lbm_R+2.5f)+0.5f, lbm_R+2.0f)) {
			const float b = sphere_plic(x, y, z, float3(0.5f*(float)Nx, 0.5f*(float)Ny-2.0f*lbm_R*tan(inclination*pif/180.0f)+0.5f, lbm_H+lbm_R+2.5f), lbm_R);
			if(b!=-1.0f) {
				lbm.u.y[n] =  sinf(inclination*pif/180.0f)*lbm_u;
				lbm.u.z[n] = -cosf(inclination*pif/180.0f)*lbm_u;
				if(b==1.0f) {
					lbm.flags[n] = TYPE_F;
					lbm.phi[n] = 1.0f;
				} else {
					lbm.flags[n] = TYPE_I;
					lbm.phi[n] = b; // initialize cell fill level phi directly instead of just flags, this way the raindrop sphere is smooth already at initialization
				}
			}
		}
		if(z==0) lbm.flags[n] = TYPE_S;
		else if(z==to_uint(lbm_H)) {
			lbm.flags[n] = TYPE_I;
			lbm.phi[n] = 0.5f; // not strictly necessary, but should be clearer (phi is automatically initialized to 0.5f for TYPE_I if not initialized)
		} else if((float)z<lbm_H) lbm.flags[n] = TYPE_F;
		else if((x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==Nz-1u)&&(float)z>lbm_H+0.5f*lbm_R) { // make drops that hit the simulation box ceiling disappear
			lbm.rho[n] = 0.5f;
			lbm.flags[n] = TYPE_E;
		}
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = lbm.get_D()==1u ? VIS_PHI_RAYTRACE : VIS_PHI_RASTERIZE;
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS) && !defined(INTERACTIVE_GRAPHICS_ASCII)
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		if(lbm.graphics.next_frame(lbm_T, 20.0f)) { // generate video
			lbm.graphics.set_camera_centered(-30.0f, 20.0f, 100.0f, 1.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/n/");
			lbm.graphics.set_camera_centered(10.0f, 40.0f, 100.0f, 1.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/p/");
			lbm.graphics.set_camera_centered(0.0f, 0.0f, 45.0f, 1.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/o/");
			lbm.graphics.set_camera_centered(0.0f, 90.0f, 45.0f, 1.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/t/");
		}
		lbm.run(1u, lbm_T);
	}
	//lbm.run(lbm_T); // only generate one image
	//lbm.graphics.set_camera_centered(-30.0f, 20.0f, 100.0f, 1.0f);
	//lbm.graphics.write_frame();
#else // GRAPHICS && !INTERACTIVE_GRAPHICS
	lbm.run();
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
} /**/



/*void main_setup() { // bursting bubble; required extensions in defines.hpp: FP16C, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(4.0f, 4.0f, 3.0f), 1000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_d = 0.25f*(float)lbm_N.x; // bubble diameter in LBM units
	const float lbm_sigma = 0.0003f; // surface tension coefficient in LBM units
	const float si_nu = 1E-6f; // kinematic shear viscosity (water) [m^2/s]
	const float si_rho = 1E3f; // density (water) [kg/m^3]
	const float si_sigma = 0.072f; // surface tension (water) [kg/s^2]
	const float si_d = 4E-3f; // bubble diameter [m]
	const float si_g = 9.81f; // gravitational acceleration [m/s^2]
	const float si_f = units.si_f_from_si_g(si_g, si_rho);
	const float lbm_rho = 1.0f;
	const float m = si_d/lbm_d; // length si_x = x*[m]
	const float kg = si_rho/lbm_rho*cb(m); // density si_rho = rho*[kg/m^3]
	const float s = sqrt(lbm_sigma/si_sigma*kg); // velocity si_sigma = sigma*[kg/s^2]
	units.set_m_kg_s(m, kg, s); // do unit conversion manually via d, rho and sigma
	const uint lbm_H = to_uint(2.0f*lbm_d);
	LBM lbm(lbm_N, units.nu(si_nu), 0.0f, 0.0f, -units.f(si_f), lbm_sigma);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(z<lbm_H) lbm.flags[n] = TYPE_F;
		const float r = 0.5f*lbm_d;
		if(sphere(x, y, z, float3(lbm.center().x, lbm.center().y, (float)lbm_H-0.5f*lbm_d), r+1.0f)) { // bubble
			const float b = clamp(sphere_plic(x, y, z, float3(lbm.center().x, lbm.center().y, (float)lbm_H-0.5f*lbm_d), r), 0.0f, 1.0f);
			if(b==1.0f) {
				lbm.flags[n] = TYPE_G;
				lbm.phi[n] = 0.0f;
			} else {
				lbm.flags[n] = TYPE_I;
				lbm.phi[n] = (1.0f-b); // initialize cell fill level phi directly instead of just flags, this way the bubble sphere is smooth already at initialization
			}
		}
		if(z==0) lbm.flags[n] = TYPE_S;
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = lbm.get_D()==1u ? VIS_PHI_RAYTRACE : VIS_PHI_RASTERIZE;
	lbm.run();
} /**/



/*void main_setup() { // cube with changing gravity; required extensions in defines.hpp: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	LBM lbm(96u, 96u, 96u, 0.02f, 0.0f, 0.0f, -0.001f, 0.001f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(x<Nx*2u/3u&&y<Ny*2u/3u) lbm.flags[n] = TYPE_F;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_S;
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = lbm.get_D()==1u ? VIS_PHI_RAYTRACE : VIS_PHI_RASTERIZE;
	lbm.run(0u); // initialize simulation
	while(true) { // main simulation loop
		lbm.set_f(0.0f, 0.0f, -0.001f);
		lbm.run(2500u);
		lbm.set_f(0.0f, +0.001f, 0.0f);
		lbm.run(2500u);
		lbm.set_f(0.0f, 0.0f, +0.001f);
		lbm.run(2500u);
		lbm.set_f(0.0f, -0.001f, 0.0f);
		lbm.run(2000u);
		lbm.set_f(0.0f, 0.0f, 0.0f);
		lbm.run(3000u);
	}
} /**/



/*void main_setup() { // periodic faucet mass conservation test; required extensions in defines.hpp: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	LBM lbm(96u, 192u, 128u, 0.02f, 0.0f, 0.0f, -0.00025f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(y>Ny*5u/6u) lbm.flags[n] = TYPE_F;
		const uint D=max(Nx, Nz), R=D/6;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u) lbm.flags[n] = TYPE_S; // x and y non periodic
		if((z==0u||z==Nz-1u) && sq(x-Nx/2)+sq(y-Nx/2)>sq(R)) lbm.flags[n] = TYPE_S; // z non periodic
		if(y<=Nx/2u+2u*R && torus_x(x, y, z, float3(Nx/2u, Nx/2u+R, Nz)+0.5f, (float)R, (float)R)) lbm.flags[n] = TYPE_S;
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_PHI_RASTERIZE;
	lbm.run();
} /**/



/*void main_setup() { // two colliding droplets in force field; required extensions in defines.hpp: FP16S, VOLUME_FORCE, FORCE_FIELD, SURFACE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	LBM lbm(256u, 256u, 128u, 0.014f, 0.0f, 0.0f, 0.0f, 0.0001f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(sphere(x, y, z, lbm.center()-float3(0u, 10u, 0u), 32.0f)) {
			lbm.flags[n] = TYPE_F;
			lbm.u.y[n] = 0.025f;
		}
		if(sphere(x, y, z, lbm.center()+float3(30u, 40u, 0u), 12.0f)) {
			lbm.flags[n] = TYPE_F;
			lbm.u.y[n] = -0.2f;
		}
		lbm.F.x[n] = -0.001f*lbm.relative_position(n).x;
		lbm.F.y[n] = -0.001f*lbm.relative_position(n).y;
		lbm.F.z[n] = -0.0005f*lbm.relative_position(n).z;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_S; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = lbm.get_D()==1u ? VIS_PHI_RAYTRACE : VIS_PHI_RASTERIZE;
	lbm.run();
} /**/



/*void main_setup() { // Rayleigh-Benard convection; required extensions in defines.hpp: FP16S, VOLUME_FORCE, TEMPERATURE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	LBM lbm(256u, 256u, 64u, 1u, 1u, 1u, 0.02f, 0.0f, 0.0f, -0.0005f, 0.0f, 1.0f, 1.0f);
	// ###################################################################################### define geometry ######################################################################################
	const uint threads = (uint)thread::hardware_concurrency();
	vector<uint> seed(threads);
	for(uint t=0u; t<threads; t++) seed[t] = 42u+t;
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), threads, [&](ulong n, uint t) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		lbm.u.x[n] = random_symmetric(seed[t], 0.015f); // initialize velocity with random noise
		lbm.u.y[n] = random_symmetric(seed[t], 0.015f);
		lbm.u.z[n] = random_symmetric(seed[t], 0.015f);
		lbm.rho[n] = units.rho_hydrostatic(0.0005f, (float)z, 0.5f*(float)Nz); // initialize density with hydrostatic pressure
		if(z==1u) {
			lbm.T[n] = 1.75f;
			lbm.flags[n] = TYPE_T;
		} else if(z==Nz-2u) {
			lbm.T[n] = 0.25f;
			lbm.flags[n] = TYPE_T;
		}
		if(z==0u||z==Nz-1u) lbm.flags[n] = TYPE_S; // leave lateral simulation box walls periodic by not closing them with TYPE_S
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_STREAMLINES;
	lbm.run();
} /**/



/*void main_setup() { // thermal convection; required extensions in defines.hpp: FP16S, VOLUME_FORCE, TEMPERATURE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	LBM lbm(32u, 196u, 60u, 1u, 1u, 1u, 0.02f, 0.0f, 0.0f, -0.0005f, 0.0f, 1.0f, 1.0f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(y==1) {
			lbm.T[n] = 1.8f;
			lbm.flags[n] = TYPE_T;
		} else if(y==Ny-2) {
			lbm.T[n] = 0.3f;
			lbm.flags[n] = TYPE_T;
		}
		lbm.rho[n] = units.rho_hydrostatic(0.0005f, (float)z, 0.5f*(float)Nz); // initialize density with hydrostatic pressure
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_S; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_STREAMLINES;
	lbm.run();
	//lbm.run(1000u); lbm.u.read_from_device(); println(lbm.u.x[lbm.index(Nx/2u, Ny/2u, Nz/2u)]); wait(); // test for binary identity
} /**/



// ╔═════════════════════════════════════════════════════════════════════════════════════════════════════════╗
// ║ WINDKANAL-SIMULATION MIT FAHRZEUG UND MOVING FLOOR - OPTIMIERT FÜR RTX 3060 Ti (8GB VRAM)                ║
// ║ Fahrzeug fährt in x-Minus-Richtung, Wind von x-Minus, Boden bewegt sich mit Vehicle                       ║
// ║ Asymmetrische Boundary Box: x[-3, +6], y[0, +3] (Symmetrie/Wand durch Fahrzeugmitte), z[0, +3]            ║
// ║ Required extensions in defines.hpp: FP16C, FORCE_FIELD, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES        ║
// ╚═════════════════════════════════════════════════════════════════════════════════════════════════════════╝

/*void main_setup_old() { // Wind tunnel simulation with vehicle and moving floor; required extensions: FP16C, FORCE_FIELD, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, INTERACTIVE_GRAPHICS
	// ################################################################## PHYSIKALISCHE PARAMETER ##################################################################
	
	// Standardatmosphäre (20°C, 1 atm)
	const float si_nu = 1.48E-5f;    // kinematische Viskosität [m^2/s]
	const float si_rho = 1.225f;     // Dichte [kg/m^3]
	const float si_u = 30.0f;        // Windgeschwindigkeit [m/s]
	
	// Boundary Box (in SI-Einheiten, Meter)
	const float si_Lx = 9.0f;        // Länge: x von 0 bis +9 [m]
	const float si_Ly = 6.0f;        // Breite: y von -3 bis +3 [m] (symmetrische Domäne)
	const float si_Lz = 3.0f;        // Höhe: z von 0 bis +3 [m]
	
	// Fahrzeugabmessungen (Schätzung basierend auf vehicle.stl Skalierung)
	const float si_vehicle_length = 4.5f;   // [m]
	const float si_vehicle_width = 1.8f;    // [m]
	const float si_vehicle_height = 1.4f;   // [m]
	const float si_vehicle_A = si_vehicle_width * si_vehicle_height;  // Frontalfläche [m^2]
	
	// VRAM und Gitterauflösung - OPTIMIERT FÜR RTX 3060 Ti (8GB)
	const uint memory = 6000u;  // stabiler Puffer, da komplexes STL
	
	// Berechne Gitterauflösung basierend auf VRAM und Aspektverhältnis
	const uint3 lbm_N = resolution(float3(si_Lx, si_Ly, si_Lz), memory);
	
	// LBM-spezifische Parameter
	const float lbm_u = 0.05f;  // LBM-Geschwindigkeit (skaliert die numerische Stabilität)
	
	// Unit-Konversion: SI ↔ LBM
	units.set_m_kg_s((float)lbm_N.x, lbm_u, 1.0f, si_Lx, si_u, si_rho);
	
	const float lbm_nu = units.nu(si_nu);
	const ulong lbm_T = units.t(15.0f);  // Simulationsdauer: 15 Sekunden
	
	// Reynolds-Zahl (turbulent/laminar Indikator)
	const float Re = units.si_Re(si_vehicle_length, si_u, si_nu);
	
	print_info("════════════════════════════════════════════════════════════════");
	print_info("  WINDKANALSIMULATION MIT MOVING FLOOR - RTX 3060 Ti 8GB");
	print_info("════════════════════════════════════════════════════════════════");
	print_info("  Fahrzeugposition: x∈[+1,+5], y∈[-1,+1], z≈[0,+1.5] (auf Boden, z-shifted)");
	print_info("  Fahrtrichtung: -x (nach links)");
	print_info("  Wind: 30 m/s von x=0 Inlet in +x Richtung");
	print_info("  Boden (z=0): Moving Floor, 30 m/s in +x Richtung");
	print_info("────────────────────────────────────────────────────────────────");
	print_info("  Fahrzeugbreite: "+to_string(si_vehicle_width, 2u)+" m");
	print_info("  Fahrzeuglänge: "+to_string(si_vehicle_length, 2u)+" m");
	print_info("  Reynolds-Zahl (basierend auf Länge): "+to_string(to_uint(Re)));
	print_info("  Windgeschwindigkeit: "+to_string(si_u, 1u)+" m/s");
	print_info("  Gitter-Auflösung: "+to_string(lbm_N.x)+" x "+to_string(lbm_N.y)+" x "+to_string(lbm_N.z));
	print_info("  Simulationszeit: 15 Sekunden = "+to_string(lbm_T)+" Zeitschritte");
	print_info("════════════════════════════════════════════════════════════════\n");
	
	// Erstelle LBM-Objekt
	LBM lbm(lbm_N.x, lbm_N.y, lbm_N.z, lbm_nu);
	
	// ################################################################## FAHRZEUGGEOMETRIE LADEN ##################################################################
	
	// Lade vehicle.stl aus scenes/ Ordner - KOMPLETTES Fahrzeug
	const float cells_per_meter = (float)lbm_N.x / si_Lx; // m -> Zellen
	Mesh* vehicle = read_stl(get_exe_path()+"../scenes/vehicle.stl", cells_per_meter);

	if(vehicle==nullptr) {
		print_error("Fehler: scenes/vehicle.stl nicht gefunden!");
		print_error("Bitte eine STL-Datei nach scenes/vehicle.stl kopieren!");
		return;
	}

	// Positioniere Fahrzeug: x∈[+1,+5], y∈[-1,+1], z∈[0,+1.5]
	// Domain: x∈[0,+9], y∈[-3,+3], z∈[0,+3]
	// Fahrzeug-Center soll bei (+3, 0, 0.75) liegen (Mittelpunkt + z-Versatz für Reifen)
	// In Grid-Koordinaten (Grid mappt zu Domain-Bereich):
	//   x: (+3 - 0) / 9 * Nx = 3/9 * Nx = Nx/3
	//   y: (0 - (-3)) / 6 * Ny = 3/6 * Ny = Ny/2
	//   z: Räder auf Boden: setze pmin.z auf eine kleine Bodenfreiheit (1 Zelle)
	const float target_x = ((3.0f) - (0.0f)) / si_Lx * (float)lbm_N.x;   // 1/3 * Nx
	const float target_y = ((0.0f) - (-3.0f)) / si_Ly * (float)lbm_N.y;   // 1/2 * Ny
	const float ground_clearance_cells = 1.0f; // ~1 Zelle = 12 mm
	const float target_z = ground_clearance_cells; // setze pmin.z auf Boden + clearance

	const float3 vehicle_center = vehicle->get_bounding_box_center();
	const float3 translate = float3(target_x - vehicle_center.x,
	                               target_y - vehicle_center.y,
	                               target_z - vehicle->pmin.z);
	vehicle->translate(translate);

	print_info("✓ Fahrzeug geladen (komplettes STL, nicht gefiltert)");
	print_info("  Bounding Box nach Skalierung und Positionierung [Zellen]:");
	print_info("    Min="+to_string(vehicle->pmin.x, 1u)+","+to_string(vehicle->pmin.y, 1u)+","+to_string(vehicle->pmin.z, 1u));
	print_info("    Max="+to_string(vehicle->pmax.x, 1u)+","+to_string(vehicle->pmax.y, 1u)+","+to_string(vehicle->pmax.z, 1u));

	// Voxelisiere das Fahrzeug (Solid mit TYPE_X für Kraft-Berechnung)
	lbm.voxelize_mesh_on_device(vehicle, TYPE_S|TYPE_X);

	print_info("✓ Fahrzeug voxelisiert und zentriert bei x∈[-2,+2], y∈[-1,+1], z∈[0,+1.5]\n");
	
	// ################################################################## DOMAIN UND GRENZBEDINGUNGEN ##################################################################
	
	const uint Nx = lbm.get_Nx(), Ny = lbm.get_Ny(), Nz = lbm.get_Nz();
	
	// Boundary Box
	// Boundary Box: x[0, +9], y[-3, +3], z[0, +3]
	// Das bedeutet: Linker Rand bei x=0, rechter Rand bei x=+9
	// Untere Wand bei y=-3 (Grid Index 0), obere Wand bei y=+3 (Grid Index Ny-1)
	// Inlet bei x=0 (Grid Index 0)
	
	parallel_for(lbm.get_N(), [&](ulong n) {
		uint x=0u, y=0u, z=0u;
		lbm.coordinates(n, x, y, z);
		
		// Default: alle Zellen sind Fluid mit Windgeschwindigkeit
		lbm.u.x[n] = lbm_u;
		lbm.u.y[n] = 0.0f;
		lbm.u.z[n] = 0.0f;
		
		// Wände in y: No-slip walls
		if(y == 0u || y == Ny-1u) {
			lbm.flags[n] = TYPE_S;
			lbm.u.x[n] = 0.0f;
			lbm.u.y[n] = 0.0f;
			lbm.u.z[n] = 0.0f;
		}
		// Decke: No-slip wall
		if(z == Nz-1u) {
			lbm.flags[n] = TYPE_S;
			lbm.u.x[n] = 0.0f;
			lbm.u.y[n] = 0.0f;
			lbm.u.z[n] = 0.0f;
		}
		// Boden: Moving floor
		if(z == 0u) {
			lbm.flags[n] = TYPE_S;
			lbm.u.x[n] = lbm_u;
			lbm.u.y[n] = 0.0f;
			lbm.u.z[n] = 0.0f;
		}
		// EINLASS: Equilibrium Boundary mit fester Geschwindigkeit (bei x=0, Grid Index 0)
		if(x == 0u) {
			lbm.flags[n] = TYPE_E;
			lbm.u.x[n] = lbm_u;
			lbm.u.y[n] = 0.0f;
			lbm.u.z[n] = 0.0f;
		}
		// AUSLASS: Equilibrium Boundary (u bleibt auf lbm_u initial)
		if(x == Nx - 1u) {
			lbm.flags[n] = TYPE_E;
		}
	});

	
	print_info("✓ Boundary Conditions gesetzt (nur Fahrzeug als Solid visualisiert):");
	print_info("  Einlass (x=0): Equilibrium Boundary, u_x=0.05 (30 m/s Wind)");
	print_info("  Auslass (x=+9): Equilibrium Boundary DRUCK-Outlet (Geschwindigkeit frei)");
} // end of old wind tunnel setup
*/
#if PHASE_7_TRIPLE_RES
// ============================================================================
// Phase 7 Run 2 Helper: Surface-Drag-Map als VTK-POLYDATA (Diagnostik 2026-05-16 late).
// Schreibt für jede TYPE_S|TYPE_X-Cell (Vehicle-Surface) einen Punkt in SI-Welt-Koords
// mit zwei Skalaren:
//   - Fx_si_N: Drag-Beitrag dieser Cell in Newton (units.si_F(F.x_lbm))
//   - Cd_local: dimensionless = Fx_si_N / (0.5·ρ·u²·A_ref)
// Diagnose-Frage: Wo am Fahrzeug entsteht der Drag? (Stirnfläche / Wheel-Wells /
// Heck-Recirculation / Splitter / Diffusor).
// File-Größe ~10-100 KB pro Domain (nur Surface-Cells, kein 400 M-Volume-Bloat).
// In ParaView: Open + Glyph (Sphere, 5mm) oder PointGaussianSplat, Color=Fx_si_N.
// ============================================================================
static void write_surface_drag_vtk_phase7(LBM& lbm, const Units& U, const string& path,
                                          const ulong step, const float dx_m,
                                          const float ox, const float oy, const float oz,
                                          const float si_rho, const float si_u, const float A_ref) {
	lbm.F.read_from_device();
	lbm.flags.read_from_device();
	const uint Nx = lbm.get_Nx(), Ny = lbm.get_Ny();
	const ulong N = lbm.get_N();
	const float q_inf = 0.5f * si_rho * si_u * si_u; // dynamic pressure SI
	// Pre-count surface cells for VTK header
	ulong n_surf = 0u;
	for(ulong n=0u; n<N; n++) {
		if((lbm.flags[n] & (TYPE_S|TYPE_X)) != 0u) n_surf++;
	}
	// Build path with timestamp
	char step_buf[32]; std::snprintf(step_buf, 32, "%09llu", (unsigned long long)step);
	const string fname = path + "drag_map-" + step_buf + ".vtk";
	#ifdef _WIN32
	std::filesystem::create_directories(path);
	#else
	(void)!system(("mkdir -p '" + path + "'").c_str()); // dir likely exists from earlier write_device_to_vtk; ignore retval
	#endif
	std::ofstream f(fname, std::ios::binary);
	f << "# vtk DataFile Version 3.0\n";
	f << "Surface-Drag-Map: Fx_si_N + Cd_local per TYPE_S|TYPE_X cell (Phase 7 Run 2 diagnostic)\n";
	f << "ASCII\n";
	f << "DATASET POLYDATA\n";
	f << "POINTS " << n_surf << " float\n";
	// Write SI positions of surface cells
	for(ulong n=0u; n<N; n++) {
		if((lbm.flags[n] & (TYPE_S|TYPE_X)) == 0u) continue;
		uint x=0u,y=0u,z=0u; lbm.coordinates(n, x, y, z);
		const float sx = ox + ((float)x + 0.5f) * dx_m;
		const float sy = oy + ((float)y + 0.5f) * dx_m;
		const float sz = oz + ((float)z + 0.5f) * dx_m;
		f << sx << " " << sy << " " << sz << "\n";
	}
	// VERTICES (1 vertex per point for ParaView to display them)
	f << "VERTICES " << n_surf << " " << 2ull*n_surf << "\n";
	for(ulong i=0u; i<n_surf; i++) f << "1 " << i << "\n";
	// POINT_DATA: two scalars
	f << "POINT_DATA " << n_surf << "\n";
	f << "SCALARS Fx_si_N float 1\n";
	f << "LOOKUP_TABLE default\n";
	for(ulong n=0u; n<N; n++) {
		if((lbm.flags[n] & (TYPE_S|TYPE_X)) == 0u) continue;
		const float Fx_si = U.si_F(lbm.F.x[n]);
		f << Fx_si << "\n";
	}
	f << "SCALARS Cd_local float 1\n";
	f << "LOOKUP_TABLE default\n";
	for(ulong n=0u; n<N; n++) {
		if((lbm.flags[n] & (TYPE_S|TYPE_X)) == 0u) continue;
		const float Fx_si = U.si_F(lbm.F.x[n]);
		const float Cd_l = Fx_si / (q_inf * A_ref);
		f << Cd_l << "\n";
	}
	// Optional: Fz_si for Downforce-Map
	f << "SCALARS Fz_si_N float 1\n";
	f << "LOOKUP_TABLE default\n";
	for(ulong n=0u; n<N; n++) {
		if((lbm.flags[n] & (TYPE_S|TYPE_X)) == 0u) continue;
		f << U.si_F(lbm.F.z[n]) << "\n";
	}
	f.close();
	print_info("Surface-Drag-Map: "+fname+" ("+to_string(n_surf)+" surface cells)");
}

// ============================================================================
// PHASE 7 — Triple-Resolution Schwarz (Near 4mm + Far 12mm + Coarse 24mm)
// Stand 2026-05-16: KÓDE COMPLETE, NOCH NICHT GETESTET. Aktivieren via
// PHASE_7_TRIPLE_RES=1 in defines/setup-Macro. Trigger: nach Phase 6D-Sichtung
// und User-Freigabe.
//
// Topologie:
//   B70 (renderD129, ~32 GiB):  Near 1500×648×402 @ 4mm  (391 M cells, 23.6 GiB)
//                                Far  734×334×218 @ 12mm  (53.4 M cells,  3.4 GiB)
//   iGPU (renderD128, shared):  Coarse 700×375×316 @ 24mm (83.0 M cells,  5.0 GiB)
// Box-Geometrie:
//   Near :  X[-0.4 , +5.6 ]  Y[-1.296, +1.296]  Z[0, +1.608]
//   Far  :  X[-0.808, +8.0]  Y[-2.004, +2.004]  Z[0, +2.616]
//   Coarse: X[-1.6 , +16.8]  Y[-4.5  , +4.5  ]  Z[0, +7.584]
// si_length = 4.4364 m (STL-native, KEINE 1.4% Up-Skalierung — Phase 7 onwards).
// ============================================================================
void main_setup_phase7_triple_res() {
	// ============================================================
	// 0. Physics + Units
	// ============================================================
	const float lbm_u   = 0.075f;
	const float si_u    = 30.0f;
	const float si_nu   = 1.48E-5f, si_rho = 1.225f;
	const float si_length = 4.4364f; // STL-native: original MR2 STL X-bbox-extent, kein Up-Scaling

	const float dx_near   = 0.004f;   // 4 mm
	const float dx_far    = 0.012f;   // 12 mm
	const float dx_coarse = 0.024f;   // 24 mm

	Units units_near, units_far, units_coarse;
	units_near  .set_m_kg_s(si_length/dx_near,   lbm_u, 1.0f, si_length, si_u, si_rho);
	units_far   .set_m_kg_s(si_length/dx_far,    lbm_u, 1.0f, si_length, si_u, si_rho);
	units_coarse.set_m_kg_s(si_length/dx_coarse, lbm_u, 1.0f, si_length, si_u, si_rho);
	units = units_far; // global = Far (mittlere Skala, für generic print_info-Kompatibilität)
	const float lbm_nu_near   = units_near  .nu(si_nu);
	const float lbm_nu_far    = units_far   .nu(si_nu);
	const float lbm_nu_coarse = units_coarse.nu(si_nu);

	print_info("==================== PHASE 7: TRIPLE-RES SCHWARZ (Near 4mm + Far 12mm B70 || Coarse 24mm iGPU) ====================");
	print_info("Re(L) = "+to_string(to_uint(units_far.si_Re(si_length, si_u, si_nu)))+" | si_length="+to_string(si_length,4u)+" m (STL-native)");

	// ============================================================
	// 1. Device Selection: B70 (most-flops) + iGPU (any other)
	// ============================================================
	const vector<Device_Info>& all_devices = get_devices();
	Device_Info dev_b70  = select_device_with_most_flops(all_devices);
	// Pick iGPU = das ANDERE Gerät mit "Graphics" im Namen (skip CPU-OpenCL und Duplikate). Auf Arrow Lake-S:
	//   B70  = "Intel(R) Graphics [0xe223]"  (discrete BMG-G31, hex ID-Suffix)
	//   iGPU = "Intel(R) Graphics"           (integrated Xe-LPG, ohne Hex-ID-Suffix)
	//   CPU  = "Intel(R) Core(TM) Ultra 9 285K"  (skip — Coarse-Domain auf CPU wäre 100× langsamer als iGPU)
	Device_Info dev_igpu = dev_b70;
	for(uint i=0u; i<(uint)all_devices.size(); i++) {
		if(all_devices[i].id == dev_b70.id) continue;
		const string nm = all_devices[i].name;
		if(nm.find("Graphics") != string::npos) { dev_igpu = all_devices[i]; break; } // bevorzugt Graphics-Device
	}
	if(dev_igpu.id == dev_b70.id) {
		// Fallback: kein zweites Graphics-Device gefunden → nimm irgendein anderes
		for(uint i=0u; i<(uint)all_devices.size(); i++) {
			if(all_devices[i].id != dev_b70.id) { dev_igpu = all_devices[i]; break; }
		}
	}
	if(dev_igpu.id == dev_b70.id) print_warning("Phase 7: nur 1 OpenCL-Device gefunden — Coarse läuft auf B70 (suboptimal, RAM-knapp!)");
	print_info("Phase 7 Devices: B70 = ID "+to_string(dev_b70.id)+" ("+dev_b70.name+") | iGPU = ID "+to_string(dev_igpu.id)+" ("+dev_igpu.name+")");

	// ============================================================
	// 2. Three LBM Instances mit explizitem Device_Info
	// ============================================================
	const uint3 near_N   = uint3(1500u, 648u, 402u);  // 391.0 M cells
	const uint3 far_N    = uint3( 734u, 334u, 218u);  //  53.4 M cells
	const uint3 coarse_N = uint3( 700u, 375u, 316u);  //  82.9 M cells
	print_info("Near  : "+to_string(near_N.x)+"x"+to_string(near_N.y)+"x"+to_string(near_N.z)+" @ 4mm  = "+to_string((ulong)near_N.x*near_N.y*near_N.z)+" cells  (B70  ~23.6 GiB)");
	print_info("Far   : "+to_string(far_N.x )+"x"+to_string(far_N.y )+"x"+to_string(far_N.z )+" @ 12mm = "+to_string((ulong)far_N.x *far_N.y *far_N.z )+" cells  (B70  ~3.4 GiB)");
	print_info("Coarse: "+to_string(coarse_N.x)+"x"+to_string(coarse_N.y)+"x"+to_string(coarse_N.z)+" @ 24mm = "+to_string((ulong)coarse_N.x*coarse_N.y*coarse_N.z)+" cells  (iGPU ~5.0 GiB)");

	LBM lbm_near  (near_N  , lbm_nu_near  , dev_b70 );
	LBM lbm_far   (far_N   , lbm_nu_far   , dev_b70 );
	LBM lbm_coarse(coarse_N, lbm_nu_coarse, dev_igpu);

#ifdef WALL_MODEL_FLOOR
	lbm_near  .wall_floor_u_road = lbm_u;
	lbm_far   .wall_floor_u_road = lbm_u;
	lbm_coarse.wall_floor_u_road = lbm_u;
	print_info("WALL_MODEL_FLOOR active on all 3 LBMs (Rolling Road u_road = "+to_string(lbm_u,4u)+" LBM = "+to_string(si_u,1u)+" m/s)");
#endif

	// ============================================================
	// 3. Vehicle Voxelization in Near + Far (NICHT in Coarse — 24mm zu coarse)
	// ============================================================
	Mesh* vehicle_near = read_stl(get_exe_path()+"../scenes/vehicle.stl");
	const float3 bbox_orig = vehicle_near->get_bounding_box_size(); // STL-native bbox, x_extent = si_length
	vehicle_near->scale((si_length / dx_near) / bbox_orig.x);
	const float3 vbbox_n = vehicle_near->get_bounding_box_size();
	const float3 vctr_n  = vehicle_near->get_bounding_box_center();
	// Near world origin = -0.4 m in X, -1.296 m in Y, 0 in Z. Vehicle bow @ SI x=0.
	const float near_veh_xctr = (0.0f - (-0.4f )) / dx_near + (si_length * 0.5f) / dx_near; // = 100 + 554.55 = 654.55
	const float near_veh_yctr = (0.0f - (-1.296f)) / dx_near;                                // = 324
	vehicle_near->translate(float3(
		near_veh_xctr - vctr_n.x,
		near_veh_yctr - vctr_n.y,
		5.0f - (vctr_n.z - vbbox_n.z * 0.5f))); // Vehicle Near z=5 cell = 20mm clearance
	print_info("Near Vehicle BBox: X["+to_string(vehicle_near->pmin.x,1u)+","+to_string(vehicle_near->pmax.x,1u)+"] Y["+to_string(vehicle_near->pmin.y,1u)+","+to_string(vehicle_near->pmax.y,1u)+"] Z["+to_string(vehicle_near->pmin.z,1u)+","+to_string(vehicle_near->pmax.z,1u)+"] (Near cells)");
	lbm_near.voxelize_mesh_on_device(vehicle_near, TYPE_S|TYPE_X);

	Mesh* vehicle_far = read_stl(get_exe_path()+"../scenes/vehicle.stl");
	vehicle_far->scale((si_length / dx_far) / bbox_orig.x);
	const float3 vbbox_f = vehicle_far->get_bounding_box_size();
	const float3 vctr_f  = vehicle_far->get_bounding_box_center();
	const float far_veh_xctr = (0.0f - (-0.808f)) / dx_far + (si_length * 0.5f) / dx_far; // = 67.33 + 184.85 = 252.18
	const float far_veh_yctr = (0.0f - (-2.004f)) / dx_far;                                // = 167
	vehicle_far->translate(float3(
		far_veh_xctr - vctr_f.x,
		far_veh_yctr - vctr_f.y,
		1.0f - (vctr_f.z - vbbox_f.z * 0.5f))); // Far z=1 cell = 12mm clearance (3-cell Near match via 3:1 ratio)
	print_info("Far Vehicle BBox:  X["+to_string(vehicle_far->pmin.x,1u)+","+to_string(vehicle_far->pmax.x,1u)+"] Y["+to_string(vehicle_far->pmin.y,1u)+","+to_string(vehicle_far->pmax.y,1u)+"] Z["+to_string(vehicle_far->pmin.z,1u)+","+to_string(vehicle_far->pmax.z,1u)+"] (Far cells)");
	lbm_far.voxelize_mesh_on_device(vehicle_far, TYPE_S|TYPE_X);

	// Run 2 (2026-05-16 late evening): Vehicle ALSO in Coarse (24mm) — User-Direktive für 3. unabhängige Fx-Quelle
	// für Multi-Field-Konvergenz-Check (statt nur Fx_far). Voxelisation bei 24mm ist grob (~185 cells along length)
	// aber genügt für Druck-Integration zur Cross-Check-Force-Diagnose.
	Mesh* vehicle_coarse = read_stl(get_exe_path()+"../scenes/vehicle.stl");
	vehicle_coarse->scale((si_length / dx_coarse) / bbox_orig.x);
	const float3 vbbox_c = vehicle_coarse->get_bounding_box_size();
	const float3 vctr_c  = vehicle_coarse->get_bounding_box_center();
	const float coarse_veh_xctr = (0.0f - (-1.6f)) / dx_coarse + (si_length * 0.5f) / dx_coarse; // = 66.67 + 92.43 = 159.09
	const float coarse_veh_yctr = (0.0f - (-4.5f)) / dx_coarse;                                    // = 187.5
	vehicle_coarse->translate(float3(
		coarse_veh_xctr - vctr_c.x,
		coarse_veh_yctr - vctr_c.y,
		1.0f - (vctr_c.z - vbbox_c.z * 0.5f))); // Coarse z=1 cell = 24mm clearance (matches Near 5-cell @ 4mm via 6:1)
	print_info("Coarse Vehicle BBox: X["+to_string(vehicle_coarse->pmin.x,1u)+","+to_string(vehicle_coarse->pmax.x,1u)+"] Y["+to_string(vehicle_coarse->pmin.y,1u)+","+to_string(vehicle_coarse->pmax.y,1u)+"] Z["+to_string(vehicle_coarse->pmin.z,1u)+","+to_string(vehicle_coarse->pmax.z,1u)+"] (Coarse cells)");
	lbm_coarse.voxelize_mesh_on_device(vehicle_coarse, TYPE_S|TYPE_X);

	// ============================================================
	// 4. Boundary Conditions: TYPE_S Moving Wall floor + TYPE_E walls (alle 3 LBMs)
	// ============================================================
	auto set_bcs = [&](LBM& L) {
		const uint Nx = L.get_Nx(), Ny = L.get_Ny(), Nz = L.get_Nz();
		parallel_for(L.get_N(), [&](ulong n) {
			uint x=0u, y=0u, z=0u; L.coordinates(n, x, y, z);
			if((L.flags[n] & TYPE_X) != 0u) {
				if(z==0u) L.u.x[n] = lbm_u; // wheel-contact-patch wandert mit Boden
				return;
			}
			if(z==0u) { L.flags[n] = TYPE_S; L.u.x[n] = lbm_u; L.u.y[n] = 0.0f; L.u.z[n] = 0.0f; }
			else if(x==0u || x==Nx-1u || y==0u || y==Ny-1u || z==Nz-1u) {
				L.flags[n] = TYPE_E; L.u.x[n] = lbm_u; L.u.y[n] = 0.0f; L.u.z[n] = 0.0f;
			} else {
				L.u.x[n] = lbm_u; L.u.y[n] = 0.0f; L.u.z[n] = 0.0f;
			}
		});
	};
	set_bcs(lbm_near);
	set_bcs(lbm_far);
	set_bcs(lbm_coarse); // Run 2: Coarse hat jetzt Fahrzeug + Floor

#ifdef WALL_VISC_BOOST
	lbm_near  .populate_wall_adj_flag(dx_near);   // 4mm → 5 layers (20mm target)
	lbm_far   .populate_wall_adj_flag(dx_far);    // 12mm → 2 layers
	lbm_coarse.populate_wall_adj_flag(dx_coarse); // 24mm → 1 layer (jetzt mit Vehicle + Floor)
	print_info("WALL_VISC_BOOST: distance-based pro Domain (Near 5L, Far 2L, Coarse 1L) — Target BL ~20mm");
#endif

	// ============================================================
	// 5. Coupling-Planes
	//    Near↔Far  (3:1): Near in Far cells = (34, 59, 0), extent (500, 216, 134)
	//    Far↔Coarse (2:1): Far  in Coarse cells = (33, 104, 0), extent (367, 167, 109)
	// ============================================================
	auto mk = [&](uint ox,uint oy,uint oz, uint ea,uint eb, uint ax, float cs){ PlaneSpec p; p.origin=uint3(ox,oy,oz); p.extent_a=ea; p.extent_b=eb; p.axis=ax; p.cell_size=cs; return p; };

	// ---------- Near↔Far Forward Plane Sources (in Far) ----------
	// Near origin in Far cells: (34, 59, 0). Far cells inside Near: x 34..533 (500), y 59..274 (216), z 0..133 (134).
	const uint NF_OX = 34u, NF_OY = 59u, NF_OZ = 0u;
	const uint NF_EX = 500u, NF_EY = 216u, NF_EZ = 134u; // extents in Far cells
	// Far→Near forward sources (skip z=0 floor)
	const PlaneSpec nf_src_xmin = mk(NF_OX,            NF_OY,            1u,           NF_EY, NF_EZ-1u, 0u, dx_far);
	const PlaneSpec nf_src_xmax = mk(NF_OX+NF_EX-1u,   NF_OY,            1u,           NF_EY, NF_EZ-1u, 0u, dx_far);
	const PlaneSpec nf_src_ymin = mk(NF_OX,            NF_OY,            1u,           NF_EX, NF_EZ-1u, 1u, dx_far);
	const PlaneSpec nf_src_ymax = mk(NF_OX,            NF_OY+NF_EY-1u,   1u,           NF_EX, NF_EZ-1u, 1u, dx_far);
	const PlaneSpec nf_src_zmax = mk(NF_OX,            NF_OY,            NF_OZ+NF_EZ-1u, NF_EX, NF_EY, 2u, dx_far);
	// Near targets (skip z=0 floor)
	const PlaneSpec nf_tgt_xmin = mk(0u,               0u,               1u, near_N.y, near_N.z-1u, 0u, dx_near);
	const PlaneSpec nf_tgt_xmax = mk(near_N.x-1u,      0u,               1u, near_N.y, near_N.z-1u, 0u, dx_near);
	const PlaneSpec nf_tgt_ymin = mk(0u,               0u,               1u, near_N.x, near_N.z-1u, 1u, dx_near);
	const PlaneSpec nf_tgt_ymax = mk(0u,               near_N.y-1u,      1u, near_N.x, near_N.z-1u, 1u, dx_near);
	const PlaneSpec nf_tgt_zmax = mk(0u,               0u,               near_N.z-1u, near_N.x, near_N.y, 2u, dx_near);

	// ---------- Far↔Coarse Forward Plane Sources (in Coarse) ----------
	// Far origin in Coarse cells: (33, 104, 0). Coarse cells inside Far: x 33..399 (367), y 104..270 (167), z 0..108 (109).
	const uint FC_OX = 33u, FC_OY = 104u, FC_OZ = 0u;
	const uint FC_EX = 367u, FC_EY = 167u, FC_EZ = 109u; // extents in Coarse cells
	const PlaneSpec fc_src_xmin = mk(FC_OX,            FC_OY,            1u,           FC_EY, FC_EZ-1u, 0u, dx_coarse);
	const PlaneSpec fc_src_xmax = mk(FC_OX+FC_EX-1u,   FC_OY,            1u,           FC_EY, FC_EZ-1u, 0u, dx_coarse);
	const PlaneSpec fc_src_ymin = mk(FC_OX,            FC_OY,            1u,           FC_EX, FC_EZ-1u, 1u, dx_coarse);
	const PlaneSpec fc_src_ymax = mk(FC_OX,            FC_OY+FC_EY-1u,   1u,           FC_EX, FC_EZ-1u, 1u, dx_coarse);
	const PlaneSpec fc_src_zmax = mk(FC_OX,            FC_OY,            FC_OZ+FC_EZ-1u, FC_EX, FC_EY, 2u, dx_coarse);
	// Far targets (skip z=0 floor)
	const PlaneSpec fc_tgt_xmin = mk(0u,               0u,               1u, far_N.y, far_N.z-1u, 0u, dx_far);
	const PlaneSpec fc_tgt_xmax = mk(far_N.x-1u,       0u,               1u, far_N.y, far_N.z-1u, 0u, dx_far);
	const PlaneSpec fc_tgt_ymin = mk(0u,               0u,               1u, far_N.x, far_N.z-1u, 1u, dx_far);
	const PlaneSpec fc_tgt_ymax = mk(0u,               far_N.y-1u,       1u, far_N.x, far_N.z-1u, 1u, dx_far);
	const PlaneSpec fc_tgt_zmax = mk(0u,               0u,               far_N.z-1u, far_N.x, far_N.y, 2u, dx_far);

	// ---------- Near↔Far Back Plane Sources (in Near) ----------
	const uint bn_nf = 2u; // band depth in Near cells
	const PlaneSpec bk_nf_src_xmin = mk(bn_nf,             0u,                  1u, near_N.y, near_N.z-1u, 0u, dx_near);
	const PlaneSpec bk_nf_src_xmax = mk(near_N.x-1u-bn_nf, 0u,                  1u, near_N.y, near_N.z-1u, 0u, dx_near);
	const PlaneSpec bk_nf_src_ymin = mk(0u,                bn_nf,               1u, near_N.x, near_N.z-1u, 1u, dx_near);
	const PlaneSpec bk_nf_src_ymax = mk(0u,                near_N.y-1u-bn_nf,   1u, near_N.x, near_N.z-1u, 1u, dx_near);
	const PlaneSpec bk_nf_src_zmax = mk(0u,                0u,                  near_N.z-1u-bn_nf, near_N.x, near_N.y, 2u, dx_near);
	// targets in Far (1 Far cell inside Near's overlap)
	const PlaneSpec bk_nf_tgt_xmin = mk(NF_OX+1u,          NF_OY,               1u, NF_EY, NF_EZ-1u, 0u, dx_far);
	const PlaneSpec bk_nf_tgt_xmax = mk(NF_OX+NF_EX-2u,    NF_OY,               1u, NF_EY, NF_EZ-1u, 0u, dx_far);
	const PlaneSpec bk_nf_tgt_ymin = mk(NF_OX,             NF_OY+1u,            1u, NF_EX, NF_EZ-1u, 1u, dx_far);
	const PlaneSpec bk_nf_tgt_ymax = mk(NF_OX,             NF_OY+NF_EY-2u,      1u, NF_EX, NF_EZ-1u, 1u, dx_far);
	const PlaneSpec bk_nf_tgt_zmax = mk(NF_OX,             NF_OY,               NF_OZ+NF_EZ-2u, NF_EX, NF_EY, 2u, dx_far);

	// ---------- Far↔Coarse Back Plane Sources (in Far) ----------
	const uint bn_fc = 2u; // band depth in Far cells (≥ 2:1 ratio)
	const PlaneSpec bk_fc_src_xmin = mk(bn_fc,             0u,                  1u, far_N.y, far_N.z-1u, 0u, dx_far);
	const PlaneSpec bk_fc_src_xmax = mk(far_N.x-1u-bn_fc,  0u,                  1u, far_N.y, far_N.z-1u, 0u, dx_far);
	const PlaneSpec bk_fc_src_ymin = mk(0u,                bn_fc,               1u, far_N.x, far_N.z-1u, 1u, dx_far);
	const PlaneSpec bk_fc_src_ymax = mk(0u,                far_N.y-1u-bn_fc,    1u, far_N.x, far_N.z-1u, 1u, dx_far);
	const PlaneSpec bk_fc_src_zmax = mk(0u,                0u,                  far_N.z-1u-bn_fc, far_N.x, far_N.y, 2u, dx_far);
	// targets in Coarse
	const PlaneSpec bk_fc_tgt_xmin = mk(FC_OX+1u,          FC_OY,               1u, FC_EY, FC_EZ-1u, 0u, dx_coarse);
	const PlaneSpec bk_fc_tgt_xmax = mk(FC_OX+FC_EX-2u,    FC_OY,               1u, FC_EY, FC_EZ-1u, 0u, dx_coarse);
	const PlaneSpec bk_fc_tgt_ymin = mk(FC_OX,             FC_OY+1u,            1u, FC_EX, FC_EZ-1u, 1u, dx_coarse);
	const PlaneSpec bk_fc_tgt_ymax = mk(FC_OX,             FC_OY+FC_EY-2u,      1u, FC_EX, FC_EZ-1u, 1u, dx_coarse);
	const PlaneSpec bk_fc_tgt_zmax = mk(FC_OX,             FC_OY,               FC_OZ+FC_EZ-2u, FC_EX, FC_EY, 2u, dx_coarse);

	// ============================================================
	// 6. Coupling Options
	// ============================================================
	CouplingOptions opts;       opts.smooth_plane = false;       opts.export_csv = false;       opts.sync_pcie = false;
	CouplingOptions opts_back;  opts_back.smooth_plane = false;  opts_back.export_csv = false;  opts_back.sync_pcie = false;

	// Blending: Plateau + Linear-Ramp wie Phase 6C (α = ALPHA_HIGH am Boundary, fällt linear ins Innere; back rise α=ALPHA_LOW boundary).
	const float ALPHA_LOW  = 0.05f;  // Phase 6C-Default (kann nach 6D-Sichtung auf 0.0 gesetzt werden)
	const float ALPHA_HIGH = 0.50f;  // Phase 6C-Default (kann nach 6D-Sichtung auf 1.0 gesetzt werden)
	// Near↔Far (3:1): 36 Near = 12 Far Band-Tiefe, Plateau 1 Far = 3 Near
	const uint  BAND_NEAR_NF = 36u; const uint BAND_FAR_NF = 12u; const uint PLATEAU_NEAR_NF = 3u; const uint PLATEAU_FAR_NF = 1u;
	// Far↔Coarse (2:1): 12 Far = 6 Coarse Band-Tiefe, Plateau 1 Coarse = 2 Far
	const uint  BAND_FAR_FC = 12u; const uint BAND_COARSE_FC = 6u; const uint PLATEAU_FAR_FC = 2u; const uint PLATEAU_COARSE_FC = 1u;

	// ============================================================
	// 7. Run-Loop (Mode 3 PERF-G mit 3 LBMs, ratio 1:3:6 für gleiche SI-Δt)
	// Run 2 2026-05-16 late: chunk_far=100 (statt 50) — dämpft Saw-tooth-Schwingung
	// (Run 1 hatte chunk_far=50 → σ/μ Fx_near 58% durch 2-Chunk-Oszillation).
	// Near chunk = 300 → Coarse chunk = 50 (Far dt = 2× Coarse dt).
	// ============================================================
	const string force_csv_path = get_exe_path()+"../bin/forces_phase7_triple_res.csv";
	std::ofstream fcsv(force_csv_path);
	fcsv << "step_far,t_si,Fx_far,Fy_far,Fz_far,Fx_near,Fy_near,Fz_near,Fx_coarse,Fy_coarse,Fz_coarse\n";
	fcsv << std::scientific;
	const uint chunk_far    = 100u;            // Run 2: 100 statt 50 (Saw-tooth-Dämpfung)
	const uint chunk_near   = chunk_far * 3u;  // 3:1 Ratio Near:Far → 300
	const uint chunk_coarse = chunk_far / 2u;  // 1:2 Ratio Coarse:Far → 50
	const uint chunks_max     = 500u;
	const uint conv_window    = 25u;
	const uint conv_min_chunks= 50u;
	const float conv_tol      = 0.02f;
	std::vector<float3> Fhist_far, Fhist_near, Fhist_coarse;
	Fhist_far.reserve(chunks_max); Fhist_near.reserve(chunks_max); Fhist_coarse.reserve(chunks_max);
	print_info("Phase 7 Run 2 start: max "+to_string(chunks_max*chunk_far)+" Far-steps | auto-stop bei max(|dFx_far|,|dFx_near|,|dFx_coarse|)/|Fx| < 2% über 25-chunk window (earliest chunk 50)");
	print_info("  Per-Chunk: "+to_string(chunk_near)+" Near + "+to_string(chunk_far)+" Far + "+to_string(chunk_coarse)+" Coarse (same SI dt = "+to_string(chunk_far*dx_far*lbm_u/si_u, 5u)+" s)");

	uint final_chunks = chunks_max;
	for(uint c = 0u; c < chunks_max; c++) {
		// ----- Concurrent Run aller 3 LBMs (PERF-G Additive Schwarz) -----
		if(c == 0u) {
			lbm_coarse.run(chunk_coarse);
			lbm_far   .run(chunk_far   );
			lbm_near  .run(chunk_near  );
		} else {
			lbm_coarse.run_async(chunk_coarse);
			lbm_far   .run_async(chunk_far   );
			lbm_near  .run_async(chunk_near  );
			lbm_coarse.finish();
			lbm_far   .finish();
			lbm_near  .finish();
		}

		// ----- PERF-D: batched device→host reads -----
		lbm_near  .u.read_from_device();   lbm_near  .rho.read_from_device();   lbm_near  .flags.read_from_device();
		lbm_far   .u.read_from_device();   lbm_far   .rho.read_from_device();   lbm_far   .flags.read_from_device();
		lbm_coarse.u.read_from_device();   lbm_coarse.rho.read_from_device();   lbm_coarse.flags.read_from_device();

		// ----- Forward Coupling Near↔Far (Coarse-Cell-Ratio 3:1, plateau-ramp) -----
		for(uint d = 0u; d < BAND_NEAR_NF; d++) {
			float a;
			if(d < PLATEAU_NEAR_NF) a = ALPHA_HIGH;
			else { const uint r = d - PLATEAU_NEAR_NF; const uint L = BAND_NEAR_NF - PLATEAU_NEAR_NF - 1u; a = ALPHA_HIGH - (float)r/(float)L * (ALPHA_HIGH - ALPHA_LOW); }
			CouplingOptions od = opts; od.alpha = a; od.keep_flags = (d > 0u);
			const uint dsrc = d / 3u;
			PlaneSpec t, s;
			t = nf_tgt_xmin; t.origin.x = d;                    s = nf_src_xmin; s.origin.x = nf_src_xmin.origin.x + dsrc;  lbm_far.couple_fields(lbm_near, s, t, od);
			t = nf_tgt_xmax; t.origin.x = nf_tgt_xmax.origin.x - d;  s = nf_src_xmax; s.origin.x = nf_src_xmax.origin.x - dsrc;  lbm_far.couple_fields(lbm_near, s, t, od);
			t = nf_tgt_ymin; t.origin.y = d;                    s = nf_src_ymin; s.origin.y = nf_src_ymin.origin.y + dsrc;  lbm_far.couple_fields(lbm_near, s, t, od);
			t = nf_tgt_ymax; t.origin.y = nf_tgt_ymax.origin.y - d;  s = nf_src_ymax; s.origin.y = nf_src_ymax.origin.y - dsrc;  lbm_far.couple_fields(lbm_near, s, t, od);
			t = nf_tgt_zmax; t.origin.z = nf_tgt_zmax.origin.z - d;  s = nf_src_zmax; s.origin.z = nf_src_zmax.origin.z - dsrc;  lbm_far.couple_fields(lbm_near, s, t, od);
		}

		// ----- Back Coupling Near→Far (3:1 ratio, plateau-ramp) -----
		for(uint df = 0u; df < BAND_FAR_NF; df++) {
			float a;
			if(df < PLATEAU_FAR_NF) a = ALPHA_LOW;
			else { const uint r = df - PLATEAU_FAR_NF; const uint L = BAND_FAR_NF - PLATEAU_FAR_NF - 1u; a = ALPHA_LOW + (float)r/(float)L * (ALPHA_HIGH - ALPHA_LOW); }
			CouplingOptions od = opts_back; od.alpha = a;
			const uint dn = 1u + 3u*df; // Near source: center of 3-cell block
			PlaneSpec s, t;
			s = bk_nf_src_xmin; s.origin.x = dn;                       t = bk_nf_tgt_xmin; t.origin.x = NF_OX+1u + df;             lbm_near.couple_fields(lbm_far, s, t, od);
			s = bk_nf_src_xmax; s.origin.x = near_N.x-1u-dn;            t = bk_nf_tgt_xmax; t.origin.x = NF_OX+NF_EX-2u - df;       lbm_near.couple_fields(lbm_far, s, t, od);
			s = bk_nf_src_ymin; s.origin.y = dn;                       t = bk_nf_tgt_ymin; t.origin.y = NF_OY+1u + df;             lbm_near.couple_fields(lbm_far, s, t, od);
			s = bk_nf_src_ymax; s.origin.y = near_N.y-1u-dn;            t = bk_nf_tgt_ymax; t.origin.y = NF_OY+NF_EY-2u - df;       lbm_near.couple_fields(lbm_far, s, t, od);
			s = bk_nf_src_zmax; s.origin.z = near_N.z-1u-dn;            t = bk_nf_tgt_zmax; t.origin.z = NF_OZ+NF_EZ-2u - df;       lbm_near.couple_fields(lbm_far, s, t, od);
		}

		// ----- Forward Coupling Far↔Coarse (2:1 ratio, plateau-ramp) -----
		for(uint d = 0u; d < BAND_FAR_FC; d++) {
			float a;
			if(d < PLATEAU_FAR_FC) a = ALPHA_HIGH;
			else { const uint r = d - PLATEAU_FAR_FC; const uint L = BAND_FAR_FC - PLATEAU_FAR_FC - 1u; a = ALPHA_HIGH - (float)r/(float)L * (ALPHA_HIGH - ALPHA_LOW); }
			CouplingOptions od = opts; od.alpha = a; od.keep_flags = (d > 0u);
			const uint dsrc = d / 2u;
			PlaneSpec t, s;
			t = fc_tgt_xmin; t.origin.x = d;                    s = fc_src_xmin; s.origin.x = fc_src_xmin.origin.x + dsrc;  lbm_coarse.couple_fields(lbm_far, s, t, od);
			t = fc_tgt_xmax; t.origin.x = fc_tgt_xmax.origin.x - d;  s = fc_src_xmax; s.origin.x = fc_src_xmax.origin.x - dsrc;  lbm_coarse.couple_fields(lbm_far, s, t, od);
			t = fc_tgt_ymin; t.origin.y = d;                    s = fc_src_ymin; s.origin.y = fc_src_ymin.origin.y + dsrc;  lbm_coarse.couple_fields(lbm_far, s, t, od);
			t = fc_tgt_ymax; t.origin.y = fc_tgt_ymax.origin.y - d;  s = fc_src_ymax; s.origin.y = fc_src_ymax.origin.y - dsrc;  lbm_coarse.couple_fields(lbm_far, s, t, od);
			t = fc_tgt_zmax; t.origin.z = fc_tgt_zmax.origin.z - d;  s = fc_src_zmax; s.origin.z = fc_src_zmax.origin.z - dsrc;  lbm_coarse.couple_fields(lbm_far, s, t, od);
		}

		// ----- Back Coupling Far→Coarse (2:1 ratio, plateau-ramp) -----
		for(uint dc = 0u; dc < BAND_COARSE_FC; dc++) {
			float a;
			if(dc < PLATEAU_COARSE_FC) a = ALPHA_LOW;
			else { const uint r = dc - PLATEAU_COARSE_FC; const uint L = BAND_COARSE_FC - PLATEAU_COARSE_FC - 1u; a = ALPHA_LOW + (float)r/(float)L * (ALPHA_HIGH - ALPHA_LOW); }
			CouplingOptions od = opts_back; od.alpha = a;
			const uint df = 2u*dc; // Far source: center of 2-cell block (0.5+0.5 floor)
			PlaneSpec s, t;
			s = bk_fc_src_xmin; s.origin.x = df;                       t = bk_fc_tgt_xmin; t.origin.x = FC_OX+1u + dc;             lbm_far.couple_fields(lbm_coarse, s, t, od);
			s = bk_fc_src_xmax; s.origin.x = far_N.x-1u-df;             t = bk_fc_tgt_xmax; t.origin.x = FC_OX+FC_EX-2u - dc;       lbm_far.couple_fields(lbm_coarse, s, t, od);
			s = bk_fc_src_ymin; s.origin.y = df;                       t = bk_fc_tgt_ymin; t.origin.y = FC_OY+1u + dc;             lbm_far.couple_fields(lbm_coarse, s, t, od);
			s = bk_fc_src_ymax; s.origin.y = far_N.y-1u-df;             t = bk_fc_tgt_ymax; t.origin.y = FC_OY+FC_EY-2u - dc;       lbm_far.couple_fields(lbm_coarse, s, t, od);
			s = bk_fc_src_zmax; s.origin.z = far_N.z-1u-df;             t = bk_fc_tgt_zmax; t.origin.z = FC_OZ+FC_EZ-2u - dc;       lbm_far.couple_fields(lbm_coarse, s, t, od);
		}

		// ----- PERF-D: batched host→device writes -----
		lbm_near  .flags.write_to_device();   lbm_near  .u.write_to_device();   lbm_near  .rho.write_to_device();
		lbm_far   .flags.write_to_device();   lbm_far   .u.write_to_device();   lbm_far   .rho.write_to_device();
		lbm_coarse.flags.write_to_device();   lbm_coarse.u.write_to_device();   lbm_coarse.rho.write_to_device();

		// ----- Force Diagnostics (Run 2: Far + Near + Coarse — alle 3 für Multi-Field-Convergence) -----
		lbm_far   .update_force_field(); const float3 F_far_lbm    = lbm_far   .object_force(TYPE_S|TYPE_X);
		lbm_near  .update_force_field(); const float3 F_near_lbm   = lbm_near  .object_force(TYPE_S|TYPE_X);
		lbm_coarse.update_force_field(); const float3 F_coarse_lbm = lbm_coarse.object_force(TYPE_S|TYPE_X);
		const float3 F_far_si    = float3(units_far   .si_F(F_far_lbm.x   ), units_far   .si_F(F_far_lbm.y   ), units_far   .si_F(F_far_lbm.z   ));
		const float3 F_near_si   = float3(units_near  .si_F(F_near_lbm.x  ), units_near  .si_F(F_near_lbm.y  ), units_near  .si_F(F_near_lbm.z  ));
		const float3 F_coarse_si = float3(units_coarse.si_F(F_coarse_lbm.x), units_coarse.si_F(F_coarse_lbm.y), units_coarse.si_F(F_coarse_lbm.z));
		const ulong step = (ulong)(c+1u) * chunk_far;
		const float t_si = units_far.si_t(step);
		fcsv << step << "," << t_si << "," << F_far_si.x << "," << F_far_si.y << "," << F_far_si.z
		             << "," << F_near_si.x << "," << F_near_si.y << "," << F_near_si.z
		             << "," << F_coarse_si.x << "," << F_coarse_si.y << "," << F_coarse_si.z << "\n";
		fcsv.flush();
		print_info("step_far="+to_string(step)+"  Fx_far="+to_string(F_far_si.x,1u)+"N  Fx_near="+to_string(F_near_si.x,1u)+"N  Fx_coarse="+to_string(F_coarse_si.x,1u)+"N  Fz_near="+to_string(F_near_si.z,1u));
		Fhist_far.push_back(F_far_si); Fhist_near.push_back(F_near_si); Fhist_coarse.push_back(F_coarse_si);

		// Run 2: Multi-Field-Convergence — max(|dFx_X|/|Fx_X|) über alle 3 Felder muss < tol sein.
		// Verhindert falsely-positive Trigger durch in-phase Saw-tooth-Pinch-Punkte in einem einzelnen Feld
		// (siehe Phase 7 Run 1 + Phase 6D Pathology).
		if(c+1u >= conv_min_chunks) {
			auto window_mean_x = [&](const std::vector<float3>& H, uint offset)->float {
				float s = 0.0f;
				for(uint k=0u; k<conv_window; k++) s += H[H.size()-1u-offset-k].x;
				return s / (float)conv_window;
			};
			const float far_r    = window_mean_x(Fhist_far,    0u);
			const float far_p    = window_mean_x(Fhist_far,    conv_window);
			const float near_r   = window_mean_x(Fhist_near,   0u);
			const float near_p   = window_mean_x(Fhist_near,   conv_window);
			const float coarse_r = window_mean_x(Fhist_coarse, 0u);
			const float coarse_p = window_mean_x(Fhist_coarse, conv_window);
			const float d_far    = (far_r    != 0.0f) ? fabs(far_r    - far_p)    / fabs(far_r)    : 1.0f;
			const float d_near   = (near_r   != 0.0f) ? fabs(near_r   - near_p)   / fabs(near_r)   : 1.0f;
			const float d_coarse = (coarse_r != 0.0f) ? fabs(coarse_r - coarse_p) / fabs(coarse_r) : 1.0f;
			const float d_max    = std::max(d_far, std::max(d_near, d_coarse));
			if(d_max < conv_tol) {
				print_info("CONVERGED at chunk "+to_string(c+1u)+" (step_far "+to_string(step)+"): max(|dFx|/Fx)="+to_string(d_max*100.0f,3u)+"%  [far="+to_string(d_far*100.0f,2u)+"%, near="+to_string(d_near*100.0f,2u)+"%, coarse="+to_string(d_coarse*100.0f,2u)+"%]  Fx: far="+to_string(far_r,1u)+" near="+to_string(near_r,1u)+" coarse="+to_string(coarse_r,1u)+"N");
				final_chunks = c+1u;
				break;
			}
		}
	}
	fcsv.close();
	print_info("Phase 7 Run done after "+to_string(final_chunks)+" chunks ("+to_string(final_chunks*chunk_far)+" Far-steps)");

	// ============================================================
	// 8. VTK Export — alle 3 Domains in eigene Subfolder
	// ============================================================
	const string export_near   = get_exe_path()+"../export/tr_near/";
	const string export_far    = get_exe_path()+"../export/tr_far/";
	const string export_coarse = get_exe_path()+"../export/tr_coarse/";
	lbm_near  .u    .write_device_to_vtk(export_near);   lbm_near  .rho.write_device_to_vtk(export_near);   lbm_near  .flags.write_device_to_vtk(export_near);   lbm_near  .F.write_device_to_vtk(export_near);
	lbm_far   .u    .write_device_to_vtk(export_far);    lbm_far   .rho.write_device_to_vtk(export_far);    lbm_far   .flags.write_device_to_vtk(export_far);    lbm_far   .F.write_device_to_vtk(export_far);
	lbm_coarse.u    .write_device_to_vtk(export_coarse); lbm_coarse.rho.write_device_to_vtk(export_coarse); lbm_coarse.flags.write_device_to_vtk(export_coarse); lbm_coarse.F.write_device_to_vtk(export_coarse);
	lbm_near  .write_mesh_to_vtk(vehicle_near,   export_near);
	lbm_far   .write_mesh_to_vtk(vehicle_far,    export_far );
	lbm_coarse.write_mesh_to_vtk(vehicle_coarse, export_coarse); // Run 2: Vehicle jetzt auch in Coarse

	// Run 2 Diagnostic: Surface-Drag-Map per Domain (Vehicle-Surface-Cells nur, klein + visualisierbar in ParaView)
	const float A_ref = 1.85f; // Vehicle frontal area [m²]
	const ulong step_near   = (ulong)final_chunks * (ulong)chunk_near;
	const ulong step_far    = (ulong)final_chunks * (ulong)chunk_far;
	const ulong step_coarse = (ulong)final_chunks * (ulong)chunk_coarse;
	write_surface_drag_vtk_phase7(lbm_near,   units_near,   export_near,   step_near,   dx_near,   -0.4f, -1.296f, 0.0f, si_rho, si_u, A_ref);
	write_surface_drag_vtk_phase7(lbm_far,    units_far,    export_far,    step_far,    dx_far,    -0.808f, -2.004f, 0.0f, si_rho, si_u, A_ref);
	write_surface_drag_vtk_phase7(lbm_coarse, units_coarse, export_coarse, step_coarse, dx_coarse, -1.6f, -4.5f, 0.0f, si_rho, si_u, A_ref);
	print_info("Phase 7 done. Force-CSV: "+force_csv_path);
	print_info("VTKs: Near="+export_near+" | Far="+export_far+" | Coarse="+export_coarse);
	std::fflush(nullptr);
	_exit(0); // xe-driver cleanup-race workaround
}
#endif // PHASE_7_TRIPLE_RES
