#include "setup.hpp"
#include <fstream>
#include <cstring>

// =============================================================================================
// KUGEL IM KANAL MIT MITBEWEGTEM BODEN
//
// D = 450 mm, U = 30 m/s, ISA-Meereshoehe. Validierungsfall gegen OpenFOAM 13 und gegen die
// Standard-Widerstandskurve (Clift/Grace/Weber), letztere ueber CFD_KUGEL_NU bei Re_D = 100..1000
// erreichbar -- dort ist die Grenzschicht aufgeloest und es braucht kein Turbulenzmodell.
//
// Dies ist der Neuaufbau auf frischem Upstream (2026-08-08). Bewusst OHNE: Wandmodell,
// NUT_PATH_A, Mozaffari-APG, Bodenclamp, Abloesesonde, Slice-Renderer. Die kommen einzeln
// zurueck, jede erst dann, wenn eine Messung zeigt, dass sie eine Zahl veraendert.
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

// Schalter-Helfer: gesetzt UND nicht ""/"0"/"false"/"off" = an.
// Das Muster getenv(X)!=nullptr schaltet bei CFD_X=0 EIN -- im alten Baum an 60 Stellen so.
static bool env_on(const char* name) {
	const char* v = getenv(name); if(v==nullptr) return false;
	return !(v[0]=='\0' || (v[0]=='0'&&v[1]=='\0') || strcmp(v,"false")==0 || strcmp(v,"off")==0);
}
static float env_f(const char* name, const float fallback) {
	const char* v = getenv(name); return v==nullptr ? fallback : (float)atof(v);
}
static uint env_u(const char* name, const uint fallback) {
	const char* v = getenv(name); return v==nullptr ? fallback : (uint)max(1, atoi(v));
}

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

void main_setup() {
	// ---------------------------------------------------------------- Physik
	const float si_u   = 30.0f;                          // Anstroemung und Bodengeschwindigkeit [m/s]
	const float si_rho = 1.225f;                         // ISA Meereshoehe [kg/m^3]
	// ν per Env fuer die Validierung gegen die Standard-Widerstandskurve: bei Re_D = 100..1000 ist
	// die Grenzschicht mit ~6 Zellen aufgeloest und tau = 0.53..0.58 liegt weit weg von 0.5.
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

	units.set_m_kg_s(D/dx, u_lat, 1.0f, D, si_u, si_rho);
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

	// SAT-Schale: jede vom STL geschnittene Zelle wird solid. Der Ray-Paritaets-Voxelizer
	// oben laesst achsparallele Ebenen loechrig -- an der Symmetrieebene y=Dy_center war das
	// im alten Baum als 72 statt ~1064 Solidzellen messbar.
	{
		const int bx0=(int)fmax(0.0f, floor(kugel->pmin.x)-2.0f), bx1=(int)fmin((float)Nx-1.0f, ceil(kugel->pmax.x)+2.0f);
		const int by0=(int)fmax(0.0f, floor(kugel->pmin.y)-2.0f), by1=(int)fmin((float)Ny-1.0f, ceil(kugel->pmax.y)+2.0f);
		const int bz0=(int)fmax(0.0f, floor(kugel->pmin.z)-2.0f), bz1=(int)fmin((float)Nz-1.0f, ceil(kugel->pmax.z)+2.0f);
		ulong added = 0ull;
		for(uint tri=0u; tri<kugel->triangle_number; tri++) {
			const float3 &V0=kugel->p0[tri], &V1=kugel->p1[tri], &V2=kugel->p2[tri];
			const int tx0=(int)fmax((float)bx0, floor(fmin(fmin(V0.x,V1.x),V2.x))-1.0f), tx1=(int)fmin((float)bx1, ceil(fmax(fmax(V0.x,V1.x),V2.x))+1.0f);
			const int ty0=(int)fmax((float)by0, floor(fmin(fmin(V0.y,V1.y),V2.y))-1.0f), ty1=(int)fmin((float)by1, ceil(fmax(fmax(V0.y,V1.y),V2.y))+1.0f);
			const int tz0=(int)fmax((float)bz0, floor(fmin(fmin(V0.z,V1.z),V2.z))-1.0f), tz1=(int)fmin((float)bz1, ceil(fmax(fmax(V0.z,V1.z),V2.z))+1.0f);
			for(int z=tz0; z<=tz1; z++) for(int y=ty0; y<=ty1; y++) for(int x=tx0; x<=tx1; x++) {
				const ulong n = (ulong)x + ((ulong)y + (ulong)z*(ulong)Ny)*(ulong)Nx;
				if((lbm.flags[n]&TYPE_S)!=0u) continue;
				if(sat_tri_box_overlap(V0, V1, V2, (float)x, (float)y, (float)z, 0.5f)) { lbm.flags[n] = TYPE_S|TYPE_X; added++; }
			}
		}
		print_info("SAT-Schale: "+to_string(added)+" Zellen ergaenzt");

		// Void-Fill. Die SAT-Schale markiert nur die geschnittenen Zellen -- das Innere bleibt hohl,
		// und Upstreams Ray-Paritaet fuellt es hier nicht zuverlaessig (an der Kugel lieferte sie eine
		// nahezu leere Symmetrieebene). Ohne diesen Schritt ist die "Kugel" eine Hohlkugel mit
		// durchstroemtem Inneren: gemessen 224 statt ~1100 Solidzellen im Mittelschnitt, also
		// R_eff = 8,4 statt 18,8 Zellen. Flood-Fill vom Aussenrand der Bounding-Box; was von aussen
		// nicht erreichbar ist, ist eingeschlossen und damit solid.
		std::vector<bool> reach((size_t)((ulong)Nx*Ny*Nz), false);
		std::vector<ulong> stack;
		auto idx = [&](const int x, const int y, const int z) { return (ulong)x + ((ulong)y + (ulong)z*(ulong)Ny)*(ulong)Nx; };
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
		ulong filled = 0ull;
		for(int z=bz0; z<=bz1; z++) for(int y=by0; y<=by1; y++) for(int x=bx0; x<=bx1; x++) {
			const ulong n = idx(x,y,z);
			if((lbm.flags[n]&TYPE_X)==0u && !reach[(size_t)n]) { lbm.flags[n] = TYPE_S|TYPE_X; filled++; }
		}
		print_info("Void-Fill: "+to_string(filled)+" eingeschlossene Zellen als solid markiert");
		lbm.flags.write_to_device(); // Host-Aenderungen muessen aufs Device, bevor initialize() laeuft
	}

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
	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
		const ulong n = (ulong)x + ((ulong)y + (ulong)z*(ulong)Ny)*(ulong)Nx;
		if((lbm.flags[n]&(TYPE_S|TYPE_X))!=0u) continue;           // Kugel unangetastet
		if(z==0u || z==Nz-1u || y==0u || y==Ny-1u) {               // Boden, Decke, Seitenwaende
			lbm.flags[n] = TYPE_S; lbm.u.x[n] = u_wall; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
		} else if(x==0u || x==Nx-1u) {                             // Ein- und Auslass
			lbm.flags[n] = TYPE_E; lbm.u.x[n] = u_lat; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
		} else {
			lbm.u.x[n] = u_lat; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f;
		}
	}

	// ---------------------------------------------------------------- Laufsteuerung
	const float t_end    = env_f("CFD_T_END", 0.600f);      // physikalische Laufzeit [s]
	const float t_warmup = env_f("CFD_T_WARMUP", 0.200f);   // ab hier wird gemittelt [s]
	const uint  sample_every = env_u("CFD_SAMPLE_EVERY", 10u);
	const ulong n_steps  = (ulong)(t_end/dt + 0.5f);
	const string run_name = getenv("CFD_RUN_NAME") ? string(getenv("CFD_RUN_NAME")) : string("kugel");
	const string out_dir = get_exe_path()+"../export/"+run_name+"/";
	create_folder(out_dir);

	const float A_nom = (float)M_PI*(0.5f*D)*(0.5f*D);
	const float q_inf = 0.5f*si_rho*si_u*si_u;
	print_info("Laufzeit "+to_string(t_end,3u)+" s = "+to_string(n_steps)+" Schritte, Mittelung ab "+to_string(t_warmup,3u)+" s");
	print_info("A_nominal = "+to_string(A_nom,5u)+" m2, q_inf = "+to_string(q_inf,2u)+" Pa");

	// ---------------------------------------------------------------- Zeitschleife
	std::vector<double> ts, fx, fy, fz;
	ts.reserve(n_steps/sample_every + 2ull);
	fx.reserve(n_steps/sample_every + 2ull); fy.reserve(fx.capacity()); fz.reserve(fx.capacity());
	lbm.run(0u, n_steps); // initialisieren ohne Zeitschritt
	for(ulong step=0ull; step<n_steps; step+=(ulong)sample_every) {
		const ulong chunk = min((ulong)sample_every, n_steps-step);
		lbm.run(chunk, n_steps);
		lbm.update_force_field();
		const float3 F_lat = lbm.object_force(TYPE_S|TYPE_X);
		ts.push_back((double)((float)(step+chunk)*dt));
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
	if(cd_w.size()<16u) { print_warning("Zu wenige Samples im Mittelungsfenster fuer eine belastbare Statistik."); return; }

	double mcd=0.0, mcz=0.0;
	for(size_t i=0u; i<cd_w.size(); i++) { mcd+=cd_w[i]; mcz+=cz_w[i]; }
	mcd/=(double)cd_w.size(); mcz/=(double)cz_w.size();
	double sd=0.0; for(size_t i=0u; i<cd_w.size(); i++) sd += (cd_w[i]-mcd)*(cd_w[i]-mcd);
	sd = sqrt(sd/(double)cd_w.size());

	print_info("---------------------------------------------------------------");
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
	print_info("---------------------------------------------------------------");

	// Der Intel-Runtime-Teardown laeuft beim regulaeren Rueckweg in ein "double free or
	// corruption (out)" (rc=134), nachdem alle Dateien geschrieben sind. Upstream hat denselben
	// Exit-Bug an anderer Stelle und musste den "sauberen" Fix einen Tag spaeter zurueckrudern.
	_exit(0);
}
