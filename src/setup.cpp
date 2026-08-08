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


// Ein y-Schnitt als PNG. Blau = langsam, weiss = Anstroemung, rot = schnell; Solid schwarz.
// u und flags muessen vorher vom Device gelesen sein -- der Aufrufer entscheidet, wie oft das passiert,
// denn es kostet einen vollen Transfer.
static void render_yslice(LBM& L, const uint Nx, const uint Ny, const uint Nz, const uint y_slice,
                          const float u2si, const float u_ref_si, const int t_ms, const string& dir) {
	Image img(Nx, Nz);
	for(uint z=0u; z<Nz; z++) for(uint x=0u; x<Nx; x++) {
		const ulong n = (ulong)x + (ulong)Nx*((ulong)y_slice + (ulong)Ny*(ulong)z);
		int col;
		if((L.flags[n]&(TYPE_S|TYPE_X))!=0u) col = 0x000000; // Solid
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
	const string sdir = dir+"slices/"; create_folder(sdir);
	string ms = to_string(t_ms); while(ms.length()<6u) ms = "0"+ms;
	write_png(sdir+"slice_y"+to_string(y_slice)+"_"+ms+"ms.png", &img);
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
	ulong filled = 0ull;
	for(int z=bz0; z<=bz1; z++) for(int y=by0; y<=by1; y++) for(int x=bx0; x<=bx1; x++) {
		const ulong n = idx(x,y,z);
		if((lbm.flags[n]&TYPE_X)==0u && !reach[(size_t)n]) { lbm.flags[n] = TYPE_S|TYPE_X; filled++; }
	}
	print_info("Void-Fill: "+to_string(filled)+" eingeschlossene Zellen als solid markiert");
	// Kein write_to_device() noetig: LBM::initialize() laedt rho, u und flags beim ersten run() hoch.
}

static void main_setup_kugel() {
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
		const uint T = env_u("CFD_TILE", 8u);
		if(T!=8u && T!=16u && T!=32u && T!=64u) print_error("CFD_TILE muss 8, 16, 32 oder 64 sein (erhalten: "+to_string(T)+").");
		LBM_Domain::s_sparse_T = T;
		print_info("Block-Tiling AKTIV, T="+to_string(T)+" (VRAM sparen auf Kosten von Durchsatz)");
	}

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
		// Kein update_fields() mehr noetig: UPDATE_FIELDS ist eingeschaltet (defines.hpp), stream_collide
		// schreibt u und rho jeden Schritt selbst. Damit sieht der Druck-Auslass das aktuelle Innenfeld
		// statt eines bis zu CFD_SAMPLE_EVERY Schritte alten -- und die Slices zeigen den echten Zustand.
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
// Default ist deshalb dx=5mm (257 M Zellen, 17.2 GB). Der Weg zu 4mm waere eine F-Bounding-Box
// (F nur um das Fahrzeug statt ueber die ganze Domaene) -- der alte Baum hatte die, sie ist noch
// nicht portiert.
// =============================================================================================
static void main_setup_fahrzeug() {
	const float si_u      = 30.0f;
	const float si_rho    = 1.225f;
	const float si_nu     = env_f("CFD_NU", 1.48e-5f);
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
	// Fahrzeug steht AUF dem Boden: Unterkante der STL auf die Bodenebene z=0. Der alte Baum liess es
	// 16 mm schweben (4 Zellen bei 4 mm) -- ein rein numerischer Versatz ohne physikalische Entsprechung,
	// der den Unterbodenspalt kuenstlich vergroessert und damit genau die Groesse verfaelscht, um die es
	// beim Abtrieb geht. CFD_Z_OFFSET_MM stellt den alten Zustand her, falls man A/B fahren will.
	const float z_offset_cells = 0.001f*env_f("CFD_Z_OFFSET_MM", 0.0f)/dx;

	LBM_Domain::s_sparse_tiles_on = env_on("CFD_SPARSE_TILES");
	if(LBM_Domain::s_sparse_tiles_on) {
		const uint T = env_u("CFD_TILE", 8u);
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
			z_offset_cells - (ctr.z - 0.5f*bb.z))); // Unterkante 16 mm ueber dem Boden
	}
	{	// F-Box = Fahrzeug plus Rand. Der Rand muss die wandnahen Fluidzellen mitnehmen, auf die
		// update_force_field schreibt; 4 Zellen decken die Reichweite von load_f (hoechstens 2) sicher ab.
		const uint M = 4u;
		const uint x0=(uint)fmax(0.0f, veh->pmin.x-(float)M), x1=(uint)fmin((float)Nx-1.0f, veh->pmax.x+(float)M);
		const uint y0=(uint)fmax(0.0f, veh->pmin.y-(float)M), y1=(uint)fmin((float)Ny-1.0f, veh->pmax.y+(float)M);
		const uint z0=(uint)fmax(0.0f, veh->pmin.z-(float)M), z1=(uint)fmin((float)Nz-1.0f, veh->pmax.z+(float)M);
		LBM_Domain::set_force_bbox(x0, y0, z0, x1-x0+1u, y1-y0+1u, z1-z0+1u);
	}

	units.set_m_kg_s(si_length/dx, u_lat, 1.0f, si_length, si_u, si_rho);
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
	// Boden mitbewegt (Rollstrasse), Decke und Seiten Freistrom, x- Einlass, x+ Auslass.
	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
		const ulong n = (ulong)x + ((ulong)y + (ulong)z*(ulong)Ny)*(ulong)Nx;
		if((lbm.flags[n]&(TYPE_S|TYPE_X))!=0u) continue;
		if(z==0u) { lbm.flags[n] = TYPE_S; lbm.u.x[n] = u_lat; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f; } // Rollstrasse
		else if(x==0u || x==Nx-1u || y==0u || y==Ny-1u || z==Nz-1u) {
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
	const uint  sample_every = env_u("CFD_SAMPLE_EVERY", 10u);
	const float slice_dt = env_f("CFD_SLICE_DT", 0.0f); // 0 = keine Slices
	const ulong n_steps  = (ulong)(t_end/dt + 0.5f);
	const string out_dir = get_exe_path()+"../export/"+(getenv("CFD_RUN_NAME")?string(getenv("CFD_RUN_NAME")):string("fahrzeug"))+"/";
	create_folder(out_dir);
	const float q_inf = 0.5f*si_rho*si_u*si_u;
	const uint y_mid = Ny/2u;
	print_info("Laufzeit "+to_string(t_end,3u)+" s = "+to_string(n_steps)+" Schritte, Mittelung ab "+to_string(t_warmup,3u)+" s");

	std::vector<double> ts, fx, fz;
	float slice_next = 0.0f;
	lbm.run(0u, n_steps);
	for(ulong step=0ull; step<n_steps; step+=(ulong)sample_every) {
		const ulong chunk = min((ulong)sample_every, n_steps-step);
		lbm.run(chunk, n_steps);
		lbm.update_force_field();  // u/rho schreibt stream_collide selbst (UPDATE_FIELDS)
		const float3 F = lbm.object_force(TYPE_S|TYPE_X);
		const double t_si = (double)((float)(step+chunk)*dt);
		ts.push_back(t_si); fx.push_back((double)units.si_F(F.x)); fz.push_back((double)units.si_F(F.z));
		if(slice_dt>0.0f && (float)t_si>=slice_next) {
			slice_next = (float)t_si + slice_dt;
			lbm.u.read_from_device(); lbm.flags.read_from_device();
			render_yslice(lbm, Nx, Ny, Nz, y_mid, si_u/u_lat, si_u, (int)((float)t_si*1000.0f+0.5f), out_dir);
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
	if(cd.size()<16u) { print_warning("Zu wenige Samples fuer eine belastbare Statistik."); _exit(0); }
	double mcd=0.0, mcz=0.0;
	for(size_t i=0u; i<cd.size(); i++) { mcd+=cd[i]; mcz+=cz[i]; }
	mcd/=(double)cd.size(); mcz/=(double)cz.size();
	print_info("---------------------------------------------------------------");
	print_info("Zeitmittel ab "+to_string(t_warmup,3u)+" s ueber "+to_string((uint)cd.size())+" Samples:");
	print_info("  Cd = "+to_string((float)mcd,4u)+"   (OF13: 0.599, Abweichung "+to_string((float)(100.0*(mcd/0.599-1.0)),1u)+" %)");
	print_info("  Cz = "+to_string((float)mcz,4u)+"   (OF13: -1.301, Abweichung "+to_string((float)(100.0*(mcz/-1.301-1.0)),1u)+" %)");
	print_info("  Block-SEM von Cd (ehrlich ist die Zahl bei WENIGEN Bloecken):");
	for(uint k : {4u, 8u, 16u}) { const double se=block_sem(cd,k); if(se>=0.0) print_info("      "+to_string(k)+" Bloecke: +- "+to_string((float)se,5u)); }
	print_info("---------------------------------------------------------------");
	_exit(0);
}

void main_setup() { // Fallauswahl: CFD_CASE=kugel (Default) oder fahrzeug
	const char* c = getenv("CFD_CASE");
	if(c!=nullptr && string(c)=="fahrzeug") main_setup_fahrzeug();
	else main_setup_kugel();
}
