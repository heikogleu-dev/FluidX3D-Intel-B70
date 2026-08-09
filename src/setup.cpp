#include "setup.hpp"
#include <fstream>
#include <cstring>
#include <filesystem>
#include <cstdio>
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

// Schalter-Helfer: gesetzt UND nicht ""/"0"/"false"/"off" = an.
// Das Muster getenv(X)!=nullptr schaltet bei CFD_X=0 EIN -- im alten Baum an 60 Stellen so.
static bool env_on(const char* name) {
	const char* v = getenv(name); if(v==nullptr) return false;
	return !(v[0]=='\0' || (v[0]=='0'&&v[1]=='\0') || strcmp(v,"false")==0 || strcmp(v,"off")==0);
}
static float env_f(const char* name, const float fallback) {
	const char* v = getenv(name); return v==nullptr ? fallback : (float)atof(v);
}
// ★ KORREKTUR 2026-08-08 (Pruefer-Befund M1): hier stand `max(1, atoi(v))`. Damit war CFD_PO_FACES=0
// nicht "kein Druck-Auslass", sondern Bit 1 = x_min = DER EINLASS. Ein Ausschalter, der einschaltet,
// und zwar ausgerechnet die eine Flaeche, die man nie meint. Kein Clamp mehr; wo eine 0 wirklich
// unzulaessig ist (Abtastweite), klemmt es die AUFRUFSTELLE -- dort weiss man, was 0 bedeuten wuerde.
static uint env_u(const char* name, const uint fallback) {
	const char* v = getenv(name); return v==nullptr ? fallback : (uint)max(0, atoi(v));
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
                          const float u2si, const float u_ref_si, const int t_ms, const string& dir, const string& tag) {
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
	string ms = to_string(t_ms); while(ms.length()<6u) ms = "0"+ms;
	write_png(dir+"schnitt_"+tag+"_"+ms+"ms.png", &img);
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

static // ---------------------------------------------------------------------------- Lauf-Sicherung
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
	if(abw_max>0.02f) print_warning(string(wo)+": der Impulsterm weicht um bis zu "+to_string(100.0f*abw_max,2u)
		+" % vom Sollwert u_lat/3 ab -- die Wandgeschwindigkeit stimmt nicht (Randzellen mit zwei Wandflaechen sind ausgenommen).");
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
	LBM_Domain::s_sponge_n = 0u; LBM_Domain::s_sponge_a = 3000.0f; LBM_Domain::s_sponge_wmin = 0.5f; // alle drei, damit keine Instanz einen Wert der vorigen erbt
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
	ts.reserve(n_steps/sample_every + 2ull);
	fx.reserve(n_steps/sample_every + 2ull); fy.reserve(fx.capacity()); fz.reserve(fx.capacity());
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
// ★ KORREKTUR 2026-08-08 (Pruefer-Befund): hier stand "Default ist deshalb dx=5mm" und "die
// F-Bounding-Box ist noch nicht portiert". Beides ueberholt -- der Default ist 4 mm, und die
// F-Bounding-Box wird 40 Zeilen weiter unten gesetzt (seit Commit 76c80be). Genau daran waere sonst
// jemand haengengeblieben, der die Speicherrechnung nachvollziehen will.
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
			z_offset_cells - (ctr.z - 0.5f*bb.z))); // Unterkante auf die Fahrbahn (z_offset_cells ist 0)
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
	// ★ Die Daempfungszone gibt es in diesem Fall NICHT, und das wird GESAGT statt still geschluckt:
	// die Zone rampt am DOMAENENRAND, und hier sind y+/y- und z+ mitbewegte Waende -- eine Zone dort
	// verdickte kuenstlich die Wandgrenzschichten. Sinnvoll ist sie nur in fernfeld und fahrzeug_dd,
	// dort nur am groben Gitter. Ein still wirkungsloser Schalter waere genau die Fehlerklasse,
	// die dieses Projekt sonst jagt.
	if(env_u("CFD_SPONGE_N", 0u)>0u) print_warning("CFD_SPONGE_N ist gesetzt, wird in diesem Fall aber NICHT angewandt (die Zone rampt am Domaenenrand, dort stehen hier mitbewegte Waende). Nur fernfeld und fahrzeug_dd nutzen sie.");
	LBM_Domain::s_sponge_n = 0u; LBM_Domain::s_sponge_a = 3000.0f; LBM_Domain::s_sponge_wmin = 0.5f; // alle drei, damit keine Instanz einen Wert der vorigen erbt
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
	const ulong n_steps  = (ulong)(t_end/dt + 0.5f);
	const string out_dir = get_exe_path()+"../export/"+(getenv("CFD_RUN_NAME")?string(getenv("CFD_RUN_NAME")):string("fahrzeug"))+"/";
	create_folder(out_dir);
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
		if(slice_dt>0.0f && (float)t_si>=slice_next) {
			slice_next = (float)t_si + slice_dt;
			lbm.u.read_from_device(); lbm.flags.read_from_device();
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
static void main_setup_fahrzeug_dd() {
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
	const float tau_f = 3.0f*nu_lat_f + 0.5f, tau_c = 3.0f*nu_lat_c + 0.5f;

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
	const float near_Lx = auf_grobe_zelle(env_f("CFD_NEAR_LX",  6.6560f));
	const float near_Ly = auf_grobe_zelle(env_f("CFD_NEAR_LY", 2.4800f));
	const float near_Lz = auf_grobe_zelle(env_f("CFD_NEAR_LZ", 1.9360f));
	// Weltkoordinaten nach V1-Konvention: die Fahrzeugnase liegt bei x = 0, der Einlass 0.6 Fahrzeug-
	// laengen davor. Das ist BEWUSST kurz -- Heiko 2026-08-08: der geringe Einlaufweg wirkt der toten
	// Stroemung in den unteren 5 bis 20 mm und der dadurch stagnierenden Unterbodenstroemung entgegen.
	// Wer das fuer einen Fehler haelt und "korrigiert", macht den Unterboden wieder falsch.
	const float far_x0  = env_f("CFD_FAR_X0", -0.6f*si_length);       // -2.66184 m
	const float near_off_x = auf_grobe_zelle(env_f("CFD_NEAR_OFF_X", 2.4320f)); // ebenfalls auf ganze grobe Zellen
	const float near_x0 = far_x0 + near_off_x;  // -0.22984 m bei dx_c = 16 mm, V1-Wert
	const float veh_x0  = 0.0f;                                      // Nase
	const float veh_x1  = veh_x0 + si_length;                        // Heck
	// Das Fahrzeug steht AUF der Fahrbahn (Heiko-Vorgabe). V1 liess es 16 mm schweben.
	const float veh_z0  = 0.001f*env_f("CFD_Z_OFFSET_MM", 0.0f);

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
	}
	print_info("tau_fein = "+to_string(tau_f,6u)+", tau_grob = "+to_string(tau_c,6u)+"  -- beide praktisch 0.5; TRT haelt die Wandlage trotzdem bei Lambda = 3/16.");
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
	LBM_Domain::s_sponge_n = 0u; LBM_Domain::s_sponge_a = 3000.0f; LBM_Domain::s_sponge_wmin = 0.5f; // alle drei, damit keine Instanz einen Wert der vorigen erbt
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
	// betraegt x- 152, x+ 199, y+- 162, z+ 430 Zellen. N=64 laesst ueberall >= 88 Zellen Luft.
	// Obergrenze N <= 120 (dann noch 32 Zellen Abstand zur Entnahmeebene x-); N=32/64 sind komfortabel.
	// Bewusst NACH lbm_f gesetzt und ohne Selbstruecksetzung: lbm_f wird ZUERST konstruiert, ein
	// read-once haette die Zone also genau der falschen Domaene gegeben.
	LBM_Domain::s_sponge_n = env_u("CFD_SPONGE_N", 0u);
	LBM_Domain::s_sponge_a = env_f("CFD_SPONGE_A", 3000.0f);
	LBM_Domain::s_sponge_wmin = env_f("CFD_SPONGE_WMIN", 0.5f);
	if(LBM_Domain::s_sponge_n>120u) print_error("CFD_SPONGE_N ueber 120 kaeme im Fernfeld der Kopplungs-Entnahmeebene x- (152 Zellen) zu nahe.");
	LBM lbm_c(uint3(cNx, cNy, cNz), nu_lat_c, dev_coarse);

	// ---------------------------------------------------------------- Voxelisieren, beide Gitter
	// Das Fahrzeug MUSS auch im groben Gitter stehen. Sonst traegt das Fernfeld die Verdraengung nicht,
	// und die Nahfeld-Raender bekaemen ungestoerte Anstroemung aufgepraegt -- der Wagen staende dann in
	// einer Stroemung, die nicht weiss, dass er da ist.
	// ★ Bewusste Asymmetrie (Pruefer-Befund 2026-08-08): hier wird nur flags zurueckgelesen, nicht u.
	// Der Voxelisierungs-Kernel schreibt geraeteseitig AUCH u (die Koerpergeschwindigkeit), und
	// initialize() laedt spaeter das Host-u hoch und ueberschreibt das bedingungslos. Heute folgenlos,
	// weil das Fahrzeug steht und beide Seiten 0 sind. WER DEM FAHRZEUG JE EINE GESCHWINDIGKEIT GIBT,
	// muss hier zusaetzlich u.read_from_device() rufen -- sonst ist sie lautlos weg, und initialize()
	// setzt dann auch kein TYPE_MS an der Fahrzeugwand, die Wand waere also nicht mitbewegt.
	// Nicht vorsorglich eingebaut, weil das Ruecklesen von u 6 GB ueber PCIe kostet und heute nichts tut.
	lbm_f.voxelize_mesh_on_device(veh_f, TYPE_S|TYPE_X); lbm_f.flags.read_from_device();
	sat_shell_and_void_fill(lbm_f, veh_f, fNx, fNy, fNz);
	lbm_c.voxelize_mesh_on_device(veh_c, TYPE_S|TYPE_X); lbm_c.flags.read_from_device();
	sat_shell_and_void_fill(lbm_c, veh_c, cNx, cNy, cNz);

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
	lbm_f.set_pressure_outlet_faces(2u, env_f("CFD_PO_RHO", 1.0f)); // Bit 2 = x_max
	lbm_c.set_pressure_outlet_faces(2u, env_f("CFD_PO_RHO", 1.0f));
	lbm_f.finalize_sparse_tiles();
	lbm_c.finalize_sparse_tiles();

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

	// ---------------------------------------------------------------- Laufsteuerung
	const float t_flush  = (float)(cNx-1u)*dx_c/si_u; // Durchspuelung des FERNfelds (far_x0 kuerzt sich weg)
	const float t_end    = env_f("CFD_T_END", 2.0f*t_flush);
	const float t_warmup = env_f("CFD_T_WARMUP", 1.0f*t_flush);
	const ulong n_outer  = (ulong)(t_end/dt_c + 0.5f);
	const uint  sample_every = max(1u, env_u("CFD_SAMPLE_EVERY", 25u)); // in groben Schritten
	const float slice_dt = env_f("CFD_SLICE_DT", 0.010f); // alle 10 ms (Heiko); 0 = aus
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
	lbm_c.alloc_coupling_planes(max_cp); // entnimmt
	lbm_f.alloc_coupling_planes(max_cp); // empfaengt dieselben Ebenen

	std::vector<float> face[5];
	lbm_c.run(1u); // ein grober Schritt, damit ein Zustand zum Entnehmen existiert
	// ★ KORREKTUR 2026-08-08 (Pruefer-Befund): hier lief die Schleife ueber ALLE FUENF Ebenen, obwohl
	// x+ gar nicht getrieben wird (dort steht der Druckauslass). Die grobe x+-Ebene wurde also jeden
	// groben Schritt entnommen -- mit blockierendem finish_queue, 304 kB Device-Read und einer
	// Host-Kopierschleife -- und dann von niemandem gelesen. Genau das Muster, das diesen Baum
	// ueberhaupt noetig gemacht hat: extrahiert, transferiert, weggeworfen.
	for(uint p=0u; p<5u; p++) if(drive_face[p]) lbm_c.extract_plane_macros(cp[p], face[p]);

	// ---------------------------------------------------------------- Zeitschleife
	// Der grobe Schritt laeuft ASYNCHRON auf dem zweiten Geraet, waehrend das Nahfeld seine ratio
	// Schritte rechnet.
	// ★ KORREKTUR 2026-08-08 (Pruefer-Befund): hier stand "Verzug". Es ist ein VORLAUF. Vor der
	// Schleife macht das grobe Gitter einen Schritt, danach wird entnommen; das Nahfeld durchlaeuft
	// also das Intervall [k, k+1] mit dem groben Zustand an dessen ENDE. Am Intervallanfang eilt der
	// Rand um dt_c voraus, am Ende stimmt er. Der Betrag ist derselbe -- bei dt_c = 4e-5 s wandert
	// die Stroemung 1,2 mm, weniger als eine feine Zelle --, aber die Richtung war falsch beschrieben.
	// Als Praediktor-Halteschema ist ein Vorlauf gegenueber einem Nachlauf eher guenstiger.
	std::vector<double> ts, fx, fz, fx_c;
	float slice_next = 0.0f;
	Clock outer_clock; double t_acc = 0.0; ulong n_acc = 0ull;
	double Fx_prev = 1e300, Fz_prev = 1e300; uint n_frozen = 0u; // fuer den Einfrier-Test, siehe Zeitschleife
	// ★ Pruefer-Befund 2026-08-08: die CSV entstand bisher ERST NACH der Schleife. Bei 2,5 Stunden
	// Laufzeit heisst das: ein Abbruch durch Treiber, Speicher, Stromausfall oder Strg-C kostet
	// SAEMTLICHE Samples. Jetzt wird jede Zeile sofort geschrieben und geleert -- die Datei ist damit
	// zu jedem Zeitpunkt vollstaendig bis zum letzten Sample, und ein abgebrochener Lauf bleibt
	// auswertbar. Die Vektoren bleiben zusaetzlich fuer die Statistik am Ende.
	std::ofstream fcsv(out_dir+"forces.csv"); fcsv.precision(8);
	fcsv << "time_s,Fx_N,Fz_N,Cd,Cz,Fx_far_N\n" << std::flush;
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
		for(uint p=0u; p<5u; p++) if(drive_face[p]) lbm_f.drive_boundary_from_coarse(fp[p], face[p], cp[p].extent_a, cp[p].extent_b, ratio);
		const auto _t1 = t_now();
		lbm_f.run((ulong)ratio, n_outer*(ulong)ratio);
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
		for(uint p=0u; p<5u; p++) if(drive_face[p]) lbm_c.extract_plane_macros(cp[p], face[p]); // nur die vier getriebenen Flaechen -- x+ ist Druckauslass, siehe oben
		const auto _t3 = t_now();
		t_acc += outer_clock.stop(); n_acc++;
		ph_kopplung += std::chrono::duration<double>(_t1-_t0).count();
		ph_fein     += std::chrono::duration<double>(_t2-_t1).count();
		ph_grob     += std::chrono::duration<double>(_t3-_t2).count();
		ph_n++;

		if((outer+1ull)%(ulong)sample_every==0ull) {
			const auto _t4 = t_now();
			lbm_f.update_force_field();
			const float3 F = lbm_f.object_force(TYPE_S|TYPE_X);
			lbm_c.update_force_field();
			const float3 Fc = lbm_c.object_force(TYPE_S|TYPE_X);
			const double t_si = (double)((float)(outer+1ull)*dt_c);
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
			// keine Warnung. Der Lauf lief zwei Stunden lang tot weiter. V1 fror am selben Punkt bei
			// 343 640 N ein: praktisch derselbe Wert, also derselbe Mechanismus. Das ist keine Kraft,
			// sondern die auf die Zahlenformatgrenze gelaufene DDF-Ablage.
			// Drei Tests statt einem. Zwei bitgleiche Kraftwerte hintereinander gibt es in einer
			// abgeloesten Stroemung nicht; drei sind ein Beweis.
			{
				const double Fxf = (double)units_coarse.si_F(Fc.x);
				const double q_A = (double)q_inf*A_ref;
				string grund = "";
				if(!std::isfinite(Fx_si) || !std::isfinite(Fz_si) || !std::isfinite(Fxf)) grund = "die Kraft ist keine Zahl mehr";
				else if(n_frozen>=2u) grund = "die Kraft steht seit drei Abtastungen BITGLEICH -- das Feld ist eingefroren (Zahlenformat gesaettigt)";
				else if(t_si>0.02 && (fabs(Fx_si)>20.0*q_A || fabs(Fz_si)>20.0*q_A)) grund = "die Kraft ist unphysikalisch gross (|Cd| oder |Cz| ueber 20)";
				if(grund!="") {
					fcsv << std::flush; fcsv.close();
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
			const auto _t5 = t_now();
			if(slice_dt>0.0f && (float)t_si>=slice_next) {
				slice_next = (float)t_si + slice_dt;
				const int t_ms = (int)((float)t_si*1000.0f+0.5f);
				lbm_f.u.read_from_device(); lbm_f.flags.read_from_device();
				render_yslice(lbm_f, fNx, fNy, fNz, fNy/2u, si_u/u_lat, si_u, t_ms, out_dir, "nah");
				lbm_c.u.read_from_device(); lbm_c.flags.read_from_device();
				render_yslice(lbm_c, cNx, cNy, cNz, cNy/2u, si_u/u_lat, si_u, t_ms, out_dir, "fern");
				print_info("[SLICE] t = "+to_string((float)t_si,3u)+" s");
			}
			ph_kraft += std::chrono::duration<double>(_t5-_t4).count();
			ph_schnitt += std::chrono::duration<double>(t_now()-_t5).count();

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
	}
	if(n_acc>0ull) print_info("Mittlere Zeit je grobem Schritt: "+to_string((float)(t_acc/(double)n_acc),4u)+" s ("+to_string(ratio)+" feine Schritte inklusive)");

	// ---------------------------------------------------------------- Auswertung
	fcsv.close(); // die Zeilen stehen bereits einzeln auf Platte, siehe Schleife
	print_info("CSV: "+out_dir+"forces.csv ("+to_string((uint)ts.size())+" Zeilen, waehrend des Laufs geschrieben)");
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
	print_info("  Cd = "+to_string((float)mcd,4u)+"   (OpenFOAM 13: 0.599, Abweichung "+to_string((float)(100.0*(mcd/0.599-1.0)),1u)+" %)");
	print_info("  Cz = "+to_string((float)mcz,4u)+"   (OpenFOAM 13: -1.301, Abweichung "+to_string((float)(100.0*(mcz/-1.301-1.0)),1u)+" %)");
	for(uint k : {4u, 8u, 16u}) { const double se=block_sem(cd,k); if(se>=0.0) print_info("      Block-SEM Cd ueber "+to_string(k)+" Bloecke: +- "+to_string((float)se,5u)); }
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
	// ★ Nachpruefer-Befund 2026-08-09: die Obergrenze stand hier nur im Kommentar, geprueft wurde sie
	// nur im dd-Fall. Damit lief CFD_SPONGE_N=400 im Diagnosefall ungeprueft durch.
	if(LBM_Domain::s_sponge_n>120u) print_error("CFD_SPONGE_N ueber 120 ist nicht vorgesehen (im dd-Fall kaeme die Zone der Kopplungs-Entnahmeebene x- bei 152 Zellen zu nahe; der Diagnosefall bleibt vergleichbar).");
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
	print_info("CSV: "+out_dir+"rauschen.csv");
	_exit(0);
}

void main_setup() { // Fallauswahl: CFD_CASE=kugel (Default), fahrzeug, fahrzeug_dd oder fernfeld
	const char* c = getenv("CFD_CASE");
	if(c!=nullptr && string(c)=="fernfeld") main_setup_fernfeld();
	if(c!=nullptr && string(c)=="fahrzeug_dd") main_setup_fahrzeug_dd();
	else if(c!=nullptr && string(c)=="fahrzeug") main_setup_fahrzeug();
	else main_setup_kugel();
}
