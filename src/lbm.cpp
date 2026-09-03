#include "lbm.hpp"
#include <atomic> // C2: Zaehler fuer CFD_DUMP_CL-Dateinamen

// ★ F-Null-Read-Gate (Perf-Audit Achse 1, Rang 3): EIN Praedikat fuer Emission, Ansage und
// F-Waechter -- nie dreimal getrennt auswerten (Drift-Schutz). Default AN.
static bool f_nur_solid_an() { const char* e = getenv("CFD_F_NUR_SOLID"); return e==nullptr||e[0]=='\0'||atoi(e)>0; } // leer gesetzt = Default AN (Pruefagent NIEDRIG-2)

Units units; // for unit conversion

#if defined(D2Q9)
const uint velocity_set = 9u;
const uint dimensions = 2u;
const uint transfers = 3u;
#elif defined(D3Q15)
const uint velocity_set = 15u;
const uint dimensions = 3u;
const uint transfers = 5u;
#elif defined(D3Q19)
const uint velocity_set = 19u;
const uint dimensions = 3u;
const uint transfers = 5u;
#elif defined(D3Q27)
const uint velocity_set = 27u;
const uint dimensions = 3u;
const uint transfers = 9u;
#endif // D3Q27

uint bytes_per_cell_host() { // returns the number of Bytes per cell allocated in host memory
	uint bytes_per_cell = 17u; // rho, u, flags
#ifdef FORCE_FIELD
	bytes_per_cell += 12u; // F
#endif // FORCE_FIELD
#ifdef SURFACE
	bytes_per_cell += 4u; // phi
#endif // SURFACE
#ifdef TEMPERATURE
	bytes_per_cell += 4u; // T
#endif // TEMPERATURE
	return bytes_per_cell;
}
uint bytes_per_cell_device() { // returns the number of Bytes per cell allocated in device memory
	uint bytes_per_cell = velocity_set*sizeof(fpxx)+17u; // fi, rho, u, flags
#ifdef FORCE_FIELD
	bytes_per_cell += 12u; // F
#endif // FORCE_FIELD
#ifdef SURFACE
	bytes_per_cell += 12u; // phi, mass, flags
#endif // SURFACE
#ifdef TEMPERATURE
	bytes_per_cell += 7u*sizeof(fpxx)+4u; // gi, T
#endif // TEMPERATURE
	return bytes_per_cell;
}
uint bandwidth_bytes_per_cell_device() { // returns the bandwidth in Bytes per cell per time step from/to device memory
	uint bandwidth_bytes_per_cell = velocity_set*2u*sizeof(fpxx)+1u; // lattice.set()*2*fi, flags
#ifdef UPDATE_FIELDS
	bandwidth_bytes_per_cell += 16u; // rho, u
#ifdef TEMPERATURE
	bandwidth_bytes_per_cell += 4u; // T
#endif // TEMPERATURE
#endif // UPDATE_FIELDS
#ifdef FORCE_FIELD
	bandwidth_bytes_per_cell += 12u; // F
#endif // FORCE_FIELD
#if defined(MOVING_BOUNDARIES)||defined(SURFACE)||defined(TEMPERATURE)
	bandwidth_bytes_per_cell += (velocity_set-1u)*1u; // neighbor flags have to be loaded
#endif // MOVING_BOUNDARIES, SURFACE or TEMPERATURE
#ifdef SURFACE
	bandwidth_bytes_per_cell += (1u+(2u*velocity_set-1u)*sizeof(fpxx)+8u+(velocity_set-1u)*4u) + 1u + 1u + (4u+velocity_set+4u+4u+4u); // surface_0 (flags, fi, mass, massex), surface_1 (flags), surface_2 (flags), surface_3 (rho, flags, mass, massex, phi)
#endif // SURFACE
#ifdef TEMPERATURE
	bandwidth_bytes_per_cell += 7u*2u*sizeof(fpxx); // 2*gi
#endif // TEMPERATURE
	return bandwidth_bytes_per_cell;
}
uint3 resolution(const float3 box_aspect_ratio, const uint memory) { // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
#ifndef D2Q9
	float memory_required = (box_aspect_ratio.x*box_aspect_ratio.y*box_aspect_ratio.z)*(float)bytes_per_cell_device()/1048576.0f; // in MB
	float scaling = cbrt((float)memory/memory_required);
	return uint3(to_uint(scaling*box_aspect_ratio.x), to_uint(scaling*box_aspect_ratio.y), to_uint(scaling*box_aspect_ratio.z));
#else // D2Q9
	float memory_required = (box_aspect_ratio.x*box_aspect_ratio.y)*(float)bytes_per_cell_device()/1048576.0f; // in MB
	float scaling = sqrt((float)memory/memory_required);
	return uint3(to_uint(scaling*box_aspect_ratio.x), to_uint(scaling*box_aspect_ratio.y), 1u);
#endif // D2Q9
}

string default_filename(const string& path, const string& name, const string& extension, const ulong t) { // generate a default filename with timestamp
	string time = "00000000"+to_string(t);
	time = substring(time, length(time)-9u, 9u);
	return (path=="" ? get_exe_path()+"export/" : path)+create_file_extension((name=="" ? "file" : name)+"-"+time, extension);
}
string default_filename(const string& name, const string& extension, const ulong t) { // generate a default filename with timestamp at exe_path/export/
	return default_filename("", name, extension, t);
}



LBM_Domain::LBM_Domain(const Device_Info& device_info, const uint Nx, const uint Ny, const uint Nz, const uint Dx, const uint Dy, const uint Dz, const int Ox, const int Oy, const int Oz, const float nu, const float fx, const float fy, const float fz, const float sigma, const float alpha, const float beta, const uint particles_N, const float particles_rho) { // constructor with manual device selection and domain offset
	this->Nx = Nx; this->Ny = Ny; this->Nz = Nz;
	this->Dx = Dx; this->Dy = Dy; this->Dz = Dz;
	this->Ox = Ox; this->Oy = Oy; this->Oz = Oz;
	this->nu = nu;
	this->fx = fx; this->fy = fy; this->fz = fz;
	this->sigma = sigma;
	this->alpha = alpha; this->beta = beta;
	this->particles_N = particles_N;
	this->particles_rho = particles_rho;
	// FORK -- F-Bounding-Box HIER aufloesen, nicht erst in allocate(). Der Konstruktor baut den
	// OpenCL-Code (und damit def_FBNX/def_FBN) weiter unten; allocate() laeuft erst DANACH. Stand die
	// Aufloesung in allocate(), wurde def_FBNX = 0 emittiert, f_bbox lieferte ueberall false und
	// SAEMTLICHE Kraefte kamen als exakt null heraus -- auch im Voll-Domaenen-Fall, wo die Box die
	// Identitaet sein muesste. Genau so ist es passiert und so wurde es gefunden.
	if(s_fbbox[3]>0u && s_fbbox[4]>0u && s_fbbox[5]>0u) {
		fbx0=s_fbbox[0]; fby0=s_fbbox[1]; fbz0=s_fbbox[2]; fbnx=s_fbbox[3]; fbny=s_fbbox[4]; fbnz=s_fbbox[5];
	} else { fbx0=0u; fby0=0u; fbz0=0u; fbnx=Nx; fbny=Ny; fbnz=Nz; }
	for(uint i=0u; i<6u; i++) s_fbbox[i]=0u; // read-once: eine zweite Domaene erbt die Box nicht
	// ★ 2026-08-08, beim Bau der Doppel-Domaene gefunden: das Block-Tiling wurde bis hier ueberall direkt
	// aus den STATISCHEN Schaltern gelesen -- auch in finalize_sparse_tiles(), das erst LANGE nach dem
	// Konstruktor laeuft. Mit zwei Domaenen ist das eine Falle: wer den Schalter zwischen den beiden
	// Konstruktoren umlegt, aendert rueckwirkend das Verhalten der ERSTEN Domaene, deren fi dann als
	// 1-Zellen-Platzhalter stehen bliebe. Deshalb jetzt genauso read-once wie die F-Bounding-Box:
	// hier einmal in Domaenen-Felder uebernehmen, danach lesen ausschliesslich diese.
#ifndef D3Q19
	// ★ Nachpruefer-Befund 2026-08-15: der "D3Q19-Guard" fuer CFD_SGS_WANDFREI war nur ein KOMMENTAR.
	// Der Schalter ist Laufzeit, kein #error kann ihn fangen -- deshalb hart zur Laufzeit: die
	// Wanderkennung benutzt die festen Flaechennachbarn j[1..6], die nur in D3Q19 stimmen.
	if(s_sgs_wandfrei) print_error("CFD_SGS_WANDFREI ist nur fuer D3Q19 gebaut (feste Flaechennachbarn j[1..6]).");
	if(s_wandfunktion) print_error("CFD_WANDFUNKTION ist nur fuer D3Q19 gebaut (feste Diagonalpaare 9/16, 11/18, 15/10, 17/12).");
	if(s_facetten) print_error("CFD_FACETTEN ist nur fuer D3Q19 gebaut (Paartabelle FACETTEN-STUFE2.md).");
#endif // D3Q19
#ifndef FORCE_FIELD
	if(s_facetten) print_error("CFD_FACETTEN braucht FORCE_FIELD (f_bbox-Indexfeld).");
#endif // FORCE_FIELD
	// ★ Pruefbefund F1 (2026-08-25): diese Guards standen erst im #ifndef-D3Q19-, dann im #ifndef-FORCE_FIELD-Block und waren im
	// Produktionsbuild FUNKTIONAL TOT. Jetzt unbedingt. alpha-Sperre entfaellt (Revision W2:
	// die Blende ist rein geometrisch, Additivterm+alpha bleiben und ihre Mathematik gilt exakt).
	if(s_fac_elibb&&(s_fac_ema>0.0f||s_fac_pema>0.0f)) print_error("CFD_FAC_ELIBB mit EMA/PEMA ist nicht definiert (Filter mischen Blende und Additivpfad) -- Messarm rein halten.");
#if defined(FORCE_FIELD)&&!defined(PARTICLES)
	// ★ Ansage-Doktrin F-Null-Read-Gate (Auditor-B B-3-Muster: Default-AN-Verhalten muss im Log stehen).
	if(f_nur_solid_an()) print_info("F-NUR-SOLID aktiv (Default): stream_collide liest F nicht -- F ist an Nicht-Solid-Zellen konstant 0 (F-Waechter prueft das bei initialize()). CFD_F_NUR_SOLID=0 stellt den Upstream-Read her.");
	else print_warning("CFD_F_NUR_SOLID=0: stream_collide liest F an jeder Fluidzelle (Upstream-Pfad, 12 B/Zelle/Schritt in der F-BBox) -- nur fuer A/B-Kontrollarme gedacht.");
#endif
	if(s_fac_elibb&&s_fac_apg!=0.0f) print_error("CFD_FAC_ELIBB mit APG ist nicht gebaut -- Messarm rein halten.");
	if(s_fac_utkorr!=1.0f) print_info("ABTASTPUNKT-MESSARM aktiv: CFD_FAC_UTKORR = "+to_string(s_fac_utkorr,3u)+" auf dem Wandmodell-Eingang (Theorie-Soll 3/2; Ansage-Doktrin).");
	if(s_fac_kappa!=0.4f) print_info("Grazing-Guard geaendert: CFD_FAC_KAPPA = "+to_string(s_fac_kappa,2u)+" (Default 0,4).");
	if(s_fac_qdiag!=0u) print_warning("CFD_FAC_QDIAG = "+to_string((ulong)s_fac_qdiag)+" -- DIAGNOSEARM (2 = nur q<0,5, 3 = nur q>0,5; Arm 1 ist seit K1' ohne Funktion). Kein Messarm fuer Abnahmen.");
	if(s_fac_elibb&&s_fac_imem) print_info("ELIBB 18-Link AKTIV (B2/B3, q>0,5 seit 26.08. als MLS-Blende chi=(2q-1)/(tau0+0,5)): rein geometrische q-Blende (u_W=0) + bestehender Additivterm; q=0,5 ist bitgleich iMEM. Wirkpfad Slot 67 (Blende gesamt), Slot 68 (MLS-q>0,5-Zweig). Blenden-Austausch wird seit B3 in fac_tau[1..5] GEBUCHT; seit dem Buchungsschluss (27.08.) buchen auch die Gate-Rueckfaelle (Slot 69, P-only; offen bleiben nur Slot 9 ut~0 und PEMA-Slot 17 mit reiner Kopfbuchung) -- Reibungspfad und object_force sind damit an allen GATE-Rueckfaellen EIN Bild.");
	// C1b: WFB und Facetten am selben Einfuegepunkt schliessen sich aus -- hart, kein stilles Nacheinander.
	if(s_facetten&&s_wandfunktion) print_error("CFD_FACETTEN und CFD_WANDFUNKTION gleichzeitig ist nicht definiert -- genau einen Pfad waehlen.");
	facetten_on = s_facetten;
#ifndef SUBGRID
	// ★ Audit-Nacharbeit 2: mit abgeschaltetem SUBGRID (Kugel-Validierung!) war der Schalter ein
	// lautloser No-Op -- jetzt harte Abweisung. Die WANDFUNKTION dagegen ist von SUBGRID unabhaengig
	// und wird seit derselben Nacharbeit ausserhalb des SUBGRID-Blocks emittiert.
	if(s_sgs_wandfrei) print_error("CFD_SGS_WANDFREI ohne SUBGRID ist sinnlos (es gaebe kein nu_t zu entfernen).");
	if(s_sgs_diag) print_error("CFD_SGS_DIAG ohne SUBGRID ist sinnlos (es gaebe kein nu_t zu messen).");
	if(s_sgs_fdwand>0u) print_error("CFD_SGS_FDWAND ohne SUBGRID ist sinnlos (der w-Ersatz-Hook liegt im SUBGRID-Block und waere still tot).");
	if(!s_sgs_guo) print_warning("CFD_SGS_GUO=0: Pi^neq OHNE Guo-Korrektur -- die Scherrate ist dort verzerrt, wo die Volumenkraft wirkt (Kontrollarm, nicht die Physik).");
#endif // SUBGRID
	// R2-Nachpruefer: Ansage NACH den harten Abweisern (vorher stand "aktiv" eine Zeile vor dem exit)
	if(s_sgs_wandfrei) print_info("SGS_WANDFREI aktiv: kein nu_t in Zellen mit solidem Flaechennachbarn (Wirkpfad-Zaehler Slot 6, Report am Laufende).");
	// ★ 03.09. (Planungsagent-Befund + Pruefagent-Vorschlag 6): der SGS_DIAG-Block lag im else-Zweig des FDWAND-Lesers und war unter
	// FDWAND an allen Facettenzellen ein stiller No-Op (Bins 35-48 leer). Seit 03.09. nachmittags steht er HINTER dem if/else und
	// misst das finale w -- die Kombination ist jetzt das Instrument fuer "nu_t IST gegen kappa*y+ SOLL" an Wandzellen.
	if(s_sgs_diag&&s_sgs_fdwand>0u) print_info("CFD_SGS_DIAG x CFD_SGS_FDWAND: DIAG misst an Facettenzellen das FD-nu_t aus fac_wfd (Block hinter dem if/else seit 03.09.; vorher stiller No-Op der Bins 35-48).");
	// ★ 03.09. UTKORR-Ansage fuer NACHBAR. Der No-Op-Waechter (NACHBAR/KDIAG ohne iMEM) sitzt an den LESESTELLEN in setup.cpp: dort
	// wird der Schalter bei fc<3 still auf 0 gesetzt, hier waere er schon unsichtbar (Rauchtest xs_guard_nb_ohne_imem 03.09.: rc=0).
	if(s_fac_nachbar>0u) print_info("NACHBARABTASTUNG aktiv (CFD_FAC_NACHBAR): Wandmodell-Eingang u_t/y_w aus der zweiten Fluidzelle entlang der Normale (Slot 72 angewandt / 73 kein Fluidnachbar / 74 Nachbar still). CFD_FAC_UTKORR = "+to_string(s_fac_utkorr,3u)+" wirkt NUR an Zellen mit Eigenabtastung -- die 3/2-BB-Deflation gilt am Nachbarn nicht (03.09.).");
	if(s_fac_ema>0.0f&&s_fac_ema<5e-7f) print_error("CFD_FAC_EMA > 0 aber unter der Emissionsquantisierung (to_string 6 Stellen) -- der Filter froere still auf dem Warmstart ein.");
	if(s_fac_pema>0.0f&&s_fac_pema<5e-7f) print_error("CFD_FAC_PEMA > 0 aber unter der Emissionsquantisierung -- der Filter froere still ein.");
	if(s_fac_apg!=0.0f&&fabs(s_fac_apg)<5e-7f) print_error("CFD_FAC_APG zu klein fuer die 6-Stellen-Emission -- wuerde still zu 0.000000 (No-Op-Arm)."); // Gross-Audit N
	if(s_fac_apg!=0.0f) print_warning("CFD_FAC_APG liest rho der Nachbarzellen im SELBEN stream_collide-Launch -- Laeufe sind NICHT bitreproduzierbar (dieselbe Fehlerklasse wie der NACHBAR-Befund B72, 03.09.; vor einem APG-A/B nach dem fac_nb-Muster auslagern).");
	if(s_fac_pema>0.0f&&s_fac_nachbar>0u) print_error("PEMA + NACHBAR: die gefilterte Kette (kernel.cpp, utb_wm = utb*def_fac_utkorr) rechnet twe aus dem eigenen gefilterten u mit yw -- die Nachbarabtastung waere dort WIRKUNGSLOS, Slot 72 zaehlte trotzdem (Pruefagent 03.09.). Kombination gesperrt.");
	if(s_fac_apg!=0.0f&&s_fac_pema>0.0f) print_error("APG + PEMA: die gefilterte Kette verwirft die APG-Korrektur still -- Kombination gesperrt (Tiefen-Audit A1-B3: Sperre jetzt IM Konstruktor, setup-unabhaengig)."); 
	if(getenv("CFD_SPALDING_IT")&&env_u("CFD_SPALDING_IT",3u)==0u) print_warning("CFD_SPALDING_IT=0 wird auf 1 GEKLEMMT (min 1; Default ohne Env ist 3) -- Gross-Audit N16.");
	if(env_u("CFD_SPALDING_IT", 0u)>0u&&!s_wandfunktion&&!s_facetten) print_warning("CFD_SPALDING_IT wirkt nur mit CFD_WANDFUNKTION oder CFD_FACETTEN -- hier WIRKUNGSLOS (Audit R3).");
#ifndef TRT
	// ★ Audit-Nacharbeit 4: CFD_LAMBDA liegt in der TRT-Emission -- unter SRT (aktueller Build) ist
	// der Schalter TOT. Ein Lambda-A/B liefe bitgleich und ohne jede Meldung; deshalb die Ansage.
	if(getenv("CFD_LAMBDA")!=nullptr) print_warning("CFD_LAMBDA ist gesetzt, aber der Build laeuft mit SRT -- der Schalter ist WIRKUNGSLOS (TRT in defines.hpp aktivieren).");
#endif // TRT
#ifdef UPDATE_FIELDS
	// ★ Audit-Nacharbeit 8: im CFD_REG_BC-Arm liest deriv_reg u[] von Nachbarzellen, waehrend
	// stream_collide unter UPDATE_FIELDS u[] im selben Kernellauf SCHREIBT -- Wettlauf, jedes
	// REG-A/B waere nicht bitreproduzierbar. Default ist aus; wer ihn zieht, wird gewarnt.
#ifdef REGULARIZED_BOUNDARIES
	if(getenv("CFD_REG_BC")!=nullptr&&atoi(getenv("CFD_REG_BC"))>0) print_warning("CFD_REG_BC=1 unter UPDATE_FIELDS: deriv_reg liest u[] im Wettlauf mit dem Schreiber -- Ergebnisse sind NICHT bitreproduzierbar (Audit-Befund 8).");
#endif // REGULARIZED_BOUNDARIES
#endif // UPDATE_FIELDS
#ifndef REGULARIZED_BOUNDARIES
	// R2: die Race-Warnung oben haengt an UPDATE_FIELDS -- ohne einkompilierten REG-Arm waere sie
	// eine Warnung ueber Code, den es nicht gibt. Stattdessen die Wirkungslos-Ansage.
	if(getenv("CFD_REG_BC")!=nullptr&&atoi(getenv("CFD_REG_BC"))>0) print_warning("CFD_REG_BC ist gesetzt, aber REGULARIZED_BOUNDARIES ist nicht einkompiliert -- der Schalter ist WIRKUNGSLOS.");
#endif // REGULARIZED_BOUNDARIES
	// ★ Daempfungszone: Sperre und Wirksamkeitsmeldung.
	if(s_sponge_n>0u) {
		// def_Nx/def_Ny/def_Nz im Sponge-Block sind DOMAENENmasse inklusive Halo -- mit mehreren
		// Domaenen rampte die Zone an jeder inneren Domaenengrenze. Bis das auf globale Koordinaten
		// umgestellt ist, wird das hart verweigert statt lautlos falsch gerechnet.
		if(get_D()!=1u) print_error("Daempfungszone (CFD_SPONGE_N) ist nur fuer eine Domaene gebaut; mit D>1 rampte sie an jeder inneren Domaenengrenze.");
		// ★ Nachpruefer-Befund: fuer n gab es eine Schranke, fuer a und wmin keine. CFD_SPONGE_WMIN=2.5
		// erzwaenge ueber fmax ein w>2, also NEGATIVE effektive Viskositaet und sofortige Explosion;
		// ein negatives a machte nu_s<0. Beides jetzt hart abgewiesen statt lautlos gerechnet.
		if(!(s_sponge_wmin>0.0f && s_sponge_wmin<2.0f)) print_error("CFD_SPONGE_WMIN muss echt zwischen 0 und 2 liegen (w>=2 bedeutet negative Viskositaet).");
		if(s_sponge_a<1.0f) print_error("CFD_SPONGE_A unter 1 wuerde die Viskositaet in der Zone SENKEN statt anheben (negativ sogar nu<0).");
		// Was die Zone laminar bewirkt, VOR dem Lauf hinschreiben -- eine Klemme, die still zuschlaegt,
		// waere genau der lautlose No-op, den dieses Projekt sonst jagt.
		const float nu_rand = nu*s_sponge_a, tau_rand = 0.5f+3.0f*nu_rand, w_rand = 1.0f/tau_rand;
		print_info("Daempfungszone: "+to_string(s_sponge_n)+" Zellen, Faktor "+to_string(s_sponge_a,1u)+" am Rand (quadratische Rampe, Boden ausgenommen).");
		print_info("  laminar am Rand: nu "+to_string(nu,8u)+" -> "+to_string(nu_rand,6u)+", tau "+to_string(tau_rand,4u)+", w "+to_string(w_rand,4u)+" (Klemme bei w = "+to_string(s_sponge_wmin,3u)+")");
		if(w_rand<s_sponge_wmin) print_warning("Die Klemme greift schon LAMINAR (w = "+to_string(w_rand,4u)+" unter "+to_string(s_sponge_wmin,3u)+") -- die Zone ist schwaecher als eingestellt. Faktor senken oder CFD_SPONGE_WMIN bewusst absenken.");
	}
	sparse_on = s_sparse_tiles_on;
	sparse_T  = s_sparse_T;
	s_sparse_tiles_on = false; s_sparse_T = 8u; // read-once beidseitig (Gross-Audit N13: T erbte sonst)
	string opencl_c_code;
#ifdef GRAPHICS
	graphics = Graphics(this);
	opencl_c_code = device_defines(device_info)+graphics.device_defines(device_info)+get_opencl_c_code();
#else // GRAPHICS
	opencl_c_code = device_defines(device_info)+get_opencl_c_code();
#endif // GRAPHICS
	// ★ C2 (aus V1 portiert, 2026-08-15): CFD_DUMP_DEFINES druckt die tatsaechlich emittierte
	// Define-Liste (schliesst die Fehlerklasse "Host-Define != OpenCL-Define" -- ein #define in
	// defines.hpp wirkt NUR host-seitig, ein Kernel-#ifdef braucht die Emission ZUSAETZLICH).
	// CFD_DUMP_CL schreibt den kompletten OpenCL-Quelltext je Domaene -- DAS Werkzeug fuer den
	// Kontrollarm-Identitaetsnachweis (Diff zweier Dumps).
	if(env_on("CFD_DUMP_DEFINES")) {
		const string d = device_defines(device_info);
		uint n=0u; string out=""; size_t pos=0ull;
		while(true) {
			const size_t a=d.find("#define", pos); if(a==string::npos) break;
			const size_t b=d.find('\n', a);
			out += "   "+d.substr(a, (b==string::npos ? d.length() : b)-a)+"\n"; n++;
			if(b==string::npos) break; pos=b+1ull;
		}
		print_info("CFD_DUMP_DEFINES: "+to_string(n)+" Defines an OpenCL emittiert:");
		print(out);
	}
	if(env_on("CFD_DUMP_CL")) {
		static std::atomic<uint> dump_nr(0u); // je Domaene eine Datei, sonst ueberschreibt die zweite die erste
		const string pfad = "/tmp/fx3d_kernel_dump_"+to_string(dump_nr++)+".cl";
		std::ofstream f(pfad); f<<opencl_c_code; f.close();
		print_info("[CFD_DUMP_CL] OpenCL-Quelltext -> "+pfad+" ("+to_string((uint)opencl_c_code.size())+" Bytes)");
	}
	this->device = Device(device_info, opencl_c_code);
	print_info("Allocating memory. This may take a few seconds.");
	allocate(device); // lbm first
#ifdef GRAPHICS
	graphics.allocate(device); // graphics after lbm
#endif // GRAPHICS
}

// FORK -- F-Bounding-Box und Block-Tiling: Voreinstellungen. Beide default aus/voll = bit-identisch
// zu Upstream. (Die Ueberschrift nannte frueher nur das Block-Tiling, direkt ueber s_fbbox.)
uint LBM_Domain::s_fbbox[6] = {0u,0u,0u,0u,0u,0u};
void LBM_Domain::set_force_bbox(const uint x0, const uint y0, const uint z0, const uint nx, const uint ny, const uint nz) {
	s_fbbox[0]=x0; s_fbbox[1]=y0; s_fbbox[2]=z0; s_fbbox[3]=nx; s_fbbox[4]=ny; s_fbbox[5]=nz;
}
uint LBM_Domain::s_sponge_n = 0u;
float LBM_Domain::s_sponge_a = 3000.0f;
float LBM_Domain::s_wf_tau = 1.0f;
bool LBM_Domain::s_wandfunktion = false;
bool LBM_Domain::s_facetten = false;
bool LBM_Domain::s_fac_imem = false;
float LBM_Domain::s_fac_ema = 0.0f;
float LBM_Domain::s_fac_pema = 0.0f;
bool LBM_Domain::s_fac_satgate = false;
uint LBM_Domain::s_boden_eq_n = 0u;
uint LBM_Domain::s_boden_eq_down = 0u;
uint LBM_Domain::s_boden_eq_split = 0xFFFFFFFFu;
float LBM_Domain::s_boden_eq_u = 0.075f;
uint LBM_Domain::s_boden_eq_abstand = 0u; // Heiko 2026-08-20: reifennahe Aussparung (Chebyshev-Abstand zu TYPE_S, Boden ausgenommen) // Setup kann eigenes u_lat durchreichen (XL-Audit B6)
uint LBM_Domain::s_einlass_eq_n = 0u; // ★ EINLASS_EQ (V1-Port apply_inlet_velocity): Spaltenzahl x=1..N hinter dem Einlass; 0 = aus
float LBM_Domain::s_einlass_eq_u = 0.075f; // Setup reicht sein u_lat durch (Konvention wie s_boden_eq_u)
bool LBM_Domain::s_schale_paritaet = false; // CFD_N2F_PARITAET (Beweisarm, s. lbm.hpp)
float LBM_Domain::s_schale_alpha = 0.0f; // ★ P9c N2F-SCHALE: Blendfaktor der near->far-Rueckkopplung; 0 = aus. Read-once wie EINLASS_EQ; Setup setzt lbm_f EXPLIZIT 0.
uint LBM_Domain::s_fac_alpha = 0u;
bool LBM_Domain::s_fac_elibb = false;
uint LBM_Domain::s_sgs_fdwand = 0u; // ★ 02.09. Geistermoden-Fix
uint LBM_Domain::s_sgs_gdiag = 0u; // ★ 31.08. g-Diagnose (CFD_SGS_GDIAG)
uint LBM_Domain::s_fac_messnur = 0u; // ★ 30.08. Mess-Nur-Modus (BB-Physik, Facetten-Instrument)
uint LBM_Domain::s_fac_nachbar = 0u; // ★ 30.08. Nachbarabtastung des Wandmodell-Eingangs
uint LBM_Domain::s_fac_kdiag = 0u; // ★ 30.08. Klassen-Diagnostik (CFD_FAC_KDIAG)
bool LBM_Domain::s_fac_elibb_pur = false; // Pur-Arm (Isolationsmessung) // ★ B1/B2 (2026-08-25): ELIBB 18-Link, q aus der Facettenebene
float LBM_Domain::s_fac_qmin = 0.1f;
float LBM_Domain::s_fac_kappa = 0.4f;
float LBM_Domain::s_fac_utkorr = 1.0f; // 3/2-Abtastpunkt-Messarm
float LBM_Domain::s_fac_qkappe = 1.0f; // Ex-Stabilitaetskappe des q>0,5-Zweigs: mit der MLS-Blende (Baustein 1, 26.08.) obsolet -- Default 1,0 = keine Kappung; Env-Hebel CFD_FAC_QKAPPE bleibt fuer A/Bs
uint LBM_Domain::s_fac_qdiag = 0u; // ★ QDIAG-Diagnosearme (Injektionsjagd 2026-08-25)  // q-Boden (P1-Entscheid): darunter HWBB, mit Zaehler
uint LBM_Domain::s_fac_kraft = 0u; // ★ 30.08. Zellkraft statt Slip (CFD_FAC_KRAFT)
bool LBM_Domain::s_fac_quergate = false; // ★ 2026-08-25 CFD_FAC_QUERGATE: BB belassen, wenn der Querrest die Wandschubspannung uebersteigt
bool LBM_Domain::s_fac_lsq = false; // ★ 2026-08-25 Default AUS nach Pruefbefund 4-A/4-B: das ist eine
// MODELLAENDERUNG, kein Numerikfix. LSQ gewichtet t1 (Stroemungsrichtung, Ziel = Spalding-tau_w, die
// eigentliche Messgroesse) und t2 (Ziel 0, eine blosse Modellannahme) GLEICH -- fuer ein Wandmodell die
// falsche Gewichtung. Ausserdem bricht sie die SATGATE-Invariante "iMEM wirkt nur, wenn es sein Ziel im
// Budget EXAKT erreichen kann": LSQ erreicht es prinzipiell nie exakt, das Gate laesst sie trotzdem
// durch. Braucht einen eigenen Messarm mit eigener Begruendung, nicht den Rang eines Defaults.
float LBM_Domain::s_fac_apg = 0.0f;
long LBM_Domain::s_fac_diagz = -1l;
float LBM_Domain::s_fac_tau = 1.0f;
float LBM_Domain::s_fac_budget = 1.0f;    // CFD_FAC_BUDGET (1a-B4t), Default bitidentisch
float LBM_Domain::s_fac_budget_sn = 1.0f; // CFD_FAC_BUDGET_SN (1a-Bsn), Default bitidentisch
bool LBM_Domain::s_sgs_wandfrei = false;
bool LBM_Domain::s_sgs_guo = true; // ★ 2026-08-25 Default AN: das ist die richtige Physik, CFD_SGS_GUO=0 ist der Kontrollarm
bool LBM_Domain::s_sgs_diag = false;
ulong LBM_Domain::s_sgs_diag_ab = 0ull;
float LBM_Domain::s_sponge_wmin = 0.5f;
bool LBM_Domain::s_sparse_tiles_on = false;
uint LBM_Domain::s_sparse_T = 8u;

void LBM_Domain::allocate(Device& device) {
	const ulong N = get_N();
	// Bei aktivem Sparse zunaechst nur ein 1-Zell-Platzhalter: finalize_sparse_tiles() legt die echte
	// sparse fi an. Grund ist kein Geschmack, sondern ein Treiberdefekt -- das Freigeben eines bereits
	// allozierten 19-GB-fi-Buffers bringt den Intel-NEO mit CL_OUT_OF_RESOURCES zu Fall. Das Move-Assign
	// in finalize gibt so nur den Platzhalter frei, was trivial ist.
	fi = Memory<fpxx>(device, sparse_on ? 1ull : N, velocity_set, false);
	rho = Memory<float>(device, N, 1u, true, true, 1.0f);
	u = Memory<float>(device, N, 3u);
	flags = Memory<uchar>(device, N);
	if(sparse_on) { // Tile-Raster aufspannen; der Inhalt kommt erst in finalize_sparse_tiles()
		sparse_tiles_x = ((uint)get_Nx()+sparse_T-1u)/sparse_T;
		sparse_tiles_y = ((uint)get_Ny()+sparse_T-1u)/sparse_T;
		sparse_tiles_z = ((uint)get_Nz()+sparse_T-1u)/sparse_T;
		tile_slot = Memory<uint>(device, (ulong)sparse_tiles_x*sparse_tiles_y*sparse_tiles_z);
	} else {
		tile_slot = Memory<uint>(device, 1ull); // Platzhalter, wird nie gelesen (TS_A ist leer)
	}
	kernel_initialize = Kernel(device, N, "initialize", fi, rho, u, flags);
	// 23 Slots (Legende R3 nachgezogen, massgeblich ist lbm.hpp; [22] N2F-SCHALE-Blend-Wirkpfad t%100 (P9c); [21] EINLASS_EQ-Wirkpfad t%100; [20] BODEN_EQ-Wirkpfad t%100): [0,1] RHO_CLAMP, [2] WFB-Wirkpfad
	// (t%100), [3] tau-Klemme, [4] u_t~0-Skips, [5] Ein-Zellen-Spalt, [6] SGS-Wirkpfad (t%100),
	// [7] Facetten-Wirkpfad (t%100), [8] Facetten-Klemmen (BEIDE, gegatet t%100 seit R3),
	// [9] Facetten-Skips (gegatet t%100 seit R3), [10] reserviert, [11] ohne offenes Paar (t%100).
	// Achtung uint: 3/4/5 zaehlen jeden Schritt
	// und koennten bei ~1e9+ Ereignissen ueberlaufen -- Ist!=Soll faellt im Report auf, aber wer
	// Slots erweitert, gate sie. Vergroesserung statt neuem Puffer: haengt schon an stream_collide,
	// keine Signaturaenderung, Kontrollarm bleibt bitgleich (neue Slots nur unter #ifdef-Emission).
	rho_clamp_hits = Memory<uint>(device, 80ull); // 72->80 am 02.09.: Slots 20-71 sind luecklos belegt (30-48 SGS_DIAG-Bins ueber BERECHNETE Indizes 30u+b/35u+bw/40u+bw/45u+..., die ein Literal-Grep nicht sieht -- zweimal bezahlte Lektion B-3/B70); neue Zaehler ab 72 // [70] KRAFTPFAD (CFD_FAC_KRAFT, saettigend; Soll Modus 1: == [69]) | [71] Kraftzellen im Anlauf t<100, UNGEGATET (saettigend) | [72..74] NACHBAR angewandt/kein-Fluid/still | [75] MESSNUR-Wirkpfad | [76] FDWAND angewandt | [77..79] frei // // [67] ELIBB-Wirkpfad beide Zweige (saettigend) | [68] MLS-q>0,5-Zweig allein (saettigend, Audit 26.08.) | [69] Rueckfall-Buchung P-only (saettigend, Buchungsschluss 27.08.; Soll = 13+15+64 +10+16 unter SATGATE) // ★ LEGENDE, Stand 2026-08-27 (Pruefbefund 3-E: die alte war in sich widerspruechlich)
	// [0..1] RHO_CLAMP unten/oben (t%100) | [2..5] Wandfunktion | [6] SGS_WANDFREI | [7..19] Facetten/iMEM
	// [20] BODEN_EQ | [21] EINLASS_EQ | [22] N2F-SCHALE | [23..24] N2F-Paritaet | [25..26] Paarungsbeweis
	// [27] Slot-13-Split | [28] Geschwindigkeitsklemme | [29] SPONGE | [30..34] nu_t/nu_0 Dekaden
	// [35..39] nu_t/nu_0 wandnaechste Lage | [40..44] davon anliegend | [45..48] oberer Schwanz
	// [49..53] Stoerform-Offset |2(S1.t)|/Ziel | [54..58] |P|/Ziel | [59] Bewegtwand-Term
	// [60..63] Guo-Korrektur, rel. Aenderung von |Pi^neq| | [64] Quergate (CFD_FAC_QUERGATE) | [65] LSQ-Rueckfall
	// ALLE Ereignis-Slots sind t%100-Stichproben; 49..58 und 60..63 zusaetzlich hash-ausgeduennt (jede 64.).
	kernel_stream_collide = Kernel(device, N, "stream_collide", fi, rho, u, flags, t, fx, fy, fz, rho_clamp_hits);
	kernel_update_fields = Kernel(device, N, "update_fields", fi, rho, u, flags, t, fx, fy, fz);
	kernel_boden_eq = Kernel(device, N, "boden_eq", fi, flags, t, 0.0f, 0u, 0u, 0u, 0u, rho_clamp_hits); // Parameter t/u/nz/nz_down/x_split/abstand je Enqueue
	boden_eq_n = s_boden_eq_n; boden_eq_u = s_boden_eq_u; boden_eq_down = s_boden_eq_down; boden_eq_split = s_boden_eq_split; boden_eq_abstand = s_boden_eq_abstand; // u_road = u_lat-Projektkonvention; Konstruktionszeit-Kopie (read-once-Doktrin)
	kernel_einlass_eq = Kernel(device, N, "einlass_eq", fi, flags, t, 0.0f, 0u, rho_clamp_hits); // ★ EINLASS_EQ (V1-Port apply_inlet_velocity): Parameter t/u/nx je Enqueue
	einlass_eq_n = s_einlass_eq_n; einlass_eq_u = s_einlass_eq_u; // Konstruktionszeit-Kopie (read-once-Doktrin)
	schale_paritaet = s_schale_paritaet; // Beweisarm: Kernel-alpha 0, Enqueue laeuft (read-once)
	schale_alpha = s_schale_alpha; // ★ P9c N2F-SCHALE: Konstruktionszeit-Kopie (read-once-Doktrin); die Kernel entstehen erst in alloc_schale (Indexlisten-Groesse steht erst nach dem Listenbau fest)

#ifdef FORCE_FIELD
	// FORK -- F-BBox: die Box wurde bereits im Konstruktor aufgeloest (sie muss vor device_defines()
	// feststehen). Hier wird sie nur noch benutzt.
	const ulong F_N = (ulong)fbnx*(ulong)fbny*(ulong)fbnz;
	if(F_N<(ulong)get_N()) print_info("F-BBox: F auf "+to_string(fbnx)+"x"+to_string(fbny)+"x"+to_string(fbnz)
		+" statt "+to_string((ulong)get_N())+" Zellen -> "+to_string((float)(((ulong)get_N()-F_N)*12ull)/1e9f,2u)+" GB gespart");
	// ★ FORK 03.09.2026, VERSUCHT UND ZURUECKGENOMMEN: F ohne Host-Spiegel anzulegen (Memory<float>(device,
	// F_N, 3u, false), fi-Bauform) spart am 4-mm-Nahfeld 1.832 MiB SYSTEM-RAM -- aber nur System-RAM, kein
	// VRAM. Der Versuch endete im Segfault (rc=139, Rauchtest xz_sparsam_cpu 03.09.), und zwar an zwei
	// Stellen, die BEIDE den Host-Spiegel brauchen: LBM::initialize() laedt F per enqueue_write_to_device
	// hoch (lbm.cpp, weiter unten), und der F-Waechter davor prueft die F-NUR-SOLID-Praemisse auf dem Host.
	// Sauber waere: bei !F_host stattdessen kernel_reset_force_field starten UND den Waechter mit Ansage
	// ueberspringen. Das sind zwei weitere Eingriffe fuer einen Posten, der auf der B70 kein einziges MB
	// VRAM bringt -- und der Waechter, den es kostet, ist ein Sicherheitsnetz. Zurueckgestellt.
	F = Memory<float>(device, F_N, 3u);
	object_sum = Memory<float>(device, 1u, 4u); // x, y, z, cell count
	kernel_stream_collide.add_parameters(F);
	kernel_update_fields.add_parameters(F);
	kernel_update_force_field = Kernel(device, N, "update_force_field", fi, flags, t, F, u, rho_clamp_hits); // ★ 2026-08-25 u + hits fuer den Bewegtwand-Term (Slot 59)
	kernel_reset_force_field = Kernel(device, N, "reset_force_field", F);
	kernel_object_center_of_mass = Kernel(device, N, "object_center_of_mass", flags, (uchar)0u, object_sum);
	of_groups = 1024u; // ★ 2026-08-25 feste Gittergroesse statt N -- Determinismus, s. Kernel-Kommentar
	of_part = Memory<float>(device, 3ull*(ulong)of_groups);
	kernel_object_force = Kernel(device, (ulong)of_groups*(ulong)WORKGROUP_SIZE, "object_force", F, flags, (uchar)0u, of_part);
	kernel_object_force_zband = Kernel(device, (ulong)of_groups*(ulong)WORKGROUP_SIZE, "object_force_zband", F, flags, (uchar)0u, 0u, 0u, of_part); // FORK Kraft-Zerlegung: Aufrufe sequenziell
	kernel_object_force_final = Kernel(device, (ulong)WORKGROUP_SIZE, "object_force_final", of_part, of_groups, object_sum);
	kernel_object_torque = Kernel(device, N, "object_torque", F, flags, (uchar)0u, 0.0f, 0.0f, 0.0f, object_sum);
#endif // FORCE_FIELD

#ifdef MOVING_BOUNDARIES
	kernel_update_moving_boundaries = Kernel(device, N, "update_moving_boundaries", u, flags);
#endif // MOVING_BOUNDARIES

#ifdef SURFACE
	phi = Memory<float>(device, N);
	mass = Memory<float>(device, N, 1u, false);
	massex = Memory<float>(device, N, 1u, false);
	kernel_initialize.add_parameters(mass, massex, phi);
	kernel_stream_collide.add_parameters(mass);
	kernel_surface_0 = Kernel(device, N, "surface_0", fi, rho, u, flags, mass, massex, phi, t, fx, fy, fz);
	kernel_surface_1 = Kernel(device, N, "surface_1", flags);
	kernel_surface_2 = Kernel(device, N, "surface_2", fi, rho, u, flags, t);
	kernel_surface_3 = Kernel(device, N, "surface_3", rho, flags, mass, massex, phi);
#endif // SURFACE

#ifdef TEMPERATURE
	gi = Memory<fpxx>(device, N, 7u, false);
	T = Memory<float>(device, N, 1u, true, true, 1.0f);
	kernel_initialize.add_parameters(gi, T);
	kernel_stream_collide.add_parameters(gi, T);
	kernel_update_fields.add_parameters(gi, T);
#endif // TEMPERATURE

#ifdef PARTICLES
	particles = Memory<float>(device, (ulong)particles_N, 3u);
	kernel_integrate_particles = Kernel(device, (ulong)particles_N, "integrate_particles", particles, u, flags, 1.0f);
#ifdef FORCE_FIELD
	kernel_integrate_particles.add_parameters(F, fx, fy, fz);
#endif // FORCE_FIELD
#endif // PARTICLES

	// ★ C1b Stufe 2 (F2): Facetten-Puffer als 1-Element-Platzhalter binden -- die echten Groessen
	// stehen erst nach Voxelisierung + baue_facetten() fest; bind_facetten() ersetzt sie per
	// set_parameters an fac_param_pos (Muster finalize_sparse_tiles). MUSS vor dem TS_P-Block stehen.
	if(facetten_on) {
		fac_ema_on = s_fac_imem&&s_fac_ema>0.0f;
		fac_geo   = Memory<float>(device, 8ull);
		fac_idx   = Memory<uint>(device, 1ull);
		fac_tau   = Memory<float>(device, 1ull);
		fac_tau_n = Memory<uint>(device, 1ull);
		fac_param_pos = kernel_stream_collide.get_number_of_parameters();
		kernel_stream_collide.add_parameters(fac_geo, fac_idx, fac_tau, fac_tau_n);
		if(fac_ema_on) { fac_us = Memory<float>(device, 3ull); kernel_stream_collide.add_parameters(fac_us); } // Signatur-Paritaet mit #ifdef FACETTEN_EMA
		fac_pema_on = s_fac_imem&&s_fac_pema>0.0f;
		if(fac_pema_on) { fac_pu = Memory<float>(device, 6ull); kernel_stream_collide.add_parameters(fac_pu); }
		fac_diagz_on = s_fac_imem&&s_fac_diagz>=0l; fac_diagz_wert = s_fac_diagz; // Gross-Audit: Konstruktionswert einfrieren -- alloc_facetten liest sonst die Statik der falschen Instanz
		if(fac_diagz_on) { fac_diag = Memory<float>(device, 19ull); kernel_stream_collide.add_parameters(fac_diag); } // 19: [17] alpha, [18] dp_ds
		fac_elibb_on = s_fac_imem&&s_fac_elibb; // ★ B2: Konstruktionszustand einfrieren (dieselbe Lektion wie diagz)
		if(fac_elibb_on) { fac_q = Memory<uchar>(device, 18ull); kernel_stream_collide.add_parameters(fac_q); } // Platzhalter; alloc_facetten_domain baut und rebindet
		fac_kdiag_on = s_fac_imem&&s_fac_kdiag>0u; // ★ Klassen-Diagnostik: Konstruktionszustand einfrieren (Signaturposition = nach fac_q)
		if(fac_kdiag_on) { fac_kd = Memory<float>(device, 10ull); kernel_stream_collide.add_parameters(fac_kd); }
		nachbar_on = s_fac_imem&&s_fac_nachbar>0u; // ★ 03.09. deterministische Nachbarabtastung: Konstruktionszustand einfrieren (Emission haengt an derselben Statik; Signaturposition = nach fac_kd, VOR fac_wfd)
		if(nachbar_on) { fac_nb = Memory<float>(device, 2ull); kernel_stream_collide.add_parameters(fac_nb); } // Platzhalter; alloc_facetten_domain baut und rebindet
		fdwand_on = s_sgs_fdwand>0u; // ★ Geistermoden-Fix: Konstruktionszustand einfrieren (Emission haengt an derselben Statik; Signaturposition = nach fac_kd)
		if(fdwand_on) { fac_wfd = Memory<float>(device, 1ull); kernel_stream_collide.add_parameters(fac_wfd); } // Platzhalter; alloc_facetten_domain baut und rebindet -- der KOHAERENZ-WAECHTER dort verhindert, dass der Platzhalter je gelesen wird
	}

	// FORK -- Block-Tiling: tile_slot ist per TS_P der LETZTE Parameter jedes fi-Kernels, muss also NACH
	// allen anderen add_parameters angehaengt werden. Bei ausgeschaltetem Sparse ist TS_P leer, dann darf
	// hier auch nichts gebunden werden -- sonst stimmt die Parameterzahl nicht mehr mit der Device-Seite
	// ueberein, und das ist genau die Fehlerklasse, die still falsche Ergebnisse produziert.
	if(sparse_on) {
		// Multi-GPU + Sparse ist nicht validiert: die transfer_*_fi-Kernel bekaemen tile_slot hier nicht
		// gebunden. Lieber laut abbrechen als still falsch rechnen.
		if(get_D()>1u) print_error("Block-Tiling (CFD_SPARSE_TILES) ist nur fuer eine einzelne GPU validiert, hier laufen "+to_string(get_D())+" Domaenen.");
		kernel_initialize.add_parameters(tile_slot);
		kernel_stream_collide.add_parameters(tile_slot);
		kernel_update_fields.add_parameters(tile_slot);
		kernel_boden_eq.add_parameters(tile_slot); // XL-Audit B1 (Pruefagent R2: NICHT unter FORCE_FIELD -- TS_P haengt nur an SPARSE_TILES)
		kernel_einlass_eq.add_parameters(tile_slot); // EINLASS_EQ: dieselbe Lektion (TS_P haengt NUR an SPARSE_TILES)
#ifdef FORCE_FIELD
		kernel_update_force_field.add_parameters(tile_slot);
#endif // FORCE_FIELD
	}

	if(get_D()>1u&&(fbnx!=Nx||fbny!=Ny||fbnz!=Nz)) print_error("F-BBox + Multi-GPU ist NICHT gebaut (transfer_F/graphics indizieren F voll-domaenig -- OOB)."); // ★ Tiefen-Audit B1: vorher pruefte er die read-once-GENULLTE Statik = toter Code; jetzt die aufgeloesten Member
	if(get_D()>1u) allocate_transfer(device);
}

void LBM_Domain::enqueue_apply_pressure_outlet() { // FORK: Druck-Auslass
	if(po_N_active==0u) return; // nicht konfiguriert -> nichts zu tun (kein Leerlauf-Dispatch)
	// Reihenfolge ist tragend: Teilsummen bilden, dann in Indexordnung zusammenfassen, dann anwenden.
	// Alle drei auf derselben in-order-Warteschlange, also ohne zusaetzliche Barriere. po_clear_mean
	// entfiel mit dem Umbau -- po_final_mean schreibt mit "=", und jeder Teilsummen-Slot wird jeden
	// Schritt ueberschrieben (2026-08-24).
	kernel_po_reduce_mean.enqueue_run();
	kernel_po_final_mean.enqueue_run();
	kernel_apply_pressure_outlet.enqueue_run();
}

void LBM_Domain::alloc_coupling_planes(const ulong max_plane_cells) { // FORK: Doppel-Domaene
	if(max_plane_cells==0ull) { print_error("alloc_coupling_planes mit 0 Zellen."); return; }
	coupling_max_plane_cells = max_plane_cells;
	coupling_plane = Memory<float>(device, max_plane_cells*4ull, 1u);
	// Ebenen-Parameter werden bei jedem Aufruf neu gesetzt; hier stehen Platzhalter, damit die
	// Kernel-Objekte ueberhaupt mit der richtigen Signatur entstehen.
	kernel_extract_plane_macros = Kernel(device, max_plane_cells, "extract_plane_macros",
		rho, u, coupling_plane, 0u, 0u, 0u, 0u, 1u, 1u);
	kernel_drive_boundary_cubic_lift = Kernel(device, max_plane_cells, "drive_boundary_cubic_lift",
		rho, u, flags, coupling_plane, 0u, 0u, 0u, 0u, 1u, 1u, 1u, 1u, 4u);
	// ★ Slice-Ebenen-Read 2026-08-26 (Hausmuster: Puffer anlegen und Kernel MIT echten Puffern
	// erzeugen -- kein Platzhalter-Bind-spaeter, die DIAGZ-Use-after-free-Klasse).
	slice_flags = Memory<uchar>(device, max_plane_cells, 1u);
	kernel_extract_plane_flags = Kernel(device, max_plane_cells, "extract_plane_flags",
		flags, slice_flags, 0u, 0u, 0u, 0u, 1u, 1u);
	print_info("Kopplungspuffer: "+to_string(max_plane_cells)+" Zellen a 4 floats = "
		+to_string((float)(max_plane_cells*16ull)/1048576.0f,2u)+" MB auf "+device.info.name+".");
}

// ★ P9c N2F-SCHALE (Muster alloc_coupling_planes): Puffer anlegen und die Kernel MIT ECHTEN
// Puffern erzeugen -- kein Platzhalter-Bind-später (die DIAGZ-Use-after-free-Klasse). MUSS nach
// finalize_sparse_tiles laufen (fi ist dann final gebunden; im dd-Fall hat das Grobgitter ohnehin
// kein Tiling, und das Setup ruft alloc erst nach run(0)).
void LBM_Domain::alloc_schale(const std::vector<ulong>& liste, const std::vector<float>& gewichte, const uint ratio, const uint modus) {
	const ulong n = (ulong)liste.size();
	if(n==0ull) { print_error("alloc_schale mit leerer Liste."); return; }
	if(n>0x55555555ull) { print_error("alloc_schale: Liste ueberschreitet 2^32/3 Zellen -- die 3u*gid-Indexprodukte in schale_extract/schale_blend (kernel.cpp) wickeln in 32 Bit VOR der 2^32-Grenze (Kernel-Pruefer 2026-08-22 abends). Praktisch unerreichbar, aber der Guard deckt jetzt seine eigene Arithmetik."); return; }
	if(ratio==0u) { print_error("alloc_schale: ratio=0 (Blockmittel-Fenster waere leer)."); return; }
	if((ulong)gewichte.size()!=n) { print_error("alloc_schale: gewichte ("+to_string((ulong)gewichte.size())+") passt nicht zur Liste ("+to_string(n)+") -- der Kernel laese daneben."); return; }
	for(ulong i=0ull; i<n; i++) if(!(gewichte[i]>=0.0f&&gewichte[i]<=1.0f)) { print_error("alloc_schale: gewicht["+to_string(i)+"] = "+to_string(gewichte[i],6u)+" liegt nicht in [0;1] (NaN faengt die Negativform mit)."); return; }
	if(modus>2u) { print_error("alloc_schale: modus = "+to_string(modus)+" (gueltig: 0 EQ, 1 FNEQ, 2 IDENT-Debug)."); return; }
	schale_n = (uint)n;
	schale_modus = modus;
	schale_liste = Memory<ulong>(device, n);
	for(ulong i=0ull; i<n; i++) schale_liste[i] = liste[i];
	schale_liste.write_to_device();
	schale_unear = Memory<float>(device, 3ull*n); // Blend-Eingang (Host-Upload); Ctor-Nullinit -> vor dem ersten Upload waere unear 0, deshalb macht das Setup einen 1-Outer-Vorlauf wie bei der Hinkopplung
	schale_uout  = Memory<float>(device, 3ull*n); // Extract-Ausgang (getrennt, damit der Waechter-Extract unear nicht ueberschreibt)
	schale_gewicht = Memory<float>(device, n); // Gradient-Blend: Zellgewichte (Lagen-Rampe), wirken als a = alpha*gewicht[gid]
	for(ulong i=0ull; i<n; i++) schale_gewicht[i] = gewichte[i];
	schale_gewicht.write_to_device();
	kernel_schale_extract = Kernel(device, n, "schale_extract", u, flags, schale_liste, (uint)n, ratio, 1u, schale_uout); // mittel (Pos. 5) je Enqueue
	kernel_schale_blend   = Kernel(device, n, "schale_blend", fi, flags, t, 0.0f, schale_liste, (uint)n, schale_unear, schale_gewicht, modus, rho_clamp_hits); // t/alpha (Pos. 2/3) je Enqueue; gewicht+modus VOR diag (Plan-Vorgabe)
	if(sparse_on) kernel_schale_blend.add_parameters(tile_slot); // TS_P haengt NUR an SPARSE_TILES (XL-Audit-B1-Lektion); der Blend laeuft zwar nur im Fernfeld (ohne Tiling), aber die Signatur muss zur Emission der Domaene passen
	print_info("N2F-Schale: "+to_string(n)+" Zellen a 2x3+1 floats + Indexliste = "
		+to_string((float)(n*36ull)/1048576.0f,2u)+" MB auf "+device.info.name+" (alpha dieser Domaene: "+to_string(schale_alpha,3u)+", modus "+to_string(modus)+(modus==2u?" IDENT-Debug":modus==1u?" FNEQ":" EQ")+").");
}

void LBM_Domain::enqueue_schale_blend() { // ★ P9c: post-stream Schalen-Blend (nach einlass_eq)
	// No-Op-Doppelgate: schale_n==0 = nie alloziert; schale_alpha==0 = alloziert, aber nur als
	// Extract-Seite (lbm_f traegt eine Deckungspunkt-Liste, darf aber NIE blenden -- Slot-22-Soll nah==0).
	if(schale_n==0u||schale_alpha==0.0f) return;
	kernel_schale_blend.set_parameters(2u, t, schale_paritaet ? 0.0f : schale_alpha).enqueue_run(); // Paritaetsarm: alpha exakt 0, aber der Kernel LAEUFT (sonst waere der Beweis ein No-Op)
}

// ★ C1b Stufe 2: Facettendaten der Domaene bauen, hochladen, Kernel neu binden (FACETTEN-STUFE2.md F1/F2).
// Filtert klasse!=0 (markierte Zellen behalten reinen BB); fac_a = 1/|n_achse| host-berechnet (R2).
// ★ 29.08. (Heiko: "so dass wir wirklich wissen, wieviel Reserve wir immer noch haben und was
// wir wirklich nutzen"). device_info.memory ist eine REKONSTRUKTION: opencl.hpp:170 rechnet NEOs
// 95-%-Deckel mit 20/19 heraus, der Treiber meldet weniger. Und der Desktop haengt an derselben
// Karte, ohne dass memory_used davon etwas sieht. Der einzige belastbare Wert steht im
// DRM-Debugfs. NICHT ueber /proc/*/fdinfo -- der unterzaehlt grob (23.08.: Faktor sieben).
// Liefert freie MiB, oder 0 wenn nicht lesbar (Debugfs braucht Rechte -- dann bleibt es bei
// der Rekonstruktion, und der Aufrufer sagt das auch so).
ulong vram_frei_gemessen() {
	for(const string& pfad : {string("/sys/kernel/debug/dri/0/tile0/vram_mm"), string("/sys/kernel/debug/dri/0/i915_gem_objects")}) {
		std::ifstream f(pfad);
		if(!f) continue;
		string z;
		while(std::getline(f, z)) { // Zeile der Form "free: 12345678 KiB" bzw. "...: N B"
			const size_t p_ = z.find("free");
			if(p_==string::npos) continue;
			ulong wert=0ull; bool ziffer=false;
			for(size_t i=p_; i<z.size(); i++) {
				if(z[i]>='0'&&z[i]<='9') { wert = wert*10ull + (ulong)(z[i]-'0'); ziffer=true; }
				else if(ziffer) break;
			}
			if(!ziffer) continue;
			if(z.find("KiB")!=string::npos||z.find("kB")!=string::npos) return wert/1024ull;
			if(z.find("MiB")!=string::npos||z.find("MB")!=string::npos) return wert;
			return wert/1048576ull; // Bytes
		}
	}
	return 0ull;
}

void LBM_Domain::alloc_facetten_domain(const std::vector<Facette>& F, const uint Nx, const uint Ny, const std::unordered_map<ulong,std::array<uchar,18>>* qmap, const uint sgs_gdiag, const uint sgs_fdwand) {
	if((sgs_fdwand>0u)!=fdwand_on) print_error("SGS_FDWAND-Konfigurationsbruch: env-Parameter ("+to_string((ulong)sgs_fdwand)+") und Konstruktionszustand ("+string(fdwand_on?"an":"aus")+") widersprechen sich -- Emission haengt am Konstruktionszustand, Puffer am Parameter; beide muessen aus DEMSELBEN CFD_SGS_FDWAND stammen (Statik-Lebensdauer-Lehre 02.09.).");
	if(!facetten_on) { print_error("alloc_facetten_domain ohne CFD_FACETTEN."); return; }
	const ulong FN = (ulong)fbnx*(ulong)fbny*(ulong)fbnz;
	if(FN==0ull) { print_error("alloc_facetten_domain: F-BBox ist leer."); return; }
	ulong aktiv=0ull, ausgeschlossen=0ull;
	for(const Facette& f : F) { if(f.klasse==0u) aktiv++; else ausgeschlossen++; }
	if(aktiv==0ull) { print_error("alloc_facetten_domain: keine aktive Facette (alle markiert?)."); return; }
	if(aktiv>=0xFFFFFFFFull) { print_error("alloc_facetten_domain: Facettenzahl kollidiert mit dem NIL-Sentinel."); return; }
	// ★ ZWEITE STUFE der Speicherpruefung (29.08.). Die Konstruktor-Vorpruefung kennt die
	// Facettenzahl noch nicht -- hier steht sie. Das ist der letzte grosse Posten, und ohne
	// diese Stufe faellt ein zu grosses Gitter erst nach zehn Minuten Aufbau auf.
	{	const ulong bytes_fac = 4ull*FN                 // fac_idx
		                      + (8ull+6ull)*4ull*aktiv  // fac_geo + fac_tau
		                      + 4ull*aktiv              // fac_tau_n
		                      + (fac_elibb_on ? 18ull*aktiv : 0ull)  // fac_q
		                      + (fac_kdiag_on ? 40ull*aktiv : 0ull)  // fac_kd (Klassen-Diagnostik, 10 float)
		                      + (nachbar_on ? 8ull*aktiv : 0ull)     // fac_nb (deterministische Nachbarabtastung, 2 float)
		                      + (sgs_gdiag>0u ? 40ull*aktiv : 0ull)  // gd_zellen (8 B) + fac_gd (32 B) der g-Diagnose
		                      + (sgs_fdwand>0u ? (sgs_gdiag>0u?4ull:12ull)*aktiv : 0ull); // fac_wfd (4 B) + gd_zellen (8 B), falls nicht schon von gdiag gebaut
		const ulong mb_fac = bytes_fac/1048576ull;
		const ulong frei_gemessen = device.info.uses_ram ? 0ull : vram_frei_gemessen();
		const ulong belegt = (ulong)device.info.memory_used;
		const ulong kapazitaet = (ulong)device.info.memory;
		print_info("SPEICHER-IST vor den Facettenpuffern: belegt "+to_string(belegt)+" MB von "
			+to_string(kapazitaet)+" MB (rekonstruiert)"
			+(frei_gemessen>0ull ? string(", GEMESSEN frei "+to_string(frei_gemessen)+" MB (DRM-Debugfs)")
			                     : string(", gemessener Frei-Wert NICHT lesbar -- Debugfs braucht Rechte"))
			+" | Facettenpuffer "+to_string(mb_fac)+" MB fuer "+to_string(aktiv)+" aktive Facetten");
		// Gegen den GEMESSENEN Wert pruefen, wenn er da ist -- sonst gegen die Rekonstruktion.
		const ulong frei = frei_gemessen>0ull ? frei_gemessen : (kapazitaet>belegt ? kapazitaet-belegt : 0ull);
		const ulong mindest = (ulong)env_u("CFD_VRAM_MIN_FREI_MB", 1024u); // Heiko 29.08.: 1,0-1,5 GB Restluft sind legitim
		if(!device.info.uses_ram && mb_fac+mindest > frei)
			print_error("Facettenpuffer passen nicht: "+to_string(mb_fac)+" MB noetig, "+to_string(frei)
				+" MB frei, Mindestluft "+to_string(mindest)+" MB (CFD_VRAM_MIN_FREI_MB). "
				+to_string(aktiv)+" aktive Facetten bei F-BBox "+to_string(fbnx)+"x"+to_string(fbny)+"x"+to_string(fbnz)
				+". Gitter verkleinern, Facettenzahl senken oder die Mindestluft bewusst herabsetzen.");
	}
	fac_geo   = Memory<float>(device, 8ull*aktiv);
	fac_idx   = Memory<uint>(device, FN);
	fac_tau   = Memory<float>(device, 6ull*aktiv); // Layout: [6k]=tw, [6k+1..3]=Wandkraft, [6k+4]=Delta-m, [6k+5]=Normalkontamination (iMEM-Umbau)
	fac_tau_n = Memory<uint>(device, aktiv);
	for(ulong i=0ull; i<FN; i++) fac_idx[i] = 0xFFFFFFFFu;
	ulong k=0ull;
	for(const Facette& f : F) {
		if(f.klasse!=0u) continue;
		const float na = (f.achse==0u) ? fabsf(f.nx) : (f.achse==1u) ? fabsf(f.ny) : fabsf(f.nz);
		fac_geo[8ull*k+0ull]=f.nx; fac_geo[8ull*k+1ull]=f.ny; fac_geo[8ull*k+2ull]=f.nz;
		fac_geo[8ull*k+3ull]=f.yw;
		fac_geo[8ull*k+4ull]=1.0f/fmax(na, 0.57735027f); // Flaechenfaktor, Kappe sqrt(3) (|n_a|>=1/sqrt(3))
		fac_geo[8ull*k+5ull]=(float)f.achse;
		fac_geo[8ull*k+6ull]=0.0f; fac_geo[8ull*k+7ull]=0.0f;
		for(ulong q6=0ull; q6<6ull; q6++) fac_tau[6ull*k+q6]=0.0f;
		fac_tau_n[k]=0u;
		// Zellindex -> F-BBox-Index (dieselbe Formel wie f_bbox im Kernel)
		const uint x=(uint)(f.n%(ulong)Nx), y=(uint)((f.n/(ulong)Nx)%(ulong)Ny), z=(uint)(f.n/((ulong)Nx*(ulong)Ny));
		if(x<fbx0||y<fby0||z<fbz0||x>=fbx0+fbnx||y>=fby0+fbny||z>=fbz0+fbnz) { print_error("Facette ausserhalb der F-BBox -- set_force_bbox deckt die Wandzellen nicht."); return; }
		const ulong fbi=(ulong)(x-fbx0)+((ulong)(y-fby0)+(ulong)(z-fbz0)*(ulong)fbny)*(ulong)fbnx;
		// ★ Invarianten-Waechter (Audit-Entwarnung 2026-08-26): ALLE fac_tau-Buchungen sind
		// nicht-atomare += und racefrei NUR wegen 1 Zelle = 1 Facette. Wuerde eine spaetere
		// Aenderung zwei Facetten auf eine Zelle legen, kaeme das Race STILL -- hier hart abfangen.
		if(fac_idx[fbi]!=0xFFFFFFFFu) { print_error("alloc_facetten_domain: Zelle traegt zwei Facetten -- die racefrei-Invariante (1 Zelle = 1 Facette) waere verletzt."); return; }
		fac_idx[fbi]=(uint)k;
		k++;
	}
	fac_N = aktiv;
	const bool diagz_gebaut = fac_diagz_on; // Audit 2/3: Rebind haengt am KONSTRUKTIONS-Zustand, nicht an der (potentiell umgesetzten) Statik
	if(fac_diagz_on) { // Iron Rule 3: Diagnose-Facette per Zellindex waehlen (CFD_FAC_DIAGZ = n)
		fac_diag = Memory<float>(device, 19ull); // [17] alpha, [18] dp_ds; Selektor bleibt [16]
		for(ulong q=0ull;q<19ull;q++) fac_diag[q]=0.0f;
		fac_diag[16] = -1.0f; ulong k2=0ull;
		for(const Facette& f : F) { if(f.klasse!=0u) { continue; } if(f.n==(ulong)fac_diagz_wert) { fac_diag[16]=(float)k2; fac_diag_fid=(uint)k2; } k2++; }
		if(fac_diag[16]<0.0f) { fac_diagz_on=false; print_warning("CFD_FAC_DIAGZ: Zelle "+to_string((ulong)fac_diagz_wert)+" traegt keine AKTIVE Facette -- Diagnose HART AUS."); }
		// ★ Nachpruefer Stufe-3: auch im Hart-Aus-Fall REBINDEN -- das Move-Assignment hat den als
		// Kernel-Arg gebundenen Platzhalter zerstoert (Use-after-free auf der iGPU-Zero-Copy);
		// der neue Puffer traegt den -1-Sentinel, der Kernelvergleich matcht nie.
		else print_info("Diagnose-Facette: Zelle "+to_string((ulong)fac_diagz_wert)+" -> fid "+to_string((ulong)fac_diag_fid));
		fac_diag.write_to_device();
	}
	if(fac_elibb_on) { // ★★ B1 (K2-LOESUNGSENTSCHEID, 2026-08-25): q je Link aus der ZELLEIGENEN
		// Facettenebene -- Stufe 1 des Entscheids ("die Facette ist die Wand", Heikos Weg).
		// q_d = y_w / (-n . c_d) = Schnitt der Ebene mit dem Link, in Bruchteilen der Linklaenge.
		// Kodierung uchar: 0 = kein Schnitt in (0,1] (Link bleibt implizites HWBB), sonst
		// q = qb/254 -- 127 ist EXAKT 0,5 (254*0,5 = 127, float-exakt; Entscheid: uchar/254
		// statt V1s 4-Bit/14, Restquantisierung 1/508 statt 1/28 Linklaenge).
		// D3Q19-Richtungstabelle WOERTLICH wie der Kernel (c() Spalten, kernel.cpp D3Q19-Zweig).
		static const int CX19[19]={0,1,-1,0,0,0,0,1,-1,1,-1,0,0,1,-1,1,-1,0,0};
		static const int CY19[19]={0,0,0,1,-1,0,0,1,-1,0,0,1,-1,-1,1,0,0,1,-1};
		static const int CZ19[19]={0,0,0,0,0,1,-1,0,0,1,-1,1,-1,0,0,-1,1,-1,1};
		fac_q = Memory<uchar>(device, 18ull*aktiv);
		ulong nq_schnitt=0ull, nq_boden=0ull, nq_klemme1=0ull; ulong hist[15]={0}; // hist: qb/17 grob (0..14)
		// ★★ B1-STUFE 2 (2026-08-25 abends): q aus der GEGLAETTETEN REMESH-FLAECHE hat VORRANG.
		// Die zelleigene PCA-Ebene ist auf Kruemmung als q-Quelle WIDERLEGT (Kugel-Gate RMS 0,67
		// statt <=0,143; auch gefiltert 0,32-0,51). Sie bleibt RUECKFALL ohne Remesh (Kanal: exakt).
		// Grazing-Guard und q-Boden gelten fuer BEIDE Quellen; die Guard-RICHTUNG kommt aus der
		// PCA-Normale (die Richtung ist robust -- nur ihre Distanz war es nicht).
		ulong nq_remesh=0ull, nq_ebene=0ull, nq_ohne_map=0ull, nq_kappe=0ull;
		std::vector<ulong> fac_zelle; fac_zelle.reserve(aktiv);
		for(const Facette& f2 : F) if(f2.klasse==0u) fac_zelle.push_back(f2.n);
		for(ulong kq=0ull; kq<aktiv; kq++) {
			const float nx=fac_geo[8ull*kq], ny=fac_geo[8ull*kq+1ull], nz=fac_geo[8ull*kq+2ull], yw=fac_geo[8ull*kq+3ull];
			const std::array<uchar,18>* qm = nullptr;
			if(qmap!=nullptr) { auto it=qmap->find(fac_zelle[kq]); if(it!=qmap->end()) qm=&it->second; else nq_ohne_map++; }
			for(uint d=1u; d<19u; d++) {
				const float ndc = nx*(float)CX19[d]+ny*(float)CY19[d]+nz*(float)CZ19[d];
				uchar qb=0u;
				const float clen = sqrtf((float)(CX19[d]*CX19[d]+CY19[d]*CY19[d]+CZ19[d]*CZ19[d]));
				if(ndc<-(float)s_fac_kappa*clen) { // ★ GRAZING-GUARD (EIGENES Interim mit offenem Abloese-Soll, eingefuehrt als K1'-Begleiter, gilt unter MLS weiter): nur Links mit -n.c_hat >= kappa (Default 0,4) interpolieren -- streifende Links sind schlecht konditioniert (q = y_w/kleiner Nenner) UND die groessten Tangentialtraeger: die Injektions-Ratsche der Kugel. Darunter: BB (qb=0).
					const float sq = yw/(-ndc); // Bruchteil der Linklaenge
					// ★ QDIAG (2026-08-25, Kugel-Falsifikation): Hypothesen-Arme fuer die Injektionsjagd.
					// 1 = q>1-Klemme AUS (sq>1 -> BB), 2 = nur q<0,5-Zweig (q>0,5 -> Identitaet),
					// 3 = nur q>0,5-Zweig (q<0,5 -> Identitaet). 0 = normal. NUR Diagnose.
					const uint qd = s_fac_qdiag;
					float sqq = -1.0f; // Quellenwahl: Remesh (Stufe 2) VOR Ebene (Rueckfall)
					if(qm!=nullptr) { const uchar rq=(*qm)[d-1u]; if(rq>0u) { sqq=(float)rq*(1.0f/254.0f); nq_remesh++; } }
					else if(sq>0.0f&&sq<=1.0f) { sqq=sq; nq_ebene++; }
					else if(sq>1.0f&&sq<=1.5f) { nq_klemme1++; } // Ebenen-q>1 -> BB (nur ohne Remesh relevant)
					if(sqq>0.0f) {
						float sqe = sqq;
						if(sqe<(float)s_fac_qmin) { sqe=0.5f; nq_boden++; } // q-Boden (P1-Entscheid)
						// ★★ EX-STABILITAETSKAPPE (historisch: Kernel-Audit Befund 1, 25.08. -- der K1'-Zweig
						// war bei kohaerentem q >= 0,75 instabil, die Kappe 0,65 selbst bei ~25k Schritten,
						// s. Wissensspeicher k1instabilitaet). Mit der MLS-Blende (Baustein 1, 26.08.) ist
						// der q>0,5-Zweig bis q=1 stabil -> Default 1,0 = KEINE Kappung: nq_kappe bleibt
						// dann konstruktiv 0 (beide Quellen liefern hier sqe<=1; Ebenen-q>1 faengt die
						// nq_klemme1-Stufe oben ab) -- nq_kappe>0 im Startprotokoll heisst also: Kappe
						// per CFD_FAC_QKAPPE<1 aktiv gesetzt. Env-Hebel bleibt fuer A/Bs erhalten.
						if(sqe>(float)s_fac_qkappe) { fac_q[18ull*kq+(ulong)(d-1u)]=0u; nq_kappe++; continue; }
						if(qd==2u&&sqe>0.5f) sqe=0.5f; // Arm 2: q>0,5 -> Identitaet
						if(qd==3u&&sqe<0.5f) sqe=0.5f; // Arm 3: q<0,5 -> Identitaet
						qb=(uchar)fmin(fmax((float)(int)(sqe*254.0f+0.5f),1.0f),254.0f);
						nq_schnitt++;
					} // ★ q>1 -> BB-RUECKFALL statt Klemme (QDIAG=1 mass das kostenneutral; eingefuehrt als K1'-Begleiter, gilt unter MLS weiter -- Muell-q bleibt Muell-q)
					// sq>1,5: Ebene weit weg -- Link bleibt HWBB (qb=0), kein Zaehler (normaler Fall der Stufenrueckseite)
				}
				fac_q[18ull*kq+(ulong)(d-1u)] = qb;
				if(qb>0u) hist[qb/17u]++;
			}
		}
		fac_q.write_to_device();
		// ★ NACHGEHOLTE B1-ABNAHME (2026-08-25 abends): fac_q + Zellkoordinaten als CSV, damit
		// das Kugel-Gate "Upload-q gegen analytisches q" offline pruefbar ist. Die Leiter lief
		// heute OHNE dieses Gate -- Prozessfehler, im Befundbuch. Nur bei CFD_FAC_QDUMP=1.
		if(getenv("CFD_FAC_QDUMP")) {
			const string qdp = get_exe_path()+"../export/fac_q_dump_"+(getenv("CFD_RUN_NAME")?string(getenv("CFD_RUN_NAME")):string("lauf"))+"_D"+to_string((ulong)Nx)+".csv"; // ★ Host-Audit Befund 4: Run+Domaenen-Suffix statt Kollision
			FILE* fq = fopen(qdp.c_str(), "w");
			if(fq==nullptr) print_warning("fac_q-Dump: "+qdp+" nicht schreibbar.");
			if(fq) {
				fprintf(fq, "# fid,x,y,z,nx,ny,nz,yw,qb1..qb18 (qb/254 = q; 0 = kein Schnitt)\n");
				ulong kq2=0ull;
				for(const Facette& f : F) { if(f.klasse!=0u) continue;
					const uint xx=(uint)(f.n%(ulong)Nx), yy=(uint)((f.n/(ulong)Nx)%(ulong)Ny), zz=(uint)(f.n/((ulong)Nx*(ulong)Ny));
					fprintf(fq, "%lu,%u,%u,%u,%.6f,%.6f,%.6f,%.6f", kq2, xx, yy, zz,
						fac_geo[8ull*kq2], fac_geo[8ull*kq2+1ull], fac_geo[8ull*kq2+2ull], fac_geo[8ull*kq2+3ull]);
					for(uint d=1u; d<19u; d++) fprintf(fq, ",%u", (uint)fac_q[18ull*kq2+(ulong)(d-1u)]);
					fprintf(fq, "\n"); kq2++;
				}
				fclose(fq); print_info("fac_q-Dump: "+qdp+" ("+to_string((ulong)aktiv)+" Facetten).");
			}
		}
		string hs=""; for(uint hb=0u; hb<15u; hb++) hs+=to_string(hist[hb])+(hb<14u?" ":"");
		print_info("ELIBB fac_q: "+to_string(nq_schnitt)+" geschnittene Links auf "+to_string(aktiv)+" Facetten; QUELLE: Remesh "+to_string(nq_remesh)+", Ebenen-Rueckfall "+to_string(nq_ebene)+", ohne Map-Treffer "+to_string(nq_ohne_map)+" Facetten; q-Boden->0,5: "+to_string(nq_boden)+", Ebenen-q>1->BB: "+to_string(nq_klemme1)+", q>Kappe->BB: "+to_string(nq_kappe)+" (CFD_FAC_QKAPPE "+to_string(s_fac_qkappe,2u)+")");
		if(qmap!=nullptr&&nq_remesh==0ull) print_error("ELIBB Stufe 2: Remesh-Map uebergeben, aber NULL Remesh-q verwendet -- lautloser No-Op der Stufe 2.");
		print_info("  q-Histogramm (Bins von 17/254, 0-basiert): "+hs+"  -- kipp0-Gate: ALLES muss im Bin 7 (q=0,5) liegen");
	}
	fac_geo.write_to_device(); fac_idx.write_to_device(); fac_tau.write_to_device(); fac_tau_n.write_to_device();
	kernel_stream_collide.set_parameters(fac_param_pos, fac_geo, fac_idx, fac_tau, fac_tau_n);
	if(fac_ema_on) { fac_us = Memory<float>(device, 3ull*aktiv); for(ulong q3=0ull;q3<3ull*aktiv;q3++) fac_us[q3]=0.0f; fac_us.write_to_device(); kernel_stream_collide.set_parameters(fac_param_pos+4u, fac_us); }
	if(fac_pema_on) { fac_pu = Memory<float>(device, 6ull*aktiv); for(ulong q6=0ull;q6<6ull*aktiv;q6++) fac_pu[q6]=0.0f; fac_pu.write_to_device(); kernel_stream_collide.set_parameters(fac_param_pos+(fac_ema_on?5u:4u), fac_pu); }
	if(diagz_gebaut&&fac_diag.length()>=19ull) kernel_stream_collide.set_parameters(fac_param_pos+4u+(fac_ema_on?1u:0u)+(fac_pema_on?1u:0u), fac_diag); // unkonditional bei DIAGZ-Emission (auch Hart-Aus: Sentinel-Puffer statt zerstoertem Platzhalter)
	if(fac_elibb_on) kernel_stream_collide.set_parameters(fac_param_pos+4u+(fac_ema_on?1u:0u)+(fac_pema_on?1u:0u)+(diagz_gebaut?1u:0u), fac_q); // ★ B2: Rebind des in alloc gebauten fac_q (Signaturposition = nach diagz)
	if(fac_kdiag_on) { fac_kd = Memory<float>(device, 10ull*aktiv); for(ulong q8=0ull;q8<10ull*aktiv;q8++) fac_kd[q8]=0.0f; fac_kd.write_to_device(); kernel_stream_collide.set_parameters(fac_param_pos+4u+(fac_ema_on?1u:0u)+(fac_pema_on?1u:0u)+(diagz_gebaut?1u:0u)+(fac_elibb_on?1u:0u), fac_kd); print_info("Klassen-Diagnostik (CFD_FAC_KDIAG): fac_kd "+to_string((ulong)(40ull*aktiv/1048576ull))+" MB, 10 float je Facette, Tabelle je Treppenklasse am Laufende."); } // ★ Rebind nach fac_q
	if(sgs_gdiag>0u||sgs_fdwand>0u||nachbar_on) { // ★ Liste fid->Zellindex wird von g-Diagnose, Geistermoden-Fix UND Nachbarabtastung (03.09.) gebraucht
		gd_zellen = Memory<ulong>(device, aktiv);
		{ ulong k=0ull; for(const Facette& f : F) { if(f.klasse!=0u) continue; gd_zellen[k++]=f.n; } }
		gd_zellen.write_to_device();
	}
	if(nachbar_on) { // ★ 03.09. DETERMINISTISCHE NACHBARABTASTUNG: Puffer bauen, Kernel binden, stream_collide-Rebind (fac_wfd-Muster, B70-bewiesen)
		fac_nb = Memory<float>(device, 2ull*aktiv);
		for(ulong q=0ull;q<aktiv;q++) { fac_nb[2ull*q]=-1.0f; fac_nb[2ull*q+1ull]=0.0f; } // Init = "kein Wert" -> Eigenzelle (zaehlt als Slot 73); enqueue_initialize fuellt vor dem ersten Schritt
		fac_nb.write_to_device();
		kernel_fac_nachbar = Kernel(device, aktiv, "fac_nachbar_ab", u, flags, fac_geo, gd_zellen, (uint)aktiv, fac_nb);
		if(sparse_on) kernel_fac_nachbar.add_parameters(tile_slot); // B-7-Lehre: TS_P haengt an SPARSE_TILES
		{ const uint nbix=fac_param_pos+4u+(fac_ema_on?1u:0u)+(fac_pema_on?1u:0u)+(diagz_gebaut?1u:0u)+(fac_elibb_on?1u:0u)+(fac_kdiag_on?1u:0u);
		  kernel_stream_collide.set_parameters(nbix, fac_nb); } // Rebind NACH dem Neubau (Platzhalter-Lektion wie fac_wfd)
		print_info("NACHBARABTASTUNG deterministisch (03.09.): Kernel fac_nachbar_ab je Schritt nach stream_collide liest das FERTIGE u-Feld und schreibt (u_t_abt, y_abt) fuer "+to_string(aktiv)+" Facetten; apply_facette_imem liest den Vorschritt (ein Schritt Versatz wie fac_wfd). Der fruehere Direktzugriff u[nb] im selben Kernel war gemessen nicht bitreproduzierbar (xu_det_mit_a/b, 03.09.).");
	}
	if(sgs_fdwand>0u) { // ★ GEISTERMODEN-FIX (02.09.): fac_wfd bauen, FD-Kernel binden, stream_collide-Rebind unten
		fac_wfd = Memory<float>(device, aktiv);
		for(ulong q=0ull;q<aktiv;q++) fac_wfd[q]=1.0f/get_tau(); // = def_w // Init = molekulares w (erster Schritt ohne nu_t an Wandzellen -- dokumentiert harmlos)
		fac_wfd.write_to_device();
		kernel_sgs_fdwand = Kernel(device, aktiv, "sgs_fdwand", u, flags, gd_zellen, (uint)aktiv, fac_wfd);
		if(sparse_on) kernel_sgs_fdwand.add_parameters(tile_slot); // gleiche B-7-Lehre wie sgs_gdiag
		{ const uint fwix=fac_param_pos+4u+(fac_ema_on?1u:0u)+(fac_pema_on?1u:0u)+(diagz_gebaut?1u:0u)+(fac_elibb_on?1u:0u)+(fac_kdiag_on?1u:0u)+(nachbar_on?1u:0u); // +nachbar_on (03.09.): fac_nb sitzt VOR fac_wfd
		  kernel_stream_collide.set_parameters(fwix, fac_wfd); } // ★ Rebind NACH dem Neubau -- der Rebind stand zuerst VOR dem Move-Assignment und band den gleich darauf ZERSTOERTEN Platzhalter (CL -52 beim ersten Enqueue; exakt die DIAGZ-Use-after-free-Lektion, 02.09. erneut bezahlt)
		print_info("SGS-GEISTERMODEN-FIX (CFD_SGS_FDWAND): w an "+to_string(aktiv)+" Facettenzellen aus |S|_FD (u-Feld, geistermodenfrei) statt aus dem Pi-Tensor; FD-Kernel je Schritt nach stream_collide (ein Schritt Versatz, deterministisch), Wirkpfad Slot 76 (B70).");
	}
	if(sgs_gdiag>0u) { // ★ g-DIAGNOSE (31.08., Parameter statt Statik seit 02.09.): Akkumulator + eigener Kernel (Liste oben).
		// KEIN Eingriff in stream_collide, keine Signaturaenderung, kein JIT-Define -- der Kernel ist
		// immer kompiliert und wird nur hier gebunden und spaeter explizit gerufen. Default-Bitgleichheit
		// ist damit trivial (Schalter aus = weder Puffer noch Launch).
		gdiag_on = true;
		fac_gd = Memory<float>(device, 8ull*aktiv);
		for(ulong q8=0ull;q8<8ull*aktiv;q8++) fac_gd[q8]=0.0f;
		fac_gd.write_to_device();
		kernel_sgs_gdiag = Kernel(device, aktiv, "sgs_gdiag", fi, u, flags, gd_zellen, (uint)aktiv, fac_gd, t, fx, fy, fz, s_sgs_guo?1u:0u);
		if(sparse_on) kernel_sgs_gdiag.add_parameters(tile_slot); // Pruefbefund B-7: TS_P haengt an SPARSE_TILES -- ohne dieses Argument stuerbe der erste Launch mit CL_INVALID_KERNEL_ARGS
		print_info("g-DIAGNOSE (CFD_SGS_GDIAG): "+to_string(aktiv)+" Wandzellen, "+to_string((ulong)(40ull*aktiv/1048576ull))+" MB -- misst |S|_FD, |S|_Pi, D_WALE, D_Sigma, |Omega| je Zelle; Physik unangetastet.");
	}
	facetten_bound = true;
	print_info("Facetten gebunden: "+to_string(aktiv)+" aktiv, "+to_string(ausgeschlossen)+" markiert (BB bleibt), Indexfeld "
		+to_string((float)(FN*4ull)/1048576.0f,1u)+" MB, Geometrie "+to_string((float)(aktiv*32ull)/1048576.0f,1u)+" MB auf "+device.info.name+".");
}

void LBM::alloc_facetten(const std::vector<Facette>& F, const std::unordered_map<ulong,std::array<uchar,18>>* qmap, const uint sgs_gdiag, const uint sgs_fdwand) {
	if(get_D()!=1u) { print_error("CFD_FACETTEN ist nur fuer eine Domaene gebaut (dd = zwei getrennte Instanzen)."); return; }
	lbm_domain[0]->alloc_facetten_domain(F, (uint)get_Nx(), (uint)get_Ny(), qmap, sgs_gdiag, sgs_fdwand); // 02.09.: BEIDE Parameter wirklich durchreichen (der Regex-Umbau hatte diese Zeile verfehlt -- Lauf 3 ist am neuen No-Op-Waechter LAUT gescheitert, genau dafuer ist er da)
}

void LBM_Domain::finalize_sparse_tiles() {
	// Nach der Voxelisierung aufrufen: erst dann steht fest, welche Tiles voll solid sind.
	// Eine Tile ist tot, wenn sie SAMT 2-Zell-Halo vollstaendig solid ist. Der Halo muss 2 sein, nicht 1:
	// update_force_field liest via load_f die Nachbarn wand-adjazenter SOLID-Zellen, greift also bis zu
	// zwei Zellen weit -- jede Zelle, die hoechstens 2 von Fluid entfernt ist, muss in einer aktiven Tile
	// bleiben. Mit 1-Halo waeren die Kraefte an der Wand still falsch.
	if(!sparse_on) return;
	const uint Nx=(uint)get_Nx(), Ny=(uint)get_Ny(), Nz=(uint)get_Nz(), T=sparse_T;
	const ulong n_tiles = (ulong)sparse_tiles_x*sparse_tiles_y*sparse_tiles_z;
	auto IDX = [&](const uint x, const uint y, const uint z) { return (ulong)x+((ulong)y+(ulong)z*(ulong)Ny)*(ulong)Nx; };
	std::vector<uint> slot(n_tiles, 0xFFFFFFFFu);
	uint n_active = 0u;
	for(uint tz=0u; tz<sparse_tiles_z; tz++) for(uint ty=0u; ty<sparse_tiles_y; ty++) for(uint tx=0u; tx<sparse_tiles_x; tx++) {
		bool all_solid = true;
		const int x0=(int)(tx*T)-2, x1=(int)(tx*T+T+1), y0=(int)(ty*T)-2, y1=(int)(ty*T+T+1), z0=(int)(tz*T)-2, z1=(int)(tz*T+T+1);
		for(int z=z0; z<=z1&&all_solid; z++) for(int y=y0; y<=y1&&all_solid; y++) for(int x=x0; x<=x1&&all_solid; x++) {
			if(x<0||x>=(int)Nx||y<0||y>=(int)Ny||z<0||z>=(int)Nz) continue; // Domaenenrand zaehlt nicht als Fluid
			if((flags[IDX((uint)x,(uint)y,(uint)z)]&TYPE_S)==0u) all_solid = false;
		}
		// Slots werden ab 1 vergeben: Slot 0 ist ein PAPIERKORB. Grund: store_f schreibt nicht nur an den
		// eigenen Index, sondern auch an die Indizes der Nachbarn. Eine Zelle in einer AKTIVEN Tile kann
		// sehr wohl einen Nachbarn in einer toten Tile haben (sie ist dann selbst solid) -- und cell_base
		// liefert fuer tote Tiles den Notfallwert 0. Zaehlte man die echten Slots ab 0, landeten diese
		// Schreibzugriffe mitten in einer echten Tile und zerstoerten sie. Genau daran sind die ersten
		// T=8- und T=4-Laeufe divergiert (Cd 18.4 bzw. 22.4), auch noch mit is_dead_tile-Ausstieg.
		if(!all_solid) slot[(ulong)tx+(ulong)sparse_tiles_x*((ulong)ty+(ulong)sparse_tiles_y*tz)] = 1u + n_active++;
	}
	for(ulong i=0ull; i<n_tiles; i++) tile_slot[i] = slot[i];
	tile_slot.write_to_device();

	const ulong sparse_cells = (ulong)(n_active+1u)*(ulong)T*T*T; // +1 fuer den Papierkorb-Slot 0
	const double full_gb = (double)get_N()*velocity_set*sizeof(fpxx)/1e9;
	const double sparse_gb = (double)sparse_cells*velocity_set*sizeof(fpxx)/1e9;
	fi = Memory<fpxx>(device, sparse_cells, velocity_set, false); // Move-Assign gibt nur den Platzhalter frei
	kernel_initialize.set_parameters(0u, fi);
	kernel_stream_collide.set_parameters(0u, fi);
	kernel_update_fields.set_parameters(0u, fi);
	kernel_boden_eq.set_parameters(0u, fi); // XL-Audit B2 (Pruefagent R2: NICHT unter FORCE_FIELD): ohne Rebind hielte boden_eq das cl_mem des FREIGEGEBENEN Platzhalters
	kernel_einlass_eq.set_parameters(0u, fi); // EINLASS_EQ: dito (Sparse-Rebind AUSSERHALB von FORCE_FIELD)
#ifdef FORCE_FIELD
	kernel_update_force_field.set_parameters(0u, fi);
#endif // FORCE_FIELD
	print_info("[SPARSE] "+to_string(n_active)+"/"+to_string(n_tiles)+" Tiles aktiv (T="+to_string(T)+") -> fi "
		+to_string((float)sparse_gb,2u)+" GB statt "+to_string((float)full_gb,2u)+" GB, also "
		+to_string((float)(full_gb-sparse_gb),2u)+" GB frei.");
}

void LBM_Domain::enqueue_initialize() { // call kernel_initialize
	kernel_initialize.enqueue_run();
	if(nachbar_on&&fac_N>0ull) kernel_fac_nachbar.enqueue_run(); // ★ 03.09. (Pruefagent Pass 2: der Platzhalter hat Laenge 2, length>1 schuetzt hier NICHT -- fac_N wird nur in alloc_facetten_domain gesetzt): Nachbarwerte schon fuer den ERSTEN Schritt (sonst zaehlte t=0 an allen Facetten als Slot 73)
}
void LBM_Domain::enqueue_stream_collide() { // call kernel_stream_collide to perform one LBM time step
	// ★ Invarianten-Waechter (Pruefagent Rang-1-Remat, NIEDRIG-3): der Remat-Block im Kernel
	// verlaesst sich darauf, dass t ein monotoner Schrittzaehler < 2^62 bleibt (t>>62 == 0).
	if(t>=(1ull<<62)) print_error("enqueue_stream_collide: t >= 2^62 -- die Remat-Invariante (t>>62==0) waere verletzt.");
	kernel_stream_collide.set_parameters(4u, t, fx, fy, fz).enqueue_run();
	if(fdwand_on&&fac_wfd.length()>1ull) kernel_sgs_fdwand.enqueue_run();
	if(nachbar_on&&fac_N>0ull) kernel_fac_nachbar.enqueue_run(); // ★ 03.09. Nachbarabtastung fuer den NAECHSTEN Schritt (Waechter fac_N>0: Platzhalter hat Laenge 2, Pruefagent Pass 2), in-order nach stream_collide (deterministisch); length-Guard = nie auf dem Platzhalter // ★ Geistermoden-Fix: FD-w fuer den NAECHSTEN Schritt, in-order nach stream_collide (deterministisch); length-Guard = nie auf dem Platzhalter
}
void LBM_Domain::enqueue_boden_eq() { // ★ V1-Port: post-stream Boden-Equilibrium (Staggered-Mode-Kur); No-Op bei n==0
	if(boden_eq_n==0u) return;
	kernel_boden_eq.set_parameters(2u, t, boden_eq_u, boden_eq_n, boden_eq_down, boden_eq_split, boden_eq_abstand).enqueue_run();
}
void LBM_Domain::enqueue_einlass_eq() { // ★ V1-Port apply_inlet_velocity: post-stream Einlass-Equilibrium x=1..nx; No-Op bei n==0
	if(einlass_eq_n==0u) return;
	kernel_einlass_eq.set_parameters(2u, t, einlass_eq_u, einlass_eq_n).enqueue_run();
}
void LBM_Domain::sgs_gdiag_gpu() { // ★ g-Diagnose: ein Mess-Launch ueber die Wandzellenliste (31.08.)
	if(!gdiag_on) return;
	kernel_sgs_gdiag.set_parameters(6u, t, fx, fy, fz).run(); // t UND fx/fy/fz aktualisieren (Pruefbefund B-6a: der Kanal REGELT fx je Chunk -- der eingefrorene Startwert verfaelschte den Guo-Term unter SGS_GUO=1); run mit finish
}
void LBM_Domain::enqueue_update_fields() { // update fields (rho, u, T) manually
#ifndef UPDATE_FIELDS
	if(t!=t_last_update_fields) { // only run kernel_update_fields if the time step has changed since last update
		kernel_update_fields.set_parameters(4u, t, fx, fy, fz).enqueue_run();
		t_last_update_fields = t;
	}
#endif // UPDATE_FIELDS
}
#ifdef SURFACE
void LBM_Domain::enqueue_surface_0() {
	kernel_surface_0.set_parameters(7u, t, fx, fy, fz).enqueue_run();
}
void LBM_Domain::enqueue_surface_1() {
	kernel_surface_1.enqueue_run();
}
void LBM_Domain::enqueue_surface_2() {
	kernel_surface_2.set_parameters(4u, t).enqueue_run();
}
void LBM_Domain::enqueue_surface_3() {
	kernel_surface_3.enqueue_run();
}
#endif // SURFACE
#ifdef FORCE_FIELD
void LBM_Domain::enqueue_update_force_field() { // calculate forces from fluid on TYPE_S cells
	if(t!=t_last_force_field) { // only run kernel_update_force_field if the time step has changed since last update
		kernel_update_force_field.set_parameters(2u, t).enqueue_run();
		t_last_force_field = t;
	}
}
void LBM_Domain::enqueue_object_center_of_mass(const uchar flag_marker) { // calculate center of mass of all cells flagged with flag_marker
	object_sum.x[0] = 0.0f; // reset object_sum
	object_sum.y[0] = 0.0f;
	object_sum.z[0] = 0.0f;
	object_sum.enqueue_write_to_device();
	kernel_object_center_of_mass.set_parameters(1u, flag_marker).enqueue_run();
	object_sum.enqueue_read_from_device();
}
void LBM_Domain::enqueue_object_force(const uchar flag_marker) { // add up force for all cells flagged with flag_marker
	enqueue_update_force_field(); // update force field if it is not yet up-to-date
	// Kein Nullen mehr noetig: object_force_final schreibt mit "=", und jeder Teilsummen-Slot wird
	// von seiner Arbeitsgruppe jeden Lauf unbedingt geschrieben (Muster po_final_mean, 849b14f).
	kernel_object_force.set_parameters(2u, flag_marker).enqueue_run();
	kernel_object_force_final.enqueue_run();
	object_sum.enqueue_read_from_device();
}
void LBM_Domain::enqueue_object_force_zband(const uchar flag_marker, const uint z_lo, const uint z_hi) { // FORK Kraft-Zerlegung: object_force auf das z-Band [z_lo,z_hi); object_sum WIEDERVERWENDET -- strikt sequenziell zu enqueue_object_force
	enqueue_update_force_field(); // update force field if it is not yet up-to-date
	// Kein Nullen mehr noetig: object_force_final schreibt mit "=", und jeder Teilsummen-Slot wird
	// von seiner Arbeitsgruppe jeden Lauf unbedingt geschrieben (Muster po_final_mean, 849b14f).
	kernel_object_force_zband.set_parameters(2u, flag_marker, z_lo, z_hi).enqueue_run();
	kernel_object_force_final.enqueue_run();
	object_sum.enqueue_read_from_device();
}
void LBM_Domain::enqueue_object_torque(const float3& rotation_center, const uchar flag_marker) { // add up torque around specified rotation_center for all cells flagged with flag_marker
	enqueue_update_force_field(); // update force field if it is not yet up-to-date
	object_sum.x[0] = 0.0f; // reset object_sum
	object_sum.y[0] = 0.0f;
	object_sum.z[0] = 0.0f;
	object_sum.enqueue_write_to_device();
	kernel_object_torque.set_parameters(2u, flag_marker, rotation_center.x, rotation_center.y, rotation_center.z).enqueue_run();
	object_sum.enqueue_read_from_device();
}
// ★ kraft_facetten-GPU-Reduktion (Muster init_pressure_outlet/set_pressure_outlet_faces): Liste der
// Markerzellen hochladen, Gruppenpuffer anlegen, Kernel binden. Der Schluessel (marker,z_per) steht
// in kf_marker/kf_zper -- der Aufrufer (setup.cpp kraft_facetten) bindet bei Wechsel neu.
void LBM_Domain::bind_kraft_facetten(const std::vector<ulong>& liste, const uchar marker, const bool z_per, const bool band_slot) {
	// ★ FORK Kraft-Zerlegung: band_slot=true waehlt den kfb_*-Membersatz (z-Band-Teilliste), sonst
	// laeuft alles wortgleich ueber den Hauptslot. kfb_zband setzt der Aufrufer (setup.cpp).
	Memory<ulong>& liste_m = band_slot ? kfb_liste : kf_liste;
	Memory<float>& psum_m  = band_slot ? kfb_psum  : kf_psum;
	Memory<uint>&  pcnt_m  = band_slot ? kfb_pcnt  : kf_pcnt;
	Kernel& kernel_m = band_slot ? kernel_kraft_facetten_band : kernel_kraft_facetten;
	const ulong liste_n = (ulong)liste.size();
	if(liste_n>0xFFFFFFFFull) print_error("kraft_facetten-Liste ueberschreitet 2^32 Zellen -- uint-Cast im Kernel-Argument wuerde stumm abschneiden (R1-N4)."); // praktisch unerreichbar, aber billig
	if(band_slot) { kfb_N = liste_n; kfb_marker = marker; kfb_zper = z_per; kfb_bound = true; } // eigene Schluessel (Pruefagent M)
	else { kf_N = liste_n; kf_marker = marker; kf_zper = z_per; kf_bound = true; }
	if(liste_n==0ull) return; // leere Liste: kraft_facetten_gpu liefert Nullen ohne Launch
	liste_m = Memory<ulong>(device, liste_n); // Ctor-Nullinit + zweiter Voll-Write = ein verschenkter 16-MB-Transfer, EINMALIG beim Bind -- bewusst toleriert (Pruefagent N2)
	for(ulong i=0ull; i<liste_n; i++) liste_m[i] = liste[i];
	liste_m.write_to_device();
	const ulong gruppen = (liste_n+(ulong)WORKGROUP_SIZE-1ull)/(ulong)WORKGROUP_SIZE; // = ceil(liste_n/64.0)
	psum_m = Memory<float>(device, 3ull*gruppen);
	pcnt_m = Memory<uint>(device, 3ull*gruppen);
	// Im AUS-Arm (!facetten_on) existieren fac_idx/fac_tau_n/fac_geo NICHT (allocate bindet sie nur
	// mit facetten_on, s.o.) -- 1-Element-Dummies anlegen, damit der Kernel gueltige Puffer bekommt.
	// NUR wenn noch nie alloziert (length()==0): ein Move-Assignment auf einen bereits als Kernel-Arg
	// gebundenen Puffer waere genau der DIAGZ-Use-after-free (Nachpruefer-Lektion in alloc_facetten_domain).
	if(!facetten_on) {
		if(fac_idx.length()==0ull)   { fac_idx   = Memory<uint>(device, 1ull);  fac_idx[0]=0xFFFFFFFFu; fac_idx.write_to_device(); }
		if(fac_tau_n.length()==0ull) { fac_tau_n = Memory<uint>(device, 1ull);  fac_tau_n[0]=0u;        fac_tau_n.write_to_device(); }
		if(fac_geo.length()==0ull)   { fac_geo   = Memory<float>(device, 8ull); for(ulong q=0ull;q<8ull;q++) fac_geo[q]=0.0f; fac_geo.write_to_device(); }
	}
	kernel_m = Kernel(device, liste_n, "kraft_facetten_gpu", F, liste_m, (uint)liste_n,
		fac_idx, fac_tau_n, fac_geo, facetten_on?1u:0u, z_per?1u:0u, psum_m, pcnt_m);
}
void LBM_Domain::kraft_facetten_gpu(double& px, double& py, double& pz, ulong& n_voll, ulong& n_proj, ulong& n_unklar, const bool band_slot) {
	px=py=pz=0.0; n_voll=n_proj=n_unklar=0ull;
	const ulong liste_n = band_slot ? kfb_N : kf_N; // ★ FORK Kraft-Zerlegung: band_slot -> kfb_*-Satz
	if(liste_n==0ull) return; // keine Markerzellen: Nullen ohne Launch
	// ★ run() MIT finish, nicht enqueue_run(): kf_psum/kf_pcnt sind auf CPU/iGPU ZERO-COPY-Puffer
	// (CL_MEM_USE_HOST_PTR) -- dort erzwingt der "blockierende" read_from_device KEINE Ausfuehrung
	// der wartenden Kommandos, und der Kernel lief erst mit dem naechsten Queue-Flush. Gemessen als
	// Ein-Aufruf-Versatz im PRUEF-Doppellauf (GPU-Werte = Host-Werte des VORHERIGEN Aufrufs, erster
	// Aufruf Nullen). Die in-order-Queue stellt zugleich sicher, dass enqueue_update_force_field
	// davor abgearbeitet ist.
	Memory<float>& psum_m = band_slot ? kfb_psum : kf_psum;
	Memory<uint>&  pcnt_m = band_slot ? kfb_pcnt : kf_pcnt;
	(band_slot ? kernel_kraft_facetten_band : kernel_kraft_facetten).run();
	psum_m.read_from_device();
	pcnt_m.read_from_device();
	const ulong gruppen = (liste_n+(ulong)WORKGROUP_SIZE-1ull)/(ulong)WORKGROUP_SIZE;
	for(ulong g=0ull; g<gruppen; g++) { // double-Endsumme in FESTER Gruppenreihenfolge (deterministisch)
		px += (double)psum_m[3ull*g]; py += (double)psum_m[3ull*g+1ull]; pz += (double)psum_m[3ull*g+2ull];
		n_voll += (ulong)pcnt_m[3ull*g]; n_proj += (ulong)pcnt_m[3ull*g+1ull]; n_unklar += (ulong)pcnt_m[3ull*g+2ull];
	}
}
#endif // FORCE_FIELD
#ifdef MOVING_BOUNDARIES
void LBM_Domain::enqueue_update_moving_boundaries() { // mark/unmark cells next to TYPE_S cells with velocity!=0 with TYPE_MS
	kernel_update_moving_boundaries.enqueue_run();
}
#endif // MOVING_BOUNDARIES
#ifdef PARTICLES
void LBM_Domain::enqueue_integrate_particles(const uint time_step_multiplicator) { // intgegrate particles forward in time and couple particles to fluid
#ifdef FORCE_FIELD
	if(particles_rho!=1.0f) kernel_reset_force_field.enqueue_run(); // only reset force field if particles have buoyancy and apply forces on fluid
	kernel_integrate_particles.set_parameters(5u, fx, fy, fz);
#endif // FORCE_FIELD
	kernel_integrate_particles.set_parameters(3u, (float)time_step_multiplicator).enqueue_run();
}
#endif // PARTICLES

void LBM_Domain::increment_time_step(const ulong steps) {
	t += steps; // increment time step
#ifdef UPDATE_FIELDS
	t_last_update_fields = t;
#endif // UPDATE_FIELDS
}
void LBM_Domain::reset_time_step() {
	t = 0ull; // increment time step
#ifdef UPDATE_FIELDS
	t_last_update_fields = t;
#endif // UPDATE_FIELDS
}
void LBM_Domain::flush_queue() { device.flush_queue(); } // Perf-Audit: siehe run_async

void LBM_Domain::finish_queue() {
	device.finish_queue();
}

uint LBM_Domain::get_velocity_set() const {
	return velocity_set;
}

void LBM_Domain::voxelize_mesh_on_device(const Mesh* mesh, const uchar flag, const float3& rotation_center, const float3& linear_velocity, const float3& rotational_velocity) { // voxelize triangle mesh
	Memory<float3> p0(device, mesh->triangle_number, 1u, mesh->p0);
	Memory<float3> p1(device, mesh->triangle_number, 1u, mesh->p1);
	Memory<float3> p2(device, mesh->triangle_number, 1u, mesh->p2);
	Memory<float> bounding_box_and_velocity(device, 16u);
	const float x0=mesh->pmin.x-2.0f, y0=mesh->pmin.y-2.0f, z0=mesh->pmin.z-2.0f, x1=mesh->pmax.x+2.0f, y1=mesh->pmax.y+2.0f, z1=mesh->pmax.z+2.0f; // use bounding box of mesh to speed up voxelization; add tolerance of 2 cells for re-voxelization of moving objects
	bounding_box_and_velocity[ 0] = as_float(mesh->triangle_number);
	bounding_box_and_velocity[ 1] = x0;
	bounding_box_and_velocity[ 2] = y0;
	bounding_box_and_velocity[ 3] = z0;
	bounding_box_and_velocity[ 4] = x1;
	bounding_box_and_velocity[ 5] = y1;
	bounding_box_and_velocity[ 6] = z1;
	bounding_box_and_velocity[ 7] = rotation_center.x;
	bounding_box_and_velocity[ 8] = rotation_center.y;
	bounding_box_and_velocity[ 9] = rotation_center.z;
	bounding_box_and_velocity[10] = linear_velocity.x;
	bounding_box_and_velocity[11] = linear_velocity.y;
	bounding_box_and_velocity[12] = linear_velocity.z;
	bounding_box_and_velocity[13] = rotational_velocity.x;
	bounding_box_and_velocity[14] = rotational_velocity.y;
	bounding_box_and_velocity[15] = rotational_velocity.z;
	uint direction = 0u;
	if(length(rotational_velocity)==0.0f) { // choose direction of minimum bounding-box cross-section area
		float v[3] = { (y1-y0)*(z1-z0), (z1-z0)*(x1-x0), (x1-x0)*(y1-y0) };
		float vmin = v[0];
		for(uint i=1u; i<3u; i++) {
			if(v[i]<vmin) {
				vmin = v[i];
				direction = i;
			}
		}
	} else { // choose direction closest to rotation axis
		float v[3] = { fabsf(rotational_velocity.x), fabsf(rotational_velocity.y), fabsf(rotational_velocity.z) };
		float vmax = v[0];
		for(uint i=1u; i<3u; i++) {
			if(v[i]>vmax) {
				vmax = v[i];
				direction = i; // find direction of minimum bounding-box cross-section area
			}
		}
	}
	const ulong A[3] = { (ulong)Ny*(ulong)Nz, (ulong)Nz*(ulong)Nx, (ulong)Nx*(ulong)Ny };
	Kernel kernel_voxelize_mesh(device, A[direction], "voxelize_mesh", direction, fi, u, flags, t+1ull, flag, p0, p1, p2, bounding_box_and_velocity);
#ifdef SURFACE
	kernel_voxelize_mesh.add_parameters(mass, massex);
#endif // SURFACE
	if(sparse_on) kernel_voxelize_mesh.add_parameters(tile_slot); // TS_P haengt tile_slot hinten an
	p0.write_to_device();
	p1.write_to_device();
	p2.write_to_device();
	bounding_box_and_velocity.write_to_device();
	kernel_voxelize_mesh.run();
}
void LBM_Domain::enqueue_unvoxelize_mesh_on_device(const Mesh* mesh, const uchar flag) { // remove voxelized triangle mesh from LBM grid
	const float x0=mesh->pmin.x, y0=mesh->pmin.y, z0=mesh->pmin.z, x1=mesh->pmax.x, y1=mesh->pmax.y, z1=mesh->pmax.z; // remove all flags in bounding box of mesh
	Kernel kernel_unvoxelize_mesh(device, get_N(), "unvoxelize_mesh", flags, flag, x0, y0, z0, x1, y1, z1);
	kernel_unvoxelize_mesh.run();
}

// ★ Auditor-B B-1 (26.08.): werkzeuge/scratch_gate/gen_main.cpp FRIERT diese Define-Liste fuer
// das Offline-Scratch-Gate ein (Kanal-Referenzpunkt). Wer hier Defines aendert/ergaenzt, zieht
// den Zwilling nach -- sonst prueft das Gate still eine Quelle, die niemand mehr faehrt.
// Voller Drift-Anker (Gate difft gegen frischen CFD_DUMP_DEFINES-Dump): Folgepunkt im Plan.
string LBM_Domain::device_defines(const Device_Info& device_info) const { return
	"\n	#define def_Nx "+to_string(Nx)+"u"
	"\n	#define def_Ny "+to_string(Ny)+"u"
	"\n	#define def_Nz "+to_string(Nz)+"u"
	"\n	#define def_N "+to_string(get_N())+"ul"
	"\n	#define uxx "+(get_N()<=(ulong)max_uint ? "uint" : "ulong")+"" // switchable data type for index calculation (32-bit uint / 64-bit ulong)

	"\n	#define def_GNx "+to_string((Nx-2u*(uint)(Dx>1u))*Dx)+"u" // global LBM grid resolution of all domains together
	"\n	#define def_GNy "+to_string((Ny-2u*(uint)(Dy>1u))*Dy)+"u"
	"\n	#define def_GNz "+to_string((Nz-2u*(uint)(Dz>1u))*Dz)+"u"

	"\n	#define def_Dx "+to_string(Dx)+"u"
	"\n	#define def_Dy "+to_string(Dy)+"u"
	"\n	#define def_Dz "+to_string(Dz)+"u"

	"\n	#define def_Ox "+to_string(Ox)+"" // offsets are signed integer!
	"\n	#define def_Oy "+to_string(Oy)+""
	"\n	#define def_Oz "+to_string(Oz)+""

	"\n	#define def_Ax "+to_string(Ny*Nz)+"u"
	"\n	#define def_Ay "+to_string(Nz*Nx)+"u"
	"\n	#define def_Az "+to_string(Nx*Ny)+"u"

	"\n	#define def_domain_offset_x "+to_string(0.5f*(float)((int)Nx+2*Ox+(int)Dx*(2*(int)(Dx>1u)-(int)Nx)))+"f"
	"\n	#define def_domain_offset_y "+to_string(0.5f*(float)((int)Ny+2*Oy+(int)Dy*(2*(int)(Dy>1u)-(int)Ny)))+"f"
	"\n	#define def_domain_offset_z "+to_string(0.5f*(float)((int)Nz+2*Oz+(int)Dz*(2*(int)(Dz>1u)-(int)Nz)))+"f"

	"\n	#define D"+to_string(dimensions)+"Q"+to_string(velocity_set)+"" // D2Q9/D3Q15/D3Q19/D3Q27
	"\n	#define def_velocity_set "+to_string(velocity_set)+"u" // LBM velocity set (D2Q9/D3Q15/D3Q19/D3Q27)
	"\n	#define def_dimensions "+to_string(dimensions)+"u" // number spatial dimensions (2D or 3D)
	"\n	#define def_transfers "+to_string(transfers)+"u" // number of DDFs that are transferred between multiple domains

	"\n	#define def_c 0.57735027f" // lattice speed of sound c = 1/sqrt(3)*dt
	"\n	#define def_w " +to_string(1.0f/get_tau())+"f" // relaxation rate w = dt/tau = dt/(nu/c^2+dt/2) = 1/(3*nu+1/2)
#if defined(D2Q9)
	"\n	#define def_w0 (1.0f/2.25f)" // center (0)
	"\n	#define def_ws (1.0f/9.0f)" // straight (1-4)
	"\n	#define def_we (1.0f/36.0f)" // edge (5-8)
#elif defined(D3Q15)
	"\n	#define def_w0 (1.0f/4.5f)" // center (0)
	"\n	#define def_ws (1.0f/9.0f)" // straight (1-6)
	"\n	#define def_wc (1.0f/72.0f)" // corner (7-14)
#elif defined(D3Q19)
	"\n	#define def_w0 (1.0f/3.0f)" // center (0)
	"\n	#define def_ws (1.0f/18.0f)" // straight (1-6)
	"\n	#define def_we (1.0f/36.0f)" // edge (7-18)
#elif defined(D3Q27)
	"\n	#define def_w0 (1.0f/3.375f)" // center (0)
	"\n	#define def_ws (1.0f/13.5f)" // straight (1-6)
	"\n	#define def_we (1.0f/54.0f)" // edge (7-18)
	"\n	#define def_wc (1.0f/216.0f)" // corner (19-26)
#endif // D3Q27

#if defined(SRT)
	"\n	#define SRT"
#elif defined(TRT)
	"\n	#define TRT"
	// ★★ LAMBDA ALS LAUFZEITPARAMETER, 2026-08-09. Bisher stand 0.1875f hart im Kernel.
	// Gerechnet (von-Neumann, D3Q19, tau+ = 0,5000071, u_lat = 0,075):
	//   Lambda = 3/16  -> tau- = 26409, max|Eigenwert| = 1,005543, e-Faltung 181 Schritte
	//   Lambda = 1/4   -> praktisch unveraendert (w- ist laengst null) -- der Umbau brachte NICHTS
	//   SRT            -> max|Eigenwert| = 1,003480, e-Faltung 288 Schritte  (GEMESSEN 2,6x besser)
	//   Lambda = 9,1e-8 (w- = 1,95) -> max|Eigenwert| = 1,000485, e-Faltung 2062 Schritte
	// SRT ist der Sonderfall Lambda = (tau-1/2)^2. Ein Knopf deckt damit ALLE Operatoren ab, und
	// ungesetzt bleibt der Quelltext bit-identisch zum bisherigen Stand.
	+"\n	#define def_lambda "+to_string(getenv("CFD_LAMBDA")!=nullptr?(float)atof(getenv("CFD_LAMBDA")):0.1875f, 12u)+"f"
#endif // TRT

	"\n	#define TYPE_S 0x01" // 0b00000001 // (stationary or moving) solid boundary
	"\n	#define TYPE_E 0x02" // 0b00000010 // equilibrium boundary (inflow/outflow)
	"\n	#define TYPE_T 0x04" // 0b00000100 // temperature boundary
	"\n	#define TYPE_F 0x08" // 0b00001000 // fluid
	"\n	#define TYPE_I 0x10" // 0b00010000 // interface
	"\n	#define TYPE_G 0x20" // 0b00100000 // gas
	"\n	#define TYPE_X 0x40" // 0b01000000 // reserved type X
	"\n	#define TYPE_Y 0x80" // 0b10000000 // reserved type Y

	// ★★ DAEMPFUNGSZONE (Sponge), 2026-08-09 -- GEOMETRISCH an den Domaenenflaechen x-/x+/y-/y+/z+
	// verankert (Abstand zur Flaeche, KEIN Flag-Test; R2-Korrektur: der alte Text behauptete eine
	// TYPE_E-Bindung, die es nie gab -- in einem periodischen Setup rampte die Zone an Flaechen
	// ohne Rand, heute nur per Konvention verhindert, der Kanal setzt sie nicht). DER Weg, der nach drei
	// gescheiterten Randumbauten uebrig bleibt -- und der einzige, der durch Messung gestuetzt ist:
	// im leeren Fernfeld hat AUSSCHLIESSLICH die Viskositaet gedaempft (nu x1000: Streuung 0,040 ->
	// 0,025), waehrend jede Aenderung der Randgleichung das Klingeln verstaerkte. Die Zone hebt nu
	// nur in einem Streifen vor den Raendern an (quadratische Rampe), also dort, wo die Quelle sitzt
	// und die Reflexionen laufen -- nicht im Messvolumen. Sie liest nichts zurueck, erhaelt Masse
	// und Impuls (reine Aenderung der Relaxationsrate) und ist unter Esoteric Pull trivial sicher.
	// Der Boden z=0 ist AUSGENOMMEN: dort ist Fahrbahn, keine TYPE_E-Flaeche, und die Grenzschicht
	// darf nicht kuenstlich verdickt werden. V1s wirksame Klemmschicht war der harte Vorlaeufer
	// dieser Idee -- gleicher Ort, aber als f-Reset statt als Viskositaet.
	// Nur emittiert, wenn CFD_SPONGE_N gesetzt ist; ohne die Variable ist der Quelltext bit-identisch.
	// ★ Aus den STATIKEN, nicht aus getenv -- siehe die Begruendung bei ihrer Deklaration in lbm.hpp.
	// Das Setup setzt sie vor jedem Konstruktor; damit ist die Zone pro Domaene schaltbar, und die
	// Auswertung der Umgebungsvariablen laeuft ueber env_u/env_f im Setup, die den WERT auswerten
	// (CFD_SPONGE_N=0 heisst aus) statt nur auf das Literal "0" zu pruefen.
	+((s_sponge_n>0u) ? (string)
	"\n	#define SPONGE"
	"\n	#define def_sponge_n "+to_string(s_sponge_n)+"u"
	"\n	#define def_sponge_a "+to_string(s_sponge_a,4u)+"f" // 4 Nachkommastellen: bei kleinen a war 1 Stelle irrefuehrend
	"\n	#define def_sponge_wmin "+to_string(s_sponge_wmin,4u)+"f"
	: (string)"")
	// FORK: REG_E(i) ist der Randwert einer TYPE_E-Zelle -- reines Gleichgewicht wie bisher, oder mit
	// REGULARIZED_BOUNDARIES zusaetzlich der rekonstruierte Nichtgleichgewichtsanteil.
	//
	// ★★ ZWEI LEHREN VOM 2026-08-08 stecken in diesen paar Zeilen:
	// (1) Das Makro war einmal der VOLLE 19-Richtungs-Ausdruck und expandierte 19-fach in die ternaere
	//     Kollisionszeile. Der Intel-Uebersetzer blieb daran haengen; beim zweiten Mal fror der ganze
	//     Rechner ein, weil der Desktop auf derselben GPU laeuft. Jetzt ist es ein Aufruf der kleinen
	//     Funktion reg_fneq(), und der TYPE_E-Zweig ist in der Kollision herausgehoben.
	// (2) Der Schalter ist eine LAUFZEIT-Entscheidung (CFD_REG_BC), keine Compile-Zeit-Entscheidung.
	//     Damit liefert DASSELBE Binary mit CFD_REG_BC=0 exakt den alten OpenCL-Quelltext -- das ist
	//     der bit-genaue Kontrollarm fuer jedes A/B, und zugleich der Rettungsanker, falls der
	//     GPU-Uebersetzer am regularisierten Code doch wieder haengen sollte.
	// ★★ GEMESSEN UND DEFAULT AUS, 2026-08-09. A/B im leeren Fernfeld, gleiches Binary, Kontrollarm:
	//   bei 0,08 s Streuung 0,0719 (aus) gegen 0,1042 (an), Zellen ueber 10 % daneben 12,1 gegen 20,2 %.
	// Der regularisierte Einlass VERSTAERKT das Klingeln. Es ist der dritte Randumbau, der am selben
	// Muster scheitert: S wird aus dem u[] der Nachbarn gebildet, und die erste Fluidzelle ist genau
	// die, die die Stoerung traegt -- der Rand koppelt das Rauschen auf sich selbst zurueck, und bei
	// w -> 2 daempft die Kollision nichts, sie spiegelt. Der reine Gleichgewichts-Reset ist in diesem
	// Regime der am wenigsten schaedliche Rand, WEIL er nichts zurueckliest.
	// Der Code bleibt (mathematisch korrekt, Erhaltung symbolisch bestaetigt) fuer Regimes mit
	// ordentlichem tau; CFD_REG_BC=1 schaltet ihn ein.
#ifdef REGULARIZED_BOUNDARIES
	// Audit-Nacharbeit 15: atoi statt "alles ausser '0' ist an" -- CFD_REG_BC=false hiess vorher AN.
	+((getenv("CFD_REG_BC")!=nullptr&&atoi(getenv("CFD_REG_BC"))>0) ? (string)
	"\n	#define REGULARIZED_BOUNDARIES"
	"\n	#define REG_E(i) (feq[i]+reg_fneq(i, regf, Sxx, Syy, Szz, Sxy, Sxz, Syz, trS3))"
	: (string)
	"\n	#define REG_E(i) (feq[i])")
	+
#else
	"\n	#define REG_E(i) (feq[i])"
#endif // REGULARIZED_BOUNDARIES
	#ifdef RHO_CLAMP
	"\n	#define RHO_CLAMP"
	"\n	#define RHO_CLAMP_MIN "+to_string(RHO_CLAMP_MIN,4u)+"f"
	"\n	#define RHO_CLAMP_MAX "+to_string(RHO_CLAMP_MAX,4u)+"f"
#endif // RHO_CLAMP
	// ★ Audit-Nacharbeit 2: SGS_WANDFREI und WANDFUNKTION standen im #ifdef-SUBGRID-Block -- mit
	// abgeschaltetem SUBGRID (die Kugel-Validierung verlangt das) waeren beide LAUTLOSE No-Ops
	// gewesen. Jetzt ausserhalb emittiert; SGS_WANDFREI ohne SUBGRID ist sinnlos und wird im
	// Konstruktor hart abgewiesen, die WFB ist von SUBGRID unabhaengig.
	+((s_sgs_wandfrei) ? (string)"\n	#define SGS_WANDFREI" : (string)"")
	+((s_sgs_guo)      ? (string)"\n	#define SGS_GUO"      : (string)"") // ★ 2026-08-25 Guo-Korrektur von Pi^neq, Default AN
	+((s_sgs_diag)     ? (string)"\n	#define SGS_DIAG"     : (string)"")
	+((s_sgs_diag)     ? (string)"\n	#define def_sgs_diag_ab "+to_string(s_sgs_diag_ab)+"ul" : (string)"")
	+((s_wandfunktion) ? (string)"\n	#define WANDFUNKTION"
	"\n	#define def_wf_Y "+to_string(0.5f/nu,8u)+"f"
	"\n	#define def_wf_tau "+to_string(s_wf_tau,4u)+"f"
	"\n	#define def_wf_spalding_it "+to_string(max(1u,env_u("CFD_SPALDING_IT",3u)))+"u" : (string)"")
	// ★ C1b Stufe 2 (FACETTEN-STUFE2.md F4): def_fac_Y ueber WOERTLICH dieselbe Emissionskette wie
	// def_wf_Y -- der Aequivalenznachweis am Kanal (yw=0,5, fac_a=1) kollabiert dann bitgenau.
	+((s_facetten) ? (string)"\n	#define FACETTEN"
	"\n	#define def_fac_Y "+to_string(0.5f/nu,8u)+"f"
	+"\n	#define def_fac_utkorr "+to_string(s_fac_utkorr, 6u)+"f" // ★ 3/2-Abtastpunkt-Messarm (CFD_FAC_UTKORR, Default 1,0 = bitgleich)
	"\n	#define def_fac_tau "+to_string(s_fac_tau,4u)+"f"
	// ★ MLS-Blende (Baustein 1, 26.08.): chi-Nenner 1/(tau0+0,5), tau0=3nu+0,5 (SRT, cs^2=1/3).
	// INTERIM I1 (deklariert im kernel.cpp-MLS-Block): tau0 statt lokalem SUBGRID-tau_eff.
	"\n	#define def_fac_chifak "+to_string(1.0f/(3.0f*nu+1.0f),8u)+"f"
	"\n	#define def_fac_budget "+to_string(s_fac_budget,4u)+"f"
	"\n	#define def_fac_budget_sn "+to_string(s_fac_budget_sn,4u)+"f"
	"\n	#define def_wf_spalding_it "+to_string(max(1u,env_u("CFD_SPALDING_IT",3u)))+"u" : (string)"")
	+((s_facetten&&s_fac_imem) ? (string)"\n	#define FACETTEN_IMEM" : (string)"") // iMEM-Umbau: Arme 3/4 (Splice ausserhalb R() -- Werkzeugfalle)
	+((s_facetten&&s_fac_imem&&s_fac_ema>0.0f) ? (string)"\n	#define FACETTEN_EMA"
	"\n	#define def_fac_ema "+to_string(s_fac_ema,6u)+"f" : (string)"") // EMA nur wenn gesetzt -- ungesetzt bitgleich zum 3x3-ohne-EMA
	+((s_facetten&&s_fac_imem&&s_fac_satgate) ? (string)"\n	#define FACETTEN_SATGATE" : (string)"") // (a-strich): Klemme -> BB-Rueckfall
	+((s_facetten&&s_fac_imem&&s_fac_alpha>0u) ? (string)"\n	#define FACETTEN_ALPHA" : (string)"") // J4-alpha: Massenkorrektur, Sum q = 0 je Facette
	+((s_facetten&&s_fac_imem&&s_fac_alpha>1u) ? (string)"\n	#define FACETTEN_ALPHA2" : (string)"")
	+((s_facetten&&s_fac_imem&&s_fac_messnur>0u) ? (string)"\n	#define FACETTEN_MESSNUR" : (string)"") // ★ 30.08. BB-Physik, nur messen
	+((s_facetten&&s_fac_imem&&s_fac_nachbar>0u) ? (string)"\n	#define FACETTEN_NACHBAR" : (string)"") // ★ 30.08. Eingang aus der zweiten Fluidzelle
	+((s_facetten&&s_fac_imem&&s_fac_kdiag>0u) ? (string)"\n	#define FACETTEN_KDIAG" : (string)"") // ★ 30.08. Klassen-Diagnostik
	+((s_facetten&&s_sgs_fdwand>0u) ? (string)"\n	#define SGS_FDWAND" : (string)"") // ★ 02.09. Geistermoden-Fix (braucht Facetten fuer fac_idx, nicht zwingend iMEM -- wirkt auch im MESSNUR/BB-Arm)
	+((s_facetten&&s_fac_imem&&s_fac_elibb) ? (string)"\n	#define FACETTEN_ELIBB" : (string)"") // ★ B2 (2026-08-25): ELIBB 18-Link, q aus der Facettenebene
	+((s_facetten&&s_fac_imem&&s_fac_elibb_pur) ? (string)"\n	#define FACETTEN_ELIBB_PUR" : (string)"") // ★ Pur-Arm: NUR Geometrie-Blende (CFD_FAC_ELIBB=2)
	+((s_facetten&&s_fac_imem&&s_fac_lsq) ? (string)"\n	#define FACETTEN_LSQ" : (string)"")
	+((s_facetten&&s_fac_imem&&s_fac_quergate) ? (string)"\n	#define FACETTEN_QUERGATE" : (string)"")
	+((s_facetten&&s_fac_imem&&s_fac_kraft>0u) ? (string)"\n	#define FACETTEN_KRAFT\n	#define def_fac_kraft "+to_string(min(2u,s_fac_kraft))+"u" : (string)"") // ★ 30.08. Zellkraft statt Slip (Weg F) // ★ 2026-08-25 Querimpuls-Gate, Slot 64 // ★ 2026-08-25 kleinste Quadrate statt Skalar-Rueckfall (CFD_FAC_LSQ, Default 1)
	+((s_facetten&&s_fac_imem&&s_fac_apg!=0.0f) ? (string)"\n	#define FACETTEN_APG"
	"\n	#define def_fac_apg "+to_string(s_fac_apg,6u)+"f" : (string)"") // APG-Messarm: Emission nur bei kappa != 0 (Kommentar-Verklebung R2 geloest) /* ALPHA2 setzt ALPHA voraus (S0/alph undeklariert sonst) -- die >1/>0-Paarung hier ist die einzige Garantie (Audit 1/3) */ // J4-alpha Stufe 2: Momenten-Downdate (Impuls-Projektion)
	+((s_facetten&&s_fac_imem&&s_fac_pema>0.0f) ? (string)"\n	#define FACETTEN_PEMA"
	"\n	#define def_fac_pema "+to_string(s_fac_pema,6u)+"f" : (string)"") // PEMA (Weg A): Eingangs-Filterung
	+((s_facetten&&s_fac_imem&&s_fac_diagz>=0l) ? (string)"\n	#define FACETTEN_DIAGZ" : (string)"") // Ziel-fid zur Laufzeit in fac_diag[16]
	+"\n	#define TYPE_MS 0x03" // 0b00000011 // cell next to moving solid boundary
	"\n	#define TYPE_BO 0x03" // 0b00000011 // any flag bit used for boundaries (temperature excluded)
	"\n	#define TYPE_IF 0x18" // 0b00011000 // change from interface to fluid
	"\n	#define TYPE_IG 0x30" // 0b00110000 // change from interface to gas
	"\n	#define TYPE_GI 0x38" // 0b00111000 // change from gas to interface
	"\n	#define TYPE_SU 0x38" // 0b00111000 // any flag bit used for SURFACE
	"\n	#define TYPE_XY 0xC0" // 0b11000000 // any flag bit used for X or Y markers

#if defined(FP16S)
	"\n	#define fpxx half" // switchable data type (scaled IEEE-754 16-bit floating-point format: 1-5-10, exp-30, +-1.99902344, +-1.86446416E-9, +-1.81898936E-12, 3.311 digits)
	"\n	#define fpxx_copy ushort" // switchable data type for direct copying (scaled IEEE-754 16-bit floating-point format: 1-5-10, exp-30, +-1.99902344, +-1.86446416E-9, +-1.81898936E-12, 3.311 digits)
	"\n	#define load(p,o) (vload_half(o,p)*3.0517578E-5f)" // special function for loading half
	"\n	#define store(p,o,x) vstore_half_rte((x)*32768.0f,o,p)" // special function for storing half
#elif defined(FP16C)
	"\n	#define fpxx ushort" // switchable data type (custom 16-bit floating-point format: 1-4-11, exp-15, +-1.99951168, +-6.10351562E-5, +-2.98023224E-8, 3.612 digits), 12.5% slower than IEEE-754 16-bit
	"\n	#define fpxx_copy ushort" // switchable data type for direct copying (custom 16-bit floating-point format: 1-4-11, exp-15, +-1.99951168, +-6.10351562E-5, +-2.98023224E-8, 3.612 digits), 12.5% slower than IEEE-754 16-bit
	"\n	#define load(p,o) half_to_float_custom((p)[o])" // special function for loading half
	"\n	#define store(p,o,x) (p)[o]=float_to_half_custom(x)" // special function for storing half
#else // FP32
	"\n	#define fpxx float" // switchable data type (regular 32-bit float)
	"\n	#define fpxx_copy float" // switchable data type for direct copying (regular 32-bit float)
	"\n	#define load(p,o) (p)[o]" // regular float read
	"\n	#define store(p,o,x) (p)[o]=(x)" // regular float write
#endif // FP32

#ifdef UPDATE_FIELDS
	"\n	#define UPDATE_FIELDS"
#endif // UPDATE_FIELDS

#ifdef VOLUME_FORCE
	"\n	#define VOLUME_FORCE"
#endif // VOLUME_FORCE

#ifdef MOVING_BOUNDARIES
	"\n	#define MOVING_BOUNDARIES"
#endif // MOVING_BOUNDARIES

#ifdef EQUILIBRIUM_BOUNDARIES
	"\n	#define EQUILIBRIUM_BOUNDARIES"
#endif // EQUILIBRIUM_BOUNDARIES

#ifdef FORCE_FIELD
	"\n	#define FORCE_FIELD"
#endif // FORCE_FIELD

#ifdef SURFACE
	"\n	#define SURFACE"
	"\n	#define def_6_sigma "+to_string(6.0f*sigma)+"f" // rho_laplace = 2*o*K, rho = 1-rho_laplace/c^2 = 1-(6*o)*K
#endif // SURFACE

#ifdef TEMPERATURE
	"\n	#define TEMPERATURE"
	"\n	#define def_w_T "+to_string(1.0f/(2.0f*alpha+0.5f))+"f" // wT = dt/tauT = 1/(2*alpha+1/2), alpha = thermal diffusion coefficient
	"\n	#define def_beta "+to_string(beta)+"f" // thermal expansion coefficient
	"\n	#define def_T_avg "+to_string(T_avg)+"f" // average temperature
#endif // TEMPERATURE

#ifdef SUBGRID
	"\n	#define SUBGRID"+(string)"" // Klebefuge: der Block muss als string enden (Audit-Nacharbeit 2 hat die Ternaere hier herausgezogen)
#endif // SUBGRID

#ifdef PARTICLES
	"\n	#define PARTICLES"
	"\n	#define def_particles_N "+to_string(particles_N)+"ul"
	"\n	#define def_particles_rho "+to_string(particles_rho)+"f"
#endif // PARTICLES

	// FORK -- F-Bounding-Box: der Kernel braucht Ursprung, Ausdehnung und Stride der Box.
	// Bei voller Domaene ist def_FBN == def_N und der Index identisch -- bit-identisch zu Upstream.
#ifdef FORCE_FIELD
	+"\n	#define def_FBX0 "+to_string(fbx0)+"u"
	+"\n	#define def_FBY0 "+to_string(fby0)+"u"
	+"\n	#define def_FBZ0 "+to_string(fbz0)+"u"
	+"\n	#define def_FBNX "+to_string(fbnx)+"u"
	+"\n	#define def_FBNY "+to_string(fbny)+"u"
	+"\n	#define def_FBNZ "+to_string(fbnz)+"u"
	+"\n	#define def_FBN "+to_string((ulong)fbnx*(ulong)fbny*(ulong)fbnz)+"ul"
#ifndef PARTICLES
	// F-Null-Read-Gate: Default AN. PARTICLES-Guard hart im Praeprozessor -- spread_force
	// schriebe F an Fluidzellen, das Gate waere still falsch (Wirkpfad-Absicherung Bein 2).
	+(f_nur_solid_an() ? (string)"\n	#define F_NUR_SOLID" : (string)"")
#endif // PARTICLES
#endif // FORCE_FIELD

	// FORK -- Block-Tiling. index_f() wird per Makro auf index_f_impl(..., tile_slot) umgeschrieben, damit
	// alle Aufrufstellen unveraendert bleiben; nur die Signaturen bekommen tile_slot ueber TS_P.
	// AUS = beide Makros leer = der erzeugte Device-Code ist bit-identisch zu Upstream.
	+(sparse_on ? (string)(
		"\n	#define SPARSE_TILES"
		"\n	#define def_TILE "+to_string(sparse_T)+"u"
		"\n	#define def_TILES_X "+to_string(((uint)get_Nx()+sparse_T-1u)/sparse_T)+"u"
		"\n	#define def_TILES_Y "+to_string(((uint)get_Ny()+sparse_T-1u)/sparse_T)+"u"
		"\n	#define def_TILE_DEAD 4294967295u"
		"\n	#define TS_P , const global uint* tile_slot"
		"\n	#define TS_A , tile_slot"
		"\n	#define index_f(n, i) index_f_impl((n), (i), tile_slot)"
	) : (string)(
		"\n	#define TS_P"
		"\n	#define TS_A"
	))
;}

#ifdef GRAPHICS
void LBM_Domain::Graphics::allocate(Device& device) {
	bitmap = Memory<int>(device, camera.width*camera.height);
	zbuffer = Memory<int>(device, camera.width*camera.height, 1u, lbm->get_D()>1u); // if there are multiple domains, allocate zbuffer also on host side
	camera_parameters = Memory<float>(device, 15u);
	kernel_clear = Kernel(device, bitmap.length(), "graphics_clear", bitmap, zbuffer);
	kernel_graphics_flags = Kernel(device, lbm->get_N(), "graphics_flags", camera_parameters, bitmap, zbuffer, lbm->flags);
	{
#ifndef FORCE_FIELD
		const uint cache_required = (cb(GRAPHICS_LSF+1u)* 1u+1023u)/1024u; // in KB
#else // FORCE_FIELD
		const uint cache_required = (cb(GRAPHICS_LSF+1u)*13u+1023u)/1024u; // in KB
#endif // FORCE_FIELD
		const bool enable_ls = GRAPHICS_LSF>0u&&device.info.max_workgroup_size>=cb(GRAPHICS_LSF)&&device.info.local_cache>=cache_required;
		if(GRAPHICS_LSF>0u&&!enable_ls) print_warning(device.info.name+" does not support local memory optimization with GRAPHICS_LSF = "+to_string(GRAPHICS_LSF)+" (max supported workgroup size: "+to_string(device.info.max_workgroup_size)+" (required: "+to_string(cb(GRAPHICS_LSF))+"), cache: "+to_string(device.info.local_cache)+"KB (required: "+to_string(cache_required)+"KB)). Disabling local memory optimization.");
		const ulong N = enable_ls ? (ulong)((lbm->get_Nx()+GRAPHICS_LSF-2u)/GRAPHICS_LSF)*(ulong)((lbm->get_Ny()+GRAPHICS_LSF-2u)/GRAPHICS_LSF)*(ulong)((lbm->get_Nz()+GRAPHICS_LSF-2u)/GRAPHICS_LSF)*(ulong)cb(GRAPHICS_LSF) : (ulong)(lbm->get_Nx()-1u)*(ulong)(lbm->get_Ny()-1u)*(ulong)(lbm->get_Nz()-1u);
		const uint workgroup_size = enable_ls ? cb(GRAPHICS_LSF) : WORKGROUP_SIZE;
		kernel_graphics_flags_mc = Kernel(device, N, workgroup_size, "graphics_flags_mc", camera_parameters, bitmap, zbuffer, lbm->flags);
	}
	kernel_graphics_field = Kernel(device, lbm->get_D()==1u ? camera.width*camera.height : lbm->get_N(), lbm->get_D()==1u ? "graphics_field_rt" : "graphics_field", camera_parameters, bitmap, zbuffer, 0, lbm->rho, lbm->u, lbm->flags); // raytraced field visualization only works for single-GPU
	kernel_graphics_field_slice = Kernel(device, lbm->get_N(), "graphics_field_slice", camera_parameters, bitmap, zbuffer, 0, 0, 0, 0, 0, lbm->rho, lbm->u, lbm->flags);
#ifndef D2Q9
	kernel_graphics_streamline = Kernel(device, (lbm->get_Nx()/GRAPHICS_STREAMLINE_SPARSE)*(lbm->get_Ny()/GRAPHICS_STREAMLINE_SPARSE)*(lbm->get_Nz()/GRAPHICS_STREAMLINE_SPARSE), "graphics_streamline", camera_parameters, bitmap, zbuffer, 0, 0, 0, 0, 0, lbm->rho, lbm->u, lbm->flags); // 3D
#else // D2Q9
	kernel_graphics_streamline = Kernel(device, (lbm->get_Nx()/GRAPHICS_STREAMLINE_SPARSE)*(lbm->get_Ny()/GRAPHICS_STREAMLINE_SPARSE), "graphics_streamline", camera_parameters, bitmap, zbuffer, 0, 0, 0, 0, 0, lbm->rho, lbm->u, lbm->flags); // 2D
#endif // D2Q9
	{
		const uint cache_required = (cb(GRAPHICS_LSQ+3u)*12u+1023u)/1024u; // in KB
		const bool enable_ls = GRAPHICS_LSQ>0u&&device.info.max_workgroup_size>=cb(GRAPHICS_LSQ)&&device.info.local_cache>=cache_required;
		if(GRAPHICS_LSQ>0u&&!enable_ls) print_warning(device.info.name+" does not support local memory optimization with GRAPHICS_LSQ = "+to_string(GRAPHICS_LSQ)+" (max supported workgroup size: "+to_string(device.info.max_workgroup_size)+" (required: "+to_string(cb(GRAPHICS_LSQ))+"), cache: "+to_string(device.info.local_cache)+"KB (required: "+to_string(cache_required)+"KB)). Disabling local memory optimization.");
		const ulong N = enable_ls ? (ulong)((lbm->get_Nx()+GRAPHICS_LSQ-2u)/GRAPHICS_LSQ)*(ulong)((lbm->get_Ny()+GRAPHICS_LSQ-2u)/GRAPHICS_LSQ)*(ulong)((lbm->get_Nz()+GRAPHICS_LSQ-2u)/GRAPHICS_LSQ)*(ulong)cb(GRAPHICS_LSQ) : (ulong)(lbm->get_Nx()-1u)*(ulong)(lbm->get_Ny()-1u)*(ulong)(lbm->get_Nz()-1u);
		const uint workgroup_size = enable_ls ? cb(GRAPHICS_LSQ) : WORKGROUP_SIZE;
		kernel_graphics_q = Kernel(device, N, workgroup_size, "graphics_q", camera_parameters, bitmap, zbuffer, 0, lbm->rho, lbm->u);
	}

#ifdef FORCE_FIELD
	kernel_graphics_flags.add_parameters(lbm->F);
	kernel_graphics_flags_mc.add_parameters(lbm->F);
#endif // FORCE_FIELD

#ifdef SURFACE
	skybox = Memory<int>(device, skybox_image->width()*skybox_image->height(), 1u, skybox_image->data());
	{
		const uint cache_required = (cb(GRAPHICS_LSP+1u)*4u+1023u)/1024u; // in KB
		const bool enable_ls = GRAPHICS_LSP>0u&&device.info.max_workgroup_size>=cb(GRAPHICS_LSP)&&device.info.local_cache>=cache_required;
		if(GRAPHICS_LSP>0u&&!enable_ls) print_warning(device.info.name+" does not support local memory optimization with GRAPHICS_LSP = "+to_string(GRAPHICS_LSP)+" (max supported workgroup size: "+to_string(device.info.max_workgroup_size)+" (required: "+to_string(cb(GRAPHICS_LSP))+"), cache: "+to_string(device.info.local_cache)+"KB (required: "+to_string(cache_required)+"KB)). Disabling local memory optimization.");
		const ulong N = enable_ls ? (ulong)((lbm->get_Nx()+GRAPHICS_LSP-2u)/GRAPHICS_LSP)*(ulong)((lbm->get_Ny()+GRAPHICS_LSP-2u)/GRAPHICS_LSP)*(ulong)((lbm->get_Nz()+GRAPHICS_LSP-2u)/GRAPHICS_LSP)*(ulong)cb(GRAPHICS_LSP) : (ulong)(lbm->get_Nx()-1u)*(ulong)(lbm->get_Ny()-1u)*(ulong)(lbm->get_Nz()-1u);
		const uint workgroup_size = enable_ls ? cb(GRAPHICS_LSP) : WORKGROUP_SIZE;
		kernel_graphics_rasterize_phi = Kernel(device, N, workgroup_size, "graphics_rasterize_phi", camera_parameters, bitmap, zbuffer, lbm->phi);
	}
	kernel_graphics_raytrace_phi = Kernel(device, bitmap.length(), "graphics_raytrace_phi", camera_parameters, bitmap, skybox, lbm->phi, lbm->flags);
	kernel_graphics_q.add_parameters(lbm->flags);
#endif // SURFACE

#ifdef TEMPERATURE
	kernel_graphics_field.add_parameters(lbm->T);
	kernel_graphics_field_slice.add_parameters(lbm->T);
	kernel_graphics_streamline.add_parameters(lbm->T);
	kernel_graphics_q.add_parameters(lbm->T);
#endif // TEMPERATURE

#ifdef PARTICLES
	kernel_graphics_particles = Kernel(device, lbm->particles.length(), "graphics_particles", camera_parameters, bitmap, zbuffer, lbm->particles);
#endif // PARTICLES
}

bool LBM_Domain::Graphics::update_camera() {
	camera.update_matrix();
	bool change = false;
	for(uint i=0u; i<15u; i++) {
		const float data = camera.data(i);
		change |= (camera_parameters[i]!=data);
		camera_parameters[i] = data;
	}
	return change; // return false if camera parameters remain unchanged
}
bool LBM_Domain::Graphics::enqueue_draw_frame(const int visualization_modes, const int field_mode, const int slice_mode, const int slice_x, const int slice_y, const int slice_z, const bool visualization_change) {
	const bool camera_update = update_camera();
#if defined(INTERACTIVE_GRAPHICS)||defined(INTERACTIVE_GRAPHICS_ASCII)
	if(!visualization_change&&!camera_update&&lbm->get_t()==t_last_rendered_frame) return false; // don't render a new frame if the scene hasn't changed since last frame
#endif // INTERACTIVE_GRAPHICS||INTERACTIVE_GRAPHICS_ASCII
	t_last_rendered_frame = lbm->get_t();
	if(camera_update) camera_parameters.enqueue_write_to_device(); // camera_parameters PCIe transfer and kernel_clear execution can happen simulataneously
	kernel_clear.enqueue_run();
	const int sx=slice_x-lbm->Ox, sy=slice_y-lbm->Oy, sz=slice_z-lbm->Oz; // subtract domain offsets
#ifdef SURFACE
	if((visualization_modes&VIS_PHI_RAYTRACE)&&lbm->get_D()==1u) kernel_graphics_raytrace_phi.enqueue_run(); // disable raytracing for multi-GPU (domain decomposition rendering doesn't work for raytracing)
	if(visualization_modes&VIS_PHI_RASTERIZE) kernel_graphics_rasterize_phi.enqueue_run();
#endif // SURFACE
	if(visualization_modes&VIS_FLAG_LATTICE) kernel_graphics_flags.enqueue_run();
	if(visualization_modes&VIS_FLAG_SURFACE) kernel_graphics_flags_mc.enqueue_run();
	if(visualization_modes&VIS_STREAMLINES) kernel_graphics_streamline.set_parameters(3u, field_mode, slice_mode, sx, sy, sz).enqueue_run();
	if(visualization_modes&VIS_Q_CRITERION) kernel_graphics_q.set_parameters(3u, field_mode).enqueue_run();
#ifdef PARTICLES
	if(visualization_modes&VIS_PARTICLES) kernel_graphics_particles.enqueue_run();
#endif // PARTICLES
	if(visualization_modes&VIS_FIELD) {
		switch(slice_mode) { // 0 (no slice), 1 (x), 2 (y), 3 (z), 4 (xz), 5 (xyz), 6 (yz), 7 (xy)
			case 0: // no slice
				kernel_graphics_field.set_parameters(3u, field_mode).enqueue_run();
				break;
			case 1: case 2: case 3: // x/y/z
				kernel_graphics_field_slice.set_ranges(lbm->get_area((uint)clamp(slice_mode-1, 0, 2))).set_parameters(3u, field_mode, slice_mode, sx, sy, sz).enqueue_run();
				break;
			case 4: // xz
				kernel_graphics_field_slice.set_ranges(lbm->get_area(0u)).set_parameters(3u, field_mode, 0u+1u, sx, sy, sz).enqueue_run();
				kernel_graphics_field_slice.set_ranges(lbm->get_area(2u)).set_parameters(3u, field_mode, 2u+1u, sx, sy, sz).enqueue_run();
				break;
			case 5: // xyz
				kernel_graphics_field_slice.set_ranges(lbm->get_area(0u)).set_parameters(3u, field_mode, 0u+1u, sx, sy, sz).enqueue_run();
				kernel_graphics_field_slice.set_ranges(lbm->get_area(1u)).set_parameters(3u, field_mode, 1u+1u, sx, sy, sz).enqueue_run();
				kernel_graphics_field_slice.set_ranges(lbm->get_area(2u)).set_parameters(3u, field_mode, 2u+1u, sx, sy, sz).enqueue_run();
				break;
			case 6: // yz
				kernel_graphics_field_slice.set_ranges(lbm->get_area(1u)).set_parameters(3u, field_mode, 1u+1u, sx, sy, sz).enqueue_run();
				kernel_graphics_field_slice.set_ranges(lbm->get_area(2u)).set_parameters(3u, field_mode, 2u+1u, sx, sy, sz).enqueue_run();
				break;
			case 7: // xy
				kernel_graphics_field_slice.set_ranges(lbm->get_area(0u)).set_parameters(3u, field_mode, 0u+1u, sx, sy, sz).enqueue_run();
				kernel_graphics_field_slice.set_ranges(lbm->get_area(1u)).set_parameters(3u, field_mode, 1u+1u, sx, sy, sz).enqueue_run();
				break;
		}
	}
	bitmap.enqueue_read_from_device();
	if(lbm->get_D()>1u) zbuffer.enqueue_read_from_device();
	return true; // new frame has been rendered
}
int* LBM_Domain::Graphics::get_bitmap() { // returns pointer to zbuffer
	return bitmap.data();
}
int* LBM_Domain::Graphics::get_zbuffer() { // returns pointer to zbuffer
	return zbuffer.data();
}

string LBM_Domain::Graphics::device_defines(const Device_Info& device_info) const { return
	"\n	#define GRAPHICS"
	"\n	#define def_background_color " +to_string(GRAPHICS_BACKGROUND_COLOR)+""
	"\n	#define def_screen_width "     +to_string(camera.width)+"u"
	"\n	#define def_screen_height "    +to_string(camera.height)+"u"
	"\n	#define def_scale_u "          +to_string(1.0f/(0.57735027f*(GRAPHICS_U_MAX)))+"f"
	"\n	#define def_scale_rho "        +to_string(0.5f/(GRAPHICS_RHO_DELTA))+"f"
	"\n	#define def_scale_T "          +to_string(0.5f/(GRAPHICS_T_DELTA))+"f"
	"\n	#define def_scale_F "          +to_string(0.5f/(GRAPHICS_F_MAX))+"f"
	"\n	#define def_scale_Q_min "      +to_string(GRAPHICS_Q_CRITERION)+"f"
	"\n	#define def_streamline_sparse "+to_string(GRAPHICS_STREAMLINE_SPARSE)+"u"
	"\n	#define def_streamline_length "+to_string(GRAPHICS_STREAMLINE_LENGTH)+"u"
	"\n	#define def_n "                +to_string(1.333f)+"f" // refractive index of water for raytracing graphics
	"\n	#define def_attenuation "      +to_string(ln(clamp(GRAPHICS_RAYTRACING_TRANSMITTANCE, 1E-9f, 1.0f))/(float)max(max(lbm->get_Nx(), lbm->get_Ny()), lbm->get_Nz()))+"f" // (negative) attenuation parameter for raytracing graphics
	"\n	#define def_absorption_color " +to_string(GRAPHICS_RAYTRACING_COLOR)+"" // absorption color of fluid for raytracing graphics

	"\n	#define COLOR_S (127<<16|127<<8|127)" // (stationary or moving) solid boundary
	"\n	#define COLOR_E (  0<<16|255<<8|  0)" // equilibrium boundary (inflow/outflow)
	"\n	#define COLOR_M (255<<16|  0<<8|255)" // cells next to moving solid boundary
	"\n	#define COLOR_T (255<<16|  0<<8|  0)" // temperature boundary
	"\n	#define COLOR_F (  0<<16|  0<<8|255)" // fluid
	"\n	#define COLOR_I (  0<<16|255<<8|255)" // interface
	"\n	#define COLOR_0 (127<<16|127<<8|127)" // regular cell or gas
	"\n	#define COLOR_X (255<<16|127<<8|  0)" // reserved type X
	"\n	#define COLOR_Y (255<<16|255<<8|  0)" // reserved type Y
	"\n	#define COLOR_P (255<<16|255<<8|191)" // particles

#ifdef GRAPHICS_TRANSPARENCY
	"\n	#define GRAPHICS_TRANSPARENCY "+to_string(GRAPHICS_TRANSPARENCY)+"f"
#endif // GRAPHICS_TRANSPARENCY

#ifndef SURFACE
	"\n	#define def_skybox_width 1u"
	"\n	#define def_skybox_height 1u"
#else // SURFACE
	"\n	#define def_skybox_width " +to_string(skybox_image->width() )+"u"
	"\n	#define def_skybox_height "+to_string(skybox_image->height())+"u"
#endif // SURFACE

#ifndef FORCE_FIELD
	"\n	#define LSF "+to_string((GRAPHICS_LSF>0u&&device_info.max_workgroup_size>=cb(GRAPHICS_LSF)&&device_info.local_cache>=(cb(GRAPHICS_LSF+1u)* 1u+1023u)/1024u) ? GRAPHICS_LSF : 0u)+"u" // local box size for graphics_flags_mc() kernel (default: 4)
#else // FORCE_FIELD
	"\n	#define LSF "+to_string((GRAPHICS_LSF>0u&&device_info.max_workgroup_size>=cb(GRAPHICS_LSF)&&device_info.local_cache>=(cb(GRAPHICS_LSF+1u)*13u+1023u)/1024u) ? GRAPHICS_LSF : 0u)+"u" // local box size for graphics_flags_mc() kernel (default: 4)
#endif // FORCE_FIELD
	"\n	#define LSQ "+to_string((GRAPHICS_LSQ>0u&&device_info.max_workgroup_size>=cb(GRAPHICS_LSQ)&&device_info.local_cache>=(cb(GRAPHICS_LSQ+3u)*12u+1023u)/1024u) ? GRAPHICS_LSQ : 0u)+"u" // local box size for graphics_q() kernel (default: 8)
	"\n	#define LSP "+to_string((GRAPHICS_LSP>0u&&device_info.max_workgroup_size>=cb(GRAPHICS_LSP)&&device_info.local_cache>=(cb(GRAPHICS_LSP+1u)* 4u+1023u)/1024u) ? GRAPHICS_LSP : 0u)+"u" // local box size for graphics_rasterize_phi() kernel (default: 4)
;}
#endif // GRAPHICS



vector<Device_Info> smart_device_selection(const uint D) {
	const vector<Device_Info>& devices = get_devices(); // a vector of all available OpenCL devices
	vector<Device_Info> device_infos(D);
	const int user_specified_devices = (int)main_arguments.size();
	if(user_specified_devices>0) { // user has selevted specific devices as command line arguments
		if(user_specified_devices==D) { // as much specified devices as domains
			for(uint d=0; d<D; d++) device_infos[d] = select_device_with_id(to_uint(main_arguments[d]), devices); // use list of devices IDs specified by user
		} else {
			print_warning("Incorrect number of devices specified. Using single fastest device for all domains.");
			for(uint d=0; d<D; d++) device_infos[d] = select_device_with_most_flops(devices);
		}
	} else { // device auto-selection
		vector<vector<Device_Info>> device_type_ids; // a vector of all different devices, containing vectors of their device IDs
		for(uint i=0u; i<(uint)devices.size(); i++) {
			const string name_i = devices[i].name;
			bool already_exists = false;
			for(uint j=0u; j<(uint)device_type_ids.size(); j++) {
				const string name_j = device_type_ids[j][0].name;
				if(name_i==name_j) {
					device_type_ids[j].push_back(devices[i]);
					already_exists = true;
				}
			}
			if(!already_exists) device_type_ids.push_back(vector<Device_Info>(1, devices[i]));
		}
		float best_value = -1.0f;
		int best_j = -1;
		for(uint j=0u; j<(uint)device_type_ids.size(); j++) {
			const float value = device_type_ids[j][0].tflops;
			if((uint)device_type_ids[j].size()>=D && value>best_value) {
				best_value = value;
				best_j = j;
			}
		}
		if(best_j>=0) { // select all devices of fastest device type with at least D devices of the same type
			for(uint d=0; d<D; d++) device_infos[d] = device_type_ids[best_j][d];
		} else {
			print_warning("Not enough devices of the same type available. Using single fastest device for all domains.");
			for(uint d=0; d<D; d++) device_infos[d] = select_device_with_most_flops(devices);
		}
		//for(uint j=0u; j<(uint)device_type_ids.size(); j++) print_info("Device Type "+to_string(j)+" ("+device_type_ids[j][0].name+"): "+to_string((uint)device_type_ids[j].size())+"x");
	}
	return device_infos;
}

LBM::LBM(const uint Nx, const uint Ny, const uint Nz, const float nu, const float fx, const float fy, const float fz, const float sigma, const float alpha, const float beta, const uint particles_N, const float particles_rho) // single device
	:LBM(Nx, Ny, Nz, 1u, 1u, 1u, nu, fx, fy, fz, sigma, alpha, beta, particles_N, particles_rho) { // delegating constructor
}
LBM::LBM(const uint Nx, const uint Ny, const uint Nz, const float nu, const float fx, const float fy, const float fz, const uint particles_N, const float particles_rho)
	:LBM(Nx, Ny, Nz, 1u, 1u, 1u, nu, fx, fy, fz, 0.0f, 0.0f, 0.0f, particles_N, particles_rho) { // delegating constructor
}
LBM::LBM(const uint Nx, const uint Ny, const uint Nz, const float nu, const uint particles_N, const float particles_rho)
	:LBM(Nx, Ny, Nz, 1u, 1u, 1u, nu, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, particles_N, particles_rho) { // delegating constructor
}
LBM::LBM(const uint3 N, const uint Dx, const uint Dy, const uint Dz, const float nu, const float fx, const float fy, const float fz, const float sigma, const float alpha, const float beta, const uint particles_N, const float particles_rho)
	:LBM(N.x, N.y, N.z, Dx, Dy, Dz, nu, fx, fy, fz, sigma, alpha, beta, particles_N, particles_rho) { // delegating constructor
}
LBM::LBM(const uint3 N, const float nu, const float fx, const float fy, const float fz, const float sigma, const float alpha, const float beta, const uint particles_N, const float particles_rho) // single device
	:LBM(N.x, N.y, N.z, 1u, 1u, 1u, nu, fx, fy, fz, sigma, alpha, beta, particles_N, particles_rho) { // delegating constructor
}
LBM::LBM(const uint3 N, const float nu, const uint particles_N, const float particles_rho)
	:LBM(N.x, N.y, N.z, 1u, 1u, 1u, nu, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, particles_N, particles_rho) { // delegating constructor
}
LBM::LBM(const uint3 N, const float nu, const float fx, const float fy, const float fz, const uint particles_N, const float particles_rho)
	:LBM(N.x, N.y, N.z, 1u, 1u, 1u, nu, fx, fy, fz, 0.0f, 0.0f, 0.0f, particles_N, particles_rho) { // delegating constructor
}
LBM::LBM(const uint Nx, const uint Ny, const uint Nz, const uint Dx, const uint Dy, const uint Dz, const float nu, const float fx, const float fy, const float fz, const float sigma, const float alpha, const float beta, const uint particles_N, const float particles_rho) { // multiple devices
	const uint NDx=(Nx/Dx)*Dx, NDy=(Ny/Dy)*Dy, NDz=(Nz/Dz)*Dz; // make resolution equally divisible by domains
	if(NDx!=Nx||NDy!=Ny||NDz!=Nz) print_warning("LBM grid ("+to_string(Nx)+"x"+to_string(Ny)+"x"+to_string(Nz)+") is not equally divisible in domains ("+to_string(Dx)+"x"+to_string(Dy)+"x"+to_string(Dz)+"). Changing resolution to ("+to_string(NDx)+"x"+to_string(NDy)+"x"+to_string(NDz)+").");
	this->Nx = NDx; this->Ny = NDy; this->Nz = NDz;
	this->Dx = Dx; this->Dy = Dy; this->Dz = Dz;
	const uint D = Dx*Dy*Dz;
	const uint Hx=Dx>1u, Hy=Dy>1u, Hz=Dz>1u; // halo offsets
	const vector<Device_Info>& device_infos = smart_device_selection(D);
	sanity_checks_constructor(device_infos, this->Nx, this->Ny, this->Nz, Dx, Dy, Dz, nu, fx, fy, fz, sigma, alpha, beta, particles_N, particles_rho);
	lbm_domain = new LBM_Domain*[D];
	for(uint d=0u; d<D; d++) { // parallel_for((ulong)D, D, [&](ulong d) {
		const uint x=((uint)d%(Dx*Dy))%Dx, y=((uint)d%(Dx*Dy))/Dx, z=(uint)d/(Dx*Dy); // d = x+(y+z*Dy)*Dx
		lbm_domain[d] = new LBM_Domain(device_infos[d], this->Nx/Dx+2u*Hx, this->Ny/Dy+2u*Hy, this->Nz/Dz+2u*Hz, Dx, Dy, Dz, (int)(x*this->Nx/Dx)-(int)Hx, (int)(y*this->Ny/Dy)-(int)Hy, (int)(z*this->Nz/Dz)-(int)Hz, nu, fx, fy, fz, sigma, alpha, beta, particles_N, particles_rho);
	} // });
	{
		Memory<float>** buffers_rho = new Memory<float>*[D];
		for(uint d=0u; d<D; d++) buffers_rho[d] = &(lbm_domain[d]->rho);
		rho = Memory_Container(this, buffers_rho, "rho");
	} {
		Memory<float>** buffers_u = new Memory<float>*[D];
		for(uint d=0u; d<D; d++) buffers_u[d] = &(lbm_domain[d]->u);
		u = Memory_Container(this, buffers_u, "u");
	} {
		Memory<uchar>** buffers_flags = new Memory<uchar>*[D];
		for(uint d=0u; d<D; d++) buffers_flags[d] = &(lbm_domain[d]->flags);
		flags = Memory_Container(this, buffers_flags, "flags");
	} {
#ifdef FORCE_FIELD
		Memory<float>** buffers_F = new Memory<float>*[D];
		for(uint d=0u; d<D; d++) buffers_F[d] = &(lbm_domain[d]->F);
		F = Memory_Container(this, buffers_F, "F");
#endif // FORCE_FIELD
	} {
#ifdef SURFACE
		Memory<float>** buffers_phi = new Memory<float>*[D];
		for(uint d=0u; d<D; d++) buffers_phi[d] = &(lbm_domain[d]->phi);
		phi = Memory_Container(this, buffers_phi, "phi");
#endif // SURFACE
	} {
#ifdef TEMPERATURE
		Memory<float>** buffers_T = new Memory<float>*[D];
		for(uint d=0u; d<D; d++) buffers_T[d] = &(lbm_domain[d]->T);
		T = Memory_Container(this, buffers_T, "T");
#endif // TEMPERATURE
	} {
#ifdef PARTICLES
		particles = &(lbm_domain[0]->particles);
#endif // PARTICLES
	}
#ifdef GRAPHICS
	graphics = Graphics(this);
#endif // GRAPHICS
}
// FORK Doppel-Domaene: Ein-Geraete-Konstruktor mit EXPLIZITEM Device_Info.
// Der Standardweg (smart_device_selection) liefert immer das schnellste Geraet; fuer die gekoppelte
// Rechnung brauchen wir zwei LBM-Instanzen auf zwei verschiedenen GPUs. Sonst identisch zum
// Ein-Geraete-Pfad oben (D=1, keine Halos, kein Offset).
LBM::LBM(const uint3 N, const float nu, const Device_Info& device_info, const float fx, const float fy, const float fz, const float sigma, const float alpha, const float beta, const uint particles_N, const float particles_rho) {
	this->Nx = N.x; this->Ny = N.y; this->Nz = N.z;
	this->Dx = 1u; this->Dy = 1u; this->Dz = 1u;
	const vector<Device_Info> device_infos(1u, device_info);
	sanity_checks_constructor(device_infos, this->Nx, this->Ny, this->Nz, 1u, 1u, 1u, nu, fx, fy, fz, sigma, alpha, beta, particles_N, particles_rho);
	lbm_domain = new LBM_Domain*[1u];
	lbm_domain[0] = new LBM_Domain(device_info, this->Nx, this->Ny, this->Nz, 1u, 1u, 1u, 0, 0, 0, nu, fx, fy, fz, sigma, alpha, beta, particles_N, particles_rho);
	{
		Memory<float>** buffers_rho = new Memory<float>*[1u];
		buffers_rho[0] = &(lbm_domain[0]->rho);
		rho = Memory_Container(this, buffers_rho, "rho");
	} {
		Memory<float>** buffers_u = new Memory<float>*[1u];
		buffers_u[0] = &(lbm_domain[0]->u);
		u = Memory_Container(this, buffers_u, "u");
	} {
		Memory<uchar>** buffers_flags = new Memory<uchar>*[1u];
		buffers_flags[0] = &(lbm_domain[0]->flags);
		flags = Memory_Container(this, buffers_flags, "flags");
	}
#ifdef FORCE_FIELD
	{
		Memory<float>** buffers_F = new Memory<float>*[1u];
		buffers_F[0] = &(lbm_domain[0]->F);
		F = Memory_Container(this, buffers_F, "F");
	}
#endif // FORCE_FIELD
#ifdef SURFACE
	{
		Memory<float>** buffers_phi = new Memory<float>*[1u];
		buffers_phi[0] = &(lbm_domain[0]->phi);
		phi = Memory_Container(this, buffers_phi, "phi");
	}
#endif // SURFACE
#ifdef TEMPERATURE
	{
		Memory<float>** buffers_T = new Memory<float>*[1u];
		buffers_T[0] = &(lbm_domain[0]->T);
		T = Memory_Container(this, buffers_T, "T");
	}
#endif // TEMPERATURE
#ifdef PARTICLES
	particles = &(lbm_domain[0]->particles);
#endif // PARTICLES
#ifdef GRAPHICS
	graphics = Graphics(this);
#endif // GRAPHICS
}
LBM::~LBM() {
#ifdef GRAPHICS
	camera.allow_rendering = false;
#endif // GRAPHICS
	info.print_finalize();
	for(uint d=0u; d<get_D(); d++) delete lbm_domain[d];
	delete[] lbm_domain;
}

void LBM::sanity_checks_constructor(const vector<Device_Info>& device_infos, const uint Nx, const uint Ny, const uint Nz, const uint Dx, const uint Dy, const uint Dz, const float nu, const float fx, const float fy, const float fz, const float sigma, const float alpha, const float beta, const uint particles_N, const float particles_rho) { // sanity checks on grid resolution and extension support
	if((ulong)Nx*(ulong)Ny*(ulong)Nz==0ull) print_error("Grid point number is 0: "+to_string(Nx)+"x"+to_string(Ny)+"x"+to_string(Nz)+" = 0.");
	if(Dx*Dy*Dz==0u) print_error("You specified 0 LBM grid domains ("+to_string(Dx)+"x"+to_string(Dy)+"x"+to_string(Dz)+"). There has to be at least 1 domain in every direction. Check your input in LBM constructor.");
	const uint local_Nx=Nx/Dx+2u*(Dx>1u), local_Ny=Ny/Dy+2u*(Dy>1u), local_Nz=Nz/Dz+2u*(Dz>1u);
	uint memory_available = max_uint; // in MB
	for(Device_Info device_info : device_infos) memory_available = min(memory_available, device_info.memory);
	// ★ FORK 2026-08-29 (Variante C+D1+D2 nach Planungsschritt). Die Vorpruefung war in BEIDE
	// Richtungen falsch und lehnte deshalb ein Gitter ab, das real passt:
	//   ZU PESSIMISTISCH: sie rechnet FORCE_FIELD mit 12 B/Zelle ueber das VOLLE Gitter, obwohl
	//     allocate() F laengst nur ueber die Bounding-Box anlegt (lbm.cpp:344-347). Beim
	//     4-mm-Fahrzeug sind das 4.109 MB zuviel (Lauflog: "F-BBox: F auf 1118x468x306 statt
	//     508701465 Zellen -> 4.18 GB gespart").
	//   ZU OPTIMISTISCH: sie kennt fac_idx (4 B je F-BBox-Zelle, lbm.cpp:513) gar nicht -- 611 MB.
	// F_N ist hier bereits bekannt: setup.cpp ruft set_force_bbox VOR dem Konstruktor, und der
	// Konstruktor ruft diese Pruefung VOR new LBM_Domain (das s_fbbox erst ausliest und nullt).
	// Also DERSELBE Ausdruck wie in allocate(), keine Schaetzung. Bei Dx*Dy*Dz>1 ist F voll-
	// domaenig (lbm.cpp:432 sperrt F-BBox im Mehrgeraetefall), dort bleibt die alte Rechnung.
	const bool fbb_gilt = (Dx*Dy*Dz==1u) && LBM_Domain::s_fbbox[3]>0u && LBM_Domain::s_fbbox[4]>0u && LBM_Domain::s_fbbox[5]>0u;
	const ulong N_dom = (ulong)Nx*(ulong)Ny*(ulong)Nz/((ulong)(Dx*Dy*Dz));
	const ulong F_N   = fbb_gilt ? (ulong)LBM_Domain::s_fbbox[3]*(ulong)LBM_Domain::s_fbbox[4]*(ulong)LBM_Domain::s_fbbox[5] : N_dom;
	const ulong b_zelle = (ulong)bytes_per_cell_device();
#ifdef FORCE_FIELD
	const ulong b_ohne_F = b_zelle-12ull;   // F wird unten mit F_N statt N verrechnet
#else
	const ulong b_ohne_F = b_zelle;
#endif // FORCE_FIELD
	ulong bytes_bekannt = N_dom*b_ohne_F;
#ifdef FORCE_FIELD
	bytes_bekannt += 12ull*F_N;
#endif // FORCE_FIELD
	if(LBM_Domain::s_facetten) bytes_bekannt += 4ull*F_N; // fac_idx (lbm.cpp:513) -- die Pruefung kannte es nicht
	uint memory_required = (uint)(bytes_bekannt/1048576ull); // in MB
	// D1: RESERVE. ★ Pruefagent A-1: die Pruefung sieht `device_info.memory`, also den
	// GESAMTspeicher -- `memory_used` wird hier nicht abgezogen (und Device_Info ist eine Kopie,
	// Belegungen der ersten Domaene erreichen die zweite Pruefung ohnehin nicht). Der freie
	// Speicher entsteht erst durch den Abzug der Reserve unten; teilen sich zwei Domaenen EIN
	// Geraet, schuetzt sie nicht. Heiko-Vorgabe 2026-08-29: 1,0-1,5 GB
	// Restluft sind legitim. Ohne benannte Reserve verschiebt die Korrektur oben den Deckel nur
	// und laesst wieder Nutzlast zu, die es nicht gibt. Drei Posten, jeder belegt:
	//   320 MB  Spaetpuffer, die erst nach dem Konstruktor entstehen (Facettengeometrie, Schale,
	//           Kopplungsebene, kf_liste) -- gemessen am 4-mm-Lauf p4_v3b
	//  1152 MB  DESKTOP: die B70 treibt den Bildschirm (card0-DP-5). memory_used sieht davon
	//           nichts. Der Abbruch vom 22.08.2026 kam genau daher: 29,3 GB Lauf + 1,1 GB Desktop.
	//  1024 MB  Mindestluft nach Heikos Untergrenze
	// Nur fuer echte Geraete -- ein Fall, der im System-RAM rechnet (iGPU-Fernfeld), bekaeme
	// sonst einen Deckel, der mit dem Hostbedarf kollidiert.
	bool nur_ram = true;
	for(Device_Info di : device_infos) nur_ram = nur_ram && di.uses_ram;
	const uint reserve = nur_ram ? 0u : (uint)env_u("CFD_VRAM_RESERVE_MB", 2496u);
	// D2: Speicherplan drucken. Kostet nichts und macht jeden kuenftigen Lauf nachrechenbar --
	// genau das fehlte, als die alte Pruefung ein passendes Gitter ablehnte.
	print_info("SPEICHERPLAN je Domaene: bekannt "+to_string(memory_required)+" MB"
		+(fbb_gilt?string(" (F ueber BBox "+to_string(LBM_Domain::s_fbbox[3])+"x"+to_string(LBM_Domain::s_fbbox[4])+"x"+to_string(LBM_Domain::s_fbbox[5])+", nicht ueber das volle Gitter)"):string(" (F voll-domaenig)"))
		+", Reserve "+to_string(reserve)+" MB (Spaetpuffer+Desktop+Mindestluft, CFD_VRAM_RESERVE_MB)"
		+", verfuegbar "+to_string(memory_available)+" MB, Schlupf "
		+((ulong)memory_required+(ulong)reserve<=(ulong)memory_available ? to_string(memory_available-memory_required-reserve)+" MB" : string("NEGATIV")));
	if((ulong)memory_required+(ulong)reserve>(ulong)memory_available) {
		// ★ Pruefagent A-4: die Reserve ist KONSTANT, sie skaliert nicht mit N^3. Sie gehoert
		// deshalb vom Verfuegbaren abgezogen, nicht zum Bedarf addiert -- sonst schlaegt die
		// Meldung eine zu kleine Aufloesung vor, und genau sie soll zum Skalieren anleiten.
		float factor = cbrt((float)(memory_available>reserve?memory_available-reserve:1u)/(float)memory_required);
		memory_required += reserve; // fuer die Textausgabe: was insgesamt gebraucht wird
		const uint maxNx=(uint)(factor*(float)Nx), maxNy=(uint)(factor*(float)Ny), maxNz=(uint)(factor*(float)Nz);
		string message = "Grid resolution ("+to_string(Nx)+", "+to_string(Ny)+", "+to_string(Nz)+") is too large: "+to_string(Dx*Dy*Dz)+"x "+to_string(memory_required)+" MB required, "+to_string(Dx*Dy*Dz)+"x "+to_string(memory_available)+" MB available. Largest possible resolution is ("+to_string(maxNx)+", "+to_string(maxNy)+", "+to_string(maxNz)+"). Restart the simulation with lower resolution or on different device(s) with more memory.";
#if !defined(FP16S)&&!defined(FP16C)
		uint memory_required_fp16 = (uint)((ulong)Nx*(ulong)Ny*(ulong)Nz/((ulong)(Dx*Dy*Dz))*(ulong)(bytes_per_cell_device()-velocity_set*2u)/1048576ull); // in MB
		float factor_fp16 = cbrt((float)memory_available/(float)memory_required_fp16);
		const uint maxNx_fp16=(uint)(factor_fp16*(float)Nx), maxNy_fp16=(uint)(factor_fp16*(float)Ny), maxNz_fp16=(uint)(factor_fp16*(float)Nz);
		message += " Consider using FP16S/FP16C memory compression to double maximum grid resolution to a maximum of ("+to_string(maxNx_fp16)+", "+to_string(maxNy_fp16)+", "+to_string(maxNz_fp16)+"); for this, uncomment \"#define FP16S\" or \"#define FP16C\" in defines.hpp.";
#endif // !FP16S&&!FP16C
		print_error(message);
	}
	if(nu==0.0f) print_error("Viscosity cannot be 0. Change it in setup.cpp."); // sanity checks for viscosity
	else if(nu<0.0f) print_error("Viscosity cannot be negative. Remove the \"-\" in setup.cpp.");
#ifdef D2Q9
	if(Nz!=1u) print_error("D2Q9 is the 2D velocity set. You have to set Nz=1u in the LBM constructor! Currently you have set Nz="+to_string(Nz)+"u.");
#endif // D2Q9
#if !defined(SRT)&&!defined(TRT)
	print_error("No LBM collision operator selected. Uncomment either \"#define SRT\" or \"#define TRT\" in defines.hpp");
#elif defined(SRT)&&defined(TRT)
	print_error("Too many LBM collision operators selected. Comment out either \"#define SRT\" or \"#define TRT\" in defines.hpp");
#endif // SRT && TRT
// ★ FORK 2026-08-08: dieselbe Absicherung fuer das Zahlenformat, und zwar aus Erfahrung. Am 2026-08-08
// waren FP16S UND FP16C gleichzeitig gesetzt; die #if defined(FP16S) / #elif defined(FP16C)-Ketten in
// device_defines() und info.cpp lassen dann still FP16S gewinnen. Die gesamte Kugelvalidierung lief
// dadurch im falschen Format, ohne eine einzige Meldung. Fuer SRT/TRT gab es diesen Waechter schon --
// fuer FP16 nicht, obwohl der Fehler dort genauso lautlos ist.
#if defined(FP16S)&&defined(FP16C)
	print_error("FP16S und FP16C sind beide gesetzt. Die #if/#elif-Ketten lassen dann still FP16S gewinnen. Genau eines von beiden in defines.hpp auskommentieren.");
#endif // FP16S && FP16C
#ifndef VOLUME_FORCE
	if(fx!=0.0f||fy!=0.0f||fz!=0.0f) print_error("Volume force is set in LBM constructor in main_setup(), but VOLUME_FORCE is not enabled. Uncomment \"#define VOLUME_FORCE\" in defines.hpp.");
#else // VOLUME_FORCE
#ifndef FORCE_FIELD
	if(fx==0.0f&&fy==0.0f&&fz==0.0f) print_warning("The VOLUME_FORCE extension is enabled but the volume force in LBM constructor is set to zero. You may disable the extension by commenting out \"#define VOLUME_FORCE\" in defines.hpp.");
#endif // FORCE_FIELD
#endif // VOLUME_FORCE
#ifndef SURFACE
	if(sigma!=0.0f) print_error("Surface tension is set in LBM constructor in main_setup(), but SURFACE is not enabled. Uncomment \"#define SURFACE\" in defines.hpp.");
#endif // SURFACE
#ifndef TEMPERATURE
	if(alpha!=0.0f||beta!=0.0f) print_error("Thermal diffusion/expansion coefficients are set in LBM constructor in main_setup(), but TEMPERATURE is not enabled. Uncomment \"#define TEMPERATURE\" in defines.hpp.");
#else // TEMPERATURE
	if(alpha==0.0f&&beta==0.0f) print_warning("The TEMPERATURE extension is enabled but the thermal diffusion/expansion coefficients alpha/beta in the LBM constructor are both set to zero. You may disable the extension by commenting out \"#define TEMPERATURE\" in defines.hpp.");
#endif // TEMPERATURE
#ifdef PARTICLES
	if(particles_N==0u) print_error("The PARTICLES extension is enabled but the number of particles is set to 0. Comment out \"#define PARTICLES\" in defines.hpp.");
#if !defined(VOLUME_FORCE)||!defined(FORCE_FIELD)
	if(particles_rho!=1.0f) print_error("Particle density is set unequal to 1, but particle-fluid 2-way-coupling is not enabled. Uncomment both \"#define VOLUME_FORCE\" and \"#define FORCE_FIELD\" in defines.hpp.");
#endif // !VOLUME_FORCE||!FORCE_FIELD
#ifdef FORCE_FIELD
	if(particles_rho==1.0f) print_warning("Particle density is set to 1, so particles behave as passive tracers without acting a force on the fluid, but particle-fluid 2-way-coupling is enabled. You may comment out \"#define FORCE_FIELD\" in defines.hpp.");
#endif // FORCE_FIELD
#else // PARTICLES
	if(particles_N>0u) print_error("The PARTICLES extension is disabled but the number of particles is set to "+to_string(particles_N)+">0. Uncomment \"#define PARTICLES\" in defines.hpp.");
#endif // PARTICLES
}

void LBM::sanity_checks_initialization() { // sanity checks during initialization on used extensions based on used flags
	uchar flags_used = 0u;
	bool moving_boundaries_used=false, equilibrium_boundaries_used=false, surface_used=false, temperature_used=false; // identify used extensions based used flags
	const uint threads = thread::hardware_concurrency();
	vector<uchar> t_flags_used(threads, 0u);
	vector<char> t_moving_boundaries_used(threads, false); // don't use vector<bool> as it uses bit-packing which is broken for multithreading
	vector<char> t_equilibrium_boundaries_used(threads, false); // don't use vector<bool> as it uses bit-packing which is broken for multithreading
	parallel_for(get_N(), threads, [&](ulong n, uint t) {
		const uchar flagsn = flags[n];
		const uchar flagsn_bo = flagsn&(TYPE_S|TYPE_E);
		t_flags_used[t] = t_flags_used[t]|flagsn;
		if(flagsn_bo&TYPE_S) t_moving_boundaries_used[t] = t_moving_boundaries_used[t] || (((flagsn_bo==TYPE_S)&&(u.x[n]!=0.0f||u.y[n]!=0.0f||u.z[n]!=0.0f))||(flagsn_bo==(TYPE_S|TYPE_E)));
		t_equilibrium_boundaries_used[t] = t_equilibrium_boundaries_used[t] || flagsn_bo==TYPE_E;
	});
	for(uint t=0u; t<threads; t++) {
		flags_used = flags_used|t_flags_used[t];
		moving_boundaries_used = moving_boundaries_used || t_moving_boundaries_used[t];
		equilibrium_boundaries_used = equilibrium_boundaries_used || t_equilibrium_boundaries_used[t];
	}
	surface_used = (bool)(flags_used&(TYPE_F|TYPE_I|TYPE_G));
	temperature_used = (bool)(flags_used&TYPE_T);
#ifndef MOVING_BOUNDARIES
	if(moving_boundaries_used) print_warning("Some boundary cells have non-zero velocity, but MOVING_BOUNDARIES is not enabled. If you intend to use moving boundaries, uncomment \"#define MOVING_BOUNDARIES\" in defines.hpp.");
#else // MOVING_BOUNDARIES
	if(!moving_boundaries_used) print_warning("The MOVING_BOUNDARIES extension is enabled but no moving boundary cells (TYPE_S flag and velocity unequal to zero) are placed in the simulation box. You may disable the extension by commenting out \"#define MOVING_BOUNDARIES\" in defines.hpp.");
#endif // MOVING_BOUNDARIES
#ifndef EQUILIBRIUM_BOUNDARIES
	if(equilibrium_boundaries_used) print_error("Some cells are set as equilibrium boundaries with the TYPE_E flag, but EQUILIBRIUM_BOUNDARIES is not enabled. Uncomment \"#define EQUILIBRIUM_BOUNDARIES\" in defines.hpp.");
#else // EQUILIBRIUM_BOUNDARIES
	if(!equilibrium_boundaries_used) print_warning("The EQUILIBRIUM_BOUNDARIES extension is enabled but no equilibrium boundary cells (TYPE_E flag) are placed in the simulation box. You may disable the extension by commenting out \"#define EQUILIBRIUM_BOUNDARIES\" in defines.hpp.");
#endif // EQUILIBRIUM_BOUNDARIES
#ifndef SURFACE
	if(surface_used) print_error("Some cells are set as fluid/interface/gas with the TYPE_F/TYPE_I/TYPE_G flags, but SURFACE is not enabled. Uncomment \"#define SURFACE\" in defines.hpp.");
#else // SURFACE
	if(!surface_used) print_error("The SURFACE extension is enabled but no fluid/interface/gas cells (TYPE_F/TYPE_I/TYPE_G flags) are placed in the simulation box. Disable the extension by commenting out \"#define SURFACE\" in defines.hpp.");
#endif // SURFACE
#ifndef TEMPERATURE
	if(temperature_used) print_error("Some cells are set as temperature boundary with the TYPE_T flag, but TEMPERATURE is not enabled. Uncomment \"#define TEMPERATURE\" in defines.hpp.");
#endif // TEMPERATURE
}

void LBM::initialize() { // write all data fields to device and call kernel_initialize
#ifndef BENCHMARK
	sanity_checks_initialization();
#endif // BENCHMARK

	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->rho.enqueue_write_to_device();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->u.enqueue_write_to_device();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->flags.enqueue_write_to_device();
#ifdef FORCE_FIELD
#ifndef PARTICLES
	// ★ F-Waechter (Wirkpfad-Absicherung des F-Null-Read-Gates, Bein 1): das Gate behauptet
	// "F ist an Nicht-Solid-Zellen 0" -- hier wird die Behauptung am Host-Puffer HART geprueft,
	// bevor F aufs Geraet geht. Faengt den einzigen realistischen kuenftigen Verletzer
	// (host-seitiges Saeen von Fluid-Volumenkraeften a la Upstream-Doku). Bewusst STRENGER
	// als die Lesemenge (prueft auch Gas-/Halozellen) -- fail-safe-Richtung (Pruefagent NIEDRIG-1).
	if(f_nur_solid_an()) {
		ulong geprueft = 0ull;
		for(uint d=0u; d<get_D(); d++) {
			LBM_Domain* dom = lbm_domain[d];
			const uint Nx=dom->get_Nx(), Ny=dom->get_Ny();
			for(uint zb=0u; zb<dom->fbnz; zb++) for(uint yb=0u; yb<dom->fbny; yb++) for(uint xb=0u; xb<dom->fbnx; xb++) {
				const ulong fbi = (ulong)xb+((ulong)yb+(ulong)zb*(ulong)dom->fbny)*(ulong)dom->fbnx;
				const ulong n   = (ulong)(dom->fbx0+xb)+((ulong)(dom->fby0+yb)+(ulong)(dom->fbz0+zb)*(ulong)Ny)*(ulong)Nx;
				if((dom->flags[n]&(TYPE_S|TYPE_E))!=TYPE_S&&(dom->F(fbi,0u)!=0.0f||dom->F(fbi,1u)!=0.0f||dom->F(fbi,2u)!=0.0f)) // Host-Maske: TYPE_BO existiert nur device-seitig; (S|E)!=S = exakt die update_force_field-Schreibbedingung invertiert
					print_error("F-NUR-SOLID aktiv, aber F != 0 an Nicht-Solid-Zelle n="+to_string(n)+" (Domaene "+to_string(d)+") -- dieses Setup nutzt Fluid-Volumenkraefte: CFD_F_NUR_SOLID=0 setzen.");
				geprueft++;
			}
		}
		print_info("F-Waechter: "+to_string(geprueft)+" F-BBox-Zellen geprueft, F an Nicht-Solid ueberall 0 -- F-NUR-SOLID-Praemisse haelt.");
	}
#endif // PARTICLES
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->F.enqueue_write_to_device();
	communicate_F();
#endif // FORCE_FIELD
#ifdef SURFACE
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->phi.enqueue_write_to_device();
#endif // SURFACE
#ifdef TEMPERATURE
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->T.enqueue_write_to_device();
#endif // TEMPERATURE
#ifdef PARTICLES
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->particles.enqueue_write_to_device();
	communicate_particles();
#endif // PARTICLES

	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->increment_time_step(); // the communicate calls at initialization need an odd time step
	communicate_rho_u_flags();
#ifdef SURFACE
	communicate_phi_massex_flags();
#endif // SURFACE
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_initialize(); // odd time step is baked-in the kernel
	communicate_rho_u_flags();
#ifdef SURFACE
	communicate_phi_massex_flags();
#endif // SURFACE
	communicate_fi(); // time step must be odd here
#ifdef TEMPERATURE
	communicate_T(); // T halo data is required for field_slice rendering
	communicate_gi(); // time step must be odd here
#endif // TEMPERATURE
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->reset_time_step(); // set time step to 0 again
	initialized = true;
}

void LBM::do_time_step(const bool sync_single_gpu) { // call kernel_stream_collide to perform one LBM time step
#ifdef SURFACE
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_surface_0();
#endif // SURFACE
	// FORK: u am Druck-Auslass VOR stream_collide setzen.
	// ★ KORREKTUR 2026-08-08 (Pruefer-Befund): hier stand, der Geschwindigkeitsteil sei innerhalb eines
	// Chunks idempotent und hinke um bis zu CFD_SAMPLE_EVERY Schritte hinterher. Das galt fuer einen
	// Stand OHNE UPDATE_FIELDS. UPDATE_FIELDS ist inzwischen fest eingeschaltet (defines.hpp), also
	// schreibt stream_collide rho und u in JEDEM Schritt (der Block ist nur fuer TYPE_E ausgenommen).
	// Der Per-Schritt-Dispatch ist damit nicht Vorrat, sondern noetig: die Neumann-Bedingung sieht das
	// Innenfeld des unmittelbar vorangegangenen Schritts. Der alte Kommentar war zu pessimistisch und
	// widersprach dem, was setup.cpp an derselben Sache richtig beschreibt.
	// ★ Audit-Nacharbeit 9 (E9), Fallzuordnung in R2 korrigiert: die 1580 vi-po-Doppelzellen
	// (Auslassebenen-Kanten) entstehen im FERNFELD-Fall mit CFD_FERN_VI=1 -- der dd-Fall setzt
	// keinen velocity_inlet (enqueue ist dort ein No-Op). Reihenfolge bewusst: erst Einlass, dann
	// Auslass -- auf Doppelzellen GEWINNT der Druck-Auslass; in-order-Queue macht es deterministisch.
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_apply_velocity_inlet(); // FORK: rho am Einlass mitlaufen lassen
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_apply_pressure_outlet();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_stream_collide(); // run LBM stream_collide kernel after domain communication
	// ★ REIHENFOLGE-UMSTELLUNG 2026-08-22 (Heiko-Vorgabe): die Aufpraegung nah->fern laeuft jetzt
	// VOR dem Moving-Floor-Fix, nicht mehr danach. Der Boden-Fix behaelt damit das letzte Wort an
	// der Fahrbahn.
	//
	// ★ RICHTIGSTELLUNG DERSELBEN SITZUNG (Pruefagent-Befund 2, MITTEL/HOCH): hier stand als
	// Begruendung, der Blend habe vorher "genau das Bodenband ueberschrieben, das der Boden-Fix
	// eben gesetzt hatte". DAS IST FALSCH. z_lo = max(1u, CFD_FERN_BODEN_EQ+1u) steht schon in
	// a4c3fa5 in BEIDEN Listenbauern (dort Zeilen 2772 und 2861) -- die Zellmengen waren immer
	// disjunkt, es gab kein Ueberschreiben zu reparieren. Die Umstellung bleibt (sie ist Heikos
	// Vorgabe und die Rangfolge ist die physikalisch gewollte), aber sie repariert nichts.
	//
	// WAS SIE STATTDESSEN TUT -- und das ist der eigentliche Punkt: sie ist NICHT wirkungslos,
	// obwohl die Mengen disjunkt sind. Esoteric-Pull laesst store_f(n) fuer jedes ungerade i in
	// den Speicher des NACHBARN j[i] schreiben, und load_f liest genau diese Slots. Die unterste
	// Blend-Lage (z = nz+1) und die oberste boden_eq-Zelle (z = nz) sind direkte Nachbarn. Wer
	// zuerst schreibt, bestimmt, was der andere liest. Beide Reihenfolgen koppeln also, nur in
	// die jeweils andere Richtung. FOLGE: jeder Lauf mit CFD_FERN_BODEN_EQ>0 UND aktivem N2F ist
	// gegen a4c3fa5 nicht mehr bitgleich -- und das ist der Normalfall. Die frueheren Schalen-A/Bs
	// sind mit diesem Binary nicht reproduzierbar.
	//
	// FOLGE FUER DEN LISTENBAUER: der z_lo-Ausschluss ist NICHT redundant (die gegenteilige Notiz
	// in setup.cpp war ebenfalls falsch und ist dort korrigiert). Ohne ihn schriebe der Blend in
	// Zellen, die boden_eq danach vollstaendig ueberschreibt -- ein echter, teilweise wirkungsloser
	// Blend plus zusaetzliche Nachbarkopplung. Er bleibt und ist tragend.
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_schale_blend(); // ★ P9c N2F-SCHALE (No-Op wenn aus ODER alpha==0)
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_boden_eq(); // V1-Port (No-Op wenn aus) -- liegt jetzt UEBER dem Blend
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_einlass_eq(); // V1-Port apply_inlet_velocity (No-Op wenn aus; Ecken-Ueberlapp mit boden_eq unkritisch, s. Kernel-Kommentar)
#if defined(SURFACE) || defined(GRAPHICS)
	communicate_rho_u_flags(); // rho/u/flags halo data is required for SURFACE extension, and u halo data is required for Q-criterion rendering
#endif // SURFACE || GRAPHICS
#ifdef SURFACE
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_surface_1();
	communicate_flags();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_surface_2();
	communicate_flags();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_surface_3();
	communicate_phi_massex_flags();
#endif // SURFACE
	communicate_fi();
#ifdef TEMPERATURE
#ifdef GRAPHICS
	communicate_T(); // T halo data is required for field_slice rendering
#endif // GRAPHICS
	communicate_gi();
#endif // TEMPERATURE
#ifdef PARTICLES
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_integrate_particles(); // intgegrate particles forward in time and couple particles to fluid
	communicate_particles(); // communicate_F() is not required in do_time_step()
#endif // PARTICLES
	// FORK: sync_single_gpu=false ueberspringt diese Barriere -- dann wartet run_async() nicht, und der Aufrufer
	// setzt die Barriere selbst per finish(). Im Mehr-Domaenen-Fall liefern die communicate_*-Aufrufe die Barrieren ohnehin.
	if(sync_single_gpu && get_D()==1u) for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue(); // this additional domain synchronization barrier is only required in single-GPU, as communication calls already provide all necessary synchronization barriers in multi-GPU
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->increment_time_step();
}

void LBM::run(const ulong steps, const ulong total_steps) { // initializes the LBM simulation (copies data to device and runs initialize kernel), then runs LBM
	info.append(steps, total_steps, get_t()); // total_steps parameter is just for runtime estimation
	// ★ C1b: Schalter an, aber nie gebunden = der lautlose No-Op, den dieses Projekt jagt -- hart.
	for(uint d=0u; d<get_D(); d++) if(lbm_domain[d]->facetten_on&&!lbm_domain[d]->facetten_bound)
		print_error("CFD_FACETTEN ist gesetzt, aber alloc_facetten() wurde nie gerufen -- der Kernel rechnete mit 1-Element-Platzhaltern.");
	if(!initialized) {
		initialize();
		info.print_initialize(this); // only print setup info if the setup is new (run() was not called before)
#ifdef GRAPHICS
		camera.allow_rendering = true;
#endif // GRAPHICS
	}
	Clock clock;
	for(ulong i=1ull; i<=steps; i++) {
#if defined(INTERACTIVE_GRAPHICS)||defined(INTERACTIVE_GRAPHICS_ASCII)
		while(!key_P&&running) sleep(0.016);
		if(!running) break;
#endif // INTERACTIVE_GRAPHICS_ASCII || INTERACTIVE_GRAPHICS
		clock.start();
		do_time_step();
		info.update(clock.stop());
	}
	if(get_D()>1u) for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue(); // wait for everything to finish (multi-GPU only)
}

// FORK Doppel-Domaene: `steps` Zeitschritte in die Warteschlange stellen und SOFORT zurueckkehren.
// Damit rechnet die Coarse-Domaene auf der iGPU, waehrend der Host die Ebenen liftet und die Fine-Domaene
// auf der dGPU ihre r Unterschritte macht. Wer danach rho/u/flags/fi liest, MUSS vorher finish() rufen.
void LBM::run_async(const ulong steps) {
	if(!initialized) { print_error("LBM::run_async vor der Initialisierung aufgerufen. Erst run() einmal rufen, dann run_async."); return; }
	info.append(steps, max_ulong, get_t());
	for(ulong i=1ull; i<=steps; i++) do_time_step(false);
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->flush_queue(); // Perf-Audit 2026-08-20: ohne Flush haengt die Submission am NEO-Treiberverhalten -- der iGPU-Overlap war bisher Glueck, jetzt Garantie
}

void LBM::finish() { // FORK: Barriere ueber alle Warteschlangen dieser LBM-Instanz
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
}

void LBM::update_fields() { // update fields (rho, u, T) manually
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_update_fields();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
}

void LBM::finalize_sparse_tiles() { // FORK: Block-Tiling abschliessen (no-op wenn ausgeschaltet)
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finalize_sparse_tiles();
}


// FORK -- Druck-Auslass, allgemeine Fassung.
//
// WAS ES IST, ehrlich benannt: ein Gleichgewichtsrand mit VORGESCHRIEBENER DICHTE und aus dem Inneren
// EXTRAPOLIERTER GESCHWINDIGKEIT. Jeden Schritt vor stream_collide wird an den Auslasszellen
// rho = rho_out gesetzt und u aus der zugehoerigen Innenzelle kopiert (Nullgradient). Die TYPE_E-Logik
// in stream_collide macht daraus f = f_eq(rho_out, u_extrapoliert).
// Das ist NICHT das Zou-He-Schema: Zou-He rekonstruiert die unbekannten Verteilungen aus den bekannten
// ueber den Nichtgleichgewichts-Anteil. Hier wird f_neq am Rand verworfen. Der Rand ist damit erster
// Ordnung und leicht ueberdaempft -- aber er ist stabil, allgemein und nachpruefbar. Ein echter
// Nichtgleichgewichts-Rand muesste fi in der Esoteric-Pull-Ablage ueberschreiben und braucht eine
// eigene Validierungskampagne; siehe die Notiz am Ende dieser Funktion.
//
// Bits im face_mask: 1=x_min 2=x_max 4=y_min 8=y_max 16=z_min 32=z_max
//
// WICHTIG: die Flags werden vom HOST gelesen. Zur Setup-Zeit ist der Host aktuell; ein
// read_from_device() wuerde die gerade gesetzten Randbedingungen mit altem Stand ueberschreiben.
// FORK -- gemeinsamer Sammler fuer BEIDE vorgeschriebenen Raender (Druck-Auslass und
// Geschwindigkeits-Einlass). Bewusst EINE Fassung statt zweier Kopien: die Zuordnung Randzelle ->
// Innenzelle ist die heikle Stelle (Kanten, Ecken, Dimensionen der Dicke 1), sie wurde am
// 2026-08-08 von drei Pruefern durchgesehen und zweimal korrigiert. Eine zweite Kopie waere die
// naechste Stelle, an der beide auseinanderlaufen, ohne dass es jemand merkt. `wofuer` geht nur
// in die Meldungen ein.
bool LBM_Domain::collect_boundary_pairs(const uint face_mask, const string& wofuer, std::vector<ulong>& cells, std::vector<ulong>& interior) {
	if(face_mask==0u) { print_warning(wofuer+": face_mask=0, nichts gesetzt."); return false; }
	const ulong NxNy = (ulong)Nx*(ulong)Ny;
	auto IDX = [&](const int x, const int y, const int z) { return (ulong)x + (ulong)y*(ulong)Nx + (ulong)z*NxNy; };
	auto is_fluid = [&](const int x, const int y, const int z) { // echte Innenzelle: weder Rand noch Solid
		if(x<0||x>=(int)Nx||y<0||y>=(int)Ny||z<0||z>=(int)Nz) return false;
		// TYPE_BO existiert nur device-seitig; host-seitig ist die Maske TYPE_S|TYPE_E.
		// HINWEIS: TYPE_G (Gas) und TYPE_I (Interface) gelten hier als Fluid. Solange SURFACE aus ist,
		// koennen sie nicht auftreten (lbm.cpp bricht sonst beim Sanity-Check ab). Wird SURFACE je
		// eingeschaltet, muesste die Maske erweitert werden -- sonst extrapoliert der Auslass aus einer Gaszelle.
		return (flags[IDX(x,y,z)]&(TYPE_S|TYPE_E))==0u;
	};
	// Welche Flaechen sind angefordert? Reihenfolge = Bitreihenfolge, Vorzeichen = einwaerts.
	const bool f_xmin=(face_mask&1u)!=0u, f_xmax=(face_mask&2u)!=0u;
	const bool f_ymin=(face_mask&4u)!=0u, f_ymax=(face_mask&8u)!=0u;
	const bool f_zmin=(face_mask&16u)!=0u, f_zmax=(face_mask&32u)!=0u;

	cells.clear(); interior.clear();
	ulong n_composite=0ull, n_fallback=0ull, n_skipped=0ull, n_degenerate=0ull, n_on_face=0ull;
	// Jede Randzelle GENAU EINMAL: ueber alle Zellen laufen und pruefen, ob sie auf einer der
	// angeforderten Flaechen liegt. Der frueher benutzte Weg (pro Flaeche sammeln) trug Kanten- und
	// Eckzellen mehrfach ein -- zwei Work-Items schrieben dann dieselbe Zelle, Reihenfolge undefiniert.
	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
		const ulong n = IDX((int)x,(int)y,(int)z);
		if((flags[n]&TYPE_E)==0u) continue; // nur TYPE_E; Solid und Fluid bleiben unangetastet
		int dx=0, dy=0, dz=0; // zusammengesetzte EINWAERTS-Richtung
		bool on_face = false; // ob die Zelle ueberhaupt auf einer angeforderten Flaeche liegt
		if(f_xmin && x==0u)      { dx += 1; on_face = true; }
		if(f_xmax && x==Nx-1u)   { dx -= 1; on_face = true; }
		if(f_ymin && y==0u)      { dy += 1; on_face = true; }
		if(f_ymax && y==Ny-1u)   { dy -= 1; on_face = true; }
		if(f_zmin && z==0u)      { dz += 1; on_face = true; }
		if(f_zmax && z==Nz-1u)   { dz -= 1; on_face = true; }
		if(!on_face) continue; // liegt wirklich auf keiner angeforderten Flaeche
		n_on_face++;
		// ★ 2026-08-08, von einem unabhaengigen Pruefer gefunden: die Flaechenzugehoerigkeit MUSS getrennt
		// von der Richtung bestimmt werden. Bei einer Dimension der Ausdehnung 1 gilt x==0 UND x==Nx-1
		// gleichzeitig; sind beide Bits dieser Achse gesetzt, heben sich +1 und -1 zu null auf. Die
		// vorige Fassung pruefte nur (dx==0&&dy==0&&dz==0) und verwarf solche Zellen STILL -- sie wurden
		// nicht einmal gezaehlt, und die Ausgabe meldete vollstaendige Behandlung. Reproduziert an
		// 1x6x6 mit face_mask=7: 6 Zellen verloren, keine Warnung.
		if(dx==0 && dy==0 && dz==0) { n_degenerate++; continue; } // gegenueberliegende Flaechen bei Dicke 1
		// 1. Versuch: ein Schritt entlang der zusammengesetzten Richtung. Fuer eine Kante (zwei Flaechen)
		//    ist das die Diagonale und landet damit im Inneren, nicht auf der jeweils anderen Flaeche.
		if(is_fluid((int)x+dx, (int)y+dy, (int)z+dz)) {
			cells.push_back(n); interior.push_back(IDX((int)x+dx,(int)y+dy,(int)z+dz)); n_composite++;
			continue;
		}
		// 2. Versuch: naechster echter Fluidnachbar in der 26er-Nachbarschaft, kuerzeste Distanz zuerst.
		int bx=0,by=0,bz=0; int best=99;
		for(int oz=-1; oz<=1; oz++) for(int oy=-1; oy<=1; oy++) for(int ox=-1; ox<=1; ox++) {
			if(ox==0&&oy==0&&oz==0) continue;
			const int d2 = ox*ox+oy*oy+oz*oz;
			if(d2>=best) continue;
			if(is_fluid((int)x+ox,(int)y+oy,(int)z+oz)) { best=d2; bx=ox; by=oy; bz=oz; }
		}
		if(best<99) { cells.push_back(n); interior.push_back(IDX((int)x+bx,(int)y+by,(int)z+bz)); n_fallback++; }
		else n_skipped++; // voellig eingeschlossen -> gar kein Auslass, auslassen statt raten
	}
	const uint N_po = (uint)cells.size();
	if(N_po==0u) { print_warning(wofuer+": keine TYPE_E-Zellen auf den angeforderten Flaechen gefunden."); return false; }

	// --- Selbstpruefung.
	// ★ 2026-08-08: die erste Fassung dieser Pruefung war TAUTOLOGISCH -- Eindeutigkeit und
	// "Innenzelle ist Fluid" sind durch die Sammelschleife bereits konstruktiv garantiert (je Zelle
	// hoechstens ein push_back; jede Innenzelle kam durch is_fluid). Sie konnte nie ausloesen und war
	// damit reine Beruhigung. Ein unabhaengiger Pruefer hat das nachgewiesen.
	// Was WIRKLICH brechen kann, ist die VOLLSTAENDIGKEIT: dass jede Zelle auf einer angeforderten
	// Flaeche auch irgendwo landet -- zugeordnet, uebersprungen oder degeneriert. Genau daran ist der
	// oben behobene Defekt vorbeigelaufen. Diese Bilanz wird jetzt geprueft.
	{
		if((ulong)N_po + n_skipped + n_degenerate != n_on_face) {
			print_error(wofuer+": Bilanz stimmt nicht -- "+to_string((uint)n_on_face)+" Zellen auf den Flaechen, aber nur "
				+to_string(N_po)+" zugeordnet + "+to_string((uint)n_skipped)+" uebersprungen + "+to_string((uint)n_degenerate)+" degeneriert. Es gehen Zellen still verloren.");
		}
		// Die konstruktiv garantierten Eigenschaften trotzdem pruefen -- nicht fuer heute, sondern als
		// Regressionsschutz, falls jemand die Sammelschleife umbaut. Dass sie heute nie auslesen, ist
		// kein Argument gegen sie, solange man nicht glaubt, sie wuerden etwas beweisen.
		std::vector<ulong> sorted_cells = cells;
		std::sort(sorted_cells.begin(), sorted_cells.end());
		if(std::adjacent_find(sorted_cells.begin(), sorted_cells.end())!=sorted_cells.end())
			print_error(wofuer+": mindestens eine Randzelle kommt mehrfach vor -- konkurrierende Schreibzugriffe.");
		for(uint i=0u; i<N_po; i++) {
			if((flags[interior[i]]&(TYPE_S|TYPE_E))!=0u) print_error(wofuer+": Innenzelle "+to_string(interior[i])+" ist selbst Rand oder Solid.");
			if(interior[i]==cells[i]) print_error(wofuer+": Innenzelle zeigt auf sich selbst.");
		}
	}
	print_info(wofuer+": "+to_string(N_po)+" Zellen (face_mask=0x"+to_string(face_mask)+")."
		+" Innenzelle direkt: "+to_string((uint)n_composite)+", ueber Nachbarsuche: "+to_string((uint)n_fallback)
		+(n_skipped? (", ohne Fluidnachbar uebersprungen: "+to_string((uint)n_skipped)) : string(""))
		+(n_degenerate? (", degeneriert (Dimension der Dicke 1): "+to_string((uint)n_degenerate)) : string(""))+".");
	return true;
}

void LBM_Domain::set_pressure_outlet_faces(const uint face_mask, const float rho_out) {
	po_rho = rho_out;
	{ const char* v = getenv("CFD_PO_SIGMA"); po_sigma = v ? (float)fmax(0.0, fmin(1.0, atof(v))) : 1.0f; }
	{ const char* v = getenv("CFD_PO_HART"); po_hart = (v && atoi(v)>0) ? 1u : 0u; } // 1 = alter harter Rand, der Kontrollarm (Audit-Nacharbeit 15: "false" zaehlte vorher als AN)
	std::vector<ulong> cells, interior;
	if(!collect_boundary_pairs(face_mask, "Druck-Auslass", cells, interior)) return;
	const uint N_po = (uint)cells.size();
	// Bit 1 ist x_min und damit ueblicherweise der EINLASS. Wer ihn als Auslass anfordert, verliert
	// stillschweigend die Zustroembedingung: u wird dort dann extrapoliert statt auf u_inf gehalten.
	if((face_mask&1u)!=0u) print_warning("Druck-Auslass: face_mask enthaelt x_min. Das ist normalerweise der EINLASS -- dort wird u jetzt extrapoliert statt vorgegeben.");

	po_N_active = N_po;
	po_cells    = Memory<ulong>(device, (ulong)N_po);
	po_interior = Memory<ulong>(device, (ulong)N_po);
	for(uint i=0u; i<N_po; i++) { po_cells[i] = cells[i]; po_interior[i] = interior[i]; }
	po_cells.write_to_device();
	po_interior.write_to_device();
	po_mean = Memory<float>(device, 1ull);
	// ★ 2026-08-24: Teilsummenpuffer je Arbeitsgruppe statt atomarer Addition. Die Allokation MUSS
	// vor der Kernel-Bindung stehen -- ein Move-Assignment auf einen bereits gebundenen Puffer ist
	// die im Baum zweimal dokumentierte Use-after-free-Klasse.
	po_groups = (uint)(((ulong)N_po+(ulong)WORKGROUP_SIZE-1ull)/(ulong)WORKGROUP_SIZE);
	po_part = Memory<float>(device, (ulong)po_groups);
	kernel_po_reduce_mean = Kernel(device, (ulong)N_po, "po_reduce_mean", rho, po_interior, N_po, po_part);
	kernel_po_final_mean  = Kernel(device, 1ull, "po_final_mean", po_part, po_groups, N_po, po_mean);
	kernel_apply_pressure_outlet = Kernel(device, (ulong)N_po, "apply_pressure_outlet", u, rho, po_cells, po_interior, N_po, po_rho, po_sigma, po_mean, po_hart);
	print_info("Druck-Auslass REDUKTION: "+to_string(N_po)+" Innenzellen in "+to_string(po_groups)+" Arbeitsgruppen, Endsumme in Indexordnung (atomikfrei, bitreproduzierbar seit 2026-08-24).");
	print_info(po_hart ? ("Druck-Auslass: HARTE Klemme rho = "+to_string(po_rho,4u)+" je Zelle (CFD_PO_HART=1, der alte Zustand als Kontrollarm), u aus der Innenzelle.")
		: ("Druck-Auslass: rho_out = "+to_string(po_rho,4u)+" als FLAECHENMITTEL verankert (Ankerrate sigma = "+to_string(po_sigma,4u)
		+"), rho der Einzelzelle laeuft frei mit, u aus der Innenzelle. Der Druck ist damit global festgelegt, seine Verteilung ueber die Ebene aber nicht."));
	// OFFEN, bewusst nicht hier geloest: ein echter Nichtgleichgewichts-Rand (Guo/Zheng/Shi 2002,
	// f_i = f_eq(rho_b,u_b) + [f_i(n) - f_eq(rho_n,u_n)]) waere zweiter Ordnung statt erster. Er muesste
	// fi am Rand NACH dem Streaming ueberschreiben, und in der Esoteric-Pull-Ablage gehoeren die Slots
	// einer Zelle teilweise ihren Nachbarn -- ein Fehler dort erzeugt still falsche Ergebnisse statt
	// eines Absturzes. Das braucht eine eigene Validierung und nicht denselben Commit.
}

// FORK 2026-08-08 -- GESCHWINDIGKEITS-EINLASS mit MITLAUFENDER DICHTE.
// Gemessen an einem leeren groben Kanal (kein Fahrzeug, keine Kopplung): der bisherige Einlass
// schreibt rho UND u vor. Fuer einen kompressiblen Loeser ist das ueberbestimmt -- eine von innen
// ankommende Druckwelle kann dort weder hinaus noch absorbiert werden, und die Massenbilanz geht
// jeden Schritt nicht auf. Die Differenz landet in der ERSTEN Fluidzelle dahinter: gemessen war
// die Stoerung dort 413 (willkuerliche Einheit) gegen 0 auf der Randebene selbst und 8 weit
// stromab. Die Streuung von u_x wuchs monoton von 0,012 auf 0,125, bei 38 % der Zellen ueber
// 10 % daneben -- im LEEREN Kanal, wo u_x konstant sein muesste.
// Viskositaet ist nicht der Hebel: der zehnfache Wert aendert nichts (10,94 -> 10,77 % verdorbene
// Zellen), der hundertfache halbiert nur (4,71 %), und selbst dann ist tau erst 0,5007.
// Die Abhilfe ist die Spiegelung des Druck-Auslasses: dort wird rho vorgeschrieben und u aus der
// Innenzelle genommen -- hier wird u vorgeschrieben (das erledigt TYPE_E) und rho aus der
// Innenzelle uebernommen. Damit ist genau EINE Groesse je Rand vorgegeben, und Druckwellen
// laufen hinaus statt zurueck.
void LBM_Domain::set_velocity_inlet_faces(const uint face_mask) {
	std::vector<ulong> cells, interior;
	if(!collect_boundary_pairs(face_mask, "Geschwindigkeits-Einlass", cells, interior)) return;
	const uint N_vi = (uint)cells.size();
	vi_N_active = N_vi;
	vi_cells    = Memory<ulong>(device, (ulong)N_vi);
	vi_interior = Memory<ulong>(device, (ulong)N_vi);
	for(uint i=0u; i<N_vi; i++) { vi_cells[i] = cells[i]; vi_interior[i] = interior[i]; }
	vi_cells.write_to_device();
	vi_interior.write_to_device();
	kernel_apply_velocity_inlet = Kernel(device, (ulong)N_vi, "apply_velocity_inlet", rho, vi_cells, vi_interior, N_vi);
	print_info("Geschwindigkeits-Einlass: u bleibt vorgeschrieben, rho laeuft mit der Innenzelle mit.");
}

void LBM_Domain::enqueue_apply_velocity_inlet() {
	if(vi_N_active==0u) return;
	kernel_apply_velocity_inlet.enqueue_run();
}

void LBM::set_velocity_inlet_faces(const uint face_mask) {
	if(get_D()!=1u) { print_warning("Geschwindigkeits-Einlass: nur fuer eine einzelne Domaene validiert, uebersprungen."); return; }
	lbm_domain[0]->set_velocity_inlet_faces(face_mask);
}

void LBM::set_pressure_outlet_faces(const uint face_mask, const float rho_out) {
	if(get_D()!=1u) { print_warning("Druck-Auslass: nur fuer eine einzelne Domaene validiert, uebersprungen."); return; }
	lbm_domain[0]->set_pressure_outlet_faces(face_mask, rho_out);
}

// =====================================================================================
// FORK -- Doppel-Domaene: Host-Seite der Kopplung grob -> fein.
// Die ausfuehrliche Begruendung des Verfahrens (und die Liste dessen, was bewusst fehlt)
// steht bei den Kernels in kernel.cpp unter "Doppel-Domaene: Kopplung grobes Fernfeld".
// =====================================================================================
void LBM::alloc_coupling_planes(const ulong max_plane_cells) {
	if(get_D()!=1u) { print_error("Doppel-Domaenen-Kopplung: nur fuer je eine Domaene je LBM-Instanz gebaut."); return; }
	if(!initialized) { print_error("alloc_coupling_planes vor der Initialisierung. Erst run(1) rufen."); return; }
	lbm_domain[0]->alloc_coupling_planes(max_plane_cells);
}

// ★ Pruefer-Befund 2026-08-08: Eine Ebene, die aus der Domaene ragt, wickelt STILL um. Der Kernel prueft
// nur n>=def_N; ein Ueberlauf in x oder y bleibt darunter und landet einfach in der naechsten Zellzeile
// bzw. -ebene. Also kein Absturz, sondern richtige Werte an falschen Zellen -- genau die Fehlerklasse,
// die weder eine Norm noch ein Kraftverlauf sichtbar macht. Deshalb hier, host-seitig, vollstaendig gefasst.
bool LBM::plane_fits(const PlaneSpec& plane, const char* who) const {
	uint na=0u, nb=0u, nn=0u, oa=0u, ob=0u, on=0u; // Ausdehnung/Ursprung entlang a, entlang b, entlang der Normalen
	if(plane.axis==0u)      { na=Ny; nb=Nz; nn=Nx; oa=plane.origin.y; ob=plane.origin.z; on=plane.origin.x; }
	else if(plane.axis==1u) { na=Nx; nb=Nz; nn=Ny; oa=plane.origin.x; ob=plane.origin.z; on=plane.origin.y; }
	else if(plane.axis==2u) { na=Nx; nb=Ny; nn=Nz; oa=plane.origin.x; ob=plane.origin.y; on=plane.origin.z; }
	else { print_error(string(who)+": Ebenenachse "+to_string(plane.axis)+" gibt es nicht (erlaubt sind 0, 1, 2)."); return false; }
	if(plane.extent_a==0u || plane.extent_b==0u) { print_error(string(who)+": Ebene mit Ausdehnung 0."); return false; }
	if(on>=nn || (ulong)oa+(ulong)plane.extent_a>(ulong)na || (ulong)ob+(ulong)plane.extent_b>(ulong)nb) {
		print_error(string(who)+": Ebene ragt aus der Domaene. Ursprung ("+to_string(plane.origin.x)+","+to_string(plane.origin.y)+","+to_string(plane.origin.z)
			+"), Ausdehnung "+to_string(plane.extent_a)+"x"+to_string(plane.extent_b)+", Achse "+to_string(plane.axis)
			+", Domaene "+to_string(Nx)+"x"+to_string(Ny)+"x"+to_string(Nz)+".");
		return false;
	}
	return true;
}

void LBM::extract_plane_macros(const PlaneSpec& plane, std::vector<float>& host_buf) {
	LBM_Domain* dom = lbm_domain[0];
	const ulong n_plane = (ulong)plane.extent_a*(ulong)plane.extent_b;
	if(!plane_fits(plane, "extract_plane_macros")) return;
	if(dom->coupling_max_plane_cells==0ull) { print_error("extract_plane_macros ohne alloc_coupling_planes."); return; }
	if(n_plane>dom->coupling_max_plane_cells) { print_error("extract_plane_macros: Ebene mit "+to_string(n_plane)+" Zellen passt nicht in den Puffer ("+to_string(dom->coupling_max_plane_cells)+")."); return; }
	dom->kernel_extract_plane_macros.set_ranges(n_plane);
	dom->kernel_extract_plane_macros.set_parameters(3u, plane.axis,
		plane.origin.x, plane.origin.y, plane.origin.z, plane.extent_a, plane.extent_b);
	dom->kernel_extract_plane_macros.enqueue_run();
	dom->finish_queue();
	host_buf.resize(n_plane*4ull);
	dom->coupling_plane.read_from_device(0ull, n_plane*4ull); // nur den belegten Anfang zurueckholen
	for(ulong i=0ull; i<n_plane*4ull; i++) host_buf[i] = dom->coupling_plane[i];
}

// ★ Slice-Ebenen-Read (Perf-Hebel 2026-08-26, Plan "Variante b"): (rho,u,flags) EINER y-Ebene
// per Device-Gather holen und in die Host-Arrays streuen. Renderer, Diff-Schnitt und Sonden
// lesen unveraendert dieselben Host-Indizes -- es aendert sich NUR der Transportweg
// (4-mm-Nahdomaene: ~14 MB statt ~8,65 GB je Slice-Ereignis). Wertgleichheit per Konstruktion:
// identische floats, Indexkonvention exakt plane_cell_index (x + y*Nx + z*Nx*Ny).
void LBM::lese_yslice_in_host(const uint y) {
	LBM_Domain* dom = lbm_domain[0];
#ifndef UPDATE_FIELDS
	if(initialized) dom->enqueue_update_fields(); // wie Memory_Container::read_from_device(): u/rho erst aktualisieren
#endif // UPDATE_FIELDS
	PlaneSpec plane; plane.origin = uint3(0u, y, 0u); plane.extent_a = Nx; plane.extent_b = Nz; plane.axis = 1u;
	static std::vector<float> ebene; // sequenzielle Nutzung im Hauptthread; vor jedem Aufruf geleert
	ebene.clear();
	extract_plane_macros(plane, ebene);
	const ulong n_plane = (ulong)Nx*(ulong)Nz;
	if(ebene.size()<n_plane*4ull) return; // Verteidigung: heute unerreichbar (jeder Wrapper-Abbruch endet in print_error/exit), bleibt fuer den Fall, dass print_error je nicht-fatal wird (Auditor-A NIEDRIG-1)
	dom->kernel_extract_plane_flags.set_ranges(n_plane);
	dom->kernel_extract_plane_flags.set_parameters(2u, plane.axis,
		plane.origin.x, plane.origin.y, plane.origin.z, plane.extent_a, plane.extent_b);
	dom->kernel_extract_plane_flags.enqueue_run();
	dom->finish_queue();
	dom->slice_flags.read_from_device(0ull, n_plane);
	for(uint z=0u; z<Nz; z++) for(uint x=0u; x<Nx; x++) {
		const ulong g = (ulong)x + (ulong)z*(ulong)Nx;                                // Ebenen-Index (a=x, b=z)
		const ulong n = (ulong)x + ((ulong)y + (ulong)z*(ulong)Ny)*(ulong)Nx;         // Domaenen-Index
		const ulong o = g*4ull;
		rho[n] = ebene[o]; u.x[n] = ebene[o+1ull]; u.y[n] = ebene[o+2ull]; u.z[n] = ebene[o+3ull];
		flags[n] = dom->slice_flags[g];
	}
}

void LBM::drive_boundary_from_coarse(const PlaneSpec& fine_plane, const std::vector<float>& coarse_face, const uint coarse_a, const uint coarse_b, const uint ratio) {
	LBM_Domain* dom = lbm_domain[0];
	const ulong n_coarse = (ulong)coarse_a*(ulong)coarse_b;
	// ratio=0 waere im Kernel eine Ganzzahldivision durch null (undefiniert), coarse_a=0 liesse
	// (coarse_a-1u)*ratio als uint unterlaufen und die Konventionspruefung unten durchgehen.
	if(ratio==0u || coarse_a==0u || coarse_b==0u) { print_error("drive_boundary_from_coarse: ratio="+to_string(ratio)+", grobe Ausdehnung "+to_string(coarse_a)+"x"+to_string(coarse_b)+" -- keines davon darf 0 sein."); return; }
	if(!plane_fits(fine_plane, "drive_boundary_from_coarse")) return;
	if(dom->coupling_max_plane_cells==0ull) { print_error("drive_boundary_from_coarse ohne alloc_coupling_planes."); return; }
	if(n_coarse>dom->coupling_max_plane_cells) { print_error("drive_boundary_from_coarse: grobe Ebene passt nicht in den Puffer."); return; }
	if((ulong)coarse_face.size()<n_coarse*4ull) { print_error("drive_boundary_from_coarse: grobe Ebene zu klein ("+to_string((ulong)coarse_face.size())+" < "+to_string((ulong)(n_coarse*4ull))+")."); return; }
	// Deckungspunkt-Konvention pruefen. Stimmt sie nicht, laege die interpolierte Ebene raeumlich
	// verschoben auf dem Rand -- ein Fehler, den kein Kraftverlauf als solchen zeigen wuerde.
	if(fine_plane.extent_a!=(coarse_a-1u)*ratio+1u || fine_plane.extent_b!=(coarse_b-1u)*ratio+1u) {
		print_error("drive_boundary_from_coarse: Ausdehnungen passen nicht zusammen. Grob "+to_string(coarse_a)+"x"+to_string(coarse_b)
			+" bei ratio="+to_string(ratio)+" verlangt fein "+to_string((coarse_a-1u)*ratio+1u)+"x"+to_string((coarse_b-1u)*ratio+1u)
			+", bekommen "+to_string(fine_plane.extent_a)+"x"+to_string(fine_plane.extent_b)+".");
		return;
	}
	for(ulong i=0ull; i<n_coarse*4ull; i++) dom->coupling_plane[i] = coarse_face[i];
	dom->coupling_plane.write_to_device(0ull, n_coarse*4ull);
	dom->kernel_drive_boundary_cubic_lift.set_ranges((ulong)fine_plane.extent_a*(ulong)fine_plane.extent_b);
	dom->kernel_drive_boundary_cubic_lift.set_parameters(4u, fine_plane.axis,
		fine_plane.origin.x, fine_plane.origin.y, fine_plane.origin.z, fine_plane.extent_a, fine_plane.extent_b,
		coarse_a, coarse_b, ratio);
	dom->kernel_drive_boundary_cubic_lift.enqueue_run();
	dom->finish_queue();
}

// ★ P9c N2F-SCHALE: LBM-Ebenen-Wrapper (Muster alloc_coupling_planes/extract_plane_macros).
void LBM::alloc_schale(const std::vector<ulong>& liste, const std::vector<float>& gewichte, const uint ratio, const uint modus) {
	if(get_D()!=1u) { print_error("N2F-Schale: nur fuer je eine Domaene je LBM-Instanz gebaut."); return; }
	if(!initialized) { print_error("alloc_schale vor der Initialisierung. Erst run(0) rufen."); return; }
	lbm_domain[0]->alloc_schale(liste, gewichte, ratio, modus);
}

void LBM::schale_extract_u(std::vector<float>& out, const uint mittel) {
	LBM_Domain* dom = lbm_domain[0];
	if(dom->schale_n==0u) { print_error("schale_extract_u ohne alloc_schale."); return; }
	// Blockierend (Muster extract_plane_macros): der Aufrufer steht im Kopplungsfenster, die
	// Warteschlange dieser Instanz ist dort ohnehin leer (lbm_f nach run(ratio), lbm_c nach finish()).
	dom->kernel_schale_extract.set_parameters(5u, mittel).enqueue_run();
	dom->finish_queue();
	dom->schale_uout.read_from_device();
	const ulong m = 3ull*(ulong)dom->schale_n;
	out.resize(m);
	for(ulong i=0ull; i<m; i++) out[i] = dom->schale_uout[i];
}

void LBM::schale_upload_unear(const std::vector<float>& unear) {
	LBM_Domain* dom = lbm_domain[0];
	if(dom->schale_n==0u) { print_error("schale_upload_unear ohne alloc_schale."); return; }
	const ulong m = 3ull*(ulong)dom->schale_n;
	if((ulong)unear.size()<m) { print_error("schale_upload_unear: Puffer zu klein ("+to_string((ulong)unear.size())+" < "+to_string(m)+")."); return; }
	for(ulong i=0ull; i<m; i++) dom->schale_unear[i] = unear[i];
	dom->schale_unear.write_to_device();
}

void LBM::reset() { // reset simulation (takes effect in following run() call)
	initialized = false;
}

#ifdef FORCE_FIELD
void LBM::update_force_field() { // calculate forces from fluid on TYPE_S cells
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_update_force_field();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
}
float3 LBM::object_center_of_mass(const uchar flag_marker) { // calculate center of mass of all cells flagged with flag_marker
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_object_center_of_mass(flag_marker);
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
	float3 object_com = float3(0.0f, 0.0f, 0.0f);
	ulong object_cells = 0ull;
	for(uint d=0u; d<get_D(); d++) {
		object_com += float3(lbm_domain[d]->object_sum.x[0], lbm_domain[d]->object_sum.y[0], lbm_domain[d]->object_sum.z[0]);
		object_cells += (ulong)as_uint(lbm_domain[d]->object_sum.w[0]);
	}
	return object_com/(float)object_cells;
}
float3 LBM::object_force(const uchar flag_marker) { // add up force for all cells flagged with flag_marker
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_object_force(flag_marker);
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
	float3 object_force = float3(0.0f, 0.0f, 0.0f);
	for(uint d=0u; d<get_D(); d++) object_force += float3(lbm_domain[d]->object_sum.x[0], lbm_domain[d]->object_sum.y[0], lbm_domain[d]->object_sum.z[0]);
	return object_force;
}
float3 LBM::object_force_zband(const uchar flag_marker, const uint z_lo, const uint z_hi) { // FORK Kraft-Zerlegung (CFD_KRAFT_ZBAND): object_force auf das z-Band [z_lo,z_hi)
	if(get_D()>1u) print_error("object_force_zband: coordinates() ist domaenenlokal -- nur D=1.");
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_object_force_zband(flag_marker, z_lo, z_hi);
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
	float3 object_force = float3(0.0f, 0.0f, 0.0f);
	for(uint d=0u; d<get_D(); d++) object_force += float3(lbm_domain[d]->object_sum.x[0], lbm_domain[d]->object_sum.y[0], lbm_domain[d]->object_sum.z[0]);
	return object_force;
}
float3 LBM::object_torque(const float3& rotation_center, const uchar flag_marker) { // add up torque around specified rotation center for all cells flagged with flag_marker
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_object_torque(rotation_center, flag_marker);
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
	float3 object_torque = float3(0.0f, 0.0f, 0.0f);
	for(uint d=0u; d<get_D(); d++) object_torque += float3(lbm_domain[d]->object_sum.x[0], lbm_domain[d]->object_sum.y[0], lbm_domain[d]->object_sum.z[0]);
	return object_torque;
}
#endif // FORCE_FIELD

#ifdef MOVING_BOUNDARIES
void LBM::update_moving_boundaries() { // mark/unmark cells next to TYPE_S cells with velocity!=0 with TYPE_MS
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_update_moving_boundaries();
	communicate_flags();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
#ifdef GRAPHICS
	camera.key_update = true; // to prevent flickering of flags in interactive graphics when camera is not moved
#endif // GRAPHICS
}
#endif // MOVING_BOUNDARIES

#if defined(PARTICLES)&&!defined(FORCE_FIELD)
void LBM::integrate_particles(const ulong steps, const ulong total_steps, const uint time_step_multiplicator) { // intgegrate passive tracer particles forward in time in stationary flow field
	info.append(steps, total_steps, get_t());
	Clock clock;
	for(ulong i=1ull; i<=steps; i+=(ulong)time_step_multiplicator) {
#if defined(INTERACTIVE_GRAPHICS)||defined(INTERACTIVE_GRAPHICS_ASCII)
		while(!key_P&&running) sleep(0.016);
		if(!running) break;
#endif // INTERACTIVE_GRAPHICS_ASCII || INTERACTIVE_GRAPHICS
		clock.start();
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_integrate_particles(time_step_multiplicator);
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->increment_time_step(time_step_multiplicator);
		info.update(clock.stop());
	}
}
#endif // PARTICLES&&!FORCE_FIELD

void LBM::write_status(const string& path) { // write LBM status report to a .txt file
	string status = "";
	status += "Grid Resolution = "+to_string(Nx)+" x "+to_string(Ny)+" x "+to_string(Nz)+" = "+to_string(get_N())+"\n";
	status += "Grid Domains = "+to_string(Dx)+" x "+to_string(Dy)+" x "+to_string(Dz)+" = "+to_string(get_D())+"\n";
	status += "LBM Type = D"+string(get_velocity_set()==9 ? "2" : "3")+"Q"+to_string(get_velocity_set())+" "+info.collision+"\n";
	status += "Memory Usage = CPU "+to_string(info.cpu_mem_required)+" MB, GPU "+to_string(get_D())+"x "+to_string(info.gpu_mem_required)+" MB\n";
	status += "Maximum Allocation Size = "+to_string((uint)(get_N()/(ulong)get_D()*(ulong)(get_velocity_set()*sizeof(fpxx))/1048576ull))+" MB\n";
	status += "Time Steps = "+to_string(get_t())+" / "+(info.steps==max_ulong ? "infinite" : to_string(info.steps))+"\n";
	status += "Runtime = "+print_time(info.runtime_total)+" (total) = "+print_time(info.runtime_lbm)+" (LBM) + "+print_time(info.runtime_total-info.runtime_lbm)+" (rendering and data evaluation)\n";
	status += "Average MLUPs/s = "+to_string(to_uint(1E-6*(double)get_N()*(double)get_t()/info.runtime_lbm))+"\n";
	status += "Kinematic Viscosity = "+to_string(get_nu())+"\n";
	status += "Relaxation Time = "+to_string(get_tau())+"\n";
	status += "Maximum Reynolds Number = "+to_string(get_Re_max())+"\n";
#ifdef VOLUME_FORCE
	status += "Volume Force = ("+to_string(get_fx())+", "+to_string(get_fy())+", "+to_string(get_fz())+")\n";
#endif // VOLUME_FORCE
#ifdef SURFACE
	status += "Surface Tension Coefficient = "+to_string(get_sigma())+"\n";
#endif // SURFACE
#ifdef TEMPERATURE
	status += "Thermal Diffusion Coefficient = "+to_string(get_alpha())+"\n";
	status += "Thermal Expansion Coefficient = "+to_string(get_beta())+"\n";
#endif // TEMPERATURE
	const string filename = default_filename(path, "status", ".txt", get_t());
	write_file(filename, status);
}

void LBM::voxelize_mesh_on_device(const Mesh* mesh, const uchar flag, const float3& rotation_center, const float3& linear_velocity, const float3& rotational_velocity) { // voxelize triangle mesh
	if(get_D()==1u) {
		lbm_domain[0]->voxelize_mesh_on_device(mesh, flag, rotation_center, linear_velocity, rotational_velocity); // if this crashes on Windows, create a TdrDelay 32-bit DWORD with decimal value 300 in Computer\HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Control\GraphicsDrivers
	} else {
		parallel_for(get_D(), get_D(), [&](uint d) {
			lbm_domain[d]->voxelize_mesh_on_device(mesh, flag, rotation_center, linear_velocity, rotational_velocity);
		});
	}
#ifdef MOVING_BOUNDARIES
	if((flag&(TYPE_S|TYPE_E))==TYPE_S&&(length(linear_velocity)>0.0f||length(rotational_velocity)>0.0f)) update_moving_boundaries();
#endif // MOVING_BOUNDARIES
	if(!initialized) {
		flags.read_from_device();
		u.read_from_device();
	}
}
void LBM::unvoxelize_mesh_on_device(const Mesh* mesh, const uchar flag) { // remove voxelized triangle mesh from LBM grid by removing all flags in mesh bounding box (only required when bounding box size changes during re-voxelization)
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_unvoxelize_mesh_on_device(mesh, flag);
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
}
void LBM::write_mesh_to_vtk(const Mesh* mesh, const string& path, const bool convert_to_si_units) const { // write mesh to binary .vtk file
	const string filename = default_filename(path, "mesh", ".vtk", get_t());
	const string header_1 = "# vtk DataFile Version 3.0\nFluidX3D "+filename.substr(filename.rfind('/')+1)+"\nBINARY\nDATASET POLYDATA\nPOINTS "+to_string(3u*mesh->triangle_number)+" float\n";
	const string header_2 = "POLYGONS "+to_string(mesh->triangle_number)+" "+to_string(4u*mesh->triangle_number)+"\n";
	float* points = new float[9u*mesh->triangle_number];
	int* triangles = new int[4u*mesh->triangle_number];
	const float spacing = convert_to_si_units ? units.si_x(1.0f) : 1.0f;
	const float3 offset = center();
	parallel_for(mesh->triangle_number, [&](uint i) {
		points[9u*i   ] = reverse_bytes(spacing*(mesh->p0[i].x-offset.x));
		points[9u*i+1u] = reverse_bytes(spacing*(mesh->p0[i].y-offset.y));
		points[9u*i+2u] = reverse_bytes(spacing*(mesh->p0[i].z-offset.z));
		points[9u*i+3u] = reverse_bytes(spacing*(mesh->p1[i].x-offset.x));
		points[9u*i+4u] = reverse_bytes(spacing*(mesh->p1[i].y-offset.y));
		points[9u*i+5u] = reverse_bytes(spacing*(mesh->p1[i].z-offset.z));
		points[9u*i+6u] = reverse_bytes(spacing*(mesh->p2[i].x-offset.x));
		points[9u*i+7u] = reverse_bytes(spacing*(mesh->p2[i].y-offset.y));
		points[9u*i+8u] = reverse_bytes(spacing*(mesh->p2[i].z-offset.z));
		triangles[4u*i   ] = reverse_bytes(3); // 3 vertices per triangle
		triangles[4u*i+1u] = reverse_bytes(3*(int)i  ); // vertex 0
		triangles[4u*i+2u] = reverse_bytes(3*(int)i+1); // vertex 1
		triangles[4u*i+3u] = reverse_bytes(3*(int)i+2); // vertex 2
	});
	create_folder(filename);
	std::ofstream file(filename, std::ios::out|std::ios::binary);
	file.write(header_1.c_str(), header_1.length()); // write non-binary file header
	file.write((char*)points, 4u*9u*mesh->triangle_number); // write binary data
	file.write(header_2.c_str(), header_2.length()); // write non-binary file header
	file.write((char*)triangles, 4u*4u*mesh->triangle_number); // write binary data
	file.close();
	delete[] points;
	delete[] triangles;
	info.allow_printing.lock();
	print_info("File \""+filename+"\" saved.");
	info.allow_printing.unlock();
}
void LBM::voxelize_stl(const string& path, const float3& center, const float3x3& rotation, const float size, const uchar flag) { // voxelize triangle mesh
	const Mesh* mesh = read_stl(path, this->size(), center, rotation, size);
	flags.write_to_device();
	voxelize_mesh_on_device(mesh, flag);
	delete mesh;
	flags.read_from_device();
}
void LBM::voxelize_stl(const string& path, const float3x3& rotation, const float size, const uchar flag) { // read and voxelize binary .stl file (place in box center)
	voxelize_stl(path, center(), rotation, size, flag);
}
void LBM::voxelize_stl(const string& path, const float3& center, const float size, const uchar flag) { // read and voxelize binary .stl file (no rotation)
	voxelize_stl(path, center, float3x3(1.0f), size, flag);
}
void LBM::voxelize_stl(const string& path, const float size, const uchar flag) { // read and voxelize binary .stl file (place in box center, no rotation)
	voxelize_stl(path, center(), float3x3(1.0f), size, flag);
}

#ifdef GRAPHICS
int* LBM::Graphics::draw_frame() {
#ifndef UPDATE_FIELDS
	if(visualization_modes&(VIS_FIELD|VIS_STREAMLINES|VIS_Q_CRITERION)) {
		for(uint d=0u; d<lbm->get_D(); d++) lbm->lbm_domain[d]->enqueue_update_fields(); // only call update_fields() if the time step has changed since the last rendered frame
	}
#endif // UPDATE_FIELDS
	if(key_1) { visualization_modes = (visualization_modes&~0b11)|(((visualization_modes&0b11)+1)%4); key_1 = false; }
	if(key_2) { visualization_modes ^= VIS_FIELD        ; key_2 = false; }
	if(key_3) { visualization_modes ^= VIS_STREAMLINES  ; key_3 = false; }
	if(key_4) { visualization_modes ^= VIS_Q_CRITERION  ; key_4 = false; }
	if(key_5) { visualization_modes ^= VIS_PHI_RASTERIZE; key_5 = false; }
	if(key_6) { visualization_modes ^= VIS_PHI_RAYTRACE ; key_6 = false; }
	if(key_7) { visualization_modes ^= VIS_PARTICLES    ; key_7 = false; }
	if(key_T) {
		slice_mode = (slice_mode+1)%8; key_T = false;
	}
	if(key_Z) {
#ifndef TEMPERATURE
		field_mode = (field_mode+1)%2; key_Z = false; // field_mode = { 0 (u), 1 (rho) }
#else // TEMPERATURE
		field_mode = (field_mode+1)%3; key_Z = false; // field_mode = { 0 (u), 1 (rho), 2 (T) }
#endif // TEMPERATURE
	}
	if(slice_mode==1u) {
		if(key_Q) { slice_x = clamp(slice_x-1, 0, (int)lbm->get_Nx()-1); key_Q = false; }
		if(key_E) { slice_x = clamp(slice_x+1, 0, (int)lbm->get_Nx()-1); key_E = false; }
	}
	if(slice_mode==2u) {
		if(key_Q) { slice_y = clamp(slice_y-1, 0, (int)lbm->get_Ny()-1); key_Q = false; }
		if(key_E) { slice_y = clamp(slice_y+1, 0, (int)lbm->get_Ny()-1); key_E = false; }
	}
	if(slice_mode==3u) {
		if(key_Q) { slice_z = clamp(slice_z-1, 0, (int)lbm->get_Nz()-1); key_Q = false; }
		if(key_E) { slice_z = clamp(slice_z+1, 0, (int)lbm->get_Nz()-1); key_E = false; }
	}
	const bool visualization_change = camera.key_update||last_visualization_modes!=visualization_modes||last_field_mode!=field_mode||last_slice_mode!=slice_mode||last_slice_x!=slice_x||last_slice_y!=slice_y||last_slice_z!=slice_z;
	camera.key_update = false;
	last_visualization_modes = visualization_modes;
	last_field_mode = field_mode;
	last_slice_mode = slice_mode;
	last_slice_x = slice_x;
	last_slice_y = slice_y;
	last_slice_z = slice_z;
	bool new_frame = true;
	for(uint d=0u; d<lbm->get_D(); d++) new_frame = new_frame && lbm->lbm_domain[d]->graphics.enqueue_draw_frame(visualization_modes, field_mode, slice_mode, slice_x, slice_y, slice_z, visualization_change);
	for(uint d=0u; d<lbm->get_D(); d++) lbm->lbm_domain[d]->finish_queue();
	int* bitmap = lbm->lbm_domain[0]->graphics.get_bitmap();
	int* zbuffer = lbm->lbm_domain[0]->graphics.get_zbuffer();
	for(uint d=1u; d<lbm->get_D()&&new_frame; d++) {
		const int* const bitmap_d = lbm->lbm_domain[d]->graphics.get_bitmap(); // each domain renders its own frame
		const int* const zbuffer_d = lbm->lbm_domain[d]->graphics.get_zbuffer();
		for(uint i=0u; i<camera.width*camera.height; i++) {
#ifndef GRAPHICS_TRANSPARENCY
			const int zdi = zbuffer_d[i];
			if(zdi>zbuffer[i]) {
				bitmap[i] = bitmap_d[i]; // overlay frames using their z-buffers
				zbuffer[i] = zdi;
			}
#else // GRAPHICS_TRANSPARENCY
			bitmap[i] = color_add(bitmap[i], bitmap_d[i]);
#endif // GRAPHICS_TRANSPARENCY
		}
	}
	camera.allow_labeling = new_frame; // only print new label on frame if a new frame has been rendered
	return bitmap;
}

void LBM::Graphics::set_camera_centered(const float rx, const float ry, const float fov, const float zoom) {
	camera.free = false;
	camera.rx = 0.5*pi+((double)rx*pi/180.0);
	camera.ry = pi-((double)ry*pi/180.0);
	camera.fov = clamp((float)fov, 1E-6f, 179.0f);
	camera.set_zoom(0.5f*(float)fmax(fmax(lbm->get_Nx(), lbm->get_Ny()), lbm->get_Nz())/zoom);
}
void LBM::Graphics::set_camera_free(const float3& p, const float rx, const float ry, const float fov) {
	camera.free = true;
	camera.rx = 0.5*pi+((double)rx*pi/180.0);
	camera.ry = pi-((double)ry*pi/180.0);
	camera.fov = clamp((float)fov, 1E-6f, 179.0f);
	camera.zoom = 1E16f;
	camera.pos = p;
}
bool LBM::Graphics::next_frame(const ulong total_time_steps, const float video_length_seconds) { // returns true once simulation time has progressed enough to render the next video frame for a 60fps video of specified length
	const uint new_frame = to_uint((float)lbm->get_t()/(float)total_time_steps*video_length_seconds*60.0f);
	if(new_frame!=last_exported_frame) {
		last_exported_frame = new_frame;
		return true;
	} else {
		return false;
	}
}
void LBM::Graphics::print_frame() { // preview current frame in console
#ifndef INTERACTIVE_GRAPHICS_ASCII
	camera.rendring_frame.lock(); // block rendering for other threads until finished
	camera.key_update = true; // force rendering new frame
	int* image_data = draw_frame(); // make sure the frame is fully rendered
	Image* image = new Image(camera.width, camera.height, image_data);
	info.allow_printing.lock();
	println();
	print_image(image);
	info.allow_printing.unlock();
	delete image;
	camera.rendring_frame.unlock();
#endif // INTERACTIVE_GRAPHICS_ASCII
}
void encode_image(Image* image, const string& filename, const string& extension, std::atomic_int* running_encoders) {
	if(extension==".png") write_png(filename, image);
	if(extension==".qoi") write_qoi(filename, image);
	if(extension==".bmp") write_bmp(filename, image);
	delete image; // delete image when done
	(*running_encoders)--;
}
void LBM::Graphics::write_frame(const string& path, const string& name, const string& extension, bool print_preview) { // save current frame as .png file (smallest file size, but slow)
	write_frame(0u, 0u, camera.width, camera.height, path, name, extension, print_preview);
}
void LBM::Graphics::write_frame(const uint x1, const uint y1, const uint x2, const uint y2, const string& path, const string& name, const string& extension, bool print_preview) { // save a cropped current frame with two corner points (x1,y1) and (x2,y2)
	camera.rendring_frame.lock(); // block rendering for other threads until finished
	camera.key_update = true; // force rendering new frame
	int* image_data = draw_frame(); // make sure the frame is fully rendered
	const string filename = default_filename(path, name, extension, lbm->get_t());
	const uint xa=max(min(x1, x2), 0u), xb=min(max(x1, x2), camera.width ); // sort coordinates if necessary
	const uint ya=max(min(y1, y2), 0u), yb=min(max(y1, y2), camera.height);
	Image* image = new Image(xb-xa, yb-ya); // create local copy of frame buffer
	for(uint y=0u; y<image->height(); y++) for(uint x=0u; x<image->width(); x++) image->set_color(x, y, image_data[camera.width*(ya+y)+(xa+x)]);
#ifndef INTERACTIVE_GRAPHICS_ASCII
	if(print_preview) {
		info.allow_printing.lock();
		println();
		print_image(image);
		print_info("Image \""+filename+"\" saved.");
		info.allow_printing.unlock();
	}
#endif // INTERACTIVE_GRAPHICS_ASCII
	running_encoders++;
	thread encoder(encode_image, image, filename, extension, &running_encoders); // the main bottleneck in rendering images to the hard disk is .png encoding, so encode image in new thread
	encoder.detach(); // detatch thread so it can run concurrently
	camera.rendring_frame.unlock();
}
void LBM::Graphics::write_frame_png(const string& path, bool print_preview) { // save current frame as .png file (smallest file size, but slow)
	write_frame(path, "image", ".png", print_preview);
}
void LBM::Graphics::write_frame_qoi(const string& path, bool print_preview) { // save current frame as .qoi file (small file size, fast)
	write_frame(path, "image", ".qoi", print_preview);
}
void LBM::Graphics::write_frame_bmp(const string& path, bool print_preview) { // save current frame as .bmp file (large file size, fast)
	write_frame(path, "image", ".bmp", print_preview);
}
void LBM::Graphics::write_frame_png(const uint x1, const uint y1, const uint x2, const uint y2, const string& path, bool print_preview) { // save current frame as .png file (smallest file size, but slow)
	write_frame(x1, y1, x2, y2, path, "image", ".png", print_preview);
}
void LBM::Graphics::write_frame_qoi(const uint x1, const uint y1, const uint x2, const uint y2, const string& path, bool print_preview) { // save current frame as .qoi file (small file size, fast)
	write_frame(x1, y1, x2, y2, path, "image", ".qoi", print_preview);
}
void LBM::Graphics::write_frame_bmp(const uint x1, const uint y1, const uint x2, const uint y2, const string& path, bool print_preview) { // save current frame as .bmp file (large file size, fast)
	write_frame(x1, y1, x2, y2, path, "image", ".bmp", print_preview);
}
#endif // GRAPHICS



void LBM_Domain::allocate_transfer(Device& device) { // allocate all memory for multi-device trqansfer
	ulong Amax = 0ull; // maximum domain side area of communicated directions
	if(Dx>1u) Amax = max(Amax, (ulong)Ny*(ulong)Nz); // Ax
	if(Dy>1u) Amax = max(Amax, (ulong)Nz*(ulong)Nx); // Ay
	if(Dz>1u) Amax = max(Amax, (ulong)Nx*(ulong)Ny); // Az

	transfer_buffer_p = Memory<char>(device, Amax, max(transfers*(uint)sizeof(fpxx), 17u), true, true, 0, false); // only allocate one set of transfer buffers in plus/minus directions, for all x/y/z transfers
	transfer_buffer_m = Memory<char>(device, Amax, max(transfers*(uint)sizeof(fpxx), 17u), true, true, 0, false); // these transfer buffers must not be zero-copy!

	kernel_transfer[enum_transfer_field::fi              ][0] = Kernel(device, 0ull, "transfer_extract_fi"              , 0u, t, transfer_buffer_p, transfer_buffer_m, fi);
	kernel_transfer[enum_transfer_field::fi              ][1] = Kernel(device, 0ull, "transfer__insert_fi"              , 0u, t, transfer_buffer_p, transfer_buffer_m, fi);
	kernel_transfer[enum_transfer_field::rho_u_flags     ][0] = Kernel(device, 0ull, "transfer_extract_rho_u_flags"     , 0u, t, transfer_buffer_p, transfer_buffer_m, rho, u, flags);
	kernel_transfer[enum_transfer_field::rho_u_flags     ][1] = Kernel(device, 0ull, "transfer__insert_rho_u_flags"     , 0u, t, transfer_buffer_p, transfer_buffer_m, rho, u, flags);
	kernel_transfer[enum_transfer_field::flags           ][0] = Kernel(device, 0ull, "transfer_extract_flags"           , 0u, t, transfer_buffer_p, transfer_buffer_m, flags);
	kernel_transfer[enum_transfer_field::flags           ][1] = Kernel(device, 0ull, "transfer__insert_flags"           , 0u, t, transfer_buffer_p, transfer_buffer_m, flags);
#ifdef FORCE_FIELD
	kernel_transfer[enum_transfer_field::F               ][0] = Kernel(device, 0ull, "transfer_extract_F"               , 0u, t, transfer_buffer_p, transfer_buffer_m, F);
	kernel_transfer[enum_transfer_field::F               ][1] = Kernel(device, 0ull, "transfer__insert_F"               , 0u, t, transfer_buffer_p, transfer_buffer_m, F);
#endif // FORCE_FIELD
#ifdef SURFACE
	kernel_transfer[enum_transfer_field::phi_massex_flags][0] = Kernel(device, 0ull, "transfer_extract_phi_massex_flags", 0u, t, transfer_buffer_p, transfer_buffer_m, phi, massex, flags);
	kernel_transfer[enum_transfer_field::phi_massex_flags][1] = Kernel(device, 0ull, "transfer__insert_phi_massex_flags", 0u, t, transfer_buffer_p, transfer_buffer_m, phi, massex, flags);
#endif // SURFACE
#ifdef TEMPERATURE
	kernel_transfer[enum_transfer_field::gi              ][0] = Kernel(device, 0ull, "transfer_extract_gi"              , 0u, t, transfer_buffer_p, transfer_buffer_m, gi);
	kernel_transfer[enum_transfer_field::gi              ][1] = Kernel(device, 0ull, "transfer__insert_gi"              , 0u, t, transfer_buffer_p, transfer_buffer_m, gi);
	kernel_transfer[enum_transfer_field::T               ][0] = Kernel(device, 0ull, "transfer_extract_T"               , 0u, t, transfer_buffer_p, transfer_buffer_m, T);
	kernel_transfer[enum_transfer_field::T               ][1] = Kernel(device, 0ull, "transfer__insert_T"               , 0u, t, transfer_buffer_p, transfer_buffer_m, T);
#endif // TEMPERATURE
}

ulong LBM_Domain::get_area(const uint direction) {
	const ulong A[3] = { (ulong)Ny*(ulong)Nz, (ulong)Nz*(ulong)Nx, (ulong)Nx*(ulong)Ny };
	return A[direction];
}
void LBM_Domain::enqueue_transfer_extract_field(Kernel& kernel_transfer_extract_field, const uint direction, const uint bytes_per_cell) {
	kernel_transfer_extract_field.set_ranges(get_area(direction)); // direction: x=0, y=1, z=2
	kernel_transfer_extract_field.set_parameters(0u, direction, get_t()).enqueue_run(); // selective in-VRAM copy
	transfer_buffer_p.enqueue_read_from_device(0ull, kernel_transfer_extract_field.range()*(ulong)bytes_per_cell); // PCIe copy (+)
	transfer_buffer_m.enqueue_read_from_device(0ull, kernel_transfer_extract_field.range()*(ulong)bytes_per_cell); // PCIe copy (-)
}
void LBM_Domain::enqueue_transfer_insert_field(Kernel& kernel_transfer_insert_field, const uint direction, const uint bytes_per_cell) {
	kernel_transfer_insert_field.set_ranges(get_area(direction)); // direction: x=0, y=1, z=2
	transfer_buffer_p.enqueue_write_to_device(0ull, kernel_transfer_insert_field.range()*(ulong)bytes_per_cell); // PCIe copy (+)
	transfer_buffer_m.enqueue_write_to_device(0ull, kernel_transfer_insert_field.range()*(ulong)bytes_per_cell); // PCIe copy (-)
	kernel_transfer_insert_field.set_parameters(0u, direction, get_t()).enqueue_run(); // selective in-VRAM copy
}
void LBM::communicate_field(const enum_transfer_field field, const uint bytes_per_cell) {
	if(Dx>1u) { // communicate in x-direction
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_transfer_extract_field(lbm_domain[d]->kernel_transfer[field][0], 0u, bytes_per_cell); // selective in-VRAM copy (x) + PCIe copy
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue(); // domain synchronization barrier
		for(uint d=0u; d<get_D(); d++) {
			const uint x=(d%(Dx*Dy))%Dx, y=(d%(Dx*Dy))/Dx, z=d/(Dx*Dy), dxp=((x+1u)%Dx)+(y+z*Dy)*Dx; // d = x+(y+z*Dy)*Dx
			lbm_domain[d]->transfer_buffer_p.exchange_host_buffer(lbm_domain[dxp]->transfer_buffer_m.exchange_host_buffer(lbm_domain[d]->transfer_buffer_p.data())); // CPU pointer swaps
		}
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]-> enqueue_transfer_insert_field(lbm_domain[d]->kernel_transfer[field][1], 0u, bytes_per_cell); // PCIe copy + selective in-VRAM copy (x)
	}
	if(Dy>1u) { // communicate in y-direction
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_transfer_extract_field(lbm_domain[d]->kernel_transfer[field][0], 1u, bytes_per_cell); // selective in-VRAM copy (y) + PCIe copy
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue(); // domain synchronization barrier
		for(uint d=0u; d<get_D(); d++) {
			const uint x=(d%(Dx*Dy))%Dx, y=(d%(Dx*Dy))/Dx, z=d/(Dx*Dy), dyp=x+(((y+1u)%Dy)+z*Dy)*Dx; // d = x+(y+z*Dy)*Dx
			lbm_domain[d]->transfer_buffer_p.exchange_host_buffer(lbm_domain[dyp]->transfer_buffer_m.exchange_host_buffer(lbm_domain[d]->transfer_buffer_p.data())); // CPU pointer swaps
		}
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]-> enqueue_transfer_insert_field(lbm_domain[d]->kernel_transfer[field][1], 1u, bytes_per_cell); // PCIe copy + selective in-VRAM copy (y)
	}
	if(Dz>1u) { // communicate in z-direction
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_transfer_extract_field(lbm_domain[d]->kernel_transfer[field][0], 2u, bytes_per_cell); // selective in-VRAM copy (z) + PCIe copy
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue(); // domain synchronization barrier
		for(uint d=0u; d<get_D(); d++) {
			const uint x=(d%(Dx*Dy))%Dx, y=(d%(Dx*Dy))/Dx, z=d/(Dx*Dy), dzp=x+(y+((z+1u)%Dz)*Dy)*Dx; // d = x+(y+z*Dy)*Dx
			lbm_domain[d]->transfer_buffer_p.exchange_host_buffer(lbm_domain[dzp]->transfer_buffer_m.exchange_host_buffer(lbm_domain[d]->transfer_buffer_p.data())); // CPU pointer swaps
		}
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]-> enqueue_transfer_insert_field(lbm_domain[d]->kernel_transfer[field][1], 2u, bytes_per_cell); // PCIe copy + selective in-VRAM copy (z)
	}
}

void LBM::communicate_fi() {
	communicate_field(enum_transfer_field::fi, transfers*sizeof(fpxx));
}
void LBM::communicate_rho_u_flags() {
	communicate_field(enum_transfer_field::rho_u_flags, 17u);
}
void LBM::communicate_flags() {
	communicate_field(enum_transfer_field::flags, 1u);
}
#ifdef FORCE_FIELD
void LBM::communicate_F() {
	communicate_field(enum_transfer_field::F, 12u);
}
#endif // FORCE_FIELD
#ifdef SURFACE
void LBM::communicate_phi_massex_flags() {
	communicate_field(enum_transfer_field::phi_massex_flags, 9u);
}
#endif // SURFACE
#ifdef TEMPERATURE
void LBM::communicate_gi() {
	communicate_field(enum_transfer_field::gi, sizeof(fpxx));
}
void LBM::communicate_T() {
	communicate_field(enum_transfer_field::T, 4u);
}
#endif // TEMPERATURE
#ifdef PARTICLES
void LBM::communicate_particles() {
	if(get_D()>1u) {
		if(initialized) {
			for(uint d=0u; d<get_D(); d++) lbm_domain[d]->particles.enqueue_read_from_device();
			for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue(); // domain synchronization barrier
			for(ulong n=0ull; n<lbm_domain[0]->particles.length(); n++) { // parallel_for(lbm_domain[0]->particles.length(), [&](ulong n) {
				for(uint d=1u; d<get_D(); d++) { // gather modified particle positions
					const float lbm_domain_d___particles_x_n_ = lbm_domain[d]->particles.x[n];
					if(as_uint(lbm_domain_d___particles_x_n_)!=0xFFFFFFFFu) { // particle was in domain d and has been modified
						lbm_domain[0]->particles.x[n] = lbm_domain_d___particles_x_n_;
						lbm_domain[0]->particles.y[n] = lbm_domain[d]->particles.y[n];
						lbm_domain[0]->particles.z[n] = lbm_domain[d]->particles.z[n];
						break; // particle can only be in one domain at a time, no need to check other domains once it has been found
					}
				}
			} // });
		}
		for(uint d=0u; d<get_D(); d++) { // broadcast unified particle positions, using pointer of lbm_domain[0] instead of memory copy
			float* lbm_domain_d_particles_data = lbm_domain[d]->particles.exchange_host_buffer(lbm_domain[0]->particles.data());
			lbm_domain[d]->particles.enqueue_write_to_device();
			lbm_domain[d]->particles.exchange_host_buffer(lbm_domain_d_particles_data);
		}
	}
}
#endif // PARTICLES