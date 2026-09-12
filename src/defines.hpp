#pragma once



//#define D2Q9 // choose D2Q9 velocity set for 2D; allocates 53 (FP32) or 35 (FP16) Bytes/cell
//#define D3Q15 // choose D3Q15 velocity set for 3D; allocates 77 (FP32) or 47 (FP16) Bytes/cell
#define D3Q19 // choose D3Q19 velocity set for 3D; allocates 93 (FP32) or 55 (FP16) Bytes/cell; (default)
//#define D3Q27 // choose D3Q27 velocity set for 3D; allocates 125 (FP32) or 71 (FP16) Bytes/cell

#define SRT // choose single-relaxation-time LBM collision operator; (default)
// ★ FORK 2026-08-08: TRT statt SRT. Bei SRT sind tau und Lambda=(tau-0.5)^2 gekoppelt; Lambda=3/16
// (die viskositaetsunabhaengige Bounce-Back-Wandposition, Ginzburg/d'Humieres PRE 68, 066614) verlangte
// tau=0.933, also Re_L=576 -- bei realistischem Re also prinzipiell unerreichbar. Gemessen lag Lambda
// beim Fahrzeug bei 7.7e-10, acht Zehnerpotenzen darunter; die effektive Wandposition wandert dann ins
// Fluid und die null- bis einzelligen Spalte an den Reifenaufstandsflaechen werden effektiv negativ breit.
// Belegt durch einen A/B mit Kontrollarm: bei tau=0.8 (Lambda=0.09) laeuft der Fall stabil (0 nan),
// bei tau=0.50003 divergiert er reproduzierbar (Fz -11.4 Mio N, 7 nan). TRT setzt Lambda fest auf 3/16,
// unabhaengig von nu UND vom Smagorinsky-nu_t.
//#define TRT // choose two-relaxation-time LBM collision operator

#define FP16S // optional for 2x speedup and 2x VRAM footprint reduction: compress LBM DDFs to range-shifted IEEE-754 FP16; number conversion is done in hardware; all arithmetic is still done in FP32
//#define RHO_FP16 // ★ FORK 2026-09-12, TODO 2 Schritt 4: rho im GERAETE- und Hostspeicher als FP16S(rho-1)
// statt float32. Spart bei 4 mm 990 MiB VRAM im Nahfeld (519.139.485 Zellen x 2 B) und noch einmal
// dieselbe Menge System-RAM, weil die B70 KEIN Zero-Copy hat und rho dort zweimal liegt; im Fernfeld
// (iGPU, Zero-Copy) sind es 388 MiB einfach. LAUFZEIT ist KEIN Argument: nach Schritt 1
// (CFD_RHO_SPARSAM) traegt rho nur noch rund 40 MB je feinem Schritt = 0,1 % des Schrittverkehrs.
// Dies ist ein reiner KAPAZITAETS-Hebel.
//   Format: gespeichert wird rho-1, nicht rho. Bei rho ~ 1 hat half einen ULP von 9,8e-4 -- das ist
//   die Groessenordnung des Signals selbst. Auf rho-1 angewandt ist die Aufloesung relativ 2^-12.
//   AM ECHTEN FELD GEMESSEN (export/p4_neu/feld_nah_000501ms.vtk, 451.428.942 Fluidzellen):
//     FP16S(rho-1): Fehler RMS 3,32e-7, max 5,63e-5     half(rho) roh: RMS 1,82e-4, max 4,88e-4
//   Die Verschiebung um 1 ist also Faktor 550 im RMS, kein Stil.
//   EHRLICH DAZU: gegen den heutigen float32-Stand ist das ein VERLUST, kein Gewinn. float32 traegt
//   bei rho nahe 1 einen absoluten Boden von 5,96e-8; FP16S(rho-1) traegt |rho-1|*2^-12, bei
//   rho-1 = 1e-3 also 2,4e-7. Der Trick macht half ueberhaupt erst brauchbar -- mehr nicht.
//   KEIN UEBERLAUF: die Skalierung traegt bis |rho-1| = 1,999; RHO_CLAMP (unten) garantiert 0,5 und
//   das Tor im Kopplungs-Lift (kernel.cpp, v[0] in (0,5; 2,0)) garantiert 1,0. Marge Faktor 2.
//   Werkzeug zum Umschalten: werkzeuge/rho_format.sh FP32|FP16
#define U_FP16 // ★ FORK 2026-09-12, TODO 2 Schritt 4: u im GERAETE- und Hostspeicher als drei FP16S
// statt drei float32. Spart bei 4 mm 2971 MiB VRAM im Nahfeld (519.139.485 Zellen x 6 B) und noch
// einmal dieselbe Menge System-RAM (die B70 ist kein Zero-Copy-Geraet, u liegt dort zweimal); im
// Fernfeld (iGPU, Zero-Copy) sind es 1164 MiB einfach. u ist damit der GROSSE Hebel -- dreimal rho.
// Bandbreite je Zelle und Schritt: 123 -> 117 B, also -4,9 %. Zum Vergleich: rho war -1,6 % (123 -> 121)
// und hat davon -1,0 % Wanduhr eingeloest; die Erwartung fuer u ist rund -1,5 %, nicht mehr.
// (BERICHTIGT 12.09., Pruefagent: hier stand -3,3 % fuer rho. Das ist rhos ANTEIL an den 123 Byte,
// nicht seine Ersparnis -- gespart werden 2 der 4 Byte.)
//   FORMAT: FP16S OHNE Verschiebung -- Wort = half(u*2^15), zurueck = Wort*2^-15. KEINE Verschiebung
//   um u_lat, und das ist kein Versehen: u hat, anders als rho, keinen Sockel. Bei rho sitzt das
//   Signal auf einer 1 und der half-ULP dort ist 9,8e-4, also so gross wie das Signal -- deshalb
//   braucht rho die Verschiebung. u ist um 0 zentriert, die Aufloesung ist ueberall relativ 2^-11,
//   auch bei u_t = 0,005 an der Wand.
//   UND die Verschiebung wuerde den WORT-FIXPUNKT zerstoeren. Beide Skalen sind exakte Zweierpotenzen
//   und es gibt keinen Unterlauf (kleinstes Ergebnis 1,8e-12), also ist h*2^-15 bitgenau und
//   store_u(load_u(w)) == w. Mit den repo-eigenen Wandlern nachgerechnet: 0 Verletzungen ueber die
//   63.488 ENDLICHEN Bitmuster, auch nach acht Umlaeufen. Das ist STAERKER als bei rho, wo nur der WERT ein
//   Fixpunkt ist. NICHT ueber alle 65536 -- berichtigt 12.09. (Pruefagent, HOCH): die 2048 Woerter
//   mit Exponent 0x1F sind ausgenommen, und zwar wegen des Inf/NaN-Durchreichers in u_unpack, den
//   dieselbe Aenderung eingebaut hat. 2046 davon verletzen den Fixpunkt (zwei treffen zufaellig).
//   Der Entwurfsschluss haengt nicht daran: kein endliches u erreicht diese Woerter, die
//   Geschwindigkeitsklemme haelt +-0,57735. Die rho-Fassung hat dieselbe Ueberdehnung schon
//   einmal zuruecknehmen muessen (lbm.hpp, "ES IST KEIN WORT-FIXPUNKT").
//   Ein Rueckweg "+u_lat" wuerde runden, und Sterbenz traegt nur auf [u_lat/2; 2*u_lat]
//   = [0,0375; 0,15] -- das Totwasser (u -> 0) und die Beschleunigungszonen (bis 0,4764) liegen
//   ausserhalb. Daran haengen pruefe_slice_ebene ("Soll exakt 0") und apply_pressure_outlet, das in
//   JEDEM Schritt auf rund 300.000 Auslasszellen nichts als u[n] = u[m] tut.
//   KEIN UEBERLAUF, und zwar konstruktiv statt gemessen: kernel.cpp klemmt jede Komponente auf
//   +-def_c = 0,57735 VOR dem Speichern (stream_collide und update_fields, beide Zweige, mit
//   Wirkpfad-Zaehler Slot 28). Die Skalierung traegt bis 1,99902. Marge Faktor 3,46.
//   WAS ES KOSTET, und es gehoert angesagt statt entdeckt: u_lat = 0,075 ist als half NICHT exakt
//   darstellbar (Wort 0x68CD = 0,075012207). Die TYPE_E-Einlasszellen HALTEN diesen Wert, der
//   Freistrom liegt also um +0,0163 % hoeher und die Kraefte um +0,0326 %. Das ist systematisch,
//   nicht zufaellig -- und es liegt rund fuenfzigfach unter der Eigenstreuung von cd_rest.
//   Werkzeug zum Umschalten: werkzeuge/u_format.sh FP32|FP16
//#define FP16C // optional for 2x speedup and 2x VRAM footprint reduction: compress LBM DDFs to more accurate custom FP16C format; number conversion is emulated in software; all arithmetic is still done in FP32

//#define BENCHMARK // disable all extensions and setups and run benchmark setup instead

#define VOLUME_FORCE // enables global force per volume in one direction (equivalent to a pressure gradient); specified in the LBM class constructor; the force can be changed on-the-fly between time steps at no performance cost
#define FORCE_FIELD // enables computing the forces on solid boundaries with lbm.update_force_field(); and enables setting the force for each lattice point independently (enable VOLUME_FORCE too); allocates an extra 12 Bytes/cell
#define REGULARIZED_BOUNDARIES // ★ FORK 2026-08-08: TYPE_E-Raender setzen f = f_eq + f_neq statt nur f = f_eq.
// Der reine Gleichgewichts-Reset legt alle 19 Verteilungen fest, wo hoechstens 5 zulaessig sind, und
// verwirft damit jeden Schritt den gesamten Spannungstensor. Gemessen am leeren groben Kanal: die
// Stoerung entsteht in der ersten Fluidzelle hinter der Einlassebene, und bei w -> 2 klingt sie nicht
// ab, sondern wechselt jeden Schritt das Vorzeichen (Periode-2-Mode = die horizontalen Streifen im
// Schnitt). Zwei Randaenderungen, die nur die vorgeschriebenen GROESSEN tauschten, halfen nicht.
// f_neq kommt aus dem Scherratentensor ueber Differenzen des FELDES u[] -- nicht aus den Verteilungen
// des Nachbarn, weil die unter Esoteric Pull teilweise diesem selbst gehoeren.
// AUSKOMMENTIEREN stellt den alten Zustand bit-genau her (der Kontrollarm).
// Test B (CFD_SGS_WANDFREI) benutzt die festen Flaechennachbarn j[1..6] -- nur fuer D3Q19 gebaut.
// (Der Schalter selbst ist Laufzeit; dieser Guard schuetzt den Kernel-Code dahinter.)
#if defined(REGULARIZED_BOUNDARIES)&&!defined(D3Q19)
#error "REGULARIZED_BOUNDARIES ist nur fuer D3Q19 gebaut (Gewichtszuordnung in reg_fneq und Achsnachbarn j[1..6]). Pruefer-Befund 2026-08-08: bei D3Q15 fehlt def_we, bei D3Q27 und D2Q9 sind die Gewichte falsch."
#endif
// ★★ RHO_CLAMP -- Dichte-Limiter in calculate_rho_u, VOR der Division u = j/rho.
// Aus V1 nachgezogen 2026-08-09. V1 hat ihn am 2026-06-25 gegen den Druckdipol ueber dem bewegten
// Boden eingefuehrt; V2 hatte ihn nicht, und der dd-Lauf starb bei 0,003 s mit NaN.
// Grenzen physikalisch-universell (low-Ma: rho = 1 +- 0,02), triggert nur bei grober Instabilitaet.
// Wieviele Zellen er wirklich trifft, meldet der Waechter zur Laufzeit -- eine still greifende
// Klemme waere genau der lautlose No-op, den dieses Projekt jagt.
#define RHO_CLAMP
#define RHO_CLAMP_MIN 0.5f
#define RHO_CLAMP_MAX 1.5f
#define EQUILIBRIUM_BOUNDARIES // enables fixing the velocity/density by marking cells with TYPE_E; can be used for inflow/outflow; does not reflect shock waves
#define MOVING_BOUNDARIES // enables moving solids: set solid cells to TYPE_S and set their velocity u unequal to zero
//#define SURFACE // enables free surface LBM: mark fluid cells with TYPE_F; at initialization the TYPE_I interface and TYPE_G gas domains will automatically be completed; allocates an extra 12 Bytes/cell
//#define TEMPERATURE // enables temperature extension; set fixed-temperature cells with TYPE_T (similar to EQUILIBRIUM_BOUNDARIES); allocates an extra 32 (FP32) or 18 (FP16) Bytes/cell
// FORK -- UPDATE_FIELDS direkt einschalten. Upstream leitet es nur aus SURFACE/PARTICLES/GRAPHICS ab;
// ohne die drei schreibt stream_collide u und rho gar nicht, und beides ist dann nur so aktuell, wie
// der Host explizit update_fields() ruft. Der Druck-Auslass extrapoliert aber u, und die Slices zeigen
// u -- beide sahen ein bis CFD_SAMPLE_EVERY Schritte altes Feld. Jetzt schreibt stream_collide sie
// jeden Schritt selbst. Kosten: rund 16 Byte/Zelle/Schritt mehr Schreibverkehr, dafuer entfaellt der
// separate update_fields-Durchlauf. Netto etwa 10 bis 15 Prozent Durchsatz -- der Preis dafuer, dass
// Rand und Diagnose nie auf veralteten Daten arbeiten.
#define UPDATE_FIELDS
// ★ FORK 2026-08-08: SUBGRID war hier unmarkiert eingeschaltet und hat die geplante Validierung
// verfaelscht -- bei Re_D=1000 addiert Smagorinsky grob 30 bis 85 Prozent zur molekularen Viskositaet
// in der Kugelgrenzschicht, die effektive Reynoldszahl liegt dann bei 550 bis 750 statt 1000.
// Der Kommentar im Kugelfall sagt ausdruecklich, dort brauche es kein Turbulenzmodell.
//
// ★ GEMESSEN 2026-08-08, nachdem ich es abgeschaltet hatte: der Fahrzeugfall wird damit SCHLECHTER,
// nicht besser -- 869 nan statt 7, Kraefte auf exakt null. TRT und SUBGRID sind keine Alternativen,
// sondern zwei verschiedene Aufgaben: TRT fixiert die WANDPOSITION, SUBGRID liefert die BULK-
// DISSIPATION. Bei nu_lat = 9.4e-6 (Fahrzeug, 4 mm, nu = 1.51e-5) traegt Smagorinsky die gesamte
// Viskositaet -- die 9.25e-6 im vorigen Kommentar gehoerten noch zu nu = 1.48e-5; ohne ihn ist die
// Stroemung faktisch reibungsfrei. Es bleibt also AN.
// OFFEN und fallweise zu loesen: fuer die Kugel-Validierung gegen die Standard-Widerstandskurve bei
// Re_D = 100..1000 MUSS es aus, sonst misst man das SGS-Modell statt der Kugel (dort addiert es 30
// bis 85 Prozent zur molekularen Viskositaet). Das ist ein Fall-Schalter, kein globaler.
#define SUBGRID // enables Smagorinsky-Lilly subgrid turbulence LES model to keep simulations with very large Reynolds number stable
//#define PARTICLES // enables particles with immersed-boundary method (for 2-way coupling also activate VOLUME_FORCE and FORCE_FIELD; only supported in single-GPU)

//#define INTERACTIVE_GRAPHICS // enable interactive graphics; start/pause the simulation by pressing P; either Windows or Linux X11 desktop must be available; on Linux: change to "compile on Linux with X11" command in make.sh
//#define INTERACTIVE_GRAPHICS_ASCII // enable interactive graphics in ASCII mode the console; start/pause the simulation by pressing P
//#define GRAPHICS // run FluidX3D in the console, but still enable graphics functionality for writing rendered frames to the hard drive

#define GRAPHICS_FRAME_WIDTH 1920 // set frame width if only GRAPHICS is enabled
#define GRAPHICS_FRAME_HEIGHT 1080 // set frame height if only GRAPHICS is enabled
#define GRAPHICS_BACKGROUND_COLOR 0x000000 // set background color; black background (default) = 0x000000, white background = 0xFFFFFF
#define GRAPHICS_U_MAX 0.18f // maximum velocity for velocity coloring in units of LBM lattice speed of sound (c=1/sqrt(3)) (default: 0.18f)
#define GRAPHICS_RHO_DELTA 0.001f // coloring range for density rho will be [1.0f-GRAPHICS_RHO_DELTA, 1.0f+GRAPHICS_RHO_DELTA] (default: 0.001f)
#define GRAPHICS_T_DELTA 1.0f // coloring range for temperature T will be [1.0f-GRAPHICS_T_DELTA, 1.0f+GRAPHICS_T_DELTA] (default: 1.0f)
#define GRAPHICS_F_MAX 0.001f // maximum force in LBM units for visualization of forces on solid boundaries if VOLUME_FORCE is enabled and lbm.update_force_field(); is called (default: 0.001f)
#define GRAPHICS_Q_CRITERION 0.0001f // Q-criterion value for Q-criterion isosurface visualization (default: 0.0001f)
#define GRAPHICS_STREAMLINE_SPARSE 8u // set how many streamlines there are every x lattice points
#define GRAPHICS_STREAMLINE_LENGTH 128u // set maximum length of streamlines
#define GRAPHICS_RAYTRACING_TRANSMITTANCE 0.25f // transmitted light fraction in raytracing graphics ("0.25f" = 1/4 of light is transmitted and 3/4 is absorbed along longest box side length, "1.0f" = no absorption)
#define GRAPHICS_RAYTRACING_COLOR 0x005F7F // absorption color of fluid in raytracing graphics
#define GRAPHICS_LSF 4u // local box size for local memory optimization in graphics_flags_mc() kernel, possible values: 0u (disable local memory optimization), 4u (default, ~40% speedup), 8u (~40% speedup)
#define GRAPHICS_LSQ 8u // local box size for local memory optimization in graphics_q() kernel, possible values: 0u (disable local memory optimization), 4u (no speedup), 8u (default, ~10-90% speedup)
#define GRAPHICS_LSP 4u // local box size for local memory optimization in graphics_rasterize_phi() kernel, possible values: 0u (disable local memory optimization), 4u (default, ~40% speedup), 8u (~40% speedup)

//#define GRAPHICS_TRANSPARENCY 0.7f // optional: comment/uncomment this line to disable/enable semi-transparent rendering (looks better but reduces framerate), number represents transparency (equal to 1-opacity) (default: 0.7f)



// #############################################################################################################

#define TYPE_S 0b00000001 // (stationary or moving) solid boundary
#define TYPE_E 0b00000010 // equilibrium boundary (inflow/outflow)
#define TYPE_T 0b00000100 // temperature boundary
#define TYPE_F 0b00001000 // fluid
#define TYPE_I 0b00010000 // interface
#define TYPE_G 0b00100000 // gas
#define TYPE_X 0b01000000 // reserved type X
#define TYPE_Y 0b10000000 // reserved type Y

#define VIS_FLAG_LATTICE  0b00000001 // lbm.graphics.visualization_modes = VIS_...|VIS_...|VIS_...;
#define VIS_FLAG_SURFACE  0b00000010
#define VIS_FIELD         0b00000100
#define VIS_STREAMLINES   0b00001000
#define VIS_Q_CRITERION   0b00010000
#define VIS_PHI_RASTERIZE 0b00100000
#define VIS_PHI_RAYTRACE  0b01000000
#define VIS_PARTICLES     0b10000000

#if defined(SURFACE) && defined(SPARSE_TILES)
#error SURFACE x SPARSE_TILES: die SURFACE-Kernel rufen load_f/store_f ohne TS_A -- Kombination nicht gebaut (Gross-Audit N)
#endif

#if defined(FP16S) || defined(FP16C)
#define fpxx ushort
#else // FP32
#define fpxx float
#endif // FP32

// ★ TODO 2 Schritt 4: Speichertyp von rho. Host-Seite; die Geraeteseite bekommt rho_t/load_rho/
// store_rho als JIT-Define (lbm.cpp, neben den fpxx-Makros). Ohne RHO_FP16 ist rhoxx float und
// jede Wandlung die Identitaet -- der Arm ist dann bitgleich zum Stand vor dieser Aenderung.
#ifdef RHO_FP16
#define rhoxx ushort
#else // RHO_FP16
#define rhoxx float
#endif // RHO_FP16

// ★ TODO 2 Schritt 4: Speichertyp von u. Host-Seite; die Geraeteseite bekommt velxx/load_u/store_u
// als JIT-Define (lbm.cpp, neben den fpxx- und rhoxx-Makros). Ohne U_FP16 ist velxx float und jede
// Wandlung die Identitaet -- der Arm ist dann bitgleich zum Stand vor dieser Aenderung.
// DER NAME: nicht u_t (kernel.cpp fuehrt u_t als Bezeichner in Kommentaren und der Wandmodellpfad
// rechnet mit einer Tangentialgeschwindigkeit dieses Namens) und erst recht nicht uxx -- das ist der
// INDEXTYP (lbm.cpp, uint oder ulong je Gittergroesse). Beim rho-Umbau hat genau diese Falle
// zugeschlagen: das Makro hiess zuerst rho_t, ueberschrieb eine lokale Variable gleichen Namens und
// brach den Geraeteuebersetzer mit -11 ab, in BEIDEN Armen.
#ifdef U_FP16
#define velxx ushort
#else // U_FP16
#define velxx float
#endif // U_FP16

// Die rho-Leser der nicht gebauten Erweiterungen stehen weiter auf "global float* rho" (kernel.cpp,
// hinter Geraete-#ifdef). Wer eine davon einschaltet, bekaeme einen Puffer als falschen Typ gelesen --
// Faktor 1e38 und unter -cl-finite-math-only ohne jede Diagnose. Deshalb hier hart statt dort still.
#if defined(RHO_FP16) && (defined(SURFACE) || defined(GRAPHICS) || defined(TEMPERATURE) || defined(PARTICLES) || defined(INTERACTIVE_GRAPHICS) || defined(INTERACTIVE_GRAPHICS_ASCII))
#error RHO_FP16 x SURFACE/GRAPHICS/TEMPERATURE/PARTICLES: deren rho-Leser sind nicht umgestellt (TODO 2 Schritt 4, 12.09.2026)
#endif

// Dasselbe fuer u, und die Lesermenge ist groesser als bei rho: SURFACE haelt vier Signaturen
// (average_neighbors_non_gas/_fluid, surface_0, surface_2), GRAPHICS dreizehn, PARTICLES eine.
// TEMPERATURE ist fuer u KEIN Blocker -- sein einziger u-Leser sitzt im ohnehin umgestellten
// initialize -- wird aber mitgenommen, damit die Bedingung wortgleich zur rho-Sperre bleibt.
#if defined(U_FP16) && (defined(SURFACE) || defined(GRAPHICS) || defined(TEMPERATURE) || defined(PARTICLES) || defined(INTERACTIVE_GRAPHICS) || defined(INTERACTIVE_GRAPHICS_ASCII))
#error U_FP16 x SURFACE/GRAPHICS/TEMPERATURE/PARTICLES: deren u-Leser sind nicht umgestellt (TODO 2 Schritt 4, 12.09.2026)
#endif

#ifdef BENCHMARK
#undef UPDATE_FIELDS
#undef VOLUME_FORCE
#undef FORCE_FIELD
#undef MOVING_BOUNDARIES
#undef EQUILIBRIUM_BOUNDARIES
#undef SURFACE
#undef TEMPERATURE
#undef SUBGRID
#undef PARTICLES
#undef INTERACTIVE_GRAPHICS
#undef INTERACTIVE_GRAPHICS_ASCII
#undef GRAPHICS
#endif // BENCHMARK

#ifdef SURFACE // (rho, u) need to be updated exactly every LBM step
#define UPDATE_FIELDS // update (rho, u, T) in every LBM step
#endif // SURFACE

#ifdef TEMPERATURE
#define VOLUME_FORCE
#endif // TEMPERATURE

#ifdef PARTICLES // (rho, u) need to be updated exactly every LBM step
#define UPDATE_FIELDS // update (rho, u, T) in every LBM step
#endif // PARTICLES

#if defined(INTERACTIVE_GRAPHICS) || defined(INTERACTIVE_GRAPHICS_ASCII)
#define GRAPHICS
#define UPDATE_FIELDS // to prevent flickering artifacts in interactive graphics
#endif // INTERACTIVE_GRAPHICS || INTERACTIVE_GRAPHICS_ASCII