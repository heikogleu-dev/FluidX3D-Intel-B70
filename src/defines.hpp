#pragma once



//#define D2Q9 // choose D2Q9 velocity set for 2D; allocates 53 (FP32) or 35 (FP16) Bytes/cell
//#define D3Q15 // choose D3Q15 velocity set for 3D; allocates 77 (FP32) or 47 (FP16) Bytes/cell
#define D3Q19 // choose D3Q19 velocity set for 3D; allocates 93 (FP32) or 55 (FP16) Bytes/cell; (default)
//#define D3Q27 // choose D3Q27 velocity set for 3D; allocates 125 (FP32) or 71 (FP16) Bytes/cell

//#define SRT // choose single-relaxation-time LBM collision operator; (default)
// ★ FORK 2026-08-08: TRT statt SRT. Bei SRT sind tau und Lambda=(tau-0.5)^2 gekoppelt; Lambda=3/16
// (die viskositaetsunabhaengige Bounce-Back-Wandposition, Ginzburg/d'Humieres PRE 68, 066614) verlangte
// tau=0.933, also Re_L=576 -- bei realistischem Re also prinzipiell unerreichbar. Gemessen lag Lambda
// beim Fahrzeug bei 7.7e-10, acht Zehnerpotenzen darunter; die effektive Wandposition wandert dann ins
// Fluid und die null- bis einzelligen Spalte an den Reifenaufstandsflaechen werden effektiv negativ breit.
// Belegt durch einen A/B mit Kontrollarm: bei tau=0.8 (Lambda=0.09) laeuft der Fall stabil (0 nan),
// bei tau=0.50003 divergiert er reproduzierbar (Fz -11.4 Mio N, 7 nan). TRT setzt Lambda fest auf 3/16,
// unabhaengig von nu UND vom Smagorinsky-nu_t.
#define TRT // choose two-relaxation-time LBM collision operator

//#define FP16S // optional for 2x speedup and 2x VRAM footprint reduction: compress LBM DDFs to range-shifted IEEE-754 FP16; number conversion is done in hardware; all arithmetic is still done in FP32
#define FP16C // optional for 2x speedup and 2x VRAM footprint reduction: compress LBM DDFs to more accurate custom FP16C format; number conversion is emulated in software; all arithmetic is still done in FP32

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
#if defined(REGULARIZED_BOUNDARIES)&&!defined(D3Q19)
#error "REGULARIZED_BOUNDARIES ist nur fuer D3Q19 gebaut (Gewichtszuordnung in reg_fneq und Achsnachbarn j[1..6]). Pruefer-Befund 2026-08-08: bei D3Q15 fehlt def_we, bei D3Q27 und D2Q9 sind die Gewichte falsch."
#endif
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

#if defined(FP16S) || defined(FP16C)
#define fpxx ushort
#else // FP32
#define fpxx float
#endif // FP32

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