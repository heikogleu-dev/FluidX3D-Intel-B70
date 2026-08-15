# Implementierungsplan C1b Stufe 2 — Kernel-Eingriff „dominante Achse" mit Paar-Gate (R1) und Flächenfaktor (R2)

Alle Pfade beziehen sich auf /home/heiko/CFD/FluidX3D-v2, Stand 29ab0f8. Alle Zeilenangaben aus dem heutigen Code gelesen.

---

## A. Architektur-Festlegungen

**F1 — Datenlayout: AoS-Geometriepuffer (8 float/Facette) + uint-Indexfeld über die F-BBox, wiederverwendete `f_bbox()`-Mechanik.**
Das Indexfeld `fac_idx` bekommt exakt die Geometrie des F-Puffers: `f_bbox(n,&fbi)` (kernel.cpp:867–871) liefert bereits den kompakten Index über die Box; am Kanal ist die F-BBox die volle Domäne (kein `set_force_bbox`-Aufruf in `main_setup_kanal`, lbm.cpp:112–114 setzt dann fbnx=Nx — geprüft), am Fahrzeug ist sie Fahrzeug-BBox+4 (setup.cpp:1267–1273; M=4 deckt die wandnahen Fluidzellen sicher ab). AoS statt SoA für die Facettendaten (`fac_geo[8*fid+k]`: nx,ny,nz,yw,fac_a,achse,frei,frei), weil damit **kein Laufzeit-Stride als Kernelargument** nötig ist — die Facettenzahl steht zur Emissionszeit nicht fest. `fac_a = 1/|n̂_achse|` wird **host-seitig** als float vorberechnet (deterministisch, am Kanal exakt 1.0f, da |n̂_z|=1.0f).

**F2 — Kernel-Signatur: emissionsbedingte Zusatzargumente, Muster FORCE_FIELD/SURFACE/TS_P; Bindung nachträglich per `set_parameters` (Muster `finalize_sparse_tiles`).**
Der Fork löst positionsgebundene Argumente konsequent so: die Kernel-Signatur enthält die Argumente nur unter `#ifdef` (kernel.cpp:1649–1659), host-seitig werden sie in **derselben Reihenfolge** per `add_parameters` angehängt (lbm.cpp:258–321), `tile_slot` (TS_P) zwingend zuletzt (Kommentar lbm.cpp:307–310). FACETTEN reiht sich davor ein. Da die echten Puffergrößen erst nach Voxelisierung+`baue_facetten()` feststehen, werden in `allocate()` 1-Element-Platzhalter gebunden und später per `kernel_stream_collide.set_parameters(fac_param_pos, …)` ersetzt — exakt das erprobte Muster von `finalize_sparse_tiles()` (lbm.cpp:382–388, `get_number_of_parameters()` existiert: opencl.hpp:707). SPONGE (nur Defines, keine Puffer) taugt hier nicht als Vorbild, `alloc_coupling_planes` (lbm.cpp:335–347) nur fürs „nachträglich binden, sonst harter Fehler".

**F3 — Einfügestelle und Abtastzustand: identisch zur z-WFB.**
Der Facettenblock kommt **direkt hinter den WANDFUNKTION-Block** (kernel.cpp:1683–1689), d. h. nach `load_f` (1677) und `apply_moving_boundaries` (1680), vor `calculate_rho_u` (1692) — mit demselben Zellgate `flagsn_bo!=TYPE_S&&flagsn_bo!=TYPE_E&&flagsn_bo!=TYPE_MS`. u/ρ werden wie in `apply_wall_function` (1624–1625) per `calculate_rho_u(fhn,…)` aus den **Registern vor der Korrektur** gebildet (Hans Abtastpunkt). Beide Blöcke schließen sich per hartem Fehler gegenseitig aus, daher kein Konfliktfall am selben Platz. Nur fhn-Register, nie fi direkt (Risiko 3, Esoteric-Pull vom Gegenprüfer für x/y verifiziert).

**F4 — Äquivalenz per konstruierter Ausdrucksbaum-Identität; Abnahme „bitgleich angestrebt, bitnah mit Band akzeptiert".**
Kernpunkt: `def_fac_Y` wird mit **wörtlich derselben Emissionskette** wie `def_wf_Y` erzeugt (`to_string(0.5f/nu,8u)`, lbm.cpp:695), und der Kernel rechnet `Y = ut*((2.0f*yw)*def_fac_Y)`. Bei yw=0.5f (Fit liefert am Kanal exakt 0,5 — verifiziert) ist `2.0f*0.5f==1.0f` und `1.0f*def_fac_Y==def_fac_Y` **exakt**, der Ausdruck kollabiert bitgenau auf `ut*def_wf_Y`. Ebenso: `fac_a==1.0f ⇒ tw*fac_a==tw`, `fmin(tw,tw_max)==tw` (tw bereits geklemmt), Tangentialprojektion bei n̂=(0,0,1) exakt (utx==uxn, uty==uyn, utz==0.0f), Paarreihenfolge τ_x-Paar vor τ_y-Paar (Auflage 4). Einziges nicht beweisbares Restrisiko ist die **Compiler-FMA-Kontraktion** von `sqrt(utx*utx+uty*uty+utz*utz)` gegenüber `sqrt(uxn*uxn+uyn*uyn)` — deshalb Abnahmekriterium zweistufig (siehe Messpunkt b).

**F5 — τ-Akkumulator: Summe+Zähler je Facette, nur bei ≥1 getauschtem Paar (Auflage 3), physikalisches τ_w ohne Flächenfaktor.**
1 Zelle = 1 Work-Item, keine Atomics (A8): `fac_tau[fid] += tw_geklemmt; fac_tau_n[fid]++` nur wenn `getauscht>0`. Akkumuliert wird das **physikalische** τ_w (nach Klemme, vor Flächenfaktor), weil daraus y⁺ = √(τ/ρ)·y_w/ν gebildet wird; der Flächenfaktor gehört nur auf die **angewandten** ±½τ-Terme (R2). Float-Summe reicht (τ~1e-5 × ≤1e6 Schritte ⇒ Summe O(10), relativer Fehler unkritisch).

**F6 — Randstreifen-Wrap in `baue_facetten()` ist Stufe-2-Pflicht.**
Stand heute schließt die Kandidatenschleife x/y-Randstreifen aus (setup.cpp:395, dokumentiert als B5 „Diagnose-only, ~5 %"). Für den Wirkpfad kippt das: die z-WFB behandelt **alle** 2·Nx·Ny Wandzellen (periodisch), der Facettenpfad ließe die Streifen als BB stehen — Äquivalenz und Ist=Soll unmöglich. Fix: x/y-Schleifen auf 0..N−1 mit periodischem Modulo im Fenster; am Fahrzeug ungefährlich (Randebenen sind TYPE_S/TYPE_E, 0x41-Zellen liegen dank BBox+M nie am Rand). Die gewrappten Kanal-Facetten liegen alle exakt in der z=0,5-Ebene ⇒ weiterhin n̂=ê_z, y_w=0,500 exakt.

**F7 — Soll-Formel ist an ALLEN Fällen exakt, nicht nur am Kanal.**
Weil Flags statisch sind und der Lookup deterministisch, läuft der Facettenblock in **jedem** Schritt auf **genau** den N_aktiv hochgeladenen (unmarkierten) Facettenzellen. Slot 7 (gegatet t%100) hat daher überall das exakte Soll `N_aktiv × ((n_steps+99)/100)` — Formel-Muster setup.cpp:673. Am Kanal ist N_aktiv = 2·Nx·Ny (nach F6), am Fahrzeug/Kugel die vom Host gedruckte Zahl. „Ist>0"-Weichkriterien sind unnötig; der Anteil tatsächlich getauschter Paare wird separat sichtbar (Slot 11 + fac_tau_n).

---

## B. Die j-Index-Paartabelle (alle 6 Wandseiten)

Abgeleitet aus: Richtungstabelle in den feq-Kommentaren kernel.cpp:1100–1108 und `calculate_rho_u` D3Q19 kernel.cpp:1141–1143 (1=+00, 2=−00, 3=0+0, 4=0−0, 5=00+, 6=00−, 7=++0, 8=−−0, 9=+0+, 10=−0−, 11=0++, 12=0−−, 13=+−0, 14=−+0, 15=+0−, 16=−0+, 17=0+−, 18=0−+; Host-Kopie FZ_C setup.cpp:376–377). Gegenrichtungs-Paarung (i ungerade ↔ i+1) belegt durch `apply_moving_boundaries` kernel.cpp:1184–1188. **Ursprungsregel:** die nach dem Streaming in fhn[i] liegende DDF stammt vom Nachbarn j[ī] mit ī=opposite(i); z-Referenz: Boden-/Decken-Zeilen kernel.cpp:1638–1643 und R1-Wortlaut im Plan. Konvention wie im Kanal-Code: das Paarmitglied mit **positiver** Tangentialkomponente bekommt `= partner + 0.5f*tau_t`, das andere `= alt − 0.5f*tau_t`; τ_t hat durch `−…*ut_t/ut` bereits das Gegenvorzeichen zu u_t. Paarreihenfolge: tangentiale Achsen aufsteigend (x<y<z).

| Wandseite (Solid bei) | Facettenbedingung | Paar 1 (Tausch, τ-Komponente) | Gate Paar 1 (BEIDE solid) | Paar 2 | Gate Paar 2 |
|---|---|---|---|---|---|
| −x: `j[2]` | achse=0, n̂_x>0 | (7,13), τ_y: `f7=f13+½τy; f13=alt7−½τy` | `flags[j[8]]`, `flags[j[14]]` | (9,15), τ_z: `f9=f15+½τz; f15=alt9−½τz` | `flags[j[10]]`, `flags[j[16]]` |
| +x: `j[1]` | achse=0, n̂_x<0 | (14,8), τ_y: `f14=f8+½τy; f8=alt14−½τy` | `flags[j[13]]`, `flags[j[7]]` | (16,10), τ_z: `f16=f10+½τz; f10=alt16−½τz` | `flags[j[15]]`, `flags[j[9]]` |
| −y: `j[4]` | achse=1, n̂_y>0 | (7,14), τ_x: `f7=f14+½τx; f14=alt7−½τx` | `flags[j[8]]`, `flags[j[13]]` | (11,17), τ_z: `f11=f17+½τz; f17=alt11−½τz` | `flags[j[12]]`, `flags[j[18]]` |
| +y: `j[3]` | achse=1, n̂_y<0 | (13,8), τ_x: `f13=f8+½τx; f8=alt13−½τx` | `flags[j[14]]`, `flags[j[7]]` | (18,12), τ_z: `f18=f12+½τz; f12=alt18−½τz` | `flags[j[17]]`, `flags[j[11]]` |
| −z: `j[6]` (Kanal-Boden, kernel.cpp:1638–1640) | achse=2, n̂_z>0 | (9,16), τ_x | `flags[j[10]]`, `flags[j[15]]` | (11,18), τ_y | `flags[j[12]]`, `flags[j[17]]` |
| +z: `j[5]` (Decke, kernel.cpp:1641–1643) | achse=2, n̂_z<0 | (15,10), τ_x | `flags[j[16]]`, `flags[j[9]]` | (17,12), τ_y | `flags[j[18]]`, `flags[j[11]]` |

Alle Gate-Prüfungen mit der WFB-Maske `(flags[jX]&TYPE_BO)==TYPE_S` (kernel.cpp:1620–1621) — schließt TYPE_MS (0x03) und TYPE_E aus; damit sind bewegte Wände auch **linkweise** strukturell ausgenommen (Risiko 5). „4 Diagonalpaare pro Achse" = 2 Paare je Seite × 2 Seiten; die Seite wählt das Vorzeichen der dominanten n̂-Komponente (|n̂_a|≥1/√3, nie 0).

---

## C. Kernel-Block (Pseudocode, `apply_facette`)

Neue Funktion unter `#ifdef FACETTEN` direkt hinter `apply_wall_function` (Anker kernel.cpp:1646/1647); `wf_spalding_uplus` wandert von `#ifdef WANDFUNKTION` unter `#if defined(WANDFUNKTION)||defined(FACETTEN)` (unverändert bis auf `3u` → `def_wf_spalding_it`, Auflage 8, Default-Emission „3u" in beiden Armen — Werte bitidentisch).

```
void apply_facette(n, fhn, j, flags, fac_geo, fac_idx, fac_tau, fac_tau_n, hits, t) {
  uxx fbi; if(!f_bbox(n,&fbi)) return;                       // F-BBox-Mechanik wiederverwendet
  const uint fid = fac_idx[fbi]; if(fid==0xFFFFFFFFu) return; // markierte Zellen: reiner BB
  nx,ny,nz,yw,faca = fac_geo[8*fid+0..4]; achse = (uint)fac_geo[8*fid+5];
  float rhon,uxn,uyn,uzn; calculate_rho_u(fhn,&rhon,&uxn,&uyn,&uzn);   // wie WFB:1625
  const float und = nx*uxn+ny*uyn+nz*uzn;                    // bei n̂=êz: exakt uzn
  const float utx=uxn-und*nx, uty=uyn-und*ny, utz=uzn-und*nz; // êz: (uxn,uyn,0.0f) exakt
  const float ut = sqrt(utx*utx+uty*uty+utz*utz);
  float tw=0.0f, twe=0.0f;
  if(ut>=1e-6f) {
    const float Y  = ut*((2.0f*yw)*def_fac_Y);               // êz,yw=0.5f: == ut*def_wf_Y bitgenau
    const float up = wf_spalding_uplus(Y);
    const float utau = ut/up;  tw = rhon*utau*utau;
    const float tw_max = 0.5f*rhon*ut;                       // Klemme wie WFB:1633
    if(tw>tw_max) { tw=tw_max; atomic_inc(&hits[8]); }
    twe = fmin(tw*faca, tw_max);                             // R2-Flächenfaktor + Stabilitätsklemme;
                                                             // faca==1.0f ⇒ twe==tw bitgenau
  } else { atomic_inc(&hits[9]); }
  // τ-Komponenten NUR für die beiden Tangentialachsen der dominanten Achse, Baum wie WFB:1635f:
  //   achse==2: tau_x=-def_fac_tau*twe*utx/ut, tau_y=…uty…;  achse==0: (uty,utz); achse==1: (utx,utz)
  uint getauscht=0u;
  … Seitenwahl über Vorzeichen von n̂_achse, dann die zwei Paare laut Tabelle B,
    je Paar: if(beide Ursprünge (flags[j..]&TYPE_BO)==TYPE_S) { Tausch ±0.5f*tau_t; getauscht++; }
  if(getauscht>0u) { fac_tau[fid]+=tw; fac_tau_n[fid]++; }   // Auflage 3: nur echte Tausch-Zellen
  else if(t%100ul==0ul) atomic_inc(&hits[11]);               // Facette da, kein Paar offen (gegatet)
  if(t%100ul==0ul) atomic_inc(&hits[7]);                     // Wirkpfad, Soll exakt (F7)
}
```
Aufruf in `stream_collide` als eigener Block zwischen kernel.cpp:1689 und 1690, Gate identisch zu Zeile 1688. Latenter Kommentar an `update_fields` (kernel.cpp:2135–2137) um FACETTEN erweitern.

---

## D. Nummerierte Implementierungsschritte

**Schritt 1 — Schalter, Statiken, Guards (Host).**
Datei src/lbm.hpp: `static bool s_facetten; static float s_fac_tau;` neben s_wandfunktion (:150–151); Domänenfelder `bool facetten_on, facetten_bound; uint fac_param_pos; ulong fac_N;` + `Memory<float> fac_geo, fac_tau; Memory<uint> fac_idx, fac_tau_n;` + Deklaration `void bind_facetten(…)`; LBM-Ebene `void alloc_facetten(const std::vector<Facette>&)` neben `alloc_coupling_planes` (:567). `struct Facette` von src/setup.cpp:340 nach lbm.hpp verschieben (alloc_facetten braucht den Typ). Datei src/lbm.cpp Konstruktor: read-once `facetten_on=s_facetten` bei :112ff; Guards bei :122–128: `#ifndef D3Q19`-Fehler für FACETTEN; unbedingt: `s_facetten&&s_wandfunktion` → print_error; `s_facetten&&Dx*Dy*Dz>1` → print_error. CFD_FACETTEN 0/1/2 wird in den Setups gelesen (2 ⇒ s_fac_tau=0.0f, Muster setup.cpp:554). **Alle 6 Statik-Stellen** setzen: setup.cpp:552–555 (kanal: aktiv), :1013–1016 (kugel: aktiv), :1283–1287 (fahrzeug: false + Warnung „erst Stufe 5"), :1670–1673 und :1693–1696 (dd: false), :2277–2288 (fernfeld: false).

**Schritt 2 — Zählerpuffer 8→12.** src/lbm.cpp:254 `Memory<uint>(device, 12ull)`; Legende lbm.hpp:145 erweitern: [7] Facetten-Wirkpfad (t%100), [8] τ-Klemme (beide Klemmen), [9] u_t≈0-Skip, [10] Achskonflikt (reserviert Stufe 4, bleibt 0), [11] Facette ohne offenes Paar (t%100). Keine Signaturänderung, Kontrollarm bitgleich (Begründung steht schon im Kommentar :248–253).

**Schritt 3 — Emission.** src/lbm.cpp hinter :696: `+((s_facetten) ? "\n #define FACETTEN" "\n #define def_fac_Y "+to_string(0.5f/nu,8u)+"f" "\n #define def_fac_tau "+to_string(s_fac_tau,4u)+"f" : "")`; `def_wf_spalding_it` (Default „3u", env CFD_SPALDING_IT) in **beiden** Blöcken (WANDFUNKTION und FACETTEN) emittieren. Kontrollarm-Nachweis: `CFD_DUMP_CL`-Diff FACETTEN=0/WANDFUNKTION=0 gegen Vorstand **leer** (Dump-Mechanik lbm.cpp:200–205).

**Schritt 4 — Kernel-Signatur + Bindung.** src/kernel.cpp:1656–1658: neuer Block `#ifdef FACETTEN , const global float* fac_geo, const global uint* fac_idx, global float* fac_tau, global uint* fac_tau_n #endif` **nach** TEMPERATURE, **vor** TS_P. src/lbm.cpp `allocate()` nach :297, vor dem Sparse-Block :311: `if(facetten_on){ 1-Element-Platzhalter; fac_param_pos=kernel_stream_collide.get_number_of_parameters(); add_parameters(…4 Puffer…); }`. `LBM::alloc_facetten` (bei :1595, Muster alloc_coupling_planes): D>1 → Fehler; filtert klasse!=0; baut fac_geo (AoS, fac_a host-berechnet), fac_idx über die F-BBox der Domäne (fbx0…fbnz-Felder), fac_tau/fac_tau_n = 0; `write_to_device()`; `kernel_stream_collide.set_parameters(fac_param_pos,…)`; `facetten_bound=true`; druckt N_aktiv, Klassenausschlüsse und Puffer-MB. Harter Fehler in `LBM::run()` (lbm.cpp:1357, vor `initialize()`): facetten_on && !facetten_bound → print_error(„CFD_FACETTEN gesetzt, aber alloc_facetten() nie gerufen").

**Schritt 5 — Kernel-Block** wie Abschnitt C (src/kernel.cpp, Anker 1646/1689). Spalding-Literal 3u → def_wf_spalding_it (:1596).

**Schritt 6 — `baue_facetten`-Wrap (F6).** src/setup.cpp:395–416 und Glättungsfenster :467–470: x/y periodisch (Modulo), z-Schleife bleibt 1..Nz−2. Kanal-Anker-Erwartung im Log aktualisieren (Facettenzahl steigt auf 2·Nx·Ny; y_w=0,500/0 markiert MUSS bleiben).

**Schritt 7 — Verdrahtung Kanal.** src/setup.cpp:592–596 erweitern: bei `CFD_FACETTEN>0` immer `auto F=baue_facetten(…); lbm.alloc_facetten(F);` vor :597 `lbm.run(0u,…)`. CSV-Ungültigkeitsmarker :605 um FACETTEN erweitern (Auflage 2/AUDIT 1). Wirkpfad-Report nach Muster :670–677: Slots 7/8/9/11 lesen, **Soll = 2·Nx·Ny·((n_steps+99)/100), Ist≠Soll = harter Fehler**; zusätzlich fac_tau/fac_tau_n-Readback → mittleres τ_w, daraus y⁺-Median, Vergleich gegen f·δ. Feld-Hash (FNV über u-Bitmuster, env CFD_FELD_HASH) für den Bitvergleich einbauen.

**Schritt 8 — Test T1+T2 (Messpunkt a).**
T1 (reiner Host-Unit-Test, keine GPU): Funktion in setup.cpp, per `CFD_CASE=facetten_test` erreichbar. Sie leitet die Paartabelle **unabhängig** aus FZ_C ab (einlaufende Diagonalen = c_achse zeigt von der Wand weg; Ursprung = j[opposite]; Partner = gleiches c_achse, gespiegelte Tangentiale) und asserted sie gegen eine Host-Kopie der im Kernel hartkodierten Tabelle — fängt Transkriptionsfehler mechanisch. Danach synthetische Flagfelder (Ebene, 45°-Treppe, Innen-/Außenkante): Gate-Auswertung host-seitig; Assertions: Ebene ⇒ alle Paare offen; Treppen-Eckzelle ⇒ genau die Paare mit zwei soliden Ursprüngen offen (R1-Beispiel: y-Paar mit fluidem Ursprung bleibt zu); nie tauscht ein Paar mit fluidem Ursprung.
T2 (Mini-Domäne, erst CPU-OpenCL, dann iGPU): im selben Testfall 32×16×24, 45°-Treppenboden (solid wenn x+z<c, Periode über x), zwei LBM-Instanzen nacheinander: Arm AUS und Arm FACETTEN=2 (τ=0, reiner Tausch). Nach **genau 1 Schritt** rho/u lesen: Differenzen dürfen NUR an Zellen liegen, für die die T1-Gate-Vorhersage ≥1 getauschtes Paar liefert; alle Facettenzellen mit geschlossenen Gates müssen **bitgleich** zum AUS-Arm sein — das ist der DDF-Integritätstest an fluiden Ursprüngen (R1), ohne fi-Readback (fi hat keinen Hostpuffer, lbm.cpp:235). Zusätzlich Slot 7 == N_aktiv, Slot 11 == vorhergesagte Null-Tausch-Zellen; dann 100 Schritte Stabilität (RHO_CLAMP=0). Der 45°-Fall deckt damit das Paar-Gate VOR Stufe 3 ab; CFD_KANAL_KIPP bleibt Stufe 3.

**Schritt 9 — Verdrahtung Kugel (Messpunkt c).** setup.cpp kugel (nach Voxelisierung+Randsetzung, vor run): wand_flag `TYPE_S|TYPE_X` (Boden/Decke sind 0x01 und damit per Konstruktion Ausschluss), 0x41-Census-Assertion wie :1387–1391; Report wie Schritt 7 mit Soll=N_aktiv×gegatete Schritte; **beide Cd-Wege** in allen Armen protokollieren (Phantom-Zahl des Impulsaustauschwegs dokumentieren — Auflage 2, Pflichtzahl).

**Reihenfolge/Commits/Testleiter (Projektregel: committen VOR dem ersten GPU-Lauf):**
- **Commit 1** = Schritte 1–8 komplett + facetten_test. Messpunkte auf **CPU**: (i) DUMP-Diff Kontrollarm leer + Feld-Hash unverändert; (ii) T1 grün; (iii) T2-Diff-Lokalisierung; (iv) Kanal N=38 Kurzlauf (~200 Schritte): FACETTEN=1 vs WANDFUNKTION=1 Feld-Hash. 
- **iGPU-Kanal (Messpunkt b):** Kurzlauf-Bitvergleich + ein 20-ETT-Lauf: c_f-Reproduktion. **Abnahmekriterium (Festlegung):** primär bitgleich (Hash identisch); falls FMA-Kontraktion das letzte Bit dreht: „bitnah" mit Band |Δc_f|/c_f<1e-5 und |ΔU_b+|/U_b+<1e-5 über alle CSV-Zeilen, Ursache im Commit-Text benannt. Ist=Soll exakt, Slots 8/9/11 am Kanal = 0. → **Commit 2** (Messstand).
- **Commit 3** = Schritt 9; iGPU-Kugel drei Arme AUS/2/1: Wirkpfad Ist=Soll, τ-Klemme ~0, RHO_CLAMP=0, Cd-Phantom-Zahl. Optional FP32-Sprosse (FP16C aus) am Kugelfall (Risiko 7). 
- **B70 erst in Stufe 5** — Stufe 2 endet auf der iGPU.

---

## E. Risiken

1. **FMA-Kontraktion bricht Bitgleichheit** (F4) — Kriterium zweistufig festgelegt; falls nötig, Tangentialsumme testweise als `(utx*utx+uty*uty)+utz*utz` klammern und Hash erneut prüfen.
2. **Randstreifen-Wrap (F6) ändert Stufe-1-Zahlen** (Facettenzahl Kanal 13.456→2·Nx·Ny; Fahrzeug-Quote kann sich um Zehntelprozent verschieben) — Anker-Kriterium (y_w=0,500, 0 markiert) muss unverändert halten, sonst Stopp.
3. **Indexfeld am Kugelfall = volle Domäne** (keine F-BBox dort): 4 B/Zelle zusätzlich; falls iGPU-Speicher drückt, `set_force_bbox` um die Kugel setzen (Rückfallebene Risiko 10).
4. **Platzhalter-Rebind**: `set_parameters` an falscher Position ist die stille Fehlerklasse des Forks — Position ausschließlich über `get_number_of_parameters()` zur Add-Zeit, nie als Literal; T2 fängt Fehlbindung sofort (Slot-7-Soll).
5. **Doppel-Klemme** `fmin(tw*faca, tw_max)`: am Kanal beweisbar Identität, an Schrägen bewusst konservativ (bis √3-Kappung des Flächenfaktors) — Treffer laufen in Slot 8 und fallen auf.
6. **update_force_field-Phantom** an getauschten Links (Auflage 2) gilt ab dem ersten FACETTEN=2-Lauf — CSV-Kennzeichnung und Kugel-A/B sind Teil von Commit 1/3, nicht „später".
7. **FP16C**: Tausch exakt (reine Wertevertauschung), ±½τ klein gegen DDF-Auflösung — FP32-Gegenprobe am Kugelfall eingeplant.

### Critical Files for Implementation
- /home/heiko/CFD/FluidX3D-v2/src/kernel.cpp (WFB-Block 1584–1647, stream_collide 1649–1692, calculate_rho_u 1141, update_fields-Kommentar 2135)
- /home/heiko/CFD/FluidX3D-v2/src/lbm.cpp (Konstruktor/Guards 112–135, allocate 229–324, Emission 693–696, finalize_sparse_tiles-Muster 349–392, run 1355)
- /home/heiko/CFD/FluidX3D-v2/src/lbm.hpp (Statiken/Member 106–157, LBM-API 540–570)
- /home/heiko/CFD/FluidX3D-v2/src/setup.cpp (struct Facette/baue_facetten 340–521, kanal 523–681, kugel 954–1018, fahrzeug 1267–1394, dd 1646–1698, fernfeld 2277–2292, main_setup 2378)
- /home/heiko/CFD/FluidX3D-v2/FACETTEN-PLAN.md (A5–A8, R1/R2, Auflagen 3/4/8 — Abnahmereferenz)

---

# Messstand Stufe 2, Commits 1-2 (2026-08-15)

**T1/T1b (Host-Unit-Test):** unabhaengig aus FZ_C hergeleitete Paartabelle == Kernel-Tabelle
(alle 12 Paare); 45-Grad-Eckzelle: x-Paar offen, y-Paar zu (das R1-Beispiel des Gegenpruefers).

**T2 (Mini-Domaene 32x16x24, 45-Grad-Treppe, AUS vs NUR-TAUSCH):** Phase 1 nach 1 Schritt
bitgleich (Tausch wertgleich wegen Solid-Symmetrie -- dokumentiert); Phase 2 nach 2 Schritten:
512 Differenzen, ALLE an Zellen mit >=1 offenem Paar, **0 verbotene** -- R1 auf DDF-Ebene
verifiziert. Wirkpfad 1344 = fac_N exakt, 320 Zellen ohne offenes Paar (Treppenkanten), 64 Skips.

**Lehrstueck des Tages:** der erste Sichtbarkeits-Fix gab den Wandzellen u!=0 -- initialize()
machte damit alle Fluidnachbarn zu TYPE_MS, und der EIGENE MS-Guard sperrte den Facettenpfad
komplett aus (Wirkpfad 0). Eine Stunde Bisektion; der Konsolen-Zeilenumbruch verschluckte dabei
zeitweise die entscheidende Zahl. Beides steht jetzt als Kommentar im Testfall.

**Aequivalenznachweis (Messpunkt b) -- STRENGSTE Form erreicht: BITGLEICH, kein Toleranzband.**
- CPU (N=38, 316 Schritte): FELD-HASH(u) Facetten == z-WFB (12755646098055097704), AUS-Arm
  verschieden; Ub_lat/f_lat/cf_kraftbilanz spaltenidentisch.
- iGPU (N=38, 12666 Schritte): FELD-HASH(u) beide 7540097450125369907; Wirkpfad beide exakt
  1.798.320 = Soll; tau-Klemme/Skips/ohne-Paar alle 0. Die FMA-Sorge (F4/E1) trat nicht ein --
  der konstruierte Ausdrucksbaum kollabiert auch auf der iGPU bitgenau.
- cf_impulsaustausch wackelt im LSB (atomare Reduktion) -- als UNGUELTIG markiert, zaehlt nicht.

Naechster Schritt: Commit 3 = Kugel-Verdrahtung (Messpunkt c, drei Arme AUS/2/1 + beide Cd-Wege
als Pflichtzahl), danach Stufe 3 (gekippter Kanal 45 Grad + 26,6 Grad).

**Iron-Rule-Nachpruefung (Commits 419cd8a/3ba6ec5/1b08432): kein Befund oberhalb NIEDRIG.**
Positiv: alle 12 Paare unabhaengig verifiziert, Bitgleichheit KONSTRUKTIV garantiert (nicht
Glueck), Akkumulator-Hygiene und Bindung sauber, F6-Wrap Stufe-1-neutral. Nacharbeiten sofort
umgesetzt: Zweitklemmen-Zaehler (Slot 8 = beide Klemmen -- ab Stufe 3 haette sie lautlos
geklemmt), verwaistes FAC_PAAR-Define, tote bind_facetten-Deklaration, Debug-Readback,
T2-Phase-1 prueft alle 3 Komponenten. Nachweis: Tests weiter 3x gruen, Feld-Hash UNVERAENDERT
(12755646098055097704). OFFEN fuer Stufe 3: y-gespiegelte T2-Treppe (achse-1-Verhaltensnachweis),
Ein-Zellen-Spalt-Semantik als bewusste Entscheidung dokumentieren.
