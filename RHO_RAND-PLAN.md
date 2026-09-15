# RHO_RAND: Vorprüfung vor dem Bau (Planungsagent, 15.09.2026)

## 0 Fassung

- **Code-Fassung stimmt:** HEAD `6298941` auf master, `src/kernel.cpp` zuletzt in `1da809a` geändert. Der Arbeitsbaum ist sauber, untracked ist nur `wissensspeicher-tagesprotokoll/2026-09-15.md`.
- **Übergabedatei hat sich während der Analyse geändert.** `UEBERGABE-2026-09-14.md` ist gitignored (`.gitignore:50`) und wurde heute um 07:57 geändert. Oben kam der Block „UMGESTELLT 15.09." dazu, alle Zeilen rutschen um +7. §2.3/2.4 stehen jetzt in Z. 196–231, §3 in Z. 265–388. Der Inhalt von §3 ist unverändert.
- **Log-Belege weichen leicht ab:**
  - Die 3 129 185 Facetten stehen in `logs/p4_register.log:557`, nicht in Z. 561.
  - **2 641 897 Bandzellen stehen nicht in p4_register**, der Lauf hatte kein SGS_BAND. Die Zahl steht in `logs/p4eb_aus.log:381`. Das ist dasselbe Gitter (1689×661×465) mit derselben Facettenzahl.

## 1 Kernbefunde (entscheidungsrelevant, alle am Code belegt)

**K1 – In R2 liest heute niemand rho.** SISM und `sgs_fdwand` lesen nur u und flags, bei Lage 1 und beim Band (Details in 2a). `stream_collide` rechnet rho an Facetten- und Bandzellen lokal aus (kernel.cpp:3082). Der einzige Pufferleser im Wandbereich ist APG (kernel.cpp:2123). APG ist nur mit `FACETTEN_APG` übersetzt (lbm.cpp:1992) und heute zweifach hart gesperrt: lbm.cpp:230 (RHO_FP16) und setup.cpp:6067 (RHO_SPARSAM). **Ohne APG wäre R2 toter Speicher.**

**K2 – R2 = L1 ∪ L2 deckt APG nicht ab.** Das zeigt ein Gegenbeispiel unter 2b. APG liest alle 18 D3Q19-Nachbarn. Dafür bräuchte es eine 18-Link-Dilatation der Facettenmenge, keine 6-Flächen-Dilatation.

**K3 – Die Fernfeld-Entnahme darf nicht rekonstruiert werden.**
- Heute liest `extract_plane_macros` (kernel.cpp:4348, gerufen in setup.cpp:7984) rho(t_c) aus dem Puffer.
- Nach dem Schritt sind die Vor-Kollisions-Populationen überschrieben: `store_f` schreibt genau die Slots, die `load_f` gelesen hat (kernel.cpp:1501–1502 gegen 1520–1521).
- Eine Rekonstruktion liefert deshalb rho(t_c+1). Dieser Wert ginge über den Lift in die TYPE_E-Zellen des Nahfelds, **Kräfte wären nicht mehr bitgleich**.
- Folge: **RHO_RAND zunächst nur im Nahfeld.** Das Fernfeld liegt ohnehin im System-RAM (`vram-nur-nahfeld-zaehlt`).

**K4 – Die Rekonstruktion ist nicht überall gleich dem gespeicherten rhon.**
- **Facettenzellen:** `apply_facette_imem` ändert fhn (kernel.cpp:2975), bevor `calculate_rho_u` läuft (3082). Die Rekonstruktion liefert dort rho vor dem Wandmodell.
- **TYPE_S:** `stream_collide` schreibt dort nie (Ausstieg 2949), der Puffer trägt 1.0f. Die DDF-Summe an Solidzellen ist dagegen beliebig, weil die Nachbarn in deren Slots schreiben. Also muss dort fest 1.0f zurückkommen.
- **TYPE_MS:** braucht `apply_moving_boundaries` wie in kernel.cpp:2958 und 4068.
- Der Satz in §3.5 „bitgleich außer TYPE_E" ist deshalb falsch.

**K5 – Stiller Heap-Überlauf, wenn nur der Puffer schrumpft.**
- `Memory_Container` übernimmt N von `lbm->get_N()` (lbm.hpp:784), nicht aus dem Puffer. Bei D=1 greift er ohne Grenzprüfung zu (lbm.hpp:691–692).
- `lese_yslice_in_host` schreibt `rho.set(n, …)` an beliebige n (lbm.cpp:3200).
- **Die `Rho_Feld`-Fassade muss deshalb im selben Commit umgebaut werden wie die Puffergröße.**

**K6 – Die R1-Schreibmaske kann die heutige RHO_SPARSAM-Arithmetik bleiben.**
- Die Maske `(n%Nx)+2 >= Nx` (kernel.cpp:3198) braucht nur einen Modulo, keine Division.
- Leser außerhalb von TYPE_E liegen nur an po_interior (x = Nx−2) und vi_interior (x = 1). Beides folgt aus `collect_boundary_pairs`, lbm.cpp:2979–3008.
- Folge: `stream_collide` kostet so viel wie HEAD mit RHO_SPARSAM. Der Pufferinhalt ist nur an TYPE_E und an diesen Innenzellen gültig. Das stellt ein Wächter sicher.

**K7 – Nicht RHO_RAND, blockiert aber Heikos Konfiguration.**
- setup.cpp:6070 sperrt `CFD_U_SPARSAM × CFD_SGS_BAND` hart ab. p4_register lief mit U_SPARSAM=1 (log:5461: 77 % der u-Schreibvorgänge gespart).
- Die Begründung dort ist vermutlich zu weit gefasst: Das Band ist auf die F-BBox beschnitten (lbm.cpp:979), `sgs_fdwand` liest nur j[1..6] (kernel.cpp:4930). Die u-Maske deckt F-BBox ±2 ab (kernel.cpp:3239). **Ungeprüft.**

## 2 Antworten zu a)–e)

**a) Liest SISM bzw. `sgs_fdwand` rho?** Nein, weder als Puffer noch als Argument.
- Signatur kernel.cpp:4905–4908: `u, flags, gd_zellen, gd_N, fac_wfd` und unter SGS_SISM `t, fac_sb, rho_clamp_hits, sbar_out`.
- Bindungen: lbm.cpp:1352/1357 (Lage 1) und 1038/1039 (Band).
- `rho_clamp_hits` ist der uint-Zählerpuffer, keine Dichte. Der Rumpf (4923–4979) liest nur u (4933–4934).

Pufferleser an Facetten- oder Bandzellen:
1. APG: Parameter kernel.cpp:1991–1993, Lesen 2117–2125, Aufrufsplice 2985–2987.
2. `initialize` (1582) liest einmal überall.
3. `extract_plane_macros` (4348), wenn eine Slice-Ebene den Wagen schneidet.

Sonst keiner. Gegengeprüft über den Typ-Zensus lbm.cpp:341–346: Soll 13 × `global rhoxx* rho`, und alle 13 stehen in der Inventur unter 3.1. Kein rho lesen: `sgs_gdiag` 4790, `fac_nachbar_ab` 4982, `update_force_field` 4590, `kraft_facetten_gpu` 4725, `schale_*` 4446/4497, `boden_eq`/`einlass_eq` (lokales rho 4036).

**b) Deckt R2 die APG-Leser ab?** Nein.
- Facettenzelle = Fluidzelle mit mindestens einem der 18 Links zur Wand (setup.cpp:2893–2899). L2 ist eine 6-Flächen-Dilatation nur über Fluid, beschnitten auf die F-BBox (lbm.cpp:967–984).
- **Gegenbeispiel, konvexe Kante:** Körper = {x<0 ∧ z<0}.
  - F = (0,y,0) ist Facette, sie hat den Diagonal-Link zu (−1,y,−1).
  - Ihr Kantennachbar E = (1,y,1) hat keinen Solid-Link. Auch keiner seiner sechs Flächennachbarn (0,y,1), (1,y,0), (2,y,1), (1,y,2), (1,y±1,1) ist Facette.
  - Also liegt E weder in L1 noch in L2. APG liest aber rho[E], denn übersprungen wird nur TYPE_S (2120).
- **APG bräuchte:** N18(L1) ohne TYPE_S, also eine 18-Link-Dilatation der Tiefe 1. TYPE_MS und TYPE_E gehören dazu.
- Eine 6-Flächen-Dilatation reicht nur mit Tiefe 2 und nur rein geometrisch, also ohne die Bedingung eines Fluidpfads. Sie wäre dann größer als nötig.
- **|N18(L1)| steht in keinem Log.** Zu zählen im Zensus (C0 in Abschnitt 9).

**c) Aktualität, Schreiber, Format.**
- Für APG muss R2 jeden Schritt aktuell sein. Schreiber wäre `stream_collide` im UPDATE_FIELDS-Block.
- Zwei Fallstricke dabei:
  - Der rho-Schreibvorgang (3216) steht vor der Facetten- und Bandsuche (3265–3268). Wiederverwendung von `fdw_fid`/`band_bid` geht nur, wenn das Speichern dahinter wandert.
  - Die Slots 204/205 zählen genau einen Schritt. R2 bräuchte ein eigenes Zählerpaar.
- **Format:** Eine eigene R2-Region kann unabhängig von RHO_FP16 in FP32 liegen (4 B).
  - Das entfernt nur die Speicherquantisierung, bei |rho−1| = 1e-3 also 4,9e-7 (lbm.cpp:230).
  - Das DDF-Rauschen von rms 6,06e-6 (Übergabe §2.4) bleibt.
  - Die Sperre lbm.cpp:230 wäre dann auf „APG liest FP16" einzugrenzen. Das ist eine Entscheidung der APG-Linie.

**d) Index.**
- **Mit CFD_SGS_BAND=2** gäbe es für L1 ∪ L2 einen Index ohne Mehrspeicher: id = `fac_fid` oder `fac_N + band_fid`, beide Masken existieren.
- **Mit CFD_SGS_BAND=0** ist `band_idx` nur ein Platzhalter aus 2 Elementen (lbm.cpp:731). Eine eigene Maske kostet 8 B je 32 F-BBox-Zellen, also 40,0 MB (Rechnung unter 8).
- Da L1 ∪ L2 die APG-Menge ohnehin nicht trifft (b), **R2 nicht an die SGS-Bandliste koppeln.** Falls APG eine Region bekommt: eigene 18-Link-Dilatation mit eigener Maske, gebaut nach dem Muster von `alloc_sgs_band`. Die Obermengen-Regeln (x/y gewickelt, z nicht) kämen aus `alloc_f_liste`, unabhängig von CFD_SGS_BAND.

**e) Empfehlung: R2 nicht bauen, auch nicht als leeren Puffer.**
- Ohne APG liest niemand rho in R2 (K1). R2 wäre toter Speicher: 11,5 MB (FP16) bzw. 23,1 MB (FP32), bei BAND=0 plus 40,0 MB Maske, plus Schreibkosten je Schritt. Und APG würde es trotzdem nicht bedienen (b).
- „Anhängbar" heißt deshalb nur Architektur: Region-Fassade, Wächter je Region und alles außerhalb der Regionen über die Rekonstruktion.
- **Die richtige APG-Menge wird im Zensus (C0) gezählt.** Dann kann Heiko zwischen zwei Wegen der APG-Linie entscheiden:
  - **(a) Region:** N18(L1) in FP32 plus 40 MB Maske.
  - **(b) Keine Region:** dp/ds aus den DDFs im eigenen Kernellauf. Kostet 4,23 B je Zelle und Schritt (Rechnung unter 8).

## 3 Vollständige rho-Inventur

### 3.1 Kernel (13 × `rhoxx`, Zensus lbm.cpp:345)

| Stelle kernel.cpp | Zugriff / Zellen | Klasse |
|---|---|---|
| `initialize` 1543/1582 | Lesen, alle Zellen, einmal | Schale aus R1, sonst Konstante 1.0f. Bitgleich, weil der Host nur `rho_pack(1)` sät (lbm.cpp:583) und kein Setup `rho.set` vor initialize ruft. Wort 0 → 1.0f exakt |
| `initialize` SURFACE 1611 | Lesen | harter Fehler (SURFACE) |
| `apply_facette_imem` APG 1991–1993, 2123, Splice 2985–2987 | Lesen, N18(Facette) | harter Fehler RHO_RAND × APG |
| `stream_collide` TYPE_E 3021, Wächter 3050/3051 | Lesen, TYPE_E auf den 5 Flächen | **R1** |
| `stream_collide` Schreiben 3216/3218 | Schreiben, Maske 3198 | **R1**, nur x ≥ Nx−2 (ggf. x < 2). `felder_voll` Bit 0 wirkungslos |
| `update_fields` 4124 | Schreiben, alle | toter Pfad (UPDATE_FIELDS: defines.hpp:114, lbm.cpp:1479–1484), aber gebunden (lbm.cpp:620). Rumpfsperre unter RHO_RAND |
| `po_reduce_mean` 4159 | Lesen `load_drho`, po_interior (x = Nx−2) | **R1** |
| `apply_pressure_outlet` 4243–4244 | Lesen m (x = Nx−2), Schreiben n (x = Nx−1) | **R1** |
| `apply_velocity_inlet` 4262 | Lesen m (x = 1), Schreiben n (x = 0) | **R1** (Nahfeld ohne VI, lbm.cpp:2820–2822) |
| `extract_plane_macros` 4348 | Lesen, Ebene | Nahfeld (Slices): **rekonstruieren**. Fernfeld-Kopplung: **Puffer**, nie rekonstruieren (K3) |
| `drive_boundary_cubic_lift` 4418 | Schreiben, TYPE_E der fp-Ebenen (Flächen, setup.cpp:6624–6629) | **R1** |
| `extract/insert_rho_u_flags` 5227/5234, `transfer_*` 5240/5246 | Halo | harter Fehler bei D>1 (nur bei D>1 gebunden, lbm.cpp:755) |
| SURFACE (3804/3895/3920) und GRAPHICS (5724 ff., lbm.cpp:2186–2199) | Lesen, `float*` | harter Fehler. Heute schließt defines.hpp:212 sie unter RHO_FP16 aus, mit dem FP32-Werkzeug wären sie wieder möglich |

### 3.2 Host lbm.cpp / lbm.hpp / info

| Stelle | Klasse |
|---|---|
| lbm.hpp:440 `Memory<rhoxx> rho`; lbm.cpp:583 Allokation | Größe der Schale (8 791 740 Zellen bei 4 mm) |
| lbm.hpp:842–854 `Rho_Feld`; lbm.cpp:2433–2435 (D>1) und 2483–2485 | **umbauen** (K5): R1-Spiegel plus Ebenen-/Schicht-Cache, sonst harter Fehler |
| lbm.cpp:2738 `rho.enqueue_write_to_device` | R1-Spiegel |
| lbm.cpp:2788/2793/2854 `communicate_rho_u_flags` | bei D=1 wirkungslos (lbm.cpp:3645 ff.) |
| lbm.cpp:3070/3072/3108 po/vi-Bindungen | R1, dazu Wächter nach `collect_boundary_pairs` |
| lbm.cpp:3158–3172 `LBM::extract_plane_macros` | unter RHO_RAND zusätzlich Rekonstruktionskernel, rho nach `out[o+0]` |
| lbm.cpp:3179–3203 `lese_yslice_in_host`, `rho.set` 3200 | Ebenen-Cache |
| lbm.cpp:1456–1459 `felder_voll` / `rho_voll_zwang` | Bit 0 unter RHO_RAND ohne Wirkung |
| lbm.cpp:1972–1974 Emission | `RHO_RAND` emittieren; `RHO_SPARSAM` in derselben Domäne nicht |
| lbm.cpp:220/230 APG-Warnung/Sperre | Sperre RHO_RAND × APG danebenstellen |
| lbm.cpp:341–346 rho-Zensus; 374–375 u-Zensus | Soll 13→14 bzw. 19→20, falls der neue Kernel `rhoxx* rho` und `velxx* u` trägt |
| lbm.cpp:28–60 `bytes_per_cell_*`; Vorprüfung 2552; info.cpp:78 | Vorprüfung rechnet 2 B/Zelle rho (bei 4 mm 990 MiB zu viel). Unter RHO_RAND abziehen und die Schale buchen |
| lbm.cpp:3440/3455 Grafik | harter Fehler |

### 3.3 Host setup.cpp (nur fahrzeug_dd, Nahfeld)

| Stelle | Klasse |
|---|---|
| 7873/7929 Kopplungs-Verify (nur TYPE_E, Z. 7893) | **R1** (Lesevorgang 17,6 MB statt 990 MiB) |
| 8366 `lese_yslice_in_host(fNy/2)` | rekonstruieren (Ebene) |
| 8380 Sonde `rho.get`, x_f = 2/10 (7625), y = fNy/2 | Ebenen-Cache. **Punktliste entfällt**, die Sonde liegt auf der Slice-Ebene |
| 8363/8364 und 8386 `CFD_SLICE_PRUEF` (Vollpuffer) | harter Fehler, außer im Prüfmodus |
| 8374 Altpfad `CFD_SLICE_GPU=0` | harter Fehler |
| 1199–1206 `schreibe_vtk_feld`; Aufrufe 8416/8429/8549 | rekonstruieren (Schicht je z·stride) |
| 1117–1137 `pruefe_slice_ebene` | nur im Prüfmodus |
| 1508–1536 Bericht 204/205 (`rho_an` = rho_takt>0, 1515); 7270–7274 Taktprüfung | an `rho_rand_on` anpassen |
| 6061–6073 Lesestelle RHO_SPARSAM/APG | Lesestelle CFD_RHO_RAND mit allen Sperren |
| 6105 Statik-Symmetrie Fernfeld | `s_rho_rand = 0` explizit |
| Fernfeld: 7460/7984 Kopplung, 8025 Interface-Druck, 8065 Bilanz, 8386–8398 Slices, 8418/8431/8551 VTK | **unverändert** (Fernfeld ohne RHO_RAND) |
| Kugel 5113–5115, Einzelgitter 5542–5544 | unverändert (Schalter nie an) |

### 3.4 Was im Entwurf §3.1 fehlt oder falsch ist

1. „`extract_plane_macros` (Fernfeld) → aus fi rekonstruieren" widerspricht der Kraft-Bitgleichheit (K3).
2. `update_fields` taugt nicht als Rekonstruktionskernel: toter Pfad, mit `store_rho` an jedem n gebunden, und sein `t_last_update_fields`-Mechanismus fehlt unter UPDATE_FIELDS. **Neuer Kernel.**
3. K4 (Facetten, TYPE_S, TYPE_MS) und K5 (Container-N) fehlen.
4. Rho-Zensus, u-Zensus und VRAM-Vorprüfung fehlen.
5. FERN-BODENKLEMME legt TYPE_E ins Fernfeld-Innere (setup.cpp:6465–6468). „TYPE_E nur am Rand" gilt dort nicht. Unkritisch, solange das Fernfeld aus bleibt.
6. „~14,4 Mio Klemmstellen" sind **Klemmereignisse über den Lauf**, keine Zellen: log:5487 zeigt 14 434 460 Treffer (unten 8 708 731, oben 5 725 729). Die Pflicht zu `calculate_rho_u` bleibt richtig.
7. Die Übergabe nennt Log-Zeile 561 für die Facetten, richtig ist 557. Die Bandzahl steht nicht in p4_register (siehe 0).

## 4 Regionen und Index

**R1 = Domänen-Randschale, Dicke 2 an allen 6 Flächen, arithmetisch gepackt, 0 B Indexkarte.** Gepackt wird so:

- z < 2: `x + Nx·(y + Ny·z)`
- z ≥ Nz−2: `2NxNy + x + Nx·(y + Ny·(z−(Nz−2)))`
- dazwischen: `4NxNy + (z−2)·(4Nx + 4(Ny−4)) + ring(x,y)`, mit
  - ring = `x + y·Nx` für y < 2,
  - `2Nx + x + (y−(Ny−2))·Nx` für y ≥ Ny−2,
  - sonst `4Nx + 4(y−2) + (x<2 ? x : x−Nx+4)`.

Summe `4NxNy + (Nz−4)(4Nx+4(Ny−4))` = 8 791 740 bei 4 mm. Das stimmt exakt mit N − (Nx−4)(Ny−4)(Nz−4) überein (nachgerechnet). Der Helfer `rr_idx(n)` wird als Funktion in R() geschrieben, nicht als `#define` (Werkzeugfalle 3). Er rechnet nur an TYPE_E-, po-, vi-, Lift- und Init-Zugriffen.

**Reicht Dicke 2?**
- TYPE_E liegt auf x = 0, Nx−1, y = 0, Ny−1, z = Nz−1 (setup.cpp:6452). Belegt durch log:5464: 3 290 677 TYPE_E-Zellen.
- po_interior und vi_interior liegen in der 26er-Nachbarschaft einer Flächenzelle. Die Flächenzellen sind alle TYPE_E, also nicht Fluid, deshalb gilt Innenzelle x = Nx−2 bzw. x = 1 (lbm.cpp:2996–3008).
- Lift: fp-Ebenen sind Flächen.
- Fernfeld: Ränder genauso. **Aber:** Die Entnahmeebenen cp[] liegen im Inneren (setup.cpp:6618–6623) und bräuchten eine Region **R3** (4 Ebenen, 188 622 Zellen bei 16 mm). Dicke hilft da nicht. Dazu kommt die BODENKLEMME ab 2.
- Dicke 1 würde an y/z reichen. Uniform 2 kostet ~8,6 MB mehr und hält die Packung einfach (offene Frage 8).

**Schreibmaske:** x ≥ Nx−2 wie heute (K6). Der Setup-Wächter prüft, dass po_interior und vi_interior darin liegen, sonst harter Fehler.

**R2/R3:** nicht im ersten Durchgang (2e, K3).

## 5 Rekonstruktion

**Neuer Kernel `rho_rek_ebene`, Signatur im Hausmuster:**
- `)+R(kernel void rho_rek_ebene)+"("+R(const global fpxx* fi, const global rhoxx* rho, const global velxx* u, const global uchar* flags, const ulong t, global float* out, <plane_axis, origin_x/y/z, extent_a/b>, global uint* hits // )`
- danach `)+R( TS_P )+") {"+R(`

**Ablauf je Zelle:**
1. Index über `plane_cell_index` (kernel.cpp:4320).
2. Bei SPARSE_TILES: `is_dead_tile` → 1.0f.
3. TYPE_S → 1.0f.
4. TYPE_E → `load_rho(rho, rr_idx(n))`.
5. Sonst: `neighbors`, `load_f(n, fhn, fi, j, t TS_A)`, bei TYPE_MS `apply_moving_boundaries`, dann **`calculate_rho_u`** (inklusive RHO_CLAMP 1243).
6. `out[4·gid]` beschreiben, Besuchszähler.

Die Klemmzähler 0/1 stehen außerhalb von `calculate_rho_u` (3016/3089) und dürfen **nicht** mitkopiert werden.

**Zeitpunkt:** Der Host übergibt das aktuelle Domänen-t, also das t nach `increment_time_step`. Dann liest `load_f` genau das, was `stream_collide(t)` gleich lesen wird, also rho der nächsten Kollision vor dem Wandmodell.
- **t−1 wäre falsch:** Das liefert Σ f_post der eigenen Zelle mit getauschten Paaren (kernel.cpp:4489–4493). Das gehört als Negativtest in die Abnahme.
- Parität t%2 steckt in `load_f`. FP16S-Laden ist exakt: `vload_half·2^-15` (lbm.cpp:2008).
- Ausführung mit `enqueue_run` plus `finish_queue` vor dem Lesen (Zero-Copy-Falle, lbm.cpp:1586–1590).

**Formen:**
- **Ebene:** Slices über `LBM::extract_plane_macros`. Unter RHO_RAND überspringt `extract_plane_macros` den rho-Lesevorgang per Rumpf-`#ifdef`, danach füllt der neue Kernel `out[o+0]`.
- **Schicht:** VTK mit axis=2 je z·stride. Nutzt den Kopplungspuffer; `max_cp` muss dafür auf ≥ fNx·fNy wachsen, heute sind es 785 385 statt 1 116 429 (setup.cpp:7428–7446).
- **Punktliste:** entfällt (3.3).

**Deklarierte Abweichung der rho-Spalten** (Slices, Sonde, VTK):
- t+1 statt t;
- an Facettenzellen der Wert vor dem Wandmodell;
- am boden_eq-Band der Stand nach dem Reset (lbm.hpp:189 beschreibt heute den Stand davor).

Kräfte und u sind nicht betroffen: Kein Kraft- oder u-Pfad liest den inneren rho-Puffer (3.1).

## 6 Schnittstellen

- **RHO_FP16:** R1 im selben Speicherwort. Die Rekonstruktion liefert float. Wortvergleiche nur mit **gerätegepackten** Wörtern, der Hostpacker rundet anders (lbm.hpp:40–46).
- **RHO_SPARSAM:** RHO_RAND ersetzt es in der Domäne, der Kernelblock wird per `#ifdef RHO_RAND` / `#else` verschachtelt (Falle 6: kein `#elif defined`). Die Slots 204/205 behalten ihre Bedeutung.
- **RHO_SMBOX:** Muster für die Emission je Domäne. Statik `s_rho_rand` wird in `device_defines` gelesen, Domänenkopie wie lbm.cpp:625, vor dem Fernfeld-Bau genullt (setup.cpp:6105).
- **U_SPARSAM:** unberührt. Die Koordinatenrechnung kernel.cpp:3237 wird **nicht** mitbenutzt, sonst hängt RHO_RAND an U_SPARSAM, und das ist mit BAND=2 gesperrt (K7).
- **Sparse Tiles:** `TS_P`, `is_dead_tile`, `add_parameters(tile_slot)` nur bei `sparse_on` (Falle 8, Muster lbm.cpp:746). p4_register lief ohne Tiling.
- **D=1:** Pflicht, sonst harter Fehler.
- **Hostspiegel:** R1-Spiegel 17,6 MB statt 990 MiB. Die B70 arbeitet nicht mit Zero-Copy.
- **Fernfeld (iGPU, System-RAM):** aus. Später nur mit R3 für die Entnahmeebenen, Gewinn 399 MB System-RAM.
- **N2F/F2N:** `schale_extract` liest nur u; der Lift schreibt R1; Kopplungs-Verify liest R1.
- **VTK-Spitze:** +5,3 MB (Puffer), sonst 0.
- **CFD_SGS_BAND 0 gegen 2:** für RHO_RAND ohne Belang, weil kein rho-Leser (a). Konflikt nur mit U_SPARSAM (K7).

## 7 Abnahme und Diagnostik

**Setup-Wächter** an der Lesestelle setup.cpp, jeder mit Negativtest (Falle 12):
1. Jede TYPE_E-Zelle liegt in R1.
2. Jede po- und vi-Innenzelle liegt in der Schreibmaske und in R1.
3. Jede fp-Ebene liegt in R1.
4. Harte Sperren: D>1, APG, `CFD_SLICE_GPU=0`, `CFD_SLICE_PRUEF` ohne Prüfmodus, UPDATE_FIELDS nicht definiert, SURFACE/GRAPHICS, RHO_SPARSAM gleichzeitig gesetzt.
5. „Jede APG-gelesene Zelle in R2" wird zur Sperre RHO_RAND × APG.

**Ungegatete, sättigende Zähler** (lbm.cpp:618 nennt 216 als nächsten freien Slot; per grep kein Literal 216–223; vorher alle berechneten Indizes dumpen, Falle 8b):
- **216:** R1-Zugriff außerhalb der Schale, im TYPE_E-Zweig und im Lift, Soll 0.
- **217:** Rekonstruktionsbesuche. **Ist = Soll** gleich der Summe der Ebenenzellen, nicht nur > 0.
- **218:** davon aus R1 (TYPE_E).
- po/vi/init haben keinen Zählerpuffer. Dort beweist der Host-Wächter über die statischen Listen, das ist ehrlicher als ein Zähler.

**Host-Zugriffssperre:** `Rho_Feld::get/set` außerhalb von Cache und R1-TYPE_E → `print_error`. Zähler „aus Cache / aus R1" im Bericht am Funktionsende.

**Prüfmodus `CFD_RHO_RAND_PRUEF`** (nur 8 mm oder Kugel):
- Zusätzlicher Vollpuffer `pruef_rho`. Der Name darf nicht mit `rho` beginnen, sonst zählt der Zensus-Teilstring mit.
- Abgriff in genau dem Schritt nach der Rekonstruktion über `felder_voll` Bit 2, also ohne neuen Parameter.
- Vergleich der Rekonstruktion (gerätegepackt) gegen `pruef_rho` auf dem Host.
- Soll: 0 Abweichungen außer TYPE_E, Facettenzellen getrennt ausgewiesen.

**Gegen die Falle „Rückleser im Kernel misst nichts":**
- Rekonstruktion und Speichern laufen in verschiedenen Kernelläufen und in verschiedene Puffer.
- Vergleich auf **iGPU und B70**, nicht nur auf der CPU.
- Negativtest mit t−1: Abweichungen müssen > 0 sein.
- Zusätzlich ein Host-Replikat an den Sondensäulen: rohe fi-Wörter sammeln, auf dem Host dekodieren, in derselben Reihenfolge summieren. Bitgleichheit zu erwarten, weil die Summe keine Multiplikation enthält. **Ungeprüft.**

## 8 Zwei Zahlen (RECHNUNG aus Gittermaßen und Log, nicht gemessen)

**VRAM Nahfeld:**

| | 4 mm (1689×661×465) | 3,75 mm (1801×709×497, Übergabe §5.1) |
|---|---:|---:|
| R1 Schale | 8 791 740 Zellen = **17,58 MB** | 10 049 468 = **20,10 MB** |
| gespart | **1020,7 MB = 973,4 MiB** VRAM, dazu gleich viel System-RAM | **1249,1 MB = 1191,3 MiB** |
| R2 = L1 ∪ L2 (nicht bauen) | 5 771 082 (3 129 185 aus log:557, 2 641 897 aus p4eb_aus.log:381) = 11,54 MB FP16 / 23,08 MB FP32; Index 0 (BAND=2) oder 40,03 MB (BAND=0: 160 106 544/32 × 8 B) | ~13,1 / ~26,3 MB (dx⁻²), Maske ~48,6 MB (dx⁻³), ungeprüft |
| VTK-Puffer | +5,3 MB | +6,0 MB |

**Fernfeld** (System-RAM, nicht empfohlen): 16 mm Schale 4 201 408 = 8,40 MB, gespart 398,6 MB. 15 mm: 4 782 492 = 9,56 MB, gespart 484,4 MB.

**Bandbreite je Zelle und Schritt.** Bezug sind 115 B, die Formel aus lbm.cpp:54–74; das Log passt dazu (log:3151: 2266 MLUPs ↔ 261 GB/s).
- **rho-Schreiben heute** (p4_register, RHO_SPARSAM): Schicht 305 117 × 2 B / 519,1 Mio = 0,0012 B, plus Vollschreiben 452 531 307 × 2 B / 60 = 0,0291 B, **zusammen 0,0303 B**.
- **RHO_RAND:** 0,0012 B. **Unterschied −0,029 B = −0,025 %.**
- TYPE_E-Lesen unverändert 0,0127 B.
- Slice-Rekonstruktion: 785 385 × 39 B / 3000 Schritte (log:819) = 0,00002 B.
- VTK: 20,2 GB je Dump, etwa **0,34 Schritt-Äquivalente**. PCIe 2,08 GB (float) statt heute 1,04 GB.
- Zum Vergleich:
  - R2 schreiben: 0,022 B (FP16) / 0,044 B (FP32).
  - Maskentest ohne Wiederverwendung: 1,23 B (+1,1 %).
  - APG-Weg (b): 18 × 39 B je Facette = 4,23 B (+3,7 %).

## 9 Baureihenfolge (kleine Commits, jeder einzeln abnehmbar)

**C0 – nur Host (Wächter, Sperren, Zensus):**
- Lesestelle `CFD_RHO_RAND` (Nahfeld), alle Sperren und Wächter aus 7, **R2-Zensus** (|L1|, |N18(L1) ohne S|, |N18(L1) ohne L1∪L2|).
- `CFD_RHO_RAND=1` endet danach mit „Kernelteil fehlt", also kein stiller No-Op.
- Negativtests: APG gesetzt; SLICE_GPU=0; Testhaken setzt eine TYPE_E-Zelle ins Innere.
- Ohne Kernelstring, also auf der freien GPU über die Queue.

**C1 – Rekonstruktionskernel ohne RHO_RAND:**
- `rho_rek_ebene`, Zensus 13→14 und 19→20.
- Prüfinstrument im Kugelfall: bei T rekonstruieren, dann `run(1)`, dann vollen rho-Puffer lesen, Wörter vergleichen. Negativtest t−1.
- Leiter: CPU (Gerät 0, `nice -n 19`, `CFD_KUGEL_DX=40`) → `scratch_gate.sh` offline → iGPU (2) → B70 (1). Vorher committen.

**C2 – R1 im Nahfeld:**
- Schalengröße, `rr_idx`, alle R1-Pfade per Rumpf-`#ifdef`, Schreibmaske, Slots 216–218.
- `Rho_Feld`-Fassade, Slices und VTK über die Rekonstruktion, Verify über R1.
- Bericht, VRAM-Vorprüfung, `max_cp`.
- Leiter: CPU mit `CFD_DEV_FINE=0 CFD_DEV_COARSE=0`, klein → iGPU `2/2` → B70 `1/2`.

**C3 – dd-Prüfmodus:** `pruef_rho` und Host-Replikat, auf iGPU und B70.

**Danach 8 mm über die Queue.** Paar HEAD gegen RHO_RAND mit sonst identischer Env, gerätekonsistent:
- `cmp` auf forces.csv und cd_facetten.csv, u-Spalten von Sonde und Slice.
- rho-Spalten mit deklarierter Verschiebung.
- fdinfo-VRAM, MLUPs.
- Determinismus-Probe: zwei identische Kurzläufe.
- Das dd-Setup wählt die Geräte über `CFD_DEV_FINE`/`CFD_DEV_COARSE` (setup.cpp:5922–5923). Diese explizit setzen, das Argument `CFD_QUEUE_DEV` allein reicht **ungeprüft**.

**4 mm nur mit Go:** VRAM (Soll ≈ −973 MiB) und Index. Bitanker gegen p4_register nur, wenn dieselbe Binärfassung belegt ist (ungeprüft).

## 10 Fallen und offene Fragen

**Fallen (knapp):**
1. K5: Container-N führt zu Heap-Überlauf.
2. Zensus-Teilstrings (`rho_*`, `unear`-Lehre, lbm.cpp:349–357).
3. Klammerfalle: Signatur nur im Muster `)+"("+R(` … `)+R( TS_P )+") {"+R(`; `#ifdef` nur auf Anweisungsebene; kein `//` vor Code in einer Zeile; kein `#if defined(A)&&…`; kein `#define` in R().
4. Guard-Konsistenz bei `tile_slot`.
5. Zero-Copy: `finish` vor dem Lesen.
6. t−1 statt t.
7. Host-Packer ≠ Gerätepacker.
8. TYPE_S, TYPE_MS, Facettenzellen (K4).
9. Klemmzähler 0/1 nicht in den neuen Kernel kopieren.
10. `print_error` beendet den Lauf: Abnahmen gehören ans Funktionsende.
11. `update_fields` ist gebunden und schreibt an jedes n.
12. `felder_voll` Bit 0 unter RHO_RAND ohne Wirkung.
13. CPU-Sprosse beweist das Instrument nicht.
14. IGC-Scratch beim neuen Kernel mit fhn[19]/j[19].

**Offene Fragen an Heiko:**
1. **Nur Nahfeld?** Das Fernfeld spart nur System-RAM und braucht R3. Empfehlung: ja, nur Nahfeld.
2. **rho-Ausgabe als float oder als gerätegepacktes FP16-Wort** (formatgleich zur Baseline)? Empfehlung: float.
3. **Deklarierte Abweichung akzeptiert?** rho(t+1), an Facetten vor dem Wandmodell.
4. **Schreibmaske nur x-Schichten?** Andere Schalenzellen wären dann im Puffer veraltet, gelesen wird dort aber nie.
5. **RHO_RAND und CFD_RHO_SPARSAM gleichzeitig gesetzt:** harter Fehler (Empfehlung) oder implizit ersetzen?
6. **Sperre U_SPARSAM × SGS_BAND (setup.cpp:6070)** prüfen lassen (K7)? Sonst kostet BAND=2 den u-SPARSAM-Gewinn.
7. **APG-Weg (a) Region oder (b) DDFs im eigenen Kernellauf?** Nach dem Zensus aus C0. RHO_RAND blockiert keinen der beiden.
8. **R1 uniform Dicke 2 (17,6 MB) oder flächenweise minimal (~9 MB)?** Empfehlung: uniform.

### Critical Files for Implementation
- /home/heiko/CFD/FluidX3D-v2/src/kernel.cpp
- /home/heiko/CFD/FluidX3D-v2/src/lbm.cpp
- /home/heiko/CFD/FluidX3D-v2/src/lbm.hpp
- /home/heiko/CFD/FluidX3D-v2/src/setup.cpp
- /home/heiko/CFD/FluidX3D-v2/src/defines.hpp

## 11 Nachtrag Hauptsitzung 15.09.: Probezellen (Heiko) — in C1 aufgenommen

Der Nachtrag an den Planungsagenten (Probezellen, FP16) ist im Bericht nicht verarbeitet, deshalb steht er hier.

**Vorgabe Heiko:** An wenigen gezielten Zellen soll sichtbar werden, welches rho dort gespeichert ist und was die Rekonstruktion liefert.

**Einordnung in den Plan:** Das gehört in **C1**. Dort wird rekonstruiert, ohne dass RHO_RAND aktiv ist, der volle Puffer steht also noch zum Vergleich da. Der Commit bleibt bitgleich und nur lesend.

**Welche Zellen:** Zellklassen auf der Rekonstruktionsebene (Slice y = fNy/2); ein zusätzlicher Kernel ist dafür nicht nötig.
- frei (Nachlauf)
- TYPE_E am Einlass (x = 0), am Deckel (z = Nz−1) und am Kopplungsrand
- po_interior (x = Nx−2)
- Facettenzelle Lage 1
- Bandzelle Lage 2
- APG-Kantennachbar (N18(L1) \ (L1∪L2), Gegenbeispiel 2b)
- boden_eq-Band
- TYPE_S und TYPE_MS (Rad)
- eine Zelle, deren rho_roh außerhalb von [0,5; 1,5] liegt, auf dem Host aus der Ebene gesucht

Je Klasse 1–3 Zellen, Index im Log.

**Was je Zelle ausgegeben wird:**
- das gespeicherte Wort und seinen dekodierten Wert (`load_rho`)
- `rho_roh` = Σ f ohne Klemme
- rho über `calculate_rho_u` mit Klemme
- das Gerätewort der Rekonstruktion
- Rekonstruktion zu t gegen gespeichertes rho aus `stream_collide(t)`, plus Negativtest t−1
- alle Differenzen

`rho_roh` braucht einen Prüfmodus-Ausgang des Kernels: im Prüfmodus trägt `out[o+1]` das rohe Σ f statt u.

**Erwartung aus dem Code (ungeprüft, genau das sollen die Probezellen zeigen):**
- Freie Zelle und po_interior: Das Gerätewort ist bitgleich.
- Facettenzelle: Die Rekonstruktion liefert den Wert vor dem Wandmodell, also eine Abweichung (K4).
- TYPE_S: Rückgabe 1,0.
- Klemmzelle: roh ≠ geklemmt, geklemmt = gespeichert.

**FP16 (Nachtrag):** Mit K3 erledigt. Die Fernfeld-Kopplung liest weiter den quantisierten Puffer und wird nie rekonstruiert. Die Nahfeld-Rekonstruktion fließt nur in Ausgaben, und die Wortvergleiche laufen gerätegepackt (Abschnitt 6).

## 12 Entscheidungen Heiko 15.09.2026 (zu §10)

1. **Nur Nahfeld:** ja. **Später auch Fernfeld**, dann mit einer eigenen Region R3 für die Entnahmeebenen (K3).
2. **rho-Ausgabe:** float.
3. **Deklarierte Abweichung** der rho-Spalten ist akzeptiert: rho(t+1), an Facetten vor dem Wandmodell.
4. **R1:** überall Dicke 2.
5. **RHO_RAND × RHO_SPARSAM:** **Entscheidung B** (Prüfbefund M3). RHO_RAND ersetzt die rho-Schreibmaske
   NUR im Nahfeld, das Fernfeld behält seine Maske. Umgesetzt an der Lesestelle (s_rho_takt Nahfeld = 0,
   mit Ansage).
6. U_SPARSAM × SGS_BAND (K7): nicht beantwortet, bleibt offen.
7. APG-Weg (a/b): offen. Grundlage ist der C0-Zensus bei 8 mm: Die APG-Lesemenge umfasst 1 502 474 Zellen,
   davon liegen 150 578 nicht in L1+L2.

## 13 Nachträge aus der C1-Prüfung (15.09.2026)

* **Deklarierte Abweichung, ergänzt zu §5:** Die TYPE_E-Auslasszellen (po_cells) hinken einen Schritt nach.
  do_time_step ruft Einlass, Auslass und dann stream_collide auf (lbm.cpp:3022–3024). Der gelesene Pufferwert
  stammt deshalb aus dem vorigen Schritt, während die Fluidzellen einen Schritt voraus sind (t+1). Im Fernfeld
  mit CFD_FERN_VI gilt dasselbe für die vi-Zellen.
* **K4 präzisiert:** Wandmodelle, die fhn vor calculate_rho_u umschreiben, weichen nur ab, wenn sie die Masse
  NICHT erhalten. iMEM mit FAC_ALPHA≥1 erhält sie analytisch. Echte Abweichungen entstehen bei FAC_ALPHA=0 und
  bei der ELIBB-Blende mit q≠0,5 (ELIBB ist am Fahrzeug Pflicht).
* **Falle für C2 (N7):** alloc_rho_rek bindet fi zum Aufrufzeitpunkt; die Rebind-Liste in finalize_sparse_tiles
  enthält den Kernel nicht. Vor dem Tiling-Umbau aufrufen oder in die Liste aufnehmen. Eine
  initialized-Prüfung wie bei alloc_coupling_planes fehlt.
* **Für C3/dd (N8):** Keine vollen rho-Reads, das wären bei 4 mm 4 GiB je Read. Stattdessen Ebenen-Gather.
* **Probezellen an der Kugel:** Einlass, Deckel, Auslass, po_interior, freie Nachlaufzelle, erste TYPE_S, TYPE_MS
  und Facette, dazu Abweichler und Klemmzellen. **Bandlage 2, APG-Kantennachbar und boden_eq-Band** gibt es an
  der Kugel nicht, sie folgen in C3 im dd-Fall.

## 14 Messbefund 15.09. vormittags: Die t+1-Rekonstruktion taugt NICHT als Ausgabe, die Nachkollisionssumme schon

Gemessen an der Kugel, 16 mm, B70, t0 = 500 Schritte (früher Anlauf, t ≈ 0,02 s); rr_c1g_ku_b70 und
rr_c1g_fac_elibb_b70. Heute liest ein Slice den Puffer nach dem letzten Schritt (w_vor). Die Ausgabeabweichung
ist deshalb |x − w_vor| und nicht |x − w_nach|.

| Ausgabe-Kandidat | Fluid | TYPE_MS | Facette (ELIBB) |
|---|---|---|---|
| **Rekonstruktion t0** (Plan §5 bisher: rho der nächsten Kollision) | Median cp 0,21, max 8,9 | Median cp **4,1**, max 10,2 | Median cp 0,35, max 2,3 |
| **Nachkollisionssumme** (Parität t0−1: eigene Populationen nach der Kollision) | max cp **0,004** | max cp 0,004 | max cp **0,001** |

(cp = (2/3)·Δrho/u_lat², u_lat 0,075)

* **Die Annahme „ein Schritt ist physikalisch belanglos“ (§3.4, Falle 1) ist WIDERLEGT.** An Wandzellen gibt es
  eine Periode-2-Mode, die Rekonstruktion trifft die andere Phase.
* **Die Nachkollisionssumme** reproduziert den heutigen Slice-Wert bis auf FP16S-Rundung, auch an
  ELIBB-Facetten, weil das Wandmodell in den Nachkollisions-Populationen schon enthalten ist. Sie ist nicht
  bitgleich. §3.4 hatte sie als „keine bitgleiche Alternative" verworfen; numerisch ist sie die bessere
  Ausgabe.
* **Physikleser sind nicht betroffen:** TYPE_E, po_interior und der Lift lesen R1.
* **Vorbehalt:** früher Anlauf an der Kugel. Die Bestätigung im dd-Fall bei entwickelter Strömung gehört in C3.
