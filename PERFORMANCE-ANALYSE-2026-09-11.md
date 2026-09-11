# Kostenanalyse des gesamten Codes (11.09.2026)

Fünf unabhängige Agenten, getrennte Zuständigkeiten, HEAD `5a8250d`. Maßstab ist die
Produktionskonfiguration (`logs/STANDARD_4mm.txt`, `export/p4dt_deteps/code/LAUF.txt`).
Jede Zahl ist gemessen oder aus einer benannten Quelle; was nicht gemessen werden konnte,
steht als solches am Ende jedes Abschnitts.

## Messgrundlage

Instruktionszahlen aus dem Offline-Compiler (`werkzeuge/scratch_gate/igc_offline.sh`, ocloc,
**ohne GPU**), Defineliste als Kopie auf die Produktion umgestellt. Reproduzierbar, zweimal
identisch. **Geräte-IDs: `0xe223` = Arc Pro B70, `0x7d67` = iGPU.**

**Es laufen zwei verschiedene `stream_collide`:**

| | Gitter | Gerät | instCount | grf / private / spill |
|---|---|---|---|---|
| Nahfeld 4 mm | 1689×661×465 = 519,1 M | B70 | **6590** | 128 / 0 / 0 |
| Fernfeld 16 mm | 768×480×552 = 203,5 M | iGPU | **2219** | 128 / 0 / 0 |

Der Nahkernel trägt kein SPONGE, der Fernkernel keine Facetten, kein FDWAND, kein SISM.

**Wanduhr** (`logs/p4dt_deteps.log`, 5669 s): Zeitschleife 5503 s über 12 526 grobe Schritte
= 439,4 ms je Schritt; 166 s außerhalb. Verteilung: Nahfeld-Launches 92,3 %, Aufbau und
Abschluss 2,93 %, Fernfeld-Synchronisation und Ernte 2,62 %, Kopplung 0,86 %, Kräfte und CSV
0,75 %, Ausgabe 0,49 %.

**Speicher** (gegen das Lauflog verifiziert): 28 314 MiB, **57,19 B je Zelle**. DDF 66,4 %,
Felder 29,7 %, Facettenkette 2,05 %, reine Diagnostik 0,88 %.

---

## 1. Messbar teuer und ohne Beitrag zum Ergebnis

| Maßnahme | gemessener Gewinn | Risiko |
|---|---|---|
| **Volumenkraft im Fahrzeugfall nicht emittieren** | −132 Instr. = **2,0 %** Nahkernel, −204 = **9,2 %** Fernkernel | gering |
| **Diagnostik-Zähler hinter ein Emissionsgate** | −340 = **5,2 %** Nahkernel, −352 = **15,9 %** Fernkernel | mittel, s. u. |
| beides kombiniert, gemessen | Nah 6590 → **6129 (−7,0 %)**, Fern 2219 → **1714 (−22,8 %)** | |
| **`CFD_FAC_KDIAG=0`** | **−191 MiB** VRAM, −400 MB Schreibverkehr je Schritt | gering |
| **P-TRT-Zähler auf `t%1000`** | **−18,9 s** Wanduhr (0,33 %), A/B-belegt | keines |
| **Sponge-Zähler auf `t%100` gaten** | −39 Instr. = **1,8 %** Fernkernel | keines |

**Die Volumenkraft ist der ärgerlichste Posten.** Im Fahrzeugfall ist sie in beiden Gittern
exakt null (`logs/p4dt_deteps.log`: „Volume Force 0.00000000" zweimal; `set_fx` wird nur vom
Kanal-Regler gerufen, `setup.cpp:3844`). Weil die Kraft ein **Kernelargument** ist, kann der
JIT sie nicht wegoptimieren: 19 Guo-Terme, eine Reziproke und 19 Skalierungen je Fluidzelle
und Schritt für eine Kraft, die es nicht gibt.

**Zu den Zählern, ausdrücklich:** sie sind teuer UND sie sind der Nullbeweis. Vorschlag ist
deshalb ein Emissionsschalter mit **Default an**, nicht Löschen — und ein Produktionslauf ohne
Zähler nur, wenn zu derselben Konfiguration ein Zählerlauf vorliegt.

### ⚠ Der Gewinn ist kleiner als die Prozente suggerieren

Die Fernfeld-Ersparnis von 22,8 % schlägt **nicht** auf die Wanduhr durch. Das Fernfeld liegt
vollständig hinter dem Nahfeld verborgen (bei 8 mm kostet die ganze Fernfeld-Phase 0,68 ms bei
59 ms feiner Deckung; bei 4 mm ist die Deckung siebenfach). Zählbar ist allein der Nahanteil:
**−7,0 % Instruktionen**, und auch das ist ein statisches Maß, kein Laufzeitmaß.

---

## 2. Kostenlos, aber Wartungslast: toter Code

| Posten | Zeilen | Befund |
|---|---|---|
| Nie emittierte Kernelzweige (SGS_DIAG 73, VANDRIEST 43, NUT_SKAL 43, WANDFREI 16, TRT 51, REG 42, F-Read 15) | **283** | **0 Instruktionen** — gemessen, nicht geschätzt |
| TRT-Zweig gesamt (Kernel, Emission, Wächter, info, setup) | **96** | `defines.hpp:10` hat SRT aktiv. Erzwang die **doppelte Pflege des P-TRT-Blocks** am 10.09. |
| `main_setup_fahrzeug` + `main_setup_facetten_test` | **516** | nie in einer Seriendatei, nie gesichert |
| Tote Zweige in `apply_facette_imem` (MASSE_X als vollständige Zweitkopie der Kaskade, PEMA, APG, UW …) | **211 von 384** | MASSE_X muss bei jeder Kaskadenänderung „Zeichen für Zeichen" nachgezogen werden |
| GRAPHICS-Familie (`graphics.cpp`, `main.cpp`, `lbm.hpp`, `lbm.cpp`, `kernel.cpp`, `src/X11/`) | **~3000 + 1,8 MB** | Upstream-Code; Löschen macht künftige Merges teuer. **Nicht ohne Freigabe.** |
| `reset_force_field` | Kernel | steht unter `#ifdef PARTICLES`, das ist aus — wird bei jedem JIT mitkompiliert |
| Werkzeuge mit gelöschten Eingabedaten (`abl_dach/of13_*`, `of13_kraft_zband.py`, `interface_serie.py`) | 9 Dateien | brechen in der ersten Lesezeile ab |

## 3. Schalterinventar

186 echte `CFD_*`-Schalter: **49 in Produktion**, 74 je gemessen, **63 nie gesetzt**.

**Vier Schalter, die man setzen kann und die nichts tun** — die Fehlerklasse, die dieses
Projekt als hart einstuft:

- `CFD_LAMBDA` — `def_lambda` wird nur im TRT-Zweig emittiert. Ein Λ-A/B liefe **bitgleich**.
  Immerhin gefangen: `lbm.cpp:220-224` warnt beim Setzen.
- `CFD_SAT`, `CFD_FILL_VOIDS`, `CFD_TILE_WG` — **existieren im Quelltext nicht**, stehen aber
  im öffentlichen README, `CFD_TILE_WG` sogar mit einer Messzahl („1.43 GB freed at −12 %").
  Wer die Zeile ausführt, bekommt schweigend den Default.

Dazu: `CFD_REG_BC` wurde **nie gesetzt** — der ganze regularisierte Rand (63 Zeilen) hat nie
gerechnet, während `defines.hpp:28-38` ihn als geltenden Fork-Fix darstellt.

## 4. Die Facettenkette

Gemessen über den Offline-Compiler: `stream_collide` 7173 Instruktionen mit der Kette, 2602
als reines Bounce-Back. **Die Kette kostet 4571 Instruktionen = 64 %.** Kein Spill, kein
Scratch — sie kostet Instruktionen, keine Occupancy.

**Nachweislich tragend:** Vollrangzweige der Kaskade (51,3 % des Wirkpfads), SATGATE (Cz
−0,23 bis −0,30), Nachbarabtastung (c_f +66 %, 38 σ), Facetten-SISM (cz_druck_rest −0,1017,
10,3 σ).

**Belegt ohne Kraftwirkung:** PINV und DETEPS kaufen Abdeckung (70,5 → 81,8 → 93,85 %),
DETEPS kostet dabei Abtrieb.

**Teurer Ballast:** SGS-Band (119 MB, gepaart 1,5 σ), `fac_kd` (191 MiB reine Diagnose),
`CFD_FAC_UTKORR` (wirkt auf 0,072 % der Besuche, eigenständig als tot gemessen).

**Der größte ungeklärte Posten ist ELIBB:** 2073 Instruktionen = **28,9 %** von
`stream_collide`, läuft an 73,4 % der Facetten je Schritt — und die einzige saubere Messung
(26-Grad-Kanal) sagt u_tau 2,382 mit gegen 1,943 ohne, also **23 % schlechter**, bei
verdreifachten Gate-Rückfällen. Am Fahrzeug existiert **kein gepaartes A/B**.

**Und die Zahl, die über der ganzen Kette steht:** von den angewandten Wandbesuchen treffen
**61,7 %** ihr Ziel auf ±10 %; **32,3 % liegen um Dekaden daneben** (18,2 % r ≤ −10,
14,2 % r ≥ 10). Abdeckung zu kaufen, solange ein Drittel der Anwendungen um Dekaden
danebenliegt, ist die teuerste Stelle der Kette.

## 5. Host-Takt

**48 Kernel-Starts und 15 blockierende Synchronisationen je grobem Schritt.** Sechs von acht
Round-Trips in Kopplung und Ernte sind bündelbar (beide Warteschlangen sind ohnehin
in-order); Obergrenze des Hebels 3,1 % der Wanduhr.

**Die Überlappung ist vollständig.** Die README-Zahlen „CONCURRENT 96,1 %" und „far wait +
extract 0,3 %" stammen vom 20.08., also vor N2F-Schale und -Band; heute sind es 2,62 %, und
das ist **Ernte, kein Warten**.

**`boden_eq` startet 519,1 M Work-Items für 1,15 M getroffene Zellen** (0,221 %), viermal je
Schritt. Der teure Teil (Flag-Load vor dem z-Test) ist behoben, der nackte Dispatch ist **nie
gemessen** worden.

**Die Ausgabe ist kein Thema:** Slices 1,1 s, VTK 26,4 s, CSV im Kräfteblock — zusammen
0,49 %.

## 6. Speicher

Zwei Hebel über der 50-MiB-Schwelle, beide nicht in der Physik:

- **`CFD_FAC_KDIAG=0`: −191 MiB.** Sechzehn Float je Facette, nur am Laufende für eine
  Klassentabelle gelesen.
- **`kf_liste` filtern: −244 MiB.** Lädt 62,2 M Markerzellen hoch, obwohl das Kraftfeld nur
  3,74 M Slots hat; für 94 % der Einträge liefert der Lesezugriff konstruktiv null. **Nicht
  bitgleich** — die Gruppierung der Reduktion ändert sich.

**Die gedruckte Speicherbilanz ist um ~301 MiB zu optimistisch:** sie behauptet der
Spitzenwert zu sein, wird aber vor den späten Puffern gedruckt. Der Reserveposten „320 MB
Spätpuffer" ist damit bis auf ~19 MiB ausgereizt.

### ⚠ Ausdrückliche Gegenempfehlungen

- **SISM-EMA-Puffer NICHT auf half** (−66 MiB). Die Filterkonstante ist 1/5000 = 2,0·10⁻⁴,
  FP16 löst relativ nur ~9,8·10⁻⁴ auf. Jeder Einzelschritt läge unter einem ULP, der
  Mittelwert bliebe auf 0 stehen, SISM würde still zu klassischem Smagorinsky — **und der
  Wirkpfad-Zähler meldete trotzdem „angewandt"**.
- **`u` und `rho` NICHT auf half** (−3,9 GiB). Das Signal ist dort jeweils die kleine
  Differenz, die FP16 wegwirft: `sgs_fdwand` bildet |S| als finite Differenz benachbarter
  u-Werte, rho schwankt um 1 ± 10⁻³.

## 7. Was nicht gemessen werden konnte

- **Laufzeitanteil einzelner Kernelblöcke.** Die Instruktionszahlen sind statisch. Nur ~0,6 %
  der Zellen sind Facettenzellen, aber sie liegen geclustert: eine SIMD16-Subgroup mit einer
  Facettenzelle rechnet den ganzen Block. Die Divergenz ist nicht beziffert.
- **Gepaarte Kraftwirkung von ELIBB, KDIAG und ALPHA2 bei 4 mm** — für keinen der drei gibt
  es ein A/B auf der Produktionssprosse.
- **Dispatchkosten von `boden_eq`** — Rezept: 8-mm-Paar mit `CFD_BODEN_EQ=0/2`.
- **GPU-Auslastung** — braucht den fdinfo-Profiler an einem laufenden Prozess.
- **Atomic-Kontention der ungegateten Zähler.**

---

# Teil 2 — Architektur- und Vorberechnungsideen (11.09.2026)

Nicht aus den Agentenberichten, sondern danach am Code geprüft. Reihenfolge nach Hebel.

## A1 — Die halbe Facettenkette ist Geometrie und ist einmalig vorberechenbar

`kernel.cpp:2117` akkumuliert je Wandbesuch und Schritt über 19 Richtungen:

```
G11 = fma(6.0f*wi, ct1*ct1, G11);  G22 = fma(6.0f*wi, ct2*ct2, G22);  G12 = fma(6.0f*wi, ct1*ct2, G12);
Sn1 = fma(6.0f*wi, ct1*cn,  Sn1);  Sn2 = fma(6.0f*wi, ct2*cn,  Sn2);
```

`ct1`, `ct2`, `cn` sind Projektionen der **Linkrichtung** auf die Facettentangenten und die
Normale, `wi` ist das Gittergewicht. **Nichts davon hängt an der Strömung.** Damit sind
konstant über den ganzen Lauf:

| Größe | hängt ab von |
|---|---|
| G11, G12, G22, Sn1, Sn2 | Linkrichtung, Tangenten, Normale — **Geometrie** |
| ALPHA2-Downdate | wirkt auf G — **Geometrie** |
| Rangbestimmung, det-ε-Wächter, Zweigwahl | Ergebnis von G — **Geometrie** |
| **P1, P2** (`kernel.cpp:2118`) | **Strömung** — muss je Schritt bleiben |

**Vorschlag:** nach dem Voxelieren einmal je Facette die fertig gedowndatete Matrix, den
Rangcode und die Lösekoeffizienten ablegen — rund 4 float + 1 Byte = **≈ 51 MiB** bei
3,13 M Facetten. Gegenfinanziert durch `CFD_FAC_KDIAG=0` (−191 MiB).

**Zweiter Schnitt in derselben Schleife:** P1 und P2 sind zwei Projektionen **derselben**
Summe über die Wandlinks. Statt je Link zwei Skalarprodukte zu bilden, ließe sich die
Vektorsumme Σ 2 c_i f_i akkumulieren (die Koeffizienten sind ±1 und 0, der Übersetzer faltet
sie zu Additionen) und erst danach zweimal projizieren: grob **228 → 67** Rechenoperationen
je Facettenbesuch. Der Vektor existiert bereits als `Pvx/Pvy/Pvz` (`kernel.cpp:2120`) — aber
**nur unter `FACETTEN_PEMA`**, das in Produktion aus ist; in Produktion gibt es die Redundanz
also nicht, wohl aber den Umbauweg.

**NICHT bitgleich.** Die Summationsreihenfolge ändert sich. Braucht ein A/B, keine
Identitätsbehauptung. Zu prüfen vor dem Bau: ob das ALPHA2-Downdate wirklich nur auf G wirkt
und nirgends eine Strömungsgröße einschleust.

## A2 — Die Spalding-Inversion ist eine feste 1D-Funktion und wird dreimal iteriert

`wf_spalding_uplus` (`kernel.cpp:1652-1677`) läuft `def_wf_spalding_it = 3` Newton-Schritte
mit Exponentialfunktion je Wandbesuch, für eine Kurve, die sich nie ändert. Eine tabellierte
Umkehrung mit linearer Interpolation wäre ein Lesezugriff statt drei Iterationen. Kosten der
Tabelle: vernachlässigbar. Genauigkeit gegen Iterationszahl ist zu messen.

## A3 — Der Facettenblock sitzt im falschen Kernel

Nur ~0,6 % der Zellen sind Facettenzellen, aber sie liegen geclustert: auf SIMD16 rechnet
eine Subgroup mit **einer** Facettenzelle den ganzen 4571-Instruktionen-Block. Ein eigener
Start über die Facettenliste hätte dort volle Auslastung und nähme die Divergenz aus dem
519-M-Zellen-Kernel. Die Infrastruktur existiert (`fac_idx` als Bitmaske plus Präfixsumme;
`sgs_fdwand` und `fac_nachbar_ab` machen es bereits so).

**Der Haken ist echt:** der Block verändert `fhn` mitten in der Kollision. Ein zweiter
Durchgang über die DDFs kostet Bandbreite, und die könnte den Gewinn auffressen. Entwurf,
keine Handreichung — vor dem Bau ist die Bandbreitenrechnung aufzumachen.

## A4 — Arbeit auf die iGPU verschieben geht kaum, und der Grund ist hart

Die iGPU hat Luft: das Fernfeld liegt vollständig hinter dem Nahfeld verborgen. Aber alles,
was man verschieben würde, braucht die DDFs des Nahfelds, und die liegen auf der B70 — der
Transfer übersteigt den Gewinn. Verschiebbar wäre nur die Ausgabe, und die kostet zusammen
0,49 %.

**Der unbequeme Rückschluss:** das Fernfeld schneller zu machen bringt **null Wanduhr**. Die
−22,8 % aus Teil 1 sind nur auf dem Papier schön; zählbar ist allein der Nahanteil.

## A5 — Ungegatete Ausgabe in der Aufbauphase

Die Aufbauphase kostet 166 s = 2,93 % und ist nicht instrumentiert. Darin werden
**unbedingt** geschrieben: `remesh_flaeche.vtk` (**249 MB ASCII**, `setup.cpp:1971`, formatiert
3,35 M Knoten und 6,69 M Dreiecke einzeln über einen Stream) und `facetten_histogramme.csv`
(**153 MB**, `setup.cpp:3330`). Das benachbarte `guete_vtk` ist korrekt hinter
`CFD_FACETTEN_VTK` gegatet, diese beiden nicht. Ein Schalter davor ist reiner Gewinn.

## A6 — Die Bandbreitenanzeige überzeichnet um rund 10 %

`bandwidth_bytes_per_cell_device()` (`lbm.cpp:54-74`) rechnet 123 B je Zelle und Schritt,
darin 12 B für das Kraftfeld, das `stream_collide` unter `F_NUR_SOLID` gar nicht liest. Die
angezeigten GB/s sind entsprechend zu hoch. Kostet keine Leistung — aber jede
Optimierungsentscheidung wird gegen diese Zahl gemessen.

## Und der größte Posten bleibt keiner von diesen sechs

**ELIBB: 2073 Instruktionen = 28,9 % von `stream_collide`**, läuft an 73,4 % der Facetten je
Schritt — und die einzige saubere Messung (26-Grad-Kanal, `AUDIT-BEFUNDE.md:3799`) sagt
u_tau 2,382 mit gegen 1,943 ohne, also 23 % schlechter, bei verdreifachten Gate-Rückfällen.
Am Fahrzeug existiert kein gepaartes A/B. Bevor Geometrie vorberechnet wird, ist das der
billigere Erkenntnisgewinn.

---

# Teil 3 — Zweite Agentenrunde, andere Blickrichtungen

Drei unabhängige Agenten, bewusst mit anderen Leitfragen als Teil 1. Der VRAM-Agent
bekam die **Gegenrichtung**: nicht „was ist zu breit", sondern „was muss überhaupt
residieren". Ergebnisse hier nur, soweit sie über Teil 1 hinausgehen oder ihn korrigieren.

## 3.1 VRAM aus der Gegenrichtung

### Die README-Zahl „1,43 GB bei −12 %" gilt für v2 NICHT

Das ist die wichtigste Korrektur dieser Runde, und sie trifft eine Zahl, die ich selbst
ungeprüft übernommen hatte. Teil 1 stufte `CFD_TILE_WG` als Phantomschalter ein — richtig,
er existiert im Quelltext nicht. Die Messzahl daneben (`README.md:221` und `:585`) ist aber
**echt**: sie stammt aus **V1**, wo `SPARSE_TILES_WG`, `active_tile_id` und `load_f_pre`
existieren (nachgeprüft: `FluidX3D/src/lbm.cpp:1058`, `kernel.cpp:2779`). In v2 null Treffer.

| Variante | Durchsatz | fi frei |
|---|---|---|
| dense | 4348 MLUPS | — |
| naiv (= v2s heutiger Stand) | 2624 MLUPS, **−40 %** | 1,43 GB |
| + Workgroup=Tile (nur V1) | 3836 MLUPS, **−12 %** | 1,43 GB |

**Wer in v2 heute SPARSE einschaltet, zahlt −40 %, nicht −12 %.** Die Portierung des
V1-Dispatch gehört davor. Die übrige Infrastruktur ist in v2 vollständig: `TS_P` liegt an
`initialize`, `stream_collide`, `boden_eq`, `einlass_eq`, `update_fields`, `schale_blend`,
`update_force_field`; `sgs_fdwand`/`fac_nachbar_ab`/`sgs_gdiag` lesen fi gar nicht.

### Gemessene Zellklassen statt geschätzter

Echt solid **12,197 %** (63 317 501 Zellen), aktiv **87,591 %**. Tote Residenz darauf:
fi 2 294,6 MiB, u+rho 966,1 MiB, flags 60,4 MiB. TYPE_E scheidet aus (Nachbarn streamen
daraus), die Dämpfungszone auch (reines Fernfeld).

| Kachelkorn T | aktive Tiles | fi | **frei** |
|---|---:|---:|---:|
| 4 | 7 395 260 / 8 215 506 | 17 152,1 MiB | 1 661,3 MiB |
| **8** | **944 536 / 1 038 164** | **17 525,6 MiB** | **1 287,8 MiB** |
| 16 | 123 710 / 133 560 | 18 363,4 MiB | 450,1 MiB |

Teil 1 schätzte „1,0–1,5 GiB bei T=8" — der Wert ist **1 287,8 MiB** gegen die Obergrenze
2 294,6 MiB. Halo-2 und Kachelkorn fressen 44 % des theoretischen Gewinns.

### Der gedruckte Spitzenwert ist keiner

`setup.cpp:7055` schreibt „Hier ist der Aufbau vollstaendig, also steht hier der
SPITZENWERT" — und drei Zeilen weiter legt `alloc_coupling_planes` (7103) an, dann
`alloc_schale` (7127), und **`kf_liste` (237,3 MiB) wird erst in der Zeitschleife
gebunden**, beim ersten Kräfte-Sample (`setup.cpp:7824` → `3583` → `lbm.cpp:1203`).

| Block | MiB | Beleg |
|---|---:|---|
| fi + rho + u + flags + f_maske + F | 27 310,9 | Log Z. 442 ✓ |
| Facettenkette | 581,3 | Log Z. 444 ✓ |
| SGS-Band | 118,8 | Log Z. 474 ✓ |
| späte Puffer (coupling · slice_flags · schale · **kf_liste** · kf_psum/pcnt) | 300,6 | nach dem Druck |
| **echter Spitzenwert** | **28 311,7** | gedruckt: 28 003 |

**Der wahre Spitzenwert fällt nach dem ersten Kräfte-Sample — also nachdem jeder
Speicherwächter passiert ist.** Ein Lauf kann den ganzen Aufbau überleben und 260 MiB
später sterben. Das ist die schärfere Fassung von Teil 1 (dort: 28 314 MiB).

### Die Reserveposten stimmen in der Summe, nicht in der Aufteilung

`reserve = 2496 MB = 320 (Spätpuffer) + 1152 (Desktop) + 1024 (Mindestluft)`,
`lbm.cpp:2159-2168`. Der 320-MB-Posten deckt 300,6 MiB real — Teil 1 bestätigt.

**Was Teil 1 nicht geprüft hat:** `bytes_bekannt` (`lbm.cpp:2136-2150`) kennt die
**Facettenkette (581,3 MiB) und das SGS-Band (118,8 MiB) überhaupt nicht**. Dagegen steht
nur der F-Überbuchungspolster (Zeile 2147 bucht 184,8 MiB, real 81,0 → 103,8 MiB Polster).
**Netto 596,3 MiB ungedeckt.** Die Größenordnung der Bilanz stimmt trotzdem, weil der
Desktop-Posten großzügig ist.

Dazu eine echte Lücke, selbst nachgeprüft: `alloc_facetten_domain` prüft
`mb_fac + mindest > frei` (`lbm.cpp:836-841`) — **`alloc_sgs_band` prüft gar nichts**
(`lbm.cpp:660-719`, nur Strukturwächter: fac_idx vorhanden, 2^32-Grenze, Bandliste
nichtleer, Präfixsumme stimmig). Die 118,8 MiB fallen ungeprüft nach dem Facettenwächter.
In diesem Lauf lief selbst der Facettenwächter blind, weil der DRM-Debugfs-Frei-Wert nicht
lesbar war (Log Z. 443) und er auf die 20/19-Rekonstruktion zurückfiel, die den Desktop
nicht sieht.

### System-RAM: kein Engpass, aber die gedruckte Zahl ist falsch

Spitze ≈ 20,6 GiB von 91 GiB. Der VTK-Dump ist **kein** RAM-Posten (`schreibe_vtk_feld`
streamt zeilenweise über `Sx*3`, `setup.cpp:1040-1071`) — die 8 417 MB sind Datei.

`info.cpp:78-82` bucht `F` host-seitig mit 12 B über die ganze F-BBox = **1 832,3 MiB**,
während `CFD_F_LISTE` nur 3 739 681 Slots = **42,8 MiB** anlegt (`lbm.cpp:789`). Dieselbe
Fehlerklasse, die der Kommentar `info.cpp:72-76` für die BBox gerade behoben hat. Gleichzeitig
fehlen der Zeile ~1 042 MiB Facetten-, Band- und Spätspiegel. „CPU 10248 MB" ist eine Formel,
real ≈ 9 501 MiB.

**Neuer Posten, größer als die bekannte 949-MiB-Dreifachhaltung:** der Glättungsindex
`std::vector<uint> feld(bnx*bny*bnz)` (`setup.cpp:3005`, selbst nachgeprüft) belegt
**593,7 MiB** (1113×463×302) für 3 275 383 Einträge = **2,1 % Belegung**, daneben
`std::vector<Facette> G=F` (`setup.cpp:3016`), Vollkopie 174,9 MiB. Der Index ist restlos
vermeidbar: `baue_facetten` scannt `for z … for y … for x` (`setup.cpp:2622`), also ist `F`
streng nach `.n` aufsteigend — `feld[bidx(...)]` ist ein `std::lower_bound` über `F` mit
null Zusatzspeicher, und liefert denselben Index. Ein `is_sorted`-Wächter wäre billiger als
die Annahme. Zweitens lebt `elibb_qmap_dd` (`setup.cpp:6156`) bis Laufende, obwohl nach
`alloc_facetten` (`setup.cpp:6237`) tot — ~174 MiB, ein `clear()` wäre eine Zeile.

### Fragmentierung: sauber, außer unter SPARSE

`fi` (18 813 MiB) wird als **erster** Puffer angelegt (`lbm.cpp:374`) — die einzige
Reihenfolge, die eine 18-GB-Allokation nicht gegen Fragmentierung laufen lässt. Alle
Neuanlagen ersetzen 1-Element-Platzhalter. **Unter SPARSE kippt das:** `fi` ist dann ein
1-Zell-Platzhalter, die echte 17,5-GB-Allokation kommt in `finalize_sparse_tiles`
(`lbm.cpp:1087`) **nach** rho/u/flags/F/f_maske — 8,5 GB liegen dann schon. Ursache ist ein
Treiberdefekt (`lbm.cpp:370-373`: Freigeben eines allozierten 19-GB-fi wirft
`CL_OUT_OF_RESOURCES`), also nicht änderbar. Wer SPARSE baut, muss das wissen.

### Die drei Maßnahmen aus Runde 2, die nicht in Teil 1 stehen

1. **`CFD_SPARSE_TILES=1 CFD_SPARSE_T=8` — 1 287,8 MiB (gemessen).** Der einzige Hebel über
   1 GiB, der das Format nicht anfasst. Preis am heutigen v2-Stand **−40 %** (5 503 s
   Zeitschleife → ~9 170 s). Die V1-Portierung von Workgroup=Tile gehört davor, sie senkte
   denselben Preis auf −12 %. Vorher: 8-mm-Paar SPARSE 0/1 auf Bitgleichheit.
2. **`tile_slot`-Indirektion auf `rho`/`u`/`flags` ziehen — +576,1 MiB, Gesamt 1 863,9 MiB.**
   Dieselbe Adressrechnung, Klassifizierer und Wächter wären schon da. Haken: `u` wird von
   `sgs_fdwand` und `fac_nachbar_ab` über Nachbaroffsets gelesen, der VTK-Export liest das
   Vollgitter host-seitig — der bräuchte einen Gather. Entwurf, keine Handreichung.
3. **Glättungsindex → `lower_bound` — 593,7 MiB System-RAM, eine Zeile.** Mit der Ansage:
   der System-RAM ist bei 20,6 von 91 GiB **nicht** die Bindung. Das ist Hygiene, keine
   Kapazität. Wer nur eine Zeile ändern darf, ändert sie nicht hier.

*Unter der Schwelle:* die drei 38,2-MiB-Bitmasken über dieselbe F-BBox (`f_maske`,
`fac_idx`, `band_idx`) zusammenzulegen bringt < 50 MiB und kostet drei getrennte
Präfixsummen.

### Was Runde 2 nicht messen konnte

- **Durchsatzkosten von SPARSE auf der B70 in v2** — die −40 %/−12 % sind V1-Zahlen.
- **Freier VRAM real** — Debugfs-Rechte fehlen, alle Frei-Werte sind die 20/19-Rekonstruktion.
- **Bitgleichheit von SPARSE bei aktiver Facettenkette** — strukturell geprüft, nie gelaufen.
- **RAM-Spitze im Betrieb** — aus Allokationen gerechnet, nicht aus `/proc/<pid>/status`.
