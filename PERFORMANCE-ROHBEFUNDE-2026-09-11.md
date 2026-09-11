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

## A1 — WIDERLEGT (siehe Teil 3.3). Ursprünglicher Titel: „Die halbe Facettenkette ist Geometrie"

> **⚠ Dieser Abschnitt ist falsch und bleibt nur als Beleg stehen.** Die Momentenmatrix ist
> **nicht** reine Geometrie: `t1 = ut/|ut|` (`kernel.cpp:2073`) zeigt in Richtung der lokalen
> tangentialen Geschwindigkeit, und die stammt aus `calculate_rho_u(fhn,…)` (`kernel.cpp:2002`,
> selbst nachgeprüft). `ct1`/`ct2` sind Projektionen auf **diese** Basis, also drehen sich
> G11/G22/G12/Sn1/Sn2 jeden Schritt mit der Strömung. Konstant sind nur der volle 3×3-Tensor
> `M = Σ 6w_i c_i c_iᵀ` und die basisunabhängigen Invarianten Snn, det(G′), tr(G′), |Sn′|².
> Der Ersatzvorschlag mit demselben Ziel und ohne Falschannahme steht in **Teil 3.3, Variante B**
> (−2,76 % Instruktionen, null zusätzlicher Speicher). Die unten genannten ≈ 51 MiB wären
> ohnehin 113 MB gewesen.


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

## A6 — Die Bandbreitenanzeige überzeichnet (Zahl in Teil 3.2 berichtigt: 47 %, nicht 10 %)

> **⚠ Die Größenordnung unten ist zu klein.** Teil 3.2 hat nachgerechnet: 63,85 gegen 43,36 GB,
> also **47 %**. Der größere Posten sind nicht die 12 B Kraftfeld, sondern **18 B Nachbar-Flags
> unter `MOVING_BOUNDARIES`**, die für 100 % der Zellen gebucht und von 0,21 % bezahlt werden.


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

## 3.2 Bandbreite und Zugriffsmuster

### Die angezeigten MLUPs sind ein Anzeigefehler — Faktor 2,551

Das ist der größte Befund dieser Runde. **Selbst nachgeprüft, Kette geschlossen:**

- `Info::print_update` rechnet `lbm->get_N() / runtime_lbm_timestep_smooth` (`info.cpp:119`).
- `info.lbm` wird in `print_initialize(LBM*)` gesetzt (`info.cpp:49`). Letzter Aufrufer im
  Aufbau ist `lbm_c.run(0u)` (`setup.cpp:7045`, direkt nach `lbm_f.run(0u)` in 7044) — der
  Zeiger zeigt also auf das **Fernfeld**.
- `info.update(clock.stop())` steht nur in `LBM::run` (`lbm.cpp:2447`; die zweite Stelle 2877
  ist der Partikelpfad, nicht aktiv). Die Zeitschleife ruft `run()` **nur fürs Nahfeld**
  (`setup.cpp:7441/7459`); das Fernfeld läuft über `run_async`, und das ruft `info.update` nie.

**Angezeigt wird also: grobe Zellzahl geteilt durch feine Schrittzeit.**

| | Wert |
|---|---|
| angezeigt (`logs/p4dt_deteps.log`) | 1946–1974 MLUPs, 239–243 GB/s |
| Nf/Nc = 519 139 485 / 203 489 280 | **2,5512** |
| **wahre Nahfeldrate** | **5028 MLUPs** (dt_fein = 103,24 ms) |
| Gegenprobe Phasenprofil | 434,6 ms/grob × 95,8 % / 4 = 104,09 ms — 0,8 % ab |

Eigenbeweis im Lauflog: nach dem Nahfeld-Header steht `519 | 64 GB/s`, nach dem
Fernfeld-Header `203 | 25 GB/s`, beide bei dt = 1,0 s Startwert. Die Zellzahl springt mit,
die Uhr nicht.

**Folge für das ganze Projekt: jede MLUPs- und GB/s-Zahl aus einem Doppeldomänenlauf ist um
2,55 zu klein.** Das betrifft `README.md:45` („1534 MLUPs / 189 GB/s", 8-mm-Sprosse, Faktor
dort 2,561). Die **Verhältnisse** der A/B-Kette („939 → 1534, +63 %") bleiben gültig — beide
Arme tragen denselben Fehler. Die **Absolutwerte** nicht. Für den laufenden ELIBB-A/B heißt
das: die Wanduhr bleibt das Maß, MLUPs taugen nur als Verhältnis.

### Korrektur am Agentenbefund: die 5464 sind belegt, nur nicht hier

Der Agent stufte die dreimal in `README.md` (48, 77, 665) genannten „≈ 5 464 MLUPS" als
unbelegt ein. **Das stimmt nicht.** Sie stehen in `FluidX3D/MODIFICATIONS.md:251` als eigene
Messung: V1, **Einzeldomäne**, 337,5 M Zellen, Baseline ohne Wandmodell (dagegen 3289 MLUPs
mit Wandmodell, −40 %). Einzeldomäne heißt: kein Anzeigefehler, die Zahl ist echt.

**Damit fällt der „Faktor 2,8" ganz weg, und das Bild dreht sich:**

| | MLUPs | Quelle |
|---|---:|---|
| Upstream B70, FP32/FP16S | 6750 (85 % von 608 GB/s) | `README_UPSTREAM.md:1223` |
| V1 Einzeldomäne, ohne Wandmodell | 5 464 | `FluidX3D/MODIFICATIONS.md:251` |
| **v2 Nahkernel heute, wahr** | **5 028** | diese Runde, gerechnet |
| V1 Einzeldomäne, mit Wandmodell | 3 289 | `FluidX3D/MODIFICATIONS.md:251` |
| v2, angezeigt | 1 946 | Anzeigefehler ×2,551 |

Der v2-Nahkernel liegt **8 % unter** der V1-Baseline ohne Wandmodell — mit der kompletten
Facettenkette, SISM, P-TRT und DETEPS obendrauf. Das ist kein Performanceproblem, das ist
ein gutes Ergebnis. **Die Suche nach dem „verlorenen Faktor" war eine Suche nach einem
Artefakt.**

### Wahrer Verkehr je Zelle und Schritt

SoA-Layout `index_f(n,i)=i*def_N+n` (`kernel.cpp:993`), aus dem Quelltext ausgezählt:

| Zellklasse | Anteil | B/Zelle/Schritt |
|---|---:|---:|
| Solid (früher Ausstieg, `kernel.cpp:2894`) | 12,2 % | 1 |
| Freistrom-Fluid außerhalb der F-BBox | 57,0 % | 93 |
| Fluid in der F-BBox | 29,6 % | 93,75 |
| Bandzelle Lage 2 | 0,51 % | 97,75 |
| **Facettenzelle** | 0,60 % | **339,5** = 3,65× Freistrom |

| Posten je feinem Schritt | GB | % |
|---|---:|---:|
| DDF lesen + schreiben (2×38 B × 455,8 M Fluidzellen) | 34,64 | 79,9 |
| rho + u schreiben | 7,29 | 16,8 |
| flags | 0,52 | 1,2 |
| Facettenpuffer gesamt | 0,77 | 1,8 |
| fac_idx (2×) + band_idx | 0,12 | 0,3 |
| **Summe** | **43,36** | |
| je Gitterzelle | 83,5 B | |
| **erreichte Bandbreite** | **420 GB/s** = 69 % der 608er Spitze, 81 % der von Upstream auf derselben Karte erreichten 520 GB/s | |

### Korrektur an Teil 2 §A6: die Anzeige überzeichnet um 47 %, nicht 10 %

63,85 gegen 43,36 GB. Zerlegung der 20,50 GB Lücke:

- **Nachbar-Flags, 18 B unter `MOVING_BOUNDARIES`** (`lbm.cpp:65-67`): 9,32 GB = **45 %**.
  `apply_moving_boundaries` läuft nur an TYPE_MS-Zellen (`kernel.cpp:2903`) — 1 102 365 von
  519 M = **0,21 %**. 18 B werden für 100 % der Zellen gebucht und von 0,21 % bezahlt.
  **Das ist der größere Posten und er fehlte in Teil 2.**
- `F` (12 B, nie gelesen unter `F_NUR_SOLID`): 6,23 GB = 30 % — der in §A6 gefundene Teil.
- DDF+rho+u über alle statt über die 87,8 % Fluidzellen: 5,83 GB = 28 %.
- Gegenbuchung Facetten-/Indexpuffer, die die Formel nicht kennt: −0,90 GB.

### Koaleszenz ist in Ordnung — am Offline-Compiler gemessen

`ocloc -device bmg-g31`, Produktionsdefines rekonstruiert. Kontrollwert: `stream_collide`
6630 instCount gegen 6590 der ersten Runde.

| Arm | inst | DDF ld/st | flags ld | f32 ld/st | Atomics |
|---|---:|---:|---:|---:|---:|
| Produktion | 6630 | **19 / 19** | 91 | 97 / 12 | 59 |
| ohne KDIAG | 6594 | 19 / 19 | 91 | 93 / 8 | 59 |
| **ohne ELIBB** | **4484** | 19 / 19 | 55 | 77 / 11 | 40 |
| ohne SGS_BAND | 6508 | 19 / 19 | 91 | 92 / 12 | 57 |
| reines Bounce-Back | 3401 | 19 / 19 | 43 | 59 / 6 | 18 |
| Fernkernel (B70) | 1628 | 19 / 19 | 19 | 44 / 4 | 10 |

**Der DDF-Pfad ist in jedem Arm exakt 19 `load.ugm.d16u32` + 19 `store` — voll koaleszierte
32-B-Nachrichten, keine einzige Zusatzberührung durch die Facettenkette.** Nur die
±x-Nachbarslots laufen um ein Element versetzt, das ist Esoteric Pull und upstream-inhärent.

Bitmaske + Präfixsumme kosten je Zelle **drei** Aufschläge (über `cbit`/popcount gezählt):
`fac_idx` in `apply_facette_imem` (`kernel.cpp:1971`), `fac_idx` **nochmal** im FDWAND-Block
(`kernel.cpp:3090`), `band_idx` (`kernel.cpp:3092`). Zwei unabhängige Ladepaare auf dieselbe
Adresse, der Übersetzer fasst sie nicht zusammen. **Bandbreitlich egal** (0,12 GB = 0,3 %,
streng sequentiell) — der Preis ist Latenz und Instruktionen.

### Neuer harter Befund: `fac_nachbar_ab` hat Scratch, und das Gate sieht ihn nicht

```
fac_nachbar_ab   simd=32  grf=128  private_size=7296  spill=0
stream_collide   simd=16  grf=128  private_size=0     spill=0
sgs_fdwand       simd=32  grf=128  private_size=0     spill=0
```

7296 B = 228 B (die 57-float-Tabelle in `c()`, `kernel.cpp:996`) × 32 Lanes. **Genau die
Fehlerklasse, für die `scratch_gate.sh` gebaut wurde** — gefunden wird sie nicht, weil das
Gate `stream_collide` fest verdrahtet als einzigen Kernel prüft (selbst nachgeprüft:
`scratch_gate.sh:33`, `igc_offline.sh "$T/$arm.cl" "$dev" stream_collide`).

Ursache und Gegenprobe, beide gemessen:

| Variante | private | inst | Scratch |
|---|---:|---:|---|
| Produktion | 7296 | 791 | 42× `store.ugm.d32x4`, 21× `load.ugm.d32x8t` |
| `opencl_unroll_hint(18)` | 7296 | 791 | unverändert — **Unrolling hilft nicht** |
| **`c(ib)` (`kernel.cpp:4788`) → `0.0f`** | **0** | **484 (−39 %)** | **alle weg** |

Es ist nicht die Schleife, es ist der eine laufzeitindizierte `c(ib)`-Zugriff **nach** ihr.
Obergrenze 2,1 GB Scratch je Launch, 8,4 GB je grobem Schritt (≤ 4,8 %); ein großer Teil
bleibt im L1, die −39 % Instruktionen sind hart.

### Das Fernfeld hat eine Wand nach oben — und das ist neu gegenüber Teil 2 §A4

| | |
|---|---:|
| Fernschritt gemessen | **369 ms** |
| Fenster für den Fernschritt (0,9 % + 95,8 % von 434,6 ms) | **420 ms** |
| **Schlupf** | **12 %** |

Teil 2 §A4 sagte richtig: dort sparen bringt null Wanduhr. **Die schärfere Folgerung fehlte:
wird das Nahfeld um mehr als 12 % schneller, wird die iGPU zum kritischen Pfad.** Jede
Nahfeld-Maßnahme über 12 % hinaus ist ohne gleichzeitige Fernfeld-Arbeit wertlos. Das ist
die Obergrenze, an der jede Optimierung dieses Dokuments zu messen ist.

Ob die iGPU bei 51,3 GB/s bandbreiten- oder rechengebunden ist, bleibt offen: 25,4 M Threads
× 2145 Instruktionen bei 512 Lanes/2,0 GHz ergibt 0,43 s theoretisch gegen 0,369 s gemessen
— **beides liegt in derselben Größenordnung**. Rezept: 25-s-Paar auf Gerät 2 mit
`UPDATE_FIELDS` an/aus (92 → 76 B/Zelle, −17 % Verkehr).

### Die drei Bandbreiten-Maßnahmen aus Runde 2

1. **`sgs_fdwand` und `fac_nachbar_ab` zu einem Launch verschmelzen.** Beide laufen über
   dieselbe Liste mit identischem Bereich (`lbm.cpp:1009` und `:1019`), beide je feinem
   Schritt, beide lesen `u` und `flags` derselben 6er-Nachbarschaft derselben 3 129 185
   Zellen, beide schreiben disjunkt, keiner liest die Ausgabe des anderen. Verschmolzen
   entfallen 6 doppelte Flag-Gather und ein Dispatch. **Geschätzt 0,4–2 GB je feinem Schritt
   = 1–4 %**, bitgleich bei erhaltener Schreibreihenfolge. Geschätzt, weil die Trefferquote
   der Gather nicht messbar war.
2. **`c(ib)` in `fac_nachbar_ab` durch Arithmetik ersetzen** — `private_size` 7296 → 0,
   instCount −39 %, bitgleich (dieselben Konstanten, nur nicht über den Speicher). **Und:
   `scratch_gate.sh` muss alle Kernel prüfen, nicht nur `stream_collide`**, sonst bleibt
   diese Fehlerklasse weiter blind. Das ist die billigste Maßnahme im ganzen Dokument.
3. **`CFD_FAC_KDIAG=0`** — 0,400 GB je feinem Schritt = 0,92 %, dazu −191 MiB VRAM. Größter
   Einzelposten unter den Zusatzpuffern und trotzdem unter einem Prozent. **Das ist die
   ehrliche Größenordnung: in `stream_collide` steckt kein großer Bandbreitenhebel mehr, weil
   80 % des Verkehrs der DDF-Strom ist und der bei D3Q19/FP16S nicht kleiner wird.**

*Bewusst nicht empfohlen:* `rho[n]` jeden Schritt zu schreiben kostet 1,82 GB = 4,2 %.
Gelesen wird rho zwischen den Schritten nur an TYPE_E, am Druck-Auslass und von der Ausgabe.
Ein Gate wäre **nicht bitgleich** und würde den Druck-Auslass verändern.

### Was Runde 2 hier nicht messen konnte

- **Laufzeit irgendeines Kernels** — alles ist Instruktions-, Nachrichten- und Byte-Zählung.
- **Trefferquote der u-/flags-Gather** — die Spanne 1,4–5 GB ist deshalb Faktor 3,5 breit.
- **Dispatchkosten von `boden_eq`** (519,1 M Work-Items für ~1,1 M aktive) — offen aus Runde 1.
- **DDR5-Spitzenbandbreite** — `dmidecode` braucht root, EDAC leer.
- **Divergenzkosten der Facettenkette** — 0,6 % der Zellen, 4571 Instruktionen je betroffener
  Subgroup. Braucht einen Laufzeit-A/B.

## 3.3 Datenfluss und Vorberechnung — was ist konstant, was dreht sich mit

Messgrundlage: Offline-Compiler, Produktionszeile rekonstruiert, Basis `stream_collide`
**7221** Instr. auf der B70 (0xe223, simd16, private 0, spill 0), **8396** auf der iGPU
(0x7d67, simd8). Die Differenz zu Runde 1 (6590) ist gemessen erklärt: P-TRT +243,
KDIAG +52, DETEPS 16 +20. Alle Deltas unten gegen **diese** Basis.

Die Voraussetzung aller Geometrieaussagen trägt: `flags` wird nach der Voxelierung nie wieder
geschrieben (`setup.cpp:6043` im Aufbau, Zeitschleife ruft nur `lbm_f.run`;
`update_moving_boundaries` läuft im Fahrzeugfall nie, `kernel.cpp:4155`).

### A1 ist widerlegt — die Basis dreht sich mit der Strömung

**Das ist eine Korrektur an mir selbst, nicht am Code.** Ich hatte in Teil 2 behauptet,
G11/G22/G12/Sn1/Sn2 seien reine Geometrie. Nachgeprüft: `t1 = ut/|ut|` (`kernel.cpp:2073`),
`ut` aus `calculate_rho_u(fhn,…)` (`kernel.cpp:2002`), `t2 = n × t1` (`:2074`), und `ct1`/`ct2`
(`:2116`) sind Projektionen auf genau diese Basis. **Sie drehen sich jeden Schritt.**

Numerisch belegt, feste Normale und feste Wandlinkmenge, nur der Azimut φ der Basis gedreht:

| φ | G11′ | G22′ | G12′ | Sn1′ | **Snn′** | **det(G′)** | **tr(G′)** |
|---|---|---|---|---|---|---|---|
| 0,0 | 0,48891 | 0,48292 | 0,01376 | −0,06324 | 0,13928 | 0,23592 | 0,97183 |
| 0,7 | 0,49999 | 0,47185 | −0,00061 | **+0,00220** | 0,13928 | 0,23592 | 0,97183 |

Sn1 wechselt das Vorzeichen, die Invarianten stehen. Konstant sind: der volle 3×3-Tensor
`M = Σ 6w_i c_i c_iᵀ`, **Snn** (der Code sagt es selbst, `kernel.cpp:2252`), det(G′), tr(G′),
|Sn′|². Das ALPHA2-Downdate (`kernel.cpp:2189`) schleust übrigens **keine** Strömungsgröße ein
— `S1` und `S0` sind reine Linkgewichtssummen, das Downdate ist exakt die Projektion des
Rang-1-Downdates. Nur nützt das nichts, weil die Basis strömungsabhängig ist.

Folge für die Zweigwahl, und die ist gemischt:
- **Geometrie** ist das Entkopplungs-Gate `kernel.cpp:2336` (`Snn<1e-8f || kop<=1e-6f*Snn*(G11+G22)`) — alle drei Größen invariant.
- **Nicht Geometrie** sind die det-ε-Wächter `kernel.cpp:2338` und `:2382`: `G11*G22` und die
  Einzeldiagonalen wandern mit dem Azimut. Genau diese Schwellen entscheiden über Vollrang
  gegen Rückfall. Sie lassen sich nicht einfrieren.

Obergrenze von A1, falls man trotzdem alles fertig gedowndatet aus einem Puffer läse:
7221 → 6928 = **−4,1 %**, und zwar für **113 MB**, nicht die von mir genannten 51 MiB.

### Variante B — derselbe Gewinn zu zwei Dritteln, für null Speicher

Im Loop den **3×3-Tensor mit konstanten Inkrementen** akkumulieren (`M_ab += 6w_i c_a c_b`,
Koeffizienten 0/±konstant), P über den Vektor `Qv = Σ 2 c_i fhn[i]`; **danach einmal**
projizieren (2 Matvecs + 5 Skalarprodukte). Algebraisch exakt — 2000 Zufallsfälle, max.
5,8·10⁻¹⁶ in double.

**Gemessen: 7221 → 7022 = −199 Instr. = −2,76 %, null zusätzlicher Speicher.** Das sind 68 %
von A1s theoretischem Maximum, ohne die Falschannahme und ohne 113 MB.

### Die Tabelle: konstant, langsam, schnell

| Größe | Ort | Klasse | Vorberechnung | Verdikt |
|---|---|---|---|---|
| G11/G22/G12/Sn1/Sn2 | `kernel.cpp:2117/2123` | **SCHNELL** | – | A1 trägt nicht; Variante B: −2,76 %, 0 B |
| **Spalding-Umkehrung** (3× Newton, 2 exp + 2 log je It.) | `kernel.cpp:1652-1677` | **KONSTANT** (feste 1-D-Kurve) | 256 float = **1 kB** | **Ja, klar.** −330 Instr. = −4,57 %, **und 450× genauer** |
| **Suche „bester Normalenlink"** | `kernel.cpp:4772-4778` | KONSTANT | 1 uint = 12,5 MB | **Ja.** 791 → **75 Instr. = −90,5 %**, `neighbors()` entfällt ganz |
| **`ywb = yw + c(ib)·n`** | `kernel.cpp:4788/4790` | KONSTANT | 1 float | **Ja** — heute 12,5 MB toter Schreibverkehr je Feinschritt |
| **Wandlink-Gate, dreimal dieselben 18 Werte** | `kernel.cpp:1857, 2113, 2597` | KONSTANT | 18 Bit = **0 B** (freier fac_geo-Slot) | **Ja.** zwei von drei ersetzt: −130 Instr. = −1,8 % |
| 6 Solid-Tests in `sgs_fdwand` | `kernel.cpp:4703` | KONSTANT | 6 Bit | **Ja.** 672 → 632 = −6,0 % |
| **ABSTAND-Scan in `boden_eq`** | `kernel.cpp:3801-3806` | KONSTANT | 1 Flagbit = 0 B | **Ja.** 1267 → **1170 = −7,7 %**; heute 347 M Reads je grobem Schritt für **0,50 % Treffer** |
| **`cubic_lift_weights`** | `kernel.cpp:4173`, Def. `:4090` | KONSTANT (bei ratio=4 nur 4 innere Sätze) | 1-D-Tabelle ≈ 54 kB | **Ja.** 962 → **683 = −29,0 %** |
| Blockfluid-Maske in `schale_extract` | `kernel.cpp:4231-4243` | KONSTANT | 64-Bit-Maske, 22 MB | **Ja** — 119 M flags-Reads je grobem Schritt |
| SISM-EMA `fac_sb` | `kernel.cpp:4749` | **LANGSAM**, echter Zustand | – | **Nein**, sbar geht je Schritt in w ein |
| `neighbors(n,j)` in `stream_collide` | `kernel.cpp:2903` | KONSTANT | **76 B/Zelle** gegen 57 B Budget | **Nein** — würde den Speicher mehr als verdoppeln. Zu Recht je Schritt gerechnet |

### Der Platz für die Vorberechnungen ist schon bezahlt

`lbm.hpp:262` beschreibt `fac_geo` selbst: „8 float je Facette: … ,achse,**[6] reserviert,
[7] frei — 8 B/Facette ungenutzt**". `lbm.cpp:861` schreibt beide mit `0.0f`, niemand liest
sie. Slot 5 (`achse`) wird nur in `apply_facette` gelesen, und die Funktion wird unter IMEM
nie gerufen (`kernel.cpp:2916`). → **25 MB unbedingt frei, 37,5 MB solange IMEM läuft**,
alloziert und hochgeladen, von nichts gelesen. Genau der Platz für Wandlinkmaske, `nb` und
`ywb`. **Diese drei Vorberechnungen kosten null zusätzliches VRAM.**

### A2 (Spalding-Tabelle) ist der stärkere Vorschlag — und keine reine Sparmaßnahme

y⁺-Bereich aus `export/p4dt_deteps/yplus_facetten.csv` (n = 3 129 185): **1,15 … 983**,
daraus Y = u⁺·y⁺ = 1,52 … 2,19·10⁴, also 4,2 Dekaden. Genauigkeit **bevölkerungsgewichtet an
den 3,13 M echten Facettenwerten** (τ_w-Fehler):

| | max | p99 | Median |
|---|---|---|---|
| Newton it=3, FP32 (heute) | **4,36 %** | **2,95 %** | 0,124 % |
| Tabelle N=64 (256 B) | 0,167 % | 0,109 % | 0,0037 % |
| **Tabelle N=256 (1 kB)** | **0,010 %** | **0,0066 %** | 0,00023 % |

23,8 % der Facetten liegen bei Y > 2400, wo der Kopfkommentar `kernel.cpp:1656` selbst
−0,44 % dokumentiert; 0,13 % über 10⁴ (dort −4,4 %). Derselbe Kommentar sagt „bei hohem
Re_tau Iterationszahl erhöhen" — **bei 4 mm ist der Fall eingetreten und die Zahl wurde nie
erhöht.** Die Tabelle ist also rund **450× genauer (p99) und spart 330 Instruktionen.**

Zur Bauform, und das ist nicht verhandelbar: **globaler Puffer, kein `__constant`, kein
privates Array** — ein laufzeitindiziertes privates Array ist exakt die Scratch-Falle, die
dieser Fork schon einmal mit Faktor 100 bezahlt hat. Gemessen mit globalem Puffer:
private 0 / spill 0 auf beiden Geräten.

**Kombination Variante B + Spalding-Tabelle: 7221 → 6700 = −7,2 % (B70), 8396 → 7776 = −7,4 %
(iGPU)**, private 0 / spill 0 beidseitig.

### Mehrfachberechnungen

1. **`fid` zweimal je Facettenzelle und Schritt im selben Launch** — `kernel.cpp:1970` und
   `:3090`. Einmal gerechnet und durchgereicht: −21 Instr. plus 2 globale uint-Reads.
   (Deckt sich mit dem `cbit`-Befund aus 3.2.)
2. **Zwei Kernel über dieselbe Zellliste im selben Schritt** — `sgs_fdwand` (`:4677`) und
   `fac_nachbar_ab` (`:4754`), beide über `gd_zellen`, beide mit `neighbors(n,j)`, beide
   dieselben flags. Zusammen 1463 Instr. in zwei Starts; zusammengelegt und mit den beiden
   Vorberechnungen **707 in einem Start**. (Zweiter unabhängiger Agent, gleiche Empfehlung.)
3. **Dasselbe Gate-Prädikat dreimal für dieselben 18 Werte** — −130 Instr. für zwei davon.
4. **Host liest `fac_tau_n` ganz vom Gerät, um ein Maximum zu bilden** (`setup.cpp:3534`):
   12,5 MB je Sample für einen **monoton wachsenden** Zähler, dessen 2²⁰-Schwelle genau
   einmal im Lauf überschritten wird.

### Geprüft und verworfen

`coordinates(n)`, `f_bbox`, `is_halo` sind reine Arithmetik. `fac_q` und `fac_geo[0..4]` sind
**bereits** vorberechnet — richtig gemacht. `tau0 = 1/def_w` und `a_ = 1/def_sgs_sism_T` sind
Compile-Zeit-Konstanten, der Übersetzer faltet sie. Der SGS-Block in `stream_collide` ist echt
schnell (Pi-Tensor aus fneq je Schritt); konstant ist daran nur der fid-Lookup.

### Was Runde 2 hier nicht prüfen konnte

- **Laufzeit** — alles sind statische Instruktionszahlen, ein Lauf war gesperrt.
- **Ob G11/G22/G12 in der Praxis doch langsam drehen.** Das wäre die einzige abgeschwächte
  Rettung für A1 (Momente nur alle k Schritte neu projizieren). Messbar wäre die
  Winkeländerung von u_t je Facette und Schritt — dafür gibt es heute keinen Ausgang.
- **Bitgleichheit.** Variante B ist algebraisch exakt, aber die Summationsreihenfolge ändert
  sich; die Tabelle erst recht. Beide brauchen ein gepaartes A/B.
- **exp/log-Latenz.** `instCount` zählt eine transzendente Instruktion wie eine Addition —
  der Laufzeitgewinn der Spalding-Tabelle ist eher **größer** als 4,6 %.
- Ob der 1-kB-Tabellenpuffer im Konstantcache landet.

## 3.4 Sparse Tiles — Portierung, Wanduhr-Rechnung, Verdikt

Auftrag von Heiko (11.09.): lässt sich Block-Tiling doch nutzen, ohne nennenswerte
Einbuße — durch andere Einbindung, andere Codierung, vielleicht mehrere Kachelgrößen
gleichzeitig?

### Die entscheidende Zahl: das Nahfeld hat KEIN Verlangsamungsbudget

**Das ist eine Korrektur an meiner eigenen Vorgabe.** Ich hatte den Agenten mitgegeben,
eine Verlangsamung des Nahfelds um bis zu 12 % koste kaum Wanduhr. Das vertauscht
kritischen und unkritischen Pfad. Selbst nachgerechnet und am Lauflog belegt
(`[PHASEN]`-Zeile in `logs/p4dt_deteps.log`: Kopplung 0,9 % | Nahfeld 4 Schritte 95,8 % |
Fernfeld synchronisieren und entnehmen 2,3 % | Kräfte 0,9 %):

| Posten | Wert |
|---|---:|
| Grobschritt-Zyklus (Wanduhr) | 434,6 ms |
| davon Nahfeld (95,8 %) | **416,35 ms** |
| Fenster fürs Fernfeld (0,9 % + 95,8 %) | 420,26 ms |
| Fernschritt gemessen | 369 ms |
| Schlupf | 51,26 ms |

**Der Schlupf gehört dem Fernfeld, nicht dem Nahfeld.** Das Fernfeld läuft asynchron
daneben und dürfte 13,9 % langsamer werden, ohne einen Millimeter Wanduhr zu kosten. Der
Taktgeber ist das Nahfeld:

- Verlangsamung um y: `Wanduhr = 434,6 + 416,35·y` — **ab dem ersten Prozent, linear,
  ohne Freibetrag.**
- Beschleunigung um x: `Wanduhr = 434,6 − 416,35·x`, gültig nur bis **x = 12,31 %**.
  Maximal erreichbarer Gewinn **51,3 ms = 11,79 % Wanduhr**, danach bindet die iGPU.

| Nahfeld langsamer | Wanduhr | Aufschlag |
|---:|---:|---:|
| 3 % | 447,1 ms | +2,87 % |
| 9 % (V1: WG, T=16) | 472,1 ms | **+8,62 %** |
| 12 % (V1: WG, T=8) | 484,6 ms | **+11,50 %** |
| 40 % (v2 heute, naiv T=8) | 601,1 ms | **+38,32 %** |

**Tiling kauft VRAM gegen Wanduhr, und zwar sofort.**

### Warum das naive Tiling 40 % kostet — die Ursache ist nicht die Indirektion

V1s Quellkommentar (wortgleich in `src/kernel.cpp:958`) nennt den `tile_slot`-Gather als
Ursache. Der größere Teil ist die **DDF-Kontiguität**. Layout `fi[slot·T³·Q + i·T³ + loc]`:

| Dispatch | 64 Threads decken ab | DDF-Zugriff je Richtung |
|---|---|---|
| dicht | 64 aufeinanderfolgende x | **128 B zusammenhängend** |
| naiv T=8 | 64 x = **8 verschiedene Tiles** | 8 × 16 B → 8 Cache-Zeilen für 128 B = **4× Verkehr** |
| naiv T=16 | 64 x = 4 Tiles | **2× Verkehr** |
| **WG=Tile** | eine z-Lage **einer** Tile | **128 B zusammenhängend, wie dicht** |

Der DDF-Strom ist 79,9 % des Verkehrs. Das Modell erklärt V1s Messreihe (naiv −40 %/−28 %,
WG −12 %/−9 %) ohne Zusatzannahme. **Wer Tiling will, braucht WG=Tile. Das naive Tiling,
also v2s heutiger Stand, ist keine ernsthafte Option.**

### V1s Bauform ist in v2 nicht 1:1 portierbar — sie spillt

Gemessen am Offline-Compiler, Repo nachweislich unverändert (`git status` leer, selbst
geprüft). Varianten: **A** ohne Tiling · **B** naiv (v2 heute) · **C** WG=Tile exakt wie V1
· **D** WG=Tile mit Remat · **E** geteiltes `cbj` ohne WG (Ursachentrennung).

| Variante | Gerät | private | **spill** | **instCount** | Δ zu A | Δ zu B8 |
|---|---|---:|---:|---:|---:|---:|
| A ohne Tiling | B70 | 0 | 0 | **6932** | — | |
| B8 naiv T=8 | B70 | 0 | 0 | 7722 | +11,40 % | — |
| **C8 WG, V1 1:1** | B70 | 0 | **1152** ❌ | 7456 | +7,56 % | −3,4 % |
| **E8 geteiltes cbj, kein WG** | B70 | 0 | **1216** ❌ | 7355 | +6,10 % | −4,8 % |
| **D8 WG + Remat** | B70 | 0 | **0** ✅ | **7978** | +15,09 % | **+3,32 %** |
| C8 WG, V1 1:1 | iGPU | 0 | **576** ❌ | 8419 | +5,77 % | |
| **D8 WG + Remat** | iGPU | 0 | **0** ✅ | 9052 | +13,72 % | +2,69 % |

**Variante E beweist die Ursache:** der Spill kommt nicht vom WG-Dispatch, sondern davon,
dass `cbj[]` (10 lebende 64-Bit-Basen = 20 GRF-Dwords) über die ganze Facettenkette am Leben
bleibt. Das ist exakt das Gegenteil der **Rang-1-Remat** (`kernel.cpp:3612-3618`), die in v2
Spill 448/832 → 0/0 gebracht hat. **V1s „Perf-Befund-1" ist in v2 nicht bezahlbar.**
Variante D ist spillfrei in allen vier Gate-Armen auf beiden Geräten, kostet aber +3,3 %
gegenüber dem naiven Tiling.

`private_size = 0` in allen Varianten — Tiling löst die Scratch-Falle nicht aus. Über den
heute gebauten Gesamtdeckungs-Modus geprüft: auch kein anderer Kernel.

### Kein v2-Pfad sperrt die Portierung — aber zwei Fallen sind v2-eigen

In `stream_collide` geht **jeder** `fi`-Zugriff durch `load_f`/`store_f`; die gesamte
Facetten-, SGS- und Bandkette arbeitet auf dem Registerarray `fhn` und auf linear indizierten
Feldern, sie fasst `fi` nicht an. `sgs_fdwand`, `fac_nachbar_ab`, `sgs_gdiag`, `schale_blend`
laufen über eigene Listen mit flachem Dispatch und haben den `is_dead_tile`-Ausstieg bereits.
V1 wendet WG=Tile ohnehin nur auf `stream_collide` an.

Die Doppeldomäne sperrt **nicht**: `lbm.cpp:533` prüft `get_D()>1u`, aber die Doppeldomäne
sind **zwei getrennte LBM-Objekte** mit je D=1 — die Prüfung feuert nie. Dass nur das Nahfeld
Tiling bekommt, regelt der Read-once-Schalter (`lbm.cpp:256-258` nullt ihn sofort).

Zwei v2-eigene Fallen, beide selbst nachgeprüft:

1. **Slot 0 ist ein Papierkorb** (`lbm.cpp:1071-1079`). V1 zählt ab 0 und benutzt `wg_slot`
   direkt als fi-Slot. **Ein 1:1-Kopieren schreibt die gesamte Simulation um eine Tile
   versetzt** — kein Absturz, still falsche Physik. Der Kommentar dort hält fest, dass genau
   daran die ersten T=8- und T=4-Läufe divergiert sind (Cd 18,4 bzw. 22,4).
2. **`CFD_TILE=4` ist in v2 hart verboten** (`setup.cpp:5683`: T muss 8, 16, 32 oder 64
   sein). Die in Teil 3.1 genannte Zeile „T=4 → 1661,3 MiB" ist eine reine Rechengröße, kein
   in v2 erreichbarer Zustand.

### Was Tiling überhaupt einbringt

| T | frei | = Anteil toter Zellen | `tile_slot` | Padding-Zuschlag |
|---:|---:|---:|---:|---:|
| 8 | 1287,8 MiB | 6,845 % | 3,96 MiB | +2,39 % |
| 16 | 450,1 MiB | 2,392 % | 0,51 MiB | +5,38 % |

Nur 6,8 % des Gitters fallen bei T=8 weg, obwohl 12,197 % der Zellen echt solid sind — den
Rest frisst der zwingende 2-Zell-Halo. Nebenbei: der Quellkommentar `kernel.cpp:958`
behauptet, `tile_slot` passe nicht in L1/L2. Bei **0,51 MiB** für T=16 ist das unplausibel.

### Portieraufwand

Rund 95 Codezeilen, mit projektüblicher Kommentardichte 150–200, davon 75 im Kernel. Der
Patch existiert als lauffähige Messfassung im Scratchpad und ist **nachweislich inert, wenn
`SPARSE_TILES_WG` aus ist** (gepatchter Kernel mit WG aus liefert instCount bitgleich zum
unveränderten Repo). Die drei riskantesten Eingriffe: Registerdruck (bereits eingetreten),
der Slot-Versatz +1, und die Parameter-Reihenfolge — `tile_slot` ist per `TS_P` der letzte
Parameter jedes fi-Kernels, `active_tile_id` muss dahinter, bei einem Kernel, dessen
Facettenparameter zusätzlich positionsgebunden nachgebunden werden. **Ein Versatz um eine
Position bindet `fac_geo` als `tile_slot`.**

### Verdikt

**Die Portierung lohnt heute nicht — nicht wegen des Aufwands, sondern wegen der
Wanduhr-Rechnung.** Selbst V1s bester Wert (−9 bis −12 % Nahfeld) kostet hier **+8,6 bis
+11,5 % Wanduhr**, also 37 bis 50 ms je Grobschritt, für 1288 MiB (T=8) bzw. 450 MiB (T=16).
Bei T=16 ist das Verhältnis besonders schlecht: 450 MiB für 8,6 % Wanduhr.

**Die Ausnahme, in der es sich lohnt:** wenn eine Rechnung sonst **gar nicht** in den
Speicher passt. Dann ist +11,5 % Wanduhr der Preis dafür, dass sie überhaupt läuft — genau
die Rolle, die `kernel.cpp:960` selbst beschreibt („ein VRAM-gegen-Tempo-Regler … kein
genereller Gewinn"). Mit 1288 MiB bei T=8 deckt es den am 29.08. gemessenen 516-MB-Fehlbetrag
der verbreiterten y-Box bei 4 mm mit Reserve.

### Der billigste Weg zu einem belastbaren Ja/Nein

**Stufe 1 — null Codezeilen, ~2 min GPU, entscheidet in 80 % der Fälle.** Nahfeld allein
(Einzeldomäne) auf der B70, 25-s-Paar über die Queue: `CFD_SPARSE_TILES=0` gegen
`=1 CFD_TILE=8` und `CFD_TILE=16`, Wanduhr je Schritt. Liegt das naive Tiling unter ~3 %, ist
der ganze Port gegenstandslos. Liegt es bei −30 bis −40 % wie in V1, ist das
Fragmentierungsmodell bestätigt.

**Stufe 2 — nur wenn Stufe 1 es rechtfertigt UND der VRAM wirklich gebraucht wird:** die
~95 Zeilen der Remat-Fassung bauen, `scratch_gate.sh` über alle Arme, dann CPU → iGPU → B70,
dann dasselbe 25-s-Paar.

**Nicht mehr messen, weil entschieden:** V1s geteiltes `cbj` — 1152/1216 B Spill sind offline
bewiesen.

### Offener Quelltext-Defekt

`src/kernel.cpp:956-957` führt „T=8: −40 % Durchsatz, **1,43 GB** gespart / T=16: −28 %,
**0,77 GB**" als v2-Zahlen. Das sind wörtlich V1s Erstversuchszahlen. v2s eigene Ersparnis
ist **1287,8 MiB bzw. 450,1 MiB**. Dieselbe Verwechslung wurde im README bereits korrigiert
(Commit e16e288), im Quelltextkommentar steht sie noch. **Wird nach dem laufenden A/B
berichtigt** — der Kommentar liegt im R()-stringifizierten Bereich, das braucht die
Klammerfallen-Prüfung und keinen Eingriff während einer laufenden Messung.

## 3.5 Gemischte Kachelgrößen 8³/16³/32³/64³/128³ — die Antwort ist nein, und sie ist beweisbar

Datengrundlage: eigene Auszählung am **Flag-Export des Produktionslaufs**
(`export/p4dt_deteps/feld_nah_000501ms.vtk`, 519 139 485 Bytes, 1:1 das Gitter).
Eichprobe: die Kachelzählung reproduziert die Werte aus Teil 3.1 exakt (T=4 → 7 395 260 von
8 215 506 und 1 661,3 MiB; T=8 → 944 536 und 1 287,8 MiB; T=16 → 123 710 und 450,1 MiB).

### Die Fahrbahnplatte ist ein Phantom

| Klasse | Flag | Zellen | Anteil |
|---|---|---:|---:|
| Fahrbahn ruhend z=0 | `0x01` | 1 116 429 | 0,215 % |
| Fahrbahn bewegt z=1 | `0x03` TYPE_MS | 1 102 365 | 0,212 % |
| **Fahrzeug, voxeliert + lochgefüllt** | `0x41` | **62 201 072** | **11,982 %** |
| Kopplungsrand TYPE_E | `0x02` | 3 290 677 | 0,634 % |
| Fluid | `0x00` | 451 428 942 | 86,957 % |

**Die Platte ist zwei Zellen dick.** Zerlegt nach Tiefe (Abstand ≥ 3 zur nächsten aktiven
Zelle, genau die Halo-2-Bedingung):

| | solid | davon tief | Anteil an der Obergrenze |
|---|---:|---:|---:|
| Platte z=0+1 | 80,7 MiB | **0,6 MiB** | **0,03 %** |
| Fahrzeug z≥2 | 2 253,8 MiB | **2 034,8 MiB** | **99,97 %** |

Die Idee „große Kacheln über der Platte, feine am Fahrzeug" hat keinen Gegenstand: der
2-Zell-Halo frisst die Platte vollständig. **Der gesamte einsparbare Bestand liegt im
Fahrzeuginneren**, einem kompakten, aber krummen Körper. Große Kacheln haben dort nichts zu
holen, was kleine nicht auch holen.

**Harte Obergrenze jeder Halo-2-Kachelung: 2 035,4 MiB, nicht 2 294,6 MiB.** Der Halo allein
kostet 259,2 MiB (11,3 %), bevor überhaupt ein Korn gewählt ist.

### 64³ und 128³ kosten Speicher, sie sparen keinen

Selbst nachgerechnet, Aufrundungspolster gegen das dichte Gitter:

| T | Raster | Polster | fi frei netto | % von 2 035,4 |
|---:|---|---:|---:|---:|
| 4 (**in v2 gesperrt**) | 423×166×117 | +31,3 MiB Tabelle | **1 630,0** | 80,1 % |
| **8** | 212×83×59 | +449,4 MiB | **1 283,9** | 63,1 % |
| 16 | 106×42×30 | +1 011,9 MiB | 449,6 | 22,1 % |
| 32 | 53×21×15 | +1 011,9 MiB | 106,7 | 5,2 % |
| **64** | 27×11×8 | **+3 758,6 MiB** | **−3 074,6** | — |
| **128** | 14×6×4 | **+6 722,6 MiB** | **−6 722,6** | — |

Bei 128³ sind 335 von 336 Kacheln aktiv — es gibt schlicht nichts mehr wegzulassen, und das
Polster allein ist größer als alles, was je einzusparen wäre.

### Ein gemischtes Schema spart am fi-Puffer exakt null Byte

Exakt gerechnet als Oktree-Optimierung über das ganze Gitter
(`kosten(Knoten) = 0` wenn tot, sonst `min(T³, Σ kosten(Kinder))`):

```
Blattgrößen 4…128 : fi frei = 1 661,3 MiB   ← identisch flach T=4
Blattgrößen 8…128 : fi frei = 1 287,8 MiB   ← identisch flach T=8
```

**Das ist kein Messergebnis, das ist ein Satz.** Acht Kinder der Kante T/2 überdecken genau
T³ Zellen, tote Kinder kosten 0, also ist `Σ Kinder ≤ T³` **immer**. Eine grobe Kachel kann
nie billiger sein als ihre Unterteilung, bestenfalls gleich teuer. Auf T=16 wurden 108 426
von 123 710 Knoten als Blatt genommen — ausnahmslos als **Gleichstand**, nie als Gewinn.

Der einzige Gewinn eines gemischten Schemas liegt in der **Indextabelle**:

| Schema | fi frei | Tabelle | netto | % von 2 035,4 |
|---|---:|---:|---:|---:|
| flach T=8 | 1 287,8 | 3,96 | 1 283,9 | 63,1 % |
| flach T=4 | 1 661,3 | 31,34 | 1 630,0 | 80,1 % |
| flach T=2 | 1 909,9 | 248,60 | 1 661,3 | 81,6 % |
| gemischt F=4 / C=16 | 1 661,3 | **4,24** | 1 657,1 | 81,4 % |
| **gemischt F=2 / C=8** | 1 909,9 | **19,42** | **1 890,5** | **92,9 %** |

Ein gemischtes Schema kann also mehr holen — aber **nicht dort, wo die Frage es vermutet**.
Nicht durch grobe Kacheln über dem Freistrom, sondern dadurch, dass die zweistufige Tabelle
ein **noch feineres** Korn bezahlbar macht. Bei den Körnern, die man durchsatzseitig
überhaupt erwägen würde (T=8, T=4), beträgt der gemischte Gewinn **0,8 bzw. 27,1 MiB** —
0,04 % bzw. 1,7 %. Das ist Rauschen.

### Der Preis der gemischten Adressrechnung

Billigste gefundene Variante: feines Korn als Allokationseinheit, Grobtabelle mit
**Größenmarke im Hochbit** — homogen lebendige Grobkacheln leiten ihren Slot arithmetisch ab,
nur gemischte nehmen eine zweite, **datenabhängige** Last. Gemessen, B70, `private=0
spill=0` überall:

| Arm | instCount | gegen dicht | gegen flach |
|---|---:|---:|---:|
| dicht | 6 875 | — | |
| flach T=4 / T=8 | 7 679 | +11,7 % | — |
| **gemischt (C=16 und C=32)** | **8 255** | **+20,1 %** | **+7,5 %** |

Die übrigen Kernel zahlen mehr: `update_fields` 1 295 → 1 842 → 2 179, `update_force_field`
1 215 → 1 643 → 1 942, `initialize` 1 410 → 1 623 → 1 914.

Verkehr: v2 löst die Basen **zweimal** auf (`load_f` bei `kernel.cpp:1473-1475`, `store_f`
noch einmal bei `:1492-1494`). Das ergibt 38,54 GB zusätzliche Gather je feinem Schritt gegen
43,36 GB heute — **+88,9 %**, oder +74,2 B je Gitterzelle gegen 83,5 B heute. **Die
Indirektion verdoppelt den nominellen Verkehr nahezu.**

**Struktureller Killer:** der einzige bekannte Hebel gegen die Durchsatzstrafe ist
Workgroup=Tile, und der **setzt eine feste Kachelkante voraus**. Bei gemischten Größen
bräuchte er einen Launch je Größenklasse, und die Nachbarauflösung über Kachelgrenzen bliebe
trotzdem der volle zweistufige Resolver. Das gemischte Schema ist ausgerechnet mit seiner
einzigen Reparatur schlecht verträglich.

### Der Gegenentwurf ohne jede Tiling-Maschinerie: Zuschnitt

Randschichten, die **ohne Verlust einer aktiven Zelle** abgeschnitten werden könnten:

| Achse | voll-solide Randschichten |
|---|---|
| z | **1** (z=0) / 0 |
| y | 0 / 0 |
| x | 0 / 0 |

**Verlustfreier Zuschnitt: 0 MiB.** Die eine Schicht ist die Bounce-back-Wand, die bleiben
muss. Fünf der sechs Domänenflächen sind TYPE_E-Kopplungsrand.

Mit Physikänderung dagegen ist es der billigste Hebel überhaupt — null Instruktionen, null
Indirektion, null Bitgleichheitsrisiko:

| | Zellen | alle Puffer (57,19 B/Zelle) |
|---|---:|---:|
| eine z-Schicht (4 mm Höhe) | 1 116 429 | **60,89 MiB** |
| eine y-Schicht | 785 385 | 42,84 MiB |
| eine x-Schicht | 307 365 | 16,76 MiB |

**21,1 z-Schichten = 84,6 mm Bauhöhe oben weg ersetzen den gesamten T=8-Gewinn** (1 287,8
MiB). 27,3 Schichten = 109 mm ersetzen T=4.

**Und trotzdem ist das kein Vorschlag.** `AUDIT-BEFUNDE.md:5343-5349` hält für genau dieses
Gitter fest: Abstände ab Hülle z+ **651 mm**, y ±401 — und nennt als Ziel eines größeren
Geräts ausdrücklich `z+ 651 → 1102..1208 mm`. **Der Nahkasten gilt im Projekt bereits als zu
knapp, nicht als zu groß.** Ein Zuschnitt bewegt sich gegen den dokumentierten Bedarf. Das
ist eine Entscheidung des Projektleiters, keine Baumaßnahme.

### Verdikt

**Kein gemischtes Schema.** Am fi-Puffer spart es beweisbar null; sein ganzer Gewinn ist die
Indextabelle, und bei jedem erwägbaren Korn sind das 0,8 bis 27 MiB. Erkauft mit +7,5 %
Instruktionen über der flachen Variante, einer datenabhängigen zweiten Last und
Unverträglichkeit mit Workgroup=Tile. „Grobe Kacheln über dem Freistrom" trägt gar nicht:
64³ und 128³ kosten 3,1 bzw. 6,7 GiB Polster.

**Wenn Tiling, dann eine Einzelgröße.** Die Analyse empfiehlt **T=4** (netto 1 630,0 MiB
gegen T=8s 1 283,9, bei identischer Instruktionszahl — die 31-MiB-Tabelle kostet Cache, keine
Befehle). ⚠ **Dagegen steht, dass `CFD_TILE=4` in v2 an drei Stellen gesperrt ist**
(`setup.cpp:4519`, `:5066`, `:5683` — eine reine Positivliste 8/16/32/64 ohne dokumentierte
Begründung). Die Sperre wäre also zu heben, bevor T=4 überhaupt messbar ist. Ob der
Cache-Nachteil der 31-MiB-Tabelle den Mehrgewinn frisst, ist die eine fehlende Messung.

**Vorrangig aber: Tiling lohnt derzeit gar nicht** — das Durchsatzbudget ist null (3.4). Vor
der Tiling-Maschinerie stehen zwei billigere Hebel: `CFD_FAC_KDIAG=0` bringt **191 MiB** ohne
Physikänderung, und wenn die Indirektion je gebaut wird, gehört sie im selben Zug auf
`rho`/`u`/`flags` gezogen — **+576,1 MiB für dieselbe, bereits aufgelöste Adresse**, was das
Verhältnis von Gewinn zu Adressrechnung um 45 % verbessert.

### Was auch Runde 2 hier nicht messen konnte

1. **Durchsatz von SPARSE auf der B70 in v2** — die −40 % sind eine V1-Zahl. Rezept:
   8-mm-Paar `CFD_SPARSE_TILES=0/1` bei `CFD_TILE=8` über die Queue, im selben Zug Kräfte
   auf Bitgleichheit.
2. **T=4 gegen T=8 als Cache-Experiment** — identische Instruktionszahl, Tabelle 31,3 gegen
   3,96 MiB, der Durchsatzunterschied **ist** die Cache-Wirkung. Zwei 25-s-Läufe genügen.
   Die L2-Größe der B70 ist nirgends im Repo belegt.
3. **Der gemischte Adressierer wurde nur übersetzt, nie ausgeführt** — der Host-Code für die
   zweistufige Slot-Vergabe existiert nicht.
4. **Instruktionszahl ≠ Durchsatz** — die Strafe sitzt im Gather, nicht in der Arithmetik.
   Die +804/+1 380 sind eine untere Schranke.


---

# Teil 4 — Die Zeit- und Architekturachse (11.09.2026 abends)

Fünf Agenten, jeder auf einer Achse, die **keine** der früheren Runden hatte. Der Anlass war
Heikos Beobachtung, dass alle bisherigen Briefings „was kostet dieser Kernel je Schritt"
fragten und nie „wann und in welcher Qualität muss das gerechnet werden".

**Alle Zahlen hier sind Wanduhr, Verkehr oder eingesparte Schritte. Keine Instruktionszahl.**

## 4.1 Der Maßstab, den niemand gezogen hatte

`u_lat = 0,075`. Das Strömungsfeld braucht **13,3 feine Schritte, um eine Zelle
weiterzurücken**. Jedes Teilmodell, dessen Eingang eine advehierte makroskopische Größe ist,
ist um diesen Faktor überabgetastet. Nur die Kollision und die Randaufprägung gegen die
Zwei-Schritt-Mode haben eine Eigenzeit von 1–2 Schritten.

**Umrechnung Verkehr → Wanduhr, aus einem gemessenen Paar geeicht:** `d8_kdiag_an/aus`,
1,68 % Verkehr ergaben 1,33 % Nahfeld-Wanduhr → **Faktor 0,79**.

## 4.2 Der größte Einzelposten: rho und u bedienen eine Leserschaft unter 2 %

**Zwei Agenten haben das unabhängig gefunden.** Selbst nachgeprüft: im ganzen
`stream_collide` (Zeilen 2866–3650) gibt es genau **sieben** Zugriffe auf `u[]`/`rho[]`, und
die drei lesenden stehen im **TYPE_E-Zweig** — 0,63 % der Zellen. Jede andere Zelle rechnet
beides aus `fhn` neu.

Geschrieben werden sie für **87,8 % der Zellen, jeden Schritt**: 7,29 GB = **16,8 % des
Verkehrs**. `defines.hpp:55-62` beziffert den Preis selbst mit „10 bis 15 Prozent Durchsatz".

| Variante | Verkehr | VRAM | projizierte Wanduhr |
|---|---|---|---|
| nur schreiben, wo gelesen wird (F-BBox + Auslassebene, Substep < ratio) | −8,2 bis −12,1 % | — | **−6,5 bis −9,5 %** |
| beide auf 2 Byte je Komponente | −8,4 % | **−3 961 MiB** | −7,2 % |
| `rho` nicht jeden Schritt | −4,2 % | — | −3,6 % |

**Und der elegante Teil: `rho` und `u` sind inhaltlich längst halbgenau.** Sie werden jeden
Schritt aus den FP16S-DDFs zurückgerechnet; das Rauschen dieser Rückrechnung ist am echten
Feld gemessen **rms 6,06e-6** für rho. Der FP32-Puffer bewahrt eine Genauigkeit auf, die sein
Inhalt nicht hat.

**Die Formatfrage ist am echten Feld beantwortet, und die Antwort ist nicht die naheliegende:**

| Format für rho | rms-Fehler | Δcp_rms |
|---|---:|---:|
| `half(rho)` roh | 1,83e-4 | **0,0217 — tödlich** |
| **`FP16S(rho−1)`** | **3,36e-7** | **4,0e-5** |
| int16 auf ±0,5 | 4,41e-6 | 5,2e-4 |

FP16S auf der Störform liegt **18-fach unter dem Rauschen, das die Rückrechnung ohnehin
erzeugt**. Für `u` dagegen ist Festkomma falsch, obwohl die Bulk-Zahlen dafür sprechen: an der
Wand liegt u_t um Faktor 8,5 niedriger, int16 wäre dort 47× schlechter **und kippt Gates** —
`fac_nb` kodiert mit 0 und −1 zwei Sonderfälle, und `apply_facette_imem` steigt bei
`ut<1e-6f` aus. FP16S mit Untergrenze 1,86e-9 nicht.

**Risiko, benannt:** die Maske ist der gefährliche Teil. Eine übersehene Leserzelle liefert
einen Wert aus einem beliebig alten Schritt, lautlos. Bauform: Maske konstruktiv als
**Obermenge**, plus ein Kontrollarm, der alles schreibt und bitgleich sein muss.

**Der dominierende Vorbehalt bei der Formatfrage:** alle Quantisierungszahlen sind ein
**Standbild bei 501 ms**. Ob ein Fehler von 1e-5 je Schritt über 50 099 Schritte in der
Rückkopplung gedämpft wird oder driftet, sagt nur ein A/B-Lauf.

## 4.3 Die Zeitachse des Laufs — 41,8 % vergehen vor der ersten verwertbaren Abtastung

| Phase | Wanduhr | Anteil |
|---|---:|---:|
| Aufbau | 153,3 s | 2,83 % |
| **Anwärmphase 0…201 ms** | **2 113 s = 35,2 min** | **38,97 %** |
| Messphase 201…501 ms | 3 144 s = 52,4 min | 57,99 % |
| Endauswertung | 11,7 s | 0,22 % |

**Ein Prüfpunkt ist der tragfähige Weg, nicht das grobe Anwärmen.** `ARBEITSLISTE.md:559` sagt
selbst: „Kein Checkpoint im Code". Zustandsgröße ≈ 28,3 GB, Schreiben und Laden je ~20 s bei
gemessenen 1,41 GB/s → **netto ~34 min je Folgelauf**. Für Arme, die das Wandmodell anfassen,
kommt ein Nachlauf von ~100 ms dazu (netto ~17 min).

**Grob anwärmen und hochsetzen ist unterlegen** — und der Grund ist hart: die Schrittzeit des
Grobgitters ist aus keinem Log ableitbar, weil `run_async` nie `info.update` ruft. Bei
realistischen 250–430 ms kostet ein Grob-Warmlauf 21–36 min, der Hebel verschwindet.

## 4.4 Der Befund, der kein Performance-Befund ist

**Das Messfenster beginnt mitten im Einschwingen.** Das SGS-Modell wird bei 150 ms
scharfgeschaltet (`CFD_SGS_SISM_AB=15000`), die Mittelung beginnt bei 201 ms, der Vorgang
klingt mit τ ≈ 12–16 ms ab und braucht 92–114 ms bis 99 %.

| Zeit | Cd |
|---|---:|
| 140 ms | +9,20 |
| 150 ms (Modell scharf) | +4,57 |
| **201 ms (Messung beginnt)** | **−2,95** |
| 290 ms | −3,95 |
| 450 ms | −3,68 |

Bias auf das Fenstermittel: **+1,34 % auf `cd_druck`**, −2,16 % auf Cd. Für **gepaarte A/B ist
das harmlos** (beide Arme tragen ihn), für **jede Absolutaussage gegen OF13 nicht** — und das
ist die offene Hauptfrage dieses Projekts. `CFD_T_WARMUP=0.29` nimmt den Anlauf heraus **und**
spart 15,6 min.

Zur Einordnung: die Nahfeldbox wird vor Messbeginn nicht einmal **einmal** durchspült
(0,225 s Durchspülzeit gegen T_WARMUP 0,201 s).

## 4.5 Kadenz — was darf seltener rechnen

| Teilmodell | Eigenzeit | Verdikt |
|---|---|---|
| `boden_eq`/`einlass_eq` | 2 Schritte (Staggered-Mode) | muss jeden Schritt |
| Druckauslass | akustisch, 1,73 Schritte/Zelle | muss jeden Schritt |
| 3×3-Kaskade, Lösung | aktuelle Populationen | muss jeden Schritt |
| SISM-EMA, `fac_wfd` | T = 5000 Schritte; Sbar-Drift 1e-6/Schritt | **kadenzierbar, k ∈ {2,4}** |
| `fac_nb` | advektiv, 13,3 Schritte/Zelle, **kein Zähler, kein t-Argument** | **kadenzierbar, zuerst** |
| Kräfte | τ = 21–41 ms **gemessen**, Kadenz 1 ms | **kadenzierbar, k ≥ 4** |

**Zwei scharfe Nebenbedingungen:** k muss `ratio` **und** `def_zaehl_takt` teilen, sonst
schweigen die Wirkpfadzähler still. Und `a_ = 1/def_sgs_sism_T` muss zu `k/T` werden, sonst
wird aus T = 5000 still T = k·5000.

**Die vermeintliche Falle ist keine:** dass `fac_nachbar_ab` wegen Bitreproduzierbarkeit aus
`stream_collide` ausgelagert wurde (03.09.), ist genau die Eigenschaft, die eine Kadenz
**erlaubt** — der Puffer wird ohnehin erst im nächsten Schritt gelesen.

## 4.6 Arbeitsteilung: der Schnitt liegt bei 93,1 % des Optimums

**T_fern = 1,82 ns × N_fern**, gemessen an einer 12-Punkte-Leiter, flach auf ±0,9 % über
Faktor 15,5 in der Fallgröße. Der vermutete Einbruch an der 4095-MB-Puffergrenze **existiert
nicht**. Geräteverhältnis **B70 : iGPU = 9,43 : 1**, CPU = 1/54 der B70.

Die iGPU kann damit **nie mehr als 9,59 %** der Gesamtleistung tragen; sie trägt 8,93 %.
**Das ganze Umverteilungsthema ist 1,61 % Wanduhr groß.**

| Szenario | Wanduhr | Δ |
|---|---:|---:|
| heute | 90,4 min | — |
| **perfekte Balance, Verkehr wie er ist** | **89,0 min** | **−1,61 %** |
| Nahfeld kostenlos (Arbeit gelöscht, nicht verschoben) | 83,2 min | −8,16 % |

**Drei Vorschläge sind damit erledigt:**
- **Das Fernfeld ist kein Komplement, sondern eine Überlagerung** — es rechnet auch unter dem
  Nahkasten (Fußabdruck nur 4,04 %). Ein kleinerer Nahkasten gibt ihm **keine** Arbeit, und
  das Überspringen des Fußabdrucks läge mit 14,9 ms **komplett im Schlupf**.
- **Nahkasten beschneiden:** 159 mm in z+ oder 113 mm je y-Seite bis zum Boden — aber
  `AUDIT-BEFUNDE.md` B71 fordert die **doppelten** Abstände, und die geforderte Box bräuchte
  **52,3 GiB gegen 31,9 GiB Kapazität**.
- **Dritte Auflösungsstufe:** softwareseitig 11–26 Stellen, **aber es gibt keinen Ort dafür**.
  In den Schlupf passen 9,4 M Zellen bei 8 mm — ein Würfel von 1,7 m Kante, kleiner als der
  Nahkasten, den er umschließen soll.

**Nicht die iGPU ist das ungenutzte Gerät, sondern die CPU.** Sie steht **95,4 % der Laufzeit**
in Barrieren (`lbm.cpp:2584` synchronisiert nach jedem feinen Schritt), und es gibt im ganzen
Baum **keine asynchrone Ausgabe**.

**Der eine saubere Neubefund:** die vier `extract_plane_macros` laufen **nach**
`lbm_c.finish()`. Reiht man sie davor in dieselbe In-Order-Queue, erledigt die iGPU sie im
eigenen Schlupf. **0,5–1,0 % Wanduhr**, vier blockierende Syncs weniger, bitneutral erwartet.

## 4.7 Berechnet und nie gebraucht — 73,3 s im Remesh

**Gemessen über ein Dateizeitstempel-Paar**, nicht geschätzt: dasselbe Fenster braucht in
`p4_neu` (ELIBB an) **80,66 s** und in `p4eb_aus` (ELIBB aus, gleiches Gitter) **7,40 s**.
**Differenz 73,3 s = 1,35 % der Wanduhr.** Darin zwei rein berichtende Rechnungen:

- **Der TREPPE-Scan ist informationell leer.** Er schießt 15 114 078 Links ab und liefert
  `q: Mittel 0.5000, min 0.5000 | Achs 0.5000, Diag 0.5000` — selbst nachgeprüft im Log. Die
  Voxelfläche halbiert jeden Link per Konstruktion; das Ergebnis steht analytisch fest, bevor
  der Scan startet. Verbraucht wird es nur von einer `print_info`-Zeile.
- **Der STL-Doppelscan** (30,2 M Strahl/Dreieck-Schüsse) misst die Aufdickung — einen Pfad,
  den die Iron Rule „Kein STL-Rückgriff nach Voxelierung" ohnehin von jeder q-Quelle
  ausschließt.

**Kein Löschen**, sondern ein Emissionsgatter mit Default an, plus TREPPE als Selbsttest auf
einer kleinen Teilbox statt auf allen 15,1 M Links.

**Ein früherer Befund ist damit widerlegt:** „153 MB + 249 MB unbedingt geschrieben" ist als
**Laufzeithebel falsch**. Offline nachgebaut beträgt die reine Schreibzeit **3,09 s = 0,057 %**.
Die 73,3 s sind Rechnung, nicht Schreiben. Was bleibt, ist ein Plattenbefund: **13,7 GB
ungelesene Ausgabe** im Archiv.

**Drei Fallen geprüft und entschärft** — die Lehre aus der Volumenkraft hat sich bestätigt:
`fac_tau[4]`/`[5]` sehen nach Buchhaltung aus, speisen aber `cd_facetten.csv`, die sechs
Werkzeuge lesen. `schale_waechter.csv` und `band_bilanz.csv` haben keinen Leser, tragen aber
**vier Abbruchentscheide**.

## 4.8 Rangliste der offenen Hebel nach dieser Runde

| Hebel | Wieviel | Physik | Aufwand |
|---|---|---|---|
| **rho/u nur schreiben wo gelesen** | **6,5–9,5 % Wanduhr** | bitgleich beweisbar | hoch |
| **Prüfpunkt/Neustart** | **17–34 min je Folgelauf** | — | 200–300 Zeilen |
| **rho/u auf 2 Byte** | −3 961 MiB VRAM, −7,2 % | ändert Zahlen, Fehler beziffert | mittel |
| **Remesh-Diagnostik gattern** | 1,35 % | keine | klein |
| Kopplungsernte alle 2 Grobschritte | 1,65 % | Treppenhalteglied | sehr klein |
| `extract_plane_macros` vorziehen | 0,5–1,0 % | bitneutral erwartet | klein |
| Kräftekadenz 1 → 4 ms | 0,56 % | keine (41-fach überabgetastet) | eine Variable |
| `fac_nachbar_ab` auf k = 4 | ≥ 0,83 % | mittel | klein |
| 150-ms-VTK-Dump, der gelöscht wird | 9 s + 11,7 GB | keine | eine Zeile |
| **`CFD_T_WARMUP` auf 0,29** | **15,6 min UND +1,3 % Genauigkeit** | Bias entfällt | eine Variable |

**Erledigt und nicht weiterverfolgen:** ratio 4 → 8 (das Nahfeld würde um 0,60 % **wachsen**,
weil `CFD_NEAR_LY` durch 32 mm nicht aufgeht), dritte Auflösungsstufe, CPU als Rechengerät,
Nahkasten beschneiden, Fernfeld-Fußabdruck überspringen.
