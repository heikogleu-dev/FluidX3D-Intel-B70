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
