# Performance und VRAM — Befundlage und Massnahmenliste

**Stand 11.09.2026.** Konsolidiert aus drei Agentenrunden und eigener Nachprüfung. Die
vollständigen Rohbefunde mit allen Zwischenrechnungen stehen in
`PERFORMANCE-ROHBEFUNDE-2026-09-11.md` (1071 Zeilen); dieses Dokument ist die Arbeitsfassung.

**Messgrundlage:** Offline-Compiler `ocloc` über `werkzeuge/scratch_gate/igc_offline.sh`
(0xe223 = Arc Pro B70, 0x7d67 = Arrow-Lake-iGPU), Lauflog und Flag-Export von
`p4dt_deteps` (4 mm, 11.09.), Quelltext auf `master`. **Alle Prozentzahlen ohne
ausdrücklichen Gegenhinweis sind statische Instruktionszahlen, keine Laufzeiten.**

---

## 1. Die vier Zahlen, die alles begrenzen

| | Wert | Folge |
|---|---:|---|
| Nahfeldanteil am Grobschritt | **95,36 %** (399,7 ms von 419,1 ms) | Das Nahfeld ist der kritische Pfad |
| Schlupf des Fernfelds | 34,1 ms (369,3 ms Schritt in 403,4 ms Fenster) | gehört der **iGPU**, nicht dem Nahfeld |
| **Maximaler Wanduhr-Gewinn** | **8,13 %** | darüber hinaus bindet die iGPU |
| Anteil des DDF-Stroms am Verkehr | **79,9 %** von 43,36 GB je feinem Schritt | bei D3Q19/FP16S nicht reduzierbar |

> **⚠ BERICHTIGT am Abend des 11.09. — und die Korrektur ist selbst ein Befund.** Hier standen
> 434,6 ms, 51,26 ms Schlupf und 11,79 % Deckel. Diese Zahlen stammen aus `p4dt_deteps`, der
> Baseline **vor** dem Audittag. Der heutige Lauf `p4_neu` liegt bei **419,1 ms** (eigene
> Gegenrechnung: 5422 s Wanduhr minus 160 s Aufbau minus 12 s Abschluss, geteilt durch 12 526
> Grobschritte). **Der Schlupf ist keine feste Größe — er schrumpft mit jeder Nahfeld-Maßnahme.**
> Die heutige Runde hat rund ein Drittel des iGPU-Vorrats mitverbraucht: 51,3 → 34,1 ms. Wer
> gegen den Deckel optimiert, muss ihn nach jeder Maßnahme neu bestimmen.

**Verlangsamung des Nahfelds um y kostet `399,7 ms · y` Wanduhr, ab dem ersten Prozent,
ohne Freibetrag.** Beschleunigung bringt `399,7 ms · x`, gültig nur bis **x = 8,53 %**.

Der Kernel erreicht 420 GB/s von 608 GB/s Spitze (69 %) und 81 % dessen, was Upstream auf
derselben Karte erreicht (520 GB/s). **Der wahre Nahfeld-Durchsatz ist 5028 MLUPs**, nicht
die angezeigten 1946 — siehe Abschnitt 4.

---

## 1a. Umgesetzt und abgenommen (11.09.2026)

**E1, E2 und E3 sind gebaut und am 8-mm-Fahrzeug gegen einen VOR dem Umbau gefahrenen
Bezugslauf geprüft.** Reihe `logs/opt_8mm_serie.txt`, Arme `o8_vor` (Binary 9c33b417) und
`o8_nach` (Binary eb56af0b), wortgleiche Schalterzeile, per `diff` belegt.

### Das Ergebnis in einem Satz: die Physik ist bitgleich, über alles.

| Geprüft | Ergebnis |
|---|---|
| **21 CSV-Dateien** byteweise | **alle bitgleich** |
| **7 Feld-Dumps** byteweise, darunter 3 × 1,11 GB Nahfeld bei 300/450/500 ms | **alle bitgleich** |
| **21 Wirkpfad- und Klemmzähler** aus dem Log | **alle identisch** |
| Fehler, beide Arme | rc=0, 0 Fehler, je 180 Warnungen |
| Gitter- und Facettenzahlen | identisch |

Nicht nur die Kräfte — **das ganze Feld zu drei Zeitpunkten, Byte für Byte.** Das ist der
schärfste Nachweis, den dieser Aufbau hergibt.

### Die einzelnen Änderungen

**E1 — Scratch in `fac_nachbar_ab` beseitigt.** Der laufzeitindizierte `c(ib)`-Zugriff nach
der Suchschleife machte die 57-float-Tabelle speicherheimisch. Behoben, indem die
Linkkomponenten **und** der Nachbarindex schon in der Schleife mitgeschrieben werden, wo der
Index compilezeitkonstant ist. Damit ist auch das zweite laufzeitindizierte Feld (`j[ib]`)
weg, das im ursprünglichen Befund nicht genannt war.

| | vorher | nachher |
|---|---:|---:|
| `private_size` B70 (0xe223) | 7296 | **0** |
| `private_size` iGPU (0x7d67) | 3648 | **0** |

Gegenprobe am erzeugten Gerätecode: **alle Änderungen liegen zwischen Zeile 17944 und 18018
der .cl-Quelle, der nächste Kernel beginnt bei 18030.** Kein anderer Kernel ist berührt.

**E2 — Glättungsindex.** Die dichte Tabelle über die Facetten-BBox (`setup.cpp`, bei 4 mm
593,7 MB für 2,1 % Belegung) ist ersetzt durch eine nach Zellindex sortierte Indexliste plus
Binärsuche, 4 B je Facette statt 4 B je BBox-Zelle. **Ohne** die Annahme, dass `F` in
Scanreihenfolge vorliegt: sortiert wird nach (n, i), nachgeschlagen wird der größte Index der
Gleichheitsgruppe — genau das, was das alte `feld[...]=i` beim Überschreiben hinterließ.

**E3 — ELIBB-Remesh-Karte freigegeben.** Sie lebte bis zum Laufende, obwohl `alloc_facetten`
sie auswertet und danach niemand mehr liest. Vor dem Bau geprüft, dass kein Member den Zeiger
hält (`lbm.cpp:929` ist die einzige Verwendung). Freigabe per `swap`, nicht `clear`, weil
`clear` bei einer Hashtabelle die Bucket-Tabelle behält.

### Wanduhr und Speicher

| | `o8_vor` | `o8_nach` | `o8_nach2` | `o8_e5` |
|---|---:|---:|---:|---:|
| Wanduhr | 426 s | 419 s | 415 s | **403 s** |
| System-RAM VmHWM | **2321,2 MB** | – | **2134,2 MB** | – |

**Gesamt 426 s → 403 s = −5,4 % Wanduhr**, bei durchgehend bitgleicher Physik.

**Wanduhr −2,1 %** (Mittel der beiden Wiederholungen gegen den Bezug). Die Streuung zwischen
den beiden identischen Wiederholungen beträgt 4 s, der Gewinn 9 s — er liegt also über der
Streuung, aber nicht weit darüber. **Der Wanduhr-Gewinn ist ausdrücklich nicht die
Rechtfertigung.** Die ist, dass eine Fehlerklasse verschwindet, die diesen Fork schon einmal
einen Faktor 100 gekostet hat.

**System-RAM −187,0 MB = −8,1 %** bei 8 mm. Die unabhängige Prüfung hat zwei Zahlen dazu
geschärft: `stable_sort` braucht einen Temporärpuffer von nochmals rund 12,5 MB, der
Nettogewinn bei 4 mm ist also etwa **581 MB statt 593,7**. Und der Nachschlag ist teurer
geworden — nachgebaut gemessen **+2,5 s Aufbauzeit bei 4 mm** gegen 594 MB. Bei einem
24-Minuten-Lauf ist das ein guter Tausch, aber die Zahl gehört genannt. Bei 4 mm ist mehr zu erwarten: der dichte
Glättungsindex skaliert mit der Facetten-BBox und belegte dort 593,7 MB statt der 258 MB bei
8 mm. Gemessen wird das erst am nächsten 4-mm-Lauf.

### E5 — P-TRT-Zählergatter von `t%100` auf `t%1000`

Eigener Arm `o8_e5` gegen `o8_nach2`. Vorher geprüft, dass keine Abnahme aus dem 100er-Raster
eine Sollzahl ausrechnet: `pruefe_ptrt` vergleicht nur relativ (203 gegen 202) und auf
Ungleichnull. Der Helfer in `setup.cpp:514`, der Zählslots aus `(t_ende-1)/100` berechnet,
gehört dem SGS-Band-Test (Slot 186) und ist nicht betroffen.

| | `o8_nach2` | `o8_e5` |
|---|---:|---:|
| CSV + Feld-Dumps | — | **28 von 28 bitgleich** |
| ausgedünnte P-TRT-Stichproben | 14 017 095 | **1 451 970** (Faktor 9,65) |
| Wanduhr | 415 s | **403 s (−2,9 %)** |

**Das ist der größte Einzelgewinn dieser Runde** — größer als E1, E2 und E3 zusammen, und
die Streuung zwischen zwei identischen Läufen beträgt 4 s gegen 12 s Gewinn.

**Ausdrücklich deklariert:** die Zählerwerte 199 bis 203 fallen um Faktor 10. Ein A/B, das
diese Slots gegen eine Baseline **vor** E5 vergleicht, meldet zu Recht eine Abweichung. Das
ist gewollt und kein Fehler.

### Zwei Nebenbefunde aus der Abnahme

**Der Lauf ist deterministisch.** `o8_nach` gegen `o8_nach2`, gleiches Binary, zweimal
gefahren: **28 Dateien bitgleich, davon alle Feld-Dumps.** Erst das macht die Bitgleichheit
zwischen `o8_vor` und `o8_nach` zu einem Beweis statt zu einem Zufall.

**Die Messkette hatte selbst einen Fehler.** Die erste VmHWM-Mitschrift war unbrauchbar, weil
`pgrep -f "bin/FluidX3D"` auch die eigene Warte-Shell trifft, deren Kommandozeile denselben
Text trägt. Richtig ist `pgrep -x FluidX3D` auf den Prozessnamen. Der Bezugswert blieb
trotzdem gültig, weil die Verunreinigung erst nach Laufende einsetzte und im Verlauf klar
abgrenzbar war.

### Werkzeugarbeit im selben Zug

**Das Scratch-Gate prüfte seit dem Bau am 26.08. nur `stream_collide`** — der Kernelname stand
in `igc_offline.sh` fest verdrahtet. Genau deshalb blieb `fac_nachbar_ab` unentdeckt. Das Gate
prüft jetzt **alle 37 Kernel auf beiden Geräten**, führt bekannte Abweichungen als benannte
Schuld mit Behebungsweg, und meldet einen Eintrag als veraltet, sobald er nicht mehr zutrifft.

Beide Wächterfunktionen haben sich sofort bewährt: der Negativtest fand die iGPU-Hälfte des
Befundes, die im Bericht fehlte, und nach dem Fix meldete das Gate seine eigenen Einträge als
veraltet. **Gate-Exit 0, alle 37 Kernel sauber.**

### Noch offen aus der Liste

E4 (`CFD_FAC_KDIAG=0`) ist ein Schalter und damit eine eigene Messvariable — nicht
mitgebündelt. E5 (P-TRT-Zähler) ist bereits A/B-belegt, aber noch nicht gesetzt. Die
Mittel-Klasse M1 bis M9 ist unangetastet.

---

## 1b. Spalding-Tabelle (M1) — umgesetzt, mit einer widerlegten Behauptung

Von Heiko am 11.09. freigegeben. **Nicht bitgleich** — hier entscheidet Messbarkeit.

**Bauform:** `__constant` im Dateibereich, 512 Stützstellen à 2 kB, Emission über
`device_defines`. Bewusst **kein** Kernelparameter (Signatur-Splice ist die R()-Klammerfalle,
in diesem Fork zweimal bezahlt) und **kein** privates Array (Scratch-Falle). Vor dem Einbau
mit einer Minimalkernel-Probe geprüft: `private_size` 0. Danach offline auf beiden Geräten
übersetzt: private 0, spill 0.

**Schalter `CFD_SPALDING_TAB`, jetzt Default an.** Der Aus-Zustand ist gegen `o8_e5`
bitgleich geprüft (28 von 28 Dateien) — der neue Code ist also nachweislich inert, wenn er
aus ist. Damit war das A/B ein Binary und eine Variable.

**Offline gemessen** (200 000 Punkte log-gleich über den gemessenen Bereich Y = 1,52…2,19·10⁴,
gegen Bisektion in double), τ_w-Fehler:

| | max | p99 |
|---|---:|---:|
| Newton it=3 (Stand bis heute) | 4,364 % | 4,235 % |
| Newton it=8 | 0,0001 % | 0,0000 % |
| **Tabelle 512 / 2 kB** | **0,0035 %** | 0,0033 % |

Geprüfte Gegenvariante ohne Tabelle: ein besserer Startwert bringt Faktor 50 (4,364 → 0,086 %),
erlaubt aber **keine** Iteration weniger (it=2 wäre 4,88 %) und kostet zusätzliche
Transzendente. Die Tabelle gewinnt.

**Am Fahrzeug gemessen.** Zielmarke ist `o8_it8` — acht Newton-Schritte, praktisch
auskonvergiert, eigens dafür gefahren. Ohne diesen Arm gäbe es keinen Bezug, gegen den sich
„messbar positiv" prüfen ließe. Zeiger ist `cd_reib`, weil dort der Spalding-Fehler
systematisch wirkt; die Druckanteile sind von turbulenter Streuung beherrscht.

| Abstand zu `o8_it8`, sechs 50-ms-Fenster | Mittel | Vorzeichen |
|---|---:|---|
| Newton it=3 (`o8_e5`) | **−0,00077** | **sechsmal negativ, systematisch** |
| Tabelle (`o8_tab1`) | **−0,00024** | wechselnd |

**Der systematische Versatz ist zu 69 % verschwunden.** Das ist der Grund, warum die Tabelle
bleibt.

### ⚠ Die Tempo-Behauptung ist widerlegt

| | Wanduhr |
|---|---:|
| `o8_e5` (Newton) | 403 s |
| `o8_tab0` (Tabelle aus) | 404 s |
| `o8_tab1` (Tabelle an) | **402 s** |

**Kein Gewinn** — die Streuung beträgt 4 s. Die in Teil 1 genannten −4,57 % Instruktionen
schlagen nicht durch, weil der Facettenpfad nur **0,6 % der Zellen** betrifft. Das ist am
selben Tag zum **zweiten Mal** dieselbe Lehre: beim ELIBB-Test war der Arm mit 2073
Instruktionen weniger sogar langsamer. **Eine Instruktionszahl ist kein Laufzeitmaß**, und
alle noch offenen Prozentzahlen in Abschnitt 2 stehen unter diesem Vorbehalt.

---

## 1c. Gemeinsamer Zähltakt (M3) — umgesetzt, Gewinn belegt

Die Zähler **abzuschalten** hätte reihenweise Ist=Soll-Abnahmen gebrochen, weil der Host die
erwartete Zahl aus dem 100er-Raster ausrechnet. Stattdessen laufen Kernel-Gatter **und**
Sollformeln über **einen** Takt, der an genau einer Stelle steht (`zaehl_takt()` in
`lbm.cpp`): **71 Gatter** in `kernel.cpp`, **12 Formeln** in `setup.cpp`, darunter alle
`ceil(n/100)`-Sollwerte der Slots 7/20/21/22/76 und der SGS-Band-Wirkpfad 186.

**Die Begründung stützt sich nicht auf Instruktionszahlen**, sondern auf E5: dort brachte das
Ausdünnen **eines** Blocks 12 s. Zähler sind Atomics auf einen gemeinsamen Puffer und
serialisieren.

| Takt | Einzelläufe | Mittel |
|---|---|---:|
| 100 (Default) | 402, 406, 401 s | 403,0 s |
| **1000** | 398, 395 s | **396,5 s** |

**−6,5 s = −1,61 %, und die beiden Spannen überlappen nicht** (schlechtester 1000er-Lauf
395…398 gegen besten 100er-Lauf 401). Das ist der Grund, warum die Maßnahme bleibt.

**Physik unverändert:** Takt 100 gegen den Vorstand 28 von 28 Dateien bitgleich (die
Umstellung ist also inert), Takt 1000 gegen Takt 100 **27 von 28** — die eine Abweichung ist
`slots_verlauf.csv`, die Zählerspur selbst. Keine Abnahme schlug Falschalarm, 0 Fehler.

**Der Code-Default bleibt bei 100.** Bei 8 mm bekäme das Fernfeld mit Takt 1000 nur **eine**
Stichprobe; bei kürzeren Läufen keine, und dann meldet ein No-Op-Wächter zu Recht nichts oder
zu Unrecht einen Defekt. Gesetzt wird der Takt deshalb in der **Standardzeile**, wo die
Schrittzahl bekannt ist: bei 4 mm sind es 50 Stichproben im Nahfeld und 12 im Fernfeld.

**Werkzeugbefund:** der Gate-Bau brach, weil die Zwillingsliste in `gen_main.cpp` den neuen
Define nicht kannte. Das Gate meldete das als **Baufehler und nicht als Scratch** — diese
Unterscheidung wurde heute früh eingebaut und hat sich damit zum ersten Mal bewährt.

---

## 1d. Klassen-Diagnostik aus (E4) — der erste eingesparte Grafikspeicher

`fac_kd` ist der **einzige** Diagnostikpuffer, der in der Produktion VRAM belegt. Die drei
übrigen Diagnosepfade (`DIAGZ`, `GDIAG`, `RDIAG`) sind bereits aus, weil ihre Schalter nicht
gesetzt sind.

| | `d8_kdiag_an` | `d8_kdiag_aus` |
|---|---:|---:|
| `fac_kd` (8 mm) | 43 MB | **nicht alloziert** |
| bei 4 mm | 191,0 MiB | — |
| Wanduhr | 397 s | **392 s (−1,26 %)** |
| `forces.csv` | — | **bitgleich** |
| Nahfeld 500 ms | — | **bitgleich** |

**Es ist reine Diagnostik** — Kräfte und Feld sind byteweise identisch.

**Was bleibt:** der Klassen-Zensus und die Wirkpfadzähler stehen weiter im Log, in beiden
Armen zeichengleich (n = 266 283, Rang2 74,5 %, `ELIBB[67]` = 14 417 598). Die
Ein-Variablen-Prüfung eines A/B, wie ich sie heute beim ELIBB-Test gefahren habe, ist also
**nicht** verloren.

**Was entfällt:** `facetten_klassen.csv` (81 kB), `facetten_persistenz.csv` (37,6 MB) und
`yplus_facetten_angewandt.csv` (4,6 MB). Wer sie braucht, setzt `CFD_FAC_KDIAG=1` — das ist
dann ein Prüflauf, kein Produktionslauf.

---

## 1e. Die 4-mm-Bestätigung (`p4_neu`, 11.09.2026)

Alles bis hier war am 8-mm-Fahrzeug gemessen. Dieser Lauf prüft, was bei 4 mm zusammen
ankommt. **Kein A/B** — drei Schalter und neuer Code auf einmal; die Einzelwirkungen sind
alle bei 8 mm gepaart belegt, jede mit eigenem Arm.

| | Baseline `p4dt_deteps` | `p4_neu` |
|---|---:|---:|
| Wanduhr | 94,5 min | **90,4 min (−4,36 %)** |
| VRAM an vergleichbarer Stelle | 28 003 MB | **27 695 MB (−308 MB)** |
| `fac_kd` | 190 MB | **nicht alloziert** |
| System-RAM VmHWM | nicht gemessen | 14 842 MB |

Der Gewinn ist kleiner als die 8 % bei 8 mm. Das ist plausibel: bei 4 mm wiegt der reine
DDF-Strom schwerer, und an dem ändert keine dieser Maßnahmen etwas.

**Die Kräfte sind unverändert**, obwohl das SGS-Band aus ist und die Spalding-Inversion
ersetzt wurde:

| | Baseline | `p4_neu` | Δ |
|---|---:|---:|---:|
| Cd_rest | 0,5387 ± 0,0122 | 0,5372 ± 0,0118 | −0,0015 |
| Cz_rest | −1,0306 ± 0,0218 | −1,0224 ± 0,0221 | +0,0082 |
| cd_reib | 0,0346 ± 0,0006 | 0,0347 ± 0,0005 | **+0,34 %** |

Cd_rest und Cz_rest liegen klar innerhalb der Fehlerbalken. Die Reibung steigt um 0,34 %,
**in derselben Richtung wie bei 8 mm gemessen** — das ist die Spalding-Tabelle, die den
systematischen Versatz der drei Newton-Schritte entfernt.

**Der neue ehrliche Spitzenwert steht zum ersten Mal im Log:** 27 734 MB nach gebundener
Kopplung und Schale, 4 921 MB rechnerisch frei, mit dem ausdrücklichen Hinweis auf
`kf_liste`, die erst in der Zeitschleife bindet. An derselben Stelle stand vorher eine Zahl,
die 300 MB zu optimistisch war.

**Das Restrisiko der Wächterarbeit ist erledigt:** der neue Speicherplan bucht 541 MB mehr
(27 452 → 27 993 MB) und lässt das Gitter trotzdem durch.

---

## 1f. Block-Tiling — zum ersten Mal in v2 gemessen (11.09.2026 abends)

Bis heute stützte sich jede Aussage dazu auf **V1-Zahlen**. Fünf Arme am 8-mm-Fahrzeug,
null Codezeilen, Bezug `bt8_aus` mit 390 s.

### Die Durchsatzkurve — und sie läuft in eine Wand

| Arm | Wanduhr | Durchsatz | zusammenhängend |
|---|---:|---:|---|
| ohne Tiling | 390 s | 100 % | dicht |
| T=8 | 549 s | **71 %** | 16 B = ¼ Cache-Zeile |
| T=16 | 497 s | 78 % | 32 B = ½ Zeile |
| T=32 | 490 s | **80 %** | 64 B = eine volle Zeile |
| T=64 | 497 s | 78 % | 128 B = zwei Zeilen |

**Der Durchsatz sättigt bei 80 % und kommt nicht zurück.** Eine volle Cache-Zeile bringt
gegenüber einer halben nur zwei Punkte, zwei volle Zeilen bringen nichts mehr.

**Damit ist die DDF-Zersplitterung als alleinige Ursache widerlegt** — sie erklärt die ersten
neun Punkte (71 → 80), nicht die restlichen zwanzig. Die bleiben beim
`tile_slot`-Zugriff selbst: eine **abhängige Ladung vor jeder Adressrechnung**, die keine
Kachelform wegformen kann. Das ist genau, was der Quelltextkommentar (`kernel.cpp:958`) seit
jeher behauptet und was ich heute Mittag noch für die halbe Wahrheit gehalten hatte.

### Bitneutralität — zum ersten Mal in v2 belegt

`bt8_t8` und `bt8_t16` gegen `bt8_aus`: **je 25 von 25 Dateien bitgleich**, Feld und Kräfte.
Bisher stützte sich das auf eine Kugelmessung in V1. Der Papierkorb-Slot
(`lbm.cpp:1071-1079`), an dem die ersten Versuche mit Cd 18,4 divergierten, hält.

### Die Kachelform: anisotrop schlägt den Würfel auf BEIDEN Achsen

Ausgezählt am Flag-Export des 4-mm-Laufs (`p4_neu`, 519 139 485 Zellen), Halo 2 wie der Code
ihn verlangt, Aufrundungspolster und Kacheltabelle eingerechnet:

| Kachel | netto frei | zusammenhängend |
|---|---:|---|
| 8×8×8 (heutiger Stand) | 1 284,3 MiB | 8 Zellen |
| **32×8×2** | **1 454,8 MiB** | 32 Zellen |
| **16×8×4** | **1 447,2 MiB** | 32 Zellen |
| 32×4×4 | 1 376,7 MiB | 32 Zellen |
| 16×16×16 | 450,8 MiB | 16 Zellen |

**Welche Achse grob sein darf, ist gemessen** (Kachelvolumen konstant 512 Zellen):

| | netto frei |
|---|---:|
| 32×4×4, grob in **x** | **1 376,7 MiB** |
| 4×32×4, grob in y | 1 124,6 MiB |
| 4×4×32, grob in z | 730,0 MiB |

Grob in x ist also nicht nur die einzige Achse, die der lineare Dispatch zulässt, sondern
auch die **beste**: das Fahrzeug ist in x lang und durchgehend, in y und z dünn und
zerklüftet. Speicherordnung und Geometrie ziehen in dieselbe Richtung.

Empfohlenes Bauziel ist **16×8×4**, nicht das Randoptimum 32×8×2: die 2 in z bedeutet einen
Halo, der doppelt so dick ist wie die Kachel, und macht das Ergebnis empfindlich gegen
Geometrieänderungen. Der Unterschied beträgt 7 MiB.

### Der Anwendungsfall: reicht es für 3,75 mm?

Zellzahl ×1,214 gegenüber 4 mm, Kapazität 32 655 MB:

| | Bedarf | |
|---|---:|---|
| dicht | 33 658 MB | **passt nicht**, 1 003 MB zu viel |
| T=16 | 33 111 MB | **passt nicht**, 456 MB zu viel |
| T=8 | 32 100 MB | passt, 554 MB Restluft — **unter der Mindestluft von 1 024 MB** |
| **16×8×4** (Bau nötig) | **31 902 MB** | passt, 752 MB Restluft, bei ~78 % Durchsatz |

**T=16 löst den 3,75-mm-Fall nicht.** Nur T=8 kommt heute in Frage, und dessen Reserve liegt
unter der Projektvorgabe. Die anisotrope Kachel ist der einzige Weg, der 3,75 mm mit
vertretbarer Reserve **und** dem besseren Durchsatz erreicht.

### Verdikt

**Block-Tiling bleibt ein Regler, der Speicher gegen Zeit tauscht — dauerhaft.** Rund 20 %
Wanduhr sind der Boden, und den senkt weder die Kachelform noch, nach dieser Kurve, V1s
Workgroup=Tile-Dispatch mit seinen behaupteten −12 %. **Nicht bauen, solange 4 921 MB frei
sind.** Es ist die Reserve für den Tag, an dem eine Rechnung sonst gar nicht passt — und für
den liegt jetzt belegt vor, was sie kostet und welche Kachelform die richtige ist.

---

## 2. Massnahmenliste — Stand nach dem Audittag

### Umgesetzt und belegt

| Nr | Massnahme | Gewinn (8 mm gepaart gemessen) | Physik |
|---|---|---|---|
| E1 | `c(ib)` durch Mitschrift in der Schleife ersetzen | `private_size` 7296 → 0 (B70) und 3648 → 0 (iGPU) | bitgleich |
| E2 | Glättungsindex → sortierte Liste + Binärsuche | −581 MB System-RAM, +2,5 s Aufbau | bitgleich |
| E3 | `elibb_qmap_dd` nach Gebrauch freigeben | −174 MB System-RAM | bitgleich |
| E1–E3 zusammen | | **−11 s Wanduhr, −187 MB RAM** | 28/28 bitgleich |
| E5 | P-TRT-Zählergatter `t%100` → `t%1000` | **−12 s = −2,9 %** | 28/28 bitgleich |
| M1 | **Spalding-Tabelle** (512 Stützstellen, `__constant`) | Tempo **±0**; systematischer Reibungsversatz **−69 %** | ändert Zahlen, belegt besser |
| M3 | **Gemeinsamer Zähltakt** (71 Gatter + 12 Sollformeln) | **−6,5 s = −1,61 %**, Spannen getrennt | 27/28, Abweichler ist die Zählerspur |
| E4 | `CFD_FAC_KDIAG=0` | **−191 MiB VRAM**, −5 s = −1,26 % | Kräfte und Feld bitgleich |
| — | **Host-Spiegel freigeben** (5 Puffer, bei 4 mm 292 MB) | **UNBELEGT** — mit VmHWM falsch gemessen | 25/25 bitgleich |

**Bei 4 mm zusammen gemessen** (`p4_neu` gegen `p4dt_deteps`): **94,5 → 90,4 min (−4,36 %)**,
VRAM 28 003 → 27 695 MB, Cd_rest und Cz_rest innerhalb der Fehlerbalken.

### Widerlegt

| | Warum |
|---|---|
| **M4 Volumenkraft nicht emittieren** | Sie ist **nicht** tot: `kernel.cpp:2993` speist das Wandmodell-Residuum als Volumenkraft ein. Entfernen hätte `CFD_FAC_KRAFT` lautlos wirkungslos gemacht. |
| **Gemischte Kachelgrössen** | Am fi-Puffer **beweisbar null** (`Σ Kinder ≤ T³`). 64³ und 128³ **kosten** 3,8 bzw. 6,7 GiB Polster. |
| **Tempoversprechen der Spalding-Tabelle** | −4,57 % Instruktionen, null Wanduhr. Der Facettenpfad betrifft 0,6 % der Zellen. |
| **„1,43 GB bei −12 %"** und **„5 464 MLUPS"** | V1-Zahlen, in v2 nie gemessen. |
| **A1 „G ist reine Geometrie"** | Die Tangentialbasis kommt aus `calculate_rho_u` und dreht sich jeden Schritt. |

### Offen, mit Begründung die NICHT an Instruktionen hängt

| Nr | Massnahme | Was fehlt |
|---|---|---|
| M5 | `sgs_fdwand` und `fac_nachbar_ab` verschmelzen | Kostensonde steckt fest: `CFD_SGS_FDWAND=0` wird abgewiesen, weil SISM ihn verlangt |
| M6 | ABSTAND-Scan in `boden_eq` durch ein Flagbit | ungemessen; 347 M Reads je Grobschritt für 0,50 % Treffer |
| M9 | Sechs von acht Round-Trips bündeln | ungemessen; Obergrenze 3,1 % |
| M2 | Geometrie in die freien `fac_geo`-Slots | **bewusst nicht gebaut**: erst die Kostenzahl von `fac_nachbar_ab` |
| — | Host-Spiegel: Arbeitssatz statt VmHWM messen, bei 4 mm | die Messung, nicht der Bau |

### Offen, aber nur durch Instruktionszahlen begründet — nicht bauen

M7 (`cubic_lift_weights`-Tabelle) und M8 (3×3-Tensor, „Variante B"). **Zweimal an einem Tag
ist eine Instruktionszahl folgenlos geblieben**, einmal sogar mit umgekehrtem Vorzeichen.

### Getrennt zu untersuchen

**Block-Tiling.** Der einzige Hebel über 1 GiB VRAM (T=8: netto 1 283,9 MiB) und
nachweislich bitneutral. Preis am heutigen v2-Stand −40 % Durchsatz; V1s Workgroup=Tile
senkt ihn auf −12 %, ist aber nicht portiert und **spillt** in v2 bei 1:1-Übernahme
(1152 B B70, 576 B iGPU). Zwei v2-eigene Fallen: Slot 0 ist ein Papierkorb, und
`active_tile_id` muss hinter `TS_P` gebunden werden. Details in 3.4 und 3.5 der Rohbefunde.

**Nahkasten beschneiden.** 60,9 MiB je z-Schicht, aber `AUDIT-BEFUNDE.md:5343` führt den
Kasten bereits als zu knapp. Physikentscheidung, keine Optimierung.

---

## 3. Wo Zeit und Speicher hingehen

### Verkehr je feinem Schritt (Nahfeld, 519 139 485 Zellen)

| Posten | GB | % |
|---|---:|---:|
| DDF lesen + schreiben (2×38 B × 455,8 M Fluidzellen) | 34,64 | 79,9 |
| rho + u schreiben | 7,29 | 16,8 |
| flags | 0,52 | 1,2 |
| Facettenpuffer gesamt | 0,77 | 1,8 |
| Bitmasken + Präfixsummen | 0,12 | 0,3 |
| **Summe** | **43,36** | **83,5 B je Gitterzelle** |

Je Zellklasse: Solid 1 B (früher Ausstieg), Freistrom-Fluid 93 B, Bandzelle 97,75 B,
**Facettenzelle 339,5 B = 3,65× Freistrom** bei 0,60 % der Zellen.

### Koaleszenz ist in Ordnung

Der DDF-Pfad ist in **jedem** Arm exakt 19 `load.ugm.d16u32` + 19 `store` — voll koaleszierte
32-B-Nachrichten, keine einzige Zusatzberührung durch die Facettenkette. Die Zusatzpuffer
machen die Koaleszenz nicht kaputt; ihr Preis ist Latenz und Instruktionen, nicht Bandbreite.

### Speicher

| | MiB |
|---|---:|
| fi + rho + u + flags + f_maske + F | 27 310,9 |
| Facettenkette | 581,3 |
| SGS-Band (im Standard aus) | 118,8 |
| späte Puffer (coupling, slice_flags, schale, **kf_liste 237,3**, kf_psum/pcnt) | 300,6 |
| **echter Spitzenwert** | **28 311,7** (gedruckt: 28 003) |

Zellklassen gemessen: **12,197 % echt solid**, 87,591 % aktiv, 0,634 % TYPE_E-Kopplungsrand.

**Drei Speicherwächter-Defekte:**
1. Der gedruckte Spitzenwert fällt 300,6 MiB zu früh. `setup.cpp:7055` behauptet, hier sei der
   Aufbau vollständig, aber `alloc_coupling_planes` (7103) und `alloc_schale` (7127) folgen,
   und `kf_liste` bindet erst in der Zeitschleife beim ersten Kräfte-Sample. **Der wahre
   Spitzenwert fällt nach jedem Wächter** — ein Lauf kann den Aufbau überleben und 260 MiB
   später sterben.
2. `bytes_bekannt` (`lbm.cpp:2136-2150`) kennt Facettenkette und SGS-Band nicht: **596,3 MiB
   ungedeckt** netto.
3. `alloc_sgs_band` (`lbm.cpp:660-719`) prüft **keinen freien Speicher**, während
   `alloc_facetten_domain` (`:836-841`) es tut. 118,8 MiB fallen ungeprüft.

---

## 4. Korrekturen — was falsch dokumentiert war

**Alle vier sind selbst nachgeprüft und im Repo behoben.**

| Was | War | Ist |
|---|---|---|
| **MLUPs/GB-s-Anzeige** | 1946 MLUPs, 239 GB/s | **5028 MLUPs.** Die Fortschrittszeile teilt die **grobe** Zellzahl durch die **feine** Schrittzeit, Faktor 2,551 |
| **„1,43 GB bei −12 %"** | als v2-Zahl geführt | **V1-Zahl.** `SPARSE_TILES_WG` existiert in v2 nicht; hier kostet Tiling **−40 %** |
| **„≈ 5 464 MLUPS"** | als v2-Baseline geführt | **V1**, Einzeldomäne, 337,5 M Zellen, ohne Wandmodell (`FluidX3D/MODIFICATIONS.md:251`) |
| **A1: „G ist reine Geometrie"** | eigene These | **falsch.** `t1 = ut/\|ut\|` (`kernel.cpp:2073`) kommt aus `calculate_rho_u` (`:2002`) — die Basis dreht sich jeden Schritt. Konstant sind nur der 3×3-Tensor und die Invarianten Snn, det, tr |
| **Bandbreitenanzeige** | „überzeichnet um 10 %" | **47 %.** Grösster Posten: 18 B Nachbar-Flags für 100 % der Zellen, bezahlt von 0,21 % |

**Zur Anzeige-Konvention:** `Info::print_update` nimmt `lbm->get_N()` (`info.cpp:119`);
`info.lbm` zeigt aufs Fernfeld, weil `lbm_c.run(0u)` als letztes initialisiert
(`setup.cpp:7045`), während `info.update` nur in `LBM::run` steht (`lbm.cpp:2447`) und die
Zeitschleife `run()` ausschliesslich fürs Nahfeld ruft. **Verhältnisse zwischen zwei Läufen
bleiben gültig, Absolutwerte nicht.**

Damit dreht sich das Gesamtbild: der v2-Nahkernel liegt **8 % unter** der nackten
V1-Baseline, mit der kompletten Facettenkette, SISM, P-TRT und DETEPS obendrauf. Der gesuchte
„Faktor 2,8" war ein Artefakt.

---

## 5. Sparse Tiles — Verdikt

**Obergrenze jeder Halo-2-Kachelung: 2 035,4 MiB**, nicht 2 294,6 — der 2-Zell-Halo kostet
259,2 MiB, bevor ein Korn gewählt ist.

| T | fi frei netto | % der Obergrenze |
|---:|---:|---:|
| 4 (**in v2 gesperrt**, `setup.cpp:4519/5066/5683`) | 1 630,0 MiB | 80,1 % |
| **8** | **1 283,9 MiB** | 63,1 % |
| 16 | 449,6 MiB | 22,1 % |
| 32 | 106,7 MiB | 5,2 % |
| **64** | **−3 074,6 MiB** | Polster > Ersparnis |
| **128** | **−6 722,6 MiB** | 335 von 336 Kacheln aktiv |

**Gemischte Grössen sparen beweisbar null Byte am fi-Puffer.** Acht Kinder der Kante T/2
überdecken genau T³ Zellen, tote Kinder kosten 0, also ist `Σ Kinder ≤ T³` immer. Eine
Oktree-Optimierung über das ganze Gitter landet exakt auf dem flachen Optimum. Ihr einziger
Gewinn ist die Indextabelle: bei erwägbaren Körnern **0,8 bis 27 MiB**, erkauft mit **+7,5 %
Instruktionen** und Unverträglichkeit mit Workgroup=Tile.

**Die Fahrbahnplatte ist ein Phantom:** zwei Zellen dick, der Halo frisst sie, **99,97 % des
einsparbaren Bestands liegen im Fahrzeuginneren**.

**Die Ursache der Durchsatzstrafe** ist nicht der `tile_slot`-Gather allein, sondern die
DDF-Kontiguität: beim flachen Dispatch decken 64 Threads bei T=8 **acht verschiedene Tiles**
ab, also 8 × 16 B statt 128 B am Stück. Das erklärt V1s Messreihe ohne Zusatzannahme.

**V1s Workgroup=Tile ist nicht 1:1 portierbar** — sie spillt in v2 (1152 B auf der B70,
576 B auf der iGPU), weil das geteilte `cbj[]` genau das Gegenteil der Rang-1-Remat ist. Eine
Remat-Fassung ist spillfrei und kostet +3,3 % gegenüber dem naiven Tiling. Zwei v2-eigene
Fallen: **Slot 0 ist ein Papierkorb** (`lbm.cpp:1071-1079`; eine 1:1-Kopie schreibt still
falsche Physik), und `active_tile_id` muss hinter `TS_P` gebunden werden, sonst bindet
`fac_geo` als `tile_slot`.

**Billigster Weg zu einem belastbaren Ja/Nein:** Nahfeld allein als Einzeldomäne, 25-s-Paar
`CFD_SPARSE_TILES=0/1` bei T=8 und T=16, Wanduhr je Schritt. Null Codezeilen.

---

## 6. Was nicht gemessen werden konnte

| offen | Rezept |
|---|---|
| **Laufzeit irgendeiner Massnahme** ausser E5 | 8-mm-Paar über die Queue, Wanduhr je Grobschritt |
| Durchsatzkosten von SPARSE in v2 | 25-s-Paar, Nahfeld einzeln |
| Trefferquote der u-/flags-Gather | A/B mit `CFD_FAC_NACHBAR=0` bzw. `CFD_SGS_FDWAND=0` |
| Dispatchkosten von `boden_eq` (519,1 M Work-Items für 1,15 M Treffer) | offen seit Runde 1 |
| Divergenzkosten der Facettenkette (0,6 % der Zellen, 4571 Instr. je Subgroup) | Laufzeit-A/B oder fdinfo-Profiler |
| Ist die iGPU bandbreiten- oder rechengebunden? | 25-s-Paar auf Gerät 2 mit `UPDATE_FIELDS` an/aus (−17 % Verkehr) |
| Freier VRAM real | Debugfs-Rechte fehlen; alle Frei-Werte sind die 20/19-Rekonstruktion |
| L2-Grösse der B70 | nirgends im Repo belegt |

---

# Was als Nächstes zu tun ist

Vollständige Herleitung jedes Punktes in `PERFORMANCE-ROHBEFUNDE-2026-09-11.md`, Teil 4.
**Alle Zahlen sind Wanduhr, Verkehr oder eingesparte Schritte — keine Instruktionszahl.**

## Die drei großen

| # | Hebel | Gewinn | Physik | Aufwand |
|---|---|---|---|---|
| 1 | **`u_lat` von 0,075 anheben** | Laufzeit **∝ 1/u_lat**: 0,10 → **−25 %**, 0,125 → −40 % | Ma 0,130 → 0,173, Kompressibilitätsfehler ×1,8 | **eine Konstante an vier Stellen** |
| 2 | **rho/u nur schreiben, wo gelesen wird** | **−6,5 bis −9,5 % Wanduhr** | bitgleich beweisbar | hoch |
| 3 | **Prüfpunkt/Neustart** | **17–34 min je Folgelauf** (Anwärmphase ist 39 %) | — | 200–300 Zeilen |

**Zu 1, und das ist der Fund dieser Runde:** `u_lat = 0.075f` steht **hart verdrahtet an vier
Stellen** in `setup.cpp`, ohne Schalter und ohne dokumentierte Herleitung. Der Zeitschritt ist
`dt = u_lat·dx/u_si`, die Laufzeit also streng proportional zu `1/u_lat`. Bei 0,075 rückt das
Feld erst nach **13,3 Schritten** eine Zelle weiter — jedes Teilmodell mit advehiertem Eingang
ist um diesen Faktor überabgetastet. Der Preis ist die Mach-Zahl (heute 0,130), der
Kompressibilitätsfehler skaliert mit Ma². Nebenbei steigt τ von 0,500028 auf 0,500038, also
**weg** von der Instabilitätsgrenze. **Nie gemessen, ein A/B auf der 8-mm-Sprosse kostet 14 min.**

**Zu 2:** Im ganzen `stream_collide` gibt es **sieben** Zugriffe auf `u[]`/`rho[]`, und die drei
lesenden stehen im TYPE_E-Zweig — **0,63 % der Zellen**. Geschrieben wird für 87,8 %, jeden
Schritt: 16,8 % des Verkehrs für eine Leserschaft unter zwei. Alternative mit derselben
Ursache: beide auf 2 Byte → **−3 961 MiB VRAM**, −7,2 % (Format ist am echten Feld
entschieden: `FP16S(rho−1)`, **nicht** int16 — das kippt Gates am Wandmodell).

## Die billigen

| Hebel | Gewinn | Aufwand |
|---|---|---|
| **`CFD_T_WARMUP` 0,201 → 0,29** | **15,6 min UND +1,3 % Genauigkeit** | eine Variable |
| Remesh-Diagnostik gattern | 1,35 % (73,3 s für zwei rein berichtende Rechnungen) | zwei `if` |
| Kopplungsernte alle 2 Grobschritte | 1,65 % | zwei Zeilen |
| `extract_plane_macros` vor `lbm_c.finish()` ziehen | 0,5–1,0 % + vier Syncs weniger | klein |
| Kräftekadenz 1 → 4 ms | 0,56 % (gemessen **41-fach** überabgetastet) | eine Variable |
| 150-ms-VTK-Dump, der wieder gelöscht wird | 9 s + 11,7 GB Schreiblast | eine Zeile |
| `fac_geo[6]/[7]` (nie gelesen) | 25 MB VRAM | Stride-Umbau |

## Der Befund, der kein Performance-Befund ist

**Das Messfenster beginnt mitten im Einschwingen.** SISM wird bei 150 ms scharf, die Mittelung
beginnt bei 201 ms, der Vorgang braucht 92–114 ms. Bias **+1,34 % auf `cd_druck`**. Für
gepaarte A/B harmlos, für **jede Absolutaussage gegen OF13 nicht** — und das ist die offene
Hauptfrage des Projekts. Die Nahfeldbox wird vor Messbeginn nicht einmal **einmal**
durchspült (0,225 s gegen T_WARMUP 0,201 s).

## Erledigt — nicht noch einmal vorschlagen

| | warum |
|---|---|
| ratio 4 → 8 | Das **Nahfeld wächst** um 0,60 %, weil `CFD_NEAR_LY` durch 32 mm nicht aufgeht |
| Dritte Auflösungsstufe | Kein Ort dafür: in den Schlupf passen 9,4 M Zellen = Würfel von 1,7 m |
| CPU als Rechengerät | 1/54 der B70; sie steht aber zu **95,4 %** in Barrieren — der Hebel ist **asynchrone Ausgabe**, nicht Rechnen |
| Nahkasten beschneiden | `AUDIT-BEFUNDE.md` B71 fordert die **doppelten** Abstände; die geforderte Box bräuchte 52,3 GiB |
| Fernfeld-Fußabdruck überspringen | 14,9 ms, liegt **komplett im Schlupf** = null Wanduhr |
| Block-Tiling | Durchsatz sättigt bei 80 %; Reserve für den Fall, dass ein Gitter sonst nicht passt |
| M2, M7, M8 | nur instruktionsbegründet — zweimal folgenlos geblieben |

## Der Deckel, und er ist beweglich

Der Schlupf des Fernfelds beträgt **8,13 %** (34,1 ms von 419,1 ms) und **schrumpft mit jeder
Nahfeld-Maßnahme** — die Runde vom 11.09. hat ein Drittel davon verbraucht. `T_fern = 1,82 ns
× N_fern`, flach über Faktor 15,5 (12-Punkte-Leiter). Die iGPU kann nie mehr als **9,59 %**
der Gesamtleistung tragen und trägt 8,93 %: **der Schnitt liegt bei 93,1 % des Optimums**.
Wer gegen den Deckel optimiert, muss ihn nach jeder Maßnahme neu bestimmen.
