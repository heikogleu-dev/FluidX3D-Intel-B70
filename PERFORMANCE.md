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
| Nahfeldanteil am Grobschritt | **95,8 %** (416,35 ms von 434,6 ms) | Das Nahfeld ist der kritische Pfad |
| Schlupf des Fernfelds | 51,26 ms (369 ms Schritt in 420 ms Fenster) | gehört der **iGPU**, nicht dem Nahfeld |
| **Maximaler Wanduhr-Gewinn** | **11,79 %** | darüber hinaus bindet die iGPU |
| Anteil des DDF-Stroms am Verkehr | **79,9 %** von 43,36 GB je feinem Schritt | bei D3Q19/FP16S nicht reduzierbar |

**Verlangsamung des Nahfelds um y kostet `416,35 ms · y` Wanduhr, ab dem ersten Prozent,
ohne Freibetrag.** Beschleunigung bringt `416,35 ms · x`, gültig nur bis x = 12,31 %.

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

## 2. Massnahmenliste

### Einfach: Schalter oder wenige Zeilen, bitgleich

| Nr | Massnahme | Ort | Gewinn | Stand |
|---|---|---|---|---|
| E1 | `c(ib)` durch Arithmetik ersetzen | `kernel.cpp:4788` | `private_size` 7296 → 0, instCount 791 → 484 (**−39 %**) | **Scratch-Falle, dringend** |
| E2 | Glättungsindex → `std::lower_bound` | `setup.cpp:3005` | **−593,7 MiB** System-RAM | `F` ist scansortiert, Wächter nötig |
| E3 | `elibb_qmap_dd` nach Gebrauch freigeben | `setup.cpp:6156/6237` | **−174 MB** System-RAM | eine Zeile |
| E4 | ~~`CFD_FAC_KDIAG=0`~~ **UMGESETZT** | Schalter | **−191 MiB VRAM, −1,26 % Wanduhr, belegt** | drei Facetten-CSVs entfallen |
| E5 | P-TRT-Zähler auf `t%1000` | `kernel.cpp` | **−18,9 s Wanduhr (0,33 %)** | **einzige A/B-belegte Laufzeitzahl** |

E2 und E3 betreffen System-RAM. **Der ist bei 20,6 von 91 GiB nicht die Bindung** — das ist
Hygiene, keine Kapazität.

### Mittel: Umbau, Bitgleichheit zu prüfen

| Nr | Massnahme | Gewinn |
|---|---|---|
| M1 | ~~Spalding-Tabelle~~ **UMGESETZT, siehe 1b** | Genauigkeit ja, Tempo **nein** |
| M2 | Geometrie in die **freien `fac_geo`-Slots** (Wandlinkmaske, `nb`, `ywb`) | `fac_nachbar_ab` 791 → 75 (**−90,5 %**), `stream_collide` −1,8 %, **null zusätzliches VRAM** |
| M3 | ~~Emissionsgate~~ **UMGESETZT als gemeinsamer Zähltakt, siehe 1c** | **−1,61 % Wanduhr, belegt** |
| M4 | ~~Volumenkraft nicht emittieren~~ **WIDERLEGT, siehe unten** | – |
| M5 | `sgs_fdwand` und `fac_nachbar_ab` zu einem Launch verschmelzen | 1463 → 707 Instr., 1–4 % Verkehr |
| M6 | ABSTAND-Scan in `boden_eq` durch ein Flagbit ersetzen | 1267 → 1170 (**−7,7 %**), 347 M Reads je Grobschritt für 0,50 % Treffer |
| M7 | `cubic_lift_weights` als 1-D-Tabelle (54 kB) | 962 → 683 (**−29,0 %**) |
| M8 | 3×3-Tensor mit konstanten Inkrementen, danach einmal projizieren („Variante B") | −199 Instr. = **−2,76 %**, null Speicher |
| M9 | Sechs von acht Round-Trips bündeln | Obergrenze **3,1 % Wanduhr** |

> **⚠ M4 IST WIDERLEGT (11.09., beim Umsetzen gefunden).** Die Volumenkraft ist im
> Fahrzeugfall **nicht** tot. `kernel.cpp:2993` speist unter `FACETTEN_KRAFT` das
> **Wandmodell-Residuum als Volumenkraft** in dieselbe Guo-Kette ein
> (`fxn += fac_kraft.x`), und `kernel.cpp:3005` addiert unter `FORCE_FIELD` ohne
> `F_NUR_SOLID` das Kraftfeld dazu. Der Befund aus Teil 1 stützte sich auf die Logzeile
> „Volume Force 0.00000000" — die meint die **konstante** Kraft `fx/fy/fz`, nicht die lokale
> `fxn`. Dass die Kette heute still ist, liegt allein daran, dass `CFD_FAC_KRAFT` per Default
> 0 ist (`lbm.cpp:343`) und `F_NUR_SOLID` per Default an. **Ein Entfernen von `VOLUME_FORCE`
> würde `CFD_FAC_KRAFT` lautlos wirkungslos machen** — genau die Fehlerklasse, die im
> Vorgängerfork den Moving-Floor-Fix jahrelang zum No-Op machte.
>
> Richtig wäre stattdessen eine **Emissionsentscheidung auf dem Host**: `VOLUME_FORCE` nur
> definieren, wenn der Fall überhaupt eine Kraft tragen kann (konstante Kraft ungleich null
> ODER `s_fac_kraft>0` ODER `FORCE_FIELD` ohne `F_NUR_SOLID` ODER Temperatur/Partikel). Das
> ist machbar, aber kein Einzeiler und braucht einen eigenen Wächter. **Bis dahin: nicht
> anfassen.**

M3 und M4 kombiniert gemessen: Nahkernel 6590 → **6129 (−7,0 %)**, Fernkernel 2219 → 1714
(−22,8 %). **Die Fernfeld-Ersparnis schlägt nicht auf die Wanduhr durch** — das Fernfeld liegt
vollständig hinter dem Nahfeld.

**Zu M3 ausdrücklich:** die Zähler sind teuer UND sie sind der Nullbeweis. Vorschlag ist ein
Emissionsschalter mit Default **an**, nicht Löschen. Ein Produktionslauf ohne Zähler nur, wenn
zu derselben Konfiguration ein Zählerlauf vorliegt.

### Nicht empfohlen

| Massnahme | Warum nicht |
|---|---|
| **Sparse Tiles** (Block-Tiling von `fi`) | 1288 MiB bei T=8, kostet aber **8,6 bis 11,5 % Wanduhr** — fast das gesamte Budget. Nur wenn eine Rechnung sonst gar nicht in den Speicher passt. Abschnitt 5. |
| **Gemischte Kachelgrössen** 8³/16³/32³/64³/128³ | Am fi-Puffer **beweisbar null** Ersparnis. Abschnitt 5. |
| **Nahkasten beschneiden** | 60,9 MiB je z-Schicht, aber verlustfrei sind **0 MiB** zu holen, und `AUDIT-BEFUNDE.md:5343` führt den Kasten bereits als **zu knapp**. |
| `rho[n]` nicht jeden Schritt schreiben | 1,82 GB = 4,2 % — aber **nicht bitgleich** und verändert den Druck-Auslass. |
| Drei 38,2-MiB-Bitmasken zusammenlegen | < 50 MiB für drei getrennte Präfixsummen. |
| **Andere Schnittstelle** (Vulkan, Level Zero, SYCL) | Die Bindung ist die DRAM-Bandbreite, die keine Schnittstelle ändert. Einziger Angriffspunkt wäre der Host-Takt, und dessen Obergrenze sind dieselben 3,1 % aus M9, erreichbar in OpenCL. |

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
