# Kasten-Regelwerk für `fahrzeug_dd`

**Festgelegt von Heiko am 21.09.2026.** Führende Quelle für alle Domänengrößen. Implementiert in
`src/setup.cpp` (Konstanten `RK_NAH_*` / `RK_FERN_*`, Helfer `spanne_regel`, Abnahme „KASTEN-REGELWERK"
im Lauflog). Die Boxen sind **keine Schalter mehr** — sie werden aus den Fahrzeugmaßen abgeleitet und
wandern bei jeder Maß- oder Auflösungsänderung mit.

## Die Regel

Fahrzeugausrichtung: Radaufstandsflächen auf z = 0, Nase bei x = 0, Mittelebene y = 0, Anströmung x− → x+.

| Abstand | Nahfeld | Fernfeld |
|---|---|---|
| X− (vor der Nase) | 0,100 L | 0,625 L |
| X+ (hinter dem Heck) | 0,625 L | 1,250 L |
| Y (je Seite) | 0,250 B | **2,250 B** |
| Z− | 0 (Fahrzeug steht auf der Fahrbahn) | 0 |
| Z+ (über dem Dach) | 0,625 H | **7,000 H** |

**Rundung (Heiko 21.09.2026):** Das Regelwerk orientiert sich **immer an den Sollabständen** und rundet
dann auf die **nächste Grobzelle** auf — auch das Nahfeld, das deshalb mit dx_c gerastert wird und nicht
mit dx_f. Grund: die Nahfeldecke muss ohnehin auf einem groben Gitterpunkt liegen (Deckungspunkt-
Konvention), und nur so ist die Box über alle Auflösungen dieselbe Geometrie. Eine Box darf den
Sollabstand überschreiten, nie unterschreiten — sonst misst der Lauf eine andere Box als die notierte.
Die Abnahme im Log bricht bei Unterschreitung ab.

Die Einlassebene des Nahfelds wird dabei **abgerundet** (also weiter stromauf), und die Boxlänge wird
erst aus dieser gerundeten Ebene bestimmt — sonst frisst die Rundung des Einlasses den Abstand X+ auf.

**Teilbarkeit (Heiko 21.09.2026):** Nx durch 16, Ny/Nz durch 4 — auf der **allozierten Gitterbreite
(Knotenzahl)**, denn daran hängt der gemessene 5-%-Effekt (TODO.md, auf der iGPU beziffert).

* **Fernfeld:** erfüllbar, wird erzwungen.
* **Nahfeld:** strukturell **unmöglich**. `fNx = (cex−1)·ratio+1` (setup.cpp) ist immer 4k+1, also
  ungerade. Vier der fünf Kopplungsebenen werden getrieben (`drive_face = {true,false,true,true,true}`),
  und `drive_boundary_from_coarse` (lbm.cpp) verlangt feine Ebenenausdehnung exakt `(grob−1)·ratio+1`.
  Dabei tritt **jede** Feindimension als Ebenenausdehnung auf: fNy/fNz in x−, fNx/fNz in y−/y+,
  fNx/fNy in z+. Das Nahfeld rundet deshalb nur auf ganze Grobzellen auf.
  **Entscheid Heiko 21.09.2026:** Regel gilt fürs Fernfeld, Nahfeld behält 4k+1. Die Kopplung
  umzubauen wäre ein Verfahrenswechsel, und der Nutzen ist auf der B70 nie gemessen (B70-Leiter offen).

**Paritätsbedingung:** `NF_OY = (cNy−cey)/2` verlangt gleiche Parität von cNy und cey. Die Regel
**erzwingt** das, indem sie die Nahfeld-y-Spanne notfalls um eine Grobzelle erhöht — von der
Aufrundungsregel ohnehin gedeckt. Ohne diesen Schritt wäre die Parität nur bei 2 / 4 / 6 / 8 mm
erfüllt und bei **3,75 / 4,5 / 5 mm verletzt**; dort hätte der frühere stille Bump die Box eine
Grobzelle breiter gemacht, als das Regelwerk sagt. Der Bump warnt jetzt zusätzlich und darf nie feuern.

## Daraus folgend bei 4 mm (Stand 21.09.2026)

Fahrzeug, skalierte STL: L 4,4364 m, B 1,83855 m, H 1,20833 m
(STL 4,4341 × 1,8376 × 1,2077, von `place()` um 1,000519 auf `si_length` gestreckt).

| | Gitter (Knoten) | Box | Zellspanne |
|---|---|---|---|
| Nahfeld (dx 4 mm) | 1917 × 693 × 473 = **628,4 Mio** | 7,664 × 2,768 × 1,888 m | 1916 × 692 × 472 |
| Fernfeld (dx 16 mm) | 800 × 636 × 608 = **309,4 Mio** | 12,784 × 10,160 × 9,712 m | 799 × 635 × 607 |

Weltlage: `far_x0 = −2,77275 m`, `NF_OX = 145`, `near_x0 = −0,45275 m`, `NF_OY = 201`, `NF_OZ = 0`.

**Ist-Abstände** (alle ≥ Soll):

| | X− | X+ | Y | Z+ |
|---|---|---|---|---|
| Nah | 0,1021 L | 0,6255 L | 0,2528 B | 0,6312 H |
| Fern | 0,6250 L | 1,2566 L | 2,2630 B | 7,0400 H |

**Versperrung 1,875 %** (Querschnitt 98,67 m², A_ref 1,850 m²) — **erstmals unter 2 %**.
Zum Vergleich: 21.09. vormittags 2,74 %, mittags 2,07 %, OF13 mr2v40H 1,93 %.

## Kosten (aus Messungen dieser Maschine)

| | Wert | Quelle |
|---|---|---|
| Nahfeld-Speicherplan | ~28 755 MB, frei **~2 511 MB** (aus der Messung p4_regel4 fortgeschrieben) | 23 118 MB bei 519,1 Mio + 46 B/Zelle Grenzkosten, `logs/p4_apg1.log:222` |
| VRAM verfügbar / Reserve | 32 655 / 2 496 MB | ebenda |
| Fernfeld-Speicherplan | ~13 952 MB von 87 444 MB | 45,1 B/Zelle aus 9 124 MB bei 202,7 Mio, `logs/p4_apg1.log:334` |
| Nahfeldfenster | ~532 ms | 363 ms bei 519,1 Mio (p4_pu8) linear skaliert — **gerechnet, nicht gemessen** |
| Fernfeldschritt | ~468 ms | 338 ms bei 202,4 Mio (p4_neu, `CFD_TIMER_FERN`) — **gerechnet, nicht gemessen** |
| Takt je Grobschritt | ~485 ms, die **iGPU** gibt ihn vor (vorher 380 ms, B70-getaktet) | 468 ms Fernfeld + ~17 ms nicht ueberlappte Anteile (aus p4_pu8: 2286 s / 6015 Grobschritte = 380,0 ms bei 363 ms Fenster) |
| Zeitschleife (501 ms) | ~61 min statt ~48 | p4_pu8 Index 5701 s_wall/s_phys, gemessen; +28 % |

Die Volumenschranke, gegen die jede Fernfeldvergrößerung läuft: bei verstecktem Fernfeld und Nahfeld
am VRAM-Anschlag passen höchstens ~280 Mio Grobzellen ≈ **1 155 m³** Far-Volumen. Daraus folgt
`Versperrung [%] ≥ 0,160 × Far-Länge [m]` — unter 2 % geht nur mit Far-Länge ≤ 12,4 m.

## Was das ablöst

Entfernt aus `basis/fahrzeug_dd.basis` (die Werte sind jetzt Regelwerk, keine Schalter):
`CFD_FAR_LX 12.2720`, `CFD_NEAR_LY 2.6400`, `CFD_NEAR_LZ 1.8560` (Regel Z+ 0,5 H -> 1,824 m), `CFD_NEAR_VOR_MM 96`,
`CFD_N2F_BAND_WAKE_START_X 311`.

`CFD_NEAR_VOR_MM` (validierter Arm, FACETTEN.md: 8-mm-A/B −0,04…−0,05 Cz) geht im neuen X− = 0,1 L
auf: der Nahfeldeinlauf wächst von 326 auf 453 mm. Das Gegenargument im Code (setup.cpp: „der geringe
Einlaufweg wirkt der toten Unterbodenströmung entgegen", Heiko 08.08.) ist überholt — gemessen trägt
das Bodenband am Einlass 99,85 % von u_inf (`export/p4_apg1/boden_laengsprofil.csv`, erste Zeile).
**Diese Zeile ist die Abnahmezahl:** fällt sie unter ~0,99, war der längere Einlauf doch ein Fehler.

## Vorbehalte

1. **Die Box ist neu, die Vergleiche sind es nicht.** Versperrung 2,74 → 2,07 %: jeder Cd/Cz-Vergleich
   gegen `p4_*` und `p375_*` ist ab jetzt kastenfremd. Eine Cd-Verbesserung nach der Umstellung ist
   **kein Modellbefund**.
2. **Fenster und Fernfeldschritt sind hochgerechnet**, nicht bei dieser Größe gemessen. 280 Mio
   Fernfeldzellen sind 38 % über dem Größten, was in v2 gelaufen ist (202,4 Mio). Beim ersten Lauf
   gegen `CFD_TIMER_FERN` gegenrechnen.
3. **Zwei Variablen im selben Wechsel:** Boxgröße und Einlaufweg (+127 mm). Bewusst so entschieden
   (Heiko 21.09.), bei der Auswertung mitführen.
4. **Zweite Fernfeld-Definition** in `main_setup_fernfeld` (Diagnosefall) folgt dem Regelwerk noch
   **nicht** — dort stehen weiter feste Meterwerte. Offen.
5. **Die /16-Regel auf cNx macht den Fernfeld-Nachlauf auflösungsabhängig:** X+ = 1,2566 L (4 mm),
   1,2530 L (8 mm), 1,2929 L (3,75 mm). Eine Gitterstudie 4 → 3,75 mm ändert die Box mit.
6. **3,75 mm ist unter diesem Regelwerk nicht mehr fahrbar:** Nahfeld über 730 Mio Zellen, weit über
   dem VRAM-Anschlag. Bekannt und von Heiko akzeptiert (21.09.) — das Ziel ist ein bei 4 mm
   ausgereizter Kasten mit Luft für mehr Physik (APG, SISM, D3Q27-Band), nicht die nächste Sprosse.
7. **Der Schlupf ist das Physikbudget:** 2 016 MB ≈ 43,8 Mio Nahfeldzellen bei 46 B/Zelle. Alles, was
   ein D3Q27-Band oder zusätzliche Felder brauchen, geht davon ab.

---

## Nachtrag 21.09.2026 abends — gemessene Zahlen aus p4_regel4 (rc 0, 14:44)

Die Prognosen dieses Dokuments wurden erstmals gegen eine Messung gehalten. **Zwei Annahmen waren falsch.**

| Größe | vorhergesagt | **gemessen** |
|---|---|---|
| Nahfeld-Speicherspitze | 28 143 MB | **27 533 MB von 32 655** |
| freier VRAM | — | **3 733 MB** (enthält den Desktop; rechnerisch frei 5 122) |
| Fernfeld-RAM | 12 636 MB | **12 561 MB** von 87 444 |
| Nahfenster | 439 ms | **510 ms** (531 ms Takt × 96,1 %) |
| Takt je Grobschritt | 468 ms, **iGPU** | **531 ms, die B70** |
| Zeitschleife 501 ms | — | **3 991,3 s = 66,5 min**, Index 7 967 |

**Falsch war: „die iGPU gibt den Takt vor".** Der Profiler sagt Nahfeld 4 Schritte **96,1 %**, Fernfeld
synchronisieren und entnehmen **1,9 %**, Kopplung 0,7 %, Kräfte 1,3 %. Das Fernfeld versteckt sich
weiterhin vollständig. Grund: das Nahfenster ist größer als gerechnet, weil **APG die B70 um 17 %
verlangsamt** (Messung 16.09.) — das war in der Fensterrechnung nicht enthalten.

**Folge für die Planung: das NAHFELD ist der Taktgeber.** Jede Nahfeldzelle geht 1:1 auf die Wanduhr,
es gibt dort kein Verstecken. Nur das Fernfeld hat freie Reserve.

### Restluft nach der Messung

**Nahfeld** (Reserve 1 200 MB): +55,1 Mio Zellen = +8,8 % Wanduhr. Davon verbraucht Z+ 0,550 → 0,625 H
rund 26,6 Mio (+4,2 %, 1 224 MB). Bleibt für x+ oder z+ etwa die Hälfte.

**Fernfeld:** Reserve im Fenster. Mit dem neuen Nahfenster (~532 ms) und dem **gerechneten**
Fernfeldschritt 468 ms sind das 64 ms = **+38 Mio Grobzellen**. Entweder-oder:

| | Ny × Nz | Abstand | Querschnitt | Versperrung |
|---|---|---|---|---|
| heute | 576 × 608 | Y 2,002 B / Z+ 7,040 H | 89,4 m² | 2,070 % |
| **nur Y** | **652** × 608 | Y **2,333 B** | 101,2 m² | **1,829 %** |
| **nur Z+** | 576 × **688** | Z+ **8,099 H** | 101,1 m² | **1,829 %** |

**Damit wäre erstmals unter 2 % erreichbar, ohne dass das Fernfeld zum Taktgeber wird.**

**ABER — und das ist der Grund, es NICHT sofort zu tun:** der Fernfeldschritt von 468 ms ist
**gerechnet, nicht gemessen** (aus 338 ms bei 202,4 Mio, p4_neu). Gemessen ist nur, dass das Fernfeld
versteckt ist, also Schritt ≤ 510 ms. Im ungünstigsten Fall (Schritt = 510) bleiben nur 22 ms Reserve
= **+13 Mio Zellen** statt 38. **Vor jeder Fernfeldvergrößerung gehört `CFD_TIMER_FERN` in die Zeile** —
das ist Punkt 2 der Reihenfolge für morgen (REKONSTRUKTION-PLAN.md §11).

## Nachtrag 2, 21.09.2026 — Far Y auf 2,25 B (Heiko)

Fernfeld **800 × 636 × 608 = 309,4 Mio**, Box 12,784 × **10,160** × 9,712 m, Y je **2,263 B**.
**Versperrung 1,875 % — erstmals unter 2 %.** RAM ~13 952 MB von 87 444. Paritäts-Bump feuert nicht.

**Die Reserve ist damit aufgebraucht:** Nahfenster ~532 ms gegen einen **gerechneten** Fernfeldschritt von
517 ms = **15 ms = 2,8 %**. Zum Vergleich: p4_pu8 hatte 25 ms (6,9 %).

**Deshalb läuft der nächste Lauf mit `CFD_TIMER_FERN`.** Die 517 ms sind aus 338 ms bei 202,4 Mio
hochgerechnet (p4_neu); gemessen ist bisher nur „versteckt", also ≤ Nahfenster. Liegt der echte Schritt
über 532 ms, wird die iGPU zum Taktgeber und die Wanduhr steigt entsprechend — der Lauf misst das selbst.

## Nachtrag 21.09.2026, 16:16 — Fern Z+ 7,000 → 6,500 H (Heiko)

**Anlass: gemessene Verdeckungsreserve, nicht geschätzt.** p4_regel5 lief mit `CFD_TIMER_FERN=1`.
Der Schalter ruft direkt nach `lbm_c.run_async(1u)` ein `lbm_c.finish()` (`src/setup.cpp:8880`) und
**hebt die Überlappung Nah/Fern auf** — das ist sein dokumentierter Zweck und macht den Fernfeldschritt
isoliert sichtbar:

| | Kopplung grob→fein | Nahfenster | je Grobschritt |
|---|---|---|---|
| p4_regel4 (ohne Timer, überlappt) | 0,7 % = 3,7 ms | 96,1 % = 510 ms | 531 ms |
| p4_regel5 (mit Timer, seriell) | 48,8 % = **542 ms** | 49,4 % = **549 ms** | 1111 ms |

Fernfeldschritt also **~538 ms** (542 − ~4 ms Drive) gegen ein Nahfenster von **549 ms**:
**Reserve 11 ms = 2,0 %.** Das Fernfeld versteckte sich bei Fern-Y 2,250 B / Z+ 7,000 H gerade noch,
aber ohne Luft — jede weitere Fernvergrößerung hätte die Verdeckung gebrochen.

**Folge der Änderung** (Sollrechnung, Ist steht nach dem Lauf in der Abnahme):
Fern Nz von 608 auf **568** (`(1+6,5)·1,20833 m / 0,016 m = 566,4 → 567`, Knotenzahl 568 durch 4 teilbar),
also −6,6 % Fernzellen. Die Verdeckungsreserve wächst entsprechend; der gemessene Wert kommt aus dem
`[PHASEN]`-Profil des nächsten Laufs.

**Lehre: Diagnoseschalter gehören nicht in eine Produktionszeile.** `CFD_TIMER_FERN=1` kostet exakt
einen Fernfeldschritt je Grobschritt (+101 % Wanduhr) und druckt seine `[FERNFELD-ZEIT]`-Statistik erst
am Laufende (`src/setup.cpp:9728`).

### Korrektur 21.09.2026, 17:10 — die Fernfeldzahl oben ist ein ANWAERM-Wert

`CFD_T_WARMUP` stand in beiden Laeufen auf 0,201 s. p4_regel5 kam bis **t = 0,052 s**, also nie
aus der Anwaermphase heraus. Die Zahlen im Nachtrag 16:16 (Fernfeldschritt ~538 ms, Nahfenster
549 ms, Reserve 2,0 %) stammen damit aus der ANWAERMPHASE und sind keine eingeschwungenen Werte.
Die Zahl aus p4_regel4 (531 ms, Kopplung 0,7 %) ist dagegen ein ENDwert — die beiden sind nicht
direkt vergleichbar. Der Entscheid Z+ 6,500 H bleibt davon unberuehrt, die QUANTITATIVE Reserve
ist offen und wird aus dem `[PHASEN]`-Profil von p4_regel6 NACH t = 0,201 s genommen.

Gueltig ist der Vergleich p4_regel5 -> p4_regel6, weil beide in derselben Phase gemessen wurden:

| | Kopplung | Nahfeld | Fern sync | je Grobschritt | t |
|---|---|---|---|---|---|
| p4_regel5 (CFD_TIMER_FERN=1) | 48,8 % | 49,4 % | — | 1111,0 ms | 0,052 s |
| p4_regel6 (ohne Timer) | 0,7 % | 96,5 % | 1,9 % | **524,5 ms** | 0,150 s |

Das belegt die Ursache (der Timer hebt die Ueberlappung auf), nicht die Groesse der Reserve.

## Nachtrag 21.09.2026, 17:22 — Fern Z+ zurück auf 7,000 H (Heiko), diesmal gemessen

Der Zwischenschritt auf 6,500 H (Nachtrag 16:16) beruhte auf einer Zahl aus der **Anwärmphase**
und war nicht belastbar — siehe Korrektur 17:10. Die Rückkehr auf 7,000 H steht dagegen auf einer
**eingeschwungenen Messung mit einem fremden Instrument**: `/proc/<pid>/fdinfo`, 60-s-Fenster bei
t > 0,201 s, Lauf p4_regel6 um 17:14.

| Gerät | Zähler | busy | abgeleitet bei 524,7 ms Grobschritt |
|---|---|---|---|
| B70 (pdev 04:00.0) | `drm-cycles-ccs` | **94,3 %** | Nahfenster ~495 ms |
| iGPU (pdev 00:02.0) | `drm-engine-compute` | **86,2 %** | Fernschritt ~452 ms |

Reserve bei Z+ 6,500 H (Fern Nz 568): **43 ms = 8,2 %** — gemessen.

Z+ 7,000 H bringt Fern Nz auf 608, **+7,04 % Fernzellen**. **Hochgerechnet** (lineare Skalierung
des Fernschritts mit der Zellzahl — auf dieser Maschine nicht gemessen): Fernschritt ~484 ms gegen
ein unverändertes Nahfenster von ~495 ms, **Reserve ~11 ms = 2,2 %**.

| | Z+ 6,500 H | Z+ 7,000 H |
|---|---|---|
| Fern Nz | 568 | 608 |
| Versperrung | 2,01 % | **1,875 %** |
| Reserve | 8,2 % (gemessen) | ~2,2 % (hochgerechnet) |

**Prüfkriterium für den nächsten Lauf:** bleibt der Grobschritt bei ~525 ms, ist die Hochrechnung
bestätigt und das Nahfeld bleibt Taktgeber. Steigt er Richtung 484 ms und darüber, wird die iGPU
zum Taktgeber und 7,000 H ist zu viel. Die Zahl steht im `[PHASEN]`-Profil und in der
`fdinfo`-Stichprobe.

**Offen und bewusst so stehengelassen:** die lineare Skalierung des Fernschritts mit der Zellzahl
ist eine Annahme, keine Messung dieser Maschine.
