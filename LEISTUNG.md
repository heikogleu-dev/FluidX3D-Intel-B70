# Leistung — Index und Phasenprofil

Aus V1 nachgezogen (dort `knowledge/performance.md`). Zwei Größen, die V2 bis 2026-08-08 fehlten.

---

## Der Leistungsindex

**Index = Wandsekunden je physikalischer Sekunde. Kleiner ist besser.**

Er ist die einzige Zahl, die über Auflösungen und Gitterverhältnisse hinweg vergleichbar bleibt.
MLUPs sind es nicht: sie haben die Zellzahl schon eingerechnet, und bei zwei Domänen wird die
Konsolenanzeige ohnehin unbrauchbar — sie mischt die grobe Zellzahl mit der feinen Schrittzeit
(von einem Prüfer am 2026-08-08 belegt). Genau deshalb hat V1 diesen Index eingeführt.

### Gemessen 2026-08-08

| | Konfiguration | Index |
|---|---|---:|
| **V1** (Standardlauf, `knowledge/performance.md`, 2026-06-23) | Nah 501 M @ 4 mm B70 + Fern 202,8 M @ 16 mm iGPU | **~12.000** |
| **V2** (`dd_lauf01`) | Nah 501,5 M @ 4 mm B70 + Fern 203,5 M @ 16 mm iGPU | **≈ 9.100** |

**V2 ist rund 24 % schneller** — bei identischer Konfiguration und identischer Hardware.

Bemerkenswert daran: V2 hat `UPDATE_FIELDS` **eingeschaltet**, V1 in keiner Konfiguration. Das
kostet 10–15 % Durchsatz, weil `stream_collide` ρ und u jeden Schritt schreibt statt nie. V2 ist
also schneller, obwohl es mehr tut.

Die V2-Zahl stammt aus den Zeitstempeln der Schnittbilder bei bekannten physikalischen Zeiten,
über vier Fenster von je 50 ms:

| Fenster | Wandzeit | Index |
|---|---:|---:|
| 1 → 51 ms | 468 s | 9.356 |
| 51 → 101 ms | 472 s | 9.433 |
| 101 → 151 ms | 459 s | 9.188 |
| 151 → 201 ms | 419 s | 8.377 |

**Vorbehalt, ausdrücklich:** der Lauf war ab 0,15 s tot (siehe [OFFENE-PUNKTE.md](OFFENE-PUNKTE.md)).
Ein eingefrorenes Feld rechnet genauso teuer weiter — als **Durchsatzmessung** ist die Zahl damit
gültig, als Ergebnis nicht. Das letzte Fenster liegt vollständig im toten Bereich und ist
trotzdem nicht schneller, was die Aussage stützt.

Seit 2026-08-08 rechnet das Programm den Index **selbst** aus und gibt ihn bei jeder Abtastung
aus, zusammen mit dem Phasenprofil.

---

## Das Phasenprofil

Wo die Zeit **wirklich** sitzt, je grobem Zeitschritt:

- **Kopplung grob → fein** — der kubische Lift auf die vier getriebenen Randflächen
- **Nahfeld** — die `ratio` feinen Zeitschritte
- **Fernfeld synchronisieren und entnehmen** — `finish()` plus die vier Ebenen-Entnahmen
- **Kräfte** — `update_force_field` und `object_force` auf beiden Gittern
- **Schnitte** — Rücklesen von u und flags plus PNG

### Warum das gemessen und nicht geschätzt wird

V1s Lehre dazu ist teuer bezahlt und steht dort wörtlich: ein Theorie-Audit suchte den Hebel im
Async-Overlap und in der Kernel-Fusion — während die größte Quelle eine **96-MB-Verschwendung je
Transfer** war, zehnmal pro äußerem Schritt. Die Messung fand sie in Minuten, das Audit gar nicht.

> „Bei Perf-Defizit ZUERST messen wo die Zeit wirklich sitzt, dann fixen."

Dasselbe Muster hat sich am 2026-08-08 in V2 wiederholt: ein Prüfer fand, dass die grobe
x+-Ebene in **jedem** groben Schritt entnommen, über PCIe geholt und weggeworfen wurde —
blockierendes `finish_queue`, 304 kB, Host-Kopierschleife, kein Leser.

---

## Hardware-Bezugswerte, gemessen 2026-08-08

Kugelfall, identische Zellzahl (14,26 M), FP16C, D3Q19, TRT:

| Gerät | MLUPs | Bandbreite |
|---|---:|---:|
| Intel Arc Pro B70 (dGPU) | **4.648** | 572 GB/s |
| Intel Graphics (iGPU) | **594** | 73 GB/s |

Die iGPU liefert **12,8 %** der B70. Daraus folgt die Aufteilung der Doppel-Domäne: das
Fernfeld kostet ∝ 1/ratio⁴, und bei ratio = 4 mit den V1-Maßen braucht ein grober Schritt
0,343 s gegen 0,432 s für vier feine — das Fernfeld verschwindet also gerade noch hinter dem
Nahfeld (79 % Auslastung). Ein größeres Fernfeld wäre der Flaschenhals.

### Nachtrag 2026-08-27: iGPU über die Fallgröße und über die Gitterform vermessen

Zwei Serien auf der iGPU, `CFD_CASE=fernfeld` (Einzelgitter, feste Box, nur `CFD_DX` bzw.
`CFD_FAR_L*` variiert), je eine Variable, Census vor und nach jeder Serie.

**Skalierung** (`logs/s_igpu_skala_serie.txt`, 12 Arme, `logs/s_igpu_dx*.log`):

| Zellen | ns/Zelle | | Zellen | ns/Zelle |
|---:|---:|---|---:|---:|
| 25,6 M | 1,828 | | 117,8 M | 1,821 |
| 47,6 M | 1,832 | | 134,0 M | 1,821 |
| 74,4 M | 1,845 | | 175,7 M | 1,821 |
| 92,7 M | 1,845 | | 203,5 M | 1,815 |
| 104,4 M | 1,835 | | 278,7 M | 1,818 |
| 110,8 M | 1,842 | | 397,4 M | 1,828 |

Faktor 15,5 in der Zellzahl, **Spannweite 1,7 %** — kein Trend, kein Sprung. Insbesondere kein
Sprung am Buffer-Limit: die iGPU meldet „Buffer Limits 4095 MB global", der DDF-Block ist unter
FP16C 38 B je Zelle und erreicht das bei 113,0 M Zellen. Die Punkte 110,8 M (4017 MB) und
117,8 M (4268 MB) klammern die Grenze ein und liefern 1,842 gegen 1,821 — der größere Fall ist
minimal schneller.

**Gitterform** (`logs/p_align_serie.txt`, 5 Arme): x = 1024 (aligned) 1,818 · x = 1025 1,818 ·
x = 1021 (prim) 1,825 · y = 512 1,795 · y = 481 1,815 ns/Zelle. Spannweite 1,7 %.
Die Regel „Coarse-Nx immer ÷64" (BAUPLAN-KOPPLUNG.md:299, aus einer V1-Messung vom 15.06.2026)
ist damit auf dem heutigen Stack **nicht mehr messbar**.

**Einordnung — und was NICHT verglichen werden darf:**
* Gegen die V1-Coarse-Step-Messung (V1-Projekt, `knowledge/hardware.md`, 15.06.2026, **andere
  Fassung**, gleiche Hardware, vergleichbarer Fall und Größenordnung ~207 M Zellen): dort
  **3,20 ns/Zelle** im besten Fall gegen **1,815** heute — Faktor 1,76. Zwischen beiden liegen
  Treiber-, Kernel- **und** Codestände; welcher Anteil woher kommt, sagt die Messung nicht.
* Gegen den Bezugswert oben (594 MLUPs = 1,68 ns/Zelle): **nicht vergleichbar.** Der stammt vom
  Kugelfall mit 14,26 M Zellen und TRT, diese Serie vom Fernfeldfall mit 25–397 M und SRT.
  Der Fernfeldfall liegt durchweg bei 542–551 MLUPs; ob die Differenz an Fallgröße, Kollisions-
  operator oder Randbedingungen liegt, ist **nicht gemessen**. Wer die 594 als Zielwert für
  Fernfeldläufe nimmt, vergleicht zwei verschiedene Fälle.

**Folge für die Box-Planung:** zwei Nebenbedingungen, die bisher als gesetzt galten — Nx-Vielfache
und das Meiden großer Fälle auf der iGPU — kosten heute nichts und bringen nichts. Für die
Fernfeld-Dimensionierung zählen nur noch Rechenzeit und Versperrung.

---

## Was V1 an Profiler hatte, das hier noch fehlt

- **GPU-Auslastung je Gerät.** V1 las sie root-frei aus `/proc/<pid>/fdinfo`:
  auf der B70 (xe-Treiber) über `drm-cycles-ccs` gegen `drm-total-cycles-ccs`, auf der iGPU
  (i915) über `drm-engine-compute`. `intel_gpu_top` funktioniert **nur** auf der iGPU, nicht auf
  der B70 — der xe-Treiber hat keine i915-PMU. V1 maß damit B70 98,4 % und iGPU ≥100 %.
- **Transfer-Zähler** (`XFER_DIAG`), der die 96-MB-Verschwendung gefunden hat.

Beides ist wenig Aufwand und lohnt, sobald der Index nicht mehr von selbst erklärbar ist.
