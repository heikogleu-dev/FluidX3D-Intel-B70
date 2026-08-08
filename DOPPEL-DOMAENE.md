# Doppel-Domäne Fahrzeugfall — was gebaut wurde und was bewusst fehlt

Stand 2026-08-08. Alles hier ist am Code belegt oder gemessen; Vermutungen sind als solche markiert.

---

## Warum

Der Einzelgitter-Fahrzeugfall hat **38,4 % Versperrung**. Windkanalpraxis liegt unter 5 %, die
OpenFOAM-Referenz `mr2v40H` bei 1,93 %. Ein Cd aus 38,4 % Versperrung ist mit dieser Referenz
nicht vergleichbar, egal wie sauber der Rest gerechnet ist — die Maskell-Rückrechnung
(ARC R&M 3400) lässt dort Cd ≈ 0,8…1,4 erwarten statt 0,599.

Ein Einzelgitter, das die OpenFOAM-Box bei 4 mm auflöst, hätte 6001 × 3001 × 2001 =
**36 Milliarden Zellen**. Genau dafür gab es im V1-Baum die gekoppelte Fern-/Nahdomäne.

> ★ Korrektur 2026-08-08 (Prüfer-Befund): hier stand vorher „1500×750×500 = 562 Mio Zellen". Das
> ist die Zellzahl bei 16 mm, nicht bei 4 mm — um Faktor 64 daneben. Die Begründung des Falls
> stand damit auf einer falschen Rechnung; die richtige macht sie nur stärker.

---

## Geometrie — die Maße stammen aus V1

Heiko 2026-08-08: die Box-Größen und die Auflösung des V1-Fahrzeugfalls waren bereits optimiert.
Sie stehen hier **physikalisch** statt in Zellen, damit dx frei bleibt; bei dx_f = 4 mm und
ratio = 4 fallen V1s Zellzahlen exakt heraus (nachgerechnet):

| | V1 / hier | OpenFOAM 13 |
|---|---|---|
| Fernfeld | 12,272 × 7,664 × 8,816 m @ 16 mm = 768 × 480 × 552 = **203,5 Mio** | x[−7;17] y[−6;6] z[0;8] m |
| Nahfeld | 6,656 × 2,480 × 1,936 m @ 4 mm = 1665 × 621 × 485 = **501,5 Mio** | — |
| Fußabdruck grob | Ursprung (152, 162, 0), Ausdehnung (417, 156, 122) | — |
| Versperrung | **2,74 %** | 1,93 % |
| Einlauf vor der Nase | 0,60 L (**bewusst kurz**) | 1,08 L |
| Nachlauf hinter dem Heck | 1,57 L | 3,33 L |

Weltkoordinaten nach V1-Konvention: Fahrzeugnase bei x = 0, Mittelebene y = 0, Fahrbahn z = 0.
`near_x0` kommt auf −0,22984 m heraus — V1s Wert auf fünf Stellen.

**Der kurze Einlauf ist Absicht** (Heiko): er wirkt der toten Strömung in den unteren 5 bis 20 mm
und der dadurch stagnierenden Unterbodenströmung entgegen. Ein Prüfer hat ihn als größten
konstruktiven Hebel auf Cd benannt — richtig gesehen und trotzdem so gewollt. Wer ihn verlängern
will, misst es über `CFD_NEAR_OFF_X` gegen den jetzigen Stand.

**Warum genau diese Größen.** Sie balancieren die beiden Geräte aus. Gemessen am 2026-08-08 mit
dem Kugelfall bei identischer Zellzahl: B70 **4648 MLUPs / 572 GB/s**, iGPU **594 MLUPs /
73 GB/s** — die iGPU liefert **12,8 %**. Vier feine Schritte kosten 0,432 s, ein grober 0,343 s:
das Fernfeld braucht **79 % der feinen Zeit** und verschwindet damit gerade noch dahinter. Ein
größeres Fernfeld — etwa die volle OpenFOAM-Box bei 16 mm mit 565 Mio Zellen — wäre der
Flaschenhals und der Lauf 2,3-mal langsamer. Das Fernfeld kostet ∝ 1/ratio⁴; `CFD_RATIO=8` mit
32 mm ist der billigere A/B, kostet aber Auflösung an der Grenzfläche.

---

## Kopplung

Einweg, grob → fein, ein grober Schritt je `ratio` feinen:

1. `extract_plane_macros` liest (ρ, u) auf den fünf groben Ebenen, die auf den Außenflächen der
   Nahfeld-Box liegen. Der Boden z = 0 ist beiden Gittern gemeinsam und wird nicht gekoppelt.
2. `drive_boundary_cubic_lift` interpoliert kubisch auf feine Auflösung (Lagrange 4-Punkt,
   Lagrava/Latt/Chopard JCP 231 (2012) Gl. 38, einseitig 3-Punkt nach Gl. 39 am Rand) und
   schreibt direkt in die TYPE_E-Randzellen des Nahfelds.
3. Das Nahfeld rechnet `ratio` Schritte. Das Fernfeld rechnet dabei asynchron auf der iGPU.

Der Nahfeld-**Auslass x+ wird nicht getrieben** — dort steht der Druckrand, sonst könnte der
aufgelöste Nachlauf nicht abströmen.

### Was bewusst nicht portiert wurde

| weggelassen | Begründung |
|---|---|
| f_neq-Weitergabe (Lagrava Gl. 29) | **beweisbar wirkungslos.** Die Nahfeld-Randzellen sind TYPE_E; `stream_collide` setzt dort `f = f_eq(ρ,u)` (`flagsn_bo==TYPE_E ? feq[i] : …`). Ein eingespritztes f_neq wird im selben Schritt überschrieben. Der alte Baum extrahierte, liftete und skalierte f_neq — und warf es dann weg. |
| Zeitliche Interpolation über die Unterschritte | Der alte Baum rief `set_macros_from_lifted_plane(…, alpha=1.0f)` — bei α = 1 wird nur der neue Puffer gelesen, der alte ist tot. Es gab also nie eine Zeitinterpolation, nur die Verwaltung dafür. |
| Doppelpuffer + `swap_lifted_buffers` | existierte nur für die nie benutzte Zeitinterpolation. Spart hier ~300 MB VRAM. |
| Rückkopplung fein → grob | im alten Baum am 2026-06-14 mit Begründung abgeschaltet: das 4× gröbere Gitter kann den aufgelösten Nachlauf nicht aufnehmen, die dezimierten Daten wirkten als Barriere, um die das Fernfeld herumströmte. |
| `box_filter_19dir_fneq`, `restrict_4to1` | wurden nur von der Rückkopplung benutzt. |
| URF-Blending, Richtungs-Blending | im alten Baum ausdrücklich als „AUSGEKLAMMERT" markiert, nie aktiv. |

Damit schrumpft der Port von rund 1600 Zeilen auf **zwei Kernel und drei Host-Funktionen**.

### Was die Kopplung nicht leistet — ausdrücklich

- Die Rückwirkung des Fahrzeugs auf das Fernfeld läuft **nur über das grobe Gitter**, in dem das
  Fahrzeug ebenfalls voxelisiert ist — nicht über die aufgelöste Nahfeld-Lösung.
- Die Nahfeld-Ränder liegen im gestörten Feld (0,28 m vor der Nase, ~0,3 m neben, ~0,68 m über
  dem Fahrzeug) und werden aus einem Gitter gespeist, das dort 32 mm auflöst. Das ist der Preis.
- Die Randvorgabe wird über `ratio` feine Schritte **festgehalten**. Bei dt_c = 8·10⁻⁵ s wandert
  die Strömung dabei 2,4 mm, also weniger als eine feine Zelle.
- OpenFOAM hat auf Decke und Seiten `zeroGradient` in U **und** p; hier steht Freistrom-Dirichlet.
  Bei 1,9 % Versperrung ist der Unterschied klein — **gemessen ist er nicht**.

---

## Wirksamkeitsnachweis, fest eingebaut

Der Fall prüft sich selbst und schreibt das Ergebnis in den Lauf-Log. Zwei getrennte Fragen:

**(A) Kommt es an?** An jedem Deckungspunkt (beide Ebenenindizes Vielfache von `ratio`) ist die
kubische Interpolation die **Identität**. Der Nahfeld-Randwert muss dort bit-genau dem groben
Wert entsprechen, den der Host verschickt hat. Das prüft die ganze Kette in einem Zug:
Entnahme-Kernel, Indexabbildung, Host-Transfer, Upload, Gewichte, Schreibpfad.

**(B) Bewirkt es etwas?** Wäre die Kopplung ein No-op, stünden die TYPE_E-Zellen exakt auf ihrem
Anfangswert u_x = u_lat. Gemessen wird die größte Abweichung davon.

Gemessen im Kleinlauf (dx_f = 16 mm), grober Schritt 40:

| Fläche | Deckungspunkte | größte Abweichung dort | Abweichung vom Freistrom |
|---|---|---|---|
| x− | 315 | 0,00000000 (identisch) | 2,85 % |
| y− | 795 | siehe Auslasskante | 22,7 % |
| y+ | 795 | siehe Auslasskante | 22,7 % |
| z+ | 1113 | siehe Auslasskante | — |

**Ein Befund der Prüfung, nachgemessen statt vermutet:** Ohne Ausnahme meldete sie 15 Abweichungen
auf den y-Flächen und 21 auf z+. Der ausgegebene Fundort zeigte, dass **alle** auf x = fNx−1
liegen — genau die Anzahl der Deckungspunkte dieser einen Kante. Ursache: `apply_pressure_outlet`
schreibt dort in jedem Zeitschritt und damit nach der Kopplung. Das ist die gewollte Rangfolge
(am Auslass gilt der Auslass); die Kante wird jetzt gezählt, aber nicht bewertet.

Zusätzlich prüft der Fall vor dem ersten Zeitschritt für alle fünf Ebenen:
Deckungspunkt-Konvention, Fahrzeugfreiheit der groben Entnahmeebenen und die Weltlage
(unabhängig von den Indizes nachgerechnet).

---

## Nebenbefunde, beim Bau gefunden und behoben

1. **`s_sparse_tiles_on` war nicht read-once.** Alle Stellen — auch `finalize_sparse_tiles()`,
   das lange nach dem Konstruktor läuft — lasen den statischen Schalter direkt. Mit zwei Domänen
   ist das eine Falle: wer ihn zwischen den Konstruktoren umlegt, ändert rückwirkend das
   Verhalten der ersten Domäne, deren `fi` dann ein 1-Zellen-Platzhalter bliebe. Jetzt genauso
   read-once wie die F-Bounding-Box.
2. **`(uint)(L/dx + 1.5f)` ist kein Runden.** `8.0f/0.128f` ergibt in float 62,499996, +1,5 sind
   63,999996, die Ganzzahl-Umwandlung schneidet auf 63 ab — eine Zelle zu wenig, lautlos und nur
   bei bestimmten dx. Jetzt `floor(x+0.5)`.
3. **Die Mittelebene y = 0 lag nicht auf einem groben Gitterpunkt.** Bei dx_c = 32 mm ist
   6,0/0,032 = 187,5. Die Nahfeld-Box lag dadurch im Kleinlauf 16 mm neben der Symmetrieebene —
   bei einem spiegelsymmetrischen Körper eine Seitenkraft aus dem Nichts. Die y-Halbweite wird
   jetzt aus ganzen groben Zellen aufgebaut.
4. **Slice-Maske (P2/M4).** `(flags & (TYPE_S|TYPE_X)) != 0` malte TYPE_MS-Zellen schwarz —
   TYPE_MS = 0x03 markiert **Fluid**zellen neben bewegten Wänden und enthält TYPE_S. Das
   Diagnosebild log also über die Geometrie, und zwar genau am Boden und am Fahrzeug. Richtig ist
   `(flags & (TYPE_S|TYPE_E)) == TYPE_S`.
5. **`env_u` klemmte auf ≥ 1 (P2/M1).** `CFD_PO_FACES=0` hieß damit nicht „kein Druckauslass",
   sondern Bit 1 = x_min = **der Einlass**. Kein Clamp mehr; wo eine 0 wirklich unzulässig ist,
   klemmt die Aufrufstelle.

---

## Was den Vergleich mit OpenFOAM begrenzt — und was nicht

Ein Prüfer hat am 2026-08-08 darauf hingewiesen, dass die Randbedingungen **nicht** die größte
Fehlerquelle sind. Selbst nachgelesen und bestätigt:

**mr2v40H rechnet stationäres RANS mit k-ω-SST** (`constant/turbulenceProperties`: `simulationType
RAS`, `RASModel kOmegaSST`; `system/controlDict`: `foamRun`, `deltaT 1` als Pseudozeit,
`endTime 1200`). Hier läuft instationäres LBM mit Smagorinsky bei τ = 0,50003, ohne Wandmodell.
Der Unterschied RANS gegen aufgelöste Instationarität plus fehlende Wandbehandlung ist bei einem
Fahrzeug typisch **5 bis 15 % im Cd** — er dominiert alles auf der Liste unten. Eine Abweichung
in dieser Größenordnung ist also **kein** Hinweis auf einen Fehler im Aufbau.

Danach, in absteigender Größe:

| Punkt | Wirkung | Stand |
|---|---|---|
| Einlauf 0,60 L statt 1,08 L, Nahfeld-Rand 0,23 m vor der Nase | direkt auf Cd | **Absicht**, siehe oben |
| Decke/Seiten Freistrom-Dirichlet statt `zeroGradient` | über Maskell ≲3 %, Cd eher zu hoch | nicht gemessen |
| Fahrzeug steht 2,99 mm höher als die Referenzgeometrie | Unterbodenspalt → **Cz** | `CFD_Z_OFFSET_MM=-2.99` ist der A/B |
| Stirnfläche **1,8734** statt A_ref 1,85 (uniforme Skalierung plus Voxelisierung) | Cd **+1,26 %** | wird jetzt **beide** ausgewiesen |
| ν war 1,48·10⁻⁵ statt 1,51·10⁻⁵ | 2 % in Re, bei Re = 9·10⁶ belanglos | **behoben**, Referenzwert ist Default |

---

## Offen

- **Laufzeit.** Zwei Fernfeld-Durchspülungen sind 0,818 s physikalisch = 81 800 feine Schritte
  ≈ 2,5 Stunden. Noch nicht gelaufen.
- **Auflösung an der Grenzfläche.** Bei 16 mm liegen ~20 grobe Zellen zwischen Fahrzeugflanke und
  Nahfeld-Rand. Ob das reicht, ist **nicht gemessen** — `CFD_RATIO=8` ist der A/B dazu.
- **Die Geometrie ist nicht exakt spiegelsymmetrisch.** Ein Prüfer hat 859 STL-Punkte gefunden,
  die weiter als 5 mm von ihrem Spiegelpartner liegen, maximal 24,8 mm, konzentriert auf dem
  Heckdeckel. Bei 4 mm wird das aufgelöst: eine kleine Seitenkraft ist **echt**, kein Artefakt.
- **Keine STL-Facette liegt in der Symmetrieebene** (geprüft: 0 bei 1e-4 m Toleranz, 599 Dreiecke
  schneiden sie, Schale geschlossen). Die Voxelizer-Entartung, die V1 mit einem Halbzellen-Versatz
  umging, kann hier nicht auslösen — der Versatz wird deshalb nicht nachgezogen.
