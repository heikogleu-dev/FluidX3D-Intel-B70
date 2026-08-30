# Übergabe FluidX3D-v2 — Stand 30.08.2026, 21:20

## Kontext in drei Sätzen
LBM-Strömungsrechnung (D3Q19, SRT, FP16S) eines MR2 Widebody bei 30 m/s auf einer Intel Arc Pro B70
(32,6 GB), Dual-Domain: Nahfeld 4 mm um das Fahrzeug, Fernfeld 16 mm, beidseitig gekoppelt.
Referenz ist OpenFOAM 13 (`~/CFD-Cases/mr2v40H`): **Cd 0,599 / Cz −1,301**.
Repo `/home/heiko/CFD/FluidX3D-v2` @ `1bbb70c`, gepusht nach `heikogleu-dev/FluidX3D-Intel-B70`.

## Wo wir stehen (4 mm, gemessen 30.08.)

| | FX 4 mm | OF13 | Abweichung |
|---|---|---|---|
| Cd (Karosserie, ohne Radkontaktband) | 0,524 | 0,599 | −12 % |
| Cz | −0,902 | −1,301 | −31 % |
| Ablösepunkt am Dach | 3,498 m | 3,629 m | 131 mm zu früh |
| Impulsverlustdicke θ am Dachscheitel | 9,34 mm | 2,00 mm | **4,7× zu dick** |
| Formfaktor H | 2,04 | 1,18 | H=2 heißt "kurz vor Ablösung" |
| Druckerholung Δcp Dach→Heck | 0,883 | 0,903 | −2 % |
| dp/dx über der Heckscheibe | 292 Pa/m | 288 Pa/m | +1 % |

**Der Druckhaushalt stimmt. Die Grenzschicht nicht.** Das ist die zentrale Erkenntnis des Tages:
die Rechnung bekommt die richtigen Drücke, kommt aber mit einer viermal zu dicken, ausgezehrten
Grenzschicht am Dachscheitel an — und löst deshalb zu früh ab.

## Was am 30.08. gemessen wurde

1. **Weg F (Zellkraft statt Slip an Rückfallzellen, `CFD_FAC_KRAFT`) ist ein validiertes Negativ.**
   Zehn Kanalarme plus 8-mm-Fahrzeug gepaart: der Mechanismus wirkt exakt wie gebaut (Wirkpfadzähler
   Slot 70 == Slot 69) und verschlechtert überall — Kanal kipp26 c_f-Faktor 2,23 → 3,42, Fahrzeug
   cd_rest +0,26 bei 18 σ. Der Fehler sitzt nicht in der Erfüllung des Wandmodells, sondern in
   seinem **Eingang**: an Treppenzellen wird u_t aus einer abgeschatteten Zelle abgetastet
   (Klassen-Diagnostik: τ_Modell/τ_Ziel = 0,04–0,20 je Klasse, u_t der Eckzelle Faktor 4,4 zu klein).

2. **BB-Nullinie gemessen** (reines Bounce-Back, sonst volle Basis): bei 4 mm cd_rest 0,573 /
   cz_rest −0,779 gegen Wandmodell 0,524 / −0,902. Beim **Widerstand ist reines BB näher an OF13**
   (0,598 gegen 0,524 bei Ziel 0,599) — das Wandmodell *senkt* Cd, weil es zu wenig Wandschub
   anzielt und dabei Slip statt Bounce-Back anwendet. Beim Abtrieb schrumpft der Wandmodell-Hebel
   mit der Auflösung: 8 mm +0,23 Cz, 4 mm nur noch +0,10.

3. **Die Druckrelaxations-Hypothese ist widerlegt.** `u_lat = 0,075` ist fest, die Mach-Zahl bei
   8 und 4 mm bitgleich (0,1299) — ein Ma²-Fehler kann kein Symptom erzeugen, das beim Halbieren
   von dx um Faktor 3,4 kleiner wird. Ebenso ausgeschlossen: rho-Randaufprägung (klingt exponentiell
   ab, erreicht das Dach bei 4 mm mit 5e-11 nicht), Wandmodell-Abdeckung (Rückfallquote 42,97 %
   gegen 42,34 % = dx⁰), Wandfunktion selbst (Kanal c_f/Ziel 0,716/0,669/0,725 über 3,8-fache
   Verfeinerung).

4. **Der offene Widerspruch** — und der Kern der nächsten Sitzung. Zwei unabhängige Agenten messen
   dasselbe Symptom und benennen verschiedene Ursachen, **beide skalieren mit dx**:
   - **(A) wandnahe SGS-Überviskosität**: ν_t/ν₀ = 60/45/15 bei dx⁺ 519/273/137; die zu dicke
     Schicht sitzt auf dem *stufenfreien* Dachstück und tritt auch im reinen Bounce-Back auf.
   - **(B) Voxeltreppe**: θ ∝ dx exakt (Faktor 2,14–2,32 zwischen 8 und 4 mm), k/δ = 12 % bei 4 mm
     gegen Stolperdraht-Wirkschwelle 1–3 %, und BB unterscheidet sich am Dach kaum vom Wandmodell.

   Möglich ist, dass beide recht haben: die Treppe stromauf erzeugt den Impulsverlust, der SGS hält
   ihn aufrecht. **Der Skalierungsfilter trennt sie nicht** — dafür braucht es den Trenntest unten.

## Ehrliche Einordnung der Gesamtsituation

**Das Gute:** Der Druckhaushalt stimmt bei 4 mm auf 1–2 %. Das ist keine Kleinigkeit — es heißt,
Geometrie, Kopplung, Randbedingungen und Auslass sind im Wesentlichen richtig. Das Lift/Drag-Verhältnis
ist nah an OF13. Die Diagnostik ist heute an einem Punkt, an dem Fehlerursachen *gemessen* statt
vermutet werden: Wirkpfadzähler an jedem Mechanismus, Klassen-Diagnostik je Wandtyp, gepaarte Arme
mit einer Variablen, ein validiertes Speichermodell (29 673 MB Vorhersage gegen 29 672 MB Lauflog).

**Das Unangenehme:** Der zentrale physikalische Fehler ist seit Wochen derselbe und wandert nur die
Erklärungskette entlang — erst "Wandmodell-Abdeckung", dann "Voxeltreppe", jetzt "SGS oder Treppe".
Jede Runde hat einen Kandidaten *ausgeschlossen*, aber noch keine Reparatur geliefert, die den Fehler
messbar verkleinert. Weg F war die vierte gebaute Idee, die sich als Negativ erwies. Das ist
wissenschaftlich sauber und fortschrittlich, aber es ist langsam.

**Die harte Randbedingung:** Es gibt **keine Gitterstufe unter 4 mm**. Nachgerechnet am validierten
Speichermodell bräuchte 2 mm 237 GB VRAM (7,9× Budget) plus 84 GB Fernfeld im RAM. Feinstes fahrbares
dx mit dieser Box: 3,978 mm. Auflösung ist damit als Lösungsweg versperrt — was bleibt, ist besseres
Modellieren auf dem Gitter, das wir haben. Umgekehrt macht das **8 mm zum Prüfstand**: jede echte
Reparatur muss den 8-mm-Lauf Richtung 4-mm-Verhalten schieben, und das kostet 6 Minuten pro Arm.

**Ein Risiko, das benannt gehört:** Ein Agent hat im 8-mm-Gitter 2857 Fahrzeugzellen als eine Zelle
dünne Membran exakt auf y = 0 gefunden (Heckscheibe, Kofferraum, Nachlauf; bei 4 mm 8 Zellen). Das ist
**noch nicht gegengeprüft**. Falls es stimmt, ist die 8-mm-Sprosse als Cz-Grobsprosse unbrauchbar,
solange der Defekt steht — und damit der Prüfstand selbst beschädigt. Das ist der erste Punkt, der
Gewissheit braucht.

## Was die zwei Fixes bewirken sollten — grobe Schätzung mit Unsicherheit

Kalibriert an der einzigen verfügbaren Sprosse (8 → 4 mm): eine Halbierung von θ (19,97 → 9,34 mm)
brachte +384 mm Ablösepunkt und +0,82 Cz. Diese Beziehung sättigt aber, je näher man der Referenz
kommt — mit **zwei Datenpunkten lässt sich keine Kurve, nur eine Richtung** belegen.

**Fix 1 — Aufgabenteilung SGS/Wandmodell (bzw. WALE/Sigma gegen spuriose ν_t).**
Zielgröße ist θ am Dachscheitel: 9,34 mm heute, 2,00 mm wäre Referenz. Wenn der Fix θ etwa halbiert
(auf ~4–5 mm), sollte der Ablösepunkt die 131-mm-Lücke weitgehend schließen und **Cz um 0,15–0,30**
zulegen — von −0,90 auf −1,05 bis −1,20 bei Ziel −1,30. Das ist die optimistische Hälfte; schließt
der Fix nur ein Viertel der θ-Lücke, bleibt es bei +0,05–0,10.
*Größte Unsicherheit:* ob die Doppelzählung überhaupt der dominante Anteil ist. Bei y⁺ = 68 ist
ν_t/ν₀ ≈ 30 der physikalisch **richtige** Log-Bereichs-Wert (κ·y⁺ = 28) — der Smagorinsky ist nicht
zu viskos, er dupliziert nur die Aufgabe des Wandmodells. `CFD_SGS_WANDFREI` entfernt deshalb auch
legitime Viskosität und **kann überschießen**; es ist der Extremtest, nicht die Endlösung.

**Fix 2 — Wandmodell-Eingang (`CFD_FAC_NACHBAR`: u_t und Wandabstand aus der zweiten Fluidzelle
entlang der Normale statt aus der abgeschatteten Wandzelle).**
Zielgröße ist Cd: heute 0,524, reines BB liefert 0,598, OF13 0,599. Das Wandmodell *verliert*
gegenüber BB 0,074 Cd, weil es an Treppenzellen auf 4–20 % des richtigen Wandschubs zielt. Ein
korrekter Eingang sollte den Großteil davon zurückholen — **Cd 0,55–0,60** erwartbar. Auf Cz wirkt
er indirekt (mehr Wandschub → dünnere Grenzschicht → spätere Ablösung), aber schwächer als Fix 1;
+0,05 wäre schon gut. Nebenwirkung, die geprüft werden muss: der Fix ersetzt zugleich den Handwert
`CFD_FAC_UTKORR=1,5` durch eine hergeleitete Größe — an der ebenen Wand ändert sich das Ergebnis
dadurch **per Konstruktion**, ein Bitanker gilt dort nicht.

**Zusammen, wenn beide greifen:** Cd 0,57–0,60 (Ziel 0,599), Cz −1,05 bis −1,25 (Ziel −1,301).
Das wäre der erste Stand, bei dem beide Beiwerte innerhalb ~10 % der Referenz lägen. Ich halte das
für erreichbar, aber nicht für wahrscheinlich in einem Zug — realistischer sind zwei bis drei
Iterationen mit je einer Teilkorrektur.

## Performance-Einschätzung

| Kennzahl | 4 mm | Quelle |
|---|---|---|
| Laufzeit 0,501 s physikalisch | **1 h 27** | queue_status 19:23 → 20:50 |
| Perf-Index (kleiner = besser) | **10 400** | export/p4_satgate0_perf.csv |
| ms je Grobschritt | 416 | dito |
| Durchsatz | 2 060 MLUPS / 253 GB/s | Lauflog |
| Zeitanteile | Nahfeld 96 %, Fernfeld 2,4 %, Kopplung 0,9 %, Kräfte 0,6–1,0 %, Slices 0,0 % | Profiler |
| VRAM | 29 672 MB von 32 655 (Reserve 2 496 MB, Desktop ~1,1 GB) | Lauflog |
| Hardware | B70: 256 CUs @ 2,8 GHz, 22,9 TFLOPs | Lauflog |

**Einordnung:** Der Lauf ist praktisch reine Nahfeldrechnung — Kopplung, Kräfte und Slices kosten
zusammen unter 2 %. Optimierung lohnt daher nur im `stream_collide`-Kernel selbst. Der ist wie jedes
LBM **bandbreitendominiert** (19 DDFs lesen und schreiben je Zelle und Schritt), nicht rechenlimitiert
bei 22,9 TFLOPs. Praktische Folge für die anstehenden Fixes: **zusätzliche Arithmetik im SUBGRID-Block
(WALE, Sigma) kostet fast nichts**, weil sie auf bereits geladenen Daten arbeitet — der fneq-Tensor
wird für Smagorinsky ohnehin gebildet. Auch `CFD_FAC_NACHBAR` ist billig (drei float-Reads aus dem
u-Feld je Facettenzelle, die 18 Flag-Reads laufen ohnehin). Die Klassen-Diagnostik kostet 40 Byte je
Facette (~125 MB bei 4 mm) und ist per Schalter aus.

Offene Perf-Posten stehen in `ARBEITSLISTE.md`, keiner davon ist dringend: UPDATE_FIELDS ablösen
(10–15 %, großer Umbau), Sparse Tiling (im Dual-Domain gesperrt), Work-Group-Size. Der Index liegt
~12 % über der V1-Referenz — vertretbar für den Funktionszuwachs.

## Konkret als Nächstes (Reihenfolge)

0. **Prüfagent auf den Diff `fd1bcad`** (drei gebaute, GPU-ungeprüfte Schalter: `CFD_FAC_KDIAG`,
   `CFD_FAC_NACHBAR`, `CFD_FAC_MESSNUR`), dann CPU-Rauchtest, iGPU, dann erst B70.
   **Nie einen FluidX3D-Prozess neben einem laufenden GPU-Lauf starten** — auch keinen 25-s-CPU-Test.
1. **Membran-Befund gegenprüfen** (2857 Zellen auf y = 0 im 8-mm-Gitter). Ohne Gewissheit ist der
   Prüfstand fraglich.
2. **Trenntest, 6 Minuten:** `w_bb` + `CFD_SGS_WANDFREI=1`. Kriterium δ₉₀ bei x = 2,50 m — Bezug
   105 mm, Ziel (4 mm) 41 mm; **≤ 72 mm ⇒ SGS ist die dominante Ursache**, ≥ 97 mm ⇒ Treppe.
3. **Im selben Zug messen**, was ν_t *ist* gegen κ·y⁺ *soll* (`CFD_SGS_DIAG` Slots 30–34 plus
   `CFD_FAC_KDIAG`). **Achtung:** `SGS_DIAG` und `SGS_WANDFREI` dürfen nicht in denselben Arm —
   WANDFREI überspringt den SUBGRID-Block, die Diagnose-Slots blieben leer.
4. Je nach Ergebnis: **Aufgabenteilung** (Doppelzählung, B57) **oder WALE/Sigma** (spuriose ν_t).
   Das sind zwei verschiedene Reparaturen, keine Stufen desselben Fixes.
5. `CFD_FAC_NACHBAR` gepaart auf 8 mm.
6. **4 mm nur für einen validierten Gewinner.**

**Vor jeder Serie:** Bezugswerte der Sprosse per `recall()` prüfen und die Serienzeile aus
`werkzeuge/basis_zeile.py` erzeugen, nie aus einer alten Serie abschreiben. Genau das Fehlen war
heute die Ursache eines halben Fehltags (Kanal-Leiter lief auf der V1- statt der Produktionskonfiguration).

## Zu korrigierende Altlasten in der Doku
- **B58s "durchgehende Ablösung ab x = 3,234 m" ist widerlegt** — es gab kein Skript dazu, nachgerechnet
  ist dort keine durchgehende Strecke; belegt ist nur "ab dort überwiegt Rückströmung in der ersten
  Zelle". Sauberes Kriterium: 24 mm Wandabstand.
- `yplus_histogramm.csv` ist **nicht** y⁺ (Faktor 18 daneben) — nur `yplus_facetten.csv` benutzen.
- `forces.csv`/`object_force` ist in Wandmodell-Armen **phantombehaftet** (cd 6,76 statt 1,02); dort
  gilt allein der Facettenpfad `cd_facetten.csv`. Für BB-Arme ist es umgekehrt gültig.
