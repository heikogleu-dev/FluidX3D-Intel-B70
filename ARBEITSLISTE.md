# Arbeitsliste — nach Wichtigkeit, häppchenweise abzuarbeiten

Stand 2026-08-09 abends. Eine Liste, eine Reihenfolge. Jeder Punkt ist so geschnitten, dass er in
einer Sitzung erledigt werden kann.

**Nachtrag 2026-08-15:** 0A (y⁺ gemessen: Median 1122, nicht 137), A1–A3 (Kanal steht, Rauwand
verstanden, WFB nach Han et al. gebaut — c_f gitterunabhängig flach, Details `WANDMODELL.md`) und
das Komplett-Audit samt Fix-Schleife (`AUDIT-BEFUNDE.md`, sauber bis Commit 06b432b) sind erledigt.
**Stand C1b (2026-08-16 abends): Leitdokument ist `FACETTEN.md`** (geltende Architektur,
Schalter-Referenz, Abnahmen mit Zahl, offene Punkte; die acht FACETTEN-*-Altdateien sind Archiv,
Verzeichnis in `FACETTEN-ARCHIV.md`). Kurzstand: Paartausch-Stufen 0–2 + Cd-Pfad bestanden
(Kanal BITGLEICH zur z-WFB, K1–K5); iMEM auf 3×3-Normal-Nullung umgebaut, mechanisch sauber
(I1-Äquivalenz +1,2 % im Rauschboden, N1-Kontamination −763k → −0,16), aber **Torus-N2 mit drei
Varianten (instantan/EMA/PEMA) verletzt** — und der IR3-Audit fand einen z-Saum-Geometriefehler
in ALLEN bisherigen Torus-Läufen (gefixt 2a8bd20).

**Nächste Schritte (in dieser Reihenfolge):**
1. **Torus-N2-Wiederholung nach Saum-Fix** (läuft) + **Klemmskalen-Messarm ±4ut**.
2. **Familienfrage** (Kompensation vs. Ersetzung vs. Richterwechsel Kugel) — bis dahin
   SUSPENDIERT, Entscheid nur mit Saum-fixer Zahl.
3. **Kugel J4** (Census, Arme AUS/4/3, beide Cd-Wege, FP32-Sprosse) → **Hangauf I2b** →
   **Fahrzeug Stufe 5** (y⁺ 1122 → ~140). Details und Audit-Reste: `FACETTEN.md` Abschnitt 4.

**Das Ziel, an dem alles gemessen wird:** Cd und Cz des Fahrzeugs gegen OpenFOAM 13
(**0,599 / −1,301**). Stand 2026-08-09: **1,20 / −0,95** — und das ohne Wandmodell, mit sichtbar totem
Unterboden.

**Zwei harte Randbedingungen (Heiko):** kein Lauf über **2,5 h**. Fernfeld immer auf der **iGPU**
(Kopplung 0,8 % gegen 10,8 % bei beiden auf der B70).

---

## Block 0 — zwei Tests, die fast nichts kosten und die Reihenfolge ändern könnten
> **Status 2026-08-15: 0A ERLEDIGT** (gemessen: Median 1122, s. WANDMODELL.md). **0B ÜBERHOLT** —
> die Prämisse c_f ∝ 1/N² ist durch die Kanalleiter widerlegt (Rauwand-Befund, WANDMODELL.md).

Beide kamen aus der Gegenprüfung und ziehen an der Wurzel der Blöcke A und C.

**0A. y⁺ am Fahrzeug MESSEN statt korrelieren.**
Die Zahl 137 ist eine Plattenkorrelation bei x = L, **nie gemessen**. `update_force_field` liefert
die echte τ_w-Verteilung — **ein einziges Sample eines vorhandenen Laufs** ergibt das reale
y⁺-Histogramm. Kosten: ein Readback.
*Warum zuerst:* die ganze Wandmodell- und Kanalplanung steht auf dieser einen ungemessenen Zahl.

**0B. Fahrzeuglauf bei dx = 6 mm.**
Das Modell aus dem Kanalplan sagt: es gibt **genau eine** Gitterweite, bei der ein Lauf ohne
Wandmodell zufällig das richtige c_f liefert — δ/dx = 10,5, am Fahrzeug **dx ≈ 6,4 mm**. Unser
4-mm-Gitter liegt auf der falschen Seite. Ein 6-mm-Lauf ist **billiger als der aktuelle** und prüft
die Vorhersage direkt am Zielobjekt, ohne Kanal und ohne Wandmodell.
*Trifft es zu, ist das ein starker Hinweis, dass der aktuelle Cd-Fehler wesentlich am fehlenden
Wandmodell hängt — und wir hätten es für den Preis eines Laufs gezeigt.*

---

## Block A — Messgrundlage schaffen (billig, macht alles andere erst bewertbar)
> **Status 2026-08-15: A1–A4 KOMPLETT ERLEDIGT** (Kanal steht samt Vorrichtungs-Validierung,
> Leiter gemessen, Lee&Moser ausgewertet — alles in WANDMODELL.md).

**A1. Turbulenter Kanal bauen** (`main_setup_kanal`, `CFD_CASE=kanal`).
Wandnormale z, periodisch in x/y, ruhende Wände, Antrieb über `VOLUME_FORCE` (endlich ein Nutzen —
bisher warnt jeder Start, dass die Kraft null ist). **Feste Durchflussrate, nicht feste Kraft** —
sonst läuft der Arm ohne Wandmodell auf feinen Gittern in den Überschall und ist still falsch.
*Warum wichtig:* erste externe Referenz des Projekts überhaupt. Der Fahrzeug-Betriebspunkt als Kanal
ist **N = 38, 286 000 Zellen, 19 MB, 31–60 s**.
★ **KORRIGIERT nach Gegenprüfung:** y⁺₁ und δ/dx sind **keine zwei Bedingungen** —
y⁺₁·(2δ/dx) = Re_τ, also 137·2·16,7 = **4576**. Bei Re_τ = 5186 trifft man nur eines von beiden.
Und die −60 % gehören zu N = 34; bei N = 38 sind es **−69 %**. Details in `WANDMODELL.md`.
★ **Der Kollisionsoperator muss vorher entschieden werden:** der Wandversatz beträgt bei einem
Log-Profil ~0,25 Zellen = **50 % des Wandabstands der ersten Zelle** (gemessen, D2Q9-TRT-Poiseuille).
Kontrollarm SRT gegen TRT kostet 60 s, `CFD_LAMBDA` existiert.
★ **FP16C muss mitgeprüft werden** — der Kanal muss in derselben Genauigkeit laufen wie das Fahrzeug.

**A2. Die Messvorrichtung validieren, bevor irgendetwas gemessen wird.**
Zwei unabhängige τ_w (Kraftbilanz f·δ gegen Impulsaustausch aus `object_force`) müssen auf wenige
Prozent übereinstimmen. Gesamtspannungsbilanz muss exakt 1 − z/δ ergeben (Toleranz 2 %).
*Das braucht keine DNS-Datei* — es ist ein reines Code-Korrektheits-Gate und kostet Minuten.

**A3. Arm 0: ohne Wandmodell über die Auflösungsleiter.**
Erwartung, hergeleitet: **c_f ∝ 1/N²**, bei Fahrzeugauflösung rund **−60 %** gegen DNS. Und es gibt
genau **eine** Gitterweite, bei der es zufällig stimmt — Gitterglück, keine Physik.
*Warum wichtig:* ohne diesen Ausgangspunkt lässt sich später keine Verbesserung dem Wandmodell
zuordnen. **Haltbare Leiter (2,5-h-Grenze, beide Durchsätze): N = 38 / 54 / 76 / 108** — zusammen unter 4 h
für alle Arme. N = 152 verletzt sie bei 2400 MLUP/s, N = 216 und 304 immer. **Kein Checkpoint im
Code** — ein Lauf über 2,5 h ist nicht teilbar.
★ Der geplante dritte Arm (van Driest) ist am Zielpunkt **widerlegt**: bei y⁺ = 137 leistet er
**1,0 %**. Der informative dritte Arm wäre **SUBGRID ganz aus**.

**A4. Referenzdaten ablegen.** Lee & Moser 2015 (`turbulence.oden.utexas.edu/channel2015/data/`),
Re_τ 5186/1995/1000. Daraus bereits berechnet: **U_b⁺ = 24,104, c_f = 3,4424·10⁻³** bei Re_τ = 5186.
Hoyas & Jiménez ist **entbehrlich** — Lee & Moser deckt Re_τ = 1995 mit demselben Format ab.

---

## Block B — der größte bekannte physikalische Fehler

**B1. Der tote Bodenstreifen über der mitbewegten Fahrbahn.**
Im Schnitt bei 181 ms deutlich sichtbar, bei 21 ms noch nicht — er **wächst über die Laufzeit**.
Bei u_Wand = u_∞ und τ ≈ 0,5 darf dort **keine Grenzschicht** entstehen.
Zu klären: Wo beginnt er — am Einlass (dann war V1s Überschreibung über drei Zellen die richtige
Antwort) oder über die ganze Länge (dann ist der Impulsübertrag der Wand zu schwach)?

**B2. Voraussetzung dafür: der Wandwirksamkeits-Nachweis muss über die ZEIT laufen.**
Er läuft heute nur bei t = 0 und meldet dort brav 1,001 — er *kann* einen über 180 ms wachsenden
Streifen nicht sehen. Profil über der Fahrbahn bei **jedem Sample** in die CSV.

**B3. Der tote Unterboden.**
Durchgehend blau von Front bis Heck. Bei diesem Fahrzeug kommt der Abtrieb wesentlich von dort —
das erklärt Cz = −0,95 statt −1,301 und das zeitweise Kippen ins Positive.
Zu trennen: Folge von B1 (tote Anströmung wird eingespeist) oder blockiert das **aufsitzende**
Fahrzeug den Kanal? V2 lässt es aufsitzen, V1 ließ es 16 mm schweben — Gegentest über
`CFD_Z_OFFSET_MM=16`, ein Lauf, keine Codeänderung.

---

## Block C — Wandmodell, nach dem Entwurfsfehler neu aufzusetzen
> **Status 2026-08-15:** C1-Regelkreis blieb widerlegt; stattdessen WFB (Han et al., WANDMODELL.md)
> + **C1b Stufen 0–2 ERLEDIGT** (FACETTEN-*.md). C2 portiert, C3 = Facetten-Stufe 1, C4 durch
> Spalding (statt Reichardt) erledigt, C6 = Kugel-Arme gelaufen. C5/C7 bleiben als Merkposten
> für Stufe 5 gültig.

**C1. Den Kopplungsmechanismus neu entwerfen.** Vollständige Begründung in `WANDMODELL.md`. Die Gegenprüfung hat den Regelkreis widerlegt:
`update_force_field` misst `2·Σ c_i f_i^in`, die Ladd-Korrektur ist **nicht enthalten**. Der wahre
Übertrag ist F_wahr = F_gemessen − u_w/3 — der Regler sieht seinen eigenen Eingriff nicht und läuft
in einen Offset in Höhe des ganzen Effekts. **Das ist V1s Fehlerklasse mit umgekehrtem Vorzeichen.**
Dazu zu lösen: Vorfaktor schwankt über Wandorientierungen um Faktor 3,5, an konvexen Kanten
entsteht wandnormale Transpiration; Reglerzeitkonstante ist vier Größenordnungen zu schnell
(braucht Zeitfilterung über 10²–10³ Schritte); keine Klemme auf u_w.

**C1b. Facetten-basierte Wandbehandlung — ZELLBASIERT (Heiko, 2026-08-15).**
Wandfunktion auf lokalen Facetten rechnen und geglättet zurückverteilen (≙ Surfel-Ansatz von
PowerFLOW). **Entscheidung: aus den Zellen, nicht aus der STL** — die Wandfunktion soll die Wand
modellieren, die der Löser hat (Halfway-Voxelwand), nicht die des CAD; die STL läge systematisch
bis dx/2 daneben, und im dd-Fall bekommt so jedes Gitter automatisch seine eigene effektive Wand.
Umsetzung: **Ausgleichsebene über Wandzellen der 3³/5³-Nachbarschaft** (die Verallgemeinerung von
Heikos Dreiecken — gemittelte Normale ohne Kollinearitäts-Sonderfälle, löst strukturell V1s
Matyka-Rauschproblem), u_w flächengewichtet über Nachbarfacetten geglättet. Kanten/dünne Teile:
schlecht konditionierte Fits markieren und zählen, nicht still mitrechnen. STL nur als einmalige
Diagnose-Gegenprobe der Normalen. Heilt den GEOMETRISCHEN Rauheitsanteil; den nicht-geometrischen
(Kanal ist gitterparallel und trotzdem rau) klärt Test B.

**C2. Werkzeuge zuerst** (Phase 0): `CFD_DUMP_CL`/`CFD_DUMP_DEFINES` aus V1 portieren (~20 Zeilen,
V1 nennt es „das Werkzeug gegen die teuerste Fehlerklasse des Projekts"), und
`env_on`/`env_f`/`env_u` nach `utilities.hpp` heben.
*Ohne beides ist kein Identitätsnachweis führbar.*

**C3. Phase 1: reine Diagnose, schreibt nichts.** Zellliste, Wandabstand und Normale aus der STL,
Kernel rechnet τ_Modell, τ_gemessen, ν_t/ν, y⁺, u⁺ — **kein Eingriff, kein TYPE_MS.**
Beantwortet vor dem ersten Eingriff: welches Vorzeichen hat u_w? (Ohne SGS unterliefert
Bounce-Back um Faktor 8, mit Smagorinsky überliefert es womöglich um 38 — jede Formulierung, die
das Vorzeichen annimmt, ist ein Glücksspiel.)

**C4. Die Reichardt-Lösung festschreiben:** Log-Log-Newton mit Startwert √Y, drei Schritte
(1,8·10⁻¹³). Das lineare Residuum **divergiert** (4·10¹³⁸ bzw. NaN), und `-cl-finite-math-only`
macht NaN zu undefiniertem Verhalten.

**C5. Die Fallen, die vor dem ersten Lauf sitzen müssen:**
- **TYPE_MS wird am Fahrzeug nie gesetzt** (Bedingung ist `u != 0`, Fahrzeugzellen haben u = 0) →
  ohne Markierung ist das Modell ein lautloser No-op. Aber: unbedingt markieren, sonst testet der
  GAIN=0-Kontrollarm nichts.
- **`object_force` vergleicht auf exakte Gleichheit** (`flags[n]==0x41`) — jedes Zusatz-Flag auf
  Fahrzeugzellen löscht sie lautlos aus Cd und Cz.
- Zweite Aufrufstelle von `apply_moving_boundaries` in `update_fields` mitdenken.
- Halo-Austausch von u_w im Zwei-Domänen-Fall.

**C6. Phase 2 am Kugelfall**, dann Phase 3 am Fahrzeug, drei Arme (AUS / GAIN=0 / AN).

**C7. Schriftlich VOR dem ersten Fahrzeuglauf festhalten:** Eine aufgeprägte Wandschubspannung ist
eine **Senke**, kein wandwärtiger Impulstransport. Sie kann den Ablösepunkt unter Druckanstieg
nicht nach hinten schieben. Wer mehr erwartet, erklärt hinterher ein korrekt arbeitendes Modell
für gescheitert.

---

## Block D — numerische Ursachen statt Symptome

**D1. Volumenviskosität für den GERADEN Sektor.** Die Dichtemode sitzt dort, Λ steuert den
ungeraden — deshalb hat kein Λ-Wert geholfen (3/16, 1/4, 9,1·10⁻⁸ alle gemessen). ω_ν bleibt bei
1,99997, ω_e/ω_ε auf ~1,0–1,2. Kostet **keine Reynoldszahl**.
Der Beleg steht in V2s eigener `defines.hpp`: bei τ = 0,8 stabil, bei τ = 0,50003 divergent, gleiche
Geometrie, einziger Unterschied τ.

**D2. RHO_CLAMP über lange Läufe beobachten.** Der Zähler ist gebaut und meldet im kurzen dd-Lauf
**null Treffer**. Über 0,2 s ist das ungeprüft. Bleibt er null, ist die Klemme ein nie auslösender
Wächter; greift sie dauernd, ist der Lauf kein Ergebnis.

---

## Block E — Hygiene 3 bis 8 (aus `HYGIENE-BEFUNDE.md`)
> **Status 2026-08-15: E4–E7 und E9 ERLEDIGT** (Nachweise in AUDIT-BEFUNDE.md R1/R2 + Code);
> Reststatus je Punkt in HYGIENE-BEFUNDE.md.

**E1. VOLUME_FORCE-Nullkraft** — läuft in allen vier Fällen umsonst mit; im **Kugelfall ist keine
F-Box gesetzt**, also 12 B/Zelle über die volle Domäne, bis ~10 % Leistung. Erst Wächter für den
F-Pfad (die „Kraft ist 0"-Warnung ist bei aktivem FORCE_FIELD unterdrückt), dann A/B.
*Nach A1 erledigt sich ein Teil von selbst — der Kanal braucht VOLUME_FORCE wirklich.*

**E2. Schalter-Inkonsistenzen:** `CFD_REG_BC` und `CFD_PO_HART` prüfen nur das Literal `"0"`,
`=off`/`=false`/`=""` schalten **ein** — und das bei Kontrollarm-Schaltern. Behoben durch C2.

**E3. Zwei neue Sparse-Landminen aus V1 geerbt?** `CFD_SPARSE_TILES` hat dort **zwei Semantiken**
(im Fahrzeugpfad schaltet `=0` es *ein*), und im Kugelpfad liefe es gegen einen
1-Zellen-Platzhalterpuffer. In V2 prüfen.

**E4. `CFD_NU`-Defaults uneinheitlich** (fahrzeug 1,48e-5, dd/fernfeld 1,51e-5) — ein A/B ohne
gesetztes CFD_NU vergleicht still zwei Viskositäten.

**E5. Zwei Kommentar/Code-Widersprüche:** der `defines.hpp`-Kopf verkauft REG noch als Lösung
(gemessen verschlechtert es); der SPONGE-Kommentar verspricht TYPE_E-Flächen, rampt aber am
Domänenrand.

**E6. Zwei Tote:** `sparse_n_active` (write-only), `fx_c` (befüllt, nie gelesen).

**E7. Zwei latente Fallen:** Kugelfall-Warnpfad endet mit `return` statt `_exit(0)` und läuft in den
dokumentierten Intel-Teardown-Crash; fehlendes `else` in der Fallauswahl.

**E8. Fünffach duplizierte Setup-Hilfslogik** zusammenlegen (Sparse-Env 3×, F-Box 3×,
Kontaktflächen-Übergabe 2×, `n_cells` 2×, Statistik-Schluss 3×).

**E9. Die 1580 vi∩po-Doppelzellen dokumentieren** — deterministisch über die Enqueue-Reihenfolge,
aber nirgends aufgeschrieben. Wer die zwei Zeilen tauscht, lässt den Einlass die Auslasskante prägen.

---

## Block F — Werkzeuge und Kleinigkeiten

**F1. `HEARTBEAT_DIAG` aus V1 portieren.** Dumpt alle 5 s Threadzustand und schickt nach 15 s im **→ ERLEDIGT 2026-08-15 (portiert, main.cpp).**
R-Zustand ein Signal für einen Backtrace. Hätte den 25-Minuten-Hänger von heute in 15 Sekunden
zugeordnet.

**F2. Sparse Tiling: Domänenmaße auf Vielfache von T bringen.** Holt **2–3 Prozentpunkte** ohne
jeden Code-Eingriff (V1s Produktionsbox hatte 1,5 % Padding, unsere ~3,3 %). Derselbe Handgriff wie
das Schnappen auf ganze grobe Zellen.

**F3. Kugel gegen die Standard-Widerstandskurve** — zweite externe Referenz, und die einzige
Geometrie mit Literaturwert. Dabei gleich die F-Box im Kugelfall setzen (E1).

**F4. F-BBox Multi-Domain-Absicherung** (P2/U1: `Memory_Container` rechnet mit voller Domänengröße,
das darunterliegende `Memory` ist bbox-groß).

---

## Was heute erledigt wurde (nicht mehr offen)

Doppel-Domäne läuft wieder (SRT statt TRT + RHO_CLAMP, 0,20 s statt Absturz bei 0,15 s) ·
Dämpfungszone gemessen und pro Domäne schaltbar · Wand-Audit neu gebaut mit Impulsterm und
Wirksamkeitsnachweis durch den Kernel · Lauf-Sicherung mit jedem Lauf (19 Quelldateien, Git-Stand,
Umgebung, Diff) · RHO_CLAMP-Zähler · Λ als Laufzeitparameter · Kopplungsmaße auf ganze grobe Zellen
geschnappt (auflösungsunabhängig) · Geräteaufteilung gemessen · Hygiene 1 und 2 samt Validierung.
