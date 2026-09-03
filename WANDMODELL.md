# Wandmodell und Kanalvalidierung — Wissensstand

Stand 2026-08-09. Zwei Planungen, beide von unabhängigen Agenten **bis auf Codetiefe gegengeprüft**.
Beide hatten tragende Fehler, die vor der ersten Codezeile gefunden wurden. Dieses Dokument hält
fest, was gilt — nicht, was geplant war.

---

## Warum überhaupt ein Wandmodell

Erste Fluidzelle bei **y⁺ ≈ 137** (Nahfeld, dx = 4 mm), im Fernfeld ~547. Halfway-Bounce-Back setzt
voraus, dass diese Zelle die viskose Unterschicht auflöst (y⁺ < 5) — sie verfehlt das um Faktor 137.
Wandaufgelöst wäre 29 µm statt 4 mm nötig, also ~10⁶-mal mehr Zellen. Die Wandschubspannung wird
dadurch rechnerisch um **Faktor ~4,7** unterschätzt.

**★ ABER: y⁺ = 137 ist nie gemessen worden.** Es ist eine Plattenkorrelation bei x = L.
`update_force_field` liefert die echte τ_w-Verteilung — **ein einziges Sample eines vorhandenen
Laufs** ergibt das reale y⁺-Histogramm und ersetzt die ganze Korrelationskette. Das gehört gemacht,
bevor eine große Validierung darauf aufgebaut wird.

**Der indirekte Hebel ist der wichtige:** Reibungswiderstand ist beim PKW nur ~10 % von Cd
(Connolly et al. 2024: „approximately 90 % coming from the pressure component"). 10 % τ_w-Fehler
sind ~1 % Cd. Entscheidend ist der Weg über den **Ablösepunkt** — Larsson et al. 2016, §3.2:
„the cumulative loss of near-wall momentum in the upstream boundary layer determines the point of
separation". Zu wenig τ_w ⇒ Grenzschicht zu energiereich ⇒ Ablösung zu spät ⇒ Cd zu klein.
Eine publizierte Transferfunktion „X % τ_w → Y % Cd" existiert **nicht**.

---

## Die Auslegung, und was die Gegenprüfung davon übrig lässt

Empfohlen war: **Reichardt + Slip-Velocity-Bounce-Back + Signed-Distance-Feld + Abtastung 2 Zellen
von der Wand + kein van Driest.** Die Ankopplung über eine effektive Viskosität scheidet aus
(Ponsin & Lozano 2025, Phys. Fluids 37, 095101: Slip-Velocity-BB ist für komplexe Geometrie der
robusteste Weg).

### ★★ Der Regelkreis ist WIDERLEGT — der zentrale Entwurfsfehler

Vorgeschlagen war, auf die **gemessene** Impulsaustauschkraft zu regeln, weil Doppelzählung dann
strukturell unmöglich sei. **Das stimmt nicht.**

`update_force_field` (`src/kernel.cpp`) misst `2·Σ c_i f_i^in` — der eigene Kommentar sagt
*„2x because fi are reflected on solid boundary cells (bounced-back)"*. **Die Ladd-Korrektur ist
nicht enthalten.** Der wahre Übertrag ist

> **F_wahr = F_gemessen − u_w/3**

Der Regler sieht seinen eigenen Eingriff nicht und läuft in einen Offset **in Höhe des gesamten
Wandmodell-Effekts**. Das ist V1s Fehlerklasse (modelliertes τ_BB subtrahieren) mit umgekehrtem
Vorzeichen.

### Weitere Befunde der Gegenprüfung

| | |
|---|---|
| **Der Vorfaktor 1/3 gilt nur für die ebene achsparallele Wand** | Gemessen über Wandorientierungen: eben 0,333 · 45°-Treppe 0,667 · Raumdiagonale 0,500 · konvexe Kante 1,167 — **Faktor 3,5 Schwankung**. An Kanten zusätzlich ein **wandnormaler** Eintrag (Transpiration), den kein Tangentialregler wegregeln kann. Auf einer voxelisierten Karosserie sind Treppen und Kanten die Mehrheit |
| **Die Newton-Form ist unterspezifiziert** | „Newton in ln(y⁺)" mit **linearem** Residuum divergiert: 4·10¹³⁸ bzw. NaN nach drei Schritten. Nur **Log-Log-Residuum mit Startwert √Y** liefert 1,8·10⁻¹³. Und `-cl-finite-math-only` macht NaN zu undefiniertem Verhalten |
| **Der GAIN=0-Kontrollarm testet nichts** | Bei u_w = 0 ist die Markierbedingung `u != 0` nicht erfüllt → kein TYPE_MS → der Pfad läuft gar nicht. Er muss **unbedingt** markieren |
| **Reglerzeitkonstante 4 Größenordnungen zu schnell** | Er würde auf momentane Turbulenzfluktuationen einrasten statt auf den Mittelwert. Braucht Zeitfilterung über 10²–10³ Schritte |
| **Keine Klemme auf u_w** | Weder im Kernel noch vorgesehen. Ein divergierender Regler schreibt beliebig große Wandgeschwindigkeiten |

### Die Falle, ohne die das Modell ein lautloser No-op wäre

**Fahrzeug-Solidzellen haben u = 0.** `initialize` und `update_moving_boundaries` setzen TYPE_MS
**nur**, wenn ein solider Nachbar `u != 0` hat. Also ist am Fahrzeug **kein TYPE_MS gesetzt**, und
`apply_moving_boundaries` läuft dort **nie**. Ohne eine unbedingte Markierung hätten wir exakt V1s
teuerste Fehlerklasse reproduziert.

Und: **`object_force` vergleicht auf exakte Gleichheit** (`flags[n]==0x41`). Jedes zusätzliche
Flag-Bit auf Fahrzeug-Solidzellen löscht sie **lautlos** aus Cd und Cz. Markierung gehört auf die
Fluidseite oder in einen eigenen Puffer.

### Erfreulich: die Ankopplung selbst ist winzig

`apply_moving_boundaries` **ist** bereits der Ladd/Krüger-Slip-Velocity-Bounce-Back und liest die
Wandgeschwindigkeit aus dem gewöhnlichen `u`-Feld am **Solid**-Index. Der Eingriff besteht darin,
u_w dort hineinzuschreiben — **keine Änderung an `stream_collide`**. Zweite Aufrufstelle in
`update_fields` mitdenken.

### Smagorinsky

ν_t/ν ≈ **290** in der Wandzelle (mit Wandabstand 0,5 und einseitigem Gradienten; unter
Log-Gesetz-Annahme 40). Smagorinsky ist dort **O(y⁰)** — die Wirbelviskosität verschwindet an der
Wand nicht, WALE und σ wären O(y³).

**Aber die naheliegende Folgerung ist falsch:** τ_eff bleibt selbst bei ν_t/ν = 290 bei **0,508**.
Die Wandzelle ist auch mit „dominanter" Wirbelviskosität praktisch reibungsfrei. **Die Wandspannung
wird vom Bounce-Back-Impulsaustausch getragen, nicht vom SGS-Modell** — also von genau dem Term,
der oben falsch gemessen wird.

Zhou & Bae (JCP 507, 112948): bei identischem Wandmodell variiert die Ablöseblase über verschiedene
SGS-Modelle von ~0 bis über 0,35 L — die Streuung übersteigt den DNS-Wert selbst, und Smagorinsky
sagt auf mittlerem Gitter **gar keine Ablösung** voraus.

---

## Der turbulente Kanal als externe Referenz

### Was die Gegenprüfung korrigiert

**Das Argument für Re_τ = 5186 ist zirkulär.** y⁺₁ und δ/dx sind **keine zwei Bedingungen**:

> y⁺₁ · (2δ/dx) = [u_τ·(dx/2)/ν] · (2δ/dx) = u_τ·δ/ν = **Re_τ**

137 · 2 · 16,7 = **4576** — das *ist* Re_τ. Bei Re_τ = 5186 trifft man nur **eines** von beiden:
y⁺₁ = 137 ⟹ δ/dx = 18,9, oder δ/dx = 16,7 ⟹ y⁺₁ = 155.

**Und die −60 % gehören zu N = 34, nicht zu y⁺₁ = 137.** Beim geplanten N = 38 sind es **−69 %**.

### Der Ausgangspunkt ohne Wandmodell — Herleitung bestätigt

Aus (ν+ν_t)·dU/dy = u_τ²(1−y/δ) mit ν_t = (C_sΔ)²|dU/dy| folgt U_b⁺ = **1,1543·N** und

> **c_f = 1,502/N² — jede Halbierung von dx viertelt das c_f**

Konstante unabhängig reproduziert: 0,76421222/(18√2) = 0,030021, C_s = 0,1733.

**★ Die wichtigste Folgerung, die der Plan selbst nicht gezogen hat:** es gibt genau **eine**
Gitterweite, bei der es zufällig stimmt — **N = 21, δ/dx = 10,5, y⁺₁ = 246**. Am Fahrzeug entspricht
das **dx ≈ 6,4 mm**. Das eigene Modell sagt also, dass unser 4-mm-Produktionsgitter auf der
*falschen* Seite liegt und ein **gröberes** zufällig richtig wäre. Ein Fahrzeuglauf bei 6 mm ist
**billiger als der aktuelle** und prüft das direkt am Zielobjekt — ohne Kanal, ohne Wandmodell.

**Vorbehalt:** die Herleitung setzt **null aufgelöste Reynoldsspannung** voraus. Sie ist eine
**Schranke, keine Prognose**; publizierte c_f-Fehler bei diesen Auflösungen liegen bei 10–30 %.

### Weitere Korrekturen

- Der Geschwindigkeits-Clamp sitzt bei **Ma = 1,0** (`def_c = 1/√3 = c_s`), nicht darunter.
  Ma_bulk ist 0,865, nicht 0,91. Er greift über die **Kanalmitte** schon ab N ≈ 107.
  Die operative Schranke ist die **Kompressibilität**, nicht der Clamp.
- **CFR ist nicht zwingend** und teurer als gedacht: N = 152 geht von 2,13 h auf **14,19 h**.
- τ_eff ist **0,5016** (47× ν durch Smagorinsky), nicht 0,50003. Der freie Knopf ist **Λ, nicht τ**.
- **van Driest leistet bei y⁺ = 137 genau 1,0 %.** Der geplante dritte Arm ist am Zielpunkt
  empirisch widerlegt. Der informative dritte Arm wäre **SUBGRID ganz aus**.
- **Kein Checkpoint/Restart im Code** — ein Lauf über 2,5 h ist nicht teilbar.

### Der Wandversatz, gemessen statt argumentiert

Ein Prüfer hat D2Q9-TRT-Poiseuille in doppelter Genauigkeit nachgebaut:

| | Wandversatz |
|---|---|
| Λ = 3/16 | **0,00000** (Theorem bestätigt) |
| Λ = 0,09 (SRT bei τ = 0,8) | −0,0068 Zellen |
| Λ = 1,09·10⁻⁹ (SRT bei τ = 0,50003) | −0,0130 Zellen |

Gesetz: u_slip = (2/3)(3/16−Λ)·|u''|. Für die laminare Parabel ist der Versatz harmlos —
**für ein logarithmisches Profil aber ~0,25 Zellen = 50 % des Wandabstands der ersten Zelle.**
y⁺₁ wäre effektiv ~205 statt 137: ein systematischer Bias **direkt in der gemessenen Größe**.
SRT gegen TRT muss vor dem ersten Lauf entschieden werden; der Kontrollarm kostet 60 s
(`CFD_LAMBDA` existiert bereits).

Umgekehrt: **der Kanal kann das Λ-Problem des Fahrzeugs nicht reproduzieren** — dort ist es akut
wegen der ein- bis nullzelligen Spalte an den Reifenaufstandsflächen (Fehler O(1)), der Kanal ist
19 Zellen breit (Fehler O(1/19)).

### Die haltbare Leiter (2,5-h-Grenze, beide Durchsätze)

| N | δ/dx | y⁺₁ | Zellen | B70 @4648 | @2400 |
|---|---|---|---|---|---|
| **38** | 19 | **137** | 286 k | 31 s | 60 s |
| 54 | 27 | 96 | 809 k | 2,1 min | 4,0 min |
| 76 | 38 | 68 | 2,22 M | 8,1 min | 15,6 min |
| 108 | 54 | 48 | 6,34 M | 33 min | 63 min |

N = 152 verletzt die Grenze bei 2400 MLUP/s; N = 216 und 304 immer. **Alle Arme zusammen unter 4 h.**

### Referenzdaten (URLs verifiziert, Daten heruntergeladen und ausgewertet)

**Lee & Moser 2015**, JFM 774, 395 — `turbulence.oden.utexas.edu/channel2015/data/`,
Re_τ = 5186 / 1995 / 1000 / 543 / 182. Daraus selbst berechnet: **U_b⁺ = 24,104, c_f = 3,4424·10⁻³**
bei Re_τ = 5186. Hoyas & Jiménez 2006 ist **entbehrlich** (Lee & Moser deckt Re_τ = 1995 ab).

### Was der Kanal nicht kann

Er ist eine **Gleichgewichts-Grenzschicht ohne Druckgradient**. Er validiert, dass das Modell die
richtige τ_w aufprägt und gitterunabhängig arbeitet — **nichts** über Nichtgleichgewicht,
Druckanstieg, Ablösung. Und er testet **nicht** den Zweig, der am Auto das Problem ist:
Wandnormale und Wandabstand auf einer voxelisierten STL, und das Verhalten in abgelösten Zonen.

**Schriftlich VOR dem ersten Fahrzeuglauf:** Eine aufgeprägte Wandschubspannung ist eine **Senke**,
kein wandwärtiger Impulstransport. Sie kann den Ablösepunkt unter Druckanstieg **nicht** nach hinten
schieben — das gilt für Slip-BB genauso wie für V1s Körperkraft. Wer mehr erwartet, erklärt
hinterher ein korrekt arbeitendes Modell für gescheitert.

---

## V1s Wandmodell — was es war und warum es nicht ankam

Es war **kein Xue/Lu**, sondern eine **Spalding-Inversion mit Werner-Wengle-Startwert** (Formel und
analytische Ableitung von einem Prüfer nachgerechnet und korrekt).

| | |
|---|---|
| Ankopplung | **Body-Force-Senke, additiv zur ohnehin wirkenden Bounce-Back-Reibung** → Doppelzählung; `WM_STRESS_CORRECT` war das Pflaster dagegen und machte die Kraft ≈ 0. In 7 von 14 Fahrzeugskripten aktiv |
| Abtastpunkt | 10 mm = **1,13·δ — außerhalb der Grenzschicht.** Es tastete die Außenströmung ab |
| Über der Fahrbahn | **τ_w ≡ 0** — die Normalen kamen aus Device-Flags, in denen der Boden nicht stand |
| Das Feld | `UPDATE_FIELDS` in V1 **nie definiert** → die Kraft rechnete auf einem bis zu **99 Schritte alten** Feld |
| Gemessene Wirkung am Fahrzeug | **keine.** Ein abgebrochener visueller Vergleich, und eine später für ungültig erklärte τ_w-Messung |

V1 wusste das teilweise selbst: in `wall-models.md` steht die Rechnung, dass die beiden
Kawai-Larsson-Bedingungen auf diesen Gittern **um Faktor ~17 unvereinbar** sind.

---

# Nachtrag 2026-08-15 — der Kanal hat gemessen, und das Bild ist ein anderes

## Die Befundkette: fünf Hypothesen, vier tot

Der Kanal (A1) lief als Leiter N = 38/54/76/108 plus Gegentests, alle mit validierter
Messvorrichtung (zwei unabhängige c_f-Wege, Übereinstimmung 1,4 % am Hauptlauf):

| Hypothese | Test | Ergebnis |
|---|---|---|
| Mischweg/ungedämpftes Smagorinsky: c_f ∝ 1/N², **−69 %** | Leiter | **tot** — gemessen +217 % bis +121 %, nur schwach fallend |
| Reynolds-Effekt | Re_τ 5186 gegen 1000, gleiches Gitter | **tot** — c_f identisch (0,01092 / 0,01093) |
| SRT-Instabilitäts-Rauschboden | U_b unter die Schwelle halbiert | **tot** — c_f fällt nur 9 % |
| FP16C-Quantisierung | FP32-Build, gleicher Lauf | **tot** — −3 % |
| **Raue Wand (vollraues Regime)** | k_s-Konsistenz über die Leiter | **trägt** |

## Die Arbeitshypothese: Bounce-Back als hydraulisch raue Wand

Aus U_b⁺ = (1/κ)·ln(δ/k_s) + 8,5 (Nikuradse, vollrau) rückgerechnet:

| N | c_f | k_s [Zellen] |
|---|---:|---:|
| 38 | 0,01092 | 2,41 |
| 54 | 0,01121 | 3,69 (Ausreißer — dieser Lauf hat auch 20 % c_f-Wege-Diskrepanz, vermutlich nicht konvergiert) |
| 76 | 0,00772 | 1,68 |
| 108 | 0,00761 | 2,29 |
| 38, U_b halbiert | 0,00995 | 1,85 |

**k_s ≈ 2 ± 1 Zellen, näherungsweise konstant** — und das vollraue Regime erklärt alle drei
Signaturen auf einmal: c_f Re-unabhängig, c_f U_b-unabhängig, nur logarithmisch in N.

Physikalische Deutung (unbewiesen, aber konsistent): unter SRT ist die effektive
Bounce-Back-Wandlage viskositätsabhängig (Λ = (τ−½)² statt 3/16), und Smagorinsky moduliert τ
lokal — die Wandposition **flackert** räumlich und zeitlich um O(Zellen). Eine flackernde
Wandposition *ist* Rauheit.

## Was das für das Wandmodell heißt

Die Aufgabe ist präziser als „fehlende Reibung ergänzen": **die Rauwand-Charakteristik durch die
korrekte Glattwand-Spannung ersetzen.** Und es gibt erstmals eine quantitative Basislinie mit
Vorhersagekraft — c_f folgt dem vollrauen Gesetz mit k_s ≈ 2 Zellen; jede Verbesserung ist gegen
diese Kurve messbar.

Konsistent damit: die y⁺-Messung am Fahrzeug (Median 1122 statt korrelierter 137, τ_w ~130× über
der Glattwand-Erwartung) — eine raue Wand liefert genau solche Überschüsse.

**Status: Arbeitshypothese nach fünf Gegentests, unabhängige Prüfung (Iron Rule 2) angestoßen.**

## Unabhängige Prüfung der Rauwand-These (Iron Rule 2) — Ergebnis

**Haltbar in präzisierter Form.** Fit gegen sechs Alternativmodelle: vollrau gewinnt mit
AICc-Abstand ≥ 19; die freie Log-Steigung (2,62 ± 0,30) trifft 1/κ = 2,44; der Re-1000-Punkt ist
ein bestandener Falsifikationsversuch (U_b⁺ ändert sich um 0,12, „glatt mit verschobenem B" sagte
+4,4). Referenz korrekt angewandt: Re_b aller Hauptläufe = 249 950–250 034 gegen Lee&Moser 250 047.

**Drei Korrekturen an meiner Fassung:**
1. **k_s ≈ 1 Zelle, nicht 2** — die Konstante B = 8,5 war bulk-inkonsistent; korrekt integriert
   (B = 6,53) ist k_s = 1,02 [0,95; 1,12], und das N=108-Profil bestätigt unabhängig 0,93 Zellen
   (B_eff = −3,42 statt glatt +5,2). Sprich: **vollrau, k_s = O(1 dx)**.
2. **Der Mechanismus ist KEINE Treppe** — die Kanalwände sind gitterparallel. Kandidaten:
   Bounce-Back ohne Wandmodell und **ungedämpftes Smagorinsky bis in die Wandzelle** (ν_t ∝ (CΔ)²|S|
   skaliert mit der Zellgröße — genau die beobachtete k_s ∝ dx-Signatur). Meine
   „flackernde Wandlage"-Deutung ist damit unbelegt.
3. Zitatkorrektur: JCP 429 (2021) 109995 ist **Cai, Degrigny, Boussuge & Sagaut**, nicht Capizzano.

**Direkter Literaturbeleg für den Befund:** Han, Ooka & Kikumoto 2021 (Fluid Dyn. Res. 53, 045506):
Standard-Bounce-Back auf groben Gittern unterschätzt die mittlere Geschwindigkeit im ganzen Feld —
exakt unsere Signatur (U_b⁺ 13,6–16,5 statt 24,1) — und wird durch eine **Spalding-Wandfunktion im
Bounce-Back** repariert.

**Offen:** n54 passt nicht (nicht monoton in N, zugleich 9 % c_f-Wege-Diskrepanz — vermutlich nicht
konvergiert); das transitionale Regime ist ungetestet (läuft als Test A); und der
Mechanismus-Entscheider ist **Test B: ν_t = 0 in der ersten Fluidlage** — fällt c_f deutlich, ist
die „Rauheit" das SGS-Modell, nicht der Bounce-Back. Das bestimmt die Stellschraube des Wandmodells.

## Nachtrag 2026-08-15 abends — WFB gebaut, Rauwand beseitigt, nächste Schicht frei

**Umsetzung nach Plan** (Free-Slip-Tausch + ½τ_w-Abzug auf den vier Diagonalen, Spalding Log-Log-
Newton, Schalter `CFD_WANDFUNKTION` 1=voll/2=nur Tausch, nur im Kanal verdrahtet). Alle Nachweise
grün: Kontrollarm reproduziert Basislinie, Nur-Tausch-Arm liefert c_f = 0 (Free-Slip wie
konstruiert), Wirkpfad-Zähler exakt, Klemmen 0.

**Ergebnis der Leiter:**

| N | c_f mit WFB | vorher (Rauwand) |
|---|---:|---:|
| 38 | 0,00107 | 0,01062 |
| 54 | 0,00110 | 0,01121 |
| 76 | 0,00109 | 0,00772 |
| 108 | 0,00114 | 0,00761 |

**Flach auf ±4 % — die Gitterunabhängigkeit ist erreicht, die Rauwand beseitigt.** Aber flach bei
−68 % (Referenz 0,00344), und die Vorhersage „c_f steigt mit N" ist **widerlegt** (auch kein
Mischweg-1/N²). Deutung: selbstkonsistenter turbulenzfreier Zustand — ohne aufgelöste Turbulenz
transportiert nichts Impuls zur Wand, die Wandfunktion sieht zu langsames u_t und liefert die dazu
passende, zu kleine Spannung. **Das Restproblem liegt im Inneren** (Turbulenz-Selbsterhaltung bei
y⁺₁ = 137–500; Han et al. liefen bei 16–50, wo das Gitter Wirbel trägt).

**Nächste Schritte:** (1) WFB um **Relativgeschwindigkeit** erweitern — die mitbewegte Fahrbahn ist
ein gitterparalleler z-Wand-Fall, der tote Bodenstreifen damit der erste Fahrzeug-Einsatzort;
(2) C1b zellbasierte Facetten für die Karosserie; (3) Abnahme am Fahrzeug: y⁺-Median 1122 → ~140,
Verteilung schmal, Bodenstreifen-Zeitreihe flach; (4) Kanal-Turbulenzerhaltung als eigene Frage.
Am Fahrzeug erzeugt die Geometrie die Turbulenz selbst — der glatte Kanal ist der Härtefall, nicht
der Normalfall.

---

# Nachtrag 2026-09-03 — Nachbarabtastung (CFD_FAC_NACHBAR) gegen den 3/2-Faktor (CFD_FAC_UTKORR)

Beide Schalter greifen an **derselben Stelle** an: dem Eingang der Spalding-Kette in
`apply_facette_imem` (src/kernel.cpp). Beide lassen Tangentialbasis, Stabilitätsklemme `tw_max`
und die SATGATE-Gates unangetastet — sie ändern nur, **welches (u, y)-Paar** das Modell sieht.
Sie tun aber nicht dasselbe.

## Die Kette, um die es geht

```
Y      = ut_wm * (2*yw_ab) * def_fac_Y     mit def_fac_Y = 0,5/nu   →   Y = u * y / nu
u+     = wf_spalding_uplus(Y)
u_tau  = ut_wm / u+
tau_w  = rho * u_tau²
```

`Y` ist die Spalding-Variable, also die **Reynoldszahl des Abtastpunkts**. In sie geht der
Wandabstand `yw_ab` **linear** ein — genau darin liegt der Unterschied.

## Was jeder Schalter tut

| | `CFD_FAC_UTKORR=1.5` | `CFD_FAC_NACHBAR=1` |
|---|---|---|
| Geschwindigkeit | `ut_wm = 1,5 · u_t(eigene Zelle)` | `ut_wm = u_t(zweite Fluidzelle)` |
| Wandabstand | **bleibt `y_w`** (ebene Wand: 0,5) | wird mitgeführt: `y_w + c_ib·n` (ebene Wand: 1,5) |
| Herkunft der Zahl | **empirisch geeicht** (Theorie 3/2 für die BB-Deflation, gemessen 1/0,70 = 1,43; 1,5 als bester A/B-Wert vom 25.08.) | **gemessen** — das Modell liest das Profil dort, wo es nicht deflatiert ist |
| Ortsabhängigkeit | **konstant**, überall derselbe Faktor | **je Zelle verschieden** (gemessen am 8-mm-Fahrzeug, `w_nb`: Hauptklasse (5, 0,50) ×2,0; Eckklasse (8, 0,20) ×4,4) |
| Determinismus | unkritisch (Skalar) | erforderte den eigenen Kernel `fac_nachbar_ab` (B72) |
| Wirkung, wenn beide gesetzt | wirkt nur noch, wo NACHBAR **nicht** greift (Slots 73/74; 8 mm: 0,1 % der Entscheide) | hat Vorrang |

## Warum beide an der ebenen Wand fast dasselbe liefern

Das ist kein Zufall, sondern die Aussage von Kawai & Larsson (2012): **liegt das Paar (u, y) auf
dem logarithmischen Profil, ist das daraus bestimmte u_tau von der Abtasthöhe unabhängig.**

- `NACHBAR` nimmt ein **echtes** Paar: (u₂, y=1,5). Gemessen (`xb_nb_kipp0_igpu`): u_t_abt 0,0581
  bei y_abt 1,5, während die eigene Zelle 0,0341 bei y 0,5 sieht — Verhältnis **1,70**, das sich
  aus BB-Deflation (≈1,43) und echtem Profilwachstum von y 0,5 auf 1,5 (≈1,19) zusammensetzt.
- `UTKORR` nimmt ein **konstruiertes** Paar: (1,5·u₁, y=0,5). Das liegt nur dann auf dem Profil,
  wenn die Deflation exakt 2/3 beträgt.

Ergebnis am ebenen Kanal N=20: u_tau-Faktor **0,920** (NACHBAR, `xb_nb_kipp0_igpu`, 03.09.) gegen
**0,921** (UTKORR=1,5, Serie g10, 25.08.). Zwei Wege, dieselbe Zahl — die Deflationstheorie
P1 ≈ −u/3 trägt, und NACHBAR ist ihre **physikalische statt empirische** Form.

## Wo sie auseinanderlaufen

An der **Treppe** ist die Deflation nicht mehr 2/3, sondern hängt von der Linkmenge und davon ab,
ob die Zelle im Stufenschatten liegt. Ein globaler Faktor kann das nicht treffen:

**Der Dreipunkt-A/B am 26-Grad-Kanal (Serie `xd_utkorr_kanal`, 03.09., eine Variable je Arm) zeigt es
in einer Zahl** — `tw/Ziel` je Treppenklasse, 1,0 wäre perfekt:

| Klasse (Solid-Links, y_w) | weder | UTKORR=1,5 | NACHBAR | ×UTKORR | ×NACHBAR |
|---|---|---|---|---|---|
| (1, 1,08) | 0,311 | 0,628 | 0,545 | 2,02 | 1,75 |
| (4, 0,71) | 0,199 | 0,399 | 0,476 | 2,00 | 2,39 |
| (8, 0,18) | 0,159 | 0,297 | 0,486 | 1,87 | **3,06** |
| **Spannweite max/min** | **1,96** | **2,11** | **1,14** | | |

`UTKORR` multipliziert alle Klassen mit **demselben** Faktor (2,02 / 2,00 / 1,87) — das Niveau steigt,
die **Verzerrung zwischen den Klassen bleibt** (1,96 → 2,11, sogar minimal schlechter). `NACHBAR`
korrigiert **klassenabhängig** (1,75 / 2,39 / 3,06 — am stärksten dort, wo der Stufenschatten am
tiefsten ist) und bringt die Klassen damit auf ein gemeinsames Niveau: Spannweite **1,96 → 1,14**.
Der globale u_τ-Faktor unterscheidet die drei Arme kaum (1,107 / 1,130 / 1,164) — der Unterschied
steckt nicht im Mittel, sondern in der **Verteilung**. Genau das ist die Aufgabe eines Wandmodells.
- Am 8-mm-Fahrzeug sieht die Eckklasse (8 Solid-Links, y_w 0,20) in der eigenen Zelle u_t = 0,0046,
  in der zweiten 0,0199. Ein Faktor 1,5 träfe dort um **Faktor 2,9** daneben; Spalding macht daraus
  τ ~ u², also rund **Faktor 8** im Ziel.

## Konsequenz für die Basis

Geht NACHBAR in die Basis, ist `CFD_FAC_UTKORR=1,5` an 99,9 % der Zellen wirkungslos und bleibt
nur noch Rückfall für Zellen ohne Fluidnachbarn in Normalenrichtung (Slots 73/74). Der Entscheid
— UTKORR dort auf 1,0 oder auf 1,5 lassen — steht bei Heiko. **Messstand:** B74 in AUDIT-BEFUNDE.md;
der direkte Dreipunkt-A/B (ohne beides / UTKORR 1,5 / NACHBAR) läuft als Serie `xd_utkorr_kanal`.
