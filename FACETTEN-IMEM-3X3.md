# iMEM-Iteration 2: 3x3-Plan (2026-08-16, auf 9688ead)

Alle Pflichtlektüre gelesen (FACETTEN-IMEM.md komplett inkl. Revision/I0/I1/I2-Durchfall, kernel.cpp 1660–1816, FACETTEN-STUFE3.md). Ich habe zusätzlich die Linkmengen-Momente aller Torus-Lagen von Hand durchgerechnet — dabei fällt ein Befund an, der die Erwartung an die I2-Wiederholung wesentlich schärft (Abschnitt 5). Hier der Plan.

---

# iMEM-Iteration 2: 3×3-System mit Normal-Nullung — Festlegung

Formelnummern setzen FACETTEN-IMEM.md (1)–(15) fort. Notation wie im Code: die 6 ist in die Momente eingefaltet, d. h. G_ab = 6Σw(c·t̂_a)(c·t̂_b), Sn_a = 6Σw(c·t̂_a)(c·n̂), Snn = 6Σw(c·n̂)², P_a = 2Σ(c·t̂_a)fhn[i], alles über L.

## 1. System (Auftrag 1)

**(16) Ansatz:** u_s = s₁t̂₁ + s₂t̂₂ + s_n·n̂ (jetzt mit Normalkomponente).

**(17) Normalziel:** Der Gesamt-Normalaustausch ist Φ_n = P_n + (Sn₁s₁ + Sn₂s₂ + Snn·s_n). Ziel: q-Beitrag = 0, also

  Sn₁s₁ + Sn₂s₂ + Snn·s_n = 0  ⇔ Φ_n = P_n = unveränderter BB-Wert.

Wichtig: **P_n muss nie berechnet werden** — die RHS der Normalzeile ist exakt 0, kein Sollwert. Der Cd-Druckpfad (E1, lebt vom unangetasteten BB-Normalaustausch) bleibt damit auf ALLEN Rangstufen geschützt.

**(18) Das 3×3-System** (symmetrisch, Zeilen/Spalten t₁,t₂,n):

```
| G11  G12  Sn1 |   | s1 |   | R1 |      R1 = −def_fac_tau·twe − P1
| G12  G22  Sn2 | · | s2 | = | R2 |      R2 = −P2
| Sn1  Sn2  Snn |   | sn |   |  0 |
```

M = 6·QᵀS₂Q ist positiv semidefinit, Rang M = dim span{c_i, i∈L}. Cauchy-Schwarz über das Maß w liefert Sn_a² ≤ G_aa·Snn — das trägt die gesamte Guard-Logik.

**(19) Lösungsform: Schur-Elimination der n-Zeile (LDL mit n-Pivot), NICHT 3×3-Cramer.** Weil RHS_n = 0, lässt die Elimination die t-RHS **unverändert**:

  G̃_ab = G_ab − Sn_a·Sn_b/Snn;  G̃·(s₁,s₂) = (R₁,R₂);  s_n = −(Sn₁s₁ + Sn₂s₂)/Snn.

Begründung gegen Cramer: (a) der Entkopplungsfall (Sn≈0) fällt **konstruktiv auf das heutige 2×2** zurück (bitgleicher Ausdrucksbaum möglich, s. u.); (b) die Rang-Kaskade wird zur G̃-Kaskade mit den heutigen Schwellenformen — keine neue det₃-Skalendiskussion; (c) die Einzellink-Degeneration wird **manifest** (G̃ ≡ 0 exakt, s. Formel 28) statt in det-Epsilontik versteckt; (d) numerisch sicher: G̃ bleibt PSD, ein float-negatives G̃aa fällt automatisch durch den ≥1e-8-Test (kein UB-Loch der Befund-2-Klasse).

**(20) Entkopplungs-Gate** (VOR der Schur-Reduktion):

  entkoppelt ⇔ Snn < 1e-8  ODER  Sn₁² + Sn₂² ≤ εc²·Snn·(G11+G22),  εc = 1e-3.

Das Verhältnis KOP/(Snn·(G11+G22)) ist per Cauchy-Schwarz ein reiner Kopplungswinkel ∈[0,1]. Schwellenherleitung: Rauschboden der Sn-Summen ~30 ulp der O(0,2)-Summanden ≈ 1e-6 absolut; kleinste physische Kopplung am Torus |Sn₂|(m1) = 1/6 ≈ 0,167 (Rechnung unten). εc=1e-3 gibt |Sn|-Schwelle ~1e-4..1e-3: ≥2 Dekaden über Rauschen, ≥2 unter Physik — trennscharf. **Entkoppelt → wörtlich der heutige 2×2-Pfad (textuell identischer Code inkl. Slot-12/13-Kaskade), s_n = 0.**

## 2. Beweise Entkopplung (Auftrag 1 + 4, von Hand gerechnet)

**(25) Ebene Wand, L = {5,9,16,11,18}, n̂ = ẑ:** S₂ = diag(2,2,6)/36. n̂ ist **exakter Eigenvektor** → Sn_a = 6t̂_a·S₂n̂ = (6·6/36)(t̂_a·n̂) = 0 für **jede** Tangentialrichtung, nicht nur t̂ = x̂. → s_n = 0, t-Zeilen unverändert, I1-Äquivalenz konstruktiv. **Float-Status (geprüft am Ausdrucksbaum):** Für exakt achsparalleles t̂₁ ist die Auslöschung bitexakt 0 (Paare ±r6w heben sich, utz = uzn − und·nz = 0 exakt). Für fluktuierendes t̂ ist sie es NICHT — die fma-Schleife (Linkreihenfolge 5,9,11,16,18) trennt die Paare (9,16)/(11,18), Rest O(2⁻²⁴) ≈ 1e-7. **Deshalb Gate (20) statt „exakt 0"**: das Gate fängt das Rauschen, der 2×2-Pfad läuft bitgleich. Ohne Gate wäre Bitgleichheit am Kanal unerreichbar.

**(26) 45° Lage-0, L = {4,5,18,8,13,16,9}, n̂ = (0,−1,1)/√2:** S₂ = [[4,0,0],[0,5,−1],[0,−1,5]]/36. n̂ ∝ (0,−1,1) ist Eigenvektor (Eigenwert 6/36) → **Sn₁ = Sn₂ = 0 exakt, für JEDES tangentiale t̂** (auch hangauf-verdrehte). G = diag(2/3, 2/3), det = 4/9, Snn = 1, S₁ = (0,−5,5)/36 ∝ n̂. → Vollrang-2×2, entkoppelt, Δm = 0. Der gemessene [5]-Wert −1,285 über 528k Schritte ist damit als **Float-Rauschen identifiziert** (Prognose bestätigt die Messung: ~600× kleiner als 26,6°). **Lage 1 = {18}: c ∥ n̂ exakt (ct₁²+ct₂² = |c|²−cn² = 2−2 = 0) → G11 = G22 = 0, KOP = 0 → entkoppelt → Slot 13 wie bisher, nie eine Modifikation.**

**(27) 26,6°, n̂ = (0,−1,2)/√5, t̂₂⁰ = (0,2,1)/√5:**
- **m0 (8 Links = 45°-Menge ∪ {11}):** S₂ = diag(4,6,6)/36 — yz-isotrop → n̂ Eigenvektor → **Sn ≡ 0 für alle t̂, m0 ist entkoppelt.** G11 = 2/3, G22 = 1, Snn = 1. Robust gegen die F8-Normalenverkippung: die Verkippung liegt in der yz-Ebene (n_x = 0 per x-Symmetrie der Wolke), und dort ist S₂ ∝ I — Entkopplung bleibt exakt.
- **m1 (4 Links {5,18,16,9}):** S₂ = [[2,0,0],[0,1,−1],[0,−1,5]]/36, S₁ = (0,−1,5)/36 ∦ n̂. **Sn₁(x̂) = 0, Sn₂ = 1/6, Snn = 5/6, G11 = 1/3, G22 = 1/6.** span L = R³ → Schur-Vollrang: G̃11 = 1/3, G̃22 = 1/6 − (1/36)/(5/6) = 2/15, det̃ = 2/45 ≫ Schwelle. s_n = −0,2·s₂ (bei t̂₁ = x̂). **Hier saß die gemessene Kontamination** (über den Fluktuationskanal s₂ und verkippte Momentan-t̂) — die 3×3 nullt sie exakt.
- **m2 (1 Link {18}):** ct₁ = 0 (x̂), ct₂ = −1/√5, cn = 3/√5. Sn₂ = −1/10, Snn = 3/10, G22 = 1/30. Gate: KOP = 1e-2 gegen εc²-Schwelle 1e-8 → gekoppelt. **G̃22 = 1/30 − (1/100)/(3/10) = 0 EXAKT** — die Einzellink-Wahrheit wird manifest → Rang-0-Pfad (s. u.), keine Modifikation. Der 2×2-Quer-Skalar-Zweig hatte hier s₂ = R₂/G22 mit Sn₂s₂-Injektion gefahren — der zweite gemessene Kontaminator, entfällt.

## 3. Rang-Kaskade und Einzellink-Entscheid (Auftrag 2)

**(21) Kaskade auf G̃** (nach Gate; Schwellenformen wie heute, Befund-2/3-Struktur erhalten):
1. det̃ = G̃11·G̃22 − G̃12² ≥ 1e-4·G̃11·G̃22 ∧ G̃11 ≥ 1e-8 ∧ G̃22 ≥ 1e-8 → **Rang 3**: Cramer auf G̃, dann s_n.
2. sonst G̃11 ≥ 1e-8 → **Rang 2**: s₁ = R₁/G̃11, s₂ = 0, s_n = −Sn₁s₁/Snn. Neuer Zähler [14].
3. sonst G̃22 ≥ 1e-8 → **Rang 2 Spiegel**: s₂ = R₂/G̃22, s₁ = 0, s_n = −Sn₂s₂/Snn. Zähler [14].
4. sonst → **Rang 0 gekoppelt: KEINE Modifikation, BB belassen**, neuer Zähler [15].

**(22) Rang-2-Entscheid (die Auftragsfrage):** Zweig 2 IST exakt das 2×2 in (t₁,n) — Nachweis: [[G11,Sn1],[Sn1,Snn]]·(s₁,s_n) = (R₁,0) hat per Elimination genau s₁ = R₁/(G11−Sn₁²/Snn) = R₁/G̃11, s_n = −Sn₁s₁/Snn. **Prioritätsordnung: 1. Normal-Nullung (fällt NIE, solange Kopplung existiert), 2. τ-Ziel t₁, 3. Quer-Nullung t₂.** Begründung aus dem Messbefund: das bisherige (t₁,t₂) erfüllte Tangentialziele unter freier Normalinjektion — exakt der gemessene Durchfallmodus. Das Opfer der Quer-Nullung (s₂=0) lässt nur den BB-Quer-Widerstand stehen — harmlos bis physikalisch (die BB-Basislinie zeigt, dass die Treppen-Querblockade kein Störterm ist), während jede Normalinjektion die gemessene Sekundärströmungsquelle ist.

**(28) Einzellink-Rechnung und Entscheid (Optionen a/b/c):**
- *45° Lage-0 (7 Links):* entkoppelt, Vollrang — alle Optionen identisch, keine Entscheidung nötig (s_n = 0, Lösung = heutiges 2×2). *45° Lage-1 {18}:* c ∥ n̂ → alle Optionen kollabieren auf „nichts tun".
- *Kugel-typischer Diagonalfall* (n̂ ≈ ẑ, L = {9} = (1,0,1), w = 1/36): G11 = Snn = Sn₁ = 1/6 — Cauchy-Schwarz mit Gleichheit, G̃11 = 0 exakt.
  - **Option (c)** (Status quo): s₁ = R₁/G11 erfüllt das τ-Ziel voll, injiziert aber Sn₁s₁ = R₁ — **Normalinjektion in voller Höhe des Tangentialziels (100 %)**, jede Zelle jeden Schritt ∝ twe: verteiltes Blasen/Saugen über die Kugel, korrumpiert zusätzlich den Cd-Druckpfad. Gemessen schlecht (Torus-m2 ist derselbe Mechanismus).
  - **Option (a)** (Nullraum-Projektion): erreichbarer Austausch = span{c}; normalfreier Unterraum davon = {0} sobald c_n ≠ 0 → **Beitrag exakt 0** — Option (a) IST Option (b), wenn c_n ≠ 0. (Kompromiss-LS auf das volle Ziel, λ = c·R/|c|²: halbe Zielerfüllung, halbe Kontamination — bleibt systematische Normalquelle ∝ twe, verworfen.)
  - **Entscheid: (a)≡(b) — BB belassen, zählen ([15]), Census-Soll.** Begründung: (i) der Messbefund macht Normalinjektion zur Durchfallursache erster Ordnung; (ii) der Cd-Druckpfad bleibt exakt; (iii) Torus-Konsistenz: m2 fällt auf BB zurück — genau wie beim Paartausch (F9: m2 „keins"), die 26,6°-Schließung lebt nachweislich auf m0+m1, und Link 18 trägt ohnehin kein c_x → kein Schließungsverlust; (iv) die Abdeckungssorge an der Kugel wird VOR dem Lauf quantifiziert: Census-Kennzahl „Anteil |L|=1 mit c_n≠0" (A5-Erweiterung). Erwartung: kleiner einstelliger Prozentbereich (die 313 war die paarbasierte, viel restriktivere Zählung; linkbasiert sind Einzellink-Zellen die Randminderheit). Liegt der Census-Anteil unerwartet hoch (>10 %), Eskalationsdiskussion mit Zahl — Kompromiss-LS bleibt als dokumentierte Schubladenoption, wird nicht still eingebaut.

## 4. Klemmen, Slots, Akkumulatoren (Auftrag 3)

**(23) Klemm-Reihenfolge (wichtig):** lösen → s₁,s₂ klemmen (±2ut/±ut wie bisher, Slot 10) → **s_n aus den GEKLEMMTEN s₁,s₂ neu berechnen** (Normal-Nullung hält exakt auch bei tangentialer Klemme!) → s_n klemmen auf **±ut**, Treffer in neuen Slot [16]. Skalenherleitung: |s_n| ≤ (|Sn₁||s₁|+|Sn₂||s₂|)/Snn ≤ √(G/Snn)·|s|; typisch am Torus |s_n| ≤ 0,2|s₂| ≪ ut; groß wird s_n nur bei schwacher Normalautorität (Snn klein bei signifikanter Kopplung) — genau dort ist eine Transpiration ≳ ut unphysikalisch. ±ut ist Sicherheitsnetz mit Headroom, kein Regler; enger als ±2ut, weil Normalinjektion gefährlicher ist als Tangentialrest. Jede s_n-Klemme macht den Rest in [5] sichtbar.

**Slots (14→17):** [10] s₁/s₂-Klemme (unverändert), [12] entkoppelter Skalar-Fallback (unverändert), [13] kein tangential wirksamer Link (unverändert — 45°-Lage-1-Sollformel 2NxNy·⌈n/100⌉ bleibt wörtlich, Kopplung an Slot 9=0 wie Auflage 4), **NEU [14]** gekoppelter Rang-2-Pfad, **NEU [15]** gekoppelt Rang 0 → BB belassen (26,6°-Soll: NxNy·⌈n/100⌉, alle m2-Zellen beider Wände; per Census bestätigen), **NEU [16]** s_n-Klemme. grep-Pflicht auf alle Slot-Leser/Legenden (lbm.hpp:158, setup.cpp-Reports).

**(24) Akkumulatoren (Layout 6 float bleibt):**
- **[5] misst jetzt den REST:** += Sn₁s₁ + Sn₂s₂ + Snn·s_n mit den finalen (geklemmten) Werten. Vollrang/Rang-2 ungeklemmt → analytisch 0 (float: ulp-Rest); Rang 0 → 0; entkoppelt → alte Formel (identisch, da s_n=0). **Harte Torus-Prüfung: |Σ[5]| ≤ 5 je Lauf** (45°-Rauschreferenz 1,285; Faktor >150 unter dem Durchfallwert 763,6) **UND Slot [16] = 0** (sonst misst [5] Klemmreste statt Mechanik).
- **[1..3] Ist-Kraft MUSS die Sn-Terme tragen:** φ_a = P_a + G_a1s₁ + G_a2s₂ + Sn_a·s_n — sonst misst der gekoppelte Pfad falsch (stiller Diagnosefehler). Ungeklemmt bleibt φ₁ = −def_fac_tau·twe, φ₂ = 0 die Wirkpfad-Identität.
- **[4] Δm** = 6(S₁·u_s) über die u_s-Komponenten — Formel unverändert korrekt, erfasst automatisch den neuen s_n-Beitrag (an m1 ändert sich Δm, gleiches Band).

## 5. Erwartung I2-Wiederholung und Abnahme NEU (Auftrag 4)

Die Lagenrechnung (26)/(27) liefert eine scharfe, unbequeme Prognose:

- **26,6°:** m0 entkoppelt-sauber, m1 exakt genullt (Vollrang), m2 → BB. Die Kontamination −763,6 fällt konstruktiv auf das Rauschband. **c_f fällt deutlich von 0,0287/0,0315** — die Winkelanomalie (16× vs 3×) verschwindet.
- **45°: die 3×3 ist dort BEWEISBAR ein No-Op** (Lage-0 entkoppelt für jedes t̂, Lage-1 inaktiv). Der 45°-Durchfall (Arm 4 = 0,0040 > BB-Basis 0,00166 bei [5] ≈ 0!) hat also eine **andere, noch unerklärte Ursache** — und die ist regimeverdächtig: Arm 4 erzwingt (ungeklemmt) exakt null mittleren Tangentialaustausch pro Zelle und Schritt, trotzdem hielt der CFR-Regler f auf c_f = 0,0040. Entweder brachen Klemmen/Skips die Erzwingung (Slot 10, [1..3]-Summen — in den vorhandenen Logs nachlesbar!), oder das nicht-selbsterhaltene Regime der Basislinie (0,00166/0,00179, F11 widerlegt) verschiebt die Messung. **Ohne Klärung wiederholt sich der 45°-Durchfall trotz korrekter 3×3.** Realistische Erwartung: 26,6° konvergiert auf das 45°-Niveau; das F12-Band wird nur erreicht, wenn die 45°-Restursache Mechanik (behebbar) und nicht Regime ist.

**Abnahmekriterien neu (zweistufig):**
- **N1 Mechanik (hart, regimeunabhängig):** |Σ[5]| ≤ 5 je Lauf und Winkel; Slot [16] = 0; Slot-Solls exakt ([13]-45°-Formel, [15]-26,6°-Formel, an Slot 9=0 gekoppelt); Δm im bisherigen Band; Arm 4: Σ|[1..3]| rel. < 1e-3 gegen den Tangentialumsatz.
- **N2 Ordnung (hart, regimeunabhängig, NEU):** c_f(Arm 4) ≤ c_f(BB-Basis) am selben Winkel — Free-Slip darf nie über BB liegen. Dieses Kriterium stellt den 45°-Widerspruch scharf, egal in welchem Regime.
- **N3 Band (bedingt):** F12-Band [0,00096; 0,00126] bleibt das Ziel, gilt aber nur bei Regime-Gleichheit (U_b⁺ im Parallel-Referenzregime). Fällt das Regime heraus, wird das Band NICHT still nachgezogen — Befund dokumentieren; die Regime-Frage (nicht-selbsterhaltener gekippter Kanal, Basis 0,0017) ist die −68-%-Baustelle und ein eigenes Arbeitspaket, kein iMEM-Kriterium.

## 6. Stufenplan (Auftrag 5)

- **J0 — Log-Nachauswertung 45° (VOR jeder Codezeile, rein lesend, kostenlos):** aus den vorhandenen I2-Logs/CSV: Slot-10-Klemmquote, Slots 12/13, Σ[1..3] des Arm-4-Laufs, Reibung_x aus kraft_facetten gegen f·V. Entscheidet Mechanik- vs Regime-Ursache des 45°-Rätsels und damit, ob N3 bei der Wiederholung überhaupt erreichbar ist.
- **J1 — T3a/T3b-Erweiterung (CPU, double-Referenz):** Referenzfälle mit den Zahlen aus (25)–(28): (a) ebene Wand 5-Link → Gate entkoppelt, bitgleich zur 2×2-Referenz; (b) 45°-Lage-0 → Momente G=diag(2/3,2/3), Snn=1, Sn=0, entkoppelt; (c) 26,6°-m1 → gekoppelt, Vollrang, gegen double-3×3 rel < 1e-6, Probe s_n = −0,2s₂, Normal-Null exakt; (d) m2-{18} → G̃ = 0 exakt, Rang-0-Pfad; (e) Kugel-Diagonal {9} → Rang 0 (Entscheidungsbeleg); (f) 2-Link-Konstruktion (z. B. {5,9}) → Rang-2-(t₁,n)-Pfad, Normal-Null exakt, s₂=0; (g) Klemmpfade: s₁-Klemme mit s_n-Neuberechnung → [5]-Rest = 0; s_n-Klemme → [5] ≠ 0 und [16] zählt. T3b: Δm-Identität mit s_n.
- **J2 — Kanal-0°-Regression:** **Kriterium: JA, bitgleich** — konstruktiv erreichbar unter zwei Bedingungen am Ausdrucksbaum: (i) der entkoppelte Zweig ist der **textuell heutige Code** (nicht s_n=0 durch neue Ausdrücke geschleust — der ±0-Randfall in usx = …+s_n·nx und die neue [5]-Formel würden Bits gefährden); (ii) die neuen Schleifenakkus (Snn, Gate-Größen) hängen als eigene fma-Ketten HINTER den bestehenden, keine Umordnung. Ablauf: iMEM-2×2-CPU-Hash (N=38/316) VOR dem Umbau festhalten → nach Umbau exakt gleich; zusätzlich Kontrollarm-Hash 12755646098055097704 unverändert. Falls der Compiler trotz expliziter fma nicht bitgleich reproduziert (Registerdruck): dokumentiert auf das statistische I1-Band zurückfallen.
- **J3 — Torus-Wiederholung, 26,6° ZUERST** (dort wirkt die 3×3; 45° ist prognostiziert bit-invariant und läuft als Bestätigung + J0-Nachtest). Je Winkel Arm 4 → Arm 3; Abnahme N1/N2 hart, N3 bedingt; Census-Solls vorab ([15]-Formel).
- **J4 — Kugel:** Census zuerst (|L|-Verteilung, Anteil gekoppelter Rang-0-Zellen = Preis des Entscheids (28), Kopplungswinkel-Histogramm); Arme AUS/4/3; beide Cd-Wege (Druckpfad jetzt konstruktiv geschützt — der Nachweis ist neu); [5]-Summe, Klemmquoten [10]/[16], FP32-Sprosse (FP16C-Risiko erbt auf den s_n-Pfad).

## 7. Risiken

1. **45°-Restursache unbekannt** — die 3×3 ändert dort beweisbar nichts; ohne J0 droht identischer Durchfall mit korrekter Mechanik. Größtes Einzelrisiko, deshalb J0 vor Code.
2. **Rang-0-Entscheid kostet Kugel-Abdeckung** — Census quantifiziert vorab; Eskalation (Kompromiss-LS) nur als dokumentierte Schublade bei Census-Anteil > 10 %.
3. **Bitgleichheitsanspruch J2** hängt am textuellen Altpfad und der fma-Ordnung; Rückfallebene definiert.
4. **s_n-Klemme vs [5]-Härtetest:** Klemmereignisse im Transienten könnten [16]=0 reißen — Kriterium auf das eingeschwungene Fenster beziehen bzw. [5]-Band als Primärkriterium.
5. **Stille Leser:** Slots 14→17, [5]-Semantikwechsel („Kontamination"→„Rest"), [1..3]-Sn-Terme — grep-Pflicht und Legenden, sonst Diagnose-Drift.
6. **Schwellen-Randzone des Gates** (Kopplungswinkel nahe εc): Zellen könnten zwischen Pfaden flackern — an den Torus-Lagen unmöglich (0 vs 0,45 Kopplungswinkel), an der Kugel per Census-Histogramm prüfen.

### Critical Files for Implementation
- /home/heiko/CFD/FluidX3D-v2/src/kernel.cpp (apply_facette_imem 1746–1815: Momentenschleife, Kaskade, Klemmen, Akkumulator — Kern des Umbaus)
- /home/heiko/CFD/FluidX3D-v2/src/setup.cpp (baue_facetten-Census A5-Erweiterung, Torus-/Kugel-Verdrahtung, Reports mit neuen Slot-Solls, facetten_test T3a/T3b)
- /home/heiko/CFD/FluidX3D-v2/src/lbm.hpp (Slot-Legende, Zähler 14→17, Schalter)
- /home/heiko/CFD/FluidX3D-v2/src/lbm.cpp (Zählerpuffer-Größe, alloc_facetten_domain, Guards)
- /home/heiko/CFD/FluidX3D-v2/FACETTEN-IMEM.md (Formeln 16–28, Entscheide, N1–N3, J0–J4 nachtragen — Doku-Drift-Pflicht vor dem ersten Messlauf)

---

# J0-Ergebnis (2026-08-16): 45-Grad-Ursache = MECHANIK, Fix bekannt

Arm-4-Log 45 Grad: u_s-Klemme 5.763.359 (~8 % der modifizierten Zellen je Messpunkt),
Skalar-Fallback 2.618.880, Slot 13 = 72,0 Mio (Lage-1 wie erwartet); Ist-Reibung 0,226 bei
Soll 0,270 (Verhaeltnis 0,837) -- das Nullziel wird an den hochbelasteten Zellen von der
Klemme gebrochen, und die instantane Inversion jagt den turbulenten P-Fluktuationen hinterher:
u_s oszilliert mit +-2ut und pumpt selbst Fluktuationen (daher Arm 4 UEBER BB-Basis).
Normalkontamination +5713 ueber 528k Schritte = Float-Rauschen (Sn=0-Beweis haelt; groesser
als Arm 3, weil |u_s| im Nullziel-Arm groesser ist).

**Konsequenz: Iteration 2 braucht BEIDE Bausteine:**
1. **3x3-Normal-Nullung** (dieser Plan) -- heilt den 26,6-Grad-Mechanismus (m1/m2-Kopplung).
2. **Zeitliche EMA-Filterung von u_s** (Asmuth Gl. 29/30; entspricht unserem A6-Plan
   "EMA 10^2-10^3 Schritte", der ausgelassen wurde) -- heilt das 45-Grad-Fluktuationsjagen.
   Umsetzung: Puffer fac_us (3 float je Facette, xyz-Rahmen -- frame-stabil gegen rotierendes
   t-Basis), u_s_angewandt = (1-a)*u_s_alt + a*u_s_geloest, a = CFD_FAC_EMA (Emission-gated:
   ungesetzt = exakt heutiger Pfad, Bitgleichheit der bestehenden Arme bleibt).
