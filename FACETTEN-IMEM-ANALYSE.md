# iMEM-Weggabelungs-Analyse (2026-08-16, auf ce690a0)

Alle Pflichtlektüre gelesen (FACETTEN-IMEM-3X3.md, FACETTEN-IMEM.md komplett, kernel.cpp 1740–2102, WANDMODELL.md), Asmuth-Volltext (17 Seiten PDF) im Original gelesen, und die sechs J3-CSVs ausgewertet. Hier der Bericht.

---

# Analyse iMEM-Weggabelung (Stand ce690a0)

## 1. Asmuth-Exegese: Gl. 29/30 filtern den EINGANG, nie die Lösung

**Gl. 29 (Schumann-Grötzbach):** τ̃_w(x,y,t) = ⟨τ_w⟩ · ũ(x,y,z₁,t)/⟨u(z₁)⟩ — mittlere Wandschubspannung aus planar gemittelter Geschwindigkeit, lokale Momentanabweichung linear in der lokal aufgelösten ũ. **Gl. 30 (Yang et al., zeitlich):** u_wm(x,y,t) = (1−ε)·u_wm(x,y,t−Δt) + ε·ũ(x,y,z₁,t), ε = Δt/T_f. Wörtlich: „we replace the planar averaging in Eq. (29) with a temporal averaging using Eq. (30)".

**u_wm ist NICHT die aufgeprägte Wandgeschwindigkeit u_w, sondern die gefilterte Abtastgeschwindigkeit** — der Eingang in MOST (Gl. 14). Kopplung in F^f: gefiltertes ũ → MOST → τ_w → F^wm = τ_w·A₁ (Gl. 27) → **F^{u_w} = F^wm − F^f mit INSTANTANEM F^f** → u_w jeden Schritt frisch (Gl. 28). Die Gesamtkraft pro Schritt ist damit exakt das glatte F^wm — **null Durchlass von F^f-Fluktuationen, und u_w selbst wird nie gefiltert.** Der J0-Schluss „Fix = Asmuths EMA auf u_s" war eine Fehllektüre; die Messung (0,0053→0,0065) hat das unabhängig bestätigt.

**Warum sein instantanes Setup trotzdem funktioniert** (Ranking nach Textlage):
1. **Keine Klemme.** Sein u*_wm-PDF (Fig. 3, Tab. I) schwankt mit σ/u* = 0,13–0,17, Flatness ~3 — ±50-%-Ausschläge laufen frei durch. Kein Rektifizierer.
2. **Volle 9-Link-Γ am ebenen Boden, D3Q27:** Gl. 28 ist diagonal (3,3,1), keine Rang-Kaskade, keine m2-BB-Reste, Normale exakt entkoppelt — unsere 3×3-Frage stellt sich bei ihm nie („the explicit form of u_w can differ … in the case of complex geometries" bleibt Randsatz).
3. **AMD-SGS statt fneq-Smagorinsky:** sein ν_t kommt aus Gradienten der aufgelösten Geschwindigkeit (Gl. 12/13, zweite-Ordnung-Cumulants/finite Differenzen) — **nicht aus den Registern, die er selbst modifiziert.** Unser Rückkopplungspfad (unten) existiert bei ihm strukturell nicht. Dazu Cumulant-Kollision mit parametrierter Höherer-Ordnung-Relaxation.
4. **Selbsterhaltenes, vollturbulentes Regime** mit endlichem, Schumann-artig mitfluktuierendem Ziel — u_w ist eine kleine Korrektur, kein Nullzwang gegen O(1)-Fluktuationen.
5. **Abtastpunkt:** z_el = z₁ (IL) funktioniert bei ihm bereits; z_el = 2Δz (ESG, Kawai-Larsson) ist sein **stärkster Hebel — aber gegen den Log-Layer-Mismatch**, nicht gegen Austauschfluktuationen: „sampling locations above the first off-wall node are found to be an effective measure to reduce the occurring log-layer mismatch"; dagegen „only little effect of the filtering applied to the input velocity". **Der Abtastpunkt rettet ihn also nicht — Punkte 1–4 tun es.** Für unser Arm-4-Problem ist er sogar irrelevant (Ziel 0 hängt an keinem Abtastwert), für Arm 3/Kugel bleibt er der LLM-Hebel.

## 2. Mechanik mit Zahlen: EIN Rückkopplungskreis, nicht zwei Hypothesen

Neu aus den CSVs (Mittel 2. Laufhälfte, `export/j*/kanal_zeit.csv`, Spalte cf_kraftbilanz):

| Lauf | cf (Mittel±σ) | Befund |
|---|---|---|
| j45_a4n (Nullziel, ohne EMA) | 0,00475 ± 0,00026 | 2,9× BB-Basis 0,00166 |
| j45_a4e (Nullziel + EMA 0,01) | 0,00669 ± 0,00048 | EMA +41 % schlechter |
| j45_a3e (τ-Ziel + EMA) | 0,00657 ± 0,00067 | **== a4e innerhalb σ** |
| j26_a4 (Nullziel) | 0,01249 ± 0,00080 | 7× BB-Basis 0,00179 |
| j26_a3 (τ-Ziel) | 0,01250 ± 0,00087 | **== a4 auf 0,1 %!** |
| j26_a3e (τ-Ziel + EMA) | 0,01168 ± 0,00072 | EMA ~6 % |

**Der schärfste neue Befund: Arm 3 und Arm 4 sind cf-identisch** (26,6°: 0,01250 vs 0,01249; 45°+EMA: 0,00657 vs 0,00669). Das aufgeprägte Ziel (Spalding-τ vs exakt Null) ist im Messwert **komplett unsichtbar** — der Restmechanismus dominiert das Signal zu 100 %. Der Torus misst derzeit ausschließlich den Fehler, nicht das Modell.

**Klemm-Quantitativ** (J0-Zahlen, Iteration 1; J3-Konsolen-Slotreports wurden nicht archiviert — export enthält nur LAUF.txt+CSVs): 5.763.359 Klemmtreffer / 5280 Messpunkte ≈ 1091 Zellen/Messpunkt ≈ 8 % der ~13,6k modifizierten Zellen; Ist-Reibung 0,837·f·V trotz Nullziel. **Reicht das allein?** Nein — ungeklemmt ist der Austausch pro Schritt exakt 0, also muss bei 45° der GESAMTE cf 0,00475 durch die 8 %-Klemmereignisse fließen (Lage-1-Zellen {18} tragen kein Tangential-c). Der rektifizierte Rest je Ereignis ist ≤ |P| des Ereignisses; damit die 8-%-Schwanzereignisse 2,9× den BB-Mittelwert tragen, muss das Fluktuationsniveau weit ÜBER dem der stillen BB-Basis liegen. **Die Fluktuationen erzeugt der Mechanismus selbst:**

**Subgrid-Kopplung strukturell bestätigt:** `stream_collide` ruft `apply_facette_imem` (kernel.cpp:1907), der die q_i in fhn addiert (1842); der Smagorinsky-Block (2031–2060) rechnet DANACH `fneqi = fhn[i]-feq[i]` (2052) **auf denselben modifizierten Registern**. Die q_i leben nur auf der Teilmenge L, sind also kein reiner Gleichgewichtsshift — der Rest landet direkt in H → Q → ν_t. Größenordnung: s oszilliert ±2ut ≈ ±0,14, ΔH ~ G·s ~ 0,3·s; ν_t-Spitzen O(10⁻³) gegen ν ≈ 1,1·10⁻⁵ — **Faktor 100+ Viskositätspulse in der Wandlage, jeden Schritt, im Takt der P-Fluktuationen.** Erhöhtes ν_t diffundiert Impuls zur Wand → P wächst → mehr Klemmereignisse → mehr Rektifikation → Drag. Klemm-Rektifizierer und Turbulenzpumpe sind **zwei Stufen einer Schleife**. Der EMA-Arm bestätigt das Bild von der anderen Seite: glattes s tötet die ±2ut-Oszillation, öffnet aber den Durchlass P′(t) als Austausch an JEDER Zelle JEDEN Schritt — gemessen 41 % schlimmer.

## 3. Wegbewertung

**(A) Zielseiten-Filterung — JA, in der beidseitigen Form.** Herleitung der Austauschbilanz: P̄ = EMA[P] (xyz-Rahmen, wie fac_us), ū = EMA[u_t], Ziel T = −def_fac_tau·twe(ū)·t̂(ū); löse G·s = T − P̄ (3×3-Normal-Nullung unverändert). Austausch pro Schritt:
  Φ(t) = P(t) + G·s = **T + P′(t)**,  P′ = P − P̄.
Mittel: ⟨Φ⟩ = T (EMA im stationären Zustand erwartungstreu) — Ziel exakt im Mittel. Fluktuation: Φ′ = P′ = **der natürliche BB-Fluktuationsaustausch** — genau die Schumann-Klasse (Asmuths Gl.-29-Philosophie: Mittel vom Modell, Momentanabweichung von der aufgelösten Strömung). s wird glatt und mittelwert-skalig → Klemmquote →0 (Rektifizierer aus), q_i quasi-stationär (ν_t-Pumpe aus). **Das IST die konsistente Formulierung** — Asmuths per-Schritt-Erzwingung (Φ ≡ F^wm exakt, kein P′-Durchlass) ist noch strenger, setzt aber ungeklemmte Vollrang-Inversion voraus, die bei unseren Linkmengen/Regime nachweislich selbstzerstörerisch ist. Ehrlicher Restposten: P′-Korrelationsarbeit an der Strömung bleibt (ein Spiegel wäre auch fluktuationsfrei — den geben asymmetrische Linkmengen nicht her; genau deshalb ist der Paartausch am parallelen Kanal robust: er ERSETZT den BB linkweise, ohne Kompensation, Klemme oder fneq-Injektion).
- **(A2) Abtastpunkt anheben:** für Arm 4 wirkungslos (Ziel 0 braucht keinen Abtastwert), für Arm 3/Kugel richtig (Asmuths LLM-Hebel). **Aber Vorsicht: nicht racefrei.** lbm.cpp:150–155 (Audit-Befund 8): `stream_collide` SCHREIBT u[] unter UPDATE_FIELDS im selben Kernellauf — der Nachbar liefert nichtdeterministisch t−1 oder t. Braucht Doppelpuffer oder dokumentierten Determinismus-Verzicht. Vormerken, nicht jetzt.

**(B) Klemmbudget — verwerfen als eigenständiger Weg.** Weiter: mehr Injektionsamplitude; enger: mehr Rest. Unter (A) wird die Klemme zum nie feuernden Wächter (und genau das ist das Messkriterium).

**(C) Richterwechsel Kugel — richtig, aber als ZWEITER Schritt.** Ehrliche Antwort auf die Kernfrage: **das F12-Band kann der Torus im nicht-selbsterhaltenen Regime auch mit korrekter Mechanik vermutlich nie liefern** — das Band entstammt dem Parallel-Referenzregime, und die gekippte BB-Basis (0,00166, unter glatt-turbulent) hat ein anderes Fluktuationsniveau; die z-WFB konnte es am parallelen Kanal, weil der Tausch den BB vollständig ersetzt (kein Kompensationsmechanismus, dessen Güte vom Regime abhängt). **Aber: N2 (Arm 4 ≤ BB-Basis) ist regimeunabhängig, derzeit 3–7× gerissen, und der Torus ist das billigste Gerät, das diesen Mechanikfehler messen kann.** Erst N2 am Torus bestehen, dann zur Kugel — sonst reist der Mechanismusfehler ununterscheidbar mit (die Arm-3==Arm-4-Blindheit gilt an der Kugel genauso).

**Empfehlung: A (beidseitige Filterung) am Torus mit N2-Abnahme; N3/F12 offiziell auf die Kugel (J4) verschieben; B streichen; A2 als Arm-3-Baustein hinter J4 einreihen.**

## 4. Nächster Schritt (Ein-Sitzungs-Häppchen)

**Vorab, ohne Codezeile (2×~25 min, iGPU):** j45_a4n und BB-Basis 45° je einmal mit `CFD_SGS_WANDFREI=1` wiederholen (Schalter existiert, Guards ok, Wirkpfad-Slot 6). Fällt cf(Arm 4) deutlich Richtung Basis, ist die ν_t-Pumpe als Verstärker bewiesen — und liefert nebenbei einen Datenpunkt für die −68-%-Baustelle. Dabei diesmal den Konsolen-Slotreport mitschneiden (J3-Klemmquoten fehlen im Archiv).

**Dann die eine Implementierung:** Ziel-EMA statt Lösungs-EMA. Neuer emission-gated Schalter (z. B. `CFD_FAC_PEMA`, a = 0,01; Asmuth-Skala T_f = 10·Δt_c ≈ 140 Schritte passt zu a≈0,007–0,01 — die Zeitkonstante war richtig, die Seite falsch): Puffer 6 float/Facette (P̄-Vektor + ū, beide xyz-Rahmen; Warmstart = erster Messwert statt 0), Lösung gegen P̄, Klemme bleibt als Wächter. Alt-EMA-Pfad unangetastet lassen (A/B-Vergleichbarkeit). T3a-Referenzfall: stationäres P ⇒ bitgleich zum ungefilterten Pfad nach Einschwingen.

**Messpunkt:** 45° Arm 4 + PEMA, ein Lauf (~25 min). Abnahme hart und regimeunabhängig: **cf ≤ 0,00166 (N2)**, Klemmquote Slot 10 ≈ 0, |Σ[5]| ≤ 5, Slot 16 = 0. Sekundär: Arm 3 vs Arm 4 müssen sich endlich TRENNEN (heute 0,1 % Abstand — erst wenn das Ziel im Messwert sichtbar wird, misst der Stand überhaupt ein Wandmodell). Danach 26,6° als Bestätigung, dann J4-Kugel-Census.

**Dateien:** /home/heiko/CFD/FluidX3D-v2/src/kernel.cpp (1746–1851: Momentenschleife/EMA-Block — der PEMA ersetzt Zeilen 1826–1838-Logik an der P-Seite), /home/heiko/CFD/FluidX3D-v2/src/lbm.cpp (764–781 Emission, fac_us-Alloc-Muster), /home/heiko/CFD/FluidX3D-v2/FACETTEN-IMEM-3X3.md (Weggabelungs-Entscheid nachtragen). Asmuth-PDF lokal: /home/heiko/.claude/projects/-home-heiko-CFD-FluidX3D/81cd8076-a5ca-4bf9-9f8a-a928d549933b/tool-results/webfetch-1786875510193-65e3if.pdf (S. 5–6: Gl. 22–30; S. 13: Conclusio zu Filterung vs Abtastpunkt).

---

# PEMA-Messstand + Diskriminator (2026-08-16 abends)

**PEMA Arm 4 (45°, 80 ETT): cf = 0,0062 -- N2 VERLETZT, exakt auf EMA-Niveau** (Auditor-Mathematik
bestaetigt: beide Filterwege teilen den Fluktuationsdurchlass Phi' ~ P'). Kernel-Audit fand
KEINEN Bug, der den Trend erklaert -- die Implementierung entspricht Weg A exakt.

**Diskriminator-Zaehler des Laufs:**
- Normalkontamination 0,0011 (PERFEKT -- die 3x3 haelt auch unter PEMA), Delta-m 199 (im Band).
- **u_s-Klemme = 4,2 Mio (~6 % der Lage-0-Zellen je Messpunkt) -- NICHT ~0**: auch mit
  gefiltertem P verlangt die Loesung an einem Zellsaum |s1|>2ut; die Klemm-Rektifikation von P'
  bleibt ein aktiver Drag-Kanal. Der Diskriminator sagt also: TEILS noch Mechanik (Klemmsaum),
  TEILS Modellgrenze (P'-Korrelationsarbeit, von beiden Filtern geteilt).
- Nebenbefund Gate-Randzone: die Lage-1-Zellen ({18}, c || n) werden vom Kopplungsgate ueber
  Float-Rauschen als "gekoppelt" eingestuft und landen in Slot 15 statt 13 (46,8 Mio vs 27,9 Mio)
  -- verhaltensidentisch (beide BB), aber die Soll-Formeln muessen die Summe 13+15 pruefen.

**Konsequenz -- die Familienfrage steht:** Die iMEM-Kompensationsfamilie hat am Torus-N2 mit
drei Varianten (instantan / EMA / PEMA) denselben strukturellen Rest gezeigt. Vor der
Familien-Entscheidung (Ersetzungs-Ansatz a la PowerFLOW-Gewichtung vs Richterwechsel Kugel)
noch EIN billiger Klemmsaum-Test: Klemmskala als Messarm (z. B. +-4ut) -- faellt cf damit auf
BB-Basis, war es der Saum; bleibt es, ist es die P'-Arbeit und damit Modell.
