# iMEM-Stabilitaetsanalyse (Theorie-Agent, 2026-08-17 nachts, auf a5d25e2)

Alle Belege sind gelesen (kernel.cpp 1746–1929, FACETTEN.md inkl. Nachträge, FACETTEN-IMEM-3X3.md Gl. 16–28, FACETTEN-IMEM.md Gl. 1–15, Defaults `s_fac_tau=1`). Hier die Analyse.

---

# Stabilitätsanalyse der Ein-Schritt-Rückkopplung (iMEM 3×3, gekoppelte Linkmengen)

**Objekt:** `apply_facette_imem` (/home/heiko/CFD/FluidX3D-v2/src/kernel.cpp:1746–1929), Stand a5d25e2. Zellisolierte Ein-Schritt-Betrachtung; Innenflüsse als eingefrorene Anregung (konservative Abschätzung der reinen Wandschleife). ρ≈1, def_fac_tau=1.

## 1. Die Karte und ihre lineare Verstärkung

**(G1)** Momente wie im Code: G_ab=6Σ_L w(c·t̂_a)(c·t̂_b), Sn_a=6Σ_L w(c·t̂_a)(c·n̂), Snn=6Σ_L w(c·n̂)²; Schur G̃_ab=G_ab−Sn_aSn_b/Snn.

**(G2) P-Linearisierung** (Verallgemeinerung von Gl. 10): mit f≈f_eq gilt P_a(u) = −ρ(G_ab u_t,b + Sn_a u_n) + P′_a. P′ = Nichtgleichgewichts-/Fluktuationsanteil (an Krümmung/Sekundärströmung auch mit **stehendem** Offset ≠ 0 — Beleg unten).

**(G3) Angewandter Impuls:** Δ(ρu_t)_a = Σ_L(c·t̂_a)q_i = G_ab s_b + Sn_a s_n = **G̃_ab s_b** (Schur-Identität). Ungeklemmt also Δu_t = T − P **exakt**, mit T=(−τ̃,0), τ̃=def_fac_tau·twe. Die Kopplung ist im ungeklemmten Vollrang-Fall im angewandten Impuls **unsichtbar**.

**(G4) Verstärkung ungeklemmt:** u′ = u + F_int + T(u_t) − P(u). Wand-Jacobian mit T=−τ̃(ut)û_t:
λ∥ = 1 − dτ̃/dut,  λ⊥ = 1 − τ̃/ut, beide ∈ (0,1) — **mild kontraktiv, unabhängig von G̃**. Zum Vergleich BB: λ = 1 − G_aa (stärker dämpfend). iMEM tauscht also BB-Dämpfung gegen exakte Zielerfüllung — marginal, aber stabil.

**(G5) Verstärkung am Klemmanschlag** (s₁ = ∓2ut fest): Δu₁ = P₁(u) + G̃₁₁(∓2ut) ⇒ λ_rail = 1 − G₁₁ − 2G̃₁₁. Wegen G̃₁₁ ≤ G₁₁ ≤ 2/3 (alle realen Teilmengen; Vollmenge über beide Halbräume wäre 2) gilt |λ_rail| ≤ 1 **immer** (Gleichheit λ=−1 nur bei der entkoppelten 45°-7er-Menge, dort klemmt praktisch nie).

**(G6) Lemma:** Die Karte hat **keinen** Eigenwert |λ|>1 — weder ungeklemmt noch am Anschlag. Das Aufschaukeln ist **kein modales, sondern ein Drift-Phänomen**: es braucht eine vorzeichen-definite mittlere Fehlinjektion. Die liefert die Klemme (Abschnitt 2), und die Bedingung „|Verstärkung|>1" wird zur Bedingung „Rektifikationsfluss ≠ 0 und Rückspeisung über die Strömung" — am Anschlag ist dφ₁/dP₁ = 1 (voller Durchgriff der Fluktuation statt Kompensation), d. h. die Schleifenverstärkung über das Fluid ist duty·ζ mit Klemm-Duty und Rückspeiseanteil ζ; sie überschreitet 1, sobald der Anschlag chronisch wird.

## 2. Klemme: Rektifikations-Theorem und Klemm-Reserve

**(G7) Lösungsgröße** (Rang-2/Diagonalfall): s₁/ut = ρ_G − β_eff − p′ mit
ρ_G := G₁₁/G̃₁₁ ≥ 1 (Kopplungsmaß), β_eff := τ̃/(G̃₁₁ut), p′ := P′₁/(G̃₁₁ut).
Der Mittelanteil ρ_G·ut ist der Slip, den das Modell gegen die BB-Überbremsung setzen **muss** — das ±2ut-Budget wurde für ρ_G≈1 dimensioniert.

**(G8) Rektifikation (formal):** φ₁ = P₁ + G̃₁₁s₁ᶜ = T₁ + G̃₁₁(s₁ᶜ − s₁). Am unteren Anschlag ist s₁ᶜ−s₁ = (−2ut−s₁)⁺ ≥ 0, also
 E[φ₁] − T₁ = G̃₁₁·E[(−2ut−s₁)⁺] ≥ 0 — **vorzeichen-definit**, unabhängig vom Vorzeichen der Fluktuation. Zweiseitig-symmetrisches Klemmen hebt sich im Mittel; einseitiges nicht. Einseitigkeit entsteht, sobald der Arbeitspunkt s̄₁ = (ρ_G−β_eff)ut näher an einer Schiene liegt bzw. P̄′ einen stehenden Offset hat. Genau das misst die m1-Diagnose: s₁=−2ut dauerhaft, φ₁=+0,0015 bei Ziel −4e-7 — die Wand reicht P′ durch **plus** einen gleichgerichteten Kick.

**(G9) Klemm-Reserve** = P′-Offset-Toleranz bis zum Anschlag (obere/untere Schiene):
 R↑ = (2G̃₁₁ − G₁₁)·ut, R↓ = (2G̃₁₁ + G₁₁)·ut + τ̃; quer (Budget ±ut): R₂ = G̃₂₂·ut.

| Linkmenge | G₁₁ | G̃₁₁ | ρ_G | min. Reserve/ut |
|---|---|---|---|---|
| ebene Wand (5 Links) | 1/3 | 1/3 | 1,0 | **0,33** |
| 45° Lage 0 (7 Links) | 2/3 | 2/3 | 1,0 | **0,67** |
| 26,6°-m1 statisch, t̂₁=x̂ | 1/3 | 1/3 | 1,0 | 0,33 (t₁), **0,13** (t₂: G̃₂₂=2/15) |
| 26,6°-m1 rotiert θ≈70° | 0,186 | 0,157 | 1,19 | **0,13** |
| Kugel {5,9} (2 Links) | 1/6 | 1/9 | 1,5 | **0,056** |
| Kugel {5,9,11} (3 Links) | 1/6 | 1/8 | 1,33 | **0,083** |

Gekoppelte Mengen verlieren Reserve **doppelt**: ρ_G→2 rückt den Arbeitspunkt an die Schiene, und das kleine G̃ verkleinert die absolute P′-Toleranz. Kugel-2–3-Link-Mengen liegen Faktor **4–12 unter der ebenen Wand** — bei gleichzeitig größerem P′ (Krümmung, Druckgradient). Chronischer, patchweise gleichsinniger Anschlag ist dort der Normalzustand ⇒ kohärentes Blasen/Saugen; dazu an Mengen mit Sn/Snn>1/2 der sn-Klemmrest Φ_n=(2Sn−Snn)ut als gleichgerichteter Normal-Integrator gegen die schwache Rückstellung ~Snn ⇒ Transpiration O(ut) ⇒ RHO_CLAMP, Cd 21. Das erklärt die Kugel-Disqualifikation ohne jedes |λ|>1.

## 3. Kopplung an die Basisrotation (die m1-Ratsche)

**(G10)** t̂₁ folgt û_t instantan. Der rektifizierte Fehlimpuls hat ⊥-Anteile (s₂-Klemme — an m1 der Kanal mit der kleinsten Reserve 0,13ut —, G̃₁₂-Terme, Rang-2-Queropfer): δθ ≈ Δu_⊥/ut, im mitgeführten Rahmen vorzeichen-definit ⇒ θ **driftet monoton** statt zu diffundieren.

**(G11)** Auf m1 gilt G₁₁(θ)=⅓cos²θ+⅙sin²θ, Sn₁(θ)=(1/6)sinθ, Snn=5/6 invariant. Bei θ≈70–80°: G₁₁=0,186→0,17, Sn₁=0,155→0,165 — **reproduziert die Diagnose (0,33→0,19; Sn₁→0,18) quantitativ**. Dabei fällt die Reserve 0,33→0,13ut, der Duty steigt, die Drift beschleunigt: Ratsche. Endzustand der Zeitreihe: ut-Kollaps ⇒ Klemmbudget 2ut→0 ⇒ Modifikation ≈ 0, Austausch = rohes BB-P′ (positiv) — genau der gemessene späte Kipp.

---

# Optionsbewertung

**(b) Ziel-Deckelung twe ≤ β·G̃₁₁·ut.** Herleitung: der Zielanteil der Lösung ist β_eff·ut ≤ β·ut; strukturell klemmt-nie verlangt |ρ_G − β_eff − p′| ≤ 2, also β ≤ 2 (untere Schiene sogar β ≤ ρ_G+2). **Aber:** der Deckel begrenzt **nur den τ̃-Anteil**, und der ist im aufgelösten Regime β_eff = ut/(G̃₁₁·(u⁺)²) ~ 10⁻³ — die Lösung ist P-dominiert. Die eigene m1-Messung beweist es: Ziel 4e-7 gegen P₁≈1,5e-3, Faktor ~4000. Der Deckel griffe nur im tw-Klemm-Regime (ut ≲ ν²/(2y_w²) ~ 10⁻⁸ — praktisch leer). **Prognose (falsifizierbar): Option (b) ändert an m1-Diagnose und Kugel nichts Messbares.** Die dokumentierte Empfehlung „(b) zuerst" adressiert den gemessenen Mechanismus nicht.

**(a) Kopplungs-Gate→BB (geometrisch).** Die basisinvariante Kopplungszahl χ = (Sn₁²+Sn₂²)/(Snn(G₁₁+G₂₂)) ist an m1 **rotationsinvariant 1/15 ≈ 0,067** — jedes praktikable statische Gate (χ* ≳ 0,1) verfehlt m1 konstruktiv; die Kugel-Problemmengen ({5,9}: χ=1/3, {5,9,11}: 1/4) fängt es nur teilweise und kostet pauschal Abdeckung. Als Alleinlösung ungeeignet, als statisches Komplement möglich.

**(c) Basis-Filterung nur gekoppelt.** Bricht die m1-Ratsche (hält Reserve bei 0,33ut), ist aber an der Kugel wirkungslos: dort ist schon die **statische** Reserve 0,056–0,083ut zu klein — kein Rotationsproblem. Zudem zielt gefiltertes t̂ im Einzelschritt am Momentan-u_t vorbei (bekannter PEMA-Preis, Slot-17-Klasse). Notwendig ist es nicht, wenn die Pumpe steht (die Rotation wird von der Rektifikation angetrieben, nicht umgekehrt).

**(d) Anwendungs-Relaxation γ (q bzw. u_s um γ<1).** φ = γT + (1−γ)P ⇒ λ = 1 − γ·dτ̃/dut − (1−γ)G₁₁ ∈ (1−G₁₁, 1): **stabil für jedes γ∈(0,1], es gibt keine Stabilitätsgrenze** — gerade deshalb kauft γ nichts gegen den Drift (der ist nicht modal). Und es ist **keine** Unterrelaxation des Fixpunkts, sondern verschiebt ihn: stationär Wandkraft = γτ̃+(1−γ)G₁₁ū, relativer Fehler (1−γ)(G₁₁ū/τ̃−1) mit G₁₁ū/τ̃ = G₁₁(u⁺)²/… = O(10²) ⇒ schon γ=0,99 verfälscht das Wandgesetz um O(1). Die echte Fixpunkt-Relaxation wäre die Lösungs-EMA — J3-widerlegt. Verworfen.

**Konsequenz aus (G8):** Der einzige vorzeichen-definite Term der ganzen Kette ist die **geklemmte Anwendung**. BB hat ihn nicht; die ungeklemmte iMEM-Lösung hat ihn nicht. Die minimal-invasive Option ist daher keine der drei dokumentierten, sondern:

**(a′) Dynamisches Sättigungs-Gate („Klemme→BB"):** Wenn die ungeklemmte Lösung ihr Budget reißt (|s₁|>2ut, |s₂|>ut oder |s_n|>ut), Modifikation **unterlassen** (BB belassen + zählen) statt geklemmt anwenden. Semantik: *iMEM wirkt nur, wenn es sein Ziel im Budget exakt erreichen kann.* Das entfernt den Rektifikationsterm exakt (Theorem G8: er existiert nur bei geklemmter Anwendung), stellt an kranken Zellen/Schritten die BB-Dämpfung −G₁₁ wieder her, entzieht der θ-Ratsche den Antrieb und ist die konsequente Verallgemeinerung des bestehenden Rang-0-Entscheids (Gl. 28: „jede Erfüllung injizierte Normalimpuls → BB") von der Geometrie auf die Dynamik. Preis: an Wechselzellen alterniert der Austausch zwischen T und P (duty-gewichtetes Mittel, kein Bias); Abdeckungsverlust wird per Slot-Zählung messbar statt als Injektion unsichtbar.

---

# EMPFEHLUNG (eine)

**Klemme durch Rückfall-Gate ersetzen: κ = 1** — jeder Anschlagsfall (bisher Slot 10/16, Kriterium ungeklemmt |s₁|>2ut ∨ |s₂|>ut ∨ |s_n|>ut) ⇒ BB belassen und zählen (Slot-Semantik „Klemme" → „Gate-Rückfall"; Rückfallebene, falls Flickern an Wechselzellen stört: Hysterese κ=2, d. h. klemmen bis 2×Budget, darüber BB). Kein neuer Gleitparameter, emission-gated einführen (eigener Define, ungesetzt = heutiger Pfad — J2-Bitgleichheit und alle Regressionsanker bleiben unberührt).

**Erwarteter Effekt:**
- **m1-Diagnose (Vorher/Nachher-Kriterium):** Verschwinden der Dauer-Anschlag-Episoden (s₁=−2ut-Plateaus → BB-Schritte); im eingeschwungenen Fenster kein persistenter Vorzeichenwechsel von φ₁ mehr (φ₁ ≤ 0 bzw. = P-Rauschen wie in der BB-Basis); G₁₁-Drift ohne Ratschen-Nachschub (kein monotones 0,33→0,19 mehr); [5]-Rest und Δm im Rauschband. 26,6° N2: cf(Arm 4) ≤ BB-Basis 0,00179 (der 1,4×-Rest ist Klemm-Rektifikation und fällt mit ihr).
- **Kugel:** RHO_CLAMP = 0, object_force-Cd von ~21 zurück auf O(1) an der AUS-Referenz 1,32, Cd_druck wieder > 0; Gate-Duty je Zelle als neue Census-Kennzahl (Eskalationsschwelle wie gehabt: Duty-gewichtete Abdeckung < 90 % ⇒ Ersetzungsansatz-Diskussion). Voraussetzung: Kugel-Report um die iMEM-Slots ergänzen (bekannter Audit-Rest), sonst bleibt der Nachweis blind.
- **Nebenbefund zum Protokoll:** Falls parallel ein (b)-A/B läuft — Prognose dieser Analyse ist **kein messbarer Effekt** (Zielanteil ~10⁻³ der Lösung); ein positiver (b)-Befund würde die P-Dominanz-Diagnose falsifizieren und wäre selbst ein Erkenntnisgewinn.

**Konsistenzantwort (Frage 3):** Ebene Wand und 45°-Lage-0 sind im selben Formalismus stabil, weil n̂ dort exakter S₂-Eigenvektor ist (Sn≡0 für **jedes** t̂): ρ_G≡1 rotationsinvariant — die Ratsche hat keinen Hebel — und die Klemm-Reserve beträgt 0,33ut bzw. **0,67ut** (45° hat wegen G=⅔·I die doppelte Marge und obendrein isotropes G, also gar keine θ-Abhängigkeit). Anschläge sind dort selten und zweiseitig ⇒ Rektifikationsmittel ≈ 0, Verstärkung λ∥=1−dτ̃/dut<1. Das deckt sich mit der Messlage (45° Arm 4 pur: N2 bestanden mit 0,000847).
