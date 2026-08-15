# C1b Stufe 3: Gekippter Kanal — Formelblatt (Planungsagent, 2026-08-15 nachts)

> **Einordnung nach der Literatur-Recherche (FACETTEN-LITERATUR.md):** Die Geometrie-, Normierungs-
> und Referenzteile (A, F1-F6, F11-F12, D, E) gelten MECHANISMUS-UNABHÄNGIG — der gekippte Kanal
> ist der Messpunkt sowohl für den Paartausch als auch für einen iMEM-Umbau. Die Gate-Abzählungen
> (F7-F10) und Schritt 6/9 sind paartausch-spezifisch und wären unter iMEM neu zu rechnen.
> Die Morgen-Entscheidung (ARBEITSLISTE.md) geht diesem Plan voraus.

Alle Grundlagen sind gelesen und die Treppengeometrie ist abgezählt (Linkzählung je Lage, Fenster-Punktwolken für y_w, Gate-Auswertung gegen die Paartabelle B). Hier das Ergebnis.

---

# Stufe 3 — Gekippter Kanal (CFD_KANAL_KIPP): Formelblatt + Implementierungsplan

Stand ecbc4e7, alle Zeilenangaben aus dem heutigen Code in /home/heiko/CFD/FluidX3D-v2 gelesen.

## A. Geometrie-Entscheidung (Punkt 1)

**Entschieden: y-periodische schräge Doppelwand über den z-Torus — ein schräger Wandslab, der Kanal schließt über den z-Wrap auf sich selbst.** Der Kernel ist in ALLEN drei Richtungen nativ periodisch (`calculate_indices`, kernel.cpp:969–980 — z wickelt mit `%def_Nz` genauso wie x/y); die bisherigen z-Deckel des Kanals sind nur Flags, keine Randbedingungs-Mechanik. Eine monotone schräge Ebene ist mit gewöhnlicher y-Periodizität unmöglich (am y-Wrap spränge z_wand um tanα·Ny — Klippe); die Alternativen scheiden aus: Shifted-Wrap (y-Wrap mit z-Versatz) wäre ein Eingriff in die Kern-Indexrechnung (Fehlerklasse „stille Inkonsistenz" an ~5 Stellen), ruhende y-Seitenwände kontaminieren mit Seitengrenzschichten, eine Zickzack-Wand (Dreieckswelle) hat Grat-/Tallinien und Sekundärströmungen. Der Torus-Slab dagegen ist **exakt homogen**: jede Wandzelle einer Lage ist translationsäquivalent — die sauberste Isolierung des Geometrieanteils, **null Kernel-Änderung**.

Die „Decke" ist dieselbe schräge Ebene, um Nz−T_v Zellen vertikal versetzt: der eine Slab hat zwei benetzte Seiten; von unten gesehen Boden, über den z-Wrap von oben gesehen Decke. Physikalisch identisch mit einem unendlichen Stapel paralleler gekippter Kanäle.

## B. Formelblatt

Notation: Steigung s = p/q (45°: p=q=1; 26,565°: p=1, q=2), α = atan(p/q), cosα = q/√(p²+q²).

**(F1) Solid-Bedingung (ganzzahlig, exakt):**
`solid ⇔ ((q·z − p·y) mod (q·Nz)) < q·T_v` (mod positiv gewickelt).
Herleitung: g = q·z − p·y ist konstant auf Ebenen der Steigung p/q; z→z+Nz gibt Δg = q·Nz (Musterperiode), y→y+Ny gibt Δg = −p·Ny. **Einkanal-Schließbedingung: q·Nz = p·Ny.** T_v = vertikale Slabdicke in Zellen je Säule.

**(F2) Parameterwahl:**
| | 45° | 26,565° |
|---|---|---|
| (p,q) | (1,1) | (1,2) |
| Ny | 60 | 90 |
| Nz = p·Ny/q | 60 | 45 |
| T_v | 4 | 3 |
| n̂ exakt | (0,−1,1)/√2 | (0,−1,2)/√5 |
| cosα | 0,70711 | 0,89443 |

T_v-Begründung: T_v ≥ 2 trennt die Fit-Fenster der beiden Slabseiten (3³-Fenster reicht Δg ±2 bzw. ±3); T_v ≤ 4 (45°) bzw. ≤ 3 (26,6°) macht **jede** Solidzelle 18er-Nachbar einer Tauschzelle → n_voll = 0, das K3-Kriterium des Cd-Pfads bleibt streng.

**(F3) Effektive Halbhöhe (senkrecht):** `δ_eff = (Nz − T_v)·cosα/2`.
Herleitung: Nz−T_v Fluidzellen je vertikaler Säule (g durchläuft je Säule jeden Rest genau q-tel-gerecht), Halfway-Konvention hebt sich zwischen beiden Wänden auf. → 45°: δ_eff = 56·0,35355 = **19,80**; 26,6°: δ_eff = 42·0,44721 = **18,78** (beide ≈ N=38-Leiterpunkt).

**(F4) Physik-Normierung (Re_τ-Erhalt bei utau_lat = 0,003 fest):**
`nu_lat = utau_lat·δ_eff/Re_tau`, `f0 = utau²/δ_eff`, `T_ett = δ_eff/utau`, `Nx = round(2π·δ_eff)` gerade, Ub_ziel = 24,104·utau unverändert.
→ 45°: nu = 1,1455e-5 (τ_LBM = 0,5000344), Nx = 124, T_ett = 6600, 80 ETT = 528.000 Schritte, 446.400 Zellen.
→ 26,6°: nu = 1,0865e-5, Nx = 118, T_ett = 6260, 80 ETT = 500.800 Schritte, 477.900 Zellen.
Spannweiten-Check: Wandlinienlänge je y-Periode = Ny·√(p²+q²)/q = 84,9 bzw. 100,6 ≥ π·δ_eff (62,2 / 59,0) ✓.

**(F5) Kraftbilanz auf BENETZTE Fläche:** τ_w·A_ben = f·V_fluid mit A_ben = 2·Nx·Ny/cosα (Planfläche beider Wände; das ist die Fläche der modellierten glatten Schrägebene, NICHT die Voxel-Treppenfläche 2·Nx·Ny·(p+q)/q — der Flächenfaktor R2 ist genau auf die Planfläche gebaut) und V_fluid = Nx·Ny·(Nz−T_v). Einsetzen:
**τ_w = f·δ_eff — formidentisch zum parallelen Kanal.** c_f = 2·f·δ_eff/U_b². Und: soll_rx = f̄·V_fluid ist am parallelen Kanal wertgleich zur bisherigen Formel f̄·δ·2NxNy.

**(F6) Wandnormale Koordinate (Profil):** `m = ((q·z − p·y) mod q·Nz) − q·T_v` ∈ [0, q·(Nz−T_v)); senkrechter Wandabstand `d⊥ = (m + ½)/√(p²+q²)`; y⁺ = d⊥·Re_tau/δ_eff. Bins über m statt über z (su[z] ist bei Schräge sinnlos); 45°: Nx·Ny Zellen je Bin, 26,6°: Nx·Ny/2 (Spaltenparität wechselt je Bin).

**(F7) Lagenstruktur — abgezählt aus Δg der 18 Richtungen** (Δg = q·Δz − p·Δy; solid ⇔ Nachbar-Δg unter Schwelle):
- **45°:** Lage 0 (g-Abstand 0 zur Wand): **7 Solid-Links** (+y, −z, (0,1,−1), (±1,1,0), (±1,0,−1)); Lage 1: **1 Link** (nur (0,1,−1)); Lage ≥2: 0. → 2 Facettenlagen je Wand.
- **26,6°:** m0: **8 Links**, m1: **4**, m2: **1**, m≥3: 0. → 3 Lagen je Wand (m0/m1 sind die zwei Spaltenparitäten der Zweiertreppe).

**(F8) Fit-Vorhersagen (3³-Fenster, Punktwolken vollständig enumeriert):**
- 45°: y_w(Lage 0) = **0,369** (69 Punkte: 54 bei ζ=−0,354, 9 bei −0,707, 6 bei 0) — deckt die Auflage-1-Zahl 0,35–0,39; y_w(Lage 1) = **1,040**. r21 ≈ 0,034 ≪ 0,15 → **kein K2**. Punktwolke exakt η-symmetrisch → n̂ exakt (0,−1,1)/√2 (Jacobi rotiert bei θ=0 exakt um 45°), als float |n_y| == |n_z| bitgleich → **argmax-Tie, Tie-Break liefert deterministisch achse=1**; Glättung (alle Nachbarn identisch) ändert nichts.
- 26,6°: y_w(m0) = **0,184**, y_w(m1) = **0,701**, y_w(m2) = **1,079**; r21 ≈ 0,06 → kein K2; n̂ ≈ (0,−1,2)/√5 mit kleiner Verkippung (Kreuzkovarianz der unsymmetrischen Wolke), achse=2 unstrittig.
- **★ K4-BEFUND: y_w(m0) = 0,184 < 0,2 → die K4-Untergrenze markiert bei 26,6° die tragende Lage** → sie fiele auf reinen BB zurück, ~50 % der Schließung fehlten (F9), c_f säße bei ~−45 %. Das ist der kritische Entscheidungspunkt der Stufe (Schritt 0/6 unten). y⁺ je Lage: 45°: 97 / 272; 26,6°: 51 / 194 / 298 — Spalding-tauglich.

**(F9) Gate-Auswertung (Paartabelle FACETTEN-STUFE2.md B, je Lage geprüft) und Monolagen-Schließung:**
- 45°, achse=1, Boden (+y-Wand-Ast): Lage 0: **τ_x-Paar (13,8) OFFEN** (Gates j[14], j[7]: Δg=−1 solid), τ_z-Paar (18,12) ZU (Gate j[11] fluid); Lage 1: **beide ZU**. Decke gespiegelt identisch. Tie-Unschädlichkeit: fiele der Tie-Break auf achse=2, wäre ebenfalls genau das τ_x-Paar (9,16) offen, mit identischem faca — der Tie ist folgenlos.
- 26,6°, achse=2: m0: **beide Paare offen** (τ_x (9,16): Gates Δg=−2 ✓; τ_y (11,18): Gates Δg=−1/−3 ✓), m1: **nur τ_x** ((11,18)-Gate j[12] fluid), m2: keins.
- **Schließung exakt:** angewandter x-Impuls je Wandperiode = Σ_Tauschzellen 1/|n_dom| · τ_w = benetzte Planlänge · τ_w: 45°: 1·√2 = √2 ✓; 26,6°: 2·(√5/2) = √5 ✓. **Der Flächenfaktor schließt die Monolage exakt — bei 26,6° aber verteilt auf m0+m1, weshalb der K4-Entscheid tragend ist.**

**(F10) Soll-Zähler (exakt, harte Prüfungen):**
fac_N = **4·Nx·Ny** (45°: 29.760) bzw. **3·Nx·Ny** (26,6°: 31.860); Slot 7 = fac_N·⌈n_steps/100⌉ (45°/80 ETT: 157.132.800 — passt in uint); Slot 11 = N_L2·⌈n/100⌉ mit N_L2 = **2·Nx·Ny** (45°) bzw. **1·Nx·Ny** (26,6°, nur m2); Slots 8/9 = 0; fac_tau_n = n_steps an jeder Tauschzelle (Anteil Zellen mit offenem x-Paar: 45° 50 % von fac_N = 100 % der Monolage; 26,6° 67 %).

**(F11) Erwartung BB-Basislinie gekippt:** vollrau U_b⁺ = 2,44·ln(δ_eff/k_s) + 6,53; parallel-BB hat k_s = 1,0 Zellen (Mechanismus BB+SGS, KEINE Treppe — Prüferbefund). Die 45°-Treppe addiert echte Geometrie, aber als **Längsrillen** (Strömung kantenparallel, Rillentiefe ~0,35 dx senkrecht, s⁺ ~ 300 — weit jenseits des Riblet-Regimes) → Zuschlag k_s +0,5…2 Zellen erwartet: **c_f(BB, gekippt) ≈ 0,010–0,014** (k_s 1–3). Dokumentationszahl, kein Gate.

**(F12) Facetten-Arm-Soll:** c_f(parallel, z-WFB) = 0,00107–0,00114 über N=38–108; Budget ±10 % (Auflage 1: ±40 % y_w ≈ ±9–10 % τ über die Log-Dämpfung; hier Fit-y_w 0,369 vs. Halfway-Konvention 0,354: nur +4 %) → **Abnahmeband c_f ∈ [0,00096; 0,00126] für beide Winkel.** U_b⁺ muss dabei im selben selbstkonsistenten Regime landen wie parallel (~42–43); Referenz ist ausdrücklich NICHT Lee&Moser (−68 %-Restproblem ist die andere Baustelle).

## C. Implementierungsschritte (alle in main_setup_kanal-Umfeld; Kernel unberührt)

**Schritt 0 — Vorab-Diagnose (vor jedem Kernel-Messlauf, CPU, Sekunden):** CFD_KANAL_KIPP=45/26 + CFD_FACETTEN_DIAG=2 → baue_facetten-Census gegen F7/F8-Zahlen. **Entscheidet den K4-Punkt mit Messwert statt Handrechnung.**

**Schritt 1 — Parameter + Geometrie (setup.cpp:586–602):** `const uint kipp = env_u("CFD_KANAL_KIPP", 0u)` — zulässig 0/45/26 (26 = atan(½) = 26,565°), sonst print_error. Unter kipp>0: (p,q), Ny (Default 60/90, env-übersteuerbar mit Schließbedingungs-Check q·Nz=p·Ny als harte Prüfung), Nz=p·Ny/q, T_v (4/3), δ_eff, nu, f0, Nx, T_ett nach F2–F4. KIPP=0-Zweig **wörtlich unverändert** (Bitgleichheit). Guard: kipp>0 && CFD_WANDFUNKTION>0 → print_error (z-WFB an Schrägen physisch falsch).

**Schritt 2 — Flag-/Initialschleife (setup.cpp:632–647):** KIPP-Ast: solid nach F1 (ganzzahliges Modulo), sonst d⊥ nach F6, zw = min(d⊥, 2δ_eff−d⊥), Reichardt wie gehabt; Störamplitude über wh = d⊥/δ_eff. Ehrlichkeitskommentar: die u_z-Störkomponente ist nicht wandtangential — Amplitude fällt mit sz² gegen die Wand, Rest klingt im Warmlauf ab (dieselbe Klasse wie der dokumentierte Divergenzrest :638–641). Ma-Wächter (:648–653) unverändert.

**Schritt 3 — baue_facetten z-periodisch (setup.cpp:369–529):** neuer Parameter `const bool z_per=false`; unter z_per: wz-Wrap-Lambda analog wx/wy, Kandidatenschleife z 0..Nz−1 (:392), Fensterklemme :411, Linkklemme :397/:415, Glättungsklemme :474 → gewickelt. Default false ⇒ Kanal-Anker/Fahrzeug bitidentisch.

**Schritt 4 — kraft_facetten z-Wrap (setup.cpp:562–567):** den vorhandenen Audit-R3-Kommentar („ein künftiger z-periodischer Fall muss hier wickeln") einlösen — Parameter z_per, Wrap wie im Kernel.

**Schritt 5 — Verdrahtung + Census-Assertion (setup.cpp:656–664):** baue_facetten/alloc mit z_per=(kipp>0); danach **harte Assertion fac_N == 4NxNy bzw. 3NxNy** (fängt jeden vergessenen Wrap mechanisch) und Log der Lagen-y_w-Mediane. alloc_facetten_domain (lbm.cpp:375–411) unverändert — F-BBox ist am Kanal die volle Domäne.

**Schritt 6 — K4-Entscheid 26,6°:** falls Schritt 0 y_w(m0) < 0,2 bestätigt (Prognose 0,184): die K4-**Untergrenze** in baue_facetten (:455, :488) als Parameter/env `CFD_FACETTEN_YWMIN` (Default 0,2f unverändert!) — die 26,6°-Messläufe fahren 0,15 als deklarierter Messarm; Fahrzeug-Kalibrierung (K4=Spalt-/Eckklasse, 101.646 Zellen) bleibt unberührt, Nachweis: Kanal-Anker + CPU-Hash mit Default unverändert. Entscheidung mit Zahl in FACETTEN-PLAN.md nachtragen.

**Schritt 7 — Messung/Zeitschleife (setup.cpp:682–726):** U_b-Summe über `flags[n]!=TYPE_S` statt z-Randabfrage (Fluidzellzahl = Nx·Ny·(Nz−T_v), als Gegenprobe geloggt); Profilakkumulatoren über m-Bins (Größe q·(Nz−T_v)); Regler :703 und tau_kraft :706 mit δ_eff; tau_mem :709 mit A_ben; Profil-CSV :740–747 mit yplus aus F6, optional wandnormal/tangential rotierte Schwankungen ((q·u_z−p·u_y)/√(p²+q²) etc.), im Kopf gekennzeichnet.

**Schritt 8 — Facetten-Report (setup.cpp:758–786):** Slot-11-Prüfung: statt Warnung „alle Paare offen" die exakte Formel N_L2·⌈n/100⌉, Ist≠Soll = harter Fehler; y⁺-Zeile :770 rechnet mit hartkodiertem y_w=0,5 — unter KIPP je Facette das eigene yw aus dem Host-Vektor FF verwenden (Lagen-getrennt ausweisen); K2-Soll :776 → f̄·V_fluid (am parallelen Kanal wertgleich; alten Ausdruck im KIPP=0-Ast belassen); K3 (Druck_x==0, n_voll==0, n_unklar==0) bleibt streng — n̂_x ist exakt 0,0f, T_v-Wahl erzwingt n_voll=0.

**Schritt 9 — Testerweiterung (setup.cpp:2565–2660, facetten_test):** T2c: Mini-Kipp-Torus 32×12×12, 45°, AUS vs. FACETTEN=2, nach 2 Schritten Differenzen NUR an vorhergesagten Tauschzellen (Lage 0), Lage 1 und geschlossene Gates bitgleich — erledigt zugleich den offenen Stufe-2-Punkt „achse-1-Verhaltensnachweis" (45°-Kipp IST der achse-1-Fall inkl. Tie-Break).

**Reihenfolge/Läufe (Projektregel: Commit vor erstem GPU-Lauf):** Commit 1 = Schritte 1–9 + CPU-Regression; dann iGPU-Serie: je Winkel BB-Basislinie (FACETTEN=0) → Zwischenarm (=2) → Vollarm (=1), 80 ETT. **Zeit:** je Lauf ~2,4e11 LU / 594 MLUPs (iGPU, LEISTUNG.md) ≈ 7 min Kernzeit + ~28 GB Chunk-Readbacks ≈ **8–12 min/Lauf, Serie (6 Läufe) ≈ 1–1,5 h**; CPU-Kurzläufe (316 Schritte) Sekunden.

## D. Abnahmekriterien

1. **Regression KIPP=0:** CPU N=38/316 Schritte, FACETTEN=1: FELD-HASH(u) exakt **12755646098055097704**; T1/T2 grün; CFD_DUMP_CL-Diff leer (Kernel unberührt — trivial erfüllbar, trotzdem prüfen).
2. **Geometrie-Census (Schritt 0, je Winkel):** fac_N exakt 29.760 (45°) / 31.860 (26,6°); markierte Zellen 0 (45°); y_w-Lagenmediane 0,369/1,040 (±0,03) bzw. 0,18/0,70/1,08 (±0,03); Winkel zur dominanten Achse 45,0°/26,6° (±1°).
3. **Wirkpfad:** Slot 7 Ist=Soll exakt (bestehender harter Fehler :765); Slot 11 exakt N_L2-Formel; Slots 8/9 = 0; fac_tau_n = n_steps an jeder Tauschzelle.
4. **BB-Basislinie gekippt:** c_f dokumentiert (Erwartung 0,010–0,014, k_s-Rückrechnung mitliefern); beide c_f-Wege konsistent < 5 % (Messvorrichtungs-Check; Impulsaustausch ist im BB-Arm gültig).
5. **HAUPT: c_f(kipp, FACETTEN=1) ∈ [0,00096; 0,00126]** bei 45° UND 26,6°; U_b⁺ im selbstkonsistenten Regime (~42±3 wie parallel); K2: Reibung_x/(f̄·V_fluid) ∈ [0,99; 1,01] im stationären Fenster; K3: Druck_x == 0,0 exakt, n_voll = 0, n_unklar = 0; |Reibung_y|/|Reibung_x| < 0,05 (Gitteranisotropie-Diagnose, dokumentiert).
6. **Wächter:** RHO_CLAMP = 0, U_b⁺ < 200, keine divergierende f-Reihe.

## E. Grenzen (expliziter Nicht-Anspruch, gehört wörtlich ins Blatt)

Dieser Fall misst ausschließlich die **kantenparallele** τ-Komponente: die mittlere Strömung läuft entlang der Stufenkanten, das offene Paar ist das τ_x-tragende. **Die Hangauf-Lücke — Strömung quer zu den Stufen, τ-Komponente entlang der dominanten Achse (R2-Strukturgrenze, bis sin45° = 71 % der Windschutzscheiben-Spannung) — bleibt hier prinzipbedingt unmessbar** und offen für Stufe 4 (Nebenachsen). Ebenso ungeprüft bleiben: die geschlossenen Zweitpaare (tragen hier nur Fluktuationen), Kanten/Ecken (der Torus ist homogen — das Fahrzeug nicht) und das −68-%-Turbulenzerhaltungsproblem (deshalb Abnahme RELATIV zum parallelen Fall).

## F. Risiken

1. **K4 markiert m0 bei 26,6° (y_w≈0,184)** → tragende Lage fällt auf BB, c_f ~ −45 % — größtes Einzelrisiko; Gegenmittel Schritt 0 + Schritt 6 (Schwelle als deklarierter Messarm-Parameter, Default und Fahrzeug unberührt, Regression als Nachweis).
2. **Vergessene z-Wraps** (drei Host-Stellen) = stille Zähl-/Normalenfehler → fac_N-Assertion (D2) fängt jeden mechanisch.
3. **26,6°-Normalenverkippung** (unsymmetrische Punktwolke) → faca weicht wenige % von √5/2 ab, Schließungsfehler klein; n̂-Median im Log ausweisen.
4. **45°-Tie-Break-Bitflip** durch Glättungsrundung: nachweislich folgenlos (beide Achsen liefern das offene τ_x-Paar mit gleichem faca) — im T2c-Test dokumentieren, nicht verhindern.
5. **FACETTEN=2 am Kipp liefert NICHT c_f=0** (anders als parallel: zweite Lage + geschlossene Paare tragen BB-Rest) — Erwartung vorab notieren, sonst Fehlalarm.
6. **Regime-Drift:** falls der gekippte Kanal anders transitioniert (δ_eff 18,8–19,8 statt 19), ist der Relativvergleich verzerrt — U_b⁺-Gleichheit (D5) ist deshalb Teil der Abnahme, nicht nur c_f.
7. **soll_rx-Umformung** (f̄·V_fluid): am parallelen Kanal wertgleich, aber andere Op-Reihenfolge in double — alten Ausdruck im KIPP=0-Ast belassen, kein stiller Diagnose-Drift.

### Critical Files for Implementation
- /home/heiko/CFD/FluidX3D-v2/src/setup.cpp (main_setup_kanal 584–798: Parameter 586–602, Geometrie/Init 632–647, Zeitschleife 682–726, Reports 750–786; baue_facetten 369–529; kraft_facetten 536–582; facetten_test 2565–2660)
- /home/heiko/CFD/FluidX3D-v2/src/kernel.cpp (NUR Referenz, keine Änderung: calculate_indices 969–980, apply_facette/fac_paar 1650–1736, f_bbox 867–872)
- /home/heiko/CFD/FluidX3D-v2/src/lbm.cpp (alloc_facetten_domain 375–411 — unverändert, aber Abnahmerelevant)
- /home/heiko/CFD/FluidX3D-v2/FACETTEN-STUFE2.md (Paartabelle B, Messstand, Hash-Referenz 12755646098055097704)
- /home/heiko/CFD/FluidX3D-v2/FACETTEN-PLAN.md (Stufe 3, R1/R2, Auflagen 1/5 — Abnahmereferenz; K4-Entscheid dort nachtragen)
