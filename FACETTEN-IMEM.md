> **Archiv — Leitdokument ist FACETTEN.md (Stand 2026-08-16); überholt sind hier:** Festlegung (12)/2×2-System (I2-Messung: Asmuths Normalinversion nötig → 3×3, s. FACETTEN-IMEM-3X3.md), 14-Slot-Legende; I2-Torus-Zahlen unter z-Saum-Vorbehalt (2a8bd20).
> Herleitung (1)–(11)/(13)–(15), Revisions-Auflagen und I0/I1-Messstand bleiben gültige Belegkette — Einordnung: FACETTEN-ARCHIV.md.

# C1b: iMEM-Umbau — Herleitung + Implementierungsplan (Planungsagent, 2026-08-15 nachts)

> STATUS: Plan liegt zur adversarialen Gegenpruefung an (Morgen-Schritt 1, ARBEITSLISTE.md).
> Entscheidung Heiko: direkter iMEM-Umbau; Paartausch (Arme 1/2) bleibt Kontrollarm.
> Quelle im Volltext gelesen: Asmuth et al. 2021, Phys. Fluids 33, 105111, Gl. 20-28.

Alle Grundlagen sind gelesen: die Facetten-Infrastruktur (fac_geo/fac_idx/4-float-Akkumulator/Paartabelle/Slots 7–11), `apply_facette`/`apply_wall_function`/`apply_moving_boundaries`/`load_f` im Kernel, das Stufe-3-Formelblatt, der Cd-Pfad — und ich habe das **Originalpaper (Asmuth et al. 2021) beschafft und die iMEM-Gleichungen (20)–(28) im Volltext gelesen** (DiVA-Postprint-PDF). Die Herleitung unten ist damit an der Quelle verankert; Abweichungen von Asmuth sind ausdrücklich gekennzeichnet.

---

# iMEM-Umbau des Facetten-Wandmodells — Herleitung und Implementierungsplan

Stand a3fcd88, alle Zeilenangaben aus dem heutigen Code in /home/heiko/CFD/FluidX3D-v2 gelesen.

## Teil 0 — Was Asmuth tatsächlich schreibt (Quelle gelesen, S. 105111-4/5)

- **Gl. (20):** Slip-BB: `f_ī(x₁,t+Δt) = f_ijk(x_BB,t) + 2 w_ijk ρ (u_w·e_ijk)/c_s² = f^BB + f^{u_w}` — der Slip-Anteil ist ein **reiner Additivterm** auf den Bounce-Back-Wert.
- **Gl. (22)–(23):** Momentum-Exchange pro Link `Δp = e_ijk(f_ijk(x₁,t) + f_ī(x₁,t+Δt))`, Gesamtkraft `F = Σ_{ijk∈Γ} Δp` über **Γ = die Menge der Links, die die Wand schneiden**.
- **Gl. (24)–(26):** Zerlegung `F = F^f + F^{u_w}` mit `F^f = Σ e(f + f^BB)` (reiner BB-Anteil, aus den Registern) und `F^{u_w} = Σ e·2wρ(u_w·e)/c_s²`.
- **Gl. (27):** Ziel `F = F^wm = τ_w·A₁`, A₁ = Wandfläche, die die Einheitszelle schneidet.
- **Inversion:** `F^{u_w} = F^wm − F^f` wird nach u_w aufgelöst; Asmuth betont wörtlich, dass das System **bestimmt bleibt, solange F^{u_w} eine Linearkombination der e_ijk∈Γ ist** — d. h. er löst grundsätzlich ein *lineares System*, nicht per se skalar; die Linkmengen-Frage (unsere Frage!) ist bei ihm der Randsatz „the explicit form of u_w can differ between different boundary nodes in the case of complex geometries".
- **Gl. (28)** (ebener Boden, D3Q27): `u_w = −(3c_s²Δt²/(ρΔx⁴))·(3F_x^{u_w}, 3F_y^{u_w}, F_z^{u_w})` — an der ebenen Wand ist das System **diagonal**, und er invertiert **auch die Normalkomponente** (mit well-conditioned/geshifteten DDFs, `f^wc = f − w`, sein Abschnitt IV — dieselbe Shifting-Klasse wie unser `rho += 1.0f`).
- Kontext: D3Q27, Cumulant-Kollision, AMD-SGS, MOST-Wandgesetz, τ_w aus u am Austauschpunkt z_el (z₁ oder höher), SG/ESG/zeitliche EMA-Filterung von u_wm (Gl. 29/30). **Nur ebene Böden.**

Alles Weitere (Teillinkmengen, Schrägen, D3Q19, Erhaltungsfragen) ist **eigene Herleitung aus dem Momentum-Exchange-Prinzip** — ehrlich gekennzeichnet.

---

## Teil 1 — Exakte D3Q19-Formulierung im Facettenrahmen

**Notation.** D3Q19, c_s² = 1/3, w_s = 1/18 (Links 1–6), w_e = 1/36 (Links 7–18); Gegenrichtung ī = opposite(i) (ungerade i ↔ i+1, belegt kernel.cpp:1184–1188). Facettenzelle n mit n̂, y_w, faca = 1/|n̂_a| (fac_geo, lbm.cpp:392–396). Alles in Gittereinheiten (Δx=Δt=1).

**(1) Linkmenge.** 
L(n) = { i ∈ 1..18 : (flags[j[ī]] & TYPE_BO) == TYPE_S } 
— **linkweise, nicht paarweise**: jeder Link, dessen Streaming-Ursprung solid ist. Per Facettenkonstruktion ist |L| ≥ 1. Die Gate-Maske ist wörtlich die der z-WFB (schließt TYPE_E/TYPE_MS linkweise aus).

**(2) Registeridentität (Esoteric Pull).** Nach `load_f` (kernel.cpp:1403–1420) gilt für i∈L: 
fhn[i] = f_out,ī(n, t−1) 
— der implizite Halfway-BB liegt bereits im Register (Solid-Zellen laufen `stream_collide` nie; der eigene Auslauf des Vorschritts wird zurückgelesen; vom Gegenprüfer für alle Achsen verifiziert, FACETTEN-PLAN R1). **Ja, der Zusatzterm ist ein reiner Additivterm auf den vorhandenen Registern** — das ist exakt der Mechanismus von `apply_moving_boundaries` (kernel.cpp:1185–1187): dort wird `fma(−6w_i, c_ī·u_w, fhn[i])` = fhn[i] + 6w_i(c_i·u_w) addiert. Also:

fhn_neu[i] = fhn[i] + q_i,  **q_i = 6·w_i·(c_i·u_s)**  für i∈L  (3)

mit ρ_w = 1 (Konvention von apply_moving_boundaries, „necessary choice to assure mass conservation" — Festlegung: übernehmen, damit die spätere Fahrbahn-Verschmelzung termidentisch ist; 2wρ/c_s² = 6wρ).

**(4) Impulsaustausch.** Impulsgewinn der Fluidzelle über Link i∈L pro Schritt = c_i·f_in,i − c_ī·f_out,ī = c_i(f_in,i + f_out,ī), also mit (2) und (3):

Φ = Σ_{i∈L} c_i·(2·fhn[i] + q_i) = Φ^f + 6·S₂·u_s,  Φ^f = 2·Σ_{i∈L} c_i·fhn[i]  (4)

mit den **Linkmengen-Momenten** S₀ = Σw_i, S₁ = Σw_i c_i, S₂ = Σw_i c_i c_iᵀ (im Kernel im entrollten Link-Loop aus flags berechnet — kein Speicherbedarf; Asmuths Gl. 24/25 sind Φ^f und der 6S₂u_s-Term). Achtung Registersemantik: fhn[i] hält f_out der **Gegenrichtung** — dadurch hat Φ^f_t am Boden das richtige (bremsende) Vorzeichen; nachgerechnet in (10).

**(5) DDF-Shifting.** fhn sind geshiftete Populationen (rho += 1.0f, kernel.cpp:1131). (4) auf den geshifteten Registern lässt den hydrostatischen Term 2Σw_i c_i weg. Der ist rein geometrisch, an ebenen Wänden tangential exakt 0, und summiert sich über jede geschlossene/periodische Solid-Oberfläche zu 0 (Teleskopsumme über alle Fluid-Solid-Links). **Festlegung: Ziel auf den geshifteten Austausch definieren** (wie Asmuth mit f^wc); der Per-Zellen-Offset an asymmetrischen Linkmengen ist Buchhaltung, keine Physik, und hebt sich im Flächenintegral weg.

**(6) Ziel und Basis.** Abtastung wie heute (Zustand VOR Korrektur, Hans Abtastpunkt): u_t = Tangentialprojektion, ut = |u_t|, Spalding mit Y = ut·(2y_w)·def_fac_Y, tw = ρ·(ut/u⁺)², Klemme tw_max = 0,5·ρ·ut, twe = fmin(tw·faca, tw_max) — **wörtlich derselbe Ausdrucksbaum wie heute** (kernel.cpp:1686–1695; Slot-8-Semantik bleibt). Tangentialbasis:

t̂₁ = u_t/ut,  t̂₂ = n̂ × t̂₁  (6)

Ziel: tangentiale Wandkraft aufs Fluid = −twe·t̂₁ (Gegenrichtung zu u_t; Querkomponente 0 — Wandschub ist per Modell antiparallel zur Tangentialgeschwindigkeit; twe trägt schon Fläche faca und def_fac_tau).

**(7) Das lineare System — Festlegung: 2×2 in der Tangentialebene, kein 3×3.** Mit u_s = s₁t̂₁ + s₂t̂₂ (u_s liegt per Konstruktion in der Tangentialebene) und G_ab = 6·t̂_aᵀS₂t̂_b:

| G₁₁ G₁₂ | |s₁|   | −twe − 2Σ(c_i·t̂₁)fhn[i] |
| G₁₂ G₂₂ |·|s₂| = |  0  − 2Σ(c_i·t̂₂)fhn[i] |  (7)

Cramer: det = G₁₁G₂₂ − G₁₂²; s₁ = (R₁G₂₂ − R₂G₁₂)/det, s₂ = (R₂G₁₁ − R₁G₁₂)/det.

**Antwort auf die Skalar-vs-Tensor-Frage:** Die skalare Projektion (Asmuth an der ebenen Wand) reicht **nicht**, aus zwei Gründen: (a) an Schrägen ist S₂ tangential anisotrop mit Kreuzterm G₁₂ ≠ 0 — ein skalares s₁ erzeugte unkontrollierten Quer-Austausch; (b) schon an der **ebenen** Wand nullt die z-WFB per Free-Slip-Tausch den gesamten tangentialen BB-Widerstand, auch quer zur Strömung — ein Skalar ließe den Rauwand-Widerstand auf Querfluktuationen stehen und bräche die Kanal-Äquivalenz. Das Quer-Ziel „0" ist also Teil des Modells. Ein 3×3 (mit Normalkomponente) ist dagegen **nicht** nötig — siehe (12). An der ebenen Wand ist G diagonal und das 2×2 kollabiert auf zwei unabhängige Skalare — die exakte D3Q19-Entsprechung von Asmuths diagonaler Gl. (28), nur ohne Normalinversion.

**(8) Degenerations-Kaskade (Festlegung).** det und G₁₁ unter `-cl-finite-math-only` hart geprüft:
- det ≥ 1e-4·G₁₁·G₂₂ **und** G₂₂ ≥ 1e-8 → volles 2×2;
- sonst G₁₁ ≥ 1e-8 → skalarer Fallback s₁ = R₁/G₁₁, s₂ = 0 (Zähler, neuer Slot 12);
- sonst → **kein tangential wirksamer Link** (z. B. rein normale Linkmenge): keine Modifikation, Zähler Slot 13. Das ist der iMEM-Nachfolger der Slot-11-Semantik „kein Paar offen" — aber viel seltener: er trifft nur Zellen, deren sämtliche Wandlinks ⊥ t̂₁ stehen.

**(9) Klemmen (Festlegung).** Nach dem Lösen: 
s₁ ← clamp(s₁, −2ut, +2ut), s₂ ← clamp(s₂, −ut, +ut), Treffer → Slot 10 (gegatet t%100). 
Begründung der Skala: der Free-Slip-Grenzfall liefert exakt s₁ → +ut (siehe (11)) — Faktor 2 Headroom nach oben und unten (s₁ < 0 = Wand bremst stärker als BB, der Normalfall an dünnen Linkmengen); s₂ ist reine Fluktuationsnullung, O(ut) genügt. Die Klemme ist das Stabilitätsnetz für kleine G₁₁ (Einzellink schräg zu t̂): die Division wird steil, der Betrag bleibt gefangen, und **Ist ≠ Soll wird im Akkumulator sichtbar** statt lautlos zu klemmen.

**(10) Kontrollrechnung ebene z-Wand (Boden), t̂₁ = x̂.** L = {5, 9, 16, 11, 18}. G₁₁ = 6·2w_e = 1/3 = G₂₂, G₁₂ = 0 (Links 9/16 haben c_y=0, 11/18 c_x=0, Link 5 beides 0 — exakt in float). Registerinhalt: fhn[9] = f_out[10], fhn[16] = f_out[15]; mit f ≈ f_eq: Φ_x^f = 2(f₁₀−f₁₅) = 2w_eρ(−6u_x + O(u²)) = **−ρu_x/3** — bremsend, Vorzeichen ✓. Angewandter Austausch danach: Φ_x = Φ_x^f + G₁₁s₁ = −twe **exakt** — dasselbe Ziel, das der Paartausch pro Paar erzwingt (Tausch nullt, ±½τ setzt −twe·û_t). **Gleicher Soll-Austausch, andere Linkverteilung** (der Tausch verschiebt die Nichtgleichgewichts-Asymmetrie, iMEM addiert einen gleichgewichtsförmigen Term) → nicht bitgleich, statistisch äquivalent; Kriterium in Teil 3.

**(11) Grenzfall τ_w → 0 (Zwischenarm).** Ziel Φ_t = 0: s₁ = −Φ_x^f/G₁₁ = +ρu_x ≈ u_x, s₂ analog → **u_s → u_t: die Wand „schwimmt mit", reiner Free-Slip im Austauschsinn.** Physikalisch richtig (τ=0 ⇔ keine tangentiale Wandkraft). Der Paartausch-Zwischenarm (=2) ist Free-Slip im *Reflexionssinn* (jedes Paar exakt 0), iMEM-Zwischenarm (=4) Free-Slip im *Summensinn* (Summe 0, Einzellinks ≠ 0 — andere Fluktuationsinjektion). Beide haben dasselbe Kanal-Abnahmekriterium c_f ≈ 0. **Das Analogon zu Arm 2 existiert also und ist def_fac_tau = 0 — dieselbe Emissionsmechanik.**

**(12) Normalimpuls — Festlegung: nicht vorschreiben, messen.** u_s ist tangential, aber q_i injiziert Normalimpuls Φ^q·n̂ = 6(Stn₁s₁ + Stn₂s₂) mit den Kreuzmomenten Stn_a = Σw_i(c_i·t̂_a)(c_i·n̂). An der ebenen Wand exakt 0 (9/16- und 11/18-Symmetrie); an Schrägen/Teilmengen ≠ 0 → parasitärer Normalaustausch. **Bewusste Abweichung von Asmuth Gl. (28)** (er invertiert F_z mit): eine dritte Gleichung machte Einzellink-Zellen unlösbar, an ebenen Wänden ist sie leer, die Nullung des *dynamischen* Normalaustauschs ist Modellwahl (keine Physikpflicht), und unser Cd-Druckpfad (FACETTEN-CD-PFAD E1) lebt gerade vom unangetasteten BB-Normalaustausch. Stattdessen: Φ^q·n̂ pro Facette **akkumulieren** (Slot [5] des erweiterten Akkumulators) und im Kugel-A/B abnehmen.

**(13) Masse.** Δm = Σq_i = 6·(S₁·u_s) = 6·(S₁_t·u_s) — nur der **Tangentialanteil von S₁** leckt. Nachgerechnet: ebene Wand S₁ = (0,0,w_s+4w_e) → Δm = 0 exakt; **45°-Torus Lage 0** (7 Links): S₁ = (0, 5, −5)/36 ∝ −n̂ → rein normal → **Δm = 0 exakt, sogar für Hangauf-u_s**; als float-Summe gleicher Beträge mit Vorzeichen sogar bitexakt 0. Allgemein gilt: symmetrische Linkmengen haben S₁ ∥ n̂; **Leck nur an asymmetrischen Mengen (Kanten, Kugel)**. Festlegung: **keine Zwangskorrektur** — Präzedenz ist `apply_moving_boundaries` selbst (korrigiert ebenfalls nicht; bekannte O(u_w)-Eigenschaft impulsbasierter BCs), und der exakte α-Korrekturansatz q_i = w_i·α + 6w_i(c_i·u_s) mit Nebenbedingung αS₀ + 6S₁·u_s = 0 wird ein 3×3, das bei |L|=1 Rang 1 hat — genau dort, wo es gebraucht würde, ist es unlösbar. Stattdessen dreistufig: (i) **host-seitiger Leck-Census** beim Facettenbau (|S₁_t| je Facette — steht VOR jedem Lauf), (ii) Δm-**Laufzeitakkumulation** je Facette (Slot [4]), (iii) Abnahmekriterium global (Teil 4/5); der α-3×3-Ausbau liegt als dokumentierte Eskalation in der Schublade, nur bei Messwert-Überschreitung.

**(14) Einzellink-Zellen (Kugel-Mehrheit: heute 313 Tausch- von ~14.669 Facettenzellen).** |L| = 1 mit Link c: G hat Rang 1, det = 0 → Kaskade (8): skalarer Fallback s₁ = R₁/(6w(c·t̂₁)²), wohldefiniert für (c·t̂₁)² ≥ ε, sonst Slot 13. u_s ist dort **wohldefiniert, aber steil** — die Klemme (9) trägt die Stabilität; geklemmte Zellen liefern Ist < Soll, sichtbar im Akkumulator. Quer-Ziel bleibt unerfüllt (Rang 1) → Rest-Queraustausch, gemessen über [1..3]/[5]. Das ist die ehrlich benannte **unpublizierte Zone** (Asmuth hat immer die volle ebene Linkmenge).

**(15) u_w ≠ 0 (mitbewegte Fahrbahn, künftige Erweiterung) — Vorzeichen sauber.** Unbekannte wird der **volle Wandvektor im BB-Term**, u_BB (tangential); Relativgrößen stehen **nur im Ziel**:
- u_rel = u_t − (u_w − (u_w·n̂)n̂); t̂₁ = u_rel/|u_rel|; Y und tw aus |u_rel|; Ziel −twe·t̂₁ (Widerstand gegen die **Relativ**bewegung).
- System (7) unverändert mit Unbekannter u_BB: G_ab(u_BB·t̂_b) = R_a. Der BB-Term R_a subtrahiert weiter den **absoluten** Registerzustand — richtig, denn die Register kennen kein Bezugssystem.
- Konsistenzprobe: u_rel → 0 ⇒ twe → 0 ⇒ Lösung u_BB,t = u_t = u_w,t — die Wand schwimmt exakt mit, Austausch 0 ✓. Slip-Diagnose: u_s := u_BB − u_w,t.
- Doppelanwendung mit `apply_moving_boundaries` ist heute strukturell ausgeschlossen (MS-Zellgate); bei späterer Öffnung **ersetzt** der iMEM-Term die MS-Injektion auf L — identische Linkmenge, identische Termform 6w(c·u), gleiche ρ_w=1-Konvention. Genau dafür wurde (3) so festgelegt.

---

## Teil 2 — Architektur-Festlegungen

**A1 — Schalter: 1/2 bleiben Paartausch, 3 = iMEM voll, 4 = iMEM-Nullziel.** Begründung: sämtliche geloggten Läufe, Soll-Formeln (Stufe-3 F7–F10) und Doku referenzieren 1/2 als Paartausch — eine Umdeutung von =1 wäre die lautlose Fehlerklasse des Projekts. Emission: Arme 3/4 setzen zusätzlich `#define FACETTEN_IMEM`; Arm 4 emittiert def_fac_tau=0 (dieselbe Mechanik wie Arm 2). Kernel: `apply_facette`-Rumpf bleibt wörtlich; der iMEM-Rumpf steht als eigene Funktion unter `#ifdef FACETTEN_IMEM`, die Aufrufstelle (kernel.cpp:1786) wählt per ifdef. **Damit ist die Kontrollarm-Regression maximal streng: Arm 1/2 emittiert bitidentischen Kernelquelltext bis auf die zwei mechanikneutralen Erweiterungen unten → CFD_DUMP_CL-Diff prüfbar klein, Feld-Hash 12755646098055097704 exakt.**

**A2 — Kernelblock iMEM.** Gemeinsame Teile wörtlich wie heute (f_bbox-Lookup, fid-Gate, fac_geo-Lesen, calculate_rho_u, Tangentialprojektion, Spalding, tw/twe samt Slot-8/9-Zählung). `achse` wird im iMEM-Arm nicht gebraucht (nur faca) — bleibt für den Kontrollarm liegen; die fac_geo-Reserveslots [6]/[7] bleiben frei. Link-Loop 1..18 (compilerentrollt, c()/w-Literale wie `apply_moving_boundaries`): Pass 1 akkumuliert G₁₁,G₂₂,G₁₂,R₁,R₂ sowie St₁ (=S₁·t̂₁), St₂, Stn₁, Stn₂ unter der Maske (flags[j[ī]]&TYPE_BO)==TYPE_S; dann Lösen (8) + Klemmen (9); Pass 2 addiert q_i. Keine fi-Direktmanipulation, nur fhn-Register (Risiko-3-Regel unverändert).

**A3 — Akkumulator 4→6 float je Facette** (mechanikneutrale Erweiterung, Feld-Hash-unschädlich — Akkumulator-Writes speisen nie in fhn zurück, E5-Nachweis): [0] tw physisch (y⁺-Semantik unverändert), [1..3] **Ist-Tangentialkraft auf die Wand** = −(Φ^f_t + G·s), in xyz rotiert (Paararm: wie heute die angewandte Korrektur — Semantik „Reibungsvektor" identisch), [4] Δm-Leck (Paararm 0), [5] parasitärer Normalaustausch Φ^q·n̂ (Paararm 0). **Antwort auf die Messfrage: beim iMEM ist die angewandte Kraft = Ziel per Konstruktion — gemessen wird deshalb der IST-Austausch nach Klemme.** Ungeklemmt ist [1..3] eine Identität zu twe·t̂ (Wirkpfadnachweis, harte Prüfung am Kanal: rel. Abweichung < 1e-6); jede Klemme macht Ist ≠ Soll sichtbar. Akkumuliert wird nur bei tatsächlich erfolgter Modifikation (Auflage-3-Hygiene, Nachfolger von `getauscht>0`). grep-Pflicht `fac_tau[` (Leser setup.cpp Kanal-/Kugel-Report, kraft_facetten; T2 liest nur fac_tau_n — unberührt).

**A4 — Zähler 12→14 Slots** (Muster der 8→12-Erweiterung, kontrollarm-neutral): [7] Wirkpfad, [8] τ-/faca-Klemme, [9] u_t≈0-Skip (iMEM: bei ut<1e-6 keine Modifikation — t̂ undefiniert; leichte, dokumentierte Abweichung vom Paararm, der den Tausch trotzdem macht) — alle drei armübergreifend; [10] **u_s-Klemme** (Legende ändern: die Achskonflikt-Reservierung entfällt, denn **Stufe 4 „Nebenachsen" ist unter iMEM obsolet** — der Tangentialvektor trägt alle Komponenten); [11] „kein Paar offen" (nur Paararm); [12] 2×2→Skalar-Fallback (iMEM, t%100); [13] kein tangential wirksamer Link (iMEM, t%100).

**A5 — Soll-Zahlen je Fall neu rechnen (F7–F10 sind paartausch-spezifisch).** Host-Vorabdiagnose in `baue_facetten` erweitern: je Facette |L|, G-Eigenwerte (für t̂=Strömungsrichtung des Falls), |S₁_t| (Leck-Suszeptibilität), Stn — der Census ersetzt Handabzählungen und liefert die exakten Slot-12/13-Solls **vor** jedem Kernellauf. Vorab aus den Stufe-3-Lagen abgeleitet: 45°-Torus: Lage 0 voll aktiv (2×2), **Lage 1 hat genau einen, rein normalen Link (0,1,−1) ∝ −n̂ → Slot-13-Soll = 2·Nx·Ny·⌈n/100⌉** (exakt zählbar); 26,6°: m0/m1 aktiv, m2 (1 Link) per Census entscheiden. Wirkpfad-Slot-7-Soll bleibt fac_N·⌈n/100⌉ unverändert.

**A6 — Alles andere unverändert nutzbar:** fac_geo/fac_idx-Layout, F-BBox-Bindung, MS-Zellgate, K1–K4-Markierung (der 26,6°-K4-Entscheid y_w(m0)=0,184 gilt wörtlich weiter — er betrifft den Fit, nicht den Mechanismus), F6-Wrap, Kanal-/Kipp-/Kugel-Verdrahtung samt Census-Assertions, Cd-Pfad-Hybride E1–E7 (mit neuer [5]-Kontaminationszahl als Zusatzdiagnose), Guards (D3Q19, FORCE_FIELD, nicht mit WANDFUNKTION, D=1).

---

## Teil 3 — Äquivalenz- und Abnahmekriterien (Festlegungen mit Zahl)

**Kanal 0° (Kriterium definiert, da nicht bitgleich erreichbar — anderer Mechanismus):**
1. **Mechanisch exakt:** Wirkpfad Slot 7 Ist=Soll; Slots 8/9/10/12/13 = 0; per-Facetten-Ist=Soll aus [1..3] gegen twe-Summe rel. < 1e-6; Δm-Akkumulat exakt 0.0 (strukturell, s. (13)).
2. **Statistisch:** 20-ETT-Lauf N=38: **|c_f(iMEM) − c_f(z-WFB)|/c_f ≤ 5 % UND |ΔU_b⁺| ≤ 1,0** (z-WFB-Referenz 0,00107; das Band ist vor dem Lauf durch eine Wiederholbarkeitsmessung zu erden — zwei z-WFB-Läufe mit verschiedenen Störphasen; ist deren Streuung > 3 %, Band nachziehen und dokumentieren, sonst misst das Kriterium Rauschen).
3. **Zwischenarm 4:** c_f ≈ 0 (gleiche Zahl wie der Arm-2-Nachweis).

**Kontrollarm-Regression:** Arm 1 CPU N=38/316 Feld-Hash(u) exakt 12755646098055097704; Arm 0 DUMP-Diff leer; T1/T2 grün.

---

## Teil 4 — Stufenplan mit Messpunkten

**I0 — CPU-Unit-Tests + Regression (Commit 1; Projektregel: Commit vor erstem GPU-Lauf).**
- T3a (reiner Host, double-Referenz): Momente/Löser/Kaskade gegen synthetische Linkmengen — ebene Wand (G = diag(1/3,1/3) exakt), 45°-Lage-0 (7 Links), Einzellink diagonal (Skalar-Fallback + Klemme), Einzellink normal (Slot-13-Pfad). Prüft auch (10)/(11) als Zahlen (Φ_x^f = −ρu_x/3, Free-Slip s₁=u_t).
- T3b Erhaltung: Δm = 6S₁·u_s-Identität; ebene Wand und 45°-Lage-0 exakt 0.
- T2-iMEM (Mini-Domäne 32×16×24, 45°-Treppe, CPU-OpenCL): AUS vs Arm 4, nach 2 Schritten Differenzen NUR an Zellen mit tangential wirksamem L (Host-Vorhersage); Slot-Solls exakt; 100 Schritte RHO_CLAMP=0; Σρ-Drift dokumentiert (Soll ~0).
- Regression wie Teil 3. Abnahme: alles exakt/grün; Aufwand Sekunden–Minuten.

**I1 — Kanal 0° (iGPU, Commit 2).** Wiederholbarkeitsmessung z-WFB (2 Läufe) → Band fixieren; dann Arm 4 (c_f≈0) und Arm 3 (Kriterien Teil 3). ~10 min/Lauf.

**I2 — Torus-Kipp 45°/26,6° (Commit 3; Geometrie/Normierung wörtlich nach FACETTEN-STUFE3.md A/F1–F6/F11–F12).**
- Schritt 0: Host-Census (A5) je Winkel; K4-Entscheid 26,6° nach Stufe-3-Schritt 6 (CFD_FACETTEN_YWMIN-Messarm, Default unberührt).
- Läufe je Winkel: BB-Basislinie → Arm 4 → Arm 3 (+ Arm 1 als dokumentierter Paartausch-Kontrast am selben Fall). 80 ETT, 8–12 min/Lauf, Serie ~1,5 h.
- Abnahme: **c_f ∈ [0,00096; 0,00126] bei beiden Winkeln** (F12-Band), U_b⁺ ~42±3; iMEM-Slot-Solls exakt (45°: Slot 13 = 2NxNy-Formel); Δm global exakt 0 (45°) bzw. < 1e-6·ρ·V_fluid pro ETT (26,6°); |Reibung_y|/|Reibung_x| < 0,05; Slot-[5]-Summe dokumentiert.
- **I2b — NEU, nur unter iMEM möglich: Hangauf-Arm.** Gleicher 45°-Torus, Antriebsvektor wandparallel hangauf (0,1,1)/√2 statt x̂ — misst die bisher strukturell unmessbare R2-Lücke (τ entlang der dominanten Achse; Stufe-3 Abschnitt E erklärte sie prinzipbedingt unmessbar — das galt für den Paartausch). Kein Kernel-Eingriff, nur f-Vektor und m-Bin-Projektion im Setup. Abnahme: dasselbe c_f-Band. 2 Läufe.

**I3 — Kugel 3 Arme (Commit 4): AUS/4/3 (+1 als Kontrast).** Abnahme: aktive Zellenquote (Census-Soll; heute 313 → erwartet ~alle mit tangential wirksamem Link); beide Cd-Wege in allen Armen (Phantomklasse neu vermessen, K4/K5-Analog), Arm 4 Cd_Reibung exakt 0; **Δm-Drift: Σρ̄-Zeitreihe, Band |Δρ̄|/ρ̄ < 1e-5 über den Lauf** — die erste Messung an einer Geometrie mit S₁_t ≠ 0, DIE neue Zahl dieser Stufe; Klemmquote Slot 10 dokumentiert (Erwartung < 5 % der Facettenzellen·Schritte, sonst Analyse vor Weitergang); einmal FP32-Sprosse (FP16C aus). Plausibilität C_d 0,45–0,5 subkritisch (kein Gate).

**I4 — Fahrzeug (Serie ≥6 Läufe à ≤2,5 h, wie FACETTEN-PLAN Stufe 5):** Arme AUS/4/3; y⁺-Median aus [0]: 1122 → ~140, Verteilung schmal; Δm-Wächter; Bodenstreifen-Zeitreihe; Cd/Cz mit C7-Notiz davor. Abbruch: RHO_CLAMP > 0, divergierende Kraftreihen, Δρ̄-Drift über Band.

Alle Läufe unter der 2,5-h-Grenze (Kanal/Kipp/Kugel: Minuten; Testleiter strikt CPU → iGPU; B70 erst I4).

---

## Teil 5 — Risiken (inkl. dessen, was Asmuth nicht abdeckt)

1. **Unpubliziertes Terrain:** Asmuth = D3Q27, Cumulant, MOST, ebener Boden, volle Linkmenge; wir = D3Q19, SRT+Smagorinsky, Spalding, **Teillinkmengen an Schrägen**. Neu und unbelegt: Rang-1-/Einzellink-Zellen (volle τ·faca-Last durch einen Diagonallink ist Extrapolation — die Kugel misst sie), Massenleck an asymmetrischen Mengen (Messpfad + α-3×3-Eskalation in der Schublade), parasitärer Normalaustausch (Slot [5] + Kugel-A/B). Zusätzlich weichen wir bewusst ab: keine Normalinversion (Gl.-28-F_z), 2×2 statt komponentenweise — beides begründet in (7)/(12).
2. **u_s-Stabilität bei kleinem G₁₁:** Klemme fängt den Betrag, aber hohe Klemmquoten heißen systematisch Ist < Soll → an der Kugel quantifizieren; bei hoher Quote Ziel-Deckelung nachdenken (z. B. zusätzlich twe ≤ β·G₁₁·ut), nicht stillschweigend nachregeln.
3. **FP16C:** R₁/R₂ sind Differenzen fast gleicher Register (f₁₀−f₁₅) — FP16-Quantisierungsrauschen geht ×3 in s₁; q_i selbst ist klein gegen die DDF-Auflösung. FP32-Sprosse an der Kugel ist Pflicht, ggf. Kanal-A/B nachziehen.
4. **-cl-finite-math-only:** alle Divisionen (det, G₁₁, ut) mit fmax-Guards; NaN = UB-Klasse (Spalding-Präzedenz).
5. **Äquivalenzband ohne Rauschboden gemessen** → Wiederholbarkeitsmessung VOR der Bandfixierung (I1), sonst prüft das Kriterium Statistik statt Mechanik.
6. **update_force_field-Phantom wechselt die Gestalt:** statt Tausch-Phantom jetzt q-Kontamination (kleiner, aber vorhanden) — CSV-Kennzeichnungen bleiben, Kugel-K4/K5-Analoga wiederholen.
7. **Stille Leser bei Layoutwechseln** (Akkumulator 4→6, Slots 12→14): grep-Pflicht `fac_tau[`/Slot-Legenden; T2 unberührt.
8. **Doku-Drift:** Stufe-3-Solls F7–F10 gelten nur für Arm 1/2 — vor dem ersten iMEM-Kipp-Lauf neue Soll-Tabelle ins Formelblatt, sonst schlagen die harten Ist=Soll-Fehler zu Unrecht an.
9. **Slot-9-Semantikabweichung** (iMEM modifiziert bei ut≈0 gar nicht, Paararm tauscht trotzdem) — bewusst, im Code kommentieren; betrifft nur ruhende Totwasserzellen.

Sources: [Asmuth et al. 2021, Phys. Fluids 33, 105111 (AIP)](https://pubs.aip.org/aip/pof/article/33/10/105111/1065118/Wall-modeled-lattice-Boltzmann-large-eddy), [DiVA-Volltext-PDF (gelesen, Gl. 20–28)](https://uu.diva-portal.org/smash/get/diva2:1640098/FULLTEXT01.pdf), [TUHH-Eintrag](https://tore.tuhh.de/entities/publication/46f1ef2a-6aa1-4da0-8f96-cca78b5ab8d8)

### Critical Files for Implementation
- /home/heiko/CFD/FluidX3D-v2/src/kernel.cpp (apply_facette 1650–1736, apply_wall_function 1608–1648, apply_moving_boundaries 1181–1190 — Vorlage des Additivterms, load_f 1403–1420, Aufrufstelle 1783–1787)
- /home/heiko/CFD/FluidX3D-v2/src/lbm.cpp (Guards 128–144, Zählerpuffer 266, alloc_facetten_domain 373–414, Emission 767–771, run-Bindungsfehler 1435)
- /home/heiko/CFD/FluidX3D-v2/src/setup.cpp (baue_facetten + Census-Erweiterung, Kanal-/Kipp-/Kugel-Verdrahtung und Reports, facetten_test T1/T2/T3)
- /home/heiko/CFD/FluidX3D-v2/src/lbm.hpp (Statiken/Schalter, Slot-Legende 158, Akkumulator-Member)
- /home/heiko/CFD/FluidX3D-v2/FACETTEN-STUFE3.md (Torus-Geometrie/Normierung F1–F6/F11–F12 — wörtlich wiederverwendet; F7–F10 durch iMEM-Solls ersetzen)

---

# Revision nach adversarialer Gegenpruefung (2026-08-16 morgens)

**Urteil: freigeben mit Auflagen.** Der mathematische Kern haelt (Faktor 2 = beide Beine
DESSELBEN Pakets, kein Doppelzaehlen; 2x2-System und G-Momente nachgerechnet; Asmuths
Gl.-28-Diagonale (3,3,1) in D3Q27 unabhaengig reproduziert; Masse-Ledger vollstaendig;
Kaskaden-Schwellen trennscharf). Die 7 Auflagen, alle uebernommen:

1. **Gl. (2) korrigiert:** fhn[i] = f_out,ī(t−2) — der Esoteric-Pull-BB ist ein
   ZWEI-Schritt-Umlauf (Slot-Paritaet nachgerechnet; fullway-artige Latenz, stationaer
   identisch zu Halfway). Formeln unberuehrt (alle zeitindexfrei); relevant erst fuer
   instationaere Fahrbahn in (15).
2. **Masse:** S₁ KOMPONENTENWEISE akkumulieren, erst danach mit u_s projizieren — nur so ist
   das Kanal-Kriterium "Δm exakt 0.0" erfuellbar (Summandenreihenfolge!). I2-Kriterium 45° von
   "exakt 0" auf Band |Δm| < 1e-7·fac_N·n_steps. Vorzeichen richtiggestellt:
   S₁(45°-Lage-0) = (0,−5,+5)/36 ∝ +n̂ (Summe ueber L, nicht ueber Solidrichtungen).
3. **T2-iMEM: 1-SCHRITT-Lokalisierung** — iMEM modifiziert schon am Gleichgewicht (s₁≈u_t,
   Gl. 11); nach Schritt 2 ist der Effekt zu Nachbarn gestreamt. Exakte Lokalisierung nur nach
   Schritt 1; der 2-Schritt-Lauf prueft nur Slot-Solls (oder 1-Ring-Erlaubtmenge).
4. **Slot-13-Soll an "Slot 9 = 0 am Torus" koppeln** (sonst kippt die harte Pruefung bei einem
   einzigen ut<1e-6-Ereignis zu Unrecht).
5. **I2b-Umbauliste vollstaendig:** U_b-Summe, CFR-Regler/set_f-Vektor, tau_mem (Fw-Projektion)
   und Ub+-Waechter alle auf (0,1,1)/√2 projizieren; Streamwise-Periode hangauf = 4,3·δ_eff
   (Konventionsabweichung, dokumentiert).
6. **A1 praezisiert:** Arm-1-Dump-Diff = EXAKT die 4→6-Stride-Stellen, sonst nichts; alle
   Host-Leser umstellen (lbm.cpp:385/397, setup.cpp:549-551/723/769/1292/1361);
   FACETTEN_IMEM-ifdefs ausserhalb der R()-Bloecke splicen (bekannte V2-Werkzeugfalle).
   Feld-Hash-Anspruch des Kontrollarms bleibt (Akkumulator speist nie in fhn zurueck).
7. **Iron Rule 2 explizit:** nach der I0-Implementierung unabhaengiger Pruefagent VOR den
   ersten Messlaeufen.

---

# Messstand I0/I1 (2026-08-16 vormittags)

**I0 (Commits bis 69dd1de):** alle 5 CPU-Tests gruen (T1/T1b/T2 Paararm, T3a double-Referenz
inkl. Spiegelfall {11}, T2-iMEM 1-Schritt-Lokalisierung: 1216 erlaubt / 0 verboten / Bilanz
1344=1216+128 exakt). Pipeline fing drei Fehler vor dem ersten Messlauf: vergessenes
def_fac_tau im Ziel (Arm 4 lief bitidentisch als Arm 3), Faktor-6-Fehler im
Normalkontaminations-Akkumulator, det=0-UB-Loch der Kaskade (stammte aus dem Plan selbst;
jetzt: Branch 1 braucht beide Diagonalen, eigener Quer-Skalar-Zweig, Slot-13-Semantik korrekt).
Kontrollarm-Regression: Arm-1-Feld-Hash exakt 12755646098055097704.

**I1 Kanal 0 Grad (iGPU, N=38): AEQUIVALENZ BESTANDEN.**
| | Phase 0,0 | Phase 1,7 | Mittel |
|---|---|---|---|
| Paararm c_f (80 ETT) | 0,0010175 | 0,0010501 | 0,001034 |
| iMEM c_f (80 ETT) | 0,0010846 | 0,0010067 | 0,001046 |

Differenz der Mittel +1,2 % INNERHALB des gemessenen Rauschbodens (Paararm-Spreizung 3,2 %);
U_b+ alle 24,10-24,11 (gleiches Regime); **Delta-m ueber 507k Schritte exakt 0,00000000**;
Wirkpfad Ist=Soll exakt (71.748.720), alle Kaskaden-/Klemmen-Zaehler 0. Arm 4 (Nullziel):
c_f -> 0 exakt (Gl.-11-Kriterium). CFD_KANAL_PHASE=0 bitidentisch zum Alt-Init (verifiziert).

**Befund am K2-Instrument (kein iMEM-Befund):** Verhaeltnis 1,035-1,042 bei ETT=80/WARM=20 in
BEIDEN Mechanismen (Paararm 1,0355/1,0356 identisch) vs 0,996-0,998 bei ETT=20/WARM=10 --
Einschwingen im Messfenster; das 1-%-Band gilt nur fuer hinreichend stationaere Fenster.
K2-Gate entsprechend nachziehen (laengerer Warmlauf oder Fensterpruefung) -- als TODO vor I2.

Naechster Schritt: I2 Torus-Kipp 45/26,6 Grad nach FACETTEN-STUFE3.md (+ neue iMEM-Solls per
Host-Census statt der paartausch-spezifischen F7-F10).

---

# Messstand I2 Torus-Kipp (2026-08-16 mittags): NICHT BESTANDEN -- Ursache identifiziert

**BB-Basislinien (Erkenntnis):** 45°: c_f 0,00166, 26,6°: 0,00179 -- die gekippte BB-Wand ist
~6x GLATTER als die parallele (0,0106) und liegt unter glatt-turbulent: der Rauheitsantrieb der
parallelen BB-Wand fehlt, der Kanal sitzt schon in der Basislinie im bekannten
nicht-selbsterhaltenen Regime (die -68-%-Baustelle, jetzt von der anderen Seite sichtbar).
F11-Schaetzung (0,010-0,014) damit widerlegt; keine Relaminarisierung (stabile Fluktuation
ueber 80 ETT).

**iMEM-Arme: DURCHGEFALLEN.**
| Lauf | c_f | Band [0,00096;0,00126] |
|---|---|---|
| 45° Arm 4 (Nullziel) | 0,0040 | 3-4x drueber, sogar UEBER BB-Basis |
| 45° Arm 3 | 0,0045 | 3,5-4,7x drueber |
| 26,6° Arm 4 | 0,0287 | katastrophal (16x) |
| 26,6° Arm 3 | 0,0315 | katastrophal |

**Diagnose (die Instrumente zeigen die Ursache):** RHO_CLAMP 0 (numerisch sauber), Delta-m
218/-627 (im Plan-Band) -- aber **Normalkontamination [5]: -1.285 (45°) bzw. -763.625 (26,6°)**
gegen ~0 am parallelen Kanal. Der Gl.-12-Mechanismus: das tangentiale u_s injiziert ueber die
Stn-Kreuzmomente der ASYMMETRISCHEN Linkmengen permanent Normalimpuls an der Wand -- eine
Sekundaerstroemungs-Quelle, die die Grenzschicht aufblaeht und als Widerstand erscheint. Genau
die im Plan als "Modellwahl, messen statt vorschreiben" markierte Abweichung von Asmuth
(er invertiert die Normalkomponente MIT, Gl. 28) -- **die Messung hat entschieden: Asmuth hatte
recht.**

**Naechster Schritt (I2-Iteration 2): 3x3-System** -- dritte Gleichung Normalziel 0 (Phi_n =
Soll-BB-Normalaustausch), Degenerationskaskade rangbehaftet erweitern (Einzellink-Zellen:
Prioritaet t1-Ziel), an der ebenen Wand entkoppelt Stn=0 exakt -> I1-Aequivalenz bleibt
konstruktiv erhalten. Danach I2 wiederholen.
