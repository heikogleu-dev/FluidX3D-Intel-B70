# Erhaltende Klemmen, Stufe 2 — Zustandsklemmen als Sicherung mit Budget + automatische Herleitung der Klemm-Handwerte

Plan des Planungsagenten, 15.09.2026 (nach Neustart fortgesetzt), Stand HEAD c86c96e. Heiko: „Handwerte möglichst automatisch
physikalisch sinnvoll setzen“; alles an 8 mm; danach Block-Audit-Schleife. **Entscheidungen E1–E6 wie empfohlen übernommen**
(Heikos Vorgabe „selbstständig weiter“ für den Klemmen-Block).

## 0 Kurzfazit

- **u-Klemme:** 0,57735 ist c_s (lbm.cpp:2032); der Handwert ist die **Form** (je Komponente). Parameterfrei: Betragsgrenze
  |u|² ≤ c_s² = 1/3 (größte isotrope Kugel mit f_eq ≥ 0, in Gittereinheiten u_lat-unabhängig). Kugel ⊂ Würfel → U_FP16-Marge 3,46 und
  Lift-Schranke 0,9021 bleiben konstruktiv gültig.
- **ρ-Klemme 0,5/1,5 ist bitgleich herleitbar:** |δρ| ≤ u_max²/(2c_s²) = ½ mit u_max = c_s (Druckäquivalent der Positivitätsgrenze,
  gleiche Ordnung wie das Gleichgewicht 2. Ordnung). Keine Positivitätsgrenze; unter Stufe 1 + Betragsklemme numerisch entbehrlich.
  Ob Sicherung oder Reparatur, zeigt nur Arm B2 (8-mm-Treffer 100 % am bewegten Boden).
- **Budget:** |ΔCd_äq,Impuls| + |ΔCd_äq,Masse| ≤ k(B)·σ_B(cd_rest); k(4) = 1,011 aus der Stichprobenunsicherheit von σ selbst;
  Masse über Impulsfluss bei u∞ (Lagally): ΔCd_m = 2·ppm·10⁻⁶·N_y·N_z/A_lat. **8 mm (kl_s0c, Rechnung):** Nah 0,006σ, Fern 0,22σ,
  Summe 0,23σ netto / 0,50σ brutto → eingehalten.
- **Neue Schalter:** CFD_KLEMM_BUDGET (Vorgabe 2 = Fehler, nur Host), CFD_U_KLEMME (0), CFD_TOR_HUELLE (0), CFD_RHO_HUELLE (0).
- **Slots 295–300** (+ optional 301/302), passt in Puffer 320 aus P1a.
- **Reihenfolge:** Z2a + Z2c sofort nach S0c/S0d-Prüfpass (vor Stufe 1); Z2b/Z2d/Z2e nach P1a; Z2f nach P1c.
- **8 mm entscheidet:** Budgeturteil, Diagonallücke, Stabilität B2 am Boden. **Nicht:** Cd-Differenzen < ≈ 0,073 (2 Arme, 2σ),
  u-Klemme bei 4 mm.
- Testhaken nur Kugel ≤ 16 mm oder CPU; keine neuen *_HAKEN-Namen (Sperren greifen nur CFD_(KLEMM|POSITIV)_HAKEN).

## 1 Handwert-Inventar

| Nr. | Klemme/Tor | Stelle | erhaltend? | 8 mm | 4 mm | Handwert? |
|---|---|---|---|---:|---:|---|
| 1 | RHO_CLAMP 0,5/1,5 | defines.hpp:101-102, Emission lbm.cpp:2180-2181, kernel.cpp:1257 | Masse nein, Impuls ja | Nah 3 176 331, Fern 62 515; nach Warmlauf 1 126 800, 100 % K1 | Nah 14 434 460, Fern 668 538 | Wert herleitbar = 0,5/1,5 (§2.2) |
| 2 | u-Klemme ±def_c je Komponente | kernel.cpp:3213-3232, update_fields :4232-4240 (tot), SURFACE (nicht gebaut) | Impuls nein | 1872 (Facetten, nur Anlauf) | 1 375 206 | Wert = c_s; Form ist Handwert (§2.1) |
| 3 | Lift-ρ-Tor (0,5; 2,0) | kernel.cpp:4658, Slot 270 | Tor (alter Randwert bleibt) | 0 (60 ms) | – | Bildhülle herleitbar (§2.3) |
| 4 | Lift-u-Tor \|v\| ≥ 1,0 | :4686, Slot 214 | Invariante | 0 bei 2,82·10⁹ | 0 | 1,5625 = Λ² hergeleitet; 1,0 Wächterschwelle |
| 5 | Wächter ρ ∉ [0,4; 2,1] an TYPE_E | :3100, Slot 210 | Zähler | 0 | 0 | Rand 0,1 Handwert; folgt Schreiberhülle |
| 6 | Wächter \|u\| ≥ 1,0 an TYPE_E | :3127, Slot 212 | Zähler | 0 | 0 | Schwelle zwischen 0,9021 und 1,99902 |
| 7 | SISM ν_t ≥ 0 | :3500, :5248 | nur w, erhaltend | Slot 127 34,8 % | – | nein (Physik; (CΔ)² aus C_K = 3/2) |
| 8 | SPONGE w ≥ 0,5 | :3625 | nur w, erhaltend | 382 795 843 | gesättigt | ja (τ ≤ 2 Genauigkeitsdeckel) — eigener Punkt, nicht Stufe 2 |
| 9 | P-TRT max(0, w−ω_g) | :3754 | erhaltend | – | – | nein |
| 10 | Wandmodell-Kette (τ-, u_s-, sn-, APG-, y_w-Klemmen) | Slots 9/10/16/19 | Injektion f_neq Facette | u_s 3 273 470, sn 1 442 517 | – | eigene Liste, nicht Stufe 2 |
| 11 | Instrumentkonstanten S=2¹⁴, Kappung 16/2, ρ_max 2,1 | lbm.cpp:2183, kernel :2924, setup :1405 | Messgerät | Kappung 0 | – | S hergeleitet; 2,1 muss mit der Hülle wandern |
| 12 | Host-Folgen (N2F-Band ±0,01/2 %/0,1; REK_PRUEF) | setup.cpp:8561, :8584ff, :5062 | Diagnose | – | – | hängen an den Makros; Z2a lässt Werte unverändert |

## 2 Herleitungen

### 2.1 u: Betrag statt Komponente
- f_eq,i + w_i = w_i·ρ·g(x_i), g(x) = 1+3x+4,5x²−1,5|u|², Minimum 0,5−1,5|u|² bei x = −1/3 → |u|² ≤ 1/3 hinreichend, größte
  isotrope Grenze. Exakte (anisotrope) Region verworfen (18 quadratische Lösungen, Spill-Risiko, bei 8 mm nach Warmlauf 0 Treffer).
- **Diagonallücke heute:** u = (0,5; 0,5; 0) wird nicht geklemmt, aber für c = (−1,0,0) ist g = −0,125 < 0 (Slot 297 zählt obere Grenze;
  exakt: Stufe-1-Slot 279).
- Betragsklemme u_c = s·u_roh, s = c_s/|u| → Δj = w·ρ_c·(s−1)·u_roh parallel zu u; Stufe-0-Buchung bleibt gültig.
- Verworfen: u_max = k·u_lat oder Ma_max (k wäre Handwert). Rundungsreserve in u verworfen (Aufgabe von Stufe 1).
- Emission `def_u2max (def_c*def_c)` → H2 folgt automatisch, [295] ⊂ [296] über monotone Rundung.

### 2.2 ρ: Konsistenzhülle, numerische Hülle, Rolle von Stufe 1
- **Konsistenzhülle:** p = c_s²ρ, Staudruck bei |u| = c_s ist ½ρc_s² → [1 − u_max²/(2c_s²); 1 + …] = [0,5; 1,5], bitgleich.
  Isotherme Exponentialform e^{∓½} = [0,607; 1,649] verworfen (nicht ordnungskonsistent). Bernoulli-Variante verworfen.
  Hülle aus physikalischem c_s², **nie aus emittiertem def_c** (sonst schrumpft sie unter H2).
- **Numerische Hülle:** RHO_FP16 → ρ ≤ 1 + 65504/32768 = 2,99902; Untergrenze DDF-Summenauflösung Σ ULP(w_i·2¹⁵)/2¹⁵ =
  (8 + 6 + 6)/32768 = 20/32768 (Rechnung). Als Brüche emittieren.
- **Unter Stufe 1 numerisch entbehrlich** (alle f ≥ 0 ⇒ |u_α| ≤ 1, ρ ≥ 0), aber nicht an K0, machtlosen Zellen, MB/iMEM/boden_eq/Lift.
  Nur eine Seite lockern ist falsch (Boden: 15 815 zugeführt / 16 722 entfernt) → B2 lockert beide.
- **Arm B2 = Stabilitätsversuch:** C+B1 mit Klemme an der numerischen Hülle, Konsistenzhülle nur gezählt (298/299). Stabil + Cd in σ →
  Klemme war Kosmetik; kippt → Reparatur am bewegten Boden (V1-Herkunft), dann ist das Thema die Bodenrandbedingung.

### 2.3 Tore und Wächter
- Λ = 1,25 hergeleitet (Lebesgue-Konstante 4-Punkt-Lagrange, kernel.cpp:4442-4460), Λ² = 1,5625.
- CFD_TOR_HUELLE=1: Bildhülle 1 ± Λ²·½ = (0,21875; 1,78125) → Tor wird Invariante (Soll 0). Heute verwirft (0,5; 2,0) legale Werte
  in (0,219; 0,5]. Unter B2 kein Bild (1 − 1,5625·(1−6·10⁻⁴) < 0) → Tor = numerische Hülle, darf greifen (Slot 270).
- Slot 210: Schreiberhülle ± eine RHO_FP16-ULP. S0c-Wickelschranke 2,1 übernimmt das Hüllenmaximum.
- Slots 212/214 (1,0) nicht umstellen (Wächterschwelle zwischen zwei hergeleiteten Grenzen).

### 2.4 Klemm-Budget
- σ_B(cd_rest) = block_sem(ber_cd, B), B = 4 (Entscheidung Heiko S0-Plan); σ(cz_rest) neu = block_sem(ber_cz, B). Kugel: cd_w/cz_w.
- **k(B) = √((1+r)² − 1)**, r = √(1−c₄²)/c₄, c₄(B) = √(2/(B−1))·Γ(B/2)/Γ((B−1)/2): k(4) = 1,011, k(8) = 0,786, k(16) = 0,634.
  Grenze: korrelierte Blöcke unterschätzen σ (Budget strenger, sichere Seite). Ablösebedingung: Lag-1-Korrelation der Blockmittel.
- Impulsäquivalent vorhanden (setup.cpp:1443). **Massenäquivalent neu:** ΔCd_m = −si_F(u_lat·Q_netto/n)/(q∞A) = 2·|Q|/(u_lat·A_lat)
  (Senke +, Quelle −); kein Cz-Anteil. **Deklariertes Interim „netto bei u∞“**; Ablösebedingung: brutto reißt, netto nicht → lokale
  Summen Σq(w·Δρ·u_x) ± (Slots 301/302).
- Urteile Nah, Fern, Nah+Fern: |ΔCd_j| + |ΔCd_m| ≤ k·σ_cd; |ΔCz_j| ≤ k·σ_cz; nur Phase nach Warmlauf; σ aus dem Nahfeld.

| 8 mm (kl_s0c, nach Warmlauf) | A_lat | Q netto/Schritt | ΔCd_m netto | Q brutto/Schritt | ΔCd_m brutto | ΔCd_j |
|---|---:|---:|---:|---:|---:|---:|
| Nahfeld (5048 Schritte) | 28 906 | −0,1797 | 1,66·10⁻⁴ = 0,006σ | 6,445 | 5,95·10⁻³ = 0,23σ | 0 |
| Fernfeld (1262 Schritte) | 1 807 | −0,3849 | 5,68·10⁻³ = 0,22σ | 0,461 | 6,81·10⁻³ = 0,26σ | 0 |
| Summe | | | 0,23σ | | 0,50σ | |

## 3 Verhalten bei Überschreitung
- In berichte_klemmbilanz nur print_warning + `klemm_budget_verletzt`; Gesamturteil `klemm_budget_gesamt(nah, fern)`;
  klemm_bilanz_abschluss meldet am Fallende EINEN print_error für beide Flags.
- CFD_KLEMM_BUDGET: 0 = nicht bewerten, 1 = Warnung, 2 = Fehler (Vorgabe).
- „Nicht prüfbar“ → Warnung: σ < 0 (< 2B Samples), keine Schritte nach Warmlauf, mehrdeutig > 0. Kappung > 0: Summen Untergrenzen;
  reißt die Untergrenze → Fehler, sonst Warnung.
- dichteklemme_fazit unter klemm_bilanz_on auf das Budgeturteil verweisen. Neue Datei klemm_budget.csv (je Domäne/Phase: n, Q_netto,
  Q_brutto, ΔCd_j, ΔCd_m netto/brutto, ΔCz_j, σ_cd, σ_cz, B, k, Urteil).

## 4 Schalter, Slots, Arme, Haken

| Schalter | Vorgabe | Wirkung | Sperre |
|---|---|---|---|
| CFD_KLEMM_BUDGET 0/1/2 | 2 | nur Host | – |
| CFD_U_KLEMME 0/1 | 0 | 1 = Betrag, `#define U_BETRAG`, `def_u2max (def_c*def_c)` | – |
| CFD_TOR_HUELLE 0/1 | 0 | Lift-ρ-Tor und Slot 210 aus der Bildhülle | – |
| CFD_RHO_HUELLE 0/1 | 0 | Zustandsklemme an der numerischen Hülle | print_error ohne CFD_POSITIV=2, CFD_U_KLEMME=1, FP16S, RHO_FP16 |

| Slot | Inhalt | Soll |
|---|---|---|
| 295 | \|u_α\| ≥ def_c (Komponentenhülle), beide Arme, kl-Bit 8 | A: [28] = [295] |
| 296 | u² ≥ def_u2max (Betragshülle), kl-Bit 16 | B1: [28] = [296]; [296] ≥ [295] |
| 297 | 296 ∧ ¬295 (Diagonallücke) | [296] = [295] + [297] |
| 298/299 | ρ_roh außerhalb Konsistenzhülle unten/oben (nur RHO_HUELLE) | – |
| 300 | Lift-ρ außerhalb Bildhülle (zählt) | A: [300] = 0 ∧ [270] = 0 → Arm T logisch bitgleich |
| 301/302 | optional Σq(w·Δρ·u_x) ± | wickelnd, Ausnahmeliste |

**8-mm-Arme** (B70, Zeile rr_c2d_dd8_b_b70, 300 ms, je EINE Variable): M-A (Vorgaben nach Z2f, bitgleich zu P1-A1) · M-B1 (U_KLEMME=1)
· M-T (TOR_HUELLE=1, nur wenn [270]+[300] > 0) · M-C (POSITIV=2, aus Stufe 1 M2 übernehmbar, wenn M-A bitgleich P1-A1) · M-CB1 (C +
U_KLEMME) · M-CB1B2 (+ RHO_HUELLE: Sicherung oder Reparatur?).

**Haken** (Absturzsperre: Kugel ≤ 16 mm auf GPU oder CPU; nur neue Werte von CFD_KLEMM_HAKEN): Budget an der Kugel mit kleinem
T_WARMUP (≥ 8 Samples) — (a) ohne Haken „eingehalten, 0 Treffer“; (b) H1 Massenbudget verletzt, genau ein print_error am Fallende;
(c) H2 Impulsbudget verletzt; (d) T_WARMUP ≥ T_END „nicht prüfbar“, rc 0; (e) H1 + BUDGET=1 nur Warnung. Z2b/Z2d: Kugel H2 →
[28] = [295] bzw. [296]. Z2e: Kugel hat keinen Lift → dd-Test auf der CPU mit H1 ([300] > 0). Z2f: H1 übersteuert beide Hüllen.

## 5 Spill und Kosten (Rechnung)
Z2b Immerpfad: u² (3 Mul, 2 Add, 1 Vergleich, 2 OR), keine Speicherzugriffe; Z2d ersetzt 6 min/max + 6 Vergleiche durch denselben
Block. Trefferzweig 1 Div, 1 sqrt, 3 Mul (Rate 8 mm ≈ 2·10⁻⁹ je Zelle-Schritt). Risiko: IGC macht select → sqrt/Div in jeder Zelle
(+1–2 % Flops, Wanduhr unbekannt → messen). Spill kam bei S0b aus Flag-/Nachbarzugriffen, nicht Arithmetik; Gate entscheidet
(neue Arme prod8nah/prod8fern mit U_BETRAG, TOR_HUELLE, RHO_HUELLE, POSITIV). Budget nur Host: 0 MB, 0 B.

## 6 Commits
- **Z2a** Herleitung statt Literal (bitgleich): RHO_CLAMP_MIN/MAX als Ausdruck aus c_s² (= 0,5f/1,5f) mit static_assert; def_c-Kommentar;
  Invarianten dokumentieren, Interims deklarieren. Soll: CFD_DUMP_CL-Diff 0 Byte (prod8nah, prod8fern, Kugel), Gate 0/0, Kugel-Hash = HEAD.
- **Z2c** Budget (Host): §2.4/§3, σ_cz, klemm_budget.csv, Fazittext, CFD_KLEMM_BUDGET. Soll: Kugel-Leiter bitgleich; Kugel (a)–(e);
  8 mm „eingehalten“ mit Zahlen aus §2.4.
- **P1a–P1d** (Stufe 1).
- **Z2b** Zähler 295–297, 300 (nach P1a). **Z2d** CFD_U_KLEMME (nach Z2b). **Z2e** CFD_TOR_HUELLE (nach Z2b).
  **Z2f** CFD_RHO_HUELLE (nach P1c und Z2d).
- 8-mm-Matrix §4, danach Block-Audit-Schleife über Stufe 0–2.

## 7 Fallen
R()-Klammerfalle (alles inline); `//` mitten in Kernelzeilen; `#ifdef` nur als String-Splice, mehrgliedrige `#if` ohne Leerzeichen;
Emission als Brüche (to_string(…,4u) schneidet ab: 20/32768 → 0,0006); print_error = exit(1) nur im Abschluss; ρ-Hülle aus physikalischem
c_s²; neue Hakenwerte nur über CFD_KLEMM_HAKEN (klemm_haken_env-Obergrenze anheben); -cl-finite-math-only (clamp/fmin machen NaN endlich,
`if(u2>lim)` lässt NaN durch); kl nur unter KLEMM_BILANZ — Klemme selbst darf nicht davon abhängen; update_fields (tot) mitziehen;
Wickelausnahme nur 301/302; kein Rückleser; Gate-Schnappschüsse + gen_main-Anhang; N2F-Band-Diagnose unter B2 umstellen.

## 8 Entscheidungen (übernommen wie empfohlen)
E1 CFD_KLEMM_BUDGET Vorgabe 2 · E2 Masse netto bei u∞ als Interim · E3 Urteil auch Nah+Fern · E4 Z2a/Z2c vor Stufe 1 · E5 B2 als letzten
Arm fahren · E6 Betragskugel statt exakter Region.

## 9 Nicht verifiziert
σ-Unterschätzung durch korrelierte Blöcke; Lagally nur exakt für Quellen im Parallelstrom; IGC-select und Wanduhr Z2b/Z2d; Spill Z2b–Z2f;
NaN-Verhalten; Monotonie [295] ⊂ [296] unter -cl-mad-enable; Lift-Tor = 0 nur 60 ms belegt; Häufigkeit Diagonallücke; Stabilität B2;
**Ursache: boden_eq sieht 0 ρ-Treffer, stream_collide am selben Band 1,13 Mio** (Hauptsitzung vermutet apply_moving_boundaries, unbelegt);
DDF-Summenauflösung 20/32768 am Gerät; Slot 279 unter B1; Kugel-Budgetzahlen H1/H2 nur Größenordnung.
