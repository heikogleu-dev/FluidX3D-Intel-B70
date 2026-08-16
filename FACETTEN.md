# FACETTEN.md — Leitdokument Facetten-Wandmodell (C1b)

Stand **2026-08-16 abends** (Repo 2a8bd20). Dieses Dokument ist der Einstieg; die acht
FACETTEN-*-Altdateien sind Archiv (Belegkette, Lesekarte in Abschnitt 5 und FACETTEN-ARCHIV.md).
Bei Widersprüchen zwischen Altdateien gilt die jüngste Messung — die Auflösungen stehen hier.

---

## 1. Architektur, die GILT

### 1.1 Facettenbau (Host, `baue_facetten()` in setup.cpp, nach set_bcs/Void-Fill)

- **Stützpunkte = Halfway-BB-Linkmittelpunkte** (alle geschnittenen Fluid→Solid-Links, 18
  Richtungen) im **3³-Fenster** — das ist die effektive Wand des Lösers, nicht die STL.
  Fenster-A/B (3³/5³/7³ am Fahrzeug 8 mm) hat 3³ geeicht: Orientierungskonflikte 2,7 % vs.
  5,4/9,0 %, y_w-Median 0,500 (7³ driftet auf 0,617).
- **TLS-Fit**: Kovarianz + kleinster Eigenvektor (3×3-Jacobi, double, Host); y_w = n̂·(x−c),
  Orientierung per Solidnachbar-Gegenprobe; Normalen-Glättung flächengewichtet (eigene_links).
- **Konditionsklassen** (markiert = reiner BB bleibt): K1 N<6 · K2 Kante **r21 > 0,15**
  (geeicht: saubere Population endet bei q80 = 0,10) · K3 Linie r10 < 0,02 (leer, Sicherheitsnetz)
  · K4 y_w ∉ [yw_min; 2,0], Default yw_min = 0,2 (Fahrzeug: Spalt-/Eckklasse) · Orientierungs-
  konflikt · Klasse 16 Glättungs-Kipp · **Klasse 64 = MS-Ausschluss** (Nachbarn bewegter Wände,
  host-seitig über u≠0 der Solidzelle erkannt — TYPE_MS entsteht erst in initialize()).
  Fahrzeug 8 mm: 21,9 % markiert (unter der 30-%-Planschwelle).
- **z_per-Parameter**: z-periodischer Bau für den Kipp-Torus; Default false ⇒ Kanal/Fahrzeug
  bitidentisch. **Saum-Fix (2a8bd20, IR3-Audit HOCH):** Geometrie UNGEWICKELT rechnen, Wrap nur
  im Speicherindex — der gewickelte z-Saum hatte in ALLEN bisherigen Torus-Läufen BB-Löcher
  (Stützpunkte ±Nz daneben) erzeugt; Census nach dem Fix: 0 markierte Zellen.
- Daten: `fac_geo` 8-float-AoS je Facette (n̂, y_w, faca = 1/|n̂_a|, achse), `fac_idx`
  uint-Indexfeld über die F-BBox (0xFFFFFFFF = keine Facette), Bindung nachträglich per
  set_parameters; harter Fehler in run() ohne alloc_facetten.

### 1.2 Kernel (gemeinsamer Kopf, zwei Mechanismen)

Gemeinsam (beide Arme, Block hinter dem WANDFUNKTION-Platz, nur fhn-Register, nie fi):
Abtastung VOR Korrektur (calculate_rho_u), Tangentialprojektion, Spalding mit
Y = ut·(2y_w)·def_fac_Y, τ-Klemme tw_max = 0,5·ρ·ut, twe = fmin(tw·faca, tw_max)
(R2-Flächenfaktor macht das τ-Integral über die Monolage exakt).

**Paartausch (Arme 1/2) — KONTROLLARM.** Dominante-Achse-Tausch mit linkweisem Paar-Gate (R1:
Paar nur wenn BEIDE Streaming-Ursprünge solid) ± ½τ auf getauschte Paare; Paartabelle aller
6 Wandseiten in FACETTEN-STUFE2.md B. An Krümmung strukturell unzureichend (Kugel: 313 Tausch-
von ~14.669 Facettenzellen), am parallelen Kanal exakt.

**iMEM 3×3 (Arme 3/4) — HAUPTPFAD** (Asmuth 2021 Gl. 20–28, eigene D3Q19-/Teillinkmengen-
Erweiterung, FACETTEN-IMEM.md (1)–(15) + FACETTEN-IMEM-3X3.md (16)–(28)):
Slip-Additivterm q_i = 6w_i(c_i·u_s) auf der Linkmenge L (jeder Link mit solidem Ursprung);
Momente G/Sn/Snn im entrollten Loop.
- **Entkopplungs-Gate** (εc = 1e-3): Sn≈0 ⇒ textuell der 2×2-Pfad (Bitgleichheit am Kanal).
- **Schur-Elimination** der Normalzeile (RHS_n = 0): Normal-Nullung exakt, BB-Normalaustausch
  bleibt unangetastet ⇒ Cd-Druckpfad konstruktiv geschützt. (Messung I2 hat entschieden:
  Asmuths Normalinversion war nötig — die 2×2-„messen statt vorschreiben"-Festlegung ist
  überholt.)
- **Rang-Kaskade** auf G̃: Vollrang → Rang 2 (Priorität: 1. Normal-Nullung, 2. τ-Ziel t₁,
  3. Quer-Nullung) → Rang 0 gekoppelt (u. a. Einzellink mit c_n≠0) = **BB belassen + zählen**.
- **Klemmen**: s₁ ±2ut, s₂ ±ut (Slot 10); s_n aus den geklemmten s₁/s₂ NEU berechnet, dann
  ±ut (Slot 16). Kein Massen-Zwangsausgleich (Δm wird gemessen, Slot [4] des Akkumulators).
- **PEMA** (optional, CFD_FAC_PEMA): beidseitige EINGANGS-Filterung P̄/ū (Asmuth Gl. 29/30
  filtern den Eingang, nie die Lösung); utb<1e-6-Fallback = BB + Slot 17 (kein stiller
  Instantan-Modus). Die Lösungs-EMA (CFD_FAC_EMA) ist messtechnisch **widerlegt**.

### 1.3 Cd/Cz-Auslesepfad — HYBRID (FACETTEN-CD-PFAD.md E1–E7)

Impulsaustausch (`object_force`) ist an behandelten Links ungültig (Phantom, dokumentiert +1,48
Cd an der Kugel). Stattdessen `kraft_facetten()`: **Druck solid-seitig** = (F·n̂)n̂ an
kontaminierten Solidzellen (Kontamination = ≥1 18er-Nachbar mit fac_tau_n>0; n̂ = normiertes
Nachbar-Facettenmittel, |Σn̂|<0,5 → voller F + n_unklar), voller F sonst; **Reibung
fluid-seitig** aus dem Akkumulator als exaktes Fenster-Delta; Druck als **Zeitmittel** an der
Sample-Kadenz (CFD_FAC_CD_EVERY, cd_facetten.csv) — die End-Momentaufnahme war wertlos
(0,164 vs. Zeitmittel 0,837 an der Kugel).

### 1.4 Diagnose-Instrumente

- **Akkumulator `fac_tau`, 6 float je Facette** (nur bei tatsächlicher Modifikation):
  [0] Σ tw physisch (y⁺-Quelle) · [1..3] Ist-Wandkraft xyz (inkl. Sn-Terme) ·
  [4] Δm-Leck · [5] Normalaustausch-REST (nach 3×3-Nullung); dazu `fac_tau_n` (Zähler).
- **Zählerpuffer `rho_clamp_hits`, 18 Slots (0–17):** [0/1] RHO_CLAMP unten/oben ·
  [2] WFB-Wirkpfad · [3] WFB-τ-Klemme · [4] WFB-u_t≈0-Skip · [5] Ein-Zellen-Spalt ·
  [6] SGS_WANDFREI-Wirkpfad · [7] Facetten-Wirkpfad (Soll = fac_N·⌈n/100⌉, Ist≠Soll = harter
  Fehler) · [8] Facetten-τ-Klemme · [9] Facetten-u_t≈0-Skip · [10] u_s-Klemme (iMEM) ·
  [11] ohne offenes Paar (nur Paararm) · [12] iMEM-Skalar-Fallback · [13] ohne tangential
  wirksamen Link · [14] gekoppelter Rang 2 · [15] gekoppelt Rang 0 → BB · [16] s_n-Klemme ·
  [17] PEMA-utb-Fallback. Wirkpfad-/Ereignis-Slots gegatet t%100. Gate-Randzone: Lage-1-Zellen
  können per Float-Rauschen in 15 statt 13 landen — Soll-Formeln prüfen die **Summe 13+15**.
- **CFD_FAC_DIAGZ** (Iron Rule 3): Ketten-Zeitreihe EINER Facette (Zellindex → Laufzeit-fid,
  Sentinel −1.0f matcht nie) → `facetten_diagz.csv` mit
  schritt,t_kernel_check,ut,twe,P1,P2,s1,s2,sn,phi1,phi2,G11,G22,Snn,Sn1,Sn2,t_kernel,rhon.
  Nur iMEM-Arme; Kanal/Torus verdrahtet, Kugel noch nicht (Warnung).
- **Host-Census** (CFD_FACETTEN_DIAG / baue_facetten-Log): Klassen, y_w-Lagen, |L|, Momente —
  liefert die exakten Slot-Solls VOR jedem Kernellauf.

---

## 2. Schalter-Referenz

| Schalter | Semantik (je 1 Zeile) + Gültigkeitsregeln |
|---|---|
| `CFD_FACETTEN` | 0 aus (Kontrollarm bitgleich) · 1/2 Paartausch voll/NUR-Tausch (τ=0) · 3/4 iMEM voll/NULLZIEL. >4 = Fehler; mit CFD_WANDFUNKTION, D≠3Q19 oder D>1 = harter Fehler; alloc_facetten Pflicht. |
| `CFD_FACETTEN_FENSTER` | Fit-Fensterradius 1..3 (3³/5³/7³); Default 1 = 3³ (geeicht — nicht ohne Neueichung ändern); 0→1 gehoben, >3 geklemmt (mit Warnung). |
| `CFD_FACETTEN_YWMIN` | K4-Untergrenze, Default 0,2 (Fahrzeug-Eichung). ≠0,2 = deklarierter Messarm (Warnung). kipp=26 VERLANGT <0,187 (harter Fehler — tragende m0-Lage liegt bei y_w=0,187). |
| `CFD_FACETTEN_DIAG` | 1 = Facettenbau-Census loggen; 2 = Census + _exit(0) (Schritt-0-Diagnose ohne Lauf). |
| `CFD_FAC_EMA` | Lösungs-EMA auf u_s (nur Arme ≥3, emission-gated). **WIDERLEGT** (J3: verschlechtert, filtert die falsche Seite) — bleibt nur für A/B-Vergleiche. |
| `CFD_FAC_PEMA` | Beidseitige Eingangs-Filterung P̄/ū, a≈0,01 (nur Arme ≥3, emission-gated; ungesetzt = exakt instantaner Pfad). N2 dennoch verletzt — kein hinreichender Fix, Messarm. |
| `CFD_FAC_DIAGZ` | Zellindex n der Diagnose-Facette → facetten_diagz.csv (nur Arme ≥3; keine aktive Facette an n = hart AUS mit Warnung; Kugel unverdrahtet). |
| `CFD_KANAL_KIPP` | 0 parallel (wortgleich Alt-Pfad) · 45 · 26 (= 26,565°): y-periodischer Torus-Slab. Nur mit CFD_FACETTEN; mit CFD_WANDFUNKTION = harter Fehler. |
| `CFD_KANAL_PHASE` | Störphase des Kanal-Inits für Rauschboden-/Wiederholbarkeitsmessungen; 0 = bitidentisch zum Alt-Init. |
| `CFD_FAC_CD_EVERY` | Kadenz der Druck-Projektions-Samples im Cd-Pfad (Default 1 = jede object_force-Kadenz). |
| `CFD_SPALDING_IT` | Spalding-Newton-Iterationen, Default 3 (min 1); in WFB- UND Facetten-Arm identisch emittiert. |
| `CFD_FAC_K4` | Kugel, nur AUS-Arm: K4-Neutralitätsprüfung kraft_facetten vs. object_force. |

---

## 3. Bestandene Abnahmen (mit Zahl)

- **Kanal-Anker EXAKT** (Stufe 1, CPU N=38): alle Klassen 0, y_w überall 0,500.
- **Stufe-2-Äquivalenz BITGLEICH** (Paartausch vs. z-WFB): Feld-Hash(u) CPU/316 Schritte
  **12755646098055097704** (bleibt Kontrollarm-Regressionsanker); iGPU/12.666 Schritte
  7540097450125369907; Wirkpfad 1.798.320 = Soll exakt; R1 auf DDF-Ebene bewiesen (T2:
  512 Differenzen, 0 verbotene).
- **J2 iMEM-3×3-Regression BITGLEICH**: entkoppelter Pfad == 2×2 (Hash 8879…785, voller Wert
  im Commit-Log 9776554); Kontrollarm 1275…704 unverändert; Tests 5× grün.
- **Cd-Pfad K1–K5**: K1 Hash exakt · K2 Reibung/Kraftbilanz **0,9961** (Band 1 %, stationäres
  20-ETT-Fenster) · K3 Druck_x exakt 0,0 (14.160 projiziert / 0 voll / 0 unklar) · K4 AUS-Arm
  rel. Abw. 1,5e-7 · K5 Phantom beseitigt: alt 1,32→2,80 (+1,48), neu Arm 2 = 0,8359 /
  Arm 1 = 0,8375 (Δ +0,002 im Band), Arm-2-Reibung exakt 0.
- **I1 iMEM-Kanal-Äquivalenz** (iGPU N=38, 2 Phasen, 80 ETT): c_f-Mittel 0,001034 (Paar) vs.
  0,001046 (iMEM) = +1,2 % IM Rauschboden (Paararm-Spreizung 3,2 %); U_b⁺ 24,10–24,11;
  Δm über 507k Schritte exakt 0; Wirkpfad 71.748.720 exakt; Arm 4: c_f → 0.
- **N1-Mechanik (J3)**: Normalkontamination 26,6° −763.625 → **−0,16**; 45°+EMA −0,00096;
  s_n-Klemme 0. **Vorbehalt:** der N1-Report las bis 2a8bd20 die Warmup-Kopie (falsches
  Fenster) UND alle Torus-Läufe trugen den z-Saum-Geometriefehler — Saum-fixe Wiederholung
  läuft (s. Abschnitt 4).
- **Saum-Fix verifiziert (Bau-Ebene)**: Census nach dem Fix 0 markierte Zellen; Kanal-Anker
  und Kontrollarm-Hash unverändert.
- **PEMA-Diskriminator**: 3×3-Normal-Nullung hält auch unter PEMA (Kontamination 0,0011);
  Klemmsaum ~6 % bleibt aktiver Drag-Kanal.

**Nicht bestanden (der ehrliche Kern):** Torus-**N2** (Arm 4 ≤ BB-Basis) mit drei Varianten
verletzt — instantan 0,0040/0,0287 (45°/26,6°), 3×3 0,0053/0,0118, PEMA 0,0062 gegen BB-Basen
0,00166/0,00179 (F11-Erwartung 0,010–0,014 damit widerlegt; Kanal sitzt gekippt im
nicht-selbsterhaltenen Grenzregime). Arm 3 == Arm 4 im c_f (Blindheit: der Torus misst derzeit
den Fehler, nicht das Modell). Mechanik dahinter: Klemm-Rektifikation der P′-Fluktuationen +
ν_t-Rückkopplung (fneq auf modifizierten Registern). ABER: alle diese Zahlen stammen aus
Läufen MIT Saum-BB-Löchern — Neubewertung nach der laufenden Wiederholung.

---

## 4. Offene Punkte (Stand 2026-08-16 abends)

1. **Torus-N2-Wiederholung nach Saum-Fix läuft** (45°/26,6°, Arme 4/3; N1-Zahlen werden mit
   korrektem Fenster neu erhoben).
2. **Familienfrage SUSPENDIERT bis zur Saum-fixen Messung**: Kompensationsfamilie (iMEM
   instantan/EMA/PEMA) vs. Ersetzungsansatz (PowerFLOW-artige BB/Specular-Gewichtung) vs.
   Richterwechsel Kugel. Davor noch der billige **Klemmskalen-Messarm** (±4ut): fällt c_f
   damit auf BB-Basis, war es der Klemmsaum; bleibt es, ist es P′-Arbeit = Modellgrenze.
3. **Kugel J4**: Census (|L|-Verteilung, Rang-0-Anteil, Kopplungswinkel), Arme AUS/4/3, beide
   Cd-Wege, Δρ̄-Drift-Band, FP32-Sprosse (FP16C). N3/F12-Band offiziell hierher verschoben.
4. **Hangauf-Arm I2b** (45°-Torus, Antrieb (0,1,1)/√2): misst die R2-Lücke — nur unter iMEM
   möglich; Umbauliste steht (IMEM-Revision Auflage 5).
5. **Fahrzeug Stufe 5** (Serie ≥6 B70-Läufe à ≤2,5 h): y⁺-Median 1122 → ~140 aus dem
   Akkumulator; Arme AUS/4/3; Δm-Wächter; Cd/Cz mit C7-Notiz davor.
6. **Niedrige Audit-Reste (IR3)**: Statik-Symmetrie diagz (s_fac_diagz nicht an allen
   Konstruktorstellen) · Kugel-iMEM-Report-Slots fehlen · cf_m-Normierung am Torus ·
   fac_diag-float-Grenzen (fid-Vergleich als float) · y⁺-Report rechnet am Torus hartkodiert
   mit y_w = 0,5.
7. K2-Instrument: 1-%-Band gilt nur für hinreichend stationäre Fenster (ETT=80/WARM=20 lief
   auf 1,035–1,042 in BEIDEN Mechanismen) — Gate vor Torus-Abnahmen nachziehen.

---

## 5. Lesekarte: welche Archivdatei wofür

| Datei | Als Beleg lesen für |
|---|---|
| FACETTEN-PLAN.md | Architektur-Entscheidungen A1–A8, Revision R1/R2 + 9 Auflagen, Stufe-1-Messstände, Fenster-/Schwellen-Eichung |
| FACETTEN-STUFE2.md | **Paartabelle aller 6 Wandseiten** (Tabelle B), Kernel-Einbau, Bitgleichheits-Messstand, T1/T2 |
| FACETTEN-STUFE3.md | **Torus-Geometrie/Normierung F1–F6** (gilt mechanismus-unabhängig), F12-Band, Lagenstruktur; F7–F10 nur Paararm |
| FACETTEN-CD-PFAD.md | Cd-Hybrid-Entscheidungen E1–E7, K1–K5-Messstand inkl. Druck-Zeitmittel |
| FACETTEN-LITERATUR.md | Quellen (Asmuth/iMEM, PowerFLOW-Surfel, Matyka, Kugel-Referenzen Achenbach/Tsutsui) — weiter voll gültig |
| FACETTEN-IMEM.md | iMEM-Herleitung (1)–(15), Gegenprüfungs-Auflagen, I0/I1-Messstand, I2-Durchfall (Diagnose Normalkontamination) |
| FACETTEN-IMEM-3X3.md | 3×3-Formeln (16)–(28), Entkopplungsbeweise je Torus-Lage, N1–N3-Kriterien, J0-Loganalyse, J3-Messstand |
| FACETTEN-IMEM-ANALYSE.md | Asmuth-Exegese (Eingangs- vs. Lösungsfilterung), ν_t-Rückkopplungsschleife, Wegbewertung A/B/C, PEMA-Messstand + Familienfrage |

---

# Nachtrag 2026-08-16 spätabends: Saum-fixe N2-Wiederholung — WENDE

| 45°-Lauf (Saum-fix, 80 ETT) | cf | Urteil |
|---|---|---|
| Arm 4 pur (instantan!) | **0,000847** | **N2 BESTANDEN** (≤ BB-Basis 0,00166) |
| Arm 4 + PEMA | 0,00119 | N2 bestanden |
| Arm 3 + PEMA | 0,00623 | Arme sind GETRENNT — das Ziel ist erstmals messbar |

N1 auf korrektem Fenster: Δm = −0,0000366, Normalkontamination 0,00051 — praktisch perfekt.

**Auflösung der Familienfrage: die iMEM-Kompensation IST tragfähig.** Der komplette N2-Durchfall
(drei Varianten) war der z-Saum-Geometriefehler: die BB-Löcher im Wandband säten die Fluktuationen,
die das Nullziel dann jagte und verstärkte. Mit sauberem Band besteht sogar der PURE instantane
Arm 4 — PEMA ist möglicherweise gar nicht nötig (Arm 4 pur schlägt Arm 4+PEMA).

**Arm 3 = 0,0062 über dem F12-Band ist KEIN Mechanikfehler**, sondern konsistente Modellantwort im
nicht-selbsterhaltenen Torus-Regime (Wandzelle trägt ~volles U_b → Spalding-τ entsprechend groß —
die −68-%-Baustelle von der anderen Seite). Genau der N3-Vorbehalt des Plans: die WERT-Frage
gehört zum Richter mit eigenerhaltener Turbulenz → **Kugel J4 ist der nächste Messpunkt.**
