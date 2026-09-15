# Erhaltende Klemmen, Stufe 1: Positivitätsbegrenzer (`CFD_POSITIV`) — Plan des Planungsagenten, 15.09.2026

Stand HEAD 086d55b (nur gelesen; S0a lag unfertig im Arbeitsbaum). Heiko 15.09.: „erstmal alles an 8 mm testen und die
erhaltenden Klemmen als Schalter implementieren“. Aufsetzend auf KLEMMEN-STUFE0-PLAN.md. **Entscheidungen E1–E9 offen (§8).**

## 0 Kurzfazit

- **Vorlagenformel f** = f_eq + s·g reicht nicht:** erhält Masse/Impuls nur ohne Klemmtreffer und bei F = 0 — genau an den
  Klemmtreffern nicht erhaltend (8 mm Nahfeld 3 176 331 ρ-Treffer, logs/rr_c2d_dd8_a_b70.log:2152).
- **Empfehlung Projektionsform:** aus dem Nichtgleichgewicht vor dem Skalieren den Anteil mit 0./1. Moment herausnehmen → Σf** = Σf*
  und Σc·f** = Σc·f* für jedes s, unabhängig von w, SGS, SPONGE, P-TRT, Guo und den Klemmen. Stufe-0-Formeln bleiben im an-Arm gültig.
- **Ort:** direkt nach der P-TRT-Korrektur im Nicht-TYPE_E-Zweig (kernel.cpp:3769/3770), vor store_f (:3833).
- **Drei Modi:** 0 nichts emittiert (bitgleich); 1 Messarm (s berechnet und gezählt, nicht angewandt; Felder bitgleich); 2 anwenden.
- **Slots 271–294**, Puffer 288 reicht nicht → **320** (+128 B je Domäne).
- **Vier Berichtigungen an Übergabe §1.3:** (1) Versatz f − w_i ist DDF-Shifting in allen Formaten, nicht FP16S; (2) nach der Kollision
  trägt f_neq das erste Moment **+F/2** (−F/2 gilt vorher), Produktion F ≡ 0; (3) Wandmodell-Ist=Soll (Slots 81–91) sitzt vor der
  Kollision und sieht den Begrenzer nicht; (4) Vorlagenformel lässt den P-TRT-Term aus.
- **Ob die alten Klemmzähler sinken, ist nicht logisch zwingend** (§5). Belastbar misst 8 mm vor allem den deterministischen Messarm.

## 1 Ort und Größe

### 1.1 Kollisionspfad

| Baustein | Stelle | Wirkung auf f* |
|---|---|---|
| Wandmodell-Injektion (iMEM/ELIBB) | kernel.cpp:2990-3015, vor calculate_rho_u | ändert fhn vor der Kollision, steckt in f̂ − f̂eq |
| MOVING_BOUNDARIES | :2976 | wie oben (TYPE_MS) |
| RHO_CLAMP | :3106 → :1257 | ρ_c ≠ ρ_roh |
| u-Klemme mit Guo | :3184-3191 | u_eq geklemmt |
| SGS/FDWAND/SISM, SPONGE | :3287-3472, :3589-3590 | nur w |
| P-TRT | :3679-3702, Addition :3761-3768 | d_i ohne 0./1. Moment (:3670-3671) |
| SRT + Guo | :3738-3739, :3760 | f* = (1−w)f̂ + w·f̂eq + (1−w/2)F_i |
| TYPE_E | :3757-3758 | f = REG_E = feq[i] (CFD_REG_BC nicht gesetzt, lbm.cpp:2146-2150) |

TRT ist nicht gebaut (defines.hpp:19) → Emission sperren (Muster lbm.cpp:1967-1969).

### 1.2 Begrenzte Größe

f̂ = Rechenform f − w_i, f* = nach Kollision und P-TRT.
- g_i = f*_i − f̂eq_i = (1−w)(f̂_i − f̂eq_i) + (1−w/2)F_i + d_i.
- Momente (Σf̂eq = ρ_c − 1, :1168/:1177; ΣF_i = 0, Σc_iF_i = F, :1272-1275; Σd = Σc·d = 0):
  m0 := Σg = (1−w)(ρ_roh − ρ_c) (≠ 0 nur bei RHO_CLAMP-Treffer); **m** := Σc·g = (1−w)(j − ρ_c·u_eq) + (1−w/2)**F**;
  ohne u-Klemme u_eq = (j + F/2)/ρ_c → **m = +F/2**; mit u-Klemme zusätzlich (1−w)ρ_c(u_roh − u_c).
- Vorlagenform f** = f̂eq + s·g: Δm = (s−1)·m0, Δj = (s−1)·**m**; Guo-Rest (1−s)·F/2; verschiebt die Stufe-0-Buchung an Zellen mit beidem.
- **Projektionsform:** G_i = g_i − w_i·(m0 + 3·c_i·**m**) mit ΣG = 0, Σc·G = 0 (Σw_i = 1, Σw_i c_i = 0, Σ3w_i c_i c_i = I);
  f**_i = f*_i − (1−s)·G_i. Exakt erhaltend bis float-Rundung, kein Guo-Rest.
- F in der Produktion ≡ 0: fahrzeug_dd ohne Kraft (setup.cpp:6434, lbm.hpp:990), F an Fluidzellen ungelesen (kernel.cpp:3124-3135),
  FACETTEN_KRAFT = 0 (lbm.cpp:450, setup.cpp:3940). F ≠ 0 nur im Kanal (setup.cpp:3990, :4105).

### 1.3 Positivität und s

- Basis B_i = f*_i + w_i − G_i = f̂eq_i + w_i + w_i(m0 + 3c_i·**m**). Bedingung B_i + s·G_i ≥ τ_i (Rundungsreserve §2a).
- s = min über i mit f*_i + w_i < τ_i von (B_i − τ_i)/(−G_i). Bei B_i ≥ τ_i ist −G_i > 0 → keine Division durch 0, s < 1 strikt.
  Ohne Klemme und F = 0 ist B_i = f_eq,i.

### 1.4 f_eq selbst negativ (machtlos)

Es gibt i mit B_i < τ_i (ohne Klemme |u|² > 1/3, kernel.cpp:1198-1206; Komponentenklemme lässt bis 1,0 zu). **Empfehlung:** f*
unverändert (s = 1) und zählen (Slot 278; „f̂eq_i + w_i < 0“ in 279). u-Klemme bleibt einzige Sicherung (Stufe 2). s = 0 nicht
empfohlen (löscht f_neq, bleibt negativ).

## 2 Die vier Fallen am Code

**(a) Versatz:** nicht FP16S-spezifisch — calculate_rho_u addiert 1,0 zuletzt (:1229), calculate_f_eq rechnet mit rho−1 (:1168, :1177);
FP16S skaliert zusätzlich mit 2^15 (lbm.cpp:2241-2242), stream_collide rechnet float32. Bedingung `fhn[i] < τ_i − w_i`
(def_w0 = 1/3, def_ws = 1/18, def_we = 1/36; lbm.cpp:2020-2022). **τ_i = halbe ULP des Halbworts (Rechnung):** vstore_half_rte rundet
auf nächste Stufe; nahe f = 0 Betrag w_i·2^15: i=0 10 922,7 → ULP 8 → τ0 = 4/32768; Achsen 1820,4 → ULP 1 → τs = 0,5/32768;
Diagonalen 910,2 → ULP 0,5 → τe = 0,25/32768. Nach Speichern f ≥ 0 garantiert. Konstanten als Ausdrücke emittieren
(`(4.0f/32768.0f-def_w0)`), nicht über to_string (lbm.cpp:2196, :2217). FP32: τ = 0. FP16C: sperren.

**(b) Facettenzellen:** apply_facette_imem schreibt fhn vor calculate_rho_u (:2993-3015) → Injektion steckt in g. Projektion erhält den
injizierten Zellimpuls, skaliert aber Π_neq (Wandschub-Weitergabe). Vorlage irrt: Zielerfüllung (81–91) wird vor der Kollision gebucht
(:2698-2724), blind für den Begrenzer; sichtbar nur integral in forces.csv/cd_facetten.csv. Maske: fdw_fid (SGS_FDWAND) ist exakt die
Injektionsmaske (:3298 gegen :2993) — nicht bis nach der Kollision leben lassen (Registerdruck), im Trefferzweig neu nachschlagen wie
Stufe 0 K0. **Empfehlung: K0 in Modus 2 ausnehmen**, Modus 1 zählt „würde begrenzen“ (Slot 273).

**(c) Guo:** nach der Kollision m = +F/2. Projektion: kein Rest. Vorlagenform: Rest (1−s)·F/2, Produktion 0.

**(d) Keine 19er-Felder:** du[9]/fneq_reg[19] ließen IGC hängen (:3612-3616), zwei weitere (:3660-3662), 19-fach-Makro fror den Rechner
ein (lbm.cpp:2127-2130). Spill: j[] rematerialisiert (:3825-3833) → nicht benutzen. Scratch: c()/w() in Schleifen werden private Felder
(scratch_gate.sh:3-16); Rate `w` verdeckt w(i) (:3640, :3657-3659). Form ohne Feld nach P-TRT-Muster (:3680-3702):

```c
// Immerpfad (Modus 1/2), nach P-TRT, Nicht-TYPE_E:
const int pos_k = (fhn[0]<def_pos_g0)|(fhn[1]<def_pos_gs)|/*...*/|(fhn[18]<def_pos_ge); // def_pos_gX = tau_X - w_X
if(pos_k) {  // selten
  const float pn0 = /* Summe fhn[i]-feq[i] ausgeschrieben */, pmx, pmy, pmz; // Muster :1239-1241
  // Pass 1: G1 = (fhn[1]-feq[1]) - def_ws*(pn0+3.0f*pmx); G13 = (fhn[13]-feq[13]) - def_we*(pn0+3.0f*(pmx-pmy));
  //         B = fhn[i]+w_i-G; machtlos |= (B<tau_i); if(fhn[i]<tau_i-w_i) ps = fmin(ps,(B-tau_i)/(-G));
  // Pass 2 (nur Modus 2, !machtlos, Klasse erlaubt): G neu rechnen, fhn[i] -= (1.0f-ps)*G;
}
```
Zwei Pässe mit neu gerechnetem G_i (jedes i hängt nur von fhn[i], feq[i], vier Skalaren ab). c_i für D3Q19 aus :1030-1032.

## 3 Wechselwirkungen

- **RHO_RAND** (Standard): rho/u werden vor der Kollision geschrieben (:3207-3213, :3277); Begrenzer ändert weder Ort noch Wert;
  TYPE_E-Lesepfad unberührt; rho_rek/rho_ausgabe summieren f (:4460-4468), Projektion erhält Σf.
- **RHO_SPARSAM/U_SPARSAM:** Masken nur koordinatenabhängig, keine Wirkung.
- **EP-Streaming:** rein lokal auf fhn vor store_f, bitreproduzierbar.
- **Fernfeld:** SPONGE nur dort; PTRT beide Domänen; Fernfeld 8 mm RHO_CLAMP 62 515, u-Klemme 0; K0 leer. **Empfehlung: ein Schalter
  für beide Domänen** über die Statik vor jedem Konstruktor (Muster s_rho_rand), Bericht Nah/Fern getrennt.
- **boden_eq/einlass_eq** laufen danach und setzen f_eq mit lokalem ρ (:4051-4054, :4069-4072) — überschreiben die Begrenzerwirkung
  in ihren Bändern.

## 4 Zähler (Puffer 320, nächster freier Slot danach 295)

| Slot | Inhalt | Art | Soll |
|---|---|---|---|
| 271 | Besuche am Prüfpunkt, ein Schritt t == def_zaehl_takt+2 | sätt. | dd: = [204]+[205]; Kugel: Host-Flagzählung |
| 272 | Kandidaten: ein f*_i + w_i < τ_i | sätt., ungegatet | = [278] + Σ[273..277] |
| 273–277 | s < 1 je Klasse K0..K4 (Stufe-0-Klassen) | sätt. | Modus 1 „würde“; K0 unter Ausnahme nicht angewandt |
| 278 | machtlos: ein B_i < τ_i | sätt. | – |
| 279 | davon f̂eq_i + w_i < 0 | sätt. | ≤ [278] |
| 280–284 | s-Eimer [0;0,25) [0,25;0,5) [0,5;0,75) [0,75;0,95) [0,95;1) | sätt. | Σ = Σ[273..277] |
| 285 | nach load_f negativ (vor MB/Facette, Nicht-E; :2973) | sätt. | Istmaß über zwei Kernelstarts |
| 286 | Kandidat und ρ-Klemme gleiche Zelle/Schritt | sätt. | – |
| 287 | Kandidat und u-Klemme | sätt. | – |
| 288 | H1: Hakenzellen im Eimer [0,25;0,5) | sätt. | = N_H (Host) |
| 289 | Nachladeprobe t == def_zaehl_takt+3: Zellen mit negativer geladener Population | sätt. | H1 Modus 1: ohne Haken + 2·N_H; Modus 2: = ohne Haken |
| 290 | Haken: Selbstprüfung Σ(f**−f*), Σc(f**−f*) über Toleranz | sätt. | 0 |
| 291 | TYPE_E mit negativem f_eq | sätt. | – |
| 292/293 | Σq(1−s), Σq(Σ\|Δf_i\|) angewandt (Festkomma) | mod 2³² | Host-Differenzen |
| 294 | Kappung zu 293 | sätt. | 0 |

292/293 in die Wickelwächter-Ausnahme. Lesen im Sample-Takt über den S0c-Leser.
Testhaken `CFD_POSITIV_HAKEN`: **H1** (nur Kugel) bei t == def_zaehl_takt+2 an K4-Zellen mit n%P == 0: nach Kollision, vor Begrenzer
`fhn[0] += 2a; fhn[1] -= a; fhn[2] -= a`, a = 2,5·def_ws (masse-/impulsfrei) → s ≈ (1 − 3|u_x|)/2,5 ∈ [0,27; 0,40] → Eimer 281;
Nachladeprobe 289 exakt. **H2:** τ_i → 1,2·w_i → 278 und 273..277 feuern sicher. **H3:** Klassenzählung bei n%7 == 0 übersprungen,
genau eine Beanstandung. No-Op-Wächter: CFD_POSITIV > 0 und [271] = 0 → Fehler (Muster pruefe_ptrt setup.cpp:4468).

## 5 Wirkung auf die alten Klemmen

**Nicht zwingend.** RHO_CLAMP und u-Klemme wirken vor der Kollision; ihre Wirkung steckt schon in f*. Die Projektion erhält Σf*/Σc·f*
— nimmt die Klemmwirkung desselben Schritts weder zurück noch verstärkt sie; Stufe-0-Formeln bleiben im B-Arm exakt. Auf spätere Klemmen
nur über Transport (Umverteilung in der Zelle, Dämpfung Π_neq). Weniger Treffer plausibel, wenn Treffer mit starkem Nichtgleichgewicht
koinzidieren; mehr nicht ausgeschlossen.
- **8 mm kann:** Modus 1 (bitgleich, deterministisch): Rate/Ort negativer Nachkollisions-Populationen, s-Verteilung, machtlos-Anteil,
  Koinzidenz 286/([0]+[1]) und 287/[28]. Modus 2 gegen 1: Faktoränderung [0]/[1]/Stufe-0-Summen, cd_rest/cz_rest gegen σ (8 mm 0,0546),
  Wanduhr.
- **8 mm kann nicht:** Kraftänderungen unter ~2σ; kleine Klemmzähler-Änderungen (ein chaotischer Verlauf je Arm); u-Klemmen-Frage
  (1872 Treffer); 4-mm-Verhalten.

## 6 Kosten (Rechnung, nicht gemessen)

VRAM 4 mm 0 MB (+128 B je Domäne). Immerpfad: 19 FP-Vergleiche, 18 OR, 1 Sprung ≈ 38 Instruktionen; Nachlade-Zähler 285 ≈ 38; 0 B
Speicherverkehr. Trefferzweig ≈ 350–400 Operationen + ≈ 26 B; Trefferrate unbekannt (Modus 1 liefert sie). Wanduhr über
[LEISTUNG-AB-MARKE] messen.

## 7 Prüfkette

1. Scratch-Gate: `gen datei` um Zusatzdefines erweitern; Arme prod8nah/prod8fern × {POSITIV, +ANWENDEN, +HAKEN}; Soll private 0 / spill 0.
2. Prüfagent gegen den Diff, committen.
3. Leiter CPU → iGPU → B70, Kugel wie rr_c2c3: ohne Schalter Hash/forces = S0-Stand; Modus 1 bitgleich + Ist=Soll; Modus 1 mit H1/H2/H3;
   Modus 2 ohne Haken (Σ[273..277] = 0 → Hash = Modus 0); Modus 2 mit H1 ([289] = ohne Haken, [290] = 0).
4. 8 mm dd B70 (Zeile logs/rr_c2d.txt:5, RHO_RAND Standard): A1 ohne Schalter bitgleich zur S0-Referenz; M1 CFD_POSITIV=1 bitgleich zu A1
   + Raten/Klassen/Koinzidenz/Wanduhr; M2 CFD_POSITIV=2 (K0 ausgenommen): cd_rest/cz_rest gegen σ, Stufe-0-Bilanz, [0]/[1]/[28], Wanduhr.
   A1 zweimal oder HEAD-Doppelmessung aus S0.

## 8 Fallen, Commits, offene Entscheidungen

**Fallen:** Block inline ohne neue Signatur (sonst Splice-Muster `)+"("+R(`); `//` mitten in Zeilen; `#ifdef` nur als String-Splice,
Defines nur über lbm.cpp-Emission; Rückleser im selben Kernel wegoptimiert (Nachweis über 285/289 im nächsten Kernelstart); `w` verdeckt
w(i); -cl-finite-math-only (NaN fällt durch Vergleiche); Statiken je Fall zurücksetzen (setup.cpp:3928, :5108, :5698, :6312, :9398).

**Commits:** P1a Host/Infrastruktur (Puffer 320, Legende, Parser CFD_POSITIV/_HAKEN/_FACETTE, Sperren TRT/nicht-D3Q19/FP16C, Wickelausnahme,
Gate-Arme; Soll: Kernelquelle bei 0 zeichengleich per CFD_DUMP_CL-Diff, Gate sauber, Kugel-CPU-Hash = S0). P1b Messarm Modus 1 (Slots
271–291, H1–H3, Ist=Soll, No-Op-Wächter). P1c Anwenden Modus 2 (τ, Facettenschalter, 290/292–294). P1d Bericht (S0c-Leser, positiv.csv,
Phasen, Nah/Fern, Koinzidenzen). Danach 8 mm A1/M1/M2. **P1b erst nach S0b/S0c.**

| Nr. | Frage | Folge | Empfehlung |
|---|---|---|---|
| E1 | Projektion oder Vorlagenform | Vorlagenform verändert an Klemmtreffern zusätzlich Masse/Impuls, entwertet Stufe-0-Buchung | Projektion |
| E2 | K0 in Modus 2 ausnehmen | Einschließen skaliert Wandspannungsübertragung, kein Wandmodell-Zähler sieht es | Ausnehmen; Modus 1 zählt „würde“ |
| E3 | Machtlos-Fall | s = 0 löscht f_neq ohne Positivität | unverändert lassen, zählen |
| E4 | Drei Modi 0/1/2 | ohne Modus 1 keine deterministische Rate, keine Bitgleich-Abnahme | ja |
| E5 | Ein Schalter beide Domänen | getrennte Schalter = zweite Variable im A/B | ja |
| E6 | Puffer 320 | mit 288 entfallen Koinzidenz, Haken-Soll, TYPE_E, Summen | 320 |
| E7 | τ = halbe FP16S-ULP | ohne τ kann gerundetes f ≈ 0 wieder negativ gespeichert werden | ja |
| E8 | Nachlade-Zähler 285 immer an | ≈ 38 Instruktionen je Zelle/Schritt | im A/B an, Wanduhr entscheidet |
| E9 | K1 (TYPE_MS) einschließen | Wirkung im BODEN_EQ-Band ohnehin überschrieben | einschließen, getrennt gezählt |

## 9 Nicht verifiziert

Natürliche Rate negativer Nachkollisions-Populationen (8 mm, Kugel), ob 272 sättigt; ob IGC mit 19 Vergleichen + Trefferzweig spill 0
hält (**Hinweis Hauptsitzung 15.09.: S0b brachte mit einer 18er-Nachbarsuche Spill 1216/864 zurück — die Stufe-1-Klasse muss die
koordinatenbasierte S0b-Klassenlogik übernehmen**); ob rhon/uxn für 286/287 Lebensdauer verlängern (kl-Bits aus S0b billiger); Bitgleichheit
Modus 1 gegen 0 unter -cl-mad-enable; τ-Werte am Gerät; ob apply_facette_imem fhn nur an Zellen mit fac_fid ≠ FFFFFFFF ändert; H1-Details
(Host-Nachbildung N_H, |u_x| ≤ 0,11); ob IGC Slot 290 zu 0 vereinfacht; negative Populationen aus Kopplungs-Lift/schale_blend; Wanduhr;
Reaktion von update_force_field auf umverteilte Populationen.

## Nachtrag P1a/P1b (Hauptsitzung 15.09.2026 abends, nach Prüfagent P1a)

* **Zähler gegatet statt ungegatet (Abweichung von §4, Absturzsperre):** Alle Zellzähler des Messarms (272–288, 291, 285) zählen nur an
  Zählschritten `t % zaehl_takt == 2` UND an Stichprobenzellen `n % def_pos_sub == 0`, `def_pos_sub = ⌈N / 6 104 700⌉` (größtes Gitter,
  an dem Atomics in fast jeder Zelle je Schritt belegt liefen, Kugel 16 mm). 8 mm Nahfeld: 11, Fernfeld: 5; Kugel ≤ 16 mm: 1. Grund: die
  natürliche Rate negativer Populationen ist ungemessen; ungegatete Atomics in vielen Zellen haben die B70 am 15.09. lahmgelegt. 271 und
  289 zählen je genau einen Schritt in allen Zellen (Last wie 204/205, belegt). Die Ist=Soll-Identitäten bleiben exakt, weil alle
  Klassenzähler dasselbe Gatter tragen. Modus 1 prüft die Kandidaten nur an Zählschritten (Hot Path ≈ 1 Vergleich); Modus 2 prüft jeden Schritt.
* **Soll-Identitäten (P1b):** [272] = [278] + Σ[273..277] (H3 verletzt genau diese); [272] = [278] + Σ[280..284]; [279] ≤ [278];
  [271] = Host-Flagzählung (Zellen ≠ TYPE_S, Flags vom Gerät); No-Op: Prüfpunkt erreicht und [271] = 0.
* **H2 berichtigt (Prüfbefund P1a M1):** τ = 1,2·w macht jede Zelle zum Kandidaten UND machtlos (B₀ < 1,2·w₀ für ρ < 1,2) → Soll
  [272] = [278] > 0, Σ[273..277] = 0. H2 prüft den Machtlos-Pfad, nicht die Klassen; K0..K3 haben keinen Positivtest (nur 8 mm M1).
* **H1-Zellen:** reine Fluidzellen (Flags & TYPE_BO = 0) außerhalb der Randschale mit n mod 1009 = 0 (und Stichprobe); an der Kugel ist die
  F-BBox die ganze Domäne, „K4“ wäre leer. Soll [288] = Host-Zählung derselben Bedingung.
* **291** = TYPE_E-Kandidaten (f_eq,i + w_i < τ_i), nicht in 272.
* **Spill:** Atomics direkt nach load_f (285/289) brachten Spill 448 auf der iGPU in allen Nahfeld-Positiv-Armen (Bisektion); dort nur ein
  Flag, gezählt hinter der Kollision → Spill 0 in allen Armen auf beiden Geräten.
* **τ (Prüfbefund P1a NIEDRIG 1):** −w_i·2¹⁵ liegt nicht auf dem Halbwort-Raster; schon τ = 0 liefert dieselben geladenen Minima. τ ist
  vorsichtig; [272] zählt f* < τ, nicht f* < 0; [285] sieht deshalb weniger als [272].
* Sperren seit P1b vor Kernelbau und Großallokation (Prüfbefund P1a NIEDRIG 2); Gate-Parser streng; Arm prod8fernp1 ergänzt.

## Messung P1b 8 mm B70 Modus 1 (kl_p1b_dd8_m1_b70, 03c8eef/8a7051c, Standardzeile 300 ms) — VORLÄUFIG bis Prüfagent P1b

forces.csv und cd_facetten.csv bitgleich zu kl_z2c_dd8_b70 (Modus 1 bitgleich). Ist=Soll in beiden Domänen erfüllt, Besuche = Host-Flagzählung
(Nah 57 181 941, Fern 25 356 531).

| | Zählschritte × Stichprobe | Kandidaten | Rate | machtlos | K0/K1/K2/K3/K4 | Eimer s | Koinz. ρ-Klemme | u-Klemme | neg. geladen |
|---|---|---:|---:|---:|---|---|---:|---:|---:|
| Nahfeld | 151 × n%11 | 4 448 | 5,7 ppm | 6 (f_eq neg. 6) | 24/**4401**/17/0/0 | 5/6/3349/673/409 | 2 316 (52 %) | 5 | 3 540 |
| Fernfeld | 38 × n%5 | 767 | 4,0 ppm | 0 | 0/**767**/0/0/0 | 0/0/74/491/202 | 74 (10 %) | 0 | 884 |

**Befund:** Negative Nachkollisions-Populationen sind bei 8 mm selten und liegen zu 99 % (Nah) bzw. 100 % (Fern) in K1 = TYPE_MS am
bewegten Boden — derselbe Ort wie die Dichteklemme (Stufe 0: 100 % am Boden). s meist 0,5–0,95. Nachladeprobe t = takt+3: Nah 256, Fern 60.
Folge für Modus 2: der Begrenzer wirkt fast nur im Bodenband; ob boden_eq diese Zellen danach überschreibt (Plan E9), ist mit dem offenen
Stufe-0-Befund „boden_eq 0 Treffer vs. stream_collide 1,13 Mio am selben Band“ verknüpft und ungeklärt.

> **BERICHTIGT nach M2:** boden_eq überschreibt die EINGEHENDEN Populationen der Bandzellen, nicht die ausgehenden f** einer K1-Zelle;
> Populationen, die in Zellen außerhalb des Bands strömen (z > nz_eff; ab x_split nz_down = 1), tragen die Begrenzerwirkung weiter.
> Gemessen (M2): negativ geladene Populationen Nah 3 540 → 71, Fern 285 → 0. Der Absatz unten ist damit überholt.

**Folge für Modus 2 am Boden (Herleitung aus kernel.cpp boden_eq, nicht gemessen):** boden_eq ersetzt NACH stream_collide alle 19 Populationen
der Zellen z = 1..nz_eff (TYPE_MS eingeschlossen, ausgenommen Zellen mit Solid im Abstand ≤ CFD_BODEN_EQ_ABSTAND in Ebene/oberhalb) durch
f_eq(ρ_lokal, u_road). Weil der Begrenzer Masse erhält, bleibt ρ_lokal gleich — die Begrenzerwirkung an diesen K1-Zellen ist danach GELÖSCHT.
Wirksam bleibt Modus 2 nur an K1-Zellen außerhalb des boden_eq-Bands (Reifennähe) und in K0/K2 (Nah 41 von 4 442 Stichproben). Das 8-mm-M2
misst daher vor allem, ob Reifennähe und Facetten etwas tragen; eine Kraftänderung ist nicht zu erwarten. Messbar über [292] (Wirkung) gegen
Kräfte.

## Messung P1c 8 mm B70 (b47076f, Standardzeile 300 ms): M2 CFD_POSITIV=2 gegen M1 CFD_POSITIV=1 (= A, forces bitgleich zu kl_z2c_dd8_b70)

Stichprobe Nah 11, Fern 13 (Primzahl). Ist=Soll in beiden Armen und Domänen erfüllt; Klemm-Budget in beiden eingehalten.

| | M1 (Messarm) | M2 (anwenden) |
|---|---|---|
| Nah Kandidaten / K0,K1,K2 | 4 448 / 24, 4 401, 17 | 2 293 / 38, 2 235, 14 |
| Nah s-Eimer | 5/6/3349/673/409 | 4/2/13/1385/883 |
| Nah negativ geladen (Zählschritte) | 3 540 | **71** |
| Nah Σ(1−s) (Stichprobe) | – | 140,96 (s̄ ≈ 0,94) |
| Fern Kandidaten (alle K1) / neg. geladen | 282 / 285 | 163 / **0** |
| cd_rest Fenster 250–300 ms (sd) | 0,4596 (0,0546) | 0,3984 (0,0578) |
| cz_rest Fenster 250–300 ms (sd) | −0,1728 (0,2972) | −0,4012 (0,2954) |
| σ_cd / σ_cz (Block-SEM 4, Budget) | 0,02574 / 0,02367 | 0,02454 / 0,02610 |
| Klemm-Budget Nah+Fern netto | 0,2227 σ | 0,0266 σ (dCd_m 6,5·10⁻⁴) |
| Wanduhr ab 100 ms (je 1 Lauf) | 132,4 s (A: 131,9 s) | 135,5 s |

**Einordnung:** M2 wirkt (negativ geladene Populationen −98 % Nah, −100 % Fern; Kandidaten halbiert, s wandert von 0,5–0,75 nach 0,75–1).
Δcd_rest = −0,061 ≈ 1,7 σ_diff (√(σ₁²+σ₂²) = 0,036); Δcz_rest = −0,23 — nach Block-SEM groß, aber Einzelrealisierung eines chaotischen
Laufs mit Fenster-sd 0,30 und k(4)-unsicherem σ; nach Plan §5 kann 8 mm Kraftänderungen dieser Größe nicht belastbar zuordnen. Wanduhr
M2 +2,3 % gegen M1, M1 +0,4 % gegen A (je eine Messung). Die Dichteklemmen-Nettomasse Nah sinkt nicht (Q_netto 0,176 → 0,596), Fernfeld
fällt stark (0,377 → 0,0069). Nicht verifiziert: Streuung zweier Realisierungen, Wanduhr-Rauschen.
