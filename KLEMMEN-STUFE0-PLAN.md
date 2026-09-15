# Erhaltende Klemmen, Stufe 0 (Messinstrument) — Plan des Planungsagenten, 15.09.2026

Stand HEAD e4eb57f (nur gelesen). Vorlage: UEBERGABE-2026-09-14.md §1. Entscheidungen Heiko: siehe Ende (offen).

## 0 Kurzfazit

- **Befund bestätigt.** Slot 0/1 (kernel.cpp:3113-3114) und Slot 28 (kernel.cpp:3191, :3197) sind ungegatet und sättigend.
  8 mm Produktion: RHO_CLAMP Nahfeld 3 176 331, Fernfeld 62 515; u-Klemme Nahfeld 1872, Fernfeld 0
  (logs/rr_c2d_dd8_a_b70.log:2122, :2152); B-Arm (RHO_RAND) identisch. 4 mm: 14 434 460 / 668 538 / 1 375 206
  (logs/p4_register.log:5455, :5487, :5521).
- **Ansatz richtig**, Formel w·ρ·(u_c − u_roh) gilt für SRT+Guo und P-TRT (§2). Vorlage übersieht 7 Punkte (§1).
- **Empfehlung:** Ganzzahl-Festkomma-Summen in uint-Slots, Host liest im Sample-Takt, Differenzen mod 2³², double-Summe,
  Wickelwächter mit Schranke aus Dekaden-Histogramm. Puffer 224 → **288**. **Fünf** Ortsklassen statt drei.
- **8 mm kann die u-Klemmen-Entscheidung nicht liefern** (§7, Rechnung). Die Entscheidungszahl braucht 4 mm (Heikos Go).

## 1 Prüfung der Vorlage

Veraltete Zeilen: u-Klemme kernel.cpp:3182-3199 (nicht 3161-3173); Dichteklemme kernel.cpp:1257 (Zählung :3113);
calculate_f_eq :1167; atomic_add_f :4169; Legende lbm.cpp:618, Pufferweg :615. „216–223 frei“ ist falsch: 216–220 RHO_RAND,
frei nur 221–223 (lbm.cpp:632).

Übersehen:
1. **ρ_roh fehlt an der Zählstelle.** Klemme in calculate_rho_u (kernel.cpp:1257), ρ_roh verlässt die Funktion nie (:1259).
   Im Trefferzweig aus fhn neu summieren; fhn bleibt bis zur Kollision unverändert (:3106 → :3760).
2. **Ungezählte Klemmstellen.** calculate_rho_u klemmt auch in boden_eq (kernel.cpp:4038), einlass_eq (:4069),
   schale_blend (:4689) — alle drei produktiv an (logs/rr_c2d.txt:5: BODEN_EQ=2, FERN_BODEN_EQ=2, FERN_EINLASS_EQ=2,
   N2F_SCHALE=0.5). boden_eq läuft NACH stream_collide (lbm.cpp:3064, :3090) und setzt f_eq(ρ_c) — der nächste stream_collide
   zählt die Grenze mit Δ≈0 („Echo“), die eigentliche Massenänderung ρ_c−ρ_roh (Faktor 1) zählt niemand. Genau der bewegte Boden,
   für den V1 die Klemme einführte (defines.hpp:95-96, kernel.cpp:1251-1253). schale_blend im FNEQ-Arm (Vorgabe, setup.cpp:6511)
   ist massenerhaltend (ρ_l kürzt sich, :4737-4738). Rho-Tor im Kopplungs-Lift (kernel.cpp:4567) ungezählt.
3. **„DETEPS setzt w später“ ist falsch.** DETEPS ist ein ε im Wandmodell (kernel.cpp:2421-2433). w wird gesetzt bei :3287,
   :3322, :3370, :3406, :3450, :3466, :3590 (SPONGE); P-TRT liest w nur (:3679).
4. **TYPE_E-Zellen sind in Slot 28** (Klemmblock gilt für alle Nicht-Solid, :3182). Dort f* = REG_E = f_eq ohne Guo
   (:3757-3758) → Faktor 1 statt w.
5. **Positivitätsgrenze falsch berichtigt.** Für i≠0 ist f_eq,i ∝ g(x) = 1+3x+4,5x²−1,5u², x = c_i·u (kernel.cpp:1198-1206);
   Minimum bei x = −1/3: 0,5−1,5u² → negativ ab **|u|² > 1/3**, nicht 2/3 (das gilt nur für f₀). Betrifft Stufe 1.
   [Hauptsitzung 15.09.: gegengeprüft — x = −1/3 ist z. B. mit u_x = 1/3 und Rest in y/z bei |u|² knapp über 1/3 erreichbar.]
6. **ΔCd ist nur ein Größenvergleich.** Entfernter Fluidimpuls ist keine Körperkraft. RHO_CLAMP ändert den Impuls nicht →
   keine direkte ΔCd-Übersetzung; Masse als Anteil am Einlass-Massenstrom berichten.
7. **Kugel-Leiter hat 0 Treffer** (rr_c2c3_ku_a_cpu/igpu, rr_c2c5_ku_b_b70) → ohne Testhaken läuft der Buchungspfad nie.

## 2 Klemmstellen

| Stelle | Pfad | lokal | Zähler heute |
|---|---|---|---|
| RHO_CLAMP stream_collide, Nicht-TYPE_E | kernel.cpp:3106 → :1257, Zählung :3113-3114 | fhn, rhon=ρ_c, uxn..uzn=j/ρ_c | Slots 0/1 ungegatet |
| RHO_CLAMP `#ifndef EQUILIBRIUM_BOUNDARIES` | :3020, :3034 | wie oben | tot kompiliert (:3108-3112, defines.hpp:103) |
| u-Klemme VOLUME_FORCE | :3184-3191 | fxn..fzn, rhon, fhn | Slot 28 |
| u-Klemme ohne VOLUME_FORCE | :3194-3197 | wie oben | Slot 28, ungeprüft (defines.hpp:78) |
| update_fields | :4105, :4143-4149 | – | nicht aufgerufen (:4005-4007) |
| boden_eq / einlass_eq | :4038 / :4069 | fhn, rho_local | keiner |
| schale_blend | :4689 | fhn, rho_l | keiner (FNEQ massenerhaltend) |
| Kopplungs-Lift | :4567 ρ-Tor, :4590 u-Tor (214) | v[0..3] | ρ-Tor keiner |
| Facetten/ELIBB calculate_rho_u :1750/:1804/:2054 | Wandmodell-Eingang | – | nicht Stufe 0 |

**Herleitung SRT+Guo** (kernel.cpp:3738-3739, :3760): f* = (1−w)f + w f_eq(ρ_c,u) + (1−w/2)F_i; ΣF_i = 0, Σc_iF_i = F
(calculate_forcing_terms :1266-1276). Masse Σf* = (1−w)ρ_roh + wρ_c → **Δm = w(ρ_c−ρ_roh)**. Impuls ohne u-Klemme: u_roh =
j/ρ_c + F/(2ρ_c) → j+F für jedes w; mit Klemme **Δj = wρ_c(u_c−u_roh)**. RHO_CLAMP impulserhaltend. P-TRT-Abzug ohne 0./1. Moment
(:3670). TRT (nicht gebaut): Δj = wm ρ_c Δu. TYPE_E: Δj = ρ(u_c−u_roh).

**Buchungsort:** direkt nach `#endif // SPONGE` (kernel.cpp:3598), vor :3600 — w final. An den Klemmstellen nur Bits in `uint kl`
(Trefferzweige :3113, :3114, :3191, :3197). Im Buchungszweig ρ_roh, j, u_roh = fma(fxn, 0,5/ρ_c, j/ρ_c) (wie :3185), u_c neu
rechnen; TYPE_E u per load_u. Grund: drei Δu-Skalare über den SGS-Block erhöhen den Registerdruck (Gate misst das); bleibt das Gate
mit drei Skalaren sauber, sind beide Wege gleichwertig. -cl-mad-enable (opencl.hpp:317): Nachrechnung nicht garantiert bitgleich,
harmlos (nur Diagnose).

## 3 RHO_RAND

Keine Wirkung auf Zählung/Ort/Wirkung (Klemme bleibt kernel.cpp:1257; RHO_RAND ändert nur Speicherort :3207-3213, :4157-4158 und
TYPE_E-Lesen :3039-3045). Belegt durch identische Zählerstände A/B, u und forces bitgleich (RHO_RAND-PLAN.md §15). Stufe 0 liest
keinen rho-Puffer. Gate-Arm mit RAND muss den Buchungsblock enthalten (scratch_gate.sh:68-69); Abnahme in beiden Armen.
**Seit b9329a5 ist RHO_RAND Standard** (Hauptsitzung) → „A-Arm“ = CFD_RHO_RAND=0.

## 4 Summenbildung

float32: Summand verloren bei |x| < S·2⁻²⁴; mit N = 1,6·10⁷ Schwelle 0,95·x̄ → O(1)-Fehler (Befund kernel.cpp:4187-4191).
Dekaden-float-Summen ~10–30 %. atomic_add_f emuliert auf Geräten ohne HW-Addition per xchg (:4177).

| Option | Genauigkeit | Aufwand/Risiko |
|---|---|---|
| A float je Slot | O(1) bei 4 mm | wertlos |
| B float je Dekade | ~10–30 % | 2× Slots |
| C float, Host liest+nullt je Fenster | ≤ E·S·6·10⁻⁸ | Host schreibt; float-Bits stören Wickelwächter |
| **D Festkomma, Host-Differenzen mod 2³², double** | ≤ N/(2S), deterministisch | nur lesen; `atomic_add` uint existiert (:4858) |

**Empfehlung D:** q = convert_uint_sat(|Δ|·S+0,5), atomic_add auf h[k]. S = JIT-Define `def_klemm_s`, Vorgabe 2¹⁴ (3,1·10⁻⁵ je
Ereignis). Kappe |Δ| ≤ 16 mit Kappungszähler (Soll 0). Lesetakt = Sample-Takt (25 grobe × ratio 4 = 100 fein; setup.cpp:7635, :6134,
:8362). Wickelwächter je Fenster: B = S·Σ_b Δc_b·2,1·ŵ·Oberkante_b < 2³², ŵ = 2; verletzt → Warnung „mehrdeutig“.
Rechnung: 8 mm ≈ 2,1·10⁴ ρ-Treffer/Fenster, alle in [0,1;1) → B = 6,9·10⁸; 4 mm ≈ 4,8·10⁴ → 1,6·10⁹, sicher.
Gegatete Stichproben abgelehnt (Aliasing mit 100er- und Kopplungstakt, belegt kernel.cpp:3242-3246). Vor/nach Warmlauf auf dem Host
aus t_sample ≥ t_warmup (Auflösung 1 Fenster = 2 ms bei 8 mm).

## 5 Ortsklassen (Rang K0 > … > K4, nur im Trefferzweig)

- **K0 Facettenzelle:** unter FACETTEN `f_bbox(n,&fbi) && fac_fid(fac_idx,fbi)!=0xFFFFFFFFu` mit Zelltor wie :3298
  (kernel.cpp:881-886, :951-959). Fernfeld ohne Facetten → leer (lbm.cpp:2174).
- **K1 Boden/MS:** flagsn_bo==TYPE_MS (:2966), kostenlos.
- **K2 wandnah:** einer der 18 Nachbarn `(flags&TYPE_BO)==TYPE_S`.
- **K3 randnah:** Zelle TYPE_E oder Nachbar TYPE_E (Nahfeld = Koppelrand).
- **K4 frei.**

Nicht `j[]` benutzen (Rang-1-Remat, sonst Spill 448/832, :3825-3833) → `calculate_indices(n,&x0,…)` mit neun Skalaren
(Muster :1158-1163) und 18 ausgeschriebene flags-Lesungen, kein 19er-Feld (:3612-3616, :3660-3662). Kein neuer Speicher.
Begründung fünf Klassen: C2d-rho-Abweichungen 483/497 in TYPE_MS bei z=1 (logs/rr_c2d_vtk_ab.txt); V1-Klemme wegen des Bodens.

## 6 Slot-Layout (Puffer 288), Ist=Soll, Testhaken

| Slots | Inhalt | Art |
|---|---|---|
| 221–225 | ρ-Klemme Treffer je Klasse K0..K4 | sättigend; Soll Σ = [0]+[1] |
| 226–230 | Σq(w·Δρ) untere Grenze (Masse zugeführt) je Klasse | mod 2³² |
| 231–235 | Σq(w·Δρ) obere Grenze (Masse entfernt) je Klasse | mod 2³² |
| 236–241 | Dekaden \|ρ_c−ρ_roh\|: <1e-4, <1e-3, <1e-2, <1e-1, <1, ≥1 | sättigend; Soll Σ = [0]+[1]; Echos im untersten Eimer |
| 242–246 | u-Klemme Treffer je Klasse (TYPE_E in K3) | Soll Σ = [28] |
| 247–251 | Σq(Δj_x > 0) je Klasse | mod 2³² |
| 252–256 | Σq(\|Δj_x\|) für Δj_x < 0 je Klasse | mod 2³² |
| 257–262 | Dekaden max\|u_c−u_roh\| | Soll Σ = [28] |
| 263/264 | Σq(Δj_z ±) alle Klassen | mod 2³² |
| 265 | Kappung | Soll 0 |
| 266–268 | S0d: BODEN/EINLASS_EQ Treffer, Σq unten, Σq oben (Faktor 1) | – |
| 269/270 | S0d: schale_blend-Klemme / Lift-ρ-Tor (nur Zähler) | – |
| 271–287 | frei; NÄCHSTER FREIER SLOT 271 | – |

Stellen mit 224: Allokation lbm.cpp:617; Kommentare lbm.hpp:361, :376, lbm.cpp:630 („204“ veraltet), :632. Kernel kennt die Größe
nicht; gen_main legt keine Puffer an. slot_alt[128] (setup.cpp:7949, :8707) unberührt. **Wickelwächter** läuft bis .length()
(setup.cpp:1470) und würde auf mod-2³²-Slots mit print_error (exit) anspringen (~4 % je Slot im Band 0xE6666666..0xF0000000,
bei ~24 Slots etwa jeder dritte Lauf) → Summenslots in die Ausnahmeliste (Muster Slot 7/75, setup.cpp:1485).

Testhaken `CFD_KLEMM_HAKEN` (nur Testarme):
- **H1:** RHO_CLAMP_MIN/MAX als 1,001/1,002 emittiert (lbm.cpp:2157-2158). Kugel mit CFD_SAMPLE_EVERY=1: K4-Treffer im ersten
  Fenster = Host-Nachbildung aus flags (exakt); Σ untere (K4) = N_K4·w·10⁻³ (Toleranz FP16 + N/(2S)). Host nutzt RHO_CLAMP_MIN
  auch setup.cpp:4948 (Kanal) — Übersteuerung mitziehen.
- **H2:** def_c als 0,05 (lbm.cpp:2009; im Kernel nur in den Klemmzeilen). Verlangt u_lat > 0,05. ΣΔj_x(K4, Schritt 1) =
  −N_K4·w·(u_lat−0,05), Toleranz 1 % (SGS-w).
- **H3:** Klassenzählung für n%7==0 übersprungen (mit H1). Soll: genau eine Ist≠Soll-Beanstandung am Berichtsende
  (print_error = exit, setup.cpp:1518-1520).
- **H4:** Wickelschranke 2¹⁶ → Mehrdeutigkeitswarnung muss feuern.

## 7 Host-Bericht

`berichte_klemmbilanz` neben `berichte_dichteklemme` (setup.cpp:1381); Aufruf dd :9010, Kugel :5485, Kanal optional :4168.
Lesen je Sample dd :8362ff (beide Domänen), Kugel-Schleife :5354ff (vorher + Ende).

Je Domäne und Phase: Treffer, Klassenanteile. Masse: ΣΔm zu/ab/netto, Rate je Schritt, ppm vom Einlass-Massenstrom u_lat·N_y·N_z,
Quantisierungsschranke N/(2S). Impuls: R_x = −ΣΔj_x; F_lat = R_x/n_Schritte (Nahfeld fein, Fernfeld grob); F_si = units_fine.si_F
bzw. units_coarse.si_F (units.hpp:71, setup.cpp:6279-6281); ΔCd_äq = F_si/(q_inf·A_ref) (setup.cpp:7730, :6121); ΔCz_äq aus Δj_z.
Kugel: units.si_F, q_inf·A_nom (:5441). σ(cd_rest): Block-SEM 4/8/16 aus ber_cd (setup.cpp:225-234; nach Warmlauf :8609, :8639;
Kompaktierung ab 4096 :8712-8715) plus Fenster-sd (:8676); 8 mm cd_rest = 0,4596 ± 0,0546 (rr_c2d_dd8_a_b70.log:2044);
Kugel block_sem(cd_w) (:5498-5501). σ immer aus dem Nahfeld. Zeitreihe `klemmen.csv`.

**Rechnung (nicht gemessen):**
- 8 mm: 1 Cd ≙ 0,5·0,075²·(1,85/0,008²) = 81,3 Gitterkraft. ΔCd ≥ σ bräuchte je Treffer (1872) im Mittel |Δj_x| ≥ 35,6,
  also |u_roh,x| ≳ 12 — ausgeschlossen.
- 4 mm (u_lat 0,125, abgeleitet aus 3000 Near-Steps je 50 ms, p4_register.log:819): 1 Cd ≙ ≈ 903, σ = 0,0367 ≙ 33 je Schritt;
  bei 46 Treffern je Schritt reicht |Δu_x| ≳ 0,24 — offen, muss gemessen werden.

## 8 Zwei Zahlen

- VRAM 4 mm: 64 uint = 256 B je Domäne → **0 MB**. Keine Signaturänderung (rho_clamp_hits hängt schon an stream_collide,
  boden_eq, einlass_eq, schale_blend, Lift: lbm.cpp:633, :635, :637, :793, :854).
- Ohne Treffer: 0 B Speicherverkehr, ~2 Instruktionen (kl = 0, Vergleich+Sprung).
- Mit Treffer: ≈ 50 Additionen, 9 Indexrechnungen, ≈ 26 B lesen, ≤ 6 atomare Schreibzugriffe. 8-mm-Mittel 3,18·10⁶/(15 000 ×
  65,6·10⁶) ≈ 3·10⁻⁶ Treffer je Zelle-Schritt ≈ 10⁻⁴ B.
- Wanduhr trotzdem messen (P-TRT-Zähler kosteten 0,33 %, kernel.cpp:3703-3705).

## 9 Prüfkette

1. Scratch-Gate (scratch_gate.sh:45-69, Geräte :74ff) zuerst auf unverändertem HEAD-Kernel. Neue Arme in gen_main.cpp:
   Nahfeld-Produktion mit SGS_FDWAND (RAND und SPARSAM), Fernfeld ohne FACETTEN mit SPONGE (lbm.cpp:2118-2121) und SPARSAM, Haken H3.
   Heute enthält gen_main weder SGS_FDWAND noch SPONGE (gen_main.cpp:12-178). Soll private 0 / spill 0.
2. Prüfagent gegen den Diff, committen.
3. Leiter über lauf_queue.sh (CFD_QUEUE_DEV 0/2/1): Kugel wie rr_c2c3 (CPU DX=40, iGPU/B70 DX=16, T_END 0,02, FELD_HASH=1),
   HEAD-Binary gegen S0-Binary, beide Arme. FELD-HASH(u) (setup.cpp:5470-5483), forces.csv per cmp; H1–H4.
   Gegenlesen: 6457576348268651081 (rr_c2c3_ku_a_cpu), 13385224609926903411 (rr_c2c5_ku_b_b70).
4. 8-mm-dd B70, Zeile logs/rr_c2d.txt:5: HEAD A, HEAD B, S0 A, S0 B; cmp forces.csv, cd_facetten.csv; rho_rand_ab.py für u.
   Wanduhr aus [LEISTUNG-AB-MARKE] (C2d 5452 / 5500 MLUPs, Rauschen unbekannt → HEAD zweimal). Zusätzlich S0+H1 60 ms.
5. Abnahme: Ist=Soll aus §6 in beiden Domänen und Armen, Kappung 0, Wächter still, Bericht+CSV, bitgleich.

## 10 Commits

- **S0a:** Puffer 288; Legende/Kommentare berichtigen (lbm.cpp:618 „(t%100)“, kernel.cpp:3029-3033 „GEGATET … SAMPLE“, :3190,
  setup.cpp:1502-1503, :1629-1631); Wickelwächter-Ausnahmeliste; neue Gate-Arme. Soll: Gate sauber auf HEAD-Kernel, Kugel-CPU-Hash = HEAD.
- **S0b:** Kernel: kl-Bits, Buchungsblock, Klassen, Treffer, Summen, Histogramme, Kappung, def_klemm_s, H1–H3; Host Ist=Soll am
  Laufende. Soll: Gate 0/0 alle Arme; Kugel bitgleich CPU→iGPU→B70 beide Arme; H1/H2 Ist=Soll mit vielen Treffern; K4 = Host-
  Nachbildung; H3 genau eine Beanstandung.
- **S0c:** Host periodisch lesen, KlemmBilanz, Phasen, klemmen.csv, Bericht F/ΔCd/ppm/σ, Mehrdeutigkeitswächter, H4, analytische
  Schritt-1-Summen H1/H2. Soll: bitgleich, analytische Sollwerte in Toleranz, H4 warnt.
- **S0d:** Nebenstellen boden_eq/einlass_eq (Treffer+Summen), Zähler schale_blend und Lift-Tor. Soll: Gate sauber, dd bitgleich,
  dd+H1 zeigt BODEN_EQ-Treffer > 0.
- Danach 8-mm-Abnahme; 4 mm nur mit Heikos Go.

## 11 Fallen

R()-Klammerfalle (keine Signaturänderung nötig); `//` mitten in R()-Zeilen frisst Code (kernel.cpp:3878-3879); `#ifdef` nur als
String-Splice, neue Defines über lbm.cpp-Emission (Muster :2155-2158); Rückleser im schreibenden Kernel wird wegoptimiert
(:2900-2910); Nullbeweis braucht ungegatete Zähler + Besuchsbeleg (:3067-3075), Kugel hat 0 Treffer → H1/H2; `j[]` am Buchungsort
holt Spill zurück; print_error = exit → Ist=Soll ans Berichtsende (setup.cpp:4367); Sättigungstest nur für Trefferzähler, Summenslots
wickeln absichtlich; Nah/Fern verschiedene Einheiten und Schrittzahlen.

## 12 Offene Entscheidungen Heiko

1. Puffer 256 oder 288 — Empfehlung 288 (+128 B; mit 256 fallen Histogramme, Wickelschranke, Nebenstellen weg).
2. Summenform — Empfehlung Festkomma (deterministische Schranke; float bei 4 mm O(10–100 %)).
3. Lesetakt — Empfehlung Sample-Takt, jeder Grobschritt nur wenn der Wächter anschlägt.
4. Nebenstellen boden_eq/einlass_eq in Stufe 0 — Empfehlung ja (S0d); sonst bleibt der Boden-Massenverlust unsichtbar.
5. Fünf statt drei Ortsklassen — Empfehlung fünf.
6. σ-Bezug — beide drucken, Entscheidung an Block-SEM(4).
7. 4-mm-Lauf für die Entscheidungszahl — Empfehlung direkt nach der 8-mm-Abnahme (Produktionslauf → Go nötig).
8. Positivitätsgrenze in Übergabe und Wissensspeicher auf |u|² ≤ 1/3 berichtigen — ja.

## 13 Nicht verifiziert

atomic_add (uint) und convert_uint_sat auf der CPU-Laufzeit; Gate-Arme ohne weitere Defines; Spill/Scratch nach S0b; Ort der 201
Klemmzellen bei 300 ms; boden_eq-Echo-Treffer (abgeleitet, nicht gemessen); ob „Near-Step“ in p4_register feine Schritte zählt;
u_lat der Kugel (für H2) und ob die Kugel-Schleife nach Schritt 1 zuerst liest; Wanduhr-Rauschen; ob HEAD und S0 bei 8 mm über
zwei Binarys bitgleich bleiben (genau die Abnahme).

## Entscheidungen Heiko 15.09.2026 (~14:45)
- **Alles an 8 mm testen** (Entscheidung 7: kein 4-mm-Lauf jetzt; die u-Klemmen-Entscheidungszahl bleibt damit offen, siehe §7).
- **Erhaltende Klemmen als Schalter implementieren** → Stufe 1 (Positivitätsbegrenzer, `CFD_POSITIV`, Vorgabe 0) wird nach
  Stufe 0 gebaut; eigener Planungsagent gestartet. Der 8-mm-A/B misst dann Schalter an/aus mit Stufe 0 als Instrument.
- Entscheidungen 1–5 wie empfohlen übernommen (technisch, keine Rückfrage): Puffer 288, Festkomma + Sample-Takt,
  boden_eq/einlass_eq in S0d, fünf Ortsklassen, beide σ drucken / Block-SEM(4) entscheidet. 8 (Positivitätsgrenze) erledigt 086d55b.

## Nachtrag S0b (Hauptsitzung 15.09.2026 ~15:10): Ortsklassen ohne Flag-Lesung — die 18er-Nachbarsuche brachte Spill zurück

Scratch-Gate nach dem ersten S0b-Bau: `stream_collide` spill 1216 (B70) / 864 (iGPU) im Produktionsarm prod8nah, private 0.
Offline-Bisektion am generierten .cl (igc_offline, prod8nah):

| Variante | B70 spill | iGPU spill |
|---|---|---|
| voll (K0 Facette + 18er-Nachbarn S/E) | 1216 | 864 |
| ohne Buchungsblock (nur kl-Bits) | 0 | 0 |
| ohne Klassenbestimmung (rho+u-Buchung) | 0 | 0 |
| ohne rho-Teil / ohne u-Teil | 1344 / 1600 | 768 / 736 |
| ohne 18er-Nachbarn (K0+MS+TYPE_E selbst) | 0 | 0 |
| ohne Facettentest (Nachbarn bleiben) | 1280 | 736 |
| nur 6 Flächennachbarn | 0 | 96 |
| **Koordinatenklassen (umgesetzt)** | **0** | **0** (auch prod8fern) |

**Umgesetzte Klassen (Abweichung von §5, erzwungen durch das Gate):** K0 Facettenzelle (wie geplant) > K1 TYPE_MS > **K2 in der
F-BBox ohne Facette („fahrzeugnah“)** > **K3 Randschale Dicke 2 (reiner Koordinatentest; Nahfeld = Koppelrand + Auslass, Fernfeld =
Domänenrand)** > K4 Rest. „Wandnah ohne Facette“ außerhalb der F-BBox ist damit nicht mehr trennbar (landet in K4); im Fernfeld gibt es
keine Facetten, dort ist K2 die F-BBox. Volles Gate danach grün (17 Arme × 2 Geräte, rc 0).

## Messung S0b/S0c, 8 mm, 300 ms, B70 (15.09.2026, VORLÄUFIG bis Prüfpass S0c)

Läufe kl_s0b_dd8_an_b70 / _aus_b70 (dbec4ce) und kl_s0c_dd8_b70 (b470bab), Zeile = rr_c2d_dd8_b_b70 (RHO_RAND Standard).
- forces.csv und cd_facetten.csv aller drei Läufe bitgleich zu rr_c2d_dd8_b_b70 (Instrument an wie aus).
- Wanduhr (je ein Lauf, Rauschen ungemessen): an 198,6 s / 5480 MLUPs, aus 196,8 s / 5512 MLUPs; C2d B ohne Instrument 5500.
- Ist=Soll exakt: Nahfeld rho 3 176 331, u 1872; Fernfeld rho 62 515, u 0.
- **Ort:** Nahfeld rho-Treffer K0..K4 = 599 / **3 175 714** / 18 / 0 / 0 → 99,98 % bewegter Boden (TYPE_MS); Fernfeld 100 % MS.
  u-Treffer 1872 / 0 / 0 / 0 / 0 → alle an Facettenzellen.
- **Größe:** |Δρ|-Dekaden Nahfeld 6684 / 56 027 / 784 769 / 2 328 800 / 51 / 0 → echte Verletzungen um 0,01–0,1, kaum „Echos“.
  |Δu|-Dekaden 0 / 2 / 10 / 167 / 1693 / 0.
- **Phase (S0c):** Nahfeld nach Warmlauf (5048 Schritte) rho 1 126 800 Treffer, u **0**. Masse netto −907 (Gitter) = **−30,9 ppm des
  Einlass-Massenstroms je Schritt** (ganzer Lauf −30,0 ppm); ΔCd_äq 0 nach Warmlauf (RHO_CLAMP erhält den Impuls), ganzer Lauf 0,00037
  aus den 1872 Anlauf-u-Treffern. σ(cd_rest, Block-SEM 4) = 0,02574. Fernfeld nach Warmlauf −77,2 ppm je Schritt. Wickelwächter still.
- **Einordnung nach Übergabe §1.2:** |ΔCd_entfernt| ≪ σ → Stufe 1 ist bei 8 mm Hygiene, nicht Rang 1. Die Dichteklemme ist ein
  **Boden-Befund** (bewegter Boden / BODEN_EQ-Band), kein Wandmodell-Befund. u-Klemme nach Warmlauf wirkungslos (bei 4 mm offen).
