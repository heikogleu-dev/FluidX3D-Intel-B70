# TODO — FluidX3D-v2 (führende Arbeitsliste)

**Umbenannt am 15.09.2026 spät aus `PERFORMANCE.md` (Heiko: „wir nutzen dieses Markdown ja doch anders“).** Oben steht die geordnete
Liste mit Stand und nächstem Schritt je Punkt, darunter die offenen Themen nach Art, dann „Erledigt — nicht noch einmal vorschlagen“.
Die bisherige Performance-Befundlage (Messwerte 11.–15.09.) steht **unverändert als Anhang** am Ende. `ARBEITSLISTE.md` ist der ältere
Stand (09.09.) und nur noch historisch; die Planungsunterlagen der einzelnen Punkte bleiben in ihren eigenen Dateien.

**Regeln für diese Liste:** Zahlen nur mit Quelle (Lauf oder Datei). Ein Punkt wandert nach „Erledigt“, wenn er gemessen und geprüft ist —
mit Commit. Eine Variable je Lauf; 4-mm- und Produktionsläufe nur nach Heikos Go.

---

## 0 · ERLEDIGT am 16.09.2026 (war „morgen zuerst" vom 15.09.)

| # | Punkt | Ergebnis | Beleg |
|---|---|---|---|
| 1 | Volle Audit-Korrektur-Schleife über den Klemmen-Block | **Abgeschlossen.** 4 Prüfdurchgänge, 3 Fix-Runden, ~45 Befunde (2 HOCH aus Durchgang 1, 2 weitere HOCH, die ich beim Fixen selbst eingebaut hatte). Letzter Prüfer: kein HOCH, kein MITTEL. Physik unverändert: 6 bitgleiche Kugel-Hashes, 8 mm bitgleich, Gate 58 Arme × 39 Kernel rc 0 | `37e33dd`, `68b051f`, `e23e8fd`; `logs/gate_a7_16-09.log`, `logs/kl_a8_ku_*`, Tagesprotokoll 16.09. |
| 2 | B70-Gegenprobe nach CAT-Error, dann M-CB1B2 | **Sauber.** Gegenprobe bitgleich zu gestern, kein CAT-Error. M-CB1B2 gefahren, Basis auf aktuellem Binary wiederholt (bitgleich) — eine Variable | `logs/kl_gp_*`, `logs/kl_z2m_b2b.txt` |
| 3 | Klemmen: was geht in den Standard? | **Entschieden (Heiko 16.09.):** `CFD_POSITIV=2` + `CFD_U_KLEMME=1` in den Standard, `CFD_RHO_HUELLE` bleibt 0. **Offen:** die Basisdateien tragen es noch nicht | `KLEMMEN-STUFE2-PLAN.md` §2.5, `logs/kl_std.txt` |

**Der Befund, der die Reihenfolge darunter geändert hat:** beide Klemmen-Arme verschieben die Kräfte **systematisch**,
nicht als Realisierungsstreuung — der neue Standard um −0,061 in `cd_druck_rest` (0 von 50 Samples mit anderem
Vorzeichen), `RHO_HUELLE` um −0,189. Das KLEMM-BUDGET bucht davon nur 1/15 bzw. 1/23: **es ist keine Schranke für die
Kraftwirkung**, weil es die Verlagerung des Zustands (u-Klemme am bewegten Boden ×10,4) konstruktiv nicht sieht.
Damit ist die alte Lesart „Kraftdifferenzen sind Einzelrealisierungen bei 1,7 σ" widerlegt (Herleitung: §2.5 des Plans).

## 1 · Reihenfolge (Heiko, 16.09.2026)

| # | Punkt | Stand | nächster Schritt |
|---|---|---|---|
| **1** | **iGPU-Leistungsleiter** — Pflicht vor 3,75 mm | **ERLEDIGT 16.09.** (a) absolute Fernfeldzeit gekoppelt: 8 mm iGPU 48,7 ms / B70 12,5 ms, **4 mm iGPU 338 ms von 413,6 ms Grobschritt = 81,7 %**, Reserve ~76 ms; (b) Skalierungsleiter 203,5 → 397,4 Mio: **+28,7 % gegen B24** (709 gegen 551 MLUPs), kein Einbruch, Nx%16-Effekt 5,0 % (bekannte Regel, jetzt beziffert). Kopplungskernel +18 % über der reinen Gitterzeit, Skalierung davon ungemessen | **Folge (Heiko):** Nx als Vielfaches von 16 für BEIDE Karten einplanen; B70-Leiter mit `CFD_QUEUE_DEV=1` offen |
| **2** | **4-mm-Voll-Lauf mit vollem aktuellem Stand** (`p4_pu8`, 16.09. 10:15–11:06) | **ERLEDIGT.** = p4_register-Zeile + `CFD_POSITIV=2 CFD_U_KLEMME=1`, eine Variable gegen die Referenz `p4_register`. rc 0, 47,9 min. **Kräfte: kein Effekt** — cd_druck_rest +0,005 ± 0,022 (168/300 Samples, 54 Vorzeichenwechsel), cz −0,004 ± 0,043. Die systematische 8-mm-Verschiebung (−0,061, 0/50) ist bei 4 mm NICHT vorhanden — 8 mm hat wieder getäuscht (wie bei SISM). Klemm-Klassen 4 mm: K0 214 928 (APG-Eingang an Facetten nicht klemmfrei). Kein neuer Bezug, `basis/` unangetastet. Reproduzierbarkeits-Punkt zurückgestellt (Heiko) | Tagesprotokoll 16.09. 11:06 |
| **3** | **APG-Plan prüfen und nachschärfen** | **Plan liegt vor: `PLAN-APG-2026-09-16.md`** (16.09., Planungsagent gegen c60190d; A1/ELIBB-Sperre/κ-Historie von mir nachgelesen). Kern: der Term ist nicht Mozaffari, sondern ein linearer Dünnschicht-Term mit zwei Baufehlern (Höhe y_w statt y_ab unter NACHBAR; 1-Parameter-Fit leckt ∂ρ/∂n); ohne Vorkernel trüge ein A/B drei Variablen; κ=0,5 ist eine widerrufene Eichung, herleitbar ist κ=1 | **Entscheide E1–E7 (Plan §F) bei Heiko**, dann Bau ohne GPU, Prüfkette, danach Gate/Kugel/A0 auf freier Queue |
   **ERGEBNIS 16.09. 12:41 (8 mm, Prüfagent Durchgang 2 sauber → Befund gültig):** APG gebaut, abgenommen (HOCH-1 Instanz-Einfrieren, HAKEN 3, dd-Beleg),
   gemessen A1 κ=1 / A2 κ=0,5 gegen A0: **keine Kraftwirkung** (cd_rest −0,009 / −0,017, Rauschen), **Klemme in 98,5 % der Besuche** (Term ≫ τ_w
   bei y_ab = 8 mm, Augustlehre bestätigt), **−17,6 % Durchsatz**. Empfehlung: PARKEN, kein 4-mm-A/B. Details PLAN-APG-2026-09-16.md §H.
| **4** | **APG/Mozaffari-Linie fahren** (Todo 2) | **ERLEDIGT 16.09. (8 mm UND 4 mm).** `p4_apg1` (4 mm, κ=1 gegen `p4_pu8`, 13:11–14:15, rc 0): **keine Kraftwirkung** — cd_druck_rest −0,0022 ± 0,0011 (150/300, 41 Wechsel), cz_druck_rest +0,0093 ± 0,0021, gegen `p4_register` ebenso Rauschen; einzige systematische Wirkung cz_reib +0,0007 (300/300) = +0,96 % der Reibung. **Autorität ≥ 1 in 96,8 %** (8 mm 98,5 %) → bleibt klemmdominiert. Kosten +17,0 % Wanduhr (47,9 → 56,0 min), +55 MB VRAM. Wirkpfad belegt ([306]=[7]=159 588 435) | **Empfehlung: endgültig PARKEN** (`CFD_FAC_APG=0` Standard, Code bleibt). Entscheid Heiko. Details PLAN-APG-2026-09-16.md §I |
| **4a** | **dx-Umrechnung vervollständigen** (Heikos Entscheid 16.09. 14:58) | **ERLEDIGT 16.09. 20:05** (Plan PLAN-DX-UMRECHNUNG-2026-09-16.md, Commits 2be8205…17c95b5, Prüfagent kein offenes HOCH): Schritt-Schalter folgen u_lat·dx, Maße bleiben Längen, Serienzeilen tragen auf ALLEN Sprossen die 4-mm-Werte (+`CFD_FAR_LX`) | **Abnahme 4/4:** Kugel-CPU-Hashes, 8 mm bitgleich, 4 mm bitgleich (`p4_pu8_dx`), 3,75 mm SISM 150,0 ms und Kräfte = p375_a. Rest: vier Prüfagent-Texte/Guards (Fix-Skript, Bau nach Queue-Ende) |
| **4b** | **VRAM-/Performance-Themen — VOR D3Q27** (Heiko 16.09. 13:10: „bevor wir mit D3Q27 weitermachen würde ich andere VRAM-/Performance-Thematiken angehen wollen") | Kandidaten stehen in Abschnitt 2 dieser Datei | **Vorschlag zur Reihenfolge (billig zuerst):** (a) `CFD_T_WARMUP` 0,201 → 0,29 — 15,6 min UND +1,3 % Genauigkeit, eine Variable, Platz 1 der Performance-Liste und seit 12.09. unerledigt; (b) E1 + B1 — asm-Statistik und Facettenzellen je SIMD-Block, **beide ohne GPU-Budget**; (c) B70-Leiter für Nx%16 (beziffert die Regel auf der zweiten Karte, Pflicht vor 3,75 mm); (d) A2 `auto-large-GRF` als Occupancy-A/B, bitgleich prüfbar; (e) Prüfpunkt/Neustart — der grösste Posten (17–34 min je Folgelauf, Anwärmphase 39 %), aber 200–300 Zeilen Bau. Entscheid Heiko |
| **4c** | **y-Halbzellen-Versatz des Fahrzeugs portieren** (Heiko 16.09. 20:2x: „hatten wir die Artefakte nicht mit dem y-Versatz weg?") | **Gemessen 16.09. an `p375_b`:** Ein-Zellen-Membran beidseits der Mittelebene, dy=±1: 28 506 Wandzellen je Seite (5× Nachbarzeilen), **82,2 % Dicke 1** (Fahrzeugmittel 2,0 %); bei 8 mm am 10.09. 58,2 %. Ursache: y=0 fällt auf jeder Sprosse auf eine ZELLMITTE (fNy ungerade). Alter Baum: FluidX3D/setup.cpp:1534–1537 „nie-wieder-Fix" (+0,5 Zelle, `CFD_NO_YSHIFT`); v2 ohne (OFFENE-PUNKTE B5 — Entwarnung widerlegt) | `place()` setup.cpp:6732: y um +0,5 feine Zelle versetzen, fein und grob derselbe physische Shift; Schalter mit Ansage; Abnahme = Membran-Tabelle (Protokoll 20:25) + Kräfte-A/B 8 mm zuerst. **Geometrieänderung, nicht bitgleich → Go Heiko** |
| 5 | **3,75 mm / 15 m** | rechnerisch machbar seit 12.09. (Anhang 1g); 3,5 mm fehlen ~3,8 GB | nach 1 |
| 6 | **Gemischter Satz D3Q19/D3Q27 nur an Wandzellen** | Idee mit Gate, NICHT begonnen; „44 %"-Ausgangszahl ohne Quelle im Bestand | **erst nach 4b** (Heiko 16.09.: VRAM/Performance vorher); Stufe 1 Zensus (8 mm, KDIAG) + Flächen-Gate |
| 7 | **Referenzabgleich OpenFOAM 13 für die Klemmen-Kraftverschiebung** (neu 16.09.) | die Verschiebung ist gemessen, aber unbewertet — der eigene Vorgängerstand taugt laut Iron Rule nicht als Maßstab | braucht kein GPU-Budget, kann parallel laufen |

### Die Verzahnung von APG, SISM und D3Q27 (im Code nachgelesen, 16.09.)

Alle drei greifen an derselben Zelllage an, über **eine Rückkopplung mit einem Schritt Verzögerung**:
`sgs_fdwand` (kernel.cpp:5439) nimmt **u** und liefert `fac_wfd` = w je Facettenzelle aus |S|_FD; `stream_collide`
liest dieses `fac_wfd` **des Vorschritts** (kernel.cpp:2983). APG nimmt das Nachbar-ρ (kernel.cpp:2010) und ändert
darüber das Wandmodell-Ziel, also die Wandschubspannung und damit u an derselben Zelle. D3Q27 an Wandzellen ändert
den Linksatz und damit den Impulsübertrag — wieder u an derselben Zelle. Seit dem 16.09. hängt die u-Betragsklemme
mit drin: sie greift genau an Facetten und am bewegten Boden.

**Folgen:** (1) die Wirkungen addieren sich **nicht**, ein kombinierter Arm ist nicht die Summe der Einzelarme —
zusätzlich zu „eine Variable je Lauf" braucht die Serie eine FESTE Bezugszeile über alle Arme. (2) APG wird *durch
SISM hindurch* gemessen; SISM senkt ν_t in Lage 1 um 85,2 % (kernel.cpp:3497) und ist Teil der Übertragungsfunktion.
(3) Die im Code stehende offene Frage „ist der SISM-Kraftgewinn Modellphysik oder nur die fehlende Wanddämpfung?"
wird schwerer beantwortbar, sobald APG obendrauf liegt — sie gehört ins Nachschärfen von Punkt 3.

## 2 · Performance — offen

Herleitung jedes Punktes: `PERFORMANCE-ROHBEFUNDE-2026-09-11.md` Teil 4, u_lat-Messung: `GITTERGESCHWINDIGKEIT.md` (Ordnung vom 12.09.,
nach der u_lat-Messung). Alle Zahlen sind Wanduhr, Verkehr oder eingesparte Schritte — keine Instruktionszahl.

### Was sich am 12.09. geändert hat

**`u_lat` ist gemessen und GEPARKT, nicht erledigt.** Der Schalter `CFD_U_LAT` ist gebaut
(Vorgabe 0,075, Inertheit zweimal mit 28 von 28 bitgleichen Dateien abgenommen). Die Laufzeit
folgt streng 1/u_lat: zwei Messungen bei 0,100 ergaben 76,0 % und 77,4 % der Wanduhr. **Aber die
Kräfte bewegen sich**: Cd_rest +0,0346 ± 0,0157 (2,2 σ) über 200–500 ms. Der Versatz sitzt in den
ersten vier Fenstern und ist im letzten verschwunden (−0,004 ± 0,017); das Feld bei 500 ms ist
zwischen den Armen **nicht unterscheidbar** (RMS 2,84 m/s gegen 2,93 m/s, die ein Arm gegen sich
selbst 50 ms später hat). Damit ist das Einschwingen der einzige verbleibende Verdächtige — und
genau deshalb rückt `CFD_T_WARMUP` auf Platz 1.

### Durchsatz-Audit Nahfeldkernel — die sechs Prüfpunkte einzeln (nachgetragen 16.09., Heiko: „die sehe ich in der Todo nicht")

Sie standen bisher nur als Fließtext weiter unten in diesem Abschnitt und tauchten in der geordneten Liste nicht auf.
Reihenfolge wie geplant „billig zuerst": E1 → B1 (beide ohne GPU) → A2 → D1 → C1.

| Nr | Was | GPU? | Stand |
|---|---|---|---|
| A1/A1b | `stream_collide` ist auf der B70 SIMD16 — Upstream ebenfalls | offline | **erledigt** 15.09.: die SIMD-Breite erklärt die Lücke NICHT |
| E1 | asm-Statistik von `stream_collide` (Arm `prod8nah`) auf Nachrichtenbreite und -zahl auszählen | nein | **erledigt 16.09.** 332 LSC-Ops statisch: 79 d16-Nachrichten (32 B = halbe Cache-Zeile je SIMD16-Nachricht), 91 Byte-Lasten (flags), 80 gegatete Atomics; SIMD16/128 GRF, spill 0. Protokoll 17:45 |
| B1 | Facettenzellen je SIMD-Block zählen (lineare Indexreihenfolge n, Blöcke zu 16) — Wanddivergenz beziffern | nein | **erledigt 16.09.** 1,10 % Zellen → **4,22 % SIMD16-Blöcke** (×3,84), 26 % Lane-Nutzung; bei SIMD32 6,63 %. Die 0,67 % (26.08.) waren ein Zellanteil — Widerspruch aufgelöst. Kosten X je Facettenblock UNGEMESSEN (Modell: 3–8 %). Protokoll 17:50, `werkzeuge/b1_divergenz.py` |
| A2 | `auto-large-GRF` als Occupancy-A/B, 8 mm, Wanduhr, bitgleich prüfen | ja, 1 A/B | **erledigt 16.09., NEGATIV:** bitgleich, aber +3,9 % Wanduhr (198,5 → 206,3 s), 5474 → 5238 MLUPs; SIMD32/256 GRF ohne Spill — Occupancy ist nicht der Hebel (`logs/a2_grf.txt`, Schalter `CFD_OCL_OPTIONS`). Protokoll 17:50 |
| D1/D2 | Zeitnahme um den `boden_eq`-Enqueue je Grobschritt; danach die 3D-Range als eigene Variable | ja | offen |
| C1 | GB/s bei 8 mm gegen 4 mm an WORTGLEICHER Zeile — nur ein Unterschied je Zelle trüge die Cache-These | ja | offen |
| A3 | `sub_group_size(32)` erzwingen — **Kerneländerung, braucht Freigabe** | ja | zurückgestellt nach A1b |

### Ausrichtungsregel Nx % 16 == 0 — für beide Karten (Heiko 16.09.)

Bekannt für die iGPU, mit der Leiter vom 16.09. am heutigen Stand beziffert: 5,0 % Durchsatz (706–709 gegen 662–692 MLUPs,
ohne Überlapp, `logs/li_igpu_skala.txt`). Heiko: grundsätzlich auch für die B70 einplanen — `stream_collide` ist dort laut
Befund A1 ebenfalls SIMD16, auch wenn der Einbruch dort bisher weniger sichtbar war. Für 3,75 mm: Nah- UND Fernfeld-Box so
wählen, dass Nx durch 16 teilbar ist. Offen: dieselbe Leiter auf der B70 (`CFD_QUEUE_DEV=1`), um den Effekt dort zu beziffern.

### Die Reihenfolge

| # | Hebel | Gewinn | Physik | Aufwand |
|---|---|---|---|---|
| **1** | **`CFD_T_WARMUP` 0,201 → 0,29** | ~~15,6 min UND~~ **+1,3 % Genauigkeit** — **NACHGEPRÜFT 16.09.:** die „15,6 min" stehen in PERFORMANCE-ROHBEFUNDE-2026-09-11.md:1172 und GITTERGESCHWINDIGKEIT.md:150 ohne Herleitung; bei gleichem `CFD_T_END` spart ein späterer Messbeginn KEINE Wanduhr. Real: entweder 89 ms weniger Messfenster (300 → 211 ms, gleiche Laufzeit) oder `T_END` 0,59 für 300 ms Fenster = +89 ms ≈ +8,5 min bei 4 mm (Rechnung aus 47,9 min/501 ms) | beseitigt den +1,34-%-Bias auf cd_druck (Messfenster beginnt mitten im SISM-Einschwingen, Nahfeld vor Messbeginn nicht einmal durchspült: 0,225 s gegen 0,201 s) | **eine Variable — Entscheid Heiko:** (a) 0,29/0,501 (kürzeres Fenster, gratis), (b) 0,29/0,59 (+8,5 min, volles Fenster), (c) bleibt. Empfehlung (b) für Absolutaussagen gegen OF13, (a) reicht für gepaarte A/Bs nicht (Fehlerbalken wachsen um √(300/211) = 1,19) |
| **2** | **Die sieben Zugriffe: rho/u nur schreiben, wo gelesen wird** | **−6,5 bis −9,5 % Wanduhr** | **bitgleich beweisbar** | hoch |
| **3** | **Prüfpunkt/Neustart** | **17–34 min je Folgelauf** (Anwärmphase ist 39 %) | — | 200–300 Zeilen |
| 4 | ~~`u_lat` erneut, HINTER Punkt 1~~ **ERLEDIGT: 8 Schritte je Zelle sind Standard seit p4_register (12./13.09.)** | −24 % Wanduhr (46,1 min Zeitschleife statt 90) | Reproduzierbarkeit auf dieser Sprosse offen (Abschnitt 3, zurückgestellt) | — |

**Zu 1, und es ist heute mehr als ein billiger Posten:** das Messfenster beginnt mitten im
Einschwingen. SISM wird bei 150 ms scharf, die Mittelung startet bei 201 ms, der Vorgang braucht
92–114 ms. Solange das so ist, trägt **jeder** Arm, der die Zeitschrittweite anfasst, einen
Einschwingunterschied mit — am 12.09. belegt. Der Punkt zahlt also doppelt: Zeit und Deutbarkeit.

**Zu 2 — die sieben Zugriffe, und das ist derselbe Punkt wie „rho/u nur schreiben":** im ganzen
`stream_collide` gibt es **sieben** Zugriffe auf `u[]`/`rho[]`. Die vier lesenden stehen im
TYPE_E-Zweig (`kernel.cpp`, „apply preset velocity/density") — **0,63 % der Zellen**. Geschrieben
wird für 87,8 %, jeden Schritt: 16,8 % des Verkehrs für eine Leserschaft unter zwei Prozent.
**Das neue Argument vom 12.09.:** dies ist der einzige grosse Posten der Liste, dessen Abnahme ein
BYTEVERGLEICH ist und kein Fehlerbalken. Die u_lat-Runde hat gezeigt, was ein Hebel kostet, der
Zahlen ändert — eine ganze Messkampagne, die im Rauschen endet.
Alternative gleicher Ursache: beide auf 2 Byte → **−3 961 MiB VRAM**, −7,2 %. Format am echten
Feld entschieden: `FP16S(rho−1)`, **nicht** int16 für u — das kippt Gates am Wandmodell.

> **Stand 15.09.:** Punkt 2 der Tabelle („sieben Zugriffe“) ist weitgehend umgesetzt — RHO_SPARSAM, U_SPARSAM, RHO_FP16, U_FP16 und
> RHO_RAND (Nahfeld, seit 15.09. Standard) stehen in der Produktionszeile. `CFD_T_WARMUP` steht dort weiter auf 0,2 (Punkt 1 offen),
> Prüfpunkt/Neustart ungebaut.
> **BERICHTIGT 16.09. (Heiko):** `u_lat` ist NICHT geparkt — **8 Schritte je Zelle (`CFD_SCHRITTE_PRO_ZELLE=8`, u_lat 0,125) SIND der
> Standard**, Bezug `p4_register` (48,9 min). Der Absatz „u_lat ist gemessen und GEPARKT" oben ist der Stand vom 12.09. und historisch.
> Am 16.09. hat mich dieser Absatz einen 4-mm-Lauf gekostet (`p4_pu` mit 0,075 gestartet, nach 22 min abgebrochen).

### Durchsatz-Audit Nahfeldkernel: warum 69 % statt 85 % Spitze (Auftrag Heiko 15.09.2026, geprüft, NICHT gemessen)

**Auftrag:** die Lücke 420 GB/s (69 % von 608) gegen Upstream 520 GB/s (85 %) auf derselben B70 mechanismenweise verorten —
messen, nicht bauen; keine Kerneländerung ohne Freigabe; eine Variable je Lauf; Screening 8 mm. Heiko: Doppelprüfungen gegen
frühere Audits sind ausdrücklich erwünscht.

**Prüfung der Ausgangslage (Hauptsitzung 15.09. abends):**
- 420 / 608 / 520 GB/s und 5028 MLUPs: belegt in diesem Dokument (Kopf), **Stand 11.09.** — also VOR den Zwei-Byte-Feldern (12.09.),
  RHO_SPARSAM (12.09.) und RHO_RAND (15.09., jetzt Standard). Vor jeder Deutung am heutigen Stand neu messen.
- „46,1 B/Zelle gemessen (8 mm)“ ist verwechselt: **gemessen 8 mm = 46,6 B/Zelle** (Nahfeld-Spitze 2916 MiB, RHO_RAND);
  46,1 B/Zelle ist die **4-mm-Rechnung** mit RHO_RAND (p4_register 48,0 gemessen minus 973 MiB gerechnet).
- „113 B/Zelle/Schritt Verkehr nach der rho-Einsparung“: **keine Quelle im Repo** (113 kommt nur als 113,0 M Zellen vor,
  LEISTUNG.md:105). Belegt ist §3: 83,5 B je Gitterzelle, Freistrom-Fluid 93 B, Facettenzelle 339,5 B (Stand 11.09.). Neu erheben.
- „Gate prüft 37 Kernel“: heute **39 Kernel × 17 Arme × 2 Geräte** (scratch_gate.sh, inkl. Produktionsarme prod8nah/prod8fern aus
  echten Defines seit 1760eaa).

| Punkt | Stand nach Prüfung | nächster Messschritt |
|---|---|---|
| **A · SIMD/GRF** | **Offline belegt (HEAD 31061af, igc_offline, beide Geräte):** `stream_collide` läuft auf der B70 mit **SIMD16** — in ALLEN 17 Gate-Armen, auch im minimalen Kanalarm e0p0; praktisch alle übrigen Kernel dort mit SIMD32 (u. a. update_fields, boden_eq, sgs_fdwand). iGPU: `stream_collide` **SIMD8**. `grf_count` = 128 in **jedem** Kernel → kein Druckmaß. Früherer Befund AUDIT-BEFUNDE.md ~970 (26.08.): „simd8/16 einzig spillfrei bei 128 GRF“; offen seit dort (Rang 4): `-cl-intel-enable-auto-large-GRF-mode`. clinfo B70: Sub-Group-Größen 16/32, lokale Arbeitsgruppe im Code 64 (opencl.hpp:3). | **A1** (offline, Minuten): Gate-Arm „upstream“ ohne Fork-Defines (kein FACETTEN/SUBGRID/Klemmen/PTRT) → SIMD-Breite von `stream_collide` auf der B70. Ist sie 32, ist der Abfall auf 16 fork-verursacht und Kandidat für die Lücke. **A2** (ein GPU-A/B, 8 mm, Wanduhr, bitgleich prüfen): Compileroption auto-large-GRF. **A3** (Kerneländerung → Freigabe): `intel_reqd_sub_group_size(32)` an `stream_collide`, erst Gate (private/spill), dann A/B. |
| **B · Divergenz Facettenkette** | **Widerspruch im Bestand:** AUDIT-BEFUNDE ~970 (26.08.) „Verzweigungen im Innersten entwarnt (Divergenz auf 0,67 % der Zellen)“ gegen §6 hier „Divergenzkosten ungemessen“. ELIBB-an/aus ist als Divergenz-A/B **nicht sauber** (ändert Physik; der Arm ohne ELIBB war 2,46 % LANGSAMER, Speicher elibb-am-fahrzeug-unverzichtbar). | **B1** (Host, ohne Lauf): Facettenzellen je SIMD-Block zählen — lineare Indexreihenfolge n, Blöcke zu 16 (B70) bzw. 64 (Arbeitsgruppe); Verteilung 0/1/…/16. **B2** nur spezifizieren: Zellklassen-Sortierung (Dispatch-Indirektion). |
| **C · „L2“** | clinfo B70: *Global Memory cache size* **24 MiB**, Zeilenlänge 256 B (Bedeutung auf einer dGPU unklar — kein CPU-L2; CPU 285K meldet 3 MiB). Rechnung Arbeitssatz eine xy-Ebene DDFs (19 × 2 B): 8 mm 845×333 → 10,7 MB (passt), 4 mm 1689×661 → 42,4 MB (passt nicht). | **C1**: GB/s bei 8 mm und 4 mm an WORTGLEICHER Zeile vergleichen (sonst vermischt); nur ein Unterschied im Durchsatz je Zelle würde die Cache-These tragen. |
| **D · boden_eq-Dispatch** | Offen seit Runde 1 (§6: 519,1 M Work-Items für 1,15 M Treffer; M6 ABSTAND-Scan per Flagbit). Früherer Vorschlag AUDIT-BEFUNDE ~963 (Rang 3): „boden_eq-3D-Range, ~250× weniger Threads, Kernel unverändert“. Neu 15.09.: boden_eq klemmt nie (Klemmen S0d, 0 Treffer bei 8,6 Mio Band-Resets). | **D1**: Zeitnahme um den boden_eq-Enqueue (finish davor/danach) je Grobschritt, 8 mm; dann **D2** 3D-Range als eigene Variable. |
| **E · Koaleszenz** | Befund §3 („19 load.ugm.d16u32 + 19 store“) stammt vom 11.09., **vor** U_FP16 und RHO_RAND. Gegenprüfung sinnvoll. | **E1** (offline): asm-Statistik von `stream_collide` im Arm prod8nah auf Nachrichtenbreite/-zahl auszählen. |
| **F · Umgruppierung** | Zweimal folgenlos bzw. mit umgekehrtem Vorzeichen (2073 Instr. weniger → 2,46 % langsamer; Spalding −4,57 % → ±0). | nur verfolgen, wenn A Registerdruck als Occupancy-Bremse zeigt; sonst abhaken. |

**Vorab festgehaltene Deutung (aus dem Auftrag):** SIMD16 statt 32 oder hoher GRF-Druck → Registerarbeit lohnt · Facetten-Blöcke
stark gestreut → Sortierung spezifizieren · Cache klein gegen Arbeitssatz → bandbreitenlimitiert, nicht kernel-limitiert ·
alles unauffällig → 69 % sind der Preis der Wandmodellkette.
**Reihenfolge (billig zuerst):** A1 → E1 → B1 (alle ohne GPU) → A2 → D1 → C1. Report als Markdown in den Chat, nicht committen.

**A1, erster Teil erledigt (15.09. abends, offline):** `stream_collide` mit den echten 8-mm-Produktions-Defines, aber OHNE alle
Fork-Features (kein FACETTEN*/SGS_*/SUBGRID/PTRT/RHO_RAND/U_SPARSAM/F_LISTE/KLEMM, rho und u als float wie Upstream) bleibt auf der
B70 **SIMD16** (iGPU SIMD8). Die zuschaltbaren Fork-Features erklären die Breite also nicht. Offen bleibt **A1b**: der Upstream-Kernel
selbst (git-Historie enthält Upstream ab 7680735, 2022-08-04; Fork-Basis bestimmen, dessen kernel.cpp mit Upstream-Defines offline
übersetzen). Erst wenn Upstream auf der B70 SIMD32 bekommt, ist die Breite ein Fork-Kernbefund.

**A1b erledigt (15.09. abends, offline, gleicher ocloc):** Upstream-`stream_collide` von der Fork-Basis **8986874** (Upstream
2026-07-14), mit Upstream-Defines übersetzt — Benchmark-Arm (FP16S, D3Q19, SRT, keine Erweiterung) UND Erweiterungs-Arm (UPDATE_FIELDS,
VOLUME_FORCE, FORCE_FIELD, MOVING_BOUNDARIES, EQUILIBRIUM_BOUNDARIES, SUBGRID) — ist auf der B70 ebenfalls **SIMD16** (iGPU SIMD8).
**Verdikt A: Die SIMD-Breite erklärt die Lücke zu Upstream NICHT** — Upstream fährt dieselbe Breite. Hypothese „fällt der Kernel auf SIMD16“
falsifiziert. Nebenbefund: die Upstream-Vergleichszahl stammt vermutlich aus dem Upstream-**Benchmark** (README_UPSTREAM.md:738–739:
B70 6750 MLUPs) — also ohne jede Erweiterung, gegen unseren vollen Produktionskernel; vor jedem Prozentvergleich klären, welche
Upstream-Konfiguration die 520 GB/s trägt. A2 (auto-large-GRF) bleibt als Occupancy-A/B offen; A3 (sub_group 32 erzwingen) verliert an
Gewicht, weil auch Upstream bei 16 liegt.

### Die billigen

| Hebel | Gewinn | Aufwand |
|---|---|---|
| Remesh-Diagnostik gattern | 1,35 % (73,3 s für zwei rein berichtende Rechnungen) | zwei `if` |
| Kopplungsernte alle 2 Grobschritte | 1,65 % | zwei Zeilen |
| `extract_plane_macros` vor `lbm_c.finish()` ziehen | 0,5–1,0 % + vier Syncs weniger | klein |
| Kräftekadenz 1 → 4 ms | 0,56 % (gemessen **41-fach** überabgetastet) | eine Variable |
| 150-ms-VTK-Dump, der wieder gelöscht wird | 9 s + 11,7 GB Schreiblast | eine Zeile |
| `fac_geo[6]/[7]` (nie gelesen) | 25 MB VRAM | Stride-Umbau |

> **Stand 15.09.:** nicht erneut geprüft; die Produktionszeile trägt weiter `CFD_FAC_CD_EVERY=1` (Kräftekadenz).

### Speicher: rho und u nur dort halten, wo sie gelesen werden (14.09.2026)

Vollständige Pläne in `UEBERGABE-2026-09-14.md`. Die Zahlen sind **Rechnung** aus p4_register, nicht
gemessen.

| # | Hebel | VRAM | Laufzeit | Stand |
|---|---|---:|---|---|
| **Todo 3** | **rho nur in der Randschale** (Dicke 2), sonst aus den DDFs | −973 MiB (4 mm), −1249 MB (3,75 mm) | ~0 (rho ~0,03 % des Verkehrs) | Plan, **vor 3,75 mm** |
| **später** | **u nur wo gelesen** (Schale + F-BBox + N2F), Heiko: „riskant, aber lohnenswert" | Var a bitgleich ~−1,4 GB, Var b ~−2,1 GB (4 mm) | offen; Var b verschiebt die N2F-Kopplung um einen Feinschritt | Idee, **erst nach Todo 3** |

Beide zusammen: 3,75 mm ~5,2–6,0 GB frei; 3,5 mm nur ~330 MB (Var b), also unter der Untergrenze.

> **Stand 15.09.:** „rho nur in der Randschale“ ist erledigt (RHO_RAND, `RHO_RAND-PLAN.md`); „u nur wo gelesen“ bleibt Idee.

## 3 · Physik und Messverfahren — offen (keine Performance-Punkte)

### Neu auf der Liste, aber unter PHYSIK, nicht unter Performance

**Abstandsgesteuerte Verfeinerung (12.09., Heiko-Frage).** Gerechnet an der echten STL
(46,1058 m² benetzte Fläche, 133 766 Dreiecke) und mit der Steiner-Formel für die Schalenvolumen:
von unserem 4-mm-Nahkasten liegen **2,33 %** innerhalb von 16 mm der Wand (0,744 von 31,96 m³).

| Variante | Zellen | Arbeit je phys. Sekunde | mit gemessener Block-Indirektion (T=8: 71 %) |
|---|---:|---:|---:|
| heute (4 mm Nahkasten + 16 mm Fernfeld) | 722,6 M | 1,00 | — |
| 1 mm bis 16 mm, dann 4/16/64 | 782,5 M | **5,28 ×** | 7,44 × |
| 1 mm nur bis 4 mm, dann 2/4/8/16 | 338,1 M | 1,63 × | 2,30 × |
| **2 mm bis 16 mm, dann 4/8/16/32** | **176,4 M** | **0,42 ×** | 0,59 × |

Die innerste Stufe skaliert mit dx⁻⁴ (acht Zellen, zwei Schritte je Halbierung). **1 mm an der
Wand ist ausgeschlossen** — 743,7 M Zellen allein in der ersten Schale, plus 264 M, wenn die
Fahrbahn als zweite Wand zählt. **2 mm an der Wand wäre ein Viertel der Zellen und 1,7-fach
schneller als heute.** Das ist kein Performance-Punkt: die Abtriebslücke (74,1 % von OF13) lebt
dort, wo der Ablöseort von der Grenzschicht bestimmt wird.
**Vier Dinge stehen dagegen, drei davon gemessen:** Block-Indirektion 29 % Durchsatz bei T=8,
2-Zell-Halo 259,2 MiB vor der Kornwahl, die heutige Zwei-Stufen-Kopplung kostet schon 3,4 % des
Grobschritts fürs Synchronisieren (bei fünf Stufen vier Grenzflächen statt einer). Das Vierte ist
das eigentliche: **unsere Kopplung ist für genau zwei Domänen auf zwei Geräten gebaut.** Und jede
Änderung der Wandzellgröße zieht y⁺ und damit die Eichung von Wandmodell, ELIBB und SISM mit.
**Nicht zu verwechseln mit dem am 11.09. Verworfenen:** dort ging es um eine dritte Stufe IM
iGPU-Schlupf, und dafür gibt es keinen Ort. Eine Abstandskaskade ist etwas anderes.

**Cauchy-Schwarz-Realisierbarkeitsschranke (12.09., aus OPEN_Ludwig).** Klemmt die
Nebendiagonalen des zweiten Moments auf |c_αβ| ≤ √(c_αα·c_ββ). Greift eine Stufe VOR unserer
Geschwindigkeitsklemme, die nachweislich feuert (p4_neu: 56 539 Treffer im Nahfeld). Einzelheiten
in `FREMDSOLVER-OPENLUDWIG.md` Abschnitt 2.2.

### Eigener Punkt: die Geschwindigkeitsklemme erhaltend bauen (12.09.2026, Heiko)

**Heute ist die Klemme ein Eingriff am Zustand, kein Filter.** `clamp(u, -def_c, def_c)` je
Komponente schneidet die Geschwindigkeit ab — und die Geschwindigkeit IST das erste Moment.
Damit nimmt die Klemme dem System Impuls weg, und das Log sagt es selbst, wenn es beim Fernfeld
„0 Treffer (Impuls ungestoert)" meldet.

**Der Vergleich mit OpenFOAM 13, am eigenen Referenzfall geprüft** (`CFD-Cases/mr2v40H`):
`fvOptions` ist **leer**, `limitVelocity`/`limitTemperature`/`rhoMin`/`rhoMax` kommen im ganzen
Fall nicht vor. Was dort steht, sind **Schema-Begrenzer**: `grad(U) cellLimited Gauss linear 1`,
`div(phi,U) bounded Gauss LUST grad(U)`, `div(phi,k) bounded Gauss limitedLinear 1.0`,
`default limited corrected 0.33`. Alle vier greifen am **Fluss oder Gradienten** und sind
**erhaltend** — sie ändern, wie eine Größe transportiert wird, nie die Größe selbst.

**In LBM geht das sauberer als in FVM.** Masse und Impuls sind das nullte und erste Moment von
f; das Gleichgewicht trägt beide vollständig, der Nichtgleichgewichtsanteil hat beide **exakt
null**. Jede Begrenzung, die nur f_neq anfasst, ist damit **exakt** erhaltend, nicht
näherungsweise.

> **BERICHTIGT 13.09.2026:** Positivität ist parameterfrei, wandert aber sehr wohl mit `u_lat`. Das
> D3Q19-Gleichgewicht ist nur für |u|² ≤ 1/3 in Gittereinheiten positiv [BERICHTIGT 15.09.: hier stand 2/3 — das gilt nur für f₀; für i≠0 ist g(x)=1+3x+4,5x²−1,5u² bei x=c_i·u=−1/3 minimal, 0,5−1,5u², KLEMMEN-STUFE0-PLAN.md §1 Punkt 5]. Die heutige Komponentenklemme
> lässt schräg zu den Achsen |u|² bis 1,0 zu. Außerdem greift die **Dichteklemme** in p4_register
> 14,4 Mio mal, zehnfach öfter als die u-Klemme, und erhält die Masse nicht. Stufenplan in
> `UEBERGABE-2026-09-14.md` §1.

**Und es gibt die parameterfreie Fassung:** Positivität. f_eq + f_neq ≥ 0 je Richtung ist eine
physikalische Forderung, keine gewählte Zahl — sie wandert nicht mit `u_lat`, anders als die
heutige Schwelle (0,57735 Gittereinheiten = 230,9 m/s bei u_lat 0,075, aber 138,6 m/s bei 0,125).
OpenLUDWIG führt genau das als `limiter: "positivity"`; die Cauchy-Schwarz-Variante auf dem
zweiten Moment ist die schärfere Schwester (`FREMDSOLVER-OPENLUDWIG.md` 2.2).

**Zwischenschritt, der heute schon zählbar wäre:** den **entfernten Impuls** mitzählen statt nur
die Treffer. Drei Zeilen und ein Slot; danach steht im Bericht nicht „1 421 219 Treffer", sondern
wieviel Promille des Gesamtimpulses die Klemme genommen hat. Erst damit ist entscheidbar, ob sie
Filter oder Eingriff ist. Bei 1,13 Treffern je Schritt (Standard) ist das akademisch, bei 47
(u_lat 0,125) nicht mehr.

**Nicht in TODO 2 mischen.** TODO 2 lässt Schreibvorgänge weg und ändert keinen Wert; seine
Abnahme ist der Bytevergleich. Eine Klemme ändert Werte und zerstört genau dieses Kriterium.
> **Stand 15.09.:** In Umsetzung als Klemmen-Block (Stufe 0 Messinstrument, Stufe 1 Positivitätsbegrenzer `CFD_POSITIV`, Stufe 2
> Budget und Hüllen `CFD_KLEMM_BUDGET`/`CFD_U_KLEMME`/`CFD_TOR_HUELLE`/`CFD_RHO_HUELLE`) — siehe Punkt 1 und 3 oben.

### Klemmen-Folgepunkte (15.09.2026)
- Ursache der 1,13 Mio ρ-Treffer an TYPE_MS-Bodenzellen bei 0 Treffern in boden_eq: Hypothese Bodenlink-Behandlung (apply_moving_boundaries)
  zwischen boden_eq und Klemme — unbelegt (`KLEMMEN-STUFE0-PLAN.md`, Nachtrag BERICHTIGT).
- Unter `CFD_RHO_HUELLE`: Druckauslass schreibt ρ an TYPE_E ungeklemmt; N2F-Band-Diagnose zählt weiter gegen 0,5/1,5 (`KLEMMEN-STUFE2-PLAN.md`).
- Budget-Interim „Masse netto bei u∞“ (Lagally); Ablösung über lokale Summen (Slots 301/302) erst, wenn brutto reißt und netto nicht.
- σ aus korrelierten Blöcken unterschätzt (k(4)-Budget ist dadurch strenger, sichere Seite).

### Zwei offene Punkte vom 12.09., beide KEINE Performance-Punkte

**1 · Die 4-mm-Sprosse ist nicht als reproduzierbar belegt — und einmal war sie es nachweislich
nicht.** `p4_u125` und `p4_u125b` (u_lat 0,125, wortgleiche Zeile) unterscheiden sich: identischer
Commit, alle Quelldateien der gesicherten Code-Kopien byteweise gleich, Binary unverändert
(md5 `800542b6…`), Umgebung bis auf `CFD_RUN_NAME` gleich, Aufbau-Log identisch — und trotzdem
weichen bei 50 ms 928 von 930 Sondenpunkten ab, maximal 0,68 m/s. Ein Lauf trug den
Einlassdefekt, der andere nicht.

**Auf der 8-mm-Sprosse gilt Determinismus dagegen, dreimal belegt:** uv8_vor/uv8_nach2 (über zwei
verschiedene Binaries), uv8_u100/uv8_u100b, uv8_u125_a/_b — je **28 von 28 Dateien bitgleich**.
u_lat ist damit als Ursache entlastet.

Die beiden bekannten Fließkomma-Reduktionen sind bereits behoben und im Code vermerkt: `po_mean`
am 24.08. (`kernel.cpp:3994`) und `object_force` am 25.08. (`:4482`). Die verbliebenen
`atomic_add_f` sitzen in `object_torque`, `object_center_of_mass` (nicht im dd-Pfad) und
`spread_force` (PARTICLES nicht gebaut). **Es ist also eine dritte, unbekannte Quelle.**

**Was es entscheidet:** ein Wiederhollauf der 4-mm-Standardzeile gegen `p4_neu`, 90 min. Fällt er
bitgleich aus, betrifft es nur den 0,125er-Betriebspunkt. Fällt er es nicht, trägt **jeder**
4-mm-A/B dieses Projekts eine unbezifferte Streuung — auch die Fehlerbalken der Baseline.

**2 · Der reflektierende Fernfeld-Einlass.** `CFD_FERN_VI=0` ist der gemessene Default, und
`setup.cpp:6362` sagt es selbst: „rho bleibt am Einlass festgenagelt, der Rand reflektiert —
bekannt und angesagt." Bei u_lat 0,075 bleibt das unter der Sichtbarkeitsschwelle (Sonde
`einlass_saeule.csv`: **0 von 552** Zellen über 2 %), bei 0,125 riss es in einem von zwei Läufen
auf (**71 von 552**, kleinstes u_x 21,53 statt 30 m/s, z = 5,74 m). Der dokumentierte Altfall lag
bei z = 4,62 bis 5,58 m — dieselbe Stelle. Nächster Schritt ist ein Arm mit `CFD_FERN_VI=1`;
er trennt „der Rand reflektiert" von „die Physik trägt die Mach-Zahl nicht".

### Der Befund, der kein Performance-Befund ist

**Das Messfenster beginnt mitten im Einschwingen** — siehe Punkt 1 oben, er ist deswegen dort
hingerückt. Bias **+1,34 % auf `cd_druck`**. Für gepaarte A/B harmlos, für **jede Absolutaussage
gegen OF13 nicht** — und das ist die offene Hauptfrage des Projekts. Die Nahfeldbox wird vor
Messbeginn nicht einmal **einmal** durchspült (0,225 s gegen T_WARMUP 0,201 s).

## 4 · Erledigt — nicht noch einmal vorschlagen

| | warum |
|---|---|
| ratio 4 → 8 | Das **Nahfeld wächst** um 0,60 %, weil `CFD_NEAR_LY` durch 32 mm nicht aufgeht |
| Dritte Auflösungsstufe | Kein Ort dafür: in den Schlupf passen 9,4 M Zellen = Würfel von 1,7 m |
| CPU als Rechengerät | 1/54 der B70; sie steht aber zu **95,4 %** in Barrieren — der Hebel ist **asynchrone Ausgabe**, nicht Rechnen |
| Nahkasten beschneiden | `AUDIT-BEFUNDE.md` B71 fordert die **doppelten** Abstände; die geforderte Box bräuchte 52,3 GiB |
| Fernfeld-Fußabdruck überspringen | 14,9 ms, liegt **komplett im Schlupf** = null Wanduhr |
| Block-Tiling | Durchsatz sättigt bei 80 %; Reserve für den Fall, dass ein Gitter sonst nicht passt |
| M2, M7, M8 | nur instruktionsbegründet — zweimal folgenlos geblieben |
| SIMD-Breite als Ursache der Durchsatzlücke | 15.09. A1b: Upstream-`stream_collide` (8986874) läuft auf der B70 ebenfalls SIMD16 — Hypothese falsifiziert |
| ρ nur in der Randschale (Todo 3) | 15.09. RHO_RAND, Nahfeld, Standard seit b9329a5 (`RHO_RAND-PLAN.md`) |

---

# Anhang · Performance-Befundlage (bis 15.09.2026, unverändert aus `PERFORMANCE.md`)

## Der Deckel, und er ist beweglich

Der Schlupf des Fernfelds beträgt **8,13 %** (34,1 ms von 419,1 ms) und **schrumpft mit jeder
Nahfeld-Maßnahme** — die Runde vom 11.09. hat ein Drittel davon verbraucht. `T_fern = 1,82 ns
× N_fern`, flach über Faktor 15,5 (12-Punkte-Leiter). Die iGPU kann nie mehr als **9,59 %**
der Gesamtleistung tragen und trägt 8,93 %: **der Schnitt liegt bei 93,1 % des Optimums**.
Wer gegen den Deckel optimiert, muss ihn nach jeder Maßnahme neu bestimmen.


**Stand 12.09.2026 (Abschnitte 1e–1g), Rest 11.09.2026.** Konsolidiert aus drei Agentenrunden und eigener Nachprüfung. Die
vollständigen Rohbefunde mit allen Zwischenrechnungen stehen in
`PERFORMANCE-ROHBEFUNDE-2026-09-11.md` (1071 Zeilen); dieses Dokument ist die Arbeitsfassung.

**Messgrundlage:** Offline-Compiler `ocloc` über `werkzeuge/scratch_gate/igc_offline.sh`
(0xe223 = Arc Pro B70, 0x7d67 = Arrow-Lake-iGPU), Lauflog und Flag-Export von
`p4dt_deteps` (4 mm, 11.09.), Quelltext auf `master`. **Alle Prozentzahlen ohne
ausdrücklichen Gegenhinweis sind statische Instruktionszahlen, keine Laufzeiten.**

---

## 1. Die vier Zahlen, die alles begrenzen

| | Wert | Folge |
|---|---:|---|
| Nahfeldanteil am Grobschritt | **95,36 %** (399,7 ms von 419,1 ms) | Das Nahfeld ist der kritische Pfad |
| Schlupf des Fernfelds | 34,1 ms (369,3 ms Schritt in 403,4 ms Fenster) | gehört der **iGPU**, nicht dem Nahfeld |
| **Maximaler Wanduhr-Gewinn** | **8,13 %** | darüber hinaus bindet die iGPU |
| Anteil des DDF-Stroms am Verkehr | **79,9 %** von 43,36 GB je feinem Schritt | bei D3Q19/FP16S nicht reduzierbar |

> **⚠ BERICHTIGT am Abend des 11.09. — und die Korrektur ist selbst ein Befund.** Hier standen
> 434,6 ms, 51,26 ms Schlupf und 11,79 % Deckel. Diese Zahlen stammen aus `p4dt_deteps`, der
> Baseline **vor** dem Audittag. Der heutige Lauf `p4_neu` liegt bei **419,1 ms** (eigene
> Gegenrechnung: 5422 s Wanduhr minus 160 s Aufbau minus 12 s Abschluss, geteilt durch 12 526
> Grobschritte). **Der Schlupf ist keine feste Größe — er schrumpft mit jeder Nahfeld-Maßnahme.**
> Die heutige Runde hat rund ein Drittel des iGPU-Vorrats mitverbraucht: 51,3 → 34,1 ms. Wer
> gegen den Deckel optimiert, muss ihn nach jeder Maßnahme neu bestimmen.

**Verlangsamung des Nahfelds um y kostet `399,7 ms · y` Wanduhr, ab dem ersten Prozent,
ohne Freibetrag.** Beschleunigung bringt `399,7 ms · x`, gültig nur bis **x = 8,53 %**.

Der Kernel erreicht 420 GB/s von 608 GB/s Spitze (69 %) und 81 % dessen, was Upstream auf
derselben Karte erreicht (520 GB/s). **Der wahre Nahfeld-Durchsatz ist 5028 MLUPs**, nicht
die angezeigten 1946 — siehe Abschnitt 4.

---

## 1a. Umgesetzt und abgenommen (11.09.2026)

**E1, E2 und E3 sind gebaut und am 8-mm-Fahrzeug gegen einen VOR dem Umbau gefahrenen
Bezugslauf geprüft.** Reihe `logs/opt_8mm_serie.txt`, Arme `o8_vor` (Binary 9c33b417) und
`o8_nach` (Binary eb56af0b), wortgleiche Schalterzeile, per `diff` belegt.

### Das Ergebnis in einem Satz: die Physik ist bitgleich, über alles.

| Geprüft | Ergebnis |
|---|---|
| **21 CSV-Dateien** byteweise | **alle bitgleich** |
| **7 Feld-Dumps** byteweise, darunter 3 × 1,11 GB Nahfeld bei 300/450/500 ms | **alle bitgleich** |
| **21 Wirkpfad- und Klemmzähler** aus dem Log | **alle identisch** |
| Fehler, beide Arme | rc=0, 0 Fehler, je 180 Warnungen |
| Gitter- und Facettenzahlen | identisch |

Nicht nur die Kräfte — **das ganze Feld zu drei Zeitpunkten, Byte für Byte.** Das ist der
schärfste Nachweis, den dieser Aufbau hergibt.

### Die einzelnen Änderungen

**E1 — Scratch in `fac_nachbar_ab` beseitigt.** Der laufzeitindizierte `c(ib)`-Zugriff nach
der Suchschleife machte die 57-float-Tabelle speicherheimisch. Behoben, indem die
Linkkomponenten **und** der Nachbarindex schon in der Schleife mitgeschrieben werden, wo der
Index compilezeitkonstant ist. Damit ist auch das zweite laufzeitindizierte Feld (`j[ib]`)
weg, das im ursprünglichen Befund nicht genannt war.

| | vorher | nachher |
|---|---:|---:|
| `private_size` B70 (0xe223) | 7296 | **0** |
| `private_size` iGPU (0x7d67) | 3648 | **0** |

Gegenprobe am erzeugten Gerätecode: **alle Änderungen liegen zwischen Zeile 17944 und 18018
der .cl-Quelle, der nächste Kernel beginnt bei 18030.** Kein anderer Kernel ist berührt.

**E2 — Glättungsindex.** Die dichte Tabelle über die Facetten-BBox (`setup.cpp`, bei 4 mm
593,7 MB für 2,1 % Belegung) ist ersetzt durch eine nach Zellindex sortierte Indexliste plus
Binärsuche, 4 B je Facette statt 4 B je BBox-Zelle. **Ohne** die Annahme, dass `F` in
Scanreihenfolge vorliegt: sortiert wird nach (n, i), nachgeschlagen wird der größte Index der
Gleichheitsgruppe — genau das, was das alte `feld[...]=i` beim Überschreiben hinterließ.

**E3 — ELIBB-Remesh-Karte freigegeben.** Sie lebte bis zum Laufende, obwohl `alloc_facetten`
sie auswertet und danach niemand mehr liest. Vor dem Bau geprüft, dass kein Member den Zeiger
hält (`lbm.cpp:929` ist die einzige Verwendung). Freigabe per `swap`, nicht `clear`, weil
`clear` bei einer Hashtabelle die Bucket-Tabelle behält.

### Wanduhr und Speicher

| | `o8_vor` | `o8_nach` | `o8_nach2` | `o8_e5` |
|---|---:|---:|---:|---:|
| Wanduhr | 426 s | 419 s | 415 s | **403 s** |
| System-RAM VmHWM | **2321,2 MB** | – | **2134,2 MB** | – |

**Gesamt 426 s → 403 s = −5,4 % Wanduhr**, bei durchgehend bitgleicher Physik.

**Wanduhr −2,1 %** (Mittel der beiden Wiederholungen gegen den Bezug). Die Streuung zwischen
den beiden identischen Wiederholungen beträgt 4 s, der Gewinn 9 s — er liegt also über der
Streuung, aber nicht weit darüber. **Der Wanduhr-Gewinn ist ausdrücklich nicht die
Rechtfertigung.** Die ist, dass eine Fehlerklasse verschwindet, die diesen Fork schon einmal
einen Faktor 100 gekostet hat.

**System-RAM −187,0 MB = −8,1 %** bei 8 mm. Die unabhängige Prüfung hat zwei Zahlen dazu
geschärft: `stable_sort` braucht einen Temporärpuffer von nochmals rund 12,5 MB, der
Nettogewinn bei 4 mm ist also etwa **581 MB statt 593,7**. Und der Nachschlag ist teurer
geworden — nachgebaut gemessen **+2,5 s Aufbauzeit bei 4 mm** gegen 594 MB. Bei einem
24-Minuten-Lauf ist das ein guter Tausch, aber die Zahl gehört genannt. Bei 4 mm ist mehr zu erwarten: der dichte
Glättungsindex skaliert mit der Facetten-BBox und belegte dort 593,7 MB statt der 258 MB bei
8 mm. Gemessen wird das erst am nächsten 4-mm-Lauf.

### E5 — P-TRT-Zählergatter von `t%100` auf `t%1000`

Eigener Arm `o8_e5` gegen `o8_nach2`. Vorher geprüft, dass keine Abnahme aus dem 100er-Raster
eine Sollzahl ausrechnet: `pruefe_ptrt` vergleicht nur relativ (203 gegen 202) und auf
Ungleichnull. Der Helfer in `setup.cpp:514`, der Zählslots aus `(t_ende-1)/100` berechnet,
gehört dem SGS-Band-Test (Slot 186) und ist nicht betroffen.

| | `o8_nach2` | `o8_e5` |
|---|---:|---:|
| CSV + Feld-Dumps | — | **28 von 28 bitgleich** |
| ausgedünnte P-TRT-Stichproben | 14 017 095 | **1 451 970** (Faktor 9,65) |
| Wanduhr | 415 s | **403 s (−2,9 %)** |

**Das ist der größte Einzelgewinn dieser Runde** — größer als E1, E2 und E3 zusammen, und
die Streuung zwischen zwei identischen Läufen beträgt 4 s gegen 12 s Gewinn.

**Ausdrücklich deklariert:** die Zählerwerte 199 bis 203 fallen um Faktor 10. Ein A/B, das
diese Slots gegen eine Baseline **vor** E5 vergleicht, meldet zu Recht eine Abweichung. Das
ist gewollt und kein Fehler.

### Zwei Nebenbefunde aus der Abnahme

**Der Lauf ist deterministisch.** `o8_nach` gegen `o8_nach2`, gleiches Binary, zweimal
gefahren: **28 Dateien bitgleich, davon alle Feld-Dumps.** Erst das macht die Bitgleichheit
zwischen `o8_vor` und `o8_nach` zu einem Beweis statt zu einem Zufall.

**Die Messkette hatte selbst einen Fehler.** Die erste VmHWM-Mitschrift war unbrauchbar, weil
`pgrep -f "bin/FluidX3D"` auch die eigene Warte-Shell trifft, deren Kommandozeile denselben
Text trägt. Richtig ist `pgrep -x FluidX3D` auf den Prozessnamen. Der Bezugswert blieb
trotzdem gültig, weil die Verunreinigung erst nach Laufende einsetzte und im Verlauf klar
abgrenzbar war.

### Werkzeugarbeit im selben Zug

**Das Scratch-Gate prüfte seit dem Bau am 26.08. nur `stream_collide`** — der Kernelname stand
in `igc_offline.sh` fest verdrahtet. Genau deshalb blieb `fac_nachbar_ab` unentdeckt. Das Gate
prüft jetzt **alle 37 Kernel auf beiden Geräten**, führt bekannte Abweichungen als benannte
Schuld mit Behebungsweg, und meldet einen Eintrag als veraltet, sobald er nicht mehr zutrifft.

Beide Wächterfunktionen haben sich sofort bewährt: der Negativtest fand die iGPU-Hälfte des
Befundes, die im Bericht fehlte, und nach dem Fix meldete das Gate seine eigenen Einträge als
veraltet. **Gate-Exit 0, alle 37 Kernel sauber.**

### Noch offen aus der Liste

E4 (`CFD_FAC_KDIAG=0`) ist ein Schalter und damit eine eigene Messvariable — nicht
mitgebündelt. E5 (P-TRT-Zähler) ist bereits A/B-belegt, aber noch nicht gesetzt. Die
Mittel-Klasse M1 bis M9 ist unangetastet.

---

## 1b. Spalding-Tabelle (M1) — umgesetzt, mit einer widerlegten Behauptung

Von Heiko am 11.09. freigegeben. **Nicht bitgleich** — hier entscheidet Messbarkeit.

**Bauform:** `__constant` im Dateibereich, 512 Stützstellen à 2 kB, Emission über
`device_defines`. Bewusst **kein** Kernelparameter (Signatur-Splice ist die R()-Klammerfalle,
in diesem Fork zweimal bezahlt) und **kein** privates Array (Scratch-Falle). Vor dem Einbau
mit einer Minimalkernel-Probe geprüft: `private_size` 0. Danach offline auf beiden Geräten
übersetzt: private 0, spill 0.

**Schalter `CFD_SPALDING_TAB`, jetzt Default an.** Der Aus-Zustand ist gegen `o8_e5`
bitgleich geprüft (28 von 28 Dateien) — der neue Code ist also nachweislich inert, wenn er
aus ist. Damit war das A/B ein Binary und eine Variable.

**Offline gemessen** (200 000 Punkte log-gleich über den gemessenen Bereich Y = 1,52…2,19·10⁴,
gegen Bisektion in double), τ_w-Fehler:

| | max | p99 |
|---|---:|---:|
| Newton it=3 (Stand bis heute) | 4,364 % | 4,235 % |
| Newton it=8 | 0,0001 % | 0,0000 % |
| **Tabelle 512 / 2 kB** | **0,0035 %** | 0,0033 % |

Geprüfte Gegenvariante ohne Tabelle: ein besserer Startwert bringt Faktor 50 (4,364 → 0,086 %),
erlaubt aber **keine** Iteration weniger (it=2 wäre 4,88 %) und kostet zusätzliche
Transzendente. Die Tabelle gewinnt.

**Am Fahrzeug gemessen.** Zielmarke ist `o8_it8` — acht Newton-Schritte, praktisch
auskonvergiert, eigens dafür gefahren. Ohne diesen Arm gäbe es keinen Bezug, gegen den sich
„messbar positiv" prüfen ließe. Zeiger ist `cd_reib`, weil dort der Spalding-Fehler
systematisch wirkt; die Druckanteile sind von turbulenter Streuung beherrscht.

| Abstand zu `o8_it8`, sechs 50-ms-Fenster | Mittel | Vorzeichen |
|---|---:|---|
| Newton it=3 (`o8_e5`) | **−0,00077** | **sechsmal negativ, systematisch** |
| Tabelle (`o8_tab1`) | **−0,00024** | wechselnd |

**Der systematische Versatz ist zu 69 % verschwunden.** Das ist der Grund, warum die Tabelle
bleibt.

### ⚠ Die Tempo-Behauptung ist widerlegt

| | Wanduhr |
|---|---:|
| `o8_e5` (Newton) | 403 s |
| `o8_tab0` (Tabelle aus) | 404 s |
| `o8_tab1` (Tabelle an) | **402 s** |

**Kein Gewinn** — die Streuung beträgt 4 s. Die in Teil 1 genannten −4,57 % Instruktionen
schlagen nicht durch, weil der Facettenpfad nur **0,6 % der Zellen** betrifft. Das ist am
selben Tag zum **zweiten Mal** dieselbe Lehre: beim ELIBB-Test war der Arm mit 2073
Instruktionen weniger sogar langsamer. **Eine Instruktionszahl ist kein Laufzeitmaß**, und
alle noch offenen Prozentzahlen in Abschnitt 2 stehen unter diesem Vorbehalt.

---

## 1c. Gemeinsamer Zähltakt (M3) — umgesetzt, Gewinn belegt

Die Zähler **abzuschalten** hätte reihenweise Ist=Soll-Abnahmen gebrochen, weil der Host die
erwartete Zahl aus dem 100er-Raster ausrechnet. Stattdessen laufen Kernel-Gatter **und**
Sollformeln über **einen** Takt, der an genau einer Stelle steht (`zaehl_takt()` in
`lbm.cpp`): **71 Gatter** in `kernel.cpp`, **12 Formeln** in `setup.cpp`, darunter alle
`ceil(n/100)`-Sollwerte der Slots 7/20/21/22/76 und der SGS-Band-Wirkpfad 186.

**Die Begründung stützt sich nicht auf Instruktionszahlen**, sondern auf E5: dort brachte das
Ausdünnen **eines** Blocks 12 s. Zähler sind Atomics auf einen gemeinsamen Puffer und
serialisieren.

| Takt | Einzelläufe | Mittel |
|---|---|---:|
| 100 (Default) | 402, 406, 401 s | 403,0 s |
| **1000** | 398, 395 s | **396,5 s** |

**−6,5 s = −1,61 %, und die beiden Spannen überlappen nicht** (schlechtester 1000er-Lauf
395…398 gegen besten 100er-Lauf 401). Das ist der Grund, warum die Maßnahme bleibt.

**Physik unverändert:** Takt 100 gegen den Vorstand 28 von 28 Dateien bitgleich (die
Umstellung ist also inert), Takt 1000 gegen Takt 100 **27 von 28** — die eine Abweichung ist
`slots_verlauf.csv`, die Zählerspur selbst. Keine Abnahme schlug Falschalarm, 0 Fehler.

**Der Code-Default bleibt bei 100.** Bei 8 mm bekäme das Fernfeld mit Takt 1000 nur **eine**
Stichprobe; bei kürzeren Läufen keine, und dann meldet ein No-Op-Wächter zu Recht nichts oder
zu Unrecht einen Defekt. Gesetzt wird der Takt deshalb in der **Standardzeile**, wo die
Schrittzahl bekannt ist: bei 4 mm sind es 50 Stichproben im Nahfeld und 12 im Fernfeld.

**Werkzeugbefund:** der Gate-Bau brach, weil die Zwillingsliste in `gen_main.cpp` den neuen
Define nicht kannte. Das Gate meldete das als **Baufehler und nicht als Scratch** — diese
Unterscheidung wurde heute früh eingebaut und hat sich damit zum ersten Mal bewährt.

---

## 1d. Klassen-Diagnostik aus (E4) — der erste eingesparte Grafikspeicher

`fac_kd` ist der **einzige** Diagnostikpuffer, der in der Produktion VRAM belegt. Die drei
übrigen Diagnosepfade (`DIAGZ`, `GDIAG`, `RDIAG`) sind bereits aus, weil ihre Schalter nicht
gesetzt sind.

| | `d8_kdiag_an` | `d8_kdiag_aus` |
|---|---:|---:|
| `fac_kd` (8 mm) | 43 MB | **nicht alloziert** |
| bei 4 mm | 191,0 MiB | — |
| Wanduhr | 397 s | **392 s (−1,26 %)** |
| `forces.csv` | — | **bitgleich** |
| Nahfeld 500 ms | — | **bitgleich** |

**Es ist reine Diagnostik** — Kräfte und Feld sind byteweise identisch.

**Was bleibt:** der Klassen-Zensus und die Wirkpfadzähler stehen weiter im Log, in beiden
Armen zeichengleich (n = 266 283, Rang2 74,5 %, `ELIBB[67]` = 14 417 598). Die
Ein-Variablen-Prüfung eines A/B, wie ich sie heute beim ELIBB-Test gefahren habe, ist also
**nicht** verloren.

**Was entfällt:** `facetten_klassen.csv` (81 kB), `facetten_persistenz.csv` (37,6 MB) und
`yplus_facetten_angewandt.csv` (4,6 MB). Wer sie braucht, setzt `CFD_FAC_KDIAG=1` — das ist
dann ein Prüflauf, kein Produktionslauf.

---

## 1e. Die 4-mm-Bestätigung (`p4_neu`, 11.09.2026)

Alles bis hier war am 8-mm-Fahrzeug gemessen. Dieser Lauf prüft, was bei 4 mm zusammen
ankommt. **Kein A/B** — drei Schalter und neuer Code auf einmal; die Einzelwirkungen sind
alle bei 8 mm gepaart belegt, jede mit eigenem Arm.

| | Baseline `p4dt_deteps` | `p4_neu` |
|---|---:|---:|
| Wanduhr | 94,5 min | **90,4 min (−4,36 %)** |
| VRAM an vergleichbarer Stelle | 28 003 MB | **27 695 MB (−308 MB)** |
| `fac_kd` | 190 MB | **nicht alloziert** |
| System-RAM VmHWM | nicht gemessen | 14 842 MB |

Der Gewinn ist kleiner als die 8 % bei 8 mm. Das ist plausibel: bei 4 mm wiegt der reine
DDF-Strom schwerer, und an dem ändert keine dieser Maßnahmen etwas.

**Die Kräfte sind unverändert**, obwohl das SGS-Band aus ist und die Spalding-Inversion
ersetzt wurde:

| | Baseline | `p4_neu` | Δ |
|---|---:|---:|---:|
| Cd_rest | 0,5387 ± 0,0122 | 0,5372 ± 0,0118 | −0,0015 |
| Cz_rest | −1,0306 ± 0,0218 | −1,0224 ± 0,0221 | +0,0082 |
| cd_reib | 0,0346 ± 0,0006 | 0,0347 ± 0,0005 | **+0,34 %** |

Cd_rest und Cz_rest liegen klar innerhalb der Fehlerbalken. Die Reibung steigt um 0,34 %,
**in derselben Richtung wie bei 8 mm gemessen** — das ist die Spalding-Tabelle, die den
systematischen Versatz der drei Newton-Schritte entfernt.

**Der neue ehrliche Spitzenwert steht zum ersten Mal im Log:** 27 734 MB nach gebundener
Kopplung und Schale, 4 921 MB rechnerisch frei, mit dem ausdrücklichen Hinweis auf
`kf_liste`, die erst in der Zeitschleife bindet. An derselben Stelle stand vorher eine Zahl,
die 300 MB zu optimistisch war.

**Das Restrisiko der Wächterarbeit ist erledigt:** der neue Speicherplan bucht 541 MB mehr
(27 452 → 27 993 MB) und lässt das Gitter trotzdem durch.

---

## 1f. Block-Tiling — zum ersten Mal in v2 gemessen (11.09.2026 abends)

Bis heute stützte sich jede Aussage dazu auf **V1-Zahlen**. Fünf Arme am 8-mm-Fahrzeug,
null Codezeilen, Bezug `bt8_aus` mit 390 s.

### Die Durchsatzkurve — und sie läuft in eine Wand

| Arm | Wanduhr | Durchsatz | zusammenhängend |
|---|---:|---:|---|
| ohne Tiling | 390 s | 100 % | dicht |
| T=8 | 549 s | **71 %** | 16 B = ¼ Cache-Zeile |
| T=16 | 497 s | 78 % | 32 B = ½ Zeile |
| T=32 | 490 s | **80 %** | 64 B = eine volle Zeile |
| T=64 | 497 s | 78 % | 128 B = zwei Zeilen |

**Der Durchsatz sättigt bei 80 % und kommt nicht zurück.** Eine volle Cache-Zeile bringt
gegenüber einer halben nur zwei Punkte, zwei volle Zeilen bringen nichts mehr.

**Damit ist die DDF-Zersplitterung als alleinige Ursache widerlegt** — sie erklärt die ersten
neun Punkte (71 → 80), nicht die restlichen zwanzig. Die bleiben beim
`tile_slot`-Zugriff selbst: eine **abhängige Ladung vor jeder Adressrechnung**, die keine
Kachelform wegformen kann. Das ist genau, was der Quelltextkommentar (`kernel.cpp:958`) seit
jeher behauptet und was ich heute Mittag noch für die halbe Wahrheit gehalten hatte.

### Bitneutralität — zum ersten Mal in v2 belegt

`bt8_t8` und `bt8_t16` gegen `bt8_aus`: **je 25 von 25 Dateien bitgleich**, Feld und Kräfte.
Bisher stützte sich das auf eine Kugelmessung in V1. Der Papierkorb-Slot
(`lbm.cpp:1071-1079`), an dem die ersten Versuche mit Cd 18,4 divergierten, hält.

### Die Kachelform: anisotrop schlägt den Würfel auf BEIDEN Achsen

Ausgezählt am Flag-Export des 4-mm-Laufs (`p4_neu`, 519 139 485 Zellen), Halo 2 wie der Code
ihn verlangt, Aufrundungspolster und Kacheltabelle eingerechnet:

| Kachel | netto frei | zusammenhängend |
|---|---:|---|
| 8×8×8 (heutiger Stand) | 1 284,3 MiB | 8 Zellen |
| **32×8×2** | **1 454,8 MiB** | 32 Zellen |
| **16×8×4** | **1 447,2 MiB** | 32 Zellen |
| 32×4×4 | 1 376,7 MiB | 32 Zellen |
| 16×16×16 | 450,8 MiB | 16 Zellen |

**Welche Achse grob sein darf, ist gemessen** (Kachelvolumen konstant 512 Zellen):

| | netto frei |
|---|---:|
| 32×4×4, grob in **x** | **1 376,7 MiB** |
| 4×32×4, grob in y | 1 124,6 MiB |
| 4×4×32, grob in z | 730,0 MiB |

Grob in x ist also nicht nur die einzige Achse, die der lineare Dispatch zulässt, sondern
auch die **beste**: das Fahrzeug ist in x lang und durchgehend, in y und z dünn und
zerklüftet. Speicherordnung und Geometrie ziehen in dieselbe Richtung.

Empfohlenes Bauziel ist **16×8×4**, nicht das Randoptimum 32×8×2: die 2 in z bedeutet einen
Halo, der doppelt so dick ist wie die Kachel, und macht das Ergebnis empfindlich gegen
Geometrieänderungen. Der Unterschied beträgt 7 MiB.

### Der Anwendungsfall: reicht es für 3,75 mm?

Zellzahl ×1,214 gegenüber 4 mm, Kapazität 32 655 MB:

| | Bedarf | |
|---|---:|---|
| dicht | 33 658 MB | **passt nicht**, 1 003 MB zu viel |
| T=16 | 33 111 MB | **passt nicht**, 456 MB zu viel |
| T=8 | 32 100 MB | passt, 554 MB Restluft — **unter der Mindestluft von 1 024 MB** |
| **16×8×4** (Bau nötig) | **31 902 MB** | passt, 752 MB Restluft, bei ~78 % Durchsatz |

*(Stand 11.09., vor rho/u auf zwei Byte — seit dem 12.09. passt 3,75 mm auch dicht, siehe §1g.)*

**T=16 löst den 3,75-mm-Fall nicht.** Nur T=8 kommt heute in Frage, und dessen Reserve liegt
unter der Projektvorgabe. Die anisotrope Kachel ist der einzige Weg, der 3,75 mm mit
vertretbarer Reserve **und** dem besseren Durchsatz erreicht.

### Verdikt

**Block-Tiling bleibt ein Regler, der Speicher gegen Zeit tauscht — dauerhaft.** Rund 20 %
Wanduhr sind der Boden, und den senkt weder die Kachelform noch, nach dieser Kurve, V1s
Workgroup=Tile-Dispatch mit seinen behaupteten −12 %. **Nicht bauen, solange 4 921 MB frei
sind.** Es ist die Reserve für den Tag, an dem eine Rechnung sonst gar nicht passt — und für
den liegt jetzt belegt vor, was sie kostet und welche Kachelform die richtige ist.

---

## 1g. Was eine Zelle kostet, und was damit an Auflösung geht (12.09.2026)

Alle Zahlen aus dem Produktionslauf `p4_register` (4 mm, 12.09.), nicht geschätzt.

### Der Preis je Zelle

| Posten im Gerätespeicher | Byte |
|---|---:|
| 19 Verteilungen als FP16S | 38 |
| `u`, drei Halbwörter | 6 |
| `rho`, ein Halbwort | 2 |
| `flags` | 1 |
| **Summe** | **47** |

`F` liegt nur über der Wand-BBox, nicht über der Domäne — das spart im 4-mm-Fall **4,31 GB**.
**Gemessen** sind deshalb 23 734 MB für 519 139 485 Zellen = **45,7 B je Zelle**, alles inbegriffen.
Der Hostspiegel kostet 6288 MB = 12,1 B je Zelle; die Bandbreite 115 B je Zelle **und Schritt**.

Zum Vergleich derselbe Löser ohne unsere Änderungen: **93 B** je Zelle mit float32 durchgehend,
**55 B** mit FP16S nur für die Verteilungen. Wir liegen bei **47 B**, also bei 85 % des besten
Upstream-Standes und bei 51 % des float32-Standes.

### 3,75 mm — machbar, und erst seit dem 12.09.

Zellwachstum (4/3,75)³ = **+21,4 %**. Nahfeld 519 → **630 Mio** Zellen, Fernfeld 203 → 247 Mio.

| | 4,00 mm gemessen | 3,75 mm hochgerechnet |
|---|---:|---:|
| VRAM-Spitze Nahfeld | 23 773 MB | **28 822 MB** |
| echt frei (fdinfo, Desktop inbegriffen) | 7450 MB | **2401 MB** |
| Wanduhr | 48,9 min | **63 min** |
| Fernfeld im System-RAM | 9124 MB | 11 073 MB (von 80 GB verfügbar) |

Skaliert wurde getrennt: volumenskalierende Puffer mit dx⁻³, die Facettenpuffer (394 MB) mit der
**Wandfläche**, also dx⁻².

**Ohne rho und u auf zwei Byte läge die Spitze bei 33 862 MB** — 2,6 GB über dem, was die Karte
hat. Die Zwei-Byte-Arbeit ist exakt das, was diese Sprosse möglich macht.

**Die Laufzeit wächst um 29,5 %, nicht um 21,4 %:** die Zellen um 21,4 %, die Schritte je
physikalischer Sekunde um weitere 6,7 %, weil dt mit dx skaliert. Arbeit je physikalischer Sekunde
geht mit **dx⁻⁴**.

**Die iGPU ist nicht der Engpass.** Beide Gitter wachsen um denselben Faktor, `ratio` bleibt 4.
Im Phasenprofil von `p4_register` steht das Fernfeld bei **2,1 %** sichtbarer Zeit (Nahfeld 95,4 %,
Kopplung 1,0 %). **Die 8,13 % Schlupf aus Abschnitt 1 sind damit überholt** — sie stammen vom
11.09., vor den Sparschaltern und den Zwei-Byte-Feldern.

**Drei Vorbehalte, die vor einer Zusage gehören:**
1. **Gitterausrichtung nicht nachgerechnet.** dx_c wäre 15 mm. Ob Nahfeldbox und Fernfeld darauf
   aufgehen, ist offen — bei `ratio` 8 ging 12,2720/0,032 „nicht einmal auf einem halben
   Gitterpunkt auf" (setup.cpp). Geht es nicht auf, wächst die Box und die Rechnung fällt.
2. **2401 MB Restluft sind 2,3-fach über der Untergrenze** statt heute 7,3-fach. Und `kf_liste`
   bindet erst **in der Zeitschleife**, hinter jedem Speicherwächter.
3. **Der Desktop-Anteil schwankt** mit dem, was offen ist (am 12.09. 1432 MB).

**Erster Schritt: ein Aufbaulauf mit kleiner Endzeit** — zeigt Ausrichtung und echten Spitzenwert
in zehn Minuten statt in 63.

#### Nachtrag 13./14.09.2026 — Berichtigungen zu diesem Abschnitt (Details: `UEBERGABE-2026-09-14.md` §3 und §5)

* **Die Gitterausrichtung ist KEIN Risiko.** Box und Versatz rasten seit 09.08. auf ganze Grobzellen ein
  (`auf_grobe_zelle`, setup.cpp). Das ist konstruktiv gesichert, Vorbehalt 1 oben ist erledigt.
* **Das Nahfeld hat 634,62 Mio Zellen, nicht 630.** Exakt aus den Setup-Formeln: 1801×709×497. Die
  y-Paritätsregel hängt eine Grobzelle an (+15 mm). Rechnerisch ~29 030 MB Spitze, **~2195 MB frei**.
* **Schritt-Schalter rechnen u_lat um, aber NICHT dx** (`env_schritte`). SISM_T/SISM_AB/SLICE_NEAR_STEPS/SAMPLE_EVERY
  müssen bei 3,75 mm von Hand ×16/15 gesetzt werden. `basis/fahrzeug_dd.basis` stammt vom 03.09. und führt
  SISM, P-TRT, DETEPS u. a. nicht.
* **`CFD_KRAFT_ZBAND` = 16 mm ist bei 3,75 mm nicht darstellbar** (15 oder 18,75 mm) → cd_rest/cz_rest
  ändern ihre Definition.
* **Neu: rho nur in der Randschale** (Todo 3, Plan 14.09.): −1249 MB VRAM bei 3,75 mm → ~3444 MB frei.
  Laufzeitgewinn ~0 (rho ist dank RHO_SPARSAM nur noch ~0,03 % des Verkehrs).
* **Grenze einer B70 bei dieser Box (Rechnung):** ~3,70 mm, mit Randschale ~3,64 mm. **3,5 mm fehlen
  ~3,8 GB.**
* **Folgeidee u nur wo gelesen** (Übergabe §6.1, Rechnung): Die U_SPARSAM-Maske ist die Lesermenge.
  Ersparnis 4 mm ~1,4 GB (bitgleich) bzw. ~2,1 GB (N2F aus den DDFs, eine Physikvariable). 3,5 mm mit
  rho- und u-Randspeicher: ~330 MB frei, unter der Untergrenze.

### Zwei B70 — die VRAM-Rechnung geht, die Zeitrechnung vermutlich nicht

| | eine Karte | zwei Karten |
|---|---:|---:|
| VRAM-Budget fürs Nahfeld | 30 199 MB | 61 830 MB |
| tragbare Zellzahl | 661 Mio | **1352 Mio** |
| **erreichbares dx** | **3,69 mm** | **2,91 mm** |

Budget = Kapazität 32 655 MB minus Desktop (1432, hängt nur an EINER Karte) minus Mindestluft
1024 MB je Karte. Der **Halo** ist vernachlässigbar: bei Teilung in x und 2,9 mm ist der
Querschnitt 911×641, also 110 MB gegen 61 830 MB Budget.

**Der Haken sitzt nicht im Speicher, sondern auf der iGPU.** Bei 2,9 mm wächst die Arbeit je
physikalischer Sekunde um **3,62×**. Zwei Karten halbieren den Nahfeldanteil auf 1,81× — das
**Fernfeld auf der iGPU wächst aber ungeteilt um 3,62×**. Ob es dann noch hinter dem Nahfeld
verschwindet, ist **nicht gemessen**: das Phasenprofil misst die WARTEZEIT (2,1 %), nicht die
Arbeit des Fernfelds. Die einzige absolute Zahl dazu ist vom **08.08.2026 bei 8 mm** und vor allen
Optimierungen — „das Fernfeld braucht 79 % der feinen Zeit" (setup.cpp). Träfe sie noch zu, wäre
die iGPU bei zwei Karten **sofort** der kritische Pfad und der zweite Beschleuniger brächte nichts.

**Was gemessen werden müsste, bevor jemand eine zweite Karte kauft:** die absolute Zeit eines
groben Schritts im heutigen Stand. Das ist ein Timer um den Fernfeld-Kernel, kein Umbau — und es
entscheidet, ob Dual-B70 eine Auflösungs- oder eine Leerlaufinvestition ist.

**Dazu kommt, dass der Code es heute nicht kann:** die Kopplung ist für genau zwei Domänen auf
zwei Geräten gebaut. Drei Geräte (zwei B70 im Nahfeld, iGPU im Fernfeld) sind in diesem Dokument
als geparkt geführt und nicht angefangen.

---

## 2. Massnahmenliste — Stand nach dem Audittag

### Umgesetzt und belegt

| Nr | Massnahme | Gewinn (8 mm gepaart gemessen) | Physik |
|---|---|---|---|
| E1 | `c(ib)` durch Mitschrift in der Schleife ersetzen | `private_size` 7296 → 0 (B70) und 3648 → 0 (iGPU) | bitgleich |
| E2 | Glättungsindex → sortierte Liste + Binärsuche | −581 MB System-RAM, +2,5 s Aufbau | bitgleich |
| E3 | `elibb_qmap_dd` nach Gebrauch freigeben | −174 MB System-RAM | bitgleich |
| E1–E3 zusammen | | **−11 s Wanduhr, −187 MB RAM** | 28/28 bitgleich |
| E5 | P-TRT-Zählergatter `t%100` → `t%1000` | **−12 s = −2,9 %** | 28/28 bitgleich |
| M1 | **Spalding-Tabelle** (512 Stützstellen, `__constant`) | Tempo **±0**; systematischer Reibungsversatz **−69 %** | ändert Zahlen, belegt besser |
| M3 | **Gemeinsamer Zähltakt** (71 Gatter + 12 Sollformeln) | **−6,5 s = −1,61 %**, Spannen getrennt | 27/28, Abweichler ist die Zählerspur |
| E4 | `CFD_FAC_KDIAG=0` | **−191 MiB VRAM**, −5 s = −1,26 % | Kräfte und Feld bitgleich |
| — | **Host-Spiegel freigeben** (5 Puffer, bei 4 mm 292 MB) | **UNBELEGT** — mit VmHWM falsch gemessen | 25/25 bitgleich |

**Bei 4 mm zusammen gemessen** (`p4_neu` gegen `p4dt_deteps`): **94,5 → 90,4 min (−4,36 %)**,
VRAM 28 003 → 27 695 MB, Cd_rest und Cz_rest innerhalb der Fehlerbalken.

### Widerlegt

| | Warum |
|---|---|
| **M4 Volumenkraft nicht emittieren** | Sie ist **nicht** tot: `kernel.cpp:2993` speist das Wandmodell-Residuum als Volumenkraft ein. Entfernen hätte `CFD_FAC_KRAFT` lautlos wirkungslos gemacht. |
| **Gemischte Kachelgrössen** | Am fi-Puffer **beweisbar null** (`Σ Kinder ≤ T³`). 64³ und 128³ **kosten** 3,8 bzw. 6,7 GiB Polster. |
| **Tempoversprechen der Spalding-Tabelle** | −4,57 % Instruktionen, null Wanduhr. Der Facettenpfad betrifft 0,6 % der Zellen. |
| **„1,43 GB bei −12 %"** und **„5 464 MLUPS"** | V1-Zahlen, in v2 nie gemessen. |
| **A1 „G ist reine Geometrie"** | Die Tangentialbasis kommt aus `calculate_rho_u` und dreht sich jeden Schritt. |

### Offen, mit Begründung die NICHT an Instruktionen hängt

| Nr | Massnahme | Was fehlt |
|---|---|---|
| M5 | `sgs_fdwand` und `fac_nachbar_ab` verschmelzen | Kostensonde steckt fest: `CFD_SGS_FDWAND=0` wird abgewiesen, weil SISM ihn verlangt |
| M6 | ABSTAND-Scan in `boden_eq` durch ein Flagbit | ungemessen; 347 M Reads je Grobschritt für 0,50 % Treffer |
| M9 | Sechs von acht Round-Trips bündeln | ungemessen; Obergrenze 3,1 % |
| M2 | Geometrie in die freien `fac_geo`-Slots | **bewusst nicht gebaut**: erst die Kostenzahl von `fac_nachbar_ab` |
| — | Host-Spiegel: Arbeitssatz statt VmHWM messen, bei 4 mm | die Messung, nicht der Bau |

### Offen, aber nur durch Instruktionszahlen begründet — nicht bauen

M7 (`cubic_lift_weights`-Tabelle) und M8 (3×3-Tensor, „Variante B"). **Zweimal an einem Tag
ist eine Instruktionszahl folgenlos geblieben**, einmal sogar mit umgekehrtem Vorzeichen.

### Getrennt zu untersuchen

**Block-Tiling.** Der einzige Hebel über 1 GiB VRAM (T=8: netto 1 283,9 MiB) und
nachweislich bitneutral. Preis am heutigen v2-Stand −40 % Durchsatz; V1s Workgroup=Tile
senkt ihn auf −12 %, ist aber nicht portiert und **spillt** in v2 bei 1:1-Übernahme
(1152 B B70, 576 B iGPU). Zwei v2-eigene Fallen: Slot 0 ist ein Papierkorb, und
`active_tile_id` muss hinter `TS_P` gebunden werden. Details in 3.4 und 3.5 der Rohbefunde.

**Nahkasten beschneiden.** 60,9 MiB je z-Schicht, aber `AUDIT-BEFUNDE.md:5343` führt den
Kasten bereits als zu knapp. Physikentscheidung, keine Optimierung.

---

## 3. Wo Zeit und Speicher hingehen

### Verkehr je feinem Schritt (Nahfeld, 519 139 485 Zellen)

| Posten | GB | % |
|---|---:|---:|
| DDF lesen + schreiben (2×38 B × 455,8 M Fluidzellen) | 34,64 | 79,9 |
| rho + u schreiben | 7,29 | 16,8 |
| flags | 0,52 | 1,2 |
| Facettenpuffer gesamt | 0,77 | 1,8 |
| Bitmasken + Präfixsummen | 0,12 | 0,3 |
| **Summe** | **43,36** | **83,5 B je Gitterzelle** |

Je Zellklasse: Solid 1 B (früher Ausstieg), Freistrom-Fluid 93 B, Bandzelle 97,75 B,
**Facettenzelle 339,5 B = 3,65× Freistrom** bei 0,60 % der Zellen.

### Koaleszenz ist in Ordnung

Der DDF-Pfad ist in **jedem** Arm exakt 19 `load.ugm.d16u32` + 19 `store` — voll koaleszierte
32-B-Nachrichten, keine einzige Zusatzberührung durch die Facettenkette. Die Zusatzpuffer
machen die Koaleszenz nicht kaputt; ihr Preis ist Latenz und Instruktionen, nicht Bandbreite.

### Speicher

| | MiB |
|---|---:|
| fi + rho + u + flags + f_maske + F | 27 310,9 |
| Facettenkette | 581,3 |
| SGS-Band (im Standard aus) | 118,8 |
| späte Puffer (coupling, slice_flags, schale, **kf_liste 237,3**, kf_psum/pcnt) | 300,6 |
| **echter Spitzenwert** | **28 311,7** (gedruckt: 28 003) |

Zellklassen gemessen: **12,197 % echt solid**, 87,591 % aktiv, 0,634 % TYPE_E-Kopplungsrand.

**Drei Speicherwächter-Defekte:**
1. Der gedruckte Spitzenwert fällt 300,6 MiB zu früh. `setup.cpp:7055` behauptet, hier sei der
   Aufbau vollständig, aber `alloc_coupling_planes` (7103) und `alloc_schale` (7127) folgen,
   und `kf_liste` bindet erst in der Zeitschleife beim ersten Kräfte-Sample. **Der wahre
   Spitzenwert fällt nach jedem Wächter** — ein Lauf kann den Aufbau überleben und 260 MiB
   später sterben.
2. `bytes_bekannt` (`lbm.cpp:2136-2150`) kennt Facettenkette und SGS-Band nicht: **596,3 MiB
   ungedeckt** netto.
3. `alloc_sgs_band` (`lbm.cpp:660-719`) prüft **keinen freien Speicher**, während
   `alloc_facetten_domain` (`:836-841`) es tut. 118,8 MiB fallen ungeprüft.

---

## 4. Korrekturen — was falsch dokumentiert war

**Alle vier sind selbst nachgeprüft und im Repo behoben.**

| Was | War | Ist |
|---|---|---|
| **MLUPs/GB-s-Anzeige** | 1946 MLUPs, 239 GB/s | **5028 MLUPs.** Die Fortschrittszeile teilt die **grobe** Zellzahl durch die **feine** Schrittzeit, Faktor 2,551 |
| **„1,43 GB bei −12 %"** | als v2-Zahl geführt | **V1-Zahl.** `SPARSE_TILES_WG` existiert in v2 nicht; hier kostet Tiling **−40 %** |
| **„≈ 5 464 MLUPS"** | als v2-Baseline geführt | **V1**, Einzeldomäne, 337,5 M Zellen, ohne Wandmodell (`FluidX3D/MODIFICATIONS.md:251`) |
| **A1: „G ist reine Geometrie"** | eigene These | **falsch.** `t1 = ut/\|ut\|` (`kernel.cpp:2073`) kommt aus `calculate_rho_u` (`:2002`) — die Basis dreht sich jeden Schritt. Konstant sind nur der 3×3-Tensor und die Invarianten Snn, det, tr |
| **Bandbreitenanzeige** | „überzeichnet um 10 %" | **47 %.** Grösster Posten: 18 B Nachbar-Flags für 100 % der Zellen, bezahlt von 0,21 % |

**Zur Anzeige-Konvention:** `Info::print_update` nimmt `lbm->get_N()` (`info.cpp:119`);
`info.lbm` zeigt aufs Fernfeld, weil `lbm_c.run(0u)` als letztes initialisiert
(`setup.cpp:7045`), während `info.update` nur in `LBM::run` steht (`lbm.cpp:2447`) und die
Zeitschleife `run()` ausschliesslich fürs Nahfeld ruft. **Verhältnisse zwischen zwei Läufen
bleiben gültig, Absolutwerte nicht.**

Damit dreht sich das Gesamtbild: der v2-Nahkernel liegt **8 % unter** der nackten
V1-Baseline, mit der kompletten Facettenkette, SISM, P-TRT und DETEPS obendrauf. Der gesuchte
„Faktor 2,8" war ein Artefakt.

---

## 5. Sparse Tiles — Verdikt

**Obergrenze jeder Halo-2-Kachelung: 2 035,4 MiB**, nicht 2 294,6 — der 2-Zell-Halo kostet
259,2 MiB, bevor ein Korn gewählt ist.

| T | fi frei netto | % der Obergrenze |
|---:|---:|---:|
| 4 (**in v2 gesperrt**, `setup.cpp:4519/5066/5683`) | 1 630,0 MiB | 80,1 % |
| **8** | **1 283,9 MiB** | 63,1 % |
| 16 | 449,6 MiB | 22,1 % |
| 32 | 106,7 MiB | 5,2 % |
| **64** | **−3 074,6 MiB** | Polster > Ersparnis |
| **128** | **−6 722,6 MiB** | 335 von 336 Kacheln aktiv |

**Gemischte Grössen sparen beweisbar null Byte am fi-Puffer.** Acht Kinder der Kante T/2
überdecken genau T³ Zellen, tote Kinder kosten 0, also ist `Σ Kinder ≤ T³` immer. Eine
Oktree-Optimierung über das ganze Gitter landet exakt auf dem flachen Optimum. Ihr einziger
Gewinn ist die Indextabelle: bei erwägbaren Körnern **0,8 bis 27 MiB**, erkauft mit **+7,5 %
Instruktionen** und Unverträglichkeit mit Workgroup=Tile.

**Die Fahrbahnplatte ist ein Phantom:** zwei Zellen dick, der Halo frisst sie, **99,97 % des
einsparbaren Bestands liegen im Fahrzeuginneren**.

**Die Ursache der Durchsatzstrafe** ist nicht der `tile_slot`-Gather allein, sondern die
DDF-Kontiguität: beim flachen Dispatch decken 64 Threads bei T=8 **acht verschiedene Tiles**
ab, also 8 × 16 B statt 128 B am Stück. Das erklärt V1s Messreihe ohne Zusatzannahme.

**V1s Workgroup=Tile ist nicht 1:1 portierbar** — sie spillt in v2 (1152 B auf der B70,
576 B auf der iGPU), weil das geteilte `cbj[]` genau das Gegenteil der Rang-1-Remat ist. Eine
Remat-Fassung ist spillfrei und kostet +3,3 % gegenüber dem naiven Tiling. Zwei v2-eigene
Fallen: **Slot 0 ist ein Papierkorb** (`lbm.cpp:1071-1079`; eine 1:1-Kopie schreibt still
falsche Physik), und `active_tile_id` muss hinter `TS_P` gebunden werden, sonst bindet
`fac_geo` als `tile_slot`.

**Billigster Weg zu einem belastbaren Ja/Nein:** Nahfeld allein als Einzeldomäne, 25-s-Paar
`CFD_SPARSE_TILES=0/1` bei T=8 und T=16, Wanduhr je Schritt. Null Codezeilen.

---

## 6. Was nicht gemessen werden konnte

| offen | Rezept |
|---|---|
| **Laufzeit irgendeiner Massnahme** ausser E5 | 8-mm-Paar über die Queue, Wanduhr je Grobschritt |
| Durchsatzkosten von SPARSE in v2 | 25-s-Paar, Nahfeld einzeln |
| Trefferquote der u-/flags-Gather | A/B mit `CFD_FAC_NACHBAR=0` bzw. `CFD_SGS_FDWAND=0` |
| Dispatchkosten von `boden_eq` (519,1 M Work-Items für 1,15 M Treffer) | offen seit Runde 1 |
| Divergenzkosten der Facettenkette (0,6 % der Zellen, 4571 Instr. je Subgroup) | Laufzeit-A/B oder fdinfo-Profiler |
| Ist die iGPU bandbreiten- oder rechengebunden? | 25-s-Paar auf Gerät 2 mit `UPDATE_FIELDS` an/aus (−17 % Verkehr) |
| Freier VRAM real | Debugfs-Rechte fehlen; alle Frei-Werte sind die 20/19-Rekonstruktion |
| L2-Grösse der B70 | nirgends im Repo belegt |

---

