# Komplett-Audit 2026-08-15 — drei unabhängige Prüfer (Kernel / Host / Zusammenspiel)

**Gesamturteil: KEIN Befund der Klasse „rechnet still falsch im aktuellen Default-Build".**
Alle aktiven Kernel formelverifiziert, alle Host-Pfade verdrahtungsgeprüft, Zusammenspiel
(Reihenfolgen, geteilte Puffer, Kopplung, Instrumente) in Ordnung. Die scharfen Kanten liegen an
den Laufzeitschaltern und latenten Pfaden. Nacharbeiten nach Schwere:

## Hoch
1. **cf_impulsaustausch ist unter WANDFUNKTION prinzipiell ungültig und NIRGENDS dokumentiert**
   (update_force_field setzt reine Reflexion voraus; die WFB ersetzt genau die). Das
   Abnahmekriterium „beide c_f-Wege gleich" widerspricht dem eigenen Schalter. → CSV-Kopf +
   Kanal-Report kennzeichnen. (Der Planungsagent hatte es angewiesen; von mir versäumt.)

## Mittel
2. **WANDFUNKTION/SGS_WANDFREI hängen still an SUBGRID** — SUBGRID aus (Kugel-Validierung
   verlangt das!) = lautlose No-Ops; SGS_WANDFREI hat keinen Wirkpfadzähler.
3. **WFB-Gate lässt TYPE_MS durch** — an bewegten Wänden Doppelbehandlung + absolutes u_t.
   Heute nur per Setup-Konvention geschützt. → fällt mit der Relativgeschwindigkeits-Erweiterung
   zusammen (MS-Guard + u relativ zur Wand).
4. **CFD_LAMBDA ist toter Schalter** (liegt im TRT-Zweig, SRT ist aktiv) und der dd-Log behauptet
   „TRT hält die Wandlage bei Λ=3/16" — unter SRT falsch.
5. **fernfeld + CFD_FERN_VEH=1: Kontaktflächen-Übergabe fehlt** — der dokumentierte
   Fz-Explosionsmechanismus ist dort ungefixt; der Diagnosearm kann sich selbst kontaminieren.
6. **CFD_WANDFUNKTION stiller No-op in kugel/fahrzeug/dd** (Statik nicht gesetzt, keine Warnung).
7. **fernfeld meldet die Dichte-Klemme nicht** (berichte_dichteklemme fehlt dort).
8. **Race auf u[] im CFD_REG_BC-Arm** (UPDATE_FIELDS schreibt, deriv_reg liest im selben Lauf) —
   Default aus, aber jedes REG-A/B wäre nicht reproduzierbar.
9. **1580 vi∩po-Doppelzellen weiter ohne Kommentar am Code** (E9).

## Niedrig
10. Statik-Hygiene asymmetrisch (s_wandfunktion/s_wf_tau nur an 2 von 6 Stellen) ·
11. wf_hits[3] mischt τ-Klemme und u_t≈0-Skip · 12. cf_k aus frisch geregeltem f (Ein-Chunk-Versatz) ·
13. dd-Kommentar „nur flags gelesen" faktisch falsch (voxelize liest auch u) ·
14. messe_yplus hinter dem <16-Samples-_exit · 15. Parsing: CFD_REG_BC/CFD_PO_HART/HEARTBEAT_DIAG
    werten "false"/"=0" als AN · 16. Audit-Kommentar behauptet nicht implementierte Ausnahme ·
17. stumm inerte Schalterkombis (Inventar in Audit 2) · 18. Kanalschleife überzieht ≤99 Schritte ·
19. PARTICLES-spread_force ohne F-BBox (wegkompiliert) · 20. update_moving_boundaries/object_torque
    ohne Aufrufer (dokumentiert) · 21. update_fields-Kernel ohne WFB-Block (tot, bei Wiederbelebung).

**Positiv verifiziert (Auszug):** Kollisionskette, Spalding/WFB-Mechanik samt Tauschpaaren,
Kopplungs-Lagrange (alle vier Zweige nachgerechnet, Summe 1), Ränder samt In-order-Abhängigkeiten,
Klemmzähler-Fix, alle sieben früheren Maskenfallen-Stellen jetzt korrekt, Einheiten aller fünf
Fälle nachgerechnet, dd-Pipeline wettlauffrei.

Vollberichte: Sitzung 2026-08-15 (drei Agenten); Details je Befund mit Datei:Zeile dort.

---

## Nacharbeit Runde 1 (2026-08-15, gleicher Tag)

Alle Befunde bearbeitet; Nachweise:

| # | Massnahme | Nachweis |
|---|---|---|
| 1 | CSV-Kopfzeile "# ACHTUNG: cf_impulsaustausch unter WANDFUNKTION UNGUELTIG" | im WFB-Regressionslauf vorhanden |
| 2 | Emission beider Schalter aus dem #ifdef-SUBGRID-Block gezogen; SGS_WANDFREI ohne SUBGRID = harter Fehler | Kontrollarm bitgleich (s.u.) |
| 3 | MS-Guard am WFB-Aufruf (flagsn_bo!=TYPE_MS) — Vollfix folgt mit der Relativgeschwindigkeit | kernel.cpp, Aufrufstelle |
| 4 | CFD_LAMBDA unter SRT: print_warning "WIRKUNGSLOS"; dd-Logzeile auf SRT korrigiert | Warnpfad-Test: 1 Treffer |
| 5 | CFD_FERN_VEH=1: print_warning (Kontaktflaechen-Uebergabe dort nicht implementiert) | Code |
| 6+10 | s_wandfunktion/s_wf_tau an allen 6 Konstruktorstellen; Warnung in kugel/fahrzeug/dd | Warnpfad-Test: 1 Treffer |
| 7 | berichte_dichteklemme im fernfeld-Fall | Code |
| 8 | REG-Race: print_warning bei CFD_REG_BC=1 unter UPDATE_FIELDS (nicht bitreproduzierbar) | Code |
| 9 | E9-Kommentar an den enqueue-Zeilen (po gewinnt auf den 1580 Doppelzellen, in-order) | lbm.cpp |
| 11 | Slot 3 = nur tau-Klemme, Slot 4 = u_t~0-Skips; Report getrennt | WFB-Lauf: "tau-Klemme 0, u_t~0-Skips 0" |
| 12 | tau_kraft aus f_wirk (dem im Chunk wirkenden f) | cf_kraftbilanz aendert sich exakt dadurch, Ub_lat bitgleich |
| 13 | dd-Kommentar ersetzt (voxelize liest flags UND u) | setup.cpp |
| 14 | messe_yplus vor den <16-Samples-Waechter gezogen (im dd-Fall; Fehlgriff in fahrzeug korrigiert) | Syntax + Code |
| 15 | CFD_REG_BC/CFD_PO_HART/HEARTBEAT_DIAG: atoi>0 statt "alles ausser 0 ist an" | HEARTBEAT_DIAG=0-Test: 0 Treffer |
| 16 | NICHT LOKALISIERT — Re-Audit soll Datei:Zeile nennen oder als erledigt bestaetigen | offen |
| 17 | Inventar stumm inerter Kombis jetzt grossteils angesagt (4/5/6/8) | Warnpfad-Tests |
| 18 | Kanalschleife: letzter Chunk min(regel_alle, n_steps-step) — run() kappt selbst NICHT | Code |
| 19/21 | Latent-Kommentare an spread_force (F-BBox!) und update_fields (kennt WFB nicht) | kernel.cpp |
| 20 | bleibt dokumentiert-latent | — |

**Regression:** Kontrollarm N=38 auf iGPU gegen Altbau-Worktree (405fd97, gleiches Geraet):
Ub_lat und f_lat BITGLEICH ueber den ganzen Kurzlauf; cf_impulsaustausch wackelt im letzten Bit
(atomare Reduktion, war schon immer so); die Abweichung gegen das kanal_n38-CSV war das
Referenz-GERAET, nicht der Code. WFB-Arm: Wirkpfad Ist=Soll=1798320, Klemmen 0.

---

## Re-Audit Runde 2 (2026-08-15, drei frische Pruefer auf 77744e5)

**Kein hoher Befund. Ein mittlerer** (fernfeld liess CFD_WANDFUNKTION weiter stumm verpuffen — die
Nacharbeits-Zeile zu Befund 6 hatte fernfeld schlicht ausgelassen), Rest niedrig. **Befund 16 von
zwei Pruefern unabhaengig lokalisiert**: setup.cpp, Warnung in audit_bewegte_waende behauptete
"Randzellen mit zwei Wandflaechen sind ausgenommen" — die Ausnahme gab es nie.

Nacharbeit Runde 2 (alle Punkte):
- fernfeld: s_wf_tau + CFD_WANDFUNKTION-Warnung (6. Stelle jetzt wirklich vollstaendig)
- Befund 16: Warnungstext ersetzt — Teil-Beitraege von Zellen mit zweitem Wandnachbarn koennen die
  Warnung legitim ausloesen, das steht jetzt drin statt der erfundenen Ausnahme
- Kanal-CSV: letzte Zeile mit GELAUFENEN Schritten etikettiert (step+chunk); Kopfzeile erklaert
  f_lat (frisch geregelt) vs cf_kraftbilanz (wirkendes f)
- SGS_WANDFREI: Wirkpfad-Zaehler Slot 6 (gegatet t%100, nur im emissions-gegateten Block ->
  Kontrollarm bitgleich), positive Ansage im Konstruktor, Report + Null-Pfad-Fehler in
  berichte_dichteklemme; Puffer 6->8 Slots, Legenden in lbm.cpp und lbm.hpp nachgezogen
- Kommentare ehrlich gemacht: Spalding-Genauigkeit (1,2e-2 bei Y=1e4, Klemme mitgenannt),
  Sponge ist GEOMETRISCH (keine TYPE_E-Bindung), E9-Fallzuordnung dd->fernfeld korrigiert,
  SRT/TRT-Logzeile selbstwahr per #ifdef
- Inventar stumm-inerter Schalter angesagt: Kanal warnt bei SPONGE_N/PO_HART/PO_SIGMA/SPARSE_TILES,
  fernfeld bei SPARSE_TILES, CFD_REG_BC ohne einkompilierten REG-Arm meldet WIRKUNGSLOS

**Regression Runde 2 (iGPU):** Kontrollarm Ub_lat bitgleich zu Runde 1 und Altbau; letzte Zeile
12666 = n_steps (vorher 12700-Etikett); SGS-Arm: Wirkpfad 1 798 320 = Soll; WFB-Arm: unveraendert
1 798 320 = Soll. Verbleibend dokumentiert-latent: HEARTBEAT_DIAG im GRAPHICS-Build stumm,
Slot-3/4/5-Ueberlauf erst ab ~1e9 Ereignissen (im Report als Ist!=Soll sichtbar).

---

## Abschluss der Schleife (2026-08-15, Commit 8a66664)

Iron-Rule-Nachpruefer auf dem Runde-2-Diff: **kein Befund der Schwere mittel oder hoeher.**
Die drei kosmetischen Restpunkte (PO-Warnung mit getenv-Falle, SGS-Ansage vor dem Abweiser,
Slot-6-Wortlaut) sind in 8a66664 nachgezogen. Damit ist die Fix-Audit-Schleife SAUBER:

- Runde 1: 21 Befunde bearbeitet, Kontrollarm bitgleich nachgewiesen (77744e5)
- Runde 2: 3 frische Pruefer -> 1 mittlerer + niedrige, alle gefixt, Befund 16 geschlossen (a0a5b5d)
- Nachpruefung: Diff bestaetigt, nur Kosmetik -> nachgezogen (8a66664)

Bewusst offen (dokumentiert-latent, kein Fix noetig): Befund 3 wird erst mit der
Relativgeschwindigkeits-Erweiterung endgueltig (MS-Guard steht als Schutz), Befund 5 traegt eine
Warnung statt einer Implementierung (Kontaktflaechen-Uebergabe im fernfeld-Diagnosearm), 19/20/21
sind wegkompilierte/aufruferlose Pfade mit Warnkommentaren am Code, HEARTBEAT_DIAG ist im
GRAPHICS-Build stumm, Slots 3/4/5 sind ungegatete uints (Ueberlauf erst ab ~1e9 Ereignissen,
faellt als Ist!=Soll auf).

---

# Komplett-Audit Runde 3 (2026-08-15 abends, drei frische Pruefer auf 8457162)

Gegenstand: die komplette C1b-Facettenkette seit dem sauberen Morgenstand 06b432b (Stufen 0-2,
Cd-Auslesepfad). **Gesamturteil: KEIN hoher Befund. Ein mittlerer** (Kanal-K2-Snapshot einen
Chunk zu frueh etikettiert -- von ZWEI Pruefern unabhaengig gefunden; verfaelschte das
K2-Instrument um 0,08 % im 20-ETT-Fenster bzw. ~5 % in Kurzfenstern, die Kugel-Verdrahtung
machte es von Anfang an richtig). Dazu ~14 niedrige (fernfeld-Warnluecke zum DRITTEN Mal an
derselben 6. Stelle, ungegatete Slots 8/9 mit Fahrzeugmassstab-Ueberlaufrisiko, K2/K3 als
Warnung statt hartem Fehler, unbewachter Orientierungskipp nach der Glaettung, stale
Legenden/Warntexte, T2-CSV im Repo-Root, stumm-inerte Schalterkombis, Sentinel-/Index-Hygiene).

**Positiv (Auszug):** alle 12 Tauschpaare in Runde 3 ERNEUT unabhaengig hergeleitet und
bestaetigt; fkraft-Vorzeichen ueber die Impulsbilanz nachgerechnet; fbi-Formel an allen vier
Stellen algebraisch identisch; MS-Restluecke ausgeschlossen (TYPE_MS entsteht im Build exakt
aus der Bedingung, die baue_facetten spiegelt, mit Ist=Soll als Fangnetz in beide Richtungen);
Cd-Pfad im AUS-Arm beweisbar aequivalent zu object_force; Lebenszyklus inkl. Mehrinstanzen-Fall
geschlossen.

**Nacharbeit (d1321fa + 66ceb9d): alle 17 Punkte umgesetzt.** M1-Fenster konsistent (Snapshot
bei step+chunk, fsum nur Folge-Chunks); Slots 8/9 gegatet; K2/K3 hart mit
5000-Schritt-Stationaritaetsgate (Kurztests laufen angesagt durch); Klasse 16 bei
Glaettungs-Kipp; fernfeld sagt jetzt FACETTEN und FACETTEN_DIAG an; Sentinel-Guard; 4ul-Index;
SPALDING_IT/FENSTER-Ansagen; Legenden/Texte aktuell; T2-CSV nach export/. Regression: Tests
3x gruen, Feld-Hash exakt 12755646098055097704. Iron-Rule-Nachpruefer: siehe Folgeabsatz;
K2-Wiederholung im stationaeren 20-ETT-Fenster mit korrigiertem Snapshot: **0,9961** (Band 1 %).

**Nachpruefer auf dem R3-Fix-Diff: kein Befund oberhalb NIEDRIG** — M1-Fenster als exakt
konsistent nachgerechnet (inkl. Randfall Snapshot-im-letzten-Chunk), Slot-Gating bricht kein
Kriterium, 5000er-Gate blockiert keinen Abnahmelauf. Die 4 niedrigen Nachschliffe + 3 Infos
sind umgesetzt (kori aus der Endbitmaske, lbm.hpp-Legende, T2-Doku-Wert, Latent-Kommentar
host-seitig statt in der Emission, SPALDING_IT/FENSTER-Ansagen, K2-Skip als Warnung).
Regression: Tests 3x gruen, Feld-Hash exakt 12755646098055097704.

**DAMIT IST DIE AUDIT-SCHLEIFE RUNDE 3 SAUBER (2026-08-15 abends).** Tagesbilanz der Schleifen:
Morgen-Audit (R1/R2, 21+Befunde) sauber bis 06b432b; C1b Stufen 0-2 mit durchgehender
Agenten-Pipeline; Abend-Audit R3 (1 mittel + ~14 niedrig) sauber bis hier.

---

# Iron-Rule-3-Audits der Stufe-3-Aera (2026-08-16/17) -- Fortschreibung

Die Audit-Historie zwischen R3 (2026-08-15) und dem Stufe-3-Abschluss lebte nur in Commits und
FACETTEN.md-Anhaengen -- diese Luecke war selbst ein Befund des Abschluss-Loops. Kurzprotokoll:

1. **IR3-Audit Bauabschnitt iMEM** (2 Pruefer): 2x MITTEL Kernel (DIAGZ-(uint)(-1.0f)-UB,
   PEMA-utb-Fallback) + 1x HOCH Host (z-Saum-Geometrie -- BB-Loecher in ALLEN Torus-Laeufen,
   per Host-Replikat bewiesen) + Stale-N1-Report. Alle gefixt (fae6a47..2a8bd20).
2. **J4-Dreifach-Analyse** (Forensik/Code-Verdacht/Stabilitaetstheorie): Einzellink-Schur-Flicker
   (546-vs-485-Beleg) -> relative Schwellen (b1f2caf); Rektifikations-Theorem G8 ->
   Saettigungs-Gate a-strich (73b354c). Kugel-Restblocker: Masseleck (J4-Etappe).
3. **Stufe-3-Abschluss-Nachpruefer**: Abnahme bestaetigt (alle Zahlen log-verifiziert);
   3x MITTEL Randpfade (DIAGZ-Rebind-Use-after-free, Kugel-Report-Stale, N1-Doku) gefixt
   (a751c8a/a0c9742).
4. **Abschluss-Loop Runde 1** (2 Pruefer, komplementaer): 2x MITTEL (Kugel-Report Slot-17-blind;
   lauf_queue-Zeilenfilter konnte unbenannte Default-Laeufe starten) + ~10 niedrige
   (Legende-Doppel-[17], SATGATE fehlte in der Schalter-Tabelle, CSV-Schema-Doku, Statik-Symmetrie
   satgate/diagz an 5 Stellen, tote Histogramm-Pushes, Queue-Traps/Herzschlag/Fehler-Marker,
   DIESE Fortschreibungsluecke). Alle in diesem Commit gefixt; Regression: Tests 5x gruen,
   Arm-1-Hash 12755646098055097704 und Arm-3-Hash 887930967142844785 exakt.

Positiv ueber alle Runden: Saum-Fix-Vollstaendigkeit, Slot-Matrix 0-17, fbi-Formel 4-fach
identisch, DIAGZ-Lebenszyklus wasserdicht, alle 16 Ifdef-Varianten konsistent, Kontrollpfade
bitidentisch. Runde 2 des Loops laeuft auf dem Fix-Diff.

## Abschluss-Loop Runde 2 (Nachpruefer auf Fix-Diff 3a6e59c)

1x MITTEL: die Runde-1-Trap-Haertung selbst -- Signal-Handler ohne `exit`, bash setzt nach dem
Handler die Schleife fort -> Kette lief nach Ctrl-C lock- und herzschlaglos weiter (empirisch
per Testskript belegt). Fix: getrennter INT/TERM-Trap mit `trap - EXIT; exit 130`.
3x NIEDRIG: neunte Konstruktorstelle (lbm_f in dd) ohne satgate/diagz-Reset (folgenlos, aber
Symmetrie-Behauptung war falsch); FACETTEN.md-Textrest "fehlen"; Kanal-Label Slot 16 nur halb
satgate-bewusst. Dazu INFO-Kleinkram der Queue gefixt: gesamt zaehlt nur ::-Zeilen, printf statt
echo im Trim, Herzschlag prueft Eltern-PID (SIGKILL-Waise schrieb sonst ewig LAEUFT), mehrfaches
'::' wird protokolliert uebersprungen. Positivliste R2: Trim-Filter dicht, Legende==Kernel-
Inkremente, toter Code rueckstandsfrei, Doku-Schema zeichenexakt, alle zitierten Hashes existieren.
Regression nach R2-Fixes: Tests 5x gruen, beide Anker-Hashes exakt.

## Abschluss-Loop Runde 3 (Nachpruefer auf Fix-Diff 95fc130): KEINE BEFUNDE

Empirisch verifiziert: Signal-Abbruch dicht (kein Doppel-Trap, kein Weiterlauf nach Ctrl-C,
rc=130, Lock weg, Herzschlag tot), $$-Waisenpruefung expandiert in der bg-Subshell korrekt zur
Haupt-PID, Mehrfach-::-Filter mathematisch dicht, Statik-Symmetrie an allen 7 Reset-Stellen
identisch, sn-Label branch-konsistent (Kernel-Kommentare + beide Reports), Doku heil, alle
zitierten Hashes real. Rest-INFOs (kosmetischer gesamt-Nenner; TERM an die Skript-PID stoppt
erst nach dem laufenden Lauf -- Ctrl-C trifft die Gruppe sofort) dokumentiert, kein Fixzwang.

**Damit ist der Iron-Rule-3-Abschluss-Loop der Stufe 3 formal geschlossen** (3 Runden:
Befunde -> Fix -> Regression -> Nachpruefung, bis leer). Regressionsanker final: facetten_test
5x bestanden, Arm-1-Hash 12755646098055097704, Arm-3-Hash 887930967142844785 (beide exakt).
Stufe 3 ist inhaltlich (N1/N2-Abnahme 2026-08-16) UND formal abgeschlossen.

---

# J4: alpha-Massenkorrektur (2026-08-17, Commits 3fca9fd ff.)

**Iron-Rule-2-Kette komplett:** Planungsagent -> Implementierung -> unabhaengiger Pruefagent.

**Planer-Kernbefund** (vor der Implementierung): die naive Massenkorrektur q_i += w_i*alpha
injiziert selbst Impuls alpha*S1 -- an der Kugel von der Groessenordnung des Lecks. Loesung:
zweistufiger Messarm CFD_FAC_ALPHA (1 = nur Masse, 2 = + symmetrisches Rang-1-Downdate
G' = G - (6/S0)*B*B^T, eine Kovarianz und damit garantiert PSD; Solve erreicht sein
Impulsziel dann INKLUSIVE alpha exakt).

**Messstand (alle drei Planer-Prognosen eingetroffen):**
- CPU Kugel DX=25: Delta-m 458,7 -> -1e-6 (St.1) / -5e-6 (St.2); Normal-Rest 0 -> -375,9
  (St.1: der vorhergesagte alpha*S1-Impuls!) -> -1e-6 (St.2). Cd-Summe -0,04 -> +0,49 (St.2).
- Ebene Wand: beide Stufen BITGLEICH zum Aus-Arm (Hash 887930967142844785) -- alpha ist dort
  konstruktiv exakt 0. Kontrollanker Arm 1/3 unveraendert exakt.
- GPU-Richter (Queue-Serie, DX=40/YWMIN=0,15/SATGATE): Delta-m 272,9 -> -1,7e-5 (Arm 3+alpha2),
  -1,7e-5 (Arm 4+alpha2); Cd -0,62 -> +0,05/+0,04 (Vorzeichen erstmals physikalisch; Wert ist
  bei D/dx=11 eine Aufloesungsfrage, kein Erhaltungsdefekt mehr).
- Torus-N2 unter alpha2: 45 Grad cf 0,0011625 <= 0,00166 (Drift -1,5 % zum alpha-losen Arm),
  Delta-m exakt 0, alpha>ut = 0. rc=1 der Torus-Laeufe = bekannter K2-Stationaritaets-Befund
  (Offen-Punkt 7), arm-unabhaengig, kein alpha-Schaden.
- **26,6 Grad (j4q_t26a4a2b): ECHTER BEFUND.** Delta-m -14,9 -> +0,04 (360x kleiner), ABER
  cf 0,0017138 -> 0,0019942 -- ueber dem N2-Band (<= 0,00179). alpha>ut = 49,5 Mio (45 Grad: 0)
  -- die K4-Ring-Zellen (m0-Lage y_w=0,188, stark unsymmetrische Linkmengen) treiben grosse
  alpha-Aktivitaet. Ehrliche Lesart: der alpha-lose 26,6-Grad-PASS von Stufe 3 stand teilweise
  auf dem Massenleck (eine unphysikalische Senke); leckfrei liegt das Modell dort 11 % ueber
  der BB-Basis. **Attribution (j4q_t26a4a1, Stufe 1):** cf 0,0016455 (im Band!),
  Delta-m -0,0008, ABER Normalkontamination -35.523 -- der von Planer/Pruefer vorhergesagte
  alpha*S1-Impuls der Stufe 1, ueber den langen Lauf massiv. FAZIT: BEIDE bisher "bestehenden"
  26,6-Grad-Arme lehnten an je einer unphysikalischen Kruecke (alpha-los: Massensenke -14,9;
  Stufe 1: Normalinjektion -35k). Der vollstaendig erhaltungstreue Arm (Stufe 2) liegt 11 %
  ueber der BB-Basis -- das ist jetzt DIE offene Physikfrage der K4-Ring-/m0-Lage-Behandlung
  (y_w=0,188), nicht der alpha-Mechanik (verifiziert; 45 Grad haelt N2, alpha dort inaktiv).
- Zellgenaue Mechanik-Bestaetigung: 6912 Einzellink-Zellen wandern unter St.2 exakt von
  Slot 15 (gekoppelt Rang-0) nach Slot 13 (Entkopplung) -- vom Pruefer VORHERGESAGT, im
  Zaehler-Messstand exakt eingetroffen.

**Pruefagent (auf 3fca9fd):** Mechanik korrekt -- Kovarianz-Identitaet, Impulsbilanz,
sn-Nullung, Reihenfolge (alpha nach Gate/Klemme/EMA), Degenerations-Waechter dicht, S0>0 an
allen erreichbaren Stellen, Emission/Puffer symmetrisch (9 Statik-Stellen), Aus-Arm bitgleich.
1x LOW (Legende "18 Slots" -> 19) + 2x INFO (fac_diag-Kommentar 16->18; Slot 4 misst das
FORMEL-Residuum, nicht das Populations-Residuum -- beide ~1e-6-Rauschbett, als "ulp-Niveau"
lesen) -- alle gefixt. Werkzeugfalle Variante 3 dokumentiert und memoriert: get_opencl_c_code()
laesst nach #if nur EIN Token zu -- mehrgliedrige #if defined(A)&&!defined(B) sind im
R()-System unmoeglich, nur #ifdef/#ifndef verschachteln.

## K4-Ring-Etappe (2026-08-17 abends): Diagnose ABGESCHLOSSEN, Mechanik entlastet

DIAGZ-Zeitreihen an je einer Zelle aller drei 26,6-Grad-Lagen (Census: exakt 3 x 10.620
Facetten bei y_w = 0,187 / 0,698 / 1,069; Zellindex jetzt Spalte n im Histogramm-CSV):
- **Ring 0,187:** KEIN Flicker -- G' = 0,68/0,95 (gesund), s moderat (kein Rail bei +-2ut),
  keine Gates. Mechanismus: das Downdate senkt Snn 1,0 -> 0,141 und macht die vorher
  ENTKOPPELTE Zelle GEKOPPELT (sn +-0,008 statt exakt 0) -- noetig, um die Normalkomponente
  des alpha-Impulses exakt zu nullen. alpha +-0,02..0,04 folgt den P-Fluktuationen.
- **Lage 0,698:** Rang-2-Pfad (s2 == 0), G' = 0,33/0,11/0,03 -- klein, aber kein Rauschen.
- **Lage 1,069:** Kette leer -> BB-belassen (Slot-13-Population, 53,2 Mio Ereignisse).
FAZIT: die 11 % ueber BB-Basis sind KEIN numerischer Defekt, sondern der strukturelle Preis
exakter Massen- UND Impulserhaltung am Ring (Regime-Wechsel entkoppelt->gekoppelt plus
Rang-2-Anwendungen). Offene PHYSIK-Frage (Messprogramm, nicht Bugfix): N2-Band vs.
erhaltungstreues Modell am 26,6-Grad-Torus -- Kandidaten: Klemmskalen-Messarm,
Band-Neueichung mit erhaltungstreuer Referenz, Spalding-Ziel der Ringlage (Y bei y_w=0,187).

---

# Komplett-Audit 2026-08-17 abends -- drei unabhaengige Pruefer auf 76cd05c (Iron Rule 3)

**Gesamturteil: KEIN HOCH-Befund.** Alle iMEM/alpha-Formeln gegen die Gleichungsnummern
nachgerechnet, Positionsarithmetik aller 4 Bind-Kombinationen, Esoteric-Pull-Paritaet,
Paartabellen aller 6 Orientierungen, TYPE-Masken, Guards (kein NaN-Pfad unter fast-math),
J4/K4-Ring-Zahlenkette log-exakt verifiziert (inkl. GPU-Gegenprobe Slot-Wanderung 7791).

## Befunde und Nacharbeit (alle im selben Zug gefixt)

MITTEL (3):
1. Pruefer 2: dd-DIAG ueberschrieb den Nahfeld-Census mit dem Fernfeld-CSV (gleicher Dateiname)
   -> je Unterordner nah/fern; analog facetten_test T2-iMEM -> t2imem/.
2. Pruefer 2: die <16-Samples-_exit-Falle (kugel/fahrzeug/dd) verschluckte Dichteklemme,
   K4-Neutralitaet, Facetten-Ist=Soll und Delta-m bei Kurzlaeufen -> Statistik wird jetzt
   gegatet (stat_ok), alle Pruefpfade laufen immer (Smoke-Test-verifiziert).
3. Pruefer 1: Slot 8 zaehlte unter PEMA die VERWORFENE Kopf-Kette statt der angewandten
   gefilterten -> Kopfzaehlung #ifndef PEMA, angewandte Kette zaehlt im Filterblock.

Dazu MITTEL-Doku (Pruefer 3): KIPP-Tabellenzeile behauptete "Nur mit CFD_FACETTEN" -- die
eigenen BB-Basis-Referenzen t45_bb/t26_bb waeren danach unmoeglich -> korrigiert; und
CFD_FACETTEN_DIAG war im Kugelfall stummer No-Op -> verdrahtet (Census-Arm + DIAG=2-Exit,
auch im aktiven Arm).

NIEDRIG gefixt: Kugel-Ansagen (SATGATE/EMA/PEMA) nachgezogen + 9. Statik s_fac_diagz;
DIAGZ-Rebind haengt jetzt am Konstruktions-Zustand statt an der Statik (Use-after-free-Klasse);
tote hist_r21/r10 + Erstpass-Reste entfernt; Slot-7-Soll mod 2^32 (uint-Wickel am
Fahrzeugmassstab); stale 16er-Legenden (lbm.hpp/kernel.cpp) -> 18; Queue: Geraet per
CFD_QUEUE_DEV uebersteuerbar (Einfrier-Regel!), gesamt zaehlt Kommentar-::-Zeilen nicht mehr;
Klassenliste Paragraph 1.1 (Bit 16 doppelt gelistet, Klasse 32 fehlte); Anker-Logs jetzt im
Repo (logs/anker/, N6-Archivluecke). LATENT dokumentiert (Kommentare im Code): PEMA-Warmstart-
Selbstblock unter SATGATE, EMA x SATGATE ungeprueft angewandter Filterwert, ALPHA2-setzt-
ALPHA-voraus, env_f-atof-Stummparsing, fac_diag[14] ab t>2^24, kanal_profil unter kipp schief.

**Regression nach dem Fix-Batch:** facetten_test 5x bestanden, Arm-1-Hash
12755646098055097704 und Arm-3-Hash 887930967142844785 exakt; Kugel-Kurzlauf-Smoke-Test
druckt jetzt Pruefpfade + Delta-m (-1e-6). Runde 2 (Nachpruefer auf dem Fix-Diff) laeuft.

## Komplett-Audit Runde 2 (Nachpruefer auf Fix-Diff 79700b6)

1x MITTEL: der PEMA-Slot-8-Fix war nur halb -- der ERSTE Kopf-Increment (tw>tw_max) zaehlte
unter PEMA weiter (bis zu 2x je Besuch: verworfene + angewandte Kette). Gefixt: beide
Kopf-Increments #ifndef PEMA, Klemme selbst bleibt (fac_tau_acc braucht tw). Positivliste R2:
stat_ok-Klammerung an allen drei Stellen dicht (keine Division durch 0, keine verschluckten
Bloecke), Kugel-DIAG-Arme schliessen sich aus + baue_facetten verifiziert flag-read-only,
Rebind-Konstruktions-Gate deckt den Hart-Aus-Fall, Unterordner mit create_folder, Queue
set-u-fest, Slot-7-Modulo auch am Wickelpunkt korrekt, nichts Eingeschlepptes. Notiert:
Slot-8-Quoten sind zwischen PEMA- und Nicht-PEMA-Armen nicht 1:1 vergleichbar (max 1 vs. 2
Inkremente je Besuch); DIAG==2 im aktiven Arm exitet vor dem census_n-Waechter (harmlos).
Regression nach R2-Fix: Tests 5x gruen, beide Anker-Hashes exakt.

## Komplett-Audit Runde 3 (Nachpruefer auf 9ffa588): KEINE BEFUNDE

Splice klammersauber (Muster byte-gleich zu bewaehrten Nachbarn, Emissionsmechanik verifiziert),
Nicht-PEMA-Arm tokenstrom-identisch zum Vorzustand, PEMA-Arm zaehlt nur noch die angewandte
Kette, R2-Protokoll faktisch korrekt, nichts Eingeschlepptes.

**Der Komplett-Audit 2026-08-17 ist damit nach Iron Rule 3 formal geschlossen** (3 Pruefer ->
Fix-Batch -> Regression -> 2 Nachpruefrunden bis leer). Kein HOCH-Befund im gesamten Audit;
Regressionsanker final: facetten_test 5x bestanden, Arm-1-Hash 12755646098055097704,
Arm-3-Hash 887930967142844785 (Reproduktion: logs/anker/).

## Kugel-Wertrichter: Aufloesungsleiter 2026-08-18 (kr_dx*, Arm 3+alpha2, YWMIN=0,15, SATGATE)

| D/dx | Cd-Hybrid (Summe) | Delta-m |
|---|---|---|
| 11 (DX40) | 0,046 | -1,7e-5 |
| 18 (DX25) | 0,344 | +2,3e-5 |
| 25 (DX18) | 0,355 | -6,7e-5 |
| 37,5 (DX12) | **0,436** | -3,3e-5 |
| DX12 BB-Basis | **0,717** (object_force, gueltig) | -- |
| DX12 Alt-Arm (alpha aus) | 0,330 | **+13.149** (!) |

Literaturband (FACETTEN-LITERATUR): subkritische Referenz Cd 0,45-0,5 (Achenbach); Bodennaehe
h/D=0,167 erhoeht eher bzw. laesst neutral (Tsutsui-Logik, mitbewegter Boden ohne direkte Kurve).
**Urteil: der erhaltungstreue Arm konvergiert monoton von unten an den unteren Bandrand (0,436
bei D/dx=37,5), waehrend die BB-Basis mit 0,717 ~50-60 % DRUEBER liegt. Erstmals ist am
Kugelfall belegt: Facetten+iMEM+alpha2 ist REALISTISCHER als reines BB.** Ehrliche Grenzen:
(a) Re_D=9,1e5 ist nominal superkritisch -- die Drag-Krise ist bei diesen Aufloesungen nicht
aufloesbar, das effektive Verhalten ist subkritisch (deshalb das 0,45-0,5-Band als Referenz);
(b) noch nicht voll konvergiert (18->37,5 hebt +0,08); (c) das Leck des Alt-Arms SKALIERT mit
der Aufloesung (272 bei DX40 -> 13.149 bei DX12) -- die alpha-Korrektur wird zum Fahrzeug hin
also WICHTIGER, nicht unwichtiger. F12/N3-Wertrichter-Frage damit beantwortet.

## APG-Messarm (270cd99 -> Korrektur): Pruefagent fing ZWEI Blocker

1x KRITISCH: mein Aufraeum-Edit brach den Baum (fehlendes // vor einem Sternchen-Kommentar) --
und ich hatte den Build-Fehler im tail-1 uebersehen: die 270cd99-Testzahlen liefen gegen das
VORHERIGE Binary. Lehre: BUILD-RC IMMER echoen, nie nur tail.
1x HOCH: der dp/ds-Schaetzer war GESPIEGELT -- j[opposite(ia)] ist der Nachbar bei n MINUS c_ia
(Rechenprobe des Pruefers: linearer Gradient ergab exakt -dp/ds). Der Arm hob tw im APG statt
zu senken; die 270cd99-Kugelzahl (+0,35) validierte die Zielphysik NICHT.
Beide gefixt; Retest auf verifiziert frischem Binary: Suite 5x, Arm-3-Anker exakt (Emissions-
Nulltest), Kugel DX40-Kurzlauf: APG=0 -0,003 / APG=1 +1,091, Slot 19 = 2829, Delta-m exakt 0,
kein RHO_CLAMP. Richtung und Stabilitaet damit belegt; kappa=1 uebersteuert am groben Kurzlauf
(Band 0,45-0,5) -- kappa-Eichung + Aufloesungsleiter = Messprogramm vor jedem Fahrzeug-APG-Arm.
Positivliste des Pruefers: Differenzform exakt 0 bei uniformem rho, Einbauort/Klemmen/Puffer/
Symmetrie/PEMA-Sperre alle verifiziert.

---

# Stufe-5-Erstserie s5b (2026-08-18 nachts, 500-ms-Deckel/Heiko): ABGESCHLOSSEN

| Arm | Cd (Fenster 0,2-0,5) | Cz | Zweck/Befund |
|---|---|---|---|
| aus (BB-Anker) | 0,818 (Spaet 0,76) | -0,41 | Anker; Drift-Rest bis ~0,4 s = Fernfeld-Erstflush |
| arm4 (Nullziel) | Druck 0,82 / Reib 0,01 | -- | Mechanik-Beweis: Ist=Soll 1,31 Mrd EXAKT, Far=0, Dm -0,114 |
| arm3+alpha2 (Hybrid) | **0,728+-0,10 (Spaet 0,71)** | -0,44 | **-11 % Richtung OF 0,599; Cz unveraendert = C7-These bestaetigt** |

Facetten-y+ Median 29,7 (2,11 Mio Facetten, q25 19,7/q75 46,4) -- Log-Schicht, Instrument
arm-konsistent. Dm-Gelbband geeicht: Rauschbett ~0,12 -> Schwelle ~1. Slot-Matrix zielunabhaengig
stabil (Rang2 317M, Gates 20 %, alpha 169M aktiv). OFFENER PRUEFAUFTRAG: Normal-Rest akkumuliert
linear, arm-UNABHAENGIG (-505 arm4 / -478 arm3) -- systematisch/geometriegebunden, ~1 Ordnung
ueber Torus-Niveau normiert; VOR Cz-Interpretationen klaeren. Async geklaert (kein Flush V1+V2,
NEO-Auto-Submit; echte Fenster = Sample-Transfers; flags-Cache-Fix drin, 4d04e1e).

## To-dos 2026-08-18
1. Normal-Rest-Diagnose (arm-unabhaengig -> Verdacht sn-Kaskade/Klemmen an Fahrzeugkruemmung;
   DIAGZ-artige Stichprobe + Slot-15/16-Attribution) -- VOR Cz-Aussagen.
2. APG: kappa-Eichung an der Kugel-Leiter (DX40/25/12, kappa 0,25/0,5/1), dann 4. Fahrzeuglauf
   s5b_arm3_apg (der Cz-Hebel).
3. Perf-Fixe aus der Async-Liste: clFlush nach run_async, Sync-Buendelung Extract, finish in
   drive_boundary streichen (je A/B mit fdinfo; flags-Cache erledigt).
4. Dm-Waechter scharf (Schwelle ~1 statt 1e-4*fac_N), K2-Stationaritaetsgate (Offen-Punkt 7).
5. Danach: Iron-Rule-3-Abschluss-Loop des Stufe-5-Bauabschnitts.

## NACHKORREKTUR (2026-08-18 frueh, Heiko-Rueckfrage): Arm 3 ~= Arm 4 im Hybrid!

cd_druck 0,715/0,718, cd_reib 0,013/0,0115 -- die ZIEL-Differenzierung (Spalding vs Null) ist
am Fahrzeug-Cd praktisch unsichtbar. Der -11-%-Sprung vs AUS-Anker ist damit NICHT dem
Spalding-Ziel zuzuschreiben, sondern Mechanismus+SCHAETZER-Differenz (AUS = object_force,
Arme = Hybrid; K4-Neutralitaet wurde am Fahrzeug NIE geprueft -- kugel-only). y+ 29,7 belegt
nur: erste Zelle in der Log-Schicht, Instrument ok. To-do-Liste ergaenzt:
0. (VOR allem anderen) Attribution des -11 %: CFD_FAC_K4-Neutralitaetscheck im dd-AUS-Arm
   verdrahten (Hybrid-Schaetzer an unbehandeltem BB) -- erst dann ist der Cd-Vergleich sauber.
   Und: warum differenziert das Ziel nicht? (20 % Gates + Klemmen bei y+~30 -- Budget pruefen.)
Mozaffari/APG ist wie geplant NOCH NICHT im Serienlauf (kappa ungeeicht) -- nach diesem Befund
ist er umso wichtiger: er ist der Arm, der wirklich differenzieren soll.

---

# Iron-Rule-3-Audit Stufe-5-Bauabschnitt (2026-08-18 nachts): KEIN Ziel-Verschlucker-Bug

Kernergebnis (alles NACHGERECHNET): Arm3~=Arm4 ist STRUKTURELLE PHYSIK, kein Defekt --
Spalding-Mehrbedarf |ds1| = 0,04 % des Gate-Budgets bei y+~30 (Gate kappt NICHTS); die
Ziel-Differenz ist konstruktiv auf die Reibungsskala (1,8 % von Cd) gedeckelt und durch 43 %
Rueckfaelle + Heck-Auslöschung auf die gemessenen 0,0015 gedaempft. ohneTang 16,8 % = STATISCHE
Einzellink-Population (G'=0 analytisch, by design) -- sitzt aber an den konvexen Abrisskanten:
dokumentiertes Abdeckungsloch. Normal-Rest -500 = EHRLICHES Konto realer Injektion (Identitaet
downgedatete Momente == roh+alpha*Bn algebraisch bewiesen), Rate 8e-3/Schritt = 4 Ordnungen
unter der Cz-Kraftskala -- keine Gefaehrdung. Befunde gefixt: cd_reib-Semantik ehrlich
(residuendominiert, Zielanteil = Arm-Differenz), Slot-13-Kommentar, [4]/[5]-Fenster-Snapshot
(dm/rest jetzt Warmup-bereinigt), APG-Klemmen-Census als NIEDRIG-Rest notiert (Slot-8-Luecke
bei guenstigem Gradienten), Slot-7-Wrap bei 3x laengeren Serien notiert. Regression: Suite 5x,
Arm-3-Anker exakt. Einzellink-Census in baue_facetten + Kanten-Abdeckung = neue Messpunkte
fuer die APG-/Abloesungs-Etappe.

## Unterboden-Befund (Heiko, Slices 2026-08-18): tote Unterbodenstroemung

Census-Gegenprobe: Abdeckung ist NICHT die Ursache -- z=4-60mm: 73 % aktive Facetten (26,7 %
markiert, davon 10 % MS-Saum Fahrbahn/Latsch); 64-160mm: 86 % aktiv. Verdacht damit: STAGNIERENDES
FLUID unter dem Wagen (der bekannte Architektur-Punkt, gegen den der kurze Einlauf gebaut wurde)
-- erklaert direkt Cz -0,44 statt -1,301, denn ohne Durchstroemung kein Abtrieb, wandmodell-
unabhaengig. TO-DO (vor APG-Fahrzeuglauf): Unterboden-Durchstroemungssonde (mittleres u_x im
Spalt unter dem Boden laengs des Fahrzeugs, je Arm in die CSV) + A/B AUS vs Facetten-Arm:
ist der Unterboden auch im AUS-Arm tot (-> Architektur/Einlauf/Fernfeld-Erbe, vorbestehend)
oder erst im Facetten-Arm (-> Mechanismus)? Slices aller drei s5b-Laeufe liegen in export/.

## S5-Loop Runde 2 (Nachpruefer auf 1c0d712/e58a03c/d67a7f5): keine schweren Befunde

4x NIEDRIG gefixt: Sonde ueberspringt Ueberhang-Saeulen (>0,35 m zdach -- Spiegel/Fluegel
zaehlten freie Seitenstroemung als Spalt), MS-Schicht-Ausschluss kommentiert (bewusst),
dm-Semantik etikettiert (Endreport kumulativ vs. CSV Fenster-Delta) inkl. FACETTEN.md-Zeile,
Gelbband-Notiz nennt den Semantikwechsel. Positivliste: Fenster-Snapshot beweisbar korrekt
(monotones +=, Einmal-Zweig), K4-Vergleich fair (identische Zellmengen), Slot-13-Kommentar
faktisch richtig, nichts Eingeschleppt. Regression: Suite 5x gruen.

## APG-kappa-Eichung (2026-08-19 frueh, Kugel-Leiter): kappa = 0,5

| kappa | DX25 (Basis 0,344) | DX12 (Basis 0,436) |
|---|---|---|
| 0,25 | 0,410 | 0,466 (Band) |
| 0,5 | 0,490 (Band) | 0,498 (Bandkante) |
| 1,0 | 0,692 (drueber) | -- |

Monotone, gut konditionierte kappa-Antwort; Delta-m ueberall ~1e-5; APG-Klemmen stabil ~39k.
kappa=0,5 ist an BEIDEN Sprossen band-gueltig (0,45-0,5 Achenbach) -> geeichter Wert fuer
Fahrzeuglauf 4 (leichte Aufloesungsabhaengigkeit dokumentiert: feiner -> frueher an der
Bandkante; kappa=0,25 als konservative Reserve).

## Kappen-Revalidierung + Unterboden-Attribution (2026-08-19 mittags)

**APG-Ernuechterung:** Mit ehrlicher relativer Kappung [0, 2*tw] faellt der Kugel-APG-Effekt
auf die Basis zurueck (DX25 0,345 / DX12 0,439 vs. Basis 0,344/0,436) -- der ungekappte
Band-Treffer (0,490/0,498) lebte von Korrekturen >> tw, also AUSSERHALB der linearen
Duennschicht-Gueltigkeit. Die kappa-Eichung von heute frueh ist damit als Ueberdehnungs-
Artefakt einzuordnen; im Gueltigkeitsbereich hat der lineare APG-Term wenig Autoritaet.
Fahrzeug-Test des gekappten Terms laeuft trotzdem (anderes Regime: echtes Heck-APG bei y+~30).

**Unterboden-Attribution (Agent, alle Zahlen quellenbelegt): VERSORGUNGSBEGRENZT, kein Defekt.**
Aufstand (Z_OFFSET=0) -> Nasenschlitz ~25-30 mm, Reifen dichten die Korridore; Maximal-
versorgung rechnerisch u_rel<=0,30, gemessen 0,247 (arithmetisch geschlossen). Q-Profil:
Einbruch an der VORDERACHSE (-53 % seitliche Leckage), der bewegte Boden laedt hinten wieder
auf (Q 0,91->1,42). Moving-Floor-Mechanik ENTLASTET (Impulsuebertrag exakt 0,025/Schritt,
0 % Abweichung, beide Domaenen). Fernfeld sekundaerer Verstaerker (Spalt <2 Grobzellen).
Entscheidender A/B: CFD_Z_OFFSET_MM=16 (V1-Schwebe; V1-Referenz sah mittig 1,14 u_inf).
**WIDERLEGT (Gross-Audit Pruefer 3 + eigene Voxelmessung + Heiko real 50-55 mm): die
"25-30-mm-Schlitz"-Zahl war falsch -- STL-Buglippe 80-85 mm, Voxel-Unterboden 68 mm. Die
Versorgungs-These bleibt qualitativ (0,247 gemessen, -53 % Vorderachs-Leckage), die
Arithmetik dazu ist hinfaellig; Moving-Floor-Entlastung und Q-Profil gelten weiter.**

---

# GROSS-AUDIT 2026-08-19 (Heiko-Auftrag: ALLES, 3 Pruefer) -- Runde 1

**HOCH (Pruefer 1, unabhaengig nachgerechnet und verifiziert): Spalding-Konstante um Faktor 10
falsch.** kernel.cpp:1594 hatte emkB = 0.010517092 = (e^0.1-1)/10 (Uebertragungsfehler) statt
exp(-0.41*5.5) = 0.104874 -- effektiv B = 11,11 statt 5,5, tau_w damit 16-38 % zu klein in ALLEN
Wandmodell-Pfaden (WFB, Paararm, iMEM, PEMA) seit dem WFB-Bau. Die dokumentierte 1e-13-
Genauigkeit war selbstkonsistente Bisektion derselben Gleichung -- konstantenblind. GEFIXT.
KONSEQUENZEN: (a) alle Spalding-basierten Messwerte (Kanal-cf-Arm3, Kugel-Leiter, APG-kappa-
Eichung, Fahrzeug-Arm3) sind mit B=11,11 gerechnet und werden neu vermessen; (b) Stufe-3-N2-
Abnahme UEBERLEBT (Arm 4 = Nullziel ist Spalding-unabhaengig); (c) der historische -68-%-
Kanalbefund ist zu einem Grossteil hierdurch erklaert (-38 % allein bei Y~2400).
NEUE REGRESSIONSANKER (B=5,5; Kontrollarm 0 zusaetzlich): Arm1 1367576470905661998,
Arm3 4032664999240533470, Arm0 14650994849991271562 (logs/anker/*_B55.log); Suite 5x gruen.

Weitere Befunde der drei Pruefer (Fix-Batch laeuft): MITTEL Kernel: isfinite-Waechter unter
finite-math toter Code (Bit-Test noetig), transfer_F/graphics ohne F-BBox-Bewusstsein (latent,
Guards); MITTEL Host: Torus-Kanalprofil falsch normiert+unetikettiert, NaN-/Einfrier-Waechter
+ CSV-Flush fehlen in kugel/fahrzeug; MITTEL Doku: 25-30-mm-Schlitz-These widerlegt (STL
vermessen: Buglippe 80-85 mm -- konsistent mit Heikos 50-55 mm real und 68 mm Voxel),
Fenster-Etikett s5b (-11 % ist fensterrein -8,6 %), CFD_COARSE_NU existiert nicht (FERN_NU);
MITTEL Zusammenspiel: PO_FACES still ignoriert in dd/fernfeld, diagz-Spaet-Lese-Pfad (latent),
units-Global im Zwei-Einheiten-Fall (latent). Dazu ~20 NIEDRIG.

## Gross-Audit Fix-Batch Runde 1 (nach dem Spalding-Fix)

GEFIXT: isfinite->Bit-Test im Lift-Waechter (unter finite-math war er toter Code); NaN-/
Einfrier-/Explosions-Waechter jetzt auch in kugel (auf F_lat) und fahrzeug (inkl. Teilreihen-
Dump forces_abbruch.csv); Torus-Profil-CSV traegt Kipp-Warnetikett (Normierung + zw vertikal);
PO_FACES-Ansage im dd; kanal-DIAG-Census in eigenen Unterordner (Ueberschreib-Falle); tote
Erstpass-Pushes wirklich entfernt; APG-Quantisierungswaechter (kappa<5e-7 = harter Fehler);
Doku: CFD_COARSE_NU->FERN_NU (existierte nicht!), DIAGZ/CD_EVERY/Slot-19-Legenden, s5b-Fenster
fensterrein (-8,6 % statt -11 %, dazu B=11,11-Vorbehalt bis zur Neumessung).
DOKUMENTIERT-LATENT (Folgepunkte, kein Fix noetig heute): transfer_F/graphics ohne F-BBox-
Bewusstsein (nur Multi-GPU+BBox, Guard-Kommentar folgt), diagz-Spaet-Lese-Pfad, units-Global
im dd, s_sparse_T ohne Read-once, SPALDING_IT-Doppel-Default (explizit 0 -> still 1),
utau_ist aus f_akt, facetten_test ohne sichere_lauf/Ansagen, Queue-stdin/</dev/null, WFB-Slots
3-5 ungegatet, Arm4xAPG-tw-Verfaelschung des y+-Reports, fernfeld-PO_FACES-Ansage.
Regression nach Batch: Suite 5x gruen, NEUE Anker exakt (Arm1 1367..., Arm3 4032...).

## Gross-Audit Runde 2 (auf 1f2dac1+e98d369): 6x NIEDRIG, alle gefixt

Spalding-Genauigkeitskommentar neu vermessen (it=3: -0,44 % bei Y~2400, -4,4 % bei 1e4, <1e-4
im Fahrzeugbereich Y<=330 -- alte Zahlen galten fuer die falsche Konstante); Kugel-Waechter
Explosions-Zweig nachgezogen; rho ebenfalls bit-getestet im Lift-Waechter; Arm0-Ankerlog
archiviert; static-Waechter-Latenz kommentiert. Positivliste R2: Konstante/Newton-Stabilitaet/
Bit-Test/Waechter-Rahmen/alle Batch-Fixes verifiziert, nichts Eingeschleppt.
Regression: Suite 5x, Arm-3-Anker exakt.

## Gross-Audit Runde 3: 2x NIEDRIG (Kommentar-Etiketten, Kugel-Teilreihen-Dump) -- gefixt.
**Der Gross-Audit-Loop (Heiko-Auftrag: ALLES) ist damit formal geschlossen** (3 Pruefer ->
Spalding-HOCH-Fix -> Batch -> 2 Nachpruefrunden). Anker final (B=5,5): Arm0 1465099484999127
1562, Arm1 1367576470905661998, Arm3 4032664999240533470 (logs/anker/*_B55.log). NAECHSTE
PFLICHT vor neuen Wert-Aussagen: Neumessung aller Spalding-Eichungen (Kanal-Arm3-cf,
Kugel-Leiter, APG-kappa) mit B=5,5.


## NIEDRIG-Komplettabbau (Heiko: keine Latentliste!) -- 22 Fixes in einem Batch

WFB-Slots 3/4/5 jetzt t%100-gegatet (uint-Wickel), transfer_F/graphics mit Guard (F-BBox+
Multi-GPU = harter Fehler statt stillem OOB) + Latentkommentaren, diagz-Spaet-Lese-Pfad
GESCHLOSSEN (Konstruktionswert eingefroren), s_sparse_T read-once-symmetrisch, SPALDING_IT=0-
Klemm-Ansage, utau_ist-Etikett, PO_FACES-Ansage auch fernfeld, facetten_test-Ansagen
(Env ignoriert), Arm4xAPG-Diagnose-Warnung, main_setup-Kommentar vollstaendig, Queue:
stdin=/dev/null + atomarer noclobber-Lock (TOCTOU) + exakter gesamt-Zaehler, SURFACE x
SPARSE_TILES = #error, alte Schlitz-These im Protokoll als WIDERLEGT markiert.
Regression: Suite 5x, beide B55-Anker exakt.

## OF13-Unterboden-Referenz (Agent, sampleY0 t=1200) + Bodenband-Messarm

**Referenz-Durchbruch:** OF13 stroemt im selben Spalt (aufstehender Wagen, Reifen -3 mm) mit
**1,18 u_inf im Mittel** (Nase 1,39, VA 1,19 OHNE Einbruch, Min 1,087 Wagenmitte, HA 1,25 --
klassischer Venturi-Bodeneffekt = die Cz-Quelle). Unser LBM: 0,247 = Faktor 4,8 zu langsam,
mit -53 %-VA-Einbruch. Meine "ehrliche Physik des tiefen Wagens"-Deutung ist damit WIDERLEGT --
die Unterboden-Versorgung ist ein Setup-Defekt (Fernfeld-Erbe: Bodenschicht am x-Einlass 0,63
statt ~1,3; V1-Einzelgitter hatte 1,14). Boden-BC identisch (OF13 bottom fixedValue 30 0 0).
**Neu: CFD_KOPPLUNG_BODENBAND=N** (unterste N Grobzeilen der x--Einlasskopplung = u_inf,
Rampe bis 2N, nur u, rho bleibt Fernfeld; 0 = bitidentisch alt). CPU-Mini-A/B: Unterboden
0,304->0,331 allein durch die Einlasskorrektur -- Mechanismus bestaetigt. **V1-Vorlaeufer (Heiko): apply_floor_velocity** -- dieselbe
Defektklasse; dessen Reparatur bewegte in V1 Cz um 31 % (Commit-Historie). V2-Unterschied:
Nahfeld-Bodenmechanik ist sauber, das Defizit kommt ueber die x--Einlass-Kopplung herein --
der Bodenband-Arm setzt am Leck an statt flaechig am Boden. Dazu geplant:
Z_OFFSET=-3-Arm (OF13-Reifeneinsenkung) und Boden-Laengsprofil-Instrument (drin).

## WM-Tiefenblicke A-D (4 Agenten, 2026-08-19 nachmittags) -- Kernergebnisse

A (Physik): Formulierung KORREKT (ersetzen, nicht addieren -- verifiziert), y+ 11-100 = Standard-
WMLES-Regime; First-Cell-Sampling kostet +-10-20 % tau_w (=0,2-0,4 % Cd) -- Bewertung auf
Abloesung/Cz umstellen. Kugel-Arm4-Leiter = Schluessel-A/B (ist Spalding-Ziel noetig?);
van-Driest bei y+~30 wieder legitimer Messarm (alte Widerlegung galt y+=137).
B (Neuherleitung): Kette mathematisch geschlossen -- faca 45 Grad exakt, Spalding-Y einheiten-
rein, P1 = korrektes DEVIATORISCHES Ziel, Vorzeichen konsistent, Klemmen B-fix-fest. Keine Bugs.
C (Mess-Pipeline): (a) Facetten-y+ SELBSTREFERENZIELL (Modell-tau), physikalische Zelle-1 bei
y+ 70-140, Modell-tau ~5x unter OF13 (Anteil Konstante vs. Unterboden: B55-Neumessung); (b)
messe_yplus am Fahrzeug absolut TOT (druckkontaminiert, 180 Pa); (c) Kanal-cf nach B=5,5
KOMPLETT unvermessen (alte -68-%-Tabelle = B=11,11); (d) Cz lief ohne SEM (ehrlich >=0,03 --
GEFIXT: Cz-Block-SEM im dd-Report); (e) OF13-Korridor fuer perfekten LBM-Lauf: Cd 0,58-0,70,
Cz -1,1..-1,45 -- unser Cz -0,38 erklaert KEIN Vergleichbarkeitsterm, nur der Unterboden.
D (Interaktionen): kein unentdeckter Doppelzaehl-Konflikt; SGS_WANDFREI-Gate-Luecke an
Diagonalzellen GEFIXT (18 Nachbarn); APG+PEMA-Nichtkommutativitaet korrekt host-gesperrt.
Serie S5e definiert: Kanal-cf-B55 (BB/Arm3/SGSFREI), Kugel-Arm4-Leiter, K4-dd produktiv.

---

# Messtag-Protokoll 2026-08-19 abends (Serien S5d/S5e, vor dem Tiefen-Audit)

**Kanal-cf B=5,5 (erste gueltige Wandmodell-Zahl):** Arm 3 cf = 0,00154 (alt 0,00107-0,00114
mit B=11,11 -- der Konstantenfix lieferte exakt die prognostizierten +40 %). Referenz 0,00344:
vom historischen -68-%-Befund bleiben ehrliche -55 % = Turbulenz-Selbsterhaltungsproblem.
BB-Basis 0,01067. SGS_WANDFREI am Kanal GESCHEITERT (cf-Kollaps 9,7e-5, K2 disqualifiziert
ordnungsgemaess) -> van Driest ist der verbleibende nu_t-Messarm.
**Kugel-Arm4-Leiter: 0,342/0,436 == Arm 3 (0,348/0,446).** Blick-A-Schluesselfrage beantwortet:
das Spalding-Ziel ist an der Kugel ENTBEHRLICH -- der gesamte 0,72->0,44-Gewinn ist Mechanismus
(Erhaltung + BB-Ersetzungsstruktur). Nullziel-Arm = billigerer gleichwertiger Standard-Kandidat.
**K4 produktiv am Fahrzeug:** 1,4e-5 rel. bei 294 N -- Schaetzer-Paritaet am vollen Massstab.
**Fahrzeug B=5,5 (s5d_arm3_b55):** Cd 0,732 / Cz -0,446 -- identisch zu B=11,11; Modell-tau
stieg (y+ 29,7->37,4 = tau +58 %), Werte strukturbedingt unbewegt. Konstantenfix ist fuer
Kanal/Kugel/Ehrlichkeit relevant, fuer Fahrzeug-Cd/Cz neutral (Audit-Prognose bestaetigt).
**Bodenband-Serie:** hartes Band: Unterboden 0,25->0,67, Cd 1,07 (Strahl); max-Variante
(Defizit-Anhebung): 0,641 / Cd 1,031 / Cz -0,471 -- max==hart, weil Fernfeld im Band ueberall
unter u_inf liegt. LEHRE: reine u-Korrektur an der Kopplung hat eine nachgewiesene Grenze
(Kolbenprofil ohne konsistentes Druckfeld -> Staukraft). Beste Cz-Balance bisher:
Arm3+Band = Cz -0,526+-0,009 bei Cd 0,89. KONSEQUENZ: Offen-Punkte 8/9 (Far-Geometrie/
Nudging = druckkonsistente Bodenschicht) steigen von "spaeter" zu "der eigentliche Weg".
S5f-Screening (DX=8-Sprosse, ~7 min/Lauf) rankt Band-N/zoff/Kombis -- neue Werkzeug-Stufe
fuer Variantensuche (Heiko-Auftrag: kuerzere Testcases).

## TIEFEN-AUDIT 2026-08-19 nachts (3 Pruefer, nichts vorausgesetzt): Runde 1 komplett gefixt

Pruefer 2 (Host): 10 Befunde -- WICHTIGSTER: mein MultiGPU/F-BBox-Guard vom Mittag war TOTER
CODE (pruefte die read-once-genullte Statik; Pruefer 3 fand dasselbe unabhaengig als HOCH-No-Op)
-> Member-Pruefung. Dazu Laengsprofil-DX-Etikett, x- zuletzt treiben, fmax-Ansage, awk-exakt,
yb-Klemme, Latent-/Inertnotizen.
Pruefer 3 (Zusammenspiel/Doku): Band-Naht wanderte durch B5 nur zu den y-Verifies -> KANTEN-
LIFT der geteilten y-Spalten; Falsifikationslauf: ALLE vier Verifies 0,00000000 unter aktivem
Band. Doku-Wahrheiten: BODENBAND+SGS_WANDFREI in die Schaltertabelle, toter 12755...-Anker als
HISTORISCH etikettiert (B55-Anker massgeblich), Slot-19/fac_diag-Legenden, APG-Zeile auf
GEPARKT (Kappen-Ernuechterung), P9-Prioritaet, Kopf-Stand; K4-Soll ehrlich 5e-5 (Atomik
skaliert mit N); e_cf_arm3-rc=1-Vorbehalt (K2/P7) nachgetragen; messe_yplus-dd-Etikett
(druckkontaminiert). 15/15 nachgerechnete Messtag-Zahlen exakt.
Pruefer 1 (Kernel): KEINE HOCH-Befunde; SGSFREI-Kollaps = PHYSIK (Gate korrekt, K2 fing die
Nichtschliessung); WFB-Gating bricht keinen Test (Soll-Formel + Anker 1798320 verifiziert);
Spalding-Neumessung numerisch reproduziert. 5 NIEDRIG gefixt: SGS-Kommentar/Ansage 18er,
APG-Nachklemm-Zaehlung Slot 8, APG-x-PEMA-Sperre in den KONSTRUKTOR (setup-unabhaengig),
WFB-Report-Sampling-Etikett.
Regression nach Batch: Suite 5x, Arm-3-B55-Anker exakt, Band-Naht-Falsifikation 4x 0,0.

## Tiefen-Audit Runde 2 (auf a618a00+4279d9b): 3x NIEDRIG, gefixt

Slot-8-Kopfzaehlung jetzt auch unter APG abgeschaltet (bis 3x je Besuch moeglich gewesen --
Spiegel des PEMA-Musters); bb_n auf < cez/2 geklemmt (z+-Verify-Schein bei sinnfreiem Band);
K4-Tabellenzeile um dd erweitert; Tippfehler. Positivliste R2: MultiGPU-Guard-Reihenfolge
bewiesen korrekt (Member vor allocate), Kanten-Lift-Index a=0 == x--Ecke verifiziert,
Konstruktor-Sperre erreichbar, alle drei B55-Hashes in den Ankerlogs wiedergefunden, awk
aequivalent. Regression: Suite 5x, B55-Anker exakt.

## Tiefen-Audit Runde 3: KEINE BEFUNDE -- **der ausfuehrliche Tiefen-Audit-Loop ist GESCHLOSSEN**
(3 Pruefer auf alles, nichts vorausgesetzt -> ~23 Befunde ueber 2 Fix-Runden, kein HOCH im
Kernel, 1 toter Guard als HOCH-No-Op gefunden+gefixt, Band-Naht geschlossen und falsifiziert,
Slot-8-Zaehlsemantik je Arm jetzt genau 1 Zaehlstelle. Anker final B=5,5, Suite 5x.)

## FERN-BODENKLEMME-Experiment (Heiko, 2026-08-20 frueh, DX=8): V1-Fix-Klasse WIRKT an der Quelle

f_fernklemme4 (Fernfeld z=1..4 TYPE_E u_inf): UB-Sonde 0,49, Profil MITTE 0,81 (Band8: ~0,6 --
die Klemme fuellt auch die y-Seitenraender nach, das Band nur x-), Diffusor-Abfall ab 2 m
bleibt (0,54 -> 0,33). Cz an der Screening-Sprosse nicht belastbar. NAECHSTER 4-mm-KANDIDAT:
Arm3 + FERN_BODENKLEMME=8 (bei 16 mm Far) gegen Arm3+Band8 (-0,526). Drei Slice-Agenten
(Punkte 1-6) laufen: Boden-Mechanik (warum baut das Fernfeld ueberhaupt eine unmoegliche
Boden-GS auf?), Einlassprofil/eckige Aufstauung, Dach/Akustik.

---

# SLICE-OFFENSIVE 2026-08-20 (6 Heiko-Befunde, 3 Agenten): ALLE AUFGEKLAERT

**P5+P6 (KERNBEFUND): Moving-Floor-Injektion landet in einer STAGGERED-MODE.** z=1 traegt ein
echtes Periode-2-Schachbrett (Autokorr. 0,96 @Lag2, Phasenwechsel je Frame), z=2 stirbt als
aktive Impulssenke; Defizitzone 4-6x dicker als eine STEHENDE Wand erzeugen koennte. Ermoeglicht
durch tau_grob=0,500007 (ungedaempft). Einlass-Ecken-Hypothese widerlegt; Render-Artefakt
ausgeschlossen. V1-Kur dokumentiert: apply_floor_velocity = Equilibrium-Reset mit LOKALEM rho
auf z=1..N (druckerhaltend; V1 kernel.cpp:1769-1800; wirkte auf exakt diese MS-Lage, Cz -31 %).
FIXPLAN (morgen, Bauabschnitt 1): (a) V1-Port CFD_BODEN_EQ=N fuer NAH+FERN (die TYPE_E-Klemme
war der Beweis-Arm, der Port die produktive Form), (b) Staggered-Daempfer (TRT/nu-Floor in den
2 Bodenlagen) als Wurzelbehandlung, (c) audit_bewegte_waende-Output nach out_dir persistieren.
**P1+P4:** Bandkante 112-128mm schneidet die Staublase (Defizit lebt <48mm) -> Band=3 oder
Referenzzeilen-Hub; Unterboden-Abbau = tote Waende + fehlender Diffusor-Sog, KEINE Leckage
(Q-Bilanz +-2 %); Near-Profil invertiert (z1=0,66>z2=0,51) = Boden zieht nicht; Couette-
Rechnung: mit ziehendem Boden Plateau ~0,9-1,0 -> OF13 1,087 VOLLSTAENDIG erklaert.
**P2+P3:** OF13-Abloese-Soll extrahiert (anliegend bis Heckscheibenfuss Nase+3,64m); Haupt-
verdaechtiger Dach = stiller YWMIN-0,2-K4-Ausschluss (15-38 % der Scheibenflaechen pures BB in
Treppenstreifen; dd-Warnung ergaenzt, D2-Lauf mit 0,15 laeuft). Akustik: Init-Puls (Cd-Start
48,7) + TYPE_E-Kavitaet (15,9 Hz ~ c/2L); CFD_SPONGE_N=64 war in allen s5d-Laeufen AUS -> an.
**Fern-Klemme-Experiment (Heiko):** wirkt an der Quelle (Profil@1,29 0,81 [Nase-Etikett, R2-Korrektur: Wagenmitte nur 0,600] vs Band 0,6 --
Seitenrand-Nachfuellung), Beweis der V1-Fix-Klasse am dd.

## BODEN_EQ-Leiter (V1-Port, 2026-08-20 nachts, DX=8) -- der Cz-Pfad steht

| Variante | Cd | Cz | Profil@x≈1,29 (Nase; R2-Etikettkorrektur — Wagenmitte deutlich niedriger: f_v1port3 1,044, OF13-Soll 1,087) | Urteil |
|---|---|---|---|---|
| Basis | 1,44 | +0,06 | schwach | Staggered-Senke |
| TYPE_E-Klemme 4 | 1,68 | +0,03 | 0,81 | Beweis-Arm |
| Port N=3 ohne Split | 3,87 | -0,73 | 1,073 | Physik-Beweis, Kraft-Artefakt (Heikos V1-Lehre live) |
| Port N=2+Split | 1,47 | +0,30 | 0,709 | KRAFTNEUTRAL bestaetigt; Senke kehrt stromab zurueck |
| **Port N=2/DOWN=1** | 1,93 | **-0,68** | **1,216** | **OF13-Profil hergestellt; Rest-Kontamination Reifen-adjazent** |

Heiko-Vorgaben eingebaut: damals N<=2 gefahren (Vorgabe: max 3, besser 2 -- R3-Etikett), x_split ab Nase (V1: "Fahrzeug stand AUF der Schicht" ->
V2-stehend: DOWN=1 laeuft bei 68 mm Freiheit beruehungsfrei unterm Wagen durch, Kontakt nur
Reifen-Nachbarn). 4-MM-KANDIDAT MORGEN: Arm3+alpha2+SATGATE + BODEN_EQ=2/DOWN=1 (+FERN dito),
Erwartung: Kontamination halbiert sich relativ (4 vs 8 mm Zwangszelle), Cz erstmals Richtung
-0,7..-0,9 bei ehrlicher Kraft-Etikettierung. Danach: YWMIN-0,15-Einzelarm (Dach), dann
P8/P9-Bauabschnitt (Far-Facetten + couple_n2f-Rueckkopplung) als druckkonsistenter Endausbau.

## Referenzlage + Konsolidierung (Heiko 2026-08-20 frueh)

**NEUE REFERENZ: Fahrzeug -3 mm unter z0 (OF13-Konvention, Reifen eingesenkt)** -- Default in
dd + Einzelgitter; CFD_Z_OFFSET_MM uebersteuert. Vertraeglichkeit CPU-verifiziert (Aufstand-/
Kontakt-/MS-Checks gruen, BODEN_EQ kompatibel); bei 16 mm ist die Fleckenzahl Subzellen-
Rauschen, wirksam ab 4 mm. ACHTUNG: damit verschieben sich die 4-mm-Anker (s5b_aus etc.) --
neue Basislaeufe noetig. Klein-Todos: BODEN_EQ-Schalterzeilen in FACETTEN.md, Band als
"durch BODEN_EQ ersetzt" etikettiert, Audit-Persist via Queue-Logs abgedeckt (logs/<lauf>.log
archiviert jede Konsole -- kein Refactor noetig), alpha-Kommentar entklebt.

## NEUSTANDARD VALIDIERT (f_neustandard, DX=8): zoff-3 + Arm3+alpha2+SATGATE + BODEN_EQ 2/1
OF13-Lage aktiv, Wirkpfad 147.166.750 = Soll exakt, Delta-m -0,003, Unterboden-Profil auf
OF13-Niveau ueber die volle Laenge (1,00/1,215/1,015/0,82), Cz -0,611 = bester Facetten-Arm
der Sprosse. DIES ist die 4-mm-Startaufstellung. XL-Audit-1-Fixes drin (tile_slot-Bindung,
dead-tile, u_road konfigurierbar); Pruefer 2+3 laufen, Schleife geht weiter bis leer.

## XL-Audit R1, Pruefer 3 (Zusammenspiel/Doku): 16 Befunde, alle DIREKT eingearbeitet
(Zaehlungs-Notiz R2: N14 = OFFENE-PUNKTE-P3.2-Vermerk, N15 = zref_mini-Evidenzluecke -- beide unter "Nachprotokolliert" bzw. in OFFENE-PUNKTE.md abgedeckt.)

Positivliste des Pruefers: boden_eq gegen Esoteric-Pull KORREKT hergeleitet (rho tauschinvariant,
u verworfen, store adressiert t+1 richtig); kein Facetten-Doppeleingriff (z=1 ist Klasse-64/BB);
Zeitschritt-Einbau sauber; Z_OFFSET-3-Vertraeglichkeit real; >15 Messzahlen exakt reproduziert.

Fixes dieser Runde (Commit siehe unten):
- M2: BODEN_EQ-Wirkpfad-Zaehler Slot 20 IM Binary (t%100), dd-Report mit Null-Fehler (Iron Rule 3).
- M3: dd-Ansage nennt jetzt DOWN/split/ABSTAND + Kontaminationswarnung.
- M1/M4: Anker-Verschiebung auch 8 mm dokumentiert (f_aus 1,444/+0,062 vs f_zoffm3 1,222/+0,208);
  BODEN_EQ-Leiter = Alt-Lage 0, NICHT gegen f_neustandard (-3) vergleichbar. FACETTEN.md: Z_OFFSET_MM-
  und FERN_BODENKLEMME-Zeilen ergaenzt, 7b ueberholt, P5/P0 gestrichen, Profil-Etikett (Nase!) korrigiert.
- N5-N13: alpha-Kommentar wirklich entklebt, kernel_boden_eq-Kommentar (schreibt NUR fi, pre-Reset-
  Anzeige), Einzelgitter-Lage-Ansage + Kommentarblock, fernfeld/kanal-Schluck-Warnungen, Kugel-MG0-
  Guard, Slot-Legenden 21, D>1 weiter latent (D=1 ueberall, B7-Kommentar im Kernel).
- NEU (Heiko 2026-08-20): CFD_BODEN_EQ_ABSTAND -- reifennahe Bandzellen (Chebyshev<=A zu TYPE_S,
  Ebene+oberhalb, Boden ausgenommen) werden ausgespart. UNGETESTET, A/B im naechsten Screening-Fenster.

### Nachprotokolliert (XL-3 N16, fehlten in den .md):
- f_zoffm3 (DX8, nur Z_OFFSET=-3): Cd 1,222 / Cz +0,208 (Basis f_aus 1,444/+0,062) -- der Referenzlage-A/B.
- f_d2_ywmin_klemme (D2; YWMIN 0,15 + Fernklemme 4 + Sponge 64 KOMBINIERT -- Arm-Disziplin-Verstoss,
  eingestanden): object_force-Cd 8,70 (phantombehaftet) / Cz +0,37. Nicht einzeln attributierbar.
- f_neustandard (DX8, zoff-3 + Arm3+alpha2+SATGATE + BODEN_EQ=2/DOWN=1 + FERN dito): Cd_druck 2,10,
  Cz_druck -0,611; object_force 9,48/-0,16 BEIDE phantombehaftet (Facetten-Arm!). Sonde 0,601,
  Profil@1,29 1,22. R2-KORREKTUR (H1): die fruehere "+0,15-Restluecke" verglich den PHANTOM-Cz
  (-0,16) mit dem gueltigen object_force-Cz der facettenfreien Leiter (-0,68) -- Schaetzer-Mischung,
  nicht belastbar. Schaetzerrein gilt nur: Referenzlage-Shift ~+0,15 fuer object_force-Paare
  (f_aus vs f_zoffm3); ein facettenfreier BODEN_EQ-Lauf auf -3-Lage als sauberer Anker fehlt noch.
- Evidenzluecke zref_mini: der -3mm-Smoke (CPU/DX16, dirty auf 5579a02) hat KEIN archiviertes
  Konsolen-Log -- "Checks gruen" ist nicht mehr belegbar. Lehre: auch Einzelaeufe ueber die Queue.

## Pruefagent R2 (ABSTAND+Zaehler-Diff): keine HOCH-Befunde, 1 MITTEL + Kleinigkeiten gefixt

Verifiziert: Argumentreihenfolge boden_eq 0..8 + tile_slot 9 exakt (keine diag/tile_slot-Kollision),
index()/uint3-Literal gueltig, Fahrbahn z=0 fuer den ABSTAND-Scan BEWEISBAR unerreichbar (dz>=0, z>=1),
(flags&TYPE_BO)==TYPE_S matcht auch TYPE_S|TYPE_X-Reifen, Statik-read-once an allen 9 Setzstellen.
Gefixt: MITTEL-1 boden_eq-Sparse-Bindungen aus #ifdef FORCE_FIELD gezogen (TS_P haengt nur an
SPARSE_TILES -- im BENCHMARK-Build waere B1/B2 sonst zurueckgekehrt); N1 fernfeld-Warntext (BODEN_EQ
wirkt auch kugel); N2 Kugel setzt down/split explizit (Statik-Symmetrie VOLL); N3 Kugel bekommt
eigenen Slot-20-Wirkpfad-Report. Bewusst belassen (N4): BODEN_EQ-Nullcheck steht VOR den Facetten-
Reports -- Disqualifikation darf Detail-Reports kosten; sehr grosser ABSTAND kann den Fehler ehrlich
ausloesen (Band komplett ausgespart = konfigurierter No-Op). N5: Slot-20-uint-Wickel Faktor 7-40
unter 2^32, praktisch irrelevant.
Regression dieser Runde: Suite 5x, B55-Anker exakt, Sparse+ABSTAND-Kombiproof gruen (Commit c091f0b).

## Performance-Audit (3 Pruefer, 2026-08-20, Heiko-Auftrag): Befunde fuer die naechste Baurunde

Konvergenz aller drei: (1) kraft_facetten liest volle F-Box (~2-2,5 GB PCIe) je Kraftfenster =
~12,5 % Gesamtlaufzeit in Facetten-Armen -- Fix: Markerzellen-Indexliste + GPU-Gather/Reduktion
(Muster object_sum). (2) UPDATE_FIELDS fest an = ~10-15 % auf beiden Gittern; Abloesung braucht
po/vi-Mini-Kernel UND Vorsicht: update_fields wuerde TYPE_E-Kopplungsraender aus fi klobbern
(pruefagentenpflichtiger Umbau, A/B). (3) Slices lesen ~6,5-9 GB je Ereignis fuer eine y-Ebene
(~10-15 MB noetig). (4) boden_eq: flags-Read VOR dem z-Test + Full-N-Launch (V1-Erbdefekt, 1-5 %).
(5) clFlush nach run_async fehlt (Overlap ist heute Treiberglueck). (6) FP16S statt FP16C =
~12,5 %-Kandidat, nur als A/B (Formatgrenzen!). Entwarnung: Zero-Copy iGPU korrekt, dd-Domaenen
laufen echt parallel (Fernfeld 0,2-0,4 % im Schatten), Kopplung 1,2 %, Sparse-Aus kostenlos,
Queue kompiliert nichts. V2-Index ~9.100-9.890 liegt schon ~24 % unter V1 (~12.000).
Reihenfolge naechste Runde: kraft_facetten-Reduktion -> Slice-Ebenen-Read -> clFlush ->
boden_eq-Early-out (bitidentisch) -> UPDATE_FIELDS/FP16S als validierte A/Bs (CPU->iGPU->B70).

## XL-Nachpruefrunde 2: R1-Fixes VOLLSTAENDIG verifiziert, Restbefunde direkt gefixt

Code-Pruefer: KEINE HOCH-Befunde, alle R1-Fixes korrekt (Sparse-Bindungen fuer alle Build-
Varianten stimmig, Argumentindizes 0..8+tile_slot=9, Slot 20 nullinitialisiert und bitgleich
im Kontrollarm, ABSTAND-Scan OpenCL-gueltig, Default-Pfad bitidentisch). Gefixt: Einzelgitter-
Fahrzeugfall verschluckte CFD_BODEN_EQ stumm (Warnschleife), Kugel-Warnungen (DOWN/FERN wirken
dort nicht, N>3-Hinweis), ABSTAND>3-Perf-Warnung (nah+kugel), Scan-Kommentar nennt jetzt die
geometrieabhaengige Annahme (diagonal-untere Solids ignoriert; konvexe Reifen decken das ab).
Doku-Pruefer: H1 = eigener Schaetzer-Mischfehler im f_neustandard-Nachprotokoll (Phantom-Cz
-0,16 mit gueltigem Leiter-Cz -0,68 verglichen) -- korrigiert: Cz_druck -0,611, Restluecken-
Attribution zurueckgezogen; schaetzerreiner facettenfreier BODEN_EQ-Anker auf -3-Lage FEHLT noch
(naechstes Screening-Fenster). Dazu: Slot-Legende 21 in FACETTEN.md, "Profil Mitte"-Etiketten an
allen Reststellen (x=1,29=Nase), In-place-Marker an P0/P5/7b, DIAGZ-Wahrheit (nur Kanal/Torus),
Kopfdaten, N<=3-Konsistenz, P3.2-Marker am Listenitem. Verifiziert log-exakt: f_zoffm3, f_aus,
f_d2, f_neustandard (Wirkpfad 147.166.750=Soll), Leiterzahlen.

## XL-Abschlussrunde 3: kein HOCH, 1 MITTEL + 5 NIEDRIG -- direkt gefixt, SCHLEIFE GESCHLOSSEN

R3 verifizierte alle R2-Fixes gruen (Warnungen feuern richtig und genau einmal, H1-Korrektur
widerspruchsfrei, read-once ueberall). Gefixt: FERN-Seite bekam N>3- und DOWN-ohne-EQ-Guards
(die FACETTEN-Behauptung "Code warnt ab N>3" gilt jetzt nah/fern/kugel); ABSTAND>3-Warnung nur
noch bei aktivem EQ; Einzelgitter-Warnliste + FERN_DOWN; kanal/fernfeld/facetten_test warnen
die ganze Familie; 1,045->1,044 (CSV-exakt); "damals N<=2 gefahren"-Etikett. Severity-Verlauf
der Schleife: R1 mehrere HOCH -> R2 1 HOCH (Doku) -> R3 nur Warntext-Asymmetrien. Regression
je Runde: Suite 5x, B55-Anker 4032664999240533470 exakt.

## Perf-Fixbatch 1 (die zwei bitidentischen Hebel aus dem Perf-Audit)

1. boden_eq: z-Band-Test VOR den flags-Load gezogen -- die Enqueue streamte sonst je Schritt das
   komplette flags-Feld (N x 1 B) fuer ein Band von <2 % der Zellen (V1-Erbdefekt, ~1-5 %).
2. clFlush nach run_async (Device/LBM_Domain::flush_queue) -- der iGPU-Overlap der dd-Kopplung
   hing bisher am NEO-Treiberverhalten, jetzt garantiert.
Pruefagent: KEINE BEFUNDE (Kommutierbarkeit der Early-outs fuer alle Zellklassen bewiesen, Flush
trifft die richtige Queue nach allen Enqueues, initialized-Guard deckt den Freigabe-Fall).
Regression: Suite 5x, Anker exakt, dd-Kombiproof mit EXAKT identischen Wirkpfad-Zaehlern
(1.851.472/207.646) -- Bitidentitaet am Objekt belegt. Offene Perf-Hebel (naechste Baurunde,
GPU-Leiter): kraft_facetten-GPU-Reduktion, Slice-Ebenen-Read, UPDATE_FIELDS-A/B, FP16S-A/B.

## Tagesabschluss 2026-08-20 / Plan 2026-08-21

Heute (Commits dc18ded..9d95eec): XL-Audit-Schleife GESCHLOSSEN (R1: 3 Pruefer, mehrere HOCH ->
R2: 1 Doku-HOCH -> R3: nur Warntexte); BODEN_EQ-Wirkpfad Slot 20 im Binary; Heiko-Reifenschutz
CFD_BODEN_EQ_ABSTAND eingebaut (UNGETESTET, A/B offen); Cz-Korrektur f_neustandard = Cz_druck
-0,611 (Phantom-Mischung zurueckgezogen); Perf-Audit 3 Pruefer + Fixbatch 1 (bitidentisch).
KEINE Produktion (Heiko-Ansage).

MORGEN, in dieser Reihenfolge:
1. 8-mm-Screening (Queue!): (a) facettenfreier BODEN_EQ-Anker auf -3-Lage (schaetzerreiner
   Vergleichspunkt, fehlt!), (b) ABSTAND-A/B (0 vs 2) auf -3-Lage. Erwartung: ABSTAND senkt die
   Reifen-Kontamination im Cd, Cz-Ordnung bleibt.
2. 4-mm-Produktionslauf Startaufstellung: zoff-3 + Arm3+alpha2+SATGATE + BODEN_EQ=2/DOWN=1
   (+FERN dito, ABSTAND nach A/B-Ausgang). Neue 4-mm-Basis noetig (Referenzlagen-Verschiebung).
3. Perf-Baurunde 2 (CPU->iGPU->B70-Leiter): kraft_facetten-GPU-Reduktion zuerst (~12,5 %),
   dann Slice-Ebenen-Read; UPDATE_FIELDS-Abloesung und FP16S nur als validierte A/Bs.
4. Danach offen: YWMIN-0,15-Einzelarm (Dach), P8/P9 (Far-Facetten + couple_n2f-Nudging),
   Unterboden-Abloesung (Arm3+Facetten am Pan, ggf. u_road-Drossel ~0,9).

## 2026-08-21 vormittags: Anker + ABSTAND-A/B (8 mm, -3-Lage) und CFD_NEAR_VOR_MM

f8_anker_m3 (facettenfrei, BODEN_EQ=2/DOWN=1 nah+fern): Cd 1,688 / Cz -0,559 (SEM Cz +-0,011),
Sonde 0,570, Profil 1,220@Nase / 1,048@Mitte. Referenzlagen-Shift konsistent (0-Lage: 1,93/-0,68).
f8_abst2 (einzige Variable ABSTAND=2): Cd 1,333 / Cz +0,176, Sonde 0,563, Profil 1,198/1,058.
BEFUND (Heikos V1-These bestaetigt): Stroemung praktisch identisch, Kraefte um dCd -0,36 /
dCz +0,74 verschoben -- die reifenadjazente Aufpraegung erzeugte ~-0,7 KUENSTLICHEN Abtrieb.
ABSTAND=2 ist ab jetzt der sauber gemessene Standard-Kandidat; alle Kraftvergleiche mit
ABSTAND=0-Laeufen sind kontaminationsbehaftet zu etikettieren. Der echte Cz-Weg laeuft ueber
Facetten + druckkonsistentes Fernfeld (P8/P9), nicht ueber das Artefakt.

CFD_NEAR_VOR_MM eingebaut (Heiko: Einlass 80-100 mm vor; Variante B: Box nach vorn verlaengert,
Heck weltfest; 96 mm = 6x16 = 3x32 mm zellausgerichtet). Planungsagent-Plan umgesetzt inkl.
dynamischem Sponge-Guard (NF_OX-32), Nahfeld-Einlauf-Print, Messsaeulen-Welt-x-Print.
Pruefagent: kein HOCH; M1 (Sponge-Guard bei N=0/Unterlauf) und M2 (NaN-Input lautlos, UB-Cast)
plus 4 NIEDRIG direkt gefixt. CPU-Smoke NEAR_VOR=96: Einlauf 0,326 m, Nachlauf 1,990 m
unveraendert, Kopplungs-Deckung gruen. Bitidentitaet bei NEAR_VOR=0 (+-0.0f) verifiziert.

## Perf-Baurunde 2, Baustein 1: kraft_facetten-GPU-Reduktion (CFD_FAC_GPU, Default AN)

Implementiert nach Planungsagenten-Plan: Markerzellen-Indexliste (einmalig beim Bind), Kernel
kraft_facetten_gpu mit 64er-Baum-Reduktion OHNE float-Atomics, Host-Endsumme double in fester
Gruppenreihenfolge; alter Host-Pfad wortgleich unter CFD_FAC_GPU=0; CFD_FAC_GPU_PRUEF=1 rechnet
beide und druckt Abweichung+Zaehler. Statt ~2-2,5 GB Voll-F-Transfer je Kraftfenster fliessen
<1 MB Partialsummen. Implementierungsagent fing selbst den Zero-Copy-Versatz (run() statt
enqueue_run()); Pruefagent fand das SPIEGELBILD im Host-Arm (finish vor F-Spiegel-Read) -- beide
Richtungen der Falle jetzt gefixt und als Lektion dokumentiert.
CPU-Abnahmen: Suite 5/5; Kugel-Doppellauf 11 Fenster Zaehler EXAKT gleich, px/pz auf 6 Dezimalen
identisch; K4 rel 3,5e-7 (Soll 1e-5); Kanal-K3 parallel: GPU wie Host exakt 0, Abweichung
0,00000000, B55-Anker exakt. Torus kipp=26-Kurzfenster: K3 faellt in BEIDEN Pfaden identisch
(fallseitig, vorbestehend -- kein Diff-Befund; langes Fenster steht als Klaerung aus).
AUSSTEHEND (GPU-Leiter): iGPU-PRUEF, dann B70-Index-A/B (Heiko: Perf-Optimierungen IMMER mit
dem Performance-Index vermessen) -- Soll: Kraefte-Phase ~37 % -> ~1 % im Messfenster, Index ~-10 %.

ERLEDIGT 2026-08-26 (g18/g19, Binary cb2675e+Slice-Baustelle): iGPU-PRUEF-Doppellauf an der
Kugel BESTANDEN (max. Relativabweichung px/py/pz 3,9e-6 -> 2,7e-7 ueber die Fenster, Soll 1e-5;
logs/g18_facgpu_pruef.log). B70-A/B am 8mm-Fahrzeug: bei CD_EVERY=4 KEIN messbarer Unterschied
(beide Arme 1478 MLUPs -- Kraftfenster zu selten, gute Nachricht fuer die Screening-Sprosse);
bei CD_EVERY=1 (4mm-Produktionskadenz, g19): Nahfeld-Phasenanteil 92,1 % (GPU) gegen 63,9 %
(Host), Wanduhr 4:58 gegen 5:44 min = ~13 % schneller -- die ~12,5-%-Prognose des Audits
bestaetigt. CFD_FAC_GPU=1 bleibt Default; Hebel ABGENOMMEN.

## ABNAHME f4_wandfrei_v2 (4mm-Produktion, 26.08. 13:15-14:47, Commit f3b4a7c, SAUBER bis t=0,5s)

Erster durchgelaufener gekoppelter 4mm-Lauf (Vorgaenger starb bei 0,405). 92 min inkl. Init
(Index ~10.700-10.840, ~13 % schneller als Vorgaenger-Index 12.429 = FAC_GPU+Slice-Gather am
Produktionspunkt). Profiler (V1-Workflow, fdinfo 180s): B70 CCS 97,2 %, iGPU 82,4 % (Zielband),
CONCURRENT 99,4 %. VTK: 10 Dateien/56,4 GB inkl. Enddump; Census 0; GT-Reset-Waechter ohne
Ausloesung; 0 Fehler.
ZAHLEN (Fenster 0,2-0,5, n=300): cd_druck 0,8968+-0,0095 | cz_druck -0,658 | cd_reib -0,009 |
cd_band 0,3588 | cd_rest 0,5380 | cz_band +0,2450 | cz_rest -0,9029.
BEFUND 1 -- RADKONTAKT-BAND-SCHRUMPF BESTANDEN: Band-cd 8mm 0,692 -> 4mm 0,359 (-48 %).
Die Deklaration "Artefakt grober Voxelraeder" ist damit BELEGT (Pflichtpunkt ext. Review).
BEFUND 2 -- KRITERIUM cz_rest <= -0,92 IM VOLLEN FENSTER VERFEHLT: 0,2-0,405 sind beide
Laeufe deckungsgleich (-0,929/-0,926), aber 0,4-0,5 (die der Vorgaenger nie erreichte) traegt
nur -0,851 -> Vollfenster -0,903. Das "erstmals erfuellt" vom 22.08. stand auf dem
abgeschnittenen Fenster. Kein Regressionsbefund, sondern niederfrequente Cz-Schwingung --
Kriterium kuenftig NUR auf dem vollen Fenster bewerten (Cz-Bewertung Vollumfang-Lauf!).
BEFUND 3 -- cd_druck-SCHAETZER-VERSATZ -0,05 ZWISCHEN CODESTAENDEN (faires Fenster: neu
0,9114 vs alt 0,9610) bei identischen Gesamtkraeften (Fx 10.223 vs 10.209 N), Fx_far -1,2 %,
Cz-Pfad identisch (0,001-0,003). FAC_GPU per g19-A/B ENTLASTET (gpu=host=1,6659 exakt).
Hauptverdacht po_mean-Determinismus (24.08.). Regel: Arm-Vergleiche nur innerhalb EINES
Codestands; Codestand-uebergreifend Versatz als Fassungs-Etikett fuehren.
BRUECKE OF13: cd_rest+cd_reib = 0,529 gegen OF13-Gesamt 0,599 (hinkt: OF13 enthaelt Raeder).
RAND-SLICES MIT RUECKKOPPLUNG (Nachholung Heiko-Morgenauftrag, export/vergleich_of13_2026-08-26/
diff_wf_*.png): x- Mittel +0,34/RMS 1,08 | y- +0,54/2,54 | z+ +0,67/0,72 m/s. Gegen den
ungekoppelten f4_std_diff2 (x- +0,28/1,19 | y- -0,10/2,06 | z+ +0,12/0,42): Raender aehnlich
ruhig; der gekoppelte Arm traegt durchweg ein leichtes +0,3..0,7-m/s-Plus (Band-Injektion),
Radnachlauf am y--Rand in beiden Armen dominant -- Domaenenbreiten-Punkt bestaetigt.

## TIEFER PERFORMANCE-CODE-AUDIT 26.08 (Heiko-Auftrag, waehrend f4_wandfrei_v2 laeuft)

Drei Achsen + Beleg-Sweep, alles offline (ocloc) bzw. aus Bestandsmessungen; Artefakte in
scratchpad/ax3(g). Rahmen: Produktionskernel 2169 MLUPs/267 GB/s auf dem Pfad mit 96 %
Wanduhr -- dieselbe B70 lieferte am Kugelfall 4648 MLUPs/572 GB/s (LEISTUNG.md, 08.08.).
FAKTOR 2,14 LUFT. Konsolidierte Baurunden-Liste (nach Lauf, uebliche Kette, je EIN Arm):

RANG 1 (HOCH, bitidentisch, COMPILER-BELEGT): store_f-Adress-Rematerialisierung.
  Der GESAMTE stream_collide-Spill (iGPU 672 B / B70 1408 B im Testgitter) ist Adress-CSE:
  IGC haelt die 64-Bit-fi-Adressen aus load_f ueber den ~2170-Instruktionen-Facettenblock
  am Leben; ALLE 508,7 M Zellen zahlen ~145 B/Zelle Scratch-Roundtrip, auch die 99,3 %
  ohne Facette (ohne FACETTEN: Spill exakt 0). 7-Zeilen-Remat in store_f -> Spill 0/0,
  laufzeit-bitidentisch (identische Adressen); produktionsreif via garantiert-0-Kernelarg.
  UMGESETZT 26.08. abends (ABI-neutral als t>>62-Variante an der AUFRUFSTELLE, nicht im
  store_f-Body -- Planungsagent-Variantenvergleich; kein neues Kernelargument noetig):
  nn = n+(uxx)(t>>62), j2 = neighbors(nn), store_f(nn,...,j2,...). ABNAHME KOMPLETT:
  offline Spill 448/832 -> 0/0 beide Geraete/Arme (Gate jetzt spill-hart); CPU-Hash-Paar
  Alt(HEAD)/Neu bitgleich (1396757895076695); iGPU kipp0-Anker exakt in an- UND aus-Arm,
  219->249 MLUPs (+14 %) / 253->258 (+2 %); B70 8mm 1478->1523 MLUPs (+3,0 %) mit 15/16
  CSVs byte-identisch (16. = nur juengere Header-Kommentarzeile). Pruefagent: GO, 0 HOCH/
  0 MITTEL, 3 NIEDRIG (Doku + t<2^62-Hostwaechter) mit dem Commit erledigt. EINORDNUNG:
  der Faktor-2,14-Abstand zur 572-GB/s-Kugelreferenz war NICHT der Spill allein -- die
  Kugel faehrt einen leichteren Kernel; Restluecke = Arbeit pro Zelle (naechste Hebel:
  F-Null-Read, FP16S). Auto-Large-GRF (Rang 4) ist mit Spill 0 voraussichtlich obsolet.
RANG 2 (HOCH, physikaendernd, NUR mit Heiko-Ansage): FP16S statt FP16C.
  FP16C-Konverter = 40,7 % des Instruktionsstroms des Nicht-Facetten-Pfads (19 h2f + 19 f2h
  a ~23 Instr./Zelle, kein natives cvt fuer 1-4-11); FP16S nutzt native hf-Konversion:
  -20,5 % Instr. im Produktionskernel (COMPILER-BELEGT). A/B 8mm gegen 4mm-Anker + Bit-Folgen.
RANG 3 (MITTEL, bitidentisch): FORCE_FIELD-F-Null-Read-Gate (~3 % GESCHAETZT; Achse 1-Neufund:
  F der Fluidzellen ist im Produktionspunkt konstant 0, Read ohne Wirkpfad).
  UMGESETZT+ABGENOMMEN 26.08. abends als F_NUR_SOLID (Emission, Default AN,
  CFD_F_NUR_SOLID=0 = Upstream-Kontrollarm; PARTICLES-Praeprozessor-Ausschluss; F-Waechter
  in initialize() prueft die Praemisse hart am Host mit Vollzugsmeldung). MESSWERT statt
  Schaetzung: B70 8mm 1523 -> 1534 MLUPs (+0,7 % -- die ~3 % waren zu hoch, Reads waren
  grossteils cache-gedeckt); CPU-Hash-Tripel AN=AUS=Remat-Referenz 1396757895076695;
  iGPU kipp0-Anker exakt; 16/16 CSVs byte-identisch zu g24_remat_b70; Pruefagent GO
  (0 HOCH/0 MITTEL, NIEDRIG 1+2 mit Commit erledigt). Verbleibende Rang-3-Posten:
  boden_eq-3D-Range
  (Full-N-Dispatch fuer 2-Lagen-Band; ~250x weniger Threads, Kernel unveraendert) +
  VTK-Erstdump-Fix (vtk_next=0 -> Dump bei t=1ms, 11,5 GB Anfangszustand) + ABSTAND-
  Praedikat vorberechnen (statisch, wird je Schritt neu gerechnet).
RANG 4 (A/Bs mit offener Richtung): B70 Auto-Large-GRF (-cl-intel-enable-auto-large-GRF-mode:
  Spill 832->0 nur fuer die 4 grossen Kernel, FP-Multiset identisch; Occupancy-Abwaegung nur
  per GPU-A/B) -- NACH Rang 1 neu bewerten (Remat macht ihn evtl. obsolet); stream_collide-
  only WGS 128/256 (bit-sicher, billig, Wirkung klein); UPDATE_FIELDS-Abloesung (~10-15 %,
  TYPE_E-Klobber-Falle, po-Kernel noetig -- grosser Bauschritt).
ENTWARNT/BELEGT SAUBER: SIMD-Wahl des Compilers (simd8/16 einzig spillfrei bei 128 GRF;
  Xe2 kann kein simd8 -- Compilerfehler als Beleg; 532 B/Lane ist der ALLGEMEINE
  stream_collide-Arbeitssatz), fast-relaxed-math (bewusst abwesend, wuerde K2 kontaminieren),
  uniform-work-group-size (byte-identisch = No-Op), >4GB-Flag (an Puffergroessen alternativlos),
  Zero-Copy-Weg, Events-statt-finish (Deckel ~1,5 %), Verzweigungen im Innersten (Divergenz
  auf 0,67 % der Zellen), keine widerlegten Mechanismen aktiv (13/13 sauber AUS), keine
  Kategorie-d-Faelle, Zaehler sauber gegatet.
BELEG-SWEEP-NEBENBEFUNDE: (a) setup.cpp:3257-Kommentar erklaert PROFIL=3 zum "STANDARD ab
  23.08" -- Lauf und gesamte validierte Kette fahren PROFIL=2, zu 3 existiert kein A/B und
  keine Doku-Zeile -> Kommentar mutmasslich falsch, mit Heiko klaeren, dann korrigieren;
  (b) N2F-Verbot vom 21.08. ("Schale UND Volumen AUS") ist durch Fx_far-Metrik-Befund
  (:1482-1484) + Heiko-Freigaben 22.08./26.08. (Slice-Nachholung) SUPERSEDED -- hiermit
  schriftlich abgeloest; (c) wandprofil_nah/fern.csv wird jeden dd-Lauf geschrieben und
  wurde NIE konsumiert (geplant: delta99) -> nutzen oder abschalten; (d) NEAR_VOR-Boxfrage:
  Wandrueckzugs-Sieger b8_sp_* liefen auf 352/LY 2.992, die f4-Linie auf 96/Standard --
  gehoert zur Domaenenbreiten-Frage (y-Rand-Befund) der naechsten 4mm-Layout-Runde.

## Code-Audit-Korrektur-Loop 26.08 nachmittags: GESCHLOSSEN (Startbedingung 4mm erfuellt)

Vier unabhaengige Pruefer ueber git diff bfa3cec..7876490 + Altbestand. Ergebnis: 0 HOCH.
- Slice-Pruefagent (11 Punkte): MITTEL-1 (j0-Formel-Duplikat + PRUEF-Luecke) -> diff_j0-Helfer
  + PRUEF prueft Diff-Ebenen; 2 NIEDRIG gefixt. Abnahme g21.
- Auditor A (Kernel/Funktionen, frisches Auge): MITTEL-1 Zero-Copy-Zweig fehlte in den
  Voll-/Offset-WRITE-Wrappern -> gefixt; NIEDRIG 1-5 (Guard-Kommentar, yc/j0-Einquellen-
  Refactor mit yc_out, NaN-harte PRUEF-Metrik, Bitgleich-Formulierung praezisiert; NIEDRIG-2
  = Notiz toter Wrapper-Fixes). Unroll-Wirkpfad unabhaengig reproduziert (ohne Hint 4256 B).
- Auditor B (Host/Pipeline, frisches Auge): Guard-Falle/Reihenfolgen/Schalter-Matrix/LAUF.txt
  SAUBER (empirisch, Offline-Compile ohne FACETTEN-Defines); B-3 Ansage-Doktrin ->
  Slice-Transportweg meldet sich jetzt in dd UND einzel; B-4 PRUEF-ohne-Kadenz-Warnung;
  B-2 igc_offline-Workdir-Override; B-1 Gate-Drift -> Querverweis in device_defines,
  VOLLER Drift-Anker (Gate difft gegen CFD_DUMP_DEFINES-Dump) = FOLGEPUNKT nach 4mm.
- fac_tau-Race: ENTWARNT mit Beweiskette (s. u.), Haertungs-Waechter b7474db.
- Eq.25-Passage: im Wissensspeicher als superseded abgeloest (Tag eq25superseded).
- kraft_zband.csv: Phantom-Reibungs-Hinweis jetzt im CSV-Header selbst.
- ZURUECKGESTELLT hinter den 4mm-Lauf (begruendet): W5-Totfacetten-Deklaration und
  Pur-Praezisionswaechter -- beide dienen der K2-Messguete, die per Heiko-Reihenfolge
  hinter dem Lauf liegt; UPDATE_FIELDS-Abloesung und FP16S-A/B (grosse gepruefte A/Bs).
Nachabnahmen: g22 (PRUEF 0/0 alle Ebenen inkl. NaN-Metrik, alle CSVs+18 PNGs byte-identisch,
Refactor-Wertgleichheit g21<->g22 byte-bewiesen), g23 (Ansagen feuern, PRUEF 18x exakt 0).
Build-RC 0, scratch_gate gruen (iGPU+B70 private_size 0).

## f4_wandfrei_prod-Absturz AUFGEKLAERT: kein OOM, sondern GPU-Engine-Reset (Forensik 26.08.)

CL_OUT_OF_RESOURCES (-5) an drive_boundary_cubic_lift war die FOLGEMELDUNG eines toten
OpenCL-Kontexts. Kernel-Journal (Boot -6): 20:38:47 Start snap.drawing (Desktop haengt an
der B70 -- card0-DP-5 ist der einzige connected-Anschluss!) -> 20:39:21 GuC-Engine-Reset
engine_class=ccs, Timedout job "in FluidX3D [57532]", Coredump -> 20:39:48 der -5 + exit(1).
Loupe (2x) und Firefox ueberlebte der Lauf, drawing nicht. VRAM war NICHT die Ursache:
Alloc-Bilanz 29 274+64 MB von 32 655 MB, byte-gleich zu f4_std_diff2 bis auf 63,85 MB
N2F-Schale; Host-RAM 24,8/91,5 GB; kein Leistungsabfall ueber 73 min (405 LEISTUNG-Samples
flach); keine Re-Allokation in der Zeitschleife (verifiziert). Transportlast des Laufs
~0,75 TB Copy-Engine (41 Slices a 8,65 GB + ~205 Kraftfenster a ~2 GB) -- exakt die Klasse
der bcs-Teardown-Resets aus Boot -4; beide Quellen sind seit heute per Default entschaerft
(CFD_SLICE_GPU: 14 MB statt 8,65 GB; CFD_FAC_GPU: kein F-Voll-Read).
KONSEQUENZEN STARTAUFSTELLUNG: (1) waehrend des Laufs KEINE GPU-Programme am Desktop
(gemessene Bruchstelle), besser Monitor an die iGPU (card1 hat freie Ausgaenge) -- Heikos
Handgriff/Entscheid; (2) CFD_VTK_DT grob setzen (Zwischen-Dumps; war beim Absturz NICHT
gesetzt, deshalb Totalverlust) + CFD_STOP_DATEI-Betriebsregel; (3) neuer
werkzeuge/gt_reset_waechter.sh: ueberwacht journalctl -k auf Engine reset/Timedout job und
loest die Stopp-Datei aus -- der ccs-Reset lag 27 s VOR dem Prozessende, der Dump waere
rettbar gewesen; (4) scratch_gate vor Start. i915-GEM-Leck-Klasse: durch intel_iommu=igfx_off
erledigt, Messung heute sauber (-32 MB).

## Perf-Baurunde 2, Baustein 2: Slice-Ebenen-Read (CFD_SLICE_GPU, Default AN) -- 2026-08-26

Nach Planungsagenten-Plan (Variante b): extract_plane_macros wiederverwendet, Zwilling
extract_plane_flags neu, LBM::lese_yslice_in_host (Gather + Host-Scatter, Renderer/Sonde/
Diff lesen unveraenderte Indizes), Kopplungspuffer um die y-Ebenen erweitert, Einzelgitter
lazy-alloc; CFD_SLICE_GPU=0 = wortgleicher Altpfad, CFD_SLICE_PRUEF = Beide-Wege-Vergleich.
Je Slice-Ereignis auf der 4-mm-Nahdomaene: ~14 MB statt ~8,65 GB PCIe (Volumina am
VTK-Layout nachgerechnet); auf 8 mm Wanduhr-neutral (Ereignisse dort zu billig).
Vorab als eigener Commit d20bc61: Zero-Copy-finish-Fix der _1d/_2d/_3d-Wrapper (die
Teilbereichs-Wrapper hatten den 25.08.-Fix nicht -- blockierend war dort kein blockierend).
PRUEFAGENT (11 Punkte, u. a. Konsumenten-Sweep des dd-Loops, Indexketten zellgenau,
Ebenengeometrie 4mm/8mm nachgerechnet): kein HOCH; MITTEL-1 (j0-Formel dupliziert +
PRUEF-Luecke) -> gemeinsamer diff_j0-Helfer + PRUEF prueft jetzt auch die Diff-Ebenen;
NIEDRIG-1 (Kommentar TYPE_MS-Quelle) und NIEDRIG-2 (PRUEF-No-Op-Warnung Einzelgitter)
gefixt; NIEDRIG-3 (static-Vector, dokumentiert) / NIEDRIG-4 (uint-Guard, Bestandsmuster)
belassen. ABNAHME g20+g21 (dd 8mm Kurzlaeufe): SLICE-PRUEF max |Delta| u/rho = 0.00000000
und flags-Differenzen 0 auf nah, fern UND fern-diff an jedem Ereignis; schnitt_diff_letzter/
einlass_saeule/forces/cd_facetten.csv und alle 18 PNGs BYTE-IDENTISCH alt gegen neu.

## Kraft-Zerlegung CFD_KRAFT_ZBAND (Heiko 2026-08-21: unterste 4 Zellen ab z0 vs Rest)

Diagnose-Kernel object_force_zband (Anker-Pfad unangetastet, Schalter aus = bitidentisch per
Konstruktion, Hash-Beleg), kraft_zband.csv mit eingebautem Selbsttest Band+Rest=Gesamt (<5e-5,
jetzt MIT Waechter-Warnung), Band-Census (z=0 beweisbar leer), Facetten-Druckpfad ueber
z-gefilterte Zweitliste (Kernel unangetastet, Band+Rest vs Gesamt 1,4e-7), Kugel-Negativkontrolle
exakt 0 N. Pruefagent: kein HOCH; kfb-Schluessel-Stolperdraht (geteilte marker/z_per-Schluessel)
+ 3 NIEDRIG direkt gefixt. Scope: Druckanteil (fac_tau ohne z -- Folgearbeit fac_geo[6]).
Einsatz: naechstes Screening beziffert das Radkontakt-Restartefakt unter ABSTAND=2.

## 4-mm-Neubasis, Lauf 1 (f4_anker_neu, facettenfrei, ABSTAND=2, NEAR_VOR=96, -3-Lage):

Cd 0,946 / Cz -0,553 (SEM +-0,011), Sonde 0,608, Profil Mitte 1,047 (OF13-Soll 1,087 -- fast
Deckung), Wirkpfad 539.300.500/55.773.396. KERNBEFUND: der sauber gemessene Cz ist bei 4 mm
DEUTLICH NEGATIV (8 mm konfigurationsgleich f8_nearvor96_abst2: +0,14; R1-N3-Praezisierung) -- der Unterboden-Sog ist real und brauchte nur Aufloesung; die
8-mm-Sprosse taugt fuer Cz-ABSOLUTWERTE nicht (Ordnung ja). Gegen OF13 (0,599/-1,301): +58 % Cd,
43 % Abtrieb, facettenfrei und erstmals artefaktbereinigt.

## einlass_eq (V1-Port apply_inlet_velocity) + Einlass-Saeule + Index-Vermessung FAC_GPU

Heiko sah Streifen hinter dem Far-Einlass; Untersuchung: lineare Instabilitaet des TYPE_E-EQ-
Einlasses (lambda 4-6 Zellen, 0,5-1,5 % u_inf, sattigt/friert ein, ERBT bis zur Kopplungsebene) --
NICHT der Boden-Staggered-Mode, aber dieselbe Therapieklasse. V1-Vorlage exakt gefunden
(apply_inlet_velocity, x=1..3, EQ mit lokalem rho, war in V1 WIRKSAM). Einbau als einlass_eq:
x=1..N nur FERNFELD (Nah-x- = Kopplungsebene tabu), Wirkpfad Slot 21 (22er-Puffer) mit Null-
Fehler + Nahfeld-Negativkontrolle, MS-Guard-Lehre statt V1s TYPE_BO-Erbfehler. Pruefagent: kein
HOCH, M1/M2 (lbm_f-Statik-Doktrin, FACETTEN.md) + N1/N2 direkt gefixt. Abnahme: Anker-Hash exakt,
Suite 5/5, fernfeld N=2 Wirkpfad 1.701.700, dd-Sparse-Kombiproof gruen. HEIKO-ANSAGE: ungetestet
auf Cd/Cz-Wirkung DIREKT in den Standard (N=2); Begleiter SPONGE_N=64 ebenfalls Standard.
Einlass-Saeule (Heiko-Vorgabe: 20 cm hinter Einlass, y-Mitte, alle z) als Dauersonde in dd+fernfeld.
INDEX-VERMESSUNG FAC_GPU (Heiko-Pflicht) auf B70: Kraftfenster-Anteil 35,8 % (alt) -> 1,3 % (neu),
iGPU-PRUEF Zaehler exakt gleich, Abweichung 8,45e-6. Die von Heiko gespuerten Chunk-Pausen im
f4_neustandard2 waren exakt diese Fenster (34,9 % live gemessen) -- ab jetzt Geschichte.

## 4-mm-Facetten-A/B (f4_neustandard2 vs f4_anker_neu, neue Basis):

Wandmodell: Cd 0,946 -> 0,839 (-0,11), Cz -0,553 -> -0,581 (-0,03, ~2xSEM) -- BEIDE Vergleiche schaetzerverschieden (Anker object_force vs Facetten-DRUCKschaetzer, R1-N2-Etikett; Armdifferenz ist die belastbare Aussage),
Sonde 0,618. Diagnostik sauber: Wirkpfad 1.310.231.000 = Soll EXAKT, tau-Klemme 0, Dm -0,069,
Rueckfallraten im s5-Rahmen (Rang2 24 %, SATGATE 15 %, BB-Rueckfall 2,4 %), y+ Median 39/q25 24,7
(Viertel im Uebergangsbereich -> YWMIN-Feintuning bleibt offen). Gegen OF13 fehlen Cd 0,24 / Cz 0,72;
Kandidaten: P8/P9-Fernfeld, Einlass-Fix (ab jetzt drin), Unterboden-Abloesung, y+-Feintuning.

NEUE STANDARDKONFIGURATION (ab s6e): -3-Lage + NEAR_VOR=96 + BODEN_EQ=2/DOWN=1 nah+fern +
ABSTAND=2 + FERN_EINLASS_EQ=2 + SPONGE_N=64 + KRAFT_ZBAND=4 (Diagnose) + FAC_GPU (Default).

## 8-mm-VALIDIERUNG des neuen Standards KOMPLETT (Heiko-Regel: erst 8mm, dann fragen)

Leiter (je 0,5 s, ZBAND=4, Saeule aktiv): Basis RMS 0,00966/maxdux 0,051, +einlass_eq=2 RMS
0,00861 (-11 %), +Sponge32 RMS 0,00016 (-98 %), +beide RMS 0,00014 (-99 %)/maxdux 0,0018.
BEIDE Fixes kraftneutral (Cd 1,355-1,359, Cz +0,128..+0,143, Band/Rest stabil) = reine
Sauberkeits-Fixes, belegt. SPONGE-SKALIERUNGSLEHRE: N ist Zellenzahl -- 64@16mm entspricht
32@32mm; der dynamische NF_OX-Guard fing die Fehlkonfiguration korrekt (2 Arme rc=1, neu
gefahren). ZBAND-KERNBEFUND an der 8mm-Sprosse: Kontaktband (4 Zellen=32mm) traegt Cd 0,833/
Cz +0,610, Karosserie-Rest Cd 0,522/Cz -0,467 -- das Radkontakt-Band maskiert den echten
Abtrieb; Cz_rest = ehrlichere Karosserie-Metrik; 4mm-Vergleich (Band dann 16mm) wird Artefakt
von Radlast trennen. Einlass-Wirkpfad 8.246.700 Spalten-Resets. PRODUKTION WARTET AUF HEIKO-GO
nach Abschluss der Audit-Schleife.

## Audit-Schleife R1 (2026-08-21 nachmittags): Code- + Doku-Pruefer, alle Befunde direkt gefixt

Code: KEIN HOCH; M1 = Erst-Kombination Facetten-Arm x voller Standard lief auf keiner Sprosse ->
geschlossen mit f8v_standard_fac (8mm, s6h). N2 (Fy im ZBAND-Selbsttest), N3 (relb im Haupt-PRUEF),
N4 (2^32-Listen-Guard) gefixt; N1 (barrier-GLOBAL-Quirk = Upstream-Kopie-Treue) und N5 (FAC_GPU-
Druckwerte ~1e-5 vs alte Host-Logs; CFD_FAC_GPU=0 fuer exakte Reproduktion) als Aktennotiz.
Doku: KEIN HOCH; alle Protokollzahlen EXAKT reproduziert (Log+CSV unabhaengig). M1-M4 (ABSTAND-
Status, Kontaminations-Etiketten, 3 fehlende Tabellenzeilen, Sponge-Skalierung) + N2/N3 gefixt.
N1: f4_anker_v2/f4_neustandard3 (Serie s6e) wurden 11:04 gestartet und auf Heiko-Anweisung nach
~3 min GESTOPPT (8mm-Validierung zuerst) -- export/f4_anker_v2 enthaelt nur einen PARTIELLEN
Abbruch-Export (ABGEBROCHEN-Marker gesetzt), f4_neustandard3 lief nie. N4: Abnahme-Konsolen
kuenftig nach logs/abnahmen_* sichern (heutige aus dem Scratchpad nachkopiert).

## Audit-Schleife R2: R1-Fixes verifiziert, M1-Schliessung bestaetigt -- SCHLEIFE GESCHLOSSEN

Alle drei Code-Fixes korrekt (Fy-Selbsttest mit bewusst strenger Skala, relb nie schlechter als
alt, 2^32-Guard vor dem Cast); alle Doku-Fixes zahlenbelegt gegengeprueft. M1-Schliessung
f8v_standard_fac unabhaengig bestaetigt: Facetten-Wirkpfad 147.166.750 = Soll EXAKT, Slot 21
Fernfeld 8.246.700/Nahfeld 0, ZBAND-Selbsttest 1,0e-5, Kraefte-Phase 1,1-1,3 % (FAC_GPU).
MATRIX-AUSSAGE: der komplette Produktionspunkt (alle 7 Achsen gleichzeitig) ist auf der
8mm-Sprosse gelaufen -- KEINE Pfadkombination laeuft zuerst in Produktion. Verbleibender
NIEDRIG (UNGETESTET-Etikett einlass_eq) gefixt; relb-1e-12-Aktennotiz dokumentiert.
Severity-Verlauf: R1 kein HOCH (2 MITTEL) -> R2 nur 1 NIEDRIG-Etikett. 4-MM-PRODUKTION
WARTET AUF HEIKO-GO.

## YWMIN-Einzelarm 8mm (f8v_ywmin15): DURCHGEFALLEN fuer den Standard

YWMIN 0,15 vs 0,2 am vollen Standard: Cd_druck 1,680->1,880 (+0,20, falsche Richtung),
Cz_druck +0,062->-0,003, aber Zerlegung: Karosserie-Rest nur -0,519->-0,528 (-0,01), der Rest
des Cz-Gewinns liegt im artefaktlastigen Kontaktband (+0,581->+0,525). y+ q25 bei 8mm ohnehin
39,8 (>30) -- der Uebergangsbereich (q25 24,7) ist eine 4mm-Eigenschaft, YWMIN final nur dort
entscheidbar. VERDIKT: nicht in den Standard; zurueckgestellt bis zur naechsten 4mm-Kampagne.
P8/P9-Plan liegt vor (Planungsagent 21.08.): Instrument -> P8 Far-Facetten -> P9a V1-Port
(x+-Ebene) -> P9b u-only-Volumen-Nudging; Schritt 0 (Interface-Druck + Verdraengungs-Census)
in Umsetzung.

## P8/P9 Schritt 0: Interface-Druck-Instrument + Verdraengungs-Census (bitidentisch, immer an)

interface_druck.csv (rho/dp min/mittel/max je getriebener Ebene an der Sample-Kadenz, aus den
ohnehin vorhandenen face[]-Host-Puffern) + Verdraengungs-Census nach Voxelisierung. KERNZAHL
(8mm-Sprosse): der 32-mm-Treppenkoerper im Fernfeld verdraengt 15,51 % MEHR Volumen (4,79 vs
4,15 m3) und hat 5,48 % mehr Stirnflaeche als das feine Fahrzeug -- Offen-Punkt 8 erstmals
quantifiziert (bei Produktions-DX=4/fern 16 mm neu zu erheben, skaliert nicht linear).
Pruefagent: kein HOCH; M1 = die PLAUSIBILITAETS-ANKER des Implementierungsberichts waren falsch
(q_inf = 551,25 Pa bei 30 m/s/1,225 -- nicht 911; rho 1,00048 = +31,4 Pa) -- Code korrekt,
Protokoll hiermit berichtigt; M2 = z=0-Fahrbahnzeile maskierte rho_min der x-/y-Ebenen (gefixt:
erste b-Zeile uebersprungen, z+ ausgenommen); N1-Aktennotiz (fmin/fmax maskieren NaN still),
N2 gefixt (ipcsv im Abbruchpfad). Anker-Hash nach Fixes erneut exakt.

## P8 Far-Facetten (CFD_FERN_FACETTEN=0..4) eingebaut und geprueft

Facettenpfad jetzt auch fuers Fernfeld-Gitter (Statik-Fenster zwischen lbm_f/lbm_c, Muster
FERN_BODEN_EQ; baue_facetten ist gitterunabhaengig, y_w skaliert mit dx_c; CSVs nach fern/;
Wirkpfad-Report + invertierte Negativ-Kontrolle; Fx_far-Phantom-Etiketten). CPU-Abnahme:
Bitidentitaet bei 0 (Anker-Hash exakt), FERN=4-Kurzlauf Wirkpfad 45.066 = Soll exakt, Nahfeld
beweisbar unbeeinflusst (4.120.669 = Soll). 47 % der Fernfeld-Facetten fallen am 32-mm-
Treppenkoerper auf BB zurueck (erwartbar grob). Pruefagent: H1 = Soll-Formel uebersah den
Vorlauf-Schritt des Grobgitters (n_outer+1 Schritte!) -- falscher exit(1) bei rundem n_outer,
gefixt auf floor(n_outer/100)+1; M1/N1/N2 (Ansagen) gefixt, N3 (Doppel-CSV bei DIAG+P8,
deterministisch identisch) toleriert-dokumentiert. Statik-Leck-Frage NEGATIV (Z.2210 nullt
alle 9 vor dem P8-Block). Anker-Hash nach Fixes erneut exakt.

## P8-A/B (f8p_fern4 vs f8p_basis, 8mm): FERN_FACETTEN=4 WIRKUNGSLOS -- ehrlich gemessen

Wirkpfad 1.419.579 = Soll exakt (Mechanik lief), aber: x--Interface-Druck 145,2 vs 145,7 Pa,
Cd_druck 1,684 vs 1,672, Cz_druck +0,046 vs +0,057, Rest -0,534 vs -0,519 -- alles im Rauschen.
ERKLAERUNG: ein Wandmodell macht den Treppenkoerper nicht schlanker; die +15,5 % Fehl-Verdraengung
(und der bewusst kurze Einlauf) dominieren die Aufstauung, nicht BB-vs-iMEM. Dazu 47 % BB-
Rueckfall am 32-mm-Koerper. VERDIKT: P8 bleibt als gemessen-neutraler Messarm (Default 0);
der druckkonsistente Hebel ist P9c (Heiko-Schalen-Blend, in Umsetzung: x- default AUS,
rho-Drift-Metrik in der Abnahme, Masse per lokalem rho konstruktiv erhalten).

## Abloese-Offensive 2026-08-21 nachmittags (Heiko: Wandmodellfrage GESCHLOSSEN)

Agent 1 (Ist-Vermessung): Dach-Abloesung 8mm x/L 0,50 (eingefroren) vs 4mm x/L 0,65 (flatternd,
sigma 0,22 m) -- Aufloesungstrend +0,46 m, Konfigurationstrend NULL (Facetten verschieben 16 mm).
Diffusor: Kick x=3,59 (x/L 0,81, ~9 Grad); davor loest NICHTS ab; am Kick kollabiert die Rampen-GS
in allen Laeufen (8mm Vollstall, 4mm Blase mit Wiederanlegen ab x=3,95); Spaltmittel stroemt vorwaerts.
Agent 3 (Physik-Ranking): 1. van Driest/SGS (nu_t/nu ~290 laminarisiert wandnah; Zhou&Bae:
Abloeseblase 0..0,35L je SGS-Modell; alte Widerlegung galt y+=137, nicht 30) -- HEIKO:
TIEFENRECHERCHE BIS CODE-EBENE BEAUFTRAGT. 2. Dachband-Abdeckungsloecher (YWMIN-Treppenstreifen
= BB-Rauwand am Dach; Facetten-Physikkorrektur ist Heikos Idee und im iMEM implementiert --
die Loecher sind der Rest; richtungsselektiver YWMIN-Arm dokumentiert und GEPARKT).
3. Aufloesung = Rahmen (Instrument: Dachlinien-Abloesesonde + Dachsaeule mit Reynoldsstress,
OF13-Soll anliegend bis Nase+3,64 m). 4. FP32-A/B GEPARKT (Prior niedrig, billig). 5. Anstroem-
turbulenz ZURUECKGESTUFT (OF13 ist RANS -- GS per Closure turbulent; eigene Streaks waren
kraftneutraler unfreiwilliger A/B; Injektion ins tau~0,5-Fernfeld zerfaellt in Akustik).
6. Operatortausch GEPARKT (doppelt gemessen schlechter; tau+ ohne Parameterpunkt).
P8-Verdikt bestaetigt das Muster: Druckphysik-Beeinflussung am Grobkoerper drehte nichts am Cz.

## OF13-Referenz GEHOBEN (Agent 2, verifiziert: Druckkraft aus Rohfeldern exakt = forces.dat)

Abloesung: OF13 loest praktisch NICHT ab -- Dach voll angelegt, am Heckscheibenknick x=3,585 m
(Nase-Konvention) nur 10-cm-Blase mit Wiederanlegen; Diffusor (Kick x=3,49, 8,2 Grad) im
Zentralkanal bis Fahrzeugende angelegt. Cz-Quellen (Druck): Splitter -0,31, ebener Boden -0,71,
Diffusor -0,33, HECKFLUEGEL -0,47, Radhaeuser -0,13, Dach +0,57 Lift. 2/3 Unterboden, 1/3 Fluegel.
Setup: kOmegaSST-RANS, Wall Functions y+~28, 7 Prismenschichten (erste 0,625 mm), Einlass I=5 %/
L=0,1 m -> nut/nu~6600 (GS per Closure abloeseresistent). PARITAET: bewegter Boden, STEHENDE Raeder
(wie wir). Referenz 0,599/-1,301 = Fenstermittel 900-1200 reproduziert. Vergleichsdaten:
postProcessing/sampleY0/1200.

## Diffusor-Mechanik (Agent 4): DER SPALT IST TOT, nicht der Diffusor krank

Spaltmittel Plateau = exakt 0,50 u_inf = COUETTE (Strasse 1,0 / Karosserie 0) -- kein Durchsatz-
ueberschuss. Die KAROSSERIE-UNTERSEITEN-GS ist ab x=1,6 m totes Wasser, fuellt am Kick (x=3,58,
8,5 Grad, A2/A1=1,45) die obere Spalthaelfte -> Rampe reisst sofort ab. Entlastet: DOWN-Band
(reisst nicht ab), ABSTAND (0 Zellen unterm Diffusor, wirkt rein buchhalterisch -- f8-A/B:
Stroemung identisch, dCz +0,73 reine Reifenkraft), u_road-Drossel (falsche Richtung).
Bernoulli-Budget funktionierender Diffusor+Induktion: -0,7..-1,0 Cz = exakt die Luecke.
Messarme dokumentiert+GEPARKT: DOWN-Fenster ab Kick (Diagnose), ABSTAND=3-Bound.
HEIKO-PRIORISIERUNG 2026-08-21: Grenzschicht-Offensive HINTER P9 angestellt; van-Driest-
Tiefenrecherche (laeuft) wird dokumentiert und wartet.

## van-Driest-Tiefenrecherche (Heiko-Auftrag) ABGESCHLOSSEN -- baufertig GEPARKT hinter P9

Einbau chirurgisch: smag_cc (kernel.cpp:2294, 0.76421222 = 18*sqrt(2)*(C*Delta)^2) multiplikativ
mit D=(1-exp(-y+/A+))^2; y+ = Y/up faellt im iMEM-Pfad als EINE Division ab (utau existiert dort,
Z.1782). Schalter CFD_SGS_VANDRIEST=A+ (0=bitidentisch, nichts emittiert), CFD_SGS_VD_MIN als
D-Floor (Abwaertsspiralen-Sicherung; stetig+selbstlimitierend = strukturell anders als der
SGS_WANDFREI-Stufenkollaps), optional CFD_SGS_WANDDAEMPF fuer Facetten-LOECHER (YWMIN-Streifen).
Slot 23 (Puffer 24), Emissions-Gating, 6 Setzstellen. WALE geparkt (braucht Rotationsanteil =
Nachbar-FD-Schreibwettlauf unter UPDATE_FIELDS). Literatur: ProLB-Klasse faehrt exakt diese
Kombination (Musker/WF + van-Driest-Smagorinsky). EHRLICHE PROGNOSE: Lage-1-Effekt (Lage 2 y+~100,
D>=0,94); 8mm A+=26 nur -12..-17 % nu_t -> A+-Leiter {26,52,104} Pflicht; Kanal-Diskriminator
braucht N=108 (N=38 ist inert, D>=0,94). VORSTUFE: Dachlinien-Abloesesonde (~80 Z., Muster
unterboden_sonde, OF13-Soll Abloesung erst x/L 0,82) -- laeuft in jedem Arm mit.
Baubeginn NACH P9-Abschluss (Heiko-Priorisierung).

## P9c Schalen-Blend (Heiko-Idee) IMPLEMENTIERT, CPU-Abnahme gruen

CFD_N2F_SCHALE=alpha (0=bitidentisch, Anker-Hash exakt): Kasten veh_c+3 Grobzellen, 45.401
Schalenzellen (x- AUS per Heiko-Default, x+ abschaltbar), Blockmittel-Restriktion, Blend mit
lokalem rho + XL-B8-Negation (TRAGEND, funktional bewiesen: Schalen-ux 0,9987 u_inf), Slot 22
fern=45.401/nah=0, Waechter schale_waechter.csv (Plateau 0,06 u_inf, Kipp-Kriterium im Transienten
geeicht: Streak zaehlt ab 0,2 u_inf), Kopplungs-Verify bitgenau, rho-Metrik: x- saettigt bei
+1,2e-3 (Blockage-Feedback, KEIN Leck -- Saekular-Trend statt Punktdifferenz ist die richtige
Metrik). WICHTIG/EINGESTANDEN: die P9c-Quelltexte wurden von meinen Doku-Commits 42d86bb/eb7157a
per git add -A MITGERISSEN (falsche Botschaften) -- Historie bleibt, dieser Eintrag ist die
Klarstellung. LEHRE: kein add -A waehrend Implementierungsagenten im Baum arbeiten.
AUSSTEHEND: Pruefagent, GPU-Leiter, 8mm-alpha-A/B gegen f8p_basis.

## P9c-A/B (f8p_schale05, alpha=0,5): ERSTER ARM, DER DIE KAROSSERIE BEWEGT

Cz_druck +0,057 -> -0,030 (dCz -0,087); Zerlegung: KAROSSERIE-Rest -0,519 -> -0,610 (dCz_rest
-0,091!), Kontaktband unveraendert (0,580), Cd unveraendert (1,673), Waechter faellt (0,047),
Sonde stabil, x--Interface +12 Pa (Blockage-Feedback). Heikos Schalen-Blend wirkt genau am
Ziel. HEIKO-SLICE-BEFUNDE: (1) uniforme alpha=0,5-Schale zeichnet sichtbares Kopplungsartefakt
in Far -> Gradient-Variante (innen alpha=1, zum Rand auf 0 geblendet) = Heikos urspruengliche
Idee, Planungsagent laeuft; (2) NEAR-Inlet weiterhin Schlieren trotz sauberem Far-Inlet ->
Quellendiagnose laeuft (Hypothesen: Streifen-Nachwachsen bis zur Entnahmeebene / Lift-Artefakt).

## Gradient-Blend-Schale + Ebenen-Glaettung: implementiert, abgenommen, geprueft

GRADIENT (Heikos Rampen-Idee + f_neq-Erhalt gegen den vermessenen Gradienten-Kollaps):
CFD_N2F_SCHALE_LAGEN (Default 4, w linear 1,0->0,25, 88.014 Zellen; 0 = Kontrollarm exakt
Altverhalten), _FNEQ (f_neu = feq_blend + ftrue - feq_loc; Paarung per IDENT-Modus BITEXAKT
bewiesen -- 88.014 store-Schreibungen/Schritt ohne Feldwirkung), _XPLUS_SKAL, _IDENT.
Waechter/Negation/Kipp auf Aussenlage, Fixpunkt-Schwellenformel. Alle 5 CPU-Abnahmen gruen.
GLAETTUNG (Schlieren-Verdikt A): CFD_KOPPLUNG_GLATT = 1-2-1-Binomialfilter auf den getriebenen
Ebenen VOR Bodenband/Drive; Instrument liest ungefiltert; kraftneutral; bitidentisch bei aus.
NEBENBEFUND (ehrlich): forces.csv ist PRINZIPIELL nie bitreproduzierbar (atomic_add_f;
Basis-Basis-Doppellauf streut 2,0e-4 > jede Patch-Differenz) -- Bit-Doktrin projektweit auf
"deterministische CSVs bitgleich + Kraefte im Rauschband" umgestellt; abn_p8s0_dd als Referenz
ausgemustert. Nahfeld-Feld-CSVs lauf-zu-lauf nicht bitreproduzierbar (vorbestehend, OFFEN).
Pruefagent Gesamt-Diff: KEIN HOCH; M1 (inneres z+-Lagen-Loch vs boden_eq-Band) + N1/N4 gefixt,
M2 = FNEQ-Laufzeit-Paritaetsbeweis (a=0-Test) OFFEN -> ERSTE AUFGABE MORGEN; heutiger 8mm-Lauf
faehrt FNEQ als deklarierten Messarm unter Waechter+Kipp. Anker-Hash nach allen Fixes exakt.

## Diagnose-Steckbriefe (nachprotokolliert): Schalen-Artefakt M1-M6 + Schlieren-Verdikt A

Artefakt alpha=0,5: Gradienten-Kollaps Faktor 6 (feq vernichtet Scherspannung), Dach-Peak
1,14->1,04, Fernwake -0,18, x+-Barriere (x- +12 Pa, y-Asym x4); Metriken M1-M6 als A/B-Soll.
Schlieren: Far-Streifen wachsen in den letzten ~8 Sponge-Zellen nach, Faktor 24 an der
Entnahmeebene, Vererbung r=0,82, Feingitter verstaerkt auf lambda=ratio; Fix = Ebenen-Filter
(zweites einlass_eq-Band und Sponge-Verlaengerung begruendet verworfen).

## IRON RULE 5 (Heiko, 2026-08-21): NIEMALS an Bildern messen -- immer an Echtdaten

Slices sind quantisiert/geklemmt/y-einschichtig und dienen NUR dem Sichten. Jede quantitative
Aussage kommt aus Feld-Echtdaten (Sonden/CSVs); fehlt das Instrument, wird es ZUERST gebaut
(Iron Rule 3). Rueckwirkend betroffen: die PNG-basierten Zahlen der Abloese-/Artefakt-/
Schlieren-Analysen von heute nachmittag gelten als HYPOTHESEN-Generatoren, nicht als Messwerte
-- Nachmessung an Echtdaten noetig, wo Entscheidungen daran haengen. FEHLENDE INSTRUMENTE
(Bauliste): Nah-x--Saeule (x=2/10 Feinzellen, wie einlass_saeule), Dachlinien-Abloesesonde,
Artefakt-Metriken M1-M6 als Feldsonden (z+-Linien-Profil ueber dem Kasten, Fernwake-Sonde).

## Volumen-Blend (Heiko-Bild) + Nah-Saeule + Zeitinterpolation: gebaut, geprueft, vermessen

VOLUMEN-Blend CFD_N2F_VOLUMEN=1: Rot-Kern (Fahrzeug+100mm) w=1, lineare Rampe zur Gruen-Box
(Fussabdruck-100mm), Schutzzone bis zu den Kopplungsebenen leer; 793k Zellen (358k Kern + 435k
Rampe) bei 8mm, 18 MB/Fenster; alle CPU-Abnahmen gruen; FERN_FACETTEN hart gesperrt.
NAH-SAEULE einlass_saeule_nah (Iron-Rule-5-Echtdaten): x_f=2/10, y-Mitte, am Slice-Hook, je
Sample geflusht. ZEITINTERPOLATION CFD_KOPPLUNG_ZEITINTERP (Lagrava-Zeitanteil, Host-Mix,
w=1-Bit-Anker): EHRLICH WIDERLEGT -- GPU-A/B quasi-stationaer +-0,2 % Nah-Saeulen-RMS, +4 %
Kosten -> GEPARKT (Default aus) wie P8. Das Zeit-Halteglied ist NICHT die Schlieren-Quelle;
naechste Kandidaten via Sonde: x2->x10-Wachstum, GLATT-A/B. NEBENBEFUND BEWIESEN: po_mean
(atomic_add_f, kernel.cpp:2784) ist FELDWIRKSAM nichtdeterministisch -- dd lauf-zu-lauf nie
bitgleich (A/A-Beweis); Fix-Kandidat deterministische Reduktion (kraft_facetten-Muster).
Pruefagent Gesamt-Diff: KEIN HOCH; M1 (Seiten ohne w<0,1-Aussenband jetzt angesagt/gewarnt),
M2 (MITTEL=0 im Volumen-Modus gesperrt -- Deckungspunkt-im-Solid-Vergiftung), M3 (ZEITINTERP-
Ansagen ueberall), N1 (sonde_csv-Abbruchpfade) gefixt; N2 (0,1-Stufen-Rundungskosmetik)
dokumentiert. Anker-Hash nach allen Fixes exakt. GPU-Leiter-PRAEZISIERUNG (Heiko): Host-only-
AEnderungen + Wirkungs-A/Bs laufen auf der FREIEN GPU; CPU-Sprosse = neue Kernel + geraete-
konsistente Bit-Referenzen (Anker-Hashes sind GERAETESPEZIFISCH).

## HEIKO-VERDIKT Volumen-Blend (f8p_volumen, gestoppt ~17:15): RUECKKOPPLUNG AUF AUS

Heiko-Slice-Sichtung: der Flow UEBER das Auto in der Near-Domain wird durch die Volumen-
Rueckkopplung UNREALISTISCH (das veraenderte Far-Feld fuettert ueber die Kopplungsebenen ein
verfaelschtes Near zurueck). ANWEISUNG: die gesamte N2F-Rueckkopplung (Schale UND Volumen)
bleibt AUS -- sie ist NICHT Teil der Standardkonfiguration (Default 0 war ohnehin aus; jetzt
auch als Nicht-Standard-Messarm etikettiert). Die Infrastruktur (Gewichtsfeld, FNEQ, Waechter,
Sonden) bleibt als vollstaendig geprueftes Werkzeug fuer spaetere, vorsichtigere Arme (z.B.
kleines alpha, nur Wake-Region). STANDARDKONFIGURATION unveraendert: -3-Lage + NEAR_VOR=96 +
BODEN_EQ=2/DOWN=1 nah+fern + ABSTAND=2 + FERN_EINLASS_EQ=2 + SPONGE (32@32mm/64@16mm) +
KRAFT_ZBAND=4 + FAC_GPU + KOPPLUNG_GLATT=1 (geerbtes Streifen-Band). export/f8p_volumen =
ABGEBROCHEN-markiert.

## TAGESABSCHLUSS 2026-08-21 abends: finale 8mm-Referenz f8_standard_final GRUEN

Finale Standardkonfiguration (OHNE N2F, MIT GLATT): Cd_druck 1,672 / Cz_druck +0,054 = exakt
Basis-Niveau (Filter kraftneutral BEWIESEN), Band +0,584/Rest -0,530, x- 145,7 Pa, Fern-Einlass-
Saeule RMS 0,00015 (sauber), Facetten-Wirkpfad 147.166.750 = Soll exakt, Sonde 0,588.
EHRLICHER REST: Nah-Saeule x_f=2 dux-RMS 0,0121 (t>0,4s) -- die NEAR-INLET-STREIFEN SIND NICHT
GELOEST (GLATT trifft nur das geerbte Band; die ratio-Eigenantwort des Kopplungsrandes bleibt;
Zeitinterpolation als Ursache widerlegt).

## TODO NAECHSTE SESSION (Heiko-Auftrag):
1. NEAR-INLET-STREIFEN LOESEN (Echtdaten-Referenz: Nah-Saeule RMS 0,0121; Kandidaten offen --
   Ursache der ratio-Eigenantwort am TYPE_E-Rand tiefer analysieren, ggf. Rand-Regularisierung/
   feq-Behandlung der Kopplungszellen; Zeit-Halteglied und raeumlicher Filter sind abgehakt).
2. RECHERCHE RICHTIGE RUECKKOPPLUNGSART near->far (Volumen-Blend verfaelschte den Near-Flow
   uebers Auto -- Literatur/Alternativen: schwaechere alpha, nur-Wake-Region, Kraft- statt
   EQ-Kopplung, Konsistenz mit der grob->fein-Kette; Infrastruktur ist komplett und geprueft).
3. FNEQ-Laufzeit-Paritaetsbeweis (a=0-Test, Pruefagent-M2 offen).
4. Grenzschicht-Kampagne (GEPARKT hinter 1+2): Dachlinien-Abloesesonde -> van-Driest-Leiter
   {26,52,104} -> richtungsselektives YWMIN; OF13-Ziele: Dach angelegt bis x=3,585, Diffusor
   angelegt, Cz-Budget Boden -1,35/Fluegel -0,47/Dach +0,57.
5. Klein: po_mean-deterministische Reduktion, Reibungs-z-Zerlegung (fac_geo[6]), Perf-Runde-2-
   Rest (Slice-Read, UPDATE_FIELDS-A/B, FP16S).

---

## TAGESABSCHLUSS 2026-08-21 spaet: 4-mm-Produktion mit Diff-Instrument + Kopplungs-Bauplan

**4-mm-PRODUKTION `f4_std_diff2` GRUEN** (Standardkonfiguration, rc=0, 1 h 32 min):
cd_druck **0,8428** / cz_druck **-0,5795** (Band +0,2461 / Rest **-0,8255**) = reproduziert die
dokumentierte 4-mm-Basis. VTK beider Domaenen weltpositioniert geschrieben (11,28 GB, Wirkpfad
2 Dateien). Leistungsindex Median **10.609** s_wall/s_phys.

**NEUE INSTRUMENTE (Host-seitig, keine Kernel-Aenderung):**
- `render_yslice_diff` (CFD_DIFF_SCHNITT, Default an): je Schnitt |u_nah|-|u_fern| an derselben
  Weltposition, Fernfeld TRILINEAR auf den Feinzellmittelpunkt. Noetig, weil die Feinmittelebene
  bei geradem cey GENAU ZWISCHEN zwei Grobzellen faellt (bestaetigt: cNy=240, Fern-Schnitt auf
  Index 120 = Welt-y +16 mm, Nah-Mittelebene y=0). Laeuft auf der CPU aus dem Hostspeicher, KEIN
  zusaetzlicher Device-Read; gemessener Phasenanteil "Schnitte 0,0 %". Dazu
  `schnitt_diff_letzter.csv` mit BEIDEN u-Vektoren zellweise.
- VTK-Feldexport (CFD_VTK_ENDE / _DT / _STRIDE): eigener Writer statt
  Memory_Container::write_vtk -- der rechnet mit dem GLOBALEN `units` (der dd-Fall fuehrt zwei)
  und setzt ORIGIN auf die Boxmitte; Nah und Fern laegen NICHT uebereinander. Abnahme am
  Rauchtest: 0 Restbytes, Weltausdehnungen = Startansagen, |u| 28,2/30,1 m/s bei u_inf 30.
- SAUBERER STOPP (CFD_STOP_DATEI, Default /tmp/cfd_stop): V1 hatte das, V2 nicht -- beim
  Neuaufbau nicht mitportiert. Ohne den Mechanismus ging bei jedem kill ALLES verloren, was
  hinter der Zeitschleife steht (VTK, Endauswertung). Stale-Datei-Falle mit portiert.

**KIPP-WAECHTER WAR AUF DER 8-mm-SPROSSE GEEICHT** (CFD_KIPP_AB, Default 0,05 s): stand fest auf
t > 0,02 s. `f4_std_diff` riss dort mit |Cd| 20,06; `f4_neustandard2` ueberlebte dieselbe Stelle
mit 18,62 -- 7 % Rest, also Glueck, keine Auslegung. Bei t = 21 ms ist die Stroemung 0,63 m weit
gelaufen, das Fahrzeug ist 4,4 m lang. Grundlage des neuen Werts: im Fenster 0,05..0,2 s liegt
max|Cd| bei 13,82 (8 mm) bzw. 14,53 (4 mm) = 27 % Luft zur Schwelle 20.

**DIFF-SCHNITT, ECHTDATEN 4 mm bei t = 495 ms** (661.281 auswertbare Zellen):
RMS **6,07 m/s**, Mittel **+0,90**, Median -0,58, p95 +14,8. x-Baender: Einlaufband vor der Nase
**0,36-0,95** (= 1-3 % von u_inf, die VORWAERTSKOPPLUNG SITZT), ab der Nase 2,3 steigend bis
**8,56 bei x=+4,1 m** (Heckabriss/nahes Totwasser). Der Fehler entsteht AM KOERPER (Grenzschicht-
saum, den das Fernfeld nicht auflaesen kann) und waechst monoton in den Nachlauf.
KORREKTUR EINER FRUEHEREN AUSSAGE: die PNG-Schaetzung derselben Groesse (8 mm) gab Mittel -0,72
und "75 % der Zellen langsamer". Beides ist ein Klemm-Artefakt -- 27,8 % der Nah-Ebene standen im
PNG an der unteren Farbklemme (u <= 15 m/s), genau das Totwasser fehlte. Iron Rule 5 bestaetigt.

**LEISTUNGSPROFIL 4 mm, 384 Berichte -- V1s Aussage gilt in V2 NICHT MEHR:**
Nahfeld (B70, 4 feine Schritte) **97,7 %** | Kraefte 1,1 % | Kopplung grob->fein 0,9 % |
**Fernfeld warten+entnehmen 0,3 %** (~1,3 ms von 424 ms) | Schnitte 0,0 %.
Der grobe Schritt verschwindet vollstaendig hinter dem feinen: **die B70 ist der Flaschenhals,
nicht die iGPU**. V1s README-Satz "iGPU coarse step ~720 ms is the saturated bottleneck" ist fuer
V2 falsch und im oeffentlichen README berichtigt. Ursache der Umkehr NICHT gemessen (beide
Generationen rechnen ~203 M Fernzellen; in V2 ist das Fernfeld bewusst reines Bounce-Back).
Nebenbei quantifiziert: der Schnitt-Hook transferiert bei 4 mm **11,3 GB je Schnitt**; in den
38 von 384 Fenstern mit Schnitt steigt der Aussenschritt 422,6 -> 474,4 ms (+12 %), ueber den
ganzen Lauf ~1 %.

**BAUPLAN KOPPLUNGS-UMBAU -> `BAUPLAN-KOPPLUNG.md`** (vier Planungs- und drei Pruefagenten).
Die drei Befunde, die dort alles andere binden:
1. **Teil A (Box) und Teil C1 (Rueckwaertsband linear) brauchen NULL Codezeilen** -- A sind drei
   Env-Schalter, C1 ist `CFD_N2F_SCHALE=0.5 CFD_N2F_SCHALE_LAGEN=8`. Der ganze C-Zweig ist ein
   Kill/Keep-Entscheid fuer 24 Minuten Rechenzeit, BEVOR irgendetwas gebaut wird.
2. **Der Fruehindikator war der falsche.** Fenstermittel [0,02;0,10 s]: der verworfene
   Volumen-Blend hatte Cd **12,03** gegen Basis 12,16 -- er war im frueher Cd BESSER als die
   Basis. `Fx_far_N` trennt um Faktor 12 (44.553 N gegen 3.760 N), im ersten Sample bei t=2 ms
   bereits 83.837 N gegen -19.924 N. **Ab jetzt ist Fx_far das Abbruchkriterium, nicht Cd.**
3. **Die Fahrbahn-Falle im Band-Listenbauer:** Saatmenge muss `(TYPE_S|TYPE_X)` sein, nicht
   `TYPE_S`. Die Fahrbahn ist bei z=0 flaechendeckend TYPE_S -- mit ihr als Saat waere das
   "Band ums Fahrzeug" ein 8 Zellen dicker Teppich ueber 2,95 Mio Zellen, still.

**y+ GEMESSEN, beide Sprossen** (Facetten-Akkumulator, y_w je Facette):
4 mm q25 24,8 / **Median 39,4** / q75 64,6 -- 8 mm q25 45,3 / **Median 72,2** / q75 120,8.
Faktor 1,83, y+ skaliert sauber mit dx. Median 39 heisst: erste Zelle oben im Buffer-Layer, die
Grenzschicht wird von im Wesentlichen EINER Zelle getragen. Smagorinskys lokales nu_t ~ D^2|S|
dissipiert, es transportiert nicht -- die auflaesungsabhaengige Abloesung ist die Vorhersage
dieses Modells (Modeled-Stress-Depletion), kein Defekt.

## TODO NAECHSTE SESSION (Heiko, Stand 2026-08-21 abends)
1. **Kopplungs-Umbau nach `BAUPLAN-KOPPLUNG.md`**, Phasen 0-2 zuerst (kein Code, 1 h 20).
2. **Kollisionsmodell HRR/RR** (NACH der Kopplung). Astoul et al. 2020 (JCP 418:109645): die
   Erzeugung falscher Wellen am Aufloesungssprung ist INTRINSISCH (Aliasing), unabhaengig vom
   Kopplungsalgorithmus -- der benannte Hebel ist die Kollision, nicht das Interface.
3. **van-Driest-Wanddaempfung** (NACH der Kopplung, baufertig dokumentiert). Adressiert die
   MSD-Ursache statt des Symptoms.
4. **2x B70, Nahfeld-Halbierung mit Halo** (Hardware in Arbeit) -- Vorpruefung siehe unten.
5. Rest unveraendert: FNEQ-Paritaetsbeweis (M2), Near-Inlet-Streifen, po_mean-Determinismus.

## VORPRUEFUNG 2x B70 (Heiko-Plan 2026-08-21 abends)
Plan: zweite B70 auf neuem Mainboard, beide fuer das NAHFELD (linke/rechte Haelfte, per Halo
verbunden), Nahfeld auf 3,5 mm, Fernfeld bleibt auf der iGPU, Blockage unveraendert (Fern-x+
entsprechend einkuerzen).

**KLEMME: 3,5 mm bricht das 4:1.** 16/3,5 ist keine ganze Zahl -- ratio 4 zwingt das Fernfeld auf
**14 mm** = +49 % Fernzellen (203,5 -> 303,8 M), iGPU-Grobschritt 343 -> **511 ms** gegen ein
Nahfenster von ~507 ms (2 Karten, 15 % Halo-Aufschlag) = **101 %**. Die iGPU wird vom versteckten
Partner zum Flaschenhals. Rechnung auf den Ankern dieser Maschine (B70 4648 MLUPs, iGPU 594
MLUPs, 55 B/Zelle, groesste je beobachtete Belegung 29.274 MB).

**Zwei Auswege, beide gerechnet:**
- **Fern-Nachlauf um 2,5 m einkuerzen** (Heikos eigener Vorschlag): 407 ms = 80 %, exakt wie
  heute. Rest 1,0 m Fernfeld hinter dem Nahfeld-Auslass statt 3,5 m. Der Nahfeld-Auslass ist ein
  Zou-He-Druckrand, sein Gegendruck kommt aus genau diesem Stueck -- eng, aber ueber
  `interface_druck.csv` messbar.
- **3,2 mm mit ratio 5**: Fernfeld bleibt unangetastet bei 16 mm, iGPU bei **56 %**, und 3,2 mm
  ist FEINER als die gewuenschten 3,5 (mehr Voxeldetail = das eigentliche Ziel). Preis: ratio 5
  statt 4 (Spektralsprung [pi/5,pi] statt [pi/4,pi]), Box waechst nicht (32,5 statt 32,0 m3).

**Die zweite Karte bezahlt in beiden Faellen die Aufloesung, nicht die Box:** 3,5 mm -> +37 %
Volumen (43,9 m3), 3,2 mm -> +2 % (32,5 m3). Nahzellen 1024 M bzw. 991 M.

**Der y-Split ist die aufwendigste Schnittebene.** Lastbalance perfekt (symmetrisch), aber der
Halo laege genau in der Fahrzeug-Mittelebene -- Dachfirst, Unterbodenmitte, Diffusormitte -- und
das ist die Ebene, in der ALLE Diagnosen messen (Diff-Schnitt, Slices, Sonden). Der Upstream-Halo
traegt fi/rho/u/flags sauber (`communicate_fi`, `kernel_transfer`, `get_D()` sind im Kern
vorhanden; beide Domaenen werden heute mit dem Einzelgeraet-Konstruktor gebaut). NICHT abgedeckt
sind die eigenen Kernel des Forks: `apply_facette` liest ein 3x3-Fenster und 18 Nachbarn,
`baue_facetten` ein 5^3-Fenster, die Kraftreduktionen summieren pro Domaene, `schale_extract`
mittelt ueber ratio^3, der Sparse-Tiles-Pfad haelt eigene Slot-Tabellen.
**Jeder davon braucht ein Halo-Audit -- das ist die Arbeit, nicht der LBM-Kern.**

---

## 2026-08-22 vormittags: Band-Listenbauer gebaut, Paritaetsbeweis gefuehrt, zwei eigene Fehler gefunden

**REIHENFOLGE UMGESTELLT (Heiko-Vorgabe):** `schale_blend` laeuft jetzt VOR `boden_eq`, nicht mehr
danach (lbm.cpp `do_time_step`). Vorher gewann die Rueckkopplung ueber das Bodenband; jetzt bringt
sie die feine Loesung ein und der Boden-Fix legt sich darueber. Der `z_lo`-Ausschluss im
Listenbauer ist damit eine zweite, redundante Absicherung -- bleibt drin, ist als solche vermerkt.

**BAND-LISTENBAUER (`CFD_N2F_BAND=1`, `_N`, `_PROFIL`, `_UNTERBODEN`).** Band vom Fahrzeug nach
aussen statt Schale um dessen Bounding-Box; Chebyshev-Distanztransformation per N Runden a drei
1-D-Dilatationen. Kein neuer Kernel -- `schale_blend` ist "dumm", Geometrie und Gewicht sind
Host-Daten. Census 8 mm, N=4: **145.869 Saatzellen** (Quervergleich Verdraengungs-Census 146.167,
Differenz 298 = die Aufstandsflaechen, denen TYPE_X entzogen wird -- passt exakt), **123.510
Bandzellen** in 4 Lagen, 22.444 im boden_eq-Band ausgelassen, 0 ausserhalb des Fussabdrucks,
**0 fluidleere Bloecke**. Lage 0 (w=1,0, direkt an der Karosserie): 37.048 Zellen, davon
**19.690 Unterboden** = 53 %. Abstand zu den getriebenen Entnahmeebenen x- 6 / y- 7 / y+ 7 /
z+ 18 -- alle >= 4. **Damit ist die Nahfeld-Box fuer diesen Arm NICHT zu eng**, die geplante
Verbreiterung ist fuer C1 nicht noetig (sie bleibt fuer das Vorwaertsband relevant).

**EIGENER FEHLER 1, gefunden vom eigenen Waechter:** die Fahrbahn-Falle war als Test "Saatmenge >
10 % der Fahrbahnflaeche" formuliert -- eine 3D-Zellzahl gegen eine 2D-Flaeche. Das Fernfeld-
Fahrzeug hat bei 8 mm 146.167 Zellen, die Fahrbahnebene nur 384x240 = 92.160; ein Volumen ist
zwangslaeufig groesser als eine Flaeche, der Waechter schlug also am RICHTIGEN Fahrzeug an.
Richtig ist der Test auf der Ebene z=0: die Fahrbahn ist dort flaechendeckend, das Fahrzeug
beruehrt sie nur mit den Aufstandsflaechen. Gemessen jetzt: 0 Saatzellen auf z=0.

**★ BEFUND, der eine Doktrin korrigiert: EIN IDENT-BITBEWEIS UEBER AUSGABEDATEIEN IST IM
DOPPEL-DOMAENEN-FALL NICHT FUEHRBAR.** Gemessen 2026-08-22 an vier Armen:
- A/A ohne N2F: `interface_druck.csv` **bitgleich** -> das Fernfeld ist fuer sich deterministisch.
- Schalen-IDENT (der historisch als "bitexakt bewiesen" vermerkte Arm) gegen aus: **weicht ab**.
- Band-IDENT gegen aus: weicht ab.
- Band-IDENT gegen **sich selbst**: **weicht ab**.
Sobald `CFD_N2F_SCHALE>0` gesetzt ist, wird das Fernfeld nichtdeterministisch -- auch im
IDENT-Modus, der nichts schreibt ausser dem Gelesenen. Die Aktivierung zieht `schale_extract_u`
auf dem Nahfeld ins Kopplungsfenster, und das Nahfeld ist ueber `po_mean` feldwirksam
nichtdeterministisch. Der historische Vermerk kann also NICHT aus einem Lauf-gegen-Lauf-Vergleich
stammen; er ist als Beleg fuer Bitgleichheit der AUSGABEN nicht belastbar.

**PARITAETSBEWEIS geraeteintern gebaut (Slots 23/24, `CFD_N2F_PARITAET=1`)** -- er haengt an
nichts als der Arithmetik der Zelle und ist damit vom Determinismus unabhaengig. Der Arm setzt das
KERNEL-alpha exakt auf 0, laesst den Mechanismus aber voll laufen (Listenbau, Extract, Upload,
Enqueue). Zwei Anlaeufe waren noetig, beide lehrreich:
1. `alpha=0` ueber `CFD_N2F_SCHALE` schaltet den Enqueue ab (`schale_alpha==0` ist das Torgatter) --
   der Beweis lief ins Leere, Wirkpfad NULL. Torwaechter und Kernel-Wert sind jetzt getrennt
   (`s_schale_paritaet`).
2. Die Forderung "muss bitexakt 0 sein" war FALSCH GESTELLT. Bei a = 0 ist `u2 == u_lokal` exakt,
   also `feq == feq_loc` bitgleich -- aber die Schlusszeile lautet `feq[i] += ftrue[i] - feq_loc[i]`,
   und **a + (b - a) ist in Gleitkomma nicht exakt b**. Der FNEQ-Arm KANN kein bitexaktes No-Op
   sein. Und der erste Ersatztest (float32-ULP) mass eine Genauigkeit, die das Feld gar nicht
   traegt: gespeichert wird in FP16C (~11 Bit Mantisse, Aufloesung 4,9e-4).
**ERGEBNIS: alpha = 0, 262.790 Verteilungen weichen in float32 ab, groesste RELATIVE Abweichung
0,000062 -- ein Achtel der FP16C-Aufloesung. Im gespeicherten Feld ist das kein Unterschied.
Pruefagent-M2 ist damit beantwortet.**

**Sauberer Stopp: gebaut, getestet, ein Fehler darin behoben.** Der Mechanismus vom 21.08. war nur
syntaxgeprueft. Erster Rauchtest: Stopp korrekt (t = 0,056 s statt 0,300, VTK mit der WIRKLICH
erreichten Zeit im Namen, Stopp-Datei entfernt) -- aber rc=1, weil drei Wirkpfad-Sollwerte aus
`n_outer` rechnen, also der GEPLANTEN Schrittzahl. Ein geretteter Lauf meldete sich als defekt.
`n_outer_ist` behoben; zweiter Rauchtest rc=0, Facetten-Wirkpfad Ist = Soll = 88.300.050 exakt.

## 2026-08-22 mittags — Heiko-Befund am Fernfeld-Slice: Geschwindigkeitsrand an der Fahrzeugnase

**Heikos Beobachtung:** ungewoehnlich hoher Geschwindigkeitsrand an der Vorderseite, dort wo
die Rampe der Rueckkopplung bei 0 % stehen sollte.

**Erste Klarstellung:** der Lauf `f8_standard_final_diff` hatte **gar keine Rueckkopplung**
(kein `CFD_N2F_*` im Log, nur die Hinkopplung). Der Rand kann nicht vom Blending stammen.

**Gemessen** an `export/f8_standard_final_diff/schnitt_diff_letzter.csv`, Mittelebene y=0:

| Groesse | Nahfeld (8 mm) | Fernfeld (32 mm) |
|---|---|---|
| Spalt Fahrbahn -> Splitter unter der Nase | 80 mm (10 Zellen) | **32 mm (1 Zelle)** |
| \|u\| dort | ~35 m/s | **112,9 m/s = 3,7 u_inf** (u_z = -112,06) |

Ursache: ein 32-mm-Voxel, das den Splitter irgendwo beruehrt, wird vollstaendig solid. Der grobe
Wagen zieht seinen Unterboden damit zwei Zellen tiefer als der feine. Derselbe Volumenstrom muss
durch einen um 60 % engeren Kanal, ueber eine mit 30 m/s bewegte Fahrbahn -- und der Kanal ist
genau EINE Zelle hoch, also unterhalb jeder Aufloesungsgrenze.

**Kein Renderer-Artefakt.** Die ueber z=1..7 identischen Werte entstehen, weil im 2x2x2-Stencil
nur eine einzige Zelle kein Solid ist; `render_yslice_diff` verteilt einen echten groben Wert,
es erfindet keinen. Beleg zusaetzlich: die Aufweitung faellt mit der Sperre zusammen
(`fern_ungueltig=1` ab z=8, feines Solid erst ab z=10).

**Umfang** (Mittelebene, 163.619 feine Fluidzellen): 892 Zellen = 0,55 % sind im Fernfeld
zugemauert; 33 Zellen = 0,020 % ueberschreiten 1,5 u_inf.

**Folge fuer den Bauplan -- BISHER NICHT ERFASST:** der Befund stuetzt die Rueckkopplung (es ist
genau die Voxelisierungsdifferenz, die ihr zugewiesen wurde), zeigt aber eine Grenze:
**wo das Fernfeld zugemauert ist, gibt es keine Zelle, die das Band korrigieren koennte.**
Die 112 m/s stehen in der NACHBARzelle, die das Band erreicht -- die wird besser; die Sperre
selbst bleibt. Der Saat-Test `flags != (TYPE_S|TYPE_X)` erfasst diese Nachbarzellen korrekt.
Zu pruefen bleibt, ob eine u-Aufpraegung bei konstantem rho in einem geometrisch versperrten
Ein-Zellen-Kanal ueberhaupt tragen kann, oder ob sie gegen die Geometrie anarbeitet.

**Reihenfolge Moving-Floor (dieselbe Sitzung):** Heikos zweiter Befund -- der Bodenfix muesse
NACH der Rueckkopplung wieder aufgetragen werden -- ist richtig und seit `919c122` umgesetzt
(`schale_blend` vor `boden_eq`, lbm.cpp:1590-1592). `f8_standard_final` ist von 2026-08-21 und
zeigt noch den Altstand, in dem der Blend ueber den Bodenfix gewann.

## 2026-08-22 mittags — Fx_far ist im BAND-Arm konstruktionsbedingt UNGUELTIG (Heiko-Einwand)

**Mein Fehler:** Ich hatte den Band-Arm auf Grund von `Fx_far_N` = 50.186 N (Basis 3.166 N)
verworfen, mit Verweis auf den 2026-08-21 verworfenen Volumen-Blend (44.553 N). Heiko hat
verlangt, erst die Herkunft der Kraft zu klaeren. Zu Recht.

**Mechanismus:** `update_force_field` (kernel.cpp) laeuft auf den SOLID-Zellen des
Fernfahrzeugs, macht dort `load_f` und rechnet die Momentum-Exchange-Kraft. Unter
Esoteric-Pull liegen die gelesenen DDFs im Speicher der ANGRENZENDEN FLUIDZELLEN. Lage 0 des
Bandes ist per Definition genau diese karosserienahe Schicht (Saat = Fahrzeugvoxel,
Chebyshev-Abstand 1), und `schale_blend` macht dort `store_f`.

**Also misst `Fx_far` die Verteilungen, die der Blend selbst geschrieben hat -- keine Kraft.**
Der Baum wusste das an einer Stelle bereits: setup.cpp:2851 sperrt `CFD_FERN_FACETTEN` mit
exakt dieser Begruendung. Die Facetten-Kraft war abgesichert, `object_force` nicht.

**FOLGE FUER DEN VOLUMEN-BLEND (2026-08-21 verworfen):** seine Rot-Box (Fahrzeug-BBox + 100 mm)
enthielt die karosserienahe Schicht ebenfalls. Sein `Fx_far` war damit nach demselben
Mechanismus ungueltig. **Die Verwerfung steht auf einer kaputten Metrik und ist neu zu
pruefen.** Das ist kein Freispruch fuer den Volumen-Blend -- nur die Feststellung, dass das
Urteil nicht auf dem beruht, worauf es zu beruhen schien.

### Was stattdessen gemessen wurde (Groessen, die der Blend nicht anfasst)

`schnitt_diff_letzter.csv`, Mittelebene y=0, t = 0,492 s, 162.727 feine Fluidzellen:

| Zone | x [m] | RMS Kontrolle | RMS Band | Aenderung |
|---|---|---|---|---|
| stromauf der Nase | -9,00..0,01 | 2,613 | 2,296 | -12,1 % |
| Nase/Splitter | 0,01..0,35 | 6,651 | 4,308 | **-35,2 %** |
| Mitte/Unterboden | 0,35..1,90 | 7,289 | 2,960 | **-59,4 %** |
| Heck | 1,90..2,60 | 6,853 | 2,881 | -58,0 % |
| Nachlauf nah | 2,60..4,00 | 7,780 | 2,876 | **-63,0 %** |
| Nachlauf fern | 4,00..9,00 | 7,309 | 4,876 | -33,3 % |
| **gesamt** | | **7,146** | **3,945** | **-44,8 %** |

max |d| 83,88 -> 67,28 m/s. Zugemauerte Zellen 892 in BEIDEN Laeufen (unveraendert -- das
Band kann dort nicht schreiben, s. Nasen-Befund derselben Sitzung).

Cd (Fenster 0,2-0,5 s, aus dem NAHFELD, das der Band-Blend nicht beruehrt -- `s_schale_alpha`
ist fuer `lbm_f` explizit 0): Basis 8,950 +- 0,028, Band 7,872 +- 0,036. Cz 0,536 -> 0,476
(-0,8 sigma, nicht signifikant). Ob 7,872 NAEHER AN DER WAHRHEIT liegt, sagt keine dieser
Zahlen -- dafuer fehlt eine Referenz (OpenFOAM 13 oder eine feinere Sprosse).

### Default-AUS bitidentisch: BELEGT
`Fx_far_N` der Kontrolle ist bis t = 0,3 s ziffernidentisch zur Basis (-19924, -19306,
-10405, 5550, 996, 4384), Divergenz erst ab 0,4 s. Das Fernfeld ist bitgleich; die
Cd-Abweichung +0,069 stammt aus der dokumentierten po_mean-Nichtdeterminie im Nahfeld.

### Offen
1. **`Fx_far` als Kriterium reparieren.** Vorschlag: Schalter fuer die Startlage des Bandes
   (Blend erst ab Chebyshev-Abstand 2), dann liest `update_force_field` wieder echte
   Rueckprall-Verteilungen. Ein Lauf, eine Variable.
2. **Volumen-Blend neu bewerten** (s. o.).
3. **`b8_band_wake` (Dichte-Abbruch)**: rho ausserhalb 1 +- 0,1 im Wake-Kasten, sofort bei
   Scharfschaltung t = 0,2 s. Kein Anlauftransient. Echter Befund.
4. **`b8_paritaet`**: an meinem eigenen Konstruktionsfehler gescheitert -- der Kipp-Waechter
   misst ||u_nah - u_fern||, bei alpha = 0 bleibt die Differenz naturgemaess gross. Der
   Beweisarm kann seinen eigenen Waechter nicht ueberleben. Unter `CFD_N2F_PARITAET` muss der
   Waechter melden statt abbrechen.

## 2026-08-22 nachmittags — der "3,1-m/s-Sockel" war ein Anlauftransient (Heiko-Frage, zwei Agenten)

**Mein Fehler.** Ich hatte aus `export/f4_std_diff/schnitt_diff_letzter.csv` abgelesen, der
Kopplungsfehler laufe auf der 4-mm-Sprosse ab 192 mm in einen Boden von 3,1 m/s, und daraus
eine Skalierungsregel abgeleitet ("bei 4 mm reichen 12 Lagen"). Beides ist falsch.

**Ursache:** `f4_std_diff` steht bei **t = 11 ms**, und `forces.csv` endet bei t = 0,020 s --
der Lauf brach weit vor seinem eigenen `CFD_T_WARMUP=0.2` ab. Der "Sockel" war ein konstanter
u_x-Versatz von +3,5 m/s: das Fernfeld war schlicht noch nicht auf Geschwindigkeit.
Gegenprobe an zwei unabhaengigen 4-mm-Laeufen jenseits 192 mm: `f4_std_diff` traegt 38,7 %
Mittelwertanteil, `f4_std_diff2` nur 0,3 %, Korrelation der beiden Differenzfelder r = +0,03.

**Mit dem vollstaendigen Lauf** `f4_std_diff2` (t = 495 ms, Umgebung bis auf SPONGE_N und den
VTK-Schalter identisch zu b8_kontrolle) faellt die Kurve durchgehend, ohne Boden:

| Abstand | 8 mm | 4 mm | Grobzellen 8 / 4 |
|---|---|---|---|
| 64 mm | 20,42 | 8,80 | 2 / 4 |
| 128 mm | 12,36 | 4,29 | 4 / 8 |
| 256 mm | 4,05 | 2,10 | 8 / 16 |
| 512 mm | 1,56 | 0,94 | 16 / 32 |

**Folge fuer die Bandbreite:** bei GLEICHER LAGENZAHL ist der Restfehler auf beiden Sprossen
etwa gleich (N=8: 4,05 gegen 4,29). Die Lagenzahl in GROBZELLEN ist damit die richtige
Kopplungsgroesse, nicht die Millimeter -- Heikos urspruengliche Formulierung "16 Fernzellen"
war richtig und uebertraegt sich sauber auf 4 mm.

**Was unabhaengig davon stehen bleibt (Code-Befund, verifiziert):** `(C*Delta)^2` steckt als
Konstante 0.76421222 in kernel.cpp:2294 mit **Delta = 1 Gitterzelle**, in beiden Domaenen
identisch. Physikalisch ist das nu_t = (C*dx)^2*|S|; das Fernfeld traegt bei gleicher
Scherung die 16-fache Wirbelviskositaet. Bei tau ~ 0,5 (setup.cpp:2131/2148-2156) ist das
praktisch die gesamte Viskositaet -- die beiden Domaenen loesen unterschiedliche effektive
Gleichungen. Korrektes Smagorinsky-Verhalten, aber eine dauerhafte Asymmetrie der Kopplung.
Frueherer Projektbefund mit derselben Diagnose:
FluidX3D/findings/2026-06-23_resolution-dependence_sgs-grid-dependence.md

**Lehre fuer mich:** vor jedem Vergleich zweier Laeufe den ZEITSTEMPEL des Schnitts und das
Ende der forces.csv pruefen. Ein Diff-Slice traegt seine Zeit in der ersten Spalte; ich habe
sie nicht gelesen.

## 2026-08-22 abends — Acht-Prüfer-Audit-Loop (Heiko-Auftrag: Kopplung komplett, Performance, Ablösung, Facetten)

Acht unabhängige Prüfer (Diff/Profil-2, Performance, Ablösung, Facetten/ELIBB, Kernel-OpenCL,
Host-Pipeline/toter-Code, Numerik/Physik, Instrumente/Wächter). Behoben in dieser Runde:

- **B-P2/B2 (2 Prüfer unabhängig):** w=0-Zellen (Profil-2-Nulllage) wurden gelistet + transferiert;
  im EQ-Arm ist a=0 eine fneq-Löschschale. Band-Bauer filtert jetzt w<=0 (wie der Volumen-Bauer),
  Census sagt die Zahl an. NICHT bitidentisch zu f4_kopplung_plateau — gewollt.
- **B-P1/B-P4:** Wächterlage bei Profil 2 auf N-2 (äußerste Lage MIT Gewicht); Negations-Schwelle
  nutzt band_w(Wächterlage) statt band_w(N)=0.
- **B-P3:** Profil 2 mit N=1 → harter Fehler; PLATEAU bei PROFIL!=2 → Warnung; PLATEAU in No-Op-Liste.
- **B1:** z_lo-_DOWN-Lücke auch in Volumen- und Schale-Bauer geschlossen (war nur im Band-Arm).
- **Instrumente-2:** Fx_far/Fx_grob-Kontaminationswarnung jetzt auch im normalen Band-Arm (war nur
  NURWAKE); kraft_zband in die Phantom-Warnung aufgenommen.
- **Instrumente-4:** band_bilanz-Kopf berichtigt (NACH dem Flag-Filter; mdot = Gittereinheiten).
- **B4:** bilcsv/swcsv-Schluss vor den drei neuen Abbruchpfaden (Symmetrie-Doktrin).
- **B-P5:** n_kern-Etikett ehrlich (Kastenkern PLUS Körperband-Lage-0 x>=wx0).
- **Kernel-1:** alloc_schale-Guard von 2^32 auf 2^32/3 (3u*gid-Produkt).
- **B6:** tau_f/tau_c (write-only) entfernt; B7/I7: Gleichzeitigkeits-Kommentare berichtigt
  (1 Grobschritt Versatz, 4e-5 s).

**Widerlegt:** Instrumente-Befund 1 ("Band berührt Kopplungsebenen bei 4 mm") — der Prüfer las
NEAR_VOR_MM=96 als Nasenabstand; real 326 mm = 20 Grobzellen, Lauf meldet selbst x- 4. Kernel-3
(PARITAET-No-Op) war bereits abgedeckt (Zeile 2525).

**Nicht behoben, als Entscheidung notiert:**
1. **FNEQ=1 als Produktionsstandard** (Numerik-Prüfer: EQ-Arm = a-unabhängige Äquilibrierung,
   effektiv ~75x Smagorinsky-Dissipation im Band; ABER: diese Dissipation dämpft derzeit die
   Rückkopplungsschleife — Umstellung nur mit Fx_far-Gate).
2. **Wand-Rückzug des Bandes** (Ablösungs-Prüfer: die Wand-Schreibung verschlechtert die
   Fernfeld-Ablösung aktiv, 11,2 % gegen 43,7 % ungekoppelt gegen 35,6 % nah — das 4x4x4-
   Blockmittel kann Ablöseschichten mit Median 32 mm nicht transportieren; Hebel: Lage 1-3
   Gewicht 0 bzw. NURWAKE-A/B, plus CFD_FERN_FACETTEN — Kombination derzeit gesperrt).
3. **Async-Überlappung des N2F-Rückwegs** (Perf: bis 3,6 % bei 4 mm; Netto-Kopplungskosten
   heute nur +0,3-0,7 %).
4. **Massenquelle:** S ~ alpha * Integral(Lage-0-Fläche) * Delta_u — N-unabhängig (erklärt beide
   Messungen); Profil 2 senkt sie NICHT (Plateau lässt den dominanten Innensprung identisch).
5. **Facetten/ELIBB:** Wirkpfad Ist=Soll exakt (877.854.770 = 2.620.462 x 335); ~35 % der
   Ereignisse fallen auf BB zurück (SATGATE-Semantik), y+ Median 42. ELIBB-Verdikt: GEHT —
   der V2-Einhängepunkt (registerlokal in stream_collide) umgeht die V1-EP-Pull-Falle
   konstruktiv; q je Link speicherfrei aus y_w/|c*n| ableitbar; ABER es braucht ein
   q-ABHÄNGIGES Schema (Marson Eq. 25/26-Klasse, nicht das q-invariante ELIBB-U, an dem V1
   scheiterte), kombiniert mit dem iMEM-u_s als Slip-Ziel.

## 2026-08-22 abends — Punkt 8 gemessen: der Far-Abdruck hat zwei Gesichter (Heiko-Befund verifiziert)

Beide 4-mm-Arme bei t=335 ms, Karosseriezone, je Grobzellen-Lage vom Fahrzeug:
- **Arm 1 (linear N=16 a=0,25): BINAERER Abdruck bestaetigt.** ||u_nah-u_fern|| in Lagen 12-16
  auf ~0,5 m/s festgenagelt, +52 % Sprung ueber die Bandgrenze; Fernfeld-Textur im Band 0,10
  gegen 0,39 ungekoppelt (4x glatter), +150 % Textursprung an der Kante. Das ist die
  a-UNABHAENGIGE fneq-Loeschung des EQ-Arms, nicht das Gewichtsprofil.
- **Arm 2 (Plateau N=8 a=0,5): KEINE Kante** (+7 % Delta-u, +14 % Textur an der Grenze). Der
  sichtbare Abdruck ist ein TEXTUR-KONTRAST: Lage 1-2 rauer als ungekoppelt (3,1-3,5 gegen
  2,7-3,8 gemischt -- die 4x4x4-Blockmittel-Flecken werden mit a=0,5 voll eingepraegt),
  Lage 3-8 halb so rau wie die Umgebung (fneq-Loeschung glaettet). Ein Hof, keine Kante.

FOLGE (Heiko-Regel "wenn binaer -> FNEQ rueckt vor"): FNEQ=1-A/B auf 8 mm gestartet
(eine Variable gegen b8_breit_n16). Der Blockflecken-Anteil bleibt davon unberuehrt ->
CFD_N2F_ZEITMITTEL (Arbeitsliste Punkt 4) und langfristig weichere Raumrestriktion.

## 2026-08-22 abends — FNEQ=1 ist Produktionsstandard (Heiko-Entscheid)

A/B `b8_breit_n16_fneq` gegen `b8_breit_n16` (eine Variable): Texturkante an der Bandgrenze
+171 % -> +8 % (weg), Band-Textur zurueck auf Umgebungsniveau, Lauf stabil (rc=0, kein
Waechter), Paarungsbeweis Slots 25/26 scharf bestanden (0 Verletzungen). Ehrlicher Preis:
cd_druck 1,568 -> 1,647 (2 sigma) -- der Kopplungs-Cd-Effekt schrumpft von -0,19 auf -0,11,
der Rest war EQ-Artefakt (kuenstliche Dissipation drueckte Cd mit).
Default CFD_N2F_SCHALE_FNEQ: 0 -> 1. EQ-Arm nur noch explizit, mit Warnung. Default-AUS der
Kopplung bleibt bitidentisch. EQ-Altlaeufe sind gegen FNEQ-Laeufe nicht direkt vergleichbar.

## 2026-08-22 abends — Facetten-1a-Screening: Budgets sind nicht der Hebel

B4t (Tangentialbudget x2): VERWORFEN -- cd_druck 1,6465 -> 1,7403 (2,4 sigma, weg von OF13),
Slot 10 nur -15 %, Slot 16 +27 % (Last verschoben, nicht geheilt). Bsn (sn-Budget x2):
neutral -- Slot 16 -43 %, Kraefte unveraendert (0,0 sigma). Kombination entfaellt.
Budgets bleiben 1,0. KERNZAHL: 22,9 von 23,6 Mio ohneTang-Ereignissen am Fahrzeug sind
Einzellink-diagonal (97 %; Kugel 94 %) -- die Klasse ist so gross wie die u_s-Klemme und
NUR per ELIBB (1c) erreichbar. 1a schliesst damit: weiter mit 1b (FERN_FACETTEN +
Band-Wandrueckzug) und 1c (Plan: FACETTEN-ELIBB-PLAN.md).

## 2026-08-22 spaet — 1b-Screening: Wandrueckzug ist der Cz-Fund des Tages

Code vorab per Pruefagent bestaetigt (e6a2c96; NURWAKE-Freigabeschiene dabei als unsicher
erkannt und gesperrt -- nur wandfrei>0 ohne NURWAKE ist EsoPull-bewiesen). Ergebnis:
cz_druck_rest -0,5107 -> **-0,7707** (w3) / -0,7721 (w3ff) -- bester Wert der Kampagne,
besser als die ungekoppelte Kontrolle (-0,652), Richtung OF13 (-1,301). Die Wand-Schreibung
des Bandes hatte den Abtrieb unterdrueckt (Mechanik vom Abloesungs-Pruefer vorhergesagt).
Preis: cd_druck +0,16 (5,7 sigma), RMS-Diff 3,4 -> 5,7, Fernfeld loest wieder selbst ab
(49,5 %); FERN_FACETTEN druecken das auf 44,1 % -- richtige Richtung, schwach bei 32-mm-
Treppe, echter Test auf 4 mm. Cz-Signifikanz erst 1,8 sigma -> Schaerfungslauf b8_1b_w3_k1
(Kadenz 1) laeuft.

## 2026-08-22 spaet — Plateau-hinter-Wandrueckzug-Kette (Heiko-Synthese, 3 Arme, Kadenz 1)

Alle drei Arme statistisch ununterscheidbar vom Bezug (cz_rest 0,1-0,7 sigma): der Cz-Gewinn
haengt ALLEIN am Wandrueckzug, nicht an Profilform oder Bandbreite. Heikos Zielkonfiguration
(N=8, WANDFREI=2, Plateau 2 ab Lage 3, Halbierung, Lage 8 = 0) liefert gleiche Kraefte bei
BESTER Feldkonsistenz der vier (RMS 5,21 gegen 5,61-5,92) und kleinstem Band (~1/3 der
N16-Zellen). => Rationaler 4-mm-Produktionskandidat: b8_sp_c-Konfiguration + WANDFREI.
Produktionsfreigabe liegt bei Heiko. Code der Verschiebung pruefagenten-bestaetigt
(0e01748 sauber; Nacharbeiten 9069bc8: Randfall-Guard wf+plateau>=N-1, Census-Ansage).

## 2026-08-22 nacht — 4-mm-PRODUKTION f4_wandfrei_prod (Heiko-Freigabe): BAUPLAN-KRITERIUM 2 ERFUELLT

Absturz bei t=0,405 s (CL_OUT_OF_RESOURCES -5: Zeichentool auf der Desktop-B70 waehrend des
Laufs); Daten bis dahin gueltig, 205 Samples im Fenster 0,2-0,405. Gegen f4_std_diff2
(gleiches Fenster): **cz_druck_rest -0,8421 -> -0,9262** -- Bauplan-Kriterium 2 (<= -0,92)
ERSTMALS erfuellt. cd_druck +0,11 (wie prognostiziert), cz_band gehalten. Effekt bei 4 mm
kleiner als bei 8 mm (-0,084 gegen -0,27); Prognose nicht falsifiziert, unterer Bandrand.
Fehlend: letzte 95 ms + VTK-Dump. Innere Bandkante am Feld vermessen (Lage 2->3: du 8,0->3,8,
Textur 3,8->1,1) -- Anrampe als naechster 8-mm-Arm geplant. FNEQ-Wirkung am Bandende
bestaetigt (weiche Kante +30 % statt +150 %).

## TAGESABSCHLUSS 2026-08-22 — Kopplungs-Kampagne: Bauplan-Kriterium 2 erfuellt

**Kernergebnis:** cz_druck_rest 4 mm: -0,842 -> **-0,926** (Kriterium <= -0,92 ERSTMALS
erfuellt), Kette FNEQ-Standard -> Wandrueckzug (WANDFREI=2) -> Heiko-Plateau-Profil, jede
Stufe 8-mm-validiert. Preis cd_druck +0,11 (prognostiziert). Produktion f4_wandfrei_prod
starb bei t=0,405 am Zeichentool (Desktop-B70, CL_OUT_OF_RESOURCES) -- 205 Samples gueltig,
VTK fehlt, Wiederholung nach Reboot empfohlen. Grafikfehler nach dem Vorfall -> Neustart.

**Heute gebaut und validiert:** band_bilanz.csv; Profil 2 (Plateau+Halbierung, spaeter ab
WANDFREI+1 verschoben); CFD_N2F_BAND_WANDFREI (+Kastenkern-Scan); NURWAKE (253-Sperre);
CFD_FAC_BUDGET/_SN (verworfen als Hebel); Slot-27-Split (97 % ohneTang = ELIBB-heilbar);
Slots 25/26 Paarungsbeweis (ersetzt blinden Zaehler); FNEQ=1 Default; ELIBB P1
(Surface-Nets-Remesh, ungetestet -- Rauchtest nach Reboot); ~40 Pruefagenten-Befunde
behoben ueber ~15 Pruefrunden.

**Nach dem Reboot, Reihenfolge:** (1) P1-Rauchtest CFD_FACETTEN_REMESH=1 (Kurzlauf,
remesh_flaeche.vtk sichten); (2) f4_wandfrei_prod-Wiederholung NACH Heiko-Go (VTK; waehrend
des Laufs keine GPU-Tools am Desktop!); (3) Anrampe gegen die innere Bandkante (8-mm-A/B:
0/0/0,25/0,5/1,0/0,5/0,25/0); (4) ELIBB P2 nach Sichtung der Remesh-Flaeche.

## 2026-08-22 spaet — ELIBB P1 Rauchtest + Pruefagent (Neustart-Sitzung)

**Rauchtest (`p1_remesh_rauch`, 8 mm, `CFD_FACETTEN_DIAG=2`, 16 s):** 780.874 Grenzquads,
780.402 Vertices, 1.561.748 Dreiecke; q-Abdeckung 3.508.284/3.508.284 Links (100 %),
0 Fallback, Mittel q=0,499, Dezile 549/46898/209815/262227/1113963/1361494/262790/208593/41615/340.
VTK 50 MB nach `export/p1_remesh_rauch/nah/remesh_flaeche.vtk`.

**Heikos Sichtung:** Duennteile sauber; **enge Bereiche (Radhaus, Kuehler) werden teils
zugeschmiert.** Deckt sich mit Pruefbefund 3 und ist der erste echte P1-Mangel.

**Pruefagent (adversarial, gegen 5257fe0 + 4795880):**
- **HOCH (Abnahmeluecke, kein Codefehler):** 100 % + Mittel 0,499 beweisen die richtige
  SEITE nicht — ein Spiegelfehler q->1-q saehe identisch aus. Fehlende Abnahmen:
  (a) Schnittzahl je Link mit t in (0,1] muss UNGERADE sein (gerade = Gegenflaechentreffer);
  (b) Kugel-Referenz gegen analytisches q (Rauchtest lief nur am Fahrzeug);
  (c) Achs- gegen Diagonallinks getrennt histogrammieren.
- **MITTEL (Bug):** Bin-Schluessel setup.cpp:612/638 nutzt `(Nz+2u)` als y-Blockgroesse
  statt `(Ny+2u)`. Nahfeld hat Ny>Nz -> Bin-Kollisionen. q bleibt korrekt (ray_tri prueft
  exakt), aber verschmolzene Bins/Totkandidaten. Vor 4 mm fixen.
- **MITTEL (= Heikos Beobachtung):** Komponentenklemme setup.cpp:597 erlaubt, dass bei
  1-Zellen-Spalt BEIDE Waende exakt die Fluidmitte erreichen -> Seitenquad degeneriert
  (det=0) und der Spalt ist aerodynamisch zu, obwohl das Voxelgitter ihn offen hat.
  Kreuzen verhindert die Klemme, Beruehren nicht.
- **Bestaetigt (Entwarnung):** 2-Bin-Suche ist vollstaendig, auch diagonal; Cutoff
  t in (1e-9, 1+1e-6] korrekt; 100 % strukturell plausibel (Klemme +-0,5); Host-`cd[18]`
  stimmt EXAKT mit der Kernel-`c()`-Reihenfolge (kernel.cpp:940-942) — P2-Auflage erfuellt;
  Dezil-Klemme korrekt; Flaeche am Latsch bewusst offen.
- **NIEDRIG:** Quads nicht orientiert (blockiert den Normalen-Vorzeichentest, stoert
  VTK-Shading); 4 mm ~3,1 M Quads, Bins 0,7-1,5 GB (CSR statt unordered_map empfohlen),
  VTK ASCII ~200-250 MB -> binaer oder gaten.

**P1-Korrekturliste, in dieser Reihenfolge:**
1. Engstellen-Schutz: Taubin-Klemme lokal auf die freie Weite begrenzen bzw. Vertices
   in Spalten <=2 Zellen einfrieren (Voxelgeometrie schlaegt dort Glaettung).
   Zaehler: Anzahl eingefrorener Vertices + min. Restweite je Spalt.
2. Bin-Schluessel `(Nz+2u)` -> `(Ny+2u)`.
3. Die drei fehlenden Abnahmezahlen (ungerade Schnittzahl, Kugel-q, Achs/Diagonal).
4. Erst danach P2 (fac_q + Kernel).

## 2026-08-22 Abschluss II — ELIBB P1 gehaertet, Radhaus-Ursache gefunden

**Ergebnis in einem Satz:** P1 steht und ist messtechnisch abgesichert; die zugeschmierten
Radhaeuser sind KEIN Flaechenproblem, sondern die 6er-Konnektivitaet des Void-Fill.

**Gebaute Abnahmen (alle mit feuerndem Zaehler, Iron Rule 8):**
- PARITY -- Schnittzahl je Verbindung muss ungerade sein. Bestanden, beide Faelle.
  (Loest den HOCH-Befund des Vorpruefers: 100 % Abdeckung belegte die SEITE nicht.)
- DURCHSCHUSS -- Achs-Fluidverbindungen duerfen nicht gekappt werden. 0 von 3.749.916.
- TOPOLOGIE -- Kantenzensus + Euler: chi=-144, 560 Quetschkanten, 464 offene Kanten.
- KUGEL-REFERENZ -- analytisches q, Bias/RMS/max.
- FREIE WEITE -- Engstellenmass je wandnaher Zelle, q getrennt eng/offen.
- VOID-FILL-KONNEKTIVITAET -- 6er- gegen 18er-Flutung, Differenz mit Bereichsangabe.

**Behobene Fehler:** Bin-Schluessel Ny/Nz (latent, Histogramm bitgleich = Abnahme);
680 an Quetschkanten verschweisste Vertices werden festgehalten; unbenutztes `len`.

**Zwei eigene Aussagen zurueckgenommen:**
1. "Der Spalt ist aerodynamisch dicht" -- der Durchschusstest sagt das Gegenteil.
   Es ist Einwoelbung (q 0,5000 -> 0,4683 in Engstellen), kein Verschluss.
2. Die Glaettung sei die Ursache des zugeschmierten Radhauses. Heikos Sichtung
   (ITER 3/8/15 identisch am Radhaus) hat das widerlegt -- und direkt auf die
   richtige Stufe gezeigt.

**Ein Agentenvorschlag per Messung verworfen:** q direkt gegen die STL. Nur 40,5 % der
Solid-Links haben eine STL-Flaeche in Reichweite; 59,5 % sind SAT-Schale und Void-Fill.

**OFFEN, zuerst in der naechsten Sitzung:** der Pruefagent auf dem P1-Haertungs-Diff
(f84f6b3) lief beim Sitzungsende noch. Nach Iron Rule 2 gilt KEIN P1-Messwert als
bestaetigter Befund, bevor er durch ist -- seine Befunde vor P2 einarbeiten.

**Reihenfolge fuer die naechste Sitzung:**
(1) Pruefagenten-Befunde zu f84f6b3 einarbeiten; (2) P2 nach FACETTEN-ELIBB-PLAN.md
(q-Puffer als uchar, Esoteric-Pull-Konvention klaeren, Kugel-Gate zuerst);
(3) Void-Fill-A/B mit KONN=18 (Arbeitsliste 12) NACH P2;
(4) Produktions-Wiederholung f4_wandfrei_prod und Kadenz-1-Bezug -- beides nur nach
    Heiko-Go, und waehrend des Laufs keine GPU-Werkzeuge am Desktop.

## 2026-08-22, NACHTRAG — Pruefagent auf f84f6b3: mehrere meiner Aussagen tragen nicht

Der Pruefer lief beim Sitzungsende noch und kam nach dem Abschlussbericht zurueck.
Iron Rule 2 greift: die folgenden Aussagen von mir sind hiermit ZURUECKGENOMMEN oder
eingeschraenkt. Wer hier weiterarbeitet, liest diesen Abschnitt VOR den beiden davor.

### FAELLT: "ABNAHME DURCHSCHUSS bestanden = es wird nichts verschlossen"
Der Test ist **strukturell blind fuer genau den Fall, fuer den ich ihn gebaut habe.**
In einem 1-Zellen-Spalt liegen beide Waende symmetrisch +-q um die Zellmitte; der
Laengslink laeuft ZWISCHEN ihnen hindurch und wird auch bei 87 % Verschluss nie
geschnitten. Bei vollem Zuschmieren liegt er exakt AUF der Flaeche -> det=0 in ray_tri
-> kein Treffer. "0 von 3.749.916 gekappt" ist mit einem zugeschmierten Spalt voll
vereinbar. Der eigene Log belegt es: Engstellen-q_min 0,0647 heisst **Restspalt 0,13
Zellen** -- und daneben steht "ABNAHME bestanden".
**Die richtige Kennzahl liegt in den Daten und wird nicht gebildet:
Restspaltweite = q+ plus q- je Zelle mit freier Weite 1.** Das ist der erste Bau in P1b.
Dazu: die von mir als "nachrichtlich" abgetane Diagonalzahl ist der EINZIGE reagierende
Indikator und waechst monoton 1152 -> 1240 -> 1380 (ITER 3/8/15). Fuer GEGLAETTET sind
das echte Schnitte, nicht der Streiffall (der erklaert nur die 316.860 bei TREPPE).

### FAELLT: "ABNAHME PARITY bestanden"
Die Zusammenfassung gleicher t (noetig wegen der Quad-Diagonale) fasst auch den
Zielfehlerfall zusammen: zwei Waende eines Spalts, die beide an der +-0,5-Klemme
liegen, sind KOPLANAR und liefern exakt gleiches t -> zusammengefasst -> Parity
ungerade -> "bestanden". Zusaetzlich ist die Flaeche gar nicht geschlossen
(464 Kanten mit Inzidenz 1, Randflaechen am Domaenenrand entstehen bewusst nicht),
die Parity-Praemisse gilt am Latsch also nicht.

### FAELLT: "Der Bauplan schliesst die STL zu Recht aus" (die MESSUNG deckt das nicht)
`if(best>1.0+1e-6) return false;` kann "keine STL da" und "die Voxelwand steht VOR der
STL" nicht unterscheiden. Die 59,5 % sind damit eine Messung der SAT-Schalen-Aufdickung,
kein Urteil ueber die STL. Die faire Pruefung kostet nichts (Fenster t<=3, oder
schlicht: lagen ueberhaupt Dreiecke in den Bins). Was bleibt: Voxelkoerper und STL
weichen um rund 0,77 Zellen voneinander ab -- STL-q gegen Voxel-flags waere INKONSISTENT,
weil der LBM an der Voxelwand abprallt. Das ist ein anderer und schwaecherer Grund als
der, den ich in den Commit geschrieben habe.

### EINGESCHRAENKT: die Kugel-Grundwahrheit
R_vol=19,522 ist an den Pruefling angepasst. Der Koerper IST um 0,77 Zellen aufgedickt
(SAT-Schale +4596, Void-Fill +1051; R_eff Symmetrieebene 19,49). Damit ist der GROESSTE
Geometriefehler per Konstruktion unsichtbar: die Glaettung poliert 0,11 RMS, waehrend
0,77 Zellen Versatz unbeanstandet stehen. Der RELATIVE Vergleich Treppe gegen Geglaettet
bleibt gueltig (gleiche Referenz); jede ABSOLUTE Genauigkeitsaussage nicht.
Der Bias (-0,033) ist ausserdem ein Zensurartefakt: `ok=(tq>0 && tq<=1)` wirft die
Links mit t<=0 einseitig weg (2248 = 8,2 %). Zum Vorzeichen: Schrumpfung eines konvexen
Koerpers ergaebe POSITIVEN Bias; beobachtet ist er monoton negativ, und Achs- gegen
Diagonal-q fallen im Verhaeltnis 0,73 (rund 1/Wurzel2) -- das ist ein gleichfoermiger
Normalversatz NACH AUSSEN. Die Glaettung dickt den Koerper zusaetzlich auf.

### FALSCHE ZAHL IM COMMIT: -41 % gehoert zu ITER=15, ausgeliefert wird 8
Bei Default 8 sind es RMS 0,17883 = **-32,3 %**. Ebenso: "Engstellen 0,4683 / min 0,0647
/ offen 0,5005" ist p1_fz15; bei Default 8 gilt **0,4774 / 0,1685 / 0,5004**.
Zwei Kopfzahlen beschreiben eine Einstellung, die derselbe Commit abschafft.

### NICHT VALIDIERT: der Quetschkanten-Fix
A/B bei ITER=15 ohne gegen mit Fix: Engstellen-q 0,4688 -> 0,4683 (schlechter),
q_min unveraendert 0,0647, Diagonalschnitte 1236 -> 1380 (+11,7 %). Beide gerichteten
Indikatoren zeigen in die falsche Richtung. Festhalten von 680 Vertices reicht nicht,
weil die NACHBARN ungeklemmt weiterwandern. Der Schluessel selbst ist kollisionsfrei.

### PRAEZISIERT: der Bin-Fix war nicht "latent"
Er kollidierte tatsaechlich (yy+1 bis 228 gegen Nz+2=141 bei der Kugel, 318 gegen 243
beim Fahrzeug) -- folgenlos nur, weil die aliasierten Dreiecke 141 bzw. 243 Zellen
entfernt liegen und von ray_tri verworfen werden. Ausserdem ist aus den Logs NICHT
feststellbar, ob der p1_stl_ab-Build den Fix schon enthielt: das Bitgleichheits-A/B
ist damit nicht zuordenbar und muss wiederholt werden.

### WAS TRAEGT
- Rohe Voxelflaeche liefert q EXAKT 0,5000 -- alle neun Punkte der Iterationskurve
  exakt bestaetigt. Die Kernaussage von P1 steht.
- `CFD_FACETTEN_REMESH=0` ist bitidentisch: geprueft und bestaetigt (liest flags an
  sechs Stellen, schreibt an keiner; beide Aufrufstellen gegatet).
- Default 8 bleibt richtig -- aber die Begruendung aendert sich. Es gibt KEINEN Knick:
  RMS minus RMS_unendlich zerfaellt sauber exponentiell (tau rund 6,5). Das belastbare
  Kriterium ist das a2-Budget an der FAHRZEUG-Engstelle: a2 = 4,9 bei ITER 8 gegen
  14,5 bei ITER 15. Danach ist 8 vertretbar und 15 nicht.
- Bin-Schluessel-Kollision: real und behoben.

### UNGEPRUEFT
Der Void-Fill-Befund (116.651 Zellen, Konnektivitaet 6 gegen 18) stammt aus demselben
Sitzungscode und wurde von diesem Pruefer NICHT betrachtet. Er gilt nach Iron Rule 2
als unbestaetigt, bis ein Pruefer ueber 02b20e6 gelaufen ist.

### REIHENFOLGE, KORRIGIERT
(1) Restspaltweite q+ plus q- fuer Zellen mit freier Weite 1 bauen -- das ist die
    Abnahme, die der Durchschusstest haette sein sollen; (2) Diagonalschnitte vom
    Nachrichtlichen zur Abnahme hochstufen; (3) Zensur im Kugelvergleich beheben und
    die Aufdickung als eigene Groesse ausweisen; (4) Quetschkanten-Fix entweder
    reparieren (Nachbarn mitklemmen) oder zuruecknehmen; (5) STL-Frage fair messen
    (Fenster t<=3); (6) Bitgleichheits-A/B des Bin-Fix wiederholen; (7) Pruefer ueber
    den Void-Fill-Befund; DANN P2.

## 2026-08-23 — Pruefagent ueber den Void-Fill-Befund: Deutung WIDERLEGT

Der letzte offene Auditpunkt vom 22.08. ist durch. Code und Zahl tragen, meine Deutung nicht.

### TRAEGT
- Die 18er-Flutung ist korrekt: gleiche Saat, gleiche Schranken, gleiches Praedikat, beide
  Flutungen komplett VOR der Fuellschleife; `reach`/`reach18` werden nicht veraendert.
  `reach` ist per Induktion Teilmenge von `reach18`, `nur18` = filled(6) - filled(18) exakt.
- Default (KONN=6) bitidentisch: `zu=!r6` ist woertlich die alte Bedingung.
- Die Zahl 116.651 von 131.322 steht.

### FAELLT: "die beiden vorderen Radhaeuser"
Nachgerechnet: Fahrzeug fein X[58,577] (519 Zellen = 4,15 m), Vorderachse bei rund 20 % der
Laenge, also x ~ 162. Der betroffene Bereich endet bei **x=141 -- komplett VOR der
Vorderachse**. Y[109,207] ist Mitte +-0,39 m, die Raeder sitzen bei +-0,6 bis 0,8 m:
**keine einzige betroffene Zelle liegt an den Flanken.** Fuellgrad der BBox 35 % -- ein
Klotz, nicht zwei duenne Seitentaschen. Ehrliche Aussage: **zusammenhaengendes Gebiet im
vorderen Mittelbau vor der Vorderachse, 0,2 bis 0,69 m ueber der Strasse = Kuehler und
Vorderwagen.** Die Commit-Ueberschrift von 02b20e6 ist damit nicht gedeckt. Radhaeuser sind
nach unten und aussen offen und werden vom Void-Fill gar nicht erreicht.
=> DAMIT IST HEIKOS RADHAUS-BEOBACHTUNG WIEDER UNERKLAERT. Weder Glaettung (alle drei
ITER-Stufen gleich) noch Void-Fill. Naechste Verdaechtige: die SAT-Schale (438.055 Zellen
ergaenzt) und die nicht nach aussen orientierte Quad-Wicklung, die das VTK-Shading stoert --
letzteres waere kein Geometrie-, sondern ein Darstellungsbefund und billig zu pruefen.

### FAELLT: KONN=18 als Produktionsschalter (HOCH, abgeraten)
- Digitale Topologie (Rosenfeld/Kong, gueltige Paare 6/26, 26/6, 6/18, 18/6): Fluid=18
  ERZWINGT Solid=6 -- zwei kantenberuehrende Solidzellen gelten dann als nicht dichtend.
  Fluid=6 / Solid=26 ist die konservative Standardwahl bei Voxel-Hohlraumfuellung.
- Der D3Q19 streamt den Kantenlink zwar (w=1/36), aber durch einen Kanal mit geometrisch
  NULL Querschnitt und ohne aufgeloeste Wandschicht. "Erreichbar" ist nicht "durchstroembar";
  das ist genau das numerische Leck, das der Void-Fill beseitigen soll.
- Aufloesungsprobe: gefuelltes Volumen 0,0672 m3 fein gegen 0,0535 m3 grob (-20 %) -- dieselbe
  Hoehle auf beiden Gittern, also Geometrie. Der Diagonalanteil aber 89 % fein gegen 3 % grob:
  die Pinholes sind Diskretisierung, nicht Geometrie.
- Kopplungsrisiko: der Motorraum kann nur WACHSEN (+8,7 % fein, +52 Zellen grob) -- die beiden
  gekoppelten Koerper stellten danach unterschiedliche Geometrie dar.
=> KONN=18 bleibt reine DIAGNOSE. Arbeitsliste 12 entsprechend zurueckgestuft.

### NIEDRIG, offen
Invariante `r6 && !r18` wird nicht geprueft (ein print_error kostet nichts); die Ansage
"nur ueber eine Diagonale erreichbar" ist unpraezise (der HOHLRAUM haengt an einer Diagonale,
nicht jede Zelle); `env_u`-Falle: 7..17 heisst still 6, >=26 still 18; die zweite Flutung
laeuft unbedingt in JEDEM Lauf (bei 4 mm 63,6 MB und ~125 M BBox-Zellen x 18, einthreadig,
Setup-Zeit ungemessen) und sollte auf die Mesh-BBox begrenzt und hinter ein Gate gestellt werden.

## 2026-08-23 Abschluss — Grenzschicht-Diagnostik: was traegt und was nicht

**Die ehrliche Bilanz des Tages: vier eigene Deutungen zurueckgenommen, kein Cz-Hebel gewonnen,
aber der Suchraum deutlich verkleinert und drei Messwerkzeuge gebaut.**

### TRAEGT (ohne Vorbehalt)
- **Kanal-cf exakt reproduziert:** 0,0015369 gegen den Aktenwert 0,00154 = **-55,4 %** gegen die
  Referenz 0,0034424. Aus etabliertem Code, nicht aus meinem.
- **Der Kanal ist NICHT SGS-limitiert.** Das wahre U_b+ ist 36,0 statt 24,1 (Log-Layer-Versatz
  +12). Ein 29-%-nu_t-Defizit in einer Zelllage verschiebt U+ um O(1-3), nicht um 12. Die
  cf-Luecke ist **wandmodell-dominiert**. (Die frueher gedruckte Zeile "Ub_plus = 24,1035 trifft
  das Ziel" ist KEIN Befund -- das ist der CFR-Regler auf seinem Ziel, normiert mit dem SOLL-u_tau.)
- **Der Fahrzeug-Einlass hat keinerlei Anstroemturbulenz** (setup.cpp, u_lat/0/0), der Kanal saet.
- **Die Voxelaufdickung ist gemessen:** 55,70 % der Solid-Links haben die STL hinter der Voxelwand.
  Heiko-Entscheidung: bleibt so ("lieber zu dick als gar nicht voxeliert").

### ZURUECKGENOMMEN (vier eigene Deutungen)
1. "Der Spalt ist aerodynamisch dicht" -> Restspalt-Messung sagt 1,0033 Zellen, nichts unter 0,5.
2. "Die Glaettung schmiert das Radhaus zu" -> alle ITER-Stufen identisch; es war der Void-Fill.
3. "Die Radhaeuser sind die Ursache" -> der Void-Fill-Bereich endet VOR der Vorderachse; es ist
   Kuehler und Vorderwagen. Und KONN=18 bleibt Diagnose (Fluid=18 erzwingt Solid=6).
4. "Das Fernfeld liegt eine Dekade hoeher" -> Faktor 1,8 im Median.
5. "Die Ueberdissipations-Lesart traegt, C senken ist die richtige Richtung" -> FALSCH. Der
   richtige Massstab ist 0,714*kappa*y+ (Smagorinsky liefert im Gleichgewicht strukturell 71,4 %,
   aufloesungsunabhaengig). C 0,173 -> 0,1 landet bei 0,7x Gleichgewicht = MSD-Zustand.
6. "Der Kanal liegt exakt auf der Erwartung, der Ueberschuss am Fahrzeug ist geometrisch" ->
   FALSCH, und zwar durch einen inneren Widerspruch: y+ = 136,5 ist der SOLL-Wert. Der Lauf misst
   u_tau IST = 0,002008 (Faktor 0,669) und y+ = 92,0 -- dieselbe Zahl wie mein cf-Defizit, denn
   cf ~ u_tau^2. Mit y+ = 92 ist die Erwartung 26,9, und der Kanal liegt SELBST 1,3-1,6x darueber,
   auf einer ebenen Wand ohne jede Treppe.

### GEBAUT (Werkzeuge, alle hinter CFD_SGS_DIAG, Default aus = kein alloc, keine Emission)
- nu_t/nu_0-Dekadenhistogramm je Domaene (Slots 28..32)
- Wandlagen-Histogramm mit Bins 5/15/30/60 (33..37), Teilsatz anliegend u_x>0 (38..42),
  oberer Schwanz 60-120/120-240/240-480/>=480 (43..46)
- Restspaltweite, Diagonalschnitt-Abnahme, Topologie/Euler, Kugel-Grundwahrheit, freie Weite,
  Void-Fill-Konnektivitaetsdiagnose (alles im Remesh-Pfad)

### BEKANNTE GRENZEN DIESER WERKZEUGE (wichtig fuer den Nachfolger)
- **Die Wandlagen-Kennzahl ist in einer Konstantspannungsschicht fast tautologisch:** aus
  (nu+nu_t)|S| = tau_w mit nu_t = C^2|S| folgt nu_t/nu direkt aus tau_w. Sie misst also weitgehend
  tau_w zurueck, nicht Modellguete.
- Der Zaehler misst |S| INSTANTAN, der Massstab kommt aus der MITTLEREN Scherung. Im
  instationaeren Nachlauf ist E[|S|]/|S_quer| deutlich groesser als 1; Verhaeltnisbildung heilt das nicht.
- Der Massstab haengt mit 1/y_w^2 von der Wandlage ab. Kanal: y_w exakt 0,500. Fahrzeug:
  Median 0,500, q10 0,170, q90 1,069 -- 15,9 % der Wandzellen haben allein daraus einen
  >=2,9x hoeheren Massstab.
- kappa*y+ setzt GLEICHGEWICHT voraus. Am Fahrzeug gilt das an Scheiben, Heck, Radhaeusern und
  Latsch nicht. Der u_x>0-Filter rettet das nicht (Kanal 0 % gefiltert, Fahrzeug 27 %).
- Kein Fall in diesem Projekt ist deterministisch: zwei identische Laeufe liefern im dd-Fall UND
  an der Kugel verschiedene Kraefte. Ein Datei-Bitvergleich ist als Abnahme nicht verfuegbar.

### BEHOBEN AN DEN WERKZEUGEN (letzte Runde)
Warmlaufsperre `CFD_SGS_DIAG_AB` (der Zaehler lief bisher ab Schritt 1, am Fahrzeug stammten
40 % der Stichproben aus der Einschwingphase); Etikett "jede 8. Zelle" auf 1/64 richtiggestellt;
der HARTKODIERTE Massstab "~21 bei y+ = 70,7" ist raus -- er wurde auch im Kanal gedruckt, wo
26,9 gilt, und hat meine eigene Analyse in die Irre gefuehrt; Wickelwaechter jetzt je Domaene
und ueber alle 19 Slots statt ueber die Domaenensumme.

## 2026-08-23, Nachtrag zum Tagesende

**Kanal-Aufloesungsreihe, ZWISCHENSTAND (Lauf N=76 war beim Sitzungsende noch nicht fertig):**
N=38 cf = 0,0015369 (-55,4 %), N=76 bei 9.580 Samples cf = 0,00159835 (**-53,6 %**).
Halbe Zellweite bringt 1,8 Prozentpunkte -- praktisch KEINE Aufloesungskonvergenz. Das stuetzt
die Diagnose "wandmodell-dominiert, nicht aufloesungslimitiert", ist aber als Zwischenstand
gekennzeichnet und gilt erst nach Laufende.

**EIGENER FEHLER, VRAM:** Ich habe den Desktop-VRAM live ueber `/proc/*/fdinfo` gemessen und kam
auf 7,4 GB. Richtig sind **1,1 GB** (xe-TTM-Allokator: total 32.656 MiB, free 31.529 MiB). Faktor
sieben zu hoch, weil fdinfo geteilte Puffer je Deskriptor mehrfach zaehlt. Der Wissensspeicher
enthaelt genau diese Warnung samt dem richtigen Weg -- ich habe gemessen, statt nachzuschlagen.
Daraus hatte ich zusaetzlich die falsche Erklaerung gebaut, der Produktionslauf habe 4 GB zu
viel gebraucht; tatsaechlich lag die Reserve bei 2,2 GB und das Zeichentool hat mehr angefordert.

## 2026-08-24 — Delta-Reihe Kanal, und der Determinismus ist lokalisiert

### KORREKTUR zu gestern: der Kanal IST deterministisch
Gestern stand hier "kein Fall in diesem Projekt ist deterministisch". Das war zu weit gefasst:
geprueft waren nur der dd-Fall und die Kugel -- **beide haben einen Druckauslass**. Heute
gemessen, drei identische Kanallaeufe bei N=20 (kn_20, kn_20_a, kn_20_b):
```
cf_kraftbilanz = 0.00166673  in ALLEN DREI, Spannweite 0,00 %
```
Der Kanal hat keinen Druckauslass (setup.cpp:1398 sagt es selbst). Damit ist die
Nichtdeterminismus-Quelle punktgenau: **die po_mean-Reduktion am Druckauslass**, nicht
"irgendeine Reduktionsreihenfolge". Der Bitvergleich als ABNAHMEMITTEL ist nicht verloren --
am Kanal sofort verfuegbar, am Fahrzeug nach Determinisierung dieser einen Reduktion.

### Delta-Reihe im Kanal, drei Punkte, alle auf 80 ETT
| | u_tau-Faktor | Delta+ | y1+ | Erwartung 0,714*kappa*y1+ | gemessen | Verhaeltnis | cf |
|---|---|---|---|---|---|---|---|
| N=20 | 0,716 | 371,3 | 185,7 | 54,3 | 84,9 | **1,56** | -51,6 % |
| N=38 | 0,669 | 182,6 |  91,3 | 26,7 | 39,9 | **1,49** | -55,4 % |
| N=76 | 0,725 |  98,9 |  49,5 | 14,5 | 15,1 | **1,05** | -53,5 % |

**Das Verhaeltnis faellt MONOTON mit der Verfeinerung** -- und weil der Kanal deterministisch
ist, sind die Unterschiede real und kein Rauschen. Das ist die MSD-Signatur auf belastbaren
Daten: die modellierte Mischung faellt gegenueber dem Gleichgewichtsbedarf, waehrend cf nicht
folgt (die aufgeloeste Spannung uebernimmt nur teilweise und ungleichmaessig).

**EINSCHRAENKUNGEN, ausdruecklich:**
- cf ist NICHT monoton (-51,6 / -55,4 / -53,5). Bei deterministischen Laeufen heisst das:
  echt, aber nicht als einfacher Trend lesbar.
- Der N=20-Wert ruht auf einer Bin-Mittenschaetzung -- dort liegt praktisch alles im >=60-Bin
  (99,8 % davon in 60-120). Feinere Bins waeren noetig, um ihn zu haerten.
- Zwischen Delta+ 371 und 183 bewegt sich das Verhaeltnis kaum (1,56 -> 1,49); der ganze
  Abfall sitzt zwischen 183 und 99.
- Alle drei Punkte enthalten die 20 ETT Warmlauf im Zaehler (Warmlaufsperre bewusst NICHT
  gesetzt, damit die drei gleich behandelt sind).

### MEINE PROGNOSE WAR FALSCH
Vorhergesagt (PROGNOSE-KANAL-N20.md, vor dem Lauf): Verhaeltnis 2,0 (Spanne 1,8-2,3),
cf -57 bis -58 %, u_tau-Faktor 0,62. Gemessen: 1,56 / -51,6 % / 0,716. Alle drei daneben,
der u_tau-Faktor sogar in der falschen Richtung (er ist nicht monoton in der Aufloesung).
Das VORAB NOTIERTE FALSIFIKATIONSKRITERIUM war dagegen brauchbar: "liegt das Verhaeltnis
nicht ueber 1,49, ist MSD widerlegt" -- es liegt darueber, also ueberlebt die Lesart, und
ich konnte das schwache Ergebnis nicht nachtraeglich als Bestaetigung umdeuten.

## 2026-08-24 — Nichtdeterminismus vollstaendig lokalisiert (Kette sauber isoliert)

**Fuenf Messungen, jede mit ihrem eigenen Zweck:**
| | Aufbau | u-Feld | Kraefte |
|---|---|---|---|
| 1 | Kanal N=20, drei identische Laeufe | -- | Spalten 1-6 bitgleich, NUR `cf_impulsaustausch` weicht ab |
| 2 | Kugel, 20 Schritte | **bitgleich** | weichen ab (schon ab Schritt 10) |
| 3 | Kugel, voller Lauf, po_mean aktiv | weicht ab | weichen ab |
| 4 | Kugel, voller Lauf, Rand GANZ WEG (`PO_FACES=0`) | bitgleich | weichen ab |
| 5 | Kugel, voller Lauf, Rand AKTIV, po_mean nicht gelesen (`PO_HART=1`) | **bitgleich** | weichen ab |

**Zeile 5 ist der saubere Test** -- Zeile 4 war es NICHT: `PO_FACES=0` entfernt den Rand ganz,
x_max wird eine nackte TYPE_E-Zelle mit rho UND u vorgeschrieben (ueberbestimmt, reflektierend,
siehe setup.cpp:2146-2149). Das ist ein anderer Fall. `PO_HART=1` laesst den Rand stehen
(31.553 Zellen) und liest nur `po_mean` nicht.

**ERGEBNIS:**
- **`po_mean` (atomare Reduktion am Druckauslass) = Quelle der LOESUNGSdivergenz.** Sie koppelt
  jeden Schritt zurueck. Verzoegerter Einsatz erklaert: solange alle Workgroup-Teilsummen
  bitgleich sind, ist die atomare Kette permutationsinvariant; erst wenn der Nachlauf die
  Auslassebene erreicht und die Werte spreizen, zaehlt die Reihenfolge.
- **`object_force` = davon unabhaengige Quelle, betrifft NUR die Kraftmeldung.** Beleg aus dem
  Kanal: von sieben Spalten weicht genau die eine ab, die aus `object_force` stammt.
- **Alle uebrigen Atomics sind Diagnosezaehler ohne Rueckkopplung** (Codebefund des Pruefers:
  kernel.cpp:2833-2853 Workgroup-Baum deterministisch, dann atomic_add_f; update_force_field
  schreibt F ohne Atomic).
- **Aufwand:** po_mean NIEDRIG (N=31.553, rund 500 Teilsummen -> Teilsummenpuffer plus
  Ein-Work-Item-Finalkernel in Indexordnung, rund 15 Zeilen). object_force MITTEL (N=14,26 Mio,
  Zwei-Ebenen-Baum oder Host-Summe ueber die F-BBox).

**DREI EIGENE FEHLER auf dem Weg:**
1. "Kein Fall in diesem Projekt ist deterministisch" (23.08.) -- zu weit gefasst.
2. "Der Kanal ist EXAKT deterministisch" (heute frueh) -- auch falsch: `cf_impulsaustausch`
   weicht ab. Ich hatte nur `cf_kraftbilanz` verglichen. Der Befund stuetzt die
   object_force-These aber, statt sie zu widerlegen.
3. `PO_FACES=0` als Isolationstest -- entfernt den Rand statt der Reduktion.
Dazu: der Feld-Hash im Kugelfall war byteweise, der im Kanal ist wortweise. Angeglichen;
die oben zitierten Hashwerte stammen aus der byteweisen Fassung (Verdikte unveraendert,
weil je Paar dieselbe Fassung lief).

**PRAEZISIERUNG (Pruefbefund 7):** bitgleicher u-Hash heisst bitgleiches **u**, nicht
zwingend bitgleiches fi -- u ist ein Moment. Aussage also: "das u-Feld ist bitgleich",
nicht "der Zeitschritt ist deterministisch".

## 2026-08-24 — po_mean atomikfrei: der Nichtdeterminismus der LOESUNG ist behoben

**Umbau:** `po_reduce_mean` schrieb bisher `atomic_add_f(&po_mean[0], cache[0]/N_po)` -- eine
atomare Addition je Arbeitsgruppe auf einen EINZELNEN Float. Jetzt schreibt jede Gruppe
exklusiv ihren Slot in `po_part`, und der neue Kernel `po_final_mean` summiert in
INDEXORDNUNG. `po_clear_mean` entfiel ersatzlos (Endsumme schreibt mit `=`), die Startzahl
bleibt also bei drei Kerneln. Bauart wortgleich zu `kraft_facetten_gpu`, das denselben Weg
schon geht.

**METHODISCHER FUND, der vorausging und wichtiger ist als der Fix:**
Sechs identische Kugellaeufe lieferten **VIER verschiedene** u-Feld-Hashes -- und zwei davon
stimmten zufaellig ueberein. **Ein Einzelpaar ist damit kein gueltiger Determinismustest.**
Alle Isolationstests dieses Vormittags standen auf Einzelpaaren; die Codebefunde bleiben,
die Messform war zu schwach. Gilt rueckwirkend auch fuer "der Kanal ist deterministisch"
und fuer die PO_HART-Isolation.

**ABNAHMEN:**
| | vorher | nachher |
|---|---|---|
| verschiedene u-Feld-Hashes (6 Laeufe) | **4 von 6** | **1 von 6** |
| Cd | 1,012537 +- 0,000055 | 1,012560 +- **0,000000** |
| ALT gegen NEU nach 20 Schritten | \multicolumn{2}{c}{**identischer Hash** 12537327743888629416, je 3 Laeufe stabil} |

Der letzte Test ist der scharfe: in dem Bereich, in dem die Auslassebene noch gleichfoermig ist
und beide Staende uebereinstimmen MUESSEN, tun sie es bitgenau. Mein erster Physiktest ueber Cd
war zu weich -- Cd kommt aus `object_force`, das SELBST noch nichtdeterministisch ist, das
Rauschband mischte also Loesungsdivergenz und Meldejitter (Pruefbefund 5).

**Numerik nebenbei verbessert:** vorher 494 Divisionen und 494 atomare Additionen, jetzt 493
Additionen und EINE Division. Fehler im Mittelwert von rund 1,2e-5 auf 6e-9 (Worst Case,
nachgerechnet vom Pruefer).

**OFFEN:** `object_force` bleibt nichtdeterministisch und betrifft die Kraftmeldung (14,26 Mio
Zellen, also Zwei-Ebenen-Baum oder Host-Summe ueber die F-BBox). Und der Kostenvergleich
alt/neu ist NICHT gemessen -- der Pruefer schaetzt wenige Mikrosekunden, das ist eine Luecke.

## 2026-08-24 Abschluss — Kipp-Versuch ohne gueltiges Ergebnis, aber mit einem Befund darunter

### DER BEFUND: K2 ist auch im EBENEN Kanal verletzt
Die K2-Abnahme vergleicht den Reibungspfad der Facetten gegen die Kraftbilanz und verlangt
Uebereinstimmung auf 1 %. Sie scheitert in ALLEN DREI Armen -- auch bei `kipp=0`, also an der
ebenen, gitterparallelen Wand im einfachsten denkbaren Fall. Das ist damit kein Kipp-Problem,
sondern eines der Facetten-Wandbehandlung selbst.
**Das verschiebt den Befund vom 23.08.:** die cf-Luecke von -55 % war als "wandmodell-dominiert"
eingeordnet. Jetzt sagt der Code selbst, dass sein Reibungspfad und seine Kraftbilanz nicht
einmal miteinander konsistent sind. Nicht "das Modell trifft die Physik nicht", sondern
"das Modell ist mit sich selbst uneins" -- eine Stufe darunter.
=> **Vor jeder weiteren Wandmodell-Messung aufzuklaeren.** Solange sich zwei Zahlen um mehr als
1 % widersprechen, ist mindestens eine falsch, und die -55 % stehen auf beiden.

### Der Kipp-Versuch selbst: KEIN gueltiges Ergebnis
| Arm | u_tau | y1+ | Erwartung | gemessen | Verhaeltnis | cf_kraftbilanz |
|---|---|---|---|---|---|---|
| kipp=0 (eben)    | 0,002008 |  91,3 | 26,7 |  40,0 | 1,49 | 0,00154 |
| kipp=26 (Treppe) | 0,005829 | 265,2 | 77,6 | 111,7 | 1,44 | 0,01314 |
| kipp=45 (Treppe) | 0,004587 | 208,7 | 61,1 |  73,1 | 1,20 | 0,00807 |

Das vorab notierte Falsifikationskriterium IST erfuellt (das Verhaeltnis steigt mit der Treppe
nicht, es faellt). **Trotzdem kein Befund**, aus zwei Gruenden:
1. Alle Arme sind von der eigenen K2-Abnahme disqualifiziert.
2. **Die Versuchsanlage war falsch.** "Gleiche Stroemung, nur die Wand gekippt" trifft nicht zu:
   u_tau unterscheidet sich um Faktor 2,9, cf um Faktor 8,5. Die gekippte Wand ist eine ganz
   andere, viel rauere Wand. Das haette beim Entwurf auffallen muessen.

### ZWEI EIGENE FEHLER
1. Die erste Kipp-Serie mit einem eigenen `timeout` abgeschossen -- SIGTERM ging an die Queue und
   riss den laufenden Lauf mit (k_kipp26 starb bei ETT 18 von 80). Steht woertlich in
   MEINE-FEHLER.md des Vorgaengerbaums, und ich bin trotzdem hineingelaufen.
2. Einen Versuch entworfen, dessen Kontrollannahme die Messung nicht hergibt.

### Nebenbefund zur Warmlaufsperre
`CFD_SGS_DIAG_AB=126700` (20 ETT) aendert cf NICHT (0,00153685 mit und ohne). Die Sperre wirkt
also nur auf die nu_t-Zaehler, nicht auf die Physik -- wie beabsichtigt.

# =========================================================================
# GROSSER CODE-AUDIT 2026-08-25, WELLE 1 (Funktionsebene) -- fuenf Agenten
# Anlass: K2-Verletzung im EBENEN Kanal. Heiko: "einen Agenten auf wirklich
# jede einzelne Funktion".
# =========================================================================

## K2 IST AUFGEKLAERT -- DREI GETRENNTE URSACHEN, NICHT EINE

**(1) Ebene Wand, 2,5 %: KEIN Codefehler.** Der Kernel ist entlastet -- alle Ereigniszaehler
exakt 0, Wirkpfad == Soll == fac_N(14160) x 5067, also jede Facette jeden Schritt genau einmal;
faca = 1,0 exakt, alph = 0 exakt (S1 parallel n), Snn downdatet auf exakt 0, phi1 = -twe
konstruktiv, phi2 = 0 => protokollierte Kraft IDENTISCH mit angewandter und mit dem wahren
Austausch. ZWEI Agenten unabhaengig: der CFR-Regler schwingt im K2-Fenster mit +-30 %
(f_lat 1,94e-7 bis 3,87e-7, Periode rund 20 ETT, export/k_kipp0/kanal_zeit.csv), K2 stellt ein
FENSTERMITTEL (FK.rx) gegen einen MOMENTANWERT (FK.px/object_force), und `soll_rx = f*delta*A`
unterschlaegt den instationaeren Term rho*V*dU_b/dt. **K2 misst dort Nichtstationaritaet.**

**(2) Gekippte Wand: ein Drittel bis die Haelfte der Facetten ist TOT.** Beim ALPHA2-Downdate
ist G' fuer Zellen mit nur EINEM Wandlink analytisch exakt 0 => Slot-13-Return, reiner BB.
Slot 13 bestaetigt auf die Stelle: kipp45 = 78.566.400 = **exakt 50,0 %**, kipp26 =
53.195.580 = **exakt 33,3 %**, kipp0 = 0. Deckt sich mit den Feuerraten (aktiv/feuernd:
14160/14160 = 100 %, 31860/21240 = 66,7 %, 29760/14880 = 50,0 %).
Gegenprobe 45 Grad: (y+_Akku/y+_utau)^2 = (38,3/208,7)^2 = **0,0337** gegen gemessenes
Verhaeltnis **0,0336**. Der Akkumulator ist dort EHRLICH; falsch ist, was angewandt wird.
`eigene_links` wird in baue_facetten erhoben (setup.cpp:1170), aber NIE als Kriterium benutzt.

**(3) Vorzeichenkipp bei 26 Grad:** der gekoppelte Rang-2-Skalarpfad (kernel.cpp:1969, Slot 14
= 33,3 %) schleppt phi2 = P2 + Gt12*s1 mit, Groessenordnung 10^2 bis 10^3 mal twe, Vorzeichen
beliebig. Gegenprobe: bei 45 Grad folgt das Verhaeltnis der quadrierten y+-Ratio, bei 26 Grad
NICHT ((69,2/265,2)^2 = 0,068 gegen |1,049|) -- genau dort, wo Slot 14 besetzt ist.

## DER TIEFERE PHYSIKFEHLER DARUNTER (HOCH)
**Der iMEM-Pfad ignoriert die Stoerungsform der Ablage.** FluidX3D speichert f^ = f - w
(calculate_rho_u:1131 addiert rho += 1.0f; load_f:1403 entshiftet NICHT). `P1 = Sum_L 2*ct1*fhn[i]`
summiert nur ueber die SOLID-Teilmenge L -- dort hebt sich der Offset nicht auf. Der wahre
Austausch ist P + 2*(S1 . t^). FluidX3Ds eigenes MEA (update_force_field:3252) summiert ueber den
VOLLEN Satz und ist deshalb verschiebungsfrei; iMEM ist es nicht.
Ebene Wand: S1 = (0,0,1/6) parallel n => 2*(S1.t^) = 0 EXAKT -- deshalb ist kipp0 sauber.
Treppe: |2*S1.t^| bis rund 0,4 gegen twe rund 2,5e-5 -- **vier Groessenordnungen**.
Betroffen sind ZIEL und BUCHHALTUNG (angewandt wird twe - 2*B1, protokolliert wird twe).
=> Am FAHRZEUG ist jede Wand eine Treppe. Das ist der Befund mit der groessten Reichweite.

## DREI BEFUNDE, DIE UEBER K2 HINAUSGEHEN

**A. HOCH, ZU VERIFIZIEREN: Randbedingung koennte FULLWAY statt HALFWAY sein.**
Agent C hat die Slot-Arithmetik simuliert (nicht nur gelesen): an einer Fluidzelle neben Solid
sei `fhn[i+1](t)` identisch mit dem eigenen post-Kollisions-`fhn[i](t-2)` -- ZWEI Schritte,
waehrend Fluid-Fluid einen braucht. Der Kommentar kernel.cpp:1182 beruft sich auf Krueger S. 180
(Halfway). Folge bei w = 1,9999: (1-w)^k = (-1)^k ist UNGEDAEMPFT, der reflektierte Anteil kommt
nach genau 2 Schritten mit zweimal gekipptem Vorzeichen zurueck -> resonante Periode-2-Mode in
der ersten Wandlage, gedaempft NUR durch nu_t.
**STATUS: NICHT UEBERNOMMEN.** Das widerspricht dem Upstream-Entwurf und wuerde die halbe
Wandposition verschieben. Vor jeder Konsequenz unabhaengig nachzurechnen -- die Iron Rule
"Vorgaengerstand taugt fuer PORTIERFEHLER" ist hier der richtige Rahmen.

**B. MITTEL: Smagorinsky-Pi_neq ohne Guo-Korrektur (kernel.cpp:2269-2277).**
Richtig waere Pi_neq = Sum cc(f - f^eq) + 0,5*(u_a F_b + F_a u_b). Mit VOLUME_FORCE/FORCE_FIELD
ist die Scherrate -- und damit die GESAMTE Viskositaet, weil nu_0 praktisch 0 ist -- ueberall
dort verzerrt, wo die Volumenkraft nicht vernachlaessigbar ist.
=> **Im KANAL ist die Volumenkraft der Antrieb.** Die Delta-Reihe vom 24.08. (nu_t-Verhaeltnis
1,56/1,49/1,05) steht damit auf einer verzerrten Scherrate. Der TREND ist davon nicht
zwingend betroffen (die Verzerrung wirkt in allen drei Punkten), die ABSOLUTWERTE schon.

**C. MITTEL: die RHO_CLAMP-Zaehler wickeln.** Ungegatete `atomic_inc` auf uint
(kernel.cpp:2160/2177); bei 2e8 Zellen nach rund 21 Schritten. **Jede grosse Klemmzahl, die ich
in den letzten Tagen zitiert habe (5,4 Mio b8_kontrolle, 6,38 Mio w_anliegend), ist damit NICHT
quantitativ.** Meine Rechnung "0,00039 % der Zellaktualisierungen" vom 23.08. ist hinfaellig.
Zusaetzlich verletzt die Dichteklemme die MASSE (nicht den Impuls): Sum f_post = rho + w(rho_c - rho).

## WEITERE BEFUNDE (Auswahl, vollstaendig in den Agentenberichten)
- Fernfeld-Einlass im PRODUKTIONSFALL ueberbestimmt: `set_velocity_inlet_faces` steht nur in
  main_setup_fernfeld, nie in main_setup_fahrzeug_dd (setup.cpp:5359). Der Fix haengt am
  Diagnosezweig. Jeden Schritt eine Massendifferenz in die erste Fluidzelle dahinter.
- Rueckkopplung erzeugt eine GEMESSENE Massenquelle: `mdot_netto_rel` stabil bei -2,0 bis
  -2,2 % des Einstroms (export/b8_breit_n16/band_bilanz.csv), bis -6 % in export/r_hart_aus.
  Steht seit Wochen in der CSV, nie ausgewertet.
- Das NAHFELD hat ueberhaupt keine Massenbilanz -- alle fuenf Flaechen vorgeschrieben.
- `update_force_field` laesst den Bewegtwand-Term aus: am ebenen mitbewegten Boden fehlen
  u_lat/3 je Randzelle und Schritt. Betrifft jede Kraftbilanz mit TYPE_S und u != 0.
- Geschwindigkeitsklemme (kernel.cpp:2244) ist die einzige UNBEOBACHTETE Klemme -- greift sie,
  ist der Impuls nicht erhalten.
- Zero-Copy-`read_from_device` ist ein No-Op ohne Queue-Drain; sicher nur ueber
  `Memory_Container`. Auf der iGPU (lbm_c) nach run_async liest der Host unsynchronisiert.
- `fac_tau` akkumuliert float32 ueber die ganze Laufzeit ohne Reset und ohne Praezisionswaechter.
- SPONGE hat KEINEN feuernden Zaehler.
- y+ im Kanal rechnet y_w hartkodiert 0,5 (setup.cpp:1680), der dd-Pfad warnt ausdruecklich
  davor. Die gemeldeten 69,2 / 38,3 sind damit um Faktor 1,4 bzw. 2,1 zu klein.

## HEIKOS FACETTENEBENE -- BEANTWORTET
Die Stuetzpunkte sind Halfway-Punkte geschnittener Links (setup.cpp:1161), die Ebene ist die
Ausgleichsebene durch deren Schwerpunkt. Nachgerechnet fuer 45 Grad: y_w = 0,369 (Randzelle) und
1,040 (Zweitlage) -- die Logs melden genau diese Werte. **Die Ebene liegt also BEREITS auf der
senkrechten Mittelebene zwischen den aeusseren Solidzellmitten und den ersten Fluidzellmitten**,
mit unter 0,02 Zellweiten Restversatz. Heikos Lage (Ebene DURCH die Solidzellmitten) verschoebe
die Wand um 0,354 (45 Grad) bis 0,5 Zellweiten (eben) ins Solid: der Kanal-Anker braeche (dort
ist die BB-Wand beweisbar bei 0,5), Modellwand und reflektierende Wand laegen 0,5 Zellen
auseinander, y_w stiege, tau_w fiele um rund 15 % -- **K2 wuerde schlechter**.
=> Heutige Lage ist richtig. Die Aufdickung kommt aus der Treppenamplitude +-0,354 und den
Zweitlagen-Facetten bei y_w rund 1,04. Der wirksame Hebel ist ein q-abhaengiger Wandabstand
je Link, also ELIBB.

## MEINE EIGENEN FEHLER IN DIESER RUNDE
1. Vorzeichen-Hypothese zu `fac_paar` (kernel.cpp:1726-1733) -- GEGENSTANDSLOS: unter
   CFD_FACETTEN=3 gilt FACETTEN_IMEM, und kernel.cpp:2130 kompiliert `fac_paar` WEG. Der Code,
   auf den ich gezeigt habe, laeuft in diesen Laeufen nicht. Die gespiegelten Indexpaare sind
   zudem nachgerechnet korrekt (alle 12 Aufrufe teilen dieselbe Invariante).
2. Flaechen-Hypothese fuer den 45-Grad-Fall -- WIDERLEGT: `fq*delta*2*Nx*Ny` ist in Wahrheit
   eine VOLUMENbilanz (f*V_fluid), die benetzte Flaeche geht gar nicht ein. Das Etikett
   "Soll f*delta*Flaeche" ist irrefuehrend.
3. Klemmzahlen als Argument benutzt, obwohl die Zaehler wickeln (siehe C).

---

# 2026-08-25 — AUDIT-KORREKTURSCHLEIFE, WELLE 1 ABGEARBEITET

Auftrag Heiko: "wie immer alle fehler direkt beheben und durch unabhängige agenten
nochmal prüfen lassen ... wenn du damit durch bist nochmal die große code audit
korrektur schleife". Commits b608487 und c69d18a.

## Behoben (zehn Befunde)

| # | Befund | Korrektur | Datei |
|---|--------|-----------|-------|
| 1 | RHO_CLAMP-Zaehler ungegatet, uint wickelt nach ~21 Schritten | `t%100`-Gatung, beide Zweige | kernel.cpp |
| 2 | Kanal-y+ hartkodiert y_w=0,5, obwohl der dd-Pfad davor warnt | je Facette aus `fac_geo[8i+3]` | setup.cpp |
| 3 | Geschwindigkeitsklemme als EINZIGE Klemme unbeobachtet | Slot 28, beide Zweige | kernel.cpp |
| 4 | SPONGE ohne jeden feuernden Zaehler (HARTER FEHLER nach Iron Rule) | Slot 29, Klemme zaehlt doppelt | kernel.cpp |
| 5 | `object_force` nichtdeterministisch (atomic_add_f je Arbeitsgruppe) | Grid-Stride + `object_force_final`, feste Summenreihenfolge | kernel.cpp/lbm.cpp |
| 6 | Zero-Copy-`read_from_device` ist ein No-Op OHNE Queue-Drain | `cl_queue.finish()` bei blocking | opencl.hpp |
| 7 | `fac_tau` float32 ohne Praezisionswaechter | Kennzahl Akkumulator/Zuwachs, 2^20 / 2^24 | setup.cpp |
| 8 | Fernfeld-Einlass im Produktionsfall ueberbestimmt | `CFD_FERN_VI` dort verfuegbar + angesagt | setup.cpp |
| 9 | Bewegtwand-Term im Kraftfeld fehlt | `-6 w_i (c_i.u_w) c_i` ueber die zurueckgeworfenen Links, Slot 59 | kernel.cpp |
| 10 | Smagorinsky-Pi^neq ohne Guo-Korrektur | `CFD_SGS_GUO` (Default 1), Slots 60..63 | kernel.cpp |

Befund 6 ist der folgenreichste: `is_zero_copy` verlangt `uses_ram`, und das gilt fuer
CPU **und iGPU** -- also fuer die GESAMTE Fernfeld-Domaene. Beim normalen Puffer wartet ein
blockierendes `enqueueReadBuffer` in der In-Order-Queue auf alle eingereihten Kernel; bei
Zero-Copy tat der Aufruf GAR NICHTS. Der Host las den Speicher, waehrend die GPU noch
hineinschrieb. Jede Fernfeld-Diagnosezahl war damit potenziell halb geschrieben.

## Zwei neue Mechanismen, beide mit Wirkpfad-Zaehler

- **`CFD_FAC_ALPHA=3` (A2-Rueckfall, Slot 64).** Das Rang-1-Downdate
  `G' = 6 Sum w (c-cq)(c-cq)^T` mit `cq = S1/S0` vernichtet die Einzellink-Facette
  ANALYTISCH: mit einem Link ist `cq = c_1`, also `c-cq = 0` und `G' == 0`. Solche Facetten
  fielen in den Slot-13-Return und blieben REINES Bounce-Back, ohne jedes Wandmodell.
  Gemessen: 50,0 % aller Facetten bei 45 Grad, 33,3 % bei 26 Grad, 0 % eben -- genau die
  Reihenfolge der K2-Abweichung. Kein Programmierfehler, sondern eine echte Entartung: mit
  einem einzigen Link ist jede Impulsinjektion eine reine Gleichverschiebung, es bleibt kein
  deviatorischer Freiheitsgrad fuer die alpha-Nebenbedingung. Der Rueckfall waehlt bewusst:
  lieber die Wandschubspannung anwenden und die alpha-Nullung fuer diese Facetten aufgeben.
  Der Preis laeuft messbar mit (fac_tau[4] Delta-m, fac_tau[5] Normalkontamination).
- **`CFD_FAC_LSQ` (Default 1, Slot 65).** Der alte Skalar-Rueckfall `s1 = R1/G11` erzwingt
  das Ziel in Richtung 1 exakt und ignoriert die zweite Gleichung ganz. Ist G11 fast entartet,
  wird s1 riesig, und weil G12 dabei nicht klein sein muss, schleppt die Loesung
  `Phi2 = G12*s1` mit. Kleinste Quadrate auf dem erreichbaren Unterraum:
  `s1 = (G11*R1 + G12*R2)/(G11^2 + G12^2)`. Bei G12 = 0 wortgleich der alte Zweig; bei
  G11 -> 0 laeuft sie gegen R2/G12 statt gegen unendlich.

## WIDERLEGT: meine eigene Stoerform-Hypothese

Behauptung vom 24./25.08.: "iMEM summiert Momente ueber die Solid-Teilmenge, waehrend
FluidX3D f^ = f - w speichert; an Treppen bis zu vier Groessenordnungen ueber tau_w."

GEMESSEN (Histogramm-Slots 49..53, |2*(S1.t)| gegen |def_fac_tau*twe|):
- ebene Wand: **100,0 % unter 1 %** des Ziels (Theorie: symmetrische Linkmenge, Offset
  hebt sich exakt weg -- bestaetigt)
- 45 Grad: **95,7 % unter 1 %, 4,2 % zwischen 1 und 10 %, nichts darueber**

Die Groessenordnungs-Behauptung ist damit falsifiziert. Der Offset ist an der Treppe klein.

Was die Messung stattdessen zeigt: **|P|/Ziel ist bei 45 Grad exakt BIMODAL** --
50,0 % unter 1 %, 50,0 % ueber 10 %, nichts dazwischen. Das ist die Signatur der tot
gelegten Einzellink-Population, nicht die eines Offsets.

## K2 an der EBENEN Wand ist kein Programmierfehler

Die Aufloesungsreihe lag seit Tagen in den Logs, ohne dass ich sie in dieser Richtung
gelesen haette. kn_20 / kn_38 / kn_76, alle uebrigen Parameter gleich:

| N | y+_1 | f*delta | tau_w modelliert | Modell/Soll | Angewandt/Modell | K2 |
|---|------|---------|------------------|-------------|------------------|-----|
| 20 | 259,3 | 4,3889e-6 | 4,39e-6 | **1,0002** | 1,0040 | 1,0043 |
| 38 | 136,5 | 4,0113e-6 | 4,09e-6 | **1,0196** | 1,0055 | 1,0253 |
| 76 |  68,2 | 4,1876e-6 | 4,34e-6 | **1,0364** | 1,0014 | 1,0378 |

Der Anwendungspfad liegt flach bei 0,1-0,6 % OHNE Trend. Der Ueberschuss sitzt vollstaendig
im MODELLIERTEN tau_w, und er waechst monoton, waehrend die erste Zelle aus der Log-Schicht
faellt (y+ 259 -> 137 -> 68). Das ist Log-Layer-Mismatch des Wandmodells -- eine
Modelleigenschaft, kein Fehler im Facettenpfad. Die K2-Schwelle von 1 % misst an der ebenen
Wand also die Genauigkeit des Wandmodells, nicht die Richtigkeit des Codes.

Folge fuer die Abnahme: die 1-%-Schwelle ist an der ebenen Wand nur bei grobem y+ erfuellbar.
Sie gehoert entweder auf den Anwendungspfad (Angewandt/Modell) umgestellt oder
aufloesungsabhaengig gefasst.

## WIDERLEGT: Fullway-Folgerung (unabhaengiger Pruefagent)

Die Schrittzahl ist bestaetigt -- der Slot einer Wand wird nur alle ZWEI Schritte angefasst,
die Zeitrechnung ist exakt Fullway (bitgleich gegen eine Lehrbuch-Fullway-Referenz in jedem
Anlaufschritt). Die daraus gezogene Folgerung ist WIDERLEGT: die gemessene Wandlage ist fuer
alle drei Schemata identisch, y_w(tau) reproduziert Ginzburg/d'Humieres (0,5206 / 0,5184 /
0,5108 / **0,5000 bei tau=0,9333** / 0,4665). y_w, tau_w und jede Kraftbilanz bleiben
unberuehrt; die Abweichung wirkt nur instationaer.

## PRUEFUNG DER WELLE 1 DURCH UNABHAENGIGE AGENTEN -- vier eigene Eingriffe zurueckgenommen

Zwei Prüfagenten (Kernel-Physik, Host/Bindung) gegen den Diff b608487~1..c69d18a.
Sie haben mehr gefunden, als die Welle 1 behoben hat.

### K.O.-BEFUND: der A2-Rueckfall war BEWEISBAR WIRKUNGSLOS und haette die Messung verfaelscht

Angewandt wird in Pass 2 (kernel.cpp)
```
q_i = 6 w_i (c_i . u_s) + w_i * alpha ,   alpha = -6 (S1 . u_s) / S0
    = 6 w_i (c_i - c_q) . u_s            mit c_q = S1/S0
```
Fuer EINEN Link ist c_q = c_1, also **q_i identisch null, fuer jedes u_s**. `G' = 0` war damit
keine numerische Entartung, die man reparieren kann, sondern die WAHRE Aussage ueber den unter der
Massen-Nebenbedingung erreichbaren Unterraum. Die Praemisse meines Eingriffs ("sie blieben reines
Bounce-Back, als sei das vermeidbar") war falsch -- sie bleiben es, weil alpha sie dazu zwingt.

Schlimmer als wirkungslos: mit zurueckgesetzten Rohmomenten haette
`phi1 = P1 + G11roh*s1` eine Wandkraft in `fac_tau[1..3]` gebucht, die NICHT angewandt wurde.
K2 waere besser geworden, ohne dass sich am Feld etwas aendert -- genau die Selbsttaeuschung,
gegen die die Abnahme gebaut ist.

EMPIRISCH BESTAETIGT: `ab_45_kontrolle` und `ab_45_a2fall` sind in jeder gedruckten Zahl gleich.
Slot 13 steht in dieser Konfiguration ausserdem bei **7**, nicht bei 50 % -- meine Zahl "50,0 %
aller Facetten bei 45 Grad" stammte aus einer anderen Konfiguration und trug den Eingriff nicht.
Damit faellt auch meine Deutung der |P|-Bimodalitaet ("die Signatur der tot gelegten
Einzellink-Population") -- sie ist unbelegt.

**ZURUECKGENOMMEN.** Der Freiheitsgrad muss aus der GEOMETRIE kommen (ELIBB, q-gewichteter
Wandabstand), nicht aus dem Zuruecknehmen einer Identitaet.

### Der LSQ-Rueckfall ist eine MODELLAENDERUNG, kein Numerikfix -> Default AUS

Die Formel ist korrekt (beide Zweige, auch im Schur-Komplement; die Normal-Nullung bleibt exakt,
weil sn aus den ANGEWANDTEN s gebildet wird). Aber:
- LSQ gewichtet t1 (Stroemungsrichtung, Ziel = Spalding-tau_w, die eigentliche Messgroesse) und
  t2 (Ziel 0, eine blosse Modellannahme) GLEICH. Fuer ein Wandmodell die falsche Gewichtung.
- Sie bricht die SATGATE-Invariante "iMEM wirkt nur, wenn es sein Ziel im Budget EXAKT erreichen
  kann" -- LSQ erreicht es prinzipiell nie exakt, das Gate laesst sie trotzdem durch.
- Der eingehandelte Fehler in Stroemungsrichtung ist exakt
  `phi1 - Ziel = G12 (G11 R2 - G12 R1)/(G11^2 + G12^2)`; der alte Zweig traf phi1 exakt.
- Die Divergenz ist NICHT beseitigt: bei G12 = 0 und G11 -> 1e-8 liefert LSQ dasselbe 1e8*R1.

Default AUS, eigener Messarm, eigene Begruendung.

### NEU statt LSQ: QUERGATE (CFD_FAC_QUERGATE, Default AUS, Slot 64)

Das eigentliche Problem -- Phi2 = G12*s1 bis 10^3 mal twe -- wird jetzt in der Doktrin behandelt,
die dafuer schon existiert: uebersteigt der Restfehler in Querrichtung die Wandschubspannung, die
ueberhaupt aufgepraegt werden soll, wird BB belassen statt einen Querimpuls einzuschleppen. Ein
exakter 2x2- oder Schur-Solve hat res2 = 0 und passiert immer.

### Der Bewegtwand-Term war nur zu einem FUENFTEL richtig

`calculate_rho_u` summiert ueber ALLE 19 Richtungen, auch ueber Links, deren Streaming-Ursprung
selbst Solid ist. Fuer die fuehrt niemand je `store_f` aus -- der Slot behaelt den Wert aus
`initialize()`. Bei ruhender Wand ist das feq(1,0), in Stoerform exakt 0; deshalb ist es nie
aufgefallen. Bei MITBEWEGTER Wand ist es feq(1,u_w) und nicht null: an der ebenen Fahrbahn sind
13 der 18 Links tot und tragen **+-5*u_w/3, mit der Paritaet des Zeitschritts oszillierend** --
drei Groessenordnungen ueber tau_w. Meine Fassung addierte nur den fehlenden Term (-u_w/3) und
liess vier Fuenftel plus die Oszillation stehen. Jetzt laufen BEIDE Summanden ueber DIESELBE
Linkmenge. Auch die Commit-Formulierung war ungenau: rho_w ist per Konstruktion 1, und das
Vorzeichen ist negativ.

### Weitere behobene Pruefbefunde

- **X-1 (HOCH):** mein SPONGE-Waechter haette den dd-Produktionslauf per `print_error` -> `exit(1)`
  abgebrochen, VOR der Cd/Cz-Ausgabe. `s_sponge_n` ist eine STATIK und traegt beim Nahfeld-Bericht
  den Fernfeld-Wert. Jetzt Ansage statt Abbruch.
- **X-2 (HOCH):** mein fac_tau-Praezisionswaechter nahm das ROHE MAXIMUM ueber 3*fac_N Verhaeltnisse
  und rief `print_error`. Eine einzige nahezu stationaere Facette mit Fensterdelta in ULP-Groesse
  liefert 1e9. **Er hat genau das getan: `ab_45_kontrolle` und `ab_45_a2fall` sind daran gestorben**
  (Akkumulator/Zuwachs = 2,4e10). Jetzt beitragsgewichtet und niemals fatal.
- **X-3 (HOCH):** durch die Gatung wurde aus "NULL Treffer -- die Klemme hat nie gegriffen" eine
  falsche Unbedenklichkeitsbescheinigung. Text sagt jetzt "Stichprobe".
- **3-A/3-B:** der Guo-Zaehler feuerte je ZELLE je 100. Schritt -- bei 2e8 Zellen wickelt uint nach
  rund 2100 Zeitschritten, also genau der Fehler, den Welle 1 bei Slot 0/1 behoben hat. Ebenso das
  Stoerform-Histogramm bei fac_N ~ 1e6. Beide jetzt zusaetzlich hash-ausgeduennt (jede 64.).
- **3-D:** der Wickelwaechter lief ab Slot 28 und meldete Geschwindigkeitsklemme und SPONGE als
  "nu_t-Bin". Jetzt ab 30, plus ein neuer Waechter ueber ALLE 66 Slots, unabhaengig von CFD_SGS_DIAG.
- **2-b:** `read_from_device_1d/2d/3d` hatten den Zero-Copy-Zweig nicht bekommen; `schreibe_wandprofil`
  las auf der iGPU weiter unsynchronisiert.
- **4-a:** `s_fac_lsq`/`s_sgs_guo` fehlten in mehreren Setup-Bloecken -- lautlose Schalter mit
  Default-Wirkung. Jetzt an allen Stellen gesetzt.
- **4-c:** die neuen Wirkpfad-Slots wurden nur im Kanal berichtet, nicht in Kugel/dd.
- **1-b:** Grid-Stride mit `uxx` haette im 65536-Fenster unter der uxx-Grenze zur Endlosschleife.
- **1-d:** SGS_GUO wurde unabhaengig von VOLUME_FORCE emittiert (heute unerreichbar, Falle bleibt).
- **3-E:** die Slot-Legende widersprach sich selbst ("[28] Geschwindigkeitsklemme" UND
  "[28..32] nu_t-Histogramm"). Sie steht jetzt an EINER Stelle.

### Bestaetigt statt widerlegt

- Guo-Vorfaktor, Vorzeichen, Normierung, `uxn` als physikalische Geschwindigkeit: **korrekt**,
  hergeleitet aus dem tatsaechlich implementierten Kraftterm (nicht aus dem Lehrbuch): das zweite
  Moment von `Fin` ist `u_aF_b+u_bF_a`, `(1-1/(2tau))` sitzt erst in der Kollision, also gehoert
  `+1/2` und kein weiterer Faktor dazu.
- Der Prueferwert fuer die Groessenordnung (rel. Aenderung von |Pi| bei y+ = 137: **7,1e-6**) und
  die Prognose "100 % im <0,1-%-Bin" wurden von der Messung `ab_eben_guo` **bestaetigt**.
- object_force-Umbau: Gittergroesse, Abdeckung, unbedingtes Schreiben, serielle Endsumme,
  In-Order-Reihenfolge und Parameterbindung von `update_force_field` -- alle korrekt.
  Einschraenkung zur Commit-Meldung: `atomic_add_f` ist NICHT tot, `object_center_of_mass` und
  `object_torque` benutzen es weiter (werden aber von keinem Setup aufgerufen).

## ELIBB-PLANUNGSAGENT (2026-08-25): ELIBB heilt 26 Grad, NICHT die 45-Grad-Klasse

Zentrale, bewiesene Aussage: fuer die Slot-13-Klasse der 45-Grad-Treppe (ein Link, c || n)
liefert JEDE Ein-Knoten-Interpolation der Marson-Familie nur eine Verschiebung der
NORMALreflexion. Alle Operanden leben auf der Link-Achse {c, -c}; c.u_s = (c.n)(n.u_s),
die Tangentialkomponente faellt exakt heraus -- fuer jedes a1/a2/a3. Der erreichbare
Unterraum eines Links ist span{c}, und bei c || n enthaelt er keine Tangente. Deckt sich
mit der V1-Vollextraktion ("ELIBB is purely geometric ... no wall shear stress") und mit
der heute bewiesenen alpha-Identitaet.

Scharfe Winkeltrennung:
- 26,6 Grad (m2-Klasse): |c.t2| = 1/sqrt(5) != 0 -- ELIBB mit u_W=u_s erzeugt dort ECHTEN
  Tangentialaustausch. Gleiches gilt fuer 97 % der ohneTang-Klasse am Fahrzeug, 94 % Kugel.
- 45 Grad Lage 1: bleibt beweisbar BB. Fuer diese Klasse braucht es einen ZELL-Kraftterm
  (tau_w*A/V als Guo-Volumenkraft) oder die Delegation an die Lage-0-Facetten (deren
  faca = 1/|n_a| das tau-Integral der wahren Flaeche bereits deckt).

WIDERSPRUCH IM WISSENSSPEICHER, benannt statt still entschieden: FACETTEN-ELIBB-PLAN.md
sagt "Eq. 25e verdruckt, a2=(1-q)/q"; die V1-Unterlagen (knowledge/wall-models.md,
recherche-archive.md, Stand 05.08.2026) sagen das Gegenteil (Sylvester-Lagrange, kein
Druckfehler; (1-q)/q passt auf keinen gedruckten Zweig). Operativ entschaerft: die
V1-Vorlage nutzt die eigene PoU-Blende (q=0,5 => exakt HWBB), und das Gate "q=0,5
bitgleich, sonst Stopp" entscheidet die Koeffizientenfrage empirisch. Das Gate ist
damit NICHT optional.

Minimalpfad (B0..B4, je Baustein eigene Abnahme): B0 EsoPull-Harness (CPU, klaert
Zeitindex und i<->ib-q-Indizierung); B1 fac_q-Hostpfad (18 x uchar je Facette, q-Boden
mit Zaehler, Kugel-RMS-Gate 0,143, kipp0 alle q im 0,5-Bin); B2 Kernelblock
FACETTEN_ELIBB (ersetzt q_i UND alpha, nie stapeln; aus = byte-identisch; q=0,5-Gate);
B3 Kraftbuchhaltung (fac_tau_n muss auch ELIBB-Facetten zaehlen, sonst killt der
K3-print_error jeden Kipp-Lauf); B4 Messleiter kipp0 -> Kugel -> kipp26 (ERSTES
K2-Ziel, mit A/A-Rauschband) -> kipp45 (FALSIFIKATIONSARM: Vorhersage Lage-1 bleibt BB).

Erwartung ehrlich: Geometriekorrektur, kein Cz-Hebel erster Ordnung (V1: Treppe trug
2 von 20 Prozentpunkten). Erst-Erfolg ist kipp26-K2 ueber dem A/A-Band plus Kugel-Cd
Richtung Achenbach-Band.

## LITERATURAGENT (2026-08-25): das Problem ist publiziert -- in der iMEM-Originalarbeit selbst

Asmuth et al. 2021 (Phys. Fluids 33, 105111 -- die Basis unseres Facettenpfads) nennen als
Loesbarkeitsbedingung woertlich: das System fuer u_w bleibt nur bestimmt, "as long as F^(u_w)
can be constructed as a linear combination of e_ijk, ijk in C" (C = wandschneidende Links).
Unsere Messung (3,4 % / 0,7 % der Sollreibung bei 45/26 Grad) ist die quantitative Konsequenz
dieser publizierten Bedingung -- KEIN Implementierungsfehler. Die explizite iMEM-Loesung ist
im Paper nur fuer die EBENE, gitterparallele Wand angegeben (Gl. 28, Vorfaktoren 3/3/1 --
genau die Querkopplung, die bei 26,57 Grad unloesbar wird).

Die Literatur hat das Problem NIE linkbasiert geloest, sondern umgangen -- vier Wege:
1. VOLLREKONSTRUKTION der Wandzell-Populationen (Malaspinas/Sagaut JCP 275 (2014);
   Maeyama et al. CAMWA 93 (2021) + C&F 233 (2022): Image Point + Wandfunktion auf
   non-body-fitted VOXELGITTERN, validiert bis 30P30N-Hochauftrieb -- die publizierte
   Referenzloesung fuer exakt unseren Fall). Architektur-kompatibel (nur Makrofeld-Reads).
2. EXPLIZITER KRAFTTERM in den Wandzellen (Kuwata & Suga JCP 433 (2021) IVW: Koerperkraft
   = Soll- minus MEM-Ist-Scherkraft; Wang et al. PoF 36 (2024) IB-WMLES; Kummerlaender et
   al. arXiv:2510.13726 homogenisierte LBM mit Spalding auf dem Geschwindigkeitsmoment).
   Richtungsfrei, massenerhaltend, minimal-invasiv ueber das vorhandene Guo-Forcing --
   BESTER FIT fuer Esoteric-Pull. Preis: erste Ordnung an der Grenzflaeche, dokumentierte
   leichte u-Unterschaetzung (IVW).
3. SURFEL-VOLUMETRIK (PowerFLOW-Schule, Chen PRE 58 (1998), NASA/CR-2000-210550):
   C_f-gesteuerte Mischung Bounce-Back/Spiegelung an der WAHREN Facette, Impuls je Surfel
   statt je Link. Loest das Problem per Konstruktion, passt aber NICHT auf unsere
   Architektur (Gather/Scatter, Nachbar-Schreibzugriffe).
4. WAHRE FLAECHENGEOMETRIE je Link (Bouzidi/ELI): vergroessert die Linkmenge, zweite
   Ordnung -- aber kein Wandmodell; allein keine Loesung.

KONVERGENZBEFUND (Einschaetzung des Agenten, konsistent mit unserer Messung): fuer rein
linkbasierte Tangentialaufpraegung konvergiert die Treppenreibung NICHT gegen den glatten
Grenzwert -- die Treppe ist selbstaehnlich, der Anteil degenerierter Zellklassen ist
aufloesungsUNabhaengig, das Defizit ist O(1). Fuer das Fahrzeug heisst das: 3,5 mm statt
4 mm aendert an diesem Fehleranteil NICHTS. Eine systematische Konvergenzstudie
"WMLES-Reibung auf Voxeltreppe" existiert nicht (Literaturluecke).

Diagnose-Nebenbefund (Stahl/Chopard/Latt C&F 39 (2010); Matyka et al. C&F 73 (2013)):
WSS unmittelbar an der Treppe ist stark verfaelscht, wenige Knoten entfernt brauchbar --
Auslese versetzt messen. Fehler winkelabhaengig, am kleinsten bei tan(a)=0 und tan(a)=1.

KONVERGENZ DER BEIDEN AGENTEN: der ELIBB-Analyseagent (Beweis: Einzellink || n hat keinen
tangentialen Unterraum) und die Literatur (Asmuths eigene Spann-Bedingung; alle Loesungen
umgehen die Linkverteilung) sagen DASSELBE. Der Hybrid -- iMEM behalten, wo die Linkmenge
traegt, plus Zell-Kraftterm (IVW-artig) nur fuer die degenerierten Klassen -- hat kein
direktes Literaturvorbild, ist aber die architekturkonforme Synthese beider Berichte.
Volltexte liegen unter tool-results/ (asmuth2021.txt, han_wfb.txt, matyka.txt, icase.txt).

## GEOMETRIE-AGENT (2026-08-25): Kombinatorik EXAKT bestaetigt -- und die 45-Grad-Diagnose gedreht

Alle Zaehlerstaende analytisch reproduziert (78.566.400 und 53.195.580 exakt; Skripte in
scratchpad/teil1..4_*.py). Praezisierung: die Wand kippt in der y-z-Ebene, Stroemung x --
STREAMWISE-Impuls quert die Wand NUR ueber Links mit c_x != 0, und die gehoeren zu 100 %
den Stufenklassen. Klassen: 45 Grad m=4 (7 Links, voll loesbar) / m=5 (1 Link || n, Slot 13);
26 Grad m=6 (8 Links, voll) / m=7 (4 Links, Rang-2: Gt22 = 0 EXAKT, Querrest = |P2|) /
m=8 (1 Link, Slot 13).

DIE WICHTIGE DREHUNG (45 Grad): die loesbare m=4-Klasse traegt die wahre Wandflaeche
BEREITS EXAKT (Sum faca = 1,0000 * A_wahr; die tote Klasse ist flaechenREDUNDANT), und das
noetige tau_w waere nur 1,08 * u_tau_ist^2 -- kein Autoritaets-, kein Flaechen-, kein
Klemmproblem (tw_max hat Faktor 249 Spielraum). Das Defizit (Faktor 29,4 = 1/K2) sitzt im
ZUSTAND: das wandnahe u_t liegt bei ~14 % des Log-Law-Gleichgewichts -- die Treppe wirkt
trotz iMEM als RAUE Wand (Re_tau IST 7930 statt 5186), die Impulsbilanz schliesst ueber
ungebuchte BB-Spitzen der SATGATE-Rueckfaelle (0,38 % der Gates mit |P| = 10-100 twe) am
Buchungspfad vorbei. Spalding am zu langsamen u_t liefert tau/27.

26 Grad, quantitativ: m6 allein muesste Faktor 2,00 der wahren Wandschubspannung melden --
liefert kein Gleichgewichts-Wandmodell. Der STUFEN-CLUSTER m6+m7 stellt vollen Rang her
(Gt22: 0 -> 1,11) UND traegt zusammen exakt A_wahr -- er adressiert beide Befunde zugleich.
ELIBB heilt an der 26-Grad-Einzellinkklasse die t2-Behandlung; x-Autoritaet der toten
Zellen braeuchte q = 1,2-1,5 (Zweitschale) = de facto wieder der Cluster. Solid-Facette:
keine zusaetzliche Autoritaet, kostet Scatter -- verworfen.

EMPFEHLUNG DES AGENTEN: tote, flaechenredundante Facetten (m5/m8) DEKLARIERT deaktivieren
statt heilen; 26 Grad ueber den Cluster; 45 Grad ist danach ehrlich ein Zustands-/
Buchungsproblem (Treppenrauigkeit), kein Loeserproblem.

## EBENE-WAND-AGENT, ABSCHLUSS (2026-08-25): K2-Ebene ist eine FP32-BULK-IMPULSQUELLE

Mit der frischen, GEFENSTERTEN Serie (k2_n20/38/76) und fuenf Kontrollarmen:

| Lauf | Modell/Soll | Angewandt/Modell | K2 | K2 aus Impulsbilanz | b je Zelle/Schritt |
|---|---|---|---|---|---|
| n20 | 1,0046 | 0,9997 | 1,0043 | 0,9937 | 4,64e-9 |
| n38 | 1,0257 | 0,9996 | 1,0253 | 1,0024 | 4,82e-9 |
| n76 | 1,0387 | 0,9992 | 1,0378 | 0,9967 | 4,53e-9 |

1. Der Anwendungspfad ist SAUBER (0,9992-0,9997; das kleine Minus ist die Projektion der
   Querfluktuationen, quantitativ erklaert). Der Modellterm traegt formal den Ueberschuss --
   ABER er ist im stationaeren Kanal von der Impulsbilanz DIKTIERT, nicht vom Wandgesetz:
   der 2x2-Solve ist exakt, also folgt <tw> der Bilanz zwangslaeufig.
2. Die exakt nachgerechnete Impulsbilanz (fq-Rekonstruktion auf 6 Stellen, Speicherterm aus
   den Fensterendpunkten) erklaert K2 NICHT. Es fehlt eine Impulsquelle
   **b = 4,5-4,8e-9 je Zelle und Schritt, KONSTANT ueber N=20/38/76, ueber zwei
   Fensterlaengen (80/160 ETT) und zwei Code-Arme (SATGATE an/aus)** -- Signatur einer
   FP32-Drift des BULK-Updates (Build: -cl-finite-math-only -cl-mad-enable). Das Wandmodell
   extrahiert sie ehrlich zusaetzlich zu f*V; K2 waechst damit LINEAR mit N
   (b*delta/tau_w reproduziert 0,4 -> 2,5 -> 3,8 % quantitativ).
3. Alle drei Ausgangshypothesen als K2-Ursache WIDERLEGT (nachgerechnet): Jensen-Effekt
   +0,13-0,31 % und bilanzneutral; Abtastpunkt ist ein grosser MODELLGUETE-Befund
   (Kernel sieht 0,698-0,707 * u1 -- BB-Theorie sagt exakt 2/3! -- die erste Lage laeuft bei
   u+ ~ 26 gegen Spalding-18), aber K2-blind; Fensterregelung +-0,02-0,13 %.
4. KONSEQUENZ FUERS GATE: das K2-1-%-Gate scheitert bei N>=38 SYSTEMATISCH an b, obwohl der
   Facettenpfad sauber ist. Redefinition noetig: K2 gegen die bilanzkorrigierte Quelle, oder
   das vorgeschlagene Chunk-Impuls-Audit einbauen (je Chunk b_chunk drucken; Bulk-Drift ->
   +4,7e-9 zeitkonstant auch im Warmlauf). Lokalisierungs-A/B: identischer Lauf ohne
   -cl-mad-enable (opencl.hpp:317) -- verschwindet b, ist es die FMA-Kontraktion.
5. NEBENBEFUND MODELLGUETE (gross, getrennt von K2): der iMEM-Abtastpunkt VOR der Korrektur
   sieht systematisch 2/3 des wahren u der ersten Lage (Wandlinks tragen die No-Slip-
   Reflexion, P1 ~ -u/3). Das erklaert einen Teil des u_tau-Defizits (IST/Ziel 0,67-0,73)
   NEBEN der Treppenrauigkeit. Ein 3/2-Korrekturfaktor auf das abgetastete u_t waere die
   billigste Massnahme -- gehoert als eigener Messarm in die Leiter, NICHT still eingebaut.

## ELIBB-MESSLEITER g1 (2026-08-25 nachmittags): B2-BLENDE FALSIFIZIERT -- Kugel-Gate schlaegt hart fehl

Sechs Laeufe, je exakt eine Variable (CFD_FAC_ELIBB), Binary 0ab98fd + Folgecommit:

| Arm | aus | an | Urteil |
|---|---|---|---|
| kipp0 (Gate) | Hash 4722579264326613690 | Hash identisch | **BESTANDEN (bitgleich)** |
| kipp26 u_tau IST/Ziel | 1,943 | **2,747** | schlechter |
| kipp26 K2 | -1,0488 | **-1,6001** | schlechter |
| kipp45 u_tau IST/Ziel | 1,529 | **1,698** | schlechter |
| kipp45 K2 | 0,0336 | 0,0139 | schlechter |
| Kugel Cd (nominal) | +10,5744 | **-4,9498** | **PHYSIKALISCH UNMOEGLICH** |
| Kugel Delta-m | +273,7 | **-52,6** | Vorzeichenwechsel |

Das NEGATIVE Cd der Kugel ist der harte Beweis: die Blende INJIZIERT Impuls ins Feld --
das ist kein Buchungs-, sondern ein Feldbefund (object_force). Beide Kippwinkel bestaetigen
die Richtung: die Wand wird RAUER statt glatter (Re_tau IST steigt), die Zustandsprognose
des Entscheids ist widerlegt. Die Gates feuern massiv haeufiger (u_s-Klemme/Gate 26 Grad:
45,2 Mio gegen 21,1 Mio; 45 Grad: 4,05 Mio gegen 0,30 Mio) -- der Solve kaempft gegen die
Blende. K3 (Druck exakt 0) besteht in allen Armen; kipp0-Bitgleichheit besteht -- der
Defekt sitzt AUSSCHLIESSLICH im q!=0,5-Zweig der Blende.

WICHTIGE PRAEZEDENZ AUS V1 (knowledge/wall-models.md, 05.08.2026): die gedruckte
Marson-Eq-25-Form erzeugte bei q=0,5 Spiegelreflexion mit **Cd_v = -1,84** -- dieselbe
Signaturklasse (negativer Widerstand = Free-Slip/Injektions-Pathologie). Der damalige
Befund "a2=(1-q)/q passt auf keinen gedruckten Zweig" und der heutige Kugelbefund
gehoeren vermutlich zur selben Wurzel: die NEBB-/Blenden-OPERANDENWAHL ist falsch, nicht
die Idee der q-Interpolation.

HARNESS-LUECKE benannt: der B0-Harness prueft Bitgleichheit bei q=0,5 und Massendrift --
er prueft NICHT das VORZEICHEN des Impulsaustauschs der Blende bei q!=0,5. Genau dort
sitzt der Defekt. Erweiterung noetig: analytischer Poiseuille-/Couette-Minifall im
Harness, Impulsaustausch der Blende gegen die BB-Referenz, Vorzeichen und Betrag.

STAND DER LEITER: Stufe 1 (kipp0) bestanden, Stufe 2 (Kugel) NICHT bestanden -->
LEITER GESTOPPT, kein Fahrzeuglauf. Naechste Schritte (Reihenfolge):
1. Harness-Erweiterung Impulsaustausch (CPU) -- Operandenwahl der Blende dort
   durchpermutieren (bb/nebb-Zweige, Vorzeichen von f_neq, Richtung des feq-Operanden),
   bis Vorzeichen und q->0,5-Grenzfall stimmen.
2. Erst mit bestandenem Harness zurueck auf die Kugel.
3. CFD_FAC_ELIBB bleibt Default AUS; der Kontrollpfad ist bitgleich unberuehrt.

## INJEKTIONSJAGD (2026-08-25): der q>0,5-ZWEIG der Blende ist der Injektor

QDIAG-Diagnosearme (Host-only, eine Variable je Lauf, Kugel DX=40, SATGATE=0, BUDGET=0,25):

| Arm | Cd nominal |
|---|---|
| Kontrolle ELIBB aus | +4,9036 |
| Blende voll | -3,3294 |
| QDIAG=1: q=1-Klemme aus | -3,2662 |
| **QDIAG=2: NUR q<0,5-Zweig** | **+2,4558** |
| QDIAG=3: nur q>0,5-Zweig | -3,1416 |

Eindeutig: der q<0,5-Zweig ist sauber (Vorzeichen physikalisch, Richtung plausibel --
Glaettung senkt die Treppen-Rauigkeitslast); der q>0,5-Zweig injiziert Impuls. Theoretisch
konsistent: bei Bouzidi ist der zweite Operand fuer q>0,5 die EIGENE
Gegenrichtungs-Population f*_ib(x_f), nicht die Wandrekonstruktion; die V1-PoU-Blende
(und damit B2) verwendet fuer beide Zweige nebb. Fuer q<0,5 ersetzt nebb den
Upstream-Operanden (Marson-Idee, funktioniert); fuer q>0,5 ist es der FALSCHE Operand.

Der Saeulen-Harness (Test E) reproduziert die Injektion NICHT -- der 1D-Fall hat keine
gekruemmte Linkpopulation; die Falsifikation brauchte das Feld. Lehre fuers
Harness-Design: Impulsaustausch-Tests brauchen mindestens einen schraegen 3D-Minifall.

ZWISCHENSTAND SCHEMA: CFD_FAC_QDIAG=2 (nur q<0,5, q>0,5 -> Identitaet = HWBB) ist das
reduzierte, nicht-injizierende Schema -- halbe Korrektur, null Risiko. Der korrekte
q>0,5-Operand (f*_ib-basiert) ist Folgearbeit mit eigenem Harness-Nachweis.

## g4-ARME (2026-08-25): auch das reduzierte Schema verschlechtert die Kippwaende -- und der Grund ist eine ASYMMETRIE

| Arm | u_tau IST/Ziel | K2 | Vergleich Kontrolle |
|---|---|---|---|
| 26 Grad QDIAG=2 | 2,245 | -1,633 | Kontrolle: 1,943 / -1,049 -> schlechter |
| 45 Grad QDIAG=2 | 1,685 | 0,0278 | Kontrolle: 1,529 / 0,0336 -> Zustand schlechter |

Erklaerung, die alle heutigen Messungen konsistent macht: die q-Blende hat zwei Haelften.
q<0,5-Links ziehen die effektive Wand NAEHER ans Fluid (mehr Widerstand); q>0,5-Links
wuerden sie WEITER wegschieben (weniger Widerstand). QDIAG=2 aktiviert nur die
widerstandsERHOEHENDE Haelfte -- die Stufenklassen (m4: q=0,26-0,52; m6: q klein) ruecken
naeher, die entlastenden q>0,5-Links (m5-Diagonale 0,735, m7-Axial 0,78) bleiben BB.
Ein einseitiges Schema MUSS die ohnehin ueberziehende Treppe (u_tau IST schon 1,5-1,9x
Ziel) weiter verschaerfen. Der korrekte q>0,5-Zweig ist also NICHT optional -- er ist
die drag-reduzierende Haelfte der Physik. Die volle Blende hatte ihn aktiv, aber mit dem
FALSCHEN Operanden (Injektion, s. Injektionsjagd).

KONSEQUENZ: Der naechste Baustein ist der KORREKTE q>0,5-Operand. Bouzidi verlangt dort
die eigene Gegenrichtungs-Population f*_ib(x_f); im Esoteric-Pull-Ladezustand ist das
naechstliegend verfuegbare Objekt fpre[ib] (t-1-Upstream) -- waehrend der bb-Operand
fpre[i] sogar t-2 traegt (Harness A). Die Zeitversatz-Frage ist genau die, die der
B0-Harness fuer den q>0,5-Zweig NIE geprueft hat. VOR der Implementierung: Planungsagent
+ 3D-Minifall-Harness (schraege Wand), der den Impulsaustausch beider Zweige gegen eine
Referenz mit exakter Wandlage prueft. KEIN weiterer GPU-Tuning-Lauf vorher.

STAND: CFD_FAC_ELIBB bleibt Default AUS; Kontrollpfad bitgleich; kipp0-Gate haelt.
Alle heutigen ELIBB-Messungen waren Screening-Laeufe am kleinen Kanal/Kugel -- kein
Produktionslauf beruehrt.

## NACHGEHOLTE B1-ABNAHME (2026-08-25 abends): die ZELLEIGENE Facettenebene ist als q-Quelle fuer GEKRUEMMTE Flaechen UNTAUGLICH

PROZESSFEHLER ZUERST: Der Entscheid definierte das Kugel-Gate "Upload-q gegen analytisches
q, RMS <= 0,143" als B1-Abnahme -- die Messleiter lief OHNE dieses Gate. Nachgeholt per
fac_q-Dump (CFD_FAC_QDUMP, g5_qdump, 725 Facetten):

- Vergleich auf 1661 gemeinsamen Links: Bias +0,65, RMS 0,666 -- GEGEN GATE 0,143.
- 2256 EXTRA-Schnitte (Upload sagt Schnitt inkl. q=1-Klemme, Analytik nicht in (0,1]).
- 700 FEHLENDE Schnitte (Analytik ja, Upload qb=0 -> purer BB; konservativ, harmlos).
- Beispiele: Upload q=1,0 wo die Analytik 0,25-0,42 sagt. y_w der Zellebenen bis 1,10.

REFERENZ-VORBEHALT (ehrlich): meine Analytik nutzte einen Kugel-Fit auf die
FACETTENZELLZENTREN (R_fit 6,96 gegen nominal 5,625) und die glatte Kugel statt des
Voxelkoerpers (Heikos Regel: der Voxelkoerper IST die Wand). Beides verschiebt die
Absolutwerte -- aber Upload 1,0 gegen 0,3 liegt WEIT jenseits jeder Referenzunsicherheit,
und die q-Verteilung erklaert die QDIAG-Messung vollstaendig: QDIAG=2 (nur q<0,5) war
sauber, weil es genau die massenhaft falschen q>=0,5-Links DEAKTIVIERT; QDIAG=3 war Gift,
weil es NUR sie aktiviert. Die Kippkanaele (exakte Ebenen -> exakte q, vom Geometrie-
Agenten bestaetigt) wurden nur schlechter-nicht-negativ; die Kugel (PCA-Ebenen mit
y_w > 1 auf grober Kruemmung) kippte ins Unphysikalische.

DAZU der CPU-Gegenbeweis: die Minikugel im 3D-Harness mit ANALYTISCHEM q zeigt KEINE
Injektion (alle Schemata innerhalb 1,7 %, auch der "falsche" q>0,5-Zweig). Der
Blenden-Operand ist damit als HAUPTtaeter entlastet (als Fehlerquelle zweiter Ordnung
bleibt er auf dem Tisch -- Planungsagent laeuft); der Haupttaeter ist die
Stufe-1-Designentscheidung "q aus der zelleigenen PCA-Ebene".

KONSEQUENZ: B1-Stufe 2 wie im Entscheid vorgesehen -- q aus der GEGLAETTETEN
REMESH-Flaeche (P1-Maschinerie, Ray-Triangle, validiert RMS 0,143 auf derselben Kugel).
Der Voxelkoerper bleibt die einzige Geometriequelle (nie STL). Danach: Kugel-q-Gate
ERST bestehen, dann Leiter.

## g8/g9 (2026-08-25 abends): REIHENFOLGE-FIX HEILT DIE KUGEL -- der Kippkanal bleibt widerspenstig

Nach dem Fix (Blende am Kopf, Wandmodell tastet den REKONSTRUIERTEN Zustand ab; Pur-Arm
CFD_FAC_ELIBB=2):
| Arm | Cd nominal | Delta-m |
|---|---|---|
| Kugel Kontrolle | +10,57 | +274 |
| Kugel PUR (nur Blende) | **+1,78** | 0 (Pur bucht nicht) |
| Kugel VOLL (Blende + iMEM) | **+3,33** | +158 |
kipp0-Anker: FELD-HASH identisch. ERSTMALS ein voller ELIBB+iMEM-Arm ohne Injektion;
die Blende nimmt der Treppenkugel massiv Scheinwiderstand (10,6 -> 3,3; Absolutbewertung
braucht die Referenzkette). Der PUR-Arm wirft den Facetten-Wirkpfad-Ist!=Soll-Fehler
(Return vor Slot 7) -- fuer den Isolationsarm dokumentiert-akzeptiert, nicht fixen.

KIPPKANAL bleibt dagegen SCHLECHT (g9, eine Variable je Lauf): 26 Grad an 4,23 / pur 2,97
(Kontrolle 1,94); 45 Grad an 1,562 / Kontrolle 1,529. Der Kanal-Sonderfall (Wand kippt in
der PERIODIK, q<0,5-Klassen dominieren die Stufen) zieht die effektive Wand naeher --
konsistent mit der Asymmetrie-Analyse. ENTSCHEID GEMAESS BROCKEN-LOGIK: der Kippkanal wird
nicht weiter totoptimiert; er misst eine Pathologie, die am Fahrzeug 3 % der Klasse
betrifft (97 % sind m2-artig). W5 (Deklaration) + ehrliches Gate sind sein Abschluss.
FAHRPLAN-PRIORITAET verschoben auf den 3/2-ABTASTPUNKT (Faktor ~2 auf tau_w, deckt die
c_f-Luecke -55 %) -- UTKORR-A/B laeuft (g10).

## g11-VALIDIERUNG (2026-08-25 nacht): Kette bestanden, Restluecke zugeordnet

JIT-Anker bitgleich; 2x2-Ecke geschlossen (kugel_aus @ neues Binary = 10,5744 exakt);
Kugel voll (projiziert + QKAPPE 0,65) = +5,25 stabil (346/2541 Links gekappt); Pur-Arm
+1,30 mit BESTANDENER Pur-Abnahme (Slot 67 = 17057, kein exit mehr).
UTKORR-Antwortkurve 1,0/1,5/2,0 -> 0,716/0,921/1,069: LINEAR, keine Saettigung -- der
wahre Faktor liegt bei ~1,7 (> Theorie 1,43-1,5): neben dem 2/3-Abtastpunkt steckt ein
weiterer linearer Eingangsdefizit-Anteil; die Turbulenz-Selbsterhaltungs-These als
Restluecken-TRAEGER ist damit gekippt (sie bleibt als Beitrag zweiter Ordnung).

## 8-mm-LEITER g12 (2026-08-25 nacht): erste Fahrzeugmessung der neuen Kette

Drei Arme (b8-Standard; +UTKORR=1,5; +ELIBB=1), 0,5 s, Fenster 0,35-0,5 s. Alle drei
liefen physikalisch komplett durch; der Bericht starb am saettigungs-blinden
Wickelwaechter (behoben: >=0xF0000000 = gewollter Endzustand, kein exit).

PROJIZIERTER PFAD (cd_facetten.csv -- der einzig gueltige an behandelten Koerpern):
| Arm | cd_druck | cz_druck | cd_reib | dm kum. |
|---|---|---|---|---|
| std | 1,6833 | -0,0541 | -0,0007 | -0,0010 |
| +UTKORR | 1,6623 (-1,2 %) | -0,0432 | +0,0018 | -0,0014 |
| +ELIBB | 1,6047 (-4,7 % ges.) | -0,0349 | +0,0084 | -0,0036 |

object_force-Pfad (forces.csv/kraft_zband, PHANTOM-kontaminiert an behandelten Links):
Cd 8,79/8,38/7,16; Cz_rest -0,144/-0,143/-0,250. WICHTIG: die Cz-RICHTUNG widerspricht
zwischen den Pfaden (projiziert: ELIBB nimmt Abtrieb WEG; object_force: gibt stark dazu).
Nach Doktrin zaehlt der projizierte Pfad; die Diskrepanz ist der Preis des offenen B3
(Blenden-Austausch ungebucht) und JETZT die dringlichste Buchhaltungsfrage.

STUFE 2 AM MASSSTAB BESTANDEN: 588.667 Facetten, 2.445.249 Remesh-Links, Ebenen-
Rueckfall 0, ohne Map-Treffer 0; ELIBB-Arm am Fahrzeug STABIL (kein NaN, Cz_band als
Kontrolle in allen Armen 0,60-0,61 deckungsgleich).

SCREENING-FAZIT: (a) Treppenglaettung wirkt am Fahrzeug in Kugel-Richtung
(cd_druck -4,7 %); (b) UTKORR ist am Fahrzeug Cd-mild (-1,2 %) -- der grosse c_f-Hebel
des Kanals uebersetzt sich NICHT linear in Fahrzeug-Cd (druckdominiert, erwartbar);
(c) NAECHSTER PFLICHTBAUSTEIN vor jeder Cz-Aussage: B3-Buchung, damit Reibungs- und
object_force-Pfad wieder EIN konsistentes Bild geben. 4 mm weiterhin nur mit Go.

## g12 KORRIGIERT (Band/Rest-Zerlegung, 2026-08-25 nacht) -- von Heiko als plausibel abgenommen

q_inf = 551,2 Pa (30 m/s ISA), A_ref = 1,85 m2. Band aus kraft_zband (druckdominiert;
ab jetzt schreibt cd_facetten.csv den exakten Facettenpfad-Split cd/cz_druck_band/rest):
| Arm | cd_druck | Band | cd_druck_REST | cz_druck |
|---|---|---|---|---|
| std | 1,683 | 0,692 | 0,992 | -0,054 |
| +UTKORR | 1,662 | 0,695 | 0,968 | -0,043 |
| +ELIBB | 1,605 | 0,692 | 0,912 | -0,035 |
KONTROLLEIGENSCHAFT: das Band ist armstabil (0,692 +- 0,003) -- die Hebel wirken an der
KAROSSERIE, nicht am Artefakt. Karosserie-Cd 0,91-0,99 bei 8 mm; 4-mm-Kriterium (<= 0,78)
in messbarer Naehe (ELIBB allein -0,079 auf den Rest).

## B3-PRUEFRUNDE (2026-08-25 nacht): meine Einfachzaehlung BEWIESEN GEKIPPT, Fix verifiziert

NO-GO des Pruefers, numerisch belegt: die Doppelzaehlung sitzt IM SELBEN Schritt -- P1 wird
NACH der Blende gemessen, das MEM-2x setzt f_out = f_in voraus, die Blende bricht das. Ihr
Tangentialanteil steckt bereits mit Faktor 2 in der phi-Buchung; meine -Dp-Kopfbuchung
machte die Gesamtsumme tangential VORZEICHENVERKEHRT (gebucht -4,89e-3 gegen wahr +4,94e-3
im Nachbau). Meine Sorge "Doppelzaehlung im naechsten Schritt" war dagegen unbegruendet
(jedes Bounce-Ereignis ist frischer Austausch).

FIX (eingebaut, vom Pruefer vorab numerisch auf ~1e-18 verifiziert): Dp-Export aus der
Blende + "+2*Dp_tangential" an der phi-Buchung; die Kopfbuchung bleibt (Normalanteil exakt;
an Rueckfall-/Pur-Pfaden, wo phi nichts bucht, ist sie allein exakt). Feld bitgleich
(reine Buchung).

DEKLARIERTER FOLGE-BEFUND (Dynamik, offen): P1 misst die Blende doppelt -> der SOLVE
verfehlt sein Ziel um -Sum(ct*Dp) -- Detektor: K2 am gekippten Kanal im ELIBB-Vollarm.
WEITER BEHOBEN: Delta-m-Waechter arm-bewusst (die Blenden-Masse ist real und haette das
Gelb-Band 200-8000x pro Fenster gerissen; unter ELIBB einmalige Ansage statt Dauersirene);
K3-Pur-Guard (n_voll-Kriterium gilt im Pur-Arm konstruktiv nicht).
OFFEN NOTIERT: Praezisionswaechter ist im Pur-Arm blind (fac_tau_n bleibt 0, B3 addiert
trotzdem) -- saturierender Additionszaehler als Folgearbeit; fac_csv-Header nennt zband
nicht. Die laufende g13-Serie traegt noch die ALTE Buchung -- ihre kugel_b3-Reibungszahlen
sind OBSOLET, nur die Hash-Anker zaehlen; Neustart nach Serienende.

## TAGESABSCHLUSS-BEFUND (2026-08-25 spaetnacht): PERF-REGRESSION im ELIBB-Arm -- Serie gesichert beendet

Der frische Kontrollanker (ELIBB=0, neues Binary c98cd88+) lief mit voller Geschwindigkeit
durch (g13b_anker, Hash unten offen -- Lauf ok); der ELIBB=1-Kanalanker kriecht dagegen
REPRODUZIERBAR bei ~2 MLUPS / 0 GB/s (g13-alt UND g13b-neu; frueher lief derselbe Arm in
~3 min, z. B. g8/g10-Aera). KEIN Treiberdefekt (Kontrollarm schnell direkt danach).
VERDACHT: Registerdruck/JIT-Spill durch die juengsten apply_facette_imem-Umbauten
(K1'-Projektion + elibb_dp-Export + B3-Buchung). MORGEN ZUERST: Perf-Bisect der
ELIBB-Emission (Kernelstand g11 gegen jetzt, CFD_DUMP_CL-Diff + Einzelschritt-Messung),
DANN erst g13b-Wiederholung (Anker-ELIBB-Hash, Kugel-B3, kipp26-Doppelzaehlungs-Detektor).
Beide Kriechserien wurden ueber den nachverfolgten Task-Stopp sauber beendet (Census 0).

## ABNAHME-EINTRAG (2026-08-26 abends): MLS-Blende ersetzt K1'-q>0,5-Zweig (Physik-Kette Baustein 1)

PRUEFKETTE VOLLSTAENDIG: Literatur-Verifikation (visuell an vier NASA/ICASE-Drucken;
Verdikt FORMEL BESTAETIGT, Attribution korrigiert auf JCP 161 (2000) 680 / PRE 65,
041203 (2002) -- NICHT MLS-1999, dort steht fuer q>=1/2 noch der FH-Zweig) -> Einbau
(kernel.cpp MLS-Block, def_fac_chifak-JIT, QKAPPE-Default 1,0 an 9 Stellen, gen_main-
Paritaet, 2 Harness-Transkriptionen) -> unabhaengiger Pruefagent gegen den Diff:
FREIGABE, kein HART-Befund. Highlights der Pruefung: Stoerform-Algebra nachgerechnet
(kein P1-Offset), c-Richtung selbst hergeleitet (3*cub = 3*e_alpha.u_bf), qb-Scan:
NUR qb=127 ergibt float-q exakt 0,5 (Bitanker konstruktiv), chi_max=0,999983<=1
(Konvexblend auch bei omega->2), def_fac_chifak-Roundtrip bitexakt, B3/slot67/dm-Block
byte-unberuehrt, nq_kappe=0-Behauptung bei Kappe 1,0 verifiziert (beide q-Quellen
liefern sqe<=1,0 bitexakt). MESSWERTE (CPU, diese Maschine): Test G (tau=0,51, 30k):
K1-alt DIV@572/200/118 ab q=0,60 -- bestaetigt die hergeleitete Neutralkurve
lambda_krit=4(2-omega)/(omega-1) rueckwirkend im Harness; MLS 5x stabil bis q=1,0.
3D: Treppe 0,9557, Kugel 0,8741 (fx=1e-5; fx=4e-5 sprengt schon HWBB -- Arbeitspunkt
dokumentiert), Flach q=0,75/1,0: 0,8658/0,8549. scratch_gate: private 0 + spill 0,
beide Arme, beide Geraete. ZWEI DEKLARIERTE INTERIMS mit Abloesebedingung im Kernel-
Kommentar: I1 tau0 statt SUBGRID-tau_eff; I2 Tangentialprojektion statt volles u_f.

ALTBEFUND-TRIAGE Test E (1D-Harness, NICHT dieser Diff -- byteidentisch am HEAD):
16/16 Verletzungen sind ein Vorzeichen-/Paritaetsartefakt: die Initialisierung schreibt
feq in lineare Slots, load_f liest bei t=0 Esoteric-Pull-vertauscht -> Start bei
Sum u_x = -0,29 statt +0,30; Kriterium su>0 kann fuer KEINEN Arm bestehen (auch HWBB
endet bei -0,0907). Zusaetzlich nutzt Test Es Blende fuer q>0,5 noch die Vor-K1'-Form
0,5/q ohne Kernel-Gegenstueck. REPARATURAUFTRAG OFFEN: Kriterium vorzeichenkonsistent
machen (|su|<=1,05*|ref|) oder Initialisierung ueber store_f-Paritaet; Blenden-Lambda
auf MLS nachziehen. Ein dauerhaft roter Harness maskiert Regressionen.

## ABNAHME-EINTRAG (2026-08-26 spaetabends): MLS-Abnahmeleiter S1-S4 komplett -- Freigabe-Basis 4mm-Vollumfang

S1 kipp0 bitgleich (4722579264326613690, beide Arme). S2 Kugel: Injektions-Pathologie
beseitigt (QDIAG=3: Cd +1,64 statt -3,3..-4,95), Wirkpfad 26825=Soll. S3: K2-Luecke
als Eigenschaft der 26-Grad-Treppenklasse attribuiert (Formel-Anteil ~0: -9,52 vs
-9,49; 45 Grad +1,13), stehender Klassenbefund -> Baustein 2. S4 8mm identische
g12-ENV: Cd/Cz-Verschiebung 1-2% (7,2655->7,4404 / +0,3235->+0,3206), Langlauf
60000 feine Schritte stabil (zweite Haelfte ruhiger), Band drei Arme konsistent
0,6734-0,6854 (2,7% unter g12-Band = Ketten-Verschiebung). Perf: 1534 MLUPs/189 GB/s
final (= Remat+F-Gate-Stand; g12-Aera 939 -> +63%). Alle Serien ueber die gelockte
Queue, Census vor/nach sauber, gt_reset_waechter aktiv waehrend der B70-Phase.
Details und Fassungen: GRENZSCHICHT-SGS-PLAN.md (Leiter-Block), Wissensspeicher
mlseinbau/mlsleiter. Morgen-Unterlage: MORGEN-4MM.md.

## ABNAHME-EINTRAG (2026-08-26 nachts): Komplettes Vor-Produktions-Audit + Korrektur-Loop geschlossen

ZWEI unabhaengige Straenge auf Heikos Anforderung, beide @ 57d1aa0:
STRANG 1 (Startaufstellung f4_vollumfang): STARTKLAR -- 32/32 Serien-Variablen mit
Konsument+Wirkpfad-Beweis; ELIBB x Wandfrei-Band kollisionsfrei; VRAM 29.321/32.655 MB
hergeleitet aus dem GEMESSENEN f4_wandfrei_v2-Fussabdruck + fac_q 47,2 MB (2.620.462
gemessene 4mm-Facetten) -- keine Trockenprobe noetig; Geraete-Default bestaetigt;
Binary byte-identisch zum Commit-Nachbau. Nebenwirkung gefunden: Einlass-Saeulen-Sonde
haengt am Slice-Block -> mit SLICE_DT=0 leer (Heiko beim Go ansagen).
STRANG 2 (Frische-Augen ueber 369c543..HEAD): SAUBER MIT NOTIZEN, kein HART-Befund;
alle 14 Doku-Zahlen-Stichproben exakt; 4 MLS-Grenzfaelle nachgerechnet; Harness
byte-identisch reproduziert; cup/feq_ib-Platzierung per IGC-Opcode-Multiset-Vergleich
als kostenlos bewiesen (7769 Instruktionen identisch).
KORREKTUR-LOOP (im selben Zug behoben + verifiziert): (1) Slot 68 = eigener
Wirkpfadzaehler des MLS-q>0,5-Zweigs (kernel.cpp, Muster Slot 67; Endreport-Zeilen
an 3 Stellen; Puffer 68->69). BEWEIS: Build RC=0, scratch_gate private 0/spill 0
unveraendert, kipp0-Bitanker HAELT (4722579264326613690, s68_kipp0_anker), Kugel-
Feuerbeweis ELIBB[67]=26825=Soll und MLS[68]=41736>0 (s68_kugel_feuer). kipp0 zeigt
konstruktiv 67=68=0 (qb=127-Kurzschluss VOR beruehrt -- konsistent zur Bitgleichheit).
(2) Stale Ansage lbm.cpp ("Reibungs-Cd kein Ist") auf B3/MLS-Stand korrigiert.
(3) lbm.hpp Slotzahl-Kommentar 66->69. OFFEN als NOTIZ (Nach-4mm-Liste): dritter
scratch_gate-Arm ohne F_NUR_SOLID; Repo-Root-Datei "0.5" aufraeumen.

## ABNAHME f4_vollumfang_mls (4mm-VOLLUMFANG, 27.08. 06:38-08:10, Commit 94a802c, SAUBER bis t=0,5s)

ERSTER 4mm-Lauf mit der kompletten validierten Kette (iMEM + ELIBB/MLS + Rueckkopplungsbaender).
KERNERGEBNIS (cd_facetten.csv, Fenster 0,2-0,5s, n=300, Block-SEM 8):
  cd_druck_rest = 0,8052 +- 0,0101   (OF13 0,599 -> +34,4 %)
  cz_druck_rest = -1,1800 +- 0,0159  (OF13 -1,301 -> -9,3 % = 91 % des Referenz-Abtriebs)
ERSTMALS ECHTER ABTRIEB IM FORK. Ketten-Beitrag eindeutig: forces.csv-Roh-Cz -0,5789 gegen
f4_wandfrei_v2 (identisch ohne Kette) -0,1341 = 15 Block-SEM; Roh-Cd unveraendert (9,8717 vs
9,8678). KOSTEN DER KETTE: NULL -- Wanduhr 91,3 min vs 92 min, Index 10.958 s-Wand/s-Physik
(Gesamtwanduhr/T_END; steady aus Schrittspalte 10.638), VRAM 29.318 MB (+44 MB, Audit-Prognose
29.321 auf 3 MB getroffen). WIRKPFADE: ELIBB[67] 894.120.500, MLS[68] 1.681.557.000, Facetten
Ist=Soll exakt, fac_q 10.869.730 Links auf 2.620.462 Facetten (100 % Remesh, 0 Rueckfaelle).
WAECHTER: Kipp still, 0 Error, GT-Reset-Waechter ohne Ausloesung, Census 0. Profiler (fdinfo
180s): B70 93,9 % @ 2512 MHz, iGPU 91,0 %, CONCURRENT 96,1 %. VTK 8 Felddateien (150/300/450/
500 ms nah+fern); y-Slice-Satz nachtraeglich aus den Dumps erzeugt (werkzeuge/vtk_yslice.py,
Format pixelgleich render_yslice) -- Anlass: CFD_SLICE_DT=0 hatte MEHR abgeschaltet als die
Interface-Slices (Sonde + normale y-Slices mit) -> Kadenzumbau (naechster Eintrag).
KORREKTUR EIGENER MELDUNGEN WAEHREND DES LAUFS: dt_grob ist 40 us (Log: "Laufzeit 0.500 s =
12500 grobe x 4 feine Schritte"), NICHT 56 us -- die zwischenzeitlich gemeldete Schritt-Tabelle
(0,7s/12501) und der "Index 7.601" waren falsch (Basis war die unbelegte 0,7s-Annahme);
korrekt: 500 ms = 12.500 grobe = 50.000 feine Schritte, Index wie oben. forces.csv/cd_facetten
werden GEPUFFERT geschrieben -- Live-Fortschritt NUR aus der Log-Schrittspalte lesen
(Werkzeugfalle, in v2-werkzeugfallen notiert).
OFFEN danach: Cd +34 % (26-Grad-Klassen-Reibungspfad = Baustein 2), 5 deklarierte Interims,
y-Interface-Verbreiterung nur via Dual-B70.


## ABNAHME-EINTRAG (27.08. vormittags): Kadenz-Umbau der fahrzeug_dd-Ausgaben (Commit d0cb0b8 + Stil-Fix)

AUFTRAG (Heiko): Slices+Kraft+VTK alle 5000 Near-Steps mit t + korrigierten cd/cz im Bild;
Interface-DIFF-Slices bleiben aus (entkoppelt); VTK an derselben Kadenz mit Behalte-2-Rotation
("nur die letzten zwei VTKs"). Plan: Planungsagent 27.08. (Befunde B1: 12,1 GB je Doppeldump
statt 8,5; B2: CSVs flushen bereits zeilenweise -- Heikos "Minuten hinterher" war die spaete
cd_facetten-Erstanlage + puffernde Leser, nicht der Schreiber; B3: Serienzeile trug noch
CFD_SLICE_DT=0 und haette den naechsten Lauf wieder blind gemacht -- im selben Commit bereinigt).
UMSETZUNG: CFD_SLICE_NEAR_STEPS (Default 5000; Legacy-Sekunden-Uhr nur bei explizitem
CFD_SLICE_DT), Einblendung ueber den graphics.cpp-6x11-Font (Kopie, da hinter #ifdef GRAPHICS),
CFD_VTK_JEDE + CFD_VTK_BEHALTE=2 (Enddump-Dedup, rotiert nie), Kadenz-Wirkpfadzaehler,
Guertel-Flush, Ansagen in allen Nicht-dd-Faellen.
HAERTEBEWEIS (8mm-Kurzlaeufe kadenz_arm_a/b, B70+iGPU, 27.08.): forces.csv UND cd_facetten.csv
BITGLEICH zwischen Kadenz-aus und Kadenz-an (cmp) -- Physik-Nullwirkung bewiesen; Wirkpfad
5/5 Kadenzpunkte (Ist=Soll); Slices 20..100 ms nah+fern mit Einblendung; VTK-Rotation
verifiziert: von 5 Doppeldumps liegen exakt die letzten zwei (80/100 ms) auf Platte.
STIL-FIX auf Heikos Sichtung: schlichte SCHWARZE Zeichen statt Balken (Sichtlauf kadenz_stil).
HOOK-STOLPERER dokumentiert: der lauf_queue-PreToolUse-Guard blockierte einen KOMBINIERTEN
Aufruf (Seriendatei-Anlage + Queue-Start) als Ganzes -- die Wiederholung lief gegen eine nie
geschriebene Serie ("0 Laeufe"). Zwei Lehren: (a) Seriendatei-Anlage und Queue-Start IMMER
getrennte Aufrufe; (b) der Guard matcht auch DOKU-Texte, die das Skript nur erwaehnen.


## BASELINE GESICHERT (27.08. vormittags, Heiko): git tag baseline-2026-08-27-f4vollumfang

Stand nach f4_vollumfang_mls-Abnahme + Kadenz-Umbau (Prueagent sauber) + README-Rework +
Repo-Bereinigung. Artefakte ohne VTK: export/baseline_2026-08-27_f4vollumfang/ (173 MB; VTK-
Enddumps im Lauf-Ordner). Referenz fuer alle folgenden A/Bs: Cd 0,805+-0,010 / Cz -1,180+-0,016
korrigiert, roh 9,8717 / -0,5789, Index 10.958, 8mm-ELIBB-Arm 1534 MLUPs, OF13-Diff RMS 5,1 m/s.
NAECHSTE REIHENFOLGE (Heiko): Baustein 2 (26-Grad) -> Perf-Restliste -> drei nachgelagerte
Auftraege (Void-Fill-Konnektivitaet, Facetten-Zensus, Cd-Luecke messen vor bauen; Volltext intern
in AUFTRAEGE-NACHGELAGERT.md, Wissensspeicher-Tag arbeitsplan).


## BAUSTEIN 2 / SCHRITT 1 -- BUCHUNGSSCHLUSS eingebaut, Pruefagent bestanden, Abnahme WARTET (27.08. mittags, Commits 7e24b88 + a18f48b)

PLANUNGSBEFUND (Planungsagent 27.08., @1f297f1): die Buchungsluecke ist real und erklaert den
26-Grad-K2-Befund VOLLSTAENDIG -- 40,2 % aller Facettenbesuche (s3_mls_26: Slot 10 = 64,1 Mio von
159,6 Mio) sind SATGATE-Rueckfaelle, weitere 33 % Einzellink (Slot 13); alle verliessen
apply_facette_imem per return VOR der phi-Buchung, gebucht wurde nur die Blenden-Korrektur -dp.
K2 = -7,4 ist die Korrektur ohne den BB-Anteil, den sie korrigiert; die Bilanz erzwingt einen
ungebuchten BB-Anteil ~ +8,4 Soll. Am 4mm-Fahrzeug dasselbe Symptom: cd_reib = -0,116 (SCHUB statt
Widerstand; OF13 +0,042). K2-Definition selbst ist konsistent (V_fluid exakt, Speicherterm ~8e-6).
NACH dem Schluss misst K2 nur noch Buchungsvollstaendigkeit (=1 per Bilanzidentitaet) -- der
Physik-Detektor fuer Schritt 2/3 ist u_tau IST/Ziel (26 Grad 2,38; 45 Grad 1,63).
EINBAU: rueckfall-Flag statt 5 returns, s=+0 explizit, Pass 2/EMA uebersprungen (Feld bitgleich
BB), bestehende phi-Buchung laeuft mit phi=P; mit Kopfbuchung -dp exakt -(2 Sum c_t fpre + d_t).
Slot 69 + Host-Detektor 69 == 13+15+64(+10+16 SATGATE) an drei Endreports. scratch_gate 4/4 sauber.
PRUEFAGENT: Kernmechanik PASS; HART 5a (Nahfeld-Detektor las das fuer lbm_c ueberschriebene
Static s_fac_satgate -> falscher exit(1)) behoben; 4a/8a/PEMA-Kommentar behoben; deklarierte
Semantikwechsel: fac_tau_n zaehlt jeden gebuchten Besuch (tau_w/y+/Tauschzellen nicht mehr
like-for-like), Markerzellen dauerhaft gegateter Facetten jetzt 'projiziert' statt 'voll'.
FX-Cd-SPLIT (Auftrag 3 Stufe A vorgezogen, cd_facetten f4_vollumfang_mls): cd_druck 1,1552 (Band
0,3500, Rest 0,8052), cd_reib -0,1163, cz_druck_rest -1,1800, cz_reib -0,0465. OF13 reproduziert
(forces.dat t>=1000, A_ref 1,85): Cd_p 0,559 + Cd_v 0,042 = 0,601 (Reibung 7,0 %), Cz_p -1,307.
Druck-gegen-Druck +0,247, Reibung-gegen-Reibung -0,158 (Vorzeichen) -- beide Luecken real.
ABNAHME A1-A4 VORBEREITET (logs/b2s1_abnahme_serie.txt, b2s1_a4_serie.txt), START BLOCKIERT:
der globale Middleware-Hook (hook_werkzeug.py) verweigert seit heute jeden lauf_queue-Aufruf hart --
auch mit run_in_background=true, weil der Hook nur den Kommandotext sieht. Heikos Entscheid.

## 2026-08-27 nachmittags -- PLAUSIBILITAETSPRUEFUNG der drei nachgelagerten CC-Auftraege

Anlass: Heiko bat um Pruefung + Anpassung der drei Auftraege, bevor sie abgearbeitet werden.
Alle Zahlen aus export/baseline_2026-08-27_f4vollumfang (4 mm) bzw. export/s4_mls_g17 (8 mm),
Fenster 0,2-0,5 s. Vier Praemissen halten nicht; zwei neue Befunde sind dazugekommen.

### B1 -- Void-Fill-Hauptverdacht gilt auf dem 4-mm-Gitter NICHT
"116.651 von 131.322 (89 %)" ist die 8-mm-Zahl (logs/s4_mls_g17.log Z. 139-144). Der
4-mm-Produktionslauf meldet 46 von 69.370 Zellen (0,066 %), X[1116,1144] Y[182,438]
Z[271,273] = Heck oben; das Grobgitter meldet "6er- und 18er-Flutung erreichen dieselben
Zellen". 46 Zellen tragen keine 0,16 Cd. Zusammen mit dem Abrat-Befund vom 23.08.
(Topologie 18/6, Kopplungsrisiko) ist Auftrag 1 vom Hauptverdacht zum 8-mm-Kill-Test degradiert.

### B2 -- die "Artefaktkorrektur -0,601" war eine Schaetzermischung
object_force (forces.csv, im Facetten-Arm phantombehaftet: Cd 9,8717 / Cz -0,5789) gegen den
Facetten-Druckpfad (cd_facetten.csv). Gemessen, Fenster 0,2-0,5 s:
  Facetten-Druck gesamt Cd 1,1552 / Cz -0,9345; z-Band Cd 0,3500 / Cz +0,2455;
  Rest (Headline) Cd 0,8052 / Cz -1,1800; Facetten-Reibung Cd -0,1180 (Vorzeichen defekt).
ECHTE Band-Korrektur innerhalb eines Instruments: Cd -0,3500 / Cz -0,2455. Der im Auftrag
verlangte "Vorzeichenwechsel" existiert nicht. Dieselbe Fehlerklasse wie R2-Korrektur H1
(Z. 749) -- zum zweiten Mal, deshalb hier ausdruecklich protokolliert.

### B3 -- z-Band trifft geometrisch genau den Latsch, mit extremem Kraftgewicht
8098 wandnahe Zellen = 0,25 % aller 3.275.381 liegen unter 16 mm, in genau zwei x-Clustern
(x 280-400 Vorderachse, x 880-1000 Hinterachse) und zwei y-Randgruppen -- dazwischen nichts.
Diese 0,25 % tragen 0,3500 von 1,1552 = 30,3 % des Druck-Cd. Das ist der Artefaktbeleg.
Skalierung 8 -> 4 mm: Band 0,6394 -> 0,3500 (Faktor 0,547, Erwartung 0,5 bei ~linear in dx =
erfuellt); Rest 1,0234 -> 0,8052 (0,787), cz_rest -0,6598 -> -1,1800 (1,79). Der alte
Falsifikator "Rest muss stabil bleiben" ist untauglich -- Artefaktabbau und echte
Gitterkonvergenz fallen zwischen zwei Sprossen zusammen.

### B4 -- Restfehler ist NICHT gleichverteilt (Diff-Zerlegung, neu gemessen)
Aus feld_nah/fern_000500ms.vtk gegen OF13 y0.xy (t=1200), XOFF 2,2063, y = 0,025 m,
dU = |u|_OF13 - |u|_FX, 670.217 auswertbare Nahfeldzellen (Gesamt-RMS 5,147 / Median -2,19
= Kontrolle gegen die Headline bestanden):
  Wandabstand <=32 mm: +4,79 m/s -- FX zu LANGSAM (zu dickes numerisches Grenzschichtprofil)
  Wandabstand >100 mm: -2,72 m/s -- FX zu SCHNELL
  stromab wandfern: vor Fahrzeug -0,64 | Front -1,61 | Mitte -2,69 | Heck -2,47 | Nachlauf -3,66
  in der Hoehe: z > 1,3 m durchweg am staerksten (Front -1,88, Mitte -2,88, Heck -4,09);
  im 16-mm-Fernfeld dasselbe Muster ueber dem Nahfeld-Deckel (z 1,3-3,0 m: -3,3 bis -4,7).
Deutung: ein mit der Lauflaenge wachsender Kanal-/Deckeleffekt, kein Verdraengungsbild eines
versiegelten Kuehlers (das saesse vorne unten). Die wandnahe Umkehr ist der direkte Nachbar
des 26-Grad-Themas und gehoert als Vorher-Wert in die Abnahme von Baustein 2.

### B5 -- Versperrung: neuer, nie bilanzierter Beitrag
FX-Fernfeld 7,68 x 8,83 m = 67,57 m2 -> A_ref/A_quer 2,74 % (Log Z. 38). OF13-Kanal laut
checkMesh (-7,-6,0)..(17,6,8) = 12 x 8 m = 96 m2 -> 1,93 %. FX ist um Faktor 1,42 staerker
versperrt. Erste Ordnung (dCd/Cd ~ 2*dB): ~1,6 % von 0,805 = ~0,013 Cd. Klein, aber
systematisch und mit genau der Signatur aus B4.

### B6 -- Auftrag 2 hatte keine Datenbasis, hat aber einen besseren Zaehler
elibb_qmap_dd (setup.cpp:3579) ist eine Host-Map und wird nie geschrieben;
facetten_histogramme.csv enthaelt keine Normalen. Ersatz: klasse-Bit 16 (Orientierungskipp,
setup.cpp:1348) = 121.793 Zellen / 3,72 % und r21 > 0,5 = 104.157 / 3,18 %; beide sitzen zu
50-56 % im ERSTEN x-Zehntel, 30-36 % im zweiten, z 32-96 Zellen = Frontschuerze/Splitter --
nicht Gurney/Canards/Endplatten am Heck (0,5-2 %). Die Motivation des Auftrags war falsch
verortet. Der kraftgewichtete Gate-Wert existiert bereits als n_unklar (setup.cpp:1512,
|Summe der 18er-Nachbarnormalen| < 0,5, geht konservativ voll in die Kraft), wird aber nur im
Kanal-Endreport gedruckt (Z. 1895 f.), nicht im dd-Report -- eine print_info-Zeile fehlt.

### FOLGE fuer die Reihenfolge
3B (Artefaktabnahme, korrigiert) und 3D (Bias/Versperrung) sind ohne Lauf machbar; 3A haengt
an der GPU-Abnahme A4 des Buchungsschlusses (cd_reib muss erst positiv sein); Auftrag 1 ist
ein 10-Minuten-Kill-Test bei 8 mm; Auftrag 2 braucht zuerst die eine Zaehlerzeile.
Details in AUFTRAEGE-NACHGELAGERT.md (Revision 2, lokal, gitignored).

### B7 -- OF13 traegt im Latschband fast nichts: der FX-Abzug ist gerechtfertigt (27.08. nachmittags)
Die in B3 offengelassene Frage ist gemessen. Werkzeug: werkzeuge/of13_kraft_zband.py (neu),
rein lesend auf ~/CFD-Cases/mr2v40H, Zeit 1200, vehicle-Patch (2.648.253 Randflaechen,
44,63 m2). p ist dort zeroGradient -> Druck der Owner-Zelle; wallShearStress liegt als
calculated-Randfeld vor; Sf aus faces+points per Fan-Triangulierung.
ABNAHME BESTANDEN, beide Kraftarten exakt: Druck Fx +565,444 / Fz -1337,296 und Reibung
Fx +42,811 / Fz +6,609 reproduzieren forces.dat auf 0,00 %. (Der erste Durchlauf lieferte die
Reibung mit exakt umgekehrtem Vorzeichen -- OpenFOAMs wallShearStress traegt das Minus bereits;
ohne die Kontrollsumme waere das unbemerkt geblieben. Beleg dafuer, dass eine Zerlegung ohne
Gesamtabnahme wertlos ist.)
ERGEBNIS (Cd/Cz auf A_ref 1,85 m2, q_inf 551,2 Pa):
  GESAMT      Cd_p +0,5545  Cd_v +0,0420  Cd 0,5964 | Cz_p -1,3113  Cz_v +0,0065  Cz -1,3048
  z <  16 mm  7.054 Flaechen (0,27 %), 0,1355 m2 (0,30 %): Cd_p +0,0071 = 1,28 % von Cd_p,
              Cz_p -0,0044 = 0,33 %
  z <  32 mm  Cd_p +0,0147 (2,64 %)   |  z < 64 mm  Cd_p +0,0290 (5,24 %)
  z < 120 mm  Cd_p +0,0612 (11,04 %), Cz_p -0,8881 = 67,72 % des Abtriebs
VERDIKT: im GLEICHEN geometrischen Band (unterste 16 mm) traegt OF13 1,28 % seines Druck-Cd,
FX dagegen 30,3 % (0,3500 von 1,1552) -- Faktor 24 im Anteil, Faktor 49 im Absolutbetrag
(0,0071 gegen 0,3500). Der FX-Bandabzug entfernt damit fast ausschliesslich Artefakt und ist
gerechtfertigt. Er nimmt aber ~0,007 ECHTE Kraft mit: der faire Vergleich lautet
  FX Rest 0,8052 gegen OF13 (Cd_p minus Band) 0,5474  ->  Luecke +0,2578
statt der bisher genannten +0,246. Die Luecke wird durch die saubere Rechnung also GROESSER,
nicht kleiner. Ab jetzt gilt 0,5474 als Druck-Messlatte fuer den bandkorrigierten FX-Cd.
NEBENBEFUND mit eigenem Wert: 67,7 % des OF13-Abtriebs sitzen unter 120 mm Hoehe (Unterboden/
Diffusor) bei nur 15 % der Wandflaeche. Wer Cz bewegen will, bewegt den Unterboden.

### B8 -- Zensus-Diagnostik fuer Auftrag 2 eingebaut (setup.cpp, uncommittet zum Zeitpunkt der Notiz)
FacKraft traegt jetzt ux/uy/uz + ukraft_ok; die Host-Kraftschleife summiert den Beitrag der
"unklaren" Zellen getrennt mit (|Summe der Nachbar-Facettennormalen| < 0,5 = gegenlaeufige
Wandseiten in EINER Nachbarschaft). Der dd-Endreport druckt Zellklassen und -- im Host-Pfad --
das Kraftgewicht. Damit liefert JEDER kuenftige Lauf den Gate-Wert, ohne den ein Cluster-Umbau
nicht zu rechtfertigen ist. Physik unveraendert (nur eine zusaetzliche Summe); im reinen
GPU-Pfad bleibt ukraft_ok false, damit keine 0 als Messwert gelesen wird.
ABNAHME OFFEN (GPU blockiert): ein 8-mm-Zensuslauf mit CFD_FAC_GPU_PRUEF=1 muss zeigen, dass
der Report feuert und die Host-Zaehler mit den GPU-Zaehlern uebereinstimmen.

### B9 -- INTERFACE-SERIE (Heiko-Auftrag 27.08. nachmittags) und ein Fund, der wichtiger ist
Auftrag: Schnittebenen durch die FERNdomaene gegen OF13, zugeschnitten aufs Nahfeld-Format,
in 8-Grobzellen-Schritten (128 mm) ab der Fahrzeughuelle -- "ab wo ist der Abdruck klein genug".
Werkzeug: werkzeuge/interface_serie.py (neu), Konvention wie export/vergleich_of13_2026-08-26
(d = |u|_FX - |u|_OF13, +-15 m/s). OF13 kommt NICHT mehr aus den vier festen sample-Ebenen,
sondern aus einem Cache der 33,5 Mio. Zellzentren -- damit ist jede Ebene ziehbar.
ABNAHME des Werkzeugs: |u|_OF13 = 29,695 +- 0,182 m/s bei x = -2,0 m (Soll 30,0) -- bestanden.
Vorher gab es zwei Fehlversuche, beide durch die Abnahme gefangen: (1) Slab +-80 mm ist duenner
als die groebsten OF13-Zellen (~250 mm) -> Ebenen fielen komplett aus (z = 3,000 m) und die
Zellzahl sprang zwischen 20.304 und 51.606; Fix: Slab 150 mm und "naechste Zelle gewinnt"
statt Mittelung ueber Verfeinerungsstufen. (2) Ein vermeintlicher Vorzeichenwiderspruch zu B4
war meine eigene Verwechslung der beiden Diff-Konventionen -- beide Werkzeuge tragen die
Konvention jetzt im Kopfkommentar.

ERGEBNIS je Richtung (Abstand ab Fahrzeughuelle; heutige Nahfeld-Grenze in Klammern):
  x- vor dem Fahrzeug (320 mm): KLINGT SAUBER AB. RMS 2,01 -> 0,15 m/s ueber 2,4 m;
     Anteil |d| > 1 m/s faellt 90,8 % -> 9,2 % bei 640 mm -> 3,1 % bei 1024 mm.
     An der HEUTIGEN Grenze stehen noch 71 % ueber 1 m/s. Die Box ist vorne zu knapp;
     640-770 mm waeren der belegte Richtwert.
  y- / y+ seitlich (316/320 mm): KLINGEN NICHT AB. Bei 2,4 m Abstand immer noch +2,0 m/s
     Mittel, 78 % ueber 1 m/s. |u|_FX 32,5-34,4 gegen OF13 30,5-31,3.
  z+ ueber dem Fahrzeug (732 mm): WIRD MIT DEM ABSTAND SCHLECHTER, +2,97 -> +6,58 m/s;
     |u|_FX steigt von 31,75 auf 37,01 waehrend OF13 bei 30,4 bleibt.
  x+ hinter dem Fahrzeug (1988 mm): bleibt bei +3,1..+4,4 m/s, RMS ~5,4 (Nachlauf, LES gegen
     RANS -- hier ist ein Teil erwartbar).

### B10 -- HARTER BEFUND: defekte Anstroemung im Fernfeld des Produktionslaufs
Die z+-Serie war physikalisch unmoeglich (37 m/s bei 2,74 % Versperrung), also Kontrolltest am
ROHEN Fernfeld, 2,4 m VOR dem Fahrzeug, wo nichts stoeren kann: dort steht die Anstroemung auf
allen Hoehen bei 29,8 m/s -- AUSSER in einer Schicht z = 4,56..5,63 m.
  ux steigt dort auf 35,5 m/s, bricht dann auf 10,8 m/s ein; uz erreicht -14,7 m/s (Abwaerts).
  Die ganze xy-Ebene bei k = 285 hat Kern-Mittel 40,26 m/s, max 53,60 m/s.
ZEITLICH EXAKT STATIONAER: 150/300/450/500 ms liefern 38,04 / 38,17 / 38,16 / 38,17 m/s am
selben Ort z = 5,07 m. Kein Transient, sondern etwas kontinuierlich Eingespeistes.
DIE EINGEBAUTE SONDE HATTE ES: export/f4_vollumfang_mls/einlass_saeule.csv weist 60 von 552
Zellen mit mehr als 2 % Abweichung aus, z = 4,624..5,584 m, min ux_rel = 0,405. Die Datei liegt
seit dem Lauf vor; es hat nur niemand hingesehen. Iron Rule 5 hat funktioniert -- die Sichtung
nicht.
ENTSTEHUNGSORT: die Einlassebene selbst (i = 0) ist mit 30,00 m/s und uz = 0 exakt richtig; die
Stoerung waechst ueber die ersten ~20 Grobzellen (0,32 m) auf ihr Maximum 39,6 m/s und sinkt
stromab schraeg ab (z = 5,63 m bei i = 1 -> 3,97 m bei i = 760). Log Z. 291 nennt den
Verdaechtigen: "Geschwindigkeits-Einlass Fernfeld AUS (CFD_FERN_VI=0, gemessener Default): rho
bleibt am Einlass festgenagelt, der Rand reflektiert".
LAUFSPEZIFISCH, NICHT GEOMETRISCH -- der entscheidende Vergleich ueber alle Laeufe mit 552
Fernfeld-Zellen: f4_std_diff2, f4_wandfrei_v2, f4_kopplung_prod, f4_kopplung_plateau sind oben
SAUBER; f4_vollumfang_mls (und sein Baseline-Abzug) tragen die Stoerung; f4_w3ff_prod hat eine
schwaechere Variante (142 Zellen, min 0,943). Es haengt also an der Konfiguration des heutigen
Produktionslaufs, nicht am Fernfeld-Aufbau.
TRAGWEITE: an der Nahfeld-Deckelkopplung z+ (z = 1,94 m) misst die Serie |u|_FX 34,03 gegen
OF13 30,79 -- 10,5 % zu schnell; seitlich +10 %. Diese Werte werden JEDEN groben Schritt ins
Nahfeld eingespeist (Log Z. 333: "Im Nahfeld werden x-, y+-, z+ jeden groben Schritt aus dem
Fernfeld uebernommen"). Ob und wieviel davon in Cd/Cz landet, ist NICHT gemessen -- aber die
Headline-Zahlen Cd 0,8052 / Cz -1,1800 stammen aus genau diesem Lauf und stehen damit unter
Vorbehalt, bis der A/B gegen einen sauberen Arm vorliegt.
FOLGE fuer den Auftrag: die Box-Dimensionierung ist aus diesem Lauf NUR fuer x- ableitbar
(640-770 mm). Fuer y und z muss zuerst die Anstroemung sauber sein -- sonst misst man den
Einlassdefekt und nennt es Boxgroesse.

### B11 -- VERGLEICH gegen die Laeufe vom 21./26.08. + Heikos 3-m/s-Kriterium (27.08. abends)
Heiko: "die Bilder waren damals generell nicht so rot" -- nachgemessen, er hat recht, und zwar
um Faktor 5. Dieselbe Serie (werkzeuge/interface_serie.py, gleiche Abnahme bestanden:
|u|_OF13 = 29,695 +- 0,182 bei x = -2,0 m) fuer drei Laeufe:
  mittleres |d| ueber ALLE Ebenen aller fuenf Richtungen:
    f4_std_diff2 (21.08., ohne N2F-Rueckkopplung)  0,65 m/s
    f4_wandfrei_v2 (26.08., mit Rueckkopplung)     0,48 m/s   <- der beste Stand
    f4_vollumfang_mls (27.08., Produktion)         2,68 m/s   <- Faktor 5,6 schlechter
  je Richtung (std_diff2 / wandfrei_v2 / vollumfang):
    x- 0,30 / 0,35 / 0,51 | x+ 0,25 / 0,10 / 3,75 | y- 0,86 / 0,62 / 2,60
    y+ 0,86 / 0,64 / 2,55 | z+ 0,95 / 0,67 / 4,00

HEIKOS KRITERIUM (|Mittel von d| < 3 m/s, auf den im Slice angezeigten Durchschnitt bezogen):
beide alten Laeufe erfuellen es in ALLEN Richtungen schon AN der Fahrzeughuelle (0 mm). Das
Kriterium taugt damit als Freigabe-Schwelle, nicht zur Box-Dimensionierung -- es ist erfuellt,
bevor Abstand ueberhaupt eine Rolle spielt. Der heutige Lauf reisst es in x+ und z+ dauerhaft.

BOX-DIMENSIONIERUNG aus dem besten sauberen Lauf (f4_wandfrei_v2), Abstand ab Fahrzeughuelle,
ab dem |Mittel| DAUERHAFT unter der Schwelle bleibt -- gegen die heute gebaute Grenze:
  Richtung      heute gebaut    < 3 m/s   < 1 m/s   < 0,5 m/s
  x- vorne          320 mm         0 mm      0 mm       0 mm
  x+ hinten        1988 mm         0 mm      0 mm       0 mm
  y- seitlich       316 mm         0 mm    384 mm    2048 mm
  y+ seitlich       320 mm         0 mm    512 mm    2048 mm
  z+ oben           732 mm         0 mm    640 mm    1792 mm
=> Bei 1-m/s-Anspruch ist die Box OBEN richtig (732 gegen 640 noetig, etwas Reserve) und
SEITLICH knapp zu klein (316/320 gegen 384/512 noetig). Bei 0,5-m/s-Anspruch muesste sie
seitlich auf rund 2 m wachsen -- das ist genau der VRAM-Deckel aus dem y-Interface-Entscheid
(Near-y breiter nur ueber Dual-B70). Die Reihenfolge der Hebel ist damit belegt:
zuerst den Lauf sauber bekommen (Faktor 5), danach lohnt seitliche Box-Erweiterung (Faktor <2).

### B12 -- KORREKTUR meiner eigenen Ursachen-Vermutung zur Einlassstoerung
Ich hatte B10 mit "laufspezifisch" geschlossen und ELIBB=1 / UTKORR=1,5 als einzigen
Konfigurationsunterschied zwischen f4_wandfrei_v2 (sauber) und f4_vollumfang_mls (gestoert)
genannt. Heiko hat widersprochen: er hat die Stoerung schon frueher bei Testlaeufen gesehen.
NACHGEMESSEN ueber ALLE 115 Laeufe mit einlass_saeule.csv -- er hat recht, meine Vermutung faellt:
  13 Laeufe zeigen eine Stoerung oberhalb der Fahrzeughoehe. Zehn davon sind Kurzlaeufe
  (T_END < 0,4 s) vom 19.-23.08., die meisten OHNE ELIBB und ohne UTKORR. Die
  ELIBB-Korrelation ist 2/25 gegen 11/89 ohne -- sie traegt nicht.
  Der beste Praediktor ist die LAUFZEIT: von 74 Kurzlaeufen sind 10 gestoert (schwaechstes
  ux_rel 0,955), von 41 Langlaeufen (>= 0,4 s) nur DREI:
    f4_w3ff_prod (22.08.)   142 Zellen, min ux_rel 0,943, z 3,38..8,78 (breit, bis zur Decke)
    f4_vollumfang_mls       60 Zellen,  min ux_rel 0,405, z 4,62..5,58 (schmal, sehr stark)
    baseline_2026-08-27     dieselbe Datei wie oben (Kopie)
  Belegt nicht-deterministisch: q_det_1 und q_det_2 sind ZWEI LAEUFE DERSELBEN Konfiguration
  (Determinismustest, 23.08. 14:46) -- beide gestoert, aber mit 28 bzw. 29 Zellen. Zwei Laeufe,
  gleiche Env, unterschiedliche Zellzahl.
DEUTUNG, ehrlich getrennt:
  (a) Die schwachen Faelle (ux_rel 0,94..0,97) in Kurzlaeufen sind plausibel ein Anlauf-
      transient -- bei 40 ms hat die Ferndomaene (12,3 m bei 30 m/s = 0,41 s) nicht einmal
      einen Durchlauf hinter sich.
  (b) Der heutige Fall ist damit NICHT erklaert: ux_rel 0,405 ist um Groessenordnungen
      staerker, und er klingt ueber 500 ms nicht ab (150/300/450/500 ms liefern 38,04/38,17/
      38,16/38,17 m/s am selben Ort z = 5,07 m). Ein Transient tut das nicht.
  URSACHE OFFEN. Was als naechstes zu messen waere: derselbe Lauf zweimal (Determinismus auf
  Langlauf-Laenge), und ein Lauf mit CFD_FERN_VI=1 -- Log Z. 291 sagt, dass der Einlassrand
  heute reflektiert, weil der Geschwindigkeits-Einlass im Fernfeld AUS ist.

### B13 -- BOX-UMSCHICHTUNG bei konstantem VRAM durchgerechnet (27.08. abends, Heiko-Idee)
Heiko: "z+ um 70 mm verringern und es lieber in y-/y+ geben; x- koennte naeher, aber die
Vorwaerts- und Rueckwaertskopplungsbaender brauchen Abstand zueinander." Nachgerechnet auf
dem heutigen Nahfeldgitter 1689 x 621 x 485 @ 4 mm = 508,7 Mio Zellen.

TAUSCHKURS (der Grund, warum z+ ein schlechter Geber ist):
  eine z-Zelle kostet 1,05 Mio Zellen, eine y-Zelle 0,82 Mio, eine x-Zelle nur 0,30 Mio.
  => 1 mm aus z kauft 0,64 mm je y-Seite; 1 mm aus x kauft 0,18 mm je y-Seite.
  z+ 72 mm abgeben bringt also nur +44 mm je y-Seite (y 316/320 -> 360/364 mm) -- der
  1-m/s-Bedarf ist 384 (y-) bis 512 mm (y+). Heikos Idee zeigt in die richtige Richtung,
  deckt aber nur etwa ein Drittel bis zwei Drittel der Luecke, und y+ gar nicht.

KOPPLUNGSBAND-ABSTAND, Heikos Sorge (Log Z. 407, eingebauter Waechter): heute x- 13, y+- 13,
z+ 39 Grobzellen, Soll >= 2, komfortabel >= 4.
  x- um 144 mm naeher ziehen => Abstand faellt auf exakt 4,0 GZ = die Komfortgrenze.
  144 mm ist damit der HARTE Spielraum in x-; er kauft nur +24 mm je y-Seite.
  z+ um 72 mm senken => 34,5 GZ, voellig unkritisch.

SZENARIEN bei konstanter Zellzahl (jeweils <= 0,3 % Abweichung):
  nur z+ 72 mm                 1689 x 643 x 467   y +44 mm/Seite -> 360/364 mm
  nur x- 144 mm                1653 x 633 x 485   y +24 mm/Seite -> 340/344 mm
  nur x+ 512 mm                1561 x 671 x 485   y +100 mm/Seite -> 416/420 mm
  z+ 72 + x- 144               1653 x 657 x 467   y +72 mm/Seite -> 388/392 mm
  z+ 72 + x- 144 + x+ 512      1525 x 713 x 467   y +184 mm/Seite -> 500/504 mm  <- deckt beide
  z+ 92 + x- 144 + x+ 768      1461 x 753 x 462   y +264 mm/Seite -> 580/584 mm

DER EIGENTLICHE HEBEL IST x+, MIT EINEM ERNSTEN VORBEHALT: x+ hat heute 1988 mm hinter dem
Fahrzeug, und das 1-m/s-Kriterium ist dort schon bei 0 mm erfuellt. Aber das Kriterium misst
die UEBEREINSTIMMUNG MIT OF13 in der Ebene, nicht ob die Physik dort aufgeloest gehoert.
Hinter dem Fahrzeug liegt der Nachlauf; OF13 ist RANS und mittelt ihn glatt, FX loest ihn auf.
Dass beide dort "uebereinstimmen", heisst NICHT, dass man den Nachlauf ins 16-mm-Gitter
schieben darf -- er bestimmt ueber den Basisdruck direkt Cd. x+ kuerzen braucht deshalb einen
eigenen A/B (Cd/Cz gegen die Baseline), nicht nur diese Kennzahl.

ZWEITER VORBEHALT -- die Bedarfszahlen stehen auf n=2, und die beiden Laeufe widersprechen sich:
  wandfrei_v2 (mit Rueckkopplung, wie Produktion): y- 384, y+ 512, z+ 640 mm
  std_diff2   (ohne Rueckkopplung):                y- 640, y+ 640, z+ 1152 mm
  heute gebaut:                                    y- 316, y+ 320, z+ 732 mm
Nach std_diff2 waere z+ heute ZU KLEIN (732 gegen 1152) -- dann waere Kuerzen falsch herum.
wandfrei_v2 ist der relevantere Lauf (gleiche Kopplung wie die Produktion), aber ein einzelner
Lauf traegt keine Geometrieentscheidung. Die y-Asymmetrie 384/512 ist ausserdem genau EINE
Rasterstufe (128 mm) an einem symmetrischen Fahrzeug -- also Rauschen, nicht Physik.

EMPFEHLUNG: die Umschichtung NICHT jetzt bauen. Sie bringt Faktor < 2, waehrend der gestoerte
Produktionslauf Faktor 5,6 kostet (B11). Reihenfolge: (1) Einlassstoerung klaeren, (2) zwei
saubere Langlaeufe mit Rueckkopplung als Basis fuer die Bedarfszahlen, (3) dann umschichten --
und x+ nur mit eigenem Cd-A/B. Bis dahin ist die belastbare Aussage: z+ hat vermutlich Reserve,
y ist zu knapp, x- hat 144 mm Spielraum bis zur Band-Komfortgrenze.

### B14 -- HEIKOS BLOCKAGE-DEUTUNG GEPRUEFT: beim Widerstand bestaetigt, beim Abtrieb offen
Heiko: "das Artefakt im Fernfeld ist bestimmt auch fuer die generell zu hohe Luftgeschwindigkeit
aufgrund von Blockage verantwortlich -- das verringert meist Abtrieb und erhoeht den Widerstand,
soweit ich mich an die Versuche mit verschiedenen Boxgroessen erinnere."

GEMESSEN (mittleres |u|_FX / |u|_OF13 ueber alle Ebenen aller fuenf Richtungen der Serie):
  f4_std_diff2   (21.08., sauber)    +1,83 %
  f4_wandfrei_v2 (26.08., sauber)    +1,59 %
  f4_vollumfang_mls (27.08., gestoert) +9,20 %
Die beiden sauberen Laeufe liegen damit in der Groessenordnung der reinen GEOMETRIE-Versperrung
(FX 2,74 % gegen OF13 1,93 %, B5) -- der Ueberschuss ist dort erklaert. Der gestoerte Lauf traegt
rund 7,5 Prozentpunkte ZUSAETZLICH. Heikos Deutung ist damit belegt: das Artefakt wirkt wie eine
zusaetzliche Versperrung.

VERTEILUNG des Ueberschusses (gestoert / sauber / Differenz in Prozentpunkten):
  vor dem Fahrzeug     +1,78 / +1,23 / +0,55 pp
  seitlich y-          +8,44 / +2,01 / +6,43 pp
  seitlich y+          +8,26 / +1,99 / +6,27 pp
  UEBER dem Fahrzeug  +13,12 / +2,20 / +10,92 pp
  hinter dem Fahrzeug +15,07 / +0,21 / +14,86 pp
Die Anstroemung ist praktisch unberuehrt, der Ueberschuss sitzt dort, wo die Stroemung um den
Koerper herum muss -- das klassische Versperrungsmuster.

WIDERSTAND -- Heikos Aussage bestaetigt, mit Betrag: Kraefte skalieren erster Ordnung mit u^2,
also Faktor ((1+0,0159)/(1+0,0920))^2 = 0,8655, um den gestoerten Lauf auf das Blockage-Niveau
des sauberen zu bringen:
  Cd gemessen 0,8052 -> blockage-bereinigt 0,6969; Luecke zu OF13 (0,5474, B7) faellt von
  +0,2578 auf +0,1495. DIE ZUSATZ-BLOCKAGE ERKLAERT DAMIT 42 % DER Cd-LUECKE.
  Das ist eine Abschaetzung erster Ordnung, keine Messung -- eine echte Windkanalkorrektur
  (Mercker/Wiedemann) ist nichtlinear und braucht Nachlauf- und Verdraengungsanteile getrennt.

ABTRIEB -- NICHT entscheidbar aus diesen Laeufen, zwei Effekte wirken gegeneinander:
  (a) reine u^2-Skalierung: Cz -1,1800 -> -1,0213, also weiter WEG von OF13 -1,3113.
  (b) Heikos Verteilungsargument: der Ueberschuss ist OBEN am groessten (+13,1 % gegen +2,2 %),
      und mehr Unterdruck ueber dem Dach kostet Abtrieb. Das wirkt (a) entgegen.
  Dazu kommt eine harte Konfundierung: f4_vollumfang_mls hat zusaetzlich ELIBB=1, MLS und
  UTKORR=1,5 -- genau die Kette, die den Abtrieb heute Morgen erst auf -1,18 gebracht hat.
  Das Cz-Signal traegt beide Ursachen und ist nicht trennbar.
  ZU MESSEN: derselbe Code, ein Lauf mit und einer ohne Stoerung (eine Variable). Erst dann
  ist Heikos Abtriebs-Aussage an diesem Aufbau pruefbar.

FOLGE FUER DIE HEADLINE-ZAHLEN: Cd 0,8052 steht unter Vorbehalt -- rund 0,11 davon sind
plausibel Artefakt-Blockage, nicht Modellfehler. Cz -1,1800 ebenfalls, mit unklarem Vorzeichen
der Korrektur. Beide bleiben als GEMESSENE Werte des Laufs gueltig; was sie ueber die Physik
sagen, haengt an der Klaerung der Einlassstoerung (B10/B12).

### B15 -- BOX-UMSCHICHTUNG festgelegt (Heiko-Entscheid 27.08. abends): z+ -64 mm, y +64 mm je Seite
Heiko: "nur z+ 72 mm herabsetzen und entsprechend y-/y+ verbreitern; wir haben noch VRAM-Reserve,
nimm 0,5 GB davon und gib die auch an y."

RASTERBEDINGUNGEN (setup.cpp:3055-3096, Fassung ce9c6a3) -- der Grund, warum 72 mm nicht geht:
  Die Nahfeld-Masse sind ans Grobgitter gebunden: fN = (ce-1)*ratio+1, ratio 4, dx_c 16 mm.
  z ist damit nur in 16-mm-Schritten aenderbar; 72 mm sind 4,5 Grobzellen. Gewaehlt: 64 mm (4 GZ),
  weil es z+ mehr Reserve laesst als 80 mm (668 gegen 652 mm bei Bedarf 640).
  In y verlangt setup.cpp:3088 zusaetzlich GERADE cey (Symmetrie um die Mittelebene, sonst waere
  die Box unsymmetrisch -- "ein stiller Fehler in genau der Groesse, die hier am empfindlichsten
  ist"). y aendert sich daher in Schritten von 2 GZ = 16 mm je Seite.

VRAM (aus dem Lauf-Log, nicht geschaetzt): B70 hat 32.655 MB, belegt waren 29.318 MB fuer
508.701.465 Zellen = 60,4 B/Zelle im Mittel (inkl. Facetten 610,8 MB Index + 80 MB Geometrie und
gesparter F-BBox). Marginal fuer neue FLUID-Zellen unter FP16C: 19 DDF x 2 + flags 1 + u 12 +
rho 4 = 55 B. Heikos 512 MB erlauben damit +9,76 Mio Zellen, Obergrenze 518,5 Mio.

FESTGELEGT (Heiko hat die groessere y-Variante gewaehlt, 27.08. abends):
  CFD_NEAR_LZ 1.9360 -> 1.8560   cez 122 -> 117, fNz 485 -> 465, z+ 732 -> 652 mm
  CFD_NEAR_LY 2.4800 -> 2.6400   cey 156 -> 166, fNy 621 -> 661, y  316/320 -> 396/400 mm
  Gitter 1689 x 661 x 465 = 519.139.485 Zellen (+2,05 %), VRAM ~29.865 MB (+547 MB).
  Das sind 35 MB ueber den genannten 512 MB (+6,9 %) -- ausdruecklich so gewaehlt; die Reserve
  auf der B70 bleibt 2.790 MB. y- (Bedarf 384 mm) ist damit uebererfuellt, y+ (512 mm) zu rund
  drei Vierteln gedeckt; der Rest braucht Dual-B70. z+ behaelt 12 mm ueber dem Bedarf 640 mm.
  Verworfene kleinere Variante: cez 118 + cey 164 -> y 380/384 mm, z+ 668 mm, 29.767 MB.
RASTERUNG GEGEN DEN CODE NACHGERECHNET (setup.cpp:3034 n_cells, :3044 auf_grobe_zelle, :3087 f.):
  auf_grobe_zelle trifft 2,6400 und 1,8560 exakt (165 bzw. 116 Grobzellen); cey 166 ist GERADE,
  die Paritaetsbedingung gegen cNy 480 ist also erfuellt und die stille Korrektur cey++ greift
  NICHT; NF_OY 157, NF_OY + cey = 323 <= 480. Beide Werte liegen fern der Rundungsgrenzen.

NEBENBEDINGUNGEN geprueft:
  - N2F-Band-Abstand z+ faellt 39 -> 34 Grobzellen (Soll >= 2, komfortabel >= 4) -- unkritisch;
    y+- steigt von 13 GZ, weil die Entnahmeebene weiter vom Koerper wegrueckt.
  - Fernfeld umschliesst die neue Box: NF_OY + cey = 158 + 164 = 322 <= cNy 480.
  - Laufzeit steigt mit der Zellzahl um rund 2,1 % (Index 10,958 -> ~11,18 s-Wand je s-Phys).
  - ABNAHME AM LOG: es MUSS "Fein (Geraet 1, ...): 1689 x 661 x 465" erscheinen. Ein anderes fNy
    hiesse, dass die Paritaetskorrektur gegriffen hat und die Box nicht die bestellte ist.
  - x bleibt unveraendert (die 144 mm Spielraum in x- wurden NICHT genommen).

VORBEREITET, NICHT GESTARTET: logs/f5_box_serie.txt enthaelt einen 8-mm-FORMTEST (Minuten statt
90 min). Er beweist, dass Kopplungspruefung ("Deckungspunkte, Fahrzeugfreiheit und Weltlage aller
fuenf Ebenen in Ordnung") und Symmetriebedingung mit den neuen Massen durchgehen, bevor 4 mm
laeuft. Die 4-mm-Zeile liegt daneben in logs/f5_box_serie.txt.4mm und braucht Heikos Go.

ZWEI VORBEHALTE, die bei der Auswertung gelten:
  (1) Die Bedarfszahlen stehen auf EINEM sauberen Lauf; f4_std_diff2 verlangt z+ 1152 mm, danach
      waere schon 732 zu klein. Heiko hat das entschieden -- der Vorbehalt bleibt protokolliert.
  (2) Solange die Einlassstoerung (B10/B12) nicht geklaert ist, misst ein Vergleich gegen die
      Baseline BOX UND STOERUNG gemischt. Ein sauberes Box-A/B verlangt zwei Laeufe mit
      demselben Stoerungszustand.

### B16 -- DUAL-B70-KAPAZITAETSRECHNUNG fuer Heikos Zielbox (27.08. abends)
Heiko gibt als Zielabstaende ab Fahrzeughuelle vor: x- 600, y-/y+ 1200, z+ 1200, x+ 3000 mm.
Frage: welche Aufloesung bei GLEICHER GPU-Auslastung wie heute?

ZIELBOX: x -0,606 .. +7,442 (8,048 m), y +-2,124 (4,248 m), z 0 .. 2,408 m = 82,32 m3.
Heute 32,50 m3 -- Faktor 2,53.

AUSLASTUNGSVORGABE: heute 29.318 MB von 32.655 MB auf EINER B70 = 89,8 %. Auf zwei B70 sind das
58.636 MB Nahfeld-Budget (physisch 65.310). Marginal 55 B/Zelle (FP16C).

ERGEBNIS (Rasterung gegen setup.cpp:3011/3034/3044/3087 gerechnet, ratio 4, Paritaet cey beachtet):
  dx_f    dx_c  |  Gitter              Zellen   VRAM ges.  je GPU   Auslastung | Ist-Abstaende x-/y/z+/x+
  4,00mm  16,0  |  2013 x 1069 x 605   1301,9M   68.287MB  34.144MB   104,6 %  | 600/1212/1208/3000  ZU GROSS
  4,25mm  17,0  |  1893 x 1005 x 569   1082,5M   56.780MB  28.390MB    86,9 %  | 600/1210/1206/2993  <- PASST
  4,50mm  18,0  |  1789 x  945 x 537    907,9M   47.619MB  23.809MB    72,9 %  | 600/1200/1204/2998
  5,00mm  20,0  |  1609 x  853 x 481    660,2M   34.627MB  17.313MB    53,0 %  | 600/1206/1192/2992
=> ANTWORT: 4,25 mm. 4,00 mm ist auch mit dem VOLLEN VRAM beider Karten nicht darstellbar
   (104,6 % je GPU). Bei 4,25 mm werden Heikos Abstaende auf +-13 mm getroffen.
   Arbeitsvolumen gegen heute: Zeitschritte fuer 0,5 s -5,9 % (dt skaliert mit dx), Zellen je GPU
   +6,4 % -- das hebt sich naeherungsweise auf.

NEBENBEFUND, der einen eigenen Hebel oeffnet: DIE BALANCE KIPPT.
  Mit den gemessenen Einzel-GPU-Werten (setup.cpp:3005: B70 4648 MLUPs, iGPU 594 MLUPs) kostet je
  grobem Schritt heute das Nahfeld 0,438 ms und das Fernfeld 0,343 ms -- Verhaeltnis 1,28, und der
  Profiler des f4-Laufs bestaetigt das (B70 93,9 %, iGPU 91,0 %, CONCURRENT 96,1 %).
  Bei dx_f 4,25 auf zwei B70 sind es 0,466 gegen 0,286 ms = 1,63 -- die iGPU haette 39 % LEERLAUF.
  Dieser Leerlauf ist Budget fuer eine GROESSERE FERNBOX, und das trifft direkt B14 (Versperrung):
    Fernbox 7,66 x 8,82 m (heute)  169,9M  Versperrung 2,74 %  Balance 1,63
    Fernbox 9,50 x 8,82 m          210,5M              2,21 %          1,31
    Fernbox 11,00 x 8,82 m         243,6M              1,91 %          1,14   <- unter OF13 (1,93 %)
    Fernbox 11,00 x 10,00 m        275,9M              1,68 %          1,00
  Eine Verbreiterung des FERNFELDS auf 11 m brächte die Versperrung erstmals unter das
  OF13-Niveau und kostet nach dieser Rechnung keine Kadenz -- sie fuellt nur den Leerlauf.

VORBEHALTE:
  (1) Die MLUPs sind EINZEL-GPU-Messungen. Was die Aufteilung des Nahfelds auf zwei B70 kostet
      (Halo-Austausch je Schritt, Transfer zwischen den Karten), ist auf dieser Maschine NICHT
      gemessen. Die Tabellen sind Kapazitaet und ideale Skalierung, KEINE Laufzeitprognose.
  (2) Der Halo-Speicher der Domaenenzerlegung ist in den 55 B/Zelle nicht enthalten.
  (3) Die iGPU hat 87.628 MB gemeinsamen Speicher, aber "Buffer Limits 4095 MB global" (Log Z. 142).
      Schon heute liegt der DDF-Block bei rund 7,7 GB, die Aufteilung funktioniert also -- an einer
      groesseren Fernbox wurde sie aber nie geprueft.
  (4) dx_c 17 mm ist kein glatter Wert; setup.cpp:3036 f. warnt ausdruecklich, dass die V1-Masse nur
      bei 16 mm aufgehen. auf_grobe_zelle faengt das ab, die Ist-Abstaende oben sind bereits die
      GESCHNAPPTEN Werte -- aber jede Box-Aenderung muss danach erneut gegen das Log geprueft werden.

### B16b -- ZWEITE ZIELBOX (Heiko, 27.08. abends): 500 / 900 / 1100 / 2800 mm
Gleiche Rechnung wie B16, kleinere Box: x- 500, y+- 900, z+ 1100, x+ 2800 mm ab Fahrzeughuelle.
  Box 7,748 x 3,648 x 2,308 m = 65,23 m3 (heute 32,50 -> Faktor 2,01; Variante A war 82,32 = 2,53).

  dx_f    dx_c  |  Gitter              Zellen   VRAM ges.  je GPU   Auslastung | Ist x-/y/z+/x+
  3,75mm  15,0  |  2069 x  973 x 617   1242,1M   65.151MB  32.576MB    99,8 %  | 500/898/1102/2807  am Limit
  4,00mm  16,0  |  1937 x  917 x 577   1024,9M   53.757MB  26.879MB    82,3 %  | 500/908/1096/2796  <- PASST
  4,25mm  17,0  |  1825 x  861 x 545    856,4M   44.919MB  22.459MB    68,8 %  | 500/904/1104/2804
  4,50mm  18,0  |  1721 x  817 x 513    721,3M   37.834MB  18.917MB    57,9 %  | 500/912/1096/2792

=> ANTWORT: 4,00 mm -- die HEUTIGE Aufloesung bleibt erhalten, bei doppeltem Boxvolumen.
   Auslastung 82,3 % je Karte gegen heute 89,8 %, also sogar etwas entspannter. Heikos Abstaende
   werden auf +-8 mm getroffen. 3,75 mm waere mit 99,8 % rechnerisch gerade noch darstellbar, hat
   aber keinen Puffer -- nicht zu empfehlen, solange der Halo-Speicher der Domaenenzerlegung nicht
   gemessen ist.

DER ENTSCHEIDENDE UNTERSCHIED ZU VARIANTE A -- die Balance bleibt erhalten:
  heute        Nah 508,7M (0,438 ms), Fern 203,5M (0,343 ms)  -> 1,28x, Profiler 96,1 % concurrent
  A @ 4,25mm   Nah 1082,5M (0,466 ms), Fern 169,9M (0,286 ms) -> 1,63x, iGPU 39 % Leerlauf
  B @ 4,00mm   Nah 1024,9M (0,441 ms), Fern 203,5M (0,343 ms) -> 1,29x, iGPU 22 % Leerlauf
  Variante B haelt dx_c bei 16 mm, also bleibt das Fernfeld unveraendert -- gleiche Kadenz, gleiche
  Zeitschrittzahl (dt haengt an dx_f), gleiche Kopplungsqualitaet. Variante A wuerde das Fernfeld
  auf 17 mm vergroebern UND die Balance kippen.

EINORDNUNG GEGEN DEN GEMESSENEN BEDARF (1-m/s-Kriterium aus f4_wandfrei_v2, B11):
  noetig y- 384 / y+ 512 / z+ 640 mm  --  Variante B liefert 908 / 908 / 1096 mm.
  Das ist rund das 2,4-fache in y und das 1,7-fache in z+. Selbst am strengeren 0,5-m/s-Massstab
  (y 2048, z+ 1792 mm) ist z+ zu drei Vierteln und y zu 44 % gedeckt.
  x+ 2796 mm ist nicht durch das Kriterium begruendet (dort ist es schon bei 0 mm erfuellt),
  sondern durch die Nachlaufaufloesung -- siehe den Vorbehalt in B13.

Vorbehalte (1) bis (4) aus B16 gelten unveraendert: die Kosten der Nahfeld-Aufteilung auf zwei B70
sind auf dieser Maschine NICHT gemessen, der Halo-Speicher steckt nicht in den 55 B/Zelle.

### B17 -- NACHLAUF: was er kostet, was er bringt, und die Fassungsfrage zu den Graphen (27.08. abends)

FASSUNG DER GRAPHEN -- Heiko fragt zurecht nach:
  Das ZUERST gezeigte verlauf.png (16:13) stammt aus f4_vollumfang_mls, also aus dem Lauf MIT der
  Einlassstoerung. Das zweite (16:42) aus f4_wandfrei_v2, sauber. Der Unterschied ist gewaltig,
  gerade im Nachlauf: |Mittel| der x+-Serie
     f4_wandfrei_v2    0,10 m/s (min 0,03, max 0,20)
     f4_std_diff2      0,25 m/s (min 0,00, max 0,57)
     f4_vollumfang_mls 3,75 m/s (min 3,10, max 4,44)   <- Faktor 37
  ALLE Bedarfszahlen, auf denen B13/B15/B16/B16b beruhen (y 384/512, z+ 640 mm), stammen aus
  f4_wandfrei_v2 -- die Box-Entscheidung steht also auf den sauberen Daten. Die einzige Aussage
  aus dem gestoerten Lauf war "x- braucht 640-770 mm"; sie wurde bereits zurueckgenommen.

NACHLAUFLAENGEN, in Fahrzeuglaengen (L = 4,448 m):
  Nahfeld heute (x+ 1988 mm)   0,45 L
  Zielbox B (x+ 2800 mm)       0,63 L
  Zielbox A (x+ 3000 mm)       0,67 L
  FERNFELD (bis x 9,610 m)     1,16 L
  OF13 (bis x 17 m)            3,32 L
  => Der Nachlauf lebt schon heute UEBERWIEGEND im Fernfeld. Keine der diskutierten Nahfeldboxen
     aendert daran etwas -- sie verschieben die Uebergabestelle zwischen 0,45 und 0,67 L.

3,75 mm IST ERREICHBAR, ABER NUR UEBER DEN NACHLAUF (Budget 58.636 MB, Variante B waere mit
99,8 % am Limit):
  Variante                          Gitter            Zellen    VRAM    Ausl. | Ist x-/y/z+/x+   Nachlauf
  B unveraendert (2800)             2069x973x617     1242,1M  65.151MB  99,8 % | 500/898/1102/2807  0,63 L  zu gross
  x+ auf 2000                       1853x973x617     1112,4M  58.349MB  89,3 % | 500/898/1102/1997  0,45 L  PASST
  y 750 + x+ 2400                   1961x893x617     1080,5M  56.673MB  86,8 % | 500/748/1102/2402  0,54 L  PASST
  y 800 + z+ 950 + x+ 2400          1961x925x577     1046,6M  54.898MB  84,1 % | 500/808/952/2402   0,54 L  PASST
  Alle drei erfuellen den gemessenen 1-m/s-Bedarf (y 384/512, z+ 640) weiterhin mit Reserve.
  x+ hat AUS DEM KRITERIUM keinen Bedarf -- dort ist |d| schon bei 0 mm Abstand erfuellt. Der
  Nachlauf ist damit rechnerisch die billigste Schrumpfrichtung; ob er physikalisch billig ist,
  ist NICHT gemessen.

WAS ZUR NACHLAUFWIRKUNG BELEGT IST UND WAS NICHT:
  BELEGT: die GESCHWINDIGKEIT im Nachlauf stimmt im sauberen Lauf sehr gut mit OF13 ueberein
  (|Mittel| 0,10 m/s ueber die gesamte x+-Serie bis 2,4 m hinter dem Heck). Ein Feingitter-Nachlauf
  ueber 0,45 L hinaus kauft dort messbar nichts.
  NICHT BELEGT: die Geschwindigkeit ist nicht die Groesse, die Cd bestimmt -- das ist der
  BASISDRUCK auf der Heckflaeche. Dass |u| uebereinstimmt, heisst nicht, dass p es tut; ein
  gemitteltes RANS-Feld und ein aufgeloester LES-Nachlauf koennen dieselbe mittlere Geschwindigkeit
  und verschiedene Druckniveaus haben. Es gibt in diesem Projekt KEINEN A/B, der die
  Nahfeld-Nachlauflaenge gegen Cd/Cz stellt.
  ZU MESSEN, billig: zwei 8-mm-Laeufe, eine Variable CFD_NEAR_LX (das Heck-Ende ist weltfest,
  setup.cpp:3047 f., NEAR_LX verlaengert nur den Nachlauf). Vorbereitet in
  logs/f5_box_serie.txt.4mm als n1_wake_kurz (6,6560) gegen n2_wake_lang (7,4560) -- 0,45 gegen
  0,63 L bei 8 mm. Kriterium: bewegt sich cd_druck_rest um mehr als die Block-SEM, ist der
  Nachlauf im Feingitter relevant und darf NICHT fuer 3,75 mm geopfert werden; bleibt er darunter,
  ist der Weg zu 3,75 mm frei.

### B18 -- ZIELBOX MIT 0,5-L-NACHLAUF: 3,75 mm geht NICHT ganz auf (27.08. abends)
Heiko: Nachlauf auf 0,5 L kuerzen (der Wake lebt ohnehin im Fernfeld), dafuer y auf 1000 und
z+ auf 1200 mm. "Waere dann doch fuer 3,75 mm das Optimum auf Dual-B70 oder?"

WUNSCHBOX x- 500 / y+- 1000 / z+ 1200 / x+ 0,5 L = 2224 mm:
  3,75 mm  1913 x 1029 x 645 = 1269,7M  66.597 MB  102,0 % je Karte  -> ZU GROSS, auch physisch
  4,00 mm  1793 x  965 x 605 = 1046,8M  54.907 MB   84,1 % je Karte  -> PASST, Ist 500/1004/1208/2220
=> Die Antwort auf die Frage ist NEIN: bei 3,75 mm passt die Wunschbox nicht. Die groesste
   3,75-mm-Box im Budget hat y 958 und z+ 952 mm (1913 x 1005 x 577 = 1109,3M, 58.186 MB, 89,1 %).

DIE EIGENTLICHE WAHL, beide mit x- 500 und x+ 0,5 L:
  4,00 mm: y 1004, z+ 1208 mm | 1046,8M | 84,1 % Auslastung | Zeitschritte wie heute
  3,75 mm: y  958, z+  952 mm | 1109,3M | 89,1 % Auslastung | +6,7 % Zeitschritte
  Der Unterschied ist also NICHT "mehr Details gegen nichts", sondern 6,7 % feineres Gitter gegen
  256 mm weniger z+ und 5 Prozentpunkte weniger VRAM-Puffer.

WAS 3,75 mm AN DER WAND WIRKLICH BRINGT -- gemessen, nicht geschaetzt:
  yplus_facetten.csv des 4-mm-Laufs (2.110.775 Facetten): y+ Median 59,4, q10 25,1, q90 137,0.
  y+ skaliert linear mit dx, also Median 59,4 -> 55,7 bei 3,75 mm. BEIDE liegen mitten im
  Wandfunktionsbereich (30 < y+ < 300) -- kein Regimewechsel, keine Aufloesung der viskosen
  Unterschicht. Der Gewinn von 3,75 mm liegt in der GEOMETRIEAUFLOESUNG (Kanten, Spalte,
  Anbauteile), nicht in der Wandbehandlung.

EINORDNUNG gegen den gemessenen Bedarf (1 m/s, f4_wandfrei_v2): y 384/512, z+ 640 mm.
  Beide Varianten liegen weit darueber -- 4,00 mm mit z+ 1208 = 1,9-fach, 3,75 mm mit 952 = 1,5-fach.
  Die z+-Kuerzung ist also nach dem Kriterium unbedenklich; sie kostet Reserve, nicht Substanz.

EMPFEHLUNG: 4,00 mm mit der vollen Wunschbox. Gruende: (1) die Box passt komplett, (2) 84,1 %
lassen Puffer fuer den NICHT gemessenen Halo-Speicher der Domaenenzerlegung -- bei 89,1 % ist der
Puffer duenn, (3) y+ aendert sich kaum, (4) 6,7 % lineare Verfeinerung ist wenig gegen 21 % weniger
z+. Das ist eine Empfehlung, keine Feststellung -- wer die Geometrieaufloesung hoeher gewichtet als
den Rand, entscheidet anders, und beide Varianten sind durchgerechnet.

VORAUSSETZUNG FUER BEIDE: der 0,5-L-Nachlauf ist bisher NUR ueber die Geschwindigkeit belegt.
Der 8-mm-A/B dazu liegt in logs/n_wake_serie.txt (drei Arme 0,45 / 0,63 / 0,81 L, eine Variable
CFD_NEAR_LX). Heiko hat ihn ausdruecklich als 8-mm-Test bestellt -- 4 mm waere zu teuer.

### B19 -- 3,75 mm bei x- 400 / y 900 / z+ 1100 / x+ 0,5 L: JA, knapp (27.08. abends)
  Rohbox 7,072 x 3,648 x 2,308 = 59,54 m3; Grobzellen 472 x 244 x 155 @ 15 mm (cey 244 gerade,
  Paritaet gegen cNy erfuellt); Gitter 1885 x 973 x 617 = 1.131,6 Mio Zellen.
  Ist-Abstaende: x- 400 / y 898 / z+ 1102 / x+ 2217 mm (0,50 L) -- trifft die Vorgabe auf +-7 mm.
  SPEICHER, mit Index-Zuschlag statt nur 55 B/Zelle:
    Zellfelder (55 B, FP16C)                          59.355 MB
    Index-Puffer waechst mit der F-BBox (4,00 B/Zelle,
      Faktor (4/3,75)^3 = 1,214 gegen die 610,8 MB heute)  +130 MB
    Facetten-Geometrie 80 MB -- wandflaechenabhaengig, unveraendert
    SUMME 59.485 MB, je Karte 29.743 MB = 91,1 %
  Heute sind es 29.318 MB = 89,8 %. Der Unterschied ist also 1,3 Prozentpunkte -- gemessen an
  Heikos Vorgabe "genauso auslasten wie der aktuelle Fall" ist das erfuellt. Gegen mein
  rechnerisches Budget (2 x 29.318 = 58.636 MB) liegt es 849 MB darueber; gegen das physische
  Limit 65.310 MB bleiben 2.912 MB je Karte als Puffer.
  DIESER PUFFER IST DIE OFFENE FRAGE: der Halo-Speicher der Domaenenzerlegung ist auf dieser
  Maschine nicht gemessen. 2,9 GB je Karte klingen reichlich, sind aber ungeprueft.

WER GANZ SICHER GEHEN WILL -- drei Varianten, die exakt ins Budget fallen, je 50 mm Verzicht:
  z+ 1050 statt 1100   1885 x 973 x 605  1109,6M  58.203 MB  89,1 %  | y 898  z+ 1057  x+ 2217
  y   850 statt 900    1885 x 949 x 617  1103,7M  57.893 MB  88,6 %  | y 853  z+ 1102  x+ 2217
  x+ 0,45 L statt 0,5  1829 x 973 x 617  1098,0M  57.594 MB  88,2 %  | y 898  z+ 1102  x+ 2007
  Der billigste davon ist z+ 1050: er kostet 45 mm an einem Rand, der mit 1057 mm ohnehin das
  1,65-fache des gemessenen Bedarfs (640 mm) haelt.

GEGENRECHNUNG 4,00 mm mit DERSELBEN Box: 1769 x 917 x 577 = 936,0M, 49.095 MB, 75,2 % je Karte.
  Bei 4 mm bliebe also so viel Luft, dass man die Box noch deutlich vergroessern koennte -- das
  ist die eigentliche Abwaegung: 3,75 mm bei 91 % Auslastung und dieser Box, oder 4,00 mm bei
  75 % und einer groesseren. Beide sind durchgerechnet; die Entscheidung haengt daran, ob
  Geometrieaufloesung oder Randabstand hoeher gewichtet wird -- und am Nachlauf-A/B, der die
  0,5 L ueberhaupt erst rechtfertigen muss (logs/n_wake_serie.txt).

### B20 -- WOHER DIE 64er-REGEL KOMMT, und warum sie fuer die B70 NIE geprueft wurde (27.08. abends)
Heiko: "fuer die iGPU mussten wir x gesamt immer durch 64 teilbar machen. Hat das etwas mit der
Speichernutzung zu tun, profitiert die B70 vielleicht auch davon? Und was ist mit y und z?"

HERKUNFT, nachgelesen. FASSUNG BEACHTEN: die Messung stammt aus dem V1-Projekt
/home/heiko/CFD/FluidX3D, knowledge/hardware.md, Eintrag 2026-06-15 -- ANDERES Projekt, aber
DIESELBE Hardware. In V2 ist sie nachgetragen in BAUPLAN-KOPPLUNG.md:299.
  A/B mit EINER Variablen, gleiche Sitzung, auf der Xe-iGPU:
    Nx  937 (prim) = 3,79 ns je Zelle, effektive Bandbreite ~40 GB/s
    Nx 1024 (2^10) = 3,20 ns je Zelle, ~48 GB/s
    -> 1024 ist 8 % SCHNELLER, obwohl es 9 % MEHR Zellen sind.
  Deutung dort: 937 erzeugt Partition-Camping auf den System-RAM-Kanaelen der iGPU; 1024 ist
  memory-aligned. Regel seither: Coarse-Nx immer durch 64 teilbar, nie prim oder ungerade.
  Heute erfuellt: cNx = 768 = 12 x 64.

WAS DIE V1-NOTIZ AUSSERDEM SAGT -- und was heute nicht mehr gilt:
  Woertlich: "Die Fine-X-Laenge (1789/1873) ist perf-irrelevant (versteckt)" und "Lever sitzt auf
  der iGPU, NICHT auf der Fine". Der Grund steht daneben: die iGPU war damals mit 100 % Duty-Cycle
  der saturierte Engpass, die B70-Fine verschwand darunter. Deshalb wurde das Nahfeld-Alignment
  NIE gemessen -- es war schlicht egal.
  HEUTE IST DAS ANDERS: der Profiler des f4-Laufs meldet B70 93,9 % / iGPU 91,0 % / concurrent
  96,1 % -- beide nahe an der Saettigung. Und in der Dual-B70-Rechnung (B16) kippt es vollends:
  Nahfeld 0,466 ms gegen Fernfeld 0,286 ms je grobem Schritt. Das Nahfeld wird der Engpass, und
  damit wird sein Alignment zum ersten Mal relevant.

DER STRUKTURELLE HAKEN (setup.cpp:3096): fNx = (cex-1)*ratio+1 ist bei geradem ratio IMMER
UNGERADE. 1689 heute, 1885 in der 3,75-mm-Planung, 1793 / 1913 in den anderen Varianten -- keine
davon ist durch 64 teilbar, keine kann es sein. Das Nahfeld kann die Regel nicht erfuellen,
solange die Deckungspunkt-Konvention gilt (und die ist physikalisch begruendet, setup.cpp:3079).
Waere der Effekt auf der B70 so gross wie auf der iGPU, laege dort ein zweistelliger Perf-Hebel
brach -- er braeuchte aber einen Layout-Umbau (Padding des Speicher-Nx gegen das Gitter-Nx),
keine Env-Aenderung.

ZU y UND z, theoretisch: die Threads laufen in x (n = x + Nx*(y + Ny*z)), also entscheidet Nx
ueber Coalescing und Workgroup-Ausrichtung. Ny und Nz beeinflussen das nicht direkt; sie gehen
nur in die Strides Nx und Nx*Ny ein. Ist Nx durch 64 teilbar, ist Nx*Ny es automatisch auch.
Erwartung daher: y/z zweitrangig. GEMESSEN IST DAS NICHT -- und die Gegenthese ist nicht absurd,
denn beim klassischen Partition-Camping ist ein ungerader Stride sogar erwuenscht. Genau deshalb
gehoert y in den Test.

VORBEREITET: logs/p_align_serie.txt, fuenf Arme, je eine Variable. Pruefstand ist
CFD_CASE=fernfeld -- ein EINZELGITTER (setup.cpp:5669 ff.) ohne Deckungskonvention, Nx frei ueber
CFD_FAR_LX, und es laeuft ueber den Standard-Konstruktor (Zeile 5735, kein dev-Argument) auf der
B70. Der einzige Fall im Projekt, in dem sich Nx unabhaengig variieren laesst.
  p1_align_x1024  1024 x 480 x 552  271,3M   x aligned (16x64), Basis
  p2_align_x1025  1025 x 480 x 552  271,6M   x +1, ungerade
  p3_align_x1021  1021 x 480 x 552  270,5M   x prim -- der V1-Fall
  p4_align_y512   1024 x 512 x 552  289,4M   y aligned (8x64)
  p5_align_y481   1024 x 481 x 552  271,9M   y ungerade
ABNAHME: ns je Zelle aus den MLUPs im Log, NICHT die Gesamtlaufzeit (die Arme haben verschiedene
Zellzahlen). p1 gegen p2/p3 beantwortet x, p1 gegen p4/p5 beantwortet y. Kosten: rund 30 s je Arm
bei T_END 0,02 s (500 Schritte a 271 Mio Zellen bei 4648 MLUPs), also gut zwei Minuten fuer alles.
EIN NULLBEFUND IST HIER GENAUSO WERTVOLL wie ein Treffer: er schliesst einen teuren Layout-Umbau
aus, der sonst als "muesste man mal machen" stehen bliebe.

### B21 -- BUCHUNGSSCHLUSS ABGENOMMEN: A1-A4 bestanden (27.08. abends, nach dem Middleware-Fix)
Middleware entsperrt (harte Falle jetzt weich; der Repo-jq-Guard uebernimmt die Erzwingung und
lehnte prompt einen Aufruf ohne run_in_background ab -- genau die gewuenschte Arbeitsteilung).
Serie logs/b2s1_abnahme_serie.txt (5 Arme, iGPU) + logs/b2s1_a4_serie.txt (8 mm, B70), Binary
aus Commit 2a50a7d, Census vor und nach beiden Serien sauber.

A1 BITANKER -- BESTANDEN. Beide kipp0-Arme (ELIBB 1 und 0): FELD-HASH(u) = 4722579264326613690
  identisch, Reibung x = 0,01639657 ziffernidentisch (Verhaeltnis 1,0043), Slot 69 = 0 == Soll.
  Der Buchungsschluss ist am ungekippten Kanal ein exakter No-Op.

A2 K2 -- BESTANDEN, mit einer Einschraenkung beim 45-Grad-Arm:
    26 Grad:  K2 -7,4189 -> 1,0004   Slot 69 = 118.657.473 == Soll (13+15+64+10+16)
    45 Grad:  K2 +1,1267 -> 1,0109   Slot 69 =  78.719.996 == Soll
  Der 45-Grad-Lauf meldet rc=1: der EINGEBAUTE Waechter verlangt <1 % Abweichung, 1,0109 sind
  1,09 %. Kein Absturz, kein Rechenfehler -- der Arm ist als Abnahmelauf disqualifiziert, die Zahl
  gilt. Die Buchungsidentitaet haelt in ALLEN Armen exakt.

A3 KUGEL -- BESTANDEN. ELIBB[67] = 26825 == Soll, Slot 69 = 10.746 == Soll (exakt der erwartete
  Wert), n_voll 500 / projiziert 545 / unklar 0, und Cd_reibung = +0,4148 -- POSITIV.

A4 FAHRZEUG 8 mm (B70) -- BESTANDEN, das ist der eigentliche Gewinn:
                          cd_druck   cd_rest   cz_rest   cd_reib
    vorher (s4_mls_g17)    +1,6628   +1,0234   -0,6598   -0,0684   (Schub, physikalisch unmoeglich)
    A4 (b2s1_8mm_g17)      +1,6868   +1,0472   -0,6613   +0,0791   (Widerstand)
  Der Druckpfad bewegt sich um 1,4 bis 2,3 %, die REIBUNG kippt das Vorzeichen. Slot 69 =
  62.189.922 == Soll.

DAMIT IST AUFTRAG 3A ERSTMALS AUSWERTBAR (die Vorbedingung aus R3 ist erfuellt):
    FX 8 mm   Druck 1,0472 + Reibung 0,0791 = 1,1263   Reibungsanteil 7,02 %
    OF13      Druck 0,5590 + Reibung 0,0420 = 0,6010   Reibungsanteil 6,99 %
  Der ANTEIL stimmt auf 0,02 Prozentpunkte. Die Frage des Auftrags -- "liegt die FX-Reibung
  deutlich ueber 7 %, dann rueckt der 26-Grad-Pfad hoch" -- ist damit beantwortet: NEIN. Druck und
  Reibung sind mit 1,87x bzw. 1,88x im GLEICHEN Verhaeltnis zu hoch; die Druckseite bleibt der
  Hauptposten. (8-mm-Sprosse, nicht mit den 4-mm-Zahlen vergleichbar.)

DER BEFUND, DER ZAEHLT -- die Physik hat sich NICHT bewegt:
    u_tau IST/Ziel   Ebene 0,716 | 26 Grad 2,382 | 45 Grad 1,633
  Das sind exakt die Werte von vor dem Buchungsschluss. K2 misst seither nur noch
  Buchungsvollstaendigkeit -- genau die Selbsttaeuschung, vor der der Planungsagent gewarnt hat,
  bestaetigt auf drei Nachkommastellen. Der Buchungsschluss hat das INSTRUMENT repariert, nicht die
  Wand: die 26-Grad-Klasse meldet unveraendert 2,4-fache Schubspannung. Gewonnen ist Messbarkeit --
  die frueheren -7,4189 waren zu 100 % Buchungsartefakt und haben jeden Physikbefund darunter
  verdeckt.
  => Stuetzt die Empfehlung, Schritt 2 NICHT als m6+m7-Cluster zu bauen (Deckel u_tau 1,5-1,7),
     sondern den linkmengen-bewussten Abtastfaktor vorzuziehen.

WIRKPFAD-ABNAHME DER ZENSUS-DIAGNOSTIK (B8) -- BESTANDEN, die Zeile feuert:
  "FACETTEN-ZELLKLASSEN der Kraftschleife: 8.096.527 Markerzellen -- unbehandelt (voll) 7.506.074
   (92,71 %), projiziert 583.875 (7,21 %), UNKLAR 6.578 (0,08 %)."
  GATE-VORZEICHEN fuer Auftrag 2 (Mehrfachfacetten): geometrisch sind es 0,08 %, also Promille.
  Das KRAFTGEWICHT fehlt noch -- der Lauf lief im GPU-Pfad, und dort weist der Report das
  ehrlich aus. Ein Lauf mit CFD_FAC_GPU=0 liefert es nach. Der Vorbehalt aus B3 gilt: eine kleine
  Zellzahl kann viel Kraft tragen (das z-Band traegt mit 0,25 % der Zellen 30 % des Druck-Cd).

### B22 -- ENV-TOR ENTSCHIEDEN: der Abtastfaktor ist tot, ELIBB ist der Hebel (27.08. spaetabends)
Serie logs/t2_tor_serie.txt, 5 Arme am 26-Grad-Kanal (N=20, iGPU), Basis woertlich aus
logs/b2s1_abnahme_serie.txt Zeile 5, je EINE Variable. Binary mit dem Nullziel-Fix (siehe unten).
Census vor und nach der Serie sauber.

  Arm            u_tau IST/Ziel   Gate/Klemme (Slot 10)   Wirkpfad
  t2_26_u10          2,382            64.144.958          159.586.740
  t2_26_u15          2,222            64.125.980          159.586.740
  t2_26_u30          2,269            63.825.129          159.586.740
  t2_26_elibb0       1,943            21.128.937          159.586.740
  t2_26_null         0,741            74.656.440          159.586.740

KONTROLLE BESTANDEN: t2_26_u10 reproduziert b2s1_kipp26 ziffernidentisch (2,382 / K2 1,0004 /
Gate 64.144.958). Der Default von CFD_FAC_UTKORR ist damit belegt 1,0, die Basislinie stimmt,
und der Kernel-Fix hat an diesem Arm nichts veraendert.

BEFUND 1 -- DER ABTASTFAKTOR IST TOT. Die Antwort auf UTKORR ist WEDER monoton steigend (Prognose
des Planungsagenten: 2,9 bis 3,7) NOCH fallend Richtung 1,0, sondern FLACH mit flachem Minimum:
eine Verdreifachung bewegt u_tau um 6,7 %, ein Teil davon zurueck in die falsche Richtung.
Die Gate-Rueckfaelle bleiben dabei bei 64,14 / 64,13 / 63,83 Mio -- der Anwendungsanteil aendert
sich um 0,5 %. Der Faktor kommt schlicht nicht durch: was das Modell an den 40 % anwendenden
Besuchen mehr einspeist, schneidet das SATGATE sofort wieder ab.
Der Sweep 1,0/1,5/3,0 klammert den GESAMTEN Mechanismus ein, weil die Formel k = 1/(1-G11) je
Klasse genau 1,00 (m2) / 1,50 (m1) / 3,00 (m0) liefert. Innerhalb dieser Klammer passiert nichts.
=> KILL-KRITERIUM GEZOGEN: fuer den linkmengen-bewussten Abtastfaktor wird KEIN Code geschrieben.

BEFUND 2 -- ELIBB IST DER GROESSTE EINZELHEBEL, DEN DIE MESSUNG KENNT:
  ELIBB=1 (heute):  u_tau 2,382, Gate 64.144.958 (40,2 % der Besuche)
  ELIBB=0:          u_tau 1,943, Gate 21.128.937 (13,2 % der Besuche)
  ELIBB verdreifacht die Gate-Rueckfaelle und verschlechtert u_tau um 23 %. Der Wert 1,943
  reproduziert die aeltere g9-Messung (1,94) exakt -- der Effekt ist reproduzierbar und alt.
  Das ist ein echter ZIELKONFLIKT: ELIBB ist die q-abhaengige Wandbehandlung, die an der
  gekruemmten Wand (Kugel) gebraucht wird; an der TREPPE kostet sie Wandtreue. Der
  4-mm-Produktionslauf f4_vollumfang_mls faehrt ELIBB=1, obwohl der Default AUS ist.

BEFUND 3 -- DAS ZIEL-INTERVALL GILT AUCH AUF HEUTIGEM CODE: Nullziel liefert 0,741 (frueher
0,706 auf altem Stand). [0,741 ... 2,382] enthaelt 1,0 -- die 26-Grad-Klasse ist ziel- und
zustandsbegrenzt, nicht durch die Geometrie festgenagelt. Die These des vorigen Planungsagenten
traegt also, nur ist der Abtastfaktor nicht der Weg dorthin.

FOLGE -- der naechste Hebel ist der ANWENDUNGSANTEIL, nicht die Zielhoehe:
  bei 40 % anwendend (ELIBB=1) -> 2,382 | bei 87 % (ELIBB=0) -> 1,943 | Nullziel 53 % -> 0,741
  Zu messen, wieder Env-only: CFD_FAC_BUDGET (0,25...4, setup.cpp:1647) greift DIREKT an Slot 10
  und ist laut eigener Codeansage "Design, nie geeicht" (setup.cpp:1649). Dazu die
  ELIBB-Wechselwirkung: BUDGET-Sweep je einmal mit ELIBB 1 und 0.

### B23 -- KERNEL-FIX: Nullziel-Arme sprangen aus der Funktion (27.08. spaetabends)
kernel.cpp:2027 trug "const float zi_ = def_fac_tau*twe; if(zi_<=0.0f) return;". Das return
verliess die GANZE Funktion apply_facette_imem, nicht nur den Histogrammblock, in dem es steht.
In einem Nullziel-Arm (def_fac_tau = 0) wurden dadurch fuer jede 64. Facette auf jedem 100.
Schritt Solve, Pass 2 UND die phi-Buchung uebersprungen -- obwohl der Wirkpfadzaehler oben bereits
gezaehlt hatte. Also eine stille Feldaenderung PLUS ein Ist!=Soll in der Buchung, ausgerechnet in
dem Arm, der die Untergrenze des Ziel-Intervalls messen soll. Gefunden vom Planungsagenten.
FIX: zwei exakte Ein-Block-Ersetzungen, "if(zi_<=0.0f) return;" -> "if(zi_>0.0f) { ... }". Fuer
alle Arme mit def_fac_tau > 0 ist der Zweig unveraendert -- t2_26_u10 belegt das ziffernidentisch
gegen b2s1_kipp26. JIT-Kurzlauf bestanden (der Arm lief sauber durch, rc-Fehler stammt vom
K2-Waechter, nicht vom Kernel).

### B24 -- iGPU-SKALIERUNG: der fruehere Einbruch ueber der Fallgroesse existiert NICHT MEHR
Heiko-Frage 27.08. spaetabends: "bricht die iGPU je nach Fallgroesse immer noch so ploetzlich ein?"
Serie logs/s_igpu_skala_serie.txt, 12 Arme, CFD_CASE=fernfeld (Einzelgitter, feste Boxgroesse),
NUR CFD_DX variiert -> Zellzahl ~ dx^-3. Geraet: iGPU (CFD_QUEUE_DEV Default 2). Census sauber.

  Zellen      ns/Zelle        Zellen      ns/Zelle
   25,6M       1,828          117,8M       1,821
   47,6M       1,832          134,0M       1,821
   74,4M       1,845          175,7M       1,821
   92,7M       1,845          203,5M       1,815
  104,4M       1,835          278,7M       1,818
  110,8M       1,842          397,4M       1,828

ERGEBNIS: ueber einen Faktor 15,5 in der Zellzahl liegt die Spannweite bei 1,7 % -- das ist
Messrauschen, kein Trend und kein Sprung. KEIN Einbruch.

DIE GEPRUEFTE HYPOTHESE FAELLT EBENFALLS: die iGPU meldet "Buffer Limits 4095 MB global"
(Log Z. 142); der DDF-Block ist 38 B je Zelle (FP16C), die Grenze liegt also bei 113,0 Mio
Zellen. Die Leiter hat sie mit zwei dichten Punkten eingeklammert -- 110,8M (4017 MB, darunter)
gegen 117,8M (4268 MB, darueber): 1,842 gegen 1,821 ns/Zelle. Der groessere Fall ist sogar
minimal SCHNELLER. Ein Buffer-Limit-Sprung existiert nicht.

EINORDNUNG gegen die V1-Messung (FluidX3D, knowledge/hardware.md, 15.06.2026 -- ANDERE Fassung,
gleiche Hardware): dort 3,20 ns/Zelle im besten Fall (Nx 1024) und 3,79 im schlechtesten (Nx 937).
Heute liegt der SCHLECHTESTE Punkt der ganzen Leiter bei 1,845. Die iGPU ist also rund
1,8-fach schneller geworden UND gutmuetig ueber die Groesse. Welcher Treiber-, Kernel- oder
Codestand das gebracht hat, sagt diese Messung nicht -- sie sagt nur, dass die alten
Vorsichtsmassnahmen (Nx durch 64, grosse Faelle meiden) heute nichts mehr kosten und nichts
mehr bringen.

ZUSAMMEN MIT B20 (Alignment, Spannweite 1,7 % ueber x aligned/ungerade/prim und y aligned/ungerade)
ergibt das ein konsistentes Bild: die frueheren iGPU-Eigenheiten sind auf dem heutigen Stack nicht
mehr messbar. Beide Nullbefunde sind wertvoll -- sie nehmen zwei Nebenbedingungen aus der
Box-Planung heraus, die bisher als gesetzt galten.

### B25 -- KORREKTUR ZU B22 + der ELIBB-Konflikt ist ENV-TRENNBAR (27.08. spaetabends)
Ein Untersuchungsagent hat den Mechanismus auseinandergenommen. Zwei Ergebnisse: ein Fehler in
MEINER Dokumentation, und ein Weg aus dem Zielkonflikt.

KORREKTUR ZU B22 (Befund des Agenten, nachgerechnet und bestaetigt): die dort genannten
Anwendungsanteile "40 % / 87 % / 53 %" vermischen zwei Definitionen -- die 40 % waren Slot 10 als
RUECKFALLanteil, die 87 % waren 100 % minus Slot 10 als ANWENDUNGSanteil, und beide ignorieren
die Slots 13 und 16. Richtig ist 1 - Slot69/Wirkpfad:
    ELIBB=1   Slot 69 = 118.657.473  ->  25,6 %   (nicht 40 %)
    ELIBB=0   Slot 69 =  85.078.217  ->  46,7 %   (nicht 87 %)
    Nullziel  Slot 69 = 129.758.580  ->  18,7 %   (nicht 53 %)
Der ELIBB-Effekt ist damit GROESSER als in B22 beschrieben (er halbiert den Anwendungsanteil),
aber die Folgerung kippt: der Nullziel-Punkt ist nicht der mittlere, sondern der NIEDRIGSTE.
Die Monotonie "mehr Anwendung -> weniger u_tau", auf der der in B22 vorgeschlagene naechste
Schritt aufbaute, existiert in den eigenen Zahlen NICHT (25,6 % -> 2,382; 46,7 % -> 1,943;
18,7 % -> 0,741). Der Buchungswaechter selbst rechnet in allen drei Armen korrekt.

DER MECHANISMUS, jetzt verstanden: die 26-Grad-Treppe zerfaellt in DREI gleich grosse Zellklassen
(je 10.620) mit y_w 0,187 / 0,698 / 1,069 und 8 / 4 / 1 Wandlinks -- keine q-Streuung, sondern
drei diskrete Werte. Die Klasse mit 1 Link sitzt dauerhaft im Slot-13-Rueckfall, wo iMEM unter
ALPHA2 beweisbar nie wirken kann (erreichbarer Unterraum {0}); die mit 4 Links haengt dauerhaft am
Schur-Skalarrueckfall (Slot 14), dessen Akzeptanzschwelle Gt11 >= 1e-4*G11 eine Verstaerkung von
bis zu 1e4 auf R1 zulaesst -- direkt vor dem harten Gate. Genau auf diese beiden Klassen legt
ELIBB seine STARKE Blende (q 0,789/0,792, chi 0,58), waehrend die vollrangige 8-Link-Klasse nur
den q<0,5-Zweig bekommt. Gemessene Folge: mittleres Modell-tau_w 2,39e-6 -> 14,42e-6 (Faktor 6,0),
y+ 87,8 -> 218,2.
GEGENPROBE, die den Mechanismus beweist: bei 45 Grad ist die MLS-Aktivitaet fast DOPPELT so hoch
(87,5 % der geblendeten Links gegen 50,0 %), Slot 10 aber 0,098 % statt 40,2 % -- dort gibt es
keine rangdefiziente Klasse (Slot 14 = 0) und die starke Blende landet auf der folgenlosen
1-Link-Klasse. Nicht ELIBB als solches ist der Taeter, sondern das Zusammentreffen.

DER AUSWEG -- die q-Baender sind fast disjunkt:
    Kugel D=40:      max q < 0,7362, Anteil im Schadensband  0,0 %
    Fahrzeug 8 mm:                                           1,38 %
    Fahrzeug 4 mm:                                           1,38 %
    kipp26:          q = 0,789/0,792                        40,0 % der geschnittenen Links
Eine q-Schwelle in [0,7362 ... 0,7894) trennt beide Faelle exakt. CFD_FAC_QKAPPE (lbm.cpp:600,
Default 1,0) ist genau dieser Schalter -- ENV-ONLY. Warnung dazu: an der Kugel ist die Kappe
hochsensibel (QKAPPE=0,65 halbierte dort den Druckwiderstand), 0,75 muss dort per Startprotokoll
als echter No-Op belegt werden ("q>Kappe->BB: 0").
Zweiter Env-Kandidat, den B22 uebersehen hat: CFD_FAC_LSQ=1 (kernel.cpp:2186) ersetzt s1 = R1/Gt11
durch die Kleinste-Quadrate-Loesung und entschaerft damit die 1e4-Verstaerkung SELBST. Der
Kernelkommentar nennt als Motiv woertlich die 26-Grad-Wand.

EMPFOHLENE REIHENFOLGE (alles Env, ~10 min je Arm am 26-Grad-Kanal):
  T0  Nullziel x ELIBB=0 -- der fehlende vierte Quadrant, schliesst die einzige Verwechslung
  C   QDIAG 2 und 3 auf dem MLS-Binary -- lokalisiert den Zweig (die alte Messung dazu stammt
      vom K1'-Stand vor dem MLS-Umbau und ist NICHT uebertragbar)
  B   LSQ=1 -- entschaerft den Verstaerker
  A   QKAPPE=0,75 -- trennt die Geometrien
Erst danach BUDGET, und Code erst, wenn C und B die Zuordnung bestaetigt haben.

EHRLICHE EINSCHAETZUNG des Agenten, die ich teile: fuer den 4-mm-PRODUKTIONSLAUF ist der Konflikt
mit hoher Wahrscheinlichkeit operativ aufloesbar (1,4 % der Links betroffen, Kugel unberuehrt).
Fuer die 26-Grad-Klasse als PHYSIKPROBLEM ist er es nicht -- ein Drittel der Zellen (1 Wandlink)
liegt prinzipiell ausserhalb dessen, was iMEM loesen kann.

### B26 -- CCS-KANARIE, sanfte Probe: PASS (27.08. spaetabends)
werkzeuge/ccs_kanarie/ccs_kanarie 1 45 3072 auf der B70 (Device 1). 27.950 MB in 8 Bloecken
belegt von 31.023 MB, Restluft 3.073 MB. Sofortkontrolle 0 fehlerhafte Worte, nach 45 s
Endkontrolle 0 fehlerhafte Worte. VERDIKT PASS.
EINSCHRAENKUNG, die das Werkzeug selbst ansagt: mit Reserve ist NICHT garantiert, dass die
oberste VRAM-Seite in unseren Puffern liegt -- und genau dort saesse der Xe-Flat-CCS-Fehler
(Aufrundung der CCS-Basis auf 128 KiB). Der sanfte Modus ist eine Vorab-Probe, kein Verdikt.
Das volle Verdikt braucht reserve_mb=0, also VRAM-Erschoepfung auf der DESKTOP-GPU -- das birgt
Freeze-Risiko und wird Heiko vorgelegt statt selbst entschieden.

### B27 -- t3-SERIE AUSGEWERTET: die Aufloesungs-These ist WIDERLEGT (28.08. morgens)
Serie logs/t3_elibb_serie.txt, 5 Arme am 26-Grad-Kanal, gelaufen 27.08. 22:48-23:37, Binary
bada4ed. Alle Kill-Kriterien waren VOR dem Lauf in der Seriendatei formuliert.

WIRKPFADE ZUERST -- alle drei Schalter haben nachweislich gegriffen, und zwar exakt in der
vorhergesagten Groesse:
  QKAPPE=0,75:  "q>Kappe->BB: 42480"     (Prognose 42.480 -- getroffen)
  LSQ=1:        Slot 65 = 53.195.580     (Prognose ~53,2 Mio -- getroffen)
  QDIAG=2:      Diagnosearm-Ansage vorhanden
Die Arme haben also getan, was sie sollten. Das Ergebnis ist trotzdem negativ.

ERGEBNIS, nach u_tau IST/Ziel sortiert (Ziel 1,000):
  Arm                u_tau   Gate(Slot 10)  Anwendung   tau_w [1e-6]    y+
  Nullziel+ELIBB0    0,761       9.295.740     55,5 %       0,4573    40,78
  ELIBB=0 (Basis)    1,943      21.128.937     46,7 %       2,3946    87,78
  QDIAG=2            2,150      68.866.776     22,7 %       2,2165    99,18
  QKAPPE=0,75        2,219      69.154.033     22,5 %       2,2102    99,04
  ELIBB=1 (Basis)    2,382      64.144.958     25,6 %      14,4248   218,16
  LSQ=1              2,455      17.228.465     52,3 %      15,6211   227,00
  QDIAG=3            2,459      58.946.622     29,2 %      38,2225   349,81

KILL-KRITERIEN, formell gezogen:
  A (QKAPPE) -- KILL. Kriterium war "Slot 10 unter 30 Mio ODER u_tau unter 2,2". Gemessen
     69.154.033 (HOEHER als die Basis) und 2,219. Die q-Trennung ist als Hebel tot, obwohl die
     Kappe exakt die vorhergesagten 42.480 Links erwischt hat.
  C1 (QDIAG=2) -- KILL. Kriterium war "Slot 10 in die Naehe von 21 Mio". Gemessen 68.866.776.
     Der m1/m2-Pfad traegt die Schuld also NICHT allein.
  B (LSQ) -- Zuordnung BESTAETIGT (Slot 10 faellt um 46,9 Mio, weit ueber der 10-Mio-Schwelle,
     Slot 65 trifft die Klassengroesse exakt), aber u_tau wird SCHLECHTER: 2,382 -> 2,455.
  T0 (Nullziel x ELIBB=0) -- MECHANISMUS BESTAETIGT: Slot 10 faellt von 74.656.440 (ELIBB=1)
     auf 9.295.740, also Faktor 8. ELIBB hebt P1 tatsaechlich massiv.

DER EIGENTLICHE BEFUND -- DIE GATE-THESE IST WIDERLEGT, UND ZWAR ANTIKORRELIERT:
  LSQ hat den zweitbesten Gate-Wert (17,2 Mio) und den SCHLECHTESTEN u_tau (2,455).
  QDIAG=2 hat den schlechtesten Gate-Wert (68,9 Mio) und den zweitbesten u_tau (2,150).
  Ueber alle sieben Arme: r(u_tau, Gate) = +0,58, r(u_tau, Anwendungsanteil) = -0,56.
  "Weniger Gate-Rueckfaelle = bessere Wandtreue" ist damit als Arbeitshypothese erledigt --
  und mit ihr der in B22 vorgeschlagene BUDGET-Sweep als NAECHSTER Schritt (er greift genau am
  Gate). Er bleibt als Messarm interessant, aber nicht als Hebel.

WAS STATTDESSEN TRAEGT -- das Modell ueberzieht, und das Ziel ist der Hebel:
  Beide ELIBB=0-Arme klammern die Loesung ein:  Nullziel 0,761  |  Vollziel 1,943.
  1,000 liegt DAZWISCHEN, bei gleichem ELIBB-Zustand und gleicher Geometrie. Das bestaetigt die
  Ziel-/Zustandsbegrenzung zum dritten Mal, jetzt auf dem heutigen Code und mit sauberem Paar.
  s_fac_tau ist heute BINAER (setup.cpp:1646/2285: fc==2||fc==4 ? 0 : 1) -- ein Zwischenwert ist
  ohne Codeaenderung nicht messbar. Eine Env-Skala waere EINE Zeile und wuerde die Antwortkurve
  zwischen 0,761 und 1,943 erschliessen. WICHTIG: der Zweck ist der BEFUND "um welchen Faktor
  ueberzieht das Modell", nicht eine Eichkonstante -- ein aus der Kurve abgelesener Zielfaktor
  waere ein Handwert und faellt unter die Handwert-Regel.

STAND DER 26-GRAD-KLASSE nach zwei Tagen: der beste erreichbare Wert bleibt ELIBB=0 mit 1,943.
Kein Schalter, keiner der drei Aufloesungs-Kandidaten und keine der beiden geplanten Baumassnahmen
hat u_tau naeher an 1,0 gebracht. Was gewonnen wurde, ist die Ausschlussliste.

### B28 -- HEIKOS FENSTER-IDEE TRIFFT: u_tau 2,382 -> 1,507 am 26-Grad-Kanal (28.08. morgens)
Heiko: "koennen wir nicht mehrere zellangrenzende Facetten pro Zelle nutzen, waere das nicht
genauer?" -- die Haelfte davon ist ohne Code messbar: CFD_FACETTEN_FENSTER (setup.cpp:1260)
steuert den Radius der Nachbarschaft, ueber die die TLS-Ausgleichsebene gefittet wird
(1 = 3^3 Default, 2 = 5^3, 3 = 7^3). Er aendert NORMALE und y_w, NICHT die Linkmenge des
iMEM-Solves. Serie logs/t4_fenster_serie.txt, je eine Variable, Binary 22496f7.

DER ANLASS, vorab gerechnet: die Treppe hat Steigung 1:2, die WAHRE Rampennormale liegt bei
atan(1/2) = 26,565 Grad. Der 3^3-Fit misst 28,168 Grad -- 1,60 Grad daneben, bei allen drei
Zellklassen identisch. Das ist Treppen-Diskretisierung, kein Rauschen.

  Fenster   Winkel   Winkelfehler   y_w Median            u_tau   tau_w[1e-6]     y+   Slot 14
  3^3        28,2     +1,63          0,187/0,698/1,069    2,382     14,4248     218,2  53.195.580
  5^3        27,2     +0,63          0,647                1,507      0,7570      52,3  43.608.670
  7^3        19,1     -7,46          1,190                ABBRUCH       -          -        -

DAS 5^3-FENSTER IST DER GROESSTE EINZELHEBEL, DEN DIESE KLASSE BISHER GESEHEN HAT:
u_tau IST/Ziel faellt von 2,382 auf 1,507 -- besser als ELIBB=0 (1,943), besser als alles aus
der t2- und t3-Serie. In der ECHTEN Kraftbilanz (u_tau IST kommt aus f_akt, nicht aus dem
Modell) entspricht das Faktor 2,50 weniger Wandschubspannung.
Der Mechanismus passt zur Erklaerung: der Winkelfehler faellt von 1,63 auf 0,63 Grad, die drei
diskreten y_w-Werte verschmelzen zu einem Median 0,647 -- die Klassentrennung, die das ganze
Problem traegt, wird weicher. Slot 14 (Rang-2-Rueckfall) faellt von 53,2 auf 43,6 Mio.

DAS 7^3-FENSTER IST KAPUTT -- auch das ein Befund: der Fit wird so grob, dass der Winkel auf
19,1 Grad einbricht (7,5 Grad UNTER der Wahrheit) und y_w auf 1,190 wandert; danach verwirft die
Klassifikation alles und der Lauf endet mit "alloc_facetten_domain: keine aktive Facette (alle
markiert?)". Der Code faengt das sauber ab. Es gibt also ein Optimum, und der ganzzahlige Radius
trifft es nicht genau -- bei 5^3 bleiben 0,63 Grad Fehler.

OFFEN UND ZU KLAEREN: K2 steht im 5^3-Arm bei 1,02 (Waechter ausgeloest, rc=1) gegen 1,0004 bei
3^3. Das ist eine Buchungsabweichung von 2 %, kein Absturz -- aber sie war bei 3^3 nicht da und
muss erklaert werden, bevor der Arm als Gewinn zaehlt.

GEGENPROBEN LAUFEN (logs/t5_gegenprobe_serie.txt), denn ein groesseres Fenster kann auch ECHTE
Geometrie verschmieren statt nur kuenstliche Stufen:
  g1 kipp0 -- an der ebenen Wand ist die Normale trivial richtig, das Fenster kann nur schaden.
     KILL: Verhaeltnis mehr als 1 % schlechter oder u_tau unter 0,68.
  g2 kipp45 -- wahre Normale 45,000 Grad. Naehert sich der Winkel UND faellt u_tau, ist der
     Mechanismus geometrisch bestaetigt und nicht auf 26 Grad zugeschnitten.
  g3 Kugel -- der eigentliche Gegenfall: keine Stufen zum Wegmitteln, nur echte Kruemmung.
     KILL: Cd_druck bewegt sich mehr als 5 % (Referenz 0,6648).
Erst wenn g1 und g3 neutral bleiben, ist das ein Hebel und kein Kompromiss.

### B29 -- VORAB-PROGNOSE zu g4 (Fenster 5^3 am Fahrzeug), vor dem Lauf festgehalten
Der Quelltext (setup.cpp:1256-1259) traegt eine Fenster-A/B vom 2026-08-15 am Fahrzeug 8 mm mit
gegenteiligem Verdikt zu B28: "3^3 gewinnt klar -- r21-q90 0,21/0,27/0,42 und Orientierungs-
konflikte 2,7/5,4/9,0 % fuer 3/5/7". Das ist kein direkter Widerspruch: jene A/B entschied nach
FIT-GUETE, B28 nach u_tau. Beides kann zugleich wahr sein.
PROGNOSE fuer g4, damit sie widerlegbar ist: wenn die alte A/B recht hat und groessere Fenster
"Kruemmung und Zweitflaechen" sehen, muss am Fahrzeug K2(Kante) STEIGEN (Basis 95.953 = 12,7 %)
und Orientierung ebenfalls (Basis 23.034 = 3,0 %, alte A/B sagt 2,7 -> 5,4 %). Bleiben beide
gleich, ist die alte Begruendung am Fahrzeug nicht reproduzierbar.

### B30 -- HEIKOS FRAGE BEANTWORTET: 29,3 % der wandnahen Zellen bekommen keine Facette
Zensus f8_standard_final (Fahrzeug 8 mm, Fenster 3^3, YWMIN-Default 0,2), 755.344 wandnahe
Fluidzellen:  K1 0 | K2(Kante) 95.953 (12,7 %) | K3 0 | K4(y_w) 100.891 (13,4 %) |
Orientierung 23.034 (3,0 %) | Punktueberlauf 0 | bewegte-Wand-Naehe 1.274 (0,2 %)
SUMME der ZAEHLER 221.152 -- aber das sind Mehrfachnennungen (allein K2 und K4 ueberlappen
in 31.605 Zellen). Die richtige Zahl steht in der Facetten-CSV: klasse==0 gilt fuer 588.667
Zellen, also bekommen 166.677 = 22,07 % KEINE Facette. (Korrektur meiner ersten Angabe 29,3 %,
die die Zaehler unzulaessig addiert hat.)
Im 26-Grad-Kanal sind dieselben Zaehler ALLE 0 -- das Verwerfen ist ein reines Fahrzeugproblem
und war an der Treppe nicht sichtbar. Die zwei grossen Toepfe entsprechen genau Heikos beiden
Ideen: K4 ist die y_w-Untergrenze (Arm g5 zieht daran, der Code empfiehlt selbst 0,15), K2 ist
die Kante -- dort kann EINE Ausgleichsebene die Geometrie prinzipiell nicht darstellen, und
genau dort waeren mehrere Facetten je Zelle der richtige Ansatz (Codeaenderung, noch nicht getan).

### B31 -- DIE VERWORFENE POPULATION SEZIERT (ohne neuen Lauf, aus export/zband_d_dd/facetten_histogramme.csv)
Fahrzeug 8 mm, 755.344 Facettenzeilen, Fenster 3^3, YWMIN 0,2. Drei Befunde, alle aus Echtdaten:

1) DIE K2-SCHWELLE 0,15 IST GUT GEEICHT, NICHT WILLKUERLICH. r21 = ew_min/ew_mid ist ein
   PCA-Ebenheitsmass (setup.cpp:1350-1353). Die SAUBERE Population endet bei q99 = 0,1327, die
   K2-Population beginnt bei q10 = 0,1635, Median 0,2591. Die Schwelle liegt genau im Tal
   zwischen beiden. Nur 21,1 % der K2-Faelle sind Grenzfaelle (r21 < 0,20) -- die uebrigen
   78,9 % sind ECHTE Kanten, an denen eine einzelne Ausgleichsebene prinzipiell falsch ist.
   FOLGERUNG: die Schwelle hochzuschrauben waere der falsche Hebel -- er wuerde falsche Ebenen
   zulassen statt Kanten richtig zu behandeln.

2) HEIKOS MEHRFACH-FACETTEN-IDEE IST FUER DIESE POPULATION NUMERISCH DURCHFUEHRBAR.
   Punkte je Zelle in der K2-Population: Median 61, q10 27, q90 92. np >= 12 (das Minimum fuer
   zwei Ebenen zu je 6 Punkten) gilt fuer 95.953 von 95.953 Zellen = 100,0 %. Es fehlt also
   nicht an Datenpunkten, sondern nur an einer Zerlegung der Wolke.

3) K4 IST ZU 100 % EIN UNTERSCHREITEN, KEIN UEBERSCHREITEN. Alle 100.891 K4-Faelle haben
   y_w < 0,20; kein einziger liegt ueber 2,0. YWMIN=0,15 rettet davon 53.869 = 53,4 %,
   YWMIN=0,10 rettet 73.708 = 73,1 %. Das ist die Grundlage der Vorab-Prognose in Arm g5.

### B32 -- GEGENPROBEN ZUM 5^3-FENSTER: der Hebel ist real, aber NICHT global brauchbar
Serie logs/t5_gegenprobe_serie.txt, je EINE Variable (CFD_FACETTEN_FENSTER=2), Basis jeweils
woertlich aus dem bestehenden Referenzarm. Census vor und nach der Serie frei.

  Fall            Winkel 3^3 -> 5^3   wahrer Winkel   Leitgroesse                3^3 -> 5^3
  kipp0 (eben)     0,0  ->  0,0         0,0           FELD-HASH             BIT-IDENTISCH
  kipp45           45,0 ->  45,0        45,000        u_tau IST          0,004900 -> 0,004641
  kipp26           28,2 ->  27,2        26,565        u_tau IST/Ziel       2,382  -> 1,507
  Kugel            36,7 ->  36,4        (kruemmung)   Cd_druck             0,6648 -> 0,6154

1) g1 EBENE WAND: BESTANDEN, und zwar maximal sauber. FELD-HASH 4722579264326613690,
   Reibung x 0,01639657, u_tau IST 0,002148 -- auf jede Stelle identisch. Der Schalter feuert
   nachweislich ("Facetten: Fenster 5^3 (CFD_FACETTEN_FENSTER=2)", R geht in die Fitschleife
   setup.cpp:1305). Exakte Neutralitaet, nicht ungefaehre.

2) g2 45-GRAD: der Winkel war schon bei 3^3 EXAKT 45,0 (die Diagonallinks des Gitters liegen
   dort genau richtig), und er bleibt 45,0. Entsprechend klein der Effekt: u_tau IST faellt nur
   5,3 %. ABER die Buchungsidentitaet verschlechtert sich: Verhaeltnis Reibungspfad/Kraftbilanz
   1,0109 -> 1,0479, also von 1,1 auf 4,8 % Abweichung. (Anmerkung: der 3^3-Bezugsarm verletzt
   K2 mit 1,0109 bereits selbst -- die K2-Verletzung ist bei 45 Grad nicht neu, sie waechst nur.)

3) DAS IST DIE EIGENTLICHE ERKENNTNIS: DER GEWINN SKALIERT MIT DEM WINKELFEHLER DES FITS.
   kipp0 Fehler 0,00 Grad -> Effekt exakt 0. kipp45 Fehler 0,00 Grad -> Effekt 5 %.
   kipp26 Fehler 1,63 Grad -> Effekt 37 %. Das ist eine geschlossene Reihe ueber drei Faelle und
   bestaetigt den Mechanismus aus B28 unabhaengig: das groessere Fenster hilft GENAU dort, wo der
   kleine Fit die Normale falsch misst, und sonst nirgends.

4) g3 KUGEL: DURCHGEFALLEN, am vorab festgelegten Kriterium (Cd_druck mehr als 5 %).
   Cd_druck 0,6648 -> 0,6154 = -7,4 %, Summe 1,0796 -> 1,0375 = -3,9 %, Slot 69
   10746 -> 12546 = +16,8 %. Der Grund ist am y_w direkt ablesbar: Median 0,814 -> 1,044
   (+28 %), q10 0,291 -> 0,530. Der 5^3-Fit schiebt die Wand nach aussen, weil er ueber echte
   Kruemmung mittelt. Beide Arme haben 184 Samples; die Kugel ist instationaer, ein Teil der
   7,4 % kann Streuung sein -- die y_w-Drift von 28 % ist dagegen eine systematische
   Geometrieverschiebung und traegt den Befund allein.
   DAMIT IST DER ALTE QUELLTEXT-KOMMENTAR (setup.cpp:1258, A/B vom 15.08.) BESTAETIGT:
   "Groessere Fenster sehen Kruemmung und Zweitflaechen, keine bessere Wand."

VERDIKT: CFD_FACETTEN_FENSTER=2 wird NICHT global uebernommen. Der Default 3^3 bleibt.
Der Befund ist trotzdem wertvoll, weil er die URSACHE der 26-Grad-Physik lokalisiert: nicht das
Wandmodell, sondern die FALSCH GEMESSENE NORMALE des Fits (1,63 Grad). Ein Fenster ist dafuer
das falsche Werkzeug -- es ist global, das Problem ist lokal.

### B33 -- FAHRZEUGARME: beide Vorab-Prognosen eingetroffen, eine davon ZELLGENAU
Serie logs/t6_fahrzeug_fenster_serie.txt, Binary 04d4e2b, je EINE Variable gegen die
Standardkonfiguration. Census vor und nach der Serie frei.

  Zaehler (von 755.344)   Basis    FENSTER=2            YWMIN=0,15
  K2 (Kante)              95.953   162.977  (+69,9 %)    95.953  (+0,0 %)
  K4 (y_w)               100.891    83.609  (-17,1 %)    47.022  (-53,4 %)
  Orientierung            23.034    55.779 (+142,2 %)    23.034  (+0,0 %)

1) PROGNOSE B29 BESTAETIGT. Vor dem Lauf festgehalten war: "wenn die alte A/B recht hat und
   groessere Fenster Kruemmung und Zweitflaechen sehen, muss am Fahrzeug K2(Kante) STEIGEN und
   Orientierung ebenfalls". Gemessen: K2 +69,9 %, Orientierung +142,2 %. Die alte A/B vom
   15.08. nannte fuer die Orientierung 2,7 -> 5,4 %; gemessen wurden 3,05 -> 7,38 %. Gleiche
   Richtung, gleiche Groessenordnung, unabhaengig reproduziert. Der Quelltext-Kommentar
   setup.cpp:1256-1259 ist damit am Fahrzeug bestaetigt, nicht nur an der Kugel (B32).

2) PROGNOSE B31 ZELLGENAU BESTAETIGT. Aus der Facetten-CSV war ohne Lauf vorhergesagt:
   YWMIN=0,15 rettet 53.869 der 100.891 K4-Zellen. Gemessen: 100.891 - 47.022 = 53.869.
   Nicht "ungefaehr" -- auf die Zelle. Damit ist die Zuordnung K4 <-> YWMIN bewiesen und die
   CSV-Auswertung aus B31 als Vorhersageinstrument abgenommen.

3) EINVARIABLEN-KONTROLLE BESTANDEN: g5 laesst K2 und Orientierung exakt unveraendert
   (95.953 / 23.034). YWMIN wirkt ausschliesslich auf K4, wie es soll.

4) DIE GEOMETRIEZAEHLER SIND UEBER DIE CODELUECKE HINWEG GUELTIG. Beim Auswerten fiel auf, dass
   der bisherige Bezugslauf f8_standard_final von Commit fdb3a03 stammt, HEAD aber d7a283b ist
   -- 69 Commits an src/, und die Kraft-CSV hat vier Spalten mehr. KRAEFTE ueber diese Luecke zu
   vergleichen waere ein zweiter Variablenwechsel und wird hier NICHT getan.
   Dass die GEOMETRIE trotzdem vergleichbar ist, ist gemessen und nicht angenommen:
   t6_fahrzeug_ywmin15 reproduziert K2 = 95.953 und Orientierung = 23.034 des alten Laufs
   zellgenau. Der Facettenbau ist ueber die 69 Commits unveraendert.
   Fuer die Kraefte laeuft der zeitgleiche Bezugsarm t7_basis_heute (logs/t7_basis_serie.txt).

### B34 -- FAHRZEUGKRAEFTE auf identischem Binary: beide Hebel bewegen Cz, keiner ist ein reiner Gewinn
t7_basis_heute / t6_fahrzeug_f2 / t6_fahrzeug_ywmin15, src/ zwischen 04d4e2b und d7a283b
nachweislich identisch (git diff --stat -- src/ ist leer). Alle drei rc=0, keine Abnahme
verletzt. Fehlerbalken = Block-SEM ueber 16 Bloecke, zweite Laufhaelfte.

                        cd_druck   +-SEM    cd_reib   Cz_rest    +-SEM    vs Basis
  Basis (3^3, YWMIN 0,2)  1,5747   0,0126    0,0576   -0,0942   0,01456      --
  FENSTER=2               1,5218   0,0114    0,0385   -0,1512   0,01632   2,61 sigma
  YWMIN=0,15              1,7828   0,0113    0,0670   -0,1845   0,01758   3,96 sigma
  OF13 mr2v40H                                         -1,3010

BEIDE HEBEL SCHIEBEN Cz IN DIE RICHTIGE RICHTUNG, und zwar signifikant. YWMIN=0,15 ist der
staerkere: Abstand zu OF13 von -92,8 auf -85,8 %. Das ist gemessen, aber es bleibt ein
Siebtel des Ziels -- der Abtrieb fehlt weiterhin fast vollstaendig.

Cd GEHT AUSEINANDER: FENSTER=2 senkt cd_druck um 0,0529 (3,1 sigma), YWMIN=0,15 HEBT es um
0,2081 (12,3 sigma). Der YWMIN-Arm ist damit KEIN reiner Gewinn, sondern ein Tausch.

DIE URSACHE DES Cd-ANSTIEGS IST AM ZAEHLER ABLESBAR: Slot 69 (Rueckfall auf reine
Druckbuchung) steigt 60.607.351 -> 65.671.896 = +8,4 %, waehrend YWMIN nur 7,1 % mehr Zellen
zulaesst (53.869 von 755.344). Die neu zugelassenen Zellen liegen bei y_w zwischen 0,15 und
0,20, also sehr dicht an der Wand -- der Reibungsloeser scheitert dort ueberwiegend und faellt
auf P-only zurueck. Sie bringen also Druckwiderstand ein, ohne die zugehoerige Reibungs-
behandlung. Das erklaert Cz-Gewinn UND Cd-Verlust aus EINEM Mechanismus.
VERDIKT: YWMIN=0,15 wird NICHT als Default uebernommen. Es bleibt ein deklarierter Messarm.

METHODISCHE BESTAETIGUNG, warum der Nachlauf noetig war: derselbe Konfigurationssatz ergab bei
Commit fdb3a03 ein Cz_rest von -0,0729, heute -0,0942. 29 % Unterschied allein aus 69
src/-Commits. Haette ich t6 gegen den alten Bezugslauf gerechnet, waere der FENSTER-Effekt
(-0,0570) fast vollstaendig aus der Codeluecke gekommen statt aus dem Schalter.

### B35 -- HEIKOS RUECKFRAGE DECKT DEN KONSTRUKTIONSFEHLER AUF: wir facettieren die Oberflaeche gar nicht
Heiko am 28.08.: "wir voxelieren und dann facettieren wir die komplette Oberflaeche des
Solidkoerpers ... ich bin mit deiner Angabe 3^3 und 5^3 bezueglich Facettenmenge etwas
ueberrascht, denn so viele duerfte es nach meiner Rechnung niemals sein."
Die Ueberraschung ist berechtigt -- die Praemisse trifft nicht zu, und das Wort "Facette" ist
im Code ein Fehlname. Belegt an FluidX3D-v2, src/setup.cpp, Commit 362f410:

WAS DER CODE WIRKLICH TUT (setup.cpp:1254-1320):
  - EINE Facette je wandnaher FLUIDZELLE, nicht je Dreieck. Am Fahrzeug 8 mm sind das 755.344.
  - 3^3/5^3 ist KEINE Facettenzahl, sondern der Radius der Nachbarschaft, aus der STUETZPUNKTE
    eingesammelt werden.
  - Die Stuetzpunkte sind LINKMITTELPUNKTE: setup.cpp:1319 setzt sie auf
    0.5*(Zellmitte + Wandzellmitte) jedes Fluid->Wand-Links der 18 D3Q19-Richtungen.
    Fest 0,5 -- kein q, kein Subgrid-Abstand, kein Dreieck.
  - Daher die Punktzahlen: bis zu 27 Zellen x 18 Richtungen = 486 moegliche Stuetzpunkte je
    Zelle bei 3^3, gemessener Median 61 (B31). Es sind Linkmitten, keine Dreiecke -- Heikos
    Rechnung, dass an einem Zellmittelpunkt niemals so viele Dreiecke haengen, stimmt.
  - baue_facetten() SIEHT DIE STL NIE: die Signatur (setup.cpp:1254-1256) hat keinen Mesh*.

FOLGE: DIE AUSGLEICHSEBENE BESCHREIBT DIE VOXELTREPPE, NICHT DIE GEOMETRIE.
Unabhaengiger Nachbau des Algorithmus in Python (Rampe 1:2, wahre Normale atan(1/2) =
26,565 Grad), 150 Zellen, identische PCA:
     Stuetzpunkt                      3^3      5^3      7^3
     Linkmitte 0,5 (heutiger Code)  28,01    27,19    26,86    Fehler +1,44 / +0,62 / +0,29
     wahrer Flaechenschnitt (q)     26,57    26,57    26,57    Fehler +0,00 / +0,00 / +0,00
Der Nachbau trifft die MESSUNG (28,2 / 27,2 aus t2_26_u10 und t4_fenster2) auf 0,2 bzw.
0,01 Grad. Der Winkelfehler ist damit KEIN Bug und kein Rauschen, sondern die zwangslaeufige
Folge der 0,5-Annahme -- und er verschwindet vollstaendig, sobald der Stuetzpunkt auf der
wahren Flaeche liegt, bei JEDER Fenstergroesse.

DAS ENTWERTET DIE HEUTIGE FENSTERSERIE ALS LOESUNG UND ERKLAERT SIE ZUGLEICH:
5^3 half am 26-Grad-Kanal (B28: u_tau 2,382 -> 1,507), weil es die Treppe teilweise
wegmittelt -- 1,44 -> 0,62 Grad. Es ist ein Behelf gegen ein Abtastproblem. An der Kugel
scheitert er (B32), weil Mitteln dort echte Kruemmung zerstoert. Mit richtigen Stuetzpunkten
braucht man das Fenster ueberhaupt nicht: 3^3 ist dann bereits exakt.

DIE MITTEL DAFUER LIEGEN BEREIT UND SIND NUR NICHT ANGESCHLOSSEN:
  - setup.cpp:3148 legt veh_f = read_stl(...) an und platziert es im Nahfeldgitter;
    baue_facetten(lbm_f, ...) wird in DERSELBEN Funktion bei 3598/3613 gerufen. Das Mesh ist
    an der Aufrufstelle in Reichweite, es wird nur nicht uebergeben.
  - Die REMESH-Stufe rechnet bereits mit echten Dreiecken (setup.cpp:1137-1172,
    stl->p0/p1/p2), der Facettenbau also nicht aus Mangel an Daten.

DAS ORDNET AUCH DIE KANTENVERWERFUNG NEU EIN (B30/B31): die 95.953 K2-Faelle sind Zellen, in
denen die LINKMITTEN nicht eben liegen. Mit echten Dreiecken waere eine Kantenzelle nicht ein
Fehlschlag, sondern schlicht eine Zelle mit zwei Flaechen -- genau Heikos urspruenglicher
Vorschlag, jetzt aber aus der richtigen Datenquelle.

NOCH NICHT BEWIESEN und ehrlich zu trennen: dass exakte Normalen die Fahrzeugkraefte
verbessern. Gezeigt ist die Normale an einer analytischen Rampe. Am Fahrzeug kommen
Dreiecksdichte gegen Zellgroesse, Mehrfachflaechen je Zelle und der Reibungsloeser dazu.

### B36 -- HEIKOS DREIECKSFACETTIERUNG DES VOXELKOERPERS: numerisch geprueft, und SIE IST SCHON GEBAUT
Heiko-Vorgabe 28.08. (bindend, gemerkt): nach dem Voxelieren wird NICHT mehr auf die STL
zurueckgegriffen. Grund: duenne/spitze Teile werden beim Voxelieren aufgedickt; Normalen aus der
STL und ein Stroemungsfeld am aufgedickten Koerper waeren zwei Wahrheiten im selben Modell.
Sein Vorschlag stattdessen: Oberflaeche des VOXELKOERPERS dreiecksfacettieren, je Zelle alle
beruehrenden Dreiecke heranziehen.

A) DIE TRAGENDE EIGENSCHAFT IST EXAKT -- und sie ist der eigentliche Fund.
   Rampe 1:2, Oberflaechenbilanz ueber die schraege Deckflaeche (jede Voxelflaeche genau einmal):
     Vektorsumme |sum n*A| = 429,33  vs wahre Flaeche 429,33   ->  +0,00 %
     Skalare Summe  sum|A| = 576,00  vs wahre Flaeche 429,33   -> +34,16 %
     Winkel der Gesamtnormalen 26,565 Grad gegen wahr 26,565   -> Fehler +0,000
   Die Voxeloberflaeche traegt den EXAKTEN Flaechenvektor des glatten Koerpers; die Aufdickung
   steckt vollstaendig im SKALAR, nicht im Vektor. Bestaetigt an der Kugel ueber die projizierte
   Flaeche (die den Druckwiderstand traegt): R=8 -4,01 %, R=20 -0,93 %, R=40 -0,27 % -- sie
   KONVERGIERT gegen exakt, waehrend die Mantelflaeche mit +44/+48,6/+49,6 % gegen +50 % laeuft.
   Duenne Platte (Heikos Aufdickungssorge): bei 0,4 / 0,8 / 1,6 Zellen Dicke ist die projizierte
   Flaeche IMMER exakt (+0,0 %), der Mantel +5,8 / +1,9 / -5,2 %. Die Aufdickung eines
   Duennteils beruehrt die projizierte Flaeche also gar nicht.

B) JE ZELLE IST KEIN VERFAHREN UEBERALL BESSER. Winkelfehler gegen die wahre Normale,
   Fenster 3^3, eigener Nachbau:
     Verfahren                          Rampe 1:2   Rampe 1:1   Kugel R=20   Ebene
     M1 alle 18 Linkmitten (HEUTE)         1,44        0,00        4,52       0,00
     M2 nur Voxelflaechen, PCA             0,83        0,00        5,19       0,00
     M3 Normalen-Vektorsumme (Heiko lit.)  8,13        0,00        4,00       0,00
   ZWEI ABLESUNGEN: (1) Die DIAGONALLINKS sind eine echte Fehlerquelle -- sie wegzulassen
   halbiert den Rampenfehler (1,44 -> 0,83), und sie liegen bauartbedingt gar nicht auf der
   Voxeloberflaeche (ihre Mitte streift die Solid-Ecke). (2) Die reine Normalensumme ist lokal
   phasenempfindlich (3^3 fasst 1,5 Treppenperioden), global aber exakt -- siehe A.

C) HEIKOS "KEINE FACETTE KANN VERWORFEN WERDEN": BESTAETIGT, mit einer Praezisierung.
   Nimmt man nur die direkt begrenzenden Flaechen, bleiben 144 von 424 Zellen (Rampe) bzw. 200
   von 408 (45 Grad) ohne Normale. Nimmt man alle Flaechen der 3^3-Nachbarschaft, sind es NULL
   -- in jedem geprueften Fall. Die heutige Klasse K2(Kante) mit 95.953 Fahrzeugzellen
   verschwindet als Fehlerart vollstaendig: eine Kantenzelle ist dann kein Fehlschlag, sondern
   eine Zelle mit zwei Flaechen. K4(y_w) braucht weiterhin eine Wandabstandsdefinition.

D) "AN KANTEN VIELLEICHT DIE AUFDICKUNG REDUZIEREN": gemessen, und es stimmt mechanisch.
   An einer 90-Grad-Aussenkante liefert die Normalensumme exakt (0,707, 0, 0,707) = 45,0 Grad,
   ueber der Deckflaeche 0,0 und vor der Seitenflaeche 90,0 Grad. Das Verfahren SCHNEIDET die
   Ecke also. EHRLICHE EINSCHRAENKUNG: bei einer ECHTEN 90-Grad-Kante ist diese Fase eine
   Erfindung, keine Korrektur. Nur wo die Kante durch Aufdickung erst entstanden ist, ist sie
   richtig -- und welcher Fall vorliegt, ist aus dem Voxelfeld allein nicht entscheidbar.

E) DER EIGENTLICHE BEFUND: DAS IST ALLES SCHON GEBAUT UND LIEGT BRACH.
   setup.cpp:753-1215, remesh_facetten_diag(), Kopfzeile woertlich: "REMESH DER VOXEL-AUSSENWAND
   (Heiko-Vorgabe, Arbeitsliste 11a): geschlossene Dreiecksflaeche ueber dem FINALEN flags-Feld
   (nicht der STL ...)". Verfahren: NAIVE SURFACE NETS, je Grenzflaeche solid|fluid ein Quad,
   Taubin-Glaettung mit harter Vertex-Klemme +-0,5 Zelle. Es berechnet q je (Zelle, Richtung)
   aus der geglaetteten Flaeche (setup.cpp:1212/1215) und fuellt elibb_qmap. Der Kernel nimmt es
   entgegen (kernel.cpp:1744/1791, fac_q). Plan: FACETTEN-ELIBB-PLAN.md, dort steht als erste
   Zeile "Facettenquelle = Remesh der VOXEL-Aussenwand (nicht STL)".
   ZWEI LUECKEN:
   1. baue_facetten() BEKOMMT DIESE FLAECHE NIE. Seine Signatur (setup.cpp:1254-1256) hat weder
      Mesh noch q. Der Wandabstand q kommt also aus der geglaetteten Voxelflaeche, die NORMALE
      aber weiter aus der 0,5-Linkmitten-PCA der rohen Treppe. Zwei Flaechen im selben
      Wandmodell -- nicht die STL/Voxel-Diskrepanz, die Heiko befuerchtet hat, aber derselbe
      Fehlertyp eine Ebene tiefer.
   2. AM FAHRZEUG LAEUFT REMESH UEBERHAUPT NICHT. Gate (setup.cpp:3600):
      CFD_FACETTEN_REMESH>0 ODER (CFD_FACETTEN>=3 UND CFD_FAC_ELIBB>0). Die Standardkonfiguration
      setzt kein ELIBB -- gezaehlt in den heutigen Logs: t7_basis_heute 0 REMESH-Zeilen,
      t6_fahrzeug_ywmin15 0, dagegen t5_kugel_f2 26.

   FOLGERUNG: der naechste Schritt ist nicht "Heikos Idee bauen", sondern die Normale an die
   Flaeche anzuschliessen, die bereits erzeugt wird. Die Taubin-geglaettete Remesh-Flaeche ist
   dabei WEDER M1 noch M2 noch M3 -- sie ist der einzige Kandidat, der Treppe und Kruemmung
   zugleich adressieren koennte, und sie ist NICHT VERMESSEN. Das ist der erste zu messende Arm.

### B37 -- TAUBIN GEGEN DUENNTEILE: die ROHE Voxelflaeche ist an Gurney/Canard exakt, die geglaettete nicht
Anlass: Heiko 28.08. -- "dachte nur das meine idee gerade bei den canards, gurny und
luftleitblechen deutlich physik/detailgetreuer waere". Der Code warnt an der Glaettung selbst
(setup.cpp:884-886): "Taubin daempft bei lam/mu jedes Merkmal unter rund 11 Zellen Periode und
loescht alles unter rund 6 aus". Genau diese Groessenordnung sind Gurneys und Canards.

Eigener Nachbau des Remesh (Surface Nets auf dem dualen Eckgitter + Taubin lam 0,5 / mu -0,53 +
harte Klemme +-0,5 Zelle, wie setup.cpp:793-901). Pruefkoerper: Lippe 1 Zelle dick, Hoehe h,
Breite 20, auf einer 1 Zelle dicken Platte. Gemessen wird die PROJIZIERTE STIRNFLAECHE in
Anstroemrichtung -- die Groesse, die den Abtrieb traegt. Soll = (h+1)*20.

   h     Soll    roh ITER=0   ITER=8 (Default)   ITER=15
   1     40,0     40,0  +0,0%    42,1  +5,3%      41,2  +2,9%
   2     60,0     60,0  +0,0%    63,5  +5,8%      63,4  +5,7%
   3     80,0     80,0  +0,0%    83,3  +4,1%      83,6  +4,4%
   4    100,0    100,0  +0,0%   103,1  +3,1%     103,2  +3,2%
   6    140,0    140,0  +0,0%   142,9  +2,1%     142,8  +2,0%
  10    220,0    220,0  +0,0%   222,5  +1,1%     222,2  +1,0%

ZWEI BEFUNDE:
1) DIE ROHE VOXELFLAECHE IST BEI JEDER LIPPENHOEHE EXAKT (+0,0 %). Heikos Ansatz ist an
   Duennteilen also nicht nur "genauer", sondern in dieser Groesse fehlerfrei -- und die
   Aufdickung selbst beruehrt die projizierte Flaeche nicht (B36 A: 0,4-Zellen-Blech, +0,0 %).
2) DIE GLAETTUNG VERSCHLECHTERT DAS, UND ZWAR AM STAERKSTEN BEI DEN KLEINSTEN MERKMALEN:
   +5,8 % bei h=2 gegen +1,1 % bei h=10. Sie loescht den Gurney nicht (die +-0,5-Klemme
   verhindert das), aber sie macht seine Stirnflaeche fett. Mehr Iterationen helfen nicht
   (ITER=15 praktisch wie ITER=8) -- es ist die Klemme, die den Endzustand bestimmt.

FOLGE FUER DEN BAU: CFD_FACETTEN_REMESH_ITER (setup.cpp:887, Default 8) ist damit KEIN
Nebenparameter, sondern ein Messarm erster Ordnung. Es gibt einen echten Zielkonflikt:
die Glaettung ist gegen die Treppe gedacht (schraege Flaechen), schadet aber duennen scharfen
Merkmalen. Ob ein einziger ITER-Wert beides bedient, ist offen und muss gemessen werden;
falls nicht, ist der vorhandene v_fest/Quetschkanten-Mechanismus (setup.cpp:875-880) die
Stelle, an der merkmalserhaltend geklemmt werden koennte.
KORREKTUR (Heiko, 28.08., unmittelbar nach der ersten Fassung dieses Befunds): mein Satz
"die MR2 hat in dieser Fassung keinen Gurney" war FALSCH. Heiko: "natuerlich hat der mr2
gurney, canards und luftleitbleche...sind selbst bei 8mm noch deutlich durch sat voxelizer
sichtbar". Damit ist B37 kein synthetisches Randthema, sondern trifft den Produktionsfall
direkt -- und zwar auf der Sprosse, auf der wir messen.
WAS DARAUS FOLGT: die Duennteil-Population der MR2 muss GEZAEHLT werden, bevor irgendein
ITER-Wert gewaehlt wird. Dafuer fehlt heute das Instrument; der Facettenzensus kennt keine
Solid-Dicke. Neu zu bauen: solid_dicke(x,y,z) als Gegenstueck zu freie_weite
(setup.cpp:1092-1105) -- Minimum der Solid-Lauflaenge ueber die drei Achsen -- und daraus ein
Histogramm {1, 2, 3, >=4 Zellen} am Fahrzeug 8 mm, plus die Winkelabweichung alt-gegen-neu
NACH Solid-Dicke aufgeschluesselt. Ohne diese Aufschluesselung geht der Duennteileffekt in
755.344 Zellen unter. Laeuft als Census (CFD_FACETTEN_DIAG=2), kostet keine GPU-Stunde.
Die Zahlen oben bleiben Geometrie, keine Stroemung.

### B38 -- DIE VERWERFUNGEN SITZEN AN HEIKOS BAUTEILEN: K2 ist 5,2-fach haeufiger an 1-Zellen-Teilen
Neues Instrument `solid_dicke` im Facettenzensus (setup.cpp, Block nach dem Winkel-Bericht):
je Wandnachbar die Solid-Lauflaenge entlang der drei Achsen, davon das Minimum = lokale
Bauteildicke; je Facettenzelle das Minimum ueber ihre Wandnachbarn (das duennste beruehrte
Teil). Lauflaenge bei 9 gekappt, Wrap-Trennung wie im Rest der Funktion. Zusaetzlich als
Spalte `solid_dicke` in facetten_histogramme.csv. Lauf t8_dicke_dd (Fahrzeug 8 mm,
CFD_FACETTEN_DIAG=2, kein Stroemungslauf), Census vor und nach der Serie frei.

DICKENVERTEILUNG, 755.344 wandnahe Fluidzellen:
  1 Zelle 57.248 (7,58 %) | 2 Zellen 102.369 (13,55 %) | 3 Zellen 40.446 (5,35 %)
  4-8 Zellen 73.160 (9,69 %) | >=9 Zellen 482.121 (63,83 %)
  <=3 Zellen zusammen 200.063 = 26,49 %. Im FERNFELD sogar 19.153 von 42.887 = 44,66 %.

KREUZTABELLE KLASSE GEGEN DICKE -- der eigentliche Befund:
  Solid-Dicke   Zellen     K2(Kante)      K4(y_w)     ohne Facette
  1 Zelle        57.248   19.953 34,9%   9.615 16,8%   23.019 40,2%
  2 Zellen      102.369   24.935 24,4%  15.575 15,2%   33.628 32,8%
  3 Zellen       40.446    9.285 23,0%   6.208 15,3%   12.368 30,6%
  4-8 Zellen     73.160    9.461 12,9%  10.867 14,9%   17.572 24,0%
  >=9 Zellen    482.121   32.319  6,7%  58.626 12,2%   80.090 16,6%
  GESAMT        755.344   95.953 12,7% 100.891 13,4%  166.677 22,1%

1) K2 IST EIN DUENNTEIL-PHAENOMEN: 34,9 % an 1-Zellen-Teilen gegen 6,7 % an Teilen >=9 Zellen,
   also FAKTOR 5,2. Die Kantenklasse ist damit keine gleichverteilte Eigenschaft der
   Geometrie, sondern konzentriert sich dort, wo Heikos Gurney, Canards und Luftleitbleche
   sitzen. An einem einzelligen Teil bekommen ZWEI VON FUENF wandnahen Zellen (40,2 %)
   ueberhaupt keine Facette.
2) K4 IST ES NICHT: 16,8 % gegen 12,2 % ist nahezu flach. Der y_w-Topf hat mit der Bauteildicke
   fast nichts zu tun -- die beiden grossen Verwerfungsgruende haben also VERSCHIEDENE
   Ursachen und brauchen verschiedene Loesungen. Das war vorher nicht bekannt.
3) GEWICHT: die Verwerfungsrate ist duenn 34,5 % gegen dick 16,6 % (Faktor 2,08). 41,4 % ALLER
   verworfenen Zellen sitzen an Teilen <=3 Zellen, die nur 26,5 % der Bevoelkerung stellen.

FOLGERUNG: Heikos Vermutung ("gerade bei den canards, gurny und luftleitblechen deutlich
physik/detailgetreuer") trifft die schlechteste Population des heutigen Verfahrens. Der
Kantenfall und seine Bauteile sind dasselbe Problem. Zusammen mit B37 (Taubin dickt genau dort
die projizierte Stirnflaeche um bis zu 5,8 % auf, waehrend die rohe Voxelflaeche exakt ist)
ergibt das eine klare Reihenfolge: die Normalenquelle muss aus der Flaeche kommen, und die
Glaettung ist an Duennteilen der Gegner, nicht der Helfer.

### B39 -- DREIFACHVERGLEICH DER NORMALENQUELLE: HEIKOS VERFAHREN GEWINNT KLAR, ERREICHT ABER NICHT NULL
Serie logs/t9_vergleich_serie.txt, Schalter CFD_FACETTEN_VERGLEICH (Default 0). Census vor und
nach der Serie frei. ABNAHME BITGLEICHHEIT bestanden: Arm t9_bitgleich (Schalter AUS, voller
Kanallauf) liefert FELD-HASH 4722579264326613690 == Soll -- der neue Code fasst die Physik nicht an.
Alle drei Verfahren speisen sich AUSSCHLIESSLICH aus dem Voxelkoerper (Heiko-Vorgabe), der
Remesh wird nicht gebraucht: die rohe Voxeloberflaeche SIND die achsparallelen Solid/Fluid-Flaechen.

  8 mm, 755.344 wandnahe Fluidzellen        K1     K2(Kante)   K4(y_w)   OHNE FACETTE
  V1 alle 18 Linkmitten (heute)              0        95.953   100.891   165.239 = 21,88 %
  V2 nur Achslinks, PCA (mein Vorschlag)   236        64.142    93.376   135.201 = 17,90 %
  V3 Flaechennormalen-Summe (HEIKO)      2.233             0    89.071    91.304 = 12,09 %

  4 mm, 3.275.381 wandnahe Fluidzellen      K1     K2(Kante)   K4(y_w)   OHNE FACETTE
  V1                                         0       312.840   483.121   651.623 = 19,89 %
  V2                                       215       204.975   467.040   548.969 = 16,76 %
  V3                                       170             0   441.212   441.382 = 13,48 %

1) K2 IST BEI V3 EXAKT NULL -- in BEIDEN Aufloesungen. Das war die Vorabprognose und sie ist
   konstruktiv: ohne Eigenwertproblem gibt es kein r21, also kann die Klasse "Kante" gar nicht
   entstehen. Damit verschwindet der Topf, der 95.953 (8 mm) bzw. 312.840 (4 mm) Zellen kostete.
2) AN DUENNTEILEN IST DER ABSTAND AM GROESSTEN -- genau dort, wo Heiko es vermutet hat.
   Verwerfungsrate an 1-Zellen-Teilen:  8 mm  V1 40,0 % -> V2 29,6 % -> V3 12,5 %
                                        4 mm  V1 53,4 % -> V2 42,7 % -> V3 16,2 %
   V3 drittelt die Verwerfung an genau der Population, an der Gurney, Canards und Leitbleche
   sitzen. Bei 4 mm ist der Effekt GROESSER als bei 8 mm, nicht kleiner.
3) HEIKOS ABNAHME "VERWORFENE FACETTEN DUERFEN WIR NICHT HABEN" IST NICHT ERFUELLT.
   V3 landet bei 12,09 % (8 mm) bzw. 13,48 % (4 mm), und der Rest ist VOLLSTAENDIG K4 (y_w):
   89.071 von 91.304 bzw. 441.212 von 441.382. K4 ist damit der alleinige verbleibende Grund.
   Das passt zu B38: K4 ist nicht dickenabhaengig (16,8 % gegen 12,2 %), hat also eine ANDERE
   Ursache als K2 und braucht eine eigene Loesung. Der naechste Schritt liegt damit fest.
4) ZWEI EHRLICHE EINSCHRAENKUNGEN AN V3:
   a) V3 hat als einziges Verfahren einen K1-Topf, der bei 8 mm auf 2.233 Zellen anwaechst
      (4 mm nur 170). Ursache ist konstruktiv: an einem einzelligen Teil zeigen die beiden
      gegenueberliegenden Voxelflaechen exakt entgegengesetzt, die Vektorsumme hebt sich auf
      und es gibt KEINE Normale. Das ist der Preis der Vektorsumme und trifft ausgerechnet die
      duennste Lage. Bei 4 mm ist dasselbe Bauteil zwei Zellen dick, deshalb der Einbruch von
      2.233 auf 170. Eine Rueckfallregel dafuer fehlt noch.
   b) Der Winkel von V3 weicht von V1 im Median um 3,55 Grad ab (q90 9,57). Welcher der beiden
      naeher an der Wahrheit liegt, sagt dieser Lauf NICHT -- am Fahrzeug gibt es keine
      Grundwahrheit. Belegt ist die Ueberlegenheit von V3 bei der VERWERFUNG, nicht beim Winkel.
      Die Winkelfrage entscheidet die Kugel gegen die analytische Normale (noch offen).
   c) Auffaellig und noch nicht erklaert: bei 4 mm melden Dicke-2-Zellen (367.988) einen
      Winkelmedian von exakt 0,00 fuer BEIDE Verfahren. Plausibel bei achsparallelen 2-Zellen-
      Blechen, wo alle drei Verfahren dieselbe Achsnormale liefern -- geprueft ist es nicht.

### B40 -- V4: HEIKOS VERFAHREN MIT SICHTBARKEITSFILTER -- VERWERFUNG VON 21,9 AUF 2,7 PROZENT
Zwei rein geometrische Zusaetze zu V3, keine gesetzten Konstanten:
 (1) SICHTBARKEIT -- eine Voxelflaeche zaehlt nur, wenn die Zelle vor ihr liegt: n_i*(Zelle-c_i)>0.
     Die angrenzende Flaeche liefert immer +0,5, die Rueckseite eines einzelligen Blechs -1,5
     und faellt heraus. Genau diese Rueckseite loeschte in V3 die Vektorsumme aus.
 (2) ABSTANDSGEWICHT w = 1/(1+d^2) -- angrenzende Flaeche (d=0,5) wiegt 0,80, Fensterecke
     (d~2,1) nur 0,18. V3 wog alle gleich, deshalb zog entfernte Geometrie die Normale mit.
 Dazu y_w ZWEIFACH: gegen den gewichteten Schwerpunkt (V4) und gegen die naechste sichtbare
 Flaeche (V4b). Bitgleichheit erneut bestanden (FELD-HASH 4722579264326613690, Schalter aus).

  8 mm, 755.344 Zellen                K1      K2      K4(y_w)   OHNE FACETTE
  V1 alle 18 Linkmitten (heute)        0   95.953    100.891    165.239 = 21,88 %
  V2 nur Achslinks (PCA)             236   64.142     93.376    135.201 = 17,90 %
  V3 Flaechennormalen-Summe        2.233        0     89.071     91.304 = 12,09 %
  V4 = V3 + Sicht + Abstandsgewicht   848        0     37.738     38.586 =  5,11 %
  V4b wie V4, y_w aus Nachbarflaeche  848        0     19.260     20.108 =  2,66 %

  4 mm, 3.275.381 Zellen               K1      K2      K4(y_w)   OHNE FACETTE
  V1                                    0  312.840    483.121    651.623 = 19,89 %
  V2                                  215  204.975    467.040    548.969 = 16,76 %
  V3                                  170        0    441.212    441.382 = 13,48 %
  V4                                   62        0    145.853    145.915 =  4,45 %
  V4b                                  62        0     79.050     79.112 =  2,42 %

1) FAKTOR 8 GEGEN HEUTE, in beiden Aufloesungen (21,88 -> 2,66 % und 19,89 -> 2,42 %).
   Der groesste Einzelschritt ist der Sichtbarkeitsfilter auf K4: 89.071 -> 37.738 (8 mm),
   441.212 -> 145.853 (4 mm). Der Grund ist derselbe wie bei K1: entfernte Flaechen zogen den
   Schwerpunkt zur Zelle und drueckten y_w unter YWMIN.
2) AN DUENNTEILEN, ohne Facette an 1-Zellen-Teilen:
     8 mm  V1 40,0 -> V3 12,5 -> V4 9,1 -> V4b 2,4 %
     4 mm  V1 53,4 -> V3 16,2 -> V4 6,3 -> V4b 4,4 %
3) HEIKOS NULL IST NOCH NICHT ERREICHT -- 2,66 bzw. 2,42 % bleiben. Zwei Resttoepfe:
   a) K1 848 (8 mm) / 62 (4 mm). Der Sichtbarkeitsfilter hat V3s 2.233 auf 848 gedrueckt, aber
      nicht auf null. Vermutung (NICHT geprueft): einzellige SPALTE, wo die Zelle zwischen zwei
      gegenueberliegenden Waenden sitzt -- beide sind sichtbar, ihre Normalen heben sich auf.
      Das waere ein geometrisch echter Zweideutigkeitsfall und braucht eine Rueckfallregel.
   b) K4 19.260 (8 mm) / 79.050 (4 mm). Ob das die Untergrenze (y_w<YWMIN) oder die Obergrenze
      (y_w>2,0) ist, meldet der Zaehler heute NICHT getrennt -- das ist die naechste Messung.
4) EHRLICH ZUM WINKEL: V4 liegt im Median naeher an V1 als V3 (2,40 gegen 3,55 Grad), hat aber
   den LAENGEREN Schwanz (q90 17,88 gegen 9,57). Das Abstandsgewicht schaerft Kanten, dort
   weicht V4 staerker ab. Bei 4 mm an 1-Zellen-Teilen weicht V4 sogar MEHR ab als V3
   (10,33 gegen 8,40 Grad). Welcher Winkel richtiger ist, sagt keiner dieser Laeufe -- am
   Fahrzeug gibt es keine Grundwahrheit. Das entscheidet die Kugel gegen die analytische Normale.

### B41 -- KUGEL GEGEN DIE ANALYTISCHE NORMALE: V1 IST AM GENAUESTEN, V4 AM SCHLECHTESTEN
Der einzige Fall mit Grundwahrheit. Serie logs/ta_kugel_serie.txt, reiner Zensus, Census frei.
Wahre Normale = (Zelle - Schwerpunkt)/|...|, Schwerpunkt im Code selbst bestimmt.

  Winkelfehler Median / q90 [Grad]     V1        V2        V3        V4
  Kugel dx=40 (794 Zellen)          6,67/11,50 6,74/13,76 9,89/12,96 13,54/21,13
  Kugel dx=20 (2522 Zellen)         4,85/ 9,81 5,40/10,89 5,53/ 9,43  9,47/16,24

DAS IST EIN KLARES NEIN ZUR GENAUIGKEIT: die heutige Implementierung V1 ist auf der glatten
gekruemmten Wand die BESTE, meine V4 die schlechteste -- rund doppelter Fehler. Auch Heikos V3
liegt hinter V1. Die Reihenfolge ist in beiden Aufloesungen dieselbe, das ist kein Ausreisser.

URSACHE, soweit aus der Reihe ablesbar: V1 6,67 -> V3 9,89 -> V4 13,54 ist genau die Reihenfolge
ABNEHMENDER MITTELUNG. V1 mittelt ueber alle 18 Linkmitten im Fenster, V3 nur ueber die
Achsflaechen, V4 gewichtet zusaetzlich nach Naehe und stuetzt sich damit auf noch weniger
Flaechen. Auf einer GLATTEN Wand ist mehr Mittelung besser -- die Diskretisierungsfehler
mitteln sich weg. Das Abstandsgewicht, das an Kanten schaerft, unterabtastet hier.

WAS DIESER LAUF NICHT ZEIGT, und das ist entscheidend fuer die Einordnung: die Kugel hat laut
eigenem Dickenzensus KEIN Teil unter 3 Zellen (Histogramm 0 0 222 0 456 0 116) und praktisch
keine Kanten (V1-K2 nur 48 von 794 bzw. 2522). Sie ist damit der BESTFALL fuer V1 und testet
genau die Population NICHT, an der V1 am Fahrzeug 22 % verwirft. Eine Grundwahrheit fuer duenne
und kantige Geometrie existiert hier nicht -- es waere ebenso falsch, aus der Kugel auf das
Fahrzeug zu schliessen, wie es falsch waere, V4 aufgrund der Verwerfungszahlen zu uebernehmen.

GESAMTBILD, ehrlich:
  Abdeckung (Verwerfung, Fahrzeug):  V4b 2,55 % << V1 21,88 %   -- V4b gewinnt klar
  Genauigkeit (Winkel, Kugel):       V1 4,85 Grad << V4 9,47    -- V1 gewinnt klar
Das ist ein TAUSCH, kein Gewinn. Und die beiden Staerken liegen an komplementaeren Stellen:
V1 versagt genau dort, wo V4b traegt (duenn, kantig), und V4 verliert genau dort, wo V1 traegt
(dick, glatt). Der naheliegende Schluss ist ein AUSWAHLKRITERIUM statt eines Siegers -- und die
Werkzeuge dafuer liegen bereits: solid_dicke (B38) trennt duenn von dick, V1s eigenes r21
erkennt die Kante. NICHT gebaut, NICHT gemessen -- das ist eine Hypothese, kein Befund.

### B42 -- V3b IST DER PARETO-PUNKT: Heikos V3 unangetastet, zwei Reparaturen, ohne meinen Sichtfilter
V5 (V3 + Sichtbarkeit, OHNE Abstandsgewicht) trennt die beiden Zusaetze aus B40. Kugel:
  Winkelfehler Median  V1 6,67 | V3 9,89 | V4 13,54 | V5 14,96   (dx=40)
                       V1 4,85 | V3 5,53 | V4  9,47 | V5  9,60   (dx=20)
V5 ist NICHT besser als V4 -- also kostet nicht das Abstandsgewicht die Genauigkeit, sondern
DER SICHTBARKEITSFILTER. Er halbiert am konvexen Koerper die beitragenden Flaechen; die
entfernten Flaechen im Fenster liegen dort auf derselben Wand und tragen echte Information.
Beide Zusaetze waren meine Idee, beide sind damit widerlegt.

V3b = V3 + NUR die zwei Reparaturen, die die Mittelung nicht antasten:
  (a) Duennteil-Rueckfall auf die naechste Flaeche, wenn die Vektorsumme entartet
  (b) y_w gegen die naechste Flaeche statt gegen den Schwerpunkt
KONTROLLE BESTANDEN: auf der Kugel liefert V3b Winkel 9,89/12,96 und 5,53/9,43 -- ZIFFERNGLEICH
mit V3. Die Kugel hat keine Duennteile, der Rueckfall darf dort nicht feuern, und er tut es
nicht (Zaehler 0, gekoppelter Waechter meldet korrekte Stille). V3b aendert die Normale also
ausschliesslich dort, wo V3 gar keine hatte.

  OHNE FACETTE      8 mm      4 mm  | Winkelfehler Kugel dx=20
  V1 (heute)      21,88 %   19,89 % | 4,85 / 9,81 Grad
  V3              12,09 %   13,48 % | 5,53 / 9,43
  V3b              7,26 %    7,80 % | 5,53 / 9,43   <- gleiche Genauigkeit wie V3
  V4b              2,55 %    2,41 % | 9,47 / 16,24  <- Abdeckung erkauft mit 4 Grad

V3b ist damit der Pareto-Punkt: beste Genauigkeit der neuen Verfahren (im q90 bei dx=20 sogar
VOR V1: 9,43 gegen 9,81) bei dreifach besserer Abdeckung als heute. V4b kauft mehr Abdeckung
mit Genauigkeit -- ein Tausch, kein Gewinn.

HEIKOS BEOBACHTUNG ZUR AUFLOESUNG, nachgerechnet: der Abstand V3 zu V1 faellt von 3,22 Grad
(dx=40) auf 0,68 Grad (dx=20), also Faktor 4,7 bei halber Zellgroesse, waehrend V1 selbst sich
nur um Faktor 1,4 verbessert. Hochrechnen ist nicht zulaessig; eine dritte Sprosse entscheidet.

### B43 -- WARNUNG: BESSERE ABDECKUNG IST NICHT BELEGT BESSERE PHYSIK
Heiko fragte, ob die bessere Abdeckung heisst, dass das Wandmodell genauer arbeitet. NEIN, und
das ist ausdruecklich festzuhalten, weil der Schluss naheliegt und falsch waere:
 1. Alles bisher ist reine Diagnose. Der Loeser rechnet unveraendert mit V1 (Bitgleichheit in
    jedem Lauf bestaetigt, FELD-HASH 4722579264326613690). Es gibt keine Cd-, Cz- oder
    u_tau-Zahl mit den neuen Normalen.
 2. Abdeckung ist notwendig, nicht hinreichend. Dass der Bounce-Back-Rueckfall schadet, ist
    gemessen; daraus folgt nur, dass FEHLENDE Facetten schaden, nicht dass JEDE hilft.
 3. Eine falsch stehende Facette kann SCHLECHTER sein als keine: sie bucht Kraft in die falsche
    Richtung, waehrend Bounce-Back keine falsche Richtung behauptet. Und die Zellen, die V3b
    neu zulaesst, sind gerade die mit kleinem y_w -- dort reagiert das Wandgesetz am staerksten.
ENTSCHEIDBAR NUR DURCH VERDRAHTEN UND MESSEN: kipp0 (darf sich nicht aendern), kipp26 gegen
u_tau IST/Ziel 2,382, dann Fahrzeug 8 mm gegen cd_druck 1,5747 +-0,0126 und Cz_rest
-0,0942 +-0,0146 (OF13 -1,301).

### B44 -- DRITTE KUGEL-SPROSSE: V3 UEBERHOLT V1 IM MEDIAN; V1s FEHLER SAETTIGT, V3s NICHT
Heikos Beobachtung ("der winkelfehler faellt mit der aufloesung") nachgemessen statt
hochgerechnet. Lauf tb_kugel_dx10, 9178 wandnahe Zellen, R_vol 23,23 Zellen. Census frei.

  Kugel        Radius   V1 Median/q90    V3 = V3b Median/q90   Abstand im Median
  dx=40         6,30 Z    6,67 / 11,50      9,89 / 12,96        V3 schlechter um 3,22
  dx=20        11,95 Z    4,85 /  9,81      5,53 /  9,43        V3 schlechter um 0,68
  dx=10        23,23 Z    4,69 /  8,78      4,54 /  9,87        V3 BESSER um 0,15

DER EIGENTLICHE BEFUND STECKT IN DER KONVERGENZ, NICHT IM UEBERHOLEN:
  V1: 6,67 -> 4,85 -> 4,69  (Verbesserung um Faktor 1,38 dann nur noch 1,03) -- SAETTIGT
  V3: 9,89 -> 5,53 -> 4,54  (Faktor 1,79 dann 1,22) -- KONVERGIERT WEITER
V1 hat einen Fehlerboden, V3 nicht. Erklaerbar aus der Konstruktion: V1 mittelt auch ueber die
12 DIAGONAL-Linkmitten, und die liegen bauartbedingt NICHT auf der Voxeloberflaeche (sie
streifen die Solid-Ecke, vgl. den Kommentar setup.cpp:1112). Dieser Versatz ist ein
SYSTEMATISCHER Beitrag, der mit feinerem Gitter nicht verschwindet. V3 nutzt ausschliesslich
die echten Voxelflaechen, sein Fehler ist reine Diskretisierung und faellt weiter.
DAS IST DAS ERSTE ARGUMENT FUER V3, DAS AUF DER PRODUKTIONSSPROSSE STAERKER WIRD STATT SCHWAECHER.

EHRLICH ZUR GEGENRICHTUNG: das q90 kreuzt NICHT mit. Bei dx=20 ist V3 im q90 besser
(9,43 gegen 9,81), bei dx=10 schlechter (9,87 gegen 8,78). Median und q90 laufen also
auseinander -- V3 hat den besseren Schwerpunkt, V1 den kuerzeren Schwanz. Ein Verfahren, das
in BEIDEN Massen gewinnt, gibt es nach diesen drei Sprossen nicht.
Abdeckung bei dx=10: V1 3,40 %, V3b 2,35 % -- auf der glatten Kugel spielt sie keine Rolle.

### B45 -- PRUEFAGENT-BEFUNDE BEHOBEN; DIE PRODUKTIVE ZAHL LIEGT VOR (22,1 -> 7,0 %)
Unabhaengige Pruefung gegen den Diff 98a5634..db7f0ee: ein HARTER Befund, acht mittlere.

H1 (HART): die Vergleichstabelle stellte V1 NACH der Glaettung gegen V2..V3b DAVOR. F[i].klasse
ist der Endzustand (die Glaettung bewertet K4 neu), kl2v..kl6v stammen aus der Zellschleife.
Damit enthielten 21,88 -> 7,26 % einen unbekannten Glaettungsanteil, und die 7,26 % beschrieben
NICHT, was der Loeser bekommt. BEHOBEN: kl1v haelt V1s Klasse vor der Glaettung, die Tabelle
zeigt beide Zeilen; die produktive Zahl kommt aus einem Lauf mit aktivem V3b.
GEMESSENE GROESSE DES BIAS (td_vgl_8mm): V1 vor der Glaettung 165.481 = 21,91 %, danach
165.239 = 21,88 %. Der Glaettungsanteil sind 242 Zellen = 0,03 Prozentpunkte. Der Befund war
methodisch richtig, seine praktische Wirkung ist klein -- aber jetzt gemessen statt angenommen.

DIE PRODUKTIVE ZAHL, td_prod_8mm, Fahrzeug 8 mm, CFD_FACETTEN_NORMQUELLE=1, gelesen an der
Klassen-Zeile (post-Glaettung), also das, was der Loeser wirklich bekaeme:
  V1 (t7_basis_heute / td_vgl_8mm)  markierte Zellen 166.677 = 22,1 %
  V3b (td_prod_8mm)                 markierte Zellen  52.717 =  7,0 %
  im Detail V3b: K1 0, K2 0, K3 0, K4 49.442, Orientierung 6.952, bewegte Wand 1.274
  gegen V1:      K1 0, K2 95.953,   K3 0, K4 100.891, Orientierung 23.034, bewegte Wand 1.274
FAKTOR 3,2 WENIGER VERWERFUNG. Nebenbefund, nicht vorhergesagt: die ORIENTIERUNGSKONFLIKTE
fallen von 23.034 auf 6.952 (-70 %). Die bewegte-Wand-Zahl bleibt exakt gleich (1.274) -- sie
ist reine Geometrie und darf sich nicht bewegen, das ist eine gute Gegenprobe.
Wirkpfad: 755.344 von 755.344 Facetten aus der Voxelflaechen-Summe, Duennteil-Rueckfall 2.233 --
exakt V3s frueherer K1-Zahl. Die Zuordnung "entartete Summe = einzellige Teile und Spalten"
ist damit zum dritten Mal unabhaengig bestaetigt.

BITANKER ZWEIMAL GEHALTEN: td_bitanker und td_kipp0_v3b liefern beide FELD-HASH
4722579264326613690. Der Pruefagent hatte V3b am ebenen Kanal von Hand nachgerechnet (nq=9
Flaechen, Summe (0,0,9), y_w exakt 0,5, cx_ identisch mit V1s Schwerpunkt) und daraus ein
hartes Abnahmetor abgeleitet -- es haelt.

BEHOBENE MITTLERE BEFUNDE: M2 unbelegte Schalterwerte warnten "V3b aktiv" und rechneten V1
(jetzt print_error). M3 der Laengenwaechter prueft jetzt alle 17 Parallelvektoren und BRICHT AB
(vorher 2 von 15, und er lief nach dem Fehler weiter -> undefiniertes Verhalten). M4 Gates im
K1-Zweig an den Normalpfad angeglichen. M5 die beiden Duennteil-Messwege hatten GEGENSAETZLICHE
Randkonventionen -- am kipp0-Kanal haette der Waechter faelschlich Alarm geschlagen. M6 der
gekoppelte Waechter lief im Produktionsarm gar nicht mit. M8 die Schalter-Ansage nennt jetzt
auch den fac_geo-Flaechenfaktor und den ELIBB-Ebenen-q als Wirkpfade. M9 die Kugel-Grundwahrheit
konnte still entfallen.
OFFEN (M7): bei NORMQUELLE=1 ohne VERGLEICH werden V2/V4/V5 unnoetig mitgerechnet, rund
131 MB Ballastspeicher am Fahrzeug 4 mm und geschaetzt +40..80 % auf den Facettenbau. Reine
Setup-Kosten, kein Laufzeiteffekt -- nicht behoben, um vor der Physikmessung keine
Umstrukturierung einzuziehen.

### B46 -- DIE KUGEL-Cd-ABWEICHUNG IST NICHT ENTSCHEIDBAR; MEIN 5-PROZENT-KRITERIUM WAR UNGUELTIG
Unabhaengige Untersuchung der -9,0 % an Cd_druck (b2s1_kugel 0,6648 gegen tc_kugel_v3b 0,6049).

1) WIRKWEG, korrigiert: die Normale geht sehr wohl in den DRUCK ein, aber nur als
   Projektionsrichtung -- setup.cpp:2009-2010 bzw. kernel.cpp:3735-3736 rechnen
   fn = F*n_dach; K.px += fn*n_dach_x. Der Kraftvektor F selbst (Impulsaustausch, 1045 Zellen,
   in beiden Armen identisch) bleibt unberuehrt. Der FLAECHENFAKTOR 1/|n_achse| (lbm.cpp:521-525)
   steht NICHT im Druckpfad, sondern nur im Reibungs-/Wandmodellpfad (kernel.cpp:1682/1697).
   FOLGE, und das ist der Kern: p_x = (F*n)n_x faellt mit <cos^2 theta>. Cd_druck ist ein nach
   unten verzerrter Schaetzer, dessen Verzerrung MIT DEM NORMALENFEHLER WAECHST. "Niedriger"
   heisst hier "staerker verzerrt", nicht "besser".
2) QUANTITATIV: die Geometrie traegt nur -1,4 bis -2,3 % (aus dem Winkelfehler gegen die
   analytische Normale, V1 6,67 gegen V3b 9,89 Grad). Der Rest kommt aus dem STROEMUNGSFELD:
   Rang0-BB steigt 7.775 -> 9.572 (+23,1 %), Anteil am Wirkpfad 29,0 -> 34,5 %. V3bs Normalen
   lassen den iMEM-Solve haeufiger entarten; diese Zellen buchen keine Modellreibung.
3) KEINE REFERENZ MOEGLICH: der Lauf steht bei Re_D = 912.162, also UEBERKRITISCH; Achenbach
   1972 / Clift-Grace-Weber erwarten dort Cd ~ 0,07-0,10 (FACETTEN-LITERATUR.md:44-46). Gemessen
   sind 1,0796 bzw. 0,9838. Beide Arme liegen um eine Groessenordnung daneben -- eine
   9-%-Verschiebung darin kann die Literatur nicht bewerten.
4) MEIN B36-ARGUMENT IST WIDERLEGT, durch Gegenbeispiel: das 5^3-Fenster schiebt y_w um +28 %
   (weg von der Voxelflaeche), V3b um -41 % (hin zu ihr) -- ENTGEGENGESETZTE Geometrievorzeichen,
   und beide senken Cd_druck aehnlich stark (-7,4 gegen -9,0 %). Die Regel "naeher an der
   Voxelflaeche also niedrigerer Druckwiderstand" gilt nicht. Was bleibt: JEDE Stoerung des
   Wandmodells senkt hier Cd_druck um 7-9 %.
5) DIE 9 % STEHEN AUF DER FALSCHEN SPROSSE: der einzige Stroemungslauf ist dx=40, und dort ist
   V3bs Normalenfehler am groessten (9,89 gegen 6,67 Grad; bei dx=10 dreht es auf 4,54 gegen
   4,69). Fuer die Produktionsaufloesung sagt die Zahl nichts.
6) DER ENTSCHEIDENDE BEFUND -- MEIN KRITERIUM WAR VON ANFANG AN UNGUELTIG: das Drucksignal
   dieses Kugelfalls ist eine ZWEI-ZUSTANDS-OSZILLATION. Gerade Samples 0,0892, ungerade 1,2404
   (V1), Amplitude also 87 % des Mittelwerts; Autokorrelation Lag 1..8 alterniert mit +-0,9.
   Grundperiode 4 oder 20 Zeitschritte -- 40- bis 200-mal schneller als die Abloeseperiode
   (St~0,2 entspricht ~750 Schritten). Das ist kein aufgeloestes Stroemungsmerkmal.
   Der gemeldete Block-SEM (2 sigma = 1,9 %) misst das nicht; ueber die Paarmittel mit
   integrierter Autokorrelationszeit sind es 2 sigma = 5,5 % (V1) und 4,5 % (V3b), kombiniert
   6,8 %. MEIN VORAB-KRITERIUM VON 5 % LAG ALSO UNTER DER EIGENSTREUUNG DES FALLS. Die -9,0 %
   sind rund 2,6 sigma -- knapp ueber der Schwelle, nicht komfortabel, und kein Kill.
   Am Fahrzeug existiert diese Oszillation NICHT (export/b8_kontrolle/forces.csv: Cd 10,175
   gegen 10,027) -- sie ist ein Spezifikum dieses Kugelfalls.
7) AUSGESCHLOSSEN: Zellenzahl im Kraftpfad (1045 in beiden), Commit-Drift (der V1-PCA-Pfad ist
   zwischen 2a50a7d und db7f0ee Byte fuer Byte unveraendert, bestaetigt durch identische
   V1-Geometrie im spaeteren Lauf). REAL und mitwirkend: Klassenmaske (aktive Facetten
   725 -> 749) und der Grazing-Guard (2541 -> 2445 ELIBB-Links, -6,9 %).
8) NEBENBEFUND: Cz bewegt sich staerker als Cd (1,0604 -> 1,2652, +19,3 %, gegen Cd -8,9 %).
   Wer nur Cd_druck ansieht, sieht die halbe Verschiebung.

FOLGERUNG: die -9,0 % sind KEIN Befund ueber V3b. Bevor an diesem Fall ueberhaupt ein Kriterium
gelten kann, muss die Eigenstreuung des AUSGANGSWERTS bekannt sein.

### B47 -- KORREKTUR AN B45: DER VERWERFUNGSGEWINN IST NUR ZUR HAELFTE PHYSIK
Unabhaengige Untersuchung des Rang0-Anstiegs. Ergebnis kippt die Bewertung von B45.

1) DIE ENTARTUNG HAENGT NICHT VON DER NORMALEN AB, sondern allein von der Zahl der eigenen
   Solid-Links der Zelle. Die Momentenmatrix M = Summe 6*w_i*c_i c_i^T (kernel.cpp:1996-2009)
   entsteht ausschliesslich aus flags; ihr Rang ist basisinvariant. Eine Ein-Link-Zelle hat
   analytisch Gt == 0 und faellt DETERMINISTISCH, bei jedem Zeitschritt, in Slot 13 oder 15.
   BELEG, exakte Arithmetik, von mir am Fahrzeug unabhaengig nachgerechnet:
     aktive Facetten (klasse==0) mit genau 1 Eigenlink: 94.236
     94.236 x 250 Abtastungen = 23.559.000 gegen geloggtes ohneTang 23.558.999 -- Abweichung 1.
   An der Kugel ebenso: V1 216x37 = 7.992 = Slot 13+15, V3b 264x37 = 9.768. Slot 14 (Rang2)
   = 48x37 = die Zellen mit genau ZWEI Eigenlinks.
   Die +23,1 % Rang0 sind damit KEINE neue Entartung, sondern eine NENNER-UMBUCHUNG: es sind
   genau die Ein-Link-Zellen, die V1 als K2 verworfen hatte und die V3b hereinnimmt.
   Meine Hypothese (V3bs diskretere Normalenrichtungen treffen haeufiger die entartete
   Ausrichtung) ist WIDERLEGT: der zugehoerige Zaehler u_t~0-Skips steht auf 2 in BEIDEN Armen.

2) DAMIT UEBERZEICHNET B45s SCHLAGZEILE. Host-Zensus von export/td_vgl_8mm/nah/
   facetten_histogramme.csv: von den 64.618 Zellen, die V1 NUR wegen Kante/Linie/Orientierung
   verwirft -- also denen, die V3b sicher hereinholt --
     haben 24.863 genau EINEN Eigenlink   -> unter ALPHA=2 entartet, kein Wandmodell moeglich
     haben  5.415 genau ZWEI Eigenlinks   -> ebenfalls entartet
     bleiben 34.340 = 53,1 %              -> nur diese koennen ein Wandmodell tragen
   Der physisch nutzbare Gewinn ist also rund die HAELFTE dessen, was "22,1 -> 7,0 %" nahelegt,
   und der K4-Verlust ist davon noch abzuziehen.

3) DAS RICHTIGE ERFOLGSMASS ist nicht die Verwerfung, sondern die Zahl BEHANDELTER Facetten:
   Slot 7 (Wirkpfad) minus Slot 9 (u_t~0) minus Slot 69 (Rueckfall-Buchung), geteilt durch die
   Abtastzahl. An der Kugel: V1 (26.825-2-10.746)/37 = 434,5 von 725 Facetten;
   V3b (27.713-2-12.135)/37 = 421,0 von 749. V3B BEHANDELT DORT 3,1 % WENIGER ZELLEN, OBWOHL ES
   3,3 % MEHR FACETTEN HAT. Verwerfungsrate und Rang0 sind beide Nennerartefakte; diese Zahl
   ist es nicht -- und sie steht am Kugelfall gegen V3b.
   EINSCHRAENKUNG: die Kugellaeufe hatten CFD_FAC_ALPHA ungesetzt (=0), die Produktion faehrt 2.
   Die Fahrzeug-Identitaet oben gilt dagegen unter ALPHA=2.

4) DIE LOESUNG BRAUCHT KEINEN CODE, SONDERN EINEN SCHALTER. Ein-Link-Zellen sind fuer den
   iMEM-Solve prinzipiell unerreichbar (ein Link = ein Freiheitsgrad; jede Erfuellung injiziert
   Normalimpuls, kernel.cpp:2190). Fuer die ELIBB-Blende sind sie voll behandelbar: sie laeuft
   VOR dem Solve (kernel.cpp:1913), braucht nur q je Link und keinen Rang -- aber ausschliesslich
   fuer Zellen, die eine FACETTE SIND (fid==0xFFFFFFFF -> return, kernel.cpp:1899). Eine
   verworfene Zelle bekommt gar nichts.
   DER BEZUGSLAUF t7_basis_heute HATTE ELIBB AUS. Ohne ELIBB kann V3bs Gewinn strukturell nicht
   sichtbar werden -- dann bekommen die Ein-Link-Zellen in BEIDEN Armen nichts, und die
   Umbuchung ist alles, was uebrig bleibt. Das A/B faehrt deshalb ELIBB in BEIDEN Armen an.
   NICHT ANZUFASSEN: kernel.cpp:2155 (Entkopplungs-Gate) und 2176-2190 (relative Kaskaden-
   schwellen). Die absoluten 1e-8-Schwellen lagen exakt auf dem float-Schur-Rauschen und liessen
   31 % der Kugelfacetten ins Rang-2 flackern; ein Aufweichen stellt genau diesen Fehler wieder her.

### B48 -- BUCHUNGSFRAGE ALS BEANTWORTET ERKLAERT (Heiko-Entscheid 28.08., 12:10)
Die Bestaetigungssprosse ti_kipp26_v3b_640 wurde bei 65 % sauber abgebrochen (Census danach:
kein Prozess, kein Lock). Grund: die Frage ist beantwortet, die Sprosse war Formalismus gegen
meine eigene Schwelle.
STAND, DER DAMIT GILT -- ausdruecklich deklariert statt stillschweigend uebernommen:
  Verhaeltnis Reibungspfad/Kraftbilanz  80 ETT 0,8853 | 160 ETT 0,9551 | 320 ETT 0,9846
  y-Reibung                             +0,1074 | +0,1901 | -0,0066  (V1-Niveau: -0,0171)
Meine vorab gesetzte Schwelle war >0,99; erreicht sind 0,9846. Die Buchungsluecke ist damit
NICHT formal abgenommen, sondern als Transienz eingeordnet -- gestuetzt auf drei unabhaengige
Zeichen: die monotone Drittelung je Verdopplung, den Zusammenfall der y-Reibung auf V1-Niveau,
und die Tatsache, dass V1 dieselbe Pruefung bei 80 ETT bereits besteht (1,0004), das Modell also
nicht die Bilanz verletzt, sondern langsamer einschwingt.
WER SPAETER DARAUF AUFBAUT, MUSS DAS WISSEN: die 1,5 % Restluecke sind gemessen, nicht erklaert
weggerechnet. Sollte am Fahrzeug etwas in dieser Groessenordnung unklar bleiben, ist die
640-ETT-Sprosse der erste Ort, an dem nachzusehen ist.

### B49 -- DER ECHTE FAHRZEUGTEST: V3b DECKT MEHR AB UND MACHT DEN ABTRIEB SCHLECHTER
Serie logs/tj_fahrzeug_ab_serie.txt, drei Arme, jedes Paar EINE Variable, alle rc=0, Census
vor und nach der Serie frei, alle bei T_END 1,0 s, ELIBB in allen dreien an.
Boxen wie bestellt gerastert: neu 845x333x233 (65,6 M), alt 845x317x241 (64,6 M) -- y 317->333
Zellen (+128 mm), z 241->233 (-64 mm). Kopplungspruefung in allen drei Armen in Ordnung.

  Arm                 behandelte Facetten     cd_druck (2.H, Block-SEM)   Cz_rest vs OF13 -1,301
  a  V1  / NEUE Box   343.640 = 44,8 %        1,5515 +- 0,0055            -0,1552  (-88,1 %)
  b  V3b / NEUE Box   403.068 = 52,6 %        1,4908 +- 0,0035            -0,0700  (-94,6 %)
  c  V1  / ALTE Box   343.330 = 45,5 %        1,5270 +- 0,0047            -0,1548  (-88,1 %)

1) V3b GEWINNT AUF DEM EHRLICHEN MASS: +59.428 behandelte Facetten = +17,3 %. Das ist die
   Groesse, auf die B47 den Vergleich umgestellt hat (Slot 7 - Slot 9 - Slot 69, geteilt durch
   500 Abtastungen), nicht die Verwerfungsrate. UND SIE WIDERSPRICHT DEM KUGELFALL: dort
   behandelte V3b 3,1 % WENIGER. Die Kugel bei 12,6 Zellen Durchmesser war also kein
   uebertragbarer Zeuge -- genau wie der Cd-Agent gewarnt hatte.
2) V3b VERLIERT AUF DER GROESSE, DIE AM MEISTEN WEHTUT: der Abtrieb halbiert sich mehr als,
   Cz_rest -0,1552 -> -0,0700, die Abweichung von OF13 waechst von -88,1 auf -94,6 %.
   Abtrieb ist die Groesse, in der dieses Modell ohnehin am schlechtesten steht; V3b macht die
   schlechteste Stelle schlechter.
3) cd_druck faellt mit V3b um 3,9 % (1,5515 -> 1,4908, kombiniert 9,4 sigma). Das sieht nach
   Annaeherung an OF13 (0,599) aus, ist aber KEIN Gewinnbeleg: nach B46 ist cd_druck ein nach
   unten verzerrter Schaetzer, dessen Verzerrung mit dem Normalenfehler waechst, und an der
   Kugel senkte JEDE Stoerung des Wandmodells diesen Wert um 7-9 %.
4) DIE BOX BRINGT NICHTS MESSBARES (a gegen c, eine Variable): Cz_rest -0,1552 gegen -0,1548 --
   identisch im Rahmen der Fehlerbalken. cd_druck 1,5515 gegen 1,5270, also +1,6 % (3,4 sigma)
   in die FALSCHE Richtung. Behandelte Facetten praktisch gleich (343.640 gegen 343.330).
   Die Umschichtung (y +128 mm, z+ -64 mm) war aus den Interface-Slices abgeleitet und ist damit
   erstmals am Kraftergebnis geprueft: sie zahlt sich dort NICHT aus. Das entwertet die
   Slice-Analyse nicht -- es zeigt, dass das |d|<1-m/s-Kriterium keine Kraftaussage traegt.
5) KONVERGENZ, aus dem Kraefteverlauf statt aus einer Annahme (Heikos Punkt): alle drei Arme
   driften von der ersten zur zweiten Haelfte nach unten -- a 1,5941 -> 1,5515, b 1,5400 ->
   1,4908, c 1,5840 -> 1,5270. Keiner ist auskonvergiert, aber V3b driftet NICHT staerker als
   V1. Die vierfache Einschwingzeit vom Kanal UEBERTRAEGT SICH NICHT auf das Fahrzeug.
6) NEBENZAHLEN: V3b hat K2 = 0 (gegen 101.972), Orientierung 7.176 (gegen 25.598), markierte
   Zellen 6,9 % (gegen 22,6 %), Duennteil-Rueckfall feuerte 3.245 mal. Der Wirkpfad meldet
   766.234 von 766.234 Facetten aus der Voxelflaechen-Summe.

VERDIKT: V3b ist auf der Abdeckung belegt besser und auf dem Abtrieb belegt schlechter. Als
Default kommt es damit NICHT in Frage. Die naechste Frage ist nicht "annehmen oder verwerfen",
sondern WARUM mehr behandelte Zellen weniger Abtrieb ergeben -- das ist der eigentliche Befund
dieses Tages und war ohne den Fahrzeuglauf nicht sichtbar.

### T1 -- OFFENE TODO: horizontale Artefakte zwischen Dach und Heck im Nahfeld-Slice
Heiko 28.08. beim Ansehen von export/tj_8mm_v3b/schnitt_nah_001000ms.png: "eine todo die wir am
near slice sehen, waere das artefakt zwischen dach und heck welche so horizontal laufen...habe
ich immer wieder mal gesehen, deshalb wurde der slice immer mit einem leichten offset von
y=2.5cm oder so gemacht. die ursache fuer diese artefakte wurde jedoch noch nie untersucht."
Sein Verdacht: Zusammenhang mit dem Blockage-Artefakt des 4-mm-Laufs vom 27.08. (B10/B12/B14,
rund 0,11 Cd, Ursache offen, nicht-deterministisch), das er "vermehrt bei sparse tiles" gesehen
hat und das "irgendwie was mit dem voxelieren zu tun" haben koennte.

SOFORT EINGEGRENZT, ohne neuen Lauf:
  CFD_SPARSE_TILES war in den heutigen Armen NICHT gesetzt -- und kann im Fahrzeugfall gar nicht
  wirken: lbm.cpp:421 bricht bei mehr als einer Domaene hart ab ("nur fuer eine einzelne GPU
  validiert"), der dd-Fall ist zweidomaenig. Das Artefakt ist in tj_8mm_v3b also OHNE Sparse
  Tiles vorhanden. Sparse Tiles koennen es demnach hoechstens VERSTAERKEN, nicht verursachen.
  Damit faellt eine der beiden Spuren aus -- und die Voxelierungsspur bleibt.

WAS ZU PRUEFEN WAERE, in dieser Reihenfolge und ohne Bildauswertung (Iron Rule 5):
 1. ABSTAND DER BAENDER. Liegen die horizontalen Streifen auf einem festen Zellabstand? Ein
    Vielfaches der Kachelgroesse deutete auf eine Block-/Kachelstruktur, ein Abstand von einer
    Zelle auf die Voxeltreppe. Messbar an einer z-Saeule aus den Feld-CSVs, nicht am Bild.
 2. DIE y-ABHAENGIGKEIT. Der Ausweichoffset von 2,5 cm ist der beste vorhandene Hinweis: das
    Artefakt ist offenbar nicht y-invariant. Eine Saeulenschar bei mehreren y wuerde zeigen, ob
    es an der Symmetrieebene haengt oder wandert.
 3. DIE GEOMETRIE DORT. Zwischen Dach und Heck ist die Karosserie flach und schwach geneigt --
    die Voxelierung erzeugt dort lange waagerechte Laeufe mit vereinzelten Einzelstufen. Das ist
    genau die Population, die heute vermessen wurde: solid_dicke und r21 liegen je Zelle in
    export/tj_8mm_v1/nah/facetten_histogramme.csv, samt Zellindex n. Eine Auswertung nur auf
    diesen Bereich zeigt, ob dort etwas Auffaelliges sitzt.
 4. ERST DANN die Verbindung zum Blockage-Artefakt pruefen -- getrennt, wie Heiko sagt.
NICHT VERMISCHEN mit der laufenden V3b-Untersuchung: das Artefakt ist in V1 und V3b gleichermassen
zu erwarten (es ist aelter als beide), und die drei Arme von heute liegen als Vergleichsmaterial
bereits vor.

## 2026-08-28 ABSCHLUSS -- die Facettenquelle, ein entwerteter Vormittag und was daraus folgt

### B50 -- DIE MESSREIHE, ZUM ERSTEN MAL EINVARIABEL
Vier Arme auf der uebersetzten Baseline (basis/fahrzeug_dd.basis, 8 mm, Rueckkopplung und
UTKORR an, T_END 1,0 s deklariert), jeder mit EINER Variablen gegen Arm a. Bitanker vor und
nach jedem Umbau gehalten (FELD-HASH 4722579264326613690).

  Arm                     Cz_rest  +-SEM    vs a      cd_druck  +-SEM
  a  Bezug (V1)            0,4102  0,0068     --        1,5217  0,0036
  b  nur Normale           0,4166  0,0077   0,6 sigma   1,5367  0,0033
  c  nur y_w-Anker         0,5091  0,0091   8,7 sigma   1,6634  0,0026
  d  Kantentest AUS        0,4304  0,0056   2,3 sigma   1,5064  0,0044
  OF13 mr2v40H: Cz gesamt -1,301

1) DIE NORMALENQUELLE IST KRAFTNEUTRAL (0,6 sigma). Damit ist die Schlagzeile vom Vormittag
   ("V3b verschlechtert den Abtrieb") widerlegt -- sie war keine Aussage ueber die Normale,
   sondern ueber die fuenf anderen Dinge, die im selben Schalter steckten.
2) DER y_w-ANKER IST DER HEBEL, und er schadet: 8,7 sigma, cd_druck +9,3 %. Das war MEINE
   Zutat (uebernommen aus V4b, um K4 zu senken -- das gelingt: 100.891 -> 42.266). Ich habe
   dabei die physikalische Groesse ausgetauscht statt sie genauer zu schaetzen.
3) DER KANTENTEST liegt mit 2,3 sigma dazwischen. Meine Hypothese, die Kantenzulassung sei der
   Hauptschaden, ist damit NICHT bestaetigt.
4) Cz GESAMT IST IN ALLEN VIER ARMEN POSITIV (+0,41 bis +0,51) gegen OF13 -1,301. Das Modell
   erzeugt Auftrieb statt Abtrieb; keiner der drei Hebel dreht das Vorzeichen. Wir vergleichen
   Nuancen innerhalb eines Regimes, das qualitativ nicht stimmt.
   ACHTUNG: CFD_KRAFT_ZBAND ist jetzt 2 statt 4 (Feingitterzellen, haelt das Latschband bei
   16 mm wie bei 4 mm). Cz_band faellt damit von 0,607 auf 0,0008 -- die Cz_rest-Zahlen dieser
   Reihe sind mit denen vom Vormittag NICHT vergleichbar. Der Code warnt selbst davor.

### B51 -- WAS HEIKOS URSPRUENGLICHE IDEE WAR, UND WAS DAVON GEBAUT WURDE
Beim Nachlesen seiner eigenen Worte: die erste Formulierung lautete "koennen wir nicht MEHRERE
zellangrenzende Facetten pro Zelle nutzen", spaeter "durch den schnitt aller dreiecke die die
zelle beruehren sowohl an KANTEN wie auch an FLAECHEN gute ergebnisse".
GEBAUT wurde EINE gemittelte Normale je Zelle. Ich habe die Idee vom ersten Moment an auf einen
Wert zusammengefaltet, weil die Datenstruktur eine Facette je Zelle haelt -- und das nie
hinterfragt. An einer Kante ist eine gemittelte Normale aber das falsche Objekt: zwei unter
90 Grad zusammenstossende Flaechen mitteln sich zu einer 45-Grad-Richtung, die auf keiner von
beiden liegt. Die gemessene Kraftneutralitaet (0,6 sigma) passt dazu -- die Mittelung kann an
genau der Stelle nichts ausrichten, an der der Gewinn liegen sollte.
NICHT GEPRUEFT, WEIL NIE GEBAUT: mehrere Facetten je Zelle. Kostenanalyse laeuft.

### B52 -- DER TEURE FEHLER DES TAGES: KONFIGURATION AUS EINEM ALTSTAND REKONSTRUIERT
Alle Fahrzeugzahlen vom Vormittag sind entwertet. Ich hatte die 8-mm-Arme aus f8_standard_final
rekonstruiert -- Name klang nach Referenz, Lauf war 7 Tage und 69 src/-Commits alt. Dabei
fehlten ELF Schalter, darunter die komplette Nah->Fern-Rueckkopplung und CFD_FAC_UTKORR=1.5.
Gemessener Preis der fehlenden Kette: rund 12 % Abtrieb (8 mm mit Kette cz_druck_rest -0,6458,
ohne -0,5666; der dokumentierte 4-mm-Effekt liegt bei 0,084).
ERSCHWEREND: Heiko hatte baseline_2026-08-27_f4vollumfang ausdruecklich als "Referenz fuer alle
folgenden A/Bs" deklariert, und es stand in AUDIT-BEFUNDE.md:3053. Ich hatte es selbst notiert
und dann ignoriert. Und ich hatte die 69-Commit-Luecke desselben Laufs am SELBEN Vormittag
gefunden und die Kraftvergleiche darueber verweigert -- aber die Konfiguration uebernommen.
Der Fehler war nicht, dass mir etwas entgangen ist, sondern dass ich zwei Dinge nicht
zusammengebracht habe, die beide vor mir lagen.
Heiko sah es am Slice ("die rueckkopplung ist offensichtlich deaktiviert"), nicht ich.

### B53 -- WAS GEBAUT WURDE, DAMIT ES NICHT WIEDERKOMMT
1) BASIS-WAECHTER (setup.cpp, erste Anweisung von main_setup_fahrzeug_dd). Prueft jede
   dd-Konfiguration gegen basis/fahrzeug_dd.basis. Er FORDERT KEINE GLEICHHEIT, er RECHNET UM:
   aus der Einheit und dem CFD_DX des Laufs leitet er den Sollwert ab (WAKE_START_X 311 -> 156,
   ZBAND 4 -> 2, SPONGE_N 64 -> 32, WAKE_ABSTAND 32 -> 16). Bei Uneindeutigkeit nennt er beide
   Kandidaten. FEHLENDE Schalter meldet er genauso hart wie abweichende -- das ist die Haelfte,
   die im entwerteten Lauf gefehlt hat; die vorhandene Pruefung schaute nur in die Gegenrichtung.
   Wirkpfad bewiesen: auf der Vormittagskonfiguration zehn fehlende, zwei abweichende, Abbruch.
   Die Referenz wird MASCHINELL erzeugt (werkzeuge/basis_aus_lauf.py) -- eine handgepflegte
   weicht irgendwann von dem ab, was tatsaechlich lief, und genau das war schon passiert.
2) SCHALTERZERLEGUNG: NORMQUELLE (nur die Normale), YWQUELLE (der y_w-Anker), KANTE (die
   Kantenschwelle, quellenunabhaengig). Vorher aenderte ein Schalter sechs Dinge zugleich.
3) SLICE-AUSGABE IST PFLICHT (Heiko: "kostet nichts"). Die Baseline trug CFD_SLICE_DT=0 -- ein
   Defekt, den ich selbst eingebaut hatte und der sich ueber die Referenz in die ganze Reihe
   fortpflanzte. Jetzt harter Fehler im Code, Korrektur im Generator, Vermerk im Referenzkopf.
   Ein Schalter ohne Physikwirkung kann eine Messreihe trotzdem unauswertbar machen.

### B54 -- KORREKTUR AN B50: cd_rest gehoert IMMER dazu, und es kippt die Aussage
Heiko 28.08. abends: "cd_rest ist immer zu rechnen". Ich hatte in B50 cd_druck berichtet und
daraus "die Normalenquelle ist kraftneutral" geschlossen -- das galt nur fuer Cz.

  Arm                    Cd_rest  +-SEM     vs a      Cz_rest   Cd_band
  a  Bezug (V1)           6,3034  0,0110      --       0,4102    0,8025
  b  nur Normale          6,4777  0,0142    9,7 sigma  0,4166    0,8013
  c  nur y_w-Anker        6,5052  0,0133   11,7 sigma  0,5091    0,8021
  d  Kantentest AUS       6,1525  0,0200    6,6 sigma  0,4304    0,8014
  OF13 mr2v40H: Cd 0,599 / Cz -1,301

1) DIE NORMALENQUELLE IST NICHT KRAFTNEUTRAL. Auf Cz war sie es (0,6 sigma), auf Cd_rest
   verschlechtert sie um 9,7 sigma (+2,8 %). Mein Schluss in B50 stand auf EINER Groesse.
2) DER y_w-ANKER schadet auf BEIDEN Groessen (11,7 sigma auf Cd_rest, 8,7 auf Cz_rest).
   Das bleibt der klarste Negativbefund und betrifft meine Zutat, nicht Heikos Idee.
3) ARM D IST DER EINZIGE, DER ETWAS VERBESSERT: Cd_rest -2,4 % (6,6 sigma) Richtung OF13.
   Der Kantentest AUS senkt den Widerstand. Das widerspricht meiner Hypothese vom Nachmittag,
   V1s K2-Verwerfung sei ein wirksamer Kantenschutz gewesen -- auf Cd_rest ist sie ein Nachteil.
   Cz bewegt sich dabei um 2,3 sigma in die falsche Richtung; die beiden Groessen zeigen also
   GEGENLAEUFIG, und der Kantentest ist damit ein Zielkonflikt, kein Schutz.
4) Cd_band ist in allen vier Armen praktisch identisch (0,8013 bis 0,8025) -- das Latschband
   ist von allen drei Hebeln unberuehrt, die Wirkung sitzt vollstaendig darueber.
LEHRE, die ueber diesen Fall hinausgeht: eine Aussage ueber "die Kraft" aus EINER Kraftgroesse
zu ziehen, war voreilig. cd_rest und cz_rest gehoeren zusammen berichtet, auch wenn nur eine
davon in der Fragestellung steht.

---

# B55–B62 — Sechs-Agenten-Audit Grenzschicht/Bounce-Back, 29.08.2026

Heiko-Auftrag: "Code Audit auf den Bounce Back Kernel (min 5 Agenten), speziell auf Fehler
die eine Abloesung der Grenzschicht speziell hinter dem flach abfallenden Dach untersucht ...
Druckrelaxation, ob die Funktionen zueinander fuer unsere Aufloesung auch passen, ob unsere
neuen Dreiecksfacetten ueberhaupt ein Hebel fuer ein solches grobes Problem sein koennen."
Bezugslauf p4_v3b (4 mm, Commit 1c361e9). Massstab: cz_druck_rest −0,9418 gegen OF13 −1,3113,
also 0,36 fehlend IM DRUCKANTEIL; cz_reib +0,0408 gegen OF13 +0,0065.

## B55 — Die Dreiecksfacetten sind NICHT der Hebel (Agent 5)
Einvariabler 4-mm-A/B: Δcz_druck_rest = −0,0325 ± 0,0061 (5,4 σ) = 9 % der Luecke 0,41.
Der Abtrieb ist zu 95,6 % Druck; das gesamte physikalische Wandschubbudget des Fahrzeugs
betraegt C_|tau| = 0,0181 = 4,4 % der Luecke. Eine Normalenkorrektur, die ueberwiegend die
Reibung trifft, kann die 0,41 konstruktiv nicht holen. Dach+Heck tragen in der Reibung 0,95 %
der Luecke, der Dachabfall allein 0,21 %. — Der Ansatz ist trotzdem besser (Verwerfung
21,9 -> 7,3 %, siehe B59) und bewegt Cz in die richtige Richtung; er ist ein Instrumenten-
und Wandmodellhebel, kein Abtriebshebel.

## B56 — 53,2 % der Wandzellen ohne Wandmodell, Gate selbstverstaerkend (Agent 1)
19,91 % markiert (klasse!=0) + 41,60 % Laufzeit-Rueckfall (Slot 69). Das SATGATE feuert, wenn
|s1| > 2·ut, also wenn der BB-Uebertrag die Ziel-Schubspannung uebersteigt -- genau an der
Treppe. Es schaltet das Modell dort ab, wo die Treppen-Ueberreibung am groessten ist.
KEIN Formelfehler: Impulsbilanz, Vorzeichen, Faktor 2 und Schrittreihenfolge nachgerechnet
und korrekt (N4/N5). ELIBB verdreifacht die Gate-Rate (Kanal 40,2 % gegen 13,2 % ohne).

## B57 — Wandmodell, SGS und tau setzen dieselbe Wandspannung ohne Aufteilung (Agent 3)
tau = 0,50002832, Λ = (tau−0,5)² = 8,02e−10 gegen die Wandlage 3/16 -- 8,4 Dekaden darunter.
Smagorinsky C_s = 0,1733 fest verdrahtet, KEINE Wanddaempfung (kein van Driest, kein WALE).
tau_SGS/tau_Wandmodell = (1+nu_t/nu_0)/(kappa·y+); bei nu_t/nu_0 = 30 und y+ = 68 ist das 1,11
-- das SGS traegt dort bereits die volle Wandspannung, das Wandmodell setzt sie ein zweites Mal.
Netto kommt trotzdem ZU WENIG heraus: c_f = 5,9e−4 gegen Plattenkorrelation 2,73e−3, Faktor
4,6 zu klein, INKLUSIVE CFD_FAC_UTKORR=1,5. Wirkrichtung: Abloesung zu spaet, Cd zu klein --
gemessen cd_rest 0,5305 gegen 0,599 (−11 %), Vorzeichen stimmt.
delta/dx = 10,3 Zellen auf dem Dach: die Abloesung ist nicht aufloesbar, sie wird vom
Wandmodell diktiert. Projekteigene Herleitung c_f = 1,502/N² (WANDMODELL.md:108) -- es gibt
genau EINE Gitterweite, bei der es zufaellig stimmt: dx ~ 6,4 mm. 4 mm liegt darunter.

## B58 — Die Voxeltreppe erzwingt die Abloesung, und ELIBB ist dort abgeschaltet (Agent 4)
u_x in der ersten Fluidzelle entlang der flachen Rampe bricht an JEDER Setzstufe zusammen:
13 Zusammenbrueche auf 352 mm (i_x 742/758/768/775/781/788/795/804/808/815/820/829, Werte
−0,40 bis +0,96 m/s zwischen Spitzen von 6,5 m/s). Am flachen Dach ohne Stufen fehlt das
Muster. Durchgehende Abloesung ab x = 3,234 m -- 428 mm vor dem Ende der Heckscheibe, deren
wahre Sekante nur 17,59° betraegt (unter der kritischen ~30°). Setzstufe 4 mm bei delta
40-60 mm: k/delta = 7-10 %, ein Windkanal-Stolperdraht liegt bei 1-3 %.
q ist auf der ROHEN Voxelflaeche 0,5 auf ALLEN 15.114.074 Links. Nach der Glaettung liegen am
flachen Dach immer noch 99,6 % bei |q−0,5| <= 0,1 -- und bei q = 0,5 ist chi = 0, ELIBB also
bitgleich reines BB. Ursache steht im Code selbst (setup.cpp:884): Taubin daempft jedes
Merkmal unter ~11 Zellen Periode; die flachste Trittstufe ist 16 Zellen lang und ueberlebt.
GENAU UMGEKEHRT ZUM BEDARF -- je flacher die Rampe, desto weniger bewegt sich q.
Aufdickung gemessen: +0,97 Zellen am flachen Dach (= analytischer Wert 0,5 SAT + 0,5 BB),
+1,78 an der mittleren Rampe. NEGATIVBEFUND: keine Quetschkante am Dach oder an der
Abrisskante -- die 198 Inzidenz-4-Kanten sitzen an Radhaeusern, die 836 offenen am Latsch.

## B59 — Warum trotz Dreiecksfacetten nur 80 % Abdeckung, und der Fix
Zwei getrennte Ursachen, gemessen am Zensus von p4_v3b (3.275.381 Zellen):
- K4 (y_w), 459.224 Zellen, ALLE an der Untergrenze, Haeufung 0,14..0,18 = die diskreten
  y_w-Klassen der geneigten Treppe. Bei y_w = 0,16 ist Y ~ 850, mitten im gueltigen Bereich.
- K2 (Kante), 312.840 Zellen. Misst r21 = Guete des PCA-Fits, den V3b gar nicht benutzt.
  27,9 % davon sitzen auf massiven Bauteilen (Dicke >= 9), sind also echte Geometriekanten.
Mehrfachnennungen sind fast ausschliesslich K2+K4 (122.824 = 18,8 % der markierten) -- die
Treppe erzeugt BEIDE Verwerfungsgruende gleichzeitig, zwei Tests fuer ein Phaenomen.
Die y_w-Schwelle ist bereits aufloesungsunabhaengig (8 mm 13,4 %, 4 mm 14,0 % unter 0,2,
gleiche Klassen) -- der Fehler war das VERWERFEN, nicht die Einheit.
FIX gebaut (Commit 865e0c8): Klemme + Kohaerenz-Kantentest, 19,9 % -> 4,5 %.

## B60 — Nahfeldraender praegen rho hart auf, Randdaten stellenweise unphysikalisch (Agent 2)
Fuenf Dirichlet-Seiten schreiben rho UND u aus dem 16-mm-Gitter (kernel.cpp:3396), die
Rueckkopplung korrigiert NUR u (kernel.cpp:3497). interface_druck.csv: cp bis +1,33 auf der
Decke, +1,48 am Einlass -- ueber der Staugrenze, inkompressibel unmoeglich. band_bilanz.csv:
cp −8,7 im groben Wake-Kasten, achtfache Sogspitze. Kein Abstandswaechter fuer getriebene
Ebenen (setup.cpp:4547 prueft nur SCHNITT mit dem Fahrzeug). ΔCd ~ 0,075 = 14 %.
ENTLASTET mit Zahl: RHO_CLAMP (greift bei cp = ∓59, APG ist O(cp 0,5) -- Faktor 140 daneben),
BODEN_EQ/EINLASS_EQ (reichen 4-32 mm, Dach bei 1,20 m, praegen kein rho), SPONGE (beginnt
4,15 m hinter dem Heck), KOPPLUNG_GLATT (Uebertragung 0,99997 fuer die Fahrzeugsignatur).
D4 GEPRUEFT UND ENTLASTET am 29.08. abends: z-Schnitt gegen OF13 bei z = 1,836 m zeigt
RMS 0,70 m/s (y-Schnitt: 5,1) und |dU| zum Rand hin FALLEND (0,536 gegen 0,720 in der Mitte)
-- ein Randabdruck wuerde umgekehrt anwachsen. Werkzeug: werkzeuge/diff_of13_zslice.py.

## B61 — Der APG-Term ist nicht im Binaer, und er ist der einzige mit passender Richtung
FACETTEN_APG (kernel.cpp:1962) korrigiert tau_w um den Druckgradienten. CFD_FAC_APG ist nicht
gesetzt, das Log sagt es im Klartext: "Gleichgewichtsmodell, kein APG -- Abloeselage bleibt
modellfrei". Eine Gleichgewichts-Wandfunktion ueberschaetzt tau_w im Druckanstieg; als
Impulssenke saugt sie die wandnahe Schicht leer -> Abloesung frueher. BLOCKIERT: lbm.cpp:144
sperrt APG und ELIBB gegeneinander.

## B62 — Sechs Instrumente, die still falsch messen
- yplus_histogramm.csv ist NICHT y+: Median 1253,7 gegen 68,1 aus yplus_facetten.csv,
  FAKTOR 18. messe_yplus nutzt hartkodiert y = 0,5 und die tangentiale F-Komponente,
  druckkontaminiert an Voxeltreppen (setup.cpp:6247 sagt es selbst).
- RHO_CLAMP-Zaehler sind als "t%100-Stichprobe" dokumentiert (lbm.cpp:331, setup.cpp:648),
  sind aber ungegatet und saettigend -- FAKTOR 100 in der Lesart. Real ~445 Zellen je Schritt.
- SPONGE-Slots 29/66 saettigen bei 0xF0000000; der Klemmzaehler steht HOEHER als der
  Zonenzaehler, was unmoeglich ist. Die gemeldeten "100,0 %" sind ein Artefakt.
- "tau-Klemme 0" (Slot 8) ist ein TOTER Zaehler: die Schwelle tw_max = 0,5·rho·ut verlangt
  u_t > 0,2222·u+², im Logbereich also ~68 bei verfuegbaren 0,075 -- Faktor ~900 daneben.
- forces.csv Cd 8,64 ist "PHANTOM-Reibung" (eigener Dateikopf), NICHT dieselbe Groesse wie
  cd_rest 0,53 aus dem Facettenpfad. Nicht mischen.
- van Driest wurde gegen y+ = 137 verworfen (Plattenkorrelation). Gemessener Median: 68,1.
  Dort daempft er 14 %, im Mittel 23,5 %, auf 17,9 % der Facetten ueber 50 %. Kein No-op.

## Heiko-Freigabe 29.08. abends
A (Regressionssuche gegen 27.08.) GESTRICHEN -- "27.08 war aber gar kontaminiert, nicht
vergleichbar". B UEBERNOMMEN (steht in der Basis). C liegen lassen, erst die neue Abdeckung
testen. D2 selbststaendig geprueft: NICHT nachweislich falsch (die R-Tabelle stammt aus D1Q3
und gilt nur fuer die ebenen-gleichfoermige Mode; der Anker sitzt seit 08.08. auf dem
Flaechenmittel) -- nichts geaendert. D4 erledigt, siehe B60. E/F/G offen.
