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
