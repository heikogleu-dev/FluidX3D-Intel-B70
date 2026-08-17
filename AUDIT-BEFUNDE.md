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
