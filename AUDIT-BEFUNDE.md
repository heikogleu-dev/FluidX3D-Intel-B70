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
