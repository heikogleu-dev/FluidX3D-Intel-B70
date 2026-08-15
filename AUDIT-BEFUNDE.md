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
