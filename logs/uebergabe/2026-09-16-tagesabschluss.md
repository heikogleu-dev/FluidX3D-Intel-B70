# Übergabe 16.09.2026 — Tagesabschluss

**Stand bei Übergabe:** Klemmen-Block und APG-Linie abgeschlossen — der 4-mm-Voll-Lauf `p4_apg1` ist um 14:15 durchgelaufen (rc 0) und
bestätigt den 8-mm-Befund: **APG bewegt die Kräfte auch bei 4 mm nicht** (cd_druck_rest −0,0022 ± 0,0011, 150/300, 41 Vorzeichenwechsel;
Autorität ≥ 1 in 96,8 %; +17,0 % Wanduhr). **Empfehlung: endgültig parken** — Entscheid Heiko. Details PLAN-APG-2026-09-16.md §I.
Führende Liste bleibt `TODO.md`. Commits heute: 37e33dd … 0cfe6ac (Klemmen-Audit, APG-Bau, APG-Messung, Rückfragen).

## 1 · Was heute entschieden und belegt wurde

| Thema | Ergebnis | Beleg |
|---|---|---|
| Klemmen-Block | Audit-Schleife durch (4 Durchgänge, ~45 Befunde). Standard: `CFD_POSITIV=2` + `CFD_U_KLEMME=1`, `CFD_RHO_HUELLE` bleibt 0 | KLEMMEN-STUFE2-PLAN.md §2.5 |
| Klemmen bei 4 mm | **Keine Kraftwirkung** gegen `p4_register` (+0,005 ± 0,022, 168/300); Verhalten aber messbar anders: Geschwindigkeitsklemme ×2,35, Radkontakt-Defekt 234 → 5 Zellen | Protokoll 12:05 |
| iGPU-Leiter | +28,7 % gegen B24 (709 gegen 551 MLUPs); Fernfeld 81,7 % des 4-mm-Grobschritts; Nx%16 = 5,0 % | `logs/li_igpu_skala.txt` |
| APG-Bau | Vorkernel `fac_apg_ab` (grad ρ aus den DDFs der 6 Achsnachbarn), 2 Prüfdurchgänge, HOCH-1 behoben | Commits 5ecc95a, 63b6575 |
| APG bei 8 mm | **Keine Kraftwirkung** (κ=1: −0,009 ± 0,004; κ=0,5: −0,017 ± 0,003), Klemme in 98,5 % der Besuche, **−17,6 % Durchsatz** | PLAN-APG-2026-09-16.md §H |
| Sensitivität (neu) | τ_w-Modulation um ±100 % an 98,5 % der Facetten bewegt `cd_druck_rest` NICHT → **bei 8 mm ist τ_w kein Hebel auf den Druckwiderstand** | Protokoll 13:05 |

## 2 · Was läuft — und wie es ausgewertet wird

**`p4_apg1`** (gestartet 16.09. 13:11, B70, Freigabe Heiko 13:10): `p4_pu8`-Zeile **+ `CFD_FAC_APG=1.0`**, exakt eine Variable.
κ=1 ist die herleitbare Variante (Dünnschicht-Bilanz), 0,5 ein deklarierter Interim. VRAM 23 118 MB (+55 gegen `p4_pu8`), Schlupf 7041 MB.
Laufzeit-Erwartung über 47,9 min (Rechnung, nicht Messung: der 8-mm-Einbruch von 17,6 % fällt bei 4 mm relativ kleiner aus, weil die
Facettenzahl mit 1/dx² wächst, die Zellzahl aber mit 1/dx³).

**Ergebnis (14:15):** keine Kraftwirkung gegen `p4_pu8` noch gegen `p4_register`; einzige systematische Wirkung cz_reib +0,0007 (300/300)
= +0,96 % der Reibung. Wirkpfad belegt ([306]=[7]=159 588 435). Kosten 47,9 → 56,0 min, +55 MB VRAM, freier B70-VRAM im Minimum 8119 MiB.
**VRAM-Einordnung:** 81 % des Nahfeldbedarfs sind die DDFs; 3,75 mm passt (+4,9 GB), 3,5 mm nicht (−3,3 GB), D3Q27 voll wären +7,9 GB —
APG mit +55 MB ist nicht der Treiber. Deshalb VRAM/Performance vor D3Q27 (Heiko 16.09.).

**Auswertungsrezept (für künftige APG-/Wandmodell-Arme):**
1. `python3 werkzeuge/zeitreihe_ab.py p4_pu8 p4_apg1 cd_druck_rest cz_druck_rest cd_reib cz_reib` — Vorzeichenkonstanz entscheidet über
   Wirkung gegen Rauschen (am S1-Paar validiert: +0,564 ± 0,003, 50/50, 0 Wechsel).
2. Zähler im Log: `[306]` = `[7]` (Vorkernel- gegen Facettenbesuche), `[308]` = `[7]−[9]`, `[309]+[310]` = `[19]`. Alle drei müssen stimmen,
   sonst ist der Wirkpfad nicht belegt.
3. Autoritätshistogramm `|κ·y_ab·dp/ds|/τ_w`: bei 8 mm lag es zu 98,5 % bei ≥ 1. **Die eigentliche Frage dieses Laufs:** sinkt der Anteil
   bei halbem y_ab spürbar? Wenn nein, ist der Term auch bei 4 mm klemmdominiert und κ bleibt nicht deutbar.
4. Kräfte gegen die LETZTE REFERENZ `p4_register` einordnen (nicht gegen `p4_pu8` als „Bezug" — `p4_pu8` ist nur der A/B-Partner).
   `basis/` bleibt unangetastet.

**Entscheid danach (Heiko):** bewegt APG bei 4 mm etwas oder nicht → Standard oder endgültig parken.

## 3 · Nächste Punkte (Reihenfolge Heiko, Stand 13:10)

**Vor D3Q27 kommen VRAM-/Performance-Themen** (Heiko 16.09.). Vorschlag, billig zuerst — Entscheid steht aus:
(a) `CFD_T_WARMUP` 0,201 → 0,29: 15,6 min UND +1,3 % Genauigkeit, eine Variable, seit 12.09. Platz 1 und unerledigt;
(b) E1 + B1 (asm-Statistik, Facettenzellen je SIMD-Block) — **ohne GPU-Budget**;
(c) B70-Leiter für Nx%16 — beziffert die Regel auf der zweiten Karte, Pflicht vor 3,75 mm;
(d) A2 `auto-large-GRF` (Occupancy-A/B, bitgleich prüfbar);
(e) Prüfpunkt/Neustart — grösster Posten (17–34 min je Folgelauf), aber 200–300 Zeilen Bau.
Danach 3,75 mm / 15 m, dann D3Q27, dann OpenFOAM-13-Abgleich.

**Zurückgestellt:** das 4-mm-Reproduzierbarkeitsproblem (`p4_u125` ≠ `p4_u125b`) — Heiko vermutet die Voxelierung, nicht von mir vorziehen.

## 4 · Fallen, die heute Zeit gekostet haben

- **„Regression bitgleich" beweist nichts über einen Pfad, den die Regression nicht betritt.** HOCH-1 (fahrzeug_dd nullt die Facetten-Statik
  vor dem Fernfeldbau) wäre im Produktionsfall ein stiller Pufferüberlauf gewesen — gefunden vom Prüfagenten, nicht von meinen sechs Kugel-Hashes.
  Für Mehrdomänen-Code gehört ein Mehrdomänen-Beleg dazu (`logs/apgdd_igpu.txt`).
- **Ein Edit-Skript über mehrere Dateien** brach am Anker der zweiten Datei ab; setup.cpp blieb unverändert und eine ganze CPU-Serie war
  bedeutungslos (rc 0!). Konsequenz: jede Datei erst am Ende schreiben ODER danach je Datei einen Anker prüfen.
- **Die Queue-Absturzsperre** fasst `CFD_FAC_APG_HAKEN` als Atomik-Haken; der dd-Beleg musste ohne Haken laufen.
- Historische Absätze in TODO.md sind gefährlich: der „u_lat geparkt"-Absatz vom 12.09. hat 22 min GPU gekostet (`p4_pu` mit 0,075).


## 5 · Nachtrag 20:00 — selbständige Performance-/VRAM-Runde (Heiko 17:35: „mache selbständig weiter")

| Punkt | Ergebnis | Beleg |
|---|---|---|
| B70-Leiter | Nx%16 +2,9 % (mit Überlapp), keine Sättigung bis 397 Mio Zellen; Nahfeld konstruktiv nie ausrichtbar | `logs/lb_b70_skala.txt`, Protokoll 17:45 |
| A2 auto-large-GRF | bitgleich, **+3,9 % langsamer** → verworfen; `CFD_OCL_OPTIONS` als Messarm | `logs/a2_grf.txt`, 17:50 |
| E1 | `stream_collide` SIMD16/128 GRF, 79 d16-Nachrichten (halbe Cache-Zeile), 91 Byte-Lasten | 17:45 |
| B1 | Facettenzellen 1,10 % der Zellen = 4,22 % der SIMD16-Blöcke (×3,84); die 0,67 % vom 26.08. waren ein Zellanteil | `werkzeuge/b1_divergenz.py`, 17:50 |
| T_WARMUP | „15,6 min" nirgends hergeleitet; Entscheid a/b/c in TODO Punkt 1 | 18:1x |
| **dx-Umrechnung (4a)** | gebaut, Prüfagent sauber (kein offenes HOCH), F1/F2/F4 bestanden, F3 läuft | PLAN-DX-UMRECHNUNG-2026-09-16.md, 18:00–19:55 |
| `p375_b` | = p375_a-Befund (kein Fenster signifikant) — Vorbehalt vom 14:36 aufgelöst | 19:55 |

**Kein neuer VRAM-/Performance-Hebel.** Offen: Prüfpunkt/Neustart (einziger zweistelliger Zeitposten), „X messen" (→ B2), D1/D2, C1, Remesh-Gatter, „u nur wo gelesen".
**Läuft/steht an:** `p4_pu8_dx` (F3, bitgleich zu p4_pu8, bis ~20:05) → `p375_c` (3,75 mm + SGS-Band Lagen 2+3, Heiko 19:50, bis ~21:15).
**Nach der Queue (kein `make` vorher):** Fix-Skript für vier Prüfagent-Texte/Guards anwenden (`scratchpad/fix_nach_queue.py`: Rundungsansage,
`isfinite` in `dx_skal_setzen`, zwei Meldungen), bauen, Kugel-CPU-Runde, committen.
**Regel neu:** Serienzeilen tragen auf ALLEN Sprossen die 4-mm-Schrittwerte (15000/5000/5000, ZAEHL_TAKT 200, SAMPLE_EVERY 50); alte 8-mm-Dateien
mit 7500 nie mit `CFD_BASIS=aus` fahren; jede dd-Zeile braucht jetzt `CFD_FAR_LX` (Basis führt ihn); 3,75-mm-Läufe mit `CFD_QUEUE_HANG_S≥600`.


## 6 · Nachtrag 20:40 — dx-Umrechnung abgenommen, y-Halbzellen-Versatz gebaut, 3,75-mm-Läufe mit Versatz laufen

- **TODO 4a ERLEDIGT:** alle vier Abnahmen bestanden (Kugel-Hashes, 8 mm bitgleich, `p4_pu8_dx` bitgleich zu p4_pu8, `p375_b` SISM 150,0 ms und Kräfte = p375_a).
  Prüfagent-Reste gebaut (67266b1, Kugel ka5 4/4). Regel: Serienzeilen tragen auf ALLEN Sprossen die 4-mm-Schrittwerte; jede dd-Zeile braucht `CFD_FAR_LX`.
- **TODO 4c (neu, Heiko 20:27):** Mittelebenen-Membran gemessen (p375_b 82,2 %, 8 mm 73,3 %), `CFD_Y_VERSATZ=1` gebaut (91843b7), Probe: Membran weg (0,8 %).
  `p375_c` (Band ohne Versatz) auf Heikos Anweisung abgebrochen und als `p375_c_ABGEBROCHEN_ohne_versatz` aufgehoben.
- **Läuft (B70, ab 20:34, `logs/p375_dv.txt`):** `p375_d` = p375_b + Versatz (Kraft-A/B gegen p375_b: eine Variable Geometrie +1,875 mm), danach
  `p375_e` = Band Lagen 2+3 + Versatz + `CFD_U_SPARSAM=0` (Codesperre Band×U_SPARSAM; deklariert) → Bandwirkung gegen p375_d. Ende ~22:45.
- **Auswertung morgen früh:** `fenster_50ms.py p375_b p375_d` und `p375_d p375_e` (cd_rest/cz_rest je 50 ms), `membran_y0.py p375_d`, Laufzeit/VRAM
  (`export/p375_dv_vram.csv`), Prüfagent-Bericht zum Versatz (lief 20:32 parallel; bei HOCH: Läufe bewerten, ggf. wiederholen).
- **Entscheide für Heiko:** Versatz in die Basis (Standard)? T_WARMUP a/b/c? Danach Performance-Restliste (X messen → B2, D1/D2, C1, Prüfpunkt-Plan).
