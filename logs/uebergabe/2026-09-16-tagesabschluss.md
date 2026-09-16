# Übergabe 16.09.2026 — Tagesabschluss

**Stand bei Übergabe:** Klemmen-Block und APG-Linie abgeschlossen; der 4-mm-Voll-Lauf mit APG (`p4_apg1`) läuft seit 13:11 auf der B70.
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

**Auswertung, wenn der Lauf steht:**
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
