# Auflösungsskalierung v2 — Befunde 17.09.2026

Stand: v2 master @ 6c3f2ef. Vier Lese-Agenten (A Schalter/Basis, B Zeit/Takte, C Physik/Kräfte, D Geometrie/Werkzeuge), nichts geändert.
Zahlen „r" = aus Code/float32-Nachbau gerechnet, „g" = aus Lauflogs gezählt. Reihenfolge der Gitter: 4 / 8 / 3,75 / 16 mm.
Stichproben am Code nachgeprüft (Hauptsitzung): KRAFT_ZBAND setup.cpp:8493 + 4264, BODEN_EQ kernel.cpp:4318-4319 + setup.cpp:6909,
Sponge-Reserve setup.cpp:6941, N2F-Blend kernel.cpp:5047, rho_rand_ab.py:12, q_dachfenster.py:14.

## Ergebnisrelevant

| # | Befund | Ort | Ist | Folge | Quelle |
|---|---|---|---|---|---|
| 1 | Kontaktband-Oberkante | setup.cpp:8493, 4264; Basis KRAFT_ZBAND 4 zellen_fein | Band = 0x41 mit z < N, Zeile z=0 ist Fahrbahn → wirksam z=1..N−1, Oberkante (N−½)·dx; Log/CSV nennen N·dx | Oberkante 14 / 12 / 13,1 mm / Band leer (16 mm, cd_rest = gesamt); Band-Volumen g: 2,17e-3 → 1,64e-3 m³ (−24 %, 8 mm) / 1,98e-3 (−9 %) | A1, C4, D1 |
| 2 | Fahrbahnwand bei +dx/2, Fahrzeug weltfest −3 mm | setup.cpp:6668, 6755-6761, 7287; kernel.cpp Zellmitte = Index | Halfway-BB-Wand der Straße liegt eine halbe Zelle über Welt-z=0 | Einsenkung r 5 / 7 / 4,9 / 11 mm statt 3; Kontaktfläche g 0,103 / 0,135 (+32 %) / 0,100 / 0,186 m²; Bodenwand Nah gegen Fern versetzt 6 / 12 / 5,6 / 24 mm | A2, C1, D2 |
| 3 | SAT-Schale dickt ∝ dx auf | setup.cpp:1251 (Halbweite 0,5 Zelle) | Wand im Mittel 0,5–0,87 dx außerhalb der STL (3,75 mm gemessen ≈0,7 dx) | je Seite r 2–3,5 / 4–7 / 1,9–3,3 mm; A_eff g 1,8752 / 1,8866 / 1,8733 m²; Volumen +4,2 % bei 8 mm; 15,3-mm-Kanallamellen-Spalt bei 8 mm r fast zu (Deutung) | C2, D7 |
| 4 | BODEN_EQ-Lagen in Zellen, nicht umgerechnet | Basis BODEN_EQ 2 / _DOWN 1 / FERN_BODEN_EQ 2 / FERN_EINLASS_EQ 2 („modus"); kernel.cpp:4319 | ABSTAND seit 16.09. Länge, die Lagenzahl nicht | unter dem Wagen 4 / 8 / 3,75 / 16 mm; vor der Nase 8 / 16 / 7,5 mm; Fernfeld 32 / 64 / 30 mm; N2F-Band-Unterkante hängt daran (8 mm: keine Bandzelle mehr im Unterbodenspalt) | A3, C3, D3 |
| 5 | N2F-Rückkopplung zellen- und grobschrittgebunden | kernel.cpp:5047 (a = alpha·gewicht je Grobschritt); setup.cpp:7106-7114 (Gewicht halbiert je Lage) | Bandbreite in m fest, Profil je Lage und Rate je dt_c nicht | Kopplung je Meter Laufweg bei 8 mm halb so stark; e-Faltung a=0,5 r 2,9 / 5,8 / 2,7 mm; 16 mm: Wächter setup.cpp:7092 bricht ab | A4, B1, D4 |
| 6 | Kontaktband-Kraft skaliert ∝ dx/u_lat² (Messbefund, Confound Code-Stand/Wandmodell) | export/*/kraft_zband.csv | Fx_band 8 mm u0,075: 830 N; 8 mm SPZ8: 297 N; 4 mm u0,075: 342 N; 4 mm SPZ8: 124 N; 3,75 mm SPZ8: 116 N | jede Gesamt-/Bandgröße nicht über dx oder u_lat vergleichbar; 8-mm-Serien kl_std/o8 (u_lat 0,075) gegen p4/p375 (SPZ 8) tragen u_lat-Confound | C5 |

## Nebenbefunde (niedrig, teils nur 16 mm)

7. Sponge-Reserve fest 32 Grobzellen (setup.cpp:6941): 16 mm bricht ab, obwohl Platz ist.
8. Kastenrundung: Nahfeld-Breite +16 mm (8 mm), NEAR_VOR 90 statt 96 mm (3,75 mm), +48 mm und Einlauf +32 mm (16 mm); Wächter prüft den Rohwert.
9. Zeitraster: SAMPLE_EVERY bei 8/16 mm gerundet (1,067 ms, 281 statt 301 Samples, Slice-Namen 201/251…); SLICE_NEAR_STEPS-Codedefault 5000 roh; VTK-Uhr driftet (300,8 ms bei 8 mm); DD_VERIFY_AT roh (Diagnose).
10. Werkzeuge mit fester Gitterannahme: interface_serie.py:44-45 (alte 4-mm-Box, auf ALLEN Sprossen falsch), q_dachfenster.py:14-15 (DX 0,004), rho_rand_ab.py:12 (U_LAT 0,075 → cp ×2,78 bei SPZ 8), zonen_kraft.py/zonen_vergleich.py (3,75-mm-Annahmen, Laufliste p375 fest), zeitreihe_ab.py:15-18 (4↔8 mm Paartest auf 19 Samples), Y_VERSATZ in diff_of13_yslice.py:44 / diff_of13_zonen.py:123 / fx_band.py:56 nicht abgezogen (Ebene körperbezogen 22 statt 25 mm bei 4 mm).
11. Diagnoseorte in Zellen (Sonde x_f=2/10, Wandprofil z=1..7): zwischen Sprossen nicht ortsgleich. Log beschriftet 3,75 mm als „4 mm" (setup.cpp:7226). Veraltete Kommentare (basis:89-93, setup.cpp:8134-8139, 8369, 8990).
12. PTRT, DETEPS, POSITIV, U_KLEMME, ZAEHL_TAKT laufen in der Produktion, stehen aber nicht in basis/fahrzeug_dd.basis → der Basis-Wächter prüft sie nicht.
13. Deutung: Ma_lat = 0,2165 auf allen Sprossen fest → der Kompressibilitätsfehler O(Ma²) verschwindet nicht mit dx.
14. Dämpfungszone: SGS-Anteil und Klemme in Gittereinheiten (≈4,2 m hinter dem Heck, niedrig).

## Geprüft und korrekt
Einheitenkette U/rho/nu → nu_lat/tau beider Domänen (nu_lat_c = nu_lat_f/ratio), units.set_m_kg_s, Kraftnormierung (forces, cd_facetten, cd_bericht, kraft_zband), Smagorinsky Δ = dx, SISM_AB/T und alle Schrittschalter über env_schritte in Sekunden, T_END/WARMUP/PERF_AB/BERICHT in s, Nah/Fern-Zeitkonsistenz (Fehler 1. Ordnung in dt, fällt mit dx), Wandmodell-Formeln (Spalding κ/B, Y in Gittereinheiten konsistent), DETEPS dimensionslos, Rang-/Akzeptanzschwellen relativ, U_KLEMME c_s, Sponge-Viskositätsfaktor, BODEN_EQ_ABSTAND/WAKE/SPONGE_N als Länge, place() Nase/Skalierung weltfest, OF13-Diff-Werkzeuge mit dx aus dem VTK.
