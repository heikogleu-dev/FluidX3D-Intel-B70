# Band-Artefakt bei 8 mm: Deckellage im Rest

Stand 17.09.2026. Heiko-Entscheid ~12:00 zu Skalierungsaudit Punkt 6: **Option 1, nur Auswertung.** Physik und Kernel bleiben unverändert, geändert werden Band-Regel, Wächter und Kennzeichnung.

## Befund

- Die Kontaktband-Kraft sitzt in den **Keilzellen z = 1** vor und hinter dem Reifenlatsch, unter dem Reifenüberhang. Ihre Links gehen auch auf die **Überhang-Unterseite z = 2 (Deckellage)**. Die Zeile z = 0 ist Fahrbahn.
- `CFD_KRAFT_ZBAND = N` zählt die Solidzellen mit z < N. Bei **N = 2** (bisherige 8-mm-Umrechnung llround(16/8); alle 208 erhobenen 8-mm-Exporte mit Band) enthält das Band nur die Keillage. Die Deckellage z = 2 zählt zum **Rest**.
- Damit tragen `Fx/Fz_rest` und `Cz_rest` in `kraft_zband.csv`, `cd_druck_rest` und `cz_druck_rest` in `cd_facetten.csv` und `cd_rest`/`cz_rest` in `cd_bericht.csv` den Deckelanteil des Keilartefakts. Dasselbe Prädikat z < N gilt in beiden Zerlegungen (object_force und kraft_facetten).
- **Größenordnung** (Untersuchung 17.09.): 8 mm, Klemmen-A/B vom 16.09. (POSITIV=2 + U_KLEMME=1), u_lat 0,075, Mittel ab 0,2 s.
  - `cz_druck_rest` verschiebt sich von −0,176 (kl_std_a_dx_b70) auf −0,449 (kl_std_pu_dx_b70), also um −0,273. Beide Mittelwerte sind aus `cd_facetten.csv` nachgerechnet: −0,1759 / −0,4485.
  - Davon sind **rund −0,27 Cz Artefakt**. Diese Zuordnung stammt aus der Untersuchung und ist hier nicht neu gemessen.
  - `cz_druck_band` liegt in beiden Armen bei −0,0001 / −0,0002. Die Verschiebung steckt also vollständig im Rest.
- **Lesehinweis:** Gesamt-Cd/Cz und die Bandgrößen sind **nie** als Aerodynamik zu lesen. Das Gesamt enthält den Keil immer, das Band ist ein Kontaktartefakt. Belastbar ist nur ein Rest, dessen Band Keil- UND Deckellage enthält.

## Korrektur ab 17.09.2026

- Die Regel für `CFD_KRAFT_ZBAND` lautet jetzt **N = max(3, nächste wirksame Oberkante (N−½)·dx an 16 mm)**, also N = max(3, ceil(16/dx)).
  - Ergebnis: 4 mm N = 4 (14,000 mm), **8 mm N = 3 (20,000 mm)**, 3,75 mm N = 5 (16,875 mm), 16 mm N = 3 (40,000 mm).
  - Das Minimum 3 heißt: mindestens Keil- UND Deckellage.
  - Stellen: `src/setup.cpp` (`KRAFT_ZBAND_N_MIN`, `kraft_zband_regel`), `werkzeuge/basis_zeile.py`, `werkzeuge/basis_aus_lauf.py`, `basis/fahrzeug_dd.basis` (`CFD_KRAFT_ZBAND 16 band_oberkante_mm`).
- Der Basis-Wächter meldet eine 8-mm-Zeile mit `CFD_KRAFT_ZBAND=2` als ABWEICHEND (Soll 3). Starten kann sie nur noch deklariert.
- Der dd-Fall warnt bei N < 3: „Deckellage im Rest, cd/cz_rest verschmutzt“.
- Die Kugel ist ausgenommen: Sie schwebt, es gibt keinen Latsch, keinen Keil und keinen Deckel. Dort bleibt N frei.
- Läufe mit N ≥ 3 enthalten die Deckellage und sind nicht markiert: 4 mm mit ZBAND 4 (43 Läufe), 3,75 mm mit ZBAND 4 (5 Läufe). Für 3,75 mm gilt aber die andere Kantenabweichung (13,125 statt 16,875 mm, SKALIERUNG-BEFUNDE Punkt 1).

## Betroffene Läufe: wirksames Band ohne Deckellage (N < 3)

**Erhebung:** alle 256 Exporte mit gesetztem `CFD_KRAFT_ZBAND` aus den 779 `export/*/code/LAUF.txt`. Dazu kommen die Lauflogs ohne Export-Ordner (28./29.08., vor der Lauf-Sicherung).
- Alle Treffer sind `fahrzeug_dd`, dx 8 mm, `CFD_KRAFT_ZBAND=2`. N = 1 kommt nicht vor, Kugelläufe mit Band gibt es nicht.
- Gegenprobe gegen die Vorarbeit `band_tabelle.txt` (Untersuchungsagent, 243 Läufe mit `kraft_zband.csv`): alle dortigen 198 Einträge mit ZBAND 2 bestätigt, dx, u_lat und ZBAND ohne Widerspruch zur LAUF.txt.
- Zusätzlich 10 Exporte mit ZBAND 2 ohne Messwerte (abgebrochen vor dem ersten Sample).

**Anzahl:** 208 Exporte + 6 nur als Log = **214 markierte Läufe**, davon 149 mit Messwerten ab 0,2 s bzw. Endbericht.
- Zeitraum der Exporte: 2026-08-30 10:59 bis 2026-09-17 09:19.
- u_lat: 0,075 (Vorgabe): 198, SPZ 8 (u_lat 0.125): 6, U_LAT 0.1: 2, U_LAT 0.125: 2.
- Spalte „Messpunkte“ = Zeilen in `kraft_zband.csv` ab t ≥ 0,2 s. Das Datum ist die Änderungszeit der LAUF.txt (Laufstart).

**D3Q27-A/B vom 17.09.:** `q19_bb8` / `q27_bb8` (8 mm, SPZ 8, ohne Wandmodell).
- Die Cz_rest-Differenz **+0,238** (Commit 7e30e2b; eigene Nachrechnung aus `kraft_zband.csv` ab 0,2 s: +0,237) steht **unter Vorbehalt**.
- Beide Arme tragen die Deckellage im Rest. Wie viel davon D3Q27 selbst und wie viel die unterschiedliche Keil-/Deckelkraft ist, ist ungeklärt.

| Lauf | dx [mm] | u_lat / SPZ | ZBAND | Datum | Messpunkte | Kennzeichnung |
|---|---|---|---|---|---|---|
| v_fp32 | 8 | 0,075 (Vorgabe) | 2 | 2026-08-30 10:59 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| v_fp16c | 8 | 0,075 (Vorgabe) | 2 | 2026-08-30 11:11 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| v_fp16s | 8 | 0,075 (Vorgabe) | 2 | 2026-08-30 11:18 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| w_ref | 8 | 0,075 (Vorgabe) | 2 | 2026-08-30 14:31 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| w_satgate0 | 8 | 0,075 (Vorgabe) | 2 | 2026-08-30 14:38 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| w_budget2 | 8 | 0,075 (Vorgabe) | 2 | 2026-08-30 14:44 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| w_elibb0 | 8 | 0,075 (Vorgabe) | 2 | 2026-08-30 14:50 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| w_kraft1 | 8 | 0,075 (Vorgabe) | 2 | 2026-08-30 18:39 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| w_bb | 8 | 0,075 (Vorgabe) | 2 | 2026-08-30 19:17 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| w_gdiag | 8 | 0,075 (Vorgabe) | 2 | 2026-09-02 21:45 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| w_fdwand | 8 | 0,075 (Vorgabe) | 2 | 2026-09-02 22:29 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| w_nb | 8 | 0,075 (Vorgabe) | 2 | 2026-09-03 08:24 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| xw_maske8 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-03 21:09 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| xw_pruef_zensus | 8 | 0,075 (Vorgabe) | 2 | 2026-09-03 21:19 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| xz_fl8 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-03 22:07 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| za_kraft1 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 06:51 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| zb_messnur | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 07:02 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| za_basis | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 07:09 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| zb_kraft1 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 07:15 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| za_messnur | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 07:18 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| zc_lsq | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 08:37 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| zf_pinv8 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 08:54 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| zk_ziel8 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 11:10 | keine (kein kraft_zband.csv) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| zk_ziel8b | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 11:12 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| zi_alpha0 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 11:20 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| zj_satgate0 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 11:34 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| ve_ziel8_pinv1 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 13:13 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| ve_ziel8_pinv0 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 13:20 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vg_zensus8 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 14:02 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vh_wander8 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 14:22 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vl_masse8_aus | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 16:21 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vl_masse8_an | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 16:28 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vm_masse8_an | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 16:44 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vo_f08_aus | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 17:37 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vo_f08_m2 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 17:43 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vp_kraft8 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-04 18:20 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vs_x8 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-05 08:59 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kh_w8_bezug | 8 | 0,075 (Vorgabe) | 2 | 2026-09-07 18:52 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kh_w8_wandfrei | 8 | 0,075 (Vorgabe) | 2 | 2026-09-07 18:59 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_s8_bezug | 8 | 0,075 (Vorgabe) | 2 | 2026-09-07 21:35 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_s8_sism | 8 | 0,075 (Vorgabe) | 2 | 2026-09-07 21:42 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kn_T25 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-07 22:48 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kn_T100 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-07 22:55 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| ko_f8_bezug | 8 | 0,075 (Vorgabe) | 2 | 2026-09-08 07:04 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| ko_f8_sism | 8 | 0,075 (Vorgabe) | 2 | 2026-09-08 07:11 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vb_k8_lage1 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-08 21:26 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vb_k8_band3 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-08 21:27 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vb_d8_bandnosism | 8 | 0,075 (Vorgabe) | 2 | 2026-09-08 21:28 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vc_k8_lage1 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-08 21:47 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vc_k8_band3 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-08 21:48 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vc_m8_bezug | 8 | 0,075 (Vorgabe) | 2 | 2026-09-08 21:51 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vc_m8_sism1 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-08 21:57 | 128 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| vc_m8_band3 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-08 22:28 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rd8_aus | 8 | 0,075 (Vorgabe) | 2 | 2026-09-09 10:53 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rd8_sism | 8 | 0,075 (Vorgabe) | 2 | 2026-09-09 10:59 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rd8_pinv | 8 | 0,075 (Vorgabe) | 2 | 2026-09-09 11:21 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rd8_pinv_bud2 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-09 12:22 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| ph8_persist | 8 | 0,075 (Vorgabe) | 2 | 2026-09-09 13:04 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| pk8_persist | 8 | 0,075 (Vorgabe) | 2 | 2026-09-09 13:20 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| de8_aus | 8 | 0,075 (Vorgabe) | 2 | 2026-09-09 16:58 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| de8_an | 8 | 0,075 (Vorgabe) | 2 | 2026-09-09 17:04 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| zs8_bezug | 8 | 0,075 (Vorgabe) | 2 | 2026-09-10 13:08 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| zs8_sism | 8 | 0,075 (Vorgabe) | 2 | 2026-09-10 13:15 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| zs8_pinv | 8 | 0,075 (Vorgabe) | 2 | 2026-09-10 13:21 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| zs8_deteps | 8 | 0,075 (Vorgabe) | 2 | 2026-09-10 13:28 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| v8_aus | 8 | 0,075 (Vorgabe) | 2 | 2026-09-10 19:12 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| v8_195 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-10 19:19 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| v8_190 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-10 19:27 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| o8_vor | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 11:21 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| o8_nach | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 11:43 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| o8_nach2 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 11:55 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| o8_e5 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 12:18 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| o8_it8 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 12:33 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| o8_tab0 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 12:41 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| o8_tab1 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 12:48 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| k8_kdiag | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 13:32 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| z8_takt100 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 14:26 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| z8_takt1000 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 14:33 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| z8_takt1000b | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 14:41 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| z8_takt100b | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 14:47 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| d8_kdiag_an | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 14:55 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| d8_kdiag_aus | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 15:02 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| w8_pruef | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 15:53 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| s8_spiegel | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 18:04 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| bt8_aus | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 19:16 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| bt8_t8 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 19:22 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| bt8_t16 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 19:31 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| bt8_t32 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 19:56 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| bt8_t64 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-11 20:04 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| uv8_vor | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 09:26 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| uv8_nach | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 09:34 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| uv8_u100 | 8 | U_LAT 0.1 | 2 | 2026-09-12 09:42 | 113 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| uv8_nach2 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 10:00 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| uv8_u100b | 8 | U_LAT 0.1 | 2 | 2026-09-12 10:07 | 113 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| uv8_u125_a | 8 | U_LAT 0.125 | 2 | 2026-09-12 11:55 | 91 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| uv8_u125_b | 8 | U_LAT 0.125 | 2 | 2026-09-12 11:59 | 91 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| uf8_aus | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 12:08 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| uf8_an | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 12:10 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rs8_aus | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 12:24 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rs8_an | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 12:31 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| us8_aus | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 12:44 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| us8_u | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 12:51 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| us8_beide | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 12:57 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| zs8_aus | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 13:06 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| zs8_beide | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 13:13 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| fs8_aus | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 13:24 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| fs8_beide | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 13:31 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| fb8_aus | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 13:40 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| fb8_beide | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 13:46 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| fc8_aus | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 14:31 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| fc8_beide | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 14:37 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| r8_vor | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 16:36 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| r8_fp32 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 16:42 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| r8_fp16 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 16:49 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| r8_fp16b | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 16:57 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| s8_sfp32 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 17:17 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| s8_sfp16 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 17:23 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| u8_fp32 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 20:08 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| u8_fp16 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 20:46 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| u8_fp16b | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 21:08 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| u8_fp32b | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 21:17 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| u8_fp16c | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 21:44 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| u8_fp16d | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 21:55 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| spz_b70b | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 22:11 | 26 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| spz_b70c | 8 | 0,075 (Vorgabe) | 2 | 2026-09-12 22:36 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c1h_dd8 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 10:40 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2a_dd8_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 12:06 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2a_dd8b_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 12:11 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2a2_dd8_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 12:24 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c_dd8_a_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 12:48 | keine (kein kraft_zband.csv) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c_dd8_b_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 12:49 | keine (kein kraft_zband.csv) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c_dd8_haken4_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 12:49 | keine (kein kraft_zband.csv) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c_dd8_haken6_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 12:49 | keine (kein kraft_zband.csv) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c2_dd8_a_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 12:50 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c2_dd8_b_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 12:51 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c2_dd8_haken4_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 12:52 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c2_dd8_haken6_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 12:53 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c3_dd8_a_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 13:12 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c3_dd8_b_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 13:13 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c3_dd8_h4_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 13:14 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c3_dd8_h6_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 13:15 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c3_dd8_h7_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 13:16 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c3_dd8_h8_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 13:17 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2d_dd8_a_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 13:18 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2d_dd8_b_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 13:21 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c4_dd8_b_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 13:28 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_c2c5_dd8_b_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 13:34 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_std_dd8_std_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 14:13 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_std_dd8_aus_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 14:14 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| rr_s0_dump_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 14:48 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_s0b_dd8_an_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 15:29 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_s0b_dd8_aus_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 15:33 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_s0c_dd8_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 15:39 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_s0d_dd8_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 15:46 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_s0d_dd8_h1_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 15:49 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_s0d_dd8_h1_cpu | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 20:08 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_z2c_dd8_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 20:25 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_p1b_dd8_m1_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 21:15 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_p1c_dd8_m2_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 21:37 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_p1c_dd8_m1_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 21:41 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_z2m_dd8_a_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 22:08 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_z2m_dd8_b1_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 22:12 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_z2m_dd8_cb1_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 22:16 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_z2h5_dd8_cpu | 8 | 0,075 (Vorgabe) | 2 | 2026-09-15 22:44 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_z2m_dd8_cb1r_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 06:36 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_z2m_dd8_cb1b2_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 06:40 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_m5_bilanz1_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 06:52 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_m5_bilanz0_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 06:56 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_a1_dd8_a_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:12 | keine (kein kraft_zband.csv) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_a1_dd8_t_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:12 | keine (kein kraft_zband.csv) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_a1_dd8_b0_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:13 | keine (kein kraft_zband.csv) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_a2_dd8_a_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:14 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_a2_dd8_t_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:15 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_a2_dd8_b0_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:16 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_a3_dd8_a_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:17 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_a3_dd8_t_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:18 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_a3_dd8_b0_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:19 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_a5_dd8_a_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:45 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_a5_dd8_t_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:47 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_a5_dd8_b0_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:48 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_a6_dd8_a_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:50 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_a6_dd8_t_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:51 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_a6_dd8_b0_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:52 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_std_a_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 07:58 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_std_pu_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 08:02 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| lt_igpu_t | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 08:58 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| lt_igpu_0 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 08:59 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| lt_b70_t | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 09:00 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_apg_reg_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 11:59 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| apg8_a0_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 12:03 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| apg8_s1_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 12:07 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| apgdd_k1_igpu | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 12:28 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| apg8_a1_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 12:31 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| apg8_a2_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 12:36 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| a2_kurz_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 17:42 | keine (kein kraft_zband.csv) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| a2_base_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 17:43 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| a2_grf_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 17:46 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_std_a_dx_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 19:06 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| kl_std_pu_dx_b70 | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 19:10 | 51 | cd/cz_rest verschmutzt (Deckellage im Rest) |
| yv_ohne_igpu | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 20:30 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| yv_mit_igpu | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 20:31 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| yv_mit2_igpu | 8 | 0,075 (Vorgabe) | 2 | 2026-09-16 21:50 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| q19_bb8_s1_cpu | 8 | SPZ 8 (u_lat 0.125) | 2 | 2026-09-17 08:58 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| q27_bb8_s1_cpu | 8 | SPZ 8 (u_lat 0.125) | 2 | 2026-09-17 09:00 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| q19_bb8_s2_igpu | 8 | SPZ 8 (u_lat 0.125) | 2 | 2026-09-17 09:03 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| q27_bb8_s2_igpu | 8 | SPZ 8 (u_lat 0.125) | 2 | 2026-09-17 09:07 | keine (vor 0,2 s beendet) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| q19_bb8 | 8 | SPZ 8 (u_lat 0.125) | 2 | 2026-09-17 09:15 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) **D3Q27-A/B 17.09.: Cz_rest-Differenz +0,238 unter Vorbehalt** |
| q27_bb8 | 8 | SPZ 8 (u_lat 0.125) | 2 | 2026-09-17 09:19 | 151 | cd/cz_rest verschmutzt (Deckellage im Rest) **D3Q27-A/B 17.09.: Cz_rest-Differenz +0,238 unter Vorbehalt** |
| tl_8mm_basis_v1 (nur Log, kein Export) | 8 | 0,075 (Vorgabe) | 2 | 2026-08-28 17:00 | 401 (Endbericht-Samples) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| tm_8mm_b_norm (nur Log, kein Export) | 8 | 0,075 (Vorgabe) | 2 | 2026-08-28 17:13 | 401 (Endbericht-Samples) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| tm_8mm_c_yw (nur Log, kein Export) | 8 | 0,075 (Vorgabe) | 2 | 2026-08-28 17:15 | kein Endbericht | cd/cz_rest verschmutzt (Deckellage im Rest) |
| tn_8mm_c_yw (nur Log, kein Export) | 8 | 0,075 (Vorgabe) | 2 | 2026-08-28 17:30 | 401 (Endbericht-Samples) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| tn_8mm_d_kante (nur Log, kein Export) | 8 | 0,075 (Vorgabe) | 2 | 2026-08-28 17:43 | 401 (Endbericht-Samples) | cd/cz_rest verschmutzt (Deckellage im Rest) |
| ub_rauchtest (nur Log, kein Export) | 8 | 0,075 (Vorgabe) | 2 | 2026-08-29 14:14 | 76 (Endbericht-Samples) | cd/cz_rest verschmutzt (Deckellage im Rest) |
