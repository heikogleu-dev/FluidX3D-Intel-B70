# Vorhersage vor dem 4-mm-Produktionslauf (2026-08-22 nachmittags)

Geschrieben VOR dem Start. Bezug: `export/f4_std_diff2` (4 mm, 2026-08-21, 0,5 s).
Arm: dieselbe Konfiguration plus `CFD_N2F_SCHALE=0.5 CFD_N2F_BAND=1 CFD_N2F_BAND_N=16
CFD_N2F_BAND_WAKE=1 CFD_N2F_BAND_WAKE_START_X=<Kabinendach>`, dazu `CFD_FAC_CD_EVERY=1`
(reine Diagnose-Kadenz, aendert die Loesung nicht -- der Block liest nur vom Geraet).

## Bezugswerte, Fenster 0,2-0,5 s

| Groesse | f4_std_diff2 | Bauplan-Zielmarke |
|---|---|---|
| cd_druck | 0,8428 +- 0,0211 (74) | <= 0,78 |
| cz_druck_rest | -0,8255 +- 0,0633 (74) | <= -0,92 |
| cz_druck_band | 0,2461 +- 0,0003 (74) | innerhalb +- 0,03 bleiben |
| Cd (object_force) | 9,0716 +- 0,0463 (300) | -- |

## Vorhersagen, falsifizierbar

1. **cd_druck faellt um 8 bis 14 Prozent auf 0,725 bis 0,776.** Herleitung: auf der
   8-mm-Sprosse in der breiten Box senkte N=16 den Wert von 1,759 auf 1,568 (-10,9 %, 5,4 sigma).
   Damit waere die Bauplan-Marke 0,78 knapp erreicht. *Falsifiziert, wenn cd_druck ueber 0,80
   bleibt oder steigt.*

2. **Der Kopplungsfehler ueber die Mittelebene faellt um mindestens 50 Prozent.** Auf 8 mm
   waren es -63 % (9,715 -> 3,613 m/s RMS). *Falsifiziert bei weniger als 50 %.*

3. **cz_druck_band bewegt sich um weniger als 0,03.** Auf 8 mm: 0,5836 -> 0,5805 (-0,003).
   Das Kontaktband ist kein Zielkanal; bewegt es sich stark, wirkt die Kopplung am falschen Ort.

4. **★ WARNUNG, die dem Bauplan widerspricht: cz_druck_rest geht voraussichtlich in die
   FALSCHE Richtung.** Auf 8 mm verschob die Kopplung ihn von -0,652 auf -0,474, also zu
   WENIGER Abtrieb -- der Bauplan will mehr (-0,92). Der Effekt lag mit 1,2 sigma im Rauschen,
   aber das Vorzeichen war in allen vier gekoppelten 8-mm-Armen dasselbe. Vorhersage:
   cz_druck_rest steigt (wird weniger negativ) um 0,1 bis 0,25 auf -0,73 bis -0,58.
   *Falsifiziert, wenn er faellt.* Trifft die Vorhersage zu, ist Bauplan-Kriterium 2 verfehlt
   und die Kopplung ist ein Gewinn fuer die Feldkonsistenz, aber kein Gewinn fuer den Abtrieb.

5. **Fx_far bleibt unbrauchbar** (das Band ueberschreibt die karosserienahe Schicht, aus der
   update_force_field liest). Kein Kriterium daraus ableiten.

## Was diesen Lauf entwerten wuerde
- Ein Waechter bricht ab, dessen Schwelle auf 8 mm geeicht ist (Kipp, WAKE-KIPP, Dichte,
  u-Negation). Der Planungsagent prueft das gerade.
- Der Wake-Kasten rueckt zu nah an den Nahfeld-Auslass: CFD_N2F_BAND_WAKE_ABSTAND ist in
  GROBZELLEN (Default 16) und damit bei 4 mm nur noch 256 mm statt 512 mm.
