# Gittergeschwindigkeit u_lat — Schalter und erste Messung

**Stand 12.09.2026 vormittags.** TODO 1 aus `PERFORMANCE.md`. Fassung: FluidX3D-v2, Zweig
master, Commit 68568dc plus die hier beschriebene Änderung an `src/setup.cpp`.

---

## 1 · Die Herleitung, die bisher fehlte

`u_lat = 0.075f` stand an vier Stellen hart verdrahtet, laut Übergabe vom 11.09. „ohne
dokumentierte Herleitung". Die Herleitung ist nachrechenbar und sie ist kein Zufall:

```
dt = u_lat · dx / si_u = 0,075 · 0,004 m / 30 m/s = 1,00000e-5 s   GENAU 10 µs
```

**0,075 ist der Wert, der den Zeitschritt der Produktionssprosse auf runde 10 µs legt.** Daran
hängt mehr als die Eleganz: alle Schalter, die in SCHRITTEN zählen, entsprechen deshalb runden
Millisekunden.

| Schalter | Schritte | bei dt = 10 µs | bei u_lat = 0,100 (13,33 µs) |
|---|---:|---:|---:|
| `CFD_SGS_SISM_AB` (SISM wird scharf) | 15000 | **150 ms** | 200 ms |
| `CFD_SGS_SISM_T` (Mittelungsfenster) | 5000 | **50 ms** | 66,7 ms |
| `CFD_SLICE_NEAR_STEPS` | 5000 | **50 ms** | 66,7 ms |
| `CFD_SAMPLE_EVERY` (25 GROBE Schritte) | 25 | **2,0 ms** | 2,67 ms |

**Das ist die Falle, und sie stand in keiner Unterlage.** Unkompensiert würde SISM bei
u_lat = 0,100 erst bei 200 ms scharf — also **erst am Messbeginn** (`CFD_T_WARMUP` 0,201). Ein
u_lat-Arm ohne Umrechnung trägt vier Änderungen statt zwei, und der Befund wäre nicht deutbar.

Dass das Projekt die dx-Umrechnung schon von Hand macht (8-mm-Zeile: 7500 / 2500 / 2500), hat
die u_lat-Abhängigkeit verdeckt: sie sieht wie eine Sprossen-Umrechnung aus, ist aber an dt
gebunden, und dt hängt an **beiden** Größen.

---

## 2 · Was gebaut wurde

**`CFD_U_LAT`, Vorgabe 0.075f**, an allen vier Stellen (`kugel`, `fahrzeug`, `fahrzeug_dd`,
`fernfeld`) über einen gemeinsamen Helfer `u_lat_schalter()`. Dazu:

* **Ansage bei Abweichung** mit allen drei gekoppelten Änderungen: Ma = u_lat·√3, der
  Kompressibilitätsfaktor (u/u₀)², das mitwachsende ν_lat und damit τ — und die
  Geschwindigkeitsklemme, die bei 0,57735 Gittereinheiten steht und in SI mit 1/u_lat skaliert
  (230,9 → 173,2 m/s bei 0,100). Der Klemmenpunkt war in der Übergabe nicht genannt.
* **Harte Obergrenze 0,3** (Ma 0,52) mit `print_error`, untere Schranke gegen 0 und negativ.
* **`u_lat_zeitwaechter()`** rechnet die schrittbasierten Schalter in physikalische Zeit um und
  sagt die Verschiebung an. Er rechnet **nicht** still um — das wäre eine zweite Variable im Arm.
* **Ansage in den beiden Fällen, die u_lat nicht kennen** (`kanal` rechnet über
  `CFD_KANAL_UTAU`, `facetten_test` hat keine Anströmung).

---

## 3 · Abnahme: der Schalter ist inert

8-mm-Fahrzeug, B70 fein + iGPU grob, Zeile wortgleich zu `logs/opt_8mm_serie.txt`.

| | `uv8_vor` | `uv8_nach` |
|---|---|---|
| Binary | Commit 68568dc, Baum **sauber** (Schalter im git stash) | 68568dc + Schalter |
| `CFD_U_LAT` | existiert nicht | **ungesetzt** |
| Wanduhr | 428 s | 413 s |
| rc / Fehler | 0 / 0 | 0 / 0 |
| **21 CSVs + 7 Feld-Dumps byteweise** | — | **28 von 28 BITGLEICH** |

**Abnahme bestanden.** Die Wanduhrdifferenz von 15 s (3,5 %) zwischen zwei **bitgleichen** Armen
ist reine Wanduhr-Streuung und gehört notiert: sie ist größer als die 4 s, die am 11.09. als
Streuung zweier identischer Wiederholungen gemessen wurden.

---

## 4 · Messung: u_lat 0,075 gegen 0,100

Arm `uv8_u100`, **eine** physikalische Variable, die drei Schrittschalter auf gleiche
physikalische Zeit umgerechnet (7500→5625, 2500→1875, 2500→1875; exakt, nicht gerundet). Die
Abweichung `CFD_SLICE_NEAR_STEPS` gegen die Basis ist deklariert — **der Basis-Wächter hat den
Arm beim ersten Versuch nach einer Sekunde angehalten**, genau dafür ist er gebaut.

### Die leichte Hälfte: die Laufzeit trifft die Vorhersage

| | `uv8_nach` | `uv8_u100` | Vorhersage |
|---|---:|---:|---:|
| dt_fein | 20,00 µs | 26,67 µs | |
| feine Schritte bis 0,501 s | 25 050 | 18 788 | |
| **Wanduhr** | **413 s** | **314 s = 76,0 %** | 75,0 % |

**Laufzeit ∝ 1/u_lat ist bestätigt.** Der Rest von 1 Prozentpunkt liegt innerhalb der 3,5 %
Wanduhr-Streuung aus Abschnitt 3.

### Die harte Hälfte: die Kräfte bewegen sich

Gepaart über `werkzeuge/fenster50.py`. **Vorbehalt zur Paarung:** die Abtastraster der beiden
Arme treffen sich nur alle 8 ms (2,0 gegen 2,67 ms), es bleiben 37 von 149 Punkten.

| Fenster | Cd_rest Δ ± BlockSEM | σ | Cz_rest Δ ± BlockSEM | σ |
|---|---:|---:|---:|---:|
| **200–500 ms** | **+0,0346 ± 0,0157** | **2,2** | **−0,0863 ± 0,0523** | 1,6 |
| 350–500 ms | +0,0119 ± 0,0239 | 0,5 | −0,1072 ± 0,0964 | 1,1 |
| 200–250 ms | +0,0760 ± 0,0788 | 1,0 | +0,0377 ± 0,2126 | 0,2 |
| 250–300 ms | +0,0498 ± 0,0473 | 1,1 | −0,2531 ± 0,0689 | 3,7 |
| 300–350 ms | +0,0496 ± 0,0422 | 1,2 | +0,0228 ± 0,2312 | 0,1 |
| 350–400 ms | +0,0464 ± 0,0301 | 1,5 | −0,0470 ± 0,1499 | 0,3 |
| 400–497 ms | −0,0040 ± 0,0166 | 0,2 | −0,1350 ± 0,0793 | 1,7 |

**Cd_rest liegt über das ganze Messfenster um +0,035 höher, das sind +7,2 % bei 2,2 σ.** Nach dem
Abnahmekriterium der Übergabe („bewegen sie sich über die Fehlerbalken hinaus, ist 0,100 zu
viel") ist das ein Nein zu 0,100.

**Aber die Zahl hat eine Struktur, und die ist wichtiger als das Vorzeichen:** der Versatz sitzt
in den ersten vier Fenstern (+0,046 bis +0,076) und ist im letzten Fenster **verschwunden**
(−0,004 ± 0,017). Über 350–500 ms bleiben 0,5 σ. Zwei Deutungen sind möglich und dieser Lauf
trennt sie nicht:

1. **Einschwingunterschied.** Die beiden Arme laufen mit verschiedenem Ma durch die Anwärmphase
   und kommen verschieden schnell an. Dann ist 0,100 brauchbar und das Messfenster zu früh —
   was `PERFORMANCE.md` ohnehin schon beziffert (+1,34 % Bias auf cd_druck).
2. **Echter, kleiner Versatz**, den 300 ms nicht auflösen.

Cz_rest ist durchweg negativer (mehr Abtrieb), aber die Streuung trägt es nicht: 1,1 bis 1,6 σ
im Gesamtfenster, ein einzelnes Fenster bei 3,7 σ und zwei bei 0,1 σ.

### Stabilität: unauffällig, eher besser

| | `uv8_nach` | `uv8_u100` |
|---|---:|---:|
| |u| max FREI (ganzes Nahfeld, nur Fluid, 500 ms) | 48,96 m/s | 50,11 m/s |
| |u| max an bewegter Wand | 139,34 m/s | **127,56 m/s** |
| Zellen > 60 m/s | 126 (0 frei) | **124 (0 frei)** |
| SISM-Klemmrate |S|<S̄ | 36,9 % | 36,0 % |
| Geschwindigkeitsklemme erreicht? | nein (Grenze 230,9 m/s) | **nein** (Grenze 173,2 m/s) |

Die Klemme rückt bei 0,100 auf 173,2 m/s herunter und wird mit 127,6 m/s **nicht** erreicht. τ
steigt auf dieser Sprosse von **0,50001416 auf 0,50001887** (ν_lat = si_nu·u_lat/(si_u·dx),
8 mm), also weg von der Grenze. Auf der 4-mm-Produktionssprosse wären es 0,50002831 →
0,50003775 — das ist die Zahl, die die Übergabe nennt, und sie gilt **nur dort**; bei 8 mm ist τ
durchweg halb so weit von 0,5 entfernt. Kein Stabilitätspreis messbar.

---

## 5 · Was als Nächstes zu entscheiden ist

Der Hebel ist **−24 % Wanduhr für eine Konstante** und damit der größte billige Posten der
Liste. Er ist nicht gratis, aber die Rechnung ist noch nicht zu Ende:

* **0,0875 messen** — der Rückfall aus der Übergabe, 87,5 % der Schritte, Ma 0,152. Kompensation:
  `CFD_SGS_SISM_AB=6429`, `_T=2143`, `CFD_SLICE_NEAR_STEPS=2143` (8 mm; 150/50/50 ms, nicht
  ganzzahlig aufgehend — die Rundung ist zu deklarieren).
* **Oder den Einschwingvorbehalt zuerst ausräumen:** `CFD_T_WARMUP` 0,201 → 0,29 steht schon auf
  der Liste, spart 15,6 min **und** beseitigt genau die Ursache, die den Cd-Versatz erklären
  könnte. Die beiden Maßnahmen greifen an derselben Stelle ineinander, und in dieser Reihenfolge
  wird der u_lat-Arm erst deutbar.
* **Der größere Weg**, falls der Kompressibilitätsfehler der Blocker ist: das
  **O(u⁴)-Gleichgewicht**. Siehe `FREMDSOLVER-OPENLUDWIG.md` Abschnitt 2.3 — dort ist es als
  Kompressibilitätskorrektur gebaut und hebt den gültigen Ma-Bereich an. Das ist der Mechanismus,
  der den Zeitgewinn behält und den Ma²-Fehler nicht bezahlt.

**Nicht ohne Entscheid:** eine Änderung der Vorgabe 0,075. Der Code-Default bleibt 0,075, bis
eine 4-mm-Messung vorliegt. `CFD_U_LAT` ist bis dahin ein Messarm, kein Produktionsschalter.
