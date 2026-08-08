# Offene Punkte — Stand 2026-08-08 abends

Neuaufbau auf frischem Upstream `8986874`. Alles hier ist gemessen oder am Code belegt;
Vermutungen sind als solche markiert.

**Validierungsregel (Heiko, 2026-08-08):** Nicht mehr gegen den alten Fork validieren.
Gültige Referenzen sind **OpenFOAM 13** (mr2v40H: Cd 0,599 / Cz −1,301) und die Literatur.
Der alte Baum taugt nur noch dazu, Portierfehler zu finden — nie, um Physik zu bewerten.

---

## Was läuft

| | Stand |
|---|---|
| Kugelfall | läuft, portiertreu (Geometrie exakt reproduziert: R_eff 19,49 Z, A_eff +8,0 %) |
| Fahrzeugfall | läuft technisch bei 4 mm, 501,5 M Zellen, 28.167 MB VRAM — **aber divergiert** |
| SAT-Voxelizer + Void-Fill | portiert, verifiziert |
| Sparse Tiling | portiert, an der Kugel bit-neutral (T=4/8/16) |
| F-Bounding-Box | portiert, spart gemessen 4,07 GB |
| Druck-Auslass | allgemein (alle 6 Flächen), drei Prüfer, drei Befunde behoben |
| Slice-Export | portiert |

---

## P0 — der Fahrzeugfall divergiert

**Symptom, gemessen** (`export/fz_kontakt/forces.csv`):
Fz erreicht bei 2,8 ms **−11,4 Mio N**, dann `nan`, dann friert das Feld ein.
Die früher berichteten +343.640 N sind der eingefrorene Rest, nicht die Ursache.

**Ursache, zwei Prüfer unabhängig:** Das Fahrzeug berührt den Boden; an den vier
Reifenaufstandsflächen entstehen null- und einzellige Fluidspalte (2694 / 5142 / 3374
Schalenzellen in z = 0/1/2). Halfway-Bounce-Back mit SRT ist dort instabil, weil
**Λ = (τ−½)² = 1,1·10⁻⁹** — die effektive Wandposition wandert ins Fluid, ein
einzelliger Spalt wird effektiv negativ breit.

**Bereits erledigt:** Kontaktzellen bei z=0 verlieren das TYPE_X-Bit und gehen an die
Straße über (2694 Zellen). Das behebt die Kraft-Buchhaltung, **nicht** die Instabilität.

**Nächste Schritte, in dieser Reihenfolge:**
1. `CFD_NU=0.16` → τ = 0,8, Λ = 0,09. Vier Minuten. Beantwortet definitiv, ob Λ die
   Ursache ist. Physikalisch falsch, als Diagnose eindeutig.
2. Falls bestätigt: **TRT mit Λ = 3/16**. Macht die Bounce-Back-Wandposition
   viskositätsunabhängig — die eigentliche Antwort auf ein bodenberührendes Fahrzeug.
   `//#define TRT` steht in `defines.hpp:11` bereit.
3. **`RHO_CLAMP` nachziehen.** Fehlt im neuen Baum vollständig; der alte hatte es mit
   ausdrücklichem Verweis auf den Kontaktpatch (ρ auf [0,5 ; 1,5]).

**Nicht tun:** das Fahrzeug wieder anheben. Das Schweben war die Ursache der
Geschwindigkeitsstagnation in den unteren Millimetern (Befund 2026-08-07) — es wäre
Zukleben, nicht Lösen.

---

## P1 — fehlende Grundlagen, ohne die Zahlen nichts wert sind

- **Kein Kugellauf mit dem aktuellen Baum.** Der letzte ist von 10:51, also vor
  UPDATE_FIELDS, vor den Tile-Änderungen, vor der Durchspülungs-Laufzeit. Die mehrfach
  benutzte Aussage „die Kugel läuft mit demselben Code sauber" ist **nicht belegt**.
- **Kugelvalidierung lief im falschen Zahlenformat.** FP16S statt FP16C (in
  `defines.hpp` waren beide gesetzt, FP16S gewinnt im `#if defined`). Behoben, aber die
  Validierung muss komplett wiederholt werden.
- **Commit `3a265bf` änderte zwei Dinge gleichzeitig** — Bodenkontakt *und*
  UPDATE_FIELDS. Aus den Daten nicht trennbar. Verstoß gegen die eigene Regel.
- **Versperrung 38,4 %.** Querschnitt 4,819 m² gegen A_ref 1,85 m². Windkanalpraxis
  liegt unter 5–10 %. Cd ist so nicht mit OF13 vergleichbar, unabhängig von P0.
  Entweder Domäne quer vergrößern oder Seiten als Freistrom.

---

## P2 — Befunde der Prüfer vom 2026-08-08 am NEUEN Code, noch offen

| | Ort | Was |
|---|---|---|
| H1 | `kernel.cpp:2082` | `object_torque` liest F mit Voll-Domänen-Index, F ist bbox-groß → Out-of-Bounds. Latent (kein Aufrufer). Gleiches in den Graphics- und PARTICLES-Pfaden. |
| H2 | `kernel.cpp:913` | Sparse Tiling: `update_force_field` läuft auf tief innenliegenden Solidzellen, deren Nachbarn in toten Tiles liegen → liest den Papierkorb-Slot 0. Bei **mitbewegten Wänden** steht dort `f_eq(1, u_lat)`, also Impuls ≠ 0. Kann Kräfte erzeugen, die es im dichten Pfad nicht gibt. **Bit-Neutralität am Fahrzeug ist ungeprüft.** |
| H3 | `lbm.cpp:108` | F-BBox ohne Multi-Domain-Absicherung, interpretiert globale als lokale Koordinaten. Sparse Tiling bricht bei D>1 ab, die F-BBox nicht. |
| M1 | `setup.cpp:39` | `env_u` klemmt auf ≥1 → `CFD_PO_FACES=0` heißt nicht „aus", sondern **x_min = der Einlass**. Widerspricht der eigenen Regel im Dateikopf. |
| M4 | `setup.cpp:95` | **Slice zeichnet TYPE_MS-Fluidzellen als Solid** (`flags & (TYPE_S\|TYPE_X)`, und TYPE_MS = 0x03 enthält TYPE_S). Die Diagnosebilder lügen über die Geometrie. Richtig: `(flags & TYPE_BO) == TYPE_S`. |
| M3 | `defines.hpp:32` | SUBGRID unmarkiert mit eingeschaltet; widerspricht dem Kommentar im Kugelfall, der sagt, bei Re = 100…1000 brauche es kein Turbulenzmodell. |
| M2, M5, M6 | `setup.cpp`, `lbm.cpp` | Veraltete oder falsche Kommentare (5-mm-Default, F-BBox-Marge-Begründung, UPDATE_FIELDS-Latenz). |
| B5 | `setup.cpp:440` | y-Halbzellen-Versatz des alten Baums fehlt (310,0 statt 310,5). **Gegengeprüft: die STL hat null Facetten in der Mittelebene, die Degeneration kann nicht auslösen.** Trotzdem eine Abweichung. |

---

## P3 — noch zu portieren

1. **Wandmodell Xue/Lu.** Spalding-Inversion nach u_τ (Newton, 5 Iterationen),
   τ_w = ρu_τ², als Bodyforce auf die wandnahen Fluidzellen. Vorlage:
   `../FluidX3D/src/kernel.cpp` (`apply_wall_model_xuelu`).
   **Ohne die Knöpfe, die am 2026-08-08 als nicht angeschlossen nachgewiesen wurden:**
   `WM_WALL_FRAME` (Aktivliste enthält nur den Körper, nie die Wände),
   `wall_adj_flag` device-seitig (wird nie dereferenziert),
   `WALL_VISC_BOOST_TARGET_BL_MM` (Tier-Sweep war bit-identisch).
   Mozaffari-APG trägt gemessen 0,87 % zu Cd bei — Komplexität lohnt nicht.
   **Erst nach P0**: das Wandmodell stabilisiert den Kontaktpatch nicht, das macht Λ.
2. **Lagrava-Latt-4:1-Kopplung.** Zurückstellen, bis der Single-Domain-Fall eine
   belastbare Zahl liefert — sonst weiß man bei einer Abweichung nicht, woher sie kommt.
3. **NUT_PATH_A** — nur, wenn eine Messung zeigt, dass es etwas verändert. An der Kugel
   war es wegen des Escudier-Caps (Mischweglänge 0,082 Zellen) praktisch inert.

---

## P4 — Validierung, die dem Projekt fehlt

- **Kugel freistehend gegen die Standard-Widerstandskurve** (Clift/Grace/Weber) bei
  Re_D = 100…1000. Die erste **externe** Referenz — bisher wurde nur gegen den eigenen
  alten Baum und gegen OF13 validiert. `CFD_KUGEL_FREE=1` ist gebaut.
  Vorher die Querausdehnung prüfen: bei nur 3 D ist die Versperrung 3,5 %.
- **Sparse Tiling am Fahrzeug**: T = 8/16/32/64, Durchsatzverlust und VRAM-Ersparnis,
  plus Bit-Neutralität (siehe H2).

---

## Regeln, die sich heute bewährt haben

- **Immer nur eine Größe ändern.** Zweimal dagegen verstoßen, zweimal Zeit verloren.
- **Nach jedem Umbau den funktionierenden Gegenfall testen**, nicht nur den neuen.
  So wurde der F-BBox-Konstruktorfehler gefunden (alle Kräfte exakt null).
- **Slices von Anfang an mitlaufen lassen.** Ein Bild hat in einer Sekunde gezeigt, was
  drei Läufe an Zahlen nicht gezeigt haben.
- **Kontrollarm nicht vergessen.** Ein A/B ohne dritten Arm lädt zum Fehlschluss ein —
  die „+1,35 % = der Erweiterungsstapel" waren so entstanden und falsch.
- **Den richtigen Parameter variieren.** ν × 200 änderte Λ von 1,1e-9 auf 3,1e-5 —
  beides faktisch null. Der Test war ungültig, der Schluss daraus falsch.
