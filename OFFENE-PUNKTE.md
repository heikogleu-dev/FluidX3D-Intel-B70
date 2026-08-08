# Offene Punkte — Stand 2026-08-08, spät abends

Neuaufbau auf frischem Upstream `8986874`. Alles hier ist gemessen oder am Code belegt;
Vermutungen sind als solche markiert.

**Validierungsregel (Heiko):** Nicht gegen den alten Fork validieren. Gültige Referenzen sind
**OpenFOAM 13** (`mr2v40H`: Cd 0,599 / Cz −1,301) und die Literatur. Der alte Baum taugt nur
dazu, Portierfehler zu finden — nie, um Physik zu bewerten.

**Iron Rule (Heiko, 2026-08-08):** Statt einer schriftlichen Vorhersage vor jedem Test gilt jetzt
die **Verifikation der Codeänderung**: für jede Änderung nachweisen, dass sie im laufenden Binary
steckt, dass ihr Pfad wirklich erreicht wird, und dass sie tut, was sie behauptet.

---

## Was läuft

| | Stand |
|---|---|
| Kugelfall | läuft; Validierung mit FP16C/TRT steht noch aus |
| Fahrzeug, Einzelgitter | läuft, aber 38,4 % Versperrung → nicht mit OF13 vergleichbar |
| **Fahrzeug, Doppel-Domäne** | **läuft, Versperrung 2,74 % — kippt aber bei ~0,15 s** |
| SAT-Voxelizer + Void-Fill | portiert, verifiziert |
| Sparse Tiling | portiert, an der Kugel bit-neutral; am Fahrzeug ungeprüft |
| F-Bounding-Box | portiert, spart gemessen 4,07 GB |
| Druck-Auslass | allgemein (alle 6 Flächen), drei Prüfer, drei Befunde behoben |
| Slice-Export | portiert, alle 10 ms, alles in einem Lauf-Ordner |
| Kopplung grob → fein | portiert, Wirksamkeit **bit-genau nachgewiesen** |
| Leistungsindex + Phasenprofil | **neu, aus V1 nachgezogen** |

**Leistung, gemessen 2026-08-08:** Index **≈ 9.100** s_wall/s_phys gegen V1s **~12.000** bei
identischer Konfiguration — V2 ist rund **24 % schneller**, und das mit eingeschaltetem
`UPDATE_FIELDS`, das V1 gar nicht hatte (kostet 10–15 %).

---

## P0 — der Doppel-Domänen-Lauf kippt bei 0,15 s

**Gemessen** (`export/dd_lauf01/forces.csv`): Cd fällt vom Anlaufstoß 53,9 sauber über
8,0 → 5,3 → 2,8 → 1,46 → 0,43 → 1,02 → 0,94. Dann, zwischen 0,141 s und 0,161 s:

| Zeit | Cd | Cz |
|---|---:|---:|
| 0,141 s | 0,942 | −0,696 |
| **0,161 s** | **0,169** | **336,398** |
| 0,201 s | 0,169 | 336,398 |

Fz steht danach auf **343.063 N**, Abtastung für Abtastung bitgleich. **V1 fror am selben Punkt
bei 343.640 N ein** — praktisch dieselbe Zahl, also derselbe Mechanismus: keine Kraft, sondern
die auf die FP16C-Grenze gelaufene DDF-Ablage.

**Der NaN-Wächter hat das nicht gefangen** — die Werte sind endlich, nur eingefroren. Behoben:
drei bitgleiche Abtastungen hintereinander gelten als Beweis, dazu ein Größentest (|Cd| oder
|Cz| über 20).

**Spur, aus den Schnitten:** Das **Fernfeld klingelt**, und zwar vom **Einlass** ausgehend. Bei
51 ms ist es eine schmale, heftig oszillierende Säule bei x = 0; bei 151 ms füllen horizontale
Streifen (Wechsel von Zeile zu Zeile in z, |u| zwischen unter 15 und über 45 m/s) die ganze
Domäne. τ_c = 0,500007 — das grobe Gitter ist praktisch reibungsfrei, akustische Moden werden
nicht gedämpft. Dieses Feld treibt vier der fünf Nahfeld-Ränder.

Im Nahfeld läuft die Stagnation von hinten nach vorn durch (Heiko-Beobachtung am Schnitt).

**Nächste Schritte, in dieser Reihenfolge:**
1. **Isolieren:** leeres Fernfeld, kein Fahrzeug, keine Kopplung. Ringt ein leerer grober Kanal
   schon, liegt es an Randbedingung und Viskosität des groben Gitters.
2. **RHO_CLAMP** nachziehen. Jetzt akut, nicht mehr vorsorglich.
3. Ein- und Auslassbehandlung des groben Gitters überarbeiten (siehe eigener Abschnitt unten).

---

## P1 — Ein- und Auslass: die Analyse, die V1 nicht hatte

Das ist kein Portierrückstand, sondern eine offene Frage, die im alten Baum nie gestellt wurde.

**Einlass (beide Gitter), heute:** TYPE_E mit ρ = 1 und u = u_∞. `stream_collide` setzt dort
`f = f_eq(ρ, u)` in jedem Schritt — ein voller Gleichgewichts-Reset, der den
Nichtgleichgewichtsanteil verwirft. Bei τ ≈ 0,5 gibt es nichts, was die dadurch erzeugte
akustische Störung dämpft; sie läuft in die Domäne und wird an den anderen Rändern reflektiert.
Genau das zeigt der Fernfeld-Schnitt.

**Auslass (beide Gitter), heute:** ρ fest auf 1,0, u aus der Innenzelle kopiert. Am **Nahfeld**
sitzt dieser Rand **0,45 Fahrzeuglängen hinter dem Heck**, also mitten im Totwasser, und ist als
einzige Fläche von der Kopplung ausgenommen. Der Basisdruck — bei einem Fahrzeug der größte
Einzelbeitrag zu Cd — wird damit nicht vom Fernfeld bestimmt, sondern auf Freistromdichte
fixiert.

**Zu prüfende Wege, keiner davon heute belegt:**
- Ein- und Auslass **nicht-reflektierend** machen (charakteristische Randbedingung, oder eine
  Dämpfungszone über einige Zellen), statt hart vorzuschreiben.
- Am groben Einlass die Störung gar nicht erst erzeugen: den Gleichgewichts-Reset durch eine
  Bedingung ersetzen, die den Nichtgleichgewichtsanteil stehen lässt.
- Am Nahfeld-Auslass ρ nicht fixieren, sondern extrapolieren — dann fehlt allerdings der
  Druckanker, und der muss von woanders kommen.

---

## P2 — Befunde der Prüfer vom 2026-08-08, noch offen

| | Ort | Was |
|---|---|---|
| G1 | `opencl.hpp` | Zero-Copy des groben u-Puffers (2,44 GB) liegt über der selbst dokumentierten 1-GB-Hangschwelle des i915-USERPTR-ioctl. Gegenmaßnahme `ZEROCOPY_THRESHOLD_MB` bewusst **nicht** gesetzt — die Konfiguration hat mehrfach alloziert und gerechnet. |
| G4 | `lbm.cpp:995` | Speicher-Vorabprüfung rechnet F über die **volle** Domäne und ignoriert die Bounding-Box: 32.043 MB nominal gegen 32.767 MB verfügbar, real belegt 28,1 GB. Geht durch, aber knapp. |
| U1 | `lbm.hpp:420` | `Memory_Container` für F rechnet mit der vollen Domänengröße, das darunterliegende `Memory` ist bbox-groß. `lbm.F[i]` liefe aus dem Puffer. Kein Aufrufer. |
| U2 | `info.cpp` | Die Konsolenanzeige ist bei zwei LBM-Instanzen unbrauchbar (mischt grobe Zellzahl mit feiner Schrittzeit). Deshalb der eigene Leistungsindex. |
| M1 | `setup.cpp` | Nach der Voxelisierung wird nur `flags` zurückgelesen, nicht `u`. Heute folgenlos (das Fahrzeug steht); dokumentiert an der Stelle. |
| M2 | `defines.hpp` | `VOLUME_FORCE` ist aktiv, tut aber nachweislich nichts (keine Volumenkraft übergeben, F nur auf Solidzellen). Abschalten spart Rechenzeit. |
| B5 | `setup.cpp` | y-Halbzellen-Versatz des alten Baums fehlt. **Gegengeprüft: die STL hat null Facetten in der Mittelebene**, die Degeneration kann nicht auslösen. |

---

## P3 — noch zu portieren, nach Bedeutung

Grundlage ist die Prüfung in [V1-GEGEN-V2.md](V1-GEGEN-V2.md): **nur was in V1 nachweislich
gewirkt hat**.

1. **RHO_CLAMP** — jetzt P0, siehe oben.
2. **Bodengeschwindigkeits-Prägung.** Cz −31 % in V1, der größte gemessene Einzelhebel dort —
   und der am schlechtesten begründete. V1 selbst schreibt: „Möglicherweise ist die richtige
   Antwort: ersatzlos entfernen." **Messen, nicht portieren.**
3. **Mozaffari-APG** — −0,87 % auf Cd, gemessen. Gehört ins Fehlerbudget.
4. **Druck-/Reibungs-Aufspaltung der Kraft** — die einzige Handhabe, um eine Cd-Abweichung
   zwischen Form- und Reibungswiderstand zuzuordnen.
5. **Ablösewinkel-Sonde θ_sep** — wirksame Diagnose mit eingebauter Gültigkeitsprüfung.
6. **`CFD_DUMP_CL` / `CFD_DUMP_DEFINES`** (~20 Zeilen) — das Werkzeug gegen die teuerste
   Fehlerklasse des Projekts.
7. **Wandmodell Xue/Lu** — ohne die nachweislich toten Knöpfe. Am Fahrzeug nie isoliert vermessen.
8. **NUT_PATH_A** — an der Kugel inert, am Fahrzeug nie vermessen.

---

## P4 — Validierung, die dem Projekt fehlt

- **Kugel freistehend gegen die Standard-Widerstandskurve** (Clift/Grace/Weber) bei
  Re_D = 100…1000. Die erste **externe** Referenz. Es gibt bis heute keinen Kugellauf mit TRT,
  UPDATE_FIELDS und korrektem FP16C.
- **Sparse Tiling am Fahrzeug**: T = 8/16/32/64, Durchsatz und VRAM, plus Bit-Neutralität.

---

## Regeln, die sich bewährt haben

- **Verifikation der Codeänderung** statt Vorhersage: greift der Pfad, tut er das Behauptete?
- **„Läuft" ist nicht „wirkt".** Der eingebaute Kopplungsnachweis prüft beides getrennt.
- **Immer nur eine Größe ändern.** Mehrfach dagegen verstoßen, mehrfach Zeit verloren.
- **Nach jedem Umbau den funktionierenden Gegenfall testen**, nicht nur den neuen.
- **Slices von Anfang an mitlaufen lassen.** Ein Bild hat den Motorraum und den Kipppunkt
  gezeigt, den drei Läufe an Zahlen nicht gezeigt haben.
- **Kontrollarm nicht vergessen.** Ein A/B ohne dritten Arm lädt zum Fehlschluss ein.
- **Ein Wächter, der nur NaN kennt, ist kein Wächter.** Der teuerste Ausfall des Tages war
  endlich, konstant und still.
