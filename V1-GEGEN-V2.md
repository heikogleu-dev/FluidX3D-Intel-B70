# V1 gegen V2 — was wirklich fehlt, und was nur so aussieht

Stand 2026-08-08, von einem unabhängigen Prüfer erstellt. Zentrale Vorgabe war: für **jede**
Fähigkeit, die V1 hat und V2 fehlt, prüfen, ob sie in V1 überhaupt **gewirkt** hat.

---

## Der Rahmen, der alles andere erklärt

V1s Default-Binary ist eine **Ein-Domänen-Kugel**. Der gesamte Fahrzeug- und Kopplungscode
(~1730 Zeilen) steht hinter `#if PHASE_7G` und ist im gebauten Programm wegpräprozessiert.
**36 % von V1s Host-Code — 2694 von 7537 Zeilen — sind präprozessor-tot.**

Eine Liste „V1 hat X, V2 nicht" wäre deshalb irreführend.

| | Upstream 8986874 | V2 | V1 |
|---|---:|---:|---:|
| Quellzeilen (ohne lodepng) | 14 155 | **14 979** | 20 165 |
| davon eigener Code | — | **+824 (5,8 %)** | +6 010 (42,5 %) |
| Env-Schalter | — | **30**, alle wertauswertend | 97, davon 44 mit `CFD_X=0` → **ein** |

---

## Was V2 fehlt und **wirklich** fehlt

Nur Fähigkeiten, die in V1 nachweislich gewirkt haben — nach Bedeutung für den Fahrzeugfall:

1. **Bodengeschwindigkeits-Prägung.** Der einzige V1-Befund mit großem gemessenem Effekt:
   **Cz −31,0 %** (5,0–9,4 σ), Cd +1,40 % (1,7 σ, als Einzelbefund nicht belastbar). Wirkt
   allerdings erst seit dem MS-Guard-Fix von 2026-08-08, 28 Minuten vor V2s erstem Commit —
   alle früheren Vergleiche sind entwertet. **V1 sagt selbst dazu:** „Er ist kein
   Geschwindigkeitsrand, sondern ein Equilibrium-Reset […] `apply_moving_boundaries` hat bereits
   eine korrekte Halfway-Bounce-Back-RB. Möglicherweise ist die richtige Antwort: ersatzlos
   entfernen." Das ist zu **messen**, nicht zu portieren.
2. **`RHO_CLAMP`.** In V1 auf dem Pfad, aber nie gefeuert — die Grenzen liegen Faktor ~33 von
   jedem realen ρ entfernt. Der Wert liegt genau darin, dass es bei gesunder Lösung nichts
   kostet. Für V2 relevant, weil τ = 0,50003 und der Kontaktpatch enge Spalte erzeugt.
3. **Mozaffari-APG.** Wirksam, isoliert vermessen: **−0,87 % auf Cd**. Bewusst weggelassen; die
   Entscheidung ist durch die Messung gedeckt, die Größe gehört ins Fehlerbudget.
4. **NUT_PATH_A.** An der Kugel wegen des Escudier-Caps (0,082 Zellen) praktisch inert. Am
   Fahrzeug ist die Lage völlig anders — dort nie isoliert vermessen.
5. **Xue/Lu-Wandmodell.** Trägt an der Kugel ~0,5 % der Wandschubspannung; über der Fahrbahn ist
   τ_w strukturell null. Am Fahrzeug nie isoliert vermessen.
6. **Ablösewinkel-Sonde θ_sep** — wirksame Diagnose mit eingebauter Gültigkeitsprüfung.
7. **Druck-/Reibungs-Aufspaltung der Kraft.** Die einzige Handhabe, um eine Cd-Abweichung
   zwischen Form- und Reibungswiderstand zuzuordnen.
8. **`CFD_DUMP_CL` / `CFD_DUMP_DEFINES`** (~20 Zeilen). Das Werkzeug gegen die teuerste
   Fehlerklasse des Projekts.

## Was V2 **nicht** fehlt — in V1 nachweislich unwirksam

`WM_WALL_FRAME` (A/B bit-identisch, max |Δu| = 0,000000 m/s) · `wall_adj_flag` / `WALL_VISC_BOOST`
(device-seitig **nie dereferenziert**) · Tier-Sweep `TARGET_BL_MM` (20/7/3,5 mm bit-identisch,
`ceil` liefert immer 1) · `WM_YMATCH` (kollabiert auf 0, gesampelt wird die Wandzelle selbst —
und die Ausgabe behauptet etwas anderes) · HRR · σ-Modell · SVF („WIP/BLOCKIERT, blow-up't") ·
WALE · SA-DDES / RAST (Lauf bit-identisch zu BASE) · · **f_neq-Kopplung** (494 MB von 598 MB Lift-Puffer sind
f_neq, das nie gelesen wird) · Zeitinterpolation (vollständig verdrahtet, **nie gerufen**) ·
fein→grob-Rückkopplung · URF-/Richtungs-Blending („AUSGEKLAMMERT") · Π_neq-Handover (183 Zeilen,
10,5 MB VRAM, 4 Kernel, die nie starten) · `VISCOUS_STRESS_DIRECT` („Sackgasse") · Sponge-Layer
(Entfernung ließ das u-Feld bit-identisch) · ELIBB / Bouzidi / Free-Slip · Ladd-Term
(Default **aus**, während der Kommentar zwei Zeilen darüber „DEFAULT AN" behauptet) ·
Halbzellen-y-Versatz.

---

## Was V2 kann, das V1 nicht oder nicht sauber konnte

1. **TRT mit Λ = 3/16 aktiv.** In V1 vorhanden, aber auskompiliert — „toter Code".
2. **`UPDATE_FIELDS` unbedingt an.** In V1 in **keiner** Konfiguration definiert: u/ρ wurden nur
   alle 100 Schritte aufgefrischt, während das Wandmodell sie **jeden** Schritt las. Es rechnete
   auf einem bis zu 99 Schritte alten Feld.
3. **Druck-Auslass allgemein** — alle sechs Flächen, ρ_out vorschreibbar, Kanten und Ecken
   eindeutig lösbar, Eindeutigkeit geprüft. V1 leitet die Richtung geräteseitig ab und kann
   Kanten nicht auflösen.
4. **Sparse Tiling bit-neutral verifiziert**, plus zwei Fehlerklassen behoben, die V1 latent hat.
5. **Kopplung mit eingebautem Wirksamkeitsnachweis** — Identität an den Deckungspunkten,
   Abweichung vom Freistrom, Kantenkonflikt. Hat dabei drei eigene Fehler gefunden.
6. **Messhygiene an der Quelle:** CSV mit Punkt statt Dezimalkomma (das Komma hat einen Parser
   still scheitern lassen); Block-SEM statt Populations-Sigma; Cd auf **beide** Bezugsflächen
   (V1 rechnete A_eff aus und **wandte es nie an** — Bias +8,0 % bei 12 mm, +3,5 % bei 6 mm, also
   ein **Scheingradient in jeder Gitterstudie**); NaN-Abbruch mit gesicherter CSV.
7. **Hohlraum-Überwachung**, Bodenkontakt- und Randbedingungs-Zählung, Blockage- und
   A_eff-Report.

---

## Die drei nächsten Schritte, laut Prüfer

1. **Die Kugel gegen eine externe Referenz messen — mit dem aktuellen Baum.** Es gibt bis heute
   keinen Kugellauf mit TRT, UPDATE_FIELDS und korrektem FP16C. Ohne diesen Anker ist jede
   Fahrzeugzahl unverankert.
2. **Die Boden-Randbedingung entscheiden statt V1s Version zu portieren.** Cz −31 % ist der
   größte gemessene Einzelhebel — und der am schlechtesten begründete. A/B mit Kontrollarm.
3. **Den Wirksamkeitsnachweis zur Pflicht machen.** Die teuerste V1-Erfahrung ist keine
   einzelne Fehlerursache, sondern eine Fehlerklasse: sechs tote Knöpfe, drei nie emittierte
   Defines, ein Kernel mit null bearbeiteten Zellen, eine 16-Lauf-Testreihe über eine Variable,
   die es im Quelltext nicht gibt.
