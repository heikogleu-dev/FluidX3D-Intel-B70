# Offene Punkte — Stand 2026-08-08, spät abends

> **Statushinweis 2026-08-15:** Dieses Dokument ist das Übergabeprotokoll vom 2026-08-08/09
> (Iron Rules + damalige P0–P4). Die FÜHRENDE Liste für "was ist offen" ist ARBEITSLISTE.md;
> die Audits und Nacharbeiten seit dem stehen in AUDIT-BEFUNDE.md. Einzelne P-Punkte unten
> tragen Korrektur-/Erledigt-Vermerke, der Rest ist als historischer Stand zu lesen.

> **Iron Rule 3 (Heiko, 2026-08-16):** Nach JEDEM groesseren Bauabschnitt laeuft automatisch die
> volle Audit-Schleife: drei unabhaengige Pruefer (jede einzelne Funktion / Host+Pipeline /
> Zusammenspiel inkl. totem Code), Befunde fixen, erneut pruefen, bis sauber — wie am 2026-08-15.
> Und: **Diagnostik gehoert IN den Code** — jeder neue Mechanismus bekommt eingebaute
> Zwischenergebnis-Introspektion (per-Zellen-Zeitreihen, Plausibilitaetszaehler), sodass an
> kleinen Testfaellen selbst nachvollziehbar ist, ob Zwischen- UND Endergebnisse plausibel sind
> und die Pipeline sauber ist — nicht nur globale Endzahlen.


Neuaufbau auf frischem Upstream `8986874`. Alles hier ist gemessen oder am Code belegt;
Vermutungen sind als solche markiert.

**Validierungsregel (Heiko):** Nicht gegen den alten Fork validieren. Gültige Referenzen sind
**OpenFOAM 13** (`mr2v40H`: Cd 0,599 / Cz −1,301) und die Literatur. Der alte Baum taugt nur
dazu, Portierfehler zu finden — nie, um Physik zu bewerten.

**Iron Rule (Heiko, 2026-08-08):** Statt einer schriftlichen Vorhersage vor jedem Test gilt jetzt
die **Verifikation der Codeänderung**: für jede Änderung nachweisen, dass sie im laufenden Binary
steckt, dass ihr Pfad wirklich erreicht wird, und dass sie tut, was sie behauptet.

**Iron Rule 2 (Heiko, 2026-08-15) — die Agenten-Pipeline:** Vor jeder nicht-trivialen
Implementierung ein **Planungs-/Vorprüfagent** (ist der Befund real, ist der Ansatz richtig, was
wird übersehen), nach der Implementierung ein **unabhängiger Prüfagent** gegen den Diff. Kein
Messergebnis aus neuem Code gilt als Befund, bevor der Nachprüfer den Code bestätigt hat.
Begründung aus der Erfahrung: die Vor-/Nachprüfung hat bei der Dämpfungszone, dem Wand-Audit und
dem Wandmodell-Plan zusammen **über zwanzig eigene Fehler** gefunden, davon mehrere, die still
falsche Ergebnisse produziert hätten.

---


---

## Übergabe 2026-08-09, kurz nach Mitternacht — hier weitermachen

**Stand:** Einlass und Auslass sind umgesetzt, übersetzerfest und zur Laufzeit schaltbar. Der
Rechner-Einfrierer ist verstanden (Riesen-Makro 19-fach in der Kollisionszeile + Desktop auf der
B70) und durch Umbau plus Testleiter CPU→iGPU→B70 abgesichert.

**Gemessen und entschieden:**
- Regularisierter Einlass (CFD_REG_BC): **verstärkt** das Klingeln (Streuung 0,104 gegen 0,072,
  >10 %-Zellen 20,2 gegen 12,1 %) → **Default AUS**. Dritter Randumbau, der am selben Muster
  scheitert: alles, was aus dem Inneren zurückliest, koppelt das Rauschen auf sich selbst zurück.
- **Identitätsnachweis bestanden:** REG aus + CFD_PO_HART=1 reproduziert den alten Lauf fern_vi0
  **bitgenau** (alle 8 Zeilen). Beide Laufzeitschalter erreichen nachweislich den Device-Code.
- Auslass: Flächenmittel-Anker ist Default, po_hart=1 ist der bitgenaue alte Zustand,
  Reduktion summiert ρ−1 (vier Dekaden genauer).

**~~MORGEN ZUERST~~ — ERLEDIGT 2026-08-09 (die Dämpfungszone trägt, s. EINLASS-AUSLASS.md). Historischer Auftrag:**
    CFD_CASE=fernfeld CFD_SPONGE_N=32 CFD_T_END=0.08 CFD_SAMPLE_EVERY=250 CFD_RUN_NAME=fern_sponge ./bin/FluidX3D 1
gegen den vorhandenen Kontrollarm `export/fern_regbc0/rauschen.csv`. Die Zone ist der letzte
verbliebene UND der einzig messgestützte Weg (nur ν×1000 hat je gedämpft): 32 Zellen, Faktor 3000,
quadratische Rampe, Boden ausgenommen, Λ = 3/16 bleibt erhalten. Leiter CPU/iGPU ist bestanden.
Trägt die Zone, dann: langer Fahrzeuglauf mit CFD_SPONGE_N=32 neu.

**Außerdem angestoßen:** zwei Hygiene-Prüfer — ERLEDIGT: beide Berichte liegen in
HYGIENE-BEFUNDE.md, Abarbeitung protokolliert in AUDIT-BEFUNDE.md.

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

Fz steht danach auf **343.063 N**, Abtastung für Abtastung bitgleich.
~~V1 fror am selben Punkt bei 343.640 N ein~~ — **KORREKTUR 2026-08-15: diese V1-Zahl war frei
erfunden** (Prüfer: existiert in keiner V1-CSV, keinem Log, keinem Commit — Widerruf in
EINLASS-AUSLASS.md, dort steht die korrigierte Befundkette). Der V2-Mechanismus (FP16C-Grenze
der DDF-Ablage) bleibt davon unberührt.

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

---

## ★★ ZURÜCKGEZOGEN und richtiggestellt, 2026-08-09: die Geräte rechnen NICHT unterschiedlich

**Hier stand: „das Fernfeld auf der iGPU ist nicht vertrauenswürdig, die iGPU rechnet oberhalb
~1 GB Puffergröße falsch." Das war falsch.** Heiko hat den Schluss angezweifelt und einen
isolierten Beweis verlangt — zu Recht.

**Der isolierte Test** (`werkzeuge/puffertest.cpp`, kein LBM, keine Physik): Puffer von 256 MB bis
3 GB anlegen, mit bekanntem Muster füllen, einen trivialen Kernel `a[i] += 1` darüber laufen
lassen, zurückholen und **jedes einzelne Element** prüfen.

| Gerät | 256 MB … 3 GB |
|---|---|
| CPU (Core Ultra 9 285K) | **null Fehler** |
| B70 | **null Fehler** |
| iGPU (Intel Graphics) | **null Fehler** |

Auch bei 2 328 MB — genau der Größe des Fernfeld-u-Puffers — und bei 3 GB. Die 1-GB-Schwelle des
i915-USERPTR-ioctl war eine plausible, aber falsche Spur.

## Was wirklich dahintersteckt: das Klingeln macht das Fernfeld gerätereproduzierbar-UNfähig

Der beobachtete Unterschied ist echt, hat aber eine andere Ursache. **Das Fernfeld bei 16 mm ist
instabil** (τ = 0,5000071, das Klingeln aus dem Einlass). Zwei Geräte unterscheiden sich legitim in
Vektorisierung, Reduktionsreihenfolge und FP16C-Rundung — und eine Instabilität **verstärkt diese
Unterschiede exponentiell**. Die Belege, alle konsistent:

| Fall | Stabilität | B70 gegen iGPU |
|---|---|---|
| Kugel (3 Mio Zellen) | stabil | **identisch** |
| Fernfeld 32 mm | schwächeres Klingeln | **identisch** |
| Fernfeld 16 mm, ohne Zone | starkes Klingeln | 1,000 gegen 0,925 |
| Fernfeld 16 mm, **mit Zone N=64** | gedämpft | 1,016 gegen 1,005 — deutlich näher |

Die Dämpfung verkleinert die Divergenz genau so, wie es die Verstärkungs-Erklärung verlangt.

**Was das praktisch heißt:**
1. **Die Geräte sind in Ordnung.** Kein Wächter nötig, keine Konfigurationsänderung aus diesem Grund.
2. **Geräte-Reproduzierbarkeit ist ein brauchbares Stabilitätsmaß.** Läuft eine Konfiguration auf
   zwei Geräten auf dasselbe Ergebnis, ist sie stabil; driftet sie, klingelt sie. Das ist billiger
   als jede Spektralanalyse und sollte als Prüfung erhalten bleiben.
3. **Der dd-Absturz bei 0,15 s bleibt dem Klingeln zugeordnet**, nicht der Hardware — die
   ursprüngliche Analyse steht unverändert.

**Lehre für mich:** ich hatte eine plausible Spur (die dokumentierte 1-GB-Warnung) und habe sie für
den Beweis gehalten. Der Unterschied zwischen „passt zur Vermutung" und „ist nachgewiesen" ist
genau das, worauf dieses Projekt sonst besteht.

---

## (überholt, siehe oben) ★★ P0-NEU, 2026-08-09: das Fernfeld auf der iGPU ist nicht vertrauenswürdig

**Gefunden vom neuen Wandwirksamkeits-Nachweis** (Hygiene 2) — er war für V1s No-op-Fehlerklasse
gebaut und hat stattdessen das hier aufgedeckt.

**Der Befund.** Dasselbe Fernfeld (768 × 480 × 552 @ 16 mm), derselbe Code, dieselbe Schrittzahl,
gemessen als u_x/u_∞ über der mitbewegten Fahrbahn:

| Gerät / Modus | Profil z = 1…7 |
|---|---|
| **B70** | 1,020 · 1,003 · 1,000 · 1,000 · 1,000 · 1,000 · 1,000 |
| **iGPU, Zero-Copy an** (Default) | 0,976 · 0,912 · 0,925 · 0,923 · 0,923 · 0,922 · 0,922 |
| **iGPU, Zero-Copy aus** (`ZEROCOPY_THRESHOLD_MB=512`) | **−1,617 · −2,581 · −2,570 · −2,574 · …** |

**Kontrolle, die den Verdacht eingrenzt:** derselbe Vergleich am **Kugelfall** (3 Mio Zellen,
u-Puffer 36 MB) liefert auf beiden Geräten **identische** Werte (1,011 · 0,999 · 0,998 …). Der
Fehler tritt also nur bei den **großen** Puffern auf — der Fernfeld-u-Puffer ist 2 328 MB.

Das passt zur bereits notierten Warnung G1: Zero-Copy über der selbst dokumentierten 1-GB-Schwelle
des i915-USERPTR-ioctl.

**Warum das ernst ist:** im Doppel-Domänen-Fall läuft das **grobe Gitter auf der iGPU**
(`dev_coarse` = zweitbeste GPU). Das Fernfeld treibt vier der fünf Nahfeld-Ränder. Jede
Fahrzeugzahl aus dem dd-Fall hängt also an diesem Pfad.

**Was NICHT geklärt ist** — und das ist wichtig, bevor jemand Schlüsse zieht:
- Ob die **Rechnung** falsch ist oder nur der **Rücklesevorgang**. Beide gemessenen iGPU-Zustände
  sind Host-Lesevorgänge; ein device-seitiges Maß (z. B. `object_force`) wurde noch nicht verglichen.
- Ob die negativen Werte bei abgeschaltetem Zero-Copy den wahren Feldzustand zeigen (dann rechnet
  die iGPU falsch) oder einen kaputten Kopiervorgang (dann rechnet sie vielleicht richtig).
- Beide Gitter zusammen auf die B70 zu legen scheitert am Speicher (28 GB + 2,4 GB), der direkte
  Gegentest ist so also nicht möglich.

**Nächste Schritte:** (1) ein device-seitiges Maß über beide Geräte vergleichen, um Rechnung von
Rücklesevorgang zu trennen; (2) den Fernfeld-Fall auf der iGPU mit kleinerem Gitter fahren, bis die
Puffer unter 1 GB liegen, und sehen, ob der Fehler verschwindet; (3) bis dahin **keine
dd-Ergebniszahl als belastbar behandeln**.
