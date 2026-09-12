# TODO 2 — die sieben Zugriffe: Bauplan

**Stand 12.09.2026.** Planungsschritt nach Iron Rule 7, vor jeder Zeile Code. Fassung:
FluidX3D-v2, master, Commit 68568dc + u_lat-Schalter. Auftrag Heiko: „alles untersuchen und
beweisen und die möglichst hohe Performance und VRAM-Einsparung bei maximaler Genauigkeit".

---

## 1 · Der Befund der Übergabe stimmt nicht ganz — und die Lücke ist gefährlich

„Sieben Zugriffe" ist eine **Textzählung** von `u[`/`rho[` im Rumpf von `stream_collide`. Sie
übersieht die Leser hinter Hilfsfunktionen. Selbst nachgeprüft:

**`deriv_reg` liest `u` an den sechs Achsnachbarn JEDER TYPE_E-Zelle, jeden feinen Schritt**
(`src/kernel.cpp:1643`, aufgerufen ab `:3431` im Zweig `if(flagsn_bo==TYPE_E)`). Das Gate
`b!=TYPE_S && b!=TYPE_E` heisst: gelesen wird an echten **Fluidzellen**. `REGULARIZED_BOUNDARIES`
ist aktiv — die `#undef`-Liste in `src/defines.hpp:135` steht unter `#ifdef BENCHMARK` und greift
in diesem Bau nicht.

Diese Nachbarn liegen auf den **Aussenflächen der Nahfeldbox** und damit vollständig ausserhalb
`f_bbox`. Eine Maske aus F-BBox plus Auslassebene, wie die Übergabe sie vorschlägt, würde sie
nicht enthalten: der regularisierte Rand rechnete seine Scherung dann aus einem bis zu drei
Schritte alten Feld. Kein Absturz, kein Zähler, plausible Zahlen — **genau der Stillfehler, den
die Übergabe selbst als Hauptgefahr benennt.** Die betroffene Zelle ist die erste Fluidzelle
hinter der Einlassfläche x⁻.

---

## 2 · Vollständige Leserkarte, selbst geprüft

**Nicht gebaut und damit irrelevant:** `SURFACE`, `TEMPERATURE`, `PARTICLES`, `GRAPHICS`
(`src/defines.hpp:53,54,78,82` — alle auskommentiert). Damit entfallen die rho-Leser in
`surface_0`/`surface_3`/`graphics_*` und der Transferpfad (D=1).

### rho — die Leserschaft ist winzig

| Leser | Ort | liest wo | Kadenz |
|---|---|---|---|
| `po_reduce_mean` | `kernel.cpp:3988` | `rho[po_interior]`, 306 704 Zellen | jeder feine Schritt |
| `apply_pressure_outlet` | `:4063-4064` | `rho[m]`, m = dieselbe Innenzelle | jeder feine Schritt |
| `stream_collide` TYPE_E | `:2987` | `rho[n]` an der **eigenen** Zelle | jeder Schritt |
| Wandmodell APG | `:2110` | `rho[j[ia]]` an Facettennachbarn | **aus** (`CFD_FAC_APG` Default 0) |
| `apply_velocity_inlet` | `:4080` | `rho[m]` | im dd-Nahfeld No-Op |
| `extract_plane_macros` | `:4166` | `rho` auf 5 Ebenen | grober Schritt, **Fernfeld** |
| Host (Slices, cp, VTK, Sonden) | `setup.cpp` | ganzes Feld | Sample-/Slice-Kadenz |

**`po_interior` ist geometrisch eingrenzbar:** die Innenzelle ist der nächste Fluidnachbar im
Chebyshev-Abstand 1 (`src/lbm.cpp:2717-2724`), bei 1587 von 306 704 über eine 26er-Suche. Zwei
Zellschichten am Auslass sind also eine **konstruktive Obermenge**, und das ist reine Arithmetik.

### u — die Leserschaft ist gross und hat Stencils

`deriv_reg` (6 Nachbarn jeder TYPE_E-Zelle), `sgs_fdwand` (`:4731`, 6 Nachbarn von 3 129 185
Facettenzellen), `fac_nachbar_ab` (`:4817`, 1 Link), `apply_pressure_outlet` (`:4063-4067`),
`schale_extract` mit `mittel=1` (`:4244-4290`, je Deckungspunkt ein 4³-Block, 1 859 770 Punkte),
dazu der Host. Latente Erweiterungen: `CFD_SGS_BAND=1` (8 Wandlagen) und `CFD_FAC_APG>0`.

---

## 3 · Der Deckel, und er begrenzt die Erwartung

Die Maßnahme wirkt im **Nahfeld**. Der Schlupf des Fernfelds beträgt 8,13 % (`PERFORMANCE.md`).
Sobald das Feinfenster um mehr als 8,13 % schrumpft, ist die iGPU der kritische Pfad. Die in der
Übergabe genannte Spanne „−6,5 bis −9,5 %" ist in ihrer oberen Hälfte **konstruktiv
unerreichbar**, solange nur das Nahfeld angefasst wird.

**Folgerung für die Reihenfolge:** das Fernfeld ist nicht der Nachtisch, sondern der Hauptgang.
`lbm_c` hat keine Facetten (`setup.cpp:6385`) und `s_sgs_fdwand` ist für `lbm_c` genullt
(`:5939`) — die ganze Facetten- und SGS-Leserklasse fällt dort weg.

---

## 4 · Bauabschnitte, in dieser Reihenfolge

| # | Schritt | Gewinn | Risiko | Abnahme |
|---|---|---|---|---|
| **0** | **`UPDATE_FIELDS`-Sonde**: 25-s-Paar mit dem Schalter an/aus | misst die **Obergrenze** aller Varianten | null | keine, reine Messung, **null Codezeilen** |
| **1** | **rho-Teil Nahfeld**: `rho[n]` nur an der Auslassschicht und an Hostlese-Schritten | 4,2 % Verkehr | klein, Obermenge trivial | Bytevergleich |
| **2** | **Fernfeldmaske**: rho+u nur an TYPE_E-Innenschale, po/vi und den 5 Entnahmeebenen | ~17 % des groben Schrittverkehrs, **auf dem kritischen Pfad** | mittel | Bytevergleich |
| **3** | **u-Teil Nahfeld**: Maske = F-BBox ∪ **TYPE_E-Innenschale** ∪ Auslassschicht ∪ N2F-Deckung, nur auf Substeps < ratio | bis 7,3 % Verkehr, gedeckelt | **hoch** | Bytevergleich + Gift-Arm |
| **4** | **Weg B, 2 Byte**: `FP16S(rho−1)` und FP16S(u) | **−3 961 MiB Nahfeld**, −8,4 % Verkehr | ändert Zahlen | Driftnachweis über den **ganzen** Lauf |

Schritt 0 kostet nichts und beantwortet vorab, ob der Rest die Wanduhr überhaupt bewegt — die
Anzeige ist laut eigener Korrektur um 47 % überzeichnet, und ob der Kernel im Schreibpfad
bandbreitengebunden ist, ist im Repo **nirgends belegt**.

---

## 5 · Die Abnahme, die den Stillfehler wirklich fängt

Bitgleichheit allein genügt **nicht**: eine Lücke, die nur an der Sample-Kadenz gelesen wird,
zeigt sich erst nach ≥ 25 Grobschritten. Drei Stufen:

1. **Obermengenbeweis auf dem Host im Aufbau**, nach dem Muster `alloc_f_liste`
   (`src/lbm.cpp:837 ff`, „DIE MASKE IST BEWUSST EINE OBERMENGE"): die vier Lesermengen sind auf
   dem Host vollständig konstruierbar — TYPE_E um 1 dilatiert, Facettenliste um 2, po/vi direkt,
   N2F-Deckungsblöcke. Jede Verletzung ist ein `print_error` **mit Zellindex**. Das ist der
   einzige Test, der konstruktiv statt statistisch ist.
2. **Wirkpfadzähler** nach Muster Slot 77 (Soll 0): in `sgs_fdwand` und im `deriv_reg`-Block die
   gelesene Nachbarzelle gegen das Maskenprädikat testen und bei Verletzung einen sättigenden
   Slot hochzählen. Freie Slots ab **204** (`src/lbm.hpp:208`). Damit ist der Fehler auch im
   Produktionslauf laut.
3. **Gift-Arm statt NaN-Arm.** NaN taugt hier nicht: übersetzt wird mit `-cl-finite-math-only`
   (`src/opencl.hpp:317`), und der Fork hat das schon bezahlt (`src/kernel.cpp:4213`: „isfinite
   ist unter -cl-finite-math-only toter Code"). Stattdessen ein **endlicher** Giftwert in den
   Nicht-Masken-Zellen, geschrieben statt übersprungen: gleicher Verkehr, kein Gewinn, aber jeder
   übersehene Leser erzeugt im selben Schritt einen sichtbar falschen Wert, und der
   Kräftewächter (`setup.cpp:7963`, |Cd| > 20) kippt beim nächsten Sample.

---

## 6 · Offen, und ehrlich benannt

* **Verkehr → Wanduhr.** Ob die B70 im `stream_collide` schreibbandbreitengebunden ist, ist
  nirgends belegt. Schritt 0 beantwortet genau das, bevor Code entsteht.
* **Der Deckel ist beweglich** und schrumpft mit jeder Nahfeldmaßnahme — nach jedem Schritt neu
  bestimmen.
* **Zwei schlafende Arme** erweitern die Lesermenge, ohne dass die Maske es merkt:
  `CFD_SGS_BAND=1` und `CFD_FAC_APG>0`. Beide brauchen einen No-Op-Wächter an der Lesestelle,
  der den Lauf anhält, wenn sie zusammen mit der Maske gesetzt werden.


---

## 7 · Stand 12.09.2026, 12:15 — Schritt 0 ist gemessen

**Die Obergrenze ist höher als die Übergabe annahm.** 8-mm-Fahrzeug, 50 ms, zwei Arme, die sich
allein im Binary unterscheiden (`UPDATE_FIELDS` an/aus), Zeile wortgleich:

| | Durchsatz | Leistungsindex (s_wall je s_phys) |
|---|---:|---:|
| `UPDATE_FIELDS` an (heute) | 1905 MLUPs | 752 |
| `UPDATE_FIELDS` aus | 2316 MLUPs | **644** |

**Obergrenze für TODO 2: −14,4 % Wanduhr**, nicht die −6,5 bis −9,5 % der Übergabe. 16,8 %
weniger Verkehr bringen 14,4 % Wanduhr — der Kernel ist im Schreibpfad also nahezu eins zu eins
bandbreitenempfindlich. Damit ist die offene Frage aus Abschnitt 6 beantwortet.

**Vorbehalt, der dazugehört:** ohne `UPDATE_FIELDS` bekommen die Leser Müll, und das Wandmodell
steigt an manchen Zellen früher aus (`ut<1e-6`). Ein Teil der 14,4 % kann eingesparte
Wandmodell-Arbeit sein statt eingesparter Schreibvorgänge. Als **Obergrenze** bleibt die Zahl
gültig. Der Rückbau ist bewiesen: das Binary nach dem Wiederherstellen ist bitgleich zum Stand
davor (md5 `800542b6ce7b8bb3d76cefb41bafa0ac`).

### Ein bequemer Weg ist ausgeschlossen

Die Idee, `rho` seltener zu schreiben und vor jedem Hostlesen mit dem vorhandenen
`update_fields`-Kernel aufzufrischen, **trägt nicht**. Der Kernel wird zwar unbedingt erzeugt
(`src/lbm.cpp:485`) und nur sein Aufruf ist unter `#ifndef UPDATE_FIELDS` stillgelegt
(`:1252-1259`) — aber er rechnet `rho`/`u` aus dem Verteilungsfeld **zum Zeitpunkt seines
Aufrufs**. Nach einem Zeitschritt steht dort schon der nächste Zustand; er schriebe also andere
Werte als `stream_collide` hinterlassen hätte, und die Bitgleichheit fiele sofort.

**Folgerung: der Voll-/Sparsam-Zustand muss als Kernelparameter vom Host kommen**, je Schritt
gesetzt. Das ist gebaut werden kann, aber es ist ein Bauabschnitt mit eigener Auditschleife.

### Die Einbaupunkte sind vermessen

* **Maske für `rho` ist bewiesen:** der Druckauslass ist die **x_max-Fläche**
  (`face_mask=0x2`, `src/lbm.cpp:2656`), und `po_interior` entsteht aus einer 26er-Nachbarsuche
  um die Flächenzelle (`:2717-2724`). Jeder Innenzellenkandidat hat damit `x >= Nx-2`.
  **`x >= def_Nx-2` ist konstruktiv eine Obermenge** und reine Arithmetik (`n % def_Nx`).
* **Die Signatur ist gefahrlos erweiterbar:** `stream_collide` verwendet bereits das sichere
  Klammermuster (`kernel.cpp:2866`: `stream_collide)+"("+R(`), und an absoluten Indizes sind nur
  0 (`fi`) und 4..7 (`t, fx, fy, fz`) verdrahtet (`lbm.cpp:1213`, `:1233`). Alle übrigen
  Positionen sind relativ zu `f_param_sc`/`fac_param_pos` und wandern automatisch mit. Ein neuer
  Skalar hinter `fz` macht `set_parameters(4u, t, fx, fy, fz, rho_voll)` zusammenhängend.
* **Die Kadenz passt:** Hostlesen findet an der Sample-Kadenz statt
  (`sample_every*ratio` = 100 feine Schritte im Standard), und `CFD_SLICE_NEAR_STEPS` ist laut
  bestehendem Wächter (`setup.cpp:7051`) ein Vielfaches davon. **Offene Kante:** der LETZTE
  Schritt eines Laufs ist kein Vielfaches (25 050 mod 100 = 50 bei 8 mm) — der Abschlusspfad
  braucht ein erzwungenes Vollschreiben, sonst ist der End-Dump veraltet. Der Bytevergleich
  würde das fangen, aber es gehört vorher gebaut.

---

## 8 · Schritt 1 ist gebaut und abgenommen (12.09.2026, 12:37)

**`CFD_RHO_SPARSAM`**, Vorgabe 0. Gattert die rho-Schreibstelle in `stream_collide` auf
`x >= Nx-2` plus jeden Schritt der Sample-Kadenz. `u` bleibt unangetastet. Nur Nahfeld.

### Abnahme

| | Ergebnis |
|---|---|
| **A — Schalter AUS gegen `uv8_nach2`** (anderes Binary, Zeile wortgleich) | **28 von 28 bitgleich** |
| **B — Schalter AN gegen Schalter AUS** | **28 von 28 bitgleich** |
| Wirkpfad | Ansage + Ist=Soll („Takt 100 feine Schritte = Sample-Kadenz 25 grob × ratio 4") nur im AN-Arm, im AUS-Arm null Treffer |

Abnahme B ist die eigentliche: rho wird für **99,88 % der Zellen an 99 von 100 Schritten nicht
mehr geschrieben**, und keine einzige der 21 CSV-Dateien und 7 Feld-Dumps ändert ein Byte.

### Wanduhr

| Arm | Wanduhr |
|---|---:|
| uv8_vor (aus) | 428 s |
| uv8_nach (aus) | 413 s |
| uv8_nach2 (aus) | 402 s |
| rs8_aus | 404 s |
| **rs8_an** | **387 s** |

**−3,7 % gegen den besten AUS-Arm, −4,8 % gegen das Mittel der drei jungen.** Der AN-Arm liegt
**unter allen vier** AUS-Armen. Vorhergesagt war −3,5 % (rho trägt 4,2 von 16,8 Verkehrsprozent
bei einer gemessenen Obergrenze von 14,4 %). Die Vorhersage stand im Serienkopf **vor** dem Lauf.

**Ehrlich dazu:** die Streuung bitgleicher Arme beträgt 26 s über vier Läufe (6,1 %), beim
kalten ersten Lauf. Über die drei jungen sind es 11 s (2,7 %). Der Gewinn liegt damit knapp,
aber sichtbar über der Streuung — und die Ordnung (AN unter allen AUS) ist das stärkere Argument.

### Vier Wächter, die mitgebaut wurden

1. Ohne das JIT-Define `RHO_SPARSAM` ist der Gerätecode an der Schreibstelle **zeichengleich**
   zu vorher — belegt durch Abnahme A über zwei Binaries.
2. **Ist=Soll** auf den Takt gegen die tatsächliche Sample-Kadenz, sonst `print_error`.
3. **`CFD_FAC_APG` schliesst sich aus** — der Zweig liest `rho` an bis zu 18 Facettennachbarn,
   ausserhalb der Maske. `print_error` statt stiller Verfälschung.
4. Ab der vorletzten Sample-Periode **erzwingt der Host wieder Vollschreiben**, weil der letzte
   Zeitschritt kein Vielfaches der Kadenz ist (25 050 mod 100 = 50).

### Als Nächstes

Schritt 2 (Fernfeldmaske, auf dem kritischen Pfad), Schritt 3 (u-Teil, die grosse Hälfte),
Schritt 4 (2 Byte, der VRAM-Hebel). Der Deckel ist nach Schritt 1 neu zu bestimmen.
