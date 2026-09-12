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

---

## 9 · Schritt 4 ist gebaut (12.09.2026) — rho auf 2 Byte

**Schalter:** `RHO_FP16` in `src/defines.hpp`, Vorgabe AUS. Umschalten mit
`werkzeuge/rho_format.sh FP32|FP16`. Kein env-Schalter, weil der Puffertyp ein C++-Typ ist
(`Memory<rhoxx>`) und zur Laufzeit feststeht — dieselbe Bauform wie `FP16S` für die Verteilungen.

### 9.1 Was der Schritt bringt, und was nicht

Die Übergabe nennt 1378 MiB für rho. Das ist die **Summe beider Domänen**. Nur das Nahfeld liegt
auf der B70; das Fernfeld rechnet auf der iGPU im System-RAM und hat dort keinen Deckel
(`logs/p4_neu.log`: „Fernfeld belegt 10676 MB (System-RAM, kein VRAM-Deckel)").

| | Zellen | rho 4→2 B | davon VRAM |
|---|---:|---:|---:|
| Nahfeld (B70) | 519 139 485 | 990 MiB | **990 MiB** |
| Fernfeld (iGPU) | 203 489 280 | 388 MiB | 0 |

Dazu kommen im Nahfeld noch einmal 990 MiB **System-RAM**: die B70 ist keine Zero-Copy-Karte
(`src/opencl.hpp`), rho liegt dort zweimal. Im Fernfeld ist der Puffer Zero-Copy und zählt einfach.

**Und die Laufzeit? Gemessen, 8-mm-Fahrzeug, fünf Arme, Zeile wortgleich:**

| Arm | Wanduhr | gegen | |
|---|---:|---:|---|
| `r8_vor` | 397 s | | Stand vor der Änderung |
| `r8_fp32` | 399 s | | bitgleich dazu |
| `r8_fp16b` | **395 s** | −1,0 % | nur Schritt 4 |
| `s8_sfp32` | **369 s** | −7,5 % | nur Schritt 1 bis 3 |
| `s8_sfp16` | **364 s** | −1,4 % | Schritt 1 bis 3 **und** 4 |

**Schritt 4 gewinnt, er verliert nichts** — in beiden Zusammenhängen rund ein Prozent, zusammen
mit den Sparschaltern −8,8 % gegen den Ausgangsstand. Vorhergesagt war −1,4 % aus dem
Verkehrsanteil (rho sind 4 von 123 B je Zelle und Schritt), gemessen −1,0 % ohne und −1,4 % mit
Sparschaltern.

**Warum es nur ein Prozent ist, und warum das kein Mangel ist:** Schritt 1 bis 3 und Schritt 4
greifen **dieselben Bytes** an. Die Schritte 1 bis 3 nehmen die rho- und u-Schreibvorgänge weg,
Schritt 4 halbiert, was davon übrig ist. Was Schritt 1 schon entfernt hat, kann Schritt 4 nicht
noch einmal einsparen — die Hebel überlappen, und genau das ist der Beweis, dass Schritt 1
funktioniert hat. Die 19 Verteilungen sind 76 der 123 Byte und liegen längst auf FP16S; rho's
Anteil ist strukturell klein.

**Ehrlich zur Streuung:** die −1,4 % mit Sparschaltern sind fünf Sekunden und liegen innerhalb
der für dieses Projekt dokumentierten Streuung gleicher Arme. Ein einzelnes Paar kann einen
echten Zweiteffekt aus dem kleineren Speicherabdruck nicht von Rauschen trennen. Belastbar ist:
**kein Verlust, eher ein kleiner Gewinn.** Der eigentliche Ertrag ist die Kapazität, und die
können die Schritte 1 bis 3 überhaupt nicht liefern: Bandbreite und Platz sind zwei Größen, und
nur eine davon gewinnt man durch Weglassen von Schreibvorgängen.

### 9.2 Das Format, am echten Feld belegt statt behauptet

Gerechnet auf `export/p4_neu/feld_nah_000501ms.vtk` (451 428 942 Fluidzellen) und
`feld_fern_000501ms.vtk` (199 949 922):

| Feld | Format | Fehler RMS | Fehler max |
|---|---|---:|---:|
| Nahfeld | `FP16S(rho−1)` | 3,320e-7 | 5,633e-5 |
| Nahfeld | `half(rho)` roh | 1,824e-4 | 4,883e-4 |
| Fernfeld | `FP16S(rho−1)` | 3,183e-7 | 3,052e-5 |

Die Verschiebung um 1 ist damit **Faktor 550 im RMS**, kein Stil. rho spannt im Nahfeld
0,769797 bis 1,111411 bei einer Eigenstreuung von 1,420e-3; der Quantisierungsfehler liegt
4280-fach darunter. Kein Überlauf (die Skalierung trägt bis |rho−1| = 1,999, `RHO_CLAMP`
garantiert 0,5 und das Tor im Kopplungs-Lift 1,0), keine Denormalzelle in beiden Domänen.

**Ehrlich dazu, weil es sonst als Gewinn gelesen wird:** gegen den heutigen float32-Stand ist das
ein **Verlust**. float32 trägt bei rho nahe 1 einen absoluten Boden von 5,96e-8; `FP16S(rho−1)`
trägt |rho−1|·2⁻¹², bei rho−1 = 1e-3 also 2,4e-7. Der Trick macht half überhaupt erst brauchbar —
mehr nicht.

### 9.3 Warum die Kette nicht driftet

`3.0517578E-5f` ist **bitgenau 2⁻¹⁵** und `32768.0f` ist 2¹⁵; beide Multiplikationen runden also
nicht. Zusammen mit der Sterbenz-Exaktheit von `(x)-1.0f` auf [0,5; 2,0] ist die Kette
Laden→Speichern ein **Fixpunkt**: `rho_unpack(rho_pack(rho_unpack(h))) == rho_unpack(h)`,
bitgleich als float32, über alle 59 394 Bitmuster der Klemmspanne und auch nach acht Umläufen
(mit den repo-eigenen Wandlern nachgerechnet). Daran hängen zwei Dinge, die sonst still brächen:
`pruefe_slice_ebene` behält sein „Soll exakt 0", und `apply_velocity_inlet`, das nichts als
`rho[n] = rho[m]` tut, driftet nicht. Es macht die Makros außerdem unempfindlich gegen
`-cl-mad-enable`: eine Kontraktion zu `mad()` kann nichts ändern, wo nichts zu runden ist.

**Zwei Lademakros, und das ist kein Luxus.** `load_rho` liefert rho, `load_drho` liefert rho−1
ohne den Umweg über „+1, dann −1". Wer rho−1 braucht und trotzdem `load_rho` nimmt, rundet auf das
float32-Raster bei 1,0 und baut damit den Boden von 5,96e-8 ein — genau gegen den der Kommentar an
`po_reduce_mean` seine 1e-9 beansprucht. Dort steht jetzt `load_drho`. An
`apply_pressure_outlet` steht bewusst `load_rho`: in Abweichungsräumen zu rechnen wäre genauer,
würde aber die Arithmetik des Arms OHNE `RHO_FP16` ändern, und dessen Bitgleichheit ist das
einzige Sicherheitsnetz dieses Umbaus.

### 9.4 Die Wächter

| Wächter | Ort | Soll |
|---|---|---|
| `Rho_Feld::get/set` statt `operator[]` | `src/lbm.hpp` | jede vergessene Hostzugriffsstelle ist ein **Übersetzungsfehler**, kein Prüfpunkt |
| Typ-Zensus auf dem emittierten OpenCL-Quelltext | `src/lbm.cpp` | 18 × `global float* rho` (alle SURFACE/GRAPHICS), 14 × `global rhoxx* rho` |
| Slot 210 — rho außerhalb 0,25..4,0 an der TYPE_E-Lesestelle | `src/kernel.cpp` | **0**, ungegatet |
| Slot 211 — Besuche derselben Stelle | `src/kernel.cpp` | **> 0**, sonst beweist die Null in 210 nichts |
| Slots 212..217 — Dekaden von \|load_rho(store_rho(x))−x\| | `store_rho_diag` | **217 == 0** |
| `#error` bei `RHO_FP16` × SURFACE/GRAPHICS/TEMPERATURE/PARTICLES | `src/defines.hpp` | deren rho-Leser sind nicht umgestellt |
| `bytes_per_cell_device/host`, Bandbreitenbilanz | `src/lbm.cpp` | folgen `sizeof(rhoxx)` |

Der Typ-Zensus ist der wichtigste davon. `Kernel::link_parameter` reicht nur die `cl::Buffer`
weiter, der Typ ist dort **vollständig gelöscht**: ein vergessenes `global float* rho` läse zwei
halbe Dichten als einen float, Größenordnung 1e38, und unter `-cl-finite-math-only` gäbe es dafür
keine Diagnose. Weil `get_opencl_c_code()` alle Leerzeichen durch Zeilenumbrüche ersetzt, zählt
der Wächter auf die **umgebrochene** Form — wer das übersieht, baut sich einen Wächter, der immer
0 findet. Genau das ist beim ersten Anlauf passiert und wurde gefangen.

**Die Kopplungsprüfung musste mitwandern.** `src/setup.cpp` vergleicht das Nahfeld an den
Deckungspunkten gegen die grobe Ebene mit der festen Schranke 1e-6. Die Quantisierung liegt bei
5,6e-5, also **56-fach darüber**. Ohne Anpassung hätte die Abnahme in jedem Lauf einen
Kopplungsdefekt gemeldet, den es nicht gibt. Die Schranke folgt jetzt dem Speicherformat; u
behält im selben `fmax` seine scharfe 1e-6.

### 9.5 Zwei Fehler dieser Runde, beide von der Leiter gefangen

1. **Namenszusammenstoß.** Das Typmakro hieß zuerst `rho_t`. `src/kernel.cpp` führt eine lokale
   Variable `float rho_t` in der Schale-Blend-Prüfung; das Makro hat sie in **beiden** Armen
   überschrieben, der Geräteübersetzer brach mit −11 ab. Gefangen auf der **CPU**-Sprosse, nicht
   auf der B70. Der Name ist jetzt `rhoxx`, wie auf der Hostseite.
2. **Der Wächter, der sich selbst blind gemacht hätte.** Beim Umbenennen traf die
   Wortgrenzen-Ersetzung das Suchmuster `"global\nrho_t*\nrho"` nicht, weil vor dem `r` das `n`
   aus `\n` steht. Der Zensus hätte danach **immer 0 gefunden** und in jedem Lauf falsch Alarm
   geschlagen.

### 9.6 Abnahme

**CPU, Kugel dx = 40, 500 Schritte, drei Arme in einer Kette (Stand davor / FP32 / FP16):**

* Stand davor gegen FP32-Arm: **23 von 23 inhaltlichen Dateien bitgleich** (einzige Abweichung ist
  der Laufname in `code/LAUF.txt`). Das Sicherheitsnetz steht.
* FP16-Arm: 0 Fehler. Slot 210/211: 5494 TYPE_E-Lesungen geprüft, **0** außerhalb 0,25..4,0.
  Quantisierung über 3 698 000 Schreibvorgänge: <1e-7 28,24 % | <1e-6 56,87 % | <1e-5 14,85 % |
  <1e-4 0,04 % | <1e-3 0,00 % | **≥1e-3 0,00 %**.
* Von 24 Ausgabedateien weicht zwischen FP32 und FP16 genau **eine** ab: `forces.csv`.
* **Mechanisch bestätigt:** bei 1 ms und 6 ms sind die Kräfte **exakt gleich**, erst ab 11 ms
  weichen sie ab. Die Kräfte kommen aus `F` (`update_force_field` liest `fi`, nicht rho); die
  Quantisierung erreicht Cd **nur über die Randbedingungen** und braucht dafür rund hundert
  Schritte Laufzeit durch die Domäne.

**Offline-Gate (`werkzeuge/scratch_gate`), erweitert um den rho-Arm:** 8 Arme × 2 Geräte
(iGPU 0x7d67, B70 0xe223), je 37 Kernel, `private_size` 0 und `spill_size` 0 durchweg. Der
Rückleser in `store_rho_diag` kostet also kein Register. Die Zwillingsliste in `gen_main.cpp`
kannte die neuen Makros nicht und meldete BAUFEHLER — nachgezogen.

**8 mm Fahrzeug, drei Arme:** läuft (Stand 12.09. nachmittags).

---

## 10 · u auf 2 Byte — gemessen, und es ist NICHT dieselbe Rechnung wie bei rho

u trägt im Nahfeld **2971 MiB VRAM** gegen 990 MiB bei rho, ist also der eigentliche Hebel.
Die Übergabe begründet die Reihenfolge mit dem Aufwand (6 gegen 125 Hostzugriffe). Es gibt einen
**numerischen** Grund, und der ist härter.

### 10.1 Im Feld ist u unauffällig

`FP16S(u)` über alle 451 428 942 Fluidzellen des 4-mm-Nahfelds bei 501 ms:

| | Wert |
|---|---|
| Fehler RMS | 9,079e-6 lat = **0,0121 % von u_inf** |
| Fehler max | 1,188e-4 lat |
| Überlauf / Denormal | 0 / 28 von 1,354 Mrd. Komponenten |

Kein Skalierungsproblem: |u| erreicht 0,4764 lat, die Skalierung trägt bis 1,999.

### 10.2 Am Gradienten ist es das nicht mehr

Zentraldifferenz `du_x/dx` über die ganze Box (1 686 924 Fluidtripel, jede 16. Zeile):

| | Wert |
|---|---:|
| RMS-Gradient | 1,129e-3 |
| Fehler RMS | 1,092e-5 = **0,97 %** |
| Median-Gradient | 1,070e-4 → die Quantisierung trägt dort **10,2 %** |

### 10.3 Und am regularisierten Rand ist es ein Befund

`deriv_reg` (`src/kernel.cpp`) an den TYPE_E-Zellen nachgerechnet, Gate genau wie im Code
(zentral bei zwei Fluidnachbarn, sonst einseitig, sonst null), jede 3. Zeile:

| | Wert |
|---|---:|
| TYPE_E-Zellen gesehen | 585 816 |
| davon mit mindestens einem Fluidnachbarn in x | 67 605 |
| \|du/dx\| RMS dort | 9,526e-5 |
| Fehler RMS | 2,331e-5 = **24,5 % des RMS-Gradienten**, 42,5 % des Medians |

**Der Mechanismus ist einfach und unangenehm:** am Einlassrand ist |u| mit ~0,075 der größte
zusammenhängende Wert im ganzen Feld und der Gradient der kleinste. Der half-ULP bei 0,075 ist
3,66e-5; die Rundung trägt dort absolut mehr als im Mittelfeld, während der Nenner sein Minimum
hat. Die schlechteste denkbare Paarung, und sie sitzt genau dort, wo `deriv_reg` den
Nichtgleichgewichtsanteil des Kopplungsrandes aufbaut.

**Vorbehalt, der dazugehört:** gemessen ist nur die x-Ableitung. `deriv_reg` baut S aus allen drei
Richtungen; die y- und z-Ableitungen entlang der Schale sind nicht vermessen. Und: 518 210 der
585 816 TYPE_E-Zellen bekommen in x ohnehin den Gradienten null, betroffen sind also 12 %.

**Folgerung.** u auf 2 Byte ist nicht ausgeschlossen, aber es ist kein Nachziehen von rho. Es
braucht eine eigene Abnahme mit Blick auf den regularisierten Rand und, wenn der Befund trägt,
eine Gegenmaßnahme — die naheliegende wäre, die Randschale der Dicke 2 in float32 zu halten
(rund 8,8 Mio Zellen, 1,7 % des Nahfelds, 101 MiB), also genau die Menge, die `felder_voll` unter
`U_SPARSAM` ohnehin schon als eigene Maske führt.

### 10.4 Was rho allein einlöst

Der Planungsagent hat vorgerechnet, dass rho allein die Gittersprosse nicht kauft: 990 MiB reichen
für 3,90 mm (+7,9 % Zellen), erst rho **und** u zusammen reichen für 3,75 mm. Das stimmt — aber es
ist nicht der einzige Verwendungszweck. Der offene Punkt „verbreiterte Nahfeldbox bei 4 mm" ist
am 29.08. an **516 MB** gescheitert. Mit 990 MiB passt sie, und es bleiben rund 470 MB übrig.
Welcher der beiden Wege genommen wird, ist eine Entscheidung für Heiko, keine für mich.
