# Wandzell-Rekonstruktion — Plan, Fassung 2

**Stand 21.09.2026 abends, Commit a683959. Zwei unabhängige Prüfungen eingearbeitet
(Physik/Verfahren und Umsetzung/Messbarkeit). Fassung 1 trug drei falsche Belege — siehe §0.**

Heiko-Entscheid 21.09.: dieser Weg wird geplant. D3Q27 ist geschlossen.

---

## 0 · Was in Fassung 1 falsch war (zuerst, damit es niemand erbt)

| Behauptung Fassung 1 | Wirklichkeit | Beleg |
|---|---|---|
| Korb zitiere Haussmann: Treppenwand, Druckfehler **zwei Größenordnungen** besser als No-Slip | Korb schreibt nur: *„unless the interpolated bounce-back is applied, all walls were approximated with a staircase approximation. Nevertheless, the wall model yields generally good agreement with the measured pressure drop"*. **Kein No-Slip-Vergleich, keine Größenordnung.** „orders of magnitude" steht in Korb nur einmal — über *energy to solution* auf GPUs. | `logs/literatur/korb.txt:859-868`, `:1214` |
| Die Formel sei Malaspinas | Sie ist **Haussmann 2020**. Malaspinas baut f_neq **aus dem Ziel-τ_w neu auf**, übernimmt es nicht — *„requires an estimation of the turbulent viscosity near the wall for the reconstruction of the non-equilibrium parts"* | `korb.txt:400-406` (Haussmann), `:436-443` (Malaspinas) |
| „vor `store_f` wird nichts geschrieben" | Gilt **innerhalb `stream_collide`**, nicht für den Zeitschritt: danach schreiben `schale_blend`, `boden_eq`, `einlass_eq` und `transfer__insert_fi` noch in `fi` | `lbm.cpp:3217-3262`, Kopplungsnotiz `:3244-3258` |
| `Σ Δm == 0` als Leck-Instrument | **identisch null per Konstruktion** (Σf_eq = ρ in beiden Termen) — es kann nicht ausschlagen | nachgerechnet |
| Der Zirkel gehöre SISM | Er gehört **`CFD_SGS_FDWAND`**: nu_t kommt aus \|S\|_FD des u-Felds. Unter Rekonstruktion ist dieses u gesetzt — der Zirkel besteht schon ohne SISM | `kernel.cpp:5473-5507` |
| `CFD_FAC_KRAFT=1` sei ein „Doppelkanal" und damit erledigt | Es ist **Kuwata & Sugas IVW**, eine von Korbs vier Klassen: `R = Ziel − P`, nicht „der ganze Tangentialimpuls". Die echten Einwände (zweites Moment, Populationsstruktur, Buchhaltung) sind **dieselben**, die §4 gegen den eigenen Vorschlag erhebt. ~~Am Fahrzeug **nie gemessen**.~~ — **dieser Satz ist selbst falsch, siehe §0b** | `kernel.cpp:2592`, `korb.txt:432-435`, `logs/herleiter_2026-09-06.md:187` |

**Folge: es gibt für Rekonstruktion an einer Voxeltreppe keinen publizierten Zahlenwert** — weder für
Reibung noch für Druck. `AUDIT-BEFUNDE.md:2626-2631` sagt es selbst: *„Eine systematische Konvergenzstudie
‚WMLES-Reibung auf Voxeltreppe' existiert nicht (Literaturlücke)."*

---

## 0b · Was in Fassung 2 falsch war (22.09.2026)

**`CFD_FAC_KRAFT=1` ist am Fahrzeug gemessen.** Der Satz „am Fahrzeug nie gemessen" in der letzten
Zeile von §0 ist falsch, und er hat Schritt 1 in §11 erzeugt.

Gepaart, identisches Binär, eine Variable: `export/w_ref` gegen `export/w_kraft1`, Commit `53d6c65`,
30.08.2026, 8 mm `fahrzeug_dd`, 151 Messpunkte ab t ≥ 0,2 s, nachgerechnet aus `cd_facetten.csv`:

| Größe | `w_ref` | `w_kraft1` | Δ ± SEM | σ | Urteil |
|---|---|---|---|---|---|
| `cd_druck_rest` | 1,0568 | 1,2880 | **+0,2313 ± 0,0102** (gepaart, r=0,497) | 22,7 | unterscheidbar |
| `cz_druck_rest` | −0,1199 | −0,0906 | +0,0293 ± 0,0275 (ungepaart, r=0,176) | 1,06 | **nicht von Rauschen zu trennen** |

Der Speicher führt das als `messung_kraftleiter` (provenance `messung`) mit dem Verdikt **„Weg F ist
ein validiertes NEGATIV"**: das Modellziel an Rückfallzellen exakt durchzusetzen *erhöht* die
Reibung, reines Bounce-Back lag näher am Ziel als das Modell.

**Zwei Vorbehalte, die die Zahl einordnen, aber nicht entwerten:**

1. Beide Arme fuhren `CFD_KRAFT_ZBAND=2` und tragen die Deckellage im Rest
   (`BAND-ARTEFAKT-8MM.md:52`). Wieviel der Armdifferenz Keil-/Deckelkraft ist, ist **ungeklärt** —
   derselbe Vorbehalt, den das Dokument für den D3Q27-A/B notiert.
2. Zwischen `w_ref` und heute liegen APG, DETEPS, NACHBAR (ersetzt das damalige
   `CFD_FAC_UTKORR=1.5`), PINV, P-TRT 1,90, POSITIV=2, U_KLEMME=1, SISM, `CFD_Y_VERSATZ=1` und die
   korrigierte ZBAND-Regel. Der Befund gilt für den Stand vom 30.08.

**Der Arm, der wirklich offen ist:** `CFD_FAC_KRAFT` hat zwei Stufen (`lbm.hpp:403`) — **1 = an
Rückfallzellen, 2 = an allen Facettenzellen**. Zensus über alle 779 `export/*/code/LAUF.txt`: Stufe 2
existiert **nur am Kanal** (`x0_kipp0_kraft2`, `x26_kraft2`, `x45_kraft2`, `jit_kipp26_kraft2`,
`jit_igpu_kipp26_kraft2`, `vt_kdiag_kraft2`), am Fahrzeug nie. Da §3.1 den Umfang der Rekonstruktion
auf `Rang≤1 / Rang≤2 / **alle**` schaltbar auslegt, ist **KRAFT=2 die passende Obergrenze für den
Umfang „alle"** — KRAFT=1 ist es nur für `Rang≤1`.

---

## 1 · Warum überhaupt — und der Präzedenzfall, der dagegen spricht

**Die Diagnose bei 26° ist richtig und publiziert.** Asmuth et al. 2021 (`logs/literatur/o3j79i.txt:391`,
Block Gl. 23–28): das System für u_w bleibt nur bestimmt, *„as long as F^(u_w) can be constructed as a
linear combination of e_ijk, ijk in C"*. Das ist die Bildraum-Bedingung an unseren Gt-Tensor. Zwei
Präzisierungen: Asmuths Gl. 28 gilt für die **ebene** Wand n = e_z **und D3Q27**, wir rechnen D3Q19.

**Bei 45° trägt die Diagnose NICHT.** `K2-LOESUNGSENTSCHEID.md:16-22`: *„Kanal 45°: KEIN Autoritäts-/
Flächenproblem — die lösbare m4-Klasse trägt die wahre Fläche EXAKT (Σfaca = 1,0000·A_wahr), nötiges τ_w
nur 1,08·u_τ². Das Defizit (Faktor 29) ist ein ZUSTANDSPROBLEM."*

**Und die Richtung stimmt anders, als „3,4 % / 0,7 % der Sollreibung" klingt:** gemessen ist am kipp26
c_f = **2,227 × Referenz**, also **zu viel** Reibung (Treppenrauheit), während das Wandmodell 0,7 % seines
Ziels liefert. `WANDMODELL.md:244` formuliert die Aufgabe richtig: *„die Rauwand-Charakteristik durch die
korrekte Glattwand-Spannung ersetzen"* — nicht „fehlende Reibung ergänzen".

### Der gemessene Präzedenzfall im eigenen Haus — das ist der zentrale Vorbehalt

`WANDMODELL.md:288-305`, WFB = Spalding im Bounce-Back am **ebenen** Kanal:

| N | c_f mit WFB | vorher (Rauwand) |
|---|---:|---:|
| 38 | 0,00107 | 0,01062 |
| 54 | 0,00110 | 0,01121 |
| 76 | 0,00109 | 0,00772 |
| 108 | 0,00114 | 0,00761 |

Flach über alle Gitter, Rauwand beseitigt — **aber flach bei −68 %** (Referenz 0,00344). Die Deutung dort
ist wörtlich der Mechanismus, den auch die Rekonstruktion hat:

> *„selbstkonsistenter turbulenzfreier Zustand — ohne aufgelöste Turbulenz transportiert nichts Impuls zur
> Wand, die Wandfunktion sieht zu langsames u_t und liefert die dazu passende, zu kleine Spannung.
> **Das Restproblem liegt im Inneren.**"* (y⁺₁ = 137–500 bei uns; Han et al. liefen bei 16–50, wo das
> Gitter Wirbel trägt.)

**Jede Randbedingung, die u an einem Punkt liest und u zurückschreibt, schließt diese Schleife.** Die
Erwartung „die Rekonstruktion erzwingt das Log-Profil" hat damit einen gemessenen, negativen Vorläufer.
Das ist kein Ausschlussgrund, aber es ist der Satz, der in jedem Zwischenbericht stehen muss.

### Fehlerbudget — was überhaupt zu holen ist

Am Fahrzeug ist die Lücke zu 95,6 % Druck; das gesamte physikalische Wandschubbudget ist
C_|τ| = 0,0181 = **4,4 % der Lücke** (Sechs-Agenten-Audit 29.08., 4 mm). Und τ_w als Hebel auf den
Druckwiderstand ist **gemessen widerlegt**: `PLAN-APG-2026-09-16.md` §I, 4 mm, cd_druck_rest
−0,0022 ± 0,0011 = Rauschen, Fazit dort wörtlich *„Eine Wiederaufnahme müsste zuerst zeigen, dass τ_w
überhaupt ein Hebel auf den Druckwiderstand ist — bei 8 mm und 4 mm ist er es nicht."*

**Fair dagegen:** die Rekonstruktion ändert das **u-Feld** in Lage 1, nicht nur das τ_w-Ziel, und dieser
Pfad hat den Druck schon einmal bewegt (NACHBAR: cd_druck_rest −2,7 %, Ablöseort 3,45 → 3,61 m,
δ₉₀ 140 → 132 mm; Ziel 41 mm **nicht** erreicht).

**Ehrliche Erwartung: einstellige Prozente auf den Druckanteil — die Größe der bestehenden Fehlerbalken.**

---

## 2 · Zwei Varianten, nicht eine

```
(A) HAUSSMANN   f_rek,i = f_eq,i(ρ_lok, u_rek) + [ f_load,i − f_eq,i(ρ_lok, u_lok) ]
                f_neq unverändert übernommen

(B) MALASPINAS  f_rek,i = f_eq,i(ρ_lok, u_rek) + f_neq,i(Π_neq aus ZIEL-τ_w)
                f_neq regularisiert neu aufgebaut
```

**Momente, nachgerechnet (D3Q19, Residuen ≤ 5,6e-17):**

| | Variante A |
|---|---|
| Σ f_rek | = ρ_lok **exakt** — Masse erhalten, Δm ≡ 0 |
| Σ c_i f_rek | = ρ·u_rek **exakt** — ΔP = ρ(u_rek − u_lok) in geschlossener Form |
| Σ c_i c_i f_rek | = Π_load + ρ(u_rek u_rek − u_lok u_lok) — **Π_neq unverändert** |

**Daraus der entscheidende Satz, den Fassung 1 nicht zog: Variante A prägt keine Spannung auf.** Der
wandnormale Impulsfluss ändert sich nur über ρ·u_t·u_n, und an einer Wandzelle ist u_n ≈ 0. Die Wirkung
läuft ausschließlich über den Gradienten, den Lage 2 im **nächsten** Schritt sieht. Genau deshalb baut
Malaspinas f_neq neu auf.

**Und A trägt eine messbar falsche Spannung weiter:** an kipp0 ist |S|_Pi/|S|_FD unter iMEM **3,39**, unter
reinem BB **1,47** (`setup.cpp:4413` nennt als Baugrund von `CFD_SGS_FDWAND` „B66/B69: Pi/FD 2,3-3,4").
Unter Rekonstruktion fällt die iMEM-Kontamination weg, der BB-Anteil (~1,5×) bleibt — und wird unverändert
in den neu gesetzten Zellzustand übernommen.

**B ist zudem der kleinere Eingriff:** kein Image Point, kein u-Interpolator, kein trilinearer Stern — also
auch **keine Kollision mit `CFD_U_SPARSAM` und `SPARSE_TILES`**, die §3.3/§8 als Hauptrisiken nennen. Und
B nimmt dem FDWAND-Zirkel den Spannungskanal weg, weil die Spannung dann aus der Wandfunktion kommt statt
aus gemessenem u.

**Entscheid: B ist gleichrangiger Arm ab S2, nicht Fußnote.** Offen: FP16S-Rauschlast auf f_neq (f_neq ist
eine kleine Differenz zweier O(w_i)-Zahlen; die ~4,9e-4 landen vollständig auf dem spannungstragenden
Anteil). Billig messbar als Histogramm |f_neq|/f_eq an Facettenzellen — **vor** S2 erledigen.

### Präzedenzfall `schale_blend` — trägt die Form, nicht den Beweis

`schale_blend` (`kernel.cpp:5019-5115`) setzt alle 19 Populationen in genau der A-Form neu. **Aber** seine
`ftrue`-Permutation (`:5057-5058`) und die u-Negation (`:5049`) existieren nur, weil es *post-stream* mit
vertauschten EsoPull-Paaren lädt. Nach `load_f` in `stream_collide` gilt das nicht — **die Permutation
mitzukopieren wäre ein Vorzeichenfehler**. Auch der Momentenbeweis (Slots 25/26, `:5074-5085`) hat am neuen
Ort kein Gegenstück. Übertragbar ist allein der **relative** Rundungszähler (Slots 23/24, `:5102-5116`).

---

## 3 · Architektur

### 3.1 Welche Zellen — statische Menge, schaltbarer Umfang

Der Rückfall flackert je Schritt (`kernel.cpp:2405-2411, 2465-2468, 2476/2483/2489`, alle am Laufzeit-u).
Die **Flags sind statisch**, und der statische Klassenzensus existiert (`zensus_statische_klassen`,
`setup.cpp:2748`; im Werkzeug `werkzeuge/d3q27_zensus.py` als Abnahme B3 exakt nachgebaut).

Die Menge wird **einmal beim Facettenbau** festgelegt, Marke in `fac_geo[8i+7]` (frei, `lbm.hpp:468`,
heute 0.0f geschrieben `lbm.cpp:1542`, kein Leser im Baum). Schalter zieht den Umfang auf:
`Rang≤1` / `Rang≤2` / `alle`.

**Falle:** `zensus_statische_klassen` läuft an jeder Aufrufstelle **nach** `alloc_facetten`
(`setup.cpp:4525` vs `4527`, `7584` vs `7594`), und dort ist `fac_geo` schon hochgeladen (`lbm.cpp:1673`).
Die Klassifikation muss **vor** `alloc_facetten` in den `Facette`-Vektor, oder es braucht einen zweiten
Upload.

**Wächter, der bei Umfang „alle" zwingend ist:** die `fac_nb`-Abtastung (2. Fluidzelle entlang der Normale,
`kernel.cpp:5521-5566`) kann dann **in die gesetzte Menge fallen** — das Modell liest seine eigene Ausgabe.
`print_error`, keine Fußnote.

**Bitgleichheit:** nur über den AUS-Schalter. Schalter an ist auch an kipp0 nicht bitgleich — kipp0 hat am
Startschritt an **allen** Facettenzellen Rückfall (`kernel.cpp:2596`, Slot 71). Genau daran scheiterte die
Bitgleichheits-Behauptung von `CFD_FAC_KRAFT=1` (`kernel.cpp:2521`). **Der kipp0-Bit-Anker geht verloren**
und wird ersetzt durch: c_f gegen den validierten Ebene-Pfad — **der liegt bei 85 % von Lee & Moser**
(kipp0 cf_kraftbilanz 2,9149e-3 gegen 3,4424e-3, NACHBAR-Leiter 03.09., `d38acfe`). **Diese Zahl gehört ins
Abnahmekriterium**, sonst importiert „validiert" still 15 % Versatz.

### 3.2 Einbauort

**In `apply_facette_imem`** (`kernel.cpp:1996-2900`), also innerhalb der bereits bewachten Region — hinter
`flagsn_bo != TYPE_S/E/MS`, `f_bbox` und `fac_fid` (`:2021-2022`, Aufruf `:3031`), und **hinter** dem
MESSNUR-Ausstieg (`:2029-2032`), sonst ist `CFD_FAC_MESSNUR` still kein reines Bounce-Back mehr. Damit
verlängert die Rekonstruktion einen vorhandenen divergenten Körper, statt neue Divergenz zu öffnen.

`load_f` (`:3004`) und `store_f` (`:4129`) treffen dieselben 19 Adressen mit vertauschten Paarinhalten
(`:1501-1538`) — nachgerechnet, stimmt. Innerhalb `stream_collide` schreibt nichts in `fi`.

**ABER — und das ist der Befund, der Fassung 1 zerlegt hat:** im selben Zeitschritt schreiben nach
`stream_collide` noch **vier** Kernel in `fi` (`lbm.cpp:3217-3262`): `schale_blend`, `boden_eq`,
`einlass_eq`, `transfer__insert_fi`.

> **`boden_eq` setzt auf allen Fluidzellen z = 1..nz_eff alle 19 auf f_eq(ρ_lok, u_road, 0, 0)**
> (`kernel.cpp:4348-4349`) — und zerstört wegen Esoteric-Pull zusätzlich die Hälfte der Populationen, die
> eine *benachbarte* Facettenzelle gerade dorthin geschrieben hat. `lbm.cpp:3244-3258` sagt es wörtlich:
> *„Wer zuerst schreibt, bestimmt, was der andere liest."*

Am Fahrzeug ist `CFD_BODEN_EQ` gesetzt (Basis). Mit `CFD_BODEN_EQ_ABSTAND=0` würde die Rekonstruktion an
Unterboden-/Fahrbahnfacetten **verworfen, nachdem ihr ΔP schon gebucht ist** → Phantomreibung. **Der Kanal
kennt `boden_eq` nicht — der Fehler taucht erst in S5 auf.** Braucht einen Konstruktor-Wächter (Muster
`lbm.cpp:180`) oder eine dokumentierte, bezifferte Ausnahme.

**Verworfen bleibt:** eigener Kernel nach `stream_collide` — das gesetzte f_neq würde im selben Schritt
nicht mehr relaxiert.

**ELIBB ist keine freie Wahl** (Fassung 1 hatte das als offene Frage): `elibb_rekonstruiere` läuft
**innerhalb** `apply_facette_imem` (`kernel.cpp:2044`) und verändert `fhn`, bevor das Wandmodell abtastet
(Reihenfolge-Fix 25.08., `:2033-2038`). Nimmt die Rekonstruktion ihr `f_load` **danach**, überlebt ELIBB im
f_neq-Anteil; nimmt sie es **davor**, ist ELIBB an diesen Zellen ein No-Op — und das ist am Fahrzeug der
bekannte Faktor-10-Reibungsfehler. **Der Abgriffpunkt muss festgelegt werden; Haussmann macht den
interpolierten Bounce-Back zuerst, die Korrektur danach** (`korb.txt:400-406`). Also: danach.

### 3.3 Determinismus und Reichweite

* **Determinismus:** der Image-Point-Eingang darf nicht im selben Launch aus `u` gelesen werden — am
  03.09. gemessen nicht bitreproduzierbar (`lbm.cpp:1716`: cf 0,00073682648 gegen 0,00073630592 bei
  identischer Konfiguration). Vorkernel + eigener Puffer, Muster `fac_nachbar_ab`/`fac_apg_ab`.
* **Der 1-Schritt-Versatz ist NICHT belanglos.** `RHO_RAND-PLAN.md:425-426`: *„Die Annahme ‚ein Schritt ist
  physikalisch belanglos' ist WIDERLEGT. An Wandzellen gibt es eine Periode-2-Mode, die Rekonstruktion
  trifft die andere Phase."* Für eine Diagnose hinnehmbar, für einen **Aktor in der geschlossenen Schleife**
  nicht. S0/S1 sehen das nicht (No-Ops) — **S2 braucht einen Phasentest**.
* **Reichweite:** unter `CFD_U_SPARSAM` wird u in SMBOX+2 geschrieben, und die Nahfeld-SM-Box **ist** die
  F-BBox (`lbm.cpp:2522`) plus Randschale 2 (`kernel.cpp:3386-3390`). `fac_nachbar_ab` liest genau einen
  Linknachbarn → innerhalb. Ein Stern bei 1,5–2 dx → **außerhalb**. Die Sperre existiert als Muster:
  `CFD_U_SPARSAM × CFD_SGS_BAND → print_error` (`setup.cpp:7058`). Kopieren.
* **Richtung von u_rek steht nirgends.** `fac_nachbar_ab` liefert einen **Skalar** |u_t| plus y
  (`kernel.cpp:5566`), −1 als Sentinel. Die Richtung kommt aus der t1-Basis der Zelle selbst
  (`kernel.cpp:2073`; `lbm.hpp:464`: „die Basis t1 rotiert mit der Stroemung") — also Spalding-**Betrag** ×
  eigene, im Vorschritt schon rekonstruierte **Richtung**. Ein zweiter Zirkel. Muss hingeschrieben und
  bezählt werden.

---

## 4 · Kraftbuchung — der Prüfstein, schärfer als gedacht

**Ausgangslage:** der Cd-Pfad ist zweigeteilt (`FACETTEN-CD-PFAD.md` E1): Druck aus `F` an Solidzellen,
**nur normalprojiziert** an *kontaminierten* Solidzellen; Reibung aus `fac_tau_acc[6·fid+1..3]`
(`kernel.cpp:1854`).

**Befund 1 — der Kontaminationstest hängt an einem Zähler, den Fassung 1 nicht kannte.**
`kernel.cpp:5289`: `if(fid==0xFFFFFFFFu || fac_tau_n[fid]==0u) continue;` (Host-Zwilling
`setup.cpp:4292`). **Nur wo der iMEM-Arm tatsächlich gebucht hat**, wird F auf n̂ projiziert. Ersetzt die
Rekonstruktion den iMEM-Arm und erhöht `fac_tau_cnt` (`kernel.cpp:1856`) nicht, ist `kontaminiert = false`
an jeder Solid-Nachbarzelle → der **volle** F inklusive Tangentialanteil wird als Druck gebucht, und die
Rekonstruktion bucht ihr ΔP_t obendrauf. **Genau die Doppelzählung, die Fassung 1 für unmöglich erklärte.**

**Befund 2 — §4 der Fassung 1 war in sich widersprüchlich.** `update_force_field`
(`F = Σ c_i(2 f_i − 6 w_i (c_i·u_w))`, `kernel.cpp:5167-5175`) wird unter Rekonstruktion **auch normal**
ungültig — Fassung 1 stellte das fest und überließ den Normalanteil dann genau diesem Pfad. Dazu: ΔP lebt
an der **Fluid**zelle, F an der **Solid**zelle, mit unterschiedlichem n̂ (solidseitig ein normiertes Mittel
aus bis zu 18 Nachbarnormalen, `:5289-5296`). „Der Normalanteil steckt schon im F-Pfad" ist **keine belegte
Identität**. Der u_w-Summand wird ebenfalls zur Doppelbuchung.

**Entscheid: normal-neutral von Anfang an bauen** — Normalimpuls der geladenen Populationen exakt erhalten.
Die Projektionsform der Positivitätsklemme zeigt, wie man das erhaltend formuliert (`kernel.cpp:3999`:
*„f** = f* − (1−s)G_i erhaelt Masse und Impuls fuer jedes s"*). Nur dann ist ΔP_t die einzige neue Buchung
und der F-Pfad bleibt gültig.

**Befund 3 — ΔP ist nicht τ_w.** ΔP = ρ(u_rek − u_lok) ist die **Relaxationskraft**, die es kostet, u
festzuhalten. Beide fallen nur im statistisch stationären Zustand zusammen; am Fahrzeug (abgelöst,
instationär) nicht. Braucht einen eigenen Zähler: gebuchtes |ΔP_t| gegen twe je Facette als Histogramm.

**Befund 4 — K2 ist heute eine Identität, danach ein Test.** `setup.cpp:4831` sagt wörtlich: *„Im Solve-Arm
ist die Buchung eine Identitaet …, unter u_w ist sie frei."* Mit ΔP-Buchung wird K2 (`setup.cpp:4837`,
`print_error` = exit(1)) **falsifizierbar** — besser, aber dann kann eine korrekte Rekonstruktion die
Abnahme reißen. Braucht ein deklariertes Band plus eine „was misst K2 jetzt"-Ansage (Muster des
u_w-Warnblocks `:4830-4834`).

**`fac_tau_acc` racefrei bei 1 Zelle = 1 Facette: ja, aktiv bewacht** (`lbm.cpp:1553-1554`). Zwei
Vorbehalte: es sind nicht-atomare globale Read-Modify-Writes — racefrei nur, solange **kein zweiter Kernel**
im selben Schritt in `fac_tau` bucht; und float32-Akkumulation über >2^20 Schritte (Warnung steht schon,
`setup.cpp:4267-4270`) wird mit mehr Akkumulatoren schlechter.

**Kosten der Layout-Erweiterung** (Iron Rule „Sparsam und performant"): je zusätzlichem float × 3,13 Mio
Facetten = **12,5 MB**; 6→10 floats = **+50 MB**; `fac_nb`-Stride 5→8 (u_IP-Vektor + rho) = **+37,6 MB**.
Vollständige Leserliste Stride 6 (überall hart kodiert): `kernel.cpp` 1855, 1981-1982, 2757, 2768-2815;
`lbm.cpp` **1500 (VRAM-Vorschätzung — muss mitziehen)**, 1530, 1543, 2992; `lbm.hpp` 470; `setup.cpp` 602,
4252-4254, 4267-4270, 4292, 4608-4609, 4745, 4772, 5913, 6054, 6064, 9364, 9366, 9381, 10027, 10033,
10090, 10439. Indirekt über `fac_tau_n`: `kernel.cpp:5289`, `setup.cpp:4292`.

---

## 5 · Stufenplan

Bestandsgarantie in jeder Stufe: Schalter aus ⇒ kein Define emittiert (`lbm.cpp:2339ff.`),
`CFD_DUMP_CL`-Diff leer, `FELD-HASH(u)` unverändert (`setup.cpp:4641-4647`).

| Stufe | Inhalt | Abnahmekriterium | Kosten |
|---|---|---|---|
| **S−1** | `hits_n` 320 → 384, Legende an der **einen** Stelle (`lbm.cpp:748`) nachziehen. **Der `scratch_gate.sh`-Arm gehört NICHT hierher, sondern nach S0** — S0/S1 steuert laut derselben Tabelle über einen *Laufzeit*-Parameter, es gibt in S−1 also kein Define, für das ein Arm zu bauen wäre (Prüfagent 22.09., Befund 2). | eigener Commit, bitgleich. **Ohne das schreibt ein Kernel still ins Nichts** — es gibt kein JIT-Define und keine Kernel-Schranke | Host, Minuten |
| **S0+S1** | volle Formel mit `u_rek := u_lok`, gesteuert über einen **Laufzeit**-Kernelparameter (alpha/modus-Muster `lbm.cpp:995`), **nicht** über ein JIT-Define | `f_rek = f_load` wäre `fhn[i]=fhn[i]` — eine Compilezeit-Identität, die IGC entfernt. Der Laufzeitparameter erzwingt, dass der Kernel **läuft**. FELD-HASH kipp0 bitgleich; Wirkpfad **und** Wirkungszähler; relativer Rundungszähler | CPU N=38/316 |
| **S1b** | `u_rek := u_lok + ε·t1`, ε klein und bekannt | ΔP **analytisch bekannt**, Δm exakt 0, K2 gibt den eingespeisten Impuls exakt wieder, ΔP·n̂-Histogramm geeicht. **Ohne diese Stufe sind ein Buchungs-Vorzeichenfehler und ein Wandmodellfehler in S2 nicht unterscheidbar** (FAC_UW-Lehre, `kernel.cpp:2546`: r = −1281) | CPU |
| **S2** | Physik, kipp0. **Zwei Arme: (A) f_neq übernommen, (B) f_neq aus Ziel-τ_w.** Image Point = vorhandene `fac_nb`-Abtastung | c_f gegen **2,9149e-3** (nicht gegen „validiert"); K2 im deklarierten Band; Phasentest gegen die Periode-2-Mode; ΔP_t/twe-Histogramm. **`CFD_SGS_FDWAND` AUS** (nicht nur SISM) | iGPU Kanal |
| **S3** | kipp26, Vorbedingung `CFD_FACETTEN_YWMIN<0,187` (`setup.cpp:3193`) | c_f(26°)/c_f(0°) → 1 (heute Faktor ~2,6 zu schließen). **Gültiges Pflicht-Gate, aber KEIN hinreichender Entscheidungspunkt** — s. u. | iGPU Kanal |
| **S3b** | **zweiter Azimut**: Antrieb in die Hangebene drehen — eine Setup-Variable, kein Kernel | der Kanal treibt heute entlang x (`setup.cpp:4486-4488`), die Kippung liegt in y–z (`:4475-4477`), und t1 folgt der Strömung (`kernel.cpp:2122`). Der Fall „Strömung den Hang hinauf" (`kernel.cpp:2439-2441`) **kann im Kanal nie entstehen** | iGPU Kanal |
| **S4** | echter Image Point + Interpolation — **nur falls S3/S3b zeigen, dass die diskrete Abtastung nicht reicht** | A/A-Bitreproduzierbarkeit; `scratch_gate` 0 Spill; Reichweiten-Wächter | iGPU |
| **S5** | Kugel (Azimut frei, Cd gegen Achenbach), dann Fahrzeug 8 mm, 4 mm nur mit Go. **`boden_eq`-Wächter muss vorher stehen** | Produktionsbasis fährt mit SISM, S2/S3 ohne → **deklarierter A/B, kein stilles Zurückschalten** | 4 mm: ~66 min |

**Zur Entscheidungsfrage:** die Rekonstruktion löst das Gt-System gar nicht mehr, sie **setzt** u. Die
Azimut-Entartung des *Aktors* ist damit konstruktiv weg, und S3 kann sie nicht prüfen. Was S3 prüft und
was nützlich ist: das Gesamtreibungsbudget. **Wenn nur eine zusätzliche Messung möglich ist, die Kugel aus
S5 vor den Entscheid ziehen** — dort ist der Azimut wirklich frei.

---

## 6 · Konflikte an derselben Zelle

* **`CFD_SGS_FDWAND` (nicht SISM) ist der Zirkel.** nu_t kommt aus |S|_FD des u-Felds (`kernel.cpp:5473-5507`).
  Unter Rekonstruktion ist dieses u gesetzt. **„SISM aus" schließt den Zirkel nicht** — FDWAND muss aus.
  Und `stream_collide` schreibt u selbst (`:3393/3396`), der Zirkel betrifft also auch den
  Wandmodell-Eingang. Die kipp0-Referenz (03.09., `d38acfe`) ist SISM-frei, die Eichung am Kanal bleibt
  damit gültig; die Produktionsbasis (11.09.) fährt mit SISM → S5 ist ein A/B.
* **ELIBB** — s. §3.2, Abgriffpunkt festlegen, S5 braucht den ELIBB-Arm in der Abnahme.
* **RHO_CLAMP** mit eigener K0-Ortsklasse für Facettenzellen (`kernel.cpp:3714-3724`) — eine zweite Klemme,
  die Fassung 1 nicht nannte.
* **Erhaltende Klemme:** Projektionsform erhält Masse und Impuls (`kernel.cpp:3999`), also verträglich. Aber
  die K0-Ausnahme mit Begründung „Wandschub-Weitergabe" (`:4000-4004`) **entfällt** unter Rekonstruktion —
  neu entscheiden, nicht erben. Die Rekonstruktion kann f_i < 0 erzeugen → Zähler (Muster Slot 285) und
  Entscheid, ob `CFD_POSITIV_FACETTE=1` Pflicht wird.
* **P-TRT:** f_neq aus `f_load` (Variante A) ⇒ Geistmoden bleiben, P-TRT greift normal. Variante B baut
  regularisiert auf ⇒ Geistmoden konstruktiv null, Slots 201/203 sähen wie ein No-Op aus und wären korrekt
  — **vorher deklarieren**.
* **APG** wirkt auf das τ_w-**Ziel**, nicht auf den Aktor — keine strukturelle Kollision, aber ein zweiter
  Modellparameter. In S2/S3 aus.

---

## 7 · Diagnostik und Registerfrage

**Slots:** nächster freier ist 317, Puffer `hits_n = **384** seit 22.09.2026 (S−1 erledigt, `lbm.cpp:748`, `lbm.hpp:382`) — **317..383 frei**, der
Plan braucht 12–14 (Wirkpfad, Wirkung, 4–5 ΔP·n̂-Eimer, negative f, 2 Rundung, 2 Umfang/Klasse,
Δm-Überlauf, ΔP_t/twe). Erhöhen ist reine Hostarbeit (`lbm.hpp:382` + Allokation `lbm.cpp:727`; Leser
`setup.cpp:1477/1505/1521/1531/1697-1698`, 384 Slots = 7680 B Arrays bzw. 8800 B je KlemmBilanz-Instanz, Zuwachs 1280 B je Instanz — die fruehere Angabe „9,2 kB" rechnete mit 24 B je Slot statt 20). **S−1 am 22.09.2026 erledigt.**

**Registerdruck ist das eigentliche Scratch-Risiko — nicht Laufzeitindizes.** `stream_collide` hält schon
**sieben** 19-Float-Arrays (fhn, feq, fhb, feb, Fin, j, j2), und die Rang-1-Remat-Notiz (`kernel.cpp:4120-4126`)
existiert, weil der Facettenblock vorher 448/832 B spillte. Die zitierten 7296 B (`:5539-5545`) sind ein
Laufzeit-**Index**-Fall; die Regel „nur Literalindizes" deckt den Array-**Anzahl**-Fall nicht ab.

> **Konkrete Vermeidung:** `f_eq(u_rek)` und `f_eq(u_lok)` **nie als zwei Arrays materialisieren**.
> `calculate_f_eq` ist geschlossene Form (`kernel.cpp:1167`, liefert f_eq − w), also Δ_i inline je
> Literalindex rechnen und `fhn[i] += Δ_i` in place; ΔP (3 Skalare) und Δm (1 Skalar) im selben
> ausgerollten Körper. **Null zusätzliche 19-Arrays.** Die w-Freiheit der Differenz fällt konstruktiv ab.

**Instrumente:** Wirkpfadzähler mit Ist=Soll (`setup.cpp:4754` Kanal, `6061` Kugel, `10017/10087` Fahrzeug)
**und** getrennter Wirkungszähler (|Δu|/|u| > 1e-6) — der Wirkpfadzähler allein feuert auch bei Null-Wirkung.
Δm-Überlaufzähler statt der nutzlosen „Σ Δm == 0"-Prüfung. Sättigende Zähler + `t%def_zaehl_takt`
(`kernel.cpp:2075-2079`). Konstruktor-Wächter gegen den iMEM-Additivterm (Muster `lbm.cpp:180/202/213`).

**Kosten, gerechnet:** 3,13 Mio Facetten = 0,498 % von 628,4 Mio Zellen. Vorkernel + 19 FMA ≈ **0,3 B je
Zelle und Schritt** gegen ~100 B/Zelle/Schritt → **< 0,5 %**. Der trilineare Stern liegt zwischen **0,24 %**
(Bytes, u16) und **7 %** (Cachezeilen, keine Koaleszenz) — die ehrliche Aussage ist „0,2 bis 7 %, je nach
Koaleszenz". Der vorhandene Proxy ist `fac_nachbar_ab` (3 solche Reads je Facette); der Stern ist 8× dieser
Kernel. **Die echten Argumente gegen S4 sind Reichweite und tote Tiles, nicht die Kosten.**

---

## 8 · Offene Punkte vor dem ersten Bau

1. Volltexte Malaspinas & Sagaut 2014 und Maeyama 2021/2022 beschaffen — **der Bestand hat nur Abstracts**,
   und der einzige vermeintliche Treppen-Zahlenwert hat sich als falsch belegt erwiesen (§0).
2. FP16S-Rauschlast auf f_neq beziffern (Histogramm |f_neq|/f_eq an Facettenzellen) — entscheidet mit
   zwischen Variante A und B.
3. `boden_eq` × Rekonstruktion: Wächter oder bezifferte Ausnahme.
4. `fac_tau_cnt`-Politik: erhöht die Rekonstruktion ihn? Davon hängt der ganze Druckpfad ab.
5. Abgriffpunkt von `f_load` relativ zu ELIBB festlegen.
6. ~~`CFD_FAC_KRAFT=1` am Fahrzeug messen~~ — **erledigt, 30.08.2026, siehe §0b**: cd_druck_rest
   +0,2313 ± 0,0102 (22,7 σ), cz_druck_rest 1,06 σ. Offen ist statt dessen **`CFD_FAC_KRAFT=2`**
   (alle Facettenzellen statt nur Rückfallzellen) — das ist die Obergrenze für den Umfang „alle"
   aus §3.1 und am Fahrzeug nie gemessen.

---

## 9 · Ankerkorrekturen aus Fassung 1

`kernel.cpp`: 2593 → **2596**; 2513 → **2521**; 5100-5103 → **5083-5086**; 5072-5090 → **5074-5085**;
5533-5540 → **5539-5545**; 2988-2992 → **2990-2994**; 3020-3022 → Kommentar **3025-3026**, echte Wächter
in `lbm.cpp:180/202/213` und `setup.cpp:4413`.
`setup.cpp`: „4798ff" (Slot-7 Ist=Soll) → **4754 / 6061 / 10017 / 10087**; 4798ff ist der
Doppelbuchungs-Detektor.

---

## 10 · APG-Performanceverlust — Befund und Hebel

**Gemessen (16.09., 4 mm):** APG kostet +17,0 % Zeitschleife und −14,6 % Durchsatz (6000 → 5124 MLUPs).
Im heutigen Lauf p4_regel4 bestätigt: 5279 MLUPs gegen p4_pu8 6000 = −12 %, Index 7967 gegen 5701.

**Der Code rechnet seine Kosten selbst vor** (`kernel.cpp:5579`):
> *„Verkehr: 7 Zellen x (19 x 2 B + 1 B) je Facette und Schritt = **273 B/Facette** (Rechnung;
> Cache-Wiederverwendung ungemessen)."*

`apg_rho_zelle` holt rho **aus den DDFs** — `load_f(…, t+1)` für die Zelle und ihre sechs Achsnachbarn.

| | |
|---|---|
| Facetten (p4_regel4) | 3 127 618 |
| APG-Verkehr | 854 MB je feinem Schritt, **3,42 GB** je Grobschritt |
| bei 254 GB/s Spitze | 13,4 ms von 510 ms = **2,6 %** |
| gemessen kostet APG | **17,0 % = 87 ms** |
| **Lücke** | **Faktor 6,5** |

**Das Volumen erklärt den Verlust nicht — das Zugriffsmuster tut es:** 7 × 19 = **133 Streuzugriffe je
Facette**, macht **416 Mio Gather-Operationen je feinem Schritt** über eine dünne Schale. Unter
Esoteric-Pull liegt jede der 19 Populationen an einer anderen Adresse.

### Warum der teure Weg gebaut wurde — und das ist der Hebel

Der Kommentar sagt es wörtlich (`kernel.cpp:5576-5578`):
> *„KEIN rho-Puffer wird gelesen: RHO_RAND, RHO_SPARSAM und RHO_FP16 bleiben unberuehrt, die alten
> Sperren entfallen."*

**Weil rho unter RHO_SPARSAM/RHO_RAND nicht überall vorgehalten wird, rekonstruiert APG es aus den DDFs.**
Die Sparschalter bremsen APG also nicht direkt — sie sind der *Grund*, warum APG den teuren Pfad nimmt.

`RHO_RAND-PLAN.md:333` führt die Alternative als **offenen Punkt**: *„APG-Weg (a) Region oder (b) DDFs im
eigenen Kernellauf? … RHO_RAND blockiert keinen der beiden."* Gebaut ist (b).

**Weg (a), aus dem heutigen Lauflog beziffert:** die APG-Lesemenge umfasst **6 380 057 Zellen**, die
rho-Region dafür kostet **12,76 MB FP16** (`logs/p4_regel4.log`, RHO_RAND C0 Zensus). Damit fiele der
Verkehr von 273 B auf **7 × 2 B = 14 B je Facette** — **Faktor 19,5** — und von 133 auf 7 Zugriffe.

**Preis: 12,76 MB VRAM** von 3 733 MB gemessener Restluft.

**Der Haken, der es bisher verhindert hat:** `load_f(t+1)` liefert *„das rho, das stream_collide(t+1) dort
selbst bilden wird … synchron zum rhon der Facette im naechsten Schritt statt des alten t-1/t-Gemischs
(Befund A3, bitreproduzierbar)"*. Ein rho-Puffer trägt rho aus Schritt t. Man tauscht also Determinismus
gegen Tempo — und der Determinismus war am 03.09. teuer erkauft (`lbm.cpp:1716`).

### Drei Hebel, nach Aufwand

| | Hebel | Erwartung | Preis |
|---|---|---|---|
| **1** | **Timer um `fac_apg_ab`** (Muster `CFD_TIMER_FERN`) | zerlegt die 17 % — **ohne das ist alles andere geraten** | Zweizeiler, 8 mm, eine Variable |
| **2** | **Zähltakt**: die Nachbarschleife läuft in **jedem** Schritt, obwohl dp/ds sich über vier feine Schritte kaum ändert. Muster `fac_nachbar_ab` | bis zu 4× weniger Aufrufe | eine Variable, **kein** Determinismusverlust |
| **3** | **Weg (a)**: rho-Region statt DDF-Rekonstruktion | Faktor 19,5 auf den Verkehr | 12,76 MB VRAM **und** der t+1-Determinismus muss neu begründet werden |

**Reihenfolge: 1, dann 2, dann erst 3.** Hebel 3 fasst eine Eigenschaft an, die einmal teuer erkämpft
wurde — das lohnt nur, wenn Hebel 1 zeigt, dass der Verkehr wirklich der Engpass ist.

**Und der Vorbehalt, der bleibt:** APG bewegt die Kräfte weder bei 8 mm noch bei 4 mm
(`PLAN-APG-2026-09-16.md` §I: cd_druck_rest −0,0022 ± 0,0011, 41 Vorzeichenwechsel). Die Übergabe vom
16.09. empfahl **„endgültig parken"**. Heiko hat ihn am 21.09. bewusst im Standard gelassen. Solange er
keine messbare Kraftwirkung hat, ist die billigste Performance-Maßnahme, ihn auszuschalten — das gehört
ehrlich neben die drei Hebel gestellt, auch wenn es nicht die gewünschte Antwort ist.

---

## 11 · Reihenfolge für morgen

| # | Schritt | Warum zuerst | Kosten |
|---|---|---|---|
| **1** | ~~`CFD_FAC_KRAFT=1` am Fahrzeug messen~~ → **entfällt (§0b)**. Ersatz, falls Heiko einen 8-mm-Lauf freigibt: **`CFD_FAC_KRAFT=2`** (alle Facettenzellen), gepaart gegen eine frische Basiszeile, `CFD_KRAFT_ZBAND=3` | KRAFT=1 ist **gemessen** (30.08.): cd_druck_rest +0,2313 ± 0,0102 = 22,7 σ, cz_druck_rest 1,06 σ — ein validiertes Negativ. KRAFT=2 ist am Fahrzeug nie gemessen und ist die Obergrenze für den Umfang „alle" aus §3.1. Vorbehalt unverändert: `object_force` sieht die Guo-Kraft nicht, der Reibanteil steht allein in der `fac_tau`-Buchung | ein 8-mm-Lauf, **nur mit Freigabe** |
| **2** | **Timer um `fac_apg_ab`** | zerlegt die 17 %, bevor irgendetwas optimiert wird | Zweizeiler + 8-mm-A/B |
| **3** | **FP16S-Rauschlast auf f_neq** beziffern (Histogramm \|f_neq\|/f_eq an Facettenzellen) | entscheidet zwischen Rekonstruktions-Variante A und B, **bevor** gebaut wird | Host-Auswertung, keine GPU |
| **4** | ~~**S−1**: `hits_n` 320 → 384~~ — **erledigt 22.09.2026**, Commit s. Tagesprotokoll. Der `scratch_gate`-Arm rutscht nach S0 (kein Define in S−1). Dabei gefunden und mitbehoben: `setup.cpp:1425` führte die Slotzahl hart als 320 | ohne das schreibt jeder neue Zähler still ins Nichts | eigener Commit |
| **5** | **S0+S1** der Rekonstruktion (Laufzeitparameter, kein JIT-Define) | beweist Einbauort und Zählung, bevor Physik dazukommt | CPU, Minuten |
| **6** | offene Punkte §8 abarbeiten (Volltexte, `boden_eq`-Wächter, `fac_tau_cnt`-Politik, ELIBB-Abgriffpunkt) | alles Lesearbeit, keine GPU | — |

**Was NICHT morgen passiert:** S2 und später. Erst müssen 1–4 stehen.
