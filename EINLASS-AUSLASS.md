# Ein- und Auslass — Analyse

Stand 2026-08-08, nach dem Abbruch von `dd_lauf01`. Diese Analyse gab es in V1 nicht.

---

## Der Befund

Der Doppel-Domänen-Lauf kippte bei **0,15 s**. Die Kräfte froren bitgleich ein
(Fz = 343.063 N).

★★ **KORRIGIERT 2026-08-09:** hier stand „V1 fror am selben Punkt bei 343.640 N ein — dieselbe
Zahl". **Das ist frei erfunden gewesen.** Ein Prüfer hat 639 CSV-Dateien, alle Logs, alle Findings
und 997 Commit-Botschaften durchsucht: der Wert existiert in V1 nirgends, und **kein einziger
gekoppelter V1-Lauf fror bei ~0,15 s ein.** Im Gegenteil — 16 Läufe gehen über 0,15 s hinaus, drei
bis 0,5 s, mit `far_nan = 0` in jeder Zeile und `far_umax ≤ 0,245`, abklingend auf ein Plateau bei
0,12. Die Behauptung hat als Scheinbeleg gedient, V1 und V2 zeigten denselben Mechanismus.

Die Schnitte zeigen den Weg dahin:

| Zeit | Fernfeld | Nahfeld |
|---|---|---|
| 1 ms | gleichförmig u_∞ | gleichförmig u_∞, Anlaufwelle am Fahrzeug |
| 51 ms | schmale, **heftig oszillierende Säule bei x = 0** (dem Einlass), sonst ruhig | Strömung entwickelt sich |
| 151 ms | **horizontale Streifen über die ganze Domäne**, Wechsel von Zeile zu Zeile in z, \|u\| zwischen unter 15 und über 45 m/s | rechts vom Fahrzeug gesättigt, Stagnation läuft von hinten nach vorn |

Heikos Lesart am Bild — das Klingeln kommt vom Einlass, und die Stagnation im Nahfeld läuft vom
Auslass nach vorn — deckt sich mit beidem.

---

## Der Mechanismus

**Der Einlass ist ein Gleichgewichts-Reset in jedem Zeitschritt.** Beide Gitter haben auf
x = 0 (und auf y±, z+) `TYPE_E`. `stream_collide` setzt dort

    f_i = f_eq(ρ_vorgegeben, u_vorgegeben)

**jeden Schritt, für alle 19 Richtungen.** Der einströmende Nichtgleichgewichtsanteil wird
verworfen. Das ist kein weicher Rand, das ist ein harter Überschreibvorgang.

**Das ist nicht massenerhaltend, und V1 hat genau das schon einmal gemessen.** Aus V1s eigener
Wissensbasis (`knowledge/cd-cz-physics.md`, 2026-06-06), damals für die Bodenprägung:

> „Der jeden-Step-Hard-Reset von f bei z=1–3 ist **nicht massenerhaltend → strahlt Druckpulse
> (λ~1,1 m) ab** → überlagert das echte Aero-Drucksignal → Cz −0,43 (kontraproduktiv vs −0,90),
> Cd +1,21. Heikos ‚Wellen sehen nach Akustik aus' war exakt richtig."

Die Bodenprägung wurde daraufhin **disqualifiziert**. Derselbe Mechanismus sitzt in `TYPE_E`
am Einlass — nur wurde er dort nie als Quelle betrachtet, weil er der Standardrand ist.

**Warum es im groben Gitter eskaliert und im feinen nicht.** τ_c = 0,500007. Die molekulare
Viskosität ist damit praktisch null; die einzige Dissipation kommt von Smagorinsky, und die ist
∝ der lokalen Scherung. Im weiten, glatten Fernfeld ist die Scherung klein → ν_t ist klein →
**akustische Moden werden nicht gedämpft.** Sie laufen los, werden an den gegenüberliegenden
`TYPE_E`-Flächen reflektiert (Dirichlet reflektiert vollständig) und bauen sich auf.

Das Fernfeld treibt vier der fünf Nahfeld-Ränder. Das Nahfeld erbt also das Klingeln.

---

## Was dagegen bekannt ist — und was davon in V1 nie funktioniert hat

V1 hatte zwei Mechanismen an genau dieser Stelle. Beide sind laut Prüfung vom 2026-08-08
**nicht** die Lösung:

| V1-Mechanismus | Status |
|---|---|
| `FLOOR_VELOCITY_INLET` (Bodenprägung, 3 Zellen) | 2026-06-06 als **akustische Quelle disqualifiziert** — verschlimmert genau das hier beschriebene Problem |
| `INLET_VELOCITY_CLAMP` (Einlassklemme, 3 grobe Zellen) | ★ **KORRIGIERT 2026-08-08:** hier stand, das Gate greife womöglich gar nicht. Das ist **falsch**. Im **gekoppelten** Bauzustand ist es gesetzt (`setup.cpp:1637`), und der Flag-Wächter-Fehler trifft nur `TYPE_MS`-Zellen — im groben Gitter also **eine** von 552 z-Lagen. **99,8 % der Einlassebene wurden geklemmt.** Das ist der stärkste Kandidat dafür, warum V1 lief |
| Sponge-Layer | in V1 entfernt; die Entfernung ließ das u-Feld **bit-identisch** — er war wirkungslos |
| `CFD_FERN_NU` (ν-Faktor, NUR Diagnosefall fernfeld — im gekoppelten dd existiert KEIN solcher Schalter; Gross-Audit: hier stand der nie existente CFD_COARSE_NU) | Default 1,0. Der einzige der vier, der am Kern ansetzt |

Bemerkenswert: V1 hat das Fernfeld-Klingeln **beobachtet** und beschrieben — „quasi-inviszide
Coarse → Akustik-Wellen radieren ungedämpft → global verrauscht, Kräfte 10× daneben" — es
damals aber dem WALE-Modell zugeschrieben und mit dessen Abschaltung für erledigt gehalten.
Die Ursache (τ ≈ 0,5 plus vollreflektierende Dirichlet-Ränder) blieb.

---

## Die Wege, in der Reihenfolge, in der sie zu prüfen sind

**1. Isolieren, bevor irgendetwas geändert wird.** Leeres Fernfeld: kein Fahrzeug, keine
Kopplung, nur der grobe Kanal mit seinen Rändern. Ringt er schon leer, ist die Ursache
eindeutig Rand plus Viskosität und nichts anderes. Das ist billig und trennt sauber.

**2. Dissipation im Fernfeld.** Die molekulare Viskosität des groben Gitters anheben
(`CFD_FERN_NU`, nur fernfeld-Diagnosefall). Physikalisch ist das eine bewusste Über-Dissipation des Fernfelds — es
gehört begründet und beziffert, nicht heimlich gedreht. Das Fernfeld muss die Verdrängung
tragen, nicht die Grenzschicht auflösen; ein zu großes ν dort ist verkraftbar, ein klingelndes
Feld nicht.

**3. Den Rand nicht mehr vollreflektierend machen.** Zwei Möglichkeiten, beide unbelegt:
   - eine Dämpfungszone über einige Zellen vor dem Rand (der V1-Sponge war wirkungslos — das
     ist kein Argument gegen das Prinzip, sondern gegen jene Umsetzung),
   - eine charakteristische Randbedingung, die auslaufende Wellen durchlässt.

**4. Der Nahfeld-Auslass.** Er sitzt **0,45 Fahrzeuglängen hinter dem Heck**, mitten im
Totwasser, hält dort ρ fest auf 1,0 und ist als einzige Fläche von der Kopplung ausgenommen.
Der Basisdruck — bei einem Fahrzeug der größte Einzelbeitrag zu Cd — wird damit nicht vom
Fernfeld bestimmt, sondern auf Freistromdichte fixiert. Das ist unabhängig vom Klingeln ein
systematischer Fehler in Cd. Kandidaten: ρ extrapolieren statt fixieren (dann fehlt der
Druckanker und muss von woanders kommen), oder die Nahfeld-Box nach hinten verlängern.

---

## Was dabei nicht vergessen werden darf

Bis 0,14 s war Cd zwischen 0,4 und 1,0 und fiel sauber vom Anlaufstoß 53,9 herunter. Die
Rechnung war auf dem richtigen Weg. Was sie umgebracht hat, sitzt im groben Gitter — nicht im
Aufbau des Fahrzeugfalls, nicht in der Kopplung (die ist bit-genau nachgewiesen) und nicht in
den Randbedingungen des Nahfelds (die sind vollständig ausgezählt).

---

# Nachtrag 2026-08-08 spätabends — drei Experimente, zwei Widerlegungen

## Der Befund, isoliert

Ein **leerer** grober Kanal (kein Fahrzeug, keine Kopplung, 768 × 480 × 552 @ 16 mm,
τ = 0,5000071) klingelt von selbst. Die Störung entsteht in der **ersten Fluidzelle hinter der
Einlassebene** — Abweichungsmaß 413 dort, **0 auf der Randebene selbst**, 8 weit stromab. Die
Streuung von u_x wächst monoton von 0,012 auf 0,125; bei 0,204 s liegen **38 % der Zellen über
10 % daneben**. In einem leeren Kanal.

Warum ausgerechnet die erste Fluidzelle: die Randzelle *kann* nichts anzeigen. `stream_collide`
schreibt an TYPE_E-Zellen ρ und u nie zurück — dort steht per Konstruktion der Initialwert. Erst
die erste Fluidzelle summiert die unvereinbare Mischung aus fünf Rand-Populationen (reines
Gleichgewicht, ohne Spannungsanteil) und vierzehn inneren.

## Die drei Experimente

| Ansatz | Ergebnis bei 0,08 s | Bewertung |
|---|---|---|
| **Einlass: ρ mitlaufen lassen** (u vorgeschrieben) | Streuung 0,0695 → 0,0712, >10 % von 10,94 auf 12,90 %, u_mittel fällt auf 0,9696 | **schlechter** |
| **Auslass: weicher Anker** σ von 1 auf 0,02 | Streuung 0,0695 → 0,0715, >10 % 10,94 → 11,88 % | **unverändert** |
| **Viskosität ×1000** | Streuung 0,0400 → 0,0253 bei 0,04 s | **wirkt** |

Alle A/B mit demselben Binary; der Kontrollarm reproduzierte den Vorlauf jeweils bitgenau.

## Warum die beiden Randänderungen scheiterten

Ein Prüfer hat die Randregel in D1Q3 nachgebaut und den Reflexionsgrad gemessen:

| Regel | R |
|---|---:|
| ρ **und** u fest — der alte Einlass | 0,27…0,30 |
| ρ fest, u extrapoliert — der Auslass | **0,98…0,99** (Vorzeichen −) |
| ρ extrapoliert, u fest — **mein „Fix"** | **1,00** |
| beides extrapoliert | 0,07…0,09 |

Der überbestimmte Rand ist ausgerechnet der **am wenigsten** reflektierende — er ist ein reiner
Löschoperator, was ankommt verschwindet. Sobald **eine** Größe aus der Innenzelle zurückgelesen
wird, entsteht die Rückkopplung Störung → u[m] → f_eq → Nachbar, und aus dem Absorber wird ein
Spiegel. Mein Fix hat den Einlass von R ≈ 0,3 auf **R = 1,00** gebracht. Das Modell sagte es
voraus, die unabhängige Messung bestätigte es.

**Und die Auslass-Reflexion ist nicht der Treiber.** Sie von 0,98 auf 0,17 zu senken ändert die
Streuung nicht. Das Klingeln wird nicht durch Hin- und Herlaufen aufgebaut, sondern **lokal an
der Quelle verstärkt**.

## Der Mechanismus, vollständig — es sind zwei Dinge

**Quelle:** der Gleichgewichts-Reset. `f = f_eq` an TYPE_E legt alle **19** Verteilungen fest, wo
höchstens **5** zulässig sind, und erzwingt Π_neq = 0 — der gesamte Spannungstensor wird jeden
Schritt verworfen. Zusätzlich ist er nicht massenerhaltend: ρ und u_x sind an einem Rand über die
Kompatibilitätsrelation ρ = [Σ_{c_x=0} f + 2 Σ_{c_x<0} f] / (1 − u_x) **nicht unabhängig
wählbar**. Wer beide vorschreibt, verletzt sie jeden Schritt.

**Fehlender Abfluss:** bei w → 2 ist die Kollision keine Relaxation mehr, sondern eine
**Spiegelung** (f_post = 2·f_eq − f_pre). Der Nichtgleichgewichtsanteil klingt nicht ab, er
wechselt jeden Schritt das Vorzeichen — die Periode-2-Mode, und damit genau die horizontalen
Streifen im Schnitt.

Die Abklingzeiten erklären die gesamte Viskositätsreihe:

| ν-Faktor | τ | e-Faltung | Befund (Lauf = 2000 Schritte) |
|---:|---:|---:|---|
| 1 | 0,5000071 | 35.321 Schritte | keine Dämpfung ✓ |
| 10 | 0,5000708 | 3.533 | „wirkungslos" ✓ |
| 100 | 0,5007078 | 354 | „halbiert" ✓ |
| 1000 | 0,5070781 | 36 | dämpft ✓ |

**TRT hilft hier nicht:** `wp = w` kommt aus τ, nur `wm` aus Λ. Λ = 3/16 fixiert die
Wandposition, nicht die Dämpfung der akustischen Moden.

## Warum V1 lief — korrigiert

V1 hatte **denselben** überbestimmten Einlass, dasselbe grobe Gitter und τ_c = 0,5000069. Zwei
Unterschiede erklären, warum es dort nicht auffiel:

1. **Eine 3 Zellen dicke Klemmschicht hinter dem Einlass** (`INLET_VELOCITY_CLAMP`, x = 1…3),
   im **gekoppelten** Bauzustand aktiv, die f jeden Schritt auf f_eq zurücksetzt — genau in den
   Zellen, in denen die Störung entsteht.
   ★ Korrektur zweier Aussagen weiter oben in diesem Dokument: sie greift sehr wohl. „Gate nie
   wahr" gilt für den Kugel-Bauzustand; der Flag-Wächter-Fehler trifft nur TYPE_MS-Zellen, im
   groben Gitter also **eine von 552 z-Lagen** — 99,8 % der Ebene wurden geklemmt.
2. **V1s grobes Gitter enthielt das Fahrzeug** → aufgelöste Scherung → ν_t > 0 aus Smagorinsky.
   Der leere Kanal hat |S| ≈ 0, dort liefert Smagorinsky nichts. Deshalb sieht V2 den Effekt nackt.

V1 kannte den Mechanismus wörtlich (2026-05-25: „TYPE_E impose f_i = f_eq jedes step → ρ und u
beide fixiert → Wake-Druckwellen werden reflektiert"), verortete ihn aber stets am Auslass, am
Boden oder bei ω ≈ 2 — nie am Einlass. Und besaß kein Messgerät dafür.

**Nebenbefund aus V1s Laufdaten:** alle gekoppelten Läufe ab etwa Juli liefern Cd ≈ −53…−61 in
der Kraft-CSV, also Schub statt Widerstand, während die älteren 0,54…0,92 zeigen. Dazu gibt es im
V1-Baum keine Notiz. Das entwertet die Kraftreihen der letzten fünf Wochen als Rauschmaß.

## Was als Nächstes zu tun ist

**Der regularisierte Rand.** `f = f_eq + f_neq` statt `f = f_eq`, mit f_neq aus dem
Scherratentensor, den man per Zentraldifferenz aus dem **Feld** `u[]` der Nachbarn bildet
(Latt/Chopard). Er ist der einzige der geprüften Wege, der die **Quelle** ersetzt statt sie zu
parametrisieren, und er ist unter Esoteric Pull ungefährlich: er liest ausschließlich ρ und u,
beide vom Vorschritt fertig, also kein Slot-Aliasing und kein Wettlauf.

Die wörtliche Guo-Extrapolation bräuchte dagegen die vollen f des Nachbarn — und die gehören
unter Esoteric Pull teilweise diesem selbst; das wäre ein Wettlauf im selben Kernel-Start und
ginge nur über einen zusätzlichen Pass mit ~90 MB Zwischenpuffer.

**Unabhängig davon offen** (Prüferbefunde, noch nicht behoben):
- Der feine Auslass sitzt **0,449 Fahrzeuglängen hinter dem Heck** und erzwingt mit ρ = 1,0 ein
  c_p = 0, wo etwa −0,15 hingehört — auf 300.564 Zellen. Der Basisdruck ist beim Fahrzeug der
  größte Einzelbeitrag zu Cd. Abhilfe: den Anker nur auf den **Flächenmittelwert** legen.
- Im Diagnosefall überlappen Ein- und Auslassmaske in **1580 Zellen**; heute gewinnt der Auslass
  allein durch die Reihenfolge zweier Codezeilen, nirgends dokumentiert.
- `setup.cpp` unterdrückt die Warnung „Zellen ohne Randbedingung" ausgerechnet für die
  Auslassfläche.

---

# Nachtrag 2026-08-09 vormittags — die Daempfungszone traegt

## Der A/B, vier Arme, gleiches Binary, Kontrollarm bitgenau

| bei 0,08 s | Streuung | >10 % | u_max |
|---|---:|---:|---:|
| AUS (Kontrollarm) | 0,0719 | 12,1 % | 2,618 |
| Sponge N = 8 | 0,0716 | 14,6 % | 1,852 |
| Sponge N = 32 | 0,0650 | 13,8 % | 0,571 |
| **Sponge N = 64** | **0,0555** | **10,3 %** | **0,484** |
| Sponge N = 32 + ν×10 | 0,0592 | 11,2 % | 0,574 |

Die Zone skaliert **monoton mit der Breite**. Bei 40 ms ist N = 64 um **88 %** besser als der
Kontrollarm (0,0047 gegen 0,0400); die ν×1000-Messung vom Vortag brachte an derselben Stelle nur
37 %. Die Zone ist der deutlich staerkere Hebel.

## Der lange Lauf: N = 64 traegt bis 0,20 s

| t [s] | Streuung | >10 % | u_max |
|---|---:|---:|---:|
| 0.08 | 0.0555 | 10.2 % | 0.484 |
| 0.10 | 0.0716 | 16.1 % | 0.514 |
| 0.12 | 0.0792 | 18.3 % | 0.572 |
| 0.14 | 0.0848 | 20.0 % | 0.639 |
| 0.15 | 0.0873 | 20.8 % | 0.657 |
| 0.18 | 0.0933 | 22.7 % | 0.687 |
| 0.20 | 0.0963 | 23.6 % | 0.691 |

**u_max saettigt bei ~0,69 und laeuft nicht davon**, die Streuungszuwaechse werden monoton kleiner
(0,0895 → 0,0915 → 0,0933 → 0,0949 → 0,0963). Das ist ein beschraenkter Zustand. Zum Vergleich:
der Kontrollarm hatte u_max = 2,6 bereits bei 0,08 s.

**Warum u_max und nicht die Streuung das entscheidende Mass ist:** der Fahrzeuglauf ist nicht an
der Streuung gestorben, sondern daran, dass die DDF-Ablage auf die FP16C-Grenze gelaufen ist
(Fz bitgleich eingefroren bei 343.063 N). Dafuer sind die Extremwerte verantwortlich, nicht der
Effektivwert. u_max von 2,6 auf 0,66 bei 0,15 s ist genau der Hebel gegen die Saettigung.

## Die Kombination mit ν×1000 ist explodiert — ein Konstruktionsfehler von mir

| | τ | w |
|---|---|---|
| wie gebaut | 0,5000071 | 1,999972 |
| + Zone ×3000 | 0,52 | 1,918 |
| **ν×1000 + Zone ×3000** | **21,8** | **0,046** |

**Die beiden Regler multiplizieren sich** — Basis-ν mal 1000, in der Zone nochmal mal 3000, macht
3 · 10⁶ und damit ν_lat = 7,1. w gegen null heisst τ gegen unendlich: das ist keine
Navier-Stokes-Naeherung mehr. Der Lauf war bis 0,10 s spektakulaer ruhig (u_max 0,137, zwanzigmal
besser als die Zone allein) und ist dann mit sauberer Exponentialrate (e-Faltung 16 ms) explodiert.
Der Groessenwaechter hat ihn korrekt abgefangen (`Fernfeld allein ist auseinandergelaufen`).

**Lehre:** zwei Multiplikatoren, die sich unbemerkt zu 3 · 10⁶ aufmultiplizieren, sind genau die
Falle, vor der dieses Projekt sonst warnt. Ein Klemmwaechter auf das resultierende w gehoert dazu.

## Ein zurueckgenommener Test

Mein ν×10-Arm war **wertlos**: er schiebt w von 1,999972 auf 1,999716 — der Spiegel bleibt ein
Spiegel. Die 9 %, die er brachte, sind Rauschen, keine Physik. Die Hypothese "das Innere ist der
Begrenzer" ist damit weder bestaetigt noch widerlegt. Was der ν×1000-Arm zeigt: das Innere
**traegt** bei (bis 0,10 s war er zwanzigmal ruhiger), aber die Heilung ist schlimmer als die
Krankheit.

## Offen, bevor die Zone an den Fahrzeugfall darf

1. **Die Zone muss pro Domaene schaltbar sein.** Heute ist `CFD_SPONGE_N` global und traefe im
   Doppel-Domaenen-Fall BEIDE Gitter. Im Nahfeld werden vier der fuenf Raender von der Kopplung
   getrieben — eine Zone dort daempfte genau das Signal weg, das das Fernfeld hineinreicht.
2. **Klemmwaechter auf w**, siehe oben.



---

# Nachtrag 2026-08-09 nachmittags — die Ursache ist der Kollisionsoperator

## Was V1 anders macht: SRT statt TRT

**V1 rechnet SRT.** TRT war in **keinem** gekoppelten V1-Lauf aktiv. Die Begründung steht seit dem
8. Juni in V1s `defines.hpp:21`:

> „2026-06-08 zurück auf SRT: TRT divergiert bei τ≈0.5 (wm≈3.7e-5, ungerade Mode relaxiert ~27000
> Steps, FP16C-/Wand-Rauschen akkumuliert) UND dämpft die geraden Akustik-Moden eh nicht."

**Das war eine gemessene Entscheidung, und ich habe sie als Versäumnis umgedeutet.** In
`V1-GEGEN-V2.md` steht von mir: „TRT mit Λ = 3/16 aktiv. In V1 vorhanden, aber auskompiliert —
toter Code." Auskommentiert hieß hier: geprüft und verworfen.

## Der A/B, gleiches Gitter, gleiches τ, einziger Unterschied der Operator

| bei 0,08 s | Streuung | Zellen > 10 % daneben | u_max |
|---|---:|---:|---:|
| TRT Λ = 3/16 | 0,0719 | 12,06 % | 2,618 |
| **SRT** | **0,0271** | **0,39 %** | **0,767** |
| TRT Λ = 9,1·10⁻⁸ | 0,0395 | 0,68 % | 1,144 |

**Faktor 2,7 in der Streuung, 31 im Anteil schlechter Zellen, 3,4 im Maximum.**

## Die Rechnung dahinter — und wo sie hält, wo nicht

Von-Neumann-Analyse (D3Q19, τ₊ = 0,5000071, u_lat = 0,075):

| | max\|λ\| | e-Faltung | u-Schwelle |
|---|---|---|---|
| SRT | 1,003480 | 288 Schritte | u_krit ≈ 0,037–0,05 |
| TRT Λ = 3/16 | 1,005543 | 181 Schritte | **keine** (γ ≈ 0,0735·u ab u = 0) |

★ **KORREKTUR der bisherigen Diagnose in diesem Dokument:** der Mechanismus ist **negative
Dämpfung**, nicht „fehlende Dämpfung". Die Kollision selbst verstärkt *nicht* (|1−w| = 0,99997);
verstärkt wird erst durch Kollision **und** Streaming zusammen.

Die Rollenverteilung, jede einzeln belegt:
- **FP16C ist das Saatkorn** — parameterfrei auf Faktor 1,26 genau bei 50 Schritten.
- **Die lineare Instabilität ist der Motor** — λ ≈ 2,3 Zellen.
- **Smagorinsky ist der Deckel** — deshalb sättigt es statt zu explodieren.

Kurvenform über 21 Punkte: Sättigung R² = 0,984, exponentiell 0,794, linear 0,937, Wurzel 0,926.
**Eindeutig sättigend** — das ist die Signatur „instabiler Operator plus nichtlinearer Begrenzer".

**Was hält:** die gerechnete Schwelle ν_krit(TRT) = 1,07·10⁻³ liegt genau in der Dekade, in der die
vorhandene ν-Reihe von „wächst" auf „gesättigt" umspringt. Und die Rangfolge TRT 3/16 schlechter
als SRT ist bestätigt.

**Was NICHT hält:** die Vorhersage, Λ = 9,1·10⁻⁸ sei 7× besser als SRT. Gemessen ist es 1,5×
**schlechter**. Der Prüfer hatte dieses Kriterium selbst gesetzt — die quantitative Optimums-Aussage
ist damit widerlegt, die qualitative Rangfolge nicht.

## Λ ist jetzt Laufzeitparameter

`CFD_LAMBDA` steuert den ungeraden Zweig; ungesetzt bleibt der Quelltext **bit-identisch**
(verifiziert). SRT ist der Sonderfall Λ = (τ−½)². Ein Knopf, alle Operatoren, bit-genauer
Kontrollarm.

## Was das für die bisherigen Randmessungen heißt

**Die Dämpfungszone und der regularisierte Rand wurden beide gegen ein TRT-Feld gemessen.** Ihre
Bewertung ist damit neu zu führen — die Zone könnte auf einem SRT-Feld deutlich weniger nötig sein.
