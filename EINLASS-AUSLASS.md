# Ein- und Auslass — Analyse

Stand 2026-08-08, nach dem Abbruch von `dd_lauf01`. Diese Analyse gab es in V1 nicht.

---

## Der Befund

Der Doppel-Domänen-Lauf kippte bei **0,15 s**. Die Kräfte froren bitgleich ein
(Fz = 343.063 N; V1 fror am selben Punkt bei 343.640 N ein — dieselbe Zahl, also die auf die
FP16C-Grenze gelaufene DDF-Ablage, keine Kraft).

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
| `INLET_VELOCITY_CLAMP` (Einlassklemme, 3 grobe Zellen) | am 2026-06-16 **nach** der Disqualifikation eingeführt, gleicher Mechanismus. Die Prüfung fand zudem: das Gate wird nur im gekoppelten Bauzustand gesetzt, und der Kernel **enthält noch den alten Flag-Wächter-Fehler** — er greift also womöglich gar nicht |
| Sponge-Layer | in V1 entfernt; die Entfernung ließ das u-Feld **bit-identisch** — er war wirkungslos |
| `CFD_COARSE_NU` (ν-Anhebung im Fernfeld) | vorhanden, Default 1,0 = aus. Der einzige der vier, der am Kern ansetzt |

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
(`CFD_COARSE_NU`). Physikalisch ist das eine bewusste Über-Dissipation des Fernfelds — es
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
