# OPEN_Ludwig — Bewertung eines fremden LBM-Lösers

**Gegenstand:** `https://github.com/McBorisson/CFD-LBM-OpenLUDWIG`, Fassung **b28d1bb**
(„minor", 09.02.2026), geklont und gelesen am 12.09.2026. 9190 Zeilen Julia, GPU über
CUDA/KernelAbstractions.jl, 19 Validierungsfälle im Repo.

**Was hier Messung ist und was nicht:** alle Aussagen über *ihren* Code sind am Quelltext der
genannten Fassung geprüft. Ihre Leistungs- und Speicherzahlen sind **ihre Dokumentation**, nicht
auf dieser Maschine gemessen; unsere Gegenzahlen stammen aus `PERFORMANCE.md` (11.09.2026,
Lauf `p4_neu`). Hardware wird **nicht** gegeneinander verrechnet — wo verglichen wird, dann als
Anteil an der jeweils eigenen Dachlinie.

---

## 1 · Was der Löser ist

D3Q27, Kumulanten-Kollision (Geier 2017) als Vorgabe, regularisiertes BGK als Altlast,
WALE-Feinstrukturmodell, Gleichgewichts-Wandmodell (WMLES), Bouzidi-Randbedingung,
block-strukturierte Mehrgitterverfeinerung (8³-Blöcke, 2:1 in Zeit und Raum, bis 12 Stufen),
optionale Kompressibilitätskorrektur mit O(u⁴)-Gleichgewicht und zweitem Verteilungssatz für die
Temperatur. Kräfte über Oberflächenspannungsintegration auf den STL-Dreiecken. Dazu ein
Nachlaufwerkzeug, das Dreieckskräfte auf FE-Knoten abbildet.

Das ist ein ernstzunehmender Funktionsumfang, und er überschneidet sich stark mit unserem.

---

## 2 · Die vier Dinge, die wir lernen könnten

### 2.1 Kumulanten-Kollision — der eigentliche Fund

**Unser Problem, wörtlich:** τ = 0,500028 am 4-mm-Fahrzeug. Dagegen haben wir P-TRT
(`CFD_PTRT=1.90`), SISM, DETEPS, eine Geschwindigkeitsklemme und eine Smagorinsky-Konstante
gebaut, die laut `GRENZSCHICHT-SGS-PLAN.md` eine **Stabilitätskrücke** ist (ohne SUBGRID 869 NaN).

Ihre Antwort auf dieselbe Frage ist die Kumulanten-Kollision: zweites Moment in Spur und
deviatorischen Anteil zerlegt, dritte und vierte Ordnung getrennt relaxiert
(`src/collision/kernel_cumulant.jl:418-470`). Das ist die Methode, für die τ → 0,5 konstruiert
ist, und sie ist ein Verfahren, nicht eine Klemme.

**Aber:** ihre Umsetzung ist nicht der Beweis. Siehe Abschnitt 4.1 — ihr „adaptives ω₄" ist
nachweisbar konstant. Wer das baut, baut es aus Geier 2017, nicht aus diesem Repo.

Aufwand hoch, Nutzen potenziell groß, Risiko: D3Q27 statt D3Q19.

### 2.2 Cauchy-Schwarz-Realisierbarkeitsgrenze — billig und prinzipiell

`kernel_cumulant.jl:440-450` klemmt die **Nebendiagonalen des zweiten Moments** auf
|c_αβ| ≤ √(c_αα·c_ββ). Nachgeprüft: `c200_s` trägt den vollen Spuranteil (Zeile 431), die Grenze
ist also die echte Realisierbarkeitsbedingung und keine Heuristik.

**Das ist genau der Ort, an dem unsere Instabilität sitzt, und wir klemmen anderswo.** Unsere
Klemme steht auf der Geschwindigkeit, je Komponente, bei 0,57735 Gittereinheiten — und sie
feuert: am 10.09. trug die heißeste Zelle in `p4_voll` zwei Komponenten **exakt** auf dem
Klemmwert 230,94008 m/s. Das ist Symptomunterdrückung am Ergebnis. Eine Schranke auf dem
Spannungstensor greift eine Stufe früher, dort wo die Divergenz entsteht.

Aufwand niedrig bis mittel. **Das ist der konkreteste übernehmbare Kandidat.**

### 2.3 O(u⁴)-Gleichgewicht — und es gehört zur heutigen Messung

Ihre Kompressibilitätskorrektur setzt Hermite-Terme dritter und vierter Ordnung ins
Gleichgewicht und verschiebt damit die gültige Mach-Zahl nach ihrer Doku auf 0,6–0,9.

Das ist **genau der Preis, den die heutige `u_lat`-Anhebung bezahlt**: der
Kompressibilitätsfehler skaliert mit Ma². Bewegen sich die Kräfte im Arm `uv8_u100` über die
Fehlerbalken, ist das O(u⁴)-Gleichgewicht der Mechanismus, der denselben Zeitgewinn ohne diesen
Fehler holt. Der Hebel Laufzeit ∝ 1/u_lat ist zu groß, um ihn an Ma² abzugeben, wenn es einen
bekannten Ausweg gibt.

Aufwand mittel. Einzuordnen **nach** dem Ergebnis des u_lat-A/B.

### 2.4 WALE statt Smagorinsky — mit einem Widerspruch zu unserem eigenen Fahrplan

WALE liefert ν_t = 0 in reiner Scherung und an der Wand **ohne** Dämpfungsfunktion. Unser
SISM-Befund (10.09.: SISM senkt ν_t in Lage 1 um 85,2 % und erzeugt genau dort die Ausreißer,
84,6 % der Ausreißer sind direkte Wandnachbarn) zeigt, dass wir dieses Problem per Konstruktion
angehen statt per Modell.

**Der Widerspruch, der benannt werden muss:** WALE braucht den u-Gradienten, also einen
Nachbar-Stencil auf dem Geschwindigkeitsfeld (`kernel_cumulant.jl:262`, `compute_velocity_gradients`).
Unsere TODO 2 will das Geschwindigkeitsfeld **seltener schreiben** (16,8 % des Verkehrs für eine
Leserschaft unter 2 %). WALE flächig eingeschaltet macht jede Zelle zur Leserin und nimmt TODO 2
die Grundlage. Beides ist zu haben — aber nicht unbesehen.

---

## 3 · Wo wir deutlich vorn sind

| | OPEN_Ludwig (b28d1bb) | FluidX3D-v2 (`p4_neu`, gemessen) |
|---|---|---|
| **Speicher je Zelle** | **476 B** in der Vorgabekonfiguration: vier volle 27er-Float32-Felder (`f`, `f_temp`, `f_post_collision`, `f_old`, `blocks.jl:145-157`) plus `rho`/`rho_old`/`vel`/`vel_temp`/`vel_old` | **39,6 B** (27 310,9 MiB für 722,6 M Zellen beider Domänen) |
| **DDF allein** | D3Q27 × FP32 × 2 Puffer = 216 B | D3Q19 × FP16S × ein Puffer = **38 B** |
| **Bandbreitenausnutzung** | 350 MLUPs × 216 B = 75,6 GB/s; gegen die Nennbandbreite der RTX 4090 (1008 GB/s, Herstellerangabe, hier nicht gemessen) = **7,5 %** — Grundlage ist ihre Doku-Tabelle „Large/Very Large" | 420 GB/s von 608 GB/s Spitze = **69 %** (gemessen, `PERFORMANCE.md`) |
| **q-Werte der Randbedingung** | float16, als Speicheroptimierung ausgewiesen | **uchar**, 18 B je aktive Facette (`lbm.hpp:268`) |
| **Wandmodell** | Volumenkraft entgegen dem **vollen** Geschwindigkeitsvektor, keine Tangentialprojektion, einseitig (`if tau_wall > tau_res`), eine Korrekturstufe | Tangentialzerlegung, Normalkompensation, Nachbarabtastung, ELIBB, Rang-1-Pseudoinverse |
| **Prüfpunkt/Neustart** | **nicht vorhanden** | ebenfalls nicht — hier ist nichts zu holen |

**Ihre Doku widerspricht sich selbst beim Speicher:** sie nennt 220 B/Zelle, die Vorgabe
(`temporal_interpolation: true`, Randzellen vorhanden) alloziert aber 476 B/Zelle. Faktor 2,2.

Und ein Muster, das wir gestern erst bei uns behoben haben, steht bei ihnen unbehoben:
`f_post_collision` wird für die **ganze** Stufe alloziert, sobald sie *irgendeine* Randzelle hat
(`blocks.jl:148-152`) — ein dichtes Feld für einen dünnen Bedarf. Das ist unser E2-Befund
(dichter Glättungsindex, 593,7 MB bei 2,1 % Belegung), nur in 27-float-Größe.

**Ein Entwurfsfehler, den wir ausdrücklich nicht übernehmen:** ihre Kräfte entstehen, indem
LBM-Daten **zurück auf die STL-Dreiecke** abgebildet werden. Genau das ist in diesem Projekt
verboten und aus gutem Grund — nach der Voxelierung gibt es zwei Wahrheiten, und die Wand muss
aus dem Voxelkörper folgen.

---

## 4 · Drei Befunde in ihrem Code

### 4.1 Das „adaptive ω₄ (Geier 2017)" ist konstant 0,01

`kernel_cumulant.jl:321-324`:

```julia
om1_m2 = omega1 - 2.0f0
denom_adaptive = lambda_param * omega1 + 1.0f-10
A_val = -(om1_m2 * om1_m2) / denom_adaptive
omega4_diag = clamp(A_val, 0.01f0, 2.0f0)
```

`omega1 = 1/max(tau_eff, 0.500001)` ist strikt ≤ 2, also ist `(omega1-2)²` ≥ 0 und `A_val` mit
positivem `lambda_param` (Vorgabe 1/6) **strikt negativ**. Die Klemme trifft in jedem Fall ihre
Untergrenze. Nachgerechnet über τ = 0,500001 / 0,501 / 0,5000283 / 0,55 / 1,0: A = −4,8e−11,
−4,8e−05, −3,8e−08, −0,109, −5,999 — **immer** → 0,01.

Das beworbene Leitmerkmal ist also eine Konstante, und die Empfehlung aus ihrer eigenen
Fehlerbehebungsliste („Enable `adaptive_omega_4: true`") schaltet nur die Klemme ein. Das kann
ein Vorzeichenfehler bei der Übernahme aus der Arbeit sein oder eine absichtliche Vereinfachung
mit irreführendem Namen; beweisbar ist nur, dass es nicht adaptiv ist.

### 4.2 Die Re-Reihe der Validierung ist in Gittereinheiten EIN Lauf

`physics_scaling.jl:101` klemmt: `tau_fine = max(tau_fine_computed, TAU_MIN)`, TAU_MIN = 0,501.

Die Fälle `*_Re1e5`, `*_Re2.5e5`, `*_Re4e5`, `*_Re7.5e5` haben **identisch**
`surface_resolution: 75`, `num_levels: 7`, `u_lattice: 0.08`, `ramp_steps: 2000`,
`inlet_turbulence_intensity: 0.0` — und das berechnete τ liegt bei 0,50018 / 0,50007 / 0,50005 /
0,50002, wird also in **allen vier** auf exakt 0,501 geklemmt. In Gittereinheiten ist das
dieselbe Rechnung; nur die physikalische Geschwindigkeit der Nachrechnung unterscheidet sich, und
Cd ist dimensionslos.

Beim Würfel sieht man es unmittelbar: Cd = **1,060 in allen vier Fällen**. Ihr Bericht liest das
als „confirming Re-independence". Es bestätigt die Klemme.

Bei der Kugel streuen dieselben vier numerisch identischen Läufe über **0,2947 bis 0,3775**
(±12 % um den Mittelwert), die Momentanwerte über 0,246 bis 0,498. Ihre berichtete
„Widerstandskrise" bei Re = 4e5 (0,2947 gegen 0,3775) liegt **innerhalb** dieser Eigenstreuung.

**Und die Klemme kostet Physik:** am Fahrzeug entspricht unser τ = 0,5000283 einem
ν_lat = 9,4e−6. Auf 0,501 geklemmt wären es 3,33e−4, also **Faktor 35 zu viel Viskosität**. Ihre
Bauweise rechnet an diesem Punkt bewusst die falsche Zähigkeit und fängt sie mit Wandmodell und
WALE auf. Unsere rechnet die richtige und kämpft mit der Stabilität. Beides ist eine
Entscheidung, und unsere ist nicht die schlechtere — aber sie ist teurer.

### 4.3 Ihre Ma-Spalte ist nicht die simulierte Mach-Zahl

Die Validierungsberichte führen Ma bis 4,37 und schalten „Kompressibilitätskorrektur ab
Ma ≥ 0,44" ein. Diese Spalte ist U_phys/343 m/s. Die Mach-Zahl, die der Löser integriert, ist
u_lattice/c_s = 0,08·√3 = **0,1386 in allen neun Fällen** — u_lattice ist in jeder Konfiguration
0,08. Deshalb sind Re = 1e7 und Re = 1e8 auf vier Stellen identisch (0,2936 / 0,2362 / 0,4340);
ihr Bericht nennt das „unphysical", es ist aber genau dieselbe Gitterrechnung.

Für uns ist das ein Seitenblick mit Wert: unsere Ma = u_lat·√3 = 0,1299 ist die Zahl, die wirklich
integriert wird, und der Kompressibilitätsfehler sitzt dort. Unsere Rahmung ist richtig, ihre
verwechselt zwei Mach-Zahlen.

---

## 5 · Was ihre Validierung uns über unsere eigene Lücke sagt

Ihr bestes und ihr schlechtestes Ergebnis sind lehrreich, **weil sie beide von derselben
Bauweise kommen**:

| Fall | ihr Cd | Referenz | Abweichung |
|---|---:|---:|---:|
| Würfel, scharfkantig, Re ≥ 1e4 | 1,060 | 1,05 (Hoerner/Blevins) | **+0,9 %** |
| Kugel, glatt, Re = 1e5…2,5e5 | 0,369…0,378 | ≈ 0,47 | **−20 bis −22 %** |
| Kugel, Re = 1e6 | 0,499 | ≈ 0,18 | **+177 %** |

Scharfkantige Ablösung trifft ein Gleichgewichts-Wandmodell auf 1 %. Ablösung, deren Ort von der
Grenzschicht abhängt, trifft es nicht. Das ist eine **unabhängige** Stütze für die Deutung
unserer offenen Hauptfrage: unser Cd liegt bei 94,3 % von OF13, unser Cz bei 74,1 %. Der
Abtrieb lebt an Unterboden und Heck, also dort, wo der Ablöseort von der Grenzschicht bestimmt
wird — genau die Klasse, in der dieser fremde Löser um 20 bis 177 % daneben liegt.

**Zu ihrer Ehre:** sie schreiben es hin. Ein Validierungsbericht, der −35 % ausweist und
benennt, warum, ist mehr Redlichkeit als in diesem Feld üblich.

---

## 6 · Verdikt

**Nicht übernehmen:** die Architektur. Speicher je Zelle Faktor 12, Bandbreitenausnutzung 7,5 %
gegen 69 %, das Wandmodell gröber als unseres, Kräfte über den verbotenen STL-Rückgriff, kein
Prüfpunkt. Unser 4-mm-Nahfeld ist in ihrem Speicherbild nicht lauffähig.

**Übernehmen prüfen, in dieser Reihenfolge:**

1. **Cauchy-Schwarz-Schranke auf die Nebendiagonalen des zweiten Moments** — billig, prinzipiell,
   greift vor unserer Geschwindigkeitsklemme, die nachweislich feuert.
2. **O(u⁴)-Gleichgewicht** — erst nach dem Ergebnis des u_lat-A/B, dann als der Weg, den
   Laufzeitgewinn ohne den Ma²-Fehler zu behalten.
3. **Kumulanten-Kollision** — der große Hebel gegen τ → 0,5, aber aus Geier 2017 gebaut, nicht
   aus diesem Repo, und gegen D3Q19 neu abgeleitet.
4. **WALE** — nur im Bewusstsein, dass es TODO 2 (u seltener schreiben) die Grundlage nimmt.

**Mitnehmen ohne Code:** ihr τ_min = 0,501 ist eine Außenmarke. Ein unabhängig gebauter
Automobil-LBM-Löser **weigert sich**, unterhalb τ = 0,501 zu rechnen. Wir rechnen bei 0,500028,
also 35-fach näher an der Grenze, und alle unsere Stabilitätsbauten der letzten drei Wochen sind
der Preis dafür. Das ist keine Fehlermeldung — aber es ist der erste externe Maßstab dafür, wie
weit draußen dieser Betriebspunkt liegt.

**Und eine Außenmarke zu u_lat:** ihre Vorgabe ist `u_lattice: 0.08`, ihr dokumentierter Bereich
„0,01 (precision) bis 0,08 (fast)". Unsere 0,075 liegt damit schon am oberen Ende dessen, was
dieser Löser als schnell ausweist. Das ist Fremddokumentation, keine Messung, und es ersetzt
unseren A/B nicht — aber es ist ein Hinweis, 0,100 nicht für selbstverständlich zu halten.
