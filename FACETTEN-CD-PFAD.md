# C1b: Cd/Cz-Auslesepfad unter Facetten (Plan, 2026-08-15)

Ich habe den Stand im richtigen Repo (/home/heiko/CFD/FluidX3D-v2, f3af022) gelesen: `apply_facette`/`fac_paar` (kernel.cpp:1650–1733), `update_force_field`/`object_force` (kernel.cpp:2546–2619), `alloc_facetten_domain` (lbm.cpp:372–407), `messe_yplus` als F-BBox-Lesemuster (setup.cpp:736–789), Kanal-Report (setup.cpp:689–703), Kugel-Report (setup.cpp:1235–1247), FACETTEN-PLAN.md Auflage 2 und FACETTEN-STUFE2.md.

## Entscheidungen mit Begründung

**E1 — Hybride Zerlegung: Druck SOLID-seitig aus F, Reibung FLUID-seitig aus dem Akkumulator.**
Nicht „entweder–oder": F lebt an den Solidzellen und seine NORMALkomponente ist dort erster Ordnung gültig (Nachprüfer-Befund: Tausch kehrt c_normal weiter um, ±½τ antisymmetrisch → Netto-Normal 0); τ lebt an den Fluid-Facettenzellen. Das Schlüsselargument gegen jede Mischung: der Flächenfaktor `faca = 1/|n̂_a|` (R2) ist genau so gebaut, dass das τ-Integral über die Facetten-Monolage die **gesamte wahre Wandfläche** der Zellsäule abdeckt. Behielte man an kontaminierten Solidzellen irgendeinen Tangentialanteil von F, wäre das eine Doppelzählung; verwirft man ihn, ist die Reibung vollständig und einfach durch den Akkumulator repräsentiert. Also: `Beitrag = (F·n̂)n̂` an kontaminierten Solidzellen, voller F sonst, plus Reibungsvektor aus dem Akkumulator.

**E2 — Kontaminationstest über die 18 Nachbarn, Indikator `fac_tau_n>0`, bewusst als Obermenge.**
Eine Solidzelle gilt als kontaminiert, wenn unter ihren 18 Nachbarn ≥1 **aktive** Facettenzelle mit `fac_tau_n>0` liegt. Da Flags statisch sind, sind die Paar-Gates zeitkonstant — `fac_tau_n` ist entweder 0 oder ≈ Schrittzahl, der Indikator ist deterministisch und misst die Ground Truth des Kernels statt einer zweiten Host-Buchführung. Die 18er-Nachbarschaft ist eine Obermenge der tatsächlich getauschten Diagonalpartner (das offene Paar (9,16) berührt j[10]/j[15], das geschlossene (11,18) andere Solidzellen) — an Treppenkanten wird dadurch etwas gültige BB-Tangentialkraft mitverworfen. Akzeptiert: Kantenzellen sind ohnehin überwiegend K2-markiert (vom Facettenpfad ausgeschlossen), und die Alternative wäre linkaufgelöste F-Buchführung = neuer Kernel. Optionale Schärfung (Host-Gate-Rekonstruktion nach T1-Logik) als Reserve notiert.

**E3 — n̂ an Kanten: normiertes Mittel der Nachbar-Facettennormalen, uniform gewichtet.**
Kein zweites Fitverfahren, keine Gewichtung nach Tauschaktivität (die würde n̂ an die Phantomstruktur koppeln). Die Normalen sind bereits geglättet (A6); benachbarte Facetten differieren wenig. Degeneriert das Mittel (|Σn̂| < 0,5, z. B. Ein-Zellen-Spalt mit Gegennormalen) → **voller F** als konservativer Rückfall + eigener Zähler `n_unklar` im Report (Kugel-Erwartung: 0). Quelle für n̂ ist der **Host-Spiegel von `fac_geo`/`fac_idx`** (Memory hält die Host-Kopie, seit `write_to_device` unverändert) — der Auslesepfad benutzt exakt die Daten des Kernels, keine FF-Vektor-Replikation, deren Reihenfolge divergieren könnte.

**E4 — Solidzellen ohne (aktiven, tauschenden) Facettennachbarn: voller F.** Dort gilt reiner BB, der Impulsaustausch ist unverfälscht; der Akkumulator hat dort dank Auflage 3 (nur `getauscht>0` akkumuliert) nie beigetragen — keine Lücke, keine Doppelzählung.

**E5 — Reibungsrichtung: Option (a), Kernel akkumuliert die ANGEWANDTE Kraft komponentenweise; `fac_tau` wird 4 float/Facette im selben Puffer.**
Option (b) (Richtung host-seitig aus dem aktuellen u) scheidet aus: an der Kugel ist die Nachlaufströmung instationär, die Richtung zur Auslesezeit repräsentiert nicht das Zeitmittel, und u am Abtastpunkt (vor Korrektur, im Register) ist host-seitig gar nicht exakt reproduzierbar. Option (a) ist exakt und billig: `fac_paar` bekommt einen `float*`-Sammelparameter und addiert im Gate `-taut` (= Wandkraftkomponente: der Tausch selbst ist tangential-neutral — spiegelnde Reflexion —, das Paar überträgt relativ zum Tausch genau τ_t an das Fluid, die Wand erhält −τ_t = +def_fac_tau·twe·ut_c/ut; Vorzeichen am Kanal geprüft: +x bei ut_x>0). Layout: `fac_tau[4·fid+0]` = Σ physisches tw (y⁺-Semantik unverändert, F5), `[1..3]` = Σ angewandte Wandkraft (mit def_fac_tau **und** faca, je Paar gegatet — im Arm 2 ehrlich 0, bei halboffenem Gate nur die offene Komponente). Kosten: +8 B/Facette (Kanal ~0,1 MB, Kugel ~MB-Bereich), 2–3 zusätzliche Global-Adds pro Facettenzelle/Schritt. Kernel-Signatur unverändert (gleicher Puffer, nur größer) — `fac_param_pos`-Mechanik unberührt. Bitgleichheit: die Akkumulator-Writes speisen nie in fhn zurück → Feld-Hash bleibt; Kontrollarm über Emission-Gating (#ifdef FACETTEN) unberührt.

**E6 — Zeitfensterung: Reibung als exaktes Fenster-Mittel per Akkumulator-Delta, Druck per Abtastung.**
`fac_tau` wird am Warmup-Ende und am Laufende gelesen (Readback ist nicht-destruktiv); Differenz/Schrittzahl = exaktes Fenstermittel ohne Abtastfehler. Der Druckanteil braucht F-Readbacks; Default: gleiche Kadenz wie `object_force`-Samples, per `CFD_FAC_CD_EVERY` ausdünnbar, falls der Voll-Domänen-F-Read an der Kugel (keine F-BBox gesetzt, Risiko 3 des Stufe-2-Plans) zu teuer wird. Rückfallebene: `set_force_bbox` um die Kugel+4 (optionaler Schritt, mit Census-Assertion).

**E7 — MS-Randfall (Frage 4): der Rückfall auf vollen F ist KORREKT.**
Klasse-64-Facetten fehlen in `fac_idx` (alloc filtert klasse!=0), tauschen also nie; zur Laufzeit sperrt zusätzlich der MS-Zellgate. An diesen Fluidzellen läuft reiner (bewegte-Wand-)BB, dessen Impulsaustausch F genau abbildet — der Rückfall reproduziert lokal exakt den AUS-Arm. An der Kugel praktisch leer (Kugel schwebt, 0x41 berührt die bewegten TYPE_S-Wände nicht; object_force zählt ohnehin nur flags==0x41), aber die Logik trägt den Fall strukturell.

## Nummerierte Schritte

1. **kernel.cpp:1664–1668 (`fac_paar`):** Signatur um `float* fkraft` erweitern; im Gate zusätzlich `*fkraft -= taut;`. **kernel.cpp:1670–1733 (`apply_facette`):** lokale `float fk_x=0,fk_y=0,fk_z=0`; an den 12 Aufrufstellen (1699–1729) die zur τ-Komponente passende Variable durchreichen; Akkumulation 1730 ersetzen durch `fac_tau_acc[4u*fid+0]+=tw; [+1]+=fk_x; [+2]+=fk_y; [+3]+=fk_z; fac_tau_cnt[fid]+=1u;` (weiter nur bei `getauscht>0`).
2. **lbm.cpp:381 (`alloc_facetten_domain`):** `fac_tau = Memory<float>(device, 4ull*aktiv)`; Initialisierung :393 auf 4 Slots; MB-Ausgabe :406 anpassen. Kommentar lbm.hpp:168 (Layout 4 float: tw, Fx, Fy, Fz) nachziehen.
3. **Bestehende `fac_tau`-Leser umstellen (grep-Pflicht `fac_tau[`):** setup.cpp:698–702 (Kanal, Index `4*i+0`), :1242–1245 (Kugel, dito). T2 (:2543f) liest nur `fac_tau_n` — unverändert.
4. **Neue Host-Funktion `kraft_facetten(LBM&, Nx,Ny,Nz, uchar marker, ulong fenster_schritte, const std::vector<double>& tau_snapshot, …)`** in setup.cpp direkt hinter `messe_yplus` (Anker :789), nach dessen Muster: `update_force_field()`, `D->F.read_from_device()`, `flags.read_from_device()`, `fac_tau.read_from_device()`, `fac_tau_n.read_from_device()`. Schleife über die F-BBox (Formel :747/:762, identisch zu alloc :397); je Zelle `flags[n]==marker`: 18 FZ_C-Nachbarn (x/y/z periodisch gewickelt wie `neighbors()` im Kernel), über Host-`fac_idx` aktive Facetten finden; kontaminiert ⇔ ≥1 Nachbar-fid mit `fac_tau_n>0` → n̂-Mittel aus Host-`fac_geo`, Beitrag `(F·n̂)n̂` (double-Summe), sonst voller F; Zähler `n_voll/n_projiziert/n_unklar`. Reibung: `Σ_fid (fac_tau[4fid+1..3] − snapshot)/fenster_schritte`. Rückgabe: Druck-float3, Reibungs-float3, Zähler. Guard: `facetten_on==false` → reiner Voll-F-Pfad (neutrale Zuordnungslogik bei 0 Facetten, Frage 3).
5. **Kanal-Verdrahtung (setup.cpp:689–703 erweitern + Zeitschleife :624–657):** beim ersten Chunk mit `step>=n_warm` einmalig `fac_tau`-Snapshot (Readback → `std::vector<double>`) und laufendes Mittel von `f_wirk` ab dort mitführen; am Ende `kraft_facetten(TYPE_S,…)` und Abnahmekriterien K2/K3 (unten) prüfen, mit `print_error` bei Verletzung.
6. **Kugel-Verdrahtung (setup.cpp:1150–1247):** Snapshot beim ersten Sample mit `ts>=t_warmup`; Druckanteil an den Sample-Punkten (Kadenz `CFD_FAC_CD_EVERY`, Default 1) als eigene Zeitreihe, Fenster-Mittel + `block_sem`; Endreport druckt **beide Wege in allen Armen**: Cd_alt (object_force), Cd_neu = Cd_Druck + Cd_Reibung (getrennt ausgewiesen), Zähler n_voll/n_projiziert/n_unklar; forces.csv um Spalten `Cd_neu_druck,Cd_neu_reibung` erweitern (Kadenz-Lücken als leere/NaN-markierte Felder vermeiden → eigene CSV `cd_facetten.csv` ist sauberer). AUS-Arm-Gleichheitsprüfung K4 am letzten Sample (F-Snapshot identisch zu dem, aus dem das letzte `object_force` kam).
7. **Messläufe (Projektregel: Commit VOR erstem GPU-Lauf):** (i) CPU Kanal N=38/316 Schritte, FACETTEN=1: Feld-Hash; (ii) `CFD_DUMP_CL`-Diff Kontrollarm; (iii) iGPU Kanal 20 ETT FACETTEN=1: K2/K3; (iv) iGPU Kugel drei Arme AUS/2/1: K4/K5 + Pflichtzahlen-Dokumentation.

## Abnahmekriterien (mit Zahlen)

- **K1 (Nichtinvasivität):** Feld-Hash(u) des FACETTEN-Arms CPU N=38/316 Schritte bleibt exakt **12755646098055097704**; `CFD_DUMP_CL`-Diff des Kontrollarms (FACETTEN=0) **leer**; Wirkpfad Ist=Soll weiter exakt, Slots 8/9/11 am Kanal = 0.
- **K2 (Kanal-Reibung):** |Reibung_x_neu/(f̄_wirk·δ·2·Nx·Ny) − 1| < **0,01** über das stationäre Fenster (Impulsbilanz-Restfehler ist O(ΔU_b·N/(2u_τ²·Schritte)) ≈ 1e-5, das Band deckt Regler-Restdrift); |Reibung_y|/|Reibung_x| < **0,01**.
- **K3 (Kanal-Druck):** Druckanteil_x am Kanal **exakt 0,0** (n̂ ist als float exakt (0,0,1), (F·n̂)n̂ hat x-Komponente identisch 0 — kein Toleranzband nötig); n_unklar = 0; n_voll = 0 (alle Wandzellen kontaminiert, F6-Wrap).
- **K4 (Kugel AUS-Arm-Neutralität):** |Fx_neu − Fx_objectforce|/|Fx| < **1e-5** am selben F-Snapshot (Host-double-Summe gegen GPU-float-Atomics; Werte identisch, nur Summationsreihenfolge differiert, bei ~10⁴–10⁵ Zellen liegt der Rundungsunterschied unter 1e-6).
- **K5 (Kugel Phantom-Beseitigung, Pflichtzahl):** Alt-Pfad dokumentiert Cd 1,32 (AUS) → 2,80 (Arm 2), Sprung +1,48. Neu-Pfad: Cd_neu(Arm 2) − Cd_neu(AUS) ∈ **[−0,4, +0,1]** — physisch ist im Tausch-only-Arm WENIGER Widerstand plausibel (Slip verzögert Ablösung), ein Sprung nach oben in der Größenordnung des Phantoms disqualifiziert. Kein harter Fehler (Arme sind physikalisch verschieden), aber Pflicht-Dokumentation beider Wege in allen drei Armen inkl. Druck/Reibungs-Split; Arm 2 muss Cd_Reibung_neu **exakt 0** liefern (def_fac_tau=0 steckt in den akkumulierten Komponenten).

## Risiken

1. **`fac_tau`-Layoutwechsel** (1→4 float) bricht stille Leser — grep-Pflicht; T2 ist unbetroffen (nur `fac_tau_n`).
2. **Obermenge-Kontamination** verwirft an Treppen-/Kantensolidzellen etwas gültige BB-Tangentialkraft (E2) — an der Kugel klein (Kanten-Facetten sind K2-ausgeschlossen), am Kanal exakt null; als bekannte Grenze dokumentieren, Host-Gate-Schärfung als Reserve.
3. **F-Readback-Kosten Kugel** (F-BBox = volle Domäne): pro Sample ~12 B/Zelle; erst messen, dann ggf. `CFD_FAC_CD_EVERY` oder `set_force_bbox` Kugel+4 (mit Census-Assertion, Risiko-3-Rückfallebene des Stufe-2-Plans).
4. **Float-Akkumulator mit Vorzeichen:** Komponentensummen ~τ·Schritte ≈ O(1) — unkritisch; bei künftigen 10⁶⁺-Schritt-Läufen Fenster-Snapshots ohnehin Pflicht (Delta-Lesart begrenzt die Absorption).
5. **Index-Formel-Drift Host/Kernel:** fbi-Formel existiert dreifach (kernel `f_bbox`, alloc :397, messe_yplus :762) — die neue Funktion MUSS die :762-Form wörtlich übernehmen; ein Formel-Kommentar-Querverweis genügt, keine vierte Variante erfinden.
6. **Druck-Abtastung vs. Reibungs-Fenstermittel mischen zwei Schätzer** — im Report getrennt ausweisen (Cd_Druck mit Sample-Zahl + Block-SEM, Cd_Reibung exakt), nicht zu einer scheingenauen Zahl verschmelzen.

### Critical Files for Implementation
- /home/heiko/CFD/FluidX3D-v2/src/kernel.cpp (fac_paar 1664–1668, apply_facette 1670–1733)
- /home/heiko/CFD/FluidX3D-v2/src/setup.cpp (messe_yplus-Muster 736–789, Kanal-Report/Zeitschleife 624–703, Kugel-Zeitschleife/Report 1150–1247)
- /home/heiko/CFD/FluidX3D-v2/src/lbm.cpp (alloc_facetten_domain 372–407, object_force 502–513)
- /home/heiko/CFD/FluidX3D-v2/src/lbm.hpp (fac_geo/fac_idx/fac_tau-Member 166–169, F-BBox-Felder 82)
- /home/heiko/CFD/FluidX3D-v2/FACETTEN-PLAN.md (Auflagen 2/3, R2-Flächenfaktor-Argument — Abnahmereferenz)

---

# Messstand Cd-Pfad (2026-08-15, Commits bis a41759b)

- **K1 bestanden:** Feld-Hash exakt 12755646098055097704 nach dem 4-float-Akkumulator, Tests 3x gruen.
- **K2 bestanden (stationaeres 20-ETT-Fenster, iGPU):** Reibung_x/Kraftbilanz = 0,9955 (Band 1 %),
  nach dem R3-Snapshot-Fix wiederholt: **0,9961**;
  das 0,3-ETT-Kurzfenster zeigte erwartbar 0,26 (Regler-Transiente -- kein Befund).
- **K3 exakt bestanden:** Druck_x = 0,00000000, 14.160 projiziert, 0 voll, 0 unklar.
- **K4 bestanden:** AUS-Arm kraft_facetten Fx == object_force Fx (rel. Abw. 1,5e-7).
- **K5 teilbestanden:** Arm 2 Cd_reibung EXAKT 0; Phantom beseitigt -- neuer Pfad Arm2 vs Arm1:
  0,1502 vs 0,1645 (Delta 0,014 statt Phantom-Sprung +1,48). EINSCHRAENKUNG (Morgen-TODO):
  Druckanteil ist End-MOMENTAUFNAHME, kein Zeitmittel -- fuer belastbare Kugel-A/Bs die
  Druck-Projektion in die Sample-Kadenz heben (CFD_FAC_CD_EVERY, Plan E6). Die Reibung ist
  bereits exaktes Fenstermittel.
- Kugel-Erkenntnis bestaetigt: nur 313 Tauschzellen / 14.356 ohne offenes Paar pro Messpunkt --
  Nebenachsen (Stufe 4) vor der Fahrzeug-Abnahme noetig; der gekippte Kanal misst das kontrolliert.
