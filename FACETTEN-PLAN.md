# C1b — Facetten-basierte Wandbehandlung: Implementierungsplan

Planungsagent 2026-08-15 (Iron Rule 2), auf Stand 2018a64. Heikos Entscheidung: zellbasiert,
nicht aus der STL. Abnahme: gemessener y⁺-Median 1122 → ~140 bei deutlich schmalerer Verteilung.

## A. Architektur-Entscheidungen

**A1. Fit host-seitig, einmalig — nicht im Kernel.** Geometrie statisch, flags liegen nach
voxelize+read host-seitig vor. Eigenwertproblem je Wandzelle in double auf dem Host: unit-testbar,
deterministisch; im Kernel wäre es ein FP32-Eigenlöser unter -cl-finite-math-only (NaN = UB).
Kosten: ~10⁶ Fits à 5³, Sekunden, einmal pro Lauf.

**A2. Datenlayout: kompakte SoA-Liste + uint-Indexfeld über die enge Fahrzeug-BBox.** Je Facette:
n̂ (3×float), y_w, gepackte Klassen-/Achsen-uchar, Zellindex (~24–32 B → ~25–30 MB); Indexfeld
0xFFFFFFFF = keine Facette (~560 MB @ 4 mm Vollgitter, akzeptiert; dd-Nahfeld kleiner). F-BBox-
Mechanik wiederverwenden. **Keine neuen Flag-Bits** — object_force vergleicht exakt flags==0x41.

**A3. Punktbasis: Halfway-BB-Linkmittelpunkte, nicht Solid-Zellzentren.** Stützpunkte = Mittelpunkte
aller geschnittenen Links (Fluid→Solid, 18 Richtungen) im 5³-Umfeld — das IST die effektive Wand
des Lösers, kein dx/2-Versatzproblem. Regressionsanker: am parallelen Kanal liefert das exakt
n̂ = ê_z, y_w = 0,5 → Facetten-WFB muss dort die z-WFB reproduzieren.

**A4. Total-Least-Squares (Kovarianz + kleinster Eigenvektor), kein Höhenfeld-Fit.** Höhenfeld
divergiert an senkrechten Wänden. c = Punktmittel, M = Σ(p−c)(p−c)ᵀ, n̂ = EV zu λ_min (3×3-Jacobi,
double, Host). y_w = n̂·(x_Zelle−c) > 0 (Orientierung ins Fluid), Gegenprobe gegen Solidnachbar.
**Konditionsklassen (markieren + zählen, nicht still rechnen):** K1 N<6; K2 Kante λ_min/λ_mid
über Schwelle; K3 Linien-Degeneration λ_mid/λ_max unter Schwelle; K4 y_w außerhalb [0,2; 2,0].
Schwellen aus den Stufe-1-Histogrammen, nicht geraten. Markierte Zellen behalten reinen BB.
V1-Kontrast (Matyka-Rauschen, V1 kernel.cpp:2277–2337): nur 18 diskrete Richtungen, ungewichtete
Glättung, kein Wandabstand, kein Konditionsmaß.

**A5. Kopplung: achsweise verallgemeinerter Free-Slip-Tausch + ½τ_w auf der dominanten Achse —
kein Guo-Quellterm als Ersatz.** Der Kanal-Mechanismus ist fhn-Registermanipulation zwischen
load_f und Kollision → Esoteric Pull strukturell unkritisch; x/y-Wände sind mechanisch analoge
Diagonalpaare. Für schräge Normalen gibt es keine gitterexakte Spiegelung → Facette liefert
τ_w-Betrag/-Richtung/y_w (Physik), angewandt auf Achse a = argmax|n̂_a| (Gitter). Guo-Quellterm
scheidet aus: müsste den BB-Widerstand subtrahieren, der über Orientierungen um Faktor 3,5 streut
(V1s Doppelzählungsklasse); der Tausch ENTFERNT den BB-Widerstand strukturell, massenerhaltend.
Guo-artiger Term später allenfalls als ERGÄNZUNG für den achsprojektions-verschluckten Anteil.
C7 gilt: Senke, kein wandwärtiger Impulstransport.

**A6. Glättung zweistufig.** Host: Normalen-Glättung sofort (1–2 Jacobi-Pässe, flächengewichtet
w_k = Anzahl geschnittener Links; renormieren, y_w neu). Laufzeit-τ-Glättung NIE über Nachbar-u
im stream_collide (Race-Klasse Audit-Befund 8) — separater Listen-Kernel facetten_tau VOR
stream_collide (liest Vorschritt-u/ρ, glättet flächengewichtet, optional EMA 10²–10³ Schritte);
zugleich der vorbereitete Platz für die Relativgeschwindigkeits-Erweiterung.

**A7. Schalter/Zähler/Kontrollarm.** CFD_FACETTEN (0/1/2 wie CFD_WANDFUNKTION), Statik an allen
6 Konstruktorstellen, Emission nur wenn gesetzt → Kontrollarm bitgleich (Nachweis CFD_DUMP_CL-Diff
+ Kurzlauf-Bitvergleich). Puffer nur bei gesetztem Schalter binden (Muster tile_slot/
alloc_coupling_planes, harter Fehler bei run() ohne Bindung). rho_clamp_hits 8→12 Slots:
[7] Wirkpfad (t%100), [8] τ-Klemme, [9] u_t≈0-Skip, [10] Achskonflikt (gegatet), [11] frei.
Einmalzähler (K1–K4, Kontaktfleck-Ausschlüsse) host-seitig ins Startlog.

**A8. Abnahme-Messgröße: modell-eigener τ-Akkumulator.** AUDIT-Befund 1 verallgemeinert sich:
update_force_field setzt Reflexion voraus → Impulsaustausch-Weg an behandelten Zellen ungültig
(betrifft messe_yplus + Cd/Cz-Reibungsanteil). Facettenpfad schreibt τ_w je Facette in eigenen
Puffer (1 Zelle = 1 Work-Item, keine Atomics); y⁺-Abnahmehistogramm kommt aus DIESEM Puffer.
messe_yplus bleibt Vorher-/Kontrollarm-Messung, bekommt Ungültigkeits-Kennzeichnung.

## B. Stufen

**Stufe 0 — Werkzeuge (C2).** CFD_DUMP_CL/CFD_DUMP_DEFINES aus V1 portieren, env_on/env_f/env_u
nach utilities.hpp. Messpunkt: Kontrollarm-DUMP-Diff leer.

**Stufe 1 — Host-Facettenbau + reine Diagnose, kein Kernel-Eingriff (C3).** baue_facetten() in
setup.cpp: Linkmittelpunkte, TLS, Konditionsklassen, Orientierung, Normalen-Glättung; Histogramme
(Kondition/y_w/Normalen). Einmalige STL-Gegenprobe (Winkelhistogramm, NUR Diagnose). τ-Diagnose
host-seitig (Slab-Readback, Spalding in double als Referenzimplementierung): τ_Modell vs
τ_gemessen vs y⁺. Reihenfolge: NACH Kontaktflächen-Übergabe + Void-Fill; nur Solidnachbarn mit
flags==0x41 (Fahrbahn/Latsch = Ausschlussklasse). Abbruch: >~30 % markiert → erst 3³/5³/7³-A/B
und Schwellenarbeit. Erwartung Normalenfehler-Median ≤ ~10–15° nach Glättung.

**Stufe 2 — Kernel-Eingriff dominante Achse, Äquivalenz am Kanal.** stream_collide-Block nach dem
WANDFUNKTION-Block, gleiche Gates; Lookup übers Indexfeld, Y = |u_t|·y_w/ν (def_wf_Y
parametrisiert), Spalding (Iterationszahl als Define — bei Y~10⁴ sind 3 Schritte ~2,3 % zu klein),
τ-Klemme, achsweiser Tausch+½τ, τ-Akkumulator. Host: 6 Statiken, Emission, alloc_facetten,
12 Slots, Warnungen; FACETTEN gleichzeitig mit WANDFUNKTION = harter Fehler; D>1 = harter Fehler.
Messpunkte: (a) CPU: Unit-Tests synthetischer Geometrien (Ebene exakt, 45°-Treppe, Kugelvoxel),
DUMP-Diff leer. (b) iGPU-Kanal: CFD_FACETTEN=1 reproduziert die z-WFB-Zahlen (Ziel bitnah);
Kontrollarm bitgleich. (c) iGPU-Kugel: drei Arme AUS/2/1, Wirkpfad Ist=Soll, Klemmen ~0.

**Stufe 3 — Gekippter Kanal: der Geometrie-Messpunkt.** Kanal um x-Achse verkippt
(CFD_KANAL_KIPP, Voxel-Treppenwand) — misst ISOLIERT den geometrischen Rauheitsanteil. Abnahme
RELATIV: gekippt+Facetten erreicht c_f des parallelen Kanals mit z-WFB (0,00107–0,00114, ±~10 %),
NICHT Lee&Moser (−68 %-Restproblem ist die andere Baustelle). BB-Basislinie gekippt dokumentiert
den Treppenzuschlag vorher.

**Stufe 4 — Glättung + Nebenachsen (Ausbau).** facetten_tau-Vorkernel; Zweitachsen-Tausch mit
statischer Konfliktregel (jedes Linkpaar genau ein Bearbeiter, Slot 10). Bringt (b) nichts →
weglassen. Kugel-Regression, Leistungsmessung (~5–8 % erwartet).

**Stufe 5 — Fahrzeug-Abnahme (B70, ≤2,5 h je Lauf).** Arme AUS/2/1 + SGS_WANDFREI-A/B; dann
dd-Fall (jede Instanz baut Facetten aus eigenen flags — kein Laufzeit-Halo nötig). Abnahme:
y⁺-Median aus dem τ-Akkumulator 1122 → ~140, deutlich schmaler; geometrisches y⁺ ~137 bleibt;
Bodenstreifen-Zeitreihe nicht schlechter; Cd/Cz-Verschiebung mit C7-Notiz DAVOR. Abbruch:
RHO_CLAMP > 0 oder divergierende Kraftreihen.

## C. Risiken → Gegenmaßnahmen

1. object_force-0x41-Falle → keine Flag-Bits; Zellzählung vor/nach Facettenbau als Assertion.
2. Impulsaustausch ungültig (AUDIT #1 verallgemeinert) → τ-Akkumulator primär, CSVs gekennzeichnet,
   Kugel-A/B beider Cd-Wege.
3. Esoteric Pull → nur fhn-Register (wie Kanal-WFB); fi-Direktmanipulation verboten.
   update_fields-Latent-Kommentar erweitern.
4. dd/Halo → getrennte Facettensätze je Instanz; D>1 hart abgewiesen; Facettenbau NACH
   Kontaktflächen-Übergabe/Void-Fill.
5. TYPE_MS/Relativgeschwindigkeit → Facetten nur bei dominantem 0x41-Nachbarn; MS-Guard;
   τ von Anfang an als f(u_Fluid − u_Wand) mit u_Wand=0 strukturiert.
6. SUBGRID → ν molekular im Spalding; SGS_WANDFREI-Arm fest eingeplant; FACETTEN nicht an
   SUBGRID gekoppelt (Befund-2-Lehre).
7. FP16C → Tausch exakt, ±½τ klein; eine FP32-Sprosse am Kugelfall.
8. Kanten/dünn → K1–K4 markieren, BB bleibt, Anteile im Startlog, Schwellen aus Messung.
9. u[]-Race → Glättung nur im Vorkernel (Befund-8-Klasse).
10. Indexfeld-Speicher (~560 MB @ 4 mm) → akzeptiert (~4 %); Rückfallebene engere BBox.
11. Zähler-Überlauf → neue Slots 7/10 gegatet; 8/9 wie 3/4 (Ist≠Soll fällt auf).

---

# Revision nach adversarialer Gegenprüfung (2026-08-15, Stand 2cfdc43)

Urteil: Architektur A1–A4/A6–A8 und Stufenlogik halten; ZWEI Befunde treffen den Kernel-Kern
und sind eingearbeitet. Der Gegenprüfer hat die Esoteric-Pull-Tausch-Identität für x/y-Wände
am Code VERIFIZIERT (load_f/store_f lesen/schreiben je Paar denselben Slot; Tausch in Registern).

## R1 (KIPPT → eingearbeitet): linkweises Tauschpaar-Gate ist PFLICHT.
Der Kanal-WFB brauchte kein Paar-Gate, weil an gitterparallelen Wänden alle vier Diagonal-
ursprünge zwingend solid sind — GEOMETRIEZUFALL. An jeder Treppen-Eckzelle hat z. B. das
y-Paar (11,18) einen FLUIDEN Ursprung; ungegatet tauscht man dort regulär gestreamte DDFs
gegen Reflexionen — destruktiv, jeder Schritt, jede Schräge. Regel: Paar (9,16) nur wenn
flags[j[10]] UND flags[j[15]] solid; (11,18) nur wenn j[12] UND j[17] solid; Decke/x/y analog.
±½τ NUR auf tatsächlich getauschte Paare (sonst Addition auf ungetauschtem BB =
V1-Doppelzählungsklasse). Der 45°-Treppen-Unit-Test prüft DDF-INTEGRITÄT an fluiden
Ursprüngen, nicht nur τ-Werte.

## R2 (KIPPT → eingearbeitet): Flächenfaktor 1/|n̂_a| sofort in Stufe 2.
An der 45°-Treppe trägt eine Eckzelle pro Periode den Tausch, wahre Wandfläche √2·dx² —
ohne Faktor fehlen 29,3 % des integrierten Widerstands, Stufe 3 wäre zum Scheitern verabredet.
Angewandtes τ = τ_w/|n̂_a| macht das Flächenintegral über die Monolage exakt; am parallelen
Kanal ist der Faktor exakt 1,0 → Bitgleichheit unberührt. STRUKTURELLE GRENZE (ehrlich
benannt): die τ-Komponente ENTLANG der dominanten Achse (Hangauf-Strömung, Windschutzscheibe
bis sin45° = 71 %) kann der dominante-Achse-Tausch nicht anwenden; der gekippte Kanal
(Strömung parallel zu den Stufenkanten) misst diese Lücke NICHT — sie bleibt als bekannte
Grenze im Plan und wird ggf. in Stufe 4 (Nebenachsen) adressiert.

## Auflagen (alle übernommen):
1. **Stufe 1:** y_w-Systematik an der 45°-Treppe (Eckzellen ~0,35–0,39 statt 0,5; Punktwolke
   nicht eben, Streuung ~0,35 dx) als BEKANNTE Zahl in die Diagnose; logarithmische Dämpfung:
   ±40 % y_w ≈ ±9–10 % τ_w — halbiert das Stufe-3-Budget, steht jetzt hier.
2. **Cd/Cz-Auslesepfad:** Druckanteil per zellweiser n̂-Projektion von F beim Auslesen,
   Reibung aus dem τ-Akkumulator; der rohe Impulsaustausch misst an getauschten Links ein
   PHANTOM (weder BB noch Modell-τ, Größenordnung Rauwand). Kugel-A/B beider Wege ist
   PFLICHT-Abnahmekriterium mit Zahl, keine bloße Gegenmaßnahme.
3. **Akkumulator-Hygiene:** τ nur für Zellen akkumulieren, die mindestens ein Paar wirklich
   getauscht haben — sonst schönt das y⁺-Histogramm unbehandelte Zellen.
4. **Stufe 2b Bitgleichheits-Vorschrift:** bei n̂=ê_z exakt derselbe Ausdrucksbaum wie die
   z-WFB (Konstante über identische to_string-Kette, τ_a = −def_wf_tau·tw·u_a/ut,
   Paarreihenfolge (9,16) vor (11,18)); andernfalls Kriterium ehrlich als "bitnah mit
   Letzt-Bit-Drift" + Toleranzband. Fit liefert am parallelen Kanal EXAKT 0,5 (verifiziert).
5. **Stufe 3 festgelegt:** iGPU; Kippwinkel 45° UND ein Nicht-Tie-Winkel (~26,6° = atan 0,5,
   Treppenperiode 2) — 45° ist der argmax-Tie-Fall, Tie-Break deterministisch (kleinste
   Achsnummer); Treppenperiode muss Ny teilen; c_f-Normierung auf BENETZTE Fläche definiert.
6. **Einhängepunkt:** baue_facetten() NACH set_bcs (Zellcensus sieht den Endzustand);
   host-flags sind dort aktuell (verifiziert: nach Readback nur host-seitige Änderungen).
7. **dd-Speicher:** Indexfeld Fahrzeug-BBox ≈ 0,63 GB (nicht 560 MB "Vollgitter" — der wäre
   2,0 GB); B70-Budget fahrzeug ≈ 29,5 GB von 32 → passt; "dd-Nahfeld kleiner" war FALSCH
   (gleiche BBox); dd-Bilanz vor Stufe 5-dd am realen Log nachweisen.
8. Spalding-Iterationszahl als Define, Default je Fall (y⁺~140 → 3 Schritte reichen ~0,1 %).
9. Stufe 5 ist eine SERIE von ≥6 B70-Läufen à ≤2,5 h — als Serie einplanen.
