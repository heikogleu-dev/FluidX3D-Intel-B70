# Bauplan Kopplungs-Umbau (Heiko-Auftrag 2026-08-21)

Vorbereitet aus vier Planungsagenten (Box, Vorwärtsband, Rückwärtsband, Messplan) und drei
Prüfagenten (Code-Realität, Literatur, Wissensspeicher-Vorgeschichte). **Noch nichts gebaut.**

---

## 0. Heiko-Vorgaben, die alles andere binden

1. **Jeder Mechanismus ausschaltbar**, Default AUS = bitidentisch (kein alloc, kein Enqueue,
   keine Logzeile).
2. **Reihenfolge im Fernfeld-Zeitschritt: erst die Aufprägung nah→fern, DANN der
   Moving-Floor-Fix darüber.** Heute ist es umgekehrt — `do_time_step` (lbm.cpp:1576-1579)
   läuft `stream_collide → boden_eq → einlass_eq → schale_blend`, der Blend gewinnt also über
   das Bodenband. **Muss getauscht werden: `schale_blend` VOR `boden_eq`.** Konsequenz: der
   `z_lo`-Ausschluss im Listenbauer wird damit überflüssig — der Boden-Fix überschreibt die
   Bandzellen im Bodenband ohnehin. Beides zusammen ändern, sonst wirkt der Ausschluss doppelt.
3. **Voxelisierungsunterschied gehört in die Rückwärtskopplung.** Der 16-mm-Körper verdrängt
   mehr als der 4-mm-Körper (bei 32/8 mm gemessen: +15,5 % Volumen, +5,48 % Stirnfläche; bei
   16/4 mm neu zu erheben). Grobzellen, die grob solid sind, überdecken feine Blöcke mit Fluid —
   deren Information wird heute verworfen (Kernel-Guard `TYPE_S → return`).
   **`CFD_N2F_SCHALE_MITTEL=0` (Punktwert) für den Band-Arm hart sperren.**
4. **Fahrzeuginnere Zellen** fallen per Konstruktion raus (`d==0` = Saat = solid). Eingeschlossene
   Fluid-Taschen (Kabine, Radhaus) haben d=1 und bleiben drin — sie werden vom Fluidleer-Census
   gefangen. **Census pro Lage ist Pflicht**, sonst wirkt der Arm schwächer als sein alpha sagt.
5. **Near-Box verbreitern** (siehe §1) — ein paar Zellen Puffer ohne jedes Blending.
6. **cos²-Profil später**, als eigener Messarm gegen linear. Ausnahme: wenn es beim Bau ohnehin
   nur eine Gewichtstabelle ist, gleich als Schalter mitbauen, Default linear.
7. **Wake-Füllung bis x+ ist Teil des Plans** (Heiko-Entscheidung; meine Ausklammerung
   zurückgenommen — Begründung §4).

---

## 1. Teil A — Nahfeld-Box. KEIN CODE.

Zielgeometrie, auf ganze Fernzellen eingerastet, Heck gekürzt auf ~500 M Zellen:

| | heute | neu |
|---|---|---|
| Gitter | 1689 × 621 × 485 = 508,7 M | **1645 × 653 × 465 = 499,5 M** |
| Luft x− / y± / z+ [Fernzellen] | 20,4 / 20,05 / 45,67 | **24,0 / 24,05 / 40,67** |
| Nachlauf hinter dem Heck | 1,990 m | **1,756 m** |
| VRAM gesamt | 28,6 GiB | **28,0 GiB** |

```
CFD_NEAR_VOR_MM=160  CFD_NEAR_LX=6.416  CFD_NEAR_LY=2.608  CFD_NEAR_LZ=1.856
```

**Warum diese Zahlen:** 160 mm = 10 × 16 = 5 × 32 → rasterrein auch auf der 8-mm-Sprosse.
`NEAR_LY` = 163 Fernzellen → `cey` = 164 **gerade** → kein stiller Paritätshub (setup.cpp:2228
erhöht `cey` sonst wortlos um 1). `NEAR_LZ` = 116 Fernzellen. `NEAR_LX` = 411 Fernzellen
(`near_vor` addiert sich in setup.cpp:2195 dazu).

**Zwangsbedingungen, die dabei erhalten bleiben** (alle geprüft):
- `fN* = (c*−1)·ratio + 1` — jede feine Kantenlänge ≡ 1 (mod 4). Verstoß = `print_error`.
- Mittelebene y=0 auf Grobzell-**Fläche** und Fein-**Gitterpunkt**: `NF_OY = (cNy−cey)/2` mit
  gerader Differenz. Sonst der dokumentierte Altfehler „Seitenkraft aus dem Nichts".
- SPONGE-Guard `s_sponge_n + 32 > NF_OX`. NF_OX sinkt → Obergrenze sinkt mit; `SPONGE_N=64`
  bleibt legal.
- Fernfeld bleibt **unverändert** (768 × 480 × 552), Versperrung bleibt 2,74 %.

**Selbsttest, der beweist, dass nur verschoben und nichts verdreht wurde** — diese Zahlen dürfen
sich NICHT ändern (Verschiebung ist ein ganzzahliges Vielfaches der Feinzelle):
SAT-Schale fein 1.852.516 · Void-Fill 69.370 · Kontaktfläche 6412 · Stirnfläche 117.201 Zellen /
1,8752 m² · Facetten K2 312.840 / K4 483.121 · Verdrängungs-Census grob 1.067.260 / 7534.

**Nachzuziehen (sonst still falsch):**
- `werkzeuge/diff_aus_png.py` hatte `NEAR_X0=-0.326` hartkodiert — **am 2026-08-21 behoben**,
  Geometrie ist jetzt Pflichtargument.
- `einlass_saeule_nah.csv`, `boden_laengsprofil.csv`, `unterboden_sonde.csv` verschieben sich
  indexweise → A/B **über Welt-x** auswerten, nie über den Index.
- Schnitt-PNGs ändern die Bildgröße → pixelweise Diffs gegen alte Läufe ungültig. VTK ist
  unkritisch (ORIGIN/SPACING tragen die Weltlage).
- Veraltete Zahlen im Quelltext: setup.cpp:2110, 2160-2162, 2224-2226, 2374 (Sponge-Abstände
  152/199/162/430 — dreimal falsch), 125 („fNy/2=158"), 3872 (hartkodiert „BB-Anker: 1122"),
  3959/3978. Dazu DOPPEL-DOMAENE.md:33-37+196, FACETTEN.md:110 („Standard 96").

---

## 2. Teil C1 — Rückwärtsband ums Fahrzeug. KEIN CODE für die lineare Fassung.

`CFD_N2F_SCHALE=0.5 CFD_N2F_SCHALE_LAGEN=8` **ist bereits der Arm**: N Ein-Zell-Lagen um die
Fahrzeug-BBox, Gewichte w_k=(N−k)/N, Blend nur auf u (rho bleibt lokal = druckerhaltend),
Wächter, Kipp-Kriterium, u-Negations-Nachweis, Wirkpfad-Slot 22, IDENT-Bitbeweis — alles da.

**Damit ist der ganze C-Zweig ein Kill/Keep-Entscheid für 24 Minuten Rechenzeit, bevor
irgendetwas gebaut wird.**

Neu zu bauen ist nur der Unterschied „Schale um die BBox" → „Band vom Fahrzeug nach außen":

- **Saatmenge = `(flags & (TYPE_S|TYPE_X)) == (TYPE_S|TYPE_X)`**, NICHT `flags & TYPE_S`.
  ⚠ **Die Fahrbahn ist bei z=0 flächendeckend TYPE_S.** Mit ihr als Saat wäre das „Band ums
  Fahrzeug" ein 8 Zellen dicker Teppich über 768×480 = **2,95 Mio Zellen** — stiller Totalschaden.
- Chebyshev-Distanztransformation per 8 Runden × drei 1-D-Dilatationen über eine Arbeitsbox.
  Deterministisch, Millisekunden, kein BFS. **Lage = d−1** → der vorhandene Lagen-Census,
  die Wächter-Außenlage und die Negations-Schwelle tragen unverändert weiter.
- Bekannte Schwäche, anzusagen: Chebyshev macht das Band an Körperkanten diagonal dicker
  (8 → 13,9 Zellen). Ersatz wäre Chamfer-3-4-5; Schaltername `CFD_N2F_BAND_METRIK` reservieren,
  erste Fassung Chebyshev, Metrik in der Census-Zeile mitdrucken.
- **BBox ≠ Körper:** die Fahrzeug-BBox ist 2,40 M Grobzellen, der voxelisierte Körper ~1,05 M —
  55 % des BBox-Inhalts ist Fluid. Die heutige Schale liegt an Motorhaube, A-Säule und über dem
  Heck teils einen halben Meter von der Karosserie weg. **Der A/B Schale(LAGEN=8) gegen
  Band(N=8) ist genau diese eine Variable.**
- Der **Unterbodenspalt** (~120 mm = 7,5 Grobzellen) liegt vollständig in Lage 1..7, ~220.000
  Zellen. Drin lassen, getrennt zählen, `CFD_N2F_BAND_UNTERBODEN=0` als Vergleichsarm.
- `CFD_FERN_FACETTEN>0` **hart sperren** — Lage 1 ist definitionsgemäß die Facettenmenge, der
  Blend überschriebe deren `fi` nach `apply_facette`.

**Zwei vorhandene Wächter schlagen beim Wake-Arm falsch an und müssen eingeschränkt werden:**
- **u-Negations-Nachweis** (setup.cpp:3496): misst mittleres u_x der Außenlage gegen ~0,52 u_inf.
  Im Nachlauf fällt u_x physikalisch darunter → Fehlabbruch. Fix: auf Außenlagen-Zellen
  **stromauf der Nase** beschränken (dort immer ≈ +u_inf, also robuster als heute).
- **Kipp-Kriterium** (setup.cpp:3515): „10 Samples monoton steigend" ist am Anfahrtransienten
  geeicht. Im Wake wächst die Nah-Fern-Diskrepanz über die ersten ~0,1 s physikalisch monoton →
  Fehlalarm-Generator. Fix: Kipp-Metrik auf `zone==0 && lage==N−1` (Körperband); für die
  Wake-Zone eigene Schwelle RMS > 1,0 u_inf plus |rho−1| > 0,1.

---

## 3. Teil B — Vorwärts-Blendband fern→nah. Das einzige echte Neuteil.

Band von N Grobzellen nach innen auf den vier getriebenen Flächen, Gewicht 100 % an der
Randschicht → 0 %, Blend der feinen Lösung gegen die kubisch geliftete grobe.

**Der zentrale Entwurfsbefund: KEINE Indexliste.** 86 M Bandzellen × (8 B Index + 4 B Gewicht) =
**1,03 GB VRAM**. Stattdessen geometrisch wie `boden_eq`: vier **disjunkte** Slabs, Abstand aus
(x,y,z) im Kernel gerechnet → **21 MB statt 1,03 GB**. Die disjunkte Konstruktion beseitigt
konstruktiv, dass zwei Bänder dieselbe Zelle zweimal blenden (nicht idempotent: w₁ dann w₂ ergibt
w₁+w₂−w₁w₂, nicht max).

**Nur u blenden, rho bleibt lokal.** Fünf Gründe: (a) der Ebenen-Lift schreibt rho nur, weil
TYPE_E beides braucht — ein Dirichlet-Argument, das im Volumen nicht gilt; (b) das Fernfeld kennt
das Fahrzeug als Treppenkörper, sein rho ist in Fahrzeugnähe systematisch falsch; (c) zwei
Druckanker (x+-Auslass plus rho-Blend über 17 % des Volumens) sind der dokumentierte
Resonanz-Mechanismus; (d) `po_mean` lässt das Nahfeld-rho ohnehin driften; (e) RHO_CLAMP würde
erst greifen, wenn der Lauf schon verdorben ist. **Messarm `CFD_VBAND_RHO=1` trotzdem bauen** —
sonst ist die Begründung eine Behauptung.

**f_neq MUSS erhalten bleiben (FNEQ-Arm), der EQ-Arm ist im Band unbrauchbar.** Bei a=0 ist
`f = f_eq(rho_l, u_lokal) ≠ f_true` — der EQ-Arm wirft den Spannungstensor auch dort weg, wo das
Gewicht null ist, und erzeugt am äußeren Bandrand einen **Sprung**. Der FNEQ-Arm degeneriert bei
a=0 zum exakten No-Op und ist über die ganze Rampe stetig.

**Fünf Sicherheitsauflagen** (alle aus `schale_blend` zu übernehmen):
1. Der Blend muss **in `do_time_step`** hängen, nicht vom Host aus enqueued werden — `t` trägt die
   EsoPull-Parität und wird erst in lbm.cpp:1605 erhöht. Ein Host-Enqueue sähe ein bereits
   erhöhtes `t` → **Paritätsvertauschung, still falsch rechnend**.
2. Flag-Prädikat exakt `bo = flags&TYPE_BO; if(bo==TYPE_S||bo==TYPE_E) return;` — **nicht**
   `flags&(TYPE_S|TYPE_E)`. TYPE_MS (0x03) ist Fluid und wird behandelt (MS-Guard-Lehre).
3. `is_dead_tile`-Guard **vor jedem `load_f`** — bei aktivem SPARSE_TILES hat eine Bandzelle in
   einer toten Tile keinen `fi`-Speicher, `cell_base()` liefert Slot 0 und zerstört fremde Daten.
4. NaN/Inf per **Bit-Test auf Exponent 0x7F800000** — `isfinite` ist unter
   `-cl-finite-math-only` toter Code.
5. Solid-Ecken im groben Stencil fallen aus der Summe, verbleibende Gewichte werden renormiert;
   keine gültige Ecke → Zelle überspringen. Muster: `render_yslice_diff` (setup.cpp:151).

**Gewichte in GLOBALEN Fußabdruck-Koordinaten berechnen**, nur die Adressierung subtrahiert den
Slab-Ursprung. Sonst fällt `cubic_lift_weights` an jeder **Slab**-Kante auf die einseitige
3-Punkt-Formel zurück — still falsch, weil eine Slab-Kante keine echte Kante ist.

**Phasengate-Falle:** ein `t%100`-Gate für den Wirkpfad-Zähler feuert bei ratio=4 **nur für
Phase 0**. Gate auf `t % (1000·ratio) == phase`, `phase` als Kernel-Parameter. Und `uint`
wickelt bei 86 M Zellen — eigener `Memory<uint> vband_diag` mit 8 Slots statt
`rho_clamp_hits` (das hat 23 Slots, alle belegt).

**Kosten:** untere Schranke 4,3 % (reines Zellverhältnis), realistischer Korridor **6–15 %**.
Der 64-Punkt-Gather gegen 24 MB L2 und der Transfer im **seriellen** Fenster (nach
`lbm_c.finish()`, versteckt sich hinter nichts) sind nicht schätzbar. **`ph_vband` als eigenen
Phasenposten führen**, damit es gemessen und nicht diskutiert wird.

---

## 4. Teil C2 — Wake-Füllung bis x+

**Warum die V1-Schadensbefunde nicht dagegen sprechen:** 2026-06-22 (Cd verdoppelt) und
2026-07-08 (Totwasser-Artefakt) betrafen beide **eine einzelne Ebene**. Der 07-08-Befund sagt
wörtlich, die Injektionsebene sei von ihren Nachbarn nicht unterscheidbar gewesen — „eine Ebene
in einem Coarse-See wird pro Step weggemittelt" — und schlägt selbst als Fix vor: „Übergabe über
eine REGION (mehrere Coarse-Ebenen/Volumen) statt 1 Ebene". Der Barrieren-Mechanismus von 06-22
hängt an der Einzelebene und entfällt bei einem gefüllten Volumen.

**Was NICHT entfällt — die Zahl, die den Arm entscheidet:** das aufgeprägte Impulsdefizit über die
Silhouette liegt bei rund **790 N** (aus den 07-08-Messwerten: Fernfeld 22–25 m/s wo das Nahfeld
14–16 m/s hat, über 1,94 m²). Der gesamte Fahrzeugwiderstand bei Cd 0,599 ist **611 N**. Wir
prägen dem Fernfeld also eine Kraft in der Größenordnung seines ganzen Widerstands auf,
zusätzlich zu der, die sein eigener Körper erzeugt. **Abnahmekriterium: der Impulsdefizit-Fluss
durch die stromabwärtige Begrenzungsebene muss gegen die Fahrzeugkraft konvergieren, nicht
darüber hinausschießen.**

**Zweiter Mechanismus — der Blend ist nicht massenerhaltend:**
`div(u_blend) = grad(w)·(u_near − u_far) + …`; der erste Term verschwindet nicht. Mit
|grad w| = 1/(N·dx_c) = 1/0,128 m und |Δu| bis 10 m/s: `div u ≈ 78 1/s`, also 3,1e−3 relative
Dichteänderung **pro Grobschritt** in den ungünstigsten Rampenzellen. **Eine breitere Rampe
reduziert das linear** → N = 4 / 8 / 16 ist ein physikalisch begründeter Messarm, nicht Geschmack.

**Geometrie:** echte yz-Silhouette (der Code dafür existiert: setup.cpp:2483-2493), nicht BBox.
Stromabwärtiges Ende **nicht** am Nahfeld-x+-Rand — die letzte Fußabdruck-Spalte bildet auf die
feine TYPE_E-Auslassebene ab; wir würden den Randwert (u = u_inf, rho = 1) ins Totwasser
einspeisen. `CFD_N2F_BAND_WAKE_ABSTAND` Default **16** Grobzellen.

**Eleganteste Umsetzung:** Wake-Kern als **zweite Saatmenge** in dieselbe Distanztransformation.
Dann ist „derselbe Auslauf, keine Kante" konstruktiv erfüllt, in y, z und stromab gleichermaßen,
mit demselben Profilschalter, ohne eine Zeile Sonderlogik.

**Zirkularität an x+:** die ≥2-Grobzellen-Regel ist dort **gegenstandslos** — x+ wird nicht
getrieben (`drive_face = {true,false,true,true,true}`). An ihre Stelle tritt der Abstand zur
feinen Auslassebene. Die **indirekte** Rückkopplung bleibt und ist schnell: effektive
Schallgeschwindigkeit 231 m/s, 0,5–2 m zu den getriebenen Ebenen = 2–9 ms. Instrument dafür ist
`interface_druck.csv`, existiert bereits.

---

## 5. Der Frühindikator — Fx_far, nicht Cd

Aus den vorhandenen Läufen nachgerechnet, Fenstermittel `forces.csv`:

| Arm | Cd [0,02–0,10 s] | **Fx_far** |
|---|---:|---:|
| Basis | 12,16 | **3.760 N** |
| Schale α=0,5 | 12,12 | 5.717 N |
| Gradient α=1,0 | 12,00 | 8.282 N |
| **Volumen-Blend (verworfen)** | **12,03** | **44.553 N** |

**Der zerstörerische Arm war im frühen Cd besser als die Basis.** Ein Cd-Gate hätte ihn
durchgewinkt. `Fx_far_N` trennt um Faktor 12 — im ersten Sample bei t = 2 ms bereits
83.837 N gegen −19.924 N.

**Gemessenes Rauschband** (quasi-A/A-Paar `f8p_basis` ↔ `f8_standard_final`):
`Fx_far_N` ≤ 0,03 N für t<0,1 s · `interface_druck dp_mittel` ≤ 0,36 Pa · `cd_druck` Δ 0,0008 ·
`cz_druck_rest` Δ 0,005. Block-SEM (k=4, 36 Samples): cd_druck ±0,029…0,038,
cz_druck_rest ±0,023…0,028 → **2σ einer Differenz ≈ 0,09 bzw. 0,07.**

### Abbruchgates

| Gate | Größe | Zeit | Schwelle |
|---|---|---|---|
| G2 (A, B) | `Fx_far_N` Δ zum Anker | t < 0,1 s (~2,8 min) | \|Δ\| > 0,5 N ⇒ Leck ins Fernfeld = Bug |
| C-G1 | `Fx_far_N`, erste 10 Samples | t = 0,02 s (~2 min) | > 15.000 N ⇒ Abbruch |
| C-G2 | `Fx_far_N` [0,02;0,10] | t = 0,10 s (~2,8 min) | > 12.000 N ⇒ Abbruch |
| C-G3 | dito | dito | < 3.900 N ⇒ **No-Op-Befund** (kein Abbruch) |
| C-G4 | `interface_druck` x− `rho_mittel` | t ≥ 0,05 s, 3 Samples | > 1,010 ⇒ Abbruch |
| C-G5 | dessen Steigung, 0,1-s-Fenster | ab t ≥ 0,10 s | > +2e−3 /s ⇒ Abbruch (≙ +130 Pa/s) |
| C-G7 | \|dp(y−) − dp(y+)\| | Mittel t ≥ 0,2 s | > 8 Pa ⇒ **Resonanz-/Limit-Cycle-Detektor** (Basis 0,38 Pa) |
| **C2-G1** | `Fx_far_N`, erste 5 Samples | t = 0,01 s (~1,8 min) | > 20.000 N ⇒ SOFORT-Abbruch |
| **C2-G4** | `interface_druck` x− `dp_mittel` | t = 0,10 s | > 230 Pa ⇒ Abbruch (Basis 164,2) |
| **C2-G6** | `cz_druck_rest` | t = 0,22 s | Vorzeichenwechsel ins Positive ⇒ Abbruch |

**Abbruch NIE per `kill`** — der i915-GEM-Leak kostet 12–16 GB je Abschuss. `touch /tmp/cfd_stop`
(seit 2026-08-21 eingebaut) verlässt die Zeitschleife an der nächsten Sample-Kadenz und schreibt
alles fertig, inklusive VTK.

---

## 6. Phasenplan

| Phase | Was | Code | GPU-Zeit |
|---|---|---|---|
| 0 | Referenzanker + **echtes A/A-Rauschband** (existiert bis heute nicht) + Kadenz-Neutralität | – | 24 min |
| 1 | Teil A: drei Achsen einzeln + kombiniert (Additivitätstest) | – | 32 min |
| 1b | 4-mm-Speicher-Trockenprobe | – | 6 min |
| 2 | **C1 linear: LAGEN 4 / 8, α 0,25 / 0,5** ⇒ Kill/Keep für den ganzen C-Zweig | – | 24 min |
| 3 | cos²-Profil + Wake-Füllung (Host-Listenbauer) | ~200 Z. | 24 min |
| 4 | Teil B (neuer Kernel; CPU-Sprosse zuerst — neue Kernel nie zuerst auf der B70) | ~600 Z. | 32 min |
| 5 | 4 mm: Anker, A/A, Sieger + VTK | – | 4 h 44 |

Screening **2 h 42**, gesamt **~9 h 40** inkl. 30 % Reserve. Die Gates machen aus jedem
gescheiterten 4-mm-Arm 7–21 Minuten statt 92.

**`CFD_FAC_CD_EVERY` von 4 auf 1** in allen Armen: halbiert 2σ auf `cd_druck` von 0,09 auf 0,05,
kostet +3 % Laufzeit. Ohne das darf kein 8-mm-Arm mit |Δ| < 0,09 „Gewinner" heißen — der
P9c-Befund dCz_rest −0,087 lag bei 1,7σ. `f9_ref_kad4` beweist die Kadenz-Neutralität.

**`CFD_SLICE_DT` niemals auf 0** — `einlass_saeule_nah` und `schnitt_diff_letzter.csv` hängen am
Slice-Hook. Wer Slices abschaltet, verliert genau die Instrumente, mit denen B und C bewertet werden.

---

## 7. Zielgrößen

Ausgangslage 4 mm (`f4_std_diff2`, Mittel ab 0,2 s):
cd_druck **0,8428** · cz_druck **−0,5795** · davon Kontaktband +0,2461 · **cz_druck_rest −0,8255**
Diff-RMS gesamt 6,070 m/s · Einlaufband 0,357/0,540/0,952 · Nachlauf max 8,558 bei x=+4,1
OF13: Cd 0,599 / Cz −1,301. 4-mm-SEM (74 Samples): cd_druck ±0,014, cz_druck_rest ±0,019.

**Gewinn, wenn auf 4 mm gleichzeitig gilt:**
1. `cd_druck` ≤ **0,78** (−25 % der Lücke)
2. `cz_druck_rest` ≤ **−0,92** (≥ 20 % der Lücke)
3. Diff-RMS im Einlaufband ≤ **0,5 ×** heute
4. Streifenmaß `einlass_saeule_nah` dux-RMS ≤ **0,0035** (halbiert)
5. keine Verschlechterung: `unterboden_sonde` u_rel nicht kleiner, `cz_druck_band` ±0,03,
   Leistungsindex ≤ 1,10 ×

**Teilerfolg** (rechtfertigt Beibehaltung, nicht die volle Ausbaustufe): 3+4 erfüllt, 1+2 im Rauschen.
**Fehlschlag:** cd_druck steigt > 0,03, ODER `Fx_far` verlässt das gutartige Fenster, ODER
cz_druck_rest wird kleiner.

---

## 8. Offene Entscheidungen

1. **„8 Fernzellen" = Grobzellen der jeweiligen Sprosse oder feste 128 mm?** Auf 8 mm ist eine
   Grobzelle 32 mm, auf 4 mm 16 mm. Empfehlung: Grobzellen der Sprosse (der Mechanismus behandelt
   ein Grobgitter-Artefakt) **plus Tiefenleiter 2/4/8**, damit der Sprossen-Übertrag gemessen wird.
   ⚠ C1 mit 8 Lagen ist auf der 8-mm-Sprosse **ohne Teil A geometrisch illegal** (d_y− = 2 = harte
   Fehlerschwelle). **C hängt an A** — echte Reihenfolge-Abhängigkeit.
2. Wird `schale_blend` vor `boden_eq` gezogen (Heiko-Vorgabe §0.2), muss der `z_lo`-Ausschluss im
   Listenbauer entfallen — sonst wirkt er doppelt.
3. Der offene Prüfagent-Befund **M2** (FNEQ-Laufzeit-Paritätsbeweis, α=0-Test) ist unerledigt.
   Solange er offen ist, darf `CFD_N2F_SCHALE_FNEQ` in keinem Verdikt vorkommen → alle C-Arme
   im EQ-Modus fahren.

---

## 9. Was fehlt und gebaut werden sollte

1. **8-mm-Referenz des Diff-Schnitts** — `f8_standard_final_diff` brach bei 74 ms ab. Ohne
   konvergierten 8-mm-Diff hat B auf der Screening-Sprosse keine Zielgröße. Phase 0 löst das.
2. **Block-SEM für `cd_druck`/`cz_druck`/`cz_druck_rest`** — der Endreport druckt ihn nur für die
   phantombehafteten `forces.csv`-Größen. ~15 Zeilen, `block_sem()` existiert.
3. **Nahfeld-Massenbilanz je Sample** (Σ(ρ−1) über Fluid) — das direkte Maß gegen Massenquellen.
4. **Seiten- und Deckel-Säulen** analog `einlass_saeule_nah` bei y+ und z+. Das Streifenmaß gibt
   es nur an x−, B wirkt aber auf **vier** Ebenen.
5. **Diff-Auswertung nach z-Bändern und Randabstand** — `diff_schnitt_auswerten.py` kann nur x.
6. **Deterministische `po_mean`-Reduktion** — hebt die halbe Beweisführung von „im Rauschband" auf
   „bitgleich". Bis dahin ist `CFD_PO_HART=1` die Krücke für jeden Bitbeweis.
7. **Luft-Ansage in Fernzellen** beim Start (heute steht nur der Einlauf in Metern im Log).
