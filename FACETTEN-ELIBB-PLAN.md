# Facetten-ELIBB Bauplan (Punkt 1c+9+11, Planungsagent 2026-08-22 abends)

Heiko-Vorgaben bindend: Facettenquelle = Remesh der VOXEL-Aussenwand (nicht STL);
alle Link-Klassen inkl. Diagonalen; Kosten/Nutzen entscheidet.

## Variantenwahl: ELIBB-FL, linkweise auf ALLEN 18 geschnittenen Links
- ELIBB-U (Gl. 23a): q-invariant fuer statische Waende -- V1-belegt (Stage A_3b, bitgleich
  q=0,5/0,3), scheidet aus.
- BFL/volle Gl. 15: liest Nachbar-DDFs -- EP-Pull-verboten (V1-Verdikt 2026-05-31).
- ELIBB-FQ: Koeffizienten nie gedruckt, q-Cutoff-Degeneration -- kein Nutzen-Beleg.
- **ELIBB-FL erfuellt alles:** Ein-Zellen, q-abhaengig, q=0,5 == exakt HWBB (bitgleiche
  Regression gratis). ACHTUNG: Eq. 25e im Paper FALSCH gedruckt -- partition-of-unity-Form
  a2=(1-q)/q verwenden (V1-Re-Derivation, gesichert in der V1-Vollextraktion 2026-05-31).
- **Portierungsvorlage existiert:** V1-Commit 75b11a9, ~30 Zeilen, axial, 4-bit-q gepackt,
  Snapshot-Fix gegen 1-Zell-Spalt-Aliasing, w_i-Perturbations-Subtraktion drin.
  V2-Neuerung = 18-Link-Schleife + u_W = u_s statt 0.
- Kosten: ~15 FLOPs je Link; q als 18 x uchar je Facette = 59 MB @ 3,3 M Wandzellen
  (empfohlen -- Remesh-q an Kanten besser als Ebenen-q); on-the-fly y_w/|c*n| als
  Kontrollarm = gratis-A/B.

## iMEM-Zusammenspiel
Reihenfolge unveraendert EP-sicher (registerlokal in stream_collide): Abtastung -> Spalding
-> iMEM-Solve u_s -> ELIBB-Rekonstruktion der Solid-Links mit f~ = w_i*rho_F*(1+3c*u_s+...)
- w_i + f_neq_quer. ELIBB ERSETZT den Additivterm q_i = 6w_i(c*u_s) -- nie stapeln
(Doppelzaehlung). Markierte Zellen (21,9 %) bleiben reiner BB. u_s = 0 => rein
geometrisches No-Slip (Kugel-Arm).

## Remesh-Pipeline (Host, nach Void-Fill)
**Naive Surface Nets statt Marching Cubes** (~150 Zeilen: 1 Vertex je Randzelle, Quads dual
zu geschnittenen Zellflaechen, garantiert geschlossen/mannigfaltig, keine MC-Tabellen).
Glaettung **Taubin lambda|mu** (0,5 / -0,53, 10-20 Iter., ~60 Zeilen), Vertex-Klemme
+-0,5 Zelle (begrenzt q-Fehler hart). Uniform-Grid-Binning, je Wandzelle 3-4 naechste
Dreiecke, je Link exakter Ray-Triangle-q (~150 Zeilen). fac_geo unveraendert, neuer
fac_q-Puffer. Voxel-Aussenwand ~ Halfway-Wand => y_w-Eichkonflikt entschaerft.
iMEM-Spalding bleibt auf y_w (geeicht), ELIBB auf q -- bewusst getrennte Semantiken.

## Phasen
P0 Harness-Port (V1: 520 LOC existieren, ~200 anpassen, CPU) -> P1 fac_q-Host-Pipeline
(~450) -> P2 Kernel-Block hinter CFD_FACETTEN_ELIBB, Default aus (~90+40) -> P3
Kraftbuchhaltung (~40; Reibungs-Akkumulator muss ELIBB-Ist-Austausch je Link buchen, sonst
reisst K2) -> P4 Messleiter. GPU-Leiter CPU -> iGPU -> B70, vorher committen.

## Validierungsleiter / Gates
V1 Kanal: ELIBB aus bitgleich (Anker) UND q==0,5 bitgleich -- sonst Stopp.
V2 Kugel: BB-Basis 0,717, Facetten+alpha2 0,436, Achenbach-Band 0,45-0,5; Gate: im Band
UND Delta-m im alpha2-Band. V3 Torus N2: cf <= BB-Basis. V4 Fahrzeug 8 mm.
**Erwartung ehrlich:** V1 mass bei aufgeloester Grenzschicht nur ~2 von 20 Pp aus der
Treppe -- der V2-Hebel liegt anders (druckdominierter Cd 98 %, 94 % der heilbaren
ohneTang-Klasse Einzellink-diagonal). Das Kugel-Gate entscheidet, nicht die Hoffnung.

## Risiken
(1) EsoPull-Zeitindex: bounced-Operand traegt Fullway-Latenz (f_out_opp(t-2)) --
Harness-Nachweis Pflicht. (2) ELIBB nicht massenerhaltend (Marson) -- Delta-m-Band-Gate.
(3) Kraftpfad: object_force bleibt Phantom; Reibungs-Akkumulator erweitern. (4) FP16C:
FP32-Sprosse in der Leiter. (5) Marson-PDF lokal nicht mehr vorhanden -- Belegkette ist
die V1-Vollextraktion; FQ-Koeffizienten und Marson-Thesis (NELI) nie beschafft.

---

# P1 ABGESCHLOSSEN (2026-08-22) — was P2 daraus erbt

## Was P1 belegt hat (Messung, nicht Annahme)
| Frage | Antwort | Beleg |
|---|---|---|
| Liefert der Remesh ueberhaupt q? | 100 % der Solid-Links, 0 HWBB-Rueckfall | `p1_fz8` |
| Ist q besser als HWBB? | Rohe Voxelflaeche gibt q **exakt 0,5000** auf ALLEN Links = HWBB. Erst die Glaettung erzeugt Information. | `p1_kugel_ref` |
| Wie genau? | RMS gegen die analytische Kugel 0,2642 (roh) -> 0,1563 (15x) -> saettigt bei 0,1430 | `p1_it*` |
| Trifft der Strahl die richtige Flaeche? | ABNAHME PARITY bestanden: jede Verbindung schneidet die Flaeche ungerade oft | beide Faelle |
| Verschliesst die Flaeche Wege? | ABNAHME DURCHSCHUSS bestanden: 0 von 3.749.916 Achs-Fluidverbindungen gekappt | `p1_fz{3,8,15}` |
| Taugt die STL als q-Quelle? | **Nein.** Nur 40,5 % der Solid-Links haben eine STL-Flaeche in Reichweite, 59,5 % sind SAT-Schale und Void-Fill. Die Wand des LBM ist der Voxelkoerper. | `p1_stl_ab` |
| Richtungstabelle? | Host-`cd[18]` deckt sich EXAKT mit der Kernel-`c()` (kernel.cpp:940-942), spaltenweise geprueft | Pruefagent |

## Feste Entscheidungen fuer P2
1. **q kommt aus der geglaetteten Remesh-Flaeche**, nicht aus der STL. Begruendet oben.
2. **Glaettungstiefe Standard 8** (`CFD_FACETTEN_REMESH_ITER`). ELIBB rechnet mit
   a2=(1-q)/q; bei ITER=8 ist q_min 0,196 (Kugel) bzw. 0,169 (Fahrzeug-Engstellen),
   also a2_max rund 5. Bei ITER=15 waere a2_max schon 9-15. Der Geometriegewinn von 15
   gegenueber 8 betraegt nur 12 % des Resthubs -- schlechtes Tauschverhaeltnis.
3. **q-Boden ist Pflicht, keine Kuer.** Auch bei ITER=8 gibt es einen Schwanz nach unten.
   Vorschlag: q < q_min -> HWBB (q=0,5) statt Extrapolation, mit eigenem Zaehler.
   q_min als Schalter, Startwert 0,1. Der Zaehler ist die Abnahme: er MUSS feuern und
   klein bleiben (bei ITER=15 waren es 549 von 3,5 Mio = 0,016 %).
4. **Eq. 25e in Marson 2021 (PRE 103:053308) ist ein Druckfehler** -- a2=(1-q)/q nutzen.

## Was P2 als Erstes braucht
1. **q auf die GPU.** Nur Solid-Links, in der Reihenfolge der `cd[18]`-Tabelle.
   Groessenordnung 4 mm: rund 3 Mio wandnahe Zellen x 18. Als `uchar` quantisiert
   (q in 1/255) reicht die Aufloesung locker -- der q-Fehler der FLAECHE ist 0,14,
   die Quantisierung 0,004. Speicher damit im dreistelligen MB-Bereich statt GB.
2. **Esoteric-Pull-Richtungskonvention** (gerade/ungerade i) -- die EINZIGE Auflage,
   die der Pruefagent fuer P2 offen gelassen hat. Vor dem ersten Kernel klaeren.
3. **Kugel-Gate zuerst** (Iron Rule Testleiter: CPU -> iGPU -> B70). Die Kugel hat
   analytisches q und eine Literatur-Cd-Kurve; ein ELIBB, das dort nicht traegt,
   braucht am Fahrzeug nicht getestet zu werden.

## Was P2 NICHT loesen muss
Die zugeschmierten Radhaeuser (Arbeitsliste 12) sind ein VOXEL-Problem, kein
Flaechenproblem -- Void-Fill mit 6er- statt Gitterkonnektivitaet. Der Schalter
`CFD_VOIDFILL_KONN=18` liegt fertig da. Bewusst nach P2 eingeplant: ob eine offene
Radhaustasche etwas bewegt, laesst sich erst beurteilen, wenn die Wandbehandlung dort
ueberhaupt q-abhaengig ist.
