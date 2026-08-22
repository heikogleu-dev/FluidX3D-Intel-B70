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
