# Grenzschicht und Fernfeld-SGS — Bauplan (zwei Planungsagenten, 2026-08-23)

Heiko wollte die Hebel 2 (Smagorinsky im Fernfeld) und 5 (Grenzschicht) vorbereitet haben.
Zwei unabhaengige Planungsagenten haben sie getrennt untersucht. Das wichtigste Ergebnis ist,
dass die beiden **keine parallelen Hebel sind, sondern eine Kette** — und dass zwei meiner
Annahmen dabei gefallen sind.

## Was gefallen ist

**1. Mein Hebel 2 war falsch benannt.** Delta = lokale Gitterweite ist LES-Lehrbuch, die
16-fache Wirbelviskositaet im Fernfeld ist die KORREKTE Modellskalierung und kein Fehler.
Angreifbar ist die **Konstante C = 0,1733** (der isotrope Wert; in scherdominierten
Wandstroemungen sind 0,1 nach Deardorff 1970 bis 0,065 nach Moin und Kim 1982 ueblich,
nu_t proportional C^2 also Faktor 0,33 bzw. 0,14).

**2. Van Driest ist GESTRICHEN — das Vorzeichen ist falsch.** Nicht weil der Effekt fehlt,
sondern weil er in die falsche Richtung zeigt:
- y+ am Fahrzeug 4 mm gemessen (yplus_facetten.csv, f4_std_diff2): q25 24,8 / Median 39,4 /
  q75 64,6. D = 1-exp(-y+/26) ergaebe nu_t mal D^2 = -62/-39/-16 Prozent, und das NUR in
  Lage 1; Lage 2 liegt bei y+ rund 118 und ist inert.
- cf liegt am Kanal schon bei 0,00154 gegen Referenz 0,00344, also **55 Prozent ZU NIEDRIG**
  (Verdikt: Turbulenz-Selbsterhaltungsproblem). Van Driest senkt nu_t weiter -- falsche Richtung.
- Der Grenzfall D=0 existiert bereits als Schalter `CFD_SGS_WANDFREI` (kernel.cpp:2266-2282)
  und **kollabierte cf auf 9,7e-5, Faktor 16**.
- V1 hatte es gebaut und STRUKTURELL verworfen: y+ aus lokalem Strain nullt sich an der
  Abloesung selbst (Rueckkopplung). Der V1-Auditsatz lautet: van Driest wird No-Op, also
  ungedaempft, und das ist fuer WMLES richtig.
- Dazu Doppeldaempfung: `apply_facette_imem` laeuft vor `calculate_rho_u` und vor SUBGRID.
=> Nicht bauen. Falls Heiko es dennoch will: ausschliesslich als deklarierter Messarm mit
A+-Leiter {26, 52, 104} und D-Boden, nie als Cz-Hebel.

## Die Kette, die niemand vorher gesehen hat

- Bei tau = 0,500028 (nah) und 0,500007 (fern) ist **SUBGRID derzeit das Einzige, was den
  Fall am Leben haelt** — ohne ihn 869 NaN und Kraefte auf null (defines.hpp:68-73, gemessen
  2026-08-08). Die Smagorinsky-Konstante ist heute eine STABILITAETSKRUECKE, keine physikalische
  Wahl.
- **RR (Recursive Regularized) uebernaehme diese Rolle** — es filtert die Geistermoden. Erst
  danach ist C_s eine physikalische Groesse, die man einstellen darf.
- **RR kann aber nicht vor ELIBB kommen:** RR reprojiziert `fhn` auf (rho, u, a2) und loescht
  damit genau den nicht-hydrodynamischen Anteil, den `apply_facette_imem` gerade
  hineingeschrieben hat. Vertraeglich wird es erst mit ELIBB, das ohnehin als
  f~ = f_eq(u_s) + f_neq_quer formuliert ist.
=> **Hebel 2 haengt an RR haengt an ELIBB.** Parallel fahren geht nicht.

## Der Kandidat, der sich vordraengt: keine Anstroemturbulenz

Am Fahrzeug setzt der Einlass `u.x = u_lat, u.y = 0, u.z = 0` — **exakt gleichfoermig, keine
Stoerung** (setup.cpp:2487 und 2506, heute verifiziert). Der Kanalfall dagegen saet eine
stromfunktionsartige Stoerung (setup.cpp:1438/1452). Fuer einen Zustand, der als
laminaraehnlich und nicht selbsterhaltend diagnostiziert ist, ist das eine Ursache ERSTER
Ordnung fuer zu frueh abloesende Grenzschichten — und genau das sieht Heiko seit Wochen im
Diff-Slice an der Front. Aufwand rund 30 Zeilen.

## Reihenfolge

**P0 — DIAGNOSTIK, bitidentisch, entscheidet beide Hebel allein.**
Ohne sie misst jeder Arm nur Cd/Cz-Rauschen (2 sigma rund 0,09 in cd_druck).
- `nu_t/nu_0`-Histogramm im Kernel direkt nach kernel.cpp:2295, Schwellen 1/10/100/1000,
  vier neue Slots (rho_clamp_hits 28 -> 32), gegatet t%100. Ein omega-Histogramm waere
  WERTLOS, weil beide Felder schon bei omega = 1,9999 stehen — gemessen werden muss
  nu_t/nu_0 logarithmisch.
- **Abloeselage x_sep/L** entlang der Dachmittellinie aus dem Vorzeichenwechsel von u_t in
  Lage 1. OF13-Soll 0,82. Neben `schreibe_wandprofil` (setup.cpp:1733).
- **cf(x)** durch x-Binning der bereits akkumulierten Wandkraft `fac_tau[6k+1..3]`.
- **delta99(x)** aus derselben wandnormalen Saeule wie wandprofil_nah.csv.
- **u+(y+)**: `schreibe_wandprofil` schreibt u/u_lat schon, es fehlt nur u_tau aus `fac_tau[6k]`.
- Nulltest dazu: zwei Kanallaeufe N=108 mit `CFD_SGS_WANDFREI=1/0`. Wiederholt sich der
  Stufenkollaps dort NICHT, ist der van-Driest-Befund zu korrigieren.

**P1 — Anstroemturbulenz / Stolpersaat** (`CFD_TRIP_*`, Default 0). Billigster echter Hebel,
zielt direkt auf Heikos Frontabloesung.

**P2 — ELIBB** nach FACETTEN-ELIBB-PLAN.md. Ohnehin geplant, und Voraussetzung fuer P3.

**P3 — RR, nicht HRR.** Reines RR braucht KEINE Nachbarn (6 Zusatzskalare, zwei
In-place-Durchlaeufe ueber `fhn`, rund +40 FLOP je Zelle), ist Esoteric-Pull-vertraeglich und
FP16C-freundlich. HRR braucht zusaetzlich den FD-Scherratentensor aus 6 Nachbar-u und waere
mit +72 Byte auf 76 Byte je Zelle speicherbandbreiten-gebunden (+50 bis 90 Prozent).
Risiko: der Intel-Uebersetzer blieb schon einmal an zusaetzlichen 19-Element-Arrays haengen
(kernel.cpp:2355) — RR muss ohne zweites 19-Array geschrieben werden.
Quelle sicher: Jacob, Malaspinas, Sagaut 2018 (J. Turbulence 19).

**P4 — Smagorinsky-Konstante als Messarm.** Erst jetzt sinnvoll. Ein Skalar
(`s_sgs_c2_skal`), zwei Statiken, Emission in lbm.cpp; bei k = 1,0 muss der OpenCL-Quelltext
BYTE-identisch bleiben. Verworfen und begruendet: SGS im Fernfeld ganz aus (= der
869-NaN-Zustand), WALE/Sigma (brauchen den vollen Gradiententensor inklusive Rotationsanteil,
f_neq liefert nur den symmetrischen; der Umweg ueber Nachbar-u ist als nicht
bitreproduzierbarer Wettlauf protokolliert), selektiv im Band (erzeugt eine neue nu_t-Kante
genau dort, wo der FNEQ-Fix gerade eine beseitigt hat).

## Ein noch offener Kandidat: Rauwand

Bounce-Back-Basis cf 0,01067 gegen Referenz 0,00344 = **3,1-fach zu hoch**, k_s rund 1 Zelle.
Das ist der einzige Kandidat mit einem gemessenen Faktor-3-Fehler, und Rauigkeit ist eine
Ursache erster Ordnung fuer fruehe Frontabloesung. Zusammen mit dem cf-Defizit von -55 Prozent
am Kanal heisst das: die reine Wand ist 3,1-fach zu rau, das Wandmodell korrigiert das und
schiesst auf -55 Prozent hinaus. Beides gehoert zusammen betrachtet.

## Abnahmekriterien (fuer alle Arme)

x_sep/L wandert Richtung 0,82; cf(x) an der Front steigt Richtung OF13-Wert;
|Delta cd_druck| > 0,09 bzw. |Delta cz_druck_rest| > 0,07 (2 sigma aus dem A/A-Paar) — darunter
heisst KEIN Arm Gewinner; Wirkpfad-Zaehler > 0; Klemmzaehler unveraendert;
Schalter aus == bitidentisch (Emission gegatet, nicht nur Laufzeit-Ternaer).

## Messleiter, billigste zuerst

Kanal-Anker (bitgleich) -> Kanal N=108 (der einzige Kanal mit y+_1 rund 48; N=38 hat
y+_1 = 136 und ist fuer jede nu_t-Frage blind) -> Kugel (Achenbach-Band 0,45 bis 0,5;
ACHTUNG: dort muss SUBGRID aus, die Kugel ist fuer den SGS-Hebel KEINE Sprosse) ->
8 mm A/B -> 4 mm nur nach ausdruecklicher Freigabe.

---

# FAHRPLAN nach dem Messtag 2026-08-23 (ersetzt die Reihenfolge oben)

Der Plan von heute morgen ist durch die Messungen ueberholt. Was davon uebrig ist:

## Gestrichen
- **Smagorinsky-Konstante senken.** Die Rechnung sagt, wir liefen in den MSD-Zustand: Smagorinsky
  liefert im Gleichgewicht strukturell 0,714 der noetigen Wandschichtviskositaet, C 0,173 -> 0,1
  teilt nu_t durch 2,99 und landet bei 0,7x Gleichgewicht. Kein Arm.
- **Van Driest.** Falsches Vorzeichen (cf am Kanal schon -55 %), Grenzfall D=0 kollabierte cf um
  Faktor 16, V1 hatte es strukturell verworfen.
- **Void-Fill KONN=18 als Produktionsschalter.** Fluid=18 erzwingt Solid=6 (Rosenfeld/Kong);
  bleibt Diagnose.

## Der eine Versuch, der die offene Frage entscheidet -- und kein neuer Kernelcode noetig
**`CFD_KANAL_KIPP=26` bzw. `45` gegen `kipp=0`.** Identisches Re_tau, identische Stroemung,
identisches Wandmodell, EINZIGER Unterschied die getreppte Wand. Steigt die Wandlagen-nu_t bei
gleichem y+ gegenueber der ebenen Wand, ist spuriose Scherung aus der Voxeltreppe bewiesen;
steigt sie nicht, ist die Treppen-Deutung endgueltig erledigt. Beide Laeufe mit dem 1/64-Stand
UND mit gesetzter Warmlaufsperre (`CFD_SGS_DIAG_AB`).
Zweite, billigere Sonde daneben: die Wandlagen-Bins nach y_w-Klasse und Facetten-Winkel
aufspalten -- die Daten liegen schon in `export/*/facetten_histogramme.csv` (yw, winkel_grad,
Zellindex). Sitzt der Ueberschuss auf den SCHRAEGEN Facetten und nicht auf den achsparallelen,
redet die Treppe; sitzt er ueberall, redet die Instationaritaet.

## VORGESCHALTET: Determinismus wiederherstellen (Rueckmeldung Claude-Chat, 2026-08-23)
Der Befund "kein Fall ist deterministisch" trifft eine Iron Rule direkt -- bitgleiche
Regressionsanker waren ein Abnahmemittel und standen ploetzlich nicht mehr zur Verfuegung.
GEMESSEN am 2026-08-23 (q_det_1 gegen q_det_2, identische Konfiguration):
  Sample 1: 3,3e-7 | Sample 2: 1,6e-6 | Sample 3: EXAKT 0 | Sample 10: 1,8e-4 | Sample 25: 7,7e-5
Die Saat liegt bei wenigen float32-ULP, ein Sample ist exakt identisch, und erst danach waechst
es -- das ist chaotische Verstaerkung einer Rundungssaat, keine Loesungsdivergenz. Zum Vergleich:
sqrt(N)*eps ueber 56 Mio Terme erlaubt 9e-4.
=> **Fall 1 (Reduktionsreihenfolge), HEILBAR.** Eine feste Reduktionsbaum-Ordnung in der
GPU-Kraftreduktion bringt den Bitvergleich zurueck. Das gehoert vor alles andere, weil es
bestimmt, welche Abnahmen ab jetzt ueberhaupt gueltig sind.
Abnahme: zwei identische Laeufe byteweise gleich in forces.csv, dd-Fall UND Kugel.

## Danach, nach Evidenzlage geordnet
1. **Das Wandmodell selbst.** Die cf-Luecke ist wandmodell-dominiert (U_b+ 36,0 statt 24,1,
   Versatz +12 in U+), nicht SGS-dominiert. Groesster gemessener Einzelfehler im Projekt, und
   der einzige auf dem SAUBERSTEN Fall -- ebene, gitterparallele Wand ohne Geometrieausrede.
   Angriffspunkt: iMEM-Slipgeschwindigkeit/Spalding auf y_w.
   ★ VORGEZOGEN (Rueckmeldung Claude-Chat): ein Wandmodell, das auf der ebenen Wand 55 % cf
   verfehlt, KONTAMINIERT JEDE NACHFOLGENDE MESSUNG -- auch die zur Anstroemturbulenz. Wer es
   auf Platz 3 laesst, misst danach gegen einen verschobenen Nullpunkt.
2. **Anstroemturbulenz / Stolpersaat am Fahrzeug** (`CFD_TRIP_*`, Default 0, rund 30 Zeilen).
   Verifiziert abwesend, zielt direkt auf die Frontabloesung. ACHTUNG: erklaert den KANAL-Fehler
   NICHT -- der ist periodisch, dort muss sich Turbulenz selbst erhalten, eine Saat zerfaellt.
3. **ELIBB** -- aber mit gedaempfter Erwartung und aus dem richtigen Grund. Es korrigiert die
   Wandposition um <=0,5 Zelle und laesst die zackige FORM im flags-Feld stehen; der Bauplan
   zitiert V1 mit "nur ~2 von 20 Prozentpunkten aus der Treppe". Es bleibt richtig als
   Geometriekorrektur, ist aber KEIN Cz-Hebel-Kandidat erster Ordnung.
4. **Delta-Reihe 16/8/4 mm** mit den gebauten Slots. Kein Hebel, aber die Entscheidung darueber,
   ob unsere Aufloesungsabhaengigkeit MSD ist -- und wenn ja, waere jede Verfeinerung ein
   Rueckschritt, was die ganze 4-mm-Strategie betrifft.

## Ungeklaert, nicht vergessen
Der **Band-Wandrueckzug** (-0,27 Cz bei 8 mm, 2,4 sigma) ist der einzige je gemessene Cz-Hebel,
aber Heiko haelt den Wirkpfad fuer unplausibel: das Band wirkt nur im Fernfeld, Cz wird im
Nahfeld gerechnet. Ein Pfad ist NICHT belegt. Kandidat waere, dass die Nahfeldbox mit nur 96 mm
Vorlauf so eng sitzt, dass ihr TYPE_E-Rand im Bandgebiet liegt. Entweder den Pfad belegen oder
den Befund als Zufall abschreiben -- so stehenlassen geht nicht.
