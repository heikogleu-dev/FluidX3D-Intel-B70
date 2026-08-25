# K2-Lösungsentscheid — gekippte/voxelisierte Wände (2026-08-25)

Grundlage: drei unabhängige Analyseagenten (Geometrie-Kombinatorik, ELIBB-Reifegrad,
Literatur), deren Zahlen sich gegenseitig exakt reproduzieren, plus eigene Nachrechnung.
Messstand: ab2-Serie (Binary nach c69d18a), Zähler analytisch bestätigt.

## Der Befund in drei Sätzen

1. **Fahrzeug/Kugel (das eigentliche Ziel):** 97 % / 94 % der "ohneTang"-Facetten sind
   m2-artig — ihr Link HAT Tangentialprojektion, nur die α-Massen-Nebenbedingung
   vernichtet sie (bewiesene Identität q_i ≡ 0). Heilbar durch q-basierte Linkbehandlung.
2. **Kanal 26°:** m7-Klasse (⅓) ist nach Schur exakt querblind (Gt22 = 0), ihr Querrest
   |P2| übersteigt das Ziel zu 99,81 % → Quergate verwirft; m6 allein müsste Faktor 2,00
   der wahren Wandschubspannung melden — liefert kein Gleichgewichtsmodell.
3. **Kanal 45°:** KEIN Autoritäts-/Flächenproblem — die lösbare m4-Klasse trägt die wahre
   Fläche EXAKT (Σfaca = 1,0000·A_wahr), nötiges τ_w nur 1,08·u_τ². Das Defizit
   (Faktor 29) ist ein ZUSTANDSPROBLEM: die Treppe wirkt als raue Wand (u_t bei ~14 %
   des Log-Gleichgewichts, Re_τ IST 7930 statt 5186), die Bilanz schließt über
   ungebuchte BB-Spitzen. Publizierte Ursache: Asmuths eigene Spann-Bedingung; die
   Literatur hat das nie linkbasiert gelöst.

## Kandidaten auf Codeebene

| Weg | Eingriffspunkt | heilt Fahrzeug | 26° | 45° | Architekturkosten |
|---|---|---|---|---|---|
| **W1 ELIBB 18-Link** (q je Link, u_W=u_s) | `apply_facette_imem` Pass 2 (kernel.cpp:2073–2098): ersetzt q_i UND α je Link | **97 % + Normallage** | t2 der m8 + **Zustand** | **Zustand** (Rauigkeit ÷14, s. Rechnung) | mittel: fac_q-Puffer (18×uchar), ein Kernelblock, Buchung B3; single-node, EP-fähig, keine Nachbar-Writes, kein Atomic |
| W2 Stufen-Cluster (m6+m7 gemeinsamer Solve) | neuer Facetten-Kernel ODER Scatter statt In-Register-Gather (kernel.cpp:2082–2087) | über W1 hinaus nichts | Rang voll (Gt22 0→1,11) + Fläche exakt | nichts (Zustand bleibt) | HOCH: zweiter Pass, Link-Ownership, Synchronisation |
| W3 IVW-Zell-Kraftterm (Soll − Ist über Guo-Forcing) | Kollision `fxn +=` (kernel.cpp:2202 ff.); Ist-Austausch liegt seit heute korrekt in F (update_force_field, gleiche Linkmenge) | 100 % richtungsfrei | ja | ja (kompensiert Rauigkeit) | niedrig — ABER erste Ordnung an der Grenzfläche, dokumentierte u-Unterschätzung (Kuwata/Suga), Doppelbehandlungs-Gefahr mit iMEM |
| W4 Vollrekonstruktion (Maeyama) | ersetzt `apply_facette_imem` KOMPLETT | ja | ja | ja | SEHR HOCH: validierter K2-Ebene-Pfad wird weggeworfen, Image-Point braucht Nachbar-Makrofelder (t−1-Versatz), größter Umbau |
| W5 Deklaration + ehrliche Buchung (tote, flächenredundante Facetten als "nur Geometrie" markieren) | Host (`baue_facetten`-Klassifikator `eigene_links` existiert) + Akkumulator | – | – | K2 misst das Richtige | minimal, kein Feldeffekt |

## Die Nachrechnungen, die entscheiden

1. **ELIBB wirkt auf das 45°-Zustandsproblem.** HWBB reflektiert jeden Link an der Mitte
   → effektive Wand = Treppe, Amplitude 0,354 dx normal. ELIBB reflektiert an der wahren
   Ebene → Restrauigkeit = q-Quantisierung (Bucket/14): 0,036–0,051 dx. **Faktor ~14
   weniger geometrische Rauigkeit.** Der Geometrie-Agent hat ELIBB für 45° nur auf der
   BUCHUNGS-Achse bewertet ("liefert exakt nichts") — auf der ZUSTANDS-Achse, wo das
   45°-Problem tatsächlich sitzt, ist es der direkte Hebel. Beide Aussagen sind wahr
   und widersprechen sich nicht.
2. **Die V1-Präzedenz "Treppe trug 2 von 20 Punkten" entkräftet die 18-Link-Fassung
   nicht:** V1-ELIBB_F lief nur über die AXIALEN Paare (V1-kernel.cpp:2838,
   `ii=1..5`) — an einer y-z-Treppe sind die tangentialtragenden Links diagonal.
3. **q=0,5-Gate ist bitscharf möglich:** (2q, 1−2q) = (1.0, 0.0) exakt; einzige
   Ausnahme −0.0 → +0.0 (Harness prüft das explizit).
4. **W3 als Erstlösung wäre verfrüht:** der Ist-Austausch je Zelle existiert zwar seit
   heute korrekt (Bewegtwand-Fix, gleiche Linkmenge), aber ein Kraftterm ÜBER dem
   bestehenden iMEM erzeugt Doppelbehandlung, solange iMEM aktiv ist; als Ersatz wäre
   er erste Ordnung und würde den validierten Ebene-Pfad aufgeben.

## ENTSCHEIDUNG

**W1 (ELIBB 18-Link, Bausteine B0–B4) ist die Hauptlösung, W5 läuft als billiger
Begleiter sofort mit.** Begründung: W1 ist der einzige Weg, der gleichzeitig
(a) die 97-%-Klasse des Fahrzeugs heilt — das eigentliche Ziel —,
(b) das 26°-t2-Problem und
(c) das 45°-Zustandsproblem adressiert, und das
(d) ohne Architekturbruch (single-node, Esoteric-Pull-kompatibel, kein Scatter,
kein Atomic, V1-Vorlage mit allen bekannten Fallen-Fixes vorhanden).

- **W2 (Cluster) zurückgestellt:** erst falls nach ELIBB die 26°-m7-Querblindheit K2
  messbar blockiert. Kein Fahrzeugnutzen über W1 hinaus, hohe Kosten.
- **W3 (Kraftterm) ist die benannte Reserve:** falls der 45°-Arm zeigt, dass der
  Zustand NICHT ausreichend heilt, kommt der IVW-Term als separater, komponierbarer
  Baustein — dann mit dem heute reparierten F-Feld als Ist-Messung.
- **W4 verworfen** (wirft validierte Maschinerie weg, größtes Risiko).

## Erwartete Messbilder je Arm (zur Abnahme, nicht als Orakel)

- kipp0: bitgleich (q≡0,5-Gate) — jede Abweichung ist ein Stopp.
- Kugel, u_s=0: Cd von 0,717 (BB) Richtung Achenbach-Band 0,45–0,5; Δm im Gelb-Band.
- kipp26: Slot 13 fällt um die m8-Population; K2 steigt über das A/A-Rauschband.
- kipp45: Slot 13 bleibt (Buchung!), aber u_t/Re_τ nähern sich dem Ziel und die
  m4-Buchung wächst — K2 steigt über den Zustandskanal. Bleibt K2 < Schwelle bei
  erholtem u_t → W3-Reserve zünden.
- Fahrzeug 8 mm erst nach Kugel; 4 mm nur mit explizitem Go.

## Offen (blockiert den Start von B0/B1 nicht)

- Ebene-Wand-Agent (Modellterm 0,4–3,8 %) und die frische Auflösungsreihe k2_n* —
  separater Track, entscheidet NICHT über W1.
- Eq.-25e-Widerspruch im Wissensspeicher: operativ entschärft (PoU-Blende + q=0,5-Gate
  entscheidet empirisch); dokumentiert in AUDIT-BEFUNDE.md.

## Heikos Frage: "Koennen nicht die FACETTEN selbst die Treppenrauigkeit ausloeschen, ganz ohne ELIBB?"

Die Idee war der Grund fuer den Facettenweg -- und sie ist als ZIEL richtig. Die Antwort
zerfaellt in eine Struktureinsicht:

**Die Facette hat heute nur die halbe Wand in der Hand.** Jede Wandbehandlung hat zwei
Seiten: das ZIEL (was soll die Wand tun -- tau_w aus Spalding auf n/y_w der Facette) und
den AKTOR (womit wird es aufgepraegt -- die Linkpopulationen). Die Facette beherrscht
heute nur die Zielseite. Der Aktor ist der rohe Bounce-Back, und der reflektiert an der
LINKMITTE -- die effektive Wandgeometrie ist damit die TREPPE, egal wie perfekt die
Facette ihr Ziel rechnet. iMEM ADDIERT nur Impuls auf diese Treppe (additives q_i,
kernel.cpp Pass 2); es verschiebt den Reflexionspunkt nicht. Empirisch belegt: TROTZ
aktivem iMEM liegt u_t an der 45-Grad-Treppe bei ~14 % des Gleichgewichts -- die
Rauigkeit kommt vom unbehandelten BB-Anteil, und an den kommt das heutige Facettenschema
konstruktiv nicht heran (Normalinjektion ist aus gutem Grund genullt, Gl.-28-Entscheid;
und die alpha-Identitaet toetet Einzellink-Autoritaet exakt).

**ELIBB ist deshalb kein Konkurrent der Facetten-Idee, sondern ihre VOLLENDUNG:** q je
Link wird AUS DER FACETTE gelesen (q = y_w / (-n . c) -- Schnitt der Facettenebene mit
dem Link) und traegt die Facettengeometrie in den Aktor hinein. Der Reflexionspunkt
wandert von der Linkmitte auf die Facettenebene; die effektive Wand IST dann die
Facette. "Alles direkt die Facetten nehmen" heisst technisch genau das. Der Voxelizer
bleibt die einzige Geometriequelle (nie STL), die Facette die einzige Wandwahrheit.

**Die Idee in Reinform bleibt als Eskalation benannt:** die Voll-Rekonstruktion aller
19 Populationen der Wandzelle aus Facettendaten (W4, Maeyama-Schule) ist "nur noch
Facette, gar kein Link-BB mehr". Sie ist verworfen als ERSTER Schritt (wirft den
validierten Ebene-Pfad weg), nicht als Endpunkt: heilt ELIBB+iMEM den 45-Grad-Zustand
nicht ausreichend, ist W4 der facettentreue naechste Schritt -- vor jedem Kraftterm.

**B1-Designentscheid daraus:** q kommt in Stufe 1 aus der ZELLEIGENEN Facettenebene
(fac_geo n/y_w -- null neue Geometriemaschinerie, kipp0 liefert exakt q=0,5, und es ist
woertlich "die Facette als Wand"). Das geglaettete Remesh-Netz (P1, Ray-Triangle) bleibt
Stufe 2, falls das Kugel-RMS-Gate mehr Genauigkeit verlangt. Quantisierung uchar/254
statt V1s 4-Bit/14: q=0,5 bleibt exakt (127/254), Restrauigkeit sinkt von 1/28 auf
1/508 Linklaenge.

## REVISION W2 (nach Pruefagent, 2026-08-25 nachmittags)

Der Pruefagent hat die selbst gestellte K.O.-Frage bestaetigt: mit u_W = u_s IN der Blende
waere bei q = 0,5 (Identitaet) der Wandmodell-Impuls an JEDER ebenen Partie ausgefallen --
Kanalboden, Fahrzeug-Unterboden. Revision: **die Blende ist rein geometrisch (u_W = 0)**
und verschiebt nur den Reflexionspunkt auf die Facettenebene; den Wandmodell-Impuls traegt
weiterhin der bestehende Additivterm samt alpha (dessen Mathematik nur am Additivterm
haengt und exakt bleibt). Kein Stapeln: die Blende traegt kein u_W. Bei q = 0,5 kollabiert
der Pfad BITGLEICH auf das heutige iMEM -- der kipp0-Anker prueft wieder echte
Bitgleichheit (qb==127-Kurzschluss). Die "ersetzt q_i und alpha"-Formulierung des
urspruenglichen B2-Plans ist damit ueberholt.

Weitere Pruefbefunde eingearbeitet: F1 (Guards standen in totem #ifndef-Block -- zweimal;
jetzt unbedingt), F2 (fac_q fehlte in der stream_collide-Signatur -- OpenCL-Compilefehler),
W3 (Harness-nebb nutzte feq der falschen Richtung; korrigiert, Drift neu gemessen:
+5,4e-4 / +8,0e-4 statt -1,1e-3 / -2,0e-4), H1 (Statik-Symmetrie facetten_test).

**W1-Kennzeichnung bis B3:** im ELIBB-Arm sind fac_tau[1..5], Reibungs-Cd und Delta-m
"Modell-Soll, kein Ist" -- die Blende wirkt ungebucht. K2 wird im ELIBB-Arm ueber
Feld-CSVs bewertet (u_t-Gleichgewicht, Re_tau, Profile), nicht ueber den fac_tau-Pfad.
B3 (Buchung Sum c(fhn_neu - fpre) je Facette) ist der naechste Baustein.
