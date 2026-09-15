# Gemischter Geschwindigkeitssatz D3Q19/D3Q27 nur an Wandzellen — Zensus zuerst

**Eingang:** CC-Auftrag von Heiko, 15.09.2026 spät, zur Prüfung abgelegt. **Status: Idee mit Gate, NICHT begonnen.** Priorität: nach der
APG-/Mozaffari-Linie (TODO.md §1 Punkt 4), wie im Auftrag selbst verlangt. Der Originaltext steht unverändert in Teil B; Teil A ist die
Prüfnotiz der Hauptsitzung.

## Teil A — Prüfnotiz (Hauptsitzung 15.09.2026, gegen den Bestand, nicht gemessen)

1. **Ausgangszahl „44 % aller Gate-Rückfälle sind der Rangboden“: im Repo und im Wissensspeicher nicht gefunden.** Die heutige
   8-mm-Standardzeile (`logs/kl_z2m_dd8_a_b70.log`, [SLOTS] t = 0,300 s) zerlegt den Rückfall 23,3 % in u_s-Gate 3,0 %, ohneTang 16,4 %,
   Rang0 2,5 %, sn-Gate 1,3 %. Vor Stufe 1 die Quelle der 44 % benennen oder am heutigen Stand (4 mm und 8 mm) neu erheben; welche
   Slot-Klasse „Rangboden“ meint (Rang0 allein oder ohneTang), festlegen.
2. **Die Ein-Link-Klasse ist flächenredundant (Messung 07.09.2026, Wissensspeicher „KORREKTUR + PLANUNGSURTEIL Ein-Link-Kandidat“,
   `logs/planungsagent_einlink_2026-09-07.md`):** am kipp26 bedienen die 4- und 8-Link-Klassen schon 0,992 · A_wahr, am kipp45 1,000,
   an der Kugel 1,087; Ablehnungsgrund (4) aus `FACETTEN-IMEM-3X3.md` Gl. 28 bleibt. Wer die Ein-Link-Zellen über Eckrichtungen in den
   Rang hebt, bucht ihre Fläche ein zweites Mal (Stand 05.09.: Flächenüberschuss 1,2537 der Ein-Link-Zellen). **Stufe 1 braucht deshalb
   ein zusätzliches Gate: Flächenbilanz der gehobenen Zellen gegen A_wahr**, nicht nur Anzahl und Kraftanteil.
3. **Rekonstruierte Eckpopulationen streamen nicht — der Rangvorteil ist dann vermutlich nur scheinbar.** iMEM prägt Wandschub über
   Populationen auf, die in die Fluidnachbarn laufen. Eine je Schritt aus lokalen Momenten rekonstruierte Eckpopulation erreicht keinen
   Nachbarn (die Nachbarn sind D3Q19 und haben keine Eckrichtung), der eingeprägte Impuls bleibt in der Zelle und wird im nächsten Schritt
   durch die Rekonstruktion wieder überschrieben. Der Auftrag nennt das selbst als Prüfpunkt („dann Streaming-Problem zurück“); nach
   meiner Lesart ist es der wahrscheinliche Ausgang. Tragfähig wäre erst ein D3Q27-Band mit echter Ecken-Advektion über mindestens eine
   Nachbarlage — dann gelten die 42 MB nicht mehr, und die Kopplungsfragen (Gleichgewicht, Geistmoden, P-TRT) werden Hauptarbeit.
4. **Bekannter Parallelbefund (04.09.2026):** „Rang heben durch Wegfall des Downdates“ ist tot (Konditionierungsschutz); als Weg darüber
   hinaus war damals genau „die LINKMENGE erweitern (Rang 2 durch echte Geometrie)“ notiert. Der Auftrag setzt also an einem offenen,
   nicht an einem verworfenen Pfad an.
5. **Stufe 1 ist trotzdem billig und lohnt:** Linkzahl-Histogramm und Eck-Solid-Test sind rein geometrisch (Host, Flags + Facettenliste,
   8 mm genügt fürs Gate, 4 mm zur Bestätigung). Datenbasis: `facetten_klassen.csv` braucht `CFD_FAC_KDIAG=1` (seit 11.09. aus, kostet
   laut TODO.md Anhang 1d 191 MiB VRAM bei 4 mm) — für den Zensus einen eigenen 8-mm-Lauf mit KDIAG fahren, nicht die Baseline ändern.

6. **Variante Heiko (15.09. spät): D3Q27 als Layerband oder enger Block um das Fahrzeug.** Das beseitigt Punkt 3 im Inneren: die
   Eckpopulationen advektieren im Band echt, rekonstruiert wird nur noch an der Band-/Blockgrenze zum D3Q19-Gebiet, weg von der Wand.
   Neue Kosten, alle ungemessen: Zellzahl des Bandes × 8 Populationen × 2 B (Rechengrundlage: am 12.09. an der STL gerechnet liegen 2,33 %
   des 4-mm-Nahkastens innerhalb 16 mm der Wand), ein zweiter Kollisionspfad bzw. Kernel, die Grenzrekonstruktion und die Frage, ob P-TRT
   und Wandmodell im Band ihre Eichung behalten. Nebeneffekt: in einem D3Q27-Block wären auch Operatoren möglich, die in der Literatur nur
   auf D3Q27 validiert sind (Cumulant/KBC — Recherche vom 16.06.2026 im ALTEN Projekt FluidX3D, für v2 nicht nachgeprüft). Der Zensus
   (Stufe 1) ändert sich dadurch nicht; für die Band-Variante kommt eine Zellzahl- und Speicherrechnung je Banddicke hinzu.

**Einschätzung in einem Satz:** Stufe 1 (Zensus mit zusätzlichem Flächen-Gate) ist sinnvoll und günstig; die reine Wandzellen-Variante
erwarte ich tot (rekonstruierte Eckpopulationen transportieren keinen Impuls), die Band-/Block-Variante ist physikalisch tragfähig, aber
ein eigenes Bauprojekt, dessen Preis erst eine Zellzahlrechnung je Banddicke zeigt.

## Teil B — Auftrag im Wortlaut (Heiko, 15.09.2026)

### Goal
Klären, ob ein **auf Wandzellen beschränkter D3Q27-Satz** den Rangboden des iMEM-Solves hebt, den wir als "unerreichbar" dokumentiert haben.
**Stufe 1 ist reine Auswertung vorhandener Daten, KEIN Code.** Bau erst nach Gate und Freigabe — und erst NACH dem APG-Wandgesetz (Begründung unten).

### Ausgangslage
- D3Q19: 18 Nicht-Ruhe-Links. **44 % aller Gate-Rückfälle sind der Rangboden** — ALPHA2 verlangt ≥3 Links für vollen Rang, ≥4 gekoppelt; bei einem Link erzwingt die Massenerhaltung, dass der injizierte Impuls identisch verschwindet, unter jedem Schema.
- Ein Drittel der 26°-Zellen hat genau EINEN Wandlink → iMEM kann dort beweisbar nie wirken.
- Volles D3Q27 verworfen: +27,3 % Speicher (8,3 GB), +42 % Verkehr.
- **Beschränkt auf Wandzellen ist die Rechnung völlig anders:** 2,62 M Facetten von 519 M Zellen = 0,5 %; 8 Extra-Links × 2 B × 2,62 M = **42 MB**.
- Die Adressierungsmaschinerie existiert: `fac_idx` als Bitmaske + Block-Präfixsumme (`fid = base + popcount`), integer-exakt und bitgleich abgenommen an allen Sprossen.

### Literaturstand (recherchiert 2026-09)
- **Das Konzept existiert**: Multi-Scale-Kopplung verschiedener Geschwindigkeitssätze (arXiv 2102.12559, arXiv 1007.4895). Motivation wörtlich: den höheren Satz über die ganze Domäne zu nutzen sei "rechnerisch aufwendig und verschwenderisch in Regionen, wo die Genauigkeit des niedrigeren Modells ausreicht". Kopplung machbar, weil beide Seiten LBM sind und das niedrigere Modell die Nichtgleichgewichts-Information behält, die das höhere braucht — anders als Kinetik-Kontinuum-Hybride.
- **ABER: publizierte Fälle koppeln wegen Mach-Zahl (D2Q9↔D2Q25) oder Knudsen-Zahl.** D3Q19↔D3Q27 wegen Wandmodell-Linkzahl ist in der Recherche NICHT gefunden worden.
- **Der ermöglichende Mechanismus ist die Regularisierung**: beim regularisierten Randschema werden alle Verteilungsfunktionen an Randknoten aus Gleichgewichts- und Nichtgleichgewichtsanteil REKONSTRUIERT, implizit beschränkt durch Dichte, Geschwindigkeit und Schergeschwindigkeitstensor (arXiv 2506.03905, LBM-Wandrandbedingungen für RANS). → Die 8 Eckpopulationen müssten also nicht gestreamt, sondern je Schritt aus den lokalen Momenten rekonstruiert werden. Das umgeht die Streaming-Schließung.
- **Der Rangvorteil ist geometrisch, nicht informationell**: ALPHA2 braucht Streuung der Linkrichtungen. Acht Eckrichtungen mehr heben den Rang auch dann, wenn ihre Populationen rekonstruiert sind.

### STUFE 1 — Zensus (Pflicht, kein Code, entscheidet alles)
Datenbasis: `facetten_klassen.csv` (mit `CFD_FAC_KDIAG=1`) und der Flag-Export des 4-mm-Laufs. Die Frage ist rein geometrisch.
1. **Linkzahl-Histogramm je Wandzelle** — wie viele Zellen haben 1, 2, 3, 4, … Wandlinks (heutige 18 Richtungen)? Besuchsgewichtet, getrennt nach `eigene_links ≥ 3` (echte Wandzellen) und `≤ 2` (zweite Reihe).
2. **DIE GATE-FRAGE:** Für jede Zelle unterhalb des Rangbodens zusätzlich die **8 Eckrichtungen** (±1,±1,±1) gegen die Solid-Maske testen. Wie viele dieser Zellen kämen damit auf ≥3 bzw. ≥4 Links? Ein Eckstrahl, der ebenfalls ins Solid zeigt, hilft nicht.
3. **Kraftgewichtung, nicht nur Anzahl** (Kontaktband-Lehre: 8× Artefakt gegen Nutzsignal): welchen Anteil an `cd_druck_rest`/`cz_druck_rest` tragen die Zellen, die profitieren würden? Über die facet-path-Zerlegung je Sample.
4. **Räumliche Verortung**: sitzen sie an Dachlinie, Motorhaube, Heckdeck (die 26°-Klasse, Ziel-Zellklasse von DETEPS) oder verstreut?

**GATE:** Bau nur, wenn die Eckrichtungen einen nennenswerten, **kraftgewichteten** Anteil der Rangboden-Zellen über die Schwelle heben. Bringen sie es nicht (weil sie ebenso ins Solid zeigen), ist die Idee tot — Befund dokumentieren, Akte schließen.

### STUFE 2 — Spezifikation (nur nach Gate + Freigabe)
- **Rekonstruktion statt Streaming.** Die 8 Eckpopulationen je Schritt regularisiert aus den lokalen Momenten (ρ, ρu, Π_neq) aufbauen, nach dem Rand-Regularisierungsschema. Damit gibt es keine Streaming-Schließung über die Gebietsgrenze — prüfen, ob das für die iMEM-Injektion ausreicht oder ob der injizierte Impuls in den Eckrichtungen persistiert werden muss (dann Streaming-Problem zurück).
- **Impuls- und Massenbilanz.** Die α-Korrektur (Σq = 0 UND Impuls exakt je Facette) muss über die erweiterte Linkmenge laufen. S0/S1 über **identische** Indexmengen wie q_i — dieselbe Falle wie beim ursprünglichen α-Bau.
- **Gleichgewichts-Inkonsistenz.** D3Q19 und D3Q27 haben verschiedene Gleichgewichte; D3Q19 hat bekannte Galilei-Invarianz-Defizite bei hohem Re (arXiv 1803.04937). Eigener Befund entlastet teilweise: D3Q19-I ändert genau einen Vierten-Moment-Term, Wirkung am Fahrzeug 0,01–0,26 %. → Größenordnung des Interface-Sprungs abschätzen, nicht annehmen.
- **Geistmoden.** P-TRT bei ω_g = 1,90 ist auf die drei geraden D3Q19-Geistmoden ausgelegt (von-Neumann auf 72³-k-Gitter). D3Q27 hat andere. Was gilt in der Mischzone?
- **Speicher/Adressierung.** 8 Extra-Populationen über die vorhandene Bitmasken+Präfixsummen-Maschinerie, analog `F` als Wand-Solid-Markerliste. Bedarf beziffern.
- **Wirkpfadzähler je neuem Zweig** (Iron Rule 1).

### STUFE 3 — Abnahme
- **Inertheitsbeweis:** Schalter aus → bitgleich zum Bezugslauf (alle Dateien, Feld und Kräfte).
- **Kanal-Bitanker** (kipp=0): darf sich nicht bewegen, wenn dort keine Rangboden-Zellen sind.
- **Kippkanal 26°** als Ziel-Detektor: Rückfallrate der 5-Link-Klasse vorher/nachher.
- **Kugel** als Erhaltungsprüfung: Δm muss in seinem Band bleiben.
- 8-mm-Fahrzeug-A/B, eine Variable, dann erst 4 mm nach Freigabe.

### Erwartung / Falsifikation (vor Stufe 1 festhalten)
| Ergebnis | Deutung |
|---|---|
| Eckrichtungen heben viele Rangboden-Zellen, kraftgewichtet relevant | Bau lohnt, Spezifikation weiter |
| Eckrichtungen zeigen ebenfalls ins Solid | Idee tot, Akte schließen |
| Zellen profitieren, tragen aber kaum Kraft | dokumentieren, nicht bauen |

### Prioritätsvermerk — NICHT vorziehen
Dies kauft **Rang und Abdeckung, nicht ein besseres τ_w**. Der DETEPS-Befund ist die Warnung: Abdeckung 82,5 → 93,85 % kostete **+0,011 Abtrieb**, also die falsche Richtung. Mehr Zellen, an denen ein APG-blindes Gleichgewichts-Wandgesetz wirkt, ist nicht automatisch besser. **Reihenfolge: erst APG/Mozaffari im τ_w-Ziel, dann mehr Zellen dafür.**

### Report (Markdown, in Chat zurück, NICHT committen)
1. Linkzahl-Histogramm, besuchsgewichtet, nach Zellklasse getrennt
2. **Gate-Antwort:** wie viele Rangboden-Zellen heben die 8 Eckrichtungen über ≥3 / ≥4?
3. Kraftgewichteter Anteil + räumliche Verortung (Bauteilzuordnung)
4. Gate-Verdikt mit Zahl begründet
5. Falls ja: Spezifikation nach Stufe 2, Speicherbedarf, Aufwand kategorisch
6. Literaturhinweis im Repo vermerken: Konzept existiert (Multi-Scale-Velocity-Set-Kopplung), diese Anwendung nicht publiziert
