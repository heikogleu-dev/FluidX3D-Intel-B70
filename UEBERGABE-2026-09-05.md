# ÜBERGABE — Stand 05.09.2026, 09:15

## ENTSCHIEDEN (05.09., Lauf vs_x8, 8-mm-Fahrzeug, B70, gepaart, 150 Proben)

**Die (A)/(B)-Frage ist beantwortet: 85 % des Cz-Schadens kommen von den neu abgedeckten Zellen, die Injektionsform ist unschuldig.**

| cz_druck_rest | Wert | |
|---|---|---|
| Basis (ALPHA2) | −0,14709 | |
| Arm X (Injektion wie Modus 1, Zellmenge wie Basis) | −0,12087 | **(B) = +0,026, 0,9 σ — im Rauschen** |
| Modus 1 (alle Links, roher Solve) | +0,02918 | **(A) = +0,150, 5,7 σ — der Schaden** |
| Summe (B)+(A) = +0,17627 | direkt gemessen +0,17627 | Konsistenzprobe auf die 5. Stelle |

**Kreuztabelle Gate × Solve-Zweig am Fahrzeug:** [94] (die Zellen, die X auf BB zwingt und Modus 1 anwendet) = 41.635.995,
davon **86,33 % roh Schur-exakt [79]**; [95] zu 99,76 % [79]. Instrumente alle grün (Slot 92 == beide angewandt,
Buchung 69 == Soll, Kaskade lückenlos, Zeilensummen/Nullspalten, Treffer 99,99 %).

**Damit bewiesen:** der Rang, den der Wegfall des Downdates gewinnt, ist ein **Scheinrang** aus der Mittelrichtung c̄ = S1/S0 —
der Richtung, in der ein Schlupf Masse durch die Wand schiebt statt Impuls zu übertragen. Das Downdate ist ein
**Konditionierungsschutz**. Nebenbefund: die Injektionsform ändert die *Reibung* messbar (cz_reib −0,008, 11,3 σ), nicht den Druck.

## 09:30 — ABDECKUNGSFRAGE GESCHLOSSEN: auflösungsunabhängig (wa_zensus4, 4 mm, Minimallauf)

| | 8 mm | 4 mm |
|---|---|---|
| aktive Facetten | 719.574 | 3.129.185 (×4,35) |
| Rang 2 / 1 / 0 (nach Downdate) | 47,20 / 33,63 / 19,17 % | **50,80 / 30,63 / 18,56 %** |
| **harter Kern** (roh Rang 0 = Einlink) | 16,39 % | **16,11 %** |
| Rangverlust durch Downdate | 36,26 % | 32,93 % |
| Flackerfenster / Prädikat-Divergenz | 2 / 0 | 7 / 0 |

Achtfache Zellzahl bewegt den harten Kern um **0,28 pp**. Für Einlink-Facetten ist `q_i ≡ 0` eine Identität (c̄ = c₁) —
kein Verfahren, keine Auflösung. Rang 2 gewinnt 3,6 pp; real, aber es holt die 42,8 % nicht zurück.

## GESCHLOSSEN — Durchgang C ist algebraisch tot (Planung 05.09., im Code gegengeprüft)

Jede Richtung, die die Nachbar-Union neu beiträgt, erfüllt `m + c_k − c_i ∈ S` **und** `m − c_i ∉ S` — sie ist also eine
Richtung, in der **diese** Zelle keine Wand hat; `fhn[i]` trägt dort eine Population aus Fluid. Auf der ebenen Wand (47 % der
Facetten) ist die Union sogar identisch der eigenen Menge. Gegenprobe im Code: jeder `store_f` schreibt in die eigene Zelle,
es gibt keinen zellfremden DDF-Schreibpfad. Heikos Kantenbedingung hätte genau die Fälle weggeschnitten, in denen überhaupt
etwas zu holen wäre — Kompatibilität und Rangzugewinn schließen sich aus.

## OFFEN — nur noch das ZIEL (Schicht 3)

Die (A)/(B)-Messung nennt den Ort: **an den Eckzellen ist P kein Reibungsaustausch, sondern Druckschub auf rückwärts gewandte
Stufenflächen** (Klassentabelle: Slip dort antiparallel zur Strömung). Das Modell ersetzt einen Druck durch eine Reibung, und
das kostet 85 % des Cz-Schadens. Nächster Schritt: P in Druck- und Reibungsanteil zerlegen und nur den deviatorischen Rest
ersetzen. Der Kernel misst den Druckanteil bereits — Offset-Histogramm Slots 49–58, `2(ρ−1)(S1·t1)`.

**Sichtdiagnose für Heiko:** `export/vs_x8/zensus_facetten.vtk` (38 MB) — ein Punkt je Facette, Skalare `rang_downdate`,
`rang_roh`, `n_wandlinks`, `entkoppelt`, `log10_lmin_lmax`, Vektor `normale`, Gitterkoordinaten. Zeigt, wo Rang fehlt und
wie die Nachbarn orientiert sind.

**Perf/VRAM (nicht gebaut):** Linkmaske (18 Bit) je Facette in die zwei toten `fac_geo`-Felder → null zusätzlicher VRAM, spart
18 Nachbar-Flag-Loads je Facettenbesuch; Rang-0-Facetten (16 %) könnten den Solve überspringen. Effekt unbekannt — beim
nächsten 4-mm-Lauf zuerst Baseline (MLUPs, freier VRAM via `werkzeuge/vram_sammler.sh`), dann A/B.

## Code-Stand

- HEAD `1aab3fc`, Binary aus `59aa64c`. Baum sauber, Queue frei.
- Neu am 04.09.: Zielerfüllungs-Instrument (Slots 81–91), statischer Klassenzensus + Wanderungsmatrix, Kreuztabelle
  Gate × Solve-Zweig (96–116), Arm X (MASSE_ALLE=3), Zensus-VTK, Sammelbehebung von 26 latenten Fehlern.
  Alles diff-geprüft, Bitgleichheit der Basisarme belegt.
- **Neuer latenter Befund (05.09.):** der Basis-Wächter prüft fehlende und abweichende BASIS-Schalter, aber **nicht
  zusätzliche**. Eine übernommene Vorlagenzeile trug `CFD_FACETTEN_DIAG=2` (Modus „Facetten bauen, Diagnose, `_exit(0)`",
  setup.cpp:5247) — der Lauf endete still nach dem Facettenbau mit rc=0, der Wächter schwieg. Behebung: jeder Schalter, der
  weder in der Basis steht noch in `CFD_BASIS_ABWEICHUNG` deklariert ist, mindestens als Warnung melden.
- **Bewusst offen:** `fac_geo`-Stride (24 MB tot), S7 (unverdrahtete Schalter in fahrzeug/fernfeld ansagen), S10
  (ALPHA-Soll-Test am Kanal), D4/D6/D8/D11 (Kosmetik).

## Verworfene Wege (alle gemessen, nicht vermutet)

PINV (wirksame Abdeckung 56,97 → 45,05 %) · SATGATE=0 (Klemme ×5.500) · ALPHA=0 (Δm ×12, Cz kippt) · Basiswahl t1
(2 Facetten; t1 ist ohnehin u_t/|u_t|, nicht frei) · globale Massenbilanz (kraftneutral, vor dem Bau verworfen) ·
MASSE_ALLE Modus 1 (+20 pp Abdeckung, aber (A)) · Modus 2 / f_0 (Bulk-Mode, f_0 ≤ 0 bei 2 %) · KRAFT=1 (NaN am Fahrzeug;
als (A)/(B)-Test strukturell ungeeignet) · Modus 4 (wäre SATGATE aus für 8–22 %).
