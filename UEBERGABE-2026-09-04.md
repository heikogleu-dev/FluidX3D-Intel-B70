# Übergabe 04.09.2026 (abends) — Wandmodell-Abdeckung, Stand und nächster Schritt

**Fassung:** FluidX3D-v2, master. Letzter geprüfter Commit `f74f8f9`. Darüber liegt **Arm X** (Commit direkt nach dieser
Datei, gekennzeichnet UNGEPRÜFT — Diff-Prüfung und iGPU-Serie liefen beim Schreiben noch).

## Was heute gemessen wurde (8-mm-Fahrzeug, B70, gepaart, 150 Proben, Block-SEM/8; Basis: cz_druck_rest −0,14709, wirksam 56,97 %)

| Hebel | Ergebnis | Status |
|---|---|---|
| PINV | wirksame Abdeckung 56,97 → 45,05 % | widerlegt |
| SATGATE=0 | Geschwindigkeitsklemme ×5.500 | abgelehnt |
| ALPHA=0 | Δm +1,36 Mio, cz +0,113 | abgelehnt |
| Basiswahl t1 | 2 Facetten; t1 ist ohnehin u_t/|u_t|, nicht frei | tot |
| Globale Massenbilanz | vor dem Bau verworfen (kraftneutral) | tot |
| **MASSE_ALLE=1** (β3 über alle 19 Links) | wirksam **77,37 %**, aber cz **+0,029** | disqualifiziert |
| MASSE_ALLE=2 (alles auf f_0) | f_0 ≤ 0 bei 2 % (Slot 93), cz +0,675, Bulk-Mode | verworfen |
| KRAFT=1 | Fahrzeug NaN bei 0,174 s; als (A)/(B)-Test strukturell ungeeignet | verworfen |

**Instrumente, abgenommen:** Zielerfüllung (Slots 81–91, `r = phi1/(−τ·twe)` je Besuch, physikneutral, Gegenprobe eingebaut);
statischer Klassenzensus (Rang je Facette vorab, Wanderungsmatrix roh→Downdate). Zensus 8 mm: Downdate kostet 36,26 % der
Facetten eine Rangstufe; informationsfreier Kern 16,39 % (Einlink).

**Muster, dreifach belegt:** jede Variante, die gegen die ROHE Momentenmatrix löst, schiebt cz ins Positive (weg von OF13 −1,301).
Offene Frage: (A) das volle Ziel an den neu abgedeckten Eckzellen ist der Schaden (dort ist P Druck-Schub, kein
Reibungsaustausch — Klassentabelle: Slip antiparallel zur Strömung) — oder (B) die Injektionsform an den gemeinsamen Zellen.

## Arm X — Ergebnis der iGPU-Serie (vq_x_serie, 19:00–20:10), Binary 0de4997

| Arm | Ergebnis |
|---|---|
| vq_aus / m1 / m2 | **BITGLEICH** zu vn_* (Hashes 113354303738885232 / 3775874723383551782 / 9660637873999918948) — Schattencode ist ausgeschaltet inert ✓ |
| vq_x (Kanal, Basis-Schalter) | 69 == Soll(+94) ✓, Kaskade lückenlos ✓, 92 == Wirkpfad−Skips−69 ✓ (33.345.725). **[95] = 13.033.014 = 8,2 % des Wirkpfads** ✗ (Soll < 0,5 %). Abdeckung X 20,90 % gegen Basis 26,82 % |
| vq_leiter_aus (Fahrzeug-Schalter am Kanal) | rc=1: **K2 verletzt** (Reibungspfad >1 % von der Kraftbilanz) — vorbestehende Kanal-Abnahme, nicht X; angewandt 37,07 %, Treffer 99,98 % |
| vq_leiter_m1 | rc=0, angewandt 57,67 %, Treffer 100 % |
| vq_leiter_x | 69/Kaskade/92 ✓, **[95] = 34.560.921 = 21,7 %** ✗, Abdeckung 22,60 % gegen 37,07 % |

**Deutung:** [95] = „roh sagt Rückfall, Schatten hätte angewandt". Rangmäßig ist der Schatten eine Obermenge (das Downdate senkt
den Rang nur), also kommt [95] aus den **Gates**: der rohe Solve liefert an diesen Zellen ein s, das SATGATE/sn-Gate reißt, der
ALPHA2-Schatten-Solve nicht. Mein Merge `rueckfall |= a2_rueckfall` (ODER) macht X damit zur **Schnittmenge** beider Abdeckungen,
nicht zur Basis-Zellmenge. X als „(B) rein" ist so NICHT gegeben. Konsequenz für morgen (Entscheid steht aus, nicht gebaut):
- **Modus 4 = Ersetzen statt ODER** (`rueckfall = a2_rueckfall`): wo der Schatten anwendet, wird der rohe Solve UNGEGATET
  angewandt. Das ist der ehrliche (B)-Test auf der Basis-Zellmenge — mit dem Risiko, dass ungegatete rohe Slips an 8–22 % der
  Besuche instabil werden (vgl. SATGATE=0: Klemme ×5.500). Wenn das passiert, ist das selbst ein Befund: der rohe Operator liefert
  an genau den Zellen unphysikalische s, die das Downdate schützte.
- Vorher **Kreuztabelle nach Gate-Ursache** (raw-Gate 10 / 16 / Rang) für [95] mitzählen, sonst bleibt „Gates" Vermutung.
- **Diff-Prüfung von Arm X (zweiter Anlauf, 20:20): NICHTS GEFUNDEN.** Block A zeichengleich zur ALPHA2-Basis, Präprozessor
  paarig, 69-Buchung hinter dem Merge, Soll +94 exakt (126.241.015 = 53.195.580 + 71.607.565 + 1.437.870).
- **[95]-Ursache bewiesen:** am Kanal [16]=0, [64]=0, [13]=0 → [95] liegt vollständig in SATGATE-[10]. Der rohe 2×2-Solve
  bedient die schwache Mittelrichtung c̄=S1/S0 (Steifigkeit ~Dd·B², dett knapp über 1e-4·Gt11·Gt22) → Slip reißt das Budget;
  der Schatten-Skalar setzt s2=0 und passiert. **Das ist eine Konditionierungsaussage über den rohen Operator**, nicht nur Buchhaltung.
- **Modus 4 (Ersetzen statt ODER) NICHT bauen:** er wäre „SATGATE aus" für genau diese 8–22 % — ungegatetes |s1|>2·budget·ut
  und sn=0 (für diese Zellen nie gerechnet). Kein sauberer (B)-Test.
- **Stattdessen morgen:** (1) `vr_x8` auf der B70 SO WIE X IST — misst (B) auf der SCHNITTMENGE; die beiden Abnahme-Wächter
  ([95] > 0,5 %, Abdeckung ≠ Soll) sind auf Warnung gestellt und das Binary ist neu gebaut (nur Host, kein Kernel).
  Konsistenzprobe bleibt: (X−Basis) + (Modus1−X) ≈ +0,176. (2) Kreuztabelle „Gate nach Solve-Zweig" ([81]/[82], lbm.cpp:352
  seit heute früh als fehlend deklariert) bauen — sie trennt Klasse B (roh exakt ↔ Schatten Schur) von Klasse C (roh Schur-exakt
  ↔ Schatten Skalar) und beantwortet, ob der rohe Operator an c̄ schlecht konditioniert ist. Das wäre der eigentliche Befund.

## 20:55 — Kreuztabelle Gate × Solve-Zweig GEBAUT (Slots 96–116, Puffer 128) — **Diff-Prüfung 21:25: nichts gefunden**

Kernel: `uint zweig` (1=[78] 2=[79] 3=[12] 4=[14]/[80]) an den Kaskadenzweigen, an jedem Gate `hits[Basis+zweig]`; unter X die
[94]/[95]-Zellen nach rohem Zweig, [116] = [95] mit rohem Rang 0. Host `bericht_gate_kreuztabelle` mit Zeilensummen-Abnahme
und konstruktiven Nullspalten (Quergate × exakt, sn-Gate × entkoppelt). NUR Zähler — alle Arme müssen bitgleich bleiben.
**Morgen:** (1) `logs/vs_kreuz_serie.txt` (iGPU, 5 Arme, Soll-Hashes im Kopf) → (2) `logs/vs_x8_serie.txt` (B70) = Arm X am
Fahrzeug + Tabelle in einem Lauf. Vorhersage Klasse C: [113] ≈ [95]. Diff-Prüfung sauber (nur Kommentar-Reste, behoben). Startklar.

## 21:15 — Feld-Hash vor die Berichte gezogen (Kanal + Kugel, host-only)

Die X-Arme hatten keinen FELD-HASH, weil ein abbrechender Wächter (`print_error` = `exit(1)`) im Bericht vor der Hash-Zeile
feuerte. Jetzt wird der Hash direkt nach dem Lauf gedruckt, vor jedem Bericht. Die Berichte ändern u nicht — die Werte
bleiben identisch, die 0/1/2-Sollhashes von heute gelten weiter. Ab morgen haben auch X-Arme einen Hash.

## 23:00 — Sammelbehebung der latenten Fehler eingecheckt (`b9dab75`), zwei Prüfrunden

26 Punkte aus den vier Audits des Tages: 3 Laufkiller (K2 FACETTEN=1/2-Abbruch in pruefe_kaskade, K3 Pur×MASSE_ALLE/KRAFT/PINV,
K1 zurückgenommen — Befund galt für einen älteren Stand), 10 stille Fehlmessungen (S1 KDIAG unter KRAFT=2, S2 Nullziel-Quergate,
S3 Slice-Legacy-Uhr ohne Zähler, S4 ELIBB=1 ohne Nullwächter, S5 BODEN_EQ_ABSTAND ohne Wirkpfad, S6 KDIAG nie alloziert,
S8 Normalrest unter MASSE_ALLE, S9 Zielerfüllung-Randfälle, D17 PINV-PSD), Rest Doku. Die Diff-Prüfung fand in meiner ersten
Fassung drei Fehler (Kommentar mitten in der Kernelzeile → QUERGATE-Build tot; K1 hätte den Abbruch eingebaut; S4 unerreichbar) —
behoben, nachgeprüft, sauber. **Bewusst offen:** D9 fac_geo-Stride (24 MB, Bitgleichheitsbeweis teurer als der Gewinn), S7
(unverdrahtete Schalter in fahrzeug/fernfeld ansagen), S10 (ALPHA-Soll-Test am Kanal), D4/D6/D8/D11 (Kosmetik).
**Abnahme:** `logs/vt_latent_serie.txt` (iGPU, 6 Arme) — läuft nach vs_kreuz; Kernel-Änderungen sind in den Hash-Armen nicht
emittiert (S1 KDIAG, S2 QUERGATE, D17 PINV) bzw. nur Bericht (S8) → aus/m1/m2 müssen bitgleich bleiben.

## 23:15 — KREUZTABELLE GEMESSEN: Klasse C zu 100 % (vs_kreuz, iGPU, Binary 6d9dd3e)

0/1/2 bitgleich, vs_x-Altslots exakt wie vq_x, alle Zeilensummen und Nullspalten halten. **[95] nach rohem Zweig: Schur-exakt
[79] = 100,00 %** — am Kanal mit Basis-Schaltern UND mit der Fahrzeug-Kombination. Die Zellen, an denen der rohe Solve das
Sättigungsgate reißt, sind ausnahmslos die, die das Downdate von Rang 2 auf Rang 1 senkt: der rohe 2×2-Solve bedient die schwache
Mittelrichtung c̄ = S1/S0 (Steifigkeit ~Dd·B²), und genau diese Richtung entfernt `G' = G − (6/S0)·S1S1ᵀ`.
**Das Downdate ist ein Konditionierungsschutz, nicht nur Massenbuchhaltung.** Der Rang-Weg über den Wegfall des Downdates ist damit
tot — der gewonnene Freiheitsgrad ist die Richtung, die das Modell nicht tragen kann. Abdeckungsgrenze unter konditioniertem Solve
≈ ALPHA2-Abdeckung (57 % am 8-mm-Fahrzeug). Darüber hinaus nur (A) das Ziel an Eckzellen (Druck vs Reibung in P trennen) oder die
Linkmenge erweitern (Rang 2 aus echter Geometrie). vs_x8 (B70) bleibt sinnvoll: es misst (B) auf der Schnittmenge — ist (B) ≈ 0,
ist der gesamte Cz-Schaden von Modus 1 der c̄-Slip an den neu abgedeckten Zellen.
Erster X-Hash (nach der Hash-Verschiebung): vs_x 17935634836335592429, vs_leiter_x 4168733461996404659. vs_x rc=1 nur durch die
vorbestehende Kanal-Abnahme K2.

## 23:40 — Zensus-VTK-Export (host-only, `CFD_FAC_ZENSUS_VTK=1`) und Plan für die Nachbar-Messbasis

`export/<Lauf>/zensus_facetten.vtk`: ein Punkt je aktive Facette (GITTERKOORDINATEN der Domäne, nicht Welt), Skalare
`rang_downdate`, `rang_roh`, `n_wandlinks`, `entkoppelt`, `log10_lmin_lmax`, Vektor `normale`. Zweck (Heiko): optische Diagnose,
wo Rang fehlt und ob der Nachbar auf derselben Fläche ähnlich orientiert ist. Am 4-mm-Fall ~200 MB → nur auf Anforderung.

**Korrektur meiner Formulierung:** tot ist nur der Rang-Weg *über den Wegfall des Downdates* (Klasse C, 100 %). Der Weg über eine
**geteilte Messbasis** — Wandlinks der Nachbarzelle auf derselben Fläche in den Solve dieser Zelle einbeziehen, jede Zelle behält Ziel
und Schlupf — lebt. Bedingung (Heiko): **nie** von einem Nachbarn borgen, dessen Normale anders orientiert ist (spitze Kanten). Die
Normalen sind statisch, „Nachbar kompatibel" ist also vorab entscheidbar und gehört in dieselbe VTK als Farbe, bevor gebaut wird.

**Perf/VRAM aus der Vorberechnung** (nicht gebaut): Linkmaske (18 Bit) je Facette in die zwei toten `fac_geo`-Felder → null VRAM,
spart 18 Nachbar-Flag-Loads je Facettenbesuch; Rang-0-Facetten (16 %) könnten den Solve überspringen. Effekt unbekannt — beim
nächsten 4-mm-Lauf zuerst die **Baseline** (MLUPs, freier VRAM via `werkzeuge/vram_sammler.sh`) mitmessen, dann A/B.

## Läuft / steht bereit

- `logs/vq_x_serie.txt` (iGPU, 7 Arme): Bitgleichheit 0/1/2 (Soll-Hashes im Kopf), X am Kanal, Leiter mit Fahrzeug-Schaltern.
- `logs/vr_x8_serie.txt` (B70, 1 Arm): Arm X am Fahrzeug, Basis-Soll 57,21 %. **NICHT starten**, solange [95] am Kanal 8 % ist —
  der Lauf würde die Abdeckungs-Abnahme reißen. Erst Merge-Entscheid (Modus 4) + Diff-Prüfung. Abnahme im Serienkopf: X−Basis = (B), Modus1−X = (A), Summe ≈ +0,176.
- Wiederaufnahme: `pgrep -a -x FluidX3D`, `tail logs/queue_status.txt`; Auswertung wie in den Serienköpfen; Bezüge
  `export/vo_f08_aus`, `export/vm_masse8_an`.

## Regeln, die heute Zeit gekostet haben (Tagesprotokoll ~/llm-middleware/wissensspeicher-tagesprotokoll/2026-09-04.md)

- Diff-Prüfung nach dem Bau war den ganzen Tag übersprungen — nachgeholt: 13 Befunde, 4 mit Abbruchpotenzial.
- Der Kanal kipp26 ist für Wandmodell-Nebenwirkungen blind (alph=0 an der ebenen Wand). Leitersprosse nur mit
  Fahrzeug-Schalterkombination.
- „Anteil mit Modell" ist keine Gütezahl; nur Treffer × Abdeckung, gepaart, mit Cz-Richtung gegen OF13.
