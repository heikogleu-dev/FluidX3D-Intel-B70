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
