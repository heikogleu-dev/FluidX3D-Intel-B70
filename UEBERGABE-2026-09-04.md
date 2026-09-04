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

## Läuft / steht bereit

- `logs/vq_x_serie.txt` (iGPU, 7 Arme): Bitgleichheit 0/1/2 (Soll-Hashes im Kopf), X am Kanal, Leiter mit Fahrzeug-Schaltern.
- `logs/vr_x8_serie.txt` (B70, 1 Arm): **Arm X am Fahrzeug**, Basis-Soll 57,21 % aus vo_f08_aus.log. Start erst nach grüner
  Diff-Prüfung UND grüner vq-Serie. Abnahme im Serienkopf: X−Basis = (B), Modus1−X = (A), Summe ≈ +0,176.
- Wiederaufnahme: `pgrep -a -x FluidX3D`, `tail logs/queue_status.txt`; Auswertung wie in den Serienköpfen; Bezüge
  `export/vo_f08_aus`, `export/vm_masse8_an`.

## Regeln, die heute Zeit gekostet haben (Tagesprotokoll ~/llm-middleware/wissensspeicher-tagesprotokoll/2026-09-04.md)

- Diff-Prüfung nach dem Bau war den ganzen Tag übersprungen — nachgeholt: 13 Befunde, 4 mit Abbruchpotenzial.
- Der Kanal kipp26 ist für Wandmodell-Nebenwirkungen blind (alph=0 an der ebenen Wand). Leitersprosse nur mit
  Fahrzeug-Schalterkombination.
- „Anteil mit Modell" ist keine Gütezahl; nur Treffer × Abdeckung, gepaart, mit Cz-Richtung gegen OF13.
