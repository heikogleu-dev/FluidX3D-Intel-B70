# Prognose vor dem Lauf — grober Kanalpunkt N=20 (2026-08-24)

**Zweck:** dritter Punkt der Delta-Reihe, um die MSD-Lesart zu bestaetigen ODER zu toeten.
Bewusst GROEBER statt feiner: N=152 kostete rund 16 Stunden, N=20 kostet Minuten.

**Bisher gemessen** (beide auf Commit 6f223f5, identische kernel.cpp-Pruefsumme):
| | Delta+ | y1+ IST | Erwartung 0,714*kappa*y1+ | gemessen | Verhaeltnis | cf |
|---|---|---|---|---|---|---|
| N=38 | 182,6 | 91,3 | 26,7 | 39,9 | 1,49 | -55,4 % |
| N=76 |  98,9 | 49,5 | 14,5 | 15,1 | 1,05 | -53,5 % |

**PROGNOSE fuer N=20** (aus der Potenzanpassung Verhaeltnis ~ Delta+^0,57):
- u_tau-Faktor faellt weiter (0,669 bei N=38, 0,725 bei N=76 -- steigt mit Verfeinerung),
  bei N=20 also rund **0,62**; damit Delta+ rund **321**, y1+ rund **161**, Erwartung rund **47**.
- **Verhaeltnis rund 2,0** (Spanne 1,8 bis 2,3). Gemessenes nu_t/nu_0 damit rund **95**,
  also im Bin 60-120 mit Auslaeufern nach 120-240.
- **cf rund -57 bis -58 %** (Trend -53,5 -> -55,4 bei Vergroeberung fortgeschrieben).

**FALSIFIKATION:** Liegt das Verhaeltnis bei N=20 NICHT ueber 1,49, ist die MSD-Lesart
widerlegt -- dann ist der Abfall 1,49 -> 1,05 kein Aufloesungstrend, sondern etwas anderes
(Zufall zweier Punkte, oder ein Artefakt der Bin-Grenzen).

**Codefassung:** aktueller HEAD a88880c. Gegenueber 6f223f5 kam nur die Warmlaufsperre dazu,
und die ist ueber `CFD_SGS_DIAG_AB` gegatet (Default 0 -> `t>=0` immer wahr -> Zaehlverhalten
BITIDENTISCH zu 6f223f5). Ich setze sie bewusst NICHT, damit alle drei Punkte gleich
behandelt sind (alle drei enthalten die 20 ETT Warmlauf im Zaehler).
