
## 2026-08-25, DRITTES MAL: timeout hat die laufende Serie gekillt
g4-Serie (Kipp-Laeufe mit QDIAG=2) im Vordergrund mit timeout=1500000 gestartet -- das
Werkzeug klemmt auf 600000, SIGTERM nach 10 min, k_g4_26 starb bei 90 %, g4_45 startete
nie. Dieselbe Falle wie am 24.08. REGEL AB JETZT OHNE AUSNAHME: jede Serie mit
Gesamtlaufzeit > 8 min laeuft run_in_background, NIE im Vordergrund.

## 2026-08-25: Mehrzeiliges non-greedy-Regex hat apply_facette_imem KOMPLETT gefressen -- und ich habe es COMMITTET
Beim Rueckbau der sechs ELIBB-Rueckfallstellen spannte re.S-non-greedy ueber ganze
Funktionsbloecke; der C++-Build blieb gruen (Kernel ist ein String!), der Schaden zeigte
sich erst als OpenCL-JIT-Fehler im Lauf. Der kaputte Stand war schon committet. REGELN:
(1) In kernel.cpp NUR exakte Ein-Block-Strings ersetzen, NIE mehrzeilige Regex ueber
R()-Grenzen. (2) Nach jeder kernel.cpp-Aenderung laeuft VOR dem Commit ein JIT-Kurzlauf
(kanal N=20, 1 Serie) -- der C++-Build beweist bei String-Kerneln nichts.
