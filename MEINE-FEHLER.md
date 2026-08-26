
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

## 2026-08-26, VIERTES timeout-Versehen -- Regel wird mechanisch
Die IGC-Dump-Serie im Vordergrund gestartet ("sind ja nur JIT-Kurzlaeufe") -- die
Dump-Schreiberei macht auch Kurzlaeufe langsam, Timeout nach 10 min, Serie tot (Schaden
diesmal null: Dumps lagen schon auf Platte, Census sauber). AB JETZT MECHANISCH: JEDE
lauf_queue.sh-Serie laeuft run_in_background, ohne Ausnahme und ohne Laufzeitschaetzung.

Nachtrag 2026-08-26 vormittags: Die Regel ist jetzt HART erzwungen -- ein
PreToolUse-Hook in .claude/settings.json (FluidX3D und FluidX3D-v2) blockiert jeden
Bash-Aufruf, der "lauf_queue" enthaelt und nicht run_in_background:true traegt, BEVOR
er ausgefuehrt wird. Erinnerung (Middleware/Doku) hat viermal nicht gereicht; diese
Fehlerklasse faengt man nur mechanisch am Werkzeugaufruf ab, nicht durch Kontext.
