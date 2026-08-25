
## 2026-08-25, DRITTES MAL: timeout hat die laufende Serie gekillt
g4-Serie (Kipp-Laeufe mit QDIAG=2) im Vordergrund mit timeout=1500000 gestartet -- das
Werkzeug klemmt auf 600000, SIGTERM nach 10 min, k_g4_26 starb bei 90 %, g4_45 startete
nie. Dieselbe Falle wie am 24.08. REGEL AB JETZT OHNE AUSNAHME: jede Serie mit
Gesamtlaufzeit > 8 min laeuft run_in_background, NIE im Vordergrund.
