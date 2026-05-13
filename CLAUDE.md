# CLAUDE.md — FluidX3D Fork Engineering Guidelines

## Engineering-Methodik — Generelle Regeln (gelten projekt-übergreifend)

### Regel 1: Skalen-Ladder

Von kleinstem isolierten Test zu voll-Setup hochskalieren. Nie zwei Tiers
gleichzeitig überspringen.

Tier-Hierarchie:
1. Single-Cell / Single-Unit Test (isoliertes Phänomen, deterministisch)
2. Small Geometry (Cube, primitiv, single-body, schnelle Runtime)
3. Medium Geometry (Ahmed Body, vehicle-class, BL-resolved, mittlere Runtime)
4. Full Production Geometry (MR2/Yaris, voll-Mesh, lange Runtime)

Beispiel-Application zu Wall Model:
- Single-Cell-Krüger-Test (Finding 33) ✓ Tier 1
- Cube CD-Validation ✓ Tier 2
- Ahmed 25° Drag-Validation — Tier 3, NIE getestet
- MR2 voll-Run — Tier 4

Fehler historisch: Sprung von Tier 2 (Cube) direkt zu Tier 4 (MR2) bei
fundamentaler Code-Änderung. Ahmed als Tier 3 hätte Skalierungseffekt
isoliert.

### Regel 2: Smoke-Test vor voll-Run

Bei jedem neuen Setup oder neuer Code-Änderung: 10-Step Smoke-Test bevor
voller Run. Sanity-Check für offensichtliche Failures (NaN, Divergenz,
unphysikalische Magnitude).

Implementation: `LBM lbm(...); lbm.run(10); print Fx;` vor dem regulären
multi-tausend-Step Run.

Kosten: 10 Sekunden bei 337M cells. Verhindert: 8 Minuten voll-Run mit
sofort-disqualifiziertem Resultat.

### Regel 3: Root-Cause vor Pivot

Bei Negativ-Resultaten oder unphysikalischen Werten (>10× erwartete
Magnitude, NaN, Vorzeichen-Flip, etc.): Diagnose-Output zwingend BEVOR
strategischer Methoden-Wechsel.

Verboten:
- "Iron-Rule-Trigger → Phase C jetzt" ohne Root-Cause-Verständnis
- Hypothese-Generierung als Ersatz für Verifikation
- Annahme dass nächste Methode das Problem nicht auch hat

Erforderlich:
1. Code-Edit-Audit (diff gegen letztes funktionierendes State)
2. Smaller-Scale Reproduktion (Skalen-Ladder rückwärts)
3. Diagnostic-Output (Visualisierung, Field-Export, Component-Decomposition)

Falls nach Root-Cause-Diagnose das Problem fundamental ist → DANN Methoden-
Pivot. Vorher nicht.

### Begründung dieser Regeln

Diese Regeln spiegeln Standard-Engineering-Workflow:
- Aerospace: Wind tunnel test → flight test envelope expansion
- Software: Unit test → Integration test → E2E test → Production
- CFD: Validation case → Verification case → Production case

CC ist als autonomes Engineering-Tool nur dann wertvoll, wenn diese
Methodik eingehalten wird. Schnelle Iteration ohne Skalen-Ladder
produziert nur scheinbar Fortschritt — tatsächlich akkumulierte
nicht-diagnostizierte Bugs.

User-Direktive: "Wenn ich einen Fehler habe, möchte ich verstehen woher
er kommt, nicht blind akzeptieren und weiter."
