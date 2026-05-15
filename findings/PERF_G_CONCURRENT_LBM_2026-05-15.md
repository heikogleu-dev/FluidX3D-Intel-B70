# PERF-G: Concurrent LBM (Far ∥ Near auf einer GPU) — Design Notiz

**Datum:** 2026-05-15
**Status:** Idee dokumentiert, NICHT implementiert. Trigger: Mode 1 Coupling-Validation muss erst plausible Felder zeigen.
**Geschätzter Aufwand:** ~1 Tag.
**Erwarteter Gain:** 20–30% MLUPs **+ physikalisch konsistentere bidirektionale Coupling**.

## Doppelter Nutzen

PERF-G ist **nicht nur Performance** — es korrigiert auch eine mathematische Inkonsistenz im aktuellen Mode 2 (sequenzielles Multiplikatives Schwarz mit asymmetrischem Phasenversatz):

| Coupling-Schema | Forward (Far→Near) | Back (Near→Far) | Symmetrie |
|---|---|---|---|
| Aktuell (seriell) | Near sieht Far(t_now)        | Far sieht Near(t_now−1chunk) | **asymmetrisch** |
| PERF-G (parallel) | Beide sehen Partner(t_now−1chunk) | Beide sehen Partner(t_now−1chunk) | **symmetrisch** |

User-Insight 2026-05-15: "Durch den seriellen Lag verwischen wir ja jedes mal etwas das Bild!?" — exakt. Im seriellen Mode 2 hat das Back-Coupling **8.4 cm Phasenversatz** (bei chunk_far=100, dt_far=1.875×10⁻⁵ s, u=45 m/s).

## Konzept

```
chunk c (parallel):
  ┌─ lbm_far.run_async(100)    [50ms compute, far_queue]  ─┐
  ├─ lbm_near.run_async(300)   [150ms compute, near_queue] ┤ → max 150ms wallclock
  └─ PCIe transfer (V5)        [80ms transfer_queue]       ─┘
  ⇣ barrier (sync all 3 queues)
  ① couple Far→Near (uses Far(t[c]))   ─┐ symmetric application
  ② couple Near→Far (uses Near(t[c]))  ─┘ both with 1-chunk lag
  next chunk
```

Pro Chunk: ~170-200 ms statt 335 ms (gegenüber PERF-D batched sync).

## Architektur

Jede `LBM`-Instanz hat schon ihr eigenes `Device`-Objekt mit eigener `cl_queue`. Beide Devices verweisen aufs gleiche physische GPU (B70). Nicht-blockierende Submission über `run_async` (existiert in FluidX3D als `enqueue_*` API) + manueller Barrier.

Kritisch:
- **Memory-Bandwidth-Constraint:** LBM ist auf B70 ~70-80% bandwidth-saturated → gleichzeitige Ausführung gibt **kein 2x speedup**, sondern ~20-30% Overlap-Gain
- **Synchronisations-Punkt am Chunk-Ende:** beide LBMs müssen warten bis langsamere fertig ist (= Near's 150ms)
- **Coupling-Latenz:** beide Domains sehen Partner mit 1 chunk lag — bei α=0.10 (validiert stabil) unproblematisch, bei α=1.0 hard overwrite riskanter (führt zu klassischer Schwarz-Konvergenz-Reduktion)

## Schwarz-Mathematik

**Aktuell (Multiplikatives Schwarz):**
```
u_far^{c+1}  = solve_far(u_near^c     | boundary)   // sieht aktuellen Near
u_near^{c+1} = solve_near(u_far^{c+1} | boundary)   // sieht FRISCHEN Far
```
Asymmetrisch. Konvergiert schneller pro Iteration aber kann oszillieren bei strong-coupling.

**PERF-G (Additives Schwarz):**
```
u_far^{c+1}  = solve_far(u_near^c | boundary)   // sieht alten Near
u_near^{c+1} = solve_near(u_far^c | boundary)   // sieht alten Far
```
Symmetrisch. Konvergiert langsamer pro Iteration aber **physikalisch konsistenter für Zeitintegration** (echte Simultanität). Beide Domains evolvieren wie in der Realität gleichzeitig.

Für **transiente Aerodynamik** (Vortex-Shedding, Wake-Pulse) ist Additive korrekter — beide Domains sollten zur gleichen physikalischen Zeit aufeinander einwirken, nicht "Far führt, Near reagiert".

## Implementations-Aufwand

### Phase A: Non-blocking LBM submission (~2 h)
- `LBM::run_async(steps)`: identisch zu `run()` aber ohne `finish_queue()` am Ende
- Returns Event-Pointer für Barrier-Wait

### Phase B: Concurrent chunk loop (~3 h)
- setup.cpp: Submit beide LBMs ohne dazwischen zu blocken
- PCIe transfer in eigenem queue (V5 Infrastruktur — Phase A ist bereits drin!)
- Barrier nur am Ende des Chunks

### Phase C: Lagged coupling correctness (~2 h)
- Pre-allocate "previous chunk snapshots" — bei chunk c werden die snapshots von c-1 für coupling genutzt
- Sanity: chunk 0 hat keine c-1 snapshots → initialize mit freestream

### Phase D: Validation (~1 h)
- Force-Vergleich gegen seriellen Mode 2 (Drift erwartbar, < 5% für stabilen Lag-Coupling)
- MLUPs-Vergleich
- ParaView slice → checkpoint dass Felder plausibler aussehen (User-Entscheidung)

## Voraussetzung für Implementation

1. ✅ PERF-F V5 Phase A (2. queue infrastructure) — bereits in opencl.hpp drin
2. 🔲 Mode 1 Run produziert plausible Felder (= das Coupling/Floor/Voxelization passt)
3. 🔲 User-Entscheidung: V5 zuerst (sicher, kleiner Gain) oder G zuerst (höherer Gain + Coupling-Konsistenz)

## Empfehlungsprioritäten

Wenn Mode 1 PLAUSIBLE Felder zeigt → **PERF-G priorisieren** (löst gleich zwei Probleme).
Wenn Mode 1 unplausibel → **erst Coupling-Mechanik debuggen** (Plane-Indizierung, Voxelization, Floor-BC), bevor Performance.

## See Also

- [[PERF_F_V5_DESIGN_2026-05-15]] — Multi-Stream PCIe pipelining (Phase A geteilt mit PERF-G)
- [[PHASE_5B_DR_ALPHA_SWEEP_2026-05-15]] — α-stability data für lag-coupling
