# PERF-F V5: Multi-Stream PCIe Pipelining — Design Doc

**Datum:** 2026-05-15
**Status:** Design only, NICHT implementiert.
**Geschätzter Aufwand:** ~1 Tag.
**Erwarteter Gain:** 2–3% MLUPs zusätzlich zu PERF-D (97% → ~99% sustained GPU).

## Ist-Zustand nach PERF-D

PERF-D batched PCIe sync ([src/setup.cpp ~1040](src/setup.cpp#L1040), [lbm.cpp:1161](src/lbm.cpp#L1161)):

```
chunk c:
  1. lbm_far.run(100)              [GPU compute, ~50 ms]
  2. lbm_far.u.read_from_device()  [PCIe read, ~30 ms]  ◀ GPU idle
  3. lbm_far.rho.read_from_device()[PCIe read, ~7 ms]   ◀ GPU idle
  4. couple_fields × 5 planes      [CPU bilinear, ~5 ms]
  5. lbm_near.u.write_to_device()  [PCIe write, ~75 ms] ◀ GPU idle
  6. lbm_near.rho.write_to_device()[PCIe write, ~18 ms] ◀ GPU idle
  7. lbm_near.run(300)             [GPU compute, ~150 ms]
  8. (Mode 2 only) Near→Far back   [analog, ~50 ms total]
```

Per chunk: GPU compute ~200 ms, PCIe idle ~80–130 ms → **~38% des Wallclocks ist PCIe-idle**.

PERF-D hat das von cyclic-sync (5+5 transfers/chunk, ~600 ms PCIe) auf batched (2+2 transfers/chunk, ~130 ms PCIe) reduziert.

## V5 Konzept: Compute/Transfer-Overlap via 2. Queue

OpenCL-Standard: einer in-order Queue serialisiert Transfers + Kernels. Aber moderne GPUs (BMG-G31 inkl.) haben **separate DMA-Engines**, die mit Compute parallel laufen können — wenn man sie auf einer **separaten Queue** anstößt.

### Architektur-Idee

Pro `Device`: zweite Queue `cl_transfer_queue` (out-of-order optional). Memory-Transfer-Methoden bekommen optionalen `use_transfer_queue=true` Parameter.

```cpp
// opencl.hpp Device:
cl::CommandQueue cl_queue;          // existing: compute queue
cl::CommandQueue cl_transfer_queue; // NEW: PCIe transfers

// opencl.hpp Memory<T>::read_from_device(use_transfer_queue=false, blocking, ...):
auto& q = use_transfer_queue ? device->cl_transfer_queue : device->cl_queue;
q.enqueueReadBuffer(device_buffer, blocking, ...);
```

### Pipelined Run-Loop

```
chunk c (Mode 1, Pfad A 3:1):
  ├─ compute queue: lbm_far.run(100)              [50 ms]
  ├─ transfer queue (parallel): nothing yet (no data from chunk c-1 needed)
  │
  ├─ compute queue.finish()  ── barrier
  ├─ transfer queue: lbm_far.u.read (event=ev_r)  [30 ms async]
  ├─ transfer queue: lbm_far.rho.read (event=ev_r2)  [7 ms async]
  │   While reads happen on transfer queue:
  ├─ compute queue: lbm_near.run(300) starts        [150 ms]
  │   (uses STALE Near data from previous chunk's coupling — first 100 of 300 steps)
  ├─ transfer queue.finish_or_wait(ev_r2)
  ├─ CPU: bilinear resample × 5 planes              [5 ms, overlapped with Near compute]
  ├─ transfer queue: lbm_near.u.write (event=ev_w)  [75 ms async, OVERLAPS Near compute middle]
  ├─ transfer queue: lbm_near.rho.write             [18 ms async]
  ├─ compute queue.finish() — wait for Near to finish 300 steps
  ├─ transfer queue.finish() — wait for writes to land
```

**KRITISCH:** Near's erste ~100 Steps laufen mit **alten Coupling-Werten** (1 chunk Latency). Bei α=0.10 stabil getestet — sollte hier funktionieren.

Bei Mode 2 (back-coupling Near→Far) analog spiegelbildlich.

### Erwartete Zeitersparnis pro Chunk

| Phase | Aktuell (PERF-D) | V5 (overlapped) |
|---|---:|---:|
| Far compute | 50 ms | 50 ms |
| Far read u/rho | 37 ms (idle) | 0 ms (overlap mit Near compute) |
| CPU bilinear | 5 ms | 0 ms (overlap) |
| Near write u/rho | 93 ms (idle) | ~60 ms (partieller overlap mit Near compute) |
| Near compute | 150 ms | 150 ms |
| **Total per chunk** | **335 ms** | **~260 ms** |
| **Speedup** | — | **~22%** |

Aber Wallclock-Speedup ist nicht 22% — denn der GPU-bound critical path bleibt Far+Near compute = 200 ms. Was wir sparen ist nur **GPU-idle time** während PCIe. Tatsächlicher Net-Gain = `idle_saved / total_wallclock` = ~75ms / 335ms = **22%** wenn 100% overlap.

In Praxis: ~50% overlap erreicht → **~10–15% Wallclock-Gain**. Konservative Schätzung **2–3%** wenn Hardware-DMA-Engine schwächelt.

## Implementations-Aufwand

### Phase A: opencl.hpp Erweiterung (~3 h)
1. Device: cl_transfer_queue als 2. Queue (gleicher context, gleicher device).
2. Memory<T>::read_from_device / write_to_device: Parameter `use_transfer_queue=false`.
3. Event-Returning: bereits da (`Event*`-Parameter existieren in API).

### Phase B: setup.cpp Pipelining (~3 h)
1. Run-Loop neu strukturieren: explizit Far→Near gefolgt von asynchronem Transfer.
2. CouplingOptions erweitern: `use_async_transfer = true`.
3. Event-Chaining zwischen Far compute → Far read → Resample → Near write → Near compute.

### Phase C: Testing (~2 h)
1. Sanity: chunk-by-chunk Force-Vergleich mit PERF-D baseline (Drift < 0.1%).
2. MLUPs-Vergleich über 100 chunks.
3. GPU-Auslastung via `intel_gpu_top` cross-check.

**Risiken:**
- Intel BMG-G31 DMA-Engine: nicht dokumentiert wie viele parallele Channels.
- α=0.5 vs α=0.10: höherer α = stärkere Bindung an aktuelle Daten → 1-Chunk-Latency-Drift problematischer.
- Memory-Coherence zwischen 2 Queues: braucht expliziten Barrier (event-waitlist).

## Alternative: PERF-F V4 (GPU-Resident Coupling)

V4 bypasses PCIe **vollständig** durch custom OpenCL-Kernel der die 5 coupling planes komplett auf der GPU resamplet + schreibt. Aber:
- Aufwand 2–3 Tage
- Erfordert separate Mapping-Buffer für Near in Far-Adressraum (komplex)
- Voller 5% Gain möglich, aber nicht garantiert größer als V5 in Praxis

**Empfehlung:** Erst V5 implementieren + messen, dann entscheiden ob V4 nötig.

## Trigger-Condition

V5 sollte erst implementiert werden, wenn:
1. ✅ PERF-D batched validiert (DONE: 95% sustained)
2. **🔲 Coupling produziert plausible Felder** (in Arbeit 2026-05-15 mit Mode 1 + α=0.5 Test)
3. 🔲 Time-Attack Targets (600 N drag, 1200 N downforce) konvergent
4. → Erst dann 1d Aufwand investieren

## See Also

- [[PHASE_5B_DR_RESULT_2026-05-14]] — PERF-D batched sync validation
- [[PHASE_5B_DR_ALPHA_SWEEP_2026-05-15]] — α=0.10 symmetric Mode 2 results
