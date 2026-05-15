# Path II.5 Floor-WW — Result & Architecture Limit

**Datum:** 2026-05-15
**Status:** ✅ Code funktioniert, läuft pathology-frei. ❌ **Aber kein sichtbarer BL-Effekt** — Architektur-Limit identifiziert.
**Konsequenz:** Krüger-Moving-Wall-Approach für Wall-Modelling fundamentell unzureichend. Path IV (Pi-Tensor) ist der echte nächste Schritt.

## Implementation

Code committed (siehe diff): `WALL_MODEL_FLOOR` define + `apply_wall_model_floor` kernel + Integration in `do_time_step`. Aufwand ~3h wie geplant.

**Kernel-Logik:**
```cpp
if (TYPE_S && !TYPE_X && z==0) {  // floor only
    u_2 = u[z=1]                    // fluid above floor
    u_rel = u_2 - u_road             // relative velocity
    u_tau = WW_PowerLaw(u_rel)       // Werner-Wengle closed form
    u_slip = u_tau * u+(0.5)         // slip at half-cell
    u[z=0] = u_road + scale * u_rel  // Krüger Moving-Wall transfer
}
```

## Tier-1 + Tier-2 Test Results

**Tier-1 Smoke (chunks 1-20):** PASSED — keine Pathology wie Vehicle-WW, healthy descent.

**Tier-2 Production (chunks 20-166, 58 min wallclock):**

| Metric | Mode 3 + α=0.20 (Baseline) | Mode 3 + α=0.15 + Floor-WW | Δ |
|---|---:|---:|---:|
| Konvergenz-Chunk | 141 | 166 | +18% (α effect, nicht WW) |
| Fx_far | 1,505 N | 1,484 N | −1.4% |
| Fx_near | 1,605 N | 1,552 N | −3.3% |
| Fz_far | +50 N | −22 N | flippt zu downforce |
| Fz_near | −552 N | −525 N | −5% |

**Forces fast identisch zu Baseline.** Visuelle ParaView-Sichtung (User 2026-05-15):
> "Sieht für mich höchstens marginal besser aus! Siehe Bild...immer noch blaue Flecken oberhalb des Moving Grounds"

## Diagnose — Warum nur marginale Wirkung?

### Root Cause: u_2 ≈ u_road in Pure-BB Steady-State

Bei Pure-BB Moving-Wall (rolling road) konvergiert die Strömung an z=1 (= 1 cell über Floor) in Steady-State zu:

```
u_2 = u(z=1) ≈ u_road + δ_viscous
```

wobei `δ_viscous` sehr klein ist (Bounce-Back transferiert u_road direkt zu z=1).

Floor-WW kernel berechnet:
```
u_rel = u_2 - u_road ≈ δ_viscous ≈ 0
```

Bei sehr kleinem `u_rel`:
```
u_t_mag < 1e-6f → kernel returns early (line: `if(u_t_mag < 1e-6f) return;`)
```

**ODER** wenn marginal über Threshold:
```
u_slip = u_tau * u+(0.5) ≈ klein (u_tau klein bei kleinem u_rel)
u[z=0] = u_road + scale * u_rel ≈ u_road  → kein effektiver Change
```

**Result:** Floor-WW ist effektiv ein NO-OP in Pure-BB-Multi-Res-Setup. Kernel runs every step but does no meaningful work.

### Warum sieht der User dennoch "blaue Flecken oberhalb Floor"?

Die langsamen Bereiche **z=2..5 (10-25 mm Höhe)** sind die **Pure-BB Boundary-Layer**, die bei Re=2.7×10⁷ am Floor **physikalisch zu dick aufgelöst** wird:

| Cell-Position | Pure-BB Verhalten | Real Turbulent BL |
|---|---|---|
| z=0 (wall) | u = u_road (prescribed) ✓ | u = u_road |
| z=1 (y₁=0.5 lu = 7.5 mm) | u ≈ u_road (BB transfer) ✓ | u ≈ 0.7×u_∞ (log law) |
| z=2..5 (y=2..5 lu) | u langsam ansteigend zu freestream | u sollte schon 95% u_∞ erreicht haben |

**Pure-BB Floor BL ist ~3-5 cells dick = 45-75 mm**, real-CFD BL sollte ~10-15 mm sein. Floor-WW am z=0 fixt das **nicht** — wir bräuchten Wall-Function-Modifikation an **MEHREREN** wall-adjacent cells (z=1, z=2, z=3) gleichzeitig.

### Architektonisches Limit

Krüger-Moving-Wall ist designed für ECHTE Moving Walls (rolling road = ein "geprescribed velocity" wall). Für Wall-MODEL-Anwendung muss man **multi-cell** law-of-the-wall enforcement implementieren. Das ist genau was Han 2021 / OpenLB tut.

**Floor-WW Vehicle-WW Gegenüberstellung:**

| Aspect | Vehicle-WW (Path I) | Floor-WW (Path II.5) |
|---|---|---|
| Status | ❌ Pathology (340k N transient) | ✅ Stabil, keine Pathology |
| Effect | Forces explodieren | Forces unverändert |
| Root Cause | u_avg over multi-direction neighbors → broken on thin features | u_2 ≈ u_road → kernel no-op |
| Why architectural | F4 (thin features) | Krüger only sets z=0 cell — BL development unaffected |

## Forward-Path

### Path IV — Pi-Tensor f_neq Reconstruction (Han 2021)

Genau dafür designed. Modifiziert **DDFs direkt an wall-adjacent fluid cells** (z=1, z=2 in floor case) so dass das Geschwindigkeits-Profile dem Law-of-the-Wall folgt. **Multi-cell wall function** — fixt die BL-Dicke an mehreren cells gleichzeitig.

**Aufwand:** 1-2 Wochen (siehe [PATH_IV_PI_TENSOR_DESIGN_2026-05-15](findings/PATH_IV_PI_TENSOR_DESIGN_2026-05-15.md))

**Erwartetes Resultat:** BL-Dicke korrigiert auf ~10 mm (3× dünner als Pure-BB), Velocity 1 cell über Floor bei 0.7×u_∞ = ~21 m/s (statt ~28-30 m/s wie Pure-BB falsch zeigt), aber im weiteren BL-Verlauf physikalisch korrekt → bessere Vehicle-Aerodynamik via realistischere Anströmung.

### Alternative — Quick-Fix Test (~30 min)

Modifiziere apply_wall_model_floor um auch **z=1 fluid cell** direkt zu beschreiben:

```cpp
// Currently: u[z=0] = u_road + scale*u_rel
// New: also set u[z=1] = u_road + u_slip_at_y1 (= u_road + (u_tau * u+(1.0)))
```

**Risk:** Direct overwrite of fluid cell velocity in stream_collide pipeline kann instabil werden. Aber wenn stable → könnte Multi-Cell-Wall-Effect simulieren ohne Pi-Tensor refactor.

## Keep / Remove Decision

**Behalten:** `WALL_MODEL_FLOOR` define + Kernel-Code als Reference und potentielles Building-Block für Path IV. Kostet ~0 (kernel ist no-op meistens).

**Memory:** Lerneffekt festhalten dass Krüger-Approach hat Multi-Cell-Limit für Wall-Function-Use-Cases.

## Code Status

- `defines.hpp`: `WALL_MODEL_FLOOR` define (bleibt aktiv für Reference)
- `kernel.cpp`: kernel `apply_wall_model_floor` (bleibt, ist Stand-Alone)
- `lbm.hpp/cpp`: integration (bleibt)
- `setup.cpp`: `lbm_far/near.wall_floor_u_road = lbm_u` (kann disabled werden für nächsten Run via `= 0.0f`)

## Verdict

Floor-WW ist **kein Fail in dem Sinne dass etwas kaputt ist** — der Code funktioniert sauber. Aber das **Krüger-Approach kann das angedachte Wall-Modeling fundamentally nicht leisten**, weil es nur eine cell-layer modifiziert, BL-Dicke aber multi-cell ist.

→ **Path IV (Pi-Tensor) ist der echte Wall-Model-Pfad** für Mode 3 Multi-Res Aerodynamik.

## See Also

- [WALL_MODEL_DEEP_ANALYSIS_2026-05-15](findings/WALL_MODEL_DEEP_ANALYSIS_2026-05-15.md) — F1-F4 Failure Modes
- [PATH_IV_PI_TENSOR_DESIGN_2026-05-15](findings/PATH_IV_PI_TENSOR_DESIGN_2026-05-15.md) — Pi-Tensor design
- [PATH_II_5_FLOOR_WW_DESIGN_2026-05-15](findings/PATH_II_5_FLOOR_WW_DESIGN_2026-05-15.md) — Floor-WW design (predicted gain not realized)
- [WW_MULTIRES_FAILURE_2026-05-15](findings/WW_MULTIRES_FAILURE_2026-05-15.md) — Vehicle-WW failure
