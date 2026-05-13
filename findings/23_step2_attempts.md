# CC#11 Option 2 Step 2 — Three Attempts, All Failed

**Datum:** 2026-05-13
**Status:** ❌ Alle 3 simple Varianten fehlgeschlagen am Cube-Test
**Folge:** Full OpenLB-Style mit Pi-Tensor f_neq Reconstruction notwendig (Step 2d, separater Sprint)

---

## Architektur (gleich für alle 3 Versuche)

1. WW Kernel (`apply_wall_model_vehicle`) schreibt u_slip in `u[wall_cell]` (mit TYPE_X-Exclusion in `apply_moving_boundaries` — kein Krüger-Artefakt)
2. Neuer Kernel `apply_wall_slip_to_fluid` läuft POST-stream-collide an TYPE_MS Fluid-Cells
3. Liest u_slip vom benachbarten Wand-Cell (TYPE_S|TYPE_X)
4. Modifiziert OWN DDFs

VRAM-Cost: 0 (u_slip carried via existing u[] array, no new buffer)

---

## Attempt 2a — Krüger-Style Additive Correction, Sign +6

**Formel:** `fhn[i] += +6 × w_i × (c_i · u_slip)` (Standard Krüger Eq. 5.27)

**Cube Result:** CD = 28.7 (target 1.05) — **FAIL**

**Diagnose:**
- Fluid neben Wand bekommt EXTRA Momentum in Slip-Richtung
- Fluid acceleriert nahe Wand → erhöhte dynamische Druck am Wand
- Wand sieht MEHR Drag, nicht weniger

---

## Attempt 2b — Krüger-Style Additive Correction, Sign -6 (flipped)

**Formel:** `fhn[i] += -6 × w_i × (c_i · u_slip)` (umgekehrt)

**Cube Result:** CD = -43.9 — **FAIL** (negative Drag, physikalisch unmöglich)

**Diagnose:**
- Fluid neben Wand verliert Momentum in Slip-Richtung
- Fluid effektiv dezeleriert / reversiert
- Wand sieht "ankommendes" Fluid eher in Gegenrichtung → negative Drag

---

## Attempt 2c — Equilibrium Reset (OpenLB-Pattern, simplified ohne f_neq)

**Formel:** `f_i = f_eq(rho_local, u_slip)` für alle 19 Richtungen

**Cube Result:** CD = 128.9 — **FAIL** (extreme Drag-Overshoot)

**Diagnose:**
- Fluid an TYPE_MS Zellen wird in Equilibrium mit u_slip ≈ 0.9 × u_∞ gezwungen
- f_neq (turbulente Wake-Information) gelöscht
- Fluid neben Wand = sehr hohe Velocity (close to u_∞)
- Massiver dynamic-pressure-Beitrag direkt nahe Wand → Drag explodiert

---

## Vergleichstabelle

| Attempt | Methode | Cube CD | vs Target 1.05 | Verdict |
|---|---|---:|---|---|
| Step 1 (WW disabled) | Bounce-back baseline | 1.10 | +5% | ✅ Reference |
| Step 2a Krüger +6 | Krüger-style add at fluid | 28.7 | +2630% | ❌ |
| Step 2b Krüger -6 | Sign-flipped Krüger | -43.9 | NEGATIVE | ❌ |
| Step 2c f_eq reset | Equilibrium with u_slip | 128.9 | +12180% | ❌ |

---

## Architektonische Erkenntnis

Die Krüger-Formel `Δf_i = +6w_i(c_i · u_wall)` ist designed für eine ganz spezifische Anwendung:
- INSIDE `stream_collide`
- An FLUID-Cell adjacent zu MOVING WALL
- Mit Esoteric-Pull-Storage-Wechselwirkung die das modifizierte DDF an die SOLID-Cell-Storage-Slots routet

Wenn die selbe Formel POST-stream-collide an Fluid-Cells angewendet wird:
- Geht die EP-Storage-Magie verloren (DDFs landen in Fluid-Storage)
- Indirekte Flow-Modifikation statt direkter Wand-Force-Modifikation
- Wirkungs-Sign und Magnitude komplett anders als designed

Daher: **Krüger-Formel kann nicht "im Nachhinein" appliziert werden um Drag-Reduction zu erzeugen**.

Die einzige in der Literatur empirisch validierte Methode für WW ohne Force-Artefakt ist:

**OpenLB-Style Regularized BC:**
```
At TYPE_MS fluid cells:
1. Compute u_slip via wall function (we have this)
2. Compute rho_BC via Zou-He extrapolation
3. Compute Pi_neq (non-equilibrium stress tensor) via finite-difference on neighbor u
4. Reconstruct f_neq[i] = -(w_i × Δt / c_s²) × (Q_i : Pi_neq)
5. Set f[i] = f_eq(rho_BC, u_slip) + f_neq[i]
```

Schritt 3-4 sind die nicht-triviale Komplikation. Erfordert 2D finite-difference auf u in Wall-Tangentialebene.

---

## Code-State

- `apply_wall_slip_to_fluid` Kernel bleibt im Source (kernel.cpp ~1525) als Dokumentation
- Kernel-Call in `do_time_step` AUSKOMMENTIERT (lbm.cpp ~930)
- Cube + MR2 verhalten sich wie nach Step 1b: Cube CD=1.10, MR2 Fx=+1820 N (Bounce-Back)
- Iron-Rule formal eingehalten (Iron Rule "WW nicht ändern" ist seit User-OK für Option 2 obsolete, aber wir haben den apply_wall_model_vehicle Kernel selbst nicht angepackt — nur den experimentellen apply_wall_slip_to_fluid)

---

## Empfehlung für nächsten Sprint

**Option 2d (Full OpenLB Regularized BC):**
- 1-2 Wochen Aufwand
- Erfordert:
  - Pi-Tensor computation aus finite-differences von u an Nachbar-Fluid-Cells
  - Q_i tensor lookup (lattice-specific)
  - f_neq reconstruction
- Würde Drag-Reduction wie OpenLB erreichen (validated against channel flow DNS)

**Alternative: akzeptiere Bounce-Back-Baseline ohne WW**
- Cube CD = 1.10 ✓ (perfekt für simple geometries)
- MR2 Fx = +1820 N (~3× over real 565 N — typische BB-Overshoot bei coarse-grid)
- Yaris Fx = positiver erwartet, kein negativer-Drag-Bug mehr
- Phase 1 (Bouzidi) als nächste Stufe für besseren Wall-Treatment (Roadmap Phase 1 ohnehin geplant)

Beide Pfade dokumentiert, User-Entscheidung notwendig.
