# Finding 26: Esoteric-Pull Storage Interactions with Boundary Modifications

**Datum:** 2026-05-13
**Status:** Pattern identified, retroactively explains multiple failure classes.

---

## Pattern Identified

Direct manipulation of `u[]` or DDFs at TYPE_S cells interacts unfavorably
with Esoteric-Pull streaming. Same mechanism manifests across multiple
problem classes:

| Problem | Symptom | Root Mechanism |
|---|---|---|
| Sym-Plane CC#7..CC#9 attempts | 13-18k N (target ~1k N) | u_solid≠0 at sym-plane TYPE_S cells |
| WW Krueger force artifact | Cube CD=80 instead of 1.10 | u_solid=u_slip via apply_moving_boundaries |
| Rotating wheel force pollution | hypothetical (untested) | Same pattern expected |

---

## Correct Pattern (When Required)

Modify DDFs at adjacent **FLUID** cells (post-stream), NOT u at solid cells:

- CC#9-V5 (`apply_freeslip_y` kernel) had this architecture for sym-plane.
  Implementation gave 14k N → architecture was right, specific bug remains.
- OpenLB f_neq reconstruction follows this pattern for WW.

---

## Practical Workaround: TYPE_S|TYPE_X Marker Isolation

If you can't fix the artifact, isolate force measurement to exclude it:

```cpp
// Body: include in force measurement
lbm.voxelize_mesh_on_device(body, TYPE_S | TYPE_X);

// Components causing artifacts: keep in flow, exclude from force
lbm.voxelize_mesh_on_device(wheels, TYPE_S);
lbm.voxelize_mesh_on_device(floor, TYPE_S);

// Force measurement filters on EXACT match
const float3 force = lbm.object_force(TYPE_S | TYPE_X);
// → Only body cells contribute, wheel/floor artifacts excluded
```

See [[27_typex_force_isolation]] for the full pattern documentation.

---

## Retroactively Explained Cases

- **Rolling Road Floor**: Was TYPE_S only → Krüger artifact existed at floor
  but didn't pollute body drag measurement. Happy accident of FluidX3D
  architecture, not intentional design.
- **CC#9-V6 cut-surface strip (20% drag reduction)**: Removing TYPE_X from
  sym-plane-adjacent vehicle cells eliminated their artifact contribution
  from force sum. Same mechanism as our Step-1b TYPE_X-Exclusion.

---

## Implications for Future Implementations

- Rotating wheels (Roadmap #4): use TYPE_S only, NOT TYPE_S|TYPE_X
- New boundary types: prefer modifying fluid-side DDFs over solid-side u
- Force measurement: explicitly filter via TYPE_S|TYPE_X exact match

See [[25_ww_six_failed_attempts]] for the attempt journey that revealed
this pattern.
