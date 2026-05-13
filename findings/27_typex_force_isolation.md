# Finding 27: TYPE_S|TYPE_X Marker Isolation — The Force-Measurement Pattern

**Datum:** 2026-05-13
**Status:** Production pattern, verified in cube test.

---

## Discovered During WW Investigation

While debugging Krueger force artifacts, we discovered that FluidX3D's
`object_force(TYPE_S | TYPE_X)` uses **exact bit-pattern match**, not
"includes any of these flags". This enables a powerful pattern for clean
force measurement on multi-component setups.

Reference: `kernel.cpp:1941` — `flags[n] == flag_marker` (exact match).

---

## The Pattern

```cpp
// What you want measured
lbm.voxelize_mesh_on_device(measured_part, TYPE_S | TYPE_X);

// What you want in the flow but NOT in force measurement
lbm.voxelize_mesh_on_device(distractor_part, TYPE_S);

// Force isolation via exact match
float3 measured_force = lbm.object_force(TYPE_S | TYPE_X);
// distractor_part is TYPE_S (without TYPE_X), excluded from sum
```

---

## Practical Applications

### Rotating Wheels (Roadmap #4)
```cpp
lbm.voxelize_mesh_on_device(body, TYPE_S | TYPE_X);
lbm.voxelize_mesh_on_device(front_wheels, TYPE_S, ..., float3(omega, 0.0f, 0.0f));
lbm.voxelize_mesh_on_device(back_wheels, TYPE_S, ..., float3(omega, 0.0f, 0.0f));
// Wheels affect flow but contribute zero force artifact to body measurement
```

### Rolling Road
```cpp
lbm.voxelize_mesh_on_device(vehicle, TYPE_S | TYPE_X);
// Floor as TYPE_S only with moving wall velocity → flow correctly modeled,
// no artifact in vehicle force measurement
```

### Symmetry Plane (Half-Domain)
```cpp
lbm.voxelize_mesh_on_device(vehicle, TYPE_S | TYPE_X);
// Sym-plane: TYPE_S | TYPE_Y or TYPE_E or other sym-method
// → Sym-plane Krüger artifact (if any) excluded from vehicle force
```

---

## What This Pattern Does NOT Solve

- Flow physics errors (drag still wrong if BB is too coarse)
- Force MAGNITUDE accuracy (only excludes specific cells from sum)
- Wall model effects on actual fluid (separate problem, see [[25_ww_six_failed_attempts]])

---

## Verification

Cube test in Safe-State:
- Cube as TYPE_S|TYPE_X: Cd = 1.10 ✓ (Hoerner reference 1.05, 97% match)
- Cube as TYPE_S only: object_force(TYPE_S|TYPE_X) returns 0 ✓

---

## Reference Implementation

ProjectPhysX Mercedes F1 W14 setup (src/setup.cpp) uses this pattern
implicitly: body without TYPE_X, wheels rotating without TYPE_X. However,
the F1 setup does NOT measure forces (visualization-only) — the pattern's
force-measurement validation is original work in this repository.

Stokes drag validation (ProjectPhysX) uses TYPE_S|TYPE_X explicitly for
sphere drag measurement — canonical reference for the pattern.

See [[26_ep_storage_boundary_pattern]] for the underlying EP-storage
mechanism that necessitates this workaround.
