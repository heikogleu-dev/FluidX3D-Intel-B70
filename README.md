# FluidX3D-v2 — Vehicle Aerodynamics on a Single Intel GPU

**A lattice-Boltzmann wall-modelled LES that resolves the forces on a road vehicle at 4 mm on one
workstation, on Intel hardware — and can prove every number it reports.**

A fork of [FluidX3D](https://github.com/ProjectPhysX/FluidX3D) by Dr. Moritz Lehmann. Upstream is
the fastest LBM solver of its class, running at 96–100 % of peak memory bandwidth. This fork does
not try to improve on that. It adds what a vehicle aerodynamics case needs and upstream does not
have: a wall model, sub-cell boundary geometry, a two-device domain decomposition, and an
instrumentation layer that makes silent errors loud.

---

## At a glance

| | |
|---|---|
| **Case** | Road vehicle, 4 mm near field, Re ≈ 8 × 10⁶, moving ground, rotating wheel contact |
| **Grid** | **654.9 M** fine cells on an Intel Arc Pro B70 (32 GB) + coarse far field @ 16 mm on an Arrow-Lake iGPU |
| **Drag** | **Cd 0.6085 ± 0.0137** vs OpenFOAM 13 **0.599** — within **1.6 %** |
| **Downforce** | **Cz −1.0535 ± 0.0234** vs OF13 **−1.301** — **81 %** of the reference |
| **Hardware** | One workstation. No cluster, no CUDA, no NVIDIA |
| **Memory** | **47 B per cell** on device, against 93 B for upstream FP32 |
| **Proof** | Every mechanism carries an action-path counter with an is = should acceptance. A switch without a firing counter is treated as a hard error |

<sub>Forces from the anchor run `p4_bandpi2_4` (git tag `anker-p4-bandpi2-4`), computed from the
field data in `cd_facetten.csv` over the window t ≥ 0.201 s, n = 300 samples, uncertainty = standard
error over six 50 ms block means. Composition is stated below — the two coefficients are not
interchangeable with the `cd_rest` figures in the run report, which are pressure-only.</sub>

---

## What this is, and what problem it solves

Vehicle aerodynamics at engineering accuracy normally means a RANS or hybrid solver on a cluster,
with a body-fitted mesh and a wall function that assumes the first cell sits in a log layer. That
route is well understood and expensive.

LBM is attractive for the opposite reason: it is a cartesian, memory-bandwidth-bound stencil that
maps almost perfectly onto a GPU. The price is that the wall is a **staircase of voxels**, not a
surface. At 4 mm on a car, a plain bounce-back wall behaves like a hydraulically rough one, the
stair-step normal is not the surface normal, and the boundary layer is unresolved by two orders of
magnitude.

Everything in this fork exists to pay that price honestly:

- **Recover the surface** from the voxel body — sub-cell wall distance per link, a fitted surface
  normal across the staircase, exact thin-feature voxelisation.
- **Model what the grid cannot resolve** — a facet-based wall model that drives the near-wall cell
  toward a law-of-the-wall target, plus a subgrid model that is consistent in the wall cell.
- **Fit a car on one GPU** — two-byte fields, a sparse-write pipeline, and a coarse far field on the
  integrated GPU while the discrete card carries the near field.
- **Never trust a number that has no counter.** This is not a slogan; see *How every number is
  proven* below. It has repeatedly caught mechanisms that were computing in the wrong place while
  every global figure looked right.

---

## Results

Both coefficients are **totals**, because the OpenFOAM 13 reference is a total. The composition is
spelled out so that no figure here can be confused with another:

| Component | Value | What it is |
|---|---|---|
| `cd_druck_rest` | **+0.5614 ± 0.0132** | Pressure drag, wheel-contact band removed |
| `cd_reib` | +0.0470 ± 0.0006 | Friction drag |
| **Cd total** | **0.6085 ± 0.0137** | vs OF13 **0.599** |
| `cz_druck_rest` | **−1.1290 ± 0.0238** | Pressure downforce, band removed |
| `cz_reib` | +0.0755 ± 0.0004 | Friction — it works *against* downforce |
| **Cz total** | **−1.0535 ± 0.0234** | vs OF13 **−1.301** |

**Why the band is split off.** The moving z-band around the wheel contact patch produces roughly
−0.7 of purely artificial downforce from the floor imprint. It is removed from the pressure term and
reported separately rather than quietly absorbed.

**The reference.** A paired OpenFOAM 13 run, 34 M cells, k-ω-SST, on the **same STL**. Validating
against an earlier version of one's own code is banned by project rule: it can only find porting
errors, and it confirms shared mistakes.

**The open gap is downforce.** Drag is essentially closed; Cz sits at 81 % of the reference. That
deficit is the active work item, and it is stated here rather than hidden behind a favourable
selection of runs.

### The 4 mm production run, measured

Anchor run `p4_bandpi2_4` (git tag `anker-p4-bandpi2-4`), 501 ms physical, on one Arc Pro B70 plus
the Arrow-Lake iGPU:

| | |
|---|---|
| Fine cells @ 4 mm (B70) | **654.9 M** |
| Coarse cells @ 16 mm (iGPU) | **289.0 M** |
| Near-field VRAM | **28 698 MB** of 32 655 MB — **43.8 B per cell**, all buffers included |
| Far-field memory | **12 956 MB system RAM**, no VRAM ceiling |
| Wall clock | **75 min** |
| Performance index | **8398** s_wall / s_phys |

The far field living in system RAM is what makes the split worth having: the discrete card's 32 GB
buys resolution where it matters, and the domain that only has to be *present* costs nothing there.

### What more memory would buy

This is **arithmetic, not a measurement**. It uses one measured input — the 43.8 B per cell above —
and two scaling laws: cells go as dx⁻³, and at fixed physical time the step count goes as dx⁻¹, so
total work goes as **dx⁻⁴**.

| Near-field memory | Fine cells | Resolution | Work vs 4 mm |
|---|---|---|---|
| 32 GB — *measured, this rig* | 0.65 G | **4.00 mm** | 1× |
| 96 GB — RTX 6000 Pro class | 2.19 G | **2.67 mm** | ~5× |
| 320 GB — 4 × H100 80 GB | 7.30 G | **1.79 mm** | ~25× |
| 768 GB — 8 × 96 GB node | 17.5 G | **1.34 mm** | ~80× |

**What this table does not say.** It states no wall clock, no speed-up and no throughput for any
hardware not in this machine — those would have to be measured, and transferring them from vendor
specifications is exactly the kind of number this project does not print. What it does say is that
the memory wall, not the algorithm, is what currently sets the resolution: the code already fits a
full vehicle at 4 mm into 32 GB, and the cost of the next halving in dx is a factor of sixteen in
work, on any hardware.

**One honest caveat for the multi-GPU rows.** Upstream FluidX3D carries a multi-GPU domain
decomposition; this fork's own decomposition is a *near/far* split across two devices of different
speed, not an N-way split of the fine domain. Running the fine domain across four cards would use
the upstream path, which this fork has not exercised — so those two rows describe what the memory
allows, not a configuration that has been run here.

### Independent validation rigs

| Rig | What it settles | Result |
|---|---|---|
| **Sphere resolution ladder**, D/dx 11 → 37.5 | Does the wall model beat plain bounce-back on a body whose drag is known from experiment? | Converges monotonically from below to **Cd 0.436** against the 0.45–0.5 subcritical band (Achenbach); the bounce-back baseline sits at **0.717**, 50–60 % over |
| **Plane channel**, N = 20, Lee & Moser | The wall model against a case with a literature answer | c_f coverage **48 % → 80 %** of the reference after taking the model input from the second fluid cell |
| **Tilted channel torus**, 0° / 26.565° / 45° | Isolates the staircase: the same physical wall presented as flat, as a 2:1 stair, as a 45° stair | Showed that a *global* sampling factor cannot fit a staircase — the per-class scatter is the diagnosis |

---

## What this fork adds to upstream FluidX3D

Upstream is a general-purpose LBM solver. None of the following exists there; all of it was added
for this case. Every figure was measured on this rig.

### Wall treatment — the core of the fork

| Addition | Why it is needed | Measured effect |
|---|---|---|
| **Facet wall model (iMEM)** — TLS surface fit across the voxel staircase, Spalding target, 3×3 momentum solve for a slip velocity, with a saturation gate | Bounce-back on a 4 mm grid is a rough wall, and the stair normal is not the surface normal | Vehicle Cd 0.818 → **0.728** at 8 mm; action path proven at 1.3 G events, is = should exact |
| **ELIBB** — link-wise sub-cell boundary from a Surface-Nets remesh; the q > ½ branch is an MLS blend whose stability limit was derived, then measured | Places the wall where it is, instead of on the nearest cell face | **10.9 M cut links on 2.62 M facets** at 4 mm, **zero fallbacks**, stable to ω → 2 |
| **Model input from the second fluid cell** | The first cell is bounce-back-deflated *and* sits in the stair shadow; the fitted factor it replaced was calibrated on one geometry at one resolution | Channel u_τ factor **0.696 → 0.920**, c_f **+66 % at 38 σ** |
| **Mass-conserving momentum exchange** (α = 2 + gate) | The facet model injects momentum; uncorrected it also injects mass, and the leak *grows* with resolution | Sphere Δm 458.7 → **−1 × 10⁻⁶**; the uncorrected leak scales 272 → 13 149 from D/dx 11 to 37.5 |
| **Π-consistent multi-layer subgrid band** | The wall-cell subgrid model needs a consistent estimator outward from the first layer | Friction **−16.9 %** at 4 mm, shape factor H 1.613 |
| **Conserving clamps** (positivity, velocity) | Density and velocity clamps shift forces *systematically*, not as realisation scatter — proven with a 50-sample sign test | Standard since 2026-09-16, with the shift quantified rather than assumed |

### Geometry

| Addition | Why | Effect |
|---|---|---|
| **SAT voxelizer** — ray-parity bulk plus every cell any triangle intersects (exact triangle–box overlap), then interior void sealing | Ray parity drops any feature whose entry and exit land in the same cell — wing end-plates, splitter, louvres simply vanish | At 4 mm resolves wheel spokes, brake ducts, diffuser strakes, underfloor channels, wing + Gurney, splitter, canards, louvres, mirror |
| **The voxel body is the only wall truth** | Voxelisation thickens; an STL-derived normal and a voxel-derived normal disagree, and the wall model then silently mixes two geometries | Project rule: wall distance, normal and link occupancy all come from the same body |

### Fitting a car on one workstation

| Addition | Effect |
|---|---|
| **Two-device domain decomposition** — fine near field on the B70, coarse far field on the iGPU, with a smoothed coupling | The far field costs system RAM instead of VRAM |
| **Two-byte fields** for density and velocity | **47 B per cell** against 93 B for upstream FP32 |
| **Sparse field writes**, register-level scheduling, anisotropic block tiling | Wall clock for the 4 mm production run roughly halved over the project |
| **Real VRAM accounting from `/proc/*/fdinfo`** | `intel_gpu_top` cannot see the B70 — the `xe` driver has no i915 PMU. Root-free per-device utilisation instead of a reconstruction |

### Intel platform robustness

Upstream assumes a well-behaved driver. On `xe` and Arrow-Lake it is not always one: a teardown
segfault after the last export, a GEM-BO leak of 12–16 GB per killed run on the iGPU, zero-copy
buffers that spin above ~1 GB, and a discrete card whose host mirrors are freed where the integrated
one keeps them. Each of these is worked around, documented with its detection method, and written so
it can be retested on a future driver.

---

## How every number is proven

This is the part that generalises beyond this car.

**Every mechanism carries an action-path counter** with a declared target. A switch whose counter
does not fire is a hard error, not a curiosity — in the predecessor fork a central fix had been a
silent no-op for years. Counters are checked as `is = should` against a number derived
independently, not against themselves.

**Diagnostics live in the code**, not in a notebook. Each new mechanism gets intermediate-result
introspection so that a small test case shows whether the *steps* are plausible, not only the
final force.

**Three independent audit passes** run after every build section — one over each function, one over
host and pipeline interplay, one over the interaction with other mechanisms and dead code. Findings
are fixed and re-checked until clean. A representative catch: a subgrid band whose action-path
counter matched its target to the last digit was dereferencing bounding-box indices as global grid
indices — correct on the channel, where the two spaces coincide, wrong on the vehicle. The counter
was right and the mechanism was computing in the wrong place.

**One variable per run**, criteria written down *before* the run. Mixed arms invalidate results
retroactively, and you find out only when you go looking for a cause.

**Rejections are kept visible.** A mechanism that was built, measured and found not to work is a
result. Keeping it on record is what stops it being proposed again.

**Every run carries its own source.** A full code copy and commit hash land in
`export/<run>/code/`, so "which code produced this number" is answerable six weeks later without
archaeology. GPU runs go through a locked queue with a status file and a process census.

---

## Build and run

```bash
g++ src/*.cpp -o bin/FluidX3D -std=c++17 -pthread -O -Wno-comment \
    -I./src/OpenCL/include -L./src/OpenCL/lib -lOpenCL
```

Cases and mechanisms are selected by `CFD_*` environment variables; the production configuration is
generated from a machine-written baseline file (`basis/*.basis`) rather than assembled by hand —
reconstructing one by hand once cost a full morning of measurements.

---

## Status

Drag is closed against the reference; downforce sits at 81 % and is the active work. The current
line of work is a **wall-cell reconstruction** that imposes the wall-model target on the cells where
the tangential solve is rank-deficient — roughly a fifth of all wall facets, because a cell with a
single wall link cannot span two tangential directions. It is built, force-booked and measured; at
8 mm it moves the pressure path in the right direction, which wall shear stress alone does not.
Calibration is in progress.

Development history, including the measurements behind every claim above and the arms that were
rejected, is in [HISTORY.md](HISTORY.md).

---

## LBM solver landscape — why FluidX3D on this hardware

Of the major open-source LBM solvers, three run GPU-accelerated on the B70: **FluidX3D** (OpenCL,
native, highest bandwidth utilisation in the field), **OpenLB-SYCL** (experimental, not yet
production-grade on Intel) and **Sailfish** (OpenCL, abandoned upstream). waLBerla, TCLB, Palabos,
lbmpy and Musubi all require CUDA or HIP. FluidX3D's missing pieces — a wall model, sub-cell
boundary geometry, a specular symmetry plane — are exactly what this fork adds.

## Companion repositories

- [ParaView / OSPRay ray-tracing on the B70](https://github.com/heikogleu-dev/Paraview---Intel-B70-Pro-OSPRAY-Raytracing)
- [OpenFOAM v2512 + PETSc-Kokkos-SYCL](https://github.com/heikogleu-dev/Openfoam-v2512-Petsc-Kokkos-Sycl-Intel-B70)
- [OpenFOAM 13 GPU offloading (Ginkgo SYCL)](https://github.com/heikogleu-dev/Openfoam13---GPU-Offloading-Intel-B70-Pro)

## License and attribution

Original FluidX3D © 2022–2026 Dr. Moritz Lehmann. The license is **unchanged** from upstream — see
[LICENSE.md](LICENSE.md): non-commercial, no military or defence use, no AI training on the source,
altered versions must be marked and their source published, and the FluidX3D references must be
cited in publications. This is an altered version; the alterations are described above and in the
commit history. Origin is not misrepresented and the license notice is preserved.
