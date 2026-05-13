# Finding 32: object_force() Code-Pfad-Audit

**Datum:** 2026-05-13
**Audit-Mode:** read-only Code-Audit, kein Compile, kein Run.
**Ziel:** Den Faktor 2 in `update_force_field` mathematisch nachvollziehen,
und klären wo der Krüger-Term in die Force-Berechnung eingreift.

**Status-Markers in diesem Dokument:**
- **BEWIESEN** = direkt aus Code-Zeilen oder Standard-LBM-Identitäten ableitbar
- **HYPOTHESE** = plausible Erklärung, theoretisch nicht voll geschlossen
- **OFFEN** = explicit verification needed via Test (Phase B)

---

## 1. Force-Aufruf-Kette

### Status: BEWIESEN

Komplette Code-Pfad-Trace aus `lbm.cpp` und `kernel.cpp`. Kein Argument-Wackelraum.

User-Aufruf:
```
lbm.object_force(TYPE_S|TYPE_X)
```

#### lbm.cpp:1009-1014 (LBM::object_force)
```cpp
float3 LBM::object_force(const uchar flag_marker) {
    for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_object_force(flag_marker);
    ...
    object_force += float3(lbm_domain[d]->object_sum.x[0], ...);
}
```
→ ruft pro Domain `enqueue_object_force(flag_marker)`.

#### lbm.cpp:243-249 (LBM_Domain::enqueue_object_force)
```cpp
void LBM_Domain::enqueue_object_force(const uchar flag_marker) {
    enqueue_update_force_field();  // ← Force-Feld auf aktuellsten Stand bringen
    ...
    kernel_object_force.set_parameters(2u, flag_marker).enqueue_run();
}
```

#### lbm.cpp:224-232 (LBM_Domain::enqueue_update_force_field)
```cpp
void LBM_Domain::enqueue_update_force_field() {
    if(t!=t_last_force_field) {
        kernel_update_force_field.set_parameters(2u, t).enqueue_run();
        t_last_force_field = t;
    }
}
```

Beide Kernels werden im selben t-Wert nur einmal aufgerufen — Idempotenz.

---

## 2. update_force_field-Kernel (kernel.cpp:2105-2116)

### Status: BEWIESEN — Code-Body und Formel

```cpp
kernel void update_force_field(const global fpxx* fi, const global uchar* flags,
                                const ulong t, global float* F) {
    const uxx n = get_global_id(0);
    if(n>=(uxx)def_N||is_halo(n)) return;
    if((flags[n]&TYPE_BO)!=TYPE_S) return;   // ← nur an Solid-Boundary-Cells
    uxx j[def_velocity_set];
    neighbors(n, j);
    float fhn[def_velocity_set];
    load_f(n, fhn, fi, j, t);                // ← Streaming Part 2 (Esoteric-Pull)
    float Fb=1.0f, fx=0.0f, fy=0.0f, fz=0.0f;
    calculate_rho_u(fhn, &Fb, &fx, &fy, &fz); // ← Abuse: Σ c_i × f_i
    store3(F, n, 2.0f*Fb*(float3)(fx, fy, fz));
    //                ^^^
    //                "2x because fi are reflected on solid boundary cells (bounced-back)"
}
```

#### Was berechnet calculate_rho_u? (kernel.cpp:1074-1099)
```cpp
*rhon = rho = Σ_i f_i + 1.0;
*uxn = (Σ_i c_i.x × f_i) / rho;   // analog für uy, uz
```

Eingesetzt: `F = 2 × rho × ((Σc·f)/rho) = 2 × Σ c_i × f_i`

→ **Standard LBM-Momentum-Exchange-Formel.**

---

## 3. BB-Physik: Faktor 2 mathematische Herleitung

### Status: BEWIESEN — Standard-Krüger Eq 5.27

Der Kommentar in Zeile 2115 ist die Antwort:
> "2x because fi are reflected on solid boundary cells (bounced-back)"

Bei Bounce-Back am Wand: das DDF f_i kommt von der Fluid-Zelle, wird am
Wand reflektiert, und f_(opp_i) verlässt mit umgekehrtem c-Vektor. Die
Impulsdifferenz pro DDF ist:
```
Δp_i = c_i × f_i - c_(opp_i) × f_(opp_i)
     = c_i × f_i - (-c_i) × f_i              (perfect BB: f_(opp_i) = f_i)
     = 2 × c_i × f_i
```

Daher der Faktor 2. **Das ist korrekte LBM-Physik, nicht zu halbieren.**

---

## 4. Krüger-Term in apply_moving_boundaries (kernel.cpp:1115-1129)

### Status: BEWIESEN — Code-Position und Formel

```cpp
void apply_moving_boundaries(float* fhn, const uxx* j, const global float* u,
                              const global uchar* flags) {
    for(uint i=1u; i<def_velocity_set; i+=2u) {
        const float w6 = -6.0f*w(i);
        //                ^^^ negatives w6 = -2·w_i·ρ_w/c_s² für D3Q19 (Krüger Eq 5.27)
        ji = j[i+1u];
        {
            const uchar fj=flags[ji];
            const bool is_ww=(fj&TYPE_BO)==TYPE_S && (fj&TYPE_X);
            fhn[i] = ((fj&TYPE_BO)==TYPE_S && !is_ww)
                   ? fma(w6, c(i+1u)*u[ji]+...+c(...)*u[2N+ji], fhn[i])
                   : fhn[i];
        }
        // symmetrisch für fhn[i+1u]
    }
}
```

Wird in `stream_collide` für FLUID-Zellen aufgerufen, die einen
TYPE_S-Nachbarn haben. Modifiziert die DDF-Buffer **fhn[]** der Fluid-Zelle:
```
fhn[i] += -6 × w_i × (c_(i+1) · u_w_at_solid_neighbor)
```

w6 = -6·w_i entspricht direkt Krügers `2·ρ_w/c_s² × w_i` mit ρ_w=1 und
c_s² = 1/3 (D3Q19). Faktor 2 in Krüger Eq 5.27 = standard BB-Korrektor.

### Status: HYPOTHESE — Propagation in update_force_field via EP-Storage

`update_force_field` läuft AUF TYPE_S-Zelle und lädt deren DDFs via `load_f` aus
den FLUID-Nachbarn. Bei Esoteric-Pull liegen post-streaming DDFs der Solid-Zelle
in Slots, die mit den Fluid-Nachbarn geteilt werden. Da Krüger die fhn[] der
**Fluid-Zelle** vor dem Streaming modifiziert, propagieren diese modifizierten
DDFs durch EP-Storage in die Solid-Zelle (wenn man die EP-Indices folgt).

→ **Vermutung:** Krüger-Term mit `-6·w_i·c·u_w` landet in den f_i, die
update_force_field liest. **Nicht voll bewiesen** — erfordert Trace durch
EP-`load_f`/`store_f` Semantik, die wir nicht im Detail auditiert haben.

---

## 5. Empirische Verifikation (Cube Test)

### Status: BEWIESEN — Messdaten aus Test-Runs

| Setup | Cube F_x | Notes |
|---|---:|---|
| Pure BB (no WW) | +1.5 N | Baseline (CD=1.10, Hoerner ref 1.05) |
| WW + Krüger (no subtract) | +110 N | Artifact contribution ≈ +108.5 N |
| Option 1 factor 12 subtract | -236 N | **2× over-subtract** |
| Option 1 factor 6 subtract | 0.4 N | **Correct factor!** |

### Status: HYPOTHESE — Faktor-6 statt Faktor-12 Mechanismus

Aus naiver Theorie:
```
Per-Cell-Artifact_Force = 2 × Σ c_i × Δf_i
                        = 2 × Σ [c_i × (-6·w_i·c_(opp_i)·u_w)]
                        = -12 × Σ c_i·w_i·(c_(opp_i)·u_w)
```

Aber empirisch reicht **6** statt **12**. **Hypothese:** Krüger schreibt
nur `fhn[i] += -6·w·c·u`, nicht symmetrisch auch `fhn[opp_i]`. Bei BB
würde `f_(opp_i) = f_i` propagieren, hier wird aber nur **eine Seite**
verschoben. Im update_force_field summieren wir `2·c_i·f_i` — ein erhöhtes
f_i auf einer Seite gibt Δp = 2·c_i·Δf_i Beitrag. Aber f_(opp_i) bleibt
unverändert, so dass effektive Verdopplung ausbleibt.

**Caveat:** diese Argumentation ist nicht vollständig wasserdicht.
Stream_collide-Semantik bei EP ist subtil — die "eine Seite" Modifikation
könnte durch das EP-Streaming-Pattern (load_f / store_f via geteilte
Slots) trotzdem doppelt im Force-Sum landen, oder gar nicht.

---

## 6. Status: OFFEN — Verification needed (Phase B Sub-Task)

Die Argumentationskette in Section 4-5 erklärt qualitativ, **warum**
Halbieren des Krüger-Formel-Faktors kosmetisch ist — aber der präzise
**Faktor-6 empirische Fit** auf Cube (Option 1 Calibration) ist nicht
sauber aus First Principles abgeleitet.

**Hypothese A:** doubled DDF-counting via Esoteric-Pull = Faktor 2
(Krueger-formula factor) × Faktor 3 (geometry-specific cube-face
contribution = 3 normal-facing DDFs per cell) ≈ Faktor 6 empirical.

**Hypothese B:** EP-Streaming "konsumiert" einen Faktor 2 weil Krüger nur
asymmetrisch eine DDF-Richtung modifiziert (siehe Section 5).

Beide brauchen Single-Cell-Isolation-Test (Phase B Sub-Task) **bevor**
irgendeine production WW Implementation entschieden wird:

### Single-Cell Verification Protocol

1. Setup: 1 TYPE_S|TYPE_X cell isolated in fluid box (z.B. 16³ domain,
   1 zentraler Solid-Cell)
2. u_solid = (0.07, 0, 0) (typical lbm_u in our runs)
3. Disable WW kernel write, **manuell** setzen statt computed: vermeidet
   Werner-Wengle-Komplexität
4. Run apply_moving_boundaries + update_force_field für 10 timesteps
5. Read F_x[solid_cell] each step, write to CSV
6. Compute analytical prediction `F_analytical = 2 × Σ c_i × w_i × c·u_w`
   for ONE cell (closed form, hand-calculable for D3Q19)

**Decision-Tree:**
- If `measured_F ≈ 6 × analytical_F_per_DDF` → Hypothesis A confirmed,
  factor 6 calibration is geometry-specific (3 DDF directions normal to
  +X face for cube) — won't transfer to other geometries.
- If `measured_F ≈ 2 × analytical_F_per_DDF` → Hypothesis B confirmed,
  factor 2 from update_force_field, NOT doubled by EP. Halving Krüger
  formula DIRECTLY fixes the artifact.
- If neither matches → both hypotheses wrong, deeper EP-Storage audit
  needed.

**Aufwand:** ca. 2-4 Stunden (existierender Cube-Setup als Basis, plus
custom single-cell setup-Funktion). Ergebnis steuert Phase-B-Method-Choice
direkt.

---

## 7. Antwort auf Heikos Halbierungs-Frage

### Status: BEWIESEN für die Hebel-Position

**Frage:** "Mit der Krüger doppelten Kraft kommt das her? Kann man die
nicht einfach halbieren?"

**Antwort:**
- **Halbierung des Faktor 2 in update_force_field:** NEIN — bricht
  fundamentale BB-Physik (Eq 5.27 Standard-LBM).
- **Halbierung des -6·w_i in apply_moving_boundaries:** möglich aber
  problematisch:
  - Halbiert den Force-Artifact (gewünscht).
  - Halbiert auch die echte WW-Slip-Wirkung auf den Flow (unerwünscht).
  - Symptom-Patching, nicht Root-Cause-Fix.

### Status: HYPOTHESE — Conditional Outcome

**UNLESS Section 6 single-cell test reveals Hypothese B is correct:**
in that case, halving the Krüger -6 to -3 might be exactly the right fix.
The single-cell test is necessary to decide between "halving works
directly" vs "halving is cosmetic compensation".

**Aktuell beste Prediction (basierend auf Hypothese A):**
- MR2 mit halved Krüger: 1820 + (-1215/2) = +1213 N vs target 565N → still 2× off
- Cube mit halved Krüger: 1.4 + 54 = +55 N (CD ≈ 40) vs target 1.4 N → broken

---

## 8. Path-Forward aus Audit

Wenn Section 6 Single-Cell-Test **vor** Phase-B-Method-Choice durchgeführt
wird, hat Phase B drei klare Alternativen mit empirischer Datenbasis:

1. **Hypothese B bestätigt → halved Krüger ausprobieren** als billigste
   Methode (ca. 4 Stunden Implementation).
2. **Hypothese A bestätigt → OpenLB Pi-Tensor** ist der einzige
   geometry-agnostische Weg (1-2 Wochen).
3. **Keine Hypothese matcht → EP-Storage-Streaming-Audit** wird zur
   Pre-Condition für jede WW-Methode (Aufwand offen).

### Status: BEWIESEN — Step-1b Safe-State als Fallback

Aktueller Code-Stand (TYPE_X-Exclusion in `apply_moving_boundaries`):
- Cube CD=1.10 (matches Hoerner reference)
- MR2 Fx=+1820 N (3.2× over target, but no artifact contamination)
- Drag-Baseline ohne WW-Reduction, bekannte Limitation

Diese Baseline ist die "sichere" Fallback-Position, falls keine
WW-Methode aus Phase B funktioniert.

---

## See Also

- [[25_ww_six_failed_attempts]] — Iron-Rule-Trigger und Attempt-Matrix
- [[26_ep_storage_boundary_pattern]] — EP-Storage Interaktion bei Boundaries
- [[29_geometry_re_yplus_matrix]] — y+ pro Geometrie
- [[30_poiseuille_ww_validation_setup]] — Canonical WW-Validation Setup
- [[31_diagnostic_kernel_sketch]] — u_slip/y+/τ_w Export für OpenFOAM-Compare
