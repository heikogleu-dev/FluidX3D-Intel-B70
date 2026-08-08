# Hygiene-Befunde — Arbeitsgrundlage für die Codehygiene

Stand 2026-08-09, kurz nach Mitternacht. Prüfer 1 (toter/unverdrahteter Code) ist fertig;
Prüfer 2 (Effektlosigkeit auf echten Pfaden) lief beim Sitzungsende noch — sein Bericht wird
hier ergänzt, sobald er vorliegt.

**Kurzfazit von Prüfer 1:** Kein einziger vollständig toter eigener Codepfad — die verworfenen
Experimente sind sauber als Negativergebnisse geparkt. Der Aufräumwert liegt in zwei
Kommentar/Code-Widersprüchen, zwei Schalter-Inkonsistenzen, zwei echten Toten, zwei latenten
Fallen und fünffach duplizierter Setup-Hilfslogik.

---

## Prüfer 1: toter/unverdrahteter Code — nach Aufräumwert sortiert

### Behebenswert (Kommentar/Code-Widersprüche — Befundklasse erster Klasse)

1. **`defines.hpp:28-36` — REG-Kommentar widerspricht dem Messurteil.** Der Block verkauft den
   regularisierten Rand als *die Lösung* und nennt das Auskommentieren den Kontrollarm. Beides
   überholt: der Kontrollarm ist der Laufzeitschalter `CFD_REG_BC` (ungesetzt = bit-identisch),
   und das Messurteil („verstärkt das Klingeln, Default AUS") steht nur in `lbm.cpp:583-591`.
   → Kommentar nachziehen: Negativergebnis + Verweis auf den Laufzeitschalter.
2. **SPONGE-Kommentar sagt „an den TYPE_E-Flächen", der Code rampt am Domänenrand**
   (`kernel.cpp:1720-1731`). Im Einzel-Fahrzeugfall und Kugelfall sind y±/z+ **mitbewegte
   Wände** — `CFD_SPONGE_N` dort gesetzt verdickt die Wandgrenzschichten. Emission gilt zudem
   für **beide** Domänen des dd-Falls. → Kommentar präzisieren: „sinnvoll nur in Fällen, deren
   Seitenflächen TYPE_E sind: fernfeld, fahrzeug_dd". (Zone selbst: hängt am Messergebnis.)

### Schalter-Inkonsistenzen

3. **`CFD_REG_BC` und `CFD_PO_HART` prüfen nur das Literal `"0"`** (`lbm.cpp:570, 1397`) —
   `=off`, `=false`, `=""` schalten **EIN**. Verletzt die eigene Regel Nr. 4 („alle Schalter
   werten ihren Wert aus"), und das ausgerechnet bei zwei Kontrollarm-Schaltern.
   → `env_on`/`env_f`/`env_u` aus setup.cpp nach utilities.hpp heben und in lbm.cpp benutzen.
4. **`CFD_NU` hat je nach Fall verschiedene Defaults:** fahrzeug 1,48e-5 (`setup.cpp:470`),
   fahrzeug_dd/fernfeld 1,51e-5 (`:716/1409`). Ein A/B fahrzeug gegen fahrzeug_dd ohne
   gesetztes CFD_NU vergleicht still zwei Viskositäten. → auf 1,51e-5 (Referenz) vereinheitlichen.
5. **`CFD_PO_FACES` nur in kugel und fahrzeug verdrahtet**; dd und fernfeld codieren `2u` hart.
   Die `drive_face[]`-Logik des dd-Falls hängt tatsächlich daran → eher dokumentieren
   („bewusst fest x_max") als nachverdrahten.
6. **Host-Meldung bei `po_hart=1`** sagt „HARTE Klemme" unabhängig von sigma (`lbm.cpp:1414`) —
   bei `CFD_PO_SIGMA=0.5` + `CFD_PO_HART=1` ist es eine weiche Zell-Klemme. Meldung präzisieren.

### Echte Tote (löschen)

7. **`sparse_n_active`** (`lbm.hpp:45`) — write-only, die Meldung nutzt das Lokale.
8. **`fx_c`** (`setup.cpp:1184/1335`) — Vektor wird den ganzen 2,5-h-Lauf befüllt und nie
   gelesen (CSV schreibt direkt). Löschen oder in der Endauswertung ausweisen.

### Latente Fallen

9. **`setup.cpp:421`** — Kugelfall-Warnpfad endet mit `return` statt `_exit(0)` und läuft damit
   in genau den Intel-Teardown-Crash, den der Kommentar 25 Zeilen darüber dokumentiert.
10. **`setup.cpp:1517`** — fehlendes `else` in der Fallauswahl: kehrte fernfeld je normal
    zurück, liefe zusätzlich der Kugelfall.

### Zusammenlegen (5× duplizierte Setup-Hilfslogik)

11. Sparse-Env-Block 3×, F-Box-Berechnung 3×, Kontaktflächen-Übergabe 2× (Begründung existiert
    nur einmal!), `n_cells`-Lambda 2×, Statistik-Schluss 3×. Die Randbedingungsschleifen von dd
    und fernfeld sind zeilengleich; kugel/fahrzeug weichen physikalisch ab und bleiben getrennt.

### Präzisierungen zur Erreichbarkeit (kein Handlungsbedarf, dokumentieren)

- **`apply_velocity_inlet`**: Kernel wird in jedem Programm kompiliert, Host-Objekt nur im
  fernfeld-Fall bei `CFD_FERN_VI=1`; `enqueue` in jedem Zeitschritt aller Fälle, aber durch
  `vi_N_active==0` garantierter No-op. Negativergebnis-Hinweis fehlt in `lbm.hpp:51-52`.
- **REG**: ohne `CFD_REG_BC` werden `reg_fneq`/`deriv_reg` vom OpenCL-Übersetzer gar nicht
  kompiliert — Kontrollarm-Behauptung bestätigt.
- **`po_sigma`**: nicht tot — wirkt in beiden Armen (Ankerrate des Flächenmittels bzw. der Klemme).
- **`kernel_update_fields`**: unter UPDATE_FIELDS gebaut, gebunden, rebindet — aber nie
  enqueue-bar; `LBM::update_fields()` ist ein stiller No-op. Behalten (Upstream), dokumentieren.
- **`index_f_impl`**: in der validierten Sparse-Konfiguration nur von kompilierten, nie
  gestarteten Pfaden referenziert (Signaturtreue). Kommentar spart die nächste Suche.
- **`object_torque`**: gebaut, nie enqueued; der F-BBox-Fix dort ist der Wert. Behalten.
- Kleinkram: `sat_shell_and_void_fill`-Kommentar sagt „beide Fälle", sind vier; `A_eff`-Scope;
  `po_rho/po_sigma/po_hart` könnten Locals sein (suggerieren Laufzeit-Verstellbarkeit).

### Upstream-Erbe ohne Aufrufer (nur Inventar)

`unvoxelize_mesh_on_device`, vier `voxelize_stl`-Overloads, `write_status`, `LBM::reset()`,
Geometriehelfer in shapes.cpp (Setups voxelisieren nur noch STL), Graphics-Stack (aus),
SURFACE/TEMPERATURE/PARTICLES-Pfade (aus).

### Ausdrücklich nicht beanstandet

`run_async`/`finish`, alle drei po-Kernel (Reihenfolge korrekt), Kopplungs-API (Parameterzahl
Host↔Device deckungsgleich), F-BBox-Kette vollständig, `ZEROCOPY_THRESHOLD_MB` korrekt und
ehrlich dokumentiert, alle CFD_KUGEL_*/CFD_FERN_*/CFD_DEV_*-Schalter verdrahtet.

---

## Prüfer 2: Effektlosigkeit auf echten Pfaden

*(ausstehend — Bericht wird ergänzt, sobald er vorliegt)*
