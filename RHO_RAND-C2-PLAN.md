# (Planungsagent 15.09.2026 auf HEAD b158116; ergänzt RHO_RAND-PLAN.md §12–§14a. Offene Fragen an Heiko in §7.)

# RHO_RAND C2: baubarer Plan

## 0 Fassung und Grundlage

- Geprüfter Stand: `/home/heiko/CFD/FluidX3D-v2`, Branch master, HEAD `b158116bea08…`, Arbeitsbaum sauber. Alle Datei:Zeile-Angaben unten beziehen sich auf diesen Stand.
- Gelesen: RHO_RAND-PLAN.md vollständig, dazu die sechs Memory-Dateien.
- **Maßgeblich** sind §12–§14a. §3–§11 gelten nur, wo sie nicht ersetzt sind.
- **Zeilenverschiebung gegenüber dem Plan:** Seit 6298941 sind die setup.cpp-Zeilen um etwa +180…+220 gewandert.
  - Beispiele: Verify jetzt 8097 statt 7873, Slices 8587 statt 8366, Lesestelle 6249 statt 6061.
  - Die Tabellen in Plan §3 sind deshalb als Zeilenbelege veraltet. Inhaltlich stimmen sie weitgehend, die Abweichungen stehen in §3.3 unten.

---

## 1 Kernel-Modus der Ausgabe (Frage 1)

### 1.1 Zweiter Kernel für die Produktion, Modus-Parameter nur im Prüfkernel

**Empfehlung: neuer schlanker Kernel `rho_ausgabe_ebene`.** Den C1-Kernel nicht umwidmen, drei Gründe:

1. **`rho_rek_ebene` liest zu viel.** Er liest an jeder Ebenenzelle `out[o+3ul] = load_rho(rho, n)` (kernel.cpp:4409). Das ist unter einem R1-großen Puffer ein Lesezugriff außerhalb des Puffers.
2. **Er überträgt zu viel.** Er schreibt 4 floats plus ein Wort je Zelle (kernel.cpp:4406–4410). Das kostet bei VTK-Schichten das 4,5-fache an PCIe.
3. **Er wendet an TYPE_MS `apply_moving_boundaries` an** (kernel.cpp:4398). Das ist für die Nachkollisionssumme falsch, siehe 1.2.

**Der C1-Kernel bleibt Prüfkernel** und bekommt einen Parameter `modus`:
- `0` = heutiges Verhalten.
- `1` = Nachkollisionsmodus: ohne MS, `out[o+1]` = Σ|f̃_i| für die hergeleitete Schranke in §4.4.

### 1.2 `apply_moving_boundaries`: nein

**Herleitung.**
- `stream_collide(t)` rechnet so: `load_f`, dann MS-Korrektur (kernel.cpp:2958), dann `rhon` = Σ fhn′ (3082), dann Kollision, dann `store_f`.
- Die Kollision erhält die Masse: Σ feq = rhon−1 (DDF-Shift), Σ Fin = 0, und die P-TRT-Geistmoden sind massenfrei.
- Daraus folgt Σ f_post = rhon. Die MS-Korrektur Σcorr ist in dieser Summe schon enthalten.
- `load_f` mit der Parität t−1 liest genau f_post, nur mit getauschten Paaren (kernel.cpp:4548–4553), die Summe bleibt dieselbe.
- Wer MS noch einmal anwendet, zählt Σcorr = −6·Σ_k w_k (c_k·u_k) doppelt (Formel aus kernel.cpp:1268–1271).

**Warum §14a das nicht sehen konnte.**
- Am gleichförmig mitbewegten Boden ist Σcorr exakt null: (1,0,−1) und (−1,0,−1) heben sich auf, (0,±1,−1) und (0,0,−1) liefern 0.
- Laut log:708–731 ist die Bodenreihe z=0 auch unter den TYPE_E-Flächen Wand. Im dd-Fall ist also jede MS-Zelle symmetrisch, und die gemessenen cp 0,0033 unterscheiden die Varianten nicht.
- Asymmetrische MS-Zellen gibt es erst dort, wo ein bewegter Solid-Nachbar auf einen Nicht-Solid-Nachbarn trifft. Kandidat ist die Kugel am Einlass-Wand-Rand x=1 (ungeprüft).

**Gegenbeleg im Code:** `boden_eq` bildet sein lokales rho aus derselben Nachkollisionslesung ohne MS (kernel.cpp:4004–4005).

### 1.3 RHO_CLAMP: ja

- Sonst ist die Summe ungeklemmt. Heute ist das Wort geklemmt (kernel.cpp:1243).
- Rechnung: Σ f_post = ρ_k + (1−w)(ρ_roh−ρ_k).
  - Bei w ≤ 1 bringt die Klemme exakt ρ_k zurück.
  - Bei w nahe 2 (LES) liegt die Summe bei ≈ 2ρ_k−ρ_roh. Das kann wieder im Intervall landen und dann stark abweichen.
- Solche Zellen sind selten: 14 434 460 Klemmtreffer über den ganzen 4-mm-Lauf (log:5487), rund 5,5·10⁻⁷ je Zellschritt.
- Sie werden als deklarierte Ausnahme separat gezählt, Klemmzähler 0/1 **nicht** mitkopiert.

### 1.4 Sonderklassen

| Klasse | Ausgabe | gegen heutigen Slice-Wert |
|---|---|---|
| TYPE_E (auch Auslass-po_cells) | `load_rho(rho, rr_idx(n))` | bitgleich als float, gleicher Pufferinhalt zum gleichen Zeitpunkt |
| TYPE_S | `1.0f` | bitgleich: kein Schreiber an TYPE_S (po_cells sind nur TYPE_E, lbm.cpp:3176), Saat `rho_pack(1)` → 1.0f |
| tote Kachel, n ≥ N | `1.0f` | wie Saat (dd ohne Tiling) |
| Fluid, MS, Facette (ELIBB), Band | Σ f̃_post (+1, geklemmt) | FP16S-Rundung, Schranke §4.4 |

### 1.5 `boden_eq`, `einlass_eq`, `schale_blend`

- **`boden_eq`** (kernel.cpp:3982–4008) läuft nach `stream_collide` mit demselben t (Aufrufreihenfolge lbm.cpp:3025–3052). Es schreibt feq(ρ_lokal = geklemmte Nachkollisionssumme, u_road).
  - Die Summe bleibt ρ_lokal, aber die feq-Werte werden in FP16S neu gerundet. An diesen Zellen kommt eine zweite Rundung hinzu, deshalb ist die Schranke dort größer (§4.4).
  - Gemessen ist das in §14a in der MS-Spalte (z=1).
- **`einlass_eq`** ist im Nahfeld aus: `s_einlass_eq_n = 0u` (setup.cpp:6278).
- **`schale_blend`** läuft nur im Fernfeld: Nahfeld-alpha = 0 (setup.cpp:6278), Kommentar kernel.cpp:4537.

### 1.6 Zeitpunkt

- **Wann gelesen wird:** nach `lbm_f.run(...)`, vor dem nächsten `drive_boundary_from_coarse`.
- **Welcher Zeitschritt:** `t_rek = get_t()−1`, weil do_time_step erst nach `stream_collide` hochzählt (lbm.cpp:3078).
- **Schutz:** Bei `get_t()==0` bricht der Wrapper mit print_error ab.
- **Warum TYPE_E so gleich bleibt:** Die Slice-Stelle setup.cpp:8591 liegt nach dem Lift dieses Außenschritts. R1 trägt damit denselben Stand wie heute.

---

## 2 Kernel: Inventur am HEAD und Skizzen (Frage 2)

### 2.1 Inventur

`rhoxx*`-Signaturen: 14 (kernel.cpp:1543, 2900, 4041, 4143, 4187, 4250, 4333, 4354, 4426, 5285, 5292, 5299, 5305 und APG 1992). Dazu `float*`-rho nur unter SURFACE/GRAPHICS (1277 ff., 3833, 3925, 5805 ff.).

| Stelle | Zugriff | Umbau unter `#ifdef RHO_RAND` |
|---|---|---|
| `rr_idx` neu, hinter `index()` kernel.cpp:826 | R()-Funktion | 2.2 |
| `initialize` 1582 | liest überall einmal | R1 lesen, sonst `1.0f` (bitgleich zur Saat) |
| APG 2123 / Aufruf 2985–2987 | Nachbar-rho | Sperre bleibt, zusätzlich im Konstruktor |
| `stream_collide` TYPE_E 3021 | Lesen | `rr_idx`, Slot 216 |
| `stream_collide` Schreibblock 3183–3219 | Schreiben | Maske x ≥ Nx−2 ohne Bit 0, verschachtelt um RHO_SPARSAM |
| `update_fields` 4124 | toter Pfad, gebunden (lbm.cpp:622) | nur R1 schreiben |
| `po_reduce_mean` 4159 | `load_drho(po_interior)` | `rr_idx` |
| `apply_pressure_outlet` 4243–4244 | liest m, schreibt n | `rr_idx` für beide |
| `apply_velocity_inlet` 4262 | liest m, schreibt n | `rr_idx`; Nahfeld ohne VI, C0-Wächter 1c |
| `extract_plane_macros` 4348 | Ebene | R1-Wert oder NaN-Marker; der Host nimmt rho aus dem Ausgabekernel |
| `rho_rek_ebene` 4388/4409 | TYPE_E und `out[3]` | `rr_idx` bzw. NaN; Host-Sperre in `alloc_rho_rek` |
| `drive_boundary_cubic_lift` 4477 | Schreiben TYPE_E | `rr_idx`, Slot 216 |
| Transfer 5286/5293 | nur D>1 gebunden (lbm.cpp:3828–3829) | Konstruktor-Sperre D>1 |
| SURFACE/GRAPHICS | float* | Konstruktor-Sperre (heute nur Setup 6272) |

### 2.2 `rr_idx` als R()-Funktion

Nach kernel.cpp:826. Kein `#define` (Falle 3), kein `//` vor Code in derselben Zeile.

```cpp
)+"#ifdef RHO_RAND"+R(
)+R(uxx rr_idx(const uxx n) { // Zelle -> Index in R1 (Dicke 2, Plan §4); def_RR_N = nicht in R1 (Papierkorb-Slot)
	const uint3 c = coordinates(n);
	const uxx a = (uxx)def_Nx*(uxx)def_Ny;
	if(c.z<2u) return n;
	if(c.z+2u>=def_Nz) return 2u*a+(n-(uxx)(def_Nz-2u)*a);
	const uxx r0 = 4u*a+(uxx)(c.z-2u)*(uxx)(4u*def_Nx+4u*(def_Ny-4u));
	if(c.y<2u) return r0+(uxx)c.x+(uxx)c.y*(uxx)def_Nx;
	if(c.y+2u>=def_Ny) return r0+2u*(uxx)def_Nx+(uxx)c.x+(uxx)(c.y+2u-def_Ny)*(uxx)def_Nx;
	if(c.x<2u) return r0+4u*(uxx)def_Nx+4u*(uxx)(c.y-2u)+(uxx)c.x;
	if(c.x+2u>=def_Nx) return r0+4u*(uxx)def_Nx+4u*(uxx)(c.y-2u)+(uxx)(c.x+4u-def_Nx);
	return (uxx)def_RR_N;
}
)+"#endif"+R( // RHO_RAND
```

- **Pufferlänge:** `def_RR_N+1`. Der letzte Slot ist der Papierkorb, damit auch eine Verletzung nie außerhalb des Puffers liest.
- **Kosten je Zugriff (Rechnung, ungeprüft):**
  - Rechenaufwand: 4 Divisionen/Modulo durch JIT-Konstanten (IGC macht daraus Multiply-high), bis zu 6 Vergleiche und 3 Additionen/Multiplikationen, zusammen etwa 15 Ganzzahloperationen.
  - Aufrufe je feinem Schritt bei 4 mm: rund 5,3 Mio. Das sind 3,29 Mio TYPE_E, 0,31 Mio Maske, 0,92 Mio po und rund 0,8 Mio Lift je Schritt, also etwa 1 % der Zellen.
  - Rechenanteil unter 0,05 %. Bandbreite unverändert, weil es dieselben 2-B-Zugriffe wie heute sind, nur in einem kompakten Puffer.

### 2.3 `stream_collide`

**TYPE_E-Lesen**, kernel.cpp:3021:

```cpp
)+"#ifdef RHO_RAND"+R(
		{ const uxx rr_ = rr_idx(n);
		  if(rr_>=(uxx)def_RR_N&&rho_clamp_hits[216]<0xF0000000u) atomic_inc(&rho_clamp_hits[216]);
		  rhon = load_rho(rho, rr_); }
)+"#else"+R(
		rhon = load_rho(rho,        n); // apply preset velocity/density
)+"#endif"+R( // RHO_RAND
```

Slot 216 ist ungegatet mit Soll 0. Als Besuchszähler dient Slot 211 im selben Zweig (3051).

**Schreibblock**, 3183–3219. Einargumentige `#ifdef` verschachtelt (Falle 6):

```cpp
		)+"#ifdef RHO_RAND"+R(
		{ const bool rho_schreiben = (uint)(n%(uxx)def_Nx)+2u>=(uint)def_Nx;
		  if(t==(ulong)def_zaehl_takt+2ul&&rho_clamp_hits[rho_schreiben?205u:204u]<0xF0000000u) atomic_inc(&rho_clamp_hits[rho_schreiben?205u:204u]);
		  if(rho_schreiben) store_rho(rho, rr_idx(n), rhon); }
		)+"#else"+R(
		)+"#ifdef RHO_SPARSAM"+R(
		... unverändert 3184-3216 ...
		)+"#else"+R(
		store_rho(rho, n, rhon); // update density field
		)+"#endif"+R( // RHO_SPARSAM
		)+"#endif"+R( // RHO_RAND
```

### 2.4 Übrige Kernel

**initialize**, 1582:

```cpp
)+"#ifdef RHO_RAND"+R(
	{ const uxx rr_ = rr_idx(n); calculate_f_eq(rr_<(uxx)def_RR_N ? load_rho(rho, rr_) : 1.0f, load_u(u, n), load_u(u, def_N+(ulong)n), load_u(u, 2ul*def_N+(ulong)n), feq); }
)+"#else"+R(
	calculate_f_eq(load_rho(rho, n), load_u(u, n), load_u(u, def_N+(ulong)n), load_u(u, 2ul*def_N+(ulong)n), feq);
)+"#endif"+R( // RHO_RAND
```

**apply_pressure_outlet**, 4243–4244. Der mehrzeilige Aufruf bleibt vollständig in einem R()-Segment (Klammerfalle):

```cpp
)+"#ifdef RHO_RAND"+R(
	const uxx rn_ = rr_idx((uxx)n), rm_ = rr_idx((uxx)m);
	store_rho(rho, rn_, po_hart!=0u ? fma(po_sigma, rho_out-load_rho(rho, rm_), load_rho(rho, rm_))
	                                : fma(po_sigma, (rho_out-1.0f)-po_mean[0], load_rho(rho, rm_)));
)+"#else"+R(
	... Original 4243-4244 ...
)+"#endif"+R( // RHO_RAND
```

**Weitere Stellen:**
- po_reduce_mean 4159: `load_drho(rho, rr_idx((uxx)po_interior[gid]))`.
- velocity_inlet 4262: dieselbe Bauform.
- update_fields 4124: `{ const uxx rr_ = rr_idx(n); if(rr_<(uxx)def_RR_N) store_rho(rho, rr_, rhon); }`
- Lift 4477: `rr_idx`. Bei Sentinel `hits[216]` hochzählen und **kein** `return`, damit die u-Schreibvorgänge unverändert bleiben.
- extract_plane_macros 4348: `{ const uxx rr_ = rr_idx(n); out[o+0ul] = rr_<(uxx)def_RR_N ? load_rho(rho, rr_) : as_float(0x7FC00000u); }`
- rho_rek_ebene 4388 und 4409 analog; `modus` einfügen nach `extent_b`, dadurch wandern hits auf Index 14 und tile_slot auf 15.

### 2.5 Neuer Kernel `rho_ausgabe_ebene`

Hinter `rho_rek_ebene`, Signatur im Hausmuster:

```cpp
)+R(kernel void rho_ausgabe_ebene)+"("+R(const global fpxx* fi, const global rhoxx* rho, const global uchar* flags, const ulong t, global float* out,
	const uint plane_axis, const uint origin_x, const uint origin_y, const uint origin_z, const uint extent_a, const uint extent_b, const uint zaehlen, global uint* rho_clamp_hits
)+R( TS_P
)+") {"+R( // rho_ausgabe_ebene()
	const uint gid = get_global_id(0);
	if(gid>=extent_a*extent_b) return;
	if(zaehlen!=0u&&rho_clamp_hits[219]<0xF0000000u) atomic_inc(&rho_clamp_hits[219]);
	const uxx n = plane_cell_index(gid, plane_axis, origin_x, origin_y, origin_z, extent_a, extent_b);
	if(n>=(uxx)def_N) { out[gid] = 1.0f; return; }
)+"#ifdef SPARSE_TILES"+R(
	if(is_dead_tile(n, tile_slot)) { out[gid] = 1.0f; return; }
)+"#endif"+R( // SPARSE_TILES
	const uchar flagsn_bo = flags[n]&TYPE_BO;
	if(flagsn_bo==TYPE_S) { out[gid] = 1.0f; return; }
)+"#ifdef EQUILIBRIUM_BOUNDARIES"+R(
	if(flagsn_bo==TYPE_E) {
)+"#ifdef RHO_RAND"+R(
		const uxx rr_ = rr_idx(n);
		if(rr_>=(uxx)def_RR_N&&rho_clamp_hits[216]<0xF0000000u) atomic_inc(&rho_clamp_hits[216]);
		out[gid] = load_rho(rho, rr_);
)+"#else"+R(
		out[gid] = load_rho(rho, n);
)+"#endif"+R( // RHO_RAND
		if(zaehlen!=0u&&rho_clamp_hits[220]<0xF0000000u) atomic_inc(&rho_clamp_hits[220]);
		return;
	}
)+"#endif"+R( // EQUILIBRIUM_BOUNDARIES
	uxx j[def_velocity_set];
	neighbors(n, j);
	float fhn[def_velocity_set];
	load_f(n, fhn, fi, j, t TS_A);
	float rhon, uxn, uyn, uzn;
	calculate_rho_u(fhn, &rhon, &uxn, &uyn, &uzn);
	out[gid] = rhon;
} // rho_ausgabe_ebene()
```

- **Host übergibt** `t = get_t()−1`. Kein MS, keine Klemmzähler, u wird verworfen (es wäre negiert).
- **Parameterindizes:** fi 0, rho 1, flags 2, t 3, out 4, axis 5…eb 10, zaehlen 11, hits 12, tile_slot 13.
- **Warum `zaehlen`:** Ein ungegatetes `atomic_inc` je Zelle wäre bei VTK 519 Mio atomare Inkremente auf einen Slot je Dump. Das ist ein Konkurrenzrisiko, das ich nicht geprüft habe. Gezählt wird deshalb nur in ausgewählten Aufrufen mit Ist=Soll.
- **Zensus** lbm.cpp:346–347: rho 14 → 15, u bleibt 20 (kein `velxx* u`).

### 2.6 Zählerslots

- **Belegung heute:** Literal 217 und 218 nur in kernel.cpp:4376/4389. Der Puffer hat 224 Slots (lbm.cpp:605).
- **Berechnete Indizes** (Dump über `rho_clamp_hits|hits|diag` mit Ausdrucksindex):
  - Maximum 185 bei `170u+8u*vd_bank+vd_bin` (3331) und 198 bei `ns_b` (3371).
  - Dazu 204–207. 216 und 219–223 sind frei.
- **Neu:**

| Slot | Bedeutung | Soll |
|---|---|---|
| 216 | R1-Sentinel in TYPE_E-Lesen, Lift und Ausgabe, ungegatet | 0; Besuchsbeleg über 211 (>0), 215 (>0) und 220 |
| 219 | Ausgabe-Besuche in gezählten Aufrufen | Summe der Ebenenzellen dieser Aufrufe |
| 220 | davon TYPE_E | Anzahl `(flags&3)==2` in diesen Ebenen, gezählt aus den Geräte-Flags |
| 204/205 | wie heute, ein Schritt | 205 = Zellen mit x ≥ Nx−2 und `(f&3)∉{1,2}`; 204+205 = alle solchen Zellen (Host-Flags, MS zählt als Fluid) |

- Die Legende in lbm.cpp:620 und lbm.hpp:353 bekommt „nächster freier Slot 221".

---

## 3 Host (Frage 3)

### 3.1 Domäne, Emission, Allokation, Rebind

**Allokation** (lbm.cpp:578 ff.). `rho_rand_on` muss **vor** die rho-Allokation, heute steht es in Z. 628 hinter Z. 585:

```cpp
	rho_rand_on = s_rho_rand>0u; // aus Z. 628 hochgezogen
	rr_N = rho_rand_on ? r1_anzahl(Nx, Ny, Nz) : 0ull;
	if(rho_rand_on) {
		if(get_D()>1u) print_error("RHO_RAND: D>1 -- Halo-Transfer liest rho am Domaenenschnitt.");
		if(Nx<5u||Ny<5u||Nz<5u) print_error("RHO_RAND: Kante < 5, die R1-Ringformel ist nicht definiert.");
		if(s_fac_apg!=0.0f) print_error("RHO_RAND x APG (Konstruktor-Sperre, setup-unabhaengig wie lbm.cpp:230).");
#if defined(SURFACE) || defined(GRAPHICS)
		print_error("RHO_RAND x SURFACE/GRAPHICS: deren rho-Leser greifen auf N zu.");
#endif
	}
	rho = Memory<rhoxx>(device, rho_rand_on ? rr_N+1ull : N, 1u, true, true, rho_pack(1.0f)); // +1 Papierkorb
```

**Emission** neben lbm.cpp:2174:
`+((s_rho_rand>0u) ? (string)"\n	#define RHO_RAND"+"\n	#define def_RR_N "+to_string(r1_anzahl(Nx,Ny,Nz))+"ul" : (string)"")`.
Das Fernfeld bleibt ohne, weil setup.cpp:6324 die Statik nullt.

**Hostzwilling** in lbm.hpp neben `rho_pack`:
- `inline ulong r1_anzahl(Nx,Ny,Nz)`
- `inline ulong rr_idx_host(n,Nx,Ny,Nz)`, wortgleich zu 2.2.

**`alloc_rho_ausgabe(max)`** nach dem Muster lbm.cpp:792–803:
- `rho_aus = Memory<float>(device, max, 1u)`
- `kernel_rho_ausgabe_ebene = Kernel(device, max, "rho_ausgabe_ebene", fi, rho, flags, t, rho_aus, 0u,0u,0u,0u,1u,1u, 0u, rho_clamp_hits); if(sparse_on) ...add_parameters(tile_slot);` (Falle 8)
- LBM-Wrapper prüft `initialized` wie lbm.cpp:3334.
- `alloc_rho_rek`: `if(rho_rand_on) print_error(...)`.

**Rebind** in `finalize_sparse_tiles` hinter lbm.cpp:1625 (N7):
`if(rho_rek_max>0ull) kernel_rho_rek_ebene.set_parameters(0u, fi); if(rho_aus_max>0ull) kernel_rho_ausgabe_ebene.set_parameters(0u, fi);`

### 3.2 LBM-Ebene: Fassade, Ausgabe, Slices, VTK

**Rho_Feld** (lbm.hpp:853–865, K5). Die Speicherstelle wird im selben Commit wie die Puffergröße umgebaut:

```cpp
	class Rho_Feld {
	private:
		Memory_Container<rhoxx> c; LBM* lbm = nullptr; bool rand = false;
		std::vector<float> ebene; uint ebene_achse = 3u, ebene_pos = 0u; ulong ebene_t = ~0ull; // EIN float-Cache
		float get_rand(const ulong n); // lbm.cpp
	public:
		ulong n_cache = 0ull, n_r1 = 0ull;
		void binde_rand(LBM* l) { lbm = l; rand = true; }
		void setze_ebene(const uint achse, const uint pos, const ulong t, std::vector<float>& w) { ebene.swap(w); ebene_achse = achse; ebene_pos = pos; ebene_t = t; }
		inline float get(const ulong n) { return rand ? get_rand(n) : rho_unpack(c[n]); }
		inline void set(const ulong n, const float r) { if(rand) print_error("RHO_RAND: Rho_Feld::set ist gesperrt (Host packt keine Nachkollisionssumme, lbm.hpp:40 ff.)."); c[n] = rho_pack(r); }
		void read_from_device();  // rand: lbm_domain[0]->rho (R1-Spiegel, 16,8 MiB)
		void write_to_device();
		const ulong length() const;
	};
```

- **`get_rand`** in lbm.cpp:
  - Ebenen-Cache nur, wenn `ebene_t==lbm->get_t()`. Das verhindert veraltete Werte, z. B. wenn Verify z+ bei y=fNy/2 liest.
  - Sonst R1, aber nur bei `(flags[n]&3)==TYPE_E` oder x ≥ Nx−2.
  - Sonst `print_error` (Host-Zugriffssperre).
- **Einbindung:** Beide LBM-Konstruktoren binden `if(lbm_domain[0]->rho_rand_on) rho.binde_rand(this);` direkt nach lbm.cpp:2635 bzw. 2685.

**`LBM::rho_ausgabe_ebene`** analog zu lbm.cpp:3376–3392:
- `set_parameters(3u, get_t()-1ull)`, `set_parameters(5u, axis,…,eb, zaehlen)`, `enqueue_run`, `finish_queue` (Zero-Copy-Falle), `rho_aus.read_from_device(0, n_plane)`.
- Bei `zaehlen`: Slots 219/220 vor und nach dem Aufruf lesen, Differenz gegen n_plane bzw. TYPE_E-Anzahl ablegen. Die Bewertung erfolgt erst am Funktionsende (§4.1).

**`lese_yslice_in_host`** (lbm.cpp:3399–3423):
- Unter rand erst `extract_plane_macros` (liefert u) und `extract_plane_flags`.
- Dann `rho_ausgabe_ebene(plane, rho_e, erster_Aufruf)`.
- Zeile 3420: `if(!rr) rho.set(n, ebene[o]);`
- Nach der Schleife `rho.setze_ebene(1u, y, get_t(), rho_e)`.

**`LBM::rho_schicht_in_host(z)`:** Achse 2, Ausdehnung Nx×Ny, dann `setze_ebene(2u, z, …)`.

**`schreibe_vtk_feld`** (setup.cpp:1199–1206):

```cpp
		const bool rr_ = L.lbm_domain[0]->rho_rand_on;
		for(uint z=0u; z<Sz; z++) {
			if(rr_) L.rho_schicht_in_host(z*stride); // RHO_RAND: Nachkollisionssumme je Schicht
			for(uint y=0u; y<Sy; y++) { for(uint x=0u; x<Sx; x++) buf[x] = reverse_bytes(L.rho.get(...)); f.write(...); }
		}
```

- `max_cp` muss dafür **nicht** wachsen (§5 alt), der Ausgabepuffer ist eigen.
- Aufruf in setup.cpp:7671: `lbm_f.alloc_rho_ausgabe(max((ulong)fNx*fNz,(ulong)fNx*fNy))`, bei rand oder Prüfarm.

### 3.3 Host-Inventur aller rho-Zugriffe

| Stelle | Klasse unter RHO_RAND |
|---|---|
| lbm.cpp:585 Alloc, 596/621/622/778–781/799/3270/3272/3308 Bindungen | R1-Puffer (Kernel-`#ifdef` §2) |
| lbm.cpp:3828–3829 Transfer, 2386–2399 Grafik | gesperrt (D>1 bzw. GRAPHICS) |
| lbm.cpp:2633–2635, 2683–2685 Container | `binde_rand` |
| lbm.cpp:2938 Upload | R1 (16,8 MiB) |
| lbm.cpp:3420 `rho.set` | Ebenen-Cache |
| lbm.cpp:2749–2805 VRAM-Vorprüfung | anpassen (§3.4) |
| info.cpp:76–82 CPU-Anzeige | anpassen (§3.4) |
| setup.cpp:1122/1129/1131 `pruefe_slice_ebene` | gesperrt (SLICE_PRUEF, 6268) |
| setup.cpp:1203 VTK | Schicht-Cache |
| setup.cpp:1515 Bericht 204/205 | `rho_an = rho_takt>0u \|\| rho_rand_on`, Ist=Soll exakt (§2.6) |
| setup.cpp:4787/4789/4795/4796 C1-Instrument, `d->rho[...]` direkt | im RAND-Arm gesperrt, sonst Lesen außerhalb des Hostpuffers. Sperre an der Lesestelle und in 9171 |
| setup.cpp:6906 bewusster C0-Abbruch | **entfällt**, ersetzt durch Info; 6905 (bad>0) bleibt; Text 6275 anpassen |
| setup.cpp:8098/8154 Kopplungs-Verify | R1-Read und `get` an TYPE_E |
| setup.cpp:8588 SLICE_PRUEF, 8599 Altpfad | gesperrt (6267/6268) |
| setup.cpp:8591/8605 Slice und Sonde | Ebenen-Cache (Sonde liegt auf y=fNy/2) |
| setup.cpp:8641/8654/8774 VTK-Reads | R1-Read (harmlos), rho aus Schichten |
| setup.cpp:8611/8643/8656/8776, Bilanz 8290, Interface 8025 | Fernfeld, unverändert |
| setup.cpp:7495–7503 `rho_takt`-Prüfung, 7974 `rho_voll_zwang` | unter rand wirkungslos (Bit 0 im Kernel ignoriert) |

Nahfeld-Nutzer von `extract_plane_macros` gibt es außer `lese_yslice` keine (grep `lbm_f.extract_plane_macros`: 0 Treffer).

### 3.4 VRAM-Vorprüfung und Anzeige

**lbm.cpp:2804**, hinter dem Facettenblock:

```cpp
	if(LBM_Domain::s_rho_rand>0u&&Dx*Dy*Dz==1u) {
		bytes_bekannt -= N_dom*(ulong)sizeof(rhoxx);
		bytes_bekannt += (r1_anzahl(Nx,Ny,Nz)+1ull)*(ulong)sizeof(rhoxx) + 4ull*max((ulong)Nx*Nz,(ulong)Nx*Ny);
	}
```

Der Nahfeld-Konstruktor (setup.cpp:6288) liegt hinter der Lesestelle 6256, die Statik ist dort also sichtbar.

**info.cpp:79:** `if(d0->rho_rand_on) b -= (ulong)sizeof(rhoxx)*(lbm->get_N()-(d0->rr_N+1ull));`

**`bandwidth_bytes_per_cell_device`** (lbm.cpp:57) zählt weiter 2 B rho. Das betrifft nur die Anzeige (Frage 5).

---

## 4 Wächter, Zähler, Prüfmodus, zwei Zahlen (Frage 4)

### 4.1 Neue Abnahme `berichte_rho_rand(L)`

- Neu in setup.cpp neben `berichte_dichteklemme` (1379). Aufruf am Ende von main_setup_fahrzeug_dd **vor** 9170 und hinter allen übrigen Abnahmen, weil print_error = exit(1).
- **Inhalt:**
  - Slot 216 = 0, mit 211 > 0 und 215 > 0.
  - 219/220 Ist=Soll aus den gezählten Aufrufen (erster Slice, erste VTK-Schicht).
  - 204/205 Ist=Soll exakt.
  - Rho_Feld `n_cache`/`n_r1`.
  - R1-Zeilen je Kennzahl kurz (Konsolenumbruch).
- **Hostselbsttest** in `pruefe_rho_rand_c0` (lbm.cpp:1100–1115, die Schleife läuft ohnehin über alle Zellen):
  - `rr_idx_host` ist über R1 eine Bijektion auf [0, RR_N), geprüft mit einem Bitfeld von 1,1 MB.
  - Innenzellen liefern den Sentinel.
- **Gerät gegen Host:**
  - Der Lift schreibt über den Geräte-`rr_idx`, Verify liest über den Host-`rr_idx`. Die heutige Deckungspunkt-Prüfung (setup.cpp:8121–8159) belegt damit die Übereinstimmung auf allen vier getriebenen Flächen.
  - po-Pfad: nur indirekt über u/Kräfte bitgleich zum A-Arm.

### 4.2 Sperren (Lesestelle setup.cpp:6258 ff.)

- **Neu:**
  - `CFD_RHO_REK_PRUEF×RHO_RAND` und `CFD_RHO_AUSGABE_PRUEF×RHO_RAND` sind harte Fehler: beide Prüfarme brauchen den vollen Puffer.
- **Bestehend:** APG, SLICE_GPU=0, SLICE_PRUEF, UPDATE_FIELDS, SURFACE/GRAPHICS (alle schon da). Neu dazu die Konstruktor-Sperren aus §3.1.

### 4.3 Prüfmodus: Soll aus einem Arm ohne RHO_RAND

**(a) Im selben Lauf, A-Arm (C2a).** Das ist die Erweiterung von `pruefe_rho_rekonstruktion` (setup.cpp:4765).
- Kandidaten:
  - `rho_ausgabe_ebene` (Produktionskernel)
  - `rho_rek_ebene` im Modus 1
  - bisher t0−1 mit MS
- Soll:
  - Produktionskernel = Modus 1 bitgleich.
  - TYPE_E und TYPE_S gegen `w_vor` exakt.
  - Alle anderen Zellen |Δ| ≤ S (§4.4).
  - Klemmzellen (ρ_roh ∉ (0,5; 1,5)) separat gezählt.
- Zusätzliche Ebene z=1 (Boden).
- Host zählt asymmetrische MS-Zellen, Σcorr ≠ 0, aus Flags und Solid-u.
  - Gibt es welche: „mit MS" muss S reißen, „ohne MS" muss S halten.
  - Gibt es keine: als „in diesem Fall nicht trennscharf" ausweisen.
- Kugel-Ebene x=1 als trennscharfer Kandidat (ungeprüft).

**(b) Über zwei Läufe (C2d).**
- A-Arm (`CFD_RHO_RAND=0`, `CFD_RHO_AUSGABE_PRUEF=1`) schreibt `rho_ausgabe` als zusätzliche Sonden-Spalte und als zusätzliches `SCALARS rho_ausgabe` im VTK.
- B-Arm (`CFD_RHO_RAND=1`).
- Soll: `B.rho ≡ A.rho_ausgabe` bitgleich (gleicher fi-Zustand, gleiche Summenreihenfolge), dazu u, Kräfte und cd_facetten bitgleich. Ungeprüft, siehe Falle 12.

### 4.4 Hergeleitete Schranke statt Handwert

- **Speicherfehler:** `vstore_half_rte` rundet je Slot relativ höchstens 2⁻¹¹. Damit ist |Σ f̃ − Σ f| ≤ 2⁻¹¹·Σ|f̃_i|·(1+2⁻¹¹).
- **Heutiges Wort:** Zusätzlich die Wortrundung |ρ−1|·2⁻¹¹.
- **Schranke:** S = 2⁻¹¹·(Σ|f̃_i| + |ρ−1|) + ε_FP.
  - An boden_eq-Zellen, also z ≤ boden_eq_n, wird Σ|f̃_i| doppelt angesetzt, weil feq dort neu gerundet wird.
  - ε_FP = Rundung der Kollisionskette. Konservativ 64·FLT_EPSILON·(1+Σ|f̃|), ungeprüft.
- **Plausibilität:** Mit Σ|f̃| ≈ 0,25 folgt S ≈ 1,2·10⁻⁴ ≙ cp 0,014. Gemessen sind max cp 0,0033 (§14a), die Schranke ist also nicht zu eng.

### 4.5 Negativtests

Jeder Wächter wird bewiesen; Testhaken `CFD_RHO_RAND_TESTHAKEN` bekommt neue Werte:

| Haken | Eingriff | Soll |
|---|---|---|
| 1 (C0) | Innen-TYPE_E im Wächter | (1a) feuert |
| 2 | Ausgabe mit `t` statt `t−1` | Prüfarm reißt S (C1: cp bis 0,41) |
| 3 | Prüfkernel Modus 1 mit MS | an asymmetrischen MS-Zellen reißt S (Kugel) |
| 4 | Sonde liest eine Innenzelle abseits der Ebene | Host-Sperre (print_error) |
| 5 (nur Kugel) | C0-(1a) auf Warnung, eine Innenzelle TYPE_E | Slot 216 > 0 → Fehler am Funktionsende |
| 6 | Cache nach `run(1)` ohne neue Ebene lesen | `ebene_t`-Sperre feuert |

Bestehende Sperren (APG, SLICE_GPU=0, SLICE_PRUEF, REK_PRUEF×RAND) je einmal als Kurzlauf.

### 4.6 Zwei Zahlen (RECHNUNG aus logs/p4_register.log, nicht gemessen)

**VRAM Nahfeld bei 4 mm (1689×661×465 = 519 139 485, log:851):**
- rho heute: 2 B × N = 1 038 278 970 B = **990,2 MiB**.
- RHO_RAND: 2 B × (8 791 740 + 1) = 17 583 482 B = 16,8 MiB. Dazu Ausgabepuffer 4 B × 1 116 429 = 4,3 MiB.
- **Netto −969,2 MiB VRAM (−1016 MB)**, dazu gleich viel System-RAM (Hostspiegel, die B70 hat kein Zero-Copy).
- Soll für den 8-mm-A/B über fdinfo: 845×333×233 → R1 2 200 924 → **−119,8 MiB**.

**B je Zelle und Schritt:**
- **Heute** (p4_register mit RHO_SPARSAM; 305 117 geschrieben und 452 226 190 übersprungen, log:5458–5459; Takt 60, log:814):
  (305 117·2·59 + 452 531 307·2)/60/519 139 485 = **0,0302 B**.
- **RHO_RAND:** 305 117·2/N = **0,0012 B**. Unterschied −0,029 B = **−0,025 %** von 115 B (log:1081: 2271 MLUPs ↔ 261 GB/s).
- **Gegen einen Bezug ohne SPARSAM:** −1,74 B = −1,5 %.
- **Unverändert:** TYPE_E-Lesen 0,0127 B, po 0,0035 B.
- **Ausgabe:**
  - Slice: 785 385·39 B / 3000 Schritte (log:819) / N = 0,00002 B.
  - VTK: 20,2 GB je Dump ≈ 0,34 Schritt-Äquivalente; PCIe 1,98 GB float statt 0,99 GB Wörter.
  - Verify: 2× 16,8 MiB statt 990 MiB.

---

## 5 Testweg (Frage 5)

**Befund:** Alle Nahfeld-Kopfzeilen in `logs/*.log` nennen Gerät 1 (grep „Fein (Geraet"; 365 Zeilen, alle B70). Ein dd-Nahfeld auf CPU oder iGPU ist **nie gelaufen**, 16 mm ebenfalls nie.

**Kleinste dd-Konfiguration (ungeprüft):**
```
CFD_DX=16  CFD_DEV_FINE=0|2  CFD_DEV_COARSE=0|2  CFD_SPONGE_N=0  CFD_N2F_BAND=0  CFD_N2F_SCHALE=0
CFD_T_WARMUP=0  CFD_T_END=0.02  CFD_SAMPLE_EVERY=1  CFD_SLICE_NEAR_STEPS=40  CFD_ZAEHL_TAKT=100  CFD_VTK_ENDE=1
```

- **Größe:** etwa 425×169×117 = 8,4 Mio Nahfeld- und etwa 3,2 Mio Fernfeldzellen, 125 Außenschritte.
- **Sponge muss aus:**
  - Die Basis skaliert `CFD_SPONGE_N` auf 16 Zellen.
  - NF_OX liegt bei 16 mm bei etwa 37 Zellen, die Sperre setup.cpp:6310 verlangt N+32 ≤ NF_OX. Mit 48 > 37 würde sie auslösen.
  - Die Abweichungen gehören in `CFD_BASIS_ABWEICHUNG`.

**Leiter:**
1. **CPU:** Wenn Heiko zustimmt (Frage 1), Kugel `CFD_KUGEL_DX=40` mit RHO_RAND als Prüfstand. Die Kugel hat TYPE_E, po, MS-Wände, Slices und deterministische Hashes. Das JIT übersetzt alle Kernel inklusive Lift.
   - Sonst dd 16 mm 0/0, `nice -n 19`, über die Queue mit `CFD_QUEUE_DEV=0`.
   - Beweist nur JIT und Absturzfreiheit, **nicht** das Instrument.
2. **`werkzeuge/scratch_gate/scratch_gate.sh`** offline. `gen_main.cpp` bekommt ein 6. Argument `rand`:
   - `#define RHO_RAND`, `def_RR_N 13776ul` (62×30×22).
   - U_SPARSAM und SM-Box ohne RHO_SPARSAM/RHO_SMBOX.
   - Zwei neue Arme: `e1p1rsuR`, `e1p1suR`. Soll private=0 und spill=0 für jeden Kernel auf 0x7d67 und 0xe223.
3. **iGPU 2/2:** dd 16 mm, danach 8 mm kurz (T_END 0,01). Negativtests 2, 4, 6 hier.
4. **B70 1/2:** dd 8 mm kurz (Rauchtest), Prüfarm (a) bei t ≈ 300 ms.
5. **A/B 8 mm, B70 1 + iGPU 2, identische Zeile außer `CFD_RHO_RAND`:**
   - **(i) Determinismus zuerst:** A gegen A′ `cmp`. Für dd nach dem po_mean-Umbau nicht belegt (ungeprüft).
   - **(ii) A gegen B, bitgleich per `cmp`:** forces.csv, cd_facetten.csv, u-Spalten der Sonde; B.rho gegen A.rho_ausgabe, auch VTK.
   - **(ii) Innerhalb S:** A.rho_ausgabe gegen A.rho im Prüfarm.
   - **(ii) VRAM:** SPEICHER-ZWISCHENSTAND und fdinfo mit Soll −119,8 MiB.
   - **(ii) MLUPs:** `[LEISTUNG-AB-MARKE]`, Soll im Rauschen.
6. **4 mm nur mit Go.**

---

## 6 Commits (Frage 6)

| Commit | Inhalt | Soll |
|---|---|---|
| **C2a** Ausgabekernel und Prüfarm, noch ohne R1 | kernel.cpp: `rho_ausgabe_ebene` (nicht-RAND-Zweig), `modus` in `rho_rek_ebene`. lbm.cpp/hpp: `alloc_rho_ausgabe`, Wrapper, Rebind, `initialized`-Prüfung, Zensus 15. setup.cpp: Prüfinstrument (a) mit z=1-Ebene, MS-Asymmetrie-Zensus, S, Negativtest 2 | Prüfarm aus: Kugel-CPU-Hash und dd-8-mm-Kräfte bitgleich zu HEAD. Prüfarm an (B70 dd 8 mm, 300 ms): E/S exakt, Rest ≤ S, Produktion = Modus 1, Haken 2 reißt |
| **C2b** Kernelpfade unter `#ifdef RHO_RAND` (nicht emittiert) | `rr_idx`, alle Blöcke aus §2.3–2.5, gen_main-RAND-Arme | Laufzeit unverändert (Define fehlt), Kugel-Hash = C2a. `scratch_gate.sh` Exit 0 in 14 Armen |
| **C2c** Aktivierung | Allokation R1 und Papierkorb, Emission, Konstruktor-Sperren, Rho_Feld-Fassade, `lese_yslice`, VTK-Schichten, VRAM-Vorprüfung, info.cpp, Bericht 204/205, `berichte_rho_rand`, Hostselbsttest, Lesestelle ohne C0-Abbruch, Sperren REK/AUSGABE_PRUEF, Haken 3–6 | CPU, dann iGPU, dann B70 kurz. 0 Errors; 216 = 0; 219/220/204/205 Ist=Soll; Verify-Deckungspunkte wie HEAD; `memory_used` Nahfeld −120 MiB (8 mm); alle Haken feuern |
| **C2d** A/B und Dokumentation | A-Arm-Zusatzspalten (Sonde, VTK `rho_ausgabe`), Auswerteskript, RHO_RAND-PLAN.md §15 | §5 Punkt 5 grün |

K5 zwingt Puffergröße und Fassade zusammen, deshalb liegen sie gemeinsam in C2c. Vor jedem ersten GPU-Kontakt committen.

---

## 7 Fallen und Fragen an Heiko (Frage 7)

### Fallen

1. **K5:** `Memory_Container` übernimmt N aus `get_N()` (lbm.hpp:795) und greift bei D=1 ungeprüft zu (703). Jedes `c[n]` im RAND-Arm ist ein Heap-Überlauf. Fassade und Puffer müssen deshalb in denselben Commit.
2. **C1-Instrument** `d->rho[zelle(g)]` (setup.cpp:4789) liest im RAND-Arm außerhalb des Hostpuffers. Sperre nötig.
3. **`rho_rand_on`** wird heute erst nach der rho-Allokation gesetzt (lbm.cpp:628 gegen 585).
4. **Papierkorb:** Ohne +1-Slot wird jeder Sentinel-Zugriff zum Lesezugriff außerhalb des Puffers. Sentinel-Schreibvorgänge nur zählen, nie ausführen.
5. **Veralteter Cache:** Die Ebene gilt nur bei `t==ebene_t`. Verify-z+ trifft y=fNy/2.
6. **Hostpacker:** Nie `rho_pack` auf die Nachkollisionssumme (lbm.hpp:40–46), deshalb float-Cache.
7. **t statt t−1:** Das liefert bis cp 0,41 (§14a). Die Parität steckt in `load_f`.
8. **Klammerfalle** (Falle 4): Signaturen nur im Muster `)+"("+R(`, `#ifdef` nie in einer offenen Klammer. Beim `store_rho(...)` in apply_pressure_outlet bleibt der ganze Aufruf in einem Segment. Kein `#elif defined` (Falle 6).
9. **Parameterindizes:** `modus` verschiebt hits/tile_slot im C1-Kernel. `tile_slot` nur unter `sparse_on` binden (Falle 8).
10. **`atomic_inc` je Zelle** auf einen Slot bei 519 Mio VTK-Zellen: deshalb `zaehlen`.
11. **CPU beweist das Instrument nicht** (rueckleser…). Bitgleichheits-Soll auf iGPU und B70.
12. **Bitgleichheit u/Kräfte gegen HEAD** setzt voraus, dass IGC die Kollisionsarithmetik trotz geändertem Schreibblock gleich übersetzt. Ob der „Bytevergleich gegen CFD_RHO_SPARSAM=0" (setup.cpp:6247) je bestanden hat, habe ich nicht belegt gefunden.
13. **16-mm-dd:** Sponge-Sperre und weitere, noch unbekannte Grobgitter-Wächter. dd-Nahfeld lief nie auf CPU/iGPU.

### Offene Fragen an Heiko

1. **Kugel als RHO_RAND-Prüfstand** zulassen (nur Test, kein Produktionsschalter)? Empfehlung: ja, als billige CPU-Sprosse mit trennscharfen MS-Zellen.
2. **MS ohne `apply_moving_boundaries`** bestätigen? Im dd-Fall ist das konstruktiv wirkungslos, weil Σcorr = 0 am gleichförmigen Boden.
3. **Prüfschranke S** aus der FP16S-Rundung (§4.4) statt eines Handwerts, Klemmzellen separat ausgewiesen: einverstanden?
4. **VTK bei STRIDE > 1:** ganze xy-Schichten (s²-Mehrkosten) oder Stride-Parameter im Kernel? Empfehlung: erst ohne, weil die Produktion mit Stride 1 fährt.
5. **8-mm-A/B-Bezugszeile** mit oder ohne `CFD_RHO_SPARSAM`? Entscheidung B: Im Fernfeld bleibt die Maske in beiden Armen.
6. **Falls u/Kräfte trotz gleicher Arithmetik nicht bitgleich** sind (Falle 12): Ersatz-Soll CPU-Paar bitgleich plus B70 als Wirkungs-A/B?
7. **Bandbreitenanzeige** (lbm.cpp:57) unter RAND um 2 B korrigieren? Betrifft nur die Anzeige.

### Critical Files for Implementation
- /home/heiko/CFD/FluidX3D-v2/src/kernel.cpp
- /home/heiko/CFD/FluidX3D-v2/src/lbm.cpp
- /home/heiko/CFD/FluidX3D-v2/src/lbm.hpp
- /home/heiko/CFD/FluidX3D-v2/src/setup.cpp
- /home/heiko/CFD/FluidX3D-v2/werkzeuge/scratch_gate/gen_main.cpp

---
## Entscheidungen Heiko 15.09.2026 zu §7
Alle sieben Empfehlungen sind übernommen:
1. Die Kugel dient als Prüfstand für RHO_RAND, nur für Tests.
2. Die Nachkollisionssumme wird OHNE apply_moving_boundaries gebildet.
3. Die Schranke S wird aus der FP16S-Rundung hergeleitet; Klemmzellen werden getrennt geführt.
4. VTK mit Stride > 1: vorerst ganze Schichten.
5. Der 8-mm-A/B-Bezug ist die Produktionszeile mit CFD_RHO_SPARSAM=1.
6. Sind u oder die Kräfte nicht bitgleich, wird zuerst die Ursache gesucht. Ersatz ist dann ein bitgleiches CPU-Paar plus Wirkungs-A/B auf der B70.
7. Die Bandbreitenanzeige wird unter RHO_RAND korrigiert.
