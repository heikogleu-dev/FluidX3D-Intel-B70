# Far + Near VTK Merge — Python Post-Processor Design

**Status:** Spec / Design — Implementation deferred. Per User-Direktive 2026-05-16: kein In-Sim-Blending mehr (Tapered α wurde rausgenommen Commit nach `cc08f0e`), stattdessen Visualization-Smoothing als Python-Step nach jedem Production-Run.

## Ziel

Aus dem Phase-4-Multi-Res-Stack zwei separate VTKs erzeugen die sich **nahtlos** in ParaView überlagern:

1. `merged_far_<step>.vtk` — Far-VTK mit Cells innerhalb Near-Footprint **geblankt** (auf `NaN` gesetzt → ParaView zeigt sie nicht)
2. `merged_near_<step>.vtk` — Near-VTK, dessen **äußere ~20 Cells** sanft auf Far's räumlich-interpolierte Werte hingedämpft sind

Ergebnis: keine doppelte Visualisierung im Overlap, kein sichtbarer Kantensprung am Übergang, Near's high-res Inneres bleibt erhalten.

## Algorithmus

### Input
- `far.vtk`: STRUCTURED_POINTS, SPACING=20 mm, ORIGIN=(-1, -3, 0), DIMS=(650, 300, 225)
- `near.vtk`: STRUCTURED_POINTS, SPACING=4 mm, ORIGIN=(-0.4, -1.26, 0), DIMS=(1600, 630, 375)
- `BAND` (default 20): Blending-Tiefe in Near-Cells = 80 mm physisch bei 4 mm dx

### Schritt 1: Near-Footprint in Far-Cells berechnen
```
near_world_min = near.origin
near_world_max = near.origin + near.spacing * (near.dims - 1)
far_cell_min   = round((near_world_min - far.origin) / far.spacing)
far_cell_max   = round((near_world_max - far.origin) / far.spacing)
```
Für Phase 4: Far cells [30..349, 87..212, 0..74] = 320×126×75 Far-Cell-Region

### Schritt 2: Far blanken im Footprint
```python
for k, j, i in far_footprint_cells:
    far_data[k, j, i] = NaN  # ParaView blanks NaN-Zellen
```

### Schritt 3: Near's äußere Band blenden
Für jede Near-Cell `(in, jn, kn)`:
- `d_min = min(in, near_dims.x-1-in, jn, near_dims.y-1-jn, near_dims.z-1-kn)` — Distanz zur nächsten Outer-Boundary (Z=0 floor ausgenommen)
- Wenn `d_min >= BAND`: bleibt unverändert (Pure Near)
- Wenn `d_min < BAND`:
  - `w_near = (d_min / BAND)^2`  (quadratisches Decay: 0 am Rand, 1 bei BAND-Tiefe)
  - `w_far = 1 - w_near`
  - `far_val = trilinear_interp(far_data, near_world_position(in,jn,kn))`
  - `near_data[kn,jn,in] = w_near * near_data[kn,jn,in] + w_far * far_val`

### Schritt 4: Output
- Schreibe modifiziertes Far als `merged_far_<step>.vtk`
- Schreibe modifiziertes Near als `merged_near_<step>.vtk`

## Implementation-Skeleton

```python
#!/usr/bin/env python3
"""merge_far_near.py — Far+Near Multi-Res VTK Merger"""
import sys, os, struct, argparse
import numpy as np

def read_vtk_sp(path):
    """Parse VTK STRUCTURED_POINTS (legacy binary). Returns (header_str, info_dict, data_array)."""
    with open(path, 'rb') as f:
        marker = b'LOOKUP_TABLE default\n'
        buf = bytearray()
        while True:
            line = f.readline()
            if not line: raise IOError("EOF")
            buf.extend(line)
            if line == marker: break
        header_str = buf.decode('ascii')
        # Parse fields
        info = {}
        for line in header_str.split('\n'):
            parts = line.split()
            if not parts: continue
            if parts[0] == 'DIMENSIONS': info['dims']    = tuple(int(v)   for v in parts[1:4])
            elif parts[0] == 'ORIGIN':   info['origin']  = tuple(float(v) for v in parts[1:4])
            elif parts[0] == 'SPACING':  info['spacing'] = tuple(float(v) for v in parts[1:4])
            elif parts[0] == 'POINT_DATA': info['n_points'] = int(parts[1])
            elif parts[0] == 'SCALARS':
                info['scalar_name'] = parts[1]
                info['scalar_type'] = parts[2]
                info['scalar_dims'] = int(parts[3]) if len(parts) > 3 else 1
        dtype_map = {'float':'>f4', 'unsigned_char':'>u1', 'unsigned_short':'>u2'}
        dtype = dtype_map.get(info['scalar_type'], '>f4')
        n = info['n_points'] * info.get('scalar_dims', 1)
        data = np.fromfile(f, dtype=dtype, count=n)
        # Reshape: (Nz, Ny, Nx, [components])
        Nx, Ny, Nz = info['dims']
        sd = info.get('scalar_dims', 1)
        data = data.reshape((Nz, Ny, Nx, sd)) if sd > 1 else data.reshape((Nz, Ny, Nx))
    return header_str, info, data.astype(np.float32 if dtype.startswith('>f') else np.uint8, copy=True)

def write_vtk_sp(path, header_str, data):
    with open(path, 'wb') as f:
        f.write(header_str.encode('ascii'))
        # convert numpy float32 to big-endian, write
        data.astype('>f4' if data.dtype.kind == 'f' else '>u1').tofile(f)

def trilinear_far(far_data, far_info, world_xyz):
    """Sample far_data at SI world_xyz via trilinear interp."""
    fx = (world_xyz[0] - far_info['origin'][0]) / far_info['spacing'][0]
    fy = (world_xyz[1] - far_info['origin'][1]) / far_info['spacing'][1]
    fz = (world_xyz[2] - far_info['origin'][2]) / far_info['spacing'][2]
    Nx, Ny, Nz = far_info['dims']
    i0, j0, k0 = int(np.floor(fx)), int(np.floor(fy)), int(np.floor(fz))
    i1, j1, k1 = min(i0+1, Nx-1), min(j0+1, Ny-1), min(k0+1, Nz-1)
    i0, j0, k0 = max(i0, 0),     max(j0, 0),     max(k0, 0)
    wx, wy, wz = fx-i0, fy-j0, fz-k0
    # 8-corner blend
    v = (
        (1-wx)*(1-wy)*(1-wz) * far_data[k0,j0,i0] +
        wx    *(1-wy)*(1-wz) * far_data[k0,j0,i1] +
        (1-wx)*wy    *(1-wz) * far_data[k0,j1,i0] +
        wx    *wy    *(1-wz) * far_data[k0,j1,i1] +
        (1-wx)*(1-wy)*wz     * far_data[k1,j0,i0] +
        wx    *(1-wy)*wz     * far_data[k1,j0,i1] +
        (1-wx)*wy    *wz     * far_data[k1,j1,i0] +
        wx    *wy    *wz     * far_data[k1,j1,i1]
    )
    return v

def merge(far_path, near_path, band, out_dir):
    far_hdr, far_info, far_data   = read_vtk_sp(far_path)
    near_hdr, near_info, near_data = read_vtk_sp(near_path)

    # Step 1: compute Near footprint in Far cells
    near_min = np.array(near_info['origin'])
    near_max = near_min + np.array(near_info['spacing']) * (np.array(near_info['dims']) - 1)
    far_origin  = np.array(far_info['origin'])
    far_spacing = np.array(far_info['spacing'])
    fc_min = ((near_min - far_origin) / far_spacing).round().astype(int)
    fc_max = ((near_max - far_origin) / far_spacing).round().astype(int) + 1

    # Step 2: blank Far in Near footprint
    # data shape: (Nz, Ny, Nx) or (Nz, Ny, Nx, dim)
    far_data[fc_min[2]:fc_max[2], fc_min[1]:fc_max[1], fc_min[0]:fc_max[0]] = np.nan

    # Step 3: blend Near outer band
    Nx, Ny, Nz = near_info['dims']
    near_sp  = np.array(near_info['spacing'])
    near_org = np.array(near_info['origin'])
    for kn in range(Nz):
        for jn in range(Ny):
            for in_ in range(Nx):
                # Distance to nearest outer boundary (skip Z=0 floor)
                d_x = min(in_, Nx-1-in_)
                d_y = min(jn, Ny-1-jn)
                d_z = Nz-1-kn  # only Z+ counts (Z=0 is floor)
                d_min = min(d_x, d_y, d_z)
                if d_min >= band: continue
                w_near = (d_min / band) ** 2
                w_far  = 1.0 - w_near
                world = near_org + near_sp * np.array([in_, jn, kn])
                far_val = trilinear_far(far_data, far_info, world)
                if np.any(np.isnan(far_val)): continue  # already blanked
                near_data[kn, jn, in_] = w_near * near_data[kn, jn, in_] + w_far * far_val

    # Step 4: write outputs
    os.makedirs(out_dir, exist_ok=True)
    far_out  = os.path.join(out_dir, 'merged_' + os.path.basename(far_path))
    near_out = os.path.join(out_dir, 'merged_' + os.path.basename(near_path))
    write_vtk_sp(far_out,  far_hdr,  far_data)
    write_vtk_sp(near_out, near_hdr, near_data)
    print(f"Wrote {far_out}")
    print(f"Wrote {near_out}")

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('far_vtk')
    ap.add_argument('near_vtk')
    ap.add_argument('--band', type=int, default=20)
    ap.add_argument('--out-dir', default='export/merged/')
    args = ap.parse_args()
    merge(args.far_vtk, args.near_vtk, args.band, args.out_dir)
```

## Performance / Scalability

- Naive Python triple-loop über 378 M Near-Cells: ~30 min pro Feld (CPython). Optimieren:
  - **Numpy-vectorize**: nur Boundary-Band-Cells berechnen (~12% der Cells), Rest skippen. **~2 min/Feld**.
  - **Numba JIT** dekorierte trilinear-Interp: **~30 s/Feld**.
- Speicher: ~1.5 GB für `u-Near` als float32 (3-Komponenten, 378 M Cells).

## ParaView-Workflow nach Merge

```bash
findings/merge_far_near.py export/dr_far/u-000008700.vtk export/dr_near/u-000043500.vtk
findings/merge_far_near.py export/dr_far/rho-000008700.vtk export/dr_near/rho-000043500.vtk
findings/merge_far_near.py export/dr_far/F-000008700.vtk export/dr_near/F-000043500.vtk
QT_QPA_PLATFORM=xcb ~/.local/bin/paraview-gpu export/merged/*.vtk export/dr_far/mesh-*.vtk
```

ParaView zeigt nahtlose Überlagerung. Kein Clip nötig (Far ist im Near-Footprint NaN-geblankt).

## Offene Fragen

1. **Vector-Felder** (`u`, `F`): pro Komponente blenden — wird durch die `scalar_dims > 1` Logik abgedeckt
2. **Integer-Felder** (`flags`): NaN nicht möglich, alternativ "outside Near" Marker-Wert. Oder einfach skip — flags-Visualisierung ist primär für Diagnose und braucht den Blend nicht
3. **Mesh-VTK**: bleibt unverändert, eine STL-Repräsentation für beide Domänen
