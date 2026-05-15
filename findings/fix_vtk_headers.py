#!/usr/bin/env python3
"""Fix FluidX3D VTK headers for Phase 5b-DR Multi-Resolution output.

Problem: FluidX3D uses global `units` for SPACING — when DR setup has units=Far,
Near's VTK gets Far's spacing (wrong). Plus default ORIGIN auto-centers around (0,0,0)
which doesn't match physical box position for our asymmetric Far/Near boxes.

Fix: rewrite ORIGIN and SPACING in VTK ASCII header to match physical reality.
Binary data is preserved as-is (only header rewritten).

Usage:
    python3 fix_vtk_headers.py        # uses default DR Mode 1/2 Pfad A config
"""
import sys
import os

def fix_vtk_header(path, spacing_m, origin_m):
    """Fix ORIGIN and SPACING in a VTK STRUCTURED_POINTS legacy file.

    spacing_m: float in meters per cell (e.g., 0.015 for Far 15mm, 0.005 for Near 5mm)
    origin_m: tuple (x, y, z) of physical box corner in meters
    """
    with open(path, 'rb') as f:
        data = f.read()

    marker = b'LOOKUP_TABLE default\n'
    idx = data.find(marker)
    if idx == -1:
        print(f"  WARN: no LOOKUP_TABLE marker in {path}, skipping")
        return False
    header_end = idx + len(marker)
    header = data[:header_end].decode('ascii')
    binary = data[header_end:]

    # Replace ORIGIN and SPACING lines
    new_lines = []
    fixed_origin = fixed_spacing = False
    for line in header.split('\n'):
        if line.startswith('ORIGIN '):
            new_lines.append(f'ORIGIN {origin_m[0]:.6f} {origin_m[1]:.6f} {origin_m[2]:.6f}')
            fixed_origin = True
        elif line.startswith('SPACING '):
            new_lines.append(f'SPACING {spacing_m:.6e} {spacing_m:.6e} {spacing_m:.6e}')
            fixed_spacing = True
        else:
            new_lines.append(line)
    new_header = '\n'.join(new_lines)

    if not (fixed_origin and fixed_spacing):
        print(f"  WARN: didn't find ORIGIN or SPACING line in {path}")
        return False

    with open(path, 'wb') as f:
        f.write(new_header.encode('ascii'))
        f.write(binary)
    return True


def main():
    base = '/home/heiko/CFD/FluidX3D/export/'

    # === Pfad A 3:1 ratio config (matches main_setup_phase5b_dr) ===
    # Far box: x=[-4, +12], y=[-4, +4], z=[0, +5] @ 15mm cells (1067 x 534 x 334)
    far_spacing = 0.015
    far_origin = (-4.0, -4.0, 0.0)

    # Near box: physical origin (-0.505, -1.345, 0) @ 5mm cells (1320 x 540 x 339)
    # (Near box is 6.6m x 2.7m x 1.695m, located inside Far)
    near_spacing = 0.005
    near_origin = (-0.505, -1.345, 0.0)

    print(f"Fixing Far VTKs (spacing={far_spacing*1000:.0f}mm, origin={far_origin} m)")
    if os.path.isdir(base + 'dr_far'):
        for fn in sorted(os.listdir(base + 'dr_far')):
            if fn.endswith('.vtk') and not fn.startswith('mesh-'):
                path = base + 'dr_far/' + fn
                if fix_vtk_header(path, far_spacing, far_origin):
                    print(f"  OK: {fn}")

    print(f"\nFixing Near VTKs (spacing={near_spacing*1000:.0f}mm, origin={near_origin} m)")
    if os.path.isdir(base + 'dr_near'):
        for fn in sorted(os.listdir(base + 'dr_near')):
            if fn.endswith('.vtk') and not fn.startswith('mesh-'):
                path = base + 'dr_near/' + fn
                if fix_vtk_header(path, near_spacing, near_origin):
                    print(f"  OK: {fn}")

    print("\nDone. VTKs now have correct physical SPACING and ORIGIN — open both Far + Near in ParaView, they will overlay correctly.")


if __name__ == '__main__':
    main()
