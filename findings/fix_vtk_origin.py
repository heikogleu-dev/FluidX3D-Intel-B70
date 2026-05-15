#!/usr/bin/env python3
"""
Fix VTK STRUCTURED_POINTS header for Phase 5b-DR Multi-Resolution output.

FluidX3D writes VTK with global `units`-derived spacing and box-centered origin.
For DR runs with two different cell sizes (Far 15mm + Near 5mm), the global
units = Far causes Near's VTK to have wrong spacing. Also both VTKs use a
box-centered origin instead of the physical box position.

This script rewrites the ASCII header section with correct SPACING and ORIGIN.
The binary data remains unchanged.

Usage:
  fix_vtk_origin.py <vtk_file> <spacing_m> <ox> <oy> <oz>

Example:
  # Fix Far VTK (15mm cells, Far box at x=-4..+12, y=-4..+4, z=0..+5)
  fix_vtk_origin.py export/dr_far/u-*.vtk 0.015 -4.0 -4.0 0.0

  # Fix Near VTK (5mm cells, Near box at x=-0.505..+6.095, y=-1.345..+1.355, z=0..+1.695)
  fix_vtk_origin.py export/dr_near/u-*.vtk 0.005 -0.505 -1.345 0.0
"""
import sys
import os
import glob

def fix_vtk(path, spacing, origin):
    # Read file as binary
    with open(path, "rb") as f:
        data = f.read()

    # VTK Legacy header is ASCII until binary data starts.
    # Header lines end with \n. The binary data starts after the LOOKUP_TABLE default\n line.
    # Find header end marker:
    marker = b"LOOKUP_TABLE default\n"
    pos = data.find(marker)
    if pos < 0:
        print(f"  ERROR: cannot find header end marker in {path}")
        return False
    header_bytes = data[:pos + len(marker)]
    binary_data = data[pos + len(marker):]

    # Convert header to text
    header_str = header_bytes.decode("ascii")
    lines = header_str.split("\n")

    # Replace ORIGIN and SPACING lines
    new_lines = []
    found_origin = False
    found_spacing = False
    for line in lines:
        if line.startswith("ORIGIN "):
            new_lines.append(f"ORIGIN {origin[0]:.8f} {origin[1]:.8f} {origin[2]:.8f}")
            found_origin = True
        elif line.startswith("SPACING "):
            new_lines.append(f"SPACING {spacing:.10E} {spacing:.10E} {spacing:.10E}")
            found_spacing = True
        else:
            new_lines.append(line)

    if not found_origin or not found_spacing:
        print(f"  ERROR: cannot find ORIGIN/SPACING in {path}")
        return False

    new_header = "\n".join(new_lines).encode("ascii")

    # Write back: header + binary
    with open(path, "wb") as f:
        f.write(new_header)
        f.write(binary_data)

    print(f"  fixed: {path}  →  SPACING={spacing}, ORIGIN={origin}")
    return True

def main():
    if len(sys.argv) < 6:
        print(__doc__)
        sys.exit(1)

    pattern = sys.argv[1]
    spacing = float(sys.argv[2])
    origin = (float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5]))

    files = sorted(glob.glob(pattern))
    if not files:
        print(f"No files match pattern: {pattern}")
        sys.exit(1)

    print(f"Fixing {len(files)} VTKs: spacing={spacing}, origin={origin}")
    for path in files:
        fix_vtk(path, spacing, origin)

    print(f"Done.")

if __name__ == "__main__":
    main()
