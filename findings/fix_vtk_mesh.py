#!/usr/bin/env python3
"""
Translate VTK POLYDATA mesh vertices by a fixed offset.

FluidX3D's write_mesh_to_vtk emits POLYDATA with vertex coords box-centered
on the LBM domain (offset = LBM box center). The fields written via
write_to_vtk are box-corner indexed; after fix_vtk_origin.py the field VTKs
are in world coords with a chosen ORIGIN. The mesh therefore needs the same
shift (= LBM box-center position in world coords) to overlay correctly.

For the Phase 5b-DR Production run 2026-05-15:
  Far box: world (-1.5, -4, 0) to (12, 4.01, 5.01) -> center (5.25, 0.005, 2.505)
  Near box: world (-0.495, -1.345, 0) to (6.105, 1.355, 1.695) -> center (2.805, 0.005, 0.8475)

Usage:
  fix_vtk_mesh.py <vtk_pattern> <dx> <dy> <dz>

Example:
  fix_vtk_mesh.py 'export/dr_far/mesh-*.vtk' 5.25 0.005 2.505
"""
import sys
import struct
import glob

def fix_mesh(path, offset):
    with open(path, "rb") as f:
        data = f.read()

    # Header is ASCII up to and including "POINTS N float\n"
    # Find the POINTS line
    nl = 0
    points_line_start = None
    points_line_end = None
    for i in range(len(data)):
        if data[i] == 0x0A:  # \n
            line = data[nl:i].decode("ascii", errors="replace")
            if line.startswith("POINTS "):
                points_line_start = nl
                points_line_end = i + 1
                npoints = int(line.split()[1])
                break
            nl = i + 1
            if i > 2048:
                print(f"  ERROR: POINTS line not found in first 2KB of {path}")
                return False

    if points_line_end is None:
        print(f"  ERROR: no POINTS line in {path}")
        return False

    header_bytes = data[:points_line_end]
    point_bytes = data[points_line_end : points_line_end + npoints * 12]
    tail_bytes = data[points_line_end + npoints * 12 :]

    if len(point_bytes) != npoints * 12:
        print(f"  ERROR: point block truncated in {path}")
        return False

    # Vertex coords: big-endian float32 triples
    fmt = ">" + "f" * (3 * npoints)
    coords = list(struct.unpack(fmt, point_bytes))
    for i in range(npoints):
        coords[3*i  ] += offset[0]
        coords[3*i+1] += offset[1]
        coords[3*i+2] += offset[2]
    new_point_bytes = struct.pack(fmt, *coords)

    with open(path, "wb") as f:
        f.write(header_bytes)
        f.write(new_point_bytes)
        f.write(tail_bytes)

    print(f"  fixed: {path}  ->  translated by {offset}, {npoints} verts")
    return True

def main():
    if len(sys.argv) < 5:
        print(__doc__)
        sys.exit(1)
    pattern = sys.argv[1]
    offset = (float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"No files match pattern: {pattern}")
        sys.exit(1)
    print(f"Translating {len(files)} mesh VTKs by {offset}")
    for path in files:
        fix_mesh(path, offset)
    print("Done.")

if __name__ == "__main__":
    main()
