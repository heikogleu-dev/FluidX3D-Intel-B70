#!/bin/bash
# Post-Run VTK Fix Pipeline for Phase 5b-DR Multi-Resolution output.
# Run after FluidX3D Phase 5b-DR finishes (export/dr_far/ + export/dr_near/ populated).
#
# Fixes:
#   1. Field VTKs (u, rho, F, flags): ORIGIN + SPACING -> world coords
#   2. Mesh VTKs (POLYDATA): translate vertices from box-centered -> world coords
#
# Box geometry (Production 2026-05-15++):
#   Far : 13.5 x 8.01 x 5.01 m, world (-1.5, -4, 0) .. (12, 4.01, 5.01), center (5.25, 0.005, 2.505)
#   Near: 6.6 x 2.7 x 1.695 m, world (-0.495, -1.345, 0) .. (6.105, 1.355, 1.695), center (2.805, 0.005, 0.8475)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$SCRIPT_DIR/.."
FAR="$ROOT/export/dr_far"
NEAR="$ROOT/export/dr_near"

echo "=== Phase 5b-DR Post-Run VTK Fix ==="

echo ""
echo "[1/4] Far fields: SPACING 15mm, ORIGIN (-1.5, -4, 0)"
for pat in 'u-*.vtk' 'rho-*.vtk' 'F-*.vtk' 'flags-*.vtk'; do
    python3 "$SCRIPT_DIR/fix_vtk_origin.py" "$FAR/$pat" 0.015 -1.5 -4.0 0.0 2>/dev/null || echo "  (no $pat in Far)"
done

echo ""
echo "[2/4] Near fields: SPACING 5mm, ORIGIN (-0.495, -1.345, 0)"
for pat in 'u-*.vtk' 'rho-*.vtk' 'F-*.vtk' 'flags-*.vtk'; do
    python3 "$SCRIPT_DIR/fix_vtk_origin.py" "$NEAR/$pat" 0.005 -0.495 -1.345 0.0 2>/dev/null || echo "  (no $pat in Near)"
done

echo ""
echo "[3/4] Far mesh: translate by (5.25, 0.005, 2.505)"
python3 "$SCRIPT_DIR/fix_vtk_mesh.py" "$FAR/mesh-*.vtk" 5.25 0.005 2.505 2>/dev/null || echo "  (no mesh in Far)"

echo ""
echo "[4/4] Near mesh: translate by (2.805, 0.005, 0.8475) (if exported)"
python3 "$SCRIPT_DIR/fix_vtk_mesh.py" "$NEAR/mesh-*.vtk" 2.805 0.005 0.8475 2>/dev/null || echo "  (no mesh in Near)"

echo ""
echo "=== Done. VTKs in world coords; load Far+Near + mesh into ParaView. ==="
