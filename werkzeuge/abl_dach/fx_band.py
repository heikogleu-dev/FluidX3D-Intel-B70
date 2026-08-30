#!/usr/bin/env python3
"""fx_band.py -- schneidet aus einem feld_nah_*.vtk (STRUCTURED_POINTS, BINARY, big-endian)
ein y-BAND um die Mittelebene heraus und legt es als .npz ab.

Datei-Layout laut src/setup.cpp schreibe_vtk_feld (Zeilen 356-395):
  <ASCII-Kopf>            endet nach "VECTORS u float\n"
  u      : np*3 float32 big-endian   (SI, m/s)
  "\nSCALARS rho float 1\nLOOKUP_TABLE default\n"
  rho    : np   float32 big-endian   (Gittereinheiten)
  "\nSCALARS flags unsigned_char 1\nLOOKUP_TABLE default\n"
  flags  : np   uint8   (TYPE_S=0x01, TYPE_E=0x02, TYPE_F=0x08, ...)

Index n = x + Nx*(y + Ny*z)  -> eine (y,z)-Zeile ist in x zusammenhaengend,
und aufeinanderfolgende y bei festem z liegen ebenfalls zusammenhaengend.

Aufruf: fx_band.py <vtk> <out.npz> [halb_mm]   (Default 40 mm zu jeder Seite)
"""
import sys, os
import numpy as np

def kopf(pfad):
    with open(pfad, "rb") as f:
        roh = f.read(1024)
    txt = roh.decode("ascii", "replace")
    i = txt.index("VECTORS u float\n") + len("VECTORS u float\n")
    d = {}
    for zeile in txt[:i].split("\n"):
        t = zeile.split()
        if not t: continue
        if t[0] == "DIMENSIONS": d["dims"] = tuple(int(v) for v in t[1:4])
        elif t[0] == "ORIGIN":   d["orig"] = tuple(float(v) for v in t[1:4])
        elif t[0] == "SPACING":  d["spac"] = tuple(float(v) for v in t[1:4])
    d["off_u"] = i
    Nx, Ny, Nz = d["dims"]
    np_ = Nx*Ny*Nz
    s1 = b"\nSCALARS rho float 1\nLOOKUP_TABLE default\n"
    s2 = b"\nSCALARS flags unsigned_char 1\nLOOKUP_TABLE default\n"
    d["off_rho"]   = i + np_*12 + len(s1)
    d["off_flags"] = d["off_rho"] + np_*4 + len(s2)
    soll = d["off_flags"] + np_
    ist  = os.path.getsize(pfad)
    if soll != ist:
        raise SystemExit(f"Layout passt nicht: erwartet {soll} Bytes, Datei hat {ist}. "
                         f"Marker pruefen (SCALARS-Zeilen).")
    # Marker gegenlesen (Beweis statt Annahme)
    with open(pfad, "rb") as f:
        f.seek(i + np_*12); assert f.read(len(s1)) == s1, "rho-Marker fehlt"
        f.seek(d["off_rho"] + np_*4); assert f.read(len(s2)) == s2, "flags-Marker fehlt"
    return d

def main():
    vtk, out = sys.argv[1], sys.argv[2]
    halb_mm = float(sys.argv[3]) if len(sys.argv) > 3 else 40.0
    d = kopf(vtk)
    Nx, Ny, Nz = d["dims"]; x0, y0, z0 = d["orig"]; dx = d["spac"][0]
    jy0 = int(round((0.0 - y0)/dx))                  # y = 0
    hb  = int(round(0.001*halb_mm/dx))
    ja, jb = max(0, jy0-hb), min(Ny-1, jy0+hb)
    ny = jb - ja + 1
    print(f"{os.path.basename(vtk)}: {Nx}x{Ny}x{Nz} dx={dx*1000:.1f} mm  "
          f"y-Band j={ja}..{jb} (y={y0+ja*dx:+.3f}..{y0+jb*dx:+.3f} m), {ny} Ebenen")
    U   = np.empty((Nz, ny, Nx, 3), dtype=np.float32)
    RHO = np.empty((Nz, ny, Nx),    dtype=np.float32)
    FL  = np.empty((Nz, ny, Nx),    dtype=np.uint8)
    with open(vtk, "rb") as f:
        for z in range(Nz):
            basis = (z*Ny + ja)*Nx
            f.seek(d["off_u"] + basis*12)
            U[z] = np.frombuffer(f.read(ny*Nx*12), dtype=">f4").reshape(ny, Nx, 3)
            f.seek(d["off_rho"] + basis*4)
            RHO[z] = np.frombuffer(f.read(ny*Nx*4), dtype=">f4").reshape(ny, Nx)
            f.seek(d["off_flags"] + basis)
            FL[z] = np.frombuffer(f.read(ny*Nx), dtype=np.uint8).reshape(ny, Nx)
    np.savez_compressed(out, u=U, rho=RHO, flags=FL,
                        dims=np.array([Nx, Ny, Nz]), orig=np.array([x0, y0, z0]),
                        dx=dx, ja=ja, jb=jb, jy0=jy0)
    print("geschrieben:", out, os.path.getsize(out)//1048576, "MB")

main()
