#!/usr/bin/env python3
# vtk_yslice.py — y-Slice aus einem FluidX3D-v2-Feld-VTK, EXAKT im Format des eingebauten
# render_yslice (setup.cpp:87-115, Fassung 94a802c): |u| in SI auf 0,5*u_ref (blau 0,0,255)
# .. u_ref (weiss) .. 1,5*u_ref (rot 255,0,0), geklemmt; Solid SCHWARZ; 1 Pixel je Zelle;
# Bildzeile 0 oben (z gespiegelt); Name schnitt_<tag>_<ms>ms.png. Kein Titel, keine Achsen.
# Heiko 27.08.: "Format und Skalierung der normalen Slice-Ausgabe identisch".
# ABWEICHUNG (dokumentiert): das Feld-VTK traegt keine flags -- Solid wird als |u|==0 erkannt
# (der eingebaute Renderer maskiert (flags&(S|E))==S). Ruhende Fluidzellen im Totwasser
# koennten dadurch schwarz erscheinen; am 150-ms-Test nicht aufgefallen.
# Liest NUR die y-Ebene (Seek je z-Zeile, kein 6-GB-Vollread).
# Aufruf: vtk_yslice.py <feld_(nah|fern)_XXXXXXms.vtk> [out.png] [y_index=Ny/2] [u_ref=30]
import sys, os, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def lies_header(f):
    dims=None
    for _ in range(20):
        line=f.readline()
        if not line: break
        s=line.decode("ascii","replace").strip()
        if s.startswith("DIMENSIONS"): dims=tuple(int(x) for x in s.split()[1:4])
        elif s.startswith("VECTORS"): return dims, f.tell()
    raise SystemExit("VECTORS-Zeile nicht gefunden")

pfad=sys.argv[1]
with open(pfad,"rb") as f:
    (Nx,Ny,Nz),off=lies_header(f)
    yq=int(sys.argv[3]) if len(sys.argv)>3 else Ny//2
    ebene=np.empty((Nz,Nx,3),dtype=np.float32)
    for z in range(Nz):
        f.seek(off+((z*Ny+yq)*Nx)*12)
        ebene[z]=np.frombuffer(f.read(Nx*12),dtype=">f4").reshape(Nx,3)
u_ref=float(sys.argv[4]) if len(sys.argv)>4 else 30.0
umag=np.linalg.norm(ebene.astype(np.float64),axis=2)
solid=umag==0.0
v=np.clip(umag,0.5*u_ref,1.5*u_ref)
img=np.empty((Nz,Nx,3),dtype=np.uint8)
# unterer Ast: blau->weiss (r=g=255*t, b=255); oberer Ast: weiss->rot (r=255, g=b=255*(1-t))
t_lo=(v-0.5*u_ref)/(0.5*u_ref); t_hi=(v-u_ref)/(0.5*u_ref)
lo=v<=u_ref
c_lo=np.rint(255.0*t_lo).astype(np.uint8); c_hi=np.rint(255.0*(1.0-t_hi)).astype(np.uint8)
img[...,0]=np.where(lo,c_lo,255); img[...,1]=np.where(lo,c_lo,c_hi); img[...,2]=np.where(lo,255,c_hi)
img[solid]=(0,0,0)
img=img[::-1]  # Bildzeile 0 ist oben, z waechst nach oben (setup.cpp:109)
m=re.search(r"(\d+)ms",os.path.basename(pfad)); ms=m.group(1) if m else "000000"
tag="nah" if "nah" in os.path.basename(pfad) else ("fern" if "fern" in os.path.basename(pfad) else "x")
out=sys.argv[2] if len(sys.argv)>2 else os.path.join(os.path.dirname(os.path.abspath(pfad)),f"schnitt_{tag}_{ms.zfill(6)}ms.png")
plt.imsave(out,img)
print(f"geschrieben: {out}  ({Nx}x{Nz}, y={yq}, |u| 0,5..1,5 x u_ref={u_ref:g} m/s, Format render_yslice)")
