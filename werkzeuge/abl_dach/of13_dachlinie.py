#!/usr/bin/env python3
"""of13_dachlinie.py -- OF13-Referenz: Oberkontur (Dach/Heckscheibe) in der Mittelebene,
Wandschubspannung darauf, und der Wanddruck aus der ersten Fluidzelle.

Eingang (alle aus ~/CFD-Cases/mr2v40H, Zeitschritt 1200, erzeugt in dieser Sitzung):
  of13_vehicle_C.txt                 Flaechenmitten der Patch-Flaechen "vehicle" (aus 1200/C.gz)
  of13_vehicle_wallShearStress.txt   tau_w/rho je Flaeche          (aus 1200/wallShearStress.gz)
  of13_slab_y003.txt                 Zellmitten + p + U im Band |y|<0.03 m (aus 1200/C.gz,p.gz,U.gz)
Alle drei in derselben Reihenfolge wie die Patch-Flaechen bzw. Zellen.
"""
import numpy as np, sys, os
SP = os.path.dirname(os.path.abspath(__file__))
XOFF = 2.2063          # x_v2 = x_OF13 + XOFF
UINF = 30.0
RHO  = 1.225

C  = np.loadtxt(SP+"/of13_vehicle_C.txt")
TW = np.loadtxt(SP+"/of13_vehicle_wallShearStress.txt")
print("Fahrzeug-Patch:", C.shape, " x", C[:,0].min(), C[:,0].max(),
      " y", C[:,1].min(), C[:,1].max(), " z", C[:,2].min(), C[:,2].max())
np.save(SP+"/of13_vehicle_C.npy", C.astype(np.float32))
np.save(SP+"/of13_vehicle_TW.npy", TW.astype(np.float32))
