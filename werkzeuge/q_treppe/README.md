# q-Treppe: Strahlschnitt der Remesh-Flaeche mit den Wandlinks (22.09.2026, Untersuchungsagent, Protokoll B24)

`remesh_q_dach.py <lauf> Nx Ny Nz x0 y0 z0 dx OFF` schneidet `export/<lauf>/remesh_flaeche.vtk` (Weltkoordinaten) mit den 18 Links der
in `dach_<lauf>.npz` ausgewaehlten Facettenzellen (xi, yi, zi, yw, wink aus facetten_histogramme.csv; Dachplateau-Kasten x_v2 2,30-2,90,
|y|<=0,6, z 1,05-1,25, achse=z, klasse=0) -- Moeller-Trumbore wie setup.cpp, Ursprung Zellmitte +0,5; Ergebnis `remeshq_<lauf>.npz`.
Befund 8 mm k3_pf_rang_8 @ ba8f4bf: Remesh-q am Normallink exakt 0,5 bei 33,9 %, |q-0,5|<=0,1 bei 85,0 %; Sub-Voxel-Information nur in
+-2 Zellen um jede Setzstufe (40,8 % der Dachzellen; 4 mm p4_neu 43,1 %, Laengsprofil zahlengleich). Fuer andere Zonen (Heckscheibe, Huelle)
den Kasten in der dach_*.npz-Erzeugung neu setzen -- die Erzeugung selbst ist im Scratchpad nicht erhalten geblieben (aus facetten_histogramme.csv
mit den obigen Filtern rekonstruierbar). Naechster Schritt laut B24: Sekanten-Prototyp mit/ohne 0,5-Zellen-Verschiebung -- Heikos Entscheid.
