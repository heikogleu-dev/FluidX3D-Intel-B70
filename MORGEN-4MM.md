# 4mm-Vollumfang-Lauf — Freigabe-Vorlage + Startrezept (Stand 26.08.2026 abends)

## Was morgen laeuft (nach deinem Go)

**f4_vollumfang_mls** — der erste 4mm-Lauf in Vollkonfiguration:
Basis f4_wandfrei_v2 (erster sauberer gekoppelter 4mm-Lauf, Abnahme ba35c52)
**plus** die komplette validierte Facettenkette: ELIBB mit **MLS-Wandformel**
(Commit 99a766b, Abnahmeleiter S0-S4 am 26.08. durchlaufen) und UTKORR 1,5.
Serie liegt fertig unter `logs/f4_vollumfang_serie.txt` (eine Zeile, ein Lauf).

An Bord (alles Standard, nichts zu tun): VTK am Ende + Zwischendumps alle 0,15 s,
Stop-Datei /tmp/cfd_stop, CD_EVERY=1 fuer die 100-ms-Statusmeldungen (korrigierte
Cd/Cz), Kraft-Z-Band, Kopplungsglaettung.
Interface-Slices DEAKTIVIERT (CFD_SLICE_DT=0 in der Serie -- Heiko 26.08. abends:
"erstmal deaktivieren, da kommen wir spaeter nochmal drauf zurueck"; der
CFD_SLICE_GPU-Transportweg bleibt im Code, nur ohne Slice-Ereignisse wirkpfadlos).

## Erwartung (ehrlich, nur Gemessenes)

- **Laufzeit:** f4_wandfrei_v2 brauchte 92 min (0,7 s phys.) bei 2004 MLUPs —
  noch OHNE ELIBB und VOR Remat/F-Gate. Der Vollumfang-Lauf traegt ELIBB
  (kostet auf der B70 seit dem Unroll-Fix nichts: 1478~1465 MLUPs bei 8mm)
  plus die +3,7 %-Kernel-Hebel. Erste echte 4mm-Vollkette-Zahl kommt aus dem Lauf.
- **Cz:** Bei 8mm bewegte die Facettenkette das Roh-Cz um −0,10 (g12_std +0,42 →
  g12_elibb +0,32; Fenstermittel t>=0,2 s, forces.csv). f4_wandfrei_v2 (4mm, ohne
  Kette) stand bei Cz −0,13. Mit Kette bei halber Gitterweite (Treppen-Artefakt
  nachweislich geschrumpft: Band 0,692 → 0,359) ist die erwartete RICHTUNG:
  weiter negativ, zu OF13 (−1,301) hin. Groesse: erst messen.
- **Cd:** Referenz OF13 0,599. Korrigierte Cd/Cz kommen alle 100 ms als Status.

## Startrezept (mechanisch, in dieser Reihenfolge)

```
cd /home/heiko/CFD/FluidX3D-v2
pgrep -ax FluidX3D                 # Census: MUSS leer sein
ls logs/queue.lock                 # MUSS fehlen
rm -f /tmp/cfd_stop                # alte Stop-Datei raeumen
# Desktop-GPU-Apps SCHLIESSEN (GuC-Reset-Todesart von f4_wandfrei_prod!)
werkzeuge/lauf_queue.sh logs/f4_vollumfang_serie.txt   # (im Hintergrund)
sleep 15 && werkzeuge/gt_reset_waechter.sh             # Waechter NACH Prozessstart
```
Monitoring: `cat logs/queue_status.txt`; Kraftverlauf `export/f4_vollumfang_mls/forces.csv`.

## Bekannte offene Punkte (kein Stopper, dokumentiert)

- K2-Reibungspfad-Luecke der 26-Grad-Treppenklasse (Verhaeltnis −7,4 statt 1,0;
  45 Grad fast geschlossen: +1,13). Bestand ueber die GESAMTE g12/f4-Vergleichskette —
  Vergleichbarkeit unberuehrt. Naechster Physik-Baustein danach.
- Interims im MLS-Zweig (je mit Abloeseplan im Kernel-Kommentar): tau0 statt tau_eff;
  Tangentialprojektion statt volles u_f. Grazing-kappa 0,4 / q-Boden 0,1 (aeltere Interims).
- UTKORR=1,5 ist Interim (Abloesung: linkmengen-bewusster Abtastfaktor, Baustein 2).

## TODO-Liste fuer NACH dem 4mm-Lauf

1. **Physik-Baustein 2:** K2-Luecke 26-Grad-Klasse (Rekonstruktionsziel −Σct·Δp) +
   linkmengen-bewusster Abtastfaktor (loest UTKORR=1,5 ab).
2. **MLS-Interims abloesen:** I1 tau_eff in chi durchreichen; I2 A/B upt vs volles u_f (8mm).
3. **Perf-Restliste:** FP16S-A/B (physikaendernd — braucht deine Ansage);
   UPDATE_FIELDS-Abloesung (Dual-B70-kompatibel bauen); Auto-Large-GRF neu bewerten
   (vermutlich obsolet bei Spill 0); boden_eq-3D-Range; ABSTAND-Praedikat;
   Gate-Drift-Anker (CFD_DUMP_DEFINES-Diff); W5-Totfacetten; Pur-Praezisionswaechter.
4. **y-Interface:** Breiten-Arm auf 8mm (Zielbreite aus Radnachlauf herleiten);
   fuer 4mm ist die y-Verbreiterung Anwendungsfall Nr. 1 des Dual-B70-Halo+iGPU-Plans.
5. **Harness-Reparatur Test E** (Vorzeichen-/Paritaetsartefakt der Initialisierung).
6. **Kleinkram:** wandprofil-CSVs nutzen oder streichen; PROFIL=3-Kommentarwiderspruch
   (mit Heiko klaeren); Hook-Scharfschaltung lauf_queue-Guard (einmal /hooks oeffnen).
