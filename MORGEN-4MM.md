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

## Startaufstellungs-Audit 26.08. nachts: STARTKLAR (Bericht: Audit-Agent, @ 57d1aa0)

32/32 Serien-Variablen haben Konsument + Wirkpfad-Beweis (kein stiller No-Op);
ELIBB x Wandfrei-Band kollisionsfrei (getrennte Domaenen/Puffer/Zellen; Band beginnt
erst >=2 Grobzellen von der Wand); Geraete-Default fein=B70/grob=iGPU bestaetigt,
Queue-Argument wirkungslos im dd-Fall; alle Waechter aktiv (Stop-Datei raeumt sich
selbst, KIPP ab 0,05s, Dm-Waechter ELIBB-bewusst, F-Waechter erstmals im 4mm);
Binary byte-identisch zum Nachbau aus 57d1aa0 (RC=0, cmp clean).
VRAM-HERLEITUNG: 29.321 MB von 32.655 MB (~3,3 GB Luft) -- Basis ist der GEMESSENE
f4_wandfrei_v2-Fussabdruck 29.274 MB, ELIBB-Zusatz ist allein fac_q = 18 B x
2.620.462 Facetten (gemessen im f4-Log) = 47,2 MB; MLS ist pufferfrei. KEINE
Trockenprobe noetig. ELIBB-Remesh verlaengert das Setup vor run(0) um Minuten
(Host-RAM, unkritisch bei 87,6 GB) -- nicht als Haenger fehldeuten.

ZWEI PUNKTE ZUM GO:
1. NEBENWIRKUNG der Slice-Abschaltung: die Einlass-Saeulen-SONDE
   (einlass_saeule_nah.csv) schreibt nur im Slice-Block -> bleibt morgen leer.
   Bewusst bestaetigen lassen (oder Slices doch an) -- kein Code-Eingriff heute
   Nacht vor dem Produktionstag.
2. NACH Laufende manuell pruefen: im Endreport ELIBB[67] > 0 UND MLS[68] > 0
   (Slot 68 = eigener Zaehler des q>0,5-MLS-Zweigs, eingebaut im Audit-Korrektur-
   Loop 26.08. nachts; Feuerbeweis Kugel: ELIBB[67]=26825=Soll, MLS[68]=41736;
   kipp0-Bitanker haelt mit dem Zaehler: 4722579264326613690. Kein Auto-Abbruch
   im Nicht-Pur-Arm -- manuell lesen).

## Frische-Augen-Codeaudit 26.08. nachts: SAUBER MIT NOTIZEN, Korrekturen eingebaut

Kein HART-Befund; alle 14 Doku-Stichproben exakt, alle 4 MLS-Grenzfaelle nachgerechnet
(RHO_CLAMP-Kante, upt2=0, qb-Quantisierung: kein Denormal moeglich, fma-Rundung),
Harness-Binaries byte-identisch, cup/feq_ib-Platzierung per IGC-Opcode-Vergleich
beweisbar kostenlos. KORRIGIERT im Loop: (1) Slot-68-Wirkpfadzaehler fuer den
MLS-Zweig (Iron Rule Diagnostik; Slot 67 zaehlt beide Zweige), (2) stale
Laufzeit-Ansage "Reibungs-Cd kein Ist" entfernt (B3-Buchung existiert laengst --
haette morgen falsches Misstrauen gesaet), (3) lbm.hpp-Slotzahl 66->69.
NEU IN TODO NACH 4MM: dritter scratch_gate-Arm ohne F_NUR_SOLID (Gate-Blindstelle);
Datei "0.5" im Repo-Root aufraeumen.

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
