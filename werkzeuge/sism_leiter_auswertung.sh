#!/bin/bash
# sism_leiter_auswertung.sh <lauf> [lauf ...] -- 22.09.2026, Auswertung der SISM-Selektivitaetsleiter (Protokoll B23).
# Je Lauf: Fehler/Abnahme, Slot 126/127 + Klemmanteil (Lage-1-Anteil = 127/(N_fac*Zaehlslots) selbst gerechnet, weil der
# Bandkernel Slot 126 mit hochzaehlt), Slot 186/187 (Band), sism_sbar.csv (Ende, Drift ab Schritt 6030 = T_WARMUP), cd/cz-Fenster
# (mitlaufend, KEINE 8-mm-Aussage), slots_verlauf letzte Zeile, dann Dachbaender + H je Zone (zonen_h.py). Immer LC_ALL=C (awk-Locale-Falle).
export LC_ALL=C; cd "$(dirname "$0")/.." || exit 1
for a in "$@"; do
  L=logs/$a.log; E=export/$a
  [ -f "$L" ] || { echo "$a: kein Log"; continue; }
  T=$(tr '|' '\n' < "$L" | sed 's/\x1b\[[0-9;]*m//g; s/^ *//' | grep -v '^$' | tr '\n' ' ' | sed 's/  */ /g')
  err=$(sed 's/\x1b\[[0-9;]*m//g' "$L" | grep -c 'Error:'); verl=$(echo "$T" | grep -o 'ABNAHME VERLETZT' | wc -l)
  s126=$(echo "$T" | grep -o 'Slot 126 (Abzug aktiv) = [0-9]*' | head -1 | grep -o '[0-9]*$'); s127=$(echo "$T" | grep -o 'Sbar) = [0-9]* = [0-9.]* %' | head -1 | awk '{print $3}')
  nfac=$(echo "$T" | grep -o 'fac_sb [0-9.]* MB fuer [0-9]* Facetten' | head -1 | awk '{print $5}'); Tst=$(echo "$T" | grep -o 'EMA T = [0-9]*' | head -1 | awk '{print $4}')
  band=$(echo "$T" | grep -o 'Slot 186[^.]*Slot 187[^.]*' | head -1 | cut -c1-120)
  nz=$(awk '!/^#/&&!/^time/{c++} END{print c}' $E/slots_verlauf.csv 2>/dev/null); zs=36
  echo "=== $a  (Errors $err, VERLETZT $verl, T = ${Tst:-?} Schritte, N_fac ${nfac:-?}) ==="
  [ -n "$s126" ] && awk -v a=$s126 -v b=$s127 -v n=${nfac:-0} -v z=$zs 'BEGIN{printf "  Slot 126 = %d, Slot 127 = %d -> Klemmanteil 127/126 = %.2f %%; Lage-1-Anteil 127/(N_fac*%d) = %.2f %%\n",a,b,100*b/a,z,(n>0?100*b/(n*z):-1)}'
  [ -n "$band" ] && echo "  Band: $band"
  awk -F, '$1~/^[0-9]/{c++; if($1>=6030&&!s){s=$3} l=$3; m=$4} END{printf "  sism_sbar: Sbar Ende %.6f, ab 6030: %.6f -> Drift %+.2f %%, sbar_max Ende %.6f (n=%d)\n",l,s,100*(l/s-1),m,c}' $E/sism_sbar.csv 2>/dev/null
  tail -1 $E/slots_verlauf.csv 2>/dev/null | awk -F, '{printf "  Rueckfall kum %.3f %%, Rang-0 %.3f %%, ohnetang13 %.3f %%, sn_gate16 %.3f %%\n",$4,100*$8/$2,100*$7/$2,100*$9/$2}'
  awk -F, '!/^#/&&!/^t/&&$1>=0.30{c++; s3+=$3;q3+=$3*$3;s5+=$5;q5+=$5*$5} END{m3=s3/c;m5=s5/c; printf "  mitlaufend (t>=0,30, n=%d, KEINE 8-mm-Aussage): cd_rest %.4f+-%.4f  cz_rest %.4f+-%.4f\n",c,m3,sqrt(q3/c-m3*m3),m5,sqrt(q5/c-m5*m5)}' $E/cd_bericht.csv 2>/dev/null
  for t in 000301 000451 000500; do [ -f $E/dach_band_${t}ms.npz ] || python3 werkzeuge/abl_dach/fx_band.py $E/feld_nah_${t}ms.vtk $E/dach_band_${t}ms.npz >/dev/null 2>&1; done
  CFD_U_LAT=0.125 python3 werkzeuge/abl_dach/fx_profil2.py $a $E/dach_band_000301ms.npz $E/dach_band_000451ms.npz $E/dach_band_000500ms.npz >/dev/null 2>&1
  for t in 000301 000451 000500; do CFD_U_LAT=0.125 python3 werkzeuge/abl_dach/fx_profil2.py ${a}_t$t $E/dach_band_${t}ms.npz >/dev/null 2>&1; done
done
echo; python3 werkzeuge/abl_dach/zonen_h.py "$@"; echo "--- je Zeitpunkt:"; python3 werkzeuge/abl_dach/zonen_h.py $(for a in "$@"; do for t in 000301 000451 000500; do echo -n "${a}_t$t "; done; done)
