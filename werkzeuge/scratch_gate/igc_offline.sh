#!/usr/bin/env bash
# igc_offline.sh — GPU-freie Diagnose-Schleife fuer OpenCL-Scratch-Probleme (FluidX3D-v2)
#
# Schleife: Kernel-Quelle aendern -> offline kompilieren (ocloc/IGC, KEIN GPU-Lauf)
#           -> .zeinfo lesen (grf_count / spill_size / private_size je Kernel).
#
# Aufruf:   ./igc_offline.sh <kernel.cl> [device] [kernelname]
#   device:     0x7d67 = iGPU Arrow Lake-S (ocloc waehlt mtl-u-a0)   [Standard]
#               0xe223 = Arc Pro B70 / Battlemage G31 (ocloc: bmg-g31)
#   kernelname: Filter fuer die Auswertung, Standard: stream_collide
#
# Verifiziert 26.08.2026: identische Quelle+Optionen erzeugen im ocloc-Dump denselben
# Hash (OCL_asm35d6996934529552) und dieselben Zahlen wie der Treiber-Dump des
# g15_fix_an-Laufs -> Offline-Zahlen sind 1:1 belastbar.
set -eu

CL="${1:?Aufruf: igc_offline.sh <kernel.cl> [device] [kernelname]}"
DEV="${2:-0x7d67}"
KERNEL="${3:-stream_collide}"

# --- Schritt 0: Arbeitsverzeichnis anlegen (Dump + ocloc-Ausgabe) ---
# Auditor-B NIEDRIG B-2: Default neben der Quelle; bei Aufruf auf eine REPO-Datei wuerde das
# ins Repo schreiben -- dann IGC_WORKDIR setzen (z. B. IGC_WORKDIR=$(mktemp -d)).
BASE="$(basename "$CL" .cl)"
WORK="${IGC_WORKDIR:-$(dirname "$(readlink -f "$CL")")}/zeinfo_${BASE}_${DEV}"
rm -rf "$WORK"; mkdir -p "$WORK/dump"

# --- Schritt 1: Quelle vorbereiten -----------------------------------------------------
# Die vom Treiber gedumpten OCL_asm*.cl enden auf ein NUL-Byte. ugrep (hier als 'grep'
# installiert!) stuft solche Dateien als binaer ein und liefert LEER mit rc=1 —
# deshalb NUL strippen und in Textwerkzeugen immer 'grep -a' verwenden.
tr -d '\000' < "$CL" > "$WORK/quelle.cl"

# --- Schritt 2: Build-Optionen — exakt die des FluidX3D-v2-Treiber-Builds --------------
# Quelle: igc2/OCL_asm35d6996934529552_options.txt (vom NEO-Treiber beim g15-Lauf gedumpt).
OPTS='-cl-std=CL3.0 -cl-finite-math-only -cl-no-signed-zeros -cl-mad-enable -cl-intel-greater-than-4GB-buffer-required -w'

# --- Schritt 3: Offline-Compile mit IGC-Shader-Dump (rechnet NICHTS auf der GPU) -------
# NEOReadDebugKeys=1 schaltet die Debug-Keys frei, IGC_ShaderDumpEnable=1 schreibt
# .zeinfo/.ll/.isaasm nach IGC_DumpToCustomDir. ocloc kompiliert rein auf der CPU.
NEOReadDebugKeys=1 IGC_ShaderDumpEnable=1 IGC_DumpToCustomDir="$WORK/dump" \
  ocloc compile -file "$WORK/quelle.cl" -device "$DEV" -options "$OPTS" \
  -out_dir "$WORK" > "$WORK/build.log" 2>&1 \
  || { echo "BUILD FEHLGESCHLAGEN — siehe $WORK/build.log"; exit 1; }
grep -a 'Auto-detected' "$WORK/build.log" || true

# --- Schritt 4: zeinfo auswerten (YAML; fehlende Schluessel bedeuten 0) ----------------
ZEINFO="$(ls "$WORK"/dump/*.zeinfo 2>/dev/null | head -1)"
[ -n "$ZEINFO" ] || { echo "Keine .zeinfo im Dump — build.log pruefen."; exit 1; }
echo "zeinfo: $ZEINFO"
python3 - "$ZEINFO" "$KERNEL" <<'PY'
import re, sys
txt = open(sys.argv[1]).read(); want = sys.argv[2]
for m in re.finditer(r'- name:\s+(\S+)(.*?)(?=\n  - name:|\Z)', txt, re.S):
    name, body = m.group(1), m.group(2)
    if want not in name or 'simd_size' not in body:   # 2. Eintrag (misc_info) ueberspringen
        continue
    v = {k: (re.search(rf'\b{k}:\s+(\S+)', body) or [None,'0'])[1]
         for k in ('simd_size','grf_count','private_size','spill_size')}
    print(f"{name}: simd={v['simd_size']} grf={v['grf_count']} "
          f"private_size={v['private_size']} spill_size={v['spill_size']}")
PY
