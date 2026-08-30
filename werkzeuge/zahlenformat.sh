#!/bin/bash
# zahlenformat.sh FP32|FP16C|FP16S -- schaltet das DDF-Zahlenformat in src/defines.hpp und baut neu.
# Genau EINES von FP16S/FP16C darf gesetzt sein; lbm.cpp:1722 bricht sonst ab (Vorfall 08.08.2026:
# beide gesetzt, die #if/#elif-Kette liess still FP16S gewinnen, die Kugelvalidierung lief im
# falschen Format). FP32 = beide auskommentiert.
set -eu
cd "$(dirname "$0")/.."
case "${1:-}" in
  FP32)  S='//#define FP16S'; C='//#define FP16C' ;;
  FP16C) S='//#define FP16S'; C='#define FP16C'   ;;
  FP16S) S='#define FP16S';   C='//#define FP16C' ;;
  *) echo "Aufruf: zahlenformat.sh FP32|FP16C|FP16S" >&2; exit 2 ;;
esac
python3 - "$S" "$C" <<'PY'
import io,sys,re
p="src/defines.hpp"; s=io.open(p,encoding="utf-8",newline="").read()
for neu,muster in ((sys.argv[1], r'^\s*(//)?#define FP16S'), (sys.argv[2], r'^\s*(//)?#define FP16C')):
    zeilen=s.split("\n"); n=0
    for i,z in enumerate(zeilen):
        if re.match(muster,z):
            rest=z.split("//",1)[-1] if "//#define" in z else z
            komm=z.split("//",2)
            nach=z[z.find("FP16"):]; nach=nach[nach.find(" //"):] if " //" in nach else ""
            zeilen[i]=neu+nach; n+=1
    assert n==1, f"{muster}: {n} Treffer"
    s="\n".join(zeilen)
io.open(p,"w",encoding="utf-8",newline="").write(s)
PY
grep -nE '^\s*(//)?#define FP16' src/defines.hpp
make -j"$(nproc)" Linux 2>&1 | grep -iE ' error' && exit 1
echo "gebaut: $1"
