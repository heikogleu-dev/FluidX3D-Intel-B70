#!/bin/bash
# Entkernung export/ (Heiko-Auftrag 2026-08-21).
# Loescht die 376 Laufordner, die in KEINER Projekt-Markdown und in KEINER
# Seriendatei vorkommen. Die Liste ist vorab geprueft: disjunkt zur Schutzliste
# (laufender Lauf, f8_standard_final, f4_std_diff), jeder Eintrag existiert,
# keiner enthaelt einen Pfadtrenner. Verzeichnis des Geloeschten mit Groesse und
# Datum steht in export/GELOESCHT-2026-08-21.txt.
set -u
cd "$(dirname "$0")/.." || exit 1
L=werkzeuge/entkernen_liste_20260821.txt
[ -f "$L" ] || { echo "Liste fehlt: $L" >&2; exit 1; }
n=0; s=0
while read -r d; do
	case "$d" in */*|.*|"") echo "UEBERSPRUNGEN (verdaechtig): $d" >&2; continue;; esac
	[ -d "export/$d" ] || continue
	s=$((s + $(du -sm "export/$d" | cut -f1)))
	rm -rf -- "export/$d" && n=$((n+1))
done < "$L"
echo "geloescht: $n Ordner, $((s/1024)) GB"
echo "verbleibend: $(ls -d export/*/ 2>/dev/null | wc -l) Ordner, $(du -sh export/ | cut -f1)"
