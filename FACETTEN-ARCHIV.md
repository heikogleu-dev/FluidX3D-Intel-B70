# FACETTEN-ARCHIV.md — annotiertes Verzeichnis der Facetten-Altdateien

Stand 2026-08-16 abends. Leitdokument ist **FACETTEN.md**; die acht Altdateien bleiben
unverändert als Belegkette liegen (nur je ein Archiv-Kopfvermerk). Hier je Datei: Inhalt,
was überholt ist (und wodurch), was als Beleg gültig bleibt. Widerspruchsregel: die JÜNGSTE
Messung gilt.

## FACETTEN-PLAN.md (2026-08-15)
C1b-Architekturplan (A1–A8: TLS-Fit host-seitig, Linkmittelpunkte, Konditionsklassen,
achsweiser Tausch, Schalter/Zähler, τ-Akkumulator), adversariale Revision (R1 Paar-Gate,
R2 Flächenfaktor, 9 Auflagen), Stufe-1-Messstände + Fenster-/Schwellen-Eichung (3³, r21>0,15,
21,9 % markiert).
**Überholt:** A5/Stufe 4 „Nebenachsen" (unter iMEM obsolet — der Tangentialvektor trägt alle
Komponenten); Paartausch als Hauptmechanismus (seit Heikos iMEM-Entscheid nur noch Kontrollarm);
die 41,8-%-Zahl (war Zählersumme, korrekt: 32,8 % → nach Eichung 21,9 %); 12-Slot-Zähler
(heute 18); „dd-Nahfeld kleiner" (falsch, in Auflage 7 selbst korrigiert).
**Gültig als Beleg:** A1–A4/A6–A8, R1/R2 samt Herleitungen, alle Auflagen, Eich-Messstände.

## FACETTEN-STUFE2.md (2026-08-15)
Stufe-2-Implementierungsplan (F1–F7), **Paartabelle aller 6 Wandseiten** (Tabelle B),
Kernel-Pseudocode, T1/T2-Tests, Messstand: Kanal-Äquivalenz zur z-WFB BITGLEICH
(CPU 12755646098055097704, iGPU 7540097450125369907), Iron-Rule-Nachprüfung.
**Überholt:** F5-Akkumulator 1 float (heute 6 float, s. FACETTEN.md 1.4); 12 Slots;
„nächster Schritt Commit 3/Stufe 3" (erledigt).
**Gültig als Beleg:** Paartabelle B (Kontrollarm + Cd-Kontaminationstest nutzen sie weiter),
Bitgleichheits-Konstruktion (F4), Messstand, Hash-Anker.

## FACETTEN-STUFE3.md (2026-08-15 nachts)
Formelblatt gekippter Kanal: Torus-Slab-Geometrie (F1–F6: Solid-Bedingung, Parameterwahl,
δ_eff, Normierung, m-Bins), Lagenstruktur/Fit-Vorhersagen (F7/F8), Gate-Abzählung (F9/F10),
Erwartungen (F11/F12), Implementierungsschritte, K4-Entscheid 26,6°.
**Überholt:** F7–F10 (paartausch-spezifische Solls; iMEM-Solls kommen aus dem Host-Census);
**F11 (BB gekippt 0,010–0,014) durch Messung widerlegt** (0,00166/0,00179 — Grenzregime);
Abnahme D5 durch N1–N3 (FACETTEN-IMEM-3X3.md) ersetzt; F12-Band als N3 zur Kugel verschoben.
**Gültig als Beleg:** Geometrie/Normierung F1–F6 (mechanismus-unabhängig, im Code verdrahtet),
y_w-Lagenprognosen (Census bestätigte 0,369/1,040 exakt; m0 0,184 → gemessen 0,187),
K4-/YWMIN-Entscheid.

## FACETTEN-CD-PFAD.md (2026-08-15/16)
Cd/Cz-Auslesepfad: Hybrid-Entscheidungen E1–E7 (Druck solid-seitig projiziert, Reibung aus dem
Akkumulator, Kontaminationstest, MS-Randfall), K1–K5-Messstand, E6-Nachtrag Druck-Zeitmittel.
**Überholt:** E5-Layout 4 float (heute 6 float); die K5-„Momentaufnahme"-Einschränkung (durch
den E6-Nachtrag im selben Dokument erledigt: Zeitmittel 0,8371 vs. Endwert 0,164).
**Gültig als Beleg:** E1–E7 (gelten wörtlich weiter, E1-Schutz jetzt sogar konstruktiv durch
die 3×3-Normal-Nullung), K1–K5 mit allen Zahlen, Phantom-Dokumentation (+1,48).

## FACETTEN-LITERATUR.md (2026-08-15)
Literatur-Recherche: Wandfunktion an schrägen Wänden (Malaspinas, Haussmann, **Asmuth/iMEM**,
Ponsin & Lozano, PowerFLOW-Surfel), Guo-Kraftterm-Frage (keine Evidenz), gekippter Kanal als
Lücke in der Literatur, Kugel-Referenzen (Achenbach ~0,45–0,5 subkritisch, Tsutsui Bodennähe).
**Überholt:** nichts inhaltlich; die Empfehlung „iMEM-Umbau" ist umgesetzt. Zwei Lesarten
wurden später präzisiert: Asmuths Normalinversion erwies sich als NÖTIG (I2-Messung), und
seine Gl.-29/30-Filterung betrifft den EINGANG, nie die Lösung (FACETTEN-IMEM-ANALYSE.md).
**Gültig als Beleg:** komplette Quellenlage inkl. DOIs/Links.

## FACETTEN-IMEM.md (2026-08-15 nachts – 2026-08-16 mittags)
iMEM-Herleitung an der Asmuth-Quelle (Gl. 20–28), D3Q19-Formulierung (1)–(15) mit 2×2-System,
Degenerationskaskade, Masse-Ledger, u_w≠0-Vorbereitung; Architektur A1–A6; Gegenprüfungs-
Revision (7 Auflagen); Messstände I0/I1 (Kanal-Äquivalenz bestanden) und I2 (DURCHGEFALLEN).
**Überholt:** Festlegung (12) „Normalimpuls messen statt vorschreiben" — **die I2-Messung hat
entschieden: Asmuth hatte recht**, die 3×3-Normal-Nullung ersetzt das 2×2 (FACETTEN-IMEM-3X3.md);
14-Slot-Legende (heute 18); alle I2-Torus-c_f-Zahlen tragen zusätzlich den z-Saum-Vorbehalt
(BB-Löcher, Fix 2a8bd20).
**Gültig als Beleg:** Herleitung (1)–(11)/(13)–(15) (im 3×3 weiterverwendet), Auflagen der
Revision (t−2-Registersemantik!), I0/I1-Messstand samt K2-Fensterbefund.

## FACETTEN-IMEM-3X3.md (2026-08-16)
3×3-Plan: Schur-Elimination (16)–(24), handgerechnete Entkopplungsbeweise je Torus-Lage
(25)–(28), Rang-Kaskade + Einzellink-Entscheid (Rang 0 → BB), Slots 14–16, Abnahme N1–N3;
J0-Loganalyse (45°-Ursache: Klemmquote 8 % + Fluktuationsjagd); J3-Messstand.
**Überholt:** die J0-Konsequenz „Fix = EMA auf u_s" — **von J3 widerlegt** (EMA verschlechtert;
Fehllektüre in FACETTEN-IMEM-ANALYSE.md korrigiert); getrennte Slot-13/15-Solls (Gate-Randzone:
Summe 13+15 prüfen); alle J3-Torus-c_f-Zahlen unter Saum- und N1-Fenster-Vorbehalt (2a8bd20).
**Gültig als Beleg:** Formeln (16)–(28) (implementierter Stand), Lagen-Momente, N1–N3-Logik,
N1-Mechanik-Ergebnis (Kontamination −763.625 → −0,16).

## FACETTEN-IMEM-ANALYSE.md (2026-08-16)
Weggabelungs-Analyse: Asmuth-Exegese (Eingangs- vs. Lösungsfilterung; warum sein Setup ohne
Klemme/mit AMD-SGS funktioniert), CSV-Befund **Arm 3 == Arm 4** (Torus misst den Fehler, nicht
das Modell), ν_t-Rückkopplungsschleife (q_i → fneq → Smagorinsky), Wegbewertung A/B/C,
PEMA-Messstand + Diskriminator, Familienfrage.
**Überholt/offen:** die Handlungsempfehlung ist ÜBERHOLT durch den Saum-Befund — vor jeder
Familien-Entscheidung erst die Saum-fixe N2-Wiederholung + Klemmskalen-Messarm (±4ut);
PEMA hat N2 nicht geheilt (0,0062).
**Gültig als Beleg:** Exegese und Schleifen-Mechanik (unabhängig vom Saum-Bug), Diskriminator-
Zahlen (3×3 hält: 0,0011; Klemmsaum 6 %), Asmuth-PDF-Fundstelle.
