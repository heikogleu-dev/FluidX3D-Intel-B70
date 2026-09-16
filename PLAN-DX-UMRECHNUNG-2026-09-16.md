# PLAN dx-Umrechnung (TODO 4a) -- 16.09.2026

Stand: git master HEAD 65774bd (Planungsagent, abgelegt durch mich um 17:55, Inhalt wörtlich). Reine Planung, nichts gebaut. Grundlage: Heikos Entscheid 14:58
("Abstaende/Masse nicht aufloesungsabhaengig aendern, nur Physik/Zeiten anpassen"), Protokoll 2026-09-16 Abschnitte 14:36 und 15:05, TODO.md Punkt 4a.
Bau ERST nach leerer Queue (Lehre 16.09.: kein make bei laufender Queue).

## 0. Befunde beim Lesen, die der Auftrag nicht nennt (alle mit Datei:Zeile)

B0.1 **Zeitwaechter-Meldung rechnet falsch.** src/setup.cpp:145-146 druckt `schritte*dt/skal*1e3` als "ms bei der Vorgabe-Gittergeschwindigkeit". Richtig ist `schritte*dt*skal` (dt_vorgabe = dt*U_LAT_VORGABE/u_lat = dt*skal). Beleg: logs/p4_pu8.log:88-89 "CFD_SGS_SISM_AB = 15000 ... steht fuer 416.67 ms" -- 15000 Schritte bei dt 1e-5 s sind 150,00 ms; der Fehlerfaktor ist 1/skal^2 = 2,78. logs/p375_a.log:90-91: 390,63 statt 140,63 ms. Reine Meldung, keine Physik; eigener Commit.

B0.2 **werkzeuge/basis_aus_lauf.py kennt `schritte_fein` nicht.** EINHEIT (Zeilen 18-36) fuehrt CFD_SLICE_NEAR_STEPS gar nicht; der Docstring (Zeilen 9-15) nennt weder `zellen_grob_laenge` noch `schritte_fein`. Der Basis-Commit 3c3e6c7 (12.09.) aenderte nur basis/fahrzeug_dd.basis:127, das Werkzeug zuletzt in 12abccc (03.09.). Eine Neuerzeugung heute stufte CFD_SLICE_NEAR_STEPS still auf `modus` zurueck (Zeile 95: `EINHEIT.get(k,'modus')`).

B0.3 **werkzeuge/basis_zeile.py:19** kennt `schritte_fein` ebenfalls nicht und laesst den Wert stehen -- heute FALSCH (gibt fuer 8 mm 5000 statt der geforderten 2500 aus), nach dem Umbau zufaellig richtig. Muss explizit werden.

B0.4 **Zweite Hauptfalle (neben der Doppelumrechnung): CFD_SAMPLE_EVERY und CFD_ZAEHL_TAKT.** Beide gehen durch die Skalierung (src/setup.cpp:8084 `env_schritte("CFD_SAMPLE_EVERY", 25u)` in GROBEN Schritten; src/lbm.cpp:564 `zaehl_takt()`). Mit dx-Faktor 0,5 wuerde bei 8 mm aus dem Code-Default 25 -> 12,5 -> llround 13 und aus CFD_ZAEHL_TAKT=100 (logs/kl_std.txt) -> 50. 13 statt 25 Grobschritte verdoppelt die Zeilen in forces.csv und cd_facetten.csv (CFD_FAC_CD_EVERY=1, setup.cpp:9100-9109) -- **die 8-mm-Regression waere NICHT bitgleich, obwohl die Physik unveraendert ist**. Zudem 8115: SLICE_NEAR_STEPS 2500 % (13*4) != 0 -> Slices quantisieren anders. Loesung in E: die 8-mm-Regressionszeile traegt CFD_SAMPLE_EVERY=50 und CFD_ZAEHL_TAKT=200 (4-mm-Referenzwerte = 2 ms Abtastung, 200 Schritte Zaehltakt; fuehren bei 8 mm exakt auf die historischen 25/100).

B0.5 **Reihenfolge der Leser erzwingt eine umgebungsreine Funktion.** Im dd-Fall laeuft `env_schritte("CFD_SLICE_NEAR_STEPS")` bei src/setup.cpp:6551 VOR `u_lat_schalter` (6570) und VOR `dx_f` (6584). Genau deshalb ist `ulat_skal()` seit dem 12.09. eine reine Funktion der Umgebung (src/lbm.cpp:528-548). Der dx-Faktor muss demselben Muster folgen (Umgebung + Ist=Soll-Setzer), sonst kehrt der Fehler "gesetzt, NACHDEM gelesen" zurueck.

B0.6 **Welche Faelle auf 4 mm definiert sind.** CFD_DX lesen nur fahrzeug (setup.cpp:6081), fahrzeug_dd (6584) und fernfeld (9850) -- alle mit si_u = 30 (6563, 9847, fahrzeug ebenso). Die Kugel liest CFD_KUGEL_DX (5494) und traegt EIGENE Schrittwerte je Zeile (logs/spz_test*.txt: CFD_KUGEL_DX=40 mit SISM_AB=15000, SISM_T=2500 -- nicht 4-mm-definiert). Der Kanal leitet SISM_T aus T_ett her (lbm.cpp:222) und wendet CFD_U_LAT nicht an (setup.cpp:4410), liest aber SISM/DIAG_AB ueber env_schritte (4356, 4368). facetten_test ruft env_schritte nicht auf. => dx-Faktor NUR fuer die drei CFD_DX-Faelle, sonst exakt 1,0 -- angesagt.

B0.7 Der u_lat-Zeitwaechter (setup.cpp:123) kehrt bei u_lat == Vorgabe sofort zurueck; nach dem Umbau muss die Bedingung `schritt_skal()==1.0` heissen, sonst schweigt er bei 8 mm mit Vorgabe-u_lat, obwohl umgerechnet wird.

## A. Referenz-dx und env_schritte

Eine Konstante, an EINER Stelle, fuer setup.cpp UND lbm.cpp sichtbar (lbm.cpp braucht sie fuer zaehl_takt, das den Kernel-Define speist, lbm.cpp:2328):

**src/lbm.hpp, hinter Zeile 200** (neben ulat_skal):
```cpp
// ★ 16.09.2026 (TODO 4a, Heiko 14:58): Schritt-Schalter folgen dx UND u_lat, weil dt = u_lat*dx/si_u.
// Referenzsprosse der schrittbasierten Schalter ist 4 mm (setup.cpp:21-26: dt = 1e-5 s bei 4 mm, u_lat 0,075, si_u 30).
#define DX_SCHRITT_VORGABE_MM 4.0f
double dx_skal();                      // DX_SCHRITT_VORGABE_MM/CFD_DX in fahrzeug/fahrzeug_dd/fernfeld, sonst exakt 1.0
void   dx_skal_setzen(const double s); // Ist=Soll aus dem Fall, wie ulat_skal_setzen
double schritt_skal();                 // = ulat_skal()*dx_skal(): DER Faktor fuer env_schritte und zaehl_takt
```

**src/lbm.cpp, hinter Zeile 548:**
```cpp
static double dx_skal_aus_umgebung() {
	const char* c = getenv("CFD_CASE");
	const bool dx_fall = c!=nullptr&&(string(c)=="fahrzeug_dd"||string(c)=="fahrzeug"||string(c)=="fernfeld"); // die drei CFD_DX-Leser (setup.cpp:6081/6584/9850)
	if(!dx_fall) return 1.0; // kugel: CFD_KUGEL_DX mit eigenen Schrittwerten je Zeile; kanal: T aus T_ett; facetten_test: kein Leser
	float dx = 4.0f; if(const char* v = getenv("CFD_DX")) { const float x = (float)atof(v); if(x>0.0f) dx = x; } // dieselbe float-Rundung wie env_f
	return (double)DX_SCHRITT_VORGABE_MM/(double)dx;
}
static double g_dx_skal = 0.0;
double dx_skal() { if(g_dx_skal==0.0) g_dx_skal = dx_skal_aus_umgebung(); return g_dx_skal; }
void dx_skal_setzen(const double s) {
	if(!(s>0.0)) print_error("dx_skal_setzen: die Skalierung muss positiv sein.");
	if(fabs(s-dx_skal())>1e-6*fmax(1.0, fabs(s))) print_error("dx-Schrittskalierung Ist != Soll: aus der Umgebung "+to_string((float)dx_skal(),7u)+", aus dem gefahrenen dx "+to_string((float)s,7u)+". Die Schritt-Schalter waeren gegen eine andere Sprosse umgerechnet als gerechnet wird.");
}
double schritt_skal() { return ulat_skal()*dx_skal(); }
```
**src/lbm.cpp:564:** `*ulat_skal()` -> `*schritt_skal()` (Zaehltakt folgt demselben EINEN Faktor; Begruendung wie 562-563).

**src/setup.cpp:**
- 76-77: `ulat_skal()!=1.0` -> `schritt_skal()!=1.0`.
- hinter 73 (Ansage, nur wenn dx_skal()!=1.0): `print_warning("SCHRITT-SCHALTER WERDEN AUF DIE SPROSSE UMGERECHNET, dx-Faktor "+to_string((float)dx_skal(),5u)+" (Referenz "+to_string(DX_SCHRITT_VORGABE_MM,2u)+" mm, dt = u_lat*dx/si_u); Gesamtfaktor mit u_lat "+to_string((float)schritt_skal(),5u)+". Serienzeilen tragen seit dem 16.09.2026 auf ALLEN Sprossen die 4-mm-Werte -- ein von Hand halbierter 8-mm-Wert wuerde DOPPELT umgerechnet; der Basis-Waechter faengt das (schritte_fein = Wert bleibt).");`
- 93: `const double sk = schritt_skal();`
- 103-105 Meldung: `... ", Faktor "+to_string((float)sk,5u)+" (u_lat "+to_string((float)ulat_skal(),5u)+" x dx "+to_string((float)dx_skal(),5u)+") -- dieselbe physikalische Zeit auf dieser Sprosse und Gittergeschwindigkeit."`; Default-Text 104: "(Code-Default, fuer u_lat = 0,075 und dx = 4 mm gewaehlt)".
- 123: `if(schritt_skal()==1.0||!(dt_f>0.0f)||!(dt_c>0.0f)) return;`  124: `const double skal = schritt_skal();`
- 146: `schritte*dt*skal*1e3` (B0.1) und Text "steht fuer X ms auf der Referenz (dx 4 mm, u_lat 0,075)".
- 151-154: "folgen u_lat UND dx seit dem 16.09.2026".
- 20-33 Kommentar: "auf der 8-mm-Sprosse von Hand halbiert" (25-26) streichen, durch "seit 16.09.2026 automatisch, dx-Faktor DX_SCHRITT_VORGABE_MM/dx" ersetzen.
- Ist=Soll-Setzer je Fall: kanal vor 4356 `dx_skal_setzen(1.0);`; kugel hinter 5494 `dx_skal_setzen(1.0); print_info("Kugel: Schritt-Schalter werden NICHT auf CFD_KUGEL_DX umgerechnet -- ihre Werte sind je Zeile fuer die gefahrene Sprosse gewaehlt (spz_*: 15000/2500 bei 40 mm), nicht auf 4 mm definiert. Die u_lat-Umrechnung bleibt.");`; fahrzeug hinter 6081, fahrzeug_dd hinter 6584, fernfeld hinter 9850 jeweils `dx_skal_setzen((double)DX_SCHRITT_VORGABE_MM/(double)env_f("CFD_DX", 4.0f));` (mm-Wert, dieselbe Rundung wie in lbm.cpp).

Welche Schalter bekommen den Faktor: alle env_schritte-Leser in den drei CFD_DX-Faellen -- CFD_SGS_SISM_AB/_T, CFD_SGS_VD_AB, CFD_SGS_DIAG_AB (6755/6769), CFD_SLICE_NEAR_STEPS (6551, 8103), CFD_SAMPLE_EVERY (6828, 6910, 8084; grob, dt_c = ratio*dt_f, Faktor identisch), CFD_ZAEHL_TAKT (lbm.cpp:564). Nicht: CFD_FAC_CD_EVERY (dimensionslos), Null bleibt Null (98).
Rechenprobe (double): 3,75 mm: 0,6000000238*1,0666667 = 0,64000003 -> 15000 -> 9600, 5000 -> 3200, 25 -> 16, 1000 -> 640 (alle eindeutig). 8 mm, u_lat Vorgabe: 1,0*0,5 exakt -> 15000 -> 7500, 5000 -> 2500, 50 -> 25, 200 -> 100. 4 mm: 0,6*1,0 = 0,6 exakt -> unveraendert (9000/3000/3000/15/600). lbm.cpp:223 "SISM_AB < 3*T": 9600 < 9600 falsch -> keine Warnung.

Erkennung einer von Hand halbierten 8-mm-Zeile: der CODE kann sie nicht erkennen (Zeitwaechter-Kommentar 132-136 sagt das schon). Der Schutz ist der Basis-Waechter (B + D): mit SISM_AB 15000 schritte_fein in der Basis und der Regel "Wert bleibt" faellt 7500 als "BASIS ABWEICHEND: CFD_SGS_SISM_AB: Soll 15000, Ist 7500" mit print_error (6528, 6541). Ohne Basis (CFD_BASIS=aus, fahrzeug, fernfeld) gibt es keinen Schutz -- siehe G.

## B. Basis-Waechter: Regel fuer schritte_fein

src/setup.cpp:6404 `pruefe_basis(const string&, const float dx_lauf)` -- Parameter u_lat_lauf entfaellt; 6430 entfaellt; Aufruf 6546: `pruefe_basis(get_exe_path()+"../basis/fahrzeug_dd.basis", env_f("CFD_DX", 4.0f));`.
6473: `schritte_fein` aus der Umrechnungsgruppe nehmen; 6476-6477: u_fak entfaellt (`roh = atof(b.wert)*skal`). Neuer Kommentar bei 6466-6472: `schritte_fein  WERT BLEIBT: der Serienwert ist auf dx_ref und U_LAT_VORGABE definiert, env_schritte rechnet ihn im Lauf LAUT um (16.09.2026). Der Waechter prueft den ROHEN Wert -- ein von Hand umgerechneter (7500 bei 8 mm) faellt als ABWEICHEND auf; das ist der Schutz vor der Doppelumrechnung.`
Neu hinter 6432: `if(fabs(dx_ref-DX_SCHRITT_VORGABE_MM)>1e-6f) print_error("BASIS-WAECHTER: dx_ref "+to_string(dx_ref,2u)+" != DX_SCHRITT_VORGABE_MM "+to_string(DX_SCHRITT_VORGABE_MM,2u)+" -- die schritte_fein-Werte der Basis waeren auf einer anderen Sprosse definiert als env_schritte annimmt.");` (basis/fahrzeug_dd.basis:3 traegt dx_ref 4).
6514: Text zu CFD_U_LAT anpassen ("schritte_fein wird von env_schritte umgerechnet, der Waechter prueft den Referenzwert"). 6405-6411 Kommentarblock nachziehen.
Damit traegt die Serienzeile bei jeder Aufloesung den 4-mm-Wert, und der Waechter prueft genau das.

## C. Einheiten umtragen

**werkzeuge/basis_aus_lauf.py**, EINHEIT (18-36):
- `"CFD_N2F_BAND_N":"zellen_grob_laenge"` (war zellen_grob, Zeile 23); `"CFD_N2F_BAND_PLATEAU"` und `"CFD_N2F_BAND_WANDFREI"` -> zellen_grob_laenge (aus Zeile 31 heraus); `"CFD_BODEN_EQ_ABSTAND":"zellen_fein"` (aus Zeile 26 heraus; Leser setup.cpp:6868 Nahfeld und 6918 Fernfeld -- derselbe Zellwert, beide skalieren mit dx).
- Neu: `"CFD_SLICE_NEAR_STEPS":"schritte_fein","CFD_SGS_SISM_AB":"schritte_fein","CFD_SGS_SISM_T":"schritte_fein","CFD_FAR_LX":"phys","CFD_PERF_AB":"phys"` (B0.2).
- Docstring 9-15 ergaenzen: `zellen_grob_laenge  gleiche WELTlaenge, skaliert mit dx_ref/dx (dx_c = ratio*dx)` und `schritte_fein  Zeitschritte, WERT BLEIBT (auf dx_ref/u_lat 0,075 definiert); env_schritte rechnet im Lauf um, der Waechter prueft den Rohwert (16.09.2026)`.
**werkzeuge/basis_zeile.py:19** explizit: `if einheit=="schritte_fein": pass  # Wert bleibt, env_schritte rechnet im Lauf um (16.09.2026)`; Kopfkommentar 2-4 ergaenzen.
**basis/fahrzeug_dd.basis** nur maschinell (D).

Kopplungspruefungen nach Umrechnung (setup.cpp:7031 WANDFREI >= N Fehler; 7049 PLATEAU = max(1,min(N-1,env)); 7051 WANDFREI+PLATEAU >= N-1 Fehler; 7054-7057 PROFIL 3 nur Warnung):
- 3,75 mm: N llround(8,533)=9 ("nicht eindeutig", 6480), WANDFREI llround(2,133)=2, PLATEAU 2, BODEN_EQ_ABSTAND 2 (= 7,5 mm, deklarierte Diskretisierungsgrenze): 2+2 >= 8 falsch -> geht auf.
- 8 mm: N 4, WANDFREI 1, PLATEAU 1, ABSTAND 1: 1+1 >= 3 falsch -> geht auf (Band 4 x 32 mm = 128 mm wie bei 4 mm).
- 16 mm (2 Altzeilen): N 2, WANDFREI llround(0,5)=1, PLATEAU 1: 1+1 >= 1 -> print_error 7051 -- muss dort deklariert werden.
- 3,5 mm: N 9, W 2, P 2 -> geht auf.

## D. Basis-Ergaenzung (Punkt 6)

basis_aus_lauf.py kann heute nur aus einer LAUF.txt neu erzeugen (58-96). Eine Neuerzeugung aus export/p4_pu8/code/LAUF.txt zoege ~15 weitere Schalter (SCHRITTE_PRO_ZELLE, POSITIV, U_KLEMME, PTRT, FAC_DETEPS, FAC_PINV, QUEUE_*, ...) als "modus" in die Basis und widersprae Heikos "kein neuer Bezug, basis/ bleibt" (Protokoll 09:59); CFD_FAR_LX steht dort gar nicht (Default). Kleinster sauberer Weg: **Modus `--nachziehen`** im selben Werkzeug (EINHEIT bleibt die eine Quelle):
```
basis_aus_lauf.py --nachziehen basis/fahrzeug_dd.basis CFD_SGS_SISM_AB=15000 CFD_SGS_SISM_T=5000 CFD_FAR_LX=12.2720 CFD_PERF_AB=0.1
```
Verhalten (~25 Zeilen Python vor Zeile 58): Kopf und Kommentare unveraendert uebernehmen; Einheitenspalte JEDER Datenzeile aus EINHEIT neu setzen, Werte NICHT anfassen; genannte NAME=WERT anhaengen, Fehler wenn der Name schon vorhanden ist; Datenzeilen sortiert ausgeben wie heute (92); unter der BEGRUENDUNGSMARKE eine Zeile `# NACHGEZOGEN (basis_aus_lauf.py --nachziehen, 2026-09-16, Heiko-Entscheid 14:58): Einheiten N2F_BAND_N zellen_grob->zellen_grob_laenge, N2F_BAND_WANDFREI/PLATEAU modus->zellen_grob_laenge, BODEN_EQ_ABSTAND modus->zellen_fein; ergaenzt SGS_SISM_AB 15000 / SGS_SISM_T 5000 (setup.cpp:24-25, p4_pu8), FAR_LX 12.2720 (Code-Default setup.cpp:6609), PERF_AB 0.1 (setup.cpp:8559). schritte_fein heisst ab jetzt WERT BLEIBT -- die Absaetze vom 12.09. (Zeilen 6-14) und 07.09. (87-91, "2500 bei 8 mm") sind damit ueberholt.` Quellen der Werte: SISM 15000/5000 aus export/p4_pu8/code/LAUF.txt (Umgebungsblock) und setup.cpp:24-25; FAR_LX Code-Default 6609; PERF_AB 0.1 (Code-Default 0.100, 8559; p4_pu8 0.1).
Erwartete neue/geaenderte Datenzeilen: `CFD_BODEN_EQ_ABSTAND 2 zellen_fein`, `CFD_FAR_LX 12.2720 phys`, `CFD_N2F_BAND_N 8 zellen_grob_laenge`, `CFD_N2F_BAND_PLATEAU 2 zellen_grob_laenge`, `CFD_N2F_BAND_WANDFREI 2 zellen_grob_laenge`, `CFD_PERF_AB 0.1 phys`, `CFD_SGS_SISM_AB 15000 schritte_fein`, `CFD_SGS_SISM_T 5000 schritte_fein`; `CFD_SLICE_NEAR_STEPS 5000 schritte_fein` unveraendert.
Unter der HEUTIGEN Waechterregel (Soll = 15000*dx_ref/dx) sind die Ergaenzungen mit allen bestehenden 4-mm- (15000) und 8-mm-Zeilen (7500) vertraeglich -- der Basis-Commit kann und muss VOR dem Code-Commit liegen (sonst gaebe es ein Fenster, in dem 7500 still zu 3750 wuerde).

## E. Umstellung der Serienzeilen

**logs/kl_std.txt** (beide Zeilen, neue Namen kl_std_a_dx_b70 / kl_std_pu_dx_b70, Kopf begruenden): `CFD_SGS_SISM_AB=7500 CFD_SGS_SISM_T=2500 CFD_SLICE_NEAR_STEPS=2500` -> `15000 / 5000 / 5000`; `CFD_ZAEHL_TAKT=100` -> `200`; neu `CFD_SAMPLE_EVERY=50` (B0.4); `CFD_BASIS_ABWEICHUNG=CFD_T_END=0.301,CFD_T_WARMUP=0.2,CFD_N2F_BAND_N=8,CFD_N2F_BAND_WANDFREI=2,CFD_N2F_BAND_PLATEAU=2,CFD_BODEN_EQ_ABSTAND=2` (die Regression faehrt die historischen 8-mm-Masse "Anzahl bleibt", deklariert; sonst Soll 4/1/1/1). Alles andere wortgleich.
**logs/p375_a.txt** -> neue Zeile p375_b: `CFD_SLICE_NEAR_STEPS=5333` -> `5000`; `CFD_N2F_BAND_N=8` -> `9`; Deklaration um `CFD_FAR_LX=12.2250` ergaenzen (steht nach D in der Basis mit 12.2720; Nx%16-Wahl bleibt begruendet). KRAFT_ZBAND 4, WAKE_ABSTAND 34, WAKE_START_X 332, SPONGE_N 68 bleiben (Laengen, Waechter-geprueft).
**Nicht anfassen:** logs/apg8_teil1.txt, apg8_teil2.txt, kl_apg_reg.txt und ~40 weitere 8-mm-Dateien mit 7500/2500/2500 (grep: 156 Nennungen) -- historisch; bei Wiederverwendung bricht der Waechter ab (gewollt). logs/p4_pu8.txt bleibt unveraendert (F3).

## F. Abnahme (je Punkt die Beweiszeile)

1. **Kugel CPU, 4 Laeufe** (logs/ka3_cpu.txt-Muster, neue Namen ka4_*): FELD-HASH(u) bitgleich -- Soll a0 10537532691050520009, h2 2458624483456925129, h3 8862123052121906633, k1 9733359479425121737 (logs/ka3_ku25_*_cpu.log:398 "FELD-HASH(u) = ..."). Zusatz: Info-Zeile "Kugel: Schritt-Schalter werden NICHT auf CFD_KUGEL_DX umgerechnet".
2. **8 mm B70** kl_std_a_dx_b70: `cmp export/kl_std_a_b70/forces.csv export/kl_std_a_dx_b70/forces.csv` und cd_facetten.csv bitgleich. Log-Belege: "SCHRITT-SCHALTER UMGERECHNET: CFD_SGS_SISM_AB = 15000 -> 7500 (aus der Umgebung), Faktor 0.50000 (u_lat 1.00000 x dx 0.50000)", "... SISM_T = 5000 -> 2500", "... SLICE_NEAR_STEPS = 5000 -> 2500", "... SAMPLE_EVERY = 50 -> 25", "Zaehltakt: 100 feine Schritte (aus 200 umgerechnet)"; unveraendert gegen logs/kl_std_a_b70.log:118 "[Nahfeld] SISM-Zeitbasis: T = 2500 Schritte = 0.0500 s, klassisch bis Schritt 7500", :777 "SLICE-KADENZ: alle 2500 Near-Steps = 625 Outer = 50.0 ms", :802 "Erwartete Samples nach dem Warmlauf: 50"; Basis: "Keine Abweichung unter den gefuehrten Schaltern" mit "6 Abweichungen deklariert" (heute :51 "2").
3. **4 mm p4_pu8** (Zeile unveraendert, neuer Name p4_pu8_dx): "Keine Abweichung unter den gefuehrten Schaltern" (logs/p4_pu8.log:64), dieselben Werte 5000 -> 3000 (:65), 5000 -> 3000 (:159), 15000 -> 9000 (:162), 25 -> 15 (:189), Zaehltakt 600 (:86), SISM-Zeitbasis T = 3000 = 0.0500 s bis 9000 = 0.1500 s (:178-179), SLICE-KADENZ 3000 (:859). Erwartete Unterschiede: "steht fuer 150.00 ms" statt 416.67 (:88-89, B0.1), "BASIS ZUSAETZLICH" 15 statt 18 Zeilen (SISM_AB/T, PERF_AB jetzt gefuehrt). Weil der Faktor bei 4 mm exakt 0,6*1,0 bleibt: forces/cd_facetten bitgleich zu export/p4_pu8 -- als Zusatzbeweis mitnehmen.
4. **3,75 mm p375_b**: Waechter "Keine Abweichung" fuer KRAFT_ZBAND 4, WAKE_ABSTAND 34, WAKE_START_X 332, SPONGE_N 68, SLICE_NEAR_STEPS 5000 (roh) sowie N2F_BAND_N 9 (nicht eindeutig gemeldet), WANDFREI 2, PLATEAU 2, BODEN_EQ_ABSTAND 2; FAR_LX deklariert. SISM-Beleg: "[Nahfeld] SISM-Zeitbasis: T = 3200 Schritte = 0.0500 s, klassisch bis Schritt 9600 = 0.1500 s (dt_f = 15.625 us" (heute logs/p375_a.log:182-183: 3000 = 0.0469 s, 9000 = 0.1406 s); "CFD_SGS_SISM_AB = 15000 -> 9600", "SISM_T = 5000 -> 3200", "SLICE_NEAR_STEPS = 5000 -> 3200", "SAMPLE_EVERY = 25 -> 16", Zaehltakt 640; "SLICE-KADENZ: alle 3200 Near-Steps = 800 Outer = 50.0 ms" unveraendert (logs/p375_a.log:864 -- die Handumrechnung 5333*0,6 traf dieselbe 3200 wie der Automat 5000*0,64). Einzige wirksame Variable gegen p375_a: N2F_BAND_N 9 (die SISM-Rampe endet in beiden vor 201 ms).

## G. Risiken, Nebenwirkungen, Commits

- Kanal, Kugel, facetten_test: Faktor exakt 1,0 -> bitgleich (F1 beweist es fuer die Kugel; Kanal ohne Lauf, weil dx_skal_aus_umgebung dort ohne CFD_DX-Lesung 1,0 liefert).
- fahrzeug/fernfeld (keine Basis): CFD_SAMPLE_EVERY (Defaults 10/100) und CFD_SGS_DIAG_AB werden bei dx != 4 umgerechnet, ungeprueft aber laut. Altfaelle.
- ~40 historische 8-mm-dd-Dateien: Wiederholung bricht am Waechter ab (Soll 15000, Ist 7500). Mit CFD_BASIS=aus wuerden sie STILL doppelt umgerechnet -> Regel ins Protokoll: alte 8-mm-Zeilen nie mit CFD_BASIS=aus fahren.
- Neue 8-mm-Laeufe fahren kuenftig N2F_BAND_N 4 / WANDFREI 1 / PLATEAU 1 / BODEN_EQ_ABSTAND 1 (Laenge bleibt) und sind mit der 8-mm-Historie (Anzahl bleibt) nicht mehr vergleichbar, ausser die Zeile deklariert die alten Werte wie die Regression. Entscheid E3.
- Zeitwaechter feuert jetzt auch bei Vorgabe-u_lat und dx != 4: drei "IN ZEIT"-Warnungen mehr je 8-mm-Log.
- Rundungsgrenzen (6480 meldet "nicht eindeutig"): N 8,53 -> 9 bei 3,75; SAMPLE_EVERY 12,5 bei 8 mm ohne Zeilenwert; WANDFREI 0,5 bei 16 mm; BODEN_EQ_ABSTAND 2 = 7,5 statt 8,0 mm bei 3,75 (deklarieren, nicht wegdefinieren).
- Dokumente nachziehen: setup.cpp:24-26, basis/fahrzeug_dd.basis:6-14 und 87-91 (per NACHGEZOGEN-Vermerk ueberholt), TODO.md:38 (4a), Tagesprotokoll.
- Folgearbeit ausserhalb dieses Plans: U_LAT_VORGABE steht in setup.cpp:32 UND als Literal in lbm.cpp:541-542 -> nach lbm.hpp neben DX_SCHRITT_VORGABE_MM; CFD_SGS_SISM=1, CFD_SAMPLE_EVERY, CFD_ZAEHL_TAKT, CFD_PTRT, CFD_FAC_DETEPS/PINV stehen weiter nicht in der Basis.

**Commits (eine Sache je Commit, in dieser Reihenfolge):**
1. Werkzeuge: basis_aus_lauf.py (EINHEIT vollstaendig, Docstring, --nachziehen), basis_zeile.py (schritte_fein explizit). Kein Laufpfad.
2. Basis maschinell nachgezogen (D) -- vertraeglich mit allen aktuellen Zeilen unter der alten Regel; 8-mm-Zeilen scheitern ab hier laut an N/WANDFREI/PLATEAU/ABSTAND (kein Lauf dazwischen geplant).
3. Zeitwaechter-Millisekunden (B0.1): setup.cpp:146 `/skal` -> `*skal`. Nur Meldung.
4. Schritt-Schalter folgen dx (A) + Waechterregel schritte_fein = Wert bleibt (B) -- MUSS ein Commit sein, beide Haelften einer Regel.
5. Serienzeilen (E): kl_std.txt, p375_b.
6. Abnahmen F1-F4 in dieser Reihenfolge (CPU zuerst, ohne GPU-Budget), Protokoll, TODO 4a auf ERLEDIGT.

## H. Entscheide fuer Heiko
E1 CFD_ZAEHL_TAKT und CFD_SAMPLE_EVERY mit dx skalieren? Empfehlung JA (EIN Faktor fuer jeden Schritt-Schalter, keine Ausnahme; Regressionszeile traegt 200/50). Alternative: beide nur u_lat -- inkonsistent, und p4_pu8 muss ohnehin 15/600 drucken.
E2 Kugel/Kanal ohne dx-Faktor, angesagt. Empfehlung JA (B0.6).
E3 8-mm-Regression deklariert die alten Masse (N=8 ...); neue 8-mm-Laeufe fahren N=4? Oder 8 mm bleibt als "Anzahl bleibt"-Sprosse deklariert?
E4 p375_b als ein Lauf (dx-Fix + BAND_N 9, eine wirksame Variable) -- Empfehlung JA.
E5 CFD_SGS_SISM=1 (und SGS_BAND=0) in die Basis? Ohne SISM=1 sind SISM_AB/T dort halb gedeckt.

## I. Meine Entscheide beim Umsetzen (Heiko hat die Performance-/VRAM-Punkte um 17:3x zur selbständigen Bearbeitung übergeben)
E1 JA. E2 JA. E3: neue 8-mm-Zeilen fahren die Längen (N=4/1/1/1) — das ist Heikos Regel; die Regression deklariert die alten Zahlen. E4 JA.
E5: CFD_SGS_SISM=1 wird mit nachgezogen (reine Ergänzung, kein Wert geändert), SGS_BAND=0 nicht (steht als Standard „aus" in den Zeilen, kein Zeitbezug).

## J. Erratum nach Prüfagent (16.09. 18:10, geprüft 65774bd..077cfca, HEAD f645036)
- §D „mit allen bestehenden Zeilen verträglich" war FALSCH: der Wächter meldet jeden geführten, aber ungesetzten Schalter als FEHLT (print_error).
  Mit `CFD_FAR_LX` in der Basis scheitert jede historische dd-Zeile ohne FAR_LX zuerst daran (F2/F3 erster Anlauf 18:00). Neue Zeilen tragen
  `CFD_FAR_LX=12.2720`; für Altzeilen ist der Notausgang die Deklaration `CFD_FAR_LX=-` (Sentinel „absichtlich ungesetzt").
- §C „16 mm: muss deklariert werden" war FALSCH: bei 16 mm ergibt die Längenregel N=2/WANDFREI=1/PLATEAU=1 → setup.cpp `WANDFREI+PLATEAU >= N−1`
  = print_error, nicht deklarierbar. Auswege: `CFD_N2F_BAND_N>=4` deklariert (Anzahl bleibt) oder `CFD_N2F_BAND=0`. 16-mm-Zeilen sind Altfälle.
- MITTEL offen (Bau nach der Queue): der Wächter meldet „nicht eindeutig" nur bei fehlt/weicht_ab, bei Ist == llround(Soll) schweigt er — bei 3,75 mm
  runden ALLE acht Längenschalter nicht eindeutig (8,53/2,13/2,13/2,13/4,27/34,13/331,7/68,27), ohne Ansage. Fix: bei gleich && unten!=oben ein
  print_info „Rundungswahl akzeptiert". Abnahmekriterium F4 „nicht eindeutig gemeldet" war so nicht erfüllbar.
- NIEDRIG offen (Bau nach der Queue): `dx_skal_setzen` ohne isfinite (CFD_DX=0 → inf passiert die Toleranz), CFD_U_LAT-Meldung mit altem „Alt:"-Satz,
  Zeitwächter-Fußzeile ohne „und dx 4 mm", toter u_lat-Parameter, U_LAT_VORGABE doppelt (setup.cpp/lbm.cpp).
- Werkzeugbefunde behoben in 17c95b5 (igc_offline reicht CFD_OCL_OPTIONS durch; basis_zeile.py llround; EINHEIT +4; --nachziehen --grund).
