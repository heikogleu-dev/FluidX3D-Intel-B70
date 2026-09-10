#!/usr/bin/env python3
"""Lineare von-Neumann-Stabilitaet des D3Q19-Kollisionsoperators dieses Forks.

    werkzeuge/vonneumann.py                    # Nachrechnung der Tabelle in lbm.cpp:1403-1407
    werkzeuge/vonneumann.py --ptrt             # zusaetzlich P-TRT mit omega_g = 1,0 und 1,9
        --tau 0.5000071   Relaxationszeit tau+ (Vorgabe: der Wert aus lbm.cpp:1403)
                          ACHTUNG, ZWEI BETRIEBSPUNKTE: 0,5000071 ist das FERNFELD bei 16 mm,
                          das 4-mm-NAHFELD hat 0,50002832 (beide stehen in logs/zd4_ohne.log).
                          Fuer Aussagen ueber den 4-mm-Defekt --tau 0.50002832 setzen. Am
                          10.09. kostete diese Verwechslung eine falsche Akkumulationszahl.
        --u 0.075         Gittergeschwindigkeit des Grundzustands
        --richtung x      Richtung des Grundzustands: x, xy oder xyz
        --n 24            Stuetzstellen je k-Achse im Grobgitter
        --fein            nach dem Grobgitter lokal nachoptimieren (langsamer, genauer)

WARUM ES DAS GIBT
  In src/lbm.cpp:1403-1407 steht eine gerechnete Tabelle (max|Eigenwert| und e-Faltung
  fuer Lambda = 3/16, 1/4, SRT und 9,1e-8). Das Skript dazu ist nicht erhalten. Am
  10.09.2026 hat ein Planungsagent die Zahlen NICHT reproduziert -- weder Betrag noch
  Reihenfolge -- und daran haengt die Wahl von omega_g fuer P-TRT. Dieses Skript ist die
  unabhaengige Nachrechnung, mit der die Diskrepanz entschieden wird.

WAS GERECHNET WIRD
  Grundzustand rho0 = 1, u0 = const. Stoerung df ~ exp(i k x). Ein Zeitschritt ist
  G(k) = S(k) C, mit C = linearisierte Kollision und S = diag(exp(-i k c_i)) (Streaming:
  die Amplitude bei x stammt aus x - c_i). Gesucht ist max|Eigenwert| ueber alle k.
  |lambda| > 1 heisst: die Mode waechst, e-Faltung = 1/ln|lambda| Schritte.

  Die Linearisierung der Kollision wird ZWEIMAL gebildet -- analytisch und numerisch
  (zentrale Differenzen auf calculate_f_eq) -- und gegeneinander geprueft. Eine stille
  Ableitungsverwechslung waere sonst nicht zu sehen.

QUELLEN IM CODE
  Gleichgewicht  src/kernel.cpp:1136-1187 (calculate_f_eq, Stoerform: feq - w_i)
  TRT-Kollision  src/kernel.cpp:3453-3486  (wp = w, wm = 1/(Lambda/(1/w-1/2)+1/2))
  Linkreihenfolge src/kernel.cpp:4385      (fzc[19][3])
"""
import sys, argparse, itertools
import numpy as np

# ---- D3Q19 in DER Reihenfolge dieses Forks (src/kernel.cpp:4385 und die feq-Zeilen 1167-1187)
C = np.array([
    ( 0, 0, 0),
    ( 1, 0, 0), (-1, 0, 0),
    ( 0, 1, 0), ( 0,-1, 0),
    ( 0, 0, 1), ( 0, 0,-1),
    ( 1, 1, 0), (-1,-1, 0),
    ( 1, 0, 1), (-1, 0,-1),
    ( 0, 1, 1), ( 0,-1,-1),
    ( 1,-1, 0), (-1, 1, 0),
    ( 1, 0,-1), (-1, 0, 1),
    ( 0, 1,-1), ( 0,-1, 1),
], dtype=float)
W = np.array([1.0/3.0] + [1.0/18.0]*6 + [1.0/36.0]*12)
Q = 19
# Gegenrichtung: im Fork liegt sie immer im Nachbarindex (i ungerade -> i+1), siehe die
# fhb/feb-Schleife in src/kernel.cpp:3470-3476.
GEGEN = np.array([0] + [i+1 if i % 2 == 1 else i-1 for i in range(1, Q)])

# Geistmoden der symmetrischen Unterraeume, korrigierte ganzzahlige Basis
# (UEBERGABE-2026-09-11.md, Abschnitt 7; die Basis des Preprints arXiv:2602.06686 A.7-A.9
# ist fuer D3Q19 nicht orthogonal). Orthogonalitaet und Gram werden unten NACHGEPRUEFT.
G = np.array([
    [ 1,-2,-2,-2,-2,-2,-2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [ 0,-2,-2, 1, 1, 1, 1, 1, 1, 1, 1,-2,-2, 1, 1, 1, 1,-2,-2],
    [ 0, 0, 0,-1,-1, 1, 1, 1, 1,-1,-1, 0, 0, 1, 1,-1,-1, 0, 0],
], dtype=float)


def feq(rho, u):
    """Gleichgewicht wie src/kernel.cpp:1136-1187, ohne die konstante Verschiebung -w_i.

    Die Verschiebung ist eine Konstante und faellt aus jeder Ableitung heraus; sie
    weggelassen zu haben ist deshalb fuer die Linearisierung exakt, nicht naeherungsweise.
    """
    cu = C @ u
    return W * rho * (1.0 + 3.0*cu + 4.5*cu*cu - 1.5*float(u @ u))


def jacobi_analytisch(rho0, u0):
    """J[i,j] = d feq_i / d f_j ueber rho = sum f und j = sum c f."""
    cu0 = C @ u0
    # d feq_i / d rho bei FESTEM u
    dfeq_drho = W * (1.0 + 3.0*cu0 + 4.5*cu0*cu0 - 1.5*float(u0 @ u0))
    # d feq_i / d u_a bei FESTEM rho
    dfeq_du = W[:, None] * rho0 * (3.0*C + 9.0*cu0[:, None]*C - 3.0*u0[None, :])
    # d rho / d f_j = 1 ; d u_a / d f_j = (c_ja - u0_a)/rho0
    du_df = (C - u0[None, :]) / rho0                      # (Q,3), Zeile j
    return dfeq_drho[:, None] * np.ones((1, Q)) + dfeq_du @ du_df.T


def jacobi_numerisch(rho0, u0, h=1e-6):
    """Dieselbe Jacobimatrix ueber zentrale Differenzen auf feq(rho(f), u(f))."""
    f0 = feq(rho0, u0)

    def bild(f):
        rho = f.sum()
        u = (C.T @ f) / rho
        return feq(rho, u)

    J = np.zeros((Q, Q))
    for j in range(Q):
        e = np.zeros(Q); e[j] = h
        J[:, j] = (bild(f0 + e) - bild(f0 - e)) / (2.0*h)
    return J


def projektor_geist():
    """Hermite-Projektor auf die drei Geistmoden.

    FALLE (UEBERGABE-2026-09-11.md, Abschnitt 7): die MOMENTENSUMME ist UNGEWICHTET
    (m = sum_i g_i f_i), die RUECKGABE gewichtet (w_i g_i m / |g|_w^2). Nimmt man auch
    die Summe gewichtet, ist das Ergebnis KEIN Projektor mehr -- ein stiller Halbtreffer.
    """
    P = np.zeros((Q, Q))
    for g in G:
        norm = float(np.sum(W * g * g))
        P += np.outer(W * g, g) / norm
    return P


def pruefe_basis():
    """Orthogonalitaet, Gram und Idempotenz -- ohne diesen Nachweis gilt die Basis nicht."""
    befund = {}
    gram = np.array([[float(np.sum(W * a * b)) for b in G] for a in G])
    befund["gram"] = gram
    befund["ausserdiagonal"] = float(np.max(np.abs(gram - np.diag(np.diag(gram)))))
    # w-Orthogonalitaet gegen Masse, Impuls und die sechs zweiten Momente
    hydro = [np.ones(Q)] + [C[:, a] for a in range(3)]
    hydro += [C[:, a]*C[:, b] for a in range(3) for b in range(a, 3)]
    befund["gegen_hydro"] = float(max(abs(np.sum(W * g * h)) for g in G for h in hydro))
    P = projektor_geist()
    befund["idempotenz"] = float(np.max(np.abs(P @ P - P)))
    return befund, P


def operator(J, wp, wm, Pg=None, omega_g=None):
    """Linearisierte Kollision. Ohne Pg: TRT (wp == wm ist SRT)."""
    I = np.eye(Q)
    Psym = np.zeros((Q, Q))                  # (x_i + x_ibar)/2
    for i in range(Q):
        Psym[i, i] += 0.5
        Psym[i, GEGEN[i]] += 0.5
    Pasym = I - Psym
    Cop = I + wp*(Psym @ (J - I)) + wm*(Pasym @ (J - I))
    if Pg is not None:
        # P-TRT: der Geistanteil des symmetrischen Nichtgleichgewichts wird mit omega_g
        # relaxiert statt mit wp. omega_g = wp gibt exakt TRT zurueck (Selbsttest unten).
        Cop = Cop - (wp - omega_g)*(Pg @ (J - I))
    return Cop


def max_eigenwert(Cop, n, fein):
    """max |Eigenwert| von S(k) C ueber den Wellenzahlwuerfel [0, pi]^3."""
    achse = np.linspace(0.0, np.pi, n)
    best, bestk = 0.0, None
    for k in itertools.product(achse, repeat=3):
        s = np.exp(-1j * (C @ np.array(k)))
        ew = np.abs(np.linalg.eigvals(s[:, None] * Cop))
        m = ew.max()
        if m > best:
            best, bestk = m, np.array(k)
    if fein and bestk is not None:
        schritt = np.pi / (n - 1)
        for _ in range(6):                    # sechs Halbierungen um das Grobgitter-Maximum
            schritt *= 0.5
            for d in itertools.product((-1.0, 0.0, 1.0), repeat=3):
                k = np.clip(bestk + schritt*np.array(d), 0.0, np.pi)
                s = np.exp(-1j * (C @ k))
                m = np.abs(np.linalg.eigvals(s[:, None] * Cop)).max()
                if m > best:
                    best, bestk = m, k
    return best, bestk


def faltung(lam):
    """e-Faltung: nach wievielen Schritten eine FREI wachsende Mode um e zunimmt."""
    return float("inf") if lam <= 1.0 else 1.0/np.log(lam)


def akkumulation(omega):
    """Verstaerkungsfaktor einer STAENDIG NACHGESPEISTEN Mode: 1/(1-|1-omega|).

    ZWEI VERSCHIEDENE ZAHLEN, DIE MAN NICHT VERWECHSELN DARF (Vorpruefung 10.09.2026):
      e-Faltung  ist eine RATE -- sie wirkt im Exponenten, ueber die Lauflaenge.
      Akkumulation ist ein KONSTANTER Faktor -- der Gleichgewichtsstand einer Mode, die
      jeden Schritt neu angeregt wird und mit |1-omega| je Schritt abklingt.
    Ein Gewinn von Faktor A an Akkumulation gegen einen Verlust von d an Wachstumsrate
    rechnet sich nur ueber ln(A)/d Schritte. Danach ist er aufgezehrt. Genau diese
    Verwechslung haette bei P-TRT beinahe zu omega_g = 1,0 als erstem Messarm gefuehrt.
    """
    return float("inf") if abs(1.0-omega) >= 1.0 else 1.0/(1.0-abs(1.0-omega))


def main(argv):
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--tau", type=float, default=0.5000071)
    ap.add_argument("--u", type=float, default=0.075)
    ap.add_argument("--richtung", default="x", choices=("x", "xy", "xyz"))
    ap.add_argument("--n", type=int, default=24)
    ap.add_argument("--fein", action="store_true")
    ap.add_argument("--ptrt", action="store_true")
    a = ap.parse_args(argv)

    richtung = {"x": np.array([1.0, 0, 0]),
                "xy": np.array([1.0, 1.0, 0]),
                "xyz": np.array([1.0, 1.0, 1.0])}[a.richtung]
    u0 = a.u * richtung/np.linalg.norm(richtung)
    rho0 = 1.0
    w = 1.0/a.tau
    nu = (a.tau - 0.5)/3.0

    print(f"D3Q19, tau+ = {a.tau:.7f}  (w = {w:.9f}, nu_gitter = {nu:.6e})")
    print(f"Grundzustand rho0 = 1, |u| = {a.u} in Richtung {a.richtung}, k-Gitter {a.n}^3"
          f"{' + Nachoptimierung' if a.fein else ''}\n")

    Ja = jacobi_analytisch(rho0, u0)
    Jn = jacobi_numerisch(rho0, u0)
    print(f"Jacobimatrix analytisch gegen numerisch: max. Abweichung {np.max(np.abs(Ja-Jn)):.2e}")
    J = Ja
    # Erhaltungssaetze: die Kollision darf Masse und Impuls nicht antasten.
    I = np.eye(Q)
    print(f"Erhaltung Masse   |sum_i (J-I)_ij|      max {np.max(np.abs(np.ones(Q) @ (J-I))):.2e}")
    print(f"Erhaltung Impuls  |sum_i c_i (J-I)_ij|  max {np.max(np.abs(C.T @ (J-I))):.2e}")

    befund, Pg = pruefe_basis()
    print(f"\nGeistbasis: Gram-Diagonale {np.diag(befund['gram'])}, "
          f"ausserdiagonal {befund['ausserdiagonal']:.1e}")
    print(f"            w-orthogonal zu Masse/Impuls/2. Momenten: {befund['gegen_hydro']:.1e}")
    print(f"            Projektor idempotent (|P^2-P|): {befund['idempotenz']:.1e}")

    arme = []
    for name, lam in (("TRT Lambda = 3/16", 0.1875), ("TRT Lambda = 1/4", 0.25),
                      ("TRT Lambda = 9,1e-8", 9.1e-8)):
        wm = 1.0/(lam/(1.0/w - 0.5) + 0.5)
        arme.append((f"{name}  (w- = {wm:.6g}, tau- = {1.0/wm:.6g})", operator(J, w, wm), w, wm))
    arme.append((f"SRT           (w- = w = {w:.6g})", operator(J, w, w), w, w))

    if a.ptrt:
        wm0 = 1.0/(0.1875/(1.0/w - 0.5) + 0.5)
        probe = operator(J, w, wm0, Pg, w)                       # Selbsttest: omega_g = wp
        print(f"\nSelbsttest P-TRT bei omega_g = wp gegen TRT: "
              f"max. Abweichung {np.max(np.abs(probe - operator(J, w, wm0))):.2e}")
        for og in (1.0, 1.9):
            arme.append((f"P-TRT Lambda = 3/16, omega_g = {og}", operator(J, w, wm0, Pg, og), og, wm0))

    print(f"\n{'Arm':<44}{'max|Eigenwert|':>16}{'e-Faltung':>12}{'Akk. ger.':>11}{'Akk. unger.':>13}   k/pi")
    for name, Cop, og_ger, og_ung in arme:
        lam, k = max_eigenwert(Cop, a.n, a.fein)
        f = faltung(lam)
        ftxt = "stabil" if f == float("inf") else f"{f:.0f}"
        ktxt = "-" if k is None else "(" + ", ".join(f"{x/np.pi:.2f}" for x in k) + ")"
        print(f"{name:<44}{lam:>16.6f}{ftxt:>12}{akkumulation(og_ger):>11.0f}{akkumulation(og_ung):>13.0f}   {ktxt}")


if __name__ == "__main__":
    main(sys.argv[1:])
