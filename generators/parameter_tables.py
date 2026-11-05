"""Tables of the parameter study.

Reads the acquisitions of ``data/sensitivity/`` and writes
``tables/parameter_study.md``:

* the resolution study, scene by scene and grid by grid: solve time, peak GPU
  memory (measured in fresh processes), path length and its ratio to the
  finest grid;
* the two settings (2, 3) and (10, 50) on the five Two-Rooms situations,
  solved as the one-parameter sweeps are;
* the one-parameter sweeps: swept area, length, cusps, share of the length
  driven backward and share of the points in a manoeuvring mode;
* the (k1, k2) grid on the situations and on the docking and Narrow-Passage
  problems.

Lengths in grid units. In the resolution study, the unit is that of the fixed
frame of each scene, whatever the number of cells: 25 cm for Two-Rooms,
12.5 cm for Narrow-Passage, 0.5 m for Warehouse. Elsewhere, cells of the grid
the study runs on (12.5 cm for Two-Rooms at 200 cells a side). CPU only.
"""
import csv
import json
import os

import study_tools as S

SENS = os.path.join(S.DATA, "sensitivity")
SITUATIONS = {"S1": "crossing", "S5": "corridor", "R1": "U-turn",
              "R2": "exit facing the wall", "R3": "entry facing the door"}


def f(v, fmt):
    x = S.fnum(v)
    return "-" if x != x else fmt % x


def resolution():
    d = os.path.join(SENS, "resolution")
    rows = list(csv.DictReader(open(os.path.join(d, "sweep_resolution.csv"))))
    mem = {}
    for line in open(os.path.join(d, "memory.jsonl")):
        e = json.loads(line)
        mem[(e["scene"], e["n"])] = e["peak_used_mb"]
    lines = []
    for r in rows:
        finest = [q for q in rows if q["scenario"] == r["scenario"]][-1]
        ratio = S.fnum(r["path_length"]) / S.fnum(finest["path_length"])
        lines.append([r["scenario"], "%sx%sx%s" % (r["Nx"], r["Ny"], r["Ntheta"]), r["npoints"],
                      f(r["solve_s"], "%.2f"), "%.0f" % mem[(r["scenario"], int(r["Nx"]))],
                      f(r["path_length"], "%.1f"), "-" if ratio != ratio else "%.3f" % ratio,
                      "yes" if r["degenerate"] == "1" else "no"])
    return ("## Grid resolution\n\n"
            "Time of the solve on the machine that measured it; memory: peak allocated by the "
            "solve, in 10^6 bytes. A degenerate path ends further than a vehicle length from "
            "its start or goal.\n\n"
            + S.md_table(["scene", "grid", "points", "time (s)", "memory (MB)", "length",
                          "length / finest", "degenerate"], lines))


def recommended():
    rows = {r["key"]: r for r in csv.DictReader(open(os.path.join(SENS, "recommended", "rows.csv")))}
    lines = []
    for tag, name in SITUATIONS.items():
        a, b = rows["%s__k1_10__k2_50" % tag], rows["%s__k1_2__k2_3" % tag]
        A, Bv = S.fnum(a["swept_area_m2"]), S.fnum(b["swept_area_m2"])
        lines.append(["%s (%s)" % (tag, name), "%.1f" % A, "%.1f" % Bv, "%+.1f %%" % (100 * (Bv - A) / A),
                      a["n_cusps"], b["n_cusps"]])
    return ("## (2, 3) against (10, 50) on the situations\n\n"
            "Swept area (m²) and cusps; clearance capped at the vehicle length, start mode "
            "free.\n\n"
            + S.md_table(["situation", "area (10, 50)", "area (2, 3)", "change",
                          "cusps (10, 50)", "cusps (2, 3)"], lines))


def one_parameter():
    rows = list(csv.DictReader(open(os.path.join(SENS, "one_parameter", "swept_1d_two_rooms.csv"))))
    lines = []
    for tag in SITUATIONS:
        for r in rows:
            if r["scenario"] != tag:
                continue
            lines.append([tag, r["param"], "%g" % S.fnum(r["value"]), f(r["swept_area_m2"], "%.1f"),
                          f(r["path_length"], "%.1f"), r["n_cusps"],
                          f(r["frac_backward_length"], "%.2f"), f(r["frac_man"], "%.2f"),
                          f(r["reach_error"], "%.1f")])
    return ("## One-parameter sweeps\n\n"
            "One parameter moved at a time around k1 = 10, k2 = 50, nu_FWD = 3, nu_2WS = 1.5, "
            "nu_4WS = 0.5, gamma_rev = 0.5 (`nu_reverse`). Backward: share of the length driven "
            "backward; manoeuvre: share of the points in a manoeuvring mode; reach error: "
            "distance from the ends of the path to the poses asked for, in cells.\n\n"
            + S.md_table(["situation", "parameter", "value", "swept area (m²)", "length",
                          "cusps", "backward", "manoeuvre", "reach error"], lines))


def k1_k2_map():
    rows = list(csv.DictReader(open(os.path.join(SENS, "k1_k2_map", "rows.csv"))))
    lines = []
    for r in rows:
        man = S.fnum(r.get("frac_man_fwd")) + S.fnum(r.get("frac_man_bwd"))
        lines.append([r["scene"], "%g" % S.fnum(r["k1"]), "%g" % S.fnum(r["k2"]),
                      f(r.get("swept_area_m2"), "%.1f"), f(r.get("path_length"), "%.1f"),
                      r.get("n_cusps", ""), "-" if man != man else "%.0f %%" % (100 * man)])
    return ("## Grid of transition costs\n\n"
            "4WS, reference speed coefficients, clearance capped at the vehicle length, "
            "start mode free. Situations at 200 "
            "cells a side; docking and Narrow-Passage on their campaign grids.\n\n"
            + S.md_table(["scene", "k1", "k2", "swept area (m²)", "length", "cusps",
                          "manoeuvre"], lines))


def main():
    S.write_table("parameter_study.md",
                  "# Parameter study\n\n" + "\n".join((resolution(), recommended(),
                                                         one_parameter(), k1_k2_map())))


if __name__ == "__main__":
    main()
