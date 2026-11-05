"""The sampling planners with a one-second planning budget.

Reads ``data/comparison/budget`` (``scripts/run_budget_campaigns.py``): the
campaigns of Two-Rooms, Warehouse, procedural and docking run again, once with
a budget of one second and once with the budget of the campaigns of
``data/campaigns/`` (15 s, 5 s for Warehouse), all other settings unchanged;
and the campaigns themselves.

Writes ``tables/budget.md`` (success, mean ± standard deviation of the length
and of the swept area, over the runs that reached the goal) and
``tables/budget_table.tex`` (body of the table: one second against the
original budget, with the relative changes). The swept area is not among the
statistics Bench-MR stores: it is measured here on the trajectories, by the
same code. CPU only.
"""
import gzip
import json
import os

import numpy as np

import study_tools as S
from shfm import DEFAULT_VEHICLE, benchmark as B, metrics as M
from shfm.environments import from_benchmark_run

BUDGET = os.path.join(S.DATA, "comparison", "budget")
SCENES = ("two_rooms", "warehouse", "procedural", "docking")
TEX_SCENES = {"two_rooms": "\\emph{Two-Rooms}", "warehouse": "\\emph{Warehouse}$^{*}$",
              "procedural": "procédural", "docking": "accostage"}
TEX_PLANNERS = [("RRT", "\\acs{RRT}"), ("RRTstar", "\\acs{RRT-star}"),
                ("InformedRRTstar", "\\acs{Informed-RRT-star}"), ("SPARS", "\\acs{SPARS}"),
                ("SST", "\\acs{SST}")]


def statistics(scene):
    """``{(planner, tag): {"ok", "n", "path_length", "swept_area_m2"}}`` for the
    runs at 1 s, at the original budget and in the campaign."""
    files = {}
    for tag in ("1s", "original"):
        f = os.path.join(BUDGET, "%s_%s.json.gz" % (scene, tag))
        files[tag] = json.load(gzip.open(f, "rt"))
    files["campaign"] = B.load_campaign(scene)
    stats = {}
    for tag, data in files.items():
        for run in data["runs"]:
            env_r = None
            for name, plan in run["plans"].items():
                if not plan:
                    continue
                st = plan.get("stats") or {}
                e = stats.setdefault((name, tag), {"ok": 0, "n": 0})
                e["n"] += 1
                if not B.reached_goal(plan):
                    continue
                e["ok"] += 1
                if st.get("path_length") is not None:
                    e.setdefault("path_length", []).append(float(st["path_length"]))
                traj = plan.get("trajectory")
                if traj:
                    if env_r is None:
                        env_r = from_benchmark_run(run, zoom=S.ZOOM[scene], n_theta=2,
                                                   vehicle=DEFAULT_VEHICLE)
                    with M.quiet():
                        e.setdefault("swept_area_m2", []).append(
                            M.swept_area(np.asarray(traj, dtype=float)[:, :3],
                                         env_r.vehicle, env_r.zoom))
    planners = sorted({name for name, _ in stats})
    return stats, planners


def mean_std(values):
    return ("%.1f ± %.1f" % (np.mean(values), np.std(values))) if values else "-"


def tex_body(rows):
    """The table body, from the cells of ``budget.md`` (rounded as printed)."""
    def num(x):
        return x.replace(".", "{,}")

    def ms(cell):
        m, s = [float(v) for v in cell.split("±")]
        return m, "$%s \\pm %s$" % (num("%.1f" % m), num("%.1f" % s))

    def change(e):
        return "$%+d$" % round(e) if round(e) != 0 else "$0$"

    out = []
    for scene in SCENES:
        out.append("    \\midrule")
        for i, (key, label) in enumerate(TEX_PLANNERS):
            r = rows[(scene, key)]
            s1, l1, a1, s0, l0, a0 = r[:6]
            ml1, tl1 = ms(l1); ml0, tl0 = ms(l0)
            ma1, ta1 = ms(a1); ma0, ta0 = ms(a0)
            head = "\\multirow{5}{*}{%s}" % TEX_SCENES[scene] if i == 0 else ""
            out.append("    %s & %s & $%s$ & $%s$ & %s & %s & %s & %s & %s & %s \\\\" % (
                head, label, s1.split("/")[0], s0.split("/")[0], tl1, tl0,
                change(100 * (ml1 - ml0) / ml0), ta1, ta0, change(100 * (ma1 - ma0) / ma0)))
    return "\n".join(out) + "\n"


def main():
    lines, cells = [], {}
    for scene in SCENES:
        stats, planners = statistics(scene)
        for name in planners:
            row = []
            for tag in ("1s", "original", "campaign"):
                e = stats.get((name, tag), {"ok": 0, "n": 0})
                row += ["%d/%d" % (e["ok"], e["n"]), mean_std(e.get("path_length", [])),
                        mean_std(e.get("swept_area_m2", []))]
            lines.append([S.NAMES[scene], name] + row)
            cells[(scene, name)] = row
        S.log("  budget %s" % scene)
    S.write_table("budget.md",
                  "# Campaigns at 1 s against the original budget\n\n"
                  "New runs with the settings of the campaigns of `data/campaigns/` except "
                  "`max_planning_time`. Means ± population standard deviations (ddof = 0, as "
                  "printed in tab. 5.6) over the runs that reached "
                  "the goal. Swept area measured on the trajectories.\n\n"
                  + S.md_table(["scene", "planner", "success 1 s", "length 1 s", "area 1 s",
                                "success original", "length original", "area original",
                                "success campaign", "length campaign", "area campaign"], lines))
    S.write_table("budget_table.tex", tex_body(cells))


if __name__ == "__main__":
    main()
