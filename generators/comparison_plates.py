"""Comparison with the sampling planners: trajectory and metric plates, tables.

Reads ``data/comparison/campaigns`` (``scripts/acquire_comparison.py --part
campaigns``: our paths at k1 = 2, k2 = 3, both vehicles, on every problem of
the campaigns), ``data/comparison/reference_costs`` (docking and
Narrow-Passage at k1 = 10, k2 = 50) and the campaigns of ``data/campaigns/``
(the sampling planners). Our paths are cleaned of the extraction chatter and
their start is closed on the requested pose (``study_tools.clean_path``)
before they are drawn or measured. On the fixed scenes the one path stands
for the twenty runs.

Figures (Bench-MR plotting, through :mod:`shfm.plates`):

* ``env1_traj`` (also as PNG at 200 dpi), ``runs_warehouse`` (idem),
  ``run_0123``: trajectories on Two-Rooms, Warehouse and four procedural maps;
* ``env1_metrics``, ``warehouse_metrics``, ``procedural_metrics``: metric plates,
  length, AOL, maximum curvature and minimum clearing above, the AHE centred
  below;
* ``creneau_2ws``, ``creneau_4WS``, ``creneau_2ws_nl``, ``creneau_4ws_nl``:
  docking, each vehicle alone, with and without legend;
* ``accostage_sota``: docking, two runs, ours beside RRT* and Informed-RRT*;
* ``2ws_only_shortcuts``, ``4ws_only_shortcuts``: Narrow-Passage;
* ``creneau_4ws_k1050_nl``, ``4ws_only_shortcuts_k1050``: the same at (10, 50).

Tables, under ``tables/``: ``comparison.md`` (our side, every scene),
``comparison_summary.tex`` (body of the summary table: ours against RRT* and
Informed-RRT* on three scenes), ``docking.md`` (the docking metrics of every
planner), ``reference_costs.md`` (ours at (10, 50)) and ``approach.md`` (the
final approach of every path: AHE over the last vehicle length, heading gap
over the last 1.2 m and 0.4 m).

The AHE (Approach Heading Error, :func:`shfm.metrics.approach_heading_error`)
is measured over the length of the vehicle, on the runs that reached the goal.

The PNG copies need ``pdftoppm`` (poppler). CPU only.
"""
import os
import shutil
import subprocess

import numpy as np

import study_tools as S
from shfm import benchmark as B, metrics as M, plates as PL
from shfm import DEFAULT_VEHICLE
from shfm.postprocess import reversals

SCENES = ("two_rooms", "warehouse", "procedural", "docking", "narrow_passage")
PLANNERS = ("RRT", "RRTstar", "InformedRRTstar", "SPARS", "SST")

TRAJ_PLATES = {
    "two_rooms":  dict(name="env1_traj", zoom=8.0, every_nth=60, run_id="0,1,2,3",
                       per_line=2, fig_w=8.0, fig_h=7.0, titles=True, png=True),
    "warehouse":  dict(name="runs_warehouse", zoom=2.0, every_nth=50, run_id="0,1",
                       per_line=2, fig_w=6.0, fig_h=6.0, titles=False, png=True),
    "procedural": dict(name="run_0123", zoom=2.0, every_nth=50, run_id="0,1,2,3",
                       per_line=2, fig_w=6.0, fig_h=6.0, titles=False, png=False),
}
METRIC_PLATES = {"two_rooms": "env1_metrics", "warehouse": "warehouse_metrics",
                 "procedural": "procedural_metrics"}
AHE = "approach_heading_error"
WINDOWS = (1.2, 0.4)
"""Shorter approach windows, in metres: the length of the forks, then the last
0.4 m."""


def out(name):
    return os.path.join(S.FIGURES, name)


def keep_nolegend(stem, target):
    """Keep the copy without legend under ``target``, drop the one with it."""
    os.replace(out(stem + "_nolegend.pdf"), out(target + ".pdf"))
    os.remove(out(stem + ".pdf"))


def to_png(name, dpi=200):
    """PNG copy of ``name.pdf``, rendered by pdftoppm."""
    if shutil.which("pdftoppm") is None:
        S.log("  pdftoppm not found: %s.png not written" % name)
        return
    subprocess.run(["pdftoppm", "-png", "-r", str(dpi), "-singlefile",
                    out(name + ".pdf"), out(name)], check=True)


def our_paths():
    """``{(scene, run, method): (row, cleaned path, splice info)}``."""
    rows, paths = S.load_part("comparison", "campaigns")
    store = {}
    for r in rows:
        if r["key"] not in paths:
            continue
        run = int(r["run"])
        q, info = S.clean_path(paths[r["key"]], r["scene"], run, r["method"])
        store[(r["scene"], run, r["method"])] = (r, q, info)
    return store


def summary_table(store):
    """Our side, every scene: means over the problems of each campaign."""
    lines, gaps, summary = [], [], {}
    for scene in SCENES:
        for method in ("2WS", "4WS"):
            items = sorted((k, v) for k, v in store.items() if k[0] == scene and k[2] == method)
            if not items:
                continue
            ms = []
            for (sc, run, _), (row, q, info) in items:
                env = S.scene_env(sc, run)
                goal_q = S.endpoints(sc, run)[1]
                m = S.measure(q, env, goal_q)
                m["man"] = S.frac_man(q)
                m["ahe"] = M.approach_heading_error(q, goal_q[2],
                                                    env.vehicle.length * env.zoom)
                ms.append(m)
                gaps.append([S.NAMES[sc], method, str(run), "%.2f" % info["gap"],
                             "%.1f" % np.degrees(info["gap_theta"]),
                             "yes" if info["joined"] is not None else "no"])
            g = lambda k: float(np.mean([m[k] for m in ms]))
            summary[(scene, method)] = {k: g(k) for k in ms[0]}
            lines.append([S.NAMES[scene], method, str(len(ms)), "%.1f" % g("path_length"),
                          "%.1f" % g("swept_area_m2"), "%.3f" % g("aol"), "%.2f" % g("max_curvature"),
                          "%.2f" % g("normalized_curvature"), "%.1f" % g("n_cusps"),
                          "%.2f" % g("min_clearing"), "%.1f" % g("mean_clearing"),
                          "%.0f %%" % (100 * g("man")), "%.3f" % g("ahe")])
    joined = sum(1 for r in gaps if r[-1] == "yes")
    pieces = []
    for method in ("2WS", "4WS"):
        q = store[("two_rooms", 0, method)][1]
        cuts = [0] + [c + 1 for c in reversals(q)] + [len(q) - 1]
        arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(q[:, :2], axis=0), axis=1))]
        pieces.append([method, " ; ".join("%.2f" % ((arc[b] - arc[a]) / S.ZOOM["two_rooms"])
                                         for a, b in zip(cuts[:-1], cuts[1:]))])
    S.write_table("comparison.md",
                  "# Our paths at k1 = 2, k2 = 3\n\n"
                  "Start mode free; paths cleaned of the extraction chatter, then the start "
                  "closed on the requested pose. Means over the problems of each campaign; "
                  "distances in cells.\n\n"
                  + S.md_table(["scene", "vehicle", "n", "length", "swept area (m²)", "AOL",
                                "max. curvature", "norm. curvature", "cusps",
                                "min. clearing", "mean clearing", "manoeuvre", "AHE (rad)"], lines)
                  + "\n## Start closed on the requested pose\n\n%d paths of %d joined.\n\n"
                  % (joined, len(gaps))
                  + S.md_table(["scene", "vehicle", "run", "gap before (cells)",
                                "gap before (°)", "joined"], gaps)
                  + "\n## Two-Rooms: length of the stretches between cusps (m)\n\n"
                  + S.md_table(["vehicle", "stretches, start to goal"], pieces))
    return summary


def plates(store):
    for scene in ("two_rooms", "warehouse", "procedural", "docking"):
        data = B.load_campaign(scene)
        n = len(data["runs"])
        distinct = {}
        for i in range(n):
            s0, g0 = B.endpoints_of(data["runs"][i])
            distinct.setdefault((data["runs"][i]["environment"]["map"], tuple(s0), tuple(g0)), i)
        ours = {"SHFM_2WS": [], "SHFM_4WS": []}
        for i in range(n):
            s0, g0 = B.endpoints_of(data["runs"][i])
            src = distinct[(data["runs"][i]["environment"]["map"], tuple(s0), tuple(g0))]
            for name, method in (("SHFM_2WS", "2WS"), ("SHFM_4WS", "4WS")):
                ours[name].append(store[(scene, src, method)][1])
        o3 = {k: [p[:, :3] for p in v] for k, v in ours.items()}
        o3l = [o3["SHFM_2WS"], o3["SHFM_4WS"]]
        jf = B.campaign_file(scene)
        if scene in METRIC_PLATES:
            envs = [S.scene_env(scene, i) for i in range(n)]
            goals = [S.endpoints(scene, i)[1] for i in range(n)]
            raw = {k: [q[::-1] for q in v] for k, v in o3.items()}
            stats = PL.custom_stats(o3, envs, goals, raw=raw)
            name = METRIC_PLATES[scene]
            PL.metric_plate(jf, stats, zoom=S.ZOOM[scene], silence=True,
                            save_file=out(name + ".pdf"))
            os.remove(out(name + "_nolegend.pdf"))
        if scene in TRAJ_PLATES:
            R = TRAJ_PLATES[scene]
            PL.trajectories(jf, o3l, every_nth=R["every_nth"], run_id=R["run_id"],
                            max_plots_per_line=R["per_line"], set_title=R["titles"],
                            fig_width=R["fig_w"], fig_height=R["fig_h"], zoom=R["zoom"],
                            save_file=out(R["name"] + ".pdf"))
            os.remove(out(R["name"] + "_nolegend.pdf"))
            if R["png"]:
                to_png(R["name"])
        if scene == "docking":
            base = "rrt,rrtstar,informedrrtstar,sst,spars"
            for label, hidden in (("2ws", "shfm4ws"), ("4WS", "shfm2ws")):
                PL.trajectories(jf, o3l, every_nth=60, run_id="0",
                                ignore_planners=base + "," + hidden,
                                save_file=out("creneau_%s.pdf" % label))
                os.replace(out("creneau_%s_nolegend.pdf" % label),
                           out("creneau_%s_nl.pdf" % label.lower()))
            PL.trajectories(jf, o3l, every_nth=200, run_id="0,1", max_plots_per_line=2,
                            ignore_planners="rrt,sst,spars,shfm2ws", num_colors=5,
                            save_file=out("accostage_sota.pdf"))
            os.remove(out("accostage_sota_nolegend.pdf"))
        S.log("  plates %s" % scene)
    jf = B.campaign_file("narrow_passage")
    o3l = [[store[("narrow_passage", 0, "2WS")][1][:, :3]],
           [store[("narrow_passage", 0, "4WS")][1][:, :3]]]
    base = "rrt,sst,rrtstar,informedrrtstar,spars"
    for label, hidden in (("2ws", "shfm4ws"), ("4ws", "shfm2ws")):
        PL.trajectories(jf, o3l, every_nth=50, run_id="0",
                        ignore_planners=base + "," + hidden,
                        save_file=out("%s_only_shortcuts_legend.pdf" % label))
        keep_nolegend("%s_only_shortcuts_legend" % label, "%s_only_shortcuts" % label)
    S.log("  plates narrow_passage")


def reference_costs():
    """Docking and Narrow-Passage at (10, 50), drawn as the plates at (2, 3)."""
    rows, paths = S.load_part("comparison", "reference_costs")
    lines = []
    for scene, every, stem in (("docking", 60, "creneau_4ws_k1050"),
                               ("narrow_passage", 50, "4ws_only_shortcuts_k1050")):
        ours = {}
        for method in ("2WS", "4WS"):
            q, _ = S.clean_path(paths["%s__run00__%s" % (scene, method)], scene, 0, method)
            ours[method] = q
            m = S.measure(q, S.scene_env(scene), S.endpoints(scene)[1])
            lines.append([S.NAMES[scene], method, "%.1f" % m["path_length"],
                          "%.1f" % m["swept_area_m2"], "%.0f" % m["n_cusps"],
                          "%.0f %%" % (100 * S.frac_man(q))])
        jf = B.campaign_file(scene)
        n = len(B.load_campaign(scene)["runs"])
        o3l = [[ours["2WS"][:, :3]] * n, [ours["4WS"][:, :3]] * n]
        PL.trajectories(jf, o3l, every_nth=every, run_id="0",
                        ignore_planners="rrt,sst,rrtstar,informedrrtstar,spars,shfm2ws",
                        save_file=out(stem + "_legend.pdf"))
        keep_nolegend(stem + "_legend", stem if scene == "narrow_passage" else stem + "_nl")
    S.write_table("reference_costs.md",
                  "# Docking and Narrow-Passage at (k1, k2) = (10, 50)\n\n"
                  "Start mode free; paths cleaned and closed on the start pose.\n\n"
                  + S.md_table(["scene", "vehicle", "length", "swept area (m²)", "cusps",
                                "manoeuvre"], lines))


def comparison_summary(summary):
    """Body of the summary table: ours (2WS / 4WS) against RRT* / Informed-RRT*."""
    refs = {sc: S.shipped_reference(sc) for sc in ("two_rooms", "warehouse", "procedural")}
    aol, ahe = {}, {}
    for sc in refs:
        for run in B.load_campaign(sc)["runs"]:
            for name in ("RRTstar", "InformedRRTstar"):
                plan = run["plans"].get(name)
                if plan and B.reached_goal(plan):
                    aol.setdefault((sc, name), []).append(plan["stats"]["aol"])
        for name, st in PL.campaign_ahe(B.campaign_file(sc), S.ZOOM[sc]).items():
            ahe[(sc, name)] = st[AHE]

    def fr(x, nd):
        return ("%%.%df" % nd % x).replace(".", "{,}")
    rows = []
    for key, label, nd in (("path_length", "Path length (cases)", 0),
                           ("min_clearing", "Min. clearance (cases)", 1),
                           ("mean_clearing", "Mean clearance (cases)", 1),
                           ("aol", "AOL (rad/case)", 3),
                           ("swept_area_m2", "Swept area (m$^2$)", 1),
                           ("ahe", "AHE (rad)", 2)):
        cells = [label]
        for sc in ("two_rooms", "warehouse", "procedural"):
            a = summary[(sc, "2WS")][key]
            b = summary[(sc, "4WS")][key]
            cells.append("$%s$ / $%s$" % (fr(a, nd), fr(b, nd)))
            if key in ("aol", "ahe"):
                src = aol if key == "aol" else ahe
                r1, r2 = (np.mean(src[(sc, n)]) for n in ("RRTstar", "InformedRRTstar"))
            else:
                rk = {"path_length": "length", "min_clearing": "min_c",
                      "mean_clearing": "mean_c", "swept_area_m2": "swept"}[key]
                r1, r2 = refs[sc]["RRTstar"][rk], refs[sc]["InformedRRTstar"][rk]
            cells.append("$%s$ / $%s$" % (fr(r1, nd), fr(r2, nd)))
        rows.append("    " + " & ".join(cells) + r" \\")
    S.write_table("comparison_summary.tex",
                  "% Ours at k1 = 2, k2 = 3 (2WS / 4WS); RRT* / Informed-RRT*, means over\n"
                  "% the runs that reached the goal. Columns: Two-Rooms, Warehouse, procedural.\n"
                  + "\n".join(rows) + "\n")


DOCKING_COLS = (("path_length", "length (cells)"),
                ("normalized_curvature", "norm. curvature"),
                ("max_curvature", "max. curvature (1/cell)"),
                ("min_clearing_distance", "min. clearing (cells)"),
                (AHE, "AHE (rad)"))


def docking_table(summary):
    """Docking metrics of every planner: mean ± standard deviation over the
    twenty runs, all runs counted, except the AHE, over the runs that reached
    the goal. The normalized curvature of the sampling planners is measured
    again on their trajectories: the value stored in the campaigns carries the
    error described in ``external/NOTICE.md``."""
    data = B.load_campaign("docking")
    vals = PL.campaign_ahe(B.campaign_file("docking"), S.ZOOM["docking"])
    for run in data["runs"]:
        for name, plan in run["plans"].items():
            if not plan:
                continue
            st = dict(plan.get("stats") or {})
            traj = plan.get("trajectory")
            if traj:
                with M.quiet():
                    st["normalized_curvature"] = M.normalized_curvature(
                        np.asarray(traj, dtype=float)[:, :3])
            for key, _ in DOCKING_COLS:
                if st.get(key) is not None:
                    vals.setdefault(name, {}).setdefault(key, []).append(float(st[key]))
    lines = []
    for name in PLANNERS:
        v = vals.get(name, {})
        cells = [name]
        for k, _ in DOCKING_COLS:
            # The AHE spread is the population one (ddof = 0), as printed in the thesis.
            ddof = 0 if k == AHE else 1
            cells.append("%.3f ± %.3f" % (np.mean(v[k]), np.std(v[k], ddof=ddof))
                         if v.get(k) else "-")
        lines.append(cells)
    ours = {"path_length": "path_length", "normalized_curvature": "normalized_curvature",
            "max_curvature": "max_curvature", "min_clearing_distance": "min_clearing",
            AHE: "ahe"}
    for method in ("2WS", "4WS"):
        s = summary[("docking", method)]
        lines.append(["SHFM %s" % method] + ["%.3f" % s[ours[k]] for k, _ in DOCKING_COLS])
    reached = ", ".join("%s %d" % (name, len(vals.get(name, {}).get(AHE, [])))
                        for name in PLANNERS)
    S.write_table("docking.md",
                  "# Docking: metrics of every planner\n\n"
                  "Sampling planners: mean ± sample standard deviation over the twenty runs of "
                  "the campaign, all runs counted; normalized curvature measured again on the "
                  "trajectories. AHE: over the runs that reached the goal (%s), mean ± "
                  "population standard deviation. Ours: k1 = 2, k2 = 3, deterministic, "
                  "one path.\n\n" % reached
                  + S.md_table(["planner"] + [c for _, c in DOCKING_COLS], lines))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def approach(q, goal, zoom):
    """The final approach of a path, over the last vehicle length:

    * heading: the AHE, mean |theta - theta_goal| (rad), weighted by step
      length (:func:`shfm.metrics.approach_heading_error`);
    * lateral: mean distance to the axis of the goal pose (m);
    * spacing: mean step over the approach / mean step over the path;
    * largest step / mean step, over the whole path;
    * ``cap_<w>``: the heading gap over the last ``w`` metres, for each of
      :data:`WINDOWS`.
    """
    heading = {"cap_%g" % w: M.approach_heading_error(q, goal[2], w * zoom)
               for w in WINDOWS}
    heading["cap"] = M.approach_heading_error(q, goal[2], DEFAULT_VEHICLE.length * zoom)
    q = np.asarray(q, float)[:, :3]
    step = np.linalg.norm(np.diff(q[:, :2], axis=0), axis=1)
    keep = step > 1e-9
    q = np.vstack([q[:1], q[1:][keep]]); step = step[keep]
    s = np.r_[0, np.cumsum(step)]
    Lu = DEFAULT_VEHICLE.length * zoom
    ia = np.where(s >= s[-1] - Lu)[0]
    w = np.r_[step, 0][ia]
    w = w if w.sum() > 0 else np.ones_like(w)
    dth = np.abs(wrap(q[ia, 2] - goal[2]))
    u = np.array([np.cos(goal[2]), np.sin(goal[2])])
    lat = np.abs((q[ia, 0] - goal[0]) * (-u[1]) + (q[ia, 1] - goal[1]) * u[0]) / zoom
    st_a = step[ia[:-1]] if len(ia) > 1 else step[-1:]
    return dict(lat=float(np.average(lat, weights=w)), cap_max=float(dth.max()),
                esp=float(st_a.mean() / step.mean()), pas_max=float(step.max() / step.mean()),
                fin=float(np.hypot(q[-1, 0] - goal[0], q[-1, 1] - goal[1]) / zoom),
                fin_cap=float(abs(wrap(q[-1, 2] - goal[2]))), **heading)


def approach_table(store):
    out_ = []
    for scene in ("two_rooms", "warehouse", "procedural", "docking"):
        zoom = S.ZOOM[scene]
        for i, run in enumerate(B.load_campaign(scene)["runs"]):
            g = run["settings"]["env"]["goal"]
            goal = (g["x"], g["y"], g["theta"])
            for name in PLANNERS:
                plan = run["plans"].get(name)
                if not plan or not B.reached_goal(plan) or not plan.get("trajectory"):
                    continue
                r = approach(plan["trajectory"], goal, zoom)
                r.update(scene=scene, method=name)
                out_.append(r)
            for method in ("2WS", "4WS"):
                if (scene, i, method) in store:
                    r = approach(store[(scene, i, method)][1], goal, zoom)
                    r.update(scene=scene, method="SHFM_" + method)
                    out_.append(r)
    lines = []
    for scene in ("two_rooms", "warehouse", "procedural", "docking"):
        for method in ("SHFM_2WS", "SHFM_4WS") + PLANNERS:
            rr = [r for r in out_ if r["scene"] == scene and r["method"] == method]
            if not rr:
                continue
            f = lambda k: np.mean([r[k] for r in rr])
            sd = lambda k: np.std([r[k] for r in rr])
            lines.append([S.NAMES[scene], method, len(rr), "%.3f ± %.3f" % (f("cap"), sd("cap")),
                          *("%.3f" % f("cap_%g" % w) for w in WINDOWS),
                          "%.3f" % f("cap_max"), "%.3f ± %.3f" % (f("lat"), sd("lat")),
                          "%.2f ± %.2f" % (f("esp"), sd("esp")), "%.1f" % f("pas_max"),
                          "%.3f" % f("fin"), "%.4f" % f("fin_cap")])
    S.write_table("approach.md",
                  "# Final approach over the last vehicle length (3.65 m)\n\n"
                  "Means over the runs that reached the goal; ours at k1 = 2, k2 = 3, paths "
                  "cleaned and closed on the start pose. AHE: mean gap to the goal heading "
                  "(rad), weighted by step length, ± population standard deviation; the same "
                  "gap over the last 1.2 m (fork length) and 0.4 m; lateral: mean distance to "
                  "the axis of the goal pose (m); spacing: mean step of the approach / mean "
                  "step of the path.\n\n"
                  + S.md_table(["scene", "method", "n", "AHE (rad)", "last 1.2 m (rad)",
                                "last 0.4 m (rad)", "max. heading gap (rad)",
                                "lateral (m)", "approach spacing", "max. step / mean step",
                                "final gap (m)", "final heading (rad)"], lines))


def main():
    store = our_paths()
    summary = summary_table(store)
    plates(store)
    reference_costs()
    comparison_summary(summary)
    docking_table(summary)
    approach_table(store)


if __name__ == "__main__":
    main()
