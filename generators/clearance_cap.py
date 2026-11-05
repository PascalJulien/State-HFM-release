"""Effect of the cap on the obstacle cost.

Reads ``data/sensitivity/clearance_cap`` (``scripts/acquire_sensitivity.py
--part clearance_cap``): the Two-Rooms, Warehouse and procedural problems of
the campaigns, both vehicles, transition costs (2, 3), for eight caps from no
obstacle cost to no cap. Paths cleaned and closed on the start pose
(``study_tools.clean_path``) before they are measured.

Writes ``saturation_metriques`` (length, mean distance to the obstacles and
swept area against the cap, per scene), ``saturation_traj_two_rooms`` (the
Two-Rooms paths for every cap) and ``tables/clearance_cap.md`` (the values,
with those of RRT* and Informed-RRT* in the campaigns). CPU only.
"""
import numpy as np
from matplotlib.ticker import FuncFormatter

import study_tools as S
from study_tools import plt

CAPS = ["no_cost", "0.25m", "0.5m", "L/4", "L/2", "L", "2L", "unsaturated"]
CAP_FR = {"no_cost": "sans coût", "unsaturated": "sans plafond", "0.25m": "0,25 m",
          "0.5m": "0,5 m", "L/4": "L/4", "L/2": "L/2", "L": "L", "2L": "2L"}
COMMA = FuncFormatter(lambda v, _: ("%g" % v).replace(".", ","))
"""Axis labels with a decimal comma."""


def main():
    rows, paths = S.load_part("sensitivity", "clearance_cap")
    data = {}
    for r in rows:
        if r["key"] not in paths or r.get("path_length", "") == "":
            continue
        p, _ = S.clean_path(paths[r["key"]], r["scene"], int(r["run"]), r["method"])
        r = dict(r, **{k: str(v) for k, v in S.measure(
            p, S.scene_env(r["scene"], int(r["run"])),
            S.endpoints(r["scene"], int(r["run"]))[1]).items()})
        data.setdefault((r["scene"], r["cap"], r["method"]), []).append(dict(
            length=S.fnum(r["path_length"]), min_c=S.fnum(r["min_clearing"]),
            mean_c=S.fnum(r["mean_clearing"]), swept=S.fnum(r["swept_area_m2"]),
            aol=S.fnum(r["aol"]), turn=S.total_turn(p), cusps=S.fnum(r["n_cusps"]),
            man=S.frac_man(p)))
    scenes = [s for s in ("two_rooms", "warehouse", "procedural") if any(k[0] == s for k in data)]
    metrics = (("length", "longueur (cases)"), ("mean_c", "distance moyenne aux obstacles (cases)"),
               ("swept", "aire balayée (m²)"))
    fig, axes = plt.subplots(len(metrics), len(scenes), figsize=(6 * len(scenes), 3.6 * len(metrics)),
                             squeeze=False)
    lines = []
    for j, scene in enumerate(scenes):
        for i, (m, yl) in enumerate(metrics):
            ax = axes[i, j]
            for method, mk in (("2WS", "o"), ("4WS", "s")):
                xs, ys, lo, hi = [], [], [], []
                for x, cap in enumerate(CAPS):
                    v = data.get((scene, cap, method))
                    if not v:
                        continue
                    a = np.array([e[m] for e in v])
                    xs.append(x); ys.append(a.mean()); lo.append(a.mean() - a.std()); hi.append(a.mean() + a.std())
                    if i == 0:
                        g = lambda k: np.mean([e[k] for e in v])
                        lines.append([S.NAMES[scene], method, CAP_FR[cap], len(v), "%.1f" % g("length"),
                                      "%.2f" % g("min_c"), "%.2f" % g("mean_c"), "%.1f" % g("swept"),
                                      "%.3f" % g("aol"), "%.2f" % g("turn"), "%.1f" % g("cusps"),
                                      "%.0f %%" % (100 * g("man"))])
                col = S.METHOD_COLOR[method]
                ax.plot(xs, ys, marker=mk, color=col, label="SHFM %s" % method,
                        ms=6 if method == "2WS" else 4, lw=1.5)
                if scene == "procedural":
                    ax.fill_between(xs, lo, hi, color=col, alpha=.08)
            ax.set_xticks(range(len(CAPS)))
            ax.set_xticklabels([CAP_FR[c] for c in CAPS], rotation=30, ha="right")
            ax.set_ylabel(yl, fontsize=9); ax.grid(alpha=0.3)
            ax.yaxis.set_major_formatter(COMMA)
            if i == 0:
                ttl = S.NAMES[scene]
                if scene == "procedural":
                    ttl += " (moyenne sur 20 cartes ; bande : écart-type entre cartes)"
                ax.set_title(ttl, fontsize=10)
            if i == 0:
                ax.legend(fontsize=7)
    fig.tight_layout()
    S.save(fig, "saturation_metriques")

    env = S.scene_env("two_rooms")
    fig, axs = plt.subplots(1, 2, figsize=(15, 7.5))
    cmap = plt.get_cmap("viridis", len(CAPS))
    for ax, method in zip(axs, ("2WS", "4WS")):
        S.draw_map(ax, env.walls, env.zoom)
        ax.set_xlabel("$x$ (cases)"); ax.set_ylabel("$y$ (cases)")
        ax.xaxis.set_major_formatter(COMMA); ax.yaxis.set_major_formatter(COMMA)
        for x, cap in enumerate(CAPS):
            key = "two_rooms__run00__%s__%s" % (cap, method)
            if key in paths:
                q, _ = S.clean_path(paths[key], "two_rooms", 0, method)
                ax.plot(q[:, 0], q[:, 1], color=cmap(x), lw=1.8, label=CAP_FR[cap])
        ax.set_title("SHFM %s" % method, fontsize=11)
        ax.legend(fontsize=8, title="plafond")
    fig.tight_layout()
    S.save(fig, "saturation_traj_two_rooms")

    refs = []
    for scene in scenes:
        for name, v in sorted(S.shipped_reference(scene).items()):
            refs.append([S.NAMES[scene], name, "%.1f" % v["length"], "%.2f" % v["min_c"],
                         "%.2f" % v["mean_c"], "%.1f" % v["swept"]])
    S.write_table("clearance_cap.md",
                  "# Cap on the clearance term of the cost\n\n"
                  "k1 = 2, k2 = 3, start mode free; paths cleaned and closed on the start pose. "
                  "Distances in cells. Total turn: sum of the absolute heading changes along the path.\n\n"
                  + S.md_table(["scene", "vehicle", "cap", "n", "length", "min. distance",
                                "mean distance", "swept area (m²)", "AOL", "total turn (rad)",
                                "cusps", "manoeuvre"], lines)
                  + "\n## RRT* and Informed-RRT* (campaigns, runs that reached the goal)\n\n"
                  + S.md_table(["scene", "planner", "length", "min. distance", "mean distance",
                                "swept area (m²)"], refs))


if __name__ == "__main__":
    main()
