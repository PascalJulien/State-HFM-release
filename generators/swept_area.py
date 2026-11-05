"""Floor area swept by the vehicle, every planner, on three scenes.

The sampling planners are measured on the trajectories of their campaigns
(``data/sensitivity/benchmr_swept.csv``, ``scripts/acquire_sensitivity.py
--part reference_swept``); ours on the paths of ``data/comparison/campaigns``
(k1 = 2, k2 = 3), cleaned and closed on the start pose, by the same Bench-MR
metric. Our planner is deterministic: on a fixed scene its single value is
repeated as many times as the campaign has runs, so that it is drawn as a flat
violin; on the procedural maps each map gives its own value.

Writes ``violin_swept_env1``, ``violin_swept_warehouse``,
``violin_swept_procedural`` and ``tables/swept_area.md`` (means and medians).
CPU only.
"""
import collections
import csv
import os

import numpy as np

import study_tools as S
from study_tools import plt
from shfm import benchmark as B

S.bench_python()
from color import get_colors                                        # noqa: E402
from utils import convert_planner_name                              # noqa: E402

SCENES = {"env1": "two_rooms", "warehouse": "warehouse", "procedural": "procedural"}
SHFM_COL = {"2WS": (0, 0, 1, 1), "4WS": (1, 0, 0, 1)}
"""Our two vehicles in blue and red, as in the plates of Bench-MR."""


def valid(vals):
    """Finite areas above zero."""
    out = []
    for v in vals:
        if v in ("", "nan", None):
            continue
        try:
            f = float(v)
        except (ValueError, TypeError):
            continue
        if np.isfinite(f) and f > 0:
            out.append(f)
    return out


def ensure_valid_violin(arr):
    """violinplot needs at least two values; otherwise the category stays empty."""
    if not arr or len(arr) == 1:
        return [float("nan"), float("nan")]
    return arr


def load_planners():
    d = collections.defaultdict(lambda: collections.defaultdict(list))
    for r in csv.DictReader(open(os.path.join(S.DATA, "sensitivity", "benchmr_swept.csv"))):
        d[r["env"]][r["planner"]].append(r["swept_area_m2"])
    return d


def our_areas():
    """``{env: {method: {run: area}}}`` and the number of values per method."""
    rows, paths = S.load_part("comparison", "campaigns")
    keys = {r["key"] for r in rows}
    d = collections.defaultdict(lambda: collections.defaultdict(dict))
    raw = collections.defaultdict(lambda: collections.defaultdict(int))
    for env, scene in SCENES.items():
        data = B.load_campaign(scene)
        fixed = len({r["environment"]["map"] for r in data["runs"]}) == 1
        for run in range(len(data["runs"])):
            if fixed and run > 0:
                continue
            s0, g0 = B.endpoints_of(data["runs"][run])
            mp = data["runs"][run]["environment"]["map"]
            src = run
            for j in range(run + 1):
                s2, g2 = B.endpoints_of(data["runs"][j])
                if data["runs"][j]["environment"]["map"] == mp and tuple(s2) == tuple(s0) \
                        and tuple(g2) == tuple(g0):
                    src = j
                    break
            for method in ("4WS", "2WS"):
                key = "%s__run%02d__%s" % (scene, src, method)
                if key not in keys:
                    continue
                q, _ = S.clean_path(paths[key], scene, src, method)
                m = S.measure(q, S.scene_env(scene, src), S.endpoints(scene, src)[1])
                d[env][method][run] = str(m["swept_area_m2"])
                raw[env][method] += 1
    return d, raw


def plot_env(env, planner_vals, shfm_vals, n_raw):
    planners = sorted(planner_vals.keys(), key=convert_planner_name)
    n_dup = max((len(v) for v in planner_vals.values()), default=20) or 20

    notes = []
    shfm_series = {}
    for m in ("2WS", "4WS"):
        vals = valid(shfm_vals.get(m, {}).values())
        if len(vals) == 0:
            shfm_series[m] = None
            notes.append("SHFM-%s degenere" % m)
        elif len(vals) == 1:
            shfm_series[m] = [vals[0]] * n_dup
        else:
            shfm_series[m] = vals
            nd = n_raw.get(m, 0) - len(vals)
            notes.append("SHFM-%s : %d/%d cartes valides%s"
                         % (m, len(vals), n_raw.get(m, 0),
                            (" (%d degen.)" % nd) if nd else ""))

    cats = list(planners) + ["SHFM 2WS", "SHFM 4WS"]
    series = [planner_vals[p] for p in planners] + [shfm_series["2WS"], shfm_series["4WS"]]
    # The colours of the Bench-MR plates: get_colors samples tab10 according to
    # the number of categories, the last two forced to blue and red.
    colors = list(get_colors(num_colors=len(planners) + 2, color_map_name="tab10"))
    cat_colors = colors[:len(planners)] + [SHFM_COL["2WS"], SHFM_COL["4WS"]]

    plt.figure(figsize=(11, 6))
    ticks = np.arange(len(cats)) + 0.5
    plt.grid(); plt.gca().set_axisbelow(True)
    violins = [ensure_valid_violin(s) for s in series]
    vs = plt.violinplot(violins, ticks, points=50, widths=0.8,
                        showmeans=True, showextrema=False, showmedians=True)
    for i, b in enumerate(vs["bodies"]):
        b.set_facecolor(cat_colors[i]); b.set_edgecolor((0, 0, 0, .6))
        b.set_alpha(.85)
    for pn in ("cmeans", "cmedians"):
        vs[pn].set_edgecolor("black")
    vs["cmeans"].set_edgecolor("green")
    plt.plot([np.nan], [np.nan], color="green", label="Moyenne")
    plt.plot([np.nan], [np.nan], color="black", label="Mediane")
    plt.legend(fontsize=11, loc="best")

    labels = [convert_planner_name(p) for p in planners] + ["SHFM 2WS", "SHFM 4WS"]
    plt.xticks(ticks, labels, rotation=0, fontsize=12)
    plt.gca().set_xlim([0, len(cats)])
    plt.ylabel(r"Aire balayee (m$^2$)", fontsize=15)
    plt.title("Aire balayee — %s" % env, fontsize=16, pad=12)
    if notes:
        plt.gcf().text(0.012, -0.02, "   |   ".join(notes), fontsize=9, color="0.3")
    plt.tight_layout()
    os.makedirs(S.FIGURES, exist_ok=True)
    for ext in ("pdf", "png"):
        plt.savefig(os.path.join(S.FIGURES, "violin_swept_%s.%s" % (env, ext)),
                    dpi=200, bbox_inches="tight")
    plt.close("all")
    S.log("  violin_swept_%s" % env)


def main():
    P = load_planners()
    ours, raw = our_areas()
    lines = []
    for env in [e for e in P if P[e]]:
        planner_vals = {k: valid(v) for k, v in P[env].items()}
        plot_env(env, planner_vals, ours.get(env, {}), raw.get(env, {}))
        for p in sorted(planner_vals, key=convert_planner_name):
            v = planner_vals[p]
            lines.append([env, convert_planner_name(p), len(v), "%.1f" % np.mean(v),
                          "%.1f" % np.median(v)])
        for m in ("2WS", "4WS"):
            v = valid(ours.get(env, {}).get(m, {}).values())
            lines.append([env, "SHFM %s" % m, len(v), "%.1f" % np.mean(v), "%.1f" % np.median(v)])
    S.write_table("swept_area.md",
                  "# Swept area (m²)\n\n"
                  "Sampling planners: every trajectory of the campaigns. Ours: k1 = 2, k2 = 3, "
                  "one path per distinct problem.\n\n"
                  + S.md_table(["scene", "planner", "n", "mean", "median"], lines))


if __name__ == "__main__":
    main()
