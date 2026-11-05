"""Paths of the 4WS on Two-Rooms against the steering saturation and the
ratio of the navigation speed coefficients.

Reads ``data/comparison/modes`` (``scripts/acquire_comparison.py --part
modes``): the Two-Rooms problem, both vehicles, k1 = 2, k2 = 3, for five
steering saturations delta*, given to both axles (the rear one only acts in
the 4WS manoeuvring modes), and four ratios nu_2WS / nu_FWD.
Paths cleaned and closed on the start pose.

Writes ``modes_two_rooms`` (4WS: each sweep over the whole map, and a detail
of the framed area) and ``tables/modes.md``: length, swept area, the turn
limit tan(delta*) / L, the largest heading change per unit length over 1 m,
the three-point curvature of Bench-MR, and the largest rotation over 1 m in
each navigation mode, on the heading and on the geometric tangent. CPU only.
"""
import numpy as np

import study_tools as S
from study_tools import plt
from shfm import DEFAULT_VEHICLE

SWEEPS = (("delta", "δ* (rad)"), ("ratio_2ws_fwd", "ν_2WS / ν_FWD"))


def heading_rate_max(p, window):
    """Largest |delta theta| / arc length over stretches of ``window`` grid units:
    the tightest turn of the path, measured over a length that averages out the
    grid-scale oscillation of the extracted points."""
    p = np.asarray(p, dtype=float)
    th = np.unwrap(p[:, 2])
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1))]
    best, j = 0.0, 0
    for i in range(len(p)):
        while j < len(p) and s[j] - s[i] < window:
            j += 1
        if j >= len(p):
            break
        best = max(best, abs(th[j] - th[i]) / (s[j] - s[i]))
    return best


def rates(q, window=8.0):
    """Largest rotation over ``window`` grid units within one mode, per mode:
    ``{mode: [on the heading, on the geometric tangent]}``, per grid unit.
    In the backward modes the tangent is turned by pi."""
    q = np.asarray(q, float)
    d = np.diff(q[:, :2], axis=0); st = np.linalg.norm(d, axis=1)
    keep = st > 1e-6
    m = q[:-1, 3].astype(int)[keep]; d = d[keep]; st = st[keep]
    th = np.unwrap(q[:-1, 2][keep])
    tang = np.arctan2(d[:, 1], d[:, 0])
    tang = np.unwrap(np.where(np.isin(m, (2, 3)), tang + np.pi, tang))
    s = np.r_[0, np.cumsum(st)][:-1]
    out, j = {}, 0
    for i in range(len(st)):
        while j < len(st) and s[j] - s[i] < window:
            j += 1
        if j >= len(st):
            break
        if np.all(m[i:j + 1] == m[i]):
            L = s[j] - s[i]
            o = out.setdefault(m[i], [0, 0])
            o[0] = max(o[0], abs(th[j] - th[i]) / L); o[1] = max(o[1], abs(tang[j] - tang[i]) / L)
    return out


def main():
    rows, paths = S.load_part("comparison", "modes")
    env = S.scene_env("two_rooms")
    goal = S.endpoints("two_rooms")[1]
    zoom = S.ZOOM["two_rooms"]
    lines, rot = [], []
    for method in ("2WS", "4WS"):
        draw = method == "4WS"
        if draw:
            fig, axs2 = plt.subplots(2, 2, figsize=(15, 14))
        for col, (sweep, lab) in enumerate(SWEEPS):
            rr = sorted([r for r in rows if r["method"] == method and r["sweep"] == sweep
                         and r["key"] in paths], key=lambda r: S.fnum(r["value"]))
            cleaned = [S.clean_path(paths[r["key"]], "two_rooms", 0, method)[0] for r in rr]
            for r, q in zip(rr, cleaned):
                v = S.fnum(r["value"])
                kappa = heading_rate_max(q, zoom)                       # over 1 m
                m = S.measure(q, env, goal)
                delta = v if sweep == "delta" else 0.5
                kmax = np.tan(delta) / (DEFAULT_VEHICLE.wheelbase * zoom)
                lines.append([method, lab, "%g" % v, "%.1f" % m["path_length"],
                              "%.1f" % m["swept_area_m2"], "%.3f" % kmax, "%.3f" % kappa,
                              "%.2f" % m["max_curvature"], "%.3f" % m["aol"]])
                o = rates(q)
                rot.append([method, lab, "%.2f" % v, "%.2f" % (np.tan(delta) / DEFAULT_VEHICLE.wheelbase)]
                           + ["%.2f / %.2f" % (o[k][0] * zoom, o[k][1] * zoom) if k in o else "-"
                              for k in (0, 3)])
            if not draw:
                continue
            for row in (0, 1):
                ax = axs2[row, col]
                S.draw_map(ax, env.walls, env.zoom)
                cmap = plt.get_cmap("viridis", max(2, len(rr)))
                for i, (r, q) in enumerate(zip(rr, cleaned)):
                    v = S.fnum(r["value"])
                    ref = abs(v - 0.5) < 1e-6
                    ax.plot(q[:, 0], q[:, 1], color=cmap(i), lw=2.6 if ref else 1.6,
                            label="%g%s" % (v, " (réf.)" if ref else ""))
                box = (28, 112, 50, 46) if sweep == "delta" else (28, 15, 55, 50)
                if row == 0:
                    ax.legend(title=lab, fontsize=9)
                    ax.set_title("SHFM %s, Two-Rooms : %s" % (method, lab), fontsize=11)
                    ax.add_patch(plt.Rectangle(box[:2], box[2], box[3], fill=False, ls="--", color="0.3"))
                else:
                    ax.set_xlim(box[0], box[0] + box[2]); ax.set_ylim(box[1], box[1] + box[3])
                    ax.set_aspect("equal")
                    ax.set_title("détail (cadre en pointillés)", fontsize=10)
        if draw:
            fig.tight_layout()
            S.save(fig, "modes_two_rooms")
    S.write_table("modes.md",
                  "# Steering saturation and ratio of the navigation speed coefficients "
                  "(k1 = 2, k2 = 3)\n\n"
                  "Two-Rooms (1 cell = 0.125 m). Turn limit: tan(delta*) / L, the tightest turn "
                  "of navigation. Rotation over 1 m: largest heading change per unit length "
                  "over a window of 8 cells. Three-point curvature: Bench-MR metric.\n\n"
                  + S.md_table(["vehicle", "sweep", "value", "length", "swept area (m²)",
                                "turn limit (1/cell)", "rotation over 1 m (1/cell)",
                                "three-point curvature", "AOL"], lines)
                  + "\n## Largest rotation over 1 m within one mode (1/m)\n\n"
                  "Heading / geometric tangent, away from the mode switches; limit: "
                  "tan(delta*) / L in 1/m.\n\n"
                  + S.md_table(["vehicle", "sweep", "value", "limit (1/m)",
                                "forward navigation", "backward navigation"], rot))


if __name__ == "__main__":
    main()
