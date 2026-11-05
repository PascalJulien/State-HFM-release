"""Running the figure scripts of ``generators/``.

Each entry below names a script and every file it writes, so that a run that
stops halfway is caught. The scripts
run in a process of their own, with ``MPLBACKEND=Agg`` and
``SHFM_FIGURE_DIR`` set to the output directory; tables of values go to its
``tables/`` subdirectory. A notebook can run one and show the result.
"""
import os
import subprocess
import sys
import time
import warnings

__all__ = ["GENERATORS", "run", "figure_path"]

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_GENERATORS_DIR = os.path.join(_REPO, "generators")
# Same default as shfm.figures.FIGURE_DIR, so that a notebook run under
# $SHFM_FIGURE_DIR sends the generators it calls to the same place.
_FIGURES = os.environ.get("SHFM_FIGURE_DIR") or os.path.join(_REPO, "figures")

GENERATORS = {
    # vehicle, collision model, configuration space
    "tissot": ("tissot.py",
     ["tissot_d_improved.pdf", "tissot_d_improved.png"]),
    "multiball": ("multiball.py",
     ["multiball_16.png", "multiball_4.png", "multiball_8.png",
      "multiball_combined.png"]),
    "env_2d3d": ("env_2d3d.py",
     ["env_2d3d_2d.png", "env_2d3d_3d.png", "env_2d3d_combined.png"]),
    "collision": ("collision_check.py",
     ["collision_check.pdf", "collision_check.png"]),
    # the scheme
    "nav_function": ("nav_function.py",
     ["4WS_nav_function.png"]),
    "nav_function_map": ("two_rooms_map.py",
     ["4WS_nav_function_map.pdf"]),
    "stencil": ("stencil_3d.py",
     ["stencil_3d_a_darker_grid.pdf", "stencil_3d_a_darker_grid.png",
      "stencil_3d_b_darker_grid.pdf", "stencil_3d_b_darker_grid.png"]),
    # the state system
    "control_fans": ("control_fans.py",
     ["control_fans.pdf", "control_fans.tex"]),
    "state_system": ("state_system_comparison.py",
     ["no_state.pdf", "with_state.pdf"]),
    # scenes
    "procedural_envs": ("procedural_envs.py",
     ["procedural_envs.pdf", "procedural_envs.png"]),
    # the worked stencil example
    "stencil_example": ("stencil_example.py",
     ["stencil.pdf", "stencil.png"]),
    # resolution study
    "resolution": ("resolution_study.py",
     ['resolution_cost_convergence.pdf', 'resolution_cost_convergence.png',
      'resolution_traj_overlay.pdf', 'resolution_traj_overlay.png']),
    "rasterisation": ("rasterisation_scheme.py",
     ['np_rasterisation.pdf', 'np_rasterisation.png']),
    # parameter study
    "parameter_sweeps": ("parameter_sweeps.py",
     ['sens_situations.pdf', 'sens_situations.png', 'sens_k2_accostage.pdf',
      'sens_k2_accostage.png', 'sens_k2_np.pdf', 'sens_k2_np.png',
      'sens_k1_demitour.pdf', 'sens_k1_demitour.png', 'sens_k1_entree.pdf',
      'sens_k1_entree.png', 'sens_gamma_rev_demitour.pdf',
      'sens_gamma_rev_demitour.png', 'sens_gamma_rev_sortie.pdf',
      'sens_gamma_rev_sortie.png', 'sens_gamma_rev_entree.pdf',
      'sens_gamma_rev_entree.png', 'sens_gamma_rev_traversee.pdf',
      'sens_gamma_rev_traversee.png', 'sens_nu_FWD_traversee.pdf',
      'sens_nu_FWD_traversee.png', 'sens_nu_2WS_sortie.pdf',
      'sens_nu_2WS_sortie.png', 'sens_nu_4WS_traversee.pdf',
      'sens_nu_4WS_traversee.png', 'sens_nu_4WS_demitour.pdf',
      'sens_nu_4WS_demitour.png']),
    "parameter_tables": ("parameter_tables.py",
     ["tables/parameter_study.md"]),
    "clearance_cap": ("clearance_cap.py",
     ['saturation_metriques.pdf', 'saturation_metriques.png',
      'saturation_traj_two_rooms.pdf', 'saturation_traj_two_rooms.png',
      'tables/clearance_cap.md']),
    # comparison with the sampling planners
    "comparison": ("comparison_plates.py",
     ['env1_traj.pdf', 'env1_traj.png', 'runs_warehouse.pdf',
      'runs_warehouse.png', 'run_0123.pdf', 'env1_metrics.pdf',
      'warehouse_metrics.pdf', 'procedural_metrics.pdf', 'creneau_2ws.pdf',
      'creneau_4WS.pdf', 'creneau_2ws_nl.pdf', 'creneau_4ws_nl.pdf',
      'accostage_sota.pdf', '2ws_only_shortcuts.pdf',
      '4ws_only_shortcuts.pdf', 'creneau_4ws_k1050_nl.pdf',
      '4ws_only_shortcuts_k1050.pdf', 'tables/comparison.md',
      'tables/comparison_summary.tex', 'tables/docking.md',
      'tables/reference_costs.md', 'tables/approach.md']),
    "modes": ("mode_use.py",
     ['modes_two_rooms.pdf', 'modes_two_rooms.png', 'tables/modes.md']),
    "swept_area": ("swept_area.py",
     ['violin_swept_env1.pdf', 'violin_swept_env1.png',
      'violin_swept_warehouse.pdf', 'violin_swept_warehouse.png',
      'violin_swept_procedural.pdf', 'violin_swept_procedural.png',
      'tables/swept_area.md']),
    "budget": ("budget_table.py",
     ['tables/budget.md', 'tables/budget_table.tex']),
}


def figure_path(name, figure_dir=None):
    return os.path.join(figure_dir or _FIGURES, name)


def run(name, figure_dir=None, timeout=3600):
    """Run one generator and return the paths it produced.

    Raises if the script fails or if a file it is supposed to write is missing.
    """
    if name not in GENERATORS:
        raise ValueError("unknown generator %r, expected one of %s"
                         % (name, ", ".join(sorted(GENERATORS))))
    script, expected = GENERATORS[name]
    out = figure_dir or _FIGURES
    os.makedirs(out, exist_ok=True)
    env = dict(os.environ, MPLBACKEND="Agg", SHFM_FIGURE_DIR=os.path.abspath(out))
    if env.get("SHFM_DATA_DIR"):
        # The script runs in generators/: a relative path is the caller's.
        env["SHFM_DATA_DIR"] = os.path.abspath(env["SHFM_DATA_DIR"])
    path = os.path.join(_GENERATORS_DIR, script)
    started = time.time()
    done = subprocess.run([sys.executable, path], cwd=os.path.dirname(path),
                          env=env, capture_output=True, timeout=timeout)
    if done.returncode != 0:
        raise RuntimeError("%s failed:\n%s\n%s"
                           % (script, done.stdout.decode("utf-8", "replace")[-2000:],
                              done.stderr.decode("utf-8", "replace")[-4000:]))
    produced = [figure_path(f, out) for f in expected]
    missing = [p for p in produced if not os.path.exists(p)]
    if missing:
        raise RuntimeError("%s did not write %s"
                           % (script, ", ".join(os.path.basename(m)
                                                for m in missing)))
    # A PNG copy that was not rewritten while its PDF was: the converter,
    # pdftoppm, is probably missing, and the script only logs it.
    stale = [p for p in produced if p.endswith(".png")
             and p[:-4] + ".pdf" in produced
             and os.path.getmtime(p) < started <= os.path.getmtime(p[:-4] + ".pdf")]
    if stale:
        warnings.warn("%s rewrote the PDF but not %s; is pdftoppm (poppler) "
                      "installed?" % (script, ", ".join(os.path.basename(p)
                                                        for p in stale)),
                      RuntimeWarning, stacklevel=2)
    return produced
