"""Where the notebooks write their figures.

``$SHFM_FIGURE_DIR`` overrides the destination. The standalone generators under
``generators/`` read the same variable, and :mod:`shfm.generators` sets it when
it runs them.
"""
import os

__all__ = ["FIGURE_DIR", "save"]

FIGURE_DIR = os.environ.get("SHFM_FIGURE_DIR") or os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "figures")


def save(fig, name, dpi=150, formats=("png",)):
    """Save ``fig`` under ``figures/`` and return the paths written."""
    os.makedirs(FIGURE_DIR, exist_ok=True)
    out = []
    for ext in formats:
        path = os.path.join(FIGURE_DIR, "%s.%s" % (name, ext))
        fig.savefig(path, dpi=dpi, bbox_inches="tight")
        out.append(path)
    return out
