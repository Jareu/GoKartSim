"""
Helper for loading Project Chrono's Python bindings with informative errors.
"""


def _load_chrono():
    """
    Try to import the official Project Chrono Python bindings.

    The upstream wheels now expose most symbols under ``pychrono.core``.
    Keep a couple of fallbacks for older builds or distro-packaged variants.
    """
    try:
        from pychrono import core as chrono_core  # type: ignore
    except ImportError:
        chrono_core = None
    else:
        if hasattr(chrono_core, "ChSystemNSC"):
            return chrono_core

    try:
        import pychrono as chrono_legacy  # type: ignore
    except ImportError:
        chrono_legacy = None
    else:
        if hasattr(chrono_legacy, "ChSystemNSC"):
            return chrono_legacy

    try:
        import chrono as chrono_alt  # type: ignore
    except ImportError:
        chrono_alt = None
    else:
        if hasattr(chrono_alt, "ChSystemNSC"):
            return chrono_alt

    raise ImportError(
        "Project Chrono Python bindings not found. "
        "Install the official PyChrono wheels from https://projectchrono.org/download/ "
        "and ensure they are on PYTHONPATH. The PyPI package named 'pychrono' is unrelated "
        "and will not work for this simulation."
    )


chrono = _load_chrono()

__all__ = ["chrono"]

