"""
Helper for loading Project Chrono Python bindings.

Raises a descriptive ImportError when the official bindings are not present.
"""


def _load_chrono():
    """
    Try to import the Project Chrono Python bindings.

    Users sometimes install the unrelated package named ``pychrono`` from PyPI,
    which lacks the simulation classes (e.g., ``ChSystemNSC``). This helper
    detects that situation and provides clearer guidance.
    """
    # Preferred import style used by the official bindings.
    try:
        import pychrono as chrono  # type: ignore
    except ImportError:
        chrono = None
    else:
        if hasattr(chrono, "ChSystemNSC"):
            return chrono
        # Some builds expose symbols under pychrono.core
        try:
            from pychrono import core as chrono_core  # type: ignore
        except ImportError:
            pass
        else:
            if hasattr(chrono_core, "ChSystemNSC"):
                return chrono_core

    # Fallback module name occasionally used in packaged builds.
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

