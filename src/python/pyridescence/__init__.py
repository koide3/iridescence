import sys as _sys

from . import _pyridescence
from ._pyridescence import gl, glk, guik, imgui, pfd

# Register the extension's submodules so that `import pyridescence.guik`
# and `from pyridescence.glk import primitives` work as regular imports.
for _name in ("gl", "glk", "guik", "imgui", "pfd"):
    _sys.modules[f"{__name__}.{_name}"] = getattr(_pyridescence, _name)
_sys.modules[f"{__name__}.glk.primitives"] = glk.primitives

try:
    from importlib.metadata import version as _version

    __version__ = _version("pyridescence")
except Exception:
    __version__ = "dev"

__all__ = ["gl", "glk", "guik", "imgui", "pfd"]
