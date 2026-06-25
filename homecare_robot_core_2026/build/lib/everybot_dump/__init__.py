from __future__ import annotations

from importlib import metadata as _metadata

__all__ = ["__version__", "get_version"]

def get_version() -> str:
    try:
        return _metadata.version("everybot")
    except _metadata.PackageNotFoundError:
        return "0.0.0"

__version__ = get_version()