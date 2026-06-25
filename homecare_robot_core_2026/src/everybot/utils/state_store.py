from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, Optional


class StateStore:
    """
    Simple JSON persistence for small runtime states.
    - No dependency to services layer (prevents circular import)
    - Stores only primitives(dict/str/bool/...)
    """
    def __init__(self, path: str):
        self._path = Path(path)

    def load(self) -> Dict[str, Any]:
        try:
            if not self._path.exists():
                return {}
            raw = self._path.read_text(encoding="utf-8")
            data = json.loads(raw)
            return data if isinstance(data, dict) else {}
        except Exception:
            return {}

    def save(self, data: Dict[str, Any]) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)

        tmp = self._path.with_suffix(self._path.suffix + ".tmp")
        payload = json.dumps(data, ensure_ascii=False, indent=2)

        tmp.write_text(payload, encoding="utf-8")
        os.replace(tmp, self._path)

    def clear(self) -> None:
        try:
            self._path.unlink(missing_ok=True)
        except Exception:
            pass