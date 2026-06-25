from __future__ import annotations

import json
from typing import Any, Dict


def dumps_line(obj: Dict[str, Any]) -> bytes:
    return (json.dumps(obj, ensure_ascii=False) + "\n").encode("utf-8")


def loads_line(line: bytes) -> Dict[str, Any]:
    return json.loads(line.decode("utf-8", errors="replace"))
