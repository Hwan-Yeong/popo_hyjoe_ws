from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

@dataclass
class GracefultShutdown:
    external_stop: Callable[[], bool]

    def should_stop(self) -> bool:
        return bool(self.external_stop())