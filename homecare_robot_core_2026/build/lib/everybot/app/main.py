from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Callable

from ..config.loader import load_robot_config
from .lifecycle import GracefultShutdown
from .wiring import build_runtime

log = logging.getLogger(__name__)

def run(config_path: Path, stop_flag: Callable[[], bool]) -> int:
     cfg = load_robot_config(config_path)
     
     runtime = build_runtime(cfg)

     shutdown = GracefultShutdown(external_stop=stop_flag)

     try:
        runtime.start()
        log.info("Runtime Started")
        
        period = 1.0 / max(cfg.tick_hz, 0.1)
        next_t = time.monotonic()

        while not shutdown.should_stop():
            runtime.tick()

            next_t += period
            sleep_t = next_t - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.monotonic()

        log.info("Stop requested")
        return 0
     except Exception:
        log.exception("Fatal error in runtime")
        return 1
     finally:
        runtime.stop()
        log.info("Runtime Stopped cleanly")



