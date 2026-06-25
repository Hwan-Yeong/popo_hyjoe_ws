from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import time
from pathlib import Path

from . import __version__

def _setup_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s.%(msecs)03d %(levelname)s %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )

def _default_config_path() -> str:
    # 1순위: 환경변수 (배포 환경)
    env = os.getenv("EVERYBOT_CONFIG")
    if env:
        return env

    # 2순위: CWD 기준 configs/everybot.yaml
    #   → venv pip install 후 프로젝트 루트에서 실행하는 경우 (가장 일반적)
    cwd_cfg = Path.cwd() / "configs" / "everybot.yaml"
    if cwd_cfg.exists():
        return str(cwd_cfg)

    # 3순위: __file__ 기준 — editable install(pip install -e .) 또는 소스 직접 실행
    #   __file__: src/everybot/__main__.py → parents[2] = 프로젝트 루트
    src_root = Path(__file__).resolve().parents[2]
    src_cfg = src_root / "configs" / "everybot.yaml"
    if src_cfg.exists():
        return str(src_cfg)

    # 폴백: CWD 기준 경로 반환 (FileNotFoundError는 loader에서 명확하게 처리)
    return str(cwd_cfg)

def _parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog="everybot", description="homecare robot runtime (python)")
    p.add_argument("--config", default=_default_config_path(), help="Config file path (yaml/json)")
    p.add_argument("--log-level", default="INFO", help="DEBUG/INFO/WARNING/ERROR")
    p.add_argument("-V", "--version", action="store_true", help="Print version and exit")
    return p.parse_args(argv)

def main(argv: list[str] | None = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)

    if args.version:
        print(__version__)
        return 0

    _setup_logging(args.log_level)
    log = logging.getLogger("robot")

    config_path = Path(args.config)
    log.info("Config : %s", config_path)

    stop = {"flag": False}

    def _on_signal(signum: int, _frame) -> None:
        stop["flag"] = True
        log.warning("", signum)

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        from .app.main import run 
        return int(run(config_path=config_path, stop_flag=lambda: stop["flag"]))
    except ModuleNotFoundError:

        log.warning("everybot.app.main.run() not found")
        while not stop["flag"]:
            time.sleep(0.2)

        log.info("Stopped")
        return 0
    except Exception:
        log.exception("Exception")
        return 1

if __name__ == "__main__":
    raise SystemExit(main())