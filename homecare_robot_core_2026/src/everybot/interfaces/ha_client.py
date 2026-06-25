from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass
from typing import Any

import requests

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class HomeAssistantConfig:
    ha_ip: str                 
    token: str                 
    timeout_sec: float = 3.0


class HomeAssistantClient:
    def __init__(self, cfg: HomeAssistantConfig):
        self._cfg = cfg
        self._base_url = self._make_base_url(cfg.ha_ip)

    @staticmethod
    def _make_base_url(ip_or_url: str) -> str:
        s = (ip_or_url or "").strip()
        if not s:
            raise ValueError("ha_ip is empty")
        if s.startswith("http://") or s.startswith("https://"):
            return s.rstrip("/")
        # allow "ip:port"
        if ":" in s:
            return f"http://{s}".rstrip("/")
        return f"http://{s}:8123"

    def _headers(self) -> dict[str, str]:
        return {
            "Authorization": f"Bearer {self._cfg.token}",
            "Content-Type": "application/json",
        }

    def _call_with_retry(self, fn, retries: int = 1, backoff: float = 1.0):
        """연결 실패 시 지수 백오프 retry. 최종 실패 시 None 반환."""
        for attempt in range(retries + 1):
            try:
                return fn()
            except (requests.exceptions.ConnectionError, requests.exceptions.Timeout) as exc:
                if attempt < retries:
                    wait = backoff * (2 ** attempt)
                    log.warning(
                        "[HA] attempt %d/%d failed: %s - retry in %.1fs",
                        attempt + 1,
                        retries + 1,
                        exc,
                        wait,
                    )
                    time.sleep(wait)
                else:
                    log.warning("[HA] unreachable after %d retries: %s", retries + 1, exc)
                    return None
            except requests.exceptions.HTTPError as exc:
                log.error("[HA] HTTP error: %s", exc)
                return None
            except Exception as exc:
                log.error("[HA] unexpected: %s", exc)
                return None

    def _list_states_raw(self) -> list[dict[str, Any]]:
        r = requests.get(
            f"{self._base_url}/api/states",
            headers=self._headers(),
            timeout=self._cfg.timeout_sec,
        )
        r.raise_for_status()
        return r.json()

    def _get_state_raw(self, entity_id: str) -> dict[str, Any]:
        r = requests.get(
            f"{self._base_url}/api/states/{entity_id}",
            headers=self._headers(),
            timeout=self._cfg.timeout_sec,
        )
        r.raise_for_status()
        return r.json()

    def _call_service_raw(self, domain: str, service: str, service_data: dict[str, Any]) -> Any:
        r = requests.post(
            f"{self._base_url}/api/services/{domain}/{service}",
            headers=self._headers(),
            data=json.dumps(service_data),
            timeout=self._cfg.timeout_sec,
        )
        r.raise_for_status()
        return r.json()

    def list_states(self) -> list[dict[str, Any]] | None:
        return self._call_with_retry(lambda: self._list_states_raw())

    def get_state(self, entity_id: str) -> dict[str, Any] | None:
        return self._call_with_retry(lambda: self._get_state_raw(entity_id))

    def call_service(self, domain: str, service: str, service_data: dict[str, Any]) -> Any:
        return self._call_with_retry(lambda: self._call_service_raw(domain, service, service_data))
