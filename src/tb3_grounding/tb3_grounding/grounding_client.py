"""HTTP client for grounding/server.py (repo root). stdlib only, so the
ROS container needs no extra pip packages."""

from __future__ import annotations

import base64
import json
import urllib.error
import urllib.request
from typing import Optional


class GroundingError(RuntimeError):
    """Grounding infrastructure failure (server unreachable, bad reply)."""


class GroundingClient:

    def __init__(self, base_url: str, timeout_sec: float = 20.0) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_sec = timeout_sec

    def health(self) -> Optional[dict]:
        """Server info, or None if unreachable."""
        try:
            req = urllib.request.Request(self.base_url + "/health")
            with urllib.request.urlopen(req, timeout=self.timeout_sec) as resp:
                return json.loads(resp.read())
        except (urllib.error.URLError, json.JSONDecodeError, OSError):
            return None

    def ground(self, image_bytes: bytes, query: str) -> list[dict]:
        """Ground the query in the encoded image; raises GroundingError on
        any transport/server/protocol failure."""
        payload = json.dumps({
            "image": base64.b64encode(image_bytes).decode("ascii"),
            "query": query,
        }).encode()

        req = urllib.request.Request(
            self.base_url + "/ground",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_sec) as resp:
                body = json.loads(resp.read())
        except urllib.error.HTTPError as exc:
            detail = ""
            try:
                detail = json.loads(exc.read()).get("error", "")
            except Exception:
                pass
            raise GroundingError(
                f"grounding server HTTP {exc.code}: {detail or exc.reason}"
            ) from exc
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
            raise GroundingError(f"grounding server unreachable: {exc}") from exc

        boxes = body.get("boxes")
        if not isinstance(boxes, list):
            raise GroundingError("malformed grounding response (no 'boxes')")
        return boxes
