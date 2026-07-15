"""HTML-report formatting helpers shared by physical_validity.py and reidentify/report.py."""
from __future__ import annotations

import html
import math
from typing import Any


def escape(value: Any) -> str:
    return html.escape(str(value), quote=True)


def format_number(value: Any) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "—"
    if not math.isfinite(number):
        return "—"
    return f"{number:.6g}"
