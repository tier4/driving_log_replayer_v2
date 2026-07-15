"""HTML-report formatting helpers shared by physical_validity.py and reidentify/report.py."""
from __future__ import annotations

from collections.abc import Iterable
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


def html_table(
    headers: Iterable[str], rows: Iterable[Iterable[Any]], *, css_class: str = "",
) -> str:
    head = "".join(f"<th>{escape(header)}</th>" for header in headers)
    body = "".join(
        "<tr>" + "".join(f"<td>{escape(cell)}</td>" for cell in row) + "</tr>"
        for row in rows
    )
    class_attr = f' class="{escape(css_class)}"' if css_class else ""
    return f'<div class="table-wrap"><table{class_attr}><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table></div>'
