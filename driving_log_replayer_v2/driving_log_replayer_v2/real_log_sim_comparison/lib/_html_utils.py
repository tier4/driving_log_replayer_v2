"""自己完結 HTML テンプレートへ JSON payload を埋め込む helper."""

from __future__ import annotations

import json
from typing import Any


def dumps_script_json(payload: Any) -> str:
    """`<script>` 埋め込み用に安全な JSON 文字列へ変換する。

    `</script>` による早期終了を避けるため、`</` を `<\\/>` に置換する。
    """
    payload_json = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    return payload_json.replace("</", "<\\/")


def render_template_with_payload(template: str, payload: Any) -> str:
    """`__PAYLOAD_JSON__` を持つ HTML テンプレートへ payload を埋め込む。"""
    return template.replace("__PAYLOAD_JSON__", dumps_script_json(payload))
