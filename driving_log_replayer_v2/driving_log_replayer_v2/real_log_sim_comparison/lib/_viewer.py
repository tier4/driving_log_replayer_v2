"""Unified trajectory playback and vehicle-model viewer."""

from __future__ import annotations

from pathlib import Path
import warnings

from ._html_utils import dumps_script_json
from ._model_seed import _seed_from_params
from ._model_viewer import MODEL_RATE_HZ, _HTML_TEMPLATE as MODEL_TEMPLATE
from ._playback_viewer import (
    BASELINE_LABEL,
    PLAYBACK_WHEELBASE_M,
    _HTML_TEMPLATE as PLAYBACK_TEMPLATE,
    build_playback_payload,
)

VIEWER_FILENAME = "viewer.html"


def build_viewer_payload(
    data: dict,
    map_ways: list | None,
    *,
    sim_params: dict | None = None,
    title: str = "",
    metrics: dict | None = None,
    model_registry: dict | None = None,
    tabs: tuple[str, ...] = ("playback", "model"),
    initial_tab: str = "playback",
    initial_t: float | None = None,
) -> dict | None:
    """Build the single payload consumed by both viewer tabs."""
    playback = build_playback_payload(
        data, map_ways, rate_hz=MODEL_RATE_HZ, title=title, metrics=metrics
    )
    if playback is None:
        return None

    baseline = data.get(BASELINE_LABEL)
    if baseline is None:
        baseline = next(iter(data.values()), None)
    model = None
    if baseline is not None:
        model = build_playback_payload(
            {BASELINE_LABEL: baseline}, map_ways, rate_hz=MODEL_RATE_HZ, title=title
        )
    if model is not None:
        params = sim_params or {}
        model["model_seed"] = _seed_from_params(params)
        model["model_registry"] = {
            name: _seed_from_params({**params, **(override or {})})
            for name, override in (model_registry or {}).items()
        }
        model["wheelbase"] = float(PLAYBACK_WHEELBASE_M)
        if initial_t is not None and initial_t > 0:
            model["initial_t"] = float(initial_t)

    available = [tab for tab in tabs if tab == "playback" or model is not None]
    if not available:
        return None
    return {
        "title": title,
        "tabs": available,
        "initial_tab": initial_tab if initial_tab in available else available[0],
        "playback": playback,
        "model": model,
    }


def render_viewer_html(payload: dict) -> str:
    """Render one offline HTML artifact with lazy tab-local runtimes."""
    return (
        _HTML_TEMPLATE.replace("__PAYLOAD_JSON__", dumps_script_json(payload))
        .replace("__PLAYBACK_TEMPLATE_JSON__", dumps_script_json(PLAYBACK_TEMPLATE))
        .replace("__MODEL_TEMPLATE_JSON__", dumps_script_json(MODEL_TEMPLATE))
    )


def write_viewer_html(payload: dict, out_path: Path) -> None:
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(render_viewer_html(payload), encoding="utf-8")
    print(f"  保存: {out_path.name}")


def plot_viewer(
    data: dict,
    map_ways: list | None,
    figs_dir: Path,
    *,
    sim_params: dict | None = None,
    title: str = "",
    metrics: dict | None = None,
    model_registry: dict | None = None,
) -> None:
    payload = build_viewer_payload(
        data,
        map_ways,
        sim_params=sim_params,
        title=title,
        metrics=metrics,
        model_registry=model_registry,
    )
    if payload is None:
        warnings.warn("kinematic データなし。viewer をスキップ", stacklevel=2)
        return
    write_viewer_html(payload, Path(figs_dir) / VIEWER_FILENAME)


_HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="ja"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>走行ログ統合ビューア</title>
<style>
html,body{margin:0;height:100%;font-family:"Hiragino Sans","Noto Sans CJK JP",sans-serif;background:#fff;color:#222}
#app{height:100%;display:flex;flex-direction:column}.tabs{display:flex;gap:4px;padding:8px 10px 0;background:#f5f7fa;border-bottom:1px solid #d8dee8}
.tabs button{border:1px solid #d8dee8;border-bottom:0;padding:8px 18px;background:#e9edf3;color:#344054;cursor:pointer;border-radius:5px 5px 0 0}
.tabs button.active{background:#fff;color:#17212b;font-weight:700;box-shadow:0 -1px 0 #fff inset}.frame{flex:1;min-height:0;background:#fff}
iframe{width:100%;height:100%;border:0;display:block}.empty{padding:2rem;color:#666}
</style></head><body><div id="app"><div class="tabs" id="tabs"></div><div class="frame" id="frame"></div></div>
<script>
const DATA=__PAYLOAD_JSON__,TEMPLATES={playback:__PLAYBACK_TEMPLATE_JSON__,model:__MODEL_TEMPLATE_JSON__};
const labels={playback:"軌跡比較",model:"モデル検証"},cache={};
function openTab(tab){
  if(!DATA.tabs.includes(tab))return;
  document.querySelectorAll("#tabs button").forEach(b=>b.classList.toggle("active",b.dataset.tab===tab));
  const host=document.getElementById("frame");host.replaceChildren();
  if(!cache[tab]){
    const payload=DATA[tab];if(!payload){host.innerHTML='<div class="empty">表示データがありません。</div>';return}
    const f=document.createElement("iframe");f.title=labels[tab];
    const token="__PAYLOAD_"+"JSON__";
    f.srcdoc=TEMPLATES[tab].replace(token,JSON.stringify(payload).replace(/<\//g,"<\\/"));
    cache[tab]=f;
  }
  host.appendChild(cache[tab]);
}
DATA.tabs.forEach(tab=>{const b=document.createElement("button");b.dataset.tab=tab;b.textContent=labels[tab];
b.onclick=()=>openTab(tab);document.getElementById("tabs").appendChild(b)});
openTab(DATA.initial_tab);
</script></body></html>"""
