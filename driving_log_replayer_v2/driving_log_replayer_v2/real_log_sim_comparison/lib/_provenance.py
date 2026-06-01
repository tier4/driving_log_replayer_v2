"""DiffusionPlanner モデル重み / autoware バージョンの provenance 取得・記録.

real データ取得時と sim 実行時で pilot-auto.x2 のバージョンや DiffusionPlanner の重みが
異なり得るため、観測された乖離が「車両モデルの忠実度」由来か「版・重みの差」由来かを
切り分けられるよう、各 sim run がどの重み・バージョンを使ったかを記録し、比較プロットに
掲載する。

- sim 側: 実行時 (step3) に `capture_dp_provenance()` で onnx の sha256/mtime/size、
  args.json の exp_name/学習データ、autoware バージョンを取得し、`<lite>/provenance.json` に書く。
- real 側: 取得時の版・重みは外部 (車両デプロイ) のため自動取得できない。scenario.yaml の
  `Conditions.real_provenance` (任意の自由文字列) を env REAL_PROVENANCE 経由で受け取り掲載する。
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
from pathlib import Path

# DP onnx の既定探索パス (autoware_launch の diffusion_planner.param.yaml::onnx_model_path)。
# env DP_ONNX_PATH で上書き可能。
_DP_ONNX_CANDIDATES = [
    "/opt/autoware/mlmodels/diffusion_planner_for_x2_exp/diffusion_planner.onnx",
    "/opt/autoware/mlmodels/diffusion_planner/model.onnx",
]


def _resolve_onnx() -> Path | None:
    env = os.environ.get("DP_ONNX_PATH")
    cands = ([env] if env else []) + _DP_ONNX_CANDIDATES
    for c in cands:
        p = Path(c)
        if p.exists():
            return p
    return None


def _sha8(path: Path) -> str | None:
    try:
        h = hashlib.sha256()
        with path.open("rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        return h.hexdigest()[:8]
    except Exception:  # noqa: BLE001
        return None


def _autoware_version() -> str:
    """autoware (pilot-auto.x2) のバージョンを best-effort で取得する。"""
    # 1) /opt/autoware/version 等のビルド manifest
    for vf in ("/opt/autoware/version", "/opt/autoware/VERSION"):
        try:
            txt = Path(vf).read_text(encoding="utf-8").strip()
            if txt:
                return txt.splitlines()[0]
        except Exception:  # noqa: BLE001
            pass
    # 2) src/autoware/universe の git describe
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / "src" / "autoware" / "universe"
        if cand.exists():
            try:
                out = subprocess.run(
                    ["git", "-C", str(cand), "describe", "--tags", "--always"],
                    capture_output=True, text=True, timeout=10, check=False,
                )
                if out.returncode == 0 and out.stdout.strip():
                    return out.stdout.strip()
            except Exception:  # noqa: BLE001
                pass
            break
    return "unknown"


def capture_dp_provenance(onnx_path: str | Path | None = None) -> dict:
    """DiffusionPlanner 重み + autoware バージョンの provenance を取得する。

    取得不能な項目は None / "unknown" を入れる (実行環境差に頑健)。
    """
    onnx = Path(onnx_path) if onnx_path else _resolve_onnx()
    prov: dict = {
        "autoware_version": _autoware_version(),
        "dp_onnx_path": str(onnx) if onnx else None,
        "dp_onnx_sha8": _sha8(onnx) if onnx else None,
        "dp_onnx_size": (onnx.stat().st_size if onnx else None),
        "dp_onnx_mtime": (
            int(onnx.stat().st_mtime) if onnx else None
        ),
        "dp_exp_name": None,
        "dp_train_set": None,
    }
    if onnx is not None:
        args_json = onnx.parent / "args.json"
        try:
            args = json.loads(args_json.read_text(encoding="utf-8"))
            prov["dp_exp_name"] = args.get("exp_name")
            train = args.get("train_set_list")
            if isinstance(train, str):
                # 学習データの識別に効く親ディレクトリ名 (例 dataset_20260323_fixed_gt) を抽出
                parts = [p for p in Path(train).parts if "dataset" in p.lower()]
                prov["dp_train_set"] = parts[-1] if parts else Path(train).name
        except Exception:  # noqa: BLE001
            pass
    return prov


def format_provenance_line(prov: dict | None) -> str:
    """provenance dict を 1 行の短い表示文字列にする。"""
    if not prov:
        return "provenance: unknown"
    exp = prov.get("dp_exp_name") or "?"
    sha = prov.get("dp_onnx_sha8") or "?"
    awv = prov.get("autoware_version") or "?"
    train = prov.get("dp_train_set")
    s = f"DP={exp} (onnx {sha}) / autoware {awv}"
    if train:
        s += f" / train {train}"
    return s


def write_provenance(lite_dir: str | Path, extra: dict | None = None) -> dict:
    """capture_dp_provenance() + extra を `<lite_dir>/provenance.json` に書き、dict を返す。"""
    prov = capture_dp_provenance()
    if extra:
        prov.update(extra)
    out = Path(lite_dir) / "provenance.json"
    try:
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(prov, ensure_ascii=False, indent=2), encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass
    return prov


def read_provenance(lite_dir: str | Path) -> dict | None:
    """`<lite_dir>/provenance.json` を読む。無ければ None。"""
    p = Path(lite_dir) / "provenance.json"
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        return None
