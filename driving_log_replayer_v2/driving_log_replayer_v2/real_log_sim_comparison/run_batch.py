#!/usr/bin/env python3
"""scenario.yaml の Evaluation.Datasets 全 UUID を順次ローカル実行するバッチドライバ。

クラウドの「1 評価ジョブ = 1 データセット」をローカルで再現する: scenario.yaml に
複数 dataset を列挙しておき、本スクリプトが UUID ごとに

  1. webauto キャッシュから t4_dataset_path を解決 (lib._dataset)
  2. 当該 dataset 1 件だけに絞った single-dataset scenario を生成
  3. `ros2 launch driving_log_replayer_v2` で per-dataset パイプライン (Stage 1〜12) を実行
     (output_dir = <batch-root>/runs/<uuid>)
  4. 成果物を collect_datasets.collect_bundle で <batch-root>/datasets/<uuid>/ に収集

を行い、最後に step13 (横断分析) と step11 --collection-dir (マルチ DS レポート) を実行する。
失敗した dataset は WARN + collection.yaml に status 記録して継続する
(multi_dataset_tune.load_datasets の skip 方針と同じ)。

使い方 (make local_batch_run 経由を推奨。ROS 環境を source 済みであること):
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.run_batch \
        --scenario sample/scenario.yaml --batch-root sample/out/batch_20260611_120000

    # 中断後の再開 (real.lite が揃っている dataset の sim 実行をスキップ)
    ... run_batch --scenario ... --batch-root <既存 batch root> --resume
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import subprocess
import sys

import yaml

from .collect_datasets import _resolve_bundle, collect_bundle, update_manifest
from .lib._dataset import DatasetResolutionError, resolve_t4_dataset_path
from .lib._io import resolve_lite_bag

_PKG = "driving_log_replayer_v2.real_log_sim_comparison"
_DEFAULT_WEBAUTO_ROOT = Path.home() / ".webauto" / "data" / "data" / "annotation_dataset"


def iter_dataset_uuids(scenario: Path) -> list[str]:
    """scenario.yaml の Evaluation.Datasets から UUID リストを取り出す (各エントリの先頭キー)。"""
    doc = yaml.safe_load(scenario.read_text(encoding="utf-8"))
    datasets = (doc.get("Evaluation") or {}).get("Datasets") or []
    uuids = []
    for entry in datasets:
        if isinstance(entry, dict) and entry:
            uuids.append(str(next(iter(entry.keys()))))
    return uuids


def write_single_dataset_scenario(scenario: Path, uuid: str, out_path: Path) -> Path:
    """Datasets を当該 UUID 1 件に絞った scenario を生成する。

    launch には t4_dataset_path/t4_dataset_id を明示で渡すが、scenario 側の Datasets も
    1 件に揃えておくことで「scenario には複数 dataset・実行は 1 dataset」という不整合を
    下流 (evaluator / step2) に持ち込まない。
    """
    doc = yaml.safe_load(scenario.read_text(encoding="utf-8"))
    datasets = (doc.get("Evaluation") or {}).get("Datasets") or []
    picked = [e for e in datasets if isinstance(e, dict) and str(next(iter(e), "")) == uuid]
    if not picked:
        raise ValueError(f"scenario に dataset {uuid} がありません: {scenario}")
    doc["Evaluation"]["Datasets"] = picked
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    return out_path


def run_one_dataset(
    uuid: str, scenario_single: Path, t4_dataset_path: Path, output_dir: Path
) -> bool:
    """1 dataset の per-dataset パイプラインを ros2 launch で実行する (成功で True)。

    Makefile local_cloud_run の launch 行と同一。ROS 環境 (install/setup.bash) は
    呼び出し元 shell で source 済みであることを前提とする。
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ros2", "launch", "driving_log_replayer_v2", "driving_log_replayer_v2.launch.py",
        f"scenario_path:={scenario_single}",
        f"t4_dataset_path:={t4_dataset_path}",
        f"t4_dataset_id:={uuid}",
        f"output_dir:={output_dir}",
        "with_autoware:=false",
    ]
    print(f"[RUN] {uuid}: {' '.join(cmd)}", flush=True)
    proc = subprocess.run(cmd, check=False)  # noqa: S603
    return proc.returncode == 0


def _bundle_has_real_lite(output_dir: Path) -> bool:
    """output_dir 配下のバンドルに real.lite が既にあるか (--resume のスキップ判定)。"""
    try:
        bundle = _resolve_bundle(output_dir)
    except FileNotFoundError:
        return False
    return resolve_lite_bag(bundle / "lite", "real") is not None


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="scenario.yaml の全 dataset を順次ローカル実行 + 収集 + 横断分析"
    )
    ap.add_argument("--scenario", required=True, help="複数 Datasets を列挙した scenario.yaml")
    ap.add_argument("--batch-root", required=True,
                    help="バッチ出力 root (runs/ datasets/ cross_dataset/ report.html を生成)")
    ap.add_argument("--webauto-root", default=str(_DEFAULT_WEBAUTO_ROOT),
                    help=f"webauto キャッシュルート (既定: {_DEFAULT_WEBAUTO_ROOT})")
    ap.add_argument("--resume", action="store_true",
                    help="runs/<uuid> に real.lite が揃っている dataset の sim 実行をスキップ")
    args = ap.parse_args()

    scenario = Path(args.scenario).resolve()
    batch_root = Path(args.batch_root).resolve()
    webauto_root = Path(args.webauto_root)

    uuids = iter_dataset_uuids(scenario)
    if not uuids:
        print(f"ERROR: scenario に Datasets がありません: {scenario}", file=sys.stderr)
        sys.exit(2)
    print(f"datasets ({len(uuids)}): {[u[:8] for u in uuids]}")

    records: list[dict] = []
    n_ok = 0
    for i, uuid in enumerate(uuids, start=1):
        print(f"\n=== [{i}/{len(uuids)}] dataset {uuid} ===", flush=True)
        output_dir = batch_root / "runs" / uuid

        if args.resume and _bundle_has_real_lite(output_dir):
            print(f"[SKIP] {uuid}: real.lite 済み (--resume)")
        else:
            try:
                t4_path = resolve_t4_dataset_path(webauto_root, uuid)
            except DatasetResolutionError as e:
                print(f"[WARN] {uuid}: dataset 解決失敗 — スキップ\n{e}", file=sys.stderr)
                records.append({"dataset_id": uuid, "status": "collect_failed",
                                "collected_at": _now()})
                continue
            scenario_single = batch_root / "scenarios" / f"{uuid}.scenario.yaml"
            write_single_dataset_scenario(scenario, uuid, scenario_single)
            ok = run_one_dataset(uuid, scenario_single, t4_path, output_dir)
            if not ok and not _bundle_has_real_lite(output_dir):
                print(f"[WARN] {uuid}: sim 実行失敗 (成果物なし) — スキップ", file=sys.stderr)
                records.append({"dataset_id": uuid, "status": "sim_failed",
                                "collected_at": _now()})
                continue
            if not ok:
                print(f"[WARN] {uuid}: launch が非ゼロ終了したが成果物あり — 収集は継続",
                      file=sys.stderr)

        try:
            bundle = _resolve_bundle(output_dir)
            rec = collect_bundle(bundle, uuid, batch_root)
        except FileNotFoundError as e:
            print(f"[WARN] {uuid}: 収集失敗 — {e}", file=sys.stderr)
            records.append({"dataset_id": uuid, "status": "collect_failed",
                            "collected_at": _now()})
            continue
        if "comparison" not in rec["linked"]:
            rec["status"] = "analysis_failed"  # real.lite はあるが解析成果物が無い
        records.append(rec)
        n_ok += 1
        print(f"[OK] {uuid}: 収集完了 ({', '.join(rec['linked'])})")

    update_manifest(batch_root, records)
    print(f"\n=== バッチ完了: 成功 {n_ok} / {len(uuids)} ===")
    if n_ok == 0:
        print("ERROR: 成功した dataset が 0 件のため横断分析をスキップ", file=sys.stderr)
        sys.exit(1)

    # batch_latest symlink (最新バッチへのショートカット)
    latest = batch_root.parent / "batch_latest"
    try:
        if latest.is_symlink() or latest.exists():
            latest.unlink()
        latest.symlink_to(batch_root.name)
    except OSError:
        pass

    print("\n=== Stage 13: 横断分析 ===", flush=True)
    r13 = subprocess.run(  # noqa: S603
        [sys.executable, "-m", f"{_PKG}.step13_cross_dataset",
         "--collection-dir", str(batch_root)],
        check=False, env=os.environ.copy(),
    )
    print("\n=== Stage 11 (マルチ DS): report.html ===", flush=True)
    r11 = subprocess.run(  # noqa: S603
        [sys.executable, "-m", f"{_PKG}.step11_build_html_report",
         "--collection-dir", str(batch_root)],
        check=False, env=os.environ.copy(),
    )
    if r13.returncode != 0 or r11.returncode != 0:
        print("[WARN] 横断分析またはレポート生成が失敗しました (per-dataset 成果物は有効)",
              file=sys.stderr)
        sys.exit(1)
    print(f"\n完了。レポート: {batch_root / 'report.html'}")


if __name__ == "__main__":
    main()
