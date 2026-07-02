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
from concurrent.futures import ProcessPoolExecutor, as_completed
import yaml
from .collect_datasets import _resolve_bundle, collect_bundle, update_manifest
from .lib._dataset import DatasetResolutionError, resolve_t4_dataset_path
from .lib._io import resolve_lite_bag

_PKG = "driving_log_replayer_v2.real_log_sim_comparison"
_DEFAULT_WEBAUTO_ROOT = Path.home() / ".webauto" / "data" / "data" / "annotation_dataset"


def _remove_path(path: Path) -> None:
    import shutil
    if path.is_symlink() or path.is_file():
        path.unlink()
    elif path.is_dir():
        shutil.rmtree(path)


def _run_and_collect_worker(args: dict) -> dict:
    uuid = args["uuid"]
    scenario = Path(args["scenario"])
    batch_root = Path(args["batch_root"])
    input_mode = args["input_mode"]
    skip_sim = args["skip_sim"]
    skip_ol = args["skip_ol"]
    resume = args["resume"]
    webauto_root = Path(args["webauto_root"])

    output_dir = batch_root / "runs" / uuid
    status = "success"

    if skip_sim:  # Autoware 不要なケース（skip_sim=True）は ros2 launch を使わず直接 Python 実行
        # Specialized direct python extraction (bypass ROS 2 launch)
        if input_mode == "raw":
            t4_path = Path(args["raw_t4_path"])
            scenario_single = batch_root / "scenarios" / f"{uuid}.scenario.yaml"
            write_raw_dataset_scenario(scenario, uuid, scenario_single)
        else:
            try:
                t4_path = resolve_t4_dataset_path(webauto_root, uuid)
            except Exception as e:
                now_str = datetime.now(timezone.utc).isoformat(timespec="seconds")
                return {"dataset_id": uuid, "status": "collect_failed", "collected_at": now_str}
            scenario_single = batch_root / "scenarios" / f"{uuid}.scenario.yaml"
            write_single_dataset_scenario(scenario, uuid, scenario_single)

        lite_dir = output_dir / "lite"
        lite_dir.mkdir(parents=True, exist_ok=True)

        existing_lite = resolve_lite_bag(t4_path, "real")
        if resume and _bundle_has_real_lite(output_dir):
            status = "skipped"
        elif existing_lite is not None:
            existing_lite = existing_lite.resolve()
            dst_lite = lite_dir / existing_lite.name
            if dst_lite.exists() or dst_lite.is_symlink():
                _remove_path(dst_lite)
            dst_lite.symlink_to(existing_lite)
            status = "skipped"
        else:
            is_writable = False
            test_file = t4_path / ".write_test"
            try:
                test_file.touch()
                test_file.unlink()
                is_writable = True
            except OSError:
                pass

            if is_writable:
                target_lite = t4_path / "real.lite"
            else:
                target_lite = lite_dir / "real.lite"

            if target_lite.exists() or target_lite.is_symlink():
                _remove_path(target_lite)

            input_bag_dir = t4_path / "input_bag"
            target_lite.parent.mkdir(parents=True, exist_ok=True)

            cmd = [
                sys.executable, "-m",
                "driving_log_replayer_v2.real_log_sim_comparison.step0_make_lite",
                "--kind", "real",
                "--input", str(input_bag_dir),
                "--output", str(target_lite),
            ]
            print(f"[DIRECT EXTRACT] {uuid[:8]}: {' '.join(cmd)}", flush=True)
            try:
                proc = subprocess.run(cmd, check=False, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                if proc.returncode != 0:
                    print(f"[ERROR] Direct extraction failed for {uuid}:\n{proc.stdout}", file=sys.stderr)
                    now_str = datetime.now(timezone.utc).isoformat(timespec="seconds")
                    return {"dataset_id": uuid, "status": "sim_failed", "collected_at": now_str}
            except Exception as e:
                print(f"[ERROR] Direct extraction failed for {uuid}: {e}", file=sys.stderr)
                now_str = datetime.now(timezone.utc).isoformat(timespec="seconds")
                return {"dataset_id": uuid, "status": "sim_failed", "collected_at": now_str}

            if is_writable:
                dst_lite = lite_dir / "real.lite"
                if dst_lite.exists() or dst_lite.is_symlink():
                    _remove_path(dst_lite)
                dst_lite.symlink_to(target_lite)

        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "dataset_id.txt").write_text(uuid, encoding="utf-8")

        # skip_ol=False のときは open-loop 解析も直接 Python で実行する (ros2 launch 不要)
        if not skip_ol and status != "skipped":
            ok = run_analysis_direct(
                uuid, scenario_single, t4_path, output_dir, skip_ol=False
            )
            if not ok:
                print(f"[WARN] run_analysis_direct failed for {uuid[:8]} (analysis stage)",
                      file=sys.stderr)

    else:  # not skip_sim: Autoware を伴う closed-loop → ros2 launch を使う
        if resume and _bundle_has_real_lite(output_dir):
            status = "skipped"
        else:
            if input_mode == "raw":
                t4_path = Path(args["raw_t4_path"])
                scenario_single = batch_root / "scenarios" / f"{uuid}.scenario.yaml"
                write_raw_dataset_scenario(scenario, uuid, scenario_single)
            else:
                try:
                    t4_path = resolve_t4_dataset_path(webauto_root, uuid)
                except Exception as e:
                    now_str = datetime.now(timezone.utc).isoformat(timespec="seconds")
                    return {"dataset_id": uuid, "status": "collect_failed", "collected_at": now_str}
                scenario_single = batch_root / "scenarios" / f"{uuid}.scenario.yaml"
                write_single_dataset_scenario(scenario, uuid, scenario_single)

            ok = run_one_dataset(
                uuid, scenario_single, t4_path, output_dir,
                skip_sim=skip_sim, skip_ol=skip_ol
            )
            if not ok and not _bundle_has_real_lite(output_dir):
                now_str = datetime.now(timezone.utc).isoformat(timespec="seconds")
                return {"dataset_id": uuid, "status": "sim_failed", "collected_at": now_str}

    try:
        bundle = _resolve_bundle(output_dir)
        rec = collect_bundle(bundle, uuid, batch_root)
        if status == "skipped":
            rec["status"] = "skipped"
    except Exception:
        now_str = datetime.now(timezone.utc).isoformat(timespec="seconds")
        return {"dataset_id": uuid, "status": "collect_failed", "collected_at": now_str}

    if "comparison" not in rec["linked"]:
        rec["status"] = "analysis_failed"

    return rec


def iter_dataset_uuids(scenario: Path) -> list[str]:
    """scenario.yaml の Evaluation.Datasets から UUID リストを取り出す (各エントリの先頭キー)。"""
    doc = yaml.safe_load(scenario.read_text(encoding="utf-8"))
    datasets = (doc.get("Evaluation") or {}).get("Datasets") or []
    uuids = []
    for entry in datasets:
        if isinstance(entry, dict) and entry:
            uuids.append(str(next(iter(entry.keys()))))
    return uuids


def write_raw_dataset_scenario(scenario: Path, uuid: str, out_path: Path) -> Path:
    """raw モード用 single-dataset scenario 生成。

    Datasets を uuid 1件に置換する (t4_dataset_path/t4_dataset_id は launch 引数で渡すため
    Datasets エントリの内容は空で問題ないが、VehicleId は必要なのでテンプレートから引き継ぐ)。
    cases / models / sim_runs 等の Conditions はテンプレート scenario の値をそのまま引き継ぐ。
    """
    doc = yaml.safe_load(scenario.read_text(encoding="utf-8"))
    vehicle_id = "default"
    datasets = (doc.get("Evaluation") or {}).get("Datasets") or []
    if datasets and isinstance(datasets[0], dict):
        first_val = next(iter(datasets[0].values()), {})
        if isinstance(first_val, dict):
            vehicle_id = first_val.get("VehicleId", "default")

    doc["Evaluation"]["Datasets"] = [{uuid: {"VehicleId": vehicle_id}}]
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    return out_path


def _discover_raw_datasets(batch_root: Path) -> list[tuple[str, Path]]:
    """raw モード: batch_root/datasets/ 配下の (uuid, t4_path) を昇順で返す。"""
    ds_root = batch_root / "datasets"
    if not ds_root.is_dir():
        return []
    return sorted(
        [(sub.name, sub.resolve()) for sub in ds_root.iterdir() if sub.is_dir()]
    )


def write_single_dataset_scenario(scenario: Path, uuid: str, out_path: Path) -> Path:
    """Datasets を当該 UUID 1 件に絞った scenario を生成する。

    launch には t4_dataset_path/t4_dataset_id を明示で渡すが、scenario 側の Datasets も
    1 件に揃えておくことで「scenario には複数 dataset・実行は 1 dataset」という不整合を
    下流 (evaluator / step2) に持ち込まない。models / cases / sim_runs は scenario.yaml に
    インライン化されているため相対パス変換は不要。
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


def run_analysis_direct(
    uuid: str, scenario_single: Path, t4_dataset_path: Path, output_dir: Path,
    *, skip_ol: bool = False,
) -> bool:
    """ros2 launch を使わず直接 Python で解析ステージを実行する (skip_sim=True のとき用)。

    evaluator_node.run_analysis を直接呼び出すことで ROS 2 の初期化オーバーヘッドを排除する。
    lite_dir = output_dir/lite (直接抽出パスと同じレイアウト)。
    """
    import logging
    from driving_log_replayer_v2.real_log_sim_comparison.evaluator_node import (
        build_common_env,
        run_analysis,
        _load_compare_config,
    )

    lite_dir = output_dir / "lite"
    comparison_dir = output_dir / "comparison"
    comparison_dir.mkdir(parents=True, exist_ok=True)

    compare_cfg = _load_compare_config(str(scenario_single))

    logger = logging.getLogger(f"run_batch.{uuid[:8]}")
    if not logger.handlers:
        logging.basicConfig(level=logging.INFO, format="[%(name)s] %(levelname)s %(message)s")

    env = build_common_env(comparison_dir, "", compare_cfg, logger)
    env["SKIP_SIM"] = "1"
    if skip_ol:
        env["SKIP_OL"] = "1"

    label = f"{uuid[:8]}{' (skip-ol)' if skip_ol else ''}"
    print(f"[DIRECT ANALYSIS] {label}: run_analysis in-process", flush=True)
    try:
        run_analysis(lite_dir, comparison_dir, env, compare_cfg, logger)
        return True
    except Exception as e:
        print(f"[ERROR] run_analysis_direct failed for {uuid}: {e}", file=sys.stderr)
        return False


def run_one_dataset(
    uuid: str, scenario_single: Path, t4_dataset_path: Path, output_dir: Path,
    *, skip_sim: bool = False, skip_ol: bool = False,
) -> bool:
    """1 dataset の per-dataset パイプラインを ros2 launch で実行する (成功で True)。

    Makefile local_cloud_run の launch 行と同一。ROS 環境 (install/setup.bash) は
    呼び出し元 shell で source 済みであることを前提とする。
    Autoware を伴う closed-loop (skip_sim=False) のときのみ呼ばれる。
    skip_ol=True で SKIP_OL=1 を伝搬し、オープンループ解析も省略する。
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
    env = os.environ.copy()
    if skip_sim:
        env["SKIP_SIM"] = "1"
    if skip_ol:
        env["SKIP_OL"] = "1"
    print(f"[RUN] {uuid}{' (skip-sim)' if skip_sim else ''}{' (skip-ol)' if skip_ol else ''}: {' '.join(cmd)}", flush=True)
    proc = subprocess.run(cmd, check=False, env=env)  # noqa: S603
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
    ap.add_argument("--skip-sim", action="store_true",
                    help="全 dataset で Stage CL2 (closed-loop sim 実行) を省略し、実機ログのみの"
                    "解析 (open-loop N-step / sweep / カバレッジ) を実行する")
    ap.add_argument("--skip-ol", action="store_true",
                    help="全 dataset でオープンループ解析 (Stage OL1/OL2/OL3 等) を省略し、"
                    "実機ログの抽出 (Stage 0) とシナリオ生成 (Stage CL1) のみを実行する")
    ap.add_argument("--closed-loop-uuids", default="",
                    help="クローズドループシミュレーションを実行するデータセットUUID（カンマ区切り）。"
                    "指定された場合、これらのUUIDのみclosed-loopを実行し、それ以外はオープンループ解析のみ（skip-sim）とします。")
    ap.add_argument("--input-mode", choices=["annotation", "raw"], default="annotation",
                    help="dataset 解決モード。"
                    "annotation: webauto キャッシュ (annotation_dataset) から解決 (既定・クラウド互換)。"
                    "raw: collect_raw_rosbags.py で事前構築した <batch-root>/datasets/<uuid>/ を"
                    "そのまま t4_dataset_path として使う (annotation_dataset 不要)。"
                    "raw モードでは Datasets は --batch-root/datasets/ を自動検出し、"
                    "aggregate_report.html として出力する")
    ap.add_argument("--jobs", type=int, default=min(4, os.cpu_count() or 1),
                    help="並列ワーカー数 (既定: コア数または4の小さい方)")
    args = ap.parse_args()

    scenario = Path(args.scenario).resolve()
    batch_root = Path(args.batch_root).resolve()
    webauto_root = Path(args.webauto_root)

    if args.input_mode == "raw":
        raw_pairs = _discover_raw_datasets(batch_root)
        if not raw_pairs:
            print(
                f"ERROR: raw モード: {batch_root}/datasets/ にデータセットが見つかりません。\n"
                "先に collect_raw_rosbags.py で収集してください",
                file=sys.stderr,
            )
            sys.exit(2)
        uuids = [u for u, _ in raw_pairs]
        raw_t4_paths = {u: p for u, p in raw_pairs}
    else:
        uuids = iter_dataset_uuids(scenario)
        raw_t4_paths = {}

    if not uuids:
        print(f"ERROR: データセットが 0 件 (scenario={scenario})", file=sys.stderr)
        sys.exit(2)
    print(f"datasets ({len(uuids)}): {[u[:8] for u in uuids]}")

    closed_loop_uuids = set()
    if args.closed_loop_uuids:
        closed_loop_uuids = {u.strip() for u in args.closed_loop_uuids.split(",") if u.strip()}

    tasks_args = []
    for uuid in uuids:
        if closed_loop_uuids:
            skip_sim_for_this = uuid not in closed_loop_uuids
        else:
            skip_sim_for_this = args.skip_sim
        t4_path_str = str(raw_t4_paths[uuid]) if args.input_mode == "raw" else ""
        tasks_args.append({
            "uuid": uuid,
            "scenario": str(scenario),
            "batch_root": str(batch_root),
            "input_mode": args.input_mode,
            "skip_sim": skip_sim_for_this,
            "skip_ol": args.skip_ol,
            "resume": args.resume,
            "webauto_root": str(webauto_root),
            "raw_t4_path": t4_path_str,
        })

    records: list[dict] = []
    n_ok = 0
    n_total = len(uuids)
    print(f"\n[INFO] 並列実行開始 (ワーカー数: {args.jobs})...", flush=True)
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        futs = {pool.submit(_run_and_collect_worker, a): a["uuid"] for a in tasks_args}
        for i, fut in enumerate(as_completed(futs), start=1):
            uuid = futs[fut]
            try:
                rec = fut.result()
                records.append(rec)
                if rec.get("status") in ["success", "skipped", "analysis_failed"]:
                    n_ok += 1
                    status_label = "SKIP" if rec.get("status") == "skipped" else "OK"
                    print(f"[{i}/{n_total}] {status_label}: {uuid[:8]} 収集完了 ({', '.join(rec.get('linked', []))})", flush=True)
                else:
                    print(f"[{i}/{n_total}] FAILED: {uuid[:8]} (status={rec.get('status')})", flush=True)
            except Exception as e:
                print(f"[{i}/{n_total}] ERROR: {uuid[:8]} ({e})", flush=True)
                now_str = datetime.now(timezone.utc).isoformat(timespec="seconds")
                records.append({"dataset_id": uuid, "status": "collect_failed", "collected_at": now_str})

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

    if args.skip_sim and args.skip_ol:
        print("\n[INFO] skip_sim と skip_ol が両方指定されているため（データ抽出のみ）、横断分析と HTML レポート生成をスキップします。", flush=True)
        print(f"\n完了。成果物は {batch_root} に収集されました。")
        return

    print("\n=== Stage Cross Dataset: 横断分析 ===", flush=True)
    r13 = subprocess.run(  # noqa: S603
        [sys.executable, "-m", f"{_PKG}.step_cross_dataset",
         "--collection-dir", str(batch_root), "--scenario", str(scenario)],
        check=False, env=os.environ.copy(),
    )
    report_name = "aggregate_report.html" if args.input_mode == "raw" else "report.html"
    print(f"\n=== Stage Report HTML (マルチ DS): {report_name} ===", flush=True)
    r11_cmd = [
        sys.executable, "-m", f"{_PKG}.step_report_html",
        "--collection-dir", str(batch_root),
        "--report-name", report_name,
        "--no-embed-viewers",
    ]
    r11 = subprocess.run(r11_cmd, check=False, env=os.environ.copy())  # noqa: S603
    if r13.returncode != 0 or r11.returncode != 0:
        print("[WARN] 横断分析またはレポート生成が失敗しました (per-dataset 成果物は有効)",
              file=sys.stderr)
        sys.exit(1)
    print(f"\n完了。レポート: {batch_root / report_name}")


if __name__ == "__main__":
    main()
