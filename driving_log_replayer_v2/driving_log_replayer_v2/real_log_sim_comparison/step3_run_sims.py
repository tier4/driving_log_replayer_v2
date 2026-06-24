"""Stage 3: 1 sim run の実行 (scenario_test_runner.launch.py 起動 + step1_make_lite --kind sim).

scenario.yaml の Conditions.sim_runs の tag に対応する設定を読み、ros2 launch を
subprocess で起動し、出力 MCAP を step1_make_lite で lite 化する。

Usage:
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.step3_run_sims \\
        --run-tag normal \\
        --scenario <auto_scenario.yaml> \\
        --config-scenario <scenario.yaml> \\
        --output-lite <lite/normal.lite>

注意:
- nested ros2 launch (外側 evaluator_node の中で本ツールが内側 scenario_test_runner)
  になる場合、呼び出し側 (evaluator_node) で ROS_DOMAIN_ID を切り替えて DDS 競合回避。
- scenario_test_runner.launch.py の output_directory:= 引数が効かない可能性に備え、
  /tmp/scenario_test_runner/ もフォールバック検索する。
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


# scenario_test_runner.launch.py が output_directory 引数を無視する場合の既定出力先
_FALLBACK_SIM_OUT_ROOT = Path("/tmp/scenario_test_runner")


def _resolve_replay(reproduce_bag: str, scenario: Path,
                    ego_replay_duration: float, position_based: bool) -> dict | None:
    """reproduce_bag から scenario_test_runner の replay launch 引数素材を解決する.

    返り値 dict: {bag, replay_start_time, ego_replay_duration, position_based}。
    - bag: input_bag ディレクトリそのもの (mcap/db3 とも metadata.yaml から rosbag2 が
      storage を自動判別する。PerceptionReproducerSensor / EgoBagReplayer 双方が直接読む)
    - replay_start_time: scenario time 0 に対応する bag 相対秒
      (= t_anchor − bag metadata starting_time)。t_anchor は step2 のサイドカー
      <scenario>.replay.json から読み、無ければ bag から AUTONOMOUS 開始を再計算する。
    """
    if not reproduce_bag:
        return None
    bag = Path(reproduce_bag)
    if not bag.exists():
        print(f"[step3_run_sims] WARN: reproduce_bag が見つかりません: {bag} "
              "(perception/ego replay をスキップ)", file=sys.stderr)
        return None

    # t_anchor: step2 サイドカー優先、無ければ再計算 (replay.json は step2 と同時生成のため
    # 通常存在する。フォールバックは scenario を手書きした場合等の保険)。
    replay_json = scenario.with_suffix(".replay.json")
    if replay_json.exists():
        t_anchor_ns = int(json.loads(replay_json.read_text())["t_anchor_ns"])
    else:
        from .lib._events import find_autonomous_start  # noqa: PLC0415
        from .lib._io import load_operation_mode, load_velocity  # noqa: PLC0415
        print(f"[step3_run_sims] WARN: {replay_json} が無いため AUTONOMOUS 開始を再計算",
              file=sys.stderr)
        t_anchor_ns = find_autonomous_start(load_operation_mode(bag), load_velocity(bag))

    import rosbag2_py  # noqa: PLC0415

    from .lib._io import _detect_storage_id  # noqa: PLC0415
    metadata = rosbag2_py.Info().read_metadata(str(bag), _detect_storage_id(bag))
    bag_start_ns = int(metadata.starting_time.nanoseconds)
    replay_start_time = (t_anchor_ns - bag_start_ns) / 1e9
    if replay_start_time < 0.0:
        print(f"[step3_run_sims] WARN: replay_start_time={replay_start_time:.3f}s < 0 "
              "(t_anchor が bag 開始より前)。0 に丸めます", file=sys.stderr)
        replay_start_time = 0.0
    return {
        "bag": bag,
        "replay_start_time": replay_start_time,
        "ego_replay_duration": ego_replay_duration,
        "position_based": position_based,
    }


def _build_launch_cmd(
    run, scenario: Path, output_directory: Path, replay: dict | None = None
) -> list[str]:
    """scenario_test_runner.launch.py の ros2 launch 起動コマンドを組み立てる.

    注: rviz の起動は Autoware (planning_simulator.launch.xml) 側のメカニズムで、
    scenario_test_runner.launch.py の launch_rviz 引数とは独立に制御される。
    ここでは追加引数を渡さず、Autoware の default 動作に任せる。
    """
    # scenario_test_runner の global_timeout は scenario 実行の **壁時計** 上限 (既定 180s)。
    # 本ユースケースの sim は実機の約 3 倍遅いため 180s ではゴール到達前に打ち切られる
    # (2026-06-03 判明: 全 sim が 180s 壁で強制終了し、scenario 側の exitSuccess が効く前に
    # 切られていた)。実終了は scenario の ReachPosition exitSuccess に任せ、global_timeout は
    # 外側の壁セーフティとして余裕を持たせる: 起動 (initialize_duration) と subprocess kill
    # (timeout_s) の間に収める。
    global_timeout = max(180, run.timeout_s - run.initialize_duration - 60)
    cmd = [
        "ros2", "launch", "scenario_test_runner", "scenario_test_runner.launch.py",
        f"vehicle_model:={run.vehicle_model}",
        f"sensor_model:={run.sensor_model}",
        "record:=true",
        "record_storage_id:=mcap",
        f"architecture_type:={run.architecture_type}",
        f"initialize_duration:={run.initialize_duration}",
        f"global_timeout:={global_timeout}",
        f"scenario:={scenario}",
        f"output_directory:={output_directory}",
    ]
    if run.godot_executable:
        cmd.append(f"godot_executable:={run.godot_executable}")
    if run.carla_path:
        cmd.append(f"carla_path:={run.carla_path}")
    if replay is not None:
        # replay_bag_path / replay_start_time / replay_ego_duration は launch の
        # make_parameters() 経由で simple_sensor_simulator (perception 再生) と
        # openscenario_interpreter (EgoBagReplayer) の両ノードに同じ値が渡る。
        cmd += [
            f"replay_bag_path:={replay['bag']}",
            f"replay_start_time:={replay['replay_start_time']}",
            f"replay_ego_duration:={replay['ego_replay_duration']}",
        ]
        if replay["position_based"]:
            cmd.append("simple_sensor_simulator.replay_use_position_based:=true")
    # params: simulator_model パラメータ上書き。
    # scenario_test_runner.launch.py が simple_sensor_simulator. 接頭辞の launch 引数を
    # 収集し、simulator_model.param.yaml の後ろに連結する (後勝ち) ため、
    # description パッケージを増やさずに dynamics 変種 (例 k_us) を作れる。
    for key, value in run.params.items():
        cmd.append(f"simple_sensor_simulator.{key}:={value}")
    return cmd


def _setup_dp_model_env(dp_model_dir: str | None) -> None:
    """dp_model_dir があれば DiffusionPlanner モデルパスの env を os.environ に設定する.

    autoware_launch の diffusion_planner.param.yaml は onnx_model_path / args_path を
    `$(env DIFFUSION_PLANNER_ONNX_PATH ...)` 置換で参照する (allow_substs=true)。
    ここで run ごとに env を上書きすることで、正規パスへのファイルコピーなしに DP モデルを
    切り替える。DP_ONNX_PATH も同 onnx に揃え、provenance が使用モデルを記録できるようにする。

    step3_run_sims は 1 run-tag = 1 プロセスのため os.environ を直接設定してよい。これにより
    (1) launch subprocess が継承し、(2) 同プロセスの provenance capture も同じ onnx を解決する。

    dp_model_dir は明示指定 (sim_runs.yaml の dp_model_dir) か自動 pull の解決結果。
    """
    if not dp_model_dir:
        return
    model_dir = Path(dp_model_dir)
    onnx = model_dir / "diffusion_planner.onnx"
    args_json = model_dir / "args.json"
    # モデルファイル欠落時は既定モデルへ黙ってフォールバックさせず loud に落とす
    # (欠落のまま進むと provenance が既定モデルを記録し「別モデルのつもりが既定」を取り逃す)。
    if not onnx.exists():
        raise RuntimeError(
            f"DP モデルが見つかりません: {onnx} "
            "(dir には diffusion_planner.onnx と args.json が必要)"
        )
    # 変更 A (param yaml の $(env) 化) を *インストール済* autoware_launch に実行時適用する。
    # 未適用 (cloud で autoware.repos 未更新等) だと Autoware は既定モデルをロードし続けるのに
    # env と provenance は dp_model_dir を指す → 「別モデルなのに同一挙動」という黙った誤りになる。
    # ここで DLR 側が param yaml に $(env ...) を注入することで launcher/autoware.repos を触らず
    # env 切り替えを有効化する (冪等・挙動不変)。書き込み不可なら loud に落とす。
    _ensure_installed_param_honors_env()
    os.environ["DIFFUSION_PLANNER_ONNX_PATH"] = str(onnx)
    os.environ["DIFFUSION_PLANNER_ARGS_PATH"] = str(args_json)
    os.environ["DP_ONNX_PATH"] = str(onnx)  # provenance の解決元と揃える
    print(f"[step3_run_sims] DP model dir: {model_dir}", flush=True)


# 変更 A の判定/注入に使う、param yaml が持つべき env 置換のマーカー。
_DP_ENV_SUBST_MARKER = "$(env DIFFUSION_PLANNER_ONNX_PATH"
# 注入対象キーと対応する環境変数名。
_DP_PARAM_ENV_KEYS = (
    ("onnx_model_path", "DIFFUSION_PLANNER_ONNX_PATH"),
    ("args_path", "DIFFUSION_PLANNER_ARGS_PATH"),
)


def _inject_env_subst(text: str) -> str:
    """param yaml の onnx_model_path / args_path 行を `$(env <VAR> <既存値>)` 形に書き換える.

    既に `$(env` を含む行は触らない。env 未設定時は <既存値> にフォールバックするため挙動不変。
    """
    for key, envvar in _DP_PARAM_ENV_KEYS:
        pat = re.compile(rf"^(?P<indent>\s*){re.escape(key)}:[ \t]*(?P<val>\S.*?)[ \t]*$", re.M)

        def _repl(m: re.Match, _envvar: str = envvar, _key: str = key) -> str:
            val = m.group("val")
            if val.startswith("$(env"):
                return m.group(0)
            return f"{m.group('indent')}{_key}: $(env {_envvar} {val})"

        text = pat.sub(_repl, text)
    return text


def _ensure_installed_param_honors_env() -> None:
    """インストール済 autoware_launch の diffusion_planner.param.yaml に変更 A を実行時適用する.

    dp_model_dir による env 切り替えは、build_only と scenario の両方が読む *インストール済*
    param yaml が onnx_model_path を `$(env DIFFUSION_PLANNER_ONNX_PATH ...)` で参照して初めて効く。
    本関数は未適用なら DLR 側で `$(env ...)` を注入する (launcher サブリポジトリ / autoware.repos を
    触らずに有効化)。注入は冪等で env 未設定時は元の既定値にフォールバックするため挙動不変、restore
    不要。書き込み不可 (install が read-only) の場合のみ loud に落とす (黙った既定モデル実行を防止)。
    """
    param = _autoware_launch_dp_param_path()
    if param is None:
        raise RuntimeError(
            "autoware_launch の diffusion_planner.param.yaml を解決できません "
            "(dp_model_dir 指定には autoware_launch のインストールが必要)"
        )
    p = Path(param)
    text = p.read_text(encoding="utf-8", errors="replace")
    if _DP_ENV_SUBST_MARKER in text:
        return  # 既に適用済 (変更 A 反映済 or 前 run で注入済)
    new_text = _inject_env_subst(text)
    if _DP_ENV_SUBST_MARKER not in new_text:
        raise RuntimeError(
            f"{param} に onnx_model_path 行が見つからず $(env ...) を注入できません "
            "(param yaml の書式を確認してください)"
        )
    try:
        p.write_text(new_text, encoding="utf-8")
    except OSError as e:
        raise RuntimeError(
            f"インストール済 {param} に $(env ...) を書き込めません ({e})。install が read-only の "
            "可能性。代替として launcher サブリポジトリに変更 A をコミットし autoware.repos の version を "
            "更新してください。このままでは dp_model_dir が黙って無視され既定モデルがロードされます。"
        ) from e
    print(f"[step3_run_sims] 変更 A を実行時適用 (DLR→install param に $(env ...) 注入): {param}",
          flush=True)


# TensorRT エンジンビルドは数分かかり得るため余裕を持たせる。
_DP_ENGINE_BUILD_TIMEOUT = 1200


def _autoware_launch_dp_param_path() -> str | None:
    """autoware_launch の diffusion_planner.param.yaml の絶対パスを解決する.

    build_only launch に diffusion_planner_param_path として渡す
    (.webauto-ci.yml の real_log_sim_comparison pre_task と同じパス)。
    """
    try:
        out = subprocess.run(
            ["ros2", "pkg", "prefix", "autoware_launch", "--share"],
            capture_output=True, text=True, timeout=30, check=False,
        )
    except Exception:  # noqa: BLE001
        return None
    if out.returncode != 0 or not out.stdout.strip():
        return None
    p = (Path(out.stdout.strip())
         / "config/planning/neural_net_planner/diffusion_planner.param.yaml")
    return str(p) if p.exists() else None


def _prebuild_dp_engine(dp_model_dir: str | None) -> None:
    """dp_model_dir の TensorRT エンジンを build_only で scenario 起動前に生成する.

    env でモデルを切り替えると各モデル初回ロード時にエンジンが遅延ビルドされ、計時付きの
    scenario 実行中だとゴール到達前にタイムアウトし得る。ここで scenario 起動前に build_only
    launch を一度走らせ、エンジンを model dir に生成しておく (エンジンは onnx と同ディレクトリに
    `<onnx名>_batch<N>_fp32.engine` でキャッシュされる: tensorrt_inference.cpp:141-145。dir-per-model
    配置なら隔離される)。既にあれば build_only は高速ロードして終了する (冪等)。

    sim_runs.yaml の dp_model_dir / dp_model_release が SSOT となり、.webauto-ci.yml にモデルごとの
    pre_task を複製する必要がない。未指定なら何もしない (既定モデルは既存の deploy/pre_task に従う)。
    呼び出し前に _setup_dp_model_env() が env を設定済みであること (subprocess が継承する)。
    """
    if not dp_model_dir:
        return
    param = _autoware_launch_dp_param_path()
    if param is None:
        print("[step3_run_sims] WARN: autoware_launch の diffusion_planner.param.yaml を解決できず "
              "エンジン事前ビルドをスキップ (scenario 実行中に遅延ビルドされ得る)", file=sys.stderr)
        return
    cmd = [
        "ros2", "launch", "autoware_diffusion_planner", "diffusion_planner.launch.xml",
        "build_only:=true",
        f"diffusion_planner_param_path:={param}",
    ]
    print(f"[step3_run_sims] DP engine 事前ビルド: {dp_model_dir}", flush=True)
    _run_subprocess(cmd, timeout=_DP_ENGINE_BUILD_TIMEOUT)


# 自動 pull の既定 Web.Auto プロジェクト (env WEBAUTO_PROJECT_ID で上書き可)。
_DEFAULT_WEBAUTO_PROJECT = "x2_dev"
# webauto CLI のタイムアウト [s] (pull は数百 MB を落とすため余裕を持たせる)。
_WEBAUTO_CLI_TIMEOUT = 1800


def _webauto_json(args: list[str]) -> dict:
    """webauto CLI を --output json で実行し dict を返す (失敗時 RuntimeError)."""
    cmd = ["webauto", *args, "--output", "json"]
    print(f"[step3_run_sims] $ {' '.join(cmd)}", flush=True)
    try:
        out = subprocess.run(
            cmd, capture_output=True, text=True,
            timeout=_WEBAUTO_CLI_TIMEOUT, check=False,
        )
    except subprocess.TimeoutExpired as e:
        raise RuntimeError(f"webauto timeout: {' '.join(cmd)}") from e
    if out.returncode != 0:
        raise RuntimeError(
            f"webauto 失敗 (rc={out.returncode}): {' '.join(cmd)}\n{out.stderr.strip()}"
        )
    # pull 等は JSON の前に進捗テキスト ("---" 等) を stdout に混ぜるため、
    # 最初の JSON 開始文字以降だけを解析する。
    stdout = out.stdout
    start = min((i for i in (stdout.find("{"), stdout.find("[")) if i >= 0), default=-1)
    try:
        return json.loads(stdout[start:]) if start >= 0 else json.loads(stdout)
    except json.JSONDecodeError as e:
        raise RuntimeError(
            f"webauto 出力の JSON 解析に失敗: {' '.join(cmd)}\n{out.stdout[:500]}"
        ) from e


def _dp_pull_root() -> Path:
    """自動 pull したモデルのキャッシュ root (env DP_MODEL_PULL_DIR で上書き可)."""
    root = os.environ.get("DP_MODEL_PULL_DIR")
    if root:
        return Path(os.path.expandvars(os.path.expanduser(root)))
    return Path.home() / ".webauto" / "data" / "ml" / "package-release"


def _resolve_dp_model_dir(run) -> str | None:
    """run の DP モデル dir を解決する (dp_model_release 指定時は webauto から自動 pull).

    - dp_model_release: webauto ml package-release search→pull し onnx を含む dir を返す (冪等)。
    - dp_model_dir: そのまま返す。
    - どちらも無し: None。
    """
    if not run.dp_model_release:
        return run.dp_model_dir

    project_id = os.environ.get("WEBAUTO_PROJECT_ID", _DEFAULT_WEBAUTO_PROJECT)
    package = run.dp_model_package
    release = run.dp_model_release

    # 名前 → (package_id, release_id) 解決
    data = _webauto_json([
        "ml", "package-release", "search",
        "--project-id", project_id,
        "--package-name", package,
        "--package-release-name", release,
    ])
    exact = [r for r in (data.get("releases") or []) if r.get("name") == release]
    if not exact:
        raise RuntimeError(
            f"ML package-release が見つかりません: package={package!r} release={release!r} "
            f"(project={project_id})。webauto 認証と名称を確認してください。"
        )
    package_id = exact[0]["package_id"]
    release_id = exact[0]["id"]

    root = _dp_pull_root()
    # 既取得チェック (release-id でキャッシュ。再 pull 不要・エンジンキャッシュも再利用)。
    release_dir = root / release_id
    if release_dir.exists():
        found = list(release_dir.glob("*/diffusion_planner.onnx")) \
            + list(release_dir.glob("diffusion_planner.onnx"))
        if found:
            print(f"[step3_run_sims] DP model 既取得を再利用: {found[0].parent}", flush=True)
            return str(found[0].parent)

    root.mkdir(parents=True, exist_ok=True)
    pulled = _webauto_json([
        "ml", "package-release", "pull",
        "--project-id", project_id,
        "--package-id", package_id,
        "--package-release-id", release_id,
        "--target-dir", str(root),
    ])
    rel_path = Path(pulled.get("package_release_path") or release_dir)
    found = list(rel_path.glob("*/diffusion_planner.onnx")) \
        + list(rel_path.glob("diffusion_planner.onnx"))
    if not found:
        raise RuntimeError(
            f"pull したが diffusion_planner.onnx が見つかりません: {rel_path}"
        )
    print(f"[step3_run_sims] DP model pull 完了: {found[0].parent}", flush=True)
    return str(found[0].parent)


def _find_output_mcap(search_dirs: list[Path]) -> Path | None:
    """指定ディレクトリ群から最新の *.mcap を 1 つ返す."""
    candidates: list[Path] = []
    for d in search_dirs:
        if d.exists():
            candidates.extend(d.rglob("*.mcap"))
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)


def _run_subprocess(cmd: list[str], timeout: int, env: dict | None = None) -> None:
    """subprocess.run の薄いラッパ. 非 0 終了で RuntimeError."""
    print(f"[step3_run_sims] $ {' '.join(cmd)}", flush=True)
    try:
        result = subprocess.run(cmd, check=False, env=env, timeout=timeout)
    except subprocess.TimeoutExpired as e:
        raise RuntimeError(f"timeout {timeout}s: {' '.join(cmd)}") from e
    if result.returncode != 0:
        raise RuntimeError(
            f"command failed (rc={result.returncode}): {' '.join(cmd)}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Stage 3: 1 sim run の実行")
    parser.add_argument("--run-tag", required=True,
                        default=os.environ.get("SIM_RUN_TAG"),
                        help="Conditions.sim_runs の対象 tag (env: SIM_RUN_TAG)")
    parser.add_argument("--scenario", required=True,
                        help="OpenSCENARIO yaml パス (Stage 2 で生成)")
    parser.add_argument("--config-scenario", required=True,
                        default=os.environ.get("SCENARIO_CONFIG_YAML"),
                        help="scenario.yaml パス (Conditions.models / sim_runs を含む; "
                             "env: SCENARIO_CONFIG_YAML)")
    parser.add_argument("--output-lite", required=True,
                        help="出力先 lite/<tag>.lite/ ディレクトリ")
    parser.add_argument("--reproduce-bag",
                        default=os.environ.get("REPRODUCE_BAG", ""),
                        help="指定すると実機 input_bag の perception (検出物体・信号・占有格子) を "
                             "simple_sensor_simulator 内蔵の PerceptionReproducerSensor で sim 時刻"
                             "同期再生する (env: REPRODUCE_BAG)。空なら再生しない。")
    parser.add_argument("--ego-replay-duration", type=float,
                        default=float(os.environ.get("EGO_REPLAY_DURATION", "0") or "0"),
                        help="scenario 開始から何秒間 ego 状態 (pose/twist/accel) を実機 bag から"
                             "注入してから closed-loop に切替えるか [s] (env: EGO_REPLAY_DURATION, "
                             "既定 0=無効)。rosbag 開始時に ego が速度を持つケースの初期状態合わせ。"
                             "reproduce_bag の指定が必要。")
    parser.add_argument("--replay-position-based", action="store_true",
                        default=os.environ.get("REPLAY_POSITION_BASED", "") == "1",
                        help="perception 再生を時刻同期ではなく sim ego 最近傍時刻のスナップショット"
                             "再生にする (env: REPLAY_POSITION_BASED=1)。closed-loop 逸脱後の"
                             "物体ずれ緩和用。")
    args = parser.parse_args()

    scenario = Path(args.scenario)
    output_lite = Path(args.output_lite)
    output_lite.parent.mkdir(parents=True, exist_ok=True)

    if not scenario.exists():
        print(f"ERROR: scenario yaml が見つかりません: {scenario}", file=sys.stderr)
        sys.exit(1)

    from .lib._sim_runs_config import load_sim_runs_config  # noqa: PLC0415
    try:
        sim_cfg = load_sim_runs_config(args.config_scenario)
        run = sim_cfg.find_run(args.run_tag)
    except (FileNotFoundError, ValueError, KeyError) as e:
        print(f"ERROR: scenario.yaml (Conditions.sim_runs): {e}", file=sys.stderr)
        sys.exit(2)

    print(f"[step3_run_sims] tag={run.tag}, vehicle_model={run.vehicle_model}, "
          f"sensor_model={run.sensor_model}, timeout={run.timeout_s}s")

    # 一時 output_directory を作る (隔離 + 衝突回避)
    tmp_root = Path(tempfile.mkdtemp(
        prefix=f"sim_runner_{run.tag}_", dir="/tmp"))
    print(f"[step3_run_sims] tmp output_directory: {tmp_root}")

    try:
        # perception/ego replay の launch 引数素材を解決 (ssv2 内蔵の
        # PerceptionReproducerSensor + EgoBagReplayer に渡す)。
        replay = _resolve_replay(
            args.reproduce_bag, scenario.resolve(),
            args.ego_replay_duration, args.replay_position_based,
        )
        if replay is None and args.ego_replay_duration > 0.0:
            print("[step3_run_sims] WARN: ego_replay_duration 指定がありますが reproduce_bag が"
                  "解決できないため ego replay は無効です", file=sys.stderr)
        if replay is not None:
            print(f"[step3_run_sims] replay: bag={replay['bag']}, "
                  f"replay_start_time={replay['replay_start_time']:.3f}s, "
                  f"ego_replay_duration={replay['ego_replay_duration']}s, "
                  f"position_based={replay['position_based']}")

        # DP モデルを解決 (dp_model_release 指定時は webauto から自動 pull)。
        dp_model_dir = _resolve_dp_model_dir(run)
        # 解決した DP モデルを os.environ に反映してから launch (subprocess が継承)。
        _setup_dp_model_env(dp_model_dir)
        # DP モデル指定時は scenario 起動前に TensorRT エンジンを事前ビルド
        # (遅延ビルドによる scenario タイムアウト回避。sim_runs.yaml 駆動)。
        _prebuild_dp_engine(dp_model_dir)
        cmd = _build_launch_cmd(run, scenario.resolve(), tmp_root, replay)
        _run_subprocess(cmd, timeout=run.timeout_s)

        # output_directory が効かない可能性に備え両方検索
        mcap = _find_output_mcap([tmp_root, _FALLBACK_SIM_OUT_ROOT])
        if mcap is None:
            raise RuntimeError(
                f"sim 出力 *.mcap が {tmp_root} / {_FALLBACK_SIM_OUT_ROOT} に見つかりません"
            )
        print(f"[step3_run_sims] sim output mcap: {mcap}")

        # step1_make_lite --kind sim で lite 化
        make_lite_cmd = [
            sys.executable, "-m",
            "driving_log_replayer_v2.real_log_sim_comparison.step1_make_lite",
            "--kind", "sim",
            "--input", str(mcap),
            "--output", str(output_lite),
        ]
        _run_subprocess(make_lite_cmd, timeout=300)

        # この sim run が使った DP モデル重み / autoware バージョンを記録 (版差の解釈用)。
        # capture は sim 実行直後 (= 使用した onnx がまだ /opt にある状態) に行う。
        from .lib._provenance import write_provenance  # noqa: PLC0415
        prov = write_provenance(output_lite, extra={
            "tag": run.tag,
            "vehicle_model": run.vehicle_model,
            "architecture_type": run.architecture_type,
            "dp_model_dir": dp_model_dir,
            "dp_model_release": run.dp_model_release,
            "params": run.params,
            # 注入区間はメトリクスが実機と「一致して当然」のため、解析側が切替時刻を
            # 参照できるよう記録する (replay 無効時は 0)。
            "ego_replay_duration": replay["ego_replay_duration"] if replay else 0.0,
            "replay_start_time": replay["replay_start_time"] if replay else None,
        })
        print(f"[step3_run_sims] provenance: DP={prov.get('dp_exp_name')} "
              f"(onnx {prov.get('dp_onnx_sha8')}) / autoware {prov.get('autoware_version')}")

        print(f"[step3_run_sims] Saved: {output_lite}")

    finally:
        shutil.rmtree(tmp_root, ignore_errors=True)
        # フォールバック先もケース毎にクリア (次 run 干渉防止)
        shutil.rmtree(_FALLBACK_SIM_OUT_ROOT, ignore_errors=True)


if __name__ == "__main__":
    main()
