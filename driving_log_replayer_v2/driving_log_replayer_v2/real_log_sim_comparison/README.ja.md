# Real Log / Sim Comparison の評価

実機走行ログ（T4 dataset の `input_bag/` rosbag）を **SSOT(Single Source of Truth)** とし、同じ走行を
Autoware + シミュレータで closed-loop 再現して、実機との乖離を統計値と図で比較する
ユースケース。速度応答・操舵応答・軌跡精度・N-step オープンループ誤差を可視化する。

他ユースケースと異なり、launch から **Autoware・bag player・bag recorder のいずれも
起動しない**。評価ノード `real_log_sim_comparison_evaluator` がパイプライン
(Stage 0〜Report HTML) を subprocess で直接実行する。複数データセットを束ねた横断分析
(Stage Cross Dataset) とマルチデータセットレポートは collection 単位で実行する
(後述「マルチデータセット評価」)。

## パイプライン（9 段階）

`sample/scenario.yaml` の `Datasets[0]` UUID = SSOT（実機 rosbag を含む
annotation-dataset）が Stage 0〜Report HTML の入口になる。Stage Cross Dataset と
Physical Validity は複数データセットの collection が入口。

> 全 stage の成果物は `result_archive/real_log_sim_comparison/` 配下の単一バンドルフォルダに
> まとめて出力する（下表の出力パスはこのバンドルフォルダ基準）。Web.Auto は `result_archive/` の
> 中身をそのまま zip 化するため、ラッパーフォルダが無いと展開時に散らばる。post_process が後段で
> `result_archive/` 直下に書く `result_jsonl.png` のみバンドル外に残る。

| Stage | 名称 | 入力 | 出力 (`result_archive/real_log_sim_comparison/` 配下) | 実行回数 |
|---|---|---|---|---|
| 0 | 実機ログ抽出 (`step0_make_lite --kind real`) | `input_bag/*.{mcap,db3}` | `lite/real.lite/` | 1 |
| OL1 | VehicleModel N-step オープンループ解析 (`step_ol1_analyze_nstep`) | `lite/real.lite/` + `Conditions.cases` の 1 ケース | `comparison/nstep/<tag>/` | N (cases) |
| OL2 | ケース集約解析 (`step_ol2_analyze_cases`) | `nstep/<tag>/nstep_delta.csv` 群 | `comparison/cases/{overlay/, cases_summary.md, cases_metrics.json}` | 1 |
| CL1 | scenario 自動生成 (`step_cl1_bag_to_scenario`) | `input_bag/` + map | `scenarios/auto_scenario.yaml` (OpenSCENARIO) | 1 |
| CL2 | closed-loop シム実行 (`step_cl2_run_sims`) | `auto_scenario.yaml` + `Conditions.sim_runs` の 1 run | `lite/<run_tag>.lite/` | N (sim_runs) |
| CL3 | 実機 + sim 比較解析 (`step_cl3_compare_logs`) | `lite/{real, <run_tag>}.lite/` 群 | `comparison/{figures/, report.md, metrics_closed_loop.json}` (N-way 重ね描き + 機械可読メトリクス・実機カバレッジ) | 1 |
| CL4 | DP軌跡比較 (`step_cl4_compare_dp_trajectory`) | `lite/{real, <sim>}.lite/` | `comparison/figures/dp_*.fig.json` | 1 |
| Report HTML | HTML レポート生成 (`step_report_html`) | `comparison/` 配下の全 `*.fig.json` + 再生 HTML + 各 `.md` + 設定 YAML | `report.html` (バンドルフォルダ直下・plotly.js を 1 回インライン + 図スペックを gzip+base64 遅延展開する単一 HTML。`--collection-dir` でマルチ DS レポート) | 1 |
| Cross Dataset | データセット横断分析 (`step_cross_dataset`) | collection 内全 DS の `metrics_closed_loop.json` + `cases_metrics.json` | `cross_dataset/{cross_*.fig.json, coverage_overview.fig.json, loo_stability.fig.json, cross_metrics.json, cross_summary.md}` (collection 単位・rollout 再実行なし) | 1 / collection |
| Physical Validity | チューニング検証レポート生成 (`physical_validity_report.py`) | collection 内の real.lite 群 + `tuned_params.yaml` | `physical_validity_report.html` (collection root 直下。チューニング済み車両パラメータの物理的妥当性を実機ログからの独立同定と理論式の両面で検証する collection 単位レポート。`make local_multidataset_cloud_run` の Step 4 で実行) | 1 / collection |

成否はパイプラインの例外有無で決まる。全 stage が完走すれば `result.jsonl` に
`Success: true`、いずれかの subprocess が非ゼロ終了またはタイムアウトすると
`Success: false` と Python traceback が記録される。

**closed-loop sim の終了条件** は Stage CL1 が auto-scenario に書き込む（「厳密さより同じコースで
確実に切り上げる」優先）。ゴール厳密到達 (`--goal-tolerance` 既定 15m) に加え、一定時間経過後に
ゴール近傍 (`--goal-vicinity-tolerance` 既定 30m) へ入れば停止・通過・周回いずれでも切り上げる
（exitSuccess）。これで「ゴール付近で止まってもタイムアウトせず」「通過しても周回し続けない」を
両立する。開始直後の誤発火（ループ経路は start≈goal）は実走 course 時間に基づく時間ゲート
（`max(30s, 0.5×course)`）で防ぐ。どの条件にも掛からず `--sim-timeout`（既定 600s）に達した
場合のみ exitFailure。長時間静止での切り上げ (`--standstill-timeout`) は **既定無効の opt-in**
（近傍停止は vicinity で捕捉済み。有効化すると `reproduce_perception` の先行車 dwell 再現を途中で
切り上げてしまうため非推奨）。（`TraveledDistanceCondition` は scenario_simulator_v2 未サポートの
ため走行距離での切り上げは不可。）

> **ゴール近傍は精度算出から除外する（終端は「追わず除外」）**: rosbag 終端のゴール付近は、実際に
> Autoware へ与えたゴールと一致しないことがある（開始時は初期状態を丁寧に合わせ込むが、終端は
> そうではない）。終端の動きの差は構造的に不可避なので、step_cl3_compare_logs は実機 kinematic 終端から
> `GOAL_EXCLUSION_M`（既定 30m、env で上書き可）以内の区間を全精度指標（mean_speed / vel_rmse /
> steer_rmse / s2r / r2s / completion）から除外する。これにより point-to-point 走行（例 takanawa）で
> 終端の減速・混雑停止が指標を支配して見かけ上の大誤差を出す問題が解消し、巡航区間の純粋な再現性で
> 比較できる（takanawa は除外後 best_normal で mean_speed +7% と odaiba 並）。sim の終了は既定の
> goal-vicinity 終端（30m）のままでよく、終端停止を sim 側で再現させる必要はない。

ゴール到達判定の Position は teleport/routing と同じ **LanePosition**（`_world_to_lane_position`）
で書く。`ReachPositionCondition` は 3D の `hypot(x,y,z)` で距離判定し WorldPosition の z を
literal 使用するため、map が標高（例 ~40m）を持つ dataset では `z=0` の WorldPosition だと ego の
map-pose z との差で**永久に発火しなかった**（2026-06-03 判明）。LanePosition は z を map から得るので
発火する。また Stage CL2 は scenario_test_runner の `global_timeout`（scenario 実行の壁時計上限、
既定 180s）を `max(180, timeout_s − initialize_duration − 60)` に引き上げる。本 sim は実機の約 3 倍
遅く 180s 壁ではゴール到達前に打ち切られるため。実終了は scenario の exitSuccess、global_timeout は
外側の壁セーフティ。

## ファイル構成（本ディレクトリに全集約）

リポジトリの他ユースケースと異なり、ソース・evaluator_node・Makefile・sample・README を
すべて本ディレクトリ配下に集約している。

| パス | 内容 |
|---|---|
| `step0_make_lite.py` … `step_cross_dataset.py` | パイプラインの各 stage 実装（ファイル名が役割を示す。step_cross_dataset のみ collection 単位） |
| `evaluator_node.py` | per-dataset パイプライン (Stage 0〜Report HTML) を orchestrate する ROS2 ノード。`lib/driving_log_replayer_v2/real_log_sim_comparison_evaluator_node.py` に install される（CMakeLists で `RENAME` 互換） |
| `run_batch.py` | scenario.yaml の Datasets 全 UUID を順次ローカル実行するバッチドライバ（収集 + Stage Cross Dataset + マルチ DS report.html まで） |
| `collect_datasets.py` | per-dataset バンドル成果物を collection に symlink 収集（`collection.yaml` manifest 出力） |
| `multi_dataset_tune.py` | 収集済み real.lite 群での open-loop ロバスト同定（robust_search） |
| `lib/_*.py` | 共有ユーティリティ・内部設定（io / events / map / params / runtime_config / cases_config / sim_runs_config / provenance / collection / coverage / multi_agg）。stage 実装から `from .lib._x import` で参照 |
| `Makefile` | `make local_cloud_run`（1 DS フル実行）/ `local_analysis_run`（解析のみ再実行）/ `local_batch_run`（複数 DS 一括）/ `local_cross_analysis_run`（横断分析のみ再実行）。詳細は `sample/README.ja.md` |
| `sample/` | cloud / local 共通サンプル一式（`scenario.yaml` と課題別 `scenario_*.yaml`）+ 手順 README。ローカル実行出力は `sample/out/`（gitignore） |

> **install パス**: CMakeLists が `sample/*` を
> `share/driving_log_replayer_v2/sample/real_log_sim_comparison/` に install する
> （登録済み cloud Web.Auto scenario が参照する install-share パスを維持）。

## 入力

### データセット（`input_bag/` = SSOT）

`scenario.yaml` の `Datasets[0]` UUID に対応する annotation-dataset を
`webauto data annotation-dataset pull --include-intermediate-artifacts` で取得し、
`t4_dataset_path/input_bag/` 配下に AUTONOMOUS 走行を含む実機 rosbag を置く。

- ディレクトリ形式の rosbag2（db3 / mcap いずれも可）または単一 `.mcap` ファイル。
- 下表「実機ログ（`TOPICS["real"]`）」のトピックが録られていること。
- `map_path`（`lanelet2_map.osm` を含む地図ディレクトリ）は軌跡プロットの地図背景に使用。
  無い場合は地図なしで描画する。

### 抽出トピック

評価ノードは ROS トピックを subscribe しない。`step0_make_lite` が rosbag を読み、
以下のトピックのみを lite bag に書き出す。

#### 実機ログ（`TOPICS["real"]`）

| Topic name | 用途 |
| ---------- | ---- |
| `/system/operation_mode/state` | AUTONOMOUS 区間の切り出し |
| `/vehicle/status/velocity_status` | 速度応答 |
| `/vehicle/status/steering_status` | 操舵応答 |
| `/vehicle/status/gear_status` | gear 状態（DRIVE 系区間のみを同定・評価に使用。欠落する旧 lite は再生成必須） |
| `/localization/kinematic_state` | 自車位置・軌跡 |
| `/localization/acceleration` | 加速度応答 |
| `/control/command/control_cmd` | 制御指令（post-gate） |
| `/control/command/gear_cmd` | gear 指令（診断・sim 比較用） |
| `/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory` | DiffusionPlanner 出力軌跡（シムと直接比較） |
| `/perception/object_recognition/tracking/objects` | 追跡物体（社会的コンテキストの確認） |
| `/planning/trajectory` | 最終プランニング軌跡（optimizer 後段出力） |

#### シミュレーションログ（`TOPICS["sim"]`）

Stage CL2 (`step_cl2_run_sims`) が `scenario_test_runner` で sim を回した結果の rosbag から、
`step0_make_lite.py` の `TOPICS["sim"]` で抽出して `lite/<run_tag>.lite/` を生成する。

| Topic name | 用途 |
| ---------- | ---- |
| `/system/operation_mode/state` | AUTONOMOUS 区間の切り出し |
| `/vehicle/status/velocity_status` | 速度応答 |
| `/vehicle/status/steering_status` | 操舵応答 |
| `/vehicle/status/gear_status` | gear 状態（DRIVE 系区間のみを同定・評価に使用） |
| `/localization/kinematic_state` | 自車位置・軌跡 |
| `/localization/acceleration` | 加速度応答 |
| `/control/trajectory_follower/control_cmd` | trajectory_follower の制御指令 |
| `/control/command/control_cmd` | 制御指令（post-gate、実機との同段比較用） |
| `/control/command/gear_cmd` | gear 指令（診断・実機との同段比較用） |
| `/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory` | DiffusionPlanner 出力軌跡（速度プロファイル分析用） |
| `/perception/traffic_light_recognition/traffic_signals` | 交通信号状態（DiffusionPlanner 入力） |

## 設定

`scenario.yaml` の `Evaluation.Conditions` で以下のキーを認識する。

| キー | 必須/任意 | 内容 |
| ---- | --- | ---- |
| `scenario_name` | 任意 | 図タイトルに表示するシナリオ名。未指定時は `ScenarioName` を使用。 |
| `models` | **必須** | 名前付き車両モデル定義レジストリ。各エントリに `vehicle_model_type`（open-loop クラス名）・`vehicle_model`（sim description パッケージ名）・`params`（simulator_model 上書き）等を記述。`cases`/`sim_runs` からこの名前で参照する。 |
| `cases` | **必須** | Stage OL1/OL2（N-step open-loop 解析）に使うモデル名リスト。各名前は `models` に `vehicle_model_type` を持つこと。 |
| `sim_runs` | **必須** | Stage CL2/CL3（closed-loop sim + N-way 比較）に使うモデル名リスト。各名前は `models` に `vehicle_model` を持つこと。 |
| `overlay` | 任意 | Stage OL2 集約の基準モデル（`reference_tag`）と重ね描き種類（`plots`）。 |
| `real_provenance` | 任意 | 実機データ取得時の pilot-auto.x2 / DiffusionPlanner 重みの自由記述。比較プロット・report.md の provenance に掲載し、sim 実行時の版・重み（自動取得）との差を解釈する。 |
| `traffic_signals` | 任意 (既定 `replay`、`reproduce_perception: true` 時は `none`) | 信号の扱い。`replay`=実機 bag の信号タイムシリーズを再現。`green`=全信号常時 green。`none`=scenario 側で信号をセットしない（`reproduce_perception` が信号 topic を直接 publish して所有する場合に使用）。sim 早期停止（旧称 D0）の真因は赤信号 replay の到達時刻 desync（実機が green 通過した信号に sim ego が赤で当たり永久停止）であり、`green`/`none` で周回を完走できる（live sim A/B 実測: replay=241m 停止 / green=519m 完走、実機 598.7m）。**赤信号停止も忠実に再現したい場合は `reproduce_perception: true`**（信号も bag から再生するため、実車が緑通過した位置は緑・赤停止した位置は赤になる）。 |
| `loop_waypoints` | 任意 (既定 0) | route 形状を強制する **実験オプション**（**D0 の修正ではない**）。Stage CL1 が start+goal に加え実走軌跡の膨らみ位置へ N 個の中間 LanePosition waypoint を挿入する。D0（sim 早期停止）の真因は赤信号 replay であり routing ではない（lanelet graph に shortcut が無く start+goal でも route は周回全体を引く）ことが live sim で確定したため、D0 解消には `traffic_signals: green` を使う。 |
| `reproduce_perception` | 任意 (既定 false) | `true` で実機 input_bag の**検出物体（detection/objects）・信号（traffic_signals）・占有格子（occupancy_grid）を sim 時刻同期**で各 sim に再生（simple_sensor_simulator 内蔵の `PerceptionReproducerSensor`。Stage CL2 が launch 引数 `replay_bag_path` / `replay_start_time` に変換し、通常の検出センサは抑止される）。auto-scenario は NPC を持たないため、実機が先行車追従主体（cruise_following 等）の走行で sim ego が先行車不在により自由加速して実機より速くなるのを防ぎ、**実機の停止・加減速を再現**する。信号も bag から再生するので、実車が緑通過した位置は緑・赤停止した位置は赤となり、**赤信号停止の忠実再現と D0 偽停止回避を両立**（`traffic_signals` の既定が `none` に切り替わり scenario 側の信号設定は無効化される）。時刻同期の代わりに sim ego 最近傍時刻のスナップショット再生にする場合は `replay_position_based: true`。完全一致は real/sim の DiffusionPlanner 重み差により頭打ち。 |
| `ego_replay_duration` | 任意 (既定 0=無効) | scenario 開始から指定秒数、実機 bag の **ego 状態（pose/twist/accel）を traffic_simulator の `EgoBagReplayer` が vehicle model に毎フレーム注入**し、経過後に closed-loop へ切替える。AUTONOMOUS 開始時点で ego が速度を持つ rosbag に対し、停止発進ではなく**実機と同じ走行中状態から sim を始める**初期状態合わせ。Autoware 初期化（localization・route・engage）は scenario time 開始前に完了しているため注入と干渉せず、vehicle model は毎フレームシード済みのため切替に段差が出ない。要 `reproduce_perception: true`（同じ bag を使う）。切替時刻は provenance（`ego_replay_duration`）に記録され、注入区間のメトリクスは実機と一致して当然である点に注意。 |
| `replay_preroll` | 任意 (既定 0) | ego replay の開始アンカーを AUTONOMOUS 開始（t0）より何秒前に取るか [s]。Stage CL1 の TeleportAction start pose も同じアンカー時刻の kinematic から取り、注入開始時の pose ジャンプを防ぐ。 |
| `replay_position_based` | 任意 (既定 false) | perception 再生を時刻同期ではなく **sim ego 最近傍時刻のスナップショット再生**にする。closed-loop 切替後に sim ego が実機タイムラインから逸脱した場合の先行車・信号ずれを緩和する。 |

- **`models`（`Conditions.models`）**: `vehicle_model_type`（open-loop）/ `vehicle_model`（sim）/ `params`（共有）を同列に名前付きで定義。`cases`/`sim_runs` はこの名前リストで参照する。`dp_model_release`（webauto から自動 pull）/ `dp_model_dir`（ローカル既存）で **同一車両モデルのまま DiffusionPlanner モデルだけを差し替えて比較**できる → [`docs/diffusion_planner_model_swap.ja.md`](docs/diffusion_planner_model_swap.ja.md)。

  書式・使用例・フィールド詳細は `sample/README.ja.md` を参照。

## 出力（`result_archive/real_log_sim_comparison/` 配下）

全成果物は `result_archive/real_log_sim_comparison/` の単一バンドルフォルダにまとめて出力する
（zip 展開で 1 フォルダにまとまるようにするため。理由は冒頭パイプライン表の注記参照）。以下のパスは
すべてこのバンドルフォルダ基準。

### 図スペック `*.fig.json`（データ + プロットライブラリ構成）

解析 stage（OL1/OL2/CL3/CL4）の各図は matplotlib SVG ではなく **plotly Figure の JSON（データ + レイアウト）** を
`<stem>.fig.json` として `comparison/` 配下に出力する（`lib/_figures` の純関数 `build_fig_*` で組み、
`lib/_fig_io.write_fig_json` が template を剥がし float を丸めて最小化 JSON 化）。描画は閲覧側
（report.html）の **plotly.js** が行う。旧方式（base64 SVG 埋め込み）と違い、画像では
なく数値データを持つためズーム/パン/hover できる。

### `report.html`（バンドルフォルダ直下・単一 HTML レポート）

`step_report_html`（Stage Report HTML）がバンドルフォルダ直下（`comparison/` の親）に生成する、
`comparison/` 配下の全 `*.fig.json`・Markdown・設定 YAML を **1 枚にまとめた外部ロード無しの単一 HTML**。
この 1 ファイルを渡すだけで Slack 等で共有・閲覧できる。出力ディレクトリ単位ではなく **比較の概念** で
5 セクションに分けて並べる（読み手が「何を何と比べた図か」で辿れるようにするため）。各図は出力先ではなく
(ディレクトリ, ファイル名 stem) の組で分類する。

| セクション | 内容 | 主な図（出力元 stage） |
|---|---|---|
| 1. 推定前：実車とシミュレーションのズレ | パラメータ推定前の、理想追従（操舵完全一致と仮定）における自車位置やモデル構造の限界によるズレ | `cross_perfect_tracking_box`・`cross_perfect_tracking_traj`(Cross Dataset) |
| 2. パラメータ推定結果 | 実機ログから最小二乗法で同定した車両パラメータとそのフィッティング精度 | `cross_physical_validity_{kus,long,steer}`(Cross Dataset) / `viewer` のモデル検証タブ(CL3) |
| 3. 推定後：シミュレーション残差の提示 | ホライズン別の予測誤差や、位置 (x, y) の変位誤差の成長度合い・分布 | nstep の `overview`/`map_distribution`(OL1) + `cases/overlay`(OL2) + `cross_long_perf_*`(Cross Dataset) |
| 4. 最終的な Closed Loop シミュレーション残差 | 最終同定パラメータで closed-loop 実行した際の実機との軌跡・速度・操舵の乖離 | `viewer` の軌跡比較タブ(CL3) + `cross_closed_loop_heatmap`/`cross_normalized_bars`/`coverage_overview`/`loo_stability`(Cross Dataset) |
| その他 | 上記いずれにも分類されなかった図 | `dp_real_vs_sim`・`dp_vs_actual`・`dp_vs_final_traj`(CL4) など |

**`report.html` と `physical_validity_report.html` の役割分担**: 前者は解析パイプラインの成果物で、
scenario の Conditions に列挙された models 群を対象に自動生成される。後者はチューニングワークフロー
（`make local_multidataset_cloud_run` Step 4）の成果物で、同定された `tuned_params.yaml` 1 点に対する
検証（score 再現・偏差テーブル・カーブビューア・closed-loop 比較）に特化する。物理妥当性の計算と図は
どちらも同じ共有ライブラリ（`lib/_physical_validity.py`・`lib/_figures/_physical_validity.py`）を
使っており、数式・定数の実装はそこに一本化されている。

各セクションには 1 行説明を付ける。**CDN 不使用・オフライン可**:

- **図 (`*.fig.json`)**: 各図の JSON を `<div class='plotly-fig'>` プレースホルダ直後の
  `<script type='application/json'>` に並置し、`plotly.min.js` は report 全体で **1 回だけ** `<head>` に
  インラインして共有する。`IntersectionObserver` でビューポート進入時に `Plotly.newPlot` 遅延描画する
  （数十図を一度に描かない）。折りたたみ `<details>` / 非選択ケースタブは display:none で IO が発火しない
  ため、details の toggle・ケース切替でも未描画図を reveal-render する。ズーム・パン・ホバー・凡例トグル可。
- **統合ビューア**（`viewer`）: 軌跡比較とモデル検証をタブ化した自己完結 canvas HTML を `<iframe srcdoc>` で隔離埋め込み。
- **プロット単位ケースタブ**（セクション 3）: `nstep/<case>/` の図をプロット種別ごとにまとめ、
  各ブロック先頭の「ケース切替: `baseline`/`kus0020`/`no_delay` …」を選ぶとそのブロックの図が切り替わる
  （純 CSS ラジオ＋`case-<slug>` クラス対応）。
- **セクション折りたたみ**: 各セクション・Markdown レポート・設定ファイルは `<details open>` で開閉できる。
- **サイド目次**: sticky 追従。「↑ 先頭へ」と各セクションの「↑ 先頭」リンク付き。

Markdown レポート（`cases_summary.md`→セクション 3）は所属セクション
末尾に折りたたみで埋め込む。さらに末尾の「実行構成」セクションにシナリオ・sim 実行設定・車両モデル
パラメータの各 YAML を生テキストで埋め込み、どの設定で生成した報告か追跡できるようにする。既知のいずれにも
分類されない図は捨てず「その他」セクションに回す（黙って誤分類しない）。`plotly.js` を含め全て 1 ファイルに
収めるため、`report.html` 1 つだけ取り出して共有してもオフラインでそのまま表示できる。

### `lite/`

`step0_make_lite` が抽出した lite bag（rosbag2 mcap）。実機 `real.lite/` と、Stage CL2 が
`Conditions.sim_runs` の各 run について生成する `<run_tag>.lite/`（例 `sim_normal.lite/`,
`sim_kus0020.lite/`, `sim_perfect.lite/`, `sim_godot.lite/`）が併置される。

### `comparison/report.md`

Markdown 形式の比較レポート。以下を含む。

- モデル重み / バージョン provenance（実機 vs 各 sim の DP 重み・autoware バージョン；版差での乖離解釈用）
- 完走時間（AUTONOMOUS 開始～停止）
- 速度統計（`VelocityReport.longitudinal_velocity` の平均・最大・標準偏差）
- 速度 RMSE（指令 vs 応答）
- ステアリング RMSE（指令 vs 応答）
- 軌跡乖離（実機の bounding box 内に入る各シム軌跡点の、実機軌跡への最近傍距離）

### `comparison/figures/`

自己完結 HTML ビューア（Stage CL3）と DP 軌跡比較の図スペック（Stage CL4）。

| ファイル | 内容 |
| -------- | ---- |
| `viewer.html` | Stage CL3: 軌跡比較と縦横モデル検証をタブ統合した、plotly非依存の自己完結canvasビューア。時刻／距離同期、DP軌跡、metrics、モデルつまみ、モデル選択、誤差表示、全最適化機能を収録 |
| `dp_real_vs_sim.fig.json` | Stage CL4: DiffusionPlanner 出力軌跡 実機 vs sim |
| `dp_vs_actual.fig.json` | Stage CL4: DP計画速度(d=0) vs actual速度 |
| `dp_vs_final_traj.fig.json` | Stage CL4: 実機 DP出力 vs 最終 planning（optimizer 補正） |

### `comparison/nstep/<case_tag>/`

`step_ol1_analyze_nstep`（Stage OL1）によるケース別 N-step オープンループ解析の成果物。
1 ケースあたり `nstep_delta.csv`（全 horizon 統一スキーマ）+ 図 2 枚
（`overview.fig.json`, `map_distribution.fig.json`）+ `summary.txt`（N=1 詳細 RMSE +
horizon 別 RMSE）。ケース横断の比較図は `cases/overlay/` が担う。
ケースは `scenario.yaml` の `Conditions.cases` で定義する。

> N=1 は毎ステップ reset のため k_us/wheelbase に非感度。大 N の rollout（`run_rollout`）が
> N ステップ連続予測で dynamics 差を顕在化する。

### `comparison/cases/`

`step_ol2_analyze_cases`（Stage OL2）による全ケース集約解析の成果物。

| ファイル | 内容 |
|---|---|
| `cases_summary.md` | N=1 詳細 RMSE 表（reference との Δsteer 付き）+ horizon 別 終端誤差 RMSE 横断表（pos/yaw + Δyaw vs ref） |
| `overlay/cascade_error_overlay.fig.json` | 全ケースを 1 枚に重ね描き（段階的誤差） |
| `overlay/error_growth_overlay.fig.json` | 全ケースの horizon 別 RMSE 成長を重ね描き（位置/yaw） |

> 誤差時系列の対話的確認は統合ビューア（`figures/viewer.html` のモデル検証「誤差パネル」）へ移設した。case × horizon の俯瞰は `cases_summary.md` の表が担う。

### `result_bag/`

post_process の `create_metadata_yaml` を通すためのプレースホルダ mcap が事前に書き込まれる。
本ユースケースは実走 bag を録らないが、後段 post_process が `result_bag_path` の存在を
要求するための互換用ファイル。

### `result.jsonl`

`output_dir/result.jsonl` の最終行で成否を判定する。`Success` がパイプライン例外有無、
`Summary` が `"Success"` または Python traceback 文字列。

```json
{"Result": {"Success": true, "Summary": "Success"}, "Stamp": {"System": 0.0}, "Frame": {}}
```

## 実行

### クラウド（Web.Auto evaluator）

`pc_dlr_type: real_log_sim_comparison` ラベルで登録した scenario を suite で選択して実行する
（`.webauto-ci.yml` の `simulations` 参照）。launch は Autoware を起動せず
`add_use_case_arguments` と `launch_evaluator_node` のみを実行する。評価ノードに渡る主な引数:

- `t4_dataset_path`: 実機 rosbag を `input_bag/` 配下に含むデータセットルート
- `map_path`: `lanelet2_map.osm` を含む地図ディレクトリ
- `result_jsonl_path` / `result_archive_path` / `result_bag_path` / `scenario_path`: 共通の出力・入力パス

クラウドでは Web.Auto が対象 dataset を固定マウント点に事前ステージし、上記
`t4_dataset_path`（`input_bag/`・`map/` を直下に持つ dir）を**直接** launch に注入する。

### ローカル（`make local_cloud_run`）

webauto で T4 dataset を pull 済みの環境で `make local_cloud_run` 一発で per-dataset パイプライン
（Stage 0〜Report HTML）がすべて走る。
ローカルでは `lib/_dataset.py`（解決の SSOT）が webauto キャッシュ
`~/.webauto/data/data/annotation_dataset/<UUID>/<frame>` を解決し、クラウドと**同一の
`t4_dataset_path` 契約**を渡す。すなわち `t4_dataset_path` 以降の扱いはローカル/クラウドで
完全に共通で、両者の違いは「誰がパスを解決するか」（クラウド=Web.Auto ステージング /
ローカル=`lib/_dataset.py`）だけ。
手順詳細・`Conditions.models`/`cases`/`sim_runs` の書式・トラブルシュート・Makefile 変数の上書き例は
[`sample/README.ja.md`](sample/README.ja.md) を参照。

### ローカル解析のみ再実行（`make local_analysis_run`）

sim 実行 (Stage 0〜CL2: lite 抽出・scenario 生成・sim 実行) は重いため、解析コード
(Stage OL1〜Report HTML) を変更して結果を作り直すときは
既存の出力バンドルを再利用して **解析ステージだけ** を回せる:

```bash
make local_analysis_run                                  # sample/out/latest を再解析
make local_analysis_run OUT_DIR=sample/out/20260603_211156   # 対象を指定
```

実体は `run_analysis.py` CLI（ROS launch 不要、オーケストレーションは
`evaluator_node.run_analysis` をフル実行と共用）。`comparison/` 配下の解析成果物と
`report.html` は上書き再生成される（`lite/`・`scenarios/` には触れない）。

## マルチデータセット評価

単一データセットの分析は走行条件（速度域・カーブ・エリア）に偏る。複数データセットを
collection に束ねて横断分析することで、モデル比較の結論がデータセットに依存しないかを
検証できる。

### 構成

```
sample/out/batch_<ts>/                  # collection root (= COLLECTION_DIR)
├── collection.yaml                     # 収集 manifest (dataset 一覧と status)
├── runs/<dataset_uuid>/                # 各 DS の launch 出力 (従来 out/<ts> と同構造)
├── datasets/<dataset_uuid>/            # 収集ビュー (collect_datasets.py が symlink)
│   ├── real.lite[.mcap]  comparison/  scenarios/
├── cross_dataset/                      # Stage Cross Dataset 出力 (横断分析の図 + cross_summary.md)
└── report.html                         # マルチ DS 単一レポート (step_report_html --collection-dir)
```

### 一括ローカル実行（`make local_batch_run`）

`scenario.yaml` の `Evaluation.Datasets` に複数 UUID を列挙し（クラウドと同じ宣言場所）、
全 dataset を pull 済みの環境で:

```bash
make local_batch_run                      # 全 UUID を順次実行 → 収集 → 横断分析 → report.html
make local_batch_run RESUME=1 BATCH_ROOT=sample/out/batch_20260611_120000  # 中断再開
make local_batch_run SKIP_SIM=1           # closed-loop sim を省略 (open-loop 解析のみ・高速)
```

`run_batch.py` が UUID ごとに single-dataset scenario を生成して per-dataset パイプラインを
実行する（クラウドの「1 評価ジョブ = 1 dataset」のローカル再現）。失敗 dataset は
`collection.yaml` に status を記録してスキップし、残りで横断分析を続行する。

**closed-loop sim のスキップ (`SKIP_SIM=1` / `Conditions.skip_sim`)**: Stage CL2 は
1 dataset あたり sim run 数 × 数分かかり、マルチ DS バッチでは支配的になる。closed-loop
比較が不要なとき（open-loop N-step・パラメータ同定・カバレッジだけ欲しいとき）は
`SKIP_SIM=1`（`local_cloud_run` / `local_batch_run` 共通、クラウドは scenario.yaml の
`Conditions.skip_sim: true`）で Stage CL2 だけを省略できる。Stage CL1（scenario 生成）と
解析ステージ（Stage OL1/OL2/CL3/CL4・Report HTML・Cross Dataset）は実行され、sim 依存の図・行列（closed-loop 比較・DP 比較）は自動的に
省略される。成否判定も「sim 0 件 = INCOMPLETE」を適用しない。後から closed-loop も
欲しくなったら `SKIP_SIM`/`RESUME` 無しで同じ `BATCH_ROOT` を再実行する（real.lite
抽出からやり直すが、支配的な sim 実行時間に対して誤差）。

### クラウド実行分の収集

クラウドで per-dataset 評価ジョブを実行済みの場合は、各ジョブの result_archive を DL して
`collect_datasets.py` で後付け収集すれば同じ collection になる:

```bash
python3 -m driving_log_replayer_v2.real_log_sim_comparison.collect_datasets \
    --bundle <DLしたバンドルA> --bundle <DLしたバンドルB> --collection-dir sample/out/multi_eval
make local_cross_analysis_run COLLECTION_DIR=sample/out/multi_eval
```

### 横断分析（Stage Cross Dataset）とマルチ DS レポート

`step_cross_dataset` は per-dataset の機械可読メトリクス
（`metrics_closed_loop.json` + `cases_metrics.json`）を再集計するだけで rollout を再実行
しない（数秒で完了）。出力は 4 系統:

1. **モデル×DS 行列**: closed-loop 軌跡乖離・完走率（DS × sim run）と open-loop N-step
   終端誤差（DS × case）のヒートマップ。どのモデルがどの dataset で悪いかを俯瞰する。
2. **正規化 mean/worst 集約**: 各 DS の baseline（`Conditions.overlay` の `reference_tag`）
   誤差で正規化してから横断 mean/worst を取り、case をロバスト性でランキングする
   （`multi_dataset_tune` の同定スコアと同一定義 = `lib/_multi_agg.py`）。
3. **走行特性カバレッジ**: DS 別の速度域滞在比率・加減速分布・曲率半径ビン別走行距離・
   カーブ数（`lib/_coverage.py`、step_cl3_compare_logs が実機ログから集計）。評価データの偏りを可視化する。
4. **leave-one-out 安定性・外れ DS 検出**: DS を 1 つ除くと best case が入れ替わらないか、
   誤差プロファイルが他と乖離した DS（robust z-score）が無いかを定量化する。
   dataset 数 < 3 では理由を明記してスキップする（単一 DS でも同一コードパスで動く）。

マルチ DS の `report.html` は**単一 HTML + データセットセレクタ**構成: デフォルトで
「0. データセット横断サマリー」を表示し、右上のセレクタで個別 DS の従来セクション
（1〜4 + 実行構成）へ一括切替する。単一 DS ではセレクタが出ず従来と同じ見た目。
図スペックは gzip+base64 で埋め込み、可視化時に DecompressionStream で展開・描画する
（Chrome 80+ / Firefox 113+。`#ds=<id>` ハッシュで選択状態を共有できる）。

ロバスト同定（`make local_multidataset_run` = `multi_dataset_tune.py` の robust_search）は
従来どおり real.lite 群での rollout 探索で、レポート用集約（Stage Cross Dataset）とは役割が異なる。

---

## 課題シナリオの作成ワークフロー（MOB リンク → DP モデル比較）

実機で起きた課題（DevOps 分析シートなどから特定）を Sim で再現し、
DiffusionPlanner モデルの切り替えで TP/FN を検証するシナリオを作成する手順。

### 0. 前提知識

- **TP (True Positive)**: 課題が起きやすいモデル（`sim_dp_0410`）で Sim でも課題が再現されること
- **FN (False Negative)**: 課題が起きにくいモデル（`sim_dp_0303` / `sim_dp_0503`）では
  Sim でも課題が起きないこと
- 課題の種別は Perception 系（誤認識など）ではなく、幾何/計画/制御系（大回り、曲がり切れない等）
  を対象とする。DP モデル切り替えで再現性の差が出ることを前提とする。

### 1. 対象課題の特定と rosbag file_id の取得

MOB プラットフォームの rosbag 詳細画面 URL から `file_id` を取得する。
例: `https://console.mob.tier4.jp/projects/x2_dev/rosbag?file_id=<FILE_ID>&rviz_id=...`

### 2. データセットの構築（`make issue_scenario`）

課題テンプレート YAML を選択し、`make issue_scenario` を実行する。

```bash
# テレポート駅ロータリー左旋回 課題の例
make -C <real_log_sim_comparison ディレクトリ> issue_scenario \
    ROSBAG_ID=f8b5e21f-8361-45a6-9079-05fbf40366a5 \
    TEMPLATE=$(pwd)/sample/scenario_issue_rotary_left.yaml \
    PROVENANCE="2026-03-31 お台場, pilot-auto.x2 ブランチ=XXX, DP モデル=XXX"
```

- **`ROSBAG_ID`**: webauto rosbag の file_id（MOB リンクの `file_id=` パラメータ）
- **`TEMPLATE`**: 課題の種類に合うシナリオテンプレート（後述）
- **`PROVENANCE`**: 実機走行時の pilot-auto.x2 ブランチ・当日コミット・DP モデルを記録する文字列。
  実験スレッド（Slack: #group-reference-vehicle-x2-experiment）を参照して入力する。

完了すると `work/dataset/<uuid>/{input_bag,map}/` と `work/scenario_<uuid>.yaml` が生成される。

> **annotation-dataset が存在する場合（webauto で pull できる場合）**:  
> `make issue_scenario` の代わりに
> `webauto data annotation-dataset pull --project-id x2_dev --annotation-dataset-id <UUID> --include-intermediate-artifacts`
> で pull し、シナリオ YAML の `<DATASET_UUID>` を置き換えても良い。

### 3. シナリオの実行

```bash
make -C <real_log_sim_comparison ディレクトリ> local_cloud_run \
    WEBAUTO_T4_ROOT=$(pwd)/work/dataset \
    LOCAL_SCENARIO=work/scenario_<uuid>.yaml
```

DP 3 モデル（`sim_dp_0303` / `sim_dp_0503` / `sim_dp_0410`）が自動で webauto から pull され、
それぞれ closed-loop sim が実行される（GPU + 長時間 要）。

### 4. 結果の確認

`sample/out/latest/report.html` を開き、「4. 最終的な Closed Loop シミュレーション残差」
セクションを確認する。

- `figures/viewer.html`: 実機 vs 各 sim の軌跡比較と縦横モデル検証
- `report.md` / `metrics_closed_loop.json`: 軌跡乖離（s2r / r2s）の定量値
- **TP**: `sim_dp_0410` の軌跡乖離が顕著に大きい（課題再現）
- **FN**: `sim_dp_0303` / `sim_dp_0503` の軌跡乖離が実機に近い（課題非再現）

### 用意されているシナリオテンプレート

| テンプレート | 対象課題 |
|---|---|
| `sample/scenario_issue_template.yaml` | 汎用テンプレート（UUID・地図名は要書き換え） |
| `sample/scenario_issue_rotary_left.yaml` | テレポート駅ロータリー入口 左旋回（曲がるタイミング早い） |
| `sample/scenario_issue_aist_curve.yaml` | 産総研脇〜テレコム 左カーブ大回り（速度超過・曲がり切れない） |
| `sample/scenario_curve_wide_turn.yaml` | テレポート駅前交差点 左折 カーブ大回り（既存） |

### pilot-auto.x2 バージョンの記録方針

Sim では autoware 本体版は `autoware.repos` で固定されているため、
「どの実機走行を Sim で再現しているか」の記録は `real_provenance` フィールドで行う。

- `real_provenance` に実機走行日・pilot-auto.x2 ブランチ（当日最新コミット）・DP モデル版を記載する
- 同日の実験ではブランチが同一であることが多いため、当日の実験スレッドを参照して確認する
- DP モデルの比較は `sim_runs_config` の `dp_model_release` で制御する（webauto から自動 pull）

> **再現率と DP 版整合の関係（2026-06-16 ローカル検証で確認）**: replay
> （`reproduce_perception` + `ego_replay_duration` + `replay_position_based`）は実機の
> perception・ego 状態を忠実に再生するため、**stop-start / 先行車追従が支配的な走行**では sim ego の
> 速度・軌跡を実機に強く近づける（odaiba A09 で平均速度誤差 +38.8%→+8.5%、vel_rmse 2.55→1.56）。
> 一方 **開放路（先行車制約が弱い区間）**では replay 窓を抜けた closed-loop の速度が DiffusionPlanner
> の巡航速度選好に支配され、実機採取時にデプロイされていた DP 版と sim の DP 版が異なると残差が
> 大きく出る（takanawa で +60%）。したがって**再現率を詰めるには採取時の DP 重み版を
> `dp_model_release` で一致させること**が要点で、`real_provenance` の記録が前提になる。
>
> 実証（takanawa f20a29ab, normal, replay 有効）: 既定 DP（`neighbor320_prev_data_sft`）では
> mean_speed 誤差 +60%・経路追従 s2r 0.32m だったのが、採取時にデプロイされていた DP
> （`diffusion_planner_for_x2_20260503-094624_with_takanawa_1day__epoch0060__epoch0080`、Slack 実験
> スレッドの走行時刻照合で特定）に `dp_model_release` で一致させると **mean_speed 誤差 +37%・
> s2r 0.15m** に改善した（速度誤差 −40%・経路追従 2 倍）。採取時 DP の特定は走行当日の実験
> スレッド（例: `#group-reference-vehicle-x2-experiment`）で rosbag 開始時刻に対応する
> 「Model: …」投稿を照合して行う。
