# Real Log / Sim Comparison の評価

実機走行ログ（T4 dataset の `input_bag/` rosbag）を **SSOT** とし、同じ走行を
Autoware + シミュレータで closed-loop 再現して、実機との乖離を統計値と図で比較する
ユースケース。速度応答・操舵応答・軌跡精度・per-step delta を可視化する。

他ユースケースと異なり、launch から **Autoware・bag player・bag recorder のいずれも
起動しない**。評価ノード `real_log_sim_comparison_evaluator` が 10 段階パイプラインを
subprocess で直接実行する。

## パイプライン（10 段階）

`sample/scenario.yaml` の `Datasets[0]` UUID = SSOT（実機 rosbag を含む
annotation-dataset）が全 stage の入口になる。

| Stage | 名称 | 入力 | 出力 (`result_archive/` 配下) | 実行回数 |
|---|---|---|---|---|
| 1 | 実機ログ抽出 (`step1_make_lite --kind real`) | `input_bag/*.{mcap,db3}` | `lite/real.lite/` | 1 |
| 2 | scenario 自動生成 (`step2_bag_to_scenario`) | `input_bag/` + map | `scenarios/auto_scenario.yaml` (OpenSCENARIO) | 1 |
| 3 | closed-loop シム実行 (`step3_run_sims`) | `auto_scenario.yaml` + `sim_runs.yaml` の 1 run | `lite/<run_tag>.lite/` | N (sim_runs) |
| 4 | 実機 + sim 比較解析 (`step4_compare_logs`) | `lite/{real, <run_tag>}.lite/` 群 | `comparison/{figures/, report.md}` (N-way 重ね描き) | 1 |
| 5 | VehicleModel per-step 解析 (`step5_analyze_per_step`) | `lite/real.lite/` + `cases.yaml` の 1 ケース | `comparison/per_step/<tag>/` | N (cases) |
| 6 | ケース集約解析 (`step6_analyze_cases`) | `per_step/<tag>/per_step_delta.csv` 群 | `comparison/cases/{overlay/, cases_summary.md}` | 1 |
| 7 | k_us 同定 (`step7_identify_kus`) | `lite/real.lite/` | `comparison/kus_sweep/{kus_sweep.csv, .png}` | 1 |
| 8 | DP軌跡比較 (`step8_compare_dp_trajectory`) | `lite/{real, <sim>}.lite/` | `comparison/figures/dp_*.png` | 1 |
| 9 | 縦パラ同定 (`step9_identify_brake`) | `lite/real.lite/` | `comparison/brake_sweep/{brake_sweep.csv, .png}` | 1 |
| 10 | カーブ乖離診断 (`step10_diagnose_curve`) | `lite/{real, <sim>}.lite/` | `comparison/curve_diag/{curve_divergence.md, .png}` | 1 |
| 11 | HTML レポート生成 (`step11_build_html_report`) | `comparison/` 配下の全 PNG + 各 `.md` | `comparison/index.html` | 1 |

成否はパイプラインの例外有無で決まる。全 stage が完走すれば `result.jsonl` に
`Success: true`、いずれかの subprocess が非ゼロ終了またはタイムアウトすると
`Success: false` と Python traceback が記録される。

## ファイル構成（本ディレクトリに全集約）

リポジトリの他ユースケースと異なり、ソース・evaluator_node・Makefile・sample・README を
すべて本ディレクトリ配下に集約している。

| パス | 内容 |
|---|---|
| `step1_make_lite.py` … `step10_diagnose_curve.py` | 10 段階パイプラインの各 stage 実装（先頭 `stepN_` が実行順） |
| `evaluator_node.py` | パイプラインを orchestrate する ROS2 ノード。`lib/driving_log_replayer_v2/real_log_sim_comparison_evaluator_node.py` に install される（CMakeLists で `RENAME` 互換） |
| `lib/_*.py` | 共有ユーティリティ・内部設定（io / events / map / params / runtime_config / cases_config / sim_runs_config / provenance）。stage 実装から `from .lib._x import` で参照 |
| `Makefile` | `make local_cloud_run` でローカル実行（詳細は `sample/README.ja.md`） |
| `sample/` | cloud / local 共通サンプル一式（`scenario.yaml`, `cases.yaml`, `sim_runs.yaml`, `curve_config_miraikan.yaml`）+ 手順 README。ローカル実行出力は `sample/out/`（gitignore） |

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

評価ノードは ROS トピックを subscribe しない。`step1_make_lite` が rosbag を読み、
以下のトピックのみを lite bag に書き出す。

#### 実機ログ（`TOPICS["real"]`）

| Topic name | 用途 |
| ---------- | ---- |
| `/system/operation_mode/state` | AUTONOMOUS 区間の切り出し |
| `/vehicle/status/velocity_status` | 速度応答 |
| `/vehicle/status/steering_status` | 操舵応答 |
| `/localization/kinematic_state` | 自車位置・軌跡 |
| `/localization/acceleration` | 加速度応答 |
| `/control/command/control_cmd` | 制御指令（post-gate） |
| `/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory` | DiffusionPlanner 出力軌跡（シムと直接比較） |
| `/perception/object_recognition/tracking/objects` | 追跡物体（社会的コンテキストの確認） |
| `/planning/trajectory` | 最終プランニング軌跡（optimizer 後段出力） |

#### シミュレーションログ（`TOPICS["sim"]`）

Stage 3 (`step3_run_sims`) が `scenario_test_runner` で sim を回した結果の rosbag から、
`step1_make_lite.py` の `TOPICS["sim"]` で抽出して `lite/<run_tag>.lite/` を生成する。

| Topic name | 用途 |
| ---------- | ---- |
| `/system/operation_mode/state` | AUTONOMOUS 区間の切り出し |
| `/vehicle/status/velocity_status` | 速度応答 |
| `/vehicle/status/steering_status` | 操舵応答 |
| `/localization/kinematic_state` | 自車位置・軌跡 |
| `/localization/acceleration` | 加速度応答 |
| `/control/trajectory_follower/control_cmd` | trajectory_follower の制御指令 |
| `/control/command/control_cmd` | 制御指令（post-gate、実機との同段比較用） |
| `/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory` | DiffusionPlanner 出力軌跡（速度プロファイル分析用） |
| `/perception/traffic_light_recognition/traffic_signals` | 交通信号状態（DiffusionPlanner 入力） |

## 設定

`scenario.yaml` の `Evaluation.Conditions` で以下のキーを認識する。

| キー | 必須/任意 | 内容 |
| ---- | --- | ---- |
| `scenario_name` | 任意 | 図タイトルに表示するシナリオ名。未指定時は `ScenarioName` を使用。 |
| `curve_config_yaml` | 任意 | カーブ別解析設定 YAML への相対パス（`scenario.yaml` 基準）または絶対パス。空文字でカーブ別解析をスキップ。 |
| `cases_config` | **必須** | Stage 5/6（VehicleModel per-step 解析 + 集約）の `cases.yaml` への相対パス。 |
| `sim_runs_config` | **必須** | Stage 3/4（closed-loop sim + N-way 比較）の `sim_runs.yaml` への相対パス。 |
| `real_provenance` | 任意 | 実機データ取得時の pilot-auto.x2 / DiffusionPlanner 重みの自由記述。比較プロット・report.md の provenance に掲載し、sim 実行時の版・重み（自動取得）との差を解釈する。 |
| `traffic_signals` | 任意 (既定 `replay`) | 信号の扱い。`replay`=実機 bag の信号タイムシリーズを再現。`green`=全信号常時 green。`none`=scenario 側で信号をセットしない（`reproduce_perception` が信号を pose-sync 再生して所有する場合に使用）。sim 早期停止（旧称 D0）の真因は赤信号 replay の到達時刻 desync（実機が green 通過した信号に sim ego が赤で当たり永久停止）であり、`green`/`none` で周回を完走できる（live sim A/B 実測: replay=241m 停止 / green=519m 完走、実機 598.7m）。**赤信号停止も忠実に再現したい場合は `none` + `reproduce_perception: true`**（信号を ego-pose 同期で再生するため、実車が緑通過した位置は緑・赤停止した位置は赤になる）。 |
| `loop_waypoints` | 任意 (既定 0) | route 形状を強制する **実験オプション**（**D0 の修正ではない**）。Stage 2 が start+goal に加え実走軌跡の膨らみ位置へ N 個の中間 LanePosition waypoint を挿入する。D0（sim 早期停止）の真因は赤信号 replay であり routing ではない（lanelet graph に shortcut が無く start+goal でも route は周回全体を引く）ことが live sim で確定したため、D0 解消には `traffic_signals: green` を使う。 |
| `reproduce_perception` | 任意 (既定 false) | `true` で実機 input_bag の**先行車（tracked objects）と信号（traffic_signals）を ego-pose 同期**で各 sim に注入（`perception_reproducer_node` を Stage 3 が並走起動、`tracking/objects` / `traffic_signals` = DiffusionPlanner 入力に publish）。auto-scenario は NPC を持たないため、実機が先行車追従主体（cruise_following 等）の走行で sim ego が先行車不在により自由加速して実機より速くなるのを防ぎ、**実機の停止・加減速を再現**する。信号も pose-sync 再生するので、実車が緑通過した位置は緑・赤停止した位置は赤となり、**赤信号停止の忠実再現と D0 偽停止回避を両立**（その場合 `traffic_signals: none` と併用し scenario 側の信号設定を無効化）。アルゴリズム=走行中は pose-sync（lead を実相対位置に）/ego 停止中は記録を時間前進（dwell→departure を再生し ego を解放）。live 検証: 実機の先行車追従停止と速度エンベロープを再現し完走（実機 598m を ~586m/170s で追従、arc0-40m 平均速度 0.89m/s ≒ 実機 0.85 / 先行車無 green 1.79）。完全一致は real/sim の DiffusionPlanner 重み差により頭打ち。 |

- **`sim_runs.yaml`**（Stage 3/4）: closed-loop sim の run 定義。`vehicle_model` と任意の
  `params`（simulator_model 上書き）で run を増やす。
- **`cases.yaml`**（Stage 5/6）: per-step 解析のケース定義。`vehicle_model` タイプと `params`。

  両 YAML の書式・使える `vehicle_model` の種別は `sample/README.ja.md` を参照。

`curve_config_yaml`（任意）は以下の構造で記述する（`sample/curve_config_miraikan.yaml` 参照。
特定 area_map 前提のハードコードを含むため、別 map の dataset では空文字でスキップ）。

- `curve_centers`: 地図座標系でのカーブ中心リスト（`label`, `cx`, `cy`, `margin`）。`curves_closeup` プロット用。
- `curve2_index`: `curve_centers` 内で per-step delta 解析の対象とするカーブのインデックス（0 始まり）。
- `curve2_window`: カーブ② 直前の一時停止を検出する時刻窓 `[start, end]`（AUTONOMOUS 開始からの経過秒）。

## 出力（`result_archive/` 配下）

### `comparison/index.html`（閲覧用エントリポイント）

`step11_build_html_report`（Stage 11）が生成する、`comparison/` 配下の全プロットを 1 枚に集約した
HTML。ブラウザで開くと、目次から各セクション（`figures/`, `per_step/<tag>/`, `cases/`, `kus_sweep/`,
`brake_sweep/`, `curve_diag/`）の全 PNG をキャプション付きで一覧でき、3 種の Markdown レポート
（`report.md`, `cases_summary.md`, `curve_divergence.md`）も埋め込まれる。画像は**相対パスリンク**なので、
`comparison/` ディレクトリごとアーカイブ・共有してもそのまま表示できる。散在するプロットをディレクトリを
辿らずに確認するための入口として使う。

### `lite/`

`step1_make_lite` が抽出した lite bag（rosbag2 mcap）。実機 `real.lite/` と、Stage 3 が
`sim_runs.yaml` の各 run について生成する `<run_tag>.lite/`（例 `sim_normal.lite/`,
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

PNG / PDF の比較プロット。

| ファイル | 内容 |
| -------- | ---- |
| `velocity.{png,pdf}` | 速度指令 vs 応答 |
| `velocity_vs_distance.{png,pdf}` | 速度応答 vs 走行距離（arc-length 基準、早期停止を露出） |
| `acceleration.{png,pdf}` | 加速度指令 vs 応答 |
| `steering.{png,pdf}` | 操舵指令 vs 応答 |
| `steering_vs_distance.{png,pdf}` | 操舵応答 vs 走行距離（arc-length 基準） |
| `trajectory_with_map.{png,pdf}` | 地図背景上での軌跡重ね合わせ |
| `dp_real_vs_sim.png` | Stage 8: DiffusionPlanner 出力軌跡 実機 vs sim |
| `dp_vs_actual.png` | Stage 8: DP計画速度(d=0) vs actual速度 |
| `dp_vs_final_traj.png` | Stage 8: 実機 DP出力 vs 最終 planning（optimizer 補正） |
| `curves_closeup.{png,pdf}` | カーブ別の詳細拡大 |
| `curve{N}_analysis.{png,pdf}` | 指定カーブ 全体解析（{N}=`plot_curves[*].index + 1`） |
| `curve{N}_steering_detail.{png,pdf}` | 指定カーブ 操舵詳細 |
| `curve{N}_yaw_steer.{png,pdf}` | 指定カーブ ヨーレート・操舵関係 |
| `curve{N}_steer_response.{png,pdf}` | 指定カーブ ステアリング応答特性 |

> `curve{N}_*` 系は `curve_config_yaml::plot_curves` で対象カーブを切り替え可能。
> 未指定なら `curve2_index` のカーブだけ生成される（既定は `curve2_*` の 4 枚）。
> `curve_config_yaml` が空ならカーブ別プロットは生成されない。

### `comparison/per_step/<case_tag>/`

`step5_analyze_per_step`（Stage 5）によるケース別 per-step delta 解析の成果物。
1 ケースあたり `per_step_delta.csv` + 図 9 枚（うち `rollout_error_growth.png`）+ `rollout.csv`
（多段 free-running rollout の horizon 別誤差）+ `summary.txt`（per-step RMSE + rollout RMSE）。
ケースは `cases.yaml` で定義する。

> per-step は 1-step reset のため k_us/wheelbase に非感度。multi-step rollout（`run_free_rollout`）が
> N ステップ連続予測で dynamics 差を顕在化する。

### `comparison/cases/`

`step6_analyze_cases`（Stage 6）による全ケース集約解析の成果物。

| ファイル | 内容 |
|---|---|
| `cases_summary.md` | per-step RMSE 表（reference との Δsteer 付き）+ multi-step rollout RMSE 横断表（horizon 別 pos/yaw + Δyaw vs ref） |
| `overlay/cascade_error_overlay.png` | 全ケースを 1 枚に重ね描き（段階的誤差） |
| `overlay/error_timeseries_overlay.png` | 全ケースを 1 枚に重ね描き（誤差時系列） |

### `comparison/kus_sweep/`

`step7_identify_kus`（Stage 7）による k_us（アンダーステア係数）同定の成果物。実機 lite を
SSOT に、k_us グリッドで free-running rollout を回し、最大 horizon の yaw RMSE を最小化する
k_us を同定する（per-step は k_us 非感度なため rollout を使用）。

| ファイル | 内容 |
|---|---|
| `kus_sweep.csv` | k_us × horizon → yaw / 位置 / 横 RMSE |
| `kus_sweep.png` | yaw・位置 RMSE vs k_us 曲線（同定値を赤線表示） |

> 同定値はログ末尾に出力（グリッド最小 + 近傍 3 点の放物線サブグリッド推定）。最小がグリッド端の
> 場合は範囲拡大を警告。`--kus-values` / `--horizons` / `--stride` で手動調整可能。

### `comparison/brake_sweep/`

`step9_identify_brake`（Stage 9）による縦方向パラメータ（brake_time_constant）同定の成果物。実機 lite の
post-gate control_cmd をオープンループ入力に縦方向モデルを brake_tc グリッドで回し、発進直後の実機
actual velocity へのフィット RMSE を最小化する brake_tc を同定する。

| ファイル | 内容 |
|---|---|
| `brake_sweep.csv` | brake_tc × (RMSE / mean_err) |
| `brake_sweep.png` | 発進フィット RMSE vs brake_tc 曲線（同定値を赤線表示） |

> 注: brake_tc は減速時に支配的で発進窓では弱くしか拘束されない（ill-posed）。RMSE が単調減少し最小が
> 非物理的大値に張り付く場合は警告する（真の同定でなく launch 過大予測の代理；mean_err>0 を併読）。
> robust な同定には減速・停止窓が必要（将来拡張）。

### `comparison/curve_diag/`

`step10_diagnose_curve`（Stage 10）によるカーブ/発進区間の軌跡乖離詳細診断。実機 vs sim を発進基準で
整列し、乖離を実機進行方向基準の縦/横成分に分解（+ヨー差・速度差）して原因を切り分ける
（step4 のカーブ別図にはない診断次元）。

| ファイル | 内容 |
|---|---|
| `curve_divergence.md` | 縦/横乖離 peak/RMS・速度差 mean/RMS の定量サマリ |
| `curve_divergence.png` | 速度/速度差/ステア/ヨー差/乖離縦横分解の 5 段時系列 |

> 縦方向支配なら pacing/速度差、横方向支配なら操舵・understeer 由来を示唆。比較対象 sim は
> sim_*.lite を自動検出（sim_normal 優先）。sim/発進が無ければスキップ。

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

webauto で T4 dataset を pull 済みの環境で `make local_cloud_run` 一発で 10 段階すべて走る。
ローカルでは `lib/_dataset.py`（解決の SSOT）が webauto キャッシュ
`~/.webauto/data/data/annotation_dataset/<UUID>/<frame>` を解決し、クラウドと**同一の
`t4_dataset_path` 契約**を渡す。すなわち `t4_dataset_path` 以降の扱いはローカル/クラウドで
完全に共通で、両者の違いは「誰がパスを解決するか」（クラウド=Web.Auto ステージング /
ローカル=`lib/_dataset.py`）だけ。
手順詳細・`sim_runs.yaml`/`cases.yaml` の書式・トラブルシュート・Makefile 変数の上書き例は
[`sample/README.ja.md`](sample/README.ja.md) を参照。
