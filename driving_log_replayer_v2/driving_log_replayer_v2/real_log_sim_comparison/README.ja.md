# Real Log / Sim Comparison の評価

実機走行ログ（T4 dataset の `input_bag/` rosbag）を **SSOT** とし、同じ走行を
Autoware + シミュレータで closed-loop 再現して、実機との乖離を統計値と図で比較する
ユースケース。速度応答・操舵応答・軌跡精度・N-step オープンループ誤差を可視化する。

他ユースケースと異なり、launch から **Autoware・bag player・bag recorder のいずれも
起動しない**。評価ノード `real_log_sim_comparison_evaluator` が 10 段階パイプラインを
subprocess で直接実行する。

## パイプライン（10 段階）

`sample/scenario.yaml` の `Datasets[0]` UUID = SSOT（実機 rosbag を含む
annotation-dataset）が全 stage の入口になる。

> 全 stage の成果物は `result_archive/real_log_sim_comparison/` 配下の単一バンドルフォルダに
> まとめて出力する（下表の出力パスはこのバンドルフォルダ基準）。Web.Auto は `result_archive/` の
> 中身をそのまま zip 化するため、ラッパーフォルダが無いと展開時に散らばる。post_process が後段で
> `result_archive/` 直下に書く `result_jsonl.png` のみバンドル外に残る。

| Stage | 名称 | 入力 | 出力 (`result_archive/real_log_sim_comparison/` 配下) | 実行回数 |
|---|---|---|---|---|
| 1 | 実機ログ抽出 (`step1_make_lite --kind real`) | `input_bag/*.{mcap,db3}` | `lite/real.lite/` | 1 |
| 2 | scenario 自動生成 (`step2_bag_to_scenario`) | `input_bag/` + map | `scenarios/auto_scenario.yaml` (OpenSCENARIO) | 1 |
| 3 | closed-loop シム実行 (`step3_run_sims`) | `auto_scenario.yaml` + `sim_runs.yaml` の 1 run | `lite/<run_tag>.lite/` | N (sim_runs) |
| 4 | 実機 + sim 比較解析 (`step4_compare_logs`) | `lite/{real, <run_tag>}.lite/` 群 | `comparison/{figures/, report.md}` (N-way 重ね描き) | 1 |
| 5 | VehicleModel N-step オープンループ解析 (`step5_analyze_nstep`) | `lite/real.lite/` + `cases.yaml` の 1 ケース | `comparison/nstep/<tag>/` | N (cases) |
| 6 | ケース集約解析 (`step6_analyze_cases`) | `nstep/<tag>/nstep_delta.csv` 群 | `comparison/cases/{overlay/, cases_summary.md}` | 1 |
| 7 | 車両モデルパラメータ sweep 同定 (`step7_sweep_params`) | `lite/real.lite/` | `comparison/param_sweep/{<param>_sweep.{csv,svg}, pair_*.{csv,svg}, param_sweep_summary.md}` | 1 |
| 8 | DP軌跡比較 (`step8_compare_dp_trajectory`) | `lite/{real, <sim>}.lite/` | `comparison/figures/dp_*.svg` | 1 |
| 9 | 縦パラ同定 (`step9_identify_brake`) | `lite/real.lite/` | `comparison/brake_sweep/{brake_sweep.csv, .svg}` | 1 |
| 10 | カーブ乖離診断 (`step10_diagnose_curve`) | `lite/{real, <sim>}.lite/` | `comparison/curve_diag/{curve_divergence.md, .svg}` | 1 |
| 11 | HTML レポート生成 (`step11_build_html_report`) | `comparison/` 配下の全 SVG/plotly HTML + 各 `.md` | `index.html` (バンドルフォルダ直下) | 1 |

成否はパイプラインの例外有無で決まる。全 stage が完走すれば `result.jsonl` に
`Success: true`、いずれかの subprocess が非ゼロ終了またはタイムアウトすると
`Success: false` と Python traceback が記録される。

**closed-loop sim の終了条件** は Stage 2 が auto-scenario に書き込む（「厳密さより同じコースで
確実に切り上げる」優先）。ゴール厳密到達 (`--goal-tolerance` 既定 15m) に加え、一定時間経過後に
ゴール近傍 (`--goal-vicinity-tolerance` 既定 30m) へ入れば停止・通過・周回いずれでも切り上げる
（exitSuccess）。これで「ゴール付近で止まってもタイムアウトせず」「通過しても周回し続けない」を
両立する。開始直後の誤発火（ループ経路は start≈goal）は実走 course 時間に基づく時間ゲート
（`max(30s, 0.5×course)`）で防ぐ。どの条件にも掛からず `--sim-timeout`（既定 600s）に達した
場合のみ exitFailure。長時間静止での切り上げ (`--standstill-timeout`) は **既定無効の opt-in**
（近傍停止は vicinity で捕捉済み。有効化すると `reproduce_perception` の先行車 dwell 再現を途中で
切り上げてしまうため非推奨）。（`TraveledDistanceCondition` は scenario_simulator_v2 未サポートの
ため走行距離での切り上げは不可。）

ゴール到達判定の Position は teleport/routing と同じ **LanePosition**（`_world_to_lane_position`）
で書く。`ReachPositionCondition` は 3D の `hypot(x,y,z)` で距離判定し WorldPosition の z を
literal 使用するため、map が標高（例 ~40m）を持つ dataset では `z=0` の WorldPosition だと ego の
map-pose z との差で**永久に発火しなかった**（2026-06-03 判明）。LanePosition は z を map から得るので
発火する。また Stage 3 は scenario_test_runner の `global_timeout`（scenario 実行の壁時計上限、
既定 180s）を `max(180, timeout_s − initialize_duration − 60)` に引き上げる。本 sim は実機の約 3 倍
遅く 180s 壁ではゴール到達前に打ち切られるため。実終了は scenario の exitSuccess、global_timeout は
外側の壁セーフティ。

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
| `cases_config` | **必須** | Stage 5/6（VehicleModel N-step 解析 + 集約）の `cases.yaml` への相対パス。 |
| `sim_runs_config` | **必須** | Stage 3/4（closed-loop sim + N-way 比較）の `sim_runs.yaml` への相対パス。 |
| `real_provenance` | 任意 | 実機データ取得時の pilot-auto.x2 / DiffusionPlanner 重みの自由記述。比較プロット・report.md の provenance に掲載し、sim 実行時の版・重み（自動取得）との差を解釈する。 |
| `traffic_signals` | 任意 (既定 `replay`) | 信号の扱い。`replay`=実機 bag の信号タイムシリーズを再現。`green`=全信号常時 green。`none`=scenario 側で信号をセットしない（`reproduce_perception` が信号を pose-sync 再生して所有する場合に使用）。sim 早期停止（旧称 D0）の真因は赤信号 replay の到達時刻 desync（実機が green 通過した信号に sim ego が赤で当たり永久停止）であり、`green`/`none` で周回を完走できる（live sim A/B 実測: replay=241m 停止 / green=519m 完走、実機 598.7m）。**赤信号停止も忠実に再現したい場合は `none` + `reproduce_perception: true`**（信号を ego-pose 同期で再生するため、実車が緑通過した位置は緑・赤停止した位置は赤になる）。 |
| `loop_waypoints` | 任意 (既定 0) | route 形状を強制する **実験オプション**（**D0 の修正ではない**）。Stage 2 が start+goal に加え実走軌跡の膨らみ位置へ N 個の中間 LanePosition waypoint を挿入する。D0（sim 早期停止）の真因は赤信号 replay であり routing ではない（lanelet graph に shortcut が無く start+goal でも route は周回全体を引く）ことが live sim で確定したため、D0 解消には `traffic_signals: green` を使う。 |
| `reproduce_perception` | 任意 (既定 false) | `true` で実機 input_bag の**先行車（tracked objects）と信号（traffic_signals）を ego-pose 同期**で各 sim に注入（`perception_reproducer_node` を Stage 3 が並走起動、`tracking/objects` / `traffic_signals` = DiffusionPlanner 入力に publish）。auto-scenario は NPC を持たないため、実機が先行車追従主体（cruise_following 等）の走行で sim ego が先行車不在により自由加速して実機より速くなるのを防ぎ、**実機の停止・加減速を再現**する。信号も pose-sync 再生するので、実車が緑通過した位置は緑・赤停止した位置は赤となり、**赤信号停止の忠実再現と D0 偽停止回避を両立**（その場合 `traffic_signals: none` と併用し scenario 側の信号設定を無効化）。アルゴリズム=走行中は pose-sync（lead を実相対位置に）/ego 停止中は記録を時間前進（dwell→departure を再生し ego を解放）。live 検証: 実機の先行車追従停止と速度エンベロープを再現し完走（実機 598m を ~586m/170s で追従、arc0-40m 平均速度 0.89m/s ≒ 実機 0.85 / 先行車無 green 1.79）。完全一致は real/sim の DiffusionPlanner 重み差により頭打ち。 |

- **`sim_runs.yaml`**（Stage 3/4）: closed-loop sim の run 定義。`vehicle_model` と任意の
  `params`（simulator_model 上書き）で run を増やす。
- **`cases.yaml`**（Stage 5/6）: N-step 解析のケース定義。`vehicle_model` タイプと `params`。

  両 YAML の書式・使える `vehicle_model` の種別は `sample/README.ja.md` を参照。

`curve_config_yaml`（任意）は以下の構造で記述する（`sample/curve_config_miraikan.yaml` 参照。
特定 area_map 前提のハードコードを含むため、別 map の dataset では空文字でスキップ）。

- `curve_centers`: 地図座標系でのカーブ中心リスト（`label`, `cx`, `cy`, `margin`）。`curves_closeup` プロット用。
- `curve2_index`: `curve_centers` 内で N-step 解析の対象とするカーブのインデックス（0 始まり）。
- `curve2_window`: カーブ② 直前の一時停止を検出する時刻窓 `[start, end]`（AUTONOMOUS 開始からの経過秒）。

## 出力（`result_archive/real_log_sim_comparison/` 配下）

全成果物は `result_archive/real_log_sim_comparison/` の単一バンドルフォルダにまとめて出力する
（zip 展開で 1 フォルダにまとまるようにするため。理由は冒頭パイプライン表の注記参照）。以下のパスは
すべてこのバンドルフォルダ基準。

### `index.html`（バンドルフォルダ直下・閲覧用エントリポイント）

`step11_build_html_report`（Stage 11）がバンドルフォルダ直下（`comparison/` の親）に生成する、
`comparison/` 配下の全プロットを 1 枚に集約した HTML（画像 src は `comparison/` 始まりの相対パス）。
出力ディレクトリ単位ではなく **比較の概念** で 5 セクションに分けて並べる（読み手が
「何を何と比べた図か」で辿れるようにするため）。各図は出力先ではなく (ディレクトリ, ファイル名) の
組で分類するので、`figures/` に混在する closed-loop 図・DP 図・brake 同定図も正しいセクションに振り分く。

| セクション | 内容 | 主な図（出力元 step） |
|---|---|---|
| 1. 実機 rosbag 解析 | 実機ログ(SSOT)のみから抽出する特性・車両パラメータ同定（sim 非介在） | `param_sweep`(7) / `brake_sweep`・`departure_brake_tc_sensitivity`・`real_cmd_acc_departure`(9) |
| 2. プランナ出力比較 | DiffusionPlanner 出力軌跡を実走・最終 planning・sim 出力と比較 | `dp_real_vs_sim`・`dp_vs_actual`・`dp_vs_final_traj`(8) |
| 3. N-step オープンループ比較 | 実機状態リセットから N ステップ連続予測した車両モデルの差（N=1 = 毎ステップリセット） | nstep の `overview`/`error_*`/`steering_analysis`/`map_distribution`/`lateral_*`/`cascade_error`(5) + `cases/overlay`(6) |
| 5. シナリオ クローズループ比較 | auto-scenario を sim で closed-loop 実行した実機との乖離 | `velocity`/`acceleration`/`steering`/`*_vs_distance`/`trajectory_with_map`/`trajectory_playback`/`curves_closeup`/`curve{N}_*`(4) + `curve_divergence`(10) |

各セクションには 1 行説明を付ける。見やすさのための機能（レポート自体は**純 CSS/HTML 実装・CDN 不使用・
オフライン可**、CSS 無効環境でも degrade して全内容を閲覧できる。plotly 図のみバンドル同梱の
`plotly.min.js` を使う）:

- **プロット単位ケースタブ**（セクション 3）: `nstep/<case>/` の図をプロット種別ごとにまとめ、
  セクション先頭の「ケース切替: `baseline`/`ideal_steer`/`kus0020`/`shorter_wb` …」を選ぶと、その
  セクション内**全ブロックの図が一斉に**そのケースへ切り替わる（ラジオ＋`case-<slug>` クラス対応）。
- **セクション折りたたみ**: 各セクション・Markdown レポートは `<details open>` で開閉できる。
- **画像の拡大表示**: SVG 図のサムネをクリックすると `:target` ライトボックスで拡大（オーバーレイは
  `:target` が効くよう `<main>` 末尾に一括配置）。「原寸を開く」で元 SVG も開ける。
- **インタラクティブ図**: マップ上プロット（`trajectory_with_map` / `map_distribution`）は plotly
  製のスタンドアロン HTML を `<iframe>` で埋め込み、ズーム・パン・ホバー・凡例トグルができる
  （ライトボックス対象外。バンドル直下に同梱する `plotly.min.js` を相対参照するためオフライン可）。
- **サイド目次**: sticky 追従。「↑ 先頭へ」と各セクションの「↑ 先頭」リンク付き。セクションは太字、
  Markdown サブ項目はインデント表示。（スクロール位置の自動ハイライトは JS が必要なため未実装。）

3 種の Markdown レポート（`report.md`→5、`cases_summary.md`→3、`curve_divergence.md`→5）は所属セクション
末尾に折りたたみで埋め込む。既知のいずれにも分類されない図は捨てず「その他」セクションに回す
（黙って誤分類しない）。画像は**相対パスリンク**なので、バンドルフォルダ
（`result_archive/real_log_sim_comparison/`）ごとアーカイブ・共有してもそのまま表示できる。

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

SVG の比較プロット（軌跡比較のみ plotly 製インタラクティブ HTML）。

| ファイル | 内容 |
| -------- | ---- |
| `velocity.svg` | 速度指令 vs 応答 |
| `velocity_vs_distance.svg` | 速度応答 vs 走行距離（arc-length 基準、早期停止を露出） |
| `acceleration.svg` | 加速度指令 vs 応答 |
| `steering.svg` | 操舵指令 vs 応答 |
| `steering_vs_distance.svg` | 操舵応答 vs 走行距離（arc-length 基準） |
| `trajectory_with_map.html` | 地図背景上での軌跡重ね合わせ（plotly・ズーム/パン/ホバー可） |
| `trajectory_playback.html` | 軌跡再生ビューア（plotly 非依存の自己完結 canvas+JS）。シークバー・再生で各 run の現在位置を地図上に重ね表示。**時刻同期**（同一経過時刻）と**位置同期**（実機の走行距離 s 基準に各 run を自軌跡の弧長一致地点へ表示＝同一走行距離での横ずれ比較）の 2 モード。速度は進行方向矢印（長さ = v×1.5s 到達距離）と凡例の v/t/s 読み出しで表示。追従ズーム（表示中 run を自動フィット、視野幅調整可）/全体表示トグル・run 表示切替付き |
| `dp_real_vs_sim.svg` | Stage 8: DiffusionPlanner 出力軌跡 実機 vs sim |
| `dp_vs_actual.svg` | Stage 8: DP計画速度(d=0) vs actual速度 |
| `dp_vs_final_traj.svg` | Stage 8: 実機 DP出力 vs 最終 planning（optimizer 補正） |
| `curves_closeup.svg` | カーブ別の詳細拡大 |
| `curve{N}_analysis.svg` | 指定カーブ 全体解析（{N}=`plot_curves[*].index + 1`） |
| `curve{N}_steering_detail.svg` | 指定カーブ 操舵詳細 |
| `curve{N}_yaw_steer.svg` | 指定カーブ ヨーレート・操舵関係 |
| `curve{N}_steer_response.svg` | 指定カーブ ステアリング応答特性 |

> `curve{N}_*` 系は `curve_config_yaml::plot_curves` で対象カーブを切り替え可能。
> 未指定なら `curve2_index` のカーブだけ生成される（既定は `curve2_*` の 4 枚）。
> `curve_config_yaml` が空ならカーブ別プロットは生成されない。

### `comparison/nstep/<case_tag>/`

`step5_analyze_nstep`（Stage 5）によるケース別 N-step オープンループ解析の成果物。
1 ケースあたり `nstep_delta.csv`（全 horizon 統一スキーマ）+ 図 9 枚（うち `map_distribution.html` は plotly）
+ `summary.txt`（N=1 詳細 RMSE + horizon 別 RMSE）。
ケースは `cases.yaml` で定義する。

> N=1 は毎ステップ reset のため k_us/wheelbase に非感度。大 N の rollout（`run_rollout`）が
> N ステップ連続予測で dynamics 差を顕在化する。

### `comparison/cases/`

`step6_analyze_cases`（Stage 6）による全ケース集約解析の成果物。

| ファイル | 内容 |
|---|---|
| `cases_summary.md` | N=1 詳細 RMSE 表（reference との Δsteer 付き）+ horizon 別 終端誤差 RMSE 横断表（pos/yaw + Δyaw vs ref） |
| `overlay/cascade_error_overlay.svg` | 全ケースを 1 枚に重ね描き（段階的誤差） |
| `overlay/error_timeseries_overlay.svg` | 全ケースを 1 枚に重ね描き（誤差時系列） |

### `comparison/param_sweep/`

`step7_sweep_params`（Stage 7）による車両モデルパラメータ sweep 同定の成果物。実機 lite を
SSOT に、各パラメータ（k_us / steer_time_constant / steer_time_delay / steer_bias /
steer_dead_band / debug_steer_scaling_factor / acc_time_constant / acc_time_delay /
debug_acc_scaling_factor。wheelbase は実測値が正しいため固定）をグリッドで sweep して
free-running rollout を回し、最大 horizon の終端誤差 RMSE（横方向系=yaw/横、縦方向系=縦）を
最小化する値を同定する（N=1 は dynamics パラメータに非感度なため大 N の rollout を使用）。
k_us × steer_time_constant / k_us × debug_steer_scaling_factor の 2D ペア sweep ヒートマップも生成する。

| ファイル | 内容 |
|---|---|
| `<param>_sweep.csv` | param 値 × horizon → yaw / 位置 / 横 / 縦 RMSE |
| `<param>_sweep.svg` | 同定メトリクス・位置 RMSE vs param 曲線（同定値=赤線、仕様値=灰点線）+ 実機ログ根拠パネル（例: k_us は横加速度 vs ステア残差の understeer 勾配。sweep と独立な生データ直接観察によるクロスチェック） |
| `pair_<a>_<b>.{csv,svg}` | 2D ペア sweep（RMSE ヒートマップ + 最小点） |
| `param_sweep_summary.md` | 全パラメータの同定値 vs 仕様値の一覧表 |

> 同定値はログ末尾に出力（グリッド最小 + 近傍 3 点の放物線サブグリッド推定）。最小がグリッド端の
> 場合は範囲拡大を警告。`--kus-values` / `--horizons` / `--stride` で手動調整可能。

### `comparison/brake_sweep/`

`step9_identify_brake`（Stage 9）による縦方向パラメータ（brake_time_constant）同定の成果物。実機 lite の
post-gate control_cmd をオープンループ入力に縦方向モデルを brake_tc グリッドで回し、発進直後の実機
actual velocity へのフィット RMSE を最小化する brake_tc を同定する。

| ファイル | 内容 |
|---|---|
| `brake_sweep.csv` | brake_tc × (RMSE / mean_err) |
| `brake_sweep.svg` | 発進フィット RMSE vs brake_tc 曲線（同定値を赤線表示） |

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
| `curve_divergence.svg` | 速度/速度差/ステア/ヨー差/乖離縦横分解の 5 段時系列 |

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
