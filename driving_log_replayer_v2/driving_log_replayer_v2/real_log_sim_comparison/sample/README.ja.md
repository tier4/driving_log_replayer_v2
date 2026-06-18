# real_log_sim_comparison サンプル (cloud / local 共通)

`scenario.yaml` / `curve_config_miraikan.yaml` は
cloud（Web.Auto evaluator）と local（`make local_cloud_run`）で**共通の単一サンプル**。

- **cloud**: これらは CMakeLists で `share/driving_log_replayer_v2/sample/real_log_sim_comparison/`
  に install され、登録済み Web.Auto scenario の `Conditions` がその install-share 絶対パスを参照する。
- **local**: `make local_cloud_run` が本ディレクトリの `scenario.yaml`（モデル定義・cases/sim_runs リストは
  `Conditions` 内に全てインライン記述）をそのまま使い、同じパイプラインをワンコマンド実行する。

Godot バイナリパスは cloud / local 共通で `/opt/godot_autoware_simulator/` に統一。cloud は
`.webauto-ci` が自動配置、local は同じ `/opt` に配置（または symlink）する。
`curve_config_miraikan.yaml` は `scenario.yaml` の `Conditions.curve_config_yaml` で参照する
（現状サンプルは空文字で無効）。

---

## パイプライン概要

10 段階パイプラインの各 stage の入出力は親ディレクトリの
[`../README.ja.md`](../README.ja.md#パイプライン10-段階) を参照。ローカル実行で押さえる点:

- Stage 3（sim 実行）と Stage 5（N-step 解析）はそれぞれ `Conditions.sim_runs` /
  `Conditions.cases` の各エントリで N 回ループする。これらは `scenario.yaml` の
  `Conditions.models` に定義されたモデル名リストで参照する。**両方未指定だとパイプラインは失敗する**。

> **実行時間**: Stage 3 は scenario_test_runner で Autoware を起動して closed-loop
> シムを回すため 1 run あたり ~5 分。既定 `scenario.yaml` は 7 sim_runs（normal /
> kus0020 / best_normal / taiga_dyn / taiga_x / perfect / godot）のため、end-to-end は ~40 分の見積もり。

---

## 前提

1. **このリポジトリを含む colcon ワークスペースをビルド済み**。
   `${WS_ROOT}/install/setup.bash` が存在すること。
2. **`webauto` CLI で認証済み**。
   `webauto auth` 系コマンドが通る状態。

## 手順

### 1. 対象走行の T4 データセットを `--include-intermediate-artifacts` 付きで pull

`scenario.yaml` の `Datasets:` 先頭キーに書かれた UUID と一致する
`annotation-dataset-id` を指定する：

```bash
webauto data annotation-dataset pull \
    --project-id x2_dev \
    --annotation-dataset-id <UUID> \
    --include-intermediate-artifacts
```

> **重要 — デフォルト UUID をそのまま使う前に確認すること**
>
> `scenario.yaml` の `Datasets[0]` UUID = SSOT (実機 rosbag) として全 stage で
> 使われる。サンプルの UUID は動作確認用の選択値なので、評価対象が異なる場合は
> **必ず自分の評価対象 annotation-dataset の UUID に書き換えること**。
>
> 比較レポート (`comparison/report.md`) や `trajectory_with_map.png` で実機と
> シム軌跡がまったく違う場所にプロットされている場合、この UUID が誤っている
> 可能性が高い。

`--include-intermediate-artifacts` を付けないと `input_bag/` に bag 本体
(`*.mcap` / `*.db3`) が含まれず、`step1_make_lite` の入力が足りずに失敗する。

ダウンロード結果は `~/.webauto/data/data/annotation_dataset/<UUID>/<frame>/` 配下に
`annotation/`, `data/`, `input_bag/`, `map/` が展開される。`make local_cloud_run` は
`lib/_dataset.py`（解決の SSOT）でこの `<UUID>/<frame>` を解決し、`input_bag/` と
`map/` を直下に持つディレクトリを `t4_dataset_path` として launch に渡す。これは
クラウド（Web.Auto が dataset を固定マウントに事前ステージして渡す path）と**同一の
契約**で、`t4_dataset_path` 以降の扱いはローカル/クラウドで共通。複数 frame があれば
最新（名前 sort の末尾）を採用する。

### 2. scenario.yaml の Conditions.models を確認/編集

モデル定義・cases/sim_runs リストは `scenario.yaml` の `Conditions` にインラインで記述する。
外部ファイル（`cases.yaml` / `sim_runs.yaml`）は存在しない。

```yaml
Conditions:
  # ...scenario_name, real_provenance, traffic_signals 等...

  # ── モデルレジストリ ──────────────────────────────────────────
  # 同列に名前付きで定義。cases / sim_runs は名前で呼び出す。
  #
  # フィールド:
  #   vehicle_model_type: open-loop VehicleModel クラス名 (cases で必須)
  #   vehicle_model:       closed-loop description パッケージ名 (sim_runs で必須)
  #   params:              open/sim 共有の simulator_model パラメータ上書き (任意)
  #   architecture_type:   sim 専用 (既定 awf/universe/20250130)
  #   godot_executable:    sim 専用 (*_godot のとき必須)
  #   dp_model_dir/release/package: sim 専用 DiffusionPlanner モデル切替 (任意)
  models:
    normal:
      vehicle_model_type: delay_steer_acc_geared_wo_fall_guard
      vehicle_model: j6_gen2
      architecture_type: awf/universe/20250130
    kus0020:
      vehicle_model_type: delay_steer_acc_geared_wo_fall_guard
      vehicle_model: j6_gen2
      params: {k_us: 0.020}
    best_normal:
      vehicle_model_type: delay_steer_acc_geared_wo_fall_guard
      vehicle_model: j6_gen2
      params: {k_us: 0.018, steer_time_constant: 0.15, debug_steer_scaling_factor: 1.0, acc_time_constant: 0.30}
    no_delay:                          # cases 専用 (vehicle_model なし → sim_runs に含められない)
      vehicle_model_type: delay_steer_acc_geared_wo_fall_guard
      params: {acc_time_delay: 0.0, acc_time_constant: 0.0, steer_time_delay: 0.0, steer_time_constant: 0.0, steer_bias: 0.0}
    taiga_dyn:
      vehicle_model_type: taiga_dyn    # sim では TAIGA_DYN enum を自動注入
      vehicle_model: j6_gen2
    taiga_x:
      vehicle_model_type: taiga_x      # sim では TAIGA_X enum を自動注入
      vehicle_model: j6_gen2
    perfect:                           # sim 専用 (vehicle_model_type なし → cases に含められない)
      vehicle_model: j6_gen2_perfect_tracker
    godot:                             # sim 専用
      vehicle_model: j6_gen2_godot
      godot_executable: /opt/godot_autoware_simulator/godot_autoware_simulator.x86_64

  cases:    [normal, kus0020, best_normal, no_delay, taiga_dyn, taiga_x]
  sim_runs: [normal, kus0020, best_normal, taiga_dyn, taiga_x, perfect, godot]
  overlay:
    reference_tag: normal
    plots: [cascade_error, error_timeseries]
```

**`vehicle_model_type`（open-loop クラス名）** は `delay_steer_acc_geared_wo_fall_guard` /
`ideal_steer_acc` / `taiga_dyn` / `taiga_x` の 4 種類。`params` で未指定のキーは
`load_sim_params()` (j6_gen2_description の YAML) で補完（`wheelbase` は不要）。

**`vehicle_model`（sim description パッケージ名）** は `j6_gen2` (通常シム) /
`j6_gen2_perfect_tracker` (理想軌跡追従) / `j6_gen2_godot` (Godot シム)。
Godot 実行時は `godot_executable` パスが必要。`params` は `simple_sensor_simulator.<key>:=<value>` として
launch に渡り、description の simulator_model.param.yaml を上書きする。

**DiffusionPlanner モデルの切り替え**（同一車両モデルで DP モデルだけ変えてモデル選定したい場合）:
- `dp_model_release`: Web.Auto ML パッケージの release 名。指定すると Stage 3 が `webauto ml package-release`
  で **search→pull して自動取得**し、そのモデルで走らせる（`.webauto-ci.yml` 編集不要）。
  `dp_model_package` は package 名（既定 `diffusion_planner_for_x2_exp`）。
- `dp_model_dir`: 既にローカルに置いた DP モデルディレクトリ（`diffusion_planner.onnx` + `args.json`）を直接指定。
  `dp_model_release` とは排他。
- いずれも未指定なら Autoware 既定モデル。詳細・前提（変更 A のクラウド反映、認証等）は
  [`../docs/diffusion_planner_model_swap.ja.md`](../docs/diffusion_planner_model_swap.ja.md) を参照。

### 3. 実行

```bash
# use case ディレクトリに集約された Makefile を直接呼ぶ
cd src/simulator/driving_log_replayer_v2/driving_log_replayer_v2/driving_log_replayer_v2/real_log_sim_comparison
make local_cloud_run
```

リポジトリ root から走らせる場合は `make -C` を使う:

```bash
make -C src/simulator/driving_log_replayer_v2/driving_log_replayer_v2/driving_log_replayer_v2/real_log_sim_comparison local_cloud_run
```

**解析だけ再実行する場合**（sim 実行済みの out/ を再利用し Stage 4〜11 のみ。
解析コード変更後の作り直しが数分で済む）:

```bash
make local_analysis_run                                    # sample/out/latest を再解析
make local_analysis_run OUT_DIR=sample/out/<timestamp>     # 対象を指定
```

主要な動作：

1. `scenario.yaml` から Datasets UUID を取得 → `~/.webauto/.../` から実体パス解決
2. `out/<タイムスタンプ>/` を作成し、`out/latest` シンボリックリンクを更新
3. `ros2 launch` で 10 段階パイプライン起動
   - Stage 1: 実機 bag → lite/real.lite
   - Stage 2: step2_bag_to_scenario → scenarios/auto_scenario.yaml
   - Stage 3: Conditions.sim_runs の各 run で scenario_test_runner → lite/<tag>.lite
   - Stage 4: step4_compare_logs → comparison/figures/, report.md (real + sim 重ね描き)
   - Stage 5: Conditions.cases の各 tag で step5_analyze_nstep → nstep/<tag>/
   - Stage 6: step6_analyze_cases → cases/overlay/, cases_summary.md
   - Stage 7: step7_sweep_params → comparison/param_sweep/ (パラメータ sweep 同定; 追加設定不要)
   - Stage 8: step8_compare_dp_trajectory → comparison/figures/dp_*.png (DP軌跡 real vs sim)
   - Stage 9: step9_identify_brake → comparison/brake_sweep/ (縦方向 brake_tc 同定)
   - Stage 10: step10_diagnose_curve → comparison/curve_diag/ (カーブ乖離 縦横分解診断)

### 4. 結果確認

出力は use case 集約ディレクトリ配下の `sample/out/` に書かれる
(`driving_log_replayer_v2/driving_log_replayer_v2/real_log_sim_comparison/sample/out/`)。

```text
sample/out/latest/
├── result.jsonl                       # 末尾行に {"Result":{"Success":true,...}}
└── result_archive/
    ├── lite/
    │   ├── real.lite/*.mcap              # Stage 1
    │   └── <run_tag>.lite/*.mcap         # Stage 3: Conditions.sim_runs の各 run
    │                                     #   (既定 normal / kus0020 / best_normal / taiga_dyn / taiga_x / perfect / godot)
    ├── scenarios/auto_scenario.yaml      # Stage 2
    └── comparison/
        ├── report.md                     # Stage 4: 比較統計レポート
        ├── figures/*.png                 # Stage 4: 速度・操舵・軌跡, Stage 8: dp_*.png
        ├── nstep/
        │   └── <case_tag>/{*.svg + map_distribution.html, nstep_delta.csv, summary.txt}  # Stage 5
        ├── cases/
        │   ├── overlay/{cascade_error_overlay.png, error_timeseries_overlay.png}
        │   └── cases_summary.md          # Stage 6: N=1 RMSE + horizon 別 RMSE 横断表
        ├── param_sweep/{<param>_sweep.{csv,svg}, pair_*.{csv,svg}, param_sweep_derivation.md(§1 導出), param_sweep_summary.md(§2-§4 追認・整合)}  # Stage 7
        ├── brake_sweep/{brake_sweep.csv, brake_sweep.png} # Stage 9: 縦方向 brake_tc 同定
        └── curve_diag/{curve_divergence.md, curve_divergence.png} # Stage 10: カーブ乖離診断
```

## 上書き可能な Makefile 変数

| 変数 | 既定値 | 用途 |
|---|---|---|
| `WEBAUTO_T4_ROOT` | `$HOME/.webauto/data/data/annotation_dataset` | webauto annotation-dataset pull の出力ルート |
| `LOCAL_SCENARIO` | `<Makefile dir>/sample/scenario.yaml` | 別の scenario.yaml を使う場合 |
| `WS_ROOT` | `Makefile` の 6 階層上 | colcon ワークスペースルート |

例：
```bash
make local_cloud_run LOCAL_SCENARIO=/path/to/other_scenario.yaml
```

## トラブルシューティング

| 症状 | 対処 |
|---|---|
| `dataset <uuid> が見つかりません` | `webauto data annotation-dataset pull --include-intermediate-artifacts` を実行 |
| `input_bag/ に *.mcap/*.db3 が無い` | 同上。`--include-intermediate-artifacts` を付け忘れている可能性 |
| `install/setup.bash が見つかりません` | colcon ワークスペースを先にビルド |
| 図に地図背景が出ない | `<frame>/map/lanelet2_map.osm` が存在するか確認 |
| `Conditions.models に未定義名` | `scenario.yaml` の `Conditions.cases` / `sim_runs` に挙げた名前が `models` に存在するか確認 |
| `cases に vehicle_model_type なし` | `cases` に挙げたモデルに `vehicle_model_type` が必要。open-loop クラス名を追加する |
| `sim_runs に vehicle_model なし` | `sim_runs` に挙げたモデルに `vehicle_model` が必要。description パッケージ名を追加する |
| `未対応の model_type` | `vehicle_model_type` は `delay_steer_acc_geared_wo_fall_guard` / `ideal_steer_acc` / `taiga_dyn` / `taiga_x` のみ。新規 model 追加は `vehicle_model_c_wrapper.cpp` への factory 追加が必要 |
| Stage 3 (sim) が全 run 失敗 | scenario_test_runner が起動できているか、auto_scenario.yaml が valid か `ros2 launch scenario_test_runner scenario_test_runner.launch.py scenario:=<...auto_scenario.yaml>` を手動で試行 |
| sim 1 run で 10 分以上かかる | `Conditions.models.<name>.timeout_s` を上げるか、`initialize_duration` を下げて確認 |

## 実機課題別 再現検証シナリオ

各実機課題は**課題が観測された実機データセット（rosbag）にしか含まれない**ため、
専用の scenario.yaml を使って個別に実行する方式を採る。可視化は既存の
curve_config + step4/step10 の領域別解析を再利用し、新ステップは追加しない。

### ① カーブ大回り（具体シナリオ・実行可能）

**ファイル**: `sample/scenario_curve_wide_turn.yaml`（models/cases/sim_runs をインライン記述）

**検証内容**:
テレポート駅前交差点の左折（curve2: cx=89301, cy=43085）で、DiffusionPlanner の
3 モデル（0303/0503/0410）を比較して大回りの再現性を検証する。

| run tag | 実機での挙動 | 検証軸 |
|---|---|---|
| `dp_0303` | 大回りなし | FN 検証（sim でも大回りしないこと） |
| `dp_0503` | 大回りなし | FN 検証（同上） |
| `dp_0410` | 大回りあり | TP 検証（sim でも大回りが再現されること） |

**実行**:
```bash
# 事前に dataset を pull
webauto data annotation-dataset pull --project-id x2_dev \
  --annotation-dataset-id 28443458-8d02-476d-b91c-528ea6027d18 \
  --include-intermediate-artifacts

# 専用シナリオで実行（クラウド評価推奨）
make local_cloud_run LOCAL_SCENARIO=$(pwd)/sample/scenario_curve_wide_turn.yaml
```

**結果の確認** — `report.html` の「4. シナリオ クローズループ比較」セクション:
- `curve2_analysis`: 実機 vs 3 sim の軌跡ズーム（大回りの有無を目視）
- `curve2_steering_detail` / `curve2_yaw_steer`: ステア角・yaw 差の時系列
- `curve_diag/curve_divergence.md`: 縦/横乖離の定量値（横乖離が大きければ大回り）

---

### ②〜⑤ その他の課題（テンプレート）

**ファイル**: `sample/scenario_issue_template.yaml`

このファイルをコピーして `scenario_issue_<課題名>.yaml` にリネームし、
**Dataset UUID** と **curve_config_yaml** を編集して使う。

| 課題 | 停止→発進 | curve_config の設定 | 確認する図 |
|---|---|---|---|
| ②停止位置手前 | あり | `curve_centers` に停止線付近を定義 + `plot_curves.launch_window` | `curveN_analysis` + `curve_divergence.md` |
| ③急ブレーキ/ジリジリ | あり | 同上 | `curveN_analysis`（accel パネル）+ `curve_divergence.md` |
| ④加速不足 | なし | `curve_centers` に加速区間を定義（`launch_window` は省略可） | `curves_closeup`（軌跡ズーム）+ `velocity.fig.json` / `velocity_vs_distance.fig.json` |
| ⑤発進停止振動 | なし | `curve_centers` に発進窓付近を定義（または空文字でスキップ） | `velocity.fig.json` / `acceleration.fig.json`（全軌跡 overlay） |

**curve_config の例（停止線付近・②③用）**:
```yaml
curve_centers:
  - {label: "停止線（A交差点）", cx: 89310, cy: 43035, margin: 20}
curve2_index: 0
curve2_window: {start: 20.0, end: 80.0}
plot_curves:
  - {index: 0, launch_window: {start: 20.0, end: 80.0}}
```

**curve_config の例（加速区間・④用）**:
```yaml
curve_centers:
  - {label: "加速区間", cx: 89350, cy: 42980, margin: 50}
# plot_curves に launch_window を指定しないと curves_closeup 軌跡ズームのみ生成。
# 速度は全軌跡の velocity.fig.json で確認する。
```

**実行**:
```bash
webauto data annotation-dataset pull --project-id x2_dev \
  --annotation-dataset-id <課題の UUID> --include-intermediate-artifacts

make local_cloud_run LOCAL_SCENARIO=$(pwd)/sample/scenario_issue_<課題名>.yaml
```

> **注意**: 複数課題の結果はシナリオごとに独立した `report.html` で確認する。
> 同一条件で複数データセットを横断評価するには、同一 scenario に複数 Datasets を列挙して
> `make local_batch_run LOCAL_SCENARIO=...` を使う。

---

## 設計上の注意

- **N=1 解析は「1 step 予測」**: 各ステップで実機状態にリセットして
  `SUB_DT × cmd_count` 秒だけモデルを進めるため、ケース間の `err_ds_long` /
  `err_ds_lat` 差は小さい (1 step ≒ 17cm 程度の移動内ではモデル差が位置に
  大きく現れない)。ケース差は主に `err_steer` に表れる。
  長期軌跡の累積差は同じ `step5.run_rollout` の N>1 が `nstep_delta.csv` /
  `error_growth.svg` として出力し、Stage 6 が `cases_summary.md` に横断集約する。
- **`overlay.reference_tag` は機能実装済み**: `cases_summary.md` の N=1 表に
  `Δsteer vs ref` 列、horizon 別横断表に `Δyaw vs ref` 列を出力する
  (`step6_analyze_cases.py::write_cases_summary`)。
- **k_us は N=1 では同定不可**: N=1 は k_us 非感度、`err_wz` は k_us=0 seeding
  バイアスを含む。k_us 等の同定には Stage 7 (`step7_sweep_params`, 大 N rollout sweep) を使う。
