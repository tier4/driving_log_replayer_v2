# real_log_sim_comparison サンプル (cloud / local 共通)

`scenario.yaml` / `cases.yaml` / `sim_runs.yaml` / `curve_config_miraikan.yaml` は
cloud（Web.Auto evaluator）と local（`make local_cloud_run`）で**共通の単一サンプル**。

- **cloud**: これらは CMakeLists で `share/driving_log_replayer_v2/sample/real_log_sim_comparison/`
  に install され、登録済み Web.Auto scenario の `Conditions` がその install-share 絶対パスを参照する。
- **local**: `make local_cloud_run` が本ディレクトリの `scenario.yaml`（`Conditions` は `./cases.yaml`
  `./sim_runs.yaml` 相対参照）をそのまま使い、同じパイプラインをワンコマンド実行する。

Godot バイナリパスは cloud / local 共通で `/opt/godot_autoware_simulator/` に統一。cloud は
`.webauto-ci` が自動配置、local は同じ `/opt` に配置（または symlink）する。
`curve_config_miraikan.yaml` は `scenario.yaml` の `Conditions.curve_config_yaml` で参照する
（現状サンプルは空文字で無効）。

---

## パイプライン概要

10 段階パイプラインの各 stage の入出力は親ディレクトリの
[`../README.ja.md`](../README.ja.md#パイプライン10-段階) を参照。ローカル実行で押さえる点:

- Stage 3（sim 実行）と Stage 5（per-step 解析）はそれぞれ `sim_runs.yaml` / `cases.yaml`
  の各エントリで N 回ループする。`scenario.yaml` の `Conditions.sim_runs_config` /
  `Conditions.cases_config` で参照され、**両方未指定だとパイプラインは失敗する**。

> **実行時間**: Stage 3 は scenario_test_runner で Autoware を起動して closed-loop
> シムを回すため 1 run あたり ~5 分。既定 `sim_runs.yaml` は 4 run（sim_normal /
> sim_kus0020 / sim_perfect / sim_godot）のため、end-to-end は ~25 分の見積もり。

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

### 2. sim_runs.yaml と cases.yaml を確認/編集

**sim_runs.yaml (Stage 3/4 用)**: closed-loop シム実行のラン定義。
```yaml
sim_runs:
  - tag: sim_normal
    vehicle_model: j6_gen2
  - tag: sim_kus0020
    vehicle_model: j6_gen2
    params: {k_us: 0.020}   # simulator_model パラメータ上書き (description を増やさず変種を作る)
  - tag: sim_godot
    vehicle_model: j6_gen2_godot
    godot_executable: /opt/godot_autoware_simulator/godot_autoware_simulator.x86_64
```
`vehicle_model` は `j6_gen2` (通常シム) / `j6_gen2_perfect_tracker` (理想軌跡追従) / `j6_gen2_godot` (Godot シム)。
Godot 実行時は `godot_executable` パスが必要。`params` は `simple_sensor_simulator.<key>:=<value>` として
launch に渡り、description の simulator_model.param.yaml を上書きする。

**cases.yaml (Stage 5/6 用)**: VehicleModel per-step 解析のケース定義。
```yaml
cases:
  - tag: baseline
    vehicle_model: delay_steer_acc_geared_wo_fall_guard
    params: {wheelbase: 4.76012}
  - tag: kus0020
    vehicle_model: delay_steer_acc_geared_wo_fall_guard
    params: {wheelbase: 4.76012, k_us: 0.020}
  - tag: shorter_wb
    vehicle_model: delay_steer_acc_geared_wo_fall_guard
    params: {wheelbase: 4.50}
  - tag: ideal_steer
    vehicle_model: ideal_steer_acc
    params: {wheelbase: 4.76012, steer_bias: 0.0}
overlay:
  reference_tag: baseline
  plots: [cascade_error, error_timeseries]
```
`vehicle_model` は `delay_steer_acc_geared_wo_fall_guard` or `ideal_steer_acc` の 2 種類。
`params` で未指定のキーは `load_sim_params()` (j6_gen2_description の YAML) で補完。

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

主要な動作：

1. `scenario.yaml` から Datasets UUID を取得 → `~/.webauto/.../` から実体パス解決
2. `out/<タイムスタンプ>/` を作成し、`out/latest` シンボリックリンクを更新
3. `ros2 launch` で 10 段階パイプライン起動
   - Stage 1: 実機 bag → lite/real.lite
   - Stage 2: step2_bag_to_scenario → scenarios/auto_scenario.yaml
   - Stage 3: sim_runs.yaml の各 run で scenario_test_runner → lite/<tag>.lite
   - Stage 4: step4_compare_logs → comparison/figures/, report.md (real + sim 重ね描き)
   - Stage 5: cases.yaml の各 tag で step5_analyze_per_step → per_step/<tag>/
   - Stage 6: step6_analyze_cases → cases/overlay/, cases_summary.md
   - Stage 7: step7_identify_kus → comparison/kus_sweep/ (k_us 同定; 追加設定不要)
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
    │   └── <run_tag>.lite/*.mcap         # Stage 3: sim_runs.yaml の各 run
    │                                     #   (既定 sim_normal / sim_kus0020 / sim_perfect / sim_godot)
    ├── scenarios/auto_scenario.yaml      # Stage 2
    └── comparison/
        ├── report.md                     # Stage 4: 比較統計レポート
        ├── figures/*.png                 # Stage 4: 速度・操舵・軌跡, Stage 8: dp_*.png
        ├── per_step/
        │   └── <case_tag>/{*.png(9枚), per_step_delta.csv, rollout.csv, summary.txt}  # Stage 5
        ├── cases/
        │   ├── overlay/{cascade_error_overlay.png, error_timeseries_overlay.png}
        │   └── cases_summary.md          # Stage 6: per-step RMSE + rollout 横断表
        ├── kus_sweep/{kus_sweep.csv, kus_sweep.png}      # Stage 7: k_us 同定
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
| `cases.yaml が見つかりません` | `scenario.yaml` の `Conditions.cases_config` の相対パスと cases.yaml の存在を確認 |
| `sim_runs.yaml が見つかりません` | `scenario.yaml` の `Conditions.sim_runs_config` の相対パスと sim_runs.yaml の存在を確認 |
| `未対応の model_type` | cases.yaml の `vehicle_model` は `delay_steer_acc_geared_wo_fall_guard` か `ideal_steer_acc` のみ。新規 model 追加は `vehicle_model_c_wrapper.cpp` への factory 追加が必要 |
| Stage 3 (sim) が全 run 失敗 | scenario_test_runner が起動できているか、auto_scenario.yaml が valid か `ros2 launch scenario_test_runner scenario_test_runner.launch.py scenario:=<...auto_scenario.yaml>` を手動で試行 |
| sim 1 run で 10 分以上かかる | `sim_runs.yaml` の `timeout_s` を上げるか、`initialize_duration` を下げて確認 |

## 設計上の注意

- **per_step 解析は「1 step 予測」**: 各ステップで実機状態にリセットして
  `SUB_DT × cmd_count` 秒だけモデルを進めるため、ケース間の `err_ds_long` /
  `err_ds_lat` 差は小さい (1 step ≒ 17cm 程度の移動内ではモデル差が位置に
  大きく現れない)。ケース差は主に `err_steer` に表れる。
  長期軌跡の累積差は `step5.run_free_rollout` (multi-step rollout) が `rollout.csv` /
  `rollout_error_growth.png` として出力し、Stage 6 が `cases_summary.md` に横断集約する。
- **`overlay.reference_tag` は機能実装済み**: `cases_summary.md` の per-step 表に
  `Δsteer vs ref` 列、rollout 横断表に `Δyaw vs ref` 列を出力する
  (`step6_analyze_cases.py::write_cases_summary`)。
- **k_us は per-step では同定不可**: per-step は k_us 非感度、`err_wz` は k_us=0 seeding
  バイアスを含む。k_us 同定には Stage 7 (`step7_identify_kus`, rollout sweep) を使う。
