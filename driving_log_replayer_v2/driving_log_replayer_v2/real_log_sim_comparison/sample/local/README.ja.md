# real_log_sim_comparison ローカル実行サンプル

クラウド（Web.Auto evaluator）と同じ `real_log_sim_comparison` パイプラインを、
ローカルに `webauto` で取得した T4 データセットを使ってワンコマンド実行するための
サンプル。

クラウド側の正本は同階層 `../cloud/scenario.yaml`。本ディレクトリの `scenario.yaml`
はそのコピーで、`Datasets:` の UUID と前置きコメントだけがローカル実行向けに調整
されている。`curve_config_miraikan.yaml` は cloud / local 共通の参照として 1 つ上の
`../curve_config_miraikan.yaml` に置いてあり、必要なときに `scenario.yaml` の
`Conditions.curve_config_yaml` で参照する (現状サンプルは空文字で無効)。

---

## パイプライン 6 段階

| Stage | 名称 | 入力 | 出力 (`result_archive/` 配下) | model 依存 | 実行回数 |
|---|---|---|---|---|---|
| 1 | 実機ログ抽出 | `input_bag/*.{mcap,db3}` | `lite/real.lite/` | なし | 1 |
| 2 | scenario 自動生成 | `input_bag/` + map | `scenarios/auto_scenario.yaml` (OpenSCENARIO) | なし | 1 |
| 3 | closed-loop シム実行 | `auto_scenario.yaml` + `sim_runs.yaml` の 1 run | `lite/<run_tag>.lite/` | あり | N (sim_runs) |
| 4 | 実機 + sim 比較解析 | `lite/{real, <run_tag>}.lite/` 群 | `comparison/{figures/, report.md}` (N-way 重ね描き) | 集約 | 1 |
| 5 | VehicleModel per-step 解析 | `lite/real.lite/` + cases.yaml の 1 ケース | `comparison/per_step/<tag>/` | あり | N (cases) |
| 6 | ケース集約解析 | `per_step/<tag>/per_step_delta.csv` 群 | `comparison/cases/{overlay/, cases_summary.md}` | 集約 | 1 |

Stage 3 (sim 実行) と Stage 5 (per-step 解析) のケース定義はそれぞれ
`sim_runs.yaml` / `cases.yaml` で行う。`scenario.yaml` の
`Conditions.sim_runs_config` / `Conditions.cases_config` で参照されており、
**両方未指定だとパイプラインは失敗する**。

> **実行時間**: Stage 3 は scenario_test_runner で Autoware を起動して closed-loop
> シムを回すため 1 run あたり ~5 分。sim_runs.yaml に 2 run (sim_normal +
> sim_godot) を書いた場合、end-to-end は ~15 分の見積もり。

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
(`*.mcap` / `*.db3`) が含まれず、`make_lite` の入力が足りずに失敗する。

ダウンロード結果は `~/.webauto/data/data/annotation_dataset/<UUID>/<frame>/` 配下に
`annotation/`, `data/`, `input_bag/`, `map/` が展開される。
ローカル実行スクリプトはここを自動探索する。

### 2. sim_runs.yaml と cases.yaml を確認/編集

**sim_runs.yaml (Stage 3/4 用)**: closed-loop シム実行のラン定義。
```yaml
sim_runs:
  - tag: sim_normal
    vehicle_model: best_model
  - tag: sim_godot
    vehicle_model: j6_gen2_godot
    godot_executable: ${HOME}/Downloads/godot_autoware_simulator.x86_64
```
`vehicle_model` は `best_model` (通常シム) または `j6_gen2_godot` (Godot シム)。
Godot 実行時は `godot_executable` パスが必要。

**cases.yaml (Stage 5/6 用)**: VehicleModel per-step 解析のケース定義。
```yaml
cases:
  - tag: baseline
    vehicle_model: delay_steer_acc_geared_wo_fall_guard
    params: {wheelbase: 4.76012, steer_bias: 0.01, steer_time_constant: 0.4983}
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
`params` で未指定のキーは `load_sim_params()` (best_model_description の YAML) で補完。

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
3. `ros2 launch` で 6 段階パイプライン起動
   - Stage 1: 実機 bag → lite/real.lite
   - Stage 2: bag_to_scenario → scenarios/auto_scenario.yaml
   - Stage 3: sim_runs.yaml の各 run で scenario_test_runner → lite/<tag>.lite
   - Stage 4: compare_logs → comparison/figures/, report.md (real + sim 重ね描き)
   - Stage 5: cases.yaml の各 tag で analyze_per_step → per_step/<tag>/
   - Stage 6: analyze_cases → cases/overlay/, cases_summary.md

### 4. 結果確認

出力は use case 集約ディレクトリ配下の `sample/local/out/` に書かれる
(`driving_log_replayer_v2/driving_log_replayer_v2/real_log_sim_comparison/sample/local/out/`)。

```text
sample/local/out/latest/
├── result.jsonl                       # 末尾行に {"Result":{"Success":true,...}}
└── result_archive/
    ├── lite/
    │   ├── real.lite/*.mcap              # Stage 1
    │   ├── sim_normal.lite/*.mcap        # Stage 3 run 1
    │   └── sim_godot.lite/*.mcap         # Stage 3 run 2 (Godot バイナリある場合)
    ├── scenarios/auto_scenario.yaml      # Stage 2
    └── comparison/
        ├── report.md                     # Stage 4: 比較統計レポート
        ├── figures/*.png                 # Stage 4: 速度・操舵・軌跡 (real + sim 重ね描き)
        ├── per_step/
        │   ├── baseline/{*.png, per_step_delta.csv, summary.txt}   # Stage 5
        │   ├── shorter_wb/{...}
        │   └── ideal_steer/{...}
        └── cases/
            ├── overlay/{cascade_error_overlay.png, error_timeseries_overlay.png}
            └── cases_summary.md          # Stage 6: tag × RMSE 表
```

## 上書き可能な Makefile 変数

| 変数 | 既定値 | 用途 |
|---|---|---|
| `WEBAUTO_T4_ROOT` | `$HOME/.webauto/data/data/annotation_dataset` | webauto annotation-dataset pull の出力ルート |
| `LOCAL_SCENARIO` | `<Makefile dir>/sample/local/scenario.yaml` | 別の scenario.yaml を使う場合 |
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
  長期軌跡の累積差を見たい場合は別解析 (将来 Stage 5 として追加候補) が必要。
- **`overlay.reference_tag` は現状 decorative**: `cases_summary.md` のヘッダに
  表示されるだけで、reference との delta/relative 比較列は未実装。必要になれば
  `analyze_cases.py` の `write_cases_summary` に追加できる。
