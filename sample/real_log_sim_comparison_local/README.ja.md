# real_log_sim_comparison ローカル実行サンプル

クラウド（Web.Auto evaluator）と同じ `real_log_sim_comparison` パイプラインを、
ローカルに `webauto` で取得した T4 データセットを使ってワンコマンド実行するための
サンプル。

クラウド側の正本は `sample/real_log_sim_comparison/scenario.yaml` 。本ディレクトリ
の `scenario.yaml` はそのコピーで、`Datasets:` の UUID と前置きコメントだけが
ローカル実行向けに調整されている。`curve_config_miraikan.yaml` はクラウド版への
相対シンボリックリンクで、二重メンテを避けている。

---

## パイプライン 4 段階

| Stage | 名称 | 入力 | 出力 (`result_archive/` 配下) | model 依存 | 実行回数 |
|---|---|---|---|---|---|
| 1 | 実機ログ抽出 | `input_bag/*.{mcap,db3}` | `lite/real.lite/` | なし | 1 |
| 2 | 実機ログ解析 | `lite/real.lite/` | `comparison/{figures/,report.md}` | なし | 1 |
| 3 | VehicleModel 解析 (ケース別) | `lite/real.lite/` + cases.yaml の 1 ケース | `comparison/per_step/<tag>/` | あり | N (cases) |
| 4 | ケース集約解析 | `comparison/per_step/<tag>/per_step_delta.csv` 群 | `comparison/cases/{overlay/, cases_summary.md}` | 集約 | 1 |

Stage 3/4 のケース定義は `cases.yaml` (本ディレクトリ) で行う。`scenario.yaml` の
`Conditions.cases_config` で参照されており、**未指定だとパイプラインは失敗する**。

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
> `scenario.yaml` のデフォルト UUID `67db293e-903e-4397-b603-be6b88d98be7` は、
> クラウド版 scenario.yaml がそのまま継承した「Web.Auto trigger 用に選ばれた
> 既定値」であり、実際に比較したい走行（例: テレポート駅→日本科学未来館）と
> 一致する保証はない。比較レポート (`comparison/report.md`) の数値や図が想定と
> 合わない場合、まずこの UUID が正しい走行を指しているかを疑うこと。
>
> 自分の対象走行に切り替える場合は `scenario.yaml` の Datasets 先頭キーを
> 書き換えてから上記コマンドを実行する。

`--include-intermediate-artifacts` を付けないと `input_bag/` に bag 本体
(`*.mcap` / `*.db3`) が含まれず、`make_lite` の入力が足りずに失敗する。

ダウンロード結果は `~/.webauto/data/data/annotation_dataset/<UUID>/<frame>/` 配下に
`annotation/`, `data/`, `input_bag/`, `map/` が展開される。
ローカル実行スクリプトはここを自動探索する。

### 2. cases.yaml を確認/編集 (Stage 3/4 のケース定義)

```yaml
# cases.yaml
cases:
  - tag: baseline
    vehicle_model: delay_steer_acc_geared_wo_fall_guard
    params: {wheelbase: 4.76012, steer_bias: 0.01, steer_time_constant: 0.4983}
  - tag: shorter_wb
    vehicle_model: delay_steer_acc_geared_wo_fall_guard
    params: {wheelbase: 4.50}
  - tag: ideal_steer
    vehicle_model: ideal_steer_acc
    params: {wheelbase: 4.76012}
overlay:
  reference_tag: baseline
  plots: [cascade_error, error_timeseries]
```

`vehicle_model` は `delay_steer_acc_geared_wo_fall_guard` or `ideal_steer_acc` の 2 種類。
`params` で 未指定のキーは `load_sim_params()` (best_model_description の YAML) で補完される。

### 3. 実行

```bash
cd src/simulator/driving_log_replayer_v2/driving_log_replayer_v2
make local_cloud_run
```

主要な動作：

1. `scenario.yaml` から Datasets UUID を取得 → `~/.webauto/.../` から実体パス解決
2. `out/<タイムスタンプ>/` を作成し、`out/latest` シンボリックリンクを更新
3. `ros2 launch` で 4 段階パイプライン起動
   - Stage 1: 実機 bag → lite/real.lite
   - Stage 2: compare_logs → comparison/figures/, report.md
   - Stage 3: cases.yaml の各 tag で analyze_per_step → per_step/<tag>/
   - Stage 4: analyze_cases → cases/overlay/, cases_summary.md

### 4. 結果確認

```text
sample/real_log_sim_comparison_local/out/latest/
├── result.jsonl                       # 末尾行に {"Result":{"Success":true,...}}
└── result_archive/
    ├── lite/real.lite/*.mcap          # Stage 1: 抽出 lite bag
    └── comparison/
        ├── report.md                  # Stage 2: 比較統計レポート
        ├── figures/*.png              # Stage 2: 速度・操舵・軌跡・カーブ別図
        ├── per_step/
        │   ├── <tag1>/{*.png, per_step_delta.csv, summary.txt}   # Stage 3 (case)
        │   ├── <tag2>/{...}
        │   └── ...
        └── cases/
            ├── overlay/{cascade_error_overlay.png, error_timeseries_overlay.png}
            └── cases_summary.md       # Stage 4: tag × RMSE 表
```

## 上書き可能な Makefile 変数

| 変数 | 既定値 | 用途 |
|---|---|---|
| `WEBAUTO_T4_ROOT` | `$HOME/.webauto/data/data/annotation_dataset` | webauto annotation-dataset pull の出力ルート |
| `LOCAL_SCENARIO` | `sample/real_log_sim_comparison_local/scenario.yaml` | 別の scenario.yaml を使う場合 |
| `WS_ROOT` | `Makefile` の 4 階層上 | colcon ワークスペースルート |

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
| `未対応の model_type` | cases.yaml の `vehicle_model` は `delay_steer_acc_geared_wo_fall_guard` か `ideal_steer_acc` のみ。新規 model 追加は `vehicle_model_c_wrapper.cpp` への factory 追加が必要 |

## 設計上の注意

- **per_step 解析は「1 step 予測」**: 各ステップで実機状態にリセットして
  `SUB_DT × cmd_count` 秒だけモデルを進めるため、ケース間の `err_ds_long` /
  `err_ds_lat` 差は小さい (1 step ≒ 17cm 程度の移動内ではモデル差が位置に
  大きく現れない)。ケース差は主に `err_steer` に表れる。
  長期軌跡の累積差を見たい場合は別解析 (将来 Stage 5 として追加候補) が必要。
- **`overlay.reference_tag` は現状 decorative**: `cases_summary.md` のヘッダに
  表示されるだけで、reference との delta/relative 比較列は未実装。必要になれば
  `analyze_cases.py` の `write_cases_summary` に追加できる。
