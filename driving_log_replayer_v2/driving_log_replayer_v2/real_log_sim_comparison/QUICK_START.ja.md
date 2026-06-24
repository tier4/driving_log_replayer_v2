# real_log_sim_comparison Quick Start

実機走行ログと closed-loop シミュレーションを比較し、`report.html` を生成するまでの手順。

## 前提

- `pilot-auto.x2` の `feat/v4.4/e2e` ブランチを checkout し、初期セットアップ（`vcs import` 等）が完了済み
- `webauto` CLI で認証済み（`webauto auth` 系コマンドが通る状態）
- CARLA を使う場合:
  - CARLA 0.10.0 バイナリを展開済み
  - CARLA Python wheel をインストール済み: `pip install <carla_path>/PythonAPI/carla/dist/carla-0.10.0-cp310-cp310-linux_x86_64.whl`
- Godot を使う場合:
  - Godot バイナリが `/opt/godot_autoware_simulator/` に配置済み

## 1. ブランチの切り替えとビルド

`simulator.repos` の version とは異なるブランチを使う。
`vcs import` 後に手動で checkout し、ワークスペース全体をビルドする。

| リポジトリ | ブランチ |
|---|---|
| `src/simulator/scenario_simulator` | `temporary/diffusion_planner_godot` |
| `src/simulator/driving_log_replayer_v2` | `sim/e2e_dlr` |

## 2. T4 データセット取得

`real_log_sim_comparison/sample/scenario.yaml` でデフォルトで指定されているデータセット `28443458-8d02-476d-b91c-528ea6027d18`（お台場 E2E 走行・交差点左折）を以下のコマンドで取得する。
別の走行を評価する場合は [3. scenario.yaml の編集 > 別の走行を評価する場合](#別の走行を評価する場合) を参照。

```bash
webauto data annotation-dataset pull \
    --project-id x2_dev \
    --annotation-dataset-id 28443458-8d02-476d-b91c-528ea6027d18 \
    --include-intermediate-artifacts # 必須
```

## 3. scenario.yaml の編集

`real_log_sim_comparison/sample/scenario.yaml` を編集する。

### sim_runs の編集

CARLA を比較対象に含める場合は `sim_runs` に `carla` を追記する:

```yaml
sim_runs: [normal, kus0020, best_normal, taiga_dyn, taiga_x, perfect, godot, carla]
```

CARLA を `/opt/carla` 以外に配置している場合は `models.carla.carla_path` を書き換える。
`carla_path` は CARLA 配布物の展開ルート（配下に `Linux/CarlaUnreal/Binaries/Linux/CarlaUnreal-Linux-Shipping` を持つディレクトリ）を指定する。

```yaml
carla:
  carla_path: /path/to/carla
```

Godot が未導入の場合は `sim_runs` から `godot` を外す。

### 別の走行を評価する場合

`Evaluation.Datasets[0]` の UUID を対象走行のものに書き換え、同じ UUID で pull する。

```yaml
Datasets:
  - <対象走行の UUID>:
      VehicleId: default
```

```bash
webauto data annotation-dataset pull \
    --project-id x2_dev \
    --annotation-dataset-id <対象走行の UUID> \
    --include-intermediate-artifacts
```

## 4. 実行

`real_log_sim_comparison/` ディレクトリで以下のコマンドを実行する。

```bash
make local_cloud_run
```

## 5. 結果確認

`real_log_sim_comparison/sample/out/latest/` に `report.html` が出力される。
`report.html` は速度・操舵・軌跡の比較プロットが含まれる単一の HTML ファイルで、ブラウザで表示できる。

real_log_sim_comparisonの詳細な処理内容については [`README.ja.md`](README.ja.md) を参照。
