# real_log_sim_comparison ローカル実行サンプル

クラウド（Web.Auto evaluator）と同じ `real_log_sim_comparison` パイプラインを、
ローカルに `webauto` で取得した T4 データセットを使ってワンコマンド実行するための
サンプル。

クラウド側の正本は `sample/real_log_sim_comparison/scenario.yaml` 。本ディレクトリ
の `scenario.yaml` はそのコピーで、`Datasets:` の UUID と前置きコメントだけが
ローカル実行向けに調整されている。`curve_config_miraikan.yaml` はクラウド版への
相対シンボリックリンクで、二重メンテを避けている。

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

### 2. 実行

```bash
cd src/simulator/driving_log_replayer_v2/driving_log_replayer_v2
make local_cloud_run
```

主要な動作：

1. `scenario.yaml` から Datasets 先頭の UUID を取得
2. `~/.webauto/data/data/annotation_dataset/` 配下から同名ディレクトリを探索
3. その下の `<frame>/input_bag/` に bag 本体 (`*.mcap`/`*.db3`) があるか検証
4. `out/<タイムスタンプ>/` を作成し、`out/latest` シンボリックリンクを更新
5. `ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py` を
   `t4_dataset_path:=<frame_dir> t4_dataset_id:=<UUID> with_autoware:=false`
   付きで起動

### 3. 結果確認

```text
sample/real_log_sim_comparison_local/out/latest/
├── result.jsonl                       # 末尾行に {"Result":{"Success":true,...}}
└── result_archive/
    ├── lite/real.lite/*.mcap          # 抽出 lite bag
    └── comparison/
        ├── report.md                  # 比較統計レポート
        └── figures/*.png              # 速度・操舵・軌跡・カーブ別図
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
