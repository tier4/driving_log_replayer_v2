# reidentify (v2)

`real_log_sim_comparison` の `reidentify` パイプライン(実機ログから車両モデルの
物理パラメータを同定する処理)を、`~/software/vehicle_model_fitting` を参考に
依存の少ないシンプルな Python スクリプト集として再構成したもの。

旧パイプライン(`physical_tuning.py` / `multi_dataset_tune.py` /
`physical_validity_report.py` / `release_model_params.py`、Makefile の
`reidentify`/`fit_lon`/`fit_steer`/`fit_merge`/`fit_report` ターゲット)は
そのまま残っており、当面併存する。

## 目的

collection ディレクトリ(複数データセットの `real.lite` を束ねたもの)から:
1. 縦方向モデル(加速度の一次遅れ+むだ時間)を直接同定
2. 操舵モデル(操舵の一次遅れ+むだ時間)+ アンダーステア勾配 (k_us) を直接同定
3. N-step オープンループロールアウト評価を目的関数に、Optuna で warm-start 統合チューニング
4. 簡易な物理妥当性レポート(matplotlib + テキスト)を生成
5. `simulator_model.param.yaml` へのリリース用パラメータ埋め込み

を行う。

## ファイル構成

```
reidentify/
├── README.md            # 本ファイル
├── csv_schema.py          # CSV キャッシュの列スキーマ (extract.py / load_data.py 共有)
├── extract.py              # [Step1] real.lite → dataset毎 CSV キャッシュ (ROS 依存はここだけ)
├── gear.py                  # DRIVE 系 gear 判定 (ROS フリー、lib._io の値を複製)
├── physical_constants.py     # k_us 定常旋回フィルタ閾値 (ROS フリー、lib._physical_validity の値を複製)
├── scenario_params.py         # scenario.yaml から wheelbase を読む薄いヘルパー
├── load_data.py                 # CSV 読込 → 同定用の resampled 配列 / rollout 用の生 DataFrame 群
├── fit_core.py                   # lib/_fit_core.py (同定カーネル SSOT) の re-export
├── fit_lon.py                     # [Step2] 縦方向直接同定 → phase1_acc.yaml
├── fit_steer.py                    # [Step3] 操舵+k_us直接同定 → phase2_steer.yaml
├── rollout.py                       # VehicleModel(ctypes 車両モデル) + N-step ロールアウト評価
├── fit_merge.py                      # [Step4a] Optuna warm-start 統合 → tuned_params.yaml
├── report.py                          # [Step4b] 簡易物理妥当性レポート (機能ダウングレード版)
├── release_params.py                   # [Step4c] release_model_params.py の移設
└── run_reidentify.py                    # 全体オーケストレーションのエントリポイント
```

## 実行方法

```bash
source <workspace>/install/setup.bash  # extract 実行時は必要 (rclpy/rosbag2_py)
python3 -m driving_log_replayer_v2.real_log_sim_comparison.reidentify.run_reidentify \
    --collection-dir <collection_dir> --scenario <scenario.yaml> --case current \
    --n-trials 50 --n-jobs 32
```

成果物は既定で `<collection_dir>/reidentify_v2/` 配下に出力される(旧パイプラインの
`<collection_dir>/phase1_acc.yaml` 等と衝突しない)。

`--only` で一部ステップだけ実行できる(例: CSV キャッシュ済みで extract 不要なら
`--only fit_lon,fit_steer,fit_merge,report,release`。この場合 `extract.py` は
import すらされないため ROS 環境の source は不要)。

```bash
python3 -m driving_log_replayer_v2.real_log_sim_comparison.reidentify.run_reidentify \
    --collection-dir <collection_dir> --scenario <scenario.yaml> \
    --only fit_lon,fit_steer,fit_merge,report,release
```

## 依存ライブラリ

- 常時: `numpy`, `scipy`, `pandas`, `PyYAML`
- `fit_merge`: `optuna`
- `report`: `matplotlib`
- `rollout`(`fit_merge` が内部で使用): `ctypes`(標準)、`ament_index_python`
  (`libvehicle_model_wrapper.so` の解決。closed-loop 実 sim と同じコンパイル済み
  車両モデルを使うことで同定精度の sim 忠実性を保つため、pure-python 再実装には
  していない)
- `extract` のみ: `rclpy`, `rosbag2_py`, `rosidl_runtime_py` (real.lite の読込)。
  他のステップはこれらに一切依存しない。

## 旧パイプラインとの差分 (意図的な機能ダウングレード)

ユーザー合意の上で、以下は今回のシンプル化で持たない。必要な場合は旧
`physical_validity_report.py` / `multi_dataset_tune.py` / `physical_tuning.py`
を使うこと。

- `fit_report` の plotly HTML レポート・地図オーバーレイ・モデル検証 viewer・
  リリースノート生成 → `report.py` は matplotlib 図 + テキストサマリーのみ
- `multi_dataset_tune.py` の `--ds-before`/`--ds-after` 録画日フィルタ、
  `--lite-dir` 手動指定、`--report`/`--report-compare` 比較レポート
- `physical_tuning.py` の `--skip-lon`/`--skip-steer` (scenario.yaml の値を
  そのまま引き継ぐパススルー)

出力 YAML (`phase1_acc.yaml` / `phase2_steer.yaml` / `tuned_params.yaml`) の
スキーマは旧パイプラインと同一に保っている。
