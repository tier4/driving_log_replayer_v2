# マルチ DiffusionPlanner モデル評価（sim_runs.yaml 駆動）

本ドキュメントは、**同一車両モデルのまま DiffusionPlanner (DP) モデルだけを差し替えて** closed-loop
シミュレーションを回し、実車試験で使う「良いモデル」を選定するための機構を説明する。`sim_runs.yaml` に
モデルを書くだけで（`.webauto-ci.yml` を編集せず）モデルの自動取得・切り替え・比較ができる。

---

## 0. 結論（要約）

- `sim_runs.yaml` の各 run に **`dp_model_release`**（または `dp_model_dir`）を書くと、同一 `vehicle_model` で
  DP モデルだけが異なる N-way 比較を 1 つの評価ジョブで実行できる。
- DP モデルの差し替えは **param yaml の `$(env)` 置換 + 環境変数**で行う（正規パスへのファイルコピーや
  simulation 定義の複製は不要）。
- `dp_model_release` 指定時は **`webauto ml package-release` で自動 pull**（`.webauto-ci.yml` 不要）。
- 既存の比較・可視化（軌跡 overlay / DP 軌跡比較 / `trajectory_playback.html` / provenance 表示）は
  **tag 単位**で N-way 対応済のため、追加実装なしでモデル間差分を可視化できる。

---

## 1. モデル切り替えの仕組み

### 1.1 param yaml の `$(env)` 置換（変更 A）

autoware_launch の `config/planning/neural_net_planner/diffusion_planner.param.yaml`:

```yaml
onnx_model_path: $(env DIFFUSION_PLANNER_ONNX_PATH /opt/autoware/mlmodels/diffusion_planner_for_x2_exp/diffusion_planner.onnx)
args_path:       $(env DIFFUSION_PLANNER_ARGS_PATH /opt/autoware/mlmodels/diffusion_planner_for_x2_exp/args.json)
```

`autoware_diffusion_planner/launch/diffusion_planner.launch.xml` が
`<param from="$(var diffusion_planner_param_path)" allow_substs="true"/>` で読み込むため、param yaml 内の
`$(env ...)` 置換が解決される。**環境変数 `DIFFUSION_PLANNER_ONNX_PATH` を設定すればそのモデルがロードされ、
未設定なら従来の正規パス**にフォールバックする（ROS ネイティブ、cpp 変更不要、ファイル都度書き換え不要）。

> **変更 A の適用は DLR が実行時に行う**: step3 が起動前に *インストール済* autoware_launch の
> `diffusion_planner.param.yaml` へ `$(env ...)` を自動注入する（`_ensure_installed_param_honors_env`）。
> これにより **launcher サブリポジトリ / autoware.repos を一切変更せず**に env 切り替えを有効化できる。
> 注入は冪等（既に `$(env` があれば no-op）で、env 未設定時は元の既定値にフォールバックするため挙動不変・
> restore 不要。ローカルは symlink-install で source に A 済なら no-op、クラウドはインストール済コピーへ注入。

> 注: `DP_ONNX_PATH` 環境変数は DP ノードでは読まれない（`lib/_provenance.py` が記録用に参照するのみ）。

### 1.2 step3 が run ごとに env を設定

`step3_run_sims` は **1 run-tag = 1 プロセス**で起動されるため、`_setup_dp_model_env()` が `os.environ` を
直接設定する。これにより (1) `scenario_test_runner` launch の subprocess が継承し、(2) 同プロセスの
provenance capture も同じ onnx を解決する。

```
DIFFUSION_PLANNER_ONNX_PATH = <dp_model_dir>/diffusion_planner.onnx
DIFFUSION_PLANNER_ARGS_PATH = <dp_model_dir>/args.json
DP_ONNX_PATH                = <dp_model_dir>/diffusion_planner.onnx   # provenance 解決元と一致させる
```

---

## 2. sim_runs.yaml の書き方

### 2.1 自動 pull（推奨・`.webauto-ci.yml` 不要）

```yaml
sim_runs:
  - tag: sim_dp_e40
    vehicle_model: j6_gen2
    dp_model_release: 20260410-145919_lambda1e-6_ridge1e-8__epoch0020__epoch0040
  - tag: sim_dp_e30
    vehicle_model: j6_gen2
    dp_model_release: 20260410-145919_lambda1e-6_ridge1e-8__epoch0020__epoch0030
    # dp_model_package: diffusion_planner_for_x2_exp   # 既定。別 package のときのみ指定
```

`step3_run_sims._resolve_dp_model_dir()` の流れ:

1. `webauto ml package-release search --package-name <pkg> --package-release-name <release>` で
   **release 名 → `package_id` / `package_release_id`** を解決。
2. `webauto ml package-release pull --target-dir <root>` で取得。
   配置は `<root>/<release-id>/<package>/{diffusion_planner.onnx, args.json}`。
3. onnx を含む dir を `dp_model_dir` として後続（env 設定・エンジン事前ビルド）に渡す。

| 項目 | 既定 | 上書き |
|---|---|---|
| Web.Auto プロジェクト | `x2_dev` | env `WEBAUTO_PROJECT_ID` |
| pull 先 root | `~/.webauto/data/ml/package-release` | env `DP_MODEL_PULL_DIR` |

- **冪等**: `release-id` でキャッシュし、既取得なら再 pull をスキップ（TensorRT エンジンキャッシュも再利用）。
- **認証必須**: `webauto` の認証が要る。ローカルは認証済なら即動作。**cloud は sim runtime コンテナで
  webauto 認証が利用可能かを要確認**（無ければ pull が loud に失敗する）。

### 2.2 ローカル既存ディレクトリを直接指定

```yaml
  - tag: sim_dp_local
    vehicle_model: j6_gen2
    dp_model_dir: /opt/autoware/mlmodels/diffusion_planner_for_x2_exp
```

`dp_model_dir` 直下に **`diffusion_planner.onnx` と `args.json` の 2 ファイル名**が必須。
`dp_model_release` とは排他（同時指定はロード時にエラー）。

---

## 3. TensorRT エンジンの扱い（混線しない理由）

`autoware_diffusion_planner/src/inference/tensorrt_inference.cpp`（`load_engine`）はエンジンファイル名を
**onnx パスから導出**する:

```cpp
engine_file_path = <onnx の親ディレクトリ> / (<onnx の stem> + "_batch" + N + "_fp32.engine")
```

→ エンジンは **onnx と同じディレクトリにモデル別名でキャッシュ**される。`dp_model_release` の自動 pull は
モデルごとに別ディレクトリ（`<release-id>/<package>/`）に配置されるため、**エンジンも完全に隔離**され、
「モデル B が黙ってモデル A のエンジンで走る」確信的誤りは起きない。

`step3` は env 切り替えによる遅延ビルドで scenario が timeout するのを避けるため、scenario 起動前に
`_prebuild_dp_engine()` で `diffusion_planner.launch.xml build_only:=true` を一度走らせ、エンジンを
model dir に事前生成する（既にあれば高速ロードして終了、冪等）。

---

## 4. silent-wrong ガード（重要）

`provenance.json` は step3 が立てた env（＝**意図**）を記録するだけで、**Autoware が実際にロードした
モデルではない**。変更 A が *インストール済* autoware_launch に未反映だと、Autoware は既定モデルを全 run で
ロードするのに provenance は dp_model_dir 別 sha8 を記録し、「別モデルなのに同一挙動」という誤りが
全チェック緑のまま通り得る。これを防ぐため `_setup_dp_model_env()` が起動前に検証する:

1. onnx / args ファイルの存在（欠落時は既定へフォールバックせず `RuntimeError`）。
2. **インストール済 param yaml への `$(env ...)` 注入**（`_ensure_installed_param_honors_env`）。
   未適用なら DLR が注入して有効化する。install が read-only で**注入できない場合のみ** `RuntimeError`
   を送出し、代替（launcher へ A をコミット + autoware.repos 更新）を案内する。

即時確認コマンド:

```bash
grep onnx_model_path "$(ros2 pkg prefix autoware_launch --share)/config/planning/neural_net_planner/diffusion_planner.param.yaml"
# → $(env DIFFUSION_PLANNER_ONNX_PATH ...) が出れば A 反映済
```

---

## 5. provenance と可視化

- `step3` が各 lite に `provenance.json` を書き、`dp_model_dir` / `dp_model_release` / `dp_onnx_sha8` /
  `dp_exp_name` を記録（`lib/_provenance.py`）。`format_provenance_line` でレポートにモデル識別を掲載。
- 軌跡 overlay（Stage 4）・DP 軌跡比較（Stage 8）・`trajectory_playback.html` は **tag 単位** N-way
  比較のため、`dp_model_*` を変えた run も追加実装なしで重ね描き・再生できる。

---

## 6. クラウドで効かせるための前提（ユーザー対応）

**変更 A のクラウド反映は不要**（step3 がインストール済 param へ実行時注入するため。§1.1）。launcher
サブリポジトリ / autoware.repos の変更は行わなくてよい。ただし以下は前提:

1. **install が書き込み可能なこと**: step3 がインストール済 `diffusion_planner.param.yaml` に `$(env ...)` を
   注入できること（sim runtime ユーザーが書き込めること）。read-only なら `RuntimeError` で停止し、代替として
   launcher へ A をコミット + autoware.repos 更新を案内する。
2. **webauto 認証**: cloud の sim runtime コンテナで `webauto` 認証が使えること（自動 pull 用）。
   使えない環境では、従来どおり `.webauto-ci.yml` の asset-deploy でモデルを配置し `dp_model_dir` で指す。

---

## 7. 検証

| レイヤ | 内容 |
|---|---|
| ローカル機構 | `grep` でインストール済 param が `$(env ...)` を含むことを確認（symlink-install なら即反映）。`webauto ml package-release search/pull` の自動取得を実 API で確認済。 |
| provenance 分離 | `sim_dp_*` 各 run の provenance.json で `dp_onnx_sha8`/`dp_exp_name` が異なること（**必要条件だが十分でない**: provenance は意図記録のため、実ロードの証明は §4 ガード or DP ノードログに依存）。 |
| 挙動差比較 | **クラウド closed-loop のみ有効**。ローカル `with_autoware:=false` の closed-loop は ego が動かず退化するため、モデル間の軌跡・速度差の比較はクラウドで行う。 |
