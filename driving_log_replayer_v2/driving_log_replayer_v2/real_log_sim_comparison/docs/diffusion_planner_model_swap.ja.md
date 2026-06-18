# Diffusion Planner モデルの入れ替えと実行

本ドキュメントは、`real_log_sim_comparison` で **同一車両モデルのまま Diffusion Planner (DP) モデルだけを
差し替えて** closed-loop シミュレーションを回し、実車で使う「良いモデル」を選定するための機構を、
**実装の仕組み**を中心に（How-to も含めて）説明する。`scenario.yaml` の `Conditions.models` に
モデルを書くだけで（`.webauto-ci.yml` を編集せず）モデルの自動取得・切り替え・N-way 比較ができる。

---

## 0. 結論（要約）

- `Conditions.models.<name>` に **`dp_model_release`**（または `dp_model_dir`）を書き、`sim_runs:` に
  その名前を並べると、同一 `vehicle_model` で DP モデルだけが異なる N-way 比較を 1 評価で実行できる。
- DP モデルの差し替えは **param yaml の `$(env)` 置換 + 環境変数**で行う（正規パスへのファイルコピーや
  simulation 定義の複製は不要）。
- `dp_model_release` 指定時は **`webauto ml package-release` で自動 pull**（`.webauto-ci.yml` 不要）。
- 既存の比較・可視化（軌跡 overlay / DP 軌跡比較 / `report.html` / provenance 表示）は **tag（モデル名）
  単位**で N-way 対応済のため、追加実装なしでモデル間差分を可視化できる。

---

## 1. 全体像（データフロー）

DP モデル指定から実ロードまでの流れ。各ステップの実装シンボルは §3 で詳述する。

```
scenario.yaml の Conditions.models.<name>.{dp_model_release | dp_model_dir}
   │  (Conditions.sim_runs: [<name>, ...] が参照)
   ▼
step3_run_sims._resolve_dp_model_dir(run)
   │  dp_model_release → webauto ml package-release search/pull → onnx を含む dir
   │  dp_model_dir     → そのまま採用
   ▼  dp_model_dir 決定
step3_run_sims._setup_dp_model_env(dp_model_dir)
   │  os.environ に DIFFUSION_PLANNER_ONNX_PATH / DIFFUSION_PLANNER_ARGS_PATH / DP_ONNX_PATH を設定
   │  併せて _ensure_installed_param_honors_env() で param yaml に $(env ...) を実行時注入
   ▼
step3_run_sims._prebuild_dp_engine(dp_model_dir)   # TensorRT エンジンを起動前に事前ビルド
   ▼
ros2 launch scenario_test_runner ...   (subprocess。os.environ を継承)
   │
   ▼
diffusion_planner.param.yaml の onnx_model_path: $(env DIFFUSION_PLANNER_ONNX_PATH ...)
   │  diffusion_planner.launch.xml が allow_substs="true" で読み込み → env で置換
   ▼
DiffusionPlanner ノードが <dp_model_dir>/diffusion_planner.onnx をロード
```

ポイントは、**正規の YAML（`diffusion_planner.param.yaml`）を書き換えずに、環境変数だけで onnx パスを
差し替える**こと。`step3_run_sims` は **1 run-tag = 1 プロセス**で起動されるため、`os.environ` を直接
設定すれば (1) launch subprocess が継承し、(2) 同プロセスの provenance capture も同じ onnx を解決する。

---

## 2. モデル指定の書き方（How-to）

### 2.1 現行方式 ＝ `Conditions.models` レジストリ（重要）

設定はすべて `scenario.yaml` の `Evaluation.Conditions` ブロックにインライン記述する。
**外部ファイル `cases.yaml` / `sim_runs.yaml` は廃止されており存在しない**
（根拠: `lib/_models_config.py` の冒頭 docstring「cases.yaml と sim_runs*.yaml は廃止され、設定はすべて
scenario.yaml の Conditions ブロックに統合された」）。

構造は **名前キーのレジストリ + 名前参照リスト**:

```yaml
Evaluation:
  Conditions:
    models:                       # 名前付きレジストリ。名前 = 出力 tag (lite/<name>.lite/)
      <name>:
        vehicle_model: j6_gen2    # closed-loop description パッケージ名 (sim_runs で必須)
        dp_model_release: ...     # ↓ §2.2 / §2.3
    cases:    [<name>, ...]       # open-loop VehicleModel 解析に使う名前リスト
    sim_runs: [<name>, ...]       # closed-loop sim 実行に使う名前リスト (DP モデル比較はこちら)
```

`sim_runs` は **モデル定義を埋め込むのではなく、`models` で定義済みの名前を参照する**。DP モデルだけを
変えて比較したいときは、`models` に DP 指定だけ違う複数エントリを足し、`sim_runs` にその名前を並べる。

```yaml
    models:
      dp_a:
        vehicle_model: j6_gen2
        dp_model_release: 20260410-145919_lambda1e-6_ridge1e-8__epoch0020__epoch0040
      dp_b:
        vehicle_model: j6_gen2
        dp_model_release: 20260410-145919_lambda1e-6_ridge1e-8__epoch0020__epoch0030
    sim_runs: [dp_a, dp_b]        # 同一 vehicle_model で DP だけ違う 2-way 比較
```

### 2.2 方式 A: Web.Auto から自動 pull（推奨・`.webauto-ci.yml` 不要）

```yaml
    models:
      dp_e40:
        vehicle_model: j6_gen2
        dp_model_release: 20260410-145919_lambda1e-6_ridge1e-8__epoch0020__epoch0040
        # dp_model_package: diffusion_planner_for_x2_exp   # 既定。別 package のときのみ指定 (§7)
```

`dp_model_release` を指定すると Stage 3 が `webauto ml package-release` で **search → pull** して自動取得し、
そのモデルで走らせる。解決ロジックは `_resolve_dp_model_dir`（§3.3）。

### 2.3 方式 B: ローカル既存ディレクトリを直接指定

```yaml
    models:
      dp_local:
        vehicle_model: j6_gen2
        dp_model_dir: /opt/autoware/mlmodels/diffusion_planner_for_x2_exp
```

`dp_model_dir` 直下に **`diffusion_planner.onnx` と `args.json` の 2 ファイル**が必須。`~` / `${VAR}` は
loader が展開する（`load_models_doc` @ `lib/_models_config.py`）。`dp_model_release` とは排他。

### 2.4 設定キー一覧とバリデーション

| キー | 指定箇所 | 説明 |
|---|---|---|
| `dp_model_release` | `models.<name>` | Web.Auto ML release 名。指定で自動 pull。`dp_model_dir` と排他 |
| `dp_model_dir` | `models.<name>` | ローカル既存ディレクトリ（onnx + args.json）。`dp_model_release` と排他 |
| `dp_model_package` | `models.<name>` | `dp_model_release` 指定時の package 名。既定 `diffusion_planner_for_x2_exp`（§7） |

`load_models_doc` @ `lib/_models_config.py` が次を検証する（不正は即 `ValueError`）:

- `dp_model_dir` と `dp_model_release` の **同時指定はエラー**（どちらか一方）。
- `dp_model_package` は **`dp_model_release` 指定時のみ有効**（単独指定はエラー）。
- いずれも未指定なら Autoware 既定モデル（param yaml の `$(env ...)` フォールバック値）。

---

## 3. 仕組み詳細（実装）

実装はすべて `step3_run_sims.py`（DP モデルの解決・env 設定・エンジン事前ビルド）と、autoware_launch /
autoware_diffusion_planner 側の launch・param yaml の連携で成り立つ。

### 3.1 param yaml の `$(env)` 置換（「変更 A」）

autoware_launch の `config/planning/neural_net_planner/diffusion_planner.param.yaml`:

```yaml
onnx_model_path: $(env DIFFUSION_PLANNER_ONNX_PATH /opt/autoware/mlmodels/diffusion_planner_for_x2_exp/diffusion_planner.onnx)
args_path:       $(env DIFFUSION_PLANNER_ARGS_PATH /opt/autoware/mlmodels/diffusion_planner_for_x2_exp/args.json)
```

`autoware_diffusion_planner/launch/diffusion_planner.launch.xml` が
`<param from="$(var diffusion_planner_param_path)" allow_substs="true"/>` で読み込むため、param yaml 内の
`$(env ...)` 置換が解決される。**環境変数 `DIFFUSION_PLANNER_ONNX_PATH` を設定すればそのモデルがロードされ、
未設定なら従来の正規パスにフォールバック**する（ROS ネイティブ、C++ 変更不要、ファイル都度書き換え不要）。

### 3.2 run ごとの env 設定（`_setup_dp_model_env`）

`_setup_dp_model_env(dp_model_dir)` @ `step3_run_sims.py` が、解決済み `dp_model_dir` から `os.environ` に
次を設定する:

```
DIFFUSION_PLANNER_ONNX_PATH = <dp_model_dir>/diffusion_planner.onnx
DIFFUSION_PLANNER_ARGS_PATH = <dp_model_dir>/args.json
DP_ONNX_PATH                = <dp_model_dir>/diffusion_planner.onnx
```

> 注: **`DP_ONNX_PATH` は DiffusionPlanner ノードでは読まれない**。DP ノードが見るのは param yaml の
> `$(env DIFFUSION_PLANNER_ONNX_PATH ...)` だけ。`DP_ONNX_PATH` は provenance 解決
> （`_resolve_onnx` @ `lib/_provenance.py`）が記録用に参照する冗長エイリアスで、記録対象を実 onnx に
> 揃えるために同値をセットする。

onnx / args ファイルが欠落していれば既定モデルへ黙ってフォールバックせず **`RuntimeError` で loud に
落とす**（§4）。

### 3.3 Web.Auto 自動 pull の解決フロー（`_resolve_dp_model_dir`）

`_resolve_dp_model_dir(run)` @ `step3_run_sims.py`:

1. `webauto ml package-release search --package-name <pkg> --package-release-name <release>` で
   **release 名 → `package_id` / `release_id`** を解決（名前完全一致でフィルタ）。
2. `webauto ml package-release pull --target-dir <root>` で取得。配置は
   `<root>/<release-id>/<package>/{diffusion_planner.onnx, args.json}`。
3. onnx を含む dir を `dp_model_dir` として後続（env 設定・エンジン事前ビルド）に渡す。

| 項目 | 既定 | 上書き |
|---|---|---|
| Web.Auto プロジェクト | `x2_dev` (`_DEFAULT_WEBAUTO_PROJECT`) | env `WEBAUTO_PROJECT_ID` |
| pull 先 root | `~/.webauto/data/ml/package-release` (`_dp_pull_root`) | env `DP_MODEL_PULL_DIR` |

- **冪等**: `release-id` でキャッシュし、既取得なら再 pull をスキップ（TensorRT エンジンキャッシュも再利用）。
- **認証必須**: `webauto` の認証が要る。`webauto` 出力は JSON の前に進捗テキスト（`---` 等）を混ぜるため、
  `_webauto_json` が最初の JSON 開始文字以降だけを解析する。

### 3.4 インストール済 param への `$(env)` 動的注入（`_ensure_installed_param_honors_env`）

§3.1 の `$(env)` 化（変更 A）は、**build_only と scenario の両方が読む *インストール済* param yaml** が
`$(env DIFFUSION_PLANNER_ONNX_PATH ...)` を持って初めて効く。`_ensure_installed_param_honors_env`
@ `step3_run_sims.py` が、起動前に *インストール済* `diffusion_planner.param.yaml`（`ros2 pkg prefix
autoware_launch --share` で解決）へ `$(env ...)` を**実行時注入**する。これにより
**launcher サブリポジトリ / autoware.repos を一切変更せず**に env 切り替えを有効化できる。

- **冪等**: 既に `$(env` があれば no-op。env 未設定時は元の既定値にフォールバックするため挙動不変・restore 不要。
- ローカルは symlink-install で source に変更 A 済なら no-op、クラウドはインストール済コピーへ注入。
- **書き込み不可（install が read-only）の場合のみ `RuntimeError`** を送出し、代替（launcher へ変更 A を
  コミット + autoware.repos 更新）を案内する（黙った既定モデル実行を防止。§4）。

### 3.5 TensorRT エンジンの事前ビルドと隔離（`_prebuild_dp_engine`）

env でモデルを切り替えると各モデルの初回ロード時にエンジンが**遅延ビルド**され、計時付きの scenario 実行
中だとゴール到達前にタイムアウトし得る。`_prebuild_dp_engine(dp_model_dir)` @ `step3_run_sims.py` が、
scenario 起動前に `diffusion_planner.launch.xml build_only:=true` を一度走らせ、エンジンを model dir に
事前生成する（既にあれば高速ロードして終了、冪等）。

エンジンファイル名は **onnx パスから導出**され（`<onnx の stem>_batch<N>_fp32.engine`、
`tensorrt_inference.cpp` の `load_engine`）、**onnx と同じディレクトリ**にキャッシュされる。自動 pull は
モデルごとに別ディレクトリ（`<release-id>/<package>/`）へ配置されるため、**エンジンも完全に隔離**され、
「モデル B が黙ってモデル A のエンジンで走る」確信的誤りは起きない。

---

## 4. silent-wrong ガード（重要）

`provenance.json` は step3 が立てた env（＝**意図**）を記録するだけで、**Autoware が実際にロードした
モデルそのものではない**。変更 A が *インストール済* autoware_launch に未反映だと、Autoware は既定モデルを
全 run でロードするのに provenance は dp_model_dir 別の sha8 を記録し、「別モデルなのに同一挙動」という
誤りが全チェック緑のまま通り得る。これを防ぐため `_setup_dp_model_env` が起動前に検証する:

1. **onnx / args ファイルの存在**（欠落時は既定へフォールバックせず `RuntimeError`）。
2. **インストール済 param yaml への `$(env ...)` 注入**（`_ensure_installed_param_honors_env`、§3.4）。
   未適用なら DLR が注入して有効化する。install が read-only で**注入できない場合のみ** `RuntimeError` を
   送出し、代替を案内する。

即時確認コマンド:

```bash
grep onnx_model_path "$(ros2 pkg prefix autoware_launch --share)/config/planning/neural_net_planner/diffusion_planner.param.yaml"
# → $(env DIFFUSION_PLANNER_ONNX_PATH ...) が出れば変更 A 反映済
```

---

## 5. provenance と可視化

- `step3` が各 lite に `provenance.json` を書く（`write_provenance` @ `lib/_provenance.py`）。記録項目は
  `dp_onnx_path` / `dp_onnx_sha8` / `dp_exp_name`（args.json の `exp_name`）/ `dp_train_set` /
  `autoware_version` と、step3 が渡す `tag` / `vehicle_model` / `dp_model_dir` / `dp_model_release` 等。
- `format_provenance_line` @ `lib/_provenance.py` が `DP=<exp> (onnx <sha8>) / autoware <ver>` の 1 行に整形し、
  比較プロット・`report.md` / `report.html` にモデル識別を掲載する。
- 軌跡 overlay（Stage 4）・DP 軌跡比較（Stage 8）・`report.html` は **tag（モデル名）単位**の N-way 比較の
  ため、`dp_model_*` を変えた run も追加実装なしで重ね描き・比較できる。

> provenance は「意図」の記録であり、**実ロードの証明には必要だが十分でない**。実際にそのモデルが
> ロードされたことの担保は §4 のガード（または DP ノードの起動ログ）に依存する。

---

## 6. クラウド実行の前提

**変更 A のクラウド反映は不要**（step3 がインストール済 param へ実行時注入するため。§3.4）。launcher
サブリポジトリ / autoware.repos の変更は行わなくてよい。ただし以下が前提:

1. **install が書き込み可能なこと**: step3 がインストール済 `diffusion_planner.param.yaml` に `$(env ...)` を
   注入できること（sim runtime ユーザーが書き込めること）。read-only なら `RuntimeError` で停止し、代替
   として launcher へ変更 A をコミット + autoware.repos 更新を案内する。
2. **webauto 認証**: cloud の sim runtime コンテナで `webauto` 認証が使えること（自動 pull 用）。使えない
   環境では、従来どおり `.webauto-ci.yml` の asset-deploy でモデルを配置し `dp_model_dir` で指す。

---

## 7. 2 つの DP package の使い分け

`dp_model_release` の解決先 package は `dp_model_package` で選ぶ。既定は **`diffusion_planner_for_x2_exp`**
（`_DEFAULT_DP_PACKAGE` @ `lib/_models_config.py`）。

| package | 位置づけ |
|---|---|
| `diffusion_planner_for_x2_exp` | 実験版（既定）。新モデル試行・診断用 |
| `diffusion_planner_for_x2` | 本番版。`.webauto-ci.yml` asset-deploy で配置される長期安定重み |

> **罠**: ある release が想定と違う package（例: `_exp` ではなく無印 `diffusion_planner_for_x2`）にしか
> 存在しない場合、`dp_model_package` を省略すると既定 `_exp` を探して search が失敗する。release が属する
> package を確認し、必要なら `dp_model_package: diffusion_planner_for_x2` を明示する。

---

## 8. 実行手順（How-to）

### ローカル（`make local_cloud_run`）

```bash
cd src/simulator/driving_log_replayer_v2/driving_log_replayer_v2/driving_log_replayer_v2/real_log_sim_comparison
make local_cloud_run                 # scenario.yaml の Datasets[0] を 1 件実行
make local_batch_run                 # Datasets 全 UUID を順次実行 + 横断分析
```

- 事前に対象走行の T4 dataset を `webauto data annotation-dataset pull --include-intermediate-artifacts`
  で取得しておくこと（詳細は [`../sample/README.ja.md`](../sample/README.ja.md)）。
- `SKIP_SIM=1` を付けると Stage 3（closed-loop sim 実行）を省略し open-loop 解析のみ走る。DP モデル比較は
  closed-loop なので、比較したいときは `SKIP_SIM` を付けない。

Makefile はローカルでも `driving_log_replayer_v2.launch.py` に `with_autoware:=false` を渡す。これは
**外側の DLR launch が Autoware を立てない**意味で、closed-loop sim 自体は Stage 3 の `step3_run_sims` が
内部で `scenario_test_runner.launch.py`（= Autoware + DiffusionPlanner）を起動して動かす。したがって
**ローカルでも closed-loop sim が走り、DP モデル間の軌跡・速度差を観察できる**（ego は自律走行する）。

> run 間でモデル挙動を比較するときは、各 run が同じ実機データセット（`Datasets[0]` の UUID）と replay
> 設定で走っていることを `provenance.json` で確認すること。dataset / replay 区間が違うと差が「モデル差」
> なのか「入力差」なのか切り分けられない。

### クラウド（Web.Auto evaluator）

登録済み Web.Auto scenario の `Conditions` に同じ `models` / `sim_runs` を書けば、評価ジョブとして同一
パイプラインが走る。前提は §6 の通り（install 書き込み可 + webauto 認証）。

---

## 9. トラブルシューティング

| 症状 | 原因 / 対処 |
|---|---|
| `DP モデルが見つかりません: .../diffusion_planner.onnx` | `dp_model_dir` 直下に `diffusion_planner.onnx` + `args.json` が無い。パスを確認（§2.3） |
| `ML package-release が見つかりません` | release 名の誤り、`dp_model_package` の取り違え（§7）、または webauto 未認証。`webauto auth` を確認 |
| `dp_model_dir と dp_model_release は同時指定できません` | 排他。どちらか一方に（§2.4） |
| `install が read-only で $(env ...) を書き込めません` | sim runtime の install が read-only。launcher へ変更 A をコミット + autoware.repos 更新（§3.4 / §6） |
| 別モデルのはずが挙動が同じ | 変更 A 未反映の疑い。§4 の `grep` で param yaml が `$(env ...)` を持つか確認 |
| scenario が DP 初回ロードでタイムアウト | エンジン遅延ビルド。`_prebuild_dp_engine` が走っているか（autoware_launch の param 解決失敗時はスキップされ警告が出る。§3.5） |

---

## 10. 検証

| レイヤ | 内容 |
|---|---|
| ローカル機構 | `grep` でインストール済 param が `$(env ...)` を含むことを確認（symlink-install なら即反映）。`webauto ml package-release search/pull` の自動取得を実行で確認 |
| provenance 分離 | 各 sim run の `provenance.json` で `dp_onnx_sha8` / `dp_exp_name` が run ごとに異なること（**必要条件だが十分でない**: provenance は意図記録のため、実ロードの証明は §4 ガード or DP ノードログに依存） |
| 挙動差比較 | ローカル / クラウドいずれも closed-loop で ego が自律走行するため、軌跡・速度差を比較できる。比較時は同一 dataset UUID / replay 設定であることを provenance で確認する |
