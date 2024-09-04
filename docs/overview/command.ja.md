# コマンド

driving_log_replayer_v2はシナリオのパスを指定することで起動できる。

```shell
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=${scenario_path} [output_dir:=${output_dir} dataset_dir:=${dataset_dir}]
```

## wasim による driving_log_replayer_v2 実行

TIER IV が提供している[Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction)へ
アクセス権がある場合は[wasim](https://docs.web.auto/developers-guides/wasim/introduction)を利用することもできる。

使い方は[ドキュメントサイト](https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/)を参照。

wasim は Autoware Evaluator からシナリオをダウンロードして実行するので、クラウド環境に登録済みのシナリオしか実行出来ない。

## driving-log-replayer-v2-cli による driving_log_replayer_v2 実行

cliを利用すると、1回のコマンド入力で複数のシナリオを連続で実行できます。

例えば以下のように、SCENARIO_ROOT配下に複数のシナリオがサブディレクトリ(SCENARIO_DIR1, SCENARIO_DIR2...)に置かれているとします。

```shell
SCENARIO_ROOT
├── SCENARIO_DIR1
│   ├── out # output directyory
│   └── SCENARIO_DIR1_DATASET0 # t4_dataset
│   │  ├── annotation
│   │  ├── data
│   │  ├── input_bag
│   │  ├── map
│   │  └── status.json
│   └── SCENARIO_DIR1_DATASET1 # t4_dataset
│   │  └── ...
│   │  ...
│   └── scenario.yaml  # scenario fixed name
│
├── SCENARIO_DIR2
│   ├── out # output directyory
│   └── SCNERIO_DIR2_DATASET0 # t4_dataset
│   │  ├── annotation
│   │  ├── data
│   │  ├── input_bag
│   │  ├── map
│   │  └── status.json
│   └── scenario.yaml  # scenario fixed name
...
```

上記のSCENARIO_ROOTにある複数シナリオをros2 launchコマンドで実行するには以下のように複数回のコマンドを叩く必要があります。

```shell
source ${AUTOWARE_ROOT}/install/setup.bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2 scenario_path:=${SCENARIO_ROOT}/SCENARIO_DIR1/scenario.yaml dataset_index:=0 # 複数datasetある場合
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2 scenario_path:=${SCENARIO_ROOT}/SCENARIO_DIR1/scenario.yaml dataset_index:=1 # 複数datasetある場合
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2 scenario_path:=${SCENARIO_ROOT}/SCENARIO_DIR2/scenario.yaml
...
```

cliを使う場合は以下のコマンドで済みます

```shell
source ${AUTOWARE_ROOT}/install/setup.bash
dlr2 simulation run ${SCENARIO_ROOT}
```

### cli installation

以下のコマンドでcliをインストールできる。

```shell
# install
pipx install git+https://github.com/tier4/driving_log_replayer_v2.git

# upgrade
pipx upgrade driving-log-replayer-v2

# uninstall
pipx uninstall driving-log-replayer-v2
```

### cli limitation

cliでシナリオファイルを自動で探すため以下の制約がある。

- シナリオファイルの名前がscenario.yamlでなければならない
- SCENARIO_ROOTの下のサブディレクトリにscenario.yamlが存在しなければならない(サブサブディレクトリは不可)

### cli output

cliを利用すると、コマンド入力を短くするだけでなく出力されるファイルが増える。

以下が、cliを利用した場合の出力先の例である。
simulationの実行コマンドと、コンソールのログがファイルとして出力されている。
複数個のテストを連続で実行し、後でエラーが出たテストだけデバッグするといった場合に利用する。

```shell
OUTPUT_LATEST
├── 0 # Datasets[0]の結果
│   ├── result.jsonl
│   ├── result_archive_path
│   └── result_bag
│       ├── metadata.yaml
│       └── result_bag_0.db3
├── 1 # Datasets[1]の結果
│   ├── result.jsonl
│   ├── result_archive_path
│   └── result_bag
│       ├── metadata.yaml
│       └── result_bag_0.db3
├── console.log # コンソールに表示されているログをファイル化したもの
└── run.bash # simulationの実行コマンド
```
