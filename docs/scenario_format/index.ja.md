# driving_log_replayer_v2 シナリオフォーマット定義

driving_log_replayer_v2 で用いるシナリオのフォーマットについて述べる。

## フォーマットに関する注意事項

- キーは CamelCase にて定義する。Scenario Format Version 3.0まで
- pydanticを用いたシナリオのバリデーションのため、新規に追加するキーはsnake_caseを推奨 Scenario Format Version 3.1より
- 座標系に関しては、 `map` 座標系を使用する
- 単位系に関しては、特に指定がなければ以下を使用する。

```shell
距離: m
速度: m/s
加速度: m/s^2
時間: s
```

## サンプル

シナリオのサンプルを[sample](https://github.com/tier4/driving_log_replayer_v2/tree/develop/sample) フォルダに置いている。

## フォーマット

基本構造は以下の通り。各キーの詳細は以下で記述する。

### driving_log_replayer_v2 Scenario Format version 3.x.x

```yaml
ScenarioFormatVersion: 3.x.x
ScenarioName: String
ScenarioDescription: String
SensorModel: String
VehicleModel: String
publish_profile: String
Evaluation:
  UseCaseName: String
  UseCaseFormatVersion: String
  Conditions: Dictionary # refer use case
  Datasets:
    - DatasetName:
        VehicleId: String
        InitialPose: Dictionary # optional, 省略時は {} として扱われる (無効化する場合はキーを省略するか {} を指定すること)
        DirectInitialPose: Dictionary # optional, 省略時は {} として扱われる (無効化する場合はキーを省略するか {} を指定すること)
        GoalPose: Dictionary # optional, 省略時は {} として扱われる (無効化する場合はキーを省略するか {} を指定すること)
        goal_method: String # optional, default: set_goal_from_scenario
        time_offset: Dictionary # optional, default: {}
include_use_case:
  UseCaseName: String
  UseCaseFormatVersion: String
  Conditions: Dictionary
```

### ScenarioFormatVersion

シナリオフォーマットのバージョン情報を記述する。セマンティックバージョンを用いる。

対応する最新バージョンは 3.3.0。従来の 3.0.0、3.1.0、3.2.0 も引き続き利用できる。

フォーマットの更新の度にマイナーバージョンを更新する。

### ScenarioName

シナリオの名前を記述する。Autoware Evaluator 上でシナリオの表示名として使用される。

### ScenarioDescription

シナリオの説明を記述する。Autoware Evaluator 上でシナリオの説明として使用される。

### SensorModel

autoware_launch/launch/logging_simulator.launch.xml の引数の sensor_model を指定する

### VehicleModel

autoware_launch/launch/logging_simulator.launch.xml の引数の vehicle_model を指定する

### publish_profile

(オプション) Publishするトピックを制御するためのトピックプロファイル名を指定する。プロファイルファイルは `config/publish_and_remap/{profile_name}.yaml` に配置する必要がある、例: `planning_control`。指定しない場合は、Rosbag中すべてのトピックがPublishされる。

### Simulation.AdditionalLaunches

ScenarioFormatVersion 3.3.0 からのオプション。Autoware と並行して常駐する追加 launch を起動する。DLR にハードウェア専用引数を追加せず、シナリオに必要な処理を指定する。初期対応は `obstacle_segmentation`、`record_only:=false` のみ。

```yaml
ScenarioFormatVersion: 3.3.0
Simulation:
  AdditionalLaunches:
    - package: aip_hd785_gen2_perception_launch
      file: radars_replay.launch.xml
      arguments:
        use_sim_time: "true"
```

この radar の例は、AIP 側に再生専用 launch を別途追加・インストールして使用する。DLR は当該 launch を提供せず、AIP パッケージへの依存も追加しない。既存の `Evaluation` 設定と `UseCaseFormatVersion: 0.3.0` は維持する。評価対象トピックの remap は別機能であり、この設定では変更されない。両機能は ScenarioFormatVersion 3.3.0 にまとめる方針だが、本ブランチの実装対象は追加 launch のみ。

- `package` と `file` は必須。インストール済みパッケージの `launch/` 配下から相対パスで解決する。サブディレクトリと `.launch.py`、`.launch.xml`、`.launch.yaml`、`.launch.yml` に対応。絶対パス、`..`、範囲外を指す symlink、存在しない package/file、同一エントリ、DLR 自身を直接再帰起動する指定はシミュレーション開始前に拒否する。
- `arguments` は省略時 `{}`。キーは launch 引数名、値は文字列。真偽値・数値も引用符で囲む。シェルを使わず引数を個別に渡す。親の `SetParameter` は別プロセスへ伝わらないため、追加 launch 側で `use_sim_time` を宣言し、ノードへ適用する。
- `Simulation` の省略・`null`・`{}`、`AdditionalLaunches` の省略・`[]` では既存の動作を維持する。`AdditionalLaunches: null` や新設定内の未知のキーはエラー。空でないリストを旧バージョン、他ユースケース、record-only で指定した場合もエラー。`with_autoware:=false` は許可するが、残りのパイプラインは呼び出し側で用意する。
- evaluator/player より前に追加 launch の起動を要求するが、ノードの準備完了は保証しない。既存の `play_delay` は変更しない。bag 再生前の subscription と起動時間を確認する。準備完了ハンドシェイクや自動再起動は提供しない。
- DLR の終了前に追加 launch が終了した場合、終了コード 0 も含めて失敗とし、シミュレーションを停止する。重要な子ノードが終了したら、追加 launch 自体も終了するよう各 launch を構成する。launch が生きたままの個別ノード故障は DLR から検出できない。DLR の通常終了時は子 launch へ停止シグナルを送り、ROS launch 標準の終了待ち・強制終了を利用する。
- `result_archive/additional_launches.json` に実行 ID、解決したパス、SHA-256、package version、指定引数を保存する。予期しない終了は `additional_launch_failure.json` に保存。後処理は既存フレームを残し、最終結果へ `AdditionalLaunchFailed` を追記する。bag 未生成でも Fail を出力し、その失敗実行では bag/plot 解析をスキップする。実行 ID により前回の失敗マーカーを区別する。
- bag の publish/remap/record 設定、評価 subscription、TopicRate 診断は変更しない。追加出力を保存する場合は別途記録対象を設定する。指定引数は平文で記録されるため、認証情報を渡さない。

[サンプルシナリオ](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/obstacle_segmentation/scenario_additional_launches.yaml)を参照。起動入口は従来の `driving_log_replayer_v2.launch.py` のままで、追加 CLI 引数は不要。Evaluator で使用する前に、3.3.0 対応 DLR イメージと、新しい設定を保存・配信できるシナリオサービスを確認する。

### Evaluation

シミュレーションの評価条件を定義する。

#### UseCaseName

評価プログラムを指定する。

ここで指定された名前と同じ名前の評価ノードを呼び出すことで評価が実行される。

#### UseCaseFormatVersion

ユースケースのフォーマットのバージョン情報を記述する。セマンティックバージョンを用いる。
メジャーバージョンが 1 になるまでは、フォーマットの更新の度にマイナーバージョンを更新する。初期バージョンは 0.1.0。

#### Conditions

ユースケース毎に設定できる条件を指定する。

指定可能な条件は各ユースケースを参照。

#### Datasets

複数個のDatasetを記述することが可能であるが、複数個のDatasetに対して、同じ評価条件で使用する場合のみ利用できる。
複数個のDatasetを記述した場合は、利用したいdatasetのindexをlaunchの起動引数に渡す必要がある。
indexは0番から始まる。
データセットが1個の場合はdataset_index:=0としてもよい。

```shell
# シナリオに記述したdataset数が1個の場合。dataset_index:=0は省略可能
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=${scenario_path} [dataset_index:=0]

# シナリオに記述したdataset数が複数の場合
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=${scenario_path} dataset_index:=${index_number}
```

#### DatasetName

t4_datasetのデータセット名

#### VehicleId

autoware_launch/launch/logging_simulator.launch.xml の引数の vehicle_id を指定する。

車両 ID が不明な場合は、`default` を設定する。

#### InitialPose

（オプション）`/localization/initialize` の service を call するための初期ポーズを指定する。

```yaml
InitialPose:
  position:
    x: float
    y: float
    z: float
  orientation:
    x: float
    y: float
    z: float
    w: float
```

#### DirectInitialPose

（オプション）`/localization/initialize` の service を call するための初期ポーズを指定する。InitialPoseとの違いは[autoware_pose_initializer](https://github.com/autowarefoundation/autoware_core/tree/main/localization/autoware_pose_initializer)を参照。InitialPose と DirectInitialPose の両方が指定された場合は、DirectInitialPose が優先され、InitialPose は無視される。

```yaml
DirectInitialPose:
  position:
    x: float
    y: float
    z: float
  orientation:
    x: float
    y: float
    z: float
    w: float
```

#### GoalPose

（オプション）`/api/routing/set_route_points` の service を call するためのゴールポーズを指定する。

```yaml
GoalPose:
  position:
    x: float
    y: float
    z: float
  orientation:
    x: float
    y: float
    z: float
    w: float
```

#### goal_method

（オプション）ゴールポーズの設定方法を指定する。シナリオYAMLのデータセットセクションで `GoalPose` と同じ階層に配置する。

| 値                       | 説明                                                                                                                                |
| ------------------------ | ----------------------------------------------------------------------------------------------------------------------------------- |
| `set_goal_from_scenario` | （デフォルト）シナリオの `GoalPose` を使用してゴールを設定する。`GoalPose` が未指定の場合は何もしない。                             |
| `set_goal_from_rosbag`   | rosbagの最後の `/localization/kinematic_state` を読み取り、その位置をゴールとして使用する。 `GoalPose` が指定されていても無視する。 |

#### time_offset

（オプション）rosbagの再生開始時刻と再生時間を指定する。前者はtimestamp基準ではなく、rosbagの開始からの相対時間で指定する。シナリオYAMLのデータセットセクションで `GoalPose` と同じ階層に配置する。

```yaml
time_offset:
  start: float # rosbag再生開始時刻 [s]
  duration: float # rosbag再生時間 [s]
```

### include_use_case

Evaluationで指定したユースケースとは別のユースケースの評価を同時に行う場合に使用する。
ここに記載したuse_caseのノードが自動で立ち上がるわけではない。

各use_caseのevaluatorノードにinclude_use_caseで指定された条件で評価する処理を追加する必要がある。
Evaluatorでは、result.jsonlの最終行を処理して、成否の判定が行われるので、Evaluationの結果result.jsonlと、include_use_caseで出力されるresult.jsonlの結果をpost_processでマージする必要がある。

現状、planning_controlでdiagnosticsを評価する機能が実装されている。
