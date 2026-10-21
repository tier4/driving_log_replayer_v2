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

### 実行設定はlaunch引数で指定

追加launch・評価topic・参照診断はシナリオに書かず、ローカルでは`ros2 launch`引数、
Evaluatorでは`.webauto-ci.yml`の`simulations[].simulator.parameters`へ同名の文字列を指定します。
シナリオの再生成・ScenarioFormatVersionやUseCaseFormatVersionの引き上げは不要です。
バージョン3.0.0〜3.3.0の既存シナリオを使用できます。

```bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/path/to/scenario.yaml \
  additional_launches:='[{"package":"aip_hd785_gen2_perception_launch","file":"radars_replay.launch.xml","arguments":{"use_sim_time":"true"}}]' \
  evaluation_topic:=/perception/radar_lidar_fusion/debug/appended_radar_points \
  evaluation_topic_rate:='null'
```

上記は起動引数部分の例です。通常のt4_dataset_path等の引数は別途指定してください。
汎用の追加launch機能であり、AIPのradar launch自体は別途インストールが必要です。
DLRへAIPパッケージや専用topic monitorの依存は追加しません。

| 引数 | 型・既定値 | 意味 |
| --- | --- | --- |
| additional_launches | JSON配列の文字列、既定`[]` | `package`・`file`・任意の`arguments`。argumentsの値も文字列 |
| evaluation_topic | ROS完全修飾topic名、既定空文字 | 評価器のPointCloud2購読先。記録対象にも追加 |
| evaluation_topic_rate | JSONオブジェクトまたはJSON `null`の文字列。既定内部値`legacy` | moduleとnode_name_suffixで既存診断を選択。新monitorは起動しない |

| evaluation_topic | evaluation_topic_rate | 動作 |
| --- | --- | --- |
| 省略 | 省略 | 元の点群と固定診断、従来の結果・記録動作を維持 |
| 指定 | 省略／`null` | 指定topicを評価・記録、レート条件はSkipped |
| 省略 | `null` | 元の点群を評価・記録、レート条件はSkipped |
| 指定／省略 | JSON設定あり | 指定topic（省略なら元の点群）を評価し、指定診断を参照 |

`legacy`は省略検出用の既定値です。明示的に監視したい場合は次のJSONを指定します。

```bash
evaluation_topic_rate:='{"module":"perception","node_name_suffix":"obstacle_segmentation_pointcloud"}'
```

生成される診断名は`topic_state_monitor_obstacle_segmentation_pointcloud: perception_topic_status`。
トピック名から診断名を推測しません。実際のAutoware設定と対応させてください。
/diagnosticsの全statusを検索し、同名が複数あれば最悪レベルを使います。
OK/WARNはレート条件を満たし、ERROR/STALEは満たしません。Hzしきい値はAutoware側の設定に従います。
指定診断が未受信・ROS時刻で2秒超の期限切れなら、5秒間隔で抑制した警告と
`TopicRateEvaluation.state: Invalid`、理由`NotReceived`または`Expired`を記録します。
`TopicRate`はFalseとなりレート条件を満たしません。設定したのに見つからない場合を自動Skipしません。
無関係な診断は期限を更新せず、古い診断・時刻巻戻り・再生pauseに対応します。
監視無効では`TopicRate: null`と`TopicRateEvaluation: {enabled: false, state: Skipped}`を記録し、
レート条件だけを合否から除外します。点群の検知・非検知・PassRateは適用し、擬似OK graphは出しません。
最終結果の独立した失敗ラッチは追加しません。点群が来ない間の最終合否・NoDataは既存処理に従います。

新しい非既定引数はobstacle_segmentation、record_only:=falseのみ対応します。
追加launchは評価器/playerより前に起動を要求しますが、準備完了は保証しません。
必要に応じplay_delayを設定し、/clockが必要な追加launchにはuse_sim_timeを明示します。
追加launchの異常終了（終了コード0を含む）は実行失敗として停止・resultへ反映します。
重要な子ノードの終了時には追加launch自体が終了するよう構成してください。
result_archive/additional_launches.jsonへ実行ID、解決パス、SHA256、引数を保存し、
失敗はadditional_launch_failure.jsonへ保存します。通常停止・過去の実行IDは失敗にしません。

recorderは従来のbase（override_topics_regex ＞ record_profile ＞ use_case標準profile）に
評価topicを完全一致で追加し、監視有効時は/diagnosticsも追加します。別の記録引数は不要です。
override指定時も追加されます。元のprofileと元点群は削除しません。指定なしなら既存regex・空profile時のskipを維持します。
補助cluster点群・他debug topicは必要なら既存record_profile/overrideへ別途含めてください。
記録対象追加はpublisher・QoS一致・全件記録の保証ではありません。
追加前後の点群を保存すると容量・I/Oが増えます。bag playerの/unused remapは変更しません。

旧案のシナリオ内`Simulation.AdditionalLaunches`（非空）または`Evaluation.EvaluationTopic`（非null）は
無視せず移行エラーにします。空・nullの旧指定は従来動作として扱います。
設定は親からpre/simulation/postへ1引数ずつ転送し、各子で再検証します。
評価器へは正規化済みJSONを文字列ROSパラメータで渡し、購読先とrecord先の一致を保ちます。
実行イメージを機能対応コミットへ固定し、起動ログで設定反映を確認してください。
analyzerは新モードの状態を`3_topic_rate_evaluation.json`にも保存します。
受信周期Hzグラフは参考値で、診断による合否とは別です。

CLI記述例は`sample/obstacle_segmentation/launch_arguments.md`にまとめています。
