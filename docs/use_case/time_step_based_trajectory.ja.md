# タイムステップベースの軌跡の評価

Autoware から出力される自車の軌跡についての評価を行う。現状、何かしらの基準に対する評価は行わない(TBD)。後処理において `autoware_tools` の `autoware_planning_data_analyzer` を通じて分析のみを行う。

## logging_simulator.launch に渡す引数

- perception: false
- localization: false
- control: false

## simulation

シミュレーション実行に必要な情報を述べる。

### 入力 rosbag に含まれてはいけない topic

| topic 名 | データ型                |
| -------- | ----------------------- |
| /clock   | rosgraph_msgs/msg/Clock |

clock は、ros2 bag play の--clock オプションによって出力しているので、bag 自体に記録されていると 2 重に出力されてしまうので bag には含めない

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/time_step_based_trajectory/scenario.yaml)参照

### 評価結果フォーマット

結果ファイルの内容は常に同じ。

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/time_step_based_trajectory/result.json)参照
