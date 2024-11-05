# Diagnosticsの評価

diagnosticsが指定の時間に指定のレベルになっているかを評価する

類似の評価にperformance_diagがあるが、そちらはLiDARに特化している。
diagnostics_evaluator_nodeは、levelを評価するシンプルな機能しかないが、任意のstatus.nameに対応

## 評価方法

launch を立ち上げると以下のことが実行され、評価される。

1. launch で評価ノード(`diagnostics_evaluator_node`)と `logging_simulator.launch`、`ros2 bag play`コマンドを立ち上げる
2. bag から出力されたセンサーデータを autoware が受け取って、`/diagnostics`に診断情報を出力する
3. 評価ノードが topic を subscribe して、各基準を満たしているかを判定して結果をファイルに記録する
4. bag の再生が終了すると自動で launch が終了して評価が終了する

## 評価結果

受信したmsgのmsg.status[0].hardware_idがシナリオに指定したhardware_idと一致し、かつmsg.header.stampがシナリオに指定した時間を満たしていれば評価される。
評価の条件を満たさない場合は、ログも出力されない。

### 正常

msg.statusにシナリオで指定したnameかつ、levelを満たすstatusが存在している。

### 異常

正常の条件を満たさないとき

## 評価ノードが使用する Topic 名とデータ型

Subscribed topics:

| Topic name   | Data type                             |
| ------------ | ------------------------------------- |
| /diagnostics | diagnostic_msgs::msg::DiagnosticArray |

Published topics:

| Topic name | Data type |
| ---------- | --------- |
| N/A        | N/A       |

## logging_simulator.launch に渡す引数

なし(デフォルトのまま起動)

bagの中に入っている、/sensing/lidar/concatenated/pointcloudを利用する場合は、launchの引数にsensing:=falseを追加する
perception、planningも同様にbagから出力する場合は、launchの引数にperception:=false planning:=falseを追加する

```shell
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=${daignostics_scenario_path} sensing:=false perception:=false planning:=false
```

## simulation

シミュレーション実行に必要な情報を述べる。

### 入力 rosbag に含まれるべき topic

どのtopicが必要かはやりたいこと次第。

### 入力 rosbag に含まれてはいけない topic

| topic 名 | データ型                |
| -------- | ----------------------- |
| /clock   | rosgraph_msgs/msg/Clock |

clock は、ros2 bag play の--clock オプションによって出力しているので、bag 自体に記録されていると 2 重に出力されてしまうので bag には含めない

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/diagnostics/scenario.yaml)参照

### 評価結果フォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/diagnostics/result.json)参照

以下に、それぞれの評価の例を記述する。
**注:結果ファイルフォーマットで解説済みの共通部分については省略する。**

設定した全ての評価条件で成功している場合に成功と判定される。

```json
{
  "Frame": {
    "Condition_IDNEX": {
      "Result": {"Total": "Success or Fail", "Frame": "Success or Fail"},
      "Info": {
        "TotalPassed": "評価条件をパスしたtopicの総数",
        "Level": "取得したstatusのLevel"
      }
    }
  }
}
```
