# 認識機能のstop_reasonを利用した評価

一般的な[認識機能の評価](/docs/use_case/perception.ja.md)に加えて、Autoware のステータスを示す一部である stop_reason を考慮した判定も行える。

基本的には認識機能の評価と同じ仕様になる。こちらでは相違点のみ説明する。

## 評価結果

pass/fail を判定する topic (/awapi/autoware/get/status) の subscribe 1 回につき、以下に記述する判定結果が出力される。但し、 `check_interval` によって評価するインターバルを調整することができる。

### 正常

シナリオのCriterionタグのCriteriaを満たすこと。

sample の scenario.yaml は以下のようなっており、

```yaml
stop_reason_criterion:
  - time_range: 1649143500-1649143505 # [second] lower_limit-(upper_limit) [Upper limit can be omitted. If omitted value is int(1.7976931348623157e+308)]
    criteria_name: check_obstacle_stop # criteria name
    pass_rate: 80.0 # How much (%) of the evaluation attempts are considered successful.
    minimum_interval: 1.0 # Minimum interval in the evaluation frame
    evaluation_type: stop # "stop" or "non_stop"
    condition:
      - reason: ObstacleStop # Specify the reason you want to meet if "stop"
        base_stop_line_dist: 0.0-10.0 # [m] lower_limit-(upper_limit) [Upper limit can be omitted. If omitted value is 1.7976931348623157e+308]
  - time_range: 1649143506-1649143510 # [second] lower_limit-(upper_limit) [Upper limit can be omitted. If omitted value is int(1.7976931348623157e+308)]
    criteria_name: check_non_stop # criteria name
    pass_rate: 90.0 # How much (%) of the evaluation attempts are considered successful.
    minimum_interval: 1.0 # Minimum interval in the evaluation frame
    evaluation_type: non_stop # "stop" or "non_stop"
    condition:
      - reason: Intersection # Specify the reason you do not want to meet if "non_stop"
        # Cannot specify "base_stop_line_dist" if "non_stop"
```

- pass/fail を判定する topic の subscribe 1回に対して、1649143500-1649143505[second]のタイムスタンプを持っているステータスの中で、 ObstacleStop の reason を持っていれば、 Frame としては Success になる。
- pass/fail を判定する topic の subscribe 1回に対して、1649143506-1649143510[second]のタイムスタンプを持っているステータスの中で、Intersection の reason を持っていなければ、 Frame としては Success になる。
- また、`PassRate >= 正常数 / 評価数 * 100`の条件を満たすとき、Total として Success になる。

### 異常

正常の条件を満たさない場合

### 評価スキップTimeout

- 対象となるタイムスタンプ出なかった場合
- 前回評価したタイムスタンプと今回評価するタイムスタンプの差が `minimum_interval` より小さかった場合

## 評価スクリプトが使用する Topic 名とデータ型

stop_reason を使用した pass/fail の判定には tier4_api_msgs/msg/AwapiAutowareStatus 型を使用する。

## logging_simulator.launch に渡す引数

stop_reason を得るためには Autoware の planning と control の機能を有効にする必要がある。

- localization: false
- planning: true
- control: true

## evaluation

### 評価結果フォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/perception/result_stop_reason.json)参照

stop_reason を使用した認識機能の評価では、output_dir/result_archive/stop_reason.jsonl の中に別で結果が書き込まれる。

各フレームのフォーマット

```json
{
 "Frame": {
  "criteria0": {
   "PassFail": {
    "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
        "Info": {
          "Reason": "指定したreasonと一致したreason or non_stop_reason",
     "Distance": "の距離 or 0.0",
     "Timestamp": "topicのheaderのタイムスタンプ",
        },
    "StopReason": {
     [
      "index": "reasonのindex",
      "reason": "停止理由",
      "dist_to_stop_pose": "停止位置との距離",
      "x": "停止位置のx座標",
      "y": "停止位置のy座標",
      "z": "停止位置のz座標",
      "qx": "停止位置の回転姿勢x",
      "qy": "停止位置の回転姿勢y",
      "qz": "停止位置の回転姿勢z",
      "qw": "停止位置の回転姿勢w",
     ]
    }
   }
  }
 }
}
```

Timeout のフォーマット

```json
{
  "Frame": {
    "criteria0": {
      "Timeout": "Timeoutした回数"
    }
  }
}
```
