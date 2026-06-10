# 車両制御モデル（数式・座標系・定数）

---

## 1. 座標系の定義

車体基準（body frame）と進行方向基準で以下を定義する。

| 記号 | 意味 | 単位 |
|---|---|---|
| \(x, y\) | 地図平面上の車両位置 | m |
| \(\theta\) | ヨー角（車体の向き。地図 X 軸からの回転） | rad |
| \(v\) | 前進速度（進行方向＝車体 X 軸成分。`lon_vel`） | m/s |
| \(a\) | 前進加速度 | m/s² |
| \(\delta\) | 前輪ステア角 | rad |
| \(\beta\) | ステアバイアス（オフセット。当てはめ用） | rad |
| \(\omega\) | ヨーレート \(\dot\theta\) | rad/s |
| \(a_y\) | 横加速度（求心加速度） | m/s² |
| \(L\) | ホイールベース（wheelbase） | m |
| \(k_{us}\) | アンダーステア係数 | rad/(m/s²) ≈ s²/m |

---

## 2. 運動方程式（実シミュレータ：C++ `calcModel`）

状態 \(\big(x, y, \theta, v, \delta, a\big)\) の連続時間微分。`pedal_acc` を加速度状態 \(a\)、`pedal_acc_des` を加速度指令 \(a_{cmd}\) と読み替える。

### 縦方向（加速度の1次遅れ）

$$\dot a = -\frac{a - a_{cmd}}{\tau_{a}}, \qquad \dot v = a + a_{slope}$$

- \(\tau_a\) = `acc_time_constant`。**1つの時定数のみ**で加減速を表す（`brake_time_constant` はこのモデルでは**使用されない**。§4 参照）。
- 指令 \(a_{cmd}\) は無駄時間（dead time）\(T_a\) = `acc_time_delay` だけ遅延して入る。実装上は固定刻み \(dt\) の入力キュー（`initializeInputQueue`、長さ \(\mathrm{round}(T_a/dt)\)）で遅延を表現する。
- \(a_{slope}\) は外部入力 `SLOPE_ACCX`（路面勾配相当）。ギア状態（DRIVE/REVERSE/NEUTRAL/停止保持）で速度符号と停止処理が分岐する（geared）。
- 指令には `debug_acc_scaling_factor` が乗り、加速度・速度はそれぞれ `vel_rate_lim` / `vel_lim` で飽和する。

### 横方向（ステアの1次遅れ + キネマティック自転車）

$$\dot\delta = \mathrm{sat}\!\left(-\frac{\delta_{meas} - \delta_{cmd}}{\tau_{\delta}},\ \pm\,\dot\delta_{lim}\right)$$

$$\omega = \dot\theta = \frac{v\,\tan\delta}{L + k_{us}\, v^{2}}, \qquad \dot x = v\cos\theta,\quad \dot y = v\sin\theta, \qquad a_y = v\,\omega$$

- \(\tau_\delta\) = `steer_time_constant`、無駄時間 \(T_\delta\) = `steer_time_delay`（同じく入力キューで遅延）。
- ステア速度は **観測（measured）ステア** \(\delta_{meas}\) と指令 \(\delta_{cmd}\) の差から作る（指令には `debug_steer_scaling_factor` が乗る）。`steer_dead_band` 不感帯と `steer_rate_lim` 飽和あり。
- **アンダーステア項** \(k_{us} v^2\): \(k_{us}=0\) で理想キネマティック自転車 \(\omega = v\tan\delta / L\) に一致する。\(k_{us}>0\) では分母が速度の2乗で増大し、同じ \(\delta\) でも高速ほどヨーレートが小さく（曲がりにくく）なる＝アンダーステアを表す。
---

## 3. 運動方程式（実機当てはめ用ビューア：`lib/_model_viewer.py`）

検証ビューア（`lon_lat_model`）は、実機 rosbag の指令系列から上記モデルを前方積算し、観測値と重ねて当てはまりを見るためのもの。実シミュレータより**当てはめ自由度が多い**式を使う（出典: `_model_viewer.py` docstring）。

$$\dot a = -\frac{a - a_{target}}{\tau}, \qquad a_{target} = a_{cmd}(t - T) + \mathrm{poly}(v) + c\,(v\,\omega)^2$$

$$\dot\delta = -\frac{\delta - \delta_{cmd}(t - T_\delta)}{\tau_\delta}, \qquad \omega = \frac{v\,\tan(\delta + \beta)}{L + k_{us}\,v^2}$$

- **throttle/brake 分離**: \(a_{cmd}\ge 0\)（駆動）と \(a_{cmd}<0\)（制動）で \(T, \tau\) を別々に持つ。
- **定常オフセット** \(\mathrm{poly}(v)=p_0+p_1 v+p_2 v^2\)（転がり抵抗・勾配・空気抵抗。各次 ON/OFF・最小二乗当てはめ）。
- **縦横連成** \(c\,(v\,\omega)^2\)（カーブで縦の実減速が増える分の一方向カップリング。ON/OFF 切替）。
- **ステアバイアス** \(\beta\): ヨーレート式に \(\delta+\beta\) として入る当てはめ用の追加自由度。
- 横チェーンの \(v\) は観測 `lon_vel` を使い（縦の誤差を横テストに混入させないアイソレーション）、つまみ \(T, \tau, k_{us}, \beta\) を調整して目視で当てはめる。

---

## 4. 実装間の差（誤解防止の注記）

| 項目 | 実シミュレータ（C++ `calcModel`） | 当てはめビューア（`_model_viewer.py`） |
|---|---|---|
| ヨーレートの \(\beta\)（bias） | **含まない**。\(\omega = v\tan\delta/(L+k_{us}v^2)\) | 含む（\(\tan(\delta+\beta)\)）。当てはめ用の追加自由度 |
| 縦の時定数 | `acc_time_constant` **1つのみ**。`brake_time_constant` は未使用 | throttle/brake で \(T,\tau\) を分離 |
| 定常オフセット | 外部入力 `SLOPE_ACCX`（勾配）のみ | \(\mathrm{poly}(v)\) を当てはめ |
| 縦横連成 \(c(v\omega)^2\) | なし | あり（ON/OFF） |
| 無駄時間 | 入力キュー `initializeInputQueue`（長さ \(\mathrm{round}(T/dt)\)） | 指令系列のシフト |
| ステア速度の基準 | 観測ステア基準 \(\dot\delta = \mathrm{sat}(-(\delta_{meas}-\delta_{cmd})/\tau_\delta)\) | 同形（積分は rad） |

> **要点**: ビューアの \(\beta\)・\(\mathrm{poly}(v)\)・縦横連成・throttle/brake 分離は、実機ログへの当てはめ精度を上げるための追加項であり、**実シミュレータの運動方程式には現れない**。レポートで「モデル」と言うときどちらを指すかに注意。

---
