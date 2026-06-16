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

### 縦方向（加速度の1次遅れ + 走行抵抗 + 縦横連成）

$$\dot a = -\frac{a - a_{target}}{\tau_{a}}, \qquad a_{target} = a_{cmd} + \mathrm{poly}(v) + c\,(v\,\omega)^2, \qquad \dot v = a + a_{slope}$$

- \(\tau_a\): **throttle/brake で分離**。\(a_{cmd}\ge 0\) で `acc_time_constant`、\(a_{cmd}<0\) で `brake_time_constant`（\(\le 0\) のとき `acc_time_constant` にフォールバック＝単一時定数）。2026-06 に検証ビューア相当へ拡張（§4）。
- **走行抵抗** \(\mathrm{poly}(v)=\) `lon_drag_c0` \(+\) `lon_drag_c1`\(\,v +\) `lon_drag_c2`\(\,v^2\)（既定 0＝無効）。転がり抵抗・空気抵抗を加速度ターゲットに加える。
- **縦横連成** \(c\,(v\,\omega)^2\)（係数 `lon_lat_coupling`、既定 0）。カーブ（求心加速度 \(a_y=v\omega\)）で縦の減速を表す（係数を負にする）。
- 指令 \(a_{cmd}\) は無駄時間（dead time）\(T_a\) = `acc_time_delay` だけ遅延して入る（固定刻み \(dt\) の入力キュー `initializeInputQueue`、長さ \(\mathrm{round}(T_a/dt)\)）。throttle/brake の \(T\) 分離はキューが単一のため未実装（τ のみ分離）。
- \(a_{slope}\) は外部入力 `SLOPE_ACCX`（路面勾配相当）。ギア状態（DRIVE/REVERSE/NEUTRAL/停止保持）で速度符号と停止処理が分岐する（geared）。
- 指令には `debug_acc_scaling_factor` が乗り、加速度・速度はそれぞれ `vel_rate_lim` / `vel_lim` で飽和する。

### 横方向（ステアの1次遅れ + キネマティック自転車 + ヨーバイアス）

$$\dot\delta = \mathrm{sat}\!\left(-\frac{\delta_{actual} - \delta_{cmd}}{\tau_{\delta}},\ \pm\,\dot\delta_{lim}\right)$$

$$\omega = \dot\theta = \frac{v\,\tan(\delta + \beta)}{L + k_{us}\, v^{2}}, \qquad \dot x = v\cos\theta,\quad \dot y = v\sin\theta, \qquad a_y = v\,\omega$$

- \(\tau_\delta\) = `steer_time_constant`、無駄時間 \(T_\delta\) = `steer_time_delay`（同じく入力キューで遅延）。
- ステア追従は **実ステア** \(\delta_{actual}\) と指令 \(\delta_{cmd}\) の差から作る（指令には `debug_steer_scaling_factor` が乗る）。`steer_dead_band` 不感帯と `steer_rate_lim` 飽和あり。
- **ヨーバイアス** \(\beta\) = `steer_bias`: yaw 式に \(\tan(\delta+\beta)\) として入る（検証ビューアの β と一致）。2026-06 拡張前は measured ステア側にのみ効きヨーには入らなかったが、追従ループで相殺されない正味のヨーオフセットを表すため yaw 式へ移した（§4）。\(\beta=0\) で従来挙動。
- **アンダーステア項** \(k_{us} v^2\): \(k_{us}=0\) かつ \(\beta=0\) で理想キネマティック自転車 \(\omega = v\tan\delta / L\) に一致する。\(k_{us}>0\) では分母が速度の2乗で増大し、同じ \(\delta\) でも高速ほどヨーレートが小さく（曲がりにくく）なる＝アンダーステアを表す。
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

## 4. 実装間の差（2026-06 に C++ をビューア相当へ拡張）

`DELAY_STEER_ACC_GEARED_WO_FALL_GUARD` の C++ `calcModel` を検証ビューア（`_model_viewer.py`
`lon_lat_model`）の表現力に揃えた。下表の項目は**両者で同形**になった（係数が 0 のとき従来の
単一 τ・キネマティック自転車に厳密一致するので既存セットアップは挙動不変）。

| 項目 | 実シミュレータ（C++ `calcModel`） | 当てはめビューア（`_model_viewer.py`） |
|---|---|---|
| ヨーレートの \(\beta\)（bias） | **含む**（`steer_bias` → \(\tan(\delta+\beta)\)）。measured 側からヨー式へ移設 | 含む（\(\tan(\delta+\beta)\)） |
| 縦の時定数 | throttle=`acc_time_constant` / brake=`brake_time_constant` で**分離**（\(\le0\) で単一 τ） | throttle/brake で \(T,\tau\) 分離 |
| 定常オフセット poly(v) | `lon_drag_c0/c1/c2` を \(a_{target}\) に加算 | \(\mathrm{poly}(v)\) を当てはめ |
| 縦横連成 \(c(v\omega)^2\) | `lon_lat_coupling` を \(a_{target}\) に加算 | あり（ON/OFF） |
| 無駄時間 | 入力キュー（単一）。throttle/brake の **\(T\) 分離は未実装**（τ のみ分離） | 指令系列のシフト（throttle/brake で \(T\) も分離） |
| ステア追従の基準 | 実ステア基準 \(\dot\delta = \mathrm{sat}(-(\delta_{actual}-\delta_{cmd})/\tau_\delta)\)（β は追従に入れない） | 同形 |

**配線**: 新パラメータ（`brake_time_constant` / `lon_drag_c0/c1/c2` / `lon_lat_coupling`）は
scenario.yaml `models.<name>.params` に書けば step3→launch→C++ `getParameter`（`ego_entity_simulation.cpp`）
へ届く（KNOWN_PARAM_KEYS 登録済）。open-loop N-step rollout 用 ctypes ラッパー
（`vehicle_model_c_wrapper.cpp` の `vm_create_*`、`step5_analyze_nstep.py` の argtypes）も同期済。

**同定方針**: 縦の新項（brake τ・poly・coupling）は `_model_viewer.py` の lon_lat_model を実機ログへ
LS フィットして得る（`tools/fit_lon_model.py` がプログラム再現。ビューアのドロップダウンで
レジストリモデルを選んで対話当てはめも可）。`k_us` のみ rollout sweep で同定する（他パラメータの
sweep は過適合のため不可とする方針）。**注意**: open-loop の accel-fit 最適は closed-loop 精度に
そのまま転写されないことがある（poly の符号など）。採否は closed-loop 再現で検証すること。

> **要点**: 係数が 0 のとき新項は消えて従来挙動に厳密一致する。レポートで「モデル」と言うとき
> closed-loop sim と検証ビューアは（係数を合わせれば）同じ運動方程式を指す。

---
