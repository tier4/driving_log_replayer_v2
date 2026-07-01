# 車両制御モデル（数式・座標系・定数）

---

## 1. 座標系の定義

車体基準（body frame）と進行方向基準で以下を定義する。

| 記号 | 意味 | 単位 |
|---|---|---|
| \(x, y\) | 地図平面上の車両位置 | m |
| \(\theta\) | ヨー角（車体の向き。地図 X 軸からの反時計回り回転） | rad |
| \(v_x\) | 前進速度（進行方向＝車体 X 軸成分。<code>lon_vel</code>） | m/s |
| \(a_{\mathrm{act}}\) | アクチュエータ出力（実加速度。勾配重力加算前の値） | m/s² |
| \(a_{\mathrm{slope}}\) | 路面勾配による重力加速度成分 | m/s² |
| \(\delta_{\mathrm{act}}\) | 前輪実ステア角 | rad |
| \(\beta\) | ステアバイアス（系統的操舵オフセット。<code>steer_bias</code>） | rad |
| \(\omega\) | ヨーレート \(\dot\theta\) | rad/s |
| \(a_y\) | 横加速度（求心加速度） | m/s² |
| \(L\) | ホイールベース（wheelbase） | m |
| \(k_{us}\) | アンダーステア係数 | rad/(m/s²) |

---

## 2. 運動方程式（実シミュレータ：C++ `calcModel`）

状態 \(\big(x, y, \theta, v_x, \delta_{\mathrm{act}}, a_{\mathrm{act}}\big)\) の連続時間微分。
C++の `PEDAL_ACCX` を実加速度 \(a_{\mathrm{act}}\)、`pedal_acc_des` を加速度指令 \(a_{\mathrm{cmd}}\) と読み替える。

### 縦方向（加速度の1次遅れ + 走行抵抗）

$$\dot a_{\mathrm{act}} = -\frac{a_{\mathrm{act}} - a_{\mathrm{target}}}{\tau_{a}}, \qquad a_{\mathrm{target}} = a_{\mathrm{cmd,del}} + \mathrm{poly}(v_x), \qquad \dot v_x = a_{\mathrm{act}} + a_{\mathrm{slope}}$$

- \(\tau_a\): **throttle/brake で分離**。\(a_{\mathrm{cmd}}\ge 0\) で `acc_time_constant`、\(a_{\mathrm{cmd}}<0\) で `brake_time_constant`（\(\le 0\) のとき `acc_time_constant` にフォールバック＝単一時定数）。
- **走行抵抗** \(\mathrm{poly}(v_x)=\) `lon_drag_c0` \(+\) `lon_drag_c1`\(\,v_x +\) `lon_drag_c2`\(\,v_x^2\)（既定 0＝無効）。転がり抵抗・空気抵抗を加速度ターゲットに加える。
- 指令 \(a_{\mathrm{cmd}}\) は無駄時間（dead time）\(T_a\) = `acc_time_delay` だけ遅延して入る \(a_{\mathrm{cmd,del}}(t) = a_{\mathrm{cmd}}(t - T_a) \cdot K_{\mathrm{acc\_scale}}\)（\(K_{\mathrm{acc\_scale}}\) は `debug_acc_scaling_factor`）。
- \(a_{\mathrm{slope}}\) は外部入力 `SLOPE_ACCX`（路面勾配による重力加速度成分）。ギア状態（DRIVE/REVERSE/NEUTRAL/停止保持）で速度符号と停止処理が分岐する（geared）。
- ※ 目標加速度および実加速度は `vel_rate_lim`、車速は `vel_lim` で制限されます。

### 横方向（ステアの1次遅れ + キネマティック自転車 + ヨーバイアス）

$$\dot\delta_{\mathrm{act}} = -\frac{\delta_{\mathrm{act}} - \delta_{\mathrm{des}}}{\tau_{\delta}}$$

$$\omega = \dot\theta = \frac{v_x\,\tan(\delta_{\mathrm{act}} + \beta)}{L + k_{us}\, v_x^{2}}, \qquad \dot x = v_x\cos\theta,\quad \dot y = v_x\sin\theta, \qquad a_y = v_x\,\omega$$

- \(\tau_\delta\) = `steer_time_constant`、無駄時間 \(T_\delta\) = `steer_time_delay`（遅延後の指令にゲインを掛けて目標操舵角とする： \(\delta_{\mathrm{des}} = K_{\mathrm{steer\_scale}} \cdot \delta_{\mathrm{cmd}}(t - T_{\delta})\)、\(K_{\mathrm{steer\_scale}}\) は `debug_steer_scaling_factor`）。
- **ヨーバイアス** \(\beta\) = `steer_bias`: 実際のステア角ではなく、運動学式のヨーレート計算時に \(\tan(\delta_{\mathrm{act}}+\beta)\) として加算される。
- **アンダーステア項** \(k_{us} v_x^2\): \(k_{us}=0\) かつ \(\beta=0\) で理想キネマティック自転車 \(\omega = v_x\tan\delta / L\) に一致する。\(k_{us}>0\) では分母が速度の2乗で増大し、同じ \(\delta_{\mathrm{act}}\) でも高速ほどヨーレートが小さく（曲がりにくく）なる＝アンダーステアを表す。
- ※ 目標操舵角は `steer_lim`、操舵速度は `steer_rate_lim` で制限され、不感帯 `steer_dead_band` が適用されます。

---

## 3. 運動方程式（実機当てはめ用ビューア：`lib/_model_viewer.py`）

検証ビューア（`lon_lat_model`）は、実機 rosbag の指令系列から上記モデルを前方積算し、観測値と重ねて当てはまりを見るためのもの。実シミュレータより**当てはめ自由度が多い**式を使う。

$$\dot a_{\mathrm{act}} = -\frac{a_{\mathrm{act}} - a_{\mathrm{target}}}{\tau}, \qquad a_{\mathrm{target}} = a_{\mathrm{cmd}}(t - T) + \mathrm{poly}(v_x)$$

$$\dot\delta_{\mathrm{act}} = -\frac{\delta_{\mathrm{act}} - \delta_{\mathrm{cmd}}(t - T_\delta)}{\tau_\delta}, \qquad \omega = \frac{v_x\,\tan(\delta_{\mathrm{act}} + \beta)}{L + k_{us}\,v_x^2}$$

- **throttle/brake 分離**: \(a_{\mathrm{cmd}}\ge 0\)（駆動）と \(a_{\mathrm{cmd}}<0\)（制動）で \(T, \tau\) を別々に持つ。
- **定常オフセット** \(\mathrm{poly}(v_x)=p_0+p_1 v_x+p_2 v_x^2\)（転がり抵抗・勾配・空気抵抗。各次 ON/OFF・最小二乗当てはめ）。
- **ステアバイアス** \(\beta\): ヨーレート式に \(\delta_{\mathrm{act}}+\beta\) として入る当てはめ用の追加自由度。
- 横チェーンの速度には観測 `lon_vel` を使い（縦の誤差を横テストに混入させないアイソレーション）、つまみ \(T, \tau, k_{us}, \beta\) を調整して目視で当てはめる。

---

## 4. 実装間の差と運用注記

2026-06 に C++ `calcModel`（§2）を検証ビューア `lon_lat_model`（§3）と同じ運動方程式に揃えた
（β・throttle/brake 分離 τ・poly(v)。係数 0 で従来の単一 τ・キネマティック自転車に厳密一致）。
**残る差は無駄時間 T のみ**: C++ は入力キューが単一で τ だけ throttle/brake 分離（T は共通）、ビューアは
\(T_{thr}/T_{brk}\) も分離する。

- **配線**: 新パラメータ（`brake_time_constant` / `lon_drag_c0/c1/c2`）は
  scenario.yaml `models.<name>.params` に書けば C++ `getParameter`（`ego_entity_simulation.cpp`）まで届く
  （open-loop rollout 用 ctypes ラッパーも同期済）。
- **同定**: 縦の新項は `tools/fit_lon_model.py`（または `_model_viewer.py` のドロップダウンで対話）で実機に
  フィット、`k_us` のみ rollout sweep。**open-loop の当てはめ最適は closed-loop 精度に転写されないことが
  ある**ため、採否は closed-loop 再現で検証する。

---
