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

## 2. 運動方程式（実シミュレータ C++ `calcModel` ＆ オフライン評価・ビューア共通）

状態 \(\big(x, y, \theta, v_x, \delta_{\mathrm{act}}, a_{\mathrm{act}}\big)\) の連続時間微分。
C++の `PEDAL_ACCX` を実加速度 \(a_{\mathrm{act}}\)、`pedal_acc_des` を加速度指令 \(a_{\mathrm{cmd}}\) と読み替える。
統合ビューア（`viewer.html` のモデル検証タブ）や各種同定・評価スクリプトも、実シミュレータと完全に同期した同一の運動方程式を使用します。

### 縦方向（加速度の1次遅れ + 走行抵抗）

$$\dot a_{\mathrm{act}} = -\frac{a_{\mathrm{act}} - a_{\mathrm{target}}}{\tau_{a}}, \qquad a_{\mathrm{target}} = a_{\mathrm{cmd,del}} + \mathrm{poly}(v_x), \qquad \dot v_x = a_{\mathrm{act}} + a_{\mathrm{slope}}$$

- \(\tau_a\): **throttle/brake で分離**。\(a_{\mathrm{cmd}}\ge 0\) で `acc_time_constant`、\(a_{\mathrm{cmd}}<0\) で `brake_time_constant`（\(\le 0\) のとき `acc_time_constant` にフォールバック＝単一時定数）。
- **走行抵抗** \(\mathrm{poly}(v_x)=\) `lon_drag_c0` \(+\) `lon_drag_c1`\(\,v_x +\) `lon_drag_c2`\(\,v_x^2\)。転がり抵抗・空気抵抗を加速度ターゲットに加える。
- 指令 \(a_{\mathrm{cmd}}\) は無駄時間（dead time）\(T_a\) = `acc_time_delay` だけ遅延して入る \(a_{\mathrm{cmd,del}}(t) = a_{\mathrm{cmd}}(t - T_a) \cdot K_{\mathrm{acc\_scale}}\)（\(K_{\mathrm{acc\_scale}}\) は `debug_acc_scaling_factor`）。
- \(a_{\mathrm{slope}}\) は外部入力 `SLOPE_ACCX`（路面勾配による重力加速度成分）。ギア状態（DRIVE/REVERSE/NEUTRAL/停止保持）で速度符号と停止処理が分岐する（geared）。
- ※ 目標加速度および実加速度は `vel_rate_lim`、車速は `vel_lim` で制限されます。

### 横方向（ステアの1次遅れ + キネマティック自転車 + ヨーバイアス）

$$\dot\delta_{\mathrm{act}} = -\frac{\delta_{\mathrm{act}} - \delta_{\mathrm{des}}}{\tau_{\delta}}$$

$$\omega = \dot\theta = \frac{v_x\,\tan(\delta_{\mathrm{act}} + \beta)}{L + k_{us}\, v_x^{2}}, \qquad \dot x = v_x\cos\theta,\quad \dot y = v_x\sin\theta, \qquad a_y = v_x\,\omega$$

- \(\tau_\delta\) = `steer_time_constant`、無駄時間 \(T_\delta\) = `steer_time_delay`（遅延後の指令にゲインを掛けて目標操舵角とする： \(\delta_{\mathrm{des}} = K_{\mathrm{steer\_scale}} \cdot \delta_{\mathrm{cmd}}(t - T_{\delta})\)、\(K_{\mathrm{steer\_scale}}\) は `debug_steer_scaling_factor`）。
- **ヨーバイアス** \(\beta\) = `steer_bias`: 実際のステア角ではなく、運動学式のヨーレート計算時に \(\tan(\delta_{\mathrm{act}}+\beta)\) として加算される。
- **アンダーステア項** \(k_{us} v_x^2\): \(k_{us}=0\) かつ \(\beta=0\) で理想キネマティック自転車 \(\omega = v_x\tan\delta / L\) に一致する。\(k_{us}>0\) では分母が速度의2乗で増大し、同じ \(\delta_{\mathrm{act}}\) でも高速ほどヨーレートが小さく（曲がりにくく）なる＝アンダーステアを表す。
- ※ 目標操舵角は `steer_lim`、操舵速度は `steer_rate_lim` で制限され、不感帯 `steer_dead_band` が適用されます。

---

## 3. チューニングおよび検証の運用方針

本レポートで提示される評価値は、上記の運動方程式をベースにして以下の二段階で同定・検証されたものです。

1. **パラメータの最適化 (オープンループ同定)**
   - 縦方向パラメータ（\(\tau_a\), 走行抵抗多項式, 遅延 \(T_a\)）は、実機走行ログの加速度・速度・指令値から最小二乗法を用いて最適値を直接同定します。
   - 横方向パラメータ（ステアバイアス \(\beta\), アンダーステア係数 \(k_{us}\)）は、理想追従予測（Bicycle Model / Perfect Tracking）をもとに探索・同定します。
2. **クローズドループ再現検証 (実シミュレータ走行)**
   - 同定された最適パラメータ群を `scenario.yaml` の `models` パラメータに設定し、実機と同じ経路・車速指示を与えてクローズドループで走行シミュレーション（rollout）を実行します。
   - オープンループでの最小誤差パラメータが、フィードバック制御や動的干渉を伴うクローズドループ精度に必ずしもそのまま直結しないため、最終的にはクローズドループ走行時の「実機軌跡からの平均乖離」および「完走率」をもって検証の合格基準とします。
