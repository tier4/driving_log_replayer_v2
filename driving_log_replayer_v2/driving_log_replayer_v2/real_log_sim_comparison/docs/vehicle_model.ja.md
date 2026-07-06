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

### 縦方向（加速度の1次遅れ）

$$\dot a_{\mathrm{act}} = -\frac{a_{\mathrm{act}} - a_{\mathrm{target}}}{\tau_{a}}, \qquad a_{\mathrm{target}} = a_{\mathrm{cmd,del}}, \qquad \dot v_x = a_{\mathrm{act}} + a_{\mathrm{slope}}$$

- \(\tau_a\): 加速度の1次遅れ時定数 `acc_time_constant`（加減速で共通の単一時定数）。
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

### 遅延モード：指令のみ遅延（旧）／ full-RHS 遅延（新）

無駄時間の掛け方には 2 通りがあり、`vehicle_model_type` で選択する。

- **指令のみ遅延（`DELAY_STEER_ACC_GEARED_WO_FALL_GUARD`、既定）**：無駄時間は**指令**にのみ掛かる。縦の \(\dot a_{\mathrm{act}}\) は現在の \(a_{\mathrm{act}}\)・現在の \(v_x\) に対して遅延指令 \(a_{\mathrm{cmd,del}}\) を、横の \(\dot\delta_{\mathrm{act}}\) は現在の \(\delta_{\mathrm{act}}\) に対して遅延指令 \(\delta_{\mathrm{des}}\) を用いる（上式のとおり）。
- **full-RHS 遅延（`DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER`、新）**：ステア・加速度チャネルの**右辺全体**を \(t-T\) で評価する。すなわち状態フィードバックも遅延させる：

$$\dot a_{\mathrm{act}}(t) = -\frac{a_{\mathrm{act}}(t-T_a) - a_{\mathrm{cmd,del}}}{\tau_a}, \qquad \dot\delta_{\mathrm{act}}(t) = -\frac{\delta_{\mathrm{act}}(t-T_\delta) - \delta_{\mathrm{des}}}{\tau_\delta}$$

  - 加速度チャネルは状態 \(a_{\mathrm{act}}\) を \(t-T_a\) で評価する（C++ の `pedal_delayed` に対応）。
  - \(a_{\mathrm{slope}}\)（`SLOPE_ACCX`）は**遅延させない**（C++ では速度式 \(\dot v_x\) 側の外部入力であり無駄時間対象外）。
  - ヨー式・位置式・速度式 \(\dot v_x = a_{\mathrm{act}} + a_{\mathrm{slope}}\) は現在状態で評価する（ヨー観測遅延 \(d_{tt}\) はこのモデルでは対象外＝将来拡張）。
  - 全遅延 \(=0\) のとき指令のみ遅延と数値的に一致する（＝旧モデルの厳密な一般化）。

統合ビューア（`viewer.html` のモデル検証タブ）は両モードに対応する。トグル **「full-RHS遅延」** で切り替え（既定は `vehicle_model_type` 由来）。ビューアは連続時間の線形補間で状態履歴を参照するため、離散キューの C++ 実装とは定式化が一致する一方で数値の完全一致（bit 一致）はしない。

---

## 3. チューニングおよび検証の運用方針

本レポートで提示される評価値は、上記の運動方程式をベースにして以下の二段階で同定・検証されたものです。

1. **パラメータの最適化 (オープンループ同定)**
   - 縦方向パラメータ（\(\tau_a\), 遅延 \(T_a\)）は、実機走行ログの加速度・速度・指令値から最小二乗法を用いて最適値を直接同定します。
   - 横方向パラメータ（ステアバイアス \(\beta\), アンダーステア係数 \(k_{us}\)）は、理想追従予測（Bicycle Model / Perfect Tracking）をもとに探索・同定します。
2. **クローズドループ再現検証 (実シミュレータ走行)**
   - 同定された最適パラメータ群を `scenario.yaml` の `models` パラメータに設定し、実機と同じ経路・車速指示を与えてクローズドループで走行シミュレーション（rollout）を実行します。
   - オープンループでの最小誤差パラメータが、フィードバック制御や動的干渉を伴うクローズドループ精度に必ずしもそのまま直結しないため、最終的にはクローズドループ走行時の「実機軌跡からの平均乖離」および「完走率」をもって検証の合格基準とします。
