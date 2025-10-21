# 旋回誘導理論と実装

## 目次

1. [概要](#概要)
2. [問題の定式化](#問題の定式化)
3. [誘導則の導出](#誘導則の導出)
4. [Lyapunov安定性解析](#lyapunov安定性解析)
5. [実装の詳細](#実装の詳細)
6. [GPS測定の統合](#gps測定の統合)
7. [性能解析](#性能解析)
8. [実用上の考察](#実用上の考察)

## 概要

### 目的

旋回誘導アルゴリズムは、固定翼UAVが指定された中心点の周りを希望する半径で円形の飛行経路を維持することを可能にします。これは以下のような基本的な能力です：

- **特定エリアの持続的監視**
- **特定位置上での通信中継**
- **指令待機中の待機飛行**
- **関心エリア周辺でのセンサーデータ収集**

### 主な課題

固定翼航空機はホバリングできないため、円形軌道を維持するには以下が必要です：

1. **十分な対気速度での継続的な前進運動**
2. **方位角変更のための協調旋回**
3. **希望軌道からの逸脱を修正するフィードバック制御**
4. **風、測定誤差、外乱に対するロバスト性**

## 問題の定式化

### 座標系

**NED（北-東-下）座標系**を使用します：

- **北 (North)**: 正の方向が北を指す
- **東 (East)**: 正の方向が東を指す
- **下 (Down)**: 正の方向が下を指す（高度は負の値）

### 変数

**与えられるもの：**
- $\mathbf{c} = [c_n, c_e, c_d]^T$: NED座標系における旋回中心位置
- $R$: 希望する旋回半径
- $\lambda$: 旋回方向（+1で反時計回り、-1で時計回り）

**状態：**
- $\mathbf{p} = [p_n, p_e, p_d]^T$: NED座標系における機体位置
- $\psi$: 機体の方位角（ヨー角）
- $V_a$: 対気速度

**制御：**
- $\psi_c$: 指令方位角

### 目標

以下を満たす $\psi_c(t)$ を計算する誘導則を設計する：

$$\lim_{t \to \infty} \|\mathbf{p}(t) - \mathbf{c}\| = R$$

同時に以下を維持：
- 方位角指令の連続性（不連続なし）
- 任意の初期位置からの収束
- 測定ノイズに対するロバスト性

## 誘導則の導出

### ベクトル場アプローチ

旋回誘導則は、空間の各点で希望する速度方向を定義する**ベクトル場**に基づいています。

#### ステップ1: 位置誤差の定義

$\mathbf{e} = \mathbf{p} - \mathbf{c}$ を旋回中心から機体への位置誤差ベクトルとする。

2次元で（誘導のために高度を無視）：

$$\mathbf{e} = \begin{bmatrix} e_n \\ e_e \end{bmatrix} = \begin{bmatrix} p_n - c_n \\ p_e - c_e \end{bmatrix}$$

#### ステップ2: 距離と半径誤差の計算

旋回中心からの現在の距離：

$$d = \|\mathbf{e}\| = \sqrt{e_n^2 + e_e^2}$$

半径誤差（軌道の外側にいる場合は正）：

$$e_r = d - R$$

#### ステップ3: 希望する速度方向の定義

完全な軌道では、速度は円に対して**接線方向**である必要があります。接線方向は径方向に対して垂直です。

単位径方向ベクトル（中心から機体を指す）：

$$\hat{\mathbf{r}} = \frac{\mathbf{e}}{d} = \begin{bmatrix} e_n/d \\ e_e/d \end{bmatrix}$$

単位接線ベクトル（径方向に垂直、旋回方向）：

**時計回り(CW)旋回**の場合 ($\lambda = -1$)：

$$\hat{\mathbf{t}}_{CW} = \begin{bmatrix} e_e/d \\ -e_n/d \end{bmatrix}$$

**反時計回り(CCW)旋回**の場合 ($\lambda = +1$)：

$$\hat{\mathbf{t}}_{CCW} = \begin{bmatrix} -e_e/d \\ e_n/d \end{bmatrix}$$

一般形：

$$\hat{\mathbf{t}} = \lambda \begin{bmatrix} -e_e/d \\ e_n/d \end{bmatrix}$$

#### ステップ4: 収束のためのフィードバック項の追加

任意の開始位置から軌道へ収束させるために、**径方向フィードバック項**を追加します：

$$\mathbf{v}_d = \hat{\mathbf{t}} - k_{orbit} \cdot e_r \cdot \hat{\mathbf{r}}$$

ここで：
- $\hat{\mathbf{t}}$: 接線成分（軌道を維持）
- $k_{orbit} \cdot e_r \cdot \hat{\mathbf{r}}$: 径方向成分（半径誤差を修正）
- $k_{orbit}$: 誘導ゲイン（収束速度を決定）

**物理的解釈：**
- $e_r > 0$（軌道の外側）のとき：径方向項が内向き → 機体は軌道に向かって移動
- $e_r < 0$（軌道の内側）のとき：径方向項が外向き → 機体は中心から離れる
- $e_r = 0$（軌道上）のとき：接線成分のみ残る → 純粋な円運動

#### ステップ5: 指令方位角の計算

希望する速度方向は：

$$\psi_c = \text{atan2}(v_{d,e}, v_{d,n})$$

ここで $v_{d,n}$ と $v_{d,e}$ は $\mathbf{v}_d$ の北と東の成分です。

### 完全な誘導則

すべてのステップを組み合わせると、旋回誘導則は：

**時計回り(CW)旋回の場合：**

$$\psi_c = \text{atan2}(e_n, e_e) + \frac{\pi}{2} - \text{atan}(k_{orbit} \cdot e_r)$$

**反時計回り(CCW)旋回の場合：**

$$\psi_c = \text{atan2}(e_n, e_e) - \frac{\pi}{2} + \text{atan}(k_{orbit} \cdot e_r)$$

ここで：
- $\text{atan2}(e_n, e_e)$: 旋回中心から機体への角度
- $\pm \pi/2$: 接線方向（径方向に対して垂直）
- $\text{atan}(k_{orbit} \cdot e_r)$: 径方向修正項

## Lyapunov安定性解析

### Lyapunov関数

収束を証明するために、半径誤差に基づくLyapunov関数を定義します：

$$V(e_r) = \frac{1}{2} e_r^2$$

これは希望する軌道から外れることに関連する「エネルギー」を表します。

### 収束の証明

時間微分を取ると：

$$\dot{V} = e_r \cdot \dot{e_r}$$

半径誤差の変化率は：

$$\dot{e_r} = \dot{d} = \frac{d}{dt}\|\mathbf{e}\| = \frac{\mathbf{e}^T \dot{\mathbf{e}}}{\|\mathbf{e}\|}$$

$\dot{\mathbf{e}} = \dot{\mathbf{p}} = V_a \mathbf{v}_d$ であるため（$\psi_c$ の完全な追従を仮定）：

$$\dot{e_r} = \frac{\mathbf{e}^T}{d} V_a \mathbf{v}_d = V_a \hat{\mathbf{r}}^T \mathbf{v}_d$$

誘導則 $\mathbf{v}_d = \hat{\mathbf{t}} - k_{orbit} e_r \hat{\mathbf{r}}$ を代入：

$$\dot{e_r} = V_a \hat{\mathbf{r}}^T (\hat{\mathbf{t}} - k_{orbit} e_r \hat{\mathbf{r}})$$

$\hat{\mathbf{r}} \perp \hat{\mathbf{t}}$（垂直）であるため、$\hat{\mathbf{r}}^T \hat{\mathbf{t}} = 0$：

$$\dot{e_r} = -V_a k_{orbit} e_r$$

したがって：

$$\dot{V} = e_r \dot{e_r} = -V_a k_{orbit} e_r^2$$

**結論：**
- すべての $e_r \neq 0$ に対して $\dot{V} < 0$（$k_{orbit} > 0$ かつ $V_a > 0$ の場合）
- $e_r = 0$（軌道上）のときのみ $\dot{V} = 0$

これは**漸近安定性**を証明します：機体は任意の初期位置から希望する軌道に収束します。

### 収束速度

半径誤差は指数関数的に減衰します：

$$e_r(t) = e_r(0) \exp(-V_a k_{orbit} t)$$

**時定数**は：

$$\tau = \frac{1}{V_a k_{orbit}}$$

**設計指針：**
- $k_{orbit}$ が大きい → 速い収束
- 対気速度 $V_a$ が高い → 速い収束
- 典型的な値：$k_{orbit} \in [1, 5]$

## 実装の詳細

### Python実装

```python
class OrbitGuidance:
    def __init__(self, center, radius, direction='CW'):
        self.center = np.array(center)
        self.radius = radius
        self.direction = 1 if direction == 'CCW' else -1  # λ

    def compute_heading_command(self, position, k_orbit=1.0):
        # 位置誤差（2次元、高度を無視）
        e_n = position[0] - self.center[0]
        e_e = position[1] - self.center[1]

        # 中心からの距離
        d = np.sqrt(e_n**2 + e_e**2)

        if d < 1e-6:  # ゼロ除算を避ける
            return 0.0

        # 半径誤差
        e_r = d - self.radius

        # 中心から機体への角度
        angle_to_aircraft = np.arctan2(e_e, e_n)

        # 指令方位角
        if self.direction == -1:  # 時計回り
            psi_c = angle_to_aircraft + np.pi/2 - np.arctan(k_orbit * e_r)
        else:  # 反時計回り
            psi_c = angle_to_aircraft - np.pi/2 + np.arctan(k_orbit * e_r)

        # [-π, π]にラップ
        psi_c = np.arctan2(np.sin(psi_c), np.cos(psi_c))

        return psi_c
```

### 数値的考慮事項

**特異点の回避：**

$d \approx 0$（機体が旋回中心に非常に近い）のとき、$d$ による除算が数値的に不安定になります。小さな閾値を追加します：

```python
if d < 1e-6:
    return 0.0  # または最後の有効な方位角
```

**角度のラッピング：**

方位角は $[-\pi, \pi]$ にラップして不連続を避ける必要があります：

```python
psi_c = np.arctan2(np.sin(psi_c), np.cos(psi_c))
```

**ゲインの飽和：**

非常に大きな誘導ゲインはチャタリングを引き起こす可能性があります。通常、制限します：

```python
k_orbit_effective = min(k_orbit, 10.0)
```

## GPS測定の統合

### 問題：ノイズのある旋回中心

実際には、旋回中心 $\mathbf{c}$ は誤差を含むGPSで測定される可能性があります：

$$\mathbf{c}_{meas} = \mathbf{c}_{true} + \mathbf{n}$$

ここで $\mathbf{n} \sim \mathcal{N}(0, \sigma_{GPS}^2)$ はGPSノイズです。

### GPS誤差モデル

GPSセンサーモデルには以下が含まれます：

1. **ガウスノイズ**: $\sigma_h \approx 2.5$ m（水平）、$\sigma_v \approx 4$ m（垂直）
2. **低周波ドリフト**: 時定数 $\tau_d \approx 15$ s の $\mathbf{d}(t)$
3. **外れ値**: 確率 $p_{outlier} \approx 0.2\%$、大きさ $\approx 25$ m

合計測定：

$$\mathbf{c}_{meas}(t) = \mathbf{c}_{true} + \mathbf{n}(t) + \mathbf{d}(t) + \mathbf{o}(t)$$

### 旋回中心推定のためのカルマンフィルタ

GPSノイズを低減するために、**カルマンフィルタ**を使用します：

**状態モデル**（旋回中心は静止と仮定）：

$$\mathbf{c}_{k+1} = \mathbf{c}_k + \mathbf{w}_k$$

ここで $\mathbf{w}_k \sim \mathcal{N}(0, Q)$ はプロセスノイズ（中心が静止しているため小さい）。

**測定モデル：**

$$\mathbf{z}_k = \mathbf{c}_k + \mathbf{v}_k$$

ここで $\mathbf{v}_k \sim \mathcal{N}(0, R)$ はGPS測定ノイズです。

**カルマンフィルタ方程式：**

**予測：**
$$\hat{\mathbf{c}}_{k|k-1} = \hat{\mathbf{c}}_{k-1|k-1}$$
$$P_{k|k-1} = P_{k-1|k-1} + Q$$

**更新：**
$$K_k = P_{k|k-1}(P_{k|k-1} + R)^{-1}$$
$$\hat{\mathbf{c}}_{k|k} = \hat{\mathbf{c}}_{k|k-1} + K_k(\mathbf{z}_k - \hat{\mathbf{c}}_{k|k-1})$$
$$P_{k|k} = (I - K_k)P_{k|k-1}$$

**パラメータ調整：**
- $Q \approx 0.01$（旋回中心は非常にゆっくり変化）
- $R \approx 6.25$（2.5m精度のGPSに対する $\sigma_{GPS}^2$）

### 外れ値の検出と除去

**統計的外れ値検出器**を使用します：

1. 最近の測定の移動窓を維持
2. 平均 $\mu$ と標準偏差 $\sigma$ を計算
3. 以下の場合、測定を除去：$|\mathbf{z}_k - \mu| > 3\sigma$

これにより、大きなGPS誤差が旋回中心推定を破損するのを防ぎます。

### 完全なGPSロバスト誘導システム

```
GPS → 外れ値検出器 → カルマンフィルタ → 旋回誘導 → 方位角指令
```

**アルゴリズム：**

```python
# GPS更新ごとに
if gps_updated:
    # 外れ値を検出
    if not outlier_detector.is_outlier(gps_measurement):
        # カルマンフィルタを更新
        orbit_center_estimated = kalman_filter.update(gps_measurement)

        # フィルタリングされた推定値で誘導を更新
        orbit_guidance.center = orbit_center_estimated

# 制御ループごとに（GPSより高速）
psi_c = orbit_guidance.compute_heading_command(position, k_orbit)
```

## 性能解析

### 理論性能

**GPS誤差なしの場合：**
- 定常状態半径誤差：$< 1$ m（制御精度により制限）
- 収束時間：典型的なパラメータで $\tau = 1/(V_a k_{orbit}) \approx 1-5$ s

**GPS誤差ありの場合（$\sigma_{GPS} = 2.5$ m）：**
- 旋回中心推定誤差（RMS）：$\approx 1.5$ m（カルマンフィルタリング後）
- 誘導される半径誤差（RMS）：$\approx 2$ m
- 外れ値除去率：$> 99\%$

### シミュレーション結果

`orbit_flight_with_gps.py` シミュレーションから：

| 指標 | 値 |
|------|-----|
| GPS測定RMS誤差 | 2.8 m |
| カルマンフィルタRMS誤差 | 1.4 m |
| 誤差低減 | 50% |
| 旋回半径RMS誤差 | 3.2 m |
| GPS更新レート | 5 Hz |
| 検出された外れ値 | ~0.2% |

### ロバスト性解析

**誘導ゲイン $k_{orbit}$ に対する感度：**

| $k_{orbit}$ | 収束 | ロバスト性 | 備考 |
|-------------|------|-----------|------|
| 0.5 | 遅い | 高い | 滑らかだが収束が遅い |
| 1.0 | 中程度 | 高い | 良いバランス |
| 2.5 | 速い | 中程度 | 速い収束 |
| 5.0 | 非常に速い | 低い | GPSノイズで振動の可能性 |

**推奨：** GPSベースのシステムには $k_{orbit} \in [1, 3]$

## 実用上の考察

### 風の影響

風は旋回に**ドリフト**を引き起こします。機体は空気塊に対して円を飛行し、それが風とともに移動します。

**緩和策：**
- GPS位置フィードバックを使用（実装済み）
- 風の多い条件では誘導ゲインをわずかに増加
- フィードフォワード補償のための風推定を検討

### 協調旋回

方位角指令 $\psi_c$ は協調旋回のための**ロール角指令** $\phi_c$ に変換する必要があります：

$$\phi_c = \arctan\left(\frac{V_a^2 (\psi_c - \psi)k_\psi}{g}\right)$$

ここで $k_\psi$ は方位角追従ゲインです。

### 高度制御

高度は独立して制御されます：

$$h_c = c_d$$

旋回誘導は主に水平面で動作します。

### 対気速度管理

一定の対気速度を維持：

$$V_{a,c} = \text{一定}$$

最小旋回半径は以下によって制約されます：

$$R_{min} = \frac{V_a^2}{g \tan(\phi_{max})}$$

小型UAV（$V_a = 15$ m/s、$\phi_{max} = 45°$）の場合：

$$R_{min} \approx 23 \text{ m}$$

中型UAV（$V_a = 25$ m/s、$\phi_{max} = 45°$）の場合：

$$R_{min} \approx 64 \text{ m}$$

### 初期化

スムーズな開始のために：

1. **外側から軌道に接近**
2. **徐々に増加**させる誘導ゲインを0から公称値まで
3. **半径誤差を監視**して収束を確認

## 数学的まとめ

### 主要方程式

**旋回誘導則（一般形）：**

$$\psi_c = \text{atan2}(e_e, e_n) + \lambda \frac{\pi}{2} - \lambda \cdot \text{atan}(k_{orbit} \cdot e_r)$$

ここで：
- $(e_n, e_e) = (p_n - c_n, p_e - c_e)$: 位置誤差
- $e_r = \sqrt{e_n^2 + e_e^2} - R$: 半径誤差
- $\lambda = +1$（反時計回り）または $-1$（時計回り）: 旋回方向
- $k_{orbit}$: 誘導ゲイン

**安定性：**

$$\dot{V} = -V_a k_{orbit} e_r^2 < 0 \quad \forall e_r \neq 0$$

**収束速度：**

$$e_r(t) = e_r(0) \exp(-V_a k_{orbit} t)$$

### 設計パラメータ

| パラメータ | 記号 | 典型的な値 | 単位 |
|-----------|------|-----------|------|
| 旋回半径 | $R$ | 50-500 | m |
| 誘導ゲイン | $k_{orbit}$ | 1-3 | - |
| 対気速度 | $V_a$ | 15-30 | m/s |
| GPSノイズ（水平） | $\sigma_h$ | 2.5 | m |
| GPS更新レート | $f_{GPS}$ | 5 | Hz |
| カルマンプロセスノイズ | $Q$ | 0.01 | m² |
| カルマン測定ノイズ | $R$ | 6.25 | m² |

## 参考文献

1. Beard, R. W., & McLain, T. W. (2012). *Small Unmanned Aircraft: Theory and Practice*. Princeton University Press.

2. Park, S., Deyst, J., & How, J. P. (2007). "Performance and Lyapunov Stability of a Nonlinear Path Following Guidance Method." *Journal of Guidance, Control, and Dynamics*, 30(6), 1718-1728.

3. Fossen, T. I., & Pettersen, K. Y. (2014). "On uniform semiglobal exponential stability (USGES) of proportional line-of-sight guidance laws." *Automatica*, 50(11), 2912-2917.

4. Lawrence, D. A., Frew, E. W., & Pisano, W. J. (2008). "Lyapunov Vector Fields for Autonomous UAV Flight Control." *Journal of Guidance, Control, and Dynamics*, 31(5), 1220-1229.

## 付録: 代替的な定式化

### 比例航法

代替案として**比例航法**（PN）があります：

$$\psi_c = \psi + k_{PN} \dot{\lambda}$$

ここで $\dot{\lambda}$ は視線角速度です。

**比較：**
- PN：迎撃ミッションに適している
- ベクトル場：旋回/待機に適している

### モデル予測制御（MPC）

より厳密な追従のために、MPCを使用できます：

- 時間範囲 $T_h$ にわたって将来の軌跡を予測
- 旋回誤差を最小化する制御を最適化
- より計算集約的

### 適応誘導

未知の風に対して：

- オンラインで風ベクトルを推定
- 補償するために誘導を調整
- 拡張カルマンフィルタ（EKF）が必要

---

**ドキュメントバージョン：** 1.0
**最終更新：** 2025-10-21
**著者：** UAVシミュレーター開発チーム
