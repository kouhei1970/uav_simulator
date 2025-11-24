# 空力モデル (aerodynamics.py)

## 概要

`aerodynamics.py`モジュールは、固定翼UAVの空力特性をモデル化し、機体に作用する空力力とモーメントを計算します。安定微係数と操縦微係数に基づく線形空力モデルを実装しています。

## クラス: AerodynamicModel

### 初期化

```python
from src.aerodynamics import AerodynamicModel

# デフォルトパラメータで初期化（小型UAV）
aero = AerodynamicModel()

# 機体タイプを指定して初期化
aero = AerodynamicModel(aircraft_type='small')

# カスタムパラメータで初期化
custom_params = {
    'C_L_0': 0.30,
    'C_L_alpha': 4.00,
    # ... その他の係数
}
aero = AerodynamicModel(params=custom_params)
```

### 利用可能な機体タイプ

- `'micro'`: 超小型UAV（翼幅0.8m、質量0.5kg）
- `'small'`: 小型UAV（翼幅1.6m、質量1.7kg）- デフォルト
- `'medium'`: 中型UAV（翼幅2.9m、質量11.0kg）
- `'large'`: 大型UAV（翼幅4.0m、質量20.0kg）

安定性バリエーション：
- `'small'`: 安定型
- `'small_slightly_unstable'`: やや不安定型
- `'small_unstable'`: 不安定型

## 舵面の符号規約

本シミュレータでは、以下の符号規約を採用しています：

### エレベータ（Elevator）
- **正の偏角 (δe > 0)**: 後縁（TE）が下がる方向
  - 効果：水平尾翼の揚力増加 → **機首下げモーメント**
  - 揚力係数：**C_L_delta_e > 0**（正）- 揚力増加
  - モーメント係数：**C_m_delta_e < 0**（負）- 機首下げ

### エルロン（Aileron）
- **正の偏角 (δa > 0)**: 左翼の後縁が下がり、右翼の後縁が上がる方向
  - 効果：**正のロールモーメント**（右翼下がり、右旋回）
  - モーメント係数：**C_l_delta_a > 0**（正）

### ラダー（Rudder）
- **正の偏角 (δr > 0)**: 後縁が左に動く方向
  - 効果：右向きの横力 → **左ヨー**（機首が左に向く）
  - 横力係数：**C_Y_delta_r > 0**（正）- 右向き横力
  - モーメント係数：**C_n_delta_r < 0**（負）- 左ヨー

## 主要メソッド

### compute_forces_moments(uav, control)

空力力とモーメントを計算します。

**パラメータ：**
- `uav`: FixedWingUAVオブジェクト（現在の状態を含む）
- `control`: 制御入力配列 `[delta_e, delta_a, delta_r, delta_t]`

**戻り値：**
- `forces_moments`: 配列 `[Fx, Fy, Fz, L, M, N]`
  - `Fx, Fy, Fz`: 機体座標系での力 [N]
  - `L, M, N`: モーメント（ロール、ピッチ、ヨー）[N⋅m]

```python
forces_moments = aero.compute_forces_moments(uav, control)
Fx, Fy, Fz = forces_moments[0:3]  # 力
L, M, N = forces_moments[3:6]     # モーメント
```

## 空力係数

### 揚力係数 (C_L)

```
C_L = C_L_0 + C_L_alpha * α + C_L_q * (c*q)/(2*Va) + C_L_delta_e * δe
```

**各項の意味：**
- **C_L_0**: 基本揚力係数（迎角ゼロでの揚力）
- **C_L_alpha**: 迎角に対する揚力変化率（揚力傾斜）
- **C_L_q**: ピッチレートに対する揚力変化
- **C_L_delta_e**: エレベータによる揚力変化（**正**）

**小型UAVの典型値：**
- C_L_0 = 0.30
- C_L_alpha = 4.00 [1/rad]
- C_L_q = 0.0
- C_L_delta_e = 0.40 [1/rad]

### 抗力係数 (C_D)

```
C_D = C_D_0 + C_D_alpha * α + C_D_q * (c*q)/(2*Va) + C_D_delta_e * δe
```

**各項の意味：**
- **C_D_0**: 基本抗力係数（有害抗力）
- **C_D_alpha**: 迎角による抗力増加
- **C_D_q**: ピッチレートによる抗力変化
- **C_D_delta_e**: エレベータによる抗力変化

**小型UAVの典型値：**
- C_D_0 = 0.028
- C_D_alpha = 0.25
- C_D_q = 0.0
- C_D_delta_e = 0.0

### 横力係数 (C_Y)

```
C_Y = C_Y_0 + C_Y_beta * β + C_Y_p * (b*p)/(2*Va) + C_Y_r * (b*r)/(2*Va)
      + C_Y_delta_a * δa + C_Y_delta_r * δr
```

**各項の意味：**
- **C_Y_beta**: 横滑り角による横力（負で安定）
- **C_Y_p**: ロールレートによる横力
- **C_Y_r**: ヨーレートによる横力
- **C_Y_delta_a**: エルロンによる横力
- **C_Y_delta_r**: ラダーによる横力（**正**）

**小型UAVの典型値：**
- C_Y_0 = 0.0
- C_Y_beta = -0.90 [1/rad]
- C_Y_delta_r = 0.20 [1/rad]

### ローリングモーメント係数 (C_l)

```
C_l = C_l_0 + C_l_beta * β + C_l_p * (b*p)/(2*Va) + C_l_r * (b*r)/(2*Va)
      + C_l_delta_a * δa + C_l_delta_r * δr
```

**各項の意味：**
- **C_l_beta**: 横滑り角による復元モーメント（負で安定、上反角効果）
- **C_l_p**: ロールダンピング（負で安定）
- **C_l_r**: ヨーレートとロールのカップリング
- **C_l_delta_a**: エルロン効果（正）
- **C_l_delta_r**: ラダーのロールカップリング

**小型UAVの典型値：**
- C_l_0 = 0.0
- C_l_beta = -0.15 [1/rad]（負で安定）
- C_l_p = -0.30（負で安定）
- C_l_delta_a = 0.12 [1/rad]

### ピッチングモーメント係数 (C_m)

```
C_m = C_m_0 + C_m_alpha * α + C_m_q * (c*q)/(2*Va) + C_m_delta_e * δe
```

**各項の意味：**
- **C_m_0**: 基本ピッチングモーメント係数
- **C_m_alpha**: 静安定微係数（負で安定）
- **C_m_q**: ピッチダンピング（負で安定）
- **C_m_delta_e**: エレベータ効果（**負**、機首下げ）

**小型UAVの典型値：**
- C_m_0 = -0.025
- C_m_alpha = -0.50 [1/rad]（負で安定）
- C_m_q = -4.0（負で安定）
- C_m_delta_e = -0.55 [1/rad]

### ヨーイングモーメント係数 (C_n)

```
C_n = C_n_0 + C_n_beta * β + C_n_p * (b*p)/(2*Va) + C_n_r * (b*r)/(2*Va)
      + C_n_delta_a * δa + C_n_delta_r * δr
```

**各項の意味：**
- **C_n_beta**: 方向安定性（正で安定、風見効果）
- **C_n_p**: ロールとヨーのカップリング
- **C_n_r**: ヨーダンピング（負で安定）
- **C_n_delta_a**: エルロンのヨーカップリング（アドバースヨー）
- **C_n_delta_r**: ラダー効果（負、左ヨー）

**小型UAVの典型値：**
- C_n_0 = 0.0
- C_n_beta = 0.30 [1/rad]（正で安定）
- C_n_r = -0.40（負で安定）
- C_n_delta_r = -0.04 [1/rad]

## 安定微係数の物理的意味

### 縦の安定性（Longitudinal Stability）

#### 静安定性（Static Stability）
- **C_m_alpha < 0**: 迎角が増加すると機首を下げるモーメントが発生
  - 例：C_m_alpha = -0.50 の場合、迎角が1ラジアン増えるとC_mが0.50減少（機首下げ）

#### 動安定性（Dynamic Stability）
- **C_m_q < 0**: ピッチレートを減衰させる効果
  - 例：C_m_q = -4.0 の場合、強い減衰効果

### 横の安定性（Lateral Stability）

#### 上反角効果（Dihedral Effect）
- **C_l_beta < 0**: 横滑りすると復元ロールモーメントが発生
  - 例：C_l_beta = -0.15 の場合、右に滑ると右翼が上がる

#### ロールダンピング
- **C_l_p < 0**: ロール運動を減衰
  - 例：C_l_p = -0.30 の場合、適度なダンピング

### 方向の安定性（Directional Stability）

#### 風見安定性（Weathercock Stability）
- **C_n_beta > 0**: 横滑りすると機首が風向きに向く
  - 例：C_n_beta = 0.30 の場合、良好な方向安定性

#### ヨーダンピング
- **C_n_r < 0**: ヨー運動を減衰
  - 例：C_n_r = -0.40 の場合、強いヨーダンピング

## 力とモーメントの計算

### 風軸での空力力

```
動圧 q_bar = 0.5 * ρ * Va²

揚力 Lift = q_bar * S * C_L
抗力 Drag = q_bar * S * C_D
横力 Side = q_bar * S * C_Y
```

**パラメータ：**
- ρ: 空気密度 [kg/m³]（標準：1.225）
- Va: 対気速度 [m/s]
- S: 主翼面積 [m²]

### 機体軸への座標変換

風軸から機体軸への変換（迎角αによる回転）：

```
Fx_aero = -Drag * cos(α) + Lift * sin(α)
Fy_aero = Side
Fz_aero = -Drag * sin(α) - Lift * cos(α)
```

### 空力モーメント

```
L (ロールモーメント)  = q_bar * S * b * C_l
M (ピッチモーメント)  = q_bar * S * c * C_m
N (ヨーモーメント)    = q_bar * S * b * C_n
```

**スケーリング：**
- b: 翼幅 [m]（ロール・ヨーに使用）
- c: 平均翼弦長 [m]（ピッチに使用）

### 推力モデル

簡易プロペラモデル：

```
Thrust = 0.5 * ρ * S_prop * C_prop * [(k_motor * δt)² - Va²]
```

**パラメータ：**
- S_prop: プロペラ面積 [m²]
- C_prop: プロペラ効率係数
- k_motor: モーター定数
- δt: スロットル [0-1]

## 使用例

### 基本的な力の計算

```python
import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel

# 初期化
uav = FixedWingUAV(aircraft_type='small')
aero = AerodynamicModel(aircraft_type='small')

# 飛行状態を設定（15 m/s、高度100m、ピッチ5度）
uav.set_state([
    0, 0, -100,           # 位置 (NED)
    15, 0, 0,             # 速度 (機体座標系)
    0, np.deg2rad(5), 0,  # 姿勢 (ロール、ピッチ、ヨー)
    0, 0, 0               # 角速度
])

# 制御入力: [エレベータ, エルロン, ラダー, スロットル]
control = np.array([0.0, 0.0, 0.0, 0.3])

# 空力力・モーメントを計算
forces_moments = aero.compute_forces_moments(uav, control)

print(f"空力力:")
print(f"  Fx = {forces_moments[0]:.2f} N")
print(f"  Fy = {forces_moments[1]:.2f} N")
print(f"  Fz = {forces_moments[2]:.2f} N")
print(f"空力モーメント:")
print(f"  L (ロール)  = {forces_moments[3]:.4f} N⋅m")
print(f"  M (ピッチ)  = {forces_moments[4]:.4f} N⋅m")
print(f"  N (ヨー)    = {forces_moments[5]:.4f} N⋅m")
```

### エレベータ効果の確認

```python
# トリム状態でのエレベータトリム値を計算
alpha_trim = np.deg2rad(1.9)
C_m_0 = aero.aero_params['C_m_0']
C_m_alpha = aero.aero_params['C_m_alpha']
C_m_delta_e = aero.aero_params['C_m_delta_e']

elevator_trim = -(C_m_0 + C_m_alpha * alpha_trim) / C_m_delta_e
print(f"エレベータトリム: {np.rad2deg(elevator_trim):.2f}°")

# 正のエレベータ偏角の効果
delta_e_positive = np.deg2rad(5.0)
control_positive = np.array([delta_e_positive, 0, 0, 0.3])
fm_positive = aero.compute_forces_moments(uav, control_positive)

print(f"\n正のエレベータ (+5°) の効果:")
print(f"  ピッチモーメント M = {fm_positive[4]:.4f} N⋅m")
print(f"  → {'機首下げ' if fm_positive[4] < 0 else '機首上げ'}")
```

### 迎角と空力係数の計算

```python
# 迎角と横滑り角を取得
alpha = uav.get_angle_of_attack()
beta = uav.get_sideslip_angle()

print(f"\n飛行状態:")
print(f"  対気速度 Va = {uav.get_airspeed():.2f} m/s")
print(f"  迎角 α = {np.rad2deg(alpha):.2f}°")
print(f"  横滑り角 β = {np.rad2deg(beta):.2f}°")

# 揚力・抗力係数を計算
C_L = (aero.aero_params['C_L_0'] +
       aero.aero_params['C_L_alpha'] * alpha)
C_D = (aero.aero_params['C_D_0'] +
       aero.aero_params['C_D_alpha'] * alpha)

print(f"\n空力係数:")
print(f"  揚力係数 C_L = {C_L:.4f}")
print(f"  抗力係数 C_D = {C_D:.4f}")
print(f"  揚抗比 L/D = {C_L/C_D:.2f}")
```

### 安定性の比較

```python
from config.aircraft_params import print_aircraft_info

# 異なる安定性の機体を比較
print("="*60)
print("安定型機体")
print("="*60)
print_aircraft_info('small')

print("\n" + "="*60)
print("やや不安定型機体")
print("="*60)
print_aircraft_info('small_slightly_unstable')

print("\n" + "="*60)
print("不安定型機体")
print("="*60)
print_aircraft_info('small_unstable')
```

## トリム計算

トリム状態（釣り合い飛行状態）では、すべての力とモーメントがゼロになります。

### ピッチトリム

```python
# トリム条件：M = 0 より
# C_m = C_m_0 + C_m_alpha * α + C_m_delta_e * δe = 0

# エレベータトリム値を求める
elevator_trim = -(C_m_0 + C_m_alpha * alpha_trim) / C_m_delta_e
```

**注意点：**
- C_m_delta_e が負（本シミュレータの符号規約）なので、通常 elevator_trim は負の値になります
- これは後縁が上がる方向（機首上げ）でトリムすることを意味します

## 動力学モデルとの統合

```python
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
import numpy as np

# 初期化
uav = FixedWingUAV(aircraft_type='small')
aero = AerodynamicModel(aircraft_type='small')

# シミュレーションパラメータ
dt = 0.01  # 時間刻み [s]
t_end = 10.0  # 終了時刻 [s]
num_steps = int(t_end / dt)

# データ記録用
history = {
    'time': [],
    'altitude': [],
    'pitch': [],
    'elevator': []
}

# シミュレーションループ
for i in range(num_steps):
    t = i * dt

    # 制御入力（例：一定）
    control = np.array([0.0, 0.0, 0.0, 0.3])

    # 空力力・モーメントを計算
    forces_moments = aero.compute_forces_moments(uav, control)

    # 動力学を更新（RK4積分）
    uav.state = uav.runge_kutta_step(
        uav.state, control,
        forces_moments[0:3], forces_moments[3:6],
        dt
    )

    # データ記録
    history['time'].append(t)
    history['altitude'].append(-uav.state[2])
    history['pitch'].append(np.rad2deg(uav.state[7]))
    history['elevator'].append(np.rad2deg(control[0]))
```

## 注意事項と制限

### 線形化の仮定
- 本モデルは小さな摂動に対して線形化されています
- 高迎角（失速領域）や高速飛行では精度が低下します
- 適用範囲：迎角 ±15度程度

### 風の影響
- 現在の実装では無風を仮定しています
- 風の影響を考慮する場合は、対気速度の計算を修正する必要があります

### プロペラモデル
- 簡易的なプロペラモデルを使用しています
- より詳細なモデルが必要な場合は、プロペラ効率曲線を実装してください

## 関連ドキュメント

- [dynamics.md](dynamics.md) - 6自由度動力学モデル
- [controller.md](controller.md) - 制御則の実装
- [aircraft_generator.md](aircraft_generator.md) - カスタム機体の生成
- [README.md](../README.md) - プロジェクト全体の説明

## 参考文献

- Beard, R. W., & McLain, T. W. (2012). *Small Unmanned Aircraft: Theory and Practice*. Princeton University Press.
- Stevens, B. L., & Lewis, F. L. (2003). *Aircraft Control and Simulation* (2nd ed.). Wiley.
- Nelson, R. C. (1998). *Flight Stability and Automatic Control* (2nd ed.). McGraw-Hill.
