# 固定翼UAVシミュレータ

固定翼UAVの制御則と誘導則を検証するための6自由度シミュレーションプログラム

## 特徴

- **6自由度動力学モデル**: 完全な位置、速度、姿勢、角速度の動力学
- **空力モデル**: 安定微係数と操縦微係数に基づく空力特性
- **複数の機体モデル**: 超小型から大型まで4種類の機体パラメータ
- **制御則**: 姿勢制御(ロール、ピッチ、ヨー)、高度制御、速度制御
- **誘導則**: 経路点追従、直線経路追従、旋回飛行
- **可視化**: リアルタイムの3D軌跡と状態プロット

## 機体モデル

### プリセット機体モデル

4種類の固定翼UAVモデルを用意しています:

| モデル名 | 翼幅 | 質量 | 安定性 | 用途 |
|---------|------|------|--------|------|
| **micro** | 0.8m | 0.5kg | 安定 | 超小型（屋内用） |
| **small** | **1.6m** | **1.7kg** | **安定** | **小型（目標機体、デフォルト）** |
| **medium** | 2.9m | 11.0kg | 安定 | 中型（Rascalクラス） |
| **large** | 4.0m | 20.0kg | 安定 | 大型（長時間飛行用） |

### 安定性バリエーション

小型UAV（1.6m, 1.7kg）の安定性バリエーションを用意:

| モデル名 | ロール安定性 | ピッチ安定性 | ヨー安定性 | 用途 |
|---------|------------|------------|-----------|------|
| **small** | 安定 | 安定 | 安定 | 標準機体（デフォルト） |
| **small_slightly_unstable** | やや不安定 | やや不安定 | 中立 | 中程度の制御課題 |
| **small_unstable** | 不安定 | 不安定 | 不安定 | 高度な制御課題 |

### カスタム機体生成

`aircraft_generator.py`を使用して、任意のパラメータで機体を生成可能:

```python
from src.aircraft_generator import create_aircraft

# 翼幅2.0m、質量3.0kgでピッチがやや不安定な機体
aircraft_params, aero_params = create_aircraft(
    wingspan=2.0,
    mass=3.0,
    roll_stability='stable',
    pitch_stability='slightly_unstable',
    yaw_stability='stable',
    aspect_ratio=10.0
)
```

## 必要なパッケージ

```bash
pip install numpy scipy matplotlib
```

## プロジェクト構造

```
uav_simulator/
├── src/
│   ├── dynamics.py            # 航空機動力学モデル
│   ├── aerodynamics.py        # 空力モデル
│   ├── controller.py          # 制御則
│   ├── guidance.py            # 誘導則
│   ├── visualization.py       # 可視化ツール
│   └── aircraft_generator.py  # 機体生成モジュール
├── config/
│   └── aircraft_params.py     # 機体パラメータ定義
├── examples/
│   ├── basic_flight.py        # 基本飛行シミュレーション
│   ├── waypoint_nav.py        # 経路点航法
│   ├── orbit_flight.py        # 旋回飛行
│   ├── small_uav_demo.py      # 小型UAVデモ
│   └── compare_aircraft.py    # 機体比較シミュレーション
├── docs/
│   └── requirements.md        # 要件定義書
├── test_sim.py                # 基本動作確認
├── test_aircraft_models.py    # 機体モデル確認
└── README.md
```

## 使用方法

### 基本的なシミュレーション

```python
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController
from src.guidance import WaypointGuidance
import numpy as np

# 小型UAV（1.6m, 1.7kg）を初期化
uav = FixedWingUAV(aircraft_type='small')
aero = AerodynamicModel(aircraft_type='small')

# 制御器を初期化
controller = AttitudeController()

# 誘導則を初期化
waypoints = np.array([[0, 0, -100], [500, 0, -100], [500, 500, -100]])
guidance = WaypointGuidance(waypoints)

# シミュレーションループ
# ... (examples参照)
```

### 機体タイプの指定

```python
# 方法1: 機体タイプを指定
uav = FixedWingUAV(aircraft_type='small')  # 'micro', 'small', 'medium', 'large'
aero = AerodynamicModel(aircraft_type='small')

# 方法2: カスタムパラメータを使用
from config.aircraft_params import get_small_uav_params, get_small_uav_aero_params
aircraft_params = get_small_uav_params()
aero_params = get_small_uav_aero_params()
uav = FixedWingUAV(params=aircraft_params)
aero = AerodynamicModel(params=aero_params)
```

### サンプル実行

```bash
# 基本飛行シミュレーション
python examples/basic_flight.py

# 経路点航法
python examples/waypoint_nav.py

# 旋回飛行
python examples/orbit_flight.py

# 小型UAV専用デモ（全幅1.6m、質量1.7kg）
python examples/small_uav_demo.py

# 複数機体の比較
python examples/compare_aircraft.py
```

## 座標系

- **北東下(NED)座標系**: 慣性座標系として使用
  - X軸: 北方向
  - Y軸: 東方向
  - Z軸: 下方向(高度は負の値)

- **機体座標系**: 機体に固定された座標系
  - X軸: 機首方向
  - Y軸: 右翼方向
  - Z軸: 下方向

## 状態変数

- **位置**: (x, y, z) [m] - NED座標系
- **速度**: (u, v, w) [m/s] - 機体座標系
- **姿勢**: (φ, θ, ψ) [rad] - ロール、ピッチ、ヨー角
- **角速度**: (p, q, r) [rad/s] - 機体座標系

## 制御入力と符号規約

### 制御入力

- **δe**: エレベータ偏角 [rad]
- **δa**: エルロン偏角 [rad]
- **δr**: ラダー偏角 [rad]
- **δt**: スロットル [0-1]

### 舵面の符号規約

本シミュレータでは以下の符号規約を採用しています：

#### エレベータ (Elevator)
- **正の偏角 (δe > 0)**: 後縁 (Trailing Edge) が下がる方向
  - 効果: 水平尾翼の揚力が増加 → 機首下げモーメント
  - 揚力係数: C_L_delta_e > 0 (正) - 揚力増加
  - モーメント係数: C_m_delta_e < 0 (負) - 機首下げ

#### エルロン (Aileron)
- **正の偏角 (δa > 0)**: 左翼の後縁が下がり、右翼の後縁が上がる方向
  - 効果: 正のロールモーメント (右翼下がり、右旋回)
  - モーメント係数: C_l_delta_a > 0 (正)

#### ラダー (Rudder)
- **正の偏角 (δr > 0)**: 後縁が左に動く方向
  - 効果: 右向きの横力 → 左ヨー (機首が左に向く)
  - 横力係数: C_Y_delta_r > 0 (正) - 右向き横力
  - モーメント係数: C_n_delta_r < 0 (負) - 左ヨー

**注意**: この符号規約は航空工学の標準的な規約に従っています。制御則を実装する際は、この規約に基づいて制御ゲインの符号を決定してください。

## ライセンス

MIT License

## 参考文献

- Beard, R. W., & McLain, T. W. (2012). Small Unmanned Aircraft: Theory and Practice.
- Stevens, B. L., & Lewis, F. L. (2003). Aircraft Control and Simulation.
