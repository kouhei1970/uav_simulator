"""
固定翼UAV機体生成モジュール

主翼スパン、質量、安定性特性から機体パラメータを自動生成
"""

import numpy as np


class StabilityLevel:
    """安定性レベルの定義"""
    STABLE = 'stable'           # 安定
    NEUTRAL = 'neutral'         # 中立
    UNSTABLE = 'unstable'       # 不安定
    SLIGHTLY_UNSTABLE = 'slightly_unstable'  # やや不安定


class AircraftGenerator:
    """機体パラメータ生成器"""

    def __init__(self):
        """初期化"""
        pass

    def generate_aircraft(self,
                         wingspan,
                         mass,
                         roll_stability=StabilityLevel.STABLE,
                         pitch_stability=StabilityLevel.STABLE,
                         yaw_stability=StabilityLevel.STABLE,
                         aspect_ratio=9.0,
                         cruise_speed=None,
                         name="custom"):
        """
        機体パラメータを生成

        パラメータ:
            wingspan: 翼幅 [m]
            mass: 質量 [kg]
            roll_stability: ロール安定性 ('stable', 'neutral', 'unstable', 'slightly_unstable')
            pitch_stability: ピッチ安定性
            yaw_stability: ヨー安定性
            aspect_ratio: アスペクト比 (デフォルト: 9.0)
            cruise_speed: 巡航速度 [m/s] (Noneの場合は翼面荷重から自動計算)
            name: 機体名

        戻り値:
            (aircraft_params, aero_params): 機体パラメータと空力パラメータのタプル
        """
        # 幾何パラメータの計算
        b = wingspan
        S_wing = b**2 / aspect_ratio  # 翼面積
        c = S_wing / b  # 平均翼弦長

        # 慣性モーメントの推定（経験式）
        # 一般的な固定翼機の慣性モーメント比率を使用
        Jx = 0.05 * mass * b**2  # ロール軸
        Jy = 0.06 * mass * (b**2 + c**2)  # ピッチ軸
        Jz = 0.11 * mass * b**2  # ヨー軸
        Jxz = 0.009 * mass * b * c  # XZ慣性乗積

        # プロペラサイズの推定（翼幅に対する比率）
        prop_diameter = 0.1 * b  # 翼幅の10%
        S_prop = np.pi * (prop_diameter / 2)**2

        # モーター定数（質量ベース）
        k_motor = 20.0 + 20.0 * np.sqrt(mass)

        # 巡航速度の推定（指定されていない場合）
        if cruise_speed is None:
            # 翼面荷重から巡航速度を推定
            # V_cruise = sqrt((2 * W) / (rho * S * C_L_cruise))
            # C_L_cruise ≈ 0.6 (typical cruise lift coefficient)
            rho = 1.225  # kg/m^3
            gravity = 9.81  # m/s^2
            C_L_cruise = 0.6
            wing_loading = (mass * gravity) / S_wing  # N/m^2
            cruise_speed = np.sqrt((2 * wing_loading) / (rho * C_L_cruise))

        # 機体パラメータ
        aircraft_params = {
            'name': name,
            'mass': mass,
            'Jx': Jx,
            'Jy': Jy,
            'Jz': Jz,
            'Jxz': Jxz,
            'S_wing': S_wing,
            'b': b,
            'c': c,
            'S_prop': S_prop,
            'k_motor': k_motor,
            'k_T_P': 0.0,
            'k_Omega': 0.0,
            'V_cruise': cruise_speed,
            'rho': 1.225,
            'gravity': 9.81,
        }

        # 慣性の計算用の定数
        aircraft_params['Gamma'] = aircraft_params['Jx'] * aircraft_params['Jz'] - aircraft_params['Jxz']**2
        aircraft_params['Gamma1'] = (aircraft_params['Jxz'] * (aircraft_params['Jx'] - aircraft_params['Jy'] + aircraft_params['Jz'])) / aircraft_params['Gamma']
        aircraft_params['Gamma2'] = (aircraft_params['Jz'] * (aircraft_params['Jz'] - aircraft_params['Jy']) + aircraft_params['Jxz']**2) / aircraft_params['Gamma']
        aircraft_params['Gamma3'] = aircraft_params['Jz'] / aircraft_params['Gamma']
        aircraft_params['Gamma4'] = aircraft_params['Jxz'] / aircraft_params['Gamma']
        aircraft_params['Gamma5'] = (aircraft_params['Jz'] - aircraft_params['Jx']) / aircraft_params['Jy']
        aircraft_params['Gamma6'] = aircraft_params['Jxz'] / aircraft_params['Jy']
        aircraft_params['Gamma7'] = ((aircraft_params['Jx'] - aircraft_params['Jy']) * aircraft_params['Jx'] + aircraft_params['Jxz']**2) / aircraft_params['Gamma']
        aircraft_params['Gamma8'] = aircraft_params['Jx'] / aircraft_params['Gamma']

        # 空力パラメータの生成
        aero_params = self._generate_aero_params(
            wingspan, mass, aspect_ratio,
            roll_stability, pitch_stability, yaw_stability
        )

        return aircraft_params, aero_params

    def _generate_aero_params(self, wingspan, mass, aspect_ratio,
                             roll_stability, pitch_stability, yaw_stability):
        """
        空力パラメータを生成

        パラメータ:
            wingspan: 翼幅 [m]
            mass: 質量 [kg]
            aspect_ratio: アスペクト比
            roll_stability: ロール安定性
            pitch_stability: ピッチ安定性
            yaw_stability: ヨー安定性

        戻り値:
            aero_params: 空力パラメータ辞書
        """
        # 基本揚力係数（アスペクト比から推定）
        C_L_0 = 0.28 + 0.01 * (aspect_ratio - 9.0)
        C_L_alpha = 2.0 * np.pi * aspect_ratio / (aspect_ratio + 2.0)  # 揚力線理論

        # 抗力係数
        C_D_0 = 0.025 + 0.005 / aspect_ratio  # 誘導抗力の影響
        C_D_alpha = 0.2 + 0.1 / aspect_ratio

        # ロール安定性パラメータ
        C_l_beta, C_l_p = self._get_roll_stability_params(roll_stability)

        # ピッチ安定性パラメータ
        C_m_alpha, C_m_q = self._get_pitch_stability_params(pitch_stability)

        # ヨー安定性パラメータ
        C_n_beta, C_n_r = self._get_yaw_stability_params(yaw_stability)

        aero_params = {
            # 揚力係数
            'C_L_0': C_L_0,
            'C_L_alpha': C_L_alpha,
            'C_L_q': 0.0,
            'C_L_delta_e': -0.36 - 0.04 * (aspect_ratio - 9.0) / 9.0,

            # 抗力係数
            'C_D_0': C_D_0,
            'C_D_alpha': C_D_alpha,
            'C_D_q': 0.0,
            'C_D_delta_e': 0.0,

            # 横力係数
            'C_Y_0': 0.0,
            'C_Y_beta': -0.90,
            'C_Y_p': 0.0,
            'C_Y_r': 0.0,
            'C_Y_delta_a': 0.0,
            'C_Y_delta_r': -0.20,

            # ローリングモーメント係数（ロール安定性）
            'C_l_0': 0.0,
            'C_l_beta': C_l_beta,
            'C_l_p': C_l_p,
            'C_l_r': 0.18,
            'C_l_delta_a': 0.10 + 0.02 / aspect_ratio,
            'C_l_delta_r': 0.10,

            # ピッチングモーメント係数（ピッチ安定性）
            'C_m_0': -0.025,
            'C_m_alpha': C_m_alpha,
            'C_m_q': C_m_q,
            'C_m_delta_e': -0.50 - 0.05 * (aspect_ratio - 9.0) / 9.0,

            # ヨーイングモーメント係数（ヨー安定性）
            'C_n_0': 0.0,
            'C_n_beta': C_n_beta,
            'C_n_p': 0.02,
            'C_n_r': C_n_r,
            'C_n_delta_a': 0.05,
            'C_n_delta_r': -0.04,

            # プロペラ係数
            'C_prop': 1.0,
            'k_T_P': 0.0,
            'k_Omega': 0.0,
        }

        return aero_params

    def _get_roll_stability_params(self, stability):
        """
        ロール安定性パラメータを取得

        C_l_beta: 上反角効果（負で安定）
        C_l_p: ロールダンピング（負で安定）
        """
        if stability == StabilityLevel.STABLE:
            C_l_beta = -0.15  # 強い復元力
            C_l_p = -0.30     # 強いダンピング
        elif stability == StabilityLevel.NEUTRAL:
            C_l_beta = -0.02  # 弱い復元力
            C_l_p = -0.10     # 弱いダンピング
        elif stability == StabilityLevel.SLIGHTLY_UNSTABLE:
            C_l_beta = 0.03   # 弱い発散傾向
            C_l_p = -0.15     # 中程度のダンピング
        elif stability == StabilityLevel.UNSTABLE:
            C_l_beta = 0.10   # 強い発散傾向
            C_l_p = -0.05     # 弱いダンピング
        else:
            C_l_beta = -0.15
            C_l_p = -0.30

        return C_l_beta, C_l_p

    def _get_pitch_stability_params(self, stability):
        """
        ピッチ安定性パラメータを取得

        C_m_alpha: ピッチング安定微係数（負で安定）
        C_m_q: ピッチダンピング（負で安定）
        """
        if stability == StabilityLevel.STABLE:
            C_m_alpha = -0.50  # 強い復元力
            C_m_q = -4.0       # 強いダンピング
        elif stability == StabilityLevel.NEUTRAL:
            C_m_alpha = -0.05  # 弱い復元力
            C_m_q = -1.0       # 弱いダンピング
        elif stability == StabilityLevel.SLIGHTLY_UNSTABLE:
            C_m_alpha = 0.05   # 弱い発散傾向
            C_m_q = -1.5       # 中程度のダンピング
        elif stability == StabilityLevel.UNSTABLE:
            C_m_alpha = 0.15   # 強い発散傾向
            C_m_q = -0.5       # 弱いダンピング
        else:
            C_m_alpha = -0.50
            C_m_q = -4.0

        return C_m_alpha, C_m_q

    def _get_yaw_stability_params(self, stability):
        """
        ヨー安定性パラメータを取得

        C_n_beta: 方向安定微係数（正で安定）
        C_n_r: ヨーダンピング（負で安定）
        """
        if stability == StabilityLevel.STABLE:
            C_n_beta = 0.30   # 強い復元力
            C_n_r = -0.40     # 強いダンピング
        elif stability == StabilityLevel.NEUTRAL:
            C_n_beta = 0.05   # 弱い復元力
            C_n_r = -0.10     # 弱いダンピング
        elif stability == StabilityLevel.SLIGHTLY_UNSTABLE:
            C_n_beta = -0.05  # 弱い発散傾向
            C_n_r = -0.15     # 中程度のダンピング
        elif stability == StabilityLevel.UNSTABLE:
            C_n_beta = -0.15  # 強い発散傾向
            C_n_r = -0.05     # 弱いダンピング
        else:
            C_n_beta = 0.30
            C_n_r = -0.40

        return C_n_beta, C_n_r


# 便利関数
def create_aircraft(wingspan, mass,
                   roll_stability='stable',
                   pitch_stability='stable',
                   yaw_stability='stable',
                   aspect_ratio=9.0,
                   cruise_speed=None,
                   name="custom"):
    """
    機体を簡単に生成する関数

    使用例:
        aircraft_params, aero_params = create_aircraft(
            wingspan=1.6,
            mass=1.7,
            roll_stability='stable',
            pitch_stability='slightly_unstable',
            yaw_stability='stable',
            cruise_speed=15.0
        )

    パラメータ:
        wingspan: 翼幅 [m]
        mass: 質量 [kg]
        roll_stability: ロール安定性 ('stable', 'neutral', 'slightly_unstable', 'unstable')
        pitch_stability: ピッチ安定性
        yaw_stability: ヨー安定性
        aspect_ratio: アスペクト比
        cruise_speed: 巡航速度 [m/s] (Noneの場合は自動計算)
        name: 機体名

    戻り値:
        (aircraft_params, aero_params): 機体パラメータと空力パラメータのタプル
    """
    generator = AircraftGenerator()
    return generator.generate_aircraft(
        wingspan=wingspan,
        mass=mass,
        roll_stability=roll_stability,
        pitch_stability=pitch_stability,
        yaw_stability=yaw_stability,
        aspect_ratio=aspect_ratio,
        cruise_speed=cruise_speed,
        name=name
    )


if __name__ == "__main__":
    # テスト
    print("=== 機体生成モジュールのテスト ===\n")

    # 安定な機体
    print("1. 安定な機体（1.6m, 1.7kg）")
    aircraft_params, aero_params = create_aircraft(
        wingspan=1.6,
        mass=1.7,
        roll_stability='stable',
        pitch_stability='stable',
        yaw_stability='stable',
        name="stable_uav"
    )
    print(f"  質量: {aircraft_params['mass']:.2f} kg")
    print(f"  翼幅: {aircraft_params['b']:.2f} m")
    print(f"  巡航速度: {aircraft_params['V_cruise']:.1f} m/s")
    print(f"  C_l_beta: {aero_params['C_l_beta']:.3f} (負で安定)")
    print(f"  C_m_alpha: {aero_params['C_m_alpha']:.3f} (負で安定)")
    print(f"  C_n_beta: {aero_params['C_n_beta']:.3f} (正で安定)")
    print()

    # やや不安定な機体
    print("2. やや不安定な機体（1.6m, 1.7kg）")
    aircraft_params, aero_params = create_aircraft(
        wingspan=1.6,
        mass=1.7,
        roll_stability='slightly_unstable',
        pitch_stability='slightly_unstable',
        yaw_stability='neutral',
        name="slightly_unstable_uav"
    )
    print(f"  C_l_beta: {aero_params['C_l_beta']:.3f}")
    print(f"  C_m_alpha: {aero_params['C_m_alpha']:.3f}")
    print(f"  C_n_beta: {aero_params['C_n_beta']:.3f}")
    print()

    # 不安定な機体
    print("3. 不安定な機体（1.6m, 1.7kg）")
    aircraft_params, aero_params = create_aircraft(
        wingspan=1.6,
        mass=1.7,
        roll_stability='unstable',
        pitch_stability='unstable',
        yaw_stability='unstable',
        name="unstable_uav"
    )
    print(f"  C_l_beta: {aero_params['C_l_beta']:.3f}")
    print(f"  C_m_alpha: {aero_params['C_m_alpha']:.3f}")
    print(f"  C_n_beta: {aero_params['C_n_beta']:.3f}")
