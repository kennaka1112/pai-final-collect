#!/usr/bin/env python3
"""CRANE+ V2 で「左右2個のブロックを中央の目標エリアへ順に押し集める」動作を生成する。

1個目（赤）を中央へ押す → 手先を持ち上げてブロックの上を越えて反対側へ回る →
2個目（青）を中央へ押す、という2段構えの軌道。
軌道は関節空間で作り、physicalai_teleop の teleop_keyboard と同じトピック・
同じ20Hzで送るので、rosbag の形式は手集めのデータと同じになる。

アームの平面2リンクIK（joint2, joint3）で手先(joint4軸)を極座標に置き、
joint4で手先の向きを真下に固定、joint1(旋回)でブロックを円弧状に押す。
"""
import math

# physicalai_teleop/teleop_keyboard.py と同じ定義
ARM_JOINT_NAMES = ['crane_plus_joint1', 'crane_plus_joint2',
                   'crane_plus_joint4', 'crane_plus_joint3']
GRIPPER_JOINT_NAME = 'crane_plus_joint_hand'
RATE_HZ = 20.0

# URDFから読んだリンク長（crane_plus.xacro）
L_UPPER = 0.083      # joint2 -> joint3
L_FORE = 0.0935      # joint3 -> joint4(手先原点)
H_SHOULDER = 0.0714  # base_link -> joint2

# 手先の高さ[m]（base_link基準）。グリッパを真下に向けたとき
#   0.131 付近で指先が天板に接触する（実測）
H_PUSH = 0.142       # 押すとき: 指先が天板すれすれ
H_OVER = 0.195       # ブロックの上を越えて移動するとき（4cmのブロックを跨ぐ）
H_LIFT = 0.205       # 押し終わって持ち上げるとき
R_WORK = 0.140       # 作業半径[m]

GRIPPER_CLOSED = 0.0
UPRIGHT = [0.0, 0.0, 0.0, 0.0]

# URDF(crane_plus.urdf.xacro)の可動域。ARM_JOINT_NAMES と同じ並び [j1, j2, j4, j3]。
# 可動域の外を指令すると実際の関節はそこで張り付き、指令値と実際の姿勢がずれる。
# とくに joint4 は端(-1.84rad)に張り付くと戻るのに時間がかかり、
# エピソードの開始姿勢がそろわなくなるので、指令の時点で丸めておく。
#（端ちょうどに張り付かないよう、少し内側に取っている）
JOINT_LIMITS = [(-2.60, 2.60),      # joint1
                (-1.78, 1.74),      # joint2
                (-1.78, 1.72),      # joint4
                (-2.50, 2.40)]      # joint3


def ik_planar(radius, height):
    """手先(joint4軸)を 半径radius・高さheight に置く (joint2, joint3)。"""
    z = height - H_SHOULDER
    cos_d = (radius * radius + z * z - L_UPPER ** 2 - L_FORE ** 2) / (2 * L_UPPER * L_FORE)
    cos_d = max(-1.0, min(1.0, cos_d))
    d = math.acos(cos_d)
    phi = math.atan2(radius, z)
    j2 = phi - math.atan2(L_FORE * math.sin(d), L_UPPER + L_FORE * math.cos(d))
    j3 = -d
    return j2, j3


def clamp_to_limits(pose):
    """指令を可動域内に丸める（ARM_JOINT_NAMES の並び）。"""
    return [min(max(v, lo), hi) for v, (lo, hi) in zip(pose, JOINT_LIMITS)]


def arm_pose(yaw, radius, height, hand_dir=math.pi):
    """ARM_JOINT_NAMES の並び [joint1, joint2, joint4, joint3] で姿勢を返す。
       hand_dir=pi はグリッパを真下に向ける。可動域は超えないよう丸める。"""
    j2, j3 = ik_planar(radius, height)
    j4 = (j2 - j3) - hand_dir
    return clamp_to_limits([yaw, j2, j4, j3])


def smoothstep(x):
    x = min(max(x, 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * x)


def _interp(a, b, t):
    s = smoothstep(t)
    return [ai + (bi - ai) * s for ai, bi in zip(a, b)]


def collect_trajectory(yaw_red_start=-0.80, yaw_red_goal=-0.21,
                       yaw_blue_start=0.95, yaw_blue_goal=0.64,
                       radius=R_WORK, h_push=H_PUSH,
                       sweep_sec=3.0, speed=1.0):
    """1エピソード分の [arm_pos, ...] を 20Hz で返す。

    yaw_*_start : そのブロックの手前に回り込む旋回角[rad]
    yaw_*_goal  : 押し終わりの旋回角[rad]（中央の目標エリア側）

    グリッパはjoint4から見て横に 14.5mm ずれて付いているため、実際の接触点は
    joint1の指令値より約0.09rad 手前側になる。押す向きが左右で逆なので、
    赤(+方向へ押す)と青(-方向へ押す)で開始・終了角は対称にならない。
    sweep_sec   : 1個あたり押している時間[s]
    speed       : 全体の速さ倍率（1.0が標準）
    """
    over_r = arm_pose(yaw_red_start, radius, H_OVER)
    down_r = arm_pose(yaw_red_start, radius, h_push)
    end_r = arm_pose(yaw_red_goal, radius, h_push)
    lift_r = arm_pose(yaw_red_goal, radius, H_LIFT)

    over_b = arm_pose(yaw_blue_start, radius, H_OVER)
    down_b = arm_pose(yaw_blue_start, radius, h_push)
    end_b = arm_pose(yaw_blue_goal, radius, h_push)
    lift_b = arm_pose(yaw_blue_goal, radius, H_LIFT)

    # 軌道の開始姿勢(直立)と終了姿勢(目標上空で静止)はあえて別のものにしている。
    # 両端を直立にすると、関節状態だけを観測する方策からは
    # 「これから押す」のか「終わった」のかが区別できず、直立のまま止まってしまう。
    phases = [
        (UPRIGHT, UPRIGHT, 0.5),       # 直立で静止（エピソードの開始合図）
        (UPRIGHT, over_r, 2.2),        # 赤ブロックの手前の上空へ
        (over_r, down_r, 1.2),         # 天板すれすれまで下ろす
        (down_r, end_r, sweep_sec),    # 赤を中央へ押す
        (end_r, lift_r, 1.0),          # 持ち上げる
        (lift_r, over_b, 2.6),         # ブロックの上を越えて反対側へ回る
        (over_b, down_b, 1.2),         # 下ろす
        (down_b, end_b, sweep_sec),    # 青を中央へ押す
        (end_b, lift_b, 1.0),          # 持ち上げる
        (lift_b, lift_b, 1.5),         # 目標エリアの上空で静止（エピソード終了）
    ]

    out = []
    for a, b, sec in phases:
        sec = max(sec / speed, 0.05)
        steps = max(int(sec * RATE_HZ), 1)
        for i in range(steps):
            out.append(_interp(a, b, (i + 1) / steps))
    return out
