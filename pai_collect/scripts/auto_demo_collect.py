#!/usr/bin/env python3
"""「2個のブロックを集める」デモを自動生成して rosbag2 に記録する。

講義の pai-auto-demo（お辞儀デモ）と同じ構成・同じトピック・同じ20Hzで動かすので、
出力される rosbag はテレオペで手集めしたものと同じ形式になる。
エピソードごとに回り込む角度・押し終わりの角度・押す速さ・手先の高さを少しずつ変え、
毎回2個のブロックを開始位置へ戻してから記録する。

  使い方: auto_demo_collect.py [-n 20] [--task collect_blocks] [--seed 0]
  前提  : 別ターミナルで collect_task.launch.py が起動していること
"""
import argparse
import os
import random
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from collect_motion import (ARM_JOINT_NAMES, GRIPPER_JOINT_NAME, GRIPPER_CLOSED,
                            RATE_HZ, UPRIGHT, collect_trajectory)

TOPICS = ['/joint_states',
          '/crane_plus_arm_controller/joint_trajectory',
          '/crane_plus_gripper_controller/joint_trajectory']
HERE = os.path.dirname(os.path.abspath(__file__))

# 目標エリア（collect_task.sdf の goal_zone と同じ）
GOAL_X, GOAL_Y = 0.1372, 0.0
HALF_X, HALF_Y = 0.0375, 0.0675


def in_goal(x, y):
    return abs(x - GOAL_X) <= HALF_X and abs(y - GOAL_Y) <= HALF_Y


class DemoPublisher(Node):
    def __init__(self):
        super().__init__('pai_auto_demo_collect')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/crane_plus_arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.joints = None

    def _on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        self.joints = [d.get(j, 0.0) for j in ARM_JOINT_NAMES]

    def send(self, arm_pos, gripper_pos=GRIPPER_CLOSED):
        for pub, names, pos in ((self.arm_pub, ARM_JOINT_NAMES, arm_pos),
                                (self.gripper_pub, [GRIPPER_JOINT_NAME], [gripper_pos])):
            jt = JointTrajectory()
            jt.joint_names = names
            pt = JointTrajectoryPoint()
            pt.positions = [float(p) for p in pos]
            pt.time_from_start.nanosec = int(0.1 * 1e9)
            jt.points.append(pt)
            pub.publish(jt)

    def hold(self, arm_pos, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(arm_pos)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(1.0 / RATE_HZ)

    def settle_upright(self, tol=0.03, timeout=25.0):
        """実際に直立へ到達するまで指令を出し続ける。

        指令を出すだけで次に進むと、関節が戻りきる前に記録が始まってしまう。
        とくに joint4 は可動域の端(-1.84rad)から戻るのに時間がかかり、
        エピソードの開始姿勢が推論時の初期姿勢と食い違う原因になる。
        """
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.send(UPRIGHT)
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.joints is not None and max(abs(v) for v in self.joints) < tol:
                # 到達後も少し保持して静止させる
                self.hold(UPRIGHT, 0.5)
                return True
            time.sleep(1.0 / RATE_HZ)
        q = self.joints
        shown = 'joint_states 未受信' if q is None else             ' '.join(f'{n.replace("crane_plus_", "")}={v:+.3f}'
                     for n, v in zip(ARM_JOINT_NAMES, q))
        print(f'  [warn] 直立への復帰がタイムアウト: {shown}', flush=True)
        return False


def reset_blocks():
    subprocess.run(['bash', os.path.join(HERE, 'reset_blocks.sh')],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def blocks_xy():
    r = subprocess.run(['bash', os.path.join(HERE, 'blocks_pose.sh')],
                       capture_output=True, text=True, check=False)
    try:
        v = [float(t) for t in r.stdout.split()]
        return v[0], v[1], v[2], v[3]
    except Exception:
        return (float('nan'),) * 4


def next_episode_dir(base):
    n = 1
    while os.path.exists(os.path.join(base, f'ep{n}')):
        n += 1
    return os.path.join(base, f'ep{n}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-n', '--episodes', type=int, default=20)
    ap.add_argument('--task', default='collect_blocks')
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    base = os.path.expanduser(f'~/dataset/rosbags/{args.task}')
    os.makedirs(base, exist_ok=True)

    rclpy.init()
    node = DemoPublisher()
    ok = 0
    try:
        for ep in range(args.episodes):
            node.settle_upright()
            reset_blocks()
            node.settle_upright()

            params = dict(
                yaw_red_start=rng.uniform(-0.84, -0.76),
                yaw_red_goal=rng.uniform(-0.25, -0.17),
                yaw_blue_start=rng.uniform(0.91, 0.99),
                yaw_blue_goal=rng.uniform(0.60, 0.68),
                h_push=rng.uniform(0.140, 0.145),
                sweep_sec=rng.uniform(2.7, 3.3),
            )
            traj = collect_trajectory(**params)

            out_dir = next_episode_dir(base)
            rec = subprocess.Popen(['ros2', 'bag', 'record', '-o', out_dir, *TOPICS],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.5)
            for pos in traj:
                node.send(pos)
                time.sleep(1.0 / RATE_HZ)
            rec.send_signal(signal.SIGINT)
            rec.wait(timeout=20)

            rx, ry, bx, by = blocks_xy()
            hit_r, hit_b = in_goal(rx, ry), in_goal(bx, by)
            hit = hit_r and hit_b
            ok += int(hit)
            print(f'[{ep + 1}/{args.episodes}] {os.path.basename(out_dir)}  '
                  f'red=({rx:+.4f},{ry:+.4f}){"OK" if hit_r else "NG"}  '
                  f'blue=({bx:+.4f},{by:+.4f}){"OK" if hit_b else "NG"}  '
                  f'=> {"SUCCESS" if hit else "FAIL"}', flush=True)
        print(f'\n成功 {ok}/{args.episodes} エピソード  保存先: {base}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


main()
