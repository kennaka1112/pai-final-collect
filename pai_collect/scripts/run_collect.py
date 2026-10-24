#!/usr/bin/env python3
"""集めるデモを1回だけ実行して動きを確認する（動作確認用）。"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from collect_motion import (ARM_JOINT_NAMES, GRIPPER_JOINT_NAME, GRIPPER_CLOSED,
                            RATE_HZ, UPRIGHT, collect_trajectory)


class Arm(Node):
    def __init__(self):
        super().__init__('pai_collect_runner')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.arm = self.create_publisher(
            JointTrajectory, '/crane_plus_arm_controller/joint_trajectory', 10)
        self.grip = self.create_publisher(
            JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', 10)

    def send(self, arm_pos, gripper=GRIPPER_CLOSED):
        for pub, names, pos in ((self.arm, ARM_JOINT_NAMES, arm_pos),
                                (self.grip, [GRIPPER_JOINT_NAME], [gripper])):
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
            time.sleep(1.0 / RATE_HZ)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaw-red-start', type=float, default=-0.80)
    ap.add_argument('--yaw-red-goal', type=float, default=-0.21)
    ap.add_argument('--yaw-blue-start', type=float, default=0.95)
    ap.add_argument('--yaw-blue-goal', type=float, default=0.64)
    ap.add_argument('--h-push', type=float, default=0.142)
    ap.add_argument('--sweep-sec', type=float, default=3.0)
    args = ap.parse_args()

    rclpy.init()
    node = Arm()
    try:
        node.hold(UPRIGHT, 1.5)
        traj = collect_trajectory(
            yaw_red_start=args.yaw_red_start, yaw_red_goal=args.yaw_red_goal,
            yaw_blue_start=args.yaw_blue_start, yaw_blue_goal=args.yaw_blue_goal,
            h_push=args.h_push, sweep_sec=args.sweep_sec)
        print(f'steps={len(traj)}  ({len(traj)/RATE_HZ:.1f}s)', flush=True)
        for pos in traj:
            node.send(pos)
            time.sleep(1.0 / RATE_HZ)
    finally:
        node.destroy_node()
        rclpy.shutdown()


main()
