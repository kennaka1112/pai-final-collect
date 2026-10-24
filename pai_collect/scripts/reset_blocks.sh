#!/bin/bash
# 2個のブロックを開始位置（作業半径0.14m, 方位 ±31.5deg）へ戻す。
set -e
source /opt/ros/humble/setup.bash
export DISPLAY=:0
set_pose() {   # name x y yaw
  local qz qw
  qz=$(python3 -c "import math,sys; print(math.sin(float(sys.argv[1])/2))" "$4")
  qw=$(python3 -c "import math,sys; print(math.cos(float(sys.argv[1])/2))" "$4")
  ign service -s /world/default/set_pose \
    --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 3000 \
    --req "name: \"$1\", position: {x: $2, y: $3, z: 1.0351}, orientation: {x: 0, y: 0, z: $qz, w: $qw}" \
    > /dev/null
}
set_pose block_red  0.1194 -0.0732 -0.55
set_pose block_blue 0.1194  0.0732  0.55
