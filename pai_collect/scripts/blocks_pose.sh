#!/bin/bash
# 2個のブロックの現在位置を「赤x 赤y 青x 青y」の1行で表示する。
# ign model は稀に応答を取りこぼすため、取得できるまで数回試す。
source /opt/ros/humble/setup.bash
export DISPLAY=:0
get_xy() {
  local m="$1" line
  for _ in 1 2 3 4 5; do
    line=$(ign model -m "$m" -p 2>&1 | grep -E '^\s+\[' | head -1 | tr -d '[]')
    if [ -n "$line" ]; then
      echo "$line" | awk '{print $1, $2}'
      return 0
    fi
    sleep 0.5
  done
  echo "nan nan"
}
echo "$(get_xy block_red) $(get_xy block_blue)"
