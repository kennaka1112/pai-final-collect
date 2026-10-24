#!/bin/bash
# 提出動画の素材を撮影する。
#   clip1: 自動デモによるデータ収集（1エピソード）
#   clip2: 学習済みACT方策による実行（2サイクルを連結）
# 出力: /workspace/video_collect/raw/collect.mp4, policy.mp4
set -e
source /opt/ros/humble/setup.bash
source ~/airobot_ws/install/setup.bash
export DISPLAY=:0
HERE="$(cd "$(dirname "$0")" && pwd)"
RAW=/workspace/video_collect/raw
mkdir -p "$RAW" "$RAW/parts"
bash "$HERE/prepare_display.sh" >/dev/null

echo "=== clip1: データ収集 ==="
pkill -f "scripts/infer.py" >/dev/null 2>&1 || true
python3 "$HERE/goto_upright.py" 4
bash "$HERE/reset_blocks.sh"
sleep 1
nohup python3 "$HERE/auto_demo_collect.py" -n 1 --task video_demo_collect --seed 5 > /tmp/collect_clip.log 2>&1 &
sleep 4
bash "$HERE/record_screen.sh" "$RAW/collect.mp4" 34
wait || true

echo "=== clip2: 方策の実行（2サイクル） ==="
rm -f "$RAW"/parts/policy_*.mp4
for i in 1 2; do
  pkill -f "scripts/infer.py" >/dev/null 2>&1 || true
  sleep 1
  python3 "$HERE/goto_upright.py" 4
  bash "$HERE/reset_blocks.sh"
  python3 "$HERE/goto_upright.py" 2
  : > /tmp/infer.log
  nohup bash "$HERE/collect-infer" > /tmp/infer.log 2>&1 &
  # 方策が指令を出し始めるまで待ってから録画（モデル読み込みの待ち時間を映さない）
  for _ in $(seq 1 180); do
    grep -q '^action:' /tmp/infer.log && break
    sleep 0.5
  done
  bash "$HERE/record_screen.sh" "$RAW/parts/policy_$i.mp4" 26
  pkill -f "scripts/infer.py" >/dev/null 2>&1 || true
  echo "  cycle $i: $(bash "$HERE/blocks_pose.sh")"
done
: > "$RAW/parts/list.txt"
for i in 1 2; do echo "file '$RAW/parts/policy_$i.mp4'" >> "$RAW/parts/list.txt"; done
ffmpeg -loglevel error -y -f concat -safe 0 -i "$RAW/parts/list.txt" -c copy "$RAW/policy.mp4"
echo "done:"
ls -la "$RAW"
