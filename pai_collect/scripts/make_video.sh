#!/bin/bash
# 提出用の動画を組み立てる。
#   前提: /workspace/video_collect/raw/ に collect.mp4 / policy.mp4 / loss.png があること
#   出力: /workspace/video_collect/PAI_final_collect.mp4  (1280x720 / 30fps)
set -e
RAW=/workspace/video_collect/raw
WORK=/workspace/video_collect/work
OUT="${1:-/workspace/video_collect/PAI_final_collect.mp4}"
F=/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc
FB=/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc
BG=0x14213d
mkdir -p "$WORK"
rm -f "$WORK"/*.mp4 "$WORK"/list.txt

enc() { echo "-c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p -r 30 -an"; }

card() {
  local out="$1" sec="$2" big="$3" small="$4"
  ffmpeg -loglevel error -y -f lavfi -i "color=c=$BG:s=1280x720:d=$sec" \
    -vf "drawtext=fontfile=$FB:text='$big':fontcolor=white:fontsize=50:x=(w-tw)/2:y=300,\
drawtext=fontfile=$F:text='$small':fontcolor=0x9bb1d4:fontsize=30:x=(w-tw)/2:y=390" \
    $(enc) "$out"
}

imgcard() {
  local out="$1" sec="$2" img="$3" cap="$4"
  ffmpeg -loglevel error -y -loop 1 -t "$sec" -i "$img" -f lavfi -t "$sec" -i "color=c=$BG:s=1280x720" \
    -filter_complex "[0:v]scale=1120:-1:force_original_aspect_ratio=decrease[im];\
[1:v][im]overlay=(W-w)/2:60:shortest=1[bg];\
[bg]drawtext=fontfile=$F:text='$cap':fontcolor=white:fontsize=30:x=(w-tw)/2:y=h-70" \
    $(enc) "$out"
}

clip() {
  local out="$1" src="$2" cap="$3" ss="${4:-0}" dur="${5:-}"
  local t=""; [ -n "$dur" ] && t="-t $dur"
  ffmpeg -loglevel error -y -ss "$ss" $t -i "$src" \
    -vf "crop=1120:630:270:85,scale=1280:720,\
drawbox=x=0:y=640:w=1280:h=80:color=black@0.55:t=fill,\
drawtext=fontfile=$F:text='$cap':fontcolor=white:fontsize=29:x=36:y=668" \
    $(enc) "$out"
}

card "$WORK/01_title.mp4" 6 "2個のブロックを中央へ集める" "CRANE+ V2 / Gazebo / ACT 模倣学習"
card "$WORK/02_sec1.mp4" 4 "1. デモデータの自動収集" "左右のブロックを順に押す動作を生成し rosbag2 に記録"
clip "$WORK/03_collect.mp4" "$RAW/collect.mp4" "自動デモ：赤を押す→上を越えて回り込む→青を押す。20エピソード記録" 2 26
card "$WORK/04_sec2.mp4" 4 "2. ACT 方策の学習" "LeRobot / RTX 3060 / 9000 step"
if [ -f "$RAW/loss.png" ]; then
  imgcard "$WORK/05_loss.mp4" 8 "$RAW/loss.png" "学習の損失（9000 step）"
fi
card "$WORK/06_sec3.mp4" 4 "3. 学習した方策で実行" "関節状態のみを入力に、2段構えの動作を再現"
clip "$WORK/07_policy.mp4" "$RAW/policy.mp4" "ACT方策による実行：1個目を押し、回り込んで2個目も中央へ" 0 50
card "$WORK/08_end.mp4" 8 "結果：10試行すべて成功" "2個とも目標エリアに収まった"

for f in "$WORK"/*.mp4; do echo "file '$f'" >> "$WORK/list.txt"; done
ffmpeg -loglevel error -y -f concat -safe 0 -i "$WORK/list.txt" -c copy "$OUT"
echo "saved: $OUT"
ffprobe -v error -show_entries format=duration -of csv=p=0 "$OUT"
