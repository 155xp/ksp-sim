#!/bin/bash
# Launch the demo COPY of KSP (~/ksp-demo/KSP), fly RTLSDemo.craft with kOS, wait for it to quit.
# Waits for any other KSP process (e.g. the fidelity harness) to exit first; never kills one.
# usage: ingame/run_demo.sh [width height [extra unity args...]]
#   RECORD=out.mp4 also records the KSP window with ffmpeg (x11grab) from liftoff to quit.
set -u
K=${KSP_DEMO:-$HOME/ksp-demo/KSP}
HERE=$(cd "$(dirname "$0")" && pwd)
W=${1:-1280}; H=${2:-720}; shift 2 2>/dev/null; EXTRA=("$@")
while pgrep -x KSP.x86_64 >/dev/null; do echo "another KSP is running, waiting..."; sleep 20; done
mkdir -p "$K/Ships/Script/demo" "$K/Ships/Script/boot"
cp "$HERE/kos/boot/demo.ks" "$K/Ships/Script/boot/"
cp "$HERE"/kos/demo/*.ks "$K/Ships/Script/demo/"
rm -f "$K/Ships/Script/demo/cam.txt" "$K/Ships/Script/demo/result.txt" "$K/Ships/Script/demo/flight.csv"
echo RTLSDemo.craft > "$K/GameData/DemoHarness/launch.txt"
export DISPLAY=:1 XAUTHORITY=$(ls /run/user/$(id -u)/xauth_* | head -1)
cd "$K"
nice -n 5 ./KSP.x86_64 -screen-fullscreen 0 -screen-width "$W" -screen-height "$H" "${EXTRA[@]}" -logFile "$K/KSP_player.log" > "$K/../launch.out" 2>&1 &
pid=$!
echo $pid > "$K/../ksp.pid"
echo "KSP pid $pid"
if [ -n "${RECORD:-}" ]; then
  # start recording once the flight log appears (scene loaded, kOS booted)
  until [ -f "$K/Ships/Script/demo/flight.csv" ] || ! kill -0 $pid 2>/dev/null; do sleep 0.5; done
  wid=$(xdotool search --pid $pid --onlyvisible 2>/dev/null | tail -1)
  geo=$(xwininfo -id "$wid" | awk '/Absolute upper-left X/{x=$4}/Absolute upper-left Y/{y=$4}/Width/{w=$2}/Height/{h=$2}END{print x, y, w, h}')
  read X Y WW HH <<< "$geo"
  echo "recording window $wid at $X,$Y ${WW}x$HH -> $RECORD"
  nice -n 5 ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -framerate 30 -video_size "${WW}x${HH}" -i "$DISPLAY+$X,$Y" \
    -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p "$RECORD.raw.mkv" &
  fpid=$!
fi
wait $pid
echo "KSP exited $?"
[ -n "${fpid:-}" ] && { kill -INT $fpid 2>/dev/null; wait $fpid; }
cat "$K/Ships/Script/demo/result.txt" 2>/dev/null
