#!/bin/bash
# Fly RTLSDemo.craft in the demo COPY of KSP (~/ksp-demo/KSP) with the ksp-sim guidance over the
# bridge (guidance/core/examples/bridge.rs <-> ingame/plugin/Bridge.cs). Waits for any other KSP
# process (e.g. the fidelity harness) to exit first; never kills one.
# usage: ingame/run_bridge.sh [width height]
#   BRIDGE_HOLD=1  pad-only plumbing check (engine never lit, quits after 20 s)
#   RECORD=out     also record the KSP window (ffmpeg x11grab) to out.raw.mkv
set -u
K=${KSP_DEMO:-$HOME/ksp-demo/KSP}
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(dirname "$HERE")
W=${1:-1280}; H=${2:-720}
export KSP_DIR=$K
(cd "$HERE/plugin" && ~/.dotnet/dotnet build -c Release -v q 2>&1 | grep -E "error|Build succeeded") || exit 1
(cd "$REPO/guidance" && nice -n 10 cargo build -q --release --example bridge) || exit 1
cp "$HERE/plugin/bin/Release/net472/DemoHarness.dll" "$K/GameData/DemoHarness/Plugins/"
rm -f "$K/GameData/DemoHarness/demo_kos.cfg"
"$REPO/.venv/bin/python" -I "$HERE/make_demo_craft.py" "$K/Ships/VAB" > /dev/null
while pgrep -x KSP.x86_64 >/dev/null; do echo "another KSP is running, waiting..."; sleep 20; done
stamp=$(date +%Y%m%d_%H%M%S)
mkdir -p "$HERE/telemetry"
export BRIDGE_LOG="$HERE/telemetry/bridge_${stamp}.csv"
"$REPO/guidance/target/release/examples/bridge" > "$HERE/telemetry/bridge_${stamp}.log" 2>&1 &
bpid=$!
echo RTLSDemo.craft > "$K/GameData/DemoHarness/launch.txt"
rm -f "$K/KSP_player.log"
sed -i "s/^SCREEN_RESOLUTION_WIDTH = .*/SCREEN_RESOLUTION_WIDTH = $W/; s/^SCREEN_RESOLUTION_HEIGHT = .*/SCREEN_RESOLUTION_HEIGHT = $H/" "$K/settings.cfg"
export DISPLAY=:1 XAUTHORITY=$(ls /run/user/$(id -u)/xauth_* | head -1)
cd "$K"
./KSP.x86_64 -screen-fullscreen 0 -screen-width "$W" -screen-height "$H" ${KSP_ARGS:-} -logFile "$K/KSP_player.log" > "$K/../launch.out" 2>&1 &
pid=$!
echo $pid > "$K/../ksp.pid"
echo "KSP pid $pid, bridge pid $bpid, log $BRIDGE_LOG"
if [ -n "${RECORD:-}" ]; then
  until grep -aq "bridge: flying" "$K/KSP_player.log" 2>/dev/null || ! kill -0 $pid 2>/dev/null; do sleep 0.5; done
  wid=$(xdotool search --pid $pid 2>/dev/null | tail -1)
  echo "recording window $wid ($(xwininfo -id "$wid" | grep -E 'Width|Height' | tr -s ' ' | tr '\n' ' ')) -> $RECORD.raw.mkv"
  # Xwayland: the root window reads back black; grab the KSP window itself
  nice -n 5 ffmpeg -y -loglevel error -f x11grab -window_id "$wid" -draw_mouse 0 -framerate 30 -i "$DISPLAY" \
    -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p "$RECORD.raw.mkv" &
  fpid=$!
fi
wait $pid
echo "KSP exited $?"
[ -n "${fpid:-}" ] && { kill -INT $fpid 2>/dev/null; wait $fpid; }
kill $bpid 2>/dev/null
cp "$K"/SimHarnessLogs/flight_* "$HERE/telemetry/" 2>/dev/null
tail -5 "$HERE/telemetry/bridge_${stamp}.log"
grep -a "\[DemoHarness\] bridge" "$K/KSP_player.log" | tail -8
