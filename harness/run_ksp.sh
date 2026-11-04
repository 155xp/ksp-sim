#!/bin/bash
# Launch the harness COPY of KSP on the desktop session, run the queue, wait for it to quit.
# usage: harness/run_ksp.sh "craft test maxsec" ...   (each arg one queue line)
set -u
K=${KSP_HARNESS:-$HOME/ksp-harness/KSP}
printf '%s\n' "$@" > "$K/GameData/SimHarness/queue.txt"
echo 0 > "$K/GameData/SimHarness/progress.txt"
rm -f "$K/SimHarnessLogs/ALL_DONE"
export DISPLAY=:1 XAUTHORITY=$(ls /run/user/$(id -u)/xauth_* | head -1)
cd "$K"
./KSP.x86_64 -screen-fullscreen 0 -screen-width 1280 -screen-height 720 -logFile "$K/KSP_player.log" > "$K/../launch.out" 2>&1 &
pid=$!
echo $pid > "$K/../ksp.pid"
echo "KSP pid $pid"
wait $pid
echo "KSP exited $?"
ls -la "$K/SimHarnessLogs"
