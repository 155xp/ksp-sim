#!/bin/sh
# (Re)start the viewer as a plain background process on 0.0.0.0:PORT (default 8787).
#   ./run.sh [port]        build first with: cargo build --release
cd "$(dirname "$0")" || exit 1
PORT=${1:-8787}
[ -f viewer.pid ] && kill "$(cat viewer.pid)" 2>/dev/null && sleep 0.5
nohup nice -n 10 ./target/release/ksp-viewer --port "$PORT" > viewer.log 2>&1 &
echo $! > viewer.pid
sleep 0.5
cat viewer.log
