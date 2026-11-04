#!/bin/bash
# Wait until N status files exist, KSP exits, or a SimHarness exception is logged.
K=${KSP_HARNESS:-$HOME/ksp-harness/KSP}; n=${1:-1}
until [ "$(ls $K/SimHarnessLogs/*.status 2>/dev/null | wc -l)" -ge "$n" ] || ! kill -0 "$(cat $K/../ksp.pid)" 2>/dev/null \
   || /usr/bin/grep -aq 'at SimHarness\|at FlightDriver.setStartupNewVessel' "$K/KSP_player.log"; do sleep 3; done
ls -la $K/SimHarnessLogs; for f in $K/SimHarnessLogs/*.status; do echo "$f: $(cat $f)"; done
/usr/bin/grep -a -A3 'at SimHarness\|setStartupNewVessel' "$K/KSP_player.log" | head -12
