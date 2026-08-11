#!/bin/bash
# Restarts dsd-fme-scanner.service if it goes quiet without actually crashing
# (RTL-SDR USB dropouts leave the process alive but hung on a stale device handle,
# so systemd's own Restart=on-failure never triggers).
set -euo pipefail

SERVICE="dsd-fme-scanner.service"
WATCHDOG_UNIT="dsd-fme-watchdog.service"
STALE_THRESHOLD=900      # seconds of silence considered "hung"
MAX_RESTARTS_PER_HOUR=3

last_ts=$(journalctl -u "$SERVICE" -n 1 -o short-unix --no-pager 2>/dev/null | awk '{print $1}' | cut -d. -f1)

if [[ -z "${last_ts:-}" ]]; then
    echo "WATCHDOG: no journal entries found for $SERVICE, skipping check"
    exit 0
fi

now=$(date +%s)
age=$(( now - last_ts ))

if (( age < STALE_THRESHOLD )); then
    exit 0
fi

recent_restarts=$(journalctl -u "$WATCHDOG_UNIT" --since "-60min" --no-pager 2>/dev/null | grep -c "restarting $SERVICE" || true)

if (( recent_restarts >= MAX_RESTARTS_PER_HOUR )); then
    echo "WATCHDOG: $SERVICE silent for ${age}s but already restarted $recent_restarts times in the last hour, giving up, needs manual attention"
    exit 1
fi

echo "WATCHDOG: $SERVICE silent for ${age}s (threshold ${STALE_THRESHOLD}s), restarting $SERVICE"
systemctl restart "$SERVICE"
