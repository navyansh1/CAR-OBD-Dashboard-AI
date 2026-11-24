#!/bin/zsh
# Start OBD Live: the Bluetooth bridge (if not running) and the dashboard server.
cd "${0:A:h}"
if [[ "$1" == "--replay" ]]; then
  # No car needed: play back the recorded drives in sample_data/
  pkill -f "obd_dashboard.py" 2>/dev/null; sleep 1
  echo "Replay dashboard: http://127.0.0.1:8327"
  (sleep 2; open "http://127.0.0.1:8327") &
  exec .venv/bin/python -u obd_dashboard.py "$@"
fi
if ! pgrep -f "bt_bridge.py" >/dev/null; then
  echo "Starting OBD Bridge.app (Bluetooth link to the ELM327)…"
  open "OBD Bridge.app"
fi
pkill -f "obd_dashboard.py" 2>/dev/null
sleep 1
echo "Dashboard: http://127.0.0.1:8327"
# keep the Mac awake (display on) for as long as the dashboard runs; lid must stay open
(sleep 3; caffeinate -dimsu -w "$(pgrep -f obd_dashboard.py | head -1)") &
(sleep 2; open "http://127.0.0.1:8327") &
exec .venv/bin/python -u obd_dashboard.py "$@"
