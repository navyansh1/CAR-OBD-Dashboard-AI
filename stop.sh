#!/bin/zsh
# Stop the dashboard and the Bluetooth bridge.
pkill -f "obd_dashboard.py" 2>/dev/null && echo "dashboard stopped"
pkill -TERM -f "bt_bridge.py" 2>/dev/null && echo "bridge stopped"
