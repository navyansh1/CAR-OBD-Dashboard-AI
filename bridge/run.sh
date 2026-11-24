#!/bin/zsh
# Project folder = the folder that contains "OBD Bridge.app"
DIR="${0:A:h:h:h:h}"
mkdir -p "$DIR/logs"
exec "$DIR/.venv/bin/python" -u "$DIR/bt_bridge.py" >> "$DIR/logs/bridge.log" 2>&1
