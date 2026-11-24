#!/bin/zsh
# One-time setup: Python environment + build and sign "OBD Bridge.app" (macOS only).
set -e
cd "${0:A:h}"
echo "1/3  Python environment (.venv)"
python3 -m venv .venv
.venv/bin/pip install -q --upgrade pip
.venv/bin/pip install -q -r requirements.txt
echo "2/3  Building OBD Bridge.app (needs Xcode command line tools: xcode-select --install)"
APP="OBD Bridge.app"
rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources"
clang -O2 -o "$APP/Contents/MacOS/obdbridge" bridge/launcher.c
cp bridge/Info.plist "$APP/Contents/Info.plist"
cp bridge/run.sh "$APP/Contents/Resources/run.sh"
chmod +x "$APP/Contents/Resources/run.sh"
codesign --force --deep -s - "$APP"
mkdir -p logs
echo "3/3  Done."
echo
echo "Pair your ELM327 in System Settings › Bluetooth (it usually shows up as OBDII), then:"
echo "   ./start.sh            live dashboard (first run: allow OBD Bridge to use Bluetooth)"
echo "   ./start.sh --replay   try it without a car, replaying a recorded drive"
