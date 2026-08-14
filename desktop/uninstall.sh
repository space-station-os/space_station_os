#!/usr/bin/env bash
# Remove the Space Station OS desktop shortcut. Leaves the repo and the pixi
# environment intact.
set -uo pipefail

DESKTOP_DIR="${XDG_DESKTOP_DIR:-$HOME/Desktop}"
APP="$DESKTOP_DIR/SpaceStationOS.desktop"

if [ -f "$APP" ]; then
  rm -f "$APP" && echo "Removed $APP"
else
  echo "No shortcut found at $APP"
fi
echo "The pixi environment and repo are untouched (use 'pixi run clean' to drop the build)."
