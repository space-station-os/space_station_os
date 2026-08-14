#!/usr/bin/env bash
# Install the Space Station OS desktop app (Tier 1: launcher-based).
#
# Places a single trusted .desktop shortcut on ~/Desktop that runs the repo's
# desktop/launch.sh (which cd's into this folder and runs `pixi run station`).
# Everything else (launcher, icon, pixi env) stays inside the space_station_os
# folder. Shows a zenity progress popup; falls back to plain text when headless.
set -uo pipefail

SCRIPT="$(readlink -f "$0")"
DESKTOP_SRC="$(cd "$(dirname "$SCRIPT")" && pwd)"
REPO_DIR="$(dirname "$DESKTOP_SRC")"        # the space_station_os folder
LOG="$DESKTOP_SRC/install.log"
: >"$LOG"

export PATH="$HOME/.pixi/bin:$PATH"

DESKTOP_DIR="${XDG_DESKTOP_DIR:-$HOME/Desktop}"
APP="$DESKTOP_DIR/SpaceStationOS.desktop"
ICON="$REPO_DIR/assets/logo/ssosapplogo.png"
LAUNCHER="$DESKTOP_SRC/launch.sh"

have_gui() { [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; }
use_zenity() { command -v zenity >/dev/null 2>&1 && have_gui; }

fail() {
  echo "ERROR: $1" | tee -a "$LOG" >&2
  use_zenity && zenity --error --title="Install Space Station OS" \
    --text="$1

See $LOG" 2>/dev/null || true
  exit 1
}

command -v pixi >/dev/null 2>&1 || \
  fail "pixi not found. Install pixi from https://pixi.sh, then re-run this installer."
mkdir -p "$DESKTOP_DIR"

# Emits progress (percent + '# label') on stdout; verbose output goes to LOG.
run_steps() {
  echo 10;  echo "# Checking environment..."
  echo 25;  echo "# Fetching ROS 2 environment (pixi install --locked)..."
  ( cd "$REPO_DIR" && pixi install --locked ) >>"$LOG" 2>&1 || return 1
  echo 65;  echo "# Building Space Station OS (this can take a minute)..."
  ( cd "$REPO_DIR" && pixi run build ) >>"$LOG" 2>&1 || return 1
  echo 85;  echo "# Installing desktop shortcut..."
  chmod +x "$LAUNCHER" || return 1
  sed -e "s|@LAUNCHER@|$LAUNCHER|g" -e "s|@ICON@|$ICON|g" \
      "$DESKTOP_SRC/space-station-os.desktop.in" > "$APP" || return 1
  chmod +x "$APP" || return 1
  # GNOME requires the shortcut to be trusted for double-click launch.
  gio set "$APP" metadata::trusted true >>"$LOG" 2>&1 || true
  echo 100; echo "# Done."
}

if use_zenity; then
  run_steps | zenity --progress --title="Install Space Station OS" \
      --text="Starting..." --percentage=0 --no-cancel --auto-close 2>/dev/null
  STATUS=${PIPESTATUS[0]}
else
  run_steps | while read -r line; do
    case "$line" in \#*) echo "  ${line#\# }" ;; esac
  done
  STATUS=${PIPESTATUS[0]}
fi

[ "$STATUS" -eq 0 ] || fail "Installation failed."

MSG="Space Station OS installed.
Double-click the Space Station OS icon on your Desktop to launch."
echo "$MSG"
use_zenity && zenity --info --title="Space Station OS" --text="$MSG" 2>/dev/null || true
