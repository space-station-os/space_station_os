#!/usr/bin/env bash
# Space Station OS desktop launcher.
#
# Invoked by the ~/Desktop shortcut. A desktop double-click runs from an
# arbitrary working directory, so this script resolves and cd's into the
# space_station_os folder (where pixi.toml lives) before running
# `pixi run station`. Keep this script inside the repo (<repo>/desktop/).
set -uo pipefail

SCRIPT="$(readlink -f "$0")"
DESKTOP_DIR="$(cd "$(dirname "$SCRIPT")" && pwd)"
REPO_DIR="$(dirname "$DESKTOP_DIR")"        # the space_station_os folder
LOG="$DESKTOP_DIR/last-run.log"

# pixi installs to ~/.pixi/bin by default; a GUI launch does not source the
# shell profile, so put it on PATH explicitly.
export PATH="$HOME/.pixi/bin:$PATH"

have_gui() { [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; }
err() {
  echo "ERROR: $1" >&2
  if command -v zenity >/dev/null 2>&1 && have_gui; then
    zenity --error --title="Space Station OS" --text="$1" 2>/dev/null || true
  fi
}

command -v pixi >/dev/null 2>&1 || {
  err "pixi was not found on PATH.\nInstall pixi (https://pixi.sh) or run desktop/install.sh."
  exit 1
}

cd "$REPO_DIR" || { err "Cannot enter $REPO_DIR"; exit 1; }

# Launch the full stack (GUI + core + sim + eclss); keep logs for troubleshooting.
pixi run station >"$LOG" 2>&1 &
STATION=$!

# Pulsating splash that closes once the GUI window appears (or the stack exits,
# or a safety timeout). No-op when headless or when zenity is unavailable.
if command -v zenity >/dev/null 2>&1 && have_gui; then
  (
    for _ in $(seq 1 120); do
      pgrep -f space_station_gui_node >/dev/null 2>&1 && break
      kill -0 "$STATION" 2>/dev/null || break
      sleep 1
    done
  ) | zenity --progress --pulsate --no-cancel --auto-close \
        --title="Space Station OS" \
        --text="Launching Space Station OS...
Building and starting subsystems." 2>/dev/null || true
fi

wait "$STATION"
STATUS=$?
[ "$STATUS" -eq 0 ] || err "Space Station OS exited with status $STATUS. See $LOG"
exit "$STATUS"
