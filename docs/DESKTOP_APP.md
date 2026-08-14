# Space Station OS desktop app (Tier 1)

A click-to-launch desktop app for Ubuntu. It installs a single shortcut on your
Desktop; double-clicking it launches the full mission-control stack (GUI + core
+ sim + eclss) via `pixi run station`.

This is the **Tier 1** launcher-based app: the shortcut runs the repo's
`pixi run station` in the background. It is not a self-contained binary yet (a
packed, offline binary is a later Tier 2 step). It depends on this checkout and
on [pixi](https://pixi.sh).

## What lives where

Only the shortcut goes on the Desktop; everything else stays in the repo:

| Item | Location |
|------|----------|
| Desktop shortcut | `~/Desktop/SpaceStationOS.desktop` |
| Launcher | `<repo>/desktop/launch.sh` |
| Icon | `<repo>/assets/logo/ssosapplogo.png` |
| pixi environment | `<repo>/.pixi/` (created by `pixi install --locked`) |

The launcher `cd`s into the `space_station_os` folder before running
`pixi run station`, because a Desktop double-click starts from an arbitrary
directory.

## Install

On a clean Ubuntu 24.04 Desktop installation:

```bash
sudo apt update
sudo apt install -y curl git zenity

curl -fsSL https://pixi.sh/install.sh | sh
export PATH="$HOME/.pixi/bin:$PATH"

git clone https://github.com/space-station-os/space_station_os.git
cd space_station_os

bash desktop/install.sh
```

The installer runs `pixi install --locked`, builds SSOS, and creates the
desktop shortcut.

Pixi is installed into `~/.pixi/bin`. A new terminal will normally pick up the
updated `PATH` automatically.

During installation, the progress popup reports environment setup, build, and
desktop-shortcut creation. When installation finishes, a confirmation popup
instructs the user to double-click the Space Station OS icon.

On Ubuntu Desktop, `zenity` provides the installation progress popups. On a
headless machine, the installer prints progress to the terminal instead.

## Launch

Double-click **Space Station OS** on your Desktop. A brief "Launching..." splash
shows while the stack builds/starts, then the GUI opens. Run logs:
`<repo>/desktop/last-run.log`.

## Uninstall

```bash
bash desktop/uninstall.sh
```

Removes the Desktop shortcut only; the repo and pixi environment stay intact.

## Updating the app (for developers)

The Tier 1 app is a thin launcher that points at this checkout, and the
`station` pixi task rebuilds before launching (`depends-on = ["build"]`). So for
most changes there is nothing special to do:

| You changed... | To pick it up |
|----------------|---------------|
| Node / GUI **code** | `git pull` (or edit) — the next double-click rebuilds and runs the latest. Run `pixi run build` first if you want to catch build errors before launching. |
| **Dependencies** in `pixi.toml` | Follow the dependency-update procedure in [PIXI.md](PIXI.md), regenerate `pixi.lock`, then verify with `pixi install --locked`. |
| The **launcher, `.desktop` template, or icon** (`desktop/`, `assets/logo/`) | re-run `bash desktop/install.sh` to refresh the shortcut, icon, and trust flag. |
| The **logo** source | regenerate `assets/logo/ssosapplogo.png` (256×256, see below), then re-run `bash desktop/install.sh`. |

Regenerate the icon from a new logo:

```bash
pixi run python - <<'PY'
from PIL import Image
im = Image.open('assets/logo/ssosapplogo.jpg').convert('RGB')
s = max(im.size)
c = Image.new('RGB', (s, s), (0, 0, 0))
c.paste(im, ((s - im.width) // 2, (s - im.height) // 2))
c.resize((256, 256), Image.LANCZOS).save('assets/logo/ssosapplogo.png')
PY
```

Because the launcher references the repo by absolute path, moving or renaming the
`space_station_os` folder breaks the shortcut — re-run `bash desktop/install.sh`
from the new location to repoint it.

## Troubleshooting

- **"Untrusted application launcher" on double-click.** GNOME requires the
  shortcut to be trusted; the installer runs `gio set ... metadata::trusted true`.
  If it still warns, right-click the icon -> "Allow Launching".
- **Nothing happens / it exits immediately.** Check `desktop/last-run.log`. Most
  often `pixi` is not on `PATH`; the launcher adds `~/.pixi/bin`, but if pixi is
  installed elsewhere, adjust `desktop/launch.sh`.
- **First launch is slow.** The first `pixi run station` builds the workspace.
  The installer pre-builds so this should be quick; a clean `pixi run clean`
  will make the next launch rebuild.
