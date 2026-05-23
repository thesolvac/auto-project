# Setup — Raspberry Pi 5 Development Environment

Bring-up steps for the deployment target: **Raspberry Pi 5 (4GB)** running
Raspberry Pi OS / Debian (64-bit). The codebase is Linux-targeted; commands below
assume a Debian shell. Editing is done in VS Code on the Pi, directly or over
Remote-SSH.

> Development may happen on another machine (e.g. Windows) and be `git pull`-ed
> on the Pi to run. The repository itself stays clean Linux/RPi style — no
> Windows-specific paths or shims are committed.

## 1. System packages

```bash
sudo apt update
sudo apt install -y \
    git \
    python3-venv python3-dev build-essential \
    libatlas-base-dev libopenblas-dev \
    v4l-utils i2c-tools
```

What they are for:

| Package | Purpose |
|---|---|
| `git` | clone / pull the repository |
| `python3-venv`, `python3-dev` | virtual environments + C extension headers |
| `build-essential` | compiler toolchain for native wheels |
| `libatlas-base-dev`, `libopenblas-dev` | optimized BLAS for numpy/scipy on ARM |
| `v4l-utils` | inspect the USB camera (`v4l2-ctl --list-devices`) — Phase 7 |
| `i2c-tools` | probe the I2C bus (`i2cdetect`) during hardware bring-up — Phase 7 |

## 2. Clone the repository

```bash
git clone https://github.com/thesolvac/auto-project.git
cd auto-project
```

## 3. Python virtual environment

`requirements.txt` is the source of truth; `.venv/` is git-ignored and recreated
on each machine.

```bash
python3 -m venv rpi/.venv
source rpi/.venv/bin/activate          # Windows dev host: rpi\.venv\Scripts\activate
pip install --upgrade pip
pip install -r rpi/requirements.txt
```

Verify the Python side:

```bash
cd rpi
ruff check src tests
pytest -q
cd ..
```

## 4. ESP32 toolchain (PlatformIO)

PlatformIO Core provides the `pio` CLI. The ESP32 toolchain downloads on first
build.

```bash
pip install platformio          # into the active venv, or pipx for a global install
cd firmware
pio run -e esp32dev             # build target firmware (no board needed)
pio test -e native             # run host-side Unity unit tests
cd ..
```

## 5. Serial port (Phase 7 only)

When the ESP32 is connected over USB it appears as `/dev/ttyUSB0` or
`/dev/ttyACM0`. Grant serial access without `sudo`:

```bash
sudo usermod -aG dialout "$USER"
# log out and back in for the group change to take effect
ls -l /dev/ttyUSB* /dev/ttyACM*
```

## 6. Pre-push check

Run this before every push (see `CLAUDE.md` for the git workflow):

```bash
cd rpi && ruff check src tests && pytest -q && cd ..
cd firmware && pio run -e esp32dev && pio test -e native && cd ..
```

Everything must pass. CI (`.github/workflows/ci.yml`) runs the same checks on
`ubuntu-latest` on every push.
