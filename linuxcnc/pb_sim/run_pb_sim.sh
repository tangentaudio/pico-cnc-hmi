#!/usr/bin/env bash
# Launch LinuxCNC with the Probe Basic GUI and HMI bridge.
# Probe Basic (QtPyVCP) must be installed in ~/dev/venv.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INI_FILE="$SCRIPT_DIR/pb_sim.ini"
RIP_ENV="/home/steve/devel/linuxcnc/scripts/rip-environment"
VENV_BIN="/home/steve/dev/venv/bin"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: $0"
  echo "Launch LinuxCNC with Probe Basic GUI and HMI bridge (3-axis XYZ sim)."
  exit 0
fi

if [[ ! -f "$INI_FILE" ]]; then
  echo "Missing INI file: $INI_FILE" >&2
  exit 1
fi

# Activate the LinuxCNC run-in-place environment if linuxcnc isn't on PATH already.
if ! command -v linuxcnc >/dev/null 2>&1; then
  if [[ -f "$RIP_ENV" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$RIP_ENV"
    set -u
  fi
fi

if ! command -v linuxcnc >/dev/null 2>&1; then
  echo "linuxcnc command not found. Source RIP environment first:" >&2
  echo "  source $RIP_ENV" >&2
  exit 1
fi

# Prepend the venv bin directory so that 'probe_basic' is found by LinuxCNC
# when it spawns the display process (DISPLAY = probe_basic in the INI).
export PATH="$VENV_BIN:$PATH"

# Verify probe_basic entry point is available.
if ! command -v probe_basic >/dev/null 2>&1; then
  echo "probe_basic not found in $VENV_BIN" >&2
  echo "Is the venv installed?  Try: pip show probe_basic in ~/dev/venv" >&2
  exit 1
fi

echo "Using probe_basic: $(command -v probe_basic)"
exec linuxcnc "$INI_FILE"
