#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="/home/steve/devel/pico-cnc-hmi/linuxcnc/hmi_sim"
INI_FILE="$ROOT_DIR/hmi_sim.ini"
RIP_ENV="/home/steve/devel/linuxcnc/scripts/rip-environment"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: $0"
  echo "Launch LinuxCNC using the local HMI simulation config."
  exit 0
fi

if [[ ! -f "$INI_FILE" ]]; then
  echo "Missing INI file: $INI_FILE" >&2
  exit 1
fi

if ! command -v linuxcnc >/dev/null 2>&1; then
  if [[ -f "$RIP_ENV" ]]; then
    # rip-environment expects some variables to be unset/empty in shell startup.
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

exec linuxcnc "$INI_FILE"
