#!/usr/bin/env bash
set -euo pipefail

# Source scrimmage env so the `scrimmage` binary + plugin paths are available to the launcher
if [ -f /root/.scrimmage/setup.bash ]; then
  set +u
  source /root/.scrimmage/setup.bash
  set -u
fi

# Hand off to the Flask launcher (PID 1)
exec python -m launcher.app
