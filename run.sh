#!/usr/bin/env bash
# viam-server launches this. First run creates the venv; on Debian/Ubuntu that needs the
# python3-venv package, which is installed here if missing (viam-server runs as root).
set -euo pipefail
cd "$(dirname "$0")"
if [ ! -x .venv/bin/python ]; then
  rm -rf .venv
  if ! python3 -m venv .venv; then
    rm -rf .venv
    apt-get install -y python3-venv
    python3 -m venv .venv
  fi
  .venv/bin/pip install -q -r requirements.txt
fi
exec .venv/bin/python src/main.py "$@"
