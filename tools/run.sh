#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"
if [ ! -d ".venv" ]; then
  echo "Missing .venv. Run ./setup_venv.sh first." >&2
  exit 1
fi

source .venv/bin/activate
python receiver_app.py "$@"
