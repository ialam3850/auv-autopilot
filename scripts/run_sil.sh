#!/usr/bin/env bash
# Build the host code, start the SIL autopilot, and fly the default mission.
#   scripts/run_sil.sh [output-prefix]      (default runs/sil)
# Writes <prefix>.csv and <prefix>.png and prints a summary.
set -euo pipefail
cd "$(dirname "$0")/.."

out=${1:-runs/sil}
port=${PORT:-5760}
mkdir -p "$(dirname "$out")"

cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build build --target sil_runner

./build/sil_runner "$port" --once &
pid=$!
trap 'kill $pid 2>/dev/null || true' EXIT
sleep 0.3  # let it start listening

python3 sim/run_mission.py --link "tcp:127.0.0.1:$port" --out "$out"
