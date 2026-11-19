#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release "$@"
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure --timeout 30
(cd sim && python3 -m unittest discover -s tests -v)
