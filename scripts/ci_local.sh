#!/usr/bin/env bash
# Runs the same checks as .github/workflows/ci.yml on this machine.
set -euo pipefail
cd "$(dirname "$0")/.."
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
cmake -S . -B build-asan -G Ninja -DCMAKE_BUILD_TYPE=Debug -DSBW_SANITIZE=ON
cmake --build build-asan
ctest --test-dir build-asan --output-on-failure
echo "ci_local: all checks passed"
