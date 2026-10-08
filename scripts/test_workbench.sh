#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# Additional arguments are CMake configuration options, e.g. -DUAM_REQUIRE_ALGORITHMS=ON.
cmake -S "$REPO_ROOT/src/uam_core" -B "$REPO_ROOT/build/workbench"   -DCMAKE_BUILD_TYPE=Debug -DUAM_REQUIRE_ALGORITHMS=OFF "$@"
cmake --build "$REPO_ROOT/build/workbench"
ctest --test-dir "$REPO_ROOT/build/workbench" --output-on-failure
printf '\nSkipped acceptance tests mean unfinished algorithms, not flight readiness.\n'
