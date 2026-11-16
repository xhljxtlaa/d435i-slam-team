#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
JOBS="${CERES_BUILD_JOBS:-2}"
[[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || { echo 'CERES_BUILD_JOBS 必须为正整数'; exit 1; }
SRC="$ROOT/.deps/ceres-src"
mkdir -p "$ROOT/.deps"
if [[ ! -d "$SRC/.git" ]]; then
  C_SHA="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["repositories"]["ceres"]["commit"])' "$ROOT/dependencies.lock.json")"
  git init "$SRC"
  git -C "$SRC" remote add origin https://ceres-solver.googlesource.com/ceres-solver
  git -C "$SRC" fetch --depth 1 origin "$C_SHA"
  git -C "$SRC" checkout --detach FETCH_HEAD
fi
cmake -S "$SRC" -B "$ROOT/.deps/ceres-build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$ROOT/.deps/ceres" -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
  -DBUILD_SHARED_LIBS=ON
cmake --build "$ROOT/.deps/ceres-build" --parallel "$JOBS"
cmake --install "$ROOT/.deps/ceres-build"
