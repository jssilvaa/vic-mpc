#!/usr/bin/env bash
# Configure + build vic-mpc.
#
# Usage:
#   ./build.sh                  # configure (if needed) and build
#   ./build.sh --clean          # wipe build/ and reconfigure from scratch
#   ./build.sh --reconfigure    # delete CMakeCache only, keep _deps (fast reset)
#   ./build.sh -t 02_viewer     # build a single target
#   ./build.sh --debug          # configure as Debug instead of Release
#
# Env overrides:
#   MUJOCO_DIR=/path/to/mujoco-3.8.0  use prebuilt MuJoCo (fast)
#                                     unset => FETCH_MUJOCO=ON (build from source)

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$ROOT/build"
BUILD_TYPE="Release"
TARGET=""
CLEAN=0
RECONFIGURE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean)        CLEAN=1; shift ;;
    --reconfigure)  RECONFIGURE=1; shift ;;
    --debug)        BUILD_TYPE="Debug"; shift ;;
    --release)      BUILD_TYPE="Release"; shift ;;
    -t|--target)    TARGET="$2"; shift 2 ;;
    -h|--help)      sed -n '2,15p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [[ $CLEAN -eq 1 ]]; then
  echo "[build.sh] wiping $BUILD_DIR"
  rm -rf "$BUILD_DIR"
fi

if [[ $RECONFIGURE -eq 1 ]]; then
  echo "[build.sh] removing CMakeCache to force reconfigure"
  rm -f "$BUILD_DIR/CMakeCache.txt"
  rm -rf "$BUILD_DIR/CMakeFiles"
fi

CMAKE_ARGS=(
  -S "$ROOT"
  -B "$BUILD_DIR"
  -G Ninja
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
)

if [[ -n "${MUJOCO_DIR:-}" ]]; then
  echo "[build.sh] using prebuilt MuJoCo at $MUJOCO_DIR"
  CMAKE_ARGS+=(-DMUJOCO_DIR="$MUJOCO_DIR" -DFETCH_MUJOCO=OFF)
else
  echo "[build.sh] MUJOCO_DIR unset — falling back to FETCH_MUJOCO=ON"
  CMAKE_ARGS+=(-DFETCH_MUJOCO=ON)
fi

# If a previous configure used a different generator (e.g. Make), wipe the
# CMake state so we can switch to Ninja cleanly. Keeps _deps/ to avoid
# re-downloading MuJoCo sources.
if [[ -f "$BUILD_DIR/CMakeCache.txt" && ! -f "$BUILD_DIR/build.ninja" ]]; then
  echo "[build.sh] generator changed — clearing CMake cache (keeping _deps sources)"
  rm -f "$BUILD_DIR/CMakeCache.txt" "$BUILD_DIR/Makefile" "$BUILD_DIR/cmake_install.cmake"
  rm -rf "$BUILD_DIR/CMakeFiles"
  # FetchContent stores the previous generator in *-subbuild/. Sources in
  # *-src/ and *-build/ stay so we don't re-download MuJoCo.
  find "$BUILD_DIR/_deps" -maxdepth 1 -type d -name '*-subbuild' -exec rm -rf {} + 2>/dev/null || true
fi

if [[ ! -f "$BUILD_DIR/build.ninja" ]]; then
  cmake "${CMAKE_ARGS[@]}"
fi

if [[ -n "$TARGET" ]]; then
  cmake --build "$BUILD_DIR" --target "$TARGET"
else
  cmake --build "$BUILD_DIR"
fi

echo "[build.sh] done — binaries in $BUILD_DIR/"
