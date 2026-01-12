#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
TARGET_DIR="$SCRIPT_DIR/../src"

black "$TARGET_DIR" --safe  --line-length=79 --target-version=py312
