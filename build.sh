#!/usr/bin/env bash
# Compile the VentWise sketches without an IDE.
#
# arduino-cli lives under ~/development alongside the Flutter SDK --
# no sudo, no system packages, same convention as the rest of this
# machine. Run ./build.sh to compile everything, or ./build.sh
# masteresp to compile one sketch.
#
# arduino-cli requires each sketch in a folder of the same name, so
# each .ino is copied into a scratch build tree first.
set -euo pipefail

DEV="$HOME/development"
CLI="$DEV/arduino-cli-bin/arduino-cli --config-file $DEV/arduino-cli.yaml"
FQBN="esp32:esp32:esp32"
SRC="$(cd "$(dirname "$0")" && pwd)"
BUILD="$DEV/build"

SKETCHES=("${@:-}")
if [ -z "${SKETCHES[0]}" ]; then
  SKETCHES=(masteresp Master_final wristband)
fi

for name in "${SKETCHES[@]}"; do
  echo "=== $name ==="
  mkdir -p "$BUILD/$name"
  cp "$SRC/$name.ino" "$BUILD/$name/$name.ino"

  # wristband.ino includes "MAX30102.h" and "Pulse.h", which are NOT
  # in this repository and never have been. They were local files on
  # the original author's machine. Until they are recovered the
  # wristband can only be type-checked against the stubs in
  # $BUILD/wristband, which are NOT drivers and must never be flashed.
  if [ "$name" = "wristband" ] && [ ! -f "$BUILD/$name/MAX30102.h" ]; then
    echo "  !! MAX30102.h / Pulse.h missing from the repo."
    echo "  !! Compiling against stubs -- DO NOT FLASH this build."
  fi

  $CLI compile --fqbn "$FQBN" "$BUILD/$name"
done
