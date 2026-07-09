#!/usr/bin/env bash
# Copy lv_conf.h into the Arduino LVGL library folder.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC="$SCRIPT_DIR/lv_conf.h"

find_lvgl() {
  for base in \
    "$HOME/Arduino/libraries" \
    "$HOME/Documents/Arduino/libraries" \
    "$HOME/.arduino15/libraries"; do
    if [ -f "$base/lvgl/lv_conf.h" ] || [ -d "$base/lvgl" ]; then
      echo "$base/lvgl/lv_conf.h"
      return 0
    fi
  done
  return 1
}

if DEST="$(find_lvgl)"; then
  cp "$SRC" "$DEST"
  echo "Installed lv_conf.h -> $DEST"
else
  echo "Could not find Arduino lvgl library."
  echo "Install LVGL 9.2 from Library Manager, then run this script again."
  echo "Or manually copy:"
  echo "  $SRC"
  echo "  -> <Arduino>/libraries/lvgl/lv_conf.h"
  exit 1
fi
