# URGENT: You are compiling the OLD file

Your error shows **line 262** with `getTouchPoint()` and `XPT2046_Bitbang`.

The **fixed** sketch in this repo is **414 lines** and has **no** `getTouchPoint`, **no** `XPT2046`, **no** `touched()`.

## Fix in 30 seconds

1. **Close** Arduino IDE
2. On your PC, go to:
   ```
   C:\Users\tjpro\Desktop\new_showduino-main\new_showduino-main\firmware\controller-cyd\showduino_cyd_director_v1\
   ```
3. **Delete** (or rename to `.bak`) these old files if present:
   - `showduino_cyd_director_v1.ino` (the broken one)
4. **Copy the entire folder** from this repo:
   ```
   firmware/controller-cyd/showduino_cyd_director_v1/
   ```
   Including:
   - `showduino_cyd_director_v1.ino`  ← must be ~414 lines
   - `build_opt.h`                    ← required for 5" 800x400
   - `lv_conf.h`
   - `install_lv_conf.sh`
5. Open `showduino_cyd_director_v1.ino` in Arduino IDE
6. Verify line 1 says `Showduino CYD Director v1` and line 13 says `#include <esp32_smartdisplay.h>`
7. Compile again

## Quick check before compile

Open the `.ino` and search (Ctrl+F):

| Search for | Should find |
|------------|-------------|
| `getTouchPoint` | **0 results** |
| `XPT2046` | **0 results** (only in comment on line 4) |
| `esp32_smartdisplay` | **1 result** |
| `build_opt.h` | file exists in same folder |

If you still see `getTouchPoint` at line 262, you are **not** using the new file.

## Libraries (not XPT2046_Bitbang)

Uninstall or ignore **XPT2046_Bitbang** for this project. Install instead:

1. **lvgl** 9.2.x
2. **esp32_smartdisplay** from https://github.com/rzeldent/esp32-smartdisplay (ZIP)

Copy `lv_conf.h` to `Documents\Arduino\libraries\lvgl\lv_conf.h`

## Board: ESP32S3 Dev Module

- Flash: 16MB
- PSRAM: OPI PSRAM  
- USB CDC On Boot: Disabled
