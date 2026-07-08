# Showduino UI — Arduino IDE (ESP32-8048S043)

Open **`UI_8048S043_ESPNOW/`** in Arduino IDE (File → Open → select `UI_8048S043_ESPNOW.ino`).

## 1. Install board support

1. **File → Preferences → Additional boards manager URLs** — add:
   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
2. **Tools → Board → Boards Manager** → install **esp32** by Espressif (3.0.x or 2.0.17+)

## 2. Install libraries

Install via **Sketch → Include Library → Manage Libraries** or from GitHub ZIP:

| Library | Source | Version |
|---------|--------|---------|
| **lvgl** | Library Manager | 9.2.x |
| **esp32_smartdisplay** | [GitHub ZIP](https://github.com/rzeldent/esp32-smartdisplay) | 2.1.x |

**Sketch → Include Library → Add .ZIP Library** → pick the downloaded `esp32-smartdisplay` zip.

## 3. Configure LVGL (required once)

Copy this folder's `lv_conf.h` into the LVGL library:

**Windows**
```
copy lv_conf.h %USERPROFILE%\Documents\Arduino\libraries\lvgl\lv_conf.h
```

**macOS**
```
cp lv_conf.h ~/Documents/Arduino/libraries/lvgl/lv_conf.h
```

**Linux**
```
cp lv_conf.h ~/Arduino/libraries/lvgl/lv_conf.h
```

Or run: `./install_lv_conf.sh` (auto-detects common paths)

## 4. Board settings

**Tools** menu:

| Setting | Value |
|---------|-------|
| Board | **ESP32S3 Dev Module** |
| USB CDC On Boot | **Disabled** |
| CPU Frequency | **240 MHz** |
| Flash Mode | **QIO 80MHz** |
| Flash Size | **16MB (128Mb)** |
| Partition Scheme | **16M Flash (3MB APP/9.9MB FATFS)** or **Huge APP** |
| PSRAM | **OPI PSRAM** |
| Upload Speed | **460800** |

`build_opt.h` in this folder supplies all ESP32-8048S043C display/touch pin defines automatically.

## 5. Edit Brain MAC

In `UI_8048S043_ESPNOW.ino`, set:

```cpp
static uint8_t SUE_MAC[6] = {0x20, 0x6E, 0xF1, 0x99, 0x83, 0x94};
```

## 6. Compile & upload

1. Select the correct serial port (**Tools → Port**)
2. Click **Upload**
3. Open **Serial Monitor** at **115200** baud

If upload fails: hold **BOOT**, tap **RESET**, release **BOOT**, upload again.

## Resistive touch variant (8048S043R)

Replace `build_opt.h` with the contents of `build_opt_R.h` (or rename files), then recompile.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `lv_conf.h` not found | Copy `lv_conf.h` into the lvgl library folder (step 3) |
| `esp32_smartdisplay.h` not found | Install library from GitHub ZIP |
| Blank display | Confirm `build_opt.h` is in the sketch folder; check PSRAM = OPI |
| Touch not working | Use `build_opt.h` (C/GT911), not `build_opt_R.h` |
| `build_opt.h` ignored | Change board setting once and recompile (clears IDE cache) |
| ESP-NOW fails | Match `SUE_MAC`; both devices on Wi-Fi channel 1 |

## PlatformIO alternative

See `../UI_RGB_8048_ESPNOW/` for the PlatformIO version of the same firmware.
