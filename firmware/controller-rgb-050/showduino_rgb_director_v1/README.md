# Showduino UI — Arduino IDE (ESP32-8048S050, 5" 800×400)

Open **`showduino_rgb_director_v1/`** in Arduino IDE (File → Open → select `showduino_rgb_director_v1.ino`).

## 1. Install board support

1. **File → Preferences → Additional boards manager URLs** — add:
   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
2. **Tools → Board → Boards Manager** → install **esp32** by Espressif (3.0.x or 2.0.17+)

## 2. Install libraries

| Library | Source | Version |
|---------|--------|---------|
| **lvgl** | Library Manager | 9.2.x |
| **esp32_smartdisplay** | [GitHub ZIP](https://github.com/rzeldent/esp32-smartdisplay) | 2.1.x |

## 3. Configure LVGL (required once)

Copy this folder's `lv_conf.h` into the LVGL library, or run `./install_lv_conf.sh`.

## 4. Board settings

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

`build_opt.h` in this folder supplies all **esp32-8048S050C_400** display/touch pin defines.

## 5. Edit Brain MAC

In `showduino_rgb_director_v1.ino`, set `SUE_MAC` to your Brain's Wi-Fi MAC.

## 6. Compile & upload

1. Select the correct serial port
2. Upload, then open Serial Monitor at **115200** baud

## PlatformIO alternative

See `../platformio/` — default env is `esp32-8048S050C_400`.

## Not for CYD

This sketch is for the **5" RGB ESP32-S3 panel**. The **2.8" CYD** Mega director is in `../../controller-cyd/showduino_cyd_director_v1/`.
