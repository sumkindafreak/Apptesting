# Showduino Studio (`new_showduino-main`)

Unified project layout for the Showduino control desk: web UI, touchscreen firmware, and Brain sketches.

## Project layout

```
new_showduino-main/
├── web/                          # GoreFX web control desk
│   ├── index.html                # Main menu
│   ├── studio.html               # Timeline / live control
│   ├── led_control.html
│   ├── hauntsync.html
│   ├── system.html
│   ├── app.js
│   ├── style.css
│   └── js/
├── firmware/
│   ├── controller-cyd/           # 2.8" CYD → Mega serial director
│   ├── controller-rgb-043/       # 4.3" 800×480 RGB (Arduino IDE)
│   ├── controller-rgb-050/       # 5" 800×400 RGB (Arduino + PlatformIO)
│   ├── brain-sue/                # SUE ESP-NOW Brain
│   ├── executor-mega/            # Arduino Mega partner (stub)
│   └── legacy-cyd-espnow/        # Older CYD ESP-NOW sketch
└── README.md
```

See [`firmware/README.md`](firmware/README.md) for the full board → sketch matrix.

## Web UI

Open `web/index.html` in a browser, or serve the `web/` folder from the SUE Brain when Wi-Fi is available.

The UI polls `GET /api/status` and sends `POST /api/command` JSON to the Brain. All hardware actions run on SUE, not in the browser.

## Firmware quick start

### 2.8" CYD Mega director (your Desktop path)

```
firmware/controller-cyd/showduino_cyd_director_v1/showduino_cyd_director_v1.ino
```

Libraries: **TFT_eSPI**, **XPT2046_Bitbang** (`ddxfish`). Board: **ESP32 Dev Module**.

Touch uses `Point p = touch.getTouch()` and IRQ pin 36 — not `touched()` or `TS_Point`.

### 5" RGB 800×400 ESP32-S3

```
firmware/controller-rgb-050/showduino_rgb_director_v1/showduino_rgb_director_v1.ino
```

Libraries: **lvgl** 9.2, **esp32_smartdisplay**. Copy `lv_conf.h` into the LVGL library (see folder README). Board: **ESP32S3 Dev Module**, PSRAM **OPI**.

### 4.3" RGB 800×480 ESP32-S3

```
firmware/controller-rgb-043/showduino_rgb_043_v1/UI_8048S043_ESPNOW.ino
```

Same stack as the 5" sketch; `build_opt.h` targets **esp32-8048S043C**.

### SUE Brain

```
firmware/brain-sue/SUE_ESPNOW_FASTLED_SINGLEPIXEL/SUE_ESPNOW_FASTLED_SINGLEPIXEL.ino
```

Set the `SUE_MAC` in the RGB UI sketch to match this board's Wi-Fi MAC.

## Sync to your Desktop (Windows)

Replace your local folder with this repo so paths match:

```
C:\Users\tjpro\Desktop\new_showduino-main\new_showduino-main\
```

1. Download or clone this repository.
2. Copy the entire contents into the path above (overwrite the old tree).
3. In Arduino IDE, open:
   ```
   firmware\controller-cyd\showduino_cyd_director_v1\showduino_cyd_director_v1.ino
   ```
4. Do **not** use an old copy that still calls `touch.touched()` — that API does not exist in XPT2046_Bitbang.

## Two different UI boards

| Board | Sketch folder | Touch driver |
|-------|---------------|--------------|
| 2.8" CYD (ESP32-2432S028R) | `controller-cyd/` | XPT2046_Bitbang |
| 5" RGB (ESP32-8048S050) | `controller-rgb-050/` | GT911 via esp32-smartdisplay |
| 4.3" RGB (ESP32-8048S043) | `controller-rgb-043/` | GT911 via esp32-smartdisplay |

Do not mix sketches between boards — each has its own `build_opt.h` and libraries.

## API (SUE Brain)

When the web UI is served from SUE:

- `GET /api/status` — device health, Wi-Fi, RTC, SD, audio
- `POST /api/command` — JSON command bridge to Brain actions
