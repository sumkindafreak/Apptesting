# Firmware

Showduino firmware organized by role and hardware.

## Board → sketch matrix

| Hardware | Role | Path | IDE |
|----------|------|------|-----|
| **ESP32-2432S028R** (2.8" CYD) | UI director → Mega serial | `controller-cyd/showduino_cyd_director_v1/` | Arduino |
| **ESP32-8048S043** (4.3" RGB) | LVGL UI → SUE ESP-NOW | `controller-rgb-043/showduino_rgb_043_v1/` | Arduino |
| **ESP32-8048S050** (5" 800×400 RGB) | LVGL UI → SUE ESP-NOW | `controller-rgb-050/showduino_rgb_director_v1/` | Arduino |
| **ESP32-8048S050** (5" RGB) | LVGL UI → SUE ESP-NOW | `controller-rgb-050/platformio/` | PlatformIO |
| **ESP32-S3** (SUE Brain) | LED / ESP-NOW executor | `brain-sue/SUE_ESPNOW_FASTLED_SINGLEPIXEL/` | Arduino |
| **Arduino Mega** | Show executor (relays, audio) | `executor-mega/showduino_mega_v1/` | Arduino (stub) |
| **ESP32 CYD** (legacy) | Old ESP-NOW CYD UI | `legacy-cyd-espnow/` | Arduino |

## Architecture

```
[ CYD 2.8" UI ] --Serial JSON--> [ Arduino Mega ]
[ RGB 5"/4.3" UI ] --ESP-NOW--> [ SUE Brain ESP32 ]
[ Web UI ] --HTTP/JSON--> [ SUE Brain ESP32 ]
```

UI boards never drive relays or LEDs directly. They send commands; the Brain or Mega executes them.

## Quick open (Arduino IDE)

1. **CYD Mega director:** `controller-cyd/showduino_cyd_director_v1/showduino_cyd_director_v1.ino`
2. **5" RGB desk:** `controller-rgb-050/showduino_rgb_director_v1/showduino_rgb_director_v1.ino`
3. **4.3" RGB desk:** `controller-rgb-043/showduino_rgb_043_v1/UI_8048S043_ESPNOW.ino`
4. **SUE Brain:** `brain-sue/SUE_ESPNOW_FASTLED_SINGLEPIXEL/SUE_ESPNOW_FASTLED_SINGLEPIXEL.ino`

Each Arduino sketch folder includes its own `README.md` with libraries and board settings.
