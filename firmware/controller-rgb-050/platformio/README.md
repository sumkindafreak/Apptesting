# Showduino UI — PlatformIO (RGB panels)

LVGL touchscreen UI for Sunton ESP32-S3 RGB panels.

## Default target (this folder)

**5" 800×400** — `esp32-8048S050C_400`

```bash
cd firmware/controller-rgb-050/platformio
pio run -e esp32-8048S050C_400 -t upload
pio device monitor
```

## Other board envs

| Env | Panel |
|-----|-------|
| `esp32-8048S050C` | 5" 800×480 |
| `esp32-8048S043C` | 4.3" 800×480 capacitive |
| `esp32-8048S043R` | 4.3" resistive |
| `JC8048W550C` | JC8048 variant |

Edit `src/main.cpp` — set `SUE_MAC` to your Brain's MAC address.

## Arduino IDE

- **5" 800×400:** `../showduino_rgb_director_v1/`
- **4.3" 800×480:** `../../controller-rgb-043/showduino_rgb_043_v1/`

## Pair with Brain

Flash `../../brain-sue/SUE_ESPNOW_FASTLED_SINGLEPIXEL/` on the SUE ESP32. Both devices must use Wi-Fi channel 1.
