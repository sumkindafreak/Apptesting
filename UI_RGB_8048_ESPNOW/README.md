# Showduino UI — ESP32-8048S043

LVGL touchscreen UI for the **ESP32-8048S043** (4.3" 800×480 RGB, ESP32-S3).

| Variant | Touch | PlatformIO env |
|---------|-------|----------------|
| **8048S043C** | GT911 capacitive (I2C) | `esp32-8048S043C` ← default |
| 8048S043R | XPT2046 resistive (SPI) | `esp32-8048S043R` |
| 8048S043N | No touch | `esp32-8048S043N` |

## Hardware

- **Display:** 800×480, ST7262, 16-bit RGB parallel
- **MCU:** ESP32-S3-WROOM, 16 MB flash, 8 MB PSRAM
- **Touch (C model):** GT911 on I2C (SDA=19, SCL=20)
- **Backlight:** GPIO 2

This UI board does not drive relays or LEDs directly. It sends ESP-NOW commands to **SUE** (the Brain). Pair with `SUE_ESPNOW_FASTLED_SINGLEPIXEL/`.

## Setup

1. Install [PlatformIO](https://platformio.org/)
2. Open this folder (`UI_RGB_8048_ESPNOW`) as the project root
3. Confirm your board variant (usually **C** for capacitive)
4. Edit `src/main.cpp` — set `SUE_MAC` to your Brain's MAC address
5. Build and upload:

```bash
cd UI_RGB_8048_ESPNOW
pio run -e esp32-8048S043C -t upload
pio device monitor
```

For resistive touch boards:

```bash
pio run -e esp32-8048S043R -t upload
```

## UI

- **Main menu** — GoreFX-themed 2×2 grid (Studio, LED Control, HauntSync, System)
- **LED Control** — RGB + brightness sliders, toggle, ESP-NOW to SUE
- Other menu items show "coming soon" until those screens are ported

## Libraries

- [esp32-smartdisplay](https://github.com/rzeldent/esp32-smartdisplay) — RGB display + GT911 driver
- Built-in ESP-NOW (no extra library) — comms to Brain

Board definitions live in `boards/` (from [platformio-espressif32-sunton](https://github.com/rzeldent/platformio-espressif32-sunton)). To refresh:

```bash
git clone --depth 1 https://github.com/rzeldent/platformio-espressif32-sunton.git boards
```

## Troubleshooting

| Issue | Fix |
|-------|-----|
| Blank screen | Check USB power (5 V). Backlight is GPIO 2. |
| Touch dead | Confirm you built the **C** env, not R. |
| ESP-NOW fails | Set `SUE_MAC` in `main.cpp`. Both boards must share Wi-Fi channel 1. |
| Upload fails | Hold BOOT, tap RESET, release BOOT, then upload. |
