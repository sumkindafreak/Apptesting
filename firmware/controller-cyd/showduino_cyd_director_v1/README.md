# Showduino CYD Director v1 — Arduino IDE

**2.8" CYD (ESP32-2432S028R)** — 320×240, Mega serial director

Sends commands to Arduino Mega via Serial2. Does **not** use esp32-smartdisplay (that's for 5" RGB panels).

## Libraries

| Library | Install |
|---------|---------|
| **TFT_eSPI** | Library Manager — configure `User_Setup` for your CYD |
| **XPT2046_Bitbang** | Library Manager (`ddxfish`) — **not** XPT2046_Touchscreen |

Use **one** Bitbang library. If Arduino picks the wrong one:
- Sketch → Include Library → **XPT2046_Bitbang** (not Slim)

## Touch fix (applied)

`XPT2046_Bitbang` does **not** have `touched()` or `TS_Point`. Correct usage:

```cpp
Point p = touch.getTouch();
```

Touch detect uses **IRQ pin 36** (LOW = pressed).

## First boot

`touch.begin()` may run **calibration** over USB Serial — follow prompts (touch top-left, then bottom-right).

## Mega wiring

| CYD | Mega |
|-----|------|
| TX pin 1 | RX1 pin 19 |
| RX pin 3 | TX1 pin 18 |
| GND | GND |

## Board

**ESP32 Dev Module** (or ESP32-WROOM) — the classic 2.8" CYD, not ESP32-S3 5" RGB.

## Touch upside-down?

In `getTouchPoint()`, uncomment the swap lines:

```cpp
int16_t tmp = x; x = y; y = tmp;
```

Or call `touch.setCalibration(xMin, yMin, xMax, yMax)` after measuring raw values.
