# Showduino CYD Director v1 — Arduino IDE

**5" RGB display, 800×400, ESP32-S3**

Matches your project path:
`firmware/controller-cyd/showduino_cyd_director_v1/`

## Why your old sketch failed

Your compile error came from mixing two different touch libraries:

| Wrong (your error) | Correct for 5" RGB |
|--------------------|-------------------|
| `XPT2046_Bitbang` | **esp32-smartdisplay** |
| `touch.touched()` | GT911 handled by LVGL |
| `TS_Point` / `getPoint()` | No manual touch reads needed |

`XPT2046_Bitbang` is for the **2.4" Cheap Yellow Display** (320×240).  
Your **5" 800×400** panel uses **16-bit RGB + GT911** (capacitive) — a completely different driver stack.

This sketch removes all `XPT2046_Bitbang` / `getTouchPoint()` code.

## Libraries

1. **lvgl** 9.2.x (Library Manager)
2. **esp32_smartdisplay** 2.1.x — [GitHub ZIP](https://github.com/rzeldent/esp32-smartdisplay)

## LVGL config (once)

```bash
./install_lv_conf.sh
```

Or copy `lv_conf.h` → `Arduino/libraries/lvgl/lv_conf.h`

## Board settings (Tools)

| Setting | Value |
|---------|-------|
| Board | **ESP32S3 Dev Module** |
| USB CDC On Boot | **Disabled** |
| Flash Size | **16MB** |
| PSRAM | **OPI PSRAM** |
| CPU | **240 MHz** |

`build_opt.h` sets `DISPLAY_WIDTH=800` and `DISPLAY_HEIGHT=400` for your 5" panel.

## If the image is wrong

Some 5" boards are actually **800×480**. If you see a cropped or rolling image, change one line in `build_opt.h`:

```
-DDISPLAY_HEIGHT=480
```

and recompile.

## Upload

1. Set `SUE_MAC` in the `.ino` file
2. Open `showduino_cyd_director_v1.ino` in Arduino IDE
3. Upload

Touch works automatically through `smartdisplay_init()` — no `getTouchPoint()` function needed.
