# Showduino Studio

Web-based control desk for the Showduino system. The UI sends JSON commands to the Brain (SUE); all hardware actions execute on the Brain.

## Main Menu (`index.html`)

Entry point with links to:

- **Studio** — timeline editor, live control, audio, devices
- **LED Control** — per-line pixel color and brightness
- **HauntSync** — community dashboard and cloud sync
- **System** — Wi-Fi, display brightness, device status

## Hardware Sketches

- **`firmware/controller-cyd/showduino_cyd_director_v1/`** — **5" 800×400** director UI (Arduino IDE)
- `UI_8048S043_ESPNOW/` — Arduino IDE sketch for ESP32-8048S043 (4.3")
- `UI_RGB_8048_ESPNOW/` — PlatformIO version
- `UI_FULL_CYD_ESPNOW_FASTLED/` — legacy 2.4" CYD (320×240, XPT2046)
- `SUE_ESPNOW_FASTLED_SINGLEPIXEL/` — Brain LED receiver

## API (SUE Brain)

The web UI expects these endpoints when served from SUE:

- `GET /api/status` — device health, Wi-Fi, RTC, SD, audio
- `POST /api/command` — JSON command bridge to Brain actions
