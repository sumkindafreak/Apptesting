# Showduino Studio

Web-based control desk for the Showduino system. The UI sends JSON commands to the Brain (SUE); all hardware actions execute on the Brain.

## Main Menu (`index.html`)

Entry point with links to:

- **Studio** — timeline editor, live control, audio, devices
- **LED Control** — per-line pixel color and brightness
- **HauntSync** — community dashboard and cloud sync
- **System** — Wi-Fi, display brightness, device status

## Hardware Sketches

- **`firmware/controller-cyd/showduino_cyd_director_v1/`** — **2.8" CYD** Mega director (TFT_eSPI + XPT2046_Bitbang)
- `UI_8048S043_ESPNOW/` — 4.3" 800×480 RGB (Arduino IDE, esp32-smartdisplay)
- `UI_RGB_8048_ESPNOW/` — PlatformIO RGB panels (4.3" / 5")

## API (SUE Brain)

The web UI expects these endpoints when served from SUE:

- `GET /api/status` — device health, Wi-Fi, RTC, SD, audio
- `POST /api/command` — JSON command bridge to Brain actions
