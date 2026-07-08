# Showduino Studio

Web-based control desk for the Showduino system. The UI sends JSON commands to the Brain (SUE); all hardware actions execute on the Brain.

## Main Menu (`index.html`)

Entry point with links to:

- **Studio** — timeline editor, live control, audio, devices
- **LED Control** — per-line pixel color and brightness
- **HauntSync** — community dashboard and cloud sync
- **System** — Wi-Fi, display brightness, device status

## Hardware Sketches

- `UI_FULL_CYD_ESPNOW_FASTLED/` — CYD touchscreen UI (ESP-NOW sender)
- `SUE_ESPNOW_FASTLED_SINGLEPIXEL/` — Brain LED receiver (ESP-NOW + FastLED)

## API (SUE Brain)

The web UI expects these endpoints when served from SUE:

- `GET /api/status` — device health, Wi-Fi, RTC, SD, audio
- `POST /api/command` — JSON command bridge to Brain actions
