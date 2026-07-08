# Showduino Web UI

GoreFX-themed control desk served from `web/`.

## Pages

| File | Purpose |
|------|---------|
| `index.html` | Main menu |
| `studio.html` | Timeline editor, live control, audio, devices |
| `led_control.html` | Per-line LED color and brightness |
| `hauntsync.html` | Community dashboard and cloud sync |
| `system.html` | Wi-Fi, display brightness, device status |

## Local preview

Open `index.html` directly, or serve the folder:

```bash
cd web
python3 -m http.server 8080
```

Then open `http://localhost:8080/`.

## SUE integration

When hosted on the Brain, the UI uses:

- `GET /api/status`
- `POST /api/command`

Scripts live in `js/`; shared app logic is in `app.js`.
