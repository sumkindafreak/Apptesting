# Showduino Mega Executor v1

Arduino Mega sketch that receives JSON commands from the **CYD director** over Serial1 and drives show hardware (relays, MP3, LEDs, etc.).

## Wiring (from CYD director)

| CYD ESP32 | Arduino Mega |
|-----------|--------------|
| TX (pin 1) | RX1 (pin 19) |
| RX (pin 3) | TX1 (pin 18) |
| GND | GND |

## Status

The Mega `.ino` is not yet in this repository. The CYD UI sketch (`../controller-cyd/showduino_cyd_director_v1/`) is ready and expects a Mega partner on Serial2 at **115200** baud.

Add your Mega firmware here as `showduino_mega_v1.ino` when ready.
