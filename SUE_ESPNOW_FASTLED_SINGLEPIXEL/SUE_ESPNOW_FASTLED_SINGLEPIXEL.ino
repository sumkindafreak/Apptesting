/*
  =========================================================
  SUE (ESP32-S3 XH3SE) - QuickESPNow RX -> FastLED (1 pixel)
  =========================================================
  LED: 1 WS2812B on GPIO 48 using FastLED

  Receives ASCII:
    "RGB r g b bri"
    "TOG"
    "OFF"
  Optionally replies to UI:
    "STATE 0" / "STATE 1"
*/

#include <Arduino.h>
#include <WiFi.h>
#include <FastLED.h>
#include <QuickEspNow.h>

// ---------------- FASTLED ----------------
#define LED_PIN     48
#define LED_COUNT   1
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

CRGB led;
uint8_t gBri = 128;
bool gOn = false;
CRGB gColor = CRGB::White;

// ---------------- ESPNOW ----------------
#define ESPNOW_CHANNEL 1

// UI STA MAC (your confirmed UI MAC)
uint8_t UI_MAC[6] = { 0x5C, 0x01, 0x3B, 0x51, 0x2A, 0x7C };

// Note: quickEspNow is provided as a global instance by the QuickEspNow library

static void printMac(const uint8_t* mac) {
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) Serial.print('0');
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(':');
  }
}

static void applyLED() {
  FastLED.setBrightness(gBri);
  led = gOn ? gColor : CRGB::Black;
  FastLED.show();

  Serial.printf("[LED] on=%d bri=%u rgb=(%u,%u,%u)\n",
                gOn, gBri, gColor.r, gColor.g, gColor.b);
}

static void sendStateToUI() {
  char msg[16];
  snprintf(msg, sizeof(msg), "STATE %d", gOn ? 1 : 0);
  quickEspNow.send(UI_MAC, (uint8_t*)msg, strlen(msg));
}

static void onDataReceived(uint8_t* address, uint8_t* data, uint8_t len, signed int rssi, bool broadcast) {
  char buf[80];
  uint8_t n = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);
  memcpy(buf, data, n);
  buf[n] = '\0';

  Serial.print("[RX] From ");
  printMac(address);
  Serial.print(" RSSI=");
  Serial.print(rssi);
  Serial.print(" msg='");
  Serial.print(buf);
  Serial.println("'");

  if (!strcmp(buf, "TOG")) {
    gOn = !gOn;
    applyLED();
    sendStateToUI();
    return;
  }

  if (!strcmp(buf, "OFF")) {
    gOn = false;
    applyLED();
    sendStateToUI();
    return;
  }

  int r, g, b, bri;
  if (sscanf(buf, "RGB %d %d %d %d", &r, &g, &b, &bri) == 4) {
    r = constrain(r, 0, 255);
    g = constrain(g, 0, 255);
    b = constrain(b, 0, 255);
    bri = constrain(bri, 0, 255);

    gColor = CRGB((uint8_t)r, (uint8_t)g, (uint8_t)b);
    gBri = (uint8_t)bri;
    gOn = true;

    applyLED();
    sendStateToUI();
    return;
  }

  Serial.println("[RX] Unknown command");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("===============================================");
  Serial.println("SUE - QuickESPNow RX -> FastLED (1px GPIO48)");
  Serial.println("===============================================");

  // FastLED init
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(&led, LED_COUNT);
  gOn = false;
  applyLED();
  Serial.println("[LED] Ready");

  // WiFi STA
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false);
  WiFi.setSleep(false);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.print("[WiFi] SUE STA MAC: ");
  printMac(mac);
  Serial.println();

  // QuickESPNow
  quickEspNow.onDataRcvd(onDataReceived);
  bool ok = quickEspNow.begin(ESPNOW_CHANNEL);
  Serial.print("[ESPNOW] begin(channel ");
  Serial.print(ESPNOW_CHANNEL);
  Serial.print(") -> ");
  Serial.println(ok ? "OK" : "FAIL");

  // Boot blink
  gColor = CRGB::Red;   gBri = 128; gOn = true;  applyLED(); delay(150);
  gColor = CRGB::Green;               applyLED(); delay(150);
  gColor = CRGB::Blue;                applyLED(); delay(150);
  gOn = false; applyLED();
}

void loop() {
  delay(5);
}
