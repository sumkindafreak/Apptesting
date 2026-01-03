/*
  =========================================================
  CYD UI (ESP32) - FULL TOUCHSCREEN UI + QuickESPNow Sender
  =========================================================
  - Keeps full CYD functions: TFT + Touch + LVGL UI
  - Adds control for SUE FastLED pixel via ESP-NOW:
      Button: LED Toggle
      Sliders: R/G/B + Brightness
  - Sends throttled messages so touch remains responsive.

  Requires libraries:
    - TFT_eSPI
    - XPT2046_Touchscreen
    - lvgl
    - QuickESPNow (gmag11)

  IMPORTANT:
    If your TFT_eSPI User_Setup has TFT_RST = 33 while touch CS = 33,
    touch can break. Prefer TFT_RST = -1 (if your CYD wiring supports it).
*/

#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>

#include <lvgl.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

#include <QuickEspNow.h>

// =========================================================
// PINS / HARDWARE (CYD common defaults)
// =========================================================
#define TFT_BL            21   // backlight pin
#define SCREEN_WIDTH      320
#define SCREEN_HEIGHT     240

// Touch (XPT2046) pins for CYD (common)
#define XPT2046_IRQ       36
#define XPT2046_CS        33
#define XPT2046_SCK       25
#define XPT2046_MISO      39
#define XPT2046_MOSI      32

// Touch calibration (adjust if needed)
#define TOUCH_MIN_X       200
#define TOUCH_MAX_X       3700
#define TOUCH_MIN_Y       200
#define TOUCH_MAX_Y       3700

// =========================================================
// ESP-NOW / QuickESPNow SETTINGS
// =========================================================
#define ESPNOW_CHANNEL    1

// SUE STA MAC (your confirmed SUE MAC)
uint8_t SUE_MAC[6] = { 0x20, 0x6E, 0xF1, 0x99, 0x83, 0x94 };

// =========================================================
// GLOBAL OBJECTS
// =========================================================
TFT_eSPI tft = TFT_eSPI();

SPIClass touchSPI(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);

// Note: quickEspNow is provided as a global instance by the QuickEspNow library

// LVGL draw buffers
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[SCREEN_WIDTH * 10];
static lv_color_t buf2[SCREEN_WIDTH * 10];

// LVGL UI objects
static lv_obj_t *labelTitle;
static lv_obj_t *labelStatus;
static lv_obj_t *labelRgb;
static lv_obj_t *labelLedState;

static lv_obj_t *btnToggle;
static lv_obj_t *btnToggleLabel;

static lv_obj_t *sliderR;
static lv_obj_t *sliderG;
static lv_obj_t *sliderB;
static lv_obj_t *sliderBri;

// =========================================================
// UI STATE
// =========================================================
static volatile bool ledOn = false;

static volatile uint8_t uiR = 0;
static volatile uint8_t uiG = 0;
static volatile uint8_t uiB = 0;
static volatile uint8_t uiBri = 128;

// Send throttling
static volatile bool pendingSendRGB = false;
static volatile bool pendingSendToggle = false;
static uint32_t lastSendMs = 0;

// =========================================================
// UTIL
// =========================================================
static void printMac(const uint8_t* mac) {
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) Serial.print('0');
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(':');
  }
}

static void setStatusText(const char* s) {
  lv_label_set_text(labelStatus, s);
}

static void updateRgbLabel() {
  char buf[64];
  snprintf(buf, sizeof(buf), "RGB %u,%u,%u  BRI %u", uiR, uiG, uiB, uiBri);
  lv_label_set_text(labelRgb, buf);
}

static void updateLedButton() {
  if (ledOn) {
    lv_label_set_text(btnToggleLabel, "LED ON");
    lv_obj_set_style_bg_color(btnToggle, lv_color_hex(0x00AA00), 0);
    lv_label_set_text(labelLedState, "State: ON");
  } else {
    lv_label_set_text(btnToggleLabel, "LED OFF");
    lv_obj_set_style_bg_color(btnToggle, lv_color_hex(0xAA0000), 0);
    lv_label_set_text(labelLedState, "State: OFF");
  }
}

// =========================================================
// QUICKESPNOW SENDERS (ASCII COMMANDS)
// =========================================================
// SUE expects:
//   "RGB r g b bri"
//   "OFF"
//   "TOG"  (toggle)
static void sendRGB(uint8_t r, uint8_t g, uint8_t b, uint8_t bri) {
  char msg[48];
  snprintf(msg, sizeof(msg), "RGB %u %u %u %u", r, g, b, bri);

  Serial.print("[TX] ");
  Serial.println(msg);

  // QuickEspNow send (unicast)
  bool ok = quickEspNow.send(SUE_MAC, (uint8_t*)msg, strlen(msg));
  setStatusText(ok ? "Status: RGB sent" : "Status: RGB send failed");
}

static void sendToggle() {
  const char* msg = "TOG";
  Serial.println("[TX] TOG");

  bool ok = quickEspNow.send(SUE_MAC, (uint8_t*)msg, strlen(msg));
  setStatusText(ok ? "Status: Toggle sent" : "Status: Toggle send failed");
}

// Optional receive callback (SUE can ACK with text)
static void onDataReceived(uint8_t* address, uint8_t* data, uint8_t len, signed int rssi, bool broadcast) {
  char buf[80];
  uint8_t n = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);
  memcpy(buf, data, n);
  buf[n] = '\0';

  Serial.print("[RX] ");
  Serial.print("RSSI=");
  Serial.print(rssi);
  Serial.print(" msg='");
  Serial.print(buf);
  Serial.println("'");

  // If SUE sends "STATE 0/1" you can reflect it:
  int st = -1;
  if (sscanf(buf, "STATE %d", &st) == 1) {
    ledOn = (st != 0);
    updateLedButton();
  }
}

// =========================================================
// LVGL: DISPLAY FLUSH
// =========================================================
static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t*)&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

// =========================================================
// LVGL: TOUCH READ
// =========================================================
static void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  if (!ts.touched()) {
    data->state = LV_INDEV_STATE_REL;
    return;
  }

  TS_Point p = ts.getPoint();

  int x = map(p.x, TOUCH_MIN_X, TOUCH_MAX_X, 0, SCREEN_WIDTH);
  int y = map(p.y, TOUCH_MIN_Y, TOUCH_MAX_Y, 0, SCREEN_HEIGHT);

  x = constrain(x, 0, SCREEN_WIDTH - 1);
  y = constrain(y, 0, SCREEN_HEIGHT - 1);

  data->point.x = x;
  data->point.y = y;
  data->state = LV_INDEV_STATE_PR;

  static uint32_t lastDbg = 0;
  if (millis() - lastDbg > 600) {
    lastDbg = millis();
    Serial.printf("[TOUCH] raw(%d,%d) -> xy(%d,%d)\n", p.x, p.y, x, y);
  }
}

// =========================================================
// LVGL EVENTS
// =========================================================
static void btn_toggle_event_cb(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
    // Don’t spam sends from callback—set flag and let loop() handle it
    pendingSendToggle = true;

    // Optimistic UI toggle
    ledOn = !ledOn;
    updateLedButton();
  }
}

static void slider_event_cb(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
    uiR   = (uint8_t)lv_slider_get_value(sliderR);
    uiG   = (uint8_t)lv_slider_get_value(sliderG);
    uiB   = (uint8_t)lv_slider_get_value(sliderB);
    uiBri = (uint8_t)lv_slider_get_value(sliderBri);

    updateRgbLabel();

    // Mark send pending (throttled in loop)
    pendingSendRGB = true;
  }
}

// =========================================================
// BUILD UI
// =========================================================
static void createUI() {
  labelTitle = lv_label_create(lv_scr_act());
  lv_label_set_text(labelTitle, "CYD -> SUE LED Control");
  lv_obj_align(labelTitle, LV_ALIGN_TOP_MID, 0, 8);

  labelStatus = lv_label_create(lv_scr_act());
  lv_label_set_text(labelStatus, "Status: Booting...");
  lv_obj_align(labelStatus, LV_ALIGN_TOP_MID, 0, 28);

  btnToggle = lv_btn_create(lv_scr_act());
  lv_obj_set_size(btnToggle, 160, 50);
  lv_obj_align(btnToggle, LV_ALIGN_TOP_MID, 0, 55);
  lv_obj_add_event_cb(btnToggle, btn_toggle_event_cb, LV_EVENT_CLICKED, NULL);

  btnToggleLabel = lv_label_create(btnToggle);
  lv_label_set_text(btnToggleLabel, "LED OFF");
  lv_obj_center(btnToggleLabel);

  labelLedState = lv_label_create(lv_scr_act());
  lv_label_set_text(labelLedState, "State: OFF");
  lv_obj_align(labelLedState, LV_ALIGN_TOP_MID, 0, 110);

  labelRgb = lv_label_create(lv_scr_act());
  lv_label_set_text(labelRgb, "RGB 0,0,0  BRI 128");
  lv_obj_align(labelRgb, LV_ALIGN_TOP_MID, 0, 130);

  // Sliders (R, G, B, Brightness)
  sliderR = lv_slider_create(lv_scr_act());
  lv_slider_set_range(sliderR, 0, 255);
  lv_slider_set_value(sliderR, uiR, LV_ANIM_OFF);
  lv_obj_set_width(sliderR, 260);
  lv_obj_align(sliderR, LV_ALIGN_TOP_MID, 0, 150);
  lv_obj_add_event_cb(sliderR, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  sliderG = lv_slider_create(lv_scr_act());
  lv_slider_set_range(sliderG, 0, 255);
  lv_slider_set_value(sliderG, uiG, LV_ANIM_OFF);
  lv_obj_set_width(sliderG, 260);
  lv_obj_align(sliderG, LV_ALIGN_TOP_MID, 0, 175);
  lv_obj_add_event_cb(sliderG, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  sliderB = lv_slider_create(lv_scr_act());
  lv_slider_set_range(sliderB, 0, 255);
  lv_slider_set_value(sliderB, uiB, LV_ANIM_OFF);
  lv_obj_set_width(sliderB, 260);
  lv_obj_align(sliderB, LV_ALIGN_TOP_MID, 0, 200);
  lv_obj_add_event_cb(sliderB, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  sliderBri = lv_slider_create(lv_scr_act());
  lv_slider_set_range(sliderBri, 0, 255);
  lv_slider_set_value(sliderBri, uiBri, LV_ANIM_OFF);
  lv_obj_set_width(sliderBri, 260);
  lv_obj_align(sliderBri, LV_ALIGN_TOP_MID, 0, 225);
  lv_obj_add_event_cb(sliderBri, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  updateLedButton();
  updateRgbLabel();
}

// =========================================================
// LVGL TICK SUPPORT (CRITICAL FOR TOUCH)
// =========================================================
static uint32_t lastTickMs = 0;
static inline void lvglTick() {
  uint32_t now = millis();
  uint32_t diff = now - lastTickMs;
  lastTickMs = now;
  lv_tick_inc(diff);
}

// =========================================================
// SETUP
// =========================================================
void setup() {
  Serial.begin(115200);
  delay(400);

  Serial.println();
  Serial.println("=====================================");
  Serial.println("CYD UI - FULL UI + QuickESPNow");
  Serial.println("=====================================");

  // Backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  Serial.println("[TFT] Backlight ON");

  // TFT
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  Serial.printf("[TFT] Init OK: %dx%d\n", tft.width(), tft.height());

  // Touch
  pinMode(XPT2046_IRQ, INPUT);
  touchSPI.begin(XPT2046_SCK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);

  if (!ts.begin(touchSPI)) {
    Serial.println("[TOUCH] ❌ init FAILED");
  } else {
    ts.setRotation(1);
    Serial.println("[TOUCH] ✓ init OK");
  }

  // LVGL
  lv_init();

  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, SCREEN_WIDTH * 10);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_WIDTH;
  disp_drv.ver_res = SCREEN_HEIGHT;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  createUI();
  Serial.println("[LVGL] ✓ UI ready");

  // WiFi (STA only, no scanning/connect)
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false);
  WiFi.setSleep(false);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.print("[WiFi] UI STA MAC: ");
  printMac(mac);
  Serial.println();

  // QuickESPNow
  quickEspNow.onDataRcvd(onDataReceived);
  bool ok = quickEspNow.begin(ESPNOW_CHANNEL);
  Serial.print("[ESPNOW] begin(channel ");
  Serial.print(ESPNOW_CHANNEL);
  Serial.print(") -> ");
  Serial.println(ok ? "OK" : "FAIL");

  setStatusText(ok ? "Status: Ready" : "Status: ESPNOW FAIL");

  lastTickMs = millis();
}

// =========================================================
// LOOP
// =========================================================
void loop() {
  // Keep LVGL alive
  lvglTick();
  lv_timer_handler();

  // Throttled sending (prevents flooding + keeps touch smooth)
  const uint32_t SEND_INTERVAL_MS = 80;

  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    if (pendingSendToggle) {
      pendingSendToggle = false;
      lastSendMs = millis();
      sendToggle();
    } else if (pendingSendRGB) {
      pendingSendRGB = false;
      lastSendMs = millis();
      sendRGB(uiR, uiG, uiB, uiBri);
    }
  }

  delay(5);
}
