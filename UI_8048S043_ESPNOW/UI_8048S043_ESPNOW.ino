/*
  Showduino Studio UI — ESP32-8048S043 (4.3" 800x480 RGB, ESP32-S3)
  Primary target: esp32-8048S043C (GT911 capacitive touch)

  UI desk only. Sends ESP-NOW commands to SUE (Brain).
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp32_smartdisplay.h>

#ifndef BOARD_NAME
#define BOARD_NAME "esp32-8048S043C"
#endif

// GoreFX palette
static const uint32_t COLOR_BG      = 0x0A0A0A;
static const uint32_t COLOR_PANEL   = 0x1A0A1A;
static const uint32_t COLOR_ACCENT  = 0xFF0044;
static const uint32_t COLOR_TEXT    = 0xE0E0E0;
static const uint32_t COLOR_MUTED   = 0x707070;
static const uint32_t COLOR_BORDER  = 0x3A1A3A;
static const uint32_t COLOR_OK      = 0x00AA00;
static const uint32_t COLOR_OFF     = 0x660000;

// SUE STA MAC — update to match your Brain
static uint8_t SUE_MAC[6] = {0x20, 0x6E, 0xF1, 0x99, 0x83, 0x94};
static const uint8_t ESPNOW_CHANNEL = 1;

// LED / send state
static volatile bool ledOn = false;
static volatile uint8_t uiR = 255;
static volatile uint8_t uiG = 0;
static volatile uint8_t uiB = 68;
static volatile uint8_t uiBri = 200;
static volatile bool pendingSendRGB = false;
static volatile bool pendingSendToggle = false;
static uint32_t lastSendMs = 0;

// Screens + widgets
static lv_obj_t *scr_main = nullptr;
static lv_obj_t *scr_led = nullptr;
static lv_obj_t *lbl_status_main = nullptr;
static lv_obj_t *lbl_status_led = nullptr;
static lv_obj_t *lbl_rgb = nullptr;
static lv_obj_t *lbl_led_state = nullptr;
static lv_obj_t *btn_toggle = nullptr;
static lv_obj_t *btn_toggle_label = nullptr;
static lv_obj_t *preview_box = nullptr;
static lv_obj_t *slider_r = nullptr;
static lv_obj_t *slider_g = nullptr;
static lv_obj_t *slider_b = nullptr;
static lv_obj_t *slider_bri = nullptr;

static lv_style_t style_screen;
static lv_style_t style_title;
static lv_style_t style_subtitle;
static lv_style_t style_card;
static lv_style_t style_card_title;
static lv_style_t style_status;

static void printMac(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) Serial.print('0');
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(':');
  }
}

static void setStatus(lv_obj_t *label, const char *text) {
  if (label) lv_label_set_text(label, text);
}

static void updateRgbLabel() {
  if (!lbl_rgb) return;
  char buf[48];
  snprintf(buf, sizeof(buf), "RGB %u,%u,%u  BRI %u", uiR, uiG, uiB, uiBri);
  lv_label_set_text(lbl_rgb, buf);
}

static void updatePreview() {
  if (!preview_box) return;
  const float scale = ledOn ? (uiBri / 255.0f) : 0.0f;
  const uint8_t r = (uint8_t)(uiR * scale);
  const uint8_t g = (uint8_t)(uiG * scale);
  const uint8_t b = (uint8_t)(uiB * scale);
  lv_obj_set_style_bg_color(preview_box, lv_color_make(r, g, b), 0);
  lv_obj_set_style_shadow_color(preview_box, lv_color_make(uiR, uiG, uiB), 0);
  lv_obj_set_style_shadow_width(preview_box, ledOn ? 30 : 0, 0);
}

static void updateLedButton() {
  if (!btn_toggle || !btn_toggle_label || !lbl_led_state) return;
  if (ledOn) {
    lv_label_set_text(btn_toggle_label, "LED ON");
    lv_obj_set_style_bg_color(btn_toggle, lv_color_hex(COLOR_OK), 0);
    lv_label_set_text(lbl_led_state, "Output: ON");
  } else {
    lv_label_set_text(btn_toggle_label, "LED OFF");
    lv_obj_set_style_bg_color(btn_toggle, lv_color_hex(COLOR_OFF), 0);
    lv_label_set_text(lbl_led_state, "Output: OFF");
  }
  updatePreview();
}

static bool espnowSend(const char *msg) {
  const size_t len = strlen(msg);
  const esp_err_t err = esp_now_send(SUE_MAC, (const uint8_t *)msg, len);
  if (err != ESP_OK) {
    Serial.printf("[TX] esp_now_send failed: %d\n", err);
    return false;
  }
  return true;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac = info->src_addr;
#else
static void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  char buf[80];
  const int n = (len < (int)sizeof(buf) - 1) ? len : (int)sizeof(buf) - 1;
  memcpy(buf, data, n);
  buf[n] = '\0';

  Serial.print("[RX] From ");
  printMac(mac);
  Serial.printf(" msg='%s'\n", buf);

  int st = -1;
  if (sscanf(buf, "STATE %d", &st) == 1) {
    ledOn = (st != 0);
    updateLedButton();
  }
}

static bool initEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] init failed");
    return false;
  }

  esp_now_register_recv_cb(onDataReceived);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, SUE_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;

  const esp_err_t addErr = esp_now_add_peer(&peer);
  if (addErr != ESP_OK) {
    Serial.printf("[ESPNOW] add_peer failed: %d\n", addErr);
    return false;
  }

  return true;
}

static void sendRGB(uint8_t r, uint8_t g, uint8_t b, uint8_t bri) {
  char msg[48];
  snprintf(msg, sizeof(msg), "RGB %u %u %u %u", r, g, b, bri);
  Serial.print("[TX] ");
  Serial.println(msg);
  const bool ok = espnowSend(msg);
  setStatus(lbl_status_led, ok ? "ESP-NOW: RGB sent" : "ESP-NOW: send failed");
}

static void sendToggle() {
  Serial.println("[TX] TOG");
  const bool ok = espnowSend("TOG");
  setStatus(lbl_status_led, ok ? "ESP-NOW: toggle sent" : "ESP-NOW: send failed");
}

static lv_obj_t *make_card(lv_obj_t *parent, const char *icon, const char *title, const char *desc, lv_event_cb_t cb) {
  lv_obj_t *card = lv_obj_create(parent);
  lv_obj_remove_style_all(card);
  lv_obj_add_style(card, &style_card, 0);
  lv_obj_set_size(card, lv_pct(48), 150);
  lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(card, cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *icon_lbl = lv_label_create(card);
  lv_label_set_text(icon_lbl, icon);
  lv_obj_set_style_text_font(icon_lbl, &lv_font_montserrat_32, 0);
  lv_obj_align(icon_lbl, LV_ALIGN_TOP_MID, 0, 12);

  lv_obj_t *title_lbl = lv_label_create(card);
  lv_label_set_text(title_lbl, title);
  lv_obj_add_style(title_lbl, &style_card_title, 0);
  lv_obj_align(title_lbl, LV_ALIGN_TOP_MID, 0, 56);

  lv_obj_t *desc_lbl = lv_label_create(card);
  lv_label_set_text(desc_lbl, desc);
  lv_obj_set_style_text_color(desc_lbl, lv_color_hex(COLOR_MUTED), 0);
  lv_obj_set_style_text_font(desc_lbl, &lv_font_montserrat_14, 0);
  lv_obj_align(desc_lbl, LV_ALIGN_TOP_MID, 0, 88);

  return card;
}

static void go_main_event(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
    lv_screen_load(scr_main);
  }
}

static void go_led_event(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
    lv_screen_load(scr_led);
  }
}

static void coming_soon_event(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
    setStatus(lbl_status_main, "Coming soon — wired in next build");
  }
}

static void btn_toggle_event(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
    pendingSendToggle = true;
    ledOn = !ledOn;
    updateLedButton();
  }
}

static void slider_event(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
  uiR = (uint8_t)lv_slider_get_value(slider_r);
  uiG = (uint8_t)lv_slider_get_value(slider_g);
  uiB = (uint8_t)lv_slider_get_value(slider_b);
  uiBri = (uint8_t)lv_slider_get_value(slider_bri);
  updateRgbLabel();
  updatePreview();
  pendingSendRGB = true;
}

static lv_obj_t *make_slider_row(lv_obj_t *parent, const char *name, int y, uint8_t value, lv_obj_t **out_slider) {
  lv_obj_t *lbl = lv_label_create(parent);
  lv_label_set_text(lbl, name);
  lv_obj_set_style_text_color(lbl, lv_color_hex(COLOR_MUTED), 0);
  lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 20, y);

  lv_obj_t *slider = lv_slider_create(parent);
  lv_slider_set_range(slider, 0, 255);
  lv_slider_set_value(slider, value, LV_ANIM_OFF);
  lv_obj_set_width(slider, 520);
  lv_obj_align(slider, LV_ALIGN_TOP_LEFT, 90, y + 2);
  lv_obj_add_event_cb(slider, slider_event, LV_EVENT_VALUE_CHANGED, nullptr);
  *out_slider = slider;
  return slider;
}

static void init_theme() {
  lv_style_init(&style_screen);
  lv_style_set_bg_color(&style_screen, lv_color_hex(COLOR_BG));
  lv_style_set_bg_opa(&style_screen, LV_OPA_COVER);

  lv_style_init(&style_title);
  lv_style_set_text_color(&style_title, lv_color_hex(COLOR_ACCENT));
  lv_style_set_text_font(&style_title, &lv_font_montserrat_32);

  lv_style_init(&style_subtitle);
  lv_style_set_text_color(&style_subtitle, lv_color_hex(COLOR_MUTED));
  lv_style_set_text_font(&style_subtitle, &lv_font_montserrat_14);

  lv_style_init(&style_card);
  lv_style_set_bg_color(&style_card, lv_color_hex(COLOR_PANEL));
  lv_style_set_bg_opa(&style_card, LV_OPA_COVER);
  lv_style_set_border_color(&style_card, lv_color_hex(COLOR_BORDER));
  lv_style_set_border_width(&style_card, 2);
  lv_style_set_radius(&style_card, 14);
  lv_style_set_pad_all(&style_card, 10);

  lv_style_init(&style_card_title);
  lv_style_set_text_color(&style_card_title, lv_color_hex(COLOR_TEXT));
  lv_style_set_text_font(&style_card_title, &lv_font_montserrat_20);

  lv_style_init(&style_status);
  lv_style_set_text_color(&style_status, lv_color_hex(COLOR_MUTED));
  lv_style_set_text_font(&style_status, &lv_font_montserrat_14);
}

static void create_main_menu() {
  scr_main = lv_obj_create(nullptr);
  lv_obj_add_style(scr_main, &style_screen, 0);

  lv_obj_t *title = lv_label_create(scr_main);
  lv_label_set_text(title, "SHOWDUINO");
  lv_obj_add_style(title, &style_title, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 24);

  lv_obj_t *subtitle = lv_label_create(scr_main);
  lv_label_set_text(subtitle, "Studio Control System");
  lv_obj_add_style(subtitle, &style_subtitle, 0);
  lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 64);

  lv_obj_t *grid = lv_obj_create(scr_main);
  lv_obj_remove_style_all(grid);
  lv_obj_set_size(grid, 760, 320);
  lv_obj_align(grid, LV_ALIGN_CENTER, 0, 20);
  lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_flex_align(grid, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_row(grid, 16, 0);
  lv_obj_set_style_pad_column(grid, 16, 0);

  make_card(grid, LV_SYMBOL_EDIT, "Studio", "Timeline & Show Builder", coming_soon_event);
  make_card(grid, LV_SYMBOL_IMAGE, "LED Control", "Pixel FX & Lighting", go_led_event);
  make_card(grid, LV_SYMBOL_WIFI, "HauntSync", "Network Dashboard", coming_soon_event);
  make_card(grid, LV_SYMBOL_SETTINGS, "System", "Wi-Fi & Settings", coming_soon_event);

  lbl_status_main = lv_label_create(scr_main);
  lv_label_set_text(lbl_status_main, "ESP-NOW ready — tap LED Control");
  lv_obj_add_style(lbl_status_main, &style_status, 0);
  lv_obj_align(lbl_status_main, LV_ALIGN_BOTTOM_MID, 0, -16);
}

static void create_led_screen() {
  scr_led = lv_obj_create(nullptr);
  lv_obj_add_style(scr_led, &style_screen, 0);

  lv_obj_t *back = lv_button_create(scr_led);
  lv_obj_set_size(back, 120, 40);
  lv_obj_align(back, LV_ALIGN_TOP_LEFT, 12, 12);
  lv_obj_add_event_cb(back, go_main_event, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *back_lbl = lv_label_create(back);
  lv_label_set_text(back_lbl, LV_SYMBOL_LEFT " Menu");
  lv_obj_center(back_lbl);

  lv_obj_t *title = lv_label_create(scr_led);
  lv_label_set_text(title, "LED Control");
  lv_obj_add_style(title, &style_card_title, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

  preview_box = lv_obj_create(scr_led);
  lv_obj_set_size(preview_box, 100, 100);
  lv_obj_set_style_radius(preview_box, 12, 0);
  lv_obj_set_style_border_color(preview_box, lv_color_hex(COLOR_BORDER), 0);
  lv_obj_set_style_border_width(preview_box, 2, 0);
  lv_obj_align(preview_box, LV_ALIGN_TOP_RIGHT, -30, 20);

  btn_toggle = lv_button_create(scr_led);
  lv_obj_set_size(btn_toggle, 180, 52);
  lv_obj_align(btn_toggle, LV_ALIGN_TOP_MID, 0, 70);
  lv_obj_add_event_cb(btn_toggle, btn_toggle_event, LV_EVENT_CLICKED, nullptr);
  btn_toggle_label = lv_label_create(btn_toggle);
  lv_label_set_text(btn_toggle_label, "LED OFF");
  lv_obj_center(btn_toggle_label);

  lbl_led_state = lv_label_create(scr_led);
  lv_label_set_text(lbl_led_state, "Output: OFF");
  lv_obj_set_style_text_color(lbl_led_state, lv_color_hex(COLOR_MUTED), 0);
  lv_obj_align(lbl_led_state, LV_ALIGN_TOP_MID, 0, 130);

  lbl_rgb = lv_label_create(scr_led);
  lv_label_set_text(lbl_rgb, "RGB 255,0,68  BRI 200");
  lv_obj_align(lbl_rgb, LV_ALIGN_TOP_MID, 0, 155);

  make_slider_row(scr_led, "Red", 190, uiR, &slider_r);
  make_slider_row(scr_led, "Green", 230, uiG, &slider_g);
  make_slider_row(scr_led, "Blue", 270, uiB, &slider_b);
  make_slider_row(scr_led, "Brightness", 310, uiBri, &slider_bri);

  lbl_status_led = lv_label_create(scr_led);
  lv_label_set_text(lbl_status_led, "ESP-NOW: waiting");
  lv_obj_add_style(lbl_status_led, &style_status, 0);
  lv_obj_align(lbl_status_led, LV_ALIGN_BOTTOM_MID, 0, -16);

  updateLedButton();
}

void setup() {
#ifdef ARDUINO_USB_CDC_ON_BOOT
  delay(2000);
#endif
  Serial.begin(115200);
  Serial.println();
  Serial.println("=====================================");
  Serial.println("Showduino UI - RGB 800x480 ESP32-S3");
  Serial.printf("Board: %s\n", BOARD_NAME);
  Serial.println("=====================================");

  smartdisplay_init();
  smartdisplay_lcd_set_backlight(0.85f);

  init_theme();
  create_main_menu();
  create_led_screen();
  lv_screen_load(scr_main);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false);
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.print("[WiFi] UI STA MAC: ");
  printMac(mac);
  Serial.println();

  const bool ok = initEspNow();
  Serial.printf("[ESPNOW] begin -> %s\n", ok ? "OK" : "FAIL");
  setStatus(lbl_status_main, ok ? "ESP-NOW ready — tap LED Control" : "ESP-NOW init failed");
}

static uint32_t lv_last_tick = 0;

void loop() {
  const uint32_t now = millis();
  lv_tick_inc(now - lv_last_tick);
  lv_last_tick = now;
  lv_timer_handler();

  const uint32_t SEND_INTERVAL_MS = 80;
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    if (pendingSendToggle) {
      pendingSendToggle = false;
      lastSendMs = now;
      sendToggle();
    } else if (pendingSendRGB) {
      pendingSendRGB = false;
      lastSendMs = now;
      sendRGB(uiR, uiG, uiB, uiBri);
    }
  }

  delay(2);
}
