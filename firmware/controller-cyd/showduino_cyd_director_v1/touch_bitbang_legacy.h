/*
  ONLY USE THIS IF YOU MUST KEEP YOUR OLD DIRECTOR SKETCH.
  For the 5" 800x400 panel, prefer the full replacement .ino in this folder.

  XPT2046_Bitbang uses Point + getTouch(), NOT touched()/TS_Point/getPoint().
*/

#pragma once
#include <XPT2046_Bitbang.h>

// CYD 2.4" pins — NOT for 5" RGB panel
#define TOUCH_MOSI 32
#define TOUCH_MISO 39
#define TOUCH_CLK  25
#define TOUCH_CS   33

#ifndef TFT_WIDTH
#define TFT_WIDTH 800
#endif
#ifndef TFT_HEIGHT
#define TFT_HEIGHT 400
#endif

extern XPT2046_Bitbang touch;

// Replace your broken getTouchPoint() with this:
inline bool getTouchPoint(int16_t &x, int16_t &y) {
  Point p = touch.getTouch();
  // Bitbang library has no touched() — treat center idle as no-touch
  if (p.x <= 1 && p.y <= 1) return false;
  x = (int16_t)p.x;
  y = (int16_t)p.y;
  return true;
}

/*
  DELETE from your old .ino:
    if (!touch.touched()) return false;
    TS_Point p = touch.getPoint();
    x = map(p.x, 200, 3900, 0, 320);

  REPLACE with:
    Point p = touch.getTouch();
    if (p.x <= 1 && p.y <= 1) return false;
    x = p.x;
    y = p.y;
    return true;
*/
