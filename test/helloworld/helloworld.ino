#include <Arduino_GFX_Library.h>

#include "FreeMono8pt7b.h"
#include "FreeSans8pt7b.h"

#define GFX_DEV_DEVICE WAVESHARE_ESP32_S3_TFT_4_3
#define _BackgroundColor 0x003030 //blue
#define _FontColor WHITE
//define GFX_BL 2

Arduino_ESP32RGBPanel* rgbpanel = new Arduino_ESP32RGBPanel(
  5 /* DE */,
  3 /* VSYNC */,
  46 /* HSYNC */,
  7 /* PCLK */,

  1 /* R0 */,
  2 /* R1 */,
  42 /* R2 */,
  41 /* R3 */,
  40 /* R4 */,

  39 /* G0 */,
  0 /* G1 */,
  45 /* G2 */,
  48 /* G3 */,
  47 /* G4 */,
  21 /* G5 */,

  14 /* B0 */,
  38 /* B1 */,
  18 /* B2 */,
  17 /* B3 */,
  10 /* B4 */,

  0 /* hsync_polarity */, 40 /* hsync_front_porch */, 48 /* hsync_pulse_width */, 88 /* hsync_back_porch */,
  0 /* vsync_polarity */, 13 /* vsync_front_porch */, 3 /* vsync_pulse_width */, 32 /* vsync_back_porch */,
  1 /* pclk_active_neg */, 16000000 /* prefer_speed */
);

//void GFXdraw16bitBeRGBBitmap(int16_t x1, int16_t y1, uint16_t* full, int16_t w, int16_t h);

Arduino_RGB_Display* gfx = new Arduino_RGB_Display(
  800 /* width */,
  480 /* height */,
  rgbpanel,
  0 /* rotation */,
  true /* auto_flush */
);

bool GFXinit() {
  Serial.println("GFX init...");

  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
    return false;
  }
  //set font
  gfx->setFont(&FreeSans8pt7b);
  //.setFont(&FreeMonoBoldOblique12pt7b);

  gfx->fillScreen(_BackgroundColor);
  //BLset(HIGH); //does not work

  // gfx->setTextColor(WHITE);
  // gfx->setTextSize(4);
  // gfx->setCursor(250, 200);
  // gfx->println("Hello world");

  return true;
}

void animate(int wait){
  int x = random(1,700);
  int y = random(1,350);
  showText(x,y,1,"Hello World",false);
  delay(wait);
  showText(x,y,1,"Hello World",true);
}

void printText(int16_t x1, int16_t y1)
{
  gfx->setTextColor(_FontColor);
  gfx->setTextSize(2);
  gfx->setCursor(x1, y1);
  gfx->println("Hello world");
}

void clearText(int16_t x1, int16_t y1)
{
  gfx->setTextColor(_BackgroundColor);
  gfx->setTextSize(2);
  gfx->setCursor(x1, y1);
  gfx->println("Hello world");
}

void showText(int x, int y, int size, String msg, bool clear){
 if(clear)
    gfx->setTextColor(_BackgroundColor);
  else
    gfx->setTextColor(_FontColor);
  gfx->setTextSize(size);
  gfx->setCursor(x, y);
  gfx->println(msg);
}


void GFXdraw16bitRGBBitmap(int16_t x1, int16_t y1, uint16_t* full, int16_t w, int16_t h) {
  gfx->draw16bitRGBBitmap(x1, y1, full, w, h);
}

void GFXflush() {
  gfx->flush();
}

void setup()
{
  GFXinit();
}

void loop()
{
  printText(100,100);
  animate(500);
}