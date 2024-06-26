#include <ArduinoBLE.h>
#include <Arduino_GFX_Library.h>

#include "FreeMono8pt7b.h"
#include "FreeMono24pt7b.h"
#include "combustionLogo.h"

#define GFX_DEV_DEVICE WAVESHARE_ESP32_S3_TFT_4_3
#define _BackgroundColor BLACK  //blue
#define _FontColor WHITE
//define GFX_BL 2

/*TODO
- figure out how to clear the screen to move to next mode and prevent screen flashing because we are clearing the entire display each time and it's in a loop
- when found, display CPT Found, initializing.
- work on state when found back to scanning

*/

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

//cpt vars START
float CoreCurrentTemp = 0;
float SurfaceCurrentTemp = 0;
float AmbientCurrentTemp = 0;
float InstantReadTemp = 0;

float TEightTemp = 0;

uint8_t probeStatusData[48] = {};

struct __attribute__((packed)) PackedProbeTemperatures {
  unsigned int temperature1 : 13;
  unsigned int temperature2 : 13;
  unsigned int temperature3 : 13;
  unsigned int temperature4 : 13;
  unsigned int temperature5 : 13;
  unsigned int temperature6 : 13;
  unsigned int temperature7 : 13;
  unsigned int temperature8 : 13;
};

struct __attribute__((packed)) PackedModeID {
  unsigned int probemode : 2;
  unsigned int colorid : 3;
  unsigned int probeid : 3;
};

struct __attribute__((packed)) PackedVirtualSensors {
  unsigned int batterystatus : 1;
  unsigned int coresensor : 3;
  unsigned int surfacesensor : 2;
  unsigned int ambientsensor : 2;
};

struct __attribute__((packed)) PackedPredictionStatus {
  unsigned int predictionstate : 4;
  unsigned int predictionmode : 2;
  unsigned int predictiontype : 2;
  unsigned int predictionsetpointtemperature : 10;
  unsigned int heatstarttemperature : 10;
  unsigned int predictionvalueseconds : 17;
  unsigned int estimatedcoretemperature : 11;
};

struct __attribute__((packed)) PackedFoodSafeData {
  unsigned int foodsafemode : 3;
  unsigned int foodsafeproduct : 10;
  unsigned int foodsafeserving : 3;
  unsigned int foodsafethresholdreftemp : 13;
  unsigned int foodsafezvalue : 13;
  unsigned int foodsafereferencetemp : 13;
  unsigned int foodsafedvalueatrt : 13;
  unsigned int foodsafetargetlogreduction : 8;
  unsigned int foodsafedatapad : 4;
};

struct __attribute__((packed)) PackedFoodSafeStatus {
  unsigned int foodsafestate : 3;
  unsigned int foodsafelogreduction : 8;
  unsigned int foodsafesecondsabovethreshold : 16;
  unsigned int foodsafelogsequencenumber : 32;
  unsigned int foodsafestatuspad : 5;
};

struct __attribute__((packed)) StatusData {
  uint32_t longRangeMin;
  uint32_t longRangeMax;
  PackedProbeTemperatures packedTemperatures;
  PackedModeID packedMode;
  PackedVirtualSensors packedSensors;
  PackedPredictionStatus packedPrediction;
  PackedFoodSafeData packedFSdata;
  PackedFoodSafeStatus packedFSstatus;
};

int CPTmode = 7;
int BatStat = 0;
int CoreID = 0;
int SurfID = 0;
int AmbiID = 0;
float CPT_RAY[9];

int PredState = 0;
int PredMode = 0;
int PredType = 0;
float PredSetPointT;
float PredHeatStartT;
int PredSeconds = 0;
float PredCoreEst;
float PredPercent;

int FoodMode = 0;
float FoodTarget;
int FoodState = 0;
float FoodLog;

bool CPTscanning = false;
bool CPTconnected = false;
bool CPTdiscovered = false;
bool CPTsubscribed = false;

int UIpage = 1;
bool ButtPush = false;
unsigned long DeBounce = millis();
bool BLE_UI = false;
bool TFTbl = false;
bool LEDred = false;
//cpt vars END

//my vars START
int CurrentScreenDisplayState = 0; //1=starting up, 2 = scan, 3 = instant, 4 = predict
//my vars END

bool GFXinit() {
  Serial.println("GFX init...");

  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
    return false;
  }
  //set font
  gfx->setFont(&FreeMono8pt7b);
  //.setFont(&FreeMonoBoldOblique12pt7b);

  gfx->fillScreen(_BackgroundColor);
  //BLset(HIGH); //does not work TODO research

  // gfx->setTextColor(WHITE);
  // gfx->setTextSize(4);
  // gfx->setCursor(250, 200);
  // gfx->println("Hello world");

  return true;
}

void animate(int wait) {
  int x = random(1, 700);
  int y = random(1, 350);
  showText(x, y, 1, "Hello World", false);
  delay(wait);
  showText(x, y, 1, "Hello World", true);
}

float convertCToF(float celcius) {
  return ((celcius * 9 / 5) + 32);
}

void showText(int x, int y, int size, String msg, bool clear) {
  gfx->setFont(&FreeMono8pt7b);
  if (clear)
    gfx->setTextColor(_BackgroundColor);
  else
    gfx->setTextColor(_FontColor);
  gfx->setTextSize(size);
  gfx->setCursor(x, y);
  gfx->println(msg);
}

void showTextLarge(int x, int y, int size, String msg, bool clear) {
  gfx->setFont(&FreeMono24pt7b);
  if (clear)
    gfx->setTextColor(_BackgroundColor);
  else
    gfx->setTextColor(_FontColor);
  gfx->setTextSize(size);
  gfx->setCursor(x, y);
  gfx->println(msg);
}

void blinkAsterisks() {
  showText(700, 20, 1, "*", false);
  delay(1000);
  showText(700, 20, 1, "*", true);
  delay(500);
}

void GFXdraw16bitRGBBitmap(int16_t x1, int16_t y1, uint16_t* full, int16_t w, int16_t h) {
  gfx->draw16bitRGBBitmap(x1, y1, full, w, h);
}

void GFXflush() {
  gfx->flush();
}

void DisplayFrame(int color) {
  int reduction = 1;
  for (int i = 1; i <= 5; i++) { //thickness
    gfx->drawRoundRect(i, i, 800-reduction, 480-reduction, 10, color);
    reduction = reduction+2;
  }

  // gfx->drawRoundRect(1, 1, 799, 479, 10, color); //800-1
  // gfx->drawRoundRect(2, 2, 797, 477, 10, color); //800-3
  // gfx->drawRoundRect(3, 3, 795, 475, 10, color); //800-5
  // gfx->drawRoundRect(4, 4, 796, 476, 10, color);
  // gfx->drawRoundRect(5, 5, 795, 475, 10, color);

  // gfx->fillRoundRect(1, 1, 800, 480, 10, color);
  // gfx->fillRoundRect(10, 10, 780, 460, 10, _BackgroundColor);
}

// void ClearDisplay() {
//   gfx->fillRoundRect(10, 10, 780, 460, 10, _BackgroundColor);
// }

void FlashFrame(int color, int waitTime, int numberOfFlash) {
  for (int i = 0; i <= numberOfFlash; i++) {
    DisplayFrame(color);
    delay(waitTime);
  }
}

void ClearDisplay(int state) {
  if (CurrentScreenDisplayState != state){
    gfx->fillScreen(_BackgroundColor);
    CurrentScreenDisplayState = state;
  }
}


void displayTemps() {
  ClearDisplay(3);
  DisplayFrame(GREEN);
  displayProbeMode();
  displayInstantTemp();
  if (!CPTmode)  //CPTMode-1=instant, 0 = predict
  {
    displaySurfaceTemp();
    displayAmbientTemp();
  } else {
    //clear non-instant temps
    gfx->fillRect(25, 225, 750, 100, _BackgroundColor);
    GFXflush();
  }
}

void displayProbeMode() {
  int y = 45;
  gfx->fillRect(175, y - 1, 300, 30, _BackgroundColor);
  GFXflush();
  if (CPTmode)  //CPTMode-1=instant, 0 = predict
    showTextLarge(175, y, 1, "Instant Mode", false);
  else
    showTextLarge(175, y, 1, "Predict Mode", false);
}

void displayInstantTemp() {
  int y = 210;
  gfx->fillRect(25, y - 30, 500, 40, _BackgroundColor);
  GFXflush();
  showTextLarge(25, y, 1, "Instant :" + String(convertCToF(InstantReadTemp)), false);
}

void displaySurfaceTemp() {
  int x = 250;
  gfx->fillRect(1, x - 25, 300, 30, _BackgroundColor);
  GFXflush();
  showText(1, x, 2, "Surface S" + String(SurfID) + " :" + String(convertCToF(SurfaceCurrentTemp)), false);
}

void displayAmbientTemp() {
  float ACTfx = AmbientCurrentTemp * 10;
  float ACTfr = round(ACTfx);
  float ACTf = ACTfr / 10;
  int ACTi = (int)(ACTf);
  int ACTic = (int)(AmbientCurrentTemp * 100);
  // if (ACTic < 9995) {
  //   canvas.setCursor(105, 121);
  // } else {
  //   canvas.setCursor(76, 121);
  // }
  // canvas.print(ACTi);

  int x = 290;
  gfx->fillRect(1, x - 25, 300, 30, _BackgroundColor);
  GFXflush();
  showText(1, x, 2, "Ambient S" + String(AmbiID) + " :" + String(convertCToF(AmbientCurrentTemp)), false);
}

void readCPTvalue(BLECharacteristic characteristic) {
  Serial.println("in readCPTvalue");
  characteristic.read();
  characteristic.readValue(&probeStatusData, 48);

  showText(1, 150, 1, "readCPT!!!", false);

  StatusData* statusData = reinterpret_cast<StatusData*>(probeStatusData);

  int32_t t1_c = (int32_t)(statusData->packedTemperatures.temperature1 * 5) - 2000;
  CPT_RAY[1] = (float)(t1_c) / 100.0;

  int32_t t2_c = (int32_t)(statusData->packedTemperatures.temperature2 * 5) - 2000;
  CPT_RAY[2] = (float)(t2_c) / 100.0;

  int32_t t3_c = (int32_t)(statusData->packedTemperatures.temperature3 * 5) - 2000;
  CPT_RAY[3] = (float)(t3_c) / 100.0;

  int32_t t4_c = (int32_t)(statusData->packedTemperatures.temperature4 * 5) - 2000;
  CPT_RAY[4] = (float)(t4_c) / 100.0;

  int32_t t5_c = (int32_t)(statusData->packedTemperatures.temperature5 * 5) - 2000;
  CPT_RAY[5] = (float)(t5_c) / 100.0;

  int32_t t6_c = (int32_t)(statusData->packedTemperatures.temperature6 * 5) - 2000;
  CPT_RAY[6] = (float)(t6_c) / 100.0;

  int32_t t7_c = (int32_t)(statusData->packedTemperatures.temperature7 * 5) - 2000;
  CPT_RAY[7] = (float)(t7_c) / 100.0;

  int32_t t8_c = (int32_t)(statusData->packedTemperatures.temperature8 * 5) - 2000;
  CPT_RAY[8] = (float)(t8_c) / 100.0;

  CPTmode = (int32_t)(statusData->packedMode.probemode);

  BatStat = (int32_t)(statusData->packedSensors.batterystatus);
  CoreID = (int32_t)(statusData->packedSensors.coresensor + 1);
  SurfID = (int32_t)(statusData->packedSensors.surfacesensor + 4);
  AmbiID = (int32_t)(statusData->packedSensors.ambientsensor + 5);

  Serial.println((String)CPTmode + " " + (String)BatStat + " " + (String)CoreID + " " + (String)SurfID + " " + (String)AmbiID);
  Serial.println("CPT 1 " + (String)CPT_RAY[1]);
  Serial.println("CPT 2 " + (String)CPT_RAY[2]);
  Serial.println("CPT 3 " + (String)CPT_RAY[3]);
  Serial.println("CPT 4 " + (String)CPT_RAY[4]);
  Serial.println("CPT 5 " + (String)CPT_RAY[5]);
  Serial.println("CPT 6 " + (String)CPT_RAY[6]);
  Serial.println("CPT 7 " + (String)CPT_RAY[7]);
  Serial.println("CPT 8 " + (String)CPT_RAY[8]);
  //Serial.println("probeStatusData " + (String)probeStatusData);
  Serial.println("t3_c " + (String)t3_c);



  PredState = (int32_t)(statusData->packedPrediction.predictionstate);
  PredMode = (int32_t)(statusData->packedPrediction.predictionmode);
  PredType = (int32_t)(statusData->packedPrediction.predictiontype);

  int32_t pspt = (int32_t)(statusData->packedPrediction.predictionsetpointtemperature);
  PredSetPointT = (float)(pspt) / 10.0;

  int32_t phst = (int32_t)(statusData->packedPrediction.heatstarttemperature);
  PredHeatStartT = (float)(phst) / 10.0;

  PredSeconds = (int32_t)(statusData->packedPrediction.predictionvalueseconds + 1);

  int32_t pect = (int32_t)(statusData->packedPrediction.estimatedcoretemperature) - 200;
  PredCoreEst = (float)(pect) / 10.0;

  PredPercent = 100.0 * ((PredCoreEst - PredHeatStartT) / (PredSetPointT - PredHeatStartT));

  FoodMode = (int32_t)(statusData->packedFSdata.foodsafemode);
  int32_t targetfood = (int32_t)(statusData->packedFSdata.foodsafetargetlogreduction);
  FoodTarget = (float)(targetfood) / 10.0;

  FoodState = (int32_t)(statusData->packedFSstatus.foodsafestate);
  int32_t loggedfood = (int32_t)(statusData->packedFSstatus.foodsafelogreduction);
  FoodLog = (float)(loggedfood) / 10.0;

  CoreCurrentTemp = CPT_RAY[CoreID];
  SurfaceCurrentTemp = CPT_RAY[SurfID];
  AmbientCurrentTemp = CPT_RAY[AmbiID];  //CPT_RAY[AmbiID];
  InstantReadTemp = CPT_RAY[1];

  showText(1, 175, 1, "starting to check ", false);

  if (true) {
    displayTemps();
    
    //showText(1,200,2,"Got Temp :" + String(InstantReadTemp), false);

    // canvas.fillScreen(ST77XX_BLACK);
    // if (CPTmode == 1) {
    //   DrawBezel(0xB5B6);
    //   canvas.setFont(&FreeMonoBold12pt7b);
    //   canvas.setTextColor(ST77XX_WHITE);
    //   canvas.setCursor(22, 40);
    //   canvas.print("Instant Read");
    //   canvas.setFont(&FreeMonoBold24pt7b);
    //   if (InstantReadTemp < 99.995) {
    //     canvas.setCursor(38, 95);
    //   } else {
    //     canvas.setCursor(22, 95);
    //   }
    //   canvas.print(InstantReadTemp);

    // } else if (CPTmode == 0) {
    //   DrawBezel(ST77XX_YELLOW);
    //   PageDisplay();
    // }
    // BattDisplay();
    // tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
  }
}

void CPTdiscoveredHandler(BLEDevice peripheral) {
  showText(1, 450, 1, "in CPTdiscoveredHandler", false);
  BLE.stopScan();
  CPTscanning = false;
  showText(1, 65, 1, "Found CPT,Connecting ...", false);
  peripheral.connect();
  delay(2000);
  if (!peripheral.connected()) {
    showTextLarge(1, 450, 1, "CPT Connection,Failure", false);
    while (!CPTconnected) {
      blinkAsterisks();
    }
  }
  CPTconnected = true;
  showText(1, 85, 1, "CPT Connected, discovering, service", false);

  while (!CPTdiscovered) {
    if (peripheral.discoverService("00000100-caab-3792-3d44-97ae51c1407a")) {
      showText(1, 450, 1, "Service Discovered", false);
      CPTdiscovered = true;
    } else {
      showText(1, 450, 1, "Update Service Undiscovered", false);
      peripheral.disconnect();
      delay(250);
      showText(1, 450, 1, "Retrying...", false);
      peripheral.connect();
      delay(500);
    }
  }
  BLEService service = peripheral.service("00000100-caab-3792-3d44-97ae51c1407a");
  BLECharacteristic characteristic = service.characteristic("00000101-caab-3792-3d44-97ae51c1407a");
  if (characteristic.canRead()) {
    characteristic.read();
  }

  delay(500);

  if (characteristic.canSubscribe()) {
    showText(1, 105, 1, "Subscribing...", false);
    while (!CPTsubscribed) {
      if (characteristic.subscribe()) {
        showText(1, 125, 1, "SUBSCRIBED", false);
        CPTsubscribed = true;
      } else {
        showText(1, 450, 1, "Subscription Failed", false);
        delay(500);
        showText(1, 450, 1, "Retrying Subscription...", false);
        characteristic.subscribe();
        delay(500);
      }
    }
  }

  // delay(1000);
  // canvas.fillScreen(ST77XX_BLACK);
  // canvas.setFont(&FreeMonoBold12pt7b);
  // DrawBezel(0xB5B6);
  // TODO BattDisplay();
  // tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);

  while (peripheral.connected()) {
    //TODO UIButtonCheck();

    BLEService service = peripheral.service("00000100-caab-3792-3d44-97ae51c1407a");
    BLECharacteristic characteristic = service.characteristic("00000101-caab-3792-3d44-97ae51c1407a");
    if (characteristic.valueUpdated()) {
      // DrawBezel(ST77XX_BLUE);
      // tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
      blinkAsterisks();
      readCPTvalue(service.characteristic("00000101-caab-3792-3d44-97ae51c1407a"));
    }
    delay(10);
  }

  CPTconnected = false;
  CPTdiscovered = false;
  CPTsubscribed = false;
  // canvas.fillScreen(ST77XX_BLACK);
  // tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
  TFTbl = true;
  // digitalWrite(TFT_BACKLITE, TFTbl);
}


void SetupCPT() {
  //showText(1,20,1,"Starting BLE, setupCPT...",false);
  Serial.println("Starting BLE, setupCPT...");

  if (!BLE.begin()) {
    showText(1, 450, 1, "BLE module failed!", false);
    Serial.println("BLE module failed!");
    while (!BLE_UI) {
      blinkAsterisks();
    }
  }
  BLE_UI = true;
  //showText(1,45,1,"BLE Ready", false);
  Serial.println("BLE Ready, calling BLE.setEventHandler");
  BLE.setEventHandler(BLEDiscovered, CPTdiscoveredHandler);
}

void DisplayStartup() {
  ClearDisplay(1);
  DisplayFrame(BLUE);
  for (int i = 0; i <= 5; i++) {
    showTextLarge(200, 200, 1, "Starting up...", false);
    delay(250);
    showTextLarge(200, 200, 1, "Starting up...", true);
    delay(450);
  }
}

void DisplayCombustionLogo(int x, int y, int width, int height, int color){
    gfx->drawBitmap(x,y, CombustionLogoBitmap, width, height, color);
}

void DisplayScanCPT() {
  ClearDisplay(2);
  DisplayFrame(YELLOW);
  for (int i = 0; i <= 3; i++) {
    showTextLarge(150, 375, 1, "Scanning for CPT...", false);
    DisplayCombustionLogo(300,100, 200, 200, RED);
    delay(250);
    DisplayCombustionLogo(300,100, 200, 200, ORANGE);
    //showTextLarge(150, 50, 1, "Scanning for CPT...", true);
    delay(450);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("start setup");
  GFXinit();
  DisplayStartup();
  SetupCPT();
}

void loop() {
  Serial.println("start loop");
  //blinkAsterisks();
  //cpt stuff
  if (!CPTscanning) {
    //ENTER YOUR CPT MAC ADDRESS HERE...
    BLE.scanForAddress("c2:71:17:61:f9:d0");  //("a0:b1:c2:d3:e4:f5");
    CPTscanning = true;
  }
  Serial.println("BLE Polling");
  DisplayScanCPT();
  BLE.poll();
  delay(100);
}