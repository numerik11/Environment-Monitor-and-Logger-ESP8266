/*********************************************************
 * Environment Monitor (ESP8266 + TFT + DHT11/Senseair S8 or SCD30)
 * - 4-min slots (360/day), LittleFS persistence
 * - Web UI: 30-day history + Today (live)
 *
 * Button (NC to GND):
 *  - Not pressed: pin LOW (grounded)
 *  - Pressed: contact opens, pin HIGH (INPUT_PULLUP)
 *  - Interrupt on RISING (LOW->HIGH)
 *
 * Long-press:
 *  - Hold 5s -> shows "Force 400 ppm?" confirm overlay
 *  - Then short-press within 5s to confirm calibration
 *********************************************************/

#include <Arduino.h>
#include "screen_schedule.h"
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>
#include <lwip/apps/sntp.h>
#include <time.h>
#include <sys/time.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_SCD30.h>
#include <DHT.h>
#include <SoftwareSerial.h>

#if ARDUINO >= 100
  #include <stdint.h>
#endif

// ===================== Config =====================
#define ENABLE_FS 1
#ifndef USE_DHT_S8
#define USE_DHT_S8 0 // SCD30 on I2C; set to 1 for DHT11 plus Senseair S8 UART
#endif
#define SCD30_ASC 1
#define DEBUG_HEAP 0

#ifndef ENABLE_BL_HTTP
  #define ENABLE_BL_HTTP 0
#endif
#ifndef AP_FALLBACK_SSID
  #define AP_FALLBACK_SSID "EnvMonitor-Setup"
#endif
#ifndef AP_FALLBACK_PASS
  #define AP_FALLBACK_PASS ""
#endif
#ifndef WIFI_STA_RETRY_MS
  #define WIFI_STA_RETRY_MS 30000UL
#endif
#ifndef FAN_DIRECT_SSID
  #define FAN_DIRECT_SSID "espcontrol"
#endif
#ifndef FAN_DIRECT_PASSWORD
  #define FAN_DIRECT_PASSWORD "123456"
#endif

#if ENABLE_FS
  #include <FS.h>
  #include <LittleFS.h>
#endif

// ===================== Pins (D1 R1) =====================
#define TFT_CS      D8
#define TFT_DC      D2
#define I2C_SDA     D4
#define I2C_SCL     D15
#if USE_DHT_S8
#define BUTTON_PIN  A0
#define BL_PIN      D3
#define DHT_PIN     D6
#define CO2_RX_PIN  D4 // ESP RX connects to S8 TX
#define CO2_TX_PIN  D0 // ESP TX connects to S8 RX
#else
#define BUTTON_PIN  D0
#define BL_PIN      D6
#endif

#define NUM_SCREENS 6

// ===================== Backlight control (ESP8266 PWM) =====================
static const uint16_t BL_RANGE  = 1023;
static const uint16_t BL_PWM_HZ = 20000;
static uint16_t blValue = 750;
static unsigned long _blLastReapply = 0;
static bool scheduleEnabled=false, scheduledOff=false;
static uint16_t scheduleOff=22*60, scheduleOn=7*60;
static void applyScheduledBacklight();
volatile bool nextScreenReq = false;

// ===================== Wi-Fi / TZ =====================
ESP8266WebServer server(80);
DNSServer dnsServer;

static const char* TZ_ADELAIDE = "ACST-9:30ACDT-10:30,M10.1.0/2,M4.1.0/3";

// ===================== TFT =====================
Adafruit_ILI9341 ili9341(TFT_CS, TFT_DC);
Adafruit_ST7789 st7789(TFT_CS, TFT_DC, -1);
static Adafruit_SPITFT* activeTft = &ili9341;
#define tft (*activeTft)
static uint8_t displayType = 0; // 0: ILI9341 240x320, 1: ST7789 170x320
static int16_t SCREEN_WIDTH  = 320;
static int16_t SCREEN_HEIGHT = 240;

#define RGB565(r,g,b) (((r&0xF8)<<8)|((g&0xFC)<<3)|((b)>>3))
// Deeper, more contrasty base — the old 12/14/18 was too close to COL_CARD
static const uint16_t COL_BG        = RGB565( 8, 10, 14);
static const uint16_t COL_CARD      = RGB565(22, 26, 33);
static const uint16_t COL_CARD_HI   = RGB565(30, 35, 44);  // card header strip
static const uint16_t COL_CARD_EDGE = RGB565(52, 60, 70);
static const uint16_t COL_GRID      = RGB565(56, 64, 76);
static const uint16_t COL_GRID_MAJ  = RGB565(80, 90,104);  // thicker major grid
static const uint16_t COL_TEXT      = ILI9341_WHITE;
static const uint16_t COL_MUTED     = RGB565(150,162,178);
static const uint16_t COL_ACCENT    = RGB565(250,210, 80);
static const uint16_t COL_CO2       = RGB565(255,118, 88);
static const uint16_t COL_TEMP      = RGB565( 82,210,255);
static const uint16_t COL_HUM       = RGB565(108,204,170);
static const uint16_t COL_VALUE_BG  = RGB565(36, 42, 52);
static const uint16_t COL_GOOD      = RGB565( 72,168,124);
static const uint16_t COL_WARN      = RGB565(224,184, 64);
static const uint16_t COL_BAD       = RGB565(224, 84, 84);

// ===================== UI layout =====================
struct Panel { int16_t x,y,w,h; };
static Panel header = {0,0,SCREEN_WIDTH,36};

static const uint8_t UI_RADIUS = 8;
static const int UI_TAB_H              = 20;
static const int UI_GUTTER             = 8;
static const int UI_PAD_X              = 8;
static const int UI_SECTION_TITLE_GAP  = 4;
static const int UI_BOTTOM_MARGIN      = 8;
static const int UI_CARD_HEADER_H      = 20;
static const int UI_GRID_PAD_TOP       = 28;
static const int UI_GRID_PAD_BOTTOM    = 22;

// ===================== Plot helpers =====================
struct GridRect { int gx0,gx1,gy0,gy1; };
struct AxisScale { float vmin; float vmax; float step; int tickCount; float ticks[6]; };

// ===================== Sensor =====================
Adafruit_SCD30 scd30;
#if USE_DHT_S8
static DHT dht(DHT_PIN, DHT11);
static SoftwareSerial s8Serial(CO2_RX_PIN, CO2_TX_PIN);
static uint32_t lastDhtPoll=0, lastS8Request=0;
static bool s8Pending=false;
static uint8_t s8Response[7], s8ResponseLength=0;
static const char* s8Status="Waiting for S8 reply";
static uint32_t lastS8ErrorLog=0;
#endif
static uint32_t lastTempHumData=0, lastCo2Data=0;
static bool hasTempHum=false, hasCo2=false;
static bool fanSendEnabled=false;
static String fanHost="192.168.4.1";
static uint32_t fanBoot=0, fanClimateSeq=0, fanCo2Seq=0, fanLastAttempt=0;
static int fanClimateHttp=0, fanCo2Http=0;
static const char* FAN_SENSOR_KEY="GrowSensor2026";
static const char* AC_CONTROL_HOST="192.168.1.127";
static uint32_t acLastAttempt=0;
static int acLastHttp=0;

// ===================== Data buffers =====================
#define SLOT_MINUTES 4
static const int SLOTS_PER_DAY  = 1440 / SLOT_MINUTES;
static const int SLOTS_PER_12H  = (12 * 60) / SLOT_MINUTES;
static const int SLOTS_PER_24H  = (24 * 60) / SLOT_MINUTES;

static uint16_t co2Slots[SLOTS_PER_DAY];
static uint8_t  tempSlots[SLOTS_PER_DAY];
static uint8_t  humSlots [SLOTS_PER_DAY];

static float   decodeTemp(uint8_t v){ return (v / 2.0f) - 10.0f; }
static uint8_t encodeTemp(float t){ return constrain((int)((t+10.0f)*2.0f), 0, 255); }
static float   decodeHum (uint8_t v){ return v / 2.0f; }
static uint8_t encodeHum (float h){ return constrain((int)(h*2.0f), 0, 200); }

// ===================== Accumulation =====================
static int currentSlot = -1;
static uint32_t co2Sum=0, humSum=0;
static int32_t tempSum=0;
static uint16_t sampleCount=0, co2SampleCount=0;

static uint32_t bootTime=0;
static int lastDrawnSlot=-1;

static float currentCO2=NAN, currentTemp=NAN, currentHum=NAN;

static uint16_t slotsSince12hReset = 0;
static uint16_t slotsSince24hReset = 0;
static int twelveGraphResetOriginSlot     = -1;
static int twentyFourGraphResetOriginSlot = -1;

// ===================== Timing =====================
static uint32_t lastRedraw=0;
static const uint32_t REDRAW_INTERVAL=60000;

static uint32_t lastSensorData=0;
static uint32_t lastSensorAttempt=0, lastSensorPoll=0;
static bool sensorInitialized=false, sensorHasSample=false;
static const uint32_t SENSOR_TIMEOUT=30000;
#if !USE_DHT_S8
static String i2cDevices="none";
static uint8_t i2cDeviceCount=0;
static bool i2cAddress61Present=false;
#endif

static const uint32_t FAST_30M_REFRESH_MS = 30000;
static const uint32_t FAN_SEND_INTERVAL_MS = 8000;
static uint32_t lastFast30mDraw = 0;

// ===================== FS periodic save =====================
#if ENABLE_FS
static bool fsReady=false;
static uint32_t lastPeriodicSaveMs = 0;
static const uint32_t PERIODIC_SAVE_MS = 120000;
#endif

// ===================== State =====================
static bool offlineMode=false;
static bool manualTimeActive=false;
static bool manualClockRestored=false;
static uint32_t softStartTime=0;
static uint8_t screenMode=0;
static uint8_t screenRotation=1;
static bool wifiChangePending=false;
static uint32_t wifiChangeAt=0;
static String pendingSsid, pendingPassword;
static bool apFallbackActive = false;
static bool mdnsStarted = false;
static uint32_t lastStaRetryMs = 0;

static time_t   timeAnchorEpoch   = 0;
static uint32_t timeAnchorMillis  = 0;
static bool     timeAnchorValid   = false;

static uint32_t nextAnchorSave    = 0;
static const uint32_t ANCHOR_SAVE_MS = 600000;

// ===================== Forward decls =====================
static bool initSensor();
static void connectWiFi();
static void initTime();
static bool isTimeValid();
static void anchorTimeToNow();
static time_t approxNow();
static bool saveManualClock(time_t epoch);
static void loadManualClock();
static void startFallbackAP();
static void onStaConnected();
static void retryStaWhileAp();
static void loadFanSettings();
static void sendFanReadings();

static void resetAccumulators();
static void commitSlotData(int slot);
static void clearData();

static void drawAllScreens();
static void drawHeader();
static void drawTabs();
static void draw30MinDashboard();
static void drawCO2FullScreen();
static void drawTempHumFullScreen();
static void drawCO212Hour();
static void drawTempHum12Hour();
static void updateDashboardBadges();

static void handleWebRoot();
static void handleDayList();
static void handleDayData();
static void handleStatus();
static void handleToday();

// ===================== Yield helper =====================
static inline void yieldOften(uint16_t n){
  if ((n & 0x0F) == 0) yield();
}

// ===================== UI primitives =====================
static uint16_t blendColor(uint16_t c1, uint16_t c2, uint8_t ratio){
  uint8_t r1=(c1>>11)&0x1F, g1=(c1>>5)&0x3F, b1=c1&0x1F;
  uint8_t r2=(c2>>11)&0x1F, g2=(c2>>5)&0x3F, b2=c2&0x1F;
  auto lerp=[&](uint8_t a,uint8_t b){ return (uint8_t)((a*(100-ratio)+b*ratio)/100); };
  uint8_t r=lerp(r1,r2), g=lerp(g1,g2), b=lerp(b1,b2);
  return (r<<11)|(g<<5)|b;
}

static void drawCard(const Panel& p){
  tft.fillRoundRect(p.x, p.y, p.w, p.h, UI_RADIUS, COL_CARD);
  tft.drawRoundRect(p.x, p.y, p.w, p.h, UI_RADIUS, COL_CARD_EDGE);
}

// Colored strip at the top of the card, with rounded top corners.
static void drawCardHeader(const Panel& p, const char* title, uint16_t color){
  const int16_t x = p.x + 1, y = p.y + 1;
  const int16_t w = p.w - 2, h = UI_CARD_HEADER_H - 1;
  const int16_t radius = UI_RADIUS - 1;
  tft.fillRoundRect(x, y, w, h, radius, color);
  tft.fillRect(x, y + radius, w, h - radius, color);

  tft.setTextSize(1);
  tft.setTextColor(COL_BG, color);
  tft.setCursor(p.x + 8, p.y + (UI_CARD_HEADER_H - 8) / 2);
  tft.print(title);
}

// callbacks
void onBtnShort() {
  nextScreenReq = true;
}

static void printRightAligned(int16_t xRight, int16_t y, const char* txt, uint8_t sz, uint16_t fg, uint16_t bg){
  int w = (int)strlen(txt) * 6 * sz;
  tft.setTextSize(sz); tft.setTextColor(fg,bg);
  tft.setCursor(xRight - w, y); tft.print(txt);
}

static void drawModernValueBadge(int16_t xRight, int16_t y, const char* value, uint16_t textColor){
  int w = (int)strlen(value)*6 + 8;
  int h = 12;
  int x = xRight - w;
  tft.fillRoundRect(x,y,w,h,6,COL_VALUE_BG);
  tft.drawRoundRect(x,y,w,h,6,COL_GRID);
  tft.setTextSize(1);
  tft.setTextColor(textColor,COL_VALUE_BG);
  tft.setCursor(x+4,y+3);
  tft.print(value);
}

static void drawModernTrendIndicator(int16_t x, int16_t y, int8_t dir,
                                     uint16_t color, const char* deltaTxt = nullptr){
  if (dir == 0) return;
  const int triW = 7, triH = 6;
  if (dir > 0) tft.fillTriangle(x, y+triH, x+triW, y+triH, x+triW/2, y, color);
  else         tft.fillTriangle(x, y,       x+triW, y,       x+triW/2, y+triH, color);

  if (deltaTxt && *deltaTxt) {
    tft.setTextSize(1);
    tft.setTextColor(color, COL_CARD);
    tft.setCursor(x + triW + 3, y);
    tft.print(deltaTxt);
  }
}

static void drawOverlayBox(const char* title, const char* msg, uint16_t accent = COL_ACCENT) {
  const int w = SCREEN_WIDTH - 40;
  const int h = SCREEN_WIDTH < 240 ? 104 : 72;
  const int x = 20;
  const int y = (SCREEN_HEIGHT - h)/2;

  tft.fillRoundRect(x, y, w, h, 10, COL_CARD);
  tft.drawRoundRect(x, y, w, h, 10, accent);

  tft.setTextSize(strlen(title)*12U > (unsigned)(w-20) ? 1 : 2); tft.setTextColor(COL_TEXT, COL_CARD);
  tft.setCursor(x+10, y+10); tft.print(title);

  tft.setTextSize(1);
  const size_t charsPerLine = (w-20)/6;
  for (size_t i=0; msg[i]; ++i) {
    if (i % charsPerLine == 0) tft.setCursor(x+10, y+36+(i/charsPerLine)*10);
    tft.print(msg[i]);
  }
}

static void showToast(const char* msg, uint16_t color = COL_TEXT, uint32_t ms = 1200) {
  const int h = 22, x = 8, w = SCREEN_WIDTH - 16, y = SCREEN_HEIGHT - h - 6;
  tft.fillRoundRect(x, y, w, h, 8, COL_CARD);
  tft.drawRoundRect(x, y, w, h, 8, COL_GRID);
  tft.setTextSize(1); tft.setTextColor(color, COL_CARD);
  tft.setCursor(x+10, y+6); tft.print(msg);
  delay(ms);
}

// ===================== Graph helpers =====================
static GridRect gridRect(const Panel& p){
  // Compute needed left pad from the widest tick we might print.
  int labelChars = 4;
  if (p.w < 260) labelChars = 3;
  const int leftPad = 6 + labelChars * 6 + 4;

  GridRect g;
  g.gx0 = p.x + min(leftPad, max(32, p.w / 4));
  g.gx1 = p.x + p.w - 8;
  g.gy0 = p.y + UI_GRID_PAD_TOP;
  g.gy1 = p.y + p.h - UI_GRID_PAD_BOTTOM;
  return g;
}

static float niceNum(float x, bool round){
  if(x<=0) return 1;
  float expv=floorf(log10f(x)), f=x/powf(10.0f,expv), nf;
  if(round){
    if(f<1.5f) nf=1; else if(f<3.f) nf=2; else if(f<7.f) nf=5; else nf=10;
  }else{
    if(f<=1.f) nf=1; else if(f<=2.f) nf=2; else if(f<=5.f) nf=5; else nf=10;
  }
  return nf*powf(10.0f,expv);
}

static AxisScale computeAxisScale(float dataMin, float dataMax, float clampMin, float clampMax, int targetTicks=5){
  AxisScale s{};
  if(!isfinite(dataMin) || !isfinite(dataMax)){ dataMin=clampMin; dataMax=clampMax; }
  if(!(dataMax>dataMin)){
    const float pad=max(0.5f,(clampMax-clampMin)*0.04f);
    dataMin-=pad; dataMax+=pad;
  }
  clampMin=min(clampMin,dataMin);
  clampMax=max(clampMax,dataMax);
  float range=dataMax-dataMin;
  if(range<=0) range=(clampMax-clampMin);
  if(range<=0) range=1;

  float d=niceNum(range/(targetTicks-1), true);
  float gmin=floorf(dataMin/d)*d, gmax=ceilf(dataMax/d)*d;

  gmin=max(gmin, clampMin);
  gmax=min(gmax, clampMax);

  if(gmax-gmin<d*(targetTicks-1)){
    d=niceNum((gmax-gmin)/(targetTicks-1), true);
    if(d<=0) d=1;
  }

  s.vmin=gmin; s.vmax=gmax; s.step=d; s.tickCount=0;
  float v=gmin;
  for(int i=0;i<6 && v<=gmax+1e-4f; ++i, v+=d){ s.ticks[i]=v; s.tickCount++; }
  if(s.tickCount<2){ s.tickCount=2; s.ticks[0]=gmin; s.ticks[1]=gmax; }
  return s;
}

static int graphTickCount(const GridRect& g){
  return constrain((g.gy1-g.gy0)/16+1, 2, 5);
}

static int mapValueToY(float v, int gy0, int gy1, float vMin, float vMax){
  if(v<=vMin) return gy1;
  if(v>=vMax) return gy0;
  float t=(v-vMin)/(vMax-vMin);
  return gy1 - (int)(t*(gy1-gy0));
}

// Draws major/minor grid lines and labels. Prints the unit only on the top
// tick to avoid repeating "ppm"/"C"/"%" on every row.
static void drawYAxis(const Panel& p, const GridRect& g, const AxisScale& sc, const char* unit){
  tft.setTextSize(1);

  const bool majorEveryOther = (sc.tickCount > 3);

  for (int i = 0; i < sc.tickCount; i++){
    const float tv = sc.ticks[i];
    const int   y  = mapValueToY(tv, g.gy0, g.gy1, sc.vmin, sc.vmax);
    const bool  major = !majorEveryOther || (i % 2 == 0);
    const uint16_t gc = major ? COL_GRID_MAJ : COL_GRID;

    tft.drawFastHLine(g.gx0, y, g.gx1 - g.gx0, gc);

    char label[16];
    if (fabsf(sc.step - floorf(sc.step)) < 1e-3f)
      snprintf(label, sizeof(label), "%.0f", tv);
    else
      snprintf(label, sizeof(label), "%.1f", tv);

    tft.setTextColor(major ? COL_TEXT : COL_MUTED, COL_CARD);
    tft.setCursor(p.x + 4, y - 4);
    tft.print(label);

    if (i == sc.tickCount - 1 && unit && *unit) {
      tft.setTextColor(COL_MUTED, COL_CARD);
      tft.setCursor(p.x + 4, y - 4 - 9);
      tft.print(unit);
    }
  }
}

static void drawEmptyGraph(const GridRect& g){
  tft.setTextSize(1);
  tft.setTextColor(COL_MUTED, COL_CARD);
  const char* msg = "No data yet";
  const int w = (int)strlen(msg) * 6;
  tft.setCursor(g.gx0 + (g.gx1 - g.gx0 - w) / 2,
                (g.gy0 + g.gy1 - 8) / 2);
  tft.print(msg);
}

static void drawTimeGrid24h(int gx0,int gx1,int gy0,int gy1){
  const int hourStep=(gx1-gx0<150)?12:6;
  for(int h=0; h<=24; h+=hourStep){
    int x = gx0 + (int)((long)h*(gx1-gx0)/24L);
    tft.drawFastVLine(x, gy0, gy1-gy0, COL_GRID);
    char label[16]; snprintf(label, sizeof(label), "%02d", h);
    tft.setTextSize(1); tft.setTextColor(COL_MUTED, COL_CARD);
    tft.setCursor(constrain(x-6, gx0, gx1-12), gy1+5); tft.print(label);
  }
}

static void drawTimeGrid12h(int gx0,int gx1,int gy0,int gy1){
  const int hourStep=(gx1-gx0<150)?6:3;
  for(int h=0; h<=12; h+=hourStep){
    int x = gx0 + (int)((long)h*(gx1-gx0)/12L);
    tft.drawFastVLine(x, gy0, gy1-gy0, COL_GRID);
    char label[16]; snprintf(label, sizeof(label), "-%dh", 12-h);
    if(h==12) strcpy(label,"now");
    tft.setTextSize(1); tft.setTextColor(COL_MUTED, COL_CARD);
    tft.setCursor(constrain(x-9, gx0, gx1-18), gy1+5); tft.print(label);
  }
}

static void drawMiniTimeGrid(const GridRect& g){
  for(int i=0;i<=2;i++){
    const int x=g.gx0+i*(g.gx1-g.gx0)/2;
    tft.drawFastVLine(x,g.gy0,g.gy1-g.gy0,COL_GRID);
  }
  tft.setTextSize(1); tft.setTextColor(COL_MUTED,COL_CARD);
  tft.setCursor(g.gx0,g.gy1+5); tft.print("-30m");
  tft.setCursor(g.gx1-18,g.gy1+5); tft.print("now");
}

static void drawNowMarkerAndTag(int gx0,int gx1,int gy0,int gy1){
  if(currentSlot<0) return;
  const int x = gx0 + (int)((long)currentSlot * (gx1 - gx0) / (SLOTS_PER_DAY - 1));

  // Dashed vertical line
  for (int yy = gy0; yy < gy1; yy += 4) {
    tft.drawFastVLine(x, yy, min(2, gy1 - yy), COL_ACCENT);
  }

  // Small "now" pill at the top, kept inside the plot area
  const char* tag = "now";
  const int tagW = (int)strlen(tag) * 6 + 6;
  int tx = x - tagW / 2;
  if (tx < gx0) tx = gx0;
  if (tx + tagW > gx1) tx = gx1 - tagW;
  tft.fillRoundRect(tx, gy0 - 11, tagW, 10, 3, COL_ACCENT);
  tft.setTextSize(1);
  tft.setTextColor(COL_BG, COL_ACCENT);
  tft.setCursor(tx + 3, gy0 - 10);
  tft.print(tag);
}

static void formatSlotTimeHHMM(int slot, char* out, size_t n){
  int mins = slot * SLOT_MINUTES;
  int hh = (mins/60)%24;
  int mm = mins%60;
  snprintf(out,n,"%02d:%02d",hh,mm);
}

// ===================== Live preview into current slot =====================
static inline void writeLiveSlotPreviewToBuffers() {
  if (currentSlot < 0) return;
  if (co2SampleCount) co2Slots[currentSlot] = co2Sum / co2SampleCount;
  if (sampleCount) {
    tempSlots[currentSlot] = encodeTemp(tempSum / (float)sampleCount / 10.0f);
    humSlots[currentSlot] = encodeHum(humSum / (float)sampleCount / 10.0f);
  }
}

// ===================== Trend helpers (used by badges) =====================
static int findLatestValidU16(const uint16_t* a, int startIdx, int maxBack) {
  if (startIdx < 0) return -1;
  for (int k = 0; k <= maxBack; k++) {
    int idx = (startIdx - k + SLOTS_PER_DAY) % SLOTS_PER_DAY;
    if (a[idx] != 0) return idx;
  }
  return -1;
}
static int findPrevValidU16(const uint16_t* a, int fromIdx, int minBack, int maxBack) {
  if (fromIdx < 0) return -1;
  for (int k = minBack; k <= maxBack; k++) {
    int idx = (fromIdx - k + SLOTS_PER_DAY) % SLOTS_PER_DAY;
    if (a[idx] != 0) return idx;
  }
  return -1;
}
static int findLatestValidU8(const uint8_t* a, int startIdx, int maxBack) {
  if (startIdx < 0) return -1;
  for (int k = 0; k <= maxBack; k++) {
    int idx = (startIdx - k + SLOTS_PER_DAY) % SLOTS_PER_DAY;
    if (a[idx] != 0) return idx;
  }
  return -1;
}
static int findPrevValidU8(const uint8_t* a, int fromIdx, int minBack, int maxBack) {
  if (fromIdx < 0) return -1;
  for (int k = minBack; k <= maxBack; k++) {
    int idx = (fromIdx - k + SLOTS_PER_DAY) % SLOTS_PER_DAY;
    if (a[idx] != 0) return idx;
  }
  return -1;
}

static int8_t co2Trend() {
  if (currentSlot < 0) return 0;
  const int LOOKBACK_LATEST = 6;
  const int MIN_BACK = 6;
  const int MAX_BACK = 36;

  int iNow = findLatestValidU16(co2Slots, currentSlot, LOOKBACK_LATEST);
  if (iNow < 0) return 0;

  int iPrev = findPrevValidU16(co2Slots, iNow, MIN_BACK, MAX_BACK);
  if (iPrev < 0) return 0;

  int nowV  = (int)co2Slots[iNow];
  int prevV = (int)co2Slots[iPrev];
  int diff = nowV - prevV;

  const int DEAD = 25;
  if (diff > DEAD)  return +1;
  if (diff < -DEAD) return -1;
  return 0;
}

static int8_t u8Trend(const uint8_t* slots) {
  if (currentSlot < 0) return 0;

  const bool isTemp = (slots == tempSlots);
  const int LOOKBACK_LATEST = 6;
  const int MIN_BACK = 6;
  const int MAX_BACK = 36;

  int iNow = findLatestValidU8(slots, currentSlot, LOOKBACK_LATEST);
  if (iNow < 0) return 0;

  int iPrev = findPrevValidU8(slots, iNow, MIN_BACK, MAX_BACK);
  if (iPrev < 0) return 0;

  float nowV  = isTemp ? decodeTemp(slots[iNow]) : decodeHum(slots[iNow]);
  float prevV = isTemp ? decodeTemp(slots[iPrev]) : decodeHum(slots[iPrev]);
  float diff = nowV - prevV;

  const float DEAD = isTemp ? 0.3f : 1.0f;
  if (diff > DEAD)  return +1;
  if (diff < -DEAD) return -1;
  return 0;
}

// ===================== Smoothed line drawing =====================
#define GRAPH_SMOOTHING_CURVE     1
#define GRAPH_SMOOTHING_SUBSTEPS  4
#define GRAPH_SMOOTHING_PREAVG    1

// Single-pass Catmull: draws a shadow row below, then the crisp line on top.
// Halves the number of drawLine calls compared to a two-pass version.
static void drawSmoothCatmull(const int* xs, const int* ys, int n, uint16_t lineColor, int substeps){
  if (n < 2) return;

  const uint16_t shadow = blendColor(lineColor, COL_BG, 55);
  int xPrev = xs[0], yPrev = ys[0];
  int yMin = ys[0], yMax = ys[0];
  for (int i = 1; i < n; i++) { yMin = min(yMin, ys[i]); yMax = max(yMax, ys[i]); }

  auto crY = [&](int i, float t) -> float {
    int i0 = max(0, i-1), i1 = i, i2 = min(n-1, i+1), i3 = min(n-1, i+2);
    float p0 = ys[i0], p1 = ys[i1], p2 = ys[i2], p3 = ys[i3];
    float t2 = t*t, t3 = t2*t;
    return 0.5f * ((2*p1) + (-p0 + p2)*t
                 + (2*p0 - 5*p1 + 4*p2 - p3)*t2
                 + (-p0 + 3*p1 - 3*p2 + p3)*t3);
  };

  for (int i = 0; i < n - 1; ++i) {
    yieldOften(i);
    const int x1 = xs[i], x2 = xs[i+1];
    const int y1 = ys[i], y2 = ys[i+1];

    if (substeps <= 1) {
      tft.drawLine(xPrev, yPrev + 1, x2, y2 + 1, shadow);
      tft.drawLine(xPrev, yPrev,     x2, y2,     lineColor);
      xPrev = x2; yPrev = y2;
      continue;
    }
    for (int s = 1; s <= substeps; ++s) {
      if ((s & 1) == 0) yield();
      const float t  = (float)s / (float)substeps;
      const int   xi = (int)(x1 + t * (x2 - x1) + 0.5f);
      const int   yi = constrain((int)(crY(i, t) + 0.5f), yMin, yMax);
      tft.drawLine(xPrev, yPrev + 1, xi, yi + 1, shadow);
      tft.drawLine(xPrev, yPrev,     xi, yi,     lineColor);
      xPrev = xi; yPrev = yi;
    }
    (void)y1; (void)y2;
  }
}

static void drawSegmentedLine(const int* xs, const int* ys, int n, uint16_t lineColor) {
  if (n < 2) return;
#if GRAPH_SMOOTHING_CURVE
  if (n >= 4) {
    drawSmoothCatmull(xs, ys, n, lineColor, GRAPH_SMOOTHING_SUBSTEPS);
  } else {
    for (int i = 0; i < n - 1; ++i) {
      yieldOften(i);
      tft.drawLine(xs[i], ys[i], xs[i+1], ys[i+1], lineColor);
    }
  }
#else
  for (int i = 0; i < n - 1; ++i) {
    yieldOften(i);
    tft.drawLine(xs[i], ys[i], xs[i+1], ys[i+1], lineColor);
  }
#endif
}

// ===================== Mini graphs (30m cards) =====================
static void drawMiniGraphU16(const Panel& p, const uint16_t* buf, uint16_t lineColor,
                            uint16_t vminClamp, uint16_t vmaxClamp, const char* /*leftLbl*/, const char* unitLbl,
                            float /*currentValue*/, int minutesSpan, bool /*showBadge*/)
{
  GridRect g=gridRect(p);

  int slots = max(2, minutesSpan / SLOT_MINUTES);
  int end   = max(0, min(currentSlot, SLOTS_PER_DAY-1));
  int start = max(0, end-(slots-1));
  int N     = max(2, end-start+1);

  uint16_t mn=65535, mx=0; bool any=false;
  for(int i=start;i<=end;i++){
    uint16_t v=buf[i]; if(!v) continue;
    any=true; if(v<mn) mn=v; if(v>mx) mx=v;
  }
  float dmin= any? (float)mn : (float)vminClamp;
  float dmax= any? (float)mx : (float)vmaxClamp;
  AxisScale sc=computeAxisScale(dmin,dmax,(float)vminClamp,(float)vmaxClamp,graphTickCount(g));

  drawMiniTimeGrid(g);

  tft.setTextSize(1);
  for(int i=0;i<sc.tickCount;i++){
    float tv=sc.ticks[i];
    int y=mapValueToY(tv,g.gy0,g.gy1,sc.vmin,sc.vmax);
    tft.drawFastHLine(g.gx0, y, (g.gx1-g.gx0), COL_GRID);
    char bufTxt[20];
    bool isInt=fabsf(sc.step-floorf(sc.step))<1e-3f;
    if(isInt) snprintf(bufTxt,sizeof(bufTxt),"%.0f",tv);
    else      snprintf(bufTxt,sizeof(bufTxt),"%.1f",tv);
    tft.setTextColor(COL_MUTED, COL_CARD);
    tft.setCursor(p.x+2,y-4);
    tft.print(bufTxt);
  }
  (void)unitLbl;
  if(!any) drawEmptyGraph(g);

  static int xs[SLOTS_PER_DAY], ys[SLOTS_PER_DAY];
  int segLen=0, lastIdx=-9999;

  auto flush=[&](){ if(segLen>=2) drawSegmentedLine(xs, ys, segLen, lineColor); segLen=0; };
  auto pushPoint=[&](int idx, float v){
    int x=g.gx0 + ((idx - start) * (g.gx1 - g.gx0)) / (N - 1);
    int y=mapValueToY(v,g.gy0,g.gy1,sc.vmin,sc.vmax);
    xs[segLen]=x; ys[segLen]=y; segLen++;
  };

  for (int i=start; i<=end; ++i){
    uint16_t r=buf[i];
    if(!r){ flush(); lastIdx=-9999; continue; }
    float v=(float)r;
#if GRAPH_SMOOTHING_PREAVG
    float w=2.0f, acc=2.0f*v;
    if(i>start && buf[i-1]) { acc+=buf[i-1]; w+=1.0f; }
    if(i<end   && buf[i+1]) { acc+=buf[i+1]; w+=1.0f; }
    v=acc/w;
#endif
    if(i!=lastIdx+1 && segLen) flush();
    pushPoint(i,v); lastIdx=i;
  }
  flush();
}

static void drawMiniGraphU8(const Panel& p, const uint8_t* buf, uint16_t lineColor,
                           uint8_t vminClamp, uint8_t vmaxClamp, const char* /*leftLbl*/, const char* unitLbl,
                           float /*currentValue*/, float (*decode)(uint8_t), int minutesSpan,
                           bool /*showBadge*/)
{
  GridRect g=gridRect(p);

  int slots = max(2, minutesSpan / SLOT_MINUTES);
  int end   = max(0, min(currentSlot, SLOTS_PER_DAY-1));
  int start = max(0, end-(slots-1));
  int N     = max(2, end-start+1);

  float clampMin = (strcmp(unitLbl,"C")==0)?10.f:((strcmp(unitLbl,"%")==0)?20.f:(float)vminClamp);
  float clampMax = (strcmp(unitLbl,"C")==0)?40.f:((strcmp(unitLbl,"%")==0)?80.f:(float)vmaxClamp);

  float mn=1e9f, mx=-1e9f; bool any=false;
  for(int i=start;i<=end;i++){
    uint8_t r=buf[i]; if(!r) continue;
    float v=decode(r);
    any=true; if(v<mn) mn=v; if(v>mx) mx=v;
  }
  AxisScale sc=computeAxisScale(any?mn:clampMin, any?mx:clampMax, clampMin, clampMax, graphTickCount(g));

  drawMiniTimeGrid(g);

  tft.setTextSize(1);
  for(int i=0;i<sc.tickCount;i++){
    float tv=sc.ticks[i];
    int y=mapValueToY(tv,g.gy0,g.gy1,sc.vmin,sc.vmax);
    tft.drawFastHLine(g.gx0, y, (g.gx1-g.gx0), COL_GRID);
    char bufTxt[20];
    bool isInt=fabsf(sc.step-floorf(sc.step))<1e-3f;
    if(isInt) snprintf(bufTxt,sizeof(bufTxt),"%.0f%s",tv,unitLbl?unitLbl:"");
    else      snprintf(bufTxt,sizeof(bufTxt),"%.1f%s",tv,unitLbl?unitLbl:"");
    tft.setTextColor(COL_MUTED, COL_CARD);
    tft.setCursor(p.x+2,y-4);
    tft.print(bufTxt);
  }
  if(!any) drawEmptyGraph(g);

  static int xs[SLOTS_PER_DAY], ys[SLOTS_PER_DAY];
  int segLen=0, lastIdx=-9999;

  auto flush=[&](){ if(segLen>=2) drawSegmentedLine(xs, ys, segLen, lineColor); segLen=0; };
  auto pushPoint=[&](int idx, float v){
    int x=g.gx0 + ((idx - start) * (g.gx1 - g.gx0)) / (N - 1);
    int y=mapValueToY(v,g.gy0,g.gy1,sc.vmin,sc.vmax);
    xs[segLen]=x; ys[segLen]=y; segLen++;
  };

  for (int i=start; i<=end; ++i){
    uint8_t r=buf[i];
    if(!r){ flush(); lastIdx=-9999; continue; }
    float v=decode(r);
#if GRAPH_SMOOTHING_PREAVG
    float w=2.0f, acc=2.0f*v;
    if(i>start && buf[i-1]) { acc+=decode(buf[i-1]); w+=1.0f; }
    if(i<end   && buf[i+1]) { acc+=decode(buf[i+1]); w+=1.0f; }
    v=acc/w;
#endif
    if(i!=lastIdx+1 && segLen) flush();
    pushPoint(i,v); lastIdx=i;
  }
  flush();
}

// Keep all drawing geometry in sync with the physical TFT orientation.
static void applyScreenRotation() {
  tft.setRotation(screenRotation);
  SCREEN_WIDTH  = tft.width();
  SCREEN_HEIGHT = tft.height();
  header.w = SCREEN_WIDTH;
  if (SCREEN_WIDTH < SCREEN_HEIGHT)        header.h = 52;  // portrait
  else if (SCREEN_HEIGHT < 200)            header.h = 32;  // short landscape
  else                                     header.h = 36;  // 320x240
}

static void initDisplay() {
  if (displayType == 1) {
    activeTft = &st7789;
    st7789.init(170, 320);
  } else {
    activeTft = &ili9341;
    ili9341.begin();
  }
  applyScreenRotation();
}

static void dashboardPanels(Panel& tempCard, Panel& humCard, Panel& co2Card) {
  const int top = header.h + UI_TAB_H + UI_SECTION_TITLE_GAP;
  const int available = SCREEN_HEIGHT - top - UI_BOTTOM_MARGIN;
  if (SCREEN_WIDTH < SCREEN_HEIGHT) {
    const int h = (available - 2 * UI_GUTTER) / 3;
    tempCard = {UI_PAD_X, (int16_t)top, (int16_t)(SCREEN_WIDTH - 2 * UI_PAD_X), (int16_t)h};
    humCard = tempCard; humCard.y += h + UI_GUTTER;
    co2Card = tempCard; co2Card.y += 2 * (h + UI_GUTTER);
    co2Card.h = SCREEN_HEIGHT - UI_BOTTOM_MARGIN - co2Card.y;
  } else {
    if (SCREEN_HEIGHT < 200) {
      const int w = (SCREEN_WIDTH - 2 * UI_PAD_X - 2 * UI_GUTTER) / 3;
      tempCard = {UI_PAD_X, (int16_t)top, (int16_t)w, (int16_t)available};
      humCard = tempCard; humCard.x += w + UI_GUTTER;
      co2Card = humCard; co2Card.x += w + UI_GUTTER;
      co2Card.w = SCREEN_WIDTH - UI_PAD_X - co2Card.x;
    } else {
      const int co2H = (available - UI_GUTTER) / 2;
      const int topH = available - co2H - UI_GUTTER;
      tempCard = {UI_PAD_X, (int16_t)top, (int16_t)(SCREEN_WIDTH/2 - UI_PAD_X - UI_GUTTER/2), (int16_t)topH};
      humCard = tempCard; humCard.x += tempCard.w + UI_GUTTER;
      co2Card = {UI_PAD_X, (int16_t)(top + topH + UI_GUTTER), (int16_t)(SCREEN_WIDTH - 2 * UI_PAD_X), (int16_t)co2H};
    }
  }
}

// ===================== Floating badges on 30m page =====================
static void updateDashboardBadges() {
  if (screenMode != 0) return;

  Panel tempCard, humCard, co2Card;
  dashboardPanels(tempCard, humCard, co2Card);

  const int BADGE_H = 14;
  const int PAD_XR  = 6;

  auto badgeY  = [&](const Panel& p){
    return p.y + (UI_CARD_HEADER_H - BADGE_H) / 2;
  };
  auto badgeXr = [&](const Panel& p){ return p.x + p.w - PAD_XR; };

  auto wipeAreaDyn = [&](const Panel& p, int w){
    int xr = badgeXr(p);
    int y  = badgeY(p);
    tft.fillRect(xr - w - 2, y - 1, w + 4, BADGE_H + 2, COL_VALUE_BG);
  };

  if (!isnan(currentCO2)) {
    char txt[20];
    // Short landscape hides the unit to stop the badge eating the title.
    snprintf(txt, sizeof(txt), "%.0f", currentCO2);
    const int w = (int)strlen(txt) * 6 + 8;
    wipeAreaDyn(co2Card, w);
    const uint16_t col = (currentCO2 > 1200) ? COL_BAD
                        : (currentCO2 >  800) ? COL_WARN
                                              : COL_GOOD;
    drawModernValueBadge(badgeXr(co2Card), badgeY(co2Card), txt, col);
  }
  if (!isnan(currentTemp)) {
    char txt[20]; snprintf(txt, sizeof(txt), "%.1fC", currentTemp);
    const int w = (int)strlen(txt) * 6 + 8;
    wipeAreaDyn(tempCard, w);
    drawModernValueBadge(badgeXr(tempCard), badgeY(tempCard), txt, COL_TEXT);
  }
  if (!isnan(currentHum)) {
    char txt[20]; snprintf(txt, sizeof(txt), "%.0f%%", currentHum);
    const int w = (int)strlen(txt) * 6 + 8;
    wipeAreaDyn(humCard, w);
    drawModernValueBadge(badgeXr(humCard), badgeY(humCard), txt, COL_TEXT);
  }
}

// ===================== Pages =====================
static void draw30MinDashboard() {
  Panel tempCard, humCard, co2Card;
  dashboardPanels(tempCard, humCard, co2Card);
  const bool compact = SCREEN_HEIGHT < 200 && SCREEN_WIDTH > SCREEN_HEIGHT;

  // Temp
  drawCard(tempCard);
  drawCardHeader(tempCard, compact ? "T" : "Temp - 30m", COL_TEMP);
  drawMiniGraphU8(tempCard, tempSlots, COL_TEMP, encodeTemp(10), encodeTemp(40), "", "C", currentTemp, decodeTemp, 36, false);

  // Hum
  drawCard(humCard);
  drawCardHeader(humCard, compact ? "RH" : "Humidity - 30m", COL_HUM);
  drawMiniGraphU8(humCard, humSlots, COL_HUM, encodeHum(20), encodeHum(80), "", "%", currentHum, decodeHum, 36, false);

  // CO2
  drawCard(co2Card);
  drawCardHeader(co2Card, compact ? "CO2" : "CO2 ppm - 30m", COL_CO2);
  drawMiniGraphU16(co2Card, co2Slots, COL_CO2, 300, 2000, "", "ppm", currentCO2, 36, false);

  updateDashboardBadges();
}

static void drawWiFiIcon(int16_t x, int16_t y, uint8_t bars, uint16_t col){
  const int w=3,gap=2,h=10;
  for(int i=0;i<3;i++){
    int bh=(i+1)*h/3;
    int xx=x+i*(w+gap);
    uint16_t c=(i<bars)?col:COL_GRID;
    tft.fillRect(xx, y+(h-bh), w, bh, c);
  }
}

static void drawHeader(){
  tft.fillRect(0, 0, SCREEN_WIDTH, header.h, COL_CARD);
  tft.fillRect(0, header.h - 3, SCREEN_WIDTH, 3, COL_ACCENT);

  const bool portrait = SCREEN_WIDTH < SCREEN_HEIGHT;

  // Left: title (or a compact tag in portrait / short landscape)
  tft.setTextSize(1);
  tft.setTextColor(COL_ACCENT, COL_CARD);
  const char* title = portrait ? "ENV" : "Environment Monitor";
  tft.setCursor(8, portrait ? header.h/2 - 4 : 8);
  tft.print(title);

  // Right side: clock, offset from the WiFi / OFF indicator
  const int clockReserve = 26;
  const int clockRight   = SCREEN_WIDTH - clockReserve;
  const int clockY       = portrait ? header.h/2 - 8 : 8;

  time_t ts = approxNow();
  char clockBuf[8];
  if (ts > 0) {
    struct tm ti; localtime_r(&ts, &ti);
    snprintf(clockBuf, sizeof(clockBuf), "%02d:%02d", ti.tm_hour, ti.tm_min);
    printRightAligned(clockRight, clockY, clockBuf, 2, COL_TEXT, COL_CARD);
  } else {
    printRightAligned(clockRight, clockY, "--:--", 2, COL_MUTED, COL_CARD);
  }

  if (manualTimeActive) {
    tft.setTextSize(1);
    tft.setTextColor(COL_ACCENT, COL_CARD);
    tft.setCursor(clockRight - 6, clockY + (portrait ? 16 : 22));
    tft.print("M");
  }

  if (WiFi.status() == WL_CONNECTED){
    int rssi = WiFi.RSSI();
    uint8_t bars = (rssi > -55) ? 3 : (rssi > -67) ? 2 : (rssi > -80) ? 1 : 0;
    drawWiFiIcon(SCREEN_WIDTH - 18, portrait ? header.h/2 - 5 : 12, bars, COL_ACCENT);
  } else {
    printRightAligned(SCREEN_WIDTH - 6, portrait ? header.h/2 - 3 : 14,
                      "OFF", 1, COL_MUTED, COL_CARD);
  }
}

static void drawTabs(){
  static const char* namesWide[6]   = {"30 min","Stats","CO2 24h","T/H 24h","CO2 12h","T/H 12h"};
  static const char* namesNarrow[6] = {"30m","Stat","C24","T24","C12","T12"};

  const bool narrow = SCREEN_WIDTH < 260;
  const int y = header.h + 2;
  const int tabWidth = (SCREEN_WIDTH - 2 * UI_PAD_X) / NUM_SCREENS;
  const int tabH = UI_TAB_H - 2;

  int x = UI_PAD_X;
  for (int i = 0; i < NUM_SCREENS; i++){
    const char* label = narrow ? namesNarrow[i] : namesWide[i];
    const int   w     = tabWidth - 2;

    if ((int)strlen(label) * 6 + 6 > w) label = namesNarrow[i];

    const bool active = (i == screenMode);
    const uint16_t bg = active ? COL_ACCENT : COL_CARD;
    const uint16_t fg = active ? COL_BG     : COL_MUTED;

    tft.fillRoundRect(x, y, w, tabH, 6, bg);

    if (active) {
      tft.fillRect(x + 4, y + tabH - 2, w - 8, 2, COL_BG);
    }

    tft.setTextSize(1);
    tft.setTextColor(fg, bg);
    const int textX = x + (w - (int)strlen(label) * 6) / 2;
    tft.setCursor(textX, y + (tabH - 8) / 2 + 1);
    tft.print(label);

    x += tabWidth;
  }
}

// ---- Fullscreen CO2 24h ----
static void drawCO2FullScreen(){
  const int contentTop = header.h + UI_TAB_H + UI_SECTION_TITLE_GAP;
  Panel full={UI_PAD_X,contentTop,SCREEN_WIDTH-2*UI_PAD_X,
              SCREEN_HEIGHT-(contentTop+UI_BOTTOM_MARGIN)};

  drawCard(full);
  drawCardHeader(full, "CO2 - Last 24h", COL_CO2);

  const GridRect g = gridRect(full);
  drawTimeGrid24h(g.gx0,g.gx1,g.gy0,g.gy1);

  const int dispStart = (twentyFourGraphResetOriginSlot >= 0) ? twentyFourGraphResetOriginSlot : 0;

  uint16_t mn16=65535, mx16=0; bool any=false;
  for(int i=dispStart;i<SLOTS_PER_DAY;i++){
    uint16_t v=co2Slots[i]; if(!v) continue;
    any=true; if(v<mn16) mn16=v; if(v>mx16) mx16=v;
  }
  float dmin= any? mn16 : 300.f;
  float dmax= any? mx16 : 2000.f;
  AxisScale sc = computeAxisScale(dmin, dmax, 300.f, 2000.f, graphTickCount(g));
  drawYAxis(full, g, sc, "ppm");

  static int xs[SLOTS_PER_DAY], ys[SLOTS_PER_DAY];
  int segLen=0, lastIdx=-9999;
  auto flush=[&](){ if(segLen>=2) drawSegmentedLine(xs, ys, segLen, COL_CO2); segLen=0; };
  auto push=[&](int i, float v){
    int x=g.gx0+(i*(g.gx1-g.gx0))/(SLOTS_PER_DAY-1);
    int y=mapValueToY(v,g.gy0,g.gy1,sc.vmin,sc.vmax);
    xs[segLen]=x; ys[segLen]=y; segLen++;
  };

  for(int i=dispStart;i<SLOTS_PER_DAY;i++){
    uint16_t r=co2Slots[i];
    if(!r){ flush(); lastIdx=-9999; continue; }
    float v=r;
#if GRAPH_SMOOTHING_PREAVG
    float w=2.0f, acc=2.0f*v;
    if(i>dispStart && co2Slots[i-1]) { acc += co2Slots[i-1]; w+=1.0f; }
    if(i<SLOTS_PER_DAY-1 && co2Slots[i+1]) { acc += co2Slots[i+1]; w+=1.0f; }
    v = acc/w;
#endif
    if(i!=lastIdx+1 && segLen) flush();
    push(i, v); lastIdx=i;
  }
  flush();

  if (any) drawNowMarkerAndTag(g.gx0,g.gx1,g.gy0,g.gy1);
  else     drawEmptyGraph(g);

  if(!isnan(currentCO2)){
    char bufv[20]; snprintf(bufv,sizeof(bufv),"%.0f",currentCO2);
    uint16_t col=(currentCO2>1200)?COL_BAD:(currentCO2>800?COL_WARN:COL_GOOD);
    drawModernValueBadge(full.x+full.w-10, full.y+4, bufv, col);
    int8_t tr=co2Trend();
    if(tr!=0) drawModernTrendIndicator(full.x+full.w-30, full.y+6, tr, (tr>0)?COL_WARN:COL_GOOD);
  }
}

// ---- Fullscreen Temp+Hum 24h ----
static void drawTempHumFullScreen(){
  const int contentTop = header.h + UI_TAB_H + UI_SECTION_TITLE_GAP;
  const int availH = SCREEN_HEIGHT - (contentTop + UI_BOTTOM_MARGIN);
  const int eachH  = (availH-UI_GUTTER)/2;
  const bool shortLandscape = SCREEN_HEIGHT < 200 && SCREEN_WIDTH > SCREEN_HEIGHT;
  const int pairW = (SCREEN_WIDTH - 2*UI_PAD_X - UI_GUTTER)/2;

  const int dispStart = (twentyFourGraphResetOriginSlot >= 0) ? twentyFourGraphResetOriginSlot : 0;

  // Temperature card
  Panel top={UI_PAD_X,(int16_t)contentTop,
             (int16_t)(shortLandscape ? pairW : SCREEN_WIDTH-2*UI_PAD_X),
             (int16_t)(shortLandscape ? availH : eachH)};
  drawCard(top);
  drawCardHeader(top, "Temperature - 24h", COL_TEMP);

  GridRect g=gridRect(top);
  drawTimeGrid24h(g.gx0,g.gx1,g.gy0,g.gy1);

  float tmin=1e9f,tmax=-1e9f; bool tany=false;
  for(int i=dispStart;i<SLOTS_PER_DAY;i++){
    uint8_t r=tempSlots[i]; if(!r) continue;
    float v=decodeTemp(r);
    tany=true; if(v<tmin) tmin=v; if(v>tmax) tmax=v;
  }
  AxisScale ts=computeAxisScale(tany?tmin:10.f, tany?tmax:40.f, 10.f, 40.f, graphTickCount(g));
  drawYAxis(top, g, ts, "C");

  static int xsT[SLOTS_PER_DAY], ysT[SLOTS_PER_DAY];
  int segLen=0, lastIdx=-9999;
  auto flushT=[&](){ if(segLen>=2) drawSegmentedLine(xsT, ysT, segLen, COL_TEMP); segLen=0; };
  auto pushT=[&](int i, float v){
    int x=g.gx0+(i*(g.gx1-g.gx0))/(SLOTS_PER_DAY-1);
    int y=mapValueToY(v,g.gy0,g.gy1,ts.vmin,ts.vmax);
    xsT[segLen]=x; ysT[segLen]=y; segLen++;
  };

  for(int i=dispStart;i<SLOTS_PER_DAY;i++){
    uint8_t r=tempSlots[i];
    if(!r){ flushT(); lastIdx=-9999; continue; }
    float v=decodeTemp(r);
#if GRAPH_SMOOTHING_PREAVG
    float w=2.0f, acc=2.0f*v;
    if(i>dispStart && tempSlots[i-1]) { acc += decodeTemp(tempSlots[i-1]); w+=1.0f; }
    if(i<SLOTS_PER_DAY-1 && tempSlots[i+1]) { acc += decodeTemp(tempSlots[i+1]); w+=1.0f; }
    v = acc/w;
#endif
    if(i!=lastIdx+1 && segLen) flushT();
    pushT(i,v); lastIdx=i;
  }
  flushT();

  if (tany) drawNowMarkerAndTag(g.gx0,g.gx1,g.gy0,g.gy1);
  else      drawEmptyGraph(g);

  if(!isnan(currentTemp)){
    char bufv[20]; snprintf(bufv,sizeof(bufv),"%.1fC",currentTemp);
    drawModernValueBadge(top.x+top.w-10, top.y+4, bufv, COL_TEXT);
    int8_t tr=u8Trend(tempSlots);
    if(tr!=0) drawModernTrendIndicator(top.x+top.w-30, top.y+6, tr, (tr>0)?COL_WARN:COL_GOOD);
  }

  // Humidity card
  Panel bot={(int16_t)(shortLandscape ? UI_PAD_X+pairW+UI_GUTTER : UI_PAD_X),
             (int16_t)(shortLandscape ? contentTop : contentTop+eachH+UI_GUTTER),
             (int16_t)(shortLandscape ? pairW : SCREEN_WIDTH-2*UI_PAD_X),
             (int16_t)(shortLandscape ? availH : eachH)};
  drawCard(bot);
  drawCardHeader(bot, "Humidity - 24h", COL_HUM);

  g=gridRect(bot);
  drawTimeGrid24h(g.gx0,g.gx1,g.gy0,g.gy1);

  float hmin=1e9f,hmax=-1e9f; bool hany=false;
  for(int i=dispStart;i<SLOTS_PER_DAY;i++){
    uint8_t r=humSlots[i]; if(!r) continue;
    float v=decodeHum(r);
    hany=true; if(v<hmin) hmin=v; if(v>hmax) hmax=v;
  }
  AxisScale hs=computeAxisScale(hany?hmin:20.f, hany?hmax:80.f, 20.f, 80.f, graphTickCount(g));
  drawYAxis(bot, g, hs, "%");

  static int xsH[SLOTS_PER_DAY], ysH[SLOTS_PER_DAY];
  int segLen2=0; lastIdx=-9999;
  auto flushH=[&](){ if(segLen2>=2) drawSegmentedLine(xsH, ysH, segLen2, COL_HUM); segLen2=0; };
  auto pushH=[&](int i, float v){
    int x=g.gx0+(i*(g.gx1-g.gx0))/(SLOTS_PER_DAY-1);
    int y=mapValueToY(v,g.gy0,g.gy1,hs.vmin,hs.vmax);
    xsH[segLen2]=x; ysH[segLen2]=y; segLen2++;
  };

  for(int i=dispStart;i<SLOTS_PER_DAY;i++){
    uint8_t r=humSlots[i];
    if(!r){ flushH(); lastIdx=-9999; continue; }
    float v=decodeHum(r);
#if GRAPH_SMOOTHING_PREAVG
    float w=2.0f, acc=2.0f*v;
    if(i>dispStart && humSlots[i-1]) { acc += decodeHum(humSlots[i-1]); w+=1.0f; }
    if(i<SLOTS_PER_DAY-1 && humSlots[i+1]) { acc += decodeHum(humSlots[i+1]); w+=1.0f; }
    v = acc/w;
#endif
    if(i!=lastIdx+1 && segLen2) flushH();
    pushH(i,v); lastIdx=i;
  }
  flushH();

  if (hany) drawNowMarkerAndTag(g.gx0,g.gx1,g.gy0,g.gy1);
  else      drawEmptyGraph(g);

  if(!isnan(currentHum)){
    char bufv[20]; snprintf(bufv,sizeof(bufv),"%.0f%%",currentHum);
    drawModernValueBadge(bot.x+bot.w-10, bot.y+4, bufv, COL_TEXT);
    int8_t tr=u8Trend(humSlots);
    if(tr!=0) drawModernTrendIndicator(bot.x+bot.w-30, bot.y+6, tr, (tr>0)?COL_WARN:COL_GOOD);
  }
}

// ---- CO2 12h ----
static void drawCO212Hour(){
  const int contentTop = header.h + UI_TAB_H + UI_SECTION_TITLE_GAP;
  Panel half = { UI_PAD_X, contentTop, SCREEN_WIDTH - 2*UI_PAD_X,
                 SCREEN_HEIGHT - (contentTop + UI_BOTTOM_MARGIN) };

  drawCard(half);
  drawCardHeader(half, "CO2 - Last 12h", COL_CO2);

  const GridRect g = gridRect(half);
  drawTimeGrid12h(g.gx0, g.gx1, g.gy0, g.gy1);

  const int baseStart = max(0, currentSlot - (SLOTS_PER_12H - 1));
  int dispStart = (twelveGraphResetOriginSlot >= 0) ? max(baseStart, twelveGraphResetOriginSlot) : baseStart;

  int count = max(0, currentSlot - dispStart + 1);
  if (count > SLOTS_PER_12H) count = SLOTS_PER_12H;

  static uint16_t buf[SLOTS_PER_12H];
  memset(buf, 0, sizeof(buf));
  for (int i = 0; i < count; ++i) {
    int idx = dispStart + i;
    if (idx >= 0 && idx < SLOTS_PER_DAY) buf[i] = co2Slots[idx];
  }

  uint16_t mn16 = 65535, mx16 = 0; bool any = false;
  for (int i = 0; i < count; ++i) {
    uint16_t v = buf[i]; if (!v) continue;
    any = true; if (v < mn16) mn16 = v; if (v > mx16) mx16 = v;
  }
  float dmin = any ? mn16 : 300.f;
  float dmax = any ? mx16 : 2000.f;
  AxisScale sc = computeAxisScale(dmin, dmax, 300.f, 2000.f, graphTickCount(g));
  drawYAxis(half, g, sc, "ppm");

  static int xs[SLOTS_PER_12H], ys[SLOTS_PER_12H];
  int segLen = 0, lastIdx = -9999;

  auto flush = [&](){ if (segLen >= 2) drawSegmentedLine(xs, ys, segLen, COL_CO2); segLen = 0; };
  auto push  = [&](int i, float v){
    int x = g.gx0 + ((SLOTS_PER_12H - count + i) * (g.gx1 - g.gx0)) / (SLOTS_PER_12H - 1);
    int y = mapValueToY(v, g.gy0, g.gy1, sc.vmin, sc.vmax);
    xs[segLen] = x; ys[segLen] = y; segLen++;
  };

  for (int i = 0; i < count; ++i) {
    uint16_t r = buf[i];
    if (!r) { flush(); lastIdx = -9999; continue; }
    float v = r;
#if GRAPH_SMOOTHING_PREAVG
    float w = 2.0f, acc = 2.0f * v;
    if (i > 0       && buf[i-1]) { acc += buf[i-1]; w += 1.0f; }
    if (i < count-1 && buf[i+1]) { acc += buf[i+1]; w += 1.0f; }
    v = acc / w;
#endif
    if (i != lastIdx + 1 && segLen) flush();
    push(i, v); lastIdx = i;
  }
  flush();

  if(!any) drawEmptyGraph(g);

  if (!isnan(currentCO2)) {
    char bufv[20]; snprintf(bufv, sizeof(bufv), "%.0f", currentCO2);
    const int badgeH = 12;
    const int yBadge = half.y + (UI_CARD_HEADER_H - badgeH) / 2;
    const int xRight = half.x + half.w - 8;
    drawModernValueBadge(xRight, yBadge, bufv,
      (currentCO2 > 1200) ? COL_BAD : (currentCO2 > 800 ? COL_WARN : COL_GOOD));
    int8_t tr = co2Trend();
    if (tr != 0) drawModernTrendIndicator(xRight - (int)strlen(bufv)*6 - 20, half.y + 6, tr,
                                          (tr > 0) ? COL_WARN : COL_GOOD);
  }
}

// ---- Temp/Hum 12h ----
static void drawTempHum12Hour(){
  const int contentTop = header.h + UI_TAB_H + UI_SECTION_TITLE_GAP;
  const int availH=SCREEN_HEIGHT-(contentTop+UI_BOTTOM_MARGIN);
  const int eachH=(availH-UI_GUTTER)/2;
  const bool shortLandscape = SCREEN_HEIGHT < 200 && SCREEN_WIDTH > SCREEN_HEIGHT;
  const int pairW = (SCREEN_WIDTH - 2*UI_PAD_X - UI_GUTTER)/2;

  const int baseStart = max(0, currentSlot - (SLOTS_PER_12H - 1));
  const int dispStart = (twelveGraphResetOriginSlot >= 0) ? max(baseStart, twelveGraphResetOriginSlot) : baseStart;

  // Temperature
  Panel top={UI_PAD_X,(int16_t)contentTop,
             (int16_t)(shortLandscape ? pairW : SCREEN_WIDTH-2*UI_PAD_X),
             (int16_t)(shortLandscape ? availH : eachH)};
  drawCard(top);
  drawCardHeader(top, "Temperature - 12h", COL_TEMP);

  GridRect g=gridRect(top);
  drawTimeGrid12h(g.gx0,g.gx1,g.gy0,g.gy1);

  int countT = max(0, currentSlot - dispStart + 1);
  if (countT > SLOTS_PER_12H) countT = SLOTS_PER_12H;

  static uint8_t bufT[SLOTS_PER_12H];
  memset(bufT,0,sizeof(bufT));
  for(int i=0;i<countT;i++){ int idx=dispStart+i; if(idx>=0 && idx<SLOTS_PER_DAY) bufT[i]=tempSlots[idx]; }

  float tmin=1e9f,tmax=-1e9f; bool tany=false;
  for(int i=0;i<countT;i++){
    uint8_t r=bufT[i]; if(!r) continue;
    float v=decodeTemp(r);
    tany=true; if(v<tmin) tmin=v; if(v>tmax) tmax=v;
  }
  AxisScale ts=computeAxisScale(tany?tmin:10.f, tany?tmax:40.f, 10.f, 40.f, graphTickCount(g));
  drawYAxis(top, g, ts, "C");

  static int xs[SLOTS_PER_12H], ys[SLOTS_PER_12H];
  int segLen=0, lastIdx=-9999;
  auto flush=[&](){ if(segLen>=2) drawSegmentedLine(xs, ys, segLen, COL_TEMP); segLen=0; };
  auto push=[&](int i, float v){
    int x=g.gx0 + ((SLOTS_PER_12H-countT+i)*(g.gx1-g.gx0))/(SLOTS_PER_12H-1);
    int y=mapValueToY(v,g.gy0,g.gy1,ts.vmin,ts.vmax);
    xs[segLen]=x; ys[segLen]=y; segLen++;
  };
  for(int i=0;i<countT;i++){
    uint8_t r=bufT[i];
    if(!r){ flush(); lastIdx=-9999; continue; }
    float v=decodeTemp(r);
#if GRAPH_SMOOTHING_PREAVG
    float w=2.0f, acc=2.0f*v;
    if(i>0 && bufT[i-1]) { acc += decodeTemp(bufT[i-1]); w+=1.0f; }
    if(i<countT-1 && bufT[i+1]) { acc += decodeTemp(bufT[i+1]); w+=1.0f; }
    v = acc/w;
#endif
    if(i!=lastIdx+1 && segLen) flush();
    push(i,v); lastIdx=i;
  }
  flush();
  if(!tany) drawEmptyGraph(g);
  if(!isnan(currentTemp)){
    char bufv[20]; snprintf(bufv,sizeof(bufv),"%.1fC",currentTemp);
    drawModernValueBadge(top.x+top.w-8, top.y+4, bufv, COL_TEXT);
    int8_t tr=u8Trend(tempSlots);
    if(tr!=0) drawModernTrendIndicator(top.x+top.w-30, top.y+6, tr, (tr>0)?COL_WARN:COL_GOOD);
  }

  // Humidity
  Panel bot={(int16_t)(shortLandscape ? UI_PAD_X+pairW+UI_GUTTER : UI_PAD_X),
             (int16_t)(shortLandscape ? contentTop : contentTop+eachH+UI_GUTTER),
             (int16_t)(shortLandscape ? pairW : SCREEN_WIDTH-2*UI_PAD_X),
             (int16_t)(shortLandscape ? availH : eachH)};
  drawCard(bot);
  drawCardHeader(bot, "Humidity - 12h", COL_HUM);

  g=gridRect(bot);
  drawTimeGrid12h(g.gx0,g.gx1,g.gy0,g.gy1);

  int countH = max(0, currentSlot - dispStart + 1);
  if (countH > SLOTS_PER_12H) countH = SLOTS_PER_12H;

  static uint8_t bufH[SLOTS_PER_12H];
  memset(bufH,0,sizeof(bufH));
  for(int i=0;i<countH;i++){ int idx=dispStart+i; if(idx>=0 && idx<SLOTS_PER_DAY) bufH[i]=humSlots[idx]; }

  float hmin=1e9f,hmax=-1e9f; bool hany=false;
  for(int i=0;i<countH;i++){
    uint8_t r=bufH[i]; if(!r) continue;
    float v=decodeHum(r);
    hany=true; if(v<hmin) hmin=v; if(v>hmax) hmax=v;
  }
  AxisScale hs=computeAxisScale(hany?hmin:20.f, hany?hmax:80.f, 20.f, 80.f, graphTickCount(g));
  drawYAxis(bot, g, hs, "%");

  static int xs2[SLOTS_PER_12H], ys2[SLOTS_PER_12H];
  int segLen2=0; lastIdx=-9999;
  auto flush2=[&](){ if(segLen2>=2) drawSegmentedLine(xs2, ys2, segLen2, COL_HUM); segLen2=0; };
  auto push2=[&](int i, float v){
    int x=g.gx0 + ((SLOTS_PER_12H-countH+i)*(g.gx1-g.gx0))/(SLOTS_PER_12H-1);
    int y=mapValueToY(v,g.gy0,g.gy1,hs.vmin,hs.vmax);
    xs2[segLen2]=x; ys2[segLen2]=y; segLen2++;
  };
  for(int i=0;i<countH;i++){
    uint8_t r=bufH[i];
    if(!r){ flush2(); lastIdx=-9999; continue; }
    float v=decodeHum(r);
#if GRAPH_SMOOTHING_PREAVG
    float w=2.0f, acc=2.0f*v;
    if(i>0 && bufH[i-1]) { acc += decodeHum(bufH[i-1]); w+=1.0f; }
    if(i<countH-1 && bufH[i+1]) { acc += decodeHum(bufH[i+1]); w+=1.0f; }
    v = acc/w;
#endif
    if(i!=lastIdx+1 && segLen2) flush2();
    push2(i,v); lastIdx=i;
  }
  flush2();
  if(!hany) drawEmptyGraph(g);
  if(!isnan(currentHum)){
    char bufv[20]; snprintf(bufv,sizeof(bufv),"%.0f%%",currentHum);
    drawModernValueBadge(bot.x+bot.w-8, bot.y+4, bufv, COL_TEXT);
    int8_t tr=u8Trend(humSlots);
    if(tr!=0) drawModernTrendIndicator(bot.x+bot.w-30, bot.y+6, tr, (tr>0)?COL_WARN:COL_GOOD);
  }
}

// ---- All screens dispatcher ----
static void drawAllScreens() {
  tft.fillScreen(COL_BG);
  drawHeader();
  drawTabs();

  switch (screenMode) {
    case 0: draw30MinDashboard();      break;

    case 1: { // STATS PAGE
      const bool compactStats = SCREEN_HEIGHT < 200;
      int y = header.h + UI_TAB_H + (compactStats ? 4 : 10);
      const int cardH = compactStats ? 28
                      : (SCREEN_WIDTH < SCREEN_HEIGHT ? 62 : 46);
      const int gutter = compactStats ? 4 : 10;

      struct StatData { float sum=0; int count=0; float min=1e9f, max=-1e9f; int idxMin=-1, idxMax=-1; };

      auto scanU16 = [&](StatData& s, const uint16_t* a){
        for (int i=0;i<SLOTS_PER_DAY;i++){
          uint16_t v=a[i]; if(!v) continue;
          s.sum+=v; s.count++;
          if(v<s.min){s.min=v; s.idxMin=i;}
          if(v>s.max){s.max=v; s.idxMax=i;}
        }
      };
      auto scanU8 = [&](StatData& s, const uint8_t* a, float(*dec)(uint8_t)){
        for (int i=0;i<SLOTS_PER_DAY;i++){
          uint8_t r=a[i]; if(!r) continue;
          float v=dec(r);
          s.sum+=v; s.count++;
          if(v<s.min){s.min=v; s.idxMin=i;}
          if(v>s.max){s.max=v; s.idxMax=i;}
        }
      };

      auto drawStatCard = [&](const char* name, StatData d, const char* unit, bool isInt, uint16_t color, int yTop){
        Panel p={UI_PAD_X,yTop,SCREEN_WIDTH-2*UI_PAD_X,cardH};
        drawCard(p);
        drawCardHeader(p, name, color);

        char avgStr[24]="--", lowStr[20]="--", highStr[20]="--", lowTime[8]="", highTime[8]="";
        if(d.count>0){
          if(isInt){
            snprintf(avgStr,sizeof(avgStr),"Avg: %.0f%s", d.sum/d.count, unit);
            snprintf(lowStr,sizeof(lowStr),"%.0f%s", d.min, unit);
            snprintf(highStr,sizeof(highStr),"%.0f%s", d.max, unit);
          }else{
            snprintf(avgStr,sizeof(avgStr),"Avg: %.1f%s", d.sum/d.count, unit);
            snprintf(lowStr,sizeof(lowStr),"%.1f%s", d.min, unit);
            snprintf(highStr,sizeof(highStr),"%.1f%s", d.max, unit);
          }
          if(d.idxMin>=0) formatSlotTimeHHMM(d.idxMin, lowTime, sizeof(lowTime));
          if(d.idxMax>=0) formatSlotTimeHHMM(d.idxMax, highTime, sizeof(highTime));
        }

        if (compactStats) {
          printRightAligned(p.x+p.w-8, p.y+5, avgStr, 1, COL_TEXT, COL_CARD_HI);
          char range[64];
          snprintf(range,sizeof(range),"L: %s   H: %s",lowStr,highStr);
          tft.setTextSize(1);
          tft.setTextColor(COL_MUTED, COL_CARD);
          tft.setCursor(p.x+8,p.y+UI_CARD_HEADER_H+3); tft.print(range);
          return;
        }
        tft.setTextSize(1);
        tft.setTextColor(COL_TEXT, COL_CARD);
        tft.setCursor(p.x+12, p.y+UI_CARD_HEADER_H+4);
        tft.print(avgStr);

        tft.setCursor(p.x+12, p.y+UI_CARD_HEADER_H+16);
        if(d.count){
          char line[80];
          if (SCREEN_WIDTH < SCREEN_HEIGHT) {
            snprintf(line,sizeof(line),"L: %s @ %s", lowStr,lowTime);
            tft.print(line);
            tft.setCursor(p.x+12, p.y+UI_CARD_HEADER_H+28);
            snprintf(line,sizeof(line),"H: %s @ %s", highStr,highTime);
          } else {
            snprintf(line,sizeof(line),"L: %s @ %s  H: %s @ %s", lowStr,lowTime, highStr,highTime);
          }
          tft.print(line);
        }else{
          tft.setTextColor(COL_MUTED, COL_CARD);
          tft.print("-- no data --");
        }
      };

      StatData co2,temp,hum;
      scanU16(co2, co2Slots);
      scanU8 (temp,tempSlots,decodeTemp);
      scanU8 (hum, humSlots, decodeHum);

      drawStatCard("CO2",  co2,  "ppm", true,  COL_CO2,  y); y+=cardH+gutter;
      drawStatCard("TEMP", temp, "C",   false, COL_TEMP, y); y+=cardH+gutter;
      drawStatCard("HUM",  hum,  "%",   true,  COL_HUM,  y);

      int totalReadings = co2.count + temp.count + hum.count;
      tft.setTextSize(1);
      tft.setTextColor(COL_MUTED, COL_BG);
      tft.setCursor(UI_PAD_X, SCREEN_HEIGHT-14);
      if(totalReadings>0){
        char s[44];
        snprintf(s,sizeof(s),"%d samples - %d%%", totalReadings/3, (totalReadings*100)/(SLOTS_PER_DAY*3));
        tft.print(s);
      } else {
        tft.print("No data available");
      }
    } break;

    case 2: drawCO2FullScreen();       break;
    case 3: drawTempHumFullScreen();   break;
    case 4: drawCO212Hour();           break;
    case 5: drawTempHum12Hour();       break;

    default:
      screenMode = 0;
      draw30MinDashboard();
      break;
  }
  lastFast30mDraw = lastRedraw = millis();
}

// ===================== Data accumulation =====================
static void resetAccumulators(){ co2Sum=humSum=0; tempSum=0; sampleCount=co2SampleCount=0; }
static void commitSlotData(int slot){
  if(slot<0 || slot>=SLOTS_PER_DAY) return;
  if(co2SampleCount) co2Slots[slot]=co2Sum/co2SampleCount;
  if(sampleCount) {
    tempSlots[slot]=encodeTemp(tempSum/(float)sampleCount/10.0f);
    humSlots[slot]=encodeHum(humSum/(float)sampleCount/10.0f);
  }
}

static void clearData(){
  memset(co2Slots,0,sizeof(co2Slots));
  memset(tempSlots,0,sizeof(tempSlots));
  memset(humSlots,0,sizeof(humSlots));
}

// ===================== SCD30 Force Recal to 400 =====================
static uint8_t scd30_crc8(uint8_t msb, uint8_t lsb) {
  uint8_t crc = 0xFF;
  uint8_t data[2] = {msb, lsb};
  for (int i = 0; i < 2; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
    }
  }
  return crc;
}

static bool forceRecal400() {
  const uint8_t SCD30_ADDR = 0x61;
  const uint16_t ref = 400;
  uint8_t msb = (ref >> 8) & 0xFF;
  uint8_t lsb = (ref & 0xFF);
  uint8_t crc = scd30_crc8(msb, lsb);

  Wire.beginTransmission(SCD30_ADDR);
  Wire.write(0x52); Wire.write(0x04);
  Wire.write(msb);  Wire.write(lsb);
  Wire.write(crc);
  if (Wire.endTransmission() != 0) return false;
  delay(50);
  return true;
}

// ===================== Cleanly isolated NC button logic =====================
class NcButton {
public:
  void begin(uint8_t pin,
             void (*onShortPress)(),
             void (*onLongRequest)(),
             void (*onConfirmPress)(),
             void (*onConfirmTimeout)(),
             uint32_t longMs = 5000,
             uint32_t confirmWindowMs = 5000)
  {
    _pin = pin;
    _onShort = onShortPress;
    _onLongReq = onLongRequest;
    _onConfirm = onConfirmPress;
    _onTimeout = onConfirmTimeout;
    _longMs = longMs;
    _confirmWindowMs = confirmWindowMs;

    pinMode(_pin, INPUT_PULLUP);

    _instance = this;
    attachInterrupt(digitalPinToInterrupt(_pin), isrThunk, RISING);
  }

  void tick(){
    if(_confirmPending && (millis() > _confirmDeadline)){
      _confirmPending = false;
      if(_onTimeout) _onTimeout();
    }

    if(_irq){
      _irq = false;

      delay(25);
      if(digitalRead(_pin) == HIGH){
        _pressed = true;
        _pressAt = millis();
        _longFired = false;
      }
    }

    if(_pressed){
      if(digitalRead(_pin) == LOW){
        _pressed = false;

        if(_longFired){
          return;
        }

        if(_confirmPending){
          _confirmPending = false;
          if(_onConfirm) _onConfirm();
        }else{
          if(_onShort) _onShort();
        }
        return;
      }

      if(!_longFired && (millis() - _pressAt >= _longMs)){
        _longFired = true;

        _confirmPending = true;
        _confirmDeadline = millis() + _confirmWindowMs;

        if(_onLongReq) _onLongReq();
      }
    }
  }

  void onIRQ(){
    static uint32_t lastUs = 0;
    uint32_t nowUs = micros();
    if(nowUs - lastUs < 3000) return;
    lastUs = nowUs;

    _irq = true;
  }

private:
  uint8_t _pin = 255;

  volatile bool _irq = false;

  bool _pressed = false;
  bool _longFired = false;
  uint32_t _pressAt = 0;

  bool _confirmPending = false;
  uint32_t _confirmDeadline = 0;

  uint32_t _longMs = 5000;
  uint32_t _confirmWindowMs = 5000;

  void (*_onShort)() = nullptr;
  void (*_onLongReq)() = nullptr;
  void (*_onConfirm)() = nullptr;
  void (*_onTimeout)() = nullptr;

  static NcButton* _instance;
  static void ICACHE_RAM_ATTR isrThunk(){
    if(_instance) _instance->onIRQ();
  }
};
NcButton* NcButton::_instance = nullptr;

static NcButton button;

static void onBtnLongRequest(){
  drawOverlayBox("Force 400 ppm?", "Press once within 5s to confirm");
}

static void onBtnConfirm(){
  drawOverlayBox("Calibrating...", "Setting baseline to 400 ppm");
  bool ok = forceRecal400();
  showToast(ok ? "Baseline set to 400 ppm" : "Calibration failed",
            ok ? COL_GOOD : ILI9341_RED, 1500);
  drawAllScreens();
}

static void onBtnConfirmTimeout(){
  showToast("Calibration canceled", COL_MUTED, 900);
  drawAllScreens();
}

// ===================== FS: time anchor =====================
#if ENABLE_FS
struct TimeAnchorBlob { uint32_t magic; uint32_t epoch; uint32_t ms; uint32_t crc; };

static void saveTimeAnchor() {
  if (!fsReady) return;
  TimeAnchorBlob b{0x54A11E42, (uint32_t)timeAnchorEpoch, timeAnchorMillis, 0};
  b.crc = b.magic ^ b.epoch ^ b.ms;
  File f = LittleFS.open("/time.anchor", "w");
  if (!f) return;
  f.write((uint8_t*)&b, sizeof(b));
  f.close();
}

static bool loadTimeAnchor() {
  if (!fsReady || !LittleFS.exists("/time.anchor")) return false;
  File f = LittleFS.open("/time.anchor", "r");
  if (!f) return false;
  TimeAnchorBlob b{};
  if (f.readBytes((char*)&b, sizeof(b)) != sizeof(b)) { f.close(); return false; }
  f.close();
  if (b.magic != 0x54A11E42) return false;
  if ((b.magic ^ b.epoch ^ b.ms) != b.crc) return false;
  timeAnchorEpoch  = b.epoch;
  timeAnchorMillis = millis();
  timeAnchorValid  = true;
  return true;
}

static bool saveManualClock(time_t epoch) {
  if (!fsReady) return false;
  File f = LittleFS.open("/manual_clock.tmp", "w");
  if (!f) return false;
  String value = String((uint32_t)epoch) + "\n";
  bool ok = f.print(value) == value.length();
  f.close();
  if (!ok) { LittleFS.remove("/manual_clock.tmp"); return false; }
  return LittleFS.rename("/manual_clock.tmp", "/manual_clock.cfg");
}

static void loadManualClock() {
  if (!fsReady || !LittleFS.exists("/manual_clock.cfg")) return;
  File f = LittleFS.open("/manual_clock.cfg", "r");
  if (!f) return;
  String value = f.readStringUntil('\n');
  f.close();
  value.trim();
  if (value.length() != 10) return;
  for (size_t i=0; i<value.length(); ++i) if (!isDigit(value[i])) return;
  const uint32_t epoch = strtoul(value.c_str(), nullptr, 10);
  if (epoch < 1609459200UL || epoch > 2145916799UL) return;
  setenv("TZ", TZ_ADELAIDE, 1);
  tzset();
  timeval tv{(time_t)epoch, 0};
  if (settimeofday(&tv, nullptr) != 0) return;
  manualTimeActive = true;
  manualClockRestored = true;
  timeAnchorEpoch = epoch;
  timeAnchorMillis = millis();
  timeAnchorValid = true;
}
#endif

// ===================== FS: per-day files =====================
#if ENABLE_FS
struct StorageHeader{
  uint32_t magic;
  uint16_t year;
  uint16_t dayOfYear;
  uint32_t checksum;
};

static bool normalizeDayName(const String& in, char* out, size_t outSize) {
  if (!out || outSize == 0) return false;
  const char* s = in.c_str();
  if (!s || !s[0]) return false;
  if (s[0] == '/') s++;
  size_t len = strlen(s);
  if (len + 1 > outSize) return false;
  if (strncmp(s, "day_", 4) != 0) return false;
  if (len < 8 || strcmp(s + len - 4, ".bin") != 0) return false;
  memcpy(out, s, len);
  out[len] = '\0';
  return true;
}

static void makeDayPath(const char* name, char* out, size_t outSize) {
  if (!out || outSize == 0) return;
  snprintf(out, outSize, "/%s", name ? name : "");
}

static void pruneOldDayFiles() {
  const int MAX_DAYS  = 30;
  const int MAX_FILES = 40;
  char names[MAX_FILES][24];
  int count = 0;

  Dir dir = LittleFS.openDir("/");
  while (dir.next()) {
    String fn = dir.fileName();
    char base[24];
    if (!normalizeDayName(fn, base, sizeof(base))) continue;
    if (count < MAX_FILES) {
      strncpy(names[count], base, sizeof(names[count]) - 1);
      names[count][sizeof(names[count]) - 1] = '\0';
      count++;
    }
  }
  if (count <= MAX_DAYS) return;

  for (int i = 0; i < count - 1; ++i) {
    for (int j = i + 1; j < count; ++j) {
      if (strcmp(names[i], names[j]) > 0) {
        char tmp[24];
        strcpy(tmp, names[i]); strcpy(names[i], names[j]); strcpy(names[j], tmp);
      }
    }
  }
  for (int i = 0; i < count - MAX_DAYS; ++i) {
    char path[28];
    makeDayPath(names[i], path, sizeof(path));
    LittleFS.remove(path);
  }
}

static void saveDataToFS(bool previousDay) {
  if (!fsReady) return;

  writeLiveSlotPreviewToBuffers();

  time_t ts = approxNow();
  if (ts <= 1600000000) return;
  if (previousDay) ts -= 4 * 3600;

  struct tm ti;
  localtime_r(&ts, &ti);
  uint16_t year = 1900 + ti.tm_year;
  uint16_t day1 = ti.tm_yday + 1;

  char path[24];
  snprintf(path, sizeof(path), "/day_%04u%03u.bin", (unsigned)year, (unsigned)day1);

  StorageHeader h{};
  h.magic     = 0xDA7ADA7A;
  h.year      = year;
  h.dayOfYear = day1;
  h.checksum  = 0;
  for (size_t i = 0; i < sizeof(co2Slots); ++i) h.checksum += ((uint8_t*)co2Slots)[i];

  File f = LittleFS.open(path, "w");
  if (!f) return;

  f.write((uint8_t*)&h, sizeof(h));
  f.write((uint8_t*)co2Slots, sizeof(co2Slots));
  f.write((uint8_t*)tempSlots, sizeof(tempSlots));
  f.write((uint8_t*)humSlots,  sizeof(humSlots));
  f.close();

  pruneOldDayFiles();
}

static bool loadDayFile(const char* path){
  if (!fsReady) return false;
  if (!LittleFS.exists(path)) return false;

  File f = LittleFS.open(path, "r");
  if (!f) return false;

  const size_t needSize =
    sizeof(StorageHeader) +
    (sizeof(uint16_t) * SLOTS_PER_DAY) +
    (sizeof(uint8_t)  * SLOTS_PER_DAY) +
    (sizeof(uint8_t)  * SLOTS_PER_DAY);

  if ((size_t)f.size() < needSize) { f.close(); return false; }

  StorageHeader h{};
  if (f.readBytes((char*)&h, sizeof(h)) != sizeof(h) || h.magic != 0xDA7ADA7A) {
    f.close(); return false;
  }

  if (f.readBytes((char*)co2Slots, sizeof(co2Slots)) != sizeof(co2Slots) ||
      f.readBytes((char*)tempSlots, sizeof(tempSlots)) != sizeof(tempSlots) ||
      f.readBytes((char*)humSlots,  sizeof(humSlots))  != sizeof(humSlots)) {
    f.close(); return false;
  }
  f.close();
  return true;
}

static void loadDataFromFS() {
  if (!fsReady) return;

  if (isTimeValid()) {
    time_t ts = time(nullptr);
    struct tm ti;
    localtime_r(&ts, &ti);

    uint16_t year = 1900 + ti.tm_year;
    uint16_t day1 = ti.tm_yday + 1;

    char path[24];
    snprintf(path, sizeof(path), "/day_%04u%03u.bin", (unsigned)year, (unsigned)day1);

    if (LittleFS.exists(path)) {
      if (loadDayFile(path)) return;
    }

    clearData();
    return;
  }

  char newest[24] = {0};
  bool found = false;

  Dir dir = LittleFS.openDir("/");
  while (dir.next()) {
    String fn = dir.fileName();
    char base[24];
    if (!normalizeDayName(fn, base, sizeof(base))) continue;
    if (!found || strcmp(base, newest) > 0) {
      strncpy(newest, base, sizeof(newest)-1);
      newest[sizeof(newest)-1] = '\0';
      found = true;
    }
  }
  if (found) {
    char path[28];
    makeDayPath(newest, path, sizeof(path));
    loadDayFile(path);
  }
}
#endif // ENABLE_FS

// ===================== Time anchor =====================
static void anchorTimeToNow() {
  if (!isTimeValid()) return;
  timeAnchorEpoch  = time(nullptr);
  timeAnchorMillis = millis();
  timeAnchorValid  = true;
#if ENABLE_FS
  if (fsReady && millis() >= nextAnchorSave) {
    saveTimeAnchor();
    if (manualTimeActive) saveManualClock(timeAnchorEpoch);
    nextAnchorSave = millis() + ANCHOR_SAVE_MS;
  }
#endif
}

static time_t approxNow() {
  if (isTimeValid()) return time(nullptr);
  if (timeAnchorValid) return timeAnchorEpoch + (millis() - timeAnchorMillis)/1000;
  return 0;
}

// ===================== Sensor init =====================
#if !USE_DHT_S8
static void scanI2cBus() {
  i2cDevices = "";
  i2cDeviceCount = 0;
  i2cAddress61Present = false;

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      char addressText[6];
      snprintf(addressText, sizeof(addressText), "0x%02X", address);
      if (i2cDeviceCount) i2cDevices += ",";
      i2cDevices += addressText;
      i2cDeviceCount++;
      if (address == 0x61) i2cAddress61Present = true;
    }
    yield();
  }
  if (!i2cDeviceCount) i2cDevices = "none";
  Serial.printf("I2C scan on SDA=GPIO%u/SCL=GPIO%u: %s\n",
                I2C_SDA, I2C_SCL, i2cDevices.c_str());
}
#endif

static bool initSensor() {
#if USE_DHT_S8
  dht.begin();
  s8Serial.begin(9600);
  sensorInitialized=true;
  Serial.println("Sensors: DHT11 data=D6; S8 TX->D4, RX->D0; TFT BL=D3; button=A0.");
  return true;
#else
  lastSensorAttempt = millis();
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(50);
  scanI2cBus();

  for (int i = 0; i < 3; i++) {
    if (scd30.begin()) {
      scd30.setMeasurementInterval(5);
      scd30.setTemperatureOffset(0);
      scd30.selfCalibrationEnabled(SCD30_ASC ? true : false);
      sensorInitialized = true;
      Serial.println("SCD30 detected; waiting for first measurement.");
      return true;
    }
    delay(250);
  }
  Serial.printf("SCD30 not detected at I2C address 0x61; SDA=GPIO%u, SCL=GPIO%u. Check power and wiring.\n",
                I2C_SDA, I2C_SCL);
  return false;
#endif
}

// ===================== Network / Time =====================
static void startFallbackAP() {
  WiFi.disconnect(false);
  WiFi.mode(WIFI_AP_STA);

  bool apOk = false;
  if (AP_FALLBACK_PASS[0] != '\0') apOk = WiFi.softAP(AP_FALLBACK_SSID, AP_FALLBACK_PASS);
  else                             apOk = WiFi.softAP(AP_FALLBACK_SSID);

  offlineMode = true;
  if (softStartTime == 0) softStartTime = millis();
  apFallbackActive = apOk;

  if (apOk) {
    IPAddress apIP = WiFi.softAPIP();
    dnsServer.start(53, "*", apIP);
    Serial.print("Fallback AP started: SSID=");
    Serial.print(AP_FALLBACK_SSID);
    Serial.print(" IP=");
    Serial.println(apIP);
  } else {
    Serial.println("Fallback AP failed to start");
  }
}

static void onStaConnected() {
  offlineMode = false;
  Serial.print("WiFi connected, IP=");
  Serial.println(WiFi.localIP());

  if (apFallbackActive) {
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    apFallbackActive = false;
    Serial.println("Fallback AP stopped (STA connected)");
  }

  if (!mdnsStarted) {
    if (MDNS.begin("envmonitor")) {
      mdnsStarted = true;
      Serial.println("mDNS responder started: http://envmonitor.local");
      MDNS.addService("http", "tcp", 80);
    } else {
      Serial.println("mDNS responder FAILED");
    }
  }
}

static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.hostname("envmonitor");

  // Prefer the Fan Controller's permanent AP. This link needs no router or
  // internet connection and avoids opening WiFiManager during normal starts.
  WiFi.begin(FAN_DIRECT_SSID, FAN_DIRECT_PASSWORD);
  const uint32_t directStarted = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - directStarted < 15000UL) {
    delay(100);
    yield();
  }
  if (WiFi.status() == WL_CONNECTED) {
    onStaConnected();
    return;
  }

  WiFiManager wm;
  wm.setConfigPortalTimeout(120);
  wm.setConfigPortalBlocking(true);

  bool ok = wm.autoConnect("EnvMonitor-Setup");

  if (!ok) {
    Serial.println("WiFiManager: failed or timed out -> AP fallback mode");
    startFallbackAP();
  } else {
    onStaConnected();
  }
}

static void retryStaWhileAp() {
  if (!apFallbackActive) return;

  if (WiFi.status() == WL_CONNECTED) {
    onStaConnected();
    if (!manualTimeActive) {
      initTime();
      if (isTimeValid()) anchorTimeToNow();
    }
    return;
  }

  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastStaRetryMs) < WIFI_STA_RETRY_MS) return;
  lastStaRetryMs = nowMs;

  Serial.println("Retrying STA connection while AP is active...");
  WiFi.begin();
}

static void initTime() {
  if (manualTimeActive) {
    setenv("TZ", TZ_ADELAIDE, 1);
    tzset();
  } else if (!offlineMode) {
    configTime(TZ_ADELAIDE, "au.pool.ntp.org", "time.google.com", "time.windows.com");
    uint32_t t0 = millis();
    while (time(nullptr) <= 1600000000 && millis() - t0 < 12000) {
      delay(200);
      yield();
    }
    if (time(nullptr) <= 1600000000) {
      offlineMode = true;
      softStartTime = millis();
    }
  } else {
    setenv("TZ", TZ_ADELAIDE, 1);
    tzset();
  }

  anchorTimeToNow();
}

static bool isTimeValid() {
  return (manualTimeActive || !offlineMode) && time(nullptr) > 1600000000;
}

static void applyScheduledBacklight() {
  scheduledOff=false;
  if (scheduleEnabled && isTimeValid()) {
    time_t now=time(nullptr); struct tm local; localtime_r(&now, &local);
    const uint16_t minute=local.tm_hour*60+local.tm_min;
    scheduledOff=screenScheduleActive(minute,scheduleOff,scheduleOn);
  }
  analogWrite(BL_PIN, scheduledOff ? 0 : blValue);
}

static String scheduleTime(uint16_t minute) {
  char value[6]; snprintf(value,sizeof(value),"%02u:%02u",unsigned((minute/60)%24),unsigned(minute%60));
  return String(value);
}
static bool parseScheduleTime(const String& value, unsigned& minute) {
  if(value.length()!=5 || value[2]!=':') return false;
  for(unsigned i: {0U,1U,3U,4U}) if(value[i]<'0' || value[i]>'9') return false;
  unsigned h=(value[0]-'0')*10+value[1]-'0', m=(value[3]-'0')*10+value[4]-'0';
  if(h>23 || m>59) return false;
  minute=h*60+m; return true;
}

//  -----   Web UI  --------

static bool saveScreenSettings(uint8_t page, uint8_t rotation, uint16_t brightness, uint8_t type, bool enabled, uint16_t off, uint16_t on) {
#if ENABLE_FS
  if (!fsReady) return false;
  File f = LittleFS.open("/screen.tmp", "w");
  if (!f) return false;
  String data = String(page) + " " + String(rotation) + " " + String(brightness) + " " + String(type) + " " + String(enabled ? 1 : 0) + " " + String(off) + " " + String(on) + "\n";
  bool ok = f.print(data) == data.length();
  f.close();
  return ok && LittleFS.rename("/screen.tmp", "/screen.cfg");
#else
  return false;
#endif
}

static void loadScreenSettings() {
#if ENABLE_FS
  if (!fsReady) return;
  File f = LittleFS.open("/screen.cfg", "r");
  if (!f) return;
  unsigned page, rotation, brightness, type=0, enabled=0, off=22*60, on=7*60;
  String data = f.readStringUntil('\n');
  f.close();
  if (sscanf(data.c_str(), "%u %u %u %u %u %u %u", &page, &rotation, &brightness, &type, &enabled, &off, &on) >= 3 &&
      page < NUM_SCREENS && rotation <= 3 && brightness <= BL_RANGE && type <= 1) {
    displayType = type;
    screenMode = page;
    screenRotation = rotation;
    blValue = brightness;
    if(enabled<=1 && off<1440 && on<1440 && (!enabled || off!=on)) {
      scheduleEnabled=enabled; scheduleOff=off; scheduleOn=on;
    }
  }
#endif
}

static String jsonString(const String& value) {
  String out = "\"";
  for (size_t i=0; i<value.length(); ++i) {
    const uint8_t c = value[i];
    if (c == '"' || c == '\\') { out += '\\'; out += char(c); }
    else if (c < 32) { char escaped[7]; snprintf(escaped, sizeof(escaped), "\\u%04x", c); out += escaped; }
    else out += char(c);
  }
  return out + "\"";
}

static bool validFanHost(const String& host) {
  if(host.isEmpty() || host.length()>63 || host[0]=='.' || host[host.length()-1]=='.') return false;
  for(size_t i=0;i<host.length();++i) {
    char c=host[i];
    if(!((c>='a' && c<='z') || (c>='A' && c<='Z') || (c>='0' && c<='9') || c=='.' || c=='-')) return false;
  }
  return true;
}

static void loadFanSettings() {
#if ENABLE_FS
  if(!fsReady) return;
  File f=LittleFS.open("/fan.cfg","r"); if(!f) return;
  String enabled=f.readStringUntil('\n'), host=f.readStringUntil('\n'); f.close();
  enabled.trim(); host.trim();
  if((enabled=="0" || enabled=="1") && validFanHost(host)) {
    fanSendEnabled=enabled=="1"; fanHost=host;
  }
#endif
}

static void handleFanSave() {
  String enabled=server.arg("enabled"), host=server.arg("host"); host.trim();
  if((enabled!="0" && enabled!="1") || !validFanHost(host)) {
    server.send(400,"text/plain","Enter a controller IP address or hostname without http:// or a path."); return;
  }
#if ENABLE_FS
  if(!fsReady) { server.send(500,"text/plain","Settings storage unavailable"); return; }
  File f=LittleFS.open("/fan.tmp","w");
  String data=enabled+"\n"+host+"\n";
  bool ok=f && f.print(data)==data.length(); f.close();
  if(!ok || !LittleFS.rename("/fan.tmp","/fan.cfg")) { server.send(500,"text/plain","Could not save fan settings"); return; }
  fanSendEnabled=enabled=="1"; fanHost=host; fanClimateHttp=fanCo2Http=0;
  server.send(200,"text/plain","Fan controller settings saved. Readings send every 8 seconds when enabled.");
#else
  server.send(500,"text/plain","Settings storage unavailable");
#endif
}

static int postFanPacket(const String& host, const char* path, const String& packet) {
  WiFiClient client; client.setTimeout(1000);
  HTTPClient http; http.setTimeout(1000); http.setReuse(false);
  if(!http.begin(client,"http://"+host+path)) return -101;
  http.addHeader("Content-Type","application/json");
  http.addHeader("X-Sensor-Key",FAN_SENSOR_KEY);
  int result=http.POST(packet); http.end();
  return result;
}

static void sendFanReadings() {
  uint32_t now=millis();
  if(!fanSendEnabled || WiFi.status()!=WL_CONNECTED || now-fanLastAttempt<FAN_SEND_INTERVAL_MS) return;
#if USE_DHT_S8
  if(s8Pending) return;
#endif
  fanLastAttempt=now;
  IPAddress address;
  if(WiFi.SSID()==FAN_DIRECT_SSID) address=IPAddress(192,168,4,1);
  else if(!WiFi.hostByName(fanHost.c_str(),address,1000)) { fanClimateHttp=fanCo2Http=-100; return; }
  String host=address.toString();

  uint32_t climateAge=millis()-lastTempHumData;
  bool climateGood=hasTempHum && climateAge<=10000 && isfinite(currentTemp) && isfinite(currentHum);
  String climate="{\"node\":\"envmonitor\",\"boot\":"+String(fanBoot)+",\"seq\":"+String(++fanClimateSeq)+",\"valid\":"+(climateGood?"true":"false");
  if(climateGood) climate+=",\"age_ms\":"+String(climateAge)+",\"temperature\":"+String(currentTemp,1)+",\"humidity\":"+String(currentHum,1);
  climate+="}";
  fanClimateHttp=postFanPacket(host,"/api/climate",climate);

  uint32_t co2Age=millis()-lastCo2Data;
  bool co2Good=hasCo2 && co2Age<=10000 && isfinite(currentCO2);
  String co2="{\"node\":\"envmonitor\",\"boot\":"+String(fanBoot)+",\"seq\":"+String(++fanCo2Seq)+",\"valid\":"+(co2Good?"true":"false");
  if(co2Good) co2+=",\"age_ms\":"+String(co2Age)+",\"co2\":"+String(currentCO2,1);
  co2+="}";
  fanCo2Http=postFanPacket(host,"/api/sensor",co2);
}

static void sendAcTemperature() {
  const uint32_t now=millis();
  if(WiFi.status()!=WL_CONNECTED || now-acLastAttempt<15000) return;
#if USE_DHT_S8
  if(s8Pending) return;
#endif
  acLastAttempt=now;
  if(!hasTempHum || now-lastTempHumData>10000 || !isfinite(currentTemp)) {
    acLastHttp=0; return;
  }
  WiFiClient client; client.setTimeout(1000);
  HTTPClient http; http.setTimeout(1000); http.setReuse(false);
  const String url=String("http://")+AC_CONTROL_HOST+"/api/env?temp="+String(currentTemp,1);
  if(!http.begin(client,url)) { acLastHttp=-101; return; }
  acLastHttp=http.GET(); http.end();
}

static void handleSettings() {
  server.sendHeader("Cache-Control", "no-store");
  String out = "{\"screen\":" + String(screenMode) + ",\"rotation\":" + String(screenRotation);
  out += ",\"nativeWidth\":" + String(displayType == 1 ? 170 : 240);
  out += ",\"displayType\":" + String(displayType);
  out += ",\"nativeHeight\":" + String(ILI9341_TFTHEIGHT);
  out += ",\"screenWidth\":" + String(SCREEN_WIDTH);
  out += ",\"screenHeight\":" + String(SCREEN_HEIGHT);
  out += ",\"brightness\":" + String(blValue);
  out += ",\"scheduleEnabled\":" + String(scheduleEnabled ? "true" : "false");
  out += ",\"offTime\":" + jsonString(scheduleTime(scheduleOff));
  out += ",\"onTime\":" + jsonString(scheduleTime(scheduleOn));
  out += ",\"scheduleClockReady\":" + String(isTimeValid() ? "true" : "false");
  time_t clockEpoch = approxNow();
  char clockLocal[17] = "";
  if (clockEpoch > 1600000000) {
    struct tm local;
    localtime_r(&clockEpoch, &local);
    strftime(clockLocal, sizeof(clockLocal), "%Y-%m-%dT%H:%M", &local);
  }
  out += ",\"clockLocal\":" + jsonString(clockLocal);
  out += ",\"clockSource\":" + jsonString(manualTimeActive ? "manual" : (isTimeValid() ? "automatic" : "waiting"));
  out += ",\"clockRestored\":" + String(manualClockRestored ? "true" : "false");
  out += ",\"fanHost\":" + jsonString(fanHost);
  out += ",\"fanEnabled\":" + String(fanSendEnabled ? "true" : "false");
  out += ",\"fanClimateHttp\":" + String(fanClimateHttp);
  out += ",\"fanCo2Http\":" + String(fanCo2Http);
  out += ",\"acLastHttp\":" + String(acLastHttp);
  out += ",\"ssid\":" + jsonString(WiFi.SSID());
  out += ",\"connected\":"; out += WiFi.status() == WL_CONNECTED ? "true" : "false";
  out += ",\"ip\":" + jsonString(WiFi.localIP().toString());
  out += ",\"setupSsid\":" + jsonString(AP_FALLBACK_SSID);
  out += ",\"apActive\":"; out += apFallbackActive ? "true" : "false";
  out += ",\"pending\":"; out += wifiChangePending ? "true" : "false";
  out += "}";
  server.send(200, "application/json", out);
}

static bool parseManualLocalTime(const String& value, time_t& epoch) {
  if (value.length() != 16 || value[4]!='-' || value[7]!='-' ||
      value[10]!='T' || value[13]!=':') return false;
  for (unsigned i : {0U,1U,2U,3U,5U,6U,8U,9U,11U,12U,14U,15U})
    if (!isDigit(value[i])) return false;
  const int year = value.substring(0,4).toInt();
  const int month = value.substring(5,7).toInt();
  const int day = value.substring(8,10).toInt();
  const int hour = value.substring(11,13).toInt();
  const int minute = value.substring(14,16).toInt();
  if (year < 2021 || year > 2037 || month < 1 || month > 12 ||
      day < 1 || day > 31 || hour > 23 || minute > 59) return false;
  struct tm entered = {};
  entered.tm_year=year-1900; entered.tm_mon=month-1; entered.tm_mday=day;
  entered.tm_hour=hour; entered.tm_min=minute; entered.tm_isdst=-1;
  const time_t result = mktime(&entered);
  if (result < 1609459200 || result > 2145916799) return false;
  struct tm check;
  localtime_r(&result, &check);
  if (check.tm_year != year-1900 || check.tm_mon != month-1 ||
      check.tm_mday != day || check.tm_hour != hour || check.tm_min != minute) return false;
  epoch = result;
  return true;
}

static void handleTimeSave() {
  const String mode = server.arg("mode");
  if (mode == "auto") {
#if ENABLE_FS
    if (fsReady && LittleFS.exists("/manual_clock.cfg") && !LittleFS.remove("/manual_clock.cfg")) {
      server.send(500,"text/plain","Could not save automatic clock mode."); return;
    }
#endif
    manualTimeActive = false;
    manualClockRestored = false;
    offlineMode = WiFi.status() != WL_CONNECTED;
    configTime(TZ_ADELAIDE, "au.pool.ntp.org", "time.google.com", "time.windows.com");
    applyScheduledBacklight();
    drawAllScreens();
    server.send(200,"text/plain","Automatic time enabled. The clock will update when NTP syncs over Wi-Fi.");
    return;
  }
  if (mode != "manual") {
    server.send(400,"text/plain","Choose manual or automatic time."); return;
  }
#if ENABLE_FS
  if (!fsReady) { server.send(503,"text/plain","Clock storage is unavailable."); return; }
#else
  server.send(503,"text/plain","Clock storage is unavailable."); return;
#endif
  setenv("TZ", TZ_ADELAIDE, 1); tzset();
  time_t epoch;
  if (!parseManualLocalTime(server.arg("local"), epoch)) {
    server.send(400,"text/plain","Enter a valid Adelaide date and time from 2021 through 2037."); return;
  }
  const time_t oldEpoch = approxNow();
  struct tm oldDay = {}, newDay = {};
  const bool oldDayKnown = oldEpoch > 1600000000;
  if (oldDayKnown) localtime_r(&oldEpoch, &oldDay);
  localtime_r(&epoch, &newDay);
  const bool sameDay = oldDayKnown && oldDay.tm_year == newDay.tm_year &&
                       oldDay.tm_yday == newDay.tm_yday;
#if ENABLE_FS
  if (currentSlot >= 0) saveDataToFS(false);
  if (!saveManualClock(epoch)) {
    server.send(500,"text/plain","Could not save manual clock time."); return;
  }
#endif
  sntp_stop();
  timeval tv{epoch,0};
  if (settimeofday(&tv,nullptr) != 0) {
    server.send(500,"text/plain","Could not set device clock."); return;
  }
  manualTimeActive = true;
  manualClockRestored = false;
  timeAnchorEpoch = epoch;
  timeAnchorMillis = millis();
  timeAnchorValid = true;
  nextAnchorSave = millis() + ANCHOR_SAVE_MS;
#if ENABLE_FS
  saveTimeAnchor();
  if (!sameDay) loadDataFromFS();
#endif
  struct tm local;
  localtime_r(&epoch,&local);
  currentSlot = (local.tm_hour*60+local.tm_min)/SLOT_MINUTES;
  lastDrawnSlot = currentSlot;
  resetAccumulators();
  twelveGraphResetOriginSlot = -1;
  twentyFourGraphResetOriginSlot = -1;
  applyScheduledBacklight();
  drawAllScreens();
  server.send(200,"text/plain","Manual Adelaide time saved and applied.");
}

static bool readSetting(const char* name, unsigned maximum, unsigned& value) {
  if (!server.hasArg(name)) return false;
  String s = server.arg(name);
  if (s.length() == 0 || s.length() > 5) return false;
  value = 0;
  for (size_t i=0; i<s.length(); ++i) {
    if (s[i] < '0' || s[i] > '9') return false;
    value = value * 10 + s[i] - '0';
  }
  return value <= maximum;
}

static void handleScreenSave() {
  unsigned page, rotation, brightness, type=displayType;
  if ((server.hasArg("displayType") && !readSetting("displayType", 1, type)) || !readSetting("screen", NUM_SCREENS-1, page) ||
      !readSetting("rotation", 3, rotation) ||
      !readSetting("brightness", BL_RANGE, brightness)) {
    server.send(400, "text/plain", "Invalid screen settings"); return;
  }
  unsigned off=scheduleOff, on=scheduleOn, enabled=scheduleEnabled;
  if ((server.hasArg("scheduleEnabled") && !readSetting("scheduleEnabled",1,enabled)) ||
      (server.hasArg("offTime") && !parseScheduleTime(server.arg("offTime"),off)) ||
      (server.hasArg("onTime") && !parseScheduleTime(server.arg("onTime"),on)) || (enabled && off==on)) {
    server.send(400,"text/plain","Enter different valid off/on times (HH:MM)."); return;
  }
  if (!saveScreenSettings(page, rotation, brightness, type, enabled, off, on)) {
    server.send(500, "text/plain", "Could not save screen settings"); return;
  }
  const bool changedDisplay = displayType != type;
  displayType = type;
  screenMode = page; screenRotation = rotation; blValue = brightness;
  scheduleEnabled=enabled; scheduleOff=off; scheduleOn=on;
  if (changedDisplay) initDisplay();
  else applyScreenRotation();
  applyScheduledBacklight();
  drawAllScreens();
  server.send(200, "text/plain", "Screen settings saved and applied.");
}

static void handleWifiSave() {
  if (wifiChangePending) { server.send(409, "text/plain", "Wi-Fi change already pending"); return; }
  String ssid = server.arg("ssid"), password = server.arg("password");
  bool open = server.arg("open") == "1";
  bool hexKey = password.length() == 64;
  if (hexKey) for (size_t i=0; i<password.length(); ++i) if (!isxdigit((unsigned char)password[i])) hexKey = false;
  if (ssid.length() == 0 || ssid.length() > 32 || strlen(ssid.c_str()) != ssid.length() ||
      (!open && !hexKey && (password.length() < 8 || password.length() > 63)) || strlen(password.c_str()) != password.length()) {
    server.send(400, "text/plain", "Enter an SSID (1-32 bytes) and password (8-63 characters or 64 hex digits), or select Open network."); return;
  }
  pendingSsid = ssid; pendingPassword = open ? "" : password;
  wifiChangePending = true; wifiChangeAt = millis();
  server.send(202, "text/plain", "Wi-Fi settings accepted. Connecting shortly; this page may disconnect. Open http://envmonitor.local on the new network. If connection fails, join the setup network and open http://192.168.4.1 to correct the settings.");
}

static void applyPendingWifi() {
  if (!wifiChangePending || millis() - wifiChangeAt < 1500) return;
  if (mdnsStarted) { MDNS.close(); mdnsStarted = false; }
  WiFi.persistent(false);
  if (!apFallbackActive) startFallbackAP();
  else WiFi.disconnect(false);
  WiFi.persistent(true);
  WiFi.begin(pendingSsid.c_str(), pendingPassword.c_str());
  WiFi.persistent(false);
  pendingPassword = ""; pendingSsid = "";
  lastStaRetryMs = millis();
  wifiChangePending = false;
}

// (web UI HTML unchanged — paste it back in verbatim from your original file)
static const char WEB_ROOT_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#0b1020"><meta name="color-scheme" content="dark light">
<title>Environment Monitor</title>
<style>
:root{
  --bg:#0b1020;--panel:#10172c;--text:#eaf0ff;--muted:rgba(234,240,255,.72);
  --grid:rgba(255,255,255,.10);--stroke:rgba(255,255,255,.12);--btn:rgba(255,255,255,.06);
  --input-bg:rgba(0,0,0,.16);--input-text:var(--text);
  --accent:#fad24f;--co2:#ff785a;--temp:#50d2ff;--hum:#8cb4ff;--r:16px;
}
[data-theme="light"]{
  --bg:#f6f7fb;--panel:#ffffff;--text:#0f1625;--muted:rgba(15,22,37,.62);
  --grid:rgba(15,22,37,.10);--stroke:rgba(15,22,37,.12);--btn:rgba(15,22,37,.06);
  --input-bg:#ffffff;--input-text:#000000;
  --accent:#aa7d00;
}
*{box-sizing:border-box}html,body{height:100%}
body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;background:var(--bg);color:var(--text)}
.wrap{max-width:1100px;margin:0 auto;padding:12px}
.top{
  position:sticky;top:0;z-index:5;padding:10px 0;background:linear-gradient(180deg,var(--bg),rgba(0,0,0,0));
  backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px)
}
.head{
  border:1px solid var(--stroke);border-radius:var(--r);background:rgba(255,255,255,.04);
  padding:12px;display:flex;gap:10px;align-items:center;justify-content:space-between
}
.h1{display:flex;flex-direction:column;gap:2px}
.h1 b{font-size:1.05rem} .h1 span{color:var(--muted);font-size:.9rem}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;justify-content:flex-end}
.card{border:1px solid var(--stroke);border-radius:var(--r);background:rgba(255,255,255,.04);padding:12px;margin-top:10px}
#deviceSettings summary{cursor:pointer;font-weight:600}
#deviceSettings fieldset{min-width:0;border:1px solid var(--stroke);border-radius:8px}
#deviceSettings .controls>label{display:flex;flex-wrap:wrap;align-items:center;gap:8px;max-width:100%}
#deviceSettings input:not([type=checkbox]){max-width:100%;padding:8px;background:var(--input-bg);color:var(--input-text);border:1px solid var(--stroke);border-radius:8px}
#deviceSettings select{max-width:100%}
#settingsMessage{overflow-wrap:anywhere}
.controls{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
.controls.scroll{
  flex-wrap:nowrap;
  overflow-x:auto;
  -webkit-overflow-scrolling:touch;
  scrollbar-width:none;
  scroll-snap-type:x mandatory;
}
.controls.scroll::-webkit-scrollbar{display:none}
.controls.scroll > *{scroll-snap-align:start}
.sp{flex:1}
button,select,label{font:inherit}
button{
  height:34px;border-radius:999px;border:1px solid var(--stroke);background:var(--btn);color:var(--text);
  padding:0 11px;cursor:pointer;touch-action:manipulation
}
select{
  height:34px;border-radius:999px;border:1px solid var(--stroke);background:var(--input-bg);color:var(--input-text);
  padding:0 11px;cursor:pointer;touch-action:manipulation
}
button:active{transform:translateY(1px)}
select{min-width:220px}
@media(max-width:720px){select{min-width:170px}}
.pill{padding:0 10px}
.chip{
  display:inline-flex;align-items:center;gap:8px;padding:6px 10px;border-radius:999px;
  border:1px solid var(--stroke);background:rgba(0,0,0,.14);color:var(--text);user-select:none
}
.chip.stale{opacity:.55}
.dot{width:10px;height:10px;border-radius:50%}
.dot.co2{background:var(--co2)}.dot.t{background:var(--temp)}.dot.h{background:var(--hum)}
canvas{
  width:100%;height:420px;display:block;border-radius:14px;border:1px solid var(--stroke);
  background:rgba(0,0,0,.12);
  touch-action:none; /* important for pan/pinch */
}
@media(max-width:720px){canvas{height:320px}}
.small{color:var(--muted);font-size:.9rem}
.grid3{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px}
@media(max-width:860px){.grid3{grid-template-columns:1fr}}
.stat{border:1px solid var(--stroke);border-radius:14px;background:rgba(0,0,0,.10);padding:10px}
.stat .k{color:var(--muted);font-size:.85rem;margin-bottom:4px}
.stat .v{font-weight:800}
#tip{
  position:fixed;z-index:50;pointer-events:none;display:none;white-space:nowrap;
  background:rgba(8,10,22,.92);color:var(--text);border:1px solid rgba(255,255,255,.18);
  padding:7px 10px;border-radius:12px;font-size:.86rem
}
[data-theme="light"] #tip{
  background:#ffffff;
  color:#000000;
  border:1px solid rgba(15,22,37,.22);
}

/* --- mobile comfort --- */
@media(max-width:720px){
  .wrap{padding:10px}
  .head{padding:10px}
  .card{padding:10px}
  button,select{
    height:42px;
    padding:0 14px;
    font-size:15px;
  }
  .pill{padding:0 14px}
  select{
    min-width:0;
    max-width:100%;
    flex:1 1 auto;
  }
  .small{font-size:.86rem}
}
@media(max-width:420px){
  canvas{height:280px}
  .row button{width:44px;padding:0}
}
</style></head><body><div class="wrap">
<div class="top"><div class="head">
  <div class="h1"><b>Environment Monitor</b><span>Live + History · ACST/ACDT</span></div>
  <div class="row">
    <button id="btnTheme" title="Toggle theme">🌓</button>
    <button id="btnPNG" title="Export PNG">📷</button>
  </div>
</div></div>

<p class="small" id="readingStatus" role="status" aria-live="polite">Waiting for device readings...</p>
<p class="small" id="dataStatus" role="status"></p>

<div class="card">
  <div class="controls scroll">
    <button class="pill" id="prev">◀</button>
    <button class="pill" id="next">▶</button>
    <select id="day"></select>
    <label class="chip"><input id="smooth" type="checkbox" checked style="accent-color:var(--accent)"> Smooth</label>
    <select id="mode" style="min-width:150px">
      <option value="line">CO₂ Line</option>
      <option value="area">CO₂ Area</option>
      <option value="bar">CO₂ Bars</option>
    </select>
    <button class="pill" data-r="6h">6h</button>
    <button class="pill" data-r="12h">12h</button>
    <button class="pill" data-r="24h">24h</button>
    <button class="pill" data-r="all">All</button>
    <div class="sp"></div>
    <div class="chip" id="liveChip"><span class="dot co2"></span><span id="live">Live: --</span></div>
  </div>
</div>

<div class="card">
  <div class="controls scroll">
    <label class="chip"><input id="cOn" type="checkbox" checked style="accent-color:var(--co2)"><span class="dot co2"></span>CO₂</label>
    <label class="chip"><input id="tOn" type="checkbox" checked style="accent-color:var(--temp)"><span class="dot t"></span>Temp</label>
    <label class="chip"><input id="hOn" type="checkbox" checked style="accent-color:var(--hum)"><span class="dot h"></span>Hum</label>
    <div class="sp"></div>
    <div class="small" id="hint">Hover: tooltip · Click: lock · Wheel: zoom</div>
  </div>
  <div style="margin-top:10px"><canvas id="cv"></canvas></div>
</div>

<div class="card" id="stats">
  <div class="controls" style="margin-bottom:8px">
    <b id="stTitle">Stats</b>
    <div class="sp"></div>
    <span class="small" id="stInfo">Samples —</span>
  </div>
  <div class="grid3">
    <div class="stat"><div class="k">CO₂</div><div class="v" id="sC">Avg —</div><div class="small" id="rC">Min — · Max —</div></div>
    <div class="stat"><div class="k">Temp</div><div class="v" id="sT">Avg —</div><div class="small" id="rT">Min — · Max —</div></div>
    <div class="stat"><div class="k">Humidity</div><div class="v" id="sH">Avg —</div><div class="small" id="rH">Min — · Max —</div></div>
  </div>
</div>

<details class="card" id="deviceSettings">
  <summary>Device settings: screen, Wi-Fi and fan controller</summary>
  <p class="small" id="networkInfo">Open settings to load device configuration.</p>
  <form id="screenForm">
    <fieldset><legend>Screen setup</legend>
      <p class="small">TFT size: <output id="screenSize">Loading...</output> (width &times; height, pixels).</p>
      <label>Display model <select id="displayType" name="displayType">
        <option value="0">ILI9341 - 240 x 320</option>
        <option value="1">1.9-inch ST7789 - 170 x 320</option>
      </select></label>
      <p class="small">Select the connected display. ST7789 reset connects to D1; other display pins stay the same.</p>
      <div class="controls">
        <label>Display page <select id="screenPage" name="screen">
          <option value="0">30-minute dashboard</option><option value="1">Daily statistics</option>
          <option value="2">24-hour CO2</option><option value="3">24-hour temperature / humidity</option>
          <option value="4">12-hour CO2</option><option value="5">12-hour temperature / humidity</option>
        </select></label>
        <label>Orientation <select id="screenRotation" name="rotation">
          <option value="1">Landscape (horizontal)</option><option value="3">Landscape flipped</option>
          <option value="0">Portrait (vertical)</option><option value="2">Portrait flipped</option>
        </select></label>
        <label>Brightness <input id="screenBrightness" name="brightness" type="range" min="0" max="1023" value="750">
          <output id="brightnessValue">73%</output></label>
        <label>Daily screen schedule <select id="scheduleEnabled" name="scheduleEnabled"><option value="0">Disabled</option><option value="1">Enabled</option></select></label>
        <label>Screen off <input id="offTime" name="offTime" type="time" required value="22:00"></label>
        <label>Screen on <input id="onTime" name="onTime" type="time" required value="07:00"></label>
        <button id="saveScreen" type="submit" disabled>Save screen settings</button>
      </div>
      <p class="small">Applies immediately and restores on restart. Brightness 0 turns the backlight off. Schedule uses Adelaide local time, including daylight saving; overnight periods are supported. Logging continues while the screen is off. The schedule uses manual time or waits for automatic clock sync.</p><p class="small" id="scheduleStatus"></p>
    </fieldset>
  </form>
  <form id="timeForm" style="margin-top:12px">
    <fieldset><legend>Clock</legend>
      <p class="small" id="clockStatus">Open settings to check the device clock.</p>
      <div class="controls">
        <label>Adelaide date and time <input id="manualDateTime" name="local" type="datetime-local" min="2021-01-01T00:00" max="2037-12-31T23:59" required></label>
        <button id="saveTime" type="submit" disabled>Set manual time</button>
        <button id="autoTime" type="button" disabled>Use automatic time</button>
      </div>
      <p class="small">Manual time runs without Wi-Fi and controls the screen schedule and logging. The device has no battery-backed clock; after a power loss it resumes from the last saved time, so check it again.</p>
    </fieldset>
  </form>
  <form id="wifiForm" style="margin-top:12px">
    <fieldset><legend>Wi-Fi configuration</legend>
      <div class="controls">
        <label>Network name (SSID) <input id="wifiSsid" name="ssid" maxlength="32" required autocomplete="off"></label>
        <label>Password <input id="wifiPassword" name="password" type="password" maxlength="64" required autocomplete="new-password"></label>
        <label><input id="wifiOpen" type="checkbox"> Open network (no password)</label>
        <button id="saveWifi" type="submit" disabled>Save and connect</button>
      </div>
      <p class="small">Enter the network password when saving. Saved passwords are never displayed. Changing networks may disconnect this page.</p>
    </fieldset>
  </form>
  <form id="fanForm" style="margin-top:12px">
    <fieldset><legend>Fan controller readings</legend>
      <div class="controls">
        <label>Send temperature, humidity and CO2 <select id="fanEnabled" name="enabled"><option value="0">Disabled</option><option value="1">Enabled</option></select></label>
        <label>Fan controller IP or hostname <input id="fanHost" name="host" maxlength="63" required placeholder="greenhouse-fan.local"></label>
        <button id="saveFan" type="submit" disabled>Save fan connection</button>
      </div>
      <p class="small">Both devices must be on the same reachable Wi-Fi network. The fan controller uses its existing sensors first and EnvMonitor when a reading becomes stale. Sends every 8 seconds, including while the screen is off.</p>
      <p class="small" id="fanStatus">Open settings to see send status.</p>
      <p class="small" id="acStatus">AC controller temperature: waiting.</p>
    </fieldset>
  </form>
  <p id="settingsMessage" role="status" aria-live="polite"></p>
</details>
<div id="tip"></div>
</div>

<script>
(() => {
  const TODAY="__TODAY__";
  const $=id=>document.getElementById(id);
  const settingsMessage = $("settingsMessage");
  let nativeScreenWidth=0, nativeScreenHeight=0;
  function updateScreenSize(){
    nativeScreenWidth=$("displayType").value==="1"?170:240;
    nativeScreenHeight=320;
    const landscape=Number($("screenRotation").value)%2===1;
    const width=landscape?nativeScreenHeight:nativeScreenWidth;
    const height=landscape?nativeScreenWidth:nativeScreenHeight;
    $("screenSize").textContent=width+" × "+height;
  }
  $("screenRotation").addEventListener("change", updateScreenSize);
  $("displayType").addEventListener("change", updateScreenSize);
  const updateBrightness = () => $("brightnessValue").textContent = Math.round(Number($("screenBrightness").value)*100/1023)+"%";
  $("screenBrightness").addEventListener("input", updateBrightness);
  $("wifiOpen").addEventListener("change", () => {
    $("wifiPassword").disabled = $("wifiOpen").checked;
    $("wifiPassword").required = !$("wifiOpen").checked;
  });
  $("deviceSettings").addEventListener("toggle", async () => {
    if (!$("deviceSettings").open) return;
    try {
      const response = await fetch("/api/settings", {cache:"no-store"});
      if (!response.ok) throw new Error("Could not load settings");
      const s = await response.json();
      $("scheduleEnabled").value=s.scheduleEnabled?"1":"0";
      $("offTime").value=s.offTime; $("onTime").value=s.onTime;
      $("scheduleStatus").textContent=s.scheduleClockReady?"Schedule clock ready.":"Waiting for clock sync; scheduled screen-off is inactive.";
      $("manualDateTime").value=s.clockLocal||"";
      $("clockStatus").textContent=s.clockSource==="manual"
        ? "Manual Adelaide time: "+(s.clockLocal||"--")+(s.clockRestored?" (restored after restart; please check).":"")
        : s.clockSource==="automatic" ? "Automatic mode (NTP enabled): "+(s.clockLocal||"--") : "Waiting for automatic clock sync.";
      $("screenPage").value = s.screen;
      $("screenRotation").value = s.rotation;
      $("displayType").value=s.displayType;
      nativeScreenWidth=s.nativeWidth;
      nativeScreenHeight=s.nativeHeight;
      updateScreenSize();
      $("screenBrightness").value = s.brightness;
      updateBrightness();
      $("wifiSsid").value = s.ssid;
      $("fanHost").value=s.fanHost; $("fanEnabled").value=s.fanEnabled?"1":"0";
      $("fanStatus").textContent=!s.fanEnabled?"Sending disabled":
        "Temperature / humidity: "+(s.fanClimateHttp===200?"accepted":s.fanClimateHttp===0?"waiting":"HTTP "+s.fanClimateHttp)+
        " · CO2: "+(s.fanCo2Http===200?"accepted":s.fanCo2Http===0?"waiting":"HTTP "+s.fanCo2Http);
      $("acStatus").textContent="AC controller temperature: "+(s.acLastHttp===200?"accepted":s.acLastHttp===0?"waiting for a fresh reading":"HTTP "+s.acLastHttp);
      $("networkInfo").textContent = (s.connected ? "Connected to " + s.ssid + " at " + s.ip : "Wi-Fi disconnected") +
        ". Recovery network: " + s.setupSsid + " (192.168.4.1)" + (s.apActive ? " - active." : " - available if setup fails.");
      $("saveScreen").disabled = false;
      $("saveTime").disabled = false;
      $("autoTime").disabled = false;
      $("saveWifi").disabled = s.pending;
      $("saveFan").disabled = false;
    } catch(e) { settingsMessage.textContent = "Could not load settings. Close and reopen settings to retry."; }
  });
  async function saveSettings(event, url, button) {
    event.preventDefault();
    button.disabled = true;
    settingsMessage.textContent = "Saving...";
    try {
      const body = new URLSearchParams(new FormData(event.target));
      if (url === "/api/wifi") body.set("open", $("wifiOpen").checked ? "1" : "0");
      const response = await fetch(url, {method:"POST", body});
      const message = await response.text();
      if (!response.ok) throw new Error(message);
      settingsMessage.textContent = message;
      if (url === "/api/wifi") $("wifiPassword").value = "";
    } catch(e) { settingsMessage.textContent = "Save could not be confirmed: " + e.message; }
    finally { button.disabled = false; }
  }
  $("screenForm").addEventListener("submit", e => saveSettings(e, "/api/screen", $("saveScreen")));
  async function updateClock(mode){
    $("saveTime").disabled=true; $("autoTime").disabled=true;
    settingsMessage.textContent="Setting device clock...";
    try {
      const body=new URLSearchParams({mode});
      if(mode==="manual") body.set("local",$("manualDateTime").value);
      const response=await fetch("/api/time",{method:"POST",body});
      const message=await response.text();
      if(!response.ok) throw new Error(message);
      settingsMessage.textContent=message;
      const fresh=await (await fetch("/api/settings",{cache:"no-store"})).json();
      $("manualDateTime").value=fresh.clockLocal||"";
      $("clockStatus").textContent=fresh.clockSource==="manual"
        ? "Manual Adelaide time: "+fresh.clockLocal
        : fresh.clockSource==="automatic" ? "Automatic mode (NTP enabled): "+fresh.clockLocal : "Waiting for automatic clock sync.";
      $("scheduleStatus").textContent=fresh.scheduleClockReady?"Schedule clock ready.":"Waiting for clock sync; scheduled screen-off is inactive.";
    } catch(e) { settingsMessage.textContent="Clock change failed: "+e.message; }
    finally { $("saveTime").disabled=false; $("autoTime").disabled=false; }
  }
  $("timeForm").addEventListener("submit",e=>{e.preventDefault();updateClock("manual");});
  $("autoTime").addEventListener("click",()=>updateClock("auto"));
  $("wifiForm").addEventListener("submit", e => saveSettings(e, "/api/wifi", $("saveWifi")));
  $("fanForm").addEventListener("submit", e => saveSettings(e, "/api/fan", $("saveFan")));
  const LS={g:(k,d)=>{try{const v=localStorage.getItem(k);return v==null?d:JSON.parse(v)}catch(e){return d}},
            s:(k,v)=>{try{localStorage.setItem(k,JSON.stringify(v))}catch(e){}}};

  const sel=$("day"), cv=$("cv"), ctx=cv.getContext("2d"), tip=$("tip");
  const liveEl=$("live"), liveChip=$("liveChip");
  const smoothEl=$("smooth"), modeEl=$("mode"), cOn=$("cOn"), tOn=$("tOn"), hOn=$("hOn");

  let dayList=[], data=null, lastStatusT=0;
  let viewMin=0, viewMax=0;
  let DPR=1, cssW=800, cssH=360;
  let hover=null, locked=false, lastPX=0, lastPY=0;

  function setTheme(th){ document.documentElement.dataset.theme=th; LS.s("theme",th); draw(); }
  $("btnTheme").onclick=()=>setTheme((document.documentElement.dataset.theme==="light")?"dark":"light");

  function toRGBA(hex,a){
    const r=parseInt(hex.slice(1,3),16), g=parseInt(hex.slice(3,5),16), b=parseInt(hex.slice(5,7),16);
    return `rgba(${r},${g},${b},${a})`;
  }
  function fmtTime(i, stepMin){
    const m=(i|0)*(stepMin||5), hh=Math.floor(m/60)%24, mm=m%60;
    return String(hh).padStart(2,"0")+":"+String(mm).padStart(2,"0");
  }

  async function fetchJSON(url, signal){
    const controller=new AbortController();
    const abort=()=>controller.abort();
    if(signal) { if(signal.aborted) abort(); else signal.addEventListener("abort",abort,{once:true}); }
    const timer=setTimeout(abort,10000);
    try {
      const r=await fetch(url,{cache:"no-store",signal:controller.signal});
      if(!r.ok) throw new Error("HTTP "+r.status);
      return await r.json();
    } finally { clearTimeout(timer); if(signal) signal.removeEventListener("abort",abort); }
  }

  let statusBusy=false, sensorStale=true;
  async function fetchStatus(){
    if(statusBusy) return;
    statusBusy=true;
    try {
      const js=await fetchJSON("/status");
      lastStatusT=Date.now(); sensorStale=!!js.sensorStale;
      const co2=js.co2!=null?Math.round(js.co2):"--";
      const t=js.temp!=null?Number(js.temp).toFixed(1):"--";
      const h=js.hum!=null?Math.round(js.hum):"--";
      liveEl.textContent=`${sensorStale?"Last reading":"Live"}: ${co2} ppm | ${t} C | ${h} %`;
      if(!js.sensorReady && js.i2cDevices!==undefined) {
        $("readingStatus").textContent=`SCD30 not detected at 0x61. I2C scan found: ${js.i2cDevices}. Check the sensor model, power, SDA and SCL.`;
      } else if(js.sensorType) {
        $("readingStatus").textContent=
          `${js.sensorType}: temperature/humidity ${js.tempHumFresh?"OK":"waiting or unavailable"}; CO2 ${js.co2Fresh?"OK":"waiting or unavailable"}. ${js.co2Fresh?"":(js.co2Status||"")}`;
      } else if(js.sampleAgeMs==null) {
        $("readingStatus").textContent="Waiting for the first valid sensor reading.";
      } else {
        $("readingStatus").textContent=sensorStale ? "Sensor readings have stopped. Retrying." : "Sensor OK.";
      }
    } catch(e) { $("readingStatus").textContent="Device not responding. Retrying automatically; check your Wi-Fi connection."; }
    finally { statusBusy=false; tickLiveChip(); }
  }
  function tickLiveChip(){
    liveChip.classList.toggle("stale", sensorStale || Date.now()-lastStatusT>15000);
  }

  function buildDaySelect(){
    sel.innerHTML="";
    const o0=document.createElement("option");
    o0.value=TODAY; o0.textContent="Today (live)";
    sel.appendChild(o0);

    dayList.forEach(d=>{
      const o=document.createElement("option");
      if(typeof d==="string"){ o.value=d; o.textContent=d; }
      else { o.value=d.name; o.textContent=d.label||d.name; }
      sel.appendChild(o);
    });

    const last=LS.g("daySel",TODAY);
    const ok=[...sel.options].some(o=>o.value===last);
    sel.value=ok?last:TODAY;
  }

  function fetchDayList(){
    fetchJSON("/api/daylist").then(list=>{
      dayList=Array.isArray(list)?list:[];
      buildDaySelect();
      load(sel.value,true);
    }).catch(()=>{
      buildDaySelect(); sel.value=TODAY; load(TODAY,true);
      $("dataStatus").textContent="History list unavailable; showing today. Reload to retry history.";
    });
  }

  function shiftDay(dir){
    let i=sel.selectedIndex+dir;
    i=Math.max(0,Math.min(sel.options.length-1,i));
    sel.selectedIndex=i;
    load(sel.value,true);
  }
  $("prev").onclick=()=>shiftDay(-1);
  $("next").onclick=()=>shiftDay(+1);

  function restorePrefs(){
    setTheme(LS.g("theme","dark"));
    smoothEl.checked=LS.g("smooth",true);
    modeEl.value=LS.g("mode","line");
    cOn.checked=LS.g("cOn",true);
    tOn.checked=LS.g("tOn",true);
    hOn.checked=LS.g("hOn",true);
  }
  function savePrefs(){
    LS.s("smooth",!!smoothEl.checked);
    LS.s("mode",modeEl.value||"line");
    LS.s("cOn",!!cOn.checked);
    LS.s("tOn",!!tOn.checked);
    LS.s("hOn",!!hOn.checked);
  }
  [smoothEl,modeEl,cOn,tOn,hOn].forEach(el=>el.addEventListener("change",()=>{savePrefs(); draw(); renderStats();}));

  let loadController=null, loadVersion=0, dataBusy=false;
  let lastGraphFetch=0;
  async function load(name, resetView){
    if(dataBusy && !resetView) return;
    if(loadController) loadController.abort();
    loadController=new AbortController();
    const version=++loadVersion;
    dataBusy=true;
    LS.s("daySel",name);
    const url = (name===TODAY) ? "/api/today" : ("/api/daydata?name="+encodeURIComponent(name));
    try {
      const d=await fetchJSON(url,loadController.signal);
      if(version!==loadVersion) return;
      lastGraphFetch=Date.now();
      $("dataStatus").textContent="";
      if(!d || !Array.isArray(d.co2) || d.co2.length<2){ data=null; draw(); renderStats(); return; }
      data=d;
      const N=data.co2.length;
      if(resetView){ viewMin=0; viewMax=N-1; }
      else{
        viewMin=Math.max(0,Math.min(viewMin,N-1));
        viewMax=Math.max(0,Math.min(viewMax,N-1));
        if(viewMax<=viewMin){ viewMin=0; viewMax=N-1; }
      }
      hover=null; locked=false;
      $("stTitle").textContent=(name===TODAY)?"Today stats":"Day stats";
      renderStats();
      draw();
    } catch(e) {
      if(version===loadVersion) $("dataStatus").textContent="Graph update failed; keeping the last data. Retrying live updates automatically.";
    } finally { if(version===loadVersion) dataBusy=false; }
  }
  sel.addEventListener("change",()=>load(sel.value,true));

  document.querySelectorAll("button[data-r]").forEach(b=>{
    b.addEventListener("click",()=>{
      if(!data||!Array.isArray(data.co2))return;
      const N=data.co2.length, step=data.stepMin||5;
      const r=b.dataset.r;
      if(r==="all"){ viewMin=0; viewMax=N-1; draw(); return; }
      const mins=(r==="6h")?360:(r==="12h")?720:1440;
      const pts=Math.max(12,Math.round(mins/step));
      viewMax=N-1; viewMin=Math.max(0,viewMax-(pts-1));
      draw();
    });
  });

  function resize(){
    const r=cv.getBoundingClientRect();
    DPR=window.devicePixelRatio||1;
    cssW=Math.max(320,Math.round(r.width));
    cssH=Math.max(240,Math.round(r.height));
    const W=Math.round(cssW*DPR), H=Math.round(cssH*DPR);
    if(cv.width!==W||cv.height!==H){ cv.width=W; cv.height=H; }
    ctx.setTransform(DPR,0,0,DPR,0,0);
    draw();
  }
  let rt=0; window.addEventListener("resize",()=>{clearTimeout(rt);rt=setTimeout(resize,120)});

  function smooth(arr, isGap){
    if(!smoothEl.checked) return arr;
    const out=arr.slice();
    for(let i=0;i<arr.length;i++){
      const v=arr[i]; if(isGap(v)) continue;
      let s=0,c=0;
      for(let k=-1;k<=1;k++){
        const j=i+k; if(j<0||j>=arr.length) continue;
        const vv=arr[j]; if(isGap(vv)) continue;
        s+=vv; c++;
      }
      if(c) out[i]=s/c;
    }
    return out;
  }

  function scan(arr,isGap){
    let mn=1e9,mx=-1e9,sum=0,c=0;
    for(let i=0;i<arr.length;i++){
      const v=arr[i]; if(isGap(v)) continue;
      mn=Math.min(mn,v); mx=Math.max(mx,v); sum+=v; c++;
    }
    return c?{mn,mx,avg:sum/c,c}:{mn:null,mx:null,avg:null,c:0};
  }
  function renderStats(){
    const sC=$("sC"),sT=$("sT"),sH=$("sH"),rC=$("rC"),rT=$("rT"),rH=$("rH"),info=$("stInfo");
    if(!data){ sC.textContent=sT.textContent=sH.textContent="Avg —"; rC.textContent=rT.textContent=rH.textContent="Min — · Max —"; info.textContent="Samples —"; return; }
    const co2=data.co2||[], t=data.temp||[], h=data.hum||[];
    const C=scan(co2,v=>!v), T=scan(t,v=>v==null), H=scan(h,v=>v==null);
    sC.textContent=(C.avg!=null)?`Avg ${Math.round(C.avg)} ppm`:"Avg —";
    sT.textContent=(T.avg!=null)?`Avg ${Number(T.avg).toFixed(1)} °C`:"Avg —";
    sH.textContent=(H.avg!=null)?`Avg ${Math.round(H.avg)} %`:"Avg —";
    rC.textContent=(C.mn!=null)?`Min ${Math.round(C.mn)} · Max ${Math.round(C.mx)}`:"Min — · Max —";
    rT.textContent=(T.mn!=null)?`Min ${Number(T.mn).toFixed(1)} · Max ${Number(T.mx).toFixed(1)}`:"Min — · Max —";
    rH.textContent=(H.mn!=null)?`Min ${Math.round(H.mn)} % · Max ${Math.round(H.mx)} %`:"Min — · Max —";
    const n=(data.slots||co2.length||360);
    info.textContent=`Samples ${Math.max(C.c,T.c,H.c)}/${n}`;
  }

  function pad(mn,mx,p){ const r=(mx-mn)||1; return [mn-r*p,mx+r*p]; }
  function yMap(v,mn,mx,gy0,gy1){ return gy1 - (v-mn)/(mx-mn) * (gy1-gy0); }

  function roundRectPath(x,y,w,h,r){
    r=Math.max(0,Math.min(r,Math.min(w,h)/2));
    ctx.beginPath();
    ctx.moveTo(x+r,y);
    ctx.arcTo(x+w,y,x+w,y+h,r);
    ctx.arcTo(x+w,y+h,x,y+h,r);
    ctx.arcTo(x,y+h,x,y,r);
    ctx.arcTo(x,y,x+w,y,r);
    ctx.closePath();
  }
  function makeFillGradient(color, y0, y1){
    const g=ctx.createLinearGradient(0,y0,0,y1);
    g.addColorStop(0, toRGBA(color,0.22));
    g.addColorStop(1, toRGBA(color,0.00));
    return g;
  }
  function drawPolyline(points){
    ctx.beginPath();
    for(let i=0;i<points.length;i++){
      const p=points[i];
      if(i===0) ctx.moveTo(p[0],p[1]);
      else ctx.lineTo(p[0],p[1]);
    }
    ctx.stroke();
  }
  function drawSpline(points){
    if(points.length<2){ return; }
    ctx.beginPath();
    ctx.moveTo(points[0][0], points[0][1]);
    const t = 0.85;
    for(let i=0;i<points.length-1;i++){
      const p0 = points[Math.max(0,i-1)];
      const p1 = points[i];
      const p2 = points[i+1];
      const p3 = points[Math.min(points.length-1,i+2)];
      const c1x = p1[0] + (p2[0]-p0[0])*(t/6);
      const c1y = p1[1] + (p2[1]-p0[1])*(t/6);
      const c2x = p2[0] - (p3[0]-p1[0])*(t/6);
      const c2y = p2[1] - (p3[1]-p1[1])*(t/6);
      ctx.bezierCurveTo(c1x,c1y,c2x,c2y,p2[0],p2[1]);
    }
    ctx.stroke();
  }

  function line(arr, color, xOf, yOf, isGap, i0,i1){
    ctx.strokeStyle=color;
    ctx.lineWidth=2.4;
    ctx.lineJoin="round";
    ctx.lineCap="round";
    ctx.shadowColor = toRGBA(color, 0.22);
    ctx.shadowBlur  = 8;

    let seg=[];
    const useSpline = !!smoothEl.checked;
    const strokeSegment=()=>{
      if(seg.length>1) { useSpline ? drawSpline(seg) : drawPolyline(seg); }
      else if(seg.length===1) { ctx.fillStyle=color; ctx.beginPath(); ctx.arc(seg[0][0],seg[0][1],3,0,Math.PI*2); ctx.fill(); }
    };

    for(let i=i0;i<=i1;i++){
      const v=arr[i];
      if(isGap(v)){
        strokeSegment();
        seg=[];
        continue;
      }
      seg.push([xOf(i), yOf(v)]);
    }
    strokeSegment();

    ctx.shadowBlur=0;
  }

  function areaFill(arr, color, xOf, yOf, isGap, i0,i1, baseY){
    ctx.fillStyle = makeFillGradient(color, baseY-80, baseY);

    let seg=[];
    const useSpline = !!smoothEl.checked;

    function fillSeg(points){
      if(points.length<2) return;

      ctx.beginPath();
      ctx.moveTo(points[0][0], baseY);
      ctx.lineTo(points[0][0], points[0][1]);

      if(useSpline){
        const t=0.85;
        for(let i=0;i<points.length-1;i++){
          const p0 = points[Math.max(0,i-1)];
          const p1 = points[i];
          const p2 = points[i+1];
          const p3 = points[Math.min(points.length-1,i+2)];
          const c1x = p1[0] + (p2[0]-p0[0])*(t/6);
          const c1y = p1[1] + (p2[1]-p0[1])*(t/6);
          const c2x = p2[0] - (p3[0]-p1[0])*(t/6);
          const c2y = p2[1] - (p3[1]-p1[1])*(t/6);
          ctx.bezierCurveTo(c1x,c1y,c2x,c2y,p2[0],p2[1]);
        }
      }else{
        for(let i=1;i<points.length;i++) ctx.lineTo(points[i][0], points[i][1]);
      }

      ctx.lineTo(points[points.length-1][0], baseY);
      ctx.closePath();
      ctx.fill();
    }

    for(let i=i0;i<=i1;i++){
      const v=arr[i];
      if(isGap(v)){ fillSeg(seg); seg=[]; continue; }
      seg.push([xOf(i), yOf(v)]);
    }
    fillSeg(seg);
  }

  function bars(arr, color, xOf, yOf, isGap, i0,i1, baseY, gxW){
    const len=(i1-i0+1);
    const bw=Math.max(2,(gxW/Math.max(1,len))*0.70);
    const r=Math.min(6, bw*0.45);

    ctx.fillStyle=toRGBA(color,0.92);
    ctx.strokeStyle=toRGBA(color,0.28);
    ctx.lineWidth=1;

    for(let i=i0;i<=i1;i++){
      const v=arr[i]; if(isGap(v)) continue;
      const cx=xOf(i);
      const x=cx-bw/2;
      const y=yOf(v);
      const h=baseY-y;

      roundRectPath(x, y, bw, h, r);
      ctx.fill();
      ctx.stroke();
    }
  }

  function draw(){
    const W=cssW,H=cssH;
    ctx.clearRect(0,0,W,H);

    const cs=getComputedStyle(document.documentElement);
    const panel=cs.getPropertyValue("--panel").trim()||"#10172c";
    const grid =cs.getPropertyValue("--grid").trim()||"rgba(255,255,255,.10)";
    const stroke=cs.getPropertyValue("--stroke").trim()||"rgba(255,255,255,.12)";
    const muted=cs.getPropertyValue("--muted").trim()||"rgba(234,240,255,.72)";
    const accent=cs.getPropertyValue("--accent").trim()||"#fad24f";

    ctx.fillStyle=panel; ctx.fillRect(0,0,W,H);

    if(!data || !Array.isArray(data.co2) || data.co2.length<2){
      ctx.fillStyle=muted; ctx.font="14px system-ui";
      ctx.fillText("No data for selected day", 16, 26);
      return;
    }

    const CO2="#ff785a", TT="#50d2ff", HH="#8cb4ff";
    const N=data.co2.length;
    const i0=Math.max(0,Math.min(viewMin,viewMax));
    const i1=Math.min(N-1,Math.max(viewMin,viewMax));
    const len=i1-i0+1;
    const step=data.stepMin||5;

    const padL=62,padR=24,padT=18,padB=34;
    const gx0=padL,gx1=W-padR,gy0=padT,gy1=H-padB;
    const gxW=(gx1-gx0);
    const xOf=(i)=>gx0 + ((i-i0)/Math.max(1,(len-1))) * gxW;

    let cmin=1e9,cmax=-1e9,tmin=1e9,tmax=-1e9,hmin=1e9,hmax=-1e9;
    for(let i=i0;i<=i1;i++){
      const c=data.co2[i]||0, t=data.temp[i], h=data.hum[i];
      if(c>0){ cmin=Math.min(cmin,c); cmax=Math.max(cmax,c); }
      if(t!=null){ tmin=Math.min(tmin,t); tmax=Math.max(tmax,t); }
      if(h!=null){ hmin=Math.min(hmin,h); hmax=Math.max(hmax,h); }
    }
    if(!(cmax>cmin)){ cmin=300; cmax=2000; }
    if(!(tmax>tmin)){ tmin=10; tmax=40; }
    if(!(hmax>hmin)){ hmin=20; hmax=80; }

    [cmin,cmax]=pad(cmin,cmax,0.08);
    [tmin,tmax]=pad(tmin,tmax,0.10);
    [hmin,hmax]=pad(hmin,hmax,0.10);

    const yC=(v)=>yMap(v,cmin,cmax,gy0,gy1);
    const yT=(v)=>yMap(v,tmin,tmax,gy0,gy1);
    const yH=(v)=>yMap(v,hmin,hmax,gy0,gy1);

    const co2=smooth(data.co2, v=>!v);
    const tt =smooth(data.temp, v=>v==null);
    const hh =smooth(data.hum,  v=>v==null);

    ctx.fillStyle=muted; ctx.font="11px ui-monospace,Menlo,Consolas,monospace";
    ctx.textAlign="left";  ctx.fillText("CO₂ ppm", 6, gy0-2);

    ctx.save();
    roundRectPath(gx0, gy0, (gx1-gx0), (gy1-gy0), 12);
    ctx.clip();

    ctx.strokeStyle=grid; ctx.lineWidth=1;
    for(let k=0;k<5;k++){
      const y=gy0 + (gy1-gy0)*(k/4);
      ctx.setLineDash([]);
      ctx.beginPath(); ctx.moveTo(gx0,y); ctx.lineTo(gx1,y); ctx.stroke();
      if(k<4){
        const y2=gy0 + (gy1-gy0)*((k+0.5)/4);
        ctx.setLineDash([4,6]);
        ctx.beginPath(); ctx.moveTo(gx0,y2); ctx.lineTo(gx1,y2); ctx.stroke();
      }
    }
    ctx.setLineDash([]);

    const tickLabels=[];
    const startMin=i0*step, endMin=i1*step, span=Math.max(1,endMin-startMin);
    const steps=[15,30,60,120,180,240,360,720];
    let tick=1440; for(const s of steps){ if(span/s<=8){ tick=s; break; } }
    const first=Math.ceil(startMin/tick)*tick;
    for(let m=first;m<=endMin;m+=tick){
      const idx=Math.round(m/step);
      if(idx<i0||idx>i1) continue;
      const x=xOf(idx);
      ctx.setLineDash([3,7]);
      ctx.beginPath(); ctx.moveTo(x,gy0); ctx.lineTo(x,gy1); ctx.stroke();
      ctx.setLineDash([]);
      tickLabels.push([x, fmtTime(idx,step)]);
    }

    const mode=modeEl.value||"line";
    const showC=!!cOn.checked, showT=!!tOn.checked, showH=!!hOn.checked;

    if(showT){ areaFill(tt, TT, xOf, yT, v=>v==null, i0,i1, gy1); line(tt, TT, xOf, yT, v=>v==null, i0,i1); }
    if(showH){ areaFill(hh, HH, xOf, yH, v=>v==null, i0,i1, gy1); line(hh, HH, xOf, yH, v=>v==null, i0,i1); }

    if(showC){
      if(mode==="bar") bars(co2, CO2, xOf, yC, v=>!v, i0,i1, gy1, gxW);
      else if(mode==="area"){ areaFill(co2, CO2, xOf, yC, v=>!v, i0,i1, gy1); line(co2, CO2, xOf, yC, v=>!v, i0,i1); }
      else line(co2, CO2, xOf, yC, v=>!v, i0,i1);
    }

    if(hover!=null && hover>=i0 && hover<=i1){
      const x=xOf(hover);
      ctx.strokeStyle=toRGBA(accent,0.95); ctx.lineWidth=1.6;
      ctx.beginPath(); ctx.moveTo(x,gy0); ctx.lineTo(x,gy1); ctx.stroke();

      function dot(v, yOf, col, ok){
        if(!ok) return;
        ctx.fillStyle=col;
        ctx.beginPath(); ctx.arc(x,yOf(v),3.4,0,Math.PI*2); ctx.fill();
      }
      const c=co2[hover], t=tt[hover], h=hh[hover];
      if(showC && c) dot(c,yC,CO2,true);
      if(showT && t!=null) dot(t,yT,TT,true);
      if(showH && h!=null) dot(h,yH,HH,true);
    }

    ctx.restore();

    ctx.strokeStyle=stroke; ctx.lineWidth=1;
    roundRectPath(gx0, gy0, (gx1-gx0), (gy1-gy0), 12);
    ctx.stroke();

    ctx.fillStyle=muted;
    ctx.font="11px ui-monospace,Menlo,Consolas,monospace";
    ctx.textAlign="right";
    for(let k=0;k<5;k++){
      const y=gy0 + (gy1-gy0)*(k/4);
      const v = cmax - (cmax-cmin)*(k/4);
      ctx.fillText(String(Math.round(v)), gx0-8, y+4);
    }

    ctx.fillStyle=muted; ctx.textAlign="center";
    for(const it of tickLabels) ctx.fillText(it[1], it[0], gy1+20);
  }

  function showTip(txt){
    tip.textContent=txt;
    tip.style.display="block";
    tip.style.left=(lastPX+12)+"px";
    tip.style.top =(lastPY+12)+"px";
  }
  function hideTip(){ tip.style.display="none"; }

  function updateTip(){
    if(!data || hover==null){ hideTip(); return; }
    const step=data.stepMin||5;
    const parts=[fmtTime(hover,step)];
    const c=data.co2[hover]||0, t=data.temp[hover], h=data.hum[hover];
    if(cOn.checked && c) parts.push(`CO₂ ${Math.round(c)} ppm`);
    if(tOn.checked && t!=null) parts.push(`T ${Number(t).toFixed(1)} °C`);
    if(hOn.checked && h!=null) parts.push(`H ${Math.round(h)} %`);
    showTip(parts.join(" · "));
  }

  cv.addEventListener("pointermove",(ev)=>{
    if(ev.pointerType==="touch") return;
    lastPX=ev.clientX; lastPY=ev.clientY;
    if(!data||!Array.isArray(data.co2)) return;
    if(locked){ if(hover!=null) updateTip(); return; }

    const r=cv.getBoundingClientRect();
    const x=ev.clientX-r.left;
    const padL=62,padR=24;
    const gx0=padL,gx1=cssW-padR;

    if(x<gx0||x>gx1){ hover=null; hideTip(); draw(); return; }

    const N=data.co2.length;
    const i0=Math.max(0,Math.min(viewMin,viewMax));
    const i1=Math.min(N-1,Math.max(viewMin,viewMax));
    const len=i1-i0+1;

    const t=(x-gx0)/Math.max(1,(gx1-gx0));
    let idx=i0+Math.round(t*(len-1));
    idx=Math.max(i0,Math.min(i1,idx));
    hover=idx;

    updateTip();
    draw();
  });

  cv.addEventListener("pointerleave",()=>{ if(!locked){ hover=null; hideTip(); draw(); }});
  cv.addEventListener("click",()=>{ locked=!locked; if(!locked&&hover==null) hideTip(); else updateTip(); });

  cv.addEventListener("wheel",(ev)=>{
    if(!data||!Array.isArray(data.co2)) return;
    ev.preventDefault();
    const N=data.co2.length;
    const i0=Math.max(0,Math.min(viewMin,viewMax));
    const i1=Math.min(N-1,Math.max(viewMin,viewMax));
    const len=i1-i0+1; if(len<2) return;

    const r=cv.getBoundingClientRect();
    const x=ev.clientX-r.left;
    const padL=62,padR=24;
    const gx0=padL,gx1=cssW-padR;

    const t=(x-gx0)/Math.max(1,(gx1-gx0));
    const center=i0+Math.round(Math.max(0,Math.min(1,t))*(len-1));

    const zoomIn=(ev.deltaY<0);
    const factor=zoomIn?1.35:(1/1.35);

    let newLen=Math.max(12,Math.round(len/factor));
    newLen=Math.min(N,newLen);

    let nm=Math.round(center-newLen/2);
    let nx=nm+newLen-1;
    if(nm<0){ nm=0; nx=newLen-1; }
    if(nx>N-1){ nx=N-1; nm=N-newLen; }

    viewMin=nm; viewMax=nx;
    draw();
  },{passive:false});

  const touchState={
    active:new Map(),
    panStartX:0, panMin:0, panMax:0,
    pinchStartDist:0, pinchStartLen:0, pinchCenterIdx:0,
    lastTapT:0
  };
  function clamp(v,a,b){ return Math.max(a,Math.min(b,v)); }
  function geom(){
    const padL=62, padR=24;
    const gx0=padL, gx1=cssW-padR;
    const N=(data&&data.co2)?data.co2.length:0;
    const i0=Math.max(0,Math.min(viewMin,viewMax));
    const i1=Math.min(N-1,Math.max(viewMin,viewMax));
    const len=(i1-i0+1)||1;
    return {gx0,gx1,N,i0,i1,len};
  }
  function pxToIdx(x){
    const g=geom();
    const t=(x-g.gx0)/Math.max(1,(g.gx1-g.gx0));
    const idx=g.i0 + Math.round(clamp(t,0,1)*(g.len-1));
    return clamp(idx, g.i0, g.i1);
  }
  function setViewCentered(center, newLen){
    const g=geom();
    if(!g.N) return;
    newLen=clamp(newLen, 12, g.N);
    let nm=Math.round(center - newLen/2);
    let nx=nm + newLen - 1;
    if(nm<0){ nm=0; nx=newLen-1; }
    if(nx>g.N-1){ nx=g.N-1; nm=g.N-newLen; }
    viewMin=nm; viewMax=nx;
    draw();
  }
  function setMobileHint(){
    const h=$("hint"); if(!h) return;
    h.textContent="Drag: pan · Pinch: zoom · Tap: tooltip · Double-tap: reset";
  }
  cv.addEventListener("pointerdown",(ev)=>{
    if(ev.pointerType!=="touch") return;
    setMobileHint();
  },{passive:true});

  cv.addEventListener("pointerdown",(ev)=>{
    if(ev.pointerType!=="touch") return;
    if(!data||!Array.isArray(data.co2)) return;

    cv.setPointerCapture(ev.pointerId);
    touchState.active.set(ev.pointerId,{x:ev.clientX,y:ev.clientY});

    const now=Date.now();
    const dt=now-touchState.lastTapT;
    touchState.lastTapT=now;

    if(dt>0 && dt<320){
      const g=geom();
      viewMin=0; viewMax=g.N-1;
      locked=false; hover=null; hideTip();
      draw();
      return;
    }

    touchState.panStartX=ev.clientX;
    touchState.panMin=viewMin;
    touchState.panMax=viewMax;

    if(touchState.active.size===2){
      const pts=[...touchState.active.values()];
      const dx=pts[0].x-pts[1].x, dy=pts[0].y-pts[1].y;
      touchState.pinchStartDist=Math.hypot(dx,dy);
      const g=geom();
      touchState.pinchStartLen=g.len;

      const midX=(pts[0].x+pts[1].x)/2;
      const r=cv.getBoundingClientRect();
      touchState.pinchCenterIdx=pxToIdx(midX - r.left);
    }
  });

  cv.addEventListener("pointermove",(ev)=>{
    if(ev.pointerType!=="touch") return;
    if(!data||!Array.isArray(data.co2)) return;
    if(!touchState.active.has(ev.pointerId)) return;

    touchState.active.set(ev.pointerId,{x:ev.clientX,y:ev.clientY});

    if(touchState.active.size===2){
      const pts=[...touchState.active.values()];
      const dx=pts[0].x-pts[1].x, dy=pts[0].y-pts[1].y;
      const dist=Math.hypot(dx,dy);
      const start=touchState.pinchStartDist||dist;
      const scale=dist/Math.max(1,start);
      const newLen=Math.round(touchState.pinchStartLen / scale);
      setViewCentered(touchState.pinchCenterIdx, newLen);
      locked=false; hover=null; hideTip();
      return;
    }

    const g=geom();
    const dx=ev.clientX - touchState.panStartX;
    const spanPx=Math.max(1,(g.gx1-g.gx0));
    const shift=Math.round(-dx/spanPx * Math.max(1,(g.len-1)));

    let nm=touchState.panMin + shift;
    let nx=touchState.panMax + shift;
    const win=(nx-nm);

    if(nm<0){ nm=0; nx=win; }
    if(nx>g.N-1){ nx=g.N-1; nm=nx-win; }
    viewMin=clamp(nm,0,g.N-1);
    viewMax=clamp(nx,0,g.N-1);

    locked=false; hover=null; hideTip();
    draw();
  });

  cv.addEventListener("pointerup",(ev)=>{
    if(ev.pointerType!=="touch") return;
    touchState.active.delete(ev.pointerId);

    if(touchState.active.size===0 && data && Array.isArray(data.co2)){
      const r=cv.getBoundingClientRect();
      const x=ev.clientX - r.left;
      const g=geom();
      if(x>=g.gx0 && x<=g.gx1){
        hover=pxToIdx(x);
        locked=true;
        lastPX=ev.clientX; lastPY=ev.clientY;
        updateTip();
        draw();
      }
    }

    if(touchState.active.size<2){
      touchState.pinchStartDist=0;
      touchState.pinchStartLen=0;
    }
  });

  cv.addEventListener("pointercancel",(ev)=>{
    if(ev.pointerType!=="touch") return;
    touchState.active.clear();
    touchState.pinchStartDist=0;
    touchState.pinchStartLen=0;
  });

  $("btnPNG").onclick=()=>{
    const url=cv.toDataURL("image/png");
    const a=document.createElement("a");
    a.download="env_graph.png"; a.href=url; a.click();
  };

  async function pollDevice(){
    if(!document.hidden){
      await fetchStatus();
      if(sel.value===TODAY && Date.now()-lastGraphFetch>=15000) await load(TODAY,false);
    }
    setTimeout(pollDevice,5000);
  }
  restorePrefs();
  requestAnimationFrame(()=>{ resize(); fetchDayList(); });
  setTimeout(pollDevice,500);

})();
</script></body></html>
)HTML";


// ---------- Web handler ----------
static void handleWebRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", WEB_ROOT_HTML);
}

static void handleStatus() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");

  String out;
  out.reserve(220);

  out += "{";
  out += "\"online\":";
  out += (WiFi.status() == WL_CONNECTED ? "true" : "false");
  out += ",\"offlineMode\":";
  out += (offlineMode ? "true" : "false");
  out += ",\"deviceEpoch\":";
  out += String((unsigned long)approxNow());

  out += ",\"co2\":";
  out += (isnan(currentCO2)  ? "null" : String(currentCO2, 1));
  out += ",\"temp\":";
  out += (isnan(currentTemp) ? "null" : String(currentTemp, 1));
  out += ",\"hum\":";
  out += (isnan(currentHum)  ? "null" : String(currentHum, 0));

  out += ",\"sensorType\":\"";
  out += USE_DHT_S8 ? "DHT11 + Senseair S8" : "SCD30";
  out += "\",\"tempHumFresh\":"; out += (hasTempHum && millis()-lastTempHumData<=SENSOR_TIMEOUT)?"true":"false";
  out += ",\"co2Fresh\":"; out += (hasCo2 && millis()-lastCo2Data<=SENSOR_TIMEOUT)?"true":"false";
#if USE_DHT_S8
  out += ",\"co2Status\":" + jsonString(s8Status);
#endif
  out += ",\"sensorReady\":"; out += sensorInitialized ? "true" : "false";
  out += ",\"sampleAgeMs\":"; out += sensorHasSample ? String(millis() - lastSensorData) : "null";
  out += ",\"sensorStale\":";
  out += (!sensorHasSample || millis() - lastSensorData > SENSOR_TIMEOUT) ? "true" : "false";
#if !USE_DHT_S8
  out += ",\"i2cDevices\":" + jsonString(i2cDevices);
  out += ",\"i2cDeviceCount\":" + String(i2cDeviceCount);
  out += ",\"i2cAddress61Present\":"; out += i2cAddress61Present ? "true" : "false";
#endif

  out += "}";

  server.send(200, "application/json", out);
}

class JsonChunks {
  char buffer[768];
  size_t used = 0;
public:
  void flush() {
    if (!used) return;
    server.sendContent(buffer, used);
    used = 0;
    yield();
  }
  void append(const char* text) {
    while (*text) {
      buffer[used++] = *text++;
      if (used == sizeof(buffer)) flush();
    }
  }
  void finish() { flush(); server.sendContent(""); }
};

static void handleToday() {
  writeLiveSlotPreviewToBuffers();

  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  JsonChunks chunks;
  auto sendS = [&](const String& s) { chunks.append(s.c_str()); };
  auto sendC = [&](const char* s)   { chunks.append(s); };

  const int slotMinutes = SLOT_MINUTES;
  const unsigned long nowEpoch = (unsigned long)approxNow();
  const int slot = currentSlot;
  const int minuteOfDay = (slot < 0) ? 0 : slot * slotMinutes;

  sendC("{");
  sendC("\"name\":\"__TODAY__\"");
  sendC(",\"stepMin\":");       sendS(String(slotMinutes));
  sendC(",\"slots\":");         sendS(String(SLOTS_PER_DAY));
  sendC(",\"epoch\":");         sendS(String(nowEpoch));
  sendC(",\"slot\":");          sendS(String(slot));
  sendC(",\"minuteOfDay\":");   sendS(String(minuteOfDay));

  sendC(",\"co2\":[");
  for (int i = 0; i < SLOTS_PER_DAY; i++) {
    if (i) sendC(",");
    sendS(String(co2Slots[i]));
  }
  sendC("]");

  sendC(",\"temp\":[");
  for (int i = 0; i < SLOTS_PER_DAY; i++) {
    if (i) sendC(",");
    if (!tempSlots[i]) sendC("null");
    else sendS(String(decodeTemp(tempSlots[i]), 1));
  }
  sendC("]");

  sendC(",\"hum\":[");
  for (int i = 0; i < SLOTS_PER_DAY; i++) {
    if (i) sendC(",");
    if (!humSlots[i]) sendC("null");
    else sendS(String(decodeHum(humSlots[i]), 0));
  }
  sendC("]");

  sendC("}");

  chunks.finish();
}

static void handleDayList() {
#if !ENABLE_FS
  server.send(200, "application/json", "[]");
#else
  if (!fsReady) { server.send(200, "application/json", "[]"); return; }

  const int MAX_FILES = 40;
  char names[MAX_FILES][24];
  int count = 0;

  Dir dir = LittleFS.openDir("/");
  while (dir.next()) {
    String fn = dir.fileName();
    char base[24];
    if (!normalizeDayName(fn, base, sizeof(base))) continue;
    if (count < MAX_FILES) {
      strncpy(names[count], base, sizeof(names[count]) - 1);
      names[count][sizeof(names[count]) - 1] = '\0';
      count++;
    }
  }
  if (count == 0) { server.send(200, "application/json", "[]"); return; }

  for (int i = 0; i < count - 1; ++i) {
    for (int j = i + 1; j < count; ++j) {
      if (strcmp(names[i], names[j]) > 0) {
        char tmp[24];
        strcpy(tmp, names[i]); strcpy(names[i], names[j]); strcpy(names[j], tmp);
      }
    }
  }

  String out = "[";
  for (int i = 0; i < count; ++i) {
    if (i) out += ',';
    char* name = names[i];
    char yearStr[5]; strncpy(yearStr, name+4, 4); yearStr[4]='\0';
    char doyStr[4];  strncpy(doyStr,  name+8, 3); doyStr[3]='\0';
    out += "{\"name\":\""; out += name;
    out += "\",\"label\":\""; out += yearStr; out += "-"; out += doyStr; out += "\"}";
  }
  out += "]";
  server.send(200, "application/json", out);
#endif
}

static void handleDayData() {
#if !ENABLE_FS
  server.send(200, "application/json", "{}");
  return;
#else
  if (!fsReady) { server.send(200, "application/json", "{}"); return; }
  if (!server.hasArg("name")) { server.send(400, "text/plain", "Missing name"); return; }

  String name = server.arg("name");
  if (name.indexOf('/') >= 0 || name.indexOf('\\') >= 0 || name.indexOf("..") >= 0) {
    server.send(400, "text/plain", "Bad name"); return;
  }
  if (!name.startsWith("day_") || !name.endsWith(".bin")) { server.send(400, "text/plain", "Bad name"); return; }

  String path = "/" + name;
  if (!LittleFS.exists(path)) { server.send(404, "text/plain", "Not found"); return; }

  File f = LittleFS.open(path, "r");
  if (!f) { server.send(500, "text/plain", "Open failed"); return; }

  const size_t needSize =
    sizeof(StorageHeader) +
    (sizeof(uint16_t) * SLOTS_PER_DAY) +
    (sizeof(uint8_t)  * SLOTS_PER_DAY) +
    (sizeof(uint8_t)  * SLOTS_PER_DAY);

  if ((size_t)f.size() < needSize) { f.close(); server.send(500, "text/plain", "File too small"); return; }

  StorageHeader h{};
  if (f.readBytes((char*)&h, sizeof(h)) != sizeof(h) || h.magic != 0xDA7ADA7A) {
    f.close(); server.send(500, "text/plain", "Bad header"); return;
  }

  static uint16_t co2Buf[SLOTS_PER_DAY];
  static uint8_t  tBuf [SLOTS_PER_DAY];
  static uint8_t  hBuf [SLOTS_PER_DAY];

  if (f.readBytes((char*)co2Buf, sizeof(co2Buf)) != sizeof(co2Buf) ||
      f.readBytes((char*)tBuf,   sizeof(tBuf))   != sizeof(tBuf)   ||
      f.readBytes((char*)hBuf,   sizeof(hBuf))   != sizeof(hBuf)) {
    f.close(); server.send(500, "text/plain", "Read failed"); return;
  }
  f.close();

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  JsonChunks chunks;
  auto sendS = [&](const String& s) { chunks.append(s.c_str()); };
  auto sendC = [&](const char* s)   { chunks.append(s); };

  const int slotMinutes = SLOT_MINUTES;

  sendC("{\"name\":\""); sendS(name);
  sendC("\",\"stepMin\":"); sendS(String(slotMinutes));
  sendC(",\"slots\":"); sendS(String(SLOTS_PER_DAY));

  sendC(",\"co2\":[");
  for (int i = 0; i < SLOTS_PER_DAY; i++) { if (i) sendC(","); sendS(String(co2Buf[i])); }
  sendC("]");

  sendC(",\"temp\":[");
  for (int i = 0; i < SLOTS_PER_DAY; i++) {
    if (i) sendC(",");
    if (!tBuf[i]) sendC("null");
    else sendS(String(decodeTemp(tBuf[i]), 1));
  }
  sendC("]");

  sendC(",\"hum\":[");
  for (int i = 0; i < SLOTS_PER_DAY; i++) {
    if (i) sendC(",");
    if (!hBuf[i]) sendC("null");
    else sendS(String(decodeHum(hBuf[i]), 0));
  }
  sendC("]}");
  chunks.finish();
#endif
}

// ===================== setup / loop =====================
void setup(){
  Serial.begin(115200);
  bootTime = millis();
  fanBoot=os_random();

  // Backlight
  pinMode(BL_PIN, OUTPUT);
  digitalWrite(BL_PIN, LOW);
  analogWriteFreq(BL_PWM_HZ);
  analogWriteRange(BL_RANGE);
  applyScheduledBacklight();

  // Data
  clearData();

#if ENABLE_FS
  fsReady = LittleFS.begin();
  if (!fsReady) {
    Serial.println("LittleFS mount failed (NOT formatting).");
  } else {
    loadTimeAnchor();
    loadManualClock();
    loadDataFromFS();
  }
#endif

  // Sensor
  loadScreenSettings();
  loadFanSettings();
  initDisplay();
  tft.fillScreen(COL_BG);
  applyScheduledBacklight();
  initSensor();

  // WiFi + Time
  connectWiFi();
  initTime();
  if (isTimeValid()) anchorTimeToNow();

  // Web routes
  server.on("/", HTTP_GET, handleWebRoot);
  server.on("/api/daylist", HTTP_GET, handleDayList);
  server.on("/api/daydata", HTTP_GET, handleDayData);
  server.on("/api/today",   HTTP_GET, handleToday);
  server.on("/status",      HTTP_GET, handleStatus);
  server.on("/api/settings", HTTP_GET, handleSettings);
  server.on("/api/time", HTTP_POST, handleTimeSave);
  server.on("/api/screen", HTTP_POST, handleScreenSave);
  server.on("/api/wifi", HTTP_POST, handleWifiSave);
  server.on("/api/fan", HTTP_POST, handleFanSave);
  server.begin();

#if !USE_DHT_S8
  button.begin(BUTTON_PIN, onBtnShort, onBtnLongRequest, onBtnConfirm, onBtnConfirmTimeout, 5000, 5000);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
#endif

  // Initial render
  drawAllScreens();
}

void loop() {
  const uint32_t nowMs = millis();

  static uint32_t lastScheduleCheck=0;
  if(nowMs-lastScheduleCheck>=1000) { lastScheduleCheck=nowMs; applyScheduledBacklight(); }

  if (nowMs - _blLastReapply > 60000UL) {
    analogWriteFreq(BL_PWM_HZ);
    analogWriteRange(BL_RANGE);
    applyScheduledBacklight();
    _blLastReapply = nowMs;
  }

  // Network tasks
  if (WiFi.getMode() != WIFI_OFF) {
    server.handleClient();
    applyPendingWifi();
    if (apFallbackActive) {
      dnsServer.processNextRequest();
      retryStaWhileAp();
    }
    if (mdnsStarted && WiFi.status() == WL_CONNECTED) {
      MDNS.update();
    }
  }

  // Button
#if USE_DHT_S8
  static bool buttonPressed=false;
  static uint32_t buttonPoll=0, buttonDown=0;
  if(nowMs-buttonPoll>=25) {
    buttonPoll=nowMs;
    int raw=analogRead(A0);
    if(!buttonPressed && raw<200) { buttonPressed=true; buttonDown=nowMs; }
    if(buttonPressed && raw>800) {
      buttonPressed=false;
      if(nowMs-buttonDown>=50) nextScreenReq=true;
    }
  }
#else
  button.tick();
#endif

  if (nextScreenReq) {
    nextScreenReq = false;
    screenMode = (screenMode + 1) % NUM_SCREENS;
    lastFast30mDraw = 0;
    lastRedraw      = 0;
    drawAllScreens();
  }

  if (isTimeValid() && nowMs >= nextAnchorSave) {
    anchorTimeToNow();
  }

  // Slot handling
  time_t ts = approxNow();
  int currentMinute = 0;

  if (ts > 0) {
    struct tm ti;
    localtime_r(&ts, &ti);
    currentMinute = ti.tm_hour * 60 + ti.tm_min;
  } else {
    if (softStartTime == 0) softStartTime = nowMs;
    currentMinute = ((nowMs - softStartTime) / 60000UL) % 1440;
  }

  const int slot = currentMinute / SLOT_MINUTES;

  if (currentSlot == -1) {
    currentSlot   = slot;
    lastDrawnSlot = slot;
  }

  if (slot != currentSlot) {
    const bool crossedMidnight = (lastDrawnSlot > slot && lastDrawnSlot >= 0);

    commitSlotData(currentSlot);

#if ENABLE_FS
    if (fsReady) saveDataToFS(crossedMidnight);
#endif

    currentSlot = slot;

    if (crossedMidnight) {
      clearData();

      slotsSince12hReset = 0;
      slotsSince24hReset = 0;
      twelveGraphResetOriginSlot     = -1;
      twentyFourGraphResetOriginSlot = -1;
    } else {
      slotsSince12hReset++;
      slotsSince24hReset++;

      if (slotsSince12hReset >= SLOTS_PER_12H) {
        slotsSince12hReset = 0;
        twelveGraphResetOriginSlot = currentSlot;
      }
      if (slotsSince24hReset >= SLOTS_PER_24H) {
        slotsSince24hReset = 0;
        twentyFourGraphResetOriginSlot = currentSlot;
      }
    }

    resetAccumulators();
    lastDrawnSlot = slot;
    drawAllScreens();
  }

#if USE_DHT_S8
  bool gotSample=false;
  if(s8Pending) {
    while(s8Serial.available() && s8ResponseLength<sizeof(s8Response))
      s8Response[s8ResponseLength++]=s8Serial.read();
    if(s8ResponseLength==sizeof(s8Response)) {
      uint16_t crc=0xFFFF;
      for(uint8_t i=0;i<5;i++) {
        crc^=s8Response[i];
        for(uint8_t bit=0;bit<8;bit++) crc=(crc&1)?(crc>>1)^0xA001:crc>>1;
      }
      const int ppm=(s8Response[3]<<8)|s8Response[4];
      if(s8Response[0]==0xFE && s8Response[1]==0x04 && s8Response[2]==2 &&
         s8Response[5]==(crc&255) && s8Response[6]==(crc>>8) && ppm>0 && ppm<10000) {
        s8Status="OK";
        currentCO2=ppm; co2Sum+=ppm; co2SampleCount++;
        hasCo2=true; lastCo2Data=millis(); gotSample=true;
      } else s8Status="Invalid S8 reply (check UART wiring and power)";
      s8Pending=false;
    } else if(nowMs-lastS8Request>=500) {
      s8Status=s8ResponseLength ? "Incomplete S8 reply" : "No S8 reply";
      s8Pending=false;
    }
    if(!s8Pending && s8Status[0]!='O' && nowMs-lastS8ErrorLog>=10000) {
      lastS8ErrorLog=nowMs;
    }
  }
  if(!s8Pending && nowMs-lastDhtPoll>=2500) {
    lastDhtPoll=nowMs;
    const float t=dht.readTemperature(), h=dht.readHumidity();
    if(isfinite(t) && isfinite(h) && t>=0 && t<=50 && h>=0 && h<=100) {
      currentTemp=t; currentHum=h;
      tempSum+=(int32_t)lroundf(t*10); humSum+=(uint32_t)lroundf(h*10); sampleCount++;
      hasTempHum=true; lastTempHumData=millis(); gotSample=true;
    }
  }
  if(!s8Pending && nowMs-lastS8Request>=2000) {
    static const uint8_t request[]={0xFE,0x04,0x00,0x03,0x00,0x01,0xD5,0xC5};
    while(s8Serial.available()) s8Serial.read();
    s8Serial.write(request,sizeof(request));
    s8ResponseLength=0; s8Pending=true; lastS8Request=millis();
  }
  if(gotSample) {
    sensorHasSample=true; lastSensorData=millis();
    writeLiveSlotPreviewToBuffers();
  }
  static uint32_t lastSensorLog=0;
  if(nowMs-lastSensorLog>=5000) {
    lastSensorLog=nowMs;
    Serial.printf("Sensor: temp=%s hum=%s co2=%s S8=%s\n",
      hasTempHum ? String(currentTemp,1).c_str() : "unavailable",
      hasTempHum ? String(currentHum,0).c_str() : "unavailable",
      hasCo2 ? String(currentCO2,0).c_str() : "unavailable",
      s8Status);
  }
#else
  if ((!sensorInitialized || !sensorHasSample || nowMs - lastSensorData > SENSOR_TIMEOUT) &&
      nowMs - lastSensorAttempt >= SENSOR_TIMEOUT) {
    lastSensorAttempt = nowMs;
    if (!sensorInitialized) initSensor();
    else scd30.startContinuousMeasurement();
  }

  bool gotSample = false;
  if (sensorInitialized && nowMs - lastSensorPoll >= 500) {
    lastSensorPoll = nowMs;
    if (scd30.dataReady() && scd30.read()) {
      const float c = scd30.CO2, t = scd30.temperature, h = scd30.relative_humidity;
      if (isfinite(c) && isfinite(t) && isfinite(h) &&
          c > 0 && c < 10000 && t > -20 && t < 80 && h >= 0 && h <= 100) {
        currentCO2 = c; currentTemp = t; currentHum = h;
        lastSensorData = millis(); sensorHasSample = true; gotSample = true;
      }
    }
  }

  if (gotSample) {
    co2Sum  += (uint32_t)(currentCO2 + 0.5f);
    tempSum += (int32_t)lroundf(currentTemp * 10.0f);
    humSum  += (uint32_t)(currentHum  * 10.0f + 0.5f);
    sampleCount++; co2SampleCount++;
    hasTempHum=hasCo2=true; lastTempHumData=lastCo2Data=lastSensorData;

    writeLiveSlotPreviewToBuffers();
  }
#endif

  sendFanReadings();
  sendAcTemperature();

#if ENABLE_FS
  if (fsReady && (nowMs - lastPeriodicSaveMs >= PERIODIC_SAVE_MS)) {
    saveDataToFS(false);
    lastPeriodicSaveMs = nowMs;
  }
#endif

  // Display refresh
  const uint32_t displayNow = millis();
  if (screenMode == 0 && (displayNow - lastFast30mDraw >= FAST_30M_REFRESH_MS)) {
    lastFast30mDraw = displayNow;
    drawHeader();
    drawTabs();
    draw30MinDashboard();
  }

  if (screenMode != 0 && (displayNow - lastRedraw >= REDRAW_INTERVAL)) {
    lastRedraw = displayNow;
    drawAllScreens();
  }

  yield();
  delay(10);
}
