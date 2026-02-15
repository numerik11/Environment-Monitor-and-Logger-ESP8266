/*********************************************************
 * Environment Monitor (ESP8266 D1 R1 + ILI9341 + SCD30)
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
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <ESP8266mDNS.h>
#include <time.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_SCD30.h>

#if ARDUINO >= 100
  #include <stdint.h>
#endif

// ===================== Config =====================
#define ENABLE_FS 1
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

#if ENABLE_FS
  #include <FS.h>
  #include <LittleFS.h>
#endif

// ===================== Pins (D1 R1) =====================
#define TFT_CS      D8
#define TFT_DC      D2
#define I2C_SDA     D4
#define I2C_SCL     D3
#define BUTTON_PIN  D0
#define BL_PIN      D6

#define NUM_SCREENS 6
// ===================== Backlight control (ESP8266 PWM) =====================
static const uint16_t BL_RANGE  = 1023;
static const uint16_t BL_PWM_HZ = 20000;
static uint16_t blValue = 750;
static unsigned long _blLastReapply = 0;
volatile bool nextScreenReq = false;

// ===================== Wi-Fi / TZ =====================
ESP8266WebServer server(80);
DNSServer dnsServer;


static const char* TZ_ADELAIDE = "ACST-9:30ACDT-10:30,M10.1.0/2,M4.1.0/3";

// ===================== TFT =====================
Adafruit_ILI9341 tft(TFT_CS, TFT_DC);
static const int16_t SCREEN_WIDTH  = 320;
static const int16_t SCREEN_HEIGHT = 240;

#define RGB565(r,g,b) (((r&0xF8)<<8)|((g&0xFC)<<3)|((b)>>3))
static const uint16_t COL_BG        = RGB565(12,14,18);
static const uint16_t COL_CARD      = RGB565(24,28,34);
static const uint16_t COL_CARD_EDGE = RGB565(48,54,62);
static const uint16_t COL_GRID      = RGB565(60,68,78);
static const uint16_t COL_TEXT      = ILI9341_WHITE;
static const uint16_t COL_MUTED     = RGB565(160,170,186);
static const uint16_t COL_ACCENT    = RGB565(250,210,80);
static const uint16_t COL_CO2       = RGB565(255,120,90);
static const uint16_t COL_TEMP      = RGB565(80,210,255);
static const uint16_t COL_HUM       = RGB565(140,180,255);
static const uint16_t COL_VALUE_BG  = RGB565(34,38,46);
static const uint16_t COL_GOOD      = RGB565(70,160,120);
static const uint16_t COL_WARN      = RGB565(220,180,60);
static const uint16_t COL_BAD       = RGB565(220,80,80);

// ===================== UI layout =====================
struct Panel { int16_t x,y,w,h; };
static Panel header = {0,0,SCREEN_WIDTH,36};

static const uint8_t UI_RADIUS = 8;
static const int UI_TAB_H              = 18;
static const int UI_GUTTER             = 8;
static const int UI_PAD_X              = 10;
static const int UI_SECTION_TITLE_GAP  = 6;
static const int UI_BOTTOM_MARGIN      = 10;
static const int UI_CARD_HEADER_H      = 18;
static const int UI_GRID_PAD_X         = 8;
static const int UI_GRID_PAD_TOP       = 18;
static const int UI_GRID_PAD_BOTTOM    = 12;

// ===================== Plot helpers =====================
struct GridRect { int gx0,gx1,gy0,gy1; };
struct AxisScale { float vmin; float vmax; float step; int tickCount; float ticks[6]; };

// ===================== Sensor =====================
Adafruit_SCD30 scd30;

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
static uint32_t co2Sum=0, tempSum=0, humSum=0;
static uint16_t sampleCount=0;

static uint32_t bootTime=0;
static int lastDrawnSlot=-1;

static float currentCO2=NAN, currentTemp=NAN, currentHum=NAN;

// Reset origins for 12h/24h pages
static uint16_t slotsSince12hReset = 0;
static uint16_t slotsSince24hReset = 0;
static int twelveGraphResetOriginSlot     = -1;
static int twentyFourGraphResetOriginSlot = -1;

// ===================== Timing =====================
static uint32_t lastRedraw=0;
static const uint32_t REDRAW_INTERVAL=20000;

static uint32_t lastSensorData=0;
static const uint32_t SENSOR_TIMEOUT=30000;

static const uint32_t FAST_30M_REFRESH_MS = 10000;
static uint32_t lastFast30mDraw = 0;

// ===================== FS periodic save =====================
#if ENABLE_FS
static bool fsReady=false;
static uint32_t lastPeriodicSaveMs = 0;
static const uint32_t PERIODIC_SAVE_MS = 120000; // every 2 minutes
#endif

// ===================== State =====================
static bool offlineMode=false;
static uint32_t softStartTime=0;
static uint8_t screenMode=0;
static bool apFallbackActive = false;
static bool mdnsStarted = false;
static uint32_t lastStaRetryMs = 0;

// ----- Offline time anchor -----
static time_t   timeAnchorEpoch   = 0;
static uint32_t timeAnchorMillis  = 0;
static bool     timeAnchorValid   = false;

static uint32_t nextAnchorSave    = 0;
static const uint32_t ANCHOR_SAVE_MS = 600000;

// ===================== Forward decls (core) =====================
static bool initSensor();
static void connectWiFi();
static void initTime();
static bool isTimeValid();
static void anchorTimeToNow();
static time_t approxNow();
static void startFallbackAP();
static void onStaConnected();
static void retryStaWhileAp();

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
  tft.fillRoundRect(p.x,p.y,p.w,p.h,UI_RADIUS,COL_CARD);
  tft.drawRoundRect(p.x,p.y,p.w,p.h,UI_RADIUS,COL_CARD_EDGE);
}

// callbacks
void onBtnShort() {
  nextScreenReq = true;   // keep it minimal/safe
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

static void drawModernTrendIndicator(int16_t x, int16_t y, int8_t dir, uint16_t color){
  if(dir>0)      tft.fillTriangle(x, y+6, x+6, y+6, x+3, y, color);
  else if(dir<0) tft.fillTriangle(x, y, x+6, y, x+3, y+6, color);
}

static void drawOverlayBox(const char* title, const char* msg, uint16_t accent = COL_ACCENT) {
  const int w = SCREEN_WIDTH - 40;
  const int h = 72;
  const int x = 20;
  const int y = (SCREEN_HEIGHT - h)/2;

  tft.fillRoundRect(x, y, w, h, 10, COL_CARD);
  tft.drawRoundRect(x, y, w, h, 10, accent);

  tft.setTextSize(2); tft.setTextColor(COL_TEXT, COL_CARD);
  tft.setCursor(x+10, y+10); tft.print(title);

  tft.setTextSize(1);
  tft.setCursor(x+10, y+42); tft.print(msg);
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
  GridRect g;
  g.gx0 = p.x + UI_GRID_PAD_X;
  g.gx1 = p.x + p.w - UI_GRID_PAD_X;
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
  if(!(dataMax>dataMin)){ dataMin=clampMin; dataMax=clampMax; }
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

static int mapValueToY(float v, int gy0, int gy1, float vMin, float vMax){
  if(v<=vMin) return gy1;
  if(v>=vMax) return gy0;
  float t=(v-vMin)/(vMax-vMin);
  return gy1 - (int)(t*(gy1-gy0));
}

static void drawTimeGrid24h(int gx0,int gx1,int gy0,int gy1){
  for(int h=0; h<=24; ++h){
    int x = gx0 + (int)((long)h*(gx1-gx0)/24L);
    tft.drawFastVLine(x, gy0, gy1-gy0, COL_GRID);
    if((h & 3)==0) yield();
  }
}

static void drawTimeGrid12h(int gx0,int gx1,int gy0,int gy1){
  for(int h=0; h<=12; ++h){
    int x = gx0 + (int)((long)h*(gx1-gx0)/12L);
    tft.drawFastVLine(x, gy0, gy1-gy0, COL_GRID);
    if((h & 3)==0) yield();
  }
}

static void drawNowMarkerAndTag(int gx0,int gx1,int gy0,int gy1){
  if(currentSlot<0) return;
  int x = gx0 + (int)((long)currentSlot*(gx1-gx0)/(SLOTS_PER_DAY-1));
  tft.drawFastVLine(x, gy0, gy1-gy0, COL_ACCENT);
}

static void formatSlotTimeHHMM(int slot, char* out, size_t n){
  int mins = slot * SLOT_MINUTES;
  int hh = (mins/60)%24;
  int mm = mins%60;
  snprintf(out,n,"%02d:%02d",hh,mm);
}

// ===================== Live preview into current slot =====================
static inline void writeLiveSlotPreviewToBuffers() {
  if (currentSlot < 0 || sampleCount == 0) return;

  uint32_t avgC = co2Sum / sampleCount;
  float    avgT = (tempSum / (float)sampleCount) / 10.0f;
  float    avgH = (humSum  / (float)sampleCount) / 10.0f;

  if (avgC > 0 && avgC <= 65535) co2Slots[currentSlot] = (uint16_t)avgC;
  if (!isnan(avgT)) tempSlots[currentSlot] = encodeTemp(avgT);
  if (!isnan(avgH)) humSlots [currentSlot] = encodeHum(avgH);
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

static void drawSmoothCatmull(const int* xs, const int* ys, int n, uint16_t lineColor, int substeps){
  if (n < 2) return;
  int xPrev = xs[0], yPrev = ys[0];
  auto crY = [&](int i, float t)->float {
    int i0 = max(0, i-1);
    int i1 = i;
    int i2 = min(n-1, i+1);
    int i3 = min(n-1, i+2);
    float p0 = ys[i0], p1 = ys[i1], p2 = ys[i2], p3 = ys[i3];
    float t2 = t*t, t3 = t2*t;
    return 0.5f * ( (2*p1) + (-p0 + p2)*t + (2*p0 - 5*p1 + 4*p2 - p3)*t2 + (-p0 + 3*p1 - 3*p2 + p3)*t3 );
  };
  for (int i = 0; i < n-1; ++i) {
    yieldOften(i);
    int x1 = xs[i];
    int x2 = xs[i+1];
    int y2 = ys[i+1];

    if (substeps <= 1) {
      tft.drawLine(xPrev, yPrev, x2, y2, blendColor(lineColor, COL_BG, 40));
      tft.drawLine(xPrev, yPrev+1, x2, y2+1, lineColor);
      xPrev = x2; yPrev = y2;
      continue;
    }
    for (int s = 1; s <= substeps; ++s) {
      if ((s & 1) == 0) yield();
      float t = (float)s / (float)substeps;
      float xf = x1 + t * (x2 - x1);
      float yf = crY(i, t);
      int xi = (int)(xf + 0.5f);
      int yi = (int)(yf + 0.5f);
      tft.drawLine(xPrev, yPrev, xi, yi, blendColor(lineColor, COL_BG, 40));
      tft.drawLine(xPrev, yPrev+1, xi, yi+1, lineColor);
      xPrev = xi; yPrev = yi;
    }
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
      tft.drawLine(xs[i], ys[i], xs[i+1], ys[i+1], blendColor(lineColor, COL_BG, 40));
      tft.drawLine(xs[i], ys[i] + 1, xs[i+1], ys[i+1] + 1, lineColor);
    }
  }
#else
  for (int i = 0; i < n - 1; ++i) {
    yieldOften(i);
    tft.drawLine(xs[i], ys[i], xs[i+1], ys[i+1], blendColor(lineColor, COL_BG, 40));
    tft.drawLine(xs[i], ys[i] + 1, xs[i+1], ys[i+1] + 1, lineColor);
  }
#endif
}

// ===================== Mini graphs (30m cards) =====================
static void drawMiniGraphU16(const Panel& p, const uint16_t* buf, uint16_t lineColor,
                            uint16_t vminClamp, uint16_t vmaxClamp, const char* /*leftLbl*/, const char* unitLbl,
                            float /*currentValue*/, int minutesSpan, bool /*showBadge*/)
{
  drawCard(p);
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
  AxisScale sc=computeAxisScale(dmin,dmax,(float)vminClamp,(float)vmaxClamp,4);

  drawTimeGrid12h(g.gx0,g.gx1,g.gy0,g.gy1);

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
  drawCard(p);
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
  AxisScale sc=computeAxisScale(any?mn:clampMin, any?mx:clampMax, clampMin, clampMax, 4);

  drawTimeGrid12h(g.gx0,g.gx1,g.gy0,g.gy1);

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

// ===================== Floating badges on 30m page =====================
static void updateDashboardBadges() {
  if (screenMode != 0) return;

  const int contentTop = header.h + UI_TAB_H + UI_SECTION_TITLE_GAP;
  const int availH     = SCREEN_HEIGHT - (contentTop + UI_BOTTOM_MARGIN);

  int co2H    = (availH * 3) / 5;
  int topH    = availH - co2H - UI_GUTTER;

  Panel tempCard = {UI_PAD_X, contentTop,
                    (SCREEN_WIDTH/2) - UI_PAD_X - (UI_GUTTER/2), topH};
  Panel humCard  = {tempCard.x + tempCard.w + UI_GUTTER, tempCard.y, tempCard.w, topH};
  Panel co2Card  = {UI_PAD_X, contentTop + topH + UI_GUTTER,
                    SCREEN_WIDTH - 2*UI_PAD_X, co2H};

  const int BADGE_H = 16;
  const int PAD_XR  = 8;

  auto badgeY  = [&](const Panel& p){ return p.y + (UI_CARD_HEADER_H / 2) - (BADGE_H / 2); };
  auto badgeXr = [&](const Panel& p){ return p.x + p.w - PAD_XR; };

  auto wipeAreaDyn = [&](const Panel& p, const char* txt){
    int w = (int)strlen(txt) * 6 + 12;
    if (w < 64) w = 64;
    if (w > p.w - 20) w = p.w - 20;
    int xr = badgeXr(p);
    int y  = badgeY(p);
    tft.fillRect(xr - w - 2, y - 2, w + 4, BADGE_H + 4, COL_CARD);
  };

  if (!isnan(currentCO2)) {
    char txt[20]; snprintf(txt, sizeof(txt), "%.0f ppm", currentCO2);
    wipeAreaDyn(co2Card, txt);
    uint16_t col = (currentCO2 > 1200) ? COL_BAD : (currentCO2 > 800) ? COL_WARN : COL_GOOD;
    drawModernValueBadge(badgeXr(co2Card), badgeY(co2Card), txt, col);
  }
  if (!isnan(currentTemp)) {
    char txt[20]; snprintf(txt, sizeof(txt), "%.1f C", currentTemp);
    wipeAreaDyn(tempCard, txt);
    drawModernValueBadge(badgeXr(tempCard), badgeY(tempCard), txt, COL_TEXT);
  }
  if (!isnan(currentHum)) {
    char txt[20]; snprintf(txt, sizeof(txt), "%.0f %%", currentHum);
    wipeAreaDyn(humCard, txt);
    drawModernValueBadge(badgeXr(humCard), badgeY(humCard), txt, COL_TEXT);
  }
}

// ===================== Pages =====================
static void draw30MinDashboard() {
  const int contentTop = header.h + UI_TAB_H + UI_SECTION_TITLE_GAP;
  const int availH = SCREEN_HEIGHT - (contentTop + UI_BOTTOM_MARGIN);

  int co2H    = (availH * 3) / 5;
  int topH    = availH - co2H - UI_GUTTER;

  Panel tempCard = {UI_PAD_X, contentTop,
                    (SCREEN_WIDTH/2) - UI_PAD_X - (UI_GUTTER/2), topH};
  Panel humCard  = {tempCard.x + tempCard.w + UI_GUTTER, tempCard.y, tempCard.w, topH};
  Panel co2Card  = {UI_PAD_X, contentTop + topH + UI_GUTTER,
                    SCREEN_WIDTH - 2*UI_PAD_X, co2H};

  // Temp
  tft.fillRoundRect(tempCard.x+2, tempCard.y+2, tempCard.w, tempCard.h, 8, COL_GRID);
  drawCard(tempCard);
  tft.fillRect(tempCard.x, tempCard.y, tempCard.w, UI_CARD_HEADER_H, COL_TEMP);
  tft.setTextSize(1); tft.setTextColor(COL_TEXT, COL_TEMP);
  tft.setCursor(tempCard.x+6, tempCard.y+5); tft.print("Temp - 30m");
  drawMiniGraphU8(tempCard, tempSlots, COL_TEMP, encodeTemp(10), encodeTemp(40), "", "C", currentTemp, decodeTemp, 36, false);

  // Hum
  tft.fillRoundRect(humCard.x+2, humCard.y+2, humCard.w, humCard.h, 8, COL_GRID);
  drawCard(humCard);
  tft.fillRect(humCard.x, humCard.y, humCard.w, UI_CARD_HEADER_H, COL_HUM);
  tft.setTextSize(1); tft.setTextColor(COL_TEXT, COL_HUM);
  tft.setCursor(humCard.x+6, humCard.y+5); tft.print("Humidity - 30m");
  drawMiniGraphU8(humCard, humSlots, COL_HUM, encodeHum(20), encodeHum(80), "", "%", currentHum, decodeHum, 36, false);

  // CO2
  tft.fillRoundRect(co2Card.x+2, co2Card.y+2, co2Card.w, co2Card.h, 8, COL_GRID);
  drawCard(co2Card);
  tft.fillRect(co2Card.x, co2Card.y, co2Card.w, UI_CARD_HEADER_H, COL_CO2);
  tft.setTextSize(1); tft.setTextColor(COL_TEXT, COL_CO2);
  tft.setCursor(co2Card.x+6, co2Card.y+5); tft.print("CO2 - 30m");
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
  tft.fillRect(0,0,SCREEN_WIDTH,header.h,COL_CARD);
  tft.fillRect(0, header.h-4, SCREEN_WIDTH, 4, COL_ACCENT);

  tft.setTextSize(1); tft.setTextColor(COL_ACCENT, COL_CARD);
  const char* title = "Environment Monitor";
  int16_t tw = (int)strlen(title)*6;
  tft.setCursor((SCREEN_WIDTH - tw)/2, 8); tft.print(title);

  time_t ts = approxNow();
  if (ts > 0) {
    struct tm ti; localtime_r(&ts, &ti);
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", ti.tm_hour, ti.tm_min);
    printRightAligned(SCREEN_WIDTH-40, 10, buf, 2, COL_TEXT, COL_CARD);
  } else {
    printRightAligned(SCREEN_WIDTH-40, 10, "--:--", 2, COL_MUTED, COL_CARD);
  }

  if (WiFi.status() == WL_CONNECTED){
    int rssi = WiFi.RSSI();
    uint8_t bars = (rssi>-55)?3:(rssi>-67)?2:(rssi>-80)?1:0;
    drawWiFiIcon(SCREEN_WIDTH-20, 12, bars, COL_ACCENT);
  } else {
    printRightAligned(SCREEN_WIDTH-8, 12, "OFF", 1, COL_MUTED, COL_CARD);
  }
}

static void drawTabs(){
  static const char* names[6] = {"30m", "STATS", "CO2 24h", "T/H 24h", "CO2 12h", "T/H 12h"};
  const int y = header.h + 2;
  int x = UI_PAD_X;
  for(int i=0;i<6;i++){
    int w = (int)strlen(names[i]) * 6 + 16;
    bool active=(i==screenMode);
    uint16_t bg = active ? COL_ACCENT : COL_CARD;
    uint16_t fg = active ? COL_BG     : COL_MUTED;
    tft.fillRoundRect(x, y, w, UI_TAB_H-2, 6, bg);
    tft.setTextSize(1); tft.setTextColor(fg, bg);
    tft.setCursor(x+6, y+6); tft.print(names[i]);
    x += w + 6;
  }
}

// ---- Fullscreen CO2 24h ----
static void drawCO2FullScreen(){
  const int contentTop = header.h + UI_TAB_H + UI_SECTION_TITLE_GAP;
  Panel full={UI_PAD_X,contentTop,SCREEN_WIDTH-2*UI_PAD_X,
              SCREEN_HEIGHT-(contentTop+UI_BOTTOM_MARGIN)};

  tft.fillRoundRect(full.x+2, full.y+2, full.w, full.h, 10, COL_GRID);
  drawCard(full);
  tft.fillRect(full.x, full.y, full.w, 20, COL_CO2);
  tft.setTextSize(1); tft.setTextColor(COL_TEXT, COL_CO2);
  tft.setCursor(full.x+6, full.y+6); tft.print("CO2 - Last 24h");

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
  AxisScale sc = computeAxisScale(dmin, dmax, 300.f, 2000.f, 5);

  for(int i=0;i<sc.tickCount;i++){
    const float tv=sc.ticks[i];
    const int y=mapValueToY(tv, g.gy0,g.gy1, sc.vmin,sc.vmax);
    tft.drawFastHLine(g.gx0, y, (g.gx1-g.gx0), COL_GRID);
    char buf[20]; snprintf(buf,sizeof(buf),"%.0fppm",tv);
    tft.setTextColor(COL_MUTED, COL_CARD);
    tft.setCursor(full.x+2, y-4);
    tft.print(buf);
  }

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
    if(i>dispStart && co2Slots[i-1]) acc += co2Slots[i-1], w+=1.0f;
    if(i<SLOTS_PER_DAY-1 && co2Slots[i+1]) acc += co2Slots[i+1], w+=1.0f;
    v = acc/w;
#endif
    if(i!=lastIdx+1 && segLen) flush();
    push(i, v); lastIdx=i;
  }
  flush();

  drawNowMarkerAndTag(g.gx0,g.gx1,g.gy0,g.gy1);

  if(!isnan(currentCO2)){
    char bufv[20]; snprintf(bufv,sizeof(bufv),"%.0f ppm",currentCO2);
    uint16_t col=(currentCO2>1200)?COL_BAD:(currentCO2>800?COL_WARN:COL_GOOD);
    drawModernValueBadge(full.x+full.w-10, full.y+24, bufv, col);
    int8_t tr=co2Trend();
    if(tr!=0) drawModernTrendIndicator(full.x+full.w-22, full.y+42, tr, (tr>0)?COL_WARN:COL_GOOD);
  }
}

// ---- Fullscreen Temp+Hum 24h ----
static void drawTempHumFullScreen(){
  const int contentTop = header.h + UI_TAB_H + UI_SECTION_TITLE_GAP;
  const int availH = SCREEN_HEIGHT - (contentTop + UI_BOTTOM_MARGIN);
  const int eachH  = (availH-UI_GUTTER)/2;

  const int dispStart = (twentyFourGraphResetOriginSlot >= 0) ? twentyFourGraphResetOriginSlot : 0;

  // Temperature card
  Panel top={UI_PAD_X,contentTop,SCREEN_WIDTH-2*UI_PAD_X,eachH};
  tft.fillRoundRect(top.x+2, top.y+2, top.w, top.h, 10, COL_GRID);
  drawCard(top);
  tft.fillRect(top.x, top.y, top.w, 20, COL_TEMP);
  tft.setTextSize(1); tft.setTextColor(COL_TEXT, COL_TEMP);
  tft.setCursor(top.x+6, top.y+6); tft.print("Temperature - 24h");

  GridRect g=gridRect(top);
  drawTimeGrid24h(g.gx0,g.gx1,g.gy0,g.gy1);

  float tmin=1e9f,tmax=-1e9f; bool tany=false;
  for(int i=dispStart;i<SLOTS_PER_DAY;i++){
    uint8_t r=tempSlots[i]; if(!r) continue;
    float v=decodeTemp(r);
    tany=true; if(v<tmin) tmin=v; if(v>tmax) tmax=v;
  }
  AxisScale ts=computeAxisScale(tany?tmin:10.f, tany?tmax:40.f, 10.f, 40.f, 5);

  for(int i=0;i<ts.tickCount;i++){
    const float tv=ts.ticks[i];
    int y=mapValueToY(tv,g.gy0,g.gy1,ts.vmin,ts.vmax);
    tft.drawFastHLine(g.gx0,y,(g.gx1-g.gx0),COL_GRID);
    char buf[20]; snprintf(buf,sizeof(buf),"%.0fC",tv);
    tft.setTextColor(COL_MUTED, COL_CARD);
    tft.setCursor(top.x+2,y-4);
    tft.print(buf);
  }

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

  drawNowMarkerAndTag(g.gx0,g.gx1,g.gy0,g.gy1);

  if(!isnan(currentTemp)){
    char bufv[20]; snprintf(bufv,sizeof(bufv),"%.1fC",currentTemp);
    drawModernValueBadge(top.x+top.w-10, top.y+24, bufv, COL_TEXT);
    int8_t tr=u8Trend(tempSlots);
    if(tr!=0) drawModernTrendIndicator(top.x+top.w-22, top.y+42, tr, (tr>0)?COL_WARN:COL_GOOD);
  }

  // Humidity card
  Panel bot={UI_PAD_X,contentTop+eachH+UI_GUTTER,SCREEN_WIDTH-2*UI_PAD_X,eachH};
  tft.fillRoundRect(bot.x+2, bot.y+2, bot.w, bot.h, 10, COL_GRID);
  drawCard(bot);
  tft.fillRect(bot.x, bot.y, bot.w, 20, COL_HUM);
  tft.setTextSize(1); tft.setTextColor(COL_TEXT, COL_HUM);
  tft.setCursor(bot.x+6, bot.y+6); tft.print("Humidity - 24h");

  g=gridRect(bot);
  drawTimeGrid24h(g.gx0,g.gx1,g.gy0,g.gy1);

  float hmin=1e9f,hmax=-1e9f; bool hany=false;
  for(int i=dispStart;i<SLOTS_PER_DAY;i++){
    uint8_t r=humSlots[i]; if(!r) continue;
    float v=decodeHum(r);
    hany=true; if(v<hmin) hmin=v; if(v>hmax) hmax=v;
  }
  AxisScale hs=computeAxisScale(hany?hmin:20.f, hany?hmax:80.f, 20.f, 80.f, 5);

  for(int i=0;i<hs.tickCount;i++){
    const float tv=hs.ticks[i];
    int y=mapValueToY(tv,g.gy0,g.gy1,hs.vmin,hs.vmax);
    tft.drawFastHLine(g.gx0,y,(g.gx1-g.gx0),COL_GRID);
    char buf[20]; snprintf(buf,sizeof(buf),"%.0f%%",tv);
    tft.setTextColor(COL_MUTED, COL_CARD);
    tft.setCursor(bot.x+2,y-4);
    tft.print(buf);
  }

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

  drawNowMarkerAndTag(g.gx0,g.gx1,g.gy0,g.gy1);

  if(!isnan(currentHum)){
    char bufv[20]; snprintf(bufv,sizeof(bufv),"%.0f%%",currentHum);
    drawModernValueBadge(bot.x+bot.w-10, bot.y+24, bufv, COL_TEXT);
    int8_t tr=u8Trend(humSlots);
    if(tr!=0) drawModernTrendIndicator(bot.x+bot.w-22, bot.y+42, tr, (tr>0)?COL_WARN:COL_GOOD);
  }
}

// ---- CO2 12h ----
static void drawCO212Hour(){
  const int contentTop = header.h + UI_TAB_H + UI_SECTION_TITLE_GAP;
  Panel half = { UI_PAD_X, contentTop, SCREEN_WIDTH - 2*UI_PAD_X,
                 SCREEN_HEIGHT - (contentTop + UI_BOTTOM_MARGIN) };

  tft.fillRoundRect(half.x+2, half.y+2, half.w, half.h, 10, COL_GRID);
  drawCard(half);
  tft.fillRect(half.x, half.y, half.w, 20, COL_CO2);
  tft.setTextSize(1); tft.setTextColor(COL_TEXT, COL_CO2);
  tft.setCursor(half.x+6, half.y+6); tft.print("CO2 - Last 12h");

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
  AxisScale sc = computeAxisScale(dmin, dmax, 300.f, 2000.f, 5);

  tft.setTextSize(1);
  for (int i = 0; i < sc.tickCount; ++i) {
    const float tv = sc.ticks[i];
    int y = mapValueToY(tv, g.gy0, g.gy1, sc.vmin, sc.vmax);
    tft.drawFastHLine(g.gx0, y, (g.gx1 - g.gx0), COL_GRID);
    char lab[20]; snprintf(lab, sizeof(lab), "%.0fppm", tv);
    tft.setTextColor(COL_MUTED, COL_CARD);
    tft.setCursor(half.x + 2, y - 4);
    tft.print(lab);
  }

  static int xs[SLOTS_PER_12H], ys[SLOTS_PER_12H];
  int segLen = 0, lastIdx = -9999;

  auto flush = [&](){ if (segLen >= 2) drawSegmentedLine(xs, ys, segLen, COL_CO2); segLen = 0; };
  auto push  = [&](int i, float v){
    int N = max(2, count);
    int x = g.gx0 + (i * (g.gx1 - g.gx0)) / (N - 1);
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

  // no "now" marker on 12h pages

  if (!isnan(currentCO2)) {
    char bufv[20]; snprintf(bufv, sizeof(bufv), "%.0f ppm", currentCO2);
    const int badgeH = 12;
    const int yBadge = half.y + (UI_CARD_HEADER_H - badgeH) / 2;
    const int xRight = half.x + half.w - 10;
    int w = (int)strlen(bufv) * 6 + 12;
    tft.fillRect(xRight - w - 2, half.y + 2, w + 4, UI_CARD_HEADER_H - 4, COL_CO2);
    drawModernValueBadge(xRight, yBadge, bufv,
      (currentCO2 > 1200) ? COL_BAD : (currentCO2 > 800 ? COL_WARN : COL_GOOD));
    int8_t tr = co2Trend();
    if (tr != 0) drawModernTrendIndicator(xRight - w - 10, half.y + 4, tr, (tr > 0) ? COL_WARN : COL_GOOD);
  }
}

// ---- Temp/Hum 12h ----
static void drawTempHum12Hour(){
  const int contentTop = header.h + UI_TAB_H + UI_SECTION_TITLE_GAP;
  const int availH=SCREEN_HEIGHT-(contentTop+UI_BOTTOM_MARGIN);
  const int eachH=(availH-UI_GUTTER)/2;

  const int baseStart = max(0, currentSlot - (SLOTS_PER_12H - 1));
  const int dispStart = (twelveGraphResetOriginSlot >= 0) ? max(baseStart, twelveGraphResetOriginSlot) : baseStart;

  // Temperature
  Panel top={UI_PAD_X,contentTop,SCREEN_WIDTH-2*UI_PAD_X,eachH};
  tft.fillRoundRect(top.x+2, top.y+2, top.w, top.h, 10, COL_GRID);
  drawCard(top);
  tft.fillRect(top.x, top.y, top.w, 20, COL_TEMP);
  tft.setTextSize(1); tft.setTextColor(COL_TEXT, COL_TEMP);
  tft.setCursor(top.x+6, top.y+6); tft.print("Temperature - 12h");

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
  AxisScale ts=computeAxisScale(tany?tmin:10.f, tany?tmax:40.f, 10.f, 40.f, 5);

  static int xs[SLOTS_PER_12H], ys[SLOTS_PER_12H];
  int segLen=0, lastIdx=-9999;
  auto flush=[&](){ if(segLen>=2) drawSegmentedLine(xs, ys, segLen, COL_TEMP); segLen=0; };
  auto push=[&](int i, float v){
    int N=max(2,countT);
    int x=g.gx0 + (i*(g.gx1-g.gx0))/(N-1);
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

  if(!isnan(currentTemp)){
    char bufv[20]; snprintf(bufv,sizeof(bufv),"%.1fC",currentTemp);
    drawModernValueBadge(top.x+top.w-10, top.y+24, bufv, COL_TEXT);
    int8_t tr=u8Trend(tempSlots);
    if(tr!=0) drawModernTrendIndicator(top.x+top.w-22, top.y+42, tr, (tr>0)?COL_WARN:COL_GOOD);
  }

  // Humidity
  Panel bot={UI_PAD_X,contentTop+eachH+UI_GUTTER,SCREEN_WIDTH-2*UI_PAD_X,eachH};
  tft.fillRoundRect(bot.x+2, bot.y+2, bot.w, bot.h, 10, COL_GRID);
  drawCard(bot);
  tft.fillRect(bot.x, bot.y, bot.w, 20, COL_HUM);
  tft.setTextSize(1); tft.setTextColor(COL_TEXT, COL_HUM);
  tft.setCursor(bot.x+6, bot.y+6); tft.print("Humidity - 12h");

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
  AxisScale hs=computeAxisScale(hany?hmin:20.f, hany?hmax:80.f, 20.f, 80.f, 5);

  static int xs2[SLOTS_PER_12H], ys2[SLOTS_PER_12H];
  int segLen2=0; lastIdx=-9999;
  auto flush2=[&](){ if(segLen2>=2) drawSegmentedLine(xs2, ys2, segLen2, COL_HUM); segLen2=0; };
  auto push2=[&](int i, float v){
    int N=max(2,countH);
    int x=g.gx0 + (i*(g.gx1-g.gx0))/(N-1);
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

  if(!isnan(currentHum)){
    char bufv[20]; snprintf(bufv,sizeof(bufv),"%.0f%%",currentHum);
    drawModernValueBadge(bot.x+bot.w-10, bot.y+24, bufv, COL_TEXT);
    int8_t tr=u8Trend(humSlots);
    if(tr!=0) drawModernTrendIndicator(bot.x+bot.w-22, bot.y+42, tr, (tr>0)?COL_WARN:COL_GOOD);
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
      int y = header.h + UI_TAB_H + 10;
      const int cardH = 40;
      const int gutter = 12;

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
        tft.setTextSize(1);
        tft.setTextColor(color,COL_CARD);
        tft.setCursor(p.x+8,p.y+6);
        tft.print(name);

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

        tft.setTextColor(COL_TEXT, COL_CARD);
        tft.setCursor(p.x+12, p.y+20);
        tft.print(avgStr);

        tft.setCursor(p.x+12, p.y+32);
        if(d.count){
          char line[46];
          snprintf(line,sizeof(line),"L: %s @ %s  H: %s @ %s", lowStr,lowTime, highStr,highTime);
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
      tft.setCursor(UI_PAD_X, SCREEN_HEIGHT-16);
      if(totalReadings>0){
        char s[44];
        snprintf(s,sizeof(s),"%d readings - %d%% complete", totalReadings/3, (totalReadings*100)/(SLOTS_PER_DAY*3));
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
}

// ===================== Data accumulation =====================
static void resetAccumulators(){ co2Sum=tempSum=humSum=0; sampleCount=0; }

static void commitSlotData(int slot){
  if(slot<0||slot>=SLOTS_PER_DAY||sampleCount==0) return;

  uint32_t c = co2Sum / sampleCount;
  float t = (tempSum/(float)sampleCount)/10.0f;
  float h = (humSum /(float)sampleCount)/10.0f;

  if(c>0 && c<=65535) co2Slots[slot] = (uint16_t)c;
  tempSlots[slot] = encodeTemp(t);
  humSlots [slot] = encodeHum(h);
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
  // NC to GND + INPUT_PULLUP:
  //  - idle LOW, press HIGH (opens)
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
    attachInterrupt(digitalPinToInterrupt(_pin), isrThunk, RISING); // NC press = LOW->HIGH
  }

  void tick(){
    // confirm window timeout
    if(_confirmPending && (millis() > _confirmDeadline)){
      _confirmPending = false;
      if(_onTimeout) _onTimeout();
    }

    // If we saw a press edge, enter pressed state (debounced)
    if(_irq){
      _irq = false;

      delay(25); // debounce settle
      if(digitalRead(_pin) == HIGH){ // still pressed
        _pressed = true;
        _pressAt = millis();
        _longFired = false;
      }
    }

    // While pressed: check for long press or release
    if(_pressed){
      if(digitalRead(_pin) == LOW){
        // released
        _pressed = false;

        // if long-press already fired, do nothing on release
        if(_longFired){
          return;
        }

        // short press:
        if(_confirmPending){
          _confirmPending = false;
          if(_onConfirm) _onConfirm();
        }else{
          if(_onShort) _onShort();
        }
        return;
      }

      // still pressed: long press?
      if(!_longFired && (millis() - _pressAt >= _longMs)){
        _longFired = true;

        // open confirm window
        _confirmPending = true;
        _confirmDeadline = millis() + _confirmWindowMs;

        if(_onLongReq) _onLongReq();
      }
    }
  }

  // called from ISR thunk
  void onIRQ(){
    // tiny ISR debounce
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
  timeAnchorMillis = b.ms;
  timeAnchorValid  = true;
  return true;
}
#endif

// ===================== FS: per-day files =====================
#if ENABLE_FS
struct StorageHeader{
  uint32_t magic;
  uint16_t year;
  uint16_t dayOfYear;  // 1..366
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

// Load today's file if time valid; else load newest file.
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

    clearData(); // time valid but no file: start clean
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
static bool initSensor() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(50);

  for (int i = 0; i < 3; i++) {
    if (scd30.begin()) {
      scd30.setMeasurementInterval(5);
      scd30.setTemperatureOffset(0);
      scd30.selfCalibrationEnabled(SCD30_ASC ? true : false);
      lastSensorData = millis();
      return true;
    }
    delay(250);
  }
  return false;
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
    if (!isTimeValid()) {
      initTime();
      if (isTimeValid()) anchorTimeToNow();
    }
    return;
  }

  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastStaRetryMs) < WIFI_STA_RETRY_MS) return;
  lastStaRetryMs = nowMs;

  Serial.println("Retrying STA connection while AP is active...");
  WiFi.begin(); // retry saved credentials
}

static void initTime() {
  if (!offlineMode) {
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

// ---------- Time helper ----------
static bool isTimeValid() {
  return !offlineMode && time(nullptr) > 1600000000;
}

//  -----   Web UI  --------

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

<div id="tip"></div>
</div>

<script>
(() => {
  const TODAY="__TODAY__";
  const $=id=>document.getElementById(id);
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

  function fetchJSON(url){
    return fetch(url,{cache:"no-store"}).then(r=>r.json());
  }

  function fetchStatus(){
    fetchJSON("/status").then(js=>{
      lastStatusT=Date.now();
      const co2=(js.co2!=null)?Math.round(js.co2):"--";
      const t  =(js.temp!=null)?Number(js.temp).toFixed(1):"--";
      const h  =(js.hum!=null)?Math.round(js.hum):"--";
      liveEl.textContent=`Live: ${co2} ppm · ${t} °C · ${h} %`;
    }).catch(()=>{});
  }
  function tickLiveChip(){
    const stale=(Date.now()-lastStatusT)>60000;
    liveChip.classList.toggle("stale", stale);
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
    }).catch(()=>{});
  }

  function shiftDay(dir){
    let i=sel.selectedIndex+dir;
    i=Math.max(0,Math.min(sel.options.length-1,i));
    sel.selectedIndex=i;
    load(sel.value,true);
  }
  $("prev").onclick=()=>shiftDay(-1);
  $("next").onclick=()=>shiftDay(+1);

  // ----- prefs -----
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

  // ----- data load -----
  function load(name, resetView){
    LS.s("daySel",name);
    const url = (name===TODAY) ? "/api/today" : ("/api/daydata?name="+encodeURIComponent(name));
    fetchJSON(url).then(d=>{
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
    }).catch(()=>{ data=null; draw(); renderStats(); });
  }
  sel.addEventListener("change",()=>load(sel.value,true));

  // ----- range buttons -----
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

  // ----- canvas sizing -----
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

  // ----- smoothing (3-tap, gap-aware) -----
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

  // ----- stats -----
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

  // ----- draw helpers -----
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

    for(let i=i0;i<=i1;i++){
      const v=arr[i];
      if(isGap(v)){
        if(seg.length>1){ useSpline ? drawSpline(seg) : drawPolyline(seg); }
        seg=[];
        continue;
      }
      seg.push([xOf(i), yOf(v)]);
    }
    if(seg.length>1){ useSpline ? drawSpline(seg) : drawPolyline(seg); }

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

    // captions
    ctx.fillStyle=muted; ctx.font="11px ui-monospace,Menlo,Consolas,monospace";
    ctx.textAlign="left";  ctx.fillText("CO₂ ppm", 6, gy0-2);

    // clip plot to rounded rect
    ctx.save();
    roundRectPath(gx0, gy0, (gx1-gx0), (gy1-gy0), 12);
    ctx.clip();

    // grid: majors + dashed minors
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

    // x ticks verticals (in-clip)
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

    // series
    const mode=modeEl.value||"line";
    const showC=!!cOn.checked, showT=!!tOn.checked, showH=!!hOn.checked;

    if(showT){ areaFill(tt, TT, xOf, yT, v=>v==null, i0,i1, gy1); line(tt, TT, xOf, yT, v=>v==null, i0,i1); }
    if(showH){ areaFill(hh, HH, xOf, yH, v=>v==null, i0,i1, gy1); line(hh, HH, xOf, yH, v=>v==null, i0,i1); }

    if(showC){
      if(mode==="bar") bars(co2, CO2, xOf, yC, v=>!v, i0,i1, gy1, gxW);
      else if(mode==="area"){ areaFill(co2, CO2, xOf, yC, v=>!v, i0,i1, gy1); line(co2, CO2, xOf, yC, v=>!v, i0,i1); }
      else line(co2, CO2, xOf, yC, v=>!v, i0,i1);
    }

    // hover crosshair + dots (in-clip)
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

    ctx.restore(); // end clip

    // plot border
    ctx.strokeStyle=stroke; ctx.lineWidth=1;
    roundRectPath(gx0, gy0, (gx1-gx0), (gy1-gy0), 12);
    ctx.stroke();

    // CO2 y labels (left)
    ctx.fillStyle=muted;
    ctx.font="11px ui-monospace,Menlo,Consolas,monospace";
    ctx.textAlign="right";
    for(let k=0;k<5;k++){
      const y=gy0 + (gy1-gy0)*(k/4);
      const v = cmax - (cmax-cmin)*(k/4);
      ctx.fillText(String(Math.round(v)), gx0-8, y+4);
    }

    // x labels (below)
    ctx.fillStyle=muted; ctx.textAlign="center";
    for(const it of tickLabels) ctx.fillText(it[1], it[0], gy1+20);
  }

  // ----- tooltip -----
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

  // mouse/pen hover
  cv.addEventListener("pointermove",(ev)=>{
    if(ev.pointerType==="touch") return; // touch handled separately
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

  // wheel zoom (desktop)
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

  // ----- touch gestures (phones): drag=pan, pinch=zoom, double-tap=reset -----
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

  // export
  $("btnPNG").onclick=()=>{
    const url=cv.toDataURL("image/png");
    const a=document.createElement("a");
    a.download="env_graph.png"; a.href=url; a.click();
  };

  // today auto refresh
  function todayTick(){ if(sel.value===TODAY) load(TODAY,false); }

  // init
  restorePrefs();
  fetchStatus(); setInterval(fetchStatus,30000);
  setInterval(tickLiveChip,1500);
  setInterval(todayTick,12000);

  requestAnimationFrame(()=>{ resize(); fetchDayList(); });

})();
</script></body></html>
)HTML";


// ---------- Web handler ----------
static void handleWebRoot() {
  server.send_P(200, "text/html; charset=utf-8", WEB_ROOT_HTML);
}

static void handleStatus() {
  // No caching
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

  out += "}";

  server.send(200, "application/json", out);
}


static void handleToday() {
  writeLiveSlotPreviewToBuffers();

  // No caching
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  auto sendS = [&](const String& s) { server.sendContent(s); yield(); };
  auto sendC = [&](const char* s)   { server.sendContent(s); yield(); };

  const int slotMinutes = SLOT_MINUTES;
  const unsigned long nowEpoch = (unsigned long)approxNow();
  const int slot = currentSlot;
  const int minuteOfDay = (slot < 0) ? 0 : slot * slotMinutes;

  sendC("{");
  sendC("\"name\":\"__TODAY__\""); // (nice for UI / stats handler)
  sendC(",\"stepMin\":");       sendS(String(slotMinutes));
  sendC(",\"slots\":");         sendS(String(SLOTS_PER_DAY));
  sendC(",\"epoch\":");         sendS(String(nowEpoch));
  sendC(",\"slot\":");          sendS(String(slot));
  sendC(",\"minuteOfDay\":");   sendS(String(minuteOfDay));

  // CO2: use 0 for gaps (UI treats 0 as "missing")
  sendC(",\"co2\":[");
  for (int i = 0; i < SLOTS_PER_DAY; i++) {
    if (i) sendC(",");
    sendS(String(co2Slots[i]));
  }
  sendC("]");

  // Temp: null for missing
  sendC(",\"temp\":[");
  for (int i = 0; i < SLOTS_PER_DAY; i++) {
    if (i) sendC(",");
    if (!tempSlots[i]) sendC("null");
    else sendS(String(decodeTemp(tempSlots[i]), 1));
  }
  sendC("]");

  // Hum: null for missing
  sendC(",\"hum\":[");
  for (int i = 0; i < SLOTS_PER_DAY; i++) {
    if (i) sendC(",");
    if (!humSlots[i]) sendC("null");
    else sendS(String(decodeHum(humSlots[i]), 0));
  }
  sendC("]");

  sendC("}");

  // IMPORTANT: terminate chunked response
  server.sendContent("");
  // optional but helps some clients
  server.client().stop();
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

  auto sendS = [&](const String& s) { server.sendContent(s); yield(); };
  auto sendC = [&](const char* s)   { server.sendContent(s); yield(); };

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
#endif
}

// ===================== setup / loop =====================
void setup(){
  Serial.begin(115200);
  bootTime = millis();

  // Backlight
  pinMode(BL_PIN, OUTPUT);
  digitalWrite(BL_PIN, LOW);
  analogWriteFreq(BL_PWM_HZ);
  analogWriteRange(BL_RANGE);
  analogWrite(BL_PIN, blValue);

  // TFT
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(COL_BG);
  tft.setTextSize(2);
  tft.setTextColor(COL_ACCENT, COL_BG);
  tft.setCursor(10, 8);
  tft.print("Booting...");

  // Data
  clearData();

#if ENABLE_FS
  fsReady = LittleFS.begin();
  if (!fsReady) {
    Serial.println("LittleFS mount failed (NOT formatting).");
  } else {
    loadTimeAnchor();
    loadDataFromFS();
  }
#endif

  // Sensor
  initSensor();

  // WiFi + Time
  connectWiFi();
  initTime();
  if (isTimeValid()) anchorTimeToNow();

  // Web routes (always serve UI; AP fallback uses 192.168.4.1)
  server.on("/", HTTP_GET, handleWebRoot);
  server.on("/api/daylist", HTTP_GET, handleDayList);
  server.on("/api/daydata", HTTP_GET, handleDayData);
  server.on("/api/today",   HTTP_GET, handleToday);
  server.on("/status",      HTTP_GET, handleStatus);
  server.begin();

  button.begin(BUTTON_PIN, onBtnShort, onBtnLongRequest, onBtnConfirm, onBtnConfirmTimeout, 5000, 5000);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.printf("PIN=%d irq=%d\n", (int)BUTTON_PIN, (int)digitalPinToInterrupt(BUTTON_PIN));;

Serial.printf("BUTTON pin=%d idle=%d irq=%d\n",
              BUTTON_PIN, digitalRead(BUTTON_PIN),
              digitalPinToInterrupt(BUTTON_PIN));
Serial.print("BUTTON pin = "); Serial.println(BUTTON_PIN);
Serial.print("BUTTON idle read = ");
Serial.println(digitalRead(BUTTON_PIN));  // should be LOW when not pressed

Serial.print("BUTTON irq = ");
Serial.println(digitalPinToInterrupt(BUTTON_PIN)); // should NOT be NOT_AN_INTERRUPT


  // Initial render
  drawAllScreens();
}

void loop() {
  const uint32_t nowMs = millis();

  // ---------- Backlight guard ----------
  if (nowMs - _blLastReapply > 60000UL) {
    analogWriteFreq(BL_PWM_HZ);
    analogWriteRange(BL_RANGE);
    analogWrite(BL_PIN, blValue);
    _blLastReapply = nowMs;
  }

  // ---------- Network tasks ----------
  if (WiFi.getMode() != WIFI_OFF) {
    server.handleClient();
    if (apFallbackActive) {
      dnsServer.processNextRequest();
      retryStaWhileAp();
    }
    if (mdnsStarted && WiFi.status() == WL_CONNECTED) {
      MDNS.update();
    }
  }

  // ---------- Button state machine ----------
  button.tick();

  static int last = -1;
  int v = digitalRead(BUTTON_PIN);
  if (v != last) {
    last = v;
    Serial.printf("read=%d\n", v);
  }
  delay(5);

  // Roll screens on short-press request
  if (nextScreenReq) {
    nextScreenReq = false;

    screenMode = (screenMode + 1) % NUM_SCREENS;

    // force immediate redraw
    lastFast30mDraw = 0;
    lastRedraw      = 0;

    drawAllScreens();
  }

  // ---------- Keep offline anchor fresh if time valid ----------
  if (isTimeValid() && nowMs >= nextAnchorSave) {
    anchorTimeToNow();
  }

  // ---------- TIME FIRST (commit slots BEFORE adding new samples) ----------
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

  // ---------- Slot rollover ----------
  if (slot != currentSlot) {
    const bool crossedMidnight = (lastDrawnSlot > slot && lastDrawnSlot >= 0);

    // Commit the slot we just finished
    commitSlotData(currentSlot);

#if ENABLE_FS
    if (fsReady) saveDataToFS(crossedMidnight);
#endif

    // Move into the new slot
    currentSlot = slot;

    // Day boundary handling
    if (crossedMidnight) {
      clearData();

      slotsSince12hReset = 0;
      slotsSince24hReset = 0;
      twelveGraphResetOriginSlot     = -1;
      twentyFourGraphResetOriginSlot = -1;
    } else {
      slotsSince12hReset++;
      slotsSince24hReset++;

      if (slotsSince12hReset >= SLOTS_PER_12H) { // 12h @ SLOT_MINUTES bins
        slotsSince12hReset = 0;
        twelveGraphResetOriginSlot = currentSlot;
      }
      if (slotsSince24hReset >= SLOTS_PER_24H) { // 24h @ SLOT_MINUTES bins
        slotsSince24hReset = 0;
        twentyFourGraphResetOriginSlot = currentSlot;
      }
    }

    resetAccumulators();
    lastDrawnSlot = slot;

    // Redraw on slot change
    drawAllScreens();
  }

  // ---------- Sensor read / watchdog ----------
  if (nowMs - lastSensorData > SENSOR_TIMEOUT) {
    initSensor();
  }

  bool gotSample = false;

  if (scd30.dataReady()) {
    if (scd30.read()) {
      lastSensorData = nowMs;

      const float c = scd30.CO2;
      const float t = scd30.temperature;
      const float h = scd30.relative_humidity;

      if (c > 0 && c < 10000 && t > -20 && t < 80 && h >= 0 && h <= 100) {
        currentCO2  = c;
        currentTemp = t;
        currentHum  = h;
        gotSample = true;
      }
    }
  }

  // Accumulate ONLY when we got a real sample
  if (gotSample) {
    co2Sum  += (uint32_t)(currentCO2 + 0.5f);
    tempSum += (uint32_t)(currentTemp * 10.0f + 0.5f);
    humSum  += (uint32_t)(currentHum  * 10.0f + 0.5f);
    sampleCount++;

    writeLiveSlotPreviewToBuffers();
  }

#if ENABLE_FS
  // Periodic save so restart still has data even within the same slot
  if (fsReady && (nowMs - lastPeriodicSaveMs >= PERIODIC_SAVE_MS)) {
    saveDataToFS(false);
    lastPeriodicSaveMs = nowMs;
  }
#endif

  // ---------- Display refresh ----------
  // Fast refresh on 30m screen (lighter)
  if (screenMode == 0 && (nowMs - lastFast30mDraw >= FAST_30M_REFRESH_MS)) {
    lastFast30mDraw = nowMs;
    drawHeader();
    drawTabs();
    draw30MinDashboard();
  }

  // Slower full refresh on other screens
  if (screenMode != 0 && (nowMs - lastRedraw >= REDRAW_INTERVAL)) {
    lastRedraw = nowMs;
    drawAllScreens();
  }

  // Keep WDT happy but stay responsive
  yield();
  delay(10);
}
