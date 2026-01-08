// TD42Tacho V3.6 STABLE (Service Mode + 10 min timeout)
/****************************************************
 * TD42Tacho v3.6  (Tacho + Speed + km + Web UI split)
 *
 * Adds:
 *  - Second VR channel (OUT2) for transfer-case speed sensor
 *  - Speed (km/h), Trip km, Total km (persisted)
 *  - Service reminders by HOURS or KM (per-item selectable)
 *  - Web UI:
 *      "/"      = dashboard (RPM, Hours, km/h, Trip km, Total km)
 *      "/config"= all settings
 *
 * Still includes:
 *  - STA Wi-Fi join + fallback AP
 *  - OLED: TD42Tach header + DUE indicator + segmented RPM bar + hours
 *
 * Wiring (example):
 *  - Cam VR conditioner OUT1  -> GPIO34 (tacho)
 *  - Speed VR conditioner OUT2-> GPIO35 (speed)
 *  - VR board 5V + GND, common ground with ESP32
 *  - OLED SPI: SCK=18, MOSI=23, CS=5, DC=16, RST=17 (VCC=5V, GND common)
 ****************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WebServer.h>

void drawBottom(int rpmVal, float psiVal, bool showRpm, bool showPsi);

// ===== Firmware update manifest =====
struct Manifest {
  String version;
  String binUrl;
  String notes;
};


// Forward declarations
static void loadAll();
static void saveAll();


bool bootFinished = false;
// --- Arduino auto-prototype fix ---
// Arduino auto-generates function prototypes at the top of the file.
// Some prototypes reference ServiceItem before it is defined, so we forward-declare it here.
struct ServiceItem;

/* ================== PINS ================== */
#define PIN_VR_RPM     35   // OUT1 from MAX9926 (cam VR)
#define PIN_VR_SPEED   34   // OUT2 from MAX9926 (t-case speed VR)

// SPI OLED pins (your screen: GND VCC CLK MOS RES DC CS)
#define OLED_SCK   18
#define OLED_MOSI  23
#define OLED_CS     5
#define OLED_DC    16
#define OLED_RST   17

/* ================== OLED ================== */
#define OLED_W 128
#define OLED_H 64
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, OLED_CS, OLED_DC, OLED_RST);

// ---- Boot animation bitmap + functions ----
// ---- Boot gate ----
#define TURBO_W 78
#define TURBO_H 52

const uint8_t turbo_bits[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0xFC, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x00, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0,

  0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0xFF,

  0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0xFC, 0xF3, 0x0F, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xFC, 0xCF, 0x3F, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x00, 0x3C, 0xF0, 0x3F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x3F, 0xC0, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF3,

  0x00, 0xCF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF3, 0x00, 0xFC,

  0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xC0, 0x03, 0xF0, 0xFF, 0x03,

  0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x3F, 0x00, 0xFF, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x0F, 0x00, 0xFC, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0xC0,

  0x03, 0x00, 0xC0, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x03, 0x00,

  0x00, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0xFF,

  0xFF, 0x0F, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0xF0, 0xFF, 0xFF, 0xFF,

  0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0xFC, 0x00, 0xF0, 0xFF, 0x03, 0x00,

  0x00, 0x0F, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F, 0x3F, 0x00, 0xC0, 0x03,

  0x00, 0x00, 0x0F, 0x00, 0x00, 0xF0, 0xFF, 0x00, 0xF0, 0x00, 0x00, 0xC0,

  0x03, 0x00, 0x00, 0xC0, 0xFF, 0x00, 0xF0, 0x00, 0x00, 0xC0, 0x00, 0x00,

  0x00, 0x00, 0xCF, 0x03, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x3C, 0x0F, 0x3C, 0x00, 0x00, 0xF0, 0xFF, 0x00, 0x00, 0x00, 0xF0, 0x0F,

  0x0C, 0x00, 0x00, 0xFF, 0xFF, 0x0F, 0x00, 0x00, 0xC0, 0x0F, 0x0F, 0x00,

  0xF0, 0xFF, 0x03, 0xFF, 0x00, 0x00, 0xC0, 0x3F, 0x0F, 0x00, 0xFC, 0xFF,

  0xFF, 0xF0, 0x03, 0x00, 0x00, 0x3F, 0x0F, 0x00, 0x3F, 0xF0, 0xFF, 0x03,

  0x0F, 0x00, 0x00, 0x3F, 0x0F, 0xC0, 0x03, 0xFC, 0xC3, 0x0F, 0x3C, 0x00,

  0x00, 0x3C, 0x0F, 0xF0, 0x00, 0x0F, 0xCF, 0x3C, 0x30, 0x00, 0x00, 0x3C,

  0x03, 0x30, 0xC0, 0x3F, 0xFC, 0xFF, 0xF0, 0x00, 0x00, 0x3C, 0x03, 0x3C,

  0xC0, 0xFF, 0xFC, 0xCF, 0xC3, 0x00, 0x00, 0x3C, 0x0F, 0x3C, 0xC0, 0xC0,

  0xFF, 0xF3, 0xC3, 0x03, 0x00, 0x3C, 0x0F, 0x0C, 0xC0, 0x00, 0xFF, 0xFF,

  0xCF, 0x03, 0x00, 0x3C, 0x0F, 0x0C, 0xC0, 0xFC, 0xFF, 0x0F, 0xCF, 0x03,

  0x00, 0x30, 0x0F, 0x0C, 0xC0, 0x33, 0xFF, 0xFF, 0xCF, 0x03, 0x00, 0x30,

  0x0C, 0x3C, 0xC0, 0x03, 0xF3, 0xCF, 0xCF, 0x03, 0x00, 0x3C, 0x3C, 0x3C,

  0x00, 0xCF, 0xF3, 0x3C, 0xCF, 0x03, 0x00, 0x3C, 0x3C, 0x3C, 0x00, 0x0F,

  0xF0, 0xFC, 0xFF, 0x03, 0x00, 0x3C, 0xF0, 0x30, 0x00, 0xFC, 0xC0, 0xF0,

  0xFF, 0x00, 0x00, 0x3C, 0xF0, 0xF0, 0x00, 0xF0, 0x0F, 0xFF, 0xFF, 0x00,

  0x00, 0x0F, 0xC0, 0xC3, 0x03, 0x00, 0xFF, 0x0F, 0x3F, 0x00, 0x00, 0x0F,

  0xC0, 0xC3, 0x0F, 0x00, 0x00, 0xC0, 0x0F, 0x00, 0x00, 0x03, 0x00, 0x0F,

  0x3F, 0x00, 0x00, 0xC0, 0x03, 0x00, 0xC0, 0x03, 0x00, 0x3C, 0xFC, 0x00,

  0x00, 0xFC, 0x00, 0x00, 0xF0, 0x00, 0x00, 0xF0, 0xC0, 0x3F, 0xC0, 0x3F,

  0x00, 0x00, 0xFC, 0x00, 0x00, 0xC0, 0x03, 0xFF, 0xFF, 0x03, 0x00, 0x00,

  0x3F, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x0C, 0x00, 0x00, 0xC0, 0x0F, 0x00,

  0x00, 0x00, 0xFC, 0x03, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00,

  0xC0, 0x3F, 0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC,

  0xFF, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,

  0x3F, 0x00, 0x00, 0x00,
};
static void drawTurboGlow(int x, int y) {
  const int W = TURBO_W, H = TURBO_H;
  for (int phase = 0; phase < 2; phase++) {
    for (int i = 0; i <= 10; i++) {
      int s = (phase == 0) ? i : (2 - i);
      u8g2.clearBuffer();
      u8g2.drawXBMP(x-s, y, W, H, turbo_bits);
      u8g2.drawXBMP(x+s, y, W, H, turbo_bits);
      u8g2.drawXBMP(x, y-s, W, H, turbo_bits);
      u8g2.drawXBMP(x, y+s, W, H, turbo_bits);
      u8g2.drawXBMP(x, y,   W, H, turbo_bits);
      u8g2.sendBuffer();
      delay(10); }
  }
}

void bootAnimation() {
  const int W = TURBO_W, H = TURBO_H;

  // Reserve bottom 12px for text, and center turbo in the remaining area
  const int textH = 12;
  const int y = (OLED_H - textH - H) / 2;
  const int targetX = (OLED_W - W) / 2;

  // Turbo slides in from left (quick)
  for (int x = -W; x <= targetX; x += 12) {
    u8g2.clearBuffer();
    u8g2.drawXBMP(x, y, W, H, turbo_bits);
    u8g2.sendBuffer();
    delay(6);
  }

  // Tiny settle
  for (int x = targetX + 2; x >= targetX; x--) {
    u8g2.clearBuffer();
    u8g2.drawXBMP(x, y, W, H, turbo_bits);
    u8g2.sendBuffer();
    delay(4);
  }

  // Fast glow pulse
  drawTurboGlow(targetX, y);

  // Bottom text slides in to MEET in the middle
  u8g2.setFont(u8g2_font_6x12_tf);
  const char* L = "KINGYS A";
  const char* R = "CUNT";
  const int gap = 6;

  const int wL = u8g2.getStrWidth(L);
  const int wR = u8g2.getStrWidth(R);
  const int total = wL + gap + wR;

  const int leftTarget  = (OLED_W - total) / 2;
  const int rightTarget = leftTarget + wL + gap;

  const int yText = OLED_H - 1; // baseline at bottom

  const int frames = 10; // quick
  for (int i = 0; i <= frames; i++) {
    float t = (float)i / (float)frames;

    int lx = (int)lroundf((-wL) + (leftTarget + wL) * t);
    int rx = (int)lroundf((OLED_W) + (rightTarget - OLED_W) * t);

    u8g2.clearBuffer();
    u8g2.drawXBMP(targetX, y, W, H, turbo_bits);
    u8g2.drawStr(lx, yText, L);
    u8g2.drawStr(rx, yText, R);
    u8g2.sendBuffer();
    delay(10);
  }

  // Hold splash/logo frame a bit longer
  delay(2000);

}



static inline int rpmToBarX(int rpm, int barX, int barW, int barMaxRpm) {
  if (barMaxRpm <= 0) return barX;
  float r = (float)rpm / (float)barMaxRpm;
  if (r < 0.0f) r = 0.0f;
  if (r > 1.0f) r = 1.0f;
  return barX + (int)lroundf(r * (float)barW);
}



/* ================== Firmware Update ================== */
struct UpdateCfg {
  String manifestUrl;   // e.g. http(s)://.../manifest.json
} upd;

struct UpdateSta {
  bool wantSta = false;
  String ssid;
  String pass;
  unsigned long startMs = 0;
  bool connected = false;
} updSta;

static bool configMode = true;        // For now WiFi/config is always enabled (testing). Later: 3-press trigger.
static String updateToken = "";       // Access token for /update (set when /config is served)

static String htmlEscape(const String& in) {
  String o; o.reserve(in.length()+8);
  for (size_t i=0;i<in.length();i++){
    char c=in[i];
    if (c=='&') o += "&amp;";
    else if (c=='<') o += "&lt;";
    else if (c=='>') o += "&gt;";
    else if (c=='"') o += "&quot;";
    else o += c;
  }
  return o;
}

static String jsonEscape(const String& in) {
  String o; o.reserve(in.length()+8);
  for (size_t i=0;i<in.length();i++){
    char c=in[i];
    if (c=='\\') o += "\\\\";
    else if (c=='\"') o += "\\\"";
    else if (c=='\n') o += "\\n";
    else if (c=='\r') o += "\\r";
    else if (c=='\t') o += "\\t";
    else o += c;
  }
  return o;
}


static String randomToken() {
  // lightweight token from esp_random
  uint32_t a = esp_random();
  uint32_t b = esp_random();
  char buf[17];
  snprintf(buf, sizeof(buf), "%08lX%08lX", (unsigned long)a, (unsigned long)b);
  return String(buf);
}

static bool parseManifest(const String& json, Manifest& m) {
  // Minimal JSON string extraction without ArduinoJson
  auto getStr = [&](const char* key)->String{
    String k = String("\"") + key + "\"";
    int i = json.indexOf(k);
    if (i < 0) return "";
    i = json.indexOf(':', i);
    if (i < 0) return "";
    i++;
    while (i < (int)json.length() && (json[i]==' '||json[i]=='\n'||json[i]=='\r'||json[i]=='\t')) i++;
    if (i >= (int)json.length() || json[i] != '"') return "";
    i++;
    String out;
    while (i < (int)json.length()) {
      char c = json[i++];
      if (c=='\\') { if (i < (int)json.length()) out += json[i++]; }
      else if (c=='"') break;
      else out += c;
    }
    return out;
  };
  m.version = getStr("version");
  m.binUrl  = getStr("bin_url");
  m.notes   = getStr("notes");
  return (m.version.length() > 0 && m.binUrl.length() > 0);
}

static int versionCmp(const String& a, const String& b) {
  // semver-ish compare X.Y.Z, ignores suffix
  int ai=0, bi=0;
  for (int part=0; part<3; part++) {
    int av=0, bv=0;
    while (ai < (int)a.length() && !isDigit(a[ai])) ai++;
    while (ai < (int)a.length() && isDigit(a[ai])) { av = av*10 + (a[ai]-'0'); ai++; }
    while (bi < (int)b.length() && !isDigit(b[bi])) bi++;
    while (bi < (int)b.length() && isDigit(b[bi])) { bv = bv*10 + (b[bi]-'0'); bi++; }
    if (av < bv) return -1;
    if (av > bv) return  1;
  }
  return 0;
}

static bool httpGetString(const String& url, String& out) {
  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  if (!http.begin(url)) return false;
  int code = http.GET();
  if (code != HTTP_CODE_OK) { http.end(); return false; }
  out = http.getString();
  http.end();
  return true;
}

static bool otaHttpUpdate(const String& binUrl, String& err) {
  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  if (!http.begin(binUrl)) { err = "HTTP begin failed"; return false; }
  int code = http.GET();
  if (code != HTTP_CODE_OK) { err = "HTTP " + String(code); http.end(); return false; }

  int len = http.getSize();
  WiFiClient* stream = http.getStreamPtr();
  if (len <= 0) { err = "No content length"; http.end(); return false; }

  if (!Update.begin((size_t)len)) { err = "Update.begin failed"; http.end(); return false; }

  size_t written = Update.writeStream(*stream);
  if (written != (size_t)len) { err = "Short write"; Update.abort(); http.end(); return false; }

  if (!Update.end()) { err = "Update.end failed"; http.end(); return false; }
  if (!Update.isFinished()) { err = "Not finished"; http.end(); return false; }

  http.end();
  return true;
}

/* ================== OTA (Arduino IDE Network Upload) ================== */
static const char* OTA_HOSTNAME = "TD42Tacho";
static const char* OTA_PASSWORD = ""; // leave empty for no password
static bool otaReady = false;

static void otaSetup() {
  // Only works when STA WiFi is connected
  if (WiFi.status() != WL_CONNECTED) return;

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  if (OTA_PASSWORD && OTA_PASSWORD[0] != '\0') ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    // Optional: could show "OTA..." on display, but leaving UI unchanged per request
    Serial.println("[OTA] Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA] End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    // Keep lightweight to avoid timing issues
    if ((progress % (total/10 + 1)) == 0) Serial.printf("[OTA] %u%%\n", (progress * 100) / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]\n", error);
  });

  ArduinoOTA.begin();
  otaReady = true;
  Serial.printf("[OTA] Ready. Host: %s\n", OTA_HOSTNAME);
}

static const char* FW_VERSION = "3.6.1";

/* ================== SERVICE MODE (Power-cycle trigger) ================== */
// Service mode keeps WiFi + Web UI + update features available for a limited time.
// Normal mode: WiFi OFF for fastest/stablest running.
static bool serviceMode = false;
static uint32_t serviceStartMs = 0;
static bool svcBootResetDone = false;

static const uint32_t SERVICE_TIMEOUT_MS     = 10UL * 60UL * 1000UL; // 10 minutes
static const uint32_t SERVICE_BOOT_WINDOW_MS = 10UL * 1000UL;        // 10 seconds uptime resets the boot counter
static const uint32_t SERVICE_TAP_COUNT      = 4;                    // 4 power cycles to enter service mode

static void serviceModeStop();
static void serviceModeTick();
static void serviceBootCounterInit();
/* ================== WIFI ================== */
// Jimbo: change these later if you want
static const char* WIFI_SSID = ""; // (unused) kept for backward compatibility
static const char* WIFI_PASS = ""; // (unused)

// Fallback AP (so you never lose access)
static const char* AP_SSID = "td42tach";
static const char* AP_PASS = "td42tach42";

static bool staConnecting = false;
static String staLastMsg = "";

// Start (or restart) STA connection while keeping AP alive.
// Fixes: "wifi: sta is connecting, cannot set config" by ensuring we disconnect before setting new config.
static void startStaConnect(const String& ssid, const String& pass) {
  staLastMsg = "";
  // Ensure we are in AP+STA
  WiFi.mode(WIFI_AP_STA);

  // Avoid persistent flash writes + auto reconnect surprises
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // Arduino-ESP32 core v3+: setAutoConnect() removed
  WiFi.setAutoReconnect(false);
#else
  WiFi.setAutoConnect(false);
#endif

  // Stop any in-progress connection BEFORE applying new config
  WiFi.disconnect(true, true);
  delay(250);

  WiFi.begin(ssid.c_str(), pass.c_str());
  staConnecting = true;
  Serial.printf("[NET] STA begin: %s\n", ssid.c_str());
}

// Stop STA and return to AP-only (keep access to the web UI).
static void stopStaConnect() {
  staConnecting = false;
  WiFi.disconnect(true, true);
  delay(50);
  WiFi.mode(WIFI_AP);
}

WebServer server(80);
Preferences prefs;

/* ================== TD42 RPM MATH ==================
   Cam sensor: 1 pulse per CAM revolution
   Engine RPM = Hz * 120
*/
static float engineRpmPerHz = 120.0f; // computed from cam triggers
static int   camTriggersPerCamRev = 1; // 1 = single-tooth cam, 6 = many Nissan wheels
static int   rpmEdgeMode = 0; // 0=FALLING (recommended), 1=RISING

// -------- FACTORY DASH TACH OUT (GU TB45 cluster) --------
// Output is OPEN-COLLECTOR style via an external transistor:
// GPIO -> 1k -> NPN base, NPN emitter -> GND, NPN collector -> dash tacho wire (cluster pull-up ~5-6V)
// IMPORTANT: Do NOT drive the dash wire high directly.

static const int PIN_TACH_OUT = 25;

static bool  dashTachEnabled   = false;
static float dashPulsesPerRev  = 3.0f;   // starting point; tune to match cluster
static int   dashCalPct        = 100;    // 100 = no correction
static bool  dashForceTest     = false;  // Option A: fixed Hz output for testing
static float dashForceHz       = 80.0f;  // fixed Hz when dashForceTest is enabled
static bool  dashHammerMode   = false;   // runtime-only: force output LOW continuously (for wiring test)

// Internal state for Nissan-style open-collector pulse generator
// IMPORTANT for 2N7000 low-side switch:
//   GPIO HIGH  -> MOSFET ON  -> pulls dash line LOW
//   GPIO LOW   -> MOSFET OFF -> dash line floats HIGH via cluster pull-up (~5-6V)
//
// Many Nissan clusters prefer short LOW pulses (open-collector) rather than a 50% duty square wave.
static uint32_t dashNextEdgeUs   = 0;     // time of next state change
static uint32_t dashPeriodUs     = 0;     // full period in microseconds
static uint32_t dashPulseLowUs   = 1500;  // LOW pulse width in microseconds (tunable)
static bool     dashPulseActive  = false; // true while we're holding the line LOW

static inline float dashCalcHzFromRpm(float rpm) {
  if (dashForceTest) return dashForceHz;
  float cal = (float)dashCalPct / 100.0f;
  float hz = (rpm / 60.0f) * dashPulsesPerRev * cal;
  return hz;
}

static void dashTachInit() {
  pinMode(PIN_TACH_OUT, OUTPUT);
  // OFF = MOSFET OFF = line floats HIGH (do not drive high!)
  digitalWrite(PIN_TACH_OUT, LOW);
  dashNextEdgeUs  = micros();
  dashPeriodUs    = 0;
  dashPulseActive = false;
}

// Call every loop. Non-blocking.
static void dashTachService(float rpmNow) {
  if (dashHammerMode) {
    // Force the line LOW continuously (MOSFET ON). Useful to prove wiring/cluster input.
    digitalWrite(PIN_TACH_OUT, HIGH);
    dashPulseActive = false;
    dashNextEdgeUs = micros() + 1000000UL;
    return;
  }
  // Default safe state: MOSFET OFF (line released high by cluster pull-up)
  if (!dashTachEnabled) {
    digitalWrite(PIN_TACH_OUT, LOW);
    dashPeriodUs = 0;
    dashPulseActive = false;
    return;
  }

  float hz = dashCalcHzFromRpm(rpmNow);

  // Stop output below ~2 Hz (prevents "fake idle" / stuck needle)
  if (hz < 2.0f) {
    digitalWrite(PIN_TACH_OUT, LOW);
    dashPeriodUs = 0;
    dashPulseActive = false;
    return;
  }

  // Clamp to sane max
  if (hz > 2000.0f) hz = 2000.0f;

  uint32_t newPeriod = (uint32_t)(1000000.0f / hz);
  if (newPeriod < 200) newPeriod = 200; // safety clamp

  dashPeriodUs = newPeriod;

  // Make sure pulse width never exceeds ~40% of the period (keeps it "short")
  uint32_t pw = dashPulseLowUs;
  uint32_t maxPw = dashPeriodUs / 2; // hard limit
  if (pw < 150) pw = 150;           // avoid too-short pulses
  if (pw > maxPw) pw = maxPw;

  uint32_t now = micros();
  if ((int32_t)(now - dashNextEdgeUs) < 0) return;

  if (!dashPulseActive) {
    // Start LOW pulse: pull line down
    digitalWrite(PIN_TACH_OUT, HIGH);      // MOSFET ON (pull LOW)
    dashPulseActive = true;
    dashNextEdgeUs = now + pw;
  } else {
    // End LOW pulse: release line
    digitalWrite(PIN_TACH_OUT, LOW);       // MOSFET OFF (float HIGH)
    dashPulseActive = false;
    // Schedule next pulse start
    uint32_t rest = (dashPeriodUs > pw) ? (dashPeriodUs - pw) : 1;
    dashNextEdgeUs = now + rest;
  }
}

volatile uint32_t rpmMinPeriodUs = 0; // noise filter (ignore pulses faster than this)


/* ================== FILTERING / TIMEOUT ================== */
static const float RPM_ALPHA = 0.15f;
static const uint32_t RPM_LOSS_TIMEOUT_MS = 800;

static const float SPEED_ALPHA = 0.20f;
static const uint32_t SPEED_LOSS_TIMEOUT_MS = 1200;

/* ================== HOURS ================== */
static const float RUNNING_RPM_THRESHOLD = 400.0f;

/* ================== ISR (RPM) ================== */
volatile uint32_t rpmLastPulseUs = 0;
volatile uint32_t rpmPeriodUs = 0;
volatile uint32_t rpmLastPulseMs = 0;
volatile uint32_t rpmPulseCount = 0;
void IRAM_ATTR isrRpm() {
  uint32_t nowUs = micros();
  uint32_t last  = rpmLastPulseUs;
  if (last == 0) { rpmLastPulseUs = nowUs; rpmLastPulseMs = millis(); return; }
  uint32_t dt = nowUs - last;
  if (rpmMinPeriodUs && dt < rpmMinPeriodUs) return; // ignore ringing/double-triggers
  rpmPeriodUs = dt;
  rpmPulseCount++;
  rpmLastPulseUs = nowUs;
  rpmLastPulseMs = millis();
}

/* ================== ISR (SPEED pulse counter) ================== */
volatile uint32_t speedPulseCount = 0;
volatile uint32_t speedLastPulseMs = 0;

void IRAM_ATTR isrSpeed() {
  speedPulseCount++;
  speedLastPulseMs = millis();
}

/* ================== RUNTIME VALUES ================== */
float rpmNow = 0.0f;
float rpmFiltered = 0.0f;

float speedNowKmh = 0.0f;
float speedFilteredKmh = 0.0f;

/* ================== STATS ================== */
uint32_t engineSeconds = 0;    // persisted
float maxRpmTrip = 0.0f;       // resets at boot
float maxRpmAll  = 0.0f;       // persisted

double totalKm = 0.0;          // persisted
double tripKm  = 0.0;          // resets at boot (or via button)

/* ================== SPEED CALIB ==================
   Use pulsesPerKm for easy calibration:
     km/h = (pulses/sec) * 3600 / pulsesPerKm
   Also distance accumulates by pulses:
     km += pulses / pulsesPerKm
*/
struct SpeedCfg {
  double pulsesPerKm = 8000.0;   // placeholder default - set via /config
};
SpeedCfg spd;

/* ================== SERVICE REMINDERS ================== */
enum ServiceMode : uint8_t { MODE_HOURS = 0, MODE_KM = 1 };

struct ServiceItem {
  // interval targets
  float intervalHours = 0.0f;   // 0 disables if mode=hours
  float intervalKm    = 0.0f;   // 0 disables if mode=km
  ServiceMode mode    = MODE_HOURS;

  // last serviced marks
  uint32_t lastSec = 0;
  double   lastKm  = 0.0;
};

struct ServiceCfg {
  ServiceItem oil;
  ServiceItem fuel;
  ServiceItem air;
  ServiceItem custom;
  String customName;
};
ServiceCfg svc;
/* ================== DIAGNOSTICS ================== */
// Web-configurable test mode to simulate RPM and force service-due messages.
// Helpful for verifying display layout and flashing behavior without sensors wired.
struct DiagCfg {
  bool enabled = false;        // master enable
  bool rpmOverride = false;    // override live RPM with rpmValue
  int  rpmValue = 0;

  bool sweep = false;          // sweep RPM automatically
  int  sweepMin = 0;
  int  sweepMax = 4500;
  int  sweepStep = 100;
  int  sweepDwellMs = 250;
  bool sweepUp = true;
  uint32_t lastSweepMs = 0;

  bool forceDue = false;       // force service due display
  uint8_t dueMask = 0;         // bit0=oil bit1=fuel bit2=air bit3=custom

  // Boost sweep (diagnostic)
  bool boostSweepEnabled = false;
  float boostSweepPsi = 0.0f;
  bool boostSweepUp = true;
  uint32_t lastBoostSweepMs = 0;

};
DiagCfg diag;



/* ================== UI SETTINGS ================== */
struct UiCfg {
  // 0=RPM bar, 1=BOOST bar
  int graphMode = 0;

  int barMinRpm = 400;
  int barMaxRpm = 4500;
  int segments  = 32;     // 16..64
  int barY      = 14;
  int barH      = 14;

  int torqueStart = 1000;
  int torqueEnd   = 2500;
  int flashRpm    = 3400;
  int redlineRpm  = 4000;
};
UiCfg ui;

// Boost settings (0.45-5.0V sensors MUST use a divider so ADC <= 3.3V)
struct BoostCfg {
  int   adcPin = 34;           // GPIO34 (ADC1) Wi-Fi safe
  float vZero  = 0.45f;        // sensor volts at 0 psi
  float vMax   = 5.00f;        // sensor volts at max psi
  float maxPsi = 30.0f;        // display range
  float flashPsi = 0.0f;      // PSI threshold to flash (0 = off)
  float divider = 0.66f;       // ADC/Sensor ratio (3.3/5.0 ~ 0.66)
  float smoothAlpha = 0.15f;   // EMA smoothing
};
BoostCfg boost;

static float boostPsi = 0.0f;


/* ================== HELPERS ================== */
static inline float secToHours(uint32_t s) { return (float)s / 3600.0f; }

static int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}


/* ================== RPM INPUT SCALING ==================
   camTriggersPerCamRev = number of conditioner output pulses per CAM revolution.
   Engine RPM = Hz * (120 / camTriggersPerCamRev)
*/
static void recomputeRpmScaling() {
  camTriggersPerCamRev = clampInt(camTriggersPerCamRev, 1, 60);
  engineRpmPerHz = 120.0f / (float)camTriggersPerCamRev;

  // Reject unrealistically fast pulses (helps with ringing/double-trigger noise).
  // Based on a plausible max engine rpm.
  const float RPM_MAX_PLAUSIBLE = 12000.0f;
  float maxHz = RPM_MAX_PLAUSIBLE / engineRpmPerHz;
  if (maxHz < 1.0f) maxHz = 1.0f;
  uint32_t minPer = (uint32_t)(1000000.0f / maxHz);
  if (minPer < 50) minPer = 50; // don't go silly
  rpmMinPeriodUs = minPer;
}

/* ================== BOOST ================== */
static float readBoostPsiRaw() {
  int raw = analogRead(boost.adcPin);
  float vAdc = (raw / 4095.0f) * 3.3f;
  float vSens = (boost.divider > 0.01f) ? (vAdc / boost.divider) : vAdc;

  float denom = (boost.vMax - boost.vZero);
  float psi = 0.0f;
  if (denom > 0.001f) psi = (vSens - boost.vZero) * (boost.maxPsi / denom);

  if (psi < 0) psi = 0;
  if (psi > boost.maxPsi) psi = boost.maxPsi;
  return psi;
}

static void updateBoost() {
  float psi = readBoostPsiRaw();
  float a = boost.smoothAlpha;
  if (a < 0.0f) a = 0.0f;
  if (a > 1.0f) a = 1.0f;
  boostPsi = (a <= 0.0f) ? psi : (boostPsi + a * (psi - boostPsi));
}

static void diagUpdateBoostSweep() {
  if (!diag.boostSweepEnabled) return;
  uint32_t now = millis();
  if (now - diag.lastBoostSweepMs < (uint32_t)diag.sweepDwellMs) return;
  diag.lastBoostSweepMs = now;

  float step = boost.maxPsi / 20.0f;
  if (step < 0.5f) step = 0.5f;

  if (diag.boostSweepUp) {
    diag.boostSweepPsi += step;
    if (diag.boostSweepPsi >= boost.maxPsi) { diag.boostSweepPsi = boost.maxPsi; diag.boostSweepUp = false; }
  } else {
    diag.boostSweepPsi -= step;
    if (diag.boostSweepPsi <= 0.0f) { diag.boostSweepPsi = 0.0f; diag.boostSweepUp = true; }
  }
}

static void drawBoostBar(float psiVal) {
  int y = ui.barY;
  int h = ui.barH;

  u8g2.drawFrame(0, y, OLED_W, h);

  int innerX = 1;
  int innerY = y + 1;
  int innerW = OLED_W - 2;
  int innerH = h - 2;

  int segs = ui.segments;
  if (segs < 1) segs = 1;

  float t = psiVal / (boost.maxPsi > 0.01f ? boost.maxPsi : 1.0f);
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  int filled = (int)roundf(t * segs);

  int gap = 1;
  int totalSegPixels = innerW - (segs - 1) * gap;
  if (totalSegPixels < segs) { gap = 0; totalSegPixels = innerW; }
  int baseW = totalSegPixels / segs;
  int remW  = totalSegPixels % segs;

  int x = innerX;
  for (int i = 0; i < segs; i++) {
    int w = baseW + (i < remW ? 1 : 0);
    if (i < filled) u8g2.drawBox(x, innerY, w, innerH);
    x += w + gap;
    if (x > innerX + innerW) break;
  }
}

static void drawBoostLabels() {
  u8g2.setFont(u8g2_font_5x8_tf);
  int y = ui.barY - 2;
  if (y < 8) y = 8;

  int innerX = 1;
  int innerW = OLED_W - 2;

  int maxP = (int)roundf(boost.maxPsi);
  if (maxP < 5) maxP = 5;
  if (maxP > 60) maxP = 60;

  const int step = 5;
  for (int p = 0; p <= maxP; p += step) {
    float t = (float)p / (float)maxP;
    int x = innerX + (int)roundf(t * innerW);

    char buf[6];
    snprintf(buf, sizeof(buf), "%d", p);
    int w = u8g2.getStrWidth(buf);
    int tx = x - (w / 2);
    if (tx < 0) tx = 0;
    if (tx + w > OLED_W) tx = OLED_W - w;

    u8g2.drawStr(tx, y, buf);
    u8g2.drawVLine(x, y + 1, 3);
  }
}

static float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
 return v;
}
static double clampDouble(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static bool dueForced(uint8_t bit) {
  return diag.enabled && diag.forceDue && ((diag.dueMask & (1 << bit)) != 0);
}

static bool serviceDueEffective(const ServiceItem& it, uint8_t bit) {
  if (dueForced(bit)) return true;
  return serviceDue(it);
}

static float rpmForUi() {
  if (diag.enabled && diag.rpmOverride) return (float)diag.rpmValue;
  return rpmFiltered;
}


static int rpmToX(int rpm) {
  int minR = ui.barMinRpm;
  int maxR = ui.barMaxRpm;
  if (maxR <= minR) maxR = minR + 1;
  long num = (long)(rpm - minR) * 126L;
  long den = (long)(maxR - minR);
  long x = 1 + (num / den);
  if (x < 1) x = 1;
  if (x > 126) x = 126;
  return (int)x;
}

static bool serviceDue(const ServiceItem& it) {
  if (it.mode == MODE_HOURS) {
    if (it.intervalHours <= 0.0f) return false;
    float sinceH = secToHours(engineSeconds - it.lastSec);
    return sinceH >= it.intervalHours;
  } else {
    if (it.intervalKm <= 0.0f) return false;
    double sinceKm = totalKm - it.lastKm;
    return sinceKm >= it.intervalKm;
  }
}

static bool anyServiceDue() {
  return serviceDueEffective(svc.oil, 0) || serviceDueEffective(svc.fuel, 1) || serviceDueEffective(svc.air, 2) || serviceDueEffective(svc.custom, 3);
}



// Build a short list of "DUE" messages for the OLED header.
// Cycles through them every 2 seconds when one or more are due.
static int buildDueList(String out[], int maxItems) {
  int n = 0;
  if (serviceDueEffective(svc.oil, 0)  && n < maxItems) out[n++] = "OIL DUE";
  if (serviceDueEffective(svc.fuel, 1) && n < maxItems) out[n++] = "FUEL DUE";
  if (serviceDueEffective(svc.air, 2)  && n < maxItems) out[n++] = "AIR DUE";
  if (serviceDueEffective(svc.custom, 3) && n < maxItems) {
    String nm = svc.customName;
    nm.trim();
    if (nm.length() == 0) nm = "CUSTOM";
    nm.toUpperCase();
    if (nm.length() > 10) nm = nm.substring(0, 10); // keep readable
    out[n++] = nm + " DUE";
  }
  return n;
}

static String modeStr(uint8_t m) {
  return (m == (uint8_t)MODE_KM) ? "km" : "hours";
}

/* ================== PERSISTENCE ================== */
void loadAll() {
  prefs.begin("td42tach", true);

  diag.boostSweepEnabled = prefs.getBool("bs", diag.boostSweepEnabled);

  // RPM input settings
  camTriggersPerCamRev = prefs.getInt("camT", camTriggersPerCamRev);
  rpmEdgeMode          = prefs.getInt("edge", rpmEdgeMode);

  // Factory dash tach out
  dashTachEnabled  = prefs.getBool("dt_en", dashTachEnabled);
  dashPulsesPerRev = prefs.getFloat("dt_ppr", dashPulsesPerRev);
  dashCalPct       = prefs.getInt("dt_cal", dashCalPct);
  dashForceTest    = prefs.getBool("dt_ft", dashForceTest);
  dashForceHz      = prefs.getFloat("dt_fh", dashForceHz);
  dashPulseLowUs   = (uint32_t)prefs.getInt("dt_pw", (int)dashPulseLowUs);


  ui.graphMode = prefs.getInt("gm", ui.graphMode);
  boost.vZero = prefs.getFloat("bz_v0", boost.vZero);
  boost.vMax  = prefs.getFloat("bz_vm", boost.vMax);
  boost.maxPsi = prefs.getFloat("bz_mp", boost.maxPsi);
  boost.flashPsi = prefs.getFloat("bz_fl", boost.flashPsi);
  boost.divider = prefs.getFloat("bz_div", boost.divider);
  boost.smoothAlpha = prefs.getFloat("bz_al", boost.smoothAlpha);
  // Firmware update settings
  upd.manifestUrl = prefs.getString("mf_url", "");
  engineSeconds = prefs.getUInt("hrs_s", 0);
  maxRpmAll     = prefs.getFloat("max_all", 0.0f);

  totalKm       = prefs.getDouble("tot_km", 0.0);

  spd.pulsesPerKm = prefs.getDouble("ppkm", 8000.0);

  // UI
  ui.barMinRpm   = prefs.getInt("barMin", 400);
  ui.barMaxRpm   = prefs.getInt("barMax", 4500);
  ui.segments    = prefs.getInt("segs", 32);
  ui.barH        = prefs.getInt("barH", 14);
  ui.torqueStart = prefs.getInt("tqS", 1000);
  ui.torqueEnd   = prefs.getInt("tqE", 2500);
  ui.flashRpm    = prefs.getInt("flash", 3400);
  ui.redlineRpm  = prefs.getInt("red", 4000);

  // Services (oil/fuel/air)
  svc.oil.intervalHours = prefs.getFloat("oil_h", 100.0f);
  svc.oil.intervalKm    = prefs.getFloat("oil_k", 5000.0f);
  svc.oil.mode          = (ServiceMode)prefs.getUChar("oil_m", (uint8_t)MODE_HOURS);
  svc.oil.lastSec       = prefs.getUInt("oil_ls", 0);
  svc.oil.lastKm        = prefs.getDouble("oil_lk", 0.0);

  svc.fuel.intervalHours= prefs.getFloat("ful_h", 250.0f);
  svc.fuel.intervalKm   = prefs.getFloat("ful_k", 20000.0f);
  svc.fuel.mode         = (ServiceMode)prefs.getUChar("ful_m", (uint8_t)MODE_HOURS);
  svc.fuel.lastSec      = prefs.getUInt("ful_ls", 0);
  svc.fuel.lastKm       = prefs.getDouble("ful_lk", 0.0);

  svc.air.intervalHours = prefs.getFloat("air_h", 200.0f);
  svc.air.intervalKm    = prefs.getFloat("air_k", 15000.0f);
  svc.air.mode          = (ServiceMode)prefs.getUChar("air_m", (uint8_t)MODE_HOURS);
  svc.air.lastSec       = prefs.getUInt("air_ls", 0);
  svc.air.lastKm        = prefs.getDouble("air_lk", 0.0);

  
// Custom service
  svc.customName      = prefs.getString("cus_name", "Custom");
  svc.custom.intervalHours = prefs.getFloat("cus_h", 0.0f);
  svc.custom.intervalKm    = prefs.getFloat("cus_k", 0.0f);
  svc.custom.mode          = (ServiceMode)prefs.getUChar("cus_m", (uint8_t)MODE_KM);
  svc.custom.lastSec       = prefs.getUInt("cus_ls", 0);
  svc.custom.lastKm        = prefs.getDouble("cus_lk", 0.0);
prefs.end();
  recomputeRpmScaling();


  // sanity
  ui.segments = clampInt(ui.segments, 16, 64);
  ui.barH     = clampInt(ui.barH, 10, 20);
  ui.barMinRpm= clampInt(ui.barMinRpm, 0, 9000);
  ui.barMaxRpm= clampInt(ui.barMaxRpm, ui.barMinRpm + 1, 9000);

  ui.torqueStart = clampInt(ui.torqueStart, 0, 9000);
  ui.torqueEnd   = clampInt(ui.torqueEnd, 0, 9000);
  ui.flashRpm    = clampInt(ui.flashRpm, 0, 9000);
  ui.redlineRpm  = clampInt(ui.redlineRpm, 0, 9000);

  spd.pulsesPerKm = clampDouble(spd.pulsesPerKm, 1.0, 1e9);

  if (svc.customName.length() == 0) svc.customName = "Custom";
}

void saveHours() {
  prefs.begin("td42tach", false);
  prefs.putUInt("hrs_s", engineSeconds);
  prefs.end();
}

void saveMaxRpm() {
  prefs.begin("td42tach", false);
  prefs.putFloat("max_all", maxRpmAll);
  prefs.end();
}

void saveKm() {
  prefs.begin("td42tach", false);
  prefs.putDouble("tot_km", totalKm);
  prefs.end();
}

void saveSpeedCfg() {
  prefs.begin("td42tach", false);
  prefs.putDouble("ppkm", spd.pulsesPerKm);
  prefs.end();
}

void saveDashCfg() {
  prefs.begin("td42tach", false);
  prefs.putBool("dt_en", dashTachEnabled);
  prefs.putFloat("dt_ppr", dashPulsesPerRev);
  prefs.putInt("dt_cal", dashCalPct);
  prefs.putBool("dt_ft", dashForceTest);
  prefs.putFloat("dt_fh", dashForceHz);
  prefs.putInt("dt_pw", (int)dashPulseLowUs);
  prefs.end();
}


void saveUi() {
  prefs.begin("td42tach", false);
  prefs.putInt("barMin", ui.barMinRpm);
  prefs.putInt("barMax", ui.barMaxRpm);
  prefs.putInt("segs", ui.segments);
  prefs.putInt("barH", ui.barH);
  prefs.putInt("tqS", ui.torqueStart);
  prefs.putInt("tqE", ui.torqueEnd);
  prefs.putInt("flash", ui.flashRpm);
  prefs.putInt("red", ui.redlineRpm);
  prefs.putInt("camT", camTriggersPerCamRev);
  prefs.putInt("edge", rpmEdgeMode);

  // Factory dash tach out
  prefs.putBool("dt_en", dashTachEnabled);
  prefs.putFloat("dt_ppr", dashPulsesPerRev);
  prefs.putInt("dt_cal", dashCalPct);
  prefs.putBool("dt_ft", dashForceTest);
  prefs.putFloat("dt_fh", dashForceHz);

  prefs.putInt("gm", ui.graphMode);
  prefs.putFloat("bz_v0", boost.vZero);
  prefs.putFloat("bz_vm", boost.vMax);
  prefs.putFloat("bz_mp", boost.maxPsi);
  prefs.putFloat("bz_fl", boost.flashPsi);
  prefs.putFloat("bz_div", boost.divider);
  prefs.putFloat("bz_al", boost.smoothAlpha);
  prefs.end();
}

void saveAll() {
  // Save everything (called from update/config pages)
  saveHours();
  saveMaxRpm();
  saveKm();
  saveSpeedCfg();

  // Service items
  saveService("oil",  svc.oil);
  saveService("fuel", svc.fuel);
  saveService("air",  svc.air);
  saveService("cus",  svc.custom);
  saveCustomName();

  // UI + misc
  saveUi();

  // Firmware update settings
  prefs.begin("td42tach", false);
  prefs.putString("mf_url", upd.manifestUrl);
  prefs.end();
}


void saveService(const char* prefix, const ServiceItem& it) {
  prefs.begin("td42tach", false);
  String p(prefix);

  prefs.putFloat((p + "_h").c_str(), it.intervalHours);
  prefs.putFloat((p + "_k").c_str(), it.intervalKm);
  prefs.putUChar((p + "_m").c_str(), (uint8_t)it.mode);
  prefs.putUInt((p + "_ls").c_str(), it.lastSec);
  prefs.putDouble((p + "_lk").c_str(), it.lastKm);

  prefs.end();
}


void saveCustomName() {
  prefs.begin("td42tach", false);
  prefs.putString("cus_name", svc.customName);
  prefs.end();
}


/* ================== WEB UI HTML HELPERS ================== */
static String htmlHeader(const String& title) {
  String s;
  s += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  s += "<title>" + title + "</title>";
  s += "<style>"
       "body{font-family:Arial;margin:16px}"
       "a{color:#06c;text-decoration:none}"
       ".topbar{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px}"
       ".grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}"
       ".card{border:1px solid #ccc;border-radius:14px;padding:12px}"
       ".big{font-size:28px;font-weight:700}"
       ".label{color:#666;font-size:12px;text-transform:uppercase;letter-spacing:.06em}"
       "button{padding:10px 12px;border-radius:10px;border:1px solid #333;background:#eee;margin-top:8px}"
       "input,select{padding:8px;width:160px}"
       ".row{margin:10px 0}"
       ".warn{color:#b00;font-weight:700}"
       "small{color:#666}"
       "</style></head><body>";
  return s;
}
static String htmlFooter() { return "</body></html>"; }

static String fmt1(double v) { char b[32]; snprintf(b, sizeof(b), "%.1f", v); return String(b); }
static String fmt2(double v) { char b[32]; snprintf(b, sizeof(b), "%.2f", v); return String(b); }

static String serviceBlock(const String& name, const String& key, const ServiceItem& it, bool nameEditable, const String& currentName) {
  bool due = serviceDue(it);
  String displayName = nameEditable ? currentName : name;
  String s;
  s += "<div class='card'>";
  s += "<b>" + displayName + "</b> ";
  s += due ? "<span class='warn'>DUE</span>" : "<small>OK</small>";
  s += "<div class='row'>Mode: <b>" + modeStr(it.mode) + "</b></div>";

  s += "<form action='/setService' method='post'>";

  if (nameEditable) {
    s += "<div class='row'>Name: <input name='cname' type='text' value='" + currentName + "'></div>";
  }

  s += "<input type='hidden' name='key' value='" + key + "'>";

  s += "<div class='row'>Mode: "
       "<select name='mode'>"
       "<option value='hours'" + String(it.mode==MODE_HOURS ? " selected":"") + ">Hours</option>"
       "<option value='km'"    + String(it.mode==MODE_KM    ? " selected":"") + ">KM</option>"
       "</select></div>";

  s += "<div class='row'>Interval Hours: <input name='ih' type='number' step='0.1' value='" + String(it.intervalHours,1) + "'></div>";
  s += "<div class='row'>Interval KM: <input name='ik' type='number' step='0.1' value='" + String(it.intervalKm,1) + "'></div>";
  s += "<button type='submit'>Save</button></form>";

  s += "<form action='/resetService' method='post'>";
  s += "<input type='hidden' name='key' value='" + key + "'>";
  s += "<button type='submit'>Reset Service Now</button></form>";

  s += "<small>Tip: set interval to 0 to disable for that mode.</small>";
  s += "</div>";
  return s;
}

/* ================== WEB ROUTES ================== */
void routeDashboard() {
  String s = htmlHeader("TD42Tacho v3.6.2");
  s += "<div class='topbar'><div><h2 style='margin:0'>TD42Tacho v3.6.2</h2>";
  s += anyServiceDue() ? "<div class='warn'>SERVICE DUE</div>" : "<div><small>All good</small></div>";
  s += "</div><div><a href='/config'>Config</a></div></div>";

  s += "<div class='grid'>";

  s += "<div class='card'><div class='label'>RPM</div><div class='big'>" + String((int)rpmFiltered) + "</div>"
       "<small>Trip Max: " + String((int)maxRpmTrip) + " | All: " + String((int)maxRpmAll) + "</small></div>";

  s += "<div class='card'><div class='label'>Speed</div><div class='big'>" + fmt1(speedFilteredKmh) + " km/h</div>"
       "<small>pulses/km: " + fmt1(spd.pulsesPerKm) + "</small></div>";

  s += "<div class='card'><div class='label'>Engine Hours</div><div class='big'>" + fmt1(secToHours(engineSeconds)) + "</div>" "<small>counts only above " + String((int)RUNNING_RPM_THRESHOLD) + " rpm</small>" "<div style='margin-top:8px;display:flex;gap:8px;flex-wrap:wrap;align-items:center'>" "<form action='/setHours' method='post' style='display:flex;gap:6px;align-items:center;margin:0'>" "<input name='hrs' type='number' step='0.1' min='0' value='" + fmt1(secToHours(engineSeconds)) + "' style='width:110px'>" "<button type='submit'>Set</button></form>" "<form action='/resetHours' method='post' style='margin:0'><button type='submit'>Reset</button></form>" "</div></div>";

  s += "<div class='card'><div class='label'>Distance</div><div class='big'>" + fmt2(tripKm) + " km</div>"
       "<small>Total: " + fmt2(totalKm) + " km</small></div>";


  s += "<div class='card'><div class='label'>Signal</div>"
       "<div><b>RPM:</b> <span id='sigR'>...</span> <small>(<span id='sigRms'>-</span> ms)</small></div>"
       "<div><b>SPD:</b> <span id='sigS'>...</span> <small>(<span id='sigSms'>-</span> ms)</small></div>"
       "<small>RPM pulses/s: <span id='sigRpps'>-</span> | SPD pulses/s: <span id='sigSpps'>-</span></small>"
       "</div>";

  s += "<script>"
       "let _lr=0,_ls=0,_lt=Date.now();"
       "async function _pollSig(){"
       " try{"
       "  const r=await fetch('/sig',{cache:'no-store'});"
       "  const j=await r.json();"
       "  const now=Date.now();"
       "  const dt=(now-_lt)/1000.0;"
       "  const rAge=j.rpmAge; const sAge=j.spdAge;"
       "  const rOk=(rAge>=0 && rAge<800);"
       "  const sOk=(sAge>=0 && sAge<1200);"
       "  document.getElementById('sigR').textContent = rOk ? 'OK' : (rAge<0?'NEVER':'LOST');"
       "  document.getElementById('sigS').textContent = sOk ? 'OK' : (sAge<0?'NEVER':'LOST');"
       "  document.getElementById('sigRms').textContent = (rAge<0?'-':String(rAge));"
       "  document.getElementById('sigSms').textContent = (sAge<0?'-':String(sAge));"
       "  if(_lr!==0 && dt>0){ document.getElementById('sigRpps').textContent=((j.rpmCount-_lr)/dt).toFixed(1); }"
       "  if(_ls!==0 && dt>0){ document.getElementById('sigSpps').textContent=((j.spdCount-_ls)/dt).toFixed(1); }"
       "  _lr=j.rpmCount; _ls=j.spdCount; _lt=now;"
       " }catch(e){}"
       " setTimeout(_pollSig,1000);"
       "}"
       "_pollSig();"
       "</script>";

  s += "</div>";

  s += "<div class='card' style='margin-top:12px'>";
  s += "<form action='/resetTripRpm' method='post'><button type='submit'>Reset Trip Max RPM</button></form>";
  s += "<form action='/resetTripKm' method='post'><button type='submit'>Reset Trip KM</button></form>";
  s += "</div>";

  
  // Dash hammer quick control (wiring test)
  s += "<div class='card'><b>Dash Tach Output Test</b><br><small>HAMMER forces the output LOW continuously (runtime only). Use to confirm 2N7000 wiring.</small><hr>";
  s += "<div class='row'><form method='post' action='/dashHammer' style='display:inline-block;margin-right:8px'>"
       "<input type='hidden' name='on' value='1'><button class='btn' type='submit'>HAMMER ON</button></form>"
       "<form method='post' action='/dashHammer' style='display:inline-block'>"
       "<input type='hidden' name='off' value='1'><button class='btn' type='submit'>HAMMER OFF</button></form>"
       "<span style='margin-left:10px'>State: <b>" + String(dashHammerMode ? "ON" : "OFF") + "</b></span></div>";
  s += "<div class='row'><small>If HAMMER ON does not pull the dash line to ~0V, the MOSFET pins/ground/wire are wrong.</small></div>";
  s += "</div>";
s += htmlFooter();
  server.send(200, "text/html", s);
}


/* ================== Update Web Handlers ================== */
static bool allowUpdatePage() {
  // Token gate: Updates page can only be opened via Config (which generates updateToken)
  if (updateToken.length() == 0) return false;
  if (!server.hasArg("t")) return false;
  return server.arg("t") == updateToken;
}

static String pageHeader(const String& title) {
  String s;
  s += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  s += "<title>"+title+"</title>";
  s += "<style>body{font-family:system-ui,Segoe UI,Arial;margin:16px;max-width:900px}a{color:#06c}input,button{font-size:16px;padding:8px}fieldset{border:1px solid #ccc;padding:12px;border-radius:10px;margin:12px 0}legend{padding:0 8px}.row{margin:8px 0}.muted{color:#666}.ok{color:#080}.warn{color:#b60}.err{color:#b00}.mono{font-family:ui-monospace,Menlo,Consolas,monospace}</style>";
  s += "</head><body>";
  s += "<div class='row'><a href='/config'><- Back to Config</a></div>";
  s += "<h2>"+title+"</h2>";
  return s;
}

static void routeUpdate() {
  if (!allowUpdatePage()) { server.send(403, "text/plain", "Updates only available from Config page."); return; }

  String t = server.arg("t");
  String s = pageHeader("Firmware Updates");

    // Auto-refresh while connecting so status updates even if the browser blocks JS polling
  if (updSta.wantSta && WiFi.status() != WL_CONNECTED) {
    String refresh = "<meta http-equiv='refresh' content='2;url=/update?t=" + t + "&r=" + String(millis()) + "'>";
    int p = s.indexOf("<style>");
    if (p > 0) s = s.substring(0, p) + refresh + s.substring(p);
  }

s += "<fieldset><legend>Manifest</legend>";
  s += "<form method='POST' action='/update/save?t="+t+"'>";
  s += "<div class='row'><label>Manifest URL</label><br><input style='width:100%' name='mf' value='"+htmlEscape(upd.manifestUrl)+"' placeholder='https://.../manifest.json'></div>";
  s += "<div class='row muted'>Manifest JSON fields: <span class='mono'>version</span>, <span class='mono'>bin_url</span>, optional <span class='mono'>notes</span></div>";
  s += "<button type='submit'>Save</button></form></fieldset>";

  s += "<fieldset><legend>Local Upload</legend>";
  s += "<form method='POST' action='/update/upload?t="+t+"' enctype='multipart/form-data'>";
  s += "<div class='row'><input type='file' name='fw' accept='.bin'></div>";
  s += "<button type='submit'>Upload & Flash</button>";
  s += "<div class='row muted'>Upload a compiled ESP32 firmware .bin</div>";
  s += "</form></fieldset>";

  s += "<fieldset><legend>Online Update (Temporary Wi-Fi)</legend>";
  s += "<form method='POST' action='/update/sta?t="+t+"'>";
  s += "<div class='row'><label>SSID</label><br><input name='ssid' style='width:100%'></div>";
  s += "<div class='row'><label>Password</label><br><input name='pass' type='password' style='width:100%'></div>";
  s += "<button type='submit'>Connect for Update</button>";
  s += "</form>";

  // status (live)
  s += "<div class='row'><b>Status:</b> <span id='staStatus'>";
  if (WiFi.status() == WL_CONNECTED) {
    s += "<span class='ok'>Connected</span> to <span class='mono'>"+htmlEscape(WiFi.SSID())+"</span>";
  } else if (updSta.wantSta) {
    s += "<span class='warn'>Connecting...</span>";
  } else if (staLastMsg.length()) {
    s += "<span class='err'>" + htmlEscape(staLastMsg) + "</span>";
  } else {
    s += "<span class='muted'>STA idle</span>";
  }
  s += "</span><div id='staDetails' class='muted mono' style='margin-top:6px'>";
  if (WiFi.status() == WL_CONNECTED) {
    s += "IP "+WiFi.localIP().toString()+" , RSSI "+String(WiFi.RSSI())+"dBm";
  }
  s += "</div></div>";

  // Live status polling so the page updates without refresh
  s += "<script>"
       "function _eh(s){return (s||'').replace(/[&<>\"']/g,function(c){return {'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;','\'':'&#039;'}[c];});}"
       "async function _poll(){try{const r=await fetch('/update/status?_='+Date.now(),{cache:'no-store'});const j=await r.json();"
       "const st=document.getElementById('staStatus');const dt=document.getElementById('staDetails');if(!st||!dt)return;"
       "if(j.connected){st.innerHTML=\"<span class='ok'>Connected</span> to <span class='mono'>\"+_eh(j.ssid)+\"</span>\";"
       "dt.textContent='IP '+j.ip+' , RSSI '+j.rssi+'dBm';if(window.__staT){clearInterval(window.__staT);window.__staT=null;}}"
       "else if(j.wantSta){st.innerHTML=\"<span class='warn'>Connecting...</span>\";dt.textContent='';}"
       "else if(j.msg&&j.msg.length){st.innerHTML=\"<span class='err'>\"+_eh(j.msg)+\"</span>\";dt.textContent='';if(window.__staT){clearInterval(window.__staT);window.__staT=null;}}"
       "else{st.innerHTML=\"<span class='muted'>STA idle</span>\";dt.textContent='';}}catch(e){}}"
       "window.__staT=window.__staT||setInterval(_poll,1000);_poll();"
       "</script>";

  s += "<div class='row'><form method='POST' action='/update/check?t="+t+"'><button type='submit'>Check for Update</button></form></div>";
  s += "<div class='row'><form method='POST' action='/update/do?t="+t+"'><button type='submit'>Download & Install</button></form></div>";
  s += "<div class='row'><form method='POST' action='/update/disc?t="+t+"'><button type='submit'>Disconnect STA</button></form></div>";
  s += "</fieldset>";

  s += "<fieldset><legend>Current</legend>";
  s += "<div class='row'>Firmware version: <span class='mono'>"+String(FW_VERSION)+"</span></div>";
  s += "</fieldset>";

  s += "</body></html>";
  server.send(200, "text/html", s);
}

static void routeUpdateSave() {
  if (!allowUpdatePage()) { server.send(403, "text/plain", "Forbidden"); return; }
  if (server.hasArg("mf")) upd.manifestUrl = server.arg("mf");
  saveAll();
  server.sendHeader("Location", String("/update?t=")+server.arg("t"));
  server.send(302, "text/plain", "");
}

static void routeUpdateSta() {
  if (!allowUpdatePage()) { server.send(403, "text/plain", "Forbidden"); return; }
  updSta.ssid = server.arg("ssid");
  updSta.pass = server.arg("pass");
  updSta.ssid.trim();
  updSta.pass.trim();
  updSta.wantSta = (updSta.ssid.length() > 0);
  updSta.startMs = millis();
  updSta.connected = false;

  if (updSta.wantSta) {
    startStaConnect(updSta.ssid, updSta.pass);
  }
  server.sendHeader("Location", String("/update?t=")+server.arg("t"));
  server.send(303, "text/plain", "");
}

static void routeUpdateDisc() {
  if (!allowUpdatePage()) { server.send(403, "text/plain", "Forbidden"); return; }
  updSta.wantSta = false;
  updSta.connected = false;
  stopStaConnect();
  // (Your AP start code elsewhere will keep it available)
  server.sendHeader("Location", String("/update?t=")+server.arg("t"));
  server.send(302, "text/plain", "");
}

static Manifest lastMf;
static String lastCheckMsg = "";

void routeUpdateStatus() {
  // Returns live STA status as JSON for the update page to poll
  bool conn = (WiFi.status() == WL_CONNECTED);
  String ssid = WiFi.SSID();
  if (!ssid.length()) ssid = updSta.ssid;

  String j = "{";
  j += "\"connected\":"; j += (conn ? "true" : "false"); j += ",";
  j += "\"wantSta\":"; j += (updSta.wantSta ? "true" : "false"); j += ",";
  j += "\"ssid\":\"" + jsonEscape(ssid) + "\",";
  j += "\"ip\":\"" + (conn ? WiFi.localIP().toString() : String("")) + "\",";
  j += "\"rssi\":" + String(conn ? WiFi.RSSI() : 0) + ",";
  j += "\"msg\":\"" + jsonEscape(staLastMsg) + "\"";
  j += "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

void routeUpdateCheck() {
  if (!allowUpdatePage()) { server.send(403, "text/plain", "Forbidden"); return; }
  lastCheckMsg = "";
  lastMf = Manifest();

  if (WiFi.status() != WL_CONNECTED) {
    lastCheckMsg = "STA not connected.";
    server.send(200, "text/plain", lastCheckMsg);
    return;
  }
  if (upd.manifestUrl.length() == 0) {
    lastCheckMsg = "Manifest URL empty.";
    server.send(200, "text/plain", lastCheckMsg);
    return;
  }

  String json;
  if (!httpGetString(upd.manifestUrl, json)) {
    lastCheckMsg = "Manifest fetch failed.";
    server.send(200, "text/plain", lastCheckMsg);
    return;
  }
  if (!parseManifest(json, lastMf)) {
    lastCheckMsg = "Manifest parse failed.";
    server.send(200, "text/plain", lastCheckMsg);
    return;
  }

  int cmp = versionCmp(String(FW_VERSION), lastMf.version);
  if (cmp < 0) lastCheckMsg = "Update available: " + lastMf.version;
  else if (cmp == 0) lastCheckMsg = "Up to date: " + lastMf.version;
  else lastCheckMsg = "Running newer than manifest: " + lastMf.version;

  server.send(200, "text/plain", lastCheckMsg);
}

static void routeUpdateDo() {
  if (!allowUpdatePage()) { server.send(403, "text/plain", "Forbidden"); return; }
  if (WiFi.status() != WL_CONNECTED) { server.send(400, "text/plain", "STA not connected"); return; }
  if (upd.manifestUrl.length() == 0) { server.send(400, "text/plain", "Manifest URL empty"); return; }

  String json;
  Manifest mf;
  if (!httpGetString(upd.manifestUrl, json) || !parseManifest(json, mf)) {
    server.send(500, "text/plain", "Manifest fetch/parse failed");
    return;
  }
  if (versionCmp(String(FW_VERSION), mf.version) >= 0) {
    server.send(200, "text/plain", "No update needed");
    return;
  }

  server.send(200, "text/plain", "Downloading & flashing... rebooting");
  delay(200);

  String err;
  if (otaHttpUpdate(mf.binUrl, err)) {
    delay(250);
    ESP.restart();
  } else {
    Serial.println("[UPD] Failed: " + err);
  }
}

// Local upload (multipart)
static void routeUpdateUpload() {
  if (!allowUpdatePage()) { server.send(403, "text/plain", "Forbidden"); return; }

  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    Serial.println("[UPD] Upload start: " + up.filename);
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Serial.println("[UPD] Update.begin failed");
    }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.write(up.buf, up.currentSize) != up.currentSize) {
      Serial.println("[UPD] write failed");
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.println("[UPD] Upload done, rebooting");
      server.send(200, "text/plain", "Flashed. Rebooting...");
      delay(250);
      ESP.restart();
    } else {
      server.send(500, "text/plain", "Update failed");
    }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    server.send(500, "text/plain", "Upload aborted");
  }
}

void routeConfig() {
  String s = htmlHeader("TD42Tacho Config");
  updateToken = randomToken();
  s += "<div class='row'><a href='/update?t=" + updateToken + "'>Firmware Updates -></a></div>";
  s += "<div class='topbar'><div><h2 style='margin:0'>Config</h2><small>v3.6.2 STABLE</small></div>"
       "<div><a href='/'>Back</a></div></div>";

  s += "<div class='card'><b>Speed Calibration</b><br><small>Best: set pulses per km using GPS calibration.</small>";
  s += "<form action='/setSpeed' method='post'>";
  s += "<div class='row'>Pulses per KM: <input name='ppkm' type='number' step='1' value='" + fmt1(spd.pulsesPerKm) + "'></div>";
  s += "<button type='submit'>Save Speed</button></form></div>";


  // ---- Factory Dash Tach Out ----
  s += "<div class='card'><b>Factory Dash Tach Out (GU TB45)</b><br>"
       "<small>Use an external transistor (open-collector). Cluster provides pull-up (~5–8V). "
       "Never drive the vehicle wire high. This output only pulls LOW.</small>";
  s += "<form action='/setDash' method='post'>";
  s += "<div class='row'><label><input type='checkbox' name='dt_en' " + String(dashTachEnabled ? "checked" : "") + "> Enable dash tacho output</label></div>";
  s += "<div class='row'>Pulses per Rev: <input name='dt_ppr' type='number' min='0.1' max='24' step='0.1' value='" + String(dashPulsesPerRev, 1) + "'></div>";
  s += "<div class='row'>Dash Cal (%): <input name='dt_cal' type='number' min='50' max='150' step='1' value='" + String(dashCalPct) + "'> <small>(if dash reads low, increase)</small></div>";
  s += "<hr><b>Test Mode (Option A)</b><br><small>Outputs a fixed frequency so the dash holds a steady RPM (for wiring/debug).</small>";
  s += "<div class='row'><label><input type='checkbox' name='dt_ft' " + String(dashForceTest ? "checked" : "") + "> Enable fixed Hz test</label></div>";
  s += "<div class='row'>Fixed Hz: <input name='dt_fh' type='number' min='2' max='2000' step='1' value='" + String((int)dashForceHz) + "'></div>";
  s += "<div class='row'>Pulse LOW width (us): <input name='dt_pw' type='number' min='150' max='5000' step='10' value='" + String((int)dashPulseLowUs) + "'></div>";
  s += "<div class='row'><button class='btn' type='submit'>Save Dash</button></div>";
  s += "</form>";
  // Hammer test (runtime only)
  s += "<hr><b>Hammer test</b> <small>(runtime only)</small><br><small>Forces the output to pull LOW continuously. Useful to prove wiring/cluster input. Turn OFF before driving.</small>";
  s += "<div class='row'><form method='post' action='/dashHammer' style='display:inline-block;margin-right:8px'>"
       "<input type='hidden' name='on' value='1'><button class='btn' type='submit'>HAMMER ON</button></form>"
       "<form method='post' action='/dashHammer' style='display:inline-block'>"
       "<input type='hidden' name='off' value='1'><button class='btn' type='submit'>HAMMER OFF</button></form>"
       "<span style='margin-left:10px'>State: <b>" + String(dashHammerMode ? "ON" : "OFF") + "</b></span></div>";
  s += "</div>";
  s += "<div class='card'><b>Display / Graph Settings</b><br><small>OLED bar + markers</small><hr>";
  s += "<form action='/setUi' method='post'>";
  s += "<div class='row'>Graph Mode: <select name='gm'>";
  s += "<option value='0' " + String(ui.graphMode==0?"selected":"") + ">RPM</option>";
  s += "<option value='1' " + String(ui.graphMode==1?"selected":"") + ">BOOST</option>";
  s += "</select></div>";
  s += "<div class='row'>Bar Min RPM: <input name='barMin' type='number' value='" + String(ui.barMinRpm) + "'></div>";
  s += "<div class='row'>Bar Max RPM: <input name='barMax' type='number' value='" + String(ui.barMaxRpm) + "'></div>";
  s += "<div class='row'>Segments (16-64): <input name='segs' type='number' value='" + String(ui.segments) + "'></div>";
  s += "<div class='row'>Bar Height (10-20): <input name='barH' type='number' value='" + String(ui.barH) + "'></div>";
  s += "<div class='row'>Torque Start: <input name='tqS' type='number' value='" + String(ui.torqueStart) + "'></div>";
  s += "<div class='row'>Torque End: <input name='tqE' type='number' value='" + String(ui.torqueEnd) + "'></div>";
  s += "<div class='row'>Flash RPM: <input name='flash' type='number' value='" + String(ui.flashRpm) + "'></div>";
  s += "<div class='row'>Redline RPM: <input name='red' type='number' value='" + String(ui.redlineRpm) + "'></div>";
  s += "<div class='row'>Cam triggers per cam rev: <input name='camT' type='number' min='1' max='60' value='" + String(camTriggersPerCamRev) + "'></div>";
  s += "<div class='row'>VR OUT edge: <select name='edge'>";
  s += String(rpmEdgeMode==0 ? "<option value='0' selected>FALLING (recommended)</option>" : "<option value='0'>FALLING (recommended)</option>");
  s += String(rpmEdgeMode==1 ? "<option value='1' selected>RISING</option>" : "<option value='1'>RISING</option>");
  s += "</select></div>";
    s += "<hr><b>Boost (only used if Graph Mode = BOOST)</b>";
  s += "<div class='row'>Zero volts (sensor): <input name='bz_v0' type='number' step='0.01' value='" + String(boost.vZero,2) + "'></div>";
  s += "<div class='row'>Max volts (sensor): <input name='bz_vm' type='number' step='0.01' value='" + String(boost.vMax,2) + "'></div>";
  s += "<div class='row'>Max PSI: <input name='bz_mp' type='number' step='0.1' value='" + String(boost.maxPsi,1) + "'></div>";
  s += "<div class='row'>Flash PSI (0=off): <input name='bz_fl' type='number' step='0.1' value='" + String(boost.flashPsi,1) + "'></div>";
  s += "<div class='row'>Divider ratio (ADC/sensor): <input name='bz_div' type='number' step='0.01' value='" + String(boost.divider,2) + "'></div>";
  s += "<div class='row'>Smoothing alpha: <input name='bz_al' type='number' step='0.01' value='" + String(boost.smoothAlpha,2) + "'></div>";
  s += "<div class='row'><small>Sensor is 0.45-5.0V, so use a divider to keep ADC &lt;= 3.3V. GPIO34 is used.</small></div>";
  s += "<button type='submit'>Save UI</button></form></div>";

  s += "<h3>Service Reminders</h3>";
  s += "<div class='grid'>";
  s += serviceBlock("Oil", "oil", svc.oil, false, "");
  s += serviceBlock("Fuel", "ful", svc.fuel, false, "");
  s += serviceBlock("Air", "air", svc.air, false, "");
  
  s += serviceBlock("Custom", "cus", svc.custom, true, svc.customName);
s += "</div>";

  s += "<div class='card' style='margin-top:12px'><b>Network</b><br>";
  wifi_mode_t m = WiFi.getMode();
  if (m == WIFI_OFF) {
    s += "Mode: <b>OFF</b>";
  } else if (m == WIFI_AP) {
    s += "Mode: <b>AP</b><br>SSID: <b>" + String(AP_SSID) + "</b><br>IP: <b>" + WiFi.softAPIP().toString() + "</b>";
  } else if (m == WIFI_STA) {
    String ss = WiFi.SSID();
    s += "Mode: <b>STA</b><br>SSID: <b>" + htmlEscape(ss) + "</b><br>IP: <b>" + WiFi.localIP().toString() + "</b>";
  } else { // WIFI_AP_STA
    s += "Mode: <b>AP+STA</b><br>AP SSID: <b>" + String(AP_SSID) + "</b> (IP <b>" + WiFi.softAPIP().toString() + "</b>)<br>";
    s += "<span id=\'staLine\'>";if (WiFi.status() == WL_CONNECTED) {  s += "STA: <b>Connected</b> to <b>" + htmlEscape(WiFi.SSID()) + "</b> (IP <b>" + WiFi.localIP().toString() + "</b>, RSSI <b>" + String(WiFi.RSSI()) + "dBm</b>)";} else if (updSta.wantSta) {  s += "STA: <b>Connecting...</b>";} else if (staLastMsg.length()) {  s += "STA: <b>" + htmlEscape(staLastMsg) + "</b>";} else {  s += "STA: <b>Idle</b>";}s += "</span>";}
  s += "</div>";

    // Live update STA status without refreshing the page
  s += "<script>"
       "function _eh(s){return (s||'').replace(/[&<>\\\"\']/g,function(c){switch(c){case '&':return '&amp;';case '<':return '&lt;';case '>':return '&gt;';case '\\\"':return '&quot;';default:return '&#39;';}});} "
       "async function _pollSta(){try{const el=document.getElementById('staLine');if(!el)return;"
       "const r=await fetch('/update/status?_='+Date.now(),{cache:'no-store'});if(!r.ok)return;const j=await r.json();"
       "if(j.connected){el.innerHTML='STA: <b>Connected</b> to <b>'+_eh(j.ssid)+'</b> (IP <b>'+_eh(j.ip)+'</b>, RSSI <b>'+j.rssi+'dBm</b>)';}"
       "else if(j.wantSta){el.innerHTML='STA: <b>Connecting...</b>';}"
       "else if(j.msg&&j.msg.length){el.innerHTML='STA: <b>'+_eh(j.msg)+'</b>';}"
       "else{el.innerHTML='STA: <b>Idle</b>';}"
       "}catch(e){}}" 
       "setInterval(_pollSta,1000);_pollSta();"
       "</script>";
s += htmlFooter();
  
  // Diagnostics / Test tools
  s += "<div class='card'><h3>Diagnostics</h3>";
  s += "<form action='/setDiag' method='post'>";
  s += "<div class='row'><label><input type='checkbox' name='en' " + String(diag.enabled ? "checked" : "") + "> Enable diagnostics</label></div>";

  s += "<h4>RPM override / sweep</h4>";
  s += "<div class='row'><label><input type='checkbox' name='ro' " + String(diag.rpmOverride ? "checked" : "") + "> Override RPM</label></div>";
  s += "<div class='row'>RPM value: <input name='rv' type='number' min='0' max='9000' value='" + String(diag.rpmValue) + "'></div>";

  s += "<div class='row'><label><input type='checkbox' name='sw' " + String(diag.sweep ? "checked" : "") + "> Sweep RPM</label></div>";
  s += "<div class='row'>Min: <input name='smin' type='number' min='0' max='9000' value='" + String(diag.sweepMin) + "'></div>";
  s += "<div class='row'>Max: <input name='smax' type='number' min='0' max='9000' value='" + String(diag.sweepMax) + "'></div>";
  s += "<div class='row'>Step: <input name='sstp' type='number' min='1' max='5000' value='" + String(diag.sweepStep) + "'></div>";
  s += "<div class='row'>Dwell ms: <input name='sdw' type='number' min='50' max='10000' value='" + String(diag.sweepDwellMs) + "'></div>";

  s += "<h4>Force service-due display</h4>";
  s += "<div class='row'><label><input type='checkbox' name='fd' " + String(diag.forceDue ? "checked" : "") + "> Force DUE messages</label></div>";
  s += "<div class='row'>";
  s += "<label><input type='checkbox' name='d0' " + String((diag.dueMask & 1) ? "checked" : "") + "> Oil</label> ";
  s += "<label><input type='checkbox' name='d1' " + String((diag.dueMask & 2) ? "checked" : "") + "> Fuel</label> ";
  s += "<label><input type='checkbox' name='d2' " + String((diag.dueMask & 4) ? "checked" : "") + "> Air</label> ";
  s += "<label><input type='checkbox' name='d3' " + String((diag.dueMask & 8) ? "checked" : "") + "> Custom</label>";
  s += "</div>";

    // Boost sweep (only affects BOOST graph mode)
  s += "<h4>Boost sweep</h4>";
  s += "<div class='row'><label><input type='checkbox' name='bs' " + String(diag.boostSweepEnabled ? "checked" : "") + "> Sweep BOOST (0-" + String((int)roundf(boost.maxPsi)) + " psi)</label></div>";
s += "<button class='btn' type='submit'>Apply</button>";
  s += "</form>";
  s += "<div class='muted'>Use Sweep to verify bar/markers and fonts. Use Force DUE to test the header flashing without changing real counters.</div>";
  s += "</div>";

  server.send(200, "text/html", s);
}


void routeSignalStatus() {
  uint32_t now = millis();

  // Snapshot volatile state
  uint32_t rLast = rpmLastPulseMs;
  uint32_t sLast = speedLastPulseMs;
  uint32_t rCnt  = rpmPulseCount;
  uint32_t sCnt  = speedPulseCount;
  uint32_t rPer  = rpmPeriodUs;

  int32_t rAge = (rLast == 0) ? -1 : (int32_t)(now - rLast);
  int32_t sAge = (sLast == 0) ? -1 : (int32_t)(now - sLast);

  float rpmEst = 0.0f;
  if (rPer > 0) {
    float hz = 1000000.0f / (float)rPer;
    rpmEst = hz * engineRpmPerHz;
  }

  String j = "{";
  j += "\"rpmAge\":" + String(rAge);
  j += ",\"spdAge\":" + String(sAge);
  j += ",\"rpmCount\":" + String(rCnt);
  j += ",\"spdCount\":" + String(sCnt);
  j += ",\"rpmPeriodUs\":" + String(rPer);
  j += ",\"rpmEst\":" + fmt1(rpmEst);
  j += "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

void setupRoutes() {
  server.on("/", HTTP_GET, routeDashboard);
  server.on("/config", HTTP_GET, routeConfig);

  server.on("/update", HTTP_GET, routeUpdate);
  server.on("/update/save", HTTP_POST, routeUpdateSave);
  server.on("/update/sta", HTTP_POST, routeUpdateSta);
  server.on("/update/disc", HTTP_POST, routeUpdateDisc);
  server.on("/update/status", HTTP_GET, routeUpdateStatus);
  server.on("/sig", HTTP_GET, routeSignalStatus);
  server.on("/update/check", HTTP_POST, routeUpdateCheck);
  server.on("/update/do", HTTP_POST, routeUpdateDo);
  server.on("/update/upload", HTTP_POST, [](){ server.send(200); }, routeUpdateUpload);
  server.on("/resetTripRpm", HTTP_POST, []() {
    maxRpmTrip = 0.0f;
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.on("/resetTripKm", HTTP_POST, []() {
    tripKm = 0.0;
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.on("/resetHours", HTTP_POST, []() {
    engineSeconds = 0;
    saveHours();
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.on("/setHours", HTTP_POST, []() {
    if (!server.hasArg("hrs")) {
      server.send(400, "text/plain", "Missing hrs");
      return;
    }
    double h = server.arg("hrs").toDouble();
    if (h < 0) h = 0;
    // store as whole seconds (uint32_t)
    double sec = h * 3600.0;
    if (sec > 4294967295.0) sec = 4294967295.0;
    engineSeconds = (uint32_t)(sec + 0.5);
    saveHours();
    server.sendHeader("Location", "/");
    server.send(303);
  });


  server.on("/setSpeed", HTTP_POST, []() {
    if (!server.hasArg("ppkm")) {
      server.send(400, "text/plain", "Missing ppkm");
      return;
    }
    double v = server.arg("ppkm").toDouble();
    spd.pulsesPerKm = clampDouble(v, 1.0, 1e9);
    saveSpeedCfg();
    server.sendHeader("Location", "/config");
    server.send(303);
  });

  server.on("/setDash", HTTP_POST, []() {
    dashTachEnabled = server.hasArg("dt_en");

    if (server.hasArg("dt_ppr")) {
      dashPulsesPerRev = constrain(server.arg("dt_ppr").toFloat(), 0.1f, 24.0f);
    }
    if (server.hasArg("dt_cal")) {
      dashCalPct = constrain(server.arg("dt_cal").toInt(), 50, 150);
    }

    dashForceTest = server.hasArg("dt_ft");
    if (server.hasArg("dt_fh")) {
      dashForceHz = constrain(server.arg("dt_fh").toFloat(), 2.0f, 2000.0f);
    }

    if (server.hasArg("dt_pw")) {
      dashPulseLowUs = (uint32_t)constrain(server.arg("dt_pw").toInt(), 150, 5000);
    }

    saveDashCfg();
    server.sendHeader("Location", "/config");
    server.send(303);
  });

  server.on("/dashHammer", HTTP_POST, []() {
    if (server.hasArg("on"))  dashHammerMode = true;
    if (server.hasArg("off")) dashHammerMode = false;
    if (!dashHammerMode) {
      // release line
      digitalWrite(PIN_TACH_OUT, LOW);
    }
    server.sendHeader("Location", "/config");
    server.send(303);
  });


  server.on("/setUi", HTTP_POST, []() {
    int oldEdge = rpmEdgeMode;
    if (server.hasArg("barMin")) ui.barMinRpm = server.arg("barMin").toInt();
    if (server.hasArg("barMax")) ui.barMaxRpm = server.arg("barMax").toInt();
    if (server.hasArg("segs"))   ui.segments  = server.arg("segs").toInt();
    if (server.hasArg("barH"))   ui.barH      = server.arg("barH").toInt();
    if (server.hasArg("tqS"))    ui.torqueStart = server.arg("tqS").toInt();
    if (server.hasArg("tqE"))    ui.torqueEnd   = server.arg("tqE").toInt();
    if (server.hasArg("flash"))  ui.flashRpm    = server.arg("flash").toInt();
    if (server.hasArg("red"))    ui.redlineRpm  = server.arg("red").toInt();
    if (server.hasArg("camT"))   camTriggersPerCamRev = server.arg("camT").toInt();
    if (server.hasArg("edge"))   rpmEdgeMode = server.arg("edge").toInt();

    if (server.hasArg("gm"))    ui.graphMode   = server.arg("gm").toInt();

    if (server.hasArg("bz_v0")) boost.vZero = server.arg("bz_v0").toFloat();
    if (server.hasArg("bz_vm")) boost.vMax  = server.arg("bz_vm").toFloat();
    if (server.hasArg("bz_mp")) boost.maxPsi = server.arg("bz_mp").toFloat();
    if (server.hasArg("bz_div")) boost.divider = server.arg("bz_div").toFloat();
    if (server.hasArg("bz_al")) boost.smoothAlpha = server.arg("bz_al").toFloat();
    if (server.hasArg("bz_fl")) boost.flashPsi = server.arg("bz_fl").toFloat();


    ui.segments  = clampInt(ui.segments, 16, 64);
    ui.barH      = clampInt(ui.barH, 10, 20);
    ui.barMinRpm = clampInt(ui.barMinRpm, 0, 9000);
    ui.barMaxRpm = clampInt(ui.barMaxRpm, ui.barMinRpm + 1, 9000);

    ui.torqueStart = clampInt(ui.torqueStart, 0, 9000);
    ui.torqueEnd   = clampInt(ui.torqueEnd, 0, 9000);
    ui.flashRpm    = clampInt(ui.flashRpm, 0, 9000);
    ui.redlineRpm  = clampInt(ui.redlineRpm, 0, 9000);
    ui.graphMode   = clampInt(ui.graphMode, 0, 1);
    camTriggersPerCamRev = clampInt(camTriggersPerCamRev, 1, 60);
    rpmEdgeMode = clampInt(rpmEdgeMode, 0, 1);
    recomputeRpmScaling();
    if (oldEdge != rpmEdgeMode) {
      detachInterrupt(digitalPinToInterrupt(PIN_VR_RPM));
      attachInterrupt(digitalPinToInterrupt(PIN_VR_RPM), isrRpm, (rpmEdgeMode==1) ? RISING : FALLING);
      detachInterrupt(digitalPinToInterrupt(PIN_VR_SPEED));
      attachInterrupt(digitalPinToInterrupt(PIN_VR_SPEED), isrSpeed, (rpmEdgeMode==1) ? RISING : FALLING);
    }

    if (boost.maxPsi < 1.0f) boost.maxPsi = 1.0f;
    if (boost.maxPsi > 60.0f) boost.maxPsi = 60.0f;
    if (boost.flashPsi < 0.0f) boost.flashPsi = 0.0f;
    if (boost.flashPsi > 100.0f) boost.flashPsi = 100.0f;
    if (boost.divider < 0.05f) boost.divider = 0.05f;
    if (boost.divider > 1.00f) boost.divider = 1.00f;
    if (boost.smoothAlpha < 0.0f) boost.smoothAlpha = 0.0f;
    if (boost.smoothAlpha > 1.0f) boost.smoothAlpha = 1.0f;

    saveUi();
    server.sendHeader("Location", "/config");
    server.send(303);
  });

  
  server.on("/setDiag", HTTP_POST, []() {
    diag.enabled = server.hasArg("en");
    diag.rpmOverride = server.hasArg("ro");
    diag.sweep = server.hasArg("sw");
    diag.forceDue = server.hasArg("fd");

    diag.rpmValue = clampInt(server.arg("rv").toInt(), 0, 9000);
    diag.sweepMin = clampInt(server.arg("smin").toInt(), 0, 9000);
    diag.sweepMax = clampInt(server.arg("smax").toInt(), 0, 9000);
    if (diag.sweepMax < diag.sweepMin) diag.sweepMax = diag.sweepMin;

    diag.sweepStep = clampInt(server.arg("sstp").toInt(), 1, 5000);
    diag.sweepDwellMs = clampInt(server.arg("sdw").toInt(), 50, 10000);

    uint8_t mask = 0;
    if (server.hasArg("d0")) mask |= 1;
    if (server.hasArg("d1")) mask |= 2;
    if (server.hasArg("d2")) mask |= 4;
    if (server.hasArg("d3")) mask |= 8;
    diag.dueMask = mask;

    // If sweep is enabled, ensure RPM override is on so UI uses it.
    if (diag.enabled && diag.sweep) diag.rpmOverride = true;


    diag.boostSweepEnabled = server.hasArg("bs");
    if (!diag.boostSweepEnabled) { diag.boostSweepPsi = 0.0f; diag.boostSweepUp = true; }
    diag.lastBoostSweepMs = millis();

    // Persist diag boost sweep only
    prefs.begin("td42tach", false);
    prefs.putBool("bs", diag.boostSweepEnabled);
    prefs.end();

    server.sendHeader("Location", "/config");
    server.send(303);
  });

server.on("/setService", HTTP_POST, []() {
    if (!server.hasArg("key") || !server.hasArg("mode") || !server.hasArg("ih") || !server.hasArg("ik")) {
      server.send(400, "text/plain", "Missing args");
      return;
    }
    String key = server.arg("key");
    String mode = server.arg("mode");
    float ih = clampFloat(server.arg("ih").toFloat(), 0.0f, 50000.0f);
    float ik = clampFloat(server.arg("ik").toFloat(), 0.0f, 1e7f);

    ServiceItem* it = nullptr;
    const char* prefix = nullptr;

    if (key == "oil") { it = &svc.oil; prefix = "oil"; }
    else if (key == "ful") { it = &svc.fuel; prefix = "ful"; }
    else if (key == "air") { it = &svc.air; prefix = "air"; }
    else if (key == "cus") { it = &svc.custom; prefix = "cus"; }
    else { server.send(400, "text/plain", "Bad key"); return; }

    it->intervalHours = ih;
    it->intervalKm    = ik;
    it->mode = (mode == "km") ? MODE_KM : MODE_HOURS;

    if (key == "cus" && server.hasArg("cname")) {
      svc.customName = server.arg("cname");
      if (svc.customName.length() == 0) svc.customName = "Custom";
      saveCustomName();
    }

    saveService(prefix, *it);
    server.sendHeader("Location", "/config");
    server.send(303);
  });

  server.on("/resetService", HTTP_POST, []() {
    if (!server.hasArg("key")) {
      server.send(400, "text/plain", "Missing key");
      return;
    }
    String key = server.arg("key");

    ServiceItem* it = nullptr;
    const char* prefix = nullptr;
    if (key == "oil") { it = &svc.oil; prefix = "oil"; }
    else if (key == "ful") { it = &svc.fuel; prefix = "ful"; }
    else if (key == "air") { it = &svc.air; prefix = "air"; }
    else if (key == "cus") { it = &svc.custom; prefix = "cus"; }
    else { server.send(400, "text/plain", "Bad key"); return; }

    it->lastSec = engineSeconds;
    it->lastKm  = totalKm;

    saveService(prefix, *it);
    server.sendHeader("Location", "/config");
    server.send(303);
  });
}

/* ================== Wi-Fi connect w/ fallback ================== */
void setupWiFiStaOrAp() {
  // Service mode networking:
  // - Start AP only (no auto-STA connect) for safety + quick boot.
  // - User can optionally connect STA from the /update page (AP+STA) when needed.
  Serial.println();
  Serial.println("[NET] Service mode: starting AP...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("[NET] AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("[NET] AP IP: ");
  Serial.println(WiFi.softAPIP());

  setupRoutes();
  server.begin();
}

/* ================== SERVICE MODE HELPERS ================== */

static void serviceBootCounterInit() {
  // Count boots in NVS. If the unit stays powered for SERVICE_BOOT_WINDOW_MS,
  // we reset the counter back to 0 (handled in serviceModeTick()).
  prefs.begin("td42tach", false);
  uint32_t bc = prefs.getUInt("svc_bc", 0);
  bc++;
  prefs.putUInt("svc_bc", bc);
  prefs.end();

  if (bc >= SERVICE_TAP_COUNT) {
    serviceMode = true;

    // Reset counter immediately so next power cycle is normal unless user taps again.
    prefs.begin("td42tach", false);
    prefs.putUInt("svc_bc", 0);
    prefs.end();

    // No need to reset again in tick().
    svcBootResetDone = true;
    Serial.println("[SERVICE] Power-cycle trigger detected (service mode requested).");
  }
}

static void serviceModeStop() {
  Serial.println("[SERVICE] Service mode timeout - stopping WiFi/Web/OTA.");

  // Stop network activity and free resources as much as possible.
  updSta.wantSta = false;
  updSta.connected = false;

  // Disable OTA handler.
  otaReady = false;

  // Drop AP + STA.
  WiFi.disconnect(true);
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);

  // Clear token so /update cannot be opened accidentally later.
  updateToken = "";
}

static void serviceModeTick() {
  // After the device has been on for a bit, reset the boot counter so
  // casual power cycles later don't accidentally enter service mode.
  if (!svcBootResetDone && millis() > SERVICE_BOOT_WINDOW_MS) {
    prefs.begin("td42tach", false);
    prefs.putUInt("svc_bc", 0);
    prefs.end();
    svcBootResetDone = true;
  }

  // Enforce service mode timeout.
  if (serviceMode && serviceStartMs != 0) {
    if ((millis() - serviceStartMs) >= SERVICE_TIMEOUT_MS) {
      serviceModeStop();
      serviceMode = false;
    }
  }
}

/* ================== OLED DRAWING ================== */

void drawHeader() {
  // Normal running: keep header empty (future: icons/status)
  String headerText = "";

  if (anyServiceDue()) {
    static uint32_t lastRotateMs = 0;
    static uint32_t lastBlinkMs  = 0;
    static int itemIdx = 0;
    static bool blinkOn = true;

    // Build list of due item names
    String items[6];
    int n = 0;
    if (serviceDueEffective(svc.oil, 0)  && n < 6) items[n++] = "OIL";
    if (serviceDueEffective(svc.fuel, 1) && n < 6) items[n++] = "FUEL";
    if (serviceDueEffective(svc.air, 2)  && n < 6) items[n++] = "AIR";
    if (serviceDueEffective(svc.custom, 3) && n < 6) {
      String nm = svc.customName;
      nm.trim();
      if (nm.length() == 0) nm = "CUSTOM";
      nm.toUpperCase();
      if (nm.length() > 10) nm = nm.substring(0, 10);
      items[n++] = nm;
    }

    uint32_t now = millis();

    // Blink (visibility only)
    if (now - lastBlinkMs >= 300) {
      lastBlinkMs = now;
      blinkOn = !blinkOn;
    }

    // Rotate items
    if (now - lastRotateMs >= 2000) {
      lastRotateMs = now;
      if (n > 0) itemIdx = (itemIdx + 1) % n;
    }

    if (n > 0 && blinkOn) {
      headerText = items[itemIdx] + " DUE";
    }
  }

  if (headerText.length() > 0) {
    u8g2.setFont(u8g2_font_6x10_tf);
    int w = u8g2.getStrWidth(headerText.c_str());
    int x = (OLED_W - w) / 2;
    if (x < 0) x = 0;
    u8g2.drawStr(x, 10, headerText.c_str());
  }
  // Service Mode indicator (blinks so you can tell without Serial)
  if (serviceMode) {
    static uint32_t svcBlinkMs = 0;
    static bool svcOn = true;
    uint32_t now = millis();
    if (now - svcBlinkMs >= 500) {
      svcBlinkMs = now;
      svcOn = !svcOn;
    }
    if (svcOn) {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(0, 10, "SERVICE");
    }
  }


// Input signal indicators (helpful for in-car diagnosis without Serial/PC)
// Solid box = pulses recently seen. Outline = no pulses (timeout).
{
  uint32_t now = millis();
  bool rpmOk = (rpmLastPulseMs != 0) && ((now - rpmLastPulseMs) < RPM_LOSS_TIMEOUT_MS);
  bool spdOk = (speedLastPulseMs != 0) && ((now - speedLastPulseMs) < SPEED_LOSS_TIMEOUT_MS);

  u8g2.setFont(u8g2_font_6x10_tf);

  // "R" + box
  u8g2.drawStr(OLED_W - 30, 10, "R");
  if (rpmOk) u8g2.drawBox(OLED_W - 24, 2, 6, 6);
  else       u8g2.drawFrame(OLED_W - 24, 2, 6, 6);

  // "S" + box
  u8g2.drawStr(OLED_W - 14, 10, "S");
  if (spdOk) u8g2.drawBox(OLED_W - 8, 2, 6, 6);
  else       u8g2.drawFrame(OLED_W - 8, 2, 6, 6);
}

}


void drawRedlineMarker() {
  int y = ui.barY;
  int innerX = 1;
  int innerW = OLED_W - 2;

  int minR = ui.barMinRpm;
  int maxR = ui.barMaxRpm;
  if (maxR <= minR) maxR = minR + 1;

  long num = (long)(ui.redlineRpm - minR) * innerW;
  long den = (long)(maxR - minR);
  int x = innerX + (int)(num / den);

  if (x < innerX) x = innerX;
  if (x > innerX + innerW) x = innerX + innerW;

  int topY = y - 1;
  u8g2.drawTriangle(x, topY, x - 3, topY - 5, x + 3, topY - 5);
}


static void drawTriangleMarker(int x, int yTop) {
  u8g2.drawTriangle(x, yTop, x - 3, yTop + 5, x + 3, yTop + 5);
}


void drawSegmentedBar(int rpmVal) {
  int y = ui.barY;
  int h = ui.barH;

  u8g2.drawFrame(0, y, OLED_W, h);

  int innerX = 1;
  int innerY = y + 1;
  int innerW = OLED_W - 2;
  int innerH = h - 2;

  int segs = ui.segments;
  if (segs < 1) segs = 1;

  int minR = ui.barMinRpm;
  int maxR = ui.barMaxRpm;
  if (maxR <= minR) maxR = minR + 1;

  float t = (rpmVal - minR) / (float)(maxR - minR);
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  int filled = (int)roundf(t * segs);

  int gap = 1;
  int totalSegPixels = innerW - (segs - 1) * gap;
  if (totalSegPixels < segs) {
    gap = 0;
    totalSegPixels = innerW;
  }
  int baseW = totalSegPixels / segs;
  int remW  = totalSegPixels % segs;

  auto rpmToSeg = [&](int r)->int {
    long num = (long)(r - minR) * segs;
    long den = (long)(maxR - minR);
    long s = (den == 0) ? 0 : (num / den);
    if (s < 0) s = 0;
    if (s > segs - 1) s = segs - 1;
    return (int)s;
  };

  int tqS = rpmToSeg(ui.torqueStart);
  int tqE = rpmToSeg(ui.torqueEnd);
  if (tqE < tqS) { int tmp = tqS; tqS = tqE; tqE = tmp; }

  int x = innerX;
  for (int i = 0; i < segs; i++) {
    int w = baseW + (i < remW ? 1 : 0);

    if (i < filled) {
      u8g2.drawBox(x, innerY, w, innerH);
    }

    if (i >= tqS && i <= tqE) {
      u8g2.drawHLine(x, innerY + innerH - 1, w);
    }

    x += w + gap;
    if (x > innerX + innerW) break;
  }
}


void drawMain() {
  u8g2.clearBuffer();
  drawHeader();

  float uiRpm = rpmForUi();

  // Main bar area: RPM or BOOST
  float psiShow = 0.0f;
  if (ui.graphMode == 1) {
    psiShow = diag.boostSweepEnabled ? diag.boostSweepPsi : boostPsi;
    drawBoostBar(psiShow);
    if (!anyServiceDue()) drawBoostLabels();
    // (numeric boost text below bar removed; bottom line shows RPM and PSI)
  } else {
    drawSegmentedBar((int)uiRpm);
    drawRedlineMarker();
  }

  // Bottom numbers (independent flashing)
  bool blinkOn = rpmFlashState();

  bool rpmFlash = (ui.flashRpm > 0) && (uiRpm >= ui.flashRpm);
  bool psiFlash = (ui.graphMode == 1) && (boost.flashPsi > 0.0f) && (psiShow >= boost.flashPsi);

  // If both RPM and PSI are flashing at the same time, flash them opposite to each other
  // so the driver can still read one value at any instant.
  bool bothFlash = rpmFlash && psiFlash;

  bool showRpm  = (!rpmFlash) || blinkOn;
  bool showPsi  = (!psiFlash) || (bothFlash ? !blinkOn : blinkOn);

  drawBottom((int)uiRpm, psiShow, showRpm, showPsi);

  u8g2.sendBuffer();
}



void drawBottom(int rpmVal, float psiVal, bool showRpm, bool showPsi) {
  // Bottom line: RPM mode uses big font; BOOST mode uses a slightly smaller font to fit RPM + PSI cleanly

  // Baseline for 64px height OLED
  const int y = 63;

  if (ui.graphMode == 1) {
    // BOOST graph selected: smaller font for split RPM / PSI
    u8g2.setFont(u8g2_font_logisoso20_tf);

    // BOOST graph selected: show RPM on the left and PSI on the right
    char rpmBuf[12] = "";
    if (showRpm) snprintf(rpmBuf, sizeof(rpmBuf), "%d", rpmVal);

    float psiShow = psiVal;
    int psiInt = (int)(psiShow >= 0 ? (psiShow + 0.5f) : (psiShow - 0.5f));
    if (psiInt < 0) psiInt = 0;

    char psiBuf[12] = "";
    if (showPsi) snprintf(psiBuf, sizeof(psiBuf), "%dPSI", psiInt);

    int rpmW = showRpm ? u8g2.getStrWidth(rpmBuf) : 0;
    int psiW = showPsi ? u8g2.getStrWidth(psiBuf) : 0;

    // Padding so the text doesn't touch the bezel
    const int leftPad = 0;
    int rightX = OLED_W - psiW;
    if (rightX < 0) rightX = 0;

    // If overlap would happen, push PSI to the right as far as possible (may slightly overlap on extreme widths)
    if (leftPad + rpmW + 2 > rightX) {
      rightX = leftPad + rpmW + 2;
      if (rightX > OLED_W - 1) rightX = OLED_W - 1;
    }

    if (showRpm) u8g2.drawStr(leftPad, y, rpmBuf);
    if (showPsi) u8g2.drawStr(rightX, y, psiBuf);
    return;
  }

  // RPM mode: centered RPM number
  u8g2.setFont(u8g2_font_logisoso24_tf);
  char rpmBuf[12];
  snprintf(rpmBuf, sizeof(rpmBuf), "%d", rpmVal);

  int rpmW = u8g2.getStrWidth(rpmBuf);
  int rpmX = (OLED_W - rpmW) / 2;
  if (rpmX < 0) rpmX = 0;

  if (showRpm) u8g2.drawStr(rpmX, y, rpmBuf);
}






/* ================== SETUP ================== */
void setup() {
  Serial.begin(115200);

  serviceBootCounterInit();
  loadAll();
analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(PIN_VR_RPM, INPUT);
  pinMode(PIN_VR_SPEED, INPUT);

  dashTachInit();


  attachInterrupt(digitalPinToInterrupt(PIN_VR_RPM), isrRpm, (rpmEdgeMode==1) ? RISING : FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_VR_SPEED), isrSpeed, (rpmEdgeMode==1) ? RISING : FALLING);
  SPI.begin(OLED_SCK, -1, OLED_MOSI, OLED_CS);
  
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  bootAnimation();
  bootFinished = true;

  maxRpmTrip = 0.0f;
  tripKm = 0.0;

  if (serviceMode) {
    setupWiFiStaOrAp();
    otaSetup();
    serviceStartMs = millis();
    Serial.println("[SERVICE] Service mode ENABLED (10 min timeout)");
  } else {
    // Normal running: keep WiFi completely OFF
    WiFi.mode(WIFI_OFF);
    otaReady = false;
  }
}


/* ================== LOOP ================== */
void loop() {
  serviceModeTick();

  updateBoost();
  diagUpdateBoostSweep();
  if (serviceMode) {
  // Update STA connect watchdog
  if (updSta.wantSta && !updSta.connected) {
    if (WiFi.status() == WL_CONNECTED) {
      updSta.connected = true;
      staConnecting = false;
    } else if (millis() - updSta.startMs > 20000) {
      // Timeout -> stop STA and return to AP-only
      staLastMsg = "STA connect timeout (check SSID/password)";
      updSta.wantSta = false;
      updSta.connected = false;
      stopStaConnect();
    }
  }
  // OTA handler (non-blocking)
  if (!otaReady) {
    if (WiFi.status() == WL_CONNECTED) otaSetup();
  } else {
    ArduinoOTA.handle();
  }
  }

  if (!bootFinished) return;
  // Diagnostics: RPM sweep / override
  if (diag.enabled && diag.sweep) {
    uint32_t now = millis();
    if (now - diag.lastSweepMs >= (uint32_t)max(10, diag.sweepDwellMs)) {
      diag.lastSweepMs = now;

      int step = max(1, diag.sweepStep);
      if (diag.sweepUp) {
        diag.rpmValue += step;
        if (diag.rpmValue >= diag.sweepMax) { diag.rpmValue = diag.sweepMax; diag.sweepUp = false; }
      } else {
        diag.rpmValue -= step;
        if (diag.rpmValue <= diag.sweepMin) { diag.rpmValue = diag.sweepMin; diag.sweepUp = true; }
      }
      diag.rpmOverride = true; // ensure UI uses it while sweeping
    }
  }

  if (serviceMode) server.handleClient();

  uint32_t nowMs = millis();

  // -------- RPM --------
  uint32_t periodUs = rpmPeriodUs;
  uint32_t lastMs   = rpmLastPulseMs;

  bool rpmLost = (lastMs == 0) || ((nowMs - lastMs) > RPM_LOSS_TIMEOUT_MS);

  if (!rpmLost && periodUs > 0) {
    float hz = 1000000.0f / (float)periodUs;
    rpmNow = hz * engineRpmPerHz;
    if (rpmNow < 0) rpmNow = 0;
    if (rpmNow > 9000) rpmNow = 9000;
  } else {
    rpmNow = 0.0f;
  }

  rpmFiltered = (RPM_ALPHA * rpmNow) + ((1.0f - RPM_ALPHA) * rpmFiltered);

  // Factory dash tacho output
  dashTachService(rpmFiltered);


  if (rpmFiltered > maxRpmTrip) maxRpmTrip = rpmFiltered;
  if (rpmFiltered > maxRpmAll) {
    maxRpmAll = rpmFiltered;
    saveMaxRpm();
  }

  // -------- Engine Hours --------
  static uint32_t lastHourTickMs = millis();
  static uint32_t saveHoursAccumSec = 0;

  if (rpmFiltered >= RUNNING_RPM_THRESHOLD) {
    uint32_t dtMs = nowMs - lastHourTickMs;
    if (dtMs >= 1000) {
      uint32_t addSec = dtMs / 1000;
      engineSeconds += addSec;
      lastHourTickMs += addSec * 1000;

      saveHoursAccumSec += addSec;
      if (saveHoursAccumSec >= 60) {
        saveHoursAccumSec = 0;
        saveHours();
      }
    }
  } else {
    lastHourTickMs = nowMs;
  }

  // -------- Speed + Distance --------
  static uint32_t lastSpeedSampleMs = millis();
  static uint32_t lastSpeedPulseCount = 0;

  if (nowMs - lastSpeedSampleMs >= 200) {
    uint32_t dt = nowMs - lastSpeedSampleMs;
    lastSpeedSampleMs = nowMs;

    uint32_t pulsesNow = speedPulseCount;
    uint32_t dp = pulsesNow - lastSpeedPulseCount;
    lastSpeedPulseCount = pulsesNow;

    double dk = (spd.pulsesPerKm > 0.0) ? ((double)dp / spd.pulsesPerKm) : 0.0;
    if (dk < 0) dk = 0;

    tripKm += dk;
    totalKm += dk;

    double hz = (dt > 0) ? ((double)dp * 1000.0 / (double)dt) : 0.0;
    double kmh = (spd.pulsesPerKm > 0.0) ? (hz * 3600.0 / spd.pulsesPerKm) : 0.0;

    bool spLost = (speedLastPulseMs == 0) || ((nowMs - speedLastPulseMs) > SPEED_LOSS_TIMEOUT_MS);
    if (spLost) kmh = 0.0;

    speedNowKmh = (float)kmh;
    speedFilteredKmh = (SPEED_ALPHA * speedNowKmh) + ((1.0f - SPEED_ALPHA) * speedFilteredKmh);
  }

  static double kmSavedAt = 0.0;
  static uint32_t kmSaveMs = millis();
  if ((totalKm - kmSavedAt) >= 0.1 || (nowMs - kmSaveMs) >= 60000) {
    kmSavedAt = totalKm;
    kmSaveMs = nowMs;
    saveKm();
  }

  // -------- OLED --------
  drawMain();

  delay(50);
}

static bool rpmFlashState() {
  static uint32_t last = 0;
  static bool on = true;
  if (millis() - last >= 300) {
    last = millis();
    on = !on;
  }
  return on;
}

// ================== BOOT ANIMATION BITMAPS ==================
// Turbo icon 72x56 (monochrome XBM)