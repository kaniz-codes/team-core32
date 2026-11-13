/*
  FastRobox 1.0 — AI-Powered Smart Classroom
  LEVEL 1: Dual-Core Tasks, Health and Watchdog
  LEVEL 2 (part 1): Wi-Fi setup access point, saved details (NVS), reconnect, mDNS
  LEVEL 2 (part 2): local status page stored in LittleFS, live data at /api/status, NTP time
  LEVEL 2 (part 3): telemetry every 5 s + BOOT / CARD_TAP events over HTTPS (certificate checked)
  LEVEL 3: heat status with hold and hysteresis, heat index, trend, fire latch, Teacher silence, touch panic

  Board   : ESP32-S3-CAM  (Arduino IDE board: "ESP32S3 Dev Module")
  Core    : Arduino-ESP32 3.x (2.x also handled with #if)
  Libs    : MFRC522 (GithubCommunity / miguelbalboa), DHT sensor library (Adafruit)
  Serial  : 115200 baud, line ending = Newline

  Task layout (write-up "task diagram"):
    Core 1:  ledTask     prio 4   50 ms   RGB LED patterns (2 Hz heartbeat)
             sensorTask  prio 3  100 ms   flame every 100 ms, DHT11 every 5 s
             mainTask    prio 2   50 ms   state / door logic (target of "hang")
    Core 0:  rfidTask    prio 2  200 ms   RC522 poll + reader health check
             statusTask  prio 1 1000 ms   [STATUS] line
             serialTask  prio 1   20 ms   Serial commands
             netTask     prio 1   10 ms   Wi-Fi setup page, connect / reconnect (Level 2)
             cloudTask   prio 1  100 ms   HTTPS telemetry every 5 s, events at once (Level 2)
*/

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <DHT.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <stdarg.h>
#include <driver/gpio.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>

// ======================= Identity =======================
#define TEAM_ID     "T04"
#define DEVICE_ID   "T04-D1"
#define FW_VERSION  "1.0.0"

// ======================= Pins ===========================
// Camera pins (4-13, 15-18) and octal-PSRAM pins (35-37) are NOT used.
#define PIN_RGB        48   // onboard WS2812 RGB LED (check your board!)
#define PIN_DHT        41
#define PIN_FLAME      42   // flame module DO
#define PIN_DHT_PWR    45   // DHT11 VCC fed from a GPIO (no breadboard, few 3V3 pins)
#define PIN_DHT_GND     3   // DHT11 GND  -> GPIO held LOW (board has only 1 GND pin)
#define PIN_FLAME_GND   2   // flame GND  -> GPIO held LOW (real GND pin goes to RC522)
#define PIN_BUZZER      1   // active buzzer + leg (wrapped on header pin, no wire)
#define PIN_BUZZER_GND  2   // active buzzer - leg (GPIO held LOW = ground)
#define PIN_TOUCH      14   // touch panic pad T14 (used from Level 3)
#define PIN_RFID_SS    21   // RC522 SDA
#define PIN_RFID_SCK   47
#define PIN_RFID_MOSI  38
#define PIN_RFID_MISO  39
#define PIN_RFID_RST   40
#define PIN_BOOT        0   // onboard BOOT button (hold 5 s = clear Wi-Fi details)

#define FLAME_ACTIVE_LOW 1   // most IR flame modules: DO = LOW when flame seen
#define RGB_BRIGHTNESS   40  // 0-255
#define WDT_TIMEOUT_MS 5000  // restart well inside the 10 s limit

// ======================= Types / shared state ===========
// Device Signals table: lower number wins
enum LightState {
  L_FIRE = 1, L_PANIC, L_GRANTED, L_DENIED, L_DANGER, L_SENSOR_ERR,
  L_CAM_FAULT, L_WX_DANGER, L_WARNING, L_WIFI_SETUP, L_SAFE
};

struct Shared {
  float tempC = NAN;
  float rh = NAN;
  bool  dhtError = false;
  bool  rfidError = false;
  bool  flameError = false;
  bool  flame = false;
  bool  doorUnlocked = false;
  const char* status = "SAFE";   // SAFE / WARNING / DANGER / FIRE (Level 3)
  char  lastUid[24] = "-";       // last card UID, shown on the local status page
  // ---- Level 3 ----
  int   heatLevel = 0;           // 0 SAFE, 1 WARNING, 2 DANGER
  float heatIndex = NAN;
  const char* trend = "STABLE";  // RISING / STABLE / FALLING
  bool  qualityBad = false;      // failed or impossible reading
  uint32_t readCount = 0;        // counts good DHT11 readings
  bool  fire = false;            // latched until Reset fire
  bool  silenced = false;
  bool  panic = false;
  bool  cannotSilence = false;
  char  tapUid[24] = "";
  bool  tapPending = false;
  bool  registerTeacher = false;
};

static Shared g;
static SemaphoreHandle_t gMtx;
static SemaphoreHandle_t gLogMtx;
static volatile bool gHang = false;
static volatile bool gNetOnline = false;   // true while Wi-Fi is connected
static const char* gBootReason = "POWER_ON";

#define LOCK()   xSemaphoreTake(gMtx, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(gMtx)

struct TaskInfo { const char* name; TaskHandle_t h; int core; uint32_t periodMs; };
static TaskInfo gTasks[8];
static int gTaskCount = 0;

DHT dht(PIN_DHT, DHT11);
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);

// ======================= Helpers ========================
void logf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (xSemaphoreTake(gLogMtx, pdMS_TO_TICKS(200)) == pdTRUE) {
    Serial.println(buf);
    xSemaphoreGive(gLogMtx);
  }
}

uint32_t uptimeS() { return (uint32_t)(esp_timer_get_time() / 1000000ULL); }

void setRGB(uint8_t r, uint8_t gg, uint8_t b) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(PIN_RGB, r, gg, b);
#else
  neopixelWrite(PIN_RGB, r, gg, b);
#endif
}

bool readFlame() {
  int v = digitalRead(PIN_FLAME);
  return FLAME_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

// Unplug test for the flame DO wire (active-LOW module with its own pull-up):
//   connected, no flame : pull-up HIGH, pull-down HIGH (module pull-up wins)
//   connected, flame    : pull-up LOW,  pull-down LOW  (module drives LOW)
//   wire unplugged      : pull-up HIGH, pull-down LOW  (pin just follows our pull)
bool flameWireMissing() {
  bool upHigh = digitalRead(PIN_FLAME) == HIGH;    // pin is normally INPUT_PULLUP
  pinMode(PIN_FLAME, INPUT_PULLDOWN);
  delayMicroseconds(50);
  bool downLow = digitalRead(PIN_FLAME) == LOW;
  pinMode(PIN_FLAME, INPUT_PULLUP);
  delayMicroseconds(50);
  return upHigh && downLow;
}

const char* resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "POWER_ON";
    case ESP_RST_SW:       return "SOFTWARE";
    case ESP_RST_TASK_WDT:
    case ESP_RST_INT_WDT:
    case ESP_RST_WDT:      return "WATCHDOG";
    case ESP_RST_PANIC:    return "CRASH";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    default:               return "POWER_ON";   // EXT / USB / unknown
  }
}

void initWatchdog() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {
    .timeout_ms     = WDT_TIMEOUT_MS,
    .idle_core_mask = (1 << 0) | (1 << 1),   // also catch a core that is starved
    .trigger_panic  = true                   // panic -> restart
  };
  if (esp_task_wdt_init(&cfg) == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_reconfigure(&cfg);          // core already started the TWDT
  }
#else
  esp_task_wdt_init(WDT_TIMEOUT_MS / 1000, true);
#endif
}

void startTask(TaskFunction_t fn, const char* name, uint32_t stack,
               UBaseType_t prio, int core, uint32_t periodMs) {
  TaskHandle_t h = nullptr;
  xTaskCreatePinnedToCore(fn, name, stack, nullptr, prio, &h, core);
  gTasks[gTaskCount++] = { name, h, core, periodMs };
}

LightState currentLight() {
  LOCK();
  bool err = g.dhtError || g.rfidError || g.flameError;
  bool fire = g.fire, panic = g.panic;
  int heat = g.heatLevel;
  UNLOCK();
  if (fire)  return L_FIRE;                  // lower number wins
  if (panic) return L_PANIC;
  if (heat == 2) return L_DANGER;
  if (err) return L_SENSOR_ERR;
  if (heat == 1) return L_WARNING;
  if (!gNetOnline) return L_WIFI_SETUP;      // blue: setup mode or offline
  return L_SAFE;
}

void colorFor(LightState s, uint8_t& r, uint8_t& gg, uint8_t& b) {
  const uint8_t B = RGB_BRIGHTNESS;
  r = gg = b = 0;
  switch (s) {
    case L_SENSOR_ERR: r = B; b = B;  break;   // magenta
    case L_CAM_FAULT:  gg = B; b = B; break;   // cyan
    case L_WARNING:    r = B; gg = B; break;   // yellow
    case L_WIFI_SETUP: b = B;         break;   // blue
    case L_DANGER:     r = B;         break;   // red
    case L_WX_DANGER:  r = B / 2; b = B; break;// purple
    case L_SAFE:
    default:           gg = B;        break;   // green
  }
}

// ======================= Status line ====================
void printStatusLine() {
  LOCK();
  float t = g.tempC, h = g.rh;
  bool dErr = g.dhtError, fl = g.flame, door = g.doorUnlocked;
  const char* st = g.status;
  float hi = g.heatIndex;
  const char* tr = g.trend;
  UNLOCK();

  char tStr[12] = "-", rhStr[8] = "-", rssiStr[8] = "-", hiStr[12] = "-";
  if (!isnan(hi)) snprintf(hiStr, sizeof(hiStr), "%.1fC", hi);
  if (gNetOnline) snprintf(rssiStr, sizeof(rssiStr), "%d", (int)WiFi.RSSI());
  if (!dErr && !isnan(t)) snprintf(tStr, sizeof(tStr), "%.1fC", t);
  if (!dErr && !isnan(h)) snprintf(rhStr, sizeof(rhStr), "%d%%", (int)lroundf(h));

  logf("[STATUS] up=%lus T=%s RH=%s HI=%s flame=%d door=%s state=%s trend=%s rssi=%s "
       "heap=%uk psram=%uk chip=%.1fC fw=%s",
       (unsigned long)uptimeS(), tStr, rhStr, hiStr, fl ? 1 : 0,
       door ? "UNLOCKED" : "LOCKED", st, tr, rssiStr,
       (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getFreePsram() / 1024),
       temperatureRead(), FW_VERSION);
}

// ======================= Commands =======================
void handleCommand(const String& cmd) {
  if (cmd == "help") {
    logf("[CMD] commands: help, status, health, tasks, ver, hang, reset, teacher, resetfire, touch | csi (Level 11)"
         " | test only: reboot (SOFTWARE), crash (CRASH)");
  } else if (cmd == "status") {
    printStatusLine();
  } else if (cmd == "health") {
    char rssiStr[8] = "-";
    if (gNetOnline) snprintf(rssiStr, sizeof(rssiStr), "%d", (int)WiFi.RSSI());
    logf("[CMD] health chip=%.1fC heap=%uk psram=%uk flash=%uMB cpu=%uMHz rssi=%s up=%lus reset=%s fw=%s",
         temperatureRead(),
         (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getFreePsram() / 1024),
         (unsigned)(ESP.getFlashChipSize() / (1024 * 1024)), (unsigned)getCpuFrequencyMhz(), rssiStr,
         (unsigned long)uptimeS(), gBootReason, FW_VERSION);
  } else if (cmd == "tasks") {
    for (int i = 0; i < gTaskCount; i++) {
      logf("[CMD] task=%s core=%d period=%lums stack_free=%uB",
           gTasks[i].name, gTasks[i].core, (unsigned long)gTasks[i].periodMs,
           (unsigned)uxTaskGetStackHighWaterMark(gTasks[i].h));   // bytes on ESP-IDF
    }
  } else if (cmd == "ver") {
    logf("[CMD] ver fw=%s build=%s %s", FW_VERSION, __DATE__, __TIME__);
  } else if (cmd == "hang") {
    logf("[CMD] hang: freezing main task, watchdog restart expected in ~%d s", WDT_TIMEOUT_MS / 1000);
    gHang = true;
  } else if (cmd == "teacher") {               // Level 3: the next card tapped becomes a Teacher tag
    LOCK(); g.registerTeacher = true; UNLOCK();
    logf("[CMD] teacher: tap the key tag now to register it as a Teacher tag");
  } else if (cmd == "resetfire") {             // Level 3: test stand-in for the dashboard Reset fire button
    resetFire();
    logf("[CMD] resetfire: fire latch cleared");
  } else if (cmd == "touch") {
    logf("[CMD] touch value=%lu baseline=%lu threshold=%lu", (unsigned long)gTouchLast,
         (unsigned long)gTouchBase, (unsigned long)gTouchThresh);
  } else if (cmd == "reset") {                 // Level 2: clear saved Wi-Fi details
    logf("[CMD] reset: clearing saved Wi-Fi details");
    clearSavedAndRestart();
  } else if (cmd == "reboot") {                // test: BOOT reason=SOFTWARE
    logf("[CMD] reboot: software restart");
    Serial.flush();
    ESP.restart();
  } else if (cmd == "crash") {                 // test: BOOT reason=CRASH
    logf("[CMD] crash: forcing a panic");
    Serial.flush();
    abort();
  } else {
    logf("[CMD] unknown command '%s' (type help)", cmd.c_str());
  }
}

// ===============================================================
//  LOCAL STATUS PAGE (Level 2): the file lives in LittleFS
// ===============================================================
static volatile bool gServerLive = false;   // set true in part 4 when the cloud server answers

const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Classroom</title>
<style>
body{font-family:sans-serif;max-width:460px;margin:16px auto;padding:0 12px;background:#f4f6f8}
h2{margin:8px 0}
table{width:100%;border-collapse:collapse;background:#fff}
td{padding:9px 10px;border-bottom:1px solid #ddd}
td:first-child{color:#555}
td:last-child{text-align:right;font-weight:bold}
#age{color:#777;font-size:13px}
#alert{background:#c00;color:#fff;font-weight:bold;font-size:20px;padding:12px;text-align:center;margin:8px 0}
#alert:empty{display:none}
</style></head><body>
<h2>Smart Classroom <span id="device"></span></h2>
<div id="alert"></div>
<table>
<tr><td>Room ID</td><td id="room">-</td></tr>
<tr><td>Local time (Asia/Dhaka)</td><td id="time">-</td></tr>
<tr><td>Temperature</td><td id="temp">-</td></tr>
<tr><td>Humidity</td><td id="rh">-</td></tr>
<tr><td>Status</td><td id="status">-</td></tr>
<tr><td>Heat index</td><td id="hi">-</td></tr>
<tr><td>Heat trend</td><td id="trend">-</td></tr>
<tr><td>Data quality</td><td id="quality">-</td></tr>
<tr><td>Flame</td><td id="flame">-</td></tr>
<tr><td>Door</td><td id="door">-</td></tr>
<tr><td>Last card UID</td><td id="uid">-</td></tr>
<tr><td>Wi-Fi signal</td><td id="rssi">-</td></tr>
<tr><td>Chip temperature</td><td id="chip">-</td></tr>
<tr><td>Firmware version</td><td id="fw">-</td></tr>
<tr><td>Connection</td><td id="conn">-</td></tr>
</table>
<p id="age">waiting for data...</p>
<script>
let last = 0;
const load = () => fetch('/api/status').then(r => r.json()).then(d => {
  for (const k in d) { const e = document.getElementById(k); if (e) e.textContent = d[k]; }
  last = Date.now();
}).catch(() => {});
setInterval(load, 1000);
setInterval(() => {
  document.getElementById('age').textContent =
    last ? 'updated ' + Math.round((Date.now() - last) / 1000) + ' s ago' : 'waiting for data...';
}, 500);
load();
</script></body></html>
)HTML";

// Store the page in the flash file system (only rewritten when it changed)
void installStatusPage() {
  if (!LittleFS.begin(true)) {               // true = format the first time
    logf("[ERROR] LittleFS mount failed");
    return;
  }
  size_t want = strlen_P(INDEX_HTML);
  File f = LittleFS.open("/index.html", "r");
  bool same = f && f.size() == want;
  if (f) f.close();
  if (!same) {
    f = LittleFS.open("/index.html", "w");
    if (f) { f.print(FPSTR(INDEX_HTML)); f.close(); }
    logf("[NET] status page written to LittleFS (%u bytes)", (unsigned)want);
  }
}

String jsonEscape(String s) {
  s.replace("\\", "\\\\");
  s.replace("\"", "\\\"");
  return s;
}

// ===============================================================
//  WI-FI (Level 2): setup access point, saved details, reconnect
// ===============================================================
Preferences prefs;          // flash storage (NVS)
WebServer   server(80);

enum NetState { NET_SETUP, NET_CONNECTING, NET_ONLINE, NET_RECONNECT };
NetState netState = NET_SETUP;

String cfgSsid, cfgPass, cfgRoom, cfgToken, cfgUrl;
bool   cfgVerified = false;   // true after the first successful connection
String apName, apPass, hostName, setupError;
uint32_t connectStart = 0, lastTry = 0, retryGap = 3000;
bool     pendingConnect = false;
uint32_t pendingSince = 0;

String htmlEscape(String s) {
  s.replace("&", "&amp;"); s.replace("<", "&lt;"); s.replace(">", "&gt;");
  s.replace("\"", "&quot;"); s.replace("'", "&#39;");
  return s;
}

// 8-character random password for the setup access point
String makePassword() {
  const char chars[] = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789";
  String p;
  for (int i = 0; i < 8; i++) p += chars[esp_random() % 32];
  return p;
}

void clearSavedAndRestart() {
  prefs.clear();
  logf("[NET] saved details cleared, restarting in setup mode");
  vTaskDelay(pdMS_TO_TICKS(300));
  ESP.restart();
}

void startSetupMode() {
  gNetOnline = false;
  WiFi.disconnect();
  WiFi.mode(WIFI_AP_STA);                 // AP for the phone, STA for scanning
  WiFi.softAP(apName.c_str(), apPass.c_str());
  netState = NET_SETUP;
  logf("[NET] setup mode ap=%s password=%s page=http://%s",
       apName.c_str(), apPass.c_str(), WiFi.softAPIP().toString().c_str());
}

void startConnect() {
  gNetOnline = false;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);           // we do the retries ourselves
  WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
  connectStart = millis();
  netState = NET_CONNECTING;
  logf("[NET] connecting ssid=%s", cfgSsid.c_str());
}

void onConnected(bool again) {
  gNetOnline = true;
  netState = NET_ONLINE;
  setupError = "";
  if (!cfgVerified) { cfgVerified = true; prefs.putBool("ok", true); }
  logf("[NET] %s ssid=%s ip=%s rssi=%d", again ? "reconnected" : "connected",
       cfgSsid.c_str(), WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
  configTzTime("<+06>-6", "pool.ntp.org", "time.google.com");   // internet time, Asia/Dhaka
  MDNS.end();
  if (MDNS.begin(hostName.c_str())) {
    MDNS.addService("http", "tcp", 80);
    logf("[NET] mdns=http://%s.local", hostName.c_str());
  }
}

// ---------------- Web pages ----------------
void handleRoot() {
  if (netState != NET_SETUP) {
    File f = LittleFS.open("/index.html", "r");   // page file from LittleFS
    if (f) { server.streamFile(f, "text/html"); f.close(); }
    else   server.send(500, "text/plain", "index.html missing in LittleFS");
    return;
  }
  String page =
    "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Device setup</title><style>body{font-family:sans-serif;max-width:420px;margin:20px auto;padding:0 12px}"
    "input,select,button{width:100%;padding:10px;margin:6px 0;box-sizing:border-box}"
    ".err{background:#fdd;border:1px solid #c00;padding:10px}</style></head><body>"
    "<h2>" + apName + " setup</h2>";
  if (setupError.length()) page += "<p class='err'>Error: " + htmlEscape(setupError) + "</p>";

  page += "<form method='POST' action='/save'><label>Wi-Fi network</label><select name='ssid'>";
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    String s = htmlEscape(WiFi.SSID(i));
    page += "<option value='" + s + "'>" + s + " (" + String(WiFi.RSSI(i)) + " dBm)</option>";
  }
  WiFi.scanDelete();
  if (n <= 0) page += "<option value=''>No networks found - reload the page</option>";
  page +=
    "</select><label>Wi-Fi password</label><input name='pass' type='password'>"
    "<label>Room ID</label><input name='room' required value='" + htmlEscape(cfgRoom) + "'>"
    "<label>Device token</label><input name='token' type='password'>"
    "<label>Server URL (https://...)</label><input name='url' value='" + htmlEscape(cfgUrl) + "'>"
    "<button type='submit'>Save and connect</button></form>"
    "<p><a href='/'>Scan again</a></p></body></html>";
  server.send(200, "text/html", page);
}

// Live values for the status page (the page asks for this every second)
void handleApiStatus() {
  LOCK();
  float t = g.tempC, h = g.rh;
  bool dErr = g.dhtError, fl = g.flame, door = g.doorUnlocked;
  const char* st = g.status;
  float hi = g.heatIndex;
  const char* tr = g.trend;
  bool qBad = g.qualityBad, fire = g.fire, panic = g.panic, cannot = g.cannotSilence;
  char uid[24]; strncpy(uid, g.lastUid, sizeof(uid)); uid[sizeof(uid) - 1] = 0;
  UNLOCK();

  char tStr[12] = "-", rhStr[8] = "-", rssiStr[12] = "-", timeStr[24] = "syncing...";
  if (!dErr && !isnan(t)) snprintf(tStr, sizeof(tStr), "%.1f C", t);
  if (!dErr && !isnan(h)) snprintf(rhStr, sizeof(rhStr), "%d %%", (int)lroundf(h));
  if (gNetOnline) snprintf(rssiStr, sizeof(rssiStr), "%d dBm", (int)WiFi.RSSI());
  time_t now = time(nullptr);
  if (now > 1700000000) {                     // clock already set by NTP
    struct tm tmv;
    localtime_r(&now, &tmv);
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &tmv);
  }
  char hiStr[12] = "-";
  if (!isnan(hi)) snprintf(hiStr, sizeof(hiStr), "%.1f C", hi);
  char alert[96] = "";
  if (fire)   strlcat(alert, "FIRE \u2013 EVACUATE", sizeof(alert));
  if (cannot) strlcat(alert, " | CANNOT SILENCE \u2013 FLAME PRESENT", sizeof(alert));
  if (panic)  strlcat(alert, alert[0] ? " | PANIC" : "PANIC", sizeof(alert));
  const char* conn = !gNetOnline ? "OFFLINE" : (gServerLive ? "LIVE" : "NO SERVER");

  char buf[900];
  snprintf(buf, sizeof(buf),
    "{\"device\":\"%s\",\"room\":\"%s\",\"time\":\"%s\",\"temp\":\"%s\",\"rh\":\"%s\","
    "\"status\":\"%s\",\"flame\":\"%s\",\"door\":\"%s\",\"uid\":\"%s\",\"rssi\":\"%s\","
    "\"chip\":\"%.1f C\",\"fw\":\"%s\",\"hi\":\"%s\",\"trend\":\"%s\",\"quality\":\"%s\","
    "\"alert\":\"%s\",\"conn\":\"%s\"}",
    DEVICE_ID, jsonEscape(cfgRoom).c_str(), timeStr, tStr, rhStr, st,
    fl ? "YES" : "no", door ? "UNLOCKED" : "LOCKED", uid, rssiStr,
    temperatureRead(), FW_VERSION, hiStr, tr, (qBad || dErr) ? "bad" : "good", alert, conn);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

void handleSave() {
  if (netState != NET_SETUP || server.arg("ssid").length() == 0) {
    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }
  cfgSsid  = server.arg("ssid");
  cfgPass  = server.arg("pass");
  cfgRoom  = server.arg("room");
  cfgToken = server.arg("token");
  cfgUrl   = server.arg("url");
  cfgUrl.trim();
  cfgVerified = false;
  prefs.putString("ssid", cfgSsid);
  prefs.putString("pass", cfgPass);
  prefs.putString("room", cfgRoom);
  prefs.putString("token", cfgToken);
  prefs.putString("url", cfgUrl);
  prefs.putBool("ok", false);
  logf("[NET] details saved ssid=%s room=%s", cfgSsid.c_str(), cfgRoom.c_str());
  server.send(200, "text/html",
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<h2>Saved</h2><p>Connecting to " + htmlEscape(cfgSsid) + "...</p>"
    "<p>If it fails, this setup network comes back within 15 seconds and shows the error.</p>");
  pendingConnect = true;          // connect a moment later, so this page is sent first
  pendingSince = millis();
}

// ---------------- Network task (core 0) ----------------
void netTask(void*) {
  esp_task_wdt_add(NULL);
  uint32_t bootHeld = 0;

  if (cfgSsid.length()) startConnect();
  else                  startSetupMode();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.begin();

  for (;;) {
    server.handleClient();

    // BOOT button held 5 s -> clear saved details
    if (digitalRead(PIN_BOOT) == LOW) {
      if (bootHeld == 0) bootHeld = millis();
      else if (millis() - bootHeld >= 5000) clearSavedAndRestart();
    } else {
      bootHeld = 0;
    }

    if (pendingConnect && millis() - pendingSince >= 1000) {
      pendingConnect = false;
      startConnect();
    }

    switch (netState) {
      case NET_SETUP:
        break;

      case NET_CONNECTING:
        if (WiFi.status() == WL_CONNECTED) {
          onConnected(false);
        } else if (millis() - connectStart >= 12000) {
          if (!cfgVerified) {
            // new details that never worked -> back to setup mode (within 15 s)
            setupError = "Could not connect to '" + cfgSsid +
                         "'. Wrong password or network not reachable.";
            logf("[NET] setup error: %s", setupError.c_str());
            startSetupMode();
          } else {
            // details worked before (e.g. after a power cut) -> keep trying
            netState = NET_RECONNECT;
            retryGap = 3000;
            lastTry = millis() - retryGap;
          }
        }
        break;

      case NET_ONLINE:
        if (WiFi.status() != WL_CONNECTED) {
          gNetOnline = false;
          logf("[NET] disconnected ssid=%s", cfgSsid.c_str());
          netState = NET_RECONNECT;
          retryGap = 3000;
          lastTry = millis() - retryGap;      // first retry at once
        }
        break;

      case NET_RECONNECT:
        if (WiFi.status() == WL_CONNECTED) {
          onConnected(true);
        } else if (millis() - lastTry >= retryGap) {
          logf("[NET] reconnecting ssid=%s (gap %lus)", cfgSsid.c_str(),
               (unsigned long)(retryGap / 1000));
          WiFi.disconnect();
          WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
          lastTry = millis();
          retryGap = min<uint32_t>(retryGap * 2, 10000);   // 3 s, 6 s, 10 s, 10 s...
        }
        break;
    }

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ===============================================================
//  CLOUD (Level 2): HTTPS telemetry and events
// ===============================================================
// Public root certificates used to check the server's certificate.
// These are public files, not secrets.
const char ROOT_CAS[] PROGMEM = R"PEM(
-----BEGIN CERTIFICATE-----
MIIFVzCCAz+gAwIBAgINAgPlk28xsBNJiGuiFzANBgkqhkiG9w0BAQwFADBHMQsw
CQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEU
MBIGA1UEAxMLR1RTIFJvb3QgUjEwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAw
MDAwWjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZp
Y2VzIExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjEwggIiMA0GCSqGSIb3DQEBAQUA
A4ICDwAwggIKAoICAQC2EQKLHuOhd5s73L+UPreVp0A8of2C+X0yBoJx9vaMf/vo
27xqLpeXo4xL+Sv2sfnOhB2x+cWX3u+58qPpvBKJXqeqUqv4IyfLpLGcY9vXmX7w
Cl7raKb0xlpHDU0QM+NOsROjyBhsS+z8CZDfnWQpJSMHobTSPS5g4M/SCYe7zUjw
TcLCeoiKu7rPWRnWr4+wB7CeMfGCwcDfLqZtbBkOtdh+JhpFAz2weaSUKK0Pfybl
qAj+lug8aJRT7oM6iCsVlgmy4HqMLnXWnOunVmSPlk9orj2XwoSPwLxAwAtcvfaH
szVsrBhQf4TgTM2S0yDpM7xSma8ytSmzJSq0SPly4cpk9+aCEI3oncKKiPo4Zor8
Y/kB+Xj9e1x3+naH+uzfsQ55lVe0vSbv1gHR6xYKu44LtcXFilWr06zqkUspzBmk
MiVOKvFlRNACzqrOSbTqn3yDsEB750Orp2yjj32JgfpMpf/VjsPOS+C12LOORc92
wO1AK/1TD7Cn1TsNsYqiA94xrcx36m97PtbfkSIS5r762DL8EGMUUXLeXdYWk70p
aDPvOmbsB4om3xPXV2V4J95eSRQAogB/mqghtqmxlbCluQ0WEdrHbEg8QOB+DVrN
VjzRlwW5y0vtOUucxD/SVRNuJLDWcfr0wbrM7Rv1/oFB2ACYPTrIrnqYNxgFlQID
AQABo0IwQDAOBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4E
FgQU5K8rJnEaK0gnhS9SZizv8IkTcT4wDQYJKoZIhvcNAQEMBQADggIBAJ+qQibb
C5u+/x6Wki4+omVKapi6Ist9wTrYggoGxval3sBOh2Z5ofmmWJyq+bXmYOfg6LEe
QkEzCzc9zolwFcq1JKjPa7XSQCGYzyI0zzvFIoTgxQ6KfF2I5DUkzps+GlQebtuy
h6f88/qBVRRiClmpIgUxPoLW7ttXNLwzldMXG+gnoot7TiYaelpkttGsN/H9oPM4
7HLwEXWdyzRSjeZ2axfG34arJ45JK3VmgRAhpuo+9K4l/3wV3s6MJT/KYnAK9y8J
ZgfIPxz88NtFMN9iiMG1D53Dn0reWVlHxYciNuaCp+0KueIHoI17eko8cdLiA6Ef
MgfdG+RCzgwARWGAtQsgWSl4vflVy2PFPEz0tv/bal8xa5meLMFrUKTX5hgUvYU/
Z6tGn6D/Qqc6f1zLXbBwHSs09dR2CQzreExZBfMzQsNhFRAbd03OIozUhfJFfbdT
6u9AWpQKXCBfTkBdYiJ23//OYb2MI3jSNwLgjt7RETeJ9r/tSQdirpLsQBqvFAnZ
0E6yove+7u7Y/9waLd64NnHi/Hm3lCXRSHNboTXns5lndcEZOitHTtNCjv0xyBZm
2tIMPNuzjsmhDYAPexZ3FL//2wmUspO8IFgV6dtxQ/PeEMMA3KgqlbbC1j+Qa3bb
bP6MvPJwNQzcmRk13NfIRmPVNnGuV/u3gm3c
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIICCTCCAY6gAwIBAgINAgPlwGjvYxqccpBQUjAKBggqhkjOPQQDAzBHMQswCQYD
VQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEUMBIG
A1UEAxMLR1RTIFJvb3QgUjQwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAwMDAw
WjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2Vz
IExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjQwdjAQBgcqhkjOPQIBBgUrgQQAIgNi
AATzdHOnaItgrkO4NcWBMHtLSZ37wWHO5t5GvWvVYRg1rkDdc/eJkTBa6zzuhXyi
QHY7qca4R9gq55KRanPpsXI5nymfopjTX15YhmUPoYRlBtHci8nHc8iMai/lxKvR
HYqjQjBAMA4GA1UdDwEB/wQEAwIBhjAPBgNVHRMBAf8EBTADAQH/MB0GA1UdDgQW
BBSATNbrdP9JNqPV2Py1PsVq8JQdjDAKBggqhkjOPQQDAwNpADBmAjEA6ED/g94D
9J+uHXqnLrmvT/aDHQ4thQEd0dlq7A/Cr8deVl5c1RxYIigL9zC2L7F8AjEA8GE8
p/SgguMh1YQdc4acLa/KNJvxn7kjNuK8YAOdgLOaVsjh4rsUecrNIdSUtUlD
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIICGzCCAaGgAwIBAgIQQdKd0XLq7qeAwSxs6S+HUjAKBggqhkjOPQQDAzBPMQsw
CQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJuZXQgU2VjdXJpdHkgUmVzZWFyY2gg
R3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBYMjAeFw0yMDA5MDQwMDAwMDBaFw00
MDA5MTcxNjAwMDBaME8xCzAJBgNVBAYTAlVTMSkwJwYDVQQKEyBJbnRlcm5ldCBT
ZWN1cml0eSBSZXNlYXJjaCBHcm91cDEVMBMGA1UEAxMMSVNSRyBSb290IFgyMHYw
EAYHKoZIzj0CAQYFK4EEACIDYgAEzZvVn4CDCuwJSvMWSj5cz3es3mcFDR0HttwW
+1qLFNvicWDEukWVEYmO6gbf9yoWHKS5xcUy4APgHoIYOIvXRdgKam7mAHf7AlF9
ItgKbppbd9/w+kHsOdx1ymgHDB/qo0IwQDAOBgNVHQ8BAf8EBAMCAQYwDwYDVR0T
AQH/BAUwAwEB/zAdBgNVHQ4EFgQUfEKWrt5LSDv6kviejM9ti6lyN5UwCgYIKoZI
zj0EAwMDaAAwZQIwe3lORlCEwkSHRhtFcP9Ymd70/aTSVaYgLXTWNLxBo1BfASdW
tL4ndQavEi51mI38AjEAi/V3bNTIZargCyzuFJ0nN6T5U6VR5CmD1/iQMVtCnwr1
/q4AaOeMSQ+2b1tbFfLn
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI
2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx
1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ
q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz
tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ
vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP
BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV
5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY
1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4
NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG
Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91
8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe
pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl
MrY=
-----END CERTIFICATE-----
)PEM";

// ---- seq: only increases, never reused, even across restarts ----
// A block of 100 numbers is reserved in flash each time, so a power cut
// can skip numbers but can never repeat one.
Preferences seqPrefs;
static SemaphoreHandle_t gSeqMtx;
static uint32_t gSeq = 1, gSeqLimit = 1;

void initSeq() {
  gSeqMtx = xSemaphoreCreateMutex();
  seqPrefs.begin("seq", false);              // own namespace: "reset" does not clear it
  gSeq = seqPrefs.getULong("next", 1);
  gSeqLimit = gSeq + 100;
  seqPrefs.putULong("next", gSeqLimit);
}

uint32_t nextSeq() {
  xSemaphoreTake(gSeqMtx, portMAX_DELAY);
  uint32_t s = gSeq++;
  if (gSeq >= gSeqLimit) {
    gSeqLimit += 100;
    seqPrefs.putULong("next", gSeqLimit);
  }
  xSemaphoreGive(gSeqMtx);
  return s;
}

// ---- events wait in a queue until the server has stored them ----
struct CloudEvent {
  char type[20];
  char detail[48];
  uint32_t seq;
  uint32_t ts;      // Unix time when it happened (0 = clock not set yet)
  uint32_t ms;      // millis() when it happened
};
static QueueHandle_t gEventQ;

void queueEvent(const char* type, const char* detail, bool print) {
  CloudEvent ev = {};
  strncpy(ev.type, type, sizeof(ev.type) - 1);
  strncpy(ev.detail, detail, sizeof(ev.detail) - 1);
  ev.seq = nextSeq();
  time_t now = time(nullptr);
  ev.ts = now > 1700000000 ? (uint32_t)now : 0;
  ev.ms = millis();
  if (print) logf("[EVENT] %s %s", type, detail);
  if (xQueueSend(gEventQ, &ev, 0) != pdTRUE) logf("[ERROR] event queue full, %s dropped", type);
}

WiFiClientSecure tlsClient;
HTTPClient http;
static int gLastHttp = 0;

// One HTTPS POST. Returns the HTTP code (negative = no connection).
int postJson(const char* body) {
  http.setReuse(true);                       // keep the TLS connection open between posts
  http.setConnectTimeout(4000);
  http.setTimeout(4000);
  if (!http.begin(tlsClient, cfgUrl)) return -1;
  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-device-token", cfgToken);
  int code = http.POST((uint8_t*)body, strlen(body));
  if (code > 0) http.getString();            // read the reply so the connection can be reused
  http.end();
  return code;
}

// Logs changes of the server connection and tells if the message is done.
bool delivered(int code, uint32_t seq) {
  bool live = (code == 200 || code == 409);
  if (code != gLastHttp) {
    if (live && !gServerLive)       logf("[NET] server connected");
    else if (code == 401)           logf("[ERROR] server refused the device token (http=401)");
    else if (!live && code != 401)  logf("[NET] server disconnected (http=%d)", code);
    gLastHttp = code;
  }
  if (code == 409) logf("[NET] server rejected seq=%lu as a replay", (unsigned long)seq);
  gServerLive = live;
  return live;
}

void cloudTask(void*) {
  // Not on the task watchdog: an HTTPS call may wait several seconds for the network.
  tlsClient.setCACert(ROOT_CAS);             // check the server's certificate
  uint32_t lastTel = 0;
  bool warned = false;
  char body[512];

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));

    if (!gNetOnline) {
      if (gServerLive) { gServerLive = false; gLastHttp = 0; logf("[NET] server disconnected (no Wi-Fi)"); }
      continue;
    }
    time_t now = time(nullptr);
    if (now < 1700000000) continue;          // wait for internet time (NTP)
    if (!cfgUrl.startsWith("https://")) {
      if (!warned) { warned = true; logf("[NET] no server URL saved: type reset and enter it on the setup page"); }
      continue;
    }

    // ---- events first, at once ----
    CloudEvent ev;
    while (xQueuePeek(gEventQ, &ev, 0) == pdTRUE) {
      uint32_t ts = ev.ts ? ev.ts : (uint32_t)time(nullptr) - (millis() - ev.ms) / 1000;
      snprintf(body, sizeof(body),
        "{\"device_id\":\"%s\",\"room_id\":\"%s\",\"seq\":%lu,\"ts\":%lu,\"type\":\"%s\",\"detail\":\"%s\"}",
        DEVICE_ID, jsonEscape(cfgRoom).c_str(), (unsigned long)ev.seq, (unsigned long)ts,
        ev.type, ev.detail);
      if (delivered(postJson(body), ev.seq)) xQueueReceive(gEventQ, &ev, 0);
      else break;                            // keep it, try again next round
    }

    // ---- telemetry every 5 s ----
    if (lastTel == 0 || millis() - lastTel >= 5000) {
      lastTel = millis();
      LOCK();
      float t = g.tempC, h = g.rh;
      bool dErr = g.dhtError, fl = g.flame, door = g.doorUnlocked;
      const char* st = g.status;
      float hi = g.heatIndex;
      const char* tr = g.trend;
      bool qBad = g.qualityBad;
      UNLOCK();
      bool good = !dErr && !qBad && !isnan(t) && !isnan(h);
      char hiStr[12] = "null";
      if (good && !isnan(hi)) snprintf(hiStr, sizeof(hiStr), "%.1f", hi);
      char tStr[12] = "null", rhStr[8] = "null";
      if (good) {
        snprintf(tStr, sizeof(tStr), "%.1f", t);
        snprintf(rhStr, sizeof(rhStr), "%d", (int)lroundf(h));
      }
      uint32_t seq = nextSeq();
      snprintf(body, sizeof(body),
        "{\"device_id\":\"%s\",\"room_id\":\"%s\",\"seq\":%lu,\"ts\":%lu,\"uptime_s\":%lu,"
        "\"temp_c\":%s,\"rh\":%s,\"heat_index_c\":%s,\"flame\":%s,\"door\":\"%s\","
        "\"status\":\"%s\",\"heat_trend\":\"%s\",\"rssi\":%d,\"heap_kb\":%u,"
        "\"chip_temp_c\":%.1f,\"fw\":\"%s\",\"quality\":\"%s\"}",
        DEVICE_ID, jsonEscape(cfgRoom).c_str(), (unsigned long)seq,
        (unsigned long)time(nullptr), (unsigned long)uptimeS(), tStr, rhStr, hiStr,
        fl ? "true" : "false", door ? "UNLOCKED" : "LOCKED", st, tr, (int)WiFi.RSSI(),
        (unsigned)(ESP.getFreeHeap() / 1024), temperatureRead(), FW_VERSION,
        good ? "good" : "bad");
      delivered(postJson(body), seq);
    }
  }
}

// ===============================================================
//  SAFETY (Level 3): heat status, fire latch, Teacher silence, panic
// ===============================================================
Preferences safePrefs;                     // flash: fire latch and Teacher tag list
static String gTeacherTags = ",";          // ",UID,UID,"
static uint32_t gTouchBase = 0, gTouchThresh = 0, gTouchLast = 0;
static const char* HEAT_NAMES[] = { "SAFE", "WARNING", "DANGER" };

bool isTeacherTag(const char* uid) {
  return gTeacherTags.indexOf(String(",") + uid + ",") >= 0;
}

void initSafety() {
  safePrefs.begin("safety", false);
  gTeacherTags = safePrefs.getString("teachers", ",");

  // Fire state survives a power cut: restart with the door still UNLOCKED
  if (safePrefs.getBool("fire", false)) {
    LOCK(); g.fire = true; g.doorUnlocked = true; g.status = "FIRE"; UNLOCK();
    logf("[EVENT] FIRE restored from flash, door UNLOCKED");
  }

  // Touch pad: measure the untouched level at start-up
  for (int i = 0; i < 5; i++) { touchRead(PIN_TOUCH); delay(20); }
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) { sum += touchRead(PIN_TOUCH); delay(20); }
  gTouchBase = sum / 16;
  gTouchThresh = gTouchBase + gTouchBase / 5;          // touched = 20 % above the baseline
  logf("[EVENT] TOUCH baseline=%lu threshold=%lu", (unsigned long)gTouchBase, (unsigned long)gTouchThresh);
}

// Reset fire (Teacher or Admin; from the dashboard in Level 4, Serial command for now)
void resetFire() {
  LOCK();
  g.fire = false; g.silenced = false;
  if (!g.flame) g.doorUnlocked = false;
  UNLOCK();
  safePrefs.putBool("fire", false);
}

// Called every 50 ms by mainTask
void safetyTick() {
  static int heat = 0, cand = 0;
  static uint32_t candSince = 0;
  static float hist[10];
  static int histN = 0;
  static uint32_t lastCount = 0;
  static uint32_t flameSince = 0, noFlameSince = 0;
  static bool clearedSent = true;
  static uint32_t touchSince = 0, panicEnd = 0, cannotEnd = 0;
  static bool touchLatched = false, panicOn = false, cannotOn = false;
  static uint32_t n = 0;

  uint32_t now = millis();
  LOCK();
  float t = g.tempC, h = g.rh;
  bool qBad = g.qualityBad, dErr = g.dhtError, flame = g.flame;
  bool fire = g.fire, silenced = g.silenced, reg = g.registerTeacher;
  uint32_t cnt = g.readCount;
  bool tap = g.tapPending;
  char uid[24];
  strncpy(uid, g.tapUid, sizeof(uid)); uid[sizeof(uid) - 1] = 0;
  g.tapPending = false;
  UNLOCK();
  bool valid = !qBad && !dErr && !isnan(t) && !isnan(h);

  // ---- new good reading: heat index and trend (last 10 readings) ----
  if (cnt != lastCount) {
    lastCount = cnt;
    if (histN < 10) hist[histN++] = t;
    else { for (int i = 0; i < 9; i++) hist[i] = hist[i + 1]; hist[9] = t; }
    float hi = dht.computeHeatIndex(t, h, false);       // Celsius
    const char* tr = "STABLE";
    if (histN >= 3) {
      float d = hist[histN - 1] - hist[0];
      if (d >= 0.5f) tr = "RISING"; else if (d <= -0.5f) tr = "FALLING";
    }
    LOCK(); g.heatIndex = hi; g.trend = tr; UNLOCK();    // trend never touches light, buzzer or door
  }

  // ---- heat status: thresholds, hysteresis, 3 s hold ----
  if (valid) {                                          // a bad reading keeps the last status
    int target = heat;
    if (heat == 0)      { if (t >= 40) target = 2; else if (t >= 32 || h >= 85) target = 1; }
    else if (heat == 1) { if (t >= 40) target = 2; else if (t <= 30 && h <= 80) target = 0; }
    else                { if (t <= 38) target = 1; }
    if (target == heat) {
      cand = heat;
    } else if (target != cand) {
      cand = target; candSince = now;
    } else if (now - candSince >= 3000) {
      char d[32];
      snprintf(d, sizeof(d), "%s->%s", HEAT_NAMES[heat], HEAT_NAMES[target]);
      heat = target;
      queueEvent("STATUS", d, true);
    }
  }

  // ---- fire: flame seen continuously for 2 s, latched, saved in flash ----
  if (flame) {
    if (!flameSince) flameSince = now | 1;
    noFlameSince = 0;
    if (fire && clearedSent) {                          // flame came back: alarm on again
      clearedSent = false; silenced = false;
      LOCK(); g.silenced = false; UNLOCK();
    }
    if (!fire && now - flameSince >= 2000) {
      fire = true; silenced = false; clearedSent = false;
      LOCK(); g.fire = true; g.silenced = false; g.doorUnlocked = true; UNLOCK();
      safePrefs.putBool("fire", true);
      queueEvent("FIRE", "flame held 2 s", true);
    }
  } else {
    flameSince = 0;
    if (fire && !clearedSent) {
      if (!noFlameSince) noFlameSince = now | 1;
      if (now - noFlameSince >= 1000) { clearedSent = true; queueEvent("FIRE_CLEARED", "flame gone", true); }
    }
  }

  // ---- card tap: register a Teacher tag, or Teacher silences the alarm ----
  if (tap) {
    if (reg) {
      if (!isTeacherTag(uid)) {
        gTeacherTags += String(uid) + ",";
        safePrefs.putString("teachers", gTeacherTags);
      }
      LOCK(); g.registerTeacher = false; UNLOCK();
      logf("[CMD] teacher tag registered uid=%s", uid);
    } else if (fire && isTeacherTag(uid)) {             // a student card never silences
      if (flame) {
        cannotOn = true; cannotEnd = now + 5000;
        logf("[EVENT] SILENCE_REFUSED flame present");
      } else if (!silenced) {
        silenced = true;
        LOCK(); g.silenced = true; UNLOCK();
        queueEvent("SILENCED", uid, true);
      }
    }
  }

  // ---- touch panic pad: 2 s touch = PANIC, shorter than 1 s is ignored ----
  gTouchLast = touchRead(PIN_TOUCH);
  if (gTouchThresh && gTouchLast > gTouchThresh) {
    if (!touchSince) touchSince = now | 1;
    if (!touchLatched && now - touchSince >= 2000) {
      touchLatched = true; panicOn = true; panicEnd = now + 10000;
      queueEvent("PANIC", "touch pad", true);
    }
  } else {
    touchSince = 0; touchLatched = false;
  }
  if (panicOn && (int32_t)(now - panicEnd) >= 0) panicOn = false;
  if (cannotOn && (int32_t)(now - cannotEnd) >= 0) cannotOn = false;

  // ---- publish ----
  LOCK();
  g.heatLevel = heat;
  g.status = fire ? "FIRE" : HEAT_NAMES[heat];
  g.panic = panicOn;
  g.cannotSilence = cannotOn;
  if (fire) g.doorUnlocked = true;                      // fire overrides every other door rule
  UNLOCK();

  // ---- buzzer (one tick = 50 ms) ----
  n++;
  bool bz = false;
  if (fire && !silenced) bz = (n % 8) < 4;              // 200 ms on / 200 ms off, until silenced
  else if (panicOn)      bz = (n % 4) < 2;              // 100 ms on / 100 ms off for 10 s
  else if (heat == 2)    bz = (n % 40) < 2;             // one short beep every 2 s
  digitalWrite(PIN_BUZZER, bz ? HIGH : LOW);
}

// ======================= Tasks ==========================
void ledTask(void*) {
  esp_task_wdt_add(NULL);
  TickType_t last = xTaskGetTickCount();
  uint32_t n = 0, lastRgb = 0xFFFFFFFF;
  for (;;) {
    esp_task_wdt_reset();
    LightState s = currentLight();
    uint32_t ms = n * 50;
    uint8_t r = 0, gg = 0, b = 0;
    if (s == L_FIRE) {                     // red, 5 flashes a second
      if ((ms % 200) < 100) r = RGB_BRIGHTNESS;
    } else if (s == L_PANIC) {             // red and blue alternating
      if ((ms % 500) < 250) r = RGB_BRIGHTNESS; else b = RGB_BRIGHTNESS;
    } else {
      bool on = (ms % 500) < 250;          // 2 Hz: 250 ms on / 250 ms off
      if (on) colorFor(s, r, gg, b);
    }
    uint32_t rgb = ((uint32_t)r << 16) | ((uint32_t)gg << 8) | b;
    if (rgb != lastRgb) { setRGB(r, gg, b); lastRgb = rgb; }
    n++;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
  }
}

void sensorTask(void*) {
  esp_task_wdt_add(NULL);
  TickType_t last = xTaskGetTickCount();
  uint32_t tick = 0, nextDht = 15;     // first DHT11 read after 1.5 s warm-up
  uint8_t dhtFails = 0, flameBad = 0;
  bool flameErr = false;
  for (;;) {
    esp_task_wdt_reset();

    // ---- flame every 100 ms ----
    bool f = readFlame();

    // ---- flame wire check every 500 ms; error after 2 bad checks (~1 s) ----
    if (tick % 5 == 0) {
      if (flameWireMissing()) {
        if (++flameBad >= 2 && !flameErr) {
          flameErr = true;
          LOCK(); g.flameError = true; UNLOCK();
          logf("[ERROR] SENSOR ERROR: FLAME");
          logf("[EVENT] SENSOR_ERROR FLAME");
        }
      } else {
        flameBad = 0;
        if (flameErr) {
          flameErr = false;
          LOCK(); g.flameError = false; UNLOCK();
          logf("[EVENT] SENSOR_ERROR FLAME CLEARED");
        }
      }
    }
    if (flameErr) f = false;               // never trust a missing sensor
    LOCK(); g.flame = f; UNLOCK();

    // ---- DHT11 every 5 s (quick 2 s retry after a failure) ----
    if (tick >= nextDht) {
      bool ok = dht.read(true);
      float t = dht.readTemperature();
      float h = dht.readHumidity();
      if (ok && !isnan(t) && !isnan(h)) {
        dhtFails = 0;
        bool inRange = (t >= 0 && t <= 50 && h >= 20 && h <= 90);   // impossible values are never used
        LOCK(); bool was = g.dhtError; g.dhtError = false;
        if (inRange) { g.tempC = t; g.rh = h; g.qualityBad = false; g.readCount++; }
        else         { g.qualityBad = true; }
        UNLOCK();
        if (was) logf("[EVENT] SENSOR_ERROR DHT11 CLEARED");
        nextDht = tick + 50;               // 50 x 100 ms = 5 s
      } else {
        dhtFails++;
        LOCK(); bool was = g.dhtError; g.qualityBad = true; if (dhtFails >= 2) g.dhtError = true; UNLOCK();
        if (dhtFails >= 2 && !was) {       // worst case about 7 s after unplug
          logf("[ERROR] SENSOR ERROR: DHT11");
          logf("[EVENT] SENSOR_ERROR DHT11");
        }
        nextDht = tick + 20;               // retry after 2 s
      }
    }

    tick++;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
  }
}

// A floating SDA (NSS) line does not always kill the reader: sometimes reads
// still work by luck. Several write/read-back patterns catch that reliably,
// because with NSS stuck low the chip mis-frames every write.
bool rfidChipOk() {
  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  if (v == 0x00 || v == 0xFF) return false;
  static const byte pat[] = { 0x55, 0xAA, 0x33, 0xCC, 0xE8 };   // 0xE8 = normal value
  for (byte p : pat) {
    rfid.PCD_WriteRegister(MFRC522::TReloadRegL, p);
    if (rfid.PCD_ReadRegister(MFRC522::TReloadRegL) != p) {
      rfid.PCD_WriteRegister(MFRC522::TReloadRegL, 0xE8);
      return false;
    }
  }
  return true;
}

void rfidTask(void*) {
  esp_task_wdt_add(NULL);
  TickType_t last = xTaskGetTickCount();
  uint32_t tick = 0;
  uint8_t badChecks = 0, goodRun = 0, misses = 0;
  bool err = false, present = false;

  for (;;) {
    esp_task_wdt_reset();

    // ---- reader health every 200 ms ----
    // Score: bad +2, good -1. Error at score >= 4:
    //   steady fault  -> 2 checks  (~0.4 s)
    //   flaky fault   -> ~5 checks (~1 s)   both well under 3 s
    // Clear only after 10 good checks in a row (2 s), then re-init the chip.
    if (!rfidChipOk()) {
      badChecks = (badChecks > 250) ? badChecks : badChecks + 2;
      goodRun = 0;
      if (badChecks >= 4 && !err) {
        err = true; present = false;
        LOCK(); g.rfidError = true; UNLOCK();
        logf("[ERROR] SENSOR ERROR: RFID");
        logf("[EVENT] SENSOR_ERROR RFID");
      }
    } else {
      if (badChecks) badChecks--;
      if (goodRun < 255) goodRun++;
      if (err && goodRun >= 10) {
        err = false; badChecks = 0;
        rfid.PCD_Init();                   // restore registers after reconnect
        LOCK(); g.rfidError = false; UNLOCK();
        logf("[EVENT] SENSOR_ERROR RFID CLEARED");
      }
    }

    // ---- card poll every 200 ms, report ONCE per tap ----
    if (!err) {
      byte atqa[2]; byte sz = sizeof(atqa);
      bool seen = false;
      char uidStr[24] = {0};
      // WUPA also wakes a HALTed card, so a held card stays "present"
      MFRC522::StatusCode s = rfid.PICC_WakeupA(atqa, &sz);
      if (s == MFRC522::STATUS_OK || s == MFRC522::STATUS_COLLISION) {
        if (rfid.PICC_ReadCardSerial()) {
          seen = true;
          for (byte i = 0; i < rfid.uid.size && i < 10; i++)
            sprintf(uidStr + i * 2, "%02X", rfid.uid.uidByte[i]);
        }
        rfid.PICC_HaltA();
      }
      if (seen) {
        misses = 0;
        if (!present) {
          present = true;
          logf("[CARD] uid=%s", uidStr);
          LOCK(); strncpy(g.lastUid, uidStr, sizeof(g.lastUid) - 1); UNLOCK();
          queueEvent("CARD_TAP", uidStr, true);
          LOCK(); strncpy(g.tapUid, uidStr, sizeof(g.tapUid) - 1); g.tapPending = true; UNLOCK();
          // Level 2+: send CARD_TAP event here
        }
      } else if (present && ++misses >= 3) {   // ~600 ms gone = card removed
        present = false;
      }
    }

    tick++;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(200));
  }
}

void mainTask(void*) {
  esp_task_wdt_add(NULL);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    if (gHang) {
      logf("[EVENT] HANG main task frozen (no watchdog feed)");
      volatile uint32_t spin = 0;
      for (;;) { spin++; }                 // real freeze: no feed, no yield
    }
    esp_task_wdt_reset();

    safetyTick();                          // Level 3: status, fire, panic, buzzer

    vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
  }
}

void statusTask(void*) {
  esp_task_wdt_add(NULL);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    esp_task_wdt_reset();
    printStatusLine();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));
  }
}

void serialTask(void*) {
  esp_task_wdt_add(NULL);
  String line;
  line.reserve(64);
  for (;;) {
    esp_task_wdt_reset();
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        line.trim();
        line.toLowerCase();
        if (line.length()) handleCommand(line);
        line = "";
      } else if (line.length() < 63) {
        line += c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ======================= Setup ==========================
void setup() {
  // GPIO "grounds" first, so the DHT11 and flame module are powered early
  pinMode(PIN_DHT_GND, OUTPUT);   digitalWrite(PIN_DHT_GND, LOW);
  pinMode(PIN_FLAME_GND, OUTPUT); digitalWrite(PIN_FLAME_GND, LOW);
  gpio_set_drive_capability((gpio_num_t)PIN_DHT_GND, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)PIN_FLAME_GND, GPIO_DRIVE_CAP_3);

  Serial.begin(115200);
  // Short, bounded wait so the BOOT line is not lost on native-USB boards
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) { delay(10); }

  gLogMtx = xSemaphoreCreateMutex();
  gMtx    = xSemaphoreCreateMutex();

  gBootReason = resetReasonStr();
  logf("[EVENT] BOOT reason=%s", gBootReason);

  pinMode(PIN_FLAME, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_BUZZER_GND, OUTPUT);
  digitalWrite(PIN_BUZZER_GND, LOW);                 // acts as buzzer ground
  gpio_set_drive_capability((gpio_num_t)PIN_BUZZER, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)PIN_BUZZER_GND, GPIO_DRIVE_CAP_3);
  setRGB(0, 0, 0);

  pinMode(PIN_DHT_PWR, OUTPUT);
  digitalWrite(PIN_DHT_PWR, HIGH);                   // power the DHT11 (~1.5 mA)
  dht.begin();                                       // first read happens >1 s later

  SPI.begin(PIN_RFID_SCK, PIN_RFID_MISO, PIN_RFID_MOSI, PIN_RFID_SS);
  rfid.PCD_Init();

  // ---- Start-up flame check FIRST: no flame -> LOCKED, flame -> UNLOCKED ----
  int hits = 0;
  for (int i = 0; i < 5; i++) { if (readFlame()) hits++; delay(10); }
  bool flameAtBoot = hits >= 3;
  g.flame = flameAtBoot;
  g.doorUnlocked = flameAtBoot;
  logf("[EVENT] DOOR %s (start-up flame check: flame=%d)",
       flameAtBoot ? "UNLOCKED" : "LOCKED", flameAtBoot ? 1 : 0);

  // ---- Level 2: saved Wi-Fi details from flash (NVS) ----
  pinMode(PIN_BOOT, INPUT_PULLUP);
  prefs.begin("cfg", false);
  cfgSsid     = prefs.getString("ssid", "");
  cfgPass     = prefs.getString("pass", "");
  cfgRoom     = prefs.getString("room", "");
  cfgToken    = prefs.getString("token", "");
  cfgUrl      = prefs.getString("url", "");
  cfgVerified = prefs.getBool("ok", false);
  apName   = String("SmartClass-") + TEAM_ID;
  apPass   = makePassword();
  hostName = String("smartclass-") + TEAM_ID;
  hostName.toLowerCase();
  installStatusPage();
  initSeq();
  gEventQ = xQueueCreate(20, sizeof(CloudEvent));
  queueEvent("BOOT", gBootReason, false);      // already printed as [EVENT] BOOT reason=...
  initSafety();

  initWatchdog();

  // Core 1: real-time side
  startTask(ledTask,    "led",    3072, 4, 1, 50);
  startTask(sensorTask, "sensor", 4096, 3, 1, 100);
  startTask(mainTask,   "main",   4096, 2, 1, 50);
  // Core 0: comms side
  startTask(rfidTask,   "rfid",   4096, 2, 0, 200);
  startTask(statusTask, "status", 4096, 1, 0, 1000);
  startTask(serialTask, "serial", 4096, 1, 0, 20);
  startTask(netTask,    "net",    8192, 1, 0, 10);
  startTask(cloudTask,  "cloud", 12288, 1, 0, 100);
}

void loop() {
  vTaskDelete(NULL);   // all work lives in FreeRTOS tasks
}
