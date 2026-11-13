#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <DHT.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <stdarg.h>
#include <driver/gpio.h>

// ======================= Identity =======================
#define TEAM_ID     "T07"        // <-- your Team ID
#define DEVICE_ID   "T07-D1"     // <-- your device ID
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
};

static Shared g;
static SemaphoreHandle_t gMtx;
static SemaphoreHandle_t gLogMtx;
static volatile bool gHang = false;
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
  UNLOCK();
  // Level 3+ adds FIRE, PANIC, DANGER, WARNING ... here (lower number first)
  if (err) return L_SENSOR_ERR;
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
  UNLOCK();

  char tStr[12] = "-", rhStr[8] = "-";
  if (!dErr && !isnan(t)) snprintf(tStr, sizeof(tStr), "%.1fC", t);
  if (!dErr && !isnan(h)) snprintf(rhStr, sizeof(rhStr), "%d%%", (int)lroundf(h));

  logf("[STATUS] up=%lus T=%s RH=%s HI=- flame=%d door=%s state=%s trend=- rssi=- "
       "heap=%uk psram=%uk chip=%.1fC fw=%s",
       (unsigned long)uptimeS(), tStr, rhStr, fl ? 1 : 0,
       door ? "UNLOCKED" : "LOCKED", st,
       (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getFreePsram() / 1024),
       temperatureRead(), FW_VERSION);
}

// ======================= Commands =======================
void handleCommand(const String& cmd) {
  if (cmd == "help") {
    logf("[CMD] commands: help, status, health, tasks, ver, hang | reset (Level 2), csi (Level 11)"
         " | test only: reboot (SOFTWARE), crash (CRASH)");
  } else if (cmd == "status") {
    printStatusLine();
  } else if (cmd == "health") {
    logf("[CMD] health chip=%.1fC heap=%uk psram=%uk flash=%uMB cpu=%uMHz rssi=- up=%lus reset=%s fw=%s",
         temperatureRead(),
         (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getFreePsram() / 1024),
         (unsigned)(ESP.getFlashChipSize() / (1024 * 1024)), (unsigned)getCpuFrequencyMhz(),
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

// ======================= Tasks ==========================
void ledTask(void*) {
  esp_task_wdt_add(NULL);
  TickType_t last = xTaskGetTickCount();
  uint32_t n = 0, lastRgb = 0xFFFFFFFF;
  for (;;) {
    esp_task_wdt_reset();
    LightState s = currentLight();
    uint32_t ms = n * 50;
    bool on = (ms % 500) < 250;            // 2 Hz: 250 ms on / 250 ms off
    uint8_t r = 0, gg = 0, b = 0;
    if (on) colorFor(s, r, gg, b);
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
        LOCK(); bool was = g.dhtError; g.tempC = t; g.rh = h; g.dhtError = false; UNLOCK();
        if (was) logf("[EVENT] SENSOR_ERROR DHT11 CLEARED");
        nextDht = tick + 50;               // 50 x 100 ms = 5 s
      } else {
        dhtFails++;
        LOCK(); bool was = g.dhtError; if (dhtFails >= 2) g.dhtError = true; UNLOCK();
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

    // Level 3+: status machine (SAFE/WARNING/DANGER/FIRE), door rules, panic ...

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

  initWatchdog();

  // Core 1: real-time side
  startTask(ledTask,    "led",    3072, 4, 1, 50);
  startTask(sensorTask, "sensor", 4096, 3, 1, 100);
  startTask(mainTask,   "main",   4096, 2, 1, 50);
  // Core 0: comms side
  startTask(rfidTask,   "rfid",   4096, 2, 0, 200);
  startTask(statusTask, "status", 4096, 1, 0, 1000);
  startTask(serialTask, "serial", 4096, 1, 0, 20);
}

void loop() {
  vTaskDelete(NULL);   // all work lives in FreeRTOS tasks
}
