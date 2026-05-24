/*
 * MediTrack: Medication Habit Monitor
 * MYOSA / ESP32 prototype
 *
 * Uses:
 * - MPU6050 for bottle motion detection
 * - SSD1306 OLED for status display
 * - Buzzer for alerts
 * - ESP32 Wi-Fi for local dashboard
 *
 * Demo timing:
 * - Dose interval: 60 seconds
 * - Grace period: 15 seconds
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------------------------------------------------------------------------
// Wi-Fi settings
// ---------------------------------------------------------------------------
const char *WIFI_SSID = "YOUR_WIFI_NAME";
const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";

// Local dashboard hotspot
const char *AP_SSID = "MediTrack";
const char *AP_PASS = "meditrack1";

// ---------------------------------------------------------------------------
// Pin map
// ---------------------------------------------------------------------------
#define I2C_SDA       21
#define I2C_SCL       22
#define BUZZER_PIN    18
#define BOOT_BTN_PIN  0       
#define OLED_WIDTH    128
#define OLED_HEIGHT   64
#define OLED_ADDR     0x3C
// MPU6050 I2C address: 0x68 (AD0 low) or 0x69 (AD0 high / 3.3V)
#define MPU_ADDR_LOW  0x68
#define MPU_ADDR_HIGH 0x69

#define TRY_ALT_I2C_PINS  1

const unsigned long DOSE_INTERVAL_SEC   = 60;
const unsigned long GRACE_PERIOD_SEC    = 15;
// After grace ends, user can still take a "delayed" dose before auto-miss (demo)
const unsigned long OVERDUE_TIMEOUT_SEC = 20;

// ---------------------------------------------------------------------------
// Motion / tilt thresholds (tune on Serial Monitor if needed)
// ---------------------------------------------------------------------------

const float GYRO_MOTION_DPS     = 34.0f;  // rotation while handling bottle 
const float TILT_CHANGE_DEG     = 8.5f;   // tilt must change this much from rest
const float ACCEL_CHANGE_G      = 0.15f;  // jerk away from rest orientation
const unsigned long MOTION_COOLDOWN_MS = 2500;
const unsigned long DOSE_SETTLE_MS = 1000;  // ignore 1s after "Dose Due" (no false trigger)

// ---------------------------------------------------------------------------
// Global objects
// ---------------------------------------------------------------------------
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Adafruit_MPU6050 mpu;
WebServer server(80);

bool oledOk = false;
bool mpuOk  = false;
bool mpuUseRawDriver = false;  // true = direct I2C (when Adafruit lib fails)
bool wifiApMode = false;       // true = phone must join "MediTrack" Wi-Fi
uint8_t mpuI2cAddr = 0;
uint8_t i2cFoundAddrs[8];
uint8_t i2cFoundCount = 0;
unsigned long lastBootBtnMs = 0;

// ---------------------------------------------------------------------------
// Dose tracking & status
// ---------------------------------------------------------------------------
unsigned long totalDoses    = 0;
unsigned long takenDoses    = 0;
unsigned long missedDoses   = 0;
unsigned long delayedDoses = 0;
float adherenceScore        = 100.0f;

String currentStatus        = "Initializing";
String lastEventTime        = "--";
unsigned long nextDoseCountdown = DOSE_INTERVAL_SEC;

// Schedule timestamps (millis)
unsigned long nextDoseDueMs     = 0;
unsigned long graceEndsMs       = 0;
unsigned long missDeadlineMs    = 0;
bool doseWindowOpen             = false;
bool intakeRecordedThisCycle    = false;
bool missedAlreadyLogged        = false;

unsigned long lastMotionTriggerMs = 0;
unsigned long lastDisplayMs       = 0;
unsigned long lastSerialDebugMs   = 0;
unsigned long statusHoldUntilMs   = 0;  

// Rest pose snapshot when each dose window opens (avoids false "taken" while still)
float restTilt = 0, restAx = 0, restAy = 0, restAz = 0, restGyro = 0;
bool doseBaselineReady = false;
unsigned long doseBaselineMs = 0;

// Buzzer state machine (non-blocking)
int beepPatternRemaining = 0;
int beepPatternStep      = 0;
unsigned long beepPhaseMs = 0;
bool buzzerOn            = false;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void initI2CBus(int sda, int scl);
void scanI2CBus();
bool initMPU6050();
bool initOLED();
bool tryMPUOnCurrentBus();
void connectWiFi();
void startWifiHotspot();
void printNetworkHelp();
void printWifiFailureHelp();
void setupWebServer();
String getDashboardUrl();
uint8_t readI2CReg(uint8_t devAddr, uint8_t reg);
bool mpuWake(uint8_t devAddr);
bool mpuWriteReg(uint8_t devAddr, uint8_t reg, uint8_t val);
bool whoAmIValid(uint8_t id);
bool scanHasAddr(uint8_t addr);
bool mpuReadRawSensors(uint8_t addr, int16_t &ax, int16_t &ay, int16_t &az,
                       int16_t &gx, int16_t &gy, int16_t &gz);
bool initMPURaw(uint8_t addr);
void checkBootButtonDemo();
String buildDashboardPage();
void updateSchedule();
void checkMissedDose();
void processMotion(bool motion, float tiltDeg, float gyroMag, float ax, float ay, float az);
void recordDoseTaken();
void recordDoseDelayed();
void recordDoseMissed();
void recalcAdherence();
void setStatus(const String &status, const String &eventLabel);
void startBeepPattern(int beeps);
void updateBuzzer();
bool readMotionAndTilt(float &tiltDeg, float &gyroMag, float &ax, float &ay, float &az, bool &motion);
void captureDoseBaseline();
void drawOled();
void drawOledError(const char *line1, const char *line2);
void formatCountdown(char *buf, size_t len);
void formatEventTime(String &out);

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("=== MediTrack v2 (raw MPU @ 0x69) ==="));

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

  // ----- I2C sensors (OLED 0x3C + MPU6050 0x68/0x69 on same bus) -----
  Serial.println(F("Initializing I2C sensors..."));
  initI2CBus(I2C_SDA, I2C_SCL);
  scanI2CBus();
  initOLED();
  initMPU6050();

#if TRY_ALT_I2C_PINS
  if (!mpuOk) {
    Serial.println(F("[I2C] MPU missing — trying SDA=25 SCL=26..."));
    initI2CBus(25, 26);
    scanI2CBus();
    tryMPUOnCurrentBus();
  }
  if (!mpuOk && oledOk) {
    initI2CBus(I2C_SDA, I2C_SCL);
  }
#endif

  if (!mpuOk) {
    Serial.println(F("[ERROR] MPU6050 not found."));
    if (scanHasAddr(0x69)) {
      Serial.println(F("  Device at 0x69 seen but not MPU — check module wiring."));
    }
    Serial.println(F("  DEMO BACKUP: press BOOT button when dose is DUE."));
    if (oledOk) {
      drawOledError("MPU6050 FAIL", "Use BOOT btn");
    }
  } else if (mpuUseRawDriver) {
    Serial.println(F("[INFO] Using RAW I2C driver (0x69). Tilt bottle to test."));
  }

  // ----- First dose window (demo: first due after 60 s) -----
  totalDoses = 0;
  nextDoseDueMs = millis() + (DOSE_INTERVAL_SEC * 1000UL);
  graceEndsMs   = nextDoseDueMs + (GRACE_PERIOD_SEC * 1000UL);
  missDeadlineMs = graceEndsMs + (OVERDUE_TIMEOUT_SEC * 1000UL);
  doseWindowOpen = false;
  intakeRecordedThisCycle = false;
  missedAlreadyLogged = false;
  currentStatus = "Waiting for Dose";
  recalcAdherence();

  connectWiFi();
  setupWebServer();
  printNetworkHelp();

  Serial.println(F("Demo: dose every 60s, grace 15s after due."));
  if (mpuOk) {
    Serial.println(F("Tip: tilt or shake the bottle when status is Dose Due."));
  } else {
    Serial.println(F("Tip: press BOOT button on ESP32 when dose is DUE."));
  }
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  server.handleClient();
  updateBuzzer();

  unsigned long now = millis();

  // Open dose window when countdown reaches zero
  if (!doseWindowOpen && now >= nextDoseDueMs) {
    doseWindowOpen = true;
    intakeRecordedThisCycle = false;
    missedAlreadyLogged = false;
    missDeadlineMs = graceEndsMs + (OVERDUE_TIMEOUT_SEC * 1000UL);
    setStatus("Dose Due", "Dose window opened");
    captureDoseBaseline();
    Serial.println(F("[SCHEDULE] Dose is now DUE — move bottle to record intake."));
  }

  // Show overdue after grace (still time for "Dose Delayed" motion)
  if (doseWindowOpen && !intakeRecordedThisCycle && now >= graceEndsMs &&
      now < missDeadlineMs && currentStatus == "Dose Due") {
    currentStatus = "Grace Ended";
  }

  checkMissedDose();

  // Motion detection
  float tiltDeg = 0, gyroMag = 0, ax = 0, ay = 0, az = 0;
  bool motion = false;
  if (mpuOk) {
    readMotionAndTilt(tiltDeg, gyroMag, ax, ay, az, motion);
    if (motion) {
      processMotion(motion, tiltDeg, gyroMag, ax, ay, az);
    }
  } else {
    checkBootButtonDemo();
  }

  updateSchedule();

  // Return to "Waiting for Dose" after showing result status 
  if (statusHoldUntilMs > 0 && millis() > statusHoldUntilMs &&
      !doseWindowOpen && currentStatus != "Dose Due") {
    currentStatus = "Waiting for Dose";
    statusHoldUntilMs = 0;
  }

  // OLED refresh ~4 Hz
  if (now - lastDisplayMs >= 250) {
    lastDisplayMs = now;
    drawOled();
  }

  // Serial debug ~2 Hz
  if (now - lastSerialDebugMs >= 500) {
    lastSerialDebugMs = now;
    char cd[16];
    formatCountdown(cd, sizeof(cd));
    Serial.print(F("[DEBUG] status="));
    Serial.print(currentStatus);
    Serial.print(F(" countdown="));
    Serial.print(cd);
    Serial.print(F(" adherence="));
    Serial.print(adherenceScore, 1);
    Serial.print(F("% mpu="));
    Serial.print(mpuOk ? 1 : 0);
    Serial.print(F(" tilt="));
    Serial.print(tiltDeg, 1);
    Serial.print(F(" gyro="));
    Serial.print(gyroMag, 1);
    Serial.print(F(" ax="));
    Serial.print(ax, 3);
    Serial.print(F(" ay="));
    Serial.print(ay, 3);
    Serial.print(F(" az="));
    Serial.print(az, 3);
    if (!mpuOk && doseWindowOpen) {
      Serial.print(F(" >>> PRESS BOOT BTN <<<"));
    }
    Serial.println();
  }
}

// =============================================================================
// I2C helpers — scan bus - i had to do this as my i2c was not able to scan both devices at once, this was just a testing code
// =============================================================================
void initI2CBus(int sda, int scl) {
  Wire.begin(sda, scl);
  Wire.setClock(100000);  // 100 kHz — more reliable on breadboards
  delay(50);
  Serial.print(F("[I2C] SDA=GPIO "));
  Serial.print(sda);
  Serial.print(F(" SCL=GPIO "));
  Serial.println(scl);
}

void scanI2CBus() {
  Serial.println(F("--- I2C scan ---"));
  i2cFoundCount = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  Found 0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      if (i2cFoundCount < 8) {
        i2cFoundAddrs[i2cFoundCount++] = addr;
      }
    }
  }
  if (i2cFoundCount == 0) {
    Serial.println(F("  No devices! Check 3.3V, GND, SDA, SCL."));
  }
  Serial.println(F("--- end scan ---"));
}

bool scanHasAddr(uint8_t addr) {
  for (uint8_t i = 0; i < i2cFoundCount; i++) {
    if (i2cFoundAddrs[i] == addr) {
      return true;
    }
  }
  return false;
}

bool initOLED() {
  const uint8_t oledAddrs[] = {0x3C, 0x3D};
  for (uint8_t i = 0; i < 2; i++) {
    if (display.begin(SSD1306_SWITCHCAPVCC, oledAddrs[i])) {
      oledOk = true;
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println(F("MediTrack"));
      display.println(F("OLED OK"));
      display.display();
      Serial.print(F("[OK] SSD1306 OLED at 0x"));
      Serial.println(oledAddrs[i], HEX);
      return true;
    }
  }
  oledOk = false;
  Serial.println(F("[ERROR] SSD1306 not at 0x3C or 0x3D."));
  return false;
}

uint8_t readI2CReg(uint8_t devAddr, uint8_t reg) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return 0xFF;
  }
  if (Wire.requestFrom((uint16_t)devAddr, (size_t)1) != 1) {
    return 0xFF;
  }
  return Wire.read();
}

bool mpuWake(uint8_t devAddr) {
  return mpuWriteReg(devAddr, 0x6B, 0x00);
}

bool mpuWriteReg(uint8_t devAddr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool whoAmIValid(uint8_t id) {
  return (id == 0x68 || id == 0x70 || id == 0x71 || id == 0x72 || id == 0x73);
}

bool mpuReadRawSensors(uint8_t addr, int16_t &ax, int16_t &ay, int16_t &az,
                       int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(addr);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t buf[14];
  uint8_t n = Wire.requestFrom(addr, (size_t)14, (bool)true);
  uint8_t i = 0;
  unsigned long t0 = millis();
  while (i < 14 && (i < n || Wire.available() > 0) && millis() - t0 < 50) {
    if (Wire.available()) {
      buf[i++] = Wire.read();
    }
  }
  if (i < 14) {
    return false;
  }

  ax = ((int16_t)buf[0] << 8) | buf[1];
  ay = ((int16_t)buf[2] << 8) | buf[3];
  az = ((int16_t)buf[4] << 8) | buf[5];
  gx = ((int16_t)buf[8] << 8) | buf[9];
  gy = ((int16_t)buf[10] << 8) | buf[11];
  gz = ((int16_t)buf[12] << 8) | buf[13];
  return true;
}

bool initMPURaw(uint8_t addr) {
  if (!scanHasAddr(addr)) {
    return false;
  }
  if (!mpuWake(addr)) {
    Serial.print(F("[RAW] No wake ACK at 0x"));
    Serial.println(addr, HEX);
    return false;
  }
  delay(80);
  mpuWriteReg(addr, 0x1C, 0x00);  // accel ±2g
  mpuWriteReg(addr, 0x1B, 0x00);  // gyro ±250 dps
  delay(20);

  int16_t ax, ay, az, gx, gy, gz;
  bool readOk = mpuReadRawSensors(addr, ax, ay, az, gx, gy, gz);
  if (readOk) {
    Serial.print(F("[RAW] Sample ax="));
    Serial.print(ax);
    Serial.print(F(" ay="));
    Serial.print(ay);
    Serial.print(F(" az="));
    Serial.print(az);
    Serial.print(F(" gx="));
    Serial.println(gx);
  } else {
    Serial.print(F("[RAW] Read failed at 0x"));
    Serial.print(addr, HEX);
    Serial.println(F(" — enabling anyway (retry in loop)"));
  }

  mpuI2cAddr = addr;
  mpuUseRawDriver = true;
  mpuOk = true;
  Serial.print(F("[OK] MPU RAW enabled at 0x"));
  Serial.println(addr, HEX);
  return true;
}

bool initMPU6050() {
  mpuUseRawDriver = false;
  const uint8_t addrs[] = {MPU_ADDR_HIGH, MPU_ADDR_LOW};

  for (uint8_t i = 0; i < 2; i++) {
    uint8_t addr = addrs[i];
    if (!scanHasAddr(addr)) {
      Serial.print(F("[SKIP] 0x"));
      Serial.print(addr, HEX);
      Serial.println(F(" not on I2C bus"));
      continue;
    }

    uint8_t who = 0xFF;
    if (mpuWake(addr)) {
      delay(30);
      who = readI2CReg(addr, 0x75);
    }
    Serial.print(F("[TRY] 0x"));
    Serial.print(addr, HEX);
    Serial.print(F(" WHO_AM_I=0x"));
    Serial.println(who, HEX);

    if (initMPURaw(addr)) {
      return true;
    }

    if (whoAmIValid(who) && mpu.begin(addr)) {
      mpuOk = true;
      mpuUseRawDriver = false;
      mpuI2cAddr = addr;
      mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      Serial.print(F("[OK] MPU6050 Adafruit lib at 0x"));
      Serial.println(addr, HEX);
      return true;
    }
  }

  if (scanHasAddr(MPU_ADDR_HIGH)) {
    Serial.println(F("[FALLBACK] Forcing MPU driver on 0x69 from I2C scan."));
    mpuWake(MPU_ADDR_HIGH);
    mpuI2cAddr = MPU_ADDR_HIGH;
    mpuUseRawDriver = true;
    mpuOk = true;
    return true;
  }

  mpuOk = false;
  mpuI2cAddr = 0;
  return false;
}

void checkBootButtonDemo() {
  if (!doseWindowOpen || intakeRecordedThisCycle) {
    return;
  }
  // BOOT = GPIO 0, active LOW when pressed
  if (digitalRead(BOOT_BTN_PIN) != LOW) {
    return;
  }
  Serial.println(F("[BOOT] Button pressed!"));
  unsigned long now = millis();
  if (now - lastBootBtnMs < 800) {
    return;
  }
  lastBootBtnMs = now;

  if (now < graceEndsMs) {
    recordDoseTaken();
    Serial.println(F("[DEMO] BOOT button — dose taken (MPU backup)."));
  } else if (now < missDeadlineMs) {
    recordDoseDelayed();
    Serial.println(F("[DEMO] BOOT button — dose delayed (MPU backup)."));
  }
}

bool tryMPUOnCurrentBus() {
  return initMPU6050();
}

// =============================================================================
// Wi-Fi & web dashboard
// =============================================================================
void printWifiFailureHelp() {
  Serial.print(F("[WiFi] Status code: "));
  Serial.println((int)WiFi.status());
  switch (WiFi.status()) {
    case WL_NO_SSID_AVAIL:
      Serial.println(F("  -> Network name not found. Use 2.4 GHz SSID (not 5 GHz)."));
      break;
    case WL_CONNECT_FAILED:
      Serial.println(F("  -> Wrong password, or router security not supported."));
      break;
    case WL_DISCONNECTED:
      Serial.println(F("  -> Still disconnected. Check signal and 2.4 GHz band."));
      break;
    default:
      Serial.println(F("  -> See ESP32 WiFi status codes in Serial output."));
      break;
  }
  String ssid = WIFI_SSID;
  if (ssid.indexOf("5G") >= 0 || ssid.indexOf("5g") >= 0) {
    Serial.println(F("  !! SSID contains '5G' — ESP32 cannot use 5 GHz Wi-Fi."));
  }
}

void startWifiHotspot() {
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                    IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);  // channel 1, visible, max 4 clients
  delay(500);
  wifiApMode = true;
  Serial.println(F("[OK] MediTrack hotspot ON (use this for the website)."));
  Serial.print(F("     Wi-Fi name: "));
  Serial.println(AP_SSID);
  Serial.print(F("     Password:   "));
  Serial.println(AP_PASS);
  Serial.print(F("     Dashboard:  http://"));
  Serial.println(WiFi.softAPIP());
}

void connectWiFi() {
  currentStatus = "Connecting WiFi";
  drawOled();

  WiFi.mode(WIFI_AP_STA);  // hotspot + home Wi-Fi at the same time
  WiFi.setSleep(false);
  startWifiHotspot();

  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print(F("Also joining home Wi-Fi: "));
  Serial.println(WIFI_SSID);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("[OK] Home Wi-Fi IP (optional): "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("[WARN] Home Wi-Fi failed — hotspot still works."));
    printWifiFailureHelp();
  }
  currentStatus = "Waiting for Dose";
}

String getDashboardUrl() {
  return String(F("http://192.168.4.1"));
}

void printNetworkHelp() {
  Serial.println();
  Serial.println(F("========== MediTrack Dashboard =========="));
  Serial.println(F("Connect to Wi-Fi: MediTrack"));
  Serial.println(F("Password: meditrack1"));
  Serial.println(F("Open browser at:"));
  Serial.println(F("http://192.168.4.1"));
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("Optional (same home Wi-Fi only): http://"));
    Serial.println(WiFi.localIP());
  }
  Serial.println(F("=============================================="));
  Serial.println();
}

void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", buildDashboardPage());
  });
  server.on("/health", HTTP_GET, []() {
    server.send(200, "text/plain", "MediTrack OK - dashboard is running");
  });
  server.onNotFound([]() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "Redirecting to MediTrack dashboard...");
  });
  server.begin();
  Serial.println(F("[OK] Web server on port 80 (AP: http://192.168.4.1)"));
}

String buildDashboardPage() {
  char countdown[24];
  formatCountdown(countdown, sizeof(countdown));

  String html;
  html.reserve(2800);
  html += F("<!DOCTYPE html><html lang='en'><head>");
  html += F("<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<meta http-equiv='refresh' content='2'>");
  html += F("<title>MediTrack Dashboard</title>");
  html += F("<style>");
  html += F("*{box-sizing:border-box;margin:0;padding:0}");
  html += F("body{font-family:Segoe UI,system-ui,sans-serif;background:linear-gradient(135deg,#0f172a,#1e3a5f);");
  html += F("color:#e2e8f0;min-height:100vh;padding:24px}");
  html += F(".wrap{max-width:720px;margin:0 auto}");
  html += F("h1{font-size:1.75rem;margin-bottom:4px;color:#38bdf8}");
  html += F(".sub{color:#94a3b8;font-size:.9rem;margin-bottom:20px}");
  html += F(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:14px}");
  html += F(".card{background:rgba(255,255,255,.08);border:1px solid rgba(255,255,255,.12);");
  html += F("border-radius:12px;padding:16px;backdrop-filter:blur(8px)}");
  html += F(".label{font-size:.75rem;text-transform:uppercase;letter-spacing:.06em;color:#94a3b8}");
  html += F(".value{font-size:1.35rem;font-weight:600;margin-top:6px}");
  html += F(".status{font-size:1.5rem;color:#4ade80}");
  html += F(".score{color:#fbbf24}");
  html += F("footer{margin-top:24px;font-size:.8rem;color:#64748b}");
  html += F("</style></head><body><div class='wrap'>");
  html += F("<h1>MediTrack</h1><p class='sub'>Medication Habit Monitor — live demo</p>");
  html += F("<div class='grid'>");

  auto card = [&](const char *label, const String &val, const char *cls) {
    html += F("<div class='card'><div class='label'>");
    html += label;
    html += F("</div><div class='value ");
    html += cls;
    html += F("'>");
    html += val;
    html += F("</div></div>");
  };

  card("Current Status", currentStatus, "status");
  card("Next Dose Countdown", String(countdown), "");
  card("Total Doses", String(totalDoses), "");
  card("Taken Doses", String(takenDoses), "");
  card("Missed Doses", String(missedDoses), "");
  card("Delayed Doses", String(delayedDoses), "");
  String scoreStr = String(adherenceScore, 1) + "%";
  card("Adherence Score", scoreStr, "score");
  card("Last Event Time", lastEventTime, "");

  html += F("</div><footer>Auto-refresh 2s &bull; ");
  html += getDashboardUrl();
  html += F(" &bull; ");
  html += mpuOk ? F("MPU OK") : F("MPU missing — use BOOT btn");
  html += F("</footer>");
  html += F("</div></body></html>");
  return html;
}

// =============================================================================
// Schedule & dose logic
// =============================================================================
void updateSchedule() {
  unsigned long now = millis();
  if (!doseWindowOpen && now < nextDoseDueMs) {
    nextDoseCountdown = (nextDoseDueMs - now) / 1000UL;
  } else {
    nextDoseCountdown = 0;  // dose due or grace/overdue window
  }
}

void checkMissedDose() {
  unsigned long now = millis();
  // Miss only if no motion through grace + overdue window
  if (doseWindowOpen && !intakeRecordedThisCycle && !missedAlreadyLogged &&
      now >= missDeadlineMs) {
    recordDoseMissed();
  }
}

void processMotion(bool motion, float tiltDeg, float gyroMag,
                   float ax, float ay, float az) {
  unsigned long now = millis();
  if (now - lastMotionTriggerMs < MOTION_COOLDOWN_MS) {
    return;
  }
  lastMotionTriggerMs = now;

  if (!doseWindowOpen && now < nextDoseDueMs) {
    Serial.println(F("[MOTION] Ignored — next dose not due yet."));
    return;
  }

  if (!doseWindowOpen) {
    return;
  }

  if (intakeRecordedThisCycle) {
    return;
  }

  Serial.print(F("[MOTION] Intake! tiltΔ="));
  Serial.print(fabsf(tiltDeg - restTilt), 1);
  Serial.print(F(" gyro="));
  Serial.println(gyroMag, 1);

  if (now < graceEndsMs) {
    recordDoseTaken();
  } else if (now < missDeadlineMs) {
    recordDoseDelayed();
  }
}

void recordDoseTaken() {
  intakeRecordedThisCycle = true;
  takenDoses++;
  totalDoses = takenDoses + missedDoses + delayedDoses;
  setStatus("Dose Taken", "Dose taken on time");
  statusHoldUntilMs = millis() + 8000;
  startBeepPattern(1);
  scheduleNextDose();
  recalcAdherence();
  Serial.println(F("[EVENT] Dose TAKEN (within grace)."));
}

void recordDoseDelayed() {
  intakeRecordedThisCycle = true;
  delayedDoses++;
  totalDoses = takenDoses + missedDoses + delayedDoses;
  setStatus("Dose Delayed", "Dose taken after grace");
  statusHoldUntilMs = millis() + 8000;
  startBeepPattern(2);
  scheduleNextDose();
  recalcAdherence();
  Serial.println(F("[EVENT] Dose DELAYED (after grace period)."));
}

void recordDoseMissed() {
  missedAlreadyLogged = true;
  intakeRecordedThisCycle = true;  // close window
  missedDoses++;
  totalDoses = takenDoses + missedDoses + delayedDoses;
  setStatus("Missed Dose", "No motion before grace ended");
  statusHoldUntilMs = millis() + 8000;
  startBeepPattern(3);
  scheduleNextDose();
  recalcAdherence();
  Serial.println(F("[EVENT] Dose MISSED (no bottle motion)."));
}

void scheduleNextDose() {
  doseWindowOpen = false;
  doseBaselineReady = false;
  unsigned long now = millis();
  nextDoseDueMs = now + (DOSE_INTERVAL_SEC * 1000UL);
  graceEndsMs   = nextDoseDueMs + (GRACE_PERIOD_SEC * 1000UL);
  missDeadlineMs = graceEndsMs + (OVERDUE_TIMEOUT_SEC * 1000UL);
  // currentStatus stays on Taken/Delayed/Missed until statusHoldUntilMs expires
  Serial.print(F("[SCHEDULE] Next dose in "));
  Serial.print(DOSE_INTERVAL_SEC);
  Serial.println(F(" seconds."));
}

void recalcAdherence() {
  unsigned long accounted = takenDoses + missedDoses + delayedDoses;
  if (accounted == 0) {
    adherenceScore = 100.0f;
    return;
  }
  // On-time taken = full credit; delayed = half; missed = zero
  float points = (float)takenDoses + 0.5f * (float)delayedDoses;
  adherenceScore = (points / (float)accounted) * 100.0f;
  if (adherenceScore > 100.0f) adherenceScore = 100.0f;
}

void setStatus(const String &status, const String &eventLabel) {
  if (currentStatus != status) {
    Serial.print(F("[STATUS] "));
    Serial.print(currentStatus);
    Serial.print(F(" -> "));
    Serial.println(status);
  }
  currentStatus = status;
  formatEventTime(lastEventTime);
  (void)eventLabel;
}

// =============================================================================
// MPU6050 motion + tilt — intake = movement vs rest pose, not absolute angle
// =============================================================================
bool readMotionAndTilt(float &tiltDeg, float &gyroMag,
                       float &ax, float &ay, float &az, bool &motion) {
  float gx = 0, gy = 0, gz = 0;
  motion = false;

  if (mpuUseRawDriver) {
    int16_t rax, ray, raz, rgx, rgy, rgz;
    if (!mpuReadRawSensors(mpuI2cAddr, rax, ray, raz, rgx, rgy, rgz)) {
      return false;
    }
    ax = rax / 16384.0f;
    ay = ray / 16384.0f;
    az = raz / 16384.0f;
    gx = rgx / 131.0f;
    gy = rgy / 131.0f;
    gz = rgz / 131.0f;
  } else {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    ax = a.acceleration.x / 9.81f;
    ay = a.acceleration.y / 9.81f;
    az = a.acceleration.z / 9.81f;
    gx = g.gyro.x * 57.2958f;
    gy = g.gyro.y * 57.2958f;
    gz = g.gyro.z * 57.2958f;
  }

  gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);

  float horiz = sqrtf(ax * ax + ay * ay);
  tiltDeg = atan2f(horiz, fabsf(az)) * 57.2958f;

  // Only compare to rest pose during an open dose window (after settle time)
  if (!doseWindowOpen || !doseBaselineReady || intakeRecordedThisCycle) {
    return true;
  }
  if (millis() - doseBaselineMs < DOSE_SETTLE_MS) {
    return true;
  }

  float tiltChange = fabsf(tiltDeg - restTilt);
  float dax = ax - restAx;
  float day = ay - restAy;
  float daz = az - restAz;
  float accelChange = sqrtf(dax * dax + day * day + daz * daz);

  bool gyroHit  = gyroMag >= GYRO_MOTION_DPS;
  bool tiltHit  = tiltChange >= TILT_CHANGE_DEG;
  bool accelHit = accelChange >= ACCEL_CHANGE_G;

  motion = gyroHit || tiltHit || accelHit;
  return true;
}

void captureDoseBaseline() {
  doseBaselineReady = false;
  float sumTilt = 0, sumAx = 0, sumAy = 0, sumAz = 0, sumGyro = 0;
  uint8_t n = 0;
  for (uint8_t i = 0; i < 8; i++) {
    float t, g, ax, ay, az;
    bool m = false;
    if (readMotionAndTilt(t, g, ax, ay, az, m)) {
      sumTilt += t;
      sumAx += ax;
      sumAy += ay;
      sumAz += az;
      sumGyro += g;
      n++;
    }
    delay(40);
  }
  if (n > 0) {
    restTilt = sumTilt / n;
    restAx = sumAx / n;
    restAy = sumAy / n;
    restAz = sumAz / n;
    restGyro = sumGyro / n;
    doseBaselineReady = true;
    doseBaselineMs = millis();
    Serial.print(F("[BASELINE] Rest tilt="));
    Serial.print(restTilt, 1);
    Serial.print(F(" gyro="));
    Serial.println(restGyro, 1);
    Serial.println(F("[BASELINE] Move bottle NOW to register dose."));
  }
}

// =============================================================================
// Buzzer (non-blocking patterns)
// =============================================================================
void startBeepPattern(int beeps) {
  beepPatternRemaining = beeps;
  beepPatternStep = 0;
  beepPhaseMs = millis();
  buzzerOn = false;
  digitalWrite(BUZZER_PIN, LOW);
}

void updateBuzzer() {
  if (beepPatternRemaining <= 0) return;

  unsigned long now = millis();
  const unsigned long BEEP_ON  = 120;
  const unsigned long BEEP_OFF = 180;

  if (beepPatternStep == 0) {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerOn = true;
    beepPhaseMs = now;
    beepPatternStep = 1;
  } else if (beepPatternStep == 1 && now - beepPhaseMs >= BEEP_ON) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;
    beepPhaseMs = now;
    beepPatternStep = 2;
  } else if (beepPatternStep == 2 && now - beepPhaseMs >= BEEP_OFF) {
    beepPatternRemaining--;
    beepPatternStep = 0;
    if (beepPatternRemaining <= 0) {
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}

// =============================================================================
// OLED display
// =============================================================================
void drawOledError(const char *line1, const char *line2) {
  if (!oledOk) return;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("MediTrack"));
  display.println(line1);
  display.println(line2);
  display.display();
}

void drawOled() {
  if (!oledOk) return;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println(F("MediTrack"));

  if (!mpuOk) {
    display.println(F("MPU: NOT FOUND"));
  }

  display.print(F("Status: "));
  String s = currentStatus;
  if (s.length() > 14) s = s.substring(0, 14);
  display.println(s);

  char cd[16];
  formatCountdown(cd, sizeof(cd));
  display.print(F("Next dose: "));
  display.println(cd);

  display.print(F("Adherence: "));
  display.print(adherenceScore, 1);
  display.println(F("%"));

  display.println(F("Site: MediTrack"));
  display.println(F("192.168.4.1"));
  display.println(F("Dashboard"));
  display.display();
}

void formatCountdown(char *buf, size_t len) {
  if (nextDoseCountdown > 0) {
    snprintf(buf, len, "%lus", nextDoseCountdown);
  } else if (doseWindowOpen) {
    snprintf(buf, len, "DUE NOW");
  } else {
    snprintf(buf, len, "0s");
  }
}

void formatEventTime(String &out) {
  unsigned long sec = millis() / 1000UL;
  unsigned long h = (sec / 3600UL) % 24UL;
  unsigned long m = (sec / 60UL) % 60UL;
  unsigned long s = sec % 60UL;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", h, m, s);
  out = String(buf);
}
