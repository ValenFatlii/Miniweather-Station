// PENGATURAN PEMULIHAN MODEM: ubah hanya nilai di bagian ini.
// Waktu dalam milidetik; jumlah siklus harus 1..32.
#define MODEM_FAILURE_MS       240000UL  // 4 menit gangguan terkonfirmasi
#define MODEM_OFF_MS            30000UL  // 30 detik daya modem OFF
#define MODEM_BOOT_MS          120000UL  // 120 detik waktu boot
#define MODEM_MIN_CYCLE_GAP_MS 600000UL  // 10 menit jarak minimal siklus
#define MODEM_CYCLE_WINDOW_MS 3600000UL  // jendela bergulir 1 jam
#define MODEM_MAX_CYCLES              4  // maksimal 4 siklus dalam jendela

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <MQTTPubSubClient.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include <math.h>
#include <esp_system.h>
#include <new>
#include <Wire.h>

#include <BH1750.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_INA219.h>
#include "SparkFun_SCD4x_Arduino_Library.h"

#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
// Kebijakan pemulihan disatukan di bawah; pengaturan tetap di baris 3-8.
#include <stdint.h>

// Pure timing policy: no WiFi, GPIO, SD, blocking delay, or wall-clock dependency.
enum class ModemHealth : uint8_t { Unknown, Good, Bad, OtherNetwork };
enum class ModemAction : uint8_t { None, PowerOff, PowerOn, ResumeNetwork };
enum class ModemPhase : uint8_t { BootWait, Monitoring, PowerOff };

struct ModemHealthInput {
  bool enabled = true, workerFault = false, linkUp = false, onModem = false;
  bool everConnected = false, lastWasModem = false;
  uint8_t offlineScans = 0;
  bool ackFresh = false, probeFresh = false, probeGood = false;
  uint8_t failedProbes = 0;
};

constexpr ModemHealth classifyModemHealth(const ModemHealthInput &i);

constexpr ModemHealth classifyModemHealth(const ModemHealthInput &i) {
  if (!i.enabled || i.workerFault || (i.linkUp && !i.onModem)) return ModemHealth::OtherNetwork;
  if (!i.linkUp)
    return (!i.everConnected || i.lastWasModem) && i.offlineScans >= 3
        ? ModemHealth::Bad : ModemHealth::Unknown;
  if (i.ackFresh) return ModemHealth::Good;
  if (!i.probeFresh) return ModemHealth::Unknown;
  if (i.probeGood) return ModemHealth::Good;
  return i.failedProbes >= 3 ? ModemHealth::Bad : ModemHealth::Unknown;
}

template <uint32_t FailureMs, uint32_t OffMs, uint32_t BootMs,
          uint32_t GapMs, uint32_t WindowMs, unsigned MaxCycles>
struct BasicModemRecoveryPolicy {
  static_assert(FailureMs > 0 && FailureMs < 0x80000000UL, "failure time must be 1..2147483647 ms");
  static_assert(OffMs > 0 && OffMs < 0x80000000UL, "OFF time must be 1..2147483647 ms");
  static_assert(BootMs > 0 && BootMs < 0x80000000UL, "boot time must be 1..2147483647 ms");
  static_assert(GapMs < 0x80000000UL, "cycle gap must be 0..2147483647 ms");
  static_assert(WindowMs > 0 && WindowMs < 0x80000000UL, "cycle window must be 1..2147483647 ms");
  static_assert(MaxCycles >= 1 && MaxCycles <= 32, "max cycles must be 1..32");
  static constexpr uint32_t failureMs = FailureMs;
  static constexpr uint32_t offMs = OffMs;
  static constexpr uint32_t bootMs = BootMs;
  static constexpr uint32_t minCycleGapMs = GapMs;
  static constexpr uint32_t hourMs = WindowMs;
  static constexpr uint8_t maxCycles = MaxCycles;
  ModemPhase phase = ModemPhase::BootWait;
  uint32_t stageSince = 0, failureSince = 0, lastCycle = 0, totalCycles = 0;
  uint32_t history[maxCycles] = {};
  uint8_t historyCount = 0;
  bool failureActive = false, hasCycle = false;

  constexpr void begin(uint32_t now, uint8_t retainedCount = 0, uint32_t retainedTotal = 0) {
    phase = ModemPhase::BootWait;
    stageSince = now;
    failureActive = false;
    totalCycles = retainedTotal;
    historyCount = retainedCount <= maxCycles ? retainedCount : maxCycles;
    // After a warm restart, uptime timestamps are unavailable: conservatively
    // count retained cycles as occurring now, for up to one more hour.
    for (uint8_t i = 0; i < historyCount; ++i) history[i] = now;
    hasCycle = historyCount > 0;
    lastCycle = now;
  }

  constexpr void prune(uint32_t now) {
    while (historyCount && (uint32_t)(now - history[0]) >= hourMs) {
      for (uint8_t i = 1; i < historyCount; ++i) history[i - 1] = history[i];
      --historyCount;
    }
  }

  constexpr bool busy() const { return phase != ModemPhase::Monitoring; }
  constexpr bool budgetBlocked() const { return historyCount >= maxCycles; }

  constexpr ModemAction tick(uint32_t now, ModemHealth health) {
    prune(now);
    // Always restore power after OFF, even if configuration/link state changes.
    if (phase == ModemPhase::PowerOff) {
      if ((uint32_t)(now - stageSince) < offMs) return ModemAction::None;
      phase = ModemPhase::BootWait;
      stageSince = now;
      return ModemAction::PowerOn;
    }
    if (phase == ModemPhase::BootWait) {
      if ((uint32_t)(now - stageSince) < bootMs) return ModemAction::None;
      phase = ModemPhase::Monitoring;
      failureActive = false;
      return ModemAction::ResumeNetwork;
    }
    if (health != ModemHealth::Bad) {
      failureActive = false;
      return ModemAction::None;
    }
    if (!failureActive) { failureSince = now; failureActive = true; }
    if ((uint32_t)(now - failureSince) < failureMs || budgetBlocked() ||
        (hasCycle && (uint32_t)(now - lastCycle) < minCycleGapMs))
      return ModemAction::None;
    history[historyCount++] = now;
    ++totalCycles;
    hasCycle = true;
    lastCycle = now;
    phase = ModemPhase::PowerOff;
    stageSince = now;
    failureActive = false;
    return ModemAction::PowerOff;
  }
};

using ModemRecoveryPolicy = BasicModemRecoveryPolicy<
    MODEM_FAILURE_MS, MODEM_OFF_MS, MODEM_BOOT_MS, MODEM_MIN_CYCLE_GAP_MS,
    MODEM_CYCLE_WINDOW_MS, MODEM_MAX_CYCLES>;


// WiFi (multi-jaringan)
// Isi daftar jaringan di bawah. Alat akan memilih jaringan/AP dengan sinyal
// (RSSI) terkuat di antara semua yang terdeteksi. Kosongkan "" pada password
// jika jaringan terbuka. Jaringan dengan SSID sama tapi AP/router berbeda
// (BSSID/IP berbeda) juga otomatis dipilih yang paling kuat.
struct WifiCred { const char* ssid; const char* pass; };
const WifiCred WIFI_LIST[] = {
  { "YOUR_MODEM_SSID", "YOUR_MODEM_PASSWORD" },
  { "YOUR_BACKUP_SSID", "YOUR_BACKUP_PASSWORD" },
};
const uint8_t WIFI_COUNT = sizeof(WIFI_LIST) / sizeof(WIFI_LIST[0]);

// MQTT WSS
const char* MQTT_HOST = "YOUR_MQTT_HOST";
const int   MQTT_PORT = 443;
const char* MQTT_PATH = "/YOUR_MQTT_WSS_PATH";
const char* MQTT_CLIENT_ID = "ESP32_WIND_RAIN";
const char* MQTT_TOPIC = "YOUR_MQTT_TOPIC";

// Sampel tiap detik; SD/MQTT berisi agregasi jendela 60 detik.
static const uint32_t SAMPLE_INTERVAL_MS = 1000UL;
static const uint32_t MEASUREMENT_INTERVAL_MS = 60000UL;

WebSocketsClient client;
using ReliableMqtt = MQTTPubSub::PubSubClient<1536>;
ReliableMqtt mqtt;
char runtimeClientId[64];
char bootId[40];
uint32_t messageSequence = 0;

// Sensors
BH1750 lightMeter;
Adafruit_BMP280 bmp;
Adafruit_INA219 ina219;
SCD4x scd40;

// Calibration report convention:
// sensor_reading = slope * reference_value + intercept
// reference_value = (sensor_reading - intercept) / slope
static inline float cal_CO2(float x)   { return (x + 134.3519f) / 1.0058f; }
static inline float cal_TEMP(float x)  { return (x + 0.0896f) / 0.9976f; }
static inline float cal_RH(float x)    { return (x + 0.0879f) / 1.0186f; }
static inline float cal_LUX(float x)   { return (x + 4.7057f) / 1.0021f; }
static inline float cal_PRES(float x)  { return (x + 178.3727f) / 1.1781f; }
static inline float cal_ALT(float x)   { return (x - 0.2240f) / 0.9641f; }
static inline float cal_VANE(float x)  { return (x + 2.0622f) / 0.9618f; }
static inline float cal_ANEMO(float x) { return (x - 0.1804f) / 0.8858f; }
static inline float cal_RAIN(float x)  { return (x <= 0.0f) ? 0.0f : (x + 0.2808f) / 1.1678f; }

static inline float clamp0(float v) { return (v < 0) ? 0 : v; }
static inline float clampRange(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

// Output precision is selected from sensor accuracy/calibration uncertainty,
// not merely from the sensor's digital resolution.
static const uint8_t DP_TEMP       = 1; // SCD40: +/-0.8 degC (15-35 degC)
static const uint8_t DP_RH         = 0; // SCD40: +/-6 %RH in specified conditions
static const uint8_t DP_PRESSURE   = 0; // BMP280: typical absolute accuracy +/-1 hPa
static const uint8_t DP_ALTITUDE   = 0; // calibration RMSE about 5.42 m; derived value
static const uint8_t DP_LUX        = 0; // BH1750 variation about +/-20%
static const uint8_t DP_WIND_SPEED = 1; // calibration RMSE about 0.32 m/s
static const uint8_t DP_RAIN       = 1; // calibration RMSE about 0.29 mm
static const uint8_t DP_WATER      = 0; // JSN-SR04T stated long-range accuracy about +/-1 cm
static const uint8_t DP_VOLTAGE    = 2; // INA219: actual uncertainty also depends on module calibration
static const uint8_t DP_CURRENT    = 1;
static const uint8_t DP_POWER      = 4; // energi mWh per jendela, bukan daya mW

static inline float roundToDp(float value, uint8_t decimals) {
  float factor = 1.0f;
  for (uint8_t i = 0; i < decimals; ++i) factor *= 10.0f;
  return roundf(value * factor) / factor;
}
static inline int wrapDeg(float deg) {
  while (deg < 0) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  int d = (int)lroundf(deg);
  if (d >= 360) d = 0;
  if (d < 0) d = 0;
  return d;
}

// Daya modem USB melalui modul MOSFET TS pada PCB revisi D.
// Pemulihan otomatis: GPIO32 aktif-HIGH sudah dikonfirmasi pengguna.
static const bool MODEM_AUTO_RECOVERY = true;
static const char* MODEM_WIFI_SSID = "YOUR_MODEM_SSID";
// Sesuaikan hanya jika SSID modem berubah; perbandingan huruf besar/kecil tepat.
// HIGH diasumsikan mengaktifkan input trigger modul N-MOSFET yang dipakai.
// Bila modul fisik aktif-LOW, ubah MODEM_ON_LEVEL menjadi LOW.
static const uint8_t MODEM_POWER_PIN = 32;
static const uint8_t MODEM_ON_LEVEL = HIGH;

bool modemRecoveryBusy();
void modemRecoveryBegin();
void serviceModemRecovery();
bool probeWorkerFault = false;
uint8_t offlineScanPasses = 0;
uint32_t modemRecoveryCount = 0;

static void modemPowerInit() {
  pinMode(MODEM_POWER_PIN, OUTPUT);
  digitalWrite(MODEM_POWER_PIN, MODEM_ON_LEVEL);
}

// LED
#define LED_R 25
#define LED_G 26
#define LED_B 27
const bool LED_COMMON_ANODE = false;

static inline void ledWriteRaw(bool r, bool g, bool b) {
  if (LED_COMMON_ANODE) { r = !r; g = !g; b = !b; }
  digitalWrite(LED_R, r ? HIGH : LOW);
  digitalWrite(LED_G, g ? HIGH : LOW);
  digitalWrite(LED_B, b ? HIGH : LOW);
}
static inline void ledOff()       { ledWriteRaw(false,false,false); }
static inline void ledRed()       { ledWriteRaw(true,false,false); }
static inline void ledGreen()     { ledWriteRaw(false,true,false); }
static inline void ledBlue()      { ledWriteRaw(false,false,true); }
static inline void ledBlueBlink(uint32_t nowMs, uint16_t periodMs=400) {
  bool on = ((nowMs / periodMs) % 2) == 0;
  ledWriteRaw(false,false,on);
}

// Status pengiriman tidak direset hanya karena CONNECT berhasil.
uint32_t failPublishCount = 0;
bool publishFault = false, storageFault = false, csvFault = false, ramOverflowFault = false;
bool sessionHasAck = false;
uint32_t lastPublishAckMs = 0;
uint32_t lastLiveAckMs = 0;
bool liveSessionHasAck = false, liveConfirmedLatest = false;
bool livePublishFault = false, backlogPublishFault = false, deliveryLogFault = false;
bool ledWorkerFault = false;
// Hanya data pada siklus pengukuran saat ini yang boleh ditandai live.
static const uint32_t LIVE_MAX_AGE_MS = MEASUREMENT_INTERVAL_MS;
static const uint32_t ACK_FRESH_MS = 90000UL;
static const uint32_t BROKER_FAILOVER_MS = 120000UL;
static const uint32_t TRANSPORT_RESTART_MS = 30000UL;
static const uint32_t MQTT_TIMEOUT_MS = 1500UL;
uint32_t brokerUnhealthySinceMs = 0, lastTransportStartMs = 0;
bool brokerUnhealthy = false;
uint32_t lastTransportStopMs = 0;
bool transportRetryWaiting = false;

void stopTransport(const char* reason);
void updateStatusLED();

// I2C
// Semua sensor I2C menggunakan bus utama ESP32:
// SDA = GPIO 21, SCL = GPIO 22
#define SDA_MAIN 21
#define SCL_MAIN 22

// AS5600
static const uint8_t AS5600_ADDR = 0x36;
static const uint8_t AS5600_RAW_ANGLE_H = 0x0C;

// Offset mekanis arah Utara.
// Isi setelah kalibrasi: arahkan vane ke Utara, lihat SensorAngle,
// lalu masukkan nilai negatif/penyesuaiannya sesuai orientasi pemasangan.
float WIND_DIR_OFFSET_DEG = 0.0f;

// AS5600 dibaca tiap detik. Rata-rata arah memakai vektor sinus/kosinus.
float currentWindDirectionDeg = 0;
int currentAS5600Raw = 0;
float currentAS5600SensorDeg = 0.0f;
bool as5600Valid = false;

bool as5600ReadRaw(uint16_t &rawOut) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_H);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received = Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (received != 2) {
    return false;
  }

  uint8_t high = Wire.read();
  uint8_t low  = Wire.read();

  rawOut = ((uint16_t)(high & 0x0F) << 8) | low;
  return true;
}

bool updateWindDirectionAS5600() {
  uint16_t raw = 0;

  if (!as5600ReadRaw(raw)) {
    as5600Valid = false;
    return false;
  }

  float sensorDeg = ((float)raw * 360.0f) / 4096.0f;

  // Pertahankan fungsi kalibrasi vane yang sudah ada pada firmware asli.
  // Setelah sistem stabil, koreksi angular lebih baik dapat disederhanakan
  // menjadi zero-offset + wrapDeg bila diperlukan.
  float correctedDeg = cal_VANE(sensorDeg + WIND_DIR_OFFSET_DEG);

  currentAS5600Raw = (int)raw;
  currentAS5600SensorDeg = sensorDeg;
  currentWindDirectionDeg = fmodf(fmodf(correctedDeg, 360.0f) + 360.0f, 360.0f);
  as5600Valid = true;

  return true;
}

// Rain
#define RAIN_PIN 13
const float MM_PER_TICK = 0.2f;
volatile uint32_t rainTicks = 0;
volatile uint32_t lastRainIsrMicros = 0;

void IRAM_ATTR rainISR() {
  uint32_t now = micros();
  if (now - lastRainIsrMicros > 3000) {
    rainTicks++;
    lastRainIsrMicros = now;
  }
}

void readRainMetrics(uint32_t intervalMs, float &rainIntervalMm, float &rainRateMmPerHour, float &totalRainMm) {
  static uint32_t lastTicksSnapshot = 0;

  noInterrupts();
  uint32_t ticksNow = rainTicks;
  interrupts();

  uint32_t deltaTicks = ticksNow - lastTicksSnapshot;
  lastTicksSnapshot = ticksNow;

  rainIntervalMm = deltaTicks * MM_PER_TICK;
  totalRainMm = ticksNow * MM_PER_TICK;
  rainRateMmPerHour = rainIntervalMm * (3600000.0f / (float)intervalMs);
}

// Anemo
#define ANEMO_PIN 14
volatile unsigned long pulseCount = 0;
unsigned long lastCheck = 0;
float lastWindSpeed = 0;

void IRAM_ATTR anemoISR() { pulseCount++; }

float readWindSpeed() {
  if (millis() - lastCheck < SAMPLE_INTERVAL_MS) return lastWindSpeed;

  uint32_t now = millis();
  uint32_t elapsedMs = now - lastCheck;
  if (elapsedMs == 0) return lastWindSpeed;

  noInterrupts();
  unsigned long count = pulseCount;
  pulseCount = 0;
  interrupts();

  float rps = count / (elapsedMs / 1000.0f);
  float speed_raw = (-0.0181f * rps * rps) + (1.3859f * rps) + 1.4055f;
  if (speed_raw < 1.5f) speed_raw = 0;

  float speed_cal = cal_ANEMO(speed_raw);
  lastWindSpeed = (speed_cal < 0) ? 0 : speed_cal;
  lastCheck = now;
  return lastWindSpeed;
}

// Ultrasonic -> water level
#define TRIG 4
#define ECHO 5
// Calibration relation: measured_distance = -1.0257 * reference_level + 88.00
static const float WATER_ZERO_DISTANCE_CM = 88.00f;
static const float WATER_LEVEL_MAX_CM = 85.0f; // physical installation limit

float readUltrasonicDistanceCm() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long dur = pulseIn(ECHO, HIGH, 40000);
  if (dur == 0) return -1;
  return dur * 0.0343f / 2.0f;
}

float computeWaterLevelFromDistance(float distanceCm) {
  if (distanceCm < 0) return 0.0f;
  float level = (WATER_ZERO_DISTANCE_CM - distanceCm) / 1.0257f;
  return clampRange(level, 0.0f, WATER_LEVEL_MAX_CM);
}

// Time
// Tidak bergantung pada WiFi. Jika NTP belum sinkron, gunakan uptime.
String getTimeString() {
  time_t now = time(nullptr);
  if (now < 1704067200) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "UNSYNCED+%lus", millis() / 1000UL);
    return String(buffer);
  }

  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  char buffer[25];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buffer);
}

// SD
#define SD_CS 15
bool sdReady = false;
uint32_t lastSDRetryMs = 0;
static const uint32_t SD_RETRY_INTERVAL_MS = 30000UL;

void createCSVHeader() {
  if (SD.exists("/data_minute.csv")) return;
  File f = SD.open("/data_minute.csv", FILE_WRITE);
  if (!f) { csvFault = true; return; }

  f.println(
    "timestamp,"
    "altitude,co2,arah_angin,curah_hujan,current,intensitas_cahaya,kecepatan_angin,kelembapan,ketinggian_air_laut,energy_mWh,tekanan_udara,temperature,voltage,"
    "rain_rate_mmh,rain_interval_mm,total_rain_mm"
  );
  f.flush();
  if (f.size() == 0) csvFault = true;
  f.close();
}

void appendCSV(
  const String& timestamp,
  float altitude, float co2, float arah_angin_deg,
  float curah_hujan, float current, float intensitas_cahaya,
  float kecepatan_angin, float kelembapan, float ketinggian_air_laut,
  float power, float tekanan_udara, float temperature, float voltage,
  float rain_rate_mmh, float rain_interval_mm, float total_rain_mm
) {
  String row;
  row.reserve(384);
  row = timestamp;
  row += "," + String(altitude, (unsigned int)DP_ALTITUDE);
  row += "," + String(co2, 0U);
  row += "," + String(arah_angin_deg, 0U);
  row += "," + String(curah_hujan, (unsigned int)DP_RAIN);
  row += "," + String(current, (unsigned int)DP_CURRENT);
  row += "," + String(intensitas_cahaya, (unsigned int)DP_LUX);
  row += "," + String(kecepatan_angin, (unsigned int)DP_WIND_SPEED);
  row += "," + String(kelembapan, (unsigned int)DP_RH);
  row += "," + String(ketinggian_air_laut, (unsigned int)DP_WATER);
  row += "," + String(power, (unsigned int)DP_POWER);
  row += "," + String(tekanan_udara, (unsigned int)DP_PRESSURE);
  row += "," + String(temperature, (unsigned int)DP_TEMP);
  row += "," + String(voltage, (unsigned int)DP_VOLTAGE);
  row += "," + String(rain_rate_mmh, (unsigned int)DP_RAIN);
  row += "," + String(rain_interval_mm, (unsigned int)DP_RAIN);
  row += "," + String(total_rain_mm, (unsigned int)DP_RAIN);
  row += '\n';
  File f = SD.open("/data_minute.csv", FILE_APPEND);
  if (!f) { csvFault = true; sdReady = false; Serial.println("SD CSV open error!"); return; }
  size_t written = f.print(row);
  f.flush();
  f.close();
  csvFault = written != row.length();
  if (csvFault) Serial.println("SD CSV write tidak lengkap!");
}

// WiFi/MQTT
// Scan WiFi async. Operasi WSS/TLS/MQTT masih sinkron sesuai pustaka.
// Timeout MQTT dibatasi; logging/sampling dilanjutkan setelah operasi selesai.
static const uint32_t WIFI_RETRY_INTERVAL_MS = 10000UL;
static const uint32_t MQTT_RETRY_INTERVAL_MS = 10000UL;
uint32_t lastWiFiAttemptMs = 0;
uint32_t lastMQTTAttemptMs = 0;
bool mqttTransportStarted = false;

// ===== MULTI-WIFI + PILIH AP TERKUAT (NON-BLOCKING) =====
// Alur: scan async -> pilih (SSID terdaftar, BSSID) dengan RSSI tertinggi ->
// konek dikunci ke BSSID itu. Saat sudah tersambung, dicek berkala apakah ada
// AP lain yang jauh lebih kuat (hysteresis) lalu pindah otomatis.
static const uint32_t WIFI_SCAN_TIMEOUT_MS    = 15000UL;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000UL;
static const uint32_t WIFI_FAIL_BLACKLIST_MS  = 60000UL;   // AP gagal dilewati sementara
static const uint32_t ROAM_CHECK_INTERVAL_MS  = 120000UL;  // cek AP lebih kuat tiap 2 menit
static const int32_t  ROAM_HYSTERESIS_DB      = 8;         // pindah jika lebih kuat >= 8 dB
static const int32_t  ROAM_SKIP_ABOVE_DBM    = -55;       // sinyal sudah sangat bagus -> tidak perlu scan

struct WifiCandidate {
  bool    valid;
  uint8_t credIdx;
  uint8_t bssid[6];
  int32_t channel;
  int32_t rssi;
};

enum WifiState : uint8_t { WS_IDLE, WS_SCANNING, WS_CONNECTING };
static WifiState     wifiState = WS_IDLE;
static uint32_t      wifiStateSinceMs = 0;
static bool          scanIsRoam = false;
static bool          scanForRecovery = false;
static WifiCandidate scanBest = {};
static WifiCandidate pendingCand = {};
struct FailedAP { uint8_t bssid[6]; uint32_t since; bool used; };
static FailedAP failedAPs[6] = {};
static uint8_t failedAPNext = 0;
static uint32_t      lastRoamCheckMs = 0;

static bool isRecentlyFailed(const uint8_t* b) {
  for (const auto &entry : failedAPs) {
    if (entry.used && millis() - entry.since < WIFI_FAIL_BLACKLIST_MS &&
        memcmp(b, entry.bssid, 6) == 0) return true;
  }
  return false;
}

static void rememberFailedAP(const uint8_t* b) {
  if (!b) return;
  for (auto &entry : failedAPs) {
    if (entry.used && memcmp(b, entry.bssid, 6) == 0) {
      entry.since = millis(); return;
    }
  }
  FailedAP &entry = failedAPs[failedAPNext];
  memcpy(entry.bssid, b, 6); entry.since = millis(); entry.used = true;
  failedAPNext = (failedAPNext + 1) % 6;
}

static void startWifiScan(bool forRoam) {
  WiFi.scanDelete();
  int r = WiFi.scanNetworks(true, false);   // async, tanpa SSID tersembunyi
  if (r == WIFI_SCAN_FAILED) {
    Serial.println("WiFi: scan gagal dimulai.");
    wifiState = WS_IDLE;
    lastWiFiAttemptMs = millis();
    lastRoamCheckMs = millis();
    return;
  }
  scanIsRoam = forRoam;
  scanForRecovery = false;
  wifiState = WS_SCANNING;
  wifiStateSinceMs = millis();
  Serial.println(forRoam ? "WiFi: scan untuk cek AP yang lebih kuat..."
                         : "WiFi: scan jaringan...");
}

// Pilih AP terkuat dari hasil scan, hanya untuk SSID yang ada di WIFI_LIST.
// Hasil disimpan di scanBest.
static bool pickBestFromScan(int n, bool verbose) {
  scanBest.valid = false;
  scanBest.rssi = -127;

  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    for (uint8_t c = 0; c < WIFI_COUNT; c++) {
      if (s != WIFI_LIST[c].ssid) continue;

      const uint8_t* b = WiFi.BSSID(i);
      if (b == nullptr || isRecentlyFailed(b)) continue;

      int32_t r = WiFi.RSSI(i);
      if (verbose) {
        Serial.printf("  - \"%s\"  %s  ch%d  %d dBm\n",
                      s.c_str(), WiFi.BSSIDstr(i).c_str(), (int)WiFi.channel(i), (int)r);
      }
      if (!scanBest.valid || r > scanBest.rssi) {
        scanBest.valid = true;
        scanBest.credIdx = c;
        memcpy(scanBest.bssid, b, 6);
        scanBest.channel = WiFi.channel(i);
        scanBest.rssi = r;
      }
    }
  }
  return scanBest.valid;
}

static void connectToBest() {
  const WifiCred& cr = WIFI_LIST[scanBest.credIdx];
  Serial.printf("WiFi: menghubungkan ke \"%s\" BSSID %02X:%02X:%02X:%02X:%02X:%02X ch%d (%d dBm)\n",
                cr.ssid,
                scanBest.bssid[0], scanBest.bssid[1], scanBest.bssid[2],
                scanBest.bssid[3], scanBest.bssid[4], scanBest.bssid[5],
                (int)scanBest.channel, (int)scanBest.rssi);

  pendingCand = scanBest;
  stopTransport("pindah AP");
  WiFi.disconnect(false, false);
  const char* pw = (strlen(cr.pass) == 0) ? nullptr : cr.pass;
  WiFi.begin(cr.ssid, pw, scanBest.channel, scanBest.bssid);   // dikunci ke AP terpilih

  wifiState = WS_CONNECTING;
  wifiStateSinceMs = millis();
  lastWiFiAttemptMs = millis();
}

void wifiInit() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);   // pemilihan AP dikelola serviceWiFi(), bukan auto-reconnect bawaan
  wifiState = WS_IDLE;
  lastWiFiAttemptMs = 0;
  Serial.printf("WiFi: %u jaringan terdaftar.\n", (unsigned)WIFI_COUNT);
  startWifiScan(false);
}

// Dipanggil tiap loop(). Return true jika WiFi tersambung.
static bool serviceWiFi(uint32_t now) {
  // Beri jeda singkat setelah begin() agar status lama dari AP sebelumnya hilang.
  if (wifiState == WS_CONNECTING && now - wifiStateSinceMs < 500) return false;

  if (WiFi.status() == WL_CONNECTED) {
    if (wifiState == WS_CONNECTING) {
      wifiState = WS_IDLE;
      lastRoamCheckMs = now;
      stopTransport("WiFi mendapatkan koneksi baru");
      transportRetryWaiting = false;
      brokerUnhealthy = true;
      brokerUnhealthySinceMs = now;
      Serial.printf("WiFi: TERHUBUNG \"%s\" BSSID %s ch%d RSSI %d dBm IP %s\n",
                    WiFi.SSID().c_str(), WiFi.BSSIDstr().c_str(),
                    (int)WiFi.channel(), (int)WiFi.RSSI(),
                    WiFi.localIP().toString().c_str());
    }

    if (wifiState == WS_SCANNING) {
      // Scan roaming sedang berjalan.
      int n = WiFi.scanComplete();
      if (n == WIFI_SCAN_RUNNING) {
        if (now - wifiStateSinceMs > WIFI_SCAN_TIMEOUT_MS) {
          WiFi.scanDelete();
          wifiState = WS_IDLE;
          lastRoamCheckMs = now;
        }
        return true;
      }

      wifiState = WS_IDLE;
      lastRoamCheckMs = now;

      if (n > 0 && pickBestFromScan(n, false)) {
        const uint8_t* cur = WiFi.BSSID();
        int32_t curRssi = WiFi.RSSI();
        bool sameAp = (cur != nullptr) && memcmp(cur, scanBest.bssid, 6) == 0;

        if (!sameAp && (scanForRecovery || scanBest.rssi >= curRssi + ROAM_HYSTERESIS_DB)) {
          Serial.printf("WiFi: AP alternatif dipilih (%d dBm vs %d dBm sekarang), pindah...\n",
                        (int)scanBest.rssi, (int)curRssi);
          WiFi.scanDelete();
          connectToBest();
          return false;
        }
      }
      WiFi.scanDelete();
      return true;
    }

    // Tersambung & idle: cek berkala apakah ada AP yang lebih kuat.
    if (wifiState == WS_IDLE && now - lastRoamCheckMs >= ROAM_CHECK_INTERVAL_MS) {
      lastRoamCheckMs = now;
      if (WiFi.RSSI() < ROAM_SKIP_ABOVE_DBM) startWifiScan(true);
    }
    return true;
  }

  // ---- Belum tersambung ----
  switch (wifiState) {
    case WS_IDLE:
      if (lastWiFiAttemptMs == 0 || now - lastWiFiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
        startWifiScan(false);
      }
      break;

    case WS_SCANNING: {
      int n = WiFi.scanComplete();
      if (n >= 0 && offlineScanPasses < 255) ++offlineScanPasses;
      if (n == WIFI_SCAN_RUNNING) {
        if (now - wifiStateSinceMs > WIFI_SCAN_TIMEOUT_MS) {
          WiFi.scanDelete();
          wifiState = WS_IDLE;
          lastWiFiAttemptMs = now;
        }
        break;
      }

      wifiState = WS_IDLE;
      lastWiFiAttemptMs = now;

      if (n > 0 && pickBestFromScan(n, true)) {
        WiFi.scanDelete();
        connectToBest();
      } else {
        WiFi.scanDelete();
        Serial.println("WiFi: tidak ada jaringan terdaftar yang terdeteksi.");
      }
      break;
    }

    case WS_CONNECTING:
      if (now - wifiStateSinceMs >= WIFI_CONNECT_TIMEOUT_MS) {
        Serial.println("WiFi: gagal terhubung (timeout). AP dilewati sementara, scan ulang.");
        rememberFailedAP(pendingCand.bssid);
        WiFi.disconnect(false, false);
        wifiState = WS_IDLE;
        lastWiFiAttemptMs = 0;   // scan ulang segera, pilih AP lain
      }
      break;
  }
  return false;
}

void stopTransport(const char* reason) {
  if (mqttTransportStarted) {
    Serial.printf("WSS reset: %s\n", reason);
    client.disconnect(); // paksa transport mati; tidak menunggu MQTT DISCONNECT
  }
  mqttTransportStarted = false;
  sessionHasAck = false;
  liveSessionHasAck = false;
  lastTransportStopMs = millis();
  transportRetryWaiting = true;
}

void markBrokerUnhealthy() {
  sessionHasAck = false;
  liveSessionHasAck = false;
  if (!brokerUnhealthy) {
    brokerUnhealthy = true;
    brokerUnhealthySinceMs = millis();
  }
}

void startMQTTTransport() {
  // begin() pada MQTT memasang callback internal WebSocket. Jangan menimpa
  // dengan client.onEvent(), karena PUBACK/data MQTT tidak akan terbaca.
  // Pustaka tidak mengosongkan flag is_connected/buffer RX di begin().
  // Buat ulang objek di alamat yang sama agar sesi baru benar-benar bersih.
  mqtt.~ReliableMqtt();
  new (&mqtt) ReliableMqtt();
  mqtt.begin(client);
  mqtt.setTimeout(MQTT_TIMEOUT_MS);
  mqtt.setKeepAliveTimeout(20);
  mqtt.setKeepAliveSendInterval(1000);
  client.beginSSL(MQTT_HOST, MQTT_PORT, MQTT_PATH, nullptr, "mqtt");
  client.setReconnectInterval(5000);
  client.enableHeartbeat(10000, 3000, 2);
  mqttTransportStarted = true;
  lastTransportStartMs = millis();
  lastMQTTAttemptMs = millis() - MQTT_RETRY_INTERVAL_MS;
  Serial.println("WSS mulai; heartbeat 10 s, timeout PONG 3 s (2 kegagalan).");
}

void serviceNetwork() {
  if (modemRecoveryBusy()) return;
  uint32_t now = millis();
  if (!serviceWiFi(now)) {
    if (mqttTransportStarted) stopTransport("WiFi terputus/belum siap");
    markBrokerUnhealthy();
    return;
  }
  // Sejak koneksi terakhir gagal, konek ulang MQTT belum membuktikan publish.
  if (sessionHasAck && now - lastPublishAckMs > ACK_FRESH_MS &&
      !storageFault && !csvFault && !ramOverflowFault) {
    publishFault = true;
    markBrokerUnhealthy();
    stopTransport("lebih dari 90 s tanpa PUBACK pengiriman data");
  }

  if (brokerUnhealthy && now - brokerUnhealthySinceMs >= BROKER_FAILOVER_MS &&
      wifiState == WS_IDLE) {
    Serial.println("Broker belum terkonfirmasi 120 s: cari AP alternatif, tanpa syarat RSSI lebih kuat.");
    rememberFailedAP(WiFi.BSSID());
    startWifiScan(true); // tetap gunakan AP sekarang sampai alternatif ditemukan
    scanForRecovery = true;
    brokerUnhealthySinceMs = now; // jeda sebelum upaya failover berikutnya
  }
  if (!mqttTransportStarted) {
    if (transportRetryWaiting && now - lastTransportStopMs < 5000UL) return;
    transportRetryWaiting = false;
    startMQTTTransport();
  }

  client.loop();
  if (!client.isConnected()) {
    markBrokerUnhealthy();
    if (millis() - lastTransportStartMs >= TRANSPORT_RESTART_MS)
      stopTransport("WSS tidak tersambung dalam 30 s");
    return;
  }
  if (mqtt.isConnected()) {
    if (!mqtt.update()) {
      Serial.printf("MQTT update gagal, error=%d\n", (int)mqtt.getLastError());
      markBrokerUnhealthy();
      stopTransport("MQTT keepalive/read gagal");
    }
    return;
  }
  markBrokerUnhealthy();
  if (millis() - lastMQTTAttemptMs < MQTT_RETRY_INTERVAL_MS) return;
  lastMQTTAttemptMs = millis();
  if (mqtt.connect(runtimeClientId)) {
    Serial.println("MQTT CONNECT OK; menunggu PUBACK data sebelum LED hijau.");
    // CONNACK membuktikan broker terjangkau, tetapi bukan bukti data terkirim.
    // Jangan menyalahkan WiFi jika SD bermasalah sehingga data belum dikirim.
    if (!publishFault) brokerUnhealthy = false;
  } else {
    Serial.printf("MQTT CONNECT gagal: error=%d, return=%d\n",
                  (int)mqtt.getLastError(), (int)mqtt.getReturnCode());
    stopTransport("MQTT CONNECT gagal");
  }
}

// Task LED memiliki pin setelah setup selesai. Snapshot melalui queue sehingga
// LED tetap kedaluwarsa ketika loop utama tertahan operasi jaringan/SD.
struct DeliveryLedSnapshot { uint32_t mainAtMs; uint32_t liveAckAtMs; uint8_t mode; };
QueueHandle_t deliveryLedQueue = nullptr;
static const uint32_t MAIN_LED_LEASE_MS = 10000UL;

void deliveryLedTask(void*) {
  DeliveryLedSnapshot state = {0, 0, 1};
  for (;;) {
    DeliveryLedSnapshot next;
    if (xQueueReceive(deliveryLedQueue, &next, 0) == pdTRUE) state = next;
    uint32_t now = millis();
    if (now - state.mainAtMs > MAIN_LED_LEASE_MS) ledRed();
    else if (state.mode == 0) ledBlueBlink(now, 400);
    else if (state.mode == 2 && now - state.liveAckAtMs <= ACK_FRESH_MS) ledGreen();
    else ledRed();
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void startDeliveryLedTask() {
  deliveryLedQueue = xQueueCreate(1, sizeof(DeliveryLedSnapshot));
  if (!deliveryLedQueue || xTaskCreate(deliveryLedTask, "delivery_led", 2048,
                                     nullptr, 1, nullptr) != pdPASS) {
    ledWorkerFault = true;
    if (deliveryLedQueue) { vQueueDelete(deliveryLedQueue); deliveryLedQueue = nullptr; }
    ledRed();
    Serial.println("LED ERROR: task LED gagal dibuat; hijau dinonaktifkan.");
  }
}

void updateStatusLED() {
  uint32_t now = millis();
  uint8_t mode = 1;
  if (modemRecoveryBusy() || WiFi.status() != WL_CONNECTED || wifiState == WS_CONNECTING) mode = 0;
  else if (mqttTransportStarted && client.isConnected() && mqtt.isConnected() &&
           liveSessionHasAck && liveConfirmedLatest && now - lastLiveAckMs <= ACK_FRESH_MS &&
           !livePublishFault && !backlogPublishFault && !publishFault && !storageFault &&
           !csvFault && !ramOverflowFault && !probeWorkerFault && !deliveryLogFault && !ledWorkerFault) mode = 2;
  if (deliveryLedQueue) {
    DeliveryLedSnapshot state = {now, lastLiveAckMs, mode};
    xQueueOverwrite(deliveryLedQueue, &state);
  } else ledRed();
}

void logDelivery(const char* kind, const char* result, const String& id, int error) {
  Serial.printf("DELIVERY %s %s id=%s error=%d\n", kind, result, id.c_str(), error);
  if (!sdReady) return;
  File f = SD.open("/delivery_events.csv", FILE_APPEND);
  if (!f) { deliveryLogFault = true; return; }
  String row = String(millis()) + "," + String((unsigned long)time(nullptr)) + "," +
      kind + "," + result + "," + id + "," + String(error) + "\n";
  size_t written = f.print(row); f.flush(); f.close();
  deliveryLogFault = written != row.length();
}

bool publishAcknowledged(const String& payload, bool isLive) {
  if (modemRecoveryBusy()) return false;
  if (WiFi.status() != WL_CONNECTED || !mqttTransportStarted ||
      !client.isConnected() || !mqtt.isConnected()) return false;
  // QoS 1: true hanya setelah pustaka menerima PUBACK dari broker.
  updateStatusLED();
  bool ok = mqtt.publish(MQTT_TOPIC, payload, false, 1);
  if (ok) {
    lastPublishAckMs = millis();
    sessionHasAck = true;
    brokerUnhealthy = false;
    publishFault = false;
    failPublishCount = 0;
    if (isLive) {
      lastLiveAckMs = lastPublishAckMs;
      liveSessionHasAck = true;
      liveConfirmedLatest = true;
      livePublishFault = false;
    } else backlogPublishFault = false;
    Serial.println(isLive ? "LIVE ACK: broker menerima data live." : "BACKLOG ACK: broker menerima satu data lama.");
  } else {
    if (failPublishCount < UINT32_MAX) ++failPublishCount;
    publishFault = true;
    if (isLive) livePublishFault = true;
    else backlogPublishFault = true;
    markBrokerUnhealthy();
    Serial.printf("PUBLISH gagal: error=%d; jumlah gagal=%lu; data tetap diantre.\n",
                  (int)mqtt.getLastError(), (unsigned long)failPublishCount);
    stopTransport("publish/PUBACK gagal");
  }
  return ok;
}

// ===== ANTREAN PERSISTEN =====
// Journal append-only: tidak dihapus ketika habis. Posisi ACK baru maju setelah
// PUBACK DAN checkpoint berhasil disimpan. Crash dapat mengulang pesan, bukan
// sengaja melompati pesan. Simpan/cadangkan journal secara berkala saat alat mati.
static const char* PENDING_FILE = "/pending.txt";
static const char* CHECKPOINT_FILE = "/pending_ack.log";
static const char* LEGACY_OFFSET_FILE = "/pending.off";
// Tidak ada flush antrean bebas. Satu izin hanya datang dari satu ACK live.
static const uint32_t OUTBOX_RETRY_MS = 5000UL;
static const size_t MAX_PAYLOAD = 1024;
bool backlogExists = false, queueInitialized = false;
uint32_t pendingOffset = 0;
static const uint8_t RAM_QUEUE_SIZE = 8;
String ramQueue[RAM_QUEUE_SIZE];
uint8_t ramHead = 0, ramCount = 0;

uint32_t offsetCRC(uint32_t offset) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (uint8_t byte = 0; byte < 4; ++byte) {
    crc ^= (offset >> (8 * byte)) & 0xFF;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320UL : 0);
  }
  return ~crc;
}

bool isQueueBoundary(File &f, uint32_t offset) {
  if (offset == 0) return true;
  if (offset > f.size() || !f.seek(offset - 1)) return false;
  return f.read() == '\n';
}

bool initializeQueue() {
  if (!sdReady) return false;
  if (queueInitialized) return true;
  pendingOffset = 0;
  File data = SD.open(PENDING_FILE, FILE_APPEND);
  if (!data) { storageFault = true; return false; }
  data.close();
  data = SD.open(PENDING_FILE, FILE_READ);
  if (!data) { storageFault = true; return false; }
  bool hadCheckpoint = SD.exists(CHECKPOINT_FILE);
  if (hadCheckpoint) {
    File checkpoints = SD.open(CHECKPOINT_FILE, FILE_READ);
    if (!checkpoints) { data.close(); storageFault = true; return false; }
    while (checkpoints.available()) {
      char line[64];
      size_t n = checkpoints.readBytesUntil('\n', line, sizeof(line) - 1);
      line[n] = 0;
      // Checkpoint ditulis dengan pemisah newline awal, sehingga baris parsial
      // akibat mati listrik tidak menempel ke checkpoint baru.
      if (n == sizeof(line) - 1) {
        while (checkpoints.available() && checkpoints.read() != '\n') {}
        continue;
      }
      unsigned long offset, crc;
      char extra;
      if (sscanf(line, "%lu,%lx %c", &offset, &crc, &extra) == 2 &&
          (uint32_t)crc == offsetCRC((uint32_t)offset) &&
          offset >= pendingOffset && isQueueBoundary(data, (uint32_t)offset))
        pendingOffset = (uint32_t)offset;
    }
    checkpoints.close();
  } else if (SD.exists(LEGACY_OFFSET_FILE)) {
    // Pertahankan antrean firmware lama. Offset lama tanpa CRC hanya diterima
    // bila menunjuk batas baris; bila meragukan, kirim ulang mulai awal.
    File legacy = SD.open(LEGACY_OFFSET_FILE, FILE_READ);
    if (!legacy) { data.close(); storageFault = true; return false; }
    String value = legacy.readStringUntil('\n'); value.trim(); legacy.close();
    bool digits = value.length() > 0 && value.length() <= 10;
    for (size_t i = 0; i < value.length(); ++i)
      if (value[i] < '0' || value[i] > '9') digits = false;
    uint64_t offset = strtoull(value.c_str(), nullptr, 10);
    if (digits && offset <= UINT32_MAX && isQueueBoundary(data, (uint32_t)offset))
      pendingOffset = (uint32_t)offset;
  }
  backlogExists = pendingOffset < data.size();
  data.close();
  queueInitialized = true;
  Serial.printf("Antrean siap: offset ACK=%lu; ada sisa=%d\n",
                (unsigned long)pendingOffset, backlogExists);
  return true;
}

bool savePendingOffset(uint32_t offset) {
  File f = SD.open(CHECKPOINT_FILE, FILE_APPEND);
  if (!f) { storageFault = true; return false; }
  char record[48];
  int length = snprintf(record, sizeof(record), "\n%lu,%08lX\n",
                        (unsigned long)offset, (unsigned long)offsetCRC(offset));
  size_t written = f.write((const uint8_t*)record, length);
  f.flush(); f.close();
  if (written != (size_t)length) { storageFault = true; return false; }
  // Baca ulang checkpoint terakhir sebelum memajukan posisi dalam RAM.
  File verify = SD.open(CHECKPOINT_FILE, FILE_READ);
  if (!verify || verify.size() < (size_t)length ||
      !verify.seek(verify.size() - length)) {
    if (verify) verify.close(); storageFault = true; return false;
  }
  char copy[48] = {};
  size_t read = verify.readBytes(copy, length); verify.close();
  if (read != (size_t)length || memcmp(copy, record, length) != 0) {
    storageFault = true; return false;
  }
  return true;
}

bool queuePayloadSD(const String& payload) {
  if (!initializeQueue()) return false;
  // Jangan menyambung JSON baru ke ekor record yang terpotong.
  File tail = SD.open(PENDING_FILE, FILE_READ);
  if (!tail) { storageFault = true; return false; }
  bool complete = isQueueBoundary(tail, tail.size());
  tail.close();
  if (!complete) {
    Serial.println("SD ERROR: baris antrean terakhir terpotong; perlu pemeriksaan SD.");
    storageFault = true; return false;
  }
  File f = SD.open(PENDING_FILE, FILE_APPEND);
  if (!f) { storageFault = true; return false; }
  uint32_t start = f.size();
  size_t written = f.print(payload);
  size_t newline = f.print('\n');
  f.flush(); f.close();
  if (written != payload.length() || newline != 1) {
    Serial.println("SD ERROR: tulis antrean tidak lengkap; data mungkin parsial, jangan dilewati.");
    storageFault = true; return false;
  }
  File check = SD.open(PENDING_FILE, FILE_READ);
  if (!check || !check.seek(start)) {
    if (check) check.close(); storageFault = true; return false;
  }
  String readback = check.readStringUntil('\n'); check.close();
  if (readback != payload) { storageFault = true; return false; }
  backlogExists = true;
  return true;
}

bool enqueuePayload(const String& payload) {
  // Bila RAM masih berisi data lama, tambahkan di belakangnya agar urutannya
  // tidak dilompati oleh data baru yang kebetulan berhasil ditulis ke SD.
  if (ramCount == 0 && queuePayloadSD(payload)) return true;
  if (ramCount >= RAM_QUEUE_SIZE) {
    ramOverflowFault = true;
    Serial.println("DATA LOSS: SD tidak dapat menampung dan antrean RAM penuh (8 pesan). Periksa CSV/Serial.");
    return false;
  }
  ramQueue[(ramHead + ramCount) % RAM_QUEUE_SIZE] = payload;
  ++ramCount;
  Serial.printf("Antrean RAM: %u/8; hilang jika listrik mati sebelum tersimpan/terkirim.\n", ramCount);
  return true;
}

// Satu outbox persisten untuk live yang belum selesai. Antrean lama tetap
// menggunakan pending.txt + pending_ack.log + pending.off agar data lama terbaca.
static const char* LIVE_OUTBOX_FILE = "/live_outbox.json";
String pendingLive, pendingLiveId, pairedLiveId;
uint32_t pendingLiveEndMs = 0;
bool pendingLiveOnDisk = false;
bool outboxRecovered = false;
bool outboxArchived = false;
bool backlogPermit = false;

String payloadId(const String& payload) {
  StaticJsonDocument<1536> doc;
  if (deserializeJson(doc, payload)) return String("invalid");
  const char* id = doc["message_id"] | "legacy";
  return String(id);
}

bool removeLiveOutbox() {
  bool ok = !SD.exists(LIVE_OUTBOX_FILE) || SD.remove(LIVE_OUTBOX_FILE);
  if (!ok) storageFault = true;
  outboxRecovered = ok;
  if (ok) outboxArchived = false;
  return ok;
}

void recoverLiveOutbox() {
  if (!sdReady || outboxRecovered || pendingLiveOnDisk) return;
  static uint32_t lastTry = 0;
  static bool tried = false;
  if (tried && millis() - lastTry < OUTBOX_RETRY_MS) return;
  tried = true; lastTry = millis();
  if (!SD.exists(LIVE_OUTBOX_FILE)) { outboxRecovered = true; return; }
  if (outboxArchived) { removeLiveOutbox(); return; }
  File f = SD.open(LIVE_OUTBOX_FILE, FILE_READ);
  if (!f || f.size() > MAX_PAYLOAD) {
    if (f) f.close(); storageFault = true; return;
  }
  String payload = f.readStringUntil('\n'); f.close();
  StaticJsonDocument<1536> check;
  if (deserializeJson(check, payload) || !check["data"].is<JsonObject>()) {
    storageFault = true;
    Serial.println("OUTBOX ERROR: JSON lama tidak lengkap; simpan file untuk pemulihan.");
    return;
  }
  // Tulis ke journal sebelum melepas outbox. Crash dapat mengulang ID yang sama.
  if (queuePayloadSD(payload)) {
    outboxArchived = true;
    if (removeLiveOutbox()) logDelivery("OUTBOX", "RECOVERED_TO_BACKLOG", payloadId(payload), 0);
  }
}

bool persistLiveOutbox(const String& payload) {
  if (!sdReady || !outboxRecovered || ramCount > 0 || SD.exists(LIVE_OUTBOX_FILE)) return false;
  File f = SD.open(LIVE_OUTBOX_FILE, FILE_WRITE);
  if (!f) { storageFault = true; return false; }
  size_t written = f.print(payload), newline = f.print('\n');
  f.flush(); f.close();
  outboxRecovered = false;
  outboxArchived = false;
  if (written != payload.length() || newline != 1) { storageFault = true; return false; }
  File check = SD.open(LIVE_OUTBOX_FILE, FILE_READ);
  if (!check) { storageFault = true; return false; }
  String copy = check.readStringUntil('\n'); check.close();
  if (copy != payload) { storageFault = true; return false; }
  return true;
}

void archivePendingLive() {
  if (!pendingLive.length()) return;
  bool queued = false;
  if (pendingLiveOnDisk) {
    queued = queuePayloadSD(pendingLive);
    if (queued) { outboxArchived = true; removeLiveOutbox(); }
    // Jika journal gagal, outbox di SD tetap menyimpan pesan tersebut.
    pendingLiveOnDisk = false;
  } else queued = enqueuePayload(pendingLive);
  logDelivery("LIVE", queued ? "QUEUED" : "QUEUE_NOT_CONFIRMED", pendingLiveId, 0);
  pendingLive = ""; pendingLiveId = "";
}

bool stageLivePayload(const String& payload, uint32_t endMs) {
  backlogPermit = false; // izin lama tidak pernah dikumpulkan menjadi burst
  liveConfirmedLatest = false;
  livePublishFault = true;
  archivePendingLive();
  recoverLiveOutbox();
  if (millis() - endMs > LIVE_MAX_AGE_MS) {
    bool queued = enqueuePayload(payload);
    logDelivery("LIVE", queued ? "LATE_TO_BACKLOG" : "QUEUE_FAILED", payloadId(payload), 0);
    return queued;
  }
  pendingLive = payload; pendingLiveId = payloadId(payload);
  pendingLiveEndMs = endMs;
  pendingLiveOnDisk = persistLiveOutbox(payload);
  return true;
}

bool buildBacklogPayload(const String& stored, String& outbound) {
  StaticJsonDocument<1536> doc;
  if (deserializeJson(doc, stored) || !doc["data"].is<JsonObject>()) return false;
  doc["delivery_type"] = "backlog";
  doc["paired_live_message_id"] = pairedLiveId;
  if (doc.overflowed()) return false;
  // Metadata tambahan tidak boleh membuat record lama yang valid macet.
  if (measureJson(doc) >= MAX_PAYLOAD) doc.remove("paired_live_message_id");
  if (measureJson(doc) >= MAX_PAYLOAD) doc.remove("delivery_type");
  if (measureJson(doc) >= MAX_PAYLOAD) return false;
  return serializeJson(doc, outbound) == measureJson(doc);
}

void sendOneBacklog() {
  if (backlogExists) {
    if (!sdReady || !queueInitialized) return;
    File f = SD.open(PENDING_FILE, FILE_READ);
    if (!f || pendingOffset > f.size() || !f.seek(pendingOffset)) {
      if (f) f.close(); storageFault = true; return;
    }
    char line[MAX_PAYLOAD + 1];
    size_t length = f.readBytesUntil('\n', line, sizeof(line) - 1);
    line[length] = 0;
    uint32_t next = f.position(), end = f.size();
    bool complete = isQueueBoundary(f, next); f.close();
    if (length >= sizeof(line) - 1 || !complete) { storageFault = true; return; }
    String stored(line), outbound;
    stored.trim();
    if (!buildBacklogPayload(stored, outbound)) { storageFault = true; return; }
    bool ok = publishAcknowledged(outbound, false);
    logDelivery("BACKLOG", ok ? "ACK" : "FAILED", payloadId(stored), (int)mqtt.getLastError());
    if (!ok) return;
    if (!savePendingOffset(next)) {
      logDelivery("BACKLOG", "CHECKPOINT_FAILED", payloadId(stored), 0);
      return;
    }
    pendingOffset = next; backlogExists = next < end; storageFault = false;
    return;
  }
  if (ramCount) {
    String outbound;
    if (!buildBacklogPayload(ramQueue[ramHead], outbound)) { storageFault = true; return; }
    bool ok = publishAcknowledged(outbound, false);
    logDelivery("BACKLOG_RAM", ok ? "ACK" : "FAILED", payloadId(ramQueue[ramHead]), (int)mqtt.getLastError());
    if (!ok) return;
    ramQueue[ramHead] = "";
    ramHead = (ramHead + 1) % RAM_QUEUE_SIZE; --ramCount;
  }
}

void flushBacklog() {
  // Setiap data live dicoba sekali pada siklusnya. Jika gagal/offline, pesan
  // menjadi backlog; reconnect sendiri tidak memberi izin menguras antrean.
  if (pendingLive.length()) {
    backlogPermit = false;
    bool ready = !modemRecoveryBusy() && WiFi.status() == WL_CONNECTED &&
                 mqttTransportStarted && client.isConnected() && mqtt.isConnected();
    bool timely = millis() - pendingLiveEndMs <= LIVE_MAX_AGE_MS;
    bool ok = ready && timely && publishAcknowledged(pendingLive, true);
    logDelivery("LIVE", ok ? "ACK" : "FAILED_OR_OFFLINE", pendingLiveId, (int)mqtt.getLastError());
    if (ok) {
      pairedLiveId = pendingLiveId;
      if (pendingLiveOnDisk) { outboxArchived = true; removeLiveOutbox(); }
      pendingLiveOnDisk = false;
      pendingLive = ""; pendingLiveId = "";
      backlogPermit = true;
    } else {
      livePublishFault = true; liveConfirmedLatest = false;
      archivePendingLive();
    }
    return; // loop berikutnya baru boleh mengirim satu backlog pasangannya
  }
  recoverLiveOutbox();
  // Pemindahan RAM ke SD tetap berjalan saat offline; ini bukan publish.
  // Outbox lama diselesaikan lebih dahulu untuk mempertahankan urutan waktu.
  if (sdReady && outboxRecovered && ramCount && queuePayloadSD(ramQueue[ramHead])) {
    ramQueue[ramHead] = "";
    ramHead = (ramHead + 1) % RAM_QUEUE_SIZE;
    --ramCount;
    storageFault = false;
  }
  if (sdReady && !queueInitialized) initializeQueue();
  if (!backlogPermit || modemRecoveryBusy() || !liveSessionHasAck ||
      !liveConfirmedLatest || millis() - lastLiveAckMs > LIVE_MAX_AGE_MS) return;
  backlogPermit = false; // termasuk kegagalan: tunggu ACK live menit berikutnya
  if (WiFi.status() == WL_CONNECTED && client.isConnected() && mqtt.isConnected()) sendOneBacklog();
}

bool sendFullPayload(
  float arah_angin_deg, float kecepatan_angin, float altitude,
  float curah_hujan, float intensitas_cahaya, float tekanan_udara,
  float temperature, float kelembapan, float co2,
  float ketinggian_air_laut, float voltage, float current, float power,
  time_t ts, uint32_t endMs
) {
  // Siklus baru harus dikonfirmasi ulang, termasuk bila pembentukan JSON gagal.
  backlogPermit = false;
  liveConfirmedLatest = false;
  livePublishFault = true;
  StaticJsonDocument<1536> doc;
  char messageId[64];
  snprintf(messageId, sizeof(messageId), "%s-%lu", bootId, (unsigned long)++messageSequence);
  doc["message_id"] = messageId;
  doc["delivery_type"] = "live";

  // Waktu pengukuran (epoch detik) agar data lama tetap punya waktu yang benar.
  // Hanya diisi jika jam sudah sinkron NTP.
  if (ts > 1704067200) doc["timestamp"] = (uint32_t)ts;

  JsonObject data = doc.createNestedObject("data");

  data["arah_angin"] = arah_angin_deg;
  data["kecepatan_angin"] = kecepatan_angin;
  data["altitude"] = altitude;
  data["curah_hujan"] = curah_hujan;
  data["intensitas_cahaya"] = intensitas_cahaya;
  data["tekanan_udara"] = tekanan_udara;
  data["temperature"] = temperature;
  data["kelembapan"] = kelembapan;
  data["co2"] = co2;
  data["ketinggian_air_laut"] = ketinggian_air_laut;
  data["voltage"] = voltage;
  data["current"] = current;
  // Kontrak baru: power = energi yang dipakai dalam 60 detik (mWh).
  data["power"] = power;
  doc["power_unit"] = "mWh";
  doc["aggregation_seconds"] = 60;
  doc["modem_recovery_count"] = modemRecoveryCount;
  doc["boot_reset_reason"] = (int)esp_reset_reason();

  char payload[MAX_PAYLOAD];
  if (doc.overflowed() || measureJson(doc) >= sizeof(payload)) {
    Serial.println("DATA ERROR: payload melebihi buffer; tidak mengirim JSON terpotong.");
    publishFault = true;
    return false;
  }
  size_t n = serializeJson(doc, payload, sizeof(payload));
  if (n == 0) { publishFault = true; return false; }
  Serial.print("QUEUE FULL: "); Serial.println(payload);
  return stageLivePayload(String(payload), endMs);
}

// Agregator waktu: nilai terakhir mewakili waktu hingga pembacaan berikutnya.
// Tidak ada pembulatan sebelum agregasi. Pembacaan gagal tidak dianggap nol.
struct TimedMean {
  double area = 0;
  uint32_t validMs = 0;
  float held = NAN;
  void set(float value) { held = isfinite(value) ? value : NAN; }
  void integrate(uint32_t dt) {
    if (isfinite(held)) { area += (double)held * dt; validMs += dt; }
  }
  float mean() const { return validMs ? (float)(area / validMs) : NAN; }
  void reset() { area = 0; validMs = 0; } // pertahankan nilai terakhir
};

TimedMean avgAltitude, avgCO2, avgTemp, avgHumidity, avgLux, avgPressure;
TimedMean avgWater, avgVoltage, avgCurrent, avgWind, powerIntegral;
TimedMean windSin, windCos;
bool lightReady = false, bmpReady = false, inaReady = false, scdReady = false;
uint32_t integratedAtMs = 0, windowElapsedMs = 0, lastSampleMs = 0;
float previousTotalRainMmCal = 0;

void integrateSensors(uint32_t dt) {
  avgAltitude.integrate(dt); avgCO2.integrate(dt); avgTemp.integrate(dt);
  avgHumidity.integrate(dt); avgLux.integrate(dt); avgPressure.integrate(dt);
  avgWater.integrate(dt); avgVoltage.integrate(dt); avgCurrent.integrate(dt);
  avgWind.integrate(dt); powerIntegral.integrate(dt);
  windSin.integrate(dt); windCos.integrate(dt);
}

void resetWindow() {
  avgAltitude.reset(); avgCO2.reset(); avgTemp.reset(); avgHumidity.reset();
  avgLux.reset(); avgPressure.reset(); avgWater.reset(); avgVoltage.reset();
  avgCurrent.reset(); avgWind.reset(); powerIntegral.reset();
  windSin.reset(); windCos.reset();
}

float meanDirection() {
  float x = windCos.mean(), y = windSin.mean();
  // Arah berlawanan yang saling meniadakan tidak memiliki rata-rata arah.
  if (!isfinite(x) || !isfinite(y) || hypotf(x, y) < 1.0e-6f) return NAN;
  return (float)wrapDeg(atan2f(y, x) * 180.0f / PI);
}

void sampleSensors() {
  if (updateWindDirectionAS5600()) {
    float radians = currentWindDirectionDeg * PI / 180.0f;
    windSin.set(sinf(radians)); windCos.set(cosf(radians));
  } else { windSin.set(NAN); windCos.set(NAN); }
  avgWind.set(readWindSpeed());

  float rawLux = lightReady ? lightMeter.readLightLevel() : NAN;
  avgLux.set(isfinite(rawLux) && rawLux >= 0 ? clamp0(cal_LUX(rawLux)) : NAN);
  if (bmpReady) {
    float p = bmp.readPressure() / 100.0f;
    float a = bmp.readAltitude(1013.25f);
    avgPressure.set(isfinite(p) && p > 0 ? clamp0(cal_PRES(p)) : NAN);
    avgAltitude.set(cal_ALT(a));
  }
  float distance = readUltrasonicDistanceCm();
  avgWater.set(distance >= 0 ? computeWaterLevelFromDistance(distance) : NAN);

  // SCD40 menghasilkan data baru sekitar tiap 5 detik. readMeasurement()
  // false ketika belum ada data baru: tahan nilai terakhir, jangan hitung nol.
  static uint32_t lastSCDValidMs = 0;
  if (scdReady && scd40.readMeasurement() && scd40.getCO2() > 0) {
    avgCO2.set(clamp0(cal_CO2((float)scd40.getCO2())));
    avgTemp.set(cal_TEMP(scd40.getTemperature()));
    avgHumidity.set(clampRange(cal_RH(scd40.getHumidity()), 0.0f, 100.0f));
    lastSCDValidMs = millis();
  } else if (millis() - lastSCDValidMs > 15000UL) {
    avgCO2.set(NAN); avgTemp.set(NAN); avgHumidity.set(NAN);
  }
  if (inaReady) {
    avgVoltage.set(ina219.getBusVoltage_V() + ina219.getShuntVoltage_mV() / 1000.0f);
    avgCurrent.set(ina219.getCurrent_mA());
    powerIntegral.set(clamp0(ina219.getPower_mW()));
  }
}

void publishMinute(time_t endEpoch, uint32_t endUptimeMs) {
  float arah = meanDirection();
  float speed = roundToDp(avgWind.mean(), DP_WIND_SPEED);
  float altitude = roundToDp(avgAltitude.mean(), DP_ALTITUDE);
  float co2 = roundToDp(avgCO2.mean(), 0);
  float temperature = roundToDp(avgTemp.mean(), DP_TEMP);
  float humidity = roundToDp(avgHumidity.mean(), DP_RH);
  float lux = roundToDp(avgLux.mean(), DP_LUX);
  float pressure = roundToDp(avgPressure.mean(), DP_PRESSURE);
  float water = roundToDp(avgWater.mean(), DP_WATER);
  float voltage = roundToDp(avgVoltage.mean(), DP_VOLTAGE);
  float current = roundToDp(avgCurrent.mean(), DP_CURRENT);
  // mW * ms / 3,600,000 = mWh. Hanya keluarkan energi jika satu menit lengkap.
  float energy = powerIntegral.validMs == MEASUREMENT_INTERVAL_MS
      ? roundToDp((float)(powerIntegral.area / 3600000.0), DP_POWER) : NAN;

  float intervalRaw, rateRaw, totalRaw;
  readRainMetrics(MEASUREMENT_INTERVAL_MS, intervalRaw, rateRaw, totalRaw);
  // Hujan adalah akumulasi, bukan rata-rata tinggi hujan. Intercept kalibrasi
  // diterapkan sekali ke total, kemudian selisihnya menjadi hujan per menit.
  float totalCal = clamp0(cal_RAIN(totalRaw));
  float intervalCal = clamp0(totalCal - previousTotalRainMmCal);
  previousTotalRainMmCal = totalCal;
  float rain = roundToDp(intervalCal, DP_RAIN);
  float rate = roundToDp(intervalCal * 60.0f, DP_RAIN);
  float total = roundToDp(totalCal, DP_RAIN);

  String timestamp;
  if (endEpoch >= 1704067200) {
    struct tm t;
    localtime_r(&endEpoch, &t);
    char buffer[25];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &t);
    timestamp = buffer;
  } else {
    timestamp = String("UNSYNCED+") + String(endUptimeMs / 1000UL) + "s";
  }
  Serial.println("--- Rata-rata 60 detik (timestamp akhir jendela) ---");
  Serial.println(timestamp);
  Serial.printf("Arah: %.0f deg, angin: %.1f m/s, CO2: %.0f ppm\n", arah, speed, co2);
  Serial.printf("Temp: %.1f C, RH: %.0f %%, lux: %.0f, tekanan: %.0f hPa\n",
                temperature, humidity, lux, pressure);
  Serial.printf("V: %.2f V, I: %.1f mA, energi/menit: %.4f mWh (cakupan %lu ms)\n",
                voltage, current, energy, (unsigned long)powerIntegral.validMs);

  if (sdReady && SD.cardType() != CARD_NONE) {
    appendCSV(timestamp, altitude, co2, arah, rain, current, lux, speed,
              humidity, water, energy, pressure, temperature, voltage, rate, rain, total);
  } else { sdReady = false; Serial.println("SD tidak tersedia."); }
  // NaN menjadi null pada JSON; CSV menuliskan nan. Bukan angka nol palsu.
  sendFullPayload(arah, speed, altitude, rain, lux, pressure, temperature,
                  humidity, co2, water, voltage, current, energy, endEpoch, endUptimeMs);
}

void serviceMeasurements() {
  const uint32_t now = millis();
  uint32_t remaining = now - integratedAtMs;
  const time_t nowEpoch = time(nullptr);
  // Pecah integrasi tepat di batas 60 detik, termasuk bila loop sedikit terlambat.
  while (remaining > 0) {
    uint32_t room = MEASUREMENT_INTERVAL_MS - windowElapsedMs;
    uint32_t dt = remaining < room ? remaining : room;
    integrateSensors(dt);
    integratedAtMs += dt;
    windowElapsedMs += dt;
    remaining -= dt;
    if (windowElapsedMs == MEASUREMENT_INTERVAL_MS) {
      time_t endEpoch = nowEpoch >= 1704067200
          ? nowEpoch - (time_t)(remaining / 1000UL) : 0;
      publishMinute(endEpoch, integratedAtMs);
      resetWindow();
      windowElapsedMs = 0;
    }
  }
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    if (now - lastSampleMs > 2000UL)
      Serial.printf("SAMPLE LAG: %lu ms; integrasi memakai sampel terakhir selama jeda.\n",
                    (unsigned long)(now - lastSampleMs));
    lastSampleMs = now;
    sampleSensors();
  }
}

// ===== PEMULIHAN MODEM (semua GPIO/SD/WiFi/MQTT tetap dimiliki loop utama) =====
static const uint32_t PROBE_INTERVAL_MS = 30000UL;
static const uint32_t PROBE_FRESH_MS = 90000UL;
static const int32_t PROBE_CONNECT_TIMEOUT_MS = 1200;
// Uji keterjangkauan TCP, bukan ping ICMP atau bukti akses internet menyeluruh.
// Dua operator independen; alamat literal menghindari DNS yang bisa macet.
static const uint8_t PROBE_A_IP[4] = {1, 1, 1, 1}; // Cloudflare, TCP 443
static const uint16_t PROBE_A_PORT = 443;
static const uint8_t PROBE_B_IP[4] = {8, 8, 8, 8}; // Google Public DNS, TCP 53
static const uint16_t PROBE_B_PORT = 53;

struct ModemProbeRequest { uint32_t token; };
struct ModemProbeResult { uint32_t token; uint32_t finishedAtMs; uint8_t successMask; };
QueueHandle_t modemProbeRequests = nullptr, modemProbeResults = nullptr;
TaskHandle_t modemProbeTaskHandle = nullptr;
ModemRecoveryPolicy modemPolicy;
struct ModemRetainedBudget { uint32_t magic, count, total, check; };
// NOINIT mempertahankan isi saat software restart; validasi dilakukan di begin.
RTC_NOINIT_ATTR ModemRetainedBudget retainedModemBudget;
static const uint32_t MODEM_BUDGET_MAGIC = 0x4D524543UL;

uint32_t probeToken = 0, lastProbeRequestMs = 0, probePendingSinceMs = 0;
uint32_t lastProbeResultMs = 0;
bool probePending = false, probeKnown = false, probeGood = false;
uint8_t failedProbeStreak = 0;
bool haveEverConnectedWiFi = false, lastConnectedWasModem = false;
bool previousLinkUp = false;
uint8_t previousBssid[6] = {};
bool modemRecoveryInitialized = false;

bool modemRecoveryBusy() {
  return modemRecoveryInitialized && modemPolicy.busy();
}

void invalidateModemProbe() {
  ++probeToken; // hasil dari AP/sesi lama tidak boleh mengendalikan GPIO
  probePending = false; probeKnown = false; probeGood = false;
  failedProbeStreak = 0;
  lastProbeRequestMs = millis() - PROBE_INTERVAL_MS;
}

void modemProbeWorker(void*) {
  ModemProbeRequest request;
  for (;;) {
    if (xQueueReceive(modemProbeRequests, &request, portMAX_DELAY) != pdTRUE) continue;
    ModemProbeResult result = {request.token, 0, 0};
    WiFiClient socketA;
    if (socketA.connect(IPAddress(PROBE_A_IP[0], PROBE_A_IP[1], PROBE_A_IP[2], PROBE_A_IP[3]),
                        PROBE_A_PORT, PROBE_CONNECT_TIMEOUT_MS)) result.successMask |= 1;
    socketA.stop();
    if (!result.successMask) {
      WiFiClient socketB;
      if (socketB.connect(IPAddress(PROBE_B_IP[0], PROBE_B_IP[1], PROBE_B_IP[2], PROBE_B_IP[3]),
                          PROBE_B_PORT, PROBE_CONNECT_TIMEOUT_MS)) result.successMask |= 2;
      socketB.stop();
    }
    result.finishedAtMs = millis();
    xQueueOverwrite(modemProbeResults, &result);
  }
}

void saveModemBudgetRTC() {
  uint32_t count = modemPolicy.historyCount, total = modemPolicy.totalCycles;
  retainedModemBudget.count = count;
  retainedModemBudget.total = total;
  retainedModemBudget.check = MODEM_BUDGET_MAGIC ^ count ^ total;
  retainedModemBudget.magic = MODEM_BUDGET_MAGIC;
  modemRecoveryCount = total;
}

void logModemEvent(const char* event, const char* reason) {
  Serial.printf("MODEM %s: %s; total=%lu; siklus/rolling-hour=%u\n",
                event, reason, (unsigned long)modemPolicy.totalCycles, modemPolicy.historyCount);
  if (!sdReady) return;
  File f = SD.open("/modem_events.csv", FILE_APPEND);
  if (!f) { storageFault = true; return; }
  char line[224];
  int length = snprintf(line, sizeof(line), "%lu,%lu,%s,%s,%lu,%u\n",
                       (unsigned long)millis(), (unsigned long)time(nullptr), event, reason,
                       (unsigned long)modemPolicy.totalCycles, modemPolicy.historyCount);
  if (length <= 0 || length >= (int)sizeof(line) ||
      f.write((const uint8_t*)line, length) != (size_t)length) storageFault = true;
  f.flush(); f.close();
}

void applyModemAction(ModemAction action, const char* reason) {
  if (action == ModemAction::PowerOff) {
    // Catat/reset status sesi sebelum melepas catu; tidak menghapus antrean.
    stopTransport("pemulihan modem: power OFF");
    WiFi.scanDelete();
    WiFi.disconnect(false, false);
    wifiState = WS_IDLE;
    invalidateModemProbe();
    previousLinkUp = false;
    digitalWrite(MODEM_POWER_PIN, MODEM_ON_LEVEL == HIGH ? LOW : HIGH);
    modemPolicy.stageSince = millis(); // ukur OFF dari saat GPIO benar-benar diubah
    saveModemBudgetRTC();
    logModemEvent("OFF", reason);
  } else if (action == ModemAction::PowerOn) {
    digitalWrite(MODEM_POWER_PIN, MODEM_ON_LEVEL);
    modemPolicy.stageSince = millis();
    invalidateModemProbe();
    for (auto &ap : failedAPs) ap.used = false; // modem bisa muncul dengan AP yang sama
    offlineScanPasses = 0;
    logModemEvent("ON", "tunggu waktu boot sesuai konfigurasi");
  } else if (action == ModemAction::ResumeNetwork) {
    invalidateModemProbe();
    offlineScanPasses = 0;
    wifiInit();
    transportRetryWaiting = false;
    logModemEvent("READY", "mulai scan dan sambung ulang jaringan");
  }
}

void modemRecoveryBegin() {
  uint8_t carry = 0;
  uint32_t total = 0;
  if (esp_reset_reason() != ESP_RST_POWERON && retainedModemBudget.magic == MODEM_BUDGET_MAGIC) {
    bool valid = retainedModemBudget.count <= ModemRecoveryPolicy::maxCycles &&
                 retainedModemBudget.check == (MODEM_BUDGET_MAGIC ^ retainedModemBudget.count ^ retainedModemBudget.total);
    carry = valid ? (uint8_t)retainedModemBudget.count : ModemRecoveryPolicy::maxCycles;
    total = retainedModemBudget.total;
  }
  modemPolicy.begin(millis(), carry, total);
  modemRecoveryInitialized = true;
  saveModemBudgetRTC();
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  modemProbeRequests = xQueueCreate(1, sizeof(ModemProbeRequest));
  modemProbeResults = xQueueCreate(1, sizeof(ModemProbeResult));
  if (!modemProbeRequests || !modemProbeResults ||
      xTaskCreate(modemProbeWorker, "modem_probe", 4096, nullptr, 1, &modemProbeTaskHandle) != pdPASS) {
    probeWorkerFault = true;
    if (modemProbeRequests) { vQueueDelete(modemProbeRequests); modemProbeRequests = nullptr; }
    if (modemProbeResults) { vQueueDelete(modemProbeResults); modemProbeResults = nullptr; }
    Serial.println("MODEM ERROR: task probe gagal dibuat; pemulihan otomatis dinonaktifkan.");
  }
  Serial.printf("MODEM recovery=%d; target SSID=%s; ON=%s; boot=%lums; OFF=%lums; failure=%lums; max=%u/window=%lums; gap=%lums.\n",
                MODEM_AUTO_RECOVERY && !probeWorkerFault, MODEM_WIFI_SSID,
                MODEM_ON_LEVEL == HIGH ? "HIGH" : "LOW",
                (unsigned long)ModemRecoveryPolicy::bootMs, (unsigned long)ModemRecoveryPolicy::offMs,
                (unsigned long)ModemRecoveryPolicy::failureMs, (unsigned)ModemRecoveryPolicy::maxCycles,
                (unsigned long)ModemRecoveryPolicy::hourMs, (unsigned long)ModemRecoveryPolicy::minCycleGapMs);
  Serial.printf("ESP reset reason=%d; retained modem budget=%u; total=%lu\n",
                (int)esp_reset_reason(), carry, (unsigned long)total);
}

void serviceModemRecovery() {
  static uint32_t lastTickMs = 0;
  static bool wasBudgetBlocked = false;
  if (!modemRecoveryInitialized || millis() - lastTickMs < 250UL) return;
  lastTickMs = millis();
  uint32_t now = lastTickMs;
  if (modemPolicy.busy()) {
    applyModemAction(modemPolicy.tick(now, ModemHealth::Unknown), "timer pemulihan");
    saveModemBudgetRTC();
    return;
  }

  bool linkUp = WiFi.status() == WL_CONNECTED;
  bool onModem = false;
  const uint8_t* bssid = linkUp ? WiFi.BSSID() : nullptr;
  bool linkChanged = linkUp != previousLinkUp ||
                     (linkUp && bssid && memcmp(previousBssid, bssid, 6) != 0);
  if (linkChanged) {
    invalidateModemProbe();
    previousLinkUp = linkUp;
    if (bssid) memcpy(previousBssid, bssid, 6);
  }
  if (linkUp) {
    onModem = WiFi.SSID() == MODEM_WIFI_SSID;
    haveEverConnectedWiFi = true;
    lastConnectedWasModem = onModem;
    offlineScanPasses = 0;
  }

  ModemProbeResult result;
  if (modemProbeResults && xQueueReceive(modemProbeResults, &result, 0) == pdTRUE &&
      probePending && result.token == probeToken && linkUp && onModem) {
    probePending = false;
    if (now - result.finishedAtMs <= PROBE_FRESH_MS) {
      probeKnown = true;
      lastProbeResultMs = result.finishedAtMs;
      probeGood = result.successMask != 0;
      if (probeGood) failedProbeStreak = 0;
      else if (failedProbeStreak < 255) ++failedProbeStreak;
      Serial.printf("MODEM PROBE: reachable=%d mask=%u failed-streak=%u\n",
                    probeGood, result.successMask, failedProbeStreak);
    } else {
      probeKnown = false;
      failedProbeStreak = 0;
      Serial.println("MODEM PROBE: hasil lama diabaikan; tunggu pemeriksaan baru.");
    }
  }
  if (probePending && now - probePendingSinceMs > PROBE_FRESH_MS) {
    // Jangan menafsirkan task yang macet sebagai bukti internet/modem mati.
    probeWorkerFault = true;
    invalidateModemProbe();
    Serial.println("MODEM ERROR: task probe tidak merespons 90 s; auto recovery dihentikan.");
  }
  if (MODEM_AUTO_RECOVERY && !probeWorkerFault && linkUp && onModem && !probePending &&
      now - lastProbeRequestMs >= PROBE_INTERVAL_MS) {
    ModemProbeRequest request = {++probeToken};
    lastProbeRequestMs = now;
    if (xQueueSend(modemProbeRequests, &request, 0) == pdTRUE) {
      probePending = true; probePendingSinceMs = now;
    }
  }

  ModemHealthInput evidence;
  evidence.enabled = MODEM_AUTO_RECOVERY && strlen(MODEM_WIFI_SSID) > 0;
  evidence.workerFault = probeWorkerFault;
  evidence.linkUp = linkUp; evidence.onModem = onModem;
  evidence.everConnected = haveEverConnectedWiFi;
  evidence.lastWasModem = lastConnectedWasModem;
  evidence.offlineScans = offlineScanPasses;
  evidence.ackFresh = sessionHasAck && now - lastPublishAckMs <= ACK_FRESH_MS;
  evidence.probeFresh = probeKnown && now - lastProbeResultMs <= PROBE_FRESH_MS;
  evidence.probeGood = probeGood; evidence.failedProbes = failedProbeStreak;
  ModemHealth health = classifyModemHealth(evidence);
  const char* reason = !linkUp ? "WiFi modem tidak tersambung setelah >=3 scan selesai"
      : "kedua tujuan TCP gagal berulang pada WiFi modem";
  applyModemAction(modemPolicy.tick(now, health), reason);
  saveModemBudgetRTC();
  bool blocked = modemPolicy.budgetBlocked();
  if (blocked && !wasBudgetBlocked)
    logModemEvent("LIMIT", "batas siklus tercapai; tunggu anggaran pulih");
  wasBudgetBlocked = blocked;
}

// Setup/Loop
void setup() {
  modemPowerInit(); // Nyalakan modem sebelum inisialisasi sensor/jaringan.
  Serial.begin(115200);
  delay(200);
  Serial.printf("MODEM: daya awal ON melalui GPIO%u (%s).\n",
                (unsigned)MODEM_POWER_PIN, MODEM_ON_LEVEL == HIGH ? "HIGH" : "LOW");

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  ledBlue(); delay(250); ledOff();

  Wire.begin(SDA_MAIN, SCL_MAIN);
  Wire.setClock(100000);

  lightReady = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  if (!lightReady) Serial.println("BH1750 init failed");
  bmpReady = bmp.begin(0x76);
  if (!bmpReady) Serial.println("BMP280 not detected!");
  inaReady = ina219.begin();
  if (!inaReady)  Serial.println("INA219 not detected!");
  scdReady = scd40.begin();
  if (!scdReady)   Serial.println("SCD40 not detected!");

  uint16_t tmp = 0;
  if (as5600ReadRaw(tmp)) {
    float sensorDeg = ((float)tmp * 360.0f) / 4096.0f;
    currentAS5600Raw = (int)tmp;
    currentAS5600SensorDeg = sensorDeg;
    currentWindDirectionDeg = wrapDeg(cal_VANE(sensorDeg + WIND_DIR_OFFSET_DEG));
    as5600Valid = true;

    Serial.print("AS5600 OK - raw: ");
    Serial.print(tmp);
    Serial.print(", angle: ");
    Serial.print(sensorDeg, 1);
    Serial.println(" deg");
  } else {
    as5600Valid = false;
    Serial.println("AS5600 NOT detected!");
  }

  pinMode(RAIN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainISR, FALLING);

  pinMode(ANEMO_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ANEMO_PIN), anemoISR, FALLING);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  sdReady = SD.begin(SD_CS);
  if (!sdReady) {
    Serial.println("SD init failed! Akan dicoba ulang tanpa menghentikan sistem.");
  } else {
    Serial.println("SD init OK");
    createCSVHeader();
  }

  // NTP akan sinkron otomatis jika WiFi tersedia.
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  snprintf(runtimeClientId, sizeof(runtimeClientId), "%s-%012llX", MQTT_CLIENT_ID,
           (unsigned long long)ESP.getEfuseMac());
  snprintf(bootId, sizeof(bootId), "%012llX-%08lX%08lX",
           (unsigned long long)ESP.getEfuseMac(),
           (unsigned long)esp_random(), (unsigned long)esp_random());
  mqtt.begin(client);
  mqtt.setTimeout(MQTT_TIMEOUT_MS);
  Serial.printf("MQTT client ID: %s\n", runtimeClientId);

  ledBlue();
  modemRecoveryBegin(); // scan dimulai setelah waktu boot modem selesai

  // Buang pulsa selama setup: awal jendela pengukuran dimulai di sini.
  noInterrupts(); rainTicks = 0; pulseCount = 0; interrupts();
  lastCheck = millis();
  sampleSensors();
  integratedAtMs = lastSampleMs = lastCheck = millis();
  noInterrupts(); pulseCount = 0; interrupts();
  startDeliveryLedTask();
  Serial.println("\n=== WEATHER STATION: LIVE + ONE BACKLOG PER MINUTE ===\n");
}

void loop() {
  serviceMeasurements();
  serviceModemRecovery();
  updateStatusLED();
  serviceNetwork();
  serviceModemRecovery();
  updateStatusLED();
  serviceMeasurements();
  flushBacklog();   // live lebih dahulu; satu backlog hanya dengan izin ACK live
  serviceMeasurements();

  // Jika SD belum tersedia, coba mount ulang tiap 30 detik.
  if ((!sdReady || storageFault || csvFault) &&
      (lastSDRetryMs == 0 || millis() - lastSDRetryMs >= SD_RETRY_INTERVAL_MS)) {
    lastSDRetryMs = millis();
    SD.end();
    sdReady = SD.begin(SD_CS);
    if (sdReady) {
      Serial.println("SD remount OK");
      queueInitialized = false;
      createCSVHeader();
    } else {
      Serial.println("SD masih belum tersedia.");
    }
  }
  updateStatusLED();
  static uint32_t lastDiagnosticMs = 0;
  if (millis() - lastDiagnosticMs >= 30000UL) {
    lastDiagnosticMs = millis();
    Serial.printf("NET wifi=%d WSS=%d MQTT=%d LIVE_ACK=%d ageLIVE=%lus backlog=%d RAM=%u SDfault=%d lost=%d heap=%lu\n",
      WiFi.status() == WL_CONNECTED, client.isConnected(), mqtt.isConnected(), liveConfirmedLatest && liveSessionHasAck,
      (unsigned long)((millis() - lastLiveAckMs) / 1000UL), backlogExists, ramCount,
      storageFault || csvFault, ramOverflowFault, (unsigned long)ESP.getFreeHeap());
  }
}
