#include <WiFi.h>
#include <WebSocketsClient.h>
#include <MQTTPubSubClient.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include <Wire.h>

#include <BH1750.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_INA219.h>
#include "SparkFun_SCD4x_Arduino_Library.h"

#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <time.h>

// WiFi
const char* ssid = "-";
const char* pass = "-";

// MQTT WSS
const char* MQTT_HOST = "-";
const int   MQTT_PORT = 443;
const char* MQTT_PATH = "-";
const char* MQTT_CLIENT_ID = "-";
const char* MQTT_TOPIC = "-";

// Interval pembacaan sensor, penyimpanan SD, dan publish MQTT: 60 detik
static const uint32_t MEASUREMENT_INTERVAL_MS = 60000UL;

WebSocketsClient client;
MQTTPubSubClient mqtt;

// Sensors
BH1750 lightMeter;
Adafruit_BMP280 bmp;
Adafruit_INA219 ina219;
SCD4x scd40;

// Calibration report convention:
// sensor_reading = slope * reference_value + intercept
// Therefore, the correction used in this code is:
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
static const uint8_t DP_POWER      = 1;

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

const uint8_t FAIL_PUBLISH_LIMIT = 3;
uint8_t failPublishCount = 0;

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

// AS5600 dibaca pada interval yang sama dengan sensor lain, yaitu 60 detik.
int currentWindDirectionDeg = 0;
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
  currentWindDirectionDeg = wrapDeg(correctedDeg);
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
  if (millis() - lastCheck < MEASUREMENT_INTERVAL_MS) return lastWindSpeed;

  uint32_t now = millis();
  uint32_t elapsedMs = now - lastCheck;
  if (lastCheck == 0 || elapsedMs == 0) elapsedMs = MEASUREMENT_INTERVAL_MS;

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
  if (SD.exists("/data.csv")) return;
  File f = SD.open("/data.csv", FILE_WRITE);
  if (!f) return;

  f.println(
    "timestamp,"
    "altitude,co2,arah_angin,curah_hujan,current,intensitas_cahaya,kecepatan_angin,kelembapan,ketinggian_air_laut,power,tekanan_udara,temperature,voltage,"
    "rain_rate_mmh,rain_interval_mm,total_rain_mm"
  );
  f.close();
}

void appendCSV(
  const String& timestamp,
  float altitude, int co2, int arah_angin_deg,
  float curah_hujan, float current, float intensitas_cahaya,
  float kecepatan_angin, float kelembapan, float ketinggian_air_laut,
  float power, float tekanan_udara, float temperature, float voltage,
  float rain_rate_mmh, float rain_interval_mm, float total_rain_mm
) {
  File f = SD.open("/data.csv", FILE_APPEND);
  if (!f) { Serial.println("SD write error!"); return; }

  f.print(timestamp); f.print(",");
  f.print(altitude, DP_ALTITUDE); f.print(",");
  f.print(co2); f.print(",");
  f.print(arah_angin_deg); f.print(",");
  f.print(curah_hujan, DP_RAIN); f.print(",");
  f.print(current, DP_CURRENT); f.print(",");
  f.print(intensitas_cahaya, DP_LUX); f.print(",");
  f.print(kecepatan_angin, DP_WIND_SPEED); f.print(",");
  f.print(kelembapan, DP_RH); f.print(",");
  f.print(ketinggian_air_laut, DP_WATER); f.print(",");
  f.print(power, DP_POWER); f.print(",");
  f.print(tekanan_udara, DP_PRESSURE); f.print(",");
  f.print(temperature, DP_TEMP); f.print(",");
  f.print(voltage, DP_VOLTAGE); f.print(",");
  f.print(rain_rate_mmh, DP_RAIN); f.print(",");
  f.print(rain_interval_mm, DP_RAIN); f.print(",");
  f.println(total_rain_mm, DP_RAIN);
  f.close();
}

// WiFi/MQTT
// NON-BLOCKING: kegagalan WiFi/MQTT tidak boleh menghentikan logging SD.
static const uint32_t WIFI_RETRY_INTERVAL_MS = 10000UL;
static const uint32_t MQTT_RETRY_INTERVAL_MS = 10000UL;
uint32_t lastWiFiAttemptMs = 0;
uint32_t lastMQTTAttemptMs = 0;
bool mqttTransportStarted = false;

void startWiFiAttempt() {
  Serial.println("WiFi: mencoba koneksi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  lastWiFiAttemptMs = millis();
}

void startMQTTTransport() {
  client.disconnect();
  client.beginSSL(MQTT_HOST, MQTT_PORT, MQTT_PATH, "", "mqtt");
  client.setReconnectInterval(2000);
  mqttTransportStarted = true;
}

void serviceNetwork() {
  uint32_t now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    ledBlueBlink(now, 500);
    if (lastWiFiAttemptMs == 0 || now - lastWiFiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
      startWiFiAttempt();
    }
    return;
  }

  if (!mqttTransportStarted) {
    startMQTTTransport();
  }

  client.loop();
  mqtt.update();

  if (!mqtt.isConnected()) {
    if (lastMQTTAttemptMs == 0 || now - lastMQTTAttemptMs >= MQTT_RETRY_INTERVAL_MS) {
      lastMQTTAttemptMs = now;
      Serial.println("MQTT: mencoba koneksi sekali...");

      if (mqtt.connect(MQTT_CLIENT_ID)) {
        Serial.println("MQTT OK");
        failPublishCount = 0;
        ledGreen();
      } else {
        Serial.println("MQTT offline; logging SD tetap berjalan.");
        ledRed();
      }
    }
  } else if (failPublishCount < FAIL_PUBLISH_LIMIT) {
    ledGreen();
  }
}

bool sendFullPayload(
  int arah_angin_deg, float kecepatan_angin, float altitude,
  float curah_hujan, float intensitas_cahaya, float tekanan_udara,
  float temperature, float kelembapan, int co2,
  float ketinggian_air_laut, float voltage, float current, float power
) {
  StaticJsonDocument<768> doc;
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
  data["power"] = power;

  char payload[768];
  size_t n = serializeJson(doc, payload, sizeof(payload));

  Serial.print("SEND FULL: ");
  Serial.println(payload);

  if (!mqtt.isConnected() || n == 0) {
    Serial.println("MQTT offline -> skip publish. SD tidak terpengaruh.");
    return false;
  }

  bool ok = mqtt.publish(MQTT_TOPIC, payload);
  if (ok) {
    failPublishCount = 0;
    ledGreen();
  } else {
    failPublishCount++;
    if (failPublishCount >= FAIL_PUBLISH_LIMIT) ledRed();
  }
  return ok;
}

// Setup/Loop
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  ledBlue(); delay(250); ledOff();

  Wire.begin(SDA_MAIN, SCL_MAIN);
  Wire.setClock(100000);

  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) Serial.println("BH1750 init failed");
  if (!bmp.begin(0x76)) Serial.println("BMP280 not detected!");
  if (!ina219.begin())  Serial.println("INA219 not detected!");
  if (!scd40.begin())   Serial.println("SCD40 not detected!");

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

  mqtt.begin(client);

  ledBlue();
  startWiFiAttempt(); // tidak menunggu koneksi berhasil

  Serial.println("\n=== WEATHER STATION (SD OFFLINE-SAFE) ===\n");
}

void loop() {
  serviceNetwork();

  // Jika SD belum tersedia, coba mount ulang tiap 30 detik.
  if (!sdReady && (lastSDRetryMs == 0 || millis() - lastSDRetryMs >= SD_RETRY_INTERVAL_MS)) {
    lastSDRetryMs = millis();
    sdReady = SD.begin(SD_CS);
    if (sdReady) {
      Serial.println("SD remount OK");
      createCSVHeader();
    } else {
      Serial.println("SD masih belum tersedia.");
    }
  }

  const uint32_t intervalMs = MEASUREMENT_INTERVAL_MS;
  static uint32_t last = 0;

  static int   co2_val = 0;
  static float temp_val = 0;
  static float hum_val = 0;

  if (millis() - last < intervalMs) return;
  last = millis();

  String timestamp = getTimeString();

  // AS5600 dibaca sekali pada setiap siklus pengukuran 60 detik,
  // sama dengan sensor cuaca lainnya. Jika pembacaan gagal,
  // nilai arah terakhir yang valid tetap digunakan.
  updateWindDirectionAS5600();
  int arah_deg = currentWindDirectionDeg;

  float kecepatan = roundToDp(readWindSpeed(), DP_WIND_SPEED);

  float rainIntervalMm_raw = 0, rainRateMmH_raw = 0, totalRainMm_raw = 0;
  readRainMetrics(intervalMs, rainIntervalMm_raw, rainRateMmH_raw, totalRainMm_raw);
  (void)rainIntervalMm_raw;
  (void)rainRateMmH_raw;

  // Apply the affine rainfall correction to the accumulated amount only once,
  // then obtain the interval amount from the difference. This prevents the
  // regression intercept from being added repeatedly every 60 seconds.
  static float previousTotalRainMmCal = 0.0f;
  float totalRainMmCal = clamp0(cal_RAIN(totalRainMm_raw));
  float rainIntervalMmCal = clamp0(totalRainMmCal - previousTotalRainMmCal);
  previousTotalRainMmCal = totalRainMmCal;
  float rainRateMmHCal = rainIntervalMmCal * (3600000.0f / (float)intervalMs);

  float rainIntervalMm = roundToDp(rainIntervalMmCal, DP_RAIN);
  float totalRainMm    = roundToDp(totalRainMmCal, DP_RAIN);
  float rainRateMmH    = roundToDp(rainRateMmHCal, DP_RAIN);

  float lux_raw = lightMeter.readLightLevel();
  if (lux_raw < 0) lux_raw = 0;
  float lux = roundToDp(clamp0(cal_LUX(lux_raw)), DP_LUX);

  float tekanan_raw = bmp.readPressure() / 100.0f;
  float altitude_raw = bmp.readAltitude(1013.25f);
  float tekanan = roundToDp(clamp0(cal_PRES(tekanan_raw)), DP_PRESSURE);
  float altitude = roundToDp(cal_ALT(altitude_raw), DP_ALTITUDE);

  float distance_cm = readUltrasonicDistanceCm();
  float water_level_cm = roundToDp(
    computeWaterLevelFromDistance(distance_cm), DP_WATER
  );

  if (scd40.readMeasurement()) {
    float co2_raw  = (float)scd40.getCO2();
    float temp_raw = scd40.getTemperature();
    float hum_raw  = scd40.getHumidity();

    co2_val  = (int)lroundf(clamp0(cal_CO2(co2_raw)));
    temp_val = roundToDp(cal_TEMP(temp_raw), DP_TEMP);
    hum_val  = roundToDp(
      clampRange(cal_RH(hum_raw), 0.0f, 100.0f), DP_RH
    );
  }

  float busV = ina219.getBusVoltage_V();
  float shuntV = ina219.getShuntVoltage_mV() / 1000.0f;
  float voltage = roundToDp(busV + shuntV, DP_VOLTAGE);
  float current = roundToDp(ina219.getCurrent_mA(), DP_CURRENT);
  float power   = roundToDp(ina219.getPower_mW(), DP_POWER);

  Serial.println("--------------------------------------------------");
  Serial.print("Time: "); Serial.println(timestamp);
  Serial.print("WindDir: "); Serial.println(arah_deg);
  Serial.print("AS5600 status: "); Serial.println(as5600Valid ? "OK" : "READ ERROR / LAST VALUE USED");
  Serial.print("AS5600 raw: "); Serial.println(currentAS5600Raw);
  Serial.print("AS5600 sensor angle: "); Serial.println(currentAS5600SensorDeg, 1);
  Serial.print("WindSpeed(m/s): "); Serial.println(kecepatan, DP_WIND_SPEED);
  Serial.print("RainRate(mm/h): "); Serial.println(rainRateMmH, DP_RAIN);
  Serial.print("Lux: "); Serial.println(lux, DP_LUX);
  Serial.print("Pressure(hPa): "); Serial.println(tekanan, DP_PRESSURE);
  Serial.print("Altitude(m): "); Serial.println(altitude, DP_ALTITUDE);
  Serial.print("WaterLevel(cm): "); Serial.println(water_level_cm, DP_WATER);
  Serial.print("CO2(ppm): "); Serial.println(co2_val);
  Serial.print("Temp(C): "); Serial.println(temp_val, DP_TEMP);
  Serial.print("RH(%): "); Serial.println(hum_val, DP_RH);
  Serial.print("Voltage(V): "); Serial.println(voltage, DP_VOLTAGE);
  Serial.print("Current(mA): "); Serial.println(current, DP_CURRENT);
  Serial.print("Power(mW): "); Serial.println(power, DP_POWER);
  Serial.print("Fail publish: "); Serial.println(failPublishCount);
  Serial.println("--------------------------------------------------\n");

  // MQTT opsional. Bila offline, publish dilewati tanpa mengganggu SD.
  sendFullPayload(
    arah_deg, kecepatan, altitude,
    rainIntervalMm, lux, tekanan,
    temp_val, hum_val, co2_val,
    water_level_cm, voltage, current, power
  );

  // SD selalu diprioritaskan setiap siklus 60 detik.
  if (sdReady && SD.cardType() != CARD_NONE) {
    appendCSV(
      timestamp,
      altitude,
      co2_val,
      arah_deg,
      rainIntervalMm,
      current,
      lux,
      kecepatan,
      hum_val,
      water_level_cm,
      power,
      tekanan,
      temp_val,
      voltage,
      rainRateMmH,
      rainIntervalMm,
      totalRainMm
    );
    Serial.println("SD: data tersimpan.");
  } else {
    sdReady = false;
    Serial.println("SD: tidak tersedia, data siklus ini tidak tersimpan.");
  }
}