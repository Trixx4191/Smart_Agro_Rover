/*
 * ============================================================
 *  SENSOR TEST — BME280 + DHT22 only
 *  Board: Arduino Nano
 *  Baud:  115200
 *
 *  Wiring
 *  ──────
 *  DHT22:  DATA → A3   VCC → 5V or 3.3V   GND → GND
 *          (add 4.7k–10k DATA→VCC if module has no pull-up)
 *  BME280: SDA → A4   SCL → A5   VCC → 3.3V   GND → GND
 *          (use 3.3V unless your module is marked 5V-safe)
 *
 *  Libraries (Arduino IDE → Manage Libraries)
 *  ─────────────────────────────────────────
 *  - DHT sensor library          (Adafruit)
 *  - Adafruit Unified Sensor
 *  - Adafruit BME280
 *
 *  Open Serial Monitor @ 115200. Readings every 2 seconds.
 * ============================================================
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DHT.h>

#define DHT_PIN   A3
#define DHT_TYPE  DHT22

DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_BME280 bme;

bool bmeOk = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("  SENSOR TEST — BME280 + DHT22"));
  Serial.println(F("========================================"));

  // ── DHT22 ────────────────────────────────────────────
  dht.begin();
  Serial.println(F("[DHT22] begin() called on pin A3"));

  // ── BME280 (try 0x76 then 0x77) ──────────────────────
  Wire.begin();
  bmeOk = bme.begin(0x76);
  if (!bmeOk) {
    Serial.println(F("[BME280] not at 0x76 — trying 0x77"));
    bmeOk = bme.begin(0x77);
  }
  if (bmeOk) {
    Serial.println(F("[BME280] OK"));
  } else {
    Serial.println(F("[BME280] NOT FOUND — check wiring / 3.3V / I2C"));
    Serial.println(F("         SDA=A4  SCL=A5  VCC=3.3V  GND=GND"));
  }

  // Scan I2C bus so you can see what addresses respond
  Serial.println(F("[I2C] Scanning..."));
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  device at 0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println(F("  (no I2C devices — BME280 not detected)"));
  }

  Serial.println(F("----------------------------------------"));
  Serial.println(F("Reading every 2s..."));
  Serial.println();
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);

  // ── DHT22 ────────────────────────────────────────────
  float dhtT = dht.readTemperature();
  float dhtH = dht.readHumidity();

  Serial.println(F("--- DHT22 (A3) ---"));
  if (isnan(dhtT) || isnan(dhtH)) {
    Serial.println(F("  FAIL — check DATA→A3, power, pull-up"));
  } else {
    Serial.print(F("  temp     = ")); Serial.print(dhtT, 1); Serial.println(F(" C"));
    Serial.print(F("  humidity = ")); Serial.print(dhtH, 1); Serial.println(F(" %"));
  }

  // ── BME280 ───────────────────────────────────────────
  Serial.println(F("--- BME280 (I2C A4/A5) ---"));
  if (!bmeOk) {
    Serial.println(F("  SKIP — not found at boot"));
  } else {
    float t = bme.readTemperature();
    float h = bme.readHumidity();
    float p = bme.readPressure() / 100.0F;
    Serial.print(F("  temp     = ")); Serial.print(t, 1); Serial.println(F(" C"));
    Serial.print(F("  humidity = ")); Serial.print(h, 1); Serial.println(F(" %"));
    Serial.print(F("  pressure = ")); Serial.print(p, 1); Serial.println(F(" hPa"));
  }

  Serial.println();
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);
}
