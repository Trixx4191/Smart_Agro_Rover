/*
 * ============================================================
 *  AUTONOMOUS FARM ROVER — Arduino Nano Firmware
 *  Layer: Reflex (real-time sensors + motor control)
 *  Board: Arduino Nano (ATmega328P)
 *  Comms: USB Serial → Raspberry Pi @ 115200 baud
 *
 *  Sensors:
 *    - 2× Capacitive soil moisture (analog A0, A1)
 *    - 2× DS18B20 soil temp (1-Wire, pin 2)
 *    - 1× BME280 temp/humidity/pressure (I2C)
 *    - 2× HC-SR04 ultrasonic (front pin 6/7, rear pin 8/9)
 *    - 4× LM393 wheel encoders (pins 3,11,A2,A3 — INT where available)
 *
 *  Motor driver: L298N Dual H-Bridge
 *    Left motors  → IN1=4, IN2=5, ENA=10 (PWM)
 *    Right motors → IN3=12, IN4=13, ENB=11 is taken by encoder
 *    NOTE: ENB moved to pin A4 (used as digital PWM via analogWrite trick)
 *    *** See pin map below for full details ***
 *
 *  Commands received from Pi (newline-terminated JSON):
 *    {"cmd":"move","dir":"forward","speed":150}
 *    {"cmd":"move","dir":"backward","speed":150}
 *    {"cmd":"move","dir":"left","speed":120}
 *    {"cmd":"move","dir":"right","speed":120}
 *    {"cmd":"stop"}
 *    {"cmd":"sample"}   → force immediate sensor read + send
 *    {"cmd":"estop"}    → emergency stop, lock motors until reset
 *
 *  Output to Pi every SAMPLE_INTERVAL ms (JSON):
 *    {
 *      "moisture":[val1,val2],
 *      "soil_temp":[t1,t2],
 *      "air_temp":23.4,
 *      "humidity":61.2,
 *      "pressure":1013.2,
 *      "us_front":45,
 *      "us_rear":120,
 *      "enc":[lf,lr,rf,rr],
 *      "estop":false
 *    }
 * ============================================================
 */

#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>

// ── Pin Definitions ─────────────────────────────────────────
// Motors (L298N)
#define MOTOR_IN1     4
#define MOTOR_IN2     5
#define MOTOR_ENA     10   // PWM — left motors
#define MOTOR_IN3     12
#define MOTOR_IN4     13
#define MOTOR_ENB     A4   // PWM capable via analogWrite (digital pin 18)

// Soil moisture (analog)
#define MOISTURE_PIN_1  A0
#define MOISTURE_PIN_2  A1

// DS18B20 (1-Wire bus — both sensors share this pin)
#define ONE_WIRE_PIN    2

// Ultrasonics
#define US_FRONT_TRIG   6
#define US_FRONT_ECHO   7
#define US_REAR_TRIG    8
#define US_REAR_ECHO    9

// Wheel encoders (interrupt-capable pins where possible)
// Nano interrupts on pins 2 and 3; pin 2 taken by 1-Wire
// So only left-front encoder gets hardware interrupt (pin 3)
// Others polled (acceptable for low-speed rovers)
#define ENC_LF    3    // INT1 — left front
#define ENC_LR    A2   // polled — left rear
#define ENC_RF    A3   // polled — right front
#define ENC_RR    11   // polled — right rear (moved ENB off this pin)

// ── Timing ──────────────────────────────────────────────────
#define SAMPLE_INTERVAL   2000   // ms between sensor broadcasts
#define US_TIMEOUT        30000  // µs — ~5m max range
#define OBSTACLE_DIST_CM  20     // auto-stop if closer than this (front)
#define SERIAL_BAUD       115200

// ── Sensor Objects ───────────────────────────────────────────
OneWire           oneWire(ONE_WIRE_PIN);
DallasTemperature dallasSensors(&oneWire);
Adafruit_BME280   bme;

// ── State Variables ──────────────────────────────────────────
volatile long encLF = 0;   // encoder counts
long encLR = 0, encRF = 0, encRR = 0;

bool  eStop         = false;
bool  bmeAvailable  = false;
int   currentSpeed  = 0;
char  currentDir[12] = "stop";

unsigned long lastSampleTime = 0;
unsigned long lastEncLR = 0, lastEncRF = 0, lastEncRR = 0;

// ── ISR ─────────────────────────────────────────────────────
void IRAM_ATTR onEncLF() {
  encLF++;
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(SERIAL_BAUD);

  // Motor pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);
  stopMotors();

  // Ultrasonic pins
  pinMode(US_FRONT_TRIG, OUTPUT);
  pinMode(US_FRONT_ECHO, INPUT);
  pinMode(US_REAR_TRIG,  OUTPUT);
  pinMode(US_REAR_ECHO,  INPUT);

  // Encoder pins
  pinMode(ENC_LF, INPUT_PULLUP);
  pinMode(ENC_LR, INPUT_PULLUP);
  pinMode(ENC_RF, INPUT_PULLUP);
  pinMode(ENC_RR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LF), onEncLF, RISING);

  // Dallas temperature
  dallasSensors.begin();

  // BME280 — I2C default address 0x76 (some modules use 0x77)
  bmeAvailable = bme.begin(0x76);
  if (!bmeAvailable) {
    bmeAvailable = bme.begin(0x77);  // try alternate address
  }

  // Ready signal to Pi
  Serial.println("{\"status\":\"ready\",\"fw\":\"1.0.0\"}");
}

// ── Main Loop ────────────────────────────────────────────────
void loop() {
  // 1. Parse any incoming command from Pi
  if (Serial.available()) {
    handleSerialCommand();
  }

  // 2. Obstacle check — always runs, even during movement
  int frontDist = readUltrasonic(US_FRONT_TRIG, US_FRONT_ECHO);
  if (frontDist > 0 && frontDist < OBSTACLE_DIST_CM && !eStop) {
    // Obstacle in front — stop and notify Pi
    stopMotors();
    strcpy(currentDir, "stop");
    Serial.print("{\"alert\":\"obstacle\",\"dist_cm\":");
    Serial.print(frontDist);
    Serial.println("}");
  }

  // 3. Poll non-interrupt encoders
  pollEncoders();

  // 4. Periodic sensor broadcast
  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = now;
    broadcastSensorData(frontDist);
  }
}

// ── Motor Control ────────────────────────────────────────────

void setLeftMotors(bool forward, int speed) {
  digitalWrite(MOTOR_IN1, forward ? HIGH : LOW);
  digitalWrite(MOTOR_IN2, forward ? LOW  : HIGH);
  analogWrite(MOTOR_ENA, speed);
}

void setRightMotors(bool forward, int speed) {
  digitalWrite(MOTOR_IN3, forward ? HIGH : LOW);
  digitalWrite(MOTOR_IN4, forward ? LOW  : HIGH);
  analogWrite(MOTOR_ENB, speed);
}

void stopMotors() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, 0);
  analogWrite(MOTOR_ENB, 0);
}

void moveForward(int speed) {
  setLeftMotors(true,  speed);
  setRightMotors(true, speed);
}

void moveBackward(int speed) {
  setLeftMotors(false,  speed);
  setRightMotors(false, speed);
}

void turnLeft(int speed) {
  // Pivot: left motors backward, right motors forward
  setLeftMotors(false, speed);
  setRightMotors(true, speed);
}

void turnRight(int speed) {
  setLeftMotors(true,  speed);
  setRightMotors(false, speed);
}

// ── Serial Command Parser ────────────────────────────────────

void handleSerialCommand() {
  String raw = Serial.readStringUntil('\n');
  raw.trim();
  if (raw.length() == 0) return;

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, raw);
  if (err) {
    Serial.println("{\"error\":\"bad_json\"}");
    return;
  }

  const char* cmd = doc["cmd"];
  if (!cmd) {
    Serial.println("{\"error\":\"no_cmd\"}");
    return;
  }

  // Emergency stop — locks movement until hardware reset or "reset" cmd
  if (strcmp(cmd, "estop") == 0) {
    eStop = true;
    stopMotors();
    Serial.println("{\"ack\":\"estop\",\"locked\":true}");
    return;
  }

  if (strcmp(cmd, "reset") == 0) {
    eStop = false;
    Serial.println("{\"ack\":\"reset\",\"locked\":false}");
    return;
  }

  // Block movement if e-stop active
  if (eStop) {
    Serial.println("{\"error\":\"estop_active\"}");
    return;
  }

  if (strcmp(cmd, "stop") == 0) {
    stopMotors();
    strcpy(currentDir, "stop");
    currentSpeed = 0;
    Serial.println("{\"ack\":\"stop\"}");
    return;
  }

  if (strcmp(cmd, "move") == 0) {
    const char* dir = doc["dir"] | "forward";
    int speed = doc["speed"] | 150;
    speed = constrain(speed, 0, 255);

    currentSpeed = speed;
    strncpy(currentDir, dir, sizeof(currentDir) - 1);

    if      (strcmp(dir, "forward")  == 0) moveForward(speed);
    else if (strcmp(dir, "backward") == 0) moveBackward(speed);
    else if (strcmp(dir, "left")     == 0) turnLeft(speed);
    else if (strcmp(dir, "right")    == 0) turnRight(speed);
    else {
      Serial.println("{\"error\":\"unknown_dir\"}");
      return;
    }
    Serial.print("{\"ack\":\"move\",\"dir\":\"");
    Serial.print(dir);
    Serial.print("\",\"speed\":");
    Serial.print(speed);
    Serial.println("}");
    return;
  }

  if (strcmp(cmd, "sample") == 0) {
    int frontDist = readUltrasonic(US_FRONT_TRIG, US_FRONT_ECHO);
    broadcastSensorData(frontDist);
    return;
  }

  Serial.println("{\"error\":\"unknown_cmd\"}");
}

// ── Sensor Reading ───────────────────────────────────────────

int readMoisture(int pin) {
  // Capacitive sensor: high raw = dry, low raw = wet
  // Map to 0–100% (0 = dry, 100 = saturated)
  // Calibration values — tune these on your specific sensor + soil type
  const int DRY_VAL = 780;   // raw ADC when sensor is in dry air
  const int WET_VAL = 330;   // raw ADC when sensor is fully submerged
  int raw = analogRead(pin);
  int pct = map(raw, DRY_VAL, WET_VAL, 0, 100);
  return constrain(pct, 0, 100);
}

float readSoilTemp(int index) {
  dallasSensors.requestTemperatures();
  float t = dallasSensors.getTempCByIndex(index);
  // DS18B20 returns -127 on error
  if (t == DEVICE_DISCONNECTED_C) return -999.0;
  return t;
}

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, US_TIMEOUT);
  if (duration == 0) return -1;   // timeout = out of range
  return duration / 58;           // convert µs to cm
}

void pollEncoders() {
  // Simple edge detection for non-interrupt encoder pins
  int lr = digitalRead(ENC_LR);
  int rf = digitalRead(ENC_RF);
  int rr = digitalRead(ENC_RR);

  if (lr == HIGH && lastEncLR == LOW) encLR++;
  if (rf == HIGH && lastEncRF == LOW) encRF++;
  if (rr == HIGH && lastEncRR == LOW) encRR++;

  lastEncLR = lr;
  lastEncRF = rf;
  lastEncRR = rr;
}

void broadcastSensorData(int frontDist) {
  // Soil moisture
  int m1 = readMoisture(MOISTURE_PIN_1);
  int m2 = readMoisture(MOISTURE_PIN_2);

  // Soil temperature
  float t1 = readSoilTemp(0);
  float t2 = readSoilTemp(1);

  // BME280
  float airTemp = -999, humidity = -999, pressure = -999;
  if (bmeAvailable) {
    airTemp  = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;  // Pa → hPa
  }

  // Rear ultrasonic (front already read in loop)
  long rearDist = readUltrasonic(US_REAR_TRIG, US_REAR_ECHO);

  // Build and send JSON
  StaticJsonDocument<256> out;
  JsonArray moisture  = out.createNestedArray("moisture");
  moisture.add(m1); moisture.add(m2);

  JsonArray soilTemp  = out.createNestedArray("soil_temp");
  soilTemp.add(t1);  soilTemp.add(t2);

  out["air_temp"]  = serialized(String(airTemp,  1));
  out["humidity"]  = serialized(String(humidity, 1));
  out["pressure"]  = serialized(String(pressure, 1));
  out["us_front"]  = frontDist;
  out["us_rear"]   = rearDist;

  JsonArray enc = out.createNestedArray("enc");
  noInterrupts();
  long lf = encLF;
  interrupts();
  enc.add(lf); enc.add(encLR); enc.add(encRF); enc.add(encRR);

  out["estop"] = eStop;
  out["dir"]   = currentDir;
  out["speed"] = currentSpeed;

  serializeJson(out, Serial);
  Serial.println();
}
