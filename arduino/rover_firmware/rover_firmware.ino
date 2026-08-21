/*
 * ============================================================
 *  AUTONOMOUS FARM ROVER — Arduino Nano Firmware
 *  Layer: Reflex (real-time sensors + motor control)
 *  Board: Arduino Nano (ATmega328P)
 *  Comms: USB Serial → Raspberry Pi @ 115200 baud  (D0/D1 — do not reuse)
 *
 *  4WD skid-steer, 2x L298N drivers, each motor independently
 *  wired (no jumpers) — software still commands both motors on
 *  a side together, but hardware allows per-wheel control later.
 *
 *  ── Pin map ─────────────────────────────────────────────────
 *  Driver 1 (LEFT)   front-left:  ENA=D3(PWM) IN1=D4  IN2=D13
 *                     rear-left:   ENB=D5(PWM) IN3=D7  IN4=D12
 *  Driver 2 (RIGHT)  front-right: ENA=D6(PWM) IN1=D8  IN2=D2
 *                     rear-right:  ENB=D9(PWM) IN3=D10 IN4=D11
 *  Ultrasonic   TRIG=A0 (shared front+rear)  FRONT_ECHO=A1  REAR_ECHO=A2
 *  DHT22        DATA=A3
 *  Soil moisture probes  SIG1→A6  SIG2→A7  (VCC+GND shared)
 *  BME280       I2C — SDA=A4  SCL=A5 (fixed)
 *  D0/D1 reserved for Pi serial — every other digital pin D2-D13
 *  is used by motors; nothing free there. Add-ons later must
 *  come off the analog pool or share/replace a motor pin.
 *  ────────────────────────────────────────────────────────────
 *
 *  Commands received from Pi (newline-terminated JSON):
 *    {"cmd":"move","dir":"forward","speed":150}
 *    {"cmd":"move","dir":"backward","speed":150}
 *    {"cmd":"move","dir":"left","speed":120}
 *    {"cmd":"move","dir":"right","speed":120}
 *    {"cmd":"stop"}
 *    {"cmd":"sample"}   → force immediate sensor read + send
 *    {"cmd":"estop"}    → emergency stop, lock motors until reset
 *    {"cmd":"reset"}    → clear e-stop lock
 *
 *  Serial Monitor test (Arduino IDE):
 *    Baud 115200, line ending = "Newline" (not "No line ending")
 *    Paste one command and click Send — you must see CMD:received: …
 * ============================================================
 */

#define SERIAL_TEST_ONLY false

#if SERIAL_TEST_ONLY

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(1000);
  Serial.println("BOOT:serial_test_mode");
  Serial.println("BOOT:firmware_started");
  Serial.println("BOOT:serial_ready");
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(800);
  Serial.println("LOOP:heartbeat");
}

#else

#include <Wire.h>
#include <ArduinoJson.h>

// ── Sensor enable flags ───────────────────────────────────────
// Flip to true ONLY after that part is wired, then re-upload.
// Order to bring up: motors → moisture → US front → DHT or BME.
#define HAS_MOTORS       true
#define HAS_US_FRONT     false   // HC-SR04: TRIG=A0  ECHO=A1
#define HAS_US_REAR      false   // HC-SR04: TRIG=A0  ECHO=A2
#define HAS_DHT22        true    // DHT22 data pin A3  (needs DHT library)
#define HAS_MOISTURE     true    // two capacitive probes: A6 + A7
#define HAS_BME280       false    // I2C A4/A5 (needs Adafruit BME280)

// Capacitive soil moisture calibration (raw ADC 0–1023 on Nano 5V)
// Measure in air (dry) and in water (wet), then set these:
#define MOISTURE_DRY_ADC  780
#define MOISTURE_WET_ADC  330

#define LEFT_MOTOR_INVERT   true
#define RIGHT_MOTOR_INVERT  true

// Motors (2x L298N)
#define FL_ENA      3
#define FL_IN1      4
#define FL_IN2      13
#define RL_ENB      5
#define RL_IN3      7
#define RL_IN4      12

#define FR_ENA      6
#define FR_IN1      8
#define FR_IN2      2
#define RR_ENB      9
#define RR_IN3      10
#define RR_IN4      11

#define US_TRIG         A0
#define US_FRONT_ECHO   A1
#define US_REAR_ECHO    A2

#define DHT_PIN     A3
#define DHT_TYPE    DHT22
#define MOISTURE_PIN_1  A6
#define MOISTURE_PIN_2  A7

#define SAMPLE_INTERVAL   2000
#define US_TIMEOUT        30000
#define OBSTACLE_DIST_CM  20
#define SERIAL_BAUD       115200
#define DEBUG_SERIAL      true
#define RX_BUF_SIZE       160

#if HAS_BME280
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;
#endif

#if HAS_DHT22
#include <DHT.h>
DHT dht(DHT_PIN, DHT_TYPE);
#endif

bool  eStop         = false;
bool  bmeAvailable  = false;
int   currentSpeed  = 0;
char  currentDir[12] = "stop";

unsigned long lastSampleTime = 0;

// Non-blocking serial line buffer (avoids String heap + readStringUntil stalls)
char rxBuf[RX_BUF_SIZE];
uint8_t rxLen = 0;

void stopMotors();
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void handleLine(char* line);
void broadcastSensorData(int frontDist);
long readUltrasonic(int trigPin, int echoPin);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(SERIAL_BAUD);
  delay(800);
  Serial.println();
  Serial.println("BOOT:firmware_started");
  Serial.println("BOOT:serial_ready");

#if HAS_MOTORS
  pinMode(FL_ENA, OUTPUT); pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(RL_ENB, OUTPUT); pinMode(RL_IN3, OUTPUT); pinMode(RL_IN4, OUTPUT);
  pinMode(FR_ENA, OUTPUT); pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(RR_ENB, OUTPUT); pinMode(RR_IN3, OUTPUT); pinMode(RR_IN4, OUTPUT);
  stopMotors();
  Serial.println("BOOT:motors_ready");
#endif

#if HAS_US_FRONT || HAS_US_REAR
  pinMode(US_TRIG, OUTPUT);
#endif
#if HAS_US_FRONT
  pinMode(US_FRONT_ECHO, INPUT);
#endif
#if HAS_US_REAR
  pinMode(US_REAR_ECHO,  INPUT);
#endif

#if HAS_DHT22
  dht.begin();
#endif

#if HAS_BME280
  bmeAvailable = bme.begin(0x76);
  if (!bmeAvailable) {
    bmeAvailable = bme.begin(0x77);
  }
  Serial.print("BOOT:bme280=");
  Serial.println(bmeAvailable ? "ok" : "missing");
#endif

  Serial.print(F("{\"status\":\"ready\",\"fw\":\"2.1.0\","));
  Serial.print(F("\"has_motors\":"));    Serial.print(HAS_MOTORS ? F("true") : F("false"));
  Serial.print(F(",\"has_us_front\":")); Serial.print(HAS_US_FRONT ? F("true") : F("false"));
  Serial.print(F(",\"has_us_rear\":"));  Serial.print(HAS_US_REAR ? F("true") : F("false"));
  Serial.print(F(",\"has_dht22\":"));    Serial.print(HAS_DHT22 ? F("true") : F("false"));
  Serial.print(F(",\"has_moisture\":")); Serial.print(HAS_MOISTURE ? F("true") : F("false"));
  Serial.print(F(",\"has_bme280\":"));   Serial.print(bmeAvailable ? F("true") : F("false"));
  Serial.println(F("}"));
  Serial.println(F("BOOT:send JSON cmds with Newline line-ending @ 115200"));
}

void loop() {
  // Drain serial into line buffer; fire handler on each complete line
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;          // ignore CR from "Both NL & CR"
    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0) handleLine(rxBuf);
      rxLen = 0;
    } else if (rxLen < RX_BUF_SIZE - 1) {
      rxBuf[rxLen++] = c;
    } else {
      // overflow — drop line
      rxLen = 0;
      Serial.println(F("{\"error\":\"line_too_long\"}"));
    }
  }

  int frontDist = -1;
#if HAS_US_FRONT
  frontDist = readUltrasonic(US_TRIG, US_FRONT_ECHO);
#if HAS_MOTORS
  if (frontDist > 0 && frontDist < OBSTACLE_DIST_CM && !eStop) {
    stopMotors();
    strcpy(currentDir, "stop");
    currentSpeed = 0;
    Serial.print(F("{\"alert\":\"obstacle\",\"dist_cm\":"));
    Serial.print(frontDist);
    Serial.println(F("}"));
  }
#endif
#endif

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = now;
    broadcastSensorData(frontDist);
  }
}

// ── Motors ───────────────────────────────────────────────────

void setLeftMotors(bool forward, int speed) {
#if HAS_MOTORS
  bool dir = LEFT_MOTOR_INVERT ? !forward : forward;
  digitalWrite(FL_IN1, dir ? HIGH : LOW);
  digitalWrite(FL_IN2, dir ? LOW  : HIGH);
  analogWrite(FL_ENA, speed);

  digitalWrite(RL_IN3, dir ? HIGH : LOW);
  digitalWrite(RL_IN4, dir ? LOW  : HIGH);
  analogWrite(RL_ENB, speed);
#endif
}

void setRightMotors(bool forward, int speed) {
#if HAS_MOTORS
  bool dir = RIGHT_MOTOR_INVERT ? !forward : forward;
  digitalWrite(FR_IN1, dir ? HIGH : LOW);
  digitalWrite(FR_IN2, dir ? LOW  : HIGH);
  analogWrite(FR_ENA, speed);

  digitalWrite(RR_IN3, dir ? HIGH : LOW);
  digitalWrite(RR_IN4, dir ? LOW  : HIGH);
  analogWrite(RR_ENB, speed);
#endif
}

void stopMotors() {
#if HAS_MOTORS
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW); analogWrite(FL_ENA, 0);
  digitalWrite(RL_IN3, LOW); digitalWrite(RL_IN4, LOW); analogWrite(RL_ENB, 0);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW); analogWrite(FR_ENA, 0);
  digitalWrite(RR_IN3, LOW); digitalWrite(RR_IN4, LOW); analogWrite(RR_ENB, 0);
#endif
}

void moveForward(int speed)  { setLeftMotors(true,  speed); setRightMotors(true,  speed); }
void moveBackward(int speed) { setLeftMotors(false, speed); setRightMotors(false, speed); }
void turnLeft(int speed)     { setLeftMotors(false, speed); setRightMotors(true,  speed); }
void turnRight(int speed)    { setLeftMotors(true,  speed); setRightMotors(false, speed); }

// ── Serial command handler ───────────────────────────────────

void handleLine(char* line) {
  // trim leading whitespace
  while (*line == ' ' || *line == '\t') line++;

  Serial.print(F("CMD:received:"));
  Serial.println(line);

  // ArduinoJson 7: JsonDocument (StaticJsonDocument is deprecated)
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    Serial.print(F("{\"error\":\"bad_json\",\"detail\":\""));
    Serial.print(err.c_str());
    Serial.println(F("\"}"));
    return;
  }

  const char* cmd = doc["cmd"];
  if (!cmd) {
    Serial.println(F("{\"error\":\"no_cmd\"}"));
    return;
  }

  if (strcmp(cmd, "estop") == 0) {
    eStop = true;
    stopMotors();
    currentSpeed = 0;
    strcpy(currentDir, "stop");
    Serial.println(F("{\"ack\":\"estop\",\"locked\":true}"));
    return;
  }

  if (strcmp(cmd, "reset") == 0) {
    eStop = false;
    Serial.println(F("{\"ack\":\"reset\",\"locked\":false}"));
    return;
  }

  if (eStop) {
    Serial.println(F("{\"error\":\"estop_active\"}"));
    return;
  }

  if (strcmp(cmd, "stop") == 0) {
    stopMotors();
    strcpy(currentDir, "stop");
    currentSpeed = 0;
    Serial.println(F("{\"ack\":\"stop\"}"));
    return;
  }

  if (strcmp(cmd, "move") == 0) {
    const char* dir = doc["dir"] | "forward";
    int speed = doc["speed"] | 150;
    speed = constrain(speed, 0, 255);

    currentSpeed = speed;
    strncpy(currentDir, dir, sizeof(currentDir) - 1);
    currentDir[sizeof(currentDir) - 1] = '\0';

#if DEBUG_SERIAL
    Serial.print(F("{\"debug\":\"move_exec\",\"dir\":\""));
    Serial.print(dir);
    Serial.print(F("\",\"speed\":"));
    Serial.print(speed);
    Serial.println(F("}"));
#endif

    if      (strcmp(dir, "forward")  == 0) moveForward(speed);
    else if (strcmp(dir, "backward") == 0) moveBackward(speed);
    else if (strcmp(dir, "left")     == 0) turnLeft(speed);
    else if (strcmp(dir, "right")    == 0) turnRight(speed);
    else {
      Serial.println(F("{\"error\":\"unknown_dir\"}"));
      return;
    }

#if HAS_MOTORS
    Serial.print(F("{\"ack\":\"move\",\"dir\":\""));
#else
    Serial.print(F("{\"ack\":\"move\",\"note\":\"motors flag off\",\"dir\":\""));
#endif
    Serial.print(dir);
    Serial.print(F("\",\"speed\":"));
    Serial.print(speed);
    Serial.println(F("}"));
    return;
  }

  if (strcmp(cmd, "sample") == 0) {
    int frontDist = -1;
#if HAS_US_FRONT
    frontDist = readUltrasonic(US_TRIG, US_FRONT_ECHO);
#endif
    broadcastSensorData(frontDist);
    return;
  }

  Serial.println(F("{\"error\":\"unknown_cmd\"}"));
}

// ── Sensors ──────────────────────────────────────────────────

int readMoistureRaw(int pin) {
  // Average a few samples — capacitive probes are noisy
  long sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  return (int)(sum / 5);
}

int readMoisturePct(int pin) {
  // Capacitive: high raw ≈ dry, low raw ≈ wet
  int raw = readMoistureRaw(pin);
  int pct = map(raw, MOISTURE_DRY_ADC, MOISTURE_WET_ADC, 0, 100);
  return constrain(pct, 0, 100);
}

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, US_TIMEOUT);
  if (duration == 0) return -1;   // timeout / not connected
  long cm = duration / 58;
  if (cm < 2 || cm > 400) return -1;
  return cm;
}

void broadcastSensorData(int frontDist) {
  // ArduinoJson 7
  JsonDocument out;

  // Moisture — Pi logger accepts a single int or [m1, m2]
#if HAS_MOISTURE
  {
    int pct1 = readMoisturePct(MOISTURE_PIN_1);
    int pct2 = readMoisturePct(MOISTURE_PIN_2);
    int raw1 = readMoistureRaw(MOISTURE_PIN_1);
    int raw2 = readMoistureRaw(MOISTURE_PIN_2);
    // Primary field: array [probe1, probe2] for Pi logger
    JsonArray marr = out["moisture"].to<JsonArray>();
    marr.add(pct1);
    marr.add(pct2);
    JsonArray rarr = out["moisture_raw"].to<JsonArray>();
    rarr.add(raw1);
    rarr.add(raw2);
    out["moisture_arr"] = marr;  // alias
  }
#else
  out["moisture"] = nullptr;
  out["moisture_raw"] = nullptr;
#endif

  // Prefer BME280 for air; fall back to DHT22 values as air_temp/humidity
  bool airSet = false;
#if HAS_BME280
  if (bmeAvailable) {
    out["air_temp"] = serialized(String(bme.readTemperature(), 1));
    out["humidity"] = serialized(String(bme.readHumidity(), 1));
    out["pressure"] = serialized(String(bme.readPressure() / 100.0F, 1));
    airSet = true;
  }
#endif
#if HAS_DHT22
  {
    float dTemp = dht.readTemperature();
    float dHum  = dht.readHumidity();
    if (!isnan(dTemp)) {
      out["dht_temp"] = serialized(String(dTemp, 1));
      if (!airSet) out["air_temp"] = serialized(String(dTemp, 1));
    } else {
      out["dht_temp"] = nullptr;
    }
    if (!isnan(dHum)) {
      out["dht_humidity"] = serialized(String(dHum, 1));
      if (!airSet) out["humidity"] = serialized(String(dHum, 1));
    } else {
      out["dht_humidity"] = nullptr;
    }
    if (!airSet) out["pressure"] = nullptr;
    airSet = airSet || (!isnan(dTemp) && !isnan(dHum));
  }
#else
  out["dht_temp"]     = nullptr;
  out["dht_humidity"] = nullptr;
#endif
  if (!airSet) {
    out["air_temp"] = nullptr;
    out["humidity"] = nullptr;
    out["pressure"] = nullptr;
  }

#if HAS_US_FRONT
  out["us_front"] = frontDist;   // cm, or -1 if no echo
#else
  out["us_front"] = nullptr;
#endif

#if HAS_US_REAR
  out["us_rear"] = readUltrasonic(US_TRIG, US_REAR_ECHO);
#else
  out["us_rear"] = nullptr;
#endif

  out["estop"] = eStop;
  out["dir"]   = currentDir;
  out["speed"] = currentSpeed;

  serializeJson(out, Serial);
  Serial.println();
}

#endif
