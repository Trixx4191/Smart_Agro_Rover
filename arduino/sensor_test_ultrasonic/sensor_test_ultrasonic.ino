/*
 * ============================================================
 *  SENSOR TEST — Ultrasonic HC-SR04 (front + rear)
 *  Board: Arduino Nano
 *  Baud:  9600
 *
 *  Wiring (matches rover firmware)
 *  ───────────────────────────────
 *  Shared TRIG → A0
 *  Front ECHO  → A1
 *  Rear  ECHO  → A2
 *  VCC → 5V    GND → GND
 *
 *  HC-SR04 is 5V. ECHO is 5V — Nano pins are 5V-tolerant, OK.
 *  Optional: only wire front first (TRIG+A1), leave rear unplugged.
 *
 *  Serial Monitor @ 9600
 * ============================================================
 */

#define US_TRIG        A0
#define US_FRONT_ECHO  A1
#define US_REAR_ECHO   A2
#define US_TIMEOUT_US  30000UL   // ~5 m max

void setup() {
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_FRONT_ECHO, INPUT);
  pinMode(US_REAR_ECHO, INPUT);
  digitalWrite(US_TRIG, LOW);

  Serial.begin(9600);
  delay(500);
  Serial.println(F("========================================"));
  Serial.println(F("  ULTRASONIC TEST — HC-SR04"));
  Serial.println(F("  TRIG=A0  FRONT_ECHO=A1  REAR_ECHO=A2"));
  Serial.println(F("========================================"));
  Serial.println(F("Point front sensor at a wall ~20–100 cm away."));
  Serial.println();
}

long readCm(int echoPin) {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, US_TIMEOUT_US);
  if (duration == 0) return -1;          // no echo / not connected

  long cm = duration / 58;
  if (cm < 2 || cm > 400) return -1;     // out of reliable range
  return cm;
}

void loop() {
  long front = readCm(US_FRONT_ECHO);
  delay(60);   // brief gap before second ping (shared TRIG)
  long rear  = readCm(US_REAR_ECHO);

  Serial.print(F("FRONT: "));
  if (front < 0) Serial.print(F("no echo"));
  else {
    Serial.print(front);
    Serial.print(F(" cm"));
  }

  Serial.print(F("   |   REAR: "));
  if (rear < 0) Serial.println(F("no echo"));
  else {
    Serial.print(rear);
    Serial.println(F(" cm"));
  }

  delay(400);
}
