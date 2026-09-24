/**
 * @file    nursery_controller.ino
 * @brief   ESP32 Automated Micro-Climate Nursery - Final Firmware v2.4
 * @author  Dulanga Basnayake
 * @date    September 2026
 * @module  COMP50069 Hardware, Microcontrollers and Sensors
 * @scenario Scenario 2 - Automated Commercial Micro-Climate Nursery
 *
 * Hardware wiring:
 *   DHT22 DATA      → GPIO 27
 *   LDR (10kΩ div)  → GPIO 34
 *   Servo signal    → GPIO 13 (LEDC PWM 50 Hz)
 *   OLED SDA        → GPIO 21
 *   OLED SCL        → GPIO 22
 *   OLED RES        → GPIO 16
 *   OLED DC         → GND
 *   OLED CS         → 3.3V
 *   LED Power       → GPIO 18
 *   LED Normal      → GPIO 33
 *   LED Temp Alarm  → GPIO 32
 *   LED Vent        → GPIO 19
 *   LED Grow (PWM)  → GPIO 4
 *   LED Humidity    → GPIO 17
 *   Button Override → GPIO 25
 *   Button Display  → GPIO 26
 *
 * Version history:
 *   v2.1 - Added DHT22 fault debouncing (3 consecutive failures required)
 *   v2.2 - Removed ESP32Servo library, drive servo via raw LEDC
 *   v2.3 - Corrected servo pulse mapping (500-2400 us for SG90)
 *   v2.4 - Rolling 30s trend window for predictive pre-venting
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================
#define OLED_SDA          21
#define OLED_SCL          22
#define OLED_RESET        16
#define OLED_ADDR         0x3C

#define DHT_PIN           27
#define DHT_TYPE          DHT22
#define LDR_PIN           34
#define SERVO_PIN         13

#define LED_POWER_PIN      18
#define LED_NORMAL_PIN     33
#define LED_TEMP_ALARM_PIN 32
#define LED_VENT_PIN       19
#define LED_LIGHT_PIN      4
#define LED_HUM_PIN        17

#define BTN_OVERRIDE_PIN   25
#define BTN_DISPLAY_PIN    26

// ============================================================
// SYSTEM CONSTANTS
// ============================================================
const float TEMP_SETPOINT_DEFAULT = 28.0f;
const float HUM_SETPOINT_DEFAULT  = 70.0f;
const float TEMP_VENT_THRESHOLD   = 30.0f;
const float HUM_WARN_THRESHOLD    = 80.0f;
const int   LIGHT_LOW_THRESHOLD   = 30;

const float MAX_VALID_TEMP = 60.0f;
const float MIN_VALID_TEMP = -10.0f;
const float MAX_VALID_HUM  = 100.0f;

const unsigned long SENSOR_INTERVAL  = 2000;
const unsigned long OLED_INTERVAL    = 500;
const unsigned long OVERRIDE_TIMEOUT = 60000UL;
const unsigned long DEBOUNCE_DELAY   = 250;
const unsigned long TREND_WINDOW     = 30000UL;

const float TEMP_WEIGHT  = 2.0f;
const float HUM_WEIGHT   = 0.5f;
const float COLD_PENALTY = 2.0f;
const uint8_t DHT_FAIL_THRESHOLD = 3;

const float SERVO_PULSE_MIN_US = 500.0f;
const float SERVO_PULSE_MAX_US = 2400.0f;
const float SERVO_MAX_ANGLE    = 180.0f;

// ============================================================
// GLOBAL OBJECTS
// ============================================================
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
DHT dht(DHT_PIN, DHT_TYPE);

// ============================================================
// SYSTEM STATE
// ============================================================
enum SystemState {
  STATE_AUTONOMOUS,
  STATE_PRE_VENTING,
  STATE_MANUAL_OVERRIDE,
  STATE_SENSOR_FAULT
};
SystemState currentState = STATE_AUTONOMOUS;

float currentTemp = 0.0f;
float currentHum  = 0.0f;
int   lightPct    = 0;

float tempSetpoint = TEMP_SETPOINT_DEFAULT;
float humSetpoint  = HUM_SETPOINT_DEFAULT;

int  ventAngle    = 0;
int  growLightPWM = 0;
bool externalCold    = false;
bool altDisplayMode  = false;
bool dhtFaultFlag    = false;
bool faultLatched    = false;
String faultMessage  = "";

uint8_t dhtFailCount = 0;

unsigned long lastSensorRead  = 0;
unsigned long lastOLEDUpdate  = 0;
unsigned long lastBtn1        = 0;
unsigned long lastBtn2        = 0;
unsigned long manualStartTime = 0;

float lastTempReading = 0.0f;
unsigned long lastTrendTime = 0;
float tempRatePerMin = 0.0f;

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================
void readSensors();
void evaluateStateMachine();
void computeVentAngle();
void computeGrowLight();
void applyActuators();
void updateStatusLEDs();
void updateOLED();
void handleButtons();
void handleSerial();
void processCommand(String cmd);
void enterManualOverride();
void exitManualOverride();
void enterFaultState(const String &msg);
void clearFault();
void setServoAngle(float angle);
void setGrowLightPWM(int pwm);
const char* stateToString(SystemState s);
void printHelp();
void printStatus();

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("============================================="));
  Serial.println(F(" Automated Micro-Climate Nursery v2.4"));
  Serial.println(F(" Scenario 2 - COMP50069"));
  Serial.println(F("============================================="));
  Serial.println(F("Type HELP for commands."));

  // Status LED pins
  pinMode(LED_POWER_PIN, OUTPUT);
  pinMode(LED_NORMAL_PIN, OUTPUT);
  pinMode(LED_TEMP_ALARM_PIN, OUTPUT);
  pinMode(LED_VENT_PIN, OUTPUT);
  pinMode(LED_HUM_PIN, OUTPUT);

  digitalWrite(LED_POWER_PIN, HIGH);
  digitalWrite(LED_NORMAL_PIN, HIGH);

  // Push buttons
  pinMode(BTN_OVERRIDE_PIN, INPUT_PULLUP);
  pinMode(BTN_DISPLAY_PIN, INPUT_PULLUP);

  // OLED hardware reset pulse
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW); delay(50);
  digitalWrite(OLED_RESET, HIGH); delay(50);

  // I2C + OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("ERROR: OLED init failed"));
    while (true) {}
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 25);
  display.println(F("SYSTEM BOOTING..."));
  display.display();

  // DHT sensor
  dht.begin();

  // Servo PWM via LEDC (50 Hz, 16-bit)
  ledcAttach(SERVO_PIN, 50, 16);
  setServoAngle(0);

  // Grow light PWM via LEDC (5 kHz, 8-bit)
  ledcAttach(LED_LIGHT_PIN, 5000, 8);
  ledcWrite(LED_LIGHT_PIN, 0);

  delay(1500);
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  handleButtons();
  handleSerial();

  // Periodic sensor read & control evaluation
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readSensors();
    evaluateStateMachine();
    computeVentAngle();
    computeGrowLight();
    applyActuators();
    updateStatusLEDs();

    // CSV telemetry stream
    Serial.print(millis());       Serial.print(F(","));
    Serial.print(currentTemp, 1); Serial.print(F(","));
    Serial.print(currentHum, 1);  Serial.print(F(","));
    Serial.print(lightPct);       Serial.print(F(","));
    Serial.print(ventAngle);      Serial.print(F(","));
    Serial.print(stateToString(currentState));
    Serial.print(F(","));
    Serial.println(externalCold ? F("COLD") : F("NORMAL"));
  }

  // OLED refresh
  if (now - lastOLEDUpdate >= OLED_INTERVAL) {
    lastOLEDUpdate = now;
    updateOLED();
  }

  // Manual override auto-timeout
  if (currentState == STATE_MANUAL_OVERRIDE &&
      (now - manualStartTime >= OVERRIDE_TIMEOUT)) {
    exitManualOverride();
  }
}

// ============================================================
// SENSOR READING
// ============================================================
void readSensors() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  bool dhtValid = !isnan(t) && !isnan(h) &&
                  t > MIN_VALID_TEMP && t < MAX_VALID_TEMP &&
                  h >= 0.0f && h <= MAX_VALID_HUM;

  if (dhtValid) {
    currentTemp = t;
    currentHum  = h;
    dhtFailCount = 0;
    dhtFaultFlag = false;
  } else {
    if (dhtFailCount < 255) dhtFailCount++;
    if (dhtFailCount >= DHT_FAIL_THRESHOLD) dhtFaultFlag = true;
  }

  int raw = analogRead(LDR_PIN);
  lightPct = constrain(map(raw, 0, 4095, 0, 100), 0, 100);

  // Rolling temperature trend (30 s reference window)
  unsigned long now = millis();
  if (lastTrendTime == 0) {
    lastTrendTime = now;
    lastTempReading = currentTemp;
  }
  unsigned long elapsed = now - lastTrendTime;
  if (elapsed >= 3000) {
    tempRatePerMin = (currentTemp - lastTempReading) * 60000.0f / (float)elapsed;
  }
  if (elapsed >= TREND_WINDOW) {
    lastTempReading = currentTemp;
    lastTrendTime = now;
  }
}

// ============================================================
// STATE MACHINE
// ============================================================
void evaluateStateMachine() {
  if (dhtFaultFlag) {
    if (!faultLatched) {
      enterFaultState(F("DHT22 SENSOR FAULT"));
      faultLatched = true;
    }
    return;
  }

  if (currentState == STATE_SENSOR_FAULT && !dhtFaultFlag) {
    clearFault();
    return;
  }

  if (currentState == STATE_MANUAL_OVERRIDE) return;

  if (currentState == STATE_AUTONOMOUS && tempRatePerMin > 0.8f) {
    currentState = STATE_PRE_VENTING;
    Serial.println(F("EVENT: PRE_VENTING entered"));
  } else if (currentState == STATE_PRE_VENTING &&
             tempRatePerMin < 0.4f &&
             currentTemp < tempSetpoint + 1.0f) {
    currentState = STATE_AUTONOMOUS;
    Serial.println(F("EVENT: returned to AUTONOMOUS"));
  }
}

// ============================================================
// VENT ANGLE COMPUTATION (weighted design-challenge logic)
// ============================================================
void computeVentAngle() {
  if (currentState == STATE_SENSOR_FAULT)    { ventAngle = 0;  return; }
  if (currentState == STATE_MANUAL_OVERRIDE) { ventAngle = 90; return; }

  float tempError = currentTemp - tempSetpoint;
  float humError  = currentHum  - humSetpoint;
  float weighted  = TEMP_WEIGHT * tempError + HUM_WEIGHT * humError;

  if (externalCold) {
    weighted -= COLD_PENALTY * max(0.0f, currentTemp - 15.0f);
  }

  float mapped = (weighted + 10.0f) * (90.0f / 20.0f);
  ventAngle = constrain((int)mapped, 0, 90);
}

// ============================================================
// GROW LIGHT PWM
// ============================================================
void computeGrowLight() {
  if (currentState == STATE_SENSOR_FAULT) { growLightPWM = 128; return; }
  if (lightPct < LIGHT_LOW_THRESHOLD) {
    growLightPWM = map(lightPct, 0, LIGHT_LOW_THRESHOLD, 255, 0);
  } else {
    growLightPWM = 0;
  }
  growLightPWM = constrain(growLightPWM, 0, 255);
}

// ============================================================
// ACTUATORS
// ============================================================
void applyActuators() {
  setServoAngle((float)ventAngle);
  setGrowLightPWM(growLightPWM);
}

/**
 * @brief  Sets the servo to a given angle via LEDC PWM.
 * @param  angle Desired angle in degrees (0-180). System clamps 0-90.
 * @note   SG90 calibration: 500 us pulse = 0°, 2400 us = 180°.
 */
void setServoAngle(float angle) {
  angle = constrain(angle, 0.0f, SERVO_MAX_ANGLE);
  float pulse = SERVO_PULSE_MIN_US +
                (angle / SERVO_MAX_ANGLE) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
  uint32_t duty = (uint32_t)((pulse / 20000.0f) * 65535.0f);
  ledcWrite(SERVO_PIN, duty);
}

void setGrowLightPWM(int pwm) {
  pwm = constrain(pwm, 0, 255);
  ledcWrite(LED_LIGHT_PIN, pwm);
}

// ============================================================
// STATUS LEDS
// ============================================================
void updateStatusLEDs() {
  digitalWrite(LED_NORMAL_PIN,
    (currentState == STATE_AUTONOMOUS || currentState == STATE_PRE_VENTING) ? HIGH : LOW);

  if (currentState == STATE_SENSOR_FAULT) {
    static unsigned long lastBlink = 0;
    static bool blinkState = false;
    if (millis() - lastBlink >= 400) {
      lastBlink = millis();
      blinkState = !blinkState;
      digitalWrite(LED_TEMP_ALARM_PIN, blinkState ? HIGH : LOW);
      digitalWrite(LED_HUM_PIN, blinkState ? HIGH : LOW);
    }
  } else {
    digitalWrite(LED_TEMP_ALARM_PIN, (currentTemp >= TEMP_VENT_THRESHOLD) ? HIGH : LOW);
    digitalWrite(LED_HUM_PIN, (currentHum >= HUM_WARN_THRESHOLD) ? HIGH : LOW);
  }

  bool ventActive = (ventAngle > 5) || (currentState == STATE_MANUAL_OVERRIDE);
  digitalWrite(LED_VENT_PIN, ventActive ? HIGH : LOW);
}

// ============================================================
// BUTTONS
// ============================================================
void handleButtons() {
  unsigned long now = millis();

  if (digitalRead(BTN_OVERRIDE_PIN) == LOW && (now - lastBtn1) > DEBOUNCE_DELAY) {
    lastBtn1 = now;
    if (currentState == STATE_MANUAL_OVERRIDE) {
      exitManualOverride();
    } else if (currentState != STATE_SENSOR_FAULT) {
      enterManualOverride();
    } else {
      Serial.println(F("Cannot override during fault."));
    }
  }

  if (digitalRead(BTN_DISPLAY_PIN) == LOW && (now - lastBtn2) > DEBOUNCE_DELAY) {
    lastBtn2 = now;
    altDisplayMode = !altDisplayMode;
    updateOLED();
  }
}

// ============================================================
// MANUAL OVERRIDE
// ============================================================
void enterManualOverride() {
  currentState = STATE_MANUAL_OVERRIDE;
  manualStartTime = millis();
  Serial.println(F("EVENT: Manual override ON (60s timeout)"));
}

void exitManualOverride() {
  currentState = STATE_AUTONOMOUS;
  Serial.println(F("EVENT: Manual override OFF"));
}

// ============================================================
// FAULT HANDLING
// ============================================================
void enterFaultState(const String &msg) {
  currentState = STATE_SENSOR_FAULT;
  faultMessage = msg;
  ventAngle = 0;
  growLightPWM = 128;
  applyActuators();
  Serial.print(F("FAULT: ")); Serial.println(msg);
}

void clearFault() {
  dhtFaultFlag = false;
  faultLatched = false;
  dhtFailCount = 0;
  faultMessage = "";
  currentState = STATE_AUTONOMOUS;
  Serial.println(F("EVENT: fault cleared - AUTONOMOUS"));
}

// ============================================================
// SERIAL COMMAND HANDLING
// ============================================================
void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() > 0) processCommand(cmd);
}

void processCommand(String cmd) {
  if (cmd == "HELP")        { printHelp(); }
  else if (cmd == "STATUS") { printStatus(); }
  else if (cmd.startsWith("SET TEMP ")) {
    float v = cmd.substring(9).toFloat();
    if (v >= 10 && v <= 40) { tempSetpoint = v;
      Serial.print(F("Setpoint TEMP = ")); Serial.println(tempSetpoint); }
    else Serial.println(F("Invalid temp (10-40)"));
  }
  else if (cmd.startsWith("SET HUM ")) {
    float v = cmd.substring(8).toFloat();
    if (v >= 20 && v <= 95) { humSetpoint = v;
      Serial.print(F("Setpoint HUM = ")); Serial.println(humSetpoint); }
    else Serial.println(F("Invalid humidity (20-95)"));
  }
  else if (cmd == "MAN ON")   { if (currentState != STATE_SENSOR_FAULT) enterManualOverride();
                                 else Serial.println(F("Cannot override during fault.")); }
  else if (cmd == "MAN OFF")  { if (currentState == STATE_MANUAL_OVERRIDE) exitManualOverride(); }
  else if (cmd == "SIM COLD") {
    externalCold = !externalCold;
    Serial.print(F("External cold = ")); Serial.println(externalCold ? F("ON") : F("OFF"));
  }
  else if (cmd == "RESET")    { if (currentState == STATE_SENSOR_FAULT) clearFault();
                                 else Serial.println(F("No fault to reset")); }
  else Serial.println(F("Unknown command. Type HELP."));
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  STATUS        - print full system status"));
  Serial.println(F("  SET TEMP <x>  - set temp setpoint 10-40"));
  Serial.println(F("  SET HUM <x>   - set humidity setpoint 20-95"));
  Serial.println(F("  MAN ON|OFF    - manual override on/off"));
  Serial.println(F("  SIM COLD      - toggle external cold condition"));
  Serial.println(F("  RESET         - clear sensor fault"));
  Serial.println(F("  HELP          - this list"));
}

void printStatus() {
  Serial.println(F("----- SYSTEM STATUS -----"));
  Serial.print(F("State       : ")); Serial.println(stateToString(currentState));
  Serial.print(F("Temperature : ")); Serial.print(currentTemp); Serial.println(F(" C"));
  Serial.print(F("Humidity    : ")); Serial.print(currentHum);  Serial.println(F(" %"));
  Serial.print(F("Light       : ")); Serial.print(lightPct);    Serial.println(F(" %"));
  Serial.print(F("Vent Angle  : ")); Serial.print(ventAngle);   Serial.println(F(" deg"));
  Serial.print(F("Grow Light  : ")); Serial.print(growLightPWM); Serial.println(F("/255"));
  Serial.print(F("Setpoints   : ")); Serial.print(tempSetpoint);
  Serial.print(F(" C / ")); Serial.print(humSetpoint); Serial.println(F(" %"));
  Serial.print(F("Ext Cold    : ")); Serial.println(externalCold ? F("YES") : F("NO"));
  Serial.print(F("Trend       : ")); Serial.print(tempRatePerMin); Serial.println(F(" C/min"));
  Serial.print(F("Fault       : ")); Serial.println(faultMessage.length() ? faultMessage : F("none"));
  Serial.println(F("-------------------------"));
}

// ============================================================
// OLED RENDERING
// ============================================================
void updateOLED() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  if (!altDisplayMode) {
    display.setCursor(0, 0);  display.println(F("== ENVIRONMENT =="));
    display.setCursor(0, 20); display.print(F("Temp:  ")); display.print(currentTemp, 1); display.println(F(" C"));
    display.setCursor(0, 35); display.print(F("Hum:   ")); display.print(currentHum, 1);  display.println(F(" %"));
    display.setCursor(0, 50); display.print(F("Light: ")); display.print(lightPct);      display.println(F(" %"));
  } else {
    display.setCursor(0, 0);  display.println(F("== SYSTEM PANEL =="));
    display.setCursor(0, 14); display.print(F("State: ")); display.println(stateToString(currentState));
    display.setCursor(0, 26); display.print(F("Vent:  ")); display.print(ventAngle);     display.println(F(" deg"));
    display.setCursor(0, 38); display.print(F("Grow:  ")); display.print(growLightPWM);  display.println(F("/255"));
    display.setCursor(0, 50); display.print(F("ExtC:  ")); display.println(externalCold ? F("ON") : F("OFF"));
  }

  if (currentState == STATE_SENSOR_FAULT) {
    display.setCursor(0, 52);
    display.print(F("FAULT: "));
    display.print(faultMessage.substring(0, 12));
  }
  display.display();
}

// ============================================================
// UTILITIES
// ============================================================
const char* stateToString(SystemState s) {
  switch (s) {
    case STATE_AUTONOMOUS:      return "AUTO";
    case STATE_PRE_VENTING:     return "PREVENT";
    case STATE_MANUAL_OVERRIDE: return "MANUAL";
    case STATE_SENSOR_FAULT:    return "FAULT";
  }
  return "?";
}