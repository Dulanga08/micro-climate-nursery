/**
 * @file    main.cpp
 * @brief   ESP32 Automated Micro-Climate Nursery - Wokwi Simulation
 * @author  Dulanga Basnayake
 * @date    September 2026
 * @scenario Scenario 2 - COMP50069
 *
 * Firmware mirrors physical prototype:
 *   DHT22 → GPIO 27    LDR → GPIO 34    Servo → GPIO 13
 *   OLED I2C → SDA 21 / SCL 22
 *   LEDs → 18 (PWR), 33 (NORM), 32 (TEMP), 19 (VENT), 4 (GROW), 17 (HUM)
 *   Buttons → 25 (Override), 26 (Display)
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <ESP32Servo.h>

// --- Pin Definitions ---
#define OLED_SDA          21
#define OLED_SCL          22
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

// --- Thresholds & Setpoints ---
const float TEMP_SETPOINT_DEFAULT = 28.0f;
const float HUM_SETPOINT_DEFAULT  = 70.0f;
const float TEMP_VENT_THRESHOLD   = 30.0f;
const float HUM_WARN_THRESHOLD    = 80.0f;
const int   LIGHT_LOW_THRESHOLD   = 30;

const float MAX_VALID_TEMP = 60.0f;
const float MIN_VALID_TEMP = -10.0f;
const float MAX_VALID_HUM  = 100.0f;

// --- Timing Constants ---
const unsigned long SENSOR_INTERVAL  = 2000;
const unsigned long OLED_INTERVAL    = 500;
const unsigned long OVERRIDE_TIMEOUT = 60000UL;
const unsigned long DEBOUNCE_DELAY   = 250;
const unsigned long TREND_WINDOW     = 30000UL;

// --- Control Weights ---
const float TEMP_WEIGHT  = 2.0f;
const float HUM_WEIGHT   = 0.5f;
const float COLD_PENALTY = 2.0f;
const uint8_t DHT_FAIL_THRESHOLD = 3;
const float SERVO_MAX_ANGLE = 180.0f;

// --- Global Hardware Objects ---
Adafruit_SSD1306 display(128, 64, &Wire, -1);
DHT dht(DHT_PIN, DHT_TYPE);
Servo ventServo;

// --- System State Machine ---
enum SystemState {
  STATE_AUTONOMOUS,
  STATE_PRE_VENTING,
  STATE_MANUAL_OVERRIDE,
  STATE_SENSOR_FAULT
};
SystemState currentState = STATE_AUTONOMOUS;

// --- Global Variables ---
float currentTemp = 0.0f, currentHum = 0.0f;
int   lightPct = 0;
float tempSetpoint = TEMP_SETPOINT_DEFAULT;
float humSetpoint  = HUM_SETPOINT_DEFAULT;
int   ventAngle = 0, growLightPWM = 0;
bool  externalCold = false, altDisplayMode = false;
bool  dhtFaultFlag = false, faultLatched = false;
String faultMessage = "";
uint8_t dhtFailCount = 0;

unsigned long lastSensorRead = 0, lastOLEDUpdate = 0;
unsigned long lastBtn1 = 0, lastBtn2 = 0, manualStartTime = 0;
float lastTempReading = 0.0f;
unsigned long lastTrendTime = 0;
float tempRatePerMin = 0.0f;

// --- Function Declarations ---
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

// ==========================================
// Setup
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("============================================="));
  Serial.println(F(" Automated Micro-Climate Nursery v2.5 (Wokwi)"));
  Serial.println(F(" Scenario 2 - COMP50069"));
  Serial.println(F("============================================="));
  Serial.println(F("Type HELP for commands."));

  // Status LEDs
  pinMode(LED_POWER_PIN, OUTPUT);
  pinMode(LED_NORMAL_PIN, OUTPUT);
  pinMode(LED_TEMP_ALARM_PIN, OUTPUT);
  pinMode(LED_VENT_PIN, OUTPUT);
  pinMode(LED_HUM_PIN, OUTPUT);
  pinMode(LED_LIGHT_PIN, OUTPUT);

  digitalWrite(LED_POWER_PIN, HIGH);
  digitalWrite(LED_NORMAL_PIN, HIGH);

  // Pushbuttons
  pinMode(BTN_OVERRIDE_PIN, INPUT_PULLUP);
  pinMode(BTN_DISPLAY_PIN, INPUT_PULLUP);

  // OLED Setup
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("ERROR: OLED initialization failed"));
    while (true) {}
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 25);
  display.println(F("SYSTEM BOOTING..."));
  display.display();

  // DHT Sensor Setup
  dht.begin();

  // Servo Setup (Standard ESP32Servo attachment)
  ventServo.setPeriodHertz(50);             // Standard 50Hz servo
  ventServo.attach(SERVO_PIN, 500, 2400);   // Pulse range 500us - 2400us
  setServoAngle(0);

  // Grow Light Initial Off State
  setGrowLightPWM(0);

  delay(1500);
}

// ==========================================
// Main Loop
// ==========================================
void loop() {
  unsigned long now = millis();

  handleButtons();
  handleSerial();

  // Periodic Sensor Read & Control Evaluation
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readSensors();
    evaluateStateMachine();
    computeVentAngle();
    computeGrowLight();
    applyActuators();
    updateStatusLEDs();

    // Telemetry output via Serial
    Serial.print(millis());               Serial.print(F(","));
    Serial.print(currentTemp, 1);        Serial.print(F(","));
    Serial.print(currentHum, 1);         Serial.print(F(","));
    Serial.print(lightPct);              Serial.print(F(","));
    Serial.print(ventAngle);             Serial.print(F(","));
    Serial.print(stateToString(currentState)); Serial.print(F(","));
    Serial.println(externalCold ? F("COLD") : F("NORMAL"));
  }

  // OLED Refresh Loop
  if (now - lastOLEDUpdate >= OLED_INTERVAL) {
    lastOLEDUpdate = now;
    updateOLED();
  }

  // Manual Override Timeout Check
  if (currentState == STATE_MANUAL_OVERRIDE &&
      (now - manualStartTime >= OVERRIDE_TIMEOUT)) {
    exitManualOverride();
  }
}

// ==========================================
// Logic & Hardware Functions
// ==========================================
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

void computeVentAngle() {
  if (currentState == STATE_SENSOR_FAULT)      { ventAngle = 0;  return; }
  if (currentState == STATE_MANUAL_OVERRIDE)   { ventAngle = 90; return; }

  float tempError = currentTemp - tempSetpoint;
  float humError  = currentHum  - humSetpoint;
  float weighted  = TEMP_WEIGHT * tempError + HUM_WEIGHT * humError;

  if (externalCold) {
    weighted -= COLD_PENALTY * max(0.0f, currentTemp - 15.0f);
  }

  float mapped = (weighted + 10.0f) * (90.0f / 20.0f);
  ventAngle = constrain((int)mapped, 0, 90);
}

void computeGrowLight() {
  if (currentState == STATE_SENSOR_FAULT) { growLightPWM = 128; return; }
  if (lightPct < LIGHT_LOW_THRESHOLD) {
    growLightPWM = map(lightPct, 0, LIGHT_LOW_THRESHOLD, 255, 0);
  } else {
    growLightPWM = 0;
  }
  growLightPWM = constrain(growLightPWM, 0, 255);
}

void applyActuators() {
  setServoAngle((float)ventAngle);
  setGrowLightPWM(growLightPWM);
}

void setServoAngle(float angle) {
  angle = constrain(angle, 0.0f, SERVO_MAX_ANGLE);
  ventServo.write((int)angle);
}

void setGrowLightPWM(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(LED_LIGHT_PIN, pwm);
}

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

void enterManualOverride() {
  currentState = STATE_MANUAL_OVERRIDE;
  manualStartTime = millis();
  Serial.println(F("EVENT: Manual override ON (60s timeout)"));
}

void exitManualOverride() {
  currentState = STATE_AUTONOMOUS;
  Serial.println(F("EVENT: Manual override OFF"));
}

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

void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() > 0) processCommand(cmd);
}

void processCommand(String cmd) {
  if (cmd == "HELP")          { printHelp(); }
  else if (cmd == "STATUS")   { printStatus(); }
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

const char* stateToString(SystemState s) {
  switch (s) {
    case STATE_AUTONOMOUS:      return "AUTO";
    case STATE_PRE_VENTING:     return "PREVENT";
    case STATE_MANUAL_OVERRIDE: return "MANUAL";
    case STATE_SENSOR_FAULT:    return "FAULT";
  }
  return "?";
}