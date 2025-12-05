#include "BlynkEdgent.h"
#include <DHT.h>
#include <Ticker.h>

// =====================
// Default configuration
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

#define MQ135_PIN 34

#define RELAY_LIGHT_PIN 25
#define RELAY_FOGGER_PIN 26
#define FAN_INTAKE_PIN 27
#define FAN_EXHAUST_PIN 14

#define LIGHT_DEFAULT_HOURS 6
#define FOGGER_DEFAULT_SECONDS 5   // default fogger ON time in seconds

// =====================
// Default thresholds (used until user updates from Blynk)
int minTemp = 20;
int maxTemp = 30;
int minHumid = 65;
int maxHumid = 76;
int minCO2 = 400;
int maxCO2 = 1000;

// Hard safety limits (never overridden)
#define TEMP_DANGER_LOW     10
#define TEMP_DANGER_HIGH    40
#define HUM_DANGER_LOW      30
#define HUM_DANGER_HIGH     90
#define CO2_DANGER_LOW      100
#define CO2_DANGER_HIGH     2000

// =====================
// States and timers
bool lightState = false;
bool foggerState = false;
bool foggerActive = false;

// Scheduler control
bool schedulerEnabled = false;      // Default OFF after power-up
bool lastSchedulerState = false;    // Keeps last known ON/OFF state during Wi-Fi loss

unsigned long userLightOnHours   = LIGHT_DEFAULT_HOURS;
unsigned long userFoggerSeconds  = FOGGER_DEFAULT_SECONDS;
unsigned long nextLightOffSec = 0;

Ticker lightTicker;
Ticker foggerOffTicker;
BlynkTimer timer;

// =====================
// Relay control
void setLightState(bool state) {
  lightState = state;
  digitalWrite(RELAY_LIGHT_PIN, state ? LOW : HIGH);
  Serial.printf("Light set %s\n", state ? "ON" : "OFF");
}

void setFoggerState(bool state) {
  foggerState = state;
  digitalWrite(RELAY_FOGGER_PIN, state ? LOW : HIGH);
  Serial.printf("Fogger set %s\n", state ? "ON" : "OFF");
}

// =====================
// Grow light + Fogger scheduling
void scheduleFoggerNow() {
  if (!lightState) return; // fogger only during light ON
  if (!foggerActive) {
    setFoggerState(true);
    foggerActive = true;
    foggerOffTicker.once(userFoggerSeconds, []() {
      setFoggerState(false);
      foggerActive = false;
    });
    Serial.printf("Fogger ON for %lu seconds\n", userFoggerSeconds);
  }
}

void scheduleGrowLight() {
  unsigned long onSec  = userLightOnHours * 3600;
  unsigned long offSec = (12 - userLightOnHours) * 3600;

  nextLightOffSec = offSec;

  setLightState(true);          // Light ON
  scheduleFoggerNow();          // Fogger at start
  lightTicker.once(onSec / 2, scheduleFoggerNow); // Fogger halfway
  lightTicker.once(onSec, []() {                 // Turn light OFF
    setLightState(false);
    lightTicker.once(nextLightOffSec, scheduleGrowLight); // Repeat cycle
  });
}

// =====================
// Fans control
const int HYST = 2;  // hysteresis margin

void controlFans(int temp, int hum, int co2) {
  static bool intakeFanState = false;
  static bool exhaustFanState = false;

  // --- Danger override ---
  bool dangerTemp = temp <= TEMP_DANGER_LOW || temp >= TEMP_DANGER_HIGH;
  bool dangerHum  = hum  <= HUM_DANGER_LOW  || hum  >= HUM_DANGER_HIGH;
  bool dangerCO2  = co2  <= CO2_DANGER_LOW  || co2  >= CO2_DANGER_HIGH;

  if (dangerTemp || dangerHum || dangerCO2) {
    // Full emergency venting
    intakeFanState = false;
    exhaustFanState = true;

    digitalWrite(FAN_INTAKE_PIN,  intakeFanState ? LOW : HIGH);
    digitalWrite(FAN_EXHAUST_PIN, exhaustFanState ? LOW : HIGH);
    return;
  }

  // ==================================================================
  // NORMAL MIN/MAX + HYSTERESIS LOGIC
  // ==================================================================

  // --- State flags for high/low (sticky with hysteresis) ---
  static bool tempHigh = false, tempLow = false;
  static bool humHigh = false,  humLow  = false;
  static bool co2High = false,  co2Low  = false;

  // Temperature states
  if (temp >= maxTemp) tempHigh = true;
  else if (temp <= maxTemp - HYST) tempHigh = false;

  if (temp <= minTemp) tempLow = true;
  else if (temp >= minTemp + HYST) tempLow = false;

  // Humidity states
  if (hum >= maxHumid) humHigh = true;
  else if (hum <= maxHumid - HYST) humHigh = false;

  if (hum <= minHumid) humLow = true;
  else if (hum >= minHumid + HYST) humLow = false;

  // CO2 states
  if (co2 >= maxCO2) co2High = true;
  else if (co2 <= maxCO2 - HYST) co2High = false;

  if (co2 <= minCO2) co2Low = true;
  else if (co2 >= minCO2 + HYST) co2Low = false;

  // ==================================================================
  // APPLY YOUR TABLE’S FAN BEHAVIOR
  // ==================================================================

  bool wantIntake  = false;
  bool wantExhaust = false;

  // --- Temperature ---
  if (tempHigh) {
    // High temp → INTAKE + EXHAUST
    wantIntake = true;
    wantExhaust = true;
  }
  else if (tempLow) {
    // Low temp → intake only
    wantIntake = true;
    // exhaust stays OFF
  }

  // --- Humidity ---
  if (humHigh) {
    // High humidity → exhaust only
    wantExhaust = true;
    // intake stays off unless tempLow/tempHigh already enabled it
  }
  else if (humLow) {
    // Low humidity → keep everything off (conserve moisture)
    wantExhaust = false;
    // intake only if tempLow or co2Low says so
  }

  // --- CO2 ---
  if (co2High) {
    // High CO2 → exhaust stale air
    wantExhaust = true;
    // intake only if temp or co2Low already requested it
  }
  else if (co2Low) {
    // Low CO2 → intake fresh air
    wantIntake = true;
  }

  // ==================================================================
  // FINAL OUTPUTS
  // ==================================================================

  intakeFanState  = wantIntake;
  exhaustFanState = wantExhaust;

  digitalWrite(FAN_INTAKE_PIN,  intakeFanState ? LOW : HIGH);
  digitalWrite(FAN_EXHAUST_PIN, exhaustFanState ? LOW : HIGH);
}


// =====================
// Sensor task
void readAndSendSensors() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  int mq135_raw = analogRead(MQ135_PIN);
  float co2 = map(mq135_raw, 0, 4095, 400, 2000);

  if (!isnan(temperature) && !isnan(humidity)) {
    // Convert to integers to reduce micro changes
    int tempInt = (int)round(temperature);
    int humInt = (int)round(humidity);
    int co2Int = (int)round(co2);

    Serial.printf("%d ", tempInt);
    Serial.printf("%d ", humInt);
    Serial.printf("%d\n", co2Int);

    controlFans(tempInt, humInt, co2Int);

    // Static last-sent values
    static int lastTemp = -999;
    static int lastHum = -999;
    static int lastCO2 = -999;

    // Only send if changed
    if (Blynk.connected()) {
      if (tempInt != lastTemp) {
        Blynk.virtualWrite(V1, tempInt);
        lastTemp = tempInt;
        Serial.printf("Temp changed: %d\n", tempInt);
      }
      if (humInt != lastHum) {
        Blynk.virtualWrite(V2, humInt);
        lastHum = humInt;
        Serial.printf("Hum changed: %d\n", humInt);
      }
      if (co2Int != lastCO2) {
        Blynk.virtualWrite(V3, co2Int);
        lastCO2 = co2Int;
        Serial.printf("CO2 changed: %d\n", co2Int);
      }
    }
  } else {
    int tempInt = (int)round(temperature);
    int humInt = (int)round(humidity);
    int co2Int = (int)round(co2);

    Serial.printf("%d ", tempInt);
    Serial.printf("%d ", humInt);
    Serial.printf("%d\n", co2Int);
  }
}

// =====================
// Scheduler management
void enableScheduler(bool enable) {
  schedulerEnabled = enable;
  lastSchedulerState = enable;  // remember last known state (for reconnect)

  lightTicker.detach();
  foggerOffTicker.detach();

  if (enable) {
    scheduleGrowLight();
    Serial.println("🌞 Scheduler ENABLED");
  } else {
    setLightState(false);
    setFoggerState(false);
    Serial.println("🌚 Scheduler DISABLED");
  }

  // Update Blynk switch if connected
  if (Blynk.connected()) {
    Blynk.virtualWrite(V0, schedulerEnabled);
  }
}

// =====================
// Blynk handlers
BLYNK_WRITE(V0) { // Scheduler switch
  bool newState = param.asInt();
  enableScheduler(newState);
}

// Threshold handlers
BLYNK_WRITE(V4) { minTemp = param.asInt(); Serial.printf("Min Temp set: %d\n", minTemp); }
BLYNK_WRITE(V5) { maxTemp = param.asInt(); Serial.printf("Max Temp set: %d\n", maxTemp); }
BLYNK_WRITE(V6) { minHumid = param.asInt(); Serial.printf("Min Humid set: %d\n", minHumid); }
BLYNK_WRITE(V9){ maxHumid = param.asInt(); Serial.printf("Max Humid set: %d\n", maxHumid); }
BLYNK_WRITE(V10){ minCO2 = param.asInt(); Serial.printf("Min CO2 set: %d\n", minCO2); }
BLYNK_WRITE(V11){ maxCO2 = param.asInt(); Serial.printf("Max CO2 set: %d\n", maxCO2); }

// Light and fogger user controls (you can assign their Vpins)
BLYNK_WRITE(V7) {
  if (!schedulerEnabled) {
    int hours = param.asInt();
    if (hours > 0 && hours < 12) userLightOnHours = hours;
    else Blynk.virtualWrite(V6, userLightOnHours);
  } else Blynk.virtualWrite(V6, userLightOnHours);
}

BLYNK_WRITE(V8) {
  if (!schedulerEnabled) {
    int seconds = param.asInt();
    if (seconds >= 5 && seconds <= 180) userFoggerSeconds = seconds;
    else Blynk.virtualWrite(V7, userFoggerSeconds);
  } else Blynk.virtualWrite(V7, userFoggerSeconds);
}

// =====================
// Handle reconnects
BLYNK_CONNECTED() {
  Serial.println("✅ Blynk reconnected!");

  // Resync thresholds and last scheduler state
  Blynk.syncVirtual(V4, V5, V9, V10, V11, V12);

  // Restore scheduler state
  if (lastSchedulerState) enableScheduler(true);
  else enableScheduler(false);
}

// =====================
void setup() {
  Serial.begin(115200);
  delay(100);

  BlynkEdgent.begin();
  dht.begin();
  analogReadResolution(12);

  pinMode(RELAY_FOGGER_PIN, OUTPUT);
  pinMode(RELAY_LIGHT_PIN, OUTPUT);
  pinMode(FAN_INTAKE_PIN, OUTPUT);
  pinMode(FAN_EXHAUST_PIN, OUTPUT);

  digitalWrite(RELAY_FOGGER_PIN, HIGH);
  digitalWrite(RELAY_LIGHT_PIN, HIGH);
  digitalWrite(FAN_INTAKE_PIN, HIGH);
  digitalWrite(FAN_EXHAUST_PIN, HIGH);

  WiFi.mode(WIFI_STA);

  // Sensors every 1s
  timer.setInterval(1000L, readAndSendSensors);

  Serial.println("✅ Setup complete. Scheduler OFF by default until user enables it.");
}

void loop() {
  BlynkEdgent.run();
  timer.run();
}
