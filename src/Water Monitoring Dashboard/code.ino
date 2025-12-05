#include "BlynkEdgent.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// -------------------- Pins --------------------
#define TURBIDITY_PIN    34
#define DS18B20_PIN      4
#define WATER_LEVEL_PIN  15

#define RELAY_PELTIER    25
#define RELAY_PUMP       26
#define RELAY_FAN        27

// -------------------- Thresholds --------------------
float OPTIMAL_TEMP_MAX = 28.0;   // From Blynk V6
float OPTIMAL_TEMP_MIN = 24.0;   // From Blynk V5
float TEMP_HYSTERESIS  = 1.0;

// -------------------- DS18B20 Setup --------------------
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

// -------------------- State --------------------
bool coolingActive = false;
bool manualDrainActive = false;

// Last sent values
int lastTemp = -999;
int lastTurbidity = -999;
int lastWaterLevel = -1;
String lastTurbidityLabel = "";

// Minimum change thresholds
const int TEMP_THRESHOLD = 1;
const int TURBIDITY_THRESHOLD = 10;

// -------------------- Helper Functions --------------------
int readTurbidityNTU() {
  int rawValue = analogRead(TURBIDITY_PIN);
  float ntu = map(rawValue, 1595, 0, 0, 3000);
  ntu = constrain(ntu, 0, 3000);
  return (int)(ntu / 10.0);
}

String getTurbidityLabel(int ntu) {
  float ntuScaled = ntu * 10;
  float haziness = (ntuScaled / 3000.0) * 100.0;
  float clarity = 100.0 - haziness;

  if (clarity > 80) return "Clear";
  else if (clarity > 50) return "Slightly Cloudy";
  else if (clarity > 20) return "Cloudy";
  else return "Very Hazy";
}

int readWaterTemp() {
  sensors.requestTemperatures();
  float temp = sensors.getTempCByIndex(0);
  if (temp == DEVICE_DISCONNECTED_C) return -99;
  return (int)temp;
}

int readWaterLevel() {
  return digitalRead(WATER_LEVEL_PIN) == HIGH ? 0 : 1; // 1 OK, 0 LOW
}

// Turn all cooling actuators ON or OFF
void controlCoolingSystem(bool on) {
  digitalWrite(RELAY_PELTIER, on ? LOW : HIGH);
  digitalWrite(RELAY_PUMP,    on ? LOW : HIGH);
  digitalWrite(RELAY_FAN,     on ? LOW : HIGH);
  coolingActive = on;
}

// -------------------- Blynk Timer --------------------
BlynkTimer timer;

// -------------------- Send Sensor Data --------------------
void sendSensorData() {
  int temp = readWaterTemp();
  int turbidity = readTurbidityNTU();
  int waterLevel = readWaterLevel();

  if (abs(temp - lastTemp) >= TEMP_THRESHOLD) {
    Blynk.virtualWrite(V0, temp);
    lastTemp = temp;
  }

  if (abs(turbidity - lastTurbidity) >= TURBIDITY_THRESHOLD) {
    Blynk.virtualWrite(V1, turbidity);
    lastTurbidity = turbidity;
  }

  String turbidityLabel = getTurbidityLabel(turbidity);
  if (turbidityLabel != lastTurbidityLabel) {
    Blynk.virtualWrite(V4, turbidityLabel);
    lastTurbidityLabel = turbidityLabel;
  }

  if (waterLevel != lastWaterLevel) {
    Blynk.virtualWrite(V2, waterLevel);
    Blynk.virtualWrite(V3, waterLevel ? "OK" : "LOW");
    lastWaterLevel = waterLevel;
  }

  Serial.print("Temp: "); Serial.print(temp);
  Serial.print(" | Turbidity: "); Serial.print(turbidity);
  Serial.print(" | Level: "); Serial.println(waterLevel ? "OK" : "LOW");
}

// -------------------- Actuator Logic --------------------
void handleActuators() {

  // -------- Manual Drain Mode --------
  if (manualDrainActive) {
    digitalWrite(RELAY_PUMP, LOW);
    digitalWrite(RELAY_PELTIER, HIGH);
    digitalWrite(RELAY_FAN, HIGH);

    Serial.println("Manual Drain ACTIVE — Pump ON ONLY");
    return;
  }

  // -------- Cooling Logic with REAL min/max + hysteresis --------
  int temp = readWaterTemp();

  static bool coolingState = false;   // sticky state with hysteresis

  // Hysteresis thresholds
  float upperOn  = OPTIMAL_TEMP_MAX;             // turn ON when above max
  float upperOff = OPTIMAL_TEMP_MAX - TEMP_HYSTERESIS; // turn OFF only after below max-hyst

  float lowerOff = OPTIMAL_TEMP_MIN;             // turn OFF when below min
  float lowerOn  = OPTIMAL_TEMP_MIN + TEMP_HYSTERESIS; // turn ON only after above min+hyst

  // --- Determine cooling state using hysteresis ---
  if (!coolingState) {
    // Cooling is currently OFF
    // Turn ON if too warm (above MAX)
    if (temp >= upperOn) {
      coolingState = true;
      Serial.println("Cooling ACTIVATED (temp >= MAX)");
    }
  } 
  else {
    // Cooling is currently ON
    // Turn OFF when water cools sufficiently
    if (temp <= lowerOff) {
      coolingState = false;
      Serial.println("Cooling DEACTIVATED (temp <= MIN)");
    }
  }

  // --- Apply final cooling output ---
  controlCoolingSystem(coolingState);

  Serial.print("Cooling State: ");
  Serial.println(coolingState ? "ON" : "OFF");
}


// -------------------- Blynk Handlers --------------------

// MAX TEMP (V6)
BLYNK_WRITE(V6) {
  int val = param.asInt();
  OPTIMAL_TEMP_MAX = val;
  Serial.print("Updated OPTIMAL_TEMP_MAX to: "); Serial.println(OPTIMAL_TEMP_MAX);
}

// MIN TEMP (V5)
BLYNK_WRITE(V5) {
  int val = param.asInt();
  OPTIMAL_TEMP_MIN = val;
  Serial.print("Updated OPTIMAL_TEMP_MIN to: "); Serial.println(OPTIMAL_TEMP_MIN);
}

// Manual Drain Button (V7)
BLYNK_WRITE(V7) {
  int state = param.asInt();
  manualDrainActive = (state == 1);

  if (manualDrainActive) {
    Serial.println("Manual Drain: ON (Pump only)");
    digitalWrite(RELAY_PUMP, LOW);
    digitalWrite(RELAY_PELTIER, HIGH);
    digitalWrite(RELAY_FAN, HIGH);
  } else {
    Serial.println("Manual Drain: OFF");
    digitalWrite(RELAY_PUMP, HIGH);
  }
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  BlynkEdgent.begin();

  analogReadResolution(12);

  pinMode(WATER_LEVEL_PIN, INPUT);
  pinMode(TURBIDITY_PIN, INPUT);

  pinMode(RELAY_PELTIER, OUTPUT);
  pinMode(RELAY_PUMP, OUTPUT);
  pinMode(RELAY_FAN, OUTPUT);

  digitalWrite(RELAY_PELTIER, HIGH);
  digitalWrite(RELAY_PUMP, HIGH);
  digitalWrite(RELAY_FAN, HIGH);

  sensors.begin();

  timer.setInterval(500, sendSensorData);     // Faster, safe for Blynk
  timer.setInterval(1000L, handleActuators);  // Actuator logic

  Serial.println("Setup complete");
}

// -------------------- Loop --------------------
void loop() {
  BlynkEdgent.run();
  timer.run();
}
