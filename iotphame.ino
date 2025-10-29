// ===== Smart Farm (DHT22 + BH1750 + Soil HD-38) =====                   // Project title (for readability)
#define BLYNK_TEMPLATE_ID   ""            // Blynk Template ID (keep the variable; fill locally, not in repo)
#define BLYNK_TEMPLATE_NAME "Smart Farm"  // Human-readable name (not secret; ok in repo)
#define BLYNK_AUTH_TOKEN    ""            // Blynk device token (keep variable; fill locally, not in repo)

#include <WiFi.h>                         // ESP32 Wi-Fi stack
#include <BlynkSimpleEsp32.h>             // Blynk client for ESP32
#include <DHTesp.h>                       // DHT22 helper library
#include <Wire.h>                         // I2C master (used by BH1750)
#include <BH1750.h>                       // BH1750 light sensor library

// ===== WiFi (keep variables, remove secrets) =====
char wifiSsid[]     = "";                 // Your Wi-Fi SSID (set locally)
char wifiPassword[] = "";                 // Your Wi-Fi password (set locally)

// ===== Pins =====
#define PIN_DHT_SENSOR          4         // GPIO for DHT22 data pin
#define PIN_MIST_RELAY         27         // Relay for mist (Active-Low → LOW = ON)
#define PIN_FAN_RELAY          14         // Relay for fan  (Active-Low → LOW = ON)
#define PIN_CURTAIN_OPEN_RELAY 26         // Relay to drive curtain OPEN
#define PIN_CURTAIN_CLOSE_RELAY 25        // Relay to drive curtain CLOSE
#define PIN_WATER_RELAY        12         // Relay for water valve (Active-Low)
#define PIN_SOIL_ANALOG        34         // ADC input for soil sensor (HD-38 AO)

// ===== I2C Pins for BH1750 =====
#define I2C_SDA_PIN            21         // I2C SDA pin
#define I2C_SCL_PIN            22         // I2C SCL pin

// ===== Virtual Pins (Blynk) =====
#define VPIN_TEMP_SET        V1           // Slider: target temperature (°C)
#define VPIN_HUMIDITY_SET    V2           // Slider: target humidity (%RH)
#define VPIN_TEMPERATURE     V41          // Gauge/Chart: current temperature
#define VPIN_HUMIDITY        V42          // Gauge/Chart: current humidity
#define VPIN_MANUAL_MIST     V20          // Button: manual mist (works when Auto OFF)
#define VPIN_MANUAL_FAN      V21          // Button: manual fan  (works when Auto OFF)
#define VPIN_STATUS          V30          // Label: status line
#define VPIN_AUTO_MODE       V31          // Switch: Auto/Manual (optional UI)
#define VPIN_LUX             V40          // Gauge/Chart: current lux
#define VPIN_MANUAL_OPEN     V10          // Button: curtain OPEN (optional UI)
#define VPIN_MANUAL_CLOSE    V11          // Button: curtain CLOSE (optional UI)
#define VPIN_MANUAL_WATER    V22          // Button: manual water (works when Auto OFF)
#define VPIN_SOIL_PERCENT    V43          // Chart: soil moisture %
#define VPIN_LUX_SETPOINT    V0           // Slider: lux setpoint (0–50000)

// ===== Sensor drivers =====
DHTesp dhtSensor;                         // DHT22 driver instance
BH1750 bh1750LightSensor;                 // BH1750 driver instance

// ===== Measurements (live values) =====
float airTemperatureCelsius = 0.0f;       // °C
float airRelativeHumidity   = 0.0f;       // %RH
float ambientLightLux       = 0.0f;       // lux

// ===== Fan/Mist base settings =====
float targetTemperatureCelsius = 28.0f;   // Reference temperature (not used by mist band below)
float targetHumidityPercent     = 60.0f;  // Fan tries to keep around this humidity
float temperatureHysteresis     = 1.0f;   // Reserved (not used in current mist logic)
float humidityHysteresis        = 5.0f;   // Fan ON above target+5, OFF below target-5

// ===== Mist thresholds (temperature-based) =====
const float mistTurnOnTemperatureCelsius  = 30.0f; // Mist ON at/above 30°C
const float mistTurnOffTemperatureCelsius = 25.0f; // Mist OFF at/below 25°C

// ===== Light thresholds (for curtain control) =====
const float luxGaugeMaxCap = 50000.0f;    // Cap lux for UI gauge
float targetLuxSetpoint    = 20000.0f;    // Desired light level
float luxDeadband          = 5000.0f;     // Open when below (setpoint - deadband)

// ===== Soil moisture calibration & rule =====
int   soilAdcDryValue      = 3200;        // ADC reading when probe is DRY (calibrate!)
int   soilAdcWetValue      = 1200;        // ADC reading when probe is WET  (calibrate!)
const float waterOnBelowSoilPercent = 60.0f; // Water ON if soil% < 60

float soilMoisturePercent  = 0.0f;        // Calculated soil moisture 0–100%

// ===== States =====
bool autoModeEnabled = true;              // Auto mode flag (Manual when false)
bool mistRelayState  = false;             // Current mist relay state
bool fanRelayState   = false;             // Current fan relay state
bool waterRelayState = false;             // Current water relay state

// ===== Curtain state machine =====
enum CurtainState { CURTAIN_STOPPED, CURTAIN_OPENING, CURTAIN_CLOSING }; // Motion states
CurtainState   curtainState         = CURTAIN_STOPPED; // Current state
unsigned long  curtainStopAtMillis  = 0;               // When to auto-stop motion
const unsigned long curtainMoveDurationMillis = 20000; // Move for 20 seconds then stop

// ===== Timers =====
unsigned long     lastSensorReadMillis      = 0;       // Last sensor read time
const unsigned long sensorReadIntervalMillis = 3000;   // Read every 3 seconds

// ===== Helpers: Relay control (Active-Low hardware) =====
void setMistRelay(bool turnOn) {                        // Set mist relay
  mistRelayState = turnOn;                              // Remember state
  digitalWrite(PIN_MIST_RELAY, turnOn ? LOW : HIGH);    // Drive relay (LOW = ON)
  Blynk.virtualWrite(VPIN_MANUAL_MIST, turnOn ? 1 : 0); // Reflect to UI
  Serial.println(String("Mist: ") + (turnOn ? "ON" : "OFF")); // Log
}

void setFanRelay(bool turnOn) {                         // Set fan relay
  fanRelayState = turnOn;                               // Remember state
  digitalWrite(PIN_FAN_RELAY, turnOn ? LOW : HIGH);     // Drive relay (LOW = ON)
  Blynk.virtualWrite(VPIN_MANUAL_FAN, turnOn ? 1 : 0);  // Reflect to UI
  Serial.println(String("Fan: ") + (turnOn ? "ON" : "OFF")); // Log
}

void setWaterRelay(bool turnOn) {                       // Set water relay
  waterRelayState = turnOn;                             // Remember state
  digitalWrite(PIN_WATER_RELAY, turnOn ? LOW : HIGH);   // Drive relay (LOW = ON)
  Blynk.virtualWrite(VPIN_MANUAL_WATER, turnOn ? 1 : 0);// Reflect to UI
  Serial.println(String("Water: ") + (turnOn ? "ON" : "OFF")); // Log
}

// ===== Curtain control (mutually exclusive relays) =====
void startCurtainOpening() {                            // Begin OPEN motion
  digitalWrite(PIN_CURTAIN_CLOSE_RELAY, HIGH);          // Ensure CLOSE OFF
  digitalWrite(PIN_CURTAIN_OPEN_RELAY, LOW);            // Drive OPEN (LOW = ON)
  curtainState        = CURTAIN_OPENING;                // Update state
  curtainStopAtMillis = millis() + curtainMoveDurationMillis; // Auto-stop time
  Serial.println("Curtain: OPENING for 20s");           // Log
}

void startCurtainClosing() {                            // Begin CLOSE motion
  digitalWrite(PIN_CURTAIN_OPEN_RELAY, HIGH);           // Ensure OPEN OFF
  digitalWrite(PIN_CURTAIN_CLOSE_RELAY, LOW);           // Drive CLOSE (LOW = ON)
  curtainState        = CURTAIN_CLOSING;                // Update state
  curtainStopAtMillis = millis() + curtainMoveDurationMillis; // Auto-stop time
  Serial.println("Curtain: CLOSING for 20s");           // Log
}

void stopCurtainMotion() {                              // Stop motion (both OFF)
  digitalWrite(PIN_CURTAIN_OPEN_RELAY, HIGH);           // OPEN OFF
  digitalWrite(PIN_CURTAIN_CLOSE_RELAY, HIGH);          // CLOSE OFF
  curtainState = CURTAIN_STOPPED;                       // Update state
  Serial.println("Curtain: STOP");                      // Log
}

void checkCurtainAutoStop() {                           // Stop after duration
  if (curtainState != CURTAIN_STOPPED && millis() >= curtainStopAtMillis) {
    stopCurtainMotion();                                // Stop relays
    Blynk.virtualWrite(VPIN_MANUAL_OPEN, 0);            // Reset UI buttons
    Blynk.virtualWrite(VPIN_MANUAL_CLOSE, 0);           // Reset UI buttons
  }
}

// ===== Sensors =====
float computeSoilPercentFromAdc(int adcValue) {         // Map ADC to 0–100% using dry/wet refs
  float percent = (float)(soilAdcDryValue - adcValue) * 100.0f /
                  (float)(soilAdcDryValue - soilAdcWetValue);      // Linear scaling
  if (percent < 0)   percent = 0;                                   // Clamp 0
  if (percent > 100) percent = 100;                                 // Clamp 100
  return percent;                                                   // Return %
}

void readAllSensors() {                                             // Read DHT22, BH1750, soil ADC
  // --- DHT22 temperature & humidity ---
  TempAndHumidity dhtData = dhtSensor.getTempAndHumidity();         // Request reading
  if (dhtSensor.getStatus() == 0) {                                 // OK?
    airTemperatureCelsius = dhtData.temperature;                    // Save temp
    airRelativeHumidity   = dhtData.humidity;                       // Save humidity
    Serial.printf("✅ DHT22 - Temp: %.1f°C | Hum: %.1f%%\n",
                  airTemperatureCelsius, airRelativeHumidity);      // Log
  } else {                                                          // Error
    Serial.println("❌ DHT Error: " + String(dhtSensor.getStatusString())); // Log error text
    airTemperatureCelsius = 0;                                      // Fail-safe
    airRelativeHumidity   = 0;                                      // Fail-safe
  }

  // --- BH1750 light sensor ---
  if (bh1750LightSensor.measurementReady()) {                       // New lux ready?
    float measuredLux = bh1750LightSensor.readLightLevel();         // Read lux
    if (!isnan(measuredLux) && measuredLux >= 0) {                  // Validate
      if (measuredLux > luxGaugeMaxCap) measuredLux = luxGaugeMaxCap; // Cap for UI
      ambientLightLux = measuredLux;                                // Save
      Serial.printf("✅ BH1750 - Lux: %.1f lx\n", ambientLightLux); // Log
    } else {                                                        // Invalid read
      Serial.println("❌ BH1750 Error: Invalid reading");           // Log
      ambientLightLux = 0;                                          // Fail-safe
    }
  } else {                                                          // Not ready
    Serial.println("⚠️ BH1750 not ready (keep last)");              // Just inform
  }

  // --- Soil moisture via analog ADC ---
  int soilAdcReading = analogRead(PIN_SOIL_ANALOG);                 // Read ADC
  soilMoisturePercent = computeSoilPercentFromAdc(soilAdcReading);  // Convert to %
  Serial.printf("🌱 Soil moisture: %.1f%% (ADC=%d; dry=%d, wet=%d)\n",
                soilMoisturePercent, soilAdcReading, soilAdcDryValue, soilAdcWetValue); // Log
  Blynk.virtualWrite(VPIN_SOIL_PERCENT, soilMoisturePercent);       // Push to chart
}

// ===== Automation rules =====
void runAutomation() {                                              // Applies when Auto mode ON
  if (!autoModeEnabled) return;                                     // Skip if Manual

  // --- Mist by temperature band (simple ON/OFF with hysteresis-like band) ---
  if (airTemperatureCelsius >= mistTurnOnTemperatureCelsius)        // Hot enough?
    setMistRelay(true);                                             // Turn mist ON
  else if (airTemperatureCelsius <= mistTurnOffTemperatureCelsius)  // Cooled down?
    setMistRelay(false);                                            // Turn mist OFF

  // --- Fan by humidity band around target ---
  if (airRelativeHumidity >  targetHumidityPercent + humidityHysteresis) setFanRelay(true);  // Too humid → ON
  else if (airRelativeHumidity < targetHumidityPercent - humidityHysteresis) setFanRelay(false); // Dry enough → OFF

  // --- Curtain by lux (only if not already moving) ---
  if (curtainState == CURTAIN_STOPPED) {                             // Prevent overlap
    if (ambientLightLux >= targetLuxSetpoint)                        // Too bright?
      startCurtainClosing();                                         // Close curtain
    else if (ambientLightLux <= max(0.0f, targetLuxSetpoint - luxDeadband)) // Too dim?
      startCurtainOpening();                                         // Open curtain
  }

  // --- Water by soil moisture ---
  if (soilMoisturePercent < waterOnBelowSoilPercent) setWaterRelay(true);  // Dry → water ON
  else                                               setWaterRelay(false); // OK → water OFF
}

// ===== Push readings & status to Blynk =====
void pushReadingsToBlynk() {                                        // Sync widgets
  Blynk.virtualWrite(VPIN_TEMPERATURE, airTemperatureCelsius);      // Temperature
  Blynk.virtualWrite(VPIN_HUMIDITY,    airRelativeHumidity);        // Humidity
  Blynk.virtualWrite(VPIN_LUX,         ambientLightLux);            // Lux

  String curtainText;                                                // Human-readable state
  switch (curtainState) {
    case CURTAIN_OPENING: curtainText = "OPENING"; break;            // Mapping
    case CURTAIN_CLOSING: curtainText = "CLOSING"; break;
    default:              curtainText = "STOP";     break;
  }

  // Compose a single concise status line for a Label widget
  String statusLine = "🌡 " + String(airTemperatureCelsius, 1) + "°C | " +
                      "💧 " + String(airRelativeHumidity, 1)   + "% | " +
                      "🌤️ " + String(ambientLightLux, 1)       + " lx | " +
                      "🎯 LuxSet " + String(targetLuxSetpoint, 0) + " (±" + String(luxDeadband, 0) + ") | " +
                      "🌱 " + String(soilMoisturePercent, 1)   + "% | " +
                      "Mist:"   + (mistRelayState  ? "ON" : "OFF") + " | " +
                      "Fan:"    + (fanRelayState   ? "ON" : "OFF") + " | " +
                      "Water:"  + (waterRelayState ? "ON" : "OFF") + " | " +
                      "Curtain:"+ curtainText + " | " +
                      "Auto:"   + (autoModeEnabled ? "ON" : "OFF");

  Blynk.virtualWrite(VPIN_STATUS, statusLine);                       // Update label
  Serial.println(statusLine);                                        // Also log to Serial
}

// ===== Blynk input handlers (from app to device) =====
BLYNK_WRITE(VPIN_TEMP_SET) {                                         // Slider changed
  targetTemperatureCelsius = param.asFloat();                        // Update target temp
  Serial.println("Set Temperature: " + String(targetTemperatureCelsius) + "°C");
}

BLYNK_WRITE(VPIN_HUMIDITY_SET)  {                                    // Slider changed
  targetHumidityPercent = param.asFloat();                           // Update target humidity
  Serial.println("Set Humidity: " + String(targetHumidityPercent) + "%");
}

BLYNK_WRITE(VPIN_MANUAL_MIST)  { if (!autoModeEnabled) setMistRelay(param.asInt());  } // Manual mist (Auto OFF)
BLYNK_WRITE(VPIN_MANUAL_FAN)   { if (!autoModeEnabled) setFanRelay(param.asInt());   } // Manual fan  (Auto OFF)
BLYNK_WRITE(VPIN_MANUAL_WATER) { if (!autoModeEnabled) setWaterRelay(param.asInt()); } // Manual water (Auto OFF)

BLYNK_WRITE(VPIN_LUX_SETPOINT) {                                     // Lux setpoint slider
  targetLuxSetpoint = constrain(param.asFloat(), 0.0f, (float)luxGaugeMaxCap); // Clamp range
  Serial.println("Lux Setpoint: " + String(targetLuxSetpoint, 0) + " lx");
}

// ===== Blynk lifecycle =====
BLYNK_CONNECTED() {                                                  // On Blynk connected
  Serial.println("✅ Blynk Connected - Seeding gauges");             // Info log
  Blynk.virtualWrite(VPIN_TEMPERATURE,  airTemperatureCelsius);      // Seed UI
  Blynk.virtualWrite(VPIN_HUMIDITY,     airRelativeHumidity);        // Seed UI
  Blynk.virtualWrite(VPIN_LUX,          ambientLightLux);            // Seed UI
  Blynk.virtualWrite(VPIN_SOIL_PERCENT, soilMoisturePercent);        // Seed UI
  Blynk.virtualWrite(VPIN_LUX_SETPOINT, targetLuxSetpoint);          // Seed UI
  Blynk.virtualWrite(VPIN_MANUAL_MIST,  mistRelayState ? 1 : 0);     // Mirror buttons
  Blynk.virtualWrite(VPIN_MANUAL_FAN,   fanRelayState  ? 1 : 0);     // Mirror buttons
  Blynk.virtualWrite(VPIN_MANUAL_WATER, waterRelayState? 1 : 0);     // Mirror buttons
  Blynk.virtualWrite(VPIN_MANUAL_OPEN,  0);                          // Reset curtain buttons
  Blynk.virtualWrite(VPIN_MANUAL_CLOSE, 0);                          // Reset curtain buttons
  Blynk.syncVirtual(VPIN_TEMP_SET, VPIN_HUMIDITY_SET, VPIN_AUTO_MODE, VPIN_LUX_SETPOINT); // Pull latest from app
}

// ===== Setup (runs once) =====
void setup() {
  Serial.begin(115200);                                              // Start serial console
  Serial.println("🚀 Starting Smart Farm System...");                // Banner

  pinMode(PIN_MIST_RELAY, OUTPUT);                                   // Relay pin → output
  pinMode(PIN_FAN_RELAY, OUTPUT);                                    // Relay pin → output
  pinMode(PIN_CURTAIN_OPEN_RELAY, OUTPUT);                           // Relay pin → output
  pinMode(PIN_CURTAIN_CLOSE_RELAY, OUTPUT);                          // Relay pin → output
  pinMode(PIN_WATER_RELAY, OUTPUT);                                  // Relay pin → output

  // Active-Low relays: HIGH = OFF at boot (safe)
  digitalWrite(PIN_MIST_RELAY, HIGH);
  digitalWrite(PIN_FAN_RELAY, HIGH);
  digitalWrite(PIN_CURTAIN_OPEN_RELAY, HIGH);
  digitalWrite(PIN_CURTAIN_CLOSE_RELAY, HIGH);
  digitalWrite(PIN_WATER_RELAY, HIGH);

  dhtSensor.setup(PIN_DHT_SENSOR, DHTesp::DHT22);                    // Init DHT22
  Serial.println("✅ DHT22 Initialized");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);                              // Init I2C with custom pins
  Serial.println("✅ I2C Initialized (SDA:21, SCL:22)");

  if (bh1750LightSensor.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {   // Init BH1750
    Serial.println("✅ BH1750 Sensor Initialized");
  } else {
    Serial.println("❌ BH1750 Sensor Failed!");                      // Warn if missing
  }

  WiFi.mode(WIFI_STA);                                               // Station mode
  WiFi.begin(wifiSsid, wifiPassword);                                // Start Wi-Fi
  Serial.print("🔗 Connecting WiFi");
  uint32_t connectStartMillis = millis();                            // For timeout
  while (WiFi.status() != WL_CONNECTED && millis() - connectStartMillis < 15000) {
    delay(300);                                                      // Small wait
    Serial.print(".");                                               // Progress
  }
  if (WiFi.status() == WL_CONNECTED) {                               // Connected?
    Serial.println("\n✅ WiFi Connected: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n⚠️ WiFi not connected (will keep trying in loop)"); // Will retry later
  }

  Blynk.config(BLYNK_AUTH_TOKEN);                                    // Configure Blynk with token
  if (WiFi.status() == WL_CONNECTED) Blynk.connect(5000);            // Try connect to cloud (5s)

  // Seed UI so widgets aren’t blank on first open
  Blynk.virtualWrite(VPIN_TEMPERATURE,  airTemperatureCelsius);
  Blynk.virtualWrite(VPIN_HUMIDITY,     airRelativeHumidity);
  Blynk.virtualWrite(VPIN_LUX,          ambientLightLux);
  Blynk.virtualWrite(VPIN_SOIL_PERCENT, soilMoisturePercent);
  Blynk.virtualWrite(VPIN_LUX_SETPOINT, targetLuxSetpoint);

  Serial.println("✅ System Initialization Complete");               // Done
}

// ===== Loop (runs forever) =====
void loop() {
  Blynk.run();                                                        // Service Blynk tasks

  unsigned long nowMillis = millis();                                 // Current time
  if (nowMillis - lastSensorReadMillis >= sensorReadIntervalMillis) { // Time to read?
    lastSensorReadMillis = nowMillis;                                 // Mark time
    readAllSensors();                                                 // Read sensors
    runAutomation();                                                  // Apply rules
    pushReadingsToBlynk();                                            // Update UI
  }

  checkCurtainAutoStop();                                             // Stop curtain if time’s up

  // Wi-Fi auto-reconnect
  if (WiFi.status() != WL_CONNECTED) {                                // Lost Wi-Fi?
    static uint32_t lastReconnectTryMillis = 0;                       // Throttle attempts
    if (millis() - lastReconnectTryMillis > 5000) {                   // Try every 5s
      lastReconnectTryMillis = millis();
      Serial.println("📶 Reconnecting WiFi...");
      WiFi.disconnect();                                              // Reset
      WiFi.begin(wifiSsid, wifiPassword);                             // Reconnect
    }
  }

  // Blynk auto-reconnect when Wi-Fi is back
  if (WiFi.status() == WL_CONNECTED && !Blynk.connected()) {          // Wi-Fi OK but Blynk down?
    Serial.println("🔌 Reconnecting to Blynk...");
    Blynk.connect();                                                  // Reconnect
  }
}

/* ============================================================
   HOW TO USE (keep this comment in the same file for GitHub)
   ============================================================
   1) Libraries you need (Arduino IDE → Library Manager):
      - Blynk (Blynk IoT, ESP32 support)
      - DHT sensor library for ESPx (DHTesp)
      - BH1750
      - (Built-in) WiFi.h, Wire.h

   2) Wiring (ESP32 default board):
      - DHT22: VCC → 3V3, GND → GND, DATA → GPIO 4 (PIN_DHT_SENSOR). Use 10k pull-up if needed.
      - BH1750: VCC → 3V3, GND → GND, SDA → GPIO 21 (I2C_SDA_PIN), SCL → GPIO 22 (I2C_SCL_PIN).
      - Soil sensor (HD-38 analog out): AO → GPIO 34 (PIN_SOIL_ANALOG), VCC → 3V3/5V*, GND → GND.
        *Check your module’s voltage—most analog types are OK at 3V3.
      - Relays (Active-Low):
          MIST   → GPIO 27 (PIN_MIST_RELAY)
          FAN    → GPIO 14 (PIN_FAN_RELAY)
          WATER  → GPIO 12 (PIN_WATER_RELAY)
          CURTAIN OPEN  → GPIO 26 (PIN_CURTAIN_OPEN_RELAY)
          CURTAIN CLOSE → GPIO 25 (PIN_CURTAIN_CLOSE_RELAY)
        NOTE: Use opto-isolated relay modules; never mix grounds with AC mains without proper isolation.

   3) Fill secrets LOCALLY (do NOT commit):
      - Set `wifiSsid` and `wifiPassword` strings.
      - Set `BLYNK_TEMPLATE_ID` and `BLYNK_AUTH_TOKEN` from your Blynk device.
      - Keep them blank in the repo; put real values only on your machine or use a secrets.h (ignored by git).

   4) Blynk dashboard widgets (match virtual pins):
      - V0  : Slider (Lux Setpoint)          0..50000
      - V1  : Slider (Target Temperature)     e.g., 20..40
      - V2  : Slider (Target Humidity)        e.g., 30..90
      - V20 : Button (Manual Mist, switch)    0/1
      - V21 : Button (Manual Fan, switch)     0/1
      - V22 : Button (Manual Water, switch)   0/1
      - V30 : Label (Status line)
      - V31 : Switch (Auto Mode) [OPTIONAL]   0/1 (hook a handler if you want)
      - V40 : Gauge/Chart (Lux)
      - V41 : Gauge/Chart (Temperature)
      - V42 : Gauge/Chart (Humidity)
      - V43 : Chart (Soil Moisture %)
      - V10 : Button (Curtain OPEN) [OPTIONAL]
      - V11 : Button (Curtain CLOSE) [OPTIONAL]

   5) Soil calibration (very important):
      - Put probe in dry air/soil → read ADC in Serial Monitor → set `soilAdcDryValue`.
      - Put probe in water-saturated soil → read ADC → set `soilAdcWetValue`.
      - Adjust `waterOnBelowSoilPercent` to your crop needs (default 60%).

   6) Curtain timing:
      - `curtainMoveDurationMillis` is set to 20000ms (20s). Tune to your motor’s full travel time.
      - The code auto-stops after this duration to protect the motor.

   7) Safety notes:
      - All relays are ACTIVE-LOW. At boot they’re forced OFF (HIGH).
      - If you use mains equipment, isolate properly and follow electrical safety standards.

   8) Extending:
      - If you want the Auto/Manual toggle from Blynk, add a handler:
          BLYNK_WRITE(VPIN_AUTO_MODE) { autoModeEnabled = param.asInt(); }
      - You can also add manual OPEN/CLOSE buttons (V10/V11) to call startCurtainOpening()/startCurtainClosing().

   9) Git hygiene:
      - Keep this file in repo with blank strings.
      - Put your real secrets in a `secrets.h` (in .gitignore) and include it to overwrite variables at compile time.

   have fun & grow things 🌱🤘
*/
