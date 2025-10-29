# iot-esp32-project-4-sensor-
An IoT-based monitoring system using ESP32 and 4 sensors (temperature, humidity, light, and gas) to collect real-time data and send it to the cloud for analysis and visualization through a web dashboard.
# 🌿 Smart Farm (IoT-ESP32-4-Sensor)

## 📘 Description
An IoT-based smart farming system using **ESP32** and four sensors (**DHT22**, **BH1750**, **HD-38 Soil Moisture**, and **Relay Modules**).  
The system monitors **temperature, humidity, light intensity,** and **soil moisture**, then automatically controls **fan, mist, curtain, and water valve** based on environmental conditions.  
All data is uploaded to **Blynk IoT** for real-time monitoring and manual control via a mobile dashboard.

---

## ⚙️ Features
- 🌡️ Measure **temperature & humidity** (DHT22 sensor)  
- 🌤️ Measure **ambient light intensity** (BH1750 sensor)  
- 🌱 Monitor **soil moisture** (HD-38 analog sensor)  
- 💧 Automatically control **mist, fan, and water valve**  
- 🪟 Smart curtain system reacts to light levels  
- 📶 Real-time data display on **Blynk IoT app**  
- 🔄 Automatic + Manual control modes  
- 🧠 Built-in hysteresis logic for stable automation  

---

## 🧩 Hardware Components
| Component | Description |
|------------|--------------|
| ESP32 | Main microcontroller with Wi-Fi |
| DHT22 | Temperature and humidity sensor |
| BH1750 | Light intensity sensor (I2C) |
| HD-38 | Soil moisture analog sensor |
| Relay Modules (x4) | Control mist, fan, curtain, and water valve |
| 12V Power Supply | Power source for relays and actuators |

---

## 🔌 Pin Configuration
| Function | ESP32 Pin |
|-----------|------------|
| DHT22 Sensor | GPIO 4 |
| Mist Relay | GPIO 27 |
| Fan Relay | GPIO 14 |
| Curtain Open | GPIO 26 |
| Curtain Close | GPIO 25 |
| Soil Moisture (Analog) | GPIO 34 |
| Water Valve Relay | GPIO 12 |
| I2C SDA (BH1750) | GPIO 21 |
| I2C SCL (BH1750) | GPIO 22 |

---

## 📡 Blynk Virtual Pins
| Function | Virtual Pin |
|-----------|--------------|
| Temperature Setpoint | V1 |
| Humidity Setpoint | V2 |
| Temperature Display | V41 |
| Humidity Display | V42 |
| Light (Lux) Display | V40 |
| Soil Moisture (%) | V43 |
| Manual Controls | V10–V22 |
| Auto Mode Toggle | V31 |
| System Status | V30 |

---

## 📱 Blynk Dashboard
- **Gauges:** Temperature, Humidity, Light, Soil Moisture  
- **Sliders:** Setpoints for temperature and light threshold  
- **Buttons:** Manual control for mist, fan, curtain, and watering  
- **Terminal Widget:** Displays real-time system status logs  

---

## 🛠️ Setup Guide
1. **Install libraries in Arduino IDE:**
   - `Blynk`
   - `DHTesp`
   - `BH1750`
   - `Wire`
   - `WiFi`
2. **Update Wi-Fi credentials** inside the code:
   ```cpp
   char wifiSsid[] = "YOUR_WIFI_SSID";
   char wifiPassword[] = "YOUR_WIFI_PASSWORD";
