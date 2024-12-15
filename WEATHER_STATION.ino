// #define BLYNK_TEMPLATE_ID           "TMPL60RU4nGLq"
// #define BLYNK_TEMPLATE_NAME         "WEATHER STATION"
// #define BLYNK_AUTH_TOKEN            "1orrm8Q7IoHEdZh9hnDXoceQYVm7cBew"

// #define BLYNK_TEMPLATE_ID "TMPL2mynv07ym"
// #define BLYNK_TEMPLATE_NAME "WEATHER STATION"
// #define BLYNK_AUTH_TOKEN "9jtdITYXF8diXzpk1HkQOgbJbyyCSwlj"

#define BLYNK_TEMPLATE_ID "TMPL6STvU11J5"
#define BLYNK_TEMPLATE_NAME "WEATHER STATION"
#define BLYNK_AUTH_TOKEN "345jsVaQn6XkwcNifZEYRURVabsVtQ_J"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"

#define BLYNK_PRINT Serial

#define DHTPIN 19
#define DHTTYPE DHT11
#define RAIN_PIN 2
#define RAIN_PER_TIP 0.40894;

DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bmp;

// char ssid[] = "HUAWEI-2.4G-k5cR";
// char pass[] = "8RdK8Q6G";

char ssid[] = "POCO X6 Pro 5G";
char pass[] = "4150013kit";

int anemometerPin = 26;
int Count = 0;

volatile unsigned long tipCount = 0;
volatile unsigned long contactTime = 0;
volatile double totalRainfall = 0.0;
volatile unsigned long previousTime = 0;

bool wifiConnected = false;

BlynkTimer timer;

void IRAM_ATTR handleAnemometer() {
  if (digitalRead(anemometerPin) == LOW) {
    Count++;
  }
}

void sendSensorData() {

  double temperature = dht.readTemperature();
  double humidity = dht.readHumidity();
  int pressure = bmp.readPressure() / 100.0F;
  double windSpeed = (Count * 8.75) / 100.0;  // Assuming the wind speed calculation formula
  static int lastTipCount = 0;
  totalRainfall = tipCount * RAIN_PER_TIP;

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from one or more sensors");
    return;
  }

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" *C\t");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %\t");
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.print(" hPa\t");
  Serial.print("Wind Speed: ");
  Serial.print(windSpeed);
  Serial.print(" m/s\t");

  if (millis() - 5000 >= previousTime) {
    Serial.print("Precipitation: ");
    Serial.print(totalRainfall);
    Serial.println(" mm\t");
    previousTime = millis();
    tipCount = 0;
  }

  // Send data to Blynk app
  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, humidity);
  Blynk.virtualWrite(V2, pressure);
  Blynk.virtualWrite(V3, windSpeed);
  Blynk.virtualWrite(V4, totalRainfall);

  Count = 0;
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid,pass);
  Serial.print("Connecting to Wi-Fi");
  for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
    delay(1000);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected");
    wifiConnected = true;
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "blynk.cloud", 80);
  }

  else {
    Serial.print("Not Connected");
  }

  // Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "blynk.cloud", 80);

  dht.begin();
  Wire.begin();
  
  pinMode(anemometerPin, INPUT_PULLUP);  // Set the interrupt pin for anemometer
  pinMode(RAIN_PIN, INPUT_PULLUP);  // Set the rain gauge pin
  
  attachInterrupt(digitalPinToInterrupt(anemometerPin), handleAnemometer, FALLING);  // Interrupt for anemometer
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), isr_rg, FALLING);  // Interrupt for rain gauge
  sei();

  Serial.println("Free heap: " + String(ESP.getFreeHeap()));

  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP sensor, check wiring!");
    while (1);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, 
                  Adafruit_BMP280::SAMPLING_X2, 
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  timer.setInterval(10000L, sendSensorData);  // Update sensor data every 10 seconds
}

void loop() {
  Blynk.run();
  timer.run();  // Trigger the sensor data sending function
}

void isr_rg() {
 if (millis() - contactTime > 500) {
      Serial.print("ISR CHECK!");
      tipCount++;
      contactTime = millis();
  }
}