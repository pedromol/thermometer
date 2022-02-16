#include <Arduino.h>
#include "credentials.h"
#define AT_BAUD_RATE 115200

// ESP8266
#include <SoftwareSerial.h>
#include <WiFiEspAT.h>
#define ESP_BAUD_RATE 9600

#define SERIAL_RX_PIN 6
#define SERIAL_TX_PIN 7
SoftwareSerial Serial1(SERIAL_RX_PIN, SERIAL_TX_PIN);

// Common
#include <Adafruit_Sensor.h>
#include <Wire.h>

// DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONEWIRE_PIN 2

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature DS18B20(&oneWire);

// BMP280
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP280_SCK 12  // SCL
#define BMP280_MISO 11 // SDD
#define BMP280_MOSI 10 // SDA
#define BMP280_CS 9    // CSB

Adafruit_BMP280 BMP280(BMP280_CS, BMP280_MOSI, BMP280_MISO, BMP280_SCK);

// DHT11
#include <DHT_U.h>
#define DHT11_PIN A2

DHT_Unified dht11(DHT11_PIN, DHT11);

// BMP180
#include <Adafruit_BMP085_U.h>

Adafruit_BMP085_Unified BMP180 = Adafruit_BMP085_Unified(10085);

// Sensors constants
const float MIN_TEMP = 10;
const float MAX_TEMP = 50;

const float MIN_HUMIDITY = 1;
const float MAX_HUMIDITY = 100;

const float MIN_PRESSURE = 800;
const float MAX_PRESSURE = 1200;

// PubSub
#include <PubSubClient.h>

#define TEMPERATURE_TOPIC "sensor/temperature"
#define HUMIDITY_TOPIC "sensor/humidity"
#define PRESSURE_TOPIC "sensor/pressure"
#define MESSAGE_DELAY 2000

WiFiClient wifiClient;
PubSubClient client(wifiClient);

long lastMessage = 0;

void serialSetup()
{
  Serial.begin(AT_BAUD_RATE);
  while (!Serial)
  {
    delay(100);
  };

  Serial1.begin(ESP_BAUD_RATE);

  WiFi.init(Serial1);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
  }
}

void addIfValid(float maxValue, float minValue, float value, float *values, int8_t *validValues)
{
  if (!isnan(value) && value > minValue && value < maxValue)
  {
    *values += value;
    *validValues += 1;
  }
}

void plimPlom()
{
  float temperatures = 0;
  int8_t validTemperatures = 0;

  float pressures = 0;
  int8_t validPressures = 0;

  float humidities = 0;
  int8_t validHumidities = 0;

  sensors_event_t event;
  float tmp;

  DS18B20.requestTemperatures();
  addIfValid(MAX_TEMP, MIN_TEMP, DS18B20.getTempCByIndex(0), &temperatures, &validTemperatures);

  addIfValid(MAX_TEMP, MIN_TEMP, BMP280.readTemperature() + 1, &temperatures, &validTemperatures);

  addIfValid(MAX_PRESSURE, MIN_PRESSURE, BMP280.readPressure() / 100, &pressures, &validPressures);

  dht11.temperature().getEvent(&event);
  addIfValid(MAX_TEMP, MIN_TEMP, event.temperature, &temperatures, &validTemperatures);

  dht11.humidity().getEvent(&event);
  addIfValid(MAX_HUMIDITY, MIN_HUMIDITY, event.relative_humidity, &humidities, &validHumidities);

  BMP180.getEvent(&event);
  BMP180.getTemperature(&tmp);
  addIfValid(MAX_TEMP, MIN_TEMP, tmp, &temperatures, &validTemperatures);
  addIfValid(MAX_PRESSURE, MIN_PRESSURE, event.pressure + 1, &pressures, &validPressures);

  temperatures /= validTemperatures;
  humidities /= validHumidities;
  pressures /= validPressures;

  client.publish(TEMPERATURE_TOPIC, String(temperatures).c_str(), true);
  client.publish(HUMIDITY_TOPIC, String(humidities).c_str(), true);
  client.publish(PRESSURE_TOPIC, String(pressures).c_str(), true);
}

void queueConnect()
{
  while (!client.connected())
  {
    if (!client.connect(MQTT_DEVICE_NAME))
    {
      delay(2000);
    }
  }
}

void setup()
{
  serialSetup();
  client.setServer(MQTT_SERVER, 1883);
  queueConnect();

  BMP280.begin();
  BMP180.begin();
  dht11.begin();
}

void loop()
{
  queueConnect();
  client.loop();

  long now = millis();
  if (now - lastMessage > MESSAGE_DELAY)
  {
    lastMessage = now;
    plimPlom();
  }
}
