#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "credentials.h"

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHT_PIN 2

DHT_Unified dht11(DHT_PIN, DHT11);

#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 0
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature DS18B20(&oneWire);

// Sensors constants
const float MIN_TEMP = 10;
const float MAX_TEMP = 50;

const float MIN_HUMIDITY = 1;
const float MAX_HUMIDITY = 100;

// PubSub
#include <PubSubClient.h>

#define TEMPERATURE_TOPIC "sensor/temperature"
#define HUMIDITY_TOPIC "sensor/humidity"
#define INVALID_TOPIC "sensor/invalid"

#define MESSAGE_DELAY 30000
#define PROBE_DELAY 2000

float temperatureValues = 0;
int temperatureProbes = 0;

float humidityValues = 0;
int humidityProbes = 0;

int failedProbes = 0;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

long lastMessage = 0;
long lastProbe = 0;

void wifiSetup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(100);
  };

  WiFi.mode(WIFI_STA);
  WiFi.persistent(true);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
  }

  WiFi.setAutoReconnect(true);

  if (time(nullptr) < 8 * 3600 * 2)
  {
    configTime(0, 0, "pool.ntp.br", "0.br.pool.ntp.org", "1.br.pool.ntp.org");
  }
}

void addIfValid(float maxValue, float minValue, float value, float *values, int8_t *validValues, int8_t *invalidProbes)
{
  if (!isnan(value) && value > minValue && value < maxValue)
  {
    *values += value;
    *validValues += 1;
  }
  else
  {
    *invalidProbes += 1;
  }
}

void probeSensors()
{
  float temperatures = 0;
  int8_t validTemperatures = 0;

  float humidities = 0;
  int8_t validHumidities = 0;

  sensors_event_t event;

  int8_t invalidProbes = 0;

  DS18B20.requestTemperatures();
  addIfValid(MAX_TEMP, MIN_TEMP, DS18B20.getTempCByIndex(0), &temperatures, &validTemperatures, &invalidProbes);

  dht11.temperature().getEvent(&event);
  addIfValid(MAX_TEMP, MIN_TEMP, event.temperature, &temperatures, &validTemperatures, &invalidProbes);

  dht11.humidity().getEvent(&event);
  addIfValid(MAX_HUMIDITY, MIN_HUMIDITY, event.relative_humidity, &humidities, &validHumidities, &invalidProbes);

  if (validTemperatures > 0)
  {
    temperatureValues += temperatures / validTemperatures;
    temperatureProbes += 1;
  }

  if (validHumidities > 0)
  {
    humidityValues += humidities / validHumidities;
    humidityProbes += 1;
  }

  if (invalidProbes > 0)
  {
    failedProbes += invalidProbes;
  }
}

void sendData()
{
  if (temperatureProbes > 0)
  {
    temperatureValues /= temperatureProbes;
    client.publish(TEMPERATURE_TOPIC, String(temperatureValues).c_str(), true);
    temperatureValues = temperatureProbes = 0;
  }

  if (humidityProbes > 0)
  {
    humidityValues /= humidityProbes;
    client.publish(HUMIDITY_TOPIC, String(humidityValues).c_str(), true);
    humidityValues = humidityProbes = 0;
  }

  client.publish(INVALID_TOPIC, String(failedProbes).c_str(), true);
  failedProbes = 0;
}

void queueConnect()
{
  while (!client.connected())
  {
    if (!client.connect(DEVICE_NAME))
    {
      delay(2000);
    }
  }
}

void setup()
{
  wifiSetup();
  client.setServer(MQTT_SERVER, 1883);
  queueConnect();

  dht11.begin();
}

void loop()
{
  queueConnect();
  client.loop();

  long now = millis();
  if (now - lastProbe > PROBE_DELAY)
  {
    probeSensors();
    lastProbe = now;
  }
  if (now - lastMessage > MESSAGE_DELAY)
  {
    sendData();
    lastMessage = now;
  }
}
