#include <Arduino.h>

#define imageWidth 128
#define imageHeight 64

const unsigned char bitmap[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0x80, 0x00, 0x7f, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xf0, 0x00, 0x00, 0x05, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x05, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xe0, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xff, 0xff, 0xf8, 0x00, 0x17, 0xf8, 0x00, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xff, 0xff, 0xc0, 0x01, 0xff, 0xff, 0xc0, 0x00, 0x3f, 0xff, 0xf0, 0x00, 0x00,
    0x00, 0x00, 0x01, 0xff, 0xfe, 0x00, 0x1f, 0xff, 0xff, 0xfc, 0x00, 0x1f, 0xff, 0xf0, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xff, 0xe0, 0x01, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x7f, 0xc0, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xff, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xf0, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0x9f, 0xff, 0x1f, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0x02, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x0f, 0xff, 0xfe, 0x00, 0x1f, 0xff, 0x00, 0x1f, 0xff, 0xff, 0xc0, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xff, 0xff, 0xf8, 0x00, 0x1f, 0xff, 0x00, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x00,
    0x00, 0x00, 0x01, 0xff, 0xff, 0x00, 0x00, 0x1f, 0xfe, 0x00, 0x00, 0xff, 0xff, 0xe0, 0x00, 0x00,
    0x00, 0x00, 0x01, 0xff, 0xe0, 0x00, 0x00, 0x0f, 0xfe, 0x00, 0x00, 0x3f, 0xff, 0xf0, 0x00, 0x00,
    0x00, 0x00, 0x01, 0xff, 0x80, 0x00, 0x00, 0x0f, 0xfc, 0x00, 0x00, 0x0f, 0xff, 0xf0, 0x00, 0x00,
    0x00, 0x00, 0x01, 0xff, 0xf0, 0x00, 0x00, 0x03, 0xf8, 0x00, 0x00, 0x1f, 0xfe, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xe0, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3f, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x80, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xfe, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xf8, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xf8, 0x00, 0x00, 0x0b, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xfd, 0x7f, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xc7, 0xc0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x50, 0x3f, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x7f, 0x80, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x07, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x07, 0xf7, 0xf0, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x0f, 0xfc, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0f, 0xc7, 0xf0, 0x00, 0x00, 0x00, 0x1f, 0x80, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0f, 0x87, 0xf0, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0f, 0x83, 0xe0, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x1f, 0xc0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0f, 0x03, 0xc0, 0x00, 0x00, 0x01, 0xfc, 0x00, 0x1f, 0xc0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x1f, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xc0, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x03, 0xfc, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x3f, 0xe0, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xf0, 0x00, 0x01, 0xff, 0x80, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xf8, 0x00, 0x07, 0xfe, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0xff, 0x10, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x03, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "credentials.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET 4
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define SDA_PIN 3
#define SCL_PIN 1

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHT_PIN 2

DHT_Unified dht11(DHT_PIN, DHT11);

#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_PIN 0
OneWire oneWire(ONE_WIRE_PIN);

DallasTemperature DS18B20(&oneWire);
DeviceAddress ds18b20;

// Sensors constants
const float MIN_TEMP = 10;
const float MAX_TEMP = 50;

const float MIN_HUMIDITY = 1;
const float MAX_HUMIDITY = 100;

const float MIN_VCC = 0;
const float MAX_VCC = 10;

// PubSub
#include <PubSubClient.h>

#define TEMPERATURE_TOPIC "sensor/temperature"
#define HUMIDITY_TOPIC "sensor/humidity"
#define VCC_TOPIC "sensor/vcc"
#define INVALID_TOPIC "sensor/invalid"

#define MESSAGE_DELAY 30000
#define PROBE_DELAY 2000
#define LOOP_DELAY 1000

float temperatureValues = 0;
int temperatureProbes = 0;
float temperatureDisplay = 0;

float humidityValues = 0;
int humidityProbes = 0;
float humidityDisplay = 0;

float vccValues = 0;
int vccProbes = 0;

int failedProbes = 0;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

long lastMessage = 0;
long lastProbe = 0;
long lastLoop = 0;

#define MAX_FAILURE 512

int totalFailed = 0;

ADC_MODE(ADC_VCC);

void wifiSetup()
{

  WiFi.disconnect();

  WiFi.mode(WIFI_STA);
  WiFi.persistent(true);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  WiFi.setAutoReconnect(true);

  configTime(-3 * 60 * 60, 0, "pool.ntp.br", "0.br.pool.ntp.org", "1.br.pool.ntp.org");
}

void displaySetup()
{

  Wire.begin(SDA_PIN, SCL_PIN);

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.setTextColor(SSD1306_WHITE);

  display.setRotation(2);
}

boolean queueConnect()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    return false;
  }

  if (!client.connected())
  {
    return client.connect(DEVICE_NAME);
  }

  return client.connected();
}

boolean publish(const char *topic, const char *payload, boolean retained)
{
  if (queueConnect())
  {
    return client.publish(topic, payload, retained);
  }
  return false;
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

  float vccs = 0;
  int8_t validVccs = 0;

  sensors_event_t event;

  int8_t invalidProbes = 0;

  DS18B20.requestTemperatures();
  addIfValid(MAX_TEMP, MIN_TEMP, DS18B20.getTempC(ds18b20), &temperatures, &validTemperatures, &invalidProbes);

  dht11.temperature().getEvent(&event);
  addIfValid(MAX_TEMP, MIN_TEMP, event.temperature, &temperatures, &validTemperatures, &invalidProbes);

  dht11.humidity().getEvent(&event);
  addIfValid(MAX_HUMIDITY, MIN_HUMIDITY, event.relative_humidity, &humidities, &validHumidities, &invalidProbes);

  addIfValid(MAX_VCC, MIN_VCC, (float)ESP.getVcc() / 1024.0f, &vccs, &validVccs, &invalidProbes);

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

  if (validVccs > 0)
  {
    vccValues += vccs / validVccs;
    vccProbes += 1;
  }

  if (invalidProbes > 0)
  {
    failedProbes += invalidProbes;
    totalFailed += invalidProbes;
  }
}

void sendData()
{
  if (temperatureProbes > 0)
  {
    temperatureValues /= temperatureProbes;
    temperatureDisplay = temperatureValues;
    publish(TEMPERATURE_TOPIC, String(temperatureValues).c_str(), true);
    temperatureValues = temperatureProbes = 0;
  }

  if (humidityProbes > 0)
  {
    humidityValues /= humidityProbes;
    humidityDisplay = humidityValues;
    publish(HUMIDITY_TOPIC, String(humidityValues).c_str(), true);
    humidityValues = humidityProbes = 0;
  }

  if (vccProbes > 0)
  {
    vccValues /= vccProbes;
    publish(VCC_TOPIC, String(vccValues).c_str(), true);
    vccValues = vccProbes = 0;
  }

  publish(INVALID_TOPIC, String(failedProbes).c_str(), true);
  failedProbes = 0;
}

void drawSplash()
{
  display.clearDisplay();
  display.drawBitmap(0, 0, bitmap, imageWidth, imageHeight, SSD1306_WHITE);
  display.display();
}

void drawDisplay()
{
  time_t now = time(nullptr);
  struct tm *info = localtime((const time_t *)&now);

  if (info->tm_year < 100)
  {
    return;
  }

  char parsedTime[9];
  char parsedDate[11];

  strftime(parsedTime, sizeof(parsedTime), "%H:%M", info);
  strftime(parsedDate, sizeof(parsedDate), "%Y-%m-%d", info);

  display.clearDisplay();

  display.setTextSize(1);

  // Clock border
  // display.drawRoundRect(0, 0, 128, 64, 8, SSD1306_WHITE);

  // Temperature
  display.setCursor(4, 4);
  display.print(temperatureDisplay);
  display.print(" C");

  // Humidity
  display.setCursor(83, 4);
  display.print(humidityDisplay);
  display.print(" %");

  // Date
  display.setCursor(32, 50);
  display.print(parsedDate);

  // Time
  display.setTextSize(3);
  display.setCursor(20, 20);
  display.print(parsedTime);

  display.display();
}

void setup()
{
  Serial.begin(115200);
  displaySetup();
  drawSplash();

  wifiSetup();
  client.setServer(MQTT_SERVER, 1883);
  queueConnect();

  dht11.begin();
  DS18B20.begin();
  DS18B20.getAddress(ds18b20, 0);
}

void loop()
{
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
  if (now - lastLoop > LOOP_DELAY)
  {
    drawDisplay();
    client.loop();
    lastLoop = now;
  }
  if (totalFailed > MAX_FAILURE)
  {
    ESP.reset();
  }
  delay(10);
}
