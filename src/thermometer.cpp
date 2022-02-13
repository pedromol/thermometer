#include <Arduino.h>

#include <AsyncTimer.h>
#define AT_BAUD_RATE 115200
AsyncTimer t;

// TZ
#include <TimeLib.h>
#define TIME_ZONE -3

// SSD1306
#include "U8glib.h"
const U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);

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
#include <Adafruit_BMP280.h>

Adafruit_BMP280 BMP280;

// DHT11
#include <DHT_U.h>
#define DHT11_PIN 2

DHT_Unified dht11(DHT11_PIN, DHT11);

// BMP180
#include <Adafruit_BMP085_U.h>

Adafruit_BMP085_Unified BMP180 = Adafruit_BMP085_Unified(10085);

// TMP36
#include <TMP36.h>
#define TMP36_PIN A0

TMP36 tmp36(TMP36_PIN, 5.0);

// LM75
#include <M2M_LM75A.h>
#define LM35_PIN A1

M2M_LM75A LM75;

void serialSetup()
{
  Serial.begin(AT_BAUD_RATE);
  while (!Serial)
  {
    delay(100);
  };

  Serial1.begin(ESP_BAUD_RATE);

  WiFi.init(Serial1);
}

void getDS18B20()
{
  DS18B20.requestTemperatures();
  Serial.print("DS18B20 : ");
  Serial.println(DS18B20.getTempCByIndex(0));
}

void getBMP280()
{
  Serial.print("BMP280 : ");
  Serial.println(BMP280.readTemperature());
  BMP280.readPressure();
  BMP280.readAltitude(1013.25);
}

void getDHT11()
{
  sensors_event_t event;
  dht11.temperature().getEvent(&event);
  Serial.print("DHT11 : ");
  Serial.println(event.temperature);
  dht11.humidity().getEvent(&event);
  // event.relative_humidity
}

void getTMP36()
{
  Serial.print("TMP36 : ");
  Serial.println(tmp36.getTempC());
}

void getLM35()
{
  Serial.print("LM35 : ");
  Serial.println((float(analogRead(LM35_PIN)) * 5 / (1023)) / 0.01);
}

void getBMP180()
{
  // sensors_event_t event;
  // BMP180.getEvent(&event);
  // BMP180.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure);

  float temperature;
  BMP180.getTemperature(&temperature);
  Serial.print("BMP180 : ");
  Serial.println(temperature);
}

void getLM75()
{
  Serial.print("LM75 : ");
  Serial.println(LM75.getTemperature());
}

void getWifiTime()
{
  setTime(WiFi.getTime() + (SECS_PER_HOUR * TIME_ZONE));
}

void draw()
{
  char buff[20];

  u8g.firstPage();
  do
  {
    // Date
    sprintf(buff, "%02d/%02d", day(), month());
    u8g.drawStr(5, 30, buff);

    // Date | Temperature
    u8g.drawLine(80, 0, 80, 35);

    // Temperature
    u8g.drawStr(89, 30, "--");

    // First line -- Second line
    u8g.drawLine(0, 35, 128, 35);

    // Hour
    sprintf(buff, "%02d:%02d:%02d", hour(), minute(), second());
    u8g.drawStr(10, 59, buff);

    // Frame
    u8g.drawRFrame(0, 0, 128, 64, 4);
  } while (u8g.nextPage());
}

void setup()
{
  analogReference(INTERNAL);
  u8g.setColorIndex(1);
  u8g.setFont(u8g_font_fub20n);

  serialSetup();
  getWifiTime();

  BMP280.begin();

  dht11.begin();

  BMP180.begin();

  pinMode(TMP36_PIN, INPUT);
  pinMode(LM35_PIN, INPUT);

  LM75.begin();

  t.setInterval([]()
                { draw(); },
                1000);
  t.setInterval([]()
                {
    getDS18B20();
    getBMP280();
    getDHT11();
    getTMP36();
    getLM35();
    getBMP180();
    getLM75();
    Serial.println("");
    Serial.println(""); },
                2000);
  t.setInterval([]()
                { getWifiTime(); },
                60000);
}

void loop() { t.handle(); }
