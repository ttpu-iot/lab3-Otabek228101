#include "Arduino.h"
#include "WiFi.h"
#include <ArduinoJson.h>
#include "PubSubClient.h"
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <time.h>

const int RED_LED_PIN = 26;
const int GREEN_LED_PIN = 27;
const int BLUE_LED_PIN = 14;
const int YELLOW_LED_PIN = 12;
const int BUTTON_PIN = 25;

const char *WIFI_SSID = "Wokwi-GUEST";
const char *WIFI_PASSWORD = "";

const char *MQTT_HOST = "mqtt.iotserver.uz";
const int MQTT_PORT = 1883;
const char *MQTT_USERNAME = "userTTPU";
const char *MQTT_PASSWORD = "mqttpass";

const char *TOPIC_LED_RED = "ttpu/iot/otabek/led/red";
const char *TOPIC_LED_GREEN = "ttpu/iot/otabek/led/green";
const char *TOPIC_LED_BLUE = "ttpu/iot/otabek/led/blue";
const char *TOPIC_LED_YELLOW = "ttpu/iot/otabek/led/yellow";
const char *TOPIC_BUTTON = "ttpu/iot/otabek/events/button";
const char *TOPIC_DISPLAY = "ttpu/iot/otabek/display";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

hd44780_I2Cexp lcd;
String lcdLine1 = "Waiting...";
String lcdLine2 = "--/-- --:--:--";

int lastButtonReading = LOW;
int buttonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

void connectToWifi();
void connectToMqtt();
void subscribeToTopics();
void handleMqttMessage(char *topic, byte *payload, unsigned int length);
void setLedState(int pin, const char *label, const char *state);
void publishButtonEvent(const char *eventName);
void checkButton();
void updateLcd();
String getFormattedTime();

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("Lab 3 - Exercise 3 (beginner version)");

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT);

  int lcdStatus = lcd.begin(16, 2);
  if (lcdStatus != 0)
  {
    Serial.print("LCD init failed, status=");
    Serial.println(lcdStatus);
  }
  lcd.backlight();
  updateLcd();

  connectToWifi();

  Serial.println("Syncing time with NTP (UTC+5)...");
  configTime(18000, 0, "pool.ntp.org");

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(handleMqttMessage);
  connectToMqtt();
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi lost, reconnecting...");
    connectToWifi();
  }

  if (!mqttClient.connected())
  {
    Serial.println("MQTT lost, reconnecting...");
    connectToMqtt();
  }

  mqttClient.loop();
  checkButton();
}

void connectToWifi()
{
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void connectToMqtt()
{
  while (!mqttClient.connected())
  {
    Serial.println("Connecting to MQTT...");
    String clientId = "lab3_ex3_" + String(random(0xffff), HEX);

    bool ok = mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD);
    if (ok)
    {
      Serial.println("MQTT connected!");
      subscribeToTopics();
    }
    else
    {
      Serial.print("MQTT failed, rc=");
      Serial.println(mqttClient.state());
      Serial.println("Retrying in 2 seconds...");
      delay(2000);
    }
  }
}

void subscribeToTopics()
{
  mqttClient.subscribe(TOPIC_LED_RED);
  mqttClient.subscribe(TOPIC_LED_GREEN);
  mqttClient.subscribe(TOPIC_LED_BLUE);
  mqttClient.subscribe(TOPIC_LED_YELLOW);
  mqttClient.subscribe(TOPIC_DISPLAY);

  Serial.println("Subscribed to:");
  Serial.println(TOPIC_LED_RED);
  Serial.println(TOPIC_LED_GREEN);
  Serial.println(TOPIC_LED_BLUE);
  Serial.println(TOPIC_LED_YELLOW);
  Serial.println(TOPIC_DISPLAY);
}

void handleMqttMessage(char *topic, byte *payload, unsigned int length)
{
  String body = "";
  for (unsigned int i = 0; i < length; i++)
  {
    body += (char)payload[i];
  }

  Serial.print("[MQTT] Message on ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(body);

  StaticJsonDocument<192> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err)
  {
    Serial.print("JSON error: ");
    Serial.println(err.c_str());
    return;
  }

  if (strcmp(topic, TOPIC_DISPLAY) == 0)
  {
    const char *text = doc["text"];
    if (text == nullptr)
    {
      Serial.println("Display payload missing 'text'");
      return;
    }
    String newLine = String(text);
    if (newLine.length() > 16)
    {
      newLine = newLine.substring(0, 16);
    }
    lcdLine1 = newLine;
    lcdLine2 = getFormattedTime();
    updateLcd();
  }
  else
  {
    const char *state = doc["state"];
    if (state == nullptr)
    {
      Serial.println("LED payload missing 'state'");
      return;
    }

    if (strcmp(topic, TOPIC_LED_RED) == 0)
    {
      setLedState(RED_LED_PIN, "Red", state);
    }
    else if (strcmp(topic, TOPIC_LED_GREEN) == 0)
    {
      setLedState(GREEN_LED_PIN, "Green", state);
    }
    else if (strcmp(topic, TOPIC_LED_BLUE) == 0)
    {
      setLedState(BLUE_LED_PIN, "Blue", state);
    }
    else if (strcmp(topic, TOPIC_LED_YELLOW) == 0)
    {
      setLedState(YELLOW_LED_PIN, "Yellow", state);
    }
  }
}

void setLedState(int pin, const char *label, const char *state)
{
  if (strcmp(state, "ON") == 0)
  {
    digitalWrite(pin, HIGH);
    Serial.print("[LED] ");
    Serial.print(label);
    Serial.println(" -> ON");
  }
  else if (strcmp(state, "OFF") == 0)
  {
    digitalWrite(pin, LOW);
    Serial.print("[LED] ");
    Serial.print(label);
    Serial.println(" -> OFF");
  }
  else
  {
    Serial.print("Unknown LED state for ");
    Serial.print(label);
    Serial.print(": ");
    Serial.println(state);
  }
}

void publishButtonEvent(const char *eventName)
{
  StaticJsonDocument<128> doc;
  doc["event"] = eventName;
  doc["timestamp"] = time(nullptr);

  char payload[128];
  serializeJson(doc, payload, sizeof(payload));

  Serial.print("[MQTT] Publish button: ");
  Serial.println(payload);

  if (!mqttClient.publish(TOPIC_BUTTON, payload))
  {
    Serial.println("Failed to publish button event");
  }
}

void checkButton()
{
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading)
  {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY)
  {
    if (reading != buttonState)
    {
      buttonState = reading;
      if (buttonState == HIGH)
      {
        publishButtonEvent("PRESSED");
      }
      else
      {
        publishButtonEvent("RELEASED");
      }
    }
  }

  lastButtonReading = reading;
}

void updateLcd()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(lcdLine1);
  lcd.setCursor(0, 1);
  lcd.print(lcdLine2);

  Serial.print("[LCD] Line1='");
  Serial.print(lcdLine1);
  Serial.print("' Line2='");
  Serial.print(lcdLine2);
  Serial.println("'");
}

String getFormattedTime()
{
  struct tm timeinfo;
  if (getLocalTime(&timeinfo))
  {
    char buffer[20];
    strftime(buffer, sizeof(buffer), "%d/%m %H:%M:%S", &timeinfo);
    return String(buffer);
  }
  else
  {
    Serial.println("Failed to read local time");
    return String("--/-- --:--:--");
  }
}
