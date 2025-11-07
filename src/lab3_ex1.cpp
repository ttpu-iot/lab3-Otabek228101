#include "Arduino.h"
#include "WiFi.h"
#include "PubSubClient.h"
#include <ArduinoJson.h>
#include <time.h>

const int RED_LED_PIN = 26;
const int GREEN_LED_PIN = 27;
const int BLUE_LED_PIN = 14;
const int YELLOW_LED_PIN = 12;
const int BUTTON_PIN = 25;
const int LIGHT_SENSOR_PIN = 33;

const char *WIFI_SSID = "Wokwi-GUEST";
const char *WIFI_PASSWORD = "";

const char *MQTT_HOST = "mqtt.iotserver.uz";
const int MQTT_PORT = 1883;
const char *MQTT_USERNAME = "userTTPU";
const char *MQTT_PASSWORD = "mqttpass";

const char *TOPIC_LIGHT = "ttpu/iot/otabek/sensors/light";
const char *TOPIC_BUTTON = "ttpu/iot/otabek/events/button";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastLightPublish = 0;
const unsigned long LIGHT_INTERVAL = 5000;

int lastButtonReading = LOW;
int buttonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

void connectToWifi();
void connectToMqtt();
void publishLightSensor();
void checkButtonEvents();

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("Lab 3 - Exercise 1 (beginner version)");

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT);

  connectToWifi();

  Serial.println("Configuring NTP...");
  configTime(0, 0, "pool.ntp.org");

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToMqtt();
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi dropped, trying again...");
    connectToWifi();
  }

  if (!mqttClient.connected())
  {
    Serial.println("MQTT disconnected, reconnecting...");
    connectToMqtt();
  }

  mqttClient.loop();

  unsigned long now = millis();
  if (now - lastLightPublish >= LIGHT_INTERVAL)
  {
    lastLightPublish = now;
    publishLightSensor();
  }

  checkButtonEvents();
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
    Serial.println("Trying to connect to MQTT...");
    String clientId = "lab3_ex1_" + String(random(0xffff), HEX);

    bool connected = mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD);
    if (connected)
    {
      Serial.println("MQTT connected!");
    }
    else
    {
      Serial.print("Failed, rc=");
      Serial.println(mqttClient.state());
      Serial.println("Trying again in 2 seconds");
      delay(2000);
    }
  }
}

void publishLightSensor()
{
  int lightValue = analogRead(LIGHT_SENSOR_PIN);
  time_t now = time(nullptr);

  StaticJsonDocument<128> doc;
  doc["light"] = lightValue;
  doc["timestamp"] = now;

  char payload[128];
  serializeJson(doc, payload, sizeof(payload));

  Serial.print("[MQTT] Publish light: ");
  Serial.println(payload);

  if (!mqttClient.publish(TOPIC_LIGHT, payload))
  {
    Serial.println("Failed to publish light data");
  }
}

void checkButtonEvents()
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

      StaticJsonDocument<128> doc;
      doc["event"] = (buttonState == HIGH) ? "PRESSED" : "RELEASED";
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
  }

  lastButtonReading = reading;
}

