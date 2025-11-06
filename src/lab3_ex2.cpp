#include "Arduino.h"
#include "WiFi.h"
#include <ArduinoJson.h>
#include "PubSubClient.h"

const int RED_LED_PIN = 26;
const int GREEN_LED_PIN = 27;
const int BLUE_LED_PIN = 14;
const int YELLOW_LED_PIN = 12;

const char *WIFI_SSID = "Wokwi-GUEST";
const char *WIFI_PASSWORD = "";

const char *MQTT_HOST = "mqtt.iotserver.uz";
const int MQTT_PORT = 1883;
const char *MQTT_USERNAME = "userTTPU";
const char *MQTT_PASSWORD = "mqttpass";

const char *TOPIC_LED_RED = "ttpu/iot/userTTPU/led/red";
const char *TOPIC_LED_GREEN = "ttpu/iot/userTTPU/led/green";
const char *TOPIC_LED_BLUE = "ttpu/iot/userTTPU/led/blue";
const char *TOPIC_LED_YELLOW = "ttpu/iot/userTTPU/led/yellow";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void connectToWifi();
void connectToMqtt();
void subscribeToTopics();
void handleMqttMessage(char *topic, byte *payload, unsigned int length);
void setLedState(int pin, const char *label, const char *state);

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("Lab 3 - Exercise 2 (beginner version)");

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);

  connectToWifi();

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(handleMqttMessage);
  connectToMqtt();
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi connection lost, reconnecting...");
    connectToWifi();
  }

  if (!mqttClient.connected())
  {
    Serial.println("MQTT connection lost, reconnecting...");
    connectToMqtt();
  }

  mqttClient.loop();
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
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void connectToMqtt()
{
  while (!mqttClient.connected())
  {
    Serial.println("Trying to connect to MQTT...");
    String clientId = "lab3_ex2_" + String(random(0xffff), HEX);

    bool connected = mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD);
    if (connected)
    {
      Serial.println("MQTT connected!");
      subscribeToTopics();
    }
    else
    {
      Serial.print("Failed, rc=");
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

  Serial.println("Subscribed to LED topics:");
  Serial.println(TOPIC_LED_RED);
  Serial.println(TOPIC_LED_GREEN);
  Serial.println(TOPIC_LED_BLUE);
  Serial.println(TOPIC_LED_YELLOW);
}

void handleMqttMessage(char *topic, byte *payload, unsigned int length)
{
  String incoming = "";
  for (unsigned int i = 0; i < length; i++)
  {
    incoming += (char)payload[i];
  }

  Serial.print("[MQTT] Received on ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(incoming);

  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error)
  {
    Serial.print("JSON parse failed: ");
    Serial.println(error.c_str());
    return;
  }

  const char *state = doc["state"];
  if (state == nullptr)
  {
    Serial.println("JSON missing 'state' field");
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
  else
  {
    Serial.println("Unknown topic, ignoring");
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
    Serial.print("Unknown state for ");
    Serial.print(label);
    Serial.print(": ");
    Serial.println(state);
  }
}
