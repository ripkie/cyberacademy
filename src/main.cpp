#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include <OneWire.h>
#include <DallasTemperature.h>

// =========================
// PIN DEFINITIONS
// =========================

// DS18B20
#define ONE_WIRE_BUS 32 // <-- PIN DATA DS18B20

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// HC-SR04
#define TRIG_PIN 5
#define ECHO_PIN 18

// =========================
// WIFI + MQTT SETUP
// =========================

const char *ssid = "Wokwi-GUEST";
const char *password = "";
const char *mqtt_server = "test.mosquitto.org";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;

// =========================
// WIFI FUNCTION
// =========================

void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// =========================
// MQTT CALLBACK
// =========================

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
}

// =========================
// RECONNECT MQTT
// =========================

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("\nAttempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str()))
    {
      Serial.println("Connected");
      client.publish("/cyberacademy/status", "Sensor Online");
      client.subscribe("/cyberacademy/control");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// =========================
// HC-SR04 FUNCTION
// =========================
float getDistance()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  float duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;
  return distance;
}

// =========================
// SETUP
// =========================

void setup()
{
  Serial.begin(115200);

  // HC-SR04 pin setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  sensors.begin(); // start DS18B20

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

// =========================
// LOOP
// =========================

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();

  if (now - lastMsg > 2000)
  {
    lastMsg = now;

    // --- Read DS18B20 ---
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);

    // --- Read Ultrasonic ---
    float distance = getDistance();

    // Publish MQTT
    client.publish("/cyberacademy/temperature", String(tempC).c_str());
    client.publish("/cyberacademy/distance", String(distance).c_str());

    // LOCAL SERIAL OUTPUT
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println("°C");

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
}
