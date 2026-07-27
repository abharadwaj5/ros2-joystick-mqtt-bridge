//Sketch ESP32 - MQTT - ROS

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* SSID   = "Wokwi-GUEST";        // sim access pt
const char* PASS   = "";
//const char* BROKER = "test.mosquitto.org"; //
const char* BROKER = "broker.hivemq.com";
const int   PORT   = 1883;

const char* T_CMD   = "remote_robot/robotthinkit/cmd";
const char* T_HB    = "remote_robot/robotthinkit/esp32_heartbeat";
const char* T_STATE = "remote_robot/robotthinkit/state";

const int PIN_LIN = 34;    // Analog ip on ADC1 
const int PIN_ANG = 35;
const int PIN_DEADMAN = 25;
const int PIN_LED = 26;

const float MAX_LIN = 0.5;   // clamp limits
const float MAX_ANG = 1.0;

WiFiClient net;
PubSubClient mqtt(net);

unsigned long lastCmd = 0, lastHb = 0, lastRetry = 0, lastState = 0;
uint32_t seq = 0;
bool robotFallback = true;

// Pot reading (0..4095) -> velocity. Centre is 2048, so subtracting it
// gives negative for reverse and positive for forward. Small readings
// near the centre are ignored
float readAxis(int pin, float maxVal) {
  int raw = analogRead(pin);
  int centred = raw - 2048;
  if (abs(centred) < 150) return 0.0;
  return (centred / 2048.0) * maxVal;
}

void onMessage(char* topic, byte* payload, unsigned int len) {
  JsonDocument doc;
  if (deserializeJson(doc, payload, len)) {
    Serial.println("[state] bad JSON");
    return;
  }
  robotFallback = doc["fallback_active"] | true;
  lastState = millis();

  Serial.print("[state] mode=");
  Serial.print((const char*)(doc["mode"] | "?"));
  Serial.print(" reason=");
  Serial.print((const char*)(doc["reason"] | "?"));
  Serial.print(" cmd_age_ms=");
  Serial.println((long)(doc["last_cmd_age_ms"] | -1));
}

void connectWifi() {
  Serial.print("WiFi connecting");
  WiFi.begin(SSID, PASS, 6);          // channel 6 speeds up Wokwi
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print("."); }
  Serial.print(" ok, IP "); Serial.println(WiFi.localIP());
}

void connectMqtt() {
  // Random id: duplicate client ids get evicted by the broker.
  String id = "esp32-op-" + String(random(0xffff), HEX);
  Serial.print("MQTT connecting as "); Serial.print(id);
  if (mqtt.connect(id.c_str())) {
    Serial.println(" ok");
    mqtt.subscribe(T_STATE);
  } else {
    Serial.print(" failed rc="); Serial.println(mqtt.state());
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_DEADMAN, INPUT_PULLUP);   // pressed = LOW
  pinMode(PIN_LED, OUTPUT);
  connectWifi();
  mqtt.setServer(BROKER, PORT);
  mqtt.setCallback(onMessage);
  connectMqtt();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) { connectWifi(); return; }

  if (!mqtt.connected()) {
    if (millis() - lastRetry > 2000) { lastRetry = millis(); connectMqtt(); }
  } else {
    mqtt.loop();

    // command @ 10 Hz - well inside the robot's 1 s timeout
    if (millis() - lastCmd >= 100) {
      lastCmd = millis();
      JsonDocument d;
      d["linear_x"]  = readAxis(PIN_LIN, MAX_LIN);
      d["angular_z"] = readAxis(PIN_ANG, MAX_ANG);
      d["deadman"]   = (digitalRead(PIN_DEADMAN) == LOW);
      d["seq"]       = seq++;
      char buf[160];
      size_t n = serializeJson(d, buf);
      mqtt.publish(T_CMD, (const uint8_t*)buf, n, false);
      if (seq % 10 == 0) { Serial.print("[cmd] "); Serial.println(buf); }
    }

    // heartbeat @ 4 Hz
    if (millis() - lastHb >= 250) {
      lastHb = millis();
      JsonDocument d;
      d["alive"]     = true;
      d["seq"]       = seq;
      d["uptime_ms"] = millis();
      char buf[128];
      size_t n = serializeJson(d, buf);
      mqtt.publish(T_HB, (const uint8_t*)buf, n, false);
    }
  }

  // LED on only if the robot recently confirmed it's obeying us
  bool fresh = (millis() - lastState) < 2000;
  digitalWrite(PIN_LED, (mqtt.connected() && fresh && !robotFallback) ? HIGH : LOW);
}