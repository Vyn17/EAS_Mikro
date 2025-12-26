#include <WiFi.h>
#include <PubSubClient.h>

// WiFi
const char* WIFI_SSID     = "L lawliet";
const char* WIFI_PASSWORD = "bangnolep10";

// MQTT
const char* MQTT_HOST = "broker.emqx.io";
const int   MQTT_PORT = 1883;

// 1 tombol trigger
const char* TOPIC_STEP  = "leddd/step";    // kirim '1' setiap kali tombol ditekan
const char* TOPIC_STATE = "leddd/state";   // status

// Pin (yang sudah terbukti jalan)
const int LED_PIN   = 2;
const int MOTOR_IN1 = 27;
const int MOTOR_IN2 = 26;
const int MOTOR_EN  = 12;

// PWM motor
const int PWM_FREQ = 30000;
const int PWM_CH   = 0;
const int PWM_RES  = 8;
int motorSpeed = 205;

// Timer LED/Motor
bool ledActive = false;
unsigned long ledStartMs = 0;
const unsigned long LED_ON_MS = 2000;

bool motorActive = false;
unsigned long motorStartMs = 0;
const unsigned long MOTOR_ON_MS = 1500;

// Counter internal
unsigned long counter = 0;

// Clients
WiFiClient espClient;
PubSubClient mqtt(espClient);

void publishState(const String& s) {
  mqtt.publish(TOPIC_STATE, s.c_str(), true);
  Serial.println("[STATE] " + s);
}

void motorStop() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  ledcWrite(PWM_CH, 0);
}

void motorOn(int speed0_255) {
  speed0_255 = constrain(speed0_255, 0, 255);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  ledcWrite(PWM_CH, speed0_255);
}

void onStepEvent() {
  counter++;
  publishState("STEP=" + String(counter));

  // Kelipatan 3 -> LED 2 detik
  if (counter % 3 == 0) {
    digitalWrite(LED_PIN, HIGH);
    ledActive = true;
    ledStartMs = millis();
    publishState("MULT3: LED_ON");
  }

  // Kelipatan 6 -> Motor 1.5 detik
  if (counter % 6 == 0) {
    motorOn(motorSpeed);
    motorActive = true;
    motorStartMs = millis();
    publishState("MULT6: MOTOR_ON");
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  Serial.printf("[MQTT] Topic: %s | Msg: %s\n", topic, msg.c_str());

  if (String(topic) != TOPIC_STEP) return;

  // tombol kirim '1' sebagai trigger
  if (msg.length() > 0 && msg[0] == '1') {
    onStepEvent();
  } else {
    publishState("IGNORED_PAYLOAD:" + msg);
  }
}

void connectWiFi() {
  Serial.print("Connecting WiFi ");
  Serial.print(WIFI_SSID);

  WiFi.disconnect(true, true);
  delay(300);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());
}

void connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  while (!mqtt.connected()) {
    String clientId = "iMCLab-step-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.print("Connecting MQTT as ");
    Serial.println(clientId);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("MQTT connected.");
      mqtt.subscribe(TOPIC_STEP);
      publishState("ONLINE: press button -> send 1 to leddd/step");
    } else {
      Serial.print("MQTT failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retry in 1s");
      delay(1000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_EN, PWM_CH);

  motorStop();

  connectWiFi();
  connectMQTT();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  if (ledActive && millis() - ledStartMs >= LED_ON_MS) {
    digitalWrite(LED_PIN, LOW);
    ledActive = false;
    publishState("LED_OFF");
  }

  if (motorActive && millis() - motorStartMs >= MOTOR_ON_MS) {
    motorStop();
    motorActive = false;
    publishState("MOTOR_OFF");
  }
}
