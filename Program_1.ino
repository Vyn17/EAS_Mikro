#include <WiFi.h>
#include <PubSubClient.h>

// =====================
// WIFI & MQTT
// =====================
const char* WIFI_SSID     = "L lawliet";
const char* WIFI_PASSWORD = "bangnolep10";

const char* MQTT_HOST = "broker.emqx.io";
const int   MQTT_PORT = 1883;

// Kirim 1/0 ke topic ini
const char* TOPIC_CMD   = "leddd";
// ESP32 publish status ke sini (opsional, buat laporan enak)
const char* TOPIC_STATE = "leddd";

// =====================
// PIN (SAMAKAN DENGAN KODE LAMA)
// =====================
const int LED_PIN   = 2;

// Ini yang terbukti jalan di kode lama kamu:
const int MOTOR_IN1 = 27;
const int MOTOR_IN2 = 26;
const int MOTOR_EN  = 12;   // enable PWM

// PWM (samakan juga)
const int PWM_FREQ    = 30000;
const int PWM_CH      = 0;
const int PWM_RES_BIT = 8;      // 0..255
int motorSpeed = 205;

// =====================
// STATE
// =====================
enum SystemState { IDLE, LED_ON_WAIT, MOTOR_RUNNING };
SystemState state = IDLE;

unsigned long ledOnStartMs = 0;
const unsigned long LED_ON_DURATION_MS = 5000;

// =====================
// CLIENTS
// =====================
WiFiClient espClient;
PubSubClient mqtt(espClient);

// =====================
// MOTOR CONTROL
// =====================
void motorStop() {
  // di kode lama kamu IN1 tetap HIGH, IN2 LOW, PWM=0
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  ledcWrite(PWM_CH, 0);
}

void motorForward(int speed0_255) {
  speed0_255 = constrain(speed0_255, 0, 255);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  ledcWrite(PWM_CH, speed0_255);
}

// =====================
// MQTT HELPERS
// =====================
void publishState(const String& s) {
  mqtt.publish(TOPIC_STATE, s.c_str(), true);
  Serial.println("[STATE] " + s);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  Serial.print("[MQTT] Topic: ");
  Serial.print(topic);
  Serial.print(" | Msg: ");
  Serial.println(msg);

  if (String(topic) != TOPIC_CMD) return;

  // payload '1' / '0'
  char c = msg.length() ? msg[0] : '\0';

  if (c == '1') {
    // START: LED ON 5 detik, lalu motor ON
    motorStop();
    digitalWrite(LED_PIN, HIGH);
    ledOnStartMs = millis();
    state = LED_ON_WAIT;
    publishState("CMD=1: LED_ON_WAIT_5S");
  } else if (c == '0') {
    // STOP
    digitalWrite(LED_PIN, LOW);
    motorStop();
    state = IDLE;
    publishState("CMD=0: STOPPED_IDLE");
  } else {
    publishState("CMD_INVALID:" + msg);
  }
}

// =====================
// CONNECT
// =====================
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
    String clientId = "iMCLab-seq-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.print("Connecting MQTT as ");
    Serial.println(clientId);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("MQTT connected.");
      mqtt.subscribe(TOPIC_CMD);
      publishState("ONLINE: IDLE (send 1=START, 0=STOP)");
    } else {
      Serial.print("MQTT failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retry in 1s");
      delay(1000);
    }
  }
}

// =====================
// SETUP & LOOP
// =====================
void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // PWM init (samakan dengan kode lama)
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES_BIT);
  ledcAttachPin(MOTOR_EN, PWM_CH);

  motorStop();

  connectWiFi();
  connectMQTT();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  // LED 5 detik -> motor jalan
  if (state == LED_ON_WAIT) {
    if (millis() - ledOnStartMs >= LED_ON_DURATION_MS) {
      digitalWrite(LED_PIN, LOW);
      motorForward(motorSpeed);
      state = MOTOR_RUNNING;
      publishState("LED_DONE: MOTOR_RUNNING_SPEED=" + String(motorSpeed));
    }
  }
}
