#include "Secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <AccelStepper.h>

// =======================
// Tópicos MQTT AWS
// =======================
#define AWS_IOT_PUBLISH_TOPIC   "esp32prot/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32prot/sub"

// =======================
// I2C Sensores
// =======================
Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft  = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();

Adafruit_MPU6050 mpu;
TwoWire I2CMPU = TwoWire(1); // segundo barramento I2C só para MPU

// =======================
// Flags de sensores
// =======================
bool frontOK = false;
bool leftOK  = false;
bool rightOK = false;
bool mpuOK   = false;

// =======================
// Variáveis de sensores
// =======================
float t = 0;
int distFront = -1, distLeft = -1, distRight = -1;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

// =======================
// AWS IoT
// =======================
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

// =======================
// Motores – 4 A4988
// =======================
// Defina pinos STEP/DIR para cada motor
AccelStepper motor1(AccelStepper::DRIVER, 25, 26); // STEP, DIR
AccelStepper motor2(AccelStepper::DRIVER, 27, 14);
AccelStepper motor3(AccelStepper::DRIVER, 12, 13);
AccelStepper motor4(AccelStepper::DRIVER, 32, 33);

// =======================
// Configuração motores
// =======================
void setupMotors() {
  motor1.setMaxSpeed(800);
  motor1.setAcceleration(400);
  motor2.setMaxSpeed(800);
  motor2.setAcceleration(400);
  motor3.setMaxSpeed(800);
  motor3.setAcceleration(400);
  motor4.setMaxSpeed(800);
  motor4.setAcceleration(400);
}

// =======================
// Funções auxiliares
// =======================
void readSensors() {
  // --- VL53L0X ---
  VL53L0X_RangingMeasurementData_t measure;

  if (frontOK) {
    loxFront.rangingTest(&measure, false);
    distFront = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;
  }
  if (leftOK) {
    loxLeft.rangingTest(&measure, false);
    distLeft = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;
  }
  if (rightOK) {
    loxRight.rangingTest(&measure, false);
    distRight = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;
  }

  // --- MPU6050 ---
  if (mpuOK) {
    sensors_event_t a, g, tempEvent;
    mpu.getEvent(&a, &g, &tempEvent);
    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;
    gx = g.gyro.x;
    gy = g.gyro.y;
    gz = g.gyro.z;
    t = tempEvent.temperature;
  }

  // Debug serial
  Serial.print("Temp: "); Serial.print(t); Serial.print(" C | ");
  Serial.print("Frente: "); Serial.print(distFront); Serial.print(" mm | ");
  Serial.print("Esquerda: "); Serial.print(distLeft); Serial.print(" mm | ");
  Serial.print("Direita: "); Serial.print(distRight); Serial.print(" mm | ");
  Serial.print("Accel[X,Y,Z]: "); Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", "); Serial.print(az); Serial.print(" | ");
  Serial.print("Gyro[X,Y,Z]: "); Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", "); Serial.println(gz);
}

void connectAWS() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  client.setServer(AWS_IOT_ENDPOINT, 8883);
  client.setCallback(messageHandler);

  Serial.println("Connecting to AWS IOT");
  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(500);
  }

  if (!client.connected()) {
    Serial.println("AWS IoT Timeout!");
    return;
  }

  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
  Serial.println("AWS IoT Connected!");
}

void publishMessage() {
  StaticJsonDocument<400> doc;
  doc["temperature"] = t;

  if (frontOK) doc["distance_front"] = distFront; else doc["distance_front"] = "N/A";
  if (leftOK)  doc["distance_left"]  = distLeft;  else doc["distance_left"]  = "N/A";
  if (rightOK) doc["distance_right"] = distRight; else doc["distance_right"] = "N/A";

  if (mpuOK) {
    JsonObject accel = doc.createNestedObject("accel");
    accel["x"] = ax;
    accel["y"] = ay;
    accel["z"] = az;

    JsonObject gyro = doc.createNestedObject("gyro");
    gyro["x"] = gx;
    gyro["y"] = gy;
    gyro["z"] = gz;
  } else {
    doc["accel"] = "N/A";
    doc["gyro"]  = "N/A";
  }

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);

  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("incoming: ");
  Serial.println(topic);

  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}

// =======================
// Setup
// =======================
void setup() {
  Serial.begin(115200);
  connectAWS();

  // Barramento 1: VL53L0X
  Wire.begin(21, 22);
  frontOK = loxFront.begin(0x30, &Wire);
  if (!frontOK) Serial.println("Erro sensor Frente");
  leftOK  = loxLeft.begin(0x31, &Wire);
  if (!leftOK) Serial.println("Erro sensor Esquerda");
  rightOK = loxRight.begin(0x32, &Wire);
  if (!rightOK) Serial.println("Erro sensor Direita");

  // Barramento 2: MPU6050
  I2CMPU.begin(18, 19);
  mpuOK = mpu.begin(MPU6050_I2CADDR_DEFAULT, &I2CMPU);
  if (!mpuOK) Serial.println("Erro MPU6050");

  setupMotors();

  // Exemplo inicial: define posições para motores
  motor1.moveTo(1000);
  motor2.moveTo(1000);
  motor3.moveTo(1000);
  motor4.moveTo(1000);
}

// =======================
// Loop principal
// =======================
void loop() {
  // Motores sempre rodando suave
  motor1.run();
  motor2.run();
  motor3.run();
  motor4.run();

  // Sensores e MQTT a cada 1 s (sem delay)
  static unsigned long lastRead = 0;
  if (millis() - lastRead > 1000) {
    readSensors();
    publishMessage();
    lastRead = millis();
  }

  client.loop();
}