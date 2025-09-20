//Project Components
/*
//Controller
ESP32 with Expansor Shield Module
Power Supply 12v 3A

//Sensors
Reed Switch Magnetic Sensors (Intelbras Xas) - 4 units - [3.3V + ESP GPIO] -> 0 or 1
DHT11 temperature & humidity Sensor - [Pin 1: 3.3V + Pin 2: ESP GPIO + Pin 3: None + Pin 4: GND] -> Temp in ºC
ACS712 current sensor (20A) - [VCC: 5V + GND: GND + OUT: GPIO 2 (with resistors)]
MPU-6050 Accelerometer & Gyroscope - [VCC: 3.3V + GND: GND + Serial Clock(SCL): GPIO(22) + Serial Data (SDA): GPIO(21)]

//Engine
12v DC engine 100rpm with gearbox
*/

//Libraries
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "DHT.h"

#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

//Pinage setup
//Reed Switches
#define magneticSensorPin1stFloor 23   // Digital output for Magnetic Reed Switch Sensor 1st floor
#define magneticSensorPin2ndFloor 19   // Digital output for Magnetic Reed Switch Sensor 2nd floor
#define magneticSensorPin3rdFloor 18   // Digital output for Magnetic Reed Switch Sensor 3rd floor
#define magneticSensorPin4thFloor 5   // Digital output for Magnetic Reed Switch Sensor 4th floor

//Temperature
#define temperatureSensorPin 4        // Analog output for DHT Sensor
#define DHTTYPE DHT11
float t;
float tcabine;
DHT dht(temperatureSensorPin, DHTTYPE);

//Current
#define currentSensorPin 34          // Analog output for Current Sensor
float current = 0;
const float sensitivity = 0.100; // 100 mV/A (20A)
const float Vref = 3.3;          // Tensão de referência ADC ESP32
int offsetRaw = 0;               // Offset calibrado (ADC)
float offsetVoltage = 0;         // Offset calibrado (Volts)

//Engine and L298N
#define IN1 25                 // L298N In 1
#define IN2 26                 // L298N In 2
#define ENA 27                 // PWM

//Buttons
const int BUTTON_1ST = 32;          // 1st floor action button
const int BUTTON_2ND = 33;          // 2nd floor action button
const int BUTTON_3RD = 12;          // 3rd floor action button
const int BUTTON_4TH = 14;          // 4th floor action button

//Accelerometer
Adafruit_MPU6050 mpu;
float accelerometerX;
float accelerometerY;
float accelerometerZ;
float gyroscopeX;
float gyroscopeY;
float gyroscopeZ;

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

void connectAWS(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting to Wi-Fi");
 
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
 
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
 
  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);
 
  // Create a message handler
  client.setCallback(messageHandler);
  Serial.println("Connecting to AWS IOT");
 
  while (!client.connect(THINGNAME)){
    Serial.print(".");
    delay(500);
  }
 
  if (!client.connected()){
    Serial.println("AWS IoT Timeout!");
    return;
  }
  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
  Serial.println("AWS IoT Connected!");
}

void publishMessage(){
  StaticJsonDocument<200> doc;
  doc["temperature"] = t;
  doc["cabineTemperature"] = tcabine;
  doc["current"] = current;
  JsonObject accelerometer = doc.createNestedObject("accelerometer");
  accelerometer["x"] = accelerometerX;
  accelerometer["y"] = accelerometerY;
  accelerometer["z"] = accelerometerZ;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client
 
  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}
 
void messageHandler(char* topic, byte* payload, unsigned int length){
  Serial.print("incoming: ");
  Serial.println(topic);
 
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);                       //H bridge
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH);
  pinMode(BUTTON_1ST, INPUT_PULLUP);          //Buttons
  pinMode(BUTTON_2ND, INPUT_PULLUP);
  pinMode(BUTTON_3RD, INPUT_PULLUP);
  pinMode(BUTTON_4TH, INPUT_PULLUP);
  pinMode(magneticSensorPin1stFloor, INPUT_PULLUP);          //Reed Switch Sensor
  pinMode(magneticSensorPin2ndFloor, INPUT_PULLUP);
  pinMode(magneticSensorPin3rdFloor, INPUT_PULLUP);
  pinMode(magneticSensorPin4thFloor, INPUT_PULLUP);

  pinMode(currentSensorPin, INPUT);           //ACS712 Sensor
  dht.begin();                                //DHT Temperature sensor initializing
  connectAWS();
  // --- Calibração do ACS712 ---
  long soma = 0;
  int n = 500;  // número de amostras para calibrar
  Serial.println("Calibrando ACS712...");
  for (int i = 0; i < n; i++) {
    soma += analogRead(currentSensorPin);
    delay(2);
  }
  offsetRaw = soma / n;
  offsetVoltage = (offsetRaw / 4095.0) * Vref;
  Serial.print("Offset ADC: ");
  Serial.print(offsetRaw);
  Serial.print(" (");
  Serial.print(offsetVoltage);
  Serial.println(" V)");
  Serial.println("Calibração concluída");
  if (!mpu.begin()) Serial.println("Failed to find MPU6050 chip"); //Accelerometer initializing
  else Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); //Set the accelerometer measurement range
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); //Set the gyroscope measurement range
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ); //Set the filter bandwidth
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

void loop() {
  //Magnetic Sensors Portion
  int sensor1 = digitalRead(magneticSensorPin1stFloor);
  int sensor2 = digitalRead(magneticSensorPin2ndFloor);
  int sensor3 = digitalRead(magneticSensorPin3rdFloor);
  int sensor4 = digitalRead(magneticSensorPin4thFloor);

  Serial.print("Andar 1: "); Serial.print(sensor1);
  Serial.print(" | Andar 2: "); Serial.print(sensor2);
  Serial.print(" | Andar 3: "); Serial.print(sensor3);
  Serial.print(" | Andar 4: "); Serial.println(sensor4);


  //Temperature Portion
  t = dht.readTemperature();
  if (isnan(t)) Serial.println(F("Failed to read from DHT sensor!"));

  Serial.print(F("Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.println("");

  //Accelerometer Portion
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
  
  Serial.println("==============#==============");

  //Corrente
  int raw = analogRead(currentSensorPin);
  Serial.print("Current Pin Analog Read: ");
  Serial.println(raw);
  float voltage = (raw / 4095.0) * Vref;
  float voltageDiff = voltage - offsetVoltage; // subtrai offset
  current = voltageDiff / sensitivity; // calcula corrente (A)

  Serial.print("Corrente: ");
  Serial.print(current, 3);
  Serial.println("A");

  bool btn1Pressed = digitalRead(BUTTON_1ST) == LOW;
  bool btn2Pressed = digitalRead(BUTTON_2ND) == LOW;
  bool btn3Pressed = digitalRead(BUTTON_3RD) == LOW;
  bool btn4Pressed = digitalRead(BUTTON_4TH) == LOW;

  if(btn1Pressed) {
    Serial.println("Button 1: Pressed");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);  // clockwise direction
  }
  else if(btn2Pressed) {
    Serial.println("Button 2: Pressed");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH); // anti clockwise direction
  }
  else if(btn3Pressed) {
    Serial.println("Button 3: Pressed");
  }
  else if(btn4Pressed) {
    Serial.println("Button 4: Pressed");
  }
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);  // Stopped engine
  }
  publishMessage();
  client.loop();
  delay(2000);
}
