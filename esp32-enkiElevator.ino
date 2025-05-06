//Project Components
/*
//Controller
ESP32 with Expansor Shield Module
Power Supply 12v 

//Sensors
Reed Switch Magnetic Sensors (Intelbras Xas) - 4 units - [3.3V + ESP GPIO] -> 0 or 1
DHT11 temperature & humidity Sensor - [Pin 1: 3.3V + Pin 2: ESP GPIO + Pin 3: None + Pin 4: GND] -> Temp in ºC
ACS712 current sensor (20A) - [VCC: 5V + GND: GND + OUT: GPIO 2 (with resistors)]
MPU-6050 Accelerometer & Gyroscope - [VCC: 3.3V + GND: GND + Serial Clock(SCL): GPIO(22) + Serial Data (SDA): GPIO(21)]

//Engine
12v DC engine 100rpm with gearbox
*/

//Libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "DHT.h"

//ESP32 Pinage setup
const int magneticSensorPin = 23;   // Digital output for Magnetic Reed Switch Sensor
const int currentSensorPin = 2;    // Analog output for Current Sensor
const int temperatureSensorPin = 4; // Analog output for DHT Sensor 

//Accelerometer Global Variables
Adafruit_MPU6050 mpu;

//Temperature Global Variables
#define DHTTYPE DHT11
DHT dht(temperatureSensorPin, DHTTYPE);

//Current Sensor Global Variables
float R1 = 6800.0;
float R2 = 12000.0;

void setup() {
  Serial.begin(115200);
  pinMode(magneticSensorPin, INPUT_PULLDOWN); //Reed Switch Sensor
  pinMode(currentSensorPin, INPUT);           //ACS712 Sensor
  dht.begin();                                //DHT Temperature sensor initializing
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
  int magneticSensorValue = digitalRead(magneticSensorPin);
  Serial.print("Magnetic Sensor Value: ");
  Serial.println(magneticSensorValue);

  //Current Sensor Portion
  int adc = analogRead(currentSensorPin);
  Serial.print("Current Analog Value: ");
  Serial.println(adc);
  float adc_voltage = adc * (3.3 / 4096.0);
  float current_voltage = (adc_voltage * (R1+R2)/R2);
  float current = (current_voltage - 2.5) / 0.100;
  Serial.print("Current Value: ");
  Serial.println(current);

  //Temperature Portion
  float t = dht.readTemperature();
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

  delay(8000);
}

