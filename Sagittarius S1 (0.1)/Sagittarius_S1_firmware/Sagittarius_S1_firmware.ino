/*
   @file    LSM6DSV16X_Sensor_Fusion.ino
   @author  STMicroelectronics
   @brief   Example to use the LSM6DSV16X library with Sensor Fusion Low Power.
 *******************************************************************************
   Copyright (c) 2022, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/

/*
 * You can display the quaternion values with a 3D model connecting for example to this link:
 * https://adafruit.github.io/Adafruit_WebSerial_3DModelViewer/
 */

#include <LSM6DSV16XSensor.h>
#include <Wire.h>

#include <WiFi.h>
#include <WiFiUdp.h>

#define CS 8
#define SCL 7
#define SDA 6
#define EN 2
#define AS 5

#define ALGO_FREQ  120U /* Algorithm frequency 120Hz */
#define ALGO_PERIOD  (1000U / ALGO_FREQ) /* Algorithm period [ms] */
unsigned long startTime, elapsedTime;

LSM6DSV16XSensor AccGyr(&Wire);
uint8_t status = 0;
uint32_t k = 0;

uint8_t tag = 0;
float quaternions[4] = {0};

struct data {
  double positionX = 0.0;
  double positionY = 0.0;
  double positionZ = 0.0;
  double rotationX = 0.0;
  double rotationY = 0.0;
  double rotationZ = 0.0;
} data;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

float currentYaw, currentPitch, currentRoll;
float lastPitch = 0.00, lastYaw = 0.00, lastRoll = 0.00;

//---------------------------------------

// WiFi network name and password:
const char * networkName = "SSID";
const char * networkPswd = "Password";

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "target computer IP";
const int udpPort = 4747;

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;

unsigned long startup;
bool started = false;

void setup()
{
  // Set up pins
  pinMode(CS, OUTPUT); // Chip Select pin on IMU is used to enable I2C
  pinMode(EN, OUTPUT); // EN pin on 1.8V LDO, used to turn on the IMU after the pins have been set correctly
  pinMode(AS, OUTPUT); // Address Select, HIGH = least significant bit of the 7-bit address is 1 (0x6b), LOW = 0 (0x6a). Connects to the SDO pin on the IMU.

  digitalWrite(CS, HIGH); // Turn on I2C
  digitalWrite(AS, HIGH); // Set address to 0x6b
  digitalWrite(EN, HIGH); // Turn on IMU

  delay(50);

  //Serial.begin(115200);

  delay(500);

  //WiFi setup
  //Serial.println("Connecting to network");
  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);

  //Serial.println("Starting I2C");
  Wire.begin(SDA, SCL);

  //Serial.println("Starting IMU");
  // Initialize LSM6DSV16X.
  AccGyr.begin();

  //Serial.println("Enabling SFLP");
  // Enable Sensor Fusion
  status |= AccGyr.Enable_Rotation_Vector();

  if (status != LSM6DSV16X_OK) {
    //Serial.println("LSM6DSV16X Sensor failed to init/configure");
    while (1);
  }
  //Serial.println("LSM6DSV16X SFLP Demo");

  startup = millis();
}

void loop()
{
  uint16_t fifo_samples;
  // Get start time of loop cycle
  startTime = millis();

  // Check the number of samples inside FIFO
  if (AccGyr.FIFO_Get_Num_Samples(&fifo_samples) != LSM6DSV16X_OK) {
    //Serial.println("LSM6DSV16X Sensor failed to get number of samples inside FIFO");
    while (1);
  }

  // Read the FIFO if there is one stored sample
  if (fifo_samples > 0) {
    for (int i = 0; i < fifo_samples; i++) {
      //Serial.println("Checking FIFO tag");
      AccGyr.FIFO_Get_Tag(&tag);
      if (tag == 0x13u) {
        //Serial.println("Getting quaternions");
        AccGyr.FIFO_Get_Rotation_Vector(&quaternions[0]);

        // Print Quaternion data
        /*Serial.print("Quaternion: ");
        Serial.print(quaternions[3], 4);
        Serial.print(", ");
        Serial.print(quaternions[0], 4);
        Serial.print(", ");
        Serial.print(quaternions[1], 4);
        Serial.print(", ");
        Serial.println(quaternions[2], 4);*/

        data.rotationX = currentYaw;
        data.rotationY = currentPitch;
        data.rotationZ = currentRoll;

        uint8_t ByteArray[sizeof(data)];
        memcpy(ByteArray, &data, sizeof(data));
        for (int i = 0; i<sizeof(ByteArray); i++) {
          Serial.print(ByteArray[i], HEX);
        }
        Serial.println();

        quaternionToEuler(quaternions[3], quaternions[0], quaternions[1], quaternions[2]);

        udp.beginPacket(udpAddress,udpPort);
        udp.write(ByteArray, sizeof(ByteArray));
        udp.endPacket();

        // Compute the elapsed time within loop cycle and wait
        elapsedTime = millis() - startTime;

        if ((long)(ALGO_PERIOD - elapsedTime) > 0) {
          delay(ALGO_PERIOD - elapsedTime);
        }

        /*if (millis() - startup > 10000 && !started) {
          currentPitch = 0;
          currentYaw = 0;
          currentRoll = 0;
          lastPitch = 0;
          lastYaw = 0;
          lastRoll = 0;
          started = true;
          Serial.println("Homing...");
        }*/
      }
    }
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    float pitch = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    float roll = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    float yaw = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    pitch *= RAD_TO_DEG;
    roll *= RAD_TO_DEG;
    yaw *= RAD_TO_DEG;

    float dPitch = pitch - lastPitch;
    float dRoll = roll - lastRoll;
    float dYaw = yaw - lastYaw;

    dPitch = fmod(dPitch + 540, 360) -180;
    dRoll = fmod(dRoll + 540, 360) -180;
    dYaw = fmod(dYaw + 540, 360) -180;

    currentPitch += dPitch;
    currentRoll += dRoll;
    currentYaw += dYaw;

    //Serial.print("Pitch out: "); Serial.print(currentPitch); Serial.print("  dPitch: "); Serial.print(dPitch, 5); Serial.print("  lastPitch: "); Serial.print(lastPitch); Serial.print("  currentPitch: "); Serial.println(pitch);

    lastPitch = pitch;
    lastRoll = roll;
    lastYaw = yaw;
}

void connectToWiFi(const char * ssid, const char * pwd){
  //Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  //Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          //When connected set 
          //Serial.print("WiFi connected! IP address: ");
          //Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
          //Serial.println("WiFi lost connection");
          connected = false;
          break;
      default: break;
    }
}