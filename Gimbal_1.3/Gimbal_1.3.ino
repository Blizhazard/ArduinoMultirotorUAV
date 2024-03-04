#include <SoftwareSerial.h>
#include <SD.h>
#include "MSP.h"
#include <Servo.h>
Servo Servo1, Servo2;
MSP msp;
File myFile;
void setup() {
  Serial.begin(115200);
  msp.begin(Serial);
  pinMode(10, OUTPUT);
  pinMode(6, INPUT);
  Servo1.attach(3);
  Servo1.write(90);
  Servo2.attach(5);
  Servo2.write(90);
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
}
void loop() {
  msp_attitude_t attitude;
  msp_rc_t rc;
  msp.request(MSP_ATTITUDE, &attitude, sizeof(msp_attitude_t));

  //if (msp.request(MSP_ATTITUDE, &attitude, sizeof(msp_attitude_t)) {
    /*myFile = SD.open("gyro.txt", FILE_WRITE);
    if (myFile) {
      int16_t yaw;
      yaw = attitude.yaw;
      myFile.println("Att-Roll: " + String(roll/10.0));
      myFile.println(" Att-Pitch: " + String(pitch/10.0));
      myFile.println(" Att-Yaw: " + String(yaw));
      myFile.close();
    } else {
      Serial.println("error opening gyro.txt");
    }*/
  //}
  //msp.request(MSP_RC, &rc, sizeof(rc));
  //uint16_t AUX2 = rc.channelValue[5];
  //uint16_t AUX1 = 2000;
  //AUX2 = 2000;
  Serial.print("\n");
  Serial.println(pulseIn(6, HIGH,10));
  Servo1.write((map_float(attitude.pitch, 90.0, -90.0, 0.0, 180.0)/4.0)+60);
  Serial.println("\n" + String(attitude.pitch));
  Servo2.write((map_float(attitude.roll, 180.0, -180.0, 0.0, 180.0)/4.0)+75);
  Serial.println(attitude.roll);
  /*if (AUX2 >= 1110) {
    Servo1.write((map_float(attitude.pitch, 90.0, -90.0, 0.0, 180.0)/4.0)+60);
    Serial.println("\n" + String(attitude.pitch));
    Servo2.write((map_float(attitude.roll, 180.0, -180.0, 0.0, 180.0)/4.0)+75);
    Serial.println(attitude.roll);
  } else {
    Servo1.write(90);
    Servo2.write(90);
  }*/
}
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}