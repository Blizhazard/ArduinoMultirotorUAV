#include <SoftwareSerial.h>
#include <SD.h>
//#include "MSP.h"
#include <Servo.h>
#define MSP_HEADER '$'
#define MSP_IDENT 100
#define MSP_ATTITUDE 108

Servo Servo1, Servo2;
//MSP msp;
File myFile;
void setup() {
  Serial.begin(115200);
  //msp.begin(Serial);
  pinMode(10, OUTPUT);
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

  uint8_t datad = 0;
  uint8_t *data = &datad;

  sendMSP(MSP_ATTITUDE, data, 0);



  // Request attitude data from the flight controller
  //sendMSPRequest(MSP_ATTITUDE);

  // Wait for response
  delay(100);  // Adjust delay as necessary
  byte count = 0;

  int16_t roll;
  int16_t pitch;
  int16_t yaw;

  // Read response from flight controller
  while (Serial.available()) {
    count += 1;
    byte c = Serial.read();
    switch (count) {
      case 6:
        roll = c;
        break;
      case 7:
        roll <<= 8;
        roll += c;
        roll = (roll & 0xFF00) >> 8 | (roll & 0x00FF) << 8;  // Reverse the order of bytes
        break;
      case 8:
        pitch += c;
        break;
      case 9:
        pitch <<= 8;
        pitch += c;
        pitch = (pitch & 0xFF00) >> 8 | (pitch & 0x00FF) << 8;  // Reverse the order of bytes
        break;
      case 10:
        yaw += c;
        break;
      case 11:
        yaw <<= 8;
        yaw += c;
        yaw = (yaw & 0xFF00) >> 8 | (yaw & 0x00FF) << 8;  // Reverse the order of bytes
        break;
    }
  }

  //msp_attitude_t attitude;
  //msp_rc_t rc;
  //msp.request(MSP_ATTITUDE, &attitude, sizeof(msp_attitude_t));
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
  uint16_t AUX2 = 2000;  //rc.channelValue[5];
  //uint16_t AUX1 = 2000;
  AUX2 = 2000;

  if (AUX2 >= 1110) {
    //Servo1.write((map_float(attitude.pitch, 90.0, -90.0, 0.0, 180.0) / 4.0) + 60);
    //Serial.println("\n" + String(attitude.pitch));
    //Servo2.write((map_float(attitude.roll, 180.0, -180.0, 0.0, 180.0) / 4.0) + 75);
    //Serial.println(attitude.roll);
    //float prevPitch;
    //float prevRoll;
    //if (prevPitch - )
    Serial.println("Roll: " + String(((roll / 10.0) + 180) / 2));
    Serial.println(" Pitch: " + String((pitch / 10.0) + 90));
    Serial.println(" Yaw: " + String(yaw));
    Servo1.write((pitch / 10.0) + 90);
    Servo2.write(((roll / 10.0) + 180) / 2);
    //float prevPitch = (pitch/10.0)+90;
    //float prevRoll = ((roll/10.0)+180)/2;

  } else {
    Servo1.write(90);
    Servo2.write(90);
  }
}

void sendMSP(uint8_t cmd, uint8_t *data, uint8_t n_bytes) {

  uint8_t checksum = 0;

  Serial.write((byte *)"$M<", 3);
  Serial.write(n_bytes);
  checksum ^= n_bytes;

  Serial.write(cmd);
  checksum ^= cmd;

  Serial.write(checksum);
}


float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}