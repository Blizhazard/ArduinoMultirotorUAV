#include <SoftwareSerial.h>
#include <SD.h>
#include "MSP.h"
#include <Servo.h>
Servo Servo1, Servo2;
MSP msp;
File myFile;

const int interruptPin = 2;
bool iniSD = false;
volatile bool on = false;

void setup() {
  // Set up interrupt pin
  pinMode(interruptPin, INPUT);

  Serial.begin(115200);
  msp.begin(Serial);
  pinMode(10, OUTPUT);
  pinMode(6, INPUT);
  Servo1.attach(4);
  Servo1.write(180);
  delay(1000);
  Servo1.write(0);
  delay(1000);
  Servo1.write(90);
  Servo2.attach(5);
  Servo2.write(180);
  delay(1000);
  Servo2.write(0);
  delay(1000);
  Servo2.write(90);
  
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    //ini = !ini;
    return;
  }
  Serial.println("initialization done.");
  
}
void loop() {
  
  /*
  while (!iniSD) {
    Serial.println("initialization failed!");
    Serial.println(SD.exists("gyro.txt"));
    delay(1000);
    Serial.println(digitalRead(10));
    if (SD.begin(10)) {
      Serial.println("breakout");
      iniSD = true;
      break;
    }
  }*/
  msp_attitude_t attitude;
  msp.request(MSP_ATTITUDE, &attitude, sizeof(msp_attitude_t));
  //Serial.println("\n  roll: "+String((map_float(attitude.roll, 180.0, -180.0, 0.0, 180.0) / 4.0) + 66.62));
  //Serial.println("pitch: " + String((map_float(attitude.pitch, 90.0, -90.0, 0.0, 180.0) / 4.0) + 62.5));
  //Serial.println(" Att-Yaw: " + String(attitude.yaw));
  //Serial.println(msp.request(MSP_ATTITUDE, &attitude, sizeof(msp_attitude_t)));
  myFile = SD.open("gyro.txt", FILE_WRITE);
  if (myFile) {
    int16_t yaw;
    yaw = attitude.yaw;
    myFile.println("Att-Roll: " + String(attitude.roll / 4.0));
    myFile.println(" Att-Pitch: " + String(attitude.pitch / 4.0));
    myFile.println(" Att-Yaw: " + String(yaw));
    myFile.close();
  } else {
    Serial.println("error opening gyro.txt");
    SD.begin(10);
  }
  
  //remove the  ",1" the line below
  if (pulseIn(interruptPin, HIGH,1) > 1800) {
    on = true;
  } else {
    on = false;
  }
  //comment line below 
  on = true;
  if (on == true) {


    Servo1.write((map_float(attitude.pitch, 90.0, -90.0, 0.0, 180.0) / 4.0) + 66.62);
    Serial.println("\n" + String((map_float(attitude.pitch, 90.0, -90.0, 0.0, 180.0) / 4.0) + 66.62));
    Servo2.write((map_float(attitude.roll, 180.0, -180.0, 0.0, 180.0) / 4.0) + 62.5);
    Serial.println((map_float(attitude.roll, 180.0, -180.0, 0.0, 180.0) / 4.0) + 62.5);
  } else {
    Servo1.write(90);
    Servo2.write(90);
  }
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}