#include <SoftwareSerial.h>
#include <SD.h>
#include "MSP.h"
#include <Servo.h>
Servo Servo1, Servo2;
MSP msp;
File myFile;

//volatile bool ini = false;
const int interruptPin = 2;

//static unsigned long lastTriggerTime = 0;

// Define debounce period in milliseconds
const unsigned long debouncePeriod = 10000;  // Adjust as needed

//int interruptState = 1800;

volatile bool on = false;

void setup() {
  //ini = false;
  // Set up interrupt pin
  pinMode(interruptPin, INPUT);

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
    //ini = !ini;
    return;
  }
  Serial.println("initialization done.");
  //ini = !ini;
  // Attach interrupt to the pin and specify ISR function
  //attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, CHANGE);
}
void loop() {
  //bool ini = true;
  //ini = true;
  //Serial.println("loop");
  //Serial.println(ini);
  msp_attitude_t attitude;
  //msp_rc_t rc;
  msp.request(MSP_ATTITUDE, &attitude, sizeof(msp_attitude_t));
  Serial.println("\n  roll: "+String((map_float(attitude.roll, 180.0, -180.0, 0.0, 180.0) / 4.0) + 70));
  Serial.println("pitch: " + String((map_float(attitude.pitch, 90.0, -90.0, 0.0, 180.0) / 4.0) + 80));
  Serial.println(" Att-Yaw: " + String(attitude.yaw));
  //if (msp.request(MSP_ATTITUDE, &attitude, sizeof(msp_attitude_t)) {
  
  myFile = SD.open("gyro.txt", FILE_WRITE);
    if (myFile) {
      int16_t yaw;
      yaw = attitude.yaw;
      myFile.println("Att-Roll: " + String(attitude.roll/10.0));
      myFile.println(" Att-Pitch: " + String(attitude.pitch/10.0));
      myFile.println(" Att-Yaw: " + String(attitude.yaw));
      myFile.close();
    } else {
      Serial.println("error opening gyro.txt");
    }
  //}
  //msp.request(MSP_RC, &rc, sizeof(rc));
  //uint16_t AUX2 = rc.channelValue[5];
  //uint16_t AUX1 = 2000;
  //AUX2 = 2000;

  /*
  Serial.print("\n");
  Serial.println(pulseIn(6, HIGH,10));
  Servo1.write((map_float(attitude.pitch, 90.0, -90.0, 0.0, 180.0)/4.0)+60);
  Serial.println("\n" + String(attitude.pitch));
  Servo2.write((map_float(attitude.roll, 180.0, -180.0, 0.0, 180.0)/4.0)+75);
  Serial.println(attitude.roll);
*/
  /*
  static unsigned long lastTriggerTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastTriggerTime >= debouncePeriod) {
    attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, CHANGE);
    Serial.println("Interrupt");
    lastTriggerTime = currentTime;
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    
  }*/
  //Serial.println(pulseIn(interruptPin, HIGH));
  /*
  if (pulseIn(interruptPin, HIGH) > 1800) {
    on = true;
  } else {
    on = false;
  }

  if (on == true) {


    Servo1.write((map_float(attitude.pitch, 90.0, -90.0, 0.0, 180.0) / 4.0) + 60);
    Serial.println("\n" + String((map_float(attitude.pitch, 90.0, -90.0, 0.0, 180.0) / 4.0) + 60));
    Servo2.write((map_float(attitude.roll, 180.0, -180.0, 0.0, 180.0) / 4.0) + 75);
    Serial.println((map_float(attitude.roll, 180.0, -180.0, 0.0, 180.0) / 4.0) + 75);
    //Serial.println("gyro pos set");

    //detachInterrupt(digitalPinToInterrupt(interruptPin));
    //setGyroServo();
    //attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, CHANGE);
  } else {
    Servo1.write(90);
    Servo2.write(90);
  }
  */
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Interrupt Service Routine (ISR) function
//void handleInterrupt() {
  // Variables to store last trigger time and interrupt state


  //static unsigned long lastTriggerTime = 0;
  //unsigned long currentTime = millis();



  // Check if it's been longer than the debounce period since the last trigger
  //if (currentTime - lastTriggerTime >= debouncePeriod) {
  // Check the current state of the interrupt pin
  //bool currentState = digitalRead(interruptPin);
  //Serial.println("interrupt");
  /*if (ini) {
      //Serial.println("interrupted");
      // Only process if the state has changed
      
      if ((pulseIn(interruptPin, HIGH) - 1800) > 0) {
        interruptState = currentState;
        if (pulseIn(interruptPin, HIGH) > 1800) {
          on = true;
        } else {
          on = false;
        }
      }
      */
  //if ((pulseIn(interruptPin, HIGH) - 1800) > 0) {
  //interruptState = currentState;
  //const unsigned long timeOut = 1000;
  //if (pulseIn(interruptPin, HIGH) > 1800){
  /*
    if ((currentTime - lastTriggerTime)==1.0){
      Serial.println("do gyro stuff");
      on = true;
    } else {
      on = false;
    }
    // Record the current time as the last trigger time
    lastTriggerTime = currentTime;
    */
//}
/*
void setGyroServo() {

  Serial.println("setting gyro pos");
  msp_attitude_t attitude;
  msp.request(MSP_ATTITUDE, &attitude, sizeof(msp_attitude_t));

  Servo1.write((map_float(attitude.pitch, 90.0, -90.0, 0.0, 180.0) / 4.0) + 60);
  Serial.println("\n" + String((map_float(attitude.pitch, 90.0, -90.0, 0.0, 180.0) / 4.0) + 60));
  Servo2.write((map_float(attitude.roll, 180.0, -180.0, 0.0, 180.0) / 4.0) + 75);
  Serial.println((map_float(attitude.roll, 180.0, -180.0, 0.0, 180.0) / 4.0) + 75);
  Serial.println("gyro pos set");
}*/
