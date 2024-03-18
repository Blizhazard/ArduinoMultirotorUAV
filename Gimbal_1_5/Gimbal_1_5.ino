#include <SoftwareSerial.h>
#include <SD.h>
#include "MSP.h"
#include <Servo.h>
Servo Servo1, Servo2;
MSP msp;
File myFile;

volatile bool ini = false;
const int interruptPin = 2;

// Define debounce period in milliseconds
const unsigned long debouncePeriod = 50;  // Adjust as needed
// Variables to store last trigger time and interrupt state
volatile unsigned long lastTriggerTime = 0;
int interruptState = 1800;

volatile bool on = false;

void setup() {
  ini = false;
  // Set up interrupt pin
  pinMode(interruptPin, INPUT);

  // Attach interrupt to the pin and specify ISR function
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, CHANGE);

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
    ini = !ini;
    return;
  }
  Serial.println("initialization done.");
  ini = !ini;
}
void loop() {
  //bool ini = true;
  //ini = true;
  //Serial.println("loop");
  //Serial.println(ini);
  msp_attitude_t attitude;
  //msp_rc_t rc;
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

  /*
  Serial.print("\n");
  Serial.println(pulseIn(6, HIGH,10));
  Servo1.write((map_float(attitude.pitch, 90.0, -90.0, 0.0, 180.0)/4.0)+60);
  Serial.println("\n" + String(attitude.pitch));
  Servo2.write((map_float(attitude.roll, 180.0, -180.0, 0.0, 180.0)/4.0)+75);
  Serial.println(attitude.roll);
*/

  if (on = true) {
    setGyroServo();
  } else {
    Servo1.write(90);
    Servo2.write(90);
  }
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Interrupt Service Routine (ISR) function
void handleInterrupt() {

  unsigned long currentTime = millis();
  // Check if it's been longer than the debounce period since the last trigger
  if (currentTime - lastTriggerTime >= debouncePeriod) {
    // Record the current time as the last trigger time
    lastTriggerTime = currentTime;

    // Check the current state of the interrupt pin
    bool currentState = digitalRead(interruptPin);

    Serial.println("interrupt");
    if (ini) {
      Serial.println("interrupted");
      // Only process if the state has changed
      if ((pulseIn(interruptPin, HIGH) - 1800) > 0) {
        interruptState = currentState;

        if (pulseIn(interruptPin, HIGH) > 1800) {
          Serial.println("Do gyro stuff");
          on = true;
          //setGyroServo();
        } else {
          //Servo1.write(90);
          //Servo2.write(90);
          Serial.println("Set gyro to 90");
          on = false;
        }
      }
    }
  }

  void setGyroServo() {
    msp_attitude_t attitude;
    msp.request(MSP_ATTITUDE, &attitude, sizeof(msp_attitude_t));

    Servo1.write((map_float(attitude.pitch, 90.0, -90.0, 0.0, 180.0) / 4.0) + 60);
    Serial.println("\n" + String(attitude.pitch));
    Servo2.write((map_float(attitude.roll, 180.0, -180.0, 0.0, 180.0) / 4.0) + 75);
    Serial.println(attitude.roll);
  }
}