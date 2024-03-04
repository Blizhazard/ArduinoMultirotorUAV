#include <Wire.h> 
//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

Servo servoX;
Servo servoY;

/*double posX = 0;
int posIntX = 0;
double posY = 0;
int posIntY = 0;
unsigned long previousMillis = 0;
unsigned long currentTime = 0;
float xError = 0;
float yError = 0;*/

//========================================Variables to reading gyro values=============================================
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}
//=====================================================================================================================

float correct;
int j = 0;
int yawValue;
int pitchValue;
int rollValue;


#define INTERRUPT_PIN 2
MPU6050 mpu;

void setup(){
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(38400);


  servoX.attach(9);
  servoX.write(90);
  
  servoY.attach(3);
  servoY.write(90);
  
  //while (!Serial)
    //delay(10); // will pause Zero, Leonardo, etc until serial console opens

  //Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  /*if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }*/
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  /*mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  //Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    //Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    //Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    //Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    //Serial.println("+-16G");
    break;
  }

    
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  //Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    //Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    //Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    //Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    //Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  //Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    //Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    //Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    //Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    //Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    //Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    //Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    //Serial.println("5 Hz");
    break;
  }*/
  


}

void loop(){
  if (!dmpReady) return;

  readMPU6050();
  
  /* Get new sensor events with the readings */
  /*sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); 
  currentTime = millis();
  while(currentTime < 1000){
    xError = g.gyro.x;
    yError = g.gyro.y;
    delay(10);
    currentTime = millis();
  }
  currentTime = millis();
  posX = posX + (g.gyro.x-xError)*((currentTime - previousMillis)/1000.0);
  posY = posY + (g.gyro.y-yError)*((currentTime - previousMillis)/1000.0);
  previousMillis = currentTime; 
  posIntX = posX*(180.0/3.1415)+90.0;
  posIntY = posY*(180.0/3.1415)+90.0;
  servoX.write(posIntX);
  servoY.write(posIntY);*/

  yawValue = map(ypr[0], -90, 90, 0, 180);
  pitchValue = map(ypr[1], -90, 90, 180, 0);
  rollValue = map(ypr[2], -90, 90, 0, 180);
  Serial.print("yaw: "+ String(yawValue) + "" + "pitch: " + String(pitchValue) + "" + "roll: " + String(rollValue)+"\n");
  /*Serial.print("\n");
  Serial.print(posY);
  Serial.print("\nRotation X: ");
  Serial.print(g.gyro.x-xError);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y-yError);
  //Serial.print("\n");
  //Serial.print(currentTime);
  Serial.print("\n");
  //Serial.print(previousMillis);
  Serial.print("\n");*/
}


void readMPU6050() {
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
   
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;

    if (j <= 300) {
      j++;
      correct = ypr[0];
      readMPU6050();
      return;
    }
    ypr[0] = ypr[0] - correct;
  }
}
