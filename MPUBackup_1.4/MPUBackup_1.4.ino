#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <SD.h>
Servo servoX;
Servo servoY;
File myFile;
float posX = 0;
float posY = 0;
float yaw = 0;
unsigned long previousMillis = 0;
unsigned long currentTime = 0;
float xError = 0;
float yError = 0;
float zError = 0;
float xAError = 0;
float yAError = 0;
double pi = 3.1415926535897932384626433832795;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
volatile bool mpuInterrupt = false;

const int interruptPin = 2;
volatile bool on = false;

void dmpDataReady() {
  mpuInterrupt = true;
}

Adafruit_MPU6050 mpu;

//#define INTERRUPT_PIN 2


void setup(void) {
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  Serial.begin(19200);


  servoX.attach(9);
  servoX.write(90);

  // Set up interrupt pin
  pinMode(interruptPin, INPUT);

  servoY.attach(3);
  servoY.write(90);


  //while (!Serial)
  //delay(10); // will pause Zero, Leonardo, etc until serial console opens

  //Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(1000);
  float cumXError;
  float cumYError;
  float cumZError;
  float cumAccErrorX;
  float cumAccErrorY;
  for (int i = 0; i < 200; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    cumXError += g.gyro.x;
    cumYError += g.gyro.y;
    cumZError += g.gyro.z;
    cumAccErrorX += a.acceleration.x;
    cumAccErrorY += a.acceleration.y;
  }
  xError = cumXError / 200;
  yError = cumYError / 200;
  zError = cumZError / 200;
  xAError = cumAccErrorX / 200;
  yAError = cumAccErrorY / 200;
  
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  
}

void loop() {

  getGyroData();
  servoX.write(posX * (180.0 / pi) + 90.0);
  servoY.write(posY * (180.0 / pi) + 90.0);
  Serial.println(posX * (180.0 / pi) + 90.0);
  //Serial.println(posY * (180.0 / pi) + 90.0);
  //Serial.println(yaw);
  
  myFile = SD.open("gyro.txt", FILE_WRITE);
  if (myFile) {
    myFile.println("Att-Roll: " + String(posY * (180.0 / pi)));
    myFile.println(" Att-Pitch: " + String(posY * (180.0 / pi)));
    myFile.println(" Att-Yaw: " + String(yaw));
    myFile.close();
  } else {
    Serial.println("error opening gyro.txt");
  }
  
  /*
  if (pulseIn(interruptPin, HIGH) > 1800) {
    on = true;
  } else {
    on = false;
  }
  if (on == true) {
    getGyroData();
    servoX.write(posX * (180.0 / pi) + 90.0);
    servoY.write(posY * (180.0 / pi) + 90.0);
    Serial.println(posX * (180.0 / pi) + 90.0);
    Serial.println(posY * (180.0 / pi) + 90.0);
    //Serial.print("\nRotation X: ");
    //Serial.print(g.gyro.x-xError);
    //Serial.print(", Y: ");
    //Serial.print(g.gyro.y-yError);
    //Serial.print("\n");
    //Serial.print(currentTime);
    //Serial.print("\n");
    //Serial.print(previousMillis);
    //Serial.print("\n");
    /*
    myFile = SD.open("gyro.txt", FILE_WRITE);
    if (myFile) {
      int16_t yaw;
      yaw = attitude.yaw;
      myFile.println("Att-Roll: " + String(attitude.roll / 10.0));
      myFile.println(" Att-Pitch: " + String(attitude.pitch / 10.0));
      myFile.println(" Att-Yaw: " + String(yaw));
      myFile.close();
    } else {
      Serial.println("error opening gyro.txt");
    }
    
  } else {
    servoX.write(90);
    servoY.write(90);
  }
  */
}

void getGyroData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  currentTime = micros();
  float accAngleX = 
  Serial.println(a.gyro.x);
  posX = posX + (g.gyro.x - xError) * ((currentTime - previousMillis) / 1000000.0);
  posY = posY + (g.gyro.y - yError) * ((currentTime - previousMillis) / 1000000.0);
  yaw = yaw + (g.gyro.z) * ((currentTime - previousMillis) / 1000000.0);
  previousMillis = micros();
}
