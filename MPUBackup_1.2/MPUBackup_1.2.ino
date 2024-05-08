#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
Servo servoX;
Servo servoY;
double posX = 0;
double posY = 0;
unsigned long previousMillis = 0;
unsigned long currentTime = 0;
float xError = 0;
float yError = 0;
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
  currentTime = millis();
  while (currentTime < 1000) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    xError = g.gyro.x;
    yError = g.gyro.y;
    delay(10);
    currentTime = millis();
  }
  /*
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    //ini = !ini;
    return;
  }
  Serial.println("initialization done.");
  */
}

void loop() {

  getGyroData();
  servoX.write(posX * (180.0 / pi) + 90.0);
  servoY.write(posY * (180.0 / pi) + 90.0);
  Serial.println(posX * (180.0 / pi) + 90.0);
  //Serial.println(posY * (180.0 / pi) + 90.0);

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
  previousMillis = micros();
  mpu.getEvent(&a, &g, &temp);
  currentTime = micros();
  posX = posX + (g.gyro.x - xError) * ((currentTime - previousMillis) / 1000000.0);
  posY = posY + (g.gyro.y - yError) * ((currentTime - previousMillis) / 1000000.0);
}
