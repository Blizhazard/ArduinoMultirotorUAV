#include <Wire.h> 
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
Servo servoX;
Servo servoY;
double posX = 0;
int posIntX = 0;
double posY = 0;
int posIntY = 0;
unsigned long previousMillis = 0;
unsigned long currentTime = 0;
float xError = 0;
float yError = 0;
double pi = 3.1415926535897932384626433832795;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

Adafruit_MPU6050 mpu;



void setup(void){
  Wire.begin();
  Serial.begin(9600);


  servoX.attach(9);
  servoX.write(90);
  
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(1000);

  currentTime = micros();
  float count;
  while(currentTime < 1000000){
    count += 1;
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp); 
    xError += g.gyro.x;
    currentTime = micros();
  }
  xError = (xError / count);
  Serial.println("Error:");
  Serial.println(xError);
}

void loop(){
  getGyroData();
  servoX.write(posX*(180.0/pi)+90);
  Serial.println(posX);
}

void getGyroData(){
  sensors_event_t a, g, temp;
  previousMillis = micros(); 
  mpu.getEvent(&a, &g, &temp); 
  currentTime = micros();
  if (abs((g.gyro.x-xError)*((currentTime - previousMillis)/1000000.0))<0.001){
    return;
  }
  posX = posX + (g.gyro.x-xError)*((currentTime - previousMillis)/1000000.0);
}
