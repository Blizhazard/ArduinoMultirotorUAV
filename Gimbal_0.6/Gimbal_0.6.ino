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

//#define INTERRUPT_PIN 2


void setup(void){
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(500000);


  servoX.attach(9);
  servoX.write(90);
  
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
  }

  //pinMode(INTERRUPT_PIN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

}

void loop(){
  //if (!dmpReady) return;

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); 
  currentTime = millis();
  while(currentTime < 1000){
    xError = g.gyro.x;
    yError = g.gyro.y;
    delay(10);
    currentTime = millis();
  }
  currentTime = millis();
  

  //posIntY = posY*(180.0/pi)+90.0;
  servoX.write(posX*(180.0/pi)+90.0);
  //servoY.write(posIntY);
  Serial.print(posX);
  Serial.print("\n");
  //Serial.print(posY);
  Serial.print("\nRotation X: ");
  Serial.print(g.gyro.x-xError);
  Serial.print(", Y: ");
  //Serial.print(g.gyro.y-yError);
  //Serial.print("\n");
  //Serial.print(currentTime);
  Serial.print("\n");
  //Serial.print(previousMillis);
  Serial.print("\n");
  


}

void getGyroData(){
  posX = posX + (g.gyro.x-xError)*((currentTime - previousMillis)/1000.0);
  //posY = posY + (g.gyro.y-yError)*((currentTime - previousMillis)/1000.0);
  previousMillis = currentTime; 
  return posX;
}
