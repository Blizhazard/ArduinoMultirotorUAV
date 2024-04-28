#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>

#define MPU6050_ADDR 0x68

Adafruit_MPU6050 mpu;
Servo servoX;
Servo servoY;

int X_AXIS_PIN = 6;
int Y_AXIS_PIN = 7;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10); // Wait for serial port to connect
  }
  Serial.println("Initializing MPU6050...");
  if (!mpu.begin(MPU6050_ADDR)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  servoX.attach(X_AXIS_PIN);
  servoY.attach(Y_AXIS_PIN);
}

void loop() {
  // Read gyro data
  sensors_event_t gyroEvent;
  mpu.getEvent(nullptr, &gyroEvent, nullptr);

  float gyroX = gyroEvent.gyro.x;
  float gyroY = gyroEvent.gyro.y;
  float gyroZ = gyroEvent.gyro.z;


  Serial.print("X: ");
  Serial.print(gyroX);
  Serial.print(" Y: ");
  Serial.print(gyroY);
  Serial.print(" Z: ");
  Serial.println(gyroZ);


  /*
  // Calculate angles (simplified, you might need to implement a filter)
  float angleX = gyroX * GYRO_SCALE_FACTOR_X;
  float angleY = gyroY * GYRO_SCALE_FACTOR_Y;

  // Assuming you have initial angles, you can integrate angular velocities to get current angles
  static float angleX = 0.0;
  static float angleY = 0.0;

  angleX += angularVelocityX * 0.01; // 0.01 is the time interval in seconds
  angleY += angularVelocityY * 0.01; // Adjust time interval as per your loop duration


  // Map angles to servo positions
  int servoPosX = map(angleX, -90, 90, SERVO_MIN_X, SERVO_MAX_X);
  int servoPosY = map(angleY, -90, 90, SERVO_MIN_Y, SERVO_MAX_Y);

  // Set servo positions
  servoX.write(servoPosX);
  servoY.write(servoPosY);

  Serial.println(servoPosX);
  Serial.println(servoPosY);
  */

  // Add some delay to control loop
  delay(100);
}
