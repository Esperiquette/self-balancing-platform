// Self-balancing platform (thrust vector control model)
// Cal Poly Pomona, ME 4990L Mechatronics Laboratory, Fall 2024
// Hardware: Arduino Uno, MPU6050 IMU (I2C), two hobby servos on pins 9 and 10.
// Code as printed in the final project report appendix, reformatted only (no logic changes).
// Known limitations are listed in README.md.

#include <Servo.h>
#include <Wire.h>

const int MPU = 0x68;  // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

// Servo objects
Servo servoX, servoY;

// Servo pins (replace with your actual pin numbers)
const int servoXPin = 9;
const int servoYPin = 10;

// PID controller variables (adjust these as needed)
float Kp = 0.5;   // Proportional gain
float Ki = 0.01;  // Integral gain
float Kd = 0.1;   // Derivative gain

// Target angles (adjust these to your desired starting position)
float targetRoll = 0.0;
float targetPitch = 0.0;

// Error and integral terms
float errorX, errorY, integralX, integralY;
float previousErrorX, previousErrorY;
// Servo limits (adjust these as needed)
const int minServoPos = 0;
const int maxServoPos = 180;
const int maxControl = 180;  // Adjust this based on your PID tuning

// Low-pass filter constants for smoothing gyroscope data
float gyroX_filtered = 0;
float gyroY_filtered = 0;
float gyroZ_filtered = 0;
float alpha = 0.9;  // Smoothing factor (close to 1 keeps the previous value, 0 gives more weight to
                    // the current)

// Gyroscope bias (calculated from stationary measurements)
float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

void setup() {
  Serial.begin(19200);
  Wire.begin();

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  calculate_IMU_error();
  delay(20);

  // Initialize servos
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);

  // Set initial servo positions
  servoX.write(90);  // Adjust this to your desired starting position
  servoY.write(90);

  // Calculate and apply gyro bias
  calculateGyroBias();
}

void loop() {
  // Read accelerometer data
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

  // Calculate Roll and Pitch from accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + AccErrorY;

  // Read gyroscope data
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;

  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0 + GyroErrorX;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0 + GyroErrorY;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0 + GyroErrorZ;
  // Apply low-pass filter to gyro data to smooth it
  gyroX_filtered = alpha * gyroX_filtered + (1 - alpha) * (GyroX - gyroBiasX);
  gyroY_filtered = alpha * gyroY_filtered + (1 - alpha) * (GyroY - gyroBiasY);
  gyroZ_filtered = alpha * gyroZ_filtered + (1 - alpha) * (GyroZ - gyroBiasZ);

  // Update angles with filtered gyroscope data
  gyroAngleX = gyroAngleX + gyroX_filtered * elapsedTime;
  gyroAngleY = gyroAngleY + gyroY_filtered * elapsedTime;
  yaw = yaw + gyroZ_filtered * elapsedTime;

  // Complementary filter - combine accelerometer and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  // Calculate error and update integral terms
  errorX = targetRoll - roll;
  errorY = targetPitch - pitch;
  integralX += errorX * elapsedTime;
  integralY += errorY * elapsedTime;

  // Prevent integral windup
  float maxIntegral = 50.0;
  integralX = constrain(integralX, -maxIntegral, maxIntegral);
  integralY = constrain(integralY, -maxIntegral, maxIntegral);

  // PID calculations
  float controlX = -(Kp * errorX + Ki * integralX + Kd * (errorX - previousErrorX) / elapsedTime);
  float controlY = Kp * errorY + Ki * integralY + Kd * (errorY - previousErrorY) / elapsedTime;

  // Map control output to servo positions
  int servoXPos = constrain(map(controlX, -180, 180, 0, 180), 0, 180);
  int servoYPos = constrain(map(controlY, -180, 180, 0, 180), 0, 180);
  // Set servo positions
  servoX.write(servoXPos);
  servoY.write(servoYPos);

  // Reset integral if close to target
  if (abs(errorX) < 1.0 && abs(errorY) < 1.0) {
    integralX = 0;
    integralY = 0;
  }

  // Update previous error for derivative term
  previousErrorX = errorX;
  previousErrorY = errorY;
}

// Function to calculate gyroscope bias by averaging stationary readings
void calculateGyroBias() {
  long sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;

  for (int i = 0; i < 100; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

    sumGyroX += GyroX;
    sumGyroY += GyroY;
    sumGyroZ += GyroZ;
    delay(10);  // Small delay for smoother readings
  }
  gyroBiasX = sumGyroX / 100;
  gyroBiasY = sumGyroY / 100;
  gyroBiasZ = sumGyroZ / 100;
}

void calculate_IMU_error() {
  // Initialize error values
  AccErrorX = 0.0;
  AccErrorY = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY = 0.0;
  GyroErrorZ = 0.0;

  // Code to calculate IMU error
  // ...
}
