#include <Wire.h>
#include <MPU6050.h>

#define TRIG_PIN 9
#define ECHO_PIN 10
#define MOTOR1_PIN 5  
#define MOTOR2_PIN 6
#define IN1 4
#define IN2 7
#define IN3 8
#define IN4 9

MPU6050 mpu;

float Kp = 1.0;
float Ki = 0.1;
float Kd = 0.05;
float integral = 0;
float previous_error = 0;
float setpoint = 35;
unsigned long lastTime = 0;

bool branchDetected = false;
float initialYaw = 0;
float yaw = 0;
float lastGyroZ = 0;
float filteredGyroZ = 0;
const float alpha = 0.1; 

void calibrateGyro() {
  const int numSamples = 1000;
  long sumZ = 0;

  Serial.begin(9600); 

  Serial.println("Calibrating gyroscope...");
  for (int i = 0; i < numSamples; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    sumZ += gz;
    delay(1);
  }

  lastGyroZ = (float)sumZ / numSamples / 131.0;
  Serial.print("Gyroscope Z bias: ");
  Serial.println(lastGyroZ);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  Serial.println("MPU6050 initialized successfully");

  // Calibrate gyroscope bias
  calibrateGyro();
  lastTime = millis();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
}

void loop() {
  // Ultrasonic sensor logic for branch detection
  long duration;
  int distance;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance < 50) { // Assuming branch detected within 50 cm
    if (!branchDetected) {
      branchDetected = true;
      Serial.println("Branch detected - continue rotating");
      initialYaw = getYaw();
      Serial.print("Initial Yaw: ");
      Serial.println(initialYaw);
    }
  }

  if (branchDetected) {
    float yaw = getYaw();
    float yawChange = abs(yaw - initialYaw);
    
    Serial.print("Current Yaw: ");
    Serial.println(yaw);
    Serial.print("Yaw Change: ");
    Serial.println(yawChange);

    if (yawChange >= setpoint) {
      Serial.println("Branch aligned with gap");
      branchDetected = false; // Reset for the next branch detection
      stopMotors();
      // Set both motors to the same speed for straight movement
      analogWrite(MOTOR1_PIN, 200);
      analogWrite(MOTOR2_PIN, 200); 
    } else {
      float motorSpeed = pidControl(yawChange);
      analogWrite(MOTOR1_PIN, motorSpeed);
      analogWrite(MOTOR2_PIN, motorSpeed / 2); 
      Serial.print("Motor Speed: ");
      Serial.println(motorSpeed);
    }
  } else if (!branchDetected) { // Continuously spin until branch detected
    analogWrite(MOTOR1_PIN, 255);
    analogWrite(MOTOR2_PIN, 150);
  }

  delay(500);
}

float getYaw() {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds

  lastTime = currentTime;

  // Convert raw gyroscope value to degrees/second and remove bias
  float gyroZ = (gz / 131.0) - lastGyroZ;

  // Apply low-pass filter
  filteredGyroZ = alpha * gyroZ + (1 - alpha) * filteredGyroZ;

  // Apply trapezoidal rule
  yaw += (filteredGyroZ + lastGyroZ) * deltaTime / 2.0;

  lastGyroZ = gyroZ;

  return yaw;
}

float pidControl(float currentYawChange) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
  lastTime = currentTime;

  float error = setpoint - currentYawChange;
  integral += error * deltaTime;
  float derivative = (error - previous_error) / deltaTime;
  float output = Kp * error + Ki * integral + Kd * derivative;

  previous_error = error;
  return constrain(output, 0, 255); 
}

void setMotorSpeed(float speed) {
  analogWrite(MOTOR1_PIN, speed);
  analogWrite(MOTOR2_PIN, speed);
}

void stopMotors() {
  analogWrite(MOTOR1_PIN, 0);
  analogWrite(MOTOR2_PIN, 0);
}
