#include <Wire.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Kalman.h"  // Add this library

MPU6050 mpu;
Kalman kalmanX;

#define INTERRUPT_PIN 2

// Motor Pins
int enableMotor1 = 9;
int motor1Pin1 = 5;
int motor1Pin2 = 6;

int enableMotor2 = 4;
int motor2Pin1 = 7;
int motor2Pin2 = 8;

// PID variables
double pitchAngle = 0;
double pitchPIDOutput = 0;
double setpointPitch = 0.0;

#define PID_KP 70
#define PID_KI 4.5
#define PID_KD 0

PID pitchPID(&pitchAngle, &pitchPIDOutput, &setpointPitch, PID_KP, PID_KI, PID_KD, DIRECT);

// MPU raw data
int16_t ax, ay, az;
int16_t gx, gy, gz;
unsigned long timer;
double dt;

// ======================= SETUP =========================
void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  // Make sure MPU is working
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Calibrated values (use your own if different)
  mpu.setXAccelOffset(-2402);
  mpu.setYAccelOffset(-2065);
  mpu.setZAccelOffset(456);
  mpu.setXGyroOffset(-2);
  mpu.setYGyroOffset(-4);
  mpu.setZGyroOffset(65);

  // Setup motors
  pinMode(enableMotor1, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enableMotor2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  rotateMotor(0, 0);

  // Kalman initial pitch
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  double roll  = atan2(ay, az) * RAD_TO_DEG;
  kalmanX.setAngle(roll);
  timer = micros();

  // PID setup
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-255, 255);
  pitchPID.SetSampleTime(10);
}

// ======================= LOOP =========================
void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  double accelAngle = atan2(ay, az) * RAD_TO_DEG;
  unsigned long now = micros();
  dt = (double)(now - timer) / 1000000;
  timer = now;

  double gyroRate = gx / 131.0; // MPU6050 gyro scaling (250°/s)
  pitchAngle = kalmanX.getAngle(accelAngle, gyroRate, dt);

  pitchPID.Compute();

  // Stop jittering if very close to balanced
  if (abs(pitchPIDOutput) < 3) {
    rotateMotor(0, 0);
  } else {
    rotateMotor(pitchPIDOutput, pitchPIDOutput);
  }

  // Debug
  Serial.print("Pitch: "); Serial.print(pitchAngle);
  Serial.print(" | PID: "); Serial.println(pitchPIDOutput);
}

// ======================= MOTOR CONTROL =========================
void rotateMotor(int speed1, int speed2) {
  // Motor 1
  if (speed1 < 0) {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  } else {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  }

  // Motor 2
  if (speed2 < 0) {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  } else {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  }

  speed1 = constrain(abs(speed1), 0, 255);
  speed2 = constrain(abs(speed2), 0, 255);

  analogWrite(enableMotor1, speed1);
  analogWrite(enableMotor2, speed2);
}
