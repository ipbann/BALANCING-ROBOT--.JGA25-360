#include <Wire.h>
#include <MPU6050.h>

// === Pin Assignment ===
#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25
#define ENA 32
#define ENB 33
#define ENCODER_A 34
#define ENCODER_B 35

MPU6050 mpu;

// === Global Variables ===
float setpoint = 0.0;
float angle, gyroY, accAngle;
float error, previousError = 0;
float integral = 0;
float derivative;

float Kp = 25.0;  // Proportional gain
float Ki = 1.5;   // Integral gain
float Kd = 1.2;   // Derivative gain

unsigned long lastTime;
float elapsedTime;

// Encoder
volatile long encoderCount = 0;

void IRAM_ATTR encoderISR() {
  bool a = digitalRead(ENCODER_A);
  bool b = digitalRead(ENCODER_B);
  if (a == b) encoderCount++;
  else encoderCount--;
}

void setupMPU() {
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
}

float getAngle() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Hitung sudut dari akselerometer
  accAngle = atan2(ay, az) * 180 / PI;

  // Hitung delta waktu
  unsigned long currentTime = millis();
  elapsedTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // Filter complementary
  gyroY = gy / 131.0;
  angle = 0.98 * (angle + gyroY * elapsedTime) + 0.02 * accAngle;

  return angle;
}

void setupMotors() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void setMotorSpeed(int motor, int pwm) {
  pwm = constrain(pwm, -255, 255);

  bool dir = pwm >= 0;
  pwm = abs(pwm);

  if (motor == 0) { // Motor A
    digitalWrite(IN1, dir);
    digitalWrite(IN2, !dir);
    ledcWrite(0, pwm);
  } else { // Motor B
    digitalWrite(IN3, dir);
    digitalWrite(IN4, !dir);
    ledcWrite(1, pwm);
  }
}

void setupPWM() {
  ledcSetup(0, 1000, 8); // channel 0 on GPIO32 (ENA)
  ledcAttachPin(ENA, 0);

  ledcSetup(1, 1000, 8); // channel 1 on GPIO33 (ENB)
  ledcAttachPin(ENB, 1);
}

void setupEncoder() {
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
}

void setup() {
  Serial.begin(115200);
  setupMPU();
  setupMotors();
  setupPWM();
  setupEncoder();
  lastTime = millis();
}

void loop() {
  float currentAngle = getAngle();

  // PID calculation
  error = currentAngle - setpoint;
  integral += error * elapsedTime;
  derivative = (error - previousError) / elapsedTime;
  previousError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;

  // Apply output to both motors
  setMotorSpeed(0, -output);
  setMotorSpeed(1, -output);

  delay(10); // loop delay
}
