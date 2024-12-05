// Basic demo for accelerometer readings from Adafruit MPU6050

#include <TimerOne.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

int leftMotorPWMPin = 6;
int leftMotorDirPin0 = 4;
int leftMotorDirPin1 = 5;

int rightMotorPWMPin = 5;
int rightMotorDirPin0 = 2;
int rightMotorDirPin1 = 3;

int oldTime = 0;
int sampleTime = 0.005;
int PWM = 0;
int BASE_PWM = 10;

int Kp = 1, Ki = 1, Kd = 1;

volatile int error = 0, errorSum = 0, currentAngle = 0, prevAngle = 0, targetAngle = 0;
volatile sensors_event_t a, g, temp;

void setup(void) {
  Serial.begin(115200);
  // Try to initialize!
  mpu.begin();

  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin0, OUTPUT);
  pinMode(leftMotorDirPin1, OUTPUT);

  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin0, OUTPUT);
  pinMode(rightMotorDirPin1, OUTPUT);
}

void setMotors(int PWM) {
  digitalWrite(leftMotorPWMPin, HIGH);
  digitalWrite(rightMotorPWMPin, HIGH);
  delayMicroseconds(abs(PWM));
  digitalWrite(leftMotorPWMPin, LOW);
  digitalWrite(rightMotorPWMPin, LOW);
  delayMicroseconds(1000 - abs(PWM));  
  if(PWM >= 0) {  
    digitalWrite(leftMotorDirPin0, HIGH);
    digitalWrite(leftMotorDirPin1, LOW);
    digitalWrite(rightMotorDirPin0, HIGH);
    digitalWrite(rightMotorDirPin1, LOW);
  }
  else {
    digitalWrite(leftMotorDirPin0, LOW);
    digitalWrite(leftMotorDirPin1, HIGH);
    digitalWrite(rightMotorDirPin0, LOW);
    digitalWrite(rightMotorDirPin1, HIGH);
  }
}

void loop() {
  setMotors(PWM);
  if (millis() - oldTime >= sampleTime || oldTime == 0) {
    Timer();
  }
}

void Timer() {
  // calculate PID values and update PWM
  // calculate angle from MPU data
  if(mpu.getMotionInterruptStatus()) {
    // Get new sensor events with the readings
    mpu.getEvent(&a, &g, &temp);
    currentAngle = prevAngle + g.gyro.x; // +/- 250 deg/sec
  }
  error = currentAngle - targetAngle;
  errorSum += error;
  errorSum = constrain(errorSum, -250, 250);
  PWM = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;
  PWM = constrain(PWM, -1000, 1000);
  oldTime = millis();

}
