// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

//had to adjust
int leftMotorPWMPin = 6;
int leftMotorDirPin0 = 4;
int leftMotorDirPin1 = 7;

int rightMotorPWMPin = 5;
int rightMotorDirPin0 = 2;
int rightMotorDirPin1 = 3;

char floatstr[10]; // float converted to string
char strbuf[64];
double oldTime = 0;
// sampleTime = 0.005;
double PWM = 0;
double BASE_PWM;
// 17, 1.8 work for sure, so do 17.8, 2.4 for balance
// 14, 3, 11(constant pwm) work for going forward at constant speed, not smooth tho
// 17, 2.4, 5.5(constant pwm) works better
double Kp = 18, Ki = 2.54, Kd = 1;

double error = 0, errorSum = 0, currentAngle = 0, prevAngle = 0, targetAngle = 0;
sensors_event_t a, g, temp;

void setup(void) {
  Serial.begin(115200);
  // Try to initialize!
  mpu.begin();
  Serial.println("Xbee reading");

  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin0, OUTPUT);
  pinMode(leftMotorDirPin1, OUTPUT);

  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin0, OUTPUT);
  pinMode(rightMotorDirPin1, OUTPUT);
// // verify motors operational
  // digitalWrite(leftMotorDirPin0, HIGH);
  // digitalWrite(leftMotorDirPin1, LOW);
  // digitalWrite(rightMotorDirPin0, HIGH);
  // digitalWrite(rightMotorDirPin1, LOW);

  delay(1000);
}

void setMotors(double PWM) {
  if(PWM < 0) {  
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
  digitalWrite(leftMotorPWMPin, HIGH);
  digitalWrite(rightMotorPWMPin, HIGH);
  delayMicroseconds(abs(PWM));
  digitalWrite(leftMotorPWMPin, LOW);
  digitalWrite(rightMotorPWMPin, LOW);
  delayMicroseconds(1000 - abs(PWM));  
  
}

// //delay between prints to xbee should be at least 5000 (can get away with like 2500 but will have some missing newlines)
void loop() {
Timer();
// delay(1000);
// // verify motors operational
//   digitalWrite(leftMotorPWMPin, HIGH);
//   digitalWrite(rightMotorPWMPin, HIGH);
//   delayMicroseconds(500);
//   digitalWrite(leftMotorPWMPin, LOW);
//   digitalWrite(rightMotorPWMPin, LOW);
//   delayMicroseconds(1000 - 500);
// Serial.println(errorSum);
// delay(100);

//   // setMotors(PWM);
  
    
//   //Serial.println(mpu.getMotionInterruptStatus());
//   // Get new sensor events with the readings
//   // mpu.getMotionInterruptStatus();

//   //sprintf doesn't support %f directly -> convert float to string using dtostrf first instead
//   //too much going on with this approach, corrupted output seen in putty
//   //dtostrf(g.gyro.x, 6, 3, floatstr);
//   //sprintf(strbuf, "\rGyro: %s", floatstr);
//   //Serial.println(strbuf);


}

void Timer() {
//   // calculate PID values and update PWM
//   // calculate angle from MPU data
  mpu.getEvent(&a, &g, &temp);
  // Serial.print("X: ");
  // Serial.println(g.gyro.x);
  // Serial.print("Y: ");
  // Serial.println(g.gyro.y);
  // Serial.print("Z: ");
  // Serial.println(g.gyro.z);

  // Get new sensor events with the readings
  // want angular speed to be zero
  currentAngle = g.gyro.y; // +/- 250 deg/sec (4.36332 rad/sec -> data is in rad/sec)
  if (currentAngle < 0) {
    BASE_PWM = 50;
  }
  else {
    BASE_PWM = -50;
  }
  
  error = -230.0*currentAngle;
  // Serial.print("Error: ");
  // Serial.println(error);
  errorSum += error;
  
  //errorSum = constrain(errorSum, -1000.0, 1000.0);
  PWM = 5.5 + BASE_PWM + Kp*(error) + Ki*(errorSum); //+ Kd*(currentAngle-prevAngle);
  // Serial.print("PWM:");
  // Serial.println(PWM);
  // prevAngle = currentAngle;
  PWM = constrain(PWM, -900.0, 900.0);
  setMotors(PWM);
}
