// ECE 302 Final Project: MPU6050 Arduino Self-Balancing Robot
// AUTHORS: Myles Anderson, Joel Awuku-Asante

#include <Adafruit_MPU6050.h> // Adafruit MPU libraries
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Arduino pins
int leftMotorPWMPin = 6;
int leftMotorDirPin0 = 4;
int leftMotorDirPin1 = 7;

int rightMotorPWMPin = 5;
int rightMotorDirPin0 = 2;
int rightMotorDirPin1 = 3;

char floatstr[10];
char strbuf[64];
double oldTime = 0;

double PWM = 0;
double BASE_PWM;

// PID parameters
double Kp = 17, Ki = 1.8, Kd = 0.01;

double error = 0, errorSum = 0, currentAngle = 0, prevAngle = 0, targetAngle = 0;
sensors_event_t a, g, temp; // acceleration, gyro, temp variables for getEvent function

void setup(void) {
  Serial.begin(115200);
  mpu.begin();
  Serial.println("Xbee reading");

  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin0, OUTPUT);
  pinMode(leftMotorDirPin1, OUTPUT);

  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin0, OUTPUT);
  pinMode(rightMotorDirPin1, OUTPUT);

  delay(1000); // delay to ensure stable before beginning
}

void setMotors(double PWM) {
  if(PWM < 0) {  // sign of PWM indicates direction
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
  // control speed of motors
  digitalWrite(leftMotorPWMPin, HIGH);
  digitalWrite(rightMotorPWMPin, HIGH);
  delayMicroseconds(abs(PWM));
  digitalWrite(leftMotorPWMPin, LOW);
  digitalWrite(rightMotorPWMPin, LOW);
  delayMicroseconds(1000 - abs(PWM));  
  
}

void loop() {
  Timer();
}

void Timer() {
  // Constants determined empirically from testing balancing performance
  // and printing MPU data.
  double CORRECTION_FACTOR = -2.8; // keep stable y velocity at zero
  double ANGLE_MULTIPLIER = -230.0; // keep error within PWM range [-900, 900]
  // calculate PID values and update PWM
  mpu.getEvent(&a, &g, &temp);

  // Get new sensor events with the readings
  // currentAngle is the y angular velocity +/- 4.36332 rad/sec
  currentAngle = g.gyro.y;
  if (currentAngle < 0) {
    BASE_PWM = 50; // BASE_PWM for added responsiveness
  }
  else {
    BASE_PWM = -50;
  }

  error = ANGLE_MULTIPLIER * currentAngle + CORRECTION_FACTOR;
  errorSum += error;
  
  PWM = BASE_PWM + Kp*(error) + Ki*(errorSum) + Kd*(currentAngle-prevAngle);
  prevAngle = currentAngle;

  PWM = constrain(PWM, -900.0, 900.0); // keep PWM within our decided range
  setMotors(PWM);
}
