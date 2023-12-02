#include <ESP32Servo.h>

#define SERVO_PIN 13
#define FRONT_ANGLE 0
#define LEFT_ANGLE 90
#define LEFT_ANGLE -90

void setupServo();
void servoMoveAngle(int angle);
void servoFront();

void servoLeft();

void servoRight();