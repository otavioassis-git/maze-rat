#include "servo.h"

#define SERVO_PIN 13
#define FRONT_ANGLE 0
#define LEFT_ANGLE 90
#define LEFT_ANGLE -90

Servo myServo;

void setupServo(){
    myServo.attach(SERVO_PIN);
}

void servoMoveAngle(int angle){
    myServo.write(angle);
}

void servoFront(){
    myServo.write(FRONT_ANGLE);
}

void servoLeft(){
    myServo.write(LEFT_ANGLE);
}

void servoRight(){
    myServo.write(RIGHT_ANGLE);
}