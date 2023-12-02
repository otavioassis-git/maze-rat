#include "servo.h"

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