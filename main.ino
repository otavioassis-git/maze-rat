#include <ESP32Servo.h>
#include <MPU6050_light.h>
#include <string.h>

#include "Wire.h"

// Pinos servo motor
#define SERVO_PIN 13
#define FRONT_ANGLE 80
#define LEFT_ANGLE 180
#define RIGHT_ANGLE 0

// Pinos sensor ultrassonico
#define MISO_ECHO 19
#define EN_TRIG 16

// Pinos sensor óptico
#define SC 34
#define SL 35
#define SR 39
#define SLC 36
#define SRC 32

// Pinos sensor giroscópio
#define SDA 21
#define SDL 22

// pinos motor
#define E_CH1 33
#define CHA_M1 25
#define E_CH2 26
#define CHA_M2 27
#define CS_Sensors 5

/////////////////////////////////////////////////// SERVO

Servo myServo;

void setupServo() {
    myServo.attach(SERVO_PIN);
}

void servoMoveAngle(int angle) {
    myServo.write(angle);
    delay(500);
}

void servoFront() {
    myServo.write(FRONT_ANGLE);
    delay(500);
}

void servoLeft() {
    myServo.write(LEFT_ANGLE);
    delay(500);
}

void servoRight() {
    myServo.write(RIGHT_ANGLE);
    delay(500);
}

/////////////////////////////////////////////////// FIM SERVO

/////////////////////////////////////////////////// ULTRASSOM

// Setup do sensor ultrassonico
void setupUltrassonicSensor() {
    pinMode(MISO_ECHO, INPUT);
    pinMode(EN_TRIG, OUTPUT);
}

// Retorna a distâncias em centímetros
float getDistance() {
    digitalWrite(EN_TRIG, LOW);
    delayMicroseconds(10);
    digitalWrite(EN_TRIG, HIGH);
    delayMicroseconds(250);
    digitalWrite(EN_TRIG, LOW);
    delayMicroseconds(10);

    unsigned long duration = pulseIn(MISO_ECHO, HIGH);
    float distance = duration / 58;

    return distance;
}

// Retorna a média da leitura de 3 distâncias
float getDistanceMean() {
    float sum = 0.0;
    for (int i = 0; i < 3; i++) {
        sum += getDistance();
    }

    return sum / 3;
}

////////////////////////////////////////////////////// FIM ULTRASSOM

///////////////////////////////////////////////////// OPTICOS
void setupOpticalSensors() {
    pinMode(SC, INPUT);
    pinMode(SL, INPUT);
    pinMode(SR, INPUT);
    pinMode(SLC, INPUT);
    pinMode(SRC, INPUT);
}

// Retorna a leitura do sensor óptio central
int getSCReading() {
    return analogRead(SC);
}

// Retorna a leitura do sensor óptio esquerdo
int getSLReading() {
    return analogRead(SL);
}

// Retorna a leitura do sensor óptio direito
int getSRReading() {
    return analogRead(SR);
}

// Retorna a leitura do sensor óptio esquerdo central
int getSLCReading() {
    return analogRead(SLC);
}

// Retorna a leitura do sensor óptio direito central
int getSRCReading() {
    return analogRead(SRC);
}

///////////////////////////////////////////////////// FIM OPTICOS

///////////////////////////////////////////////////// GIROSCÓPIO

MPU6050 mpu(Wire);

void setupGyro() {
    Wire.begin();
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    while (status != 0) {
    }  // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets();  // gyro and accelero
    Serial.println("Done!\n");
}

int getAngle() {
    mpu.update();
    delay(10);
    return mpu.getAngleZ();
}

///////////////////////////////////////////////////// FIM GIROSCÓPIO

////////////////////////////////////////////////////// MOTORES
void setupMotors() {
    pinMode(E_CH1, OUTPUT);
    pinMode(E_CH2, OUTPUT);

    pinMode(CHA_M1, OUTPUT);
    pinMode(CHA_M2, OUTPUT);
}

void enableMotors() {
    digitalWrite(CS_Sensors, HIGH);
    digitalWrite(E_CH1, HIGH);
    digitalWrite(E_CH2, HIGH);
}

void disableMotors() {
    digitalWrite(CS_Sensors, LOW);
    digitalWrite(E_CH1, LOW);
    digitalWrite(E_CH2, LOW);
}

void pwmMotors(int pwm_right, int pwm_left) {
    analogWrite(CHA_M1, pwm_right);
    analogWrite(CHA_M2, pwm_left);
}
////////////////////////////////////////////////////// FIM MOTORES

////////////////////////////////////////////////////// SEGUE LINHA
#define CROSSING_OFFSET 1500

float rSensor;
float lSensor;
float rcSensor;
float lcSensor;

float MIN_SPEED_R = 30.0;
float MIN_SPEED_L = 50.0;

void followLine() {
    rSensor = float(getSRReading());
    lSensor = float(getSLReading());

    // freiando caso identifique um cruzamento com os sensores frontais
    if (rSensor >= CROSSING_OFFSET && lSensor >= CROSSING_OFFSET) {
        MIN_SPEED_R = 20.0;
        MIN_SPEED_L = 40.0;
    }

    float kp = 0.006;

    // calculando a velocidade a ser somada
    float rSpeed = kp * lSensor;
    float lSpeed = kp * rSensor;

    // calculando o valor absoluto da velocidade para cada motor
    float pwm_sr = 128 + (rSpeed + MIN_SPEED_R);
    float pwm_sl = 128 - (lSpeed + MIN_SPEED_L);

    pwmMotors(int(pwm_sr), int(pwm_sl));
    enableMotors();
}

void turnRight() {
    int target = getAngle() - 85;
    while (getAngle() > target) {
        pwmMotors(88, 88);
        enableMotors();
    }
    disableMotors();
}

void turnLeft() {
    int target = getAngle() + 85;
    while (getAngle() < target) {
        pwmMotors(168, 168);
        enableMotors();
    }
    disableMotors();
}

void turnAround() {
    int target = getAngle() - 170;
    while (getAngle() > target) {
        pwmMotors(88, 88);
        enableMotors();
    }
    disableMotors();
}
////////////////////////////////////////////////////// SEGUE LINHA FIM

///////////////////////////////////////////////////// UTILS

byte isCloseToWall(float distance) {
    if (distance > 40)
        return false;
    return true;
}

byte isOverCrossing() {
    rcSensor = getSRCReading();
    lcSensor = getSLCReading();
    if (rcSensor > CROSSING_OFFSET && lcSensor > CROSSING_OFFSET)
        return true;
    return false;
}

///////////////////////////////////////////////////// FIM UTILS

float wallDistance = 400;

void setup() {
    setupOpticalSensors();
    setupUltrassonicSensor();
    setupServo();
    setupMotors();
    Serial.begin(115200);  // ESP32
    setupGyro();           // tem que ficar depois do Serial.begin
    servoFront();
}

void loop() {
    Serial.println(getDistance());
    if (!isOverCrossing()) {
        followLine();
    } else {
        disableMotors();
        MIN_SPEED_R = 30.0;
        MIN_SPEED_L = 50.0;
        servoRight();
        wallDistance = getDistanceMean();
        int count = 50;
        if (!isCloseToWall(wallDistance)) {
            Serial.println("Virando direita");
            turnRight();
            servoFront();
            for (int i = 0; i < count; i++) {
                followLine();
                delay(10);
            }
            return;
        }

        servoFront();
        wallDistance = getDistance();
        if (!isCloseToWall(wallDistance)) {
            Serial.println("Seguindo em frente");
            for (int i = 0; i < count; i++) {
                followLine();
                delay(10);
            }
            return;
        }

        servoLeft();
        wallDistance = getDistance();
        if (!isCloseToWall(wallDistance)) {
            Serial.println("Virando esquerda");
            turnLeft();
            servoFront();
            for (int i = 0; i < count; i++) {
                followLine();
                delay(10);
            }
            return;
        }

        turnAround();
        servoFront();
    }
}
