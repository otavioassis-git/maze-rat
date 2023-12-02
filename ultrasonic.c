#define OUTPUT 1
#define INPUT 0

#define HIGH 1
#define LOW 0

// Pinos sensor ultrassonico
#define MISO_ECHO 19
#define EN_TRIG 16

void setupUltrassonicSensor() {
    pinMode(MISO_ECHO, INPUT);
    pinMode(EN_TRIG, OUTPUT);
}

// Pega a distância em centímetros
float getDistance() {
    digitalWrite(EN_TRIG, LOW);
    delayMicroseconds(10);
    digitalWrite(EN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(EN_TRIG, LOW);
    delayMicroseconds(10);

    unsigned long duration = pulseIn(MISO_ECHO, HIGH);
    return duration / 58;
}