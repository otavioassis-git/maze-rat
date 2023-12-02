#define OUTPUT 1
#define INPUT 0

#define HIGH 1
#define LOW 0

// Pinos sensor ultrassonico
#define MISO_ECHO 19
#define EN_TRIG 16

float readings[3];
int sizeReadings;

// Setup do sensor ultrassonico
void setupUltrassonicSensor() {
    pinMode(MISO_ECHO, INPUT);
    pinMode(EN_TRIG, OUTPUT);
}

// Retorna a média das últimas 3 distâncias em centímetros
float getDistance() {
    digitalWrite(EN_TRIG, LOW);
    delayMicroseconds(10);
    digitalWrite(EN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(EN_TRIG, LOW);
    delayMicroseconds(10);

    unsigned long duration = pulseIn(MISO_ECHO, HIGH);
    float distance = duration / 58;

    if (sizeReadings == 3) {
        // retira a última leitura e coloca a nova
        readings[2] = readings[1];
        readings[1] = readings[0];
        readings[0] = distance;
    } else {
        // caso o vetor ainda não esteja cheio
        readings[sizeReadings] = distance;
        sizeReadings++;
    }

    // calcula e retorna a média das últimas leituras
    float sum = 0;
    for (int i = 0; i < sizeReadings; i++) {
        sum += readings[i];
    }

    return sum / sizeReadings;
}