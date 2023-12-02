#define OUTPUT 1
#define INPUT 0

#define HIGH 1
#define LOW 0

// Pinos sensor óptico
#define SC 34
#define SL 35
#define SR 39
#define SLC 36
#define SRC 32
#define CS_Sensors 29

int readings[5][5];
int sizeReadings[5] = {0, 0, 0, 0, 0};

void setupOpticalSensors() {
    pinmode(SC, INPUT);
    pinmode(SL, INPUT);
    pinmode(SR, INPUT);
    pinmode(SLC, INPUT);
    pinmode(SRC, INPUT);
    pinmode(CS_Sensors, OUTPUT);
}

void initReading(int pin) {
    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin, LOW);
}

float readingReturn(int pin) {
    int reading;

    switch (pin) {
        case SC:
            return calculateMean(0, SC);

        case SL:
            return calculateMean(1, SL);

        case SR:
            return calculateMean(2, SR);

        case SLC:
            return calculateMean(3, SLC);

        case SRC:
            return calculateMean(4, SRC);
    }
}

float calculateMean(int id, int pin) {
    int reading = digitalRead(pin);

    // atualiza o vetor com as leituras
    if (sizeReadings[id] == 5) {
        // retira a última leitura e coloca a nova
        for (int i = 4; i > 0; i++) {
            readings[id][i] = readings[id][i - 1];
        }
        readings[id][0] = reading;
    } else {
        // caso o vetor ainda não esteja cheio insere na última posição
        readings[id][sizeReadings[id]] = reading;
        sizeReadings[id]++;
    }

    // calcula a média das últimas leituras
    float sum = 0;
    for (int i = 0; i < sizeReadings[id]; i++) {
        sum += readings[id][i];
    }

    return sum / sizeReadings[id];
}

// Retorna a leitura do sensor óptio central
float getSCReading() {
    initReading(SC);

    return readingReturn(SC);
}

// Retorna a leitura do sensor óptio esquerdo
float getSLReading() {
    initReading(SL);

    return readingReturn(SL);
}

// Retorna a leitura do sensor óptio direito
float getSRReading() {
    initReading(SR);

    return readingReturn(SR);
}

// Retorna a leitura do sensor óptio esquerdo central
float getSLCReading() {
    initReading(SLC);

    return readingReturn(SLC);
}

// Retorna a leitura do sensor óptio direito central
float getSRCReading() {
    initReading(SRC);

    return readingReturn(SRC);
}
