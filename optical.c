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

int readingReturn(int pin) {
    return digitalRead(pin);
}

// Retorna a leitura do sensor óptio central
int getSCReading() {
    initReading(SC);

    return readingReturn(SC);
}

// Retorna a leitura do sensor óptio esquerdo
int getSLReading() {
    initReading(SL);

    return readingReturn(SL);
}

// Retorna a leitura do sensor óptio direito
int getSRReading() {
    initReading(SR);

    return readingReturn(SR);
}

// Retorna a leitura do sensor óptio esquerdo central
int getSLCReading() {
    initReading(SLC);

    return readingReturn(SLC);
}

// Retorna a leitura do sensor óptio direito central
int getSRCReading() {
    initReading(SRC);

    return readingReturn(SRC);
}
