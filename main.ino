#include <ESP32Servo.h>
#include <string.h>

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

const int DATA[] = {4, 0, 2, 15};
#define DATA_SIZE 4

// Commands: [- - - RS D7 D6 D5 D4]
#define FUNCTION_SET 0x20     // 0010 0000 - 4 bits
#define DISPLAY_CONTROL 0x0C  // 0000 1100
#define CLEAR_DISPLAY 0x01    // 0000 0001
#define RETURN_HOME 0x02      // 0000 0010
#define ENTRY_MODE_SET 0x06   // 0000 0110
#define SHIFT_CURSOR_RIGHT 0b000010100
#define SHIFT_DISPLAY_RIGHT 0b000011100
#define SHIFT_DISPLAY_LEFT 0b000011000

// States
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define DISPLAY_FUNCTION 0x08

// pinos do lcd
#define RS 17
#define EN_TRIG 16
#define RL_LCD 12

// pinos motor
#define E_CH1 33
#define CHA_M1 25
#define E_CH2 26
#define CHA_M2 27
#define CS_Sensors 5

// config pwm
#define PWM1_Ch 0
#define PWM2_Ch 1
#define PWM1_Res 8
#define PWM1_Freq 1000

#define MAX_OPTICAL 2400.0
#define MAX_SPEED 32.0
#define MIN_SPEED 10.0

/////////////////////////////////////////////////// SERVO

Servo myServo;

void setupServo() {
    myServo.attach(SERVO_PIN);
}

void servoMoveAngle(int angle) {
    myServo.write(angle);
}

void servoFront() {
    myServo.write(FRONT_ANGLE);
}

void servoLeft() {
    myServo.write(LEFT_ANGLE);
}

void servoRight() {
    myServo.write(RIGHT_ANGLE);
}

/////////////////////////////////////////////////// FIM SERVO

/////////////////////////////////////////////////// ULTRASSOM
float readingsUltrasonic[3];
int sizeReadingsUltrasonic = 0;

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

    if (sizeReadingsUltrasonic == 3) {
        // retira a última leitura e coloca a nova
        readingsUltrasonic[2] = readingsUltrasonic[1];
        readingsUltrasonic[1] = readingsUltrasonic[0];
        readingsUltrasonic[0] = distance;
    } else {
        // caso o vetor ainda não esteja cheio insere na última posição
        readingsUltrasonic[sizeReadingsUltrasonic] = distance;
        sizeReadingsUltrasonic++;
    }

    // calcula e retorna a média das últimas leituras
    float sum = 0;
    for (int i = 0; i < sizeReadingsUltrasonic; i++) {
        sum += readingsUltrasonic[i];
    }

    return sum / sizeReadingsUltrasonic;
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

////////////////////////////////////////////////////// LCD
void setupLcd() {
    pinMode(RS, OUTPUT);
    pinMode(EN_TRIG, OUTPUT);
    for (int i = 0; i < DATA_SIZE; i++) {
        pinMode(DATA[i], OUTPUT);
    }

    initLCD();

    // Always good to clear and return home
    write8bits(CLEAR_DISPLAY);
    write8bits(RETURN_HOME);
}

void pulseEnable() {
    // Making sure the pin is LOW at first
    digitalWrite(EN_TRIG, LOW);
    delayMicroseconds(1);

    // Pulse the Enable pin
    digitalWrite(EN_TRIG, HIGH);
    delayMicroseconds(1);
    digitalWrite(EN_TRIG, LOW);
    delayMicroseconds(100);
}

/* ------------------------------------------------------
  Initializes the display as in Figure 24 of the
    HD44780U datasheet requests
*/
void initLCD() {
    // Waiting at first
    delay(40);

    // Serial.println("Function set 4 bits 0b0010.");
    digitalWrite(RS, LOW);

    // Function set the interface with 4 bits
    write4bits(FUNCTION_SET >> 4);
    delayMicroseconds(4500);  // A little more than 4.1 ms

    // Now, we set:
    // - Number of lines in the display (2 lines: 16x2)
    // - Size of the pixel matrix (5x8)
    // RS remains 0 (only is 1 when writing)
    // Serial.println("Function set 4 bits 0b0010 1000.");
    write8bits(FUNCTION_SET | DISPLAY_FUNCTION);

    // Display OFF
    // Serial.println("Display ON/OFF control 0b0000 1100.");
    write8bits(DISPLAY_CONTROL);

    // Entry mode set
    // Serial.println("Entry mode set 0b0000 0110.");
    write8bits(ENTRY_MODE_SET);

    // Clearing and returning home
    write8bits(CLEAR_DISPLAY);
    write8bits(RETURN_HOME);
    delay(2);  // 1.52ms delay needed for the Return Home command

    // Now you're free to use the display
}

void clearDisplay() {
    write8bits(CLEAR_DISPLAY);
}

/* ------------------------------------------------------
  Actually sends the commands to the display
*/
void write4bits(int value) {
    for (int i = 0; i < 4; i++) {
        // Only the value corresponding to the bit of interest
        digitalWrite(DATA[i], (value >> (3 - i)) & 0x1);
    }
    pulseEnable();

    for (int i = 0; i < 4; i++) {
        // Only the value corresponding to the bit of interest
        digitalWrite(DATA[i], 0);
    }
}

/* ------------------------------------------------------
  Writes first half of data, than second half
*/
void write8bits(int value) {
    // Sends first half of the data (upper part):
    write4bits(value >> 4);
    // Sends last half of the data (lower part):
    write4bits(value);
}

void writeAbove(const char *value) {
    write8bits(RETURN_HOME);
    delay(2);  // 1.52ms delay needed for the Return Home command

    char c;
    for (int i = 0; i < strlen(value); i++) {
        c = value[i];

        digitalWrite(RS, HIGH);
        write8bits(c);
        digitalWrite(RS, LOW);
    }
}

void writeBelow(const char *value) {
    write8bits(RETURN_HOME);
    delay(2);  // 1.52ms delay needed for the Return Home command
    for (int i = 0; i < 40; i++) {
        write8bits(SHIFT_CURSOR_RIGHT);
    }
    delay(2);
    char c;
    for (int i = 0; i < strlen(value); i++) {
        c = value[i];

        digitalWrite(RS, HIGH);
        write8bits(c);
        digitalWrite(RS, LOW);
    }
}
////////////////////////////////////////////////////////////// FIM LCD

////////////////////////////////////////////////////// MOTORES
void setupMotors() {
    pinMode(E_CH1, OUTPUT);
    pinMode(E_CH2, OUTPUT);

    ledcAttachPin(CHA_M1, PWM1_Ch);
    ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);

    ledcAttachPin(CHA_M2, PWM2_Ch);
    ledcSetup(PWM2_Ch, PWM1_Freq, PWM1_Res);
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
    ledcWrite(PWM1_Ch, pwm_right);
    ledcWrite(PWM2_Ch, pwm_left);
}
////////////////////////////////////////////////////// FIM MOTORES

////////////////////////////////////////////////////// SEGUE LINHA
float cSensor;
float rSensor;
float lSensor;
float rcSensor;
float lcSensor;
float maxOptical[5] = {2000.0, 2000.0, 2000.0, 2000.0, 2000.0};

int inCrossingTransition = 0;

/* Primeira opção:
void followLine() {
    // lendo os sensores e atualizando o maxOptical caso necessário
    cSensor = float(getSCReading());
    if (cSensor > maxOptical[0]) maxOptical[0] = cSensor;

    rSensor = float(getSRReading());
    if (rSensor > maxOptical[1]) maxOptical[1] = rSensor;

    lSensor = float(getSLReading());
    if (lSensor > maxOptical[2]) maxOptical[2] = lSensor;

    // calculando as taxas
    float scRate = cSensor / maxOptical[0];
    float srRate = (1 - (rSensor / maxOptical[1]));
    float slRate = (1 - (lSensor / maxOptical[2]));

    // calculando a velocidade a ser somada
    float rSpeed = scRate * srRate * MAX_SPEED;
    float lSpeed = scRate * slRate * MAX_SPEED;

    // calculando o valor absoluto da velocidade para cada motor
    float pwm_sr = 128 - rSpeed;
    float pwm_sl = 128 + lSpeed;

    enableMotors();
    pwmMotors(int(pwm_sr), int(pwm_sl));

    clearDisplay();
    char szRSensor[10];
    char szLSensor[10];
    dtostrf(pwm_sr, 4, 3, szRSensor);
    dtostrf(pwm_sl, 4, 3, szLSensor);
    writeAbove(szRSensor);
    writeBelow(szLSensor);
}
*/

/* Primeira opção com detecção de cruzamento*/
void followLine() {
    // lendo os sensores e atualizando o maxOptical caso necessário
    cSensor = float(getSCReading());
    if (cSensor > maxOptical[0]) maxOptical[0] = cSensor;

    rSensor = float(getSRReading());
    if (rSensor > maxOptical[1]) maxOptical[1] = rSensor;

    lSensor = float(getSLReading());
    if (lSensor > maxOptical[2]) maxOptical[2] = lSensor;

    float pwm_sr = 0.0;
    float pwm_sl = 0.0;
    float rSpeed = 0.0;
    float lSpeed = 0.0;

    // detectando cruzamento
    if (cSensor > 1000 && rSensor > 1000 && lSensor > 1000 || inCrossingTransition) {
        rcSensor = float(getSRCReading());
        lcSensor = float(getSLCReading());

        // detecta se já chegou no cruzamento (para testes por enquanto vou fazer ele só parar (acho que esse lógica nem tinha que estar aqui, só estou colocando para testar mesmo))
        //  coloquei um || para caso ele não faça alguma curva e só detecte um lado primeiro (lembrar que a gente vai ter que ver o mult p/ as rodas girarem na msm vel.)
        if (rcSensor > 1000 || lcSensor > 1000) {
            pwm_sr = 0.0;
            pwm_sl = 0.0;
        } else {
            // caso precise colocar um multiplicador para as duas rodas girarem na mesma velocidade
            rSpeed = 1.0 * MIN_SPEED;
            lSpeed = 1.0 * MIN_SPEED;

            pwm_sr = 128 - rSpeed;
            pwm_sl = 128 + lSpeed;
        }

        inCrossingTransition = 1;
    } else {
        // calculando as taxas
        float scRate = cSensor / maxOptical[0];
        float srRate = (1 - (rSensor / maxOptical[1]));
        float slRate = (1 - (lSensor / maxOptical[2]));

        // calculando a velocidade a ser somada
        rSpeed = scRate * srRate * MAX_SPEED;
        lSpeed = scRate * slRate * MAX_SPEED;

        // calculando o valor absoluto da velocidade para cada motor
        pwm_sr = 128 - rSpeed;
        pwm_sl = 128 + lSpeed;
    }

    enableMotors();
    pwmMotors(int(pwm_sr), int(pwm_sl));

    clearDisplay();
    char szRSensor[10];
    char szLSensor[10];
    dtostrf(pwm_sr, 4, 3, szRSensor);
    dtostrf(pwm_sl, 4, 3, szLSensor);
    writeAbove(szRSensor);
    writeBelow(szLSensor);
}

void turnRight() {
    enableMotors();
    pwmMotors(20, 20);

    while (float scReading = getSCReading()) {
        if (scReading < 300) {
            break;
        }
    }

    while (float scReading = getSCReading()) {
        if (scReading > 1000) {
            break;
        }
    }

    disableMotors();
}

void turnLeft() {
    enableMotors();
    pwmMotors(236, 236);

    while (float scReading = getSCReading()) {
        if (scReading < 300) {
            break;
        }
    }

    while (float scReading = getSCReading()) {
        if (scReading > 1000) {
            break;
        }
    }

    disableMotors();
}

void turnAround() {
    turnRight();
    turnRight();
}
////////////////////////////////////////////////////// SEGUE LINHA FIM

///////////////////////////////////////////////////// UTILS

byte isCloseToWall(float distance) {
    if (distance > 30)
        return false;
    return true;
}

#define CROSSING_OFFSET 1500
byte isOverCrossing() {
    rcSensor = getSRCReading();
    lcSensor = getSLCReading();
    if (rcSensor > CROSSING_OFFSET && lcSensor > CROSSING_OFFSET)
        return true;
    return false;
}

///////////////////////////////////////////////////// FIM UTILS

///////////////////////////////////////////// GLOBALS

byte bCloseToWall = false;

float wallDistance = 400;
char szWallDistance[10];

///////////////////////////////////////////// FIM GLOBALS

void setup() {
    setupOpticalSensors();
    setupUltrassonicSensor();
    setupLcd();
    setupServo();
    setupMotors();
    enableMotors();
    Serial.begin(115200);  // ESP32
    servoFront();
}

void loop() {
    // followLine();
    // if (isOverCrossing())
    // {
    //     disableMotors();

    //     servoRight();
    //     wallDistance = getDistance();
    //     dtostrf(wallDistance, 3, 2, szWallDistance);

    //     clearDisplay();
    //     writeAbove("Distancia parede");
    //     writeBelow(szWallDistance);
    //     if (!isCloseToWall(wallDistance))
    //     {
    //         turnRight();
    //         servoFront();
    //         return;
    //     }

    //     servoFront();
    //     wallDistance = getDistance();
    //     dtostrf(wallDistance, 3, 2, szWallDistance);
    //     // clearDisplay();
    //     //
    //     // writeAbove(szWallDistance);
    //     if (!isCloseToWall(wallDistance))
    //     {
    //         return;
    //     }

    //     servoLeft();
    //     wallDistance = getDistance();
    //     dtostrf(wallDistance, 3, 2, szWallDistance);
    //     // clearDisplay();
    //     //
    //     // writeAbove(szWallDistance);
    //     if (!isCloseToWall(wallDistance))
    //     {
    //         turnLeft();
    //         servoFront();
    //         return;
    //     }

    //     turnAround();
    // }
    followLine();

    // delay(500);
}