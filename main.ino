#include <string.h>

#include "lcd.h"
#include "optical.h"
#include "ultrasonic.h"

// pinos de sensores
#define SLC 36
#define SL 35
#define SR 39
#define SC 34
#define SRC 32

void setup() {
    setupLcd();
    setupOpticalSensors();
    setupUltrassonicSensor();
}

void loop() {
}