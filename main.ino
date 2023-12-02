#include <string.h>

#include "lcd.h"
#include "optical.h"
#include "ultrasonic.h"

void setup() {
    setupLcd();
    setupOpticalSensors();
    setupUltrassonicSensor();
}

void loop() {
}