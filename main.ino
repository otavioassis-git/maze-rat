#include <string.h>

#include "lcd.h"
#include "servo.h"

// pinos de sensores
#define SLC 36
#define SL 35
#define SR 39
#define SC 34
#define SRC 32

void setup() {
  setupLcd();
  setupServo();
}

void loop() {
  
}