#include <SPI.h>
#include "printf.h"

void setup(void)
{
  Serial.begin(57600);
}

void loop(void) {
  tone(8, 160, 2000);
  delay(3000);
}

