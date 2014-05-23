#include <SPI.h>
#include "printf.h"

const int led_pin = 7;
const int threshold = 40;

void setup(void)
{

    Serial.begin(57600);
    printf_begin();
    pinMode(7, OUTPUT);

}

const int timespan = 2000;
const int stepsize = 20;
int counter = 0;

int measure1 = 0;
int measure2 = 0;

void loop(void) {
    if(counter >= timespan) {
        printf("counter >= timespan\n");

// Stuur een signaal (poort 8, 1334 Hz, 100ms)
        tone(8, 1334, 30);
        delay(timespan - 30);

// Reset de state van de machine
        counter = 0;
    }

    if(counter % 4 == 0) {
      printf("m1 %3d  --- m2 %3d\n", measure1, measure2);
    }

    if(measure1 - measure2 > threshold || measure2 - measure1 > threshold) { // ontvang signaal
        printf("Received signal - m1 %3d  --- m2 %3d\n", measure1, measure2);
        counter = timespan - (timespan-counter)/3;
        measure1 = 0;
        measure2 = 0;
    }
    else {
       measure1 = analogRead(A0);
       delay(stepsize);
       counter += stepsize;
       measure2 = analogRead(A0);
    }
}

