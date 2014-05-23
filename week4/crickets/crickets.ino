#include <SPI.h>
#include "printf.h"

const int led_pin = 7;
const float thresholdfactor = 3.0;

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
float avgdiff = 0.0;

void loop(void) {
    if(counter >= timespan) {
        printf("counter >= timespan\n");

// Stuur een signaal (poort 8, 1334 Hz, 100ms)
        tone(8, 3000, stepsize);
        delay(timespan - stepsize);

// Reset de state van de machine
        counter = 0;
    }

    float diff = abs(measure1-measure2);
    float difffloat = (float) diff;
    avgdiff = ((10.0 * difffloat) + 90.0*avgdiff)/100.0; // Slowly adjust avgdiff

    if(difffloat > (avgdiff*thresholdfactor)) { // ontvang signaal

        Serial.print("Received signal, m1=");
        Serial.print(measure1);
        Serial.print(", m2=");
        Serial.print(measure2);
        Serial.print(", diff=");
        Serial.print(diff);
        Serial.print(", avgdiff=");
        Serial.print(avgdiff);
        Serial.print("\n");
      
        counter = timespan - (timespan-counter)/3;
        measure1 = 0;
        measure2 = 0;
    }
     measure1 = analogRead(A0);
     delay(stepsize);
     counter += stepsize;
     measure2 = analogRead(A0);
}

