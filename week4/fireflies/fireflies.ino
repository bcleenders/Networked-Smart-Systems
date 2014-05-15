#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"


// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
// (MV:) Adapted to work with the configuration of the shield. Original: RF24 radio(9,10);

RF24 radio(3, 9);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipe = 0xF080C090F1LL;

const int led_pin = 13;

void setup(void)
{

  Serial.begin(57600);
  printf_begin();

  //
  // Setup and configure rf radio
  //

  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);

  // optionally, reduce the payload size.  seems to
  // improve reliability
  radio.setPayloadSize(8);

  //
  // Open pipes to other nodes for communication
  //
    radio.openWritingPipe(pipe);
    radio.openReadingPipe(0, pipe);


  //
  // Start listening
  //

  radio.startListening();

  //
  // Dump the configuration of the rf unit for debugging
  //

  radio.printDetails();
}

const int timespan = 2000;
const int stepsize = 100;
int counter = 0;

void loop(void)
{
// Slaap altijd iig de helft van de tijd
delay(timespan);

while(counter < timespan) {
    counter += stepsize;

    if(counter >= timespan) {
        // Stuur een signaal & laat LED knipperen
        radio.stopListening();
        radio.write("1", sizeof(int));
        digitalWrite(led_pin, HIGH); // laat LED branden
        delay(500);
        digitalWrite(led_pin, LOW); // zet LED uit
        // Reset de state van de machine
        counter = 0;
        radio.startListening();
    }

    if(radio.available()) {// ontvang signaal
        counter = timespan - (timespan-counter)/2;
    }

    delay(stepsize);
}
}
// vim:cin:ai:sts=2 sw=2 ft=cpp
