#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

RF24 radio(3, 9);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipe = 0xF080C090F1LL;

const int led_pin = 7;

void setup(void)
{

  Serial.begin(57600);
  printf_begin();
  pinMode(7, OUTPUT);
  
  //
  // Setup and configure rf radio
  //

  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(1,1);

  // optionally, reduce the payload size.  seems to
  // improve reliability
  radio.setPayloadSize(8);

  radio.openWritingPipe(pipe);
  radio.openReadingPipe(0, pipe);

  radio.startListening();

  radio.printDetails();
}

const int timespan = 2000;
const int stepsize = 100;
int counter = 0;

void loop(void) {
  // Slaap altijd iig de helft van de tijd

    if(counter >= timespan) {
      printf("counter >= timespan\n");
    // Stuur een signaal & laat LED knipperen
    radio.stopListening();
    radio.write("1", sizeof(int));
    digitalWrite(led_pin, HIGH); // laat LED branden
    delay(1000);
    digitalWrite(led_pin, LOW); // zet LED uit
    
    // Reset de state van de machine
    counter = 0;
    // Slaap altijd helft van de tijd (refractory period)
    delay(timespan);
    radio.startListening();
  }

  if(radio.available()) { // ontvang signaal
  printf("Radio available");
    counter = timespan + stepsize;  //- (timespan-counter)/2;
  }
  else {
      printf("Not radio available");
    delay(stepsize);
    counter += stepsize;
  }
}

