#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

RF24 radio(3, 9);

void setup() {
  // initialize the serial communication:
  Serial.begin(9600);
  printf_begin();

  // Setup and configure rf radio
  radio.begin();
  radio.setRetries(0,0);

  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(76);
  radio.setPayloadSize(1);
  radio.openReadingPipe(1, 0xdeadbeefa1LL);
  radio.openWritingPipe(0xdeadbeefa1LL);
  radio.startListening();
  radio.setAutoAck(false);
  //radio.printDetails();
}

unsigned long radiotime;
unsigned long audiotime;
unsigned long timelimit = 50000LL;
int prevMeasure = 0;
int currMeasure = 0;
uint8_t activeBeacon;

void loop() {
  while(radio.available()) { 
    radio.read(&activeBeacon, sizeof(uint8_t)); 
  }
  while (! radio.available()) { 
  }

  radiotime = micros();
  radio.read( &activeBeacon, sizeof(uint8_t));

  currMeasure = analogRead(A0);

  while(analogRead(A0) < 50) {
    audiotime = micros();
    if(audiotime - radiotime > timelimit) {
      Serial.println("timeout");
      return; 
    }
  }
  

  float diff = audiotime - radiotime;

  Serial.print(activeBeacon);
  Serial.print(" ");
  Serial.print((diff*0.3432)/1000);
  Serial.println("m difference");
}


