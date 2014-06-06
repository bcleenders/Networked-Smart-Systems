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
  radio.printDetails();
}

unsigned long radiotime;
unsigned long audiotime;
int prevMeasure = 0;
int currMeasure = 0;

void loop() {
  while (! radio.available()) { }

  radiotime = micros();
  uint8_t activeBeacon;
  radio.read( &activeBeacon, sizeof(uint8_t));
  Serial.print(activeBeacon);

  do {
    prevMeasure = currMeasure;
    currMeasure = analogRead(A0);
    audiotime = micros();
  }
  while((currMeasure - prevMeasure < 100) && (audiotime - radiotime < 40000));

  float diff = audiotime - radiotime;

  Serial.print(" ");
  Serial.print((diff*0.3432)/10);
  Serial.println("cm difference");

  // wait a bit for the analog-to-digital converter 
  // to stabilize after the last reading:
  delay(10);
}
