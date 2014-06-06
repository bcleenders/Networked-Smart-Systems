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

  Serial.println("hi!");

  while (! radio.available()) { 
    //Serial.print("."); 
  }
  radiotime = micros();
  uint8_t activeBeacon;
  radio.read( &activeBeacon, sizeof(uint8_t));
  Serial.print(activeBeacon);

  do {
    prevMeasure = currMeasure;
    currMeasure = analogRead(A0);
    //Serial.print(",");
  }
  while(currMeasure - prevMeasure < 100);

  audiotime = micros();
  float diff = audiotime - radiotime;

  Serial.print(" ");
  Serial.print((diff*0.3432)/10);
  Serial.println("cm difference");

  // wait a bit for the analog-to-digital converter 
  // to stabilize after the last reading:
  delay(10);
}

/* Processing code:
 
 // Graphing sketch
 
 
 // This program takes ASCII-encoded strings
 // from the serial port at 9600 baud and graphs them. It expects values in the
 // range 0 to 1023, followed by a newline, or newline and carriage return
 
 import processing.serial.*;
 
 Serial myPort;        // The serial port
 int xPos = 1;         // horizontal position of the graph
 
 void setup () {
 // set the window size:
 size(400, 300);        
 
 // List all the available serial ports
 println(Serial.list());
 // I know that the first port in the serial list on my mac
 // is always my  Arduino, so I open Serial.list()[0].
 // Open whatever port is the one you're using.
 myPort = new Serial(this, Serial.list()[0], 9600);
 // don't generate a serialEvent() unless you get a newline character:
 myPort.bufferUntil('\n');
 // set inital background:
 background(0);
 }
 void draw () {
 // everything happens in the serialEvent()
 }
 
 void serialEvent (Serial myPort) {
 // get the ASCII string:
 String inString = myPort.readStringUntil('\n');
 
 if (inString != null) {
 // trim off any whitespace:
 inString = trim(inString);
 // convert to an int and map to the screen height:
 float inByte = float(inString); 
 inByte = map(inByte, 0, 1023, 0, height);
 
 // draw the line:
 stroke(127,34,255);
 line(xPos, height, xPos, height - inByte);
 
 // at the edge of the screen, go back to the beginning:
 if (xPos >= width) {
 xPos = 0;
 background(0); 
 } 
 else {
 // increment the horizontal position:
 xPos++;
 }
 }
 }
 
 */

