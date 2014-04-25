// Graphing sketch


// This program takes ASCII-encoded strings
// from the serial port at 9600 baud and graphs them. It expects values in the
// range 0 to 1023, followed by a newline, or newline and carriage return

// Created 20 Apr 2005
// Updated 18 Jan 2008
// by Tom Igoe
// This example code is in the public domain.

import processing.serial.*;

Serial myPort;        // The serial port
int xPos = 1;         // horizontal position of the graph

void setup () {
  // set the window size:
  size(1200, 600);


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
  System.out.println("Test");  

  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');

  if (inString != null) {
    // inString heeft vorm "ID waarde"
    int id = Integer.parseInt(inString.substring(0, 1));
    int value = Integer.parseInt(inString.substring(2)); 

    int yPos = height;
    if (id < 3) { 
      yPos = height / 2;
    }
    int xOffset = (width/3) * (id % 3);

    stroke((255 - 30 * id), (50 + 30*id), 255);

    float inByte = float(value);
    line(xOffset + xPos, yPos, xOffset + xPos, yPos - inByte);

    if (id == 1) {
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
}

